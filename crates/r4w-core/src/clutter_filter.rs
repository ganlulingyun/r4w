//! Clutter Filter -- Clutter Cancellation for Radar Signal Processing
//!
//! Removes stationary or slow-moving clutter returns from pulsed radar data,
//! allowing moving targets to be detected against strong ground, sea, or weather
//! clutter. Implements Moving Target Indicator (MTI) cancellers (single and
//! double), programmable FIR Doppler filters along slow time, clutter notch
//! design, and clutter map subtraction.
//!
//! Complex IQ samples are represented as `(f64, f64)` tuples where the first
//! element is the real (I) component and the second is the imaginary (Q)
//! component. Pulse data is organized as `Vec<(f64, f64)>` per range profile,
//! with each element corresponding to one range bin.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::clutter_filter::{MtiFilter, DoubleCanceller, DopplerFilter};
//!
//! // Simple 2-pulse MTI canceller removes DC (stationary) clutter
//! let mut mti = MtiFilter::new();
//! let clutter_pulse = vec![(1.0, 0.0); 8]; // constant across pulses
//! let output = mti.process(&[clutter_pulse.clone(), clutter_pulse.clone()]);
//! assert_eq!(output.len(), 1);
//! // Each range bin should be near zero (clutter cancelled)
//! for (re, im) in &output[0] {
//!     assert!(re.abs() < 1e-10 && im.abs() < 1e-10);
//! }
//!
//! // A moving target with pulse-to-pulse phase change passes through
//! let mut mti2 = MtiFilter::new();
//! let p0 = vec![(1.0, 0.0)];
//! let p1 = vec![(-1.0, 0.0)]; // 180-degree phase shift
//! let out = mti2.process(&[p0, p1]);
//! assert!((out[0][0].0 - (-2.0)).abs() < 1e-10);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (inline, no external crate)
// ---------------------------------------------------------------------------

#[inline]
fn cx_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn cx_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn cx_scale(s: f64, c: (f64, f64)) -> (f64, f64) {
    (s * c.0, s * c.1)
}

#[inline]
fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn cx_mag_sq(c: (f64, f64)) -> f64 {
    c.0 * c.0 + c.1 * c.1
}

// ---------------------------------------------------------------------------
// MtiFilter -- 2-pulse canceller
// ---------------------------------------------------------------------------

/// Moving Target Indicator (MTI) filter -- simple 2-pulse canceller.
///
/// Computes `y[n] = x[n] - x[n-1]` for each range bin across successive
/// pulses, placing a null at zero Doppler (DC) to reject stationary clutter.
#[derive(Debug, Clone)]
pub struct MtiFilter {
    /// Previous pulse for streaming mode.
    prev_pulse: Option<Vec<(f64, f64)>>,
}

impl MtiFilter {
    /// Create a new MTI filter with no prior state.
    pub fn new() -> Self {
        Self { prev_pulse: None }
    }

    /// Process a batch of pulses, returning cancelled output.
    ///
    /// Input: `pulses[pulse_index][range_bin]` -- at least 2 pulses required.
    /// Output: `N-1` output pulses where `out[n] = pulses[n+1] - pulses[n]`
    /// per range bin.
    ///
    /// The internal state (`prev_pulse`) is updated to the last pulse in the
    /// batch so that subsequent calls to [`process`] or [`single_pulse`]
    /// continue seamlessly.
    pub fn process(&mut self, pulses: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        if pulses.len() < 2 {
            if let Some(p) = pulses.first() {
                self.prev_pulse = Some(p.clone());
            }
            return Vec::new();
        }

        let mut output = Vec::with_capacity(pulses.len() - 1);
        for i in 1..pulses.len() {
            let prev = &pulses[i - 1];
            let curr = &pulses[i];
            let num_bins = prev.len().min(curr.len());
            let mut cancelled = Vec::with_capacity(num_bins);
            for b in 0..num_bins {
                cancelled.push(cx_sub(curr[b], prev[b]));
            }
            output.push(cancelled);
        }
        self.prev_pulse = Some(pulses.last().unwrap().clone());
        output
    }

    /// Process a single pulse in streaming fashion.
    ///
    /// Returns `None` on the first call (no previous pulse available) and
    /// `Some(cancelled)` on subsequent calls, where each range bin is
    /// `current[bin] - previous[bin]`.
    pub fn single_pulse(&mut self, pulse: &[(f64, f64)]) -> Option<Vec<(f64, f64)>> {
        let result = self.prev_pulse.as_ref().map(|prev| {
            let num_bins = prev.len().min(pulse.len());
            (0..num_bins)
                .map(|b| cx_sub(pulse[b], prev[b]))
                .collect()
        });
        self.prev_pulse = Some(pulse.to_vec());
        result
    }
}

// ---------------------------------------------------------------------------
// DoubleCanceller -- 3-pulse canceller
// ---------------------------------------------------------------------------

/// Double canceller -- 3-pulse clutter rejection filter.
///
/// Computes `y[n] = x[n] - 2*x[n-1] + x[n-2]` for each range bin, providing
/// a second-order null at zero Doppler for deeper clutter suppression than a
/// single MTI canceller.
#[derive(Debug, Clone)]
pub struct DoubleCanceller {
    /// Two most recent pulses: `[n-2, n-1]`.
    history: Vec<Vec<(f64, f64)>>,
}

impl DoubleCanceller {
    /// Create a new double canceller with no prior state.
    pub fn new() -> Self {
        Self {
            history: Vec::new(),
        }
    }

    /// Process a batch of pulses, returning cancelled output.
    ///
    /// Input: `pulses[pulse_index][range_bin]` -- at least 3 pulses required.
    /// Output: `N-2` output pulses where
    /// `out[n] = pulses[n+2] - 2*pulses[n+1] + pulses[n]` per range bin.
    pub fn process(&mut self, pulses: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        if pulses.len() < 3 {
            // Buffer whatever we have
            for p in pulses {
                self.history.push(p.clone());
                if self.history.len() > 2 {
                    self.history.remove(0);
                }
            }
            return Vec::new();
        }

        let mut output = Vec::with_capacity(pulses.len() - 2);
        for i in 2..pulses.len() {
            let p0 = &pulses[i - 2];
            let p1 = &pulses[i - 1];
            let p2 = &pulses[i];
            let num_bins = p0.len().min(p1.len()).min(p2.len());
            let mut cancelled = Vec::with_capacity(num_bins);
            for b in 0..num_bins {
                // y = x[n] - 2*x[n-1] + x[n-2]
                let val = cx_add(cx_sub(p2[b], cx_scale(2.0, p1[b])), p0[b]);
                cancelled.push(val);
            }
            output.push(cancelled);
        }
        // Update history with last two pulses
        self.history.clear();
        let len = pulses.len();
        self.history.push(pulses[len - 2].clone());
        self.history.push(pulses[len - 1].clone());
        output
    }
}

// ---------------------------------------------------------------------------
// DopplerFilter -- FIR along slow time
// ---------------------------------------------------------------------------

/// Programmable FIR filter applied along slow time (pulse dimension) for
/// clutter rejection.
///
/// For each range bin, the filter convolves the slow-time sequence with the
/// supplied FIR coefficients, enabling arbitrary clutter notch shapes and
/// bandpass Doppler selection.
#[derive(Debug, Clone)]
pub struct DopplerFilter {
    /// FIR tap coefficients applied across pulses.
    coefficients: Vec<f64>,
}

impl DopplerFilter {
    /// Create a new Doppler filter with the given FIR coefficients.
    ///
    /// The number of taps determines the minimum number of pulses required.
    pub fn new(coefficients: Vec<f64>) -> Self {
        Self { coefficients }
    }

    /// Apply the FIR filter across the pulse dimension for each range bin.
    ///
    /// Input: `pulses[pulse_index][range_bin]`.
    /// Output: `(N - taps + 1)` output pulses (valid convolution region).
    ///
    /// For each output pulse index `n` and range bin `b`:
    /// ```text
    /// out[n][b] = sum_k( coefficients[k] * pulses[n + k][b] )
    /// ```
    pub fn process(&self, pulses: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        let taps = self.coefficients.len();
        if pulses.len() < taps || taps == 0 {
            return Vec::new();
        }

        let num_output = pulses.len() - taps + 1;
        let num_bins = pulses.iter().map(|p| p.len()).min().unwrap_or(0);

        let mut output = Vec::with_capacity(num_output);
        for n in 0..num_output {
            let mut out_pulse = Vec::with_capacity(num_bins);
            for b in 0..num_bins {
                let mut acc = (0.0, 0.0);
                for (k, &c) in self.coefficients.iter().enumerate() {
                    acc = cx_add(acc, cx_scale(c, pulses[n + k][b]));
                }
                out_pulse.push(acc);
            }
            output.push(out_pulse);
        }
        output
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Theoretical MTI clutter improvement factor in dB.
///
/// For an ideal single canceller with `num_pulses` coherently integrated
/// pulses, the improvement factor is approximately `3 * num_pulses` dB
/// against Gaussian clutter with zero mean Doppler. For a single 2-pulse
/// canceller the improvement is approximately 3 dB per pulse pair.
///
/// Returns the improvement factor in dB (linear scale: `10 * log10(value)`).
pub fn mti_improvement_factor(num_pulses: usize) -> f64 {
    if num_pulses < 2 {
        return 0.0;
    }
    // For a single canceller with N pulses, the clutter improvement factor
    // is approximately N (linear), or 10*log10(N) dB, reflecting the
    // coherent integration gain against white clutter residue.
    10.0 * (num_pulses as f64).log10()
}

/// Design a highpass FIR filter for clutter notch at DC / zero Doppler.
///
/// Creates FIR coefficients that reject frequencies within `notch_width` of DC
/// (normalized to the PRF, so `notch_width` in `[0, 0.5]`), passing all other
/// Doppler frequencies.
///
/// Uses windowed-sinc design: an ideal highpass impulse response windowed with
/// a Hamming window.
///
/// # Arguments
///
/// * `notch_width` -- Half-width of the notch in normalized frequency (0..0.5).
/// * `num_taps` -- Number of FIR filter taps (odd recommended for symmetry).
///
/// # Returns
///
/// Vector of `num_taps` FIR coefficients.
pub fn design_clutter_notch(notch_width: f64, num_taps: usize) -> Vec<f64> {
    if num_taps == 0 {
        return Vec::new();
    }
    if num_taps == 1 {
        return vec![1.0];
    }

    let fc = notch_width.clamp(0.0, 0.5);
    let m = num_taps as f64;
    let mid = (num_taps - 1) as f64 / 2.0;

    let mut h: Vec<f64> = (0..num_taps)
        .map(|i| {
            let n = i as f64;
            // Hamming window
            let w = 0.54 - 0.46 * (2.0 * PI * n / (m - 1.0)).cos();

            if (n - mid).abs() < 1e-12 {
                // Limit value: highpass at cutoff fc => 1 - 2*fc
                (1.0 - 2.0 * fc) * w
            } else {
                // Ideal highpass impulse response: delta[n-mid] - lowpass
                // h_hp[n] = -sin(2*pi*fc*(n-mid)) / (pi*(n-mid))
                let x = n - mid;
                let sinc_val = (2.0 * PI * fc * x).sin() / (PI * x);
                -sinc_val * w
            }
        })
        .collect();

    // Normalize so that the gain at Nyquist (f = 0.5) is unity
    let nyquist_gain: f64 = h
        .iter()
        .enumerate()
        .map(|(i, &c)| c * if i % 2 == 0 { 1.0 } else { -1.0 })
        .sum::<f64>()
        .abs();
    if nyquist_gain > 1e-15 {
        for c in &mut h {
            *c /= nyquist_gain;
        }
    }

    h
}

/// Apply clutter map subtraction to a power signal.
///
/// Implements exponentially weighted moving average (EWMA) clutter map
/// subtraction, commonly used for CFAR-like adaptive clutter removal.
///
/// For each range bin:
/// ```text
/// output[i] = signal[i] - clutter_map[i]
/// ```
///
/// Negative values are clamped to zero (clutter residue should not produce
/// negative power).
///
/// # Arguments
///
/// * `signal` -- Input power profile (linear scale).
/// * `clutter_map` -- Estimated clutter power per range bin.
/// * `alpha` -- Blending factor in `[0, 1]`. Higher values weight the current
///   signal more heavily; `alpha = 1.0` means no map subtraction.
///
/// # Returns
///
/// Clutter-subtracted power profile, clamped to non-negative values.
pub fn apply_clutter_map(signal: &[f64], clutter_map: &[f64], alpha: f64) -> Vec<f64> {
    let len = signal.len().min(clutter_map.len());
    let alpha_c = alpha.clamp(0.0, 1.0);

    (0..len)
        .map(|i| {
            let subtracted = signal[i] - (1.0 - alpha_c) * clutter_map[i];
            subtracted.max(0.0)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mti_canceller() {
        // Constant clutter should be fully cancelled
        let mut mti = MtiFilter::new();
        let clutter = vec![(5.0, 3.0); 16];
        let output = mti.process(&[clutter.clone(), clutter.clone(), clutter.clone()]);
        assert_eq!(output.len(), 2);
        for pulse in &output {
            for &(re, im) in pulse {
                assert!(re.abs() < 1e-10, "real not cancelled: {re}");
                assert!(im.abs() < 1e-10, "imag not cancelled: {im}");
            }
        }
    }

    #[test]
    fn test_double_canceller() {
        // Constant clutter: y = x - 2x + x = 0
        let mut dc = DoubleCanceller::new();
        let clutter = vec![(1.0, 0.5); 8];
        let output = dc.process(&[
            clutter.clone(),
            clutter.clone(),
            clutter.clone(),
            clutter.clone(),
        ]);
        assert_eq!(output.len(), 2);
        for pulse in &output {
            for &(re, im) in pulse {
                assert!(re.abs() < 1e-10, "double canceller real residue: {re}");
                assert!(im.abs() < 1e-10, "double canceller imag residue: {im}");
            }
        }

        // Linearly increasing clutter (amplitude ramp): also cancelled by double
        let mut dc2 = DoubleCanceller::new();
        let p0 = vec![(1.0, 0.0)];
        let p1 = vec![(2.0, 0.0)];
        let p2 = vec![(3.0, 0.0)];
        let output2 = dc2.process(&[p0, p1, p2]);
        // y = 3 - 2*2 + 1 = 0
        assert_eq!(output2.len(), 1);
        assert!(output2[0][0].0.abs() < 1e-10, "linear ramp not cancelled");
    }

    #[test]
    fn test_dc_rejection() {
        // MTI filter frequency response should be zero at DC
        // We verify by sending a pure DC signal (constant across pulses)
        let mut mti = MtiFilter::new();
        let num_bins = 4;
        let dc_val = (10.0, -3.0);
        let pulse = vec![dc_val; num_bins];
        let output = mti.process(&[pulse.clone(), pulse.clone()]);
        assert_eq!(output.len(), 1);
        let power: f64 = output[0].iter().map(|s| cx_mag_sq(*s)).sum();
        assert!(power < 1e-20, "DC power not rejected: {power}");
    }

    #[test]
    fn test_moving_target_passes() {
        // A target with 180-degree phase shift per pulse should pass at full gain
        let mut mti = MtiFilter::new();
        let p0 = vec![(1.0, 0.0); 4];
        let p1 = vec![(-1.0, 0.0); 4]; // max Doppler
        let output = mti.process(&[p0, p1]);
        assert_eq!(output.len(), 1);
        for &(re, im) in &output[0] {
            // y = (-1) - (1) = -2
            assert!((re - (-2.0)).abs() < 1e-10, "target not passed: re={re}");
            assert!(im.abs() < 1e-10, "imaginary leakage: im={im}");
        }

        // Intermediate Doppler: 90-degree phase shift
        let mut mti2 = MtiFilter::new();
        let q0 = vec![(1.0, 0.0)];
        let q1 = vec![(0.0, 1.0)]; // 90 degrees
        let out2 = mti2.process(&[q0, q1]);
        // y = (0,1) - (1,0) = (-1, 1), magnitude = sqrt(2)
        let mag = cx_mag_sq(out2[0][0]).sqrt();
        assert!(
            (mag - std::f64::consts::SQRT_2).abs() < 1e-10,
            "90-deg target magnitude wrong: {mag}"
        );
    }

    #[test]
    fn test_doppler_filter() {
        // Use simple MTI-like coefficients [1, -1] through DopplerFilter
        let df = DopplerFilter::new(vec![1.0, -1.0]);
        let p0 = vec![(3.0, 1.0); 4];
        let p1 = vec![(3.0, 1.0); 4]; // same = DC
        let output = df.process(&[p0, p1]);
        assert_eq!(output.len(), 1);
        // Should cancel DC: coeff[0]*p0 + coeff[1]*p1 = p0 - p1 = 0
        // Wait -- the convolution direction: out[0][b] = 1.0*p0[b] + (-1.0)*p1[b]
        for &(re, im) in &output[0] {
            assert!(re.abs() < 1e-10, "Doppler filter DC not rejected: re={re}");
            assert!(im.abs() < 1e-10, "Doppler filter DC not rejected: im={im}");
        }

        // Non-DC target should pass
        let df2 = DopplerFilter::new(vec![1.0, -1.0]);
        let q0 = vec![(1.0, 0.0)];
        let q1 = vec![(-1.0, 0.0)];
        let out2 = df2.process(&[q0, q1]);
        // out = 1.0*(1,0) + (-1.0)*(-1,0) = (1+1, 0) = (2, 0)
        assert!((out2[0][0].0 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_clutter_notch_design() {
        let notch_width = 0.05; // 5% of PRF
        let num_taps = 31;
        let h = design_clutter_notch(notch_width, num_taps);
        assert_eq!(h.len(), num_taps);

        // Evaluate frequency response at DC: should be near zero
        let h_dc: f64 = h.iter().sum();
        assert!(
            h_dc.abs() < 0.05,
            "Clutter notch DC response too high: {h_dc}"
        );

        // Evaluate at f = 0.5 (Nyquist): should be near 1.0
        let h_nyq: f64 = h
            .iter()
            .enumerate()
            .map(|(i, &c)| c * if i % 2 == 0 { 1.0 } else { -1.0 })
            .sum();
        assert!(
            (h_nyq.abs() - 1.0).abs() < 0.05,
            "Clutter notch Nyquist response not near 1.0: {h_nyq}"
        );
    }

    #[test]
    fn test_improvement_factor() {
        // 1 pulse: no improvement
        assert_eq!(mti_improvement_factor(1), 0.0);

        // 2 pulses: ~3 dB
        let if2 = mti_improvement_factor(2);
        assert!(
            (if2 - 3.010).abs() < 0.1,
            "2-pulse IF = {if2}, expected ~3.01 dB"
        );

        // 10 pulses: 10 dB
        let if10 = mti_improvement_factor(10);
        assert!(
            (if10 - 10.0).abs() < 0.01,
            "10-pulse IF = {if10}, expected 10 dB"
        );

        // Monotonically increasing
        let if4 = mti_improvement_factor(4);
        let if8 = mti_improvement_factor(8);
        assert!(if8 > if4, "IF should increase with more pulses");
    }

    #[test]
    fn test_clutter_map() {
        let signal = vec![10.0, 20.0, 5.0, 15.0];
        let clutter = vec![8.0, 18.0, 6.0, 10.0];

        // alpha = 0: full map subtraction
        let out0 = apply_clutter_map(&signal, &clutter, 0.0);
        assert!((out0[0] - 2.0).abs() < 1e-10); // 10 - 8
        assert!((out0[1] - 2.0).abs() < 1e-10); // 20 - 18
        assert!((out0[2] - 0.0).abs() < 1e-10); // 5 - 6 clamped to 0
        assert!((out0[3] - 5.0).abs() < 1e-10); // 15 - 10

        // alpha = 1: no subtraction (output = signal)
        let out1 = apply_clutter_map(&signal, &clutter, 1.0);
        for i in 0..4 {
            assert!(
                (out1[i] - signal[i]).abs() < 1e-10,
                "alpha=1 should pass through"
            );
        }

        // alpha = 0.5: partial subtraction
        let out05 = apply_clutter_map(&signal, &clutter, 0.5);
        assert!((out05[0] - (10.0 - 0.5 * 8.0)).abs() < 1e-10);
    }

    #[test]
    fn test_streaming_mti() {
        let mut mti = MtiFilter::new();

        // First pulse: no output
        let p0 = vec![(1.0, 2.0), (3.0, 4.0)];
        assert!(mti.single_pulse(&p0).is_none());

        // Second pulse: output = p1 - p0
        let p1 = vec![(5.0, 6.0), (7.0, 8.0)];
        let out1 = mti.single_pulse(&p1).unwrap();
        assert_eq!(out1.len(), 2);
        assert!((out1[0].0 - 4.0).abs() < 1e-10); // 5-1
        assert!((out1[0].1 - 4.0).abs() < 1e-10); // 6-2
        assert!((out1[1].0 - 4.0).abs() < 1e-10); // 7-3
        assert!((out1[1].1 - 4.0).abs() < 1e-10); // 8-4

        // Third pulse: continues streaming
        let p2 = vec![(5.0, 6.0), (7.0, 8.0)]; // same as p1
        let out2 = mti.single_pulse(&p2).unwrap();
        for &(re, im) in &out2 {
            assert!(re.abs() < 1e-10, "streaming DC not cancelled");
            assert!(im.abs() < 1e-10, "streaming DC not cancelled");
        }
    }

    #[test]
    fn test_empty_input() {
        // MtiFilter with empty input
        let mut mti = MtiFilter::new();
        let output = mti.process(&[]);
        assert!(output.is_empty());

        // MtiFilter with single pulse (needs 2)
        let output2 = mti.process(&[vec![(1.0, 0.0)]]);
        assert!(output2.is_empty());

        // DoubleCanceller with fewer than 3 pulses
        let mut dc = DoubleCanceller::new();
        let out_dc = dc.process(&[vec![(1.0, 0.0)], vec![(2.0, 0.0)]]);
        assert!(out_dc.is_empty());

        // DopplerFilter with no pulses
        let df = DopplerFilter::new(vec![1.0, -1.0]);
        let out_df = df.process(&[]);
        assert!(out_df.is_empty());

        // DopplerFilter with fewer pulses than taps
        let out_df2 = df.process(&[vec![(1.0, 0.0)]]);
        assert!(out_df2.is_empty());

        // design_clutter_notch with 0 taps
        let h = design_clutter_notch(0.1, 0);
        assert!(h.is_empty());

        // apply_clutter_map with empty inputs
        let out_cm = apply_clutter_map(&[], &[], 0.5);
        assert!(out_cm.is_empty());
    }
}

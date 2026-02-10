//! PLL-Based Carrier Tracking
//!
//! Locks onto and tracks a carrier frequency in received IQ samples,
//! outputting the carrier-removed (baseband) signal. Uses a second-order
//! control loop with proportional-integral (PI) filter to track the
//! carrier phase and frequency.
//!
//! The loop operates sample-by-sample:
//! 1. Mix the input down by the NCO phase to produce a baseband sample
//! 2. Compute the phase error via `atan2(im, re)` of the mixed-down sample
//! 3. Update the NCO: `freq += beta * error; phase += freq + alpha * error`
//! 4. Wrap phase to `[-pi, pi]`
//!
//! Also provides [`PllFreqDet`], a frequency-detector variant that outputs
//! instantaneous frequency estimates instead of baseband IQ.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::pll_carrier_tracking::PllCarrierTracking;
//! use std::f64::consts::PI;
//!
//! // Create a PLL to track a carrier with loop BW = 0.05 rad/sample
//! let mut pll = PllCarrierTracking::new(0.05, 0.5, -0.5);
//!
//! // Generate a carrier at 0.1 rad/sample
//! let carrier_freq = 0.1;
//! let input: Vec<(f64, f64)> = (0..500)
//!     .map(|i| {
//!         let phase = carrier_freq * i as f64;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let baseband = pll.process(&input);
//!
//! // After convergence the PLL should be locked
//! assert!(pll.is_locked());
//! // Frequency estimate should be close to 0.1 rad/sample
//! assert!((pll.frequency() - carrier_freq).abs() < 0.01);
//! ```

use std::f64::consts::PI;

/// PLL-based carrier tracking block.
///
/// Mixes input IQ samples down by a numerically-controlled oscillator (NCO),
/// computes the phase error of the resulting baseband signal, and feeds the
/// error through a second-order PI loop filter to update the NCO phase and
/// frequency.
#[derive(Debug, Clone)]
pub struct PllCarrierTracking {
    /// NCO phase accumulator (radians).
    phase: f64,
    /// NCO frequency (radians/sample).
    freq: f64,
    /// Proportional gain.
    alpha: f64,
    /// Integral gain.
    beta: f64,
    /// Minimum NCO frequency (radians/sample).
    min_freq: f64,
    /// Maximum NCO frequency (radians/sample).
    max_freq: f64,
    /// Lock detector smoothing factor.
    lock_detector_alpha: f64,
    /// Smoothed power of the in-phase (real) component after mixing.
    lock_power: f64,
    /// Smoothed total signal power after mixing.
    signal_power: f64,
}

impl PllCarrierTracking {
    /// Create a new PLL carrier tracking block.
    ///
    /// * `loop_bandwidth` - Loop bandwidth in radians per sample.
    /// * `max_freq` - Maximum NCO frequency (radians/sample).
    /// * `min_freq` - Minimum NCO frequency (radians/sample).
    ///
    /// Uses a damping factor of `sqrt(2)/2` (critically damped).
    pub fn new(loop_bandwidth: f64, max_freq: f64, min_freq: f64) -> Self {
        let (alpha, beta) = compute_gains(loop_bandwidth);
        Self {
            phase: 0.0,
            freq: 0.0,
            alpha,
            beta,
            min_freq,
            max_freq,
            lock_detector_alpha: 0.02,
            lock_power: 0.0,
            signal_power: 0.0,
        }
    }

    /// Process a block of IQ samples.
    ///
    /// For each sample, mixes down by the NCO, computes the phase error,
    /// and updates the loop. Returns the carrier-removed baseband signal.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &(re_in, im_in) in input {
            // Mix down by NCO: multiply input by exp(-j * phase)
            let nco_cos = self.phase.cos();
            let nco_sin = self.phase.sin();
            // (re_in + j*im_in) * (cos(phase) - j*sin(phase))
            let bb_re = re_in * nco_cos + im_in * nco_sin;
            let bb_im = im_in * nco_cos - re_in * nco_sin;

            output.push((bb_re, bb_im));

            // Phase error detector: atan2(im, re) of mixed-down signal
            let error = bb_im.atan2(bb_re);

            // Update lock detector
            let alpha_ld = self.lock_detector_alpha;
            self.lock_power =
                (1.0 - alpha_ld) * self.lock_power + alpha_ld * bb_re.abs();
            self.signal_power =
                (1.0 - alpha_ld) * self.signal_power + alpha_ld * (bb_re * bb_re + bb_im * bb_im).sqrt();

            // Loop filter update (PI controller)
            self.freq += self.beta * error;
            self.freq = self.freq.clamp(self.min_freq, self.max_freq);
            self.phase += self.freq + self.alpha * error;

            // Wrap phase to [-pi, pi]
            self.phase = wrap_phase(self.phase);
        }
        output
    }

    /// Get the current NCO phase (radians).
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Get the current NCO frequency estimate (radians/sample).
    pub fn frequency(&self) -> f64 {
        self.freq
    }

    /// Check whether the PLL is locked.
    ///
    /// Returns `true` when the lock detector power ratio exceeds 0.8,
    /// meaning most of the signal power is concentrated in the real
    /// (in-phase) component after carrier removal.
    pub fn is_locked(&self) -> bool {
        self.lock_indicator() > 0.8
    }

    /// Lock indicator value in the range `[0.0, 1.0]`.
    ///
    /// - `0.0` = unlocked (signal energy evenly split between I and Q)
    /// - `1.0` = locked (all signal energy in the I component)
    pub fn lock_indicator(&self) -> f64 {
        if self.signal_power < 1e-20 {
            return 0.0;
        }
        (self.lock_power / self.signal_power).clamp(0.0, 1.0)
    }

    /// Reset all state to initial values.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = 0.0;
        self.lock_power = 0.0;
        self.signal_power = 0.0;
    }

    /// Set the loop bandwidth, recomputing alpha and beta.
    pub fn set_loop_bandwidth(&mut self, bw: f64) {
        let (alpha, beta) = compute_gains(bw);
        self.alpha = alpha;
        self.beta = beta;
    }
}

/// PLL frequency detector variant.
///
/// Same second-order loop as [`PllCarrierTracking`], but outputs
/// instantaneous frequency estimates (radians/sample) instead of
/// baseband IQ samples.
#[derive(Debug, Clone)]
pub struct PllFreqDet {
    /// NCO phase accumulator (radians).
    phase: f64,
    /// NCO frequency (radians/sample).
    freq: f64,
    /// Proportional gain.
    alpha: f64,
    /// Integral gain.
    beta: f64,
    /// Minimum NCO frequency (radians/sample).
    min_freq: f64,
    /// Maximum NCO frequency (radians/sample).
    max_freq: f64,
}

impl PllFreqDet {
    /// Create a new PLL frequency detector.
    ///
    /// * `loop_bandwidth` - Loop bandwidth in radians per sample.
    /// * `max_freq` - Maximum NCO frequency (radians/sample).
    /// * `min_freq` - Minimum NCO frequency (radians/sample).
    pub fn new(loop_bandwidth: f64, max_freq: f64, min_freq: f64) -> Self {
        let (alpha, beta) = compute_gains(loop_bandwidth);
        Self {
            phase: 0.0,
            freq: 0.0,
            alpha,
            beta,
            min_freq,
            max_freq,
        }
    }

    /// Process a block of IQ samples.
    ///
    /// Returns a vector of instantaneous frequency estimates
    /// (radians/sample) for each input sample.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &(re_in, im_in) in input {
            // Mix down by NCO
            let nco_cos = self.phase.cos();
            let nco_sin = self.phase.sin();
            let bb_re = re_in * nco_cos + im_in * nco_sin;
            let bb_im = im_in * nco_cos - re_in * nco_sin;

            // Phase error
            let error = bb_im.atan2(bb_re);

            // Loop filter update
            self.freq += self.beta * error;
            self.freq = self.freq.clamp(self.min_freq, self.max_freq);
            self.phase += self.freq + self.alpha * error;
            self.phase = wrap_phase(self.phase);

            output.push(self.freq);
        }
        output
    }

    /// Get the current frequency estimate (radians/sample).
    pub fn frequency(&self) -> f64 {
        self.freq
    }

    /// Reset all state to initial values.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = 0.0;
    }
}

/// Compute PI controller gains from loop bandwidth.
///
/// Uses damping factor `sqrt(2)/2` (critically damped) and the standard
/// second-order loop formula:
///
/// ```text
/// denom = 1 + 2*damping*bw + bw^2
/// alpha = 4*damping*bw / denom
/// beta  = 4*bw^2 / denom
/// ```
fn compute_gains(bw: f64) -> (f64, f64) {
    let damping = 1.0 / 2.0_f64.sqrt(); // sqrt(2)/2
    let denom = 1.0 + 2.0 * damping * bw + bw * bw;
    let alpha = 4.0 * damping * bw / denom;
    let beta = 4.0 * bw * bw / denom;
    (alpha, beta)
}

/// Wrap phase to `[-pi, pi]`.
fn wrap_phase(phase: f64) -> f64 {
    let mut p = phase;
    while p > PI {
        p -= 2.0 * PI;
    }
    while p < -PI {
        p += 2.0 * PI;
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate a pure carrier signal at the given frequency (rad/sample).
    fn generate_carrier(freq: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = freq * i as f64;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_lock_to_carrier() {
        let carrier_freq = 0.1; // rad/sample
        let mut pll = PllCarrierTracking::new(0.05, 0.5, -0.5);
        let input = generate_carrier(carrier_freq, 1000);

        let output = pll.process(&input);

        // Output length must match input
        assert_eq!(output.len(), input.len());

        // After convergence the baseband signal should have most energy
        // on the real axis (carrier removed, residual near DC).
        let last_100 = &output[900..];
        let avg_im: f64 = last_100.iter().map(|(_, im)| im.abs()).sum::<f64>() / 100.0;
        assert!(
            avg_im < 0.2,
            "Imaginary component should be small after lock: avg |im| = {avg_im:.4}"
        );

        // PLL should report locked
        assert!(pll.is_locked(), "PLL should be locked after processing");
    }

    #[test]
    fn test_frequency_estimate() {
        let carrier_freq = 0.15;
        let mut pll = PllCarrierTracking::new(0.06, 0.5, -0.5);
        let input = generate_carrier(carrier_freq, 1500);
        pll.process(&input);

        let est = pll.frequency();
        assert!(
            (est - carrier_freq).abs() < 0.01,
            "Frequency estimate {est:.5} should be close to {carrier_freq}"
        );
    }

    #[test]
    fn test_phase_tracking() {
        // Start with a known phase offset
        let carrier_freq = 0.08;
        let phase_offset = 1.0; // radians
        let mut pll = PllCarrierTracking::new(0.05, 0.5, -0.5);

        let input: Vec<(f64, f64)> = (0..2000)
            .map(|i| {
                let phase = carrier_freq * i as f64 + phase_offset;
                (phase.cos(), phase.sin())
            })
            .collect();

        let output = pll.process(&input);

        // After convergence the output should be near-real (carrier stripped)
        let tail = &output[1500..];
        let avg_im: f64 = tail.iter().map(|(_, im)| im.abs()).sum::<f64>() / tail.len() as f64;
        assert!(
            avg_im < 0.15,
            "Phase tracking should remove carrier: avg |im| = {avg_im:.4}"
        );
    }

    #[test]
    fn test_lock_detector() {
        let mut pll = PllCarrierTracking::new(0.05, 0.5, -0.5);

        // Before processing, lock indicator should be 0 (no signal)
        assert!(
            pll.lock_indicator() < 0.01,
            "Lock indicator should start at 0: got {:.3}",
            pll.lock_indicator()
        );

        // Process a clean carrier to achieve lock
        let input = generate_carrier(0.1, 1000);
        pll.process(&input);

        let li = pll.lock_indicator();
        assert!(
            li > 0.7,
            "Lock indicator should be high after tracking clean carrier: {li:.3}"
        );
        assert!(pll.is_locked());
    }

    #[test]
    fn test_reset() {
        let mut pll = PllCarrierTracking::new(0.05, 0.5, -0.5);
        let input = generate_carrier(0.1, 500);
        pll.process(&input);

        // Verify state is non-zero
        assert!(pll.frequency().abs() > 0.01);
        assert!(pll.lock_indicator() > 0.1);

        pll.reset();
        assert!(
            pll.phase().abs() < 1e-12,
            "Phase should be zero after reset"
        );
        assert!(
            pll.frequency().abs() < 1e-12,
            "Frequency should be zero after reset"
        );
        assert!(
            pll.lock_indicator() < 1e-12,
            "Lock indicator should be zero after reset"
        );
    }

    #[test]
    fn test_freq_limits() {
        let mut pll = PllCarrierTracking::new(0.1, 0.2, -0.2);

        // Drive with a carrier beyond the max frequency
        let input = generate_carrier(0.4, 2000);
        pll.process(&input);

        assert!(
            pll.frequency() <= 0.2 + 1e-12,
            "Frequency should not exceed max_freq: got {:.5}",
            pll.frequency()
        );
        assert!(
            pll.frequency() >= -0.2 - 1e-12,
            "Frequency should not go below min_freq: got {:.5}",
            pll.frequency()
        );
    }

    #[test]
    fn test_freq_det() {
        let carrier_freq = 0.12;
        let mut fd = PllFreqDet::new(0.06, 0.5, -0.5);
        let input = generate_carrier(carrier_freq, 1500);

        let freqs = fd.process(&input);
        assert_eq!(freqs.len(), input.len());

        // After convergence, frequency estimates should be near carrier_freq
        let tail = &freqs[1000..];
        let avg: f64 = tail.iter().sum::<f64>() / tail.len() as f64;
        assert!(
            (avg - carrier_freq).abs() < 0.01,
            "FreqDet average {avg:.5} should be near {carrier_freq}"
        );

        // Final estimate via getter
        assert!(
            (fd.frequency() - carrier_freq).abs() < 0.01,
            "FreqDet.frequency() = {:.5}, expected ~{carrier_freq}",
            fd.frequency()
        );
    }

    #[test]
    fn test_bandwidth_change() {
        let mut pll = PllCarrierTracking::new(0.01, 0.5, -0.5);

        // With narrow bandwidth, convergence is slow
        let input = generate_carrier(0.1, 300);
        pll.process(&input);
        let freq_narrow = pll.frequency();

        // Widen bandwidth and process more
        pll.reset();
        pll.set_loop_bandwidth(0.08);
        pll.process(&input);
        let freq_wide = pll.frequency();

        // Wider bandwidth should converge faster (closer to true freq)
        assert!(
            (freq_wide - 0.1).abs() < (freq_narrow - 0.1).abs(),
            "Wider BW should converge faster: narrow err={:.4}, wide err={:.4}",
            (freq_narrow - 0.1).abs(),
            (freq_wide - 0.1).abs()
        );
    }

    #[test]
    fn test_zero_input() {
        let mut pll = PllCarrierTracking::new(0.05, 0.5, -0.5);
        let input = vec![(0.0, 0.0); 100];
        let output = pll.process(&input);

        assert_eq!(output.len(), 100);
        // All outputs should be zero
        for (re, im) in &output {
            assert!(re.abs() < 1e-15, "Real should be zero for zero input");
            assert!(im.abs() < 1e-15, "Imag should be zero for zero input");
        }
        // Frequency should stay near zero
        assert!(
            pll.frequency().abs() < 1e-10,
            "Frequency should remain zero for zero input"
        );
    }

    #[test]
    fn test_noisy_carrier() {
        let carrier_freq = 0.1;
        let mut pll = PllCarrierTracking::new(0.04, 0.5, -0.5);

        // Simple deterministic "noise" via a secondary incommensurate tone
        let input: Vec<(f64, f64)> = (0..3000)
            .map(|i| {
                let t = i as f64;
                let phase = carrier_freq * t;
                let noise_re = 0.1 * (0.7123 * t).sin();
                let noise_im = 0.1 * (1.3217 * t).cos();
                (phase.cos() + noise_re, phase.sin() + noise_im)
            })
            .collect();

        pll.process(&input);

        let est = pll.frequency();
        assert!(
            (est - carrier_freq).abs() < 0.02,
            "PLL should track carrier in noise: est={est:.5}, expected={carrier_freq}"
        );
        // Lock indicator should still be reasonable (above 0.5)
        assert!(
            pll.lock_indicator() > 0.5,
            "Lock indicator should be above 0.5 with mild noise: {:.3}",
            pll.lock_indicator()
        );
    }
}

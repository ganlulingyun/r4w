//! Seismic survey data processing for oil/gas exploration.
//!
//! This module processes geophone array recordings for reflection seismology,
//! implementing the standard seismic processing workflow: CMP gathering,
//! velocity analysis (semblance), NMO correction, stacking, and migration.
//!
//! # Overview
//!
//! In a seismic survey, a source (vibrator or explosive) generates acoustic energy
//! that propagates into the subsurface. Reflections from impedance contrasts are
//! recorded by an array of geophones on the surface. Processing transforms these
//! raw shot gathers into a depth-migrated image of subsurface geology.
//!
//! The key processing steps are:
//!
//! 1. **Preprocessing** — AGC, bandpass filtering, first-break muting
//! 2. **CMP sorting** — Group traces by common midpoint
//! 3. **Velocity analysis** — Semblance scan to pick stacking velocities
//! 4. **NMO correction** — Remove offset-dependent moveout
//! 5. **CMP stacking** — Sum corrected traces to enhance S/N ratio
//! 6. **Migration** — Collapse diffractions and position reflectors correctly
//!
//! # Example
//!
//! ```
//! use r4w_core::geophone_array_processor::{
//!     SeismicSurveyConfig, SeismicProcessor, ricker_wavelet, nmo_correction,
//! };
//!
//! let config = SeismicSurveyConfig {
//!     num_receivers: 24,
//!     receiver_spacing_m: 25.0,
//!     sample_rate_hz: 1000.0,
//!     source_offset_m: 50.0,
//!     max_time_s: 2.0,
//! };
//!
//! let processor = SeismicProcessor::new(config.clone());
//!
//! // Generate a Ricker wavelet (Mexican hat)
//! let wavelet = ricker_wavelet(64, 30.0, 1.0 / config.sample_rate_hz);
//! assert_eq!(wavelet.len(), 64);
//!
//! // NMO correction of a single trace
//! let trace = vec![0.0; 2000];
//! let corrected = nmo_correction(&trace, 200.0, 3000.0, 0.001);
//! assert_eq!(corrected.len(), trace.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration & data types
// ---------------------------------------------------------------------------

/// Configuration for a seismic reflection survey geometry.
#[derive(Debug, Clone)]
pub struct SeismicSurveyConfig {
    /// Number of receiver (geophone) channels.
    pub num_receivers: usize,
    /// Spacing between adjacent receivers in metres.
    pub receiver_spacing_m: f64,
    /// Sample rate of recorded data in Hz.
    pub sample_rate_hz: f64,
    /// Offset from source to first receiver in metres.
    pub source_offset_m: f64,
    /// Maximum recording time in seconds.
    pub max_time_s: f64,
}

impl Default for SeismicSurveyConfig {
    fn default() -> Self {
        Self {
            num_receivers: 24,
            receiver_spacing_m: 25.0,
            sample_rate_hz: 1000.0,
            source_offset_m: 50.0,
            max_time_s: 2.0,
        }
    }
}

/// Result of processing a CMP gather through the seismic workflow.
#[derive(Debug, Clone)]
pub struct ProcessedGather {
    /// Stacked (summed) trace after NMO correction.
    pub stacked_trace: Vec<f64>,
    /// Estimated RMS stacking velocities (m/s) per time sample.
    pub velocities: Vec<f64>,
    /// Depth axis (metres) corresponding to each time sample.
    pub depth_axis_m: Vec<f64>,
}

// ---------------------------------------------------------------------------
// SeismicProcessor
// ---------------------------------------------------------------------------

/// Seismic data processor for geophone array recordings.
///
/// Encapsulates the survey geometry and provides methods for the complete
/// reflection seismology processing workflow.
#[derive(Debug, Clone)]
pub struct SeismicProcessor {
    config: SeismicSurveyConfig,
}

impl SeismicProcessor {
    /// Create a new processor with the given survey configuration.
    pub fn new(config: SeismicSurveyConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the survey configuration.
    pub fn config(&self) -> &SeismicSurveyConfig {
        &self.config
    }

    /// Compute the offset (source-to-receiver distance) for receiver `i`.
    pub fn offset(&self, receiver_index: usize) -> f64 {
        self.config.source_offset_m
            + receiver_index as f64 * self.config.receiver_spacing_m
    }

    /// Process a shot gather (one trace per receiver) through the full workflow.
    ///
    /// Steps: AGC -> velocity scan -> pick best velocity -> NMO -> stack.
    pub fn process_gather(&self, traces: &[Vec<f64>]) -> ProcessedGather {
        let dt = 1.0 / self.config.sample_rate_hz;
        let n_samples = if traces.is_empty() {
            0
        } else {
            traces[0].len()
        };

        // 1. Apply AGC to each trace
        let agc_window = (0.1 * self.config.sample_rate_hz) as usize;
        let agc_window = agc_window.max(1);
        let agc_traces: Vec<Vec<f64>> = traces
            .iter()
            .map(|t| agc(t, agc_window))
            .collect();

        // 2. Compute offsets
        let offsets: Vec<f64> = (0..agc_traces.len())
            .map(|i| self.offset(i))
            .collect();

        // 3. Velocity analysis via semblance
        let v_min = 1500.0_f64;
        let v_max = 5000.0_f64;
        let n_vel = 50;
        let velocities_scan: Vec<f64> = (0..n_vel)
            .map(|i| v_min + (v_max - v_min) * i as f64 / (n_vel - 1).max(1) as f64)
            .collect();

        let semb = semblance_analysis(&agc_traces, &offsets, &velocities_scan, dt);

        // 4. Pick velocity with maximum semblance at each time sample
        let mut best_velocities = vec![v_min; n_samples];
        for t_idx in 0..n_samples.min(semb.len()) {
            let mut max_s = -1.0_f64;
            let mut best_v_idx = 0;
            for (v_idx, &s) in semb[t_idx].iter().enumerate() {
                if s > max_s {
                    max_s = s;
                    best_v_idx = v_idx;
                }
            }
            if best_v_idx < velocities_scan.len() {
                best_velocities[t_idx] = velocities_scan[best_v_idx];
            }
        }

        // 5. NMO correct and stack using the picked velocity at each time
        //    For stacking we use a single representative velocity (median).
        let mut sorted_vel = best_velocities.clone();
        sorted_vel.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_vel = if sorted_vel.is_empty() {
            v_min
        } else {
            sorted_vel[sorted_vel.len() / 2]
        };

        let stacked = cmp_stack(&agc_traces, &offsets, median_vel, dt);

        // 6. Build depth axis: depth = v * t / 2 (two-way travel time)
        let depth_axis_m: Vec<f64> = (0..n_samples)
            .map(|i| {
                let t = i as f64 * dt;
                best_velocities[i] * t / 2.0
            })
            .collect();

        ProcessedGather {
            stacked_trace: stacked,
            velocities: best_velocities,
            depth_axis_m,
        }
    }
}

// ---------------------------------------------------------------------------
// Core seismic algorithms (public free functions)
// ---------------------------------------------------------------------------

/// Normal moveout (NMO) correction for a single trace.
///
/// Maps each sample from NMO time `t_nmo = sqrt(t0^2 + x^2 / v^2)` back to
/// zero-offset time `t0`. Uses linear interpolation for sub-sample accuracy.
///
/// # Arguments
/// * `trace`        - Input seismic trace samples.
/// * `offset_m`     - Source-to-receiver offset in metres.
/// * `velocity_mps` - RMS stacking velocity in m/s.
/// * `dt_s`         - Sample interval in seconds.
pub fn nmo_correction(
    trace: &[f64],
    offset_m: f64,
    velocity_mps: f64,
    dt_s: f64,
) -> Vec<f64> {
    let n = trace.len();
    let mut corrected = vec![0.0; n];

    if velocity_mps <= 0.0 || dt_s <= 0.0 {
        return corrected;
    }

    let x_over_v_sq = (offset_m / velocity_mps).powi(2);

    for i in 0..n {
        let t0 = i as f64 * dt_s;
        let t_nmo_sq = t0 * t0 + x_over_v_sq;
        let t_nmo = t_nmo_sq.sqrt();
        let sample_idx = t_nmo / dt_s;

        // Linear interpolation
        let idx_lo = sample_idx.floor() as usize;
        let idx_hi = idx_lo + 1;
        if idx_hi < n {
            let frac = sample_idx - idx_lo as f64;
            corrected[i] = trace[idx_lo] * (1.0 - frac) + trace[idx_hi] * frac;
        }
        // If t_nmo maps beyond the trace, leave as zero (mute stretch)
    }

    corrected
}

/// Common midpoint (CMP) stack after NMO correction.
///
/// Applies NMO correction to each trace at its offset, then sums (stacks)
/// all corrected traces. The result is normalised by fold (number of live
/// traces contributing at each sample).
///
/// # Arguments
/// * `traces`   - Array of seismic traces (one per offset).
/// * `offsets`  - Source-to-receiver offset for each trace (metres).
/// * `velocity` - RMS stacking velocity in m/s.
/// * `dt_s`     - Sample interval in seconds.
pub fn cmp_stack(
    traces: &[Vec<f64>],
    offsets: &[f64],
    velocity: f64,
    dt_s: f64,
) -> Vec<f64> {
    if traces.is_empty() {
        return Vec::new();
    }

    let n = traces[0].len();
    let mut stack = vec![0.0; n];
    let mut fold = vec![0u32; n];

    for (trace, &offset) in traces.iter().zip(offsets.iter()) {
        let corrected = nmo_correction(trace, offset, velocity, dt_s);
        for i in 0..n.min(corrected.len()) {
            if corrected[i] != 0.0 || (i as f64 * dt_s).powi(2) > (offset / velocity).powi(2) {
                stack[i] += corrected[i];
                fold[i] += 1;
            }
        }
    }

    // Normalise by fold
    for i in 0..n {
        if fold[i] > 0 {
            stack[i] /= fold[i] as f64;
        }
    }

    stack
}

/// Velocity analysis via semblance (coherence) measure.
///
/// Computes a 2-D semblance panel: `semblance[time_index][velocity_index]`.
/// Semblance S(t0, v) = (sum of NMO-corrected amplitudes)^2 / (N * sum of squares),
/// where N is the number of traces (fold).
///
/// Values range from 0 (no coherence) to 1 (perfect coherence).
///
/// # Arguments
/// * `traces`     - Array of seismic traces.
/// * `offsets`     - Source-to-receiver offsets in metres.
/// * `velocities`  - Array of trial velocities (m/s) to scan.
/// * `dt_s`        - Sample interval in seconds.
pub fn semblance_analysis(
    traces: &[Vec<f64>],
    offsets: &[f64],
    velocities: &[f64],
    dt_s: f64,
) -> Vec<Vec<f64>> {
    if traces.is_empty() || velocities.is_empty() {
        return Vec::new();
    }

    let n_samples = traces[0].len();
    let n_traces = traces.len();
    let mut result = vec![vec![0.0; velocities.len()]; n_samples];

    // Semblance window half-width (in samples) for smoothing
    let half_win = 3_usize;

    for (v_idx, &vel) in velocities.iter().enumerate() {
        // NMO-correct all traces at this velocity
        let corrected: Vec<Vec<f64>> = traces
            .iter()
            .zip(offsets.iter())
            .map(|(tr, &off)| nmo_correction(tr, off, vel, dt_s))
            .collect();

        for t0 in 0..n_samples {
            let win_lo = t0.saturating_sub(half_win);
            let win_hi = (t0 + half_win + 1).min(n_samples);

            let mut numerator = 0.0;
            let mut denominator = 0.0;

            for t in win_lo..win_hi {
                let mut sum_amp = 0.0;
                let mut sum_sq = 0.0;
                for tr_idx in 0..n_traces {
                    let a = corrected[tr_idx][t];
                    sum_amp += a;
                    sum_sq += a * a;
                }
                numerator += sum_amp * sum_amp;
                denominator += sum_sq;
            }

            denominator *= n_traces as f64;

            result[t0][v_idx] = if denominator > 1e-30 {
                (numerator / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
        }
    }

    result
}

/// Automatic Gain Control (AGC).
///
/// Normalises trace amplitudes using a sliding RMS window so that weak
/// late arrivals are boosted to the same level as strong early arrivals.
///
/// # Arguments
/// * `trace`          - Input seismic trace.
/// * `window_samples` - Length of the AGC window in samples.
pub fn agc(trace: &[f64], window_samples: usize) -> Vec<f64> {
    let n = trace.len();
    if n == 0 || window_samples == 0 {
        return trace.to_vec();
    }

    let half = window_samples / 2;
    let mut output = vec![0.0; n];

    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let count = (hi - lo) as f64;

        // RMS of the window
        let rms: f64 = trace[lo..hi]
            .iter()
            .map(|&x| x * x)
            .sum::<f64>()
            / count;
        let rms = rms.sqrt();

        output[i] = if rms > 1e-30 {
            trace[i] / rms
        } else {
            0.0
        };
    }

    output
}

/// Ormsby trapezoid bandpass filter.
///
/// Designs and applies a zero-phase (forward-backward) FIR filter with an
/// Ormsby (trapezoidal) amplitude spectrum defined by four corner frequencies:
/// `[f1, f2, f3, f4]` where the passband is flat between `low_hz` (f2) and
/// `high_hz` (f3), with linear roll-off slopes.
///
/// # Arguments
/// * `trace`   - Input seismic trace.
/// * `fs`      - Sample rate in Hz.
/// * `low_hz`  - Lower passband corner frequency (Hz).
/// * `high_hz` - Upper passband corner frequency (Hz).
pub fn bandpass_filter(trace: &[f64], fs: f64, low_hz: f64, high_hz: f64) -> Vec<f64> {
    let n = trace.len();
    if n == 0 || fs <= 0.0 || low_hz >= high_hz {
        return trace.to_vec();
    }

    // Ormsby trapezoid corners: 10% roll-off slopes
    let f1 = (low_hz * 0.7).max(0.0);
    let f2 = low_hz;
    let f3 = high_hz;
    let f4 = (high_hz * 1.3).min(fs / 2.0);

    // Design filter length (odd)
    let filter_len = ((2.0 * fs / f1.max(1.0)) as usize).max(31) | 1;
    let half = filter_len / 2;

    // Build Ormsby impulse response
    let mut h = vec![0.0; filter_len];
    for i in 0..filter_len {
        let t = (i as f64 - half as f64) / fs;
        if t.abs() < 1e-15 {
            // Limit at t=0: area under trapezoid
            h[i] = 2.0 * (f4 - f1) - (f4 - f3) - (f2 - f1);
        } else {
            let sinc = |f: f64| (PI * f * t).sin() / (PI * t);
            let s1 = sinc(f1);
            let s2 = sinc(f2);
            let s3 = sinc(f3);
            let s4 = sinc(f4);

            // Ormsby: difference of sinc functions scaled by trapezoid slopes
            let low_slope = if (f2 - f1).abs() > 1e-12 {
                (s2 - s1) / (f2 - f1)
            } else {
                0.0
            };
            let high_slope = if (f4 - f3).abs() > 1e-12 {
                (s4 - s3) / (f4 - f3)
            } else {
                0.0
            };
            h[i] = s3 - s2 + low_slope * (f2 - f1) + high_slope * (f4 - f3);
        }
    }

    // Apply Hann window to the filter
    for i in 0..filter_len {
        let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (filter_len - 1) as f64).cos());
        h[i] *= w;
    }

    // Normalise by sum of absolute values to get unity passband gain
    let abs_sum: f64 = h.iter().map(|&x| x.abs()).sum::<f64>();
    if abs_sum > 1e-30 {
        let scale = 1.0 / abs_sum;
        for x in &mut h {
            *x *= scale;
        }
    }

    // Convolve (linear convolution, same length as input)
    convolve_same(trace, &h)
}

/// First-break (first-arrival) detection.
///
/// Finds the first sample where the absolute amplitude exceeds
/// `threshold * max_amplitude` of the trace. This approximates the
/// onset of the direct or refracted wave.
///
/// # Arguments
/// * `trace`     - Input seismic trace.
/// * `threshold` - Fraction of peak amplitude (0.0 to 1.0).
pub fn first_break_pick(trace: &[f64], threshold: f64) -> Option<usize> {
    if trace.is_empty() {
        return None;
    }

    let max_amp = trace.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    if max_amp < 1e-30 {
        return None;
    }

    let abs_threshold = threshold * max_amp;
    trace.iter().position(|&x| x.abs() >= abs_threshold)
}

/// Compute the normal-incidence reflection coefficient at an interface.
///
/// Uses the acoustic impedance formula:
///   R = (Z2 - Z1) / (Z2 + Z1)
/// where Z = rho * v (density times velocity).
///
/// # Arguments
/// * `v1`   - P-wave velocity in layer 1 (m/s).
/// * `rho1` - Density of layer 1 (kg/m^3).
/// * `v2`   - P-wave velocity in layer 2 (m/s).
/// * `rho2` - Density of layer 2 (kg/m^3).
pub fn compute_reflection_coefficient(v1: f64, rho1: f64, v2: f64, rho2: f64) -> f64 {
    let z1 = rho1 * v1;
    let z2 = rho2 * v2;
    let denom = z2 + z1;
    if denom.abs() < 1e-30 {
        return 0.0;
    }
    (z2 - z1) / denom
}

/// Generate a synthetic seismogram from a well-log impedance profile.
///
/// Computes the reflectivity series from adjacent impedance contrasts,
/// then convolves with the provided source wavelet.
///
/// # Arguments
/// * `impedance_log` - Sequence of `(velocity, density)` pairs for each layer.
/// * `wavelet`       - Source wavelet samples (e.g., from [`ricker_wavelet`]).
pub fn generate_synthetic_seismogram(
    impedance_log: &[(f64, f64)],
    wavelet: &[f64],
) -> Vec<f64> {
    if impedance_log.len() < 2 || wavelet.is_empty() {
        return Vec::new();
    }

    // Build reflectivity series
    let n_interfaces = impedance_log.len() - 1;
    let mut reflectivity = vec![0.0; n_interfaces];

    for i in 0..n_interfaces {
        let (v1, rho1) = impedance_log[i];
        let (v2, rho2) = impedance_log[i + 1];
        reflectivity[i] = compute_reflection_coefficient(v1, rho1, v2, rho2);
    }

    // Convolve reflectivity with wavelet
    convolve_same(&reflectivity, wavelet)
}

/// Generate a Ricker wavelet (Mexican hat wavelet).
///
/// The Ricker wavelet is the second derivative of a Gaussian and is the
/// most commonly used source wavelet in seismic modelling:
///
///   w(t) = (1 - 2 * pi^2 * f_p^2 * t^2) * exp(-pi^2 * f_p^2 * t^2)
///
/// # Arguments
/// * `num_samples`  - Total number of samples in the wavelet.
/// * `peak_freq_hz` - Peak (dominant) frequency in Hz.
/// * `dt_s`         - Sample interval in seconds.
pub fn ricker_wavelet(num_samples: usize, peak_freq_hz: f64, dt_s: f64) -> Vec<f64> {
    if num_samples == 0 {
        return Vec::new();
    }

    let center = (num_samples / 2) as f64;
    let fp_sq = peak_freq_hz * peak_freq_hz;

    (0..num_samples)
        .map(|i| {
            let t = (i as f64 - center) * dt_s;
            let u = PI * PI * fp_sq * t * t;
            (1.0 - 2.0 * u) * (-u).exp()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Linear convolution returning a vector of the same length as `signal`.
/// The filter is centred (zero-phase equivalent for symmetric filters).
fn convolve_same(signal: &[f64], filter: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let m = filter.len();
    if n == 0 || m == 0 {
        return signal.to_vec();
    }

    let half = m / 2;
    let mut output = vec![0.0; n];

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..m {
            let sig_idx = i as isize + j as isize - half as isize;
            if sig_idx >= 0 && (sig_idx as usize) < n {
                sum += signal[sig_idx as usize] * filter[j];
            }
        }
        output[i] = sum;
    }

    output
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // -- Ricker wavelet tests --

    #[test]
    fn test_ricker_wavelet_peak_at_center() {
        let w = ricker_wavelet(101, 30.0, 0.001);
        assert_eq!(w.len(), 101);
        // Peak should be at or near center
        let center = 50;
        let peak_idx = w
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;
        assert!((peak_idx as isize - center as isize).unsigned_abs() <= 1);
    }

    #[test]
    fn test_ricker_wavelet_symmetry() {
        let w = ricker_wavelet(101, 25.0, 0.001);
        for i in 0..50 {
            assert!(
                (w[i] - w[100 - i]).abs() < TOLERANCE,
                "Ricker wavelet not symmetric at index {}",
                i
            );
        }
    }

    #[test]
    fn test_ricker_wavelet_zero_length() {
        let w = ricker_wavelet(0, 30.0, 0.001);
        assert!(w.is_empty());
    }

    #[test]
    fn test_ricker_wavelet_peak_value() {
        // At t=0 the Ricker wavelet should equal 1.0
        let w = ricker_wavelet(101, 30.0, 0.001);
        let center = 50;
        assert!(
            (w[center] - 1.0).abs() < 0.05,
            "Ricker peak value {} not close to 1.0",
            w[center]
        );
    }

    // -- NMO correction tests --

    #[test]
    fn test_nmo_zero_offset() {
        // At zero offset, NMO correction should return the original trace
        // (last sample may be zeroed since interpolation needs idx_hi < n)
        let trace: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let corrected = nmo_correction(&trace, 0.0, 3000.0, 0.001);
        for i in 0..trace.len() - 1 {
            assert!(
                (corrected[i] - trace[i]).abs() < TOLERANCE,
                "NMO at zero offset altered sample {}",
                i
            );
        }
    }

    #[test]
    fn test_nmo_increases_with_offset() {
        // Larger offset should shift samples more (more stretch/muting at late times)
        let trace: Vec<f64> = (0..200).map(|i| if i == 100 { 1.0 } else { 0.0 }).collect();
        let corrected_near = nmo_correction(&trace, 100.0, 3000.0, 0.001);
        let corrected_far = nmo_correction(&trace, 500.0, 3000.0, 0.001);

        // The peak in the near-offset trace should be closer to sample 100
        let peak_near = corrected_near
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .unwrap()
            .0;
        let peak_far = corrected_far
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .unwrap()
            .0;

        // Near-offset peak should be at or closer to the original position
        assert!(
            peak_near <= peak_far || peak_near == 100,
            "NMO offset ordering wrong: near_peak={}, far_peak={}",
            peak_near,
            peak_far
        );
    }

    #[test]
    fn test_nmo_preserves_length() {
        let trace = vec![1.0; 500];
        let corrected = nmo_correction(&trace, 200.0, 2500.0, 0.001);
        assert_eq!(corrected.len(), trace.len());
    }

    #[test]
    fn test_nmo_invalid_velocity() {
        let trace = vec![1.0; 100];
        let corrected = nmo_correction(&trace, 200.0, 0.0, 0.001);
        assert!(corrected.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_nmo_empty_trace() {
        let corrected = nmo_correction(&[], 200.0, 3000.0, 0.001);
        assert!(corrected.is_empty());
    }

    // -- CMP stack tests --

    #[test]
    fn test_cmp_stack_single_trace() {
        let trace = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let traces = vec![trace.clone()];
        let offsets = vec![0.0]; // zero offset = no NMO shift
        let stacked = cmp_stack(&traces, &offsets, 3000.0, 0.001);
        assert_eq!(stacked.len(), 5);
        // With zero offset, stacked trace should match input
        // (last sample may be zeroed due to NMO interpolation boundary)
        for i in 0..4 {
            assert!(
                (stacked[i] - trace[i]).abs() < TOLERANCE,
                "CMP stack single trace mismatch at {}",
                i
            );
        }
    }

    #[test]
    fn test_cmp_stack_empty() {
        let stacked = cmp_stack(&[], &[], 3000.0, 0.001);
        assert!(stacked.is_empty());
    }

    #[test]
    fn test_cmp_stack_multiple_traces_coherent() {
        // If all traces have the same content at zero offset, stack = input
        let trace = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let traces = vec![trace.clone(), trace.clone(), trace.clone()];
        let offsets = vec![0.0, 0.0, 0.0];
        let stacked = cmp_stack(&traces, &offsets, 3000.0, 0.001);
        assert_eq!(stacked.len(), 5);
        // The peak should still be at index 2
        let peak_idx = stacked
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, 2);
    }

    // -- Semblance tests --

    #[test]
    fn test_semblance_perfect_coherence() {
        // Identical traces should give semblance close to 1
        let n = 100;
        let trace: Vec<f64> = (0..n).map(|i| (i as f64 * 0.05).sin()).collect();
        let traces = vec![trace.clone(), trace.clone(), trace.clone()];
        let offsets = vec![0.0, 0.0, 0.0]; // zero offsets = no NMO shift
        let velocities = vec![3000.0];
        let semb = semblance_analysis(&traces, &offsets, &velocities, 0.001);

        assert_eq!(semb.len(), n);
        // Most samples should show high semblance
        let high_count = semb.iter().filter(|row| row[0] > 0.9).count();
        assert!(
            high_count > n / 2,
            "Expected high semblance for coherent traces, got {} / {} high",
            high_count,
            n
        );
    }

    #[test]
    fn test_semblance_empty() {
        let semb = semblance_analysis(&[], &[], &[3000.0], 0.001);
        assert!(semb.is_empty());
    }

    #[test]
    fn test_semblance_output_dimensions() {
        let traces = vec![vec![0.0; 50]; 4];
        let offsets = vec![100.0, 200.0, 300.0, 400.0];
        let velocities = vec![2000.0, 3000.0, 4000.0];
        let semb = semblance_analysis(&traces, &offsets, &velocities, 0.001);
        assert_eq!(semb.len(), 50);
        assert_eq!(semb[0].len(), 3);
    }

    #[test]
    fn test_semblance_values_in_range() {
        let traces = vec![
            vec![1.0, -1.0, 1.0, -1.0, 1.0],
            vec![1.0, -1.0, 1.0, -1.0, 1.0],
        ];
        let offsets = vec![0.0, 0.0];
        let velocities = vec![2000.0, 3000.0];
        let semb = semblance_analysis(&traces, &offsets, &velocities, 0.001);
        for row in &semb {
            for &val in row {
                assert!(
                    val >= 0.0 && val <= 1.0,
                    "Semblance value {} out of [0,1] range",
                    val
                );
            }
        }
    }

    // -- AGC tests --

    #[test]
    fn test_agc_normalizes_amplitude() {
        // A trace with varying amplitude should be equalised
        let mut trace = vec![0.0; 200];
        for i in 0..100 {
            trace[i] = 10.0 * (i as f64 * 0.1).sin();
        }
        for i in 100..200 {
            trace[i] = 0.1 * (i as f64 * 0.1).sin();
        }

        let result = agc(&trace, 50);
        assert_eq!(result.len(), 200);

        // Early and late RMS should be similar after AGC
        let rms_early: f64 = (result[25..75].iter().map(|x| x * x).sum::<f64>() / 50.0).sqrt();
        let rms_late: f64 =
            (result[125..175].iter().map(|x| x * x).sum::<f64>() / 50.0).sqrt();

        // They should be within an order of magnitude
        if rms_early > 0.01 && rms_late > 0.01 {
            let ratio = rms_early / rms_late;
            assert!(
                ratio > 0.1 && ratio < 10.0,
                "AGC did not normalise: ratio = {}",
                ratio
            );
        }
    }

    #[test]
    fn test_agc_empty() {
        let result = agc(&[], 10);
        assert!(result.is_empty());
    }

    #[test]
    fn test_agc_zero_window() {
        let trace = vec![1.0, 2.0, 3.0];
        let result = agc(&trace, 0);
        assert_eq!(result, trace);
    }

    #[test]
    fn test_agc_silent_trace() {
        let trace = vec![0.0; 100];
        let result = agc(&trace, 20);
        assert!(result.iter().all(|&x| x == 0.0));
    }

    // -- Bandpass filter tests --

    #[test]
    fn test_bandpass_preserves_length() {
        let trace: Vec<f64> = (0..500).map(|i| (i as f64 * 0.1).sin()).collect();
        let filtered = bandpass_filter(&trace, 1000.0, 10.0, 100.0);
        assert_eq!(filtered.len(), trace.len());
    }

    #[test]
    fn test_bandpass_empty() {
        let filtered = bandpass_filter(&[], 1000.0, 10.0, 100.0);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_bandpass_attenuates_dc() {
        // DC component should be attenuated by bandpass
        let trace = vec![5.0; 500];
        let filtered = bandpass_filter(&trace, 1000.0, 10.0, 100.0);
        let dc: f64 = filtered.iter().sum::<f64>() / filtered.len() as f64;
        // The mean should be significantly less than the input DC level of 5.0
        assert!(
            dc.abs() < 4.0,
            "Bandpass did not attenuate DC: mean = {}",
            dc
        );
    }

    #[test]
    fn test_bandpass_invalid_frequencies() {
        // low >= high should return original
        let trace = vec![1.0, 2.0, 3.0];
        let filtered = bandpass_filter(&trace, 1000.0, 200.0, 100.0);
        assert_eq!(filtered, trace);
    }

    // -- First break tests --

    #[test]
    fn test_first_break_basic() {
        let mut trace = vec![0.0; 100];
        trace[42] = 1.0;
        let fb = first_break_pick(&trace, 0.5);
        assert_eq!(fb, Some(42));
    }

    #[test]
    fn test_first_break_threshold() {
        let mut trace = vec![0.0; 100];
        trace[30] = 0.3;
        trace[50] = 1.0;
        // Threshold 0.5 should skip the small arrival
        let fb = first_break_pick(&trace, 0.5);
        assert_eq!(fb, Some(50));
    }

    #[test]
    fn test_first_break_empty() {
        assert_eq!(first_break_pick(&[], 0.5), None);
    }

    #[test]
    fn test_first_break_silent() {
        let trace = vec![0.0; 100];
        assert_eq!(first_break_pick(&trace, 0.5), None);
    }

    // -- Reflection coefficient tests --

    #[test]
    fn test_reflection_coefficient_identical_layers() {
        let r = compute_reflection_coefficient(3000.0, 2500.0, 3000.0, 2500.0);
        assert!(r.abs() < TOLERANCE, "Identical layers should give R=0");
    }

    #[test]
    fn test_reflection_coefficient_hard_boundary() {
        // Hard boundary: much higher impedance below
        let r = compute_reflection_coefficient(1500.0, 1000.0, 5000.0, 2700.0);
        assert!(r > 0.0, "Hard boundary should give positive R");
        assert!(r < 1.0, "R should be less than 1");
    }

    #[test]
    fn test_reflection_coefficient_free_surface() {
        // Free surface: Z2 ≈ 0
        let r = compute_reflection_coefficient(3000.0, 2500.0, 0.001, 0.001);
        assert!(
            (r - (-1.0)).abs() < 0.01,
            "Free surface should give R ≈ -1, got {}",
            r
        );
    }

    #[test]
    fn test_reflection_coefficient_antisymmetry() {
        let r12 = compute_reflection_coefficient(2000.0, 2000.0, 4000.0, 2500.0);
        let r21 = compute_reflection_coefficient(4000.0, 2500.0, 2000.0, 2000.0);
        assert!(
            (r12 + r21).abs() < TOLERANCE,
            "R12 + R21 should equal 0: {} + {} = {}",
            r12,
            r21,
            r12 + r21
        );
    }

    // -- Synthetic seismogram tests --

    #[test]
    fn test_synthetic_seismogram_basic() {
        let impedance_log = vec![
            (2000.0, 2000.0),
            (3000.0, 2500.0),
            (4000.0, 2700.0),
        ];
        let wavelet = ricker_wavelet(31, 30.0, 0.001);
        let seismogram = generate_synthetic_seismogram(&impedance_log, &wavelet);
        assert!(!seismogram.is_empty());
        // Should have non-zero values (reflections exist)
        assert!(seismogram.iter().any(|&x| x.abs() > 1e-10));
    }

    #[test]
    fn test_synthetic_seismogram_no_contrast() {
        // Uniform impedance => no reflections => zero seismogram
        let impedance_log = vec![
            (3000.0, 2500.0),
            (3000.0, 2500.0),
            (3000.0, 2500.0),
        ];
        let wavelet = ricker_wavelet(31, 30.0, 0.001);
        let seismogram = generate_synthetic_seismogram(&impedance_log, &wavelet);
        assert!(
            seismogram.iter().all(|&x| x.abs() < TOLERANCE),
            "Uniform impedance should give zero seismogram"
        );
    }

    #[test]
    fn test_synthetic_seismogram_single_layer() {
        // Only one layer => no interfaces => empty
        let impedance_log = vec![(3000.0, 2500.0)];
        let wavelet = ricker_wavelet(31, 30.0, 0.001);
        let seismogram = generate_synthetic_seismogram(&impedance_log, &wavelet);
        assert!(seismogram.is_empty());
    }

    #[test]
    fn test_synthetic_seismogram_empty_wavelet() {
        let impedance_log = vec![(2000.0, 2000.0), (3000.0, 2500.0)];
        let seismogram = generate_synthetic_seismogram(&impedance_log, &[]);
        assert!(seismogram.is_empty());
    }

    // -- ProcessedGather / process_gather tests --

    #[test]
    fn test_process_gather_basic() {
        let config = SeismicSurveyConfig {
            num_receivers: 4,
            receiver_spacing_m: 25.0,
            sample_rate_hz: 1000.0,
            source_offset_m: 50.0,
            max_time_s: 0.1,
        };
        let processor = SeismicProcessor::new(config);

        // Create simple synthetic traces with a reflection
        let n = 100;
        let wavelet = ricker_wavelet(21, 30.0, 0.001);
        let mut traces = Vec::new();
        for i in 0..4 {
            let offset = processor.offset(i);
            let t_arrival = ((0.05_f64).powi(2) + (offset / 3000.0).powi(2)).sqrt();
            let sample_arrival = (t_arrival * 1000.0) as usize;
            let mut trace = vec![0.0; n];
            for (j, &w) in wavelet.iter().enumerate() {
                let idx = sample_arrival + j;
                if idx < n {
                    trace[idx] = w;
                }
            }
            traces.push(trace);
        }

        let result = processor.process_gather(&traces);
        assert_eq!(result.stacked_trace.len(), n);
        assert_eq!(result.velocities.len(), n);
        assert_eq!(result.depth_axis_m.len(), n);
    }

    #[test]
    fn test_process_gather_empty() {
        let config = SeismicSurveyConfig::default();
        let processor = SeismicProcessor::new(config);
        let result = processor.process_gather(&[]);
        assert!(result.stacked_trace.is_empty());
    }

    #[test]
    fn test_processor_offset() {
        let config = SeismicSurveyConfig {
            num_receivers: 10,
            receiver_spacing_m: 25.0,
            sample_rate_hz: 1000.0,
            source_offset_m: 50.0,
            max_time_s: 2.0,
        };
        let processor = SeismicProcessor::new(config);
        assert!((processor.offset(0) - 50.0).abs() < TOLERANCE);
        assert!((processor.offset(1) - 75.0).abs() < TOLERANCE);
        assert!((processor.offset(3) - 125.0).abs() < TOLERANCE);
    }

    // -- convolve_same tests --

    #[test]
    fn test_convolve_same_impulse() {
        // Convolving with a centred impulse should return the original signal
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let impulse = vec![0.0, 0.0, 1.0, 0.0, 0.0]; // impulse at centre
        let result = convolve_same(&signal, &impulse);
        for i in 0..signal.len() {
            assert!(
                (result[i] - signal[i]).abs() < TOLERANCE,
                "Impulse convolution mismatch at {}",
                i
            );
        }
    }

    #[test]
    fn test_convolve_same_preserves_length() {
        let signal = vec![1.0; 100];
        let filter = vec![0.25; 7];
        let result = convolve_same(&signal, &filter);
        assert_eq!(result.len(), signal.len());
    }

    // -- Edge case: very short traces --

    #[test]
    fn test_nmo_single_sample() {
        let trace = vec![1.0];
        let corrected = nmo_correction(&trace, 100.0, 3000.0, 0.001);
        assert_eq!(corrected.len(), 1);
    }

    #[test]
    fn test_default_config() {
        let config = SeismicSurveyConfig::default();
        assert_eq!(config.num_receivers, 24);
        assert!((config.receiver_spacing_m - 25.0).abs() < TOLERANCE);
        assert!((config.sample_rate_hz - 1000.0).abs() < TOLERANCE);
    }
}

//! Acoustic Emission (AE) sensor signal processing for structural health monitoring.
//!
//! This module processes high-frequency acoustic emission signals for non-destructive
//! testing (NDT) and structural health monitoring (SHM). It implements:
//!
//! - **Hit detection** with HDT (Hit Definition Time), HLT (Hit Lockout Time),
//!   and PDT (Peak Definition Time) timing parameters
//! - **Parametric analysis** including amplitude, duration, rise time, threshold
//!   crossing counts, signal energy, and frequency centroid
//! - **Source localization** via time-of-arrival (TOA) differences in 1D and 2D
//! - **Wave velocity estimation** from known source positions
//! - **Felicity ratio** computation for damage assessment
//! - **Kaiser effect** detection for load history analysis
//!
//! # Example
//!
//! ```
//! use r4w_core::acoustic_emission_sensor::{AeConfig, AeProcessor};
//!
//! let config = AeConfig {
//!     sample_rate_hz: 1_000_000.0,
//!     threshold_v: 0.1,
//!     hdt_us: 200.0,
//!     hlt_us: 300.0,
//!     pdt_us: 100.0,
//! };
//! let processor = AeProcessor::new(config);
//!
//! // Generate a synthetic burst signal
//! let mut signal = vec![0.0f64; 2000];
//! for i in 100..200 {
//!     let t = (i - 100) as f64 / 100.0;
//!     signal[i] = 0.5 * (2.0 * std::f64::consts::PI * 5.0 * t).sin();
//! }
//!
//! let hits = processor.detect_hits(&signal);
//! assert!(!hits.is_empty());
//! assert!(hits[0].amplitude_v > 0.1);
//! ```

use std::f64::consts::PI;

/// Configuration parameters for acoustic emission hit detection.
///
/// These timing parameters control how the detector identifies and separates
/// individual AE events (hits) from a continuous waveform stream.
#[derive(Debug, Clone)]
pub struct AeConfig {
    /// Sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// Detection threshold in volts. Signal must exceed this to start a hit.
    pub threshold_v: f64,
    /// Hit Definition Time in microseconds. If the signal drops below threshold
    /// and stays below for longer than HDT, the hit ends.
    pub hdt_us: f64,
    /// Hit Lockout Time in microseconds. After a hit ends, the detector is
    /// disabled for this duration to avoid retriggering on reflections.
    pub hlt_us: f64,
    /// Peak Definition Time in microseconds. The time window after the first
    /// threshold crossing within which the peak amplitude must occur.
    pub pdt_us: f64,
}

impl Default for AeConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 1_000_000.0,
            threshold_v: 0.1,
            hdt_us: 200.0,
            hlt_us: 300.0,
            pdt_us: 100.0,
        }
    }
}

impl AeConfig {
    /// Convert a time in microseconds to a sample count at the configured sample rate.
    fn us_to_samples(&self, us: f64) -> usize {
        ((us * 1e-6) * self.sample_rate_hz).round() as usize
    }
}

/// A detected acoustic emission hit with all parametric features.
///
/// Each hit represents a single AE event extracted from the continuous waveform.
/// Parametric features are computed from the waveform segment spanning the hit.
#[derive(Debug, Clone, PartialEq)]
pub struct AeHit {
    /// Index of the first sample that crossed the threshold.
    pub start_sample: usize,
    /// Index of the last sample before the signal dropped below threshold
    /// for longer than HDT.
    pub end_sample: usize,
    /// Index of the sample with the peak absolute amplitude.
    pub peak_sample: usize,
    /// Peak absolute amplitude in volts.
    pub amplitude_v: f64,
    /// Duration of the hit in microseconds.
    pub duration_us: f64,
    /// Rise time from first threshold crossing to peak amplitude in microseconds.
    pub rise_time_us: f64,
    /// Number of positive-going threshold crossings during the hit.
    pub counts: usize,
    /// Signal energy (sum of squared amplitudes) over the hit duration.
    pub energy: f64,
    /// Frequency centroid (spectral first moment) in Hz.
    pub freq_centroid_hz: f64,
}

impl Default for AeHit {
    fn default() -> Self {
        Self {
            start_sample: 0,
            end_sample: 0,
            peak_sample: 0,
            amplitude_v: 0.0,
            duration_us: 0.0,
            rise_time_us: 0.0,
            counts: 0,
            energy: 0.0,
            freq_centroid_hz: 0.0,
        }
    }
}

/// Main acoustic emission signal processor.
///
/// Holds the detection configuration and provides methods for hit detection,
/// parametric analysis, source localization, and damage assessment.
#[derive(Debug, Clone)]
pub struct AeProcessor {
    /// Detection and timing configuration.
    pub config: AeConfig,
}

impl AeProcessor {
    /// Create a new AE processor with the given configuration.
    pub fn new(config: AeConfig) -> Self {
        Self { config }
    }

    /// Detect acoustic emission hits in a signal using threshold crossing
    /// with HDT/HLT/PDT timing.
    ///
    /// The algorithm:
    /// 1. Scan for first threshold crossing (start of hit).
    /// 2. Track peak amplitude within PDT window.
    /// 3. If signal stays below threshold for longer than HDT, end the hit.
    /// 4. After a hit ends, lock out for HLT before searching for the next hit.
    ///
    /// Returns a vector of detected hits with full parametric features.
    pub fn detect_hits(&self, signal: &[f64]) -> Vec<AeHit> {
        let threshold = self.config.threshold_v;
        let hdt_samples = self.config.us_to_samples(self.config.hdt_us);
        let hlt_samples = self.config.us_to_samples(self.config.hlt_us);
        let sample_rate = self.config.sample_rate_hz;

        let mut hits = Vec::new();
        let len = signal.len();
        let mut i = 0;

        while i < len {
            // Search for threshold crossing
            if signal[i].abs() >= threshold {
                let start_sample = i;
                let mut peak_sample = i;
                let mut peak_val = signal[i].abs();
                let mut below_count = 0;
                let mut end_sample = i;

                // Advance through the hit
                i += 1;
                while i < len {
                    let abs_val = signal[i].abs();

                    if abs_val >= threshold {
                        below_count = 0;
                        end_sample = i;
                        if abs_val > peak_val {
                            peak_val = abs_val;
                            peak_sample = i;
                        }
                    } else {
                        below_count += 1;
                        if below_count > hdt_samples {
                            break;
                        }
                    }
                    i += 1;
                }

                // Compute parametric features for this hit
                let hit = self.compute_hit_params(signal, start_sample, end_sample, peak_sample, sample_rate);
                hits.push(hit);

                // Apply HLT lockout
                i = end_sample + 1 + hlt_samples;
            } else {
                i += 1;
            }
        }

        hits
    }

    /// Compute all parametric features for a detected hit.
    fn compute_hit_params(
        &self,
        signal: &[f64],
        start_sample: usize,
        end_sample: usize,
        peak_sample: usize,
        sample_rate: f64,
    ) -> AeHit {
        let threshold = self.config.threshold_v;

        let duration_samples = end_sample.saturating_sub(start_sample) + 1;
        let duration_us = (duration_samples as f64 / sample_rate) * 1e6;

        let rise_samples = peak_sample.saturating_sub(start_sample);
        let rise_time_us = (rise_samples as f64 / sample_rate) * 1e6;

        let amplitude_v = signal[peak_sample].abs();

        // Count positive-going threshold crossings
        let mut counts = 0;
        let end = (end_sample + 1).min(signal.len());
        if start_sample < end && signal[start_sample] >= threshold {
            counts = 1; // first crossing
        }
        for j in (start_sample + 1)..end {
            if signal[j] >= threshold && signal[j - 1] < threshold {
                counts += 1;
            }
        }

        // Energy: sum of squared values
        let energy: f64 = signal[start_sample..end].iter().map(|&x| x * x).sum();

        // Frequency centroid of the hit segment
        let hit_signal = &signal[start_sample..end];
        let freq_centroid_hz = frequency_centroid(hit_signal, sample_rate);

        AeHit {
            start_sample,
            end_sample,
            peak_sample,
            amplitude_v,
            duration_us,
            rise_time_us,
            counts,
            energy,
            freq_centroid_hz,
        }
    }

    /// Perform parametric analysis on a signal segment defined by a hit.
    ///
    /// This recomputes all parametric features from the raw signal data,
    /// which is useful when hit boundaries were determined externally.
    pub fn parametric_analysis(&self, signal: &[f64], hit: &AeHit) -> AeHit {
        self.compute_hit_params(
            signal,
            hit.start_sample,
            hit.end_sample,
            hit.peak_sample,
            self.config.sample_rate_hz,
        )
    }
}

/// Compute the frequency centroid (spectral first moment) of a signal.
///
/// The frequency centroid is the amplitude-weighted average frequency:
///
/// ```text
/// f_c = sum(f_k * |X_k|) / sum(|X_k|)
/// ```
///
/// where X_k is the DFT of the signal and f_k is the frequency of bin k.
///
/// Uses a simple DFT (not FFT) since this is std-only. Only positive
/// frequencies (up to Nyquist) are considered.
///
/// Returns 0.0 if the signal is empty or has zero energy.
pub fn frequency_centroid(signal: &[f64], sample_rate: f64) -> f64 {
    let n = signal.len();
    if n == 0 {
        return 0.0;
    }

    let n_positive = n / 2 + 1; // bins from DC to Nyquist
    let mut weighted_sum = 0.0;
    let mut magnitude_sum = 0.0;

    for k in 0..n_positive {
        // DFT bin k
        let mut real = 0.0;
        let mut imag = 0.0;
        for (idx, &s) in signal.iter().enumerate() {
            let angle = 2.0 * PI * (k as f64) * (idx as f64) / (n as f64);
            real += s * angle.cos();
            imag -= s * angle.sin();
        }
        let magnitude = (real * real + imag * imag).sqrt();
        let freq = (k as f64) * sample_rate / (n as f64);

        weighted_sum += freq * magnitude;
        magnitude_sum += magnitude;
    }

    if magnitude_sum < 1e-30 {
        0.0
    } else {
        weighted_sum / magnitude_sum
    }
}

/// Localize an AE source in 1D using time-of-arrival differences.
///
/// Given sensor positions along a line and their measured arrival times,
/// estimates the source position using a least-squares approach.
///
/// For two sensors, the solution is exact:
/// ```text
/// x_source = (x1 + x2) / 2 + velocity * (t1 - t2) / 2
/// ```
///
/// For more sensors, uses a least-squares approach averaging pairwise estimates.
///
/// # Arguments
/// * `toa_us` - Time of arrival at each sensor in microseconds
/// * `positions_m` - Position of each sensor in meters
/// * `velocity_mps` - Wave propagation velocity in m/s
///
/// # Panics
/// Panics if `toa_us` and `positions_m` have different lengths or fewer than 2 elements.
pub fn localize_1d(toa_us: &[f64], positions_m: &[f64], velocity_mps: f64) -> f64 {
    assert_eq!(toa_us.len(), positions_m.len(), "TOA and positions must have same length");
    assert!(toa_us.len() >= 2, "Need at least 2 sensors for localization");

    let n = toa_us.len();

    if n == 2 {
        // Exact solution for two sensors
        let mid = (positions_m[0] + positions_m[1]) / 2.0;
        let dt_s = (toa_us[0] - toa_us[1]) * 1e-6;
        mid + velocity_mps * dt_s / 2.0
    } else {
        // Average pairwise estimates
        let mut sum = 0.0;
        let mut count = 0.0;
        for i in 0..n {
            for j in (i + 1)..n {
                let mid = (positions_m[i] + positions_m[j]) / 2.0;
                let dt_s = (toa_us[i] - toa_us[j]) * 1e-6;
                sum += mid + velocity_mps * dt_s / 2.0;
                count += 1.0;
            }
        }
        sum / count
    }
}

/// Localize an AE source in 2D using time-of-arrival differences.
///
/// Uses TDOA (Time Difference of Arrival) linearization. With the first
/// sensor as reference, forms a system of linear equations and solves
/// using least-squares (normal equations).
///
/// For 3 sensors, the system is exactly determined. For more sensors,
/// the least-squares solution minimizes residual error.
///
/// # Arguments
/// * `toa_us` - Time of arrival at each sensor in microseconds
/// * `positions` - (x, y) position of each sensor in meters
/// * `velocity_mps` - Wave propagation velocity in m/s
///
/// # Panics
/// Panics if arrays have different lengths or fewer than 3 elements.
pub fn localize_2d(
    toa_us: &[f64],
    positions: &[(f64, f64)],
    velocity_mps: f64,
) -> (f64, f64) {
    assert_eq!(toa_us.len(), positions.len(), "TOA and positions must have same length");
    assert!(toa_us.len() >= 3, "Need at least 3 sensors for 2D localization");

    let n = toa_us.len();

    // Reference sensor is index 0
    let (x0, y0) = positions[0];
    let t0 = toa_us[0] * 1e-6; // convert to seconds

    // Linearized TDOA: for each sensor i vs reference 0:
    //   d_i - d_0 = v * (t_i - t_0)
    // Squaring and subtracting gives linear equations.

    // Initial estimate: centroid of sensors weighted by inverse TOA
    let mut x_est = 0.0;
    let mut y_est = 0.0;
    let mut w_sum = 0.0;
    for i in 0..n {
        let w = 1.0 / (1.0 + toa_us[i]);
        x_est += positions[i].0 * w;
        y_est += positions[i].1 * w;
        w_sum += w;
    }
    x_est /= w_sum;
    y_est /= w_sum;

    // Iterative linearization (Gauss-Newton style)
    for _iter in 0..20 {
        // Compute distance from current estimate to reference sensor
        let d0_est = ((x_est - x0).powi(2) + (y_est - y0).powi(2)).sqrt();

        // Build the linear system A * [x, y]^T = b
        let rows = n - 1;
        if rows < 2 {
            break;
        }

        // A: rows x 2, b: rows x 1 => accumulate A^T A (2x2) and A^T b (2x1)
        let mut ata = [0.0f64; 4]; // 2x2 in row-major
        let mut atb = [0.0f64; 2]; // 2x1

        for i in 1..n {
            let (xi, yi) = positions[i];
            let ti = toa_us[i] * 1e-6;
            let dt = ti - t0;
            let ri = d0_est + velocity_mps * dt;

            let ax = 2.0 * (xi - x0);
            let ay = 2.0 * (yi - y0);
            let bi = (xi * xi + yi * yi) - (x0 * x0 + y0 * y0) - (ri * ri - d0_est * d0_est);

            // Accumulate A^T A
            ata[0] += ax * ax;
            ata[1] += ax * ay;
            ata[2] += ay * ax;
            ata[3] += ay * ay;

            // Accumulate A^T b
            atb[0] += ax * bi;
            atb[1] += ay * bi;
        }

        // Solve 2x2 system: [ata] * [x, y] = [atb]
        let det = ata[0] * ata[3] - ata[1] * ata[2];
        if det.abs() < 1e-30 {
            break; // Singular, keep current estimate
        }

        let new_x = (ata[3] * atb[0] - ata[1] * atb[1]) / det;
        let new_y = (-ata[2] * atb[0] + ata[0] * atb[1]) / det;

        let dx = new_x - x_est;
        let dy = new_y - y_est;

        x_est = new_x;
        y_est = new_y;

        if dx * dx + dy * dy < 1e-12 {
            break;
        }
    }

    (x_est, y_est)
}

/// Estimate wave propagation velocity from a known source distance and TOA difference.
///
/// ```text
/// velocity = distance / delta_t
/// ```
///
/// # Arguments
/// * `distance_m` - Known distance between two sensors (or sensor and source) in meters
/// * `toa_diff_us` - Time difference of arrival in microseconds
///
/// Returns velocity in m/s. Returns `f64::INFINITY` if `toa_diff_us` is zero.
pub fn estimate_velocity(distance_m: f64, toa_diff_us: f64) -> f64 {
    if toa_diff_us.abs() < 1e-30 {
        return f64::INFINITY;
    }
    distance_m / (toa_diff_us * 1e-6)
}

/// Compute the Felicity ratio for damage assessment.
///
/// The Felicity ratio is the ratio of the load at which AE activity resumes
/// on reloading to the previously applied maximum load. A ratio less than 1.0
/// indicates structural damage (the material emits AE before reaching the
/// previous load level).
///
/// ```text
/// FR = load_at_first_AE_on_reload / previous_max_load
/// ```
///
/// # Arguments
/// * `previous_load_hits` - Loads (in arbitrary units) at which hits occurred during
///   the previous loading cycle
/// * `current_load_hits` - Loads at which hits occurred during the current loading cycle
///
/// Returns 1.0 if either slice is empty (no damage indication without data).
pub fn felicity_ratio(previous_load_hits: &[f64], current_load_hits: &[f64]) -> f64 {
    if previous_load_hits.is_empty() || current_load_hits.is_empty() {
        return 1.0;
    }

    // Previous max load
    let prev_max = previous_load_hits
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    // First AE hit load in current cycle (minimum load at which AE resumed)
    let current_first = current_load_hits
        .iter()
        .cloned()
        .fold(f64::INFINITY, f64::min);

    if prev_max.abs() < 1e-30 {
        return 1.0;
    }

    current_first / prev_max
}

/// Check for the Kaiser effect in a load history.
///
/// The Kaiser effect states that AE should not occur until the previous
/// maximum stress is exceeded. If AE occurs below the previous maximum,
/// the Kaiser effect is violated, indicating damage or material degradation.
///
/// # Arguments
/// * `load_history` - Sequential load values applied to the structure
/// * `hit_loads` - Load levels at which AE hits were detected
///
/// Returns `true` if the Kaiser effect holds (no AE below previous max),
/// `false` if it is violated (AE occurs below previously reached load).
pub fn kaiser_effect(load_history: &[f64], hit_loads: &[f64]) -> bool {
    if load_history.is_empty() || hit_loads.is_empty() {
        return true; // No data means no violation
    }

    // Find the previous maximum load from history
    let prev_max = load_history
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    // Check if any AE hit occurred below the previous max
    // Kaiser effect holds if ALL hits occur at or above previous max
    hit_loads.iter().all(|&load| load >= prev_max - 1e-10)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_processor() -> AeProcessor {
        let config = AeConfig {
            sample_rate_hz: 1_000_000.0,
            threshold_v: 0.1,
            hdt_us: 50.0,
            hlt_us: 100.0,
            pdt_us: 50.0,
        };
        AeProcessor::new(config)
    }

    /// Generate a simple burst: a sine tone with parabolic envelope.
    fn make_burst(
        center: usize,
        half_width: usize,
        amplitude: f64,
        freq_hz: f64,
        sample_rate: f64,
        total_len: usize,
    ) -> Vec<f64> {
        let mut signal = vec![0.0; total_len];
        let start = center.saturating_sub(half_width);
        let end = (center + half_width).min(total_len);
        for i in start..end {
            let env = 1.0 - ((i as f64 - center as f64) / half_width as f64).powi(2);
            let env = env.max(0.0);
            let t = i as f64 / sample_rate;
            signal[i] = amplitude * env * (2.0 * PI * freq_hz * t).sin();
        }
        signal
    }

    #[test]
    fn test_config_default() {
        let config = AeConfig::default();
        assert_eq!(config.sample_rate_hz, 1_000_000.0);
        assert_eq!(config.threshold_v, 0.1);
        assert_eq!(config.hdt_us, 200.0);
        assert_eq!(config.hlt_us, 300.0);
        assert_eq!(config.pdt_us, 100.0);
    }

    #[test]
    fn test_us_to_samples() {
        let config = AeConfig {
            sample_rate_hz: 1_000_000.0,
            ..Default::default()
        };
        assert_eq!(config.us_to_samples(100.0), 100);
        assert_eq!(config.us_to_samples(1.0), 1);
        assert_eq!(config.us_to_samples(0.0), 0);
    }

    #[test]
    fn test_detect_no_hits_below_threshold() {
        let proc = make_processor();
        let signal = vec![0.05; 1000];
        let hits = proc.detect_hits(&signal);
        assert!(hits.is_empty());
    }

    #[test]
    fn test_detect_single_burst() {
        let proc = make_processor();
        let signal = make_burst(500, 100, 0.5, 100_000.0, 1_000_000.0, 2000);
        let hits = proc.detect_hits(&signal);
        assert_eq!(hits.len(), 1);
        assert!(hits[0].amplitude_v > 0.1);
        assert!(hits[0].amplitude_v <= 0.5 + 0.01);
        assert!(hits[0].start_sample < 500);
        assert!(hits[0].end_sample > 500);
    }

    #[test]
    fn test_detect_two_separated_bursts() {
        let proc = make_processor();
        let mut signal = make_burst(300, 80, 0.5, 100_000.0, 1_000_000.0, 3000);
        let burst2 = make_burst(2000, 80, 0.4, 150_000.0, 1_000_000.0, 3000);
        for i in 0..signal.len() {
            signal[i] += burst2[i];
        }
        let hits = proc.detect_hits(&signal);
        assert_eq!(hits.len(), 2);
        assert!(hits[0].start_sample < hits[1].start_sample);
    }

    #[test]
    fn test_hit_duration() {
        let proc = make_processor();
        let signal = make_burst(500, 100, 0.8, 50_000.0, 1_000_000.0, 2000);
        let hits = proc.detect_hits(&signal);
        assert_eq!(hits.len(), 1);
        // Duration should be positive and reasonable
        assert!(hits[0].duration_us > 0.0);
        assert!(hits[0].duration_us < 500.0);
    }

    #[test]
    fn test_hit_rise_time() {
        let proc = make_processor();
        let signal = make_burst(500, 100, 0.8, 50_000.0, 1_000_000.0, 2000);
        let hits = proc.detect_hits(&signal);
        assert_eq!(hits.len(), 1);
        assert!(hits[0].rise_time_us >= 0.0);
        assert!(hits[0].rise_time_us <= hits[0].duration_us);
    }

    #[test]
    fn test_hit_counts() {
        let proc = make_processor();
        // A sine burst will cross the threshold multiple times
        let signal = make_burst(500, 100, 0.8, 50_000.0, 1_000_000.0, 2000);
        let hits = proc.detect_hits(&signal);
        assert_eq!(hits.len(), 1);
        assert!(hits[0].counts >= 1);
    }

    #[test]
    fn test_hit_energy_positive() {
        let proc = make_processor();
        let signal = make_burst(500, 100, 0.5, 100_000.0, 1_000_000.0, 2000);
        let hits = proc.detect_hits(&signal);
        assert_eq!(hits.len(), 1);
        assert!(hits[0].energy > 0.0);
    }

    #[test]
    fn test_parametric_analysis_recomputes() {
        let proc = make_processor();
        let signal = make_burst(500, 100, 0.6, 80_000.0, 1_000_000.0, 2000);
        let hits = proc.detect_hits(&signal);
        assert_eq!(hits.len(), 1);

        let recomputed = proc.parametric_analysis(&signal, &hits[0]);
        assert!((recomputed.amplitude_v - hits[0].amplitude_v).abs() < 1e-10);
        assert!((recomputed.energy - hits[0].energy).abs() < 1e-10);
        assert_eq!(recomputed.counts, hits[0].counts);
    }

    #[test]
    fn test_frequency_centroid_dc() {
        // A constant (DC) signal should have centroid near 0 Hz
        let signal = vec![1.0; 64];
        let centroid = frequency_centroid(&signal, 1000.0);
        assert!(centroid < 50.0);
    }

    #[test]
    fn test_frequency_centroid_pure_tone() {
        // Generate a pure tone at 250 Hz, sampled at 1000 Hz
        let sample_rate = 1000.0;
        let freq = 250.0;
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect();
        let centroid = frequency_centroid(&signal, sample_rate);
        assert!((centroid - freq).abs() < 5.0, "Centroid {} not near {}", centroid, freq);
    }

    #[test]
    fn test_frequency_centroid_empty() {
        assert_eq!(frequency_centroid(&[], 1000.0), 0.0);
    }

    #[test]
    fn test_localize_1d_two_sensors() {
        // Source at 0.3 m, sensors at 0.0 and 1.0 m, velocity 5000 m/s
        let velocity = 5000.0;
        let source = 0.3;
        let positions: [f64; 2] = [0.0, 1.0];
        let d0 = (source - positions[0]).abs();
        let d1 = (source - positions[1]).abs();
        let toa = [d0 / velocity * 1e6, d1 / velocity * 1e6];

        let est = localize_1d(&toa, &positions, velocity);
        assert!((est - source).abs() < 0.01, "Estimated {}, expected {}", est, source);
    }

    #[test]
    fn test_localize_1d_three_sensors() {
        let velocity = 3000.0;
        let source = 0.5;
        let positions: [f64; 3] = [0.0, 0.4, 1.0];
        let toa: Vec<f64> = positions
            .iter()
            .map(|&p| (source - p).abs() / velocity * 1e6)
            .collect();

        let est = localize_1d(&toa, &positions, velocity);
        assert!((est - source).abs() < 0.05, "Estimated {}, expected {}", est, source);
    }

    #[test]
    fn test_localize_2d_three_sensors() {
        let velocity = 5000.0;
        let source = (0.3, 0.4);
        let positions: [(f64, f64); 3] = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)];
        let toa: Vec<f64> = positions
            .iter()
            .map(|&(x, y)| {
                let d = ((source.0 - x).powi(2) + (source.1 - y).powi(2)).sqrt();
                d / velocity * 1e6
            })
            .collect();

        let (ex, ey) = localize_2d(&toa, &positions, velocity);
        let err = ((ex - source.0).powi(2) + (ey - source.1).powi(2)).sqrt();
        assert!(err < 0.05, "Estimated ({}, {}), expected {:?}, error {}", ex, ey, source, err);
    }

    #[test]
    fn test_localize_2d_four_sensors() {
        let velocity = 3000.0;
        let source = (0.6, 0.2);
        let positions: [(f64, f64); 4] = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let toa: Vec<f64> = positions
            .iter()
            .map(|&(x, y)| {
                let d = ((source.0 - x).powi(2) + (source.1 - y).powi(2)).sqrt();
                d / velocity * 1e6
            })
            .collect();

        let (ex, ey) = localize_2d(&toa, &positions, velocity);
        let err = ((ex - source.0).powi(2) + (ey - source.1).powi(2)).sqrt();
        assert!(err < 0.05, "Estimated ({}, {}), expected {:?}, error {}", ex, ey, source, err);
    }

    #[test]
    fn test_estimate_velocity() {
        let v = estimate_velocity(1.0, 200.0);
        // 1 m / 200 us = 5000 m/s
        assert!((v - 5000.0).abs() < 0.1);
    }

    #[test]
    fn test_estimate_velocity_zero_time() {
        let v = estimate_velocity(1.0, 0.0);
        assert!(v.is_infinite());
    }

    #[test]
    fn test_felicity_ratio_no_damage() {
        // AE resumes at same load as previous max => FR = 1.0
        let prev = vec![100.0, 200.0, 300.0];
        let curr = vec![300.0, 350.0];
        let fr = felicity_ratio(&prev, &curr);
        assert!((fr - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_felicity_ratio_damage() {
        // AE resumes at 150 but previous max was 300 => FR = 0.5
        let prev = vec![100.0, 200.0, 300.0];
        let curr = vec![150.0, 200.0, 300.0];
        let fr = felicity_ratio(&prev, &curr);
        assert!((fr - 0.5).abs() < 0.01, "FR = {}, expected 0.5", fr);
    }

    #[test]
    fn test_felicity_ratio_empty() {
        assert_eq!(felicity_ratio(&[], &[100.0]), 1.0);
        assert_eq!(felicity_ratio(&[100.0], &[]), 1.0);
    }

    #[test]
    fn test_kaiser_effect_holds() {
        let load_history = vec![100.0, 200.0, 300.0];
        let hit_loads = vec![300.0, 350.0, 400.0];
        assert!(kaiser_effect(&load_history, &hit_loads));
    }

    #[test]
    fn test_kaiser_effect_violated() {
        let load_history = vec![100.0, 200.0, 300.0];
        let hit_loads = vec![250.0, 300.0, 350.0]; // 250 < 300
        assert!(!kaiser_effect(&load_history, &hit_loads));
    }

    #[test]
    fn test_kaiser_effect_empty() {
        assert!(kaiser_effect(&[], &[100.0]));
        assert!(kaiser_effect(&[100.0], &[]));
    }

    #[test]
    fn test_ae_hit_default() {
        let hit = AeHit::default();
        assert_eq!(hit.start_sample, 0);
        assert_eq!(hit.amplitude_v, 0.0);
        assert_eq!(hit.counts, 0);
        assert_eq!(hit.energy, 0.0);
    }

    #[test]
    fn test_hlt_lockout_prevents_retrigger() {
        // Two bursts close together - HLT should skip the second
        let config = AeConfig {
            sample_rate_hz: 1_000_000.0,
            threshold_v: 0.1,
            hdt_us: 20.0,
            hlt_us: 500.0, // Very long lockout
            pdt_us: 20.0,
        };
        let proc = AeProcessor::new(config);

        let mut signal = make_burst(200, 50, 0.5, 100_000.0, 1_000_000.0, 1000);
        let burst2 = make_burst(500, 50, 0.5, 100_000.0, 1_000_000.0, 1000);
        for i in 0..signal.len() {
            signal[i] += burst2[i];
        }

        let hits = proc.detect_hits(&signal);
        // With 500 us HLT (500 samples at 1 MHz), second burst at sample 500
        // should be locked out
        assert_eq!(hits.len(), 1, "HLT should prevent second hit detection");
    }
}

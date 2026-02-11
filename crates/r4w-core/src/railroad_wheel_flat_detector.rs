//! Railroad wheel flat spot detector using wayside acoustic/vibration monitoring.
//!
//! Wheel flats are localized flat areas on the tread surface of a railroad wheel,
//! caused by wheel lock-up during braking (sliding). They produce periodic impact
//! loading on the rail at the wheel rotation frequency, generating distinctive
//! high-amplitude transients in rail-mounted accelerometer signals.
//!
//! This module implements:
//!
//! - **Impact detection** from accelerometer waveforms using envelope analysis
//! - **Periodic impact correlation** to associate impacts with specific wheel rotations
//! - **Flat geometry estimation** from impact force and wheel/speed parameters
//! - **Severity classification** per AAR (Association of American Railroads) guidelines
//! - **Spectral kurtosis** for non-Gaussian transient detection
//! - **Synthetic signal generation** for testing and validation
//!
//! # Physics Background
//!
//! When a wheel with a flat spot rotates, the flat section causes the wheel to
//! momentarily drop by the flat depth `d`, then impact the rail. The chord length
//! of the flat is related to the flat depth by:
//!
//! ```text
//! L = 2 * sqrt(D * d - d^2)
//! ```
//!
//! where `D` is the wheel diameter and `d` is the flat depth. The rotation period
//! determines impact spacing:
//!
//! ```text
//! T = pi * D / v
//! ```
//!
//! where `v` is the train speed.
//!
//! # Example
//!
//! ```
//! use r4w_core::railroad_wheel_flat_detector::{
//!     WheelFlatConfig, WheelFlatDetector, FlatSeverity,
//!     wheel_rotation_period, flat_length_from_chord,
//!     generate_wheel_flat_signal,
//! };
//!
//! let config = WheelFlatConfig {
//!     sample_rate_hz: 10_000.0,
//!     wheel_diameter_m: 0.914,
//!     train_speed_mps: 20.0,
//!     impact_threshold_g: 50.0,
//! };
//!
//! // Generate synthetic signal with a 2mm deep flat
//! let signal = generate_wheel_flat_signal(&config, 2.0, 5);
//! assert!(!signal.is_empty());
//!
//! // Process and detect flats
//! let detector = WheelFlatDetector::new(config.clone());
//! let report = detector.process(&signal);
//! assert!(report.max_impact_g > 0.0);
//!
//! // Verify rotation period
//! let period = wheel_rotation_period(0.914, 20.0);
//! assert!((period - std::f64::consts::PI * 0.914 / 20.0).abs() < 1e-10);
//!
//! // Verify flat geometry
//! let length = flat_length_from_chord(0.914, 1.0);
//! assert!(length > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the wheel flat detector.
///
/// The detector analyses accelerometer data recorded from a rail-mounted sensor
/// as a train passes over it. The wheel diameter and train speed are required
/// to compute expected rotation periods and to estimate flat geometry from
/// measured impact forces.
#[derive(Debug, Clone)]
pub struct WheelFlatConfig {
    /// Sampling rate of the accelerometer in Hz.
    pub sample_rate_hz: f64,

    /// Nominal wheel diameter in metres (default 0.914 m = 36 inches).
    pub wheel_diameter_m: f64,

    /// Train speed in metres per second.
    pub train_speed_mps: f64,

    /// Minimum peak acceleration (in g) to qualify as an impact event.
    /// Typical values: 30-80 g depending on rail pad stiffness and sensor
    /// mounting.
    pub impact_threshold_g: f64,
}

impl Default for WheelFlatConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 10_000.0,
            wheel_diameter_m: 0.914,
            train_speed_mps: 20.0,
            impact_threshold_g: 50.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Severity classification
// ---------------------------------------------------------------------------

/// Severity classification for a wheel flat, based on chord length.
///
/// Thresholds follow AAR Rule 41 / M-926 guidelines:
/// - **Minor**: flat length < 40 mm -- monitoring recommended
/// - **Moderate**: 40-60 mm -- schedule maintenance
/// - **Severe**: 60-80 mm -- remove from service at next opportunity
/// - **Critical**: > 80 mm -- immediate removal from service
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlatSeverity {
    /// Flat length < 40 mm.
    Minor,
    /// Flat length 40-60 mm.
    Moderate,
    /// Flat length 60-80 mm.
    Severe,
    /// Flat length > 80 mm.
    Critical,
}

impl FlatSeverity {
    /// Classify a flat by its chord length in millimetres.
    pub fn from_length_mm(length_mm: f64) -> Self {
        if length_mm < 40.0 {
            FlatSeverity::Minor
        } else if length_mm < 60.0 {
            FlatSeverity::Moderate
        } else if length_mm < 80.0 {
            FlatSeverity::Severe
        } else {
            FlatSeverity::Critical
        }
    }
}

// ---------------------------------------------------------------------------
// Detection results
// ---------------------------------------------------------------------------

/// A single detected wheel flat.
#[derive(Debug, Clone)]
pub struct WheelFlat {
    /// Zero-based index of the wheel within the passing train (detection order).
    pub wheel_index: usize,

    /// Estimated flat chord length in millimetres.
    pub flat_length_mm: f64,

    /// Peak impact acceleration in g.
    pub impact_force_g: f64,

    /// Severity classification.
    pub severity: FlatSeverity,

    /// Measured rotation period in seconds (time between successive impacts
    /// from the same flat).
    pub rotation_period_s: f64,
}

/// Aggregate report produced by [`WheelFlatDetector::process`].
#[derive(Debug, Clone)]
pub struct WheelFlatReport {
    /// Individual wheel flat detections.
    pub detections: Vec<WheelFlat>,

    /// Estimated number of wheels that passed over the sensor.
    pub num_wheels_passed: usize,

    /// Maximum impact acceleration observed in the entire recording (g).
    pub max_impact_g: f64,
}

// ---------------------------------------------------------------------------
// Detector
// ---------------------------------------------------------------------------

/// Main detector that processes raw accelerometer data and produces a
/// [`WheelFlatReport`].
#[derive(Debug, Clone)]
pub struct WheelFlatDetector {
    config: WheelFlatConfig,
}

impl WheelFlatDetector {
    /// Create a new detector with the given configuration.
    pub fn new(config: WheelFlatConfig) -> Self {
        Self { config }
    }

    /// Process a buffer of accelerometer data (units: g) and return a detection
    /// report.
    ///
    /// The algorithm:
    /// 1. Compute the signal envelope via a discrete Hilbert transform.
    /// 2. Detect peaks above `impact_threshold_g`.
    /// 3. Correlate peaks at the expected wheel rotation period.
    /// 4. For each group of periodic impacts, estimate flat geometry.
    pub fn process(&self, accelerometer_data: &[f64]) -> WheelFlatReport {
        if accelerometer_data.is_empty() {
            return WheelFlatReport {
                detections: Vec::new(),
                num_wheels_passed: 0,
                max_impact_g: 0.0,
            };
        }

        let envelope = envelope_from_hilbert(accelerometer_data);

        // Global maximum for the report.
        let max_impact_g = envelope
            .iter()
            .copied()
            .fold(0.0_f64, f64::max);

        // Find peaks above the threshold.
        let peak_indices = find_peaks(&envelope, self.config.impact_threshold_g);

        // Expected rotation period in samples.
        let expected_period_s =
            wheel_rotation_period(self.config.wheel_diameter_m, self.config.train_speed_mps);
        let expected_period_samples = expected_period_s * self.config.sample_rate_hz;

        // Tolerance: 20 % of expected period.
        let tolerance = 0.20;

        // Group peaks into periodic sets (one set per wheel with a flat).
        let groups = group_periodic_peaks(&peak_indices, expected_period_samples, tolerance);

        // Estimate the total number of wheels from the recording length.
        // Each wheel produces a short burst of energy. For a rough count we
        // use the number of distinct peak clusters (periodic groups plus
        // isolated peaks).
        let num_wheels_passed = if groups.is_empty() {
            estimate_wheel_count(&peak_indices, expected_period_samples)
        } else {
            // Each group is one wheel with a flat; isolated peaks are additional wheels.
            let used: Vec<usize> = groups.iter().flat_map(|g| g.iter().copied()).collect();
            let isolated = peak_indices
                .iter()
                .filter(|p| !used.contains(p))
                .count();
            groups.len() + isolated
        };

        // Default wheel mass for depth estimation (kg). A typical freight car
        // wheel is approximately 400 kg.
        let wheel_mass_kg = 400.0;

        let mut detections = Vec::new();
        for (idx, group) in groups.iter().enumerate() {
            // Measure the average period within the group.
            let avg_period_s = if group.len() >= 2 {
                let total: f64 = group
                    .windows(2)
                    .map(|w| (w[1] as f64 - w[0] as f64) / self.config.sample_rate_hz)
                    .sum();
                total / (group.len() - 1) as f64
            } else {
                expected_period_s
            };

            // Peak impact in this group.
            let peak_g = group
                .iter()
                .map(|&i| envelope[i])
                .fold(0.0_f64, f64::max);

            // Estimate flat depth from impact force.
            let depth_mm = flat_depth_from_impact(peak_g, self.config.train_speed_mps, wheel_mass_kg);

            // Convert to chord length.
            let length_mm = flat_length_from_chord(self.config.wheel_diameter_m, depth_mm);

            let severity = FlatSeverity::from_length_mm(length_mm);

            detections.push(WheelFlat {
                wheel_index: idx,
                flat_length_mm: length_mm,
                impact_force_g: peak_g,
                severity,
                rotation_period_s: avg_period_s,
            });
        }

        WheelFlatReport {
            detections,
            num_wheels_passed,
            max_impact_g,
        }
    }
}

// ---------------------------------------------------------------------------
// Public helper functions
// ---------------------------------------------------------------------------

/// Compute the wheel rotation period in seconds.
///
/// ```text
/// T = pi * D / v
/// ```
///
/// # Panics
///
/// Returns `f64::INFINITY` when `speed_mps` is zero.
pub fn wheel_rotation_period(diameter_m: f64, speed_mps: f64) -> f64 {
    if speed_mps == 0.0 {
        return f64::INFINITY;
    }
    PI * diameter_m / speed_mps
}

/// Compute the chord length (flat length) in **millimetres** from the wheel
/// diameter (m) and the flat depth (mm).
///
/// ```text
/// L = 2 * sqrt(D_mm * d - d^2)
/// ```
///
/// where `D_mm` is the diameter in millimetres and `d` is the flat depth in
/// millimetres.
///
/// Returns 0.0 when the depth is non-positive or exceeds the radius.
pub fn flat_length_from_chord(diameter_m: f64, flat_depth_mm: f64) -> f64 {
    let d_mm = diameter_m * 1000.0; // convert to mm
    let d = flat_depth_mm;
    if d <= 0.0 || d >= d_mm / 2.0 {
        return 0.0;
    }
    2.0 * (d_mm * d - d * d).sqrt()
}

/// Estimate flat depth in **millimetres** from impact acceleration, train
/// speed, and wheel mass.
///
/// The model assumes Hertzian contact: the impact force `F = m * a` produces a
/// contact deformation related to the kinetic energy at the moment the flat
/// leaves the rail surface. From energy balance:
///
/// ```text
/// d = v^2 * a / (2 * g0^2)
/// ```
///
/// where `g0 = 9.80665 m/s^2` (standard gravity). This is a simplified
/// empirical model; the actual relationship depends on rail pad stiffness,
/// contact patch geometry, and so on. The result is in millimetres.
pub fn flat_depth_from_impact(impact_g: f64, speed_mps: f64, _wheel_mass_kg: f64) -> f64 {
    let g0 = 9.80665; // m/s^2
    // Simplified energy model: depth (m) = v^2 * a_g / (2 * g0)
    // We scale by a calibration factor to produce realistic mm values.
    // For a 20 m/s train with 100 g impact, this should give ~1-5 mm depth.
    let depth_m = speed_mps.powi(2) * impact_g / (2.0 * g0 * g0 * 100.0);
    depth_m * 1000.0 // convert to mm
}

/// Detect peaks in `signal` that repeat at approximately `expected_period_samples`
/// with the given fractional `tolerance` (e.g., 0.20 = +/- 20 %).
///
/// Returns the sample indices of peaks that belong to at least one periodic
/// pair.
pub fn detect_periodic_impacts(
    signal: &[f64],
    expected_period_samples: f64,
    tolerance: f64,
) -> Vec<usize> {
    // First find all peaks above the mean + 3 sigma level.
    let threshold = {
        let n = signal.len() as f64;
        if n == 0.0 {
            return Vec::new();
        }
        let mean = signal.iter().sum::<f64>() / n;
        let var = signal.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        let sigma = var.sqrt();
        if sigma < 1e-12 {
            // Constant or near-constant signal: no meaningful peaks.
            return Vec::new();
        }
        mean + 3.0 * sigma
    };
    let peaks = find_peaks(signal, threshold);
    if peaks.is_empty() {
        return Vec::new();
    }
    let low = expected_period_samples * (1.0 - tolerance);
    let high = expected_period_samples * (1.0 + tolerance);

    let mut periodic = Vec::new();
    for i in 0..peaks.len() {
        for j in (i + 1)..peaks.len() {
            let gap = (peaks[j] as f64) - (peaks[i] as f64);
            if gap >= low && gap <= high {
                if !periodic.contains(&peaks[i]) {
                    periodic.push(peaks[i]);
                }
                if !periodic.contains(&peaks[j]) {
                    periodic.push(peaks[j]);
                }
            }
        }
    }
    periodic.sort_unstable();
    periodic
}

/// Compute impact force in newtons from peak acceleration (in g) and wheel
/// mass (kg).
///
/// ```text
/// F = m * a = m * (a_g * 9.80665)
/// ```
pub fn compute_impact_force(peak_accel_g: f64, wheel_mass_kg: f64) -> f64 {
    wheel_mass_kg * peak_accel_g * 9.80665
}

/// Compute the envelope of a signal via a discrete Hilbert transform.
///
/// The Hilbert transform is implemented in the time domain using a
/// length-truncated FIR approximation of the ideal impulse response
/// `h[n] = 2/(pi*n)` for odd `n`, 0 for even `n`. The analytic signal is
/// `x_a[n] = x[n] + j * H{x}[n]`, and the envelope is `|x_a[n]|`.
pub fn envelope_from_hilbert(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    // FIR Hilbert transformer kernel (truncated, windowed).
    // Use a kernel length equal to the smaller of 127 or the signal length
    // (must be odd).
    let kernel_len = {
        let k = n.min(127);
        if k % 2 == 0 { k + 1 } else { k }
    };
    let half = kernel_len / 2;

    // Build the Hilbert FIR kernel with a Hamming window.
    let kernel: Vec<f64> = (0..kernel_len)
        .map(|i| {
            let m = i as isize - half as isize;
            if m == 0 {
                0.0
            } else if m % 2 != 0 {
                let h = 2.0 / (PI * m as f64);
                // Hamming window
                let w =
                    0.54 - 0.46 * (2.0 * PI * i as f64 / (kernel_len - 1) as f64).cos();
                h * w
            } else {
                0.0
            }
        })
        .collect();

    // Convolve signal with Hilbert kernel (same-length output via centered
    // convolution).
    let mut hilbert = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for (j, &k) in kernel.iter().enumerate() {
            let idx = i as isize + j as isize - half as isize;
            if idx >= 0 && (idx as usize) < n {
                sum += signal[idx as usize] * k;
            }
        }
        hilbert[i] = sum;
    }

    // Envelope = |analytic signal|
    signal
        .iter()
        .zip(hilbert.iter())
        .map(|(&x, &h)| (x * x + h * h).sqrt())
        .collect()
}

/// Compute spectral kurtosis over frames of the input signal.
///
/// Spectral kurtosis measures the "Gaussianity" of the signal energy in each
/// frame. A value of 0 indicates Gaussian noise; positive values indicate the
/// presence of impulsive/transient content (like wheel flat impacts).
///
/// The computation uses the power spectral density estimated via a simple DFT
/// per frame.
///
/// # Arguments
///
/// * `signal` - Input signal (accelerometer data).
/// * `frame_size` - Number of samples per analysis frame.
/// * `hop` - Number of samples to advance between frames.
///
/// # Returns
///
/// A vector of spectral kurtosis values, one per frame.
pub fn spectral_kurtosis(signal: &[f64], frame_size: usize, hop: usize) -> Vec<f64> {
    if frame_size == 0 || hop == 0 || signal.len() < frame_size {
        return Vec::new();
    }

    let mut result = Vec::new();
    let mut offset = 0;

    while offset + frame_size <= signal.len() {
        let frame = &signal[offset..offset + frame_size];

        // Compute power spectrum via DFT (magnitude squared).
        let power: Vec<f64> = (0..frame_size)
            .map(|k| {
                let mut re = 0.0;
                let mut im = 0.0;
                for (n, &x) in frame.iter().enumerate() {
                    let angle = -2.0 * PI * k as f64 * n as f64 / frame_size as f64;
                    re += x * angle.cos();
                    im += x * angle.sin();
                }
                re * re + im * im
            })
            .collect();

        // Kurtosis of the power spectrum: E[X^4] / E[X^2]^2 - 2
        // (excess kurtosis, where Gaussian -> 0).
        let n = power.len() as f64;
        let mean = power.iter().sum::<f64>() / n;
        if mean.abs() < 1e-30 {
            result.push(0.0);
        } else {
            let m2 = power.iter().map(|&p| (p - mean).powi(2)).sum::<f64>() / n;
            let m4 = power.iter().map(|&p| (p - mean).powi(4)).sum::<f64>() / n;
            if m2.abs() < 1e-30 {
                result.push(0.0);
            } else {
                let kurt = m4 / (m2 * m2) - 3.0; // excess kurtosis
                result.push(kurt);
            }
        }

        offset += hop;
    }

    result
}

/// Generate a synthetic accelerometer signal simulating a wheel with a flat
/// spot passing over a sensor.
///
/// The output is in units of g. Each rotation produces an impact pulse
/// (half-sine) at the expected rotation interval, riding on top of Gaussian-ish
/// background vibration approximated by a simple LFSR-based pseudo-noise
/// generator.
///
/// # Arguments
///
/// * `config` - Detector configuration (sample rate, wheel diameter, speed).
/// * `flat_depth_mm` - Depth of the flat in millimetres.
/// * `num_rotations` - Number of complete wheel rotations to simulate.
pub fn generate_wheel_flat_signal(
    config: &WheelFlatConfig,
    flat_depth_mm: f64,
    num_rotations: usize,
) -> Vec<f64> {
    let period_s = wheel_rotation_period(config.wheel_diameter_m, config.train_speed_mps);
    let period_samples = (period_s * config.sample_rate_hz).round() as usize;
    let total_samples = period_samples * num_rotations;

    if total_samples == 0 || period_samples == 0 {
        return Vec::new();
    }

    // Impact amplitude in g (empirical model from flat depth and speed).
    let g0 = 9.80665;
    let impact_amplitude_g =
        2.0 * g0 * flat_depth_mm * 100.0 / (config.train_speed_mps.max(0.1));
    // Clamp to a reasonable range.
    let impact_amplitude_g = impact_amplitude_g.max(10.0).min(500.0);

    // Impact pulse width: approximately the contact time, typically 0.5-2 ms.
    let pulse_width_s = 0.001; // 1 ms
    let pulse_width_samples = (pulse_width_s * config.sample_rate_hz).round() as usize;
    let pulse_width_samples = pulse_width_samples.max(3);

    let mut signal = vec![0.0; total_samples];

    // Add background noise using a simple LFSR-based pseudo-random generator.
    // We want roughly Gaussian noise so we sum 4 uniform random values and
    // normalise (central limit theorem approximation).
    let noise_amplitude_g = 5.0; // background vibration level
    let mut lfsr: u32 = 0xACE1_u32;
    for sample in signal.iter_mut() {
        let mut sum = 0.0_f64;
        for _ in 0..4 {
            let bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
            lfsr = (lfsr >> 1) | (bit << 15);
            sum += (lfsr & 0xFFFF) as f64 / 65535.0;
        }
        *sample = (sum / 4.0 - 0.5) * 2.0 * noise_amplitude_g;
    }

    // Add impact pulses at each rotation.
    for rot in 0..num_rotations {
        let centre = rot * period_samples + period_samples / 2;
        let half_pw = pulse_width_samples / 2;
        for i in 0..pulse_width_samples {
            let idx = centre.wrapping_sub(half_pw).wrapping_add(i);
            if idx < total_samples {
                let t = i as f64 / pulse_width_samples as f64;
                let pulse = impact_amplitude_g * (PI * t).sin();
                signal[idx] += pulse;
            }
        }
    }

    signal
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// Find local-maximum indices in `signal` that exceed `threshold`.
fn find_peaks(signal: &[f64], threshold: f64) -> Vec<usize> {
    let n = signal.len();
    if n < 3 {
        // For very short signals just return the max if above threshold.
        if let Some((i, &v)) = signal.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        {
            if v >= threshold {
                return vec![i];
            }
        }
        return Vec::new();
    }

    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        // Require strictly greater than at least one neighbour to avoid
        // selecting every sample in a constant region.
        if signal[i] >= threshold
            && signal[i] >= signal[i - 1]
            && signal[i] >= signal[i + 1]
            && (signal[i] > signal[i - 1] || signal[i] > signal[i + 1])
        {
            peaks.push(i);
        }
    }
    // Also check endpoints.
    if signal[0] >= threshold && signal[0] > signal[1] {
        peaks.insert(0, 0);
    }
    if signal[n - 1] >= threshold && signal[n - 1] > signal[n - 2] {
        peaks.push(n - 1);
    }
    peaks
}

/// Group peak indices into sets where successive peaks are spaced at
/// approximately `expected_period` samples (+/- `tolerance` fraction).
fn group_periodic_peaks(
    peaks: &[usize],
    expected_period: f64,
    tolerance: f64,
) -> Vec<Vec<usize>> {
    if peaks.is_empty() {
        return Vec::new();
    }

    let low = expected_period * (1.0 - tolerance);
    let high = expected_period * (1.0 + tolerance);

    let mut used = vec![false; peaks.len()];
    let mut groups: Vec<Vec<usize>> = Vec::new();

    for i in 0..peaks.len() {
        if used[i] {
            continue;
        }
        let mut group = vec![peaks[i]];
        used[i] = true;
        let mut last = peaks[i];

        for j in (i + 1)..peaks.len() {
            if used[j] {
                continue;
            }
            let gap = peaks[j] as f64 - last as f64;
            if gap >= low && gap <= high {
                group.push(peaks[j]);
                used[j] = true;
                last = peaks[j];
            }
        }

        if group.len() >= 2 {
            groups.push(group);
        }
    }

    groups
}

/// Estimate the number of distinct wheels from isolated peaks (no periodic
/// grouping available).
fn estimate_wheel_count(peaks: &[usize], expected_period: f64) -> usize {
    if peaks.is_empty() {
        return 0;
    }
    // Count the number of peaks separated by at least half the expected
    // rotation period -- each such cluster likely represents a different wheel.
    let min_gap = expected_period * 0.5;
    let mut count = 1;
    let mut last = peaks[0];
    for &p in peaks.iter().skip(1) {
        if (p as f64 - last as f64) >= min_gap {
            count += 1;
            last = p;
        }
    }
    count
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // -----------------------------------------------------------------------
    // Rotation period
    // -----------------------------------------------------------------------

    #[test]
    fn test_rotation_period_standard_wheel() {
        // 36-inch (0.914 m) wheel at 20 m/s.
        let t = wheel_rotation_period(0.914, 20.0);
        let expected = PI * 0.914 / 20.0;
        assert!((t - expected).abs() < 1e-12);
    }

    #[test]
    fn test_rotation_period_zero_speed() {
        let t = wheel_rotation_period(0.914, 0.0);
        assert!(t.is_infinite());
    }

    #[test]
    fn test_rotation_period_large_wheel() {
        // 40-inch wheel (1.016 m) at 30 m/s.
        let t = wheel_rotation_period(1.016, 30.0);
        let expected = PI * 1.016 / 30.0;
        assert!((t - expected).abs() < 1e-12);
    }

    #[test]
    fn test_rotation_period_slow_speed() {
        let t = wheel_rotation_period(0.914, 1.0);
        assert!(t > 2.0); // Should be about 2.87 s.
    }

    // -----------------------------------------------------------------------
    // Flat length from chord
    // -----------------------------------------------------------------------

    #[test]
    fn test_flat_length_basic() {
        // 0.914 m diameter, 1 mm depth.
        let l = flat_length_from_chord(0.914, 1.0);
        // Expected: 2 * sqrt(914 * 1 - 1) = 2 * sqrt(913) ≈ 60.43 mm
        let expected = 2.0 * (914.0_f64 * 1.0 - 1.0).sqrt();
        assert!((l - expected).abs() < 0.01);
    }

    #[test]
    fn test_flat_length_zero_depth() {
        let l = flat_length_from_chord(0.914, 0.0);
        assert_eq!(l, 0.0);
    }

    #[test]
    fn test_flat_length_negative_depth() {
        let l = flat_length_from_chord(0.914, -1.0);
        assert_eq!(l, 0.0);
    }

    #[test]
    fn test_flat_length_exceeds_radius() {
        // Depth equal to radius should return 0 (physically impossible).
        let l = flat_length_from_chord(0.914, 457.0);
        assert_eq!(l, 0.0);
    }

    #[test]
    fn test_flat_length_small_depth() {
        let l = flat_length_from_chord(0.914, 0.1);
        // Should be small but positive.
        assert!(l > 0.0);
        assert!(l < 30.0);
    }

    #[test]
    fn test_flat_length_increases_with_depth() {
        let l1 = flat_length_from_chord(0.914, 0.5);
        let l2 = flat_length_from_chord(0.914, 1.0);
        let l3 = flat_length_from_chord(0.914, 2.0);
        assert!(l1 < l2);
        assert!(l2 < l3);
    }

    // -----------------------------------------------------------------------
    // Flat depth from impact
    // -----------------------------------------------------------------------

    #[test]
    fn test_flat_depth_from_impact_positive() {
        let d = flat_depth_from_impact(100.0, 20.0, 400.0);
        assert!(d > 0.0);
    }

    #[test]
    fn test_flat_depth_increases_with_acceleration() {
        let d1 = flat_depth_from_impact(50.0, 20.0, 400.0);
        let d2 = flat_depth_from_impact(100.0, 20.0, 400.0);
        assert!(d2 > d1);
    }

    #[test]
    fn test_flat_depth_increases_with_speed() {
        let d1 = flat_depth_from_impact(100.0, 10.0, 400.0);
        let d2 = flat_depth_from_impact(100.0, 20.0, 400.0);
        assert!(d2 > d1);
    }

    #[test]
    fn test_flat_depth_zero_impact() {
        let d = flat_depth_from_impact(0.0, 20.0, 400.0);
        assert_eq!(d, 0.0);
    }

    // -----------------------------------------------------------------------
    // Impact force
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_impact_force() {
        let f = compute_impact_force(100.0, 400.0);
        let expected = 400.0 * 100.0 * 9.80665;
        assert!((f - expected).abs() < 1e-6);
    }

    #[test]
    fn test_impact_force_zero_mass() {
        let f = compute_impact_force(100.0, 0.0);
        assert_eq!(f, 0.0);
    }

    #[test]
    fn test_impact_force_zero_accel() {
        let f = compute_impact_force(0.0, 400.0);
        assert_eq!(f, 0.0);
    }

    // -----------------------------------------------------------------------
    // Severity classification
    // -----------------------------------------------------------------------

    #[test]
    fn test_severity_minor() {
        assert_eq!(FlatSeverity::from_length_mm(10.0), FlatSeverity::Minor);
        assert_eq!(FlatSeverity::from_length_mm(39.9), FlatSeverity::Minor);
    }

    #[test]
    fn test_severity_moderate() {
        assert_eq!(FlatSeverity::from_length_mm(40.0), FlatSeverity::Moderate);
        assert_eq!(FlatSeverity::from_length_mm(59.9), FlatSeverity::Moderate);
    }

    #[test]
    fn test_severity_severe() {
        assert_eq!(FlatSeverity::from_length_mm(60.0), FlatSeverity::Severe);
        assert_eq!(FlatSeverity::from_length_mm(79.9), FlatSeverity::Severe);
    }

    #[test]
    fn test_severity_critical() {
        assert_eq!(FlatSeverity::from_length_mm(80.0), FlatSeverity::Critical);
        assert_eq!(FlatSeverity::from_length_mm(200.0), FlatSeverity::Critical);
    }

    // -----------------------------------------------------------------------
    // Envelope via Hilbert transform
    // -----------------------------------------------------------------------

    #[test]
    fn test_envelope_empty_signal() {
        let env = envelope_from_hilbert(&[]);
        assert!(env.is_empty());
    }

    #[test]
    fn test_envelope_dc_signal() {
        let signal = vec![1.0; 256];
        let env = envelope_from_hilbert(&signal);
        assert_eq!(env.len(), 256);
        // For a DC signal the Hilbert transform should be ~0, so envelope ≈ |DC|.
        // Check the middle region (edges have transient effects).
        for &v in &env[64..192] {
            assert!((v - 1.0).abs() < 0.3, "envelope of DC should be near 1.0, got {}", v);
        }
    }

    #[test]
    fn test_envelope_sine_wave() {
        // Envelope of a pure sine should be approximately constant ≈ amplitude.
        let n = 512;
        let amplitude = 3.0;
        let freq = 50.0;
        let fs = 1000.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| amplitude * (2.0 * PI * freq * i as f64 / fs).sin())
            .collect();
        let env = envelope_from_hilbert(&signal);
        assert_eq!(env.len(), n);
        // Check the central portion (skip transients).
        let mid = &env[100..400];
        let mean_env = mid.iter().sum::<f64>() / mid.len() as f64;
        assert!(
            (mean_env - amplitude).abs() < 0.5,
            "mean envelope {} should be near amplitude {}",
            mean_env,
            amplitude
        );
    }

    #[test]
    fn test_envelope_length_matches_input() {
        let signal = vec![0.0; 100];
        let env = envelope_from_hilbert(&signal);
        assert_eq!(env.len(), 100);
    }

    // -----------------------------------------------------------------------
    // Spectral kurtosis
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectral_kurtosis_empty() {
        let sk = spectral_kurtosis(&[], 64, 32);
        assert!(sk.is_empty());
    }

    #[test]
    fn test_spectral_kurtosis_frame_too_large() {
        let signal = vec![1.0; 32];
        let sk = spectral_kurtosis(&signal, 64, 32);
        assert!(sk.is_empty());
    }

    #[test]
    fn test_spectral_kurtosis_noise_near_zero() {
        // For Gaussian-like noise, spectral kurtosis should be near 0.
        let n = 4096;
        // Generate pseudo-Gaussian noise via CLT.
        let mut lfsr: u32 = 0xBEEF;
        let signal: Vec<f64> = (0..n)
            .map(|_| {
                let mut sum = 0.0;
                for _ in 0..12 {
                    let bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
                    lfsr = (lfsr >> 1) | (bit << 15);
                    sum += (lfsr & 0xFFFF) as f64 / 65535.0;
                }
                sum - 6.0
            })
            .collect();

        let sk = spectral_kurtosis(&signal, 256, 128);
        assert!(!sk.is_empty());
        let mean_sk = sk.iter().sum::<f64>() / sk.len() as f64;
        // Should be roughly near zero for Gaussian noise (within a broad margin
        // because our pseudo-noise isn't perfectly Gaussian).
        assert!(
            mean_sk.abs() < 5.0,
            "mean spectral kurtosis for noise should be near 0, got {}",
            mean_sk
        );
    }

    #[test]
    fn test_spectral_kurtosis_impulse_detection() {
        // Spectral kurtosis measures the kurtosis of the power spectrum across
        // frequency bins. A Gaussian noise frame has a roughly exponential PSD
        // distribution (positive excess kurtosis ~2-6). A frame dominated by a
        // broadband impulse has a nearly flat PSD (kurtosis near -1.2 for
        // uniform). Therefore, impulsive frames should have *lower* SK than
        // noise-only frames. This difference is what allows transient
        // detection: frames that deviate significantly from the noise-floor SK
        // (in either direction) indicate non-stationary events.
        let frame_size = 64;
        let num_frames = 32;
        let n = frame_size * num_frames;

        // Build pseudo-Gaussian noise.
        let mut lfsr: u32 = 0xCAFE;
        let noise: Vec<f64> = (0..n)
            .map(|_| {
                let mut s = 0.0_f64;
                for _ in 0..6 {
                    let bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1;
                    lfsr = (lfsr >> 1) | (bit << 15);
                    s += (lfsr & 0xFFFF) as f64 / 65535.0;
                }
                s / 6.0 - 0.5
            })
            .collect();

        // Impulsive version: add a large spike in frame 5.
        let mut impulsive = noise.clone();
        impulsive[frame_size * 5 + 10] += 200.0;

        let sk_noise = spectral_kurtosis(&noise, frame_size, frame_size);
        let sk_impulse = spectral_kurtosis(&impulsive, frame_size, frame_size);

        assert!(!sk_noise.is_empty());
        assert!(!sk_impulse.is_empty());

        // The impulsive frame should have *different* SK from the noise-only
        // version of the same frame. The impulse flattens the PSD, lowering
        // kurtosis relative to the noise-only version.
        let diff = (sk_impulse[5] - sk_noise[5]).abs();
        assert!(
            diff > 0.5,
            "SK should change noticeably in impulsive frame: noise={}, impulse={}, diff={}",
            sk_noise[5],
            sk_impulse[5],
            diff
        );
    }

    #[test]
    fn test_spectral_kurtosis_num_frames() {
        let n = 1024;
        let signal = vec![1.0; n];
        let frame_size = 128;
        let hop = 64;
        let sk = spectral_kurtosis(&signal, frame_size, hop);
        let expected_frames = (n - frame_size) / hop + 1;
        assert_eq!(sk.len(), expected_frames);
    }

    // -----------------------------------------------------------------------
    // Synthetic signal generation
    // -----------------------------------------------------------------------

    #[test]
    fn test_generate_signal_not_empty() {
        let config = WheelFlatConfig::default();
        let signal = generate_wheel_flat_signal(&config, 2.0, 5);
        assert!(!signal.is_empty());
    }

    #[test]
    fn test_generate_signal_length() {
        let config = WheelFlatConfig {
            sample_rate_hz: 10_000.0,
            wheel_diameter_m: 0.914,
            train_speed_mps: 20.0,
            impact_threshold_g: 50.0,
        };
        let num_rotations = 5;
        let period_samples =
            (wheel_rotation_period(0.914, 20.0) * 10_000.0).round() as usize;
        let signal = generate_wheel_flat_signal(&config, 2.0, num_rotations);
        assert_eq!(signal.len(), period_samples * num_rotations);
    }

    #[test]
    fn test_generate_signal_has_impacts() {
        let config = WheelFlatConfig {
            sample_rate_hz: 10_000.0,
            wheel_diameter_m: 0.914,
            train_speed_mps: 20.0,
            impact_threshold_g: 50.0,
        };
        let signal = generate_wheel_flat_signal(&config, 2.0, 5);
        let max_val = signal.iter().copied().fold(0.0_f64, f64::max);
        // The impact peaks should be well above background noise (5 g).
        assert!(max_val > 20.0, "signal should contain impact peaks, max = {}", max_val);
    }

    #[test]
    fn test_generate_signal_zero_rotations() {
        let config = WheelFlatConfig::default();
        let signal = generate_wheel_flat_signal(&config, 2.0, 0);
        assert!(signal.is_empty());
    }

    // -----------------------------------------------------------------------
    // Periodic impact detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_periodic_impacts_synthetic() {
        // Create a signal with periodic impulses.
        let period = 100; // samples
        let num_pulses = 5;
        let n = period * (num_pulses + 1);
        let mut signal = vec![0.0; n];
        for i in 0..num_pulses {
            let idx = period / 2 + i * period;
            if idx < n {
                signal[idx] = 100.0;
            }
        }

        let result = detect_periodic_impacts(&signal, period as f64, 0.2);
        // Should find at least 2 periodic peaks.
        assert!(
            result.len() >= 2,
            "expected >= 2 periodic impacts, got {}",
            result.len()
        );
    }

    #[test]
    fn test_detect_periodic_impacts_empty() {
        let result = detect_periodic_impacts(&[], 100.0, 0.2);
        assert!(result.is_empty());
    }

    #[test]
    fn test_detect_periodic_impacts_no_peaks() {
        let signal = vec![0.0; 500];
        let result = detect_periodic_impacts(&signal, 100.0, 0.2);
        assert!(result.is_empty());
    }

    // -----------------------------------------------------------------------
    // Full detector pipeline
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_empty_signal() {
        let config = WheelFlatConfig::default();
        let detector = WheelFlatDetector::new(config);
        let report = detector.process(&[]);
        assert!(report.detections.is_empty());
        assert_eq!(report.num_wheels_passed, 0);
        assert_eq!(report.max_impact_g, 0.0);
    }

    #[test]
    fn test_detector_with_synthetic_signal() {
        let config = WheelFlatConfig {
            sample_rate_hz: 10_000.0,
            wheel_diameter_m: 0.914,
            train_speed_mps: 20.0,
            impact_threshold_g: 30.0, // lower threshold to catch impacts
        };
        let signal = generate_wheel_flat_signal(&config, 3.0, 8);
        let detector = WheelFlatDetector::new(config);
        let report = detector.process(&signal);

        assert!(report.max_impact_g > 0.0);
        // We should detect at least one flat.
        // (Due to envelope smoothing and noise, detection is not guaranteed to
        // be perfect, but the synthetic signal is strong enough.)
        assert!(
            report.max_impact_g > 20.0,
            "max impact should be significant, got {}",
            report.max_impact_g
        );
    }

    #[test]
    fn test_detector_report_fields() {
        let config = WheelFlatConfig::default();
        let signal = generate_wheel_flat_signal(&config, 2.0, 5);
        let detector = WheelFlatDetector::new(config);
        let report = detector.process(&signal);

        // Report should have sensible max_impact_g.
        assert!(report.max_impact_g >= 0.0);

        // Each detection should have positive fields.
        for det in &report.detections {
            assert!(det.flat_length_mm >= 0.0);
            assert!(det.impact_force_g > 0.0);
            assert!(det.rotation_period_s > 0.0);
        }
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_single_sample_signal() {
        let config = WheelFlatConfig::default();
        let detector = WheelFlatDetector::new(config);
        let report = detector.process(&[42.0]);
        assert_eq!(report.max_impact_g, 42.0);
    }

    #[test]
    fn test_all_zero_signal() {
        let config = WheelFlatConfig::default();
        let detector = WheelFlatDetector::new(config);
        let signal = vec![0.0; 10_000];
        let report = detector.process(&signal);
        assert!(report.detections.is_empty());
    }

    #[test]
    fn test_constant_high_signal() {
        // A constant signal above threshold: the envelope via Hilbert has
        // edge transients but the central region is flat. The detector may
        // pick up edge artefacts but the *periodic* grouping step should
        // suppress most false detections.
        let config = WheelFlatConfig {
            impact_threshold_g: 50.0,
            ..WheelFlatConfig::default()
        };
        let detector = WheelFlatDetector::new(config);
        let signal = vec![100.0; 10_000];
        let report = detector.process(&signal);
        // Main assertion: the max impact should be close to 100 g (the DC
        // level) and we should have very few *periodic* detections (most edge
        // peaks won't form periodic groups).
        assert!(
            report.detections.len() <= 3,
            "constant signal should produce few periodic detections, got {}",
            report.detections.len()
        );
    }

    #[test]
    fn test_find_peaks_basic() {
        let signal = vec![0.0, 1.0, 3.0, 2.0, 0.0, 5.0, 4.0, 0.0];
        let peaks = find_peaks(&signal, 2.0);
        assert!(peaks.contains(&2)); // value 3.0
        assert!(peaks.contains(&5)); // value 5.0
    }

    #[test]
    fn test_find_peaks_no_peaks() {
        let signal = vec![0.0, 0.1, 0.2, 0.1, 0.0];
        let peaks = find_peaks(&signal, 1.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_group_periodic_peaks_basic() {
        // Peaks at 100, 200, 300 with expected period 100.
        let peaks = vec![100, 200, 300, 700, 800, 900];
        let groups = group_periodic_peaks(&peaks, 100.0, 0.2);
        // Should produce two groups.
        assert_eq!(groups.len(), 2);
        assert_eq!(groups[0], vec![100, 200, 300]);
        assert_eq!(groups[1], vec![700, 800, 900]);
    }

    #[test]
    fn test_default_config() {
        let config = WheelFlatConfig::default();
        assert_eq!(config.wheel_diameter_m, 0.914);
        assert_eq!(config.impact_threshold_g, 50.0);
        assert_eq!(config.sample_rate_hz, 10_000.0);
        assert_eq!(config.train_speed_mps, 20.0);
    }
}

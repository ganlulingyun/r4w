//! Acoustic detection and identification of unmanned aerial vehicles (UAVs/drones).
//!
//! This module provides algorithms for detecting drones by their acoustic
//! signatures. It analyzes the blade passage frequency (BPF) of propellers,
//! extracts harmonic series, estimates motor RPM, classifies multi-rotor types,
//! detects approach/recede via Doppler shift, estimates range from signal level,
//! and generates alerts.
//!
//! # Overview
//!
//! Drone propellers produce a characteristic acoustic signature consisting of a
//! fundamental blade passage frequency (BPF) and its harmonics. The BPF is
//! determined by the motor RPM and the number of propeller blades:
//!
//! ```text
//! BPF = (RPM × num_blades) / 60
//! ```
//!
//! Different drone types exhibit distinct harmonic patterns that enable
//! classification. Doppler shift of the BPF over time reveals whether a drone
//! is approaching or receding.
//!
//! # Example
//!
//! ```
//! use r4w_core::drone_acoustic_detector::{DroneConfig, DroneDetector, estimate_rpm};
//!
//! let config = DroneConfig {
//!     sample_rate_hz: 44100.0,
//!     fft_size: 4096,
//!     min_freq_hz: 50.0,
//!     max_freq_hz: 2000.0,
//!     num_blades: 2,
//! };
//!
//! let detector = DroneDetector::new(config);
//!
//! // A 2-blade propeller spinning at 6000 RPM produces BPF = 200 Hz
//! let rpm = estimate_rpm(200.0, 2);
//! assert!((rpm - 6000.0).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

/// Configuration for the drone acoustic detector.
#[derive(Debug, Clone)]
pub struct DroneConfig {
    /// Audio sample rate in Hz (e.g., 44100.0).
    pub sample_rate_hz: f64,
    /// FFT size used for spectral analysis.
    pub fft_size: usize,
    /// Minimum frequency of interest in Hz.
    pub min_freq_hz: f64,
    /// Maximum frequency of interest in Hz.
    pub max_freq_hz: f64,
    /// Number of blades per propeller.
    pub num_blades: u8,
}

impl Default for DroneConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 44100.0,
            fft_size: 4096,
            min_freq_hz: 50.0,
            max_freq_hz: 2000.0,
            num_blades: 2,
        }
    }
}

/// Result of a drone detection analysis.
#[derive(Debug, Clone)]
pub struct DroneDetection {
    /// Whether a drone was detected.
    pub detected: bool,
    /// Detection confidence in the range [0.0, 1.0].
    pub confidence: f64,
    /// Estimated blade passage frequency in Hz.
    pub bpf_hz: f64,
    /// Estimated motor RPM.
    pub rpm: f64,
    /// Number of harmonics found above the noise floor.
    pub num_harmonics: usize,
    /// Estimated range to the drone in meters.
    pub estimated_range_m: f64,
    /// Whether the drone is approaching (true) or receding (false).
    pub approaching: bool,
}

impl Default for DroneDetection {
    fn default() -> Self {
        Self {
            detected: false,
            confidence: 0.0,
            bpf_hz: 0.0,
            rpm: 0.0,
            num_harmonics: 0,
            estimated_range_m: f64::INFINITY,
            approaching: false,
        }
    }
}

/// Classification of drone type based on acoustic signature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DroneType {
    /// Four-rotor multicopter (most common consumer drones).
    QuadRotor,
    /// Six-rotor multicopter (professional/heavy-lift drones).
    HexaRotor,
    /// Eight-rotor multicopter (cinema/industrial drones).
    OctoRotor,
    /// Fixed-wing UAV (single propeller, distinct signature).
    FixedWing,
    /// Unclassified drone type.
    Unknown,
}

/// Main drone acoustic detector.
///
/// Maintains state across successive frames for Doppler tracking and
/// temporal smoothing of detection results.
#[derive(Debug, Clone)]
pub struct DroneDetector {
    config: DroneConfig,
    bpf_history: Vec<f64>,
    max_history: usize,
}

impl DroneDetector {
    /// Create a new `DroneDetector` with the given configuration.
    pub fn new(config: DroneConfig) -> Self {
        Self {
            config,
            bpf_history: Vec::new(),
            max_history: 64,
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &DroneConfig {
        &self.config
    }

    /// Return the BPF history buffer.
    pub fn bpf_history(&self) -> &[f64] {
        &self.bpf_history
    }

    /// Process a single frame of audio (time-domain samples) and return a
    /// detection result.
    ///
    /// The method computes a magnitude spectrum internally using a simple DFT,
    /// then delegates to the stateless helper functions for BPF detection,
    /// harmonic extraction, classification, and so on.
    pub fn process_frame(
        &mut self,
        samples: &[f64],
        reference_level_db: f64,
        reference_range_m: f64,
        alert_threshold: f64,
    ) -> DroneDetection {
        if samples.is_empty() {
            return DroneDetection::default();
        }

        let n = self.config.fft_size.min(samples.len());
        let spectrum = simple_magnitude_spectrum(&samples[..n]);
        let freq_resolution = self.config.sample_rate_hz / n as f64;

        // Estimate noise floor as the median of the spectrum.
        let noise_floor = estimate_noise_floor(&spectrum);

        // Detect fundamental BPF.
        let bpf = match detect_bpf(
            &spectrum,
            freq_resolution,
            self.config.min_freq_hz,
            self.config.max_freq_hz,
        ) {
            Some(f) => f,
            None => return DroneDetection::default(),
        };

        // Extract harmonics.
        let harmonics = extract_harmonics(&spectrum, bpf, freq_resolution, 8);

        // Compute detection metric.
        let noise_floor_vec = vec![noise_floor; spectrum.len()];
        let metric = compute_detection_metric(&spectrum, &noise_floor_vec);

        // RPM estimate.
        let rpm = estimate_rpm(bpf, self.config.num_blades);

        // Signal level at BPF bin.
        let bpf_bin = (bpf / freq_resolution).round() as usize;
        let signal_level_db = if bpf_bin < spectrum.len() && spectrum[bpf_bin] > 0.0 {
            20.0 * spectrum[bpf_bin].log10()
        } else {
            -100.0
        };

        let estimated_range = estimate_range(signal_level_db, reference_level_db, reference_range_m);

        // Doppler tracking.
        self.bpf_history.push(bpf);
        if self.bpf_history.len() > self.max_history {
            self.bpf_history.remove(0);
        }

        let doppler_rate = detect_doppler_shift(&self.bpf_history);
        let approaching = is_approaching(doppler_rate);

        // Confidence: blend detection metric and harmonic count.
        let harmonic_factor = (harmonics.len() as f64 / 5.0).min(1.0);
        let confidence = (0.6 * metric.min(1.0) + 0.4 * harmonic_factor).clamp(0.0, 1.0);

        DroneDetection {
            detected: generate_alert_from_metric(metric, alert_threshold),
            confidence,
            bpf_hz: bpf,
            rpm,
            num_harmonics: harmonics.len(),
            estimated_range_m: estimated_range,
            approaching,
        }
    }

    /// Reset the internal state (BPF history).
    pub fn reset(&mut self) {
        self.bpf_history.clear();
    }
}

// ---------------------------------------------------------------------------
// Public stateless functions
// ---------------------------------------------------------------------------

/// Detect the blade passage frequency from a magnitude spectrum.
///
/// Searches for the strongest spectral peak between `min_freq` and `max_freq`.
/// Returns `None` if no peak is found above a minimal threshold.
///
/// # Arguments
///
/// * `spectrum` - Magnitude spectrum (one-sided).
/// * `freq_resolution` - Frequency spacing between bins in Hz.
/// * `min_freq` - Lower bound of the search range in Hz.
/// * `max_freq` - Upper bound of the search range in Hz.
pub fn detect_bpf(
    spectrum: &[f64],
    freq_resolution: f64,
    min_freq: f64,
    max_freq: f64,
) -> Option<f64> {
    if spectrum.is_empty() || freq_resolution <= 0.0 {
        return None;
    }

    let min_bin = (min_freq / freq_resolution).ceil() as usize;
    let max_bin = ((max_freq / freq_resolution).floor() as usize).min(spectrum.len() - 1);

    if min_bin > max_bin || min_bin >= spectrum.len() {
        return None;
    }

    let mut best_bin = min_bin;
    let mut best_val = spectrum[min_bin];

    for i in (min_bin + 1)..=max_bin {
        if spectrum[i] > best_val {
            best_val = spectrum[i];
            best_bin = i;
        }
    }

    // Require the peak to be above a minimal threshold to avoid noise-only detections.
    let mean_val: f64 = spectrum[min_bin..=max_bin].iter().sum::<f64>()
        / (max_bin - min_bin + 1) as f64;

    if best_val > mean_val * 2.0 && best_val > 1e-12 {
        Some(best_bin as f64 * freq_resolution)
    } else {
        None
    }
}

/// Extract harmonic frequencies and their amplitudes from a magnitude spectrum.
///
/// Starting from `fundamental_hz`, checks each integer multiple up to
/// `max_harmonics`. A harmonic is considered present if the spectral energy
/// in a small window around the expected bin is significantly above the local
/// average.
///
/// Returns a vector of `(frequency_hz, amplitude)` pairs for each detected
/// harmonic, including the fundamental.
pub fn extract_harmonics(
    spectrum: &[f64],
    fundamental_hz: f64,
    freq_resolution: f64,
    max_harmonics: usize,
) -> Vec<(f64, f64)> {
    if spectrum.is_empty() || freq_resolution <= 0.0 || fundamental_hz <= 0.0 {
        return Vec::new();
    }

    let mut harmonics = Vec::new();
    let half_window = 2; // bins around expected peak

    for h in 1..=max_harmonics {
        let expected_freq = fundamental_hz * h as f64;
        let expected_bin = (expected_freq / freq_resolution).round() as usize;

        if expected_bin >= spectrum.len() {
            break;
        }

        let lo = expected_bin.saturating_sub(half_window);
        let hi = (expected_bin + half_window).min(spectrum.len() - 1);

        // Find peak in window.
        let mut peak_bin = lo;
        let mut peak_val = spectrum[lo];
        for i in (lo + 1)..=hi {
            if spectrum[i] > peak_val {
                peak_val = spectrum[i];
                peak_bin = i;
            }
        }

        // Compute local noise estimate from a wider region excluding the window.
        let wide_lo = expected_bin.saturating_sub(half_window * 5);
        let wide_hi = (expected_bin + half_window * 5).min(spectrum.len() - 1);
        let mut noise_sum = 0.0;
        let mut noise_count = 0usize;
        for i in wide_lo..=wide_hi {
            if i < lo || i > hi {
                noise_sum += spectrum[i];
                noise_count += 1;
            }
        }
        let local_noise = if noise_count > 0 {
            noise_sum / noise_count as f64
        } else {
            0.0
        };

        // Accept harmonic if peak is 3 dB above local noise.
        if peak_val > local_noise * 1.4 && peak_val > 1e-12 {
            harmonics.push((peak_bin as f64 * freq_resolution, peak_val));
        }
    }

    harmonics
}

/// Estimate motor RPM from the blade passage frequency and number of blades.
///
/// ```text
/// RPM = (BPF × 60) / num_blades
/// ```
pub fn estimate_rpm(bpf_hz: f64, num_blades: u8) -> f64 {
    if num_blades == 0 {
        return 0.0;
    }
    (bpf_hz * 60.0) / num_blades as f64
}

/// Classify the drone type from its harmonic pattern and BPF.
///
/// Uses a simple heuristic:
/// - **QuadRotor**: 3-5 strong harmonics, BPF typically 100-400 Hz.
/// - **HexaRotor**: 4-6 harmonics with broader spectral spread.
/// - **OctoRotor**: 5+ harmonics with characteristic even-harmonic emphasis.
/// - **FixedWing**: 1-2 dominant harmonics (single propeller).
/// - **Unknown**: Anything else.
pub fn classify_drone_type(harmonics: &[(f64, f64)], bpf_hz: f64) -> DroneType {
    let num = harmonics.len();

    if num == 0 {
        return DroneType::Unknown;
    }

    // Check for even-harmonic emphasis (OctoRotor indicator).
    if num >= 5 {
        let even_energy: f64 = harmonics
            .iter()
            .enumerate()
            .filter(|(i, _)| (i + 1) % 2 == 0)
            .map(|(_, (_, a))| a)
            .sum();
        let total_energy: f64 = harmonics.iter().map(|(_, a)| a).sum();
        if total_energy > 0.0 && even_energy / total_energy > 0.55 {
            return DroneType::OctoRotor;
        }
    }

    if num >= 4 && bpf_hz > 300.0 {
        return DroneType::HexaRotor;
    }

    if num >= 3 && bpf_hz >= 80.0 && bpf_hz <= 500.0 {
        return DroneType::QuadRotor;
    }

    if num <= 2 && bpf_hz < 150.0 {
        return DroneType::FixedWing;
    }

    DroneType::Unknown
}

/// Compute the rate of change of BPF over time (Hz per frame).
///
/// Uses a simple linear regression on the BPF history to estimate the trend.
/// A positive value means the frequency is increasing (approaching source),
/// negative means decreasing (receding source).
pub fn detect_doppler_shift(bpf_history: &[f64]) -> f64 {
    let n = bpf_history.len();
    if n < 2 {
        return 0.0;
    }

    // Simple linear regression: slope of BPF vs. time index.
    let n_f = n as f64;
    let sum_x: f64 = (0..n).map(|i| i as f64).sum();
    let sum_y: f64 = bpf_history.iter().sum();
    let sum_xy: f64 = bpf_history
        .iter()
        .enumerate()
        .map(|(i, &y)| i as f64 * y)
        .sum();
    let sum_x2: f64 = (0..n).map(|i| (i as f64) * (i as f64)).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-12 {
        return 0.0;
    }

    (n_f * sum_xy - sum_x * sum_y) / denom
}

/// Determine if a drone is approaching based on the Doppler rate.
///
/// A positive Doppler rate (BPF increasing over time) indicates approach.
pub fn is_approaching(doppler_rate: f64) -> bool {
    doppler_rate > 0.0
}

/// Estimate range to the drone using the inverse-square law.
///
/// Given a measured `signal_level_db`, a calibrated `reference_level_db` at
/// a known `reference_range_m`, the estimated range is:
///
/// ```text
/// range = reference_range × 10^((reference_level_db - signal_level_db) / 20)
/// ```
pub fn estimate_range(
    signal_level_db: f64,
    reference_level_db: f64,
    reference_range_m: f64,
) -> f64 {
    if reference_range_m <= 0.0 {
        return f64::INFINITY;
    }
    let delta_db = reference_level_db - signal_level_db;
    let range = reference_range_m * 10.0_f64.powf(delta_db / 20.0);
    range.max(0.0)
}

/// Compute a detection metric (SNR-like score) comparing the spectrum to a
/// noise floor estimate.
///
/// Returns a value in [0, infinity) where higher values indicate stronger drone-like
/// signatures. The metric is the average ratio of spectrum to noise floor
/// across all bins, minus 1 (so pure noise yields approximately 0).
pub fn compute_detection_metric(spectrum: &[f64], noise_floor: &[f64]) -> f64 {
    if spectrum.is_empty() || noise_floor.is_empty() {
        return 0.0;
    }

    let len = spectrum.len().min(noise_floor.len());
    let mut ratio_sum = 0.0;
    let mut count = 0usize;

    for i in 0..len {
        if noise_floor[i] > 1e-20 {
            ratio_sum += spectrum[i] / noise_floor[i];
            count += 1;
        }
    }

    if count == 0 {
        return 0.0;
    }

    let mean_ratio = ratio_sum / count as f64;
    (mean_ratio - 1.0).max(0.0)
}

/// Generate an alert if the detection is active and confidence exceeds the threshold.
pub fn generate_alert(detection: &DroneDetection, threshold: f64) -> bool {
    detection.detected && detection.confidence >= threshold
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Simple alert generation from metric value.
fn generate_alert_from_metric(metric: f64, threshold: f64) -> bool {
    metric >= threshold
}

/// Compute a simple magnitude spectrum using a brute-force DFT.
///
/// This is intentionally O(N^2) to avoid any FFT crate dependency. For
/// production use, replace with an FFT library.
fn simple_magnitude_spectrum(samples: &[f64]) -> Vec<f64> {
    let n = samples.len();
    if n == 0 {
        return Vec::new();
    }

    let half = n / 2 + 1;
    let mut spectrum = Vec::with_capacity(half);

    for k in 0..half {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &s) in samples.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += s * angle.cos();
            im += s * angle.sin();
        }
        spectrum.push((re * re + im * im).sqrt() / n as f64);
    }

    spectrum
}

/// Estimate the noise floor of a spectrum as its median value.
fn estimate_noise_floor(spectrum: &[f64]) -> f64 {
    if spectrum.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f64> = spectrum.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    sorted[sorted.len() / 2]
}

// ===========================================================================
// Tests
// ===========================================================================
#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a sinusoidal tone at the given frequency.
    fn generate_tone(freq_hz: f64, sample_rate: f64, num_samples: usize, amplitude: f64) -> Vec<f64> {
        (0..num_samples)
            .map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 / sample_rate).sin())
            .collect()
    }

    /// Helper: generate a signal with multiple harmonics.
    fn generate_harmonic_signal(
        fundamental_hz: f64,
        num_harmonics: usize,
        sample_rate: f64,
        num_samples: usize,
    ) -> Vec<f64> {
        let mut samples = vec![0.0; num_samples];
        for h in 1..=num_harmonics {
            let amplitude = 1.0 / h as f64;
            let freq = fundamental_hz * h as f64;
            for i in 0..num_samples {
                samples[i] += amplitude * (2.0 * PI * freq * i as f64 / sample_rate).sin();
            }
        }
        samples
    }

    #[test]
    fn test_drone_config_default() {
        let config = DroneConfig::default();
        assert_eq!(config.sample_rate_hz, 44100.0);
        assert_eq!(config.fft_size, 4096);
        assert_eq!(config.min_freq_hz, 50.0);
        assert_eq!(config.max_freq_hz, 2000.0);
        assert_eq!(config.num_blades, 2);
    }

    #[test]
    fn test_drone_detection_default() {
        let det = DroneDetection::default();
        assert!(!det.detected);
        assert_eq!(det.confidence, 0.0);
        assert_eq!(det.bpf_hz, 0.0);
        assert_eq!(det.rpm, 0.0);
        assert_eq!(det.num_harmonics, 0);
        assert!(det.estimated_range_m.is_infinite());
        assert!(!det.approaching);
    }

    #[test]
    fn test_estimate_rpm_two_blades() {
        let rpm = estimate_rpm(200.0, 2);
        assert!((rpm - 6000.0).abs() < 1e-6);
    }

    #[test]
    fn test_estimate_rpm_three_blades() {
        let rpm = estimate_rpm(300.0, 3);
        assert!((rpm - 6000.0).abs() < 1e-6);
    }

    #[test]
    fn test_estimate_rpm_zero_blades() {
        let rpm = estimate_rpm(200.0, 0);
        assert_eq!(rpm, 0.0);
    }

    #[test]
    fn test_detect_bpf_single_tone() {
        let sample_rate = 8000.0;
        let n = 1024;
        let tone_freq = 200.0;
        let samples = generate_tone(tone_freq, sample_rate, n, 1.0);
        let spectrum = simple_magnitude_spectrum(&samples);
        let freq_res = sample_rate / n as f64;

        let bpf = detect_bpf(&spectrum, freq_res, 50.0, 1000.0);
        assert!(bpf.is_some());
        let bpf = bpf.unwrap();
        assert!((bpf - tone_freq).abs() < freq_res * 1.5);
    }

    #[test]
    fn test_detect_bpf_empty_spectrum() {
        let result = detect_bpf(&[], 10.0, 50.0, 1000.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_bpf_zero_resolution() {
        let spectrum = vec![1.0; 100];
        let result = detect_bpf(&spectrum, 0.0, 50.0, 1000.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_extract_harmonics_basic() {
        let sample_rate = 8000.0;
        let n = 2048;
        let fundamental = 200.0;
        let samples = generate_harmonic_signal(fundamental, 4, sample_rate, n);
        let spectrum = simple_magnitude_spectrum(&samples);
        let freq_res = sample_rate / n as f64;

        let harmonics = extract_harmonics(&spectrum, fundamental, freq_res, 6);
        assert!(!harmonics.is_empty(), "Should detect at least one harmonic");

        let first_freq = harmonics[0].0;
        assert!(
            (first_freq - fundamental).abs() < freq_res * 2.0,
            "First harmonic {first_freq} should be near fundamental {fundamental}"
        );
    }

    #[test]
    fn test_extract_harmonics_empty() {
        let result = extract_harmonics(&[], 200.0, 10.0, 5);
        assert!(result.is_empty());
    }

    #[test]
    fn test_extract_harmonics_zero_fundamental() {
        let spectrum = vec![1.0; 100];
        let result = extract_harmonics(&spectrum, 0.0, 10.0, 5);
        assert!(result.is_empty());
    }

    #[test]
    fn test_classify_drone_type_quad_rotor() {
        let harmonics = vec![(200.0, 1.0), (400.0, 0.5), (600.0, 0.25)];
        let dtype = classify_drone_type(&harmonics, 200.0);
        assert_eq!(dtype, DroneType::QuadRotor);
    }

    #[test]
    fn test_classify_drone_type_fixed_wing() {
        let harmonics = vec![(80.0, 1.0)];
        let dtype = classify_drone_type(&harmonics, 80.0);
        assert_eq!(dtype, DroneType::FixedWing);
    }

    #[test]
    fn test_classify_drone_type_hex_rotor() {
        let harmonics = vec![
            (350.0, 1.0),
            (700.0, 0.7),
            (1050.0, 0.4),
            (1400.0, 0.2),
        ];
        let dtype = classify_drone_type(&harmonics, 350.0);
        assert_eq!(dtype, DroneType::HexaRotor);
    }

    #[test]
    fn test_classify_drone_type_unknown() {
        let harmonics: Vec<(f64, f64)> = vec![];
        let dtype = classify_drone_type(&harmonics, 200.0);
        assert_eq!(dtype, DroneType::Unknown);
    }

    #[test]
    fn test_detect_doppler_shift_increasing() {
        let history: Vec<f64> = (0..10).map(|i| 200.0 + i as f64 * 0.5).collect();
        let rate = detect_doppler_shift(&history);
        assert!(rate > 0.0, "Doppler rate should be positive for increasing BPF");
        assert!((rate - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_detect_doppler_shift_decreasing() {
        let history: Vec<f64> = (0..10).map(|i| 200.0 - i as f64 * 0.3).collect();
        let rate = detect_doppler_shift(&history);
        assert!(rate < 0.0, "Doppler rate should be negative for decreasing BPF");
    }

    #[test]
    fn test_detect_doppler_shift_constant() {
        let history = vec![200.0; 10];
        let rate = detect_doppler_shift(&history);
        assert!(rate.abs() < 1e-10, "Doppler rate should be ~0 for constant BPF");
    }

    #[test]
    fn test_detect_doppler_shift_too_short() {
        let rate = detect_doppler_shift(&[200.0]);
        assert_eq!(rate, 0.0);
    }

    #[test]
    fn test_is_approaching() {
        assert!(is_approaching(0.5));
        assert!(!is_approaching(-0.5));
        assert!(!is_approaching(0.0));
    }

    #[test]
    fn test_estimate_range_same_level() {
        let range = estimate_range(-40.0, -40.0, 10.0);
        assert!((range - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_estimate_range_6db_weaker() {
        let range = estimate_range(-46.0, -40.0, 10.0);
        assert!((range - 20.0).abs() < 0.5);
    }

    #[test]
    fn test_estimate_range_zero_reference() {
        let range = estimate_range(-40.0, -30.0, 0.0);
        assert!(range.is_infinite());
    }

    #[test]
    fn test_compute_detection_metric_pure_noise() {
        let spectrum = vec![1.0; 100];
        let noise_floor = vec![1.0; 100];
        let metric = compute_detection_metric(&spectrum, &noise_floor);
        assert!(metric.abs() < 1e-6);
    }

    #[test]
    fn test_compute_detection_metric_signal_present() {
        let spectrum = vec![3.0; 100];
        let noise_floor = vec![1.0; 100];
        let metric = compute_detection_metric(&spectrum, &noise_floor);
        assert!((metric - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_detection_metric_empty() {
        assert_eq!(compute_detection_metric(&[], &[]), 0.0);
    }

    #[test]
    fn test_generate_alert_true() {
        let det = DroneDetection {
            detected: true,
            confidence: 0.9,
            ..DroneDetection::default()
        };
        assert!(generate_alert(&det, 0.8));
    }

    #[test]
    fn test_generate_alert_below_threshold() {
        let det = DroneDetection {
            detected: true,
            confidence: 0.5,
            ..DroneDetection::default()
        };
        assert!(!generate_alert(&det, 0.8));
    }

    #[test]
    fn test_generate_alert_not_detected() {
        let det = DroneDetection {
            detected: false,
            confidence: 0.9,
            ..DroneDetection::default()
        };
        assert!(!generate_alert(&det, 0.5));
    }

    #[test]
    fn test_drone_detector_process_tone() {
        let sample_rate = 8000.0;
        let n = 2048;
        let tone_freq = 200.0;
        let samples = generate_harmonic_signal(tone_freq, 4, sample_rate, n);

        let config = DroneConfig {
            sample_rate_hz: sample_rate,
            fft_size: n,
            min_freq_hz: 50.0,
            max_freq_hz: 2000.0,
            num_blades: 2,
        };

        let mut detector = DroneDetector::new(config);
        let detection = detector.process_frame(&samples, -20.0, 10.0, 0.1);

        let freq_res = sample_rate / n as f64;
        assert!(
            (detection.bpf_hz - tone_freq).abs() < freq_res * 2.0,
            "BPF {} should be near {} Hz",
            detection.bpf_hz,
            tone_freq
        );

        assert!(
            (detection.rpm - 6000.0).abs() < freq_res * 60.0,
            "RPM {} should be near 6000",
            detection.rpm
        );
    }

    #[test]
    fn test_drone_detector_empty_frame() {
        let config = DroneConfig::default();
        let mut detector = DroneDetector::new(config);
        let detection = detector.process_frame(&[], -20.0, 10.0, 0.5);
        assert!(!detection.detected);
    }

    #[test]
    fn test_drone_detector_reset() {
        let config = DroneConfig::default();
        let mut detector = DroneDetector::new(config);
        detector.bpf_history.push(100.0);
        detector.bpf_history.push(101.0);
        assert_eq!(detector.bpf_history().len(), 2);
        detector.reset();
        assert!(detector.bpf_history().is_empty());
    }

    #[test]
    fn test_classify_drone_type_octo_rotor() {
        let harmonics = vec![
            (100.0, 0.2),
            (200.0, 1.0),
            (300.0, 0.2),
            (400.0, 0.9),
            (500.0, 0.2),
            (600.0, 0.8),
        ];
        let dtype = classify_drone_type(&harmonics, 100.0);
        assert_eq!(dtype, DroneType::OctoRotor);
    }

    #[test]
    fn test_simple_magnitude_spectrum_dc() {
        let samples = vec![1.0; 64];
        let spectrum = simple_magnitude_spectrum(&samples);
        assert!(!spectrum.is_empty());
        let dc = spectrum[0];
        for &val in &spectrum[1..] {
            assert!(dc >= val - 1e-10);
        }
    }

    #[test]
    fn test_estimate_noise_floor() {
        let spectrum = vec![1.0, 2.0, 3.0, 100.0, 5.0];
        let nf = estimate_noise_floor(&spectrum);
        assert!((nf - 3.0).abs() < 1e-10);
    }
}

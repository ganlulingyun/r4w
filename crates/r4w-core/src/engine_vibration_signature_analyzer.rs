//! Aircraft/turbine engine vibration signature analysis for health monitoring and prognostics.
//!
//! This module provides tools for analyzing vibration signatures from aircraft and turbine
//! engines to detect faults, monitor health, and estimate remaining useful life. Unlike
//! general-purpose vibration analysis (e.g., ISO 10816 for wind turbines), this module
//! focuses on rotating machinery diagnostics specific to gas turbine and turbofan engines:
//!
//! - **Shaft order analysis**: Decompose vibration into multiples of shaft rotational frequency
//! - **Rotor imbalance detection**: Monitor 1X (once-per-revolution) vibration amplitude
//! - **Misalignment detection**: Monitor 2X (twice-per-revolution) vibration amplitude
//! - **Blade pass frequency (BPF)**: Track vibration at N × RPM where N = number of blades
//! - **Bearing defect frequencies**: Calculate BPFO, BPFI, BSF, and FTF from bearing geometry
//! - **Blade tip timing**: Detect individual blade deflections from arrival time deviations
//! - **Spectral baseline comparison**: Quantify deviation from a known-good reference spectrum
//! - **Remaining life estimation**: Extrapolate degradation trends to predict failure
//!
//! # Example
//!
//! ```
//! use r4w_core::engine_vibration_signature_analyzer::{
//!     EngineConfig, EngineAnalyzer, blade_pass_frequency, overall_vibration_level,
//! };
//!
//! let config = EngineConfig {
//!     shaft_rpm: 12000.0,
//!     num_blades: 24,
//!     num_stages: 3,
//!     sample_rate_hz: 51200.0,
//! };
//!
//! // Generate a simple test signal (1X + noise)
//! let shaft_freq = config.shaft_rpm / 60.0; // 200 Hz
//! let dt = 1.0 / config.sample_rate_hz;
//! let signal: Vec<f64> = (0..51200)
//!     .map(|i| {
//!         let t = i as f64 * dt;
//!         0.5 * (2.0 * std::f64::consts::PI * shaft_freq * t).sin()
//!     })
//!     .collect();
//!
//! let analyzer = EngineAnalyzer::new(config);
//! let health = analyzer.analyze(&signal);
//! assert!(health.overall_level_g > 0.0);
//! assert!(health.health_score >= 0.0 && health.health_score <= 100.0);
//!
//! let bpf = blade_pass_frequency(12000.0, 24);
//! assert!((bpf - 4800.0).abs() < 1e-9);
//!
//! let rms = overall_vibration_level(&signal);
//! assert!(rms > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration and result types
// ---------------------------------------------------------------------------

/// Engine configuration parameters for vibration analysis.
#[derive(Debug, Clone)]
pub struct EngineConfig {
    /// Shaft rotational speed in revolutions per minute.
    pub shaft_rpm: f64,
    /// Number of blades on the monitored rotor stage.
    pub num_blades: u32,
    /// Number of compressor/turbine stages.
    pub num_stages: u32,
    /// Vibration sensor sample rate in Hz.
    pub sample_rate_hz: f64,
}

/// Summary of engine vibration health indicators.
#[derive(Debug, Clone)]
pub struct VibrationHealth {
    /// Root-mean-square vibration level in g.
    pub overall_level_g: f64,
    /// Amplitude at the 1X (shaft frequency) component in g.
    pub imbalance_1x_g: f64,
    /// Amplitude at the 2X (twice shaft frequency) component in g.
    pub misalignment_2x_g: f64,
    /// Amplitude at the blade pass frequency in g.
    pub blade_pass_g: f64,
    /// Composite health score from 0 (critical) to 100 (healthy).
    pub health_score: f64,
}

/// Enumeration of common rotating-machinery fault types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultType {
    /// No significant fault detected.
    Normal,
    /// Rotor mass imbalance (dominant 1X vibration).
    Imbalance,
    /// Shaft or coupling misalignment (dominant 2X vibration).
    Misalignment,
    /// Rolling-element bearing defect (characteristic defect frequencies).
    BearingDefect,
    /// Blade crack, erosion, or foreign-object damage.
    BladeDamage,
    /// Mechanical looseness (broadband sub-harmonics and harmonics).
    Looseness,
    /// Fault signature does not match known patterns.
    Unknown,
}

// ---------------------------------------------------------------------------
// Standalone utility functions
// ---------------------------------------------------------------------------

/// Compute the order spectrum of `signal` given a constant `rpm`.
///
/// Returns a vector of `(order, amplitude)` pairs from order 0 up to
/// `max_order` (inclusive integer orders).  Each amplitude is the magnitude of
/// the DFT bin closest to that order's frequency.
///
/// The order-frequency relationship is: `f = order * rpm / 60`.
pub fn order_spectrum(signal: &[f64], rpm: f64, sample_rate: f64, max_order: f64) -> Vec<(f64, f64)> {
    if signal.is_empty() || rpm <= 0.0 || sample_rate <= 0.0 || max_order <= 0.0 {
        return Vec::new();
    }

    let n = signal.len();
    let spectrum = real_fft(signal);
    let freq_resolution = sample_rate / n as f64;
    let shaft_freq = rpm / 60.0;

    let mut results = Vec::new();
    let mut order = 0.0_f64;
    while order <= max_order {
        let target_freq = order * shaft_freq;
        let bin = (target_freq / freq_resolution).round() as usize;
        let amplitude = if bin < spectrum.len() { spectrum[bin] } else { 0.0 };
        results.push((order, amplitude));
        order += 1.0;
    }
    results
}

/// Detect rotor imbalance by checking the 1X component amplitude.
///
/// Returns `(detected, amplitude)` where `detected` is `true` when the 1X
/// amplitude exceeds 0.1 g (a common alert threshold for rotating machinery).
pub fn detect_imbalance(spectrum: &[(f64, f64)], _rpm: f64) -> (bool, f64) {
    let amp_1x = spectrum
        .iter()
        .find(|(order, _)| (*order - 1.0).abs() < 0.5)
        .map(|(_, a)| *a)
        .unwrap_or(0.0);

    let threshold = 0.1; // g
    (amp_1x > threshold, amp_1x)
}

/// Detect shaft/coupling misalignment by checking the 2X component amplitude.
///
/// Returns `(detected, amplitude)` where `detected` is `true` when the 2X
/// amplitude exceeds 0.08 g.
pub fn detect_misalignment(spectrum: &[(f64, f64)], _rpm: f64) -> (bool, f64) {
    let amp_2x = spectrum
        .iter()
        .find(|(order, _)| (*order - 2.0).abs() < 0.5)
        .map(|(_, a)| *a)
        .unwrap_or(0.0);

    let threshold = 0.08; // g
    (amp_2x > threshold, amp_2x)
}

/// Calculate the blade pass frequency in Hz.
///
/// `bpf = rpm / 60 * num_blades`
pub fn blade_pass_frequency(rpm: f64, num_blades: u32) -> f64 {
    rpm / 60.0 * num_blades as f64
}

/// Compute blade tip timing deviations between expected and measured arrival times.
///
/// Returns a vector of deviations (`measured - expected`) in the same time
/// units as the inputs.  The length equals the minimum of the two slices.
pub fn blade_tip_timing_deviation(expected_times: &[f64], measured_times: &[f64]) -> Vec<f64> {
    expected_times
        .iter()
        .zip(measured_times.iter())
        .map(|(e, m)| m - e)
        .collect()
}

/// Calculate rolling-element bearing characteristic defect frequencies.
///
/// Given bearing geometry and shaft RPM, returns `(bpfo, bpfi, bsf, ftf)`:
///
/// - **BPFO** -- Ball Pass Frequency Outer race
/// - **BPFI** -- Ball Pass Frequency Inner race
/// - **BSF**  -- Ball Spin Frequency
/// - **FTF**  -- Fundamental Train (cage) Frequency
pub fn bearing_fault_freqs(
    num_elements: u8,
    ball_dia_mm: f64,
    pitch_dia_mm: f64,
    contact_angle_deg: f64,
    rpm: f64,
) -> (f64, f64, f64, f64) {
    let n = num_elements as f64;
    let bd = ball_dia_mm;
    let pd = pitch_dia_mm;
    let phi = contact_angle_deg * PI / 180.0;
    let shaft_freq = rpm / 60.0;
    let ratio = bd / pd;
    let cos_phi = phi.cos();

    let ftf = shaft_freq / 2.0 * (1.0 - ratio * cos_phi);
    let bpfo = n / 2.0 * shaft_freq * (1.0 - ratio * cos_phi);
    let bpfi = n / 2.0 * shaft_freq * (1.0 + ratio * cos_phi);
    let bsf = pd / (2.0 * bd) * shaft_freq * (1.0 - (ratio * cos_phi).powi(2));

    (bpfo, bpfi, bsf, ftf)
}

/// Compute the RMS (root-mean-square) vibration level in g.
pub fn overall_vibration_level(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = signal.iter().map(|&x| x * x).sum();
    (sum_sq / signal.len() as f64).sqrt()
}

/// Compute spectral similarity between `current` and `baseline` signals.
///
/// Returns a value in `[0.0, 1.0]` where 1.0 means identical spectra (cosine
/// similarity of magnitude spectra).
pub fn spectral_comparison(current: &[f64], baseline: &[f64]) -> f64 {
    if current.is_empty() || baseline.is_empty() {
        return 0.0;
    }

    let spec_a = real_fft(current);
    let spec_b = real_fft(baseline);
    let len = spec_a.len().min(spec_b.len());
    if len == 0 {
        return 0.0;
    }

    let mut dot = 0.0_f64;
    let mut norm_a = 0.0_f64;
    let mut norm_b = 0.0_f64;
    for i in 0..len {
        dot += spec_a[i] * spec_b[i];
        norm_a += spec_a[i] * spec_a[i];
        norm_b += spec_b[i] * spec_b[i];
    }

    let denom = norm_a.sqrt() * norm_b.sqrt();
    if denom < 1e-30 {
        return 0.0;
    }
    (dot / denom).clamp(0.0, 1.0)
}

/// Classify the most likely fault type from vibration health indicators.
///
/// Uses simple threshold-based rules typical of Level-1 machinery diagnostics:
/// - 1X dominant -> Imbalance
/// - 2X dominant -> Misalignment
/// - Blade pass elevated -> BladeDamage
/// - Broadband high but no dominant order -> Looseness
pub fn diagnose_fault(health: &VibrationHealth) -> FaultType {
    // Thresholds (g) -- representative of small turbomachinery
    const IMBALANCE_THRESH: f64 = 0.10;
    const MISALIGN_THRESH: f64 = 0.08;
    const BLADE_THRESH: f64 = 0.05;
    const LOOSENESS_OVERALL: f64 = 0.30;

    if health.imbalance_1x_g > IMBALANCE_THRESH
        && health.imbalance_1x_g > health.misalignment_2x_g
        && health.imbalance_1x_g > health.blade_pass_g
    {
        return FaultType::Imbalance;
    }

    if health.misalignment_2x_g > MISALIGN_THRESH
        && health.misalignment_2x_g > health.imbalance_1x_g
    {
        return FaultType::Misalignment;
    }

    if health.blade_pass_g > BLADE_THRESH
        && health.blade_pass_g > health.imbalance_1x_g
        && health.blade_pass_g > health.misalignment_2x_g
    {
        return FaultType::BladeDamage;
    }

    if health.overall_level_g > LOOSENESS_OVERALL
        && health.imbalance_1x_g < IMBALANCE_THRESH
        && health.misalignment_2x_g < MISALIGN_THRESH
        && health.blade_pass_g < BLADE_THRESH
    {
        return FaultType::Looseness;
    }

    if health.health_score >= 80.0 {
        return FaultType::Normal;
    }

    FaultType::Unknown
}

/// Estimate remaining useful life by linear extrapolation of a degradation trend.
///
/// Given a series of periodic vibration level measurements (`trend`) and a
/// failure `threshold`, fits a line to the most recent half of the data and
/// projects when the threshold will be reached.
///
/// Returns `Some(remaining_periods)` if the trend is increasing toward the
/// threshold, or `None` if the trend is flat/decreasing or already above.
pub fn estimate_remaining_life(trend: &[f64], threshold: f64) -> Option<f64> {
    if trend.len() < 2 {
        return None;
    }

    // Use the most recent half (at least 2 points) for the fit
    let start = if trend.len() > 4 { trend.len() / 2 } else { 0 };
    let window = &trend[start..];

    let n = window.len() as f64;
    let mut sum_x = 0.0_f64;
    let mut sum_y = 0.0_f64;
    let mut sum_xy = 0.0_f64;
    let mut sum_xx = 0.0_f64;

    for (i, &y) in window.iter().enumerate() {
        let x = (start + i) as f64;
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return None;
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;

    // Slope must be positive (degradation increasing) and current value below threshold
    let last = *trend.last().unwrap();
    if slope <= 0.0 || last >= threshold {
        return None;
    }

    // x_cross = (threshold - intercept) / slope
    let x_cross = (threshold - intercept) / slope;
    let current_x = (trend.len() - 1) as f64;
    let remaining = x_cross - current_x;

    if remaining > 0.0 {
        Some(remaining)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Engine analyzer
// ---------------------------------------------------------------------------

/// Main engine vibration analyzer.
///
/// Combines order analysis, fault detection, and health scoring into a single
/// analysis pass.
#[derive(Debug, Clone)]
pub struct EngineAnalyzer {
    config: EngineConfig,
}

impl EngineAnalyzer {
    /// Create a new analyzer for the given engine configuration.
    pub fn new(config: EngineConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current engine configuration.
    pub fn config(&self) -> &EngineConfig {
        &self.config
    }

    /// Perform a full vibration analysis on the time-domain `signal`.
    ///
    /// Returns a [`VibrationHealth`] summary including overall level, 1X, 2X,
    /// blade-pass amplitudes, and a composite health score.
    pub fn analyze(&self, signal: &[f64]) -> VibrationHealth {
        if signal.is_empty() {
            return VibrationHealth {
                overall_level_g: 0.0,
                imbalance_1x_g: 0.0,
                misalignment_2x_g: 0.0,
                blade_pass_g: 0.0,
                health_score: 100.0,
            };
        }

        let rpm = self.config.shaft_rpm;
        let sr = self.config.sample_rate_hz;
        let blades = self.config.num_blades;

        let overall = overall_vibration_level(signal);

        // Compute spectrum up to blade-pass order + margin
        let bpf_order = blades as f64;
        let max_order = (bpf_order + 4.0).max(10.0);
        let orders = order_spectrum(signal, rpm, sr, max_order);

        let (_, amp_1x) = detect_imbalance(&orders, rpm);
        let (_, amp_2x) = detect_misalignment(&orders, rpm);

        // Blade pass amplitude: find the order closest to num_blades
        let blade_pass_g = orders
            .iter()
            .find(|(o, _)| (*o - bpf_order).abs() < 0.5)
            .map(|(_, a)| *a)
            .unwrap_or(0.0);

        // Composite health score: 100 when everything is low, decreasing with vibration
        let penalty_1x = (amp_1x / 0.10).min(1.0) * 30.0;
        let penalty_2x = (amp_2x / 0.08).min(1.0) * 25.0;
        let penalty_bp = (blade_pass_g / 0.05).min(1.0) * 20.0;
        let penalty_overall = (overall / 0.50).min(1.0) * 25.0;
        let score = (100.0 - penalty_1x - penalty_2x - penalty_bp - penalty_overall).max(0.0);

        VibrationHealth {
            overall_level_g: overall,
            imbalance_1x_g: amp_1x,
            misalignment_2x_g: amp_2x,
            blade_pass_g,
            health_score: score,
        }
    }

    /// Analyze the signal and return the diagnosed fault type.
    pub fn diagnose(&self, signal: &[f64]) -> FaultType {
        let health = self.analyze(signal);
        diagnose_fault(&health)
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Simple magnitude-spectrum via DFT (no external crate).
///
/// Returns magnitudes for bins 0..=N/2 (positive frequencies only), each
/// normalised by N so that a unit-amplitude sinusoid yields ~0.5 in its bin.
fn real_fft(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }
    let half = n / 2 + 1;
    let mut magnitudes = Vec::with_capacity(half);

    for k in 0..half {
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        let freq = 2.0 * PI * k as f64 / n as f64;
        for (i, &x) in signal.iter().enumerate() {
            let angle = freq * i as f64;
            re += x * angle.cos();
            im -= x * angle.sin();
        }
        let mag = (re * re + im * im).sqrt() / n as f64;
        magnitudes.push(mag);
    }

    magnitudes
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a sinusoidal signal at `freq_hz` with given amplitude.
    fn sine_signal(freq_hz: f64, amplitude: f64, sample_rate: f64, duration_s: f64) -> Vec<f64> {
        let n = (sample_rate * duration_s) as usize;
        let dt = 1.0 / sample_rate;
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 * dt).sin())
            .collect()
    }

    /// Helper: default engine config for tests.
    fn test_config() -> EngineConfig {
        EngineConfig {
            shaft_rpm: 6000.0,  // 100 Hz shaft freq
            num_blades: 20,
            num_stages: 2,
            sample_rate_hz: 10240.0,
        }
    }

    // -- order_spectrum tests -----------------------------------------------

    #[test]
    fn test_order_spectrum_pure_1x() {
        let cfg = test_config();
        let shaft_freq = cfg.shaft_rpm / 60.0; // 100 Hz
        let signal = sine_signal(shaft_freq, 1.0, cfg.sample_rate_hz, 1.0);
        let orders = order_spectrum(&signal, cfg.shaft_rpm, cfg.sample_rate_hz, 5.0);

        // Order 1 should have the highest amplitude
        let amp_1 = orders.iter().find(|(o, _)| (*o - 1.0).abs() < 0.01).unwrap().1;
        let amp_0 = orders.iter().find(|(o, _)| *o < 0.5).unwrap().1;
        let amp_2 = orders.iter().find(|(o, _)| (*o - 2.0).abs() < 0.01).unwrap().1;
        assert!(amp_1 > amp_0 * 10.0, "1X should dominate DC");
        assert!(amp_1 > amp_2 * 10.0, "1X should dominate 2X");
    }

    #[test]
    fn test_order_spectrum_empty_signal() {
        let result = order_spectrum(&[], 6000.0, 10240.0, 5.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_order_spectrum_bad_params() {
        let signal = vec![0.0; 100];
        assert!(order_spectrum(&signal, 0.0, 10240.0, 5.0).is_empty());
        assert!(order_spectrum(&signal, 6000.0, 0.0, 5.0).is_empty());
        assert!(order_spectrum(&signal, 6000.0, 10240.0, 0.0).is_empty());
    }

    // -- detect_imbalance tests ---------------------------------------------

    #[test]
    fn test_detect_imbalance_high() {
        let cfg = test_config();
        let shaft_freq = cfg.shaft_rpm / 60.0;
        let signal = sine_signal(shaft_freq, 1.0, cfg.sample_rate_hz, 1.0);
        let orders = order_spectrum(&signal, cfg.shaft_rpm, cfg.sample_rate_hz, 5.0);
        let (detected, amp) = detect_imbalance(&orders, cfg.shaft_rpm);
        assert!(detected, "Should detect imbalance for large 1X");
        assert!(amp > 0.1);
    }

    #[test]
    fn test_detect_imbalance_low() {
        let cfg = test_config();
        let shaft_freq = cfg.shaft_rpm / 60.0;
        // Very small amplitude -> should NOT detect
        let signal = sine_signal(shaft_freq, 0.001, cfg.sample_rate_hz, 1.0);
        let orders = order_spectrum(&signal, cfg.shaft_rpm, cfg.sample_rate_hz, 5.0);
        let (detected, _) = detect_imbalance(&orders, cfg.shaft_rpm);
        assert!(!detected, "Should not detect imbalance for tiny 1X");
    }

    // -- detect_misalignment tests ------------------------------------------

    #[test]
    fn test_detect_misalignment_high() {
        let cfg = test_config();
        let shaft_freq = cfg.shaft_rpm / 60.0;
        let signal = sine_signal(2.0 * shaft_freq, 0.5, cfg.sample_rate_hz, 1.0);
        let orders = order_spectrum(&signal, cfg.shaft_rpm, cfg.sample_rate_hz, 5.0);
        let (detected, amp) = detect_misalignment(&orders, cfg.shaft_rpm);
        assert!(detected, "Should detect misalignment for large 2X");
        assert!(amp > 0.08);
    }

    #[test]
    fn test_detect_misalignment_low() {
        let cfg = test_config();
        let shaft_freq = cfg.shaft_rpm / 60.0;
        let signal = sine_signal(2.0 * shaft_freq, 0.001, cfg.sample_rate_hz, 1.0);
        let orders = order_spectrum(&signal, cfg.shaft_rpm, cfg.sample_rate_hz, 5.0);
        let (detected, _) = detect_misalignment(&orders, cfg.shaft_rpm);
        assert!(!detected);
    }

    // -- blade_pass_frequency tests -----------------------------------------

    #[test]
    fn test_blade_pass_frequency_basic() {
        // 6000 RPM, 20 blades -> 100 Hz x 20 = 2000 Hz
        assert!((blade_pass_frequency(6000.0, 20) - 2000.0).abs() < 1e-9);
    }

    #[test]
    fn test_blade_pass_frequency_high_speed() {
        // 30000 RPM, 36 blades -> 500 Hz x 36 = 18000 Hz
        assert!((blade_pass_frequency(30000.0, 36) - 18000.0).abs() < 1e-9);
    }

    // -- blade_tip_timing_deviation tests -----------------------------------

    #[test]
    fn test_blade_tip_timing_no_deviation() {
        let expected = vec![0.0, 0.001, 0.002, 0.003];
        let measured = vec![0.0, 0.001, 0.002, 0.003];
        let devs = blade_tip_timing_deviation(&expected, &measured);
        assert_eq!(devs.len(), 4);
        for d in &devs {
            assert!(d.abs() < 1e-15);
        }
    }

    #[test]
    fn test_blade_tip_timing_with_deviation() {
        let expected = vec![0.0, 0.001, 0.002];
        let measured = vec![0.0001, 0.0012, 0.0019];
        let devs = blade_tip_timing_deviation(&expected, &measured);
        assert_eq!(devs.len(), 3);
        assert!((devs[0] - 0.0001).abs() < 1e-10);
        assert!((devs[1] - 0.0002).abs() < 1e-10);
        assert!((devs[2] - (-0.0001)).abs() < 1e-10);
    }

    #[test]
    fn test_blade_tip_timing_unequal_lengths() {
        let expected = vec![0.0, 0.001, 0.002, 0.003, 0.004];
        let measured = vec![0.0, 0.001, 0.002];
        let devs = blade_tip_timing_deviation(&expected, &measured);
        assert_eq!(devs.len(), 3); // minimum length
    }

    // -- bearing_fault_freqs tests ------------------------------------------

    #[test]
    fn test_bearing_fault_freqs_zero_contact_angle() {
        // Simplified: contact_angle = 0 -> cos(0) = 1
        let (bpfo, bpfi, bsf, ftf) = bearing_fault_freqs(
            8,    // 8 balls
            10.0, // ball dia mm
            50.0, // pitch dia mm
            0.0,  // contact angle
            6000.0,
        );
        let shaft_freq = 100.0; // 6000/60
        let ratio = 10.0 / 50.0; // 0.2

        let expected_ftf = shaft_freq / 2.0 * (1.0 - ratio);
        let expected_bpfo = 4.0 * shaft_freq * (1.0 - ratio);
        let expected_bpfi = 4.0 * shaft_freq * (1.0 + ratio);
        let expected_bsf = 50.0 / 20.0 * shaft_freq * (1.0 - ratio * ratio);

        assert!((ftf - expected_ftf).abs() < 1e-6);
        assert!((bpfo - expected_bpfo).abs() < 1e-6);
        assert!((bpfi - expected_bpfi).abs() < 1e-6);
        assert!((bsf - expected_bsf).abs() < 1e-6);
    }

    #[test]
    fn test_bearing_fault_freqs_nonzero_angle() {
        let (bpfo, bpfi, bsf, ftf) = bearing_fault_freqs(
            12,   // elements
            8.0,  // ball dia
            40.0, // pitch dia
            15.0, // contact angle degrees
            12000.0,
        );
        // Just check reasonable ordering: BPFI > BPFO, BSF > 0, FTF > 0
        assert!(bpfi > bpfo, "BPFI should be larger than BPFO");
        assert!(bsf > 0.0);
        assert!(ftf > 0.0);
        assert!(ftf < bpfo, "FTF (cage) should be less than BPFO");
    }

    // -- overall_vibration_level tests --------------------------------------

    #[test]
    fn test_overall_vibration_level_sine() {
        let signal = sine_signal(100.0, 1.0, 10240.0, 1.0);
        let rms = overall_vibration_level(&signal);
        // RMS of a sine with amplitude 1.0 = 1/sqrt(2) ~ 0.7071
        assert!((rms - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.01);
    }

    #[test]
    fn test_overall_vibration_level_empty() {
        assert_eq!(overall_vibration_level(&[]), 0.0);
    }

    #[test]
    fn test_overall_vibration_level_dc() {
        let signal = vec![3.0; 100];
        let rms = overall_vibration_level(&signal);
        assert!((rms - 3.0).abs() < 1e-9);
    }

    // -- spectral_comparison tests ------------------------------------------

    #[test]
    fn test_spectral_comparison_identical() {
        let sig = sine_signal(200.0, 1.0, 10240.0, 0.1);
        let sim = spectral_comparison(&sig, &sig);
        assert!((sim - 1.0).abs() < 1e-6, "Identical signals should have similarity ~1.0");
    }

    #[test]
    fn test_spectral_comparison_different() {
        let sig_a = sine_signal(200.0, 1.0, 10240.0, 0.1);
        let sig_b = sine_signal(2000.0, 1.0, 10240.0, 0.1);
        let sim = spectral_comparison(&sig_a, &sig_b);
        assert!(sim < 0.5, "Different freq signals should have low similarity, got {sim}");
    }

    #[test]
    fn test_spectral_comparison_empty() {
        assert_eq!(spectral_comparison(&[], &[1.0, 2.0]), 0.0);
        assert_eq!(spectral_comparison(&[1.0], &[]), 0.0);
    }

    // -- diagnose_fault tests -----------------------------------------------

    #[test]
    fn test_diagnose_fault_normal() {
        let health = VibrationHealth {
            overall_level_g: 0.02,
            imbalance_1x_g: 0.01,
            misalignment_2x_g: 0.005,
            blade_pass_g: 0.003,
            health_score: 95.0,
        };
        assert_eq!(diagnose_fault(&health), FaultType::Normal);
    }

    #[test]
    fn test_diagnose_fault_imbalance() {
        let health = VibrationHealth {
            overall_level_g: 0.20,
            imbalance_1x_g: 0.25,
            misalignment_2x_g: 0.02,
            blade_pass_g: 0.01,
            health_score: 40.0,
        };
        assert_eq!(diagnose_fault(&health), FaultType::Imbalance);
    }

    #[test]
    fn test_diagnose_fault_misalignment() {
        let health = VibrationHealth {
            overall_level_g: 0.15,
            imbalance_1x_g: 0.03,
            misalignment_2x_g: 0.20,
            blade_pass_g: 0.01,
            health_score: 40.0,
        };
        assert_eq!(diagnose_fault(&health), FaultType::Misalignment);
    }

    #[test]
    fn test_diagnose_fault_blade_damage() {
        let health = VibrationHealth {
            overall_level_g: 0.10,
            imbalance_1x_g: 0.02,
            misalignment_2x_g: 0.02,
            blade_pass_g: 0.15,
            health_score: 50.0,
        };
        assert_eq!(diagnose_fault(&health), FaultType::BladeDamage);
    }

    #[test]
    fn test_diagnose_fault_looseness() {
        let health = VibrationHealth {
            overall_level_g: 0.50,
            imbalance_1x_g: 0.05,
            misalignment_2x_g: 0.03,
            blade_pass_g: 0.02,
            health_score: 30.0,
        };
        assert_eq!(diagnose_fault(&health), FaultType::Looseness);
    }

    // -- estimate_remaining_life tests --------------------------------------

    #[test]
    fn test_remaining_life_increasing_trend() {
        // Linearly increasing: 0.1, 0.2, 0.3, 0.4, threshold at 1.0
        let trend = vec![0.1, 0.2, 0.3, 0.4];
        let result = estimate_remaining_life(&trend, 1.0);
        assert!(result.is_some());
        let remaining = result.unwrap();
        assert!(remaining > 0.0, "Should predict positive remaining life");
    }

    #[test]
    fn test_remaining_life_flat_trend() {
        let trend = vec![0.2, 0.2, 0.2, 0.2, 0.2];
        let result = estimate_remaining_life(&trend, 1.0);
        assert!(result.is_none(), "Flat trend should return None");
    }

    #[test]
    fn test_remaining_life_too_short() {
        let trend = vec![0.5];
        assert!(estimate_remaining_life(&trend, 1.0).is_none());
    }

    #[test]
    fn test_remaining_life_already_exceeded() {
        let trend = vec![0.5, 0.7, 0.9, 1.1];
        let result = estimate_remaining_life(&trend, 1.0);
        // Last value is above threshold -> None
        assert!(result.is_none());
    }

    // -- EngineAnalyzer integration tests -----------------------------------

    #[test]
    fn test_analyzer_clean_signal() {
        let cfg = test_config();
        let analyzer = EngineAnalyzer::new(cfg.clone());
        // Very small vibration -> healthy
        let signal = sine_signal(cfg.shaft_rpm / 60.0, 0.001, cfg.sample_rate_hz, 1.0);
        let health = analyzer.analyze(&signal);
        assert!(health.health_score > 80.0, "Clean signal should score > 80, got {}", health.health_score);
        assert_eq!(analyzer.diagnose(&signal), FaultType::Normal);
    }

    #[test]
    fn test_analyzer_empty_signal() {
        let cfg = test_config();
        let analyzer = EngineAnalyzer::new(cfg);
        let health = analyzer.analyze(&[]);
        assert_eq!(health.overall_level_g, 0.0);
        assert_eq!(health.health_score, 100.0);
    }

    #[test]
    fn test_analyzer_config_accessor() {
        let cfg = test_config();
        let analyzer = EngineAnalyzer::new(cfg.clone());
        assert!((analyzer.config().shaft_rpm - 6000.0).abs() < 1e-9);
        assert_eq!(analyzer.config().num_blades, 20);
        assert_eq!(analyzer.config().num_stages, 2);
    }

    #[test]
    fn test_analyzer_imbalanced_signal() {
        let cfg = test_config();
        let analyzer = EngineAnalyzer::new(cfg.clone());
        // Large 1X component
        let signal = sine_signal(cfg.shaft_rpm / 60.0, 2.0, cfg.sample_rate_hz, 1.0);
        let health = analyzer.analyze(&signal);
        assert!(health.imbalance_1x_g > 0.1, "Should detect high 1X");
        assert!(health.health_score < 80.0, "Imbalanced signal should score < 80");
    }
}

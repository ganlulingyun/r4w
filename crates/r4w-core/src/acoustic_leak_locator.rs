//! Acoustic leak detection and localization for water/gas pipeline monitoring.
//!
//! This module provides cross-correlation-based leak localization between two
//! acoustic sensors mounted on a pipeline, along with noise coherence analysis,
//! spectral characterization, propagation velocity estimation, severity
//! classification, background noise removal, and multi-sensor network fusion.
//!
//! # Overview
//!
//! When a leak occurs in a pressurized pipeline, the escaping fluid generates
//! broadband acoustic noise that propagates along the pipe wall. By placing two
//! sensors on either side of the suspected leak location, the time-of-arrival
//! difference reveals the leak position.
//!
//! # Example
//!
//! ```
//! use r4w_core::acoustic_leak_locator::{
//!     LeakConfig, LeakLocator, PipeMaterial, LeakSeverity,
//! };
//!
//! let config = LeakConfig {
//!     sample_rate_hz: 8000.0,
//!     pipe_material: PipeMaterial::Steel,
//!     pipe_diameter_m: 0.3,
//!     sensor_spacing_m: 100.0,
//! };
//!
//! let locator = LeakLocator::new(config);
//! let velocity = locator.propagation_velocity();
//! assert!(velocity > 900.0 && velocity < 1500.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public enums
// ---------------------------------------------------------------------------

/// Pipe material, which strongly influences acoustic propagation velocity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipeMaterial {
    /// Welded or seamless steel pipe (~1200 m/s).
    Steel,
    /// Cast iron pipe (~1100 m/s).
    CastIron,
    /// Ductile iron pipe (~1050 m/s).
    Ductile,
    /// Polyvinyl chloride pipe (~480 m/s).
    PVC,
    /// High-density polyethylene pipe (~400 m/s).
    HDPE,
    /// Copper pipe (~1300 m/s).
    Copper,
}

/// Severity classification of a detected leak.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LeakSeverity {
    /// No leak detected.
    None,
    /// Small seepage - schedule inspection.
    Minor,
    /// Moderate leak - plan repair within days.
    Moderate,
    /// Significant leak - repair within hours.
    Major,
    /// Catastrophic leak - immediate shutdown recommended.
    Critical,
}

// ---------------------------------------------------------------------------
// Configuration and result structs
// ---------------------------------------------------------------------------

/// Configuration for a two-sensor leak localization run.
#[derive(Debug, Clone)]
pub struct LeakConfig {
    /// Sampling rate of both sensors in Hz.
    pub sample_rate_hz: f64,
    /// Material of the pipe under test.
    pub pipe_material: PipeMaterial,
    /// Pipe outer diameter in metres.
    pub pipe_diameter_m: f64,
    /// Distance between sensor A and sensor B in metres.
    pub sensor_spacing_m: f64,
}

/// Result of a leak localization analysis.
#[derive(Debug, Clone)]
pub struct LeakResult {
    /// Estimated distance from sensor A to the leak in metres.
    pub distance_from_sensor_a_m: f64,
    /// Normalized cross-correlation peak (0.0 - 1.0).
    pub correlation_peak: f64,
    /// Confidence metric (0.0 - 1.0) combining correlation and coherence.
    pub confidence: f64,
    /// Classified severity of the leak.
    pub severity: LeakSeverity,
}

// ---------------------------------------------------------------------------
// LeakLocator
// ---------------------------------------------------------------------------

/// Main leak detection and localization engine.
///
/// Wraps a [`LeakConfig`] and exposes the full analysis pipeline.
#[derive(Debug, Clone)]
pub struct LeakLocator {
    config: LeakConfig,
    velocity: f64,
}

impl LeakLocator {
    /// Create a new locator from the given configuration.
    pub fn new(config: LeakConfig) -> Self {
        let velocity =
            estimate_propagation_velocity(&config.pipe_material, config.pipe_diameter_m);
        Self { config, velocity }
    }

    /// Return the estimated propagation velocity in m/s.
    pub fn propagation_velocity(&self) -> f64 {
        self.velocity
    }

    /// Run the full localization pipeline on a pair of sensor recordings.
    ///
    /// Returns a [`LeakResult`] with position, correlation, confidence, and
    /// severity.
    pub fn analyze(&self, sensor_a: &[f64], sensor_b: &[f64]) -> LeakResult {
        let (delay_samples, peak) = cross_correlate_leak(sensor_a, sensor_b);
        let distance = localize_leak(
            delay_samples,
            self.config.sample_rate_hz,
            self.config.sensor_spacing_m,
            self.velocity,
        );

        // Coherence peak in leak band (200-2000 Hz typical).
        let coh = coherence(sensor_a, sensor_b, 256);
        let coherence_peak = coh.iter().cloned().fold(0.0_f64, f64::max);

        let energy = leak_spectral_energy(sensor_a, self.config.sample_rate_hz, 200.0, 2000.0);
        let severity = classify_severity(energy, coherence_peak);

        let confidence = (peak * 0.6 + coherence_peak * 0.4).clamp(0.0, 1.0);

        LeakResult {
            distance_from_sensor_a_m: distance,
            correlation_peak: peak,
            confidence,
            severity,
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Compute the normalized cross-correlation between two sensor signals.
///
/// Returns `(delay_samples, peak_correlation)` where `delay_samples` is the
/// lag of sensor B relative to sensor A (positive means B leads) and
/// `peak_correlation` is the normalized peak value in `[0, 1]`.
pub fn cross_correlate_leak(sensor_a: &[f64], sensor_b: &[f64]) -> (i64, f64) {
    let n = sensor_a.len().min(sensor_b.len());
    if n == 0 {
        return (0, 0.0);
    }

    let energy_a: f64 = sensor_a[..n].iter().map(|x| x * x).sum();
    let energy_b: f64 = sensor_b[..n].iter().map(|x| x * x).sum();
    let norm = (energy_a * energy_b).sqrt();
    if norm < 1e-30 {
        return (0, 0.0);
    }

    let max_lag = n as i64;
    let mut best_lag: i64 = 0;
    let mut best_val: f64 = f64::NEG_INFINITY;

    for lag in -max_lag + 1..max_lag {
        let mut sum = 0.0;
        let start_a = if lag >= 0 { lag as usize } else { 0 };
        let start_b = if lag >= 0 { 0 } else { (-lag) as usize };
        let count = n.saturating_sub(lag.unsigned_abs() as usize);
        for i in 0..count {
            sum += sensor_a[start_a + i] * sensor_b[start_b + i];
        }
        if sum > best_val {
            best_val = sum;
            best_lag = lag;
        }
    }

    let peak = (best_val / norm).clamp(0.0, 1.0);
    (best_lag, peak)
}

/// Compute the distance from sensor A to the leak.
///
/// Given the delay in samples, the sample rate, sensor spacing, and
/// propagation velocity, the leak position is:
///
/// ```text
/// d_A = (L - v * dt) / 2
/// ```
///
/// where `dt = delay_samples / sample_rate` and `L` is the sensor spacing.
pub fn localize_leak(
    delay_samples: i64,
    sample_rate: f64,
    sensor_spacing_m: f64,
    propagation_velocity_mps: f64,
) -> f64 {
    let dt = delay_samples as f64 / sample_rate;
    let d = (sensor_spacing_m - propagation_velocity_mps * dt) / 2.0;
    d.clamp(0.0, sensor_spacing_m)
}

/// Estimate the acoustic propagation velocity in the pipe wall (m/s).
///
/// Velocity depends primarily on material and, to a lesser extent, on pipe
/// diameter (larger pipes generally carry sound slightly faster).
pub fn estimate_propagation_velocity(material: &PipeMaterial, diameter_m: f64) -> f64 {
    // Base velocity per material (empirical mid-range values).
    let base = match material {
        PipeMaterial::Steel => 1200.0,
        PipeMaterial::CastIron => 1100.0,
        PipeMaterial::Ductile => 1050.0,
        PipeMaterial::PVC => 480.0,
        PipeMaterial::HDPE => 400.0,
        PipeMaterial::Copper => 1300.0,
    };
    // Diameter correction: +2% per 0.1 m above 0.1 m reference.
    let correction = 1.0 + 0.02 * ((diameter_m - 0.1) / 0.1).max(0.0);
    base * correction
}

/// Compute the magnitude-squared coherence between two signals.
///
/// The result is a vector of length `fft_size / 2 + 1` with values in `[0, 1]`.
/// Coherence near 1 indicates a strong common signal (likely a leak);
/// coherence near 0 indicates uncorrelated noise.
pub fn coherence(sensor_a: &[f64], sensor_b: &[f64], fft_size: usize) -> Vec<f64> {
    let n = sensor_a.len().min(sensor_b.len());
    let fft_size = fft_size.max(4);
    let half = fft_size / 2 + 1;

    if n < fft_size {
        return vec![0.0; half];
    }

    let num_segments = n / fft_size;
    if num_segments == 0 {
        return vec![0.0; half];
    }

    let mut paa = vec![0.0; half];
    let mut pbb = vec![0.0; half];
    let mut pab_re = vec![0.0; half];
    let mut pab_im = vec![0.0; half];

    for seg in 0..num_segments {
        let offset = seg * fft_size;
        let spec_a = simple_dft(&sensor_a[offset..offset + fft_size]);
        let spec_b = simple_dft(&sensor_b[offset..offset + fft_size]);

        for k in 0..half {
            let (ar, ai) = spec_a[k];
            let (br, bi) = spec_b[k];
            paa[k] += ar * ar + ai * ai;
            pbb[k] += br * br + bi * bi;
            // Cross-spectrum: A* x B
            pab_re[k] += ar * br + ai * bi;
            pab_im[k] += -ai * br + ar * bi;
        }
    }

    let mut coh = vec![0.0; half];
    for k in 0..half {
        let cross_mag2 = pab_re[k] * pab_re[k] + pab_im[k] * pab_im[k];
        let denom = paa[k] * pbb[k];
        if denom > 1e-30 {
            coh[k] = (cross_mag2 / denom).clamp(0.0, 1.0);
        }
    }
    coh
}

/// Compute the spectral energy of `signal` in the frequency band
/// `[low_hz, high_hz]`.
///
/// Uses a simple DFT to obtain the power spectrum and sums the bins falling
/// within the requested band.
pub fn leak_spectral_energy(signal: &[f64], sample_rate: f64, low_hz: f64, high_hz: f64) -> f64 {
    if signal.is_empty() || sample_rate <= 0.0 {
        return 0.0;
    }
    let n = signal.len();
    let spec = simple_dft(signal);
    let half = n / 2 + 1;
    let bin_width = sample_rate / n as f64;

    let mut energy = 0.0;
    for k in 0..half {
        let freq = k as f64 * bin_width;
        if freq >= low_hz && freq <= high_hz {
            let (re, im) = spec[k];
            energy += re * re + im * im;
        }
    }
    // Normalize by number of samples squared to get average power-like quantity.
    energy / (n as f64 * n as f64)
}

/// Classify the leak severity from spectral energy and coherence peak.
///
/// Higher energy and coherence indicate a more severe leak.
pub fn classify_severity(spectral_energy: f64, coherence_peak: f64) -> LeakSeverity {
    let score = spectral_energy.sqrt() * 1000.0 + coherence_peak * 100.0;
    match score {
        s if s < 5.0 => LeakSeverity::None,
        s if s < 20.0 => LeakSeverity::Minor,
        s if s < 50.0 => LeakSeverity::Moderate,
        s if s < 90.0 => LeakSeverity::Major,
        _ => LeakSeverity::Critical,
    }
}

/// Remove background noise from `signal` via spectral subtraction.
///
/// `background` should be a noise-only reference recording. Spectral
/// magnitudes of `background` are subtracted from `signal` (floored at zero)
/// and the signal is reconstructed via inverse DFT.
pub fn remove_background(signal: &mut Vec<f64>, background: &[f64]) {
    let n = signal.len();
    if n == 0 || background.is_empty() {
        return;
    }
    let bg_n = background.len().min(n);

    // Zero-pad background to match signal length.
    let mut bg_padded = vec![0.0; n];
    bg_padded[..bg_n].copy_from_slice(&background[..bg_n]);

    let spec_sig = simple_dft(signal);
    let spec_bg = simple_dft(&bg_padded);

    let mut cleaned_spec: Vec<(f64, f64)> = Vec::with_capacity(n);
    for k in 0..n {
        let (sr, si) = spec_sig[k];
        let (br, bi) = spec_bg[k];
        let mag_sig = (sr * sr + si * si).sqrt();
        let mag_bg = (br * br + bi * bi).sqrt();
        let mag_clean = (mag_sig - mag_bg).max(0.0);
        if mag_sig > 1e-30 {
            let scale = mag_clean / mag_sig;
            cleaned_spec.push((sr * scale, si * scale));
        } else {
            cleaned_spec.push((0.0, 0.0));
        }
    }

    let reconstructed = simple_idft(&cleaned_spec);
    signal.clear();
    signal.extend_from_slice(&reconstructed[..n]);
}

/// Fuse multiple independent leak position estimates via weighted averaging.
///
/// Each element of `leak_candidates` is `(position_m, weight)`.
/// Returns `(fused_position_m, total_weight)`.
pub fn multi_sensor_fuse(leak_candidates: &[(f64, f64)]) -> (f64, f64) {
    if leak_candidates.is_empty() {
        return (0.0, 0.0);
    }
    let mut sum_pos = 0.0;
    let mut sum_w = 0.0;
    for &(pos, w) in leak_candidates {
        sum_pos += pos * w;
        sum_w += w;
    }
    if sum_w < 1e-30 {
        return (0.0, 0.0);
    }
    (sum_pos / sum_w, sum_w)
}

/// Generate synthetic leak noise centred at `center_freq_hz` with the given
/// `bandwidth_hz`.
///
/// Produces band-limited pseudo-random noise that approximates the spectral
/// shape of a pressurized leak.
pub fn generate_leak_noise(
    sample_rate: f64,
    duration_s: f64,
    center_freq_hz: f64,
    bandwidth_hz: f64,
) -> Vec<f64> {
    let n = (sample_rate * duration_s).ceil() as usize;
    if n == 0 {
        return Vec::new();
    }

    // Generate white noise via a simple LCG PRNG (deterministic, no external crate).
    let mut rng_state: u64 = 0xDEAD_BEEF_CAFE_1234;
    let mut white: Vec<f64> = Vec::with_capacity(n);
    for _ in 0..n {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
        // Map to roughly [-1, 1].
        let val = (rng_state >> 33) as f64 / (1u64 << 31) as f64 - 1.0;
        white.push(val);
    }

    // Band-pass filter in frequency domain.
    let spec = simple_dft(&white);
    let bin_width = sample_rate / n as f64;
    let low = center_freq_hz - bandwidth_hz / 2.0;
    let high = center_freq_hz + bandwidth_hz / 2.0;

    let mut filtered_spec: Vec<(f64, f64)> = Vec::with_capacity(n);
    for k in 0..n {
        let freq = if k <= n / 2 {
            k as f64 * bin_width
        } else {
            (k as f64 - n as f64) * bin_width
        };
        let abs_freq = freq.abs();
        if abs_freq >= low && abs_freq <= high {
            filtered_spec.push(spec[k]);
        } else {
            filtered_spec.push((0.0, 0.0));
        }
    }

    simple_idft(&filtered_spec)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Naive DFT (O(N^2)) - sufficient for the moderate sizes used in leak analysis.
fn simple_dft(x: &[f64]) -> Vec<(f64, f64)> {
    let n = x.len();
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &xi) in x.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += xi * angle.cos();
            im += xi * angle.sin();
        }
        out.push((re, im));
    }
    out
}

/// Inverse DFT.
fn simple_idft(spec: &[(f64, f64)]) -> Vec<f64> {
    let n = spec.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut val = 0.0;
        for (k, &(re, im)) in spec.iter().enumerate() {
            let angle = 2.0 * PI * k as f64 * i as f64 / n as f64;
            val += re * angle.cos() - im * angle.sin();
        }
        out.push(val / n as f64);
    }
    out
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- PipeMaterial & velocity ----

    #[test]
    fn test_propagation_velocity_steel() {
        let v = estimate_propagation_velocity(&PipeMaterial::Steel, 0.1);
        // At reference diameter the correction factor is 1.0.
        assert!((v - 1200.0).abs() < 1.0);
    }

    #[test]
    fn test_propagation_velocity_pvc() {
        let v = estimate_propagation_velocity(&PipeMaterial::PVC, 0.1);
        assert!((v - 480.0).abs() < 1.0);
    }

    #[test]
    fn test_propagation_velocity_hdpe() {
        let v = estimate_propagation_velocity(&PipeMaterial::HDPE, 0.2);
        // Diameter correction: +2% for extra 0.1 m.
        let expected = 400.0 * 1.02;
        assert!((v - expected).abs() < 1.0);
    }

    #[test]
    fn test_propagation_velocity_copper_large_diameter() {
        let v = estimate_propagation_velocity(&PipeMaterial::Copper, 0.5);
        // 4 increments of 0.1 m above reference -> 1.08 factor.
        let expected = 1300.0 * 1.08;
        assert!((v - expected).abs() < 1.0);
    }

    #[test]
    fn test_propagation_velocity_cast_iron() {
        let v = estimate_propagation_velocity(&PipeMaterial::CastIron, 0.1);
        assert!((v - 1100.0).abs() < 1.0);
    }

    #[test]
    fn test_propagation_velocity_ductile() {
        let v = estimate_propagation_velocity(&PipeMaterial::Ductile, 0.1);
        assert!((v - 1050.0).abs() < 1.0);
    }

    // ---- Cross-correlation ----

    #[test]
    fn test_cross_correlate_identical_signals() {
        let sig: Vec<f64> = (0..64).map(|i| (i as f64 * 0.1).sin()).collect();
        let (lag, peak) = cross_correlate_leak(&sig, &sig);
        assert_eq!(lag, 0);
        assert!((peak - 1.0).abs() < 1e-6, "peak={peak}");
    }

    #[test]
    fn test_cross_correlate_shifted_signal() {
        let n = 128;
        let shift = 5;
        let mut a = vec![0.0; n];
        let mut b = vec![0.0; n];
        // Place an impulse at position 40 in A and 40+shift in B.
        a[40] = 1.0;
        b[40 + shift] = 1.0;
        let (lag, _peak) = cross_correlate_leak(&a, &b);
        // Negative lag means B is delayed relative to A.
        assert_eq!(lag, -(shift as i64));
    }

    #[test]
    fn test_cross_correlate_empty() {
        let (lag, peak) = cross_correlate_leak(&[], &[]);
        assert_eq!(lag, 0);
        assert!((peak - 0.0).abs() < 1e-12);
    }

    // ---- Localization ----

    #[test]
    fn test_localize_leak_midpoint() {
        // Zero delay -> leak at midpoint.
        let d = localize_leak(0, 8000.0, 100.0, 1200.0);
        assert!((d - 50.0).abs() < 1e-6);
    }

    #[test]
    fn test_localize_leak_towards_a() {
        // Positive delay (B arrives first) -> leak closer to A.
        let d = localize_leak(10, 8000.0, 100.0, 1200.0);
        assert!(d < 50.0, "d={d}");
    }

    #[test]
    fn test_localize_leak_towards_b() {
        // Negative delay -> leak closer to B.
        let d = localize_leak(-10, 8000.0, 100.0, 1200.0);
        assert!(d > 50.0, "d={d}");
    }

    #[test]
    fn test_localize_leak_clamp() {
        // Extreme delay that would place leak outside the pipe.
        let d = localize_leak(100_000, 8000.0, 100.0, 1200.0);
        assert!((d - 0.0).abs() < 1e-6);
    }

    // ---- Coherence ----

    #[test]
    fn test_coherence_identical_signals() {
        let sig: Vec<f64> = (0..256).map(|i| (2.0 * PI * i as f64 / 32.0).sin()).collect();
        let coh = coherence(&sig, &sig, 64);
        // For identical signals coherence should be 1 everywhere.
        let max_coh = coh.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_coh > 0.99, "max_coh={max_coh}");
    }

    #[test]
    fn test_coherence_uncorrelated() {
        // Two very different deterministic signals.
        let a: Vec<f64> = (0..256).map(|i| (2.0 * PI * i as f64 / 8.0).sin()).collect();
        let b: Vec<f64> = (0..256).map(|i| (2.0 * PI * i as f64 / 37.0).cos()).collect();
        let coh = coherence(&a, &b, 64);
        let mean: f64 = coh.iter().sum::<f64>() / coh.len() as f64;
        // Should be low on average (not perfectly zero due to finite segments).
        assert!(mean < 0.7, "mean coherence={mean}");
    }

    #[test]
    fn test_coherence_short_signal() {
        let sig = vec![1.0, 2.0];
        let coh = coherence(&sig, &sig, 64);
        // Signal shorter than FFT size -> all zeros.
        assert!(coh.iter().all(|&c| c == 0.0));
    }

    // ---- Spectral energy ----

    #[test]
    fn test_leak_spectral_energy_tone_in_band() {
        let n = 256;
        let fs = 8000.0;
        let freq = 500.0;
        let sig: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / fs).sin())
            .collect();
        let energy = leak_spectral_energy(&sig, fs, 400.0, 600.0);
        assert!(energy > 0.1, "energy={energy}");
    }

    #[test]
    fn test_leak_spectral_energy_tone_out_of_band() {
        let n = 256;
        let fs = 8000.0;
        let freq = 3000.0;
        let sig: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / fs).sin())
            .collect();
        let energy = leak_spectral_energy(&sig, fs, 400.0, 600.0);
        assert!(energy < 0.01, "energy={energy}");
    }

    #[test]
    fn test_leak_spectral_energy_empty() {
        let energy = leak_spectral_energy(&[], 8000.0, 200.0, 2000.0);
        assert!((energy - 0.0).abs() < 1e-12);
    }

    // ---- Severity classification ----

    #[test]
    fn test_classify_severity_none() {
        let sev = classify_severity(0.0, 0.0);
        assert_eq!(sev, LeakSeverity::None);
    }

    #[test]
    fn test_classify_severity_critical() {
        let sev = classify_severity(100.0, 1.0);
        assert_eq!(sev, LeakSeverity::Critical);
    }

    // ---- Background removal ----

    #[test]
    fn test_remove_background_reduces_noise() {
        let n = 128;
        let fs = 8000.0;
        // Signal = tone + noise-like pattern.
        let tone: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 500.0 * i as f64 / fs).sin())
            .collect();
        let noise: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * PI * 100.0 * i as f64 / fs).sin())
            .collect();
        let mut signal: Vec<f64> = tone.iter().zip(&noise).map(|(t, n)| t + n).collect();

        let energy_before: f64 = signal.iter().map(|x| x * x).sum();
        remove_background(&mut signal, &noise);
        let energy_after: f64 = signal.iter().map(|x| x * x).sum();

        assert!(
            energy_after < energy_before,
            "energy_after={energy_after} >= energy_before={energy_before}"
        );
    }

    // ---- Multi-sensor fusion ----

    #[test]
    fn test_multi_sensor_fuse_single() {
        let (pos, w) = multi_sensor_fuse(&[(42.0, 1.0)]);
        assert!((pos - 42.0).abs() < 1e-6);
        assert!((w - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_multi_sensor_fuse_weighted() {
        let (pos, _w) = multi_sensor_fuse(&[(10.0, 1.0), (30.0, 3.0)]);
        // Weighted average: (10*1 + 30*3) / 4 = 100/4 = 25.
        assert!((pos - 25.0).abs() < 1e-6, "pos={pos}");
    }

    #[test]
    fn test_multi_sensor_fuse_empty() {
        let (pos, w) = multi_sensor_fuse(&[]);
        assert!((pos - 0.0).abs() < 1e-12);
        assert!((w - 0.0).abs() < 1e-12);
    }

    // ---- Generate leak noise ----

    #[test]
    fn test_generate_leak_noise_length() {
        let noise = generate_leak_noise(8000.0, 0.01, 1000.0, 500.0);
        assert_eq!(noise.len(), 80);
    }

    #[test]
    fn test_generate_leak_noise_zero_duration() {
        let noise = generate_leak_noise(8000.0, 0.0, 1000.0, 500.0);
        assert!(noise.is_empty());
    }

    // ---- LeakLocator integration ----

    #[test]
    fn test_leak_locator_analyze() {
        let config = LeakConfig {
            sample_rate_hz: 8000.0,
            pipe_material: PipeMaterial::Steel,
            pipe_diameter_m: 0.3,
            sensor_spacing_m: 100.0,
        };
        let locator = LeakLocator::new(config);
        // Identical signals -> leak at midpoint.
        let sig: Vec<f64> = (0..256)
            .map(|i| (2.0 * PI * 500.0 * i as f64 / 8000.0).sin())
            .collect();
        let result = locator.analyze(&sig, &sig);
        assert!((result.distance_from_sensor_a_m - 50.0).abs() < 1.0);
        assert!(result.correlation_peak > 0.9);
    }
}

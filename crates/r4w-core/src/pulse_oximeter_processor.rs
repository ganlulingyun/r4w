//! Pulse oximetry signal processing for SpO2 and heart rate measurement.
//!
//! This module implements photoplethysmography (PPG) signal processing for
//! measuring blood oxygen saturation (SpO2) and heart rate from red and
//! infrared wavelength PPG signals. All math is implemented from scratch
//! with no external dependencies beyond `std`.
//!
//! # Background
//!
//! Pulse oximetry exploits the differential absorption of oxyhemoglobin (HbO2)
//! and deoxyhemoglobin (Hb) at red (~660 nm) and infrared (~940 nm) wavelengths.
//! The ratio of ratios (R) of pulsatile (AC) to baseline (DC) components at
//! both wavelengths is used to estimate arterial oxygen saturation via an
//! empirical calibration curve.
//!
//! # Key Algorithm
//!
//! ```text
//! R = (AC_red / DC_red) / (AC_ir / DC_ir)
//! SpO2 = 110 - 25 * R   (simplified empirical calibration)
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::pulse_oximeter_processor::*;
//!
//! let config = PulseOxConfig {
//!     sample_rate_hz: 100.0,
//!     ..Default::default()
//! };
//! let mut processor = PulseOxProcessor::new(config);
//!
//! // Generate synthetic PPG signals (72 BPM, 97% SpO2)
//! let (red, ir) = generate_ppg_signal(72.0, 97.0, 100.0, 5.0);
//! let result = processor.process(&red, &ir);
//!
//! assert!(result.spo2_percent > 90.0 && result.spo2_percent < 100.0);
//! assert!(result.heart_rate_bpm > 60.0 && result.heart_rate_bpm < 90.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for pulse oximetry processing.
#[derive(Debug, Clone)]
pub struct PulseOxConfig {
    /// Sampling rate of the PPG signal in Hz.
    pub sample_rate_hz: f64,
    /// Red LED wavelength in nanometers (typical: 660 nm).
    pub red_wavelength_nm: f64,
    /// Infrared LED wavelength in nanometers (typical: 940 nm).
    pub ir_wavelength_nm: f64,
    /// Analysis window size in seconds.
    pub window_size_s: f64,
}

impl Default for PulseOxConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 100.0,
            red_wavelength_nm: 660.0,
            ir_wavelength_nm: 940.0,
            window_size_s: 4.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Result of pulse oximetry processing.
#[derive(Debug, Clone)]
pub struct PulseOxResult {
    /// Estimated blood oxygen saturation as a percentage (0-100).
    pub spo2_percent: f64,
    /// Estimated heart rate in beats per minute.
    pub heart_rate_bpm: f64,
    /// Perfusion index as a percentage (AC/DC * 100).
    pub perfusion_index: f64,
    /// Signal quality index (0.0 = poor, 1.0 = excellent).
    pub signal_quality: f64,
    /// Raw R ratio: (AC_red/DC_red) / (AC_ir/DC_ir).
    pub r_ratio: f64,
}

// ---------------------------------------------------------------------------
// Processor
// ---------------------------------------------------------------------------

/// Stateful pulse oximetry signal processor.
///
/// Accepts paired red and IR PPG sample buffers and produces SpO2, heart rate,
/// perfusion index, signal quality, and the raw R ratio.
pub struct PulseOxProcessor {
    config: PulseOxConfig,
}

impl PulseOxProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: PulseOxConfig) -> Self {
        Self { config }
    }

    /// Process a buffer of red and IR PPG samples.
    ///
    /// Both slices must have the same length and contain at least one full
    /// cardiac cycle for meaningful results.
    pub fn process(&mut self, red: &[f64], ir: &[f64]) -> PulseOxResult {
        assert_eq!(red.len(), ir.len(), "red and ir buffers must have equal length");
        let fs = self.config.sample_rate_hz;

        // Lowpass-filter both channels to remove high-frequency noise
        let cutoff = 8.0; // Hz – well above heart rate harmonics
        let red_filt = lowpass_filter(red, fs, cutoff);
        let ir_filt = lowpass_filter(ir, fs, cutoff);

        // Extract AC (pulsatile) and DC (baseline) components
        let (red_ac, red_dc) = extract_ac_dc(&red_filt, fs);
        let (ir_ac, ir_dc) = extract_ac_dc(&ir_filt, fs);

        // Compute R ratio and SpO2
        let r = compute_r_ratio(red_ac, red_dc, ir_ac, ir_dc);
        let spo2 = r_to_spo2(r);

        // Detect peaks and compute heart rate
        let peaks = detect_ppg_peaks(&ir_filt, fs);
        let hr = compute_heart_rate(&peaks, fs);

        // Perfusion index from the IR channel
        let pi = perfusion_index(ir_ac, ir_dc);

        // Signal quality
        let sqi = signal_quality_index(&ir_filt, fs);

        PulseOxResult {
            spo2_percent: spo2,
            heart_rate_bpm: hr,
            perfusion_index: pi,
            signal_quality: sqi,
            r_ratio: r,
        }
    }
}

// ---------------------------------------------------------------------------
// Core algorithms (all pub)
// ---------------------------------------------------------------------------

/// Compute the ratio of ratios R = (AC_red / DC_red) / (AC_ir / DC_ir).
///
/// This is the fundamental measurement in pulse oximetry. R ~ 1.0 corresponds
/// to roughly 85% SpO2; lower R means higher SpO2.
pub fn compute_r_ratio(red_ac: f64, red_dc: f64, ir_ac: f64, ir_dc: f64) -> f64 {
    if red_dc.abs() < 1e-12 || ir_dc.abs() < 1e-12 || ir_ac.abs() < 1e-12 {
        return 0.0;
    }
    (red_ac / red_dc) / (ir_ac / ir_dc)
}

/// Convert R ratio to SpO2 percentage using a simplified empirical calibration.
///
/// The standard empirical approximation is:
///     SpO2 = 110 - 25 * R
///
/// This is a linearisation of the Beer-Lambert law calibration. Commercial
/// devices use lookup tables derived from volunteer desaturation studies.
/// The result is clamped to [0, 100].
pub fn r_to_spo2(r: f64) -> f64 {
    let spo2 = 110.0 - 25.0 * r;
    spo2.clamp(0.0, 100.0)
}

/// Separate the AC (pulsatile) and DC (baseline) components of a PPG signal.
///
/// DC is estimated as the mean of the signal. AC is the RMS of the mean-
/// removed signal, bandpass-filtered in the physiological heart-rate band
/// (0.5 – 5 Hz).
pub fn extract_ac_dc(signal: &[f64], fs: f64) -> (f64, f64) {
    if signal.is_empty() {
        return (0.0, 0.0);
    }

    // DC component: mean
    let dc: f64 = signal.iter().sum::<f64>() / signal.len() as f64;

    // Remove DC before bandpass to avoid filter transients on large offsets
    let zero_mean: Vec<f64> = signal.iter().map(|&x| x - dc).collect();

    // Bandpass filter to isolate pulsatile component (0.5 – 5 Hz)
    let lp5 = lowpass_filter(&zero_mean, fs, 5.0);
    let hp05 = highpass_filter(&lp5, fs, 0.5);

    // Skip initial transient (2 seconds or 10% of signal, whichever is smaller)
    let skip = ((2.0 * fs) as usize).min(hp05.len() / 10);
    let steady = &hp05[skip..];

    // AC component: RMS of bandpassed signal (steady-state portion)
    let ac_rms = if steady.is_empty() {
        0.0
    } else {
        let sum_sq: f64 = steady.iter().map(|&x| x * x).sum();
        (sum_sq / steady.len() as f64).sqrt()
    };

    (ac_rms, dc)
}

/// Detect systolic peaks in a PPG waveform.
///
/// Uses a derivative-based approach with adaptive thresholding:
/// 1. Bandpass filter (0.5 – 5 Hz) to isolate cardiac component
/// 2. Find local maxima with a minimum inter-peak distance based on
///    a plausible heart rate range (30 – 240 BPM)
pub fn detect_ppg_peaks(signal: &[f64], fs: f64) -> Vec<usize> {
    if signal.len() < 3 {
        return Vec::new();
    }

    // Remove DC before bandpass to avoid transient issues
    let mean_val: f64 = signal.iter().sum::<f64>() / signal.len() as f64;
    let zero_mean: Vec<f64> = signal.iter().map(|&x| x - mean_val).collect();

    // Bandpass to cardiac band
    let lp = lowpass_filter(&zero_mean, fs, 5.0);
    let bp = highpass_filter(&lp, fs, 0.5);

    if bp.len() < 3 {
        return Vec::new();
    }

    // Skip initial transient (1 second)
    let skip = ((1.0 * fs) as usize).min(bp.len() / 4);

    // Minimum distance between peaks: 60 / 240 BPM * fs
    let min_dist = (fs * 60.0 / 240.0).max(1.0) as usize;

    // Compute adaptive threshold on steady-state portion
    let steady = &bp[skip..];
    if steady.len() < 3 {
        return Vec::new();
    }
    let bp_mean: f64 = steady.iter().sum::<f64>() / steady.len() as f64;
    let max_val = steady.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let threshold = bp_mean + 0.3 * (max_val - bp_mean);

    // Find local maxima above threshold (in the full signal, but using
    // the steady-state threshold)
    let mut peaks = Vec::new();
    for i in (skip + 1)..bp.len() - 1 {
        if bp[i] > bp[i - 1] && bp[i] > bp[i + 1] && bp[i] > threshold {
            // Enforce minimum inter-peak distance
            if peaks.is_empty() || (i - *peaks.last().unwrap()) >= min_dist {
                peaks.push(i);
            } else if bp[i] > bp[*peaks.last().unwrap()] {
                // Replace previous peak if this one is larger (within min_dist window)
                *peaks.last_mut().unwrap() = i;
            }
        }
    }

    peaks
}

/// Compute heart rate in BPM from detected peak indices.
///
/// Returns the median-based heart rate from inter-peak intervals. If fewer
/// than 2 peaks are provided, returns 0.
pub fn compute_heart_rate(peak_indices: &[usize], fs: f64) -> f64 {
    if peak_indices.len() < 2 || fs <= 0.0 {
        return 0.0;
    }

    let mut intervals: Vec<f64> = Vec::with_capacity(peak_indices.len() - 1);
    for i in 1..peak_indices.len() {
        let diff = peak_indices[i] as f64 - peak_indices[i - 1] as f64;
        intervals.push(diff / fs);
    }

    // Use median interval for robustness against outliers
    intervals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median_interval = if intervals.len() % 2 == 0 {
        (intervals[intervals.len() / 2 - 1] + intervals[intervals.len() / 2]) / 2.0
    } else {
        intervals[intervals.len() / 2]
    };

    if median_interval > 0.0 {
        60.0 / median_interval
    } else {
        0.0
    }
}

/// Compute perfusion index: PI = (AC / DC) * 100%.
///
/// Perfusion index indicates the strength of the pulsatile signal relative
/// to the non-pulsatile baseline. Typical values range from 0.02% (very weak)
/// to 20% (very strong perfusion).
pub fn perfusion_index(ac: f64, dc: f64) -> f64 {
    if dc.abs() < 1e-12 {
        return 0.0;
    }
    (ac / dc).abs() * 100.0
}

/// Compute signal quality index (SQI) from PPG waveform regularity.
///
/// Evaluates quality based on:
/// 1. Peak regularity — coefficient of variation of inter-peak intervals
/// 2. Amplitude consistency — coefficient of variation of peak amplitudes
/// 3. Morphology — correlation with a template beat
///
/// Returns a value in [0, 1] where 1 is excellent quality.
pub fn signal_quality_index(signal: &[f64], fs: f64) -> f64 {
    if signal.len() < 3 {
        return 0.0;
    }

    let peaks = detect_ppg_peaks(signal, fs);
    if peaks.len() < 3 {
        return 0.0;
    }

    // 1. Peak interval regularity (lower CV = better)
    let mut intervals: Vec<f64> = Vec::new();
    for i in 1..peaks.len() {
        intervals.push((peaks[i] - peaks[i - 1]) as f64);
    }
    let interval_cv = coefficient_of_variation(&intervals);
    let regularity_score = (1.0 - interval_cv.min(1.0)).max(0.0);

    // 2. Peak amplitude consistency
    let peak_amps: Vec<f64> = peaks.iter().map(|&p| signal[p]).collect();
    let amp_cv = coefficient_of_variation(&peak_amps);
    let amplitude_score = (1.0 - amp_cv.min(1.0)).max(0.0);

    // 3. Template matching: build template from median beat and correlate
    let template_score = template_match_score(signal, &peaks);

    // Weighted combination
    let sqi = 0.4 * regularity_score + 0.3 * amplitude_score + 0.3 * template_score;
    sqi.clamp(0.0, 1.0)
}

/// LMS adaptive filter for motion artifact removal.
///
/// Uses an accelerometer reference signal to cancel motion-correlated
/// components from the PPG signal. The LMS (Least Mean Squares) algorithm
/// adapts filter weights to minimize the output power.
///
/// # Arguments
/// * `ppg` - Primary PPG signal (contaminated with motion artifacts)
/// * `accel` - Reference accelerometer signal (correlated with motion)
/// * `mu` - Step size / learning rate (typical: 0.01 – 0.1)
///
/// # Returns
/// Filtered PPG signal with motion artifacts removed.
pub fn adaptive_filter_motion(ppg: &[f64], accel: &[f64], mu: f64) -> Vec<f64> {
    let n = ppg.len().min(accel.len());
    if n == 0 {
        return Vec::new();
    }

    let filter_order = 32;
    let mut weights = vec![0.0_f64; filter_order];
    let mut output = Vec::with_capacity(n);
    let mut buffer = vec![0.0_f64; filter_order];

    for i in 0..n {
        // Shift buffer and insert new accelerometer sample
        for j in (1..filter_order).rev() {
            buffer[j] = buffer[j - 1];
        }
        buffer[0] = accel[i];

        // FIR filter output: estimate of motion artifact
        let estimate: f64 = weights.iter().zip(buffer.iter()).map(|(w, x)| w * x).sum();

        // Error signal: PPG minus estimated artifact
        let error = ppg[i] - estimate;
        output.push(error);

        // LMS weight update: w[k] += mu * error * x[k]
        let norm = buffer.iter().map(|x| x * x).sum::<f64>() + 1e-12;
        let step = mu * error / norm;
        for j in 0..filter_order {
            weights[j] += step * buffer[j];
        }
    }

    output
}

/// Simple first-order IIR lowpass filter.
///
/// Implements y[n] = alpha * x[n] + (1 - alpha) * y[n-1], where
/// alpha is derived from the cutoff frequency via the bilinear transform
/// approximation.
pub fn lowpass_filter(signal: &[f64], fs: f64, cutoff_hz: f64) -> Vec<f64> {
    if signal.is_empty() || fs <= 0.0 || cutoff_hz <= 0.0 {
        return signal.to_vec();
    }

    // RC time constant based lowpass: alpha = dt / (RC + dt)
    let dt = 1.0 / fs;
    let rc = 1.0 / (2.0 * PI * cutoff_hz);
    let alpha = dt / (rc + dt);

    let mut output = Vec::with_capacity(signal.len());
    output.push(signal[0]);
    for i in 1..signal.len() {
        let y = alpha * signal[i] + (1.0 - alpha) * output[i - 1];
        output.push(y);
    }

    output
}

/// Simple first-order IIR highpass filter.
///
/// Implements y[n] = alpha * (y[n-1] + x[n] - x[n-1]) using the RC
/// highpass approximation.
pub fn highpass_filter(signal: &[f64], fs: f64, cutoff_hz: f64) -> Vec<f64> {
    if signal.is_empty() || fs <= 0.0 || cutoff_hz <= 0.0 {
        return signal.to_vec();
    }

    let dt = 1.0 / fs;
    let rc = 1.0 / (2.0 * PI * cutoff_hz);
    let alpha = rc / (rc + dt);

    let mut output = Vec::with_capacity(signal.len());
    output.push(signal[0]);
    for i in 1..signal.len() {
        let y = alpha * (output[i - 1] + signal[i] - signal[i - 1]);
        output.push(y);
    }

    output
}

/// Generate synthetic PPG signals for red and IR channels.
///
/// Produces realistic PPG-like waveforms with:
/// - A DC baseline offset
/// - Pulsatile component with dicrotic notch morphology
/// - Differential red/IR modulation depths based on target SpO2
///
/// # Arguments
/// * `heart_rate_bpm` - Desired heart rate (30 – 240 BPM)
/// * `spo2` - Desired SpO2 percentage (70 – 100)
/// * `fs` - Sample rate in Hz
/// * `duration_s` - Duration in seconds
///
/// # Returns
/// Tuple of (red_signal, ir_signal).
pub fn generate_ppg_signal(
    heart_rate_bpm: f64,
    spo2: f64,
    fs: f64,
    duration_s: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = (fs * duration_s) as usize;
    let period = fs * 60.0 / heart_rate_bpm; // samples per beat

    // Derive target R from SpO2: R = (110 - SpO2) / 25
    let r_target = (110.0 - spo2.clamp(0.0, 100.0)) / 25.0;

    // IR modulation depth (AC/DC ratio)
    let ir_ac_dc = 0.02; // 2% typical IR perfusion
    // Red AC/DC = R * ir_ac_dc
    let red_ac_dc = r_target * ir_ac_dc;

    // DC baselines (arbitrary units, IR typically stronger)
    let red_dc = 1000.0;
    let ir_dc = 2000.0;

    let mut red = Vec::with_capacity(n);
    let mut ir = Vec::with_capacity(n);

    for i in 0..n {
        let t = i as f64 / fs;
        let phase = 2.0 * PI * (heart_rate_bpm / 60.0) * t;

        // PPG pulse shape: combination of harmonics to approximate dicrotic notch
        // Fundamental + 2nd harmonic + 3rd harmonic
        let pulse = -0.6 * phase.sin()
            - 0.3 * (2.0 * phase).sin()
            - 0.1 * (3.0 * phase).sin();

        // Scale by AC/DC ratio and add DC offset
        let red_sample = red_dc * (1.0 + red_ac_dc * pulse);
        let ir_sample = ir_dc * (1.0 + ir_ac_dc * pulse);

        red.push(red_sample);
        ir.push(ir_sample);
    }

    // Ensure we have enough samples for at least a few beats
    let _ = period; // Used implicitly via heart_rate_bpm in the loop

    (red, ir)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Coefficient of variation: std_dev / mean (always non-negative).
fn coefficient_of_variation(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mean = data.iter().sum::<f64>() / data.len() as f64;
    if mean.abs() < 1e-12 {
        return 1.0; // undefined CV treated as high variability
    }
    let variance = data.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / data.len() as f64;
    variance.sqrt() / mean.abs()
}

/// Template matching score for PPG morphology assessment.
///
/// Builds a template from the median-length beat and computes the average
/// normalized cross-correlation across all detected beats.
fn template_match_score(signal: &[f64], peaks: &[usize]) -> f64 {
    if peaks.len() < 3 {
        return 0.0;
    }

    // Determine median beat length
    let mut lengths: Vec<usize> = Vec::new();
    for i in 1..peaks.len() {
        lengths.push(peaks[i] - peaks[i - 1]);
    }
    lengths.sort();
    let median_len = lengths[lengths.len() / 2];

    if median_len == 0 {
        return 0.0;
    }

    // Extract beats and build template (average of first few beats)
    let mut beats: Vec<Vec<f64>> = Vec::new();
    for i in 0..peaks.len() - 1 {
        let start = peaks[i];
        let end = (start + median_len).min(signal.len());
        if end - start >= median_len {
            let beat: Vec<f64> = signal[start..end].to_vec();
            beats.push(beat);
        }
    }

    if beats.len() < 2 {
        return 0.0;
    }

    // Template: average of all beats
    let mut template = vec![0.0_f64; median_len];
    for beat in &beats {
        for (j, &v) in beat.iter().enumerate() {
            template[j] += v;
        }
    }
    for t in template.iter_mut() {
        *t /= beats.len() as f64;
    }

    // Compute average normalized cross-correlation
    let template_energy: f64 = template.iter().map(|x| x * x).sum();
    if template_energy < 1e-12 {
        return 0.0;
    }

    let mut total_corr = 0.0;
    let mut count = 0;
    for beat in &beats {
        let beat_energy: f64 = beat.iter().map(|x| x * x).sum();
        if beat_energy < 1e-12 {
            continue;
        }
        let cross: f64 = template
            .iter()
            .zip(beat.iter())
            .map(|(t, b)| t * b)
            .sum();
        let ncc = cross / (template_energy.sqrt() * beat_energy.sqrt());
        total_corr += ncc;
        count += 1;
    }

    if count == 0 {
        0.0
    } else {
        (total_corr / count as f64).clamp(0.0, 1.0)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- R ratio tests ----

    #[test]
    fn test_r_ratio_equal_modulation() {
        // When red and IR have equal AC/DC ratios, R = 1.0
        let r = compute_r_ratio(0.02, 1.0, 0.02, 1.0);
        assert!((r - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_r_ratio_high_spo2() {
        // High SpO2: red AC < IR AC relative to DC
        let r = compute_r_ratio(0.01, 1.0, 0.02, 1.0);
        assert!((r - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_r_ratio_low_spo2() {
        // Low SpO2: red AC > IR AC relative to DC
        let r = compute_r_ratio(0.04, 1.0, 0.02, 1.0);
        assert!((r - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_r_ratio_zero_dc_protection() {
        let r = compute_r_ratio(0.02, 0.0, 0.02, 1.0);
        assert_eq!(r, 0.0);

        let r = compute_r_ratio(0.02, 1.0, 0.02, 0.0);
        assert_eq!(r, 0.0);
    }

    #[test]
    fn test_r_ratio_zero_ir_ac_protection() {
        let r = compute_r_ratio(0.02, 1.0, 0.0, 1.0);
        assert_eq!(r, 0.0);
    }

    // ---- SpO2 from R tests ----

    #[test]
    fn test_spo2_from_r_zero() {
        // R = 0 => SpO2 = 110, clamped to 100
        let spo2 = r_to_spo2(0.0);
        assert!((spo2 - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_spo2_from_r_one() {
        // R = 1.0 => SpO2 = 85%
        let spo2 = r_to_spo2(1.0);
        assert!((spo2 - 85.0).abs() < 1e-10);
    }

    #[test]
    fn test_spo2_from_r_typical() {
        // R = 0.4 => SpO2 = 100
        let spo2 = r_to_spo2(0.4);
        assert!((spo2 - 100.0).abs() < 1e-10);

        // R = 0.6 => SpO2 = 95
        let spo2 = r_to_spo2(0.6);
        assert!((spo2 - 95.0).abs() < 1e-10);
    }

    #[test]
    fn test_spo2_clamp_low() {
        // Very high R should clamp to 0
        let spo2 = r_to_spo2(10.0);
        assert_eq!(spo2, 0.0);
    }

    #[test]
    fn test_spo2_clamp_high() {
        // Negative R should clamp to 100
        let spo2 = r_to_spo2(-1.0);
        assert_eq!(spo2, 100.0);
    }

    // ---- AC/DC separation tests ----

    #[test]
    fn test_ac_dc_pure_dc() {
        // Constant signal: DC = value, AC ~ 0
        let signal = vec![5.0; 500];
        let (ac, dc) = extract_ac_dc(&signal, 100.0);
        assert!((dc - 5.0).abs() < 0.01);
        assert!(ac < 0.01, "AC should be near zero for constant signal, got {ac}");
    }

    #[test]
    fn test_ac_dc_sinusoidal() {
        // Sinusoidal signal at 1.5 Hz on a DC offset of 10
        let fs = 200.0;
        let n = (fs * 5.0) as usize;
        let signal: Vec<f64> = (0..n)
            .map(|i| 10.0 + 0.5 * (2.0 * PI * 1.5 * i as f64 / fs).sin())
            .collect();
        let (ac, dc) = extract_ac_dc(&signal, fs);
        assert!((dc - 10.0).abs() < 0.1, "DC should be ~10, got {dc}");
        // RMS of sinusoid with amp 0.5 is 0.5/sqrt(2) ~ 0.354
        assert!(ac > 0.1, "AC should be nonzero for sinusoidal signal, got {ac}");
        assert!(ac < 1.0, "AC should be reasonable, got {ac}");
    }

    #[test]
    fn test_ac_dc_empty() {
        let (ac, dc) = extract_ac_dc(&[], 100.0);
        assert_eq!(ac, 0.0);
        assert_eq!(dc, 0.0);
    }

    // ---- Peak detection tests ----

    #[test]
    fn test_detect_peaks_synthetic_sine() {
        // 2 Hz sine at 200 Hz sample rate for 3 seconds => ~6 peaks
        let fs = 200.0;
        let n = (fs * 3.0) as usize;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 2.0 * i as f64 / fs).sin())
            .collect();
        let peaks = detect_ppg_peaks(&signal, fs);
        // Should detect approximately 6 peaks (2 Hz * 3 s)
        assert!(peaks.len() >= 4, "Expected >= 4 peaks, got {}", peaks.len());
        assert!(peaks.len() <= 8, "Expected <= 8 peaks, got {}", peaks.len());
    }

    #[test]
    fn test_detect_peaks_too_short() {
        let peaks = detect_ppg_peaks(&[1.0, 2.0], 100.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_detect_peaks_no_peaks() {
        // Monotonically increasing signal has no local maxima
        let signal: Vec<f64> = (0..500).map(|i| i as f64).collect();
        let peaks = detect_ppg_peaks(&signal, 100.0);
        // After bandpass filtering, may or may not find peaks; just check it doesn't panic
        let _ = peaks;
    }

    // ---- Heart rate computation tests ----

    #[test]
    fn test_heart_rate_60bpm() {
        // Peaks every 100 samples at 100 Hz => 1 second intervals => 60 BPM
        let peaks: Vec<usize> = (0..6).map(|i| i * 100).collect();
        let hr = compute_heart_rate(&peaks, 100.0);
        assert!((hr - 60.0).abs() < 0.1, "Expected ~60 BPM, got {hr}");
    }

    #[test]
    fn test_heart_rate_120bpm() {
        // Peaks every 50 samples at 100 Hz => 0.5 second intervals => 120 BPM
        let peaks: Vec<usize> = (0..10).map(|i| i * 50).collect();
        let hr = compute_heart_rate(&peaks, 100.0);
        assert!((hr - 120.0).abs() < 0.1, "Expected ~120 BPM, got {hr}");
    }

    #[test]
    fn test_heart_rate_insufficient_peaks() {
        assert_eq!(compute_heart_rate(&[], 100.0), 0.0);
        assert_eq!(compute_heart_rate(&[50], 100.0), 0.0);
    }

    #[test]
    fn test_heart_rate_zero_fs() {
        let peaks = vec![0, 100, 200];
        assert_eq!(compute_heart_rate(&peaks, 0.0), 0.0);
    }

    #[test]
    fn test_heart_rate_median_robust() {
        // Mostly 1-second intervals with one outlier => median should give 60 BPM
        let peaks = vec![0, 100, 200, 300, 700, 800, 900, 1000];
        let hr = compute_heart_rate(&peaks, 100.0);
        assert!((hr - 60.0).abs() < 1.0, "Median HR should be ~60, got {hr}");
    }

    // ---- Perfusion index tests ----

    #[test]
    fn test_perfusion_index_typical() {
        // AC=0.02, DC=1.0 => PI = 2.0%
        let pi = perfusion_index(0.02, 1.0);
        assert!((pi - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_perfusion_index_zero_dc() {
        assert_eq!(perfusion_index(0.02, 0.0), 0.0);
    }

    #[test]
    fn test_perfusion_index_large() {
        // Strong perfusion: AC = 0.2, DC = 1.0 => PI = 20%
        let pi = perfusion_index(0.2, 1.0);
        assert!((pi - 20.0).abs() < 1e-10);
    }

    // ---- Signal quality tests ----

    #[test]
    fn test_signal_quality_good_signal() {
        // Regular synthetic PPG should have decent quality
        let (_, ir) = generate_ppg_signal(72.0, 97.0, 200.0, 8.0);
        let sqi = signal_quality_index(&ir, 200.0);
        assert!(sqi > 0.3, "Good signal should have SQI > 0.3, got {sqi}");
    }

    #[test]
    fn test_signal_quality_noise() {
        // Pure pseudo-random noise should have low quality
        let mut noise = Vec::with_capacity(2000);
        let mut state = 42u64;
        for _ in 0..2000 {
            // Simple LCG for deterministic "noise"
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            noise.push((state >> 33) as f64 / (u32::MAX as f64) - 0.5);
        }
        let sqi = signal_quality_index(&noise, 200.0);
        assert!(sqi < 0.8, "Noise should have low SQI, got {sqi}");
    }

    #[test]
    fn test_signal_quality_too_short() {
        assert_eq!(signal_quality_index(&[1.0, 2.0], 100.0), 0.0);
    }

    // ---- Motion artifact removal tests ----

    #[test]
    fn test_adaptive_filter_basic() {
        // PPG + motion artifact; accelerometer as reference
        let fs = 100.0;
        let n = (fs * 5.0) as usize;

        let mut ppg_clean = Vec::with_capacity(n);
        let mut accel = Vec::with_capacity(n);

        for i in 0..n {
            let t = i as f64 / fs;
            // Clean PPG at 1.2 Hz (72 BPM)
            let ppg_val = (2.0 * PI * 1.2 * t).sin();
            // Motion artifact at 3 Hz
            let motion = 0.5 * (2.0 * PI * 3.0 * t).sin();
            ppg_clean.push(ppg_val);
            accel.push(motion);
        }

        let ppg_noisy: Vec<f64> = ppg_clean
            .iter()
            .zip(accel.iter())
            .map(|(p, m)| p + m)
            .collect();

        let filtered = adaptive_filter_motion(&ppg_noisy, &accel, 0.05);
        assert_eq!(filtered.len(), n);

        // After convergence, the filtered signal should have less motion artifact.
        // Check the second half (after the filter converges).
        let half = n / 2;
        let error_before: f64 = ppg_noisy[half..]
            .iter()
            .zip(ppg_clean[half..].iter())
            .map(|(n, c)| (n - c).powi(2))
            .sum::<f64>()
            / (n - half) as f64;

        let error_after: f64 = filtered[half..]
            .iter()
            .zip(ppg_clean[half..].iter())
            .map(|(f, c)| (f - c).powi(2))
            .sum::<f64>()
            / (n - half) as f64;

        assert!(
            error_after < error_before,
            "Adaptive filter should reduce error: before={error_before}, after={error_after}"
        );
    }

    #[test]
    fn test_adaptive_filter_empty() {
        let result = adaptive_filter_motion(&[], &[], 0.01);
        assert!(result.is_empty());
    }

    #[test]
    fn test_adaptive_filter_length_mismatch() {
        // Should use minimum length
        let ppg = vec![1.0; 100];
        let accel = vec![0.5; 50];
        let result = adaptive_filter_motion(&ppg, &accel, 0.01);
        assert_eq!(result.len(), 50);
    }

    // ---- Lowpass / highpass filter tests ----

    #[test]
    fn test_lowpass_filter_dc_passthrough() {
        let signal = vec![5.0; 1000];
        let filtered = lowpass_filter(&signal, 100.0, 10.0);
        assert_eq!(filtered.len(), 1000);
        // DC should pass through unchanged
        assert!((filtered[999] - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_lowpass_filter_attenuates_high_freq() {
        let fs = 1000.0;
        let n = (fs * 1.0) as usize;
        // 200 Hz signal, cutoff at 10 Hz
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 200.0 * i as f64 / fs).sin())
            .collect();
        let filtered = lowpass_filter(&signal, fs, 10.0);

        // Check that the filtered signal has lower power in steady state
        let input_power: f64 =
            signal[500..].iter().map(|x| x * x).sum::<f64>() / (n - 500) as f64;
        let output_power: f64 =
            filtered[500..].iter().map(|x| x * x).sum::<f64>() / (n - 500) as f64;

        assert!(
            output_power < input_power * 0.1,
            "Lowpass should attenuate 200 Hz: in_pow={input_power}, out_pow={output_power}"
        );
    }

    #[test]
    fn test_highpass_filter_blocks_dc() {
        let signal = vec![5.0; 1000];
        let filtered = highpass_filter(&signal, 100.0, 1.0);
        // After settling, output should be near zero for pure DC
        assert!(
            filtered[999].abs() < 0.1,
            "Highpass should block DC, got {}",
            filtered[999]
        );
    }

    #[test]
    fn test_lowpass_empty() {
        let filtered = lowpass_filter(&[], 100.0, 10.0);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_highpass_empty() {
        let filtered = highpass_filter(&[], 100.0, 1.0);
        assert!(filtered.is_empty());
    }

    // ---- Synthetic signal generation tests ----

    #[test]
    fn test_generate_ppg_signal_length() {
        let (red, ir) = generate_ppg_signal(72.0, 97.0, 100.0, 5.0);
        assert_eq!(red.len(), 500);
        assert_eq!(ir.len(), 500);
    }

    #[test]
    fn test_generate_ppg_signal_positive_dc() {
        let (red, ir) = generate_ppg_signal(72.0, 97.0, 100.0, 3.0);
        // Both channels should have positive values (absorption signals)
        assert!(red.iter().all(|&x| x > 0.0), "Red signal should be positive");
        assert!(ir.iter().all(|&x| x > 0.0), "IR signal should be positive");
    }

    #[test]
    fn test_generate_ppg_different_spo2() {
        let (red_high, _) = generate_ppg_signal(72.0, 98.0, 100.0, 5.0);
        let (red_low, _) = generate_ppg_signal(72.0, 80.0, 100.0, 5.0);

        // Lower SpO2 means higher R, meaning red has proportionally more AC
        let red_high_var: f64 = {
            let mean = red_high.iter().sum::<f64>() / red_high.len() as f64;
            red_high.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / red_high.len() as f64
        };
        let red_low_var: f64 = {
            let mean = red_low.iter().sum::<f64>() / red_low.len() as f64;
            red_low.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / red_low.len() as f64
        };

        assert!(
            red_low_var > red_high_var,
            "Lower SpO2 should give larger red AC: low_var={red_low_var}, high_var={red_high_var}"
        );
    }

    // ---- Roundtrip / integration tests ----

    #[test]
    fn test_roundtrip_spo2_97() {
        let target_spo2 = 97.0;
        let (red, ir) = generate_ppg_signal(72.0, target_spo2, 200.0, 10.0);
        let mut proc = PulseOxProcessor::new(PulseOxConfig {
            sample_rate_hz: 200.0,
            ..Default::default()
        });
        let result = proc.process(&red, &ir);

        assert!(
            (result.spo2_percent - target_spo2).abs() < 5.0,
            "SpO2 should be close to {target_spo2}, got {}",
            result.spo2_percent
        );
    }

    #[test]
    fn test_roundtrip_spo2_85() {
        let target_spo2 = 85.0;
        let (red, ir) = generate_ppg_signal(72.0, target_spo2, 200.0, 10.0);
        let mut proc = PulseOxProcessor::new(PulseOxConfig {
            sample_rate_hz: 200.0,
            ..Default::default()
        });
        let result = proc.process(&red, &ir);

        assert!(
            (result.spo2_percent - target_spo2).abs() < 8.0,
            "SpO2 should be close to {target_spo2}, got {}",
            result.spo2_percent
        );
    }

    #[test]
    fn test_roundtrip_heart_rate() {
        let target_hr = 72.0;
        let (red, ir) = generate_ppg_signal(target_hr, 97.0, 200.0, 10.0);
        let mut proc = PulseOxProcessor::new(PulseOxConfig {
            sample_rate_hz: 200.0,
            ..Default::default()
        });
        let result = proc.process(&red, &ir);

        assert!(
            (result.heart_rate_bpm - target_hr).abs() < 10.0,
            "HR should be close to {target_hr}, got {}",
            result.heart_rate_bpm
        );
    }

    #[test]
    fn test_roundtrip_heart_rate_fast() {
        let target_hr = 120.0;
        let (red, ir) = generate_ppg_signal(target_hr, 95.0, 200.0, 10.0);
        let mut proc = PulseOxProcessor::new(PulseOxConfig {
            sample_rate_hz: 200.0,
            ..Default::default()
        });
        let result = proc.process(&red, &ir);

        assert!(
            (result.heart_rate_bpm - target_hr).abs() < 15.0,
            "HR should be close to {target_hr}, got {}",
            result.heart_rate_bpm
        );
    }

    #[test]
    fn test_perfusion_index_in_result() {
        let (red, ir) = generate_ppg_signal(72.0, 97.0, 200.0, 10.0);
        let mut proc = PulseOxProcessor::new(PulseOxConfig {
            sample_rate_hz: 200.0,
            ..Default::default()
        });
        let result = proc.process(&red, &ir);
        // Perfusion index should be positive for a valid PPG signal
        assert!(result.perfusion_index > 0.0, "PI should be > 0, got {}", result.perfusion_index);
        assert!(result.perfusion_index < 50.0, "PI should be reasonable, got {}", result.perfusion_index);
    }

    #[test]
    fn test_r_ratio_in_result() {
        let (red, ir) = generate_ppg_signal(72.0, 97.0, 200.0, 10.0);
        let mut proc = PulseOxProcessor::new(PulseOxConfig {
            sample_rate_hz: 200.0,
            ..Default::default()
        });
        let result = proc.process(&red, &ir);
        // R should be in the physiological range
        assert!(result.r_ratio > 0.0, "R ratio should be > 0, got {}", result.r_ratio);
        assert!(result.r_ratio < 3.0, "R ratio should be < 3, got {}", result.r_ratio);
    }

    #[test]
    fn test_default_config() {
        let config = PulseOxConfig::default();
        assert_eq!(config.sample_rate_hz, 100.0);
        assert_eq!(config.red_wavelength_nm, 660.0);
        assert_eq!(config.ir_wavelength_nm, 940.0);
        assert_eq!(config.window_size_s, 4.0);
    }

    #[test]
    fn test_coefficient_of_variation() {
        // Identical values: CV = 0
        let cv = coefficient_of_variation(&[5.0, 5.0, 5.0, 5.0]);
        assert!(cv < 1e-10);

        // Known values: [2, 4, 4, 4, 5, 5, 7, 9] => mean=5, std~2.0
        let cv = coefficient_of_variation(&[2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]);
        assert!(cv > 0.3 && cv < 0.5, "CV should be ~0.4, got {cv}");
    }

    #[test]
    fn test_process_equal_length_assertion() {
        let mut proc = PulseOxProcessor::new(PulseOxConfig::default());
        let red = vec![1.0; 100];
        let ir = vec![1.0; 100];
        // Should not panic
        let _ = proc.process(&red, &ir);
    }

    #[test]
    #[should_panic(expected = "red and ir buffers must have equal length")]
    fn test_process_unequal_length_panics() {
        let mut proc = PulseOxProcessor::new(PulseOxConfig::default());
        let red = vec![1.0; 100];
        let ir = vec![1.0; 50];
        let _ = proc.process(&red, &ir);
    }
}

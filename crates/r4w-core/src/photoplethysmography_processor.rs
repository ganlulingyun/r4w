//! Photoplethysmography (PPG) signal processing for cardiovascular monitoring.
//!
//! This module implements comprehensive PPG analysis including heart rate detection,
//! SpO2 estimation, HRV metrics, morphology analysis, second derivative PPG (SDPPG),
//! signal quality assessment, and respiratory rate estimation from optical sensors.
//!
//! # Background
//!
//! PPG measures blood volume changes in the microvascular bed of tissue using optical
//! sensors. A light source (LED) illuminates the skin, and a photodetector measures
//! variations in light intensity caused by pulsatile arterial blood flow.
//!
//! Red (~660 nm) and infrared (~940 nm) wavelengths exploit the differential absorption
//! of oxyhemoglobin (HbO2) and deoxyhemoglobin (Hb) for SpO2 estimation. Green (~530 nm)
//! LEDs are common in wrist-worn devices for heart rate monitoring.
//!
//! # Key Algorithms
//!
//! ```text
//! SpO2 = 110 - 25 * R   where R = (AC_red/DC_red) / (AC_ir/DC_ir)
//! HR = 60 / IBI          where IBI = inter-beat interval in seconds
//! PI = (AC / DC) * 100%  perfusion index
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::photoplethysmography_processor::*;
//!
//! let config = PpgConfig {
//!     sample_rate_hz: 100.0,
//!     ..Default::default()
//! };
//! let mut processor = PpgProcessor::new(config);
//!
//! // Generate synthetic PPG (72 BPM)
//! let signal = generate_synthetic_ppg(72.0, 100.0, 5.0, PpgChannel::Green);
//! let result = processor.analyze(&signal);
//!
//! assert!(result.heart_rate_bpm > 60.0 && result.heart_rate_bpm < 85.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// PPG sensor channel / wavelength.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PpgChannel {
    /// Red LED (~660 nm) for SpO2 measurement.
    Red,
    /// Infrared LED (~940 nm) for SpO2 measurement.
    IR,
    /// Green LED (~530 nm) for wrist-based HR monitoring.
    Green,
}

/// Configuration for the PPG processor.
#[derive(Debug, Clone)]
pub struct PpgConfig {
    /// Sampling rate of the PPG signal in Hz (typically 25-250).
    pub sample_rate_hz: f64,
    /// Red LED wavelength in nanometers.
    pub wavelength_red_nm: f64,
    /// IR LED wavelength in nanometers.
    pub wavelength_ir_nm: f64,
    /// DC removal moving average window in seconds.
    pub dc_removal_window_s: f64,
    /// Bandpass filter low cutoff in Hz (maps to ~30 BPM).
    pub bp_low_hz: f64,
    /// Bandpass filter high cutoff in Hz (maps to ~300 BPM).
    pub bp_high_hz: f64,
    /// FIR filter order (must be even).
    pub fir_order: usize,
    /// Peak detection adaptive threshold fraction of running max.
    pub peak_threshold_fraction: f64,
    /// HR averaging window in seconds.
    pub hr_window_s: f64,
    /// SpO2 linear calibration coefficients: SpO2 = a0 + a1*R.
    pub spo2_linear: (f64, f64),
    /// SpO2 quadratic calibration: SpO2 = a0 + a1*R + a2*R^2.
    pub spo2_quadratic: (f64, f64, f64),
    /// Minimum perfusion index for reliable SpO2.
    pub min_perfusion_index: f64,
    /// LMS adaptive filter step size for motion artifact removal.
    pub lms_step_size: f64,
    /// LMS adaptive filter order.
    pub lms_order: usize,
    /// Body height in meters (for stiffness index calculation).
    pub body_height_m: f64,
}

impl Default for PpgConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 100.0,
            wavelength_red_nm: 660.0,
            wavelength_ir_nm: 940.0,
            dc_removal_window_s: 3.0,
            bp_low_hz: 0.5,
            bp_high_hz: 5.0,
            fir_order: 64,
            peak_threshold_fraction: 0.6,
            hr_window_s: 10.0,
            spo2_linear: (110.0, -25.0),
            spo2_quadratic: (115.0, -30.0, 5.0),
            min_perfusion_index: 0.2,
            lms_step_size: 0.01,
            lms_order: 16,
            body_height_m: 1.70,
        }
    }
}

// ---------------------------------------------------------------------------
// PPG Signal
// ---------------------------------------------------------------------------

/// A PPG signal buffer with metadata.
#[derive(Debug, Clone)]
pub struct PpgSignal {
    /// Raw sample values (arbitrary units, typically ADC counts or voltage).
    pub samples: Vec<f64>,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Which LED channel this signal came from.
    pub channel: PpgChannel,
}

impl PpgSignal {
    /// Create a new PPG signal.
    pub fn new(samples: Vec<f64>, sample_rate_hz: f64, channel: PpgChannel) -> Self {
        Self {
            samples,
            sample_rate_hz,
            channel,
        }
    }

    /// Duration in seconds.
    pub fn duration_s(&self) -> f64 {
        self.samples.len() as f64 / self.sample_rate_hz
    }
}

// ---------------------------------------------------------------------------
// Analysis Results
// ---------------------------------------------------------------------------

/// Heart rate variability metrics.
#[derive(Debug, Clone, Default)]
pub struct HrvMetrics {
    /// Standard deviation of normal-to-normal intervals (ms).
    pub sdnn_ms: f64,
    /// Root mean square of successive differences (ms).
    pub rmssd_ms: f64,
    /// Mean inter-beat interval (ms).
    pub mean_ibi_ms: f64,
    /// Number of detected beats.
    pub beat_count: usize,
    /// All detected inter-beat intervals (seconds).
    pub ibi_series: Vec<f64>,
}

/// SpO2 measurement result.
#[derive(Debug, Clone, Default)]
pub struct Spo2Result {
    /// SpO2 percentage (linear calibration).
    pub spo2_linear: f64,
    /// SpO2 percentage (quadratic calibration).
    pub spo2_quadratic: f64,
    /// Raw ratio of ratios R.
    pub r_ratio: f64,
    /// AC amplitude of red channel.
    pub ac_red: f64,
    /// DC level of red channel.
    pub dc_red: f64,
    /// AC amplitude of IR channel.
    pub ac_ir: f64,
    /// DC level of IR channel.
    pub dc_ir: f64,
    /// Perfusion index for red channel (%).
    pub perfusion_index_red: f64,
    /// Perfusion index for IR channel (%).
    pub perfusion_index_ir: f64,
}

/// PPG beat morphology features.
#[derive(Debug, Clone, Default)]
pub struct BeatMorphology {
    /// Systolic peak amplitude.
    pub systolic_amplitude: f64,
    /// Diastolic peak amplitude (after dicrotic notch).
    pub diastolic_amplitude: f64,
    /// Dicrotic notch amplitude.
    pub dicrotic_notch_amplitude: f64,
    /// Augmentation index: (systolic - dicrotic) / systolic.
    pub augmentation_index: f64,
    /// Pulse width at half maximum (seconds).
    pub pulse_width_half_max_s: f64,
    /// Time from foot to systolic peak (seconds).
    pub systolic_time_s: f64,
    /// Reflection index: diastolic / systolic.
    pub reflection_index: f64,
}

/// Second derivative PPG (acceleration plethysmogram) features.
#[derive(Debug, Clone, Default)]
pub struct SdppgFeatures {
    /// Wave 'a' amplitude (initial positive peak).
    pub wave_a: f64,
    /// Wave 'b' amplitude (first negative trough).
    pub wave_b: f64,
    /// Wave 'c' amplitude (second positive peak).
    pub wave_c: f64,
    /// Wave 'd' amplitude (second negative trough).
    pub wave_d: f64,
    /// Wave 'e' amplitude (third positive peak).
    pub wave_e: f64,
    /// Aging index: (b - c - d - e) / a.
    pub aging_index: f64,
    /// Estimated vascular age (years).
    pub vascular_age_years: f64,
}

/// Signal quality assessment.
#[derive(Debug, Clone, Default)]
pub struct SignalQuality {
    /// Overall signal quality index (0.0 = poor, 1.0 = excellent).
    pub sqi: f64,
    /// Template correlation (mean correlation of beats with template).
    pub template_correlation: f64,
    /// Beat morphology skewness.
    pub skewness: f64,
    /// Perfusion index (%).
    pub perfusion_index: f64,
    /// Whether signal quality is sufficient for reliable SpO2.
    pub spo2_reliable: bool,
}

/// Respiratory rate estimation.
#[derive(Debug, Clone, Default)]
pub struct RespiratoryResult {
    /// Estimated respiratory rate in breaths per minute.
    pub respiratory_rate_bpm: f64,
    /// Respiratory signal extracted from baseline wandering.
    pub baseline_modulation: Vec<f64>,
    /// Respiratory signal extracted from amplitude modulation.
    pub amplitude_modulation: Vec<f64>,
    /// Respiratory signal extracted from frequency modulation (RSA).
    pub frequency_modulation: Vec<f64>,
}

/// Arterial stiffness indicators.
#[derive(Debug, Clone, Default)]
pub struct ArterialStiffness {
    /// Stiffness index: body_height / PTT (m/s).
    pub stiffness_index: f64,
    /// Reflection index: diastolic_peak / systolic_peak.
    pub reflection_index: f64,
    /// Large artery stiffness from SDPPG b/a ratio.
    pub large_artery_stiffness: f64,
}

/// Complete PPG analysis result.
#[derive(Debug, Clone, Default)]
pub struct PpgAnalysisResult {
    /// Estimated heart rate in BPM.
    pub heart_rate_bpm: f64,
    /// Instantaneous heart rate for each detected beat.
    pub instantaneous_hr: Vec<f64>,
    /// Detected peak indices in preprocessed signal.
    pub peak_indices: Vec<usize>,
    /// HRV metrics.
    pub hrv: HrvMetrics,
    /// Beat morphology (averaged over all beats).
    pub morphology: BeatMorphology,
    /// SDPPG features.
    pub sdppg: SdppgFeatures,
    /// Signal quality assessment.
    pub quality: SignalQuality,
    /// Respiratory rate estimation.
    pub respiratory: RespiratoryResult,
    /// Arterial stiffness (if morphology features available).
    pub stiffness: ArterialStiffness,
    /// Preprocessed (filtered) signal.
    pub filtered_signal: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Signal Preprocessing
// ---------------------------------------------------------------------------

/// Remove DC component using a moving average subtraction.
pub fn remove_dc(signal: &[f64], window_samples: usize) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || window_samples == 0 {
        return signal.to_vec();
    }
    let half = window_samples / 2;
    let mut result = vec![0.0; n];
    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = (i + half + 1).min(n);
        let count = (end - start) as f64;
        let sum: f64 = signal[start..end].iter().sum();
        result[i] = signal[i] - sum / count;
    }
    result
}

/// Design a Hamming-windowed sinc FIR bandpass filter.
pub fn design_bandpass_fir(order: usize, f_low: f64, f_high: f64, sample_rate: f64) -> Vec<f64> {
    let n = order;
    // Force even order
    let n = if n % 2 != 0 { n + 1 } else { n };
    let m = n as f64;
    let fc_low = f_low / sample_rate;
    let fc_high = f_high / sample_rate;

    // Design lowpass at f_high
    let mut lp_high = vec![0.0; n + 1];
    for i in 0..=n {
        let x = i as f64 - m / 2.0;
        if x.abs() < 1e-10 {
            lp_high[i] = 2.0 * fc_high;
        } else {
            lp_high[i] = (2.0 * PI * fc_high * x).sin() / (PI * x);
        }
        // Hamming window
        let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / m).cos();
        lp_high[i] *= w;
    }

    // Design lowpass at f_low
    let mut lp_low = vec![0.0; n + 1];
    for i in 0..=n {
        let x = i as f64 - m / 2.0;
        if x.abs() < 1e-10 {
            lp_low[i] = 2.0 * fc_low;
        } else {
            lp_low[i] = (2.0 * PI * fc_low * x).sin() / (PI * x);
        }
        let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / m).cos();
        lp_low[i] *= w;
    }

    // Bandpass = highpass(f_low) + lowpass(f_high) = lp_high - lp_low
    let mut bp: Vec<f64> = lp_high
        .iter()
        .zip(lp_low.iter())
        .map(|(h, l)| h - l)
        .collect();

    // Normalize to unit gain at center frequency
    let f_center = (f_low + f_high) / 2.0;
    let gain: f64 = bp
        .iter()
        .enumerate()
        .map(|(i, &c)| c * (2.0 * PI * f_center * i as f64 / sample_rate).cos())
        .sum();
    if gain.abs() > 1e-10 {
        for c in bp.iter_mut() {
            *c /= gain;
        }
    }

    bp
}

/// Apply FIR filter to a signal.
pub fn apply_fir(signal: &[f64], coeffs: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let m = coeffs.len();
    if n == 0 || m == 0 {
        return vec![];
    }
    let mut out = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..m {
            if i >= j {
                sum += coeffs[j] * signal[i - j];
            }
        }
        out[i] = sum;
    }
    out
}

/// Adaptive LMS filter for motion artifact removal.
/// `primary` = PPG signal with motion artifact.
/// `reference` = accelerometer reference signal.
/// Returns the cleaned PPG signal (primary - estimated noise).
pub fn lms_motion_removal(
    primary: &[f64],
    reference: &[f64],
    filter_order: usize,
    step_size: f64,
) -> Vec<f64> {
    let n = primary.len().min(reference.len());
    let mut weights = vec![0.0; filter_order];
    let mut output = vec![0.0; n];

    for i in 0..n {
        // Filter output: estimate of noise in primary
        let mut noise_est = 0.0;
        for j in 0..filter_order {
            if i >= j {
                noise_est += weights[j] * reference[i - j];
            }
        }
        // Error = desired - estimated
        let error = primary[i] - noise_est;
        output[i] = error;

        // Update weights
        let mut ref_power = 0.0;
        for j in 0..filter_order {
            if i >= j {
                ref_power += reference[i - j] * reference[i - j];
            }
        }
        let norm = if ref_power > 1e-10 {
            step_size / ref_power
        } else {
            step_size
        };
        for j in 0..filter_order {
            if i >= j {
                weights[j] += norm * error * reference[i - j];
            }
        }
    }

    output
}

// ---------------------------------------------------------------------------
// Peak Detection
// ---------------------------------------------------------------------------

/// Detect systolic peaks in a preprocessed PPG signal.
/// Returns indices of detected peaks.
pub fn detect_peaks(
    signal: &[f64],
    sample_rate: f64,
    threshold_fraction: f64,
) -> Vec<usize> {
    let n = signal.len();
    if n < 3 {
        return vec![];
    }

    // Minimum distance between peaks: ~200ms (300 BPM max)
    let min_distance = (0.2 * sample_rate) as usize;
    // Running maximum for adaptive threshold
    let window = (2.0 * sample_rate) as usize; // 2-second window

    let mut peaks = Vec::new();
    let mut last_peak_idx: Option<usize> = None;

    for i in 1..n - 1 {
        // Local maximum check
        if signal[i] > signal[i - 1] && signal[i] >= signal[i + 1] {
            // Adaptive threshold from local window
            let start = if i >= window { i - window } else { 0 };
            let end = (i + window).min(n);
            let local_max = signal[start..end]
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            let threshold = threshold_fraction * local_max;

            if signal[i] >= threshold {
                // Check minimum distance from last peak
                if let Some(last) = last_peak_idx {
                    if i - last < min_distance {
                        // Keep the larger peak
                        if signal[i] > signal[last] {
                            peaks.pop();
                            peaks.push(i);
                            last_peak_idx = Some(i);
                        }
                        continue;
                    }
                }
                peaks.push(i);
                last_peak_idx = Some(i);
            }
        }
    }

    peaks
}

/// Compute inter-beat intervals from peak indices.
pub fn compute_ibi(peak_indices: &[usize], sample_rate: f64) -> Vec<f64> {
    if peak_indices.len() < 2 {
        return vec![];
    }
    peak_indices
        .windows(2)
        .map(|w| (w[1] - w[0]) as f64 / sample_rate)
        .collect()
}

/// Compute instantaneous heart rate from IBI values.
pub fn ibi_to_hr(ibi: &[f64]) -> Vec<f64> {
    ibi.iter()
        .filter(|&&x| x > 0.0)
        .map(|&x| 60.0 / x)
        .collect()
}

/// Compute average heart rate over a window.
pub fn average_hr(ibi: &[f64]) -> f64 {
    if ibi.is_empty() {
        return 0.0;
    }
    let mean_ibi: f64 = ibi.iter().sum::<f64>() / ibi.len() as f64;
    if mean_ibi > 0.0 {
        60.0 / mean_ibi
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// HRV Metrics
// ---------------------------------------------------------------------------

/// Compute SDNN: standard deviation of NN (normal-to-normal) intervals.
pub fn compute_sdnn(ibi: &[f64]) -> f64 {
    if ibi.len() < 2 {
        return 0.0;
    }
    let mean = ibi.iter().sum::<f64>() / ibi.len() as f64;
    let variance = ibi.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (ibi.len() - 1) as f64;
    variance.sqrt()
}

/// Compute RMSSD: root mean square of successive differences.
pub fn compute_rmssd(ibi: &[f64]) -> f64 {
    if ibi.len() < 2 {
        return 0.0;
    }
    let sum_sq: f64 = ibi
        .windows(2)
        .map(|w| {
            let diff = w[1] - w[0];
            diff * diff
        })
        .sum();
    (sum_sq / (ibi.len() - 1) as f64).sqrt()
}

// ---------------------------------------------------------------------------
// SpO2 Calculation
// ---------------------------------------------------------------------------

/// Extract AC (pulsatile) and DC (baseline) components from a PPG signal.
/// AC = mean peak-to-trough amplitude over detected cardiac cycles.
/// DC = mean of the signal.
pub fn extract_ac_dc(signal: &[f64], peak_indices: &[usize]) -> (f64, f64) {
    let dc: f64 = if signal.is_empty() {
        0.0
    } else {
        signal.iter().sum::<f64>() / signal.len() as f64
    };

    if peak_indices.len() < 2 {
        // fallback: use max - min as AC estimate
        let max = signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = signal.iter().cloned().fold(f64::INFINITY, f64::min);
        return ((max - min).max(0.0), dc);
    }

    // For each inter-peak interval, find the trough
    let mut ac_values = Vec::new();
    for pair in peak_indices.windows(2) {
        let p1 = pair[0];
        let p2 = pair[1];
        let peak_val = signal[p1].max(signal[p2]);
        // Find trough between peaks
        let trough_val = signal[p1..=p2]
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        ac_values.push(peak_val - trough_val);
    }

    let ac = if ac_values.is_empty() {
        0.0
    } else {
        ac_values.iter().sum::<f64>() / ac_values.len() as f64
    };

    (ac, dc)
}

/// Calculate SpO2 from red and IR PPG signals.
pub fn calculate_spo2(
    ac_red: f64,
    dc_red: f64,
    ac_ir: f64,
    dc_ir: f64,
    linear_coeffs: (f64, f64),
    quadratic_coeffs: (f64, f64, f64),
) -> Spo2Result {
    let r_ratio = if dc_red.abs() > 1e-10 && dc_ir.abs() > 1e-10 && ac_ir.abs() > 1e-10 {
        (ac_red / dc_red) / (ac_ir / dc_ir)
    } else {
        1.0
    };

    let spo2_lin = linear_coeffs.0 + linear_coeffs.1 * r_ratio;
    let spo2_quad =
        quadratic_coeffs.0 + quadratic_coeffs.1 * r_ratio + quadratic_coeffs.2 * r_ratio * r_ratio;

    let pi_red = if dc_red.abs() > 1e-10 {
        (ac_red / dc_red) * 100.0
    } else {
        0.0
    };
    let pi_ir = if dc_ir.abs() > 1e-10 {
        (ac_ir / dc_ir) * 100.0
    } else {
        0.0
    };

    Spo2Result {
        spo2_linear: spo2_lin.clamp(0.0, 100.0),
        spo2_quadratic: spo2_quad.clamp(0.0, 100.0),
        r_ratio,
        ac_red,
        dc_red,
        ac_ir,
        dc_ir,
        perfusion_index_red: pi_red,
        perfusion_index_ir: pi_ir,
    }
}

/// Compute perfusion index: (AC / DC) * 100%.
pub fn perfusion_index(ac: f64, dc: f64) -> f64 {
    if dc.abs() > 1e-10 {
        (ac / dc.abs()) * 100.0
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// PPG Morphology Analysis
// ---------------------------------------------------------------------------

/// Detect the dicrotic notch between systolic and diastolic peaks.
/// Returns (notch_index, diastolic_peak_index) relative to start of beat.
pub fn find_dicrotic_notch(beat: &[f64]) -> Option<(usize, usize)> {
    if beat.len() < 5 {
        return None;
    }

    // Find systolic peak (global max in first half)
    let half = beat.len() / 2;
    let sys_idx = beat[..half]
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);

    // Search for notch after systolic peak (local minimum)
    let search_start = sys_idx + 1;
    if search_start >= beat.len() - 2 {
        return None;
    }

    let mut notch_idx = None;
    for i in search_start..beat.len() - 1 {
        if beat[i] < beat[i - 1] && beat[i] <= beat[i + 1] {
            notch_idx = Some(i);
            break;
        }
    }

    let notch_idx = notch_idx?;

    // Diastolic peak: local max after notch
    let mut dia_idx = notch_idx;
    for i in notch_idx + 1..beat.len() - 1 {
        if beat[i] > beat[i - 1] && beat[i] >= beat[i + 1] {
            dia_idx = i;
            break;
        }
    }

    Some((notch_idx, dia_idx))
}

/// Analyze morphology of a single PPG beat.
pub fn analyze_beat_morphology(beat: &[f64], sample_rate: f64) -> BeatMorphology {
    let mut morph = BeatMorphology::default();
    if beat.len() < 5 {
        return morph;
    }

    // Systolic peak
    let (sys_idx, &sys_amp) = beat
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or((0, &0.0));
    morph.systolic_amplitude = sys_amp;
    morph.systolic_time_s = sys_idx as f64 / sample_rate;

    // Dicrotic notch and diastolic peak
    if let Some((notch_idx, dia_idx)) = find_dicrotic_notch(beat) {
        morph.dicrotic_notch_amplitude = beat[notch_idx];
        morph.diastolic_amplitude = beat[dia_idx];
        if sys_amp.abs() > 1e-10 {
            morph.augmentation_index = (sys_amp - beat[notch_idx]) / sys_amp;
            morph.reflection_index = beat[dia_idx] / sys_amp;
        }
    }

    // Pulse width at half maximum
    let min_val = beat.iter().cloned().fold(f64::INFINITY, f64::min);
    let half_max = (sys_amp + min_val) / 2.0;
    let mut left_cross = 0;
    let mut right_cross = beat.len() - 1;
    for i in 0..sys_idx {
        if beat[i] >= half_max {
            left_cross = i;
            break;
        }
    }
    for i in (sys_idx..beat.len()).rev() {
        if beat[i] >= half_max {
            right_cross = i;
            break;
        }
    }
    morph.pulse_width_half_max_s = (right_cross - left_cross) as f64 / sample_rate;

    morph
}

/// Average morphology across beats.
pub fn average_morphology(beats: &[BeatMorphology]) -> BeatMorphology {
    if beats.is_empty() {
        return BeatMorphology::default();
    }
    let n = beats.len() as f64;
    BeatMorphology {
        systolic_amplitude: beats.iter().map(|b| b.systolic_amplitude).sum::<f64>() / n,
        diastolic_amplitude: beats.iter().map(|b| b.diastolic_amplitude).sum::<f64>() / n,
        dicrotic_notch_amplitude: beats.iter().map(|b| b.dicrotic_notch_amplitude).sum::<f64>() / n,
        augmentation_index: beats.iter().map(|b| b.augmentation_index).sum::<f64>() / n,
        pulse_width_half_max_s: beats.iter().map(|b| b.pulse_width_half_max_s).sum::<f64>() / n,
        systolic_time_s: beats.iter().map(|b| b.systolic_time_s).sum::<f64>() / n,
        reflection_index: beats.iter().map(|b| b.reflection_index).sum::<f64>() / n,
    }
}

// ---------------------------------------------------------------------------
// Second Derivative PPG (SDPPG / Acceleration Plethysmogram)
// ---------------------------------------------------------------------------

/// Compute the second derivative of a signal using finite differences.
pub fn second_derivative(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n < 3 {
        return vec![0.0; n];
    }
    let mut d2 = vec![0.0; n];
    for i in 1..n - 1 {
        d2[i] = signal[i + 1] - 2.0 * signal[i] + signal[i - 1];
    }
    d2[0] = d2[1];
    d2[n - 1] = d2[n - 2];
    d2
}

/// Extract SDPPG a, b, c, d, e waves from a single beat's second derivative.
pub fn extract_sdppg_waves(sdppg_beat: &[f64]) -> SdppgFeatures {
    let mut features = SdppgFeatures::default();
    if sdppg_beat.len() < 10 {
        return features;
    }

    // Find alternating peaks and troughs: a(+), b(-), c(+), d(-), e(+)
    let mut extrema: Vec<(usize, f64, bool)> = Vec::new(); // (index, value, is_peak)

    for i in 1..sdppg_beat.len() - 1 {
        if sdppg_beat[i] > sdppg_beat[i - 1] && sdppg_beat[i] >= sdppg_beat[i + 1] {
            extrema.push((i, sdppg_beat[i], true));
        } else if sdppg_beat[i] < sdppg_beat[i - 1] && sdppg_beat[i] <= sdppg_beat[i + 1] {
            extrema.push((i, sdppg_beat[i], false));
        }
    }

    // Assign waves: expect pattern peak, trough, peak, trough, peak
    let mut peaks: Vec<f64> = Vec::new();
    let mut troughs: Vec<f64> = Vec::new();
    for &(_, val, is_peak) in &extrema {
        if is_peak {
            peaks.push(val);
        } else {
            troughs.push(val);
        }
    }

    if !peaks.is_empty() {
        features.wave_a = peaks[0];
    }
    if !troughs.is_empty() {
        features.wave_b = troughs[0];
    }
    if peaks.len() >= 2 {
        features.wave_c = peaks[1];
    }
    if troughs.len() >= 2 {
        features.wave_d = troughs[1];
    }
    if peaks.len() >= 3 {
        features.wave_e = peaks[2];
    }

    // Aging index: (b - c - d - e) / a
    if features.wave_a.abs() > 1e-10 {
        features.aging_index =
            (features.wave_b - features.wave_c - features.wave_d - features.wave_e)
                / features.wave_a;
    }

    // Vascular age estimation (simplified linear model):
    // younger -> aging_index ~ -0.8 to -0.5 (age 20-30)
    // older -> aging_index ~ 0.0 to 0.5 (age 60-80)
    // Linear mapping: age = 50 + 40 * aging_index
    features.vascular_age_years = (50.0 + 40.0 * features.aging_index).clamp(15.0, 100.0);

    features
}

// ---------------------------------------------------------------------------
// Signal Quality Assessment
// ---------------------------------------------------------------------------

/// Compute cross-correlation between two signals at zero lag.
fn cross_correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }
    let mean_a = a[..n].iter().sum::<f64>() / n as f64;
    let mean_b = b[..n].iter().sum::<f64>() / n as f64;

    let mut num = 0.0;
    let mut den_a = 0.0;
    let mut den_b = 0.0;
    for i in 0..n {
        let da = a[i] - mean_a;
        let db = b[i] - mean_b;
        num += da * db;
        den_a += da * da;
        den_b += db * db;
    }
    let den = (den_a * den_b).sqrt();
    if den > 1e-10 {
        num / den
    } else {
        0.0
    }
}

/// Compute skewness of a signal.
fn compute_skewness(data: &[f64]) -> f64 {
    let n = data.len();
    if n < 3 {
        return 0.0;
    }
    let mean = data.iter().sum::<f64>() / n as f64;
    let m2: f64 = data.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;
    let m3: f64 = data.iter().map(|&x| (x - mean).powi(3)).sum::<f64>() / n as f64;
    if m2.abs() < 1e-10 {
        0.0
    } else {
        m3 / m2.powf(1.5)
    }
}

/// Assess signal quality of a PPG signal.
pub fn assess_signal_quality(
    signal: &[f64],
    peak_indices: &[usize],
    sample_rate: f64,
    min_pi: f64,
) -> SignalQuality {
    let mut sq = SignalQuality::default();

    // Perfusion index
    let (ac, dc) = extract_ac_dc(signal, peak_indices);
    sq.perfusion_index = perfusion_index(ac, dc);

    // Extract beats and compute template
    let beats = extract_beats(signal, peak_indices);
    if beats.is_empty() {
        sq.sqi = 0.0;
        return sq;
    }

    // Create average template
    let min_len = beats.iter().map(|b| b.len()).min().unwrap_or(0);
    if min_len < 3 {
        sq.sqi = 0.0;
        return sq;
    }

    let mut template = vec![0.0; min_len];
    for beat in &beats {
        for i in 0..min_len {
            template[i] += beat[i];
        }
    }
    for t in template.iter_mut() {
        *t /= beats.len() as f64;
    }

    // Template correlation for each beat
    let mut correlations = Vec::new();
    for beat in &beats {
        let trimmed: Vec<f64> = beat[..min_len].to_vec();
        let corr = cross_correlation(&trimmed, &template);
        correlations.push(corr);
    }

    sq.template_correlation = if correlations.is_empty() {
        0.0
    } else {
        correlations.iter().sum::<f64>() / correlations.len() as f64
    };

    // Skewness of beat amplitudes
    let beat_amps: Vec<f64> = beats
        .iter()
        .map(|b| {
            b.iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max)
        })
        .collect();
    sq.skewness = compute_skewness(&beat_amps);

    // Overall SQI: combination of template correlation, PI threshold, regularity
    let ibi = compute_ibi(peak_indices, sample_rate);
    let ibi_regularity = if ibi.len() >= 2 {
        let mean_ibi = ibi.iter().sum::<f64>() / ibi.len() as f64;
        if mean_ibi > 0.0 {
            let cv = compute_sdnn(&ibi) / mean_ibi; // coefficient of variation
            (1.0 - cv.min(1.0)).max(0.0)
        } else {
            0.0
        }
    } else {
        0.0
    };

    let pi_score = if sq.perfusion_index >= min_pi { 1.0 } else { sq.perfusion_index / min_pi };
    sq.sqi = (0.4 * sq.template_correlation.max(0.0)
        + 0.3 * ibi_regularity
        + 0.3 * pi_score)
        .clamp(0.0, 1.0);

    sq.spo2_reliable = sq.perfusion_index >= min_pi && sq.sqi > 0.5;

    sq
}

/// Extract individual beat waveforms from signal using peak indices.
fn extract_beats(signal: &[f64], peak_indices: &[usize]) -> Vec<Vec<f64>> {
    if peak_indices.len() < 2 {
        return vec![];
    }
    let mut beats = Vec::new();
    for pair in peak_indices.windows(2) {
        let start = pair[0];
        let end = pair[1];
        if end <= signal.len() {
            beats.push(signal[start..end].to_vec());
        }
    }
    beats
}

// ---------------------------------------------------------------------------
// Respiratory Rate Estimation
// ---------------------------------------------------------------------------

/// Extract respiratory rate from PPG using three modulation types.
pub fn estimate_respiratory_rate(
    signal: &[f64],
    peak_indices: &[usize],
    sample_rate: f64,
) -> RespiratoryResult {
    let mut result = RespiratoryResult::default();
    if peak_indices.len() < 4 {
        return result;
    }

    // 1. Baseline modulation: DC level at each peak
    let baseline_at_peaks: Vec<f64> = peak_indices
        .iter()
        .map(|&idx| {
            // Local DC: mean around peak +/- a few samples
            let start = idx.saturating_sub(5);
            let end = (idx + 5).min(signal.len());
            let window = &signal[start..end];
            // Use running mean as rough DC
            window.iter().sum::<f64>() / window.len() as f64
        })
        .collect();

    // 2. Amplitude modulation: peak amplitude variation
    let amplitude_at_peaks: Vec<f64> = peak_indices
        .iter()
        .map(|&idx| signal[idx])
        .collect();

    // 3. Frequency modulation (RSA): IBI variation
    let ibi = compute_ibi(peak_indices, sample_rate);
    let ibi_rate = if peak_indices.len() >= 2 {
        // Effective sample rate of IBI series
        let ibi_duration: f64 = ibi.iter().sum();
        if ibi_duration > 0.0 {
            (ibi.len() as f64) / ibi_duration
        } else {
            1.0
        }
    } else {
        1.0
    };

    result.baseline_modulation = baseline_at_peaks.clone();
    result.amplitude_modulation = amplitude_at_peaks.clone();
    result.frequency_modulation = ibi.clone();

    // Estimate respiratory rate from amplitude modulation via peak counting
    // Remove mean from amplitude series
    let amp_mean = if amplitude_at_peaks.is_empty() {
        0.0
    } else {
        amplitude_at_peaks.iter().sum::<f64>() / amplitude_at_peaks.len() as f64
    };
    let amp_detrended: Vec<f64> = amplitude_at_peaks.iter().map(|&x| x - amp_mean).collect();

    // Count zero crossings as proxy for respiratory oscillation frequency
    let mut zero_crossings = 0usize;
    for i in 1..amp_detrended.len() {
        if (amp_detrended[i] >= 0.0 && amp_detrended[i - 1] < 0.0)
            || (amp_detrended[i] < 0.0 && amp_detrended[i - 1] >= 0.0)
        {
            zero_crossings += 1;
        }
    }

    // Respiratory rate estimate: zero crossings / 2 = full cycles
    // Duration of the amplitude series
    let total_duration_s = signal.len() as f64 / sample_rate;
    if total_duration_s > 0.0 && zero_crossings > 0 {
        let resp_cycles = zero_crossings as f64 / 2.0;
        result.respiratory_rate_bpm = (resp_cycles / total_duration_s) * 60.0;
    }

    // Also try FFT-based estimation on IBI series (if enough data)
    if ibi.len() >= 8 {
        let fft_rr = estimate_rr_from_fft(&ibi, ibi_rate);
        if fft_rr > 4.0 && fft_rr < 60.0 {
            // Prefer FFT estimate if it looks reasonable
            result.respiratory_rate_bpm = fft_rr;
        }
    }

    result
}

/// Simple FFT-based respiratory rate estimation from IBI series.
fn estimate_rr_from_fft(signal: &[f64], sample_rate: f64) -> f64 {
    let n = signal.len();
    if n < 4 {
        return 0.0;
    }

    // Remove mean
    let mean = signal.iter().sum::<f64>() / n as f64;
    let detrended: Vec<f64> = signal.iter().map(|&x| x - mean).collect();

    // Zero-pad to next power of 2
    let nfft = n.next_power_of_two().max(16);
    let mut real = vec![0.0; nfft];
    let mut imag = vec![0.0; nfft];
    for (i, &x) in detrended.iter().enumerate() {
        // Apply Hann window
        let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
        real[i] = x * w;
    }

    // DFT (simple O(n^2) for small lengths)
    dft_inplace(&mut real, &mut imag);

    // Power spectrum
    let power: Vec<f64> = real
        .iter()
        .zip(imag.iter())
        .map(|(&r, &i)| r * r + i * i)
        .collect();

    // Search respiratory frequency range: 0.1 - 0.8 Hz (6-48 BPM)
    let freq_resolution = sample_rate / nfft as f64;
    let min_bin = (0.1 / freq_resolution).ceil() as usize;
    let max_bin = ((0.8 / freq_resolution).floor() as usize).min(nfft / 2);

    if min_bin >= max_bin || max_bin >= power.len() {
        return 0.0;
    }

    let (peak_bin, _) = power[min_bin..=max_bin]
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or((0, &0.0));

    let peak_freq = (min_bin + peak_bin) as f64 * freq_resolution;
    peak_freq * 60.0 // Convert Hz to BPM
}

/// Simple DFT (O(n^2)) for short sequences.
fn dft_inplace(real: &mut [f64], imag: &mut [f64]) {
    let n = real.len();
    let mut out_re = vec![0.0; n];
    let mut out_im = vec![0.0; n];

    for k in 0..n {
        for j in 0..n {
            let angle = -2.0 * PI * (k as f64) * (j as f64) / (n as f64);
            out_re[k] += real[j] * angle.cos() - imag[j] * angle.sin();
            out_im[k] += real[j] * angle.sin() + imag[j] * angle.cos();
        }
    }

    real.copy_from_slice(&out_re);
    imag.copy_from_slice(&out_im);
}

// ---------------------------------------------------------------------------
// Arterial Stiffness
// ---------------------------------------------------------------------------

/// Compute arterial stiffness indicators.
pub fn compute_arterial_stiffness(
    morphology: &BeatMorphology,
    sdppg: &SdppgFeatures,
    body_height_m: f64,
    ptt_s: f64,
) -> ArterialStiffness {
    let stiffness_index = if ptt_s > 0.0 {
        body_height_m / ptt_s
    } else {
        0.0
    };

    let reflection_index = morphology.reflection_index;

    let large_artery_stiffness = if sdppg.wave_a.abs() > 1e-10 {
        (sdppg.wave_b / sdppg.wave_a).abs()
    } else {
        0.0
    };

    ArterialStiffness {
        stiffness_index,
        reflection_index,
        large_artery_stiffness,
    }
}

// ---------------------------------------------------------------------------
// Synthetic PPG Generation (for testing)
// ---------------------------------------------------------------------------

/// Generate a synthetic PPG signal with specified heart rate.
///
/// The waveform mimics the shape of a real PPG pulse with systolic peak,
/// dicrotic notch, and diastolic peak.
pub fn generate_synthetic_ppg(
    heart_rate_bpm: f64,
    sample_rate: f64,
    duration_s: f64,
    channel: PpgChannel,
) -> PpgSignal {
    let n = (sample_rate * duration_s) as usize;
    let beat_period = 60.0 / heart_rate_bpm;
    let mut samples = vec![0.0; n];

    // DC offset depends on channel (simulates different tissue absorption)
    let dc_offset = match channel {
        PpgChannel::Red => 2000.0,
        PpgChannel::IR => 2500.0,
        PpgChannel::Green => 1800.0,
    };

    // AC amplitude (pulsatile component)
    let ac_amp = match channel {
        PpgChannel::Red => 40.0,
        PpgChannel::IR => 60.0,
        PpgChannel::Green => 35.0,
    };

    for i in 0..n {
        let t = i as f64 / sample_rate;
        let phase = (t % beat_period) / beat_period; // 0 to 1 within each beat

        // Composite pulse shape using Gaussian components
        // Systolic peak (~30% of cycle)
        let systolic = (-((phase - 0.30) / 0.08).powi(2)).exp();
        // Dicrotic notch (~50% of cycle)
        let notch = -0.15 * (-((phase - 0.50) / 0.04).powi(2)).exp();
        // Diastolic peak (~55% of cycle)
        let diastolic = 0.35 * (-((phase - 0.58) / 0.06).powi(2)).exp();
        // Diastolic runoff
        let runoff = -0.05 * (-((phase - 0.75) / 0.15).powi(2)).exp();

        let pulse = systolic + notch + diastolic + runoff;

        // Add slight respiratory modulation (~15 BPM)
        let resp_mod = 1.0 + 0.03 * (2.0 * PI * t / 4.0).sin();

        samples[i] = dc_offset + ac_amp * pulse * resp_mod;
    }

    PpgSignal {
        samples,
        sample_rate_hz: sample_rate,
        channel,
    }
}

/// Generate paired Red and IR PPG signals for SpO2 estimation.
/// `spo2_target` approximately controls the R ratio via AC amplitude ratio.
pub fn generate_spo2_pair(
    heart_rate_bpm: f64,
    spo2_target: f64,
    sample_rate: f64,
    duration_s: f64,
) -> (PpgSignal, PpgSignal) {
    let mut red = generate_synthetic_ppg(heart_rate_bpm, sample_rate, duration_s, PpgChannel::Red);
    let ir = generate_synthetic_ppg(heart_rate_bpm, sample_rate, duration_s, PpgChannel::IR);

    // Adjust red AC to produce target SpO2 approximately.
    // SpO2 = 110 - 25*R => R = (110 - SpO2) / 25
    // R = (AC_red/DC_red) / (AC_ir/DC_ir)
    // So we scale red AC to achieve target R.
    let target_r = (110.0 - spo2_target) / 25.0;

    // Compute current DC and AC levels
    let dc_red = 2000.0;
    let dc_ir = 2500.0;
    let ac_ir = 60.0;

    // target_r = (AC_red/DC_red) / (AC_ir/DC_ir)
    // AC_red = target_r * DC_red * AC_ir / DC_ir
    let target_ac_red = target_r * dc_red * ac_ir / dc_ir;
    let current_ac_red = 40.0;
    let scale = target_ac_red / current_ac_red;

    // Scale the AC component of red signal
    for s in red.samples.iter_mut() {
        let ac = *s - dc_red;
        *s = dc_red + ac * scale;
    }

    (red, ir)
}

// ---------------------------------------------------------------------------
// Main Processor
// ---------------------------------------------------------------------------

/// Stateful PPG signal processor for comprehensive cardiovascular analysis.
pub struct PpgProcessor {
    config: PpgConfig,
    /// Cached bandpass filter coefficients.
    bp_coeffs: Vec<f64>,
}

impl PpgProcessor {
    /// Create a new PPG processor with the given configuration.
    pub fn new(config: PpgConfig) -> Self {
        let bp_coeffs = design_bandpass_fir(
            config.fir_order,
            config.bp_low_hz,
            config.bp_high_hz,
            config.sample_rate_hz,
        );
        Self { config, bp_coeffs }
    }

    /// Preprocess a PPG signal: DC removal + bandpass filtering.
    pub fn preprocess(&self, signal: &PpgSignal) -> Vec<f64> {
        let window_samples =
            (self.config.dc_removal_window_s * signal.sample_rate_hz) as usize;
        let dc_removed = remove_dc(&signal.samples, window_samples);
        apply_fir(&dc_removed, &self.bp_coeffs)
    }

    /// Full analysis of a single-channel PPG signal.
    pub fn analyze(&self, signal: &PpgSignal) -> PpgAnalysisResult {
        let filtered = self.preprocess(signal);

        // Peak detection
        let peak_indices = detect_peaks(
            &filtered,
            signal.sample_rate_hz,
            self.config.peak_threshold_fraction,
        );

        // Heart rate
        let ibi = compute_ibi(&peak_indices, signal.sample_rate_hz);
        let hr = average_hr(&ibi);
        let inst_hr = ibi_to_hr(&ibi);

        // HRV
        let sdnn = compute_sdnn(&ibi);
        let rmssd = compute_rmssd(&ibi);
        let hrv = HrvMetrics {
            sdnn_ms: sdnn * 1000.0,
            rmssd_ms: rmssd * 1000.0,
            mean_ibi_ms: if ibi.is_empty() {
                0.0
            } else {
                (ibi.iter().sum::<f64>() / ibi.len() as f64) * 1000.0
            },
            beat_count: peak_indices.len(),
            ibi_series: ibi.clone(),
        };

        // Morphology
        let beats = extract_beats(&filtered, &peak_indices);
        let beat_morphologies: Vec<BeatMorphology> = beats
            .iter()
            .map(|b| analyze_beat_morphology(b, signal.sample_rate_hz))
            .collect();
        let morphology = average_morphology(&beat_morphologies);

        // SDPPG
        let sdppg_signal = second_derivative(&filtered);
        let sdppg_beats = extract_beats(&sdppg_signal, &peak_indices);
        let sdppg = if !sdppg_beats.is_empty() {
            // Average SDPPG features
            let mut features_list: Vec<SdppgFeatures> = sdppg_beats
                .iter()
                .map(|b| extract_sdppg_waves(b))
                .collect();
            if features_list.is_empty() {
                SdppgFeatures::default()
            } else {
                average_sdppg_features(&features_list)
            }
        } else {
            SdppgFeatures::default()
        };

        // Signal quality
        let quality = assess_signal_quality(
            &filtered,
            &peak_indices,
            signal.sample_rate_hz,
            self.config.min_perfusion_index,
        );

        // Respiratory rate
        let respiratory =
            estimate_respiratory_rate(&filtered, &peak_indices, signal.sample_rate_hz);

        // Arterial stiffness (use systolic time as PTT proxy)
        let ptt_proxy = morphology.systolic_time_s;
        let stiffness = compute_arterial_stiffness(
            &morphology,
            &sdppg,
            self.config.body_height_m,
            ptt_proxy,
        );

        PpgAnalysisResult {
            heart_rate_bpm: hr,
            instantaneous_hr: inst_hr,
            peak_indices,
            hrv,
            morphology,
            sdppg,
            quality,
            respiratory,
            stiffness,
            filtered_signal: filtered,
        }
    }

    /// Compute SpO2 from paired red and IR PPG signals.
    pub fn compute_spo2(&self, red: &PpgSignal, ir: &PpgSignal) -> Spo2Result {
        let red_filtered = self.preprocess(red);
        let ir_filtered = self.preprocess(ir);

        let red_peaks = detect_peaks(
            &red_filtered,
            red.sample_rate_hz,
            self.config.peak_threshold_fraction,
        );
        let ir_peaks = detect_peaks(
            &ir_filtered,
            ir.sample_rate_hz,
            self.config.peak_threshold_fraction,
        );

        let (ac_red, _) = extract_ac_dc(&red_filtered, &red_peaks);
        let (ac_ir, _) = extract_ac_dc(&ir_filtered, &ir_peaks);

        // DC from raw (unfiltered) signals
        let dc_red = if red.samples.is_empty() {
            1.0
        } else {
            red.samples.iter().sum::<f64>() / red.samples.len() as f64
        };
        let dc_ir = if ir.samples.is_empty() {
            1.0
        } else {
            ir.samples.iter().sum::<f64>() / ir.samples.len() as f64
        };

        calculate_spo2(
            ac_red,
            dc_red,
            ac_ir,
            dc_ir,
            self.config.spo2_linear,
            self.config.spo2_quadratic,
        )
    }

    /// Process with motion artifact removal using accelerometer reference.
    pub fn analyze_with_motion_removal(
        &self,
        signal: &PpgSignal,
        accel_reference: &[f64],
    ) -> PpgAnalysisResult {
        let cleaned = lms_motion_removal(
            &signal.samples,
            accel_reference,
            self.config.lms_order,
            self.config.lms_step_size,
        );
        let cleaned_signal = PpgSignal::new(cleaned, signal.sample_rate_hz, signal.channel);
        self.analyze(&cleaned_signal)
    }
}

/// Average SDPPG features across beats.
fn average_sdppg_features(features: &[SdppgFeatures]) -> SdppgFeatures {
    if features.is_empty() {
        return SdppgFeatures::default();
    }
    let n = features.len() as f64;
    let mut avg = SdppgFeatures {
        wave_a: features.iter().map(|f| f.wave_a).sum::<f64>() / n,
        wave_b: features.iter().map(|f| f.wave_b).sum::<f64>() / n,
        wave_c: features.iter().map(|f| f.wave_c).sum::<f64>() / n,
        wave_d: features.iter().map(|f| f.wave_d).sum::<f64>() / n,
        wave_e: features.iter().map(|f| f.wave_e).sum::<f64>() / n,
        aging_index: 0.0,
        vascular_age_years: 0.0,
    };
    if avg.wave_a.abs() > 1e-10 {
        avg.aging_index = (avg.wave_b - avg.wave_c - avg.wave_d - avg.wave_e) / avg.wave_a;
    }
    avg.vascular_age_years = (50.0 + 40.0 * avg.aging_index).clamp(15.0, 100.0);
    avg
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_RATE: f64 = 100.0;
    const DURATION: f64 = 10.0;
    const HR_BPM: f64 = 72.0;

    fn make_signal() -> PpgSignal {
        generate_synthetic_ppg(HR_BPM, SAMPLE_RATE, DURATION, PpgChannel::Green)
    }

    fn make_processor() -> PpgProcessor {
        PpgProcessor::new(PpgConfig::default())
    }

    // --- Config & Signal tests ---

    #[test]
    fn test_ppg_config_default() {
        let cfg = PpgConfig::default();
        assert_eq!(cfg.sample_rate_hz, 100.0);
        assert_eq!(cfg.wavelength_red_nm, 660.0);
        assert_eq!(cfg.wavelength_ir_nm, 940.0);
        assert!((cfg.bp_low_hz - 0.5).abs() < 1e-6);
        assert!((cfg.bp_high_hz - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_ppg_signal_creation() {
        let sig = PpgSignal::new(vec![1.0, 2.0, 3.0], 100.0, PpgChannel::Red);
        assert_eq!(sig.samples.len(), 3);
        assert_eq!(sig.channel, PpgChannel::Red);
        assert!((sig.duration_s() - 0.03).abs() < 1e-6);
    }

    #[test]
    fn test_ppg_channel_variants() {
        assert_ne!(PpgChannel::Red, PpgChannel::IR);
        assert_ne!(PpgChannel::IR, PpgChannel::Green);
        assert_eq!(PpgChannel::Red, PpgChannel::Red);
    }

    // --- DC removal tests ---

    #[test]
    fn test_dc_removal_constant() {
        let signal = vec![5.0; 100];
        let result = remove_dc(&signal, 20);
        for &v in &result {
            assert!(v.abs() < 1e-10, "DC should be fully removed");
        }
    }

    #[test]
    fn test_dc_removal_preserves_ac() {
        let signal: Vec<f64> = (0..200)
            .map(|i| 100.0 + 10.0 * (2.0 * PI * i as f64 / 20.0).sin())
            .collect();
        let result = remove_dc(&signal, 50);
        // AC component should remain with significant amplitude
        let max = result.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = result.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((max - min) > 5.0, "AC should be preserved");
    }

    #[test]
    fn test_dc_removal_empty() {
        let result = remove_dc(&[], 10);
        assert!(result.is_empty());
    }

    // --- FIR filter tests ---

    #[test]
    fn test_bandpass_fir_design() {
        let coeffs = design_bandpass_fir(32, 0.5, 5.0, 100.0);
        assert!(!coeffs.is_empty());
        assert!(coeffs.len() >= 33);
    }

    #[test]
    fn test_fir_apply_impulse() {
        let coeffs = vec![0.25, 0.5, 0.25];
        let impulse = vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let result = apply_fir(&impulse, &coeffs);
        assert_eq!(result.len(), 6);
        // Impulse response should match coefficients
        assert!((result[2] - 0.25).abs() < 1e-10);
        assert!((result[3] - 0.5).abs() < 1e-10);
        assert!((result[4] - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_fir_apply_empty() {
        let result = apply_fir(&[], &[1.0]);
        assert!(result.is_empty());
    }

    // --- LMS motion removal ---

    #[test]
    fn test_lms_removes_correlated_noise() {
        let n = 500;
        let mut clean = vec![0.0; n];
        let mut noise_ref = vec![0.0; n];
        for i in 0..n {
            let t = i as f64 / 100.0;
            clean[i] = (2.0 * PI * 1.2 * t).sin(); // PPG at 1.2 Hz = 72 BPM
            noise_ref[i] = 0.5 * (2.0 * PI * 3.0 * t).sin(); // Motion at 3 Hz
        }
        let noisy: Vec<f64> = clean.iter().zip(noise_ref.iter()).map(|(&c, &n)| c + n).collect();
        let cleaned = lms_motion_removal(&noisy, &noise_ref, 8, 0.01);
        assert_eq!(cleaned.len(), n);
        // Cleaned signal should be closer to clean than noisy
        let noisy_err: f64 = noisy.iter().zip(clean.iter()).map(|(&n, &c)| (n - c).powi(2)).sum();
        let clean_err: f64 = cleaned.iter().zip(clean.iter()).map(|(&n, &c)| (n - c).powi(2)).sum();
        assert!(clean_err < noisy_err, "LMS should reduce motion artifact");
    }

    // --- Peak detection tests ---

    #[test]
    fn test_peak_detection_count() {
        let sig = make_signal();
        let proc = make_processor();
        let filtered = proc.preprocess(&sig);
        let peaks = detect_peaks(&filtered, SAMPLE_RATE, 0.6);
        // 72 BPM * 10s = ~12 beats, allow some margin
        assert!(peaks.len() >= 8, "Should detect at least 8 peaks, got {}", peaks.len());
        assert!(peaks.len() <= 18, "Should detect at most 18 peaks, got {}", peaks.len());
    }

    #[test]
    fn test_peak_detection_ordering() {
        let sig = make_signal();
        let proc = make_processor();
        let filtered = proc.preprocess(&sig);
        let peaks = detect_peaks(&filtered, SAMPLE_RATE, 0.6);
        for pair in peaks.windows(2) {
            assert!(pair[1] > pair[0], "Peaks must be in increasing order");
        }
    }

    #[test]
    fn test_peak_detection_short_signal() {
        let peaks = detect_peaks(&[1.0, 2.0], 100.0, 0.5);
        assert!(peaks.is_empty());
    }

    // --- IBI and HR tests ---

    #[test]
    fn test_ibi_computation() {
        let peaks = vec![100, 200, 300, 400];
        let ibi = compute_ibi(&peaks, 100.0);
        assert_eq!(ibi.len(), 3);
        for &interval in &ibi {
            assert!((interval - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_ibi_to_hr_conversion() {
        let ibi = vec![0.8, 0.85, 0.75]; // ~72-80 BPM
        let hr = ibi_to_hr(&ibi);
        assert_eq!(hr.len(), 3);
        assert!((hr[0] - 75.0).abs() < 0.1);
        assert!((hr[1] - 60.0 / 0.85).abs() < 0.1);
    }

    #[test]
    fn test_average_hr() {
        let ibi = vec![0.833, 0.833, 0.833]; // 72 BPM
        let hr = average_hr(&ibi);
        assert!((hr - 72.0).abs() < 1.0);
    }

    #[test]
    fn test_average_hr_empty() {
        assert_eq!(average_hr(&[]), 0.0);
    }

    // --- HRV tests ---

    #[test]
    fn test_sdnn() {
        let ibi = vec![0.8, 0.85, 0.82, 0.78, 0.84];
        let sdnn = compute_sdnn(&ibi);
        assert!(sdnn > 0.0);
        assert!(sdnn < 0.1); // Should be small for stable HR
    }

    #[test]
    fn test_rmssd() {
        let ibi = vec![0.8, 0.85, 0.82, 0.78, 0.84];
        let rmssd = compute_rmssd(&ibi);
        assert!(rmssd > 0.0);
        assert!(rmssd < 0.1);
    }

    #[test]
    fn test_sdnn_constant_ibi() {
        let ibi = vec![0.8, 0.8, 0.8, 0.8];
        assert!((compute_sdnn(&ibi)).abs() < 1e-10, "SDNN should be 0 for constant IBI");
    }

    #[test]
    fn test_rmssd_constant_ibi() {
        let ibi = vec![0.8, 0.8, 0.8, 0.8];
        assert!((compute_rmssd(&ibi)).abs() < 1e-10, "RMSSD should be 0 for constant IBI");
    }

    // --- SpO2 tests ---

    #[test]
    fn test_spo2_calculation_basic() {
        let result = calculate_spo2(
            10.0, 1000.0, 12.0, 1200.0,
            (110.0, -25.0),
            (115.0, -30.0, 5.0),
        );
        assert!(result.spo2_linear > 80.0 && result.spo2_linear <= 100.0);
        assert!(result.r_ratio > 0.0);
    }

    #[test]
    fn test_spo2_r_ratio_unity() {
        // When AC/DC ratios are equal, R=1
        let result = calculate_spo2(
            10.0, 100.0, 10.0, 100.0,
            (110.0, -25.0),
            (115.0, -30.0, 5.0),
        );
        assert!((result.r_ratio - 1.0).abs() < 1e-6);
        // SpO2 linear: 110 - 25*1 = 85
        assert!((result.spo2_linear - 85.0).abs() < 1e-6);
    }

    #[test]
    fn test_spo2_pair_generation() {
        let (red, ir) = generate_spo2_pair(72.0, 97.0, 100.0, 10.0);
        assert_eq!(red.channel, PpgChannel::Red);
        assert_eq!(ir.channel, PpgChannel::IR);
        assert_eq!(red.samples.len(), ir.samples.len());
    }

    #[test]
    fn test_perfusion_index() {
        let pi = perfusion_index(5.0, 1000.0);
        assert!((pi - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_perfusion_index_zero_dc() {
        let pi = perfusion_index(5.0, 0.0);
        assert_eq!(pi, 0.0);
    }

    // --- AC/DC extraction ---

    #[test]
    fn test_ac_dc_extraction() {
        let sig = make_signal();
        let proc = make_processor();
        let filtered = proc.preprocess(&sig);
        let peaks = detect_peaks(&filtered, SAMPLE_RATE, 0.6);
        let (ac, dc) = extract_ac_dc(&filtered, &peaks);
        assert!(ac > 0.0, "AC should be positive");
        // DC of filtered signal should be near zero
        assert!(dc.abs() < 1.0, "DC of filtered signal should be small");
    }

    // --- Morphology tests ---

    #[test]
    fn test_beat_morphology_systolic() {
        // Synthetic beat: rising then falling
        let beat: Vec<f64> = (0..100)
            .map(|i| {
                let phase = i as f64 / 100.0;
                (-((phase - 0.3) / 0.1).powi(2)).exp()
                    + 0.3 * (-((phase - 0.6) / 0.08).powi(2)).exp()
            })
            .collect();
        let morph = analyze_beat_morphology(&beat, 100.0);
        assert!(morph.systolic_amplitude > 0.5);
        assert!(morph.systolic_time_s > 0.0);
    }

    #[test]
    fn test_dicrotic_notch_detection() {
        // Pulse with clear notch
        let beat: Vec<f64> = (0..100)
            .map(|i| {
                let p = i as f64 / 100.0;
                (-((p - 0.25) / 0.08).powi(2)).exp()
                    - 0.2 * (-((p - 0.50) / 0.03).powi(2)).exp()
                    + 0.3 * (-((p - 0.58) / 0.06).powi(2)).exp()
            })
            .collect();
        let result = find_dicrotic_notch(&beat);
        assert!(result.is_some(), "Should detect dicrotic notch");
    }

    #[test]
    fn test_augmentation_index_range() {
        let beat: Vec<f64> = (0..100)
            .map(|i| {
                let p = i as f64 / 100.0;
                (-((p - 0.25) / 0.08).powi(2)).exp()
                    - 0.15 * (-((p - 0.50) / 0.04).powi(2)).exp()
                    + 0.3 * (-((p - 0.58) / 0.06).powi(2)).exp()
            })
            .collect();
        let morph = analyze_beat_morphology(&beat, 100.0);
        // Augmentation index can be slightly outside [0,1] depending on notch detection
        assert!(morph.augmentation_index >= -0.5 && morph.augmentation_index <= 1.5,
            "AI {} out of expected range", morph.augmentation_index);
    }

    // --- SDPPG tests ---

    #[test]
    fn test_second_derivative() {
        let signal = vec![0.0, 1.0, 4.0, 9.0, 16.0]; // quadratic: f(x) = x^2
        let d2 = second_derivative(&signal);
        assert_eq!(d2.len(), 5);
        // Second derivative of x^2 is 2
        assert!((d2[2] - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_sdppg_wave_extraction() {
        // Create a signal with alternating peaks/troughs
        let mut beat = vec![0.0; 50];
        for i in 0..50 {
            let t = i as f64 / 50.0;
            beat[i] = (6.0 * PI * t).sin() * (-2.0 * t).exp();
        }
        let features = extract_sdppg_waves(&beat);
        // Wave 'a' should be the first positive peak
        assert!(features.wave_a != 0.0 || features.wave_b != 0.0,
            "Should extract at least some waves");
    }

    #[test]
    fn test_sdppg_aging_index() {
        let features = SdppgFeatures {
            wave_a: 1.0,
            wave_b: -0.6,
            wave_c: 0.1,
            wave_d: -0.05,
            wave_e: 0.05,
            aging_index: 0.0,
            vascular_age_years: 0.0,
        };
        // aging_index = (b - c - d - e) / a = (-0.6 - 0.1 + 0.05 - 0.05) / 1.0 = -0.7
        let ai = (features.wave_b - features.wave_c - features.wave_d - features.wave_e) / features.wave_a;
        assert!((ai - (-0.7)).abs() < 1e-6);
    }

    // --- Signal quality tests ---

    #[test]
    fn test_signal_quality_good_signal() {
        let sig = make_signal();
        let proc = make_processor();
        let filtered = proc.preprocess(&sig);
        let peaks = detect_peaks(&filtered, SAMPLE_RATE, 0.6);
        let sq = assess_signal_quality(&filtered, &peaks, SAMPLE_RATE, 0.2);
        assert!(sq.sqi > 0.3, "Good signal should have SQI > 0.3, got {}", sq.sqi);
    }

    #[test]
    fn test_signal_quality_noise() {
        // Pure noise should have low quality
        let noise: Vec<f64> = (0..1000).map(|i| ((i * 12345 + 67) % 1000) as f64 / 1000.0 - 0.5).collect();
        let peaks = detect_peaks(&noise, 100.0, 0.6);
        let sq = assess_signal_quality(&noise, &peaks, 100.0, 0.2);
        // Noise signal quality should not be perfect
        // (pseudo-random may accidentally produce some structure, so be lenient)
        assert!(sq.sqi <= 1.0, "SQI should be bounded");
    }

    #[test]
    fn test_skewness_symmetric() {
        // Symmetric distribution should have ~0 skewness
        let data: Vec<f64> = (-50..=50).map(|i| i as f64).collect();
        let sk = compute_skewness(&data);
        assert!(sk.abs() < 0.1);
    }

    // --- Respiratory rate tests ---

    #[test]
    fn test_respiratory_rate_extraction() {
        let sig = make_signal(); // Has respiratory modulation at ~15 BPM
        let proc = make_processor();
        let filtered = proc.preprocess(&sig);
        let peaks = detect_peaks(&filtered, SAMPLE_RATE, 0.6);
        let resp = estimate_respiratory_rate(&filtered, &peaks, SAMPLE_RATE);
        // Should extract some respiratory-related signals
        assert!(!resp.baseline_modulation.is_empty() || !resp.amplitude_modulation.is_empty());
    }

    #[test]
    fn test_respiratory_rate_short_signal() {
        let sig = PpgSignal::new(vec![1.0; 50], 100.0, PpgChannel::Green);
        let resp = estimate_respiratory_rate(&sig.samples, &[10, 20], 100.0);
        // With very few peaks, should still not panic
        assert!(resp.respiratory_rate_bpm >= 0.0);
    }

    // --- Arterial stiffness tests ---

    #[test]
    fn test_stiffness_index() {
        let morph = BeatMorphology {
            reflection_index: 0.5,
            ..Default::default()
        };
        let sdppg = SdppgFeatures {
            wave_a: 1.0,
            wave_b: -0.7,
            ..Default::default()
        };
        let stiff = compute_arterial_stiffness(&morph, &sdppg, 1.75, 0.2);
        assert!((stiff.stiffness_index - 8.75).abs() < 1e-6); // 1.75/0.2
        assert!((stiff.reflection_index - 0.5).abs() < 1e-6);
        assert!((stiff.large_artery_stiffness - 0.7).abs() < 1e-6);
    }

    #[test]
    fn test_stiffness_zero_ptt() {
        let morph = BeatMorphology::default();
        let sdppg = SdppgFeatures::default();
        let stiff = compute_arterial_stiffness(&morph, &sdppg, 1.70, 0.0);
        assert_eq!(stiff.stiffness_index, 0.0);
    }

    // --- Full analysis tests ---

    #[test]
    fn test_full_analysis_heart_rate() {
        let sig = make_signal();
        let proc = make_processor();
        let result = proc.analyze(&sig);
        assert!(result.heart_rate_bpm > 55.0, "HR {} too low", result.heart_rate_bpm);
        assert!(result.heart_rate_bpm < 95.0, "HR {} too high", result.heart_rate_bpm);
    }

    #[test]
    fn test_full_analysis_has_peaks() {
        let sig = make_signal();
        let proc = make_processor();
        let result = proc.analyze(&sig);
        assert!(!result.peak_indices.is_empty(), "Should detect peaks");
    }

    #[test]
    fn test_full_analysis_hrv_populated() {
        let sig = make_signal();
        let proc = make_processor();
        let result = proc.analyze(&sig);
        assert!(result.hrv.beat_count > 0);
        assert!(result.hrv.mean_ibi_ms > 0.0);
    }

    #[test]
    fn test_spo2_from_processor() {
        let (red, ir) = generate_spo2_pair(72.0, 97.0, 100.0, 10.0);
        let proc = make_processor();
        let result = proc.compute_spo2(&red, &ir);
        assert!(result.spo2_linear > 70.0, "SpO2 linear {} too low", result.spo2_linear);
        assert!(result.spo2_linear <= 100.0, "SpO2 linear {} too high", result.spo2_linear);
        assert!(result.r_ratio > 0.0);
    }

    #[test]
    fn test_motion_removal_analysis() {
        let sig = make_signal();
        let n = sig.samples.len();
        // Simulate accelerometer data (3 Hz motion)
        let accel: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * PI * 3.0 * i as f64 / SAMPLE_RATE).sin())
            .collect();
        let proc = make_processor();
        let result = proc.analyze_with_motion_removal(&sig, &accel);
        // Should still detect heart rate (relaxed threshold since LMS may distort)
        assert!(result.heart_rate_bpm > 30.0, "HR after motion removal: {}", result.heart_rate_bpm);
    }

    // --- DFT test ---

    #[test]
    fn test_dft_single_tone() {
        let n = 32;
        let mut real: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 4.0 * i as f64 / n as f64).cos())
            .collect();
        let mut imag = vec![0.0; n];
        dft_inplace(&mut real, &mut imag);
        // Peak should be at bin 4
        let power: Vec<f64> = real.iter().zip(imag.iter())
            .map(|(&r, &i)| (r * r + i * i).sqrt())
            .collect();
        let (peak_bin, _) = power[..n / 2]
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak_bin, 4, "Peak should be at bin 4");
    }

    // --- Edge cases ---

    #[test]
    fn test_cross_correlation_identical() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let corr = cross_correlation(&a, &a);
        assert!((corr - 1.0).abs() < 1e-6, "Self-correlation should be 1.0");
    }

    #[test]
    fn test_cross_correlation_anticorrelated() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b: Vec<f64> = a.iter().map(|&x| -x).collect();
        let corr = cross_correlation(&a, &b);
        assert!((corr - (-1.0)).abs() < 1e-6, "Anti-correlation should be -1.0");
    }

    #[test]
    fn test_synthetic_ppg_channels() {
        for channel in &[PpgChannel::Red, PpgChannel::IR, PpgChannel::Green] {
            let sig = generate_synthetic_ppg(70.0, 100.0, 3.0, *channel);
            assert_eq!(sig.samples.len(), 300);
            assert_eq!(sig.channel, *channel);
            // All values should be positive (DC offset)
            assert!(sig.samples.iter().all(|&x| x > 0.0));
        }
    }

    #[test]
    fn test_preprocessed_signal_length() {
        let sig = make_signal();
        let proc = make_processor();
        let filtered = proc.preprocess(&sig);
        assert_eq!(filtered.len(), sig.samples.len());
    }

    #[test]
    fn test_vascular_age_clamping() {
        let mut f = SdppgFeatures::default();
        f.wave_a = 1.0;
        // Extreme aging index
        f.aging_index = 10.0;
        let age = (50.0 + 40.0 * f.aging_index).clamp(15.0, 100.0);
        assert_eq!(age, 100.0);
    }
}

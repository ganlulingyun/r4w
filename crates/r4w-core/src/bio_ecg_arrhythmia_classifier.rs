//! Cardiac arrhythmia detection and classification from ECG signals.
//!
//! This module provides tools for analyzing ECG (electrocardiogram) signals
//! beyond basic QRS detection. It focuses on:
//!
//! - **R-R interval extraction** from detected R-peak positions
//! - **Heart Rate Variability (HRV)** analysis: SDNN, RMSSD, pNN50
//! - **Beat classification** via template matching: Normal, PVC, PAC, BBB
//! - **Rhythm classification**: Normal Sinus, Bradycardia, Tachycardia,
//!   Atrial Fibrillation, Ventricular Tachycardia
//! - **ST-segment deviation** analysis for ischemia detection
//! - **Atrial fibrillation detection** with irregularity scoring
//!
//! # Example
//!
//! ```
//! use r4w_core::bio_ecg_arrhythmia_classifier::{ArrhythmiaClassifier, RhythmType};
//!
//! let classifier = ArrhythmiaClassifier::new(360.0, 20);
//!
//! // Suppose we have R-peak sample indices from a QRS detector
//! let r_peaks: Vec<usize> = (0..10).map(|i| i * 360).collect(); // 1 second apart => 60 bpm
//!
//! let rr = classifier.extract_rr_intervals(&r_peaks);
//! assert_eq!(rr.len(), 9);
//!
//! let hrv = classifier.compute_hrv(&rr);
//! assert!((hrv.mean_rr - 1.0).abs() < 1e-9);
//!
//! let rhythm = classifier.classify_rhythm(&rr);
//! assert_eq!(rhythm, RhythmType::NormalSinus);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Classification of a single heartbeat morphology.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BeatType {
    /// Normal sinus beat.
    Normal,
    /// Premature ventricular contraction.
    Pvc,
    /// Premature atrial contraction.
    Pac,
    /// Bundle branch block.
    Bbb,
    /// Unrecognized morphology.
    Unknown,
}

/// Classification of the overall cardiac rhythm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RhythmType {
    /// Normal sinus rhythm (60-100 bpm, regular).
    NormalSinus,
    /// Heart rate below 60 bpm.
    Bradycardia,
    /// Heart rate above 100 bpm.
    Tachycardia,
    /// Irregularly irregular rhythm (atrial fibrillation).
    AtrialFibrillation,
    /// Rapid ventricular rhythm (>= 3 consecutive PVCs at high rate).
    VentricularTachycardia,
    /// Cannot determine rhythm.
    Unknown,
}

// ---------------------------------------------------------------------------
// HRV metrics
// ---------------------------------------------------------------------------

/// Heart rate variability metrics computed from R-R intervals.
#[derive(Debug, Clone, PartialEq)]
pub struct HrvMetrics {
    /// Standard deviation of normal-to-normal R-R intervals (seconds).
    pub sdnn: f64,
    /// Root mean square of successive R-R differences (seconds).
    pub rmssd: f64,
    /// Percentage of successive R-R differences exceeding 50 ms.
    pub pnn50: f64,
    /// Mean R-R interval (seconds).
    pub mean_rr: f64,
    /// Minimum R-R interval (seconds).
    pub min_rr: f64,
    /// Maximum R-R interval (seconds).
    pub max_rr: f64,
}

impl Default for HrvMetrics {
    fn default() -> Self {
        Self {
            sdnn: 0.0,
            rmssd: 0.0,
            pnn50: 0.0,
            mean_rr: 0.0,
            min_rr: 0.0,
            max_rr: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Classifier
// ---------------------------------------------------------------------------

/// Main arrhythmia classifier.
///
/// Holds configuration parameters that are shared across the various analysis
/// functions.
#[derive(Debug, Clone)]
pub struct ArrhythmiaClassifier {
    /// ECG sample rate in Hz (e.g. 360.0 for MIT-BIH).
    pub sample_rate_hz: f64,
    /// Number of R-R intervals to consider for windowed analysis.
    pub rr_window_size: usize,
}

impl ArrhythmiaClassifier {
    /// Create a new classifier.
    ///
    /// # Arguments
    /// * `sample_rate_hz` - sample rate of the ECG signal in Hz.
    /// * `rr_window_size` - number of R-R intervals for windowed analysis.
    pub fn new(sample_rate_hz: f64, rr_window_size: usize) -> Self {
        Self {
            sample_rate_hz,
            rr_window_size,
        }
    }

    /// Convert R-peak sample indices to R-R intervals in seconds.
    ///
    /// Returns a vector of length `r_peaks_samples.len() - 1`.  Each element
    /// is the time difference between successive R-peaks.
    ///
    /// Does not panic; returns an empty vector if fewer than 2 peaks are given.
    pub fn extract_rr_intervals(&self, r_peaks_samples: &[usize]) -> Vec<f64> {
        extract_rr_intervals(r_peaks_samples, self.sample_rate_hz)
    }

    /// Compute HRV metrics from R-R intervals (in seconds).
    pub fn compute_hrv(&self, rr_intervals: &[f64]) -> HrvMetrics {
        compute_hrv(rr_intervals)
    }

    /// Classify a single beat by correlating it against a normal template.
    ///
    /// Returns `(BeatType, correlation)` where `correlation` is in \[-1, 1\].
    pub fn classify_beat(&self, template: &[f64], beat: &[f64]) -> (BeatType, f64) {
        classify_beat(template, beat)
    }

    /// Classify the overall cardiac rhythm from R-R intervals.
    pub fn classify_rhythm(&self, rr_intervals: &[f64]) -> RhythmType {
        classify_rhythm(rr_intervals)
    }

    /// Measure ST-segment deviation for each beat.
    ///
    /// For each R-peak the ST segment is taken as the region
    /// `[r_peak + offset .. r_peak + offset + window]` where
    /// `offset = 0.08 * sample_rate` and `window = 0.04 * sample_rate`.
    /// The deviation is the mean amplitude minus `baseline`.
    pub fn detect_st_deviation(
        &self,
        ecg: &[f64],
        r_peaks: &[usize],
        baseline: f64,
    ) -> Vec<f64> {
        detect_st_deviation(ecg, r_peaks, baseline, self.sample_rate_hz)
    }

    /// Detect atrial fibrillation from R-R interval irregularity.
    ///
    /// Returns `(is_afib, irregularity_score)`.  The irregularity score is the
    /// coefficient of variation of successive R-R differences.
    pub fn detect_afib(&self, rr_intervals: &[f64]) -> (bool, f64) {
        detect_afib(rr_intervals)
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Convert R-peak sample indices to R-R intervals in seconds.
///
/// Each output element is `(r_peaks[i+1] - r_peaks[i]) / sample_rate_hz`.
pub fn extract_rr_intervals(r_peaks_samples: &[usize], sample_rate_hz: f64) -> Vec<f64> {
    if r_peaks_samples.len() < 2 {
        return Vec::new();
    }
    r_peaks_samples
        .windows(2)
        .map(|w| (w[1] as f64 - w[0] as f64) / sample_rate_hz)
        .collect()
}

/// Compute HRV metrics from R-R intervals given in seconds.
///
/// Returns default (all-zero) metrics if `rr_intervals` is empty.
pub fn compute_hrv(rr_intervals: &[f64]) -> HrvMetrics {
    if rr_intervals.is_empty() {
        return HrvMetrics::default();
    }

    let n = rr_intervals.len() as f64;
    let mean_rr = rr_intervals.iter().sum::<f64>() / n;

    let min_rr = rr_intervals
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    let max_rr = rr_intervals
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);

    // SDNN - standard deviation of all RR intervals
    let variance = rr_intervals
        .iter()
        .map(|&rr| (rr - mean_rr).powi(2))
        .sum::<f64>()
        / n;
    let sdnn = variance.sqrt();

    // Successive differences
    let diffs: Vec<f64> = rr_intervals
        .windows(2)
        .map(|w| w[1] - w[0])
        .collect();

    // RMSSD
    let rmssd = if diffs.is_empty() {
        0.0
    } else {
        let sum_sq: f64 = diffs.iter().map(|d| d.powi(2)).sum();
        (sum_sq / diffs.len() as f64).sqrt()
    };

    // pNN50 - percentage of successive differences > 50 ms
    let pnn50 = if diffs.is_empty() {
        0.0
    } else {
        let count = diffs.iter().filter(|d| d.abs() > 0.05).count();
        (count as f64 / diffs.len() as f64) * 100.0
    };

    HrvMetrics {
        sdnn,
        rmssd,
        pnn50,
        mean_rr,
        min_rr,
        max_rr,
    }
}

/// Classify a single beat by cross-correlating against a normal-beat template.
///
/// The correlation coefficient is computed using the Pearson formula.  Beat
/// classification thresholds:
///
/// | Correlation | Classification |
/// |-------------|----------------|
/// | >= 0.90     | Normal         |
/// | >= 0.70     | PAC            |
/// | >= 0.50     | BBB            |
/// | >= 0.30     | PVC            |
/// | < 0.30      | Unknown        |
///
/// If either `template` or `beat` is empty, returns `(Unknown, 0.0)`.
pub fn classify_beat(template: &[f64], beat: &[f64]) -> (BeatType, f64) {
    if template.is_empty() || beat.is_empty() {
        return (BeatType::Unknown, 0.0);
    }

    let corr = pearson_correlation(template, beat);

    let beat_type = if corr >= 0.90 {
        BeatType::Normal
    } else if corr >= 0.70 {
        BeatType::Pac
    } else if corr >= 0.50 {
        BeatType::Bbb
    } else if corr >= 0.30 {
        BeatType::Pvc
    } else {
        BeatType::Unknown
    };

    (beat_type, corr)
}

/// Classify the cardiac rhythm from R-R intervals (in seconds).
///
/// Decision logic:
/// 1. If atrial fibrillation is detected -> `AtrialFibrillation`
/// 2. Mean heart rate > 150 bpm -> `VentricularTachycardia`
/// 3. Mean heart rate < 60 bpm -> `Bradycardia`
/// 4. Mean heart rate > 100 bpm -> `Tachycardia`
/// 5. Otherwise -> `NormalSinus`
///
/// Returns `Unknown` if fewer than 3 intervals are provided.
pub fn classify_rhythm(rr_intervals: &[f64]) -> RhythmType {
    if rr_intervals.len() < 3 {
        return RhythmType::Unknown;
    }

    let (is_afib, _score) = detect_afib(rr_intervals);
    if is_afib {
        return RhythmType::AtrialFibrillation;
    }

    let mean_rr = rr_intervals.iter().sum::<f64>() / rr_intervals.len() as f64;
    let mean_hr = 60.0 / mean_rr; // beats per minute

    // Check for ventricular tachycardia: very fast and somewhat irregular
    if mean_hr > 150.0 {
        return RhythmType::VentricularTachycardia;
    }

    if mean_hr < 60.0 {
        RhythmType::Bradycardia
    } else if mean_hr > 100.0 {
        RhythmType::Tachycardia
    } else {
        RhythmType::NormalSinus
    }
}

/// Measure ST-segment deviation relative to a baseline for each R-peak.
///
/// The ST segment is sampled starting 80 ms after each R-peak for a 40 ms
/// window.  The deviation is `mean(ST segment) - baseline`.
///
/// Beats whose ST window extends beyond the signal are excluded.
pub fn detect_st_deviation(
    ecg: &[f64],
    r_peaks: &[usize],
    baseline: f64,
    sample_rate_hz: f64,
) -> Vec<f64> {
    let offset = (0.08 * sample_rate_hz) as usize; // 80 ms after R-peak
    let window = (0.04 * sample_rate_hz).max(1.0) as usize; // 40 ms window

    r_peaks
        .iter()
        .filter_map(|&rp| {
            let start = rp + offset;
            let end = start + window;
            if end <= ecg.len() {
                let mean_st: f64 =
                    ecg[start..end].iter().sum::<f64>() / window as f64;
                Some(mean_st - baseline)
            } else {
                None
            }
        })
        .collect()
}

/// Detect atrial fibrillation from R-R interval irregularity.
///
/// Uses the coefficient of variation (CV) of successive R-R differences.
/// A CV above `0.10` (10 %) is classified as atrial fibrillation.
///
/// Returns `(is_afib, irregularity_score)` where the score is the CV.
/// Returns `(false, 0.0)` if fewer than 3 intervals are given.
pub fn detect_afib(rr_intervals: &[f64]) -> (bool, f64) {
    if rr_intervals.len() < 3 {
        return (false, 0.0);
    }

    let diffs: Vec<f64> = rr_intervals
        .windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .collect();

    let mean_diff = diffs.iter().sum::<f64>() / diffs.len() as f64;
    let mean_rr = rr_intervals.iter().sum::<f64>() / rr_intervals.len() as f64;

    if mean_rr.abs() < 1e-12 {
        return (false, 0.0);
    }

    let irregularity = mean_diff / mean_rr;

    let threshold = 0.10;
    (irregularity > threshold, irregularity)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Pearson correlation coefficient between two slices.
///
/// If the slices are different lengths the shorter length is used.
/// Returns 0.0 for constant or zero-length inputs.
fn pearson_correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }

    let mean_a = a[..n].iter().sum::<f64>() / n as f64;
    let mean_b = b[..n].iter().sum::<f64>() / n as f64;

    let mut cov = 0.0;
    let mut var_a = 0.0;
    let mut var_b = 0.0;

    for i in 0..n {
        let da = a[i] - mean_a;
        let db = b[i] - mean_b;
        cov += da * db;
        var_a += da * da;
        var_b += db * db;
    }

    let denom = (var_a * var_b).sqrt();
    if denom < 1e-15 {
        0.0
    } else {
        cov / denom
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // extract_rr_intervals
    // -----------------------------------------------------------------------

    #[test]
    fn test_extract_rr_intervals_basic() {
        let peaks = vec![0, 360, 720, 1080];
        let rr = extract_rr_intervals(&peaks, 360.0);
        assert_eq!(rr.len(), 3);
        for &interval in &rr {
            assert!((interval - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn test_extract_rr_intervals_empty() {
        let rr = extract_rr_intervals(&[], 360.0);
        assert!(rr.is_empty());
    }

    #[test]
    fn test_extract_rr_intervals_single_peak() {
        let rr = extract_rr_intervals(&[100], 360.0);
        assert!(rr.is_empty());
    }

    #[test]
    fn test_extract_rr_intervals_uneven() {
        let peaks = vec![0, 180, 540]; // 0.5s, 1.0s
        let rr = extract_rr_intervals(&peaks, 360.0);
        assert_eq!(rr.len(), 2);
        assert!((rr[0] - 0.5).abs() < 1e-9);
        assert!((rr[1] - 1.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // compute_hrv
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_hrv_constant_rr() {
        let rr = vec![1.0; 10];
        let hrv = compute_hrv(&rr);
        assert!((hrv.mean_rr - 1.0).abs() < 1e-9);
        assert!(hrv.sdnn.abs() < 1e-9);
        assert!(hrv.rmssd.abs() < 1e-9);
        assert!((hrv.pnn50).abs() < 1e-9);
        assert!((hrv.min_rr - 1.0).abs() < 1e-9);
        assert!((hrv.max_rr - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_compute_hrv_empty() {
        let hrv = compute_hrv(&[]);
        assert!((hrv.mean_rr).abs() < 1e-9);
        assert!((hrv.sdnn).abs() < 1e-9);
    }

    #[test]
    fn test_compute_hrv_varying_rr() {
        let rr = vec![0.8, 1.0, 0.9, 1.1];
        let hrv = compute_hrv(&rr);
        let expected_mean = (0.8 + 1.0 + 0.9 + 1.1) / 4.0;
        assert!((hrv.mean_rr - expected_mean).abs() < 1e-9);
        assert!(hrv.sdnn > 0.0);
        assert!(hrv.rmssd > 0.0);
        assert!((hrv.min_rr - 0.8).abs() < 1e-9);
        assert!((hrv.max_rr - 1.1).abs() < 1e-9);
    }

    #[test]
    fn test_compute_hrv_pnn50_all_above() {
        // Successive diffs all > 50 ms
        let rr = vec![0.8, 0.9, 1.0, 1.1, 1.2];
        let hrv = compute_hrv(&rr);
        // Diffs: 0.1, 0.1, 0.1, 0.1 - all > 0.05
        assert!((hrv.pnn50 - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_compute_hrv_pnn50_none_above() {
        // Successive diffs all < 50 ms
        let rr = vec![1.0, 1.01, 1.02, 1.03];
        let hrv = compute_hrv(&rr);
        // Diffs: 0.01, 0.01, 0.01 - none > 0.05
        assert!((hrv.pnn50).abs() < 1e-9);
    }

    #[test]
    fn test_compute_hrv_single_interval() {
        let rr = vec![0.85];
        let hrv = compute_hrv(&rr);
        assert!((hrv.mean_rr - 0.85).abs() < 1e-9);
        assert!(hrv.sdnn.abs() < 1e-9); // No variance with one value
        assert!(hrv.rmssd.abs() < 1e-9); // No successive diffs
    }

    // -----------------------------------------------------------------------
    // classify_beat
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_beat_identical() {
        let template = vec![0.0, 0.5, 1.0, 0.5, 0.0];
        let beat = template.clone();
        let (bt, corr) = classify_beat(&template, &beat);
        assert_eq!(bt, BeatType::Normal);
        assert!((corr - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_classify_beat_inverted() {
        let template = vec![0.0, 0.5, 1.0, 0.5, 0.0];
        let beat: Vec<f64> = template.iter().map(|x| -x).collect();
        let (bt, corr) = classify_beat(&template, &beat);
        assert!((corr + 1.0).abs() < 1e-9);
        assert_eq!(bt, BeatType::Unknown);
    }

    #[test]
    fn test_classify_beat_empty() {
        let (bt, corr) = classify_beat(&[], &[1.0, 2.0]);
        assert_eq!(bt, BeatType::Unknown);
        assert!((corr).abs() < 1e-9);
    }

    #[test]
    fn test_classify_beat_scaled() {
        // Scaled version should still be Normal (correlation = 1.0)
        let template = vec![0.0, 1.0, 2.0, 1.0, 0.0];
        let beat: Vec<f64> = template.iter().map(|x| x * 3.0).collect();
        let (bt, corr) = classify_beat(&template, &beat);
        assert_eq!(bt, BeatType::Normal);
        assert!((corr - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_classify_beat_noisy() {
        let template: Vec<f64> = (0..50)
            .map(|i| (2.0 * PI * i as f64 / 50.0).sin())
            .collect();
        // Create a partially corrupted beat
        let beat: Vec<f64> = template
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                if i % 3 == 0 {
                    x * 0.5 + 0.3
                } else {
                    x
                }
            })
            .collect();
        let (_bt, corr) = classify_beat(&template, &beat);
        // Just verify we get a meaningful correlation
        assert!(corr > 0.0);
        assert!(corr < 1.0);
    }

    // -----------------------------------------------------------------------
    // classify_rhythm
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_rhythm_normal_sinus() {
        // 75 bpm -> RR = 0.8 s, steady
        let rr = vec![0.8; 20];
        assert_eq!(classify_rhythm(&rr), RhythmType::NormalSinus);
    }

    #[test]
    fn test_classify_rhythm_bradycardia() {
        // 50 bpm -> RR = 1.2 s
        let rr = vec![1.2; 20];
        assert_eq!(classify_rhythm(&rr), RhythmType::Bradycardia);
    }

    #[test]
    fn test_classify_rhythm_tachycardia() {
        // 120 bpm -> RR = 0.5 s
        let rr = vec![0.5; 20];
        assert_eq!(classify_rhythm(&rr), RhythmType::Tachycardia);
    }

    #[test]
    fn test_classify_rhythm_too_few_intervals() {
        let rr = vec![0.8, 0.8];
        assert_eq!(classify_rhythm(&rr), RhythmType::Unknown);
    }

    #[test]
    fn test_classify_rhythm_vtach() {
        // 180 bpm -> RR = 0.333 s, regular
        let rr = vec![0.333; 20];
        assert_eq!(classify_rhythm(&rr), RhythmType::VentricularTachycardia);
    }

    // -----------------------------------------------------------------------
    // detect_st_deviation
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_st_deviation_flat_signal() {
        let ecg = vec![0.5; 1000];
        let r_peaks = vec![100, 300, 500];
        let devs = detect_st_deviation(&ecg, &r_peaks, 0.5, 360.0);
        assert_eq!(devs.len(), 3);
        for d in &devs {
            assert!(d.abs() < 1e-9);
        }
    }

    #[test]
    fn test_detect_st_deviation_elevated() {
        let ecg = vec![1.0; 1000];
        let r_peaks = vec![100, 300, 500];
        let devs = detect_st_deviation(&ecg, &r_peaks, 0.0, 360.0);
        for d in &devs {
            assert!((d - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn test_detect_st_deviation_out_of_bounds() {
        let ecg = vec![0.5; 100];
        // R-peak near the end - ST window extends past signal
        let r_peaks = vec![10, 95];
        let devs = detect_st_deviation(&ecg, &r_peaks, 0.5, 360.0);
        // Only the first peak should produce a result
        assert_eq!(devs.len(), 1);
    }

    // -----------------------------------------------------------------------
    // detect_afib
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_afib_regular() {
        let rr = vec![0.8; 20];
        let (is_afib, score) = detect_afib(&rr);
        assert!(!is_afib);
        assert!(score.abs() < 1e-9);
    }

    #[test]
    fn test_detect_afib_irregular() {
        // Highly irregular intervals -> should detect AFib
        let rr = vec![0.5, 0.9, 0.4, 1.1, 0.3, 1.0, 0.6, 0.8, 0.35, 1.2];
        let (is_afib, score) = detect_afib(&rr);
        assert!(is_afib);
        assert!(score > 0.10);
    }

    #[test]
    fn test_detect_afib_too_few() {
        let (is_afib, score) = detect_afib(&[0.8, 0.9]);
        assert!(!is_afib);
        assert!(score.abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // ArrhythmiaClassifier (method wrappers)
    // -----------------------------------------------------------------------

    #[test]
    fn test_classifier_roundtrip() {
        let clf = ArrhythmiaClassifier::new(500.0, 30);
        assert!((clf.sample_rate_hz - 500.0).abs() < 1e-9);
        assert_eq!(clf.rr_window_size, 30);

        let peaks: Vec<usize> = (0..6).map(|i| i * 500).collect(); // 1 s apart
        let rr = clf.extract_rr_intervals(&peaks);
        assert_eq!(rr.len(), 5);
        for &r in &rr {
            assert!((r - 1.0).abs() < 1e-9);
        }

        let hrv = clf.compute_hrv(&rr);
        assert!((hrv.mean_rr - 1.0).abs() < 1e-9);

        let rhythm = clf.classify_rhythm(&rr);
        assert_eq!(rhythm, RhythmType::NormalSinus);

        let ecg = vec![0.0; 3000];
        let devs = clf.detect_st_deviation(&ecg, &[100, 500, 1000], 0.0);
        assert_eq!(devs.len(), 3);

        let (afib, _) = clf.detect_afib(&rr);
        assert!(!afib);
    }

    // -----------------------------------------------------------------------
    // Pearson correlation (internal)
    // -----------------------------------------------------------------------

    #[test]
    fn test_pearson_perfect_positive() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let r = pearson_correlation(&a, &b);
        assert!((r - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_pearson_perfect_negative() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b = vec![10.0, 8.0, 6.0, 4.0, 2.0];
        let r = pearson_correlation(&a, &b);
        assert!((r + 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_pearson_constant() {
        let a = vec![5.0; 10];
        let b = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let r = pearson_correlation(&a, &b);
        assert!(r.abs() < 1e-9);
    }
}

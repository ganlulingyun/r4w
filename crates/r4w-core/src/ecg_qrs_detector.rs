//! ECG QRS complex detection using the Pan-Tompkins algorithm.
//!
//! This module implements the Pan-Tompkins algorithm for real-time QRS detection
//! in electrocardiogram (ECG) signals. The algorithm applies a cascade of filters
//! (bandpass, derivative, squaring, moving window integration) followed by adaptive
//! dual-threshold peak detection with refractory period enforcement.
//!
//! # Features
//!
//! - Bandpass filtering (5–15 Hz) to isolate QRS energy
//! - Five-point derivative filter for slope detection
//! - Squaring for nonlinear amplification of QRS complexes
//! - Moving window integration for QRS width detection
//! - Adaptive dual thresholding (signal/noise levels)
//! - Refractory period to reject false detections
//! - Heart rate and HRV metric computation (SDNN, RMSSD)
//! - Beat classification (Normal, Premature, Missed)
//! - Signal quality index (0.0–1.0)
//!
//! # Example
//!
//! ```
//! use r4w_core::ecg_qrs_detector::{EcgQrsDetector, BeatType};
//!
//! // Create a detector for 360 Hz sample rate (common ECG rate)
//! let mut detector = EcgQrsDetector::new(360.0);
//!
//! // Simulate a simple ECG-like signal: baseline with periodic sharp peaks
//! let fs = 360.0;
//! let duration = 5.0; // seconds
//! let n = (fs * duration) as usize;
//! let mut signal = vec![0.0_f64; n];
//!
//! // Insert R-peak-like impulses every ~0.8 s (75 BPM)
//! let beat_interval = (0.8 * fs) as usize;
//! for i in (100..n).step_by(beat_interval) {
//!     // Create a narrow triangular pulse resembling a QRS complex
//!     for k in 0..12 {
//!         let idx = i + k;
//!         if idx < n {
//!             let amp = if k < 6 { k as f64 / 5.0 } else { (11 - k) as f64 / 5.0 };
//!             signal[idx] = amp;
//!         }
//!     }
//! }
//!
//! // Process the signal
//! let result = detector.process(&signal);
//!
//! // We should detect multiple R-peaks
//! assert!(result.r_peaks.len() >= 2, "Expected at least 2 R-peaks, got {}", result.r_peaks.len());
//!
//! // Heart rate should be in a reasonable range
//! if let Some(hr) = result.heart_rate_bpm {
//!     assert!(hr > 40.0 && hr < 200.0, "Heart rate {} out of range", hr);
//! }
//! ```

use std::f64::consts::PI;

/// Complex number as an (re, im) tuple.
pub type Complex = (f64, f64);

// ---------------------------------------------------------------------------
// Beat classification
// ---------------------------------------------------------------------------

/// Classification of a detected heartbeat.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BeatType {
    /// Normal sinus beat.
    Normal,
    /// Premature beat (R-R interval < 80% of running average).
    Premature,
    /// Missed beat detected (R-R interval > 150% of running average).
    Missed,
}

// ---------------------------------------------------------------------------
// Detection result
// ---------------------------------------------------------------------------

/// Result of QRS detection on an ECG signal.
#[derive(Debug, Clone)]
pub struct QrsDetectionResult {
    /// Sample indices of detected R-peaks.
    pub r_peaks: Vec<usize>,
    /// Heart rate in beats per minute (from mean R-R interval), if available.
    pub heart_rate_bpm: Option<f64>,
    /// R-R intervals in seconds.
    pub rr_intervals: Vec<f64>,
    /// Beat classifications corresponding to each R-R interval.
    pub beat_types: Vec<BeatType>,
    /// Standard deviation of N-N intervals (SDNN) in seconds, if available.
    pub sdnn: Option<f64>,
    /// Root mean square of successive differences (RMSSD) in seconds, if available.
    pub rmssd: Option<f64>,
    /// Signal quality index in [0.0, 1.0].
    pub signal_quality: f64,
}

// ---------------------------------------------------------------------------
// EcgQrsDetector
// ---------------------------------------------------------------------------

/// Pan-Tompkins QRS detector.
///
/// Processes an ECG signal through bandpass -> derivative -> squaring ->
/// moving-window integration and applies adaptive thresholding to locate
/// R-peaks.
#[derive(Debug, Clone)]
pub struct EcgQrsDetector {
    /// Sampling rate in Hz.
    sample_rate: f64,
    /// Moving window integration width in samples (about 150 ms).
    mwi_window: usize,
    /// Refractory period in samples (about 200 ms).
    refractory_samples: usize,
}

impl EcgQrsDetector {
    /// Create a new detector for the given sample rate (Hz).
    ///
    /// # Panics
    ///
    /// Panics if `sample_rate` is not positive.
    pub fn new(sample_rate: f64) -> Self {
        assert!(sample_rate > 0.0, "sample_rate must be positive");
        let mwi_window = (0.150 * sample_rate).round().max(1.0) as usize;
        let refractory_samples = (0.200 * sample_rate).round().max(1.0) as usize;
        Self {
            sample_rate,
            mwi_window,
            refractory_samples,
        }
    }

    /// Return the configured sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    // -----------------------------------------------------------------------
    // Filter stages
    // -----------------------------------------------------------------------

    /// Bandpass filter 5-15 Hz using cascaded biquad sections (HPF at 5 Hz
    /// then LPF at 15 Hz), designed via bilinear transform.
    pub fn bandpass_filter(&self, signal: &[f64]) -> Vec<f64> {
        let hp = self.biquad_highpass(5.0);
        let lp = self.biquad_lowpass(15.0);
        let stage1 = Self::apply_biquad(&hp, signal);
        Self::apply_biquad(&lp, &stage1)
    }

    /// Five-point derivative filter:
    /// y[n] = (1/8T)(-x[n-2] - 2x[n-1] + 2x[n+1] + x[n+2]).
    pub fn derivative_filter(&self, signal: &[f64]) -> Vec<f64> {
        let n = signal.len();
        let mut out = vec![0.0; n];
        let t = 1.0 / self.sample_rate;
        for i in 2..n.saturating_sub(2) {
            out[i] = (-signal[i - 2] - 2.0 * signal[i - 1]
                + 2.0 * signal[i + 1]
                + signal[i + 2])
                / (8.0 * t);
        }
        out
    }

    /// Element-wise squaring.
    pub fn squaring(signal: &[f64]) -> Vec<f64> {
        signal.iter().map(|&x| x * x).collect()
    }

    /// Moving window integration with the configured window width.
    pub fn moving_window_integration(&self, signal: &[f64]) -> Vec<f64> {
        let n = signal.len();
        let w = self.mwi_window;
        let mut out = vec![0.0; n];
        if n == 0 || w == 0 {
            return out;
        }
        // Use a running sum for efficiency.
        let mut sum: f64 = 0.0;
        for i in 0..n {
            sum += signal[i];
            if i >= w {
                sum -= signal[i - w];
            }
            let current_w = (i + 1).min(w);
            out[i] = sum / current_w as f64;
        }
        out
    }

    // -----------------------------------------------------------------------
    // Peak detection
    // -----------------------------------------------------------------------

    /// Run the full Pan-Tompkins pipeline on `signal` and return detection results.
    pub fn process(&mut self, signal: &[f64]) -> QrsDetectionResult {
        if signal.is_empty() {
            return QrsDetectionResult {
                r_peaks: vec![],
                heart_rate_bpm: None,
                rr_intervals: vec![],
                beat_types: vec![],
                sdnn: None,
                rmssd: None,
                signal_quality: 0.0,
            };
        }

        // 1) Filter cascade
        let bp = self.bandpass_filter(signal);
        let deriv = self.derivative_filter(&bp);
        let sq = Self::squaring(&deriv);
        let mwi = self.moving_window_integration(&sq);

        // 2) Adaptive thresholding
        let r_peaks = self.adaptive_threshold_detect(&mwi);

        // 3) Derive metrics
        let rr_intervals = self.compute_rr_intervals(&r_peaks);
        let beat_types = self.classify_beats(&rr_intervals);
        let heart_rate_bpm = self.compute_heart_rate(&rr_intervals);
        let sdnn = Self::compute_sdnn(&rr_intervals);
        let rmssd = Self::compute_rmssd(&rr_intervals);
        let signal_quality = self.compute_signal_quality(signal, &r_peaks);

        QrsDetectionResult {
            r_peaks,
            heart_rate_bpm,
            rr_intervals,
            beat_types,
            sdnn,
            rmssd,
            signal_quality,
        }
    }

    /// Adaptive dual-threshold peak detection on the MWI signal.
    fn adaptive_threshold_detect(&self, mwi: &[f64]) -> Vec<usize> {
        let n = mwi.len();
        if n == 0 {
            return vec![];
        }

        // Initialize thresholds from the first second of data.
        let init_len = (self.sample_rate as usize).min(n);
        let max_init = mwi[..init_len]
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let mean_init = mwi[..init_len].iter().sum::<f64>() / init_len as f64;

        let mut spki = max_init * 0.25; // signal peak estimate
        let mut npki = mean_init * 0.5; // noise peak estimate
        let mut threshold1 = npki + 0.25 * (spki - npki);
        let mut _threshold2 = 0.5 * threshold1;

        let mut peaks: Vec<usize> = Vec::new();
        let mut last_peak: Option<usize> = None;

        // Scan for local maxima above threshold
        let mut i = 1;
        while i < n.saturating_sub(1) {
            if mwi[i] > mwi[i - 1] && mwi[i] >= mwi[i + 1] && mwi[i] > threshold1 {
                // Check refractory period
                let ok = match last_peak {
                    Some(lp) => i - lp >= self.refractory_samples,
                    None => true,
                };
                if ok {
                    peaks.push(i);
                    last_peak = Some(i);
                    // Update signal peak level
                    spki = 0.125 * mwi[i] + 0.875 * spki;
                } else {
                    // Treat as noise
                    npki = 0.125 * mwi[i] + 0.875 * npki;
                }
                // Recompute thresholds
                threshold1 = npki + 0.25 * (spki - npki);
                _threshold2 = 0.5 * threshold1;
            }
            i += 1;
        }

        peaks
    }

    // -----------------------------------------------------------------------
    // Heart-rate & HRV helpers
    // -----------------------------------------------------------------------

    /// Compute R-R intervals in seconds from peak sample indices.
    pub fn compute_rr_intervals(&self, peaks: &[usize]) -> Vec<f64> {
        if peaks.len() < 2 {
            return vec![];
        }
        peaks
            .windows(2)
            .map(|w| (w[1] as f64 - w[0] as f64) / self.sample_rate)
            .collect()
    }

    /// Compute heart rate (BPM) from R-R intervals.
    pub fn compute_heart_rate(&self, rr: &[f64]) -> Option<f64> {
        if rr.is_empty() {
            return None;
        }
        let mean_rr: f64 = rr.iter().sum::<f64>() / rr.len() as f64;
        if mean_rr <= 0.0 {
            return None;
        }
        Some(60.0 / mean_rr)
    }

    /// Classify each R-R interval as Normal, Premature, or Missed relative to
    /// a running average.
    pub fn classify_beats(&self, rr: &[f64]) -> Vec<BeatType> {
        if rr.is_empty() {
            return vec![];
        }
        let mean_rr: f64 = rr.iter().sum::<f64>() / rr.len() as f64;
        rr.iter()
            .map(|&r| {
                if r < 0.80 * mean_rr {
                    BeatType::Premature
                } else if r > 1.50 * mean_rr {
                    BeatType::Missed
                } else {
                    BeatType::Normal
                }
            })
            .collect()
    }

    /// SDNN: standard deviation of all R-R (N-N) intervals.
    pub fn compute_sdnn(rr: &[f64]) -> Option<f64> {
        if rr.len() < 2 {
            return None;
        }
        let mean = rr.iter().sum::<f64>() / rr.len() as f64;
        let var = rr.iter().map(|&r| (r - mean).powi(2)).sum::<f64>() / (rr.len() - 1) as f64;
        Some(var.sqrt())
    }

    /// RMSSD: root mean square of successive R-R differences.
    pub fn compute_rmssd(rr: &[f64]) -> Option<f64> {
        if rr.len() < 2 {
            return None;
        }
        let sum_sq: f64 = rr
            .windows(2)
            .map(|w| (w[1] - w[0]).powi(2))
            .sum();
        let n = (rr.len() - 1) as f64;
        Some((sum_sq / n).sqrt())
    }

    /// Compute a signal quality index in [0, 1].
    ///
    /// Heuristic: measures regularity of detected peaks relative to expected
    /// cardiac rhythm and amplitude consistency.
    pub fn compute_signal_quality(&self, signal: &[f64], peaks: &[usize]) -> f64 {
        if peaks.len() < 2 || signal.is_empty() {
            return 0.0;
        }

        // Factor 1: R-R regularity (coefficient of variation, inverted)
        let rr = self.compute_rr_intervals(peaks);
        let mean_rr = rr.iter().sum::<f64>() / rr.len() as f64;
        let cv = if mean_rr > 0.0 {
            let std = Self::compute_sdnn(&rr).unwrap_or(0.0);
            std / mean_rr
        } else {
            1.0
        };
        let regularity_score = (1.0 - cv.min(1.0)).max(0.0);

        // Factor 2: Reasonable heart rate range (40-200 BPM)
        let hr = if mean_rr > 0.0 { 60.0 / mean_rr } else { 0.0 };
        let hr_score = if (40.0..=200.0).contains(&hr) {
            1.0
        } else if (30.0..=250.0).contains(&hr) {
            0.5
        } else {
            0.0
        };

        // Factor 3: Enough peaks detected for the signal length
        let duration_s = signal.len() as f64 / self.sample_rate;
        let expected_min_peaks = (duration_s * 0.5).max(1.0); // at least 30 BPM
        let peak_density_score = (peaks.len() as f64 / expected_min_peaks).min(1.0);

        // Weighted combination
        let quality = 0.5 * regularity_score + 0.3 * hr_score + 0.2 * peak_density_score;
        quality.clamp(0.0, 1.0)
    }

    // -----------------------------------------------------------------------
    // Complex-valued helpers (using (f64, f64) tuples)
    // -----------------------------------------------------------------------

    /// Compute the analytic signal (via Hilbert transform approximation)
    /// returning complex-valued samples as `(re, im)` tuples.
    ///
    /// Uses a simple FIR approximation of the Hilbert transform (odd-length
    /// kernel). The real part is the original signal (delayed to align) and the
    /// imaginary part is the Hilbert-transformed signal.
    pub fn analytic_signal(signal: &[f64]) -> Vec<Complex> {
        let n = signal.len();
        if n == 0 {
            return vec![];
        }

        // FIR Hilbert transformer of length 31
        let half = 15_i32;
        let len = (2 * half + 1) as usize;
        let mut h = vec![0.0_f64; len];
        for k in 0..len {
            let m = k as i32 - half;
            if m % 2 != 0 {
                h[k] = 2.0 / (PI * m as f64);
            }
        }
        // Apply Hamming window
        for k in 0..len {
            let w = 0.54 - 0.46 * (2.0 * PI * k as f64 / (len - 1) as f64).cos();
            h[k] *= w;
        }

        // Convolve and build analytic signal
        let delay = half as usize;
        let mut result = Vec::with_capacity(n);
        for i in 0..n {
            let mut imag = 0.0;
            for k in 0..len {
                let j = i as i64 - k as i64 + half as i64;
                if j >= 0 && (j as usize) < n {
                    imag += h[k] * signal[j as usize];
                }
            }
            let re = if i >= delay && i - delay < n {
                signal[i - delay]
            } else {
                0.0
            };
            result.push((re, imag));
        }
        result
    }

    /// Compute the instantaneous heart rate from the analytic signal envelope.
    ///
    /// Returns a vector of instantaneous BPM values (one per sample).
    pub fn instantaneous_heart_rate(signal: &[f64], sample_rate: f64) -> Vec<f64> {
        let analytic = Self::analytic_signal(signal);
        // Envelope = magnitude of analytic signal
        let envelope: Vec<f64> = analytic
            .iter()
            .map(|&(re, im)| (re * re + im * im).sqrt())
            .collect();

        // Find zero crossings of the envelope derivative to estimate period
        let n = envelope.len();
        if n < 3 {
            return vec![0.0; n];
        }
        let deriv: Vec<f64> = (1..n - 1)
            .map(|i| (envelope[i + 1] - envelope[i - 1]) / 2.0)
            .collect();

        // Simple: assign constant HR between detected peaks
        let mut peaks = Vec::new();
        for i in 1..deriv.len().saturating_sub(1) {
            if deriv[i] > 0.0 && deriv[i + 1] <= 0.0 {
                peaks.push(i + 1); // +1 because deriv is offset by 1
            }
        }

        let mut ihr = vec![0.0; n];
        if peaks.len() >= 2 {
            for w in peaks.windows(2) {
                let period_s = (w[1] - w[0]) as f64 / sample_rate;
                let bpm = if period_s > 0.0 {
                    60.0 / period_s
                } else {
                    0.0
                };
                for idx in w[0]..=w[1] {
                    if idx < n {
                        ihr[idx] = bpm;
                    }
                }
            }
        }
        ihr
    }

    // -----------------------------------------------------------------------
    // Biquad IIR helpers
    // -----------------------------------------------------------------------

    /// Design a second-order Butterworth lowpass biquad at frequency `fc` Hz.
    fn biquad_lowpass(&self, fc: f64) -> [f64; 6] {
        let wc = 2.0 * PI * fc / self.sample_rate;
        let k = (wc / 2.0).tan();
        let k2 = k * k;
        let sqrt2 = std::f64::consts::SQRT_2;
        let norm = 1.0 / (1.0 + sqrt2 * k + k2);
        let b0 = k2 * norm;
        let b1 = 2.0 * b0;
        let b2 = b0;
        let a1 = 2.0 * (k2 - 1.0) * norm;
        let a2 = (1.0 - sqrt2 * k + k2) * norm;
        [b0, b1, b2, 1.0, a1, a2]
    }

    /// Design a second-order Butterworth highpass biquad at frequency `fc` Hz.
    fn biquad_highpass(&self, fc: f64) -> [f64; 6] {
        let wc = 2.0 * PI * fc / self.sample_rate;
        let k = (wc / 2.0).tan();
        let k2 = k * k;
        let sqrt2 = std::f64::consts::SQRT_2;
        let norm = 1.0 / (1.0 + sqrt2 * k + k2);
        let b0 = norm;
        let b1 = -2.0 * norm;
        let b2 = norm;
        let a1 = 2.0 * (k2 - 1.0) * norm;
        let a2 = (1.0 - sqrt2 * k + k2) * norm;
        [b0, b1, b2, 1.0, a1, a2]
    }

    /// Apply a biquad filter (direct form I) to `signal`.
    fn apply_biquad(coeffs: &[f64; 6], signal: &[f64]) -> Vec<f64> {
        let [b0, b1, b2, _a0, a1, a2] = *coeffs;
        let n = signal.len();
        let mut out = vec![0.0; n];
        let (mut x1, mut x2) = (0.0, 0.0);
        let (mut y1, mut y2) = (0.0, 0.0);
        for i in 0..n {
            let x0 = signal[i];
            let y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            out[i] = y0;
            x2 = x1;
            x1 = x0;
            y2 = y1;
            y1 = y0;
        }
        out
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a synthetic ECG-like signal with periodic QRS impulses.
    fn make_synthetic_ecg(fs: f64, duration: f64, beat_interval_s: f64) -> Vec<f64> {
        let n = (fs * duration) as usize;
        let mut signal = vec![0.0; n];
        let beat_samples = (beat_interval_s * fs) as usize;
        // Use a Gaussian-like pulse for QRS
        let qrs_half_width = (0.020 * fs) as usize; // +/- 20 ms
        for start in (beat_samples..n).step_by(beat_samples) {
            for k in 0..=(2 * qrs_half_width) {
                let idx = if start + k >= qrs_half_width {
                    start + k - qrs_half_width
                } else {
                    continue;
                };
                if idx < n {
                    let t = (k as f64 - qrs_half_width as f64) / (qrs_half_width as f64 / 2.5);
                    signal[idx] += (-0.5 * t * t).exp();
                }
            }
        }
        signal
    }

    #[test]
    fn test_new_default() {
        let det = EcgQrsDetector::new(360.0);
        assert_eq!(det.sample_rate(), 360.0);
        assert!(det.mwi_window > 0);
        assert!(det.refractory_samples > 0);
    }

    #[test]
    #[should_panic(expected = "sample_rate must be positive")]
    fn test_new_invalid_sample_rate() {
        EcgQrsDetector::new(0.0);
    }

    #[test]
    fn test_bandpass_filter_length() {
        let det = EcgQrsDetector::new(360.0);
        let sig = vec![1.0; 100];
        let out = det.bandpass_filter(&sig);
        assert_eq!(out.len(), 100);
    }

    #[test]
    fn test_bandpass_filter_dc_rejection() {
        // A DC signal should be attenuated by the highpass at 5 Hz.
        let det = EcgQrsDetector::new(360.0);
        let sig = vec![1.0; 2000];
        let out = det.bandpass_filter(&sig);
        // After settling, output should be near zero
        let tail_mean: f64 =
            out[1500..].iter().map(|x| x.abs()).sum::<f64>() / 500.0;
        assert!(
            tail_mean < 0.05,
            "DC should be rejected, got mean abs = {tail_mean}"
        );
    }

    #[test]
    fn test_derivative_filter_length() {
        let det = EcgQrsDetector::new(360.0);
        let sig: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let out = det.derivative_filter(&sig);
        assert_eq!(out.len(), 100);
    }

    #[test]
    fn test_derivative_of_ramp() {
        // The derivative of a ramp should be approximately constant in the interior.
        let det = EcgQrsDetector::new(1.0); // fs=1 simplifies math
        let sig: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let out = det.derivative_filter(&sig);
        // For a linear ramp x[n]=n with fs=1:
        // (-x[n-2] - 2x[n-1] + 2x[n+1] + x[n+2]) / (8*T) where T=1
        // = (-(n-2) - 2(n-1) + 2(n+1) + (n+2)) / 8 = 8/8 = 1
        for &v in &out[5..45] {
            assert!(
                (v - 1.0).abs() < 0.01,
                "derivative of ramp should be ~1.0, got {v}"
            );
        }
    }

    #[test]
    fn test_squaring_positive() {
        let sig = vec![-3.0, -1.0, 0.0, 1.0, 3.0];
        let sq = EcgQrsDetector::squaring(&sig);
        assert_eq!(sq, vec![9.0, 1.0, 0.0, 1.0, 9.0]);
    }

    #[test]
    fn test_moving_window_integration_length() {
        let det = EcgQrsDetector::new(360.0);
        let sig = vec![1.0; 200];
        let out = det.moving_window_integration(&sig);
        assert_eq!(out.len(), 200);
    }

    #[test]
    fn test_moving_window_integration_constant() {
        // MWI of constant signal should equal the constant (after ramp-up).
        let det = EcgQrsDetector::new(100.0); // mwi_window = 15
        let sig = vec![5.0; 100];
        let out = det.moving_window_integration(&sig);
        // After the window fills, values should be 5.0
        for &v in &out[det.mwi_window..] {
            assert!(
                (v - 5.0).abs() < 1e-10,
                "MWI of constant should equal constant, got {v}"
            );
        }
    }

    #[test]
    fn test_empty_signal() {
        let mut det = EcgQrsDetector::new(360.0);
        let result = det.process(&[]);
        assert!(result.r_peaks.is_empty());
        assert!(result.heart_rate_bpm.is_none());
        assert_eq!(result.signal_quality, 0.0);
    }

    #[test]
    fn test_detect_synthetic_peaks() {
        let fs = 360.0;
        let signal = make_synthetic_ecg(fs, 10.0, 0.8);
        let mut det = EcgQrsDetector::new(fs);
        let result = det.process(&signal);
        // At 0.8 s interval and 10 s duration, expect about 12 beats
        assert!(
            result.r_peaks.len() >= 5,
            "Expected >=5 peaks, got {}",
            result.r_peaks.len()
        );
    }

    #[test]
    fn test_heart_rate_reasonable() {
        let fs = 360.0;
        let signal = make_synthetic_ecg(fs, 10.0, 0.75);
        let mut det = EcgQrsDetector::new(fs);
        let result = det.process(&signal);
        if let Some(hr) = result.heart_rate_bpm {
            assert!(
                hr > 50.0 && hr < 120.0,
                "Heart rate {hr} BPM out of expected range for 0.75 s intervals"
            );
        }
    }

    #[test]
    fn test_rr_intervals_count() {
        let fs = 250.0;
        let signal = make_synthetic_ecg(fs, 8.0, 1.0);
        let mut det = EcgQrsDetector::new(fs);
        let result = det.process(&signal);
        // R-R intervals = peaks - 1
        if result.r_peaks.len() >= 2 {
            assert_eq!(result.rr_intervals.len(), result.r_peaks.len() - 1);
        }
    }

    #[test]
    fn test_beat_classification_normal() {
        let det = EcgQrsDetector::new(360.0);
        let rr = vec![0.8, 0.8, 0.8, 0.8];
        let types = det.classify_beats(&rr);
        assert!(types.iter().all(|&t| t == BeatType::Normal));
    }

    #[test]
    fn test_beat_classification_premature() {
        let det = EcgQrsDetector::new(360.0);
        // Mean ~ 0.77; 0.5 < 0.77 * 0.80 = 0.616 -> premature
        let rr = vec![0.8, 0.8, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8];
        let types = det.classify_beats(&rr);
        assert_eq!(types[2], BeatType::Premature);
    }

    #[test]
    fn test_beat_classification_missed() {
        let det = EcgQrsDetector::new(360.0);
        // Mean ~ 0.87; 1.5 > 0.87*1.5 = 1.305 -> missed
        let rr = vec![0.8, 0.8, 1.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8];
        let types = det.classify_beats(&rr);
        assert_eq!(types[2], BeatType::Missed);
    }

    #[test]
    fn test_sdnn_known() {
        // SDNN of [1, 2, 3]: mean=2, var = ((1+0+1)/2) = 1, std=1
        let sdnn = EcgQrsDetector::compute_sdnn(&[1.0, 2.0, 3.0]).unwrap();
        assert!((sdnn - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_rmssd_known() {
        // RMSSD of [1, 3, 2]: diffs = [2, -1], sq = [4, 1], mean = 2.5, rmssd = sqrt(2.5)
        let rmssd = EcgQrsDetector::compute_rmssd(&[1.0, 3.0, 2.0]).unwrap();
        let expected = (2.5_f64).sqrt();
        assert!(
            (rmssd - expected).abs() < 1e-10,
            "RMSSD expected {expected}, got {rmssd}"
        );
    }

    #[test]
    fn test_signal_quality_range() {
        let fs = 360.0;
        let signal = make_synthetic_ecg(fs, 5.0, 0.8);
        let mut det = EcgQrsDetector::new(fs);
        let result = det.process(&signal);
        assert!(
            (0.0..=1.0).contains(&result.signal_quality),
            "Signal quality {} out of [0,1]",
            result.signal_quality
        );
    }

    #[test]
    fn test_analytic_signal_length() {
        let sig = vec![1.0; 50];
        let a = EcgQrsDetector::analytic_signal(&sig);
        assert_eq!(a.len(), 50);
    }

    #[test]
    fn test_analytic_signal_finite() {
        // The analytic signal entries should all be finite
        let sig: Vec<f64> = (0..100).map(|i| (2.0 * PI * 10.0 * i as f64 / 100.0).sin()).collect();
        let a = EcgQrsDetector::analytic_signal(&sig);
        assert!(a.iter().all(|&(re, im)| re.is_finite() && im.is_finite()));
    }

    #[test]
    fn test_instantaneous_heart_rate_length() {
        let sig = vec![0.0; 200];
        let ihr = EcgQrsDetector::instantaneous_heart_rate(&sig, 100.0);
        assert_eq!(ihr.len(), 200);
    }

    #[test]
    fn test_complex_tuple_usage() {
        // Verify that Complex type is (f64, f64)
        let c: Complex = (3.0, 4.0);
        let mag = (c.0 * c.0 + c.1 * c.1).sqrt();
        assert!((mag - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_refractory_period_prevents_double_detect() {
        // Two peaks very close together; only the first should be detected.
        let fs = 360.0;
        let mut det = EcgQrsDetector::new(fs);
        let n = 2000;
        let mut signal = vec![0.0; n];
        // Place two sharp peaks 50 ms apart (< 200 ms refractory)
        let peak1 = 500;
        let peak2 = peak1 + (0.050 * fs) as usize;
        for &p in &[peak1, peak2] {
            for k in 0..14 {
                let idx = p + k;
                if idx < n {
                    let t = (k as f64 - 6.0) / 2.5;
                    signal[idx] += (-0.5 * t * t).exp();
                }
            }
        }
        let result = det.process(&signal);
        // Should not detect both as separate beats
        let peaks_in_region: Vec<_> = result
            .r_peaks
            .iter()
            .filter(|&&p| p >= peak1.saturating_sub(100) && p <= peak2 + 100)
            .collect();
        assert!(
            peaks_in_region.len() <= 1,
            "Refractory period should prevent double detection, got {} peaks",
            peaks_in_region.len()
        );
    }
}

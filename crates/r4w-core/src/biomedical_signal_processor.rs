//! Biomedical signal processing for ECG and EEG applications.
//!
//! This module provides tools for processing biomedical signals including
//! powerline interference removal, baseline wander correction, QRS complex
//! detection using the Pan-Tompkins algorithm, heart rate computation,
//! and heart rate variability (HRV) analysis.
//!
//! # Example
//!
//! ```
//! use r4w_core::biomedical_signal_processor::{BiomedicalProcessor, ProcessorConfig};
//!
//! // Configure for 360 Hz ECG with 60 Hz powerline
//! let config = ProcessorConfig {
//!     sample_rate: 360.0,
//!     powerline_freq: 60.0,
//!     notch_bandwidth: 2.0,
//! };
//! let processor = BiomedicalProcessor::new(config);
//!
//! // Generate a synthetic ECG-like signal (1 second)
//! let n = 360;
//! let mut ecg: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 360.0;
//!         // Simulated R-peak every 0.8 seconds (75 BPM)
//!         let phase = (t % 0.8) / 0.8;
//!         let r_peak = if (phase - 0.4).abs() < 0.02 { 1.5 } else { 0.0 };
//!         // Add baseline wander and powerline noise
//!         let baseline = 0.3 * (2.0 * std::f64::consts::PI * 0.2 * t).sin();
//!         let powerline = 0.1 * (2.0 * std::f64::consts::PI * 60.0 * t).sin();
//!         r_peak + baseline + powerline
//!     })
//!     .collect();
//!
//! // Remove powerline interference
//! let clean = processor.remove_powerline(&ecg);
//! assert_eq!(clean.len(), ecg.len());
//!
//! // Remove baseline wander
//! let corrected = processor.remove_baseline_wander(&clean);
//! assert_eq!(corrected.len(), clean.len());
//! ```

use std::f64::consts::PI;

/// Configuration for the biomedical signal processor.
#[derive(Debug, Clone)]
pub struct ProcessorConfig {
    /// Sampling rate in Hz (e.g., 360.0 for MIT-BIH).
    pub sample_rate: f64,
    /// Powerline frequency in Hz (50.0 or 60.0).
    pub powerline_freq: f64,
    /// Notch filter bandwidth in Hz around the powerline frequency.
    pub notch_bandwidth: f64,
}

impl Default for ProcessorConfig {
    fn default() -> Self {
        Self {
            sample_rate: 360.0,
            powerline_freq: 60.0,
            notch_bandwidth: 2.0,
        }
    }
}

/// Result from QRS complex detection.
#[derive(Debug, Clone)]
pub struct QrsDetection {
    /// Sample index of the detected R-peak.
    pub sample_index: usize,
    /// R-R interval in milliseconds to the previous beat (None for the first beat).
    pub rr_interval_ms: Option<f64>,
    /// Instantaneous heart rate in BPM derived from the R-R interval.
    pub heart_rate_bpm: Option<f64>,
}

/// Heart rate variability metrics computed from R-R intervals.
#[derive(Debug, Clone)]
pub struct HrvMetrics {
    /// Standard deviation of N-N intervals in milliseconds.
    pub sdnn_ms: f64,
    /// Root mean square of successive differences in milliseconds.
    pub rmssd_ms: f64,
    /// Percentage of successive N-N intervals differing by more than 50 ms.
    pub pnn50_percent: f64,
    /// Mean R-R interval in milliseconds.
    pub mean_rr_ms: f64,
}

/// Artifact detection result.
#[derive(Debug, Clone)]
pub struct Artifact {
    /// Starting sample index of the artifact.
    pub start: usize,
    /// Ending sample index of the artifact (exclusive).
    pub end: usize,
    /// Peak amplitude within the artifact region.
    pub peak_amplitude: f64,
}

/// Biomedical signal processor for ECG/EEG signals.
///
/// Provides powerline removal, baseline correction, QRS detection,
/// heart rate computation, and HRV analysis.
#[derive(Debug, Clone)]
pub struct BiomedicalProcessor {
    config: ProcessorConfig,
}

impl BiomedicalProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: ProcessorConfig) -> Self {
        Self { config }
    }

    /// Return the current configuration.
    pub fn config(&self) -> &ProcessorConfig {
        &self.config
    }

    /// Remove powerline interference at the configured frequency and its harmonics.
    ///
    /// Uses a second-order IIR notch filter at each harmonic frequency below Nyquist.
    /// The notch bandwidth is controlled by `ProcessorConfig::notch_bandwidth`.
    pub fn remove_powerline(&self, signal: &[f64]) -> Vec<f64> {
        if signal.is_empty() {
            return Vec::new();
        }

        let fs = self.config.sample_rate;
        let nyquist = fs / 2.0;
        let mut output = signal.to_vec();

        // Apply notch at fundamental and harmonics
        let mut harmonic = 1u32;
        loop {
            let freq = self.config.powerline_freq * harmonic as f64;
            if freq >= nyquist {
                break;
            }
            output = self.apply_notch_filter(&output, freq);
            harmonic += 1;
        }

        output
    }

    /// Apply a second-order IIR notch filter at the given frequency.
    fn apply_notch_filter(&self, signal: &[f64], freq: f64) -> Vec<f64> {
        let fs = self.config.sample_rate;
        let w0 = 2.0 * PI * freq / fs;
        // Quality factor from bandwidth
        let bw = self.config.notch_bandwidth;
        let q = freq / bw;
        let alpha = w0.sin() / (2.0 * q);

        // Transfer function coefficients for notch (all-pass minus bandpass)
        let b0 = 1.0;
        let b1 = -2.0 * w0.cos();
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * w0.cos();
        let a2 = 1.0 - alpha;

        // Normalize
        let b0 = b0 / a0;
        let b1 = b1 / a0;
        let b2 = b2 / a0;
        let a1 = a1 / a0;
        let a2 = a2 / a0;

        // Direct Form II Transposed
        let mut output = vec![0.0; signal.len()];
        let mut d1 = 0.0;
        let mut d2 = 0.0;

        for i in 0..signal.len() {
            let x = signal[i];
            let y = b0 * x + d1;
            d1 = b1 * x - a1 * y + d2;
            d2 = b2 * x - a2 * y;
            output[i] = y;
        }

        output
    }

    /// Remove baseline wander using a high-pass filter with 0.5 Hz cutoff.
    ///
    /// Implemented as a single-pole IIR high-pass filter:
    /// `y[n] = alpha * (y[n-1] + x[n] - x[n-1])`
    pub fn remove_baseline_wander(&self, signal: &[f64]) -> Vec<f64> {
        if signal.is_empty() {
            return Vec::new();
        }

        let fs = self.config.sample_rate;
        let cutoff = 0.5; // Hz
        let rc = 1.0 / (2.0 * PI * cutoff);
        let dt = 1.0 / fs;
        let alpha = rc / (rc + dt);

        let mut output = vec![0.0; signal.len()];
        output[0] = signal[0];

        for i in 1..signal.len() {
            output[i] = alpha * (output[i - 1] + signal[i] - signal[i - 1]);
        }

        output
    }

    /// Bandpass filter the signal between 0.5 and 40 Hz for ECG preprocessing.
    ///
    /// Applies the baseline wander removal (high-pass at 0.5 Hz) followed
    /// by a low-pass filter at 40 Hz to remove high-frequency noise.
    pub fn bandpass_filter(&self, signal: &[f64]) -> Vec<f64> {
        if signal.is_empty() {
            return Vec::new();
        }

        // High-pass at 0.5 Hz
        let hp = self.remove_baseline_wander(signal);

        // Low-pass at 40 Hz using single-pole IIR
        let fs = self.config.sample_rate;
        let cutoff = 40.0;
        let rc = 1.0 / (2.0 * PI * cutoff);
        let dt = 1.0 / fs;
        let alpha = dt / (rc + dt);

        let mut output = vec![0.0; hp.len()];
        output[0] = alpha * hp[0];

        for i in 1..hp.len() {
            output[i] = output[i - 1] + alpha * (hp[i] - output[i - 1]);
        }

        output
    }

    /// Detect QRS complexes using the Pan-Tompkins algorithm.
    ///
    /// Steps:
    /// 1. Bandpass filter (5-15 Hz)
    /// 2. Differentiation
    /// 3. Squaring
    /// 4. Moving window integration
    /// 5. Adaptive thresholding
    ///
    /// Returns a list of detected QRS complexes with R-R intervals and heart rates.
    pub fn detect_qrs(&self, signal: &[f64]) -> Vec<QrsDetection> {
        if signal.len() < 10 {
            return Vec::new();
        }

        let fs = self.config.sample_rate;

        // Step 1: Bandpass filter 5-15 Hz using cascaded HP + LP
        let filtered = self.pan_tompkins_bandpass(signal);

        // Step 2: Derivative (five-point derivative)
        let derivative = self.five_point_derivative(&filtered);

        // Step 3: Squaring
        let squared: Vec<f64> = derivative.iter().map(|&x| x * x).collect();

        // Step 4: Moving window integration
        // Window width ~150ms
        let window_len = (0.150 * fs).round() as usize;
        let window_len = window_len.max(1);
        let integrated = self.moving_window_integration(&squared, window_len);

        // Step 5: Adaptive thresholding to find R-peaks
        let peaks = self.adaptive_threshold_peaks(&integrated, fs);

        // Build QRS detections with R-R intervals
        let mut detections = Vec::new();
        for (idx, &peak_sample) in peaks.iter().enumerate() {
            let (rr_ms, hr_bpm) = if idx > 0 {
                let rr_samples = peak_sample as f64 - peaks[idx - 1] as f64;
                let rr_ms = rr_samples / fs * 1000.0;
                let hr = 60000.0 / rr_ms;
                (Some(rr_ms), Some(hr))
            } else {
                (None, None)
            };

            detections.push(QrsDetection {
                sample_index: peak_sample,
                rr_interval_ms: rr_ms,
                heart_rate_bpm: hr_bpm,
            });
        }

        detections
    }

    /// Pan-Tompkins bandpass filter (5-15 Hz) using cascaded IIR filters.
    fn pan_tompkins_bandpass(&self, signal: &[f64]) -> Vec<f64> {
        let fs = self.config.sample_rate;

        // High-pass at 5 Hz
        let cutoff_hp = 5.0;
        let rc_hp = 1.0 / (2.0 * PI * cutoff_hp);
        let dt = 1.0 / fs;
        let alpha_hp = rc_hp / (rc_hp + dt);

        let mut hp = vec![0.0; signal.len()];
        if !signal.is_empty() {
            hp[0] = signal[0];
            for i in 1..signal.len() {
                hp[i] = alpha_hp * (hp[i - 1] + signal[i] - signal[i - 1]);
            }
        }

        // Low-pass at 15 Hz
        let cutoff_lp = 15.0;
        let rc_lp = 1.0 / (2.0 * PI * cutoff_lp);
        let alpha_lp = dt / (rc_lp + dt);

        let mut lp = vec![0.0; hp.len()];
        if !hp.is_empty() {
            lp[0] = alpha_lp * hp[0];
            for i in 1..hp.len() {
                lp[i] = lp[i - 1] + alpha_lp * (hp[i] - lp[i - 1]);
            }
        }

        lp
    }

    /// Five-point derivative filter.
    fn five_point_derivative(&self, signal: &[f64]) -> Vec<f64> {
        let n = signal.len();
        let mut deriv = vec![0.0; n];
        for i in 2..n.saturating_sub(2) {
            deriv[i] = (-signal[i.saturating_sub(2)] - 2.0 * signal[i.saturating_sub(1)]
                + 2.0 * signal.get(i + 1).copied().unwrap_or(0.0)
                + signal.get(i + 2).copied().unwrap_or(0.0))
                / 8.0;
        }
        deriv
    }

    /// Moving window integration.
    fn moving_window_integration(&self, signal: &[f64], window_len: usize) -> Vec<f64> {
        let n = signal.len();
        let mut output = vec![0.0; n];
        let mut sum = 0.0;

        for i in 0..n {
            sum += signal[i];
            if i >= window_len {
                sum -= signal[i - window_len];
            }
            let count = (i + 1).min(window_len);
            output[i] = sum / count as f64;
        }

        output
    }

    /// Adaptive thresholding for peak detection in the integrated signal.
    fn adaptive_threshold_peaks(&self, signal: &[f64], fs: f64) -> Vec<usize> {
        let n = signal.len();
        if n == 0 {
            return Vec::new();
        }

        // Find max for initial threshold
        let max_val = signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if max_val <= 0.0 {
            return Vec::new();
        }

        let mut threshold = 0.3 * max_val;
        let refractory_period = (0.200 * fs) as usize; // 200ms refractory

        let mut peaks = Vec::new();
        let mut last_peak: Option<usize> = None;

        for i in 1..n - 1 {
            // Check refractory period
            if let Some(lp) = last_peak {
                if i < lp + refractory_period {
                    continue;
                }
            }

            // Local maximum above threshold
            if signal[i] > threshold && signal[i] >= signal[i - 1] && signal[i] >= signal[i + 1] {
                // Refine: search for the actual peak in a small window around i
                let search_start = i.saturating_sub(refractory_period / 4);
                let search_end = (i + refractory_period / 4).min(n);
                let mut best = i;
                for j in search_start..search_end {
                    if signal[j] > signal[best] {
                        best = j;
                    }
                }

                peaks.push(best);
                last_peak = Some(best);

                // Update threshold: 0.25 * peak + 0.75 * old threshold
                threshold = 0.25 * signal[best] + 0.75 * threshold;
            }
        }

        peaks
    }

    /// Compute average heart rate in BPM from QRS detections.
    ///
    /// Returns `None` if fewer than 2 beats are detected.
    pub fn compute_heart_rate(&self, detections: &[QrsDetection]) -> Option<f64> {
        let rr_intervals: Vec<f64> = detections
            .iter()
            .filter_map(|d| d.rr_interval_ms)
            .collect();

        if rr_intervals.is_empty() {
            return None;
        }

        let mean_rr = rr_intervals.iter().sum::<f64>() / rr_intervals.len() as f64;
        Some(60000.0 / mean_rr)
    }

    /// Compute heart rate variability metrics from QRS detections.
    ///
    /// Returns `None` if fewer than 3 beats are detected (need at least 2 R-R intervals).
    pub fn hrv_metrics(&self, detections: &[QrsDetection]) -> Option<HrvMetrics> {
        let rr_intervals: Vec<f64> = detections
            .iter()
            .filter_map(|d| d.rr_interval_ms)
            .collect();

        if rr_intervals.len() < 2 {
            return None;
        }

        let n = rr_intervals.len() as f64;
        let mean_rr = rr_intervals.iter().sum::<f64>() / n;

        // SDNN: standard deviation of R-R intervals
        let variance = rr_intervals
            .iter()
            .map(|&rr| (rr - mean_rr).powi(2))
            .sum::<f64>()
            / (n - 1.0);
        let sdnn = variance.sqrt();

        // RMSSD: root mean square of successive differences
        let mut sum_sq_diff = 0.0;
        let mut count_diff = 0usize;
        let mut count_nn50 = 0usize;

        for i in 1..rr_intervals.len() {
            let diff = rr_intervals[i] - rr_intervals[i - 1];
            sum_sq_diff += diff * diff;
            count_diff += 1;
            if diff.abs() > 50.0 {
                count_nn50 += 1;
            }
        }

        let rmssd = if count_diff > 0 {
            (sum_sq_diff / count_diff as f64).sqrt()
        } else {
            0.0
        };

        // pNN50: percentage of successive intervals differing by > 50 ms
        let pnn50 = if count_diff > 0 {
            (count_nn50 as f64 / count_diff as f64) * 100.0
        } else {
            0.0
        };

        Some(HrvMetrics {
            sdnn_ms: sdnn,
            rmssd_ms: rmssd,
            pnn50_percent: pnn50,
            mean_rr_ms: mean_rr,
        })
    }

    /// Detect large-amplitude artifacts in the signal.
    ///
    /// An artifact is a region where the absolute amplitude exceeds
    /// `threshold_factor` times the signal's standard deviation. Adjacent
    /// artifact samples are merged into contiguous regions.
    pub fn detect_artifacts(&self, signal: &[f64], threshold_factor: f64) -> Vec<Artifact> {
        if signal.is_empty() {
            return Vec::new();
        }

        let n = signal.len() as f64;
        let mean = signal.iter().sum::<f64>() / n;
        let variance = signal.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        let std_dev = variance.sqrt();

        if std_dev == 0.0 {
            return Vec::new();
        }

        let threshold = threshold_factor * std_dev;

        let mut artifacts = Vec::new();
        let mut in_artifact = false;
        let mut start = 0usize;
        let mut peak_amp = 0.0f64;

        for (i, &sample) in signal.iter().enumerate() {
            let amp = sample.abs();
            if amp > threshold {
                if !in_artifact {
                    in_artifact = true;
                    start = i;
                    peak_amp = amp;
                } else if amp > peak_amp {
                    peak_amp = amp;
                }
            } else if in_artifact {
                artifacts.push(Artifact {
                    start,
                    end: i,
                    peak_amplitude: peak_amp,
                });
                in_artifact = false;
            }
        }

        // Close trailing artifact
        if in_artifact {
            artifacts.push(Artifact {
                start,
                end: signal.len(),
                peak_amplitude: peak_amp,
            });
        }

        artifacts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> BiomedicalProcessor {
        BiomedicalProcessor::new(ProcessorConfig::default())
    }

    #[test]
    fn test_config_default() {
        let config = ProcessorConfig::default();
        assert_eq!(config.sample_rate, 360.0);
        assert_eq!(config.powerline_freq, 60.0);
        assert_eq!(config.notch_bandwidth, 2.0);
    }

    #[test]
    fn test_remove_powerline_pure_60hz() {
        let proc = default_processor();
        let fs = 360.0;
        let n = 3600; // 10 seconds
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * 60.0 * t).sin()
            })
            .collect();

        let clean = proc.remove_powerline(&signal);
        assert_eq!(clean.len(), n);

        // After the filter settles (skip first 0.5s), residual should be small
        let skip = (0.5 * fs) as usize;
        let max_residual = clean[skip..]
            .iter()
            .cloned()
            .fold(0.0f64, |a, b| a.max(b.abs()));
        assert!(
            max_residual < 0.15,
            "Residual 60 Hz too large: {max_residual}"
        );
    }

    #[test]
    fn test_remove_powerline_preserves_ecg() {
        let proc = default_processor();
        let fs = 360.0;
        let n = 3600;
        // 1 Hz signal (ECG-like fundamental) should pass through
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * 1.0 * t).sin()
            })
            .collect();

        let clean = proc.remove_powerline(&signal);
        // The 1 Hz component should be largely preserved
        let skip = (0.5 * fs) as usize;
        let power_in: f64 = signal[skip..].iter().map(|x| x * x).sum::<f64>();
        let power_out: f64 = clean[skip..].iter().map(|x| x * x).sum::<f64>();
        let ratio = power_out / power_in;
        assert!(
            ratio > 0.85,
            "1 Hz signal attenuated too much: ratio={ratio}"
        );
    }

    #[test]
    fn test_remove_powerline_50hz() {
        let config = ProcessorConfig {
            sample_rate: 500.0,
            powerline_freq: 50.0,
            notch_bandwidth: 2.0,
        };
        let proc = BiomedicalProcessor::new(config);
        let n = 5000;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / 500.0;
                (2.0 * PI * 50.0 * t).sin()
            })
            .collect();

        let clean = proc.remove_powerline(&signal);
        let skip = 500;
        let max_residual = clean[skip..]
            .iter()
            .cloned()
            .fold(0.0f64, |a, b| a.max(b.abs()));
        assert!(
            max_residual < 0.15,
            "Residual 50 Hz too large: {max_residual}"
        );
    }

    #[test]
    fn test_remove_baseline_wander() {
        let proc = default_processor();
        let fs = 360.0;
        let n = 7200; // 20 seconds
        // 0.1 Hz baseline wander + 10 Hz ECG-like
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let baseline = 2.0 * (2.0 * PI * 0.1 * t).sin();
                let ecg = (2.0 * PI * 10.0 * t).sin();
                baseline + ecg
            })
            .collect();

        let corrected = proc.remove_baseline_wander(&signal);
        assert_eq!(corrected.len(), n);

        // After settling, the slow baseline should be largely removed
        let last_5s = &corrected[(n - (5.0 * fs) as usize)..];
        let mean_abs: f64 = last_5s.iter().map(|x| x.abs()).sum::<f64>() / last_5s.len() as f64;
        assert!(
            mean_abs < 1.5,
            "Baseline wander not sufficiently removed: mean_abs={mean_abs}"
        );
    }

    #[test]
    fn test_bandpass_filter() {
        let proc = default_processor();
        let fs = 360.0;
        let n = 3600;

        // Very high frequency (100 Hz) should be attenuated
        let high_freq: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * 100.0 * t).sin()
            })
            .collect();

        let filtered = proc.bandpass_filter(&high_freq);
        let skip = (1.0 * fs) as usize;
        let power_in: f64 = high_freq[skip..].iter().map(|x| x * x).sum::<f64>();
        let power_out: f64 = filtered[skip..].iter().map(|x| x * x).sum::<f64>();
        let ratio = power_out / power_in;
        assert!(
            ratio < 0.3,
            "100 Hz not sufficiently attenuated: ratio={ratio}"
        );
    }

    #[test]
    fn test_detect_qrs_synthetic() {
        let fs = 360.0;
        let duration = 5.0;
        let n = (fs * duration) as usize;
        let beat_interval = 0.8; // 75 BPM

        // Generate synthetic ECG with sharp R-peaks
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let beat_phase = t % beat_interval;
                let peak_center = beat_interval * 0.5;
                let sigma = 0.01;
                let r_peak =
                    2.0 * (-(beat_phase - peak_center).powi(2) / (2.0 * sigma * sigma)).exp();
                r_peak
            })
            .collect();

        let proc = default_processor();
        let detections = proc.detect_qrs(&signal);

        assert!(
            detections.len() >= 3,
            "Expected at least 3 QRS detections, got {}",
            detections.len()
        );
    }

    #[test]
    fn test_compute_heart_rate() {
        let proc = default_processor();

        let detections = vec![
            QrsDetection {
                sample_index: 0,
                rr_interval_ms: None,
                heart_rate_bpm: None,
            },
            QrsDetection {
                sample_index: 288,
                rr_interval_ms: Some(800.0), // 75 BPM
                heart_rate_bpm: Some(75.0),
            },
            QrsDetection {
                sample_index: 576,
                rr_interval_ms: Some(800.0),
                heart_rate_bpm: Some(75.0),
            },
        ];

        let hr = proc.compute_heart_rate(&detections);
        assert!(hr.is_some());
        let hr = hr.unwrap();
        assert!(
            (hr - 75.0).abs() < 0.1,
            "Expected ~75 BPM, got {hr}"
        );
    }

    #[test]
    fn test_compute_heart_rate_insufficient_data() {
        let proc = default_processor();
        let detections = vec![QrsDetection {
            sample_index: 0,
            rr_interval_ms: None,
            heart_rate_bpm: None,
        }];

        assert!(proc.compute_heart_rate(&detections).is_none());
        assert!(proc.compute_heart_rate(&[]).is_none());
    }

    #[test]
    fn test_hrv_metrics() {
        let proc = default_processor();

        let detections = vec![
            QrsDetection {
                sample_index: 0,
                rr_interval_ms: None,
                heart_rate_bpm: None,
            },
            QrsDetection {
                sample_index: 288,
                rr_interval_ms: Some(800.0),
                heart_rate_bpm: Some(75.0),
            },
            QrsDetection {
                sample_index: 590,
                rr_interval_ms: Some(838.9),
                heart_rate_bpm: Some(71.5),
            },
            QrsDetection {
                sample_index: 870,
                rr_interval_ms: Some(777.8),
                heart_rate_bpm: Some(77.1),
            },
            QrsDetection {
                sample_index: 1160,
                rr_interval_ms: Some(805.6),
                heart_rate_bpm: Some(74.5),
            },
        ];

        let hrv = proc.hrv_metrics(&detections);
        assert!(hrv.is_some());
        let hrv = hrv.unwrap();

        assert!(hrv.sdnn_ms > 0.0, "SDNN should be positive");
        assert!(hrv.rmssd_ms > 0.0, "RMSSD should be positive");
        assert!(
            hrv.pnn50_percent >= 0.0 && hrv.pnn50_percent <= 100.0,
            "pNN50 should be 0-100%"
        );
        assert!(hrv.mean_rr_ms > 0.0, "Mean RR should be positive");

        let expected_mean = (800.0 + 838.9 + 777.8 + 805.6) / 4.0;
        assert!(
            (hrv.mean_rr_ms - expected_mean).abs() < 0.1,
            "Mean RR mismatch: {} vs {}",
            hrv.mean_rr_ms,
            expected_mean
        );
    }

    #[test]
    fn test_hrv_insufficient_data() {
        let proc = default_processor();
        let detections = vec![
            QrsDetection {
                sample_index: 0,
                rr_interval_ms: None,
                heart_rate_bpm: None,
            },
            QrsDetection {
                sample_index: 288,
                rr_interval_ms: Some(800.0),
                heart_rate_bpm: Some(75.0),
            },
        ];

        // Only 1 R-R interval, need at least 2 for HRV
        assert!(proc.hrv_metrics(&detections).is_none());
    }

    #[test]
    fn test_detect_artifacts() {
        let proc = default_processor();

        let mut signal = vec![0.0; 1000];
        for i in 0..1000 {
            let t = i as f64 / 360.0;
            signal[i] = 0.5 * (2.0 * PI * 1.0 * t).sin();
        }
        // Insert a large artifact at samples 500-510
        for i in 500..510 {
            signal[i] = 10.0;
        }

        let artifacts = proc.detect_artifacts(&signal, 3.0);
        assert!(
            !artifacts.is_empty(),
            "Should detect at least one artifact"
        );

        let contains_500 = artifacts.iter().any(|a| a.start <= 500 && a.end > 500);
        assert!(contains_500, "Artifact at sample 500 not detected");

        let artifact = artifacts.iter().find(|a| a.start <= 500 && a.end > 500).unwrap();
        assert!(
            (artifact.peak_amplitude - 10.0).abs() < 0.01,
            "Peak amplitude should be 10.0, got {}",
            artifact.peak_amplitude
        );
    }

    #[test]
    fn test_empty_signal_handling() {
        let proc = default_processor();
        assert!(proc.remove_powerline(&[]).is_empty());
        assert!(proc.remove_baseline_wander(&[]).is_empty());
        assert!(proc.bandpass_filter(&[]).is_empty());
        assert!(proc.detect_qrs(&[]).is_empty());
        assert!(proc.detect_artifacts(&[], 3.0).is_empty());
    }

    #[test]
    fn test_detect_artifacts_no_artifacts() {
        let proc = default_processor();
        let signal = vec![1.0; 100];
        let artifacts = proc.detect_artifacts(&signal, 3.0);
        assert!(artifacts.is_empty());
    }
}

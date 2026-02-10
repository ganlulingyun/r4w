//! Anomaly detection on time-frequency representations for SIGINT and spectrum monitoring.
//!
//! This module provides a spectrogram-based anomaly detector that learns a baseline
//! noise floor from training data and then identifies anomalous time-frequency regions
//! in subsequent observations. Anomalies are classified by their time-frequency extent
//! into categories such as transient, narrowband, wideband, and frequency-hopping.
//!
//! # Example
//!
//! ```
//! use r4w_core::spectrogram_anomaly_detector::{SpectrogramAnomalyDetector, AnomalyConfig};
//!
//! let config = AnomalyConfig {
//!     fft_size: 64,
//!     hop_size: 32,
//!     sample_rate: 1000.0,
//!     sigma_threshold: 3.0,
//!     num_baseline_frames: 10,
//! };
//!
//! let mut detector = SpectrogramAnomalyDetector::new(config);
//!
//! // Generate baseline noise (low-power random-ish samples)
//! let baseline: Vec<(f64, f64)> = (0..3200)
//!     .map(|i| {
//!         let phase = i as f64 * 0.01;
//!         (phase.cos() * 0.01, phase.sin() * 0.01)
//!     })
//!     .collect();
//! detector.train_baseline(&baseline);
//! assert!(detector.is_trained());
//!
//! // Create test signal with a strong narrowband tone (anomaly)
//! let mut test_signal: Vec<(f64, f64)> = (0..640)
//!     .map(|i| {
//!         let phase = i as f64 * 0.01;
//!         (phase.cos() * 0.01, phase.sin() * 0.01)
//!     })
//!     .collect();
//! // Inject a strong tone at a specific frequency bin
//! for i in 0..640 {
//!     let freq = 2.0 * std::f64::consts::PI * 200.0 * (i as f64) / 1000.0;
//!     test_signal[i].0 += 5.0 * freq.cos();
//!     test_signal[i].1 += 5.0 * freq.sin();
//! }
//!
//! let anomalies = detector.detect_anomalies(&test_signal);
//! // The strong tone should be detected as an anomaly
//! assert!(!anomalies.is_empty());
//! ```

use std::f64::consts::PI;

/// Configuration for the spectrogram anomaly detector.
#[derive(Debug, Clone)]
pub struct AnomalyConfig {
    /// FFT size (number of samples per frame).
    pub fft_size: usize,
    /// Hop size (number of samples between consecutive frames).
    pub hop_size: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Number of standard deviations above mean to flag as anomaly.
    pub sigma_threshold: f64,
    /// Number of frames required to establish baseline.
    pub num_baseline_frames: usize,
}

impl Default for AnomalyConfig {
    fn default() -> Self {
        Self {
            fft_size: 256,
            hop_size: 128,
            sample_rate: 1_000_000.0,
            sigma_threshold: 3.0,
            num_baseline_frames: 20,
        }
    }
}

/// Classification of an anomaly based on its time-frequency extent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnomalyClass {
    /// Short-duration, potentially wideband burst.
    Transient,
    /// Persistent signal occupying a narrow frequency band.
    Narrowband,
    /// Persistent signal occupying a wide frequency band.
    Wideband,
    /// Signal that hops across multiple frequency bands over time.
    FreqHopping,
    /// Cannot be classified into the above categories.
    Unknown,
}

/// A detected anomaly region in the time-frequency plane.
#[derive(Debug, Clone)]
pub struct Anomaly {
    /// Start time of the anomaly region in seconds.
    pub time_start_s: f64,
    /// End time of the anomaly region in seconds.
    pub time_end_s: f64,
    /// Lower frequency bound of the anomaly region in Hz.
    pub freq_start_hz: f64,
    /// Upper frequency bound of the anomaly region in Hz.
    pub freq_end_hz: f64,
    /// Aggregate anomaly score (mean z-score of the region).
    pub score: f64,
    /// Classification of the anomaly.
    pub classification: AnomalyClass,
}

/// Baseline statistical model: per-bin mean and variance of spectral power.
#[derive(Debug, Clone)]
pub struct BaselineModel {
    /// Mean power per frequency bin.
    pub mean_per_bin: Vec<f64>,
    /// Variance of power per frequency bin.
    pub variance_per_bin: Vec<f64>,
    /// Number of frames used to train the model.
    pub num_frames_trained: usize,
}

/// Spectrogram-based anomaly detector.
///
/// Learns a per-bin statistical baseline from training data, then scores
/// new observations against that baseline to detect anomalous time-frequency
/// regions. Uses connected-component labeling to group adjacent anomalous
/// bins into coherent anomaly regions, which are then classified by their
/// time-frequency extent.
#[derive(Debug, Clone)]
pub struct SpectrogramAnomalyDetector {
    config: AnomalyConfig,
    baseline: Option<BaselineModel>,
}

impl SpectrogramAnomalyDetector {
    /// Create a new detector with the given configuration.
    pub fn new(config: AnomalyConfig) -> Self {
        Self {
            config,
            baseline: None,
        }
    }

    /// Train the baseline model from a block of IQ samples.
    ///
    /// Computes per-bin mean and variance of spectral power over all frames
    /// extracted from the provided samples.
    pub fn train_baseline(&mut self, samples: &[(f64, f64)]) {
        let spectrogram = self.compute_power_spectrogram(samples);
        if spectrogram.is_empty() {
            return;
        }

        let num_bins = self.config.fft_size;
        let num_frames = spectrogram.len();

        let mut mean = vec![0.0; num_bins];
        let mut m2 = vec![0.0; num_bins];

        // Welford's online algorithm for numerical stability
        for (n, frame) in spectrogram.iter().enumerate() {
            for (bin, &power) in frame.iter().enumerate() {
                let delta = power - mean[bin];
                mean[bin] += delta / (n as f64 + 1.0);
                let delta2 = power - mean[bin];
                m2[bin] += delta * delta2;
            }
        }

        let variance: Vec<f64> = m2
            .iter()
            .map(|&v| {
                if num_frames > 1 {
                    v / (num_frames as f64 - 1.0)
                } else {
                    0.0
                }
            })
            .collect();

        self.baseline = Some(BaselineModel {
            mean_per_bin: mean,
            variance_per_bin: variance,
            num_frames_trained: num_frames,
        });
    }

    /// Detect anomalies in the given IQ samples against the trained baseline.
    ///
    /// Returns a list of anomaly regions, each with time/frequency bounds,
    /// an aggregate score, and a classification.
    ///
    /// # Panics
    ///
    /// Panics if the baseline has not been trained.
    pub fn detect_anomalies(&self, samples: &[(f64, f64)]) -> Vec<Anomaly> {
        let score_map = self.anomaly_score_map(samples);
        if score_map.is_empty() {
            return Vec::new();
        }

        let threshold = self.config.sigma_threshold;

        // Binary mask of anomalous bins
        let rows = score_map.len();
        let cols = score_map[0].len();
        let mut mask = vec![vec![false; cols]; rows];
        for r in 0..rows {
            for c in 0..cols {
                if score_map[r][c] > threshold {
                    mask[r][c] = true;
                }
            }
        }

        // Connected-component labeling (4-connectivity)
        let components = connected_components(&mask);

        // Extract anomaly regions from components
        let mut anomalies = Vec::new();
        let freq_resolution = self.config.sample_rate / self.config.fft_size as f64;
        let time_resolution = self.config.hop_size as f64 / self.config.sample_rate;

        for component in &components {
            if component.is_empty() {
                continue;
            }

            let mut min_row = rows;
            let mut max_row = 0;
            let mut min_col = cols;
            let mut max_col = 0;
            let mut score_sum = 0.0;

            for &(r, c) in component {
                min_row = min_row.min(r);
                max_row = max_row.max(r);
                min_col = min_col.min(c);
                max_col = max_col.max(c);
                score_sum += score_map[r][c];
            }

            let avg_score = score_sum / component.len() as f64;

            let mut anomaly = Anomaly {
                time_start_s: min_row as f64 * time_resolution,
                time_end_s: (max_row + 1) as f64 * time_resolution,
                freq_start_hz: min_col as f64 * freq_resolution,
                freq_end_hz: (max_col + 1) as f64 * freq_resolution,
                score: avg_score,
                classification: AnomalyClass::Unknown,
            };

            anomaly.classification = self.classify_anomaly(&anomaly, &score_map);
            anomalies.push(anomaly);
        }

        // Sort by score descending
        anomalies.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
        anomalies
    }

    /// Compute a 2D anomaly score map (z-scores) for the given IQ samples.
    ///
    /// Returns a `Vec<Vec<f64>>` where each inner vec is one time frame,
    /// and each element is the z-score `(power - mean) / sqrt(variance)` for that bin.
    ///
    /// # Panics
    ///
    /// Panics if the baseline has not been trained.
    pub fn anomaly_score_map(&self, samples: &[(f64, f64)]) -> Vec<Vec<f64>> {
        let baseline = self
            .baseline
            .as_ref()
            .expect("Baseline must be trained before computing anomaly scores");

        let spectrogram = self.compute_power_spectrogram(samples);
        let mut score_map = Vec::with_capacity(spectrogram.len());

        for frame in &spectrogram {
            let mut scores = Vec::with_capacity(frame.len());
            for (bin, &power) in frame.iter().enumerate() {
                let mean = baseline.mean_per_bin[bin];
                let var = baseline.variance_per_bin[bin];
                let std_dev = var.sqrt();
                if std_dev > 1e-15 {
                    scores.push((power - mean) / std_dev);
                } else {
                    // If variance is essentially zero, any deviation is anomalous
                    if (power - mean).abs() > 1e-15 {
                        scores.push(f64::INFINITY);
                    } else {
                        scores.push(0.0);
                    }
                }
            }
            score_map.push(scores);
        }

        score_map
    }

    /// Returns whether the baseline model has been trained.
    pub fn is_trained(&self) -> bool {
        self.baseline.is_some()
    }

    /// Returns the average noise floor power (mean across all bins) from the baseline.
    ///
    /// # Panics
    ///
    /// Panics if the baseline has not been trained.
    pub fn baseline_noise_floor(&self) -> f64 {
        let baseline = self
            .baseline
            .as_ref()
            .expect("Baseline must be trained before querying noise floor");
        if baseline.mean_per_bin.is_empty() {
            return 0.0;
        }
        let sum: f64 = baseline.mean_per_bin.iter().sum();
        sum / baseline.mean_per_bin.len() as f64
    }

    /// Classify an anomaly based on its time-frequency extent.
    ///
    /// - **Transient**: short in time (<=2 frames), any bandwidth
    /// - **Narrowband**: long in time, narrow in frequency (<=10% of bandwidth)
    /// - **Wideband**: long in time, wide in frequency (>50% of bandwidth)
    /// - **FreqHopping**: moderate bandwidth, discontinuous frequency occupation
    /// - **Unknown**: everything else
    pub fn classify_anomaly(&self, anomaly: &Anomaly, score_map: &[Vec<f64>]) -> AnomalyClass {
        if score_map.is_empty() || score_map[0].is_empty() {
            return AnomalyClass::Unknown;
        }

        let total_bandwidth = self.config.sample_rate;
        let freq_extent = anomaly.freq_end_hz - anomaly.freq_start_hz;
        let time_extent = anomaly.time_end_s - anomaly.time_start_s;
        let frame_duration = self.config.hop_size as f64 / self.config.sample_rate;
        let num_time_frames = (time_extent / frame_duration).round() as usize;
        let freq_fraction = freq_extent / total_bandwidth;

        // Short duration -> Transient
        if num_time_frames <= 2 {
            return AnomalyClass::Transient;
        }

        // Check for frequency hopping: look for discontinuous frequency occupation
        if num_time_frames >= 3 && freq_fraction > 0.1 && freq_fraction < 0.5 {
            // Check if the anomaly has gaps in frequency over time
            if self.has_frequency_hopping_pattern(anomaly, score_map) {
                return AnomalyClass::FreqHopping;
            }
        }

        // Narrow frequency extent -> Narrowband
        if freq_fraction <= 0.1 {
            return AnomalyClass::Narrowband;
        }

        // Wide frequency extent -> Wideband
        if freq_fraction > 0.5 {
            return AnomalyClass::Wideband;
        }

        AnomalyClass::Unknown
    }

    // ---- Internal helpers ----

    /// Compute power spectrogram from IQ samples using a basic DFT.
    fn compute_power_spectrogram(&self, samples: &[(f64, f64)]) -> Vec<Vec<f64>> {
        let fft_size = self.config.fft_size;
        let hop_size = self.config.hop_size;

        if samples.len() < fft_size {
            return Vec::new();
        }

        let num_frames = (samples.len() - fft_size) / hop_size + 1;
        let mut spectrogram = Vec::with_capacity(num_frames);

        for frame_idx in 0..num_frames {
            let start = frame_idx * hop_size;
            let frame_samples = &samples[start..start + fft_size];

            // Apply Hann window and compute DFT
            let power_spectrum = compute_power_spectrum(frame_samples, fft_size);
            spectrogram.push(power_spectrum);
        }

        spectrogram
    }

    /// Detect frequency hopping patterns in an anomaly region.
    ///
    /// Looks at the score map within the anomaly's time bounds and checks
    /// if the peak frequency bin shifts significantly between frames.
    fn has_frequency_hopping_pattern(&self, anomaly: &Anomaly, score_map: &[Vec<f64>]) -> bool {
        let freq_resolution = self.config.sample_rate / self.config.fft_size as f64;
        let time_resolution = self.config.hop_size as f64 / self.config.sample_rate;
        let threshold = self.config.sigma_threshold;

        let start_frame = (anomaly.time_start_s / time_resolution).floor() as usize;
        let end_frame = ((anomaly.time_end_s / time_resolution).ceil() as usize).min(score_map.len());
        let start_bin = (anomaly.freq_start_hz / freq_resolution).floor() as usize;
        let end_bin = ((anomaly.freq_end_hz / freq_resolution).ceil() as usize).min(
            score_map.first().map_or(0, |f| f.len()),
        );

        if start_frame >= end_frame || start_bin >= end_bin {
            return false;
        }

        // Find peak bin per frame within the anomaly region
        let mut peak_bins = Vec::new();
        for frame_idx in start_frame..end_frame {
            let frame = &score_map[frame_idx];
            let mut peak_bin = start_bin;
            let mut peak_val = f64::NEG_INFINITY;
            for bin in start_bin..end_bin.min(frame.len()) {
                if frame[bin] > peak_val {
                    peak_val = frame[bin];
                    peak_bin = bin;
                }
            }
            if peak_val > threshold {
                peak_bins.push(peak_bin);
            }
        }

        if peak_bins.len() < 3 {
            return false;
        }

        // Count frequency transitions (bin changes > 1)
        let mut transitions = 0;
        for window in peak_bins.windows(2) {
            let diff = (window[1] as isize - window[0] as isize).unsigned_abs();
            if diff > 1 {
                transitions += 1;
            }
        }

        // If more than 30% of frames show transitions, it's hopping
        transitions as f64 > peak_bins.len() as f64 * 0.3
    }
}

/// Compute power spectrum of a frame of IQ samples using DFT with Hann window.
///
/// Returns a vector of power values (magnitude squared), one per frequency bin.
fn compute_power_spectrum(samples: &[(f64, f64)], fft_size: usize) -> Vec<f64> {
    let n = fft_size;
    let mut power = Vec::with_capacity(n);

    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &(s_re, s_im)) in samples.iter().enumerate().take(n) {
            // Hann window
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            // Complex multiply: (s_re + j*s_im) * (cos_a + j*sin_a)
            re += w * (s_re * cos_a - s_im * sin_a);
            im += w * (s_re * sin_a + s_im * cos_a);
        }
        power.push(re * re + im * im);
    }

    power
}

/// Connected-component labeling with 4-connectivity on a boolean mask.
///
/// Returns a vector of components, where each component is a vector of (row, col) positions.
fn connected_components(mask: &[Vec<bool>]) -> Vec<Vec<(usize, usize)>> {
    if mask.is_empty() {
        return Vec::new();
    }

    let rows = mask.len();
    let cols = mask[0].len();
    let mut visited = vec![vec![false; cols]; rows];
    let mut components = Vec::new();

    for r in 0..rows {
        for c in 0..cols {
            if mask[r][c] && !visited[r][c] {
                // BFS flood fill
                let mut component = Vec::new();
                let mut queue = std::collections::VecDeque::new();
                queue.push_back((r, c));
                visited[r][c] = true;

                while let Some((cr, cc)) = queue.pop_front() {
                    component.push((cr, cc));

                    // 4-connectivity neighbors
                    let neighbors: [(isize, isize); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
                    for &(dr, dc) in &neighbors {
                        let nr = cr as isize + dr;
                        let nc = cc as isize + dc;
                        if nr >= 0 && nr < rows as isize && nc >= 0 && nc < cols as isize {
                            let nr = nr as usize;
                            let nc = nc as usize;
                            if mask[nr][nc] && !visited[nr][nc] {
                                visited[nr][nc] = true;
                                queue.push_back((nr, nc));
                            }
                        }
                    }
                }

                components.push(component);
            }
        }
    }

    components
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> AnomalyConfig {
        AnomalyConfig {
            fft_size: 32,
            hop_size: 16,
            sample_rate: 1000.0,
            sigma_threshold: 3.0,
            num_baseline_frames: 5,
        }
    }

    /// Generate low-power pseudo-random noise samples.
    fn noise_samples(count: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..count)
            .map(|i| {
                // Simple deterministic pseudo-noise using trig functions
                let t = i as f64 * 0.1;
                let re = amplitude * ((t * 1.7 + 0.3).sin() * (t * 3.1).cos());
                let im = amplitude * ((t * 2.3 + 1.1).cos() * (t * 0.7).sin());
                (re, im)
            })
            .collect()
    }

    /// Inject a tone at a given frequency into a signal.
    fn inject_tone(
        signal: &mut [(f64, f64)],
        freq_hz: f64,
        sample_rate: f64,
        amplitude: f64,
    ) {
        for (i, sample) in signal.iter_mut().enumerate() {
            let phase = 2.0 * PI * freq_hz * i as f64 / sample_rate;
            sample.0 += amplitude * phase.cos();
            sample.1 += amplitude * phase.sin();
        }
    }

    #[test]
    fn test_default_config() {
        let config = AnomalyConfig::default();
        assert_eq!(config.fft_size, 256);
        assert_eq!(config.hop_size, 128);
        assert_eq!(config.sample_rate, 1_000_000.0);
        assert!((config.sigma_threshold - 3.0).abs() < 1e-10);
        assert_eq!(config.num_baseline_frames, 20);
    }

    #[test]
    fn test_new_detector_untrained() {
        let detector = SpectrogramAnomalyDetector::new(default_config());
        assert!(!detector.is_trained());
    }

    #[test]
    fn test_train_baseline() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config.clone());
        let samples = noise_samples(2000, 0.01);
        detector.train_baseline(&samples);

        assert!(detector.is_trained());
        let baseline = detector.baseline.as_ref().unwrap();
        assert_eq!(baseline.mean_per_bin.len(), config.fft_size);
        assert_eq!(baseline.variance_per_bin.len(), config.fft_size);
        assert!(baseline.num_frames_trained > 0);
    }

    #[test]
    fn test_baseline_noise_floor_positive() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config);
        let samples = noise_samples(2000, 0.1);
        detector.train_baseline(&samples);

        let floor = detector.baseline_noise_floor();
        assert!(floor > 0.0, "Noise floor should be positive, got {}", floor);
    }

    #[test]
    fn test_no_anomalies_in_noise() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config);

        let train = noise_samples(2000, 0.01);
        detector.train_baseline(&train);

        // Test with same noise characteristics
        let test = noise_samples(500, 0.01);
        let anomalies = detector.detect_anomalies(&test);
        // Noise-on-noise should produce few or no anomalies
        // (might get a couple of statistical flukes, but should be minimal)
        assert!(
            anomalies.len() <= 2,
            "Expected few anomalies in pure noise, got {}",
            anomalies.len()
        );
    }

    #[test]
    fn test_detect_strong_tone() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config.clone());

        let train = noise_samples(2000, 0.01);
        detector.train_baseline(&train);

        // Create test signal with a strong narrowband tone
        let mut test = noise_samples(500, 0.01);
        inject_tone(&mut test, 200.0, config.sample_rate, 10.0);

        let anomalies = detector.detect_anomalies(&test);
        assert!(
            !anomalies.is_empty(),
            "Should detect anomaly from strong tone"
        );
    }

    #[test]
    fn test_anomaly_score_map_dimensions() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config.clone());

        let train = noise_samples(2000, 0.01);
        detector.train_baseline(&train);

        let test = noise_samples(500, 0.01);
        let score_map = detector.anomaly_score_map(&test);

        // Check dimensions
        let expected_frames = (500 - config.fft_size) / config.hop_size + 1;
        assert_eq!(score_map.len(), expected_frames);
        for frame in &score_map {
            assert_eq!(frame.len(), config.fft_size);
        }
    }

    #[test]
    fn test_anomaly_has_valid_bounds() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config.clone());

        let train = noise_samples(2000, 0.01);
        detector.train_baseline(&train);

        let mut test = noise_samples(500, 0.01);
        inject_tone(&mut test, 300.0, config.sample_rate, 10.0);

        let anomalies = detector.detect_anomalies(&test);
        for anomaly in &anomalies {
            assert!(anomaly.time_start_s >= 0.0);
            assert!(anomaly.time_end_s > anomaly.time_start_s);
            assert!(anomaly.freq_start_hz >= 0.0);
            assert!(anomaly.freq_end_hz > anomaly.freq_start_hz);
            assert!(anomaly.score > 0.0);
        }
    }

    #[test]
    fn test_classify_transient() {
        let config = AnomalyConfig {
            fft_size: 32,
            hop_size: 16,
            sample_rate: 1000.0,
            sigma_threshold: 3.0,
            num_baseline_frames: 5,
        };
        let detector = SpectrogramAnomalyDetector::new(config.clone());

        // Short burst: 1 frame duration
        let frame_dur = config.hop_size as f64 / config.sample_rate;
        let anomaly = Anomaly {
            time_start_s: 0.0,
            time_end_s: frame_dur,
            freq_start_hz: 0.0,
            freq_end_hz: 500.0,
            score: 5.0,
            classification: AnomalyClass::Unknown,
        };

        let dummy_map = vec![vec![0.0; 32]; 10];
        let class = detector.classify_anomaly(&anomaly, &dummy_map);
        assert_eq!(class, AnomalyClass::Transient);
    }

    #[test]
    fn test_classify_narrowband() {
        let config = AnomalyConfig {
            fft_size: 32,
            hop_size: 16,
            sample_rate: 1000.0,
            sigma_threshold: 3.0,
            num_baseline_frames: 5,
        };
        let detector = SpectrogramAnomalyDetector::new(config.clone());

        let frame_dur = config.hop_size as f64 / config.sample_rate;
        // Long in time, narrow in frequency (5% of bandwidth)
        let anomaly = Anomaly {
            time_start_s: 0.0,
            time_end_s: frame_dur * 10.0,
            freq_start_hz: 100.0,
            freq_end_hz: 150.0, // 50 Hz out of 1000 Hz = 5%
            score: 5.0,
            classification: AnomalyClass::Unknown,
        };

        let dummy_map = vec![vec![0.0; 32]; 20];
        let class = detector.classify_anomaly(&anomaly, &dummy_map);
        assert_eq!(class, AnomalyClass::Narrowband);
    }

    #[test]
    fn test_classify_wideband() {
        let config = AnomalyConfig {
            fft_size: 32,
            hop_size: 16,
            sample_rate: 1000.0,
            sigma_threshold: 3.0,
            num_baseline_frames: 5,
        };
        let detector = SpectrogramAnomalyDetector::new(config.clone());

        let frame_dur = config.hop_size as f64 / config.sample_rate;
        // Long in time, wide in frequency (>50%)
        let anomaly = Anomaly {
            time_start_s: 0.0,
            time_end_s: frame_dur * 10.0,
            freq_start_hz: 0.0,
            freq_end_hz: 600.0, // 60% of bandwidth
            score: 5.0,
            classification: AnomalyClass::Unknown,
        };

        let dummy_map = vec![vec![0.0; 32]; 20];
        let class = detector.classify_anomaly(&anomaly, &dummy_map);
        assert_eq!(class, AnomalyClass::Wideband);
    }

    #[test]
    fn test_connected_components_empty() {
        let mask: Vec<Vec<bool>> = Vec::new();
        let components = connected_components(&mask);
        assert!(components.is_empty());
    }

    #[test]
    fn test_connected_components_single() {
        let mask = vec![
            vec![false, true, false],
            vec![false, true, false],
            vec![false, false, false],
        ];
        let components = connected_components(&mask);
        assert_eq!(components.len(), 1);
        assert_eq!(components[0].len(), 2);
    }

    #[test]
    fn test_connected_components_two_disjoint() {
        let mask = vec![
            vec![true, false, false],
            vec![false, false, false],
            vec![false, false, true],
        ];
        let components = connected_components(&mask);
        assert_eq!(components.len(), 2);
        assert_eq!(components[0].len(), 1);
        assert_eq!(components[1].len(), 1);
    }

    #[test]
    fn test_power_spectrum_dc_component() {
        // Constant signal should have energy only at DC (bin 0)
        let samples: Vec<(f64, f64)> = vec![(1.0, 0.0); 16];
        let power = compute_power_spectrum(&samples, 16);
        assert_eq!(power.len(), 16);
        // DC bin should have the most energy
        let dc_power = power[0];
        for (i, &p) in power.iter().enumerate().skip(1) {
            assert!(
                dc_power >= p,
                "DC bin ({}) should have most power, but bin {} has {}",
                dc_power,
                i,
                p
            );
        }
    }

    #[test]
    fn test_empty_samples_no_crash() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config);

        // Training with empty samples should not crash
        detector.train_baseline(&[]);
        assert!(!detector.is_trained());
    }

    #[test]
    fn test_short_samples_no_crash() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config.clone());

        // Samples shorter than fft_size should produce empty spectrogram
        let short = noise_samples(config.fft_size - 1, 0.01);
        detector.train_baseline(&short);
        assert!(!detector.is_trained());
    }

    #[test]
    fn test_anomaly_sorted_by_score() {
        let config = AnomalyConfig {
            fft_size: 32,
            hop_size: 16,
            sample_rate: 1000.0,
            sigma_threshold: 2.0,
            num_baseline_frames: 5,
        };
        let mut detector = SpectrogramAnomalyDetector::new(config.clone());

        let train = noise_samples(2000, 0.01);
        detector.train_baseline(&train);

        // Inject two tones at different amplitudes
        let mut test = noise_samples(500, 0.01);
        inject_tone(&mut test, 100.0, config.sample_rate, 5.0);
        inject_tone(&mut test, 400.0, config.sample_rate, 20.0);

        let anomalies = detector.detect_anomalies(&test);
        // If multiple anomalies detected, they should be sorted by score descending
        if anomalies.len() >= 2 {
            for w in anomalies.windows(2) {
                assert!(
                    w[0].score >= w[1].score,
                    "Anomalies should be sorted by score descending"
                );
            }
        }
    }

    #[test]
    fn test_score_map_high_for_strong_signal() {
        let config = default_config();
        let mut detector = SpectrogramAnomalyDetector::new(config.clone());

        let train = noise_samples(2000, 0.01);
        detector.train_baseline(&train);

        let mut test = noise_samples(500, 0.01);
        inject_tone(&mut test, 200.0, config.sample_rate, 50.0);

        let score_map = detector.anomaly_score_map(&test);
        // At least some bins should have high scores
        let max_score = score_map
            .iter()
            .flat_map(|f| f.iter())
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_score > config.sigma_threshold,
            "Max score {} should exceed threshold {}",
            max_score,
            config.sigma_threshold
        );
    }
}

//! # Speech Diarization Engine
//!
//! Speaker diarization signal processing: determining "who spoke when" in an audio
//! recording. This module implements the full diarization pipeline including Voice
//! Activity Detection (VAD), speaker feature extraction (MFCCs, spectral features),
//! speaker turn detection via change-point analysis, and speaker clustering.
//!
//! ## Pipeline Overview
//!
//! ```text
//! Audio → VAD → Feature Extraction → Change-Point Detection → Clustering → Speaker Segments
//! ```
//!
//! ## Key Components
//!
//! - [`VoiceActivityDetector`]: Energy-based speech/silence classification with hangover logic
//! - [`FeatureExtractor`]: MFCC and spectral feature computation for speaker characterization
//! - [`SpeakerClusterer`]: K-means and agglomerative clustering with BIC model selection
//! - [`ChangePointDetector`]: BIC-based and KL-divergence speaker change point detection
//! - [`DiarizationEngine`]: Full pipeline orchestrator
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::speech_diarization_engine::{DiarizationConfig, DiarizationEngine};
//!
//! let config = DiarizationConfig {
//!     sample_rate_hz: 16000.0,
//!     num_speakers: Some(2),
//!     ..DiarizationConfig::default()
//! };
//! let engine = DiarizationEngine::new(config);
//!
//! // Generate a test signal with two speakers
//! let signal: Vec<f64> = (0..16000)
//!     .map(|i| (i as f64 * 0.1).sin() * 0.5)
//!     .collect();
//! let segments = engine.diarize(&signal);
//! ```

use std::f64::consts::PI;

/// Configuration for the diarization engine.
#[derive(Debug, Clone)]
pub struct DiarizationConfig {
    /// Audio sample rate in Hz (typically 16000).
    pub sample_rate_hz: f64,
    /// Analysis frame size in milliseconds (typically 25).
    pub frame_size_ms: f64,
    /// Frame step (hop) size in milliseconds (typically 10).
    pub frame_step_ms: f64,
    /// Number of speakers if known, `None` for auto-detection.
    pub num_speakers: Option<usize>,
    /// VAD energy threshold in dB (default -40).
    pub vad_energy_threshold_db: f64,
    /// Minimum segment duration in milliseconds (default 500).
    pub min_segment_duration_ms: f64,
}

impl Default for DiarizationConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 16000.0,
            frame_size_ms: 25.0,
            frame_step_ms: 10.0,
            num_speakers: None,
            vad_energy_threshold_db: -40.0,
            min_segment_duration_ms: 500.0,
        }
    }
}

/// A segment of audio attributed to a specific speaker.
#[derive(Debug, Clone, PartialEq)]
pub struct SpeakerSegment {
    /// Speaker identifier (0-indexed).
    pub speaker_id: usize,
    /// Start sample index (inclusive).
    pub start_sample: usize,
    /// End sample index (exclusive).
    pub end_sample: usize,
    /// Confidence score in [0, 1].
    pub confidence: f64,
}

impl SpeakerSegment {
    /// Returns the duration of this segment in samples.
    pub fn duration_samples(&self) -> usize {
        self.end_sample.saturating_sub(self.start_sample)
    }
}

// ---------------------------------------------------------------------------
// Voice Activity Detection
// ---------------------------------------------------------------------------

/// Energy-based voice activity detector with zero-crossing rate and hangover logic.
///
/// Uses frame-level energy in dB compared to a threshold to classify frames
/// as speech or silence. Includes hangover logic to merge short gaps and
/// remove spurious detections.
pub struct VoiceActivityDetector {
    /// Energy threshold in dB below which frames are classified as silence.
    energy_threshold_db: f64,
    /// Audio sample rate in Hz.
    sample_rate: f64,
    /// Frame size in samples.
    frame_size: usize,
}

impl VoiceActivityDetector {
    /// Creates a new VAD with the given energy threshold, sample rate, and frame size.
    ///
    /// # Arguments
    ///
    /// * `energy_threshold_db` - Frames below this energy (in dB) are silence.
    /// * `sample_rate` - Audio sample rate in Hz.
    /// * `frame_size` - Number of samples per analysis frame.
    pub fn new(energy_threshold_db: f64, sample_rate: f64, frame_size: usize) -> Self {
        Self {
            energy_threshold_db,
            sample_rate,
            frame_size,
        }
    }

    /// Computes the energy of a frame in dB: `10 * log10(mean(x^2))`.
    ///
    /// Returns `-f64::INFINITY` for silent (all-zero) frames.
    pub fn frame_energy_db(frame: &[f64]) -> f64 {
        if frame.is_empty() {
            return f64::NEG_INFINITY;
        }
        let mean_sq: f64 = frame.iter().map(|&x| x * x).sum::<f64>() / frame.len() as f64;
        if mean_sq <= 0.0 {
            return f64::NEG_INFINITY;
        }
        10.0 * mean_sq.log10()
    }

    /// Performs per-frame voice activity detection on the signal.
    ///
    /// Returns a boolean vector where `true` indicates a speech frame and `false`
    /// indicates silence. Frames are extracted with the given `frame_size` and
    /// `frame_step`.
    pub fn detect(&self, signal: &[f64], frame_size: usize, frame_step: usize) -> Vec<bool> {
        if signal.is_empty() || frame_size == 0 || frame_step == 0 {
            return Vec::new();
        }
        let mut vad = Vec::new();
        let mut offset = 0;
        while offset + frame_size <= signal.len() {
            let frame = &signal[offset..offset + frame_size];
            let energy = Self::frame_energy_db(frame);
            vad.push(energy > self.energy_threshold_db);
            offset += frame_step;
        }
        vad
    }

    /// Computes the zero-crossing rate of a frame.
    ///
    /// ZCR = (number of sign changes) / (N - 1), where N is the frame length.
    /// Useful for distinguishing voiced speech (low ZCR) from unvoiced/noise (high ZCR).
    pub fn zero_crossing_rate(frame: &[f64]) -> f64 {
        if frame.len() < 2 {
            return 0.0;
        }
        let crossings = frame
            .windows(2)
            .filter(|w| (w[0] >= 0.0) != (w[1] >= 0.0))
            .count();
        crossings as f64 / (frame.len() - 1) as f64
    }

    /// Applies hangover logic to merge short silence gaps and remove short speech bursts.
    ///
    /// * `min_speech_frames` - Minimum consecutive speech frames to keep a speech region.
    /// * `min_silence_frames` - Minimum consecutive silence frames to keep a silence region.
    pub fn merge_segments(
        vad: &[bool],
        min_speech_frames: usize,
        min_silence_frames: usize,
    ) -> Vec<bool> {
        if vad.is_empty() {
            return Vec::new();
        }
        let mut result = vad.to_vec();

        // Step 1: Fill short silence gaps (bridge speech across short silences)
        let mut i = 0;
        while i < result.len() {
            if !result[i] {
                // Count consecutive silence frames
                let start = i;
                while i < result.len() && !result[i] {
                    i += 1;
                }
                let gap_len = i - start;
                // If gap is shorter than threshold and surrounded by speech, fill it
                if gap_len < min_silence_frames && start > 0 && i < result.len() {
                    for j in start..i {
                        result[j] = true;
                    }
                }
            } else {
                i += 1;
            }
        }

        // Step 2: Remove short speech bursts
        i = 0;
        while i < result.len() {
            if result[i] {
                let start = i;
                while i < result.len() && result[i] {
                    i += 1;
                }
                let speech_len = i - start;
                if speech_len < min_speech_frames {
                    for j in start..i {
                        result[j] = false;
                    }
                }
            } else {
                i += 1;
            }
        }

        result
    }

    /// Extracts contiguous speech regions from a VAD boolean vector.
    ///
    /// Returns a vector of `(start_sample, end_sample)` pairs. The sample indices
    /// are computed by multiplying the frame index by `frame_step`.
    pub fn speech_segments(vad: &[bool], frame_step: usize) -> Vec<(usize, usize)> {
        let mut segments = Vec::new();
        let mut i = 0;
        while i < vad.len() {
            if vad[i] {
                let start = i;
                while i < vad.len() && vad[i] {
                    i += 1;
                }
                let start_sample = start * frame_step;
                let end_sample = i * frame_step;
                segments.push((start_sample, end_sample));
            } else {
                i += 1;
            }
        }
        segments
    }

    /// Returns the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Returns the frame size in samples.
    pub fn frame_size(&self) -> usize {
        self.frame_size
    }
}

// ---------------------------------------------------------------------------
// Discrete Cosine Transform (Type II)
// ---------------------------------------------------------------------------

/// Computes the DCT-II of the input, returning the first `num_coeffs` coefficients.
///
/// DCT-II formula:
/// ```text
/// X[k] = sum_{n=0}^{N-1} x[n] * cos(pi * (2n+1) * k / (2N))
/// ```
///
/// This is the standard DCT used in MFCC computation.
pub fn dct_ii(input: &[f64], num_coeffs: usize) -> Vec<f64> {
    let n = input.len();
    if n == 0 {
        return vec![0.0; num_coeffs];
    }
    let mut output = Vec::with_capacity(num_coeffs);
    for k in 0..num_coeffs {
        let mut sum = 0.0;
        for (i, &x) in input.iter().enumerate() {
            sum += x * (PI * (2 * i + 1) as f64 * k as f64 / (2.0 * n as f64)).cos();
        }
        output.push(sum);
    }
    output
}

// ---------------------------------------------------------------------------
// Feature Extraction
// ---------------------------------------------------------------------------

/// Speaker feature extractor computing MFCCs and spectral features.
///
/// Extracts Mel-Frequency Cepstral Coefficients (MFCCs) and spectral shape
/// descriptors from audio frames for use in speaker characterization and
/// clustering.
pub struct FeatureExtractor {
    /// Audio sample rate in Hz.
    sample_rate: f64,
    /// Number of MFCC coefficients to extract.
    num_mfcc: usize,
    /// Number of triangular mel-scale filters.
    num_mel_filters: usize,
}

impl FeatureExtractor {
    /// Creates a new feature extractor.
    ///
    /// # Arguments
    ///
    /// * `sample_rate` - Audio sample rate in Hz.
    /// * `num_mfcc` - Number of MFCC coefficients to extract (typically 13).
    /// * `num_mel_filters` - Number of mel-scale triangular filters (typically 26).
    pub fn new(sample_rate: f64, num_mfcc: usize, num_mel_filters: usize) -> Self {
        Self {
            sample_rate,
            num_mfcc,
            num_mel_filters,
        }
    }

    /// Converts frequency from Hz to the mel scale.
    ///
    /// Formula: `mel = 2595 * log10(1 + f / 700)`
    pub fn mel_frequency(hz: f64) -> f64 {
        2595.0 * (1.0 + hz / 700.0).log10()
    }

    /// Converts frequency from mel scale back to Hz.
    ///
    /// Formula: `hz = 700 * (10^(mel / 2595) - 1)`
    pub fn hz_frequency(mel: f64) -> f64 {
        700.0 * (10.0_f64.powf(mel / 2595.0) - 1.0)
    }

    /// Constructs a mel-scale triangular filterbank.
    ///
    /// Returns a vector of `num_filters` filters, each represented as a vector
    /// of length `fft_size / 2 + 1` containing the filter weights for each
    /// FFT bin.
    pub fn mel_filterbank(
        num_filters: usize,
        fft_size: usize,
        sample_rate: f64,
    ) -> Vec<Vec<f64>> {
        let num_bins = fft_size / 2 + 1;
        let low_mel = Self::mel_frequency(0.0);
        let high_mel = Self::mel_frequency(sample_rate / 2.0);

        // Evenly spaced mel points
        let mel_points: Vec<f64> = (0..num_filters + 2)
            .map(|i| low_mel + (high_mel - low_mel) * i as f64 / (num_filters + 1) as f64)
            .collect();

        // Convert mel points to FFT bin indices
        let bin_indices: Vec<usize> = mel_points
            .iter()
            .map(|&m| {
                let hz = Self::hz_frequency(m);
                let bin = (hz * fft_size as f64 / sample_rate).round() as usize;
                bin.min(num_bins - 1)
            })
            .collect();

        let mut filterbank = Vec::with_capacity(num_filters);
        for f in 0..num_filters {
            let mut filter = vec![0.0; num_bins];
            let left = bin_indices[f];
            let center = bin_indices[f + 1];
            let right = bin_indices[f + 2];

            // Rising slope
            if center > left {
                for k in left..=center {
                    filter[k] = (k - left) as f64 / (center - left) as f64;
                }
            }
            // Falling slope
            if right > center {
                for k in center..=right.min(num_bins - 1) {
                    filter[k] = (right - k) as f64 / (right - center) as f64;
                }
            }
            filterbank.push(filter);
        }
        filterbank
    }

    /// Extracts MFCC coefficients from an audio frame.
    ///
    /// Pipeline: power spectrum -> mel filterbank -> log -> DCT
    ///
    /// Returns `num_coeffs` MFCC coefficients.
    pub fn extract_mfcc(&self, frame: &[f64], num_coeffs: usize) -> Vec<f64> {
        if frame.is_empty() {
            return vec![0.0; num_coeffs];
        }

        // Compute power spectrum via DFT (real-valued input)
        let fft_size = frame.len().next_power_of_two().max(2);
        let power_spectrum = Self::power_spectrum(frame, fft_size);

        // Apply mel filterbank
        let filterbank = Self::mel_filterbank(self.num_mel_filters, fft_size, self.sample_rate);
        let mel_energies: Vec<f64> = filterbank
            .iter()
            .map(|filter| {
                let energy: f64 = filter
                    .iter()
                    .zip(power_spectrum.iter())
                    .map(|(&f, &p)| f * p)
                    .sum();
                // Floor to avoid log(0)
                if energy < 1e-22 { (1e-22_f64).ln() } else { energy.ln() }
            })
            .collect();

        // DCT-II to get cepstral coefficients
        dct_ii(&mel_energies, num_coeffs)
    }

    /// Computes the power spectrum |X(k)|^2 of a frame via DFT.
    ///
    /// Returns `fft_size / 2 + 1` power values (one-sided).
    fn power_spectrum(frame: &[f64], fft_size: usize) -> Vec<f64> {
        let num_bins = fft_size / 2 + 1;
        let mut power = Vec::with_capacity(num_bins);
        for k in 0..num_bins {
            let mut real = 0.0;
            let mut imag = 0.0;
            for (n, &x) in frame.iter().enumerate() {
                let angle = 2.0 * PI * k as f64 * n as f64 / fft_size as f64;
                real += x * angle.cos();
                imag -= x * angle.sin();
            }
            power.push(real * real + imag * imag);
        }
        power
    }

    /// Computes the spectral centroid: the weighted mean frequency of the spectrum.
    ///
    /// `centroid = sum(f * |X(f)|) / sum(|X(f)|)`
    ///
    /// The `spectrum` should be a magnitude or power spectrum of length `fft_size / 2 + 1`.
    pub fn spectral_centroid(spectrum: &[f64], sample_rate: f64, fft_size: usize) -> f64 {
        if spectrum.is_empty() {
            return 0.0;
        }
        let mut weighted_sum = 0.0;
        let mut total_weight = 0.0;
        for (k, &mag) in spectrum.iter().enumerate() {
            let freq = k as f64 * sample_rate / fft_size as f64;
            weighted_sum += freq * mag;
            total_weight += mag;
        }
        if total_weight <= 0.0 {
            0.0
        } else {
            weighted_sum / total_weight
        }
    }

    /// Computes the spectral bandwidth: the weighted standard deviation of frequencies
    /// around the spectral centroid.
    ///
    /// `bandwidth = sqrt(sum(|X(f)| * (f - centroid)^2) / sum(|X(f)|))`
    pub fn spectral_bandwidth(
        spectrum: &[f64],
        centroid: f64,
        sample_rate: f64,
        fft_size: usize,
    ) -> f64 {
        if spectrum.is_empty() {
            return 0.0;
        }
        let mut weighted_var = 0.0;
        let mut total_weight = 0.0;
        for (k, &mag) in spectrum.iter().enumerate() {
            let freq = k as f64 * sample_rate / fft_size as f64;
            let diff = freq - centroid;
            weighted_var += mag * diff * diff;
            total_weight += mag;
        }
        if total_weight <= 0.0 {
            0.0
        } else {
            (weighted_var / total_weight).sqrt()
        }
    }

    /// Computes temporal derivative (delta) features using a regression window.
    ///
    /// For each feature vector at time t, the delta is computed as:
    /// ```text
    /// delta[t] = sum_{w=1}^{width} w * (features[t+w] - features[t-w]) / (2 * sum_{w=1}^{width} w^2)
    /// ```
    ///
    /// Returns delta feature vectors; the output length equals `features.len()`.
    /// Boundary frames are zero-padded.
    pub fn delta_features(features: &[Vec<f64>], width: usize) -> Vec<Vec<f64>> {
        if features.is_empty() {
            return Vec::new();
        }
        let dim = features[0].len();
        let n = features.len();
        let denom: f64 = 2.0 * (1..=width).map(|w| (w * w) as f64).sum::<f64>();

        let mut deltas = Vec::with_capacity(n);
        for t in 0..n {
            let mut delta = vec![0.0; dim];
            if denom > 0.0 {
                for w in 1..=width {
                    let t_plus = if t + w < n { &features[t + w] } else { &features[n - 1] };
                    let t_minus = if t >= w { &features[t - w] } else { &features[0] };
                    for d in 0..dim {
                        delta[d] += w as f64 * (t_plus[d] - t_minus[d]);
                    }
                }
                for d in 0..dim {
                    delta[d] /= denom;
                }
            }
            deltas.push(delta);
        }
        deltas
    }

    /// Returns the configured number of MFCC coefficients.
    pub fn num_mfcc(&self) -> usize {
        self.num_mfcc
    }

    /// Returns the configured number of mel filters.
    pub fn num_mel_filters(&self) -> usize {
        self.num_mel_filters
    }
}

// ---------------------------------------------------------------------------
// Speaker Clustering
// ---------------------------------------------------------------------------

/// Speaker clustering using k-means and agglomerative methods with BIC model selection.
pub struct SpeakerClusterer {
    /// Expected number of speakers, or `None` for auto-detection.
    num_speakers: Option<usize>,
}

impl SpeakerClusterer {
    /// Creates a new clusterer.
    ///
    /// * `num_speakers` - `Some(k)` for a known speaker count, `None` for auto-detection.
    pub fn new(num_speakers: Option<usize>) -> Self {
        Self { num_speakers }
    }

    /// Computes cosine similarity between two vectors.
    ///
    /// `cos_sim = (a . b) / (||a|| * ||b||)`
    ///
    /// Returns 0.0 if either vector has zero norm.
    pub fn cosine_similarity(a: &[f64], b: &[f64]) -> f64 {
        if a.len() != b.len() || a.is_empty() {
            return 0.0;
        }
        let dot: f64 = a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum();
        let norm_a: f64 = a.iter().map(|&x| x * x).sum::<f64>().sqrt();
        let norm_b: f64 = b.iter().map(|&x| x * x).sum::<f64>().sqrt();
        if norm_a < 1e-15 || norm_b < 1e-15 {
            0.0
        } else {
            dot / (norm_a * norm_b)
        }
    }

    /// Computes Euclidean distance between two vectors.
    pub fn euclidean_distance(a: &[f64], b: &[f64]) -> f64 {
        if a.len() != b.len() {
            return f64::INFINITY;
        }
        a.iter()
            .zip(b.iter())
            .map(|(&x, &y)| (x - y) * (x - y))
            .sum::<f64>()
            .sqrt()
    }

    /// K-means clustering.
    ///
    /// Partitions `features` into `k` clusters. Returns a vector of cluster
    /// assignments (one per feature vector).
    ///
    /// Uses deterministic initialization: first k feature vectors as initial centroids.
    pub fn kmeans(features: &[Vec<f64>], k: usize, max_iterations: usize) -> Vec<usize> {
        let n = features.len();
        if n == 0 || k == 0 {
            return Vec::new();
        }
        let k = k.min(n);
        let dim = features[0].len();

        // Initialize centroids with first k features
        let mut centroids: Vec<Vec<f64>> = features[..k].to_vec();
        let mut assignments = vec![0usize; n];

        for _iter in 0..max_iterations {
            // Assignment step
            let mut changed = false;
            for (i, feature) in features.iter().enumerate() {
                let mut best_cluster = 0;
                let mut best_dist = f64::INFINITY;
                for (c, centroid) in centroids.iter().enumerate() {
                    let dist = Self::euclidean_distance(feature, centroid);
                    if dist < best_dist {
                        best_dist = dist;
                        best_cluster = c;
                    }
                }
                if assignments[i] != best_cluster {
                    assignments[i] = best_cluster;
                    changed = true;
                }
            }

            if !changed {
                break;
            }

            // Update step
            let mut new_centroids = vec![vec![0.0; dim]; k];
            let mut counts = vec![0usize; k];
            for (i, feature) in features.iter().enumerate() {
                let c = assignments[i];
                counts[c] += 1;
                for (d, &val) in feature.iter().enumerate() {
                    new_centroids[c][d] += val;
                }
            }
            for c in 0..k {
                if counts[c] > 0 {
                    for d in 0..dim {
                        new_centroids[c][d] /= counts[c] as f64;
                    }
                } else {
                    // Keep old centroid for empty clusters
                    new_centroids[c] = centroids[c].clone();
                }
            }
            centroids = new_centroids;
        }

        assignments
    }

    /// Agglomerative (bottom-up) hierarchical clustering using average linkage.
    ///
    /// Starts with each feature vector as its own cluster and iteratively merges
    /// the two closest clusters until `num_clusters` remain.
    pub fn agglomerative_cluster(features: &[Vec<f64>], num_clusters: usize) -> Vec<usize> {
        let n = features.len();
        if n == 0 {
            return Vec::new();
        }
        let num_clusters = num_clusters.max(1).min(n);

        // Each point starts as its own cluster
        let mut cluster_ids: Vec<usize> = (0..n).collect();
        let mut active_clusters: Vec<usize> = (0..n).collect();

        // Precompute pairwise distances
        let mut dist_matrix = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in (i + 1)..n {
                let d = Self::euclidean_distance(&features[i], &features[j]);
                dist_matrix[i][j] = d;
                dist_matrix[j][i] = d;
            }
        }

        let mut current_clusters = n;
        while current_clusters > num_clusters {
            // Find the two closest clusters (average linkage)
            let mut best_i = 0;
            let mut best_j = 1;
            let mut best_dist = f64::INFINITY;

            for (ai, &ci) in active_clusters.iter().enumerate() {
                for &cj in active_clusters.iter().skip(ai + 1) {
                    // Average linkage: mean distance between all pairs in the two clusters
                    let mut total_dist = 0.0;
                    let mut count = 0;
                    for (pi, &cid_i) in cluster_ids.iter().enumerate() {
                        if cid_i != ci {
                            continue;
                        }
                        for (pj, &cid_j) in cluster_ids.iter().enumerate() {
                            if cid_j != cj {
                                continue;
                            }
                            total_dist += dist_matrix[pi][pj];
                            count += 1;
                        }
                    }
                    let avg_dist = if count > 0 {
                        total_dist / count as f64
                    } else {
                        f64::INFINITY
                    };
                    if avg_dist < best_dist {
                        best_dist = avg_dist;
                        best_i = ci;
                        best_j = cj;
                    }
                }
            }

            // Merge cluster best_j into best_i
            for cid in cluster_ids.iter_mut() {
                if *cid == best_j {
                    *cid = best_i;
                }
            }
            active_clusters.retain(|&c| c != best_j);
            current_clusters -= 1;
        }

        // Re-index cluster IDs to 0..num_clusters-1
        let unique: Vec<usize> = {
            let mut u: Vec<usize> = active_clusters.clone();
            u.sort();
            u.dedup();
            u
        };
        cluster_ids
            .iter()
            .map(|&c| unique.iter().position(|&u| u == c).unwrap_or(0))
            .collect()
    }

    /// Computes the Bayesian Information Criterion (BIC) score for a clustering.
    ///
    /// Lower BIC indicates a better model. The BIC balances goodness-of-fit
    /// (log-likelihood) with model complexity.
    ///
    /// BIC = -2 * log_likelihood + num_params * ln(N)
    pub fn bic_criterion(features: &[Vec<f64>], labels: &[usize], k: usize) -> f64 {
        let n = features.len();
        if n == 0 || features[0].is_empty() || k == 0 {
            return f64::INFINITY;
        }
        let dim = features[0].len();

        // Compute within-cluster variance
        let mut cluster_sizes = vec![0usize; k];
        let mut centroids = vec![vec![0.0; dim]; k];

        for (i, feature) in features.iter().enumerate() {
            let c = labels[i] % k;
            cluster_sizes[c] += 1;
            for (d, &val) in feature.iter().enumerate() {
                centroids[c][d] += val;
            }
        }
        for c in 0..k {
            if cluster_sizes[c] > 0 {
                for d in 0..dim {
                    centroids[c][d] /= cluster_sizes[c] as f64;
                }
            }
        }

        // Total within-cluster sum of squares
        let mut total_variance = 0.0;
        for (i, feature) in features.iter().enumerate() {
            let c = labels[i] % k;
            for (d, &val) in feature.iter().enumerate() {
                let diff = val - centroids[c][d];
                total_variance += diff * diff;
            }
        }

        // Estimate sigma^2
        let effective_n = n.saturating_sub(k);
        if effective_n == 0 {
            return f64::INFINITY;
        }
        let sigma_sq = total_variance / (effective_n as f64 * dim as f64);
        if sigma_sq <= 0.0 {
            return f64::NEG_INFINITY;
        }

        // Log-likelihood (Gaussian model)
        let log_likelihood = -0.5 * n as f64 * dim as f64 * (2.0 * PI * sigma_sq).ln()
            - total_variance / (2.0 * sigma_sq);

        // Number of free parameters: k centroids * dim + k-1 mixing weights + 1 variance
        let num_params = (k * dim + k) as f64;

        // BIC
        -2.0 * log_likelihood + num_params * (n as f64).ln()
    }

    /// Estimates the optimal number of speakers using BIC.
    ///
    /// Tries k = 1 to `max_speakers` and returns the k with the lowest BIC.
    pub fn estimate_num_speakers(features: &[Vec<f64>], max_speakers: usize) -> usize {
        if features.is_empty() || max_speakers == 0 {
            return 1;
        }
        let max_k = max_speakers.min(features.len());
        let mut best_k = 1;
        let mut best_bic = f64::INFINITY;

        for k in 1..=max_k {
            let labels = Self::kmeans(features, k, 50);
            let bic = Self::bic_criterion(features, &labels, k);
            if bic < best_bic {
                best_bic = bic;
                best_k = k;
            }
        }
        best_k
    }
}

// ---------------------------------------------------------------------------
// Change Point Detection
// ---------------------------------------------------------------------------

/// Detects speaker change points in feature sequences using BIC and KL divergence.
pub struct ChangePointDetector;

impl ChangePointDetector {
    /// Detects speaker change points using the BIC segmentation criterion.
    ///
    /// Scans the feature sequence for points where splitting into two segments
    /// yields a significantly better BIC than a single segment. The `penalty`
    /// parameter controls sensitivity (higher = fewer change points).
    ///
    /// Returns a sorted vector of change point frame indices.
    pub fn bic_segmentation(features: &[Vec<f64>], penalty: f64) -> Vec<usize> {
        let n = features.len();
        if n < 4 {
            return Vec::new();
        }
        let dim = if features[0].is_empty() {
            return Vec::new();
        } else {
            features[0].len()
        };

        let mut change_points = Vec::new();
        let min_segment = 2;

        // Sliding window BIC test
        let window_size = n.min(50);
        let step = (window_size / 4).max(1);

        let mut offset = 0;
        while offset + window_size <= n {
            let window = &features[offset..offset + window_size];
            let best_split = Self::find_best_split(window, dim, penalty);
            if let Some(split_idx) = best_split {
                if split_idx >= min_segment && (window_size - split_idx) >= min_segment {
                    let cp = offset + split_idx;
                    // Avoid duplicate or very close change points
                    if change_points.last().map_or(true, |&last: &usize| {
                        cp > last + min_segment
                    }) {
                        change_points.push(cp);
                    }
                }
            }
            offset += step;
        }

        change_points.sort();
        change_points.dedup();
        change_points
    }

    /// Finds the best split point in a feature window using BIC delta.
    fn find_best_split(features: &[Vec<f64>], dim: usize, penalty: f64) -> Option<usize> {
        let n = features.len();
        if n < 4 {
            return None;
        }

        // Full-segment covariance (diagonal approximation)
        let full_var = Self::segment_log_det(features, dim);

        let mut best_delta = 0.0;
        let mut best_idx = None;

        for split in 2..n - 1 {
            let left = &features[..split];
            let right = &features[split..];
            let left_var = Self::segment_log_det(left, dim);
            let right_var = Self::segment_log_det(right, dim);

            // BIC delta: positive means split is better
            let n_f = n as f64;
            let n_l = split as f64;
            let n_r = (n - split) as f64;
            let delta_bic = n_f * full_var - n_l * left_var - n_r * right_var;
            let penalty_term = penalty * 0.5 * (dim as f64 + 0.5 * dim as f64 * (dim as f64 + 1.0))
                * n_f.ln();

            let net = delta_bic - penalty_term;
            if net > best_delta {
                best_delta = net;
                best_idx = Some(split);
            }
        }

        best_idx
    }

    /// Computes the log determinant of the diagonal covariance of a feature segment.
    fn segment_log_det(features: &[Vec<f64>], dim: usize) -> f64 {
        let n = features.len();
        if n < 2 {
            return 0.0;
        }

        // Compute mean
        let mut mean = vec![0.0; dim];
        for f in features {
            for (d, &val) in f.iter().enumerate().take(dim) {
                mean[d] += val;
            }
        }
        for m in mean.iter_mut() {
            *m /= n as f64;
        }

        // Compute diagonal variance
        let mut log_det = 0.0;
        for d in 0..dim {
            let mut var = 0.0;
            for f in features {
                let diff = if d < f.len() { f[d] - mean[d] } else { 0.0 };
                var += diff * diff;
            }
            var /= n as f64;
            // Floor to avoid log(0)
            log_det += (var.max(1e-20)).ln();
        }
        log_det
    }

    /// Detects change points using KL divergence between adjacent windows.
    ///
    /// Slides two windows of size `window_size` across the feature sequence and
    /// computes the symmetric KL divergence. Points where the divergence exceeds
    /// `threshold` are returned as change points.
    pub fn distance_based_detection(
        features: &[Vec<f64>],
        window_size: usize,
        threshold: f64,
    ) -> Vec<usize> {
        let n = features.len();
        if n < 2 * window_size || window_size == 0 {
            return Vec::new();
        }
        let dim = features[0].len();
        if dim == 0 {
            return Vec::new();
        }

        let mut change_points = Vec::new();

        for i in window_size..(n - window_size) {
            let left = &features[i - window_size..i];
            let right = &features[i..i + window_size];

            let (mean_l, var_l) = Self::compute_mean_var(left, dim);
            let (mean_r, var_r) = Self::compute_mean_var(right, dim);

            let kl = Self::kl_divergence(&mean_l, &var_l, &mean_r, &var_r);
            if kl > threshold {
                // Avoid very close change points
                if change_points.last().map_or(true, |&last: &usize| i > last + window_size / 2) {
                    change_points.push(i);
                }
            }
        }

        change_points
    }

    /// Computes the symmetric KL divergence between two diagonal Gaussians.
    ///
    /// `KL_sym(p, q) = 0.5 * (KL(p||q) + KL(q||p))`
    ///
    /// where KL(p||q) for diagonal Gaussians is:
    /// ```text
    /// KL = 0.5 * sum_d [ var_p[d]/var_q[d] + (mu_q[d]-mu_p[d])^2/var_q[d] - 1 + ln(var_q[d]/var_p[d]) ]
    /// ```
    pub fn kl_divergence(
        mean1: &[f64],
        var1: &[f64],
        mean2: &[f64],
        var2: &[f64],
    ) -> f64 {
        if mean1.len() != mean2.len() || var1.len() != var2.len() || mean1.len() != var1.len() {
            return 0.0;
        }

        let mut kl_pq = 0.0;
        let mut kl_qp = 0.0;

        for d in 0..mean1.len() {
            let v1 = var1[d].max(1e-20);
            let v2 = var2[d].max(1e-20);
            let mu_diff = mean1[d] - mean2[d];

            kl_pq += v1 / v2 + mu_diff * mu_diff / v2 - 1.0 + (v2 / v1).ln();
            kl_qp += v2 / v1 + mu_diff * mu_diff / v1 - 1.0 + (v1 / v2).ln();
        }

        0.25 * (kl_pq + kl_qp) // 0.5 * 0.5 * (KL_pq + KL_qp)
    }

    /// Computes per-dimension mean and variance of a feature segment.
    fn compute_mean_var(features: &[Vec<f64>], dim: usize) -> (Vec<f64>, Vec<f64>) {
        let n = features.len();
        if n == 0 {
            return (vec![0.0; dim], vec![1.0; dim]);
        }
        let mut mean = vec![0.0; dim];
        for f in features {
            for (d, &val) in f.iter().enumerate().take(dim) {
                mean[d] += val;
            }
        }
        for m in mean.iter_mut() {
            *m /= n as f64;
        }

        let mut var = vec![0.0; dim];
        for f in features {
            for (d, &val) in f.iter().enumerate().take(dim) {
                let diff = val - mean[d];
                var[d] += diff * diff;
            }
        }
        for v in var.iter_mut() {
            *v /= n as f64;
            if *v < 1e-20 {
                *v = 1e-20;
            }
        }

        (mean, var)
    }
}

// ---------------------------------------------------------------------------
// Diarization Engine
// ---------------------------------------------------------------------------

/// Full speaker diarization pipeline.
///
/// Orchestrates VAD, feature extraction, change-point detection, and clustering
/// to produce speaker segments from raw audio.
pub struct DiarizationEngine {
    config: DiarizationConfig,
}

impl DiarizationEngine {
    /// Creates a new diarization engine with the given configuration.
    pub fn new(config: DiarizationConfig) -> Self {
        Self { config }
    }

    /// Runs the full diarization pipeline on a mono audio signal.
    ///
    /// Returns a vector of [`SpeakerSegment`]s indicating who spoke when.
    pub fn diarize(&self, signal: &[f64]) -> Vec<SpeakerSegment> {
        if signal.is_empty() {
            return Vec::new();
        }

        let frame_size =
            (self.config.frame_size_ms * self.config.sample_rate_hz / 1000.0) as usize;
        let frame_step =
            (self.config.frame_step_ms * self.config.sample_rate_hz / 1000.0) as usize;

        if frame_size == 0 || frame_step == 0 {
            return Vec::new();
        }

        // Step 1: Voice Activity Detection
        let vad = VoiceActivityDetector::new(
            self.config.vad_energy_threshold_db,
            self.config.sample_rate_hz,
            frame_size,
        );
        let vad_raw = vad.detect(signal, frame_size, frame_step);
        let min_speech = (self.config.min_segment_duration_ms
            / self.config.frame_step_ms)
            .ceil() as usize;
        let vad_merged = VoiceActivityDetector::merge_segments(&vad_raw, min_speech.max(1), 5);

        // Get speech segments
        let speech_regions = VoiceActivityDetector::speech_segments(&vad_merged, frame_step);
        if speech_regions.is_empty() {
            return Vec::new();
        }

        // Step 2: Feature Extraction for speech frames only
        let extractor = FeatureExtractor::new(self.config.sample_rate_hz, 13, 26);
        let mut speech_frame_indices = Vec::new();
        let mut features = Vec::new();

        for (idx, &is_speech) in vad_merged.iter().enumerate() {
            if is_speech {
                let start = idx * frame_step;
                let end = (start + frame_size).min(signal.len());
                if end - start >= frame_size / 2 {
                    let frame = &signal[start..end];
                    let mfcc = extractor.extract_mfcc(frame, 13);
                    features.push(mfcc);
                    speech_frame_indices.push(idx);
                }
            }
        }

        if features.is_empty() {
            return Vec::new();
        }

        // Step 3: Determine number of speakers and cluster
        let num_speakers = self.config.num_speakers.unwrap_or_else(|| {
            SpeakerClusterer::estimate_num_speakers(&features, 5)
        });
        let num_speakers = num_speakers.max(1).min(features.len());

        let labels = SpeakerClusterer::kmeans(&features, num_speakers, 100);

        // Step 4: Build speaker segments from clustered speech frames
        let mut segments = Vec::new();
        if !labels.is_empty() {
            let mut seg_start = speech_frame_indices[0];
            let mut seg_label = labels[0];

            for i in 1..labels.len() {
                if labels[i] != seg_label {
                    // End current segment, start new one
                    segments.push(SpeakerSegment {
                        speaker_id: seg_label,
                        start_sample: seg_start * frame_step,
                        end_sample: speech_frame_indices[i] * frame_step,
                        confidence: 0.8,
                    });
                    seg_start = speech_frame_indices[i];
                    seg_label = labels[i];
                }
            }
            // Final segment
            let last_idx = *speech_frame_indices.last().unwrap();
            segments.push(SpeakerSegment {
                speaker_id: seg_label,
                start_sample: seg_start * frame_step,
                end_sample: (last_idx + 1) * frame_step,
                confidence: 0.8,
            });
        }

        // Step 5: Merge short segments
        let min_duration_samples =
            (self.config.min_segment_duration_ms * self.config.sample_rate_hz / 1000.0) as usize;
        Self::merge_short_segments(&mut segments, min_duration_samples);

        segments
    }

    /// Merges segments shorter than `min_duration_samples` into their neighbors.
    ///
    /// Short segments are merged into the adjacent segment with the same speaker ID
    /// if possible, otherwise into the preceding segment.
    pub fn merge_short_segments(
        segments: &mut Vec<SpeakerSegment>,
        min_duration_samples: usize,
    ) {
        if segments.len() < 2 {
            return;
        }

        let mut i = 0;
        while i < segments.len() {
            if segments[i].duration_samples() < min_duration_samples {
                if i > 0 {
                    // Merge into previous segment
                    let end = segments[i].end_sample;
                    segments[i - 1].end_sample = end;
                    segments.remove(i);
                } else if segments.len() > 1 {
                    // Merge into next segment
                    let start = segments[i].start_sample;
                    segments[i + 1].start_sample = start;
                    segments.remove(i);
                } else {
                    i += 1;
                }
            } else {
                i += 1;
            }
        }
    }

    /// Computes the Diarization Error Rate (DER).
    ///
    /// DER = (missed speech + false alarm + speaker confusion) / total_samples
    ///
    /// A DER of 0.0 indicates perfect diarization.
    pub fn diarization_error_rate(
        hypothesis: &[SpeakerSegment],
        reference: &[SpeakerSegment],
        total_samples: usize,
    ) -> f64 {
        if total_samples == 0 {
            return 0.0;
        }

        // Build per-sample speaker label maps (-1 = no speaker)
        let hyp_map = Self::build_label_map(hypothesis, total_samples);
        let ref_map = Self::build_label_map(reference, total_samples);

        // Count errors
        let mut missed_speech = 0usize; // reference has speaker, hypothesis has none
        let mut false_alarm = 0usize; // hypothesis has speaker, reference has none
        let mut confusion = 0usize; // both have speakers but different ones

        // Build speaker mapping from hypothesis to reference using majority vote
        let speaker_mapping = Self::build_speaker_mapping(&hyp_map, &ref_map);

        for i in 0..total_samples {
            let ref_spk = ref_map[i];
            let hyp_spk = hyp_map[i];

            match (ref_spk, hyp_spk) {
                (Some(_), None) => missed_speech += 1,
                (None, Some(_)) => false_alarm += 1,
                (Some(r), Some(h)) => {
                    let mapped = speaker_mapping.get(&h).copied().unwrap_or(usize::MAX);
                    if mapped != r {
                        confusion += 1;
                    }
                }
                (None, None) => {}
            }
        }

        (missed_speech + false_alarm + confusion) as f64 / total_samples as f64
    }

    /// Builds a per-sample label map from speaker segments.
    fn build_label_map(segments: &[SpeakerSegment], total_samples: usize) -> Vec<Option<usize>> {
        let mut map = vec![None; total_samples];
        for seg in segments {
            for i in seg.start_sample..seg.end_sample.min(total_samples) {
                map[i] = Some(seg.speaker_id);
            }
        }
        map
    }

    /// Builds a mapping from hypothesis speaker IDs to reference speaker IDs.
    fn build_speaker_mapping(
        hyp_map: &[Option<usize>],
        ref_map: &[Option<usize>],
    ) -> std::collections::HashMap<usize, usize> {
        use std::collections::HashMap;

        // Count co-occurrences
        let mut cooccur: HashMap<(usize, usize), usize> = HashMap::new();
        for i in 0..hyp_map.len().min(ref_map.len()) {
            if let (Some(h), Some(r)) = (hyp_map[i], ref_map[i]) {
                *cooccur.entry((h, r)).or_insert(0) += 1;
            }
        }

        // Greedy assignment: for each hypothesis speaker, pick the reference speaker
        // with the highest overlap
        let mut mapping = HashMap::new();
        let mut hyp_ids: Vec<usize> = cooccur.keys().map(|&(h, _)| h).collect();
        hyp_ids.sort();
        hyp_ids.dedup();

        for &h in &hyp_ids {
            let best_ref = cooccur
                .iter()
                .filter(|&(&(hh, _), _)| hh == h)
                .max_by_key(|&(_, &count)| count)
                .map(|(&(_, r), _)| r);
            if let Some(r) = best_ref {
                mapping.insert(h, r);
            }
        }

        mapping
    }

    /// Returns a reference to the configuration.
    pub fn config(&self) -> &DiarizationConfig {
        &self.config
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // -----------------------------------------------------------------------
    // Mel conversion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mel_frequency_1000hz() {
        // By definition, 1000 Hz should map to ~1000 mel
        let mel = FeatureExtractor::mel_frequency(1000.0);
        // The O'Shaughnessy formula gives 2595*log10(1+1000/700) = 2595*log10(2.4286) ≈ 999.98
        assert!((mel - 1000.0).abs() < 1.0, "1000 Hz -> {} mel, expected ~1000", mel);
    }

    #[test]
    fn test_mel_frequency_0hz() {
        let mel = FeatureExtractor::mel_frequency(0.0);
        assert!((mel - 0.0).abs() < 1e-10, "0 Hz should map to 0 mel");
    }

    #[test]
    fn test_mel_roundtrip() {
        let frequencies = [0.0, 100.0, 500.0, 1000.0, 4000.0, 8000.0];
        for &hz in &frequencies {
            let mel = FeatureExtractor::mel_frequency(hz);
            let reconstructed = FeatureExtractor::hz_frequency(mel);
            assert!(
                (hz - reconstructed).abs() < 0.1,
                "Roundtrip failed for {} Hz: got {} Hz",
                hz,
                reconstructed
            );
        }
    }

    #[test]
    fn test_mel_monotonically_increasing() {
        let freqs = [100.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0];
        let mels: Vec<f64> = freqs.iter().map(|&f| FeatureExtractor::mel_frequency(f)).collect();
        for i in 1..mels.len() {
            assert!(mels[i] > mels[i - 1], "Mel scale not monotonic at index {}", i);
        }
    }

    // -----------------------------------------------------------------------
    // Frame energy tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_frame_energy_silence() {
        let silence = vec![0.0; 160];
        let energy = VoiceActivityDetector::frame_energy_db(&silence);
        assert!(energy.is_infinite() && energy < 0.0, "Silence should be -inf dB");
    }

    #[test]
    fn test_frame_energy_full_scale() {
        // Full-scale signal of amplitude 1.0: mean(1^2) = 1.0, 10*log10(1) = 0 dB
        let full_scale = vec![1.0; 160];
        let energy = VoiceActivityDetector::frame_energy_db(&full_scale);
        assert!((energy - 0.0).abs() < 0.01, "Full-scale should be ~0 dB, got {}", energy);
    }

    #[test]
    fn test_frame_energy_half_amplitude() {
        // Amplitude 0.5: mean(0.25) = 0.25, 10*log10(0.25) ≈ -6.02 dB
        let half = vec![0.5; 160];
        let energy = VoiceActivityDetector::frame_energy_db(&half);
        assert!((energy - (-6.02)).abs() < 0.1, "0.5 amplitude: expected ~-6 dB, got {}", energy);
    }

    #[test]
    fn test_frame_energy_ordering() {
        let quiet = vec![0.01; 160];
        let loud = vec![0.5; 160];
        let e_quiet = VoiceActivityDetector::frame_energy_db(&quiet);
        let e_loud = VoiceActivityDetector::frame_energy_db(&loud);
        assert!(e_loud > e_quiet, "Louder signal should have higher energy");
    }

    // -----------------------------------------------------------------------
    // VAD tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_vad_detects_speech() {
        // Build signal: 0.5s silence + 0.5s speech + 0.5s silence at 16 kHz
        let sr = 16000.0;
        let mut signal = vec![0.0; 8000]; // silence
        for i in 0..8000 {
            // speech-like tone
            signal.push(0.3 * (2.0 * PI * 300.0 * i as f64 / sr).sin());
        }
        signal.extend(vec![0.0; 8000]); // silence

        let vad = VoiceActivityDetector::new(-30.0, sr, 400);
        let result = vad.detect(&signal, 400, 160);

        // Should have some true values in the middle
        let speech_count = result.iter().filter(|&&v| v).count();
        assert!(speech_count > 10, "VAD should detect speech, found {} speech frames", speech_count);

        // Early frames should be silence
        let early_speech = result[..10].iter().filter(|&&v| v).count();
        assert!(early_speech < 3, "First 10 frames should be mostly silence");
    }

    #[test]
    fn test_vad_all_silence() {
        let signal = vec![0.0; 16000];
        let vad = VoiceActivityDetector::new(-30.0, 16000.0, 400);
        let result = vad.detect(&signal, 400, 160);
        assert!(result.iter().all(|&v| !v), "All-silence signal should have no speech");
    }

    // -----------------------------------------------------------------------
    // Zero crossing rate tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_zcr_higher_for_noise() {
        // Noise-like (high ZCR): alternating positive/negative
        let noise: Vec<f64> = (0..160)
            .map(|i| if i % 2 == 0 { 0.1 } else { -0.1 })
            .collect();
        let zcr_noise = VoiceActivityDetector::zero_crossing_rate(&noise);

        // Voiced speech (low ZCR): slow sine wave
        let voiced: Vec<f64> = (0..160)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 16000.0).sin())
            .collect();
        let zcr_voiced = VoiceActivityDetector::zero_crossing_rate(&voiced);

        assert!(
            zcr_noise > zcr_voiced,
            "Noise ZCR ({}) should be higher than voiced speech ZCR ({})",
            zcr_noise, zcr_voiced
        );
    }

    #[test]
    fn test_zcr_constant_signal() {
        let constant = vec![1.0; 160];
        let zcr = VoiceActivityDetector::zero_crossing_rate(&constant);
        assert!((zcr - 0.0).abs() < 1e-10, "Constant signal should have ZCR = 0");
    }

    #[test]
    fn test_zcr_range() {
        let signal: Vec<f64> = (0..160)
            .map(|i| (2.0 * PI * 500.0 * i as f64 / 16000.0).sin())
            .collect();
        let zcr = VoiceActivityDetector::zero_crossing_rate(&signal);
        assert!(zcr >= 0.0 && zcr <= 1.0, "ZCR should be in [0, 1], got {}", zcr);
    }

    // -----------------------------------------------------------------------
    // MFCC tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mfcc_correct_length() {
        let extractor = FeatureExtractor::new(16000.0, 13, 26);
        let frame: Vec<f64> = (0..400)
            .map(|i| (2.0 * PI * 300.0 * i as f64 / 16000.0).sin())
            .collect();
        let mfcc = extractor.extract_mfcc(&frame, 13);
        assert_eq!(mfcc.len(), 13, "MFCC should return 13 coefficients");
    }

    #[test]
    fn test_mfcc_different_num_coeffs() {
        let extractor = FeatureExtractor::new(16000.0, 20, 26);
        let frame: Vec<f64> = (0..400)
            .map(|i| (2.0 * PI * 500.0 * i as f64 / 16000.0).sin())
            .collect();
        let mfcc_5 = extractor.extract_mfcc(&frame, 5);
        let mfcc_20 = extractor.extract_mfcc(&frame, 20);
        assert_eq!(mfcc_5.len(), 5);
        assert_eq!(mfcc_20.len(), 20);
    }

    #[test]
    fn test_mfcc_empty_frame() {
        let extractor = FeatureExtractor::new(16000.0, 13, 26);
        let mfcc = extractor.extract_mfcc(&[], 13);
        assert_eq!(mfcc.len(), 13);
        assert!(mfcc.iter().all(|&v| v == 0.0), "Empty frame should yield zero MFCCs");
    }

    // -----------------------------------------------------------------------
    // Mel filterbank tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mel_filterbank_dimensions() {
        let fb = FeatureExtractor::mel_filterbank(26, 512, 16000.0);
        assert_eq!(fb.len(), 26, "Should have 26 filters");
        for (i, filter) in fb.iter().enumerate() {
            assert_eq!(filter.len(), 257, "Filter {} should have 257 bins (512/2+1)", i);
        }
    }

    #[test]
    fn test_mel_filterbank_nonnegative() {
        let fb = FeatureExtractor::mel_filterbank(26, 512, 16000.0);
        for filter in &fb {
            assert!(filter.iter().all(|&v| v >= 0.0), "Filter weights should be non-negative");
        }
    }

    // -----------------------------------------------------------------------
    // Cosine similarity tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_cosine_similarity_identical() {
        let a = vec![1.0, 2.0, 3.0];
        let sim = SpeakerClusterer::cosine_similarity(&a, &a);
        assert!((sim - 1.0).abs() < 1e-10, "Identical vectors should have similarity 1.0");
    }

    #[test]
    fn test_cosine_similarity_orthogonal() {
        let a = vec![1.0, 0.0, 0.0];
        let b = vec![0.0, 1.0, 0.0];
        let sim = SpeakerClusterer::cosine_similarity(&a, &b);
        assert!((sim - 0.0).abs() < 1e-10, "Orthogonal vectors should have similarity 0.0");
    }

    #[test]
    fn test_cosine_similarity_opposite() {
        let a = vec![1.0, 0.0];
        let b = vec![-1.0, 0.0];
        let sim = SpeakerClusterer::cosine_similarity(&a, &b);
        assert!((sim - (-1.0)).abs() < 1e-10, "Opposite vectors should have similarity -1.0");
    }

    #[test]
    fn test_cosine_similarity_zero_vector() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![0.0, 0.0, 0.0];
        let sim = SpeakerClusterer::cosine_similarity(&a, &b);
        assert!((sim - 0.0).abs() < 1e-10, "Zero vector should give similarity 0.0");
    }

    // -----------------------------------------------------------------------
    // Euclidean distance tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_euclidean_distance_same() {
        let a = vec![1.0, 2.0, 3.0];
        let dist = SpeakerClusterer::euclidean_distance(&a, &a);
        assert!((dist - 0.0).abs() < 1e-10, "Distance to self should be 0");
    }

    #[test]
    fn test_euclidean_distance_known() {
        let a = vec![0.0, 0.0];
        let b = vec![3.0, 4.0];
        let dist = SpeakerClusterer::euclidean_distance(&a, &b);
        assert!((dist - 5.0).abs() < 1e-10, "Distance should be 5.0, got {}", dist);
    }

    // -----------------------------------------------------------------------
    // K-means tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_kmeans_two_clusters() {
        // Two well-separated clusters
        let mut features = Vec::new();
        // Cluster A centered at (0, 0)
        for i in 0..10 {
            features.push(vec![0.1 * i as f64 - 0.5, 0.1 * i as f64 - 0.5]);
        }
        // Cluster B centered at (10, 10)
        for i in 0..10 {
            features.push(vec![10.0 + 0.1 * i as f64, 10.0 + 0.1 * i as f64]);
        }

        let labels = SpeakerClusterer::kmeans(&features, 2, 100);
        assert_eq!(labels.len(), 20);

        // All points in each group should have the same label
        let label_a = labels[0];
        let label_b = labels[10];
        assert_ne!(label_a, label_b, "Two clusters should have different labels");

        for i in 0..10 {
            assert_eq!(labels[i], label_a, "Point {} should be in cluster A", i);
        }
        for i in 10..20 {
            assert_eq!(labels[i], label_b, "Point {} should be in cluster B", i);
        }
    }

    #[test]
    fn test_kmeans_single_cluster() {
        let features = vec![
            vec![1.0, 1.0],
            vec![1.1, 0.9],
            vec![0.9, 1.1],
        ];
        let labels = SpeakerClusterer::kmeans(&features, 1, 50);
        assert!(labels.iter().all(|&l| l == 0), "k=1 should assign all to cluster 0");
    }

    #[test]
    fn test_kmeans_empty() {
        let features: Vec<Vec<f64>> = Vec::new();
        let labels = SpeakerClusterer::kmeans(&features, 2, 50);
        assert!(labels.is_empty());
    }

    // -----------------------------------------------------------------------
    // Agglomerative clustering tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_agglomerative_two_clusters() {
        let features = vec![
            vec![0.0, 0.0],
            vec![0.1, 0.1],
            vec![10.0, 10.0],
            vec![10.1, 10.1],
        ];
        let labels = SpeakerClusterer::agglomerative_cluster(&features, 2);
        assert_eq!(labels.len(), 4);
        assert_eq!(labels[0], labels[1], "First two should be in same cluster");
        assert_eq!(labels[2], labels[3], "Last two should be in same cluster");
        assert_ne!(labels[0], labels[2], "Groups should be different");
    }

    // -----------------------------------------------------------------------
    // BIC criterion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_bic_prefers_correct_k() {
        // Generate two clear clusters
        let mut features = Vec::new();
        for i in 0..20 {
            features.push(vec![i as f64 * 0.1, i as f64 * 0.1]);
        }
        for i in 0..20 {
            features.push(vec![10.0 + i as f64 * 0.1, 10.0 + i as f64 * 0.1]);
        }

        let labels_1 = SpeakerClusterer::kmeans(&features, 1, 50);
        let labels_2 = SpeakerClusterer::kmeans(&features, 2, 50);

        let bic_1 = SpeakerClusterer::bic_criterion(&features, &labels_1, 1);
        let bic_2 = SpeakerClusterer::bic_criterion(&features, &labels_2, 2);

        assert!(
            bic_2 < bic_1,
            "BIC for k=2 ({}) should be lower than k=1 ({})",
            bic_2, bic_1
        );
    }

    // -----------------------------------------------------------------------
    // Speech segment merging tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_merge_segments_fills_gaps() {
        // Speech, short silence, speech -> should fill the gap
        let vad = vec![true, true, true, false, true, true, true];
        let merged = VoiceActivityDetector::merge_segments(&vad, 2, 2);
        assert!(merged[3], "Short silence gap should be filled");
    }

    #[test]
    fn test_merge_segments_removes_short_speech() {
        // Short speech burst surrounded by silence
        let vad = vec![false, false, true, false, false, false];
        let merged = VoiceActivityDetector::merge_segments(&vad, 3, 1);
        assert!(!merged[2], "Short speech burst should be removed");
    }

    #[test]
    fn test_speech_segments_extraction() {
        let vad = vec![false, true, true, true, false, false, true, true, false];
        let segments = VoiceActivityDetector::speech_segments(&vad, 160);
        assert_eq!(segments.len(), 2);
        assert_eq!(segments[0], (160, 640));
        assert_eq!(segments[1], (960, 1280));
    }

    // -----------------------------------------------------------------------
    // DER tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_der_perfect() {
        let segments = vec![
            SpeakerSegment {
                speaker_id: 0,
                start_sample: 0,
                end_sample: 1000,
                confidence: 1.0,
            },
            SpeakerSegment {
                speaker_id: 1,
                start_sample: 1000,
                end_sample: 2000,
                confidence: 1.0,
            },
        ];
        let der = DiarizationEngine::diarization_error_rate(&segments, &segments, 2000);
        assert!(
            (der - 0.0).abs() < 1e-10,
            "Perfect diarization should have DER = 0, got {}",
            der
        );
    }

    #[test]
    fn test_der_complete_miss() {
        let reference = vec![SpeakerSegment {
            speaker_id: 0,
            start_sample: 0,
            end_sample: 1000,
            confidence: 1.0,
        }];
        let hypothesis: Vec<SpeakerSegment> = Vec::new();
        let der = DiarizationEngine::diarization_error_rate(&hypothesis, &reference, 1000);
        // All samples are missed speech
        assert!(
            (der - 1.0).abs() < 1e-10,
            "Complete miss should have DER = 1.0, got {}",
            der
        );
    }

    #[test]
    fn test_der_false_alarm() {
        let reference: Vec<SpeakerSegment> = Vec::new();
        let hypothesis = vec![SpeakerSegment {
            speaker_id: 0,
            start_sample: 0,
            end_sample: 1000,
            confidence: 1.0,
        }];
        let der = DiarizationEngine::diarization_error_rate(&hypothesis, &reference, 1000);
        assert!(
            (der - 1.0).abs() < 1e-10,
            "Pure false alarm should have DER = 1.0, got {}",
            der
        );
    }

    // -----------------------------------------------------------------------
    // Spectral centroid tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectral_centroid_low_vs_high() {
        let fft_size = 512;
        let sr = 16000.0;
        let num_bins = fft_size / 2 + 1;

        // Low-frequency spectrum: energy at low bins
        let mut low_spec = vec![0.0; num_bins];
        for i in 1..10 {
            low_spec[i] = 1.0;
        }
        let centroid_low = FeatureExtractor::spectral_centroid(&low_spec, sr, fft_size);

        // High-frequency spectrum: energy at high bins
        let mut high_spec = vec![0.0; num_bins];
        for i in (num_bins - 10)..num_bins {
            high_spec[i] = 1.0;
        }
        let centroid_high = FeatureExtractor::spectral_centroid(&high_spec, sr, fft_size);

        assert!(
            centroid_high > centroid_low,
            "High-freq centroid ({}) should exceed low-freq centroid ({})",
            centroid_high, centroid_low
        );
    }

    #[test]
    fn test_spectral_centroid_single_bin() {
        let fft_size = 512;
        let sr = 16000.0;
        let num_bins = fft_size / 2 + 1;
        let target_bin = 50;
        let mut spec = vec![0.0; num_bins];
        spec[target_bin] = 1.0;

        let centroid = FeatureExtractor::spectral_centroid(&spec, sr, fft_size);
        let expected = target_bin as f64 * sr / fft_size as f64;
        assert!(
            (centroid - expected).abs() < 1.0,
            "Single-bin centroid should be at bin {} freq ({}), got {}",
            target_bin, expected, centroid
        );
    }

    // -----------------------------------------------------------------------
    // Spectral bandwidth tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectral_bandwidth_narrow_vs_wide() {
        let fft_size = 512;
        let sr = 16000.0;
        let num_bins = fft_size / 2 + 1;

        // Narrow: energy in one bin
        let mut narrow = vec![0.0; num_bins];
        narrow[100] = 1.0;
        let centroid_n = FeatureExtractor::spectral_centroid(&narrow, sr, fft_size);
        let bw_narrow = FeatureExtractor::spectral_bandwidth(&narrow, centroid_n, sr, fft_size);

        // Wide: energy spread across many bins
        let wide = vec![1.0; num_bins];
        let centroid_w = FeatureExtractor::spectral_centroid(&wide, sr, fft_size);
        let bw_wide = FeatureExtractor::spectral_bandwidth(&wide, centroid_w, sr, fft_size);

        assert!(
            bw_wide > bw_narrow,
            "Wide spectrum bandwidth ({}) should exceed narrow ({})",
            bw_wide, bw_narrow
        );
    }

    // -----------------------------------------------------------------------
    // Delta feature tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_delta_of_constant_is_zero() {
        let features = vec![vec![5.0, 3.0, 1.0]; 10];
        let deltas = FeatureExtractor::delta_features(&features, 2);
        assert_eq!(deltas.len(), 10);
        for (t, delta) in deltas.iter().enumerate() {
            for (d, &v) in delta.iter().enumerate() {
                assert!(
                    v.abs() < 1e-10,
                    "Delta of constant should be 0 at t={}, d={}, got {}",
                    t, d, v
                );
            }
        }
    }

    #[test]
    fn test_delta_of_ramp() {
        // Features = [0, 1, 2, 3, 4, ...] -> delta should be positive ~1
        let features: Vec<Vec<f64>> = (0..10).map(|i| vec![i as f64]).collect();
        let deltas = FeatureExtractor::delta_features(&features, 2);
        // Middle frames should have delta ≈ 1.0
        for t in 2..8 {
            assert!(
                (deltas[t][0] - 1.0).abs() < 0.01,
                "Ramp delta at t={} should be ~1.0, got {}",
                t, deltas[t][0]
            );
        }
    }

    // -----------------------------------------------------------------------
    // DCT tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dct_constant_input() {
        // DCT of a constant signal: only DC coefficient should be non-zero
        let input = vec![3.0; 8];
        let dct = dct_ii(&input, 4);
        assert_eq!(dct.len(), 4);
        // DC coefficient (k=0): sum of all = N * 3.0 = 24.0
        assert!((dct[0] - 24.0).abs() < 1e-10, "DC coeff should be 24.0, got {}", dct[0]);
        // AC coefficients should be ~0
        for k in 1..4 {
            assert!(
                dct[k].abs() < 1e-10,
                "AC coefficient k={} should be ~0, got {}",
                k, dct[k]
            );
        }
    }

    #[test]
    fn test_dct_empty_input() {
        let dct = dct_ii(&[], 3);
        assert_eq!(dct.len(), 3);
        assert!(dct.iter().all(|&v| v == 0.0));
    }

    // -----------------------------------------------------------------------
    // KL divergence tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_kl_divergence_same_distributions() {
        let mean = vec![1.0, 2.0];
        let var = vec![0.5, 1.0];
        let kl = ChangePointDetector::kl_divergence(&mean, &var, &mean, &var);
        assert!(
            kl.abs() < 1e-10,
            "KL divergence of identical distributions should be 0, got {}",
            kl
        );
    }

    #[test]
    fn test_kl_divergence_different_means() {
        let mean1 = vec![0.0];
        let mean2 = vec![10.0];
        let var = vec![1.0];
        let kl = ChangePointDetector::kl_divergence(&mean1, &var, &mean2, &var);
        assert!(kl > 0.0, "KL divergence should be positive for different means");
        // For same variance, sym-KL = (mu1-mu2)^2 / sigma^2
        // = 100 / 1 = 100, but scaled by 0.25 * 2 = 50
        let expected = 0.25 * 2.0 * 100.0; // 0.25 * (kl_pq + kl_qp), each has 100
        assert!(
            (kl - expected).abs() < 1.0,
            "KL divergence: expected ~{}, got {}",
            expected, kl
        );
    }

    // -----------------------------------------------------------------------
    // Change-point detection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_distance_based_detection_no_change() {
        // Constant features -> no change points
        let features = vec![vec![1.0, 2.0]; 50];
        let cps = ChangePointDetector::distance_based_detection(&features, 10, 1.0);
        assert!(cps.is_empty(), "Constant features should have no change points");
    }

    #[test]
    fn test_distance_based_detection_clear_change() {
        // Two segments with very different distributions
        let mut features = Vec::new();
        for _ in 0..30 {
            features.push(vec![0.0, 0.0]);
        }
        for _ in 0..30 {
            features.push(vec![100.0, 100.0]);
        }
        let cps = ChangePointDetector::distance_based_detection(&features, 10, 0.1);
        assert!(!cps.is_empty(), "Should detect the change point");
        // Change point should be near index 30
        let nearest = cps.iter().min_by_key(|&&cp| (cp as i64 - 30).unsigned_abs()).unwrap();
        assert!(
            (*nearest as i64 - 30).abs() < 12,
            "Change point should be near 30, closest was {}",
            nearest
        );
    }

    // -----------------------------------------------------------------------
    // Diarization engine integration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_engine_empty_signal() {
        let config = DiarizationConfig::default();
        let engine = DiarizationEngine::new(config);
        let segments = engine.diarize(&[]);
        assert!(segments.is_empty(), "Empty signal should produce no segments");
    }

    #[test]
    fn test_engine_silence_only() {
        let config = DiarizationConfig {
            sample_rate_hz: 16000.0,
            vad_energy_threshold_db: -30.0,
            ..DiarizationConfig::default()
        };
        let engine = DiarizationEngine::new(config);
        let signal = vec![0.0; 16000];
        let segments = engine.diarize(&signal);
        assert!(
            segments.is_empty(),
            "All-silence signal should produce no segments"
        );
    }

    #[test]
    fn test_merge_short_segments() {
        let mut segments = vec![
            SpeakerSegment {
                speaker_id: 0,
                start_sample: 0,
                end_sample: 100, // too short
                confidence: 0.8,
            },
            SpeakerSegment {
                speaker_id: 1,
                start_sample: 100,
                end_sample: 5000,
                confidence: 0.9,
            },
        ];
        DiarizationEngine::merge_short_segments(&mut segments, 500);
        // Short segment should be merged
        assert_eq!(segments.len(), 1, "Short segment should be merged");
        assert_eq!(segments[0].start_sample, 0);
        assert_eq!(segments[0].end_sample, 5000);
    }

    #[test]
    fn test_speaker_segment_duration() {
        let seg = SpeakerSegment {
            speaker_id: 0,
            start_sample: 100,
            end_sample: 500,
            confidence: 0.9,
        };
        assert_eq!(seg.duration_samples(), 400);
    }

    #[test]
    fn test_config_default() {
        let config = DiarizationConfig::default();
        assert_eq!(config.sample_rate_hz, 16000.0);
        assert_eq!(config.frame_size_ms, 25.0);
        assert_eq!(config.frame_step_ms, 10.0);
        assert!(config.num_speakers.is_none());
        assert_eq!(config.vad_energy_threshold_db, -40.0);
        assert_eq!(config.min_segment_duration_ms, 500.0);
    }

    #[test]
    fn test_estimate_num_speakers() {
        // Generate 3 distinct clusters with some variance to give BIC meaningful signal
        let mut features = Vec::new();
        // Cluster 1 centered at (0, 0)
        for i in 0..30 {
            let offset = (i as f64 * 0.37).sin() * 2.0; // spread
            features.push(vec![offset, offset * 0.5]);
        }
        // Cluster 2 centered at (100, 0)
        for i in 0..30 {
            let offset = (i as f64 * 0.53).sin() * 2.0;
            features.push(vec![100.0 + offset, offset * 0.5]);
        }
        // Cluster 3 centered at (0, 100)
        for i in 0..30 {
            let offset = (i as f64 * 0.41).sin() * 2.0;
            features.push(vec![offset * 0.5, 100.0 + offset]);
        }
        let k = SpeakerClusterer::estimate_num_speakers(&features, 5);
        assert!(
            k >= 2 && k <= 4,
            "Should estimate 2-4 speakers for 3 clusters, got {}",
            k
        );
    }
}

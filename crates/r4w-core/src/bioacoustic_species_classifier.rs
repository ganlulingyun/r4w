//! Bioacoustic species classifier for wildlife vocalization analysis.
//!
//! This module classifies wildlife vocalizations (birds, whales, insects, frogs)
//! using acoustic features extracted from spectrograms. It provides:
//!
//! - **Spectrogram computation** via short-time DFT with configurable FFT size and hop
//! - **Call detection** using energy thresholding on spectrograms
//! - **Template matching** via normalized cross-correlation
//! - **Syllable segmentation** based on energy gaps
//! - **Acoustic diversity indices**: Shannon (H'), Simpson (1-D), and ACI
//! - **Soundscape power spectral density** for long-term monitoring
//! - **Frequency contour extraction** tracking dominant frequency over time
//! - **Band energy ratios** for species-specific spectral shape analysis
//! - **Species classification** via nearest-template Euclidean distance
//!
//! # Example
//!
//! ```
//! use r4w_core::bioacoustic_species_classifier::{
//!     BioacousticConfig, BioacousticClassifier,
//!     compute_spectrogram, shannon_diversity_index,
//! };
//!
//! let config = BioacousticConfig {
//!     sample_rate_hz: 44100.0,
//!     fft_size: 512,
//!     hop_size: 256,
//!     min_freq_hz: 500.0,
//!     max_freq_hz: 8000.0,
//! };
//!
//! // Generate a simple test tone
//! let signal: Vec<f64> = (0..2048)
//!     .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 44100.0).sin())
//!     .collect();
//!
//! let spectrogram = compute_spectrogram(&signal, config.fft_size, config.hop_size);
//! assert!(!spectrogram.is_empty());
//!
//! let energies = vec![0.3, 0.5, 0.1, 0.1];
//! let h = shannon_diversity_index(&energies);
//! assert!(h > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration and data types
// ---------------------------------------------------------------------------

/// Configuration for the bioacoustic analysis pipeline.
#[derive(Debug, Clone)]
pub struct BioacousticConfig {
    /// Sampling rate in Hertz.
    pub sample_rate_hz: f64,
    /// FFT size (number of points per frame).
    pub fft_size: usize,
    /// Hop size (stride between successive frames).
    pub hop_size: usize,
    /// Minimum frequency of interest in Hertz.
    pub min_freq_hz: f64,
    /// Maximum frequency of interest in Hertz.
    pub max_freq_hz: f64,
}

/// A single detected call event within a recording.
#[derive(Debug, Clone)]
pub struct CallDetection {
    /// Start position in samples.
    pub start_sample: usize,
    /// End position in samples.
    pub end_sample: usize,
    /// Peak frequency in Hertz.
    pub peak_freq_hz: f64,
    /// Detection confidence in `[0, 1]`.
    pub confidence: f64,
    /// Assigned species label (or `"unknown"`).
    pub species_label: String,
}

/// Main classifier that holds a configuration and a library of species templates.
#[derive(Debug, Clone)]
pub struct BioacousticClassifier {
    /// Pipeline configuration.
    pub config: BioacousticConfig,
    /// Named feature templates: `(species_name, feature_vector)`.
    pub templates: Vec<(String, Vec<f64>)>,
}

impl BioacousticClassifier {
    /// Create a new classifier with the given configuration and no templates.
    pub fn new(config: BioacousticConfig) -> Self {
        Self {
            config,
            templates: Vec::new(),
        }
    }

    /// Register a species template (name + feature vector).
    pub fn add_template(&mut self, name: &str, features: Vec<f64>) {
        self.templates.push((name.to_string(), features));
    }

    /// Run full detection + classification pipeline on a raw audio signal.
    ///
    /// Returns a list of [`CallDetection`] events found in the signal.
    pub fn analyze(&self, signal: &[f64]) -> Vec<CallDetection> {
        let spec = compute_spectrogram(signal, self.config.fft_size, self.config.hop_size);
        if spec.is_empty() {
            return Vec::new();
        }

        let threshold_db = -20.0;
        let min_dur = 2;
        let raw_calls = detect_calls(&spec, threshold_db, min_dur);

        let freq_res = self.config.sample_rate_hz / self.config.fft_size as f64;

        let templates_ref: Vec<(&str, Vec<f64>)> = self
            .templates
            .iter()
            .map(|(n, v)| (n.as_str(), v.clone()))
            .collect();

        raw_calls
            .iter()
            .map(|&(start_frame, end_frame, peak_bin)| {
                // Extract features from the call region
                let call_spec = &spec[start_frame..=end_frame.min(spec.len() - 1)];
                let contour = frequency_contour(call_spec, freq_res);
                let mean_freq = if contour.is_empty() {
                    peak_bin as f64 * freq_res
                } else {
                    contour.iter().sum::<f64>() / contour.len() as f64
                };

                // Build a simple feature vector from the call region
                let features = extract_region_features(call_spec);

                let (label, confidence) = if templates_ref.is_empty() {
                    ("unknown".to_string(), 0.0)
                } else {
                    classify_call(&features, &templates_ref)
                };

                CallDetection {
                    start_sample: start_frame * self.config.hop_size,
                    end_sample: end_frame * self.config.hop_size,
                    peak_freq_hz: mean_freq,
                    confidence,
                    species_label: label,
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Spectrogram computation (real-valued DFT)
// ---------------------------------------------------------------------------

/// Compute a power spectrogram from a time-domain signal.
///
/// Each column of the output is the magnitude-squared of the DFT of one
/// windowed frame. Only the first `fft_size / 2 + 1` (non-negative frequency)
/// bins are returned.
///
/// # Arguments
/// * `signal` - Input samples.
/// * `fft_size` - Number of samples per frame / DFT length.
/// * `hop_size` - Stride between successive frames.
///
/// # Returns
/// A `Vec<Vec<f64>>` where `result[frame][bin]` is the power in that
/// time-frequency cell.
pub fn compute_spectrogram(signal: &[f64], fft_size: usize, hop_size: usize) -> Vec<Vec<f64>> {
    if signal.len() < fft_size || fft_size == 0 || hop_size == 0 {
        return Vec::new();
    }

    let num_bins = fft_size / 2 + 1;
    let num_frames = (signal.len() - fft_size) / hop_size + 1;
    let mut spectrogram = Vec::with_capacity(num_frames);

    // Pre-compute Hann window
    let window: Vec<f64> = (0..fft_size)
        .map(|n| 0.5 * (1.0 - (2.0 * PI * n as f64 / fft_size as f64).cos()))
        .collect();

    for frame_idx in 0..num_frames {
        let offset = frame_idx * hop_size;
        let frame = &signal[offset..offset + fft_size];

        // Apply window
        let windowed: Vec<f64> = frame.iter().zip(&window).map(|(s, w)| s * w).collect();

        // DFT for non-negative frequencies
        let mut power = Vec::with_capacity(num_bins);
        for k in 0..num_bins {
            let mut re = 0.0;
            let mut im = 0.0;
            let freq = 2.0 * PI * k as f64 / fft_size as f64;
            for (n, &x) in windowed.iter().enumerate() {
                re += x * (freq * n as f64).cos();
                im -= x * (freq * n as f64).sin();
            }
            power.push(re * re + im * im);
        }
        spectrogram.push(power);
    }

    spectrogram
}

// ---------------------------------------------------------------------------
// Call detection
// ---------------------------------------------------------------------------

/// Detect call events in a spectrogram using an energy threshold.
///
/// Returns a list of `(start_frame, end_frame, peak_bin)` tuples where
/// `peak_bin` is the frequency bin with the highest energy summed across
/// the detection interval.
///
/// # Arguments
/// * `spectrogram` - Power spectrogram (`[frame][bin]`).
/// * `threshold_db` - Detection threshold in dB relative to the maximum
///   spectral energy across the entire spectrogram.
/// * `min_duration_frames` - Minimum number of consecutive frames above
///   threshold to count as a detection.
pub fn detect_calls(
    spectrogram: &[Vec<f64>],
    threshold_db: f64,
    min_duration_frames: usize,
) -> Vec<(usize, usize, usize)> {
    if spectrogram.is_empty() {
        return Vec::new();
    }

    // Per-frame total energy
    let frame_energy: Vec<f64> = spectrogram
        .iter()
        .map(|bins| bins.iter().sum::<f64>())
        .collect();

    let max_energy = frame_energy
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    if max_energy <= 0.0 {
        return Vec::new();
    }

    let threshold_linear = max_energy * 10.0_f64.powf(threshold_db / 10.0);

    let mut detections = Vec::new();
    let mut start: Option<usize> = None;

    for (i, &e) in frame_energy.iter().enumerate() {
        if e >= threshold_linear {
            if start.is_none() {
                start = Some(i);
            }
        } else if let Some(s) = start {
            if i - s >= min_duration_frames {
                let peak_bin = peak_bin_in_region(spectrogram, s, i - 1);
                detections.push((s, i - 1, peak_bin));
            }
            start = None;
        }
    }

    // Handle detection that runs to the end
    if let Some(s) = start {
        let end = frame_energy.len() - 1;
        if end + 1 - s >= min_duration_frames {
            let peak_bin = peak_bin_in_region(spectrogram, s, end);
            detections.push((s, end, peak_bin));
        }
    }

    detections
}

/// Find the frequency bin with the highest summed energy across a frame range.
fn peak_bin_in_region(spectrogram: &[Vec<f64>], start: usize, end: usize) -> usize {
    let num_bins = spectrogram[start].len();
    let mut bin_sums = vec![0.0; num_bins];
    for frame in &spectrogram[start..=end] {
        for (b, &v) in frame.iter().enumerate() {
            if b < num_bins {
                bin_sums[b] += v;
            }
        }
    }
    bin_sums
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(idx, _)| idx)
        .unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Template matching
// ---------------------------------------------------------------------------

/// Slide a spectral template across a spectrogram and return correlation scores.
///
/// Uses normalized cross-correlation between the template and each aligned
/// sub-region of the spectrogram.
///
/// # Returns
/// A list of `(start_frame, correlation)` pairs. Correlation values are in
/// `[-1, 1]` where 1 indicates a perfect match.
pub fn match_template(
    spectrogram: &[Vec<f64>],
    template: &[Vec<f64>],
) -> Vec<(usize, f64)> {
    if spectrogram.is_empty() || template.is_empty() || template.len() > spectrogram.len() {
        return Vec::new();
    }

    let t_len = template.len();
    let slide_len = spectrogram.len() - t_len + 1;

    // Flatten template and precompute its norm
    let t_flat: Vec<f64> = template.iter().flat_map(|row| row.iter().copied()).collect();
    let t_mean = t_flat.iter().sum::<f64>() / t_flat.len() as f64;
    let t_centered: Vec<f64> = t_flat.iter().map(|&v| v - t_mean).collect();
    let t_norm = t_centered.iter().map(|v| v * v).sum::<f64>().sqrt();

    if t_norm < 1e-15 {
        return (0..slide_len).map(|i| (i, 0.0)).collect();
    }

    let mut results = Vec::with_capacity(slide_len);

    for start in 0..slide_len {
        let s_flat: Vec<f64> = spectrogram[start..start + t_len]
            .iter()
            .flat_map(|row| row.iter().copied())
            .collect();

        // Ensure matching dimensionality - use the shorter length
        let len = s_flat.len().min(t_centered.len());
        let s_mean = s_flat[..len].iter().sum::<f64>() / len as f64;

        let mut dot = 0.0;
        let mut s_var = 0.0;
        for i in 0..len {
            let sc = s_flat[i] - s_mean;
            dot += sc * t_centered[i];
            s_var += sc * sc;
        }

        let s_norm = s_var.sqrt();
        let corr = if s_norm < 1e-15 {
            0.0
        } else {
            dot / (s_norm * t_norm)
        };

        results.push((start, corr));
    }

    results
}

// ---------------------------------------------------------------------------
// Syllable segmentation
// ---------------------------------------------------------------------------

/// Segment a signal into syllable intervals based on energy gaps.
///
/// Returns a list of `(start_sample, end_sample)` for each syllable found.
///
/// # Arguments
/// * `signal` - Raw audio samples.
/// * `sample_rate` - Sample rate in Hz.
/// * `min_gap_s` - Minimum silent gap (seconds) between syllables.
pub fn segment_syllables(
    signal: &[f64],
    sample_rate: f64,
    min_gap_s: f64,
) -> Vec<(usize, usize)> {
    if signal.is_empty() || sample_rate <= 0.0 {
        return Vec::new();
    }

    let frame_len = (sample_rate * 0.01).max(1.0) as usize; // 10 ms frames
    let min_gap_frames = (min_gap_s * sample_rate / frame_len as f64).ceil() as usize;

    // Compute short-term energy per frame
    let num_frames = signal.len() / frame_len;
    if num_frames == 0 {
        return vec![(0, signal.len().saturating_sub(1))];
    }

    let energies: Vec<f64> = (0..num_frames)
        .map(|i| {
            let start = i * frame_len;
            let end = (start + frame_len).min(signal.len());
            signal[start..end].iter().map(|x| x * x).sum::<f64>() / frame_len as f64
        })
        .collect();

    // Threshold: use 10% of peak energy, but at least a tiny value
    let max_e = energies
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    let threshold = (max_e * 0.1).max(1e-12);

    let mut syllables = Vec::new();
    let mut start: Option<usize> = None;
    let mut silent_count = 0_usize;

    for (i, &e) in energies.iter().enumerate() {
        if e >= threshold {
            if start.is_none() {
                start = Some(i);
            }
            silent_count = 0;
        } else {
            silent_count += 1;
            if let Some(s) = start {
                if silent_count >= min_gap_frames.max(1) {
                    let end_frame = i.saturating_sub(silent_count);
                    syllables.push((s * frame_len, (end_frame + 1) * frame_len));
                    start = None;
                }
            }
        }
    }

    // Flush final syllable
    if let Some(s) = start {
        syllables.push((s * frame_len, num_frames * frame_len));
    }

    syllables
}

// ---------------------------------------------------------------------------
// Acoustic diversity indices
// ---------------------------------------------------------------------------

/// Compute the Shannon diversity index (H') from band energy proportions.
///
/// H' = -sum(p_i * ln(p_i)) where p_i is the proportion of energy in band i.
///
/// A higher value indicates a more acoustically diverse soundscape.
pub fn shannon_diversity_index(band_energies: &[f64]) -> f64 {
    if band_energies.is_empty() {
        return 0.0;
    }

    let total: f64 = band_energies.iter().filter(|&&e| e > 0.0).sum();
    if total <= 0.0 {
        return 0.0;
    }

    let mut h = 0.0;
    for &e in band_energies {
        if e > 0.0 {
            let p = e / total;
            h -= p * p.ln();
        }
    }
    h
}

/// Compute the Simpson diversity index (1 - D) from band energy proportions.
///
/// D = sum(p_i^2), and the diversity is reported as 1 - D.
/// A value near 1 indicates high diversity; near 0 indicates dominance by
/// one band.
pub fn simpson_diversity_index(band_energies: &[f64]) -> f64 {
    if band_energies.is_empty() {
        return 0.0;
    }

    let total: f64 = band_energies.iter().filter(|&&e| e > 0.0).sum();
    if total <= 0.0 {
        return 0.0;
    }

    let d: f64 = band_energies
        .iter()
        .filter(|&&e| e > 0.0)
        .map(|&e| {
            let p = e / total;
            p * p
        })
        .sum();

    1.0 - d
}

// ---------------------------------------------------------------------------
// Acoustic complexity index (ACI)
// ---------------------------------------------------------------------------

/// Compute the Acoustic Complexity Index from a spectrogram.
///
/// ACI measures the variability of successive amplitude values in each
/// frequency bin across time. Higher values indicate more complex (often
/// biologically richer) soundscapes.
///
/// ACI = sum over bins of (sum |d_k| / sum a_k) where d_k is the
/// frame-to-frame difference and a_k is the amplitude.
pub fn acoustic_complexity_index(spectrogram: &[Vec<f64>]) -> f64 {
    if spectrogram.len() < 2 {
        return 0.0;
    }

    let num_bins = spectrogram[0].len();
    let mut aci = 0.0;

    for bin in 0..num_bins {
        let mut diff_sum = 0.0;
        let mut amp_sum = 0.0;

        for t in 1..spectrogram.len() {
            let a_prev = spectrogram[t - 1].get(bin).copied().unwrap_or(0.0).sqrt();
            let a_curr = spectrogram[t].get(bin).copied().unwrap_or(0.0).sqrt();
            diff_sum += (a_curr - a_prev).abs();
            amp_sum += a_curr;
        }

        if amp_sum > 1e-15 {
            aci += diff_sum / amp_sum;
        }
    }

    aci
}

// ---------------------------------------------------------------------------
// Soundscape power spectral density
// ---------------------------------------------------------------------------

/// Compute the average power spectral density of a signal.
///
/// Returns the averaged magnitude-squared DFT (Welch-style) for
/// non-negative frequencies.
pub fn soundscape_power_density(signal: &[f64], sample_rate: f64, fft_size: usize) -> Vec<f64> {
    if signal.is_empty() || fft_size == 0 || sample_rate <= 0.0 {
        return Vec::new();
    }

    let hop = fft_size / 2;
    let spec = compute_spectrogram(signal, fft_size, hop.max(1));
    if spec.is_empty() {
        return Vec::new();
    }

    let num_bins = spec[0].len();
    let mut avg = vec![0.0; num_bins];
    for frame in &spec {
        for (b, &v) in frame.iter().enumerate() {
            if b < num_bins {
                avg[b] += v;
            }
        }
    }

    let n = spec.len() as f64;
    // Normalize: divide by number of frames and by fft_size for PSD scaling
    let scale = n * fft_size as f64 * sample_rate;
    for v in &mut avg {
        *v /= scale.max(1e-15);
    }

    avg
}

// ---------------------------------------------------------------------------
// Frequency contour extraction
// ---------------------------------------------------------------------------

/// Extract the dominant frequency contour from a spectrogram.
///
/// For each frame, returns the frequency (Hz) of the bin with the highest
/// energy, scaled by `freq_resolution` (Hz per bin).
///
/// # Arguments
/// * `spectrogram` - Power spectrogram `[frame][bin]`.
/// * `freq_resolution` - Frequency spacing between bins (e.g., sample_rate / fft_size).
pub fn frequency_contour(spectrogram: &[Vec<f64>], freq_resolution: f64) -> Vec<f64> {
    spectrogram
        .iter()
        .map(|frame| {
            let peak_bin = frame
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(idx, _)| idx)
                .unwrap_or(0);
            peak_bin as f64 * freq_resolution
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Band energy ratio
// ---------------------------------------------------------------------------

/// Compute energy ratios between frequency bands.
///
/// Given a magnitude spectrum and three boundary bin indices that define
/// a *low* band `[0, low_bin)`, a *mid* band `[low_bin, mid_bin)`, and a
/// *high* band `[mid_bin, high_bin)`, returns `(low/mid, mid/high)` ratios.
///
/// If any denominator band has zero energy the corresponding ratio is 0.
pub fn band_energy_ratio(
    spectrum: &[f64],
    low_bin: usize,
    mid_bin: usize,
    high_bin: usize,
) -> (f64, f64) {
    if spectrum.is_empty() || low_bin >= mid_bin || mid_bin >= high_bin {
        return (0.0, 0.0);
    }

    let sum_range = |start: usize, end: usize| -> f64 {
        spectrum[start..end.min(spectrum.len())]
            .iter()
            .sum::<f64>()
    };

    let low_energy = sum_range(0, low_bin);
    let mid_energy = sum_range(low_bin, mid_bin);
    let high_energy = sum_range(mid_bin, high_bin);

    let low_mid = if mid_energy > 1e-15 {
        low_energy / mid_energy
    } else {
        0.0
    };

    let mid_high = if high_energy > 1e-15 {
        mid_energy / high_energy
    } else {
        0.0
    };

    (low_mid, mid_high)
}

// ---------------------------------------------------------------------------
// Species classification
// ---------------------------------------------------------------------------

/// Classify a call by comparing a feature vector to a set of species templates.
///
/// Uses minimum Euclidean distance with confidence derived from the ratio
/// of the best to second-best distance. Returns `(species_name, confidence)`.
///
/// # Arguments
/// * `features` - Feature vector extracted from the call.
/// * `templates` - Slice of `(species_name, feature_vector)` pairs.
pub fn classify_call(features: &[f64], templates: &[(&str, Vec<f64>)]) -> (String, f64) {
    if templates.is_empty() {
        return ("unknown".to_string(), 0.0);
    }

    let distances: Vec<(usize, f64)> = templates
        .iter()
        .enumerate()
        .map(|(i, (_, tmpl))| {
            let len = features.len().min(tmpl.len());
            let dist: f64 = (0..len)
                .map(|j| (features[j] - tmpl[j]).powi(2))
                .sum::<f64>();
            // Penalize dimension mismatch
            let mismatch_penalty =
                (features.len() as i64 - tmpl.len() as i64).unsigned_abs() as f64;
            (i, dist.sqrt() + mismatch_penalty)
        })
        .collect();

    let best = distances
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap();

    let confidence = if templates.len() == 1 {
        // Single template: confidence based on distance magnitude
        1.0 / (1.0 + best.1)
    } else {
        // Multiple templates: ratio of best to second-best
        let mut sorted: Vec<f64> = distances.iter().map(|d| d.1).collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        if sorted[1] < 1e-15 {
            0.5
        } else {
            1.0 - sorted[0] / sorted[1]
        }
    };

    let label = &templates[best.0].0;
    (label.to_string(), confidence.clamp(0.0, 1.0))
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Extract a simple feature vector from a spectrogram region.
///
/// Features: [mean_energy, spectral_centroid, spectral_bandwidth, aci_segment,
///            peak_bin_ratio, temporal_variance]
fn extract_region_features(spec: &[Vec<f64>]) -> Vec<f64> {
    if spec.is_empty() {
        return vec![0.0; 6];
    }

    let num_bins = spec[0].len();

    // Mean energy
    let total: f64 = spec.iter().flat_map(|f| f.iter()).sum();
    let count = (spec.len() * num_bins).max(1) as f64;
    let mean_energy = total / count;

    // Average spectral centroid
    let mut centroid_sum = 0.0;
    for frame in spec {
        let frame_total: f64 = frame.iter().sum();
        if frame_total > 1e-15 {
            let c: f64 = frame
                .iter()
                .enumerate()
                .map(|(b, &v)| b as f64 * v)
                .sum::<f64>()
                / frame_total;
            centroid_sum += c;
        }
    }
    let spectral_centroid = centroid_sum / spec.len() as f64;

    // Spectral bandwidth (RMS spread around centroid)
    let mut bw_sum = 0.0;
    for frame in spec {
        let frame_total: f64 = frame.iter().sum();
        if frame_total > 1e-15 {
            let bw: f64 = frame
                .iter()
                .enumerate()
                .map(|(b, &v)| (b as f64 - spectral_centroid).powi(2) * v)
                .sum::<f64>()
                / frame_total;
            bw_sum += bw.sqrt();
        }
    }
    let spectral_bandwidth = bw_sum / spec.len() as f64;

    // ACI of the region
    let aci = acoustic_complexity_index(spec);

    // Peak bin ratio (peak bin / total bins)
    let peak_ratio = spectral_centroid / num_bins.max(1) as f64;

    // Temporal energy variance
    let frame_energies: Vec<f64> = spec.iter().map(|f| f.iter().sum::<f64>()).collect();
    let mean_fe = frame_energies.iter().sum::<f64>() / frame_energies.len() as f64;
    let var = frame_energies
        .iter()
        .map(|&e| (e - mean_fe).powi(2))
        .sum::<f64>()
        / frame_energies.len() as f64;

    vec![
        mean_energy,
        spectral_centroid,
        spectral_bandwidth,
        aci,
        peak_ratio,
        var.sqrt(),
    ]
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a sine wave signal.
    fn sine_wave(freq: f64, sample_rate: f64, num_samples: usize) -> Vec<f64> {
        (0..num_samples)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    /// Helper: generate a simple chirp signal (linear frequency sweep).
    #[allow(dead_code)]
    fn chirp(f_start: f64, f_end: f64, sample_rate: f64, num_samples: usize) -> Vec<f64> {
        let duration = num_samples as f64 / sample_rate;
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let freq = f_start + (f_end - f_start) * t / duration;
                (2.0 * PI * freq * t).sin()
            })
            .collect()
    }

    // ---------------------------
    // Config tests
    // ---------------------------

    #[test]
    fn test_config_creation() {
        let config = BioacousticConfig {
            sample_rate_hz: 44100.0,
            fft_size: 1024,
            hop_size: 512,
            min_freq_hz: 200.0,
            max_freq_hz: 10000.0,
        };
        assert_eq!(config.fft_size, 1024);
        assert_eq!(config.hop_size, 512);
        assert!((config.sample_rate_hz - 44100.0).abs() < f64::EPSILON);
    }

    // ---------------------------
    // Spectrogram tests
    // ---------------------------

    #[test]
    fn test_spectrogram_dimensions() {
        let sig = sine_wave(1000.0, 44100.0, 4096);
        let spec = compute_spectrogram(&sig, 512, 256);
        // Expected frames: (4096 - 512) / 256 + 1 = 15
        assert_eq!(spec.len(), 15);
        // Expected bins: 512/2 + 1 = 257
        assert_eq!(spec[0].len(), 257);
    }

    #[test]
    fn test_spectrogram_empty_signal() {
        let spec = compute_spectrogram(&[], 512, 256);
        assert!(spec.is_empty());
    }

    #[test]
    fn test_spectrogram_signal_too_short() {
        let sig = vec![1.0; 100];
        let spec = compute_spectrogram(&sig, 512, 256);
        assert!(spec.is_empty());
    }

    #[test]
    fn test_spectrogram_tone_peak() {
        // A 1000 Hz tone at 8000 Hz sample rate in a 256-point FFT
        // should produce a peak near bin index 1000/8000*256 = 32.
        let sig = sine_wave(1000.0, 8000.0, 2048);
        let spec = compute_spectrogram(&sig, 256, 128);
        // Look at the middle frame
        let mid = spec.len() / 2;
        let frame = &spec[mid];
        let peak_bin = frame
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        // Expected bin ~32, allow +/- 2
        assert!(
            (peak_bin as i32 - 32).unsigned_abs() <= 2,
            "peak at bin {}, expected ~32",
            peak_bin
        );
    }

    // ---------------------------
    // Call detection tests
    // ---------------------------

    #[test]
    fn test_detect_calls_empty() {
        let calls = detect_calls(&[], -10.0, 2);
        assert!(calls.is_empty());
    }

    #[test]
    fn test_detect_calls_single_tone() {
        let sig = sine_wave(2000.0, 8000.0, 4096);
        let spec = compute_spectrogram(&sig, 256, 128);
        let calls = detect_calls(&spec, -30.0, 2);
        // A continuous tone should appear as one detection spanning most frames
        assert!(
            !calls.is_empty(),
            "should detect at least one call in continuous tone"
        );
    }

    #[test]
    fn test_detect_calls_silence() {
        let sig = vec![0.0; 4096];
        let spec = compute_spectrogram(&sig, 256, 128);
        let calls = detect_calls(&spec, -10.0, 2);
        assert!(calls.is_empty(), "should detect nothing in silence");
    }

    // ---------------------------
    // Template matching tests
    // ---------------------------

    #[test]
    fn test_match_template_self() {
        let sig = sine_wave(1500.0, 8000.0, 2048);
        let spec = compute_spectrogram(&sig, 256, 128);
        // Use the whole spectrogram as a template - should get correlation ~1 at offset 0
        let matches = match_template(&spec, &spec);
        assert_eq!(matches.len(), 1);
        assert!(
            (matches[0].1 - 1.0).abs() < 1e-6,
            "self-match should be ~1.0, got {}",
            matches[0].1
        );
    }

    #[test]
    fn test_match_template_empty() {
        let matches = match_template(&[], &[vec![1.0]]);
        assert!(matches.is_empty());
    }

    #[test]
    fn test_match_template_partial() {
        let sig = sine_wave(1500.0, 8000.0, 4096);
        let spec = compute_spectrogram(&sig, 256, 128);
        // Use the first 3 frames as template
        let template: Vec<Vec<f64>> = spec[0..3].to_vec();
        let matches = match_template(&spec, &template);
        assert!(!matches.is_empty());
        // All correlations should be high for a steady tone
        for &(_, corr) in &matches {
            assert!(
                corr > 0.8,
                "correlation {} should be high for steady tone",
                corr
            );
        }
    }

    // ---------------------------
    // Syllable segmentation tests
    // ---------------------------

    #[test]
    fn test_segment_syllables_single() {
        let sig = sine_wave(1000.0, 8000.0, 8000); // 1 second of tone
        let syllables = segment_syllables(&sig, 8000.0, 0.05);
        assert!(
            !syllables.is_empty(),
            "should find at least one syllable in continuous tone"
        );
    }

    #[test]
    fn test_segment_syllables_with_gap() {
        // Two tone bursts separated by silence
        let sr = 8000.0;
        let mut sig = sine_wave(1000.0, sr, 2000); // 250ms tone
        sig.extend(vec![0.0; 1600]); // 200ms silence
        sig.extend(sine_wave(1000.0, sr, 2000)); // 250ms tone

        let syllables = segment_syllables(&sig, sr, 0.05);
        assert!(
            syllables.len() >= 2,
            "should find 2 syllables, found {}",
            syllables.len()
        );
    }

    #[test]
    fn test_segment_syllables_empty() {
        let syllables = segment_syllables(&[], 8000.0, 0.05);
        assert!(syllables.is_empty());
    }

    // ---------------------------
    // Shannon diversity tests
    // ---------------------------

    #[test]
    fn test_shannon_uniform() {
        // Uniform distribution across 4 bands -> H' = ln(4) ~ 1.386
        let energies = vec![1.0, 1.0, 1.0, 1.0];
        let h = shannon_diversity_index(&energies);
        assert!(
            (h - 4.0_f64.ln()).abs() < 1e-10,
            "H' for uniform 4-band = {}, expected {}",
            h,
            4.0_f64.ln()
        );
    }

    #[test]
    fn test_shannon_single_band() {
        // All energy in one band -> H' = 0
        let energies = vec![1.0, 0.0, 0.0, 0.0];
        let h = shannon_diversity_index(&energies);
        assert!(h.abs() < 1e-10, "H' should be 0 for single band, got {}", h);
    }

    #[test]
    fn test_shannon_empty() {
        assert_eq!(shannon_diversity_index(&[]), 0.0);
    }

    // ---------------------------
    // Simpson diversity tests
    // ---------------------------

    #[test]
    fn test_simpson_uniform() {
        // Uniform: D = 4*(0.25^2) = 0.25, so 1-D = 0.75
        let energies = vec![1.0, 1.0, 1.0, 1.0];
        let s = simpson_diversity_index(&energies);
        assert!(
            (s - 0.75).abs() < 1e-10,
            "1-D for uniform 4-band = {}, expected 0.75",
            s
        );
    }

    #[test]
    fn test_simpson_single_band() {
        let energies = vec![1.0, 0.0, 0.0];
        let s = simpson_diversity_index(&energies);
        assert!(
            s.abs() < 1e-10,
            "1-D should be 0 for single band, got {}",
            s
        );
    }

    // ---------------------------
    // ACI tests
    // ---------------------------

    #[test]
    fn test_aci_constant() {
        // Constant spectrogram -> ACI should be 0
        let spec = vec![vec![1.0, 1.0, 1.0]; 10];
        let aci = acoustic_complexity_index(&spec);
        assert!(
            aci.abs() < 1e-10,
            "ACI of constant spectrogram should be ~0, got {}",
            aci
        );
    }

    #[test]
    fn test_aci_variable() {
        // Alternating frames -> should produce positive ACI
        let spec: Vec<Vec<f64>> = (0..10)
            .map(|i| {
                if i % 2 == 0 {
                    vec![1.0, 0.1, 1.0]
                } else {
                    vec![0.1, 1.0, 0.1]
                }
            })
            .collect();
        let aci = acoustic_complexity_index(&spec);
        assert!(
            aci > 0.0,
            "ACI of variable spectrogram should be > 0, got {}",
            aci
        );
    }

    #[test]
    fn test_aci_too_short() {
        assert_eq!(acoustic_complexity_index(&[vec![1.0]]), 0.0);
    }

    // ---------------------------
    // Soundscape PSD tests
    // ---------------------------

    #[test]
    fn test_soundscape_psd_length() {
        let sig = sine_wave(1000.0, 8000.0, 4096);
        let psd = soundscape_power_density(&sig, 8000.0, 256);
        assert_eq!(psd.len(), 129); // 256/2 + 1
    }

    #[test]
    fn test_soundscape_psd_empty() {
        let psd = soundscape_power_density(&[], 8000.0, 256);
        assert!(psd.is_empty());
    }

    // ---------------------------
    // Frequency contour tests
    // ---------------------------

    #[test]
    fn test_frequency_contour_steady_tone() {
        let sig = sine_wave(1000.0, 8000.0, 4096);
        let spec = compute_spectrogram(&sig, 256, 128);
        let freq_res = 8000.0 / 256.0; // 31.25 Hz per bin
        let contour = frequency_contour(&spec, freq_res);

        assert_eq!(contour.len(), spec.len());
        // All values should be near 1000 Hz
        for (i, &f) in contour.iter().enumerate() {
            assert!(
                (f - 1000.0).abs() < 100.0,
                "contour[{}] = {} Hz, expected ~1000 Hz",
                i,
                f
            );
        }
    }

    // ---------------------------
    // Band energy ratio tests
    // ---------------------------

    #[test]
    fn test_band_energy_ratio_equal() {
        let spectrum = vec![1.0; 30];
        let (lm, mh) = band_energy_ratio(&spectrum, 10, 20, 30);
        assert!(
            (lm - 1.0).abs() < 1e-10,
            "low/mid should be 1.0, got {}",
            lm
        );
        assert!(
            (mh - 1.0).abs() < 1e-10,
            "mid/high should be 1.0, got {}",
            mh
        );
    }

    #[test]
    fn test_band_energy_ratio_invalid() {
        let (lm, mh) = band_energy_ratio(&[1.0; 10], 5, 3, 8);
        assert_eq!(lm, 0.0);
        assert_eq!(mh, 0.0);
    }

    // ---------------------------
    // Classification tests
    // ---------------------------

    #[test]
    fn test_classify_call_nearest() {
        let features = vec![1.0, 0.0, 0.0];
        let templates = vec![
            ("bird_a", vec![1.0, 0.1, 0.0]),
            ("frog_b", vec![0.0, 1.0, 0.0]),
        ];
        let (label, confidence) = classify_call(&features, &templates);
        assert_eq!(label, "bird_a");
        assert!(confidence > 0.0, "confidence should be positive");
    }

    #[test]
    fn test_classify_call_empty_templates() {
        let (label, confidence) = classify_call(&[1.0], &[]);
        assert_eq!(label, "unknown");
        assert_eq!(confidence, 0.0);
    }

    // ---------------------------
    // Classifier integration test
    // ---------------------------

    #[test]
    fn test_classifier_analyze() {
        let config = BioacousticConfig {
            sample_rate_hz: 8000.0,
            fft_size: 256,
            hop_size: 128,
            min_freq_hz: 200.0,
            max_freq_hz: 4000.0,
        };
        let mut clf = BioacousticClassifier::new(config);
        clf.add_template("robin", vec![0.5, 0.3, 0.2, 0.1, 0.4, 0.1]);

        let sig = sine_wave(2000.0, 8000.0, 4096);
        let detections = clf.analyze(&sig);
        // Should detect something in a clear tone
        assert!(
            !detections.is_empty(),
            "classifier should detect at least one call"
        );
        // Each detection should have the robin label (only template)
        for det in &detections {
            assert_eq!(det.species_label, "robin");
            assert!(det.confidence > 0.0);
        }
    }

    #[test]
    fn test_classifier_no_templates() {
        let config = BioacousticConfig {
            sample_rate_hz: 8000.0,
            fft_size: 256,
            hop_size: 128,
            min_freq_hz: 200.0,
            max_freq_hz: 4000.0,
        };
        let clf = BioacousticClassifier::new(config);
        let sig = sine_wave(2000.0, 8000.0, 4096);
        let detections = clf.analyze(&sig);
        for det in &detections {
            assert_eq!(det.species_label, "unknown");
        }
    }

    #[test]
    fn test_call_detection_struct_fields() {
        let cd = CallDetection {
            start_sample: 0,
            end_sample: 1000,
            peak_freq_hz: 3200.0,
            confidence: 0.85,
            species_label: "whale_humpback".to_string(),
        };
        assert_eq!(cd.start_sample, 0);
        assert_eq!(cd.end_sample, 1000);
        assert!((cd.peak_freq_hz - 3200.0).abs() < f64::EPSILON);
        assert!((cd.confidence - 0.85).abs() < f64::EPSILON);
        assert_eq!(cd.species_label, "whale_humpback");
    }
}

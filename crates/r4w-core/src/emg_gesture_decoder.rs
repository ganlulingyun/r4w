//! Surface EMG (sEMG) Multi-Channel Gesture Decoder
//!
//! Real-time classification of human-machine interface gestures from
//! multi-channel surface electromyography signals. Implements time-domain
//! feature extraction, blind source separation (FastICA), motor unit action
//! potential (MUAP) template matching, and k-nearest-neighbor classification.
//!
//! All math is implemented from scratch with no external dependencies beyond `std`.
//!
//! ## Pipeline
//!
//! ```text
//! Raw sEMG channels
//!   -> Bandpass filter (20-450 Hz)
//!   -> FastICA source separation
//!   -> Feature extraction (RMS, MAV, ZC, WL, SSC)
//!   -> k-NN classification
//!   -> Gesture label + confidence
//! ```
//!
//! ## Example
//!
//! ```rust,no_run
//! use r4w_core::emg_gesture_decoder::*;
//!
//! // Configure for 4-channel EMG at 1000 Hz
//! let config = EmgGestureConfig {
//!     num_channels: 4,
//!     sample_rate_hz: 1000.0,
//!     window_size_ms: 200.0,
//!     gesture_labels: vec![
//!         "rest".into(), "fist".into(), "open".into(), "pinch".into(),
//!     ],
//! };
//!
//! // Extract features from a multi-channel window
//! let window: Vec<Vec<f64>> = vec![vec![0.0; 200]; 4];
//! let features = extract_features(&window);
//!
//! // Train classifier
//! let mut classifier = EmgClassifier::new(3);
//! classifier.train(&[features.clone()], &[0]);
//!
//! // Classify
//! let result = classifier.classify(&features);
//! println!("Gesture: {}, Confidence: {:.2}", result.gesture_index, result.confidence);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the EMG gesture decoder pipeline.
#[derive(Debug, Clone)]
pub struct EmgGestureConfig {
    /// Number of EMG electrode channels.
    pub num_channels: usize,
    /// Sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// Analysis window length in milliseconds.
    pub window_size_ms: f64,
    /// Human-readable labels for each gesture class.
    pub gesture_labels: Vec<String>,
}

impl EmgGestureConfig {
    /// Returns the window size in samples.
    pub fn window_samples(&self) -> usize {
        (self.sample_rate_hz * self.window_size_ms / 1000.0).round() as usize
    }
}

// ---------------------------------------------------------------------------
// Feature vector
// ---------------------------------------------------------------------------

/// Time-domain feature vector extracted from a multi-channel EMG window.
#[derive(Debug, Clone)]
pub struct EmgFeatureVector {
    /// Root-mean-square value per channel.
    pub rms: Vec<f64>,
    /// Mean absolute value per channel.
    pub mean_absolute_value: Vec<f64>,
    /// Zero-crossing count per channel.
    pub zero_crossings: Vec<usize>,
    /// Waveform length (cumulative absolute first-difference) per channel.
    pub waveform_length: Vec<f64>,
    /// Slope sign change count per channel.
    pub slope_sign_changes: Vec<usize>,
}

impl EmgFeatureVector {
    /// Flatten all features into a single f64 vector for distance computation.
    pub fn to_flat_vec(&self) -> Vec<f64> {
        let mut v = Vec::new();
        v.extend_from_slice(&self.rms);
        v.extend_from_slice(&self.mean_absolute_value);
        for &zc in &self.zero_crossings {
            v.push(zc as f64);
        }
        v.extend_from_slice(&self.waveform_length);
        for &ssc in &self.slope_sign_changes {
            v.push(ssc as f64);
        }
        v
    }

    /// Number of channels represented.
    pub fn num_channels(&self) -> usize {
        self.rms.len()
    }
}

// ---------------------------------------------------------------------------
// Per-channel feature functions
// ---------------------------------------------------------------------------

/// Compute the root-mean-square of a signal.
///
/// Returns 0.0 for an empty signal.
pub fn compute_rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = signal.iter().map(|&x| x * x).sum();
    (sum_sq / signal.len() as f64).sqrt()
}

/// Compute the mean absolute value of a signal.
///
/// Returns 0.0 for an empty signal.
pub fn compute_mav(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_abs: f64 = signal.iter().map(|x| x.abs()).sum();
    sum_abs / signal.len() as f64
}

/// Count the number of zero crossings in a signal.
///
/// A zero crossing occurs when consecutive samples have opposite signs
/// (strictly: `x[n] * x[n-1] < 0`).
pub fn count_zero_crossings(signal: &[f64]) -> usize {
    if signal.len() < 2 {
        return 0;
    }
    signal
        .windows(2)
        .filter(|w| w[0] * w[1] < 0.0)
        .count()
}

/// Compute the waveform length (cumulative absolute first difference).
///
/// `WL = sum(|x[n] - x[n-1]|)` for n = 1..N-1.
pub fn compute_waveform_length(signal: &[f64]) -> f64 {
    if signal.len() < 2 {
        return 0.0;
    }
    signal
        .windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .sum()
}

/// Count slope sign changes.
///
/// A slope sign change occurs when the first difference changes sign:
/// `(x[n] - x[n-1]) * (x[n+1] - x[n]) < 0`.
pub fn count_slope_sign_changes(signal: &[f64]) -> usize {
    if signal.len() < 3 {
        return 0;
    }
    let mut count = 0usize;
    for i in 1..signal.len() - 1 {
        let d_prev = signal[i] - signal[i - 1];
        let d_next = signal[i + 1] - signal[i];
        if d_prev * d_next < 0.0 {
            count += 1;
        }
    }
    count
}

// ---------------------------------------------------------------------------
// Multi-channel feature extraction
// ---------------------------------------------------------------------------

/// Extract the standard time-domain EMG feature vector from a multi-channel
/// window.
///
/// `multichannel_window[ch]` is the slice of samples for channel `ch`.
pub fn extract_features(multichannel_window: &[Vec<f64>]) -> EmgFeatureVector {
    let num_ch = multichannel_window.len();
    let mut rms = Vec::with_capacity(num_ch);
    let mut mav = Vec::with_capacity(num_ch);
    let mut zc = Vec::with_capacity(num_ch);
    let mut wl = Vec::with_capacity(num_ch);
    let mut ssc = Vec::with_capacity(num_ch);

    for ch in multichannel_window {
        rms.push(compute_rms(ch));
        mav.push(compute_mav(ch));
        zc.push(count_zero_crossings(ch));
        wl.push(compute_waveform_length(ch));
        ssc.push(count_slope_sign_changes(ch));
    }

    EmgFeatureVector {
        rms,
        mean_absolute_value: mav,
        zero_crossings: zc,
        waveform_length: wl,
        slope_sign_changes: ssc,
    }
}

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

/// Result of gesture classification.
#[derive(Debug, Clone)]
pub struct GestureClassification {
    /// Index of the classified gesture (into `EmgGestureConfig::gesture_labels`).
    pub gesture_index: usize,
    /// Confidence score in [0, 1] (fraction of k neighbors voting for this class).
    pub confidence: f64,
    /// Sorted distances to each of the k nearest training samples.
    pub distances: Vec<f64>,
}

/// k-Nearest-Neighbor classifier for EMG gesture recognition.
#[derive(Debug, Clone)]
pub struct EmgClassifier {
    k: usize,
    training_features: Vec<Vec<f64>>,
    training_labels: Vec<usize>,
}

impl EmgClassifier {
    /// Create a new k-NN classifier with the given k.
    pub fn new(k: usize) -> Self {
        Self {
            k: if k == 0 { 1 } else { k },
            training_features: Vec::new(),
            training_labels: Vec::new(),
        }
    }

    /// Train the classifier by storing labelled feature vectors.
    ///
    /// Existing training data is replaced.
    pub fn train(&mut self, features: &[EmgFeatureVector], labels: &[usize]) {
        assert_eq!(features.len(), labels.len(), "features and labels must have equal length");
        self.training_features = features.iter().map(|f| f.to_flat_vec()).collect();
        self.training_labels = labels.to_vec();
    }

    /// Classify a new feature vector.
    ///
    /// Panics if the classifier has not been trained.
    pub fn classify(&self, features: &EmgFeatureVector) -> GestureClassification {
        assert!(
            !self.training_features.is_empty(),
            "classifier must be trained before classification"
        );

        let query = features.to_flat_vec();

        // Compute distances to every training sample.
        let mut dists: Vec<(f64, usize)> = self
            .training_features
            .iter()
            .zip(self.training_labels.iter())
            .map(|(tf, &label)| (euclidean_distance(&query, tf), label))
            .collect();

        // Sort by distance ascending.
        dists.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let effective_k = self.k.min(dists.len());
        let neighbors = &dists[..effective_k];

        // Majority vote.
        let max_label = neighbors.iter().map(|&(_, l)| l).max().unwrap_or(0);
        let mut votes = vec![0usize; max_label + 1];
        for &(_, label) in neighbors {
            if label < votes.len() {
                votes[label] += 1;
            }
        }

        let (gesture_index, &max_votes) = votes
            .iter()
            .enumerate()
            .max_by_key(|&(_, &v)| v)
            .unwrap();

        let confidence = max_votes as f64 / effective_k as f64;
        let distances: Vec<f64> = neighbors.iter().map(|&(d, _)| d).collect();

        GestureClassification {
            gesture_index,
            confidence,
            distances,
        }
    }
}

/// Euclidean distance between two flat feature vectors.
fn euclidean_distance(a: &[f64], b: &[f64]) -> f64 {
    let len = a.len().min(b.len());
    let sum: f64 = (0..len).map(|i| (a[i] - b[i]).powi(2)).sum();
    sum.sqrt()
}

// ---------------------------------------------------------------------------
// FastICA blind source separation
// ---------------------------------------------------------------------------

/// Simplified FastICA separator for blind source separation of mixed EMG
/// channels.
///
/// Uses PCA whitening followed by fixed-point iteration with `tanh`
/// nonlinearity.
#[derive(Debug, Clone)]
pub struct FastIcaSeparator {
    num_components: usize,
    max_iterations: usize,
    tolerance: f64,
}

impl FastIcaSeparator {
    /// Create a new FastICA separator targeting `num_components` independent
    /// sources.
    pub fn new(num_components: usize) -> Self {
        Self {
            num_components: if num_components == 0 { 1 } else { num_components },
            max_iterations: 200,
            tolerance: 1e-6,
        }
    }

    /// Separate mixed signals into independent components.
    ///
    /// `mixed[i]` is the i-th observed channel (all channels must be equal
    /// length). Returns `num_components` separated signals.
    pub fn separate(&self, mixed: &[Vec<f64>]) -> Vec<Vec<f64>> {
        if mixed.is_empty() {
            return Vec::new();
        }
        let num_channels = mixed.len();
        let n_samples = mixed[0].len();
        if n_samples == 0 {
            return vec![Vec::new(); self.num_components];
        }

        let num_comp = self.num_components.min(num_channels);

        // -- Center the data (subtract mean per channel) --
        let mut centered = vec![vec![0.0f64; n_samples]; num_channels];
        for (ch, row) in mixed.iter().enumerate() {
            let mean = row.iter().sum::<f64>() / n_samples as f64;
            for (j, &v) in row.iter().enumerate() {
                centered[ch][j] = v - mean;
            }
        }

        // -- Compute covariance matrix --
        let mut cov = vec![vec![0.0f64; num_channels]; num_channels];
        for i in 0..num_channels {
            for j in i..num_channels {
                let mut s = 0.0;
                for k in 0..n_samples {
                    s += centered[i][k] * centered[j][k];
                }
                s /= n_samples as f64;
                cov[i][j] = s;
                cov[j][i] = s;
            }
        }

        // -- Eigendecomposition via Jacobi iteration (symmetric) --
        let (eigenvalues, eigenvectors) = symmetric_eigen(&cov);

        // -- Whitening matrix: D^{-1/2} * E^T --
        // Sort by descending eigenvalue, take top num_comp.
        let mut idx: Vec<usize> = (0..num_channels).collect();
        idx.sort_by(|&a, &b| eigenvalues[b].partial_cmp(&eigenvalues[a]).unwrap_or(std::cmp::Ordering::Equal));

        let mut whitening = vec![vec![0.0f64; num_channels]; num_comp];
        for (row, &ei) in idx.iter().take(num_comp).enumerate() {
            let scale = if eigenvalues[ei] > 1e-12 {
                1.0 / eigenvalues[ei].sqrt()
            } else {
                0.0
            };
            for col in 0..num_channels {
                whitening[row][col] = scale * eigenvectors[col][ei];
            }
        }

        // -- Apply whitening: z = W * centered --
        let mut z = vec![vec![0.0f64; n_samples]; num_comp];
        for i in 0..num_comp {
            for k in 0..n_samples {
                let mut s = 0.0;
                for c in 0..num_channels {
                    s += whitening[i][c] * centered[c][k];
                }
                z[i][k] = s;
            }
        }

        // -- Fixed-point FastICA with deflation --
        // Use a deterministic seed for reproducibility.
        let mut w_all = vec![vec![0.0f64; num_comp]; num_comp];
        for p in 0..num_comp {
            // Deterministic initial vector.
            let mut w = vec![0.0f64; num_comp];
            for i in 0..num_comp {
                // Simple hash-like deterministic init.
                w[i] = ((p * 7 + i * 13 + 3) as f64).sin();
            }
            normalize_vec(&mut w);

            for _iter in 0..self.max_iterations {
                // w_new = E{z * g(w^T z)} - E{g'(w^T z)} * w
                // g(u) = tanh(u), g'(u) = 1 - tanh(u)^2
                let mut w_new = vec![0.0f64; num_comp];
                let mut mean_gp = 0.0f64;

                for k in 0..n_samples {
                    let u: f64 = (0..num_comp).map(|i| w[i] * z[i][k]).sum();
                    let g = u.tanh();
                    let gp = 1.0 - g * g;
                    for i in 0..num_comp {
                        w_new[i] += z[i][k] * g;
                    }
                    mean_gp += gp;
                }
                let n_f = n_samples as f64;
                for i in 0..num_comp {
                    w_new[i] = w_new[i] / n_f - (mean_gp / n_f) * w[i];
                }

                // Deflation: orthogonalise against previously found components.
                for prev in 0..p {
                    let dot: f64 = (0..num_comp).map(|i| w_new[i] * w_all[prev][i]).sum();
                    for i in 0..num_comp {
                        w_new[i] -= dot * w_all[prev][i];
                    }
                }

                normalize_vec(&mut w_new);

                // Convergence check.
                let dot: f64 = (0..num_comp).map(|i| w[i] * w_new[i]).sum();
                w = w_new;
                if (dot.abs() - 1.0).abs() < self.tolerance {
                    break;
                }
            }
            w_all[p] = w;
        }

        // -- Compute separated sources: s = W_ica * z --
        let mut sources = vec![vec![0.0f64; n_samples]; num_comp];
        for p in 0..num_comp {
            for k in 0..n_samples {
                let mut s = 0.0;
                for i in 0..num_comp {
                    s += w_all[p][i] * z[i][k];
                }
                sources[p][k] = s;
            }
        }

        sources
    }
}

/// Normalize a vector in-place to unit length. If the vector is zero, leave
/// it unchanged.
fn normalize_vec(v: &mut [f64]) {
    let norm: f64 = v.iter().map(|&x| x * x).sum::<f64>().sqrt();
    if norm > 1e-15 {
        for x in v.iter_mut() {
            *x /= norm;
        }
    }
}

/// Symmetric eigendecomposition via Jacobi rotation.
///
/// Returns `(eigenvalues, eigenvectors)` where `eigenvectors[col][row]`
/// gives column vectors.
fn symmetric_eigen(mat: &[Vec<f64>]) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = mat.len();
    if n == 0 {
        return (Vec::new(), Vec::new());
    }

    // Clone into working matrix.
    let mut a: Vec<Vec<f64>> = mat.to_vec();
    // Eigenvector matrix (identity initially).
    let mut v = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        v[i][i] = 1.0;
    }

    let max_sweeps = 100;
    for _sweep in 0..max_sweeps {
        // Find the off-diagonal element with largest absolute value.
        let mut max_val = 0.0f64;
        let mut p = 0;
        let mut q = 1;
        for i in 0..n {
            for j in (i + 1)..n {
                if a[i][j].abs() > max_val {
                    max_val = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max_val < 1e-15 {
            break;
        }

        // Compute rotation angle.
        let theta = if (a[p][p] - a[q][q]).abs() < 1e-30 {
            PI / 4.0
        } else {
            0.5 * ((2.0 * a[p][q]) / (a[p][p] - a[q][q])).atan()
        };
        let c = theta.cos();
        let s = theta.sin();

        // Apply Givens rotation to A.
        let mut a_new = a.clone();
        for i in 0..n {
            if i != p && i != q {
                a_new[i][p] = c * a[i][p] + s * a[i][q];
                a_new[p][i] = a_new[i][p];
                a_new[i][q] = -s * a[i][p] + c * a[i][q];
                a_new[q][i] = a_new[i][q];
            }
        }
        a_new[p][p] = c * c * a[p][p] + 2.0 * s * c * a[p][q] + s * s * a[q][q];
        a_new[q][q] = s * s * a[p][p] - 2.0 * s * c * a[p][q] + c * c * a[q][q];
        a_new[p][q] = 0.0;
        a_new[q][p] = 0.0;
        a = a_new;

        // Rotate eigenvectors.
        for i in 0..n {
            let vip = v[i][p];
            let viq = v[i][q];
            v[i][p] = c * vip + s * viq;
            v[i][q] = -s * vip + c * viq;
        }
    }

    let eigenvalues: Vec<f64> = (0..n).map(|i| a[i][i]).collect();
    (eigenvalues, v)
}

// ---------------------------------------------------------------------------
// MUAP detection
// ---------------------------------------------------------------------------

/// A detected Motor Unit Action Potential.
#[derive(Debug, Clone)]
pub struct MuapDetection {
    /// Sample index where the MUAP onset was detected.
    pub onset_index: usize,
    /// Sample index of the peak amplitude.
    pub peak_index: usize,
    /// Peak amplitude (absolute value).
    pub amplitude: f64,
    /// Duration in samples from onset to the end of the MUAP.
    pub duration_samples: usize,
}

/// Detect Motor Unit Action Potentials in a single-channel EMG signal using
/// simple threshold crossing.
///
/// `threshold` is an absolute amplitude threshold. Returns a list of detected
/// MUAPs sorted by onset index.
pub fn detect_muap(signal: &[f64], threshold: f64) -> Vec<MuapDetection> {
    if signal.is_empty() {
        return Vec::new();
    }

    let mut detections = Vec::new();
    let n = signal.len();
    let mut i = 0;

    while i < n {
        if signal[i].abs() >= threshold {
            let onset = i;
            let mut peak_idx = i;
            let mut peak_amp = signal[i].abs();

            // Scan forward while above a fraction of the threshold (hysteresis).
            let hyst = threshold * 0.5;
            while i < n && signal[i].abs() >= hyst {
                if signal[i].abs() > peak_amp {
                    peak_amp = signal[i].abs();
                    peak_idx = i;
                }
                i += 1;
            }

            let end = i;
            detections.push(MuapDetection {
                onset_index: onset,
                peak_index: peak_idx,
                amplitude: peak_amp,
                duration_samples: end - onset,
            });
        } else {
            i += 1;
        }
    }

    detections
}

// ---------------------------------------------------------------------------
// Template matching
// ---------------------------------------------------------------------------

/// Compute the normalized cross-correlation between a signal segment and a
/// template.
///
/// Returns a value in [-1, 1]. Returns 0.0 if either input is empty, has
/// zero energy, or the lengths differ.
pub fn template_match_score(signal_segment: &[f64], template: &[f64]) -> f64 {
    if signal_segment.is_empty() || template.is_empty() {
        return 0.0;
    }
    if signal_segment.len() != template.len() {
        return 0.0;
    }

    let n = signal_segment.len() as f64;
    let mean_s: f64 = signal_segment.iter().sum::<f64>() / n;
    let mean_t: f64 = template.iter().sum::<f64>() / n;

    let mut num = 0.0f64;
    let mut den_s = 0.0f64;
    let mut den_t = 0.0f64;

    for i in 0..signal_segment.len() {
        let ds = signal_segment[i] - mean_s;
        let dt = template[i] - mean_t;
        num += ds * dt;
        den_s += ds * ds;
        den_t += dt * dt;
    }

    let den = (den_s * den_t).sqrt();
    if den < 1e-15 {
        return 0.0;
    }
    num / den
}

// ---------------------------------------------------------------------------
// Bandpass filter (Butterworth approximation)
// ---------------------------------------------------------------------------

/// Apply a simple second-order Butterworth-approximation bandpass filter to
/// a signal.
///
/// Implements a cascade of a high-pass and a low-pass second-order IIR
/// section using the bilinear transform. Processes the signal forward and
/// then backward (zero-phase, `filtfilt`-style) for zero phase distortion.
///
/// Typical EMG bandpass: `low_hz = 20.0`, `high_hz = 450.0`.
pub fn bandpass_emg(signal: &[f64], fs: f64, low_hz: f64, high_hz: f64) -> Vec<f64> {
    if signal.is_empty() || fs <= 0.0 {
        return signal.to_vec();
    }

    // High-pass to remove DC / motion artifacts.
    let hp = biquad_highpass(signal, fs, low_hz);
    // Low-pass to remove high-frequency noise.
    let lp = biquad_lowpass(&hp, fs, high_hz);

    // Zero-phase: reverse, filter again, reverse.
    let mut rev: Vec<f64> = lp.into_iter().rev().collect();
    rev = biquad_highpass(&rev, fs, low_hz);
    rev = biquad_lowpass(&rev, fs, high_hz);
    rev.reverse();
    rev
}

/// Second-order Butterworth low-pass filter (bilinear transform).
fn biquad_lowpass(signal: &[f64], fs: f64, fc: f64) -> Vec<f64> {
    let wc = (PI * fc / fs).tan();
    let wc2 = wc * wc;
    let sqrt2 = std::f64::consts::SQRT_2;
    let k = 1.0 + sqrt2 * wc + wc2;

    let b0 = wc2 / k;
    let b1 = 2.0 * wc2 / k;
    let b2 = wc2 / k;
    let a1 = 2.0 * (wc2 - 1.0) / k;
    let a2 = (1.0 - sqrt2 * wc + wc2) / k;

    iir_filter_2nd(signal, b0, b1, b2, a1, a2)
}

/// Second-order Butterworth high-pass filter (bilinear transform).
fn biquad_highpass(signal: &[f64], fs: f64, fc: f64) -> Vec<f64> {
    let wc = (PI * fc / fs).tan();
    let wc2 = wc * wc;
    let sqrt2 = std::f64::consts::SQRT_2;
    let k = 1.0 + sqrt2 * wc + wc2;

    let b0 = 1.0 / k;
    let b1 = -2.0 / k;
    let b2 = 1.0 / k;
    let a1 = 2.0 * (wc2 - 1.0) / k;
    let a2 = (1.0 - sqrt2 * wc + wc2) / k;

    iir_filter_2nd(signal, b0, b1, b2, a1, a2)
}

/// Apply a second-order IIR filter (Direct Form I).
fn iir_filter_2nd(
    signal: &[f64],
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
) -> Vec<f64> {
    let n = signal.len();
    let mut out = vec![0.0f64; n];
    for i in 0..n {
        let x0 = signal[i];
        let x1 = if i >= 1 { signal[i - 1] } else { 0.0 };
        let x2 = if i >= 2 { signal[i - 2] } else { 0.0 };
        let y1 = if i >= 1 { out[i - 1] } else { 0.0 };
        let y2 = if i >= 2 { out[i - 2] } else { 0.0 };
        out[i] = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    }
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    // -----------------------------------------------------------------------
    // RMS tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rms_sine_wave() {
        // RMS of a sine wave with amplitude A is A / sqrt(2).
        let n = 10_000;
        let amplitude = 3.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| amplitude * (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let rms = compute_rms(&signal);
        let expected = amplitude / 2.0_f64.sqrt();
        assert!(
            (rms - expected).abs() < 0.01,
            "RMS of sine wave: got {}, expected {}",
            rms,
            expected
        );
    }

    #[test]
    fn test_rms_dc_signal() {
        let signal = vec![5.0; 100];
        let rms = compute_rms(&signal);
        assert!((rms - 5.0).abs() < TOL);
    }

    #[test]
    fn test_rms_empty() {
        assert_eq!(compute_rms(&[]), 0.0);
    }

    #[test]
    fn test_rms_single_sample() {
        assert!((compute_rms(&[7.0]) - 7.0).abs() < TOL);
    }

    #[test]
    fn test_rms_all_zeros() {
        let signal = vec![0.0; 50];
        assert_eq!(compute_rms(&signal), 0.0);
    }

    // -----------------------------------------------------------------------
    // MAV tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mav_known() {
        let signal = vec![1.0, -2.0, 3.0, -4.0, 5.0];
        let mav = compute_mav(&signal);
        let expected = (1.0 + 2.0 + 3.0 + 4.0 + 5.0) / 5.0;
        assert!((mav - expected).abs() < TOL);
    }

    #[test]
    fn test_mav_empty() {
        assert_eq!(compute_mav(&[]), 0.0);
    }

    #[test]
    fn test_mav_all_positive() {
        let signal = vec![2.0, 4.0, 6.0];
        assert!((compute_mav(&signal) - 4.0).abs() < TOL);
    }

    // -----------------------------------------------------------------------
    // Zero crossing tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_zero_crossings_alternating() {
        // +1, -1, +1, -1, ... => n-1 crossings
        let signal: Vec<f64> = (0..10).map(|i| if i % 2 == 0 { 1.0 } else { -1.0 }).collect();
        assert_eq!(count_zero_crossings(&signal), 9);
    }

    #[test]
    fn test_zero_crossings_no_crossings() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        assert_eq!(count_zero_crossings(&signal), 0);
    }

    #[test]
    fn test_zero_crossings_empty() {
        assert_eq!(count_zero_crossings(&[]), 0);
    }

    #[test]
    fn test_zero_crossings_single_sample() {
        assert_eq!(count_zero_crossings(&[5.0]), 0);
    }

    // -----------------------------------------------------------------------
    // Waveform length tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_waveform_length_ramp() {
        // Linear ramp from 0 to 9 => each step is 1.0, 9 steps => WL = 9.0
        let signal: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let wl = compute_waveform_length(&signal);
        assert!((wl - 9.0).abs() < TOL);
    }

    #[test]
    fn test_waveform_length_constant() {
        let signal = vec![3.0; 20];
        assert_eq!(compute_waveform_length(&signal), 0.0);
    }

    #[test]
    fn test_waveform_length_empty() {
        assert_eq!(compute_waveform_length(&[]), 0.0);
    }

    #[test]
    fn test_waveform_length_single_sample() {
        assert_eq!(compute_waveform_length(&[42.0]), 0.0);
    }

    // -----------------------------------------------------------------------
    // Slope sign change tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ssc_zigzag() {
        // 0, 2, 0, 2, 0 => slopes: +2, -2, +2, -2 => 3 sign changes
        let signal = vec![0.0, 2.0, 0.0, 2.0, 0.0];
        assert_eq!(count_slope_sign_changes(&signal), 3);
    }

    #[test]
    fn test_ssc_monotone() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert_eq!(count_slope_sign_changes(&signal), 0);
    }

    #[test]
    fn test_ssc_too_short() {
        assert_eq!(count_slope_sign_changes(&[1.0, 2.0]), 0);
        assert_eq!(count_slope_sign_changes(&[1.0]), 0);
        assert_eq!(count_slope_sign_changes(&[]), 0);
    }

    // -----------------------------------------------------------------------
    // Feature extraction
    // -----------------------------------------------------------------------

    #[test]
    fn test_extract_features_multi_channel() {
        let ch0 = vec![1.0, -1.0, 1.0, -1.0];
        let ch1 = vec![0.0, 1.0, 2.0, 3.0];
        let window = vec![ch0, ch1];
        let feats = extract_features(&window);

        assert_eq!(feats.num_channels(), 2);
        assert_eq!(feats.rms.len(), 2);
        assert_eq!(feats.mean_absolute_value.len(), 2);
        assert_eq!(feats.zero_crossings.len(), 2);
        assert_eq!(feats.waveform_length.len(), 2);
        assert_eq!(feats.slope_sign_changes.len(), 2);
    }

    #[test]
    fn test_extract_features_consistency() {
        let ch = vec![1.0, -2.0, 3.0, -4.0, 5.0];
        let window = vec![ch.clone()];
        let feats = extract_features(&window);

        assert!((feats.rms[0] - compute_rms(&ch)).abs() < TOL);
        assert!((feats.mean_absolute_value[0] - compute_mav(&ch)).abs() < TOL);
        assert_eq!(feats.zero_crossings[0], count_zero_crossings(&ch));
        assert!((feats.waveform_length[0] - compute_waveform_length(&ch)).abs() < TOL);
        assert_eq!(feats.slope_sign_changes[0], count_slope_sign_changes(&ch));
    }

    #[test]
    fn test_feature_vector_flat() {
        let window = vec![vec![1.0, 2.0, 3.0]];
        let feats = extract_features(&window);
        let flat = feats.to_flat_vec();
        // 5 features * 1 channel = 5 elements
        assert_eq!(flat.len(), 5);
    }

    // -----------------------------------------------------------------------
    // k-NN classifier
    // -----------------------------------------------------------------------

    #[test]
    fn test_knn_simple_classification() {
        let mut classifier = EmgClassifier::new(3);

        // Create two distinct gesture classes.
        let class0: Vec<Vec<f64>> = (0..5).map(|_| vec![1.0; 100]).collect();
        let class1: Vec<Vec<f64>> = (0..5).map(|_| vec![10.0; 100]).collect();

        let feats: Vec<EmgFeatureVector> = class0
            .iter()
            .chain(class1.iter())
            .map(|ch| extract_features(&[ch.clone()]))
            .collect();
        let labels: Vec<usize> = (0..5).map(|_| 0).chain((0..5).map(|_| 1)).collect();

        classifier.train(&feats, &labels);

        // Query close to class 0.
        let q = extract_features(&[vec![1.1; 100]]);
        let result = classifier.classify(&q);
        assert_eq!(result.gesture_index, 0);
        assert!(result.confidence > 0.5);
    }

    #[test]
    fn test_knn_confidence() {
        let mut classifier = EmgClassifier::new(5);

        // All training samples are class 0.
        let feats: Vec<EmgFeatureVector> = (0..5)
            .map(|_| extract_features(&[vec![1.0; 50]]))
            .collect();
        let labels = vec![0; 5];

        classifier.train(&feats, &labels);

        let q = extract_features(&[vec![1.0; 50]]);
        let result = classifier.classify(&q);
        assert_eq!(result.gesture_index, 0);
        // All 5 neighbors vote for class 0 => confidence = 1.0.
        assert!((result.confidence - 1.0).abs() < TOL);
    }

    #[test]
    fn test_knn_distances_sorted() {
        let mut classifier = EmgClassifier::new(3);

        let feats: Vec<EmgFeatureVector> = vec![
            extract_features(&[vec![1.0; 20]]),
            extract_features(&[vec![2.0; 20]]),
            extract_features(&[vec![3.0; 20]]),
        ];
        let labels = vec![0, 0, 1];

        classifier.train(&feats, &labels);

        let q = extract_features(&[vec![1.5; 20]]);
        let result = classifier.classify(&q);

        // Distances should be sorted ascending.
        for i in 1..result.distances.len() {
            assert!(result.distances[i] >= result.distances[i - 1]);
        }
    }

    #[test]
    #[should_panic(expected = "classifier must be trained")]
    fn test_knn_classify_untrained() {
        let classifier = EmgClassifier::new(3);
        let q = extract_features(&[vec![1.0; 10]]);
        classifier.classify(&q);
    }

    // -----------------------------------------------------------------------
    // FastICA
    // -----------------------------------------------------------------------

    #[test]
    fn test_fastica_two_sines() {
        // Mix two sinusoids at different frequencies and attempt separation.
        let n = 2000;
        let s1: Vec<f64> = (0..n).map(|i| (2.0 * PI * 3.0 * i as f64 / n as f64).sin()).collect();
        let s2: Vec<f64> = (0..n).map(|i| (2.0 * PI * 7.0 * i as f64 / n as f64).sin()).collect();

        // Mixing matrix.
        let mix0: Vec<f64> = (0..n).map(|i| 0.8 * s1[i] + 0.6 * s2[i]).collect();
        let mix1: Vec<f64> = (0..n).map(|i| 0.4 * s1[i] + 0.9 * s2[i]).collect();

        let ica = FastIcaSeparator::new(2);
        let separated = ica.separate(&[mix0, mix1]);

        assert_eq!(separated.len(), 2);
        assert_eq!(separated[0].len(), n);

        // Verify separation quality: each output should correlate strongly
        // with one of the original sources (up to sign/scale ambiguity).
        let corr_00 = template_match_score(&separated[0], &s1).abs();
        let corr_01 = template_match_score(&separated[0], &s2).abs();
        let corr_10 = template_match_score(&separated[1], &s1).abs();
        let corr_11 = template_match_score(&separated[1], &s2).abs();

        // At least one output should match each source with correlation > 0.8.
        let best_s1 = corr_00.max(corr_10);
        let best_s2 = corr_01.max(corr_11);
        assert!(
            best_s1 > 0.8,
            "FastICA failed to recover source 1: best corr = {}",
            best_s1
        );
        assert!(
            best_s2 > 0.8,
            "FastICA failed to recover source 2: best corr = {}",
            best_s2
        );
    }

    #[test]
    fn test_fastica_empty_input() {
        let ica = FastIcaSeparator::new(2);
        let result = ica.separate(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_fastica_empty_channels() {
        let ica = FastIcaSeparator::new(2);
        let result = ica.separate(&[vec![], vec![]]);
        assert_eq!(result.len(), 2);
        assert!(result[0].is_empty());
    }

    // -----------------------------------------------------------------------
    // MUAP detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_muap_synthetic_biphasic() {
        // Create a synthetic biphasic MUAP: positive then negative pulse.
        let mut signal = vec![0.0f64; 200];
        // First MUAP at sample 50.
        for i in 50..60 {
            signal[i] = 2.0 * (PI * (i - 50) as f64 / 10.0).sin();
        }
        for i in 60..70 {
            signal[i] = -1.5 * (PI * (i - 60) as f64 / 10.0).sin();
        }
        // Second MUAP at sample 150.
        for i in 150..160 {
            signal[i] = 3.0 * (PI * (i - 150) as f64 / 10.0).sin();
        }

        let detections = detect_muap(&signal, 1.0);
        assert!(
            detections.len() >= 2,
            "Expected at least 2 MUAPs, got {}",
            detections.len()
        );
        assert!(detections[0].onset_index <= 55);
        assert!(detections[0].amplitude >= 1.0);
    }

    #[test]
    fn test_muap_empty_signal() {
        let detections = detect_muap(&[], 1.0);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_muap_below_threshold() {
        let signal = vec![0.1, 0.2, 0.1, -0.1, -0.2, -0.1];
        let detections = detect_muap(&signal, 1.0);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_muap_duration() {
        // Single pulse.
        let mut signal = vec![0.0; 100];
        for i in 20..30 {
            signal[i] = 5.0;
        }
        let detections = detect_muap(&signal, 1.0);
        assert_eq!(detections.len(), 1);
        assert!(detections[0].duration_samples >= 10);
    }

    // -----------------------------------------------------------------------
    // Template matching
    // -----------------------------------------------------------------------

    #[test]
    fn test_template_perfect_match() {
        let template = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let score = template_match_score(&template, &template);
        assert!(
            (score - 1.0).abs() < 1e-10,
            "Perfect match score should be 1.0, got {}",
            score
        );
    }

    #[test]
    fn test_template_negated_match() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b: Vec<f64> = a.iter().map(|&x| -x).collect();
        let score = template_match_score(&a, &b);
        assert!(
            (score + 1.0).abs() < 1e-10,
            "Negated match score should be -1.0, got {}",
            score
        );
    }

    #[test]
    fn test_template_uncorrelated() {
        // Two signals with very different structure.
        let n = 1000;
        let a: Vec<f64> = (0..n).map(|i| (2.0 * PI * 3.0 * i as f64 / n as f64).sin()).collect();
        let b: Vec<f64> = (0..n).map(|i| (2.0 * PI * 17.0 * i as f64 / n as f64).sin()).collect();
        let score = template_match_score(&a, &b);
        assert!(
            score.abs() < 0.15,
            "Uncorrelated signals should have near-zero correlation, got {}",
            score
        );
    }

    #[test]
    fn test_template_empty() {
        assert_eq!(template_match_score(&[], &[]), 0.0);
    }

    #[test]
    fn test_template_length_mismatch() {
        assert_eq!(template_match_score(&[1.0, 2.0], &[1.0]), 0.0);
    }

    #[test]
    fn test_template_constant_signal() {
        // Constant signal has zero variance => score should be 0.
        let a = vec![5.0; 10];
        let b = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        assert_eq!(template_match_score(&a, &b), 0.0);
    }

    // -----------------------------------------------------------------------
    // Bandpass filter
    // -----------------------------------------------------------------------

    #[test]
    fn test_bandpass_passes_inband() {
        // Generate a 100 Hz tone at 1000 Hz sample rate.
        let fs = 1000.0;
        let n = 2000;
        let freq = 100.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / fs).sin())
            .collect();

        let filtered = bandpass_emg(&signal, fs, 20.0, 450.0);

        // Compute power ratio: filtered should retain most energy.
        let power_in: f64 = signal.iter().map(|x| x * x).sum::<f64>();
        let power_out: f64 = filtered.iter().map(|x| x * x).sum::<f64>();
        let ratio = power_out / power_in;
        assert!(
            ratio > 0.5,
            "In-band signal should pass through filter, power ratio = {}",
            ratio
        );
    }

    #[test]
    fn test_bandpass_attenuates_out_of_band() {
        // Generate a 5 Hz tone (below EMG band) at 1000 Hz sample rate.
        let fs = 1000.0;
        let n = 4000;
        let freq = 5.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / fs).sin())
            .collect();

        let filtered = bandpass_emg(&signal, fs, 20.0, 450.0);

        let power_in: f64 = signal.iter().map(|x| x * x).sum::<f64>();
        let power_out: f64 = filtered.iter().map(|x| x * x).sum::<f64>();
        let ratio = power_out / power_in;
        assert!(
            ratio < 0.2,
            "Out-of-band signal should be attenuated, power ratio = {}",
            ratio
        );
    }

    #[test]
    fn test_bandpass_empty() {
        let result = bandpass_emg(&[], 1000.0, 20.0, 450.0);
        assert!(result.is_empty());
    }

    // -----------------------------------------------------------------------
    // Config
    // -----------------------------------------------------------------------

    #[test]
    fn test_config_window_samples() {
        let config = EmgGestureConfig {
            num_channels: 4,
            sample_rate_hz: 1000.0,
            window_size_ms: 200.0,
            gesture_labels: vec!["rest".into()],
        };
        assert_eq!(config.window_samples(), 200);
    }

    // -----------------------------------------------------------------------
    // Euclidean distance (internal)
    // -----------------------------------------------------------------------

    #[test]
    fn test_euclidean_distance_known() {
        let a = vec![0.0, 0.0];
        let b = vec![3.0, 4.0];
        assert!((euclidean_distance(&a, &b) - 5.0).abs() < TOL);
    }

    #[test]
    fn test_euclidean_distance_same() {
        let a = vec![1.0, 2.0, 3.0];
        assert!(euclidean_distance(&a, &a) < TOL);
    }
}

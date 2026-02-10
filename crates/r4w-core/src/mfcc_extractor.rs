//! Mel-Frequency Cepstral Coefficient (MFCC) extraction for acoustic and speech feature analysis.
//!
//! This module provides a configurable MFCC pipeline that converts raw audio samples into
//! compact spectral feature vectors widely used in speech recognition, speaker identification,
//! and acoustic event classification.
//!
//! The processing chain is:
//! 1. Pre-emphasis high-pass filter
//! 2. Framing with configurable hop size
//! 3. Windowing (Hamming)
//! 4. Power spectrum via DFT
//! 5. Mel-scale triangular filterbank
//! 6. Log energy computation
//! 7. DCT-II for cepstral coefficients
//! 8. Optional delta and delta-delta features
//! 9. Optional cepstral mean normalization (CMN)
//!
//! # Example
//!
//! ```
//! use r4w_core::mfcc_extractor::{MfccExtractor, MfccConfig};
//!
//! let config = MfccConfig {
//!     sample_rate: 16000.0,
//!     fft_size: 512,
//!     num_mel_filters: 26,
//!     num_coefficients: 13,
//!     pre_emphasis: 0.97,
//!     hop_size: 160,
//!     frame_size: 400,
//!     low_freq: 0.0,
//!     high_freq: 8000.0,
//!     include_deltas: false,
//!     apply_cmn: false,
//! };
//!
//! let extractor = MfccExtractor::new(config);
//!
//! // Generate a simple 400-sample frame (25 ms at 16 kHz)
//! let signal: Vec<f64> = (0..400)
//!     .map(|i| (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 16000.0).sin())
//!     .collect();
//!
//! let mfccs = extractor.extract_frame(&signal);
//! assert_eq!(mfccs.len(), 13);
//! ```

use std::f64::consts::PI;

/// Configuration parameters for MFCC extraction.
#[derive(Debug, Clone)]
pub struct MfccConfig {
    /// Audio sample rate in Hz.
    pub sample_rate: f64,
    /// FFT size (number of DFT bins). Should be a power of 2 for efficiency,
    /// but any positive value is accepted.
    pub fft_size: usize,
    /// Number of triangular mel-scale filters in the filterbank.
    pub num_mel_filters: usize,
    /// Number of cepstral coefficients to return (typically 12-13).
    pub num_coefficients: usize,
    /// Pre-emphasis filter coefficient (0.0 to disable, typically 0.95-0.97).
    pub pre_emphasis: f64,
    /// Hop size in samples between consecutive frames.
    pub hop_size: usize,
    /// Frame size in samples (the analysis window length).
    pub frame_size: usize,
    /// Lower frequency edge of the mel filterbank in Hz.
    pub low_freq: f64,
    /// Upper frequency edge of the mel filterbank in Hz.
    pub high_freq: f64,
    /// Whether to append delta and delta-delta coefficients.
    pub include_deltas: bool,
    /// Whether to apply cepstral mean normalization across frames.
    pub apply_cmn: bool,
}

impl Default for MfccConfig {
    fn default() -> Self {
        Self {
            sample_rate: 16000.0,
            fft_size: 512,
            num_mel_filters: 26,
            num_coefficients: 13,
            pre_emphasis: 0.97,
            hop_size: 160,
            frame_size: 400,
            low_freq: 0.0,
            high_freq: 8000.0,
            include_deltas: false,
            apply_cmn: false,
        }
    }
}

/// A single MFCC feature frame containing cepstral coefficients and optional
/// delta/delta-delta features.
#[derive(Debug, Clone)]
pub struct MfccFrame {
    /// The cepstral coefficients for this frame.
    pub coefficients: Vec<f64>,
    /// Delta (velocity) coefficients, if computed.
    pub deltas: Option<Vec<f64>>,
    /// Delta-delta (acceleration) coefficients, if computed.
    pub delta_deltas: Option<Vec<f64>>,
}

/// MFCC feature extractor.
///
/// Construct with [`MfccExtractor::new`], then call [`MfccExtractor::extract_frame`]
/// for single-frame extraction or [`MfccExtractor::extract`] for multi-frame
/// extraction from a full signal.
pub struct MfccExtractor {
    config: MfccConfig,
    /// Pre-computed mel filterbank matrix: `num_mel_filters` rows,
    /// each row has `fft_size / 2 + 1` weights.
    filterbank: Vec<Vec<f64>>,
    /// Pre-computed Hamming window of length `frame_size`.
    window: Vec<f64>,
}

impl MfccExtractor {
    /// Create a new MFCC extractor with the given configuration.
    ///
    /// This pre-computes the mel filterbank and the Hamming window.
    pub fn new(config: MfccConfig) -> Self {
        let window = hamming_window(config.frame_size);
        let filterbank = build_mel_filterbank(
            config.num_mel_filters,
            config.fft_size,
            config.sample_rate,
            config.low_freq,
            config.high_freq,
        );
        Self {
            config,
            filterbank,
            window,
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &MfccConfig {
        &self.config
    }

    /// Return a reference to the pre-computed mel filterbank.
    pub fn filterbank(&self) -> &Vec<Vec<f64>> {
        &self.filterbank
    }

    /// Apply pre-emphasis to a signal, returning a new vector.
    ///
    /// `y[n] = x[n] - coeff * x[n-1]`
    pub fn pre_emphasize(&self, signal: &[f64]) -> Vec<f64> {
        pre_emphasis_filter(signal, self.config.pre_emphasis)
    }

    /// Extract MFCC coefficients from a single frame of samples.
    ///
    /// The input slice should have at least `frame_size` samples.
    /// If shorter, it is zero-padded. If longer, only the first `frame_size`
    /// samples are used.
    pub fn extract_frame(&self, frame: &[f64]) -> Vec<f64> {
        let n = self.config.frame_size;
        let fft_size = self.config.fft_size;

        // Apply window
        let mut windowed = vec![0.0; n];
        for i in 0..n {
            let sample = if i < frame.len() { frame[i] } else { 0.0 };
            windowed[i] = sample * self.window[i];
        }

        // Compute power spectrum
        let power_spec = power_spectrum(&windowed, fft_size);

        // Apply mel filterbank
        let mel_energies = apply_filterbank(&power_spec, &self.filterbank);

        // Log energies (floor to avoid log(0))
        let log_energies: Vec<f64> = mel_energies
            .iter()
            .map(|&e| (e.max(1e-22)).ln())
            .collect();

        // DCT-II to get cepstral coefficients
        let cepstral = dct_ii(&log_energies);

        // Take only the requested number of coefficients
        cepstral[..self.config.num_coefficients.min(cepstral.len())].to_vec()
    }

    /// Extract MFCC features from an entire signal, frame by frame.
    ///
    /// Returns a vector of [`MfccFrame`] structs. If `include_deltas` is set
    /// in the config, delta and delta-delta coefficients are appended.
    /// If `apply_cmn` is set, cepstral mean normalization is applied across
    /// all frames.
    pub fn extract(&self, signal: &[f64]) -> Vec<MfccFrame> {
        // Pre-emphasis
        let emphasized = if self.config.pre_emphasis > 0.0 {
            self.pre_emphasize(signal)
        } else {
            signal.to_vec()
        };

        // Frame the signal
        let frames = frame_signal(
            &emphasized,
            self.config.frame_size,
            self.config.hop_size,
        );

        // Extract MFCCs for each frame
        let mut mfcc_matrix: Vec<Vec<f64>> = frames
            .iter()
            .map(|f| self.extract_frame(f))
            .collect();

        // Cepstral mean normalization
        if self.config.apply_cmn {
            apply_cmn(&mut mfcc_matrix);
        }

        // Build MfccFrame structs with optional deltas
        if self.config.include_deltas && mfcc_matrix.len() >= 3 {
            let deltas = compute_deltas(&mfcc_matrix, 2);
            let delta_deltas = compute_deltas(&deltas, 2);

            mfcc_matrix
                .into_iter()
                .zip(deltas.into_iter())
                .zip(delta_deltas.into_iter())
                .map(|((coefficients, d), dd)| MfccFrame {
                    coefficients,
                    deltas: Some(d),
                    delta_deltas: Some(dd),
                })
                .collect()
        } else {
            mfcc_matrix
                .into_iter()
                .map(|coefficients| MfccFrame {
                    coefficients,
                    deltas: None,
                    delta_deltas: None,
                })
                .collect()
        }
    }
}

// ---------------------------------------------------------------------------
// Public free functions
// ---------------------------------------------------------------------------

/// Convert a frequency in Hz to the mel scale.
///
/// Uses the O'Shaughnessy formula: `mel = 2595 * log10(1 + hz / 700)`.
pub fn hz_to_mel(hz: f64) -> f64 {
    2595.0 * (1.0 + hz / 700.0).log10()
}

/// Convert a mel-scale value back to Hz.
///
/// Inverse of [`hz_to_mel`]: `hz = 700 * (10^(mel / 2595) - 1)`.
pub fn mel_to_hz(mel: f64) -> f64 {
    700.0 * (10.0_f64.powf(mel / 2595.0) - 1.0)
}

/// Apply a pre-emphasis high-pass filter to the signal.
///
/// `y[n] = x[n] - coeff * x[n-1]`, with `y[0] = x[0]`.
pub fn pre_emphasis_filter(signal: &[f64], coeff: f64) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(signal.len());
    out.push(signal[0]);
    for i in 1..signal.len() {
        out.push(signal[i] - coeff * signal[i - 1]);
    }
    out
}

/// Compute the power spectrum of a real signal using a direct DFT.
///
/// The input is zero-padded (or truncated) to `fft_size`. Returns
/// `fft_size / 2 + 1` power values (the non-negative frequency bins).
pub fn power_spectrum(signal: &[f64], fft_size: usize) -> Vec<f64> {
    let spectrum = real_dft(signal, fft_size);
    let n_bins = fft_size / 2 + 1;
    let scale = 1.0 / (fft_size as f64);
    spectrum[..n_bins]
        .iter()
        .map(|&(re, im)| (re * re + im * im) * scale)
        .collect()
}

/// Build a mel-scale triangular filterbank.
///
/// Returns a matrix of `num_filters` rows, each of length `fft_size / 2 + 1`.
pub fn build_mel_filterbank(
    num_filters: usize,
    fft_size: usize,
    sample_rate: f64,
    low_freq: f64,
    high_freq: f64,
) -> Vec<Vec<f64>> {
    let n_bins = fft_size / 2 + 1;
    let mel_low = hz_to_mel(low_freq);
    let mel_high = hz_to_mel(high_freq);

    // Linearly spaced mel points (num_filters + 2 for the edges)
    let num_points = num_filters + 2;
    let mel_points: Vec<f64> = (0..num_points)
        .map(|i| mel_low + (mel_high - mel_low) * i as f64 / (num_points - 1) as f64)
        .collect();

    // Convert mel points back to Hz, then to FFT bin indices
    let hz_points: Vec<f64> = mel_points.iter().map(|&m| mel_to_hz(m)).collect();
    let bin_indices: Vec<usize> = hz_points
        .iter()
        .map(|&hz| {
            let bin = ((fft_size as f64 + 1.0) * hz / sample_rate).floor() as usize;
            bin.min(n_bins.saturating_sub(1))
        })
        .collect();

    let mut filterbank = Vec::with_capacity(num_filters);
    for m in 0..num_filters {
        let mut filt = vec![0.0; n_bins];
        let f_left = bin_indices[m];
        let f_center = bin_indices[m + 1];
        let f_right = bin_indices[m + 2];

        // Rising slope
        if f_center > f_left {
            for k in f_left..=f_center {
                filt[k] = (k - f_left) as f64 / (f_center - f_left) as f64;
            }
        }
        // Falling slope
        if f_right > f_center {
            for k in f_center..=f_right.min(n_bins - 1) {
                filt[k] = (f_right - k) as f64 / (f_right - f_center) as f64;
            }
        }
        filterbank.push(filt);
    }
    filterbank
}

/// Apply a filterbank matrix to a power spectrum, returning mel-band energies.
pub fn apply_filterbank(power_spec: &[f64], filterbank: &[Vec<f64>]) -> Vec<f64> {
    filterbank
        .iter()
        .map(|filt| {
            filt.iter()
                .zip(power_spec.iter())
                .map(|(&w, &p)| w * p)
                .sum()
        })
        .collect()
}

/// Compute the DCT-II of a sequence (unscaled, type-II).
///
/// `X[k] = sum_{n=0}^{N-1} x[n] * cos(PI * (n + 0.5) * k / N)` for k in 0..N.
pub fn dct_ii(input: &[f64]) -> Vec<f64> {
    let n = input.len();
    if n == 0 {
        return Vec::new();
    }
    let mut output = Vec::with_capacity(n);
    for k in 0..n {
        let mut sum = 0.0;
        for (ni, &x) in input.iter().enumerate() {
            sum += x * (PI * (ni as f64 + 0.5) * k as f64 / n as f64).cos();
        }
        output.push(sum);
    }
    output
}

/// Compute delta (differential) coefficients over a sequence of feature vectors.
///
/// Uses a simple regression formula over a window of +/- `win` frames:
/// `d[t] = (sum_{n=1}^{win} n * (c[t+n] - c[t-n])) / (2 * sum_{n=1}^{win} n^2)`
///
/// Edge frames are handled by clamping the indices.
pub fn compute_deltas(features: &[Vec<f64>], win: usize) -> Vec<Vec<f64>> {
    let num_frames = features.len();
    if num_frames == 0 {
        return Vec::new();
    }
    let num_coeffs = features[0].len();
    let denom: f64 = 2.0 * (1..=win).map(|n| (n * n) as f64).sum::<f64>();

    let mut deltas = Vec::with_capacity(num_frames);
    for t in 0..num_frames {
        let mut delta_vec = vec![0.0; num_coeffs];
        for n in 1..=win {
            let t_plus = (t + n).min(num_frames - 1);
            let t_minus = if t >= n { t - n } else { 0 };
            for c in 0..num_coeffs {
                delta_vec[c] += n as f64 * (features[t_plus][c] - features[t_minus][c]);
            }
        }
        if denom > 0.0 {
            for c in 0..num_coeffs {
                delta_vec[c] /= denom;
            }
        }
        deltas.push(delta_vec);
    }
    deltas
}

/// Apply cepstral mean normalization (CMN) in-place.
///
/// Subtracts the mean of each coefficient dimension across all frames.
pub fn apply_cmn(frames: &mut [Vec<f64>]) {
    if frames.is_empty() {
        return;
    }
    let num_frames = frames.len();
    let num_coeffs = frames[0].len();

    // Compute mean for each coefficient
    let mut means = vec![0.0; num_coeffs];
    for frame in frames.iter() {
        for (c, &val) in frame.iter().enumerate() {
            means[c] += val;
        }
    }
    for m in means.iter_mut() {
        *m /= num_frames as f64;
    }

    // Subtract means
    for frame in frames.iter_mut() {
        for (c, val) in frame.iter_mut().enumerate() {
            *val -= means[c];
        }
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the DFT of a real signal, zero-padded or truncated to `fft_size`.
/// Returns `fft_size` complex bins as `(re, im)` tuples.
fn real_dft(signal: &[f64], fft_size: usize) -> Vec<(f64, f64)> {
    let mut padded = vec![0.0; fft_size];
    let copy_len = signal.len().min(fft_size);
    padded[..copy_len].copy_from_slice(&signal[..copy_len]);

    let mut spectrum = Vec::with_capacity(fft_size);
    for k in 0..fft_size {
        let mut re = 0.0;
        let mut im = 0.0;
        for (n, &x) in padded.iter().enumerate() {
            let angle = 2.0 * PI * k as f64 * n as f64 / fft_size as f64;
            re += x * angle.cos();
            im -= x * angle.sin();
        }
        spectrum.push((re, im));
    }
    spectrum
}

/// Generate a Hamming window of the given length.
fn hamming_window(length: usize) -> Vec<f64> {
    (0..length)
        .map(|n| 0.54 - 0.46 * (2.0 * PI * n as f64 / (length as f64 - 1.0)).cos())
        .collect()
}

/// Split a signal into overlapping frames.
fn frame_signal(signal: &[f64], frame_size: usize, hop_size: usize) -> Vec<Vec<f64>> {
    if signal.is_empty() || frame_size == 0 || hop_size == 0 {
        return Vec::new();
    }
    let mut frames = Vec::new();
    let mut start = 0;
    while start + frame_size <= signal.len() {
        frames.push(signal[start..start + frame_size].to_vec());
        start += hop_size;
    }
    frames
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- Hz / Mel conversion ---

    #[test]
    fn test_hz_to_mel_zero() {
        assert!((hz_to_mel(0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_hz_to_mel_1000() {
        // 1000 Hz => 2595 * log10(1 + 1000/700) = 2595 * log10(2.4286) ~= 999.99
        let mel = hz_to_mel(1000.0);
        assert!((mel - 999.9856).abs() < 0.01, "mel={}", mel);
    }

    #[test]
    fn test_mel_to_hz_roundtrip() {
        for &freq in &[0.0, 300.0, 1000.0, 4000.0, 8000.0] {
            let roundtrip = mel_to_hz(hz_to_mel(freq));
            assert!(
                (roundtrip - freq).abs() < 0.01,
                "freq={}, roundtrip={}",
                freq,
                roundtrip
            );
        }
    }

    #[test]
    fn test_mel_scale_monotonic() {
        let freqs: Vec<f64> = (0..20).map(|i| i as f64 * 400.0).collect();
        let mels: Vec<f64> = freqs.iter().map(|&f| hz_to_mel(f)).collect();
        for i in 1..mels.len() {
            assert!(mels[i] > mels[i - 1], "Mel scale not monotonic at {}", i);
        }
    }

    // --- Pre-emphasis ---

    #[test]
    fn test_pre_emphasis_empty() {
        let result = pre_emphasis_filter(&[], 0.97);
        assert!(result.is_empty());
    }

    #[test]
    fn test_pre_emphasis_single_sample() {
        let result = pre_emphasis_filter(&[1.0], 0.97);
        assert_eq!(result.len(), 1);
        assert!((result[0] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_pre_emphasis_known_values() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let result = pre_emphasis_filter(&signal, 0.5);
        // y[0] = 1.0
        // y[1] = 2.0 - 0.5*1.0 = 1.5
        // y[2] = 3.0 - 0.5*2.0 = 2.0
        // y[3] = 4.0 - 0.5*3.0 = 2.5
        assert!((result[0] - 1.0).abs() < EPSILON);
        assert!((result[1] - 1.5).abs() < EPSILON);
        assert!((result[2] - 2.0).abs() < EPSILON);
        assert!((result[3] - 2.5).abs() < EPSILON);
    }

    // --- Hamming window ---

    #[test]
    fn test_hamming_window_endpoints() {
        let w = hamming_window(256);
        assert_eq!(w.len(), 256);
        // Hamming window endpoints are 0.08 (= 0.54 - 0.46)
        assert!((w[0] - 0.08).abs() < EPSILON, "w[0]={}", w[0]);
        assert!((w[255] - 0.08).abs() < EPSILON, "w[255]={}", w[255]);
    }

    #[test]
    fn test_hamming_window_peak() {
        let w = hamming_window(256);
        // Peak should be at the center, value = 1.0
        let mid = 127; // for even-length, close to center
        assert!(w[mid] > 0.99, "w[mid]={}", w[mid]);
    }

    // --- Power spectrum ---

    #[test]
    fn test_power_spectrum_dc() {
        // Constant signal => all energy at DC
        let signal = vec![1.0; 64];
        let ps = power_spectrum(&signal, 64);
        assert_eq!(ps.len(), 33); // 64/2 + 1
        // DC bin should have energy = N^2 / N = N = 64
        assert!(ps[0] > 50.0, "DC power={}", ps[0]);
        // Other bins should be near zero
        for &p in &ps[1..] {
            assert!(p < EPSILON, "Non-DC bin energy={}", p);
        }
    }

    #[test]
    fn test_power_spectrum_length() {
        let signal = vec![0.5; 128];
        let ps = power_spectrum(&signal, 256);
        assert_eq!(ps.len(), 129); // 256/2 + 1
    }

    // --- Mel filterbank ---

    #[test]
    fn test_filterbank_dimensions() {
        let fb = build_mel_filterbank(26, 512, 16000.0, 0.0, 8000.0);
        assert_eq!(fb.len(), 26);
        for filt in &fb {
            assert_eq!(filt.len(), 257); // 512/2 + 1
        }
    }

    #[test]
    fn test_filterbank_non_negative() {
        let fb = build_mel_filterbank(26, 512, 16000.0, 0.0, 8000.0);
        for filt in &fb {
            for &w in filt {
                assert!(w >= 0.0, "Negative filterbank weight: {}", w);
            }
        }
    }

    #[test]
    fn test_filterbank_peak_at_one() {
        let fb = build_mel_filterbank(26, 512, 16000.0, 0.0, 8000.0);
        for (i, filt) in fb.iter().enumerate() {
            let max_val = filt.iter().cloned().fold(0.0_f64, f64::max);
            assert!(
                (max_val - 1.0).abs() < EPSILON,
                "Filter {} peak = {} (expected 1.0)",
                i,
                max_val
            );
        }
    }

    // --- DCT-II ---

    #[test]
    fn test_dct_ii_constant_input() {
        // DCT-II of a constant vector: only the 0th coefficient is nonzero
        let input = vec![1.0; 8];
        let dct = dct_ii(&input);
        assert_eq!(dct.len(), 8);
        // DC: sum of cos(pi*(n+0.5)*0/N) = sum of 1 = N
        assert!((dct[0] - 8.0).abs() < EPSILON, "dct[0]={}", dct[0]);
        for k in 1..8 {
            assert!(dct[k].abs() < EPSILON, "dct[{}]={}", k, dct[k]);
        }
    }

    #[test]
    fn test_dct_ii_empty() {
        let dct = dct_ii(&[]);
        assert!(dct.is_empty());
    }

    #[test]
    fn test_dct_ii_single() {
        let dct = dct_ii(&[3.5]);
        assert_eq!(dct.len(), 1);
        assert!((dct[0] - 3.5).abs() < EPSILON);
    }

    // --- Delta coefficients ---

    #[test]
    fn test_deltas_constant_features() {
        // If all frames are the same, deltas should be zero
        let features = vec![vec![1.0, 2.0, 3.0]; 10];
        let deltas = compute_deltas(&features, 2);
        assert_eq!(deltas.len(), 10);
        for d in &deltas {
            for &val in d {
                assert!(val.abs() < EPSILON, "Expected zero delta, got {}", val);
            }
        }
    }

    #[test]
    fn test_deltas_linear_ramp() {
        // Features linearly increase: frame t has coefficients [t, t, t]
        let features: Vec<Vec<f64>> = (0..10)
            .map(|t| vec![t as f64; 3])
            .collect();
        let deltas = compute_deltas(&features, 2);
        // Interior frames should have delta ~= 1.0 (slope of the ramp)
        for d in &deltas[2..8] {
            for &val in d {
                assert!(
                    (val - 1.0).abs() < EPSILON,
                    "Expected delta ~= 1.0, got {}",
                    val
                );
            }
        }
    }

    // --- Cepstral mean normalization ---

    #[test]
    fn test_cmn_subtracts_mean() {
        let mut frames = vec![
            vec![2.0, 4.0],
            vec![4.0, 6.0],
            vec![6.0, 8.0],
        ];
        apply_cmn(&mut frames);
        // Mean was [4.0, 6.0]
        assert!((frames[0][0] - (-2.0)).abs() < EPSILON);
        assert!((frames[0][1] - (-2.0)).abs() < EPSILON);
        assert!((frames[1][0]).abs() < EPSILON);
        assert!((frames[1][1]).abs() < EPSILON);
        assert!((frames[2][0] - 2.0).abs() < EPSILON);
        assert!((frames[2][1] - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_cmn_empty() {
        let mut frames: Vec<Vec<f64>> = Vec::new();
        apply_cmn(&mut frames); // Should not panic
    }

    // --- Frame signal ---

    #[test]
    fn test_frame_signal_basic() {
        let signal: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let frames = frame_signal(&signal, 40, 20);
        // With length 100, frame_size 40, hop 20:
        // frame 0: [0..40], frame 1: [20..60], frame 2: [40..80], frame 3: [60..100]
        assert_eq!(frames.len(), 4, "Expected 4 frames, got {}", frames.len());
        assert_eq!(frames[0].len(), 40);
        assert!((frames[0][0]).abs() < EPSILON);
        assert!((frames[1][0] - 20.0).abs() < EPSILON);
    }

    // --- Full extraction pipeline ---

    #[test]
    fn test_extract_frame_returns_correct_count() {
        let config = MfccConfig {
            num_coefficients: 13,
            ..MfccConfig::default()
        };
        let extractor = MfccExtractor::new(config);
        let frame = vec![0.0; 400];
        let mfccs = extractor.extract_frame(&frame);
        assert_eq!(mfccs.len(), 13);
    }

    #[test]
    fn test_extract_multi_frame_with_deltas() {
        let config = MfccConfig {
            sample_rate: 16000.0,
            fft_size: 256,
            num_mel_filters: 20,
            num_coefficients: 10,
            pre_emphasis: 0.97,
            hop_size: 80,
            frame_size: 160,
            low_freq: 0.0,
            high_freq: 8000.0,
            include_deltas: true,
            apply_cmn: false,
        };
        let extractor = MfccExtractor::new(config);

        // Generate a 1-second tone at 440 Hz
        let signal: Vec<f64> = (0..16000)
            .map(|i| (2.0 * PI * 440.0 * i as f64 / 16000.0).sin())
            .collect();

        let result = extractor.extract(&signal);
        assert!(!result.is_empty());

        // Each frame should have 10 coefficients, deltas, and delta-deltas
        for frame in &result {
            assert_eq!(frame.coefficients.len(), 10);
            assert!(frame.deltas.is_some());
            assert!(frame.delta_deltas.is_some());
            assert_eq!(frame.deltas.as_ref().unwrap().len(), 10);
            assert_eq!(frame.delta_deltas.as_ref().unwrap().len(), 10);
        }
    }

    #[test]
    fn test_extract_with_cmn() {
        let config = MfccConfig {
            sample_rate: 16000.0,
            fft_size: 256,
            num_mel_filters: 20,
            num_coefficients: 10,
            pre_emphasis: 0.97,
            hop_size: 80,
            frame_size: 160,
            low_freq: 0.0,
            high_freq: 8000.0,
            include_deltas: false,
            apply_cmn: true,
        };
        let extractor = MfccExtractor::new(config);

        let signal: Vec<f64> = (0..4000)
            .map(|i| (2.0 * PI * 440.0 * i as f64 / 16000.0).sin())
            .collect();

        let result = extractor.extract(&signal);
        assert!(!result.is_empty());

        // After CMN, the mean of each coefficient across frames should be ~0
        let num_coeffs = result[0].coefficients.len();
        for c in 0..num_coeffs {
            let mean: f64 = result.iter().map(|f| f.coefficients[c]).sum::<f64>()
                / result.len() as f64;
            assert!(
                mean.abs() < 1e-10,
                "CMN mean for coeff {} = {} (expected ~0)",
                c,
                mean
            );
        }
    }

    #[test]
    fn test_extract_frame_short_input_zero_padded() {
        // Providing fewer samples than frame_size should still work (zero-padded)
        let config = MfccConfig::default();
        let extractor = MfccExtractor::new(config);
        let short = vec![1.0; 100]; // less than frame_size=400
        let mfccs = extractor.extract_frame(&short);
        assert_eq!(mfccs.len(), 13);
        // Should produce finite values
        for &v in &mfccs {
            assert!(v.is_finite(), "MFCC value is not finite: {}", v);
        }
    }

    #[test]
    fn test_different_num_coefficients() {
        for &num_c in &[5, 10, 13, 20] {
            let config = MfccConfig {
                num_coefficients: num_c,
                num_mel_filters: 26_usize.max(num_c),
                ..MfccConfig::default()
            };
            let extractor = MfccExtractor::new(config);
            let frame = vec![0.5; 400];
            let mfccs = extractor.extract_frame(&frame);
            assert_eq!(mfccs.len(), num_c, "Expected {} coefficients", num_c);
        }
    }
}

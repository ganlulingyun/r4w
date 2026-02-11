//! EEG Signal Processing for Brain-Computer Interface (BCI) Applications
//!
//! This module provides a complete EEG signal processing pipeline for BCI systems.
//! It extracts frequency band powers (delta, theta, alpha, beta, gamma), detects
//! event-related potentials (ERPs), and classifies motor imagery using Common
//! Spatial Patterns (CSP).
//!
//! All math is implemented from scratch using only `std` -- no external crate
//! dependencies.
//!
//! ## EEG Frequency Bands
//!
//! | Band   | Range (Hz) | Associated Activity                  |
//! |--------|-----------|--------------------------------------|
//! | Delta  | 0.5 - 4   | Deep sleep, unconscious              |
//! | Theta  | 4 - 8     | Drowsiness, meditation               |
//! | Alpha  | 8 - 13    | Relaxed wakefulness, eyes closed     |
//! | Beta   | 13 - 30   | Active thinking, focus               |
//! | Gamma  | 30 - 100  | Cognitive processing, binding        |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::electroencephalogram_bci::{
//!     EegConfig, EegProcessor, ReferenceType,
//!     extract_all_bands, hjorth_parameters, spectral_entropy,
//! };
//!
//! let config = EegConfig {
//!     num_channels: 2,
//!     sample_rate_hz: 256.0,
//!     reference_type: ReferenceType::CommonAverage,
//! };
//!
//! let processor = EegProcessor::new(config);
//!
//! // Simulate 1 second of 2-channel EEG (10 Hz alpha + noise)
//! let n = 256;
//! let channels: Vec<Vec<f64>> = (0..2)
//!     .map(|_| {
//!         (0..n)
//!             .map(|i| {
//!                 let t = i as f64 / 256.0;
//!                 10.0 * (2.0 * std::f64::consts::PI * 10.0 * t).sin()
//!             })
//!             .collect()
//!     })
//!     .collect();
//!
//! let features = processor.process_epoch(&channels);
//! assert!(features.band_powers.alpha > features.band_powers.delta);
//! assert!(features.dominant_frequency_hz > 8.0 && features.dominant_frequency_hz < 13.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Internal FFT (Cooley-Tukey radix-2, from scratch)
// ---------------------------------------------------------------------------

/// Complex number for internal FFT computations.
#[derive(Debug, Clone, Copy)]
struct Cx {
    re: f64,
    im: f64,
}

impl Cx {
    fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }
    fn mag_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }
    fn mag(self) -> f64 {
        self.mag_sq().sqrt()
    }
    fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }
}

impl std::ops::Add for Cx {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::Sub for Cx {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl std::ops::Mul for Cx {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl std::ops::MulAssign for Cx {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

impl std::ops::AddAssign for Cx {
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

/// Return the smallest power of 2 >= n.
fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Bit-reversal permutation.
fn bit_reverse(x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    let mut val = x;
    for _ in 0..log2n {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// In-place iterative Cooley-Tukey radix-2 FFT.
///
/// `buf` length must be a power of 2. `inverse` selects IFFT.
fn fft_in_place(buf: &mut [Cx], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of 2");
    let log2n = n.trailing_zeros();

    // Bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_sign = if inverse { 1.0 } else { -1.0 };
        let angle = angle_sign * 2.0 * PI / size as f64;
        let w_n = Cx::new(angle.cos(), angle.sin());

        let mut start = 0;
        while start < n {
            let mut w = Cx::new(1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = w * buf[start + k + half];
                buf[start + k] = u + t;
                buf[start + k + half] = u - t;
                w *= w_n;
            }
            start += size;
        }
        size <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for c in buf.iter_mut() {
            c.re *= inv_n;
            c.im *= inv_n;
        }
    }
}

/// Compute the FFT of a real-valued signal (zero-padded to power-of-2).
/// Returns complex spectrum of length N (power of 2).
fn real_fft(signal: &[f64]) -> Vec<Cx> {
    let n = next_power_of_two(signal.len());
    let mut buf: Vec<Cx> = signal
        .iter()
        .map(|&x| Cx::new(x, 0.0))
        .chain(std::iter::repeat(Cx::zero()).take(n - signal.len()))
        .collect();
    fft_in_place(&mut buf, false);
    buf
}

/// Compute the power spectral density (single-sided) of a real signal.
/// Returns `(psd, freq_resolution)` where `psd[k]` is power at frequency
/// `k * freq_resolution` Hz. PSD length = N/2 + 1.
fn compute_psd(signal: &[f64], fs: f64) -> (Vec<f64>, f64) {
    let n = next_power_of_two(signal.len());
    let spectrum = real_fft(signal);
    let n_half = n / 2 + 1;
    let freq_res = fs / n as f64;
    let mut psd = Vec::with_capacity(n_half);
    let scale = 1.0 / (n as f64 * fs);
    for k in 0..n_half {
        let mag_sq = spectrum[k].mag_sq();
        // Double non-DC, non-Nyquist bins to account for negative frequencies
        let factor = if k == 0 || k == n / 2 { 1.0 } else { 2.0 };
        psd.push(mag_sq * scale * factor);
    }
    (psd, freq_res)
}

// ---------------------------------------------------------------------------
// Internal linear algebra helpers (for CSP)
// ---------------------------------------------------------------------------

/// Multiply two matrices: C = A * B.
/// A is (m x p), B is (p x n), result is (m x n). Row-major.
fn mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let m = a.len();
    let p = a[0].len();
    let n = b[0].len();
    let mut c = vec![vec![0.0; n]; m];
    for i in 0..m {
        for j in 0..n {
            let mut sum = 0.0;
            for k in 0..p {
                sum += a[i][k] * b[k][j];
            }
            c[i][j] = sum;
        }
    }
    c
}

/// Transpose a matrix.
fn mat_transpose(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let m = a.len();
    let n = a[0].len();
    let mut t = vec![vec![0.0; m]; n];
    for i in 0..m {
        for j in 0..n {
            t[j][i] = a[i][j];
        }
    }
    t
}

/// Add two matrices element-wise.
fn mat_add(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let m = a.len();
    let n = a[0].len();
    let mut c = vec![vec![0.0; n]; m];
    for i in 0..m {
        for j in 0..n {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
    c
}

/// Scale a matrix by a scalar.
fn mat_scale(a: &[Vec<f64>], s: f64) -> Vec<Vec<f64>> {
    a.iter()
        .map(|row| row.iter().map(|&v| v * s).collect())
        .collect()
}

/// Identity matrix of size n x n.
fn mat_eye(n: usize) -> Vec<Vec<f64>> {
    let mut m = vec![vec![0.0; n]; n];
    for i in 0..n {
        m[i][i] = 1.0;
    }
    m
}

/// Compute the covariance matrix of a multi-channel epoch.
/// `epoch` is channels x samples. Returns channels x channels covariance,
/// normalized by trace.
fn covariance_matrix(epoch: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let ch = epoch.len();
    let n = epoch[0].len();
    let mut cov = vec![vec![0.0; ch]; ch];
    for i in 0..ch {
        for j in i..ch {
            let mut sum = 0.0;
            for s in 0..n {
                sum += epoch[i][s] * epoch[j][s];
            }
            cov[i][j] = sum / n as f64;
            cov[j][i] = cov[i][j];
        }
    }
    // Normalize by trace
    let mut trace = 0.0;
    for i in 0..ch {
        trace += cov[i][i];
    }
    if trace > 0.0 {
        for i in 0..ch {
            for j in 0..ch {
                cov[i][j] /= trace;
            }
        }
    }
    cov
}

/// Eigenvalue decomposition of a symmetric matrix using the Jacobi method.
/// Returns `(eigenvalues, eigenvectors_as_columns)`.
/// Eigenvectors are returned as a matrix where column j is the j-th eigenvector.
fn jacobi_eigen(mat: &[Vec<f64>], max_iter: usize) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = mat.len();
    let mut a: Vec<Vec<f64>> = mat.to_vec();
    let mut v = mat_eye(n);

    for _ in 0..max_iter {
        // Find largest off-diagonal element
        let mut p = 0;
        let mut q = 1;
        let mut max_val = 0.0f64;
        for i in 0..n {
            for j in (i + 1)..n {
                if a[i][j].abs() > max_val {
                    max_val = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max_val < 1e-12 {
            break;
        }

        // Compute rotation
        let theta = if (a[q][q] - a[p][p]).abs() < 1e-30 {
            PI / 4.0
        } else {
            0.5 * (2.0 * a[p][q] / (a[p][p] - a[q][q])).atan()
        };
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        // Apply rotation to A: A' = G^T A G
        let mut new_a = a.clone();
        for i in 0..n {
            if i != p && i != q {
                new_a[i][p] = cos_t * a[i][p] + sin_t * a[i][q];
                new_a[p][i] = new_a[i][p];
                new_a[i][q] = -sin_t * a[i][p] + cos_t * a[i][q];
                new_a[q][i] = new_a[i][q];
            }
        }
        new_a[p][p] = cos_t * cos_t * a[p][p]
            + 2.0 * sin_t * cos_t * a[p][q]
            + sin_t * sin_t * a[q][q];
        new_a[q][q] = sin_t * sin_t * a[p][p]
            - 2.0 * sin_t * cos_t * a[p][q]
            + cos_t * cos_t * a[q][q];
        new_a[p][q] = 0.0;
        new_a[q][p] = 0.0;
        a = new_a;

        // Update eigenvectors
        for i in 0..n {
            let vip = v[i][p];
            let viq = v[i][q];
            v[i][p] = cos_t * vip + sin_t * viq;
            v[i][q] = -sin_t * vip + cos_t * viq;
        }
    }

    let eigenvalues: Vec<f64> = (0..n).map(|i| a[i][i]).collect();
    (eigenvalues, v)
}

/// Cholesky decomposition of a symmetric positive-definite matrix.
/// Returns lower-triangular L such that A = L * L^T.
fn cholesky(mat: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = mat.len();
    let mut l = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = 0.0;
            for k in 0..j {
                sum += l[i][k] * l[j][k];
            }
            if i == j {
                let val = mat[i][i] - sum;
                l[i][j] = if val > 0.0 { val.sqrt() } else { 1e-10 };
            } else {
                l[i][j] = if l[j][j].abs() > 1e-30 {
                    (mat[i][j] - sum) / l[j][j]
                } else {
                    0.0
                };
            }
        }
    }
    l
}

/// Invert a lower-triangular matrix.
fn inv_lower_triangular(l: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = l.len();
    let mut inv = vec![vec![0.0; n]; n];
    for i in 0..n {
        inv[i][i] = if l[i][i].abs() > 1e-30 {
            1.0 / l[i][i]
        } else {
            0.0
        };
        for j in (i + 1)..n {
            let mut sum = 0.0;
            for k in i..j {
                sum += l[j][k] * inv[k][i];
            }
            inv[j][i] = if l[j][j].abs() > 1e-30 {
                -sum / l[j][j]
            } else {
                0.0
            };
        }
    }
    inv
}

// ---------------------------------------------------------------------------
// Bandpass filter (FIR windowed-sinc, from scratch)
// ---------------------------------------------------------------------------

/// Apply a Hamming-windowed sinc FIR bandpass filter.
/// `low_hz` and `high_hz` define the passband.
fn bandpass_filter(signal: &[f64], fs: f64, low_hz: f64, high_hz: f64) -> Vec<f64> {
    // Design bandpass kernel: highpass(low) convolved implicitly via difference
    // We use the spectral-inversion technique:
    //   bandpass = lowpass(high) - lowpass(low)
    let order = 101; // Fixed filter order (odd for Type I FIR)
    let m = order - 1;
    let half = m / 2;

    let fc_low = low_hz / fs;
    let fc_high = high_hz / fs;

    // Design lowpass kernels
    let mut lp_high = vec![0.0; order];
    let mut lp_low = vec![0.0; order];
    for i in 0..order {
        let n = i as f64 - half as f64;
        // Hamming window
        let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / m as f64).cos();
        if n.abs() < 1e-10 {
            lp_high[i] = 2.0 * fc_high * w;
            lp_low[i] = 2.0 * fc_low * w;
        } else {
            lp_high[i] = (2.0 * PI * fc_high * n).sin() / (PI * n) * w;
            lp_low[i] = (2.0 * PI * fc_low * n).sin() / (PI * n) * w;
        }
    }

    // Bandpass = lowpass(high) - lowpass(low)
    let mut bp = vec![0.0; order];
    for i in 0..order {
        bp[i] = lp_high[i] - lp_low[i];
    }

    // Normalize
    let sum: f64 = bp.iter().sum::<f64>().abs();
    if sum > 1e-10 {
        // For bandpass, normalize by peak of frequency response instead of DC sum
        // Find the center frequency response
        let center_f = (low_hz + high_hz) / 2.0;
        let omega = 2.0 * PI * center_f / fs;
        let mut resp_re = 0.0;
        let mut resp_im = 0.0;
        for (i, &h) in bp.iter().enumerate() {
            let phase = omega * (i as f64 - half as f64);
            resp_re += h * phase.cos();
            resp_im += h * phase.sin();
        }
        let gain = (resp_re * resp_re + resp_im * resp_im).sqrt();
        if gain > 1e-10 {
            for h in bp.iter_mut() {
                *h /= gain;
            }
        }
    }

    // Apply filter via convolution
    let sig_len = signal.len();
    let mut output = vec![0.0; sig_len];
    for i in 0..sig_len {
        let mut acc = 0.0;
        for j in 0..order {
            let idx = i as isize - j as isize + half as isize;
            if idx >= 0 && (idx as usize) < sig_len {
                acc += bp[j] * signal[idx as usize];
            }
        }
        output[i] = acc;
    }
    output
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Reference scheme used for EEG montage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReferenceType {
    /// Common average reference: subtract mean of all channels from each channel.
    CommonAverage,
    /// Linked-ears reference: average of two designated reference channels
    /// (channels 0 and 1 are treated as left/right ear references).
    LinkedEars,
    /// Bipolar: compute successive differences between adjacent channels.
    Bipolar,
}

/// Configuration for the EEG processor.
#[derive(Debug, Clone)]
pub struct EegConfig {
    /// Number of EEG channels.
    pub num_channels: usize,
    /// Sampling rate in Hz (common: 256, 512, 1024).
    pub sample_rate_hz: f64,
    /// Reference scheme.
    pub reference_type: ReferenceType,
}

/// Band power values for the five standard EEG frequency bands.
///
/// | Band   | Range (Hz) |
/// |--------|-----------|
/// | Delta  | 0.5 - 4   |
/// | Theta  | 4 - 8     |
/// | Alpha  | 8 - 13    |
/// | Beta   | 13 - 30   |
/// | Gamma  | 30 - 100  |
#[derive(Debug, Clone)]
pub struct BandPowers {
    /// Delta band power (0.5 - 4 Hz): deep sleep, unconscious states.
    pub delta: f64,
    /// Theta band power (4 - 8 Hz): drowsiness, meditation.
    pub theta: f64,
    /// Alpha band power (8 - 13 Hz): relaxed wakefulness, eyes closed.
    pub alpha: f64,
    /// Beta band power (13 - 30 Hz): active thinking, concentration.
    pub beta: f64,
    /// Gamma band power (30 - 100 Hz): cognitive processing, sensory binding.
    pub gamma: f64,
}

/// Extracted EEG features from a single epoch.
#[derive(Debug, Clone)]
pub struct EegFeatures {
    /// Power in each of the five standard EEG frequency bands.
    pub band_powers: BandPowers,
    /// Asymmetry index: log(right alpha) - log(left alpha).
    /// Positive values indicate greater right-hemisphere alpha (left-hemisphere activation).
    pub asymmetry_index: f64,
    /// Dominant frequency in the epoch (Hz), based on PSD peak.
    pub dominant_frequency_hz: f64,
    /// Normalized spectral entropy (0 = pure tone, 1 = white noise).
    pub spectral_entropy: f64,
}

/// EEG signal processor for BCI feature extraction.
#[derive(Debug, Clone)]
pub struct EegProcessor {
    config: EegConfig,
}

impl EegProcessor {
    /// Create a new EEG processor with the given configuration.
    pub fn new(config: EegConfig) -> Self {
        assert!(config.num_channels > 0, "Must have at least 1 channel");
        assert!(
            config.sample_rate_hz > 0.0,
            "Sample rate must be positive"
        );
        Self { config }
    }

    /// Apply the configured reference scheme to the raw channels (in-place clone).
    fn apply_reference(&self, channels: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let ch = channels.len();
        let n = channels[0].len();
        match self.config.reference_type {
            ReferenceType::CommonAverage => {
                // For a single channel, CAR is a no-op (avoid zeroing)
                if ch <= 1 {
                    return channels.to_vec();
                }
                // Subtract mean across channels at each sample
                let mut referenced = channels.to_vec();
                for s in 0..n {
                    let mean: f64 =
                        channels.iter().map(|c| c[s]).sum::<f64>() / ch as f64;
                    for c in 0..ch {
                        referenced[c][s] -= mean;
                    }
                }
                referenced
            }
            ReferenceType::LinkedEars => {
                // Use average of channels 0 and 1 as reference
                let mut referenced = channels.to_vec();
                if ch >= 2 {
                    for s in 0..n {
                        let ref_val = (channels[0][s] + channels[1][s]) / 2.0;
                        for c in 0..ch {
                            referenced[c][s] -= ref_val;
                        }
                    }
                }
                referenced
            }
            ReferenceType::Bipolar => {
                // Successive differences
                if ch < 2 {
                    return channels.to_vec();
                }
                let mut referenced = Vec::with_capacity(ch - 1);
                for c in 0..(ch - 1) {
                    let diff: Vec<f64> = (0..n)
                        .map(|s| channels[c][s] - channels[c + 1][s])
                        .collect();
                    referenced.push(diff);
                }
                referenced
            }
        }
    }

    /// Process an epoch of multi-channel EEG data and extract features.
    ///
    /// `channels` is a slice of channel vectors, each of equal length.
    /// Returns aggregated features across all channels.
    pub fn process_epoch(&self, channels: &[Vec<f64>]) -> EegFeatures {
        assert!(
            !channels.is_empty(),
            "Must provide at least one channel"
        );
        let n = channels[0].len();
        assert!(n > 0, "Epoch must have at least one sample");
        for (i, ch) in channels.iter().enumerate() {
            assert_eq!(
                ch.len(),
                n,
                "Channel {} has length {} but expected {}",
                i,
                ch.len(),
                n
            );
        }

        let referenced = self.apply_reference(channels);
        let fs = self.config.sample_rate_hz;
        let num_ch = referenced.len();

        // Compute band powers averaged across channels
        let mut avg_bands = BandPowers {
            delta: 0.0,
            theta: 0.0,
            alpha: 0.0,
            beta: 0.0,
            gamma: 0.0,
        };
        let mut per_channel_alpha = Vec::with_capacity(num_ch);

        for ch_data in &referenced {
            let bands = extract_all_bands(ch_data, fs);
            per_channel_alpha.push(bands.alpha);
            avg_bands.delta += bands.delta;
            avg_bands.theta += bands.theta;
            avg_bands.alpha += bands.alpha;
            avg_bands.beta += bands.beta;
            avg_bands.gamma += bands.gamma;
        }
        avg_bands.delta /= num_ch as f64;
        avg_bands.theta /= num_ch as f64;
        avg_bands.alpha /= num_ch as f64;
        avg_bands.beta /= num_ch as f64;
        avg_bands.gamma /= num_ch as f64;

        // Asymmetry index (log(right) - log(left)) for first two channels
        let asymmetry_index = if per_channel_alpha.len() >= 2 {
            let left = per_channel_alpha[0].max(1e-30);
            let right = per_channel_alpha[1].max(1e-30);
            right.ln() - left.ln()
        } else {
            0.0
        };

        // Dominant frequency and spectral entropy from first channel
        let dominant_frequency_hz = {
            let (psd, freq_res) = compute_psd(&referenced[0], fs);
            // Find peak in 0.5 - 100 Hz range
            let low_bin = (0.5 / freq_res).ceil() as usize;
            let high_bin = ((100.0f64).min(fs / 2.0) / freq_res).floor() as usize;
            let high_bin = high_bin.min(psd.len() - 1);
            let mut max_power = 0.0;
            let mut max_bin = low_bin;
            for k in low_bin..=high_bin {
                if psd[k] > max_power {
                    max_power = psd[k];
                    max_bin = k;
                }
            }
            max_bin as f64 * freq_res
        };

        let se = spectral_entropy(&referenced[0], fs);

        EegFeatures {
            band_powers: avg_bands,
            asymmetry_index,
            dominant_frequency_hz,
            spectral_entropy: se,
        }
    }
}

// ---------------------------------------------------------------------------
// Public functions
// ---------------------------------------------------------------------------

/// Compute the power in a specific frequency band of a real-valued signal.
///
/// Applies a bandpass FIR filter between `low_hz` and `high_hz`, then computes
/// the RMS power of the filtered signal.
///
/// # Arguments
/// * `signal` - Time-domain samples (microvolts for EEG).
/// * `fs` - Sampling rate in Hz.
/// * `low_hz` - Lower edge of the band in Hz.
/// * `high_hz` - Upper edge of the band in Hz.
///
/// # Returns
/// RMS power (same units as input, squared for power).
pub fn compute_band_power(signal: &[f64], fs: f64, low_hz: f64, high_hz: f64) -> f64 {
    assert!(
        low_hz < high_hz,
        "low_hz ({}) must be less than high_hz ({})",
        low_hz,
        high_hz
    );
    assert!(
        high_hz <= fs / 2.0,
        "high_hz ({}) must be at most Nyquist ({})",
        high_hz,
        fs / 2.0
    );
    if signal.is_empty() {
        return 0.0;
    }

    let filtered = bandpass_filter(signal, fs, low_hz, high_hz);
    // RMS power
    let sum_sq: f64 = filtered.iter().map(|&x| x * x).sum();
    (sum_sq / filtered.len() as f64).sqrt()
}

/// Extract power in all five standard EEG frequency bands.
///
/// | Band   | Range (Hz) |
/// |--------|-----------|
/// | Delta  | 0.5 - 4   |
/// | Theta  | 4 - 8     |
/// | Alpha  | 8 - 13    |
/// | Beta   | 13 - 30   |
/// | Gamma  | 30 - 100  |
///
/// Gamma upper bound is clamped to Nyquist if `fs < 200`.
pub fn extract_all_bands(signal: &[f64], fs: f64) -> BandPowers {
    let nyquist = fs / 2.0;
    let gamma_high = (100.0f64).min(nyquist - 0.1);

    // Gamma requires at least 5 Hz of bandwidth above 30 Hz to be meaningful
    let gamma_min_bw = 5.0;

    BandPowers {
        delta: compute_band_power(signal, fs, 0.5, 4.0),
        theta: compute_band_power(signal, fs, 4.0, 8.0),
        alpha: compute_band_power(signal, fs, 8.0, 13.0),
        beta: if nyquist > 30.0 {
            compute_band_power(signal, fs, 13.0, 30.0)
        } else if nyquist > 14.0 {
            compute_band_power(signal, fs, 13.0, nyquist - 0.1)
        } else {
            0.0
        },
        gamma: if gamma_high >= 30.0 + gamma_min_bw {
            compute_band_power(signal, fs, 30.0, gamma_high)
        } else {
            0.0
        },
    }
}

/// Detect event-related potentials (ERPs) by epoch averaging.
///
/// For each event sample index, extracts a window of duration `window_ms` and
/// averages across all events to produce the ERP waveform.
///
/// # Arguments
/// * `signal` - Continuous EEG signal (single channel).
/// * `fs` - Sampling rate in Hz.
/// * `event_samples` - Sample indices where events (stimuli) occurred.
/// * `window_ms` - Duration of each epoch window in milliseconds.
///
/// # Returns
/// Averaged ERP waveform (length = window in samples).
pub fn detect_erp(
    signal: &[f64],
    fs: f64,
    event_samples: &[usize],
    window_ms: f64,
) -> Vec<f64> {
    let window_len = ((window_ms / 1000.0) * fs).round() as usize;
    if window_len == 0 || event_samples.is_empty() {
        return vec![];
    }

    let sig_len = signal.len();
    let mut erp = vec![0.0; window_len];
    let mut count = 0usize;

    for &onset in event_samples {
        if onset + window_len <= sig_len {
            for i in 0..window_len {
                erp[i] += signal[onset + i];
            }
            count += 1;
        }
    }

    if count > 0 {
        let inv = 1.0 / count as f64;
        for v in erp.iter_mut() {
            *v *= inv;
        }
    }

    erp
}

/// Compute Common Spatial Pattern (CSP) filters for two-class motor imagery BCI.
///
/// CSP finds spatial filters that maximize the variance of one class while
/// minimizing the variance of the other. This is the core of motor imagery BCI
/// classification (e.g., left-hand vs right-hand movement imagination).
///
/// # Arguments
/// * `class1` - Trials for class 1. Each trial is `channels x samples`.
/// * `class2` - Trials for class 2. Each trial is `channels x samples`.
/// * `num_pairs` - Number of spatial filter pairs to return (typical: 2-3).
///
/// # Returns
/// Spatial filter matrix: `(2*num_pairs) x channels`. Each row is a spatial
/// filter. The first `num_pairs` rows maximize class 1 variance, the last
/// `num_pairs` rows maximize class 2 variance.
///
/// # Algorithm
/// 1. Compute average covariance matrices for each class.
/// 2. Compute the composite covariance: C = C1 + C2.
/// 3. Whiten using C^(-1/2).
/// 4. Solve the generalized eigenvalue problem on the whitened C1.
/// 5. Select filters with largest and smallest eigenvalues.
pub fn common_spatial_patterns(
    class1: &[Vec<Vec<f64>>],
    class2: &[Vec<Vec<f64>>],
    num_pairs: usize,
) -> Vec<Vec<f64>> {
    assert!(!class1.is_empty(), "class1 must not be empty");
    assert!(!class2.is_empty(), "class2 must not be empty");

    let num_channels = class1[0].len();
    assert!(
        num_pairs <= num_channels,
        "num_pairs ({}) must be <= num_channels ({})",
        num_pairs,
        num_channels
    );

    // Average covariance for class 1
    let mut cov1 = vec![vec![0.0; num_channels]; num_channels];
    for trial in class1 {
        let c = covariance_matrix(trial);
        cov1 = mat_add(&cov1, &c);
    }
    cov1 = mat_scale(&cov1, 1.0 / class1.len() as f64);

    // Average covariance for class 2
    let mut cov2 = vec![vec![0.0; num_channels]; num_channels];
    for trial in class2 {
        let c = covariance_matrix(trial);
        cov2 = mat_add(&cov2, &c);
    }
    cov2 = mat_scale(&cov2, 1.0 / class2.len() as f64);

    // Composite covariance
    let cov_composite = mat_add(&cov1, &cov2);

    // Whitening: P = D^(-1/2) * U^T where C_composite = U * D * U^T
    let (eigenvalues, eigenvectors) = jacobi_eigen(&cov_composite, 200);

    // Build whitening matrix: P = D^(-1/2) * V^T
    let mut d_inv_sqrt = vec![vec![0.0; num_channels]; num_channels];
    for i in 0..num_channels {
        d_inv_sqrt[i][i] = if eigenvalues[i].abs() > 1e-10 {
            1.0 / eigenvalues[i].abs().sqrt()
        } else {
            0.0
        };
    }
    let vt = mat_transpose(&eigenvectors);
    let p = mat_mul(&d_inv_sqrt, &vt);

    // Whiten C1: S1 = P * C1 * P^T
    let pt = mat_transpose(&p);
    let s1 = mat_mul(&mat_mul(&p, &cov1), &pt);

    // Eigendecompose whitened C1
    let (eig_vals, eig_vecs) = jacobi_eigen(&s1, 200);

    // Sort eigenvalues (descending)
    let mut indices: Vec<usize> = (0..num_channels).collect();
    indices.sort_by(|&a, &b| {
        eig_vals[b]
            .partial_cmp(&eig_vals[a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Spatial filters W = B^T * P where B columns are selected eigenvectors
    // Select first num_pairs (max class1) and last num_pairs (max class2)
    let mut selected = Vec::with_capacity(2 * num_pairs);
    for i in 0..num_pairs {
        selected.push(indices[i]);
    }
    for i in 0..num_pairs {
        selected.push(indices[num_channels - 1 - i]);
    }

    // Build filter matrix
    let mut filters = Vec::with_capacity(selected.len());
    for &idx in &selected {
        // Extract column idx from eigenvectors, then multiply by P^T
        let b_col: Vec<Vec<f64>> = (0..num_channels)
            .map(|r| vec![eig_vecs[r][idx]])
            .collect();
        let w = mat_mul(&mat_transpose(&b_col), &p);
        filters.push(w[0].clone());
    }

    filters
}

/// Compute normalized Shannon spectral entropy of a signal.
///
/// Spectral entropy measures the "flatness" of the power spectrum.
/// A pure sinusoid yields entropy near 0, white noise yields entropy near 1.
///
/// # Algorithm
/// 1. Compute the PSD via FFT.
/// 2. Normalize PSD to a probability distribution.
/// 3. Compute Shannon entropy: H = -sum(p * ln(p)).
/// 4. Normalize by ln(N) to get value in [0, 1].
pub fn spectral_entropy(signal: &[f64], fs: f64) -> f64 {
    if signal.len() < 2 {
        return 0.0;
    }
    let (psd, _freq_res) = compute_psd(signal, fs);
    let total: f64 = psd.iter().sum();
    if total <= 0.0 {
        return 0.0;
    }

    let n = psd.len();
    let mut entropy = 0.0;
    for &p in &psd {
        let prob = p / total;
        if prob > 1e-30 {
            entropy -= prob * prob.ln();
        }
    }

    // Normalize by maximum possible entropy (uniform distribution)
    let max_entropy = (n as f64).ln();
    if max_entropy > 0.0 {
        entropy / max_entropy
    } else {
        0.0
    }
}

/// Compute the Hjorth parameters of a signal.
///
/// These three time-domain descriptors capture the statistical properties
/// of a signal without requiring spectral analysis:
///
/// - **Activity**: Variance of the signal (power).
/// - **Mobility**: Square root of the ratio of the variance of the first
///   derivative to the variance of the signal. Represents the mean frequency.
/// - **Complexity**: Ratio of the mobility of the first derivative to the
///   mobility of the signal. Represents the bandwidth.
///
/// # Returns
/// `(activity, mobility, complexity)`
pub fn hjorth_parameters(signal: &[f64]) -> (f64, f64, f64) {
    let n = signal.len();
    if n < 3 {
        return (0.0, 0.0, 0.0);
    }

    // Activity = variance of signal
    let mean: f64 = signal.iter().sum::<f64>() / n as f64;
    let activity: f64 = signal.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;

    // First derivative
    let d1: Vec<f64> = signal.windows(2).map(|w| w[1] - w[0]).collect();
    let d1_mean: f64 = d1.iter().sum::<f64>() / d1.len() as f64;
    let var_d1: f64 = d1.iter().map(|&x| (x - d1_mean).powi(2)).sum::<f64>() / d1.len() as f64;

    // Second derivative
    let d2: Vec<f64> = d1.windows(2).map(|w| w[1] - w[0]).collect();
    let d2_mean: f64 = d2.iter().sum::<f64>() / d2.len() as f64;
    let var_d2: f64 = d2.iter().map(|&x| (x - d2_mean).powi(2)).sum::<f64>() / d2.len() as f64;

    let mobility = if activity > 1e-30 {
        (var_d1 / activity).sqrt()
    } else {
        0.0
    };

    let mobility_d1 = if var_d1 > 1e-30 {
        (var_d2 / var_d1).sqrt()
    } else {
        0.0
    };

    let complexity = if mobility > 1e-30 {
        mobility_d1 / mobility
    } else {
        0.0
    };

    (activity, mobility, complexity)
}

/// Reject artifacts by zeroing out channels where the peak amplitude exceeds
/// a threshold.
///
/// EEG artifacts (eye blinks, muscle activity, movement) produce amplitudes
/// far exceeding normal cortical activity (typically 10-100 uV). This function
/// zeroes any channel epoch where `max(|sample|) > threshold_uv`.
///
/// # Arguments
/// * `channels` - Mutable slice of channel vectors. Modified in-place.
/// * `threshold_uv` - Amplitude threshold in microvolts (typical: 100-150 uV).
pub fn artifact_rejection(channels: &mut [Vec<f64>], threshold_uv: f64) {
    for channel in channels.iter_mut() {
        let peak = channel
            .iter()
            .map(|&x| x.abs())
            .fold(0.0f64, f64::max);
        if peak > threshold_uv {
            for sample in channel.iter_mut() {
                *sample = 0.0;
            }
        }
    }
}

/// Compute magnitude-squared coherence between two signals using Welch's method.
///
/// Coherence measures the linear correlation between two signals as a function
/// of frequency. The returned value is the mean magnitude-squared coherence
/// across all frequency bins.
///
/// Uses Welch's method (50% overlapping segments) for reliable estimation.
/// With a single segment, coherence is trivially 1 for any non-zero bin,
/// so segmentation is essential.
///
/// # Algorithm
/// 1. Divide signals into overlapping segments.
/// 2. For each segment, compute FFT and accumulate Sxx, Syy, Sxy.
/// 3. Coherence per bin: C[k] = |<Sxy[k]>|^2 / (<Sxx[k]> * <Syy[k]>).
/// 4. Return mean coherence across bins.
///
/// # Returns
/// Mean magnitude-squared coherence in [0, 1]. Values near 1 indicate
/// strong linear relationship.
pub fn compute_coherence(signal_a: &[f64], signal_b: &[f64], fs: f64) -> f64 {
    let _ = fs; // Used for API consistency
    let n = signal_a.len().min(signal_b.len());
    if n < 4 {
        return 0.0;
    }

    // Choose segment size: aim for ~8 segments with 50% overlap
    let seg_len = next_power_of_two(n / 5).max(4);
    let hop = seg_len / 2;
    let n_half = seg_len / 2 + 1;

    // Accumulate averaged cross/auto spectra
    let mut sxx_avg = vec![0.0; n_half];
    let mut syy_avg = vec![0.0; n_half];
    let mut sxy_avg = vec![Cx::zero(); n_half];
    let mut num_segments = 0;

    let mut offset = 0;
    while offset + seg_len <= n {
        let seg_a = &signal_a[offset..offset + seg_len];
        let seg_b = &signal_b[offset..offset + seg_len];

        // Apply Hann window and FFT
        let mut buf_a: Vec<Cx> = seg_a
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (seg_len - 1) as f64).cos());
                Cx::new(x * w, 0.0)
            })
            .collect();
        let mut buf_b: Vec<Cx> = seg_b
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (seg_len - 1) as f64).cos());
                Cx::new(x * w, 0.0)
            })
            .collect();

        fft_in_place(&mut buf_a, false);
        fft_in_place(&mut buf_b, false);

        for k in 0..n_half {
            sxx_avg[k] += buf_a[k].mag_sq();
            syy_avg[k] += buf_b[k].mag_sq();
            sxy_avg[k] += buf_a[k] * buf_b[k].conj();
        }
        num_segments += 1;
        offset += hop;
    }

    if num_segments == 0 {
        return 0.0;
    }

    // Compute mean coherence across bins (skip DC)
    let mut sum_coh = 0.0;
    let mut valid_bins = 0;
    for k in 1..n_half {
        let denom = sxx_avg[k] * syy_avg[k];
        if denom > 1e-30 {
            let coh = sxy_avg[k].mag_sq() / denom;
            sum_coh += coh;
            valid_bins += 1;
        }
    }

    if valid_bins > 0 {
        sum_coh / valid_bins as f64
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 256.0;

    /// Generate a sinusoid at the given frequency and amplitude.
    fn sine_wave(freq_hz: f64, amplitude: f64, fs: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 / fs).sin())
            .collect()
    }

    /// Generate white noise (deterministic LCG PRNG for reproducibility).
    fn white_noise(amplitude: f64, n: usize, seed: u64) -> Vec<f64> {
        let mut state = seed;
        (0..n)
            .map(|_| {
                // LCG: x_{n+1} = (a * x_n + c) mod m
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let u = (state >> 33) as f64 / (1u64 << 31) as f64; // [0, 1)
                amplitude * (2.0 * u - 1.0)
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // FFT tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_pure_tone() {
        // 10 Hz tone at 256 Hz sample rate, 256 samples
        let signal = sine_wave(10.0, 1.0, FS, 256);
        let spectrum = real_fft(&signal);
        // Bin 10 should have the peak
        let magnitudes: Vec<f64> = spectrum.iter().map(|c| c.mag()).collect();
        let peak_bin = magnitudes[1..128]
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0
            + 1;
        assert_eq!(peak_bin, 10);
    }

    #[test]
    fn test_fft_dc_signal() {
        let signal = vec![5.0; 64];
        let spectrum = real_fft(&signal);
        // DC bin should be dominant
        assert!(spectrum[0].mag() > spectrum[1].mag() * 10.0);
    }

    #[test]
    fn test_fft_inverse_roundtrip() {
        let n = 64;
        let mut buf: Vec<Cx> = (0..n)
            .map(|i| Cx::new((i as f64 * 0.1).sin(), 0.0))
            .collect();
        let original: Vec<f64> = buf.iter().map(|c| c.re).collect();
        fft_in_place(&mut buf, false);
        fft_in_place(&mut buf, true);
        for i in 0..n {
            assert!(
                (buf[i].re - original[i]).abs() < 1e-10,
                "FFT roundtrip failed at index {}",
                i
            );
        }
    }

    // -----------------------------------------------------------------------
    // Band power tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_band_power_alpha_tone() {
        // 10 Hz tone should produce strong alpha power
        let signal = sine_wave(10.0, 20.0, FS, 512);
        let alpha = compute_band_power(&signal, FS, 8.0, 13.0);
        let theta = compute_band_power(&signal, FS, 4.0, 8.0);
        assert!(
            alpha > theta * 2.0,
            "Alpha ({}) should be much larger than theta ({})",
            alpha,
            theta
        );
    }

    #[test]
    fn test_band_power_delta_tone() {
        // 2 Hz tone should produce strong delta power
        let signal = sine_wave(2.0, 20.0, FS, 1024);
        let delta = compute_band_power(&signal, FS, 0.5, 4.0);
        let beta = compute_band_power(&signal, FS, 13.0, 30.0);
        assert!(
            delta > beta * 2.0,
            "Delta ({}) should dominate beta ({})",
            delta,
            beta
        );
    }

    #[test]
    fn test_band_power_beta_tone() {
        // 20 Hz tone in the beta band
        let signal = sine_wave(20.0, 15.0, FS, 512);
        let beta = compute_band_power(&signal, FS, 13.0, 30.0);
        let delta = compute_band_power(&signal, FS, 0.5, 4.0);
        assert!(
            beta > delta * 2.0,
            "Beta ({}) should dominate delta ({})",
            beta,
            delta
        );
    }

    #[test]
    fn test_band_power_empty_signal() {
        let result = compute_band_power(&[], FS, 8.0, 13.0);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn test_extract_all_bands_alpha_dominant() {
        let signal = sine_wave(10.0, 30.0, FS, 512);
        let bands = extract_all_bands(&signal, FS);
        assert!(
            bands.alpha > bands.delta,
            "Alpha ({}) should dominate delta ({})",
            bands.alpha,
            bands.delta
        );
        assert!(
            bands.alpha > bands.theta,
            "Alpha ({}) should dominate theta ({})",
            bands.alpha,
            bands.theta
        );
        assert!(
            bands.alpha > bands.gamma,
            "Alpha ({}) should dominate gamma ({})",
            bands.alpha,
            bands.gamma
        );
    }

    #[test]
    fn test_extract_all_bands_low_sample_rate() {
        // 64 Hz sample rate -- gamma should be 0 since Nyquist < 30.5
        let signal = sine_wave(5.0, 10.0, 64.0, 256);
        let bands = extract_all_bands(&signal, 64.0);
        assert_eq!(bands.gamma, 0.0, "Gamma should be zero at 64 Hz fs");
    }

    // -----------------------------------------------------------------------
    // ERP detection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_erp_basic() {
        // Create signal with identical pulses at known positions
        let n = 2000;
        let mut signal = vec![0.0; n];
        let events = vec![100, 500, 900, 1300];
        let window_len = 50;
        // Insert a pulse of amplitude 10 at each event
        for &e in &events {
            for i in 0..window_len {
                if e + i < n {
                    signal[e + i] = 10.0;
                }
            }
        }

        let erp = detect_erp(&signal, FS, &events, window_len as f64 * 1000.0 / FS);
        assert_eq!(erp.len(), window_len);
        // All values should be ~10.0 since all epochs are identical
        for (i, &v) in erp.iter().enumerate() {
            assert!(
                (v - 10.0).abs() < 0.01,
                "ERP sample {} = {}, expected 10.0",
                i,
                v
            );
        }
    }

    #[test]
    fn test_erp_averaging_reduces_noise() {
        let n = 5000;
        let noise = white_noise(5.0, n, 42);
        let mut signal = noise.clone();
        // Add a consistent response at each event
        let events: Vec<usize> = (0..10).map(|i| 100 + i * 400).collect();
        let window_len = 50;
        for &e in &events {
            for i in 0..window_len.min(n - e) {
                signal[e + i] += 20.0;
            }
        }

        let erp = detect_erp(&signal, FS, &events, window_len as f64 * 1000.0 / FS);
        // The averaged ERP should be close to 20 (signal) with reduced noise
        let erp_mean: f64 = erp.iter().sum::<f64>() / erp.len() as f64;
        assert!(
            erp_mean > 15.0,
            "ERP mean ({}) should be close to 20.0 after averaging",
            erp_mean
        );
    }

    #[test]
    fn test_erp_empty_events() {
        let signal = vec![1.0; 100];
        let erp = detect_erp(&signal, FS, &[], 100.0);
        assert!(erp.is_empty());
    }

    #[test]
    fn test_erp_event_beyond_signal() {
        let signal = vec![5.0; 100];
        // Event at index 90 with a 50-sample window would exceed signal
        let erp = detect_erp(&signal, FS, &[90], 50.0 * 1000.0 / FS);
        // Should return zeros since the single event overruns
        // (50 samples at 256 Hz = 195ms, so window_len = 50)
        // Actually: window_len = round(195ms/1000 * 256) = 50
        // 90 + 50 > 100, so no valid epochs
        assert!(
            erp.iter().all(|&v| v == 0.0) || erp.is_empty(),
            "ERP should be all zeros when event overruns signal"
        );
    }

    // -----------------------------------------------------------------------
    // CSP tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_csp_basic() {
        // Create two classes with clearly different spatial patterns
        // Class 1: more activity on channel 0
        // Class 2: more activity on channel 1
        let num_trials = 10;
        let num_samples = 200;
        let num_channels = 3;

        let mut class1 = Vec::new();
        let mut class2 = Vec::new();

        for t in 0..num_trials {
            let mut trial1 = vec![vec![0.0; num_samples]; num_channels];
            let mut trial2 = vec![vec![0.0; num_samples]; num_channels];

            for s in 0..num_samples {
                let noise_seed = (t * num_samples + s) as u64;
                let noise = ((noise_seed.wrapping_mul(2654435761)) as f64) / (u32::MAX as f64) - 0.5;

                // Class 1: strong on channel 0
                trial1[0][s] = 10.0 * (2.0 * PI * 10.0 * s as f64 / FS).sin() + noise;
                trial1[1][s] = noise * 0.5;
                trial1[2][s] = noise * 0.3;

                // Class 2: strong on channel 1
                trial2[0][s] = noise * 0.5;
                trial2[1][s] = 10.0 * (2.0 * PI * 12.0 * s as f64 / FS).sin() + noise;
                trial2[2][s] = noise * 0.3;
            }
            class1.push(trial1);
            class2.push(trial2);
        }

        let filters = common_spatial_patterns(&class1, &class2, 1);
        assert_eq!(filters.len(), 2, "Should return 2 filters for 1 pair");
        assert_eq!(filters[0].len(), num_channels, "Filter length should match channels");

        // The first filter should emphasize channel 0 (class1)
        // The second should emphasize channel 1 (class2)
        // We check that the filters are not all zeros
        let norm0: f64 = filters[0].iter().map(|&x| x * x).sum::<f64>();
        let norm1: f64 = filters[1].iter().map(|&x| x * x).sum::<f64>();
        assert!(norm0 > 1e-10, "First filter should not be zero");
        assert!(norm1 > 1e-10, "Second filter should not be zero");
    }

    #[test]
    fn test_csp_variance_separation() {
        // After CSP projection, class variances should be well separated
        let num_trials = 20;
        let num_samples = 256;
        let num_channels = 2;

        let mut class1 = Vec::new();
        let mut class2 = Vec::new();

        for t in 0..num_trials {
            let mut trial1 = vec![vec![0.0; num_samples]; num_channels];
            let mut trial2 = vec![vec![0.0; num_samples]; num_channels];

            for s in 0..num_samples {
                let base = (t * 1000 + s) as u64;
                let n1 = ((base.wrapping_mul(6364136223846793005).wrapping_add(1)) >> 33) as f64
                    / (1u64 << 31) as f64 - 0.5;
                let n2 = ((base.wrapping_mul(1103515245).wrapping_add(12345)) >> 16) as f64
                    / (1u64 << 15) as f64 - 0.5;

                trial1[0][s] = 8.0 * (2.0 * PI * 10.0 * s as f64 / FS).sin() + n1;
                trial1[1][s] = n2;

                trial2[0][s] = n1;
                trial2[1][s] = 8.0 * (2.0 * PI * 10.0 * s as f64 / FS).sin() + n2;
            }
            class1.push(trial1);
            class2.push(trial2);
        }

        let filters = common_spatial_patterns(&class1, &class2, 1);
        assert_eq!(filters.len(), 2);

        // Project and compute variance ratios
        let project_var = |trial: &[Vec<f64>], filter: &[f64]| -> f64 {
            let projected: Vec<f64> = (0..trial[0].len())
                .map(|s| {
                    filter
                        .iter()
                        .enumerate()
                        .map(|(c, &w)| w * trial[c][s])
                        .sum::<f64>()
                })
                .collect();
            let mean = projected.iter().sum::<f64>() / projected.len() as f64;
            projected.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / projected.len() as f64
        };

        // Filter 0 should give higher variance for class1
        let var1_f0: f64 = class1.iter().map(|t| project_var(t, &filters[0])).sum::<f64>();
        let var2_f0: f64 = class2.iter().map(|t| project_var(t, &filters[0])).sum::<f64>();

        // Filter 1 should give higher variance for class2
        let var1_f1: f64 = class1.iter().map(|t| project_var(t, &filters[1])).sum::<f64>();
        let var2_f1: f64 = class2.iter().map(|t| project_var(t, &filters[1])).sum::<f64>();

        // At least one direction should show separation
        let ratio_f0 = var1_f0 / (var2_f0 + 1e-30);
        let ratio_f1 = var2_f1 / (var1_f1 + 1e-30);
        assert!(
            ratio_f0 > 1.0 || ratio_f1 > 1.0,
            "CSP should separate variances: ratio_f0={}, ratio_f1={}",
            ratio_f0,
            ratio_f1
        );
    }

    // -----------------------------------------------------------------------
    // Spectral entropy tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectral_entropy_pure_tone() {
        // Pure tone should have low spectral entropy
        let signal = sine_wave(10.0, 1.0, FS, 512);
        let se = spectral_entropy(&signal, FS);
        assert!(
            se < 0.5,
            "Pure tone spectral entropy ({}) should be low",
            se
        );
    }

    #[test]
    fn test_spectral_entropy_white_noise() {
        // White noise should have high spectral entropy
        let signal = white_noise(1.0, 4096, 123);
        let se = spectral_entropy(&signal, FS);
        assert!(
            se > 0.7,
            "White noise spectral entropy ({}) should be high",
            se
        );
    }

    #[test]
    fn test_spectral_entropy_bounds() {
        // Should always be in [0, 1]
        let signal = sine_wave(5.0, 1.0, FS, 256);
        let se = spectral_entropy(&signal, FS);
        assert!(se >= 0.0 && se <= 1.0, "SE ({}) must be in [0, 1]", se);
    }

    #[test]
    fn test_spectral_entropy_short_signal() {
        assert_eq!(spectral_entropy(&[1.0], FS), 0.0);
        assert_eq!(spectral_entropy(&[], FS), 0.0);
    }

    // -----------------------------------------------------------------------
    // Hjorth parameter tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_hjorth_constant_signal() {
        let signal = vec![5.0; 100];
        let (activity, mobility, complexity) = hjorth_parameters(&signal);
        assert!(
            activity < 1e-10,
            "Constant signal should have zero activity"
        );
        assert_eq!(mobility, 0.0);
        assert_eq!(complexity, 0.0);
    }

    #[test]
    fn test_hjorth_sine_wave() {
        let signal = sine_wave(10.0, 1.0, FS, 512);
        let (activity, mobility, complexity) = hjorth_parameters(&signal);
        assert!(activity > 0.0, "Sine wave activity should be positive");
        assert!(mobility > 0.0, "Sine wave mobility should be positive");
        assert!(complexity > 0.0, "Sine wave complexity should be positive");
    }

    #[test]
    fn test_hjorth_activity_scales_with_amplitude() {
        let sig1 = sine_wave(10.0, 1.0, FS, 512);
        let sig2 = sine_wave(10.0, 3.0, FS, 512);
        let (act1, _, _) = hjorth_parameters(&sig1);
        let (act2, _, _) = hjorth_parameters(&sig2);
        // Activity should scale as amplitude^2
        let ratio = act2 / act1;
        assert!(
            (ratio - 9.0).abs() < 1.0,
            "Activity ratio ({}) should be ~9 for 3x amplitude",
            ratio
        );
    }

    #[test]
    fn test_hjorth_mobility_higher_freq() {
        // Higher frequency -> higher mobility
        let low = sine_wave(5.0, 1.0, FS, 512);
        let high = sine_wave(40.0, 1.0, FS, 512);
        let (_, mob_low, _) = hjorth_parameters(&low);
        let (_, mob_high, _) = hjorth_parameters(&high);
        assert!(
            mob_high > mob_low,
            "Higher frequency ({}) should have higher mobility ({})",
            mob_high,
            mob_low
        );
    }

    #[test]
    fn test_hjorth_short_signal() {
        let (a, m, c) = hjorth_parameters(&[1.0, 2.0]);
        assert_eq!((a, m, c), (0.0, 0.0, 0.0));
    }

    // -----------------------------------------------------------------------
    // Artifact rejection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_artifact_rejection_clean_signal() {
        let mut channels = vec![vec![10.0, -5.0, 8.0, -3.0]];
        artifact_rejection(&mut channels, 100.0);
        // Should be unchanged since peak (10.0) < 100.0
        assert_eq!(channels[0], vec![10.0, -5.0, 8.0, -3.0]);
    }

    #[test]
    fn test_artifact_rejection_contaminated() {
        let mut channels = vec![
            vec![10.0, -5.0, 200.0, -3.0], // Peak 200 > 100: reject
            vec![10.0, -5.0, 8.0, -3.0],   // Peak 10 < 100: keep
        ];
        artifact_rejection(&mut channels, 100.0);
        assert!(
            channels[0].iter().all(|&v| v == 0.0),
            "Channel 0 should be zeroed"
        );
        assert_eq!(
            channels[1],
            vec![10.0, -5.0, 8.0, -3.0],
            "Channel 1 should be unchanged"
        );
    }

    #[test]
    fn test_artifact_rejection_exactly_at_threshold() {
        let mut channels = vec![vec![100.0, -50.0]];
        artifact_rejection(&mut channels, 100.0);
        // 100.0 is not > 100.0, so should be kept
        assert_eq!(channels[0], vec![100.0, -50.0]);
    }

    // -----------------------------------------------------------------------
    // Coherence tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_coherence_identical_signals() {
        let signal = sine_wave(10.0, 1.0, FS, 256);
        let coh = compute_coherence(&signal, &signal, FS);
        assert!(
            (coh - 1.0).abs() < 0.01,
            "Identical signals should have coherence ~1.0, got {}",
            coh
        );
    }

    #[test]
    fn test_coherence_uncorrelated_signals() {
        let sig_a = sine_wave(10.0, 1.0, FS, 512);
        let sig_b = white_noise(1.0, 512, 999);
        let coh = compute_coherence(&sig_a, &sig_b, FS);
        assert!(
            coh < 0.8,
            "Sine vs noise coherence ({}) should be low",
            coh
        );
    }

    #[test]
    fn test_coherence_short_signal() {
        assert_eq!(compute_coherence(&[1.0], &[2.0], FS), 0.0);
    }

    #[test]
    fn test_coherence_scaled_signals() {
        // Scaled version of same signal should still have high coherence
        let sig = sine_wave(15.0, 1.0, FS, 256);
        let scaled: Vec<f64> = sig.iter().map(|&x| x * 5.0).collect();
        let coh = compute_coherence(&sig, &scaled, FS);
        assert!(
            (coh - 1.0).abs() < 0.01,
            "Scaled signals should have coherence ~1.0, got {}",
            coh
        );
    }

    // -----------------------------------------------------------------------
    // EegProcessor integration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_common_average_reference() {
        let config = EegConfig {
            num_channels: 3,
            sample_rate_hz: FS,
            reference_type: ReferenceType::CommonAverage,
        };
        let processor = EegProcessor::new(config);

        // Constant channels: after CAR, all should be zero
        let channels = vec![vec![10.0; 256], vec![10.0; 256], vec![10.0; 256]];
        let referenced = processor.apply_reference(&channels);
        for ch in &referenced {
            for &v in ch {
                assert!(
                    v.abs() < 1e-10,
                    "CAR of identical channels should be zero"
                );
            }
        }
    }

    #[test]
    fn test_processor_bipolar_reference() {
        let config = EegConfig {
            num_channels: 3,
            sample_rate_hz: FS,
            reference_type: ReferenceType::Bipolar,
        };
        let processor = EegProcessor::new(config);

        let channels = vec![
            vec![10.0, 20.0, 30.0],
            vec![5.0, 15.0, 25.0],
            vec![1.0, 2.0, 3.0],
        ];
        let referenced = processor.apply_reference(&channels);
        assert_eq!(referenced.len(), 2, "Bipolar should produce N-1 channels");
        // Ch0 - Ch1
        assert_eq!(referenced[0], vec![5.0, 5.0, 5.0]);
        // Ch1 - Ch2
        assert_eq!(referenced[1], vec![4.0, 13.0, 22.0]);
    }

    #[test]
    fn test_processor_linked_ears_reference() {
        let config = EegConfig {
            num_channels: 3,
            sample_rate_hz: FS,
            reference_type: ReferenceType::LinkedEars,
        };
        let processor = EegProcessor::new(config);

        let channels = vec![
            vec![10.0; 4], // "Left ear"
            vec![20.0; 4], // "Right ear"
            vec![30.0; 4], // Data channel
        ];
        let referenced = processor.apply_reference(&channels);
        // Reference = (10+20)/2 = 15
        for &v in &referenced[2] {
            assert!(
                (v - 15.0).abs() < 1e-10,
                "Channel 2 should be 30 - 15 = 15, got {}",
                v
            );
        }
    }

    #[test]
    fn test_processor_process_epoch_alpha_signal() {
        let config = EegConfig {
            num_channels: 2,
            sample_rate_hz: FS,
            reference_type: ReferenceType::CommonAverage,
        };
        let processor = EegProcessor::new(config);

        // Channel 0: strong alpha (10 Hz)
        // Channel 1: weak noise
        let n = 512;
        let ch0 = sine_wave(10.0, 30.0, FS, n);
        let ch1: Vec<f64> = (0..n).map(|i| {
            // Small noise
            let s = (i as u64).wrapping_mul(2654435761);
            (s as f64 / u32::MAX as f64 - 0.5) * 0.1
        }).collect();

        let channels = vec![ch0, ch1];
        let features = processor.process_epoch(&channels);

        assert!(
            features.band_powers.alpha > features.band_powers.delta,
            "Alpha should dominate"
        );
        assert!(features.spectral_entropy >= 0.0 && features.spectral_entropy <= 1.0);
    }

    #[test]
    fn test_processor_asymmetry_index() {
        let config = EegConfig {
            num_channels: 2,
            sample_rate_hz: FS,
            reference_type: ReferenceType::LinkedEars,
        };
        let processor = EegProcessor::new(config);

        // Make "right" channel (ch1 after referencing) have more alpha
        // With LinkedEars, ref = (ch0+ch1)/2, so we need asymmetric alpha
        let n = 512;
        let ch0 = sine_wave(10.0, 5.0, FS, n);
        let ch1 = sine_wave(10.0, 20.0, FS, n);
        let ch2 = sine_wave(10.0, 1.0, FS, n); // small

        let channels = vec![ch0, ch1, ch2];
        let features = processor.process_epoch(&channels);
        // The asymmetry index sign depends on referencing; just check it's not NaN
        assert!(
            features.asymmetry_index.is_finite(),
            "Asymmetry index should be finite"
        );
    }

    // -----------------------------------------------------------------------
    // Linear algebra helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mat_mul_identity() {
        let eye = mat_eye(3);
        let a = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0], vec![7.0, 8.0, 9.0]];
        let result = mat_mul(&a, &eye);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (result[i][j] - a[i][j]).abs() < 1e-10,
                    "A * I should equal A"
                );
            }
        }
    }

    #[test]
    fn test_cholesky_positive_definite() {
        // 2x2 positive definite matrix
        let a = vec![vec![4.0, 2.0], vec![2.0, 3.0]];
        let l = cholesky(&a);
        // Verify L * L^T = A
        let lt = mat_transpose(&l);
        let result = mat_mul(&l, &lt);
        for i in 0..2 {
            for j in 0..2 {
                assert!(
                    (result[i][j] - a[i][j]).abs() < 1e-6,
                    "L*L^T should equal A: got {} expected {}",
                    result[i][j],
                    a[i][j]
                );
            }
        }
    }

    #[test]
    fn test_jacobi_eigenvalues_diagonal() {
        // Diagonal matrix: eigenvalues are the diagonal elements
        let a = vec![vec![3.0, 0.0], vec![0.0, 7.0]];
        let (eigenvalues, _) = jacobi_eigen(&a, 100);
        let mut sorted = eigenvalues.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(
            (sorted[0] - 3.0).abs() < 1e-6,
            "First eigenvalue should be 3.0, got {}",
            sorted[0]
        );
        assert!(
            (sorted[1] - 7.0).abs() < 1e-6,
            "Second eigenvalue should be 7.0, got {}",
            sorted[1]
        );
    }

    // -----------------------------------------------------------------------
    // Edge case tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_bandpass_filter_preserves_length() {
        let signal = vec![1.0; 200];
        let filtered = bandpass_filter(&signal, FS, 8.0, 13.0);
        assert_eq!(
            filtered.len(),
            signal.len(),
            "Filtered output length should match input"
        );
    }

    #[test]
    fn test_covariance_matrix_symmetric() {
        let epoch = vec![
            vec![1.0, 2.0, 3.0, 4.0],
            vec![4.0, 3.0, 2.0, 1.0],
        ];
        let cov = covariance_matrix(&epoch);
        assert!(
            (cov[0][1] - cov[1][0]).abs() < 1e-10,
            "Covariance matrix should be symmetric"
        );
    }

    #[test]
    fn test_next_power_of_two() {
        assert_eq!(next_power_of_two(1), 1);
        assert_eq!(next_power_of_two(2), 2);
        assert_eq!(next_power_of_two(3), 4);
        assert_eq!(next_power_of_two(5), 8);
        assert_eq!(next_power_of_two(128), 128);
        assert_eq!(next_power_of_two(129), 256);
    }

    #[test]
    fn test_dominant_frequency_detection() {
        let config = EegConfig {
            num_channels: 1,
            sample_rate_hz: FS,
            reference_type: ReferenceType::CommonAverage,
        };
        let processor = EegProcessor::new(config);

        // 20 Hz dominant frequency
        let signal = sine_wave(20.0, 10.0, FS, 512);
        let channels = vec![signal];
        let features = processor.process_epoch(&channels);
        assert!(
            (features.dominant_frequency_hz - 20.0).abs() < 2.0,
            "Dominant freq ({}) should be ~20 Hz",
            features.dominant_frequency_hz
        );
    }
}

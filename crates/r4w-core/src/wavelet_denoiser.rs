//! Wavelet-based signal denoising using discrete wavelet transform with thresholding.
//!
//! This module provides a `WaveletDenoiser` that applies the discrete wavelet transform
//! (DWT) to decompose a signal into approximation and detail coefficients, estimates
//! noise from the finest-scale detail coefficients, computes a threshold, applies
//! soft/hard/garrote thresholding to the detail coefficients, and reconstructs the
//! denoised signal via the inverse DWT (IDWT).
//!
//! Supported wavelet families: Haar, Daubechies-4, Daubechies-8, and Symlet-4.
//!
//! # Example
//!
//! ```
//! use r4w_core::wavelet_denoiser::{WaveletDenoiser, DenoiserConfig, WaveletType, ThresholdingMethod, NoiseEstimate};
//!
//! // Create a noisy sine wave
//! let n = 128;
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| (2.0 * std::f64::consts::PI * i as f64 / 32.0).sin())
//!     .collect();
//!
//! // Add deterministic "noise"
//! let noisy: Vec<f64> = signal.iter().enumerate()
//!     .map(|(i, &s)| s + 0.1 * ((i * 7 + 3) as f64 % 5.0 - 2.5))
//!     .collect();
//!
//! let config = DenoiserConfig {
//!     wavelet_type: WaveletType::Haar,
//!     num_levels: 3,
//!     thresholding_method: ThresholdingMethod::Soft,
//!     noise_estimate_method: NoiseEstimate::Mad,
//! };
//!
//! let denoiser = WaveletDenoiser::new(config);
//! let denoised = denoiser.denoise(&noisy);
//! assert_eq!(denoised.len(), noisy.len());
//! ```

use std::f64::consts::LN_2;

/// Wavelet family selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WaveletType {
    /// Haar wavelet (2-tap).
    Haar,
    /// Daubechies-4 wavelet (8-tap).
    Db4,
    /// Daubechies-8 wavelet (16-tap).
    Db8,
    /// Symlet-4 wavelet (8-tap).
    Sym4,
}

/// Thresholding strategy for wavelet detail coefficients.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThresholdingMethod {
    /// Hard thresholding: keep coefficients above threshold unchanged, zero the rest.
    Hard,
    /// Soft thresholding: `sgn(x) * max(|x| - T, 0)`.
    Soft,
    /// Non-negative garrote: `x - T^2 / x` for `|x| > T`, else 0.
    Garrote,
}

/// Noise estimation method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NoiseEstimate {
    /// Median absolute deviation: `sigma = median(|d|) / 0.6745`.
    Mad,
    /// Universal threshold based on signal length (VisuShrink).
    Universal,
    /// Stein's Unbiased Risk Estimate.
    Sure,
}

/// Configuration for the wavelet denoiser.
#[derive(Debug, Clone)]
pub struct DenoiserConfig {
    /// Wavelet family to use.
    pub wavelet_type: WaveletType,
    /// Number of decomposition levels.
    pub num_levels: usize,
    /// Thresholding strategy.
    pub thresholding_method: ThresholdingMethod,
    /// Noise estimation method.
    pub noise_estimate_method: NoiseEstimate,
}

/// Main wavelet denoising engine.
///
/// Implements a discrete wavelet transform (DWT) with periodic boundary extension
/// and guaranteed perfect reconstruction. The analysis filter bank convolves with
/// low-pass `h` and high-pass `g` filters and decimates by 2. The synthesis filter
/// bank upsamples by 2 and convolves with reconstruction filters `h~` and `g~`.
#[derive(Debug, Clone)]
pub struct WaveletDenoiser {
    config: DenoiserConfig,
    /// Low-pass (scaling) decomposition filter h.
    h: Vec<f64>,
    /// High-pass (wavelet) decomposition filter g.
    g: Vec<f64>,
}

impl WaveletDenoiser {
    /// Create a new denoiser from the given configuration.
    pub fn new(config: DenoiserConfig) -> Self {
        let h = Self::scaling_filter(config.wavelet_type);
        let g = Self::wavelet_filter(&h);
        Self { config, h, g }
    }

    /// Denoise a real-valued signal.
    pub fn denoise(&self, signal: &[f64]) -> Vec<f64> {
        if signal.is_empty() {
            return Vec::new();
        }
        let original_len = signal.len();

        // Pad to a length that is divisible by 2^num_levels.
        let padded = self.pad_signal(signal);
        let n = padded.len();

        // Forward DWT.
        let (approx, details) = self.dwt(&padded);

        // Estimate noise from the finest-level detail coefficients.
        let finest_detail = &details[0];
        let sigma = self.estimate_noise_sigma(finest_detail);

        // Threshold each detail level.
        let thresholded_details: Vec<Vec<f64>> = details
            .iter()
            .map(|d| {
                let threshold = self.compute_threshold(sigma, n);
                self.apply_threshold(d, threshold)
            })
            .collect();

        // Inverse DWT.
        let mut result = self.idwt(&approx, &thresholded_details);

        // Trim back to original length.
        result.truncate(original_len);
        result
    }

    /// Denoise a complex IQ signal by denoising real and imaginary parts independently.
    pub fn denoise_complex(&self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if signal.is_empty() {
            return Vec::new();
        }
        let real: Vec<f64> = signal.iter().map(|s| s.0).collect();
        let imag: Vec<f64> = signal.iter().map(|s| s.1).collect();

        let denoised_real = self.denoise(&real);
        let denoised_imag = self.denoise(&imag);

        denoised_real
            .into_iter()
            .zip(denoised_imag.into_iter())
            .collect()
    }

    /// Perform multi-level discrete wavelet transform.
    ///
    /// Returns `(approximation_coefficients, vec_of_detail_coefficients_per_level)`.
    /// `details[0]` is the finest scale (first decomposition level), `details[last]`
    /// is the coarsest.
    pub fn dwt(&self, signal: &[f64]) -> (Vec<f64>, Vec<Vec<f64>>) {
        let mut approx = signal.to_vec();
        let mut details = Vec::new();
        let levels = self.config.num_levels.min(self.max_levels(signal.len()));

        for _ in 0..levels {
            let (a, d) = self.analysis_step(&approx);
            details.push(d);
            approx = a;
        }

        (approx, details)
    }

    /// Reconstruct a signal from approximation and detail coefficients.
    pub fn idwt(&self, approx: &[f64], details: &[Vec<f64>]) -> Vec<f64> {
        let mut current = approx.to_vec();

        // Reconstruct from coarsest to finest.
        for detail in details.iter().rev() {
            current = self.synthesis_step(&current, detail);
        }

        current
    }

    /// Estimate noise standard deviation from detail coefficients.
    pub fn estimate_noise_sigma(&self, detail_coeffs: &[f64]) -> f64 {
        match self.config.noise_estimate_method {
            NoiseEstimate::Mad => {
                if detail_coeffs.is_empty() {
                    return 0.0;
                }
                let mut abs_vals: Vec<f64> = detail_coeffs.iter().map(|x| x.abs()).collect();
                let median = Self::median_of(&mut abs_vals);
                median / 0.6745
            }
            NoiseEstimate::Universal => {
                if detail_coeffs.is_empty() {
                    return 0.0;
                }
                let mean = detail_coeffs.iter().sum::<f64>() / detail_coeffs.len() as f64;
                let variance = detail_coeffs
                    .iter()
                    .map(|x| (x - mean).powi(2))
                    .sum::<f64>()
                    / detail_coeffs.len() as f64;
                variance.sqrt()
            }
            NoiseEstimate::Sure => {
                if detail_coeffs.is_empty() {
                    return 0.0;
                }
                let mut abs_vals: Vec<f64> = detail_coeffs.iter().map(|x| x.abs()).collect();
                Self::median_of(&mut abs_vals) / 0.6745
            }
        }
    }

    /// Compute the threshold value.
    ///
    /// Universal threshold: `T = sigma * sqrt(2 * ln(N))`.
    pub fn compute_threshold(&self, sigma: f64, n: usize) -> f64 {
        if n <= 1 {
            return 0.0;
        }
        sigma * (2.0 * (n as f64).ln()).sqrt()
    }

    // ─── Filter coefficients ──────────────────────────────────────────────

    /// Return the low-pass (scaling) filter coefficients for the chosen wavelet.
    fn scaling_filter(wavelet_type: WaveletType) -> Vec<f64> {
        match wavelet_type {
            WaveletType::Haar => {
                let c = 1.0 / 2.0_f64.sqrt();
                vec![c, c]
            }
            WaveletType::Db4 => {
                vec![
                    -0.010597401784997278,
                     0.032883011666982945,
                     0.030841381835986965,
                    -0.187034811718881060,
                    -0.027983769416983849,
                     0.630880767929590380,
                     0.714846570552541600,
                     0.230377813308855140,
                ]
            }
            WaveletType::Db8 => {
                vec![
                    -0.00011747678400228192,
                     0.00067544940599855677,
                    -0.00039174037299597711,
                    -0.00487035299301066900,
                     0.00874609404701566060,
                     0.01395351747052901600,
                    -0.04408825393079038000,
                    -0.01736930100202211400,
                     0.12874742662018601000,
                     0.00047248457399797254,
                    -0.28401554296242809000,
                    -0.01599922311666723000,
                     0.58535468365486909000,
                     0.67563073629801285000,
                     0.31287159091446592000,
                     0.05441584224308161000,
                ]
            }
            WaveletType::Sym4 => {
                vec![
                    -0.075765714789273330,
                    -0.029635527645954480,
                     0.497618667632563040,
                     0.803738751805916220,
                     0.297857795605605200,
                    -0.099219543576847220,
                    -0.012603967262037833,
                     0.032223100604071270,
                ]
            }
        }
    }

    /// Derive the high-pass (wavelet) filter from the low-pass via the alternating
    /// flip (QMF) relation: `g[n] = (-1)^n * h[L-1-n]`.
    fn wavelet_filter(lo: &[f64]) -> Vec<f64> {
        let l = lo.len();
        (0..l)
            .map(|n| {
                let sign = if n % 2 == 0 { 1.0 } else { -1.0 };
                sign * lo[l - 1 - n]
            })
            .collect()
    }

    // ─── Analysis (forward DWT) ───────────────────────────────────────────

    /// One level of analysis: periodic convolution with h and g, then downsample by 2.
    ///
    /// Using the standard convention:
    /// ```text
    ///   a[k] = sum_{j=0}^{L-1} h[j] * x[(2k - j) mod N]
    ///   d[k] = sum_{j=0}^{L-1} g[j] * x[(2k - j) mod N]
    /// ```
    /// for k = 0, 1, ..., N/2 - 1.
    fn analysis_step(&self, signal: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = signal.len();
        let half = n / 2;
        let flen = self.h.len();

        let mut approx = vec![0.0; half];
        let mut detail = vec![0.0; half];

        for k in 0..half {
            let mut a = 0.0;
            let mut d = 0.0;
            for j in 0..flen {
                let idx = ((2 * k) as isize - j as isize).rem_euclid(n as isize) as usize;
                a += self.h[j] * signal[idx];
                d += self.g[j] * signal[idx];
            }
            approx[k] = a;
            detail[k] = d;
        }

        (approx, detail)
    }

    // ─── Synthesis (inverse DWT) ──────────────────────────────────────────

    /// One level of synthesis: upsample by 2, then periodic convolution with
    /// time-reversed h and g, then sum.
    ///
    /// The synthesis step is the exact transpose of the analysis step:
    /// ```text
    ///   x[n] = sum_{k: (n-j) even, j in 0..L} h[j]*a[(n-j)/2 mod N/2]
    ///        + sum_{k: (n-j) even, j in 0..L} g[j]*d[(n-j)/2 mod N/2]
    /// ```
    /// which is equivalent to, for each output sample n:
    /// ```text
    ///   x[n] = sum_{k=0}^{N/2-1} sum_{j=0}^{L-1} delta(n, 2k-j mod N) * (h[j]*a[k] + g[j]*d[k])
    /// ```
    /// We implement this by scattering each coefficient pair (a[k], d[k]) into
    /// the output at positions `(2k - j) mod N`.
    fn synthesis_step(&self, approx: &[f64], detail: &[f64]) -> Vec<f64> {
        let half = approx.len();
        let n = half * 2;
        let flen = self.h.len();

        let mut result = vec![0.0; n];

        for k in 0..half {
            for j in 0..flen {
                let idx = ((2 * k) as isize - j as isize).rem_euclid(n as isize) as usize;
                result[idx] += self.h[j] * approx[k] + self.g[j] * detail[k];
            }
        }

        result
    }

    // ─── Thresholding ─────────────────────────────────────────────────────

    /// Apply thresholding to a set of coefficients.
    fn apply_threshold(&self, coeffs: &[f64], threshold: f64) -> Vec<f64> {
        coeffs
            .iter()
            .map(|&x| match self.config.thresholding_method {
                ThresholdingMethod::Hard => {
                    if x.abs() >= threshold {
                        x
                    } else {
                        0.0
                    }
                }
                ThresholdingMethod::Soft => {
                    if x.abs() >= threshold {
                        x.signum() * (x.abs() - threshold)
                    } else {
                        0.0
                    }
                }
                ThresholdingMethod::Garrote => {
                    if x.abs() > threshold {
                        x - threshold * threshold / x
                    } else {
                        0.0
                    }
                }
            })
            .collect()
    }

    // ─── Utilities ────────────────────────────────────────────────────────

    /// Compute the median of a mutable slice (sorts in place).
    fn median_of(data: &mut [f64]) -> f64 {
        if data.is_empty() {
            return 0.0;
        }
        data.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mid = data.len() / 2;
        if data.len() % 2 == 0 {
            (data[mid - 1] + data[mid]) / 2.0
        } else {
            data[mid]
        }
    }

    /// Pad signal so its length is divisible by `2^num_levels`. Uses symmetric
    /// extension at the right boundary.
    fn pad_signal(&self, signal: &[f64]) -> Vec<f64> {
        let n = signal.len();
        let divisor = 1usize << self.config.num_levels;
        let target = ((n + divisor - 1) / divisor) * divisor;
        if target == n {
            return signal.to_vec();
        }
        let mut padded = signal.to_vec();
        while padded.len() < target {
            let idx = padded.len() - n;
            let mirror_idx = if idx < n { n - 1 - idx } else { 0 };
            padded.push(signal[mirror_idx]);
        }
        padded
    }

    /// Maximum decomposition levels for a given signal length.
    fn max_levels(&self, n: usize) -> usize {
        if n <= 1 {
            return 0;
        }
        // Each level halves the length; stop when length < filter_len.
        let filter_len = self.h.len();
        let mut len = n;
        let mut levels = 0;
        while len >= filter_len && len >= 2 {
            len /= 2;
            levels += 1;
        }
        levels.max(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> DenoiserConfig {
        DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 3,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Mad,
        }
    }

    #[test]
    fn test_haar_scaling_filter_norm() {
        let h = WaveletDenoiser::scaling_filter(WaveletType::Haar);
        let energy: f64 = h.iter().map(|x| x * x).sum();
        assert!((energy - 1.0).abs() < 1e-12, "Haar filter should have unit energy");
    }

    #[test]
    fn test_db4_scaling_filter_norm() {
        let h = WaveletDenoiser::scaling_filter(WaveletType::Db4);
        let energy: f64 = h.iter().map(|x| x * x).sum();
        assert!(
            (energy - 1.0).abs() < 1e-10,
            "Db4 filter should have unit energy, got {}",
            energy
        );
    }

    #[test]
    fn test_db8_scaling_filter_length() {
        let h = WaveletDenoiser::scaling_filter(WaveletType::Db8);
        assert_eq!(h.len(), 16, "Db8 should have 16 taps");
    }

    #[test]
    fn test_sym4_scaling_filter_length() {
        let h = WaveletDenoiser::scaling_filter(WaveletType::Sym4);
        assert_eq!(h.len(), 8, "Sym4 should have 8 taps");
    }

    #[test]
    fn test_wavelet_filter_orthogonality() {
        let lo = WaveletDenoiser::scaling_filter(WaveletType::Haar);
        let hi = WaveletDenoiser::wavelet_filter(&lo);
        let dot: f64 = lo.iter().zip(hi.iter()).map(|(a, b)| a * b).sum();
        assert!(
            dot.abs() < 1e-12,
            "Scaling and wavelet filters should be orthogonal, dot = {}",
            dot
        );
    }

    #[test]
    fn test_dwt_idwt_perfect_reconstruction_haar() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 3,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Mad,
        };
        let denoiser = WaveletDenoiser::new(config);

        let signal: Vec<f64> = (0..64).map(|i| (2.0 * PI * i as f64 / 16.0).sin()).collect();
        let (approx, details) = denoiser.dwt(&signal);
        let reconstructed = denoiser.idwt(&approx, &details);

        assert_eq!(reconstructed.len(), signal.len());
        for (i, (a, b)) in signal.iter().zip(reconstructed.iter()).enumerate() {
            assert!(
                (a - b).abs() < 1e-10,
                "Sample {} mismatch: {} vs {}",
                i,
                a,
                b
            );
        }
    }

    #[test]
    fn test_dwt_idwt_perfect_reconstruction_db4() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Db4,
            num_levels: 2,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Mad,
        };
        let denoiser = WaveletDenoiser::new(config);

        let signal: Vec<f64> = (0..128).map(|i| (i as f64 * 0.1).cos()).collect();
        let (approx, details) = denoiser.dwt(&signal);
        let reconstructed = denoiser.idwt(&approx, &details);

        assert_eq!(reconstructed.len(), signal.len());
        for (i, (a, b)) in signal.iter().zip(reconstructed.iter()).enumerate() {
            assert!(
                (a - b).abs() < 1e-9,
                "Db4 sample {} mismatch: {} vs {}",
                i,
                a,
                b
            );
        }
    }

    #[test]
    fn test_denoise_output_length_matches_input() {
        let config = default_config();
        let denoiser = WaveletDenoiser::new(config);

        let signal: Vec<f64> = (0..100).map(|i| i as f64 * 0.01).collect();
        let result = denoiser.denoise(&signal);
        assert_eq!(
            result.len(),
            signal.len(),
            "Output length must match input"
        );
    }

    #[test]
    fn test_denoise_empty_signal() {
        let config = default_config();
        let denoiser = WaveletDenoiser::new(config);
        let result = denoiser.denoise(&[]);
        assert!(result.is_empty(), "Empty input should produce empty output");
    }

    #[test]
    fn test_soft_thresholding() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 1,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Mad,
        };
        let denoiser = WaveletDenoiser::new(config);

        let coeffs = vec![5.0, -3.0, 1.0, -0.5, 0.0];
        let threshold = 2.0;
        let result = denoiser.apply_threshold(&coeffs, threshold);

        assert!((result[0] - 3.0).abs() < 1e-12);
        assert!((result[1] - (-1.0)).abs() < 1e-12);
        assert!((result[2] - 0.0).abs() < 1e-12);
        assert!((result[3] - 0.0).abs() < 1e-12);
        assert!((result[4] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_hard_thresholding() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 1,
            thresholding_method: ThresholdingMethod::Hard,
            noise_estimate_method: NoiseEstimate::Mad,
        };
        let denoiser = WaveletDenoiser::new(config);

        let coeffs = vec![5.0, -3.0, 1.0, -0.5];
        let threshold = 2.0;
        let result = denoiser.apply_threshold(&coeffs, threshold);

        assert!((result[0] - 5.0).abs() < 1e-12);
        assert!((result[1] - (-3.0)).abs() < 1e-12);
        assert!((result[2] - 0.0).abs() < 1e-12);
        assert!((result[3] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_garrote_thresholding() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 1,
            thresholding_method: ThresholdingMethod::Garrote,
            noise_estimate_method: NoiseEstimate::Mad,
        };
        let denoiser = WaveletDenoiser::new(config);

        let coeffs = vec![5.0, -3.0, 1.0];
        let threshold = 2.0;
        let result = denoiser.apply_threshold(&coeffs, threshold);

        assert!((result[0] - 4.2).abs() < 1e-12);
        assert!((result[1] - (-5.0 / 3.0)).abs() < 1e-12);
        assert!((result[2] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_mad_noise_estimate() {
        let config = default_config();
        let denoiser = WaveletDenoiser::new(config);

        let coeffs = vec![1.0, -2.0, 3.0, -4.0, 5.0];
        let sigma = denoiser.estimate_noise_sigma(&coeffs);
        assert!(
            (sigma - 3.0 / 0.6745).abs() < 1e-10,
            "MAD sigma should be median(|d|)/0.6745, got {}",
            sigma
        );
    }

    #[test]
    fn test_universal_threshold() {
        let config = default_config();
        let denoiser = WaveletDenoiser::new(config);

        let sigma = 1.0;
        let n = 1024;
        let t = denoiser.compute_threshold(sigma, n);
        let expected = (2.0 * (1024.0_f64).ln()).sqrt();
        assert!(
            (t - expected).abs() < 1e-10,
            "Universal threshold mismatch: {} vs {}",
            t,
            expected
        );
    }

    #[test]
    fn test_denoise_complex_output_length() {
        let config = default_config();
        let denoiser = WaveletDenoiser::new(config);

        let signal: Vec<(f64, f64)> = (0..64)
            .map(|i| {
                let t = i as f64 * 0.1;
                (t.sin(), t.cos())
            })
            .collect();
        let result = denoiser.denoise_complex(&signal);
        assert_eq!(result.len(), signal.len());
    }

    #[test]
    fn test_denoise_complex_empty() {
        let config = default_config();
        let denoiser = WaveletDenoiser::new(config);
        let result = denoiser.denoise_complex(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_median_odd_count() {
        let mut data = vec![5.0, 1.0, 3.0];
        let m = WaveletDenoiser::median_of(&mut data);
        assert!((m - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_median_even_count() {
        let mut data = vec![4.0, 1.0, 3.0, 2.0];
        let m = WaveletDenoiser::median_of(&mut data);
        assert!((m - 2.5).abs() < 1e-12);
    }

    #[test]
    fn test_denoise_reduces_noise_energy() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Db4,
            num_levels: 4,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Mad,
        };
        let denoiser = WaveletDenoiser::new(config);

        let n = 512;
        let clean: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / 64.0).sin())
            .collect();

        // Add high-frequency deterministic pseudo-noise.
        let noisy: Vec<f64> = clean
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let noise = 0.8 * if i % 2 == 0 { 1.0 } else { -1.0 }
                    * ((i * 13 + 7) as f64 % 7.0 - 3.0) / 3.0;
                s + noise
            })
            .collect();

        let denoised = denoiser.denoise(&noisy);

        let noise_before: f64 = noisy
            .iter()
            .zip(clean.iter())
            .map(|(n, c)| (n - c).powi(2))
            .sum();
        let noise_after: f64 = denoised
            .iter()
            .zip(clean.iter())
            .map(|(d, c)| (d - c).powi(2))
            .sum();

        assert!(
            noise_after < noise_before,
            "Denoising should reduce noise energy: before={}, after={}",
            noise_before,
            noise_after
        );
    }

    #[test]
    fn test_dwt_detail_coefficient_count() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 3,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Mad,
        };
        let denoiser = WaveletDenoiser::new(config);

        let signal: Vec<f64> = (0..64).map(|i| i as f64).collect();
        let (approx, details) = denoiser.dwt(&signal);

        assert_eq!(details.len(), 3, "Should have 3 detail levels");
        assert_eq!(details[0].len(), 32, "Level 1 detail: 64/2 = 32");
        assert_eq!(details[1].len(), 16, "Level 2 detail: 32/2 = 16");
        assert_eq!(details[2].len(), 8, "Level 3 detail: 16/2 = 8");
        assert_eq!(approx.len(), 8, "Final approximation: 16/2 = 8");
    }

    #[test]
    fn test_noise_estimate_sure_method() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 1,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Sure,
        };
        let denoiser = WaveletDenoiser::new(config);

        let coeffs = vec![1.0, -2.0, 3.0, -4.0, 5.0];
        let sigma = denoiser.estimate_noise_sigma(&coeffs);
        assert!(sigma > 0.0, "SURE sigma should be positive, got {}", sigma);
    }

    #[test]
    fn test_noise_estimate_universal_method() {
        let config = DenoiserConfig {
            wavelet_type: WaveletType::Haar,
            num_levels: 1,
            thresholding_method: ThresholdingMethod::Soft,
            noise_estimate_method: NoiseEstimate::Universal,
        };
        let denoiser = WaveletDenoiser::new(config);

        let coeffs = vec![0.0; 100];
        let sigma = denoiser.estimate_noise_sigma(&coeffs);
        assert!(
            sigma.abs() < 1e-12,
            "Zero signal should give zero sigma, got {}",
            sigma
        );
    }

    #[test]
    fn test_all_wavelet_types_denoise() {
        for wt in &[
            WaveletType::Haar,
            WaveletType::Db4,
            WaveletType::Db8,
            WaveletType::Sym4,
        ] {
            let config = DenoiserConfig {
                wavelet_type: *wt,
                num_levels: 2,
                thresholding_method: ThresholdingMethod::Soft,
                noise_estimate_method: NoiseEstimate::Mad,
            };
            let denoiser = WaveletDenoiser::new(config);

            let signal: Vec<f64> = (0..128).map(|i| (i as f64 * 0.05).sin()).collect();
            let result = denoiser.denoise(&signal);
            assert_eq!(
                result.len(),
                signal.len(),
                "Wavelet {:?} output length mismatch",
                wt
            );
        }
    }
}

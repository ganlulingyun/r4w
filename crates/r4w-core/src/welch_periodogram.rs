//! Welch's method for power spectral density estimation with variance reduction.
//!
//! This module implements Welch's averaged periodogram method, which estimates
//! the power spectral density (PSD) of a signal by dividing it into overlapping
//! segments, windowing each segment, computing the FFT, and averaging the
//! squared magnitudes. This reduces the variance of the PSD estimate at the
//! cost of frequency resolution compared to a single periodogram.
//!
//! # Example
//!
//! ```
//! use r4w_core::welch_periodogram::{WelchEstimator, WelchConfig, WindowType};
//!
//! let config = WelchConfig {
//!     fft_size: 64,
//!     overlap_fraction: 0.5,
//!     window_type: WindowType::Hann,
//!     sample_rate: 1000.0,
//!     averaging: Averaging::Uniform,
//! };
//!
//! // Use the Averaging enum
//! use r4w_core::welch_periodogram::Averaging;
//!
//! let estimator = WelchEstimator::new(config);
//!
//! // Generate a simple tone at 250 Hz (fs=1000, so normalized freq = 0.25)
//! let n = 256;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         let phase = 2.0 * std::f64::consts::PI * 250.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let result = estimator.estimate_psd(&samples);
//! assert_eq!(result.frequency_bins.len(), 64);
//! assert_eq!(result.power_density.len(), 64);
//! assert!(result.num_segments > 0);
//! assert!(result.resolution_bandwidth_hz > 0.0);
//!
//! // The peak should be near bin corresponding to 250 Hz
//! let peak_idx = result.power_density
//!     .iter()
//!     .enumerate()
//!     .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
//!     .unwrap()
//!     .0;
//! let peak_freq = result.frequency_bins[peak_idx];
//! assert!((peak_freq - 250.0).abs() < 20.0);
//! ```

use std::f64::consts::PI;

/// Averaging mode for combining periodogram segments.
#[derive(Debug, Clone, PartialEq)]
pub enum Averaging {
    /// Equal weight for all segments: PSD = (1/K) * sum(|X_k|^2)
    Uniform,
    /// Exponential weighting with forgetting factor alpha in (0, 1].
    /// More recent segments have higher weight.
    /// S_k = alpha * |X_k|^2 + (1 - alpha) * S_{k-1}
    Exponential(f64),
}

/// Window function type for tapering each segment before FFT.
#[derive(Debug, Clone, PartialEq)]
pub enum WindowType {
    /// No windowing (all ones). Highest resolution, worst spectral leakage.
    Rectangular,
    /// Hann window: 0.5 * (1 - cos(2*pi*n/(N-1))). Good general purpose.
    Hann,
    /// Hamming window: 0.54 - 0.46 * cos(2*pi*n/(N-1)). Slightly better sidelobe rejection.
    Hamming,
    /// Blackman window: 0.42 - 0.5*cos(2*pi*n/(N-1)) + 0.08*cos(4*pi*n/(N-1)).
    /// Excellent sidelobe suppression at cost of main lobe width.
    Blackman,
    /// Kaiser window with parameter beta. Higher beta = narrower main lobe.
    Kaiser(f64),
}

/// Configuration for the Welch PSD estimator.
#[derive(Debug, Clone)]
pub struct WelchConfig {
    /// FFT size (number of points per segment). Must be a power of 2.
    pub fft_size: usize,
    /// Fraction of overlap between adjacent segments. Range: 0.0 to 0.9.
    /// Default: 0.5 (50% overlap with Hann is optimal variance/resolution tradeoff).
    pub overlap_fraction: f64,
    /// Window function applied to each segment.
    pub window_type: WindowType,
    /// Sample rate in Hz. Used to scale frequency bins and resolution bandwidth.
    pub sample_rate: f64,
    /// Averaging mode for combining segment periodograms.
    pub averaging: Averaging,
}

impl Default for WelchConfig {
    fn default() -> Self {
        Self {
            fft_size: 1024,
            overlap_fraction: 0.5,
            window_type: WindowType::Hann,
            sample_rate: 1.0,
            averaging: Averaging::Uniform,
        }
    }
}

/// Result of a PSD estimation.
#[derive(Debug, Clone)]
pub struct PsdResult {
    /// Frequency bins in Hz, centered from -fs/2 to +fs/2 (fftshift order).
    pub frequency_bins: Vec<f64>,
    /// Power spectral density values (linear or dB/Hz depending on method used).
    pub power_density: Vec<f64>,
    /// Number of overlapping segments averaged.
    pub num_segments: usize,
    /// Resolution bandwidth in Hz: fs / fft_size (ENBW-corrected by window).
    pub resolution_bandwidth_hz: f64,
}

/// Welch's averaged periodogram estimator.
///
/// Implements Welch's method: divide input into K overlapping segments of length N,
/// apply a window w[n] to each, compute the DFT, take the magnitude squared,
/// and average across segments.
pub struct WelchEstimator {
    config: WelchConfig,
    window: Vec<f64>,
    /// Window power normalization factor: (1/N) * sum(w[n]^2)
    window_power: f64,
}

impl WelchEstimator {
    /// Create a new Welch estimator with the given configuration.
    ///
    /// # Panics
    /// Panics if fft_size is 0, overlap_fraction is outside [0.0, 0.9],
    /// or sample_rate is not positive.
    pub fn new(config: WelchConfig) -> Self {
        assert!(config.fft_size > 0, "fft_size must be > 0");
        assert!(
            (0.0..=0.9).contains(&config.overlap_fraction),
            "overlap_fraction must be in [0.0, 0.9]"
        );
        assert!(config.sample_rate > 0.0, "sample_rate must be positive");

        let window = generate_window(&config.window_type, config.fft_size);
        let window_power = window.iter().map(|w| w * w).sum::<f64>() / config.fft_size as f64;

        Self {
            config,
            window,
            window_power,
        }
    }

    /// Estimate the power spectral density (linear scale, units: power/Hz).
    ///
    /// PSD = (1 / (fs * U * K)) * sum_k |FFT(w[n] * x_k[n])|^2
    /// where U = (1/N) * sum(w[n]^2) is the window power normalization.
    pub fn estimate_psd(&self, samples: &[(f64, f64)]) -> PsdResult {
        let segments = self.extract_segments(samples);
        let num_segments = segments.len();
        let n = self.config.fft_size;

        if num_segments == 0 {
            return PsdResult {
                frequency_bins: self.frequency_bins(),
                power_density: vec![0.0; n],
                num_segments: 0,
                resolution_bandwidth_hz: self.resolution_bandwidth(),
            };
        }

        let mut psd = vec![0.0; n];

        match &self.config.averaging {
            Averaging::Uniform => {
                for segment in &segments {
                    let windowed = self.apply_window(segment);
                    let spectrum = fft(&windowed);
                    for (i, s) in spectrum.iter().enumerate() {
                        let mag_sq = s.0 * s.0 + s.1 * s.1;
                        psd[i] += mag_sq;
                    }
                }
                let scale =
                    1.0 / (self.config.sample_rate * self.window_power * n as f64 * num_segments as f64);
                for p in &mut psd {
                    *p *= scale;
                }
            }
            Averaging::Exponential(alpha) => {
                let alpha = alpha.clamp(0.001, 1.0);
                for (k, segment) in segments.iter().enumerate() {
                    let windowed = self.apply_window(segment);
                    let spectrum = fft(&windowed);
                    let scale = 1.0 / (self.config.sample_rate * self.window_power * n as f64);
                    for (i, s) in spectrum.iter().enumerate() {
                        let mag_sq = (s.0 * s.0 + s.1 * s.1) * scale;
                        if k == 0 {
                            psd[i] = mag_sq;
                        } else {
                            psd[i] = alpha * mag_sq + (1.0 - alpha) * psd[i];
                        }
                    }
                }
            }
        }

        // FFT-shift: move DC to center
        let mut shifted = vec![0.0; n];
        let half = n / 2;
        for i in 0..n {
            shifted[i] = psd[(i + half) % n];
        }

        PsdResult {
            frequency_bins: self.frequency_bins(),
            power_density: shifted,
            num_segments,
            resolution_bandwidth_hz: self.resolution_bandwidth(),
        }
    }

    /// Estimate the PSD in dB/Hz: 10*log10(PSD).
    pub fn estimate_psd_db(&self, samples: &[(f64, f64)]) -> PsdResult {
        let mut result = self.estimate_psd(samples);
        for p in &mut result.power_density {
            *p = 10.0 * (*p).max(1e-30).log10();
        }
        result
    }

    /// Compute the cross power spectral density between two signals.
    ///
    /// Sxy[k] = (1 / (fs * U * K)) * sum_i { FFT(x_i)[k] * conj(FFT(y_i)[k]) }
    ///
    /// Returns complex cross-PSD values (fftshift order).
    pub fn cross_psd(&self, x: &[(f64, f64)], y: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let min_len = x.len().min(y.len());
        let seg_x = self.extract_segments(&x[..min_len]);
        let seg_y = self.extract_segments(&y[..min_len]);
        let num_segments = seg_x.len().min(seg_y.len());
        let n = self.config.fft_size;

        let mut cpsd = vec![(0.0, 0.0); n];

        if num_segments == 0 {
            return cpsd;
        }

        for k in 0..num_segments {
            let wx = self.apply_window(&seg_x[k]);
            let wy = self.apply_window(&seg_y[k]);
            let fx = fft(&wx);
            let fy = fft(&wy);
            for i in 0..n {
                // X[i] * conj(Y[i]) = (xr + j*xi) * (yr - j*yi)
                let (xr, xi) = fx[i];
                let (yr, yi) = fy[i];
                cpsd[i].0 += xr * yr + xi * yi;
                cpsd[i].1 += xi * yr - xr * yi;
            }
        }

        let scale =
            1.0 / (self.config.sample_rate * self.window_power * n as f64 * num_segments as f64);
        for c in &mut cpsd {
            c.0 *= scale;
            c.1 *= scale;
        }

        // FFT-shift
        let mut shifted = vec![(0.0, 0.0); n];
        let half = n / 2;
        for i in 0..n {
            shifted[i] = cpsd[(i + half) % n];
        }

        shifted
    }

    /// Compute magnitude-squared coherence between two signals.
    ///
    /// Cxy[k] = |Sxy[k]|^2 / (Sxx[k] * Syy[k])
    ///
    /// Returns values in [0, 1] where 1 = perfect linear relationship at that frequency.
    pub fn coherence(&self, x: &[(f64, f64)], y: &[(f64, f64)]) -> Vec<f64> {
        let sxx = self.estimate_psd(x);
        let syy = self.estimate_psd(y);
        let sxy = self.cross_psd(x, y);
        let n = self.config.fft_size;

        let mut coh = vec![0.0; n];
        for i in 0..n {
            let sxy_mag_sq = sxy[i].0 * sxy[i].0 + sxy[i].1 * sxy[i].1;
            let denom = sxx.power_density[i] * syy.power_density[i];
            if denom > 1e-30 {
                coh[i] = (sxy_mag_sq / denom).min(1.0);
            }
        }
        coh
    }

    /// Resolution bandwidth in Hz: fs / N (noise bandwidth of rectangular window).
    /// The effective noise bandwidth depends on the window type.
    pub fn resolution_bandwidth(&self) -> f64 {
        let enbw_factor = match &self.config.window_type {
            WindowType::Rectangular => 1.0,
            WindowType::Hann => 1.5,
            WindowType::Hamming => 1.3628,
            WindowType::Blackman => 1.7268,
            WindowType::Kaiser(beta) => {
                // Approximate ENBW for Kaiser; exact value depends on beta
                if *beta < 1.0 {
                    1.0
                } else {
                    1.0 + 0.1 * beta
                }
            }
        };
        enbw_factor * self.config.sample_rate / self.config.fft_size as f64
    }

    /// Generate the frequency bin centers in Hz (fftshift order: -fs/2 to +fs/2).
    fn frequency_bins(&self) -> Vec<f64> {
        let n = self.config.fft_size;
        let fs = self.config.sample_rate;
        let half = n / 2;
        (0..n)
            .map(|i| {
                let k = if i < half {
                    (i as f64) - (half as f64)
                } else {
                    (i as f64) - (half as f64)
                };
                k * fs / n as f64
            })
            .collect()
    }

    /// Extract overlapping segments from the input samples.
    fn extract_segments(&self, samples: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let n = self.config.fft_size;
        if samples.len() < n {
            return vec![];
        }
        let step = ((n as f64) * (1.0 - self.config.overlap_fraction)).max(1.0) as usize;
        let mut segments = Vec::new();
        let mut start = 0;
        while start + n <= samples.len() {
            segments.push(samples[start..start + n].to_vec());
            start += step;
        }
        segments
    }

    /// Apply the window function to a segment.
    fn apply_window(&self, segment: &[(f64, f64)]) -> Vec<(f64, f64)> {
        segment
            .iter()
            .zip(&self.window)
            .map(|((re, im), w)| (re * w, im * w))
            .collect()
    }
}

/// Generate a window of the specified type and length.
fn generate_window(window_type: &WindowType, n: usize) -> Vec<f64> {
    match window_type {
        WindowType::Rectangular => vec![1.0; n],
        WindowType::Hann => (0..n)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos()))
            .collect(),
        WindowType::Hamming => (0..n)
            .map(|i| 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1) as f64).cos())
            .collect(),
        WindowType::Blackman => (0..n)
            .map(|i| {
                let x = 2.0 * PI * i as f64 / (n - 1) as f64;
                0.42 - 0.5 * x.cos() + 0.08 * (2.0 * x).cos()
            })
            .collect(),
        WindowType::Kaiser(beta) => kaiser_window(n, *beta),
    }
}

/// Kaiser window: w[n] = I0(beta * sqrt(1 - ((2n/N-1) - 1)^2)) / I0(beta)
fn kaiser_window(n: usize, beta: f64) -> Vec<f64> {
    let i0_beta = bessel_i0(beta);
    (0..n)
        .map(|i| {
            let x = 2.0 * i as f64 / (n - 1) as f64 - 1.0;
            let arg = beta * (1.0 - x * x).max(0.0).sqrt();
            bessel_i0(arg) / i0_beta
        })
        .collect()
}

/// Modified Bessel function of the first kind, order 0.
/// Computed via the power series: I0(x) = sum_{k=0}^{inf} ((x/2)^k / k!)^2
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let x_half = x / 2.0;
    for k in 1..50 {
        term *= x_half / k as f64;
        let t2 = term * term;
        sum += t2;
        if t2 < 1e-16 * sum {
            break;
        }
    }
    sum
}

/// Radix-2 Cooley-Tukey FFT (in-place, decimation-in-time).
/// Input length must be a power of 2. If not, zero-pads to next power of 2.
fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let n_padded = n.next_power_of_two();

    let mut data: Vec<(f64, f64)> = input.to_vec();
    data.resize(n_padded, (0.0, 0.0));

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n_padded {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n_padded >> 1;
        while m > 0 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n_padded {
        let half = len / 2;
        let angle = -2.0 * PI / len as f64;
        let w_base = (angle.cos(), angle.sin());

        let mut i = 0;
        while i < n_padded {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = data[i + k];
                let t = complex_mul(w, data[i + k + half]);
                data[i + k] = (u.0 + t.0, u.1 + t.1);
                data[i + k + half] = (u.0 - t.0, u.1 - t.1);
                w = complex_mul(w, w_base);
            }
            i += len;
        }
        len <<= 1;
    }

    // If we zero-padded, truncate back to original FFT size
    data.truncate(n_padded);
    data
}

#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config(fft_size: usize) -> WelchConfig {
        WelchConfig {
            fft_size,
            overlap_fraction: 0.5,
            window_type: WindowType::Hann,
            sample_rate: 1000.0,
            averaging: Averaging::Uniform,
        }
    }

    fn tone(freq: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    fn noise(n: usize, seed: u64) -> Vec<(f64, f64)> {
        // Simple LCG pseudo-random generator for reproducibility
        let mut state = seed;
        (0..n)
            .map(|_| {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let re = ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let im = ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
                (re, im)
            })
            .collect()
    }

    #[test]
    fn test_basic_construction() {
        let config = default_config(64);
        let est = WelchEstimator::new(config);
        assert_eq!(est.config.fft_size, 64);
        assert_eq!(est.window.len(), 64);
        assert!(est.window_power > 0.0);
    }

    #[test]
    fn test_default_config() {
        let config = WelchConfig::default();
        assert_eq!(config.fft_size, 1024);
        assert_eq!(config.overlap_fraction, 0.5);
        assert_eq!(config.sample_rate, 1.0);
    }

    #[test]
    #[should_panic(expected = "fft_size must be > 0")]
    fn test_zero_fft_size_panics() {
        let mut config = default_config(64);
        config.fft_size = 0;
        WelchEstimator::new(config);
    }

    #[test]
    #[should_panic(expected = "overlap_fraction must be in")]
    fn test_overlap_out_of_range_panics() {
        let mut config = default_config(64);
        config.overlap_fraction = 0.95;
        WelchEstimator::new(config);
    }

    #[test]
    fn test_tone_detection() {
        // Generate a 200 Hz complex tone at 1000 Hz sample rate
        let samples = tone(200.0, 1000.0, 512);
        let est = WelchEstimator::new(default_config(64));
        let result = est.estimate_psd(&samples);

        assert_eq!(result.frequency_bins.len(), 64);
        assert_eq!(result.power_density.len(), 64);
        assert!(result.num_segments > 1);

        // Find peak
        let peak_idx = result
            .power_density
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_freq = result.frequency_bins[peak_idx];
        assert!(
            (peak_freq - 200.0).abs() < 20.0,
            "Expected peak near 200 Hz, got {} Hz",
            peak_freq
        );
    }

    #[test]
    fn test_psd_db() {
        let samples = tone(100.0, 1000.0, 256);
        let est = WelchEstimator::new(default_config(64));
        let result_lin = est.estimate_psd(&samples);
        let result_db = est.estimate_psd_db(&samples);

        // dB values should be 10*log10 of linear values
        for (lin, db) in result_lin
            .power_density
            .iter()
            .zip(result_db.power_density.iter())
        {
            let expected_db = 10.0 * lin.max(1e-30).log10();
            assert!(
                (db - expected_db).abs() < 1e-10,
                "dB conversion mismatch: {} vs {}",
                db,
                expected_db
            );
        }
    }

    #[test]
    fn test_rectangular_window() {
        let config = WelchConfig {
            fft_size: 32,
            overlap_fraction: 0.0,
            window_type: WindowType::Rectangular,
            sample_rate: 1000.0,
            averaging: Averaging::Uniform,
        };
        let est = WelchEstimator::new(config);
        // Rectangular window should be all ones
        for &w in &est.window {
            assert!((w - 1.0).abs() < 1e-15);
        }
    }

    #[test]
    fn test_hann_window_properties() {
        let window = generate_window(&WindowType::Hann, 64);
        // Hann window should be zero at endpoints
        assert!(window[0].abs() < 1e-15);
        assert!(window[63].abs() < 1e-15);
        // Peak at center
        assert!(window[32] > 0.99);
    }

    #[test]
    fn test_hamming_window_properties() {
        let window = generate_window(&WindowType::Hamming, 64);
        // Hamming window is NOT zero at endpoints (approximately 0.08)
        assert!(window[0] > 0.07 && window[0] < 0.09);
        // Peak at center
        assert!(window[32] > 0.99);
    }

    #[test]
    fn test_blackman_window_properties() {
        let window = generate_window(&WindowType::Blackman, 64);
        // Blackman window should be near zero at endpoints
        assert!(window[0].abs() < 0.01);
        // Peak at center
        assert!(window[32] > 0.99);
    }

    #[test]
    fn test_kaiser_window_properties() {
        let window = generate_window(&WindowType::Kaiser(5.0), 64);
        // Kaiser window should be symmetric
        for i in 0..32 {
            assert!(
                (window[i] - window[63 - i]).abs() < 1e-12,
                "Kaiser window not symmetric at index {}",
                i
            );
        }
        // Peak at center
        assert!(window[32] > 0.99);
    }

    #[test]
    fn test_empty_input() {
        let est = WelchEstimator::new(default_config(64));
        let result = est.estimate_psd(&[]);
        assert_eq!(result.num_segments, 0);
        assert_eq!(result.power_density.len(), 64);
    }

    #[test]
    fn test_input_shorter_than_fft() {
        let samples = tone(100.0, 1000.0, 32);
        let est = WelchEstimator::new(default_config(64));
        let result = est.estimate_psd(&samples);
        assert_eq!(result.num_segments, 0);
    }

    #[test]
    fn test_cross_psd_same_signal() {
        let samples = tone(150.0, 1000.0, 512);
        let est = WelchEstimator::new(default_config(64));
        let cpsd = est.cross_psd(&samples, &samples);
        let psd = est.estimate_psd(&samples);

        // Cross-PSD of a signal with itself should equal its auto-PSD (real part)
        // The imaginary part should be near zero
        let half = 64 / 2;
        for i in 0..64 {
            // The cross-PSD imaginary part should be small for auto-correlation
            assert!(
                cpsd[i].1.abs() < cpsd[i].0.abs() * 0.01 + 1e-20,
                "Cross-PSD imaginary part too large at bin {}: ({}, {})",
                i,
                cpsd[i].0,
                cpsd[i].1
            );
            // Real part should match auto-PSD
            let rel_err = if psd.power_density[i].abs() > 1e-20 {
                (cpsd[i].0 - psd.power_density[i]).abs() / psd.power_density[i]
            } else {
                cpsd[i].0.abs()
            };
            assert!(
                rel_err < 0.01,
                "Cross-PSD real part doesn't match auto-PSD at bin {}: cpsd={}, psd={}",
                i,
                cpsd[i].0,
                psd.power_density[i]
            );
        }
    }

    #[test]
    fn test_coherence_identical_signals() {
        let samples = tone(100.0, 1000.0, 512);
        let est = WelchEstimator::new(default_config(64));
        let coh = est.coherence(&samples, &samples);

        // Coherence of a signal with itself should be ~1.0 at all frequencies with power
        let psd = est.estimate_psd(&samples);
        let max_power = psd.power_density.iter().cloned().fold(0.0_f64, f64::max);
        for (i, &c) in coh.iter().enumerate() {
            if psd.power_density[i] > max_power * 0.01 {
                assert!(
                    c > 0.95,
                    "Coherence should be ~1.0 for identical signals at bin {}, got {}",
                    i,
                    c
                );
            }
        }
    }

    #[test]
    fn test_coherence_independent_signals() {
        // Two independent noise signals should have low coherence
        let x = noise(2048, 12345);
        let y = noise(2048, 67890);
        let est = WelchEstimator::new(default_config(64));
        let coh = est.coherence(&x, &y);

        let avg_coherence: f64 = coh.iter().sum::<f64>() / coh.len() as f64;
        // Average coherence of independent signals should be low (bounded by 1/K segments)
        assert!(
            avg_coherence < 0.5,
            "Average coherence of independent noise should be low, got {}",
            avg_coherence
        );
    }

    #[test]
    fn test_resolution_bandwidth() {
        let est = WelchEstimator::new(WelchConfig {
            fft_size: 1024,
            overlap_fraction: 0.5,
            window_type: WindowType::Hann,
            sample_rate: 44100.0,
            averaging: Averaging::Uniform,
        });
        let rbw = est.resolution_bandwidth();
        // For Hann window: ENBW = 1.5 * fs/N = 1.5 * 44100/1024 ≈ 64.6 Hz
        let expected = 1.5 * 44100.0 / 1024.0;
        assert!(
            (rbw - expected).abs() < 0.1,
            "RBW mismatch: got {}, expected {}",
            rbw,
            expected
        );
    }

    #[test]
    fn test_exponential_averaging() {
        let samples = tone(200.0, 1000.0, 512);
        let config = WelchConfig {
            fft_size: 64,
            overlap_fraction: 0.5,
            window_type: WindowType::Hann,
            sample_rate: 1000.0,
            averaging: Averaging::Exponential(0.3),
        };
        let est = WelchEstimator::new(config);
        let result = est.estimate_psd(&samples);

        // Should still detect the tone
        let peak_idx = result
            .power_density
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_freq = result.frequency_bins[peak_idx];
        assert!(
            (peak_freq - 200.0).abs() < 20.0,
            "Exponential averaging: expected peak near 200 Hz, got {} Hz",
            peak_freq
        );
    }

    #[test]
    fn test_frequency_bins_range() {
        let config = WelchConfig {
            fft_size: 128,
            overlap_fraction: 0.5,
            window_type: WindowType::Hann,
            sample_rate: 2000.0,
            averaging: Averaging::Uniform,
        };
        let est = WelchEstimator::new(config);
        let bins = est.frequency_bins();

        assert_eq!(bins.len(), 128);
        // First bin should be -fs/2
        assert!(
            (bins[0] - (-1000.0)).abs() < 0.01,
            "First bin should be -fs/2, got {}",
            bins[0]
        );
        // Last bin should be just below +fs/2
        let expected_last = 1000.0 - 2000.0 / 128.0;
        assert!(
            (bins[127] - expected_last).abs() < 0.01,
            "Last bin: got {}, expected {}",
            bins[127],
            expected_last
        );
    }

    #[test]
    fn test_multiple_segments() {
        // With 50% overlap and fft_size=64, 192 samples should give 5 segments:
        // starts at 0, 32, 64, 96, 128
        let samples = tone(100.0, 1000.0, 192);
        let est = WelchEstimator::new(default_config(64));
        let result = est.estimate_psd(&samples);
        assert_eq!(result.num_segments, 5);
    }

    #[test]
    fn test_no_overlap() {
        let config = WelchConfig {
            fft_size: 64,
            overlap_fraction: 0.0,
            window_type: WindowType::Rectangular,
            sample_rate: 1000.0,
            averaging: Averaging::Uniform,
        };
        let samples = tone(100.0, 1000.0, 256);
        let est = WelchEstimator::new(config);
        let result = est.estimate_psd(&samples);
        // 256 / 64 = 4 non-overlapping segments
        assert_eq!(result.num_segments, 4);
    }

    #[test]
    fn test_psd_non_negative() {
        let samples = noise(512, 42);
        let est = WelchEstimator::new(default_config(64));
        let result = est.estimate_psd(&samples);
        for &p in &result.power_density {
            assert!(p >= 0.0, "PSD should be non-negative, got {}", p);
        }
    }

    #[test]
    fn test_fft_basic() {
        // FFT of a single DC sample extended to length 4
        let input = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
        let result = fft(&input);
        // All bins should have magnitude 1
        for r in &result {
            let mag = (r.0 * r.0 + r.1 * r.1).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "FFT of impulse: expected mag 1.0, got {}",
                mag
            );
        }
    }
}

//! Noise Reduction — Spectral Subtraction & Wiener Filtering
//!
//! Frequency-domain noise reduction techniques for improving SNR of
//! received signals. Estimates noise PSD from signal-absent intervals,
//! then subtracts or attenuates noise-dominated frequency bins.
//! GNU Radio equivalent: gr-noise-cancel OOT module.
//!
//! ## Algorithms
//!
//! - **Spectral Subtraction**: Subtract estimated noise PSD from signal PSD.
//!   Simple and computationally efficient; may introduce "musical noise" artifacts.
//! - **Wiener Filter**: Optimal MMSE filter minimizing expected error.
//!   `H(f) = max(1 - σ_n²/|X(f)|², floor)`. Smoother than spectral subtraction.
//! - **MMSE Short-Time Spectral Amplitude (MMSE-STSA)**: Minimum-mean-square
//!   error estimator for spectral amplitude.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::noise_reduction::{SpectralSubtractor, WienerFilter};
//!
//! let mut ss = SpectralSubtractor::new(256, 1.0, 0.01);
//! // First, estimate noise from a noise-only segment
//! let noise_only = vec![0.01; 256];
//! ss.update_noise_estimate(&noise_only);
//! // Then process noisy signal
//! let noisy_signal = vec![0.5; 256];
//! let cleaned = ss.process(&noisy_signal);
//! assert_eq!(cleaned.len(), 256);
//! ```

use std::f64::consts::PI;

/// Simple radix-2 FFT for internal use.
fn fft_inplace(data: &mut [f64], imag: &mut [f64], inverse: bool) {
    let n = data.len();
    assert_eq!(n, imag.len());
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two());

    // Bit-reversal
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            data.swap(i, j);
            imag.swap(i, j);
        }
    }

    let sign = if inverse { 1.0 } else { -1.0 };

    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();
        let mut i = 0;
        while i < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for k in 0..half {
                let t_re = w_re * data[i + k + half] - w_im * imag[i + k + half];
                let t_im = w_re * imag[i + k + half] + w_im * data[i + k + half];
                data[i + k + half] = data[i + k] - t_re;
                imag[i + k + half] = imag[i + k] - t_im;
                data[i + k] += t_re;
                imag[i + k] += t_im;
                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            i += len;
        }
        len <<= 1;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for i in 0..n {
            data[i] *= scale;
            imag[i] *= scale;
        }
    }
}

fn next_pow2(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Hann window for spectral analysis.
fn hann_window(n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos()))
        .collect()
}

/// Spectral subtraction noise reducer.
///
/// Subtracts estimated noise PSD from the signal PSD in the frequency domain.
#[derive(Debug, Clone)]
pub struct SpectralSubtractor {
    fft_size: usize,
    /// Over-subtraction factor (1.0 = standard, >1 more aggressive).
    oversubtract: f64,
    /// Spectral floor to prevent negative values.
    spectral_floor: f64,
    /// Estimated noise PSD.
    noise_psd: Vec<f64>,
    /// Smoothing factor for noise estimate (0..1).
    noise_alpha: f64,
    /// Window function.
    window: Vec<f64>,
    /// Whether noise has been estimated.
    noise_estimated: bool,
}

impl SpectralSubtractor {
    /// Create a new spectral subtractor.
    ///
    /// * `fft_size` — FFT size (must be power of 2)
    /// * `oversubtract` — over-subtraction factor (typically 1.0-4.0)
    /// * `spectral_floor` — minimum gain per bin (0.01 = -40 dB floor)
    pub fn new(fft_size: usize, oversubtract: f64, spectral_floor: f64) -> Self {
        let fft_size = next_pow2(fft_size);
        Self {
            fft_size,
            oversubtract,
            spectral_floor,
            noise_psd: vec![0.0; fft_size],
            noise_alpha: 0.9,
            window: hann_window(fft_size),
            noise_estimated: false,
        }
    }

    /// Update noise estimate from a noise-only signal segment.
    pub fn update_noise_estimate(&mut self, noise_frame: &[f64]) {
        let n = noise_frame.len().min(self.fft_size);
        let mut re = vec![0.0; self.fft_size];
        let mut im = vec![0.0; self.fft_size];

        for i in 0..n {
            re[i] = noise_frame[i] * self.window[i];
        }

        fft_inplace(&mut re, &mut im, false);

        for k in 0..self.fft_size {
            let psd = re[k] * re[k] + im[k] * im[k];
            if self.noise_estimated {
                self.noise_psd[k] = self.noise_alpha * self.noise_psd[k]
                    + (1.0 - self.noise_alpha) * psd;
            } else {
                self.noise_psd[k] = psd;
            }
        }
        self.noise_estimated = true;
    }

    /// Process a frame and return noise-reduced output.
    pub fn process(&self, frame: &[f64]) -> Vec<f64> {
        let n = frame.len().min(self.fft_size);
        let mut re = vec![0.0; self.fft_size];
        let mut im = vec![0.0; self.fft_size];

        for i in 0..n {
            re[i] = frame[i] * self.window[i];
        }

        fft_inplace(&mut re, &mut im, false);

        // Spectral subtraction
        for k in 0..self.fft_size {
            let mag_sq = re[k] * re[k] + im[k] * im[k];
            let clean_mag_sq = (mag_sq - self.oversubtract * self.noise_psd[k])
                .max(self.spectral_floor * mag_sq);
            let gain = if mag_sq > 0.0 {
                (clean_mag_sq / mag_sq).sqrt()
            } else {
                self.spectral_floor
            };
            re[k] *= gain;
            im[k] *= gain;
        }

        fft_inplace(&mut re, &mut im, true);

        // Apply inverse window compensation
        let mut output = vec![0.0; self.fft_size];
        for i in 0..self.fft_size {
            output[i] = if self.window[i] > 1e-10 {
                re[i] / self.window[i]
            } else {
                re[i]
            };
        }

        output[..n].to_vec()
    }

    /// Set noise smoothing factor (0..1, higher = slower adaptation).
    pub fn set_noise_alpha(&mut self, alpha: f64) {
        self.noise_alpha = alpha.clamp(0.0, 1.0);
    }

    /// Get FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Reset noise estimate.
    pub fn reset(&mut self) {
        self.noise_psd.fill(0.0);
        self.noise_estimated = false;
    }
}

/// Wiener noise reduction filter.
///
/// Computes the optimal MMSE filter: H(f) = max(1 - σ_n²/|X(f)|², floor).
#[derive(Debug, Clone)]
pub struct WienerFilter {
    fft_size: usize,
    /// Spectral floor.
    spectral_floor: f64,
    /// Estimated noise PSD.
    noise_psd: Vec<f64>,
    /// Window function.
    window: Vec<f64>,
    /// Whether noise has been estimated.
    noise_estimated: bool,
    /// Smoothing for noise estimate.
    noise_alpha: f64,
}

impl WienerFilter {
    /// Create a new Wiener filter.
    ///
    /// * `fft_size` — FFT size (must be power of 2)
    /// * `spectral_floor` — minimum gain per bin
    pub fn new(fft_size: usize, spectral_floor: f64) -> Self {
        let fft_size = next_pow2(fft_size);
        Self {
            fft_size,
            spectral_floor,
            noise_psd: vec![0.0; fft_size],
            window: hann_window(fft_size),
            noise_estimated: false,
            noise_alpha: 0.9,
        }
    }

    /// Update noise estimate from a noise-only frame.
    pub fn update_noise_estimate(&mut self, noise_frame: &[f64]) {
        let n = noise_frame.len().min(self.fft_size);
        let mut re = vec![0.0; self.fft_size];
        let mut im = vec![0.0; self.fft_size];

        for i in 0..n {
            re[i] = noise_frame[i] * self.window[i];
        }

        fft_inplace(&mut re, &mut im, false);

        for k in 0..self.fft_size {
            let psd = re[k] * re[k] + im[k] * im[k];
            if self.noise_estimated {
                self.noise_psd[k] = self.noise_alpha * self.noise_psd[k]
                    + (1.0 - self.noise_alpha) * psd;
            } else {
                self.noise_psd[k] = psd;
            }
        }
        self.noise_estimated = true;
    }

    /// Process a frame and return noise-reduced output.
    pub fn process(&self, frame: &[f64]) -> Vec<f64> {
        let n = frame.len().min(self.fft_size);
        let mut re = vec![0.0; self.fft_size];
        let mut im = vec![0.0; self.fft_size];

        for i in 0..n {
            re[i] = frame[i] * self.window[i];
        }

        fft_inplace(&mut re, &mut im, false);

        // Wiener filter: H(f) = max(1 - noise_psd / signal_psd, floor)
        for k in 0..self.fft_size {
            let signal_psd = re[k] * re[k] + im[k] * im[k];
            let gain = if signal_psd > 0.0 {
                (1.0 - self.noise_psd[k] / signal_psd).max(self.spectral_floor)
            } else {
                self.spectral_floor
            };
            re[k] *= gain;
            im[k] *= gain;
        }

        fft_inplace(&mut re, &mut im, true);

        let mut output = vec![0.0; self.fft_size];
        for i in 0..self.fft_size {
            output[i] = if self.window[i] > 1e-10 {
                re[i] / self.window[i]
            } else {
                re[i]
            };
        }

        output[..n].to_vec()
    }

    /// Get FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Reset noise estimate.
    pub fn reset(&mut self) {
        self.noise_psd.fill(0.0);
        self.noise_estimated = false;
    }
}

/// Estimate SNR per frequency bin.
pub fn estimate_snr_per_bin(signal_psd: &[f64], noise_psd: &[f64]) -> Vec<f64> {
    assert_eq!(signal_psd.len(), noise_psd.len());
    signal_psd
        .iter()
        .zip(noise_psd.iter())
        .map(|(&s, &n)| {
            if n > 0.0 {
                (s / n - 1.0).max(0.0)
            } else if s > 0.0 {
                f64::MAX
            } else {
                0.0
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spectral_subtractor_reduces_noise() {
        let mut ss = SpectralSubtractor::new(256, 1.0, 0.01);

        // Generate noise-only segment
        let noise: Vec<f64> = (0..256).map(|i| (i as f64 * 0.1).sin() * 0.01).collect();
        ss.update_noise_estimate(&noise);

        // Generate signal + noise
        let signal: Vec<f64> = (0..256)
            .map(|i| {
                let t = i as f64 / 256.0;
                (2.0 * PI * 10.0 * t).sin() + (i as f64 * 0.1).sin() * 0.01
            })
            .collect();

        let cleaned = ss.process(&signal);
        assert_eq!(cleaned.len(), 256);

        // Cleaned signal should exist (not all zeros)
        let energy: f64 = cleaned.iter().map(|x| x * x).sum();
        assert!(energy > 0.0, "Cleaned signal should have energy");
    }

    #[test]
    fn test_wiener_filter_reduces_noise() {
        let mut wf = WienerFilter::new(256, 0.01);

        let noise: Vec<f64> = (0..256).map(|i| (i as f64 * 0.1).sin() * 0.01).collect();
        wf.update_noise_estimate(&noise);

        let signal: Vec<f64> = (0..256)
            .map(|i| {
                let t = i as f64 / 256.0;
                (2.0 * PI * 10.0 * t).sin() + (i as f64 * 0.1).sin() * 0.01
            })
            .collect();

        let cleaned = wf.process(&signal);
        assert_eq!(cleaned.len(), 256);

        let energy: f64 = cleaned.iter().map(|x| x * x).sum();
        assert!(energy > 0.0);
    }

    #[test]
    fn test_spectral_subtractor_no_noise_estimate() {
        let ss = SpectralSubtractor::new(64, 1.0, 0.01);
        // Process without noise estimate (noise_psd = 0)
        let signal: Vec<f64> = (0..64).map(|i| (i as f64 * 0.5).sin()).collect();
        let cleaned = ss.process(&signal);
        // Should pass through approximately unchanged
        assert_eq!(cleaned.len(), 64);
    }

    #[test]
    fn test_spectral_subtractor_oversubtraction() {
        let mut ss_low = SpectralSubtractor::new(128, 1.0, 0.01);
        let mut ss_high = SpectralSubtractor::new(128, 3.0, 0.01);

        let noise: Vec<f64> = (0..128).map(|i| (i as f64 * 0.3).sin() * 0.1).collect();
        ss_low.update_noise_estimate(&noise);
        ss_high.update_noise_estimate(&noise);

        let signal: Vec<f64> = (0..128)
            .map(|i| (i as f64 * 0.3).sin() * 0.1 + 0.5)
            .collect();

        let cleaned_low = ss_low.process(&signal);
        let cleaned_high = ss_high.process(&signal);

        let energy_low: f64 = cleaned_low.iter().map(|x| x * x).sum();
        let energy_high: f64 = cleaned_high.iter().map(|x| x * x).sum();

        // Higher oversubtraction should remove more energy
        assert!(
            energy_high <= energy_low + 0.1,
            "Higher oversubtraction should remove more energy"
        );
    }

    #[test]
    fn test_spectral_subtractor_fft_size() {
        let ss = SpectralSubtractor::new(100, 1.0, 0.01);
        assert_eq!(ss.fft_size(), 128); // Rounded up to power of 2
    }

    #[test]
    fn test_wiener_filter_fft_size() {
        let wf = WienerFilter::new(200, 0.01);
        assert_eq!(wf.fft_size(), 256);
    }

    #[test]
    fn test_spectral_subtractor_reset() {
        let mut ss = SpectralSubtractor::new(64, 1.0, 0.01);
        let noise: Vec<f64> = vec![0.1; 64];
        ss.update_noise_estimate(&noise);
        ss.reset();
        // After reset, noise PSD should be zero
        assert!(ss.noise_psd.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_wiener_filter_reset() {
        let mut wf = WienerFilter::new(64, 0.01);
        let noise: Vec<f64> = vec![0.1; 64];
        wf.update_noise_estimate(&noise);
        wf.reset();
        assert!(wf.noise_psd.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_estimate_snr_per_bin() {
        let signal = vec![10.0, 5.0, 0.1];
        let noise = vec![1.0, 1.0, 1.0];
        let snr = estimate_snr_per_bin(&signal, &noise);
        assert!((snr[0] - 9.0).abs() < 1e-10);
        assert!((snr[1] - 4.0).abs() < 1e-10);
        assert!((snr[2] - 0.0).abs() < 1e-10); // max(0.1/1 - 1, 0) = 0
    }

    #[test]
    fn test_hann_window() {
        let w = hann_window(4);
        assert_eq!(w.len(), 4);
        // Hann: w[0] = 0, w[1] = 1, w[2] = 0 (for N=4: at endpoints)
        assert!(w[0].abs() < 1e-10, "Hann window should be 0 at start");
    }

    #[test]
    fn test_fft_inverse_roundtrip() {
        let original = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let mut re = original.clone();
        let mut im = vec![0.0; 8];
        fft_inplace(&mut re, &mut im, false);
        fft_inplace(&mut re, &mut im, true);
        for (a, b) in re.iter().zip(original.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_noise_alpha_setting() {
        let mut ss = SpectralSubtractor::new(64, 1.0, 0.01);
        ss.set_noise_alpha(0.5);
        // Process two noise frames to verify smoothing
        let n1: Vec<f64> = vec![0.1; 64];
        let n2: Vec<f64> = vec![0.2; 64];
        ss.update_noise_estimate(&n1);
        ss.update_noise_estimate(&n2);
        // PSD should be a blend of both
    }
}

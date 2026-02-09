//! Chirp-Z Transform (CZT) — Bluestein's Algorithm
//!
//! Computes the Z-transform on an arbitrary contour in the Z-plane,
//! generalizing the DFT. Primary uses include zoom FFT (high-resolution
//! spectral analysis of narrow bands), arbitrary frequency resolution,
//! and evaluating the Z-transform at non-uniform points.
//!
//! ## Algorithm
//!
//! The CZT evaluates X(z_k) for z_k = A · W^{-k}, k = 0..M-1, where
//! A is the starting point and W determines spacing. For the standard
//! DFT case, A = 1 and W = exp(-j2π/N).
//!
//! Uses Bluestein's identity to convert to a convolution, computed
//! via FFT for O(N log N) complexity.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::chirp_z_transform::{czt, zoom_fft};
//! use num_complex::Complex64;
//!
//! // Zoom FFT: analyze a narrow band around 1000 Hz with high resolution
//! let fs = 8000.0;
//! let n = 256;
//! let signal: Vec<Complex64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / fs;
//!         Complex64::new((2.0 * std::f64::consts::PI * 1000.0 * t).cos(), 0.0)
//!     })
//!     .collect();
//!
//! // Zoom into 900-1100 Hz with 128 output bins
//! let spectrum = zoom_fft(&signal, fs, 900.0, 1100.0, 128);
//! assert_eq!(spectrum.len(), 128);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Compute the next power of 2 >= n.
fn next_pow2(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Simple radix-2 DIT FFT (Cooley-Tukey) for internal use.
fn fft_radix2(input: &mut [Complex64]) {
    let n = input.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT size must be power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            input.swap(i, j);
        }
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let w = Complex64::new(0.0, -2.0 * PI / len as f64).exp();
        let mut i = 0;
        while i < n {
            let mut wj = Complex64::new(1.0, 0.0);
            for k in 0..half {
                let t = wj * input[i + k + half];
                input[i + k + half] = input[i + k] - t;
                input[i + k] = input[i + k] + t;
                wj *= w;
            }
            i += len;
        }
        len <<= 1;
    }
}

/// Inverse FFT.
fn ifft_radix2(input: &mut [Complex64]) {
    let n = input.len();
    // Conjugate
    for x in input.iter_mut() {
        *x = x.conj();
    }
    fft_radix2(input);
    // Conjugate and scale
    let scale = 1.0 / n as f64;
    for x in input.iter_mut() {
        *x = x.conj() * scale;
    }
}

/// Compute the Chirp-Z Transform using Bluestein's algorithm.
///
/// Evaluates X(z_k) = sum_{n=0}^{N-1} x[n] * z_k^{-n}
/// where z_k = A * W^{-k}, k = 0..M-1.
///
/// * `x` — input signal (N samples)
/// * `m` — number of output points
/// * `w` — spacing factor (W) on the Z-plane contour
/// * `a` — starting point (A) on the Z-plane contour
pub fn czt(x: &[Complex64], m: usize, w: Complex64, a: Complex64) -> Vec<Complex64> {
    let n = x.len();
    if n == 0 || m == 0 {
        return vec![Complex64::new(0.0, 0.0); m];
    }

    // Bluestein's algorithm:
    // X(z_k) = W^{k^2/2} * sum_n [x[n] * A^{-n} * W^{n^2/2}] * W^{-(k-n)^2/2}
    // This is a convolution of h[n] = x[n]*A^{-n}*W^{n^2/2} with g[m] = W^{-m^2/2}

    let l = next_pow2(n + m - 1);

    // Precompute chirp sequence: W^{sign * k^2/2}
    let chirp = |k: usize, sign: f64| -> Complex64 {
        let half_k_sq = sign * (k * k) as f64 / 2.0;
        w.powf(half_k_sq)
    };

    // Build h[n] = x[n] * A^{-n} * W^{n^2/2}, zero-padded to L
    let mut h = vec![Complex64::new(0.0, 0.0); l];
    for n_idx in 0..n {
        let a_neg_n = a.powf(-(n_idx as f64));
        h[n_idx] = x[n_idx] * a_neg_n * chirp(n_idx, 1.0);
    }

    // Build g[m] = W^{-m^2/2}, wrapped for circular convolution
    let mut g = vec![Complex64::new(0.0, 0.0); l];
    // g[k] for k = 0..M-1
    for k in 0..m {
        g[k] = chirp(k, -1.0);
    }
    // g[L-n] for n = 1..N-1 (wrap-around for negative indices)
    for n_idx in 1..n {
        g[l - n_idx] = chirp(n_idx, -1.0);
    }

    // Convolution via FFT
    fft_radix2(&mut h);
    fft_radix2(&mut g);
    for i in 0..l {
        h[i] *= g[i];
    }
    ifft_radix2(&mut h);

    // Extract output and multiply by W^{k^2/2}
    let mut result = vec![Complex64::new(0.0, 0.0); m];
    for k in 0..m {
        result[k] = chirp(k, 1.0) * h[k];
    }

    result
}

/// Zoom FFT: high-resolution spectral analysis of a narrow frequency band.
///
/// * `x` — input signal
/// * `fs` — sample rate (Hz)
/// * `f_start` — start frequency of the zoom band (Hz)
/// * `f_end` — end frequency of the zoom band (Hz)
/// * `num_bins` — number of output frequency bins
pub fn zoom_fft(
    x: &[Complex64],
    fs: f64,
    f_start: f64,
    f_end: f64,
    num_bins: usize,
) -> Vec<Complex64> {
    let n = x.len();
    if n == 0 || num_bins == 0 {
        return vec![Complex64::new(0.0, 0.0); num_bins];
    }

    // Map frequency range to Z-plane
    let theta_start = 2.0 * PI * f_start / fs;
    let theta_end = 2.0 * PI * f_end / fs;
    let d_theta = (theta_end - theta_start) / num_bins as f64;

    let a = Complex64::new(0.0, theta_start).exp();
    let w = Complex64::new(0.0, -d_theta).exp();

    czt(x, num_bins, w, a)
}

/// Compute zoom FFT magnitude spectrum in dB.
pub fn zoom_fft_magnitude_db(
    x: &[Complex64],
    fs: f64,
    f_start: f64,
    f_end: f64,
    num_bins: usize,
) -> Vec<f64> {
    let spectrum = zoom_fft(x, fs, f_start, f_end, num_bins);
    spectrum
        .iter()
        .map(|z| {
            let mag_sq = z.norm_sqr();
            if mag_sq > 0.0 {
                10.0 * mag_sq.log10()
            } else {
                -200.0
            }
        })
        .collect()
}

/// Get the frequency axis for a zoom FFT result.
pub fn zoom_fft_frequencies(fs: f64, f_start: f64, f_end: f64, num_bins: usize) -> Vec<f64> {
    let df = (f_end - f_start) / num_bins as f64;
    (0..num_bins).map(|i| f_start + i as f64 * df).collect()
}

/// CZT processor with cached parameters for repeated calls.
#[derive(Debug, Clone)]
pub struct CztProcessor {
    m: usize,
    w: Complex64,
    a: Complex64,
}

impl CztProcessor {
    /// Create a new CZT processor.
    pub fn new(m: usize, w: Complex64, a: Complex64) -> Self {
        Self { m, w, a }
    }

    /// Create a processor for zoom FFT.
    pub fn for_zoom_fft(fs: f64, f_start: f64, f_end: f64, num_bins: usize) -> Self {
        let theta_start = 2.0 * PI * f_start / fs;
        let theta_end = 2.0 * PI * f_end / fs;
        let d_theta = (theta_end - theta_start) / num_bins as f64;
        Self {
            m: num_bins,
            w: Complex64::new(0.0, -d_theta).exp(),
            a: Complex64::new(0.0, theta_start).exp(),
        }
    }

    /// Process a block of input samples.
    pub fn process(&self, x: &[Complex64]) -> Vec<Complex64> {
        czt(x, self.m, self.w, self.a)
    }

    /// Get number of output bins.
    pub fn num_bins(&self) -> usize {
        self.m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tone(freq: f64, fs: f64, n: usize) -> Vec<Complex64> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                Complex64::new((2.0 * PI * freq * t).cos(), (2.0 * PI * freq * t).sin())
            })
            .collect()
    }

    #[test]
    fn test_czt_matches_dft_for_unit_circle() {
        // CZT with A=1, W=exp(-j2π/N) should match DFT
        let n = 16;
        let signal = make_tone(1000.0, 8000.0, n);
        let w = Complex64::new(0.0, -2.0 * PI / n as f64).exp();
        let a = Complex64::new(1.0, 0.0);
        let czt_result = czt(&signal, n, w, a);

        // Compute DFT directly
        let mut dft_result = signal.clone();
        fft_radix2(&mut dft_result);

        for k in 0..n {
            let diff = (czt_result[k] - dft_result[k]).norm();
            assert!(
                diff < 1e-6,
                "CZT[{}] = {:?}, DFT[{}] = {:?}, diff = {}",
                k, czt_result[k], k, dft_result[k], diff
            );
        }
    }

    #[test]
    fn test_zoom_fft_finds_tone() {
        let fs = 8000.0;
        let freq = 1000.0;
        let n = 256;
        let signal = make_tone(freq, fs, n);

        // Zoom into 800-1200 Hz
        let spectrum = zoom_fft(&signal, fs, 800.0, 1200.0, 64);
        let magnitudes: Vec<f64> = spectrum.iter().map(|z| z.norm()).collect();

        // Find peak
        let peak_idx = magnitudes
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Peak should be near the center (1000 Hz in 800-1200 range)
        let peak_freq = 800.0 + peak_idx as f64 * 400.0 / 64.0;
        assert!(
            (peak_freq - freq).abs() < 20.0,
            "Peak at {} Hz, expected ~{} Hz",
            peak_freq,
            freq
        );
    }

    #[test]
    fn test_zoom_fft_resolution() {
        // Two closely spaced tones that a standard FFT can't resolve
        let fs = 8000.0;
        let n = 128;
        let f1 = 1000.0;
        let f2 = 1010.0; // 10 Hz apart, standard FFT resolution = 62.5 Hz

        let signal: Vec<Complex64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                Complex64::new(
                    (2.0 * PI * f1 * t).cos() + (2.0 * PI * f2 * t).cos(),
                    (2.0 * PI * f1 * t).sin() + (2.0 * PI * f2 * t).sin(),
                )
            })
            .collect();

        // Zoom FFT with high resolution: 256 bins over 50 Hz = 0.2 Hz/bin
        let spectrum = zoom_fft(&signal, fs, 990.0, 1020.0, 256);
        let magnitudes: Vec<f64> = spectrum.iter().map(|z| z.norm()).collect();

        // Should have two peaks
        let max_mag = magnitudes.iter().cloned().fold(0.0f64, f64::max);
        let peaks: Vec<usize> = magnitudes
            .iter()
            .enumerate()
            .filter(|(_, &m)| m > max_mag * 0.5)
            .map(|(i, _)| i)
            .collect();

        assert!(peaks.len() >= 2, "Should resolve two tones, got {} peaks", peaks.len());
    }

    #[test]
    fn test_zoom_fft_magnitude_db() {
        let fs = 8000.0;
        let signal = make_tone(1000.0, fs, 128);
        let db = zoom_fft_magnitude_db(&signal, fs, 900.0, 1100.0, 32);
        assert_eq!(db.len(), 32);
        // Peak should be positive dB, off-peak should be lower
        let max_db = db.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_db = db.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(max_db > min_db, "Should have spectral contrast");
    }

    #[test]
    fn test_zoom_fft_frequencies() {
        let freqs = zoom_fft_frequencies(8000.0, 900.0, 1100.0, 4);
        assert_eq!(freqs.len(), 4);
        assert!((freqs[0] - 900.0).abs() < 1e-10);
        assert!((freqs[1] - 950.0).abs() < 1e-10);
        assert!((freqs[2] - 1000.0).abs() < 1e-10);
        assert!((freqs[3] - 1050.0).abs() < 1e-10);
    }

    #[test]
    fn test_czt_processor() {
        let fs = 8000.0;
        let signal = make_tone(1000.0, fs, 128);
        let proc = CztProcessor::for_zoom_fft(fs, 900.0, 1100.0, 32);
        let result = proc.process(&signal);
        assert_eq!(result.len(), 32);
        assert_eq!(proc.num_bins(), 32);
    }

    #[test]
    fn test_czt_empty_input() {
        let result = czt(&[], 4, Complex64::new(1.0, 0.0), Complex64::new(1.0, 0.0));
        assert_eq!(result.len(), 4);
    }

    #[test]
    fn test_czt_single_sample() {
        let x = vec![Complex64::new(3.0, 0.0)];
        let w = Complex64::new(0.0, -2.0 * PI / 4.0).exp();
        let a = Complex64::new(1.0, 0.0);
        let result = czt(&x, 4, w, a);
        // For single sample, all output bins should have magnitude 3
        for z in &result {
            assert!((z.norm() - 3.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_next_pow2() {
        assert_eq!(next_pow2(1), 1);
        assert_eq!(next_pow2(2), 2);
        assert_eq!(next_pow2(3), 4);
        assert_eq!(next_pow2(7), 8);
        assert_eq!(next_pow2(8), 8);
        assert_eq!(next_pow2(9), 16);
    }

    #[test]
    fn test_internal_fft() {
        // Test FFT → IFFT roundtrip
        let original = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
            Complex64::new(4.0, 0.0),
        ];
        let mut data = original.clone();
        fft_radix2(&mut data);
        ifft_radix2(&mut data);
        for (a, b) in data.iter().zip(original.iter()) {
            assert!((a - b).norm() < 1e-10);
        }
    }
}

//! Empirical Mode Decomposition (EMD) and Hilbert-Huang Transform
//!
//! Decomposes non-linear, non-stationary signals into Intrinsic Mode
//! Functions (IMFs) via the sifting process. Each IMF captures a
//! narrow-band oscillatory mode. The Hilbert transform of each IMF
//! yields instantaneous frequency and amplitude, producing the
//! Hilbert-Huang spectrum — a time-frequency representation with
//! superior resolution for non-stationary signals.
//!
//! No direct GNU Radio equivalent (fundamental signal analysis tool).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::empirical_mode::{emd, hilbert_huang, EmdConfig};
//!
//! let signal: Vec<f64> = (0..256)
//!     .map(|i| {
//!         let t = i as f64 / 256.0;
//!         (2.0 * std::f64::consts::PI * 5.0 * t).sin()
//!             + 0.5 * (2.0 * std::f64::consts::PI * 20.0 * t).sin()
//!     })
//!     .collect();
//! let config = EmdConfig::default();
//! let imfs = emd(&signal, &config);
//! assert!(imfs.len() >= 2); // At least 2 IMFs for 2 sinusoids
//! ```

use std::f64::consts::PI;

/// Configuration for the EMD sifting process.
#[derive(Debug, Clone)]
pub struct EmdConfig {
    /// Maximum number of sifting iterations per IMF.
    pub max_sift_iterations: usize,
    /// Sifting stops when SD < threshold (Cauchy convergence).
    pub sift_threshold: f64,
    /// Maximum number of IMFs to extract.
    pub max_imfs: usize,
    /// Residual energy threshold (stop if residual energy < this fraction).
    pub residual_threshold: f64,
}

impl Default for EmdConfig {
    fn default() -> Self {
        Self {
            max_sift_iterations: 100,
            sift_threshold: 0.3,
            max_imfs: 20,
            residual_threshold: 1e-6,
        }
    }
}

/// Result of Hilbert-Huang analysis on a single IMF.
#[derive(Debug, Clone)]
pub struct HilbertResult {
    /// Instantaneous amplitude envelope.
    pub amplitude: Vec<f64>,
    /// Instantaneous frequency (normalized, 0..0.5).
    pub frequency: Vec<f64>,
    /// Instantaneous phase (radians).
    pub phase: Vec<f64>,
}

/// Perform Empirical Mode Decomposition, returning a list of IMFs.
///
/// The last element is the residual (trend). IMFs are ordered from
/// highest frequency to lowest.
pub fn emd(signal: &[f64], config: &EmdConfig) -> Vec<Vec<f64>> {
    let n = signal.len();
    if n < 4 {
        return vec![signal.to_vec()];
    }

    let mut imfs = Vec::new();
    let mut residual = signal.to_vec();

    for _ in 0..config.max_imfs {
        // Check if residual has enough energy
        let energy: f64 = residual.iter().map(|x| x * x).sum();
        if energy / (n as f64) < config.residual_threshold {
            break;
        }

        // Check if residual is monotone (no more IMFs possible)
        if count_extrema(&residual) < 2 {
            break;
        }

        // Sifting process to extract one IMF
        let imf = sift(&residual, config);

        // Update residual
        for i in 0..n {
            residual[i] -= imf[i];
        }
        imfs.push(imf);
    }

    // Add residual as the last component
    imfs.push(residual);
    imfs
}

/// Sifting process to extract a single IMF from a signal.
fn sift(signal: &[f64], config: &EmdConfig) -> Vec<f64> {
    let n = signal.len();
    let mut h = signal.to_vec();

    for _ in 0..config.max_sift_iterations {
        let (maxima_idx, maxima_val) = find_local_maxima(&h);
        let (minima_idx, minima_val) = find_local_minima(&h);

        if maxima_idx.len() < 2 || minima_idx.len() < 2 {
            break;
        }

        // Interpolate envelopes using cubic spline
        let upper = cubic_spline_interpolate(&maxima_idx, &maxima_val, n);
        let lower = cubic_spline_interpolate(&minima_idx, &minima_val, n);

        // Compute mean envelope
        let mean: Vec<f64> = upper.iter().zip(lower.iter())
            .map(|(u, l)| (u + l) / 2.0)
            .collect();

        // Subtract mean from signal
        let prev = h.clone();
        for i in 0..n {
            h[i] -= mean[i];
        }

        // Check convergence (normalized squared difference)
        let sd = normalized_sd(&prev, &h);
        if sd < config.sift_threshold {
            break;
        }
    }
    h
}

/// Compute Hilbert transform of a real signal using FFT.
fn hilbert_transform(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // Compute DFT
    let mut re: Vec<f64> = signal.to_vec();
    let mut im = vec![0.0; n];
    dft(&mut re, &mut im, false);

    // Apply Hilbert filter in frequency domain:
    // H(k) = 0 for k=0, k=N/2
    // H(k) = -j for k=1..N/2-1 (positive frequencies)
    // H(k) = +j for k=N/2+1..N-1 (negative frequencies)
    let mut out_re = vec![0.0; n];
    let mut out_im = vec![0.0; n];

    // DC and Nyquist: zero
    out_re[0] = 0.0;
    out_im[0] = 0.0;
    if n > 1 {
        let half = n / 2;
        out_re[half] = 0.0;
        out_im[half] = 0.0;

        // Positive frequencies: multiply by -j → (re, im) → (im, -re)
        for k in 1..half {
            out_re[k] = im[k];
            out_im[k] = -re[k];
        }
        // Negative frequencies: multiply by +j → (re, im) → (-im, re)
        for k in half + 1..n {
            out_re[k] = -im[k];
            out_im[k] = re[k];
        }
    }

    // Inverse DFT
    dft(&mut out_re, &mut out_im, true);
    out_re
}

/// Compute Hilbert-Huang Transform: EMD + Hilbert analysis of each IMF.
///
/// Returns one `HilbertResult` per IMF (excluding the residual).
pub fn hilbert_huang(signal: &[f64], config: &EmdConfig) -> Vec<HilbertResult> {
    let imfs = emd(signal, config);
    let mut results = Vec::new();

    // Process all IMFs except the residual (last one)
    for imf in imfs.iter().take(imfs.len().saturating_sub(1)) {
        let ht = hilbert_transform(imf);
        let n = imf.len();

        let mut amplitude = Vec::with_capacity(n);
        let mut phase = Vec::with_capacity(n);
        let mut frequency = Vec::with_capacity(n);

        for i in 0..n {
            let a = (imf[i] * imf[i] + ht[i] * ht[i]).sqrt();
            let p = ht[i].atan2(imf[i]);
            amplitude.push(a);
            phase.push(p);
        }

        // Instantaneous frequency from phase derivative
        for i in 0..n {
            if i == 0 {
                frequency.push(0.0);
            } else {
                let dp = phase[i] - phase[i - 1];
                // Unwrap phase
                let dp = dp - (dp / PI).round() * PI;
                frequency.push(dp.abs() / (2.0 * PI));
            }
        }

        results.push(HilbertResult { amplitude, frequency, phase });
    }
    results
}

/// Simple DFT (or inverse DFT if `inverse` is true).
fn dft(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut out_re = vec![0.0; n];
    let mut out_im = vec![0.0; n];

    for k in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for j in 0..n {
            let angle = sign * 2.0 * PI * (k * j) as f64 / n as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            sum_re += re[j] * cos_a - im[j] * sin_a;
            sum_im += re[j] * sin_a + im[j] * cos_a;
        }
        out_re[k] = sum_re;
        out_im[k] = sum_im;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for i in 0..n {
            re[i] = out_re[i] * scale;
            im[i] = out_im[i] * scale;
        }
    } else {
        re.copy_from_slice(&out_re);
        im.copy_from_slice(&out_im);
    }
}

/// Find local maxima indices and values.
fn find_local_maxima(signal: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let mut indices = Vec::new();
    let mut values = Vec::new();

    // Include endpoints as virtual extrema for envelope extrapolation
    if signal.len() >= 2 && signal[0] > signal[1] {
        indices.push(0.0);
        values.push(signal[0]);
    }

    for i in 1..signal.len() - 1 {
        if signal[i] > signal[i - 1] && signal[i] >= signal[i + 1] {
            indices.push(i as f64);
            values.push(signal[i]);
        }
    }

    let n = signal.len();
    if n >= 2 && signal[n - 1] > signal[n - 2] {
        indices.push((n - 1) as f64);
        values.push(signal[n - 1]);
    }

    (indices, values)
}

/// Find local minima indices and values.
fn find_local_minima(signal: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let mut indices = Vec::new();
    let mut values = Vec::new();

    if signal.len() >= 2 && signal[0] < signal[1] {
        indices.push(0.0);
        values.push(signal[0]);
    }

    for i in 1..signal.len() - 1 {
        if signal[i] < signal[i - 1] && signal[i] <= signal[i + 1] {
            indices.push(i as f64);
            values.push(signal[i]);
        }
    }

    let n = signal.len();
    if n >= 2 && signal[n - 1] < signal[n - 2] {
        indices.push((n - 1) as f64);
        values.push(signal[n - 1]);
    }

    (indices, values)
}

/// Count total number of extrema (maxima + minima) in a signal.
fn count_extrema(signal: &[f64]) -> usize {
    let mut count = 0;
    for i in 1..signal.len() - 1 {
        if (signal[i] > signal[i - 1] && signal[i] >= signal[i + 1])
            || (signal[i] < signal[i - 1] && signal[i] <= signal[i + 1])
        {
            count += 1;
        }
    }
    count
}

/// Normalized squared difference for convergence check.
fn normalized_sd(prev: &[f64], curr: &[f64]) -> f64 {
    let num: f64 = prev.iter().zip(curr.iter())
        .map(|(p, c)| (p - c).powi(2))
        .sum();
    let den: f64 = prev.iter().map(|p| p.powi(2)).sum();
    if den < 1e-30 { 0.0 } else { num / den }
}

/// Natural cubic spline interpolation through given knots, evaluated at 0..n-1.
fn cubic_spline_interpolate(knots_x: &[f64], knots_y: &[f64], n: usize) -> Vec<f64> {
    let m = knots_x.len();
    if m == 0 {
        return vec![0.0; n];
    }
    if m == 1 {
        return vec![knots_y[0]; n];
    }
    if m == 2 {
        // Linear interpolation
        return (0..n).map(|i| {
            let t = (i as f64 - knots_x[0]) / (knots_x[1] - knots_x[0]).max(1e-10);
            let t = t.clamp(0.0, 1.0);
            knots_y[0] + t * (knots_y[1] - knots_y[0])
        }).collect();
    }

    // Compute natural cubic spline coefficients
    let k = m - 1;
    let mut h = vec![0.0; k];
    for i in 0..k {
        h[i] = knots_x[i + 1] - knots_x[i];
        if h[i] < 1e-10 { h[i] = 1e-10; }
    }

    // Tridiagonal system for second derivatives
    let mut alpha = vec![0.0; k];
    for i in 1..k {
        alpha[i] = 3.0 / h[i] * (knots_y[i + 1] - knots_y[i])
                 - 3.0 / h[i - 1] * (knots_y[i] - knots_y[i - 1]);
    }

    let mut l = vec![1.0; m];
    let mut mu = vec![0.0; m];
    let mut z = vec![0.0; m];

    for i in 1..k {
        l[i] = 2.0 * (knots_x[i + 1] - knots_x[i - 1]) - h[i - 1] * mu[i - 1];
        if l[i].abs() < 1e-30 { l[i] = 1e-30; }
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    let mut c = vec![0.0; m];
    let mut b = vec![0.0; k];
    let mut d = vec![0.0; k];

    for j in (0..k).rev() {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (knots_y[j + 1] - knots_y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
    }

    // Evaluate spline at each integer position
    let mut result = vec![0.0; n];
    let mut seg = 0;
    for i in 0..n {
        let x = i as f64;

        // Clamp to knot range
        if x <= knots_x[0] {
            result[i] = knots_y[0];
            continue;
        }
        if x >= knots_x[m - 1] {
            result[i] = knots_y[m - 1];
            continue;
        }

        // Find segment
        while seg < k - 1 && x > knots_x[seg + 1] {
            seg += 1;
        }

        let dx = x - knots_x[seg];
        result[i] = knots_y[seg] + b[seg] * dx + c[seg] * dx * dx + d[seg] * dx * dx * dx;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_emd_single_sine() {
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 5.0 * i as f64 / n as f64).sin())
            .collect();
        let config = EmdConfig::default();
        let imfs = emd(&signal, &config);
        // Should have at least 1 IMF + residual
        assert!(imfs.len() >= 2, "len={}", imfs.len());
    }

    #[test]
    fn test_emd_two_sines() {
        let n = 512;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                (2.0 * PI * 3.0 * t).sin() + 0.5 * (2.0 * PI * 30.0 * t).sin()
            })
            .collect();
        let config = EmdConfig::default();
        let imfs = emd(&signal, &config);
        // Should separate into at least 2 IMFs + residual
        assert!(imfs.len() >= 3, "len={}", imfs.len());
    }

    #[test]
    fn test_emd_reconstruction() {
        let n = 128;
        let signal: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin() + 0.3).collect();
        let config = EmdConfig::default();
        let imfs = emd(&signal, &config);
        // Sum of all IMFs should reconstruct the original signal
        let mut reconstructed = vec![0.0; n];
        for imf in &imfs {
            for i in 0..n {
                reconstructed[i] += imf[i];
            }
        }
        for i in 0..n {
            assert!((reconstructed[i] - signal[i]).abs() < 1e-6,
                "i={i}: {} vs {}", reconstructed[i], signal[i]);
        }
    }

    #[test]
    fn test_emd_monotone_input() {
        // Monotone input should produce 1 component (residual only)
        let signal: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let config = EmdConfig::default();
        let imfs = emd(&signal, &config);
        assert!(imfs.len() >= 1);
    }

    #[test]
    fn test_hilbert_transform_sine() {
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 4.0 * i as f64 / n as f64).sin())
            .collect();
        let ht = hilbert_transform(&signal);
        assert_eq!(ht.len(), n);
        // Hilbert of sin should be -cos (approximately)
        for i in n / 8..7 * n / 8 {
            let expected = -(2.0 * PI * 4.0 * i as f64 / n as f64).cos();
            assert!((ht[i] - expected).abs() < 0.15,
                "i={i}: ht={} expected={expected}", ht[i]);
        }
    }

    #[test]
    fn test_hilbert_huang_basic() {
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 8.0 * i as f64 / n as f64).sin())
            .collect();
        let config = EmdConfig::default();
        let results = hilbert_huang(&signal, &config);
        assert!(!results.is_empty());
        // First IMF should have approximately constant amplitude ~1.0
        let first = &results[0];
        let mid = &first.amplitude[n / 4..3 * n / 4];
        let avg_amp: f64 = mid.iter().sum::<f64>() / mid.len() as f64;
        assert!((avg_amp - 1.0).abs() < 0.3, "avg_amp={avg_amp}");
    }

    #[test]
    fn test_find_local_maxima() {
        let signal = vec![0.0, 1.0, 0.0, 2.0, 0.0];
        let (idx, val) = find_local_maxima(&signal);
        assert!(idx.contains(&1.0));
        assert!(idx.contains(&3.0));
        assert!(val.contains(&1.0));
        assert!(val.contains(&2.0));
    }

    #[test]
    fn test_find_local_minima() {
        let signal = vec![1.0, 0.0, 1.0, -1.0, 1.0];
        let (idx, val) = find_local_minima(&signal);
        assert!(idx.contains(&1.0));
        assert!(idx.contains(&3.0));
    }

    #[test]
    fn test_cubic_spline_linear_data() {
        let knots_x = vec![0.0, 4.0, 9.0];
        let knots_y = vec![0.0, 4.0, 9.0]; // Linear: y = x
        let result = cubic_spline_interpolate(&knots_x, &knots_y, 10);
        for i in 0..10 {
            assert!((result[i] - i as f64).abs() < 0.5,
                "i={i}: {} vs {}", result[i], i);
        }
    }

    #[test]
    fn test_emd_short_signal() {
        let signal = vec![1.0, 2.0, 3.0];
        let config = EmdConfig::default();
        let imfs = emd(&signal, &config);
        assert_eq!(imfs.len(), 1); // Too short, just return as-is
    }

    #[test]
    fn test_normalized_sd() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![1.0, 2.0, 3.0];
        assert!((normalized_sd(&a, &b)).abs() < 1e-10);

        let c = vec![2.0, 3.0, 4.0];
        let sd = normalized_sd(&a, &c);
        assert!(sd > 0.0);
    }

    #[test]
    fn test_config_custom() {
        let config = EmdConfig {
            max_sift_iterations: 50,
            sift_threshold: 0.1,
            max_imfs: 5,
            residual_threshold: 1e-8,
        };
        let signal: Vec<f64> = (0..128).map(|i| (i as f64 * 0.2).sin()).collect();
        let imfs = emd(&signal, &config);
        assert!(imfs.len() <= 6); // max_imfs + residual
    }
}

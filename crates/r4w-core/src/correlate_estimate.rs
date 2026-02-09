//! Correlate Estimate — Time-domain cross-correlation for delay estimation
//!
//! Computes cross-correlation between two signals to estimate time delay,
//! Doppler shift, and signal similarity. Essential for synchronization,
//! channel sounding, radar processing, and TDOA estimation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::correlate_estimate::{cross_correlate, find_delay};
//!
//! let reference = vec![0.0, 0.0, 1.0, 1.0, 0.0, 0.0];
//! let delayed   = vec![0.0, 0.0, 0.0, 1.0, 1.0, 0.0];
//! let delay = find_delay(&reference, &delayed);
//! assert_eq!(delay, 1); // Delayed by 1 sample
//! ```

use num_complex::Complex64;

/// Compute normalized cross-correlation between two real signals.
///
/// Returns correlation values for lags from -(len-1) to +(len-1).
/// Output length = 2*max(a.len(), b.len()) - 1.
pub fn cross_correlate(a: &[f64], b: &[f64]) -> Vec<f64> {
    let n = a.len().max(b.len());
    let out_len = 2 * n - 1;
    let mut result = vec![0.0; out_len];

    for lag in 0..out_len {
        let shift = lag as isize - (n as isize - 1);
        let mut sum = 0.0;
        for i in 0..a.len() {
            let j = i as isize + shift;
            if j >= 0 && (j as usize) < b.len() {
                sum += a[i] * b[j as usize];
            }
        }
        result[lag] = sum;
    }
    result
}

/// Compute cross-correlation between complex signals.
pub fn cross_correlate_complex(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    let n = a.len().max(b.len());
    let out_len = 2 * n - 1;
    let mut result = vec![Complex64::new(0.0, 0.0); out_len];

    for lag in 0..out_len {
        let shift = lag as isize - (n as isize - 1);
        let mut sum = Complex64::new(0.0, 0.0);
        for i in 0..a.len() {
            let j = i as isize + shift;
            if j >= 0 && (j as usize) < b.len() {
                sum += a[i] * b[j as usize].conj();
            }
        }
        result[lag] = sum;
    }
    result
}

/// Find the lag (in samples) that maximizes cross-correlation.
///
/// Positive result means `b` is delayed relative to `a`.
pub fn find_delay(a: &[f64], b: &[f64]) -> isize {
    let n = a.len().max(b.len());
    let corr = cross_correlate(a, b);
    let max_idx = corr
        .iter()
        .enumerate()
        .max_by(|(_, x), (_, y)| x.partial_cmp(y).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(n - 1);
    max_idx as isize - (n as isize - 1)
}

/// Find delay between complex signals.
pub fn find_delay_complex(a: &[Complex64], b: &[Complex64]) -> isize {
    let n = a.len().max(b.len());
    let corr = cross_correlate_complex(a, b);
    let max_idx = corr
        .iter()
        .enumerate()
        .max_by(|(_, x), (_, y)| x.norm().partial_cmp(&y.norm()).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(n - 1);
    max_idx as isize - (n as isize - 1)
}

/// Compute autocorrelation of a real signal.
pub fn autocorrelate(signal: &[f64]) -> Vec<f64> {
    cross_correlate(signal, signal)
}

/// Compute autocorrelation of a complex signal.
pub fn autocorrelate_complex(signal: &[Complex64]) -> Vec<Complex64> {
    cross_correlate_complex(signal, signal)
}

/// Normalized cross-correlation (Pearson-like, range [-1, 1]).
///
/// Normalizes by the geometric mean of the energies.
pub fn cross_correlate_normalized(a: &[f64], b: &[f64]) -> Vec<f64> {
    let energy_a: f64 = a.iter().map(|&x| x * x).sum();
    let energy_b: f64 = b.iter().map(|&x| x * x).sum();
    let norm = (energy_a * energy_b).sqrt();

    let max_len = a.len().max(b.len());
    if norm < 1e-30 || max_len == 0 {
        return if max_len == 0 {
            Vec::new()
        } else {
            vec![0.0; 2 * max_len - 1]
        };
    }

    cross_correlate(a, b)
        .into_iter()
        .map(|v| v / norm)
        .collect()
}

/// Compute the peak correlation coefficient between two signals.
///
/// Returns value in [0, 1] indicating similarity.
pub fn correlation_coefficient(a: &[f64], b: &[f64]) -> f64 {
    let corr = cross_correlate_normalized(a, b);
    corr.iter()
        .map(|&v| v.abs())
        .fold(0.0f64, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_identity_correlation() {
        let sig = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let delay = find_delay(&sig, &sig);
        assert_eq!(delay, 0);
    }

    #[test]
    fn test_delayed_signal() {
        let a = vec![0.0, 0.0, 1.0, 1.0, 0.0, 0.0];
        let b = vec![0.0, 0.0, 0.0, 1.0, 1.0, 0.0];
        let delay = find_delay(&a, &b);
        assert_eq!(delay, 1);
    }

    #[test]
    fn test_negative_delay() {
        let a = vec![0.0, 0.0, 0.0, 1.0, 1.0, 0.0];
        let b = vec![0.0, 0.0, 1.0, 1.0, 0.0, 0.0];
        let delay = find_delay(&a, &b);
        assert_eq!(delay, -1);
    }

    #[test]
    fn test_autocorrelation_peak() {
        let sig = vec![1.0, -1.0, 1.0, -1.0];
        let acf = autocorrelate(&sig);
        // Peak should be at center (zero lag)
        let center = acf.len() / 2;
        let peak = acf[center];
        for (i, &v) in acf.iter().enumerate() {
            assert!(
                v <= peak + 1e-10,
                "autocorrelation peak should be at center, but index {i} has {v} > {peak}"
            );
        }
    }

    #[test]
    fn test_complex_delay() {
        let a: Vec<Complex64> = vec![
            Complex64::new(0.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
        ];
        let b: Vec<Complex64> = vec![
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
        ];
        let delay = find_delay_complex(&a, &b);
        assert_eq!(delay, 1);
    }

    #[test]
    fn test_normalized_range() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let corr = cross_correlate_normalized(&a, &b);
        for &v in &corr {
            assert!(
                v.abs() <= 1.0 + 1e-10,
                "normalized correlation should be <= 1, got {v}"
            );
        }
        // Peak should be near 1.0
        let peak = corr.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!((peak - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_correlation_coefficient() {
        let a = vec![1.0, 0.0, -1.0, 0.0, 1.0];
        let cc = correlation_coefficient(&a, &a);
        assert!((cc - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_orthogonal_at_zero_lag() {
        // Sine and cosine are orthogonal at zero lag
        let n = 1000;
        let a: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let b: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).cos())
            .collect();
        // Zero-lag cross-correlation should be ~0
        let corr = cross_correlate_normalized(&a, &b);
        let center = corr.len() / 2;
        assert!(
            corr[center].abs() < 0.05,
            "sin/cos zero-lag should be ~0, got {}",
            corr[center]
        );
    }

    #[test]
    fn test_cross_correlate_length() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![1.0, 2.0, 3.0];
        let corr = cross_correlate(&a, &b);
        assert_eq!(corr.len(), 5); // 2*3-1
    }

    #[test]
    fn test_single_sample() {
        let delay = find_delay(&[1.0], &[1.0]);
        assert_eq!(delay, 0);
    }

    #[test]
    fn test_empty_normalized() {
        let corr = cross_correlate_normalized(&[], &[]);
        assert!(corr.is_empty() || corr.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_complex_autocorrelation() {
        let sig: Vec<Complex64> = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(-1.0, 0.0),
        ];
        let acf = autocorrelate_complex(&sig);
        // Peak at center
        let center = acf.len() / 2;
        let peak_mag = acf[center].norm();
        for &v in &acf {
            assert!(v.norm() <= peak_mag + 1e-10);
        }
    }
}

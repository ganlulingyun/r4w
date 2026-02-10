//! # Cross Correlator
//!
//! Cross-correlation between two signals for delay estimation,
//! pattern matching, and signal alignment. Includes both direct
//! computation and normalized correlation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cross_correlator::{cross_correlate, find_delay};
//!
//! let reference = vec![0.0, 0.0, 1.0, 0.0, 0.0];
//! let delayed = vec![0.0, 0.0, 0.0, 1.0, 0.0];
//! let delay = find_delay(&reference, &delayed, 3);
//! assert_eq!(delay, Some(1)); // delayed by 1 sample
//! ```

/// Cross-correlate two signals for lags 0..max_lag.
///
/// R_xy(lag) = sum_n x[n] * y[n + lag]
pub fn cross_correlate(x: &[f64], y: &[f64], max_lag: usize) -> Vec<f64> {
    let n = x.len().min(y.len());
    if n == 0 {
        return Vec::new();
    }
    let max_lag = max_lag.min(n.saturating_sub(1));
    let mut result = Vec::with_capacity(max_lag + 1);

    for lag in 0..=max_lag {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += x[i] * y[i + lag];
        }
        result.push(sum);
    }
    result
}

/// Cross-correlate with negative and positive lags.
///
/// Returns lags from -max_lag to +max_lag.
pub fn cross_correlate_full(x: &[f64], y: &[f64], max_lag: usize) -> Vec<f64> {
    let n = x.len().min(y.len());
    if n == 0 {
        return Vec::new();
    }
    let max_lag = max_lag.min(n.saturating_sub(1));
    let mut result = Vec::with_capacity(2 * max_lag + 1);

    // Negative lags (y is ahead of x).
    for lag in (1..=max_lag).rev() {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += x[i + lag] * y[i];
        }
        result.push(sum);
    }

    // Zero and positive lags.
    for lag in 0..=max_lag {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += x[i] * y[i + lag];
        }
        result.push(sum);
    }

    result
}

/// Normalized cross-correlation (Pearson).
///
/// Output values in [-1, 1].
pub fn cross_correlate_normalized(x: &[f64], y: &[f64], max_lag: usize) -> Vec<f64> {
    let n = x.len().min(y.len());
    if n == 0 {
        return Vec::new();
    }
    let max_lag = max_lag.min(n.saturating_sub(1));

    let mean_x: f64 = x[..n].iter().sum::<f64>() / n as f64;
    let mean_y: f64 = y[..n].iter().sum::<f64>() / n as f64;
    let energy_x: f64 = x[..n].iter().map(|&v| (v - mean_x).powi(2)).sum();
    let energy_y: f64 = y[..n].iter().map(|&v| (v - mean_y).powi(2)).sum();
    let denom = (energy_x * energy_y).sqrt();

    let mut result = Vec::with_capacity(max_lag + 1);
    for lag in 0..=max_lag {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += (x[i] - mean_x) * (y[i + lag] - mean_y);
        }
        if denom > 1e-30 {
            result.push(sum / denom);
        } else {
            result.push(0.0);
        }
    }
    result
}

/// Complex cross-correlation.
pub fn cross_correlate_complex(
    x: &[(f64, f64)],
    y: &[(f64, f64)],
    max_lag: usize,
) -> Vec<(f64, f64)> {
    let n = x.len().min(y.len());
    if n == 0 {
        return Vec::new();
    }
    let max_lag = max_lag.min(n.saturating_sub(1));
    let mut result = Vec::with_capacity(max_lag + 1);

    for lag in 0..=max_lag {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for i in 0..(n - lag) {
            // x[i] * conj(y[i+lag])
            let (xr, xi) = x[i];
            let (yr, yi) = y[i + lag];
            sum_re += xr * yr + xi * yi;
            sum_im += xi * yr - xr * yi;
        }
        result.push((sum_re, sum_im));
    }
    result
}

/// Find the delay between two signals using cross-correlation.
///
/// Returns the lag (in samples) where correlation is maximum.
pub fn find_delay(reference: &[f64], signal: &[f64], max_lag: usize) -> Option<usize> {
    let corr = cross_correlate(reference, signal, max_lag);
    if corr.is_empty() {
        return None;
    }
    let (max_idx, _) = corr
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))?;
    Some(max_idx)
}

/// Find delay with sub-sample precision using parabolic interpolation.
pub fn find_delay_subsample(reference: &[f64], signal: &[f64], max_lag: usize) -> Option<f64> {
    let corr = cross_correlate(reference, signal, max_lag);
    if corr.len() < 3 {
        return find_delay(reference, signal, max_lag).map(|d| d as f64);
    }

    // Find peak index.
    let (peak_idx, _) = corr
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))?;

    if peak_idx == 0 || peak_idx >= corr.len() - 1 {
        return Some(peak_idx as f64);
    }

    // Parabolic interpolation around peak.
    let y0 = corr[peak_idx - 1];
    let y1 = corr[peak_idx];
    let y2 = corr[peak_idx + 1];
    let delta = 0.5 * (y0 - y2) / (y0 - 2.0 * y1 + y2);
    Some(peak_idx as f64 + delta)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_correlate_impulse() {
        let x = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let corr = cross_correlate(&x, &x, 4);
        // Auto-correlation of impulse: peak at lag 0.
        assert_eq!(corr[0], 1.0);
        assert_eq!(corr[1], 0.0);
    }

    #[test]
    fn test_cross_correlate_delayed() {
        let x = vec![0.0, 1.0, 0.0, 0.0, 0.0];
        let y = vec![0.0, 0.0, 1.0, 0.0, 0.0]; // Delayed by 1.
        let corr = cross_correlate(&x, &y, 3);
        assert_eq!(corr[1], 1.0); // Peak at lag 1.
    }

    #[test]
    fn test_find_delay() {
        let reference = vec![0.0, 0.0, 1.0, 2.0, 1.0, 0.0];
        let mut delayed = vec![0.0; 3];
        delayed.extend(&reference);
        delayed.truncate(reference.len());
        let delay = find_delay(&reference, &delayed, 5);
        assert!(delay.is_some());
    }

    #[test]
    fn test_find_delay_zero() {
        let x = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let delay = find_delay(&x, &x, 3);
        assert_eq!(delay, Some(0));
    }

    #[test]
    fn test_normalized() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let corr = cross_correlate_normalized(&x, &x, 0);
        assert!((corr[0] - 1.0).abs() < 1e-10); // Self-correlation = 1.0.
    }

    #[test]
    fn test_full_correlation() {
        let x = vec![0.0, 1.0, 0.0];
        let y = vec![0.0, 1.0, 0.0];
        let full = cross_correlate_full(&x, &y, 2);
        assert_eq!(full.len(), 5); // -2, -1, 0, 1, 2
        assert_eq!(full[2], 1.0); // Lag 0.
    }

    #[test]
    fn test_complex_correlation() {
        let x = vec![(1.0, 0.0), (0.0, 1.0), (0.0, 0.0)];
        let y = vec![(1.0, 0.0), (0.0, 1.0), (0.0, 0.0)];
        let corr = cross_correlate_complex(&x, &y, 2);
        // Auto: lag 0 should be sum of |x|² = 1 + 1 = 2.
        assert!((corr[0].0 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_subsample_delay() {
        let x = vec![0.0, 0.0, 1.0, 2.0, 1.0, 0.0, 0.0];
        let delay = find_delay_subsample(&x, &x, 3);
        assert!(delay.is_some());
        assert!((delay.unwrap() - 0.0).abs() < 0.5);
    }

    #[test]
    fn test_empty_inputs() {
        assert!(cross_correlate(&[], &[1.0], 5).is_empty());
        assert!(find_delay(&[], &[], 5).is_none());
    }

    #[test]
    fn test_different_lengths() {
        let x = vec![1.0, 2.0, 3.0];
        let y = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let corr = cross_correlate(&x, &y, 2);
        assert_eq!(corr.len(), 3);
    }
}

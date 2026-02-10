//! # Moving Autocorrelation
//!
//! Sliding-window autocorrelation for periodicity detection,
//! signal characterization, and synchronization. Computes the
//! autocorrelation function over a sliding window for real-time
//! period estimation and signal quality assessment.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::moving_autocorrelation::MovingAutocorrelation;
//!
//! let mut mac = MovingAutocorrelation::new(64, 32);
//! let signal: Vec<f64> = (0..128).map(|i| (i as f64 * 0.1).sin()).collect();
//! let acf = mac.process(&signal);
//! assert!(!acf.is_empty());
//! ```

/// Moving autocorrelation estimator.
#[derive(Debug, Clone)]
pub struct MovingAutocorrelation {
    /// Window size for autocorrelation computation.
    window_size: usize,
    /// Maximum lag to compute.
    max_lag: usize,
    /// Internal buffer.
    buffer: Vec<f64>,
    /// Whether to normalize output.
    normalize: bool,
}

impl MovingAutocorrelation {
    /// Create a new moving autocorrelation.
    ///
    /// * `window_size` - Number of samples in the sliding window
    /// * `max_lag` - Maximum autocorrelation lag to compute
    pub fn new(window_size: usize, max_lag: usize) -> Self {
        Self {
            window_size: window_size.max(2),
            max_lag: max_lag.min(window_size.saturating_sub(1)).max(1),
            buffer: Vec::new(),
            normalize: true,
        }
    }

    /// Create without normalization (raw correlation values).
    pub fn unnormalized(window_size: usize, max_lag: usize) -> Self {
        let mut mac = Self::new(window_size, max_lag);
        mac.normalize = false;
        mac
    }

    /// Process a block of samples, returning ACF for each output position.
    ///
    /// Returns one ACF vector per output sample (from the point where
    /// the buffer has `window_size` samples).
    pub fn process(&mut self, input: &[f64]) -> Vec<Vec<f64>> {
        self.buffer.extend_from_slice(input);
        let mut outputs = Vec::new();

        while self.buffer.len() >= self.window_size {
            let window = &self.buffer[..self.window_size];
            let acf = autocorrelate_window(window, self.max_lag, self.normalize);
            outputs.push(acf);
            self.buffer.drain(..1);
        }

        outputs
    }

    /// Process and return only the last ACF (most recent window).
    pub fn process_last(&mut self, input: &[f64]) -> Option<Vec<f64>> {
        self.buffer.extend_from_slice(input);
        if self.buffer.len() < self.window_size {
            return None;
        }
        // Keep only the most recent window_size samples.
        let excess = self.buffer.len() - self.window_size;
        if excess > 0 {
            self.buffer.drain(..excess);
        }
        let acf = autocorrelate_window(&self.buffer, self.max_lag, self.normalize);
        Some(acf)
    }

    /// Get the dominant period from the most recent ACF.
    pub fn detect_period(&mut self, input: &[f64]) -> Option<usize> {
        let acf = self.process_last(input)?;
        find_dominant_period(&acf)
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.buffer.clear();
    }

    /// Get window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Get max lag.
    pub fn max_lag(&self) -> usize {
        self.max_lag
    }
}

/// Compute autocorrelation of a window for lags 0..max_lag.
pub fn autocorrelate_window(window: &[f64], max_lag: usize, normalize: bool) -> Vec<f64> {
    let n = window.len();
    let max_lag = max_lag.min(n - 1);
    let mut acf = Vec::with_capacity(max_lag + 1);

    // Compute mean.
    let mean: f64 = window.iter().sum::<f64>() / n as f64;

    // Compute R(0) for normalization.
    let r0: f64 = window.iter().map(|&x| (x - mean) * (x - mean)).sum();

    for lag in 0..=max_lag {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += (window[i] - mean) * (window[i + lag] - mean);
        }
        if normalize && r0 > 1e-30 {
            acf.push(sum / r0);
        } else {
            acf.push(sum);
        }
    }

    acf
}

/// Compute autocorrelation for complex samples.
pub fn autocorrelate_complex(
    window: &[(f64, f64)],
    max_lag: usize,
) -> Vec<(f64, f64)> {
    let n = window.len();
    let max_lag = max_lag.min(n.saturating_sub(1));
    let mut acf = Vec::with_capacity(max_lag + 1);

    for lag in 0..=max_lag {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for i in 0..(n - lag) {
            // x[i] * conj(x[i+lag])
            let (a_re, a_im) = window[i];
            let (b_re, b_im) = window[i + lag];
            sum_re += a_re * b_re + a_im * b_im;
            sum_im += a_im * b_re - a_re * b_im;
        }
        acf.push((sum_re, sum_im));
    }

    acf
}

/// Find the dominant period from an ACF by finding the first peak after lag 0.
pub fn find_dominant_period(acf: &[f64]) -> Option<usize> {
    if acf.len() < 3 {
        return None;
    }

    // Skip lag 0, find first local maximum.
    let mut in_descent = false;
    for i in 1..acf.len() {
        if !in_descent && acf[i] < acf[i - 1] {
            in_descent = true;
        }
        if in_descent && i + 1 < acf.len() && acf[i] > acf[i - 1] && acf[i] >= acf[i + 1] {
            // Found a peak — check it's significant (> 0.1 of lag-0).
            if acf[i] > 0.1 * acf[0] {
                return Some(i);
            }
        }
    }
    // Check last element.
    if in_descent && acf.len() >= 2 && acf[acf.len() - 1] > acf[acf.len() - 2] {
        let last = acf.len() - 1;
        if acf[last] > 0.1 * acf[0] {
            return Some(last);
        }
    }

    None
}

/// Compute the pitch/period estimate from a signal using autocorrelation.
pub fn estimate_period(signal: &[f64], min_period: usize, max_period: usize) -> Option<usize> {
    let max_lag = max_period.min(signal.len().saturating_sub(1));
    let acf = autocorrelate_window(signal, max_lag, true);

    // Find highest peak in [min_period, max_period].
    let mut best_lag = 0;
    let mut best_val = f64::NEG_INFINITY;
    for lag in min_period..=max_lag.min(acf.len().saturating_sub(1)) {
        if acf[lag] > best_val {
            best_val = acf[lag];
            best_lag = lag;
        }
    }

    if best_val > 0.1 {
        Some(best_lag)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_autocorrelate_constant() {
        let window = vec![5.0; 32];
        let acf = autocorrelate_window(&window, 10, true);
        // Constant signal: after mean removal, all zeros → R(0) = 0.
        // Should be all zeros or NaN-safe.
        assert_eq!(acf.len(), 11);
    }

    #[test]
    fn test_autocorrelate_sine() {
        // Sine wave with known period.
        let period = 16;
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / period as f64).sin())
            .collect();
        let acf = autocorrelate_window(&signal, 64, true);
        // ACF at lag=period should be strongly positive (biased estimator ~0.94).
        assert!(acf[period] > 0.85);
        // ACF at lag=period/2 should be strongly negative.
        assert!(acf[period / 2] < -0.85);
    }

    #[test]
    fn test_normalized_lag0() {
        let signal: Vec<f64> = (0..64).map(|i| i as f64).collect();
        let acf = autocorrelate_window(&signal, 10, true);
        assert!((acf[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_unnormalized() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let acf_norm = autocorrelate_window(&signal, 3, true);
        let acf_raw = autocorrelate_window(&signal, 3, false);
        // Normalized lag-0 = 1.0, raw lag-0 = sum of squares of deviations.
        assert!((acf_norm[0] - 1.0).abs() < 1e-10);
        assert!(acf_raw[0] > 0.0);
    }

    #[test]
    fn test_find_dominant_period() {
        let period = 20;
        let n = 200;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / period as f64).sin())
            .collect();
        let acf = autocorrelate_window(&signal, 100, true);
        let detected = find_dominant_period(&acf);
        assert!(detected.is_some());
        assert!((detected.unwrap() as i32 - period as i32).unsigned_abs() <= 1);
    }

    #[test]
    fn test_estimate_period() {
        let period = 25;
        let n = 200;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / period as f64).sin())
            .collect();
        let est = estimate_period(&signal, 10, 50);
        assert!(est.is_some());
        assert_eq!(est.unwrap(), period);
    }

    #[test]
    fn test_moving_autocorrelation() {
        let mut mac = MovingAutocorrelation::new(32, 16);
        let signal: Vec<f64> = (0..64).map(|i| (i as f64 * 0.3).sin()).collect();
        let results = mac.process(&signal);
        assert!(!results.is_empty());
        // Each result should have max_lag+1 elements.
        for acf in &results {
            assert_eq!(acf.len(), 17); // 0..16
        }
    }

    #[test]
    fn test_process_last() {
        let mut mac = MovingAutocorrelation::new(32, 8);
        let signal: Vec<f64> = (0..64).map(|i| i as f64).collect();
        let acf = mac.process_last(&signal);
        assert!(acf.is_some());
        assert_eq!(acf.unwrap().len(), 9);
    }

    #[test]
    fn test_complex_autocorrelation() {
        let n = 32;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * i as f64 / 8.0;
                (phase.cos(), phase.sin())
            })
            .collect();
        let acf = autocorrelate_complex(&signal, 16);
        assert_eq!(acf.len(), 17);
        // Lag 0 should have large real component.
        assert!(acf[0].0 > 0.0);
    }

    #[test]
    fn test_detect_period() {
        let period = 10;
        let signal: Vec<f64> = (0..100)
            .map(|i| (2.0 * PI * i as f64 / period as f64).sin())
            .collect();
        let mut mac = MovingAutocorrelation::new(80, 40);
        let detected = mac.detect_period(&signal);
        assert!(detected.is_some());
        assert!((detected.unwrap() as i32 - period as i32).unsigned_abs() <= 1);
    }
}

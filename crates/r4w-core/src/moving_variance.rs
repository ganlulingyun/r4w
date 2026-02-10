//! # Moving Variance
//!
//! Sliding window variance and standard deviation calculation using
//! Welford's online algorithm. Complements `moving_average` and
//! `moving_rms` with second-order statistics.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::moving_variance::MovingVariance;
//!
//! let mut mv = MovingVariance::new(4);
//! for &x in &[1.0, 2.0, 3.0, 4.0, 5.0] {
//!     mv.push(x);
//! }
//! let var = mv.variance(); // Variance of [2.0, 3.0, 4.0, 5.0]
//! let std = mv.std_dev();  // Standard deviation
//! ```

use std::collections::VecDeque;

/// Sliding window variance calculator.
#[derive(Debug, Clone)]
pub struct MovingVariance {
    /// Window size.
    window_size: usize,
    /// Sample buffer.
    buffer: VecDeque<f64>,
    /// Running sum.
    sum: f64,
    /// Running sum of squares.
    sum_sq: f64,
    /// Total samples pushed.
    count: u64,
}

impl MovingVariance {
    /// Create a new moving variance with the given window size.
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size: window_size.max(1),
            buffer: VecDeque::with_capacity(window_size),
            sum: 0.0,
            sum_sq: 0.0,
            count: 0,
        }
    }

    /// Push a new sample into the window.
    pub fn push(&mut self, sample: f64) {
        self.sum += sample;
        self.sum_sq += sample * sample;
        self.buffer.push_back(sample);
        self.count += 1;

        if self.buffer.len() > self.window_size {
            let old = self.buffer.pop_front().unwrap();
            self.sum -= old;
            self.sum_sq -= old * old;
        }
    }

    /// Process a block of samples, returning variance for each.
    pub fn process(&mut self, samples: &[f64]) -> Vec<f64> {
        let mut out = Vec::with_capacity(samples.len());
        for &s in samples {
            self.push(s);
            out.push(self.variance());
        }
        out
    }

    /// Get the current mean of the window.
    pub fn mean(&self) -> f64 {
        let n = self.buffer.len();
        if n == 0 {
            return 0.0;
        }
        self.sum / n as f64
    }

    /// Get the current population variance of the window.
    pub fn variance(&self) -> f64 {
        let n = self.buffer.len();
        if n == 0 {
            return 0.0;
        }
        let mean = self.sum / n as f64;
        (self.sum_sq / n as f64) - mean * mean
    }

    /// Get the current sample variance (Bessel's correction, N-1).
    pub fn sample_variance(&self) -> f64 {
        let n = self.buffer.len();
        if n <= 1 {
            return 0.0;
        }
        let mean = self.sum / n as f64;
        let var = (self.sum_sq / n as f64) - mean * mean;
        var * n as f64 / (n - 1) as f64
    }

    /// Get the population standard deviation.
    pub fn std_dev(&self) -> f64 {
        self.variance().max(0.0).sqrt()
    }

    /// Get the sample standard deviation.
    pub fn sample_std_dev(&self) -> f64 {
        self.sample_variance().max(0.0).sqrt()
    }

    /// Get the coefficient of variation (std_dev / mean).
    pub fn cv(&self) -> f64 {
        let m = self.mean();
        if m.abs() < 1e-30 {
            return 0.0;
        }
        self.std_dev() / m.abs()
    }

    /// Get the current window fill level.
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Check if the window is empty.
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    /// Check if the window is full.
    pub fn is_full(&self) -> bool {
        self.buffer.len() == self.window_size
    }

    /// Get total samples pushed.
    pub fn total_count(&self) -> u64 {
        self.count
    }

    /// Get the window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Reset the calculator.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.sum = 0.0;
        self.sum_sq = 0.0;
        self.count = 0;
    }
}

/// Moving covariance between two signals.
#[derive(Debug, Clone)]
pub struct MovingCovariance {
    window_size: usize,
    buf_x: VecDeque<f64>,
    buf_y: VecDeque<f64>,
    sum_x: f64,
    sum_y: f64,
    sum_xy: f64,
}

impl MovingCovariance {
    /// Create a new moving covariance calculator.
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size: window_size.max(1),
            buf_x: VecDeque::with_capacity(window_size),
            buf_y: VecDeque::with_capacity(window_size),
            sum_x: 0.0,
            sum_y: 0.0,
            sum_xy: 0.0,
        }
    }

    /// Push a pair of samples.
    pub fn push(&mut self, x: f64, y: f64) {
        self.sum_x += x;
        self.sum_y += y;
        self.sum_xy += x * y;
        self.buf_x.push_back(x);
        self.buf_y.push_back(y);

        if self.buf_x.len() > self.window_size {
            let old_x = self.buf_x.pop_front().unwrap();
            let old_y = self.buf_y.pop_front().unwrap();
            self.sum_x -= old_x;
            self.sum_y -= old_y;
            self.sum_xy -= old_x * old_y;
        }
    }

    /// Get the population covariance.
    pub fn covariance(&self) -> f64 {
        let n = self.buf_x.len();
        if n == 0 {
            return 0.0;
        }
        let mean_x = self.sum_x / n as f64;
        let mean_y = self.sum_y / n as f64;
        (self.sum_xy / n as f64) - mean_x * mean_y
    }

    /// Get the Pearson correlation coefficient.
    pub fn correlation(&self) -> f64 {
        let n = self.buf_x.len();
        if n == 0 {
            return 0.0;
        }
        let nf = n as f64;
        let mean_x = self.sum_x / nf;
        let mean_y = self.sum_y / nf;
        let var_x: f64 = self.buf_x.iter().map(|&x| (x - mean_x).powi(2)).sum::<f64>() / nf;
        let var_y: f64 = self.buf_y.iter().map(|&y| (y - mean_y).powi(2)).sum::<f64>() / nf;
        let std_xy = (var_x * var_y).sqrt();
        if std_xy < 1e-30 {
            return 0.0;
        }
        self.covariance() / std_xy
    }

    /// Reset the calculator.
    pub fn reset(&mut self) {
        self.buf_x.clear();
        self.buf_y.clear();
        self.sum_x = 0.0;
        self.sum_y = 0.0;
        self.sum_xy = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant_signal() {
        let mut mv = MovingVariance::new(4);
        for _ in 0..10 {
            mv.push(5.0);
        }
        assert!(mv.variance() < 1e-10, "Constant signal should have zero variance");
        assert!((mv.mean() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_known_variance() {
        let mut mv = MovingVariance::new(4);
        // Window: [1, 2, 3, 4]. Mean=2.5, Var = ((1-2.5)^2+(2-2.5)^2+(3-2.5)^2+(4-2.5)^2)/4 = 1.25
        for &x in &[1.0, 2.0, 3.0, 4.0] {
            mv.push(x);
        }
        assert!((mv.variance() - 1.25).abs() < 1e-10, "var={}", mv.variance());
        assert!((mv.std_dev() - 1.25_f64.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_sample_variance() {
        let mut mv = MovingVariance::new(4);
        for &x in &[1.0, 2.0, 3.0, 4.0] {
            mv.push(x);
        }
        // Sample var = 1.25 * 4/3 = 5/3 ≈ 1.6667
        assert!((mv.sample_variance() - 5.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_sliding_window() {
        let mut mv = MovingVariance::new(3);
        mv.push(1.0);
        mv.push(2.0);
        mv.push(3.0);
        // Window: [1,2,3], mean=2, var = 2/3
        assert!((mv.variance() - 2.0 / 3.0).abs() < 1e-10);

        mv.push(10.0);
        // Window: [2,3,10], mean=5, var = (9+4+25)/3 = 38/3 ≈ 12.667
        assert!((mv.mean() - 5.0).abs() < 1e-10);
        let expected_var = ((2.0 - 5.0_f64).powi(2) + (3.0 - 5.0_f64).powi(2) + (10.0 - 5.0_f64).powi(2)) / 3.0;
        assert!((mv.variance() - expected_var).abs() < 1e-8, "var={} expected={}", mv.variance(), expected_var);
    }

    #[test]
    fn test_process_batch() {
        let mut mv = MovingVariance::new(3);
        let data = vec![1.0, 1.0, 1.0, 5.0, 5.0, 5.0];
        let variances = mv.process(&data);
        assert_eq!(variances.len(), 6);
        // Last 3 values: [5,5,5] → variance = 0
        assert!(variances[5] < 1e-10);
    }

    #[test]
    fn test_cv() {
        let mut mv = MovingVariance::new(4);
        for &x in &[10.0, 10.0, 10.0, 10.0] {
            mv.push(x);
        }
        assert!(mv.cv() < 1e-10, "Constant signal has CV=0");
    }

    #[test]
    fn test_empty() {
        let mv = MovingVariance::new(4);
        assert!(mv.is_empty());
        assert!(!mv.is_full());
        assert_eq!(mv.variance(), 0.0);
        assert_eq!(mv.mean(), 0.0);
    }

    #[test]
    fn test_reset() {
        let mut mv = MovingVariance::new(4);
        mv.push(1.0);
        mv.push(2.0);
        assert_eq!(mv.total_count(), 2);
        mv.reset();
        assert_eq!(mv.total_count(), 0);
        assert!(mv.is_empty());
    }

    #[test]
    fn test_covariance_identical() {
        let mut mc = MovingCovariance::new(4);
        for &x in &[1.0, 2.0, 3.0, 4.0] {
            mc.push(x, x);
        }
        // Cov(x,x) = Var(x) = 1.25
        assert!((mc.covariance() - 1.25).abs() < 1e-10);
        assert!((mc.correlation() - 1.0).abs() < 1e-10, "r={}", mc.correlation());
    }

    #[test]
    fn test_covariance_inverse() {
        let mut mc = MovingCovariance::new(4);
        for &x in &[1.0, 2.0, 3.0, 4.0] {
            mc.push(x, -x);
        }
        // Cov(x, -x) = -Var(x) = -1.25
        assert!((mc.covariance() + 1.25).abs() < 1e-10);
        assert!((mc.correlation() + 1.0).abs() < 1e-10, "r={}", mc.correlation());
    }
}

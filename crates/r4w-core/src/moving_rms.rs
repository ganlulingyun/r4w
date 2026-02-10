//! # Moving RMS Estimator
//!
//! Computes the running Root Mean Square (RMS) power of a signal using a
//! sliding window. Useful for power estimation, AGC control, voice activity
//! detection, and signal level monitoring.
//!
//! ## Algorithm
//!
//! RMS = sqrt(1/N * sum(x[i]^2)) over a window of N samples.
//!
//! Uses an efficient incremental update: add new sample squared, subtract
//! oldest sample squared, avoiding full window recomputation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::moving_rms::MovingRms;
//!
//! let mut rms = MovingRms::new(4);
//! // Feed 4 samples of amplitude 1.0
//! for _ in 0..4 {
//!     rms.push(1.0);
//! }
//! assert!((rms.rms() - 1.0).abs() < 1e-10);
//! ```

use std::collections::VecDeque;

/// Moving RMS estimator with sliding window.
#[derive(Debug, Clone)]
pub struct MovingRms {
    /// Window size.
    window_size: usize,
    /// Circular buffer of samples.
    buffer: VecDeque<f64>,
    /// Running sum of squared samples.
    sum_sq: f64,
    /// Total samples processed.
    samples_processed: u64,
}

impl MovingRms {
    /// Create a new moving RMS estimator with the given window size.
    pub fn new(window_size: usize) -> Self {
        let window_size = window_size.max(1);
        Self {
            window_size,
            buffer: VecDeque::with_capacity(window_size),
            sum_sq: 0.0,
            samples_processed: 0,
        }
    }

    /// Push a new sample and update the RMS estimate.
    pub fn push(&mut self, sample: f64) {
        let sq = sample * sample;
        self.sum_sq += sq;

        self.buffer.push_back(sample);
        if self.buffer.len() > self.window_size {
            let old = self.buffer.pop_front().unwrap();
            self.sum_sq -= old * old;
            // Clamp to zero to avoid floating-point drift.
            if self.sum_sq < 0.0 {
                self.sum_sq = 0.0;
            }
        }

        self.samples_processed += 1;
    }

    /// Push a complex sample (uses magnitude squared).
    pub fn push_complex(&mut self, re: f64, im: f64) {
        let mag = (re * re + im * im).sqrt();
        self.push(mag);
    }

    /// Push a block of samples.
    pub fn push_block(&mut self, samples: &[f64]) {
        for &s in samples {
            self.push(s);
        }
    }

    /// Get the current RMS value.
    pub fn rms(&self) -> f64 {
        if self.buffer.is_empty() {
            return 0.0;
        }
        (self.sum_sq / self.buffer.len() as f64).sqrt()
    }

    /// Get the current RMS in dB (relative to full scale = 1.0).
    pub fn rms_db(&self) -> f64 {
        let rms = self.rms();
        if rms > 1e-30 {
            20.0 * rms.log10()
        } else {
            -300.0
        }
    }

    /// Get the current mean power (sum_sq / N).
    pub fn mean_power(&self) -> f64 {
        if self.buffer.is_empty() {
            return 0.0;
        }
        self.sum_sq / self.buffer.len() as f64
    }

    /// Get the current mean power in dB.
    pub fn mean_power_db(&self) -> f64 {
        let p = self.mean_power();
        if p > 1e-30 {
            10.0 * p.log10()
        } else {
            -300.0
        }
    }

    /// Get the peak sample value in the current window.
    pub fn peak(&self) -> f64 {
        self.buffer
            .iter()
            .map(|x| x.abs())
            .fold(0.0_f64, f64::max)
    }

    /// Get the crest factor (peak/RMS).
    pub fn crest_factor(&self) -> f64 {
        let rms = self.rms();
        if rms > 1e-30 {
            self.peak() / rms
        } else {
            0.0
        }
    }

    /// Get the window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Get total samples processed.
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed
    }

    /// Check if the window is full.
    pub fn is_full(&self) -> bool {
        self.buffer.len() >= self.window_size
    }

    /// Reset the estimator.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.sum_sq = 0.0;
        self.samples_processed = 0;
    }
}

/// Moving RMS for complex (IQ) signals.
#[derive(Debug, Clone)]
pub struct MovingRmsComplex {
    inner: MovingRms,
}

impl MovingRmsComplex {
    /// Create a new complex moving RMS estimator.
    pub fn new(window_size: usize) -> Self {
        Self {
            inner: MovingRms::new(window_size),
        }
    }

    /// Push a complex IQ sample.
    pub fn push(&mut self, re: f64, im: f64) {
        self.inner.push_complex(re, im);
    }

    /// Push a block of complex samples.
    pub fn push_block(&mut self, samples: &[(f64, f64)]) {
        for &(re, im) in samples {
            self.push(re, im);
        }
    }

    /// Get the current RMS value.
    pub fn rms(&self) -> f64 {
        self.inner.rms()
    }

    /// Get the current RMS in dB.
    pub fn rms_db(&self) -> f64 {
        self.inner.rms_db()
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.inner.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant_signal() {
        let mut rms = MovingRms::new(10);
        for _ in 0..10 {
            rms.push(3.0);
        }
        assert!((rms.rms() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_alternating_signal() {
        let mut rms = MovingRms::new(4);
        rms.push(1.0);
        rms.push(-1.0);
        rms.push(1.0);
        rms.push(-1.0);
        assert!((rms.rms() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_sliding_window() {
        let mut rms = MovingRms::new(3);
        rms.push(1.0);
        rms.push(1.0);
        rms.push(1.0);
        assert!((rms.rms() - 1.0).abs() < 1e-10);
        rms.push(0.0); // window: [1, 1, 0]
        let expected = ((1.0 + 1.0 + 0.0) / 3.0_f64).sqrt();
        assert!((rms.rms() - expected).abs() < 1e-10);
    }

    #[test]
    fn test_empty() {
        let rms = MovingRms::new(5);
        assert!((rms.rms() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_rms_db() {
        let mut rms = MovingRms::new(10);
        for _ in 0..10 {
            rms.push(1.0);
        }
        assert!((rms.rms_db() - 0.0).abs() < 1e-6); // 1.0 → 0 dB
    }

    #[test]
    fn test_mean_power() {
        let mut rms = MovingRms::new(4);
        for _ in 0..4 {
            rms.push(2.0);
        }
        assert!((rms.mean_power() - 4.0).abs() < 1e-10); // 2^2 = 4
    }

    #[test]
    fn test_peak_and_crest_factor() {
        let mut rms = MovingRms::new(4);
        rms.push(1.0);
        rms.push(0.0);
        rms.push(0.0);
        rms.push(0.0);
        assert!((rms.peak() - 1.0).abs() < 1e-10);
        // RMS = sqrt(1/4) = 0.5, crest = 1.0/0.5 = 2.0
        assert!((rms.crest_factor() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_push_block() {
        let mut rms = MovingRms::new(4);
        rms.push_block(&[1.0, 1.0, 1.0, 1.0]);
        assert!((rms.rms() - 1.0).abs() < 1e-10);
        assert!(rms.is_full());
    }

    #[test]
    fn test_reset() {
        let mut rms = MovingRms::new(4);
        rms.push_block(&[1.0, 2.0, 3.0]);
        assert!(rms.samples_processed() > 0);
        rms.reset();
        assert_eq!(rms.samples_processed(), 0);
        assert!((rms.rms() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_rms() {
        let mut rms = MovingRmsComplex::new(4);
        // Unit circle: |z| = 1
        rms.push(1.0, 0.0);
        rms.push(0.0, 1.0);
        rms.push(-1.0, 0.0);
        rms.push(0.0, -1.0);
        assert!((rms.rms() - 1.0).abs() < 1e-10);
    }
}

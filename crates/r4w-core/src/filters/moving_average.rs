//! Moving Average Filter
//!
//! Efficient O(1)-per-sample moving average using a circular buffer.
//! The running sum is updated incrementally by adding the new sample
//! and subtracting the oldest.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::filters::moving_average::MovingAverage;
//! use num_complex::Complex64;
//!
//! let mut ma = MovingAverage::new(4);
//! let input = vec![Complex64::new(1.0, 0.0); 10];
//! let output = ma.process_block(&input);
//! assert_eq!(output.len(), 10);
//! // After 4 samples of DC 1.0, output should converge to 1.0
//! assert!((output[4].re - 1.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Moving average filter with O(1) per-sample cost.
#[derive(Debug, Clone)]
pub struct MovingAverage {
    /// Window length
    length: usize,
    /// Circular buffer
    buffer: Vec<Complex64>,
    /// Write index
    write_idx: usize,
    /// Running sum
    sum: Complex64,
    /// Number of samples inserted (up to length)
    count: usize,
}

impl MovingAverage {
    /// Create a new moving average filter with the given window length.
    pub fn new(length: usize) -> Self {
        assert!(length > 0, "Length must be positive");
        Self {
            length,
            buffer: vec![Complex64::new(0.0, 0.0); length],
            write_idx: 0,
            sum: Complex64::new(0.0, 0.0),
            count: 0,
        }
    }

    /// Process a single complex sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        // Subtract oldest
        self.sum -= self.buffer[self.write_idx];
        // Store and add new
        self.buffer[self.write_idx] = input;
        self.sum += input;
        self.write_idx = (self.write_idx + 1) % self.length;

        if self.count < self.length {
            self.count += 1;
        }

        self.sum / self.count as f64
    }

    /// Process a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process a single real sample.
    pub fn process_real_sample(&mut self, input: f64) -> f64 {
        self.process(Complex64::new(input, 0.0)).re
    }

    /// Process a block of real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| self.process_real_sample(s)).collect()
    }

    /// Get the window length.
    pub fn length(&self) -> usize {
        self.length
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.buffer.fill(Complex64::new(0.0, 0.0));
        self.write_idx = 0;
        self.sum = Complex64::new(0.0, 0.0);
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dc_convergence() {
        let mut ma = MovingAverage::new(4);
        let dc = vec![Complex64::new(1.0, 0.0); 10];
        let out = ma.process_block(&dc);
        // After window fills, output should be exactly 1.0
        for &v in &out[4..] {
            assert!((v.re - 1.0).abs() < 1e-10, "Expected 1.0, got {}", v.re);
        }
    }

    #[test]
    fn test_ramp_averaging() {
        let mut ma = MovingAverage::new(3);
        let ramp: Vec<Complex64> = (0..6)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let out = ma.process_block(&ramp);
        // After 3 samples: avg of [0,1,2] = 1.0
        assert!((out[2].re - 1.0).abs() < 1e-10);
        // avg of [1,2,3] = 2.0
        assert!((out[3].re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_length_one() {
        let mut ma = MovingAverage::new(1);
        let input = vec![
            Complex64::new(5.0, 0.0),
            Complex64::new(3.0, 0.0),
        ];
        let out = ma.process_block(&input);
        assert!((out[0].re - 5.0).abs() < 1e-10);
        assert!((out[1].re - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_real_processing() {
        let mut ma = MovingAverage::new(2);
        let out = ma.process_real(&[1.0, 3.0, 5.0, 7.0]);
        assert!((out[1] - 2.0).abs() < 1e-10); // avg(1, 3)
        assert!((out[2] - 4.0).abs() < 1e-10); // avg(3, 5)
        assert!((out[3] - 6.0).abs() < 1e-10); // avg(5, 7)
    }

    #[test]
    fn test_reset() {
        let mut ma = MovingAverage::new(4);
        ma.process(Complex64::new(100.0, 0.0));
        ma.reset();
        let out = ma.process(Complex64::new(1.0, 0.0));
        assert!((out.re - 1.0).abs() < 1e-10, "After reset, first output should be 1.0");
    }

    #[test]
    fn test_complex_averaging() {
        let mut ma = MovingAverage::new(2);
        let input = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(3.0, 4.0),
        ];
        let out = ma.process_block(&input);
        // avg of (1+2j, 3+4j) = 2+3j
        assert!((out[1].re - 2.0).abs() < 1e-10);
        assert!((out[1].im - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_output_length_matches_input() {
        let mut ma = MovingAverage::new(8);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let out = ma.process_block(&input);
        assert_eq!(out.len(), 100);
    }
}

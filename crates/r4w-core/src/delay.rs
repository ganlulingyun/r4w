//! Sample Delay Block
//!
//! Delays an input sample stream by a configurable number of samples,
//! inserting zeros at the beginning. Used for path alignment, filter
//! group delay compensation, and building feedback loops.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::delay::Delay;
//! use num_complex::Complex64;
//!
//! let mut delay = Delay::new(3);
//! let input = vec![
//!     Complex64::new(1.0, 0.0),
//!     Complex64::new(2.0, 0.0),
//!     Complex64::new(3.0, 0.0),
//! ];
//! let output = delay.process_block(&input);
//! // First 3 samples are zero (delay), then input follows
//! assert_eq!(output.len(), 3);
//! assert!((output[0].re - 0.0).abs() < 1e-10); // delayed zero
//! ```

use num_complex::Complex64;
use std::collections::VecDeque;

/// Configurable sample delay block.
///
/// Inserts `delay` zero-valued samples at the beginning of the stream.
/// After the delay is filled, input samples pass through.
#[derive(Debug, Clone)]
pub struct Delay {
    buffer: VecDeque<Complex64>,
    delay: usize,
}

impl Delay {
    /// Create a new delay block with the given number of samples.
    pub fn new(delay: usize) -> Self {
        let mut buffer = VecDeque::with_capacity(delay);
        for _ in 0..delay {
            buffer.push_back(Complex64::new(0.0, 0.0));
        }
        Self { buffer, delay }
    }

    /// Process a single sample through the delay.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        self.buffer.push_back(input);
        self.buffer.pop_front().unwrap_or(Complex64::new(0.0, 0.0))
    }

    /// Process a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process a block of real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| {
            let out = self.process(Complex64::new(s, 0.0));
            out.re
        }).collect()
    }

    /// Get the delay in samples.
    pub fn delay(&self) -> usize {
        self.delay
    }

    /// Set a new delay value. Resets internal state.
    pub fn set_delay(&mut self, delay: usize) {
        self.delay = delay;
        self.buffer.clear();
        for _ in 0..delay {
            self.buffer.push_back(Complex64::new(0.0, 0.0));
        }
    }

    /// Reset internal state (refill buffer with zeros).
    pub fn reset(&mut self) {
        self.buffer.clear();
        for _ in 0..self.delay {
            self.buffer.push_back(Complex64::new(0.0, 0.0));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_delay() {
        let mut d = Delay::new(0);
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)];
        let output = d.process_block(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[1].re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_delay_inserts_zeros() {
        let mut d = Delay::new(3);
        let input: Vec<Complex64> = (1..=5)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = d.process_block(&input);
        assert_eq!(output.len(), 5);
        // First 3 outputs are zeros
        assert!((output[0].re - 0.0).abs() < 1e-10);
        assert!((output[1].re - 0.0).abs() < 1e-10);
        assert!((output[2].re - 0.0).abs() < 1e-10);
        // Then input follows
        assert!((output[3].re - 1.0).abs() < 1e-10);
        assert!((output[4].re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_delay_continuity() {
        let mut d = Delay::new(2);
        // First block
        let out1 = d.process_block(&[Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)]);
        assert!((out1[0].re - 0.0).abs() < 1e-10);
        assert!((out1[1].re - 0.0).abs() < 1e-10);
        // Second block continues from where first left off
        let out2 = d.process_block(&[Complex64::new(3.0, 0.0)]);
        assert!((out2[0].re - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_delay_real() {
        let mut d = Delay::new(2);
        let output = d.process_real(&[10.0, 20.0, 30.0, 40.0]);
        assert!((output[0] - 0.0).abs() < 1e-10);
        assert!((output[1] - 0.0).abs() < 1e-10);
        assert!((output[2] - 10.0).abs() < 1e-10);
        assert!((output[3] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_delay_complex() {
        let mut d = Delay::new(1);
        let input = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(3.0, 4.0),
        ];
        let output = d.process_block(&input);
        assert!((output[0].re - 0.0).abs() < 1e-10);
        assert!((output[0].im - 0.0).abs() < 1e-10);
        assert!((output[1].re - 1.0).abs() < 1e-10);
        assert!((output[1].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_set_delay() {
        let mut d = Delay::new(2);
        d.process(Complex64::new(1.0, 0.0));
        d.set_delay(1);
        let out = d.process(Complex64::new(5.0, 0.0));
        assert!((out.re - 0.0).abs() < 1e-10); // New delay, fresh zeros
        let out = d.process(Complex64::new(6.0, 0.0));
        assert!((out.re - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut d = Delay::new(1);
        d.process(Complex64::new(10.0, 0.0));
        // Buffer now has [10.0] so next output would be 10.0
        d.reset();
        // After reset, buffer has [0.0] again
        let out = d.process(Complex64::new(20.0, 0.0));
        assert!((out.re - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_output_length_matches_input() {
        let mut d = Delay::new(10);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = d.process_block(&input);
        assert_eq!(output.len(), 100);
    }
}

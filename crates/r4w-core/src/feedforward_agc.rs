//! Feedforward AGC — Non-causal Automatic Gain Control
//!
//! Computes gain by looking ahead at a window of future samples,
//! providing fast response without overshoot. Unlike feedback AGC,
//! this introduces a fixed delay equal to half the window size.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::feedforward_agc::FeedforwardAgc;
//! use num_complex::Complex64;
//!
//! let mut agc = FeedforwardAgc::new(16, 1.0);
//! let input: Vec<Complex64> = (0..100)
//!     .map(|_| Complex64::new(5.0, 0.0))
//!     .collect();
//! let output = agc.process_block(&input);
//! // After settling, output amplitude should be near 1.0
//! ```

use num_complex::Complex64;
use std::collections::VecDeque;

/// Feedforward AGC — non-causal gain control.
///
/// Looks ahead over a window of samples to determine the
/// maximum amplitude, then applies the inverse gain to normalize
/// the output to the reference level.
#[derive(Debug, Clone)]
pub struct FeedforwardAgc {
    /// Window size for lookahead.
    window_size: usize,
    /// Target output amplitude.
    reference: f64,
    /// Maximum gain (prevents noise amplification in silence).
    max_gain: f64,
    /// Sample buffer.
    buffer: VecDeque<Complex64>,
}

impl FeedforwardAgc {
    pub fn new(window_size: usize, reference: f64) -> Self {
        Self {
            window_size: window_size.max(1),
            reference,
            max_gain: 65536.0,
            buffer: VecDeque::with_capacity(window_size + 1),
        }
    }

    /// Set the maximum gain limit.
    pub fn set_max_gain(&mut self, max_gain: f64) {
        self.max_gain = max_gain;
    }

    /// Process a single sample. Returns None until the window is filled.
    pub fn process_sample(&mut self, input: Complex64) -> Option<Complex64> {
        self.buffer.push_back(input);

        if self.buffer.len() < self.window_size {
            return None;
        }

        // Find max magnitude in the window
        let max_mag = self
            .buffer
            .iter()
            .map(|s| s.norm())
            .fold(0.0_f64, f64::max);

        // Compute gain
        let gain = if max_mag > 1e-30 {
            (self.reference / max_mag).min(self.max_gain)
        } else {
            1.0
        };

        // Output the oldest sample with the computed gain
        let out = self.buffer.pop_front().unwrap() * gain;
        Some(out)
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &s in input {
            if let Some(out) = self.process_sample(s) {
                output.push(out);
            }
        }
        output
    }

    /// Flush remaining buffered samples.
    pub fn flush(&mut self) -> Vec<Complex64> {
        let mut output = Vec::new();
        while !self.buffer.is_empty() {
            let max_mag = self
                .buffer
                .iter()
                .map(|s| s.norm())
                .fold(0.0_f64, f64::max);
            let gain = if max_mag > 1e-30 {
                (self.reference / max_mag).min(self.max_gain)
            } else {
                1.0
            };
            let out = self.buffer.pop_front().unwrap() * gain;
            output.push(out);
        }
        output
    }

    /// Get the delay in samples introduced by the AGC.
    pub fn delay(&self) -> usize {
        self.window_size - 1
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant_amplitude() {
        let mut agc = FeedforwardAgc::new(8, 1.0);
        let input: Vec<Complex64> = vec![Complex64::new(5.0, 0.0); 50];
        let output = agc.process_block(&input);
        // After window fills, output should be normalized to ~1.0
        for s in output.iter().skip(1) {
            assert!(
                (s.norm() - 1.0).abs() < 0.1,
                "Should normalize to 1.0, got {}",
                s.norm()
            );
        }
    }

    #[test]
    fn test_varying_amplitude() {
        let mut agc = FeedforwardAgc::new(8, 1.0);
        let mut input = Vec::new();
        // Low amplitude
        input.extend(std::iter::repeat(Complex64::new(0.1, 0.0)).take(20));
        // High amplitude
        input.extend(std::iter::repeat(Complex64::new(10.0, 0.0)).take(20));

        let output = agc.process_block(&input);
        // All outputs should be roughly normalized
        for s in &output {
            assert!(s.norm() < 2.0, "Output should be bounded, got {}", s.norm());
        }
    }

    #[test]
    fn test_zero_input() {
        let mut agc = FeedforwardAgc::new(4, 1.0);
        let input = vec![Complex64::new(0.0, 0.0); 20];
        let output = agc.process_block(&input);
        // Should handle zeros gracefully (gain = 1.0, output = 0)
        for s in &output {
            assert!(s.norm() < 1e-10);
        }
    }

    #[test]
    fn test_delay() {
        let agc = FeedforwardAgc::new(16, 1.0);
        assert_eq!(agc.delay(), 15);
    }

    #[test]
    fn test_max_gain() {
        let mut agc = FeedforwardAgc::new(4, 1.0);
        agc.set_max_gain(10.0);
        let input = vec![Complex64::new(0.001, 0.0); 20];
        let output = agc.process_block(&input);
        // With max_gain=10, output should not exceed 10*0.001 = 0.01
        for s in &output {
            assert!(
                s.norm() < 0.011,
                "Max gain should limit output, got {}",
                s.norm()
            );
        }
    }

    #[test]
    fn test_flush() {
        let mut agc = FeedforwardAgc::new(8, 1.0);
        let input = vec![Complex64::new(2.0, 0.0); 5];
        let output = agc.process_block(&input);
        assert!(output.is_empty()); // Window not yet filled
        let flushed = agc.flush();
        assert_eq!(flushed.len(), 5);
    }

    #[test]
    fn test_reset() {
        let mut agc = FeedforwardAgc::new(4, 1.0);
        agc.process_block(&vec![Complex64::new(1.0, 0.0); 10]);
        agc.reset();
        assert!(agc.buffer.is_empty());
    }

    #[test]
    fn test_complex_signal() {
        let mut agc = FeedforwardAgc::new(8, 1.0);
        let input: Vec<Complex64> = (0..50)
            .map(|i| Complex64::from_polar(3.0, i as f64 * 0.5))
            .collect();
        let output = agc.process_block(&input);
        for s in output.iter().skip(1) {
            assert!(
                (s.norm() - 1.0).abs() < 0.2,
                "Complex signal should normalize, got {}",
                s.norm()
            );
        }
    }
}

//! Multiply and Multiply-by-Constant Blocks
//!
//! Element-wise multiplication of sample streams. Used for mixing (frequency
//! shifting), applying gain, windowing, and building custom DSP chains.
//!
//! - `Multiply`: Element-wise product of two complex streams
//! - `MultiplyConst`: Multiply every sample by a fixed complex constant
//! - `MultiplyConstReal`: Multiply every real sample by a scalar
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::multiply::{Multiply, MultiplyConst};
//! use num_complex::Complex64;
//!
//! // Multiply two streams
//! let a = vec![Complex64::new(2.0, 0.0), Complex64::new(3.0, 1.0)];
//! let b = vec![Complex64::new(0.5, 0.0), Complex64::new(1.0, -1.0)];
//! let result = Multiply::process(&a, &b);
//! assert!((result[0].re - 1.0).abs() < 1e-10);
//!
//! // Multiply by constant (e.g., apply gain)
//! let mc = MultiplyConst::new(Complex64::new(2.0, 0.0));
//! let input = vec![Complex64::new(3.0, 4.0)];
//! let output = mc.process_block(&input);
//! assert!((output[0].re - 6.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Element-wise multiply of two complex streams.
pub struct Multiply;

impl Multiply {
    /// Multiply two complex streams element-wise.
    /// Output length = min(a.len(), b.len()).
    pub fn process(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x * y).collect()
    }

    /// Multiply two real streams element-wise.
    pub fn process_real(a: &[f64], b: &[f64]) -> Vec<f64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x * y).collect()
    }
}

/// Multiply every complex sample by a fixed constant.
#[derive(Debug, Clone)]
pub struct MultiplyConst {
    constant: Complex64,
}

impl MultiplyConst {
    /// Create with a complex constant.
    pub fn new(constant: Complex64) -> Self {
        Self { constant }
    }

    /// Create from a real scalar (imaginary part = 0).
    pub fn from_real(gain: f64) -> Self {
        Self { constant: Complex64::new(gain, 0.0) }
    }

    /// Process a block of complex samples.
    pub fn process_block(&self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| s * self.constant).collect()
    }

    /// Get the constant.
    pub fn constant(&self) -> Complex64 {
        self.constant
    }

    /// Set a new constant.
    pub fn set_constant(&mut self, constant: Complex64) {
        self.constant = constant;
    }
}

/// Multiply every real sample by a fixed scalar.
#[derive(Debug, Clone)]
pub struct MultiplyConstReal {
    constant: f64,
}

impl MultiplyConstReal {
    /// Create with a real scalar.
    pub fn new(constant: f64) -> Self {
        Self { constant }
    }

    /// Process a block of real samples.
    pub fn process_block(&self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| s * self.constant).collect()
    }

    /// Get the constant.
    pub fn constant(&self) -> f64 {
        self.constant
    }

    /// Set a new constant.
    pub fn set_constant(&mut self, constant: f64) {
        self.constant = constant;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_multiply_basic() {
        let a = vec![Complex64::new(2.0, 0.0), Complex64::new(3.0, 0.0)];
        let b = vec![Complex64::new(4.0, 0.0), Complex64::new(5.0, 0.0)];
        let result = Multiply::process(&a, &b);
        assert!((result[0].re - 8.0).abs() < 1e-10);
        assert!((result[1].re - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_complex() {
        // (1+2j) * (3+4j) = (1*3 - 2*4) + (1*4 + 2*3)j = -5 + 10j
        let a = vec![Complex64::new(1.0, 2.0)];
        let b = vec![Complex64::new(3.0, 4.0)];
        let result = Multiply::process(&a, &b);
        assert!((result[0].re - (-5.0)).abs() < 1e-10);
        assert!((result[0].im - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_different_lengths() {
        let a = vec![Complex64::new(1.0, 0.0); 5];
        let b = vec![Complex64::new(2.0, 0.0); 3];
        let result = Multiply::process(&a, &b);
        assert_eq!(result.len(), 3); // min(5, 3)
    }

    #[test]
    fn test_multiply_real() {
        let result = Multiply::process_real(&[1.0, 2.0, 3.0], &[4.0, 5.0, 6.0]);
        assert!((result[0] - 4.0).abs() < 1e-10);
        assert!((result[1] - 10.0).abs() < 1e-10);
        assert!((result[2] - 18.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_const_gain() {
        let mc = MultiplyConst::from_real(3.0);
        let input = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let output = mc.process_block(&input);
        assert!((output[0].re - 3.0).abs() < 1e-10);
        assert!((output[0].im - 6.0).abs() < 1e-10);
        assert!((output[1].re - 9.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_const_rotation() {
        // Multiply by j rotates 90 degrees
        let mc = MultiplyConst::new(Complex64::new(0.0, 1.0));
        let input = vec![Complex64::new(1.0, 0.0)];
        let output = mc.process_block(&input);
        assert!((output[0].re - 0.0).abs() < 1e-10);
        assert!((output[0].im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_as_mixer() {
        // Frequency shift: multiply by exp(j*2*pi*f*t)
        let n = 100;
        let f_shift = 0.1; // normalized frequency
        let lo: Vec<Complex64> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * f_shift * i as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        let dc = vec![Complex64::new(1.0, 0.0); n];
        let shifted = Multiply::process(&dc, &lo);
        // Output should be a tone at f_shift, first sample = 1+0j
        assert!((shifted[0].re - 1.0).abs() < 1e-10);
        // At sample 5 (phase = pi), should have negative real
        let at_5 = 2.0 * PI * f_shift * 5.0;
        assert!((shifted[5].re - at_5.cos()).abs() < 1e-10);
        assert!((shifted[5].im - at_5.sin()).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_const_real() {
        let mc = MultiplyConstReal::new(0.5);
        let output = mc.process_block(&[2.0, 4.0, 6.0]);
        assert!((output[0] - 1.0).abs() < 1e-10);
        assert!((output[1] - 2.0).abs() < 1e-10);
        assert!((output[2] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_const_set_constant() {
        let mut mc = MultiplyConst::from_real(2.0);
        assert!((mc.constant().re - 2.0).abs() < 1e-10);
        mc.set_constant(Complex64::new(0.0, 1.0));
        assert!((mc.constant().im - 1.0).abs() < 1e-10);
    }
}

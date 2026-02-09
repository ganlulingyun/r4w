//! Arithmetic Operations on Sample Streams
//!
//! Element-wise arithmetic blocks for combining, offsetting, and scaling signals.
//! Supports both complex and real-valued streams.
//!
//! ## Blocks
//!
//! - **Add**: Sum two or more streams element-wise
//! - **Subtract**: Difference of two streams
//! - **Divide**: Element-wise division with zero protection
//! - **AddConst**: Add a constant to every sample
//! - **SubtractConst**: Subtract a constant from every sample
//! - **DivideConst**: Divide every sample by a constant
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::arithmetic::{add_complex, add_const_complex, divide_safe};
//! use num_complex::Complex64;
//!
//! let a = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
//! let b = vec![Complex64::new(0.5, 0.5), Complex64::new(1.0, 1.0)];
//! let sum = add_complex(&a, &b);
//! assert_eq!(sum[0], Complex64::new(1.5, 2.5));
//! ```

use num_complex::Complex64;

// ── Complex stream operations ──────────────────────────────────────────

/// Add two complex streams element-wise.
pub fn add_complex(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x + y).collect()
}

/// Add multiple complex streams element-wise.
pub fn add_multi_complex(streams: &[&[Complex64]]) -> Vec<Complex64> {
    if streams.is_empty() {
        return Vec::new();
    }
    let len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    (0..len)
        .map(|i| streams.iter().map(|s| s[i]).sum())
        .collect()
}

/// Subtract two complex streams: a - b.
pub fn subtract_complex(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x - y).collect()
}

/// Divide two complex streams with zero protection.
pub fn divide_complex(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    a.iter()
        .zip(b.iter())
        .map(|(&x, &y)| {
            if y.norm_sqr() < 1e-30 {
                Complex64::new(0.0, 0.0)
            } else {
                x / y
            }
        })
        .collect()
}

/// Add a complex constant to every sample.
pub fn add_const_complex(input: &[Complex64], c: Complex64) -> Vec<Complex64> {
    input.iter().map(|&x| x + c).collect()
}

/// Subtract a complex constant from every sample.
pub fn subtract_const_complex(input: &[Complex64], c: Complex64) -> Vec<Complex64> {
    input.iter().map(|&x| x - c).collect()
}

/// Divide every sample by a complex constant.
pub fn divide_const_complex(input: &[Complex64], c: Complex64) -> Vec<Complex64> {
    if c.norm_sqr() < 1e-30 {
        vec![Complex64::new(0.0, 0.0); input.len()]
    } else {
        input.iter().map(|&x| x / c).collect()
    }
}

// ── Real stream operations ─────────────────────────────────────────────

/// Add two real streams element-wise.
pub fn add_real(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x + y).collect()
}

/// Subtract two real streams: a - b.
pub fn subtract_real(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x - y).collect()
}

/// Divide two real streams with zero protection.
pub fn divide_safe(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter()
        .zip(b.iter())
        .map(|(&x, &y)| if y.abs() < 1e-30 { 0.0 } else { x / y })
        .collect()
}

/// Add a real constant to every sample.
pub fn add_const_real(input: &[f64], c: f64) -> Vec<f64> {
    input.iter().map(|&x| x + c).collect()
}

/// Subtract a real constant from every sample.
pub fn subtract_const_real(input: &[f64], c: f64) -> Vec<f64> {
    input.iter().map(|&x| x - c).collect()
}

/// Divide every sample by a real constant.
pub fn divide_const_real(input: &[f64], c: f64) -> Vec<f64> {
    if c.abs() < 1e-30 {
        vec![0.0; input.len()]
    } else {
        input.iter().map(|&x| x / c).collect()
    }
}

// ── Stateful block wrappers ────────────────────────────────────────────

/// Block-style adder that can accumulate streams.
#[derive(Debug, Clone)]
pub struct Adder {
    num_inputs: usize,
}

impl Adder {
    /// Create an adder for `num_inputs` streams.
    pub fn new(num_inputs: usize) -> Self {
        assert!(num_inputs >= 2);
        Self { num_inputs }
    }

    /// Process multiple streams (all must be the same length).
    pub fn process(&self, streams: &[&[Complex64]]) -> Vec<Complex64> {
        assert_eq!(streams.len(), self.num_inputs);
        add_multi_complex(streams)
    }

    /// Number of input streams.
    pub fn num_inputs(&self) -> usize {
        self.num_inputs
    }
}

/// Block-style constant adder.
#[derive(Debug, Clone)]
pub struct AddConstBlock {
    constant: Complex64,
}

impl AddConstBlock {
    pub fn new(constant: Complex64) -> Self {
        Self { constant }
    }

    pub fn new_real(c: f64) -> Self {
        Self {
            constant: Complex64::new(c, 0.0),
        }
    }

    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        add_const_complex(input, self.constant)
    }

    pub fn constant(&self) -> Complex64 {
        self.constant
    }

    pub fn set_constant(&mut self, c: Complex64) {
        self.constant = c;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_complex() {
        let a = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let b = vec![Complex64::new(0.5, 0.5), Complex64::new(1.0, 1.0)];
        let result = add_complex(&a, &b);
        assert_eq!(result[0], Complex64::new(1.5, 2.5));
        assert_eq!(result[1], Complex64::new(4.0, 5.0));
    }

    #[test]
    fn test_add_multi_complex() {
        let a = vec![Complex64::new(1.0, 0.0); 3];
        let b = vec![Complex64::new(2.0, 0.0); 3];
        let c = vec![Complex64::new(3.0, 0.0); 3];
        let result = add_multi_complex(&[&a, &b, &c]);
        assert_eq!(result.len(), 3);
        assert!((result[0].re - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_subtract_complex() {
        let a = vec![Complex64::new(5.0, 3.0)];
        let b = vec![Complex64::new(2.0, 1.0)];
        let result = subtract_complex(&a, &b);
        assert_eq!(result[0], Complex64::new(3.0, 2.0));
    }

    #[test]
    fn test_divide_complex_safe() {
        let a = vec![Complex64::new(4.0, 2.0)];
        let b = vec![Complex64::new(2.0, 0.0)];
        let result = divide_complex(&a, &b);
        assert!((result[0].re - 2.0).abs() < 1e-10);
        assert!((result[0].im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_divide_by_zero_complex() {
        let a = vec![Complex64::new(1.0, 1.0)];
        let b = vec![Complex64::new(0.0, 0.0)];
        let result = divide_complex(&a, &b);
        assert_eq!(result[0], Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_add_const_complex() {
        let input = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let result = add_const_complex(&input, Complex64::new(10.0, 0.0));
        assert_eq!(result[0], Complex64::new(11.0, 2.0));
    }

    #[test]
    fn test_subtract_const_complex() {
        let input = vec![Complex64::new(10.0, 5.0)];
        let result = subtract_const_complex(&input, Complex64::new(3.0, 2.0));
        assert_eq!(result[0], Complex64::new(7.0, 3.0));
    }

    #[test]
    fn test_divide_const_complex() {
        let input = vec![Complex64::new(4.0, 6.0)];
        let result = divide_const_complex(&input, Complex64::new(2.0, 0.0));
        assert!((result[0].re - 2.0).abs() < 1e-10);
        assert!((result[0].im - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_divide_const_zero() {
        let input = vec![Complex64::new(1.0, 1.0)];
        let result = divide_const_complex(&input, Complex64::new(0.0, 0.0));
        assert_eq!(result[0], Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_add_real() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![0.5, 0.5, 0.5];
        let result = add_real(&a, &b);
        assert_eq!(result, vec![1.5, 2.5, 3.5]);
    }

    #[test]
    fn test_subtract_real() {
        let result = subtract_real(&[10.0, 20.0], &[3.0, 5.0]);
        assert_eq!(result, vec![7.0, 15.0]);
    }

    #[test]
    fn test_divide_safe_real() {
        let result = divide_safe(&[10.0, 5.0, 3.0], &[2.0, 0.0, 1.0]);
        assert!((result[0] - 5.0).abs() < 1e-10);
        assert_eq!(result[1], 0.0); // zero denominator → 0
        assert!((result[2] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_add_const_real() {
        let result = add_const_real(&[1.0, 2.0], 10.0);
        assert_eq!(result, vec![11.0, 12.0]);
    }

    #[test]
    fn test_subtract_const_real() {
        let result = subtract_const_real(&[10.0, 20.0], 3.0);
        assert_eq!(result, vec![7.0, 17.0]);
    }

    #[test]
    fn test_divide_const_real() {
        let result = divide_const_real(&[10.0, 20.0], 5.0);
        assert_eq!(result, vec![2.0, 4.0]);
    }

    #[test]
    fn test_divide_const_real_zero() {
        let result = divide_const_real(&[1.0, 2.0], 0.0);
        assert_eq!(result, vec![0.0, 0.0]);
    }

    #[test]
    fn test_adder_block() {
        let adder = Adder::new(3);
        let a = vec![Complex64::new(1.0, 0.0); 4];
        let b = vec![Complex64::new(2.0, 0.0); 4];
        let c = vec![Complex64::new(3.0, 0.0); 4];
        let result = adder.process(&[&a, &b, &c]);
        assert_eq!(result.len(), 4);
        assert!((result[0].re - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_add_const_block() {
        let block = AddConstBlock::new_real(5.0);
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)];
        let result = block.process(&input);
        assert!((result[0].re - 6.0).abs() < 1e-10);
        assert!((result[1].re - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_empty_input() {
        assert!(add_complex(&[], &[]).is_empty());
        assert!(add_real(&[], &[]).is_empty());
        assert!(add_const_complex(&[], Complex64::new(1.0, 0.0)).is_empty());
    }

    #[test]
    fn test_mismatched_lengths() {
        // Shorter length used
        let a = vec![Complex64::new(1.0, 0.0); 5];
        let b = vec![Complex64::new(2.0, 0.0); 3];
        let result = add_complex(&a, &b);
        assert_eq!(result.len(), 3);
    }
}

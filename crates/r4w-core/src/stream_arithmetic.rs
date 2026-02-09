//! Stream Arithmetic — Element-wise operations between multiple streams
//!
//! Add, subtract, multiply, and divide element-wise between two or more
//! complex IQ streams. Essential for combining signal + noise, cancelling
//! interference, and beamforming operations.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_arithmetic::{StreamAdd, StreamSubtract};
//! use num_complex::Complex64;
//!
//! let a: Vec<Complex64> = vec![Complex64::new(1.0, 2.0); 10];
//! let b: Vec<Complex64> = vec![Complex64::new(0.5, -0.5); 10];
//! let sum = StreamAdd::process(&a, &b);
//! assert_eq!(sum[0], Complex64::new(1.5, 1.5));
//!
//! let diff = StreamSubtract::process(&a, &b);
//! assert_eq!(diff[0], Complex64::new(0.5, 2.5));
//! ```

use num_complex::Complex64;

/// Element-wise addition of two complex streams.
pub struct StreamAdd;

impl StreamAdd {
    /// Add two streams element-wise. Output length = min(a.len(), b.len()).
    pub fn process(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x + y).collect()
    }

    /// Add N streams element-wise.
    pub fn process_multi(streams: &[&[Complex64]]) -> Vec<Complex64> {
        if streams.is_empty() {
            return Vec::new();
        }
        let min_len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
        (0..min_len)
            .map(|i| streams.iter().map(|s| s[i]).sum())
            .collect()
    }
}

/// Element-wise subtraction of two complex streams.
pub struct StreamSubtract;

impl StreamSubtract {
    /// Subtract b from a element-wise.
    pub fn process(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x - y).collect()
    }
}

/// Element-wise multiplication of two complex streams.
pub struct StreamMultiply;

impl StreamMultiply {
    /// Multiply two streams element-wise (complex multiply).
    pub fn process(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x * y).collect()
    }
}

/// Element-wise division of two complex streams.
pub struct StreamDivide;

impl StreamDivide {
    /// Divide a by b element-wise. Returns zero where b is near zero.
    pub fn process(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
        a.iter()
            .zip(b.iter())
            .map(|(&x, &y)| {
                if y.norm() > 1e-30 {
                    x / y
                } else {
                    Complex64::new(0.0, 0.0)
                }
            })
            .collect()
    }
}

/// Real-valued stream arithmetic.
pub struct RealStreamAdd;

impl RealStreamAdd {
    pub fn process(a: &[f64], b: &[f64]) -> Vec<f64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x + y).collect()
    }
}

pub struct RealStreamSubtract;

impl RealStreamSubtract {
    pub fn process(a: &[f64], b: &[f64]) -> Vec<f64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x - y).collect()
    }
}

pub struct RealStreamMultiply;

impl RealStreamMultiply {
    pub fn process(a: &[f64], b: &[f64]) -> Vec<f64> {
        a.iter().zip(b.iter()).map(|(&x, &y)| x * y).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        let a = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let b = vec![Complex64::new(0.5, -0.5), Complex64::new(1.0, 1.0)];
        let result = StreamAdd::process(&a, &b);
        assert_eq!(result.len(), 2);
        assert!((result[0] - Complex64::new(1.5, 1.5)).norm() < 1e-10);
        assert!((result[1] - Complex64::new(4.0, 5.0)).norm() < 1e-10);
    }

    #[test]
    fn test_subtract() {
        let a = vec![Complex64::new(3.0, 4.0)];
        let b = vec![Complex64::new(1.0, 1.0)];
        let result = StreamSubtract::process(&a, &b);
        assert!((result[0] - Complex64::new(2.0, 3.0)).norm() < 1e-10);
    }

    #[test]
    fn test_multiply() {
        let a = vec![Complex64::new(2.0, 3.0)];
        let b = vec![Complex64::new(4.0, -1.0)];
        let result = StreamMultiply::process(&a, &b);
        // (2+3j)(4-1j) = 8-2j+12j-3j² = 8+10j+3 = 11+10j
        assert!((result[0] - Complex64::new(11.0, 10.0)).norm() < 1e-10);
    }

    #[test]
    fn test_divide() {
        let a = vec![Complex64::new(4.0, 2.0)];
        let b = vec![Complex64::new(2.0, 0.0)];
        let result = StreamDivide::process(&a, &b);
        assert!((result[0] - Complex64::new(2.0, 1.0)).norm() < 1e-10);
    }

    #[test]
    fn test_divide_by_zero() {
        let a = vec![Complex64::new(1.0, 0.0)];
        let b = vec![Complex64::new(0.0, 0.0)];
        let result = StreamDivide::process(&a, &b);
        assert_eq!(result[0], Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_add_multi() {
        let a = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)];
        let b = vec![Complex64::new(3.0, 0.0), Complex64::new(4.0, 0.0)];
        let c = vec![Complex64::new(5.0, 0.0), Complex64::new(6.0, 0.0)];
        let result = StreamAdd::process_multi(&[&a, &b, &c]);
        assert!((result[0] - Complex64::new(9.0, 0.0)).norm() < 1e-10);
        assert!((result[1] - Complex64::new(12.0, 0.0)).norm() < 1e-10);
    }

    #[test]
    fn test_different_lengths() {
        let a = vec![Complex64::new(1.0, 0.0); 5];
        let b = vec![Complex64::new(2.0, 0.0); 3];
        let result = StreamAdd::process(&a, &b);
        assert_eq!(result.len(), 3); // Truncated to shorter
    }

    #[test]
    fn test_empty_streams() {
        let result = StreamAdd::process(&[], &[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_real_add() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![4.0, 5.0, 6.0];
        let result = RealStreamAdd::process(&a, &b);
        assert_eq!(result, vec![5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_real_subtract() {
        let a = vec![5.0, 10.0];
        let b = vec![3.0, 4.0];
        let result = RealStreamSubtract::process(&a, &b);
        assert_eq!(result, vec![2.0, 6.0]);
    }

    #[test]
    fn test_real_multiply() {
        let a = vec![2.0, 3.0];
        let b = vec![4.0, 5.0];
        let result = RealStreamMultiply::process(&a, &b);
        assert_eq!(result, vec![8.0, 15.0]);
    }
}

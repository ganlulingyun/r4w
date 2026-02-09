//! Complex Conjugate Operations
//!
//! Conjugate and multiply-conjugate blocks for phase reversal, cross-correlation,
//! and decision-directed synchronization loops.
//!
//! ## Blocks
//!
//! - **Conjugate**: `y[n] = conj(x[n])` — flips imaginary component
//! - **MultiplyConjugate**: `y[n] = a[n] * conj(b[n])` — used in cross-correlation
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::conjugate::{conjugate, multiply_conjugate};
//! use num_complex::Complex64;
//!
//! let x = vec![Complex64::new(3.0, 4.0)];
//! let conj = conjugate(&x);
//! assert_eq!(conj[0], Complex64::new(3.0, -4.0));
//!
//! let a = vec![Complex64::new(1.0, 2.0)];
//! let b = vec![Complex64::new(3.0, 4.0)];
//! let mc = multiply_conjugate(&a, &b);
//! // (1+2j) * (3-4j) = 3-4j+6j-8j^2 = 11+2j
//! assert!((mc[0].re - 11.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Compute complex conjugate of each sample.
pub fn conjugate(input: &[Complex64]) -> Vec<Complex64> {
    input.iter().map(|z| z.conj()).collect()
}

/// Multiply first stream by conjugate of second: a[n] * conj(b[n]).
pub fn multiply_conjugate(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    a.iter()
        .zip(b.iter())
        .map(|(&x, &y)| x * y.conj())
        .collect()
}

/// In-place conjugate.
pub fn conjugate_inplace(data: &mut [Complex64]) {
    for z in data.iter_mut() {
        *z = z.conj();
    }
}

/// Block-style conjugate processor.
#[derive(Debug, Clone, Default)]
pub struct ConjugateBlock;

impl ConjugateBlock {
    pub fn new() -> Self {
        Self
    }

    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        conjugate(input)
    }
}

/// Block-style multiply-conjugate processor.
#[derive(Debug, Clone, Default)]
pub struct MultiplyConjugateBlock;

impl MultiplyConjugateBlock {
    pub fn new() -> Self {
        Self
    }

    /// Process two aligned streams: output[n] = a[n] * conj(b[n]).
    pub fn process(&self, a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
        multiply_conjugate(a, b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conjugate() {
        let input = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(-3.0, 4.0),
            Complex64::new(0.0, -1.0),
        ];
        let result = conjugate(&input);
        assert_eq!(result[0], Complex64::new(1.0, -2.0));
        assert_eq!(result[1], Complex64::new(-3.0, -4.0));
        assert_eq!(result[2], Complex64::new(0.0, 1.0));
    }

    #[test]
    fn test_conjugate_real() {
        // Real signal: conjugate is identity
        let input = vec![Complex64::new(5.0, 0.0)];
        let result = conjugate(&input);
        assert_eq!(result[0], Complex64::new(5.0, 0.0));
    }

    #[test]
    fn test_conjugate_inplace() {
        let mut data = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, -4.0)];
        conjugate_inplace(&mut data);
        assert_eq!(data[0], Complex64::new(1.0, -2.0));
        assert_eq!(data[1], Complex64::new(3.0, 4.0));
    }

    #[test]
    fn test_multiply_conjugate() {
        let a = vec![Complex64::new(1.0, 2.0)];
        let b = vec![Complex64::new(3.0, 4.0)];
        let result = multiply_conjugate(&a, &b);
        // (1+2j) * (3-4j) = 3 - 4j + 6j - 8j^2 = 3 + 2j + 8 = 11 + 2j
        assert!((result[0].re - 11.0).abs() < 1e-10);
        assert!((result[0].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_conjugate_autocorrelation() {
        // x * conj(x) = |x|^2 (purely real)
        let x = vec![Complex64::new(3.0, 4.0)];
        let result = multiply_conjugate(&x, &x);
        assert!((result[0].re - 25.0).abs() < 1e-10); // 3^2 + 4^2
        assert!(result[0].im.abs() < 1e-10); // should be zero
    }

    #[test]
    fn test_multiply_conjugate_commutative_check() {
        let a = vec![Complex64::new(2.0, 3.0)];
        let b = vec![Complex64::new(4.0, 1.0)];
        let ab = multiply_conjugate(&a, &b);
        let ba = multiply_conjugate(&b, &a);
        // a * conj(b) != b * conj(a) in general, but conj(a*conj(b)) = b*conj(a)
        assert!((ab[0].conj() - ba[0]).norm() < 1e-10);
    }

    #[test]
    fn test_empty_inputs() {
        assert!(conjugate(&[]).is_empty());
        assert!(multiply_conjugate(&[], &[]).is_empty());
    }

    #[test]
    fn test_mismatched_lengths() {
        let a = vec![Complex64::new(1.0, 0.0); 5];
        let b = vec![Complex64::new(1.0, 0.0); 3];
        let result = multiply_conjugate(&a, &b);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn test_conjugate_block() {
        let block = ConjugateBlock::new();
        let input = vec![Complex64::new(1.0, 2.0)];
        let result = block.process(&input);
        assert_eq!(result[0], Complex64::new(1.0, -2.0));
    }

    #[test]
    fn test_multiply_conjugate_block() {
        let block = MultiplyConjugateBlock::new();
        let a = vec![Complex64::new(1.0, 1.0)];
        let b = vec![Complex64::new(1.0, 1.0)];
        let result = block.process(&a, &b);
        // (1+j)*(1-j) = 1 - j + j - j^2 = 2
        assert!((result[0].re - 2.0).abs() < 1e-10);
        assert!(result[0].im.abs() < 1e-10);
    }
}

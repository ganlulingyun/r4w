//! Sample Operations — Keep-One-in-N, Repeat
//!
//! Simple decimation-without-filtering and sample repetition blocks.
//!
//! - `KeepOneInN`: Keep every Nth sample, discard the rest (no anti-alias filtering)
//! - `Repeat`: Replicate each sample N times (no interpolation filtering)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sample_ops::{KeepOneInN, Repeat};
//! use num_complex::Complex64;
//!
//! let mut keep = KeepOneInN::new(3);
//! let input = vec![Complex64::new(1.0, 0.0); 9];
//! let output = keep.process_block(&input);
//! assert_eq!(output.len(), 3);
//!
//! let repeat = Repeat::new(4);
//! let input = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)];
//! let output = repeat.process_block(&input);
//! assert_eq!(output.len(), 8);
//! ```

use num_complex::Complex64;

/// Keep one sample in every N, discarding the rest.
///
/// This is decimation without anti-alias filtering. Use when the signal
/// is already bandwidth-limited (e.g., after a lowpass filter).
#[derive(Debug, Clone)]
pub struct KeepOneInN {
    n: usize,
    /// Current position within the N-sample cycle
    count: usize,
}

impl KeepOneInN {
    /// Create with decimation factor N.
    pub fn new(n: usize) -> Self {
        assert!(n > 0, "N must be positive");
        Self { n, count: 0 }
    }

    /// Process a single sample. Returns Some if this is the kept sample.
    pub fn process(&mut self, input: Complex64) -> Option<Complex64> {
        let result = if self.count == 0 {
            Some(input)
        } else {
            None
        };
        self.count = (self.count + 1) % self.n;
        result
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.n + 1);
        for &s in input {
            if let Some(kept) = self.process(s) {
                output.push(kept);
            }
        }
        output
    }

    /// Process real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.n + 1);
        for &s in input {
            if self.count == 0 {
                output.push(s);
            }
            self.count = (self.count + 1) % self.n;
        }
        output
    }

    /// Get the decimation factor.
    pub fn n(&self) -> usize {
        self.n
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.count = 0;
    }
}

/// Replicate each input sample N times.
///
/// This is interpolation without anti-image filtering. Use when the signal
/// will be filtered afterwards (e.g., by a pulse shaping filter).
#[derive(Debug, Clone)]
pub struct Repeat {
    n: usize,
}

impl Repeat {
    /// Create with repetition factor N.
    pub fn new(n: usize) -> Self {
        assert!(n > 0, "N must be positive");
        Self { n }
    }

    /// Process a block of samples, repeating each N times.
    pub fn process_block(&self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() * self.n);
        for &s in input {
            for _ in 0..self.n {
                output.push(s);
            }
        }
        output
    }

    /// Process real samples.
    pub fn process_real(&self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() * self.n);
        for &s in input {
            for _ in 0..self.n {
                output.push(s);
            }
        }
        output
    }

    /// Get the repetition factor.
    pub fn n(&self) -> usize {
        self.n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_keep_one_in_n_basic() {
        let mut k = KeepOneInN::new(3);
        let input: Vec<Complex64> = (0..9)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = k.process_block(&input);
        assert_eq!(output.len(), 3);
        assert!((output[0].re - 0.0).abs() < 1e-10);
        assert!((output[1].re - 3.0).abs() < 1e-10);
        assert!((output[2].re - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_keep_one_in_n_factor_1() {
        let mut k = KeepOneInN::new(1);
        let input = vec![Complex64::new(1.0, 0.0); 5];
        let output = k.process_block(&input);
        assert_eq!(output.len(), 5); // No decimation
    }

    #[test]
    fn test_keep_one_in_n_sample_by_sample() {
        let mut k = KeepOneInN::new(2);
        assert!(k.process(Complex64::new(1.0, 0.0)).is_some());
        assert!(k.process(Complex64::new(2.0, 0.0)).is_none());
        assert!(k.process(Complex64::new(3.0, 0.0)).is_some());
    }

    #[test]
    fn test_keep_one_in_n_real() {
        let mut k = KeepOneInN::new(2);
        let output = k.process_real(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert_eq!(output, vec![1.0, 3.0, 5.0]);
    }

    #[test]
    fn test_keep_one_in_n_reset() {
        let mut k = KeepOneInN::new(3);
        k.process(Complex64::new(1.0, 0.0));
        k.process(Complex64::new(2.0, 0.0));
        // count is now 2 (would skip next)
        k.reset();
        // After reset, next sample should be kept
        assert!(k.process(Complex64::new(3.0, 0.0)).is_some());
    }

    #[test]
    fn test_repeat_basic() {
        let r = Repeat::new(3);
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
        ];
        let output = r.process_block(&input);
        assert_eq!(output.len(), 6);
        for i in 0..3 {
            assert!((output[i].re - 1.0).abs() < 1e-10);
        }
        for i in 3..6 {
            assert!((output[i].re - 2.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_repeat_factor_1() {
        let r = Repeat::new(1);
        let input = vec![Complex64::new(1.0, 0.0); 5];
        let output = r.process_block(&input);
        assert_eq!(output.len(), 5);
    }

    #[test]
    fn test_repeat_real() {
        let r = Repeat::new(2);
        let output = r.process_real(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![1.0, 1.0, 2.0, 2.0, 3.0, 3.0]);
    }

    #[test]
    fn test_keep_repeat_inverse() {
        // Repeat 4x then keep-one-in-4 should recover original
        let r = Repeat::new(4);
        let mut k = KeepOneInN::new(4);

        let original = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
        ];
        let expanded = r.process_block(&original);
        let recovered = k.process_block(&expanded);

        assert_eq!(recovered.len(), 3);
        for (orig, rec) in original.iter().zip(recovered.iter()) {
            assert!((orig.re - rec.re).abs() < 1e-10);
        }
    }
}

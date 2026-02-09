//! Repeat & Keep-One-in-N — Rate-change utility blocks
//!
//! Simple rate change operations for signal processing pipelines.
//! `Repeat` duplicates each sample N times (upsampling without filtering).
//! `KeepOneInN` keeps every Nth sample (downsampling without filtering).
//! GNU Radio equivalents: `repeat`, `keep_one_in_n`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::repeat::{Repeat, KeepOneInN};
//!
//! // Repeat each sample 3 times
//! let rep = Repeat::new(3);
//! let input = vec![1.0, 2.0, 3.0];
//! let output = rep.process(&input);
//! assert_eq!(output, vec![1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0]);
//!
//! // Keep every 4th sample
//! let keep = KeepOneInN::new(4);
//! let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
//! let output = keep.process(&input);
//! assert_eq!(output, vec![1.0, 5.0]);
//! ```

/// Repeat — duplicates each input sample N times.
///
/// This is zero-order hold upsampling. No anti-imaging filter is applied.
/// For filtered interpolation, use `PolyphaseInterpolator` instead.
#[derive(Debug, Clone)]
pub struct Repeat {
    /// Repetition factor.
    n: usize,
}

impl Repeat {
    /// Create a repeater with factor `n`.
    ///
    /// If `n == 0`, it is set to 1 (passthrough).
    pub fn new(n: usize) -> Self {
        Self { n: n.max(1) }
    }

    /// Process a block of f64 samples.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() * self.n);
        for &x in input {
            for _ in 0..self.n {
                output.push(x);
            }
        }
        output
    }

    /// Process a block of complex samples.
    pub fn process_complex(
        &self,
        input: &[num_complex::Complex64],
    ) -> Vec<num_complex::Complex64> {
        let mut output = Vec::with_capacity(input.len() * self.n);
        for &x in input {
            for _ in 0..self.n {
                output.push(x);
            }
        }
        output
    }

    /// Process generic: repeat bytes.
    pub fn process_bytes(&self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len() * self.n);
        for &x in input {
            for _ in 0..self.n {
                output.push(x);
            }
        }
        output
    }

    /// Get the repetition factor.
    pub fn n(&self) -> usize {
        self.n
    }

    /// Set the repetition factor.
    pub fn set_n(&mut self, n: usize) {
        self.n = n.max(1);
    }

    /// Output length for given input length.
    pub fn output_len(&self, input_len: usize) -> usize {
        input_len * self.n
    }
}

/// Keep-One-in-N — decimation without filtering.
///
/// Keeps every Nth sample, discarding the rest. Starting offset is
/// configurable. For filtered decimation, use `PolyphaseDecimator` or
/// `DecimatingFir`.
#[derive(Debug, Clone)]
pub struct KeepOneInN {
    /// Decimation factor.
    n: usize,
    /// Current index within the N-sample block.
    index: usize,
    /// Which sample within each block to keep (0-based).
    offset: usize,
}

impl KeepOneInN {
    /// Create with decimation factor `n`. Keeps sample 0 of each block.
    pub fn new(n: usize) -> Self {
        Self {
            n: n.max(1),
            index: 0,
            offset: 0,
        }
    }

    /// Create with decimation factor and offset within the block.
    pub fn with_offset(n: usize, offset: usize) -> Self {
        let n = n.max(1);
        Self {
            n,
            index: 0,
            offset: offset % n,
        }
    }

    /// Process a block of f64 samples.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.n + 1);
        let mut idx = self.offset;
        while idx < input.len() {
            output.push(input[idx]);
            idx += self.n;
        }
        output
    }

    /// Streaming process: maintains state across calls.
    pub fn process_streaming(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.n + 1);
        for &x in input {
            if self.index == self.offset {
                output.push(x);
            }
            self.index += 1;
            if self.index >= self.n {
                self.index = 0;
            }
        }
        output
    }

    /// Process complex samples.
    pub fn process_complex(
        &self,
        input: &[num_complex::Complex64],
    ) -> Vec<num_complex::Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.n + 1);
        let mut idx = self.offset;
        while idx < input.len() {
            output.push(input[idx]);
            idx += self.n;
        }
        output
    }

    /// Get the decimation factor.
    pub fn n(&self) -> usize {
        self.n
    }

    /// Set the decimation factor.
    pub fn set_n(&mut self, n: usize) {
        self.n = n.max(1);
        self.offset = self.offset % self.n;
        self.index = 0;
    }

    /// Get the offset.
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// Reset streaming state.
    pub fn reset(&mut self) {
        self.index = 0;
    }

    /// Output length for given input length (batch mode).
    pub fn output_len(&self, input_len: usize) -> usize {
        if input_len <= self.offset {
            0
        } else {
            (input_len - self.offset + self.n - 1) / self.n
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex64;

    // -- Repeat tests --

    #[test]
    fn test_repeat_basic() {
        let rep = Repeat::new(3);
        let output = rep.process(&[1.0, 2.0]);
        assert_eq!(output, vec![1.0, 1.0, 1.0, 2.0, 2.0, 2.0]);
    }

    #[test]
    fn test_repeat_one() {
        let rep = Repeat::new(1);
        let output = rep.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_repeat_zero_becomes_one() {
        let rep = Repeat::new(0);
        assert_eq!(rep.n(), 1);
    }

    #[test]
    fn test_repeat_empty() {
        let rep = Repeat::new(5);
        let output = rep.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_repeat_complex() {
        let rep = Repeat::new(2);
        let input = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let output = rep.process_complex(&input);
        assert_eq!(output.len(), 4);
        assert_eq!(output[0], output[1]);
        assert_eq!(output[2], output[3]);
    }

    #[test]
    fn test_repeat_bytes() {
        let rep = Repeat::new(3);
        let output = rep.process_bytes(&[0xAA, 0xBB]);
        assert_eq!(output, vec![0xAA, 0xAA, 0xAA, 0xBB, 0xBB, 0xBB]);
    }

    #[test]
    fn test_repeat_output_len() {
        let rep = Repeat::new(4);
        assert_eq!(rep.output_len(10), 40);
        assert_eq!(rep.output_len(0), 0);
    }

    // -- KeepOneInN tests --

    #[test]
    fn test_keep_one_basic() {
        let keep = KeepOneInN::new(4);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let output = keep.process(&input);
        assert_eq!(output, vec![1.0, 5.0]);
    }

    #[test]
    fn test_keep_one_with_offset() {
        let keep = KeepOneInN::with_offset(4, 2);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let output = keep.process(&input);
        assert_eq!(output, vec![3.0, 7.0]);
    }

    #[test]
    fn test_keep_one_passthrough() {
        let keep = KeepOneInN::new(1);
        let input = vec![1.0, 2.0, 3.0];
        let output = keep.process(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_keep_one_empty() {
        let keep = KeepOneInN::new(5);
        let output = keep.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_keep_one_streaming() {
        let mut keep = KeepOneInN::new(3);
        let o1 = keep.process_streaming(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        // Keeps index 0, 3: values 1.0, 4.0
        assert_eq!(o1, vec![1.0, 4.0]);
        // Continue from index 2 (5 % 3 = 2)
        let o2 = keep.process_streaming(&[6.0, 7.0, 8.0, 9.0]);
        // index resumes: 2,0,1,2 → keeps at 0: 7.0
        assert_eq!(o2, vec![7.0]);
    }

    #[test]
    fn test_keep_one_complex() {
        let keep = KeepOneInN::new(2);
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
            Complex64::new(4.0, 0.0),
        ];
        let output = keep.process_complex(&input);
        assert_eq!(output.len(), 2);
        assert_eq!(output[0].re, 1.0);
        assert_eq!(output[1].re, 3.0);
    }

    #[test]
    fn test_keep_one_output_len() {
        let keep = KeepOneInN::new(4);
        assert_eq!(keep.output_len(8), 2);
        assert_eq!(keep.output_len(9), 3);
        assert_eq!(keep.output_len(0), 0);
    }

    #[test]
    fn test_keep_one_reset() {
        let mut keep = KeepOneInN::new(3);
        keep.process_streaming(&[1.0, 2.0]);
        keep.reset();
        let output = keep.process_streaming(&[10.0, 20.0, 30.0]);
        assert_eq!(output, vec![10.0]);
    }

    #[test]
    fn test_set_n() {
        let mut keep = KeepOneInN::with_offset(4, 3);
        keep.set_n(2);
        assert_eq!(keep.n(), 2);
        assert_eq!(keep.offset(), 1); // 3 % 2 = 1
    }
}

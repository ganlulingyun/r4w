//! Keep M in N — Extract M contiguous samples from every N
//!
//! Useful for OFDM cyclic prefix removal, TDMA slot extraction,
//! preamble stripping, and frame-based windowing.
//! GNU Radio equivalent: `keep_m_in_n`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::keep_m_in_n::KeepMInN;
//!
//! let mut k = KeepMInN::new(3, 5, 0);
//! let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
//! let output = k.process(&input);
//! // Keeps first 3 of every 5: [1,2,3, 6,7,8]
//! assert_eq!(output, vec![1.0, 2.0, 3.0, 6.0, 7.0, 8.0]);
//! ```

use num_complex::Complex64;

/// Extract M contiguous samples from every N.
#[derive(Debug, Clone)]
pub struct KeepMInN {
    /// Number of samples to keep.
    m: usize,
    /// Window size (period).
    n: usize,
    /// Offset within the window where keeping starts.
    offset: usize,
    /// Current position within the N-sample window.
    count: usize,
}

impl KeepMInN {
    /// Create a new KeepMInN block.
    ///
    /// `m`: number of samples to keep per window.
    /// `n`: window size (must be >= m).
    /// `offset`: starting position within window (0-based).
    pub fn new(m: usize, n: usize, offset: usize) -> Self {
        assert!(m <= n, "m ({}) must be <= n ({})", m, n);
        assert!(offset + m <= n, "offset ({}) + m ({}) must be <= n ({})", offset, m, n);
        Self {
            m,
            n: n.max(1),
            offset,
            count: 0,
        }
    }

    /// Process real samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() * self.m / self.n + 1);
        for &x in input {
            if self.count >= self.offset && self.count < self.offset + self.m {
                output.push(x);
            }
            self.count += 1;
            if self.count >= self.n {
                self.count = 0;
            }
        }
        output
    }

    /// Process complex samples.
    pub fn process_complex(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() * self.m / self.n + 1);
        for &x in input {
            if self.count >= self.offset && self.count < self.offset + self.m {
                output.push(x);
            }
            self.count += 1;
            if self.count >= self.n {
                self.count = 0;
            }
        }
        output
    }

    /// Process byte data.
    pub fn process_bytes(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len() * self.m / self.n + 1);
        for &x in input {
            if self.count >= self.offset && self.count < self.offset + self.m {
                output.push(x);
            }
            self.count += 1;
            if self.count >= self.n {
                self.count = 0;
            }
        }
        output
    }

    /// Get M.
    pub fn m(&self) -> usize { self.m }

    /// Get N.
    pub fn n(&self) -> usize { self.n }

    /// Get offset.
    pub fn offset(&self) -> usize { self.offset }

    /// Set offset.
    pub fn set_offset(&mut self, offset: usize) {
        assert!(offset + self.m <= self.n);
        self.offset = offset;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.count = 0;
    }

    /// Get the effective rate (output/input).
    pub fn rate(&self) -> f64 {
        self.m as f64 / self.n as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic() {
        let mut k = KeepMInN::new(3, 5, 0);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let output = k.process(&input);
        assert_eq!(output, vec![1.0, 2.0, 3.0, 6.0, 7.0, 8.0]);
    }

    #[test]
    fn test_with_offset() {
        let mut k = KeepMInN::new(2, 5, 1);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let output = k.process(&input);
        // offset=1: keep indices 1,2 from each window of 5
        assert_eq!(output, vec![2.0, 3.0, 7.0, 8.0]);
    }

    #[test]
    fn test_keep_all() {
        let mut k = KeepMInN::new(5, 5, 0);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let output = k.process(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_keep_one() {
        let mut k = KeepMInN::new(1, 3, 0);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let output = k.process(&input);
        assert_eq!(output, vec![1.0, 4.0]);
    }

    #[test]
    fn test_complex() {
        let mut k = KeepMInN::new(2, 4, 0);
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
            Complex64::new(4.0, 0.0),
        ];
        let output = k.process_complex(&input);
        assert_eq!(output.len(), 2);
        assert_eq!(output[0].re, 1.0);
        assert_eq!(output[1].re, 2.0);
    }

    #[test]
    fn test_bytes() {
        let mut k = KeepMInN::new(1, 2, 0);
        let output = k.process_bytes(&[10, 20, 30, 40, 50, 60]);
        assert_eq!(output, vec![10, 30, 50]);
    }

    #[test]
    fn test_streaming() {
        let mut k = KeepMInN::new(2, 4, 0);
        let out1 = k.process(&[1.0, 2.0, 3.0]);
        let out2 = k.process(&[4.0, 5.0, 6.0, 7.0, 8.0]);
        // First call: items 0,1 kept, item 2 dropped, then partial window
        // Second call: item 3 dropped (count=3), items 4,5 kept (new window), items 6,7 dropped
        let combined: Vec<f64> = out1.into_iter().chain(out2).collect();
        assert_eq!(combined, vec![1.0, 2.0, 5.0, 6.0]);
    }

    #[test]
    fn test_rate() {
        let k = KeepMInN::new(3, 10, 0);
        assert!((k.rate() - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut k = KeepMInN::new(1, 3, 0);
        k.process(&[1.0, 2.0]); // count = 2
        k.reset();
        let output = k.process(&[10.0, 20.0, 30.0]);
        assert_eq!(output, vec![10.0]); // First of every 3
    }

    #[test]
    fn test_set_offset() {
        let mut k = KeepMInN::new(1, 4, 0);
        k.set_offset(2);
        let output = k.process(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(output, vec![3.0]);
    }

    #[test]
    fn test_empty_input() {
        let mut k = KeepMInN::new(2, 5, 0);
        let output = k.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_partial_window() {
        let mut k = KeepMInN::new(2, 5, 0);
        let output = k.process(&[1.0, 2.0, 3.0]); // Only 3 of 5
        assert_eq!(output, vec![1.0, 2.0]); // Keep first 2
    }

    #[test]
    #[should_panic]
    fn test_m_greater_than_n_panics() {
        KeepMInN::new(6, 5, 0);
    }
}

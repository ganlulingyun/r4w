//! Head & SkipHead — Stream limiting blocks
//!
//! `Head` passes only the first N items, then stops.
//! `SkipHead` discards the first N items, then passes the rest.
//! GNU Radio equivalents: `head`, `skiphead`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::head::{Head, SkipHead};
//!
//! // Take first 3 samples
//! let mut head = Head::new(3);
//! let out1 = head.process(&[1.0, 2.0]);
//! assert_eq!(out1, vec![1.0, 2.0]); // still need 1 more
//! let out2 = head.process(&[3.0, 4.0, 5.0]);
//! assert_eq!(out2, vec![3.0]); // got the 3rd, done
//! assert!(head.is_done());
//!
//! // Skip first 2 samples
//! let mut skip = SkipHead::new(2);
//! let out1 = skip.process(&[1.0, 2.0, 3.0, 4.0]);
//! assert_eq!(out1, vec![3.0, 4.0]);
//! ```

use num_complex::Complex64;

/// Head — passes only the first N items from a stream.
///
/// Useful for limiting simulation duration or capturing a fixed number
/// of samples. Stateful: tracks how many items have been passed.
#[derive(Debug, Clone)]
pub struct Head {
    /// Maximum number of items to pass.
    limit: u64,
    /// Items passed so far.
    count: u64,
}

impl Head {
    /// Create a head block that passes `limit` items.
    pub fn new(limit: u64) -> Self {
        Self { limit, count: 0 }
    }

    /// Process f64 samples. Returns only the items within the limit.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        if self.count >= self.limit {
            return Vec::new();
        }
        let remaining = (self.limit - self.count) as usize;
        let take = input.len().min(remaining);
        self.count += take as u64;
        input[..take].to_vec()
    }

    /// Process complex samples.
    pub fn process_complex(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        if self.count >= self.limit {
            return Vec::new();
        }
        let remaining = (self.limit - self.count) as usize;
        let take = input.len().min(remaining);
        self.count += take as u64;
        input[..take].to_vec()
    }

    /// Process bytes.
    pub fn process_bytes(&mut self, input: &[u8]) -> Vec<u8> {
        if self.count >= self.limit {
            return Vec::new();
        }
        let remaining = (self.limit - self.count) as usize;
        let take = input.len().min(remaining);
        self.count += take as u64;
        input[..take].to_vec()
    }

    /// Whether the limit has been reached.
    pub fn is_done(&self) -> bool {
        self.count >= self.limit
    }

    /// Items passed so far.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Get the limit.
    pub fn limit(&self) -> u64 {
        self.limit
    }

    /// Remaining items before done.
    pub fn remaining(&self) -> u64 {
        self.limit.saturating_sub(self.count)
    }

    /// Reset the counter to start over.
    pub fn reset(&mut self) {
        self.count = 0;
    }

    /// Set a new limit and reset.
    pub fn set_limit(&mut self, limit: u64) {
        self.limit = limit;
        self.count = 0;
    }
}

/// SkipHead — discards the first N items, then passes the rest.
///
/// Useful for skipping transients, preambles, or warm-up periods.
#[derive(Debug, Clone)]
pub struct SkipHead {
    /// Number of items to skip.
    skip: u64,
    /// Items skipped so far.
    skipped: u64,
}

impl SkipHead {
    /// Create a skip-head that discards the first `skip` items.
    pub fn new(skip: u64) -> Self {
        Self { skip, skipped: 0 }
    }

    /// Process f64 samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        if self.skipped >= self.skip {
            return input.to_vec();
        }
        let to_skip = (self.skip - self.skipped) as usize;
        if to_skip >= input.len() {
            self.skipped += input.len() as u64;
            return Vec::new();
        }
        self.skipped = self.skip;
        input[to_skip..].to_vec()
    }

    /// Process complex samples.
    pub fn process_complex(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        if self.skipped >= self.skip {
            return input.to_vec();
        }
        let to_skip = (self.skip - self.skipped) as usize;
        if to_skip >= input.len() {
            self.skipped += input.len() as u64;
            return Vec::new();
        }
        self.skipped = self.skip;
        input[to_skip..].to_vec()
    }

    /// Process bytes.
    pub fn process_bytes(&mut self, input: &[u8]) -> Vec<u8> {
        if self.skipped >= self.skip {
            return input.to_vec();
        }
        let to_skip = (self.skip - self.skipped) as usize;
        if to_skip >= input.len() {
            self.skipped += input.len() as u64;
            return Vec::new();
        }
        self.skipped = self.skip;
        input[to_skip..].to_vec()
    }

    /// Whether all items have been skipped.
    pub fn is_active(&self) -> bool {
        self.skipped >= self.skip
    }

    /// Items skipped so far.
    pub fn skipped(&self) -> u64 {
        self.skipped
    }

    /// Total items to skip.
    pub fn skip_count(&self) -> u64 {
        self.skip
    }

    /// Remaining items to skip.
    pub fn remaining_skip(&self) -> u64 {
        self.skip.saturating_sub(self.skipped)
    }

    /// Reset the skip counter.
    pub fn reset(&mut self) {
        self.skipped = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // -- Head tests --

    #[test]
    fn test_head_basic() {
        let mut head = Head::new(5);
        let output = head.process(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
        assert_eq!(output, vec![1.0, 2.0, 3.0, 4.0, 5.0]);
        assert!(head.is_done());
    }

    #[test]
    fn test_head_streaming() {
        let mut head = Head::new(5);
        let o1 = head.process(&[1.0, 2.0, 3.0]);
        assert_eq!(o1, vec![1.0, 2.0, 3.0]);
        assert!(!head.is_done());
        assert_eq!(head.remaining(), 2);

        let o2 = head.process(&[4.0, 5.0, 6.0]);
        assert_eq!(o2, vec![4.0, 5.0]);
        assert!(head.is_done());

        let o3 = head.process(&[7.0, 8.0]);
        assert!(o3.is_empty());
    }

    #[test]
    fn test_head_exact() {
        let mut head = Head::new(3);
        let output = head.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![1.0, 2.0, 3.0]);
        assert!(head.is_done());
    }

    #[test]
    fn test_head_zero() {
        let mut head = Head::new(0);
        assert!(head.is_done());
        let output = head.process(&[1.0, 2.0]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_head_complex() {
        let mut head = Head::new(2);
        let input = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(3.0, 4.0),
            Complex64::new(5.0, 6.0),
        ];
        let output = head.process_complex(&input);
        assert_eq!(output.len(), 2);
        assert_eq!(output[0], Complex64::new(1.0, 2.0));
    }

    #[test]
    fn test_head_bytes() {
        let mut head = Head::new(3);
        let output = head.process_bytes(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(output, vec![0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn test_head_reset() {
        let mut head = Head::new(2);
        head.process(&[1.0, 2.0, 3.0]);
        assert!(head.is_done());
        head.reset();
        assert!(!head.is_done());
        let output = head.process(&[4.0, 5.0, 6.0]);
        assert_eq!(output, vec![4.0, 5.0]);
    }

    #[test]
    fn test_head_set_limit() {
        let mut head = Head::new(2);
        head.process(&[1.0, 2.0]);
        head.set_limit(10);
        assert_eq!(head.remaining(), 10);
    }

    // -- SkipHead tests --

    #[test]
    fn test_skip_basic() {
        let mut skip = SkipHead::new(3);
        let output = skip.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(output, vec![4.0, 5.0]);
        assert!(skip.is_active());
    }

    #[test]
    fn test_skip_streaming() {
        let mut skip = SkipHead::new(5);
        let o1 = skip.process(&[1.0, 2.0, 3.0]);
        assert!(o1.is_empty());
        assert!(!skip.is_active());
        assert_eq!(skip.remaining_skip(), 2);

        let o2 = skip.process(&[4.0, 5.0, 6.0, 7.0]);
        assert_eq!(o2, vec![6.0, 7.0]);
        assert!(skip.is_active());

        // After skip complete, everything passes through
        let o3 = skip.process(&[8.0, 9.0]);
        assert_eq!(o3, vec![8.0, 9.0]);
    }

    #[test]
    fn test_skip_zero() {
        let mut skip = SkipHead::new(0);
        assert!(skip.is_active());
        let output = skip.process(&[1.0, 2.0]);
        assert_eq!(output, vec![1.0, 2.0]);
    }

    #[test]
    fn test_skip_more_than_input() {
        let mut skip = SkipHead::new(100);
        let output = skip.process(&[1.0, 2.0, 3.0]);
        assert!(output.is_empty());
        assert_eq!(skip.skipped(), 3);
    }

    #[test]
    fn test_skip_complex() {
        let mut skip = SkipHead::new(1);
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)];
        let output = skip.process_complex(&input);
        assert_eq!(output.len(), 1);
        assert_eq!(output[0].re, 2.0);
    }

    #[test]
    fn test_skip_bytes() {
        let mut skip = SkipHead::new(2);
        let output = skip.process_bytes(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(output, vec![0xCC, 0xDD]);
    }

    #[test]
    fn test_skip_reset() {
        let mut skip = SkipHead::new(5);
        skip.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        assert!(skip.is_active());
        skip.reset();
        assert!(!skip.is_active());
        assert_eq!(skip.remaining_skip(), 5);
    }
}

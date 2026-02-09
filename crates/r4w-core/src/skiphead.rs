//! Skip Head — Drop first N items from a stream
//!
//! Discards the first N samples from a stream, then passes all
//! subsequent samples through unchanged. Useful for removing
//! transient startup artifacts from filters, PLLs, and AGC loops.
//! GNU Radio equivalent: `skiphead`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::skiphead::SkipHead;
//!
//! let mut skip = SkipHead::new(3);
//! let out1 = skip.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
//! assert_eq!(out1, vec![4.0, 5.0]); // First 3 skipped
//!
//! let out2 = skip.process(&[6.0, 7.0]);
//! assert_eq!(out2, vec![6.0, 7.0]); // No more skipping
//! ```

/// Skip the first N items from a stream.
#[derive(Debug, Clone)]
pub struct SkipHead {
    /// Total items to skip.
    skip_count: u64,
    /// Items skipped so far.
    skipped: u64,
}

impl SkipHead {
    /// Create a SkipHead that drops the first `n` items.
    pub fn new(n: u64) -> Self {
        Self {
            skip_count: n,
            skipped: 0,
        }
    }

    /// Process a block of samples, returning only the non-skipped portion.
    pub fn process<T: Clone>(&mut self, input: &[T]) -> Vec<T> {
        let remaining_to_skip = self.skip_count.saturating_sub(self.skipped);

        if remaining_to_skip == 0 {
            // All skipping is done — pass everything through
            return input.to_vec();
        }

        let to_skip_now = (remaining_to_skip as usize).min(input.len());
        self.skipped += to_skip_now as u64;

        if to_skip_now >= input.len() {
            // Entire block is skipped
            Vec::new()
        } else {
            input[to_skip_now..].to_vec()
        }
    }

    /// Process in-place by returning a slice of the kept portion.
    pub fn process_slice<'a, T>(&mut self, input: &'a [T]) -> &'a [T] {
        let remaining_to_skip = self.skip_count.saturating_sub(self.skipped);

        if remaining_to_skip == 0 {
            return input;
        }

        let to_skip_now = (remaining_to_skip as usize).min(input.len());
        self.skipped += to_skip_now as u64;
        &input[to_skip_now..]
    }

    /// Check if all items have been skipped (now in pass-through mode).
    pub fn done_skipping(&self) -> bool {
        self.skipped >= self.skip_count
    }

    /// Get number of items skipped so far.
    pub fn items_skipped(&self) -> u64 {
        self.skipped
    }

    /// Get total items to skip.
    pub fn skip_count(&self) -> u64 {
        self.skip_count
    }

    /// Get remaining items to skip.
    pub fn remaining(&self) -> u64 {
        self.skip_count.saturating_sub(self.skipped)
    }

    /// Reset the skip counter (start skipping again).
    pub fn reset(&mut self) {
        self.skipped = 0;
    }
}

/// Skip the first `n` items from a slice (one-shot, no state).
pub fn skip_head<T: Clone>(input: &[T], n: usize) -> Vec<T> {
    if n >= input.len() {
        Vec::new()
    } else {
        input[n..].to_vec()
    }
}

/// Skip the last `n` items from a slice.
pub fn skip_tail<T: Clone>(input: &[T], n: usize) -> Vec<T> {
    if n >= input.len() {
        Vec::new()
    } else {
        input[..input.len() - n].to_vec()
    }
}

/// Keep only items from index `start` to `end` (exclusive).
pub fn keep_range<T: Clone>(input: &[T], start: usize, end: usize) -> Vec<T> {
    let start = start.min(input.len());
    let end = end.min(input.len());
    if start >= end {
        Vec::new()
    } else {
        input[start..end].to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_skip() {
        let mut skip = SkipHead::new(3);
        let out = skip.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(out, vec![4.0, 5.0]);
    }

    #[test]
    fn test_skip_across_blocks() {
        let mut skip = SkipHead::new(5);
        let out1 = skip.process(&[1, 2, 3]);
        assert!(out1.is_empty());
        assert_eq!(skip.items_skipped(), 3);

        let out2 = skip.process(&[4, 5, 6, 7]);
        assert_eq!(out2, vec![6, 7]);
        assert!(skip.done_skipping());
    }

    #[test]
    fn test_passthrough_after_skip() {
        let mut skip = SkipHead::new(2);
        skip.process(&[1, 2, 3]);
        let out = skip.process(&[4, 5, 6]);
        assert_eq!(out, vec![4, 5, 6]);
    }

    #[test]
    fn test_skip_zero() {
        let mut skip = SkipHead::new(0);
        assert!(skip.done_skipping());
        let out = skip.process(&[1, 2, 3]);
        assert_eq!(out, vec![1, 2, 3]);
    }

    #[test]
    fn test_skip_more_than_available() {
        let mut skip = SkipHead::new(100);
        let out = skip.process(&[1, 2, 3]);
        assert!(out.is_empty());
        assert!(!skip.done_skipping());
        assert_eq!(skip.remaining(), 97);
    }

    #[test]
    fn test_process_slice() {
        let mut skip = SkipHead::new(2);
        let data = [10, 20, 30, 40, 50];
        let out = skip.process_slice(&data);
        assert_eq!(out, &[30, 40, 50]);
    }

    #[test]
    fn test_reset() {
        let mut skip = SkipHead::new(3);
        skip.process(&[1, 2, 3, 4, 5]);
        assert!(skip.done_skipping());
        skip.reset();
        assert!(!skip.done_skipping());
        assert_eq!(skip.remaining(), 3);
    }

    #[test]
    fn test_skip_head_fn() {
        assert_eq!(skip_head(&[1, 2, 3, 4, 5], 2), vec![3, 4, 5]);
        assert_eq!(skip_head(&[1, 2], 5), Vec::<i32>::new());
        assert_eq!(skip_head(&[1, 2, 3], 0), vec![1, 2, 3]);
    }

    #[test]
    fn test_skip_tail_fn() {
        assert_eq!(skip_tail(&[1, 2, 3, 4, 5], 2), vec![1, 2, 3]);
        assert_eq!(skip_tail(&[1, 2], 5), Vec::<i32>::new());
    }

    #[test]
    fn test_keep_range() {
        assert_eq!(keep_range(&[10, 20, 30, 40, 50], 1, 4), vec![20, 30, 40]);
        assert_eq!(keep_range(&[10, 20, 30], 5, 10), Vec::<i32>::new());
        assert_eq!(keep_range(&[10, 20, 30], 2, 1), Vec::<i32>::new());
    }

    #[test]
    fn test_empty_input() {
        let mut skip = SkipHead::new(5);
        assert!(skip.process::<i32>(&[]).is_empty());
    }

    #[test]
    fn test_exact_skip() {
        let mut skip = SkipHead::new(3);
        let out = skip.process(&[1, 2, 3]);
        assert!(out.is_empty());
        assert!(skip.done_skipping());
    }

    #[test]
    fn test_accessors() {
        let skip = SkipHead::new(42);
        assert_eq!(skip.skip_count(), 42);
        assert_eq!(skip.items_skipped(), 0);
        assert_eq!(skip.remaining(), 42);
    }
}

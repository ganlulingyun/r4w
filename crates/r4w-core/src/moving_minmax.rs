//! Moving Min/Max — Sliding window minimum and maximum filters
//!
//! Tracks the running minimum and/or maximum over a sliding window.
//! Uses a monotonic deque algorithm for O(1) amortized per sample.
//! Useful for envelope detection, noise floor estimation, peak tracking,
//! and adaptive thresholding.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::moving_minmax::{MovingMax, MovingMin};
//!
//! let mut max_filter = MovingMax::new(3);
//! let input = vec![1.0, 3.0, 2.0, 5.0, 4.0, 1.0];
//! let output = max_filter.process(&input);
//! // Window of 3: max of [1]=1, [1,3]=3, [1,3,2]=3, [3,2,5]=5, [2,5,4]=5, [5,4,1]=5
//! assert_eq!(output[2], 3.0); // max(1, 3, 2)
//! assert_eq!(output[3], 5.0); // max(3, 2, 5)
//! ```

use std::collections::VecDeque;

/// Sliding window maximum filter.
#[derive(Debug, Clone)]
pub struct MovingMax {
    window: usize,
    /// Monotonic deque: (index, value) pairs in decreasing value order.
    deque: VecDeque<(usize, f64)>,
    /// Current sample index.
    idx: usize,
}

impl MovingMax {
    /// Create a moving max filter with given window size.
    pub fn new(window: usize) -> Self {
        Self {
            window: window.max(1),
            deque: VecDeque::new(),
            idx: 0,
        }
    }

    /// Process a single sample, returning the max over the window.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        // Remove elements older than window
        while let Some(&(i, _)) = self.deque.front() {
            if self.idx >= self.window && i <= self.idx - self.window {
                self.deque.pop_front();
            } else {
                break;
            }
        }
        // Remove elements smaller than x from back
        while let Some(&(_, v)) = self.deque.back() {
            if v <= x {
                self.deque.pop_back();
            } else {
                break;
            }
        }
        self.deque.push_back((self.idx, x));
        self.idx += 1;
        self.deque.front().unwrap().1
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Get current maximum.
    pub fn current(&self) -> f64 {
        self.deque.front().map(|&(_, v)| v).unwrap_or(f64::NEG_INFINITY)
    }

    /// Get window size.
    pub fn window(&self) -> usize {
        self.window
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.deque.clear();
        self.idx = 0;
    }
}

/// Sliding window minimum filter.
#[derive(Debug, Clone)]
pub struct MovingMin {
    window: usize,
    deque: VecDeque<(usize, f64)>,
    idx: usize,
}

impl MovingMin {
    /// Create a moving min filter with given window size.
    pub fn new(window: usize) -> Self {
        Self {
            window: window.max(1),
            deque: VecDeque::new(),
            idx: 0,
        }
    }

    /// Process a single sample, returning the min over the window.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        while let Some(&(i, _)) = self.deque.front() {
            if self.idx >= self.window && i <= self.idx - self.window {
                self.deque.pop_front();
            } else {
                break;
            }
        }
        while let Some(&(_, v)) = self.deque.back() {
            if v >= x {
                self.deque.pop_back();
            } else {
                break;
            }
        }
        self.deque.push_back((self.idx, x));
        self.idx += 1;
        self.deque.front().unwrap().1
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Get current minimum.
    pub fn current(&self) -> f64 {
        self.deque.front().map(|&(_, v)| v).unwrap_or(f64::INFINITY)
    }

    /// Get window size.
    pub fn window(&self) -> usize {
        self.window
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.deque.clear();
        self.idx = 0;
    }
}

/// Combined min/max tracker for range/envelope.
#[derive(Debug, Clone)]
pub struct MovingMinMax {
    min_filter: MovingMin,
    max_filter: MovingMax,
}

impl MovingMinMax {
    /// Create a combined min/max tracker.
    pub fn new(window: usize) -> Self {
        Self {
            min_filter: MovingMin::new(window),
            max_filter: MovingMax::new(window),
        }
    }

    /// Process a single sample, returning (min, max).
    pub fn process_sample(&mut self, x: f64) -> (f64, f64) {
        let min = self.min_filter.process_sample(x);
        let max = self.max_filter.process_sample(x);
        (min, max)
    }

    /// Process a block, returning (min_vec, max_vec).
    pub fn process(&mut self, input: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let mut mins = Vec::with_capacity(input.len());
        let mut maxs = Vec::with_capacity(input.len());
        for &x in input {
            let (mn, mx) = self.process_sample(x);
            mins.push(mn);
            maxs.push(mx);
        }
        (mins, maxs)
    }

    /// Get current range (max - min).
    pub fn range(&self) -> f64 {
        self.max_filter.current() - self.min_filter.current()
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.min_filter.reset();
        self.max_filter.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_moving_max_basic() {
        let mut f = MovingMax::new(3);
        let out = f.process(&[1.0, 3.0, 2.0, 5.0, 4.0, 1.0]);
        assert_eq!(out[0], 1.0); // max(1) = 1
        assert_eq!(out[1], 3.0); // max(1,3) = 3
        assert_eq!(out[2], 3.0); // max(1,3,2) = 3
        assert_eq!(out[3], 5.0); // max(3,2,5) = 5
        assert_eq!(out[4], 5.0); // max(2,5,4) = 5
        assert_eq!(out[5], 5.0); // max(5,4,1) = 5
    }

    #[test]
    fn test_moving_min_basic() {
        let mut f = MovingMin::new(3);
        let out = f.process(&[5.0, 3.0, 4.0, 1.0, 2.0, 6.0]);
        assert_eq!(out[0], 5.0); // min(5) = 5
        assert_eq!(out[1], 3.0); // min(5,3) = 3
        assert_eq!(out[2], 3.0); // min(5,3,4) = 3
        assert_eq!(out[3], 1.0); // min(3,4,1) = 1
        assert_eq!(out[4], 1.0); // min(4,1,2) = 1
        assert_eq!(out[5], 1.0); // min(1,2,6) = 1
    }

    #[test]
    fn test_moving_max_window_1() {
        let mut f = MovingMax::new(1);
        let out = f.process(&[3.0, 1.0, 4.0, 1.0, 5.0]);
        assert_eq!(out, vec![3.0, 1.0, 4.0, 1.0, 5.0]); // Passthrough
    }

    #[test]
    fn test_moving_min_window_1() {
        let mut f = MovingMin::new(1);
        let out = f.process(&[3.0, 1.0, 4.0]);
        assert_eq!(out, vec![3.0, 1.0, 4.0]); // Passthrough
    }

    #[test]
    fn test_constant_signal_max() {
        let mut f = MovingMax::new(5);
        let out = f.process(&[7.0; 10]);
        for &v in &out {
            assert_eq!(v, 7.0);
        }
    }

    #[test]
    fn test_constant_signal_min() {
        let mut f = MovingMin::new(5);
        let out = f.process(&[3.0; 10]);
        for &v in &out {
            assert_eq!(v, 3.0);
        }
    }

    #[test]
    fn test_decreasing_max() {
        let mut f = MovingMax::new(3);
        let out = f.process(&[5.0, 4.0, 3.0, 2.0, 1.0]);
        assert_eq!(out[0], 5.0);
        assert_eq!(out[1], 5.0);
        assert_eq!(out[2], 5.0);
        assert_eq!(out[3], 4.0); // 5 expired, max(4,3,2) = 4
        assert_eq!(out[4], 3.0); // max(3,2,1) = 3
    }

    #[test]
    fn test_increasing_min() {
        let mut f = MovingMin::new(3);
        let out = f.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(out[0], 1.0);
        assert_eq!(out[1], 1.0);
        assert_eq!(out[2], 1.0);
        assert_eq!(out[3], 2.0); // 1 expired, min(2,3,4) = 2
        assert_eq!(out[4], 3.0); // min(3,4,5) = 3
    }

    #[test]
    fn test_minmax_combined() {
        let mut f = MovingMinMax::new(3);
        let (mins, maxs) = f.process(&[1.0, 5.0, 3.0, 7.0, 2.0]);
        assert_eq!(maxs[2], 5.0); // max(1,5,3)
        assert_eq!(mins[2], 1.0); // min(1,5,3)
        assert_eq!(maxs[3], 7.0); // max(5,3,7)
        assert_eq!(mins[3], 3.0); // min(5,3,7)
    }

    #[test]
    fn test_range() {
        let mut f = MovingMinMax::new(3);
        f.process(&[1.0, 5.0, 3.0]);
        assert!((f.range() - 4.0).abs() < 1e-10); // 5 - 1 = 4
    }

    #[test]
    fn test_reset() {
        let mut f = MovingMax::new(3);
        f.process(&[10.0, 20.0]);
        f.reset();
        let out = f.process_sample(1.0);
        assert_eq!(out, 1.0);
    }

    #[test]
    fn test_empty() {
        let mut f = MovingMax::new(3);
        assert!(f.process(&[]).is_empty());
    }

    #[test]
    fn test_streaming() {
        let mut f = MovingMax::new(2);
        assert_eq!(f.process_sample(3.0), 3.0);
        assert_eq!(f.process_sample(1.0), 3.0); // max(3,1)
        assert_eq!(f.process_sample(4.0), 4.0); // max(1,4), 3 expired
    }
}

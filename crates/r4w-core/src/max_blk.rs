//! # Max / Min Block
//!
//! Computes element-wise maximum or minimum across multiple input streams.
//! Also provides running max/min over a sliding window for single streams.
//! GNU Radio equivalent of `gr::blocks::max` and `gr::blocks::min`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::max_blk::{elementwise_max, elementwise_min, RunningMax};
//!
//! // Element-wise max across 3 streams
//! let a = vec![1.0, 5.0, 3.0];
//! let b = vec![4.0, 2.0, 6.0];
//! let c = vec![2.0, 8.0, 1.0];
//! let result = elementwise_max(&[&a, &b, &c]);
//! assert_eq!(result, vec![4.0, 8.0, 6.0]);
//!
//! // Running max over window of 3
//! let mut rm = RunningMax::new(3);
//! let output = rm.process(&[1.0, 5.0, 3.0, 2.0, 8.0]);
//! ```

use std::collections::VecDeque;

/// Compute element-wise maximum across multiple streams.
pub fn elementwise_max(streams: &[&[f64]]) -> Vec<f64> {
    if streams.is_empty() {
        return Vec::new();
    }
    let len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let mut result = Vec::with_capacity(len);
    for i in 0..len {
        let mut max_val = f64::NEG_INFINITY;
        for stream in streams {
            if stream[i] > max_val {
                max_val = stream[i];
            }
        }
        result.push(max_val);
    }
    result
}

/// Compute element-wise minimum across multiple streams.
pub fn elementwise_min(streams: &[&[f64]]) -> Vec<f64> {
    if streams.is_empty() {
        return Vec::new();
    }
    let len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let mut result = Vec::with_capacity(len);
    for i in 0..len {
        let mut min_val = f64::INFINITY;
        for stream in streams {
            if stream[i] < min_val {
                min_val = stream[i];
            }
        }
        result.push(min_val);
    }
    result
}

/// Compute element-wise max with index (which stream had the max).
pub fn elementwise_argmax(streams: &[&[f64]]) -> Vec<(f64, usize)> {
    if streams.is_empty() {
        return Vec::new();
    }
    let len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let mut result = Vec::with_capacity(len);
    for i in 0..len {
        let mut max_val = f64::NEG_INFINITY;
        let mut max_idx = 0;
        for (j, stream) in streams.iter().enumerate() {
            if stream[i] > max_val {
                max_val = stream[i];
                max_idx = j;
            }
        }
        result.push((max_val, max_idx));
    }
    result
}

/// Running maximum over a sliding window.
#[derive(Debug, Clone)]
pub struct RunningMax {
    window_size: usize,
    /// Monotone deque: front is always the max.
    deque: VecDeque<(usize, f64)>,
    index: usize,
}

impl RunningMax {
    /// Create a new running max with the given window size.
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size: window_size.max(1),
            deque: VecDeque::new(),
            index: 0,
        }
    }

    /// Push a sample and return the current window maximum.
    pub fn push(&mut self, value: f64) -> f64 {
        // Remove elements smaller than current from back.
        while let Some(&(_, back_val)) = self.deque.back() {
            if back_val <= value {
                self.deque.pop_back();
            } else {
                break;
            }
        }

        self.deque.push_back((self.index, value));

        // Remove expired elements from front.
        while let Some(&(front_idx, _)) = self.deque.front() {
            if self.index >= self.window_size && front_idx <= self.index - self.window_size {
                self.deque.pop_front();
            } else {
                break;
            }
        }

        self.index += 1;
        self.deque.front().map(|&(_, v)| v).unwrap_or(f64::NEG_INFINITY)
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.push(x)).collect()
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.deque.clear();
        self.index = 0;
    }
}

/// Running minimum over a sliding window.
#[derive(Debug, Clone)]
pub struct RunningMin {
    window_size: usize,
    deque: VecDeque<(usize, f64)>,
    index: usize,
}

impl RunningMin {
    /// Create a new running min with the given window size.
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size: window_size.max(1),
            deque: VecDeque::new(),
            index: 0,
        }
    }

    /// Push a sample and return the current window minimum.
    pub fn push(&mut self, value: f64) -> f64 {
        while let Some(&(_, back_val)) = self.deque.back() {
            if back_val >= value {
                self.deque.pop_back();
            } else {
                break;
            }
        }

        self.deque.push_back((self.index, value));

        while let Some(&(front_idx, _)) = self.deque.front() {
            if self.index >= self.window_size && front_idx <= self.index - self.window_size {
                self.deque.pop_front();
            } else {
                break;
            }
        }

        self.index += 1;
        self.deque.front().map(|&(_, v)| v).unwrap_or(f64::INFINITY)
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.push(x)).collect()
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.deque.clear();
        self.index = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_elementwise_max() {
        let a = vec![1.0, 5.0, 3.0];
        let b = vec![4.0, 2.0, 6.0];
        let result = elementwise_max(&[&a, &b]);
        assert_eq!(result, vec![4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_elementwise_min() {
        let a = vec![1.0, 5.0, 3.0];
        let b = vec![4.0, 2.0, 6.0];
        let result = elementwise_min(&[&a, &b]);
        assert_eq!(result, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_elementwise_argmax() {
        let a = vec![1.0, 5.0];
        let b = vec![4.0, 2.0];
        let result = elementwise_argmax(&[&a, &b]);
        assert_eq!(result[0], (4.0, 1));
        assert_eq!(result[1], (5.0, 0));
    }

    #[test]
    fn test_three_streams() {
        let a = vec![1.0, 8.0, 3.0];
        let b = vec![4.0, 2.0, 9.0];
        let c = vec![7.0, 5.0, 6.0];
        let result = elementwise_max(&[&a, &b, &c]);
        assert_eq!(result, vec![7.0, 8.0, 9.0]);
    }

    #[test]
    fn test_empty_streams() {
        let result = elementwise_max(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_running_max() {
        let mut rm = RunningMax::new(3);
        let output = rm.process(&[1.0, 3.0, 2.0, 5.0, 1.0]);
        assert_eq!(output[0], 1.0); // window: [1]
        assert_eq!(output[1], 3.0); // window: [1,3]
        assert_eq!(output[2], 3.0); // window: [1,3,2]
        assert_eq!(output[3], 5.0); // window: [3,2,5]
        assert_eq!(output[4], 5.0); // window: [2,5,1]
    }

    #[test]
    fn test_running_min() {
        let mut rm = RunningMin::new(3);
        let output = rm.process(&[5.0, 3.0, 4.0, 1.0, 6.0]);
        assert_eq!(output[0], 5.0); // window: [5]
        assert_eq!(output[1], 3.0); // window: [5,3]
        assert_eq!(output[2], 3.0); // window: [5,3,4]
        assert_eq!(output[3], 1.0); // window: [3,4,1]
        assert_eq!(output[4], 1.0); // window: [4,1,6]
    }

    #[test]
    fn test_running_max_constant() {
        let mut rm = RunningMax::new(4);
        let output = rm.process(&[5.0, 5.0, 5.0, 5.0]);
        assert!(output.iter().all(|&v| (v - 5.0).abs() < 1e-10));
    }

    #[test]
    fn test_running_max_reset() {
        let mut rm = RunningMax::new(3);
        rm.process(&[1.0, 2.0, 3.0]);
        rm.reset();
        let out = rm.process(&[10.0]);
        assert_eq!(out[0], 10.0);
    }

    #[test]
    fn test_different_lengths() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![4.0, 5.0]; // shorter
        let result = elementwise_max(&[&a, &b]);
        assert_eq!(result.len(), 2); // Min length
    }
}

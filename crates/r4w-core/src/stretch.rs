//! Stretch — Signal normalization and range mapping
//!
//! Maps signal values to [0, 1] range by stretching the minimum to 0
//! and the maximum to 1. Useful for display normalization, feature
//! scaling, and pre-processing for threshold-based detection.
//! GNU Radio equivalent: `stretch_ff`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stretch::{stretch, stretch_to_range};
//!
//! let data = vec![2.0, 4.0, 6.0, 8.0, 10.0];
//! let normalized = stretch(&data);
//! assert!((normalized[0] - 0.0).abs() < 1e-10);
//! assert!((normalized[4] - 1.0).abs() < 1e-10);
//!
//! // Map to custom range
//! let mapped = stretch_to_range(&data, -1.0, 1.0);
//! assert!((mapped[0] - (-1.0)).abs() < 1e-10);
//! assert!((mapped[4] - 1.0).abs() < 1e-10);
//! ```

/// Stretch signal to [0, 1] range.
///
/// If all values are equal, returns all zeros.
pub fn stretch(input: &[f64]) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }
    let min = input.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = input.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let range = max - min;
    if range == 0.0 {
        return vec![0.0; input.len()];
    }
    input.iter().map(|&x| (x - min) / range).collect()
}

/// Stretch signal to [lo, hi] range.
pub fn stretch_to_range(input: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }
    let min = input.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = input.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let range = max - min;
    if range == 0.0 {
        return vec![lo; input.len()];
    }
    let scale = hi - lo;
    input
        .iter()
        .map(|&x| lo + scale * (x - min) / range)
        .collect()
}

/// Stretch in-place to [0, 1].
pub fn stretch_inplace(data: &mut [f64]) {
    if data.is_empty() {
        return;
    }
    let min = data.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let range = max - min;
    if range == 0.0 {
        for x in data.iter_mut() {
            *x = 0.0;
        }
        return;
    }
    for x in data.iter_mut() {
        *x = (*x - min) / range;
    }
}

/// Streaming stretch block with running min/max.
#[derive(Debug, Clone)]
pub struct StreamStretch {
    /// Window size for min/max estimation.
    window_size: usize,
    /// Circular buffer for windowed min/max.
    buffer: Vec<f64>,
    /// Write position.
    pos: usize,
    /// Number of samples seen.
    count: usize,
}

impl StreamStretch {
    /// Create a streaming stretch block.
    ///
    /// `window_size`: number of samples for min/max estimation.
    pub fn new(window_size: usize) -> Self {
        let window_size = window_size.max(1);
        Self {
            window_size,
            buffer: Vec::with_capacity(window_size),
            pos: 0,
            count: 0,
        }
    }

    /// Process a single sample, returning normalized value.
    pub fn process_sample(&mut self, x: f64) -> f64 {
        if self.buffer.len() < self.window_size {
            self.buffer.push(x);
        } else {
            self.buffer[self.pos] = x;
        }
        self.pos = (self.pos + 1) % self.window_size;
        self.count += 1;

        let min = self.buffer.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = self
            .buffer
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let range = max - min;
        if range == 0.0 {
            0.0
        } else {
            (x - min) / range
        }
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.pos = 0;
        self.count = 0;
    }
}

/// Clip values to [lo, hi] range.
pub fn clip(input: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    input.iter().map(|&x| x.clamp(lo, hi)).collect()
}

/// Clip in-place.
pub fn clip_inplace(data: &mut [f64], lo: f64, hi: f64) {
    for x in data.iter_mut() {
        *x = x.clamp(lo, hi);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stretch_basic() {
        let data = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let out = stretch(&data);
        assert!((out[0] - 0.0).abs() < 1e-10);
        assert!((out[2] - 0.5).abs() < 1e-10);
        assert!((out[4] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_stretch_negative() {
        let data = vec![-10.0, 0.0, 10.0];
        let out = stretch(&data);
        assert!((out[0] - 0.0).abs() < 1e-10);
        assert!((out[1] - 0.5).abs() < 1e-10);
        assert!((out[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_stretch_constant() {
        let data = vec![5.0, 5.0, 5.0];
        let out = stretch(&data);
        assert_eq!(out, vec![0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_stretch_empty() {
        assert!(stretch(&[]).is_empty());
    }

    #[test]
    fn test_stretch_single() {
        let out = stretch(&[42.0]);
        assert_eq!(out, vec![0.0]);
    }

    #[test]
    fn test_stretch_to_range() {
        let data = vec![0.0, 5.0, 10.0];
        let out = stretch_to_range(&data, -1.0, 1.0);
        assert!((out[0] - (-1.0)).abs() < 1e-10);
        assert!((out[1] - 0.0).abs() < 1e-10);
        assert!((out[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_stretch_to_range_custom() {
        let data = vec![0.0, 10.0];
        let out = stretch_to_range(&data, 100.0, 200.0);
        assert!((out[0] - 100.0).abs() < 1e-10);
        assert!((out[1] - 200.0).abs() < 1e-10);
    }

    #[test]
    fn test_stretch_inplace() {
        let mut data = vec![2.0, 4.0, 6.0];
        stretch_inplace(&mut data);
        assert!((data[0] - 0.0).abs() < 1e-10);
        assert!((data[1] - 0.5).abs() < 1e-10);
        assert!((data[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_stream_stretch() {
        let mut ss = StreamStretch::new(5);
        // Feed 5 values: 1,2,3,4,5
        let mut outputs = Vec::new();
        for &v in &[1.0, 2.0, 3.0, 4.0, 5.0] {
            outputs.push(ss.process_sample(v));
        }
        // After all 5, min=1 max=5 range=4
        // last output: (5-1)/4 = 1.0
        assert!((outputs[4] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_stream_stretch_reset() {
        let mut ss = StreamStretch::new(10);
        ss.process(&[1.0, 2.0, 3.0]);
        ss.reset();
        assert_eq!(ss.count, 0);
    }

    #[test]
    fn test_clip_basic() {
        let data = vec![-5.0, 0.0, 5.0, 10.0];
        let out = clip(&data, 0.0, 5.0);
        assert_eq!(out, vec![0.0, 0.0, 5.0, 5.0]);
    }

    #[test]
    fn test_clip_inplace() {
        let mut data = vec![-2.0, 0.5, 2.0];
        clip_inplace(&mut data, -1.0, 1.0);
        assert_eq!(data, vec![-1.0, 0.5, 1.0]);
    }

    #[test]
    fn test_stretch_preserves_order() {
        let data = vec![3.0, 1.0, 4.0, 1.0, 5.0, 9.0];
        let out = stretch(&data);
        // Min=1 (index 1,3), Max=9 (index 5)
        assert!((out[5] - 1.0).abs() < 1e-10); // max -> 1.0
        assert!((out[1] - 0.0).abs() < 1e-10); // min -> 0.0
        assert!((out[3] - 0.0).abs() < 1e-10); // min -> 0.0
    }
}

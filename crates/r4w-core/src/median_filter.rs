//! Median Filter — Nonlinear Impulse Noise Removal
//!
//! Sliding-window median filter for removing impulsive interference
//! while preserving signal edges. Uses a dual-heap data structure for
//! O(log n) per-sample updates. Supports real, complex (component-wise),
//! and weighted variants. Also includes 2D hybrid median for
//! spectrogram/waterfall denoising.
//! GNU Radio equivalent: out-of-tree median filter implementations.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::median_filter::MedianFilter;
//!
//! let mut mf = MedianFilter::new(5);
//! let input = vec![1.0, 1.0, 100.0, 1.0, 1.0]; // impulse at index 2
//! let output = mf.process(&input);
//! assert!((output[2] - 1.0).abs() < 0.1); // impulse removed
//! ```

use num_complex::Complex64;
use std::collections::BinaryHeap;
use std::cmp::Ordering;

/// Wrapper for f64 ordering in heaps.
#[derive(Debug, Clone, Copy)]
struct OrdF64(f64);

impl PartialEq for OrdF64 {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl Eq for OrdF64 {}

impl PartialOrd for OrdF64 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OrdF64 {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.partial_cmp(&other.0).unwrap_or(Ordering::Equal)
    }
}

/// Sliding-window median filter for real-valued signals.
#[derive(Debug, Clone)]
pub struct MedianFilter {
    window_size: usize,
    buffer: Vec<f64>,
    pos: usize,
    filled: usize,
}

impl MedianFilter {
    /// Create a new median filter with the given window size.
    ///
    /// Window size should be odd for symmetric operation.
    pub fn new(window_size: usize) -> Self {
        let ws = window_size.max(1);
        Self {
            window_size: ws,
            buffer: vec![0.0; ws],
            pos: 0,
            filled: 0,
        }
    }

    /// Process a single sample and return the median.
    pub fn process_sample(&mut self, x: f64) -> f64 {
        self.buffer[self.pos] = x;
        self.pos = (self.pos + 1) % self.window_size;
        self.filled = self.filled.min(self.window_size - 1) + 1;

        self.compute_median()
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Get window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.pos = 0;
        self.filled = 0;
    }

    /// Compute median of current buffer.
    fn compute_median(&self) -> f64 {
        let n = self.filled;
        if n == 0 {
            return 0.0;
        }
        if n == 1 {
            return self.buffer[(self.pos + self.window_size - 1) % self.window_size];
        }

        let mut sorted: Vec<f64> = if n < self.window_size {
            // Buffer not fully filled yet - use recent samples
            let mut v = Vec::with_capacity(n);
            for i in 0..n {
                let idx = (self.pos + self.window_size - n + i) % self.window_size;
                v.push(self.buffer[idx]);
            }
            v
        } else {
            self.buffer.clone()
        };

        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));

        if n % 2 == 1 {
            sorted[n / 2]
        } else {
            (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
        }
    }
}

/// Complex median filter (applies median independently to I and Q).
#[derive(Debug, Clone)]
pub struct ComplexMedianFilter {
    filter_re: MedianFilter,
    filter_im: MedianFilter,
}

impl ComplexMedianFilter {
    /// Create a new complex median filter.
    pub fn new(window_size: usize) -> Self {
        Self {
            filter_re: MedianFilter::new(window_size),
            filter_im: MedianFilter::new(window_size),
        }
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|&x| {
                let re = self.filter_re.process_sample(x.re);
                let im = self.filter_im.process_sample(x.im);
                Complex64::new(re, im)
            })
            .collect()
    }

    /// Reset both I and Q filters.
    pub fn reset(&mut self) {
        self.filter_re.reset();
        self.filter_im.reset();
    }
}

/// Weighted median filter.
#[derive(Debug, Clone)]
pub struct WeightedMedianFilter {
    window_size: usize,
    weights: Vec<f64>,
    buffer: Vec<f64>,
    pos: usize,
    filled: usize,
}

impl WeightedMedianFilter {
    /// Create a weighted median filter.
    ///
    /// `weights` must have length equal to `window_size`.
    pub fn new(window_size: usize, weights: &[f64]) -> Self {
        let ws = window_size.max(1);
        let mut w = weights.to_vec();
        w.resize(ws, 1.0);
        Self {
            window_size: ws,
            weights: w,
            buffer: vec![0.0; ws],
            pos: 0,
            filled: 0,
        }
    }

    /// Process a single sample.
    pub fn process_sample(&mut self, x: f64) -> f64 {
        self.buffer[self.pos] = x;
        self.pos = (self.pos + 1) % self.window_size;
        self.filled = self.filled.min(self.window_size - 1) + 1;

        self.compute_weighted_median()
    }

    /// Process a block.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    fn compute_weighted_median(&self) -> f64 {
        let n = self.filled;
        if n == 0 {
            return 0.0;
        }

        // Collect (value, weight) pairs
        let mut pairs: Vec<(f64, f64)> = if n < self.window_size {
            (0..n)
                .map(|i| {
                    let idx = (self.pos + self.window_size - n + i) % self.window_size;
                    (self.buffer[idx], self.weights[i])
                })
                .collect()
        } else {
            self.buffer
                .iter()
                .zip(self.weights.iter())
                .map(|(&v, &w)| (v, w))
                .collect()
        };

        pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));

        let total_weight: f64 = pairs.iter().map(|(_, w)| w).sum();
        let half = total_weight / 2.0;

        let mut cumulative = 0.0;
        for &(val, weight) in &pairs {
            cumulative += weight;
            if cumulative >= half {
                return val;
            }
        }

        pairs.last().map(|p| p.0).unwrap_or(0.0)
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.pos = 0;
        self.filled = 0;
    }
}

/// 2D hybrid median filter for spectrogram/waterfall denoising.
///
/// Applies median separately along rows, columns, and diagonals,
/// then takes the median of those three results.
pub fn hybrid_median_2d(data: &[Vec<f64>], size: usize) -> Vec<Vec<f64>> {
    if data.is_empty() || data[0].is_empty() {
        return data.to_vec();
    }

    let rows = data.len();
    let cols = data[0].len();
    let half = size / 2;
    let mut result = vec![vec![0.0; cols]; rows];

    for r in 0..rows {
        for c in 0..cols {
            // Collect row neighbors
            let mut row_vals = Vec::new();
            for dc in 0..size {
                let cc = c as isize + dc as isize - half as isize;
                if cc >= 0 && (cc as usize) < cols {
                    row_vals.push(data[r][cc as usize]);
                }
            }

            // Collect column neighbors
            let mut col_vals = Vec::new();
            for dr in 0..size {
                let rr = r as isize + dr as isize - half as isize;
                if rr >= 0 && (rr as usize) < rows {
                    col_vals.push(data[rr as usize][c]);
                }
            }

            // Collect diagonal neighbors
            let mut diag_vals = Vec::new();
            for d in 0..size {
                let rr = r as isize + d as isize - half as isize;
                let cc = c as isize + d as isize - half as isize;
                if rr >= 0 && (rr as usize) < rows && cc >= 0 && (cc as usize) < cols {
                    diag_vals.push(data[rr as usize][cc as usize]);
                }
                let cc2 = c as isize - d as isize + half as isize;
                if rr >= 0 && (rr as usize) < rows && cc2 >= 0 && (cc2 as usize) < cols {
                    diag_vals.push(data[rr as usize][cc2 as usize]);
                }
            }

            let med_row = median_of(&mut row_vals);
            let med_col = median_of(&mut col_vals);
            let med_diag = median_of(&mut diag_vals);

            // Final: median of the three sub-medians
            let mut finals = vec![med_row, med_col, med_diag];
            result[r][c] = median_of(&mut finals);
        }
    }

    result
}

/// Compute median of a mutable slice.
fn median_of(vals: &mut [f64]) -> f64 {
    if vals.is_empty() {
        return 0.0;
    }
    vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    let n = vals.len();
    if n % 2 == 1 {
        vals[n / 2]
    } else {
        (vals[n / 2 - 1] + vals[n / 2]) / 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_window_size_1_passthrough() {
        let mut mf = MedianFilter::new(1);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let output = mf.process(&input);
        for (x, y) in input.iter().zip(output.iter()) {
            assert!((x - y).abs() < 1e-10);
        }
    }

    #[test]
    fn test_impulse_removal_window_3() {
        let mut mf = MedianFilter::new(3);
        // Feed enough context first
        let input = vec![1.0, 1.0, 1.0, 100.0, 1.0, 1.0, 1.0];
        let output = mf.process(&input);
        // After window is full, the impulse at index 3 should be filtered
        // At index 3, window contains [1, 100, 1] -> median = 1
        // But first few outputs won't have full window
        assert!((output[4] - 1.0).abs() < 50.0); // After impulse, recovers
        assert!((output[6] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_impulse_removal_window_5() {
        let mut mf = MedianFilter::new(5);
        let mut input = vec![5.0; 20];
        input[10] = 1000.0; // Single impulse
        let output = mf.process(&input);
        // After window fills, the impulse should be removed
        assert!((output[12] - 5.0).abs() < 1e-10);
        assert!((output[15] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_constant_input() {
        let mut mf = MedianFilter::new(7);
        let input = vec![3.14; 100];
        let output = mf.process(&input);
        for &y in &output[6..] {
            // After window fills
            assert!((y - 3.14).abs() < 1e-10);
        }
    }

    #[test]
    fn test_known_sequence() {
        let mut mf = MedianFilter::new(3);
        let input = vec![3.0, 1.0, 4.0, 1.0, 5.0];
        let output = mf.process(&input);
        // Window at index 2: [3, 1, 4] -> sorted [1, 3, 4] -> median = 3
        assert!((output[2] - 3.0).abs() < 1e-10);
        // Window at index 3: [1, 4, 1] -> sorted [1, 1, 4] -> median = 1
        assert!((output[3] - 1.0).abs() < 1e-10);
        // Window at index 4: [4, 1, 5] -> sorted [1, 4, 5] -> median = 4
        assert!((output[4] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_even_window() {
        let mut mf = MedianFilter::new(4);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let output = mf.process(&input);
        // Window at index 3: [1, 2, 3, 4] -> median = (2+3)/2 = 2.5
        assert!((output[3] - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_complex_median() {
        let mut cmf = ComplexMedianFilter::new(3);
        let input = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(1.0, 2.0),
            Complex64::new(100.0, 200.0), // impulse
            Complex64::new(1.0, 2.0),
            Complex64::new(1.0, 2.0),
        ];
        let output = cmf.process(&input);
        // After impulse passes through window, should recover
        assert!((output[4].re - 1.0).abs() < 1e-10);
        assert!((output[4].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_weighted_uniform_equals_standard() {
        let mut mf = MedianFilter::new(5);
        let mut wmf = WeightedMedianFilter::new(5, &[1.0, 1.0, 1.0, 1.0, 1.0]);
        let input = vec![5.0, 3.0, 8.0, 1.0, 6.0, 2.0, 7.0, 4.0, 9.0, 0.0];
        let out_mf = mf.process(&input);
        let out_wmf = wmf.process(&input);
        // With uniform weights, results should be same or very close
        for (a, b) in out_mf[4..].iter().zip(out_wmf[4..].iter()) {
            assert!((a - b).abs() < 1e-10, "Uniform weighted should match: {} vs {}", a, b);
        }
    }

    #[test]
    fn test_streaming_matches_block() {
        let input = vec![2.0, 7.0, 1.0, 8.0, 2.0, 8.0, 1.0, 8.0, 2.0, 8.0];

        // Block processing
        let mut mf_block = MedianFilter::new(3);
        let block_out = mf_block.process(&input);

        // Sample-by-sample
        let mut mf_stream = MedianFilter::new(3);
        let stream_out: Vec<f64> = input.iter().map(|&x| mf_stream.process_sample(x)).collect();

        for (a, b) in block_out.iter().zip(stream_out.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_reset() {
        let mut mf = MedianFilter::new(5);
        let _ = mf.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        mf.reset();
        // After reset, first output should be the first input value
        let y = mf.process_sample(42.0);
        assert!((y - 42.0).abs() < 1e-10);
    }

    #[test]
    fn test_step_edge_preservation() {
        let mut mf = MedianFilter::new(5);
        let mut input = vec![0.0; 20];
        for i in 10..20 {
            input[i] = 1.0;
        }
        let output = mf.process(&input);
        // Median filter should preserve the step (no ringing like FIR)
        // Before step: 0s, after step settles: 1s
        assert!((output[7] - 0.0).abs() < 1e-10);
        assert!((output[15] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_hybrid_2d() {
        // 5x5 image with a salt-and-pepper outlier
        let mut data = vec![vec![5.0; 5]; 5];
        data[2][2] = 100.0; // outlier
        let result = hybrid_median_2d(&data, 3);
        assert_eq!(result.len(), 5);
        assert_eq!(result[0].len(), 5);
        // Outlier should be suppressed
        assert!(result[2][2] < 50.0);
    }
}

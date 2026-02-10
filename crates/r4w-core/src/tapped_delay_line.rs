//! # Tapped Delay Line
//!
//! A delay line with tapped outputs at configurable offsets.
//! Provides access to delayed versions of the input signal,
//! useful for building custom FIR-like structures, correlators,
//! and multi-path simulators.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tapped_delay_line::TappedDelayLine;
//!
//! let mut tdl = TappedDelayLine::new(4, vec![0, 1, 3]);
//! tdl.push(1.0);
//! tdl.push(2.0);
//! tdl.push(3.0);
//! tdl.push(4.0);
//! let taps = tdl.read_taps();
//! assert_eq!(taps, vec![4.0, 3.0, 1.0]); // delays 0, 1, 3
//! ```

use std::collections::VecDeque;

/// Tapped delay line with configurable tap positions.
#[derive(Debug, Clone)]
pub struct TappedDelayLine {
    /// Delay buffer.
    buffer: VecDeque<f64>,
    /// Maximum delay (buffer size).
    max_delay: usize,
    /// Tap positions (offsets from newest sample).
    tap_positions: Vec<usize>,
    /// Total samples pushed.
    count: u64,
}

impl TappedDelayLine {
    /// Create a new tapped delay line.
    ///
    /// * `max_delay` - Maximum delay in samples (buffer size)
    /// * `tap_positions` - Tap offsets from newest sample (0 = current)
    pub fn new(max_delay: usize, tap_positions: Vec<usize>) -> Self {
        let mut buffer = VecDeque::with_capacity(max_delay + 1);
        for _ in 0..=max_delay {
            buffer.push_back(0.0);
        }
        Self {
            buffer,
            max_delay,
            tap_positions,
            count: 0,
        }
    }

    /// Push a new sample into the delay line.
    pub fn push(&mut self, sample: f64) {
        self.buffer.push_front(sample);
        if self.buffer.len() > self.max_delay + 1 {
            self.buffer.pop_back();
        }
        self.count += 1;
    }

    /// Read all tap outputs.
    pub fn read_taps(&self) -> Vec<f64> {
        self.tap_positions
            .iter()
            .map(|&pos| {
                if pos < self.buffer.len() {
                    self.buffer[pos]
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Read a single tap output.
    pub fn read_tap(&self, index: usize) -> f64 {
        if index < self.tap_positions.len() {
            let pos = self.tap_positions[index];
            if pos < self.buffer.len() {
                self.buffer[pos]
            } else {
                0.0
            }
        } else {
            0.0
        }
    }

    /// Process a block of samples, returning tap outputs for each.
    pub fn process(&mut self, input: &[f64]) -> Vec<Vec<f64>> {
        let mut outputs = Vec::with_capacity(input.len());
        for &sample in input {
            self.push(sample);
            outputs.push(self.read_taps());
        }
        outputs
    }

    /// Get the weighted sum of taps (like an FIR filter output).
    pub fn weighted_sum(&self, weights: &[f64]) -> f64 {
        let taps = self.read_taps();
        taps.iter()
            .zip(weights.iter())
            .map(|(&t, &w)| t * w)
            .sum()
    }

    /// Get the number of taps.
    pub fn num_taps(&self) -> usize {
        self.tap_positions.len()
    }

    /// Get the tap positions.
    pub fn tap_positions(&self) -> &[usize] {
        &self.tap_positions
    }

    /// Set new tap positions.
    pub fn set_tap_positions(&mut self, positions: Vec<usize>) {
        self.tap_positions = positions;
    }

    /// Get the maximum delay.
    pub fn max_delay(&self) -> usize {
        self.max_delay
    }

    /// Get total samples pushed.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Reset the delay line (fill with zeros).
    pub fn reset(&mut self) {
        for sample in self.buffer.iter_mut() {
            *sample = 0.0;
        }
        self.count = 0;
    }
}

/// Complex tapped delay line.
#[derive(Debug, Clone)]
pub struct TappedDelayLineComplex {
    buffer: VecDeque<(f64, f64)>,
    max_delay: usize,
    tap_positions: Vec<usize>,
}

impl TappedDelayLineComplex {
    /// Create a new complex tapped delay line.
    pub fn new(max_delay: usize, tap_positions: Vec<usize>) -> Self {
        let mut buffer = VecDeque::with_capacity(max_delay + 1);
        for _ in 0..=max_delay {
            buffer.push_back((0.0, 0.0));
        }
        Self {
            buffer,
            max_delay,
            tap_positions,
        }
    }

    /// Push a complex sample.
    pub fn push(&mut self, sample: (f64, f64)) {
        self.buffer.push_front(sample);
        if self.buffer.len() > self.max_delay + 1 {
            self.buffer.pop_back();
        }
    }

    /// Read all tap outputs.
    pub fn read_taps(&self) -> Vec<(f64, f64)> {
        self.tap_positions
            .iter()
            .map(|&pos| {
                if pos < self.buffer.len() {
                    self.buffer[pos]
                } else {
                    (0.0, 0.0)
                }
            })
            .collect()
    }

    /// Reset.
    pub fn reset(&mut self) {
        for sample in self.buffer.iter_mut() {
            *sample = (0.0, 0.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_delay() {
        let mut tdl = TappedDelayLine::new(4, vec![0, 1, 2, 3, 4]);
        tdl.push(5.0);
        tdl.push(4.0);
        tdl.push(3.0);
        tdl.push(2.0);
        tdl.push(1.0);
        let taps = tdl.read_taps();
        assert_eq!(taps, vec![1.0, 2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_sparse_taps() {
        let mut tdl = TappedDelayLine::new(10, vec![0, 5, 10]);
        for i in 0..11 {
            tdl.push(i as f64);
        }
        let taps = tdl.read_taps();
        assert_eq!(taps[0], 10.0); // delay 0
        assert_eq!(taps[1], 5.0);  // delay 5
        assert_eq!(taps[2], 0.0);  // delay 10
    }

    #[test]
    fn test_weighted_sum() {
        let mut tdl = TappedDelayLine::new(2, vec![0, 1, 2]);
        tdl.push(1.0);
        tdl.push(2.0);
        tdl.push(3.0);
        let weights = vec![1.0, 0.5, 0.25];
        let result = tdl.weighted_sum(&weights);
        // 3.0*1.0 + 2.0*0.5 + 1.0*0.25 = 4.25
        assert!((result - 4.25).abs() < 1e-10);
    }

    #[test]
    fn test_process_block() {
        let mut tdl = TappedDelayLine::new(1, vec![0, 1]);
        let output = tdl.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output.len(), 3);
        assert_eq!(output[0], vec![1.0, 0.0]); // delay line initially zero
        assert_eq!(output[1], vec![2.0, 1.0]);
        assert_eq!(output[2], vec![3.0, 2.0]);
    }

    #[test]
    fn test_read_single_tap() {
        let mut tdl = TappedDelayLine::new(3, vec![0, 1, 2, 3]);
        tdl.push(1.0);
        tdl.push(2.0);
        assert_eq!(tdl.read_tap(0), 2.0);
        assert_eq!(tdl.read_tap(1), 1.0);
    }

    #[test]
    fn test_reset() {
        let mut tdl = TappedDelayLine::new(3, vec![0, 1]);
        tdl.push(5.0);
        tdl.push(10.0);
        tdl.reset();
        assert_eq!(tdl.read_taps(), vec![0.0, 0.0]);
        assert_eq!(tdl.count(), 0);
    }

    #[test]
    fn test_set_tap_positions() {
        let mut tdl = TappedDelayLine::new(5, vec![0]);
        assert_eq!(tdl.num_taps(), 1);
        tdl.set_tap_positions(vec![0, 2, 4]);
        assert_eq!(tdl.num_taps(), 3);
    }

    #[test]
    fn test_complex_tdl() {
        let mut tdl = TappedDelayLineComplex::new(2, vec![0, 1, 2]);
        tdl.push((1.0, 0.0));
        tdl.push((2.0, 0.0));
        tdl.push((3.0, 0.0));
        let taps = tdl.read_taps();
        assert_eq!(taps[0], (3.0, 0.0));
        assert_eq!(taps[1], (2.0, 0.0));
        assert_eq!(taps[2], (1.0, 0.0));
    }

    #[test]
    fn test_overflow_protection() {
        let mut tdl = TappedDelayLine::new(2, vec![0, 1, 2]);
        tdl.push(1.0);
        tdl.push(2.0);
        tdl.push(3.0);
        tdl.push(4.0); // Buffer should maintain max 3 elements.
        let taps = tdl.read_taps();
        assert_eq!(taps[0], 4.0);
        assert_eq!(taps[1], 3.0);
        assert_eq!(taps[2], 2.0);
    }

    #[test]
    fn test_count() {
        let mut tdl = TappedDelayLine::new(3, vec![0]);
        tdl.push(1.0);
        tdl.push(2.0);
        assert_eq!(tdl.count(), 2);
    }
}

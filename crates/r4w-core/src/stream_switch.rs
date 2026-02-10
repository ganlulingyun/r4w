//! # Stream Switch
//!
//! Dynamically selects between multiple input streams or routes
//! one input to multiple outputs based on a control signal. Useful
//! for diversity combining, A/B testing, and adaptive processing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_switch::{StreamSwitch, MuxSelector};
//!
//! let mut sw = StreamSwitch::new(3); // 3 inputs
//! let streams = vec![
//!     vec![1.0, 2.0, 3.0],
//!     vec![4.0, 5.0, 6.0],
//!     vec![7.0, 8.0, 9.0],
//! ];
//! sw.select(1); // Select second stream.
//! let out = sw.process(&streams);
//! assert_eq!(out, vec![4.0, 5.0, 6.0]);
//! ```

/// N:1 stream multiplexer (selects one of N inputs).
#[derive(Debug, Clone)]
pub struct StreamSwitch {
    num_inputs: usize,
    selected: usize,
    fade_samples: usize,
    switching: bool,
    fade_pos: usize,
    prev_selected: usize,
}

impl StreamSwitch {
    /// Create a new stream switch with N inputs.
    pub fn new(num_inputs: usize) -> Self {
        Self {
            num_inputs: num_inputs.max(1),
            selected: 0,
            fade_samples: 0,
            switching: false,
            fade_pos: 0,
            prev_selected: 0,
        }
    }

    /// Create with crossfade transition.
    pub fn with_crossfade(num_inputs: usize, fade_samples: usize) -> Self {
        Self {
            num_inputs: num_inputs.max(1),
            selected: 0,
            fade_samples,
            switching: false,
            fade_pos: 0,
            prev_selected: 0,
        }
    }

    /// Select input stream by index.
    pub fn select(&mut self, index: usize) {
        let new = index.min(self.num_inputs - 1);
        if new != self.selected && self.fade_samples > 0 {
            self.prev_selected = self.selected;
            self.switching = true;
            self.fade_pos = 0;
        }
        self.selected = new;
    }

    /// Get currently selected input.
    pub fn selected(&self) -> usize {
        self.selected
    }

    /// Process: select output from the correct input stream.
    pub fn process(&mut self, inputs: &[Vec<f64>]) -> Vec<f64> {
        if inputs.is_empty() || self.selected >= inputs.len() {
            return Vec::new();
        }

        if !self.switching {
            return inputs[self.selected].clone();
        }

        // Crossfade between prev and current.
        let prev = &inputs[self.prev_selected.min(inputs.len() - 1)];
        let curr = &inputs[self.selected];
        let len = prev.len().min(curr.len());
        let mut output = Vec::with_capacity(len);

        for i in 0..len {
            let alpha = if self.fade_pos < self.fade_samples {
                self.fade_pos as f64 / self.fade_samples as f64
            } else {
                1.0
            };
            let sample = prev[i] * (1.0 - alpha) + curr[i] * alpha;
            output.push(sample);
            self.fade_pos += 1;
        }

        if self.fade_pos >= self.fade_samples {
            self.switching = false;
        }

        output
    }

    /// Process complex streams.
    pub fn process_complex(&mut self, inputs: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
        if inputs.is_empty() || self.selected >= inputs.len() {
            return Vec::new();
        }
        if !self.switching {
            return inputs[self.selected].clone();
        }
        let prev = &inputs[self.prev_selected.min(inputs.len() - 1)];
        let curr = &inputs[self.selected];
        let len = prev.len().min(curr.len());
        let mut output = Vec::with_capacity(len);
        for i in 0..len {
            let alpha = if self.fade_pos < self.fade_samples {
                self.fade_pos as f64 / self.fade_samples as f64
            } else {
                1.0
            };
            output.push((
                prev[i].0 * (1.0 - alpha) + curr[i].0 * alpha,
                prev[i].1 * (1.0 - alpha) + curr[i].1 * alpha,
            ));
            self.fade_pos += 1;
        }
        if self.fade_pos >= self.fade_samples {
            self.switching = false;
        }
        output
    }

    /// Get number of inputs.
    pub fn num_inputs(&self) -> usize {
        self.num_inputs
    }
}

/// Mux selector that cycles through inputs based on time.
#[derive(Debug, Clone)]
pub struct MuxSelector {
    num_inputs: usize,
    current: usize,
    samples_per_input: usize,
    sample_count: usize,
}

impl MuxSelector {
    /// Create a new mux selector that switches every N samples.
    pub fn new(num_inputs: usize, samples_per_input: usize) -> Self {
        Self {
            num_inputs: num_inputs.max(1),
            current: 0,
            samples_per_input: samples_per_input.max(1),
            sample_count: 0,
        }
    }

    /// Advance by N samples, returning the selected input index.
    pub fn advance(&mut self, n: usize) -> usize {
        self.sample_count += n;
        while self.sample_count >= self.samples_per_input {
            self.sample_count -= self.samples_per_input;
            self.current = (self.current + 1) % self.num_inputs;
        }
        self.current
    }

    /// Get current selection.
    pub fn current(&self) -> usize {
        self.current
    }

    /// Reset to first input.
    pub fn reset(&mut self) {
        self.current = 0;
        self.sample_count = 0;
    }

    /// Process: build output by cycling through inputs.
    pub fn process(&mut self, inputs: &[Vec<f64>], block_size: usize) -> Vec<f64> {
        let mut output = Vec::with_capacity(block_size);
        let mut remaining = block_size;

        while remaining > 0 {
            let until_switch = self.samples_per_input - self.sample_count;
            let take = remaining.min(until_switch);
            if self.current < inputs.len() {
                let src = &inputs[self.current];
                let start = block_size - remaining;
                for i in 0..take {
                    if start + i < src.len() {
                        output.push(src[start + i]);
                    } else {
                        output.push(0.0);
                    }
                }
            } else {
                for _ in 0..take {
                    output.push(0.0);
                }
            }
            remaining -= take;
            self.sample_count += take;
            if self.sample_count >= self.samples_per_input {
                self.sample_count = 0;
                self.current = (self.current + 1) % self.num_inputs;
            }
        }

        output
    }
}

/// 1:N demux — routes one input to one of N outputs.
#[derive(Debug, Clone)]
pub struct StreamDemuxSwitch {
    num_outputs: usize,
    selected: usize,
}

impl StreamDemuxSwitch {
    /// Create a new 1:N demux switch.
    pub fn new(num_outputs: usize) -> Self {
        Self {
            num_outputs: num_outputs.max(1),
            selected: 0,
        }
    }

    /// Select the output port.
    pub fn select(&mut self, index: usize) {
        self.selected = index.min(self.num_outputs - 1);
    }

    /// Route input to the selected output. Returns N vectors.
    pub fn process(&self, input: &[f64]) -> Vec<Vec<f64>> {
        let mut outputs = vec![Vec::new(); self.num_outputs];
        outputs[self.selected] = input.to_vec();
        outputs
    }

    /// Get selected output index.
    pub fn selected(&self) -> usize {
        self.selected
    }

    /// Get number of outputs.
    pub fn num_outputs(&self) -> usize {
        self.num_outputs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_switch() {
        let mut sw = StreamSwitch::new(3);
        let inputs = vec![vec![1.0], vec![2.0], vec![3.0]];
        assert_eq!(sw.process(&inputs), vec![1.0]);
        sw.select(2);
        assert_eq!(sw.process(&inputs), vec![3.0]);
    }

    #[test]
    fn test_crossfade() {
        let mut sw = StreamSwitch::with_crossfade(2, 4);
        let inputs = vec![vec![0.0, 0.0, 0.0, 0.0], vec![1.0, 1.0, 1.0, 1.0]];
        sw.select(1);
        let out = sw.process(&inputs);
        // First sample: alpha=0 → 0.0, last: alpha=3/4 → 0.75.
        assert!(out[0] < 0.01);
        assert!(out[3] > 0.5);
    }

    #[test]
    fn test_no_crossfade() {
        let mut sw = StreamSwitch::new(2);
        let inputs = vec![vec![0.0; 4], vec![1.0; 4]];
        sw.select(1);
        assert_eq!(sw.process(&inputs), vec![1.0; 4]);
    }

    #[test]
    fn test_complex_switch() {
        let mut sw = StreamSwitch::new(2);
        let inputs = vec![vec![(1.0, 0.0)], vec![(0.0, 1.0)]];
        sw.select(1);
        assert_eq!(sw.process_complex(&inputs), vec![(0.0, 1.0)]);
    }

    #[test]
    fn test_mux_selector_basic() {
        let mut mux = MuxSelector::new(3, 10);
        assert_eq!(mux.current(), 0);
        mux.advance(10);
        assert_eq!(mux.current(), 1);
        mux.advance(10);
        assert_eq!(mux.current(), 2);
        mux.advance(10);
        assert_eq!(mux.current(), 0); // Wraps.
    }

    #[test]
    fn test_mux_selector_reset() {
        let mut mux = MuxSelector::new(3, 10);
        mux.advance(25);
        mux.reset();
        assert_eq!(mux.current(), 0);
    }

    #[test]
    fn test_demux_switch() {
        let mut demux = StreamDemuxSwitch::new(3);
        let input = vec![1.0, 2.0, 3.0];
        demux.select(1);
        let outputs = demux.process(&input);
        assert!(outputs[0].is_empty());
        assert_eq!(outputs[1], vec![1.0, 2.0, 3.0]);
        assert!(outputs[2].is_empty());
    }

    #[test]
    fn test_empty_inputs() {
        let mut sw = StreamSwitch::new(2);
        assert!(sw.process(&[]).is_empty());
    }

    #[test]
    fn test_selected_accessor() {
        let mut sw = StreamSwitch::new(5);
        sw.select(3);
        assert_eq!(sw.selected(), 3);
        assert_eq!(sw.num_inputs(), 5);
    }

    #[test]
    fn test_demux_accessors() {
        let demux = StreamDemuxSwitch::new(4);
        assert_eq!(demux.num_outputs(), 4);
        assert_eq!(demux.selected(), 0);
    }
}

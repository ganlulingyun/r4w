//! Selector and Valve Blocks
//!
//! Dynamic stream routing blocks for selecting between multiple inputs
//! or gating output based on control signals.
//!
//! ## Blocks
//!
//! - **Selector**: Routes one of N input streams to the output
//! - **Valve**: Gates a stream on/off based on a boolean control
//! - **Mute**: Replaces samples with zeros when active
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::selector::{Selector, Valve};
//! use num_complex::Complex64;
//!
//! // Select between two streams
//! let sel = Selector::new(2);
//! let streams = vec![
//!     vec![Complex64::new(1.0, 0.0); 10],
//!     vec![Complex64::new(2.0, 0.0); 10],
//! ];
//! let output = sel.select(&streams, 1); // Select stream 1
//! assert!((output[0].re - 2.0).abs() < 1e-10);
//!
//! // Gate a stream
//! let valve = Valve::new();
//! let input = vec![Complex64::new(1.0, 0.0); 10];
//! assert_eq!(valve.process(&input, true).len(), 10); // open → pass through
//! assert!(valve.process(&input, false).is_empty()); // closed → empty
//! ```

use num_complex::Complex64;

/// Stream selector — routes one of N inputs to output.
#[derive(Debug, Clone)]
pub struct Selector {
    /// Number of input streams
    num_inputs: usize,
    /// Currently selected input (0-indexed)
    selected: usize,
}

impl Selector {
    /// Create a new selector for `num_inputs` streams.
    pub fn new(num_inputs: usize) -> Self {
        assert!(num_inputs >= 1);
        Self {
            num_inputs,
            selected: 0,
        }
    }

    /// Select and output the chosen stream.
    pub fn select(&self, streams: &[Vec<Complex64>], index: usize) -> Vec<Complex64> {
        if index < streams.len() {
            streams[index].clone()
        } else {
            Vec::new()
        }
    }

    /// Select using the internally stored index.
    pub fn process(&self, streams: &[Vec<Complex64>]) -> Vec<Complex64> {
        self.select(streams, self.selected)
    }

    /// Set selected input index.
    pub fn set_selected(&mut self, index: usize) {
        self.selected = index.min(self.num_inputs - 1);
    }

    /// Get selected input index.
    pub fn selected(&self) -> usize {
        self.selected
    }

    /// Get number of inputs.
    pub fn num_inputs(&self) -> usize {
        self.num_inputs
    }
}

/// Real-valued stream selector.
#[derive(Debug, Clone)]
pub struct SelectorReal {
    num_inputs: usize,
    selected: usize,
}

impl SelectorReal {
    pub fn new(num_inputs: usize) -> Self {
        assert!(num_inputs >= 1);
        Self {
            num_inputs,
            selected: 0,
        }
    }

    pub fn select(&self, streams: &[&[f64]], index: usize) -> Vec<f64> {
        if index < streams.len() {
            streams[index].to_vec()
        } else {
            Vec::new()
        }
    }

    pub fn set_selected(&mut self, index: usize) {
        self.selected = index.min(self.num_inputs - 1);
    }

    pub fn selected(&self) -> usize {
        self.selected
    }
}

/// Valve — gates a stream on/off.
#[derive(Debug, Clone, Default)]
pub struct Valve {
    /// Whether the valve is open (true = pass through, false = block)
    open: bool,
}

impl Valve {
    /// Create a new valve (default: closed).
    pub fn new() -> Self {
        Self { open: false }
    }

    /// Create an open valve.
    pub fn open_valve() -> Self {
        Self { open: true }
    }

    /// Process: pass through if open, empty if closed.
    pub fn process(&self, input: &[Complex64], open: bool) -> Vec<Complex64> {
        if open {
            input.to_vec()
        } else {
            Vec::new()
        }
    }

    /// Process using internal state.
    pub fn process_stateful(&self, input: &[Complex64]) -> Vec<Complex64> {
        if self.open {
            input.to_vec()
        } else {
            Vec::new()
        }
    }

    /// Set valve state.
    pub fn set_open(&mut self, open: bool) {
        self.open = open;
    }

    /// Check if open.
    pub fn is_open(&self) -> bool {
        self.open
    }
}

/// Mute block — replaces samples with zeros when muted.
///
/// Unlike Valve which drops samples, Mute preserves timing by outputting zeros.
#[derive(Debug, Clone)]
pub struct Mute {
    muted: bool,
}

impl Mute {
    pub fn new(muted: bool) -> Self {
        Self { muted }
    }

    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        if self.muted {
            vec![Complex64::new(0.0, 0.0); input.len()]
        } else {
            input.to_vec()
        }
    }

    pub fn process_real(&self, input: &[f64]) -> Vec<f64> {
        if self.muted {
            vec![0.0; input.len()]
        } else {
            input.to_vec()
        }
    }

    pub fn set_muted(&mut self, muted: bool) {
        self.muted = muted;
    }

    pub fn is_muted(&self) -> bool {
        self.muted
    }
}

/// Rail — clamp signal amplitude to [-max, max].
///
/// Commonly used to limit signal swing for DAC output or to prevent overflow.
#[derive(Debug, Clone)]
pub struct Rail {
    /// Maximum amplitude (positive)
    max: f64,
}

impl Rail {
    /// Create a rail with given maximum amplitude.
    pub fn new(max: f64) -> Self {
        Self { max: max.abs() }
    }

    /// Default rail: [-1.0, 1.0].
    pub fn unit() -> Self {
        Self::new(1.0)
    }

    /// Process real-valued samples.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        input
            .iter()
            .map(|&x| x.clamp(-self.max, self.max))
            .collect()
    }

    /// Process complex samples (clamp real and imaginary independently).
    pub fn process_complex(&self, input: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|z| {
                Complex64::new(
                    z.re.clamp(-self.max, self.max),
                    z.im.clamp(-self.max, self.max),
                )
            })
            .collect()
    }

    /// Get maximum amplitude.
    pub fn max(&self) -> f64 {
        self.max
    }
}

/// Threshold detector — outputs 1.0 if input > threshold, else 0.0.
#[derive(Debug, Clone)]
pub struct Threshold {
    /// Threshold value
    threshold: f64,
}

impl Threshold {
    pub fn new(threshold: f64) -> Self {
        Self { threshold }
    }

    /// Process real-valued samples.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        input
            .iter()
            .map(|&x| if x > self.threshold { 1.0 } else { 0.0 })
            .collect()
    }

    /// Process and output booleans.
    pub fn detect(&self, input: &[f64]) -> Vec<bool> {
        input.iter().map(|&x| x > self.threshold).collect()
    }

    pub fn threshold(&self) -> f64 {
        self.threshold
    }

    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_selector_basic() {
        let sel = Selector::new(3);
        let streams = vec![
            vec![Complex64::new(1.0, 0.0); 5],
            vec![Complex64::new(2.0, 0.0); 5],
            vec![Complex64::new(3.0, 0.0); 5],
        ];
        let out = sel.select(&streams, 1);
        assert_eq!(out.len(), 5);
        assert!((out[0].re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_selector_out_of_range() {
        let sel = Selector::new(2);
        let streams = vec![
            vec![Complex64::new(1.0, 0.0); 5],
        ];
        let out = sel.select(&streams, 5);
        assert!(out.is_empty());
    }

    #[test]
    fn test_selector_set_selected() {
        let mut sel = Selector::new(3);
        sel.set_selected(2);
        assert_eq!(sel.selected(), 2);
        sel.set_selected(10); // Clamp to max
        assert_eq!(sel.selected(), 2);
    }

    #[test]
    fn test_valve_open() {
        let valve = Valve::new();
        let input = vec![Complex64::new(1.0, 2.0); 5];
        let output = valve.process(&input, true);
        assert_eq!(output.len(), 5);
        assert_eq!(output[0], Complex64::new(1.0, 2.0));
    }

    #[test]
    fn test_valve_closed() {
        let valve = Valve::new();
        let input = vec![Complex64::new(1.0, 2.0); 5];
        let output = valve.process(&input, false);
        assert!(output.is_empty());
    }

    #[test]
    fn test_valve_stateful() {
        let mut valve = Valve::new();
        let input = vec![Complex64::new(1.0, 0.0); 5];
        assert!(valve.process_stateful(&input).is_empty()); // Default closed
        valve.set_open(true);
        assert_eq!(valve.process_stateful(&input).len(), 5);
    }

    #[test]
    fn test_mute_active() {
        let mute = Mute::new(true);
        let input = vec![Complex64::new(1.0, 2.0); 5];
        let output = mute.process(&input);
        assert_eq!(output.len(), 5);
        for z in &output {
            assert_eq!(*z, Complex64::new(0.0, 0.0));
        }
    }

    #[test]
    fn test_mute_inactive() {
        let mute = Mute::new(false);
        let input = vec![Complex64::new(1.0, 2.0); 5];
        let output = mute.process(&input);
        assert_eq!(output[0], Complex64::new(1.0, 2.0));
    }

    #[test]
    fn test_rail_unit() {
        let rail = Rail::unit();
        let input = vec![-2.0, -0.5, 0.0, 0.5, 2.0];
        let output = rail.process(&input);
        assert_eq!(output, vec![-1.0, -0.5, 0.0, 0.5, 1.0]);
    }

    #[test]
    fn test_rail_complex() {
        let rail = Rail::new(0.5);
        let input = vec![Complex64::new(1.0, -1.0)];
        let output = rail.process_complex(&input);
        assert!((output[0].re - 0.5).abs() < 1e-10);
        assert!((output[0].im + 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_threshold_basic() {
        let th = Threshold::new(0.5);
        let input = vec![0.0, 0.3, 0.5, 0.7, 1.0];
        let output = th.process(&input);
        assert_eq!(output, vec![0.0, 0.0, 0.0, 1.0, 1.0]);
    }

    #[test]
    fn test_threshold_detect() {
        let th = Threshold::new(0.5);
        let input = vec![0.0, 0.6, 1.0];
        let bools = th.detect(&input);
        assert_eq!(bools, vec![false, true, true]);
    }

    #[test]
    fn test_mute_real() {
        let mute = Mute::new(true);
        let output = mute.process_real(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_selector_real() {
        let sel = SelectorReal::new(2);
        let a = vec![1.0, 2.0];
        let b = vec![3.0, 4.0];
        let out = sel.select(&[&a, &b], 1);
        assert_eq!(out, vec![3.0, 4.0]);
    }
}

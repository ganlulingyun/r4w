//! Threshold — Signal level detector with hysteresis
//!
//! Configurable threshold detector that outputs 1.0 when input exceeds
//! a high threshold and 0.0 when it drops below a low threshold, with
//! hysteresis to prevent chattering. Useful for squelch, burst detection,
//! and level-triggered events.
//! GNU Radio equivalent: `threshold_ff`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::threshold::Threshold;
//!
//! let mut thresh = Threshold::new(0.3, 0.7);
//! // Below low threshold → 0
//! assert_eq!(thresh.process_sample(0.1), 0.0);
//! // Above high threshold → 1
//! assert_eq!(thresh.process_sample(0.8), 1.0);
//! // In hysteresis band, stays at previous state (1)
//! assert_eq!(thresh.process_sample(0.5), 1.0);
//! // Below low threshold → 0
//! assert_eq!(thresh.process_sample(0.2), 0.0);
//! ```

/// Threshold detector with hysteresis.
#[derive(Debug, Clone)]
pub struct Threshold {
    /// Low threshold (transition to 0).
    low: f64,
    /// High threshold (transition to 1).
    high: f64,
    /// Current output state.
    state: bool,
    /// Initial state.
    initial_state: bool,
}

impl Threshold {
    /// Create a threshold detector.
    ///
    /// - `low`: below this → output 0.0
    /// - `high`: above this → output 1.0
    /// - Between low and high: hold previous state (hysteresis)
    pub fn new(low: f64, high: f64) -> Self {
        Self {
            low: low.min(high),
            high: low.max(high),
            state: false,
            initial_state: false,
        }
    }

    /// Create with no hysteresis (single threshold).
    pub fn simple(threshold: f64) -> Self {
        Self::new(threshold, threshold)
    }

    /// Create with initial state set to high.
    pub fn with_initial_state(mut self, state: bool) -> Self {
        self.state = state;
        self.initial_state = state;
        self
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        if x >= self.high {
            self.state = true;
        } else if x <= self.low {
            self.state = false;
        }
        if self.state { 1.0 } else { 0.0 }
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x));
        }
        output
    }

    /// Process producing boolean output.
    pub fn process_bool(&mut self, input: &[f64]) -> Vec<bool> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            self.process_sample(x);
            output.push(self.state);
        }
        output
    }

    /// Get current state.
    pub fn state(&self) -> bool {
        self.state
    }

    /// Get low threshold.
    pub fn low(&self) -> f64 {
        self.low
    }

    /// Get high threshold.
    pub fn high(&self) -> f64 {
        self.high
    }

    /// Set thresholds.
    pub fn set_thresholds(&mut self, low: f64, high: f64) {
        self.low = low.min(high);
        self.high = low.max(high);
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.state = self.initial_state;
    }
}

/// Simple comparator: output 1.0 if input >= threshold, else 0.0.
/// No hysteresis, no state.
pub fn compare(input: &[f64], threshold: f64) -> Vec<f64> {
    input
        .iter()
        .map(|&x| if x >= threshold { 1.0 } else { 0.0 })
        .collect()
}

/// Detect rising edges (0→1 transitions) in threshold output.
pub fn rising_edges(signal: &[f64], threshold: f64) -> Vec<usize> {
    let mut edges = Vec::new();
    let mut prev_above = false;
    for (i, &x) in signal.iter().enumerate() {
        let above = x >= threshold;
        if above && !prev_above {
            edges.push(i);
        }
        prev_above = above;
    }
    edges
}

/// Detect falling edges (1→0 transitions) in threshold output.
pub fn falling_edges(signal: &[f64], threshold: f64) -> Vec<usize> {
    let mut edges = Vec::new();
    let mut prev_above = false;
    for (i, &x) in signal.iter().enumerate() {
        let above = x >= threshold;
        if !above && prev_above {
            edges.push(i);
        }
        prev_above = above;
    }
    edges
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_threshold() {
        let mut t = Threshold::new(0.3, 0.7);
        assert_eq!(t.process_sample(0.1), 0.0);
        assert_eq!(t.process_sample(0.8), 1.0);
        assert_eq!(t.process_sample(0.2), 0.0);
    }

    #[test]
    fn test_hysteresis() {
        let mut t = Threshold::new(0.3, 0.7);
        // Start below → 0
        assert_eq!(t.process_sample(0.1), 0.0);
        // Go above high → 1
        assert_eq!(t.process_sample(0.8), 1.0);
        // In hysteresis band → stays 1
        assert_eq!(t.process_sample(0.5), 1.0);
        assert_eq!(t.process_sample(0.4), 1.0);
        // Drop below low → 0
        assert_eq!(t.process_sample(0.2), 0.0);
        // Back in band → stays 0
        assert_eq!(t.process_sample(0.5), 0.0);
    }

    #[test]
    fn test_simple_threshold() {
        let mut t = Threshold::simple(0.5);
        assert_eq!(t.process_sample(0.3), 0.0);
        assert_eq!(t.process_sample(0.7), 1.0);
        assert_eq!(t.process_sample(0.3), 0.0);
    }

    #[test]
    fn test_block_process() {
        let mut t = Threshold::new(0.3, 0.7);
        let input = vec![0.1, 0.8, 0.5, 0.2, 0.5, 0.9];
        let output = t.process(&input);
        assert_eq!(output, vec![0.0, 1.0, 1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_bool_output() {
        let mut t = Threshold::simple(0.5);
        let input = vec![0.3, 0.7, 0.3];
        let output = t.process_bool(&input);
        assert_eq!(output, vec![false, true, false]);
    }

    #[test]
    fn test_initial_state_high() {
        let mut t = Threshold::new(0.3, 0.7).with_initial_state(true);
        // In band, but initial state is true → 1
        assert_eq!(t.process_sample(0.5), 1.0);
        // Below low → 0
        assert_eq!(t.process_sample(0.1), 0.0);
    }

    #[test]
    fn test_set_thresholds() {
        let mut t = Threshold::new(0.3, 0.7);
        t.set_thresholds(0.1, 0.9);
        assert_eq!(t.low(), 0.1);
        assert_eq!(t.high(), 0.9);
    }

    #[test]
    fn test_swapped_thresholds() {
        // If user passes high < low, should auto-swap
        let t = Threshold::new(0.7, 0.3);
        assert_eq!(t.low(), 0.3);
        assert_eq!(t.high(), 0.7);
    }

    #[test]
    fn test_reset() {
        let mut t = Threshold::new(0.3, 0.7);
        t.process_sample(0.8); // state = true
        t.reset();
        assert!(!t.state());
    }

    #[test]
    fn test_compare() {
        let result = compare(&[0.1, 0.5, 0.9], 0.5);
        assert_eq!(result, vec![0.0, 1.0, 1.0]);
    }

    #[test]
    fn test_rising_edges() {
        let signal = vec![0.0, 0.0, 1.0, 1.0, 0.0, 1.0];
        let edges = rising_edges(&signal, 0.5);
        assert_eq!(edges, vec![2, 5]);
    }

    #[test]
    fn test_falling_edges() {
        let signal = vec![1.0, 1.0, 0.0, 0.0, 1.0, 0.0];
        let edges = falling_edges(&signal, 0.5);
        assert_eq!(edges, vec![2, 5]);
    }

    #[test]
    fn test_no_edges() {
        let signal = vec![0.0, 0.0, 0.0];
        assert!(rising_edges(&signal, 0.5).is_empty());
        assert!(falling_edges(&signal, 0.5).is_empty());
    }

    #[test]
    fn test_empty() {
        let mut t = Threshold::new(0.3, 0.7);
        assert!(t.process(&[]).is_empty());
    }
}

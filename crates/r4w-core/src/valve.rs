//! Valve — Stream gating and flow control
//!
//! Controls signal flow through a processing chain. Supports
//! open/closed gating, triggered burst capture, and sample-counted
//! pass-through. Useful for controlled signal capture, triggered
//! recording, and conditional processing.
//! GNU Radio equivalent: `copy` / `valve`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::valve::{Valve, ValveMode};
//!
//! let mut valve = Valve::new(ValveMode::Open);
//! let input = vec![1.0, 2.0, 3.0, 4.0];
//! let output = valve.process(&input);
//! assert_eq!(output, input); // Passes through when open
//!
//! valve.set_mode(ValveMode::Closed);
//! let output = valve.process(&input);
//! assert!(output.is_empty()); // Blocks when closed
//! ```

/// Valve operating mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ValveMode {
    /// Pass all samples through.
    Open,
    /// Block all samples.
    Closed,
    /// Pass exactly N samples, then close.
    CountedBurst(usize),
    /// Pass samples only while triggered, then close.
    Triggered,
}

/// Stream valve / gate for flow control.
#[derive(Debug, Clone)]
pub struct Valve {
    /// Current operating mode.
    mode: ValveMode,
    /// Trigger state (for Triggered mode).
    triggered: bool,
    /// Samples remaining (for CountedBurst mode).
    remaining: usize,
    /// Total samples passed through.
    total_passed: usize,
    /// Total samples blocked.
    total_blocked: usize,
}

impl Valve {
    /// Create a new valve.
    pub fn new(mode: ValveMode) -> Self {
        let remaining = match mode {
            ValveMode::CountedBurst(n) => n,
            _ => 0,
        };
        Self {
            mode,
            triggered: false,
            remaining,
            total_passed: 0,
            total_blocked: 0,
        }
    }

    /// Create an open valve (pass-through).
    pub fn open() -> Self {
        Self::new(ValveMode::Open)
    }

    /// Create a closed valve (blocks all).
    pub fn closed() -> Self {
        Self::new(ValveMode::Closed)
    }

    /// Create a counted burst valve.
    pub fn counted(count: usize) -> Self {
        Self::new(ValveMode::CountedBurst(count))
    }

    /// Process a block of f64 samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        match self.mode {
            ValveMode::Open => {
                self.total_passed += input.len();
                input.to_vec()
            }
            ValveMode::Closed => {
                self.total_blocked += input.len();
                Vec::new()
            }
            ValveMode::CountedBurst(_) => {
                if self.remaining == 0 {
                    self.total_blocked += input.len();
                    return Vec::new();
                }
                let pass = input.len().min(self.remaining);
                self.remaining -= pass;
                self.total_passed += pass;
                self.total_blocked += input.len() - pass;
                input[..pass].to_vec()
            }
            ValveMode::Triggered => {
                if self.triggered {
                    self.total_passed += input.len();
                    input.to_vec()
                } else {
                    self.total_blocked += input.len();
                    Vec::new()
                }
            }
        }
    }

    /// Process a block of generic samples using a closure for cloning.
    pub fn process_generic<T: Clone>(&mut self, input: &[T]) -> Vec<T> {
        match self.mode {
            ValveMode::Open => {
                self.total_passed += input.len();
                input.to_vec()
            }
            ValveMode::Closed => {
                self.total_blocked += input.len();
                Vec::new()
            }
            ValveMode::CountedBurst(_) => {
                if self.remaining == 0 {
                    self.total_blocked += input.len();
                    return Vec::new();
                }
                let pass = input.len().min(self.remaining);
                self.remaining -= pass;
                self.total_passed += pass;
                self.total_blocked += input.len() - pass;
                input[..pass].to_vec()
            }
            ValveMode::Triggered => {
                if self.triggered {
                    self.total_passed += input.len();
                    input.to_vec()
                } else {
                    self.total_blocked += input.len();
                    Vec::new()
                }
            }
        }
    }

    /// Set the valve mode.
    pub fn set_mode(&mut self, mode: ValveMode) {
        self.mode = mode;
        if let ValveMode::CountedBurst(n) = mode {
            self.remaining = n;
        }
    }

    /// Get the current mode.
    pub fn mode(&self) -> ValveMode {
        self.mode
    }

    /// Set trigger state (for Triggered mode).
    pub fn set_trigger(&mut self, triggered: bool) {
        self.triggered = triggered;
    }

    /// Get trigger state.
    pub fn is_triggered(&self) -> bool {
        self.triggered
    }

    /// Check if the valve is currently passing samples.
    pub fn is_passing(&self) -> bool {
        match self.mode {
            ValveMode::Open => true,
            ValveMode::Closed => false,
            ValveMode::CountedBurst(_) => self.remaining > 0,
            ValveMode::Triggered => self.triggered,
        }
    }

    /// Get remaining sample count (CountedBurst mode).
    pub fn remaining(&self) -> usize {
        self.remaining
    }

    /// Total samples that passed through.
    pub fn total_passed(&self) -> usize {
        self.total_passed
    }

    /// Total samples that were blocked.
    pub fn total_blocked(&self) -> usize {
        self.total_blocked
    }

    /// Reset statistics and state.
    pub fn reset(&mut self) {
        self.triggered = false;
        self.total_passed = 0;
        self.total_blocked = 0;
        if let ValveMode::CountedBurst(n) = self.mode {
            self.remaining = n;
        }
    }
}

/// Gate a signal: pass samples where mask is true, zero where false.
pub fn gate_signal(signal: &[f64], mask: &[bool]) -> Vec<f64> {
    signal
        .iter()
        .zip(mask.iter().chain(std::iter::repeat(&false)))
        .map(|(&s, &m)| if m { s } else { 0.0 })
        .collect()
}

/// Extract active segments from a signal based on a mask.
///
/// Returns a vector of (start_index, samples) tuples.
pub fn extract_segments(signal: &[f64], mask: &[bool]) -> Vec<(usize, Vec<f64>)> {
    let mut segments = Vec::new();
    let mut in_segment = false;
    let mut start = 0;
    let mut current = Vec::new();

    let n = signal.len().min(mask.len());
    for i in 0..n {
        if mask[i] {
            if !in_segment {
                start = i;
                in_segment = true;
            }
            current.push(signal[i]);
        } else if in_segment {
            segments.push((start, std::mem::take(&mut current)));
            in_segment = false;
        }
    }
    if in_segment && !current.is_empty() {
        segments.push((start, current));
    }

    segments
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_open() {
        let mut v = Valve::open();
        let input = vec![1.0, 2.0, 3.0];
        let output = v.process(&input);
        assert_eq!(output, input);
        assert!(v.is_passing());
    }

    #[test]
    fn test_closed() {
        let mut v = Valve::closed();
        let input = vec![1.0, 2.0, 3.0];
        let output = v.process(&input);
        assert!(output.is_empty());
        assert!(!v.is_passing());
    }

    #[test]
    fn test_counted_burst() {
        let mut v = Valve::counted(5);
        let input = vec![1.0, 2.0, 3.0, 4.0];
        let out1 = v.process(&input); // 4 of 5
        assert_eq!(out1.len(), 4);
        assert_eq!(v.remaining(), 1);

        let out2 = v.process(&input); // 1 of remaining
        assert_eq!(out2.len(), 1);
        assert_eq!(v.remaining(), 0);

        let out3 = v.process(&input); // None left
        assert!(out3.is_empty());
        assert!(!v.is_passing());
    }

    #[test]
    fn test_triggered() {
        let mut v = Valve::new(ValveMode::Triggered);
        let input = vec![1.0, 2.0, 3.0];

        assert!(!v.is_passing());
        let out = v.process(&input);
        assert!(out.is_empty());

        v.set_trigger(true);
        assert!(v.is_passing());
        let out = v.process(&input);
        assert_eq!(out, input);

        v.set_trigger(false);
        let out = v.process(&input);
        assert!(out.is_empty());
    }

    #[test]
    fn test_mode_switch() {
        let mut v = Valve::open();
        let input = vec![1.0, 2.0];

        assert_eq!(v.process(&input).len(), 2);
        v.set_mode(ValveMode::Closed);
        assert!(v.process(&input).is_empty());
        v.set_mode(ValveMode::Open);
        assert_eq!(v.process(&input).len(), 2);
    }

    #[test]
    fn test_statistics() {
        let mut v = Valve::open();
        v.process(&[1.0, 2.0, 3.0]);
        assert_eq!(v.total_passed(), 3);
        assert_eq!(v.total_blocked(), 0);

        v.set_mode(ValveMode::Closed);
        v.process(&[4.0, 5.0]);
        assert_eq!(v.total_passed(), 3);
        assert_eq!(v.total_blocked(), 2);
    }

    #[test]
    fn test_generic() {
        let mut v = Valve::open();
        let input: Vec<i32> = vec![10, 20, 30];
        let output = v.process_generic(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_reset() {
        let mut v = Valve::counted(10);
        v.process(&[1.0; 5]);
        assert_eq!(v.remaining(), 5);
        v.reset();
        assert_eq!(v.remaining(), 10);
        assert_eq!(v.total_passed(), 0);
    }

    #[test]
    fn test_gate_signal() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let mask = vec![true, false, true, false];
        let gated = gate_signal(&signal, &mask);
        assert_eq!(gated, vec![1.0, 0.0, 3.0, 0.0]);
    }

    #[test]
    fn test_gate_short_mask() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let mask = vec![true, true]; // Shorter than signal
        let gated = gate_signal(&signal, &mask);
        assert_eq!(gated, vec![1.0, 2.0, 0.0, 0.0]);
    }

    #[test]
    fn test_extract_segments() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let mask = vec![false, true, true, false, true, false];
        let segs = extract_segments(&signal, &mask);
        assert_eq!(segs.len(), 2);
        assert_eq!(segs[0], (1, vec![2.0, 3.0]));
        assert_eq!(segs[1], (4, vec![5.0]));
    }

    #[test]
    fn test_extract_trailing_segment() {
        let signal = vec![1.0, 2.0, 3.0];
        let mask = vec![false, true, true]; // Active at end
        let segs = extract_segments(&signal, &mask);
        assert_eq!(segs.len(), 1);
        assert_eq!(segs[0], (1, vec![2.0, 3.0]));
    }

    #[test]
    fn test_empty() {
        let mut v = Valve::open();
        let out = v.process(&[]);
        assert!(out.is_empty());

        assert!(extract_segments(&[], &[]).is_empty());
        assert!(gate_signal(&[], &[]).is_empty());
    }
}

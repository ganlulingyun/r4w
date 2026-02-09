//! Regenerate BB — Bit regeneration / pulse stretcher
//!
//! When a 1 is detected, outputs 1 for the next N samples, then reverts
//! to 0. Useful for stretching trigger pulses, converting edge detects
//! to gate signals, and creating hold intervals for downstream processing.
//! GNU Radio equivalent: `regenerate_bb`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::regenerate_bb::RegenerateBB;
//!
//! let mut regen = RegenerateBB::new(3, 1);
//! let input  = vec![0, 1, 0, 0, 0, 0, 0, 1, 0, 0];
//! let output = regen.process(&input);
//! // After a 1, output stays 1 for 3 samples:
//! assert_eq!(output, vec![0, 1, 1, 1, 0, 0, 0, 1, 1, 1]);
//! ```

/// Bit regenerator / pulse stretcher.
#[derive(Debug, Clone)]
pub struct RegenerateBB {
    /// Duration to hold output high after trigger.
    period: usize,
    /// Samples until next allowed trigger (guard interval).
    guard: usize,
    /// Counter: samples remaining to output 1.
    hold_count: usize,
    /// Counter: samples remaining in guard interval.
    guard_count: usize,
}

impl RegenerateBB {
    /// Create a regenerator.
    ///
    /// - `period`: how many 1s to output per trigger (including the trigger sample)
    /// - `guard`: minimum samples between triggers (0 = no guard, re-trigger immediately)
    pub fn new(period: usize, guard: usize) -> Self {
        Self {
            period: period.max(1),
            guard,
            hold_count: 0,
            guard_count: 0,
        }
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: u8) -> u8 {
        // Decrement guard
        if self.guard_count > 0 {
            self.guard_count -= 1;
        }

        // Check for trigger
        if x != 0 && self.hold_count == 0 && self.guard_count == 0 {
            self.hold_count = self.period;
            self.guard_count = self.guard;
        }

        // Output
        if self.hold_count > 0 {
            self.hold_count -= 1;
            1
        } else {
            0
        }
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x));
        }
        output
    }

    /// Process boolean input/output.
    pub fn process_bool(&mut self, input: &[bool]) -> Vec<bool> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x as u8) != 0);
        }
        output
    }

    /// Process f64 input (threshold at 0.5) and f64 output (0.0 or 1.0).
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            let trigger = if x >= 0.5 { 1u8 } else { 0u8 };
            output.push(self.process_sample(trigger) as f64);
        }
        output
    }

    /// Get period.
    pub fn period(&self) -> usize {
        self.period
    }

    /// Set period.
    pub fn set_period(&mut self, period: usize) {
        self.period = period.max(1);
    }

    /// Get guard interval.
    pub fn guard(&self) -> usize {
        self.guard
    }

    /// Set guard interval.
    pub fn set_guard(&mut self, guard: usize) {
        self.guard = guard;
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.hold_count = 0;
        self.guard_count = 0;
    }
}

/// One-shot pulse generator: outputs a pulse of `width` samples
/// at the specified `delay` from the start, then zeros forever.
#[derive(Debug, Clone)]
pub struct PulseGenerator {
    delay: usize,
    width: usize,
    count: usize,
}

impl PulseGenerator {
    /// Create a pulse generator.
    pub fn new(delay: usize, width: usize) -> Self {
        Self {
            delay,
            width: width.max(1),
            count: 0,
        }
    }

    /// Generate the next N samples.
    pub fn generate(&mut self, n: usize) -> Vec<f64> {
        let mut output = Vec::with_capacity(n);
        for _ in 0..n {
            if self.count >= self.delay && self.count < self.delay + self.width {
                output.push(1.0);
            } else {
                output.push(0.0);
            }
            self.count += 1;
        }
        output
    }

    /// Reset to start.
    pub fn reset(&mut self) {
        self.count = 0;
    }
}

/// Create a rectangular pulse of given width.
pub fn rect_pulse(width: usize) -> Vec<f64> {
    vec![1.0; width]
}

/// Create a rectangular pulse with rising/falling edges of given length.
pub fn trapezoidal_pulse(width: usize, edge_len: usize) -> Vec<f64> {
    let edge = edge_len.min(width / 2);
    let flat = width.saturating_sub(2 * edge);
    let mut pulse = Vec::with_capacity(width);
    // Rising edge
    for i in 0..edge {
        pulse.push((i + 1) as f64 / (edge + 1) as f64);
    }
    // Flat top
    for _ in 0..flat {
        pulse.push(1.0);
    }
    // Falling edge
    for i in 0..edge {
        pulse.push((edge - i) as f64 / (edge + 1) as f64);
    }
    pulse
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_regenerate() {
        let mut regen = RegenerateBB::new(3, 0);
        let input = vec![0, 1, 0, 0, 0, 0];
        let output = regen.process(&input);
        assert_eq!(output, vec![0, 1, 1, 1, 0, 0]);
    }

    #[test]
    fn test_multiple_triggers() {
        let mut regen = RegenerateBB::new(2, 0);
        let input = vec![0, 1, 0, 0, 1, 0, 0];
        let output = regen.process(&input);
        assert_eq!(output, vec![0, 1, 1, 0, 1, 1, 0]);
    }

    #[test]
    fn test_overlapping_triggers() {
        let mut regen = RegenerateBB::new(3, 0);
        // Second trigger at sample 2, while still holding from sample 1
        let input = vec![0, 1, 1, 0, 0, 0];
        let output = regen.process(&input);
        // First trigger: hold at 1,2,3. Second trigger ignored (hold_count > 0)
        assert_eq!(output, vec![0, 1, 1, 1, 0, 0]);
    }

    #[test]
    fn test_guard_interval() {
        let mut regen = RegenerateBB::new(2, 3);
        let input = vec![0, 1, 0, 0, 1, 0, 0, 0, 1, 0];
        let output = regen.process(&input);
        // First trigger at 1: hold 1,2. Guard=3 (samples 2,3,4 decrement).
        // At sample 4: guard decrements 1→0, then trigger check passes → retriggers.
        // Trigger at 8: guard has expired, triggers.
        assert_eq!(output, vec![0, 1, 1, 0, 1, 1, 0, 0, 1, 1]);
    }

    #[test]
    fn test_period_one() {
        let mut regen = RegenerateBB::new(1, 0);
        let input = vec![0, 1, 0, 1, 0];
        let output = regen.process(&input);
        assert_eq!(output, vec![0, 1, 0, 1, 0]);
    }

    #[test]
    fn test_bool_interface() {
        let mut regen = RegenerateBB::new(2, 0);
        let input = vec![false, true, false, false];
        let output = regen.process_bool(&input);
        assert_eq!(output, vec![false, true, true, false]);
    }

    #[test]
    fn test_real_interface() {
        let mut regen = RegenerateBB::new(2, 0);
        let input = vec![0.0, 0.9, 0.1, 0.0];
        let output = regen.process_real(&input);
        assert_eq!(output, vec![0.0, 1.0, 1.0, 0.0]);
    }

    #[test]
    fn test_reset() {
        let mut regen = RegenerateBB::new(5, 0);
        regen.process(&[1]); // Start hold
        regen.reset();
        let output = regen.process(&[0, 0]);
        assert_eq!(output, vec![0, 0]); // Hold was reset
    }

    #[test]
    fn test_set_period() {
        let mut regen = RegenerateBB::new(2, 0);
        regen.set_period(4);
        assert_eq!(regen.period(), 4);
    }

    #[test]
    fn test_pulse_generator() {
        let mut gen = PulseGenerator::new(2, 3);
        let out = gen.generate(8);
        assert_eq!(out, vec![0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_pulse_generator_reset() {
        let mut gen = PulseGenerator::new(1, 2);
        gen.generate(5);
        gen.reset();
        let out = gen.generate(5);
        assert_eq!(out, vec![0.0, 1.0, 1.0, 0.0, 0.0]);
    }

    #[test]
    fn test_rect_pulse() {
        let p = rect_pulse(4);
        assert_eq!(p, vec![1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn test_trapezoidal_pulse() {
        let p = trapezoidal_pulse(6, 2);
        assert_eq!(p.len(), 6); // 2 rising + 2 flat + 2 falling
        assert!(p[0] > 0.0 && p[0] < 1.0); // Rising edge
        assert!(p[1] > p[0]); // Still rising
        assert_eq!(p[2], 1.0); // Flat top
        assert_eq!(p[3], 1.0); // Flat top
        assert!(p[4] < 1.0 && p[4] > 0.0); // Falling edge
        assert!(p[5] < p[4]); // Still falling
    }

    #[test]
    fn test_empty_input() {
        let mut regen = RegenerateBB::new(3, 0);
        assert!(regen.process(&[]).is_empty());
    }

    #[test]
    fn test_all_triggers() {
        let mut regen = RegenerateBB::new(1, 0);
        let input = vec![1, 1, 1, 1];
        let output = regen.process(&input);
        assert_eq!(output, vec![1, 1, 1, 1]);
    }
}

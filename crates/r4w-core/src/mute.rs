//! Mute — Mute or unmute a signal stream
//!
//! Passes or blocks samples based on a mute flag. When muted, outputs
//! zeros (or nothing). Useful for squelch gating, push-to-talk, timed
//! blanking, and noise blanking.
//! GNU Radio equivalent: `mute_ff`, `mute_cc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mute::Mute;
//! use num_complex::Complex64;
//!
//! let mut mute = Mute::new(false); // Start unmuted
//! let signal = vec![Complex64::new(1.0, 0.0); 4];
//!
//! let out = mute.process(&signal);
//! assert_eq!(out[0], Complex64::new(1.0, 0.0)); // Passes through
//!
//! mute.set_muted(true);
//! let out = mute.process(&signal);
//! assert_eq!(out[0], Complex64::new(0.0, 0.0)); // Zeroed
//! ```

use num_complex::Complex64;

/// Mute block for complex signals.
#[derive(Debug, Clone)]
pub struct Mute {
    muted: bool,
}

impl Mute {
    /// Create a mute block.
    pub fn new(muted: bool) -> Self {
        Self { muted }
    }

    /// Process complex samples.
    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        if self.muted {
            vec![Complex64::new(0.0, 0.0); input.len()]
        } else {
            input.to_vec()
        }
    }

    /// Process complex samples in-place.
    pub fn process_inplace(&self, data: &mut [Complex64]) {
        if self.muted {
            data.fill(Complex64::new(0.0, 0.0));
        }
    }

    /// Process a single complex sample.
    #[inline]
    pub fn process_sample(&self, x: Complex64) -> Complex64 {
        if self.muted {
            Complex64::new(0.0, 0.0)
        } else {
            x
        }
    }

    /// Check if muted.
    pub fn is_muted(&self) -> bool {
        self.muted
    }

    /// Set mute state.
    pub fn set_muted(&mut self, muted: bool) {
        self.muted = muted;
    }

    /// Toggle mute state.
    pub fn toggle(&mut self) {
        self.muted = !self.muted;
    }
}

/// Mute block for real signals.
#[derive(Debug, Clone)]
pub struct MuteReal {
    muted: bool,
}

impl MuteReal {
    /// Create a mute block for real signals.
    pub fn new(muted: bool) -> Self {
        Self { muted }
    }

    /// Process real samples.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        if self.muted {
            vec![0.0; input.len()]
        } else {
            input.to_vec()
        }
    }

    /// Process in-place.
    pub fn process_inplace(&self, data: &mut [f64]) {
        if self.muted {
            data.fill(0.0);
        }
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&self, x: f64) -> f64 {
        if self.muted { 0.0 } else { x }
    }

    /// Check if muted.
    pub fn is_muted(&self) -> bool {
        self.muted
    }

    /// Set mute state.
    pub fn set_muted(&mut self, muted: bool) {
        self.muted = muted;
    }

    /// Toggle mute state.
    pub fn toggle(&mut self) {
        self.muted = !self.muted;
    }
}

/// Timed mute: mutes for a specified number of samples after trigger.
#[derive(Debug, Clone)]
pub struct TimedMute {
    /// Remaining samples to mute.
    remaining: usize,
}

impl TimedMute {
    /// Create a timed mute (initially unmuted).
    pub fn new() -> Self {
        Self { remaining: 0 }
    }

    /// Trigger muting for `n` samples.
    pub fn trigger(&mut self, n: usize) {
        self.remaining = n;
    }

    /// Process complex samples, muting triggered portions.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            if self.remaining > 0 {
                output.push(Complex64::new(0.0, 0.0));
                self.remaining -= 1;
            } else {
                output.push(x);
            }
        }
        output
    }

    /// Process real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            if self.remaining > 0 {
                output.push(0.0);
                self.remaining -= 1;
            } else {
                output.push(x);
            }
        }
        output
    }

    /// Check if currently muting.
    pub fn is_muted(&self) -> bool {
        self.remaining > 0
    }

    /// Remaining muted samples.
    pub fn remaining(&self) -> usize {
        self.remaining
    }

    /// Reset (unmute immediately).
    pub fn reset(&mut self) {
        self.remaining = 0;
    }
}

impl Default for TimedMute {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unmuted_passthrough() {
        let mute = Mute::new(false);
        let input = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let output = mute.process(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_muted_zeros() {
        let mute = Mute::new(true);
        let input = vec![Complex64::new(1.0, 2.0); 3];
        let output = mute.process(&input);
        for z in &output {
            assert_eq!(*z, Complex64::new(0.0, 0.0));
        }
    }

    #[test]
    fn test_toggle() {
        let mut mute = Mute::new(false);
        assert!(!mute.is_muted());
        mute.toggle();
        assert!(mute.is_muted());
        mute.toggle();
        assert!(!mute.is_muted());
    }

    #[test]
    fn test_set_muted() {
        let mut mute = Mute::new(false);
        mute.set_muted(true);
        assert!(mute.is_muted());
    }

    #[test]
    fn test_inplace() {
        let mute = Mute::new(true);
        let mut data = vec![Complex64::new(5.0, 6.0); 2];
        mute.process_inplace(&mut data);
        assert_eq!(data[0], Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_real_mute() {
        let mute = MuteReal::new(true);
        let output = mute.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_real_unmute() {
        let mute = MuteReal::new(false);
        let input = vec![1.0, 2.0, 3.0];
        assert_eq!(mute.process(&input), input);
    }

    #[test]
    fn test_real_toggle() {
        let mut mute = MuteReal::new(false);
        mute.toggle();
        assert!(mute.is_muted());
    }

    #[test]
    fn test_real_inplace() {
        let mute = MuteReal::new(true);
        let mut data = vec![5.0, 6.0];
        mute.process_inplace(&mut data);
        assert_eq!(data, vec![0.0, 0.0]);
    }

    #[test]
    fn test_timed_mute() {
        let mut tm = TimedMute::new();
        assert!(!tm.is_muted());

        tm.trigger(3);
        assert!(tm.is_muted());

        let input = vec![Complex64::new(1.0, 0.0); 5];
        let output = tm.process(&input);
        // First 3 muted, last 2 pass through
        assert_eq!(output[0], Complex64::new(0.0, 0.0));
        assert_eq!(output[2], Complex64::new(0.0, 0.0));
        assert_eq!(output[3], Complex64::new(1.0, 0.0));
        assert_eq!(output[4], Complex64::new(1.0, 0.0));
        assert!(!tm.is_muted());
    }

    #[test]
    fn test_timed_mute_real() {
        let mut tm = TimedMute::new();
        tm.trigger(2);
        let output = tm.process_real(&[5.0, 5.0, 5.0, 5.0]);
        assert_eq!(output, vec![0.0, 0.0, 5.0, 5.0]);
    }

    #[test]
    fn test_timed_mute_reset() {
        let mut tm = TimedMute::new();
        tm.trigger(100);
        tm.reset();
        assert!(!tm.is_muted());
        assert_eq!(tm.remaining(), 0);
    }

    #[test]
    fn test_empty() {
        let mute = Mute::new(true);
        assert!(mute.process(&[]).is_empty());

        let mute_real = MuteReal::new(true);
        assert!(mute_real.process(&[]).is_empty());
    }

    #[test]
    fn test_single_sample() {
        let mute = Mute::new(false);
        let out = mute.process_sample(Complex64::new(42.0, 0.0));
        assert_eq!(out, Complex64::new(42.0, 0.0));

        let mute = Mute::new(true);
        let out = mute.process_sample(Complex64::new(42.0, 0.0));
        assert_eq!(out, Complex64::new(0.0, 0.0));
    }
}

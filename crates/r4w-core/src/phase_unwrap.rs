//! Phase Unwrap — Remove 2π discontinuities from phase signals
//!
//! Unwraps phase signals by detecting and correcting jumps greater than π.
//! Essential for phase tracking, FM demodulation, and phase measurements.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::phase_unwrap::PhaseUnwrap;
//! use std::f64::consts::PI;
//!
//! let mut unwrap = PhaseUnwrap::new();
//! // Phase that wraps from π to -π
//! let wrapped = vec![0.0, 1.0, 2.0, 3.0, -3.0, -2.0, -1.0, 0.0];
//! let unwrapped = unwrap.process(&wrapped);
//! // Should be monotonically increasing
//! for i in 1..unwrapped.len() {
//!     assert!(unwrapped[i] > unwrapped[i-1] - 0.1);
//! }
//! ```

use std::f64::consts::PI;

/// Phase unwrapper.
#[derive(Debug, Clone)]
pub struct PhaseUnwrap {
    /// Accumulated correction.
    correction: f64,
    /// Previous input sample.
    prev: f64,
    /// Whether initialized.
    initialized: bool,
    /// Tolerance for detecting wraps (default: π).
    tolerance: f64,
}

impl PhaseUnwrap {
    /// Create a new phase unwrapper.
    pub fn new() -> Self {
        Self {
            correction: 0.0,
            prev: 0.0,
            initialized: false,
            tolerance: PI,
        }
    }

    /// Create with custom tolerance.
    pub fn with_tolerance(tolerance: f64) -> Self {
        Self {
            tolerance,
            ..Self::new()
        }
    }

    /// Process a block of phase samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x));
        }
        output
    }

    /// Process a single phase sample.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        if !self.initialized {
            self.prev = x;
            self.initialized = true;
            return x;
        }

        let diff = x - self.prev;
        if diff > self.tolerance {
            self.correction -= 2.0 * PI;
        } else if diff < -self.tolerance {
            self.correction += 2.0 * PI;
        }
        self.prev = x;
        x + self.correction
    }

    /// Get current correction.
    pub fn correction(&self) -> f64 {
        self.correction
    }

    /// Reset the unwrapper.
    pub fn reset(&mut self) {
        self.correction = 0.0;
        self.prev = 0.0;
        self.initialized = false;
    }
}

impl Default for PhaseUnwrap {
    fn default() -> Self {
        Self::new()
    }
}

/// Wrap phase to [-π, π] range.
pub fn wrap_phase(phase: f64) -> f64 {
    let mut p = phase % (2.0 * PI);
    if p > PI {
        p -= 2.0 * PI;
    } else if p < -PI {
        p += 2.0 * PI;
    }
    p
}

/// Wrap a vector of phases to [-π, π].
pub fn wrap_phase_vec(phases: &[f64]) -> Vec<f64> {
    phases.iter().map(|&p| wrap_phase(p)).collect()
}

/// Compute phase difference between two samples, wrapped to [-π, π].
pub fn phase_diff(a: f64, b: f64) -> f64 {
    wrap_phase(a - b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_wrap() {
        let mut u = PhaseUnwrap::new();
        let input = vec![0.0, 0.5, 1.0, 1.5, 2.0];
        let output = u.process(&input);
        assert_eq!(output, input); // No wraps, no change
    }

    #[test]
    fn test_positive_wrap() {
        let mut u = PhaseUnwrap::new();
        // Phase increases past π, wraps to -π+ε
        let input = vec![2.5, 3.0, -3.0, -2.5]; // wrap at π
        let output = u.process(&input);
        // Should be monotonically increasing
        assert!(output[2] > output[1], "expected {} > {}", output[2], output[1]);
        assert!(output[3] > output[2]);
    }

    #[test]
    fn test_negative_wrap() {
        let mut u = PhaseUnwrap::new();
        // Phase decreases past -π, wraps to π-ε
        let input = vec![-2.5, -3.0, 3.0, 2.5];
        let output = u.process(&input);
        // Should be monotonically decreasing
        assert!(output[2] < output[1]);
        assert!(output[3] < output[2]);
    }

    #[test]
    fn test_linear_phase() {
        let mut u = PhaseUnwrap::new();
        // Create wrapped linear phase
        let n = 100;
        let freq = 0.3; // rad/sample
        let input: Vec<f64> = (0..n).map(|i| wrap_phase(freq * i as f64)).collect();
        let output = u.process(&input);
        // Should be approximately linear after unwrapping
        for i in 2..n {
            let diff = output[i] - output[i - 1];
            assert!((diff - freq).abs() < 0.01, "non-linear at {i}: diff={diff}");
        }
    }

    #[test]
    fn test_reset() {
        let mut u = PhaseUnwrap::new();
        u.process(&[0.0, 3.0, -3.0]);
        u.reset();
        assert_eq!(u.correction(), 0.0);
        let out = u.process_sample(1.0);
        assert_eq!(out, 1.0);
    }

    #[test]
    fn test_single_sample() {
        let mut u = PhaseUnwrap::new();
        assert_eq!(u.process_sample(1.5), 1.5);
    }

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0)).abs() < 1e-10);
        assert!((wrap_phase(PI) - PI).abs() < 1e-10);
        assert!((wrap_phase(2.0 * PI)).abs() < 1e-10);
        assert!((wrap_phase(3.0 * PI) - PI).abs() < 1e-10);
        assert!((wrap_phase(-3.0 * PI) - (-PI)).abs() < 1e-10);
    }

    #[test]
    fn test_wrap_phase_vec() {
        let input = vec![0.0, PI, 2.0 * PI, 3.0 * PI];
        let wrapped = wrap_phase_vec(&input);
        assert!(wrapped[0].abs() < 1e-10);
        assert!((wrapped[2]).abs() < 1e-10);
    }

    #[test]
    fn test_phase_diff() {
        assert!((phase_diff(0.1, 0.0) - 0.1).abs() < 1e-10);
        // Across wrap boundary
        let d = phase_diff(-3.0, 3.0);
        assert!(d.abs() < PI + 0.1);
    }

    #[test]
    fn test_custom_tolerance() {
        let mut u = PhaseUnwrap::with_tolerance(1.0); // Tighter tolerance
        let input = vec![0.0, 1.5, -1.5]; // Jump of 3.0 > tolerance
        let output = u.process(&input);
        assert!(output[2] > output[1]); // Should unwrap
    }

    #[test]
    fn test_empty() {
        let mut u = PhaseUnwrap::new();
        let output = u.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_correction_accessor() {
        let mut u = PhaseUnwrap::new();
        u.process(&[3.0, -3.0]); // One positive wrap
        assert!((u.correction() - 2.0 * PI).abs() < 1e-10);
    }
}

//! Integrate — Running integrator / accumulator
//!
//! Accumulates input samples over configurable integration windows.
//! Supports running (IIR) and windowed (FIR) integration modes.
//! GNU Radio equivalent: `integrate_ff`, `integrate_cc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::integrate::{Integrate, WindowedIntegrate};
//!
//! // Running integrator: each output = sum of N inputs
//! let mut integ = Integrate::new(4);
//! let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
//! let output = integ.process(&input);
//! assert_eq!(output, vec![10.0, 26.0]); // [1+2+3+4, 5+6+7+8]
//!
//! // Windowed (sliding) integrator
//! let mut winteg = WindowedIntegrate::new(3);
//! let output = winteg.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
//! assert_eq!(output, vec![6.0, 9.0, 12.0]); // sliding sum of 3
//! ```

use num_complex::Complex64;

/// Running integrator — sums N input samples to produce one output.
///
/// Decimates by factor N: for every N inputs, emits one output that is
/// the sum of those N samples. This is the GNU Radio `integrate` block.
#[derive(Debug, Clone)]
pub struct Integrate {
    /// Integration length (decimation factor).
    n: usize,
    /// Accumulator.
    accum: f64,
    /// Current sample index within window.
    index: usize,
}

impl Integrate {
    /// Create an integrator that sums `n` samples per output.
    pub fn new(n: usize) -> Self {
        Self {
            n: n.max(1),
            accum: 0.0,
            index: 0,
        }
    }

    /// Process a block of samples.
    ///
    /// Returns one output for every `n` input samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.n + 1);
        for &x in input {
            self.accum += x;
            self.index += 1;
            if self.index >= self.n {
                output.push(self.accum);
                self.accum = 0.0;
                self.index = 0;
            }
        }
        output
    }

    /// Get current accumulator value.
    pub fn accumulator(&self) -> f64 {
        self.accum
    }

    /// Get integration length.
    pub fn n(&self) -> usize {
        self.n
    }

    /// Set integration length and reset.
    pub fn set_n(&mut self, n: usize) {
        self.n = n.max(1);
        self.reset();
    }

    /// Reset accumulator and index.
    pub fn reset(&mut self) {
        self.accum = 0.0;
        self.index = 0;
    }
}

/// Complex running integrator.
#[derive(Debug, Clone)]
pub struct IntegrateComplex {
    /// Integration length.
    n: usize,
    /// Accumulator.
    accum: Complex64,
    /// Current index.
    index: usize,
}

impl IntegrateComplex {
    /// Create a complex integrator.
    pub fn new(n: usize) -> Self {
        Self {
            n: n.max(1),
            accum: Complex64::new(0.0, 0.0),
            index: 0,
        }
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.n + 1);
        for &x in input {
            self.accum += x;
            self.index += 1;
            if self.index >= self.n {
                output.push(self.accum);
                self.accum = Complex64::new(0.0, 0.0);
                self.index = 0;
            }
        }
        output
    }

    /// Get current accumulator.
    pub fn accumulator(&self) -> Complex64 {
        self.accum
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.accum = Complex64::new(0.0, 0.0);
        self.index = 0;
    }
}

/// Windowed (sliding) integrator — moving sum over a window.
///
/// Unlike `Integrate` which decimates, this produces one output per
/// input sample, where each output is the sum of the last N samples.
#[derive(Debug, Clone)]
pub struct WindowedIntegrate {
    /// Window size.
    n: usize,
    /// Circular buffer of recent samples.
    buffer: Vec<f64>,
    /// Write index in buffer.
    write_idx: usize,
    /// Running sum.
    sum: f64,
    /// Samples seen (for initial fill).
    count: usize,
}

impl WindowedIntegrate {
    /// Create a windowed integrator with window size `n`.
    pub fn new(n: usize) -> Self {
        let n = n.max(1);
        Self {
            n,
            buffer: vec![0.0; n],
            write_idx: 0,
            sum: 0.0,
            count: 0,
        }
    }

    /// Process samples. Outputs start once window is full.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            // Subtract oldest, add newest
            self.sum -= self.buffer[self.write_idx];
            self.buffer[self.write_idx] = x;
            self.sum += x;
            self.write_idx = (self.write_idx + 1) % self.n;
            self.count += 1;
            if self.count >= self.n {
                output.push(self.sum);
            }
        }
        output
    }

    /// Get current sum.
    pub fn current_sum(&self) -> f64 {
        self.sum
    }

    /// Get window size.
    pub fn n(&self) -> usize {
        self.n
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_idx = 0;
        self.sum = 0.0;
        self.count = 0;
    }
}

/// Leaky integrator — exponential moving sum.
///
/// `output[n] = alpha * input[n] + (1 - alpha) * output[n-1]`
///
/// Equivalent to a single-pole IIR lowpass filter.
#[derive(Debug, Clone)]
pub struct LeakyIntegrate {
    /// Smoothing factor (0 < alpha <= 1).
    alpha: f64,
    /// Current state.
    state: f64,
}

impl LeakyIntegrate {
    /// Create a leaky integrator.
    ///
    /// `alpha`: smoothing factor. Larger = faster response. Range (0, 1].
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.001, 1.0),
            state: 0.0,
        }
    }

    /// Create from time constant in samples.
    ///
    /// `tau`: time constant. `alpha = 1.0 / tau`.
    pub fn from_tau(tau: f64) -> Self {
        Self::new(1.0 / tau.max(1.0))
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            self.state = self.alpha * x + (1.0 - self.alpha) * self.state;
            output.push(self.state);
        }
        output
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        self.state = self.alpha * x + (1.0 - self.alpha) * self.state;
        self.state
    }

    /// Get current state.
    pub fn state(&self) -> f64 {
        self.state
    }

    /// Get alpha.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // -- Integrate tests --

    #[test]
    fn test_integrate_basic() {
        let mut integ = Integrate::new(4);
        let output = integ.process(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(output, vec![10.0]);
    }

    #[test]
    fn test_integrate_multiple_outputs() {
        let mut integ = Integrate::new(2);
        let output = integ.process(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert_eq!(output, vec![3.0, 7.0, 11.0]);
    }

    #[test]
    fn test_integrate_streaming() {
        let mut integ = Integrate::new(4);
        let o1 = integ.process(&[1.0, 2.0]);
        assert!(o1.is_empty());
        assert_eq!(integ.accumulator(), 3.0);
        let o2 = integ.process(&[3.0, 4.0]);
        assert_eq!(o2, vec![10.0]);
    }

    #[test]
    fn test_integrate_partial() {
        let mut integ = Integrate::new(3);
        let output = integ.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(output, vec![6.0]); // only first 3 sum
        assert_eq!(integ.accumulator(), 9.0); // 4+5 pending
    }

    #[test]
    fn test_integrate_one() {
        let mut integ = Integrate::new(1);
        let output = integ.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_integrate_reset() {
        let mut integ = Integrate::new(4);
        integ.process(&[1.0, 2.0]);
        integ.reset();
        assert_eq!(integ.accumulator(), 0.0);
    }

    // -- IntegrateComplex tests --

    #[test]
    fn test_integrate_complex() {
        let mut integ = IntegrateComplex::new(2);
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(0.0, 2.0),
        ];
        let output = integ.process(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[0].im - 1.0).abs() < 1e-10);
        assert!((output[1].re - 2.0).abs() < 1e-10);
        assert!((output[1].im - 2.0).abs() < 1e-10);
    }

    // -- WindowedIntegrate tests --

    #[test]
    fn test_windowed_basic() {
        let mut winteg = WindowedIntegrate::new(3);
        let output = winteg.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(output.len(), 3);
        assert!((output[0] - 6.0).abs() < 1e-10); // 1+2+3
        assert!((output[1] - 9.0).abs() < 1e-10); // 2+3+4
        assert!((output[2] - 12.0).abs() < 1e-10); // 3+4+5
    }

    #[test]
    fn test_windowed_one() {
        let mut winteg = WindowedIntegrate::new(1);
        let output = winteg.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_windowed_streaming() {
        let mut winteg = WindowedIntegrate::new(2);
        let o1 = winteg.process(&[1.0]);
        assert!(o1.is_empty()); // need 2 to fill window
        let o2 = winteg.process(&[2.0, 3.0]);
        assert_eq!(o2.len(), 2);
        assert!((o2[0] - 3.0).abs() < 1e-10); // 1+2
        assert!((o2[1] - 5.0).abs() < 1e-10); // 2+3
    }

    #[test]
    fn test_windowed_reset() {
        let mut winteg = WindowedIntegrate::new(3);
        winteg.process(&[1.0, 2.0, 3.0]);
        winteg.reset();
        assert_eq!(winteg.current_sum(), 0.0);
    }

    // -- LeakyIntegrate tests --

    #[test]
    fn test_leaky_step_response() {
        let mut leaky = LeakyIntegrate::new(0.1);
        // Step input: should approach 1.0
        let input = vec![1.0; 100];
        let output = leaky.process(&input);
        assert!(output[99] > 0.99);
        assert!(output[0] < 0.2);
    }

    #[test]
    fn test_leaky_alpha_one() {
        let mut leaky = LeakyIntegrate::new(1.0);
        let output = leaky.process(&[5.0, 3.0, 7.0]);
        assert_eq!(output, vec![5.0, 3.0, 7.0]); // No smoothing
    }

    #[test]
    fn test_leaky_from_tau() {
        let leaky = LeakyIntegrate::from_tau(10.0);
        assert!((leaky.alpha() - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_leaky_reset() {
        let mut leaky = LeakyIntegrate::new(0.5);
        leaky.process_sample(10.0);
        assert!(leaky.state() > 0.0);
        leaky.reset();
        assert_eq!(leaky.state(), 0.0);
    }
}

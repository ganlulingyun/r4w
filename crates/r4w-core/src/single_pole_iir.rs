//! Single-Pole IIR Filter — Exponential moving average
//!
//! Lightweight first-order IIR filter for signal smoothing, power
//! averaging, and DC tracking. Much cheaper than full IIR filters.
//! GNU Radio equivalent: `single_pole_iir_filter_ff`, `_cc`.
//!
//! Transfer function: `H(z) = α / (1 - (1-α)z⁻¹)`
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::single_pole_iir::{SinglePoleIir, SinglePoleIirComplex};
//! use num_complex::Complex64;
//!
//! // Real smoothing filter
//! let mut iir = SinglePoleIir::new(0.1);
//! let step = vec![1.0; 50];
//! let output = iir.process(&step);
//! // First sample initializes state to input, then smooths
//! assert!((output[0] - 1.0).abs() < 1e-10); // Initialized to input
//! assert!(output[49] > 0.99); // Converged
//!
//! // From time constant (samples)
//! let iir2 = SinglePoleIir::from_tau(10.0);
//! assert!((iir2.alpha() - 0.1).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Real-valued single-pole IIR filter.
///
/// `y[n] = α·x[n] + (1-α)·y[n-1]`
#[derive(Debug, Clone)]
pub struct SinglePoleIir {
    /// Smoothing coefficient (0 < α ≤ 1).
    alpha: f64,
    /// One minus alpha (cached).
    one_minus_alpha: f64,
    /// Filter state.
    state: f64,
    /// Whether state has been initialized.
    initialized: bool,
}

impl SinglePoleIir {
    /// Create with smoothing factor α.
    ///
    /// α = 1.0: no smoothing (output = input).
    /// α → 0: maximum smoothing (very slow response).
    pub fn new(alpha: f64) -> Self {
        let alpha = alpha.clamp(1e-10, 1.0);
        Self {
            alpha,
            one_minus_alpha: 1.0 - alpha,
            state: 0.0,
            initialized: false,
        }
    }

    /// Create from time constant in samples.
    ///
    /// `tau` = number of samples for ~63% step response.
    /// `α = 1/tau`.
    pub fn from_tau(tau: f64) -> Self {
        Self::new(1.0 / tau.max(1.0))
    }

    /// Create from 3dB bandwidth.
    ///
    /// `alpha = 1 - exp(-2π·f_3dB/sample_rate)`
    pub fn from_bandwidth(f_3db: f64, sample_rate: f64) -> Self {
        let alpha = 1.0 - (-2.0 * std::f64::consts::PI * f_3db / sample_rate).exp();
        Self::new(alpha)
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            if !self.initialized {
                self.state = x;
                self.initialized = true;
            } else {
                self.state = self.alpha * x + self.one_minus_alpha * self.state;
            }
            output.push(self.state);
        }
        output
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        if !self.initialized {
            self.state = x;
            self.initialized = true;
        } else {
            self.state = self.alpha * x + self.one_minus_alpha * self.state;
        }
        self.state
    }

    /// Get current filter output.
    pub fn state(&self) -> f64 {
        self.state
    }

    /// Get alpha.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Set alpha.
    pub fn set_alpha(&mut self, alpha: f64) {
        self.alpha = alpha.clamp(1e-10, 1.0);
        self.one_minus_alpha = 1.0 - self.alpha;
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.state = 0.0;
        self.initialized = false;
    }

    /// Set filter state to a specific value.
    pub fn set_state(&mut self, state: f64) {
        self.state = state;
        self.initialized = true;
    }

    /// Get the equivalent -3dB frequency for a given sample rate.
    pub fn bandwidth(&self, sample_rate: f64) -> f64 {
        // α = 1 - exp(-2π·f/fs) → f = -fs/(2π)·ln(1-α)
        -sample_rate / (2.0 * std::f64::consts::PI) * (1.0 - self.alpha).ln()
    }
}

/// Complex-valued single-pole IIR filter.
///
/// `y[n] = α·x[n] + (1-α)·y[n-1]`
#[derive(Debug, Clone)]
pub struct SinglePoleIirComplex {
    /// Smoothing coefficient.
    alpha: f64,
    /// One minus alpha.
    one_minus_alpha: f64,
    /// Filter state.
    state: Complex64,
    /// Whether initialized.
    initialized: bool,
}

impl SinglePoleIirComplex {
    /// Create a complex single-pole IIR filter.
    pub fn new(alpha: f64) -> Self {
        let alpha = alpha.clamp(1e-10, 1.0);
        Self {
            alpha,
            one_minus_alpha: 1.0 - alpha,
            state: Complex64::new(0.0, 0.0),
            initialized: false,
        }
    }

    /// Create from time constant.
    pub fn from_tau(tau: f64) -> Self {
        Self::new(1.0 / tau.max(1.0))
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            if !self.initialized {
                self.state = x;
                self.initialized = true;
            } else {
                self.state = x * self.alpha + self.state * self.one_minus_alpha;
            }
            output.push(self.state);
        }
        output
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: Complex64) -> Complex64 {
        if !self.initialized {
            self.state = x;
            self.initialized = true;
        } else {
            self.state = x * self.alpha + self.state * self.one_minus_alpha;
        }
        self.state
    }

    /// Get current state.
    pub fn state(&self) -> Complex64 {
        self.state
    }

    /// Get alpha.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.state = Complex64::new(0.0, 0.0);
        self.initialized = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_step_response() {
        let mut iir = SinglePoleIir::new(0.1);
        let step = vec![1.0; 100];
        let output = iir.process(&step);
        // First output initialized to input
        assert!((output[0] - 1.0).abs() < 1e-10);
        // Should converge toward 1.0
        assert!(output[99] > 0.999);
    }

    #[test]
    fn test_alpha_one_passthrough() {
        let mut iir = SinglePoleIir::new(1.0);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let output = iir.process(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_small_alpha_smoothing() {
        let mut iir = SinglePoleIir::new(0.01);
        let input = vec![0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0];
        let output = iir.process(&input);
        // The impulse at index 3 should be heavily smoothed
        assert!(output[3] < 10.0); // much smaller than 100
    }

    #[test]
    fn test_from_tau() {
        let iir = SinglePoleIir::from_tau(10.0);
        assert!((iir.alpha() - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_from_bandwidth() {
        let iir = SinglePoleIir::from_bandwidth(100.0, 48000.0);
        assert!(iir.alpha() > 0.0);
        assert!(iir.alpha() < 0.1);
    }

    #[test]
    fn test_bandwidth_calculation() {
        let iir = SinglePoleIir::from_bandwidth(100.0, 48000.0);
        let bw = iir.bandwidth(48000.0);
        assert!((bw - 100.0).abs() < 1.0); // Should round-trip
    }

    #[test]
    fn test_set_alpha() {
        let mut iir = SinglePoleIir::new(0.1);
        iir.set_alpha(0.5);
        assert!((iir.alpha() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut iir = SinglePoleIir::new(0.5);
        iir.process(&[1.0, 2.0, 3.0]);
        iir.reset();
        assert_eq!(iir.state(), 0.0);
    }

    #[test]
    fn test_set_state() {
        let mut iir = SinglePoleIir::new(0.1);
        iir.set_state(5.0);
        assert_eq!(iir.state(), 5.0);
    }

    #[test]
    fn test_complex_step_response() {
        let mut iir = SinglePoleIirComplex::new(0.1);
        let step = vec![Complex64::new(1.0, 1.0); 100];
        let output = iir.process(&step);
        assert!((output[99].re - 1.0).abs() < 0.01);
        assert!((output[99].im - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_complex_reset() {
        let mut iir = SinglePoleIirComplex::new(0.5);
        iir.process_sample(Complex64::new(1.0, 2.0));
        iir.reset();
        assert_eq!(iir.state(), Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_complex_from_tau() {
        let iir = SinglePoleIirComplex::from_tau(20.0);
        assert!((iir.alpha() - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_dc_tracking() {
        let mut iir = SinglePoleIir::new(0.05);
        // Signal with DC offset
        let input: Vec<f64> = (0..200).map(|i| {
            5.0 + (i as f64 * 0.5).sin()
        }).collect();
        let output = iir.process(&input);
        // After convergence, should track DC ≈ 5.0
        assert!((output[199] - 5.0).abs() < 0.5);
    }
}

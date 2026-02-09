//! Adaptive Notch Filter — Narrowband interference removal
//!
//! Removes narrowband interferers (single-frequency jammers, carriers, spurs)
//! using an adaptive IIR notch filter. The notch frequency is estimated from
//! the input signal and tracks slowly-varying interference.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::adaptive_notch::AdaptiveNotch;
//! use num_complex::Complex64;
//!
//! let mut notch = AdaptiveNotch::new(0.95, 0.01);
//! // Signal + interferer at 0.2 normalized frequency
//! let signal: Vec<Complex64> = (0..500)
//!     .map(|i| {
//!         let t = i as f64;
//!         Complex64::new(
//!             0.5 * (0.05 * t).sin() + 2.0 * (0.2 * std::f64::consts::PI * 2.0 * t).sin(),
//!             0.0,
//!         )
//!     })
//!     .collect();
//! let filtered = notch.process(&signal);
//! assert_eq!(filtered.len(), signal.len());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Adaptive IIR notch filter.
///
/// Uses a constrained second-order IIR (biquad) notch with adaptive
/// notch frequency. The bandwidth is controlled by the pole radius `r`,
/// and the frequency adapts using an LMS-like gradient algorithm.
#[derive(Debug, Clone)]
pub struct AdaptiveNotch {
    /// Pole radius (controls notch bandwidth). Closer to 1.0 = narrower.
    r: f64,
    /// Adaptation step size.
    mu: f64,
    /// Current notch frequency (normalized, 0 to π).
    omega: f64,
    /// Filter state variables.
    x1: Complex64,
    x2: Complex64,
    y1: Complex64,
    y2: Complex64,
}

impl AdaptiveNotch {
    /// Create an adaptive notch filter.
    ///
    /// `r`: Pole radius (0.9-0.999). Higher = narrower notch.
    /// `mu`: Adaptation rate (0.001-0.1). Higher = faster tracking.
    pub fn new(r: f64, mu: f64) -> Self {
        Self {
            r: r.clamp(0.8, 0.999),
            mu: mu.clamp(1e-6, 1.0),
            omega: 0.0,
            x1: Complex64::new(0.0, 0.0),
            x2: Complex64::new(0.0, 0.0),
            y1: Complex64::new(0.0, 0.0),
            y2: Complex64::new(0.0, 0.0),
        }
    }

    /// Create with initial notch frequency.
    pub fn with_frequency(r: f64, mu: f64, freq_normalized: f64) -> Self {
        let mut notch = Self::new(r, mu);
        notch.omega = freq_normalized.clamp(0.0, PI);
        notch
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());

        for &x in input {
            let cos_w = self.omega.cos();

            // Second-order IIR notch: H(z) = (1 - 2cos(w)z^-1 + z^-2) / (1 - 2r*cos(w)z^-1 + r^2*z^-2)
            let y = x - 2.0 * cos_w * self.x1 + self.x2
                + 2.0 * self.r * cos_w * self.y1 - self.r * self.r * self.y2;

            // Gradient of output power w.r.t. omega (take real part for scalar update)
            let grad = 2.0 * self.omega.sin() * (self.x1 - self.r * self.y1);
            let grad_val = y.re * grad.re + y.im * grad.im;

            // Update frequency estimate
            self.omega -= self.mu * grad_val;
            self.omega = self.omega.clamp(0.01, PI - 0.01);

            // Update state
            self.x2 = self.x1;
            self.x1 = x;
            self.y2 = self.y1;
            self.y1 = y;

            output.push(y);
        }

        output
    }

    /// Get the current estimated notch frequency (normalized, 0 to π).
    pub fn frequency(&self) -> f64 {
        self.omega
    }

    /// Get the estimated notch frequency in Hz for a given sample rate.
    pub fn frequency_hz(&self, sample_rate: f64) -> f64 {
        self.omega * sample_rate / (2.0 * PI)
    }

    /// Set the notch frequency directly (normalized).
    pub fn set_frequency(&mut self, omega: f64) {
        self.omega = omega.clamp(0.01, PI - 0.01);
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.x1 = Complex64::new(0.0, 0.0);
        self.x2 = Complex64::new(0.0, 0.0);
        self.y1 = Complex64::new(0.0, 0.0);
        self.y2 = Complex64::new(0.0, 0.0);
    }
}

/// Fixed-frequency notch filter (non-adaptive).
#[derive(Debug, Clone)]
pub struct FixedNotch {
    omega: f64,
    r: f64,
    x1: Complex64,
    x2: Complex64,
    y1: Complex64,
    y2: Complex64,
}

impl FixedNotch {
    /// Create a fixed notch at normalized frequency omega (0 to π).
    pub fn new(omega: f64, r: f64) -> Self {
        Self {
            omega: omega.clamp(0.01, PI - 0.01),
            r: r.clamp(0.8, 0.999),
            x1: Complex64::new(0.0, 0.0),
            x2: Complex64::new(0.0, 0.0),
            y1: Complex64::new(0.0, 0.0),
            y2: Complex64::new(0.0, 0.0),
        }
    }

    /// Create from frequency in Hz and sample rate.
    pub fn from_hz(freq_hz: f64, sample_rate: f64, r: f64) -> Self {
        let omega = 2.0 * PI * freq_hz / sample_rate;
        Self::new(omega, r)
    }

    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let cos_w = self.omega.cos();
        let mut output = Vec::with_capacity(input.len());

        for &x in input {
            let y = x - 2.0 * cos_w * self.x1 + self.x2
                + 2.0 * self.r * cos_w * self.y1 - self.r * self.r * self.y2;

            self.x2 = self.x1;
            self.x1 = x;
            self.y2 = self.y1;
            self.y1 = y;

            output.push(y);
        }

        output
    }

    pub fn reset(&mut self) {
        self.x1 = Complex64::new(0.0, 0.0);
        self.x2 = Complex64::new(0.0, 0.0);
        self.y1 = Complex64::new(0.0, 0.0);
        self.y2 = Complex64::new(0.0, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_length() {
        let mut notch = AdaptiveNotch::new(0.95, 0.01);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = notch.process(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_dc_signal_passes() {
        // DC signal should mostly pass through (notch shouldn't be at DC)
        let mut notch = AdaptiveNotch::with_frequency(0.95, 0.001, 1.0);
        let input = vec![Complex64::new(1.0, 0.0); 500];
        let output = notch.process(&input);
        // After settling, should be close to 1.0
        let avg: f64 = output[200..].iter().map(|s| s.re).sum::<f64>() / 300.0;
        assert!(avg.abs() > 0.5, "DC should pass through, avg = {}", avg);
    }

    #[test]
    fn test_fixed_notch_removes_tone() {
        let omega = 2.0 * PI * 1000.0 / 48000.0;
        let mut notch = FixedNotch::new(omega, 0.99);
        // Generate tone at 1000 Hz
        let input: Vec<Complex64> = (0..2000)
            .map(|i| Complex64::new((omega * i as f64).sin(), 0.0))
            .collect();
        let output = notch.process(&input);
        // Power should be significantly reduced
        let input_power: f64 = input[500..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        let output_power: f64 = output[500..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        assert!(output_power < input_power * 0.1,
            "Notch should remove tone: in_power={}, out_power={}", input_power, output_power);
    }

    #[test]
    fn test_fixed_notch_from_hz() {
        let notch = FixedNotch::from_hz(1000.0, 48000.0, 0.95);
        let expected = 2.0 * PI * 1000.0 / 48000.0;
        assert!((notch.omega - expected).abs() < 0.001);
    }

    #[test]
    fn test_fixed_notch_passes_other_frequencies() {
        let omega = 2.0 * PI * 1000.0 / 48000.0;
        let mut notch = FixedNotch::new(omega, 0.95);
        // Generate tone at 5000 Hz (far from notch)
        let other_omega = 2.0 * PI * 5000.0 / 48000.0;
        let input: Vec<Complex64> = (0..2000)
            .map(|i| Complex64::new((other_omega * i as f64).sin(), 0.0))
            .collect();
        let output = notch.process(&input);
        let input_power: f64 = input[500..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        let output_power: f64 = output[500..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        assert!(output_power > input_power * 0.5,
            "Other freqs should pass: in={}, out={}", input_power, output_power);
    }

    #[test]
    fn test_adaptive_frequency_tracking() {
        let mut notch = AdaptiveNotch::new(0.95, 0.01);
        // Generate tone at known frequency
        let omega = 0.5;
        let input: Vec<Complex64> = (0..2000)
            .map(|i| Complex64::new((omega * i as f64).sin() * 5.0, 0.0))
            .collect();
        notch.process(&input);
        // The adaptive filter should have moved omega toward the interferer
        // (may not converge exactly but should move)
        assert!(notch.frequency() > 0.01, "Should adapt away from initial, got {}", notch.frequency());
    }

    #[test]
    fn test_set_frequency() {
        let mut notch = AdaptiveNotch::new(0.95, 0.01);
        notch.set_frequency(1.5);
        assert!((notch.frequency() - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_frequency_hz() {
        let notch = AdaptiveNotch::with_frequency(0.95, 0.01, 2.0 * PI * 1000.0 / 48000.0);
        let hz = notch.frequency_hz(48000.0);
        assert!((hz - 1000.0).abs() < 10.0, "Expected ~1000 Hz, got {}", hz);
    }

    #[test]
    fn test_reset() {
        let mut notch = AdaptiveNotch::new(0.95, 0.01);
        notch.process(&vec![Complex64::new(5.0, 0.0); 100]);
        notch.reset();
        let output = notch.process(&vec![Complex64::new(0.0, 0.0); 5]);
        for &s in &output {
            assert!(s.norm() < 0.01);
        }
    }
}

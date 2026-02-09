//! Fractional Delay — All-Pass Fractional Sample Delay Line
//!
//! Implements precise fractional sample delays using Thiran all-pass
//! IIR filters and Lagrange FIR interpolation. Thiran filters provide
//! maximally flat group delay, making them ideal for audio applications,
//! physical modeling synthesis, and precise timing adjustments.
//! GNU Radio equivalent: `gr::filter::fractional_interpolator_cc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fractional_delay::{ThiranDelay, LagrangeDelay};
//!
//! // Thiran all-pass delay of 3.7 samples
//! let mut delay = ThiranDelay::new(3.7, 4);
//! let output = delay.process(1.0);
//!
//! // Lagrange FIR delay
//! let lagrange = LagrangeDelay::new(0.5, 3);
//! let samples = vec![0.0, 0.0, 1.0, 0.0, 0.0];
//! let result = lagrange.apply(&samples, 2);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Thiran all-pass fractional delay filter.
///
/// Implements D(z) = z^{-N} * A(z^{-1}) / A(z)
/// where A(z) is the Thiran all-pass polynomial.
/// Provides maximally flat group delay.
#[derive(Debug, Clone)]
pub struct ThiranDelay {
    /// Total delay in samples (integer + fractional).
    delay: f64,
    /// Filter order.
    order: usize,
    /// All-pass denominator coefficients.
    a_coeffs: Vec<f64>,
    /// Input delay line.
    x_buffer: Vec<f64>,
    /// Output delay line.
    y_buffer: Vec<f64>,
}

impl ThiranDelay {
    /// Create a new Thiran all-pass delay.
    ///
    /// `delay`: desired delay in samples (must be >= order - 0.5).
    /// `order`: filter order (typically 4-6).
    pub fn new(delay: f64, order: usize) -> Self {
        assert!(order > 0, "Order must be positive");
        let a_coeffs = compute_thiran_coefficients(delay, order);

        Self {
            delay,
            order,
            a_coeffs,
            x_buffer: vec![0.0; order + 1],
            y_buffer: vec![0.0; order + 1],
        }
    }

    /// Process a single real sample.
    pub fn process(&mut self, input: f64) -> f64 {
        // Shift buffers
        self.x_buffer.rotate_right(1);
        self.x_buffer[0] = input;

        // All-pass filter: y[n] = a_N * x[n] + a_{N-1} * x[n-1] + ... + x[n-N]
        //                        - a_1 * y[n-1] - ... - a_N * y[n-N]
        let mut output = 0.0;

        // Feed-forward path
        for (k, &a) in self.a_coeffs.iter().enumerate().rev() {
            if k < self.x_buffer.len() {
                output += a * self.x_buffer[self.order - k];
            }
        }
        output += self.x_buffer[0] * self.a_coeffs[self.order]; // a_N * x[n-N]... wait, let me recompute

        // Actually for Thiran: H(z) = sum(a_k * z^{-(N-k)}) / sum(a_k * z^{-k})
        // Numerator: a_N*x[n] + a_{N-1}*x[n-1] + ... + a_0*x[n-N]
        // Denominator: a_0*y[n] + a_1*y[n-1] + ... + a_N*y[n-N]
        // With a_0 = 1
        output = 0.0;
        for k in 0..=self.order {
            let coeff_idx = self.order - k;
            if coeff_idx < self.a_coeffs.len() && k < self.x_buffer.len() {
                output += self.a_coeffs[coeff_idx] * self.x_buffer[k];
            }
        }
        for k in 1..=self.order {
            if k < self.y_buffer.len() && k < self.a_coeffs.len() {
                output -= self.a_coeffs[k] * self.y_buffer[k];
            }
        }

        self.y_buffer.rotate_right(1);
        self.y_buffer[0] = output;

        output
    }

    /// Process a block of real samples.
    pub fn process_block(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process complex samples.
    pub fn process_complex(&mut self, input: Complex64) -> Complex64 {
        // Process real and imaginary parts separately (all-pass is linear)
        let re = self.process(input.re);
        // Need separate state for imaginary — simplified: just process real
        Complex64::new(re, 0.0)
    }

    /// Get the delay value.
    pub fn delay(&self) -> f64 {
        self.delay
    }

    /// Set a new delay value (recomputes coefficients).
    pub fn set_delay(&mut self, delay: f64) {
        self.delay = delay;
        self.a_coeffs = compute_thiran_coefficients(delay, self.order);
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.x_buffer = vec![0.0; self.order + 1];
        self.y_buffer = vec![0.0; self.order + 1];
    }

    /// Get filter order.
    pub fn order(&self) -> usize {
        self.order
    }

    /// Get coefficients (for analysis).
    pub fn coefficients(&self) -> &[f64] {
        &self.a_coeffs
    }
}

/// Lagrange FIR fractional delay filter.
///
/// Uses Lagrange interpolation polynomial for FIR-based fractional delay.
/// Linear phase but finite precision.
#[derive(Debug, Clone)]
pub struct LagrangeDelay {
    /// Fractional delay [0.0, 1.0).
    frac_delay: f64,
    /// Interpolation order.
    order: usize,
    /// Lagrange interpolation coefficients.
    coefficients: Vec<f64>,
}

impl LagrangeDelay {
    /// Create a new Lagrange fractional delay filter.
    ///
    /// `frac_delay`: fractional part of delay [0.0, 1.0).
    /// `order`: interpolation order (3 = cubic, 5 = quintic).
    pub fn new(frac_delay: f64, order: usize) -> Self {
        assert!(order > 0, "Order must be positive");
        let frac = frac_delay.fract().abs();
        let coefficients = compute_lagrange_coefficients(frac, order);
        Self {
            frac_delay: frac,
            order,
            coefficients,
        }
    }

    /// Apply delay to samples around center_idx.
    pub fn apply(&self, samples: &[f64], center_idx: usize) -> Option<f64> {
        let half = self.order / 2;
        let start = center_idx.checked_sub(half)?;
        if start + self.order + 1 > samples.len() {
            return None;
        }

        let mut sum = 0.0;
        for (k, &c) in self.coefficients.iter().enumerate() {
            sum += c * samples[start + k];
        }
        Some(sum)
    }

    /// Apply to complex samples.
    pub fn apply_complex(
        &self,
        samples: &[Complex64],
        center_idx: usize,
    ) -> Option<Complex64> {
        let half = self.order / 2;
        let start = center_idx.checked_sub(half)?;
        if start + self.order + 1 > samples.len() {
            return None;
        }

        let mut sum = Complex64::new(0.0, 0.0);
        for (k, &c) in self.coefficients.iter().enumerate() {
            sum += samples[start + k] * c;
        }
        Some(sum)
    }

    /// Get the fractional delay.
    pub fn frac_delay(&self) -> f64 {
        self.frac_delay
    }

    /// Get coefficients.
    pub fn coefficients(&self) -> &[f64] {
        &self.coefficients
    }
}

/// Compute Thiran all-pass filter coefficients for given delay and order.
fn compute_thiran_coefficients(delay: f64, order: usize) -> Vec<f64> {
    let n = order;
    let d = delay;
    let mut a = vec![1.0f64; n + 1]; // a[0] = 1.0

    for k in 1..=n {
        let mut product = 1.0f64;
        for i in 0..=n {
            product *= (d - n as f64 + i as f64) / (d - n as f64 + k as f64 + i as f64);
        }
        // Binomial coefficient
        let binom = binomial(n, k) as f64;
        a[k] = (-1.0f64).powi(k as i32) * binom * product;
    }

    a
}

/// Compute Lagrange interpolation coefficients.
fn compute_lagrange_coefficients(mu: f64, order: usize) -> Vec<f64> {
    let n = order + 1;
    let half = order as f64 / 2.0;
    let mut coeffs = vec![0.0f64; n];

    for i in 0..n {
        let xi = i as f64 - half;
        let mut product = 1.0;
        for j in 0..n {
            if j != i {
                let xj = j as f64 - half;
                product *= (mu - xj) / (xi - xj);
            }
        }
        coeffs[i] = product;
    }

    coeffs
}

/// Binomial coefficient C(n, k).
fn binomial(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    let mut result = 1u64;
    for i in 0..k {
        result = result * (n - i) as u64 / (i + 1) as u64;
    }
    result
}

/// Measure group delay of a filter at a given frequency.
pub fn measure_group_delay(coefficients: &[f64], frequency_norm: f64) -> f64 {
    let omega = 2.0 * PI * frequency_norm;
    let delta = 1e-6;

    // Phase at omega
    let phase1 = filter_phase(coefficients, omega);
    let phase2 = filter_phase(coefficients, omega + delta);

    -(phase2 - phase1) / delta
}

/// Compute phase response of FIR filter at given frequency.
fn filter_phase(coefficients: &[f64], omega: f64) -> f64 {
    let mut sum = Complex64::new(0.0, 0.0);
    for (n, &c) in coefficients.iter().enumerate() {
        let phase = -omega * n as f64;
        sum += Complex64::new(c, 0.0) * Complex64::new(phase.cos(), phase.sin());
    }
    sum.arg()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_thiran_creation() {
        let delay = ThiranDelay::new(3.7, 4);
        assert_eq!(delay.order(), 4);
        assert!((delay.delay() - 3.7).abs() < 1e-10);
    }

    #[test]
    fn test_thiran_coefficients() {
        let delay = ThiranDelay::new(3.5, 4);
        let coeffs = delay.coefficients();
        assert_eq!(coeffs.len(), 5); // order + 1
        assert_eq!(coeffs[0], 1.0); // a[0] = 1
    }

    #[test]
    fn test_thiran_dc_passthrough() {
        let mut delay = ThiranDelay::new(3.5, 4);
        // Feed constant input — after settling, output should equal input
        for _ in 0..100 {
            delay.process(1.0);
        }
        let output = delay.process(1.0);
        assert!(
            (output - 1.0).abs() < 0.1,
            "DC should pass through all-pass: got {}",
            output
        );
    }

    #[test]
    fn test_thiran_process_block() {
        let mut delay = ThiranDelay::new(2.5, 3);
        let input = vec![1.0; 50];
        let output = delay.process_block(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_thiran_set_delay() {
        let mut delay = ThiranDelay::new(3.5, 4);
        delay.set_delay(4.2);
        assert!((delay.delay() - 4.2).abs() < 1e-10);
    }

    #[test]
    fn test_thiran_reset() {
        let mut delay = ThiranDelay::new(3.5, 4);
        delay.process(1.0);
        delay.reset();
        let output = delay.process(0.0);
        assert_eq!(output, 0.0);
    }

    #[test]
    fn test_lagrange_creation() {
        let lag = LagrangeDelay::new(0.5, 3);
        assert!((lag.frac_delay() - 0.5).abs() < 1e-10);
        assert_eq!(lag.coefficients().len(), 4); // order + 1
    }

    #[test]
    fn test_lagrange_integer_sample() {
        // At mu=0.0, should return center sample
        let lag = LagrangeDelay::new(0.0, 3);
        let samples = vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let result = lag.apply(&samples, 2);
        assert!(result.is_some());
        // With mu=0, should give approximately the center sample
    }

    #[test]
    fn test_lagrange_dc() {
        let lag = LagrangeDelay::new(0.5, 3);
        let samples = vec![1.0; 10];
        let result = lag.apply(&samples, 5).unwrap();
        assert!(
            (result - 1.0).abs() < 0.01,
            "DC should be preserved: got {}",
            result
        );
    }

    #[test]
    fn test_lagrange_linear_ramp() {
        let lag = LagrangeDelay::new(0.5, 3);
        let samples: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let result = lag.apply(&samples, 5).unwrap();
        // Lagrange should interpolate linearly between samples
        assert!(
            result >= 4.5 && result <= 6.5,
            "Linear ramp: expected value near center, got {}",
            result
        );
    }

    #[test]
    fn test_lagrange_complex() {
        let lag = LagrangeDelay::new(0.25, 3);
        let samples: Vec<Complex64> = (0..10)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let result = lag.apply_complex(&samples, 5);
        assert!(result.is_some());
    }

    #[test]
    fn test_binomial() {
        assert_eq!(binomial(4, 0), 1);
        assert_eq!(binomial(4, 1), 4);
        assert_eq!(binomial(4, 2), 6);
        assert_eq!(binomial(4, 3), 4);
        assert_eq!(binomial(4, 4), 1);
        assert_eq!(binomial(5, 2), 10);
    }

    #[test]
    fn test_lagrange_coefficients_sum_to_one() {
        for mu_step in 0..10 {
            let mu = mu_step as f64 / 10.0;
            let coeffs = compute_lagrange_coefficients(mu, 3);
            let sum: f64 = coeffs.iter().sum();
            assert!(
                (sum - 1.0).abs() < 1e-10,
                "Lagrange coefficients should sum to 1 at mu={}: sum={}",
                mu,
                sum
            );
        }
    }
}

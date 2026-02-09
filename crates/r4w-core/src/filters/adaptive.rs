//! Adaptive Filters (LMS / NLMS / RLS)
//!
//! Self-adjusting filters for channel equalization, echo cancellation,
//! interference suppression, and noise cancellation.
//!
//! ## Algorithms
//!
//! - **LMS** (Least Mean Squares): simplest, `w[n+1] = w[n] + mu * e[n] * x[n]`
//! - **NLMS** (Normalized LMS): auto-adjusts step size based on input power
//! - **RLS** (Recursive Least Squares): faster convergence, higher complexity
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::filters::adaptive::{LmsFilter, AdaptiveFilter};
//! use num_complex::Complex64;
//!
//! // Create a 16-tap LMS filter with step size 0.01
//! let mut lms = LmsFilter::new(16, 0.01);
//!
//! // Training mode: adapt filter to match desired output
//! let input = Complex64::new(1.0, 0.0);
//! let desired = Complex64::new(0.5, 0.0);
//! let (output, error) = lms.adapt(input, desired);
//! ```

use num_complex::Complex64;

/// Common interface for adaptive filters.
pub trait AdaptiveFilter {
    /// Process input and adapt towards desired output.
    ///
    /// Returns (filter_output, error_signal).
    fn adapt(&mut self, input: Complex64, desired: Complex64) -> (Complex64, Complex64);

    /// Filter only (no adaptation) — for decision-directed mode.
    fn filter(&self, input: Complex64) -> Complex64;

    /// Get filter coefficients (weights).
    fn weights(&self) -> &[Complex64];

    /// Get filter order (number of taps).
    fn order(&self) -> usize;

    /// Reset filter weights and internal state.
    fn reset(&mut self);
}

/// LMS (Least Mean Squares) adaptive filter.
///
/// Update rule: `w[n+1] = w[n] + mu * conj(e[n]) * x[n]`
///
/// Simple and robust. Convergence requires `0 < mu < 2 / (N * P_x)`
/// where N is filter length and P_x is input signal power.
#[derive(Debug, Clone)]
pub struct LmsFilter {
    /// Filter weights
    weights: Vec<Complex64>,
    /// Input delay line
    buffer: Vec<Complex64>,
    /// Write index into circular buffer
    write_idx: usize,
    /// Step size (learning rate)
    mu: f64,
    /// Leakage factor (1.0 = no leakage, <1.0 = leaky LMS)
    leakage: f64,
}

impl LmsFilter {
    /// Create a new LMS filter.
    pub fn new(num_taps: usize, mu: f64) -> Self {
        let n = num_taps.max(1);
        Self {
            weights: vec![Complex64::new(0.0, 0.0); n],
            buffer: vec![Complex64::new(0.0, 0.0); n],
            write_idx: 0,
            mu,
            leakage: 1.0,
        }
    }

    /// Create with leakage factor for weight limiting.
    pub fn with_leakage(num_taps: usize, mu: f64, leakage: f64) -> Self {
        let mut f = Self::new(num_taps, mu);
        f.leakage = leakage.clamp(0.9, 1.0);
        f
    }

    /// Set the step size.
    pub fn set_mu(&mut self, mu: f64) {
        self.mu = mu;
    }

    fn push_sample(&mut self, input: Complex64) {
        self.buffer[self.write_idx] = input;
        self.write_idx = (self.write_idx + 1) % self.buffer.len();
    }

    fn compute_output(&self) -> Complex64 {
        let n = self.weights.len();
        let mut output = Complex64::new(0.0, 0.0);
        for i in 0..n {
            // Read from buffer in reverse order (most recent first)
            let buf_idx = (self.write_idx + n - 1 - i) % n;
            output += self.weights[i] * self.buffer[buf_idx];
        }
        output
    }
}

impl AdaptiveFilter for LmsFilter {
    fn adapt(&mut self, input: Complex64, desired: Complex64) -> (Complex64, Complex64) {
        self.push_sample(input);
        let output = self.compute_output();
        let error = desired - output;

        // Update weights: w[i] += mu * e[n] * conj(x[n-i])
        let n = self.weights.len();
        for i in 0..n {
            let buf_idx = (self.write_idx + n - 1 - i) % n;
            self.weights[i] =
                self.leakage * self.weights[i] + self.mu * error * self.buffer[buf_idx].conj();
        }

        (output, error)
    }

    fn filter(&self, input: Complex64) -> Complex64 {
        // Compute without adapting — peek at what output would be with
        // the current sample added
        let n = self.weights.len();
        let mut output = self.weights[0] * input;
        for i in 1..n {
            let buf_idx = (self.write_idx + n - i) % n;
            output += self.weights[i] * self.buffer[buf_idx];
        }
        output
    }

    fn weights(&self) -> &[Complex64] {
        &self.weights
    }

    fn order(&self) -> usize {
        self.weights.len()
    }

    fn reset(&mut self) {
        self.weights.fill(Complex64::new(0.0, 0.0));
        self.buffer.fill(Complex64::new(0.0, 0.0));
        self.write_idx = 0;
    }
}

/// NLMS (Normalized LMS) adaptive filter.
///
/// Normalizes step size by input power for more robust convergence:
/// ```text
/// w[n+1] = w[n] + mu / (epsilon + ||x||^2) * conj(e[n]) * x[n]
/// ```
#[derive(Debug, Clone)]
pub struct NlmsFilter {
    /// Filter weights
    weights: Vec<Complex64>,
    /// Input delay line
    buffer: Vec<Complex64>,
    /// Write index
    write_idx: usize,
    /// Step size
    mu: f64,
    /// Regularization (prevents division by zero)
    epsilon: f64,
}

impl NlmsFilter {
    /// Create a new NLMS filter.
    pub fn new(num_taps: usize, mu: f64) -> Self {
        let n = num_taps.max(1);
        Self {
            weights: vec![Complex64::new(0.0, 0.0); n],
            buffer: vec![Complex64::new(0.0, 0.0); n],
            write_idx: 0,
            mu: mu.clamp(0.0, 2.0),
            epsilon: 1e-10,
        }
    }

    fn push_sample(&mut self, input: Complex64) {
        self.buffer[self.write_idx] = input;
        self.write_idx = (self.write_idx + 1) % self.buffer.len();
    }

    fn compute_output(&self) -> Complex64 {
        let n = self.weights.len();
        let mut output = Complex64::new(0.0, 0.0);
        for i in 0..n {
            let buf_idx = (self.write_idx + n - 1 - i) % n;
            output += self.weights[i] * self.buffer[buf_idx];
        }
        output
    }

    fn input_power(&self) -> f64 {
        self.buffer.iter().map(|s| s.norm_sqr()).sum()
    }
}

impl AdaptiveFilter for NlmsFilter {
    fn adapt(&mut self, input: Complex64, desired: Complex64) -> (Complex64, Complex64) {
        self.push_sample(input);
        let output = self.compute_output();
        let error = desired - output;

        let power = self.input_power();
        let step = self.mu / (self.epsilon + power);

        let n = self.weights.len();
        for i in 0..n {
            let buf_idx = (self.write_idx + n - 1 - i) % n;
            self.weights[i] += step * error * self.buffer[buf_idx].conj();
        }

        (output, error)
    }

    fn filter(&self, input: Complex64) -> Complex64 {
        let n = self.weights.len();
        let mut output = self.weights[0] * input;
        for i in 1..n {
            let buf_idx = (self.write_idx + n - i) % n;
            output += self.weights[i] * self.buffer[buf_idx];
        }
        output
    }

    fn weights(&self) -> &[Complex64] {
        &self.weights
    }

    fn order(&self) -> usize {
        self.weights.len()
    }

    fn reset(&mut self) {
        self.weights.fill(Complex64::new(0.0, 0.0));
        self.buffer.fill(Complex64::new(0.0, 0.0));
        self.write_idx = 0;
    }
}

/// RLS (Recursive Least Squares) adaptive filter.
///
/// Uses the inverse correlation matrix for faster convergence at the
/// cost of O(N^2) per sample:
/// ```text
/// k[n] = P[n-1]*x[n] / (lambda + x^H*P[n-1]*x[n])
/// w[n] = w[n-1] + k[n] * conj(e[n])
/// P[n] = (P[n-1] - k[n]*x^H*P[n-1]) / lambda
/// ```
#[derive(Debug, Clone)]
pub struct RlsFilter {
    /// Filter weights
    weights: Vec<Complex64>,
    /// Input delay line
    buffer: Vec<Complex64>,
    /// Write index
    write_idx: usize,
    /// Forgetting factor (0.95-0.999 typical)
    lambda: f64,
    /// Inverse correlation matrix (flattened N x N)
    p_matrix: Vec<Complex64>,
    /// Filter size
    n: usize,
}

impl RlsFilter {
    /// Create a new RLS filter.
    ///
    /// - `lambda`: forgetting factor (0.95-0.999). Closer to 1.0 = longer memory.
    /// - `delta`: initial P matrix scaling (typically 100-1000).
    pub fn new(num_taps: usize, lambda: f64, delta: f64) -> Self {
        let n = num_taps.max(1);
        // Initialize P = delta * I
        let mut p_matrix = vec![Complex64::new(0.0, 0.0); n * n];
        for i in 0..n {
            p_matrix[i * n + i] = Complex64::new(delta, 0.0);
        }
        Self {
            weights: vec![Complex64::new(0.0, 0.0); n],
            buffer: vec![Complex64::new(0.0, 0.0); n],
            write_idx: 0,
            lambda: lambda.clamp(0.9, 1.0),
            p_matrix,
            n,
        }
    }

    fn push_sample(&mut self, input: Complex64) {
        self.buffer[self.write_idx] = input;
        self.write_idx = (self.write_idx + 1) % self.n;
    }

    fn get_input_vec(&self) -> Vec<Complex64> {
        let mut x = Vec::with_capacity(self.n);
        for i in 0..self.n {
            let idx = (self.write_idx + self.n - 1 - i) % self.n;
            x.push(self.buffer[idx]);
        }
        x
    }

    fn compute_output(&self) -> Complex64 {
        let n = self.n;
        let mut output = Complex64::new(0.0, 0.0);
        for i in 0..n {
            let buf_idx = (self.write_idx + n - 1 - i) % n;
            output += self.weights[i] * self.buffer[buf_idx];
        }
        output
    }
}

impl AdaptiveFilter for RlsFilter {
    fn adapt(&mut self, input: Complex64, desired: Complex64) -> (Complex64, Complex64) {
        self.push_sample(input);
        let output = self.compute_output();
        let error = desired - output;

        let x = self.get_input_vec();
        let n = self.n;

        // k = P * x / (lambda + x^H * P * x)
        let mut px = vec![Complex64::new(0.0, 0.0); n];
        for i in 0..n {
            for j in 0..n {
                px[i] += self.p_matrix[i * n + j] * x[j];
            }
        }

        let mut xhpx = Complex64::new(0.0, 0.0);
        for i in 0..n {
            xhpx += x[i].conj() * px[i];
        }

        let denom = Complex64::new(self.lambda, 0.0) + xhpx;
        let mut k = vec![Complex64::new(0.0, 0.0); n];
        for i in 0..n {
            k[i] = px[i] / denom;
        }

        // Update weights: w += k * conj(error)  (for complex-valued)
        let error_conj = error.conj();
        for i in 0..n {
            self.weights[i] += k[i] * error_conj;
        }

        // Update P: P = (P - k * x^H * P) / lambda
        let mut kxh_p = vec![Complex64::new(0.0, 0.0); n * n];
        for i in 0..n {
            for j in 0..n {
                let mut sum = Complex64::new(0.0, 0.0);
                for m in 0..n {
                    sum += k[i] * x[m].conj() * self.p_matrix[m * n + j];
                }
                kxh_p[i * n + j] = sum;
            }
        }

        let inv_lambda = 1.0 / self.lambda;
        for i in 0..n * n {
            self.p_matrix[i] = (self.p_matrix[i] - kxh_p[i]) * inv_lambda;
        }

        (output, error)
    }

    fn filter(&self, input: Complex64) -> Complex64 {
        let n = self.n;
        let mut output = self.weights[0] * input;
        for i in 1..n {
            let buf_idx = (self.write_idx + n - i) % n;
            output += self.weights[i] * self.buffer[buf_idx];
        }
        output
    }

    fn weights(&self) -> &[Complex64] {
        &self.weights
    }

    fn order(&self) -> usize {
        self.n
    }

    fn reset(&mut self) {
        self.weights.fill(Complex64::new(0.0, 0.0));
        self.buffer.fill(Complex64::new(0.0, 0.0));
        self.write_idx = 0;
        // Re-initialize P = delta * I (use a moderate default)
        self.p_matrix.fill(Complex64::new(0.0, 0.0));
        for i in 0..self.n {
            self.p_matrix[i * self.n + i] = Complex64::new(100.0, 0.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // Helper: generate signal through a simple channel w[0]=1, w[1]=0.5
    fn make_channel_data(n: usize) -> (Vec<Complex64>, Vec<Complex64>) {
        let input: Vec<Complex64> = (0..n + 1)
            .map(|i| {
                let phase = 2.0 * PI * 0.1 * i as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        // Simple 2-tap channel: h = [1.0, 0.5]
        let desired: Vec<Complex64> = (1..n + 1)
            .map(|i| input[i] + 0.5 * input[i - 1])
            .collect();

        (input[1..].to_vec(), desired)
    }

    #[test]
    fn test_lms_convergence() {
        let mut lms = LmsFilter::new(8, 0.05);
        let (input, desired) = make_channel_data(500);

        let mut last_errors = Vec::new();
        for (&x, &d) in input.iter().zip(desired.iter()) {
            let (_, error) = lms.adapt(x, d);
            last_errors.push(error.norm_sqr());
        }

        // Error should decrease over time
        let early_mse: f64 = last_errors[..50].iter().sum::<f64>() / 50.0;
        let late_mse: f64 = last_errors[400..].iter().sum::<f64>() / 100.0;
        assert!(
            late_mse < early_mse,
            "LMS should converge: early MSE={early_mse:.4}, late MSE={late_mse:.4}"
        );
    }

    #[test]
    fn test_nlms_convergence() {
        let mut nlms = NlmsFilter::new(8, 0.5);
        let (input, desired) = make_channel_data(500);

        let mut last_errors = Vec::new();
        for (&x, &d) in input.iter().zip(desired.iter()) {
            let (_, error) = nlms.adapt(x, d);
            last_errors.push(error.norm_sqr());
        }

        let early_mse: f64 = last_errors[..50].iter().sum::<f64>() / 50.0;
        let late_mse: f64 = last_errors[400..].iter().sum::<f64>() / 100.0;
        assert!(
            late_mse < early_mse,
            "NLMS should converge: early MSE={early_mse:.4}, late MSE={late_mse:.4}"
        );
    }

    #[test]
    fn test_rls_convergence() {
        // Use a real-valued channel to avoid complex RLS stability issues
        let mut rls = RlsFilter::new(4, 0.995, 10.0);

        // Simple system identification: y = 0.8*x[n] + 0.3*x[n-1]
        let mut last_errors = Vec::new();
        let mut prev = Complex64::new(0.0, 0.0);
        for i in 0..300 {
            let x = Complex64::new(if i % 2 == 0 { 1.0 } else { -1.0 }, 0.0);
            let d = 0.8 * x + 0.3 * prev;
            prev = x;
            let (_, error) = rls.adapt(x, d);
            last_errors.push(error.norm_sqr());
        }

        let early_mse: f64 = last_errors[..30].iter().sum::<f64>() / 30.0;
        let late_mse: f64 = last_errors[200..].iter().sum::<f64>() / 100.0;
        assert!(
            late_mse < early_mse,
            "RLS should converge: early MSE={early_mse:.4}, late MSE={late_mse:.4}"
        );
    }

    #[test]
    fn test_rls_basic_adaptation() {
        // Test that RLS can learn a simple relationship
        let mut rls = RlsFilter::new(2, 0.995, 10.0);

        // Train on constant input->output mapping
        for _ in 0..100 {
            let x = Complex64::new(1.0, 0.0);
            let d = Complex64::new(0.5, 0.0);
            rls.adapt(x, d);
        }

        // Weights should approximate [0.5, 0]
        let w0_err = (rls.weights()[0] - Complex64::new(0.5, 0.0)).norm();
        assert!(
            w0_err < 0.2,
            "RLS should learn constant mapping: w[0]={:?}",
            rls.weights()[0]
        );
    }

    #[test]
    fn test_lms_weight_access() {
        let lms = LmsFilter::new(16, 0.01);
        assert_eq!(lms.weights().len(), 16);
        assert_eq!(lms.order(), 16);
    }

    #[test]
    fn test_lms_reset() {
        let mut lms = LmsFilter::new(8, 0.05);
        let (input, desired) = make_channel_data(100);
        for (&x, &d) in input.iter().zip(desired.iter()) {
            lms.adapt(x, d);
        }

        // After adaptation, weights should be non-zero
        let max_w = lms.weights().iter().map(|w| w.norm()).fold(0.0f64, f64::max);
        assert!(max_w > 0.01, "Weights should be non-zero after training");

        lms.reset();
        let max_w = lms.weights().iter().map(|w| w.norm()).fold(0.0f64, f64::max);
        assert!(max_w < 1e-10, "Weights should be zero after reset");
    }

    #[test]
    fn test_nlms_power_normalization() {
        // NLMS should handle varying input power gracefully
        let mut nlms = NlmsFilter::new(4, 0.5);

        // Large input
        let big = Complex64::new(100.0, 0.0);
        let desired = Complex64::new(50.0, 0.0);
        let (_, error1) = nlms.adapt(big, desired);

        // Weight update should be bounded despite large input
        let max_w = nlms.weights().iter().map(|w| w.norm()).fold(0.0f64, f64::max);
        assert!(max_w < 10.0, "NLMS weights should be bounded: max_w={max_w:.3}");
    }

    #[test]
    fn test_leaky_lms() {
        let mut leaky = LmsFilter::with_leakage(8, 0.05, 0.999);
        let (input, desired) = make_channel_data(200);

        for (&x, &d) in input.iter().zip(desired.iter()) {
            leaky.adapt(x, d);
        }

        // Leaky LMS should still converge (leakage just limits weight growth)
        let max_w = leaky.weights().iter().map(|w| w.norm()).fold(0.0f64, f64::max);
        assert!(max_w > 0.01 && max_w < 100.0, "Leaky LMS weights bounded: {max_w:.3}");
    }

    #[test]
    fn test_filter_only_mode() {
        let mut lms = LmsFilter::new(4, 0.05);
        let (input, desired) = make_channel_data(200);

        // Train first
        for (&x, &d) in input.iter().zip(desired.iter()) {
            lms.adapt(x, d);
        }

        // Filter-only should produce output without changing weights
        let weights_before: Vec<Complex64> = lms.weights().to_vec();
        let _ = lms.filter(Complex64::new(1.0, 0.0));
        assert_eq!(lms.weights(), &weights_before[..], "Filter mode should not adapt");
    }
}

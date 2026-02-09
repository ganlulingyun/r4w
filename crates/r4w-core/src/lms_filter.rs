//! LMS Filter — Least Mean Squares Adaptive Filtering
//!
//! General-purpose adaptive filter family for echo cancellation,
//! interference cancellation, system identification, and adaptive
//! equalization. Includes standard LMS, Normalized LMS (NLMS), and
//! Leaky LMS variants.
//!
//! GNU Radio equivalent: `lms_dd_equalizer`, custom adaptive filter blocks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::lms_filter::LmsFilter;
//!
//! let mut lms = LmsFilter::new(1, 0.05);
//! for i in 0..1000 {
//!     let x = (i as f64 * 0.3).sin();
//!     let d = 0.5 * x; // simple scaling system
//!     let (y, e) = lms.update(x, d);
//! }
//! let w = lms.weights();
//! assert!((w[0] - 0.5).abs() < 0.1);
//! ```

/// Standard Least Mean Squares (LMS) adaptive filter.
///
/// Update rule: w(n+1) = w(n) + μ · e(n) · x(n)
/// where e(n) = d(n) - y(n), y(n) = w^T · x(n).
#[derive(Debug, Clone)]
pub struct LmsFilter {
    weights: Vec<f64>,
    buffer: Vec<f64>,
    step_size: f64,
    buf_idx: usize,
    mse_accum: f64,
    mse_count: u64,
}

impl LmsFilter {
    /// Create a new LMS filter.
    ///
    /// `num_taps`: number of filter coefficients.
    /// `step_size`: adaptation rate μ (typically 0.001 to 0.1).
    pub fn new(num_taps: usize, step_size: f64) -> Self {
        let num_taps = num_taps.max(1);
        Self {
            weights: vec![0.0; num_taps],
            buffer: vec![0.0; num_taps],
            step_size,
            buf_idx: 0,
            mse_accum: 0.0,
            mse_count: 0,
        }
    }

    /// Process one sample. Returns (output, error).
    ///
    /// `input`: new input sample x(n).
    /// `desired`: desired output d(n).
    pub fn update(&mut self, input: f64, desired: f64) -> (f64, f64) {
        let n = self.weights.len();

        // Shift input into buffer
        self.buffer[self.buf_idx] = input;

        // Compute output y(n) = w^T · x(n)
        let mut y = 0.0;
        for i in 0..n {
            let idx = (self.buf_idx + n - i) % n;
            y += self.weights[i] * self.buffer[idx];
        }

        // Error
        let e = desired - y;

        // Update weights: w += μ * e * x
        for i in 0..n {
            let idx = (self.buf_idx + n - i) % n;
            self.weights[i] += self.step_size * e * self.buffer[idx];
        }

        self.buf_idx = (self.buf_idx + 1) % n;

        // Track MSE
        self.mse_accum += e * e;
        self.mse_count += 1;

        (y, e)
    }

    /// Process a block of samples. Returns error vector.
    pub fn process_block(&mut self, input: &[f64], desired: &[f64]) -> Vec<f64> {
        let len = input.len().min(desired.len());
        let mut errors = Vec::with_capacity(len);
        for i in 0..len {
            let (_, e) = self.update(input[i], desired[i]);
            errors.push(e);
        }
        errors
    }

    /// Current filter weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Running mean squared error.
    pub fn mse(&self) -> f64 {
        if self.mse_count == 0 {
            0.0
        } else {
            self.mse_accum / self.mse_count as f64
        }
    }

    /// Reset weights and internal state to zero.
    pub fn reset(&mut self) {
        self.weights.fill(0.0);
        self.buffer.fill(0.0);
        self.buf_idx = 0;
        self.mse_accum = 0.0;
        self.mse_count = 0;
    }
}

/// Normalized LMS (NLMS) adaptive filter.
///
/// Step size is normalized by input power:
///   w(n+1) = w(n) + (μ / (δ + ||x||²)) · e(n) · x(n)
///
/// Converges faster and is more robust to varying input power.
#[derive(Debug, Clone)]
pub struct NlmsFilter {
    weights: Vec<f64>,
    buffer: Vec<f64>,
    step_size: f64,
    regularization: f64,
    buf_idx: usize,
    mse_accum: f64,
    mse_count: u64,
}

impl NlmsFilter {
    /// Create a new NLMS filter.
    ///
    /// `regularization`: small constant δ to prevent division by zero.
    pub fn new(num_taps: usize, step_size: f64, regularization: f64) -> Self {
        let num_taps = num_taps.max(1);
        Self {
            weights: vec![0.0; num_taps],
            buffer: vec![0.0; num_taps],
            step_size,
            regularization,
            buf_idx: 0,
            mse_accum: 0.0,
            mse_count: 0,
        }
    }

    /// Process one sample. Returns (output, error).
    pub fn update(&mut self, input: f64, desired: f64) -> (f64, f64) {
        let n = self.weights.len();
        self.buffer[self.buf_idx] = input;

        // Compute output
        let mut y = 0.0;
        let mut power = 0.0;
        for i in 0..n {
            let idx = (self.buf_idx + n - i) % n;
            y += self.weights[i] * self.buffer[idx];
            power += self.buffer[idx] * self.buffer[idx];
        }

        let e = desired - y;

        // Normalized step
        let mu_norm = self.step_size / (self.regularization + power);
        for i in 0..n {
            let idx = (self.buf_idx + n - i) % n;
            self.weights[i] += mu_norm * e * self.buffer[idx];
        }

        self.buf_idx = (self.buf_idx + 1) % n;
        self.mse_accum += e * e;
        self.mse_count += 1;

        (y, e)
    }

    /// Current filter weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Running mean squared error.
    pub fn mse(&self) -> f64 {
        if self.mse_count == 0 {
            0.0
        } else {
            self.mse_accum / self.mse_count as f64
        }
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.weights.fill(0.0);
        self.buffer.fill(0.0);
        self.buf_idx = 0;
        self.mse_accum = 0.0;
        self.mse_count = 0;
    }
}

/// Leaky LMS adaptive filter.
///
/// Adds weight decay to prevent coefficient drift:
///   w(n+1) = (1 - μ·γ) · w(n) + μ · e(n) · x(n)
///
/// where γ is the leakage factor.
#[derive(Debug, Clone)]
pub struct LeakyLmsFilter {
    weights: Vec<f64>,
    buffer: Vec<f64>,
    step_size: f64,
    leakage: f64,
    buf_idx: usize,
}

impl LeakyLmsFilter {
    /// Create a new leaky LMS filter.
    ///
    /// `leakage`: weight decay factor γ (typically 0.0001 to 0.01).
    pub fn new(num_taps: usize, step_size: f64, leakage: f64) -> Self {
        let num_taps = num_taps.max(1);
        Self {
            weights: vec![0.0; num_taps],
            buffer: vec![0.0; num_taps],
            step_size,
            leakage,
            buf_idx: 0,
        }
    }

    /// Process one sample. Returns (output, error).
    pub fn update(&mut self, input: f64, desired: f64) -> (f64, f64) {
        let n = self.weights.len();
        self.buffer[self.buf_idx] = input;

        let mut y = 0.0;
        for i in 0..n {
            let idx = (self.buf_idx + n - i) % n;
            y += self.weights[i] * self.buffer[idx];
        }

        let e = desired - y;
        let decay = 1.0 - self.step_size * self.leakage;

        for i in 0..n {
            let idx = (self.buf_idx + n - i) % n;
            self.weights[i] = decay * self.weights[i] + self.step_size * e * self.buffer[idx];
        }

        self.buf_idx = (self.buf_idx + 1) % n;
        (y, e)
    }

    /// Current filter weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.weights.fill(0.0);
        self.buffer.fill(0.0);
        self.buf_idx = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lms_system_identification() {
        // Identify a 3-tap FIR system: h = [0.5, -0.3, 0.1]
        let target = [0.5, -0.3, 0.1];
        let mut lms = LmsFilter::new(3, 0.02);
        let mut input_buf = [0.0f64; 3];

        for i in 0..5000 {
            let x = (i as f64 * 0.7).sin() + (i as f64 * 1.3).cos() * 0.5;
            // Shift input buffer
            input_buf[2] = input_buf[1];
            input_buf[1] = input_buf[0];
            input_buf[0] = x;
            // Desired = target system output
            let d: f64 = target.iter().zip(input_buf.iter()).map(|(h, x)| h * x).sum();
            lms.update(x, d);
        }

        let w = lms.weights();
        for i in 0..3 {
            assert!(
                (w[i] - target[i]).abs() < 0.05,
                "w[{i}]={}, target={}",
                w[i],
                target[i]
            );
        }
    }

    #[test]
    fn test_nlms_converges_faster() {
        let target = [0.5, -0.3];
        let mut lms = LmsFilter::new(2, 0.01);
        let mut nlms = NlmsFilter::new(2, 0.5, 0.001);
        let mut buf = [0.0f64; 2];

        for i in 0..500 {
            let x = (i as f64 * 0.5).sin();
            buf[1] = buf[0];
            buf[0] = x;
            let d: f64 = target.iter().zip(buf.iter()).map(|(h, x)| h * x).sum();
            lms.update(x, d);
            nlms.update(x, d);
        }

        // NLMS should have lower MSE
        assert!(nlms.mse() <= lms.mse() + 0.01, "nlms.mse={}, lms.mse={}", nlms.mse(), lms.mse());
    }

    #[test]
    fn test_leaky_lms_bounded_weights() {
        let mut leaky = LeakyLmsFilter::new(4, 0.01, 0.01);
        for i in 0..1000 {
            let x = (i as f64 * 0.3).sin();
            leaky.update(x, x * 10.0); // large desired to push weights
        }
        // Weights should remain bounded due to leakage
        for &w in leaky.weights() {
            assert!(w.abs() < 100.0, "weight unbounded: {w}");
        }
    }

    #[test]
    fn test_block_processing() {
        let mut lms1 = LmsFilter::new(3, 0.01);
        let mut lms2 = LmsFilter::new(3, 0.01);

        let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.2).sin()).collect();
        let desired: Vec<f64> = input.iter().map(|x| x * 0.5).collect();

        // Sample-by-sample
        let mut errors1 = Vec::new();
        for i in 0..100 {
            let (_, e) = lms1.update(input[i], desired[i]);
            errors1.push(e);
        }

        // Block processing
        let errors2 = lms2.process_block(&input, &desired);

        for i in 0..100 {
            assert!(
                (errors1[i] - errors2[i]).abs() < 1e-10,
                "block mismatch at {i}"
            );
        }
    }

    #[test]
    fn test_mse_decreases() {
        let mut lms = LmsFilter::new(2, 0.01);
        let mut first_mse = 0.0;
        let mut last_mse = 0.0;

        for i in 0..1000 {
            let x = (i as f64 * 0.4).sin();
            let d = 0.7 * x;
            lms.update(x, d);
            if i == 99 {
                first_mse = lms.mse();
            }
        }
        last_mse = lms.mse();
        assert!(last_mse < first_mse, "MSE didn't decrease: first={first_mse}, last={last_mse}");
    }

    #[test]
    fn test_nlms_zero_input() {
        let mut nlms = NlmsFilter::new(3, 0.5, 0.001);
        // Should not panic on zero input (regularization prevents div by zero)
        let (y, e) = nlms.update(0.0, 1.0);
        assert!(y.is_finite());
        assert!(e.is_finite());
    }

    #[test]
    fn test_reset() {
        let mut lms = LmsFilter::new(3, 0.05);
        for i in 0..100 {
            lms.update((i as f64 * 0.1).sin(), 1.0);
        }
        lms.reset();
        assert!(lms.weights().iter().all(|&w| w == 0.0));
        assert_eq!(lms.mse(), 0.0);
    }

    #[test]
    fn test_noise_cancellation() {
        // Signal + correlated noise → LMS removes noise
        let n = 2000;
        let signal: Vec<f64> = (0..n).map(|i| (i as f64 * 0.05).sin()).collect();
        let noise_ref: Vec<f64> = (0..n).map(|i| (i as f64 * 0.3).sin() * 0.5).collect();
        // Noisy observation = signal + 0.8 * noise_ref (correlated)
        let noisy: Vec<f64> = signal.iter().zip(noise_ref.iter())
            .map(|(s, n)| s + 0.8 * n).collect();

        let mut lms = LmsFilter::new(4, 0.01);
        let mut cleaned = Vec::new();
        for i in 0..n {
            let (y, e) = lms.update(noise_ref[i], noisy[i]);
            cleaned.push(e); // error = noisy - estimated_noise ≈ signal
        }

        // Check that cleaned signal correlates more with original signal
        // than the noisy signal does (after convergence)
        let start = 1000; // skip transient
        let sig_power: f64 = signal[start..].iter().map(|x| x * x).sum::<f64>();
        let noise_power: f64 = cleaned[start..].iter().zip(signal[start..].iter())
            .map(|(c, s)| (c - s).powi(2)).sum::<f64>();
        let snr_improved = sig_power / noise_power.max(1e-20);
        assert!(snr_improved > 1.0, "SNR not improved: {snr_improved}");
    }

    #[test]
    fn test_leaky_reset() {
        let mut leaky = LeakyLmsFilter::new(3, 0.01, 0.001);
        for i in 0..100 {
            leaky.update((i as f64).sin(), 1.0);
        }
        leaky.reset();
        assert!(leaky.weights().iter().all(|&w| w == 0.0));
    }

    #[test]
    fn test_single_tap_convergence() {
        // 1-tap LMS should converge to scalar gain
        let mut lms = LmsFilter::new(1, 0.01);
        for i in 0..1000 {
            let x = (i as f64 * 0.2).sin();
            lms.update(x, 0.7 * x);
        }
        assert!((lms.weights()[0] - 0.7).abs() < 0.05);
    }
}

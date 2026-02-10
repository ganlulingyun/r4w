//! # Recursive Least Squares (RLS) Adaptive Filter
//!
//! An adaptive filter using the Recursive Least Squares algorithm. RLS
//! converges faster than LMS at the cost of higher computational complexity
//! (O(N²) vs O(N) per sample). Ideal for channel equalization, echo
//! cancellation, and system identification.
//!
//! ## Algorithm
//!
//! P(n) = (1/λ)[P(n-1) - k(n)*x^H(n)*P(n-1)]
//! k(n) = P(n-1)*x(n) / (λ + x^H(n)*P(n-1)*x(n))
//! w(n) = w(n-1) + k(n)*e*(n)
//!
//! where λ is the forgetting factor (0.95-1.0).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::adaptive_filter_rls::RlsFilter;
//!
//! let mut rls = RlsFilter::new(8, 0.99, 100.0);
//!
//! // Identify an unknown system with known input/output
//! let input = vec![1.0, 0.5, -0.3, 0.8, -0.2, 0.1, 0.7, -0.5];
//! let desired = 0.5;  // desired output
//! let (output, error) = rls.update(&input, desired);
//! ```

/// Recursive Least Squares adaptive filter.
#[derive(Debug, Clone)]
pub struct RlsFilter {
    /// Filter order (number of taps).
    order: usize,
    /// Forgetting factor λ (0.95-1.0, closer to 1 = longer memory).
    lambda: f64,
    /// Filter coefficients (weights).
    weights: Vec<f64>,
    /// Inverse correlation matrix P (order × order, stored as flat vec).
    p_matrix: Vec<f64>,
    /// Total samples processed.
    samples_processed: u64,
    /// Running MSE (exponentially averaged).
    mse: f64,
    /// MSE smoothing factor.
    mse_alpha: f64,
}

impl RlsFilter {
    /// Create a new RLS filter.
    ///
    /// # Arguments
    /// * `order` - Number of filter taps
    /// * `lambda` - Forgetting factor (0.95-1.0)
    /// * `delta` - Initialization parameter for P matrix (P = δI)
    pub fn new(order: usize, lambda: f64, delta: f64) -> Self {
        let order = order.max(1);
        let mut p_matrix = vec![0.0; order * order];
        // Initialize P = delta * I
        for i in 0..order {
            p_matrix[i * order + i] = delta;
        }

        Self {
            order,
            lambda: lambda.clamp(0.9, 1.0),
            weights: vec![0.0; order],
            p_matrix,
            samples_processed: 0,
            mse: 0.0,
            mse_alpha: 0.01,
        }
    }

    /// Update the filter with a new input vector and desired output.
    /// Returns (filter_output, error).
    pub fn update(&mut self, input: &[f64], desired: f64) -> (f64, f64) {
        assert!(input.len() >= self.order);
        let x = &input[..self.order];
        let n = self.order;

        // Filter output: y = w^T * x
        let y: f64 = self.weights.iter().zip(x).map(|(w, xi)| w * xi).sum();

        // Error: e = d - y
        let error = desired - y;

        // Kalman gain: k = P*x / (lambda + x^T*P*x)
        let mut px = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                px[i] += self.p_matrix[i * n + j] * x[j];
            }
        }

        let mut xpx = 0.0;
        for j in 0..n {
            xpx += x[j] * px[j];
        }

        let denom = self.lambda + xpx;
        let mut k = vec![0.0; n];
        for i in 0..n {
            k[i] = px[i] / denom;
        }

        // Update weights: w = w + k * e
        for i in 0..n {
            self.weights[i] += k[i] * error;
        }

        // Update P: P = (1/lambda) * (P - k * x^T * P)
        // P = (1/lambda) * (P - k * px^T)  [since px = Px]
        let inv_lambda = 1.0 / self.lambda;
        for i in 0..n {
            for j in 0..n {
                self.p_matrix[i * n + j] =
                    inv_lambda * (self.p_matrix[i * n + j] - k[i] * px[j]);
            }
        }

        self.samples_processed += 1;
        self.mse = (1.0 - self.mse_alpha) * self.mse + self.mse_alpha * error * error;

        (y, error)
    }

    /// Get the current filter weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Get the filter order.
    pub fn order(&self) -> usize {
        self.order
    }

    /// Get the forgetting factor.
    pub fn lambda(&self) -> f64 {
        self.lambda
    }

    /// Get the current MSE estimate.
    pub fn mse(&self) -> f64 {
        self.mse
    }

    /// Get total samples processed.
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed
    }

    /// Reset the filter.
    pub fn reset(&mut self, delta: f64) {
        self.weights = vec![0.0; self.order];
        self.p_matrix = vec![0.0; self.order * self.order];
        for i in 0..self.order {
            self.p_matrix[i * self.order + i] = delta;
        }
        self.samples_processed = 0;
        self.mse = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_system_identification() {
        // Identify a simple 3-tap FIR system: h = [0.5, 0.3, 0.1]
        let h = [0.5, 0.3, 0.1];
        let mut rls = RlsFilter::new(3, 0.99, 100.0);

        let mut input_buf = [0.0_f64; 3];
        let mut total_error_sq = 0.0;

        for i in 0..200 {
            // Simple deterministic input.
            let x = ((i as f64) * 0.7).sin();
            input_buf[2] = input_buf[1];
            input_buf[1] = input_buf[0];
            input_buf[0] = x;

            let desired: f64 = h.iter().zip(input_buf.iter()).map(|(hi, xi)| hi * xi).sum();
            let (_, err) = rls.update(&input_buf, desired);
            if i > 100 {
                total_error_sq += err * err;
            }
        }

        // After convergence, weights should be close to h.
        let w = rls.weights();
        assert!((w[0] - 0.5).abs() < 0.05, "w[0]={} should be ~0.5", w[0]);
        assert!((w[1] - 0.3).abs() < 0.05, "w[1]={} should be ~0.3", w[1]);
        assert!((w[2] - 0.1).abs() < 0.05, "w[2]={} should be ~0.1", w[2]);
    }

    #[test]
    fn test_convergence_speed() {
        let mut rls = RlsFilter::new(4, 0.99, 100.0);
        let h = [1.0, -0.5, 0.25, -0.125];
        let mut buf = [0.0_f64; 4];

        for i in 0..50 {
            let x = if i % 2 == 0 { 1.0 } else { -1.0 };
            buf[3] = buf[2];
            buf[2] = buf[1];
            buf[1] = buf[0];
            buf[0] = x;
            let d: f64 = h.iter().zip(buf.iter()).map(|(a, b)| a * b).sum();
            rls.update(&buf, d);
        }
        // RLS should converge fast (within 50 samples).
        assert!(rls.mse() < 0.1, "MSE should be small after 50 samples");
    }

    #[test]
    fn test_order() {
        let rls = RlsFilter::new(8, 0.99, 100.0);
        assert_eq!(rls.order(), 8);
        assert_eq!(rls.weights().len(), 8);
    }

    #[test]
    fn test_lambda() {
        let rls = RlsFilter::new(4, 0.95, 50.0);
        assert!((rls.lambda() - 0.95).abs() < 1e-10);
    }

    #[test]
    fn test_lambda_clamping() {
        let rls = RlsFilter::new(4, 0.5, 50.0);
        assert!(rls.lambda() >= 0.9); // clamped to [0.9, 1.0]
    }

    #[test]
    fn test_reset() {
        let mut rls = RlsFilter::new(4, 0.99, 100.0);
        let input = [1.0, 0.0, 0.0, 0.0];
        rls.update(&input, 1.0);
        assert!(rls.samples_processed() > 0);
        rls.reset(100.0);
        assert_eq!(rls.samples_processed(), 0);
        assert!(rls.weights().iter().all(|&w| w == 0.0));
    }

    #[test]
    fn test_single_tap() {
        // Single tap should converge to desired/input ratio.
        let mut rls = RlsFilter::new(1, 0.99, 100.0);
        for _ in 0..100 {
            rls.update(&[2.0], 6.0); // w should converge to 3.0
        }
        assert!(
            (rls.weights()[0] - 3.0).abs() < 0.1,
            "w={} should be ~3.0",
            rls.weights()[0]
        );
    }

    #[test]
    fn test_output_equals_w_dot_x() {
        let mut rls = RlsFilter::new(3, 0.99, 100.0);
        let input = [1.0, 2.0, 3.0];
        let (y, _) = rls.update(&input, 0.0);
        // First iteration with zero weights: y should be 0.
        assert!((y - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_mse_decreases() {
        let mut rls = RlsFilter::new(2, 0.99, 100.0);
        let mut mse_start = f64::MAX;
        let mut buf = [0.0_f64; 2];

        for i in 0..200 {
            let x = ((i as f64) * 0.5).sin();
            buf[1] = buf[0];
            buf[0] = x;
            let d = 0.8 * buf[0] + 0.2 * buf[1];
            rls.update(&buf, d);
            if i == 20 {
                mse_start = rls.mse();
            }
        }
        assert!(rls.mse() < mse_start, "MSE should decrease over time");
    }

    #[test]
    fn test_samples_processed() {
        let mut rls = RlsFilter::new(2, 0.99, 100.0);
        for _ in 0..10 {
            rls.update(&[1.0, 0.0], 0.5);
        }
        assert_eq!(rls.samples_processed(), 10);
    }
}

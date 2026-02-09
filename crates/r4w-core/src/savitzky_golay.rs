//! Savitzky-Golay — Polynomial Smoothing & Differentiation Filter
//!
//! Fits a local polynomial of degree `p` to a sliding window of `2m+1`
//! samples, producing smoothed values and optional derivatives.
//! Preserves peak shapes, widths, and positions better than moving
//! averages. Widely used in spectral processing, chromatography,
//! and signal conditioning.
//!
//! Reference: Savitzky & Golay, "Smoothing and Differentiation of Data
//! by Simplified Least Squares Procedures" (Analytical Chemistry, 1964).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::savitzky_golay::{SavitzkyGolay, sg_smooth};
//!
//! // Smooth with window=5 (2*2+1), polynomial order 2
//! let noisy = vec![1.0, 1.2, 2.0, 1.8, 1.1, 0.9, 1.5, 1.3, 1.0];
//! let smooth = sg_smooth(&noisy, 2, 2);
//! assert_eq!(smooth.len(), noisy.len());
//!
//! // Using the struct
//! let sg = SavitzkyGolay::new(2, 2).unwrap();
//! let result = sg.smooth(&noisy);
//! assert_eq!(result.len(), noisy.len());
//! ```

/// Error types for Savitzky-Golay operations.
#[derive(Debug, Clone, PartialEq)]
pub enum SgError {
    /// Polynomial order must be less than window size.
    OrderTooHigh { order: usize, window: usize },
    /// Window half-width must be positive.
    ZeroHalfWidth,
}

impl std::fmt::Display for SgError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OrderTooHigh { order, window } => {
                write!(f, "polynomial order {} >= window size {}", order, window)
            }
            Self::ZeroHalfWidth => write!(f, "half-width must be > 0"),
        }
    }
}

impl std::error::Error for SgError {}

/// Compute Savitzky-Golay coefficients for smoothing or differentiation.
///
/// * `half_width` — m, giving window size 2m+1
/// * `poly_order` — polynomial degree (must be < 2m+1)
/// * `deriv_order` — derivative order (0 = smoothing, 1 = first deriv, etc.)
///
/// Returns coefficient vector of length 2m+1.
pub fn sg_coefficients(
    half_width: usize,
    poly_order: usize,
    deriv_order: usize,
) -> Result<Vec<f64>, SgError> {
    let m = half_width;
    if m == 0 {
        return Err(SgError::ZeroHalfWidth);
    }
    let window = 2 * m + 1;
    if poly_order >= window {
        return Err(SgError::OrderTooHigh {
            order: poly_order,
            window,
        });
    }

    let p = poly_order + 1;

    // Build the Vandermonde-like matrix J where J[i][k] = i^k
    // for i in -m..=m, k in 0..p
    let n = window;
    let mut j = vec![vec![0.0; p]; n];
    for (idx, row) in j.iter_mut().enumerate() {
        let x = idx as f64 - m as f64;
        let mut xk = 1.0;
        for k in 0..p {
            row[k] = xk;
            xk *= x;
        }
    }

    // Compute (J^T J) via normal equations
    let mut jtj = vec![vec![0.0; p]; p];
    for row in 0..p {
        for col in 0..p {
            let mut sum = 0.0;
            for i in 0..n {
                sum += j[i][row] * j[i][col];
            }
            jtj[row][col] = sum;
        }
    }

    // Solve (J^T J) C = J^T via Gauss-Jordan elimination
    // We need the row of the pseudoinverse corresponding to deriv_order
    // Build augmented matrix [JTJ | I]
    let mut aug = vec![vec![0.0; 2 * p]; p];
    for i in 0..p {
        for jj in 0..p {
            aug[i][jj] = jtj[i][jj];
        }
        aug[i][p + i] = 1.0;
    }

    // Gauss-Jordan
    for col in 0..p {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..p {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-15 {
            // Singular matrix — reduce order
            return sg_coefficients(half_width, poly_order.saturating_sub(1), deriv_order);
        }

        for jj in 0..2 * p {
            aug[col][jj] /= pivot;
        }

        for row in 0..p {
            if row != col {
                let factor = aug[row][col];
                for jj in 0..2 * p {
                    aug[row][jj] -= factor * aug[col][jj];
                }
            }
        }
    }

    // Extract inverse: right half of augmented matrix
    let mut inv_jtj = vec![vec![0.0; p]; p];
    for i in 0..p {
        for jj in 0..p {
            inv_jtj[i][jj] = aug[i][p + jj];
        }
    }

    // Compute coefficients: c_i = sum_k inv_jtj[deriv_order][k] * J[i][k] * deriv_order!
    let d = deriv_order.min(p - 1);
    let factorial: f64 = (1..=d).map(|x| x as f64).product::<f64>().max(1.0);

    let mut coeffs = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for k in 0..p {
            sum += inv_jtj[d][k] * j[i][k];
        }
        coeffs[i] = sum * factorial;
    }

    Ok(coeffs)
}

/// Apply Savitzky-Golay smoothing to a signal.
///
/// * `data` — input signal
/// * `half_width` — m, giving window 2m+1
/// * `poly_order` — polynomial degree
pub fn sg_smooth(data: &[f64], half_width: usize, poly_order: usize) -> Vec<f64> {
    let coeffs = match sg_coefficients(half_width, poly_order, 0) {
        Ok(c) => c,
        Err(_) => return data.to_vec(),
    };

    apply_symmetric_filter(data, &coeffs, half_width)
}

/// Apply Savitzky-Golay first derivative to a signal.
pub fn sg_derivative(data: &[f64], half_width: usize, poly_order: usize) -> Vec<f64> {
    let coeffs = match sg_coefficients(half_width, poly_order, 1) {
        Ok(c) => c,
        Err(_) => return vec![0.0; data.len()],
    };

    apply_symmetric_filter(data, &coeffs, half_width)
}

/// Apply a symmetric filter with edge handling.
fn apply_symmetric_filter(data: &[f64], coeffs: &[f64], half_width: usize) -> Vec<f64> {
    let n = data.len();
    let m = half_width;
    let mut output = vec![0.0; n];

    for i in 0..n {
        let mut sum = 0.0;
        for (k, &c) in coeffs.iter().enumerate() {
            let j = i as i64 + k as i64 - m as i64;
            let idx = if j < 0 {
                (-j) as usize // Mirror at start
            } else if j >= n as i64 {
                2 * n - 2 - j as usize // Mirror at end
            } else {
                j as usize
            };
            let idx = idx.min(n - 1);
            sum += c * data[idx];
        }
        output[i] = sum;
    }

    output
}

/// Savitzky-Golay filter with cached coefficients.
#[derive(Debug, Clone)]
pub struct SavitzkyGolay {
    half_width: usize,
    poly_order: usize,
    smooth_coeffs: Vec<f64>,
    deriv_coeffs: Vec<f64>,
}

impl SavitzkyGolay {
    /// Create a new Savitzky-Golay filter.
    pub fn new(half_width: usize, poly_order: usize) -> Result<Self, SgError> {
        let smooth = sg_coefficients(half_width, poly_order, 0)?;
        let deriv = sg_coefficients(half_width, poly_order, 1)?;
        Ok(Self {
            half_width,
            poly_order,
            smooth_coeffs: smooth,
            deriv_coeffs: deriv,
        })
    }

    /// Apply smoothing.
    pub fn smooth(&self, data: &[f64]) -> Vec<f64> {
        apply_symmetric_filter(data, &self.smooth_coeffs, self.half_width)
    }

    /// Compute first derivative.
    pub fn derivative(&self, data: &[f64]) -> Vec<f64> {
        apply_symmetric_filter(data, &self.deriv_coeffs, self.half_width)
    }

    /// Get window size (2m+1).
    pub fn window_size(&self) -> usize {
        2 * self.half_width + 1
    }

    /// Get polynomial order.
    pub fn poly_order(&self) -> usize {
        self.poly_order
    }

    /// Get smoothing coefficients.
    pub fn smooth_coefficients(&self) -> &[f64] {
        &self.smooth_coeffs
    }
}

/// Compute Savitzky-Golay coefficients for common configurations.
pub mod presets {
    use super::sg_coefficients;

    /// 5-point quadratic smoothing (m=2, p=2).
    pub fn smooth_5_quadratic() -> Vec<f64> {
        sg_coefficients(2, 2, 0).unwrap()
    }

    /// 7-point quadratic smoothing (m=3, p=2).
    pub fn smooth_7_quadratic() -> Vec<f64> {
        sg_coefficients(3, 2, 0).unwrap()
    }

    /// 9-point quartic smoothing (m=4, p=4).
    pub fn smooth_9_quartic() -> Vec<f64> {
        sg_coefficients(4, 4, 0).unwrap()
    }

    /// 5-point quadratic first derivative (m=2, p=2).
    pub fn deriv_5_quadratic() -> Vec<f64> {
        sg_coefficients(2, 2, 1).unwrap()
    }

    /// 7-point quadratic first derivative (m=3, p=2).
    pub fn deriv_7_quadratic() -> Vec<f64> {
        sg_coefficients(3, 2, 1).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smooth_constant_signal() {
        let data = vec![5.0; 20];
        let smoothed = sg_smooth(&data, 3, 2);
        for &v in &smoothed {
            assert!((v - 5.0).abs() < 1e-10, "Constant signal should be unchanged: {}", v);
        }
    }

    #[test]
    fn test_smooth_linear_signal() {
        // A linear signal should pass through a polynomial filter of order >= 1
        let data: Vec<f64> = (0..20).map(|i| 2.0 * i as f64 + 1.0).collect();
        let smoothed = sg_smooth(&data, 3, 2);
        // Interior points (away from edges) should be exact
        for i in 3..17 {
            assert!(
                (smoothed[i] - data[i]).abs() < 1e-8,
                "Linear signal should be preserved at index {}: got {}, expected {}",
                i,
                smoothed[i],
                data[i]
            );
        }
    }

    #[test]
    fn test_smooth_reduces_noise() {
        // Noisy sinusoid
        let data: Vec<f64> = (0..100)
            .map(|i| {
                let t = i as f64 / 100.0;
                (2.0 * std::f64::consts::PI * 3.0 * t).sin()
                    + 0.3 * ((i * 7 + 3) as f64 * 0.1).sin() // pseudo-noise
            })
            .collect();

        let smoothed = sg_smooth(&data, 5, 3);

        // Smoothed signal should have less variance than original
        let var_orig: f64 = data.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>();
        let var_smooth: f64 = smoothed
            .windows(2)
            .map(|w| (w[1] - w[0]).powi(2))
            .sum::<f64>();

        assert!(
            var_smooth < var_orig,
            "Smoothed signal should have less sample-to-sample variation"
        );
    }

    #[test]
    fn test_derivative_of_linear() {
        // Derivative of y = 2x + 1 should be 2
        let data: Vec<f64> = (0..20).map(|i| 2.0 * i as f64 + 1.0).collect();
        let deriv = sg_derivative(&data, 3, 2);
        // Interior points should be close to 2
        for i in 3..17 {
            assert!(
                (deriv[i] - 2.0).abs() < 0.1,
                "Derivative at {} = {}, expected ~2.0",
                i,
                deriv[i]
            );
        }
    }

    #[test]
    fn test_derivative_of_quadratic() {
        // y = x^2, dy/dx = 2x
        let data: Vec<f64> = (0..20).map(|i| (i as f64).powi(2)).collect();
        let deriv = sg_derivative(&data, 3, 3);
        // Interior: derivative at x=10 should be ~20
        for i in 5..15 {
            let expected = 2.0 * i as f64;
            assert!(
                (deriv[i] - expected).abs() < 1.0,
                "Derivative at {} = {}, expected ~{}",
                i,
                deriv[i],
                expected
            );
        }
    }

    #[test]
    fn test_coefficients_symmetry() {
        let coeffs = sg_coefficients(3, 2, 0).unwrap();
        assert_eq!(coeffs.len(), 7);
        // Smoothing coefficients should be symmetric
        for i in 0..3 {
            assert!(
                (coeffs[i] - coeffs[6 - i]).abs() < 1e-10,
                "Coefficients should be symmetric"
            );
        }
    }

    #[test]
    fn test_coefficients_sum_to_one() {
        let coeffs = sg_coefficients(3, 2, 0).unwrap();
        let sum: f64 = coeffs.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-10,
            "Smoothing coefficients should sum to 1, got {}",
            sum
        );
    }

    #[test]
    fn test_derivative_coefficients_antisymmetric() {
        let coeffs = sg_coefficients(3, 2, 1).unwrap();
        assert_eq!(coeffs.len(), 7);
        // First derivative coefficients should be antisymmetric
        for i in 0..3 {
            assert!(
                (coeffs[i] + coeffs[6 - i]).abs() < 1e-10,
                "Derivative coefficients should be antisymmetric"
            );
        }
    }

    #[test]
    fn test_sg_struct() {
        let sg = SavitzkyGolay::new(2, 2).unwrap();
        assert_eq!(sg.window_size(), 5);
        assert_eq!(sg.poly_order(), 2);
        let data = vec![1.0; 10];
        let smoothed = sg.smooth(&data);
        assert_eq!(smoothed.len(), 10);
    }

    #[test]
    fn test_sg_error_order_too_high() {
        let result = SavitzkyGolay::new(1, 5); // window=3, order=5
        assert!(result.is_err());
    }

    #[test]
    fn test_presets() {
        let c5 = presets::smooth_5_quadratic();
        assert_eq!(c5.len(), 5);
        let sum: f64 = c5.iter().sum();
        assert!((sum - 1.0).abs() < 1e-10);

        let c7 = presets::smooth_7_quadratic();
        assert_eq!(c7.len(), 7);

        let d5 = presets::deriv_5_quadratic();
        assert_eq!(d5.len(), 5);
    }

    #[test]
    fn test_short_signal() {
        let data = vec![1.0, 3.0, 2.0];
        let smoothed = sg_smooth(&data, 1, 1);
        assert_eq!(smoothed.len(), 3);
    }

    #[test]
    fn test_peak_preservation() {
        // S-G filter should preserve peak position better than moving average
        let mut data = vec![0.0; 50];
        data[25] = 10.0; // Sharp peak

        let smoothed = sg_smooth(&data, 3, 4);

        // Peak should still be near index 25
        let peak_idx = smoothed
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, 25, "Peak should be preserved at original position");
    }
}

//! # Lagrange Polynomial Interpolator
//!
//! Implements Lagrange polynomial interpolation for arbitrary-precision sample rate
//! conversion and signal reconstruction. Provides both the classical Lagrange form
//! and the more efficient barycentric form for repeated interpolation, as well as
//! Newton's divided-difference form.
//!
//! ## Features
//!
//! - Configurable polynomial order (1 to 20)
//! - Barycentric weights for O(n) per-query evaluation after O(n²) precomputation
//! - Complex signal interpolation using `(f64, f64)` tuples
//! - Fractional delay filter
//! - Non-uniform to uniform resampling
//! - Arbitrary ratio sample rate conversion
//! - Interpolation error estimation
//! - Newton form via divided differences
//!
//! ## Example
//!
//! ```
//! use r4w_core::lagrange_polynomial_interpolator::LagrangeInterpolator;
//!
//! // Interpolate y = x² from 3 points
//! let xs = vec![0.0, 1.0, 2.0];
//! let ys = vec![0.0, 1.0, 4.0];
//! let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
//! let val = interp.evaluate(1.5);
//! assert!((val - 2.25).abs() < 1e-12);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// LagrangeInterpolator – classical + barycentric forms
// ---------------------------------------------------------------------------

/// Lagrange polynomial interpolator with barycentric weights.
///
/// Stores the nodes `xs`, values `ys`, and precomputed barycentric weights so that
/// each subsequent evaluation is O(n) rather than O(n²).
#[derive(Debug, Clone)]
pub struct LagrangeInterpolator {
    xs: Vec<f64>,
    ys: Vec<f64>,
    weights: Vec<f64>, // barycentric weights
}

impl LagrangeInterpolator {
    /// Create a new interpolator from node positions `xs` and values `ys`.
    ///
    /// Returns `Err` if the lengths differ, fewer than 2 points are provided,
    /// or any two x-values coincide.
    pub fn new(xs: &[f64], ys: &[f64]) -> Result<Self, InterpolationError> {
        if xs.len() != ys.len() {
            return Err(InterpolationError::LengthMismatch);
        }
        if xs.len() < 2 {
            return Err(InterpolationError::TooFewPoints);
        }
        // Check for duplicate nodes
        for i in 0..xs.len() {
            for j in (i + 1)..xs.len() {
                if (xs[i] - xs[j]).abs() < 1e-15 {
                    return Err(InterpolationError::DuplicateNodes);
                }
            }
        }
        let weights = barycentric_weights(xs);
        Ok(Self {
            xs: xs.to_vec(),
            ys: ys.to_vec(),
            weights,
        })
    }

    /// Number of interpolation nodes (polynomial order + 1).
    pub fn order(&self) -> usize {
        self.xs.len() - 1
    }

    // -- Classical Lagrange basis ------------------------------------------

    /// Evaluate the j-th Lagrange basis polynomial at `x`.
    ///
    /// L_j(x) = ∏_{m≠j} (x - x_m) / (x_j - x_m)
    pub fn basis(&self, j: usize, x: f64) -> f64 {
        lagrange_basis(&self.xs, j, x)
    }

    /// Evaluate the interpolating polynomial at `x` using the classical form.
    ///
    /// P(x) = Σ_j y_j · L_j(x)
    pub fn evaluate_classical(&self, x: f64) -> f64 {
        let n = self.xs.len();
        let mut result = 0.0;
        for j in 0..n {
            result += self.ys[j] * self.basis(j, x);
        }
        result
    }

    // -- Barycentric form --------------------------------------------------

    /// Evaluate the interpolating polynomial at `x` using the barycentric form.
    ///
    /// This is O(n) per evaluation (after the O(n²) weight precomputation).
    pub fn evaluate(&self, x: f64) -> f64 {
        barycentric_eval(&self.xs, &self.ys, &self.weights, x)
    }

    /// Return a reference to the precomputed barycentric weights.
    pub fn barycentric_weights(&self) -> &[f64] {
        &self.weights
    }
}

// ---------------------------------------------------------------------------
// Free-standing helpers
// ---------------------------------------------------------------------------

/// Evaluate the j-th Lagrange basis polynomial at `x` for the given nodes.
pub fn lagrange_basis(xs: &[f64], j: usize, x: f64) -> f64 {
    let mut prod = 1.0;
    for (m, &xm) in xs.iter().enumerate() {
        if m != j {
            prod *= (x - xm) / (xs[j] - xm);
        }
    }
    prod
}

/// Compute barycentric weights for the given nodes.
///
/// w_j = 1 / ∏_{m≠j} (x_j - x_m)
pub fn barycentric_weights(xs: &[f64]) -> Vec<f64> {
    let n = xs.len();
    let mut w = vec![1.0; n];
    for j in 0..n {
        for m in 0..n {
            if m != j {
                w[j] /= xs[j] - xs[m];
            }
        }
    }
    w
}

/// Barycentric interpolation evaluation.
///
/// Uses the second form of the barycentric formula:
///   P(x) = [ Σ_j w_j / (x - x_j) · y_j ] / [ Σ_j w_j / (x - x_j) ]
///
/// If `x` is exactly a node, the stored value is returned directly.
pub fn barycentric_eval(xs: &[f64], ys: &[f64], weights: &[f64], x: f64) -> f64 {
    let n = xs.len();
    // Check if x coincides with a node
    for i in 0..n {
        let diff = x - xs[i];
        if diff.abs() < 1e-15 {
            return ys[i];
        }
    }
    let mut numer = 0.0;
    let mut denom = 0.0;
    for j in 0..n {
        let t = weights[j] / (x - xs[j]);
        numer += t * ys[j];
        denom += t;
    }
    numer / denom
}

// ---------------------------------------------------------------------------
// Complex interpolation
// ---------------------------------------------------------------------------

/// Interpolate a complex-valued signal at fractional position `x`.
///
/// Real and imaginary parts are interpolated independently using the
/// barycentric form.
pub fn interpolate_complex(
    xs: &[f64],
    samples: &[(f64, f64)],
    weights: &[f64],
    x: f64,
) -> (f64, f64) {
    let re: Vec<f64> = samples.iter().map(|s| s.0).collect();
    let im: Vec<f64> = samples.iter().map(|s| s.1).collect();
    let out_re = barycentric_eval(xs, &re, weights, x);
    let out_im = barycentric_eval(xs, &im, weights, x);
    (out_re, out_im)
}

// ---------------------------------------------------------------------------
// Fractional delay
// ---------------------------------------------------------------------------

/// Apply a fractional delay to a uniformly sampled signal.
///
/// Given `samples` at integer indices `0, 1, 2, …`, returns the value at
/// position `delay` (in samples) using Lagrange interpolation of the given
/// `order`. The `order + 1` nearest samples centred around the delay are used.
///
/// Returns `Err` if there are not enough samples for the chosen order.
pub fn fractional_delay(
    samples: &[f64],
    delay: f64,
    order: usize,
) -> Result<f64, InterpolationError> {
    if order < 1 || order > 20 {
        return Err(InterpolationError::InvalidOrder);
    }
    let n_points = order + 1;
    if samples.len() < n_points {
        return Err(InterpolationError::TooFewPoints);
    }

    // Centre the window around `delay`
    let centre = delay.floor() as isize;
    let half = (n_points as isize) / 2;
    let mut start = centre - half + 1;
    // Clamp to valid range
    if start < 0 {
        start = 0;
    }
    if start + n_points as isize > samples.len() as isize {
        start = samples.len() as isize - n_points as isize;
    }
    let start = start as usize;

    let xs: Vec<f64> = (0..n_points).map(|i| (start + i) as f64).collect();
    let ys: Vec<f64> = samples[start..start + n_points].to_vec();
    let w = barycentric_weights(&xs);
    Ok(barycentric_eval(&xs, &ys, &w, delay))
}

/// Apply a fractional delay to a complex signal.
pub fn fractional_delay_complex(
    samples: &[(f64, f64)],
    delay: f64,
    order: usize,
) -> Result<(f64, f64), InterpolationError> {
    let re: Vec<f64> = samples.iter().map(|s| s.0).collect();
    let im: Vec<f64> = samples.iter().map(|s| s.1).collect();
    let out_re = fractional_delay(&re, delay, order)?;
    let out_im = fractional_delay(&im, delay, order)?;
    Ok((out_re, out_im))
}

// ---------------------------------------------------------------------------
// Non-uniform → uniform resampling
// ---------------------------------------------------------------------------

/// Resample a non-uniformly sampled signal onto a uniform grid.
///
/// Given `(t, y)` pairs (not necessarily sorted), produces `n_out` uniformly
/// spaced samples covering `[t_min, t_max]`. Uses local Lagrange interpolation
/// of the specified `order`.
pub fn resample_nonuniform_to_uniform(
    points: &[(f64, f64)],
    n_out: usize,
    order: usize,
) -> Result<Vec<f64>, InterpolationError> {
    if points.len() < order + 1 {
        return Err(InterpolationError::TooFewPoints);
    }
    if order < 1 || order > 20 {
        return Err(InterpolationError::InvalidOrder);
    }
    if n_out == 0 {
        return Ok(Vec::new());
    }

    // Sort by time
    let mut sorted: Vec<(f64, f64)> = points.to_vec();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let t_min = sorted.first().unwrap().0;
    let t_max = sorted.last().unwrap().0;
    let step = if n_out > 1 {
        (t_max - t_min) / (n_out - 1) as f64
    } else {
        0.0
    };

    let n_pts = order + 1;
    let mut output = Vec::with_capacity(n_out);

    for k in 0..n_out {
        let t = t_min + k as f64 * step;

        // Find the nearest cluster of n_pts points
        let mut best = 0usize;
        let mut best_dist = f64::MAX;
        for i in 0..sorted.len() {
            let d = (sorted[i].0 - t).abs();
            if d < best_dist {
                best_dist = d;
                best = i;
            }
        }

        // Build window centred on `best`
        let half = n_pts / 2;
        let start = if best >= half { best - half } else { 0 };
        let start = start.min(sorted.len().saturating_sub(n_pts));
        let end = (start + n_pts).min(sorted.len());

        let xs: Vec<f64> = sorted[start..end].iter().map(|p| p.0).collect();
        let ys: Vec<f64> = sorted[start..end].iter().map(|p| p.1).collect();

        let w = barycentric_weights(&xs);
        output.push(barycentric_eval(&xs, &ys, &w, t));
    }

    Ok(output)
}

// ---------------------------------------------------------------------------
// Arbitrary ratio sample rate conversion
// ---------------------------------------------------------------------------

/// Resample a uniformly sampled signal by an arbitrary ratio `rate_out / rate_in`.
///
/// Uses local Lagrange interpolation of the given `order`.
pub fn resample_arbitrary(
    input: &[f64],
    rate_in: f64,
    rate_out: f64,
    order: usize,
) -> Result<Vec<f64>, InterpolationError> {
    if input.len() < order + 1 {
        return Err(InterpolationError::TooFewPoints);
    }
    if order < 1 || order > 20 {
        return Err(InterpolationError::InvalidOrder);
    }
    if rate_in <= 0.0 || rate_out <= 0.0 {
        return Err(InterpolationError::InvalidRate);
    }

    let duration = (input.len() - 1) as f64 / rate_in;
    let n_out = (duration * rate_out).floor() as usize + 1;
    let ratio = rate_in / rate_out;

    let mut output = Vec::with_capacity(n_out);

    for k in 0..n_out {
        let pos = k as f64 * ratio; // position in input sample indices
        output.push(fractional_delay(input, pos, order)?);
    }

    Ok(output)
}

/// Resample a complex signal by an arbitrary ratio.
pub fn resample_arbitrary_complex(
    input: &[(f64, f64)],
    rate_in: f64,
    rate_out: f64,
    order: usize,
) -> Result<Vec<(f64, f64)>, InterpolationError> {
    let re: Vec<f64> = input.iter().map(|s| s.0).collect();
    let im: Vec<f64> = input.iter().map(|s| s.1).collect();
    let out_re = resample_arbitrary(&re, rate_in, rate_out, order)?;
    let out_im = resample_arbitrary(&im, rate_in, rate_out, order)?;
    Ok(out_re.into_iter().zip(out_im).collect())
}

// ---------------------------------------------------------------------------
// Error estimation
// ---------------------------------------------------------------------------

/// Estimate the interpolation error at `x` by comparing interpolation with
/// `n` and `n-1` nodes (dropping the node farthest from `x`).
///
/// A larger difference suggests the polynomial order may be insufficient.
pub fn estimate_error(xs: &[f64], ys: &[f64], x: f64) -> Result<f64, InterpolationError> {
    if xs.len() < 3 {
        return Err(InterpolationError::TooFewPoints);
    }
    if xs.len() != ys.len() {
        return Err(InterpolationError::LengthMismatch);
    }

    let w_full = barycentric_weights(xs);
    let val_full = barycentric_eval(xs, ys, &w_full, x);

    // Drop the farthest node
    let farthest = xs
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            ((*a) - x)
                .abs()
                .partial_cmp(&((*b) - x).abs())
                .unwrap()
        })
        .unwrap()
        .0;

    let mut xs_red: Vec<f64> = xs.to_vec();
    let mut ys_red: Vec<f64> = ys.to_vec();
    xs_red.remove(farthest);
    ys_red.remove(farthest);

    let w_red = barycentric_weights(&xs_red);
    let val_red = barycentric_eval(&xs_red, &ys_red, &w_red, x);

    Ok((val_full - val_red).abs())
}

// ---------------------------------------------------------------------------
// Newton form (divided differences)
// ---------------------------------------------------------------------------

/// Newton form of the interpolation polynomial using divided differences.
#[derive(Debug, Clone)]
pub struct NewtonInterpolator {
    xs: Vec<f64>,
    coeffs: Vec<f64>, // divided difference coefficients
}

impl NewtonInterpolator {
    /// Build the Newton interpolation from nodes `xs` and values `ys`.
    pub fn new(xs: &[f64], ys: &[f64]) -> Result<Self, InterpolationError> {
        if xs.len() != ys.len() {
            return Err(InterpolationError::LengthMismatch);
        }
        if xs.len() < 2 {
            return Err(InterpolationError::TooFewPoints);
        }
        let coeffs = divided_differences(xs, ys);
        Ok(Self {
            xs: xs.to_vec(),
            coeffs,
        })
    }

    /// Evaluate the Newton interpolating polynomial at `x` using Horner's method.
    pub fn evaluate(&self, x: f64) -> f64 {
        let n = self.coeffs.len();
        let mut result = self.coeffs[n - 1];
        for i in (0..n - 1).rev() {
            result = result * (x - self.xs[i]) + self.coeffs[i];
        }
        result
    }

    /// Return the divided-difference coefficients.
    pub fn coefficients(&self) -> &[f64] {
        &self.coeffs
    }
}

/// Compute the divided-difference table and return the top-row coefficients.
pub fn divided_differences(xs: &[f64], ys: &[f64]) -> Vec<f64> {
    let n = xs.len();
    let mut table = ys.to_vec();
    let mut coeffs = vec![table[0]];
    for k in 1..n {
        for i in (k..n).rev() {
            table[i] = (table[i] - table[i - 1]) / (xs[i] - xs[i - k]);
        }
        coeffs.push(table[k]);
    }
    coeffs
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors that can occur during interpolation.
#[derive(Debug, Clone, PartialEq)]
pub enum InterpolationError {
    /// x and y arrays have different lengths.
    LengthMismatch,
    /// Fewer than 2 points supplied.
    TooFewPoints,
    /// Two or more x-values are identical.
    DuplicateNodes,
    /// Polynomial order out of range (must be 1..=20).
    InvalidOrder,
    /// Sample rate must be positive.
    InvalidRate,
}

impl std::fmt::Display for InterpolationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LengthMismatch => write!(f, "x and y arrays differ in length"),
            Self::TooFewPoints => write!(f, "fewer than 2 interpolation points"),
            Self::DuplicateNodes => write!(f, "duplicate x-values in nodes"),
            Self::InvalidOrder => write!(f, "polynomial order out of range (1..=20)"),
            Self::InvalidRate => write!(f, "sample rate must be positive"),
        }
    }
}

impl std::error::Error for InterpolationError {}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    // -- 1. Basic quadratic interpolation ----------------------------------
    #[test]
    fn test_quadratic_interpolation() {
        // y = x²  from 3 points => exact
        let xs = vec![0.0, 1.0, 2.0];
        let ys = vec![0.0, 1.0, 4.0];
        let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
        assert!((interp.evaluate(0.5) - 0.25).abs() < EPS);
        assert!((interp.evaluate(1.5) - 2.25).abs() < EPS);
    }

    // -- 2. Classical vs barycentric agree ---------------------------------
    #[test]
    fn test_classical_equals_barycentric() {
        let xs = vec![-1.0, 0.0, 1.0, 2.0];
        let ys = vec![1.0, 0.0, 1.0, 8.0];
        let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
        for &x in &[-0.5, 0.25, 0.75, 1.5] {
            let c = interp.evaluate_classical(x);
            let b = interp.evaluate(x);
            assert!(
                (c - b).abs() < 1e-12,
                "classical={c} barycentric={b} at x={x}"
            );
        }
    }

    // -- 3. Interpolation at nodes returns exact values --------------------
    #[test]
    fn test_exact_at_nodes() {
        let xs = vec![1.0, 3.0, 5.0, 7.0];
        let ys = vec![2.0, -1.0, 4.0, 0.5];
        let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
        for (i, &x) in xs.iter().enumerate() {
            assert!(
                (interp.evaluate(x) - ys[i]).abs() < EPS,
                "node {i}: expected {} got {}",
                ys[i],
                interp.evaluate(x)
            );
        }
    }

    // -- 4. Basis polynomials are a partition of unity ---------------------
    #[test]
    fn test_basis_partition_of_unity() {
        let xs = vec![0.0, 1.0, 3.0, 6.0];
        let ys = vec![0.0; 4]; // values don't matter for basis check
        let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
        for &x in &[0.5, 2.0, 4.5] {
            let sum: f64 = (0..xs.len()).map(|j| interp.basis(j, x)).sum();
            assert!(
                (sum - 1.0).abs() < EPS,
                "basis sum = {sum} at x = {x}"
            );
        }
    }

    // -- 5. Linear interpolation ------------------------------------------
    #[test]
    fn test_linear_interpolation() {
        // y = 3x + 2
        let xs = vec![0.0, 10.0];
        let ys = vec![2.0, 32.0];
        let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
        assert!((interp.evaluate(5.0) - 17.0).abs() < EPS);
    }

    // -- 6. Error on length mismatch --------------------------------------
    #[test]
    fn test_error_length_mismatch() {
        let r = LagrangeInterpolator::new(&[1.0, 2.0], &[1.0]);
        assert_eq!(r.unwrap_err(), InterpolationError::LengthMismatch);
    }

    // -- 7. Error on too few points ---------------------------------------
    #[test]
    fn test_error_too_few_points() {
        let r = LagrangeInterpolator::new(&[1.0], &[1.0]);
        assert_eq!(r.unwrap_err(), InterpolationError::TooFewPoints);
    }

    // -- 8. Error on duplicate nodes --------------------------------------
    #[test]
    fn test_error_duplicate_nodes() {
        let r = LagrangeInterpolator::new(&[1.0, 1.0, 2.0], &[0.0, 1.0, 2.0]);
        assert_eq!(r.unwrap_err(), InterpolationError::DuplicateNodes);
    }

    // -- 9. Complex interpolation -----------------------------------------
    #[test]
    fn test_complex_interpolation() {
        // Complex exponential e^{j·pi·x} sampled at 5 points for good accuracy
        let xs = vec![0.0, 0.25, 0.5, 0.75, 1.0];
        let samples: Vec<(f64, f64)> = xs
            .iter()
            .map(|&x| (f64::cos(PI * x), f64::sin(PI * x)))
            .collect();
        let w = barycentric_weights(&xs);
        let (re, im) = interpolate_complex(&xs, &samples, &w, 0.125);
        let expected_re = f64::cos(PI * 0.125);
        let expected_im = f64::sin(PI * 0.125);
        // 4th-order polynomial approximation of cos/sin should be close
        assert!(
            (re - expected_re).abs() < 0.01,
            "re: {re} vs {expected_re}"
        );
        assert!(
            (im - expected_im).abs() < 0.01,
            "im: {im} vs {expected_im}"
        );
    }

    // -- 10. Fractional delay – integer delay is exact --------------------
    #[test]
    fn test_fractional_delay_integer() {
        let samples: Vec<f64> = (0..10).map(|i| i as f64 * 2.0).collect();
        // delay = 3.0 should return samples[3] = 6.0
        let val = fractional_delay(&samples, 3.0, 4).unwrap();
        assert!((val - 6.0).abs() < EPS);
    }

    // -- 11. Fractional delay – half-sample on linear ramp ----------------
    #[test]
    fn test_fractional_delay_half_sample() {
        // Linear ramp => any-order interpolation is exact
        let samples: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let val = fractional_delay(&samples, 5.5, 3).unwrap();
        assert!((val - 5.5).abs() < EPS);
    }

    // -- 12. Fractional delay complex -------------------------------------
    #[test]
    fn test_fractional_delay_complex() {
        // Linear ramp in both components
        let samples: Vec<(f64, f64)> = (0..10)
            .map(|i| (i as f64, -(i as f64)))
            .collect();
        let (re, im) = fractional_delay_complex(&samples, 4.5, 3).unwrap();
        assert!((re - 4.5).abs() < EPS);
        assert!((im - (-4.5)).abs() < EPS);
    }

    // -- 13. Newton form matches Lagrange ---------------------------------
    #[test]
    fn test_newton_matches_lagrange() {
        let xs = vec![-2.0, -1.0, 0.0, 1.0, 2.0];
        let ys: Vec<f64> = xs.iter().map(|&x| x * x * x - 2.0 * x + 1.0).collect();
        let lagrange = LagrangeInterpolator::new(&xs, &ys).unwrap();
        let newton = NewtonInterpolator::new(&xs, &ys).unwrap();
        for &x in &[-1.5, -0.3, 0.7, 1.8] {
            let l = lagrange.evaluate(x);
            let n = newton.evaluate(x);
            assert!(
                (l - n).abs() < 1e-10,
                "lagrange={l} newton={n} at x={x}"
            );
        }
    }

    // -- 14. Divided differences for linear => constant -------------------
    #[test]
    fn test_divided_differences_linear() {
        // y = 3x + 1 => coeffs = [1, 3]  (constant first diff)
        let xs = vec![0.0, 1.0, 2.0];
        let ys = vec![1.0, 4.0, 7.0];
        let dd = divided_differences(&xs, &ys);
        assert!((dd[0] - 1.0).abs() < EPS); // f[x0]
        assert!((dd[1] - 3.0).abs() < EPS); // f[x0,x1]
        assert!(dd[2].abs() < EPS); // f[x0,x1,x2] = 0 for linear
    }

    // -- 15. Arbitrary resampling – upsample 2x on linear -----------------
    #[test]
    fn test_resample_upsample_2x() {
        let input: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let out = resample_arbitrary(&input, 1.0, 2.0, 3).unwrap();
        // Output should be ~0.0, 0.5, 1.0, 1.5, …
        for (k, &v) in out.iter().enumerate() {
            let expected = k as f64 * 0.5;
            assert!(
                (v - expected).abs() < 1e-8,
                "k={k}: {v} vs {expected}"
            );
        }
    }

    // -- 16. Arbitrary resampling complex ---------------------------------
    #[test]
    fn test_resample_complex() {
        let input: Vec<(f64, f64)> = (0..10).map(|i| (i as f64, 0.0)).collect();
        let out = resample_arbitrary_complex(&input, 1.0, 2.0, 3).unwrap();
        assert!(out.len() > 10);
        // First sample should be (0, 0)
        assert!((out[0].0).abs() < EPS);
        assert!((out[0].1).abs() < EPS);
    }

    // -- 17. Non-uniform to uniform resampling ----------------------------
    #[test]
    fn test_nonuniform_to_uniform() {
        // y = x  sampled non-uniformly
        let points: Vec<(f64, f64)> = vec![
            (0.0, 0.0),
            (0.3, 0.3),
            (0.7, 0.7),
            (1.0, 1.0),
            (1.5, 1.5),
            (2.0, 2.0),
        ];
        let out = resample_nonuniform_to_uniform(&points, 5, 2).unwrap();
        // Should produce 5 points from 0.0 to 2.0
        assert_eq!(out.len(), 5);
        for (k, &v) in out.iter().enumerate() {
            let expected = k as f64 * 0.5;
            assert!(
                (v - expected).abs() < 1e-8,
                "k={k}: {v} vs {expected}"
            );
        }
    }

    // -- 18. Error estimation – low error for exact polynomial ------------
    #[test]
    fn test_error_estimation_exact() {
        // y = x² with 4 points (3rd order poly can represent x² exactly)
        // dropping a point should still represent x² exactly => error ~ 0
        let xs = vec![0.0, 1.0, 2.0, 3.0];
        let ys = vec![0.0, 1.0, 4.0, 9.0];
        let err = estimate_error(&xs, &ys, 1.5).unwrap();
        assert!(
            err < 1e-10,
            "expected near-zero error for exact polynomial, got {err}"
        );
    }

    // -- 19. Error estimation – higher error for under-sampled function ---
    #[test]
    fn test_error_estimation_nonzero() {
        // sin(x) with only 3 points – reducing to 2 should show noticeable diff
        let xs = vec![0.0, PI / 2.0, PI];
        let ys: Vec<f64> = xs.iter().map(|&x| x.sin()).collect();
        let err = estimate_error(&xs, &ys, PI / 4.0).unwrap();
        assert!(
            err > 1e-4,
            "expected nonzero error for sin with few points, got {err}"
        );
    }

    // -- 20. High-order polynomial – 5th degree exact ---------------------
    #[test]
    fn test_high_order_exact() {
        // p(x) = x^5 - 3x^3 + x  with 6 points => exact interpolation
        let xs: Vec<f64> = (0..6).map(|i| i as f64).collect();
        let poly = |x: f64| x.powi(5) - 3.0 * x.powi(3) + x;
        let ys: Vec<f64> = xs.iter().map(|&x| poly(x)).collect();
        let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
        for &x in &[0.5, 1.5, 2.5, 3.5, 4.5] {
            assert!(
                (interp.evaluate(x) - poly(x)).abs() < 1e-6,
                "at x={x}: {} vs {}",
                interp.evaluate(x),
                poly(x)
            );
        }
    }

    // -- 21. Barycentric weights for equispaced nodes ---------------------
    #[test]
    fn test_barycentric_weights_equispaced() {
        // For 3 equispaced nodes {0,1,2}: w = [0.5, -1, 0.5]
        let xs = vec![0.0, 1.0, 2.0];
        let w = barycentric_weights(&xs);
        assert!((w[0] - 0.5).abs() < EPS);
        assert!((w[1] - (-1.0)).abs() < EPS);
        assert!((w[2] - 0.5).abs() < EPS);
    }

    // -- 22. Order accessor -----------------------------------------------
    #[test]
    fn test_order_accessor() {
        let xs = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let ys = vec![0.0; 5];
        let interp = LagrangeInterpolator::new(&xs, &ys).unwrap();
        assert_eq!(interp.order(), 4);
    }
}

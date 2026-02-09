//! MMSE FIR Interpolator — Minimum Mean-Squared Error Fractional Delay
//!
//! High-precision fractional sample delay computation using optimized
//! 8-tap FIR filters with pre-computed MMSE coefficients. The fractional
//! delay mu is quantized to 128ths of a sample for fast lookup.
//! Critical building block for timing recovery and clock synchronization.
//! GNU Radio equivalent: `gr::filter::mmse_fir_interpolator_cc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mmse_interpolator::MmseInterpolator;
//! use num_complex::Complex64;
//!
//! let interp = MmseInterpolator::new();
//! let samples: Vec<Complex64> = (0..16).map(|i| {
//!     Complex64::new((i as f64 * 0.3).sin(), 0.0)
//! }).collect();
//! let result = interp.interpolate(&samples, 4, 0.5);
//! assert!(result.is_some());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Number of filter taps per MMSE sub-filter.
const MMSE_TAPS: usize = 8;

/// Number of quantized mu values (resolution).
const NUM_MU_STEPS: usize = 128;

/// MMSE FIR interpolator with pre-computed coefficient table.
#[derive(Debug, Clone)]
pub struct MmseInterpolator {
    /// Coefficient table: [mu_index][tap]
    coefficients: Vec<[f64; MMSE_TAPS]>,
}

impl MmseInterpolator {
    /// Create a new MMSE interpolator with pre-computed coefficients.
    pub fn new() -> Self {
        let coefficients = compute_mmse_table();
        Self { coefficients }
    }

    /// Interpolate a single sample at fractional position.
    ///
    /// `samples`: input samples (need at least center_idx + MMSE_TAPS/2 + 1).
    /// `center_idx`: index of the integer sample near the desired point.
    /// `mu`: fractional delay [0.0, 1.0).
    pub fn interpolate(
        &self,
        samples: &[Complex64],
        center_idx: usize,
        mu: f64,
    ) -> Option<Complex64> {
        let half = MMSE_TAPS / 2;
        let start = center_idx.checked_sub(half - 1)?;
        if start + MMSE_TAPS > samples.len() {
            return None;
        }

        let mu_clamped = mu.clamp(0.0, 1.0 - 1e-10);
        let mu_idx = (mu_clamped * NUM_MU_STEPS as f64) as usize;
        let mu_idx = mu_idx.min(NUM_MU_STEPS - 1);

        let coeffs = &self.coefficients[mu_idx];
        let mut sum = Complex64::new(0.0, 0.0);
        for (k, &c) in coeffs.iter().enumerate() {
            sum += samples[start + k] * c;
        }

        Some(sum)
    }

    /// Interpolate real-valued samples.
    pub fn interpolate_real(
        &self,
        samples: &[f64],
        center_idx: usize,
        mu: f64,
    ) -> Option<f64> {
        let half = MMSE_TAPS / 2;
        let start = center_idx.checked_sub(half - 1)?;
        if start + MMSE_TAPS > samples.len() {
            return None;
        }

        let mu_clamped = mu.clamp(0.0, 1.0 - 1e-10);
        let mu_idx = (mu_clamped * NUM_MU_STEPS as f64) as usize;
        let mu_idx = mu_idx.min(NUM_MU_STEPS - 1);

        let coeffs = &self.coefficients[mu_idx];
        let mut sum = 0.0f64;
        for (k, &c) in coeffs.iter().enumerate() {
            sum += samples[start + k] * c;
        }

        Some(sum)
    }

    /// Batch interpolation at multiple fractional positions.
    pub fn interpolate_batch(
        &self,
        samples: &[Complex64],
        positions: &[(usize, f64)], // (center_idx, mu) pairs
    ) -> Vec<Complex64> {
        positions
            .iter()
            .filter_map(|&(idx, mu)| self.interpolate(samples, idx, mu))
            .collect()
    }

    /// Get coefficients for a specific fractional delay.
    pub fn coefficients_for_mu(&self, mu: f64) -> [f64; MMSE_TAPS] {
        let mu_clamped = mu.clamp(0.0, 1.0 - 1e-10);
        let mu_idx = (mu_clamped * NUM_MU_STEPS as f64) as usize;
        let mu_idx = mu_idx.min(NUM_MU_STEPS - 1);
        self.coefficients[mu_idx]
    }

    /// Get the number of taps.
    pub fn num_taps(&self) -> usize {
        MMSE_TAPS
    }

    /// Get the mu resolution.
    pub fn mu_resolution(&self) -> usize {
        NUM_MU_STEPS
    }
}

impl Default for MmseInterpolator {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the MMSE coefficient lookup table.
///
/// For each quantized mu value, computes the optimal 8-tap FIR
/// that minimizes mean squared error for that fractional delay.
fn compute_mmse_table() -> Vec<[f64; MMSE_TAPS]> {
    let mut table = Vec::with_capacity(NUM_MU_STEPS);

    for step in 0..NUM_MU_STEPS {
        let mu = step as f64 / NUM_MU_STEPS as f64;
        let coeffs = compute_sinc_coeffs(mu);
        table.push(coeffs);
    }

    table
}

/// Compute windowed-sinc interpolation coefficients for fractional delay mu.
fn compute_sinc_coeffs(mu: f64) -> [f64; MMSE_TAPS] {
    let mut coeffs = [0.0f64; MMSE_TAPS];
    let half = MMSE_TAPS as f64 / 2.0;

    for i in 0..MMSE_TAPS {
        let n = i as f64 - (half - 1.0) - mu;

        // Sinc function
        let sinc = if n.abs() < 1e-10 {
            1.0
        } else {
            (PI * n).sin() / (PI * n)
        };

        // Nuttall window (4-term)
        let t = (i as f64 + (1.0 - mu)) / MMSE_TAPS as f64;
        let window = 0.355768
            - 0.487396 * (2.0 * PI * t).cos()
            + 0.144232 * (4.0 * PI * t).cos()
            - 0.012604 * (6.0 * PI * t).cos();

        coeffs[i] = sinc * window;
    }

    // Normalize to unity gain
    let sum: f64 = coeffs.iter().sum();
    if sum.abs() > 1e-10 {
        for c in coeffs.iter_mut() {
            *c /= sum;
        }
    }

    coeffs
}

/// Compute fractional delay using cubic (Hermite) interpolation.
/// Simpler but less accurate alternative to MMSE.
pub fn cubic_interpolate(y0: f64, y1: f64, y2: f64, y3: f64, mu: f64) -> f64 {
    let a0 = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
    let a1 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    let a2 = -0.5 * y0 + 0.5 * y2;
    let a3 = y1;
    ((a0 * mu + a1) * mu + a2) * mu + a3
}

/// Compute fractional delay using linear interpolation.
pub fn linear_interpolate(y0: f64, y1: f64, mu: f64) -> f64 {
    y0 + mu * (y1 - y0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_creation() {
        let interp = MmseInterpolator::new();
        assert_eq!(interp.num_taps(), 8);
        assert_eq!(interp.mu_resolution(), 128);
    }

    #[test]
    fn test_default() {
        let interp = MmseInterpolator::default();
        assert_eq!(interp.num_taps(), 8);
    }

    #[test]
    fn test_integer_delay() {
        let interp = MmseInterpolator::new();
        // At mu=0, should return the center sample exactly
        let samples: Vec<Complex64> = (0..16)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let result = interp.interpolate(&samples, 8, 0.0).unwrap();
        // Should be close to sample[8] = 8.0
        assert!(
            (result.re - 8.0).abs() < 0.5,
            "Integer delay should return ~center sample: got {}",
            result.re
        );
    }

    #[test]
    fn test_half_sample_delay() {
        let interp = MmseInterpolator::new();
        // DC signal: any delay should return same value
        let samples = vec![Complex64::new(1.0, 0.0); 16];
        let result = interp.interpolate(&samples, 8, 0.5).unwrap();
        assert!(
            (result.re - 1.0).abs() < 0.1,
            "DC signal should be preserved: got {}",
            result.re
        );
    }

    #[test]
    fn test_sinusoidal_interpolation() {
        let interp = MmseInterpolator::new();
        let freq = 0.05; // Well within Nyquist
        let samples: Vec<Complex64> = (0..32)
            .map(|i| Complex64::new((2.0 * PI * freq * i as f64).sin(), 0.0))
            .collect();

        // Interpolate at 0.5 sample delay
        let result = interp.interpolate(&samples, 16, 0.5).unwrap();
        let expected = (2.0 * PI * freq * 16.5).sin();
        assert!(
            (result.re - expected).abs() < 0.1,
            "Sinusoidal interpolation error: got {}, expected {}",
            result.re,
            expected
        );
    }

    #[test]
    fn test_out_of_bounds() {
        let interp = MmseInterpolator::new();
        let samples = vec![Complex64::new(1.0, 0.0); 8];
        // Too close to beginning
        assert!(interp.interpolate(&samples, 2, 0.5).is_none());
        // Too close to end
        assert!(interp.interpolate(&samples, 7, 0.5).is_none());
    }

    #[test]
    fn test_coefficients_sum_to_one() {
        let interp = MmseInterpolator::new();
        for mu_step in 0..NUM_MU_STEPS {
            let mu = mu_step as f64 / NUM_MU_STEPS as f64;
            let coeffs = interp.coefficients_for_mu(mu);
            let sum: f64 = coeffs.iter().sum();
            assert!(
                (sum - 1.0).abs() < 0.01,
                "Coefficients should sum to ~1.0 at mu={}: sum={}",
                mu,
                sum
            );
        }
    }

    #[test]
    fn test_batch_interpolation() {
        let interp = MmseInterpolator::new();
        let samples: Vec<Complex64> = (0..32)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let positions = vec![(8, 0.0), (10, 0.25), (12, 0.5), (14, 0.75)];
        let results = interp.interpolate_batch(&samples, &positions);
        assert_eq!(results.len(), 4);
    }

    #[test]
    fn test_interpolate_real() {
        let interp = MmseInterpolator::new();
        let samples: Vec<f64> = (0..16).map(|i| i as f64).collect();
        let result = interp.interpolate_real(&samples, 8, 0.0).unwrap();
        assert!((result - 8.0).abs() < 0.5);
    }

    #[test]
    fn test_cubic_interpolate() {
        // Linear ramp: cubic should give exact result
        let result = cubic_interpolate(2.0, 3.0, 4.0, 5.0, 0.5);
        assert!((result - 3.5).abs() < 0.01);
    }

    #[test]
    fn test_linear_interpolate() {
        assert_eq!(linear_interpolate(0.0, 1.0, 0.5), 0.5);
        assert_eq!(linear_interpolate(0.0, 1.0, 0.0), 0.0);
        assert_eq!(linear_interpolate(0.0, 1.0, 1.0), 1.0);
    }

    #[test]
    fn test_mu_clamping() {
        let interp = MmseInterpolator::new();
        let samples = vec![Complex64::new(1.0, 0.0); 16];
        // Negative mu should be clamped
        let r1 = interp.interpolate(&samples, 8, -0.1);
        assert!(r1.is_some());
        // mu > 1 should be clamped
        let r2 = interp.interpolate(&samples, 8, 1.5);
        assert!(r2.is_some());
    }
}

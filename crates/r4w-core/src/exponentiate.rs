//! Exponentiate — Raise samples to arbitrary power
//!
//! Computes y[n] = x[n]^p for real or complex samples. Supports
//! integer and fractional exponents. Used for signal squaring
//! (carrier recovery), power-law compression, and nonlinear
//! transformations.
//! GNU Radio equivalent: `exponentiate_const_cci/ccf`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::exponentiate::{exponentiate, exponentiate_complex};
//! use num_complex::Complex64;
//!
//! let input = vec![2.0, 3.0, 4.0];
//! let squared = exponentiate(&input, 2.0);
//! assert!((squared[0] - 4.0).abs() < 1e-10);
//! assert!((squared[1] - 9.0).abs() < 1e-10);
//!
//! let c = vec![Complex64::new(0.0, 1.0)]; // j
//! let c2 = exponentiate_complex(&c, 2); // j^2 = -1
//! assert!((c2[0].re - (-1.0)).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Raise each real sample to a power: y[n] = x[n]^p.
pub fn exponentiate(input: &[f64], power: f64) -> Vec<f64> {
    input.iter().map(|&x| x.powf(power)).collect()
}

/// Raise each real sample to an integer power (faster than powf).
pub fn exponentiate_int(input: &[f64], power: i32) -> Vec<f64> {
    input.iter().map(|&x| x.powi(power)).collect()
}

/// Raise each complex sample to an integer power: y[n] = x[n]^p.
///
/// Uses repeated multiplication for small positive exponents,
/// falls back to polar form for larger or negative exponents.
pub fn exponentiate_complex(input: &[Complex64], power: i32) -> Vec<Complex64> {
    input.iter().map(|&x| complex_powi(x, power)).collect()
}

/// Raise each complex sample to a real power using polar form.
///
/// y = |x|^p * exp(j * p * angle(x))
pub fn exponentiate_complex_real(input: &[Complex64], power: f64) -> Vec<Complex64> {
    input
        .iter()
        .map(|&x| {
            let r = x.norm();
            let theta = x.arg();
            let new_r = r.powf(power);
            let new_theta = theta * power;
            Complex64::new(new_r * new_theta.cos(), new_r * new_theta.sin())
        })
        .collect()
}

/// In-place exponentiation for real samples.
pub fn exponentiate_inplace(data: &mut [f64], power: f64) {
    for x in data.iter_mut() {
        *x = x.powf(power);
    }
}

/// In-place integer exponentiation for real samples.
pub fn exponentiate_int_inplace(data: &mut [f64], power: i32) {
    for x in data.iter_mut() {
        *x = x.powi(power);
    }
}

/// Square each sample: y[n] = x[n]^2.
///
/// Common operation for carrier recovery (e.g., squaring loop for BPSK).
pub fn square(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x * x).collect()
}

/// Square each complex sample: y[n] = x[n]^2.
pub fn square_complex(input: &[Complex64]) -> Vec<Complex64> {
    input.iter().map(|&x| x * x).collect()
}

/// Compute the reciprocal: y[n] = 1 / x[n].
///
/// Clamps near-zero values to avoid infinity.
pub fn reciprocal(input: &[f64], min_abs: f64) -> Vec<f64> {
    let min_abs = min_abs.abs().max(1e-30);
    input
        .iter()
        .map(|&x| {
            if x.abs() < min_abs {
                1.0 / x.signum() * min_abs.recip()
            } else {
                1.0 / x
            }
        })
        .collect()
}

/// Compute the square root: y[n] = sqrt(x[n]).
///
/// Negative inputs yield NaN (use abs first if needed).
pub fn sqrt_real(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x.sqrt()).collect()
}

/// Compute the complex square root.
pub fn sqrt_complex(input: &[Complex64]) -> Vec<Complex64> {
    input.iter().map(|&x| x.sqrt()).collect()
}

/// Integer power of a complex number via repeated squaring.
fn complex_powi(x: Complex64, n: i32) -> Complex64 {
    if n == 0 {
        return Complex64::new(1.0, 0.0);
    }
    if n < 0 {
        let inv = complex_powi(x, -n);
        return Complex64::new(1.0, 0.0) / inv;
    }
    // Exponentiation by squaring
    let mut result = Complex64::new(1.0, 0.0);
    let mut base = x;
    let mut exp = n as u32;
    while exp > 0 {
        if exp & 1 == 1 {
            result *= base;
        }
        base *= base;
        exp >>= 1;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_square() {
        let input = vec![1.0, 2.0, 3.0, -4.0];
        let out = exponentiate(&input, 2.0);
        assert!((out[0] - 1.0).abs() < 1e-10);
        assert!((out[1] - 4.0).abs() < 1e-10);
        assert!((out[2] - 9.0).abs() < 1e-10);
        assert!((out[3] - 16.0).abs() < 1e-10);
    }

    #[test]
    fn test_cube() {
        let out = exponentiate_int(&[2.0, 3.0], 3);
        assert!((out[0] - 8.0).abs() < 1e-10);
        assert!((out[1] - 27.0).abs() < 1e-10);
    }

    #[test]
    fn test_sqrt() {
        let out = exponentiate(&[4.0, 9.0, 16.0], 0.5);
        assert!((out[0] - 2.0).abs() < 1e-10);
        assert!((out[1] - 3.0).abs() < 1e-10);
        assert!((out[2] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_power_zero() {
        let out = exponentiate(&[5.0, -3.0, 100.0], 0.0);
        for &v in &out {
            assert!((v - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_complex_square() {
        // (1+j)^2 = 2j
        let input = vec![Complex64::new(1.0, 1.0)];
        let out = exponentiate_complex(&input, 2);
        assert!((out[0].re - 0.0).abs() < 1e-10);
        assert!((out[0].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_cube() {
        // j^3 = -j
        let input = vec![Complex64::new(0.0, 1.0)];
        let out = exponentiate_complex(&input, 3);
        assert!((out[0].re - 0.0).abs() < 1e-10);
        assert!((out[0].im - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_complex_negative_power() {
        // (2+0j)^-1 = 0.5
        let input = vec![Complex64::new(2.0, 0.0)];
        let out = exponentiate_complex(&input, -1);
        assert!((out[0].re - 0.5).abs() < 1e-10);
        assert!(out[0].im.abs() < 1e-10);
    }

    #[test]
    fn test_complex_real_power() {
        // |1+j|^0.5 * exp(j * 0.5 * pi/4) — fractional power
        let input = vec![Complex64::new(1.0, 1.0)];
        let out = exponentiate_complex_real(&input, 2.0);
        // (1+j)^2 = 2j
        assert!((out[0].re - 0.0).abs() < 1e-10);
        assert!((out[0].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_inplace() {
        let mut data = vec![2.0, 3.0, 4.0];
        exponentiate_inplace(&mut data, 2.0);
        assert!((data[0] - 4.0).abs() < 1e-10);
        assert!((data[1] - 9.0).abs() < 1e-10);
        assert!((data[2] - 16.0).abs() < 1e-10);
    }

    #[test]
    fn test_square_fn() {
        let out = square(&[3.0, -5.0]);
        assert!((out[0] - 9.0).abs() < 1e-10);
        assert!((out[1] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_square_complex_fn() {
        let out = square_complex(&[Complex64::new(0.0, 1.0)]);
        assert!((out[0].re - (-1.0)).abs() < 1e-10);
        assert!(out[0].im.abs() < 1e-10);
    }

    #[test]
    fn test_reciprocal() {
        let out = reciprocal(&[2.0, 4.0, 0.5], 1e-10);
        assert!((out[0] - 0.5).abs() < 1e-10);
        assert!((out[1] - 0.25).abs() < 1e-10);
        assert!((out[2] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_sqrt_real_fn() {
        let out = sqrt_real(&[4.0, 25.0]);
        assert!((out[0] - 2.0).abs() < 1e-10);
        assert!((out[1] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_sqrt_complex_fn() {
        // sqrt(j) = (1+j)/sqrt(2)
        let out = sqrt_complex(&[Complex64::new(0.0, 1.0)]);
        let expected_re = 1.0 / 2.0f64.sqrt();
        let expected_im = 1.0 / 2.0f64.sqrt();
        assert!((out[0].re - expected_re).abs() < 1e-10);
        assert!((out[0].im - expected_im).abs() < 1e-10);
    }

    #[test]
    fn test_empty() {
        assert!(exponentiate(&[], 2.0).is_empty());
        assert!(exponentiate_complex(&[], 2).is_empty());
        assert!(square(&[]).is_empty());
    }
}

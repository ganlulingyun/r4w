//! Transcendental and Math Operations on Streams
//!
//! Element-wise mathematical operations for signal processing.
//! Includes logarithmic, exponential, absolute value, and clamping operations.
//!
//! ## Blocks
//!
//! - **Log10**: `y[n] = 10 * log10(|x[n]|^2)` (power in dB)
//! - **Ln**: `y[n] = ln(x[n])` (natural logarithm)
//! - **Exp**: `y[n] = exp(x[n])`
//! - **Abs**: `y[n] = |x[n]|` (magnitude for complex, absolute value for real)
//! - **AbsSq**: `y[n] = |x[n]|^2` (power)
//! - **Clamp**: `y[n] = clamp(x[n], min, max)`
//! - **Max/Min**: Element-wise max/min of two streams
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::transcendental::{power_db, abs_complex};
//! use num_complex::Complex64;
//!
//! let signal = vec![Complex64::new(3.0, 4.0)]; // magnitude = 5
//! let mag = abs_complex(&signal);
//! assert!((mag[0] - 5.0).abs() < 1e-10);
//!
//! let db = power_db(&signal);
//! // 10*log10(25) ≈ 13.98 dB
//! assert!((db[0] - 13.979).abs() < 0.01);
//! ```

use num_complex::Complex64;

// ── Complex operations ─────────────────────────────────────────────────

/// Compute magnitude: |x[n]|.
pub fn abs_complex(input: &[Complex64]) -> Vec<f64> {
    input.iter().map(|z| z.norm()).collect()
}

/// Compute squared magnitude: |x[n]|^2.
pub fn abs_sq_complex(input: &[Complex64]) -> Vec<f64> {
    input.iter().map(|z| z.norm_sqr()).collect()
}

/// Compute power in dB: 10 * log10(|x[n]|^2).
pub fn power_db(input: &[Complex64]) -> Vec<f64> {
    input
        .iter()
        .map(|z| {
            let power = z.norm_sqr();
            if power < 1e-30 {
                -300.0 // floor
            } else {
                10.0 * power.log10()
            }
        })
        .collect()
}

/// Complex exponential: exp(x[n]) where x is complex.
pub fn exp_complex(input: &[Complex64]) -> Vec<Complex64> {
    input
        .iter()
        .map(|z| {
            let r = z.re.exp();
            Complex64::new(r * z.im.cos(), r * z.im.sin())
        })
        .collect()
}

/// Complex natural logarithm: ln(x[n]).
pub fn ln_complex(input: &[Complex64]) -> Vec<Complex64> {
    input
        .iter()
        .map(|z| {
            let r = z.norm();
            if r < 1e-30 {
                Complex64::new(-300.0, 0.0)
            } else {
                Complex64::new(r.ln(), z.arg())
            }
        })
        .collect()
}

// ── Real operations ────────────────────────────────────────────────────

/// Absolute value: |x[n]|.
pub fn abs_real(input: &[f64]) -> Vec<f64> {
    input.iter().map(|x| x.abs()).collect()
}

/// Natural logarithm: ln(x[n]) (returns -inf for x ≤ 0).
pub fn ln_real(input: &[f64]) -> Vec<f64> {
    input
        .iter()
        .map(|&x| {
            if x <= 0.0 {
                -300.0
            } else {
                x.ln()
            }
        })
        .collect()
}

/// Log base 10: log10(x[n]).
pub fn log10_real(input: &[f64]) -> Vec<f64> {
    input
        .iter()
        .map(|&x| {
            if x <= 0.0 {
                -300.0
            } else {
                x.log10()
            }
        })
        .collect()
}

/// Exponential: exp(x[n]).
pub fn exp_real(input: &[f64]) -> Vec<f64> {
    input.iter().map(|x| x.exp()).collect()
}

/// Clamp values to [min, max].
pub fn clamp_real(input: &[f64], min: f64, max: f64) -> Vec<f64> {
    input.iter().map(|x| x.clamp(min, max)).collect()
}

/// Element-wise maximum of two streams.
pub fn max_real(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x.max(y)).collect()
}

/// Element-wise minimum of two streams.
pub fn min_real(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x.min(y)).collect()
}

/// Convert real to dB: 10 * log10(|x[n]|).
pub fn to_db(input: &[f64]) -> Vec<f64> {
    input
        .iter()
        .map(|&x| {
            let v = x.abs();
            if v < 1e-30 {
                -300.0
            } else {
                10.0 * v.log10()
            }
        })
        .collect()
}

/// Convert dB to linear: 10^(x[n]/10).
pub fn from_db(input: &[f64]) -> Vec<f64> {
    input.iter().map(|x| 10.0_f64.powf(x / 10.0)).collect()
}

// ── Normalize operations ───────────────────────────────────────────────

/// Normalize complex signal to unit average power.
pub fn normalize_power(input: &[Complex64]) -> Vec<Complex64> {
    if input.is_empty() {
        return Vec::new();
    }
    let avg_power: f64 = input.iter().map(|z| z.norm_sqr()).sum::<f64>() / input.len() as f64;
    if avg_power < 1e-30 {
        return input.to_vec();
    }
    let scale = 1.0 / avg_power.sqrt();
    input.iter().map(|z| z * scale).collect()
}

/// Normalize real signal to unit RMS.
pub fn normalize_rms(input: &[f64]) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }
    let rms: f64 = (input.iter().map(|x| x * x).sum::<f64>() / input.len() as f64).sqrt();
    if rms < 1e-30 {
        return input.to_vec();
    }
    input.iter().map(|x| x / rms).collect()
}

/// Normalize to peak amplitude.
pub fn normalize_peak(input: &[f64]) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }
    let peak = input.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    if peak < 1e-30 {
        return input.to_vec();
    }
    input.iter().map(|x| x / peak).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::E;

    #[test]
    fn test_abs_complex() {
        let input = vec![Complex64::new(3.0, 4.0), Complex64::new(0.0, 1.0)];
        let result = abs_complex(&input);
        assert!((result[0] - 5.0).abs() < 1e-10);
        assert!((result[1] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_abs_sq_complex() {
        let input = vec![Complex64::new(3.0, 4.0)];
        let result = abs_sq_complex(&input);
        assert!((result[0] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_power_db() {
        let input = vec![Complex64::new(1.0, 0.0)]; // |x|^2 = 1.0
        let result = power_db(&input);
        assert!(result[0].abs() < 1e-10); // 10*log10(1) = 0 dB
    }

    #[test]
    fn test_power_db_floor() {
        let input = vec![Complex64::new(0.0, 0.0)];
        let result = power_db(&input);
        assert_eq!(result[0], -300.0);
    }

    #[test]
    fn test_exp_complex() {
        let input = vec![Complex64::new(0.0, 0.0)]; // exp(0) = 1
        let result = exp_complex(&input);
        assert!((result[0].re - 1.0).abs() < 1e-10);
        assert!(result[0].im.abs() < 1e-10);
    }

    #[test]
    fn test_ln_complex() {
        let input = vec![Complex64::new(E, 0.0)]; // ln(e) = 1
        let result = ln_complex(&input);
        assert!((result[0].re - 1.0).abs() < 1e-10);
        assert!(result[0].im.abs() < 1e-10);
    }

    #[test]
    fn test_abs_real() {
        let result = abs_real(&[-3.0, 0.0, 5.0]);
        assert_eq!(result, vec![3.0, 0.0, 5.0]);
    }

    #[test]
    fn test_ln_real() {
        let result = ln_real(&[E, 1.0]);
        assert!((result[0] - 1.0).abs() < 1e-10);
        assert!(result[1].abs() < 1e-10);
    }

    #[test]
    fn test_ln_real_negative() {
        let result = ln_real(&[-1.0, 0.0]);
        assert_eq!(result[0], -300.0);
        assert_eq!(result[1], -300.0);
    }

    #[test]
    fn test_log10_real() {
        let result = log10_real(&[100.0, 10.0, 1.0]);
        assert!((result[0] - 2.0).abs() < 1e-10);
        assert!((result[1] - 1.0).abs() < 1e-10);
        assert!(result[2].abs() < 1e-10);
    }

    #[test]
    fn test_exp_real() {
        let result = exp_real(&[0.0, 1.0]);
        assert!((result[0] - 1.0).abs() < 1e-10);
        assert!((result[1] - E).abs() < 1e-10);
    }

    #[test]
    fn test_clamp_real() {
        let result = clamp_real(&[-5.0, 0.0, 10.0], -1.0, 1.0);
        assert_eq!(result, vec![-1.0, 0.0, 1.0]);
    }

    #[test]
    fn test_max_real() {
        let result = max_real(&[1.0, 5.0, 3.0], &[2.0, 4.0, 6.0]);
        assert_eq!(result, vec![2.0, 5.0, 6.0]);
    }

    #[test]
    fn test_min_real() {
        let result = min_real(&[1.0, 5.0, 3.0], &[2.0, 4.0, 6.0]);
        assert_eq!(result, vec![1.0, 4.0, 3.0]);
    }

    #[test]
    fn test_to_db_from_db_roundtrip() {
        let linear = vec![0.001, 0.01, 0.1, 1.0, 10.0, 100.0];
        let db = to_db(&linear);
        let recovered = from_db(&db);
        for (a, b) in linear.iter().zip(recovered.iter()) {
            assert!((a - b).abs() < 1e-6, "{} != {}", a, b);
        }
    }

    #[test]
    fn test_normalize_power() {
        let input = vec![
            Complex64::new(3.0, 0.0),
            Complex64::new(4.0, 0.0),
        ];
        let normalized = normalize_power(&input);
        let avg_power: f64 = normalized.iter().map(|z| z.norm_sqr()).sum::<f64>() / normalized.len() as f64;
        assert!((avg_power - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_rms() {
        let input = vec![1.0, 2.0, 3.0, 4.0];
        let normalized = normalize_rms(&input);
        let rms: f64 = (normalized.iter().map(|x| x * x).sum::<f64>() / normalized.len() as f64).sqrt();
        assert!((rms - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_peak() {
        let input = vec![-2.0, 1.0, 5.0, -3.0];
        let normalized = normalize_peak(&input);
        let peak = normalized.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        assert!((peak - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_empty() {
        assert!(normalize_power(&[]).is_empty());
        assert!(normalize_rms(&[]).is_empty());
        assert!(normalize_peak(&[]).is_empty());
    }

    #[test]
    fn test_normalize_zero_signal() {
        let zeros = vec![Complex64::new(0.0, 0.0); 5];
        let result = normalize_power(&zeros);
        assert_eq!(result.len(), 5);
        // Should return zeros unchanged (not NaN)
        for z in &result {
            assert!(!z.re.is_nan());
        }
    }
}

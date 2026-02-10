//! # Absolute Value / Magnitude Block
//!
//! Computes element-wise absolute value for real streams and magnitude
//! for complex streams. GNU Radio equivalent of `gr::blocks::abs`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::abs_blk::{abs_real, abs_complex};
//!
//! let real = vec![-3.0, 4.0, -1.5, 0.0];
//! assert_eq!(abs_real(&real), vec![3.0, 4.0, 1.5, 0.0]);
//!
//! let complex = vec![(3.0, 4.0), (0.0, -1.0)];
//! let mags = abs_complex(&complex);
//! assert!((mags[0] - 5.0).abs() < 1e-10);
//! ```

/// Absolute value of real samples.
pub fn abs_real(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x.abs()).collect()
}

/// Absolute value of f32 real samples.
pub fn abs_real_f32(input: &[f32]) -> Vec<f32> {
    input.iter().map(|&x| x.abs()).collect()
}

/// Magnitude of complex samples: sqrt(re² + im²).
pub fn abs_complex(input: &[(f64, f64)]) -> Vec<f64> {
    input
        .iter()
        .map(|&(re, im)| (re * re + im * im).sqrt())
        .collect()
}

/// Magnitude squared of complex samples: re² + im² (avoids sqrt).
pub fn abs_complex_sq(input: &[(f64, f64)]) -> Vec<f64> {
    input
        .iter()
        .map(|&(re, im)| re * re + im * im)
        .collect()
}

/// In-place absolute value for real samples.
pub fn abs_real_inplace(samples: &mut [f64]) {
    for s in samples.iter_mut() {
        *s = s.abs();
    }
}

/// Absolute value with sign preservation (returns sign * |x|, i.e., x itself).
/// Useful for debugging/testing.
pub fn sign(input: &[f64]) -> Vec<f64> {
    input
        .iter()
        .map(|&x| {
            if x > 0.0 {
                1.0
            } else if x < 0.0 {
                -1.0
            } else {
                0.0
            }
        })
        .collect()
}

/// Clip (clamp) real samples to [-limit, +limit].
pub fn clip(input: &[f64], limit: f64) -> Vec<f64> {
    let lim = limit.abs();
    input.iter().map(|&x| x.clamp(-lim, lim)).collect()
}

/// Clip complex samples by magnitude.
pub fn clip_complex(input: &[(f64, f64)], max_mag: f64) -> Vec<(f64, f64)> {
    input
        .iter()
        .map(|&(re, im)| {
            let mag = (re * re + im * im).sqrt();
            if mag > max_mag && mag > 0.0 {
                let scale = max_mag / mag;
                (re * scale, im * scale)
            } else {
                (re, im)
            }
        })
        .collect()
}

/// Rectify: half-wave (keep positive, zero negative) or full-wave (abs).
pub fn half_wave_rectify(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| if x > 0.0 { x } else { 0.0 }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_abs_real() {
        let input = vec![-3.0, 4.0, -1.5, 0.0, 2.0];
        assert_eq!(abs_real(&input), vec![3.0, 4.0, 1.5, 0.0, 2.0]);
    }

    #[test]
    fn test_abs_real_f32() {
        let input = vec![-1.0f32, 2.0, -3.0];
        assert_eq!(abs_real_f32(&input), vec![1.0f32, 2.0, 3.0]);
    }

    #[test]
    fn test_abs_complex() {
        let input = vec![(3.0, 4.0), (0.0, -1.0), (1.0, 0.0)];
        let mags = abs_complex(&input);
        assert!((mags[0] - 5.0).abs() < 1e-10);
        assert!((mags[1] - 1.0).abs() < 1e-10);
        assert!((mags[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_abs_complex_sq() {
        let input = vec![(3.0, 4.0)];
        let mag_sq = abs_complex_sq(&input);
        assert!((mag_sq[0] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_abs_inplace() {
        let mut data = vec![-1.0, 2.0, -3.0];
        abs_real_inplace(&mut data);
        assert_eq!(data, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_sign() {
        let input = vec![-5.0, 0.0, 3.0, -0.1];
        let s = sign(&input);
        assert_eq!(s, vec![-1.0, 0.0, 1.0, -1.0]);
    }

    #[test]
    fn test_clip() {
        let input = vec![-5.0, 0.5, 3.0, -0.1];
        let clipped = clip(&input, 1.0);
        assert_eq!(clipped, vec![-1.0, 0.5, 1.0, -0.1]);
    }

    #[test]
    fn test_clip_complex() {
        let input = vec![(3.0, 4.0), (0.1, 0.1)]; // mag=5.0, mag≈0.14
        let clipped = clip_complex(&input, 2.0);
        let mag0 = (clipped[0].0.powi(2) + clipped[0].1.powi(2)).sqrt();
        assert!((mag0 - 2.0).abs() < 1e-10, "Should be clipped to 2.0, got {}", mag0);
        // Small one should be unchanged.
        assert!((clipped[1].0 - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_half_wave_rectify() {
        let input = vec![-2.0, 1.0, -0.5, 3.0, 0.0];
        assert_eq!(half_wave_rectify(&input), vec![0.0, 1.0, 0.0, 3.0, 0.0]);
    }

    #[test]
    fn test_empty() {
        assert!(abs_real(&[]).is_empty());
        assert!(abs_complex(&[]).is_empty());
    }
}

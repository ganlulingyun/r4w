//! # Interpolating Resampler
//!
//! Simple interpolation-based resampler supporting linear, cubic,
//! and windowed-sinc methods. For fractional rate conversion without
//! the complexity of polyphase FIR (see `sample_rate_converter` for that).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::interpolating_resampler::{resample_linear, resample_ratio};
//!
//! let input = vec![0.0, 1.0, 0.0, -1.0, 0.0];
//! let upsampled = resample_linear(&input, 2.0);
//! assert!(upsampled.len() >= 9);
//!
//! // Resample from 44100 to 48000
//! let resampled = resample_ratio(&input, 44100.0, 48000.0);
//! ```

/// Resample using linear interpolation.
pub fn resample_linear(input: &[f64], ratio: f64) -> Vec<f64> {
    if input.is_empty() || ratio <= 0.0 {
        return Vec::new();
    }

    let out_len = ((input.len() as f64 - 1.0) * ratio).ceil() as usize + 1;
    let mut output = Vec::with_capacity(out_len);

    let mut t = 0.0;
    let step = 1.0 / ratio;
    let last = (input.len() - 1) as f64;

    while t <= last + 1e-10 {
        let idx = (t as usize).min(input.len() - 2);
        let frac = t - idx as f64;
        let val = input[idx] * (1.0 - frac) + input[idx + 1] * frac;
        output.push(val);
        t += step;
    }

    output
}

/// Resample using cubic (Hermite) interpolation.
pub fn resample_cubic(input: &[f64], ratio: f64) -> Vec<f64> {
    if input.len() < 4 || ratio <= 0.0 {
        return resample_linear(input, ratio);
    }

    let out_len = ((input.len() as f64 - 1.0) * ratio).ceil() as usize + 1;
    let mut output = Vec::with_capacity(out_len);

    let mut t = 0.0;
    let step = 1.0 / ratio;

    while t < (input.len() - 1) as f64 {
        let idx = t as usize;
        let frac = t - idx as f64;

        // Hermite interpolation with 4 points.
        let y0 = if idx > 0 { input[idx - 1] } else { input[0] };
        let y1 = input[idx];
        let y2 = input[(idx + 1).min(input.len() - 1)];
        let y3 = input[(idx + 2).min(input.len() - 1)];

        let val = hermite(frac, y0, y1, y2, y3);
        output.push(val);
        t += step;
    }

    output
}

fn hermite(t: f64, y0: f64, y1: f64, y2: f64, y3: f64) -> f64 {
    let c0 = y1;
    let c1 = 0.5 * (y2 - y0);
    let c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    let c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
    ((c3 * t + c2) * t + c1) * t + c0
}

/// Resample given input and output sample rates.
pub fn resample_ratio(input: &[f64], in_rate: f64, out_rate: f64) -> Vec<f64> {
    let ratio = out_rate / in_rate;
    resample_linear(input, ratio)
}

/// Resample complex IQ samples using linear interpolation.
pub fn resample_complex_linear(input: &[(f64, f64)], ratio: f64) -> Vec<(f64, f64)> {
    if input.is_empty() || ratio <= 0.0 {
        return Vec::new();
    }

    let mut output = Vec::new();
    let mut t = 0.0;
    let step = 1.0 / ratio;

    while t < (input.len() - 1) as f64 {
        let idx = t as usize;
        let frac = t - idx as f64;
        let re = input[idx].0 * (1.0 - frac) + input[idx + 1].0 * frac;
        let im = input[idx].1 * (1.0 - frac) + input[idx + 1].1 * frac;
        output.push((re, im));
        t += step;
    }

    output
}

/// Zero-order hold (nearest-neighbor) resampling.
pub fn resample_zoh(input: &[f64], ratio: f64) -> Vec<f64> {
    if input.is_empty() || ratio <= 0.0 {
        return Vec::new();
    }

    let mut output = Vec::new();
    let mut t = 0.0;
    let step = 1.0 / ratio;

    while t < input.len() as f64 {
        let idx = (t as usize).min(input.len() - 1);
        output.push(input[idx]);
        t += step;
    }

    output
}

/// Simple integer upsample by inserting zeros.
pub fn upsample_zeros(input: &[f64], factor: usize) -> Vec<f64> {
    let mut output = Vec::with_capacity(input.len() * factor);
    for &sample in input {
        output.push(sample);
        for _ in 1..factor {
            output.push(0.0);
        }
    }
    output
}

/// Simple integer downsample by keeping every Nth sample.
pub fn downsample(input: &[f64], factor: usize) -> Vec<f64> {
    if factor == 0 {
        return Vec::new();
    }
    input.iter().step_by(factor).copied().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_linear_upsample_2x() {
        let input = vec![0.0, 1.0, 0.0];
        let output = resample_linear(&input, 2.0);
        // Should produce samples at t=0, 0.5, 1.0, 1.5, 2.0
        assert!(output.len() >= 4);
        assert!((output[0] - 0.0).abs() < 1e-10);
        assert!((output[1] - 0.5).abs() < 1e-10);
        assert!((output[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_downsample_2x() {
        let input = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let output = resample_linear(&input, 0.5);
        assert!(output.len() >= 2);
    }

    #[test]
    fn test_cubic_smooth() {
        let input = vec![0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0];
        let output = resample_cubic(&input, 2.0);
        assert!(output.len() > input.len());
    }

    #[test]
    fn test_ratio_upsample() {
        let input = vec![1.0; 100];
        let output = resample_ratio(&input, 44100.0, 48000.0);
        // 48000/44100 ≈ 1.088, so output should be ~108 samples
        assert!(output.len() > 100);
        // Constant input → constant output.
        for &v in &output {
            assert!((v - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_complex_linear() {
        let input = vec![(0.0, 0.0), (1.0, 1.0), (0.0, 0.0)];
        let output = resample_complex_linear(&input, 2.0);
        assert!(output.len() >= 4);
        assert!((output[0].0 - 0.0).abs() < 1e-10);
        assert!((output[1].0 - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_zoh() {
        let input = vec![1.0, 2.0, 3.0];
        let output = resample_zoh(&input, 2.0);
        // ZOH: repeat each sample.
        assert_eq!(output[0], 1.0);
        assert_eq!(output[1], 1.0);
        assert_eq!(output[2], 2.0);
    }

    #[test]
    fn test_upsample_zeros() {
        let input = vec![1.0, 2.0, 3.0];
        let output = upsample_zeros(&input, 3);
        assert_eq!(output, vec![1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 3.0, 0.0, 0.0]);
    }

    #[test]
    fn test_downsample() {
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        assert_eq!(downsample(&input, 2), vec![1.0, 3.0, 5.0]);
        assert_eq!(downsample(&input, 3), vec![1.0, 4.0]);
    }

    #[test]
    fn test_empty() {
        assert!(resample_linear(&[], 2.0).is_empty());
        assert!(resample_cubic(&[], 2.0).is_empty());
        assert!(resample_zoh(&[], 2.0).is_empty());
    }

    #[test]
    fn test_identity() {
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let output = resample_linear(&input, 1.0);
        assert_eq!(output.len(), input.len());
        for (i, o) in input.iter().zip(output.iter()) {
            assert!((i - o).abs() < 1e-10);
        }
    }
}

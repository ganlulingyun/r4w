//! # Complex Normalize
//!
//! Normalizes complex samples to unit magnitude while preserving phase.
//! Useful for hard-decision demodulation, constellation normalization,
//! and PSK signal conditioning. Includes batch, streaming, and
//! power-normalized variants.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::complex_normalize::{normalize_complex, normalize_power};
//!
//! let samples = vec![(3.0, 4.0), (0.0, 2.0), (1.0, 0.0)];
//! let normed = normalize_complex(&samples);
//! // Each sample now has magnitude ~1.0.
//! for &(re, im) in &normed {
//!     let mag = (re * re + im * im).sqrt();
//!     assert!((mag - 1.0).abs() < 1e-10);
//! }
//! ```

/// Normalize complex samples to unit magnitude.
pub fn normalize_complex(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    input
        .iter()
        .map(|&(re, im)| {
            let mag = (re * re + im * im).sqrt();
            if mag > 1e-30 {
                (re / mag, im / mag)
            } else {
                (0.0, 0.0)
            }
        })
        .collect()
}

/// Normalize to target magnitude.
pub fn normalize_to_magnitude(input: &[(f64, f64)], target: f64) -> Vec<(f64, f64)> {
    input
        .iter()
        .map(|&(re, im)| {
            let mag = (re * re + im * im).sqrt();
            if mag > 1e-30 {
                let scale = target / mag;
                (re * scale, im * scale)
            } else {
                (0.0, 0.0)
            }
        })
        .collect()
}

/// Normalize to unit average power (RMS = 1).
pub fn normalize_power(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if input.is_empty() {
        return Vec::new();
    }
    let avg_power: f64 = input.iter().map(|&(re, im)| re * re + im * im).sum::<f64>()
        / input.len() as f64;
    if avg_power < 1e-30 {
        return input.to_vec();
    }
    let scale = 1.0 / avg_power.sqrt();
    input
        .iter()
        .map(|&(re, im)| (re * scale, im * scale))
        .collect()
}

/// Normalize to target average power.
pub fn normalize_to_power(input: &[(f64, f64)], target_power: f64) -> Vec<(f64, f64)> {
    if input.is_empty() {
        return Vec::new();
    }
    let avg_power: f64 = input.iter().map(|&(re, im)| re * re + im * im).sum::<f64>()
        / input.len() as f64;
    if avg_power < 1e-30 {
        return input.to_vec();
    }
    let scale = (target_power / avg_power).sqrt();
    input
        .iter()
        .map(|&(re, im)| (re * scale, im * scale))
        .collect()
}

/// Normalize to peak magnitude = 1.
pub fn normalize_peak(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if input.is_empty() {
        return Vec::new();
    }
    let peak = input
        .iter()
        .map(|&(re, im)| (re * re + im * im).sqrt())
        .fold(0.0_f64, f64::max);
    if peak < 1e-30 {
        return input.to_vec();
    }
    let scale = 1.0 / peak;
    input
        .iter()
        .map(|&(re, im)| (re * scale, im * scale))
        .collect()
}

/// Streaming complex normalizer with exponential averaging.
#[derive(Debug, Clone)]
pub struct StreamingNormalizer {
    target_power: f64,
    alpha: f64,
    avg_power: f64,
    initialized: bool,
}

impl StreamingNormalizer {
    /// Create a new streaming normalizer.
    pub fn new(target_power: f64, alpha: f64) -> Self {
        Self {
            target_power,
            alpha: alpha.clamp(0.001, 1.0),
            avg_power: 0.0,
            initialized: false,
        }
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &(re, im) in input {
            let power = re * re + im * im;
            if !self.initialized {
                self.avg_power = power;
                self.initialized = true;
            } else {
                self.avg_power = self.alpha * power + (1.0 - self.alpha) * self.avg_power;
            }
            let scale = if self.avg_power > 1e-30 {
                (self.target_power / self.avg_power).sqrt()
            } else {
                1.0
            };
            output.push((re * scale, im * scale));
        }
        output
    }

    /// Get current average power estimate.
    pub fn avg_power(&self) -> f64 {
        self.avg_power
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.avg_power = 0.0;
        self.initialized = false;
    }
}

/// Normalize real-valued samples to [-1, 1] range.
pub fn normalize_real(input: &[f64]) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }
    let peak = input.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    if peak < 1e-30 {
        return input.to_vec();
    }
    input.iter().map(|&x| x / peak).collect()
}

/// Normalize real samples to zero mean, unit variance.
pub fn normalize_zscore(input: &[f64]) -> Vec<f64> {
    if input.len() < 2 {
        return input.to_vec();
    }
    let mean: f64 = input.iter().sum::<f64>() / input.len() as f64;
    let variance: f64 =
        input.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (input.len() - 1) as f64;
    let std_dev = variance.sqrt();
    if std_dev < 1e-30 {
        return vec![0.0; input.len()];
    }
    input.iter().map(|&x| (x - mean) / std_dev).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_complex() {
        let input = vec![(3.0, 4.0), (0.0, 5.0), (1.0, 0.0)];
        let output = normalize_complex(&input);
        for &(re, im) in &output {
            let mag = (re * re + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_normalize_to_magnitude() {
        let input = vec![(3.0, 4.0)]; // mag = 5
        let output = normalize_to_magnitude(&input, 10.0);
        let mag = (output[0].0 * output[0].0 + output[0].1 * output[0].1).sqrt();
        assert!((mag - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_power() {
        let input = vec![(2.0, 0.0), (0.0, 2.0), (2.0, 0.0), (0.0, 2.0)];
        let output = normalize_power(&input);
        let avg_pow: f64 = output.iter().map(|&(r, i)| r * r + i * i).sum::<f64>()
            / output.len() as f64;
        assert!((avg_pow - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_to_power() {
        let input = vec![(1.0, 0.0), (0.0, 1.0)];
        let output = normalize_to_power(&input, 4.0);
        let avg_pow: f64 = output.iter().map(|&(r, i)| r * r + i * i).sum::<f64>()
            / output.len() as f64;
        assert!((avg_pow - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_peak() {
        let input = vec![(1.0, 0.0), (3.0, 4.0), (0.0, 2.0)];
        let output = normalize_peak(&input);
        let peak_mag = output
            .iter()
            .map(|&(r, i)| (r * r + i * i).sqrt())
            .fold(0.0_f64, f64::max);
        assert!((peak_mag - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_streaming_normalizer() {
        let mut norm = StreamingNormalizer::new(1.0, 0.1);
        let input = vec![(5.0, 0.0); 100];
        let output = norm.process(&input);
        // After convergence, output power should be ~1.0.
        let last_pow = output.last().map(|&(r, i)| r * r + i * i).unwrap();
        assert!((last_pow - 1.0).abs() < 0.5);
    }

    #[test]
    fn test_normalize_real() {
        let input = vec![-5.0, 3.0, 1.0, -2.0];
        let output = normalize_real(&input);
        assert!((output[0] - (-1.0)).abs() < 1e-10);
        assert!((output[1] - 0.6).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_zscore() {
        let input = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let output = normalize_zscore(&input);
        let mean: f64 = output.iter().sum::<f64>() / output.len() as f64;
        assert!(mean.abs() < 1e-10);
        let var: f64 = output.iter().map(|x| x * x).sum::<f64>() / (output.len() - 1) as f64;
        assert!((var - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_zero_input() {
        let input = vec![(0.0, 0.0)];
        let output = normalize_complex(&input);
        assert_eq!(output[0], (0.0, 0.0)); // No division by zero.
    }

    #[test]
    fn test_empty() {
        assert!(normalize_complex(&[]).is_empty());
        assert!(normalize_power(&[]).is_empty());
        assert!(normalize_peak(&[]).is_empty());
        assert!(normalize_real(&[]).is_empty());
    }
}

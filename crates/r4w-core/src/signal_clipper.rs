//! # Signal Clipper
//!
//! Hard and soft clipping for amplitude limiting. Prevents ADC
//! overflow, reduces PAPR (peak-to-average power ratio), and
//! implements non-linear distortion models. Includes configurable
//! soft-clipping curves (tanh, cubic, polynomial).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::signal_clipper::{hard_clip, soft_clip_tanh};
//!
//! let input = vec![0.5, 1.5, -2.0, 0.3];
//! let clipped = hard_clip(&input, 1.0);
//! assert_eq!(clipped, vec![0.5, 1.0, -1.0, 0.3]);
//!
//! let soft = soft_clip_tanh(&input, 1.0);
//! assert!(soft[1] < 1.0); // Smoothly compressed.
//! ```

/// Hard clip real samples to [-limit, limit].
pub fn hard_clip(input: &[f64], limit: f64) -> Vec<f64> {
    input.iter().map(|&x| x.clamp(-limit, limit)).collect()
}

/// Hard clip complex samples by magnitude.
pub fn hard_clip_complex(input: &[(f64, f64)], limit: f64) -> Vec<(f64, f64)> {
    input
        .iter()
        .map(|&(re, im)| {
            let mag = (re * re + im * im).sqrt();
            if mag > limit && mag > 1e-30 {
                let scale = limit / mag;
                (re * scale, im * scale)
            } else {
                (re, im)
            }
        })
        .collect()
}

/// Soft clip using tanh function.
///
/// output = limit * tanh(input / limit)
pub fn soft_clip_tanh(input: &[f64], limit: f64) -> Vec<f64> {
    let l = limit.max(1e-30);
    input.iter().map(|&x| l * (x / l).tanh()).collect()
}

/// Soft clip complex using tanh on magnitude.
pub fn soft_clip_complex_tanh(input: &[(f64, f64)], limit: f64) -> Vec<(f64, f64)> {
    let l = limit.max(1e-30);
    input
        .iter()
        .map(|&(re, im)| {
            let mag = (re * re + im * im).sqrt();
            if mag < 1e-30 {
                return (re, im);
            }
            let new_mag = l * (mag / l).tanh();
            let scale = new_mag / mag;
            (re * scale, im * scale)
        })
        .collect()
}

/// Cubic soft clipper: y = x - x³/3 for |x| ≤ 1, clamped otherwise.
pub fn soft_clip_cubic(input: &[f64], limit: f64) -> Vec<f64> {
    let l = limit.max(1e-30);
    input
        .iter()
        .map(|&x| {
            let normalized = x / l;
            if normalized.abs() <= 1.0 {
                l * (normalized - normalized * normalized * normalized / 3.0)
            } else {
                l * (2.0 / 3.0) * normalized.signum()
            }
        })
        .collect()
}

/// Polynomial soft clipper with configurable knee.
///
/// Uses smooth transition between linear and clipped regions.
pub fn soft_clip_poly(input: &[f64], limit: f64, knee: f64) -> Vec<f64> {
    let l = limit.max(1e-30);
    let k = knee.clamp(0.01, 0.99);
    let threshold = l * k;

    input
        .iter()
        .map(|&x| {
            let abs_x = x.abs();
            if abs_x <= threshold {
                x
            } else if abs_x <= l {
                // Quadratic compression in the knee region.
                let t = (abs_x - threshold) / (l - threshold);
                let compressed = threshold + (l - threshold) * (2.0 * t - t * t);
                compressed * x.signum()
            } else {
                l * x.signum()
            }
        })
        .collect()
}

/// Stateful clipper that tracks clipping statistics.
#[derive(Debug, Clone)]
pub struct SignalClipper {
    limit: f64,
    mode: ClipMode,
    clipped_samples: u64,
    total_samples: u64,
}

/// Clipping mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClipMode {
    Hard,
    SoftTanh,
    SoftCubic,
}

impl SignalClipper {
    /// Create a new clipper.
    pub fn new(limit: f64, mode: ClipMode) -> Self {
        Self {
            limit: limit.max(1e-30),
            mode,
            clipped_samples: 0,
            total_samples: 0,
        }
    }

    /// Process real samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        self.total_samples += input.len() as u64;
        self.clipped_samples += input.iter().filter(|&&x| x.abs() > self.limit).count() as u64;

        match self.mode {
            ClipMode::Hard => hard_clip(input, self.limit),
            ClipMode::SoftTanh => soft_clip_tanh(input, self.limit),
            ClipMode::SoftCubic => soft_clip_cubic(input, self.limit),
        }
    }

    /// Process complex samples.
    pub fn process_complex(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        self.total_samples += input.len() as u64;
        self.clipped_samples += input
            .iter()
            .filter(|&&(re, im)| (re * re + im * im).sqrt() > self.limit)
            .count() as u64;

        match self.mode {
            ClipMode::Hard => hard_clip_complex(input, self.limit),
            ClipMode::SoftTanh => soft_clip_complex_tanh(input, self.limit),
            ClipMode::SoftCubic => {
                // Use tanh for complex cubic.
                soft_clip_complex_tanh(input, self.limit)
            }
        }
    }

    /// Get clip ratio (fraction of samples that were clipped).
    pub fn clip_ratio(&self) -> f64 {
        if self.total_samples == 0 {
            0.0
        } else {
            self.clipped_samples as f64 / self.total_samples as f64
        }
    }

    /// Get total clipped samples.
    pub fn clipped_samples(&self) -> u64 {
        self.clipped_samples
    }

    /// Set limit.
    pub fn set_limit(&mut self, limit: f64) {
        self.limit = limit.max(1e-30);
    }

    /// Reset statistics.
    pub fn reset_stats(&mut self) {
        self.clipped_samples = 0;
        self.total_samples = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hard_clip() {
        let input = vec![-2.0, -0.5, 0.0, 0.5, 2.0];
        let output = hard_clip(&input, 1.0);
        assert_eq!(output, vec![-1.0, -0.5, 0.0, 0.5, 1.0]);
    }

    #[test]
    fn test_hard_clip_complex() {
        let input = vec![(3.0, 4.0)]; // mag = 5
        let output = hard_clip_complex(&input, 1.0);
        let mag = (output[0].0 * output[0].0 + output[0].1 * output[0].1).sqrt();
        assert!((mag - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_soft_clip_tanh() {
        let input = vec![0.0, 0.5, 1.0, 2.0, 5.0];
        let output = soft_clip_tanh(&input, 1.0);
        // tanh(0) = 0, tanh(0.5) ≈ 0.46, all < 1.0.
        assert!((output[0]).abs() < 1e-10);
        assert!(output[4] < 1.0);
        // Monotonically increasing.
        for i in 1..output.len() {
            assert!(output[i] > output[i - 1]);
        }
    }

    #[test]
    fn test_soft_clip_cubic() {
        let input = vec![0.0, 0.5, 1.0, 2.0];
        let output = soft_clip_cubic(&input, 1.0);
        assert!((output[0]).abs() < 1e-10);
        assert!(output[1] > 0.0);
        assert!(output[3] > 0.0); // Positive.
    }

    #[test]
    fn test_soft_clip_poly() {
        let input = vec![0.0, 0.5, 0.8, 1.0, 2.0];
        let output = soft_clip_poly(&input, 1.0, 0.5);
        assert!((output[0]).abs() < 1e-10);
        assert!((output[1] - 0.5).abs() < 1e-10); // Below knee → linear.
        assert!(output[3] <= 1.0); // At limit.
    }

    #[test]
    fn test_complex_tanh() {
        let input = vec![(3.0, 4.0), (0.0, 0.0)]; // mag=5, mag=0
        let output = soft_clip_complex_tanh(&input, 1.0);
        let mag0 = (output[0].0 * output[0].0 + output[0].1 * output[0].1).sqrt();
        assert!(mag0 < 1.0); // Compressed below limit.
        assert_eq!(output[1], (0.0, 0.0));
    }

    #[test]
    fn test_stateful_clipper() {
        let mut clipper = SignalClipper::new(1.0, ClipMode::Hard);
        let input = vec![0.5, 1.5, 2.0, 0.3, -1.5];
        let output = clipper.process(&input);
        assert_eq!(output, vec![0.5, 1.0, 1.0, 0.3, -1.0]);
        assert_eq!(clipper.clipped_samples(), 3);
        assert!((clipper.clip_ratio() - 0.6).abs() < 1e-10);
    }

    #[test]
    fn test_clipper_complex() {
        let mut clipper = SignalClipper::new(1.0, ClipMode::Hard);
        let input = vec![(3.0, 4.0), (0.5, 0.0)];
        let output = clipper.process_complex(&input);
        let mag0 = (output[0].0.powi(2) + output[0].1.powi(2)).sqrt();
        assert!((mag0 - 1.0).abs() < 1e-10);
        assert_eq!(clipper.clipped_samples(), 1);
    }

    #[test]
    fn test_reset_stats() {
        let mut clipper = SignalClipper::new(1.0, ClipMode::Hard);
        clipper.process(&[2.0, 3.0]);
        assert_eq!(clipper.clipped_samples(), 2);
        clipper.reset_stats();
        assert_eq!(clipper.clipped_samples(), 0);
        assert_eq!(clipper.clip_ratio(), 0.0);
    }

    #[test]
    fn test_empty() {
        assert!(hard_clip(&[], 1.0).is_empty());
        assert!(hard_clip_complex(&[], 1.0).is_empty());
        assert!(soft_clip_tanh(&[], 1.0).is_empty());
    }
}

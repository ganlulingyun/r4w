//! RMS Power Measurement and Normalization
//!
//! Computes the Root-Mean-Square (RMS) power of complex or real signals
//! with configurable averaging. Optionally normalizes output to unit power.
//!
//! ## Modes
//!
//! - **Measure**: Output the RMS power level as a scalar stream
//! - **Normalize**: Divide input by RMS to produce constant-envelope output
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rms::{RmsPower, RmsNormalizer};
//! use num_complex::Complex64;
//!
//! // Measure RMS power
//! let mut rms = RmsPower::new(0.01);
//! let input = vec![Complex64::new(3.0, 4.0); 100]; // magnitude = 5
//! let powers = rms.process(&input);
//! // RMS converges toward 5.0
//!
//! // Normalize to unit power
//! let mut norm = RmsNormalizer::new(0.01);
//! let output = norm.process(&input);
//! // Output magnitudes converge toward 1.0
//! ```

use num_complex::Complex64;

/// RMS power measurement block.
///
/// Computes smoothed RMS power: `rms = sqrt(alpha * |x|^2 + (1-alpha) * rms^2)`
#[derive(Debug, Clone)]
pub struct RmsPower {
    /// Smoothing factor (0 < alpha <= 1)
    alpha: f64,
    /// Current RMS estimate squared
    rms_sq: f64,
}

impl RmsPower {
    /// Create a new RMS power meter.
    ///
    /// - `alpha`: Smoothing factor (smaller = more averaging). Typical: 0.001 to 0.1
    pub fn new(alpha: f64) -> Self {
        assert!(alpha > 0.0 && alpha <= 1.0, "Alpha must be in (0, 1]");
        Self { alpha, rms_sq: 0.0 }
    }

    /// Process complex samples, returning RMS power for each sample.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &sample in input {
            let power = sample.norm_sqr();
            self.rms_sq = self.alpha * power + (1.0 - self.alpha) * self.rms_sq;
            output.push(self.rms_sq.sqrt());
        }
        output
    }

    /// Process real samples, returning RMS power for each sample.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &sample in input {
            let power = sample * sample;
            self.rms_sq = self.alpha * power + (1.0 - self.alpha) * self.rms_sq;
            output.push(self.rms_sq.sqrt());
        }
        output
    }

    /// Process a single complex sample, returning current RMS.
    pub fn process_one(&mut self, sample: Complex64) -> f64 {
        let power = sample.norm_sqr();
        self.rms_sq = self.alpha * power + (1.0 - self.alpha) * self.rms_sq;
        self.rms_sq.sqrt()
    }

    /// Get the current RMS estimate.
    pub fn rms(&self) -> f64 {
        self.rms_sq.sqrt()
    }

    /// Get the current RMS in dB (relative to 1.0).
    pub fn rms_db(&self) -> f64 {
        if self.rms_sq > 0.0 {
            10.0 * self.rms_sq.log10()
        } else {
            f64::NEG_INFINITY
        }
    }

    /// Compute block RMS (no state update).
    pub fn block_rms(input: &[Complex64]) -> f64 {
        if input.is_empty() {
            return 0.0;
        }
        let sum: f64 = input.iter().map(|s| s.norm_sqr()).sum();
        (sum / input.len() as f64).sqrt()
    }

    /// Compute block RMS for real samples (no state update).
    pub fn block_rms_real(input: &[f64]) -> f64 {
        if input.is_empty() {
            return 0.0;
        }
        let sum: f64 = input.iter().map(|&s| s * s).sum();
        (sum / input.len() as f64).sqrt()
    }

    /// Reset the RMS estimate.
    pub fn reset(&mut self) {
        self.rms_sq = 0.0;
    }
}

/// RMS normalizer — divides input by its RMS to produce unit-power output.
#[derive(Debug, Clone)]
pub struct RmsNormalizer {
    /// Internal RMS measurement
    rms: RmsPower,
    /// Minimum RMS to prevent division by near-zero
    min_rms: f64,
}

impl RmsNormalizer {
    /// Create a new RMS normalizer.
    pub fn new(alpha: f64) -> Self {
        Self {
            rms: RmsPower::new(alpha),
            min_rms: 1e-10,
        }
    }

    /// Create with custom minimum RMS floor.
    pub fn with_min_rms(alpha: f64, min_rms: f64) -> Self {
        Self {
            rms: RmsPower::new(alpha),
            min_rms,
        }
    }

    /// Process complex samples, returning normalized output.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &sample in input {
            let rms_val = self.rms.process_one(sample);
            let gain = if rms_val > self.min_rms {
                1.0 / rms_val
            } else {
                1.0
            };
            output.push(sample * gain);
        }
        output
    }

    /// Process real samples, returning normalized output.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &sample in input {
            let power = sample * sample;
            self.rms.rms_sq =
                self.rms.alpha * power + (1.0 - self.rms.alpha) * self.rms.rms_sq;
            let rms_val = self.rms.rms_sq.sqrt();
            let gain = if rms_val > self.min_rms {
                1.0 / rms_val
            } else {
                1.0
            };
            output.push(sample * gain);
        }
        output
    }

    /// Get the current RMS estimate.
    pub fn rms(&self) -> f64 {
        self.rms.rms()
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.rms.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rms_constant_signal() {
        let mut rms = RmsPower::new(0.1);
        let input = vec![Complex64::new(3.0, 4.0); 200]; // magnitude = 5
        let output = rms.process(&input);

        // Should converge toward 5.0
        let last = *output.last().unwrap();
        assert!(
            (last - 5.0).abs() < 0.5,
            "RMS {} not close to 5.0",
            last
        );
    }

    #[test]
    fn test_rms_real_signal() {
        let mut rms = RmsPower::new(0.1);
        let input = vec![3.0_f64; 200];
        let output = rms.process_real(&input);

        let last = *output.last().unwrap();
        assert!(
            (last - 3.0).abs() < 0.3,
            "RMS {} not close to 3.0",
            last
        );
    }

    #[test]
    fn test_block_rms() {
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let rms = RmsPower::block_rms(&input);
        assert!((rms - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_block_rms_real() {
        let input = vec![2.0_f64; 100];
        let rms = RmsPower::block_rms_real(&input);
        assert!((rms - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_rms_db() {
        let mut rms = RmsPower::new(1.0); // alpha=1 → no averaging
        rms.process_one(Complex64::new(10.0, 0.0));
        // rms = 10, rms_sq = 100 → 10*log10(100) = 20 dB
        assert!((rms.rms_db() - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_normalizer_converges() {
        let mut norm = RmsNormalizer::new(0.05);
        let input = vec![Complex64::new(5.0, 0.0); 500];
        let output = norm.process(&input);

        // Last samples should have magnitude near 1.0
        let last_mag = output.last().unwrap().norm();
        assert!(
            (last_mag - 1.0).abs() < 0.3,
            "Normalized magnitude {} not close to 1.0",
            last_mag
        );
    }

    #[test]
    fn test_normalizer_real() {
        let mut norm = RmsNormalizer::new(0.05);
        let input = vec![3.0_f64; 500];
        let output = norm.process_real(&input);

        let last = output.last().unwrap().abs();
        assert!(
            (last - 1.0).abs() < 0.3,
            "Normalized value {} not close to 1.0",
            last
        );
    }

    #[test]
    fn test_normalizer_near_zero() {
        let mut norm = RmsNormalizer::new(0.1);
        let input = vec![Complex64::new(1e-15, 0.0); 10];
        let output = norm.process(&input);
        // Should not explode
        for &s in &output {
            assert!(s.norm() < 1e6, "Output exploded: {}", s.norm());
        }
    }

    #[test]
    fn test_empty_block_rms() {
        assert_eq!(RmsPower::block_rms(&[]), 0.0);
        assert_eq!(RmsPower::block_rms_real(&[]), 0.0);
    }

    #[test]
    fn test_reset() {
        let mut rms = RmsPower::new(0.1);
        rms.process(&vec![Complex64::new(10.0, 0.0); 50]);
        assert!(rms.rms() > 1.0);
        rms.reset();
        assert_eq!(rms.rms(), 0.0);
    }
}

//! # IQ Imbalance Estimator and Corrector
//!
//! Estimates and corrects IQ gain and phase imbalance in SDR receivers.
//! IQ imbalance causes image frequency leakage, degrading signal quality.
//! Complements the existing `iq_balance` module with estimation capabilities.
//!
//! ## IQ Imbalance Model
//!
//! y_I(t) = x_I(t)
//! y_Q(t) = α * x_Q(t) * cos(φ) + α * x_I(t) * sin(φ)
//!
//! where α is the gain imbalance and φ is the phase imbalance.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::iq_imbalance_estimator::IqImbalanceEstimator;
//!
//! let mut estimator = IqImbalanceEstimator::new();
//!
//! // Feed IQ samples from receiver
//! let samples = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
//! estimator.process(&samples);
//!
//! let (gain_db, phase_deg) = estimator.estimate();
//! ```

/// IQ imbalance estimator using statistical moments.
#[derive(Debug, Clone)]
pub struct IqImbalanceEstimator {
    /// Running sum of I^2.
    sum_i2: f64,
    /// Running sum of Q^2.
    sum_q2: f64,
    /// Running sum of I*Q.
    sum_iq: f64,
    /// Total samples processed.
    count: u64,
    /// Exponential smoothing factor (0 = no smoothing, 1 = infinite memory).
    alpha: f64,
}

impl IqImbalanceEstimator {
    /// Create a new IQ imbalance estimator.
    pub fn new() -> Self {
        Self {
            sum_i2: 0.0,
            sum_q2: 0.0,
            sum_iq: 0.0,
            count: 0,
            alpha: 0.0, // batch mode (no smoothing)
        }
    }

    /// Create an estimator with exponential smoothing.
    pub fn with_smoothing(alpha: f64) -> Self {
        Self {
            sum_i2: 0.0,
            sum_q2: 0.0,
            sum_iq: 0.0,
            count: 0,
            alpha: alpha.clamp(0.0, 0.9999),
        }
    }

    /// Process a block of IQ samples.
    pub fn process(&mut self, samples: &[(f64, f64)]) {
        for &(i, q) in samples {
            if self.alpha > 0.0 {
                self.sum_i2 = self.alpha * self.sum_i2 + (1.0 - self.alpha) * i * i;
                self.sum_q2 = self.alpha * self.sum_q2 + (1.0 - self.alpha) * q * q;
                self.sum_iq = self.alpha * self.sum_iq + (1.0 - self.alpha) * i * q;
            } else {
                self.sum_i2 += i * i;
                self.sum_q2 += q * q;
                self.sum_iq += i * q;
            }
            self.count += 1;
        }
    }

    /// Estimate the IQ imbalance.
    /// Returns (gain_imbalance_dB, phase_imbalance_degrees).
    pub fn estimate(&self) -> (f64, f64) {
        if self.count == 0 {
            return (0.0, 0.0);
        }

        let (e_i2, e_q2, e_iq) = if self.alpha > 0.0 {
            (self.sum_i2, self.sum_q2, self.sum_iq)
        } else {
            let n = self.count as f64;
            (self.sum_i2 / n, self.sum_q2 / n, self.sum_iq / n)
        };

        // Gain imbalance: α = sqrt(E[Q²] / E[I²])
        let gain_linear = if e_i2 > 1e-30 {
            (e_q2 / e_i2).sqrt()
        } else {
            1.0
        };
        let gain_db = 20.0 * gain_linear.log10();

        // Phase imbalance: sin(φ) ≈ E[I*Q] / sqrt(E[I²]*E[Q²])
        let cross = if e_i2 > 1e-30 && e_q2 > 1e-30 {
            e_iq / (e_i2 * e_q2).sqrt()
        } else {
            0.0
        };
        let phase_rad = cross.clamp(-1.0, 1.0).asin();
        let phase_deg = phase_rad.to_degrees();

        (gain_db, phase_deg)
    }

    /// Get the image rejection ratio (IRR) in dB.
    pub fn image_rejection_db(&self) -> f64 {
        let (gain_db, phase_deg) = self.estimate();
        let g = 10.0_f64.powf(gain_db / 20.0);
        let phi = phase_deg.to_radians();

        // IRR = (1 + 2g*cos(φ) + g²) / (1 - 2g*cos(φ) + g²)
        let num = 1.0 + 2.0 * g * phi.cos() + g * g;
        let den = 1.0 - 2.0 * g * phi.cos() + g * g;

        if den > 1e-30 {
            10.0 * (num / den).log10()
        } else {
            100.0 // perfect balance
        }
    }

    /// Get total samples processed.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Reset the estimator.
    pub fn reset(&mut self) {
        self.sum_i2 = 0.0;
        self.sum_q2 = 0.0;
        self.sum_iq = 0.0;
        self.count = 0;
    }
}

impl Default for IqImbalanceEstimator {
    fn default() -> Self {
        Self::new()
    }
}

/// Apply IQ imbalance correction to samples.
///
/// # Arguments
/// * `samples` - Input IQ samples
/// * `gain_correction` - Gain correction factor for Q channel (linear)
/// * `phase_correction` - Phase correction in radians
pub fn correct_iq_imbalance(
    samples: &[(f64, f64)],
    gain_correction: f64,
    phase_correction: f64,
) -> Vec<(f64, f64)> {
    let sin_phi = phase_correction.sin();
    let cos_phi = phase_correction.cos();

    samples
        .iter()
        .map(|&(i, q)| {
            // Correct: Q' = (Q - I*sin(φ)) / (α*cos(φ))
            let q_corrected = (q - i * sin_phi) / (gain_correction * cos_phi);
            (i, q_corrected)
        })
        .collect()
}

/// Apply IQ imbalance to clean signal (for testing).
pub fn apply_iq_imbalance(
    samples: &[(f64, f64)],
    gain_imbalance: f64,
    phase_imbalance_rad: f64,
) -> Vec<(f64, f64)> {
    let sin_phi = phase_imbalance_rad.sin();
    let cos_phi = phase_imbalance_rad.cos();

    samples
        .iter()
        .map(|&(i, q)| {
            let q_impaired = gain_imbalance * (q * cos_phi + i * sin_phi);
            (i, q_impaired)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_tone(n: usize, freq: f64, fs: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_balanced_signal() {
        let mut est = IqImbalanceEstimator::new();
        let signal = make_tone(10000, 1000.0, 48000.0);
        est.process(&signal);
        let (gain_db, phase_deg) = est.estimate();
        assert!(gain_db.abs() < 0.5, "Balanced signal: gain_db={}", gain_db);
        assert!(
            phase_deg.abs() < 2.0,
            "Balanced signal: phase_deg={}",
            phase_deg
        );
    }

    #[test]
    fn test_gain_imbalance_detection() {
        let mut est = IqImbalanceEstimator::new();
        let signal = make_tone(10000, 1000.0, 48000.0);
        // Apply 3dB gain imbalance to Q.
        let gain = 10.0_f64.powf(3.0 / 20.0); // ~1.41
        let impaired = apply_iq_imbalance(&signal, gain, 0.0);
        est.process(&impaired);
        let (gain_db, _) = est.estimate();
        assert!(
            (gain_db - 3.0).abs() < 1.0,
            "Should detect ~3dB gain imbalance, got {}",
            gain_db
        );
    }

    #[test]
    fn test_phase_imbalance_detection() {
        let mut est = IqImbalanceEstimator::new();
        let signal = make_tone(10000, 1000.0, 48000.0);
        let phase_rad = 5.0_f64.to_radians();
        let impaired = apply_iq_imbalance(&signal, 1.0, phase_rad);
        est.process(&impaired);
        let (_, phase_deg) = est.estimate();
        assert!(
            (phase_deg - 5.0).abs() < 2.0,
            "Should detect ~5° phase imbalance, got {}",
            phase_deg
        );
    }

    #[test]
    fn test_image_rejection() {
        let mut est = IqImbalanceEstimator::new();
        let signal = make_tone(10000, 1000.0, 48000.0);
        est.process(&signal);
        let irr = est.image_rejection_db();
        assert!(irr > 30.0, "Balanced signal should have high IRR, got {}", irr);
    }

    #[test]
    fn test_correction() {
        let signal = make_tone(1000, 1000.0, 48000.0);
        let gain = 1.2;
        let phase = 0.05; // radians
        let impaired = apply_iq_imbalance(&signal, gain, phase);
        let corrected = correct_iq_imbalance(&impaired, gain, phase);

        // After correction, signal should be close to original.
        let mut max_error = 0.0_f64;
        for (orig, corr) in signal.iter().zip(corrected.iter()) {
            let err_i = (orig.0 - corr.0).abs();
            let err_q = (orig.1 - corr.1).abs();
            max_error = max_error.max(err_i).max(err_q);
        }
        assert!(
            max_error < 0.1,
            "Correction should restore signal, max_error={}",
            max_error
        );
    }

    #[test]
    fn test_empty() {
        let est = IqImbalanceEstimator::new();
        let (g, p) = est.estimate();
        assert!((g - 0.0).abs() < 1e-10);
        assert!((p - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_smoothing() {
        let mut est = IqImbalanceEstimator::with_smoothing(0.99);
        let signal = make_tone(1000, 1000.0, 48000.0);
        est.process(&signal);
        assert_eq!(est.count(), 1000);
        let (g, _) = est.estimate();
        assert!(g.abs() < 1.0);
    }

    #[test]
    fn test_reset() {
        let mut est = IqImbalanceEstimator::new();
        est.process(&[(1.0, 0.5), (0.0, 1.0)]);
        assert_eq!(est.count(), 2);
        est.reset();
        assert_eq!(est.count(), 0);
    }

    #[test]
    fn test_apply_imbalance() {
        let signal = vec![(1.0, 0.0), (0.0, 1.0)];
        let impaired = apply_iq_imbalance(&signal, 2.0, 0.0);
        // Q scaled by 2x, no phase offset.
        assert!((impaired[0].1 - 0.0).abs() < 1e-10);
        assert!((impaired[1].1 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_count() {
        let mut est = IqImbalanceEstimator::new();
        est.process(&[(1.0, 0.0); 50]);
        assert_eq!(est.count(), 50);
    }
}

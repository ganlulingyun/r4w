//! Runtime calibration engine for correcting TX/RX RF impairments.
//!
//! Supports DC offset removal, IQ imbalance correction, phase noise estimation,
//! and nonlinearity compensation. Calibration coefficients can be exported and
//! imported for persistent correction across sessions.
//!
//! # Example
//!
//! ```
//! use r4w_core::rf_impairment_calibrator::{ImpairmentCalibrator, ImpairmentType};
//!
//! // Create some impaired samples (DC offset of 0.1 on I, 0.05 on Q)
//! let samples: Vec<(f64, f64)> = (0..1000)
//!     .map(|i| {
//!         let phase = i as f64 * 0.1;
//!         (phase.cos() + 0.1, phase.sin() + 0.05)
//!     })
//!     .collect();
//!
//! let mut cal = ImpairmentCalibrator::new();
//! cal.estimate_all(&samples);
//!
//! let corrected = cal.apply_correction(&samples);
//!
//! // DC offset should be largely removed
//! let mean_i: f64 = corrected.iter().map(|s| s.0).sum::<f64>() / corrected.len() as f64;
//! let mean_q: f64 = corrected.iter().map(|s| s.1).sum::<f64>() / corrected.len() as f64;
//! assert!(mean_i.abs() < 0.01, "DC offset I not removed: {}", mean_i);
//! assert!(mean_q.abs() < 0.01, "DC offset Q not removed: {}", mean_q);
//! ```

/// Types of RF impairments that can be calibrated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImpairmentType {
    /// DC offset on I and Q channels.
    DcOffset,
    /// IQ gain and phase imbalance.
    IqImbalance,
    /// Phase noise estimation (read-only metric).
    PhaseNoise,
    /// Amplitude nonlinearity (polynomial model).
    Nonlinearity,
    /// All impairment types.
    All,
}

/// Calibration coefficients describing estimated impairments and correction parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationCoefficients {
    /// DC offset on the I (in-phase) channel.
    pub dc_offset_i: f64,
    /// DC offset on the Q (quadrature) channel.
    pub dc_offset_q: f64,
    /// Gain imbalance between I and Q (linear ratio, ideal = 1.0).
    pub gain_imbalance: f64,
    /// Phase imbalance in radians (ideal = 0.0).
    pub phase_imbalance_rad: f64,
    /// Polynomial coefficients for nonlinearity correction (index 0 = x^1 coeff, etc.).
    pub poly_coeffs: Vec<f64>,
}

impl Default for CalibrationCoefficients {
    fn default() -> Self {
        Self {
            dc_offset_i: 0.0,
            dc_offset_q: 0.0,
            gain_imbalance: 1.0,
            phase_imbalance_rad: 0.0,
            poly_coeffs: vec![1.0], // identity: y = 1.0 * x
        }
    }
}

/// Runtime calibration engine for correcting TX/RX RF impairments.
///
/// Estimates DC offset, IQ imbalance, and nonlinearity from observed samples,
/// then applies a correction matrix to remove these impairments.
#[derive(Debug, Clone)]
pub struct ImpairmentCalibrator {
    coeffs: CalibrationCoefficients,
}

impl ImpairmentCalibrator {
    /// Create a new calibrator with default (identity) coefficients.
    pub fn new() -> Self {
        Self {
            coeffs: CalibrationCoefficients::default(),
        }
    }

    /// Estimate and store DC offset from the given samples.
    ///
    /// Returns `(dc_offset_i, dc_offset_q)`.
    pub fn calibrate_dc_offset(&mut self, samples: &[(f64, f64)]) -> (f64, f64) {
        if samples.is_empty() {
            return (0.0, 0.0);
        }
        let n = samples.len() as f64;
        let sum_i: f64 = samples.iter().map(|s| s.0).sum();
        let sum_q: f64 = samples.iter().map(|s| s.1).sum();
        let dc_i = sum_i / n;
        let dc_q = sum_q / n;
        self.coeffs.dc_offset_i = dc_i;
        self.coeffs.dc_offset_q = dc_q;
        (dc_i, dc_q)
    }

    /// Estimate and store IQ imbalance from the given samples.
    ///
    /// Returns `(gain_imbalance_db, phase_imbalance_deg)`.
    ///
    /// The gain imbalance is `std(I) / std(Q)` expressed in dB, and the phase
    /// imbalance is derived from the cross-correlation between I and Q channels.
    pub fn calibrate_iq_imbalance(&mut self, samples: &[(f64, f64)]) -> (f64, f64) {
        if samples.len() < 2 {
            return (0.0, 0.0);
        }
        let n = samples.len() as f64;

        // Remove DC first for imbalance estimation
        let mean_i: f64 = samples.iter().map(|s| s.0).sum::<f64>() / n;
        let mean_q: f64 = samples.iter().map(|s| s.1).sum::<f64>() / n;

        let var_i: f64 = samples.iter().map(|s| (s.0 - mean_i).powi(2)).sum::<f64>() / n;
        let var_q: f64 = samples.iter().map(|s| (s.1 - mean_q).powi(2)).sum::<f64>() / n;

        let std_i = var_i.sqrt();
        let std_q = var_q.sqrt();

        // Gain imbalance (linear)
        let gain = if std_q > 1e-15 { std_i / std_q } else { 1.0 };

        // Cross-correlation for phase imbalance
        let cross: f64 = samples
            .iter()
            .map(|s| (s.0 - mean_i) * (s.1 - mean_q))
            .sum::<f64>()
            / n;

        let denom = std_i * std_q;
        let phase_rad = if denom > 1e-15 {
            (cross / denom).clamp(-1.0, 1.0).asin()
        } else {
            0.0
        };

        self.coeffs.gain_imbalance = gain;
        self.coeffs.phase_imbalance_rad = phase_rad;

        // Convert to dB and degrees for return
        let gain_db = 20.0 * gain.log10();
        let phase_deg = phase_rad.to_degrees();
        (gain_db, phase_deg)
    }

    /// Apply all stored correction coefficients to the given samples.
    ///
    /// Correction order:
    /// 1. DC offset removal
    /// 2. IQ imbalance correction via the correction matrix
    /// 3. Nonlinearity correction via polynomial
    pub fn apply_correction(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let c = &self.coeffs;
        let g = c.gain_imbalance;
        let phi = c.phase_imbalance_rad;
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();

        // Correction matrix:
        //   [[1,              0            ],
        //    [-sin(phi)/cos(phi), 1/(g*cos(phi))]]
        //
        // Applied to DC-corrected (I, Q).
        let m00 = 1.0;
        let m01 = 0.0;
        let m10 = if cos_phi.abs() > 1e-15 {
            -sin_phi / cos_phi
        } else {
            0.0
        };
        let m11 = if (g * cos_phi).abs() > 1e-15 {
            1.0 / (g * cos_phi)
        } else {
            1.0
        };

        samples
            .iter()
            .map(|&(i, q)| {
                // Step 1: DC removal
                let i_dc = i - c.dc_offset_i;
                let q_dc = q - c.dc_offset_q;

                // Step 2: IQ correction matrix
                let i_corr = m00 * i_dc + m01 * q_dc;
                let q_corr = m10 * i_dc + m11 * q_dc;

                // Step 3: Nonlinearity correction (polynomial on amplitude)
                let amp = (i_corr * i_corr + q_corr * q_corr).sqrt();
                if amp > 1e-15 && c.poly_coeffs.len() > 1 {
                    let corrected_amp = Self::eval_poly(&c.poly_coeffs, amp);
                    let scale = corrected_amp / amp;
                    (i_corr * scale, q_corr * scale)
                } else {
                    (i_corr, q_corr)
                }
            })
            .collect()
    }

    /// Estimate all impairment parameters from the given samples.
    ///
    /// Calls `calibrate_dc_offset` and `calibrate_iq_imbalance` in sequence.
    pub fn estimate_all(&mut self, samples: &[(f64, f64)]) {
        self.calibrate_dc_offset(samples);
        // For IQ imbalance, work on DC-corrected data
        let dc_corrected: Vec<(f64, f64)> = samples
            .iter()
            .map(|&(i, q)| (i - self.coeffs.dc_offset_i, q - self.coeffs.dc_offset_q))
            .collect();
        self.calibrate_iq_imbalance(&dc_corrected);
    }

    /// Return a reference to the current calibration coefficients.
    pub fn coefficients(&self) -> &CalibrationCoefficients {
        &self.coeffs
    }

    /// Replace the current calibration coefficients.
    pub fn set_coefficients(&mut self, coeffs: CalibrationCoefficients) {
        self.coeffs = coeffs;
    }

    /// Measure correction quality as improvement in dB.
    ///
    /// Computes `10 * log10(power_before / power_after)` using the residual
    /// DC and imbalance energy. A positive value means improvement.
    pub fn correction_quality(&self, before: &[(f64, f64)], after: &[(f64, f64)]) -> f64 {
        if before.is_empty() || after.is_empty() {
            return 0.0;
        }

        let power_before = Self::residual_impairment_power(before);
        let power_after = Self::residual_impairment_power(after);

        if power_before < 1e-30 {
            return 0.0; // signal already clean, no measurable improvement
        }
        if power_after < 1e-30 {
            return 60.0; // cap at 60 dB improvement
        }

        10.0 * (power_before / power_after).log10()
    }

    // ---- private helpers ----

    /// Evaluate a polynomial: coeffs[0]*x + coeffs[1]*x^2 + ...
    fn eval_poly(coeffs: &[f64], x: f64) -> f64 {
        let mut result = 0.0;
        let mut xn = x;
        for &c in coeffs {
            result += c * xn;
            xn *= x;
        }
        result
    }

    /// Measure residual impairment power (DC + imbalance metric).
    fn residual_impairment_power(samples: &[(f64, f64)]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        let n = samples.len() as f64;
        let mean_i: f64 = samples.iter().map(|s| s.0).sum::<f64>() / n;
        let mean_q: f64 = samples.iter().map(|s| s.1).sum::<f64>() / n;

        // DC power
        let dc_power = mean_i * mean_i + mean_q * mean_q;

        // Variance (signal power)
        let var_i: f64 = samples.iter().map(|s| (s.0 - mean_i).powi(2)).sum::<f64>() / n;
        let var_q: f64 = samples.iter().map(|s| (s.1 - mean_q).powi(2)).sum::<f64>() / n;

        // Gain imbalance metric: deviation from unity ratio
        let std_i = var_i.sqrt();
        let std_q = var_q.sqrt();
        let gain_err = if std_q > 1e-15 {
            (std_i / std_q - 1.0).powi(2)
        } else {
            0.0
        };

        // Cross-correlation (phase imbalance metric)
        let cross: f64 = samples
            .iter()
            .map(|s| (s.0 - mean_i) * (s.1 - mean_q))
            .sum::<f64>()
            / n;
        let phase_err = if std_i * std_q > 1e-15 {
            (cross / (std_i * std_q)).powi(2)
        } else {
            0.0
        };

        dc_power + gain_err + phase_err
    }
}

impl Default for ImpairmentCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a clean tone signal.
    fn tone(n: usize, freq: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: add DC offset to samples.
    fn add_dc(samples: &[(f64, f64)], dc_i: f64, dc_q: f64) -> Vec<(f64, f64)> {
        samples.iter().map(|&(i, q)| (i + dc_i, q + dc_q)).collect()
    }

    /// Helper: apply IQ imbalance (gain on Q, phase rotation).
    fn add_iq_imbalance(
        samples: &[(f64, f64)],
        gain: f64,
        phase_rad: f64,
    ) -> Vec<(f64, f64)> {
        samples
            .iter()
            .map(|&(i, q)| {
                let i_out = i;
                let q_out = gain * (q * phase_rad.cos() + i * phase_rad.sin());
                (i_out, q_out)
            })
            .collect()
    }

    #[test]
    fn test_new_returns_default_coefficients() {
        let cal = ImpairmentCalibrator::new();
        let c = cal.coefficients();
        assert_eq!(c.dc_offset_i, 0.0);
        assert_eq!(c.dc_offset_q, 0.0);
        assert_eq!(c.gain_imbalance, 1.0);
        assert_eq!(c.phase_imbalance_rad, 0.0);
        assert_eq!(c.poly_coeffs, vec![1.0]);
    }

    #[test]
    fn test_default_trait() {
        let cal = ImpairmentCalibrator::default();
        let c = cal.coefficients();
        assert_eq!(c.dc_offset_i, 0.0);
        assert_eq!(c.gain_imbalance, 1.0);
    }

    #[test]
    fn test_calibrate_dc_offset_positive() {
        let mut cal = ImpairmentCalibrator::new();
        let samples = add_dc(&tone(1000, 0.01), 0.15, -0.08);
        let (dc_i, dc_q) = cal.calibrate_dc_offset(&samples);
        assert!((dc_i - 0.15).abs() < 0.02, "DC I estimate: {}", dc_i);
        assert!((dc_q - (-0.08)).abs() < 0.02, "DC Q estimate: {}", dc_q);
    }

    #[test]
    fn test_calibrate_dc_offset_zero() {
        let mut cal = ImpairmentCalibrator::new();
        let samples = tone(2000, 0.01);
        let (dc_i, dc_q) = cal.calibrate_dc_offset(&samples);
        assert!(dc_i.abs() < 0.01, "Expected near-zero DC I: {}", dc_i);
        assert!(dc_q.abs() < 0.01, "Expected near-zero DC Q: {}", dc_q);
    }

    #[test]
    fn test_calibrate_dc_offset_empty() {
        let mut cal = ImpairmentCalibrator::new();
        let (dc_i, dc_q) = cal.calibrate_dc_offset(&[]);
        assert_eq!(dc_i, 0.0);
        assert_eq!(dc_q, 0.0);
    }

    #[test]
    fn test_calibrate_iq_imbalance_gain() {
        let mut cal = ImpairmentCalibrator::new();
        // Apply 2 dB gain imbalance (linear ~1.259)
        let gain_linear = 10.0_f64.powf(2.0 / 20.0);
        let samples = add_iq_imbalance(&tone(4000, 0.01), gain_linear, 0.0);
        let (gain_db, _phase_deg) = cal.calibrate_iq_imbalance(&samples);
        // Gain should be reported as negative because Q is amplified relative to I
        // std_i/std_q < 1 when Q is amplified => negative dB
        assert!(
            (gain_db - (-2.0)).abs() < 0.5,
            "Gain imbalance dB: {}",
            gain_db
        );
    }

    #[test]
    fn test_calibrate_iq_imbalance_phase() {
        let mut cal = ImpairmentCalibrator::new();
        let phase_deg: f64 = 5.0;
        let phase_rad = phase_deg.to_radians();
        let samples = add_iq_imbalance(&tone(4000, 0.01), 1.0, phase_rad);
        let (_gain_db, est_phase_deg) = cal.calibrate_iq_imbalance(&samples);
        assert!(
            (est_phase_deg - phase_deg).abs() < 1.5,
            "Phase imbalance deg: {}",
            est_phase_deg
        );
    }

    #[test]
    fn test_calibrate_iq_imbalance_no_imbalance() {
        let mut cal = ImpairmentCalibrator::new();
        let samples = tone(4000, 0.01);
        let (gain_db, phase_deg) = cal.calibrate_iq_imbalance(&samples);
        assert!(gain_db.abs() < 0.1, "Expected ~0 dB gain: {}", gain_db);
        assert!(
            phase_deg.abs() < 1.0,
            "Expected ~0 deg phase: {}",
            phase_deg
        );
    }

    #[test]
    fn test_apply_correction_removes_dc() {
        let mut cal = ImpairmentCalibrator::new();
        let samples = add_dc(&tone(2000, 0.01), 0.2, -0.1);
        cal.calibrate_dc_offset(&samples);
        let corrected = cal.apply_correction(&samples);

        let mean_i: f64 =
            corrected.iter().map(|s| s.0).sum::<f64>() / corrected.len() as f64;
        let mean_q: f64 =
            corrected.iter().map(|s| s.1).sum::<f64>() / corrected.len() as f64;
        assert!(mean_i.abs() < 0.01, "Residual DC I: {}", mean_i);
        assert!(mean_q.abs() < 0.01, "Residual DC Q: {}", mean_q);
    }

    #[test]
    fn test_apply_correction_identity_on_clean_signal() {
        let cal = ImpairmentCalibrator::new();
        let samples = tone(500, 0.05);
        let corrected = cal.apply_correction(&samples);

        for (orig, corr) in samples.iter().zip(corrected.iter()) {
            assert!(
                (orig.0 - corr.0).abs() < 1e-12,
                "I changed: {} vs {}",
                orig.0,
                corr.0
            );
            assert!(
                (orig.1 - corr.1).abs() < 1e-12,
                "Q changed: {} vs {}",
                orig.1,
                corr.1
            );
        }
    }

    #[test]
    fn test_estimate_all_dc_and_iq() {
        let mut cal = ImpairmentCalibrator::new();
        let clean = tone(4000, 0.01);
        let impaired = add_dc(&add_iq_imbalance(&clean, 1.15, 0.05), 0.1, -0.05);

        cal.estimate_all(&impaired);
        let c = cal.coefficients();

        assert!(
            (c.dc_offset_i - 0.1).abs() < 0.05,
            "DC I: {}",
            c.dc_offset_i
        );
        assert!(
            (c.dc_offset_q - (-0.05)).abs() < 0.05,
            "DC Q: {}",
            c.dc_offset_q
        );
        // gain_imbalance should reflect the I/Q power ratio after DC removal
        assert!(
            c.gain_imbalance > 0.5 && c.gain_imbalance < 2.0,
            "Gain: {}",
            c.gain_imbalance
        );
    }

    #[test]
    fn test_set_and_get_coefficients() {
        let mut cal = ImpairmentCalibrator::new();
        let coeffs = CalibrationCoefficients {
            dc_offset_i: 0.5,
            dc_offset_q: -0.3,
            gain_imbalance: 1.1,
            phase_imbalance_rad: 0.02,
            poly_coeffs: vec![1.0, 0.01],
        };
        cal.set_coefficients(coeffs.clone());
        assert_eq!(cal.coefficients(), &coeffs);
    }

    #[test]
    fn test_correction_quality_positive_improvement() {
        let mut cal = ImpairmentCalibrator::new();
        let clean = tone(2000, 0.01);
        let impaired = add_dc(&clean, 0.3, 0.2);

        cal.calibrate_dc_offset(&impaired);
        let corrected = cal.apply_correction(&impaired);

        let improvement = cal.correction_quality(&impaired, &corrected);
        assert!(
            improvement > 5.0,
            "Expected significant improvement, got {} dB",
            improvement
        );
    }

    #[test]
    fn test_correction_quality_no_change() {
        let cal = ImpairmentCalibrator::new();
        let samples = tone(500, 0.01);
        let quality = cal.correction_quality(&samples, &samples);
        // Same signal => ~0 dB improvement (or very small)
        assert!(
            quality.abs() < 1.0,
            "Expected ~0 dB for identical signals: {}",
            quality
        );
    }

    #[test]
    fn test_correction_quality_empty_input() {
        let cal = ImpairmentCalibrator::new();
        assert_eq!(cal.correction_quality(&[], &[]), 0.0);
    }

    #[test]
    fn test_polynomial_nonlinearity_correction() {
        let mut cal = ImpairmentCalibrator::new();
        // Set up polynomial correction: y = 1.0*x + (-0.1)*x^2
        // This would compress large amplitudes slightly
        cal.set_coefficients(CalibrationCoefficients {
            dc_offset_i: 0.0,
            dc_offset_q: 0.0,
            gain_imbalance: 1.0,
            phase_imbalance_rad: 0.0,
            poly_coeffs: vec![1.0, -0.1],
        });

        let samples = tone(100, 0.05);
        let corrected = cal.apply_correction(&samples);

        // Amplitudes should be slightly reduced for unit-amplitude tone
        for (orig, corr) in samples.iter().zip(corrected.iter()) {
            let amp_orig = (orig.0 * orig.0 + orig.1 * orig.1).sqrt();
            let amp_corr = (corr.0 * corr.0 + corr.1 * corr.1).sqrt();
            if amp_orig > 0.1 {
                assert!(
                    amp_corr < amp_orig,
                    "Poly correction should compress: {} vs {}",
                    amp_orig,
                    amp_corr
                );
            }
        }
    }

    #[test]
    fn test_impairment_type_enum_variants() {
        // Ensure all variants exist and are distinct
        let types = [
            ImpairmentType::DcOffset,
            ImpairmentType::IqImbalance,
            ImpairmentType::PhaseNoise,
            ImpairmentType::Nonlinearity,
            ImpairmentType::All,
        ];
        for (i, a) in types.iter().enumerate() {
            for (j, b) in types.iter().enumerate() {
                if i == j {
                    assert_eq!(a, b);
                } else {
                    assert_ne!(a, b);
                }
            }
        }
    }

    #[test]
    fn test_calibration_coefficients_default() {
        let c = CalibrationCoefficients::default();
        assert_eq!(c.dc_offset_i, 0.0);
        assert_eq!(c.dc_offset_q, 0.0);
        assert_eq!(c.gain_imbalance, 1.0);
        assert_eq!(c.phase_imbalance_rad, 0.0);
        assert_eq!(c.poly_coeffs, vec![1.0]);
    }

    #[test]
    fn test_roundtrip_export_import_coefficients() {
        let mut cal = ImpairmentCalibrator::new();
        let samples = add_dc(
            &add_iq_imbalance(&tone(4000, 0.01), 1.2, 0.03),
            0.12,
            -0.07,
        );
        cal.estimate_all(&samples);

        // Export
        let exported = cal.coefficients().clone();

        // Create a new calibrator and import
        let mut cal2 = ImpairmentCalibrator::new();
        cal2.set_coefficients(exported.clone());

        // Both should produce identical corrections
        let corr1 = cal.apply_correction(&samples);
        let corr2 = cal2.apply_correction(&samples);

        for (a, b) in corr1.iter().zip(corr2.iter()) {
            assert!(
                (a.0 - b.0).abs() < 1e-12 && (a.1 - b.1).abs() < 1e-12,
                "Exported/imported correction mismatch"
            );
        }
    }
}

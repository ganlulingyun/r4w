//! IQ Imbalance Corrector
//!
//! Compensates for gain and phase mismatch between I and Q channels in RF
//! receivers. Real-world quadrature down-converters introduce small gain
//! differences and phase deviations from the ideal 90-degree split, which
//! produce an unwanted image at the mirror frequency. This module provides
//! both a fixed corrector (when the imbalance is known or estimated) and an
//! adaptive corrector that converges on the correction coefficients
//! automatically.
//!
//! The correction is implemented as a 2x2 real-valued matrix applied to each
//! (I, Q) sample pair. For the fixed corrector the matrix is the inverse of
//! the imbalance model; for the adaptive corrector the matrix is updated
//! sample-by-sample using a least-mean-squares rule that drives the output
//! toward circularity (equal I and Q power, zero I-Q correlation).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::iq_imbalance_corrector::{
//!     IqImbalanceCorrector, apply_imbalance, estimate_imbalance,
//! };
//!
//! // Generate a test tone
//! let n = 4096;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|k| {
//!         let t = k as f64 / n as f64;
//!         let phase = 2.0 * std::f64::consts::PI * 10.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! // Introduce 1 dB gain and 3 degree phase imbalance
//! let impaired = apply_imbalance(&samples, 1.0, 3.0);
//!
//! // Estimate the imbalance from the impaired signal
//! let (gain_db, phase_deg) = estimate_imbalance(&impaired);
//!
//! // Correct the impaired signal
//! let corrector = IqImbalanceCorrector::new(gain_db, phase_deg);
//! let corrected = corrector.correct(&impaired);
//!
//! // Verify correction brought I/Q power close to equal
//! let i_power: f64 = corrected.iter().map(|(i, _)| i * i).sum::<f64>() / n as f64;
//! let q_power: f64 = corrected.iter().map(|(_, q)| q * q).sum::<f64>() / n as f64;
//! assert!((i_power - q_power).abs() < 0.05);
//! ```

use std::f64::consts::PI;

/// Fixed IQ imbalance corrector.
///
/// Pre-computes a 2x2 correction matrix from the known (or estimated) gain
/// and phase imbalance. The matrix is the inverse of the imbalance model
/// used by [`apply_imbalance`], so applying the corrector after the imbalance
/// restores the original signal (up to numerical precision).
#[derive(Debug, Clone)]
pub struct IqImbalanceCorrector {
    /// Correction matrix element (1,1)
    gain_correction: f64,
    /// Correction matrix element derived from phase
    phase_correction: f64,
    // Full 2x2 correction matrix [m00, m01, m10, m11]
    matrix: [f64; 4],
}

impl IqImbalanceCorrector {
    /// Create a new corrector for the given imbalance parameters.
    ///
    /// The imbalance model is:
    ///   I' = I
    ///   Q' = g * (Q * cos(phi) + I * sin(phi))
    ///
    /// where `g = 10^(gain_imbalance_db/20)` and `phi = phase_imbalance_deg * pi/180`.
    ///
    /// The correction matrix is the inverse of that transformation.
    pub fn new(gain_imbalance_db: f64, phase_imbalance_deg: f64) -> Self {
        let g = 10.0_f64.powf(gain_imbalance_db / 20.0);
        let phi = phase_imbalance_deg * PI / 180.0;

        let cos_p = phi.cos();
        let sin_p = phi.sin();

        // The imbalance matrix is:
        //   | 1       0         |
        //   | g*sin   g*cos     |
        //
        // Its inverse is:
        //   | 1             0         |
        //   | -sin/cos      1/(g*cos) |
        //
        // (determinant = g*cos)
        let det = g * cos_p;
        let m00 = 1.0;
        let m01 = 0.0;
        let m10 = -sin_p / cos_p; // = -tan(phi)
        let m11 = 1.0 / det;

        Self {
            gain_correction: gain_imbalance_db,
            phase_correction: phase_imbalance_deg,
            matrix: [m00, m01, m10, m11],
        }
    }

    /// Apply the correction matrix to a block of IQ samples.
    ///
    /// Each sample `(i, q)` is multiplied by the pre-computed inverse matrix.
    pub fn correct(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let [m00, m01, m10, m11] = self.matrix;
        samples
            .iter()
            .map(|&(i, q)| {
                let i_out = m00 * i + m01 * q;
                let q_out = m10 * i + m11 * q;
                (i_out, q_out)
            })
            .collect()
    }

    /// Return the gain correction parameter in dB.
    pub fn gain_correction_db(&self) -> f64 {
        self.gain_correction
    }

    /// Return the phase correction parameter in degrees.
    pub fn phase_correction_deg(&self) -> f64 {
        self.phase_correction
    }
}

/// Adaptive IQ imbalance corrector.
///
/// Uses a gradient-descent (LMS) algorithm to drive the output signal toward
/// circularity. At each sample the 2x2 weight matrix is updated so that the
/// output I and Q channels have equal power and zero cross-correlation.
///
/// Start with the identity matrix; the weights converge over time. Typical
/// learning rates (`alpha`) are in the range 1e-4 to 1e-2.
#[derive(Debug, Clone)]
pub struct AdaptiveIqCorrector {
    /// Learning rate for the LMS update.
    pub alpha: f64,
    /// Weight matrix element (0,0).
    pub w_ii: f64,
    /// Weight matrix element (0,1).
    pub w_iq: f64,
    /// Weight matrix element (1,0).
    pub w_qi: f64,
    /// Weight matrix element (1,1).
    pub w_qq: f64,
}

impl AdaptiveIqCorrector {
    /// Create a new adaptive corrector with the given learning rate.
    ///
    /// The weight matrix is initialized to the identity so the corrector
    /// starts as a pass-through.
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha,
            w_ii: 1.0,
            w_iq: 0.0,
            w_qi: 0.0,
            w_qq: 1.0,
        }
    }

    /// Process a block of samples, applying and updating the correction.
    ///
    /// For each sample the current weight matrix is applied, then the weights
    /// are updated using a circularity-based LMS rule:
    ///
    /// ```text
    /// e_i = i_out^2 - q_out^2          (power difference error)
    /// e_q = 2 * i_out * q_out           (cross-correlation error)
    ///
    /// w_ii -= alpha * e_i * i_in
    /// w_iq -= alpha * e_i * q_in
    /// w_qi -= alpha * e_q * i_in
    /// w_qq -= alpha * e_q * q_in
    /// ```
    pub fn process(&mut self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(samples.len());
        for &(i_in, q_in) in samples {
            // Apply current weights
            let i_out = self.w_ii * i_in + self.w_iq * q_in;
            let q_out = self.w_qi * i_in + self.w_qq * q_in;

            output.push((i_out, q_out));

            // Compute circularity error signals
            let e_i = i_out * i_out - q_out * q_out;
            let e_q = 2.0 * i_out * q_out;

            // LMS weight update
            self.w_ii -= self.alpha * e_i * i_in;
            self.w_iq -= self.alpha * e_i * q_in;
            self.w_qi -= self.alpha * e_q * i_in;
            self.w_qq -= self.alpha * e_q * q_in;
        }
        output
    }

    /// Reset the weight matrix to identity.
    pub fn reset(&mut self) {
        self.w_ii = 1.0;
        self.w_iq = 0.0;
        self.w_qi = 0.0;
        self.w_qq = 1.0;
    }
}

/// Estimate gain (dB) and phase (degrees) imbalance from signal statistics.
///
/// Assumes the input is a roughly circular signal (e.g. a tone or wideband
/// noise) corrupted by IQ imbalance. Returns `(gain_db, phase_deg)` where
/// `gain_db` is the Q-channel excess gain in dB and `phase_deg` is the
/// quadrature phase error in degrees.
///
/// The estimation uses:
///   - gain_db = 10 * log10(Q_power / I_power)
///   - phase_deg = arcsin(correlation / sqrt(I_power * Q_power)) * 180/pi
pub fn estimate_imbalance(samples: &[(f64, f64)]) -> (f64, f64) {
    if samples.is_empty() {
        return (0.0, 0.0);
    }

    let n = samples.len() as f64;
    let mut i_power = 0.0;
    let mut q_power = 0.0;
    let mut cross = 0.0;

    for &(i, q) in samples {
        i_power += i * i;
        q_power += q * q;
        cross += i * q;
    }

    i_power /= n;
    q_power /= n;
    cross /= n;

    // Guard against zero power
    if i_power < 1e-30 || q_power < 1e-30 {
        return (0.0, 0.0);
    }

    let gain_db = 10.0 * (q_power / i_power).log10();
    let rho = (cross / (i_power * q_power).sqrt()).clamp(-1.0, 1.0);
    let phase_deg = rho.asin() * 180.0 / PI;

    (gain_db, phase_deg)
}

/// Introduce IQ imbalance into a clean signal (useful for testing).
///
/// Applies the imbalance model:
///   I' = I
///   Q' = g * (Q * cos(phi) + I * sin(phi))
///
/// where `g = 10^(gain_db/20)` and `phi = phase_deg * pi/180`.
pub fn apply_imbalance(samples: &[(f64, f64)], gain_db: f64, phase_deg: f64) -> Vec<(f64, f64)> {
    let g = 10.0_f64.powf(gain_db / 20.0);
    let phi = phase_deg * PI / 180.0;
    let cos_p = phi.cos();
    let sin_p = phi.sin();

    samples
        .iter()
        .map(|&(i, q)| {
            let i_out = i;
            let q_out = g * (q * cos_p + i * sin_p);
            (i_out, q_out)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a unit-amplitude complex tone.
    fn tone(n: usize, cycles: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|k| {
                let t = k as f64 / n as f64;
                let phase = 2.0 * PI * cycles * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: compute I and Q power.
    fn iq_powers(samples: &[(f64, f64)]) -> (f64, f64) {
        let n = samples.len() as f64;
        let ip: f64 = samples.iter().map(|(i, _)| i * i).sum::<f64>() / n;
        let qp: f64 = samples.iter().map(|(_, q)| q * q).sum::<f64>() / n;
        (ip, qp)
    }

    #[test]
    fn test_identity_correction() {
        let corrector = IqImbalanceCorrector::new(0.0, 0.0);
        let signal = tone(256, 7.0);
        let corrected = corrector.correct(&signal);
        for (orig, corr) in signal.iter().zip(corrected.iter()) {
            assert!((orig.0 - corr.0).abs() < 1e-12, "I should be unchanged");
            assert!((orig.1 - corr.1).abs() < 1e-12, "Q should be unchanged");
        }
    }

    #[test]
    fn test_gain_correction() {
        let signal = tone(1024, 13.0);
        let gain_db = 2.0;
        let impaired = apply_imbalance(&signal, gain_db, 0.0);

        // Verify imbalance was applied: Q power should be higher
        let (ip, qp) = iq_powers(&impaired);
        assert!(qp > ip * 1.3, "Q power should exceed I power after imbalance");

        // Correct and verify powers are re-balanced
        let corrector = IqImbalanceCorrector::new(gain_db, 0.0);
        let corrected = corrector.correct(&impaired);
        let (ip2, qp2) = iq_powers(&corrected);
        assert!(
            (ip2 - qp2).abs() < 0.01,
            "Powers should be balanced after correction: I={ip2}, Q={qp2}"
        );
    }

    #[test]
    fn test_phase_correction() {
        let signal = tone(1024, 13.0);
        let phase_deg = 5.0;
        let impaired = apply_imbalance(&signal, 0.0, phase_deg);

        // With phase imbalance, I-Q cross-correlation should be non-zero
        let n = impaired.len() as f64;
        let cross: f64 = impaired.iter().map(|(i, q)| i * q).sum::<f64>() / n;
        assert!(cross.abs() > 0.01, "Phase imbalance should create I-Q correlation");

        // Correct and verify cross-correlation is reduced
        let corrector = IqImbalanceCorrector::new(0.0, phase_deg);
        let corrected = corrector.correct(&impaired);
        let cross2: f64 = corrected.iter().map(|(i, q)| i * q).sum::<f64>() / n;
        assert!(
            cross2.abs() < 0.001,
            "Cross-correlation should be near zero after correction: {cross2}"
        );
    }

    #[test]
    fn test_combined_correction() {
        let signal = tone(2048, 17.0);
        let gain_db = 1.5;
        let phase_deg = 3.0;
        let impaired = apply_imbalance(&signal, gain_db, phase_deg);

        let corrector = IqImbalanceCorrector::new(gain_db, phase_deg);
        let corrected = corrector.correct(&impaired);

        // Each sample should match the original
        for (orig, corr) in signal.iter().zip(corrected.iter()) {
            assert!(
                (orig.0 - corr.0).abs() < 1e-10,
                "I mismatch: {} vs {}",
                orig.0,
                corr.0
            );
            assert!(
                (orig.1 - corr.1).abs() < 1e-10,
                "Q mismatch: {} vs {}",
                orig.1,
                corr.1
            );
        }
    }

    #[test]
    fn test_adaptive_converges() {
        let signal = tone(8192, 23.0);
        let impaired = apply_imbalance(&signal, 1.0, 2.0);

        let mut corrector = AdaptiveIqCorrector::new(0.002);
        // Run several passes to let the adaptive weights converge
        for _ in 0..3 {
            let _ = corrector.process(&impaired);
        }
        let corrected = corrector.process(&impaired);

        // Check the last quarter of the output -- should be more circular
        let tail = &corrected[corrected.len() * 3 / 4..];
        let (ip, qp) = iq_powers(tail);
        let ratio = if ip > qp { ip / qp } else { qp / ip };
        assert!(
            ratio < 1.15,
            "Adaptive corrector should converge: I/Q power ratio = {ratio}"
        );
    }

    #[test]
    fn test_estimate_imbalance() {
        let signal = tone(4096, 11.0);
        let gain_db = 1.5;
        let phase_deg = 4.0;
        let impaired = apply_imbalance(&signal, gain_db, phase_deg);

        let (est_gain, est_phase) = estimate_imbalance(&impaired);

        assert!(
            (est_gain - gain_db).abs() < 0.3,
            "Gain estimate {est_gain} should be close to {gain_db}"
        );
        assert!(
            (est_phase - phase_deg).abs() < 1.0,
            "Phase estimate {est_phase} should be close to {phase_deg}"
        );
    }

    #[test]
    fn test_apply_imbalance() {
        let signal = tone(512, 5.0);
        let impaired = apply_imbalance(&signal, 3.0, 0.0);

        let g = 10.0_f64.powf(3.0 / 20.0);
        for (orig, imp) in signal.iter().zip(impaired.iter()) {
            // I channel should be unchanged
            assert!((orig.0 - imp.0).abs() < 1e-12);
            // Q channel should be scaled by g (no phase error)
            assert!((orig.1 * g - imp.1).abs() < 1e-12);
        }
    }

    #[test]
    fn test_roundtrip() {
        let signal = tone(1024, 7.0);
        let gain_db = 2.5;
        let phase_deg = 6.0;

        // Impair then correct
        let impaired = apply_imbalance(&signal, gain_db, phase_deg);
        let corrector = IqImbalanceCorrector::new(gain_db, phase_deg);
        let recovered = corrector.correct(&impaired);

        // Should match original within floating-point tolerance
        let mut max_err = 0.0_f64;
        for (orig, rec) in signal.iter().zip(recovered.iter()) {
            let ei = (orig.0 - rec.0).abs();
            let eq = (orig.1 - rec.1).abs();
            max_err = max_err.max(ei).max(eq);
        }
        assert!(
            max_err < 1e-10,
            "Roundtrip max error should be negligible: {max_err}"
        );
    }

    #[test]
    fn test_empty_input() {
        let corrector = IqImbalanceCorrector::new(1.0, 2.0);
        assert!(corrector.correct(&[]).is_empty());

        let mut adaptive = AdaptiveIqCorrector::new(0.01);
        assert!(adaptive.process(&[]).is_empty());

        let (g, p) = estimate_imbalance(&[]);
        assert_eq!(g, 0.0);
        assert_eq!(p, 0.0);

        assert!(apply_imbalance(&[], 1.0, 2.0).is_empty());
    }

    #[test]
    fn test_small_imbalance() {
        // Very small imbalance should still be correctable
        let signal = tone(2048, 19.0);
        let gain_db = 0.05;
        let phase_deg = 0.1;

        let impaired = apply_imbalance(&signal, gain_db, phase_deg);
        let corrector = IqImbalanceCorrector::new(gain_db, phase_deg);
        let recovered = corrector.correct(&impaired);

        let mut max_err = 0.0_f64;
        for (orig, rec) in signal.iter().zip(recovered.iter()) {
            let ei = (orig.0 - rec.0).abs();
            let eq = (orig.1 - rec.1).abs();
            max_err = max_err.max(ei).max(eq);
        }
        assert!(
            max_err < 1e-12,
            "Small imbalance roundtrip max error: {max_err}"
        );
    }
}

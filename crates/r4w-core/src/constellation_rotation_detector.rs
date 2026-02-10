//! Automatic detection and correction of constellation rotation / phase ambiguity
//! in QAM/PSK signals.
//!
//! When a coherent receiver locks onto a carrier, its phase-locked loop can
//! converge to any of the symmetry-equivalent phase offsets of the
//! constellation. This module provides three estimation methods and a
//! correction facility to resolve the ambiguity.
//!
//! # Example
//!
//! ```
//! use r4w_core::constellation_rotation_detector::{
//!     ConstellationRotationDetector, RotationCorrector, RotationMode, DetectionMethod,
//! };
//!
//! // Build a small QPSK constellation rotated by 15 degrees
//! let angle: f64 = 15.0_f64.to_radians();
//! let (s, c) = angle.sin_cos();
//! let ideal: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
//! let rotated: Vec<(f64, f64)> = ideal.iter()
//!     .map(|&(i, q)| (i * c - q * s, i * s + q * c))
//!     .collect();
//!
//! let detector = ConstellationRotationDetector::new(
//!     RotationMode::QPSK,
//!     DetectionMethod::FourthPower,
//! );
//! let est = detector.detect_rotation(&rotated);
//! // The fourth-power method estimates the rotation modulo 90 degrees.
//! assert!((est - angle).abs() < 0.05, "estimated {est} expected ~{angle}");
//!
//! let corrector = RotationCorrector::new();
//! let fixed = corrector.correct(&rotated, est);
//! // After correction the points should be close to the ideal positions.
//! for (f, r) in fixed.iter().zip(ideal.iter()) {
//!     assert!((f.0 - r.0).abs() < 0.05, "I mismatch: {} vs {}", f.0, r.0);
//!     assert!((f.1 - r.1).abs() < 0.05, "Q mismatch: {} vs {}", f.1, r.1);
//! }
//! ```

use std::f64::consts::PI;

// ── enums ────────────────────────────────────────────────────────────

/// Modulation symmetry mode – determines the set of ambiguous phase offsets.
#[derive(Debug, Clone, PartialEq)]
pub enum RotationMode {
    /// BPSK – 180° symmetry (2 candidates).
    BPSK,
    /// QPSK – 90° symmetry (4 candidates).
    QPSK,
    /// 8-PSK – 45° symmetry (8 candidates).
    EightPSK,
    /// 16-QAM – 90° symmetry (same as QPSK for this purpose).
    SixteenQAM,
    /// Arbitrary symmetry angle in radians.
    General(f64),
}

impl RotationMode {
    /// The fundamental symmetry angle in radians.
    pub fn symmetry_angle(&self) -> f64 {
        match self {
            RotationMode::BPSK => PI,
            RotationMode::QPSK => PI / 2.0,
            RotationMode::EightPSK => PI / 4.0,
            RotationMode::SixteenQAM => PI / 2.0,
            RotationMode::General(a) => *a,
        }
    }

    /// The exponent N such that raising to the Nth power collapses symmetry.
    fn power_exponent(&self) -> u32 {
        match self {
            RotationMode::BPSK => 2,
            RotationMode::QPSK => 4,
            RotationMode::EightPSK => 8,
            RotationMode::SixteenQAM => 4,
            RotationMode::General(a) => {
                let n = (2.0 * PI / a).round() as u32;
                if n < 1 { 1 } else { n }
            }
        }
    }
}

/// Phase-offset estimation algorithm.
#[derive(Debug, Clone, PartialEq)]
pub enum DetectionMethod {
    /// Raise samples to the Nth power and extract the mean angle.
    FourthPower,
    /// Bin the sample phases into histogram bins within one symmetry period.
    Histogram,
    /// Decision-directed: compare each sample to its nearest ideal point.
    DecisionDirected,
}

// ── ConstellationRotationDetector ────────────────────────────────────

/// Estimates the residual constellation rotation from a block of IQ samples.
#[derive(Debug, Clone)]
pub struct ConstellationRotationDetector {
    mode: RotationMode,
    method: DetectionMethod,
}

impl ConstellationRotationDetector {
    /// Create a new detector with the given symmetry mode and estimation method.
    pub fn new(mode: RotationMode, method: DetectionMethod) -> Self {
        Self { mode, method }
    }

    /// Return all phase ambiguity candidates in `[0, 2*pi)`.
    pub fn phase_ambiguity_candidates(&self) -> Vec<f64> {
        let step = self.mode.symmetry_angle();
        let n = (2.0 * PI / step).round() as usize;
        (0..n).map(|k| k as f64 * step).collect()
    }

    /// Estimate the rotation angle (radians) present in `samples`.
    ///
    /// The returned angle is in the range `[-symmetry/2, symmetry/2)`.
    pub fn detect_rotation(&self, samples: &[(f64, f64)]) -> f64 {
        match self.method {
            DetectionMethod::FourthPower => self.detect_nth_power(samples),
            DetectionMethod::Histogram => self.detect_histogram(samples),
            DetectionMethod::DecisionDirected => self.detect_decision_directed(samples),
        }
    }

    // ── Fourth-power (Nth-power) method ──────────────────────────────
    //
    // For a constellation with N-fold rotational symmetry, raising every
    // sample to the Nth power in polar form maps all ideal constellation
    // points to the same angle. Any residual rotation theta becomes N*theta
    // in the raised domain, so dividing the mean angle by N recovers theta.

    fn detect_nth_power(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        let n = self.mode.power_exponent();
        let (mut sum_sin, mut sum_cos) = (0.0_f64, 0.0_f64);
        for &(i, q) in samples {
            let r2 = i * i + q * q;
            if r2 < 1e-30 {
                continue;
            }
            let theta = q.atan2(i);
            let raised = theta * n as f64;
            sum_sin += raised.sin();
            sum_cos += raised.cos();
        }
        let mean_angle = sum_sin.atan2(sum_cos);
        // The ideal (unrotated) constellation contributes a fixed offset
        // to the Nth-power angle. We must subtract that reference so only
        // the residual rotation remains.
        let ref_offset = self.nth_power_reference_offset();
        let est = (mean_angle - ref_offset) / n as f64;
        wrap_angle(est, self.mode.symmetry_angle())
    }

    /// The angle that the Nth-power operation yields for the ideal (zero-rotation)
    /// constellation.  We compute it from the reference points themselves.
    fn nth_power_reference_offset(&self) -> f64 {
        let ref_pts = reference_constellation(&self.mode);
        if ref_pts.is_empty() {
            return 0.0;
        }
        let n = self.mode.power_exponent();
        let (mut ss, mut sc) = (0.0_f64, 0.0_f64);
        for &(i, q) in &ref_pts {
            let theta = q.atan2(i) * n as f64;
            ss += theta.sin();
            sc += theta.cos();
        }
        ss.atan2(sc)
    }

    // ── Histogram method ─────────────────────────────────────────────

    fn detect_histogram(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        let sym = self.mode.symmetry_angle();
        let num_bins: usize = 64;
        let bin_width = sym / num_bins as f64;
        let mut bins = vec![0u64; num_bins];

        for &(i, q) in samples {
            let theta = q.atan2(i).rem_euclid(sym);
            let idx = ((theta / bin_width) as usize).min(num_bins - 1);
            bins[idx] += 1;
        }

        // Find the dominant peak bin.
        let peak_bin = bins.iter().enumerate().max_by_key(|(_, &c)| c).unwrap().0;
        let raw = peak_bin as f64 * bin_width + bin_width / 2.0;

        // Compute where the ideal constellation's first peak would land.
        let ref_pts = reference_constellation(&self.mode);
        let expected_first_peak = if ref_pts.is_empty() {
            0.0
        } else {
            // The phase of the first reference point, folded into [0, sym).
            let theta = ref_pts[0].1.atan2(ref_pts[0].0).rem_euclid(sym);
            theta
        };

        wrap_angle(raw - expected_first_peak, sym)
    }

    // ── Decision-directed method ─────────────────────────────────────

    fn detect_decision_directed(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        let reference = reference_constellation(&self.mode);
        if reference.is_empty() {
            return self.detect_nth_power(samples);
        }
        let sym = self.mode.symmetry_angle();
        let mut total_err = 0.0_f64;
        let mut count = 0usize;
        for &(si, sq) in samples {
            let (ri, rq) = nearest_point(si, sq, &reference);
            let sample_phase = sq.atan2(si);
            let ref_phase = rq.atan2(ri);
            let diff = sample_phase - ref_phase;
            total_err += wrap_angle(diff, sym);
            count += 1;
        }
        if count == 0 {
            return 0.0;
        }
        wrap_angle(total_err / count as f64, sym)
    }
}

// ── RotationCorrector ────────────────────────────────────────────────

/// Applies a de-rotation to IQ samples.
#[derive(Debug, Clone)]
pub struct RotationCorrector {
    _private: (),
}

impl RotationCorrector {
    /// Create a new corrector.
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Rotate every sample by `-angle` radians.
    pub fn correct(&self, samples: &[(f64, f64)], angle: f64) -> Vec<(f64, f64)> {
        let (s, c) = (-angle).sin_cos();
        samples
            .iter()
            .map(|&(i, q)| (i * c - q * s, i * s + q * c))
            .collect()
    }

    /// Detect rotation then correct in one shot, using the given detector.
    pub fn auto_correct(
        &self,
        samples: &[(f64, f64)],
        detector: &ConstellationRotationDetector,
    ) -> Vec<(f64, f64)> {
        let angle = detector.detect_rotation(samples);
        self.correct(samples, angle)
    }
}

impl Default for RotationCorrector {
    fn default() -> Self {
        Self::new()
    }
}

// ── helpers ──────────────────────────────────────────────────────────

/// Wrap `angle` into `[-period/2, period/2)`.
fn wrap_angle(angle: f64, period: f64) -> f64 {
    let half = period / 2.0;
    let mut a = ((angle + half) % period + period) % period - half;
    if a < -half {
        a += period;
    }
    a
}

/// Build a reference constellation for decision-directed estimation.
fn reference_constellation(mode: &RotationMode) -> Vec<(f64, f64)> {
    match mode {
        RotationMode::BPSK => vec![(1.0, 0.0), (-1.0, 0.0)],
        RotationMode::QPSK => {
            // Standard QPSK with points on axes: 0, 90, 180, 270 degrees.
            vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
        }
        RotationMode::EightPSK => (0..8)
            .map(|k| {
                let a = k as f64 * PI / 4.0;
                (a.cos(), a.sin())
            })
            .collect(),
        RotationMode::SixteenQAM => {
            let mut pts = Vec::with_capacity(16);
            for &i in &[-3.0, -1.0, 1.0, 3.0] {
                for &q in &[-3.0, -1.0, 1.0, 3.0] {
                    pts.push((i, q));
                }
            }
            pts
        }
        RotationMode::General(_) => Vec::new(),
    }
}

/// Find the nearest point in `ref_pts` to `(si, sq)`.
fn nearest_point(si: f64, sq: f64, ref_pts: &[(f64, f64)]) -> (f64, f64) {
    ref_pts
        .iter()
        .copied()
        .min_by(|&(ai, aq), &(bi, bq)| {
            let da = (ai - si).powi(2) + (aq - sq).powi(2);
            let db = (bi - si).powi(2) + (bq - sq).powi(2);
            da.partial_cmp(&db).unwrap()
        })
        .unwrap()
}

// ── tests ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    /// Rotate a set of IQ points by `angle` radians.
    fn rotate_samples(samples: &[(f64, f64)], angle: f64) -> Vec<(f64, f64)> {
        let (s, c) = angle.sin_cos();
        samples
            .iter()
            .map(|&(i, q)| (i * c - q * s, i * s + q * c))
            .collect()
    }

    /// Standard QPSK: points on axes at unit distance.
    fn qpsk_constellation() -> Vec<(f64, f64)> {
        vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
    }

    fn bpsk_constellation() -> Vec<(f64, f64)> {
        vec![(1.0, 0.0), (-1.0, 0.0)]
    }

    // ---- RotationMode tests ----

    #[test]
    fn symmetry_angles_correct() {
        assert!((RotationMode::BPSK.symmetry_angle() - PI).abs() < 1e-12);
        assert!((RotationMode::QPSK.symmetry_angle() - FRAC_PI_2).abs() < 1e-12);
        assert!((RotationMode::EightPSK.symmetry_angle() - FRAC_PI_4).abs() < 1e-12);
        assert!((RotationMode::SixteenQAM.symmetry_angle() - FRAC_PI_2).abs() < 1e-12);
        assert!((RotationMode::General(1.0).symmetry_angle() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn power_exponents_correct() {
        assert_eq!(RotationMode::BPSK.power_exponent(), 2);
        assert_eq!(RotationMode::QPSK.power_exponent(), 4);
        assert_eq!(RotationMode::EightPSK.power_exponent(), 8);
        assert_eq!(RotationMode::SixteenQAM.power_exponent(), 4);
        assert_eq!(RotationMode::General(PI / 3.0).power_exponent(), 6);
    }

    #[test]
    fn phase_ambiguity_candidates_count() {
        let d = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::FourthPower,
        );
        let cands = d.phase_ambiguity_candidates();
        assert_eq!(cands.len(), 4);
        assert!((cands[0] - 0.0).abs() < 1e-12);
        assert!((cands[1] - FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn bpsk_ambiguity_candidates() {
        let d = ConstellationRotationDetector::new(
            RotationMode::BPSK,
            DetectionMethod::FourthPower,
        );
        let cands = d.phase_ambiguity_candidates();
        assert_eq!(cands.len(), 2);
        assert!((cands[0] - 0.0).abs() < 1e-12);
        assert!((cands[1] - PI).abs() < 1e-12);
    }

    #[test]
    fn eight_psk_ambiguity_candidates() {
        let d = ConstellationRotationDetector::new(
            RotationMode::EightPSK,
            DetectionMethod::FourthPower,
        );
        let cands = d.phase_ambiguity_candidates();
        assert_eq!(cands.len(), 8);
        assert!((cands[2] - FRAC_PI_2).abs() < 1e-12);
    }

    // ---- FourthPower detection tests ----

    #[test]
    fn fourth_power_qpsk_zero_rotation() {
        let pts = qpsk_constellation();
        let d = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::FourthPower,
        );
        let est = d.detect_rotation(&pts);
        assert!(est.abs() < 0.01, "expected ~0, got {est}");
    }

    #[test]
    fn fourth_power_qpsk_small_rotation() {
        let angle = 0.25_f64; // ~14.3 degrees
        let pts = rotate_samples(&qpsk_constellation(), angle);
        let d = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::FourthPower,
        );
        let est = d.detect_rotation(&pts);
        assert!(
            (est - angle).abs() < 0.01,
            "expected {angle}, got {est}"
        );
    }

    #[test]
    fn fourth_power_qpsk_negative_rotation() {
        let angle = -0.3_f64;
        let pts = rotate_samples(&qpsk_constellation(), angle);
        let d = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::FourthPower,
        );
        let est = d.detect_rotation(&pts);
        assert!(
            (est - angle).abs() < 0.01,
            "expected {angle}, got {est}"
        );
    }

    #[test]
    fn fourth_power_bpsk_rotation() {
        let angle = 0.4_f64;
        let pts = rotate_samples(&bpsk_constellation(), angle);
        let d = ConstellationRotationDetector::new(
            RotationMode::BPSK,
            DetectionMethod::FourthPower,
        );
        let est = d.detect_rotation(&pts);
        assert!(
            (est - angle).abs() < 0.01,
            "expected {angle}, got {est}"
        );
    }

    #[test]
    fn fourth_power_8psk_rotation() {
        let constellation: Vec<(f64, f64)> = (0..8)
            .map(|k| {
                let a = k as f64 * FRAC_PI_4;
                (a.cos(), a.sin())
            })
            .collect();
        let angle = 0.15_f64;
        let pts = rotate_samples(&constellation, angle);
        let d = ConstellationRotationDetector::new(
            RotationMode::EightPSK,
            DetectionMethod::FourthPower,
        );
        let est = d.detect_rotation(&pts);
        assert!(
            (est - angle).abs() < 0.01,
            "expected {angle}, got {est}"
        );
    }

    #[test]
    fn fourth_power_empty_samples() {
        let d = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::FourthPower,
        );
        let est = d.detect_rotation(&[]);
        assert!((est - 0.0).abs() < 1e-12);
    }

    // ---- Corrector tests ----

    #[test]
    fn corrector_roundtrip() {
        let pts = qpsk_constellation();
        let angle = 0.5;
        let rotated = rotate_samples(&pts, angle);
        let corrector = RotationCorrector::new();
        let fixed = corrector.correct(&rotated, angle);
        for (f, r) in fixed.iter().zip(pts.iter()) {
            assert!((f.0 - r.0).abs() < 1e-10, "I mismatch");
            assert!((f.1 - r.1).abs() < 1e-10, "Q mismatch");
        }
    }

    #[test]
    fn auto_correct_qpsk() {
        let pts = qpsk_constellation();
        let angle = 0.35;
        let rotated = rotate_samples(&pts, angle);
        let detector = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::FourthPower,
        );
        let corrector = RotationCorrector::new();
        let fixed = corrector.auto_correct(&rotated, &detector);
        for (f, r) in fixed.iter().zip(pts.iter()) {
            assert!((f.0 - r.0).abs() < 0.05, "I: {} vs {}", f.0, r.0);
            assert!((f.1 - r.1).abs() < 0.05, "Q: {} vs {}", f.1, r.1);
        }
    }

    #[test]
    fn corrector_default_trait() {
        let c = RotationCorrector::default();
        let pts = vec![(1.0, 0.0)];
        let out = c.correct(&pts, 0.0);
        assert!((out[0].0 - 1.0).abs() < 1e-12);
        assert!((out[0].1 - 0.0).abs() < 1e-12);
    }

    // ---- Decision-directed tests ----

    #[test]
    fn decision_directed_qpsk() {
        let pts = qpsk_constellation();
        let angle = 0.2;
        let rotated = rotate_samples(&pts, angle);
        let d = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::DecisionDirected,
        );
        let est = d.detect_rotation(&rotated);
        assert!(
            (est - angle).abs() < 0.15,
            "expected {angle}, got {est}"
        );
    }

    #[test]
    fn decision_directed_bpsk() {
        let pts = bpsk_constellation();
        let angle = 0.15;
        let rotated = rotate_samples(&pts, angle);
        let d = ConstellationRotationDetector::new(
            RotationMode::BPSK,
            DetectionMethod::DecisionDirected,
        );
        let est = d.detect_rotation(&rotated);
        assert!(
            (est - angle).abs() < 0.15,
            "expected {angle}, got {est}"
        );
    }

    // ---- Histogram tests ----

    #[test]
    fn histogram_qpsk_no_rotation() {
        // Generate many copies to fill histogram bins well.
        let mut pts = Vec::new();
        for _ in 0..200 {
            pts.extend_from_slice(&qpsk_constellation());
        }
        let d = ConstellationRotationDetector::new(
            RotationMode::QPSK,
            DetectionMethod::Histogram,
        );
        let est = d.detect_rotation(&pts);
        assert!(
            est.abs() < 0.15,
            "expected ~0, got {est}"
        );
    }

    // ---- wrap_angle helper test ----

    #[test]
    fn wrap_angle_basic() {
        assert!((wrap_angle(0.0, FRAC_PI_2) - 0.0).abs() < 1e-12);
        // PI should wrap within [-PI/4, PI/4)
        let w = wrap_angle(PI, FRAC_PI_2);
        assert!(w.abs() <= FRAC_PI_4 + 1e-12, "wrapped {w}");
        // Small positive
        assert!((wrap_angle(0.1, FRAC_PI_2) - 0.1).abs() < 1e-12);
        // Small negative
        assert!((wrap_angle(-0.1, FRAC_PI_2) - (-0.1)).abs() < 1e-12);
    }

    // ---- General mode ----

    #[test]
    fn general_mode_60_degrees() {
        // 6-fold symmetry (60 degrees)
        let sym = PI / 3.0;
        let constellation: Vec<(f64, f64)> = (0..6)
            .map(|k| {
                let a = k as f64 * sym;
                (a.cos(), a.sin())
            })
            .collect();
        let angle = 0.12;
        let pts = rotate_samples(&constellation, angle);
        let d = ConstellationRotationDetector::new(
            RotationMode::General(sym),
            DetectionMethod::FourthPower,
        );
        let est = d.detect_rotation(&pts);
        assert!(
            (est - angle).abs() < 0.1,
            "expected {angle}, got {est}"
        );
    }

    // ---- Reference constellation helpers ----

    #[test]
    fn reference_constellation_16qam_has_16_points() {
        let pts = reference_constellation(&RotationMode::SixteenQAM);
        assert_eq!(pts.len(), 16);
    }

    #[test]
    fn nearest_point_finds_closest() {
        let pts = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        let (ri, rq) = nearest_point(0.9, 0.1, &pts);
        assert!((ri - 1.0).abs() < 1e-12);
        assert!((rq - 0.0).abs() < 1e-12);
    }
}

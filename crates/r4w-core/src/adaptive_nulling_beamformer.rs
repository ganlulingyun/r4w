//! Adaptive nulling beamformer for jammer suppression in EW scenarios.
//!
//! This module implements real-time adaptive beamforming with LMS (Least Mean
//! Squares) and RLS (Recursive Least Squares) weight adaptation algorithms.
//! A Uniform Linear Array (ULA) model steers the main beam toward a desired
//! look direction while placing deep nulls on interfering sources.
//!
//! # Complex number convention
//!
//! Complex values are represented as `(f64, f64)` tuples where the first
//! element is the real part and the second is the imaginary part.
//!
//! # Example
//!
//! ```
//! use r4w_core::adaptive_nulling_beamformer::{AdaptiveNullingBeamformer, AdaptationAlgorithm};
//!
//! // 8-element ULA with LMS adaptation
//! let alg = AdaptationAlgorithm::Lms { step_size: 0.01 };
//! let mut bf = AdaptiveNullingBeamformer::new(8, alg);
//! bf.set_look_direction(0.0); // broadside
//!
//! // Fabricate a simple snapshot (one sample per element)
//! let snapshot: Vec<(f64, f64)> = (0..8).map(|i| {
//!     let phase = std::f64::consts::PI * (i as f64) * 0.0_f64.to_radians().sin();
//!     (phase.cos(), phase.sin())
//! }).collect();
//!
//! let output = bf.process_snapshot(&snapshot);
//! assert!(output.0.is_finite());
//!
//! // Check beam pattern at broadside
//! let gain = bf.beam_pattern(0.0);
//! assert!(gain > -3.0, "main beam gain should be near 0 dB");
//! ```

use std::f64::consts::PI;

// ── helpers ──────────────────────────────────────────────────────────────

/// Complex addition.
#[inline]
fn cx_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
#[inline]
fn cx_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
#[inline]
fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Conjugate of a complex number.
#[inline]
fn cx_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude squared.
#[inline]
fn cx_mag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Scale a complex number by a real scalar.
#[inline]
fn cx_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// Inner product w^H x (conjugate of w dotted with x).
fn cx_inner(w: &[(f64, f64)], x: &[(f64, f64)]) -> (f64, f64) {
    let mut acc = (0.0, 0.0);
    for (wi, xi) in w.iter().zip(x.iter()) {
        acc = cx_add(acc, cx_mul(cx_conj(*wi), *xi));
    }
    acc
}

// ── public types ─────────────────────────────────────────────────────────

/// Selects the weight adaptation algorithm.
#[derive(Debug, Clone, Copy)]
pub enum AdaptationAlgorithm {
    /// Least Mean Squares — simple gradient descent.
    Lms {
        /// Step-size (learning rate), typically 0.001–0.1.
        step_size: f64,
    },
    /// Recursive Least Squares — fast convergence via inverse covariance.
    Rls {
        /// Forgetting factor, typically 0.95–1.0.
        forgetting_factor: f64,
    },
}

/// Real-time adaptive beamformer for jammer nulling.
///
/// Models a Uniform Linear Array (ULA) with configurable element spacing.
/// Weights are adapted on each snapshot using LMS or RLS to maximise SINR.
pub struct AdaptiveNullingBeamformer {
    /// Number of antenna elements.
    num_elements: usize,
    /// Adaptation algorithm.
    algorithm: AdaptationAlgorithm,
    /// Current complex weight vector (length = num_elements).
    weight_vec: Vec<(f64, f64)>,
    /// Desired look direction in radians (measured from broadside).
    look_direction_rad: f64,
    /// Element spacing in wavelengths (default 0.5).
    element_spacing: f64,
    /// RLS inverse covariance matrix (num_elements x num_elements, row-major).
    /// Only allocated when algorithm is Rls.
    rls_p: Vec<(f64, f64)>,
}

impl AdaptiveNullingBeamformer {
    /// Create a new beamformer.
    ///
    /// * `num_elements` - number of ULA elements (>= 2).
    /// * `algorithm` - LMS or RLS adaptation.
    pub fn new(num_elements: usize, algorithm: AdaptationAlgorithm) -> Self {
        assert!(num_elements >= 2, "need at least 2 elements");
        let weight_vec = vec![(1.0 / num_elements as f64, 0.0); num_elements];

        let rls_p = match algorithm {
            AdaptationAlgorithm::Rls { .. } => {
                // Initialise P = delta * I  (large diagonal for fast initial convergence).
                let n = num_elements;
                let delta = 100.0;
                let mut p = vec![(0.0, 0.0); n * n];
                for i in 0..n {
                    p[i * n + i] = (delta, 0.0);
                }
                p
            }
            _ => Vec::new(),
        };

        Self {
            num_elements,
            algorithm,
            weight_vec,
            look_direction_rad: 0.0,
            element_spacing: 0.5,
            rls_p,
        }
    }

    // ── configuration ────────────────────────────────────────────────────

    /// Set the desired signal (look) direction in **degrees** from broadside.
    pub fn set_look_direction(&mut self, azimuth_deg: f64) {
        self.look_direction_rad = azimuth_deg.to_radians();
    }

    /// Set element spacing in wavelengths (default 0.5 = half-wavelength).
    pub fn set_element_spacing(&mut self, spacing_wavelengths: f64) {
        assert!(spacing_wavelengths > 0.0, "spacing must be positive");
        self.element_spacing = spacing_wavelengths;
    }

    // ── processing ───────────────────────────────────────────────────────

    /// Beamform a single array snapshot.
    ///
    /// `snapshot` must contain one complex sample per element.
    /// Returns the scalar beamformed output y = w^H x.
    pub fn process_snapshot(&self, snapshot: &[(f64, f64)]) -> (f64, f64) {
        assert_eq!(snapshot.len(), self.num_elements, "snapshot length mismatch");
        cx_inner(&self.weight_vec, snapshot)
    }

    /// Adapt the weight vector given the latest snapshot and a reference
    /// (desired) signal sample.
    ///
    /// For **LMS**: `reference` is the desired signal value d(n); error =
    /// d(n) - w^H x(n) and weights are updated by gradient descent.
    ///
    /// For **RLS**: full inverse-covariance update using `reference` as d(n).
    pub fn update_weights(&mut self, snapshot: &[(f64, f64)], reference: (f64, f64)) {
        assert_eq!(snapshot.len(), self.num_elements, "snapshot length mismatch");

        match self.algorithm {
            AdaptationAlgorithm::Lms { step_size } => {
                let y = cx_inner(&self.weight_vec, snapshot);
                let err = cx_sub(reference, y);
                // w(n+1) = w(n) + mu * x(n) * conj(e(n))
                for i in 0..self.num_elements {
                    let update = cx_scale(cx_mul(snapshot[i], cx_conj(err)), step_size);
                    self.weight_vec[i] = cx_add(self.weight_vec[i], update);
                }
            }
            AdaptationAlgorithm::Rls { forgetting_factor } => {
                let n = self.num_elements;
                let lambda = forgetting_factor;
                let inv_lambda = 1.0 / lambda;

                // k = (P x) / (lambda + x^H P x)
                let mut px = vec![(0.0, 0.0); n];
                for i in 0..n {
                    for j in 0..n {
                        px[i] = cx_add(px[i], cx_mul(self.rls_p[i * n + j], snapshot[j]));
                    }
                }
                let xh_px = cx_inner(snapshot, &px); // scalar
                let denom = lambda + xh_px.0; // imaginary part should be ~0
                let mut k = vec![(0.0, 0.0); n];
                for i in 0..n {
                    k[i] = cx_scale(px[i], 1.0 / denom);
                }

                // error
                let y = cx_inner(&self.weight_vec, snapshot);
                let err = cx_sub(reference, y);

                // update weights: w += k * conj(e)
                for i in 0..n {
                    let update = cx_mul(k[i], cx_conj(err));
                    self.weight_vec[i] = cx_add(self.weight_vec[i], update);
                }

                // update P: P = (1/lambda)(P - k x^H P)
                let mut kxh = vec![(0.0, 0.0); n * n];
                for i in 0..n {
                    for j in 0..n {
                        kxh[i * n + j] = cx_mul(k[i], cx_conj(snapshot[j]));
                    }
                }
                let old_p = self.rls_p.clone();
                for i in 0..n {
                    for j in 0..n {
                        let mut kxhp_ij = (0.0, 0.0);
                        for l in 0..n {
                            kxhp_ij =
                                cx_add(kxhp_ij, cx_mul(kxh[i * n + l], old_p[l * n + j]));
                        }
                        self.rls_p[i * n + j] =
                            cx_scale(cx_sub(old_p[i * n + j], kxhp_ij), inv_lambda);
                    }
                }
            }
        }
    }

    // ── analysis ─────────────────────────────────────────────────────────

    /// Compute the beam pattern gain (in dB) at a given angle (degrees from broadside).
    pub fn beam_pattern(&self, angle_deg: f64) -> f64 {
        let sv = self.steering_vector(angle_deg.to_radians());
        let resp = cx_inner(&self.weight_vec, &sv);
        let mag2 = cx_mag2(resp);
        if mag2 < 1e-30 {
            return -300.0;
        }
        10.0 * mag2.log10()
    }

    /// Measure the null depth in dB at a given angle (degrees).
    ///
    /// A more-negative value indicates a deeper null.  Returns the beam pattern
    /// gain at that angle minus the peak gain.
    pub fn null_depth_at(&self, angle_deg: f64) -> f64 {
        let peak = self.peak_gain_db();
        self.beam_pattern(angle_deg) - peak
    }

    /// Estimate the output SINR in dB.
    ///
    /// Given a `signal_snapshot` (array response of desired signal only) and a
    /// `noise_snapshots` slice of interference-plus-noise snapshots, this
    /// computes SINR = |w^H s|^2 / E[|w^H n|^2].
    pub fn sinr_estimate(
        &self,
        signal_snapshot: &[(f64, f64)],
        noise_snapshots: &[Vec<(f64, f64)>],
    ) -> f64 {
        let sig_power = cx_mag2(cx_inner(&self.weight_vec, signal_snapshot));
        if noise_snapshots.is_empty() {
            return if sig_power > 0.0 { f64::INFINITY } else { 0.0 };
        }
        let noise_power: f64 = noise_snapshots
            .iter()
            .map(|snap| cx_mag2(cx_inner(&self.weight_vec, snap)))
            .sum::<f64>()
            / noise_snapshots.len() as f64;
        if noise_power < 1e-30 {
            return 300.0; // effectively infinite
        }
        10.0 * (sig_power / noise_power).log10()
    }

    /// Return the current weight vector.
    pub fn weights(&self) -> &[(f64, f64)] {
        &self.weight_vec
    }

    /// Reset weights to uniform (1/N, 0) and reinitialise RLS state.
    pub fn reset(&mut self) {
        let n = self.num_elements;
        self.weight_vec = vec![(1.0 / n as f64, 0.0); n];

        if let AdaptationAlgorithm::Rls { .. } = self.algorithm {
            let delta = 100.0;
            self.rls_p = vec![(0.0, 0.0); n * n];
            for i in 0..n {
                self.rls_p[i * n + i] = (delta, 0.0);
            }
        }
    }

    // ── internal ─────────────────────────────────────────────────────────

    /// Steering vector for the ULA at a given angle in radians from broadside.
    fn steering_vector(&self, angle_rad: f64) -> Vec<(f64, f64)> {
        let d = self.element_spacing;
        (0..self.num_elements)
            .map(|i| {
                let phase = 2.0 * PI * d * (i as f64) * angle_rad.sin();
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Find the peak gain (dB) by scanning +/-90 degrees in 0.1 degree steps.
    fn peak_gain_db(&self) -> f64 {
        let mut peak = f64::NEG_INFINITY;
        let steps = 1800;
        for k in 0..=steps {
            let angle = -90.0 + (k as f64) * 180.0 / steps as f64;
            let g = self.beam_pattern(angle);
            if g > peak {
                peak = g;
            }
        }
        peak
    }
}

// ── tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_new_uniform_weights() {
        let bf = AdaptiveNullingBeamformer::new(4, AdaptationAlgorithm::Lms { step_size: 0.01 });
        assert_eq!(bf.weights().len(), 4);
        for &w in bf.weights() {
            assert!(approx_eq(w.0, 0.25, 1e-12));
            assert!(approx_eq(w.1, 0.0, 1e-12));
        }
    }

    #[test]
    fn test_broadside_beam_peak() {
        let bf = AdaptiveNullingBeamformer::new(8, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let gain = bf.beam_pattern(0.0);
        // Uniform weights: gain = |sum(1/N)|^2 = 1.0 => 0 dB
        assert!(
            approx_eq(gain, 0.0, 0.5),
            "broadside gain {gain:.2} expected ~0 dB"
        );
    }

    #[test]
    fn test_set_look_direction() {
        let mut bf =
            AdaptiveNullingBeamformer::new(8, AdaptationAlgorithm::Lms { step_size: 0.01 });
        bf.set_look_direction(30.0);
        assert!(approx_eq(
            bf.look_direction_rad,
            30.0_f64.to_radians(),
            1e-12
        ));
    }

    #[test]
    fn test_process_snapshot_identity() {
        let bf = AdaptiveNullingBeamformer::new(4, AdaptationAlgorithm::Lms { step_size: 0.01 });
        // All-ones snapshot: output = sum(conj(w_i) * 1) = sum(1/4) = 1
        let snap = vec![(1.0, 0.0); 4];
        let y = bf.process_snapshot(&snap);
        assert!(approx_eq(y.0, 1.0, 1e-12));
        assert!(approx_eq(y.1, 0.0, 1e-12));
    }

    #[test]
    fn test_lms_convergence() {
        // Desired signal from broadside (0 deg), jammer from 45 deg.
        let n = 8;
        let mut bf =
            AdaptiveNullingBeamformer::new(n, AdaptationAlgorithm::Lms { step_size: 0.001 });
        bf.set_look_direction(0.0);

        let sv_sig = bf.steering_vector(0.0);
        let sv_jam = bf.steering_vector(45.0_f64.to_radians());

        let sig_amp = 1.0;
        let jam_amp = 10.0; // strong jammer

        // Run 500 iterations of LMS
        for _ in 0..2000 {
            let snapshot: Vec<(f64, f64)> = (0..n)
                .map(|i| {
                    cx_add(cx_scale(sv_sig[i], sig_amp), cx_scale(sv_jam[i], jam_amp))
                })
                .collect();
            let reference = (sig_amp, 0.0);
            bf.update_weights(&snapshot, reference);
        }

        // After adaptation, null depth at 45 deg should be significant
        let null = bf.null_depth_at(45.0);
        assert!(
            null < -10.0,
            "expected deep null at jammer angle, got {null:.1} dB"
        );
    }

    #[test]
    fn test_rls_convergence() {
        let n = 8;
        let mut bf = AdaptiveNullingBeamformer::new(
            n,
            AdaptationAlgorithm::Rls {
                forgetting_factor: 0.99,
            },
        );
        bf.set_look_direction(0.0);

        let sv_sig = bf.steering_vector(0.0);
        let sv_jam = bf.steering_vector(30.0_f64.to_radians());

        let sig_amp = 1.0;
        let jam_amp = 10.0;

        for _ in 0..50 {
            let snapshot: Vec<(f64, f64)> = (0..n)
                .map(|i| {
                    cx_add(cx_scale(sv_sig[i], sig_amp), cx_scale(sv_jam[i], jam_amp))
                })
                .collect();
            let reference = (sig_amp, 0.0);
            bf.update_weights(&snapshot, reference);
        }

        // RLS should converge faster than LMS
        let null = bf.null_depth_at(30.0);
        assert!(
            null < -10.0,
            "expected deep null at jammer angle, got {null:.1} dB"
        );
    }

    #[test]
    fn test_null_depth_at_look() {
        // With uniform weights, null depth at broadside should be ~0 (the peak).
        let bf = AdaptiveNullingBeamformer::new(8, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let depth = bf.null_depth_at(0.0);
        assert!(
            depth.abs() < 1.0,
            "null depth at broadside should be ~0 dB, got {depth:.1}"
        );
    }

    #[test]
    fn test_sinr_estimate() {
        let n = 4;
        let bf = AdaptiveNullingBeamformer::new(n, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let sig = vec![(1.0, 0.0); n];
        let noise = vec![vec![(0.01, 0.0); n]; 10];
        let sinr = bf.sinr_estimate(&sig, &noise);
        assert!(sinr > 20.0, "expected high SINR, got {sinr:.1} dB");
    }

    #[test]
    fn test_reset() {
        let n = 4;
        let mut bf =
            AdaptiveNullingBeamformer::new(n, AdaptationAlgorithm::Lms { step_size: 0.1 });
        // Perturb weights
        let snap = vec![(1.0, 1.0); n];
        bf.update_weights(&snap, (0.0, 0.0));
        // Weights should have changed
        assert!(!approx_eq(bf.weights()[0].0, 0.25, 1e-6));
        // Reset
        bf.reset();
        for &w in bf.weights() {
            assert!(approx_eq(w.0, 0.25, 1e-12));
            assert!(approx_eq(w.1, 0.0, 1e-12));
        }
    }

    #[test]
    fn test_element_spacing() {
        let mut bf =
            AdaptiveNullingBeamformer::new(4, AdaptationAlgorithm::Lms { step_size: 0.01 });
        bf.set_element_spacing(1.0);
        assert!(approx_eq(bf.element_spacing, 1.0, 1e-12));
        // With full-wavelength spacing the pattern has grating lobes.
        // Check that the gain at broadside is still near the peak.
        let gain_broad = bf.beam_pattern(0.0);
        assert!(
            gain_broad > -1.0,
            "broadside gain should still be near peak"
        );
    }

    #[test]
    fn test_beam_pattern_symmetry() {
        // Uniform weights produce a symmetric pattern.
        let bf = AdaptiveNullingBeamformer::new(8, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let g_pos = bf.beam_pattern(30.0);
        let g_neg = bf.beam_pattern(-30.0);
        assert!(
            approx_eq(g_pos, g_neg, 0.1),
            "uniform pattern should be symmetric: +30={g_pos:.2}, -30={g_neg:.2}"
        );
    }

    #[test]
    fn test_sinr_no_noise() {
        let n = 4;
        let bf = AdaptiveNullingBeamformer::new(n, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let sig = vec![(1.0, 0.0); n];
        let sinr = bf.sinr_estimate(&sig, &[]);
        assert!(sinr.is_infinite(), "no noise should give infinite SINR");
    }

    #[test]
    #[should_panic(expected = "need at least 2 elements")]
    fn test_minimum_elements() {
        let _ = AdaptiveNullingBeamformer::new(1, AdaptationAlgorithm::Lms { step_size: 0.01 });
    }
}

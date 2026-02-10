//! Phased array beam steering controller with dynamic weight computation,
//! null placement, and sidelobe suppression.
//!
//! This module provides [`BeamSteeringController`] for computing element-level
//! complex weights that steer a phased array beam to a desired direction while
//! optionally placing nulls toward interferers and applying amplitude tapers
//! to reduce sidelobes.
//!
//! # Supported array geometries
//!
//! * **ULA** – Uniform Linear Array with inter-element spacing `d/λ`.
//! * **UCA** – Uniform Circular Array with radius `r/λ`.
//! * **Custom** – Arbitrary 2-D element positions `(x/λ, y/λ)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::beam_steering_controller::{
//!     ArrayGeometry, BeamSteeringController, SteeringCommand, TaperKind,
//! };
//!
//! // 8-element half-wavelength-spaced ULA
//! let ctrl = BeamSteeringController::new(ArrayGeometry::Ula { n: 8, d_over_lambda: 0.5 });
//!
//! // Steer to 30° azimuth, 0° elevation
//! let cmd = SteeringCommand {
//!     azimuth_deg: 30.0,
//!     elevation_deg: 0.0,
//!     null_directions: vec![],
//! };
//! let weights = ctrl.compute_weights(&cmd);
//! assert_eq!(weights.len(), 8);
//!
//! // Apply Hamming taper for sidelobe control
//! let tapered = ctrl.apply_taper(&weights, TaperKind::Hamming);
//! assert_eq!(tapered.len(), 8);
//!
//! // Compute beam pattern (should peak near 30°)
//! let (angles, pattern_db) = ctrl.beam_pattern(&tapered, -90.0, 90.0, 361);
//! let peak_idx = pattern_db
//!     .iter()
//!     .enumerate()
//!     .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
//!     .unwrap()
//!     .0;
//! let peak_angle = angles[peak_idx];
//! assert!((peak_angle - 30.0).abs() < 1.0, "peak at {peak_angle}°");
//! ```

use std::f64::consts::PI;

// ── complex helpers (re, im) stored as (f64, f64) ──────────────────────────

/// Multiply two complex numbers.
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers.
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex conjugate.
#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude squared.
#[inline]
fn c_mag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Scale a complex number by a real scalar.
#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// `exp(j * theta)`.
#[inline]
fn c_exp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

// ── public types ────────────────────────────────────────────────────────────

/// Array geometry description.
#[derive(Debug, Clone)]
pub enum ArrayGeometry {
    /// Uniform Linear Array along the x-axis.
    /// `d_over_lambda` is the element spacing in wavelengths (typically 0.5).
    Ula {
        n: usize,
        d_over_lambda: f64,
    },
    /// Uniform Circular Array in the x-y plane.
    /// `r_over_lambda` is the array radius in wavelengths.
    Uca {
        n: usize,
        r_over_lambda: f64,
    },
    /// Arbitrary element positions `(x/λ, y/λ)`.
    Custom {
        positions: Vec<(f64, f64)>,
    },
}

/// A steering command specifying the desired look direction and optional nulls.
#[derive(Debug, Clone)]
pub struct SteeringCommand {
    /// Target azimuth in degrees (measured from broadside for ULA).
    pub azimuth_deg: f64,
    /// Target elevation in degrees (0 = horizon, 90 = zenith).
    pub elevation_deg: f64,
    /// Directions (azimuth_deg, elevation_deg) in which to place nulls.
    pub null_directions: Vec<(f64, f64)>,
}

/// Amplitude tapering window types for sidelobe control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TaperKind {
    /// No tapering – uniform weights.
    Uniform,
    /// Hamming window: `0.54 - 0.46 * cos(2πn/(N-1))`.
    Hamming,
    /// Chebyshev window with the specified sidelobe level in dB (positive value, e.g. 30.0).
    Chebyshev(f64),
}

// ── controller ──────────────────────────────────────────────────────────────

/// Phased array beam steering controller.
///
/// Computes per-element complex weights for conventional (phase-shift)
/// beamforming, with optional null placement and amplitude tapering.
#[derive(Debug, Clone)]
pub struct BeamSteeringController {
    /// Element positions in wavelengths: `(x/λ, y/λ)`.
    positions: Vec<(f64, f64)>,
}

impl BeamSteeringController {
    /// Create a new controller from the given array geometry.
    pub fn new(geom: ArrayGeometry) -> Self {
        let positions = match geom {
            ArrayGeometry::Ula { n, d_over_lambda } => {
                (0..n).map(|i| (i as f64 * d_over_lambda, 0.0)).collect()
            }
            ArrayGeometry::Uca { n, r_over_lambda } => {
                (0..n)
                    .map(|i| {
                        let phi = 2.0 * PI * i as f64 / n as f64;
                        (r_over_lambda * phi.cos(), r_over_lambda * phi.sin())
                    })
                    .collect()
            }
            ArrayGeometry::Custom { positions } => positions,
        };
        Self { positions }
    }

    /// Number of elements.
    pub fn num_elements(&self) -> usize {
        self.positions.len()
    }

    /// Compute the steering vector for a given direction.
    ///
    /// For a planar array the phase at element `(x, y)` is
    /// `2π (x sin(az) cos(el) + y sin(el))` where angles are in radians.
    /// For a simple ULA with `el = 0` this reduces to
    /// `a_n = exp(j 2π d n sin(θ) / λ)`.
    pub fn steering_vector(&self, azimuth_deg: f64, elevation_deg: f64) -> Vec<(f64, f64)> {
        let az = azimuth_deg.to_radians();
        let el = elevation_deg.to_radians();
        let ux = az.sin() * el.cos();
        let uy = el.sin();
        self.positions
            .iter()
            .map(|&(x, y)| c_exp_j(2.0 * PI * (x * ux + y * uy)))
            .collect()
    }

    /// Compute conventional (phase-shift) beamforming weights for the
    /// commanded steering direction.  Weights are normalised so that the
    /// array gain in the look direction equals the number of elements.
    pub fn compute_weights(&self, cmd: &SteeringCommand) -> Vec<(f64, f64)> {
        let sv = self.steering_vector(cmd.azimuth_deg, cmd.elevation_deg);
        // w = conj(a) / N  (matched-filter weights)
        let n = sv.len() as f64;
        sv.iter().map(|&a| c_scale(c_conj(a), 1.0 / n)).collect()
    }

    /// Modify existing weights to place nulls in the specified directions
    /// using projection-based null steering.
    ///
    /// For each null direction, the component of the weight vector along
    /// that steering vector is removed:
    /// `w = w - a (a^H w) / (a^H a)`.
    pub fn place_nulls(
        &self,
        weights: &[(f64, f64)],
        null_directions: &[(f64, f64)], // (az_deg, el_deg)
    ) -> Vec<(f64, f64)> {
        let mut w: Vec<(f64, f64)> = weights.to_vec();
        for &(az, el) in null_directions {
            let a = self.steering_vector(az, el);
            // numerator: a^H w (inner product)
            let ah_w = a
                .iter()
                .zip(w.iter())
                .fold((0.0, 0.0), |acc, (&ai, &wi)| c_add(acc, c_mul(c_conj(ai), wi)));
            // denominator: a^H a (always real)
            let ah_a: f64 = a.iter().map(|&ai| c_mag2(ai)).sum();
            if ah_a < 1e-30 {
                continue;
            }
            let scale = (ah_w.0 / ah_a, ah_w.1 / ah_a);
            for (wi, &ai) in w.iter_mut().zip(a.iter()) {
                let proj = c_mul(ai, scale);
                *wi = (wi.0 - proj.0, wi.1 - proj.1);
            }
        }
        w
    }

    /// Apply an amplitude taper (window) to existing complex weights.
    ///
    /// Each weight is scaled by the real-valued window coefficient.
    pub fn apply_taper(&self, weights: &[(f64, f64)], kind: TaperKind) -> Vec<(f64, f64)> {
        let n = weights.len();
        let taper = match kind {
            TaperKind::Uniform => vec![1.0; n],
            TaperKind::Hamming => hamming_window(n),
            TaperKind::Chebyshev(sll_db) => chebyshev_window(n, sll_db),
        };
        weights
            .iter()
            .zip(taper.iter())
            .map(|(&w, &t)| c_scale(w, t))
            .collect()
    }

    /// Compute the beam pattern (power in dB vs azimuth) at a fixed elevation.
    ///
    /// Returns `(angles_deg, pattern_db)` where `pattern_db` is normalised
    /// so that the peak is 0 dB.
    pub fn beam_pattern(
        &self,
        weights: &[(f64, f64)],
        az_start_deg: f64,
        az_end_deg: f64,
        num_points: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let el_deg = 0.0;
        let step = (az_end_deg - az_start_deg) / (num_points as f64 - 1.0);
        let mut angles = Vec::with_capacity(num_points);
        let mut power = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let az = az_start_deg + i as f64 * step;
            angles.push(az);
            let sv = self.steering_vector(az, el_deg);
            let sum = sv
                .iter()
                .zip(weights.iter())
                .fold((0.0, 0.0), |acc, (&a, &w)| c_add(acc, c_mul(w, a)));
            power.push(c_mag2(sum));
        }
        let max_p = power.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let pattern_db: Vec<f64> = power
            .iter()
            .map(|&p| {
                if p < 1e-30 {
                    -300.0
                } else {
                    10.0 * (p / max_p).log10()
                }
            })
            .collect();
        (angles, pattern_db)
    }

    /// Compute weights for multiple simultaneous beams by summing the
    /// individual weight vectors.
    pub fn multiple_beams(&self, commands: &[SteeringCommand]) -> Vec<(f64, f64)> {
        let n = self.num_elements();
        let mut combined = vec![(0.0, 0.0); n];
        for cmd in commands {
            let w = self.compute_weights(cmd);
            for (c, wi) in combined.iter_mut().zip(w.iter()) {
                *c = c_add(*c, *wi);
            }
        }
        // Normalise so peak magnitude = 1/N per beam (scale by 1/num_beams)
        let nb = commands.len() as f64;
        combined.iter_mut().for_each(|c| *c = c_scale(*c, 1.0 / nb));
        combined
    }

    /// Estimate the half-power (3 dB) beamwidth in degrees by scanning the
    /// beam pattern around the look direction.
    ///
    /// Uses 0.01° resolution for accuracy.
    pub fn half_power_beamwidth(&self, weights: &[(f64, f64)], look_az_deg: f64) -> f64 {
        let scan_half = 90.0_f64;
        let start = (look_az_deg - scan_half).max(-90.0);
        let end = (look_az_deg + scan_half).min(90.0);
        let npts = ((end - start) / 0.01) as usize + 1;
        let (angles, pattern_db) = self.beam_pattern(weights, start, end, npts);

        // Find the peak
        let peak_idx = pattern_db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Scan left from peak to find -3 dB crossing
        let mut left_angle = angles[peak_idx];
        for i in (0..peak_idx).rev() {
            if pattern_db[i] <= -3.0 {
                left_angle = angles[i];
                break;
            }
        }

        // Scan right from peak to find -3 dB crossing
        let mut right_angle = angles[peak_idx];
        for i in (peak_idx + 1)..angles.len() {
            if pattern_db[i] <= -3.0 {
                right_angle = angles[i];
                break;
            }
        }

        (right_angle - left_angle).abs()
    }
}

// ── window functions ────────────────────────────────────────────────────────

/// Generate a Hamming window of length `n`.
fn hamming_window(n: usize) -> Vec<f64> {
    if n <= 1 {
        return vec![1.0; n];
    }
    (0..n)
        .map(|i| 0.54 - 0.46 * (2.0 * PI * i as f64 / (n as f64 - 1.0)).cos())
        .collect()
}

/// Generate a Dolph-Chebyshev window of length `n` with the given sidelobe
/// attenuation in dB (positive value, e.g. 30 means -30 dB sidelobes).
///
/// Uses the frequency-sampling method via Chebyshev polynomials of the first
/// kind evaluated at the DFT frequencies.
fn chebyshev_window(n: usize, sll_db: f64) -> Vec<f64> {
    if n <= 1 {
        return vec![1.0; n];
    }

    let m = n - 1; // polynomial order
    let r = 10.0_f64.powf(sll_db / 20.0); // voltage ratio
    // x0 = cosh( (1/M) * acosh(r) )
    let x0 = ((1.0 / m as f64) * r.acosh()).cosh();

    // Evaluate the DFT of the window via Chebyshev polynomial at N frequency bins
    let nn = n as f64;
    let mut w_freq = Vec::with_capacity(n);
    for k in 0..n {
        let arg = x0 * (PI * k as f64 / nn).cos();
        let val = chebyshev_poly(m as u32, arg);
        w_freq.push(val);
    }

    // Inverse DFT (real, symmetric) to get time-domain window
    let mut w = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for k in 0..n {
            sum += w_freq[k] * (2.0 * PI * k as f64 * i as f64 / nn).cos();
        }
        w[i] = sum / nn;
    }

    // Normalise peak to 1.0
    let max_val = w
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    if max_val.abs() > 1e-30 {
        for v in &mut w {
            *v /= max_val;
        }
    }
    // Ensure all values are non-negative (numerical noise can cause tiny negatives)
    for v in &mut w {
        if *v < 0.0 {
            *v = v.abs();
        }
    }
    w
}

/// Evaluate the Chebyshev polynomial of the first kind T_n(x).
fn chebyshev_poly(n: u32, x: f64) -> f64 {
    if x.abs() <= 1.0 {
        (n as f64 * x.acos()).cos()
    } else if x > 1.0 {
        (n as f64 * x.acosh()).cosh()
    } else {
        // x < -1
        let sign = if n % 2 == 0 { 1.0 } else { -1.0 };
        sign * (n as f64 * (-x).acosh()).cosh()
    }
}

// ── peak sidelobe measurement helper ────────────────────────────────────────

/// Given a beam pattern in dB and the peak index, find the worst (highest)
/// sidelobe level outside the main lobe.  The main lobe is defined as the
/// contiguous region around `peak_idx` where the pattern stays above the
/// first null (first point that drops below `null_threshold_db`).
fn peak_sidelobe_level(pattern_db: &[f64], peak_idx: usize, null_threshold_db: f64) -> f64 {
    // Find left edge of main lobe
    let mut left = peak_idx;
    for i in (0..peak_idx).rev() {
        if pattern_db[i] <= null_threshold_db {
            left = i;
            break;
        }
    }
    // Find right edge of main lobe
    let mut right = peak_idx;
    for i in (peak_idx + 1)..pattern_db.len() {
        if pattern_db[i] <= null_threshold_db {
            right = i;
            break;
        }
    }
    // Worst sidelobe outside main lobe
    pattern_db
        .iter()
        .enumerate()
        .filter(|&(i, _)| i < left || i > right)
        .map(|(_, &v)| v)
        .fold(f64::NEG_INFINITY, f64::max)
}

// ── tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: magnitude of a complex number.
    fn mag(c: (f64, f64)) -> f64 {
        c_mag2(c).sqrt()
    }

    // 1. ULA element count
    #[test]
    fn test_ula_element_count() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 16,
            d_over_lambda: 0.5,
        });
        assert_eq!(ctrl.num_elements(), 16);
    }

    // 2. UCA element count and position symmetry
    #[test]
    fn test_uca_positions_symmetric() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Uca {
            n: 8,
            r_over_lambda: 0.5,
        });
        assert_eq!(ctrl.num_elements(), 8);
        // Opposite elements should be symmetric about origin
        let p0 = ctrl.positions[0];
        let p4 = ctrl.positions[4];
        assert!((p0.0 + p4.0).abs() < 1e-12);
        assert!((p0.1 + p4.1).abs() < 1e-12);
    }

    // 3. Custom geometry
    #[test]
    fn test_custom_geometry() {
        let positions = vec![(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)];
        let ctrl = BeamSteeringController::new(ArrayGeometry::Custom { positions });
        assert_eq!(ctrl.num_elements(), 3);
    }

    // 4. Steering vector at broadside is all-ones for ULA
    #[test]
    fn test_steering_vector_broadside() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 4,
            d_over_lambda: 0.5,
        });
        let sv = ctrl.steering_vector(0.0, 0.0); // broadside
        for &s in &sv {
            assert!((s.0 - 1.0).abs() < 1e-12, "real part should be 1.0");
            assert!(s.1.abs() < 1e-12, "imag part should be 0.0");
        }
    }

    // 5. Weights magnitude uniform for phase-shift beamforming
    #[test]
    fn test_weights_uniform_magnitude() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 8,
            d_over_lambda: 0.5,
        });
        let cmd = SteeringCommand {
            azimuth_deg: 20.0,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w = ctrl.compute_weights(&cmd);
        let expected_mag = 1.0 / 8.0;
        for &wi in &w {
            assert!(
                (mag(wi) - expected_mag).abs() < 1e-12,
                "all weights should have magnitude 1/N"
            );
        }
    }

    // 6. Beam pattern peaks at the steered direction
    #[test]
    fn test_beam_pattern_peak_direction() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 16,
            d_over_lambda: 0.5,
        });
        let steer_az = 25.0;
        let cmd = SteeringCommand {
            azimuth_deg: steer_az,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w = ctrl.compute_weights(&cmd);
        let (angles, pattern_db) = ctrl.beam_pattern(&w, -90.0, 90.0, 1801);
        let peak_idx = pattern_db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_angle = angles[peak_idx];
        assert!(
            (peak_angle - steer_az).abs() < 0.5,
            "peak at {peak_angle}°, expected ~{steer_az}°"
        );
    }

    // 7. Null placement suppresses response in null direction
    #[test]
    fn test_null_placement() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 8,
            d_over_lambda: 0.5,
        });
        let cmd = SteeringCommand {
            azimuth_deg: 0.0,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w = ctrl.compute_weights(&cmd);
        let w_nulled = ctrl.place_nulls(&w, &[(30.0, 0.0)]);

        // Evaluate pattern at the null direction
        let sv_null = ctrl.steering_vector(30.0, 0.0);
        let response = sv_null
            .iter()
            .zip(w_nulled.iter())
            .fold((0.0, 0.0), |acc, (&a, &w)| c_add(acc, c_mul(w, a)));
        let null_power = c_mag2(response);
        assert!(
            null_power < 1e-20,
            "null direction should have near-zero response, got {null_power}"
        );
    }

    // 8. Hamming taper reduces sidelobes compared to uniform
    #[test]
    fn test_hamming_taper_reduces_sidelobes() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 32,
            d_over_lambda: 0.5,
        });
        let cmd = SteeringCommand {
            azimuth_deg: 0.0,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w_uniform = ctrl.compute_weights(&cmd);
        let w_hamming = ctrl.apply_taper(&w_uniform, TaperKind::Hamming);

        let (_, pat_uniform) = ctrl.beam_pattern(&w_uniform, -90.0, 90.0, 3601);
        let (_, pat_hamming) = ctrl.beam_pattern(&w_hamming, -90.0, 90.0, 3601);

        // Find peak indices (should be at broadside = index 1800)
        let peak_u = pat_uniform
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_h = pat_hamming
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let sl_uniform = peak_sidelobe_level(&pat_uniform, peak_u, -20.0);
        let sl_hamming = peak_sidelobe_level(&pat_hamming, peak_h, -20.0);

        assert!(
            sl_hamming < sl_uniform,
            "Hamming sidelobe {sl_hamming:.1} dB should be lower than uniform {sl_uniform:.1} dB"
        );
    }

    // 9. Chebyshev taper produces a valid window
    #[test]
    fn test_chebyshev_taper() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 8,
            d_over_lambda: 0.5,
        });
        let cmd = SteeringCommand {
            azimuth_deg: 0.0,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w = ctrl.compute_weights(&cmd);
        let tapered = ctrl.apply_taper(&w, TaperKind::Chebyshev(30.0));
        assert_eq!(tapered.len(), 8);
        // All coefficients should be finite and non-zero
        for &t in &tapered {
            assert!(t.0.is_finite(), "real part must be finite, got {}", t.0);
            assert!(t.1.is_finite(), "imag part must be finite, got {}", t.1);
            assert!(mag(t) > 0.0, "weight magnitude must be > 0");
        }
    }

    // 10. Multiple beams produce a combined pattern with two peaks
    #[test]
    fn test_multiple_beams() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 16,
            d_over_lambda: 0.5,
        });
        let cmds = vec![
            SteeringCommand {
                azimuth_deg: -30.0,
                elevation_deg: 0.0,
                null_directions: vec![],
            },
            SteeringCommand {
                azimuth_deg: 30.0,
                elevation_deg: 0.0,
                null_directions: vec![],
            },
        ];
        let w = ctrl.multiple_beams(&cmds);
        let (angles, pat) = ctrl.beam_pattern(&w, -90.0, 90.0, 1801);

        // Should have a local peak near -30° and +30°
        let idx_m30 = angles.iter().position(|&a| (a - (-30.0)).abs() < 0.15).unwrap();
        let idx_p30 = angles.iter().position(|&a| (a - 30.0).abs() < 0.15).unwrap();
        assert!(pat[idx_m30] > -3.0, "left beam should be within 3 dB of peak");
        assert!(pat[idx_p30] > -3.0, "right beam should be within 3 dB of peak");
    }

    // 11. Half-power beamwidth is reasonable for 16-element ULA
    #[test]
    fn test_half_power_beamwidth() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 16,
            d_over_lambda: 0.5,
        });
        let cmd = SteeringCommand {
            azimuth_deg: 0.0,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w = ctrl.compute_weights(&cmd);
        let hpbw = ctrl.half_power_beamwidth(&w, 0.0);
        // Theoretical ≈ 0.886 * λ/(N*d) radians ≈ 6.4° for N=16, d=0.5λ
        assert!(
            hpbw > 3.0 && hpbw < 15.0,
            "HPBW = {hpbw}° should be in reasonable range"
        );
    }

    // 12. Uniform taper is identity
    #[test]
    fn test_uniform_taper_identity() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 4,
            d_over_lambda: 0.5,
        });
        let cmd = SteeringCommand {
            azimuth_deg: 15.0,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w = ctrl.compute_weights(&cmd);
        let tapered = ctrl.apply_taper(&w, TaperKind::Uniform);
        for (a, b) in w.iter().zip(tapered.iter()) {
            assert!((a.0 - b.0).abs() < 1e-15);
            assert!((a.1 - b.1).abs() < 1e-15);
        }
    }

    // 13. Steering vector magnitudes are all unity
    #[test]
    fn test_steering_vector_unit_magnitude() {
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n: 8,
            d_over_lambda: 0.5,
        });
        let sv = ctrl.steering_vector(45.0, 10.0);
        for &s in &sv {
            assert!(
                (mag(s) - 1.0).abs() < 1e-12,
                "steering vector elements should have unit magnitude"
            );
        }
    }

    // 14. Array gain at look direction equals 1.0 for matched-filter weights
    #[test]
    fn test_array_gain_at_look() {
        let n = 8;
        let ctrl = BeamSteeringController::new(ArrayGeometry::Ula {
            n,
            d_over_lambda: 0.5,
        });
        let cmd = SteeringCommand {
            azimuth_deg: 20.0,
            elevation_deg: 0.0,
            null_directions: vec![],
        };
        let w = ctrl.compute_weights(&cmd);
        let sv = ctrl.steering_vector(20.0, 0.0);
        let response = sv
            .iter()
            .zip(w.iter())
            .fold((0.0, 0.0), |acc, (&a, &wi)| c_add(acc, c_mul(wi, a)));
        // |w^H a| should equal 1.0 (since we normalised by N and sum of |a|^2 = N)
        let gain = mag(response);
        assert!(
            (gain - 1.0).abs() < 1e-10,
            "array gain at look direction should be 1.0, got {gain}"
        );
    }
}

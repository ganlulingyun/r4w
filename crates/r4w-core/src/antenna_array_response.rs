//! Antenna Array Response
//!
//! Implements antenna array response calculations for beamforming and
//! direction-of-arrival (DOA) estimation. Supports uniform linear arrays (ULA),
//! uniform circular arrays (UCA), and arbitrary 2-D element placements.
//!
//! Complex values are represented as `(f64, f64)` tuples of `(re, im)` to keep
//! the module self-contained with no external crate dependencies.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::antenna_array_response::{ArrayGeometry, ArrayResponse, uniform_weights};
//!
//! // 8-element ULA with half-wavelength spacing at 1 GHz (lambda = 0.3 m)
//! let wavelength = 0.3;
//! let geometry = ArrayGeometry::ULA { num_elements: 8, spacing: wavelength / 2.0 };
//! let arr = ArrayResponse::new(geometry, wavelength);
//!
//! // Steer the beam to 30 degrees
//! let weights = arr.steer(30_f64.to_radians());
//! assert_eq!(weights.len(), 8);
//!
//! // Compute the beam pattern
//! let pattern = arr.beam_pattern(&weights, 360);
//! assert_eq!(pattern.len(), 360);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (re, im) tuples
// ---------------------------------------------------------------------------

/// Multiply two complex numbers represented as (re, im) tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex conjugate.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Squared magnitude of a complex number.
#[inline]
fn cnorm_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Construct a complex exponential exp(j * phase).
#[inline]
fn cexp_j(phase: f64) -> (f64, f64) {
    (phase.cos(), phase.sin())
}

// ---------------------------------------------------------------------------
// Array geometry
// ---------------------------------------------------------------------------

/// Antenna array geometry description.
#[derive(Debug, Clone)]
pub enum ArrayGeometry {
    /// Uniform Linear Array along the x-axis.
    ///
    /// Element *n* is located at `(n * spacing, 0)` for `n = 0..num_elements-1`.
    ULA {
        /// Number of antenna elements.
        num_elements: usize,
        /// Inter-element spacing in metres.
        spacing: f64,
    },
    /// Uniform Circular Array in the x-y plane.
    ///
    /// Element *n* is located at angle `2*pi*n/N` on a circle of the given
    /// radius.
    UCA {
        /// Number of antenna elements.
        num_elements: usize,
        /// Array radius in metres.
        radius: f64,
    },
    /// Arbitrary 2-D element positions (x, y) in metres.
    Custom {
        /// Position of each element as `(x, y)` in metres.
        positions: Vec<(f64, f64)>,
    },
}

impl ArrayGeometry {
    /// Return the number of elements in the array.
    pub fn num_elements(&self) -> usize {
        match self {
            ArrayGeometry::ULA { num_elements, .. } => *num_elements,
            ArrayGeometry::UCA { num_elements, .. } => *num_elements,
            ArrayGeometry::Custom { positions } => positions.len(),
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Compute the array steering vector **a**(theta) for a given direction.
///
/// Each element of the returned vector is a complex phasor `(re, im)`
/// representing the phase shift at that element for a plane wave arriving from
/// angle `theta_rad` (measured from broadside for ULA, from the x-axis for
/// UCA).
///
/// # Formulas
///
/// - **ULA**: `a_n = exp(j * 2*pi * d * n * sin(theta) / lambda)`
/// - **UCA**: `a_n = exp(j * 2*pi * r * cos(theta - 2*pi*n/N) / lambda)`
/// - **Custom**: `a_n = exp(j * 2*pi * (x_n*cos(theta) + y_n*sin(theta)) / lambda)`
pub fn steering_vector(
    geometry: &ArrayGeometry,
    wavelength: f64,
    theta_rad: f64,
) -> Vec<(f64, f64)> {
    let k = 2.0 * PI / wavelength;
    match geometry {
        ArrayGeometry::ULA {
            num_elements,
            spacing,
        } => {
            let phase_inc = k * spacing * theta_rad.sin();
            (0..*num_elements)
                .map(|n| cexp_j(phase_inc * n as f64))
                .collect()
        }
        ArrayGeometry::UCA {
            num_elements,
            radius,
        } => {
            let n_f = *num_elements as f64;
            (0..*num_elements)
                .map(|n| {
                    let phi_n = 2.0 * PI * n as f64 / n_f;
                    let phase = k * radius * (theta_rad - phi_n).cos();
                    cexp_j(phase)
                })
                .collect()
        }
        ArrayGeometry::Custom { positions } => positions
            .iter()
            .map(|&(x, y)| {
                let phase = k * (x * theta_rad.cos() + y * theta_rad.sin());
                cexp_j(phase)
            })
            .collect(),
    }
}

/// Compute the array factor (beam response) at a single angle.
///
/// The array factor is the weighted sum of the steering vector:
///
/// `AF(theta) = sum_n  w_n * a_n(theta)`
///
/// Returns a complex value `(re, im)`.
pub fn array_factor(
    geometry: &ArrayGeometry,
    wavelength: f64,
    weights: &[(f64, f64)],
    theta_rad: f64,
) -> (f64, f64) {
    let sv = steering_vector(geometry, wavelength, theta_rad);
    assert_eq!(
        weights.len(),
        sv.len(),
        "Weight vector length must match the number of array elements"
    );
    sv.iter()
        .zip(weights.iter())
        .fold((0.0, 0.0), |acc, (&a, &w)| cadd(acc, cmul(w, a)))
}

/// Compute the beam pattern (power in dB vs angle) over `[-pi, pi]`.
///
/// Returns `num_points` pairs of `(theta, power_dB)`. The pattern is
/// normalised so that the peak is at 0 dB.
pub fn beam_pattern(
    geometry: &ArrayGeometry,
    wavelength: f64,
    weights: &[(f64, f64)],
    num_points: usize,
) -> Vec<(f64, f64)> {
    assert!(num_points > 0, "num_points must be > 0");

    let step = 2.0 * PI / num_points as f64;
    let mut raw: Vec<(f64, f64)> = Vec::with_capacity(num_points);
    let mut max_power: f64 = f64::NEG_INFINITY;

    for i in 0..num_points {
        let theta = -PI + step * i as f64;
        let af = array_factor(geometry, wavelength, weights, theta);
        let power = cnorm_sq(af);
        let power_db = if power > 0.0 {
            10.0 * power.log10()
        } else {
            -200.0
        };
        if power_db > max_power {
            max_power = power_db;
        }
        raw.push((theta, power_db));
    }

    // Normalise to 0 dB peak
    raw.iter().map(|&(t, p)| (t, p - max_power)).collect()
}

/// Generate uniform (equal-amplitude, zero-phase) weights for `num_elements`.
///
/// Each weight is `(1/N, 0)` so that the total power is normalised to unity.
pub fn uniform_weights(num_elements: usize) -> Vec<(f64, f64)> {
    let w = 1.0 / num_elements as f64;
    vec![(w, 0.0); num_elements]
}

/// Generate steering weights that point the beam towards `steer_angle_rad`.
///
/// The weights are the conjugate of the steering vector, normalised by the
/// number of elements:
///
/// `w_n = conj(a_n(theta_0)) / N`
pub fn steering_weights(
    geometry: &ArrayGeometry,
    wavelength: f64,
    steer_angle_rad: f64,
) -> Vec<(f64, f64)> {
    let sv = steering_vector(geometry, wavelength, steer_angle_rad);
    let n = sv.len() as f64;
    sv.iter().map(|&a| {
        let c = conj(a);
        (c.0 / n, c.1 / n)
    }).collect()
}

/// Estimate the half-power (3 dB) beamwidth of the given array and weights.
///
/// The function scans the beam pattern with 3600 points to find the angles at
/// which the power drops 3 dB below the peak, then returns the angular width
/// in radians.
pub fn half_power_beamwidth(
    geometry: &ArrayGeometry,
    wavelength: f64,
    weights: &[(f64, f64)],
) -> f64 {
    let pattern = beam_pattern(geometry, wavelength, weights, 3600);

    // Find peak index
    let (peak_idx, _) = pattern
        .iter()
        .enumerate()
        .max_by(|a, b| a.1 .1.partial_cmp(&b.1 .1).unwrap())
        .unwrap();

    // Search left from peak for -3 dB crossing
    let mut left_angle = pattern[peak_idx].0;
    for i in 1..pattern.len() {
        let idx = (peak_idx + pattern.len() - i) % pattern.len();
        if pattern[idx].1 < -3.0 {
            left_angle = pattern[(idx + 1) % pattern.len()].0;
            break;
        }
    }

    // Search right from peak for -3 dB crossing
    let mut right_angle = pattern[peak_idx].0;
    for i in 1..pattern.len() {
        let idx = (peak_idx + i) % pattern.len();
        if pattern[idx].1 < -3.0 {
            right_angle = pattern[(idx + pattern.len() - 1) % pattern.len()].0;
            break;
        }
    }

    let width = right_angle - left_angle;
    if width < 0.0 {
        width + 2.0 * PI
    } else {
        width
    }
}

// ---------------------------------------------------------------------------
// Convenience struct
// ---------------------------------------------------------------------------

/// Convenience wrapper combining an [`ArrayGeometry`] with a fixed wavelength.
///
/// Provides methods that mirror the free functions but without the need to
/// pass geometry and wavelength every time.
#[derive(Debug, Clone)]
pub struct ArrayResponse {
    /// The antenna array geometry.
    pub geometry: ArrayGeometry,
    /// Operating wavelength in metres.
    pub wavelength: f64,
}

impl ArrayResponse {
    /// Create a new `ArrayResponse` for the given geometry and wavelength.
    pub fn new(geometry: ArrayGeometry, wavelength: f64) -> Self {
        Self {
            geometry,
            wavelength,
        }
    }

    /// Return steering weights that point the beam towards `angle_rad`.
    pub fn steer(&self, angle_rad: f64) -> Vec<(f64, f64)> {
        steering_weights(&self.geometry, self.wavelength, angle_rad)
    }

    /// Compute the beam pattern over `[-pi, pi]`.
    pub fn beam_pattern(&self, weights: &[(f64, f64)], num_points: usize) -> Vec<(f64, f64)> {
        beam_pattern(&self.geometry, self.wavelength, weights, num_points)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-10;

    /// Helper: magnitude of a complex tuple.
    fn cmag(c: (f64, f64)) -> f64 {
        cnorm_sq(c).sqrt()
    }

    #[test]
    fn test_ula_steering_vector() {
        // Half-wavelength ULA, 4 elements, broadside (theta=0)
        let geom = ArrayGeometry::ULA {
            num_elements: 4,
            spacing: 0.5,
        };
        let sv = steering_vector(&geom, 1.0, 0.0);
        assert_eq!(sv.len(), 4);
        // At broadside all elements have zero phase -> (1, 0)
        for &(re, im) in &sv {
            assert!((re - 1.0).abs() < TOL, "re={re}");
            assert!(im.abs() < TOL, "im={im}");
        }

        // At theta = pi/2 (endfire), phase increment = pi per element
        let sv2 = steering_vector(&geom, 1.0, PI / 2.0);
        // Element 0: phase 0 -> (1, 0)
        assert!((sv2[0].0 - 1.0).abs() < TOL);
        // Element 1: phase pi -> (-1, 0)
        assert!((sv2[1].0 - (-1.0)).abs() < TOL);
        assert!(sv2[1].1.abs() < TOL);
    }

    #[test]
    fn test_uca_steering_vector() {
        // 4-element UCA, radius = lambda / (2*pi) so that k*r = 1
        let r = 1.0 / (2.0 * PI);
        let geom = ArrayGeometry::UCA {
            num_elements: 4,
            radius: r,
        };
        let sv = steering_vector(&geom, 1.0, 0.0);
        assert_eq!(sv.len(), 4);

        // All elements should have unit magnitude (they are phasors)
        for &s in &sv {
            let mag = cmag(s);
            assert!(
                (mag - 1.0).abs() < TOL,
                "UCA steering vector element should have unit magnitude, got {mag}"
            );
        }
    }

    #[test]
    fn test_broadside_beam() {
        // Uniform-weighted ULA should have maximum at broadside (theta=0)
        let geom = ArrayGeometry::ULA {
            num_elements: 8,
            spacing: 0.5,
        };
        let w = uniform_weights(8);
        let af_broadside = array_factor(&geom, 1.0, &w, 0.0);
        let power_broadside = cnorm_sq(af_broadside);

        // Power at broadside should be 1.0 (weights sum to 1)
        assert!(
            (power_broadside - 1.0).abs() < TOL,
            "Broadside power = {power_broadside}"
        );

        // Power off-broadside should be less
        let af_off = array_factor(&geom, 1.0, &w, PI / 4.0);
        let power_off = cnorm_sq(af_off);
        assert!(
            power_off < power_broadside,
            "Off-broadside power {power_off} should be < broadside {power_broadside}"
        );
    }

    #[test]
    fn test_steered_beam() {
        // Steering weights to 30 deg should produce maximum at 30 deg
        let geom = ArrayGeometry::ULA {
            num_elements: 8,
            spacing: 0.5,
        };
        let steer_angle = 30.0_f64.to_radians();
        let w = steering_weights(&geom, 1.0, steer_angle);

        let power_at_steer = cnorm_sq(array_factor(&geom, 1.0, &w, steer_angle));
        let power_at_zero = cnorm_sq(array_factor(&geom, 1.0, &w, 0.0));
        let power_at_neg = cnorm_sq(array_factor(&geom, 1.0, &w, -PI / 3.0));

        assert!(
            power_at_steer > power_at_zero,
            "Steered power {power_at_steer} should exceed broadside {power_at_zero}"
        );
        assert!(
            power_at_steer > power_at_neg,
            "Steered power {power_at_steer} should exceed off-axis {power_at_neg}"
        );
    }

    #[test]
    fn test_beam_pattern_shape() {
        let geom = ArrayGeometry::ULA {
            num_elements: 4,
            spacing: 0.5,
        };
        let w = uniform_weights(4);
        let pattern = beam_pattern(&geom, 1.0, &w, 360);

        assert_eq!(pattern.len(), 360);

        // Pattern peak should be at 0 dB (normalised)
        let peak_db = pattern.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            peak_db.abs() < 1e-6,
            "Peak should be 0 dB, got {peak_db}"
        );

        // All values should be <= 0 dB
        for &(_, db) in &pattern {
            assert!(db <= 1e-6, "Pattern value {db} dB should be <= 0");
        }
    }

    #[test]
    fn test_uniform_weights() {
        let w = uniform_weights(5);
        assert_eq!(w.len(), 5);
        for &(re, im) in &w {
            assert!((re - 0.2).abs() < TOL);
            assert!(im.abs() < TOL);
        }
    }

    #[test]
    fn test_half_power_beamwidth() {
        // Larger array -> narrower beam
        let geom_4 = ArrayGeometry::ULA {
            num_elements: 4,
            spacing: 0.5,
        };
        let geom_16 = ArrayGeometry::ULA {
            num_elements: 16,
            spacing: 0.5,
        };
        let w4 = uniform_weights(4);
        let w16 = uniform_weights(16);

        let bw4 = half_power_beamwidth(&geom_4, 1.0, &w4);
        let bw16 = half_power_beamwidth(&geom_16, 1.0, &w16);

        assert!(bw4 > 0.0, "Beamwidth must be positive");
        assert!(bw16 > 0.0, "Beamwidth must be positive");
        assert!(
            bw16 < bw4,
            "16-element beamwidth ({bw16:.4} rad) should be narrower than 4-element ({bw4:.4} rad)"
        );
    }

    #[test]
    fn test_custom_array() {
        // Custom 3-element L-shaped array
        let geom = ArrayGeometry::Custom {
            positions: vec![(0.0, 0.0), (0.5, 0.0), (0.0, 0.5)],
        };
        assert_eq!(geom.num_elements(), 3);

        let sv = steering_vector(&geom, 1.0, 0.0);
        assert_eq!(sv.len(), 3);
        // Element at origin always has phase 0
        assert!((sv[0].0 - 1.0).abs() < TOL);
        assert!(sv[0].1.abs() < TOL);

        // Should be able to compute array factor
        let w = uniform_weights(3);
        let af = array_factor(&geom, 1.0, &w, 0.0);
        assert!(cmag(af) > 0.0);
    }

    #[test]
    fn test_single_element() {
        // A single-element array should have omnidirectional response
        let geom = ArrayGeometry::ULA {
            num_elements: 1,
            spacing: 0.5,
        };
        let w = uniform_weights(1);

        let p1 = cnorm_sq(array_factor(&geom, 1.0, &w, 0.0));
        let p2 = cnorm_sq(array_factor(&geom, 1.0, &w, PI / 3.0));
        let p3 = cnorm_sq(array_factor(&geom, 1.0, &w, -PI / 2.0));

        assert!(
            (p1 - p2).abs() < TOL,
            "Single element should be omnidirectional"
        );
        assert!(
            (p1 - p3).abs() < TOL,
            "Single element should be omnidirectional"
        );
    }

    #[test]
    fn test_array_response_struct() {
        let geom = ArrayGeometry::ULA {
            num_elements: 8,
            spacing: 0.15,
        };
        let arr = ArrayResponse::new(geom, 0.3);

        // Steer to 0 rad
        let w = arr.steer(0.0);
        assert_eq!(w.len(), 8);

        // Beam pattern
        let pattern = arr.beam_pattern(&w, 180);
        assert_eq!(pattern.len(), 180);

        // Peak should be near 0 dB at theta ~= 0
        let (peak_theta, peak_db) = pattern
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .copied()
            .unwrap();
        assert!(
            peak_db.abs() < 1e-6,
            "Peak should be 0 dB, got {peak_db}"
        );
        assert!(
            peak_theta.abs() < 0.1,
            "Peak should be near broadside, got {peak_theta:.4} rad"
        );
    }
}

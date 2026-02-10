//! MVDR (Capon) Adaptive Beamformer for Array Signal Processing
//!
//! Implements the Minimum Variance Distortionless Response beamformer and a
//! conventional delay-and-sum beamformer for uniform linear arrays (ULA).
//! Complex values are represented as `(f64, f64)` tuples (real, imaginary).
//! No external crate dependencies — uses only `std`.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::mvdr_beamformer::{
//!     MvdrBeamformer, steering_vector, sample_covariance, diagonal_loading,
//! };
//!
//! // Build a 4-element ULA at half-wavelength spacing
//! let mut bf = MvdrBeamformer::new(4, 0.5);
//!
//! // Fabricate a trivial identity covariance (no signals)
//! let mut cov = vec![vec![0.0; 8]; 4]; // 4x4 complex stored as 2*N reals
//! for i in 0..4 {
//!     cov[i][2 * i] = 1.0; // real part of diagonal = 1
//! }
//!
//! // Add diagonal loading for numerical stability
//! diagonal_loading(&mut cov, 0.01);
//!
//! // Compute MVDR weights steered to broadside (0 degrees)
//! let weights = bf.compute_weights(&cov, 0.0);
//! assert_eq!(weights.len(), 4);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex helpers (no external crate)
// ---------------------------------------------------------------------------

/// Multiply two complex numbers represented as `(re, im)`.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Magnitude squared.
#[inline]
fn cmag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ---------------------------------------------------------------------------
// Hermitian‐matrix helpers  (stored as Vec<Vec<f64>> with row‐major packing:
//   row i → [re(i,0), im(i,0), re(i,1), im(i,1), …])
// ---------------------------------------------------------------------------

/// Read element (i,j) from packed complex matrix.
#[inline]
fn mat_get(m: &[Vec<f64>], i: usize, j: usize) -> (f64, f64) {
    (m[i][2 * j], m[i][2 * j + 1])
}

/// Write element (i,j) to packed complex matrix.
#[inline]
fn mat_set(m: &mut [Vec<f64>], i: usize, j: usize, v: (f64, f64)) {
    m[i][2 * j] = v.0;
    m[i][2 * j + 1] = v.1;
}

/// Matrix–vector product  y = M * x  (complex, M is n×n packed).
fn mat_vec(m: &[Vec<f64>], x: &[(f64, f64)], n: usize) -> Vec<(f64, f64)> {
    let mut y = vec![(0.0, 0.0); n];
    for i in 0..n {
        for j in 0..n {
            y[i] = cadd(y[i], cmul(mat_get(m, i, j), x[j]));
        }
    }
    y
}

// ---------------------------------------------------------------------------
// Public free functions
// ---------------------------------------------------------------------------

/// Compute the ULA steering vector for `num_elements` elements separated by
/// `spacing` wavelengths, steered to `angle_deg` (degrees from broadside).
pub fn steering_vector(num_elements: usize, spacing: f64, angle_deg: f64) -> Vec<(f64, f64)> {
    let angle_rad = angle_deg * PI / 180.0;
    let phase_inc = 2.0 * PI * spacing * angle_rad.sin();
    (0..num_elements)
        .map(|k| {
            let phase = k as f64 * phase_inc;
            (phase.cos(), phase.sin())
        })
        .collect()
}

/// Estimate the sample covariance matrix from an array of snapshots.
///
/// Each snapshot is a `Vec<(f64,f64)>` of length `num_elements`.  The result
/// is an `N×N` packed complex matrix (`Vec<Vec<f64>>` with 2N reals per row).
pub fn sample_covariance(snapshots: &[Vec<(f64, f64)>]) -> Vec<Vec<f64>> {
    assert!(!snapshots.is_empty(), "need at least one snapshot");
    let n = snapshots[0].len();
    let k = snapshots.len() as f64;
    let mut cov = vec![vec![0.0; 2 * n]; n];
    for snap in snapshots {
        for i in 0..n {
            for j in 0..n {
                let v = cmul(snap[i], conj(snap[j]));
                cov[i][2 * j] += v.0 / k;
                cov[i][2 * j + 1] += v.1 / k;
            }
        }
    }
    cov
}

/// Invert a complex Hermitian matrix via Gauss–Jordan elimination.
///
/// `matrix` is `n×n` packed (each row has 2n f64s).  Returns the inverse in
/// the same format.
pub fn invert_hermitian(matrix: &[Vec<f64>], n: usize) -> Vec<Vec<f64>> {
    // Augment [A | I]
    let mut aug = vec![vec![0.0; 4 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][2 * j] = matrix[i][2 * j];
            aug[i][2 * j + 1] = matrix[i][2 * j + 1];
        }
        // Identity on the right half
        aug[i][2 * n + 2 * i] = 1.0;
    }

    for col in 0..n {
        // Partial pivoting – pick row with largest magnitude in column `col`
        let mut best_row = col;
        let mut best_mag = 0.0;
        for row in col..n {
            let mag = cmag2((aug[row][2 * col], aug[row][2 * col + 1]));
            if mag > best_mag {
                best_mag = mag;
                best_row = row;
            }
        }
        aug.swap(col, best_row);

        // Scale pivot row
        let pivot = (aug[col][2 * col], aug[col][2 * col + 1]);
        let pivot_inv = {
            let d = cmag2(pivot);
            assert!(d > 1e-30, "singular matrix");
            (pivot.0 / d, -pivot.1 / d)
        };
        for k in 0..4 * n {
            let v = (aug[col][k], 0.0); // need pairs
            // operate on pairs
            let _ = v; // placeholder; do element‐wise below
        }
        // Element‐wise multiply row by pivot_inv (complex)
        {
            let mut new_row = vec![0.0; 4 * n];
            for k in 0..2 * n {
                let val = (aug[col][2 * k], aug[col][2 * k + 1]);
                let prod = cmul(val, pivot_inv);
                new_row[2 * k] = prod.0;
                new_row[2 * k + 1] = prod.1;
            }
            aug[col] = new_row;
        }

        // Eliminate column in other rows
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = (aug[row][2 * col], aug[row][2 * col + 1]);
            for k in 0..2 * n {
                let a_val = (aug[col][2 * k], aug[col][2 * k + 1]);
                let sub = cmul(factor, a_val);
                aug[row][2 * k] -= sub.0;
                aug[row][2 * k + 1] -= sub.1;
            }
        }
    }

    // Extract right half
    let mut inv = vec![vec![0.0; 2 * n]; n];
    for i in 0..n {
        for j in 0..2 * n {
            inv[i][j] = aug[i][2 * n + j];
        }
    }
    inv
}

/// Add diagonal loading `loading` to the covariance matrix (in‐place).
///
/// Adds `loading` to the real part of each diagonal element.
pub fn diagonal_loading(covariance: &mut [Vec<f64>], loading: f64) {
    let n = covariance.len();
    for i in 0..n {
        covariance[i][2 * i] += loading;
    }
}

// ---------------------------------------------------------------------------
// MvdrBeamformer
// ---------------------------------------------------------------------------

/// Minimum Variance Distortionless Response (MVDR / Capon) adaptive beamformer.
#[derive(Debug, Clone)]
pub struct MvdrBeamformer {
    /// Number of antenna elements.
    pub num_elements: usize,
    /// Element spacing in wavelengths.
    pub element_spacing_wavelengths: f64,
}

impl MvdrBeamformer {
    /// Create a new MVDR beamformer for a ULA.
    pub fn new(num_elements: usize, element_spacing_wavelengths: f64) -> Self {
        Self {
            num_elements,
            element_spacing_wavelengths,
        }
    }

    /// Compute MVDR weight vector for the given covariance matrix and
    /// steering angle (degrees from broadside).
    ///
    /// `covariance` is `N×N` packed complex (each row has `2N` f64 values).
    ///
    /// Returns complex weights `Vec<(f64, f64)>` of length `N`.
    pub fn compute_weights(
        &mut self,
        covariance: &[Vec<f64>],
        steering_angle_deg: f64,
    ) -> Vec<(f64, f64)> {
        let n = self.num_elements;
        let a = steering_vector(n, self.element_spacing_wavelengths, steering_angle_deg);
        let r_inv = invert_hermitian(covariance, n);

        // w_unnorm = R^{-1} a
        let w_unnorm = mat_vec(&r_inv, &a, n);

        // denominator = a^H R^{-1} a  (scalar, complex)
        let mut denom = (0.0, 0.0);
        for i in 0..n {
            denom = cadd(denom, cmul(conj(a[i]), w_unnorm[i]));
        }
        let denom_inv = {
            let d = cmag2(denom);
            (denom.0 / d, -denom.1 / d)
        };

        // w = R^{-1} a / (a^H R^{-1} a)
        w_unnorm.iter().map(|&wi| cmul(wi, denom_inv)).collect()
    }

    /// Apply weight vector to each snapshot, producing one complex output per
    /// snapshot:  y[k] = w^H x[k].
    pub fn apply_weights(
        &self,
        weights: &[(f64, f64)],
        snapshots: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        snapshots
            .iter()
            .map(|snap| {
                let mut y = (0.0, 0.0);
                for (w, x) in weights.iter().zip(snap.iter()) {
                    y = cadd(y, cmul(conj(*w), *x));
                }
                y
            })
            .collect()
    }

    /// Compute the Capon (MVDR) spatial spectrum over the given angle range.
    ///
    /// Returns `Vec<(angle_deg, power_dB)>`.
    pub fn spatial_spectrum(
        &mut self,
        covariance: &[Vec<f64>],
        angle_range: (f64, f64),
        angle_step: f64,
    ) -> Vec<(f64, f64)> {
        let n = self.num_elements;
        let r_inv = invert_hermitian(covariance, n);

        let mut result = Vec::new();
        let mut angle = angle_range.0;
        while angle <= angle_range.1 + 1e-9 {
            let a = steering_vector(n, self.element_spacing_wavelengths, angle);
            let r_inv_a = mat_vec(&r_inv, &a, n);
            let mut denom = (0.0, 0.0);
            for i in 0..n {
                denom = cadd(denom, cmul(conj(a[i]), r_inv_a[i]));
            }
            // P(theta) = 1 / (a^H R^{-1} a)  — take real part
            let power = 1.0 / denom.0;
            let power_db = 10.0 * power.abs().log10();
            result.push((angle, power_db));
            angle += angle_step;
        }
        result
    }
}

/// Factory: create a 4-element ULA at half-wavelength spacing.
pub fn ula_4_element() -> MvdrBeamformer {
    MvdrBeamformer::new(4, 0.5)
}

// ---------------------------------------------------------------------------
// ConventionalBeamformer
// ---------------------------------------------------------------------------

/// Conventional (delay-and-sum) beamformer for a ULA.
#[derive(Debug, Clone)]
pub struct ConventionalBeamformer {
    /// Number of antenna elements.
    pub num_elements: usize,
    /// Element spacing in wavelengths.
    pub element_spacing_wavelengths: f64,
}

impl ConventionalBeamformer {
    /// Create a new conventional beamformer.
    pub fn new(num_elements: usize, element_spacing_wavelengths: f64) -> Self {
        Self {
            num_elements,
            element_spacing_wavelengths,
        }
    }

    /// Delay-and-sum beamforming: steer to `angle_deg` and apply to each
    /// snapshot, yielding one complex output per snapshot.
    pub fn beamform(
        &self,
        snapshots: &[Vec<(f64, f64)>],
        angle_deg: f64,
    ) -> Vec<(f64, f64)> {
        let a = steering_vector(self.num_elements, self.element_spacing_wavelengths, angle_deg);
        let n_inv = 1.0 / self.num_elements as f64;
        snapshots
            .iter()
            .map(|snap| {
                let mut y = (0.0, 0.0);
                for (ai, xi) in a.iter().zip(snap.iter()) {
                    y = cadd(y, cmul(conj(*ai), *xi));
                }
                (y.0 * n_inv, y.1 * n_inv)
            })
            .collect()
    }

    /// Compute the array factor (beam pattern) in dB for a list of angles.
    ///
    /// The pattern is normalised so the peak is 0 dB.
    pub fn beam_pattern(&self, angles: &[f64]) -> Vec<f64> {
        let n = self.num_elements as f64;
        let d = self.element_spacing_wavelengths;
        let pattern: Vec<f64> = angles
            .iter()
            .map(|&theta_deg| {
                let theta_rad = theta_deg * PI / 180.0;
                let psi = PI * d * theta_rad.sin();
                // AF = sin(N*psi) / (N * sin(psi))
                let af = if psi.abs() < 1e-12 {
                    1.0
                } else {
                    ((n * psi).sin() / (n * psi.sin())).abs()
                };
                af
            })
            .collect();

        // Normalise to 0 dB peak
        let peak = pattern.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        pattern
            .iter()
            .map(|&p| 20.0 * (p / peak).log10())
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < TOL
    }

    fn capprox_eq(a: (f64, f64), b: (f64, f64)) -> bool {
        approx_eq(a.0, b.0) && approx_eq(a.1, b.1)
    }

    /// Helper: build an identity covariance matrix of size n.
    fn identity_cov(n: usize) -> Vec<Vec<f64>> {
        let mut cov = vec![vec![0.0; 2 * n]; n];
        for i in 0..n {
            cov[i][2 * i] = 1.0;
        }
        cov
    }

    // 1. Steering vector at broadside (0 deg) should be all ones.
    #[test]
    fn test_steering_vector_broadside() {
        let sv = steering_vector(4, 0.5, 0.0);
        assert_eq!(sv.len(), 4);
        for v in &sv {
            assert!(capprox_eq(*v, (1.0, 0.0)), "expected (1,0), got {:?}", v);
        }
    }

    // 2. Steering vector at 30 degrees has correct phase progression.
    #[test]
    fn test_steering_vector_30_degrees() {
        let sv = steering_vector(4, 0.5, 30.0);
        // phase increment = 2*pi*0.5*sin(30deg) = pi*0.5 = pi/2
        let phase_inc = PI / 2.0;
        for (k, v) in sv.iter().enumerate() {
            let expected_phase = k as f64 * phase_inc;
            let expected = (expected_phase.cos(), expected_phase.sin());
            assert!(
                capprox_eq(*v, expected),
                "element {}: expected {:?}, got {:?}",
                k,
                expected,
                v
            );
        }
    }

    // 3. Conventional beamformer delay-and-sum at broadside.
    #[test]
    fn test_conventional_beamformer_delay_and_sum() {
        let cbf = ConventionalBeamformer::new(4, 0.5);
        // All-ones snapshot; beamform at broadside => output should be (1, 0)
        let snapshots = vec![vec![(1.0, 0.0); 4]];
        let out = cbf.beamform(&snapshots, 0.0);
        assert_eq!(out.len(), 1);
        assert!(
            approx_eq(out[0].0, 1.0) && approx_eq(out[0].1, 0.0),
            "expected (1,0), got {:?}",
            out[0]
        );
    }

    // 4. Beam pattern has its peak at broadside (0 degrees).
    #[test]
    fn test_beam_pattern_peak_at_broadside() {
        let cbf = ConventionalBeamformer::new(8, 0.5);
        let angles: Vec<f64> = (-90..=90).map(|a| a as f64).collect();
        let pattern = cbf.beam_pattern(&angles);
        // Find index of maximum (should be at 0 degrees, index 90)
        let (max_idx, _) = pattern
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        let peak_angle = angles[max_idx];
        assert!(
            peak_angle.abs() < 1.0,
            "peak at {} deg, expected ~0",
            peak_angle
        );
    }

    // 5. MVDR weight computation with identity covariance yields equal weights.
    #[test]
    fn test_mvdr_weight_computation() {
        let mut bf = MvdrBeamformer::new(4, 0.5);
        let cov = identity_cov(4);
        let weights = bf.compute_weights(&cov, 0.0);
        assert_eq!(weights.len(), 4);
        // With R = I and broadside steering, w = a / (a^H a) = (1,0)/4 for each element
        for w in &weights {
            assert!(
                approx_eq(w.0, 0.25) && approx_eq(w.1, 0.0),
                "expected (0.25, 0), got {:?}",
                w
            );
        }
    }

    // 6. Spatial spectrum has a peak near the signal direction.
    #[test]
    fn test_spatial_spectrum_peak() {
        let n = 4;
        let spacing = 0.5;
        let signal_angle = 20.0;

        // Create a rank-1 covariance R = a * a^H + 0.01 * I
        let a = steering_vector(n, spacing, signal_angle);
        let mut cov = vec![vec![0.0; 2 * n]; n];
        for i in 0..n {
            for j in 0..n {
                let v = cmul(a[i], conj(a[j]));
                mat_set(&mut cov, i, j, v);
            }
        }
        diagonal_loading(&mut cov, 0.01);

        let mut bf = MvdrBeamformer::new(n, spacing);
        let spectrum = bf.spatial_spectrum(&cov, (-90.0, 90.0), 1.0);

        // Find the peak
        let (peak_angle, _) = spectrum
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();

        assert!(
            (*peak_angle - signal_angle).abs() < 2.0,
            "peak at {} deg, expected ~{}",
            peak_angle,
            signal_angle
        );
    }

    // 7. Sample covariance has correct dimensions.
    #[test]
    fn test_sample_covariance_dimensions() {
        let snapshots = vec![
            vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)],
            vec![(0.5, 0.5), (1.0, 0.0), (0.0, 0.0)],
        ];
        let cov = sample_covariance(&snapshots);
        assert_eq!(cov.len(), 3, "expected 3 rows");
        for row in &cov {
            assert_eq!(row.len(), 6, "expected 6 reals per row (3 complex)");
        }
    }

    // 8. Diagonal loading adds to diagonal elements.
    #[test]
    fn test_diagonal_loading_adds() {
        let n = 3;
        let mut cov = identity_cov(n);
        diagonal_loading(&mut cov, 0.5);
        for i in 0..n {
            let diag_re = cov[i][2 * i];
            assert!(
                approx_eq(diag_re, 1.5),
                "diagonal({}) = {}, expected 1.5",
                i,
                diag_re
            );
            // Off-diag should still be 0
            for j in 0..n {
                if j != i {
                    assert!(
                        approx_eq(cov[i][2 * j], 0.0),
                        "off-diagonal({},{}) real should be 0",
                        i,
                        j
                    );
                }
            }
        }
    }

    // 9. Matrix inversion of a known 2x2 identity gives identity.
    #[test]
    fn test_matrix_inversion_2x2() {
        // A = [[2, 0], [0, 3]]  (real diagonal)
        let n = 2;
        let mut a = vec![vec![0.0; 4]; 2];
        mat_set(&mut a, 0, 0, (2.0, 0.0));
        mat_set(&mut a, 1, 1, (3.0, 0.0));
        let inv = invert_hermitian(&a, n);
        // Expected: [[0.5, 0], [0, 1/3]]
        assert!(
            approx_eq(mat_get(&inv, 0, 0).0, 0.5),
            "inv(0,0) = {:?}, expected 0.5",
            mat_get(&inv, 0, 0)
        );
        assert!(
            approx_eq(mat_get(&inv, 1, 1).0, 1.0 / 3.0),
            "inv(1,1) = {:?}, expected 1/3",
            mat_get(&inv, 1, 1)
        );
        assert!(
            approx_eq(mat_get(&inv, 0, 1).0, 0.0),
            "inv(0,1) should be 0"
        );
        assert!(
            approx_eq(mat_get(&inv, 1, 0).0, 0.0),
            "inv(1,0) should be 0"
        );
    }

    // 10. ULA factory produces correct configuration.
    #[test]
    fn test_ula_4_element_factory() {
        let bf = ula_4_element();
        assert_eq!(bf.num_elements, 4);
        assert!(approx_eq(bf.element_spacing_wavelengths, 0.5));
    }
}

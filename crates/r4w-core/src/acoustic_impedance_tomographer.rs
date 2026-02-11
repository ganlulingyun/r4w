//! Electrical Impedance Tomography (EIT) reconstruction algorithms for medical imaging.
//!
//! EIT measures boundary voltages from injected currents to reconstruct internal
//! conductivity distributions. This module provides back-projection and Tikhonov
//! regularization reconstruction, sensitivity-based Jacobian computation, drive
//! pattern generation, and forward voltage simulation for homogeneous and
//! inclusion phantoms.
//!
//! # Example
//!
//! ```
//! use r4w_core::acoustic_impedance_tomographer::{
//!     EitConfig, EitReconstructor, DrivePattern,
//!     generate_homogeneous_voltages, generate_inclusion_voltages,
//! };
//!
//! let config = EitConfig::default();
//! let reference = generate_homogeneous_voltages(config.num_electrodes, 1.0);
//! let measured = generate_inclusion_voltages(
//!     config.num_electrodes, 1.0, 2.0, (0.3, 0.0), 0.15,
//! );
//! let reconstructor = EitReconstructor::new(config);
//! let image = reconstructor.reconstruct(&measured, &reference);
//! assert_eq!(image.size, 64);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Drive/measurement pattern for electrode excitation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DrivePattern {
    /// Adjacent electrode pairs (e.g., (0,1), (1,2), ...).
    Adjacent,
    /// Opposite electrode pairs (e.g., (0, N/2), (1, N/2+1), ...).
    Opposite,
    /// Trigonometric (sinusoidal) current patterns.
    Trigonometric,
}

/// Configuration for an EIT reconstruction.
#[derive(Debug, Clone)]
pub struct EitConfig {
    /// Number of boundary electrodes (default 16).
    pub num_electrodes: usize,
    /// Reconstructed image side length in pixels (default 64).
    pub image_size: usize,
    /// Tikhonov regularization parameter (default 0.01).
    pub regularization: f64,
    /// Current drive/measurement pattern.
    pub drive_pattern: DrivePattern,
}

impl Default for EitConfig {
    fn default() -> Self {
        Self {
            num_electrodes: 16,
            image_size: 64,
            regularization: 0.01,
            drive_pattern: DrivePattern::Adjacent,
        }
    }
}

// ---------------------------------------------------------------------------
// EIT image
// ---------------------------------------------------------------------------

/// Reconstructed conductivity image.
#[derive(Debug, Clone)]
pub struct EitImage {
    /// 2-D pixel grid, row-major.  `pixels[row][col]`.
    pub pixels: Vec<Vec<f64>>,
    /// Side length (image is `size x size`).
    pub size: usize,
    /// (min, max) conductivity values in the image.
    pub conductivity_range: (f64, f64),
}

impl EitImage {
    /// Create a new zero-valued image of the given size.
    pub fn new(size: usize) -> Self {
        Self {
            pixels: vec![vec![0.0; size]; size],
            size,
            conductivity_range: (0.0, 0.0),
        }
    }

    /// Recompute `conductivity_range` from the pixel data.
    pub fn update_range(&mut self) {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for row in &self.pixels {
            for &v in row {
                if v < lo {
                    lo = v;
                }
                if v > hi {
                    hi = v;
                }
            }
        }
        if lo > hi {
            lo = 0.0;
            hi = 0.0;
        }
        self.conductivity_range = (lo, hi);
    }
}

// ---------------------------------------------------------------------------
// Electrode geometry
// ---------------------------------------------------------------------------

/// Return the (x, y) position of electrode `index` on a circular boundary of
/// given `radius`, centred at the origin.  Electrodes are equally spaced
/// starting at angle 0.
pub fn electrode_position(index: usize, num_electrodes: usize, radius: f64) -> (f64, f64) {
    let angle = 2.0 * PI * (index as f64) / (num_electrodes as f64);
    (radius * angle.cos(), radius * angle.sin())
}

// ---------------------------------------------------------------------------
// Drive patterns
// ---------------------------------------------------------------------------

/// Generate adjacent drive/measurement electrode pairs.
///
/// For `n` electrodes this produces `n` pairs: (0,1), (1,2), ..., (n-1, 0).
pub fn adjacent_drive_pairs(num_electrodes: usize) -> Vec<(usize, usize)> {
    (0..num_electrodes)
        .map(|i| (i, (i + 1) % num_electrodes))
        .collect()
}

/// Generate opposite drive/measurement electrode pairs.
fn opposite_drive_pairs(num_electrodes: usize) -> Vec<(usize, usize)> {
    let half = num_electrodes / 2;
    (0..num_electrodes)
        .map(|i| (i, (i + half) % num_electrodes))
        .collect()
}

/// Generate trigonometric drive indices (pairs of sine-pattern electrodes).
fn trigonometric_drive_pairs(num_electrodes: usize) -> Vec<(usize, usize)> {
    // Use harmonic pairs: for pattern k, drive electrode k and k + N/4 (quadrature).
    let quarter = num_electrodes / 4;
    (0..num_electrodes)
        .map(|i| (i, (i + quarter) % num_electrodes))
        .collect()
}

/// Return drive pairs for the given pattern.
pub fn drive_pairs_for_pattern(pattern: DrivePattern, num_electrodes: usize) -> Vec<(usize, usize)> {
    match pattern {
        DrivePattern::Adjacent => adjacent_drive_pairs(num_electrodes),
        DrivePattern::Opposite => opposite_drive_pairs(num_electrodes),
        DrivePattern::Trigonometric => trigonometric_drive_pairs(num_electrodes),
    }
}

// ---------------------------------------------------------------------------
// Sensitivity / Jacobian
// ---------------------------------------------------------------------------

/// Compute the sensitivity of a `pixel` location to a measurement between
/// `electrode_a` and `electrode_b`.
///
/// Uses the Geselowitz sensitivity theorem approximation:
///   S = (dot(grad_phi1, grad_phi2)) / (r_a^2 * r_b^2)
/// where r_a and r_b are distances from the pixel to each electrode.
///
/// For a unit-disk domain the potential gradient of a point source at
/// electrode position **e** evaluated at pixel **p** is proportional to
/// **(p - e) / |p - e|^2**, so the sensitivity reduces to:
///
///   S = dot(p - e_a, p - e_b) / (|p - e_a|^2 * |p - e_b|^2)
pub fn compute_sensitivity(
    electrode_a: (f64, f64),
    electrode_b: (f64, f64),
    pixel: (f64, f64),
) -> f64 {
    let da = (pixel.0 - electrode_a.0, pixel.1 - electrode_a.1);
    let db = (pixel.0 - electrode_b.0, pixel.1 - electrode_b.1);
    let ra2 = da.0 * da.0 + da.1 * da.1;
    let rb2 = db.0 * db.0 + db.1 * db.1;
    let guard = 1e-12;
    if ra2 < guard || rb2 < guard {
        return 0.0;
    }
    let dot = da.0 * db.0 + da.1 * db.1;
    dot / (ra2 * rb2)
}

/// Build a simplified sensitivity (Jacobian) matrix of shape
/// `[num_measurements x num_pixels]`.
///
/// `num_pixels` is the total number of image pixels (`image_size * image_size`).
/// Each measurement corresponds to an adjacent drive pair; for each drive pair
/// we compute the sensitivity at every pixel.
pub fn compute_jacobian(num_electrodes: usize, num_pixels: usize) -> Vec<Vec<f64>> {
    let image_size = (num_pixels as f64).sqrt().round() as usize;
    let pairs = adjacent_drive_pairs(num_electrodes);
    let radius = 1.0;
    let mut jacobian = Vec::with_capacity(pairs.len());
    for &(a, b) in &pairs {
        let ea = electrode_position(a, num_electrodes, radius);
        let eb = electrode_position(b, num_electrodes, radius);
        let mut row = Vec::with_capacity(num_pixels);
        for r in 0..image_size {
            for c in 0..image_size {
                // Map pixel (r, c) to physical coordinates in [-1, 1].
                let px = 2.0 * (c as f64 + 0.5) / (image_size as f64) - 1.0;
                let py = 2.0 * (r as f64 + 0.5) / (image_size as f64) - 1.0;
                row.push(compute_sensitivity(ea, eb, (px, py)));
            }
        }
        jacobian.push(row);
    }
    jacobian
}

// ---------------------------------------------------------------------------
// Linear algebra helpers (small, self-contained)
// ---------------------------------------------------------------------------

/// Transpose a row-major matrix.
fn mat_transpose(m: &[Vec<f64>]) -> Vec<Vec<f64>> {
    if m.is_empty() {
        return vec![];
    }
    let rows = m.len();
    let cols = m[0].len();
    let mut t = vec![vec![0.0; rows]; cols];
    for i in 0..rows {
        for j in 0..cols {
            t[j][i] = m[i][j];
        }
    }
    t
}

/// Multiply two row-major matrices.
fn mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let m = a.len();
    let n = b[0].len();
    let k = b.len();
    let mut c = vec![vec![0.0; n]; m];
    for i in 0..m {
        for j in 0..n {
            let mut s = 0.0;
            for p in 0..k {
                s += a[i][p] * b[p][j];
            }
            c[i][j] = s;
        }
    }
    c
}

/// Multiply a matrix by a column vector.
fn mat_vec_mul(a: &[Vec<f64>], x: &[f64]) -> Vec<f64> {
    a.iter()
        .map(|row| row.iter().zip(x.iter()).map(|(a, b)| a * b).sum())
        .collect()
}

/// Add `lambda * I` to a square matrix (in place).
fn add_lambda_identity(m: &mut [Vec<f64>], lambda: f64) {
    for i in 0..m.len() {
        m[i][i] += lambda;
    }
}

/// Solve a symmetric positive-definite system `A x = b` using Cholesky
/// decomposition.  Falls back to a simple diagonal solve if decomposition
/// fails (e.g. near-singular).
fn solve_spd(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = a.len();
    // Cholesky: A = L L^T
    let mut l = vec![vec![0.0; n]; n];
    let mut ok = true;
    for i in 0..n {
        for j in 0..=i {
            let s: f64 = (0..j).map(|k| l[i][k] * l[j][k]).sum();
            if i == j {
                let diag = a[i][i] - s;
                if diag <= 0.0 {
                    ok = false;
                    break;
                }
                l[i][j] = diag.sqrt();
            } else {
                l[i][j] = (a[i][j] - s) / l[j][j];
            }
        }
        if !ok {
            break;
        }
    }
    if !ok {
        // Fallback: diagonal solve
        return (0..n)
            .map(|i| {
                let d = a[i][i];
                if d.abs() > 1e-30 {
                    b[i] / d
                } else {
                    0.0
                }
            })
            .collect();
    }
    // Forward substitute: L y = b
    let mut y = vec![0.0; n];
    for i in 0..n {
        let mut s = b[i];
        for j in 0..i {
            s -= l[i][j] * y[j];
        }
        y[i] = s / l[i][i];
    }
    // Back substitute: L^T x = y
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = y[i];
        for j in (i + 1)..n {
            s -= l[j][i] * x[j];
        }
        x[i] = s / l[i][i];
    }
    x
}

// ---------------------------------------------------------------------------
// Reconstruction algorithms
// ---------------------------------------------------------------------------

/// Simple back-projection reconstruction.
///
/// For each drive pair, the voltage difference `(measured - reference)` is
/// projected back along the sensitivity profile onto the image grid.
pub fn back_projection(
    voltages: &[f64],
    reference: &[f64],
    num_electrodes: usize,
    image_size: usize,
) -> Vec<Vec<f64>> {
    let pairs = adjacent_drive_pairs(num_electrodes);
    let n_meas = pairs.len().min(voltages.len()).min(reference.len());
    let radius = 1.0;
    let mut img = vec![vec![0.0; image_size]; image_size];
    for m in 0..n_meas {
        let (a, b) = pairs[m];
        let ea = electrode_position(a, num_electrodes, radius);
        let eb = electrode_position(b, num_electrodes, radius);
        let dv = voltages[m] - reference[m];
        for r in 0..image_size {
            for c in 0..image_size {
                let px = 2.0 * (c as f64 + 0.5) / (image_size as f64) - 1.0;
                let py = 2.0 * (r as f64 + 0.5) / (image_size as f64) - 1.0;
                let s = compute_sensitivity(ea, eb, (px, py));
                img[r][c] += dv * s;
            }
        }
    }
    img
}

/// Tikhonov-regularized least-squares inversion.
///
///   x = (J^T J + lambda * I)^{-1} J^T b
///
/// `jacobian` is `[m x n]`, `measurements` is length `m`, result is length `n`.
pub fn tikhonov_regularization(
    jacobian: &[Vec<f64>],
    measurements: &[f64],
    lambda: f64,
) -> Vec<f64> {
    let jt = mat_transpose(jacobian);
    // J^T * J: jt is (n x m), jacobian is (m x n) -> result is (n x n).
    let mut jtj = mat_mul(&jt, jacobian);
    add_lambda_identity(&mut jtj, lambda);
    let jtb = mat_vec_mul(&jt, measurements);
    solve_spd(&jtj, &jtb)
}

// ---------------------------------------------------------------------------
// Forward voltage simulation
// ---------------------------------------------------------------------------

/// Generate reference boundary voltages for a homogeneous circular domain of
/// given `conductivity`.
///
/// Uses a simplified analytical model: voltage between measurement electrodes
/// is inversely proportional to conductivity and depends on electrode geometry.
pub fn generate_homogeneous_voltages(num_electrodes: usize, conductivity: f64) -> Vec<f64> {
    let pairs = adjacent_drive_pairs(num_electrodes);
    let radius = 1.0;
    let sigma = if conductivity.abs() < 1e-30 {
        1e-30
    } else {
        conductivity
    };
    pairs
        .iter()
        .map(|&(a, b)| {
            let ea = electrode_position(a, num_electrodes, radius);
            let eb = electrode_position(b, num_electrodes, radius);
            let dx = ea.0 - eb.0;
            let dy = ea.1 - eb.1;
            let dist = (dx * dx + dy * dy).sqrt();
            // V = I * R, with R ~ ln(2 pi / (dist * sigma))
            // Simplified: proportional to 1/sigma, modulated by geometry.
            (1.0 / sigma) * (1.0 + 0.5 * dist)
        })
        .collect()
}

/// Generate boundary voltages for a homogeneous medium with a circular
/// inclusion of different conductivity.
///
/// The inclusion is centred at `inclusion_center` (in normalised [-1,1] coords)
/// with the given `inclusion_radius` (same coordinate space).
pub fn generate_inclusion_voltages(
    num_electrodes: usize,
    bg_conductivity: f64,
    inclusion_conductivity: f64,
    inclusion_center: (f64, f64),
    inclusion_radius: f64,
) -> Vec<f64> {
    let pairs = adjacent_drive_pairs(num_electrodes);
    let radius = 1.0;
    let sigma_bg = if bg_conductivity.abs() < 1e-30 {
        1e-30
    } else {
        bg_conductivity
    };
    let sigma_inc = if inclusion_conductivity.abs() < 1e-30 {
        1e-30
    } else {
        inclusion_conductivity
    };

    pairs
        .iter()
        .map(|&(a, b)| {
            let ea = electrode_position(a, num_electrodes, radius);
            let eb = electrode_position(b, num_electrodes, radius);
            let dx = ea.0 - eb.0;
            let dy = ea.1 - eb.1;
            let dist = (dx * dx + dy * dy).sqrt();

            // Base voltage from background.
            let v_bg = (1.0 / sigma_bg) * (1.0 + 0.5 * dist);

            // Perturbation from inclusion: depends on how close the measurement
            // path is to the inclusion.  We approximate the effect by computing
            // the sensitivity at the inclusion centre and scaling by the
            // conductivity contrast.
            let s = compute_sensitivity(ea, eb, inclusion_center);
            let contrast = (sigma_inc - sigma_bg) / sigma_bg;
            let perturbation = s * contrast * inclusion_radius * inclusion_radius * PI;

            v_bg + perturbation
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Image post-processing
// ---------------------------------------------------------------------------

/// Normalise image pixel values to [0, 1].
pub fn normalize_image(image: &mut EitImage) {
    image.update_range();
    let (lo, hi) = image.conductivity_range;
    let span = hi - lo;
    if span.abs() < 1e-30 {
        // Flat image -- set everything to 0.5.
        for row in &mut image.pixels {
            for v in row.iter_mut() {
                *v = 0.5;
            }
        }
    } else {
        for row in &mut image.pixels {
            for v in row.iter_mut() {
                *v = (*v - lo) / span;
            }
        }
    }
    image.conductivity_range = (0.0, 1.0);
}

/// Zero all pixels outside a centred circle of `radius_fraction` of the image
/// half-width (1.0 keeps everything inside the full inscribed circle).
pub fn apply_circular_mask(image: &mut EitImage, radius_fraction: f64) {
    let n = image.size;
    let half = n as f64 / 2.0;
    let r_max = half * radius_fraction;
    let r_max2 = r_max * r_max;
    for r in 0..n {
        for c in 0..n {
            let dx = c as f64 + 0.5 - half;
            let dy = r as f64 + 0.5 - half;
            if dx * dx + dy * dy > r_max2 {
                image.pixels[r][c] = 0.0;
            }
        }
    }
    image.update_range();
}

// ---------------------------------------------------------------------------
// Reconstructor
// ---------------------------------------------------------------------------

/// Main EIT reconstruction engine.
pub struct EitReconstructor {
    config: EitConfig,
    jacobian: Vec<Vec<f64>>,
}

impl EitReconstructor {
    /// Create a new reconstructor for the given configuration.  Pre-computes
    /// the Jacobian matrix.
    pub fn new(config: EitConfig) -> Self {
        let num_pixels = config.image_size * config.image_size;
        let jacobian = compute_jacobian(config.num_electrodes, num_pixels);
        Self { config, jacobian }
    }

    /// Reconstruct a conductivity image from measured and reference voltages.
    ///
    /// Uses Tikhonov-regularized inversion on the precomputed Jacobian.
    pub fn reconstruct(&self, voltages: &[f64], reference_voltages: &[f64]) -> EitImage {
        let n_meas = voltages.len().min(reference_voltages.len());
        let dv: Vec<f64> = voltages
            .iter()
            .zip(reference_voltages.iter())
            .take(n_meas)
            .map(|(m, r)| m - r)
            .collect();

        // Trim Jacobian rows to available measurements.
        let j_trimmed: Vec<Vec<f64>> = self.jacobian.iter().take(n_meas).cloned().collect();

        let x = tikhonov_regularization(&j_trimmed, &dv, self.config.regularization);

        let size = self.config.image_size;
        let mut image = EitImage::new(size);
        for (idx, &val) in x.iter().enumerate() {
            let r = idx / size;
            let c = idx % size;
            if r < size && c < size {
                image.pixels[r][c] = val;
            }
        }
        image.update_range();
        image
    }

    /// Convenience: reconstruct and then normalise + mask.
    pub fn reconstruct_and_postprocess(
        &self,
        voltages: &[f64],
        reference_voltages: &[f64],
        mask_radius: f64,
    ) -> EitImage {
        let mut img = self.reconstruct(voltages, reference_voltages);
        apply_circular_mask(&mut img, mask_radius);
        normalize_image(&mut img);
        img
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPS: f64 = 1e-9;

    // -----------------------------------------------------------------------
    // Electrode positions
    // -----------------------------------------------------------------------

    #[test]
    fn test_electrode_position_first() {
        let (x, y) = electrode_position(0, 16, 1.0);
        assert!((x - 1.0).abs() < EPS, "x should be 1.0, got {x}");
        assert!(y.abs() < EPS, "y should be 0.0, got {y}");
    }

    #[test]
    fn test_electrode_position_quarter() {
        // Electrode at index N/4 should be at angle PI/2 -> (0, 1).
        let (x, y) = electrode_position(4, 16, 1.0);
        assert!(x.abs() < EPS, "x should be ~0, got {x}");
        assert!((y - 1.0).abs() < EPS, "y should be 1.0, got {y}");
    }

    #[test]
    fn test_electrode_position_half() {
        // Electrode at index N/2 -> angle PI -> (-1, 0).
        let (x, y) = electrode_position(8, 16, 1.0);
        assert!((x + 1.0).abs() < EPS, "x should be -1.0, got {x}");
        assert!(y.abs() < EPS, "y should be ~0, got {y}");
    }

    #[test]
    fn test_electrode_position_radius_scaling() {
        let r = 2.5;
        let (x, y) = electrode_position(0, 16, r);
        assert!((x - r).abs() < EPS);
        assert!(y.abs() < EPS);
    }

    #[test]
    fn test_electrode_positions_all_on_circle() {
        let n = 32;
        let r = 1.0;
        for i in 0..n {
            let (x, y) = electrode_position(i, n, r);
            let d = (x * x + y * y).sqrt();
            assert!((d - r).abs() < 1e-12, "electrode {i} off circle: d={d}");
        }
    }

    // -----------------------------------------------------------------------
    // Drive patterns
    // -----------------------------------------------------------------------

    #[test]
    fn test_adjacent_drive_pairs_count() {
        let pairs = adjacent_drive_pairs(16);
        assert_eq!(pairs.len(), 16);
    }

    #[test]
    fn test_adjacent_drive_pairs_wrap() {
        let pairs = adjacent_drive_pairs(8);
        assert_eq!(pairs[7], (7, 0), "last pair should wrap to 0");
    }

    #[test]
    fn test_adjacent_drive_pairs_first() {
        let pairs = adjacent_drive_pairs(16);
        assert_eq!(pairs[0], (0, 1));
    }

    #[test]
    fn test_opposite_drive_pairs_half_offset() {
        let pairs = opposite_drive_pairs(8);
        assert_eq!(pairs[0], (0, 4));
        assert_eq!(pairs[3], (3, 7));
    }

    #[test]
    fn test_trigonometric_drive_pairs_quarter_offset() {
        let pairs = trigonometric_drive_pairs(16);
        assert_eq!(pairs[0], (0, 4));
        assert_eq!(pairs[5], (5, 9));
    }

    #[test]
    fn test_drive_pairs_for_pattern_dispatches() {
        let a = drive_pairs_for_pattern(DrivePattern::Adjacent, 8);
        assert_eq!(a.len(), 8);
        let o = drive_pairs_for_pattern(DrivePattern::Opposite, 8);
        assert_eq!(o.len(), 8);
        let t = drive_pairs_for_pattern(DrivePattern::Trigonometric, 8);
        assert_eq!(t.len(), 8);
    }

    // -----------------------------------------------------------------------
    // Sensitivity
    // -----------------------------------------------------------------------

    #[test]
    fn test_sensitivity_symmetric() {
        let ea = (1.0, 0.0);
        let eb = (0.0, 1.0);
        let px = (0.3, 0.3);
        let s1 = compute_sensitivity(ea, eb, px);
        let s2 = compute_sensitivity(eb, ea, px);
        assert!((s1 - s2).abs() < EPS, "sensitivity should be symmetric");
    }

    #[test]
    fn test_sensitivity_at_electrode_returns_zero() {
        let ea = (1.0, 0.0);
        let eb = (0.0, 1.0);
        let s = compute_sensitivity(ea, eb, ea);
        // Should be zero due to guard
        assert!(s.abs() < 1e-6, "sensitivity at electrode should be ~0");
    }

    #[test]
    fn test_sensitivity_origin() {
        // At origin, vectors to both electrodes are antiparallel for adjacent
        // electrodes at (1,0) and (0,1) => dot product is negative.
        let ea = (1.0, 0.0);
        let eb = (-1.0, 0.0);
        let s = compute_sensitivity(ea, eb, (0.0, 0.0));
        // da = (-1,0), db = (1,0), dot = -1, ra2 = rb2 = 1 => s = -1
        assert!((s + 1.0).abs() < EPS, "expected -1.0, got {s}");
    }

    // -----------------------------------------------------------------------
    // Jacobian
    // -----------------------------------------------------------------------

    #[test]
    fn test_jacobian_dimensions() {
        let j = compute_jacobian(8, 16); // 8 electrodes, 4x4 image
        assert_eq!(j.len(), 8, "should have 8 measurement rows");
        assert_eq!(j[0].len(), 16, "should have 16 pixel columns");
    }

    #[test]
    fn test_jacobian_not_all_zero() {
        let j = compute_jacobian(8, 16);
        let sum: f64 = j.iter().flat_map(|r| r.iter()).map(|v| v.abs()).sum();
        assert!(sum > 0.0, "Jacobian should have nonzero entries");
    }

    // -----------------------------------------------------------------------
    // Homogeneous voltages
    // -----------------------------------------------------------------------

    #[test]
    fn test_homogeneous_voltages_count() {
        let v = generate_homogeneous_voltages(16, 1.0);
        assert_eq!(v.len(), 16);
    }

    #[test]
    fn test_homogeneous_voltages_positive() {
        let v = generate_homogeneous_voltages(16, 1.0);
        for &val in &v {
            assert!(val > 0.0, "voltages should be positive, got {val}");
        }
    }

    #[test]
    fn test_homogeneous_voltages_conductivity_inverse() {
        let v1 = generate_homogeneous_voltages(16, 1.0);
        let v2 = generate_homogeneous_voltages(16, 2.0);
        // Higher conductivity -> lower voltages (Ohm's law).
        for (a, b) in v1.iter().zip(v2.iter()) {
            assert!(a > b, "v(sigma=1) > v(sigma=2) expected: {a} > {b}");
        }
    }

    #[test]
    fn test_homogeneous_voltages_uniform_pattern() {
        // Adjacent pattern on a circle: all pairs are geometrically identical,
        // so all voltages should be equal.
        let v = generate_homogeneous_voltages(16, 1.0);
        let first = v[0];
        for &val in &v[1..] {
            assert!(
                (val - first).abs() < 1e-12,
                "all adjacent-pair voltages should be equal for homogeneous medium"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Inclusion voltages
    // -----------------------------------------------------------------------

    #[test]
    fn test_inclusion_voltages_differ_from_homogeneous() {
        let ref_v = generate_homogeneous_voltages(16, 1.0);
        let inc_v = generate_inclusion_voltages(16, 1.0, 3.0, (0.5, 0.0), 0.2);
        let diff: f64 = ref_v
            .iter()
            .zip(inc_v.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(diff > 1e-6, "inclusion should perturb voltages");
    }

    #[test]
    fn test_inclusion_at_zero_contrast_equals_homogeneous() {
        let ref_v = generate_homogeneous_voltages(16, 1.0);
        let inc_v = generate_inclusion_voltages(16, 1.0, 1.0, (0.5, 0.0), 0.2);
        for (a, b) in ref_v.iter().zip(inc_v.iter()) {
            assert!(
                (a - b).abs() < 1e-12,
                "zero contrast should match homogeneous"
            );
        }
    }

    #[test]
    fn test_inclusion_voltages_count() {
        let v = generate_inclusion_voltages(8, 1.0, 2.0, (0.0, 0.0), 0.1);
        assert_eq!(v.len(), 8);
    }

    // -----------------------------------------------------------------------
    // Back-projection
    // -----------------------------------------------------------------------

    #[test]
    fn test_back_projection_dimensions() {
        let ref_v = generate_homogeneous_voltages(8, 1.0);
        let meas_v = generate_inclusion_voltages(8, 1.0, 2.0, (0.3, 0.0), 0.15);
        let img = back_projection(&meas_v, &ref_v, 8, 16);
        assert_eq!(img.len(), 16);
        assert_eq!(img[0].len(), 16);
    }

    #[test]
    fn test_back_projection_zero_difference_is_zero() {
        let v = generate_homogeneous_voltages(8, 1.0);
        let img = back_projection(&v, &v, 8, 16);
        let sum: f64 = img.iter().flat_map(|r| r.iter()).map(|v| v.abs()).sum();
        assert!(sum < 1e-12, "zero difference should produce zero image");
    }

    #[test]
    fn test_back_projection_nonzero_for_inclusion() {
        let ref_v = generate_homogeneous_voltages(16, 1.0);
        let inc_v = generate_inclusion_voltages(16, 1.0, 3.0, (0.4, 0.0), 0.2);
        let img = back_projection(&inc_v, &ref_v, 16, 32);
        let sum: f64 = img.iter().flat_map(|r| r.iter()).map(|v| v.abs()).sum();
        assert!(sum > 1e-6, "inclusion should produce nonzero image");
    }

    // -----------------------------------------------------------------------
    // Tikhonov regularization
    // -----------------------------------------------------------------------

    #[test]
    fn test_tikhonov_identity_jacobian() {
        // J = I(3x3), b = [1,2,3], lambda = 0
        // x = (I + 0*I)^{-1} * I * b = b
        let j = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let b = vec![1.0, 2.0, 3.0];
        let x = tikhonov_regularization(&j, &b, 0.0);
        assert_eq!(x.len(), 3);
        for i in 0..3 {
            assert!(
                (x[i] - b[i]).abs() < 1e-6,
                "x[{i}]={} expected {}",
                x[i],
                b[i]
            );
        }
    }

    #[test]
    fn test_tikhonov_with_regularization() {
        // With lambda > 0 the solution is damped towards zero.
        let j = vec![
            vec![1.0, 0.0],
            vec![0.0, 1.0],
        ];
        let b = vec![4.0, 6.0];
        let x_noreg = tikhonov_regularization(&j, &b, 0.0);
        let x_reg = tikhonov_regularization(&j, &b, 1.0);
        for i in 0..2 {
            assert!(
                x_reg[i].abs() < x_noreg[i].abs(),
                "regularization should shrink solution"
            );
        }
    }

    #[test]
    fn test_tikhonov_overdetermined() {
        // 3 measurements, 2 unknowns.
        let j = vec![
            vec![1.0, 0.0],
            vec![0.0, 1.0],
            vec![1.0, 1.0],
        ];
        let b = vec![2.0, 3.0, 5.0];
        let x = tikhonov_regularization(&j, &b, 0.001);
        // Least-squares solution should be close to [2, 3].
        assert!((x[0] - 2.0).abs() < 0.5, "x[0] ~ 2.0, got {}", x[0]);
        assert!((x[1] - 3.0).abs() < 0.5, "x[1] ~ 3.0, got {}", x[1]);
    }

    // -----------------------------------------------------------------------
    // EitImage and post-processing
    // -----------------------------------------------------------------------

    #[test]
    fn test_eit_image_new() {
        let img = EitImage::new(8);
        assert_eq!(img.size, 8);
        assert_eq!(img.pixels.len(), 8);
        assert_eq!(img.pixels[0].len(), 8);
    }

    #[test]
    fn test_normalize_image_scales_to_01() {
        let mut img = EitImage::new(4);
        img.pixels[0][0] = -2.0;
        img.pixels[3][3] = 5.0;
        normalize_image(&mut img);
        assert!((img.pixels[0][0] - 0.0).abs() < EPS, "min should map to 0");
        assert!((img.pixels[3][3] - 1.0).abs() < EPS, "max should map to 1");
    }

    #[test]
    fn test_normalize_flat_image() {
        let mut img = EitImage::new(4);
        // All zeros.
        normalize_image(&mut img);
        // Should become all 0.5.
        for row in &img.pixels {
            for &v in row {
                assert!((v - 0.5).abs() < EPS, "flat image should normalise to 0.5");
            }
        }
    }

    #[test]
    fn test_circular_mask_corners_zeroed() {
        let mut img = EitImage::new(8);
        for row in &mut img.pixels {
            for v in row.iter_mut() {
                *v = 1.0;
            }
        }
        apply_circular_mask(&mut img, 1.0);
        // Corner pixel (0,0) should be zeroed (outside inscribed circle).
        assert!(
            img.pixels[0][0].abs() < EPS,
            "corner should be masked: {}",
            img.pixels[0][0]
        );
        // Centre pixel should remain 1.0.
        let mid = 4;
        assert!(
            (img.pixels[mid][mid] - 1.0).abs() < EPS,
            "centre should remain 1.0"
        );
    }

    #[test]
    fn test_circular_mask_small_radius() {
        let mut img = EitImage::new(16);
        for row in &mut img.pixels {
            for v in row.iter_mut() {
                *v = 1.0;
            }
        }
        apply_circular_mask(&mut img, 0.1);
        // Most pixels should be zeroed.
        let total: f64 = img.pixels.iter().flat_map(|r| r.iter()).sum();
        let max_possible = 16.0 * 16.0;
        assert!(
            total < max_possible * 0.1,
            "small mask should zero most pixels"
        );
    }

    // -----------------------------------------------------------------------
    // Reconstructor end-to-end
    // -----------------------------------------------------------------------

    #[test]
    fn test_reconstructor_image_size() {
        let config = EitConfig {
            num_electrodes: 8,
            image_size: 8,
            regularization: 0.01,
            drive_pattern: DrivePattern::Adjacent,
        };
        let recon = EitReconstructor::new(config);
        let ref_v = generate_homogeneous_voltages(8, 1.0);
        let inc_v = generate_inclusion_voltages(8, 1.0, 2.0, (0.3, 0.0), 0.15);
        let img = recon.reconstruct(&inc_v, &ref_v);
        assert_eq!(img.size, 8);
        assert_eq!(img.pixels.len(), 8);
        assert_eq!(img.pixels[0].len(), 8);
    }

    #[test]
    fn test_reconstructor_homogeneous_is_near_zero() {
        let config = EitConfig {
            num_electrodes: 8,
            image_size: 8,
            regularization: 0.01,
            drive_pattern: DrivePattern::Adjacent,
        };
        let recon = EitReconstructor::new(config);
        let v = generate_homogeneous_voltages(8, 1.0);
        let img = recon.reconstruct(&v, &v);
        let sum: f64 = img
            .pixels
            .iter()
            .flat_map(|r| r.iter())
            .map(|v| v.abs())
            .sum();
        assert!(sum < 1e-10, "same voltages should give near-zero image");
    }

    #[test]
    fn test_reconstructor_detects_inclusion() {
        let config = EitConfig {
            num_electrodes: 16,
            image_size: 16,
            regularization: 0.01,
            drive_pattern: DrivePattern::Adjacent,
        };
        let recon = EitReconstructor::new(config);
        let ref_v = generate_homogeneous_voltages(16, 1.0);
        let inc_v = generate_inclusion_voltages(16, 1.0, 3.0, (0.4, 0.0), 0.2);
        let img = recon.reconstruct(&inc_v, &ref_v);
        let sum: f64 = img
            .pixels
            .iter()
            .flat_map(|r| r.iter())
            .map(|v| v.abs())
            .sum();
        assert!(sum > 1e-6, "inclusion should be detected in reconstruction");
    }

    #[test]
    fn test_reconstruct_and_postprocess() {
        let config = EitConfig {
            num_electrodes: 8,
            image_size: 8,
            regularization: 0.01,
            drive_pattern: DrivePattern::Adjacent,
        };
        let recon = EitReconstructor::new(config);
        let ref_v = generate_homogeneous_voltages(8, 1.0);
        let inc_v = generate_inclusion_voltages(8, 1.0, 2.0, (0.3, 0.0), 0.15);
        let img = recon.reconstruct_and_postprocess(&inc_v, &ref_v, 0.9);
        // All values should be in [0, 1] after normalisation.
        for row in &img.pixels {
            for &v in row {
                assert!(v >= -EPS && v <= 1.0 + EPS, "pixel out of [0,1]: {v}");
            }
        }
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_config_default() {
        let c = EitConfig::default();
        assert_eq!(c.num_electrodes, 16);
        assert_eq!(c.image_size, 64);
        assert!((c.regularization - 0.01).abs() < EPS);
        assert_eq!(c.drive_pattern, DrivePattern::Adjacent);
    }

    #[test]
    fn test_empty_voltages() {
        let img = back_projection(&[], &[], 8, 8);
        let sum: f64 = img.iter().flat_map(|r| r.iter()).map(|v| v.abs()).sum();
        assert!(sum < EPS, "empty voltages should give zero image");
    }

    #[test]
    fn test_two_electrodes() {
        // Minimal case: 2 electrodes.
        let v = generate_homogeneous_voltages(2, 1.0);
        assert_eq!(v.len(), 2);
    }

    #[test]
    fn test_update_range() {
        let mut img = EitImage::new(4);
        img.pixels[1][2] = -3.0;
        img.pixels[0][0] = 7.0;
        img.update_range();
        assert!((img.conductivity_range.0 + 3.0).abs() < EPS);
        assert!((img.conductivity_range.1 - 7.0).abs() < EPS);
    }

    #[test]
    fn test_large_regularization_shrinks_solution() {
        let j = vec![
            vec![1.0, 0.0],
            vec![0.0, 1.0],
        ];
        let b = vec![10.0, 10.0];
        let x_small = tikhonov_regularization(&j, &b, 0.001);
        let x_large = tikhonov_regularization(&j, &b, 100.0);
        let norm_small: f64 = x_small.iter().map(|v| v * v).sum();
        let norm_large: f64 = x_large.iter().map(|v| v * v).sum();
        assert!(
            norm_large < norm_small,
            "large lambda should shrink more: {norm_large} < {norm_small}"
        );
    }
}

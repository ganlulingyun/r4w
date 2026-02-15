//! Full Tensor Gravity (FTG) gradiometer data processing for geophysical exploration and geodesy.
//!
//! This module implements comprehensive processing of gravity gradient tensor data
//! from FTG gradiometers used in airborne, marine, and borehole surveys for mineral
//! exploration, oil/gas prospecting, and geodetic applications.
//!
//! # Overview
//!
//! - **GravityGradientTensor**: 3x3 symmetric tensor in Eotvos (1 E = 10^-9 s^-2)
//! - **Forward modeling**: Point mass, sphere, infinite horizontal cylinder, rectangular prism, vertical cylinder
//! - **Tensor analysis**: Eigenvalue decomposition (Jacobi), principal axes, dimensionality index
//! - **Euler deconvolution**: Source location from gradient components with structural index
//! - **Curvature attributes**: Maximum/minimum/Gaussian/mean curvature
//! - **Noise reduction**: Low-pass filtering, Laplace enforcement, leveling corrections
//! - **Terrain correction**: Bouguer slab, free-air gradient
//! - **Survey processing**: Along-line/cross-line gradients, grid interpolation, tilt angle
//!
//! # Units
//!
//! Gravity gradients are in Eotvos (E), where 1 E = 10^-9 s^-2.
//! Gravity is in mGal, where 1 mGal = 10^-5 m/s^2.
//!
//! # Example
//!
//! ```
//! use r4w_core::gravitational_gradient_tensor_processor::{
//!     GravityGradientTensor, GradientProcessor, GradientProcessorConfig,
//!     forward_point_mass,
//! };
//!
//! let config = GradientProcessorConfig {
//!     sample_rate_hz: 10.0,
//!     filter_bandwidth_hz: 1.0,
//!     noise_level_eotvos: 5.0,
//! };
//! let processor = GradientProcessor::new(config);
//!
//! // Forward model: point mass 1e10 kg at (0, 0, -500) m observed at origin
//! let tensor = forward_point_mass(1e10, 0.0, 0.0, 500.0);
//! assert!(tensor.laplace_residual().abs() < 1e-6);
//! ```

use std::f64::consts::PI;

/// Gravitational constant G in m^3 kg^-1 s^-2.
const G: f64 = 6.674_30e-11;

/// Conversion: 1 Eotvos = 1e-9 s^-2.
const EOTVOS_TO_SI: f64 = 1e-9;

/// Free-air gradient: 0.3086 mGal/m ≈ 0.3086 E per meter of elevation.
const FREE_AIR_GRADIENT_MGAL_PER_M: f64 = 0.3086;

// ─────────────────────────────────────────────────────────────────────
// Gravity Gradient Tensor
// ─────────────────────────────────────────────────────────────────────

/// A 3x3 symmetric gravity gradient tensor.
///
/// The tensor T_ij = ∂²U/∂x_i∂x_j where U is the gravitational potential.
/// In free-air (no mass at the measurement point), the Laplace condition holds:
/// Txx + Tyy + Tzz = 0, leaving only 5 independent components.
///
/// Components are stored in Eotvos (E).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravityGradientTensor {
    /// Txx component (E).
    pub txx: f64,
    /// Tyy component (E).
    pub tyy: f64,
    /// Tzz component (E).
    pub tzz: f64,
    /// Txy = Tyx component (E).
    pub txy: f64,
    /// Txz = Tzx component (E).
    pub txz: f64,
    /// Tyz = Tzy component (E).
    pub tyz: f64,
}

impl GravityGradientTensor {
    /// Create a new tensor from all 6 independent components.
    pub fn new(txx: f64, tyy: f64, tzz: f64, txy: f64, txz: f64, tyz: f64) -> Self {
        Self { txx, tyy, tzz, txy, txz, tyz }
    }

    /// Create a tensor from 5 independent components, enforcing Laplace condition.
    /// Tzz is computed as -(Txx + Tyy).
    pub fn from_independent(txx: f64, tyy: f64, txy: f64, txz: f64, tyz: f64) -> Self {
        Self {
            txx,
            tyy,
            tzz: -(txx + tyy),
            txy,
            txz,
            tyz,
        }
    }

    /// Create a zero tensor.
    pub fn zero() -> Self {
        Self { txx: 0.0, tyy: 0.0, tzz: 0.0, txy: 0.0, txz: 0.0, tyz: 0.0 }
    }

    /// Return the tensor as a 3x3 array.
    pub fn as_matrix(&self) -> [[f64; 3]; 3] {
        [
            [self.txx, self.txy, self.txz],
            [self.txy, self.tyy, self.tyz],
            [self.txz, self.tyz, self.tzz],
        ]
    }

    /// Trace: Txx + Tyy + Tzz. Should be zero for Laplace condition.
    pub fn trace(&self) -> f64 {
        self.txx + self.tyy + self.tzz
    }

    /// Laplace residual: |Txx + Tyy + Tzz|. Ideally zero in free-air.
    pub fn laplace_residual(&self) -> f64 {
        self.trace().abs()
    }

    /// Check if Laplace condition is satisfied within tolerance (Eotvos).
    pub fn satisfies_laplace(&self, tolerance_eotvos: f64) -> bool {
        self.laplace_residual() < tolerance_eotvos
    }

    /// Enforce Laplace condition by adjusting Tzz = -(Txx + Tyy).
    pub fn enforce_laplace(&mut self) {
        self.tzz = -(self.txx + self.tyy);
    }

    /// First invariant I0 = trace (should be ~0).
    pub fn invariant_i0(&self) -> f64 {
        self.trace()
    }

    /// Second invariant I1 = Txx*Tyy + Tyy*Tzz + Txx*Tzz - Txy² - Txz² - Tyz².
    pub fn invariant_i1(&self) -> f64 {
        self.txx * self.tyy + self.tyy * self.tzz + self.txx * self.tzz
            - self.txy * self.txy
            - self.txz * self.txz
            - self.tyz * self.tyz
    }

    /// Third invariant I2 = determinant of the tensor matrix.
    pub fn invariant_i2(&self) -> f64 {
        let m = self.as_matrix();
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Dimensionality index D = -(27/4) * I2² / I1³  (Pedersen & Rasmussen, 1990).
    ///
    /// Ranges from 0 to 1:
    /// - D ≈ 0: quasi-2D (line/pipe source)
    /// - D ≈ 0.5: intermediate geometry
    /// - D ≈ 1: quasi-0D (point/sphere source)
    ///
    /// Returns None if I1 ≈ 0.
    pub fn dimensionality_index(&self) -> Option<f64> {
        let i1 = self.invariant_i1();
        let i2 = self.invariant_i2();
        if i1.abs() < 1e-30 {
            return None;
        }
        let i1_cubed = i1 * i1 * i1;
        // D = -(27/4) * I2^2 / I1^3, clamped to [0, 1]
        let d = -(27.0 / 4.0) * i2 * i2 / i1_cubed;
        Some(d.clamp(0.0, 1.0))
    }

    /// Horizontal gradient magnitude: sqrt(Txz² + Tyz²).
    pub fn horizontal_gradient_magnitude(&self) -> f64 {
        (self.txz * self.txz + self.tyz * self.tyz).sqrt()
    }

    /// Tilt angle: theta = atan2(Tzz, sqrt(Txz² + Tyz²)) in radians.
    pub fn tilt_angle(&self) -> f64 {
        let hgm = self.horizontal_gradient_magnitude();
        self.tzz.atan2(hgm)
    }

    /// Tilt angle in degrees.
    pub fn tilt_angle_deg(&self) -> f64 {
        self.tilt_angle().to_degrees()
    }

    /// Frobenius norm: sqrt(sum of squares of all 9 entries) accounting for symmetry.
    pub fn frobenius_norm(&self) -> f64 {
        (self.txx * self.txx
            + self.tyy * self.tyy
            + self.tzz * self.tzz
            + 2.0 * self.txy * self.txy
            + 2.0 * self.txz * self.txz
            + 2.0 * self.tyz * self.tyz)
            .sqrt()
    }

    /// Add two tensors component-wise.
    pub fn add(&self, other: &Self) -> Self {
        Self {
            txx: self.txx + other.txx,
            tyy: self.tyy + other.tyy,
            tzz: self.tzz + other.tzz,
            txy: self.txy + other.txy,
            txz: self.txz + other.txz,
            tyz: self.tyz + other.tyz,
        }
    }

    /// Scale tensor by a scalar.
    pub fn scale(&self, factor: f64) -> Self {
        Self {
            txx: self.txx * factor,
            tyy: self.tyy * factor,
            tzz: self.tzz * factor,
            txy: self.txy * factor,
            txz: self.txz * factor,
            tyz: self.tyz * factor,
        }
    }

    /// Eigenvalue decomposition of the 3x3 symmetric tensor using Jacobi iteration.
    ///
    /// Returns (eigenvalues, eigenvectors) where eigenvalues are sorted in descending order
    /// and eigenvectors are column vectors stored as [[col0], [col1], [col2]].
    pub fn eigendecomposition(&self) -> ([f64; 3], [[f64; 3]; 3]) {
        jacobi_eigendecomposition_3x3(self.as_matrix())
    }

    /// Principal eigenvalues sorted in descending order: lambda1 >= lambda2 >= lambda3.
    pub fn principal_eigenvalues(&self) -> [f64; 3] {
        self.eigendecomposition().0
    }

    /// Maximum curvature: largest eigenvalue of horizontal gradient sub-tensor.
    ///
    /// The horizontal gradient tensor is the 2x2 sub-matrix [[Txx, Txy], [Txy, Tyy]].
    pub fn maximum_curvature(&self) -> f64 {
        let (e1, e2) = eigenvalues_2x2_symmetric(self.txx, self.txy, self.tyy);
        e1.max(e2)
    }

    /// Minimum curvature: smallest eigenvalue of horizontal gradient sub-tensor.
    pub fn minimum_curvature(&self) -> f64 {
        let (e1, e2) = eigenvalues_2x2_symmetric(self.txx, self.txy, self.tyy);
        e1.min(e2)
    }

    /// Gaussian curvature: product of eigenvalues of horizontal gradient sub-tensor.
    pub fn gaussian_curvature(&self) -> f64 {
        self.txx * self.tyy - self.txy * self.txy
    }

    /// Mean curvature: average of eigenvalues of horizontal gradient sub-tensor.
    pub fn mean_curvature(&self) -> f64 {
        0.5 * (self.txx + self.tyy)
    }

    /// Shape index from curvature eigenvalues, range [-1, 1].
    /// Indicates shape: -1=bowl, -0.5=valley, 0=saddle, +0.5=ridge, +1=dome.
    pub fn shape_index(&self) -> Option<f64> {
        let kmax = self.maximum_curvature();
        let kmin = self.minimum_curvature();
        let diff = kmax - kmin;
        if diff.abs() < 1e-30 {
            return None;
        }
        Some(-(2.0 / PI) * ((kmax + kmin) / diff).atan())
    }
}

// ─────────────────────────────────────────────────────────────────────
// Forward modeling functions
// ─────────────────────────────────────────────────────────────────────

/// Forward model: gravity gradient tensor from a point mass.
///
/// The mass `m` (kg) is located at (0, 0, -depth) below the observation point at origin.
/// `depth` is positive downward in meters.
///
/// Tij = G * m * (3*ri*rj / r^5 - delta_ij / r^3) converted to Eotvos.
///
/// For a point directly below: r = (0, 0, depth), |r| = depth.
pub fn forward_point_mass(mass_kg: f64, x_offset: f64, y_offset: f64, depth: f64) -> GravityGradientTensor {
    let rx = x_offset;
    let ry = y_offset;
    let rz = depth; // positive downward
    let r2 = rx * rx + ry * ry + rz * rz;
    if r2 < 1e-30 {
        return GravityGradientTensor::zero();
    }
    let r = r2.sqrt();
    let r3 = r2 * r;
    let r5 = r3 * r2;
    let gm = G * mass_kg;

    // Tij = G*m*(3*ri*rj/r^5 - delta_ij/r^3), convert from s^-2 to Eotvos (1E = 1e-9 s^-2)
    let to_eotvos = 1.0 / EOTVOS_TO_SI;

    let txx = gm * (3.0 * rx * rx / r5 - 1.0 / r3) * to_eotvos;
    let tyy = gm * (3.0 * ry * ry / r5 - 1.0 / r3) * to_eotvos;
    let tzz = gm * (3.0 * rz * rz / r5 - 1.0 / r3) * to_eotvos;
    let txy = gm * (3.0 * rx * ry / r5) * to_eotvos;
    let txz = gm * (3.0 * rx * rz / r5) * to_eotvos;
    let tyz = gm * (3.0 * ry * rz / r5) * to_eotvos;

    GravityGradientTensor::new(txx, tyy, tzz, txy, txz, tyz)
}

/// Forward model: gravity gradient tensor from a uniform sphere.
///
/// Equivalent to a point mass at the sphere center for external observations.
/// `mass_kg` = (4/3)*pi*radius^3 * density.
pub fn forward_sphere(
    density_kg_m3: f64,
    radius_m: f64,
    x_offset: f64,
    y_offset: f64,
    depth: f64,
) -> GravityGradientTensor {
    let mass = (4.0 / 3.0) * PI * radius_m.powi(3) * density_kg_m3;
    forward_point_mass(mass, x_offset, y_offset, depth)
}

/// Forward model: gravity gradient tensor from an infinite horizontal cylinder.
///
/// A 2D line source along the y-axis with linear mass density `lambda` (kg/m).
/// Observed at offset (x_offset, depth) from the cylinder axis.
///
/// For a line source along y: only Txx, Tzz, Txz are non-zero (Tyy = 0 in 2D).
pub fn forward_infinite_cylinder(
    density_kg_m3: f64,
    radius_m: f64,
    x_offset: f64,
    depth: f64,
) -> GravityGradientTensor {
    let lambda = PI * radius_m * radius_m * density_kg_m3; // mass per unit length
    let rx = x_offset;
    let rz = depth;
    let r2 = rx * rx + rz * rz;
    if r2 < 1e-30 {
        return GravityGradientTensor::zero();
    }
    let r4 = r2 * r2;
    let to_eotvos = 1.0 / EOTVOS_TO_SI;
    let coeff = 2.0 * G * lambda * to_eotvos;

    // 2D gradient: Tij = 2*G*lambda*(2*ri*rj/r^4 - delta_ij/r^2), for i,j in {x,z}
    let txx = coeff * (2.0 * rx * rx / r4 - 1.0 / r2);
    let tzz = coeff * (2.0 * rz * rz / r4 - 1.0 / r2);
    let txz = coeff * (2.0 * rx * rz / r4);
    // Tyy = -(Txx + Tzz) from Laplace in 3D, but for infinite cylinder Tyy = 0
    // and Laplace is Txx + Tzz = 0 in 2D.
    let tyy = 0.0;

    GravityGradientTensor::new(txx, tyy, tzz, 0.0, txz, 0.0)
}

/// Forward model: gravity gradient tensor from a rectangular prism (right rectangular parallelepiped).
///
/// The prism has corners at (x1,y1,z1) to (x2,y2,z2), observed at origin.
/// Uses the analytical formula with arctangent and logarithm terms.
/// `density_kg_m3` is the density contrast.
pub fn forward_rectangular_prism(
    density_kg_m3: f64,
    x1: f64, x2: f64,
    y1: f64, y2: f64,
    z1: f64, z2: f64,
) -> GravityGradientTensor {
    let to_eotvos = 1.0 / EOTVOS_TO_SI;
    let coeff = G * density_kg_m3 * to_eotvos;

    let xs = [x1, x2];
    let ys = [y1, y2];
    let zs = [z1, z2];

    let mut txx = 0.0;
    let mut tyy = 0.0;
    let mut tzz = 0.0;
    let mut txy = 0.0;
    let mut txz = 0.0;
    let mut tyz = 0.0;

    // Summation over 8 corners with alternating signs
    for (i, &x) in xs.iter().enumerate() {
        for (j, &y) in ys.iter().enumerate() {
            for (k, &z) in zs.iter().enumerate() {
                let sign = if (i + j + k) % 2 == 0 { 1.0 } else { -1.0 };
                let r = (x * x + y * y + z * z).sqrt();
                if r < 1e-15 {
                    continue;
                }

                // Analytical formulas for gravity gradient of a rectangular prism
                // using the arctangent-logarithm formulation
                let safe_div = |num: f64, den: f64| -> f64 {
                    if den.abs() < 1e-30 { 0.0 } else { num / den }
                };

                // Tzz uses atan2(x*y, z*r)
                let atan_term_zz = if z.abs() > 1e-15 {
                    (x * y / (z * r)).atan()
                } else {
                    if x.abs() > 1e-15 && y.abs() > 1e-15 {
                        PI / 2.0 * (x * y).signum()
                    } else {
                        0.0
                    }
                };

                let atan_term_xx = if x.abs() > 1e-15 {
                    (y * z / (x * r)).atan()
                } else {
                    if y.abs() > 1e-15 && z.abs() > 1e-15 {
                        PI / 2.0 * (y * z).signum()
                    } else {
                        0.0
                    }
                };

                let atan_term_yy = if y.abs() > 1e-15 {
                    (x * z / (y * r)).atan()
                } else {
                    if x.abs() > 1e-15 && z.abs() > 1e-15 {
                        PI / 2.0 * (x * z).signum()
                    } else {
                        0.0
                    }
                };

                txx += sign * atan_term_xx;
                tyy += sign * atan_term_yy;
                tzz += sign * atan_term_zz;

                // Off-diagonal: Txy = -G*rho * ln(z + r), etc.
                let log_zr = if (z + r).abs() > 1e-15 { (z + r).ln() } else { 0.0 };
                let log_yr = if (y + r).abs() > 1e-15 { (y + r).ln() } else { 0.0 };
                let log_xr = if (x + r).abs() > 1e-15 { (x + r).ln() } else { 0.0 };

                txy -= sign * log_zr;
                txz -= sign * log_yr;
                tyz -= sign * log_xr;
            }
        }
    }

    GravityGradientTensor::new(
        coeff * txx,
        coeff * tyy,
        coeff * tzz,
        coeff * txy,
        coeff * txz,
        coeff * tyz,
    )
}

/// Forward model: gravity gradient tensor from a vertical cylinder (approximate).
///
/// Approximation using a stack of horizontal disks.
/// The cylinder has `density_kg_m3`, `radius_m`, extends from `z_top` to `z_bottom` (both positive downward).
/// Observed at (x_offset, 0, 0).
pub fn forward_vertical_cylinder(
    density_kg_m3: f64,
    radius_m: f64,
    z_top: f64,
    z_bottom: f64,
    x_offset: f64,
) -> GravityGradientTensor {
    // Approximate as a series of point masses along the vertical axis
    let n_segments = 50;
    let dz = (z_bottom - z_top) / n_segments as f64;
    let segment_mass = PI * radius_m * radius_m * dz * density_kg_m3;

    let mut result = GravityGradientTensor::zero();
    for i in 0..n_segments {
        let z = z_top + (i as f64 + 0.5) * dz;
        let seg = forward_point_mass(segment_mass, x_offset, 0.0, z);
        result = result.add(&seg);
    }
    result
}

// ─────────────────────────────────────────────────────────────────────
// Eigenvalue decomposition for 3x3 symmetric matrices (Jacobi)
// ─────────────────────────────────────────────────────────────────────

/// Compute eigenvalues of a 2x2 symmetric matrix [[a, b], [b, c]].
fn eigenvalues_2x2_symmetric(a: f64, b: f64, c: f64) -> (f64, f64) {
    let trace = a + c;
    let det = a * c - b * b;
    let disc = (trace * trace - 4.0 * det).max(0.0).sqrt();
    ((trace + disc) / 2.0, (trace - disc) / 2.0)
}

/// Jacobi eigenvalue algorithm for a 3x3 symmetric matrix.
///
/// Returns (eigenvalues sorted descending, eigenvectors as columns).
fn jacobi_eigendecomposition_3x3(mat: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut a = mat;
    // V = identity (accumulates rotations)
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    let max_iter = 100;
    for _ in 0..max_iter {
        // Find largest off-diagonal element
        let mut p = 0;
        let mut q = 1;
        let mut max_val = a[0][1].abs();
        for i in 0..3 {
            for j in (i + 1)..3 {
                if a[i][j].abs() > max_val {
                    max_val = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }

        if max_val < 1e-15 {
            break;
        }

        // Compute Jacobi rotation angle
        let diff = a[q][q] - a[p][p];
        let t = if diff.abs() < 1e-30 {
            1.0 // theta = pi/4
        } else {
            let tau = diff / (2.0 * a[p][q]);
            // Choose the smaller root for numerical stability
            let sign_tau = if tau >= 0.0 { 1.0 } else { -1.0 };
            sign_tau / (tau.abs() + (1.0 + tau * tau).sqrt())
        };

        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        let tau_val = s / (1.0 + c);

        // Update matrix A
        let a_pq = a[p][q];
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        a[p][p] -= t * a_pq;
        a[q][q] += t * a_pq;

        // Update remaining elements
        for r in 0..3 {
            if r == p || r == q {
                continue;
            }
            let a_rp = a[r][p];
            let a_rq = a[r][q];
            a[r][p] = a_rp - s * (a_rq + tau_val * a_rp);
            a[p][r] = a[r][p];
            a[r][q] = a_rq + s * (a_rp - tau_val * a_rq);
            a[q][r] = a[r][q];
        }

        // Update eigenvector matrix
        for r in 0..3 {
            let v_rp = v[r][p];
            let v_rq = v[r][q];
            v[r][p] = v_rp - s * (v_rq + tau_val * v_rp);
            v[r][q] = v_rq + s * (v_rp - tau_val * v_rq);
        }
    }

    // Extract eigenvalues from diagonal
    let mut evals = [a[0][0], a[1][1], a[2][2]];
    let mut evecs = v;

    // Sort descending by eigenvalue
    for i in 0..3 {
        let mut max_idx = i;
        for j in (i + 1)..3 {
            if evals[j] > evals[max_idx] {
                max_idx = j;
            }
        }
        if max_idx != i {
            evals.swap(i, max_idx);
            // Swap columns in eigenvector matrix
            for r in 0..3 {
                evecs[r].swap(i, max_idx);
            }
        }
    }

    (evals, evecs)
}

// ─────────────────────────────────────────────────────────────────────
// Euler Deconvolution
// ─────────────────────────────────────────────────────────────────────

/// Result of Euler deconvolution.
#[derive(Debug, Clone, Copy)]
pub struct EulerDeconvolutionResult {
    /// Estimated source x-position (m).
    pub x0: f64,
    /// Estimated source y-position (m).
    pub y0: f64,
    /// Estimated source depth (m, positive downward).
    pub z0: f64,
    /// Structural index used.
    pub structural_index: f64,
}

/// Structural index values for common source geometries.
pub struct StructuralIndex;

impl StructuralIndex {
    /// Contact or fault (SI = 0).
    pub const CONTACT: f64 = 0.0;
    /// Thin sheet or sill (SI = 1).
    pub const THIN_SHEET: f64 = 1.0;
    /// Line source / pipe / horizontal cylinder (SI = 2).
    pub const LINE: f64 = 2.0;
    /// Point mass / sphere (SI = 3).
    pub const POINT: f64 = 3.0;
}

/// Perform Euler deconvolution using the gravity gradient tensor.
///
/// Given a tensor measured at position (x, y) and a structural index SI,
/// estimates the source location using:
///   x0 = x - SI * Txz / Tzz
///   y0 = y - SI * Tyz / Tzz
///   z0 = SI * |some_component| / |Tzz|  (simplified depth estimate)
///
/// Returns None if Tzz is too small for stable division.
pub fn euler_deconvolution(
    tensor: &GravityGradientTensor,
    obs_x: f64,
    obs_y: f64,
    structural_index: f64,
) -> Option<EulerDeconvolutionResult> {
    if tensor.tzz.abs() < 1e-20 {
        return None;
    }

    let x0 = obs_x - structural_index * tensor.txz / tensor.tzz;
    let y0 = obs_y - structural_index * tensor.tyz / tensor.tzz;

    // Depth estimate from vertical gradient
    // For a point source: Tzz ∝ 1/z^(SI+1), so z ∝ (SI * Tz_mag / |Tzz|)
    // Simplified: z0 from the relationship between gradient components
    let hgm = tensor.horizontal_gradient_magnitude();
    let z0 = if hgm > 1e-20 {
        structural_index * hgm / tensor.tzz.abs()
    } else {
        // Fall back: use the Tzz magnitude with distance estimate
        // For point mass at depth d: Tzz = 2*G*m/d^3, so estimate from magnitude
        structural_index.max(1.0)
    };

    Some(EulerDeconvolutionResult {
        x0,
        y0,
        z0: z0.abs(),
        structural_index,
    })
}

// ─────────────────────────────────────────────────────────────────────
// Terrain and Gravity Corrections
// ─────────────────────────────────────────────────────────────────────

/// Simple Bouguer correction for a tensor measurement.
///
/// Removes the effect of a uniform slab of thickness `elevation_m` and density `density_kg_m3`.
/// Bouguer slab gradient contribution: Tzz_bouguer = 2 * pi * G * rho (in s^-2 → convert to E).
///
/// Returns the Bouguer correction in Eotvos to be subtracted from Tzz.
pub fn bouguer_slab_correction(density_kg_m3: f64, _elevation_m: f64) -> f64 {
    // The gradient of the Bouguer slab: d(gz)/dz = 2*pi*G*rho
    // Convert to Eotvos
    2.0 * PI * G * density_kg_m3 / EOTVOS_TO_SI
}

/// Free-air correction for gravity gradient.
///
/// The vertical gravity gradient in free air is approximately 0.3086 E per meter.
/// Returns the free-air gradient correction in Eotvos for the given elevation.
pub fn free_air_correction(elevation_m: f64) -> f64 {
    FREE_AIR_GRADIENT_MGAL_PER_M * elevation_m
}

/// Apply Bouguer correction to a tensor, removing slab effect from all components.
///
/// The infinite slab only affects diagonal components: Txx, Tyy get no contribution,
/// Tzz gets -2*pi*G*rho. The corrected tensor is returned.
pub fn apply_bouguer_correction(
    tensor: &GravityGradientTensor,
    density_kg_m3: f64,
) -> GravityGradientTensor {
    let correction = bouguer_slab_correction(density_kg_m3, 0.0);
    GravityGradientTensor::new(
        tensor.txx,
        tensor.tyy,
        tensor.tzz - correction,
        tensor.txy,
        tensor.txz,
        tensor.tyz,
    )
}

// ─────────────────────────────────────────────────────────────────────
// Gradient Processor
// ─────────────────────────────────────────────────────────────────────

/// Configuration for the gradient processor.
#[derive(Debug, Clone)]
pub struct GradientProcessorConfig {
    /// Measurement sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// Low-pass filter bandwidth in Hz.
    pub filter_bandwidth_hz: f64,
    /// Noise level in Eotvos (typical: 1-10 E airborne, 0.01-1 E borehole).
    pub noise_level_eotvos: f64,
}

impl Default for GradientProcessorConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 10.0,
            filter_bandwidth_hz: 1.0,
            noise_level_eotvos: 5.0,
        }
    }
}

/// Result of processing a tensor measurement.
#[derive(Debug, Clone)]
pub struct ProcessedTensorResult {
    /// Filtered tensor (after low-pass and Laplace enforcement).
    pub tensor: GravityGradientTensor,
    /// Eigenvalues (descending order).
    pub eigenvalues: [f64; 3],
    /// Horizontal gradient magnitude (E).
    pub horizontal_gradient: f64,
    /// Tilt angle (degrees).
    pub tilt_angle_deg: f64,
    /// Dimensionality index if computable.
    pub dimensionality_index: Option<f64>,
    /// Maximum curvature.
    pub max_curvature: f64,
    /// Minimum curvature.
    pub min_curvature: f64,
    /// Laplace residual before correction.
    pub laplace_residual_before: f64,
    /// Signal-to-noise ratio estimate.
    pub estimated_snr: f64,
}

/// Gradient processor for FTG survey data.
///
/// Provides filtering, Laplace enforcement, and attribute extraction.
pub struct GradientProcessor {
    config: GradientProcessorConfig,
    /// Simple IIR low-pass filter state for each of 6 components.
    filter_state: [f64; 6],
    /// IIR coefficient alpha.
    alpha: f64,
}

impl GradientProcessor {
    /// Create a new gradient processor.
    pub fn new(config: GradientProcessorConfig) -> Self {
        // Simple first-order IIR: alpha = 2*pi*fc / (fs + 2*pi*fc)
        let alpha = if config.sample_rate_hz > 0.0 {
            let wc = 2.0 * PI * config.filter_bandwidth_hz;
            wc / (config.sample_rate_hz + wc)
        } else {
            1.0 // no filtering
        };
        Self {
            config,
            filter_state: [0.0; 6],
            alpha,
        }
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.filter_state = [0.0; 6];
    }

    /// Low-pass filter a tensor, updating internal state.
    pub fn filter_tensor(&mut self, tensor: &GravityGradientTensor) -> GravityGradientTensor {
        let components = [tensor.txx, tensor.tyy, tensor.tzz, tensor.txy, tensor.txz, tensor.tyz];
        for (i, &c) in components.iter().enumerate() {
            self.filter_state[i] += self.alpha * (c - self.filter_state[i]);
        }
        GravityGradientTensor::new(
            self.filter_state[0],
            self.filter_state[1],
            self.filter_state[2],
            self.filter_state[3],
            self.filter_state[4],
            self.filter_state[5],
        )
    }

    /// Process a single tensor measurement: filter, enforce Laplace, extract attributes.
    pub fn process(&mut self, tensor: &GravityGradientTensor) -> ProcessedTensorResult {
        let laplace_residual_before = tensor.laplace_residual();

        // Low-pass filter
        let mut filtered = self.filter_tensor(tensor);

        // Enforce Laplace condition
        filtered.enforce_laplace();

        // Eigendecomposition
        let eigenvalues = filtered.principal_eigenvalues();

        // Attributes
        let horizontal_gradient = filtered.horizontal_gradient_magnitude();
        let tilt_angle_deg = filtered.tilt_angle_deg();
        let dimensionality_index = filtered.dimensionality_index();
        let max_curvature = filtered.maximum_curvature();
        let min_curvature = filtered.minimum_curvature();

        // SNR estimate: signal power / noise power
        let signal_power = filtered.frobenius_norm();
        let noise_power = self.config.noise_level_eotvos;
        let estimated_snr = if noise_power > 0.0 {
            20.0 * (signal_power / noise_power).log10()
        } else {
            f64::INFINITY
        };

        ProcessedTensorResult {
            tensor: filtered,
            eigenvalues,
            horizontal_gradient,
            tilt_angle_deg,
            dimensionality_index,
            max_curvature,
            min_curvature,
            laplace_residual_before,
            estimated_snr,
        }
    }

    /// Process a batch of tensors.
    pub fn process_batch(&mut self, tensors: &[GravityGradientTensor]) -> Vec<ProcessedTensorResult> {
        tensors.iter().map(|t| self.process(t)).collect()
    }

    /// Get configuration reference.
    pub fn config(&self) -> &GradientProcessorConfig {
        &self.config
    }
}

// ─────────────────────────────────────────────────────────────────────
// Survey Data Processing
// ─────────────────────────────────────────────────────────────────────

/// A survey data point with position and tensor measurement.
#[derive(Debug, Clone)]
pub struct SurveyPoint {
    /// Easting (x) in meters.
    pub x: f64,
    /// Northing (y) in meters.
    pub y: f64,
    /// Elevation (z) in meters.
    pub z: f64,
    /// Measured gravity gradient tensor.
    pub tensor: GravityGradientTensor,
}

/// Along-line gradient: numerical derivative of a tensor component along the survey line.
///
/// Returns dT/ds for each point (central differences, forward/backward at endpoints).
pub fn along_line_gradient(points: &[SurveyPoint], component_fn: fn(&GravityGradientTensor) -> f64) -> Vec<f64> {
    let n = points.len();
    if n < 2 {
        return vec![0.0; n];
    }

    let vals: Vec<f64> = points.iter().map(|p| component_fn(&p.tensor)).collect();
    let dists: Vec<f64> = (0..n)
        .map(|i| {
            if i == 0 {
                0.0
            } else {
                let dx = points[i].x - points[i - 1].x;
                let dy = points[i].y - points[i - 1].y;
                (dx * dx + dy * dy).sqrt()
            }
        })
        .collect();

    let mut gradients = vec![0.0; n];

    // Forward difference for first point
    if dists[1] > 1e-10 {
        gradients[0] = (vals[1] - vals[0]) / dists[1];
    }

    // Central differences for interior points
    for i in 1..(n - 1) {
        let ds = dists[i] + dists[i + 1];
        if ds > 1e-10 {
            gradients[i] = (vals[i + 1] - vals[i - 1]) / ds;
        }
    }

    // Backward difference for last point
    if n >= 2 && dists[n - 1] > 1e-10 {
        gradients[n - 1] = (vals[n - 1] - vals[n - 2]) / dists[n - 1];
    }

    gradients
}

/// Cross-line gradient: compute component differences between adjacent survey lines.
///
/// Given two parallel survey lines, returns the cross-line gradient for each pair.
pub fn cross_line_gradient(
    line1: &[SurveyPoint],
    line2: &[SurveyPoint],
    component_fn: fn(&GravityGradientTensor) -> f64,
) -> Vec<f64> {
    let n = line1.len().min(line2.len());
    let mut gradients = Vec::with_capacity(n);

    for i in 0..n {
        let dy = ((line2[i].x - line1[i].x).powi(2) + (line2[i].y - line1[i].y).powi(2)).sqrt();
        if dy > 1e-10 {
            let dval = component_fn(&line2[i].tensor) - component_fn(&line1[i].tensor);
            gradients.push(dval / dy);
        } else {
            gradients.push(0.0);
        }
    }

    gradients
}

/// Simple grid interpolation using inverse distance weighting (IDW).
///
/// Interpolates a tensor component onto a regular grid from scattered survey points.
pub fn grid_interpolation(
    points: &[SurveyPoint],
    component_fn: fn(&GravityGradientTensor) -> f64,
    grid_x: &[f64],
    grid_y: &[f64],
    power: f64,
) -> Vec<Vec<f64>> {
    let ny = grid_y.len();
    let nx = grid_x.len();
    let mut grid = vec![vec![0.0; nx]; ny];

    for (iy, &gy) in grid_y.iter().enumerate() {
        for (ix, &gx) in grid_x.iter().enumerate() {
            let mut weight_sum = 0.0;
            let mut value_sum = 0.0;

            for pt in points {
                let dx = gx - pt.x;
                let dy = gy - pt.y;
                let dist = (dx * dx + dy * dy).sqrt();
                if dist < 1e-10 {
                    // Exact point
                    value_sum = component_fn(&pt.tensor);
                    weight_sum = 1.0;
                    break;
                }
                let w = 1.0 / dist.powf(power);
                weight_sum += w;
                value_sum += w * component_fn(&pt.tensor);
            }

            grid[iy][ix] = if weight_sum > 0.0 { value_sum / weight_sum } else { 0.0 };
        }
    }

    grid
}

/// Leveling correction: remove DC offset and linear trend from a survey line.
///
/// Fits y = a + b*x and subtracts it from the data.
pub fn leveling_correction(values: &[f64]) -> Vec<f64> {
    let n = values.len();
    if n < 2 {
        return values.to_vec();
    }

    // Least-squares linear fit
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;
    let nf = n as f64;

    for (i, &v) in values.iter().enumerate() {
        let x = i as f64;
        sum_x += x;
        sum_y += v;
        sum_xx += x * x;
        sum_xy += x * v;
    }

    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return values.to_vec();
    }

    let b = (nf * sum_xy - sum_x * sum_y) / denom;
    let a = (sum_y - b * sum_x) / nf;

    values
        .iter()
        .enumerate()
        .map(|(i, &v)| v - (a + b * i as f64))
        .collect()
}

// ─────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // Helper: check approximate equality
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── Tensor basics ──

    #[test]
    fn test_tensor_creation() {
        let t = GravityGradientTensor::new(10.0, -5.0, -5.0, 2.0, 3.0, 1.0);
        assert_eq!(t.txx, 10.0);
        assert_eq!(t.tyy, -5.0);
        assert_eq!(t.tzz, -5.0);
        assert_eq!(t.txy, 2.0);
        assert_eq!(t.txz, 3.0);
        assert_eq!(t.tyz, 1.0);
    }

    #[test]
    fn test_tensor_from_independent() {
        let t = GravityGradientTensor::from_independent(10.0, -3.0, 2.0, 1.0, 0.5);
        assert!(approx_eq(t.tzz, -7.0, TOL));
        assert!(approx_eq(t.trace(), 0.0, TOL));
    }

    #[test]
    fn test_tensor_zero() {
        let t = GravityGradientTensor::zero();
        assert_eq!(t.frobenius_norm(), 0.0);
        assert_eq!(t.trace(), 0.0);
    }

    #[test]
    fn test_tensor_as_matrix() {
        let t = GravityGradientTensor::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let m = t.as_matrix();
        assert_eq!(m[0][0], 1.0);
        assert_eq!(m[1][1], 2.0);
        assert_eq!(m[2][2], 3.0);
        assert_eq!(m[0][1], 4.0);
        assert_eq!(m[1][0], 4.0); // symmetric
        assert_eq!(m[0][2], 5.0);
        assert_eq!(m[2][0], 5.0); // symmetric
    }

    #[test]
    fn test_laplace_condition() {
        let t = GravityGradientTensor::new(10.0, -5.0, -5.0, 0.0, 0.0, 0.0);
        assert!(t.satisfies_laplace(TOL));
        assert!(approx_eq(t.laplace_residual(), 0.0, TOL));
    }

    #[test]
    fn test_enforce_laplace() {
        let mut t = GravityGradientTensor::new(10.0, -3.0, -2.0, 1.0, 1.0, 1.0);
        assert!(!t.satisfies_laplace(TOL));
        t.enforce_laplace();
        assert!(t.satisfies_laplace(TOL));
        assert!(approx_eq(t.tzz, -7.0, TOL));
    }

    // ── Invariants ──

    #[test]
    fn test_invariant_i0_is_trace() {
        let t = GravityGradientTensor::new(5.0, -2.0, -3.0, 1.0, 0.5, -0.3);
        assert!(approx_eq(t.invariant_i0(), t.trace(), TOL));
    }

    #[test]
    fn test_invariant_i1_diagonal() {
        // For diagonal tensor [a, 0, 0; 0, b, 0; 0, 0, c]: I1 = ab + bc + ac
        let t = GravityGradientTensor::new(3.0, -1.0, -2.0, 0.0, 0.0, 0.0);
        let expected = 3.0 * (-1.0) + (-1.0) * (-2.0) + 3.0 * (-2.0);
        assert!(approx_eq(t.invariant_i1(), expected, TOL));
    }

    #[test]
    fn test_invariant_i2_determinant() {
        // Diagonal tensor: det = a*b*c
        let t = GravityGradientTensor::new(3.0, -1.0, -2.0, 0.0, 0.0, 0.0);
        assert!(approx_eq(t.invariant_i2(), 3.0 * (-1.0) * (-2.0), TOL));
    }

    #[test]
    fn test_dimensionality_index_point_source() {
        // Point mass directly below → should give D near 1.0 (3D source)
        let t = forward_point_mass(1e12, 0.0, 0.0, 1000.0);
        if let Some(d) = t.dimensionality_index() {
            assert!(d > 0.9, "Point source D={d} should be near 1.0");
        }
    }

    // ── Forward modeling ──

    #[test]
    fn test_point_mass_laplace() {
        let t = forward_point_mass(1e10, 0.0, 0.0, 500.0);
        assert!(t.satisfies_laplace(1e-6), "Laplace residual: {}", t.laplace_residual());
    }

    #[test]
    fn test_point_mass_symmetry_off_axis() {
        // Symmetric offset: (100, 100, 500) should give Txy != 0
        let t = forward_point_mass(1e10, 100.0, 100.0, 500.0);
        assert!(t.txy.abs() > 0.0, "Off-axis should have nonzero Txy");
        assert!(t.satisfies_laplace(1e-6));
    }

    #[test]
    fn test_point_mass_on_axis() {
        // Directly below: Txx = Tyy, Txy = Txz = Tyz = 0
        let t = forward_point_mass(1e10, 0.0, 0.0, 500.0);
        assert!(approx_eq(t.txx, t.tyy, 1e-10), "On-axis: Txx should equal Tyy");
        assert!(approx_eq(t.txy, 0.0, 1e-10));
        assert!(approx_eq(t.txz, 0.0, 1e-10));
        assert!(approx_eq(t.tyz, 0.0, 1e-10));
    }

    #[test]
    fn test_point_mass_distance_scaling() {
        // Gradient scales as 1/r^3 for point mass, so doubling distance → 1/8 gradient
        let t1 = forward_point_mass(1e10, 0.0, 0.0, 500.0);
        let t2 = forward_point_mass(1e10, 0.0, 0.0, 1000.0);
        let ratio = t1.tzz / t2.tzz;
        assert!(approx_eq(ratio, 8.0, 0.01), "Distance scaling ratio={ratio}, expected 8");
    }

    #[test]
    fn test_sphere_equals_point_mass() {
        let density: f64 = 2700.0; // kg/m³
        let radius: f64 = 100.0; // m
        let mass: f64 = (4.0 / 3.0) * PI * radius.powi(3) * density;

        let t_sphere = forward_sphere(density, radius, 0.0, 0.0, 1000.0);
        let t_point = forward_point_mass(mass, 0.0, 0.0, 1000.0);

        assert!(approx_eq(t_sphere.txx, t_point.txx, 1e-10));
        assert!(approx_eq(t_sphere.tzz, t_point.tzz, 1e-10));
    }

    #[test]
    fn test_infinite_cylinder_laplace_2d() {
        // For infinite horizontal cylinder (2D), Txx + Tzz ≈ 0
        let t = forward_infinite_cylinder(2700.0, 50.0, 0.0, 500.0);
        let laplace_2d = t.txx + t.tzz;
        assert!(laplace_2d.abs() < 1e-6, "2D Laplace: Txx+Tzz={laplace_2d}");
    }

    #[test]
    fn test_infinite_cylinder_on_axis() {
        // Directly above cylinder: Txz = 0 (by symmetry)
        let t = forward_infinite_cylinder(2700.0, 50.0, 0.0, 500.0);
        assert!(approx_eq(t.txz, 0.0, 1e-10));
    }

    #[test]
    fn test_rectangular_prism_nonzero() {
        let t = forward_rectangular_prism(2700.0, -100.0, 100.0, -100.0, 100.0, 200.0, 400.0);
        // Should produce nonzero gradients
        assert!(t.frobenius_norm() > 0.0, "Prism tensor should be nonzero");
    }

    #[test]
    fn test_vertical_cylinder_nonzero() {
        let t = forward_vertical_cylinder(2700.0, 50.0, 200.0, 500.0, 0.0);
        assert!(t.frobenius_norm() > 0.0, "Vertical cylinder should produce nonzero tensor");
        // On axis: Txy, Tyz should be ~0
        assert!(t.txy.abs() < 1e-6);
    }

    // ── Eigenvalue decomposition ──

    #[test]
    fn test_eigenvalues_diagonal() {
        let t = GravityGradientTensor::new(5.0, -2.0, -3.0, 0.0, 0.0, 0.0);
        let evals = t.principal_eigenvalues();
        // Sorted descending: 5, -2, -3
        assert!(approx_eq(evals[0], 5.0, TOL));
        assert!(approx_eq(evals[1], -2.0, TOL));
        assert!(approx_eq(evals[2], -3.0, TOL));
    }

    #[test]
    fn test_eigenvalues_sum_equals_trace() {
        let t = GravityGradientTensor::new(10.0, -3.0, -7.0, 2.0, 1.0, -0.5);
        let evals = t.principal_eigenvalues();
        let sum = evals[0] + evals[1] + evals[2];
        assert!(approx_eq(sum, t.trace(), 1e-8), "Eigenvalue sum {sum} != trace {}", t.trace());
    }

    #[test]
    fn test_eigenvalues_product_equals_determinant() {
        let t = GravityGradientTensor::new(10.0, -3.0, -7.0, 2.0, 1.0, -0.5);
        let evals = t.principal_eigenvalues();
        let product = evals[0] * evals[1] * evals[2];
        let det = t.invariant_i2();
        assert!(approx_eq(product, det, 1e-4), "Product={product}, det={det}");
    }

    #[test]
    fn test_eigenvectors_orthogonal() {
        let t = GravityGradientTensor::new(10.0, -3.0, -7.0, 2.0, 1.0, -0.5);
        let (_, evecs) = t.eigendecomposition();

        // Check dot products of column vectors
        for i in 0..3 {
            for j in (i + 1)..3 {
                let dot: f64 = (0..3).map(|r| evecs[r][i] * evecs[r][j]).sum();
                assert!(dot.abs() < 1e-8, "Eigenvectors {i} and {j} not orthogonal: dot={dot}");
            }
        }
    }

    // ── Curvature attributes ──

    #[test]
    fn test_curvature_diagonal() {
        // Diagonal tensor: eigenvalues of 2x2 submatrix [[Txx,0],[0,Tyy]] = Txx, Tyy
        let t = GravityGradientTensor::new(8.0, -3.0, -5.0, 0.0, 0.0, 0.0);
        assert!(approx_eq(t.maximum_curvature(), 8.0, TOL));
        assert!(approx_eq(t.minimum_curvature(), -3.0, TOL));
        assert!(approx_eq(t.gaussian_curvature(), 8.0 * (-3.0), TOL));
        assert!(approx_eq(t.mean_curvature(), 0.5 * (8.0 + (-3.0)), TOL));
    }

    #[test]
    fn test_gaussian_curvature_formula() {
        let t = GravityGradientTensor::new(5.0, -2.0, -3.0, 1.0, 0.0, 0.0);
        // Gaussian = Txx*Tyy - Txy² = 5*(-2) - 1 = -11
        assert!(approx_eq(t.gaussian_curvature(), -11.0, TOL));
    }

    #[test]
    fn test_shape_index_dome() {
        // A dome: both curvatures positive and equal → shape_index → None or near 1
        let t = GravityGradientTensor::new(5.0, 5.0, -10.0, 0.0, 0.0, 0.0);
        // Equal curvatures → shape_index is None (indeterminate)
        assert!(t.shape_index().is_none());
    }

    // ── Euler deconvolution ──

    #[test]
    fn test_euler_deconvolution_point_mass() {
        // Point mass at (0, 0, -500), observed at (100, 0, 0)
        let t = forward_point_mass(1e10, 100.0, 0.0, 500.0);
        let result = euler_deconvolution(&t, 100.0, 0.0, StructuralIndex::POINT);
        assert!(result.is_some());
        let r = result.unwrap();
        // x0 should be closer to 0 (the mass location)
        assert!(r.x0.abs() < 100.0, "Euler x0={} should be closer to source at 0", r.x0);
    }

    #[test]
    fn test_euler_deconvolution_zero_tzz() {
        let t = GravityGradientTensor::new(1.0, -1.0, 0.0, 0.0, 0.5, 0.0);
        let result = euler_deconvolution(&t, 0.0, 0.0, 3.0);
        // Tzz = 0 → should return None
        assert!(result.is_none());
    }

    // ── Terrain corrections ──

    #[test]
    fn test_bouguer_correction_positive() {
        let correction = bouguer_slab_correction(2670.0, 100.0);
        // 2*pi*G*rho / 1e-9 should be positive
        assert!(correction > 0.0, "Bouguer correction should be positive");
        // Expected: 2*pi*6.674e-11*2670 / 1e-9 ≈ 1.119 E
        assert!(approx_eq(correction, 2.0 * PI * G * 2670.0 / EOTVOS_TO_SI, 0.001));
    }

    #[test]
    fn test_free_air_correction() {
        let correction = free_air_correction(100.0);
        assert!(approx_eq(correction, 0.3086 * 100.0, 0.01));
    }

    #[test]
    fn test_apply_bouguer_correction() {
        let t = GravityGradientTensor::new(10.0, -5.0, -5.0, 0.0, 0.0, 0.0);
        let corrected = apply_bouguer_correction(&t, 2670.0);
        // Only Tzz changes
        assert!(approx_eq(corrected.txx, t.txx, TOL));
        assert!(corrected.tzz < t.tzz, "Bouguer correction should reduce Tzz");
    }

    // ── Gradient processor ──

    #[test]
    fn test_processor_creation() {
        let config = GradientProcessorConfig::default();
        let processor = GradientProcessor::new(config);
        assert!(processor.alpha > 0.0 && processor.alpha < 1.0);
    }

    #[test]
    fn test_processor_filter_converges() {
        let config = GradientProcessorConfig {
            sample_rate_hz: 100.0,
            filter_bandwidth_hz: 10.0,
            noise_level_eotvos: 1.0,
        };
        let mut processor = GradientProcessor::new(config);
        let t = GravityGradientTensor::new(10.0, -5.0, -5.0, 1.0, 2.0, -1.0);

        // Feed the same tensor many times; filter should converge
        let mut filtered = GravityGradientTensor::zero();
        for _ in 0..1000 {
            filtered = processor.filter_tensor(&t);
        }
        assert!(approx_eq(filtered.txx, t.txx, 0.01));
        assert!(approx_eq(filtered.tzz, t.tzz, 0.01));
    }

    #[test]
    fn test_processor_process_enforces_laplace() {
        let config = GradientProcessorConfig::default();
        let mut processor = GradientProcessor::new(config);

        // Feed a tensor that violates Laplace
        let t = GravityGradientTensor::new(10.0, -3.0, -2.0, 0.0, 0.0, 0.0);
        // Run enough to converge
        let mut result = processor.process(&t);
        for _ in 0..500 {
            result = processor.process(&t);
        }
        assert!(result.tensor.satisfies_laplace(0.01));
    }

    #[test]
    fn test_processor_batch() {
        let config = GradientProcessorConfig::default();
        let mut processor = GradientProcessor::new(config);
        let tensors = vec![
            GravityGradientTensor::new(5.0, -2.0, -3.0, 0.0, 0.0, 0.0),
            GravityGradientTensor::new(8.0, -4.0, -4.0, 1.0, 0.0, 0.0),
        ];
        let results = processor.process_batch(&tensors);
        assert_eq!(results.len(), 2);
    }

    // ── Survey data processing ──

    #[test]
    fn test_along_line_gradient() {
        // Linear increase in Tzz along a line
        let points: Vec<SurveyPoint> = (0..5)
            .map(|i| SurveyPoint {
                x: i as f64 * 100.0,
                y: 0.0,
                z: 0.0,
                tensor: GravityGradientTensor::new(0.0, 0.0, i as f64 * 10.0, 0.0, 0.0, 0.0),
            })
            .collect();

        let grad = along_line_gradient(&points, |t| t.tzz);
        // Interior gradients should be ~10/100 = 0.1 E/m
        assert!(approx_eq(grad[2], 0.1, 0.01));
    }

    #[test]
    fn test_cross_line_gradient() {
        let line1: Vec<SurveyPoint> = (0..3)
            .map(|i| SurveyPoint {
                x: i as f64 * 100.0,
                y: 0.0,
                z: 0.0,
                tensor: GravityGradientTensor::new(10.0, -5.0, -5.0, 0.0, 0.0, 0.0),
            })
            .collect();
        let line2: Vec<SurveyPoint> = (0..3)
            .map(|i| SurveyPoint {
                x: i as f64 * 100.0,
                y: 200.0,
                z: 0.0,
                tensor: GravityGradientTensor::new(20.0, -10.0, -10.0, 0.0, 0.0, 0.0),
            })
            .collect();

        let grad = cross_line_gradient(&line1, &line2, |t| t.txx);
        // (20 - 10) / 200 = 0.05
        assert!(approx_eq(grad[0], 0.05, 0.001));
    }

    #[test]
    fn test_grid_interpolation() {
        let points = vec![
            SurveyPoint {
                x: 0.0, y: 0.0, z: 0.0,
                tensor: GravityGradientTensor::new(10.0, 0.0, -10.0, 0.0, 0.0, 0.0),
            },
            SurveyPoint {
                x: 100.0, y: 0.0, z: 0.0,
                tensor: GravityGradientTensor::new(20.0, 0.0, -20.0, 0.0, 0.0, 0.0),
            },
        ];
        let grid_x = vec![0.0, 50.0, 100.0];
        let grid_y = vec![0.0];
        let grid = grid_interpolation(&points, |t| t.txx, &grid_x, &grid_y, 2.0);

        // At x=0: should be 10, at x=100: should be 20, at x=50: between
        assert!(approx_eq(grid[0][0], 10.0, TOL));
        assert!(approx_eq(grid[0][2], 20.0, TOL));
        assert!(grid[0][1] > 10.0 && grid[0][1] < 20.0);
    }

    #[test]
    fn test_leveling_correction() {
        // Linear trend: 0, 10, 20, 30, 40
        let values = vec![0.0, 10.0, 20.0, 30.0, 40.0];
        let corrected = leveling_correction(&values);
        // After removing linear trend, all should be ~0
        for v in &corrected {
            assert!(v.abs() < 0.01, "Leveled value {v} should be near zero");
        }
    }

    #[test]
    fn test_leveling_correction_preserves_anomaly() {
        // Linear trend + anomaly at center
        let values = vec![0.0, 10.0, 25.0, 30.0, 40.0];
        let corrected = leveling_correction(&values);
        // The anomaly at index 2 should remain as a residual
        let max_residual = corrected.iter().cloned().fold(0.0f64, f64::max);
        assert!(max_residual > 1.0, "Anomaly residual should be preserved");
    }

    // ── Tilt angle and horizontal gradient ──

    #[test]
    fn test_tilt_angle_vertical() {
        // Pure vertical gradient: tilt should be 90 degrees
        let t = GravityGradientTensor::new(0.0, 0.0, 10.0, 0.0, 0.0, 0.0);
        assert!(approx_eq(t.tilt_angle_deg(), 90.0, 0.01));
    }

    #[test]
    fn test_horizontal_gradient_magnitude() {
        let t = GravityGradientTensor::new(0.0, 0.0, 0.0, 0.0, 3.0, 4.0);
        assert!(approx_eq(t.horizontal_gradient_magnitude(), 5.0, TOL));
    }

    // ── Tensor arithmetic ──

    #[test]
    fn test_tensor_add() {
        let a = GravityGradientTensor::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let b = GravityGradientTensor::new(10.0, 20.0, 30.0, 40.0, 50.0, 60.0);
        let c = a.add(&b);
        assert!(approx_eq(c.txx, 11.0, TOL));
        assert!(approx_eq(c.tyz, 66.0, TOL));
    }

    #[test]
    fn test_tensor_scale() {
        let t = GravityGradientTensor::new(2.0, 4.0, 6.0, 8.0, 10.0, 12.0);
        let s = t.scale(0.5);
        assert!(approx_eq(s.txx, 1.0, TOL));
        assert!(approx_eq(s.tyz, 6.0, TOL));
    }

    #[test]
    fn test_frobenius_norm() {
        // Diagonal: norm = sqrt(a² + b² + c²)
        let t = GravityGradientTensor::new(3.0, 4.0, 0.0, 0.0, 0.0, 0.0);
        assert!(approx_eq(t.frobenius_norm(), 5.0, TOL));
    }
}

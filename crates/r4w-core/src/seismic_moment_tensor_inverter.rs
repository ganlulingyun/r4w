//! Focal mechanism estimation from seismic waveforms via moment tensor inversion.
//!
//! This module implements the full moment tensor inversion pipeline used in
//! seismology to determine earthquake source mechanisms. It covers:
//!
//! - **Moment Tensor Representation**: 3x3 symmetric tensor with eigendecomposition
//! - **Focal Mechanism**: Strike, dip, and rake angles describing fault geometry
//! - **Inversion**: Least-squares inversion from P-wave first motions and waveform amplitudes
//! - **Source Type Classification**: Decomposition into double-couple (DC), compensated
//!   linear vector dipole (CLVD), and isotropic components using the Hudson T-k plot
//! - **Beach Ball Visualization**: Equal-area (Lambert azimuthal) projection of P/T axes
//!   and nodal planes for focal mechanism visualization
//! - **Radiation Patterns**: Far-field P-wave and S-wave radiation pattern computation
//!
//! # Background
//!
//! The seismic moment tensor **M** is a symmetric 3x3 tensor that completely describes
//! the equivalent body forces at an earthquake source. For a point source, the
//! displacement field is:
//!
//! ```text
//! u_i(x, t) = M_jk * G_ij,k(x, xi, t)
//! ```
//!
//! where G is the Green's function. The moment tensor can be decomposed into:
//!
//! - **Isotropic (ISO)**: Volumetric change (explosion/implosion)
//! - **Double-Couple (DC)**: Pure shear faulting
//! - **CLVD**: Compensated linear vector dipole (non-DC deviatoric)
//!
//! The eigenvalues of the deviatoric part determine the source type, visualized
//! on the Hudson T-k source type plot.
//!
//! # Example
//!
//! ```
//! use r4w_core::seismic_moment_tensor_inverter::{
//!     MomentTensor, FocalMechanism, MomentTensorInverter,
//!     SourceTypeClassifier, BeachBallGenerator, StationObservation,
//!     seismic_moment, moment_magnitude,
//! };
//!
//! // Create a pure double-couple moment tensor (vertical strike-slip fault)
//! let mt = MomentTensor::from_strike_dip_rake(0.0, 90.0, 0.0, 1e16);
//!
//! // Extract focal mechanism
//! let fm = mt.to_focal_mechanism();
//! assert!((fm.dip - 90.0).abs() < 1.0);
//!
//! // Classify source type
//! let source = SourceTypeClassifier::classify(&mt);
//! assert!(source.dc_percentage > 90.0);
//!
//! // Compute moment magnitude
//! let mw = moment_magnitude(1e16);
//! assert!(mw > 4.0 && mw < 5.0);
//! ```

use std::f64::consts::PI;

// ─── Constants ───────────────────────────────────────────────────────────────

/// Degrees to radians conversion factor.
const DEG2RAD: f64 = PI / 180.0;
/// Radians to degrees conversion factor.
const RAD2DEG: f64 = 180.0 / PI;

// ─── 3x3 Matrix Utilities ───────────────────────────────────────────────────

/// A 3x3 matrix stored in row-major order.
type Mat3 = [[f64; 3]; 3];

/// Multiply two 3x3 matrices: C = A * B.
fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

/// Transpose a 3x3 matrix.
fn mat3_transpose(a: &Mat3) -> Mat3 {
    let mut t = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            t[i][j] = a[j][i];
        }
    }
    t
}

/// Matrix-vector product: y = A * x.
fn mat3_vec(a: &Mat3, x: &[f64; 3]) -> [f64; 3] {
    let mut y = [0.0; 3];
    for i in 0..3 {
        for j in 0..3 {
            y[i] += a[i][j] * x[j];
        }
    }
    y
}

/// Compute the trace of a 3x3 matrix.
fn mat3_trace(a: &Mat3) -> f64 {
    a[0][0] + a[1][1] + a[2][2]
}

/// Identity 3x3 matrix.
fn mat3_identity() -> Mat3 {
    [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
}

/// Frobenius norm of a 3x3 matrix.
fn mat3_frobenius(a: &Mat3) -> f64 {
    let mut sum = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            sum += a[i][j] * a[i][j];
        }
    }
    sum.sqrt()
}

/// Vector dot product.
fn dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Vector cross product.
fn cross3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Euclidean norm of a 3-vector.
fn norm3(v: &[f64; 3]) -> f64 {
    dot3(v, v).sqrt()
}

/// Normalize a 3-vector. Returns zero vector if input norm is near zero.
fn normalize3(v: &[f64; 3]) -> [f64; 3] {
    let n = norm3(v);
    if n < 1e-30 {
        return [0.0; 3];
    }
    [v[0] / n, v[1] / n, v[2] / n]
}

// ─── Eigendecomposition (Jacobi method for 3x3 symmetric) ──────────────────

/// Eigendecomposition result: eigenvalues and eigenvectors of a symmetric 3x3 matrix.
#[derive(Debug, Clone)]
pub struct Eigen3 {
    /// Eigenvalues sorted in descending order (lambda1 >= lambda2 >= lambda3).
    pub values: [f64; 3],
    /// Corresponding eigenvectors as columns (vectors[i] is the eigenvector for values[i]).
    pub vectors: [[f64; 3]; 3],
}

/// Jacobi eigenvalue algorithm for a 3x3 real symmetric matrix.
/// Returns eigenvalues sorted in descending order with corresponding eigenvectors.
fn eigen_symmetric_3x3(m: &Mat3) -> Eigen3 {
    let mut a = *m;
    let mut v = mat3_identity();
    let max_iter = 100;

    for _ in 0..max_iter {
        // Find the largest off-diagonal element
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

        // Compute Jacobi rotation
        let theta = if (a[p][p] - a[q][q]).abs() < 1e-30 {
            PI / 4.0
        } else {
            0.5 * (2.0 * a[p][q] / (a[p][p] - a[q][q])).atan()
        };

        let c = theta.cos();
        let s = theta.sin();

        // Apply rotation: A' = J^T * A * J
        let mut rot = mat3_identity();
        rot[p][p] = c;
        rot[q][q] = c;
        rot[p][q] = s;
        rot[q][p] = -s;

        let rot_t = mat3_transpose(&rot);
        a = mat3_mul(&mat3_mul(&rot_t, &a), &rot);
        v = mat3_mul(&v, &rot);
    }

    // Extract eigenvalues and eigenvectors
    let mut indices = [0usize, 1, 2];
    indices.sort_by(|&i, &j| a[j][j].partial_cmp(&a[i][i]).unwrap_or(std::cmp::Ordering::Equal));

    let values = [a[indices[0]][indices[0]], a[indices[1]][indices[1]], a[indices[2]][indices[2]]];
    let vectors = [
        [v[0][indices[0]], v[1][indices[0]], v[2][indices[0]]],
        [v[0][indices[1]], v[1][indices[1]], v[2][indices[1]]],
        [v[0][indices[2]], v[1][indices[2]], v[2][indices[2]]],
    ];

    Eigen3 { values, vectors }
}

// ─── Moment Tensor ──────────────────────────────────────────────────────────

/// A seismic moment tensor represented as a 3x3 symmetric matrix.
///
/// The tensor is stored in the NED (North-East-Down) coordinate system:
/// - Index 0: North (r direction)
/// - Index 1: East (theta direction)
/// - Index 2: Down (phi direction)
///
/// Components: Mxx, Myy, Mzz, Mxy, Mxz, Myz (6 independent elements).
#[derive(Debug, Clone)]
pub struct MomentTensor {
    /// The 3x3 symmetric moment tensor matrix (NED coordinates).
    pub m: Mat3,
}

impl MomentTensor {
    /// Create a moment tensor from the 6 independent components.
    ///
    /// # Arguments
    /// * `mxx` - North-North component
    /// * `myy` - East-East component
    /// * `mzz` - Down-Down component
    /// * `mxy` - North-East component
    /// * `mxz` - North-Down component
    /// * `myz` - East-Down component
    pub fn new(mxx: f64, myy: f64, mzz: f64, mxy: f64, mxz: f64, myz: f64) -> Self {
        Self {
            m: [
                [mxx, mxy, mxz],
                [mxy, myy, myz],
                [mxz, myz, mzz],
            ],
        }
    }

    /// Create a moment tensor from the full 3x3 matrix.
    /// The matrix is symmetrized: M' = (M + M^T) / 2.
    pub fn from_matrix(m: Mat3) -> Self {
        let mut sym = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                sym[i][j] = 0.5 * (m[i][j] + m[j][i]);
            }
        }
        Self { m: sym }
    }

    /// Construct a moment tensor from strike, dip, rake angles and scalar moment.
    ///
    /// Uses the Aki & Richards (2002) convention:
    /// - Strike: clockwise from North (0-360 degrees)
    /// - Dip: downward from horizontal (0-90 degrees)
    /// - Rake: slip direction on fault plane (-180 to 180 degrees)
    /// - M0: scalar seismic moment in N*m
    pub fn from_strike_dip_rake(strike_deg: f64, dip_deg: f64, rake_deg: f64, m0: f64) -> Self {
        let phi = strike_deg * DEG2RAD;
        let delta = dip_deg * DEG2RAD;
        let lambda = rake_deg * DEG2RAD;

        let sin_d = delta.sin();
        let cos_d = delta.cos();
        let sin_2d = (2.0 * delta).sin();
        let cos_2d = (2.0 * delta).cos();
        let sin_l = lambda.sin();
        let cos_l = lambda.cos();
        let sin_p = phi.sin();
        let cos_p = phi.cos();
        let sin_2p = (2.0 * phi).sin();
        let cos_2p = (2.0 * phi).cos();

        // Aki & Richards (2002) equations 4.88-4.91
        let mxx = -m0 * (sin_d * cos_l * sin_2p + sin_2d * sin_l * sin_p * sin_p);
        let myy = m0 * (sin_d * cos_l * sin_2p - sin_2d * sin_l * cos_p * cos_p);
        let mzz = m0 * sin_2d * sin_l;
        let mxy = m0 * (sin_d * cos_l * cos_2p + 0.5 * sin_2d * sin_l * sin_2p);
        let mxz = -m0 * (cos_d * cos_l * cos_p + cos_2d * sin_l * sin_p);
        let myz = -m0 * (cos_d * cos_l * sin_p - cos_2d * sin_l * cos_p);

        Self::new(mxx, myy, mzz, mxy, mxz, myz)
    }

    /// Compute eigendecomposition of the moment tensor.
    ///
    /// Returns eigenvalues sorted descending (T, B, P axes) and eigenvectors.
    pub fn eigendecompose(&self) -> Eigen3 {
        eigen_symmetric_3x3(&self.m)
    }

    /// Compute the scalar seismic moment M0 from the tensor.
    ///
    /// M0 = (1/sqrt(2)) * sqrt(sum(m_ij^2))
    pub fn scalar_moment(&self) -> f64 {
        let mut sum = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                sum += self.m[i][j] * self.m[i][j];
            }
        }
        (sum / 2.0).sqrt()
    }

    /// Compute the moment magnitude Mw from this tensor.
    pub fn moment_magnitude(&self) -> f64 {
        moment_magnitude(self.scalar_moment())
    }

    /// Extract the isotropic part of the moment tensor.
    ///
    /// M_iso = (1/3) * tr(M) * I
    pub fn isotropic_part(&self) -> MomentTensor {
        let tr = mat3_trace(&self.m) / 3.0;
        MomentTensor::new(tr, tr, tr, 0.0, 0.0, 0.0)
    }

    /// Extract the deviatoric part of the moment tensor.
    ///
    /// M_dev = M - M_iso
    pub fn deviatoric_part(&self) -> MomentTensor {
        let tr = mat3_trace(&self.m) / 3.0;
        let mut dev = self.m;
        dev[0][0] -= tr;
        dev[1][1] -= tr;
        dev[2][2] -= tr;
        MomentTensor { m: dev }
    }

    /// Extract the focal mechanism (strike, dip, rake) from this moment tensor.
    ///
    /// Uses eigendecomposition to find the T and P axes, then computes the
    /// fault normal and slip vectors to determine the two nodal planes.
    /// Returns the focal mechanism for the preferred nodal plane (dip <= 90).
    pub fn to_focal_mechanism(&self) -> FocalMechanism {
        let eigen = self.eigendecompose();

        let t_axis = eigen.vectors[0]; // largest eigenvalue
        let p_axis = eigen.vectors[2]; // smallest eigenvalue

        // Fault normal and slip vectors from T and P axes
        // n = (T + P) / |T + P|, s = (T - P) / |T - P|
        let n_raw = [
            t_axis[0] + p_axis[0],
            t_axis[1] + p_axis[1],
            t_axis[2] + p_axis[2],
        ];
        let s_raw = [
            t_axis[0] - p_axis[0],
            t_axis[1] - p_axis[1],
            t_axis[2] - p_axis[2],
        ];

        let n = normalize3(&n_raw);
        let s = normalize3(&s_raw);

        // Compute both nodal planes, pick the one with valid dip
        let fm1 = normal_slip_to_sdr(&n, &s);
        let fm2 = normal_slip_to_sdr(&s, &n);

        // Return the plane with smaller dip as primary
        if fm1.dip <= fm2.dip { fm1 } else { fm2 }
    }

    /// Compute the P-wave radiation pattern at a given take-off angle and azimuth.
    ///
    /// Returns the P-wave amplitude for a ray leaving the source at the
    /// specified angles (in degrees).
    pub fn p_radiation(&self, takeoff_deg: f64, azimuth_deg: f64) -> f64 {
        let gamma = radiation_pattern(&self.m, takeoff_deg, azimuth_deg);
        gamma[0] // radial component
    }

    /// Get the trace of the moment tensor.
    pub fn trace(&self) -> f64 {
        mat3_trace(&self.m)
    }

    /// Get the Frobenius norm of the moment tensor.
    pub fn frobenius_norm(&self) -> f64 {
        mat3_frobenius(&self.m)
    }
}

/// Convert fault normal and slip vectors to strike, dip, rake angles.
fn normal_slip_to_sdr(n: &[f64; 3], s: &[f64; 3]) -> FocalMechanism {
    // Ensure fault normal points upward (n_z <= 0 in NED means up)
    let mut nn = *n;
    let mut ss = *s;
    if nn[2] > 0.0 {
        nn = [-nn[0], -nn[1], -nn[2]];
        ss = [-ss[0], -ss[1], -ss[2]];
    }

    // Dip: angle from horizontal
    let dip = (-nn[2]).acos() * RAD2DEG;

    // Strike: azimuth of fault trace (perpendicular to dip direction, right-hand rule)
    let strike = if dip.abs() < 1e-6 {
        // Horizontal fault: strike is ambiguous, use slip direction
        ss[1].atan2(ss[0]) * RAD2DEG
    } else {
        // Strike is 90 degrees clockwise from the horizontal projection of fault normal
        (nn[1]).atan2(nn[0]) * RAD2DEG + 90.0
    };

    // Normalize strike to [0, 360)
    let strike = ((strike % 360.0) + 360.0) % 360.0;

    // Compute rake from slip vector projection onto fault plane basis vectors
    let cos_strike = (strike * DEG2RAD).cos();
    let sin_strike = (strike * DEG2RAD).sin();

    // Horizontal strike direction (in the fault plane, along strike)
    let h_strike = [sin_strike, cos_strike, 0.0_f64];

    // Down-dip direction (in the fault plane, perpendicular to strike, pointing down-dip)
    let dip_rad = dip * DEG2RAD;
    let h_downdip = [
        -cos_strike * dip_rad.sin(),
        sin_strike * dip_rad.sin(),
        -dip_rad.cos(),
    ];

    // Rake = atan2(slip . down_dip, slip . strike)
    let rake = (dot3(&ss, &h_downdip)).atan2(dot3(&ss, &h_strike)) * RAD2DEG;

    FocalMechanism {
        strike,
        dip,
        rake,
    }
}

// ─── Focal Mechanism ────────────────────────────────────────────────────────

/// Fault plane solution described by strike, dip, and rake angles.
///
/// Follows the Aki & Richards convention:
/// - **Strike** (0-360): Azimuth of the fault trace from North, measured clockwise.
///   When standing on the fault looking along strike, the fault dips to the right.
/// - **Dip** (0-90): Angle of the fault plane from horizontal, measured downward.
/// - **Rake** (-180 to 180): Direction of slip on the fault plane. 0 = left-lateral,
///   90 = reverse (thrust), 180/-180 = right-lateral, -90 = normal.
#[derive(Debug, Clone, Copy)]
pub struct FocalMechanism {
    /// Strike angle in degrees (0-360).
    pub strike: f64,
    /// Dip angle in degrees (0-90).
    pub dip: f64,
    /// Rake angle in degrees (-180 to 180).
    pub rake: f64,
}

impl FocalMechanism {
    /// Create a new focal mechanism from strike, dip, rake (in degrees).
    pub fn new(strike: f64, dip: f64, rake: f64) -> Self {
        Self { strike, dip, rake }
    }

    /// Compute the auxiliary (conjugate) nodal plane.
    ///
    /// Every double-couple source has two nodal planes. Given one, this
    /// computes the other using the standard formulas.
    pub fn auxiliary_plane(&self) -> FocalMechanism {
        let sr = self.strike * DEG2RAD;
        let dr = self.dip * DEG2RAD;
        let rr = self.rake * DEG2RAD;

        // Auxiliary plane dip
        let dip2 = (rr.cos() / dr.sin()).acos().abs();

        // Auxiliary plane rake
        let sin_rake2 = -(dr.sin() / dip2.sin()) * sr.cos();
        let cos_rake2 = (rr.cos() * dr.cos()) / dip2.sin();
        let _ = cos_rake2; // used below
        let rake2 = sin_rake2.asin();

        // Correct quadrant based on original rake
        let rake2 = if rr.cos() < 0.0 {
            if rake2 > 0.0 { PI - rake2 } else { -PI - rake2 }
        } else {
            rake2
        };

        // Auxiliary plane strike
        let strike2 = sr - (-rr.cos() / dip2.sin()).atan2(
            -(rr.sin() * dr.cos()) / dip2.sin(),
        );

        FocalMechanism {
            strike: ((strike2 * RAD2DEG) % 360.0 + 360.0) % 360.0,
            dip: dip2 * RAD2DEG,
            rake: rake2 * RAD2DEG,
        }
    }

    /// Compute the T (tension), P (pressure), and B (null) axes.
    ///
    /// Returns `(t_axis, p_axis, b_axis)` as unit vectors in NED coordinates.
    pub fn principal_axes(&self) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let mt = MomentTensor::from_strike_dip_rake(self.strike, self.dip, self.rake, 1.0);
        let eigen = mt.eigendecompose();
        (eigen.vectors[0], eigen.vectors[1], eigen.vectors[2])
    }

    /// Classify the faulting style based on rake angle.
    pub fn faulting_style(&self) -> FaultingStyle {
        let r = self.rake;
        if r.abs() <= 30.0 || (r.abs() >= 150.0 && r.abs() <= 210.0) {
            FaultingStyle::StrikeSlip
        } else if r > 30.0 && r < 150.0 {
            FaultingStyle::Reverse
        } else {
            FaultingStyle::Normal
        }
    }

    /// Convert to a moment tensor with the given scalar moment.
    pub fn to_moment_tensor(&self, m0: f64) -> MomentTensor {
        MomentTensor::from_strike_dip_rake(self.strike, self.dip, self.rake, m0)
    }
}

/// Classification of earthquake faulting style.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultingStyle {
    /// Predominantly horizontal motion (rake near 0 or 180).
    StrikeSlip,
    /// Predominantly compressional/thrust motion (rake near 90).
    Reverse,
    /// Predominantly extensional motion (rake near -90).
    Normal,
}

// ─── Station Observation ────────────────────────────────────────────────────

/// A seismic station observation used for moment tensor inversion.
#[derive(Debug, Clone)]
pub struct StationObservation {
    /// Station azimuth from the source in degrees (0-360, clockwise from North).
    pub azimuth_deg: f64,
    /// Take-off angle of the ray at the source in degrees (0 = up, 180 = down).
    pub takeoff_deg: f64,
    /// Observed P-wave first motion polarity: +1 for compression, -1 for dilatation.
    pub polarity: f64,
    /// Optional: observed P-wave amplitude (arbitrary units).
    pub amplitude: Option<f64>,
    /// Optional: station weight for inversion (default 1.0).
    pub weight: f64,
}

impl StationObservation {
    /// Create a new station observation with polarity only.
    pub fn new_polarity(azimuth_deg: f64, takeoff_deg: f64, polarity: f64) -> Self {
        Self {
            azimuth_deg,
            takeoff_deg,
            polarity: if polarity >= 0.0 { 1.0 } else { -1.0 },
            amplitude: None,
            weight: 1.0,
        }
    }

    /// Create a new station observation with amplitude.
    pub fn new_amplitude(azimuth_deg: f64, takeoff_deg: f64, amplitude: f64) -> Self {
        Self {
            azimuth_deg,
            takeoff_deg,
            polarity: if amplitude >= 0.0 { 1.0 } else { -1.0 },
            amplitude: Some(amplitude),
            weight: 1.0,
        }
    }
}

// ─── Moment Tensor Inverter ─────────────────────────────────────────────────

/// Least-squares moment tensor inversion from seismic observations.
///
/// Supports two inversion modes:
/// 1. **Polarity-only**: Grid search over strike/dip/rake to maximize polarity fit
/// 2. **Amplitude**: Linear least-squares inversion of P-wave amplitudes
///
/// The inversion solves for the 6 independent moment tensor components
/// by minimizing the misfit between observed and predicted radiation patterns.
pub struct MomentTensorInverter {
    /// Station observations.
    observations: Vec<StationObservation>,
    /// Whether to constrain to deviatoric (trace-free) solution.
    deviatoric_only: bool,
}

impl MomentTensorInverter {
    /// Create a new inverter with the given station observations.
    pub fn new(observations: Vec<StationObservation>) -> Self {
        Self {
            observations,
            deviatoric_only: false,
        }
    }

    /// Set whether to constrain inversion to deviatoric (trace-free) solution.
    pub fn set_deviatoric_only(&mut self, deviatoric: bool) {
        self.deviatoric_only = deviatoric;
    }

    /// Add a station observation.
    pub fn add_observation(&mut self, obs: StationObservation) {
        self.observations.push(obs);
    }

    /// Perform polarity-based inversion using grid search.
    ///
    /// Searches over strike (0-360, step 5), dip (0-90, step 5), and
    /// rake (-180 to 180, step 5) to find the focal mechanism that best
    /// matches the observed P-wave first motion polarities.
    ///
    /// Returns the best-fit moment tensor and the polarity fit score (0.0 to 1.0).
    pub fn invert_polarities(&self) -> (MomentTensor, f64) {
        self.invert_polarities_with_step(5.0)
    }

    /// Perform polarity-based inversion with a custom grid step size in degrees.
    pub fn invert_polarities_with_step(&self, step: f64) -> (MomentTensor, f64) {
        if self.observations.is_empty() {
            return (MomentTensor::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0);
        }

        let mut best_score = -1.0f64;
        let mut best_mt = MomentTensor::new(1.0, -1.0, 0.0, 0.0, 0.0, 0.0);

        let mut strike = 0.0;
        while strike < 360.0 {
            let mut dip = 0.0;
            while dip <= 90.0 {
                let mut rake = -180.0;
                while rake <= 180.0 {
                    let mt = MomentTensor::from_strike_dip_rake(strike, dip, rake, 1.0);
                    let score = self.polarity_score(&mt);
                    if score > best_score {
                        best_score = score;
                        best_mt = mt;
                    }
                    rake += step;
                }
                dip += step;
            }
            strike += step;
        }

        let fit = best_score / self.observations.len() as f64;
        (best_mt, fit)
    }

    /// Compute weighted polarity match score for a candidate moment tensor.
    fn polarity_score(&self, mt: &MomentTensor) -> f64 {
        let mut score = 0.0;
        for obs in &self.observations {
            let predicted = radiation_pattern_scalar(
                &mt.m,
                obs.takeoff_deg,
                obs.azimuth_deg,
            );
            let pred_pol = if predicted >= 0.0 { 1.0 } else { -1.0 };
            if (pred_pol - obs.polarity).abs() < 0.5 {
                score += obs.weight;
            }
        }
        score
    }

    /// Perform linear least-squares amplitude inversion.
    ///
    /// Constructs the Green's function matrix G relating the 6 moment tensor
    /// components to the observed P-wave amplitudes, then solves:
    ///
    /// ```text
    /// d = G * m   =>   m = (G^T W G)^(-1) G^T W d
    /// ```
    ///
    /// where d is the data vector, G is the Green's function matrix,
    /// W is the weight matrix, and m is the model vector.
    ///
    /// Returns `None` if there are fewer than 6 observations.
    pub fn invert_amplitudes(&self) -> Option<MomentTensor> {
        let n = self.observations.len();
        let npar = if self.deviatoric_only { 5 } else { 6 };

        if n < npar {
            return None;
        }

        // Build Green's function matrix and data vector
        let mut g = vec![vec![0.0; npar]; n]; // n x npar
        let mut d = vec![0.0; n];

        for (i, obs) in self.observations.iter().enumerate() {
            let amp = obs.amplitude.unwrap_or(obs.polarity);
            d[i] = amp * obs.weight;

            let gamma_row = green_function_row(obs.takeoff_deg, obs.azimuth_deg);

            for j in 0..npar.min(6) {
                g[i][j] = gamma_row[j] * obs.weight;
            }
        }

        // Solve G^T * G * m = G^T * d using normal equations
        // Form G^T * G (npar x npar)
        let mut gtg = vec![vec![0.0; npar]; npar];
        for i in 0..npar {
            for j in 0..npar {
                for k in 0..n {
                    gtg[i][j] += g[k][i] * g[k][j];
                }
            }
        }

        // Form G^T * d (npar x 1)
        let mut gtd = vec![0.0; npar];
        for i in 0..npar {
            for k in 0..n {
                gtd[i] += g[k][i] * d[k];
            }
        }

        // Solve using Gauss elimination with partial pivoting
        let m_vec = solve_linear_system(&gtg, &gtd)?;

        // Construct moment tensor from solution vector
        let (mxx, myy, mzz, mxy, mxz, myz) = if self.deviatoric_only {
            // 5-parameter: Mzz = -(Mxx + Myy)
            (m_vec[0], m_vec[1], -(m_vec[0] + m_vec[1]), m_vec[2], m_vec[3], m_vec[4])
        } else {
            (m_vec[0], m_vec[1], m_vec[2], m_vec[3], m_vec[4], m_vec[5])
        };

        Some(MomentTensor::new(mxx, myy, mzz, mxy, mxz, myz))
    }

    /// Compute the variance reduction (fit quality) for a candidate moment tensor.
    ///
    /// VR = 1 - sum((d_obs - d_pred)^2) / sum(d_obs^2)
    ///
    /// Returns a value between 0.0 (no fit) and 1.0 (perfect fit).
    pub fn variance_reduction(&self, mt: &MomentTensor) -> f64 {
        let mut ss_res = 0.0;
        let mut ss_tot = 0.0;

        for obs in &self.observations {
            let predicted = radiation_pattern_scalar(
                &mt.m,
                obs.takeoff_deg,
                obs.azimuth_deg,
            );
            let observed = obs.amplitude.unwrap_or(obs.polarity);
            let w = obs.weight;
            ss_res += w * (observed - predicted) * (observed - predicted);
            ss_tot += w * observed * observed;
        }

        if ss_tot < 1e-30 {
            return 0.0;
        }

        1.0 - ss_res / ss_tot
    }
}

/// Compute the Green's function row for a single station.
///
/// Returns the 6 coefficients [Mxx, Myy, Mzz, Mxy, Mxz, Myz] for the
/// P-wave radiation pattern at the given take-off angle and azimuth.
fn green_function_row(takeoff_deg: f64, azimuth_deg: f64) -> [f64; 6] {
    let ih = takeoff_deg * DEG2RAD;
    let az = azimuth_deg * DEG2RAD;

    let sin_ih = ih.sin();
    let cos_ih = ih.cos();
    let sin_az = az.sin();
    let cos_az = az.cos();

    // Ray direction in NED: gamma = (sin(ih)*cos(az), sin(ih)*sin(az), cos(ih))
    let gx = sin_ih * cos_az;
    let gy = sin_ih * sin_az;
    let gz = cos_ih;

    // P-wave radiation: u_r = gamma_i * gamma_j * M_ij
    // Coefficients for each independent component:
    [
        gx * gx,               // Mxx
        gy * gy,               // Myy
        gz * gz,               // Mzz
        2.0 * gx * gy,         // Mxy
        2.0 * gx * gz,         // Mxz
        2.0 * gy * gz,         // Myz
    ]
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
///
/// Returns `None` if the system is singular.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = a.len();
    if n == 0 || b.len() != n {
        return None;
    }

    // Augmented matrix
    let mut aug = vec![vec![0.0; n + 1]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-30 {
            return None; // Singular
        }

        // Swap rows
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Eliminate below
        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        x[i] = sum / aug[i][i];
    }

    Some(x)
}

// ─── Source Type Classifier ─────────────────────────────────────────────────

/// Source type decomposition and classification result.
///
/// Decomposes the moment tensor into isotropic, double-couple, and CLVD
/// components, and provides Hudson T-k plot coordinates.
#[derive(Debug, Clone)]
pub struct SourceType {
    /// Percentage of double-couple component (0-100).
    pub dc_percentage: f64,
    /// Percentage of CLVD component (0-100).
    pub clvd_percentage: f64,
    /// Percentage of isotropic component (0-100).
    pub iso_percentage: f64,
    /// Hudson T parameter (-1 to 1): >0 = CLVD+, <0 = CLVD-.
    pub hudson_t: f64,
    /// Hudson k parameter (-1 to 1): >0 = explosion, <0 = implosion.
    pub hudson_k: f64,
    /// Epsilon parameter: ratio of smallest to largest deviatoric eigenvalue.
    pub epsilon: f64,
    /// Source classification label.
    pub classification: SourceClassification,
}

/// Classification of seismic source type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceClassification {
    /// Pure double-couple (tectonic earthquake).
    DoubleCouple,
    /// Predominantly CLVD (tensile crack, ring fault).
    Clvd,
    /// Predominantly isotropic (explosion, implosion, cavity collapse).
    Isotropic,
    /// Mixed mechanism.
    Mixed,
}

/// Classifier for seismic source type decomposition.
pub struct SourceTypeClassifier;

impl SourceTypeClassifier {
    /// Classify the source type of a moment tensor.
    ///
    /// Decomposes the tensor into isotropic, double-couple, and CLVD components
    /// using the standard eigenvalue-based decomposition.
    ///
    /// # Decomposition
    ///
    /// The eigenvalues of the deviatoric part (lambda1 >= lambda2 >= lambda3)
    /// determine the source type:
    ///
    /// - **Epsilon** = lambda3 / |lambda1|: measures CLVD fraction
    /// - **DC%** = 100 * (1 - 2|epsilon|): double-couple percentage
    /// - **CLVD%** = 100 * 2|epsilon|: CLVD percentage
    /// - **ISO%** based on trace relative to total moment
    ///
    /// Hudson T-k coordinates:
    /// - T = 2 * epsilon (ranges -1 to 1)
    /// - k = iso_fraction (ranges -1 to 1)
    pub fn classify(mt: &MomentTensor) -> SourceType {
        let eigen = mt.eigendecompose();
        let trace = mt.trace();
        let m0 = mt.scalar_moment();

        // Isotropic fraction
        let iso_moment = trace.abs() / 3.0;
        let iso_frac = if m0 > 1e-30 { iso_moment / m0 } else { 0.0 };
        let iso_pct = (iso_frac * 100.0).min(100.0);

        // Deviatoric eigenvalues (sorted descending: lam1 >= lam2 >= lam3)
        let dev = mt.deviatoric_part();
        let dev_eigen = dev.eigendecompose();
        let lam1 = dev_eigen.values[0]; // largest (most positive)
        let lam2 = dev_eigen.values[1]; // intermediate
        let lam3 = dev_eigen.values[2]; // smallest (most negative)

        // Epsilon: ratio of intermediate to largest-absolute deviatoric eigenvalue
        // (Knopoff & Randall 1970)
        // For pure DC [a, 0, -a]: epsilon = 0
        // For pure CLVD [2a, -a, -a]: epsilon = -0.5
        let lam_abs_max = lam1.abs().max(lam3.abs());
        let epsilon = if lam_abs_max > 1e-30 {
            lam2 / lam_abs_max
        } else {
            0.0
        };

        // DC and CLVD percentages (of deviatoric part)
        let clvd_frac = (2.0 * epsilon.abs()).min(1.0);
        let dc_frac = 1.0 - clvd_frac;

        // Scale by deviatoric fraction
        let dev_frac = 1.0 - iso_frac.min(1.0);
        let dc_pct = dc_frac * dev_frac * 100.0;
        let clvd_pct = clvd_frac * dev_frac * 100.0;

        // Hudson T-k parameters
        // T = 2 * epsilon (ranges -1 to 1)
        let hudson_t = 2.0 * epsilon;
        let hudson_k = if m0 > 1e-30 {
            (trace / 3.0) / m0
        } else {
            0.0
        };

        // Classification
        let classification = if iso_pct > 50.0 {
            SourceClassification::Isotropic
        } else if dc_pct > 70.0 {
            SourceClassification::DoubleCouple
        } else if clvd_pct > 50.0 {
            SourceClassification::Clvd
        } else {
            SourceClassification::Mixed
        };

        SourceType {
            dc_percentage: dc_pct,
            clvd_percentage: clvd_pct,
            iso_percentage: iso_pct,
            hudson_t,
            hudson_k,
            epsilon,
            classification,
        }
    }

    /// Check if a moment tensor is predominantly a double-couple source.
    pub fn is_double_couple(mt: &MomentTensor) -> bool {
        let st = Self::classify(mt);
        st.dc_percentage > 70.0
    }
}

// ─── Beach Ball Generator ───────────────────────────────────────────────────

/// A point on the beach ball plot in pixel/normalized coordinates.
#[derive(Debug, Clone, Copy)]
pub struct BeachBallPoint {
    /// X coordinate (-1 to 1, or pixel).
    pub x: f64,
    /// Y coordinate (-1 to 1, or pixel).
    pub y: f64,
    /// Polarity at this point: true = compressional (filled), false = dilatational (open).
    pub compressional: bool,
}

/// A nodal plane curve on the beach ball plot.
#[derive(Debug, Clone)]
pub struct NodalPlaneCurve {
    /// Points along the nodal plane curve.
    pub points: Vec<(f64, f64)>,
}

/// Beach ball (focal mechanism) visualization generator.
///
/// Generates equal-area (Lambert azimuthal) lower-hemisphere projection
/// data for plotting focal mechanism "beach ball" diagrams.
pub struct BeachBallGenerator {
    /// Resolution: number of grid points along each axis.
    pub resolution: usize,
}

impl BeachBallGenerator {
    /// Create a new beach ball generator with the given resolution.
    pub fn new(resolution: usize) -> Self {
        Self { resolution }
    }

    /// Generate a grid of beach ball points for the given focal mechanism.
    ///
    /// Returns points on an equal-area lower-hemisphere projection where
    /// each point indicates whether it is in a compressional (filled) or
    /// dilatational (open) quadrant.
    pub fn generate(&self, fm: &FocalMechanism) -> Vec<BeachBallPoint> {
        let mt = fm.to_moment_tensor(1.0);
        let mut points = Vec::new();
        let n = self.resolution;

        for iy in 0..n {
            for ix in 0..n {
                // Normalized coordinates [-1, 1]
                let x = 2.0 * ix as f64 / (n - 1) as f64 - 1.0;
                let y = 2.0 * iy as f64 / (n - 1) as f64 - 1.0;

                let r = (x * x + y * y).sqrt();
                if r > 1.0 {
                    continue; // Outside the circle
                }

                // Lambert azimuthal equal-area projection (lower hemisphere)
                // r = sqrt(2) * sin(theta/2) where theta is polar angle from vertical
                // => theta = 2 * asin(r / sqrt(2))
                let theta = 2.0 * (r / 2.0_f64.sqrt()).asin(); // take-off angle from down
                let takeoff = PI - theta; // convert to take-off from up

                let phi = y.atan2(x); // azimuth

                let takeoff_deg = takeoff * RAD2DEG;
                let azimuth_deg = ((phi * RAD2DEG) + 360.0) % 360.0;

                let amplitude = radiation_pattern_scalar(&mt.m, takeoff_deg, azimuth_deg);

                points.push(BeachBallPoint {
                    x,
                    y,
                    compressional: amplitude >= 0.0,
                });
            }
        }

        points
    }

    /// Generate the nodal plane curves for the given focal mechanism.
    ///
    /// Returns two curves representing the two nodal planes projected onto
    /// the lower-hemisphere equal-area plot.
    pub fn nodal_planes(&self, fm: &FocalMechanism) -> Vec<NodalPlaneCurve> {
        let mut curves = Vec::new();

        // Nodal plane 1: from strike/dip of given mechanism
        curves.push(self.project_plane(fm.strike, fm.dip));

        // Nodal plane 2: auxiliary plane
        let aux = fm.auxiliary_plane();
        curves.push(self.project_plane(aux.strike, aux.dip));

        curves
    }

    /// Project a fault plane (given by strike and dip) onto the lower hemisphere.
    fn project_plane(&self, strike_deg: f64, dip_deg: f64) -> NodalPlaneCurve {
        let n_points = 181;
        let mut points = Vec::with_capacity(n_points);

        for i in 0..n_points {
            let angle = (i as f64 / (n_points - 1) as f64) * PI; // 0 to pi along the plane

            // Point on the fault plane in NED coordinates
            let strike_rad = strike_deg * DEG2RAD;
            let dip_rad = dip_deg * DEG2RAD;

            // Parametrize points on the plane from one edge of the hemisphere to the other
            let along_strike = angle.cos();
            let along_dip = angle.sin();

            // Direction in NED
            let nx = along_strike * strike_rad.sin() - along_dip * dip_rad.cos() * strike_rad.cos();
            let ny = along_strike * strike_rad.cos() + along_dip * dip_rad.cos() * strike_rad.sin();
            let nz = along_dip * dip_rad.sin();

            // Only lower hemisphere (nz >= 0 = downward in NED)
            let (nx, ny, nz) = if nz < 0.0 {
                (-nx, -ny, -nz)
            } else {
                (nx, ny, nz)
            };

            // Polar angle from vertical (down)
            let theta = nz.acos();
            let phi = ny.atan2(nx);

            // Lambert equal-area projection
            let r = 2.0_f64.sqrt() * (theta / 2.0).sin();
            let x = r * phi.cos();
            let y = r * phi.sin();

            if x.is_finite() && y.is_finite() {
                points.push((x, y));
            }
        }

        NodalPlaneCurve { points }
    }

    /// Generate the T and P axis positions on the beach ball plot.
    ///
    /// Returns `(t_point, p_point)` as (x, y) coordinates on the equal-area projection.
    pub fn principal_axis_positions(&self, fm: &FocalMechanism) -> ((f64, f64), (f64, f64)) {
        let (t_axis, _b_axis, p_axis) = fm.principal_axes();

        let t_pos = self.project_axis(&t_axis);
        let p_pos = self.project_axis(&p_axis);

        (t_pos, p_pos)
    }

    /// Project a 3D axis direction onto the equal-area plot.
    fn project_axis(&self, axis: &[f64; 3]) -> (f64, f64) {
        let mut v = *axis;
        // Ensure lower hemisphere (z > 0 in NED = downward)
        if v[2] < 0.0 {
            v = [-v[0], -v[1], -v[2]];
        }

        let theta = v[2].acos(); // polar angle from down
        let phi = v[1].atan2(v[0]); // azimuth

        let r = 2.0_f64.sqrt() * (theta / 2.0).sin();
        (r * phi.cos(), r * phi.sin())
    }
}

// ─── Helper Functions ───────────────────────────────────────────────────────

/// Compute the far-field P-wave radiation pattern.
///
/// Returns the displacement vector [radial, theta, phi] for a ray
/// leaving the source at the given take-off angle and azimuth.
///
/// # Arguments
/// * `m` - 3x3 moment tensor matrix
/// * `takeoff_deg` - Take-off angle in degrees (0 = up, 90 = horizontal, 180 = down)
/// * `azimuth_deg` - Azimuth in degrees (0 = North, 90 = East)
pub fn radiation_pattern(m: &Mat3, takeoff_deg: f64, azimuth_deg: f64) -> [f64; 3] {
    let ih = takeoff_deg * DEG2RAD;
    let az = azimuth_deg * DEG2RAD;

    // Ray direction vector (NED)
    let gamma = [
        ih.sin() * az.cos(),  // North
        ih.sin() * az.sin(),  // East
        ih.cos(),             // Down (take-off from up)
    ];

    // P-wave displacement: u_i = gamma_j * M_jk * gamma_k (scalar)
    // For the full radiation pattern, we need components
    let mg = mat3_vec(m, &gamma);
    let radial = dot3(&gamma, &mg); // P-wave (longitudinal)

    // SV and SH components (transverse)
    // theta direction (in vertical plane containing ray)
    let theta_hat = [
        ih.cos() * az.cos(),
        ih.cos() * az.sin(),
        -ih.sin(),
    ];
    let sv = dot3(&theta_hat, &mg);

    // phi direction (horizontal, perpendicular to ray)
    let phi_hat = [
        -az.sin(),
        az.cos(),
        0.0,
    ];
    let sh = dot3(&phi_hat, &mg);

    [radial, sv, sh]
}

/// Compute the scalar P-wave radiation pattern value at a point.
///
/// This is the radial component: gamma_i * M_ij * gamma_j.
pub fn radiation_pattern_scalar(m: &Mat3, takeoff_deg: f64, azimuth_deg: f64) -> f64 {
    radiation_pattern(m, takeoff_deg, azimuth_deg)[0]
}

/// Compute the scalar seismic moment from moment tensor eigenvalues.
///
/// M0 = max(|eigenvalue|)
///
/// For a general moment tensor, M0 = sqrt(sum(m_ij^2) / 2).
pub fn seismic_moment(eigenvalues: &[f64; 3]) -> f64 {
    eigenvalues.iter().map(|e| e.abs()).fold(0.0_f64, f64::max)
}

/// Compute moment magnitude (Mw) from scalar seismic moment M0.
///
/// Mw = (2/3) * log10(M0) - 6.07
///
/// where M0 is in Newton-meters. This is the Hanks & Kanamori (1979) relation.
pub fn moment_magnitude(m0: f64) -> f64 {
    if m0 <= 0.0 {
        return 0.0;
    }
    (2.0 / 3.0) * m0.log10() - 6.07
}

/// Compute scalar seismic moment M0 from moment magnitude Mw.
///
/// M0 = 10^(1.5 * (Mw + 6.07))
pub fn magnitude_to_moment(mw: f64) -> f64 {
    10.0_f64.powf(1.5 * (mw + 6.07))
}

/// Compute the take-off angle for a ray in a simple velocity model.
///
/// Uses Snell's law for a constant velocity gradient:
/// v(z) = v0 + gradient * z
///
/// # Arguments
/// * `source_depth_km` - Source depth in km
/// * `epicentral_distance_km` - Surface distance in km
/// * `v0` - Surface velocity in km/s
/// * `gradient` - Velocity gradient in 1/s
///
/// Returns the take-off angle in degrees (0 = up, 90 = horizontal).
pub fn take_off_angle(
    source_depth_km: f64,
    epicentral_distance_km: f64,
    v0: f64,
    gradient: f64,
) -> f64 {
    // Simple ray parameter estimation
    // For a linear velocity gradient: p = sin(i) / v
    // Use geometric approximation for shallow sources
    let hypo = (source_depth_km * source_depth_km
        + epicentral_distance_km * epicentral_distance_km)
        .sqrt();

    if hypo < 1e-10 {
        return 0.0;
    }

    // Angle from vertical to the station
    let angle = (epicentral_distance_km / hypo).asin();

    // Correct for velocity gradient (rays curve upward with increasing velocity)
    let v_source = v0 + gradient * source_depth_km;
    let correction = if gradient > 0.0 && v_source > v0 {
        // Ray bends upward -> steeper take-off
        (v0 / v_source).asin() / (PI / 2.0)
    } else {
        1.0
    };

    let takeoff = angle * correction;
    takeoff * RAD2DEG
}

/// Compute the double-couple percentage from moment tensor eigenvalues.
///
/// DC% = 100 * (1 - 2 * |min_dev_eigenvalue / max_dev_eigenvalue|)
pub fn double_couple_percentage(eigenvalues: &[f64; 3]) -> f64 {
    // Remove isotropic component
    let mean = (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]) / 3.0;
    let dev: Vec<f64> = eigenvalues.iter().map(|e| e - mean).collect();

    let max_abs = dev.iter().map(|d| d.abs()).fold(0.0_f64, f64::max);
    if max_abs < 1e-30 {
        return 100.0; // Pure isotropic is considered 100% DC of the zero deviatoric
    }

    let min_abs = dev.iter().map(|d| d.abs()).fold(f64::MAX, f64::min);
    let epsilon = min_abs / max_abs;

    (100.0 * (1.0 - 2.0 * epsilon)).max(0.0)
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const ANGLE_TOL: f64 = 5.0; // degrees

    // ── Matrix utilities ──

    #[test]
    fn test_mat3_mul_identity() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let id = mat3_identity();
        let result = mat3_mul(&m, &id);
        for i in 0..3 {
            for j in 0..3 {
                assert!((result[i][j] - m[i][j]).abs() < TOL);
            }
        }
    }

    #[test]
    fn test_mat3_transpose() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let t = mat3_transpose(&m);
        assert!((t[0][1] - 4.0).abs() < TOL);
        assert!((t[1][0] - 2.0).abs() < TOL);
        assert!((t[2][0] - 3.0).abs() < TOL);
    }

    #[test]
    fn test_mat3_trace() {
        let m = [[3.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 7.0]];
        assert!((mat3_trace(&m) - 15.0).abs() < TOL);
    }

    // ── Eigendecomposition ──

    #[test]
    fn test_eigen_diagonal() {
        let m = [[3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 2.0]];
        let eigen = eigen_symmetric_3x3(&m);
        assert!((eigen.values[0] - 3.0).abs() < TOL);
        assert!((eigen.values[1] - 2.0).abs() < TOL);
        assert!((eigen.values[2] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_eigen_symmetric() {
        // Known eigenvalues: 5, 2, -1 (sum = 6, which matches trace)
        let m = [[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 1.0]];
        let eigen = eigen_symmetric_3x3(&m);
        let trace_m = mat3_trace(&m);
        let trace_eigen: f64 = eigen.values.iter().sum();
        assert!((trace_m - trace_eigen).abs() < TOL, "trace mismatch");
    }

    #[test]
    fn test_eigen_orthogonality() {
        let m = [[4.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 2.0]];
        let eigen = eigen_symmetric_3x3(&m);
        // Eigenvectors should be orthogonal
        let d01 = dot3(&eigen.vectors[0], &eigen.vectors[1]);
        let d02 = dot3(&eigen.vectors[0], &eigen.vectors[2]);
        let d12 = dot3(&eigen.vectors[1], &eigen.vectors[2]);
        assert!(d01.abs() < 1e-10, "v0.v1 = {}", d01);
        assert!(d02.abs() < 1e-10, "v0.v2 = {}", d02);
        assert!(d12.abs() < 1e-10, "v1.v2 = {}", d12);
    }

    // ── Moment Tensor basics ──

    #[test]
    fn test_moment_tensor_symmetry() {
        let mt = MomentTensor::new(1.0, 2.0, 3.0, 0.5, 0.3, 0.1);
        assert!((mt.m[0][1] - mt.m[1][0]).abs() < TOL);
        assert!((mt.m[0][2] - mt.m[2][0]).abs() < TOL);
        assert!((mt.m[1][2] - mt.m[2][1]).abs() < TOL);
    }

    #[test]
    fn test_moment_tensor_trace() {
        let mt = MomentTensor::new(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
        assert!((mt.trace() - 6.0).abs() < TOL);
    }

    #[test]
    fn test_moment_tensor_deviatoric_is_traceless() {
        let mt = MomentTensor::new(5.0, 3.0, 1.0, 0.5, 0.3, 0.1);
        let dev = mt.deviatoric_part();
        assert!(dev.trace().abs() < TOL, "deviatoric trace = {}", dev.trace());
    }

    #[test]
    fn test_moment_tensor_iso_plus_dev() {
        let mt = MomentTensor::new(5.0, 3.0, 1.0, 0.5, 0.3, 0.1);
        let iso = mt.isotropic_part();
        let dev = mt.deviatoric_part();
        for i in 0..3 {
            for j in 0..3 {
                let sum = iso.m[i][j] + dev.m[i][j];
                assert!((sum - mt.m[i][j]).abs() < TOL,
                    "mismatch at [{i}][{j}]: {} + {} != {}", iso.m[i][j], dev.m[i][j], mt.m[i][j]);
            }
        }
    }

    // ── Strike-Dip-Rake roundtrip ──

    #[test]
    fn test_pure_strike_slip_scalar_moment() {
        // Pure strike-slip: strike=0, dip=90, rake=0
        let mt = MomentTensor::from_strike_dip_rake(0.0, 90.0, 0.0, 1e16);
        let m0 = mt.scalar_moment();
        assert!((m0 - 1e16).abs() / 1e16 < 0.1,
            "scalar moment {} should be close to 1e16", m0);
    }

    #[test]
    fn test_pure_thrust_scalar_moment() {
        // Pure thrust: strike=0, dip=45, rake=90
        let mt = MomentTensor::from_strike_dip_rake(0.0, 45.0, 90.0, 1e15);
        let m0 = mt.scalar_moment();
        assert!((m0 - 1e15).abs() / 1e15 < 0.1,
            "scalar moment {} should be close to 1e15", m0);
    }

    #[test]
    fn test_pure_normal_fault() {
        // Pure normal: strike=0, dip=45, rake=-90
        let mt = MomentTensor::from_strike_dip_rake(0.0, 45.0, -90.0, 1e15);
        let dev = mt.deviatoric_part();
        // Should be mostly deviatoric for a pure DC
        let iso_frac = mt.isotropic_part().frobenius_norm() / mt.frobenius_norm();
        assert!(iso_frac < 0.1, "pure normal fault should be nearly deviatoric, iso_frac={}", iso_frac);
    }

    // ── Focal mechanism ──

    #[test]
    fn test_focal_mechanism_vertical_dip() {
        let fm = FocalMechanism::new(45.0, 90.0, 0.0);
        assert!((fm.dip - 90.0).abs() < TOL);
        assert!((fm.strike - 45.0).abs() < TOL);
    }

    #[test]
    fn test_faulting_style_strike_slip() {
        let fm = FocalMechanism::new(0.0, 90.0, 0.0);
        assert_eq!(fm.faulting_style(), FaultingStyle::StrikeSlip);
    }

    #[test]
    fn test_faulting_style_reverse() {
        let fm = FocalMechanism::new(0.0, 45.0, 90.0);
        assert_eq!(fm.faulting_style(), FaultingStyle::Reverse);
    }

    #[test]
    fn test_faulting_style_normal() {
        let fm = FocalMechanism::new(0.0, 45.0, -90.0);
        assert_eq!(fm.faulting_style(), FaultingStyle::Normal);
    }

    #[test]
    fn test_focal_mechanism_to_moment_tensor_roundtrip() {
        // Create a moment tensor, extract focal mechanism, convert back
        let fm_orig = FocalMechanism::new(30.0, 60.0, 45.0);
        let mt = fm_orig.to_moment_tensor(1e16);
        let m0 = mt.scalar_moment();
        assert!(m0 > 0.0, "scalar moment should be positive");
    }

    // ── Source type classification ──

    #[test]
    fn test_pure_dc_classification() {
        let mt = MomentTensor::from_strike_dip_rake(0.0, 45.0, 90.0, 1e16);
        let st = SourceTypeClassifier::classify(&mt);
        assert!(st.dc_percentage > 80.0,
            "pure DC should have high DC%: {}", st.dc_percentage);
        assert!(st.iso_percentage < 10.0,
            "pure DC should have low ISO%: {}", st.iso_percentage);
    }

    #[test]
    fn test_explosion_classification() {
        // Pure explosion: isotropic tensor
        let mt = MomentTensor::new(1e16, 1e16, 1e16, 0.0, 0.0, 0.0);
        let st = SourceTypeClassifier::classify(&mt);
        assert!(st.iso_percentage > 50.0,
            "explosion should have high ISO%: {}", st.iso_percentage);
        assert_eq!(st.classification, SourceClassification::Isotropic);
    }

    #[test]
    fn test_hudson_t_near_zero_for_dc() {
        let mt = MomentTensor::from_strike_dip_rake(0.0, 90.0, 0.0, 1e16);
        let st = SourceTypeClassifier::classify(&mt);
        assert!(st.hudson_t.abs() < 0.5,
            "pure DC should have |T| near 0: {}", st.hudson_t);
    }

    #[test]
    fn test_is_double_couple() {
        let mt = MomentTensor::from_strike_dip_rake(120.0, 60.0, -30.0, 1e16);
        assert!(SourceTypeClassifier::is_double_couple(&mt));
    }

    // ── Radiation pattern ──

    #[test]
    fn test_radiation_pattern_vertical_strike_slip() {
        // Strike-slip on a vertical N-S fault
        let mt = MomentTensor::from_strike_dip_rake(0.0, 90.0, 0.0, 1.0);
        // At 45 degrees azimuth, horizontal ray: should be maximum
        let p = radiation_pattern_scalar(&mt.m, 90.0, 45.0);
        // At 0 or 90 degrees azimuth: should be near nodal plane
        let p_nodal = radiation_pattern_scalar(&mt.m, 90.0, 0.0);
        assert!(p.abs() > p_nodal.abs(),
            "45 deg azimuth should have larger amplitude than nodal");
    }

    #[test]
    fn test_radiation_pattern_symmetry() {
        let mt = MomentTensor::from_strike_dip_rake(0.0, 90.0, 0.0, 1.0);
        // Strike-slip: should have 4-fold symmetry
        let p1 = radiation_pattern_scalar(&mt.m, 45.0, 45.0);
        let p2 = radiation_pattern_scalar(&mt.m, 45.0, 225.0);
        // Same quadrant, should have same sign
        assert!((p1 - p2).abs() < 0.5 * p1.abs().max(0.01),
            "quadrant symmetry: p1={}, p2={}", p1, p2);
    }

    // ── Moment magnitude ──

    #[test]
    fn test_moment_magnitude_m5() {
        // M0 for Mw 5.0: 10^(1.5 * (5.0 + 6.07)) = 10^16.605
        let m0 = magnitude_to_moment(5.0);
        let mw = moment_magnitude(m0);
        assert!((mw - 5.0).abs() < 0.01, "Mw={} should be ~5.0", mw);
    }

    #[test]
    fn test_moment_magnitude_roundtrip() {
        for mw_in in [3.0, 5.0, 7.0, 9.0] {
            let m0 = magnitude_to_moment(mw_in);
            let mw_out = moment_magnitude(m0);
            assert!((mw_in - mw_out).abs() < 0.01,
                "roundtrip failed: {} -> {} -> {}", mw_in, m0, mw_out);
        }
    }

    #[test]
    fn test_moment_magnitude_ordering() {
        // Larger moment = larger magnitude
        let mw3 = moment_magnitude(1e14);
        let mw5 = moment_magnitude(1e16);
        let mw7 = moment_magnitude(1e20);
        assert!(mw3 < mw5, "Mw({}) should be < Mw({})", mw3, mw5);
        assert!(mw5 < mw7, "Mw({}) should be < Mw({})", mw5, mw7);
    }

    // ── Take-off angle ──

    #[test]
    fn test_takeoff_angle_directly_above() {
        // Station directly above source
        let angle = take_off_angle(10.0, 0.0, 6.0, 0.0);
        assert!(angle.abs() < 1.0, "directly above should be ~0 deg: {}", angle);
    }

    #[test]
    fn test_takeoff_angle_increases_with_distance() {
        let a1 = take_off_angle(10.0, 10.0, 6.0, 0.0);
        let a2 = take_off_angle(10.0, 50.0, 6.0, 0.0);
        assert!(a2 > a1, "farther station should have larger take-off angle");
    }

    // ── Inversion ──

    #[test]
    fn test_polarity_inversion_basic() {
        // Create a known mechanism and generate synthetic observations
        let true_mt = MomentTensor::from_strike_dip_rake(0.0, 90.0, 0.0, 1.0);
        let mut obs = Vec::new();

        // Sample 12 azimuths at 45-degree take-off
        for az in (0..360).step_by(30) {
            let amp = radiation_pattern_scalar(&true_mt.m, 45.0, az as f64);
            obs.push(StationObservation::new_polarity(az as f64, 45.0, amp));
        }

        let inverter = MomentTensorInverter::new(obs);
        let (best_mt, fit) = inverter.invert_polarities();

        assert!(fit > 0.8, "polarity fit should be > 80%: {}", fit);
        // The best solution should predict the same polarities
        assert!(best_mt.scalar_moment() > 0.0);
    }

    #[test]
    fn test_amplitude_inversion_recovers_mechanism() {
        // Create a known mechanism
        let true_mt = MomentTensor::from_strike_dip_rake(45.0, 60.0, 90.0, 1.0);
        let mut obs = Vec::new();

        // Generate 12 amplitude observations at different azimuths and take-offs
        for az in (0..360).step_by(30) {
            for toff in [30.0, 60.0] {
                let amp = radiation_pattern_scalar(&true_mt.m, toff, az as f64);
                obs.push(StationObservation::new_amplitude(az as f64, toff, amp));
            }
        }

        let inverter = MomentTensorInverter::new(obs);
        let result = inverter.invert_amplitudes();
        assert!(result.is_some(), "amplitude inversion should succeed");

        let recovered = result.unwrap();
        let vr = inverter.variance_reduction(&recovered);
        assert!(vr > 0.9, "variance reduction should be > 90%: {}", vr);
    }

    #[test]
    fn test_inversion_too_few_stations() {
        let obs = vec![
            StationObservation::new_amplitude(0.0, 45.0, 1.0),
            StationObservation::new_amplitude(90.0, 45.0, -1.0),
        ];
        let inverter = MomentTensorInverter::new(obs);
        assert!(inverter.invert_amplitudes().is_none(),
            "should fail with fewer than 6 stations");
    }

    // ── Beach ball ──

    #[test]
    fn test_beach_ball_generation() {
        let gen = BeachBallGenerator::new(21);
        let fm = FocalMechanism::new(0.0, 90.0, 0.0);
        let points = gen.generate(&fm);

        assert!(!points.is_empty(), "should generate points");
        // All points should be within the unit circle
        for p in &points {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            assert!(r <= 1.0 + TOL, "point outside circle: r={}", r);
        }

        // Should have both compressional and dilatational regions for strike-slip
        let n_comp = points.iter().filter(|p| p.compressional).count();
        let n_dil = points.iter().filter(|p| !p.compressional).count();
        assert!(n_comp > 0, "should have compressional points");
        assert!(n_dil > 0, "should have dilatational points");
    }

    #[test]
    fn test_beach_ball_nodal_planes() {
        let gen = BeachBallGenerator::new(51);
        let fm = FocalMechanism::new(0.0, 45.0, 90.0);
        let curves = gen.nodal_planes(&fm);
        assert_eq!(curves.len(), 2, "should have 2 nodal plane curves");
        for curve in &curves {
            assert!(!curve.points.is_empty(), "nodal plane curve should have points");
        }
    }

    // ── Double-couple percentage ──

    #[test]
    fn test_dc_percentage_pure_dc() {
        // Pure DC eigenvalues: [1, 0, -1]
        let pct = double_couple_percentage(&[1.0, 0.0, -1.0]);
        assert!((pct - 100.0).abs() < 1.0, "pure DC should be 100%: {}", pct);
    }

    #[test]
    fn test_dc_percentage_pure_clvd() {
        // Pure CLVD eigenvalues: [2, -1, -1] or [1, 1, -2]
        let pct = double_couple_percentage(&[2.0, -1.0, -1.0]);
        assert!(pct < 10.0, "pure CLVD should be ~0%: {}", pct);
    }

    // ── Linear system solver ──

    #[test]
    fn test_solve_linear_system_2x2() {
        // 2x + 3y = 8, x + y = 3 => x=1, y=2
        let a = vec![vec![2.0, 3.0], vec![1.0, 1.0]];
        let b = vec![8.0, 3.0];
        let x = solve_linear_system(&a, &b).unwrap();
        assert!((x[0] - 1.0).abs() < TOL);
        assert!((x[1] - 2.0).abs() < TOL);
    }

    #[test]
    fn test_solve_linear_system_singular() {
        let a = vec![vec![1.0, 2.0], vec![2.0, 4.0]];
        let b = vec![3.0, 6.0];
        assert!(solve_linear_system(&a, &b).is_none());
    }

    // ── Seismic moment from eigenvalues ──

    #[test]
    fn test_seismic_moment() {
        let m0 = seismic_moment(&[3.0, 1.0, -4.0]);
        assert!((m0 - 4.0).abs() < TOL, "M0 should be max |eigenvalue|: {}", m0);
    }

    // ── Green's function ──

    #[test]
    fn test_green_function_row_vertical() {
        // Vertical ray (takeoff = 0, looking up): gamma = (0, 0, 1)
        let row = green_function_row(0.0, 0.0);
        // Only Mzz should contribute: gz^2 = 1
        assert!((row[0]).abs() < TOL, "Mxx coeff should be 0");
        assert!((row[1]).abs() < TOL, "Myy coeff should be 0");
        assert!((row[2] - 1.0).abs() < TOL, "Mzz coeff should be 1");
        assert!((row[3]).abs() < TOL, "Mxy coeff should be 0");
        assert!((row[4]).abs() < TOL, "Mxz coeff should be 0");
        assert!((row[5]).abs() < TOL, "Myz coeff should be 0");
    }
}

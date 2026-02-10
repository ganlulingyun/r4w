//! Generalized Sidelobe Canceller (GSC) for adaptive interference nulling.
//!
//! The GSC is a constrained adaptive beamformer that decomposes the problem into
//! a fixed (quiescent) beamformer and an adaptive sidelobe canceller. This allows
//! maintaining a distortionless response toward the desired signal direction while
//! adaptively nulling interferers.
//!
//! Architecture:
//! ```text
//!   x(n) ──┬── wq^H ────────────(+)──► y(n)
//!           │                     (-)
//!           └── B^H ── wa^H ──────┘
//! ```
//!
//! Where `wq` is the quiescent (steering) weight vector, `B` is the blocking
//! matrix (orthogonal complement of the steering vector), and `wa` are the
//! adaptive weights updated via LMS or NLMS.
//!
//! # Example
//!
//! ```
//! use r4w_core::generalized_sidelobe_canceller::{
//!     GeneralizedSidelobeCanceller, GscConfig, AdaptationAlgorithm,
//! };
//! use std::f64::consts::PI;
//!
//! // 4-element ULA, look direction at broadside (0 degrees)
//! let config = GscConfig {
//!     num_elements: 4,
//!     look_direction_rad: 0.0,
//!     element_spacing_wavelengths: 0.5,
//!     step_size: 0.01,
//!     algorithm: AdaptationAlgorithm::Lms,
//! };
//! let mut gsc = GeneralizedSidelobeCanceller::new(config);
//!
//! // Generate a simple snapshot: desired signal from broadside
//! let snapshot: Vec<(f64, f64)> = (0..4).map(|_| (1.0, 0.0)).collect();
//! let output = gsc.process(&snapshot);
//! // Output should preserve the desired signal
//! assert!(output.0.abs() > 0.5);
//! ```

use std::f64::consts::PI;

// ── Complex arithmetic helpers ──────────────────────────────────────────────

/// Multiply two complex numbers represented as `(re, im)` tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Conjugate of a complex number.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers: a - b.
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Squared magnitude of a complex number.
#[inline]
fn cmag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Scale a complex number by a real scalar.
#[inline]
fn cscale(s: f64, a: (f64, f64)) -> (f64, f64) {
    (s * a.0, s * a.1)
}

/// Inner product: sum of conj(a_i) * b_i.
fn inner_product(a: &[(f64, f64)], b: &[(f64, f64)]) -> (f64, f64) {
    let mut acc = (0.0, 0.0);
    for (&ai, &bi) in a.iter().zip(b.iter()) {
        acc = cadd(acc, cmul(conj(ai), bi));
    }
    acc
}

// ── Public types ────────────────────────────────────────────────────────────

/// Adaptation algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AdaptationAlgorithm {
    /// Least Mean Squares with fixed step size.
    Lms,
    /// Normalized LMS with power-normalized step size.
    Nlms,
}

/// Configuration for the Generalized Sidelobe Canceller.
#[derive(Debug, Clone)]
pub struct GscConfig {
    /// Number of antenna array elements.
    pub num_elements: usize,
    /// Look direction in radians (0 = broadside for a ULA).
    pub look_direction_rad: f64,
    /// Element spacing in wavelengths (typically 0.5).
    pub element_spacing_wavelengths: f64,
    /// Adaptation step size (mu). For LMS, typical values are 0.001-0.05.
    pub step_size: f64,
    /// Which adaptation algorithm to use.
    pub algorithm: AdaptationAlgorithm,
}

/// A constraint specification for multi-constraint GSC designs.
#[derive(Debug, Clone)]
pub struct Constraint {
    /// Direction of the constraint in radians.
    pub direction_rad: f64,
    /// Desired complex response in that direction.
    pub response: (f64, f64),
}

/// Convergence statistics from the GSC.
#[derive(Debug, Clone)]
pub struct ConvergenceStats {
    /// Number of snapshots processed so far.
    pub iterations: usize,
    /// Current norm of the adaptive weight vector.
    pub weight_norm: f64,
    /// Exponentially averaged output power.
    pub output_power: f64,
    /// Estimated interference-to-noise ratio improvement in dB.
    pub inr_improvement_db: f64,
}

/// The Generalized Sidelobe Canceller.
#[derive(Debug, Clone)]
pub struct GeneralizedSidelobeCanceller {
    num_elements: usize,
    step_size: f64,
    algorithm: AdaptationAlgorithm,
    /// Quiescent weight vector (steering vector / N).
    wq: Vec<(f64, f64)>,
    /// Blocking matrix: (num_elements) x (num_elements - num_constraints) stored
    /// column-major. Each column has `num_elements` entries.
    blocking_matrix: Vec<Vec<(f64, f64)>>,
    /// Number of columns in the blocking matrix.
    num_aux: usize,
    /// Adaptive weight vector (length = num_aux).
    wa: Vec<(f64, f64)>,
    /// Convergence tracking.
    iterations: usize,
    output_power_avg: f64,
    initial_power_avg: f64,
    /// NLMS regularization.
    nlms_delta: f64,
}

impl GeneralizedSidelobeCanceller {
    /// Create a new GSC with a single look-direction constraint.
    pub fn new(config: GscConfig) -> Self {
        assert!(config.num_elements >= 2, "Need at least 2 elements");
        let n = config.num_elements;
        let sv = steering_vector(n, config.look_direction_rad, config.element_spacing_wavelengths);

        // Quiescent weights: wq = steering_vector / N (conventional beamformer)
        let wq: Vec<(f64, f64)> = sv.iter().map(|&s| cscale(1.0 / n as f64, s)).collect();

        // Blocking matrix via Gram-Schmidt orthogonal complement
        let blocking_matrix = build_blocking_matrix(&sv);
        let num_aux = blocking_matrix.len();

        let wa = vec![(0.0, 0.0); num_aux];

        Self {
            num_elements: n,
            step_size: config.step_size,
            algorithm: config.algorithm,
            wq,
            blocking_matrix,
            num_aux,
            wa,
            iterations: 0,
            output_power_avg: 0.0,
            initial_power_avg: 0.0,
            nlms_delta: 1e-8,
        }
    }

    /// Create a GSC with multiple constraints (e.g., nulls or unity responses
    /// in specified directions).
    pub fn with_constraints(
        num_elements: usize,
        element_spacing_wavelengths: f64,
        constraints: &[Constraint],
        step_size: f64,
        algorithm: AdaptationAlgorithm,
    ) -> Self {
        assert!(num_elements >= 2, "Need at least 2 elements");
        assert!(!constraints.is_empty(), "Need at least one constraint");
        assert!(
            constraints.len() < num_elements,
            "Number of constraints must be less than number of elements"
        );

        let n = num_elements;

        // Build constraint matrix C: each column is a steering vector for a constraint direction.
        let c_columns: Vec<Vec<(f64, f64)>> = constraints
            .iter()
            .map(|c| steering_vector(n, c.direction_rad, element_spacing_wavelengths))
            .collect();

        // Desired response vector f.
        let f: Vec<(f64, f64)> = constraints.iter().map(|c| c.response).collect();

        // Quiescent weights: wq = C (C^H C)^{-1} f
        let wq = compute_lcmv_weights(&c_columns, &f, n);

        // Blocking matrix: orthogonal complement of all constraint columns
        let blocking_matrix = build_blocking_matrix_multi(&c_columns, n);
        let num_aux = blocking_matrix.len();

        let wa = vec![(0.0, 0.0); num_aux];

        Self {
            num_elements: n,
            step_size,
            algorithm,
            wq,
            blocking_matrix,
            num_aux,
            wa,
            iterations: 0,
            output_power_avg: 0.0,
            initial_power_avg: 0.0,
            nlms_delta: 1e-8,
        }
    }

    /// Process a single snapshot (array of complex samples, one per element).
    /// Returns the scalar beamformer output.
    pub fn process(&mut self, snapshot: &[(f64, f64)]) -> (f64, f64) {
        assert_eq!(
            snapshot.len(),
            self.num_elements,
            "Snapshot length must equal number of elements"
        );

        // Quiescent output: yq = wq^H * x
        let yq = inner_product(&self.wq, snapshot);

        // Blocked signal: z = B^H * x  (length = num_aux)
        let z = self.apply_blocking_matrix(snapshot);

        // Adaptive path output: ya = wa^H * z
        let ya = inner_product(&self.wa, &z);

        // GSC output: y = yq - ya
        let y = csub(yq, ya);

        // Update adaptive weights using output as error signal
        self.update_weights(&z, y);

        // Track convergence
        self.iterations += 1;
        let alpha = if self.iterations < 10 {
            1.0 / self.iterations as f64
        } else {
            0.05
        };
        let out_pwr = cmag_sq(y);
        let init_pwr = cmag_sq(yq);
        self.output_power_avg = (1.0 - alpha) * self.output_power_avg + alpha * out_pwr;
        self.initial_power_avg = (1.0 - alpha) * self.initial_power_avg + alpha * init_pwr;

        y
    }

    /// Process a batch of snapshots, returning a vector of outputs.
    pub fn process_batch(&mut self, snapshots: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
        snapshots.iter().map(|s| self.process(s)).collect()
    }

    /// Get the current quiescent weight vector.
    pub fn quiescent_weights(&self) -> &[(f64, f64)] {
        &self.wq
    }

    /// Get the current adaptive weight vector.
    pub fn adaptive_weights(&self) -> &[(f64, f64)] {
        &self.wa
    }

    /// Get the effective combined weight vector: w = wq - B * wa.
    pub fn effective_weights(&self) -> Vec<(f64, f64)> {
        let mut w = self.wq.clone();
        for (col_idx, col) in self.blocking_matrix.iter().enumerate() {
            for (row, &b_val) in col.iter().enumerate() {
                let prod = cmul(b_val, self.wa[col_idx]);
                w[row] = csub(w[row], prod);
            }
        }
        w
    }

    /// Get convergence statistics.
    pub fn convergence_stats(&self) -> ConvergenceStats {
        let weight_norm: f64 = self.wa.iter().map(|w| cmag_sq(*w)).sum::<f64>().sqrt();
        let inr_improvement_db = if self.initial_power_avg > 1e-30 && self.output_power_avg > 1e-30
        {
            10.0 * (self.initial_power_avg / self.output_power_avg).log10()
        } else {
            0.0
        };
        ConvergenceStats {
            iterations: self.iterations,
            weight_norm,
            output_power: self.output_power_avg,
            inr_improvement_db,
        }
    }

    /// Compute the beam pattern (complex response) at a given angle in radians.
    pub fn beam_response(&self, angle_rad: f64, element_spacing_wavelengths: f64) -> (f64, f64) {
        let sv = steering_vector(self.num_elements, angle_rad, element_spacing_wavelengths);
        let ew = self.effective_weights();
        inner_product(&ew, &sv)
    }

    /// Reset adaptive weights to zero.
    pub fn reset(&mut self) {
        self.wa = vec![(0.0, 0.0); self.num_aux];
        self.iterations = 0;
        self.output_power_avg = 0.0;
        self.initial_power_avg = 0.0;
    }

    /// Set the NLMS regularization parameter (default: 1e-8).
    pub fn set_nlms_delta(&mut self, delta: f64) {
        self.nlms_delta = delta;
    }

    /// Get the number of auxiliary (adaptive) channels.
    pub fn num_auxiliary_channels(&self) -> usize {
        self.num_aux
    }

    /// Get the blocking matrix columns.
    pub fn blocking_matrix(&self) -> &[Vec<(f64, f64)>] {
        &self.blocking_matrix
    }

    // ── Private helpers ──────────────────────────────────────────────────

    /// Apply the blocking matrix: z = B^H * x.
    fn apply_blocking_matrix(&self, x: &[(f64, f64)]) -> Vec<(f64, f64)> {
        self.blocking_matrix
            .iter()
            .map(|col| inner_product(col, x))
            .collect()
    }

    /// Update adaptive weights using the chosen algorithm.
    fn update_weights(&mut self, z: &[(f64, f64)], error: (f64, f64)) {
        match self.algorithm {
            AdaptationAlgorithm::Lms => {
                // wa(n+1) = wa(n) + mu * z * conj(e)
                let e_conj = conj(error);
                for (i, &zi) in z.iter().enumerate() {
                    let update = cmul(zi, e_conj);
                    self.wa[i] = cadd(self.wa[i], cscale(self.step_size, update));
                }
            }
            AdaptationAlgorithm::Nlms => {
                // wa(n+1) = wa(n) + mu / (z^H z + delta) * z * conj(e)
                let z_power: f64 = z.iter().map(|&zi| cmag_sq(zi)).sum();
                let norm_step = self.step_size / (z_power + self.nlms_delta);
                let e_conj = conj(error);
                for (i, &zi) in z.iter().enumerate() {
                    let update = cmul(zi, e_conj);
                    self.wa[i] = cadd(self.wa[i], cscale(norm_step, update));
                }
            }
        }
    }
}

// ── Free functions ──────────────────────────────────────────────────────────

/// Compute the steering vector for a Uniform Linear Array (ULA).
///
/// `num_elements`: number of array elements.
/// `angle_rad`: arrival angle in radians (0 = broadside).
/// `d_wavelengths`: element spacing in wavelengths (typically 0.5).
pub fn steering_vector(
    num_elements: usize,
    angle_rad: f64,
    d_wavelengths: f64,
) -> Vec<(f64, f64)> {
    let phase_inc = 2.0 * PI * d_wavelengths * angle_rad.sin();
    (0..num_elements)
        .map(|k| {
            let phase = k as f64 * phase_inc;
            (phase.cos(), phase.sin())
        })
        .collect()
}

/// Build a blocking matrix as the orthogonal complement of a single steering vector.
///
/// Uses Gram-Schmidt orthogonalization starting from the standard basis vectors,
/// projecting out the steering vector component.
fn build_blocking_matrix(sv: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
    let n = sv.len();
    // Normalize the steering vector
    let sv_norm: f64 = sv.iter().map(|s| cmag_sq(*s)).sum::<f64>().sqrt();
    let sv_n: Vec<(f64, f64)> = sv.iter().map(|&s| cscale(1.0 / sv_norm, s)).collect();

    let mut columns: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n - 1);

    // Start from identity columns and orthogonalize against sv and each other
    for i in 0..n {
        // Standard basis vector e_i
        let mut col: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
        col[i] = (1.0, 0.0);

        // Project out steering vector component
        let proj_sv = inner_product(&sv_n, &col);
        for (j, &svj) in sv_n.iter().enumerate() {
            col[j] = csub(col[j], cmul(proj_sv, svj));
        }

        // Project out previously found orthogonal columns
        for prev in &columns {
            let proj = inner_product(prev, &col);
            for (j, &pj) in prev.iter().enumerate() {
                col[j] = csub(col[j], cmul(proj, pj));
            }
        }

        // Check if this column has significant norm (not redundant)
        let col_norm: f64 = col.iter().map(|c| cmag_sq(*c)).sum::<f64>().sqrt();
        if col_norm > 1e-10 {
            // Normalize
            let col_normalized: Vec<(f64, f64)> =
                col.iter().map(|&c| cscale(1.0 / col_norm, c)).collect();
            columns.push(col_normalized);
        }

        if columns.len() == n - 1 {
            break;
        }
    }

    columns
}

/// Build blocking matrix for multiple constraint vectors.
fn build_blocking_matrix_multi(
    constraint_columns: &[Vec<(f64, f64)>],
    n: usize,
) -> Vec<Vec<(f64, f64)>> {
    let num_constraints = constraint_columns.len();
    let num_aux = n - num_constraints;

    // Orthonormalize the constraint columns first
    let mut ortho_constraints: Vec<Vec<(f64, f64)>> = Vec::with_capacity(num_constraints);
    for c_col in constraint_columns {
        let mut col = c_col.clone();
        for prev in &ortho_constraints {
            let proj = inner_product(prev, &col);
            for (j, &pj) in prev.iter().enumerate() {
                col[j] = csub(col[j], cmul(proj, pj));
            }
        }
        let norm: f64 = col.iter().map(|c| cmag_sq(*c)).sum::<f64>().sqrt();
        if norm > 1e-10 {
            let normalized: Vec<(f64, f64)> =
                col.iter().map(|&c| cscale(1.0 / norm, c)).collect();
            ortho_constraints.push(normalized);
        }
    }

    // Now find orthogonal complement
    let mut columns: Vec<Vec<(f64, f64)>> = Vec::with_capacity(num_aux);

    for i in 0..n {
        let mut col: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
        col[i] = (1.0, 0.0);

        // Project out all constraint vectors
        for oc in &ortho_constraints {
            let proj = inner_product(oc, &col);
            for (j, &ocj) in oc.iter().enumerate() {
                col[j] = csub(col[j], cmul(proj, ocj));
            }
        }

        // Project out previously found blocking columns
        for prev in &columns {
            let proj = inner_product(prev, &col);
            for (j, &pj) in prev.iter().enumerate() {
                col[j] = csub(col[j], cmul(proj, pj));
            }
        }

        let col_norm: f64 = col.iter().map(|c| cmag_sq(*c)).sum::<f64>().sqrt();
        if col_norm > 1e-10 {
            let col_normalized: Vec<(f64, f64)> =
                col.iter().map(|&c| cscale(1.0 / col_norm, c)).collect();
            columns.push(col_normalized);
        }

        if columns.len() == num_aux {
            break;
        }
    }

    columns
}

/// Compute LCMV quiescent weights: wq = C (C^H C)^{-1} f.
///
/// `c_columns`: each element is a steering vector (column of C).
/// `f`: desired response vector.
/// `n`: number of elements.
fn compute_lcmv_weights(
    c_columns: &[Vec<(f64, f64)>],
    f: &[(f64, f64)],
    n: usize,
) -> Vec<(f64, f64)> {
    let m = c_columns.len(); // number of constraints

    // Compute Gram matrix G = C^H C (m x m)
    let mut gram: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); m]; m];
    for i in 0..m {
        for j in 0..m {
            gram[i][j] = inner_product(&c_columns[i], &c_columns[j]);
        }
    }

    // Invert the Gram matrix using Gauss-Jordan elimination
    let gram_inv = invert_matrix(&gram);

    // Compute (C^H C)^{-1} f
    let mut g_inv_f: Vec<(f64, f64)> = vec![(0.0, 0.0); m];
    for i in 0..m {
        for j in 0..m {
            g_inv_f[i] = cadd(g_inv_f[i], cmul(gram_inv[i][j], f[j]));
        }
    }

    // wq = C * (C^H C)^{-1} f
    let mut wq = vec![(0.0, 0.0); n];
    for (col_idx, col) in c_columns.iter().enumerate() {
        for (row, &c_val) in col.iter().enumerate() {
            wq[row] = cadd(wq[row], cmul(c_val, g_inv_f[col_idx]));
        }
    }

    wq
}

/// Invert a small complex matrix using Gauss-Jordan elimination.
fn invert_matrix(a: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    let n = a.len();
    // Augmented matrix [A | I]
    let mut aug: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = (1.0, 0.0);
    }

    for col in 0..n {
        // Partial pivoting
        let mut max_mag = cmag_sq(aug[col][col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let mag = cmag_sq(aug[row][col]);
            if mag > max_mag {
                max_mag = mag;
                max_row = row;
            }
        }
        if max_row != col {
            aug.swap(col, max_row);
        }

        let pivot = aug[col][col];
        assert!(
            cmag_sq(pivot) > 1e-30,
            "Singular matrix in LCMV weight computation"
        );
        let pivot_inv = cscale(1.0 / cmag_sq(pivot), conj(pivot));

        // Scale pivot row
        for j in 0..2 * n {
            aug[col][j] = cmul(pivot_inv, aug[col][j]);
        }

        // Eliminate other rows
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..2 * n {
                let sub = cmul(factor, aug[col][j]);
                aug[row][j] = csub(aug[row][j], sub);
            }
        }
    }

    // Extract inverse
    let mut inv = vec![vec![(0.0, 0.0); n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = aug[i][n + j];
        }
    }
    inv
}

/// Compute the interference-to-noise ratio improvement for a given set of
/// effective weights and interference directions.
///
/// Returns improvement in dB, comparing the INR of the adapted weights
/// versus the quiescent weights.
pub fn inr_improvement_db(
    quiescent_weights: &[(f64, f64)],
    effective_weights: &[(f64, f64)],
    interference_directions_rad: &[f64],
    element_spacing_wavelengths: f64,
) -> f64 {
    let n = quiescent_weights.len();

    let mut quiescent_inr = 0.0;
    let mut adapted_inr = 0.0;

    for &dir in interference_directions_rad {
        let sv = steering_vector(n, dir, element_spacing_wavelengths);
        let resp_q = inner_product(quiescent_weights, &sv);
        let resp_a = inner_product(effective_weights, &sv);
        quiescent_inr += cmag_sq(resp_q);
        adapted_inr += cmag_sq(resp_a);
    }

    if adapted_inr < 1e-30 {
        return 100.0; // effectively infinite improvement, cap at 100 dB
    }

    10.0 * (quiescent_inr / adapted_inr).log10()
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn complex_approx_eq(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    #[test]
    fn test_steering_vector_broadside() {
        // At broadside (0 rad), all elements have the same phase
        let sv = steering_vector(4, 0.0, 0.5);
        for s in &sv {
            assert!(complex_approx_eq(*s, (1.0, 0.0), 1e-12));
        }
    }

    #[test]
    fn test_steering_vector_endfire() {
        // At 90 degrees (endfire), phase increments by pi for d=0.5
        let sv = steering_vector(4, PI / 2.0, 0.5);
        assert!(complex_approx_eq(sv[0], (1.0, 0.0), 1e-12));
        assert!(complex_approx_eq(sv[1], (-1.0, 0.0), 1e-12));
        assert!(complex_approx_eq(sv[2], (1.0, 0.0), 1e-12));
        assert!(complex_approx_eq(sv[3], (-1.0, 0.0), 1e-12));
    }

    #[test]
    fn test_steering_vector_unit_norm() {
        let sv = steering_vector(8, 0.3, 0.5);
        let norm_sq: f64 = sv.iter().map(|s| cmag_sq(*s)).sum();
        // Each element has unit magnitude, so norm^2 = N
        assert!(approx_eq(norm_sq, 8.0, 1e-12));
    }

    #[test]
    fn test_blocking_matrix_orthogonality() {
        // B^H * sv should be zero (blocking matrix blocks the steering vector)
        let sv = steering_vector(4, 0.2, 0.5);
        let bm = build_blocking_matrix(&sv);

        assert_eq!(bm.len(), 3); // N-1 columns

        for col in &bm {
            let proj = inner_product(col, &sv);
            assert!(
                cmag_sq(proj) < 1e-20,
                "Blocking matrix column not orthogonal to steering vector: {:?}",
                proj
            );
        }
    }

    #[test]
    fn test_blocking_matrix_columns_orthonormal() {
        let sv = steering_vector(6, 0.4, 0.5);
        let bm = build_blocking_matrix(&sv);

        for i in 0..bm.len() {
            // Self inner product should be 1
            let self_ip = inner_product(&bm[i], &bm[i]);
            assert!(
                approx_eq(self_ip.0, 1.0, 1e-10),
                "Column {} not unit norm: {}",
                i,
                self_ip.0
            );
            assert!(approx_eq(self_ip.1, 0.0, 1e-10));

            // Cross inner products should be 0
            for j in (i + 1)..bm.len() {
                let cross_ip = inner_product(&bm[i], &bm[j]);
                assert!(
                    cmag_sq(cross_ip) < 1e-20,
                    "Columns {} and {} not orthogonal",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_quiescent_weights_are_steering_over_n() {
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.3,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let gsc = GeneralizedSidelobeCanceller::new(config);
        let sv = steering_vector(4, 0.3, 0.5);

        for (wq, sv_val) in gsc.quiescent_weights().iter().zip(sv.iter()) {
            let expected = cscale(0.25, *sv_val);
            assert!(complex_approx_eq(*wq, expected, 1e-12));
        }
    }

    #[test]
    fn test_gsc_passes_desired_signal() {
        // Signal from the look direction should pass through mostly unchanged
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.001,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);

        let sv_desired = steering_vector(4, 0.0, 0.5);
        // Process many snapshots of pure desired signal
        let mut last_output = (0.0, 0.0);
        for _ in 0..100 {
            last_output = gsc.process(&sv_desired);
        }

        // The output magnitude should be close to 1.0 (desired signal preserved)
        let mag = cmag_sq(last_output).sqrt();
        assert!(
            mag > 0.8,
            "Desired signal too attenuated: mag = {}",
            mag
        );
    }

    #[test]
    fn test_gsc_nulls_interference() {
        // Desired signal from 0 rad, interference from 0.5 rad
        let config = GscConfig {
            num_elements: 8,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.02,
            algorithm: AdaptationAlgorithm::Nlms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);

        let sv_desired = steering_vector(8, 0.0, 0.5);
        let sv_interf = steering_vector(8, 0.5, 0.5);

        // Mixed signal: desired (amplitude 1) + interference (amplitude 5)
        let make_snapshot = |desired_amp: f64, interf_amp: f64| -> Vec<(f64, f64)> {
            (0..8)
                .map(|i| {
                    cadd(
                        cscale(desired_amp, sv_desired[i]),
                        cscale(interf_amp, sv_interf[i]),
                    )
                })
                .collect()
        };

        // Process many snapshots for convergence
        for _ in 0..500 {
            let snapshot = make_snapshot(1.0, 5.0);
            gsc.process(&snapshot);
        }

        // Check beam pattern: look direction should have high response,
        // interference direction should be nulled
        let resp_desired = gsc.beam_response(0.0, 0.5);
        let resp_interf = gsc.beam_response(0.5, 0.5);

        let mag_desired = cmag_sq(resp_desired).sqrt();
        let mag_interf = cmag_sq(resp_interf).sqrt();

        assert!(
            mag_desired > 0.5,
            "Desired direction response too low: {}",
            mag_desired
        );
        assert!(
            mag_interf < mag_desired * 0.3,
            "Interference not sufficiently nulled: interf={}, desired={}",
            mag_interf,
            mag_desired
        );
    }

    #[test]
    fn test_gsc_lms_vs_nlms() {
        // Both should converge, NLMS typically faster
        for algo in &[AdaptationAlgorithm::Lms, AdaptationAlgorithm::Nlms] {
            let config = GscConfig {
                num_elements: 4,
                look_direction_rad: 0.0,
                element_spacing_wavelengths: 0.5,
                step_size: if *algo == AdaptationAlgorithm::Lms {
                    0.005
                } else {
                    0.5
                },
                algorithm: *algo,
            };
            let mut gsc = GeneralizedSidelobeCanceller::new(config);

            let sv_desired = steering_vector(4, 0.0, 0.5);
            let sv_interf = steering_vector(4, 0.8, 0.5);

            for _ in 0..300 {
                let snapshot: Vec<(f64, f64)> = (0..4)
                    .map(|i| cadd(sv_desired[i], cscale(3.0, sv_interf[i])))
                    .collect();
                gsc.process(&snapshot);
            }

            let stats = gsc.convergence_stats();
            assert_eq!(stats.iterations, 300);
            assert!(stats.weight_norm > 0.0, "Weights should have adapted");
        }
    }

    #[test]
    fn test_effective_weights_constraint_preserved() {
        // The effective weights should still satisfy C^H w = f
        // For single constraint: sv^H w = 1/N * sv^H sv = 1
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.3,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);

        let sv = steering_vector(4, 0.3, 0.5);
        // Process some data to change adaptive weights
        let sv_interf = steering_vector(4, 1.0, 0.5);
        for _ in 0..100 {
            let snapshot: Vec<(f64, f64)> = (0..4)
                .map(|i| cadd(sv[i], cscale(2.0, sv_interf[i])))
                .collect();
            gsc.process(&snapshot);
        }

        let ew = gsc.effective_weights();
        let response = inner_product(&ew, &sv);
        // Should be close to 1.0 (distortionless constraint)
        assert!(
            approx_eq(response.0, 1.0, 0.05),
            "Constraint not preserved: response = {:?}",
            response
        );
        assert!(
            approx_eq(response.1, 0.0, 0.05),
            "Constraint imaginary part not zero: {:?}",
            response
        );
    }

    #[test]
    fn test_multi_constraint_gsc() {
        // Two constraints: unity gain at 0 rad, zero gain at 0.5 rad
        let constraints = vec![
            Constraint {
                direction_rad: 0.0,
                response: (1.0, 0.0),
            },
            Constraint {
                direction_rad: 0.5,
                response: (0.0, 0.0),
            },
        ];

        let gsc = GeneralizedSidelobeCanceller::with_constraints(
            6,
            0.5,
            &constraints,
            0.01,
            AdaptationAlgorithm::Lms,
        );

        // Check quiescent weights satisfy both constraints
        let sv0 = steering_vector(6, 0.0, 0.5);
        let sv1 = steering_vector(6, 0.5, 0.5);

        let resp0 = inner_product(gsc.quiescent_weights(), &sv0);
        let resp1 = inner_product(gsc.quiescent_weights(), &sv1);

        assert!(
            approx_eq(resp0.0, 1.0, 0.01),
            "Unity constraint not met: {:?}",
            resp0
        );
        assert!(
            cmag_sq(resp1) < 0.01,
            "Null constraint not met: {:?}",
            resp1
        );
    }

    #[test]
    fn test_multi_constraint_blocking_matrix_dimensions() {
        let constraints = vec![
            Constraint {
                direction_rad: 0.0,
                response: (1.0, 0.0),
            },
            Constraint {
                direction_rad: 0.3,
                response: (1.0, 0.0),
            },
        ];

        let gsc = GeneralizedSidelobeCanceller::with_constraints(
            6,
            0.5,
            &constraints,
            0.01,
            AdaptationAlgorithm::Lms,
        );

        // N=6, 2 constraints => 4 auxiliary channels
        assert_eq!(gsc.num_auxiliary_channels(), 4);
        assert_eq!(gsc.blocking_matrix().len(), 4);
    }

    #[test]
    fn test_convergence_stats_initial() {
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let gsc = GeneralizedSidelobeCanceller::new(config);
        let stats = gsc.convergence_stats();

        assert_eq!(stats.iterations, 0);
        assert_eq!(stats.weight_norm, 0.0);
        assert_eq!(stats.output_power, 0.0);
    }

    #[test]
    fn test_reset() {
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);

        // Process some data
        let snapshot = vec![(1.0, 0.0); 4];
        for _ in 0..50 {
            gsc.process(&snapshot);
        }
        assert!(gsc.convergence_stats().iterations > 0);

        gsc.reset();
        let stats = gsc.convergence_stats();
        assert_eq!(stats.iterations, 0);
        assert_eq!(stats.weight_norm, 0.0);
    }

    #[test]
    fn test_process_batch() {
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);

        let snapshots: Vec<Vec<(f64, f64)>> = (0..10).map(|_| vec![(1.0, 0.0); 4]).collect();
        let outputs = gsc.process_batch(&snapshots);
        assert_eq!(outputs.len(), 10);
        assert_eq!(gsc.convergence_stats().iterations, 10);
    }

    #[test]
    fn test_inr_improvement_metric() {
        let config = GscConfig {
            num_elements: 8,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.03,
            algorithm: AdaptationAlgorithm::Nlms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);

        let sv_desired = steering_vector(8, 0.0, 0.5);
        let sv_interf = steering_vector(8, 0.6, 0.5);

        // Use time-varying phases to make the signals distinguishable
        for k in 0..1000 {
            let desired_phase = k as f64 * 0.1;
            let interf_phase = k as f64 * 0.37;
            let desired_phasor = (desired_phase.cos(), desired_phase.sin());
            let interf_phasor = (interf_phase.cos(), interf_phase.sin());
            let snapshot: Vec<(f64, f64)> = (0..8)
                .map(|i| {
                    cadd(
                        cmul(desired_phasor, sv_desired[i]),
                        cscale(5.0, cmul(interf_phasor, sv_interf[i])),
                    )
                })
                .collect();
            gsc.process(&snapshot);
        }

        let improvement = inr_improvement_db(
            gsc.quiescent_weights(),
            &gsc.effective_weights(),
            &[0.6],
            0.5,
        );
        assert!(
            improvement > 3.0,
            "INR improvement should be significant: {} dB",
            improvement
        );
    }

    #[test]
    fn test_multiple_interferers() {
        // Test nulling of two interferers simultaneously
        let config = GscConfig {
            num_elements: 8,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.02,
            algorithm: AdaptationAlgorithm::Nlms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);

        let sv_desired = steering_vector(8, 0.0, 0.5);
        let sv_int1 = steering_vector(8, 0.5, 0.5);
        let sv_int2 = steering_vector(8, -0.7, 0.5);

        for _ in 0..800 {
            let snapshot: Vec<(f64, f64)> = (0..8)
                .map(|i| {
                    cadd(
                        sv_desired[i],
                        cadd(cscale(4.0, sv_int1[i]), cscale(3.0, sv_int2[i])),
                    )
                })
                .collect();
            gsc.process(&snapshot);
        }

        let resp_desired = gsc.beam_response(0.0, 0.5);
        let resp_int1 = gsc.beam_response(0.5, 0.5);
        let resp_int2 = gsc.beam_response(-0.7, 0.5);

        let mag_desired = cmag_sq(resp_desired).sqrt();
        let mag_int1 = cmag_sq(resp_int1).sqrt();
        let mag_int2 = cmag_sq(resp_int2).sqrt();

        assert!(
            mag_desired > 0.5,
            "Desired signal attenuated too much: {}",
            mag_desired
        );
        assert!(
            mag_int1 < mag_desired * 0.4,
            "Interferer 1 not sufficiently nulled: int1={}, desired={}",
            mag_int1,
            mag_desired
        );
        assert!(
            mag_int2 < mag_desired * 0.4,
            "Interferer 2 not sufficiently nulled: int2={}, desired={}",
            mag_int2,
            mag_desired
        );
    }

    #[test]
    fn test_beam_response_at_look_direction() {
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.3,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let gsc = GeneralizedSidelobeCanceller::new(config);

        // Before adaptation, response at look direction should be ~1.0
        let resp = gsc.beam_response(0.3, 0.5);
        assert!(
            approx_eq(resp.0, 1.0, 0.01),
            "Look direction response should be unity: {:?}",
            resp
        );
    }

    #[test]
    fn test_nlms_delta_setter() {
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.5,
            algorithm: AdaptationAlgorithm::Nlms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);
        gsc.set_nlms_delta(1e-4);

        // Should not panic and should still process
        let snapshot = vec![(1.0, 0.0); 4];
        let _output = gsc.process(&snapshot);
    }

    #[test]
    #[should_panic(expected = "Need at least 2 elements")]
    fn test_minimum_elements() {
        let config = GscConfig {
            num_elements: 1,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let _ = GeneralizedSidelobeCanceller::new(config);
    }

    #[test]
    #[should_panic(expected = "Snapshot length must equal number of elements")]
    fn test_snapshot_size_mismatch() {
        let config = GscConfig {
            num_elements: 4,
            look_direction_rad: 0.0,
            element_spacing_wavelengths: 0.5,
            step_size: 0.01,
            algorithm: AdaptationAlgorithm::Lms,
        };
        let mut gsc = GeneralizedSidelobeCanceller::new(config);
        let bad_snapshot = vec![(1.0, 0.0); 3];
        gsc.process(&bad_snapshot);
    }

    #[test]
    fn test_complex_arithmetic_helpers() {
        assert_eq!(cmul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0));
        assert_eq!(conj((3.0, -4.0)), (3.0, 4.0));
        assert_eq!(cadd((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
        assert_eq!(csub((5.0, 6.0), (3.0, 4.0)), (2.0, 2.0));
        assert!(approx_eq(cmag_sq((3.0, 4.0)), 25.0, 1e-12));
        assert_eq!(cscale(2.0, (3.0, 4.0)), (6.0, 8.0));
    }

    #[test]
    fn test_inner_product() {
        let a = vec![(1.0, 0.0), (0.0, 1.0)];
        let b = vec![(1.0, 0.0), (0.0, 1.0)];
        // conj(1,0)*(1,0) + conj(0,1)*(0,1) = (1,0) + (0,-1)*(0,1) = (1,0) + (1,0) = (2,0)
        let ip = inner_product(&a, &b);
        assert!(complex_approx_eq(ip, (2.0, 0.0), 1e-12));
    }
}

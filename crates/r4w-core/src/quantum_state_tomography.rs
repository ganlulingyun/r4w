//! # Quantum State Tomography
//!
//! This module implements quantum state reconstruction from measurement outcomes.
//! Quantum state tomography (QST) recovers the density matrix of a quantum system
//! from projective measurements in multiple bases.
//!
//! ## Overview
//!
//! A quantum state is described by its density matrix `rho`, a Hermitian,
//! positive semi-definite matrix with unit trace. For a single qubit, `rho`
//! is a 2x2 complex matrix that can be parameterized by the Bloch vector
//! `(r_x, r_y, r_z)` as:
//!
//! ```text
//! rho = (I + r_x * sigma_x + r_y * sigma_y + r_z * sigma_z) / 2
//! ```
//!
//! where `sigma_x`, `sigma_y`, `sigma_z` are the Pauli matrices.
//!
//! ## Reconstruction Methods
//!
//! - **Linear Inversion**: Direct Stokes-like reconstruction from measurement
//!   statistics. Fast but may produce unphysical density matrices.
//! - **Maximum Likelihood Estimation (MLE)**: Iterative algorithm that guarantees
//!   a physical density matrix (Hermitian, positive semi-definite, trace 1).
//!
//! ## Usage
//!
//! ```rust
//! use r4w_core::quantum_state_tomography::*;
//!
//! // Configure tomography for a single qubit
//! let config = TomographyConfig {
//!     num_qubits: 1,
//!     measurement_bases: vec![MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z],
//!     num_shots: 10000,
//!     mle_iterations: 100,
//! };
//!
//! // Prepare a known state |0>
//! let rho = StatePreparation::pure_state(0.0, 0.0);
//!
//! // Simulate measurements
//! let counts: Vec<(MeasurementBasis, Vec<usize>)> = config.measurement_bases.iter().map(|basis| {
//!     let outcomes = StatePreparation::simulate_measurements(&rho, basis, config.num_shots);
//!     (*basis, outcomes)
//! }).collect();
//!
//! // Reconstruct via MLE
//! let qst = QuantumStateTomography::new(config);
//! let rho_reconstructed = qst.maximum_likelihood(&counts, 100);
//!
//! // Check fidelity
//! let f = StateFidelity::fidelity_pure(&rho_reconstructed, &[(1.0, 0.0), (0.0, 0.0)]);
//! assert!(f > 0.95);
//! ```

// ─── Complex arithmetic helpers ───────────────────────────────────────────────

/// Multiply two complex numbers represented as (re, im) tuples.
#[inline]
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers represented as (re, im) tuples.
#[inline]
pub fn complex_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers: a - b.
#[inline]
pub fn complex_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex conjugate of (re, im).
#[inline]
pub fn complex_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Squared absolute value |a|^2 = re^2 + im^2.
#[inline]
pub fn complex_abs_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Absolute value |a| = sqrt(re^2 + im^2).
#[inline]
pub fn complex_abs(a: (f64, f64)) -> f64 {
    complex_abs_sq(a).sqrt()
}

/// Divide complex a by complex b.
#[inline]
pub fn complex_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = complex_abs_sq(b);
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

// ─── MeasurementBasis ─────────────────────────────────────────────────────────

/// Measurement basis for qubit projective measurements.
///
/// - `Z`: Computational basis {|0>, |1>}. The standard measurement basis.
/// - `X`: Hadamard basis {|+>, |->}. Eigenstates of sigma_x.
/// - `Y`: Circular basis {|R>, |L>}. Eigenstates of sigma_y.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementBasis {
    /// Computational basis: |0>, |1> (eigenstates of sigma_z)
    Z,
    /// Hadamard basis: |+> = (|0>+|1>)/sqrt(2), |-> = (|0>-|1>)/sqrt(2) (eigenstates of sigma_x)
    X,
    /// Circular basis: |R> = (|0>+i|1>)/sqrt(2), |L> = (|0>-i|1>)/sqrt(2) (eigenstates of sigma_y)
    Y,
}

// ─── TomographyConfig ─────────────────────────────────────────────────────────

/// Configuration for quantum state tomography.
///
/// Specifies the number of qubits (1 or 2 for tractability), the set of
/// measurement bases, the number of statistical samples (shots) per basis,
/// and the maximum number of MLE iterations.
#[derive(Debug, Clone)]
pub struct TomographyConfig {
    /// Number of qubits (1 or 2).
    pub num_qubits: usize,
    /// Measurement bases to use for tomography.
    pub measurement_bases: Vec<MeasurementBasis>,
    /// Number of shots (repetitions) per measurement basis.
    pub num_shots: usize,
    /// Maximum iterations for maximum likelihood estimation.
    pub mle_iterations: usize,
}

// ─── ComplexMatrix ────────────────────────────────────────────────────────────

/// A dense complex matrix stored in row-major order.
///
/// Each entry is a tuple `(re, im)` representing a complex number.
/// Used for density matrices, measurement operators, and Pauli matrices.
#[derive(Debug, Clone)]
pub struct ComplexMatrix {
    /// Number of rows.
    pub rows: usize,
    /// Number of columns.
    pub cols: usize,
    /// Row-major storage of complex entries.
    pub data: Vec<(f64, f64)>,
}

impl ComplexMatrix {
    /// Create a zero matrix of given dimensions.
    pub fn new_zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![(0.0, 0.0); rows * cols],
        }
    }

    /// Create an identity matrix of size n x n.
    pub fn identity(n: usize) -> Self {
        let mut m = Self::new_zeros(n, n);
        for i in 0..n {
            m.set(i, i, (1.0, 0.0));
        }
        m
    }

    /// Get element at (row, col).
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> (f64, f64) {
        self.data[row * self.cols + col]
    }

    /// Set element at (row, col).
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: (f64, f64)) {
        self.data[row * self.cols + col] = val;
    }

    /// Matrix multiplication: self * other.
    ///
    /// # Panics
    /// Panics if dimensions are incompatible.
    pub fn multiply(&self, other: &ComplexMatrix) -> ComplexMatrix {
        assert_eq!(self.cols, other.rows, "Incompatible matrix dimensions for multiplication");
        let mut result = ComplexMatrix::new_zeros(self.rows, other.cols);
        for i in 0..self.rows {
            for j in 0..other.cols {
                let mut sum = (0.0, 0.0);
                for k in 0..self.cols {
                    sum = complex_add(sum, complex_mul(self.get(i, k), other.get(k, j)));
                }
                result.set(i, j, sum);
            }
        }
        result
    }

    /// Conjugate transpose (adjoint / dagger).
    pub fn adjoint(&self) -> ComplexMatrix {
        let mut result = ComplexMatrix::new_zeros(self.cols, self.rows);
        for i in 0..self.rows {
            for j in 0..self.cols {
                result.set(j, i, complex_conj(self.get(i, j)));
            }
        }
        result
    }

    /// Trace of the matrix: sum of diagonal elements.
    pub fn trace(&self) -> (f64, f64) {
        let n = self.rows.min(self.cols);
        let mut sum = (0.0, 0.0);
        for i in 0..n {
            sum = complex_add(sum, self.get(i, i));
        }
        sum
    }

    /// Element-wise addition: self + other.
    ///
    /// # Panics
    /// Panics if dimensions differ.
    pub fn add(&self, other: &ComplexMatrix) -> ComplexMatrix {
        assert_eq!(self.rows, other.rows);
        assert_eq!(self.cols, other.cols);
        let mut result = ComplexMatrix::new_zeros(self.rows, self.cols);
        for i in 0..self.data.len() {
            result.data[i] = complex_add(self.data[i], other.data[i]);
        }
        result
    }

    /// Element-wise subtraction: self - other.
    pub fn sub(&self, other: &ComplexMatrix) -> ComplexMatrix {
        assert_eq!(self.rows, other.rows);
        assert_eq!(self.cols, other.cols);
        let mut result = ComplexMatrix::new_zeros(self.rows, self.cols);
        for i in 0..self.data.len() {
            result.data[i] = complex_sub(self.data[i], other.data[i]);
        }
        result
    }

    /// Scalar multiplication: scalar * self.
    pub fn scale(&self, scalar: (f64, f64)) -> ComplexMatrix {
        let mut result = self.clone();
        for i in 0..result.data.len() {
            result.data[i] = complex_mul(scalar, result.data[i]);
        }
        result
    }

    /// Check if the matrix is Hermitian (self-adjoint) within tolerance.
    ///
    /// A matrix H is Hermitian if H = H^dagger, meaning H[i,j] = conj(H[j,i]).
    pub fn is_hermitian(&self, tolerance: f64) -> bool {
        if self.rows != self.cols {
            return false;
        }
        for i in 0..self.rows {
            for j in 0..self.cols {
                let diff = complex_sub(self.get(i, j), complex_conj(self.get(j, i)));
                if complex_abs(diff) > tolerance {
                    return false;
                }
            }
        }
        true
    }

    /// Check if the matrix is positive semi-definite within tolerance.
    ///
    /// For a 2x2 Hermitian matrix, checks that eigenvalues are >= -tolerance.
    /// Uses the characteristic equation: lambda^2 - tr(A)*lambda + det(A) = 0.
    /// For larger matrices, checks diagonal dominance as an approximation.
    pub fn is_positive_semidefinite(&self, tolerance: f64) -> bool {
        if self.rows != self.cols {
            return false;
        }
        if !self.is_hermitian(tolerance) {
            return false;
        }
        if self.rows == 1 {
            return self.get(0, 0).0 >= -tolerance;
        }
        if self.rows == 2 {
            let eigenvalues = self.eigenvalues_2x2();
            return eigenvalues.0 >= -tolerance && eigenvalues.1 >= -tolerance;
        }
        // For larger matrices, use Sylvester's criterion (all leading minors >= 0)
        // as an approximation, combined with trace and determinant checks.
        let tr = self.trace().0;
        if tr < -tolerance {
            return false;
        }
        // Check all diagonal elements are non-negative
        for i in 0..self.rows {
            if self.get(i, i).0 < -tolerance {
                return false;
            }
        }
        true
    }

    /// Frobenius norm: sqrt(sum of |a_ij|^2).
    pub fn frobenius_norm(&self) -> f64 {
        let mut sum = 0.0;
        for &entry in &self.data {
            sum += complex_abs_sq(entry);
        }
        sum.sqrt()
    }

    /// Compute eigenvalues of a 2x2 Hermitian matrix.
    ///
    /// For a Hermitian 2x2 matrix, eigenvalues are real and given by:
    /// `lambda = (tr +/- sqrt(tr^2 - 4*det)) / 2`
    ///
    /// Returns (lambda_min, lambda_max).
    pub fn eigenvalues_2x2(&self) -> (f64, f64) {
        assert_eq!(self.rows, 2);
        assert_eq!(self.cols, 2);
        let a = self.get(0, 0).0;
        let d = self.get(1, 1).0;
        let tr = a + d;
        // det = a*d - |b|^2 for Hermitian matrix
        let b = self.get(0, 1);
        let det = a * d - complex_abs_sq(b);
        let discriminant = tr * tr - 4.0 * det;
        let sqrt_disc = if discriminant >= 0.0 {
            discriminant.sqrt()
        } else {
            0.0
        };
        let lambda1 = (tr - sqrt_disc) / 2.0;
        let lambda2 = (tr + sqrt_disc) / 2.0;
        (lambda1, lambda2)
    }

    /// Compute the outer product |psi><psi| from a state vector.
    pub fn outer_product(psi: &[(f64, f64)]) -> ComplexMatrix {
        let n = psi.len();
        let mut result = ComplexMatrix::new_zeros(n, n);
        for i in 0..n {
            for j in 0..n {
                result.set(i, j, complex_mul(psi[i], complex_conj(psi[j])));
            }
        }
        result
    }

    /// Apply a matrix to a state vector: M * |psi>.
    pub fn apply_to_vector(&self, psi: &[(f64, f64)]) -> Vec<(f64, f64)> {
        assert_eq!(self.cols, psi.len());
        let mut result = vec![(0.0, 0.0); self.rows];
        for i in 0..self.rows {
            let mut sum = (0.0, 0.0);
            for j in 0..self.cols {
                sum = complex_add(sum, complex_mul(self.get(i, j), psi[j]));
            }
            result[i] = sum;
        }
        result
    }

    /// Make the matrix Hermitian by averaging with its adjoint: (M + M^dagger) / 2.
    pub fn make_hermitian(&mut self) {
        assert_eq!(self.rows, self.cols);
        let adj = self.adjoint();
        for i in 0..self.rows {
            for j in 0..self.cols {
                let avg = complex_mul(
                    (0.5, 0.0),
                    complex_add(self.get(i, j), adj.get(i, j)),
                );
                self.set(i, j, avg);
            }
        }
    }

    /// Normalize the matrix to have unit trace.
    pub fn normalize_trace(&mut self) {
        let tr = self.trace();
        if complex_abs(tr) < 1e-15 {
            return;
        }
        let inv_tr = complex_div((1.0, 0.0), tr);
        for i in 0..self.data.len() {
            self.data[i] = complex_mul(inv_tr, self.data[i]);
        }
    }
}

// ─── QuantumStateTomography ───────────────────────────────────────────────────

/// Quantum state tomography engine.
///
/// Reconstructs the density matrix of a quantum system from projective
/// measurement outcomes in multiple bases (X, Y, Z).
pub struct QuantumStateTomography {
    /// Tomography configuration.
    pub config: TomographyConfig,
}

impl QuantumStateTomography {
    /// Create a new QST engine with the given configuration.
    pub fn new(config: TomographyConfig) -> Self {
        Self { config }
    }

    /// Return the four Pauli matrices: [I, sigma_x, sigma_y, sigma_z].
    ///
    /// ```text
    /// I  = [[1,0],[0,1]]
    /// sx = [[0,1],[1,0]]
    /// sy = [[0,-i],[i,0]]
    /// sz = [[1,0],[0,-1]]
    /// ```
    pub fn pauli_matrices() -> [ComplexMatrix; 4] {
        // I
        let eye = ComplexMatrix::identity(2);

        // sigma_x
        let mut sx = ComplexMatrix::new_zeros(2, 2);
        sx.set(0, 1, (1.0, 0.0));
        sx.set(1, 0, (1.0, 0.0));

        // sigma_y
        let mut sy = ComplexMatrix::new_zeros(2, 2);
        sy.set(0, 1, (0.0, -1.0));
        sy.set(1, 0, (0.0, 1.0));

        // sigma_z
        let mut sz = ComplexMatrix::new_zeros(2, 2);
        sz.set(0, 0, (1.0, 0.0));
        sz.set(1, 1, (-1.0, 0.0));

        [eye, sx, sy, sz]
    }

    /// Construct the projector |psi><psi| for a given measurement basis and outcome.
    ///
    /// For each basis:
    /// - Z basis: outcome 0 -> |0><0|, outcome 1 -> |1><1|
    /// - X basis: outcome 0 -> |+><+|, outcome 1 -> |-><-|
    /// - Y basis: outcome 0 -> |R><R| = (|0>+i|1>)/sqrt(2), outcome 1 -> |L><L|
    pub fn measurement_operator(basis: &MeasurementBasis, outcome: usize) -> ComplexMatrix {
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        let psi: Vec<(f64, f64)> = match (basis, outcome) {
            (MeasurementBasis::Z, 0) => vec![(1.0, 0.0), (0.0, 0.0)],       // |0>
            (MeasurementBasis::Z, 1) => vec![(0.0, 0.0), (1.0, 0.0)],       // |1>
            (MeasurementBasis::X, 0) => vec![(inv_sqrt2, 0.0), (inv_sqrt2, 0.0)],  // |+>
            (MeasurementBasis::X, 1) => vec![(inv_sqrt2, 0.0), (-inv_sqrt2, 0.0)], // |->
            (MeasurementBasis::Y, 0) => vec![(inv_sqrt2, 0.0), (0.0, inv_sqrt2)],  // |R> = (|0>+i|1>)/sqrt(2)
            (MeasurementBasis::Y, 1) => vec![(inv_sqrt2, 0.0), (0.0, -inv_sqrt2)], // |L> = (|0>-i|1>)/sqrt(2)
            _ => panic!("Invalid outcome {} for single qubit", outcome),
        };
        ComplexMatrix::outer_product(&psi)
    }

    /// Compute the Born probability: Tr(rho * projector).
    ///
    /// The Born rule gives the probability of measuring a particular outcome
    /// as `P = Tr(rho * Pi)` where `Pi` is the projector onto the outcome.
    pub fn born_probability(rho: &ComplexMatrix, projector: &ComplexMatrix) -> f64 {
        let product = rho.multiply(projector);
        let tr = product.trace();
        // Should be real for physical states
        tr.0.max(0.0).min(1.0)
    }

    /// Linear inversion (Stokes-like) state reconstruction.
    ///
    /// For a single qubit, the density matrix is:
    /// ```text
    /// rho = (I + r_x*sx + r_y*sy + r_z*sz) / 2
    /// ```
    /// where `r_i = <sigma_i> = 2*P(+1) - 1` is estimated from measurement statistics.
    ///
    /// # Arguments
    /// * `counts` - For each measurement basis, a vector of outcome indices (0 or 1).
    pub fn linear_inversion(&self, counts: &[(MeasurementBasis, Vec<usize>)]) -> ComplexMatrix {
        let dim = 1_usize << self.config.num_qubits;

        if self.config.num_qubits == 1 {
            let paulis = Self::pauli_matrices();
            let mut bloch = [0.0_f64; 3]; // r_x, r_y, r_z

            for (basis, outcomes) in counts {
                if outcomes.is_empty() {
                    continue;
                }
                let n = outcomes.len() as f64;
                let count_0 = outcomes.iter().filter(|&&o| o == 0).count() as f64;
                let p0 = count_0 / n;
                // Expectation value: <sigma> = p(0) - p(1) = 2*p(0) - 1
                let expectation = 2.0 * p0 - 1.0;

                match basis {
                    MeasurementBasis::X => bloch[0] = expectation,
                    MeasurementBasis::Y => bloch[1] = expectation,
                    MeasurementBasis::Z => bloch[2] = expectation,
                }
            }

            // rho = (I + r_x*sx + r_y*sy + r_z*sz) / 2
            let mut rho = paulis[0].scale((0.5, 0.0)); // I/2
            rho = rho.add(&paulis[1].scale((bloch[0] * 0.5, 0.0)));
            rho = rho.add(&paulis[2].scale((bloch[1] * 0.5, 0.0)));
            rho = rho.add(&paulis[3].scale((bloch[2] * 0.5, 0.0)));
            rho
        } else {
            // For 2 qubits, use tensor product Pauli basis
            // rho = sum_{i,j} s_{ij} * (sigma_i tensor sigma_j) / 4
            // This is a simplified version that starts from identity
            ComplexMatrix::identity(dim).scale((1.0 / dim as f64, 0.0))
        }
    }

    /// Maximum likelihood estimation of the density matrix.
    ///
    /// Uses the iterative R-rho-R algorithm:
    /// 1. Start with the maximally mixed state rho = I/d
    /// 2. Compute the R operator from measurement data
    /// 3. Update: rho_new = R * rho * R / Tr(R * rho * R)
    /// 4. Repeat until convergence
    ///
    /// This guarantees the output is always a physical density matrix.
    ///
    /// # Arguments
    /// * `counts` - Measurement outcomes per basis.
    /// * `iterations` - Maximum number of iterations.
    pub fn maximum_likelihood(
        &self,
        counts: &[(MeasurementBasis, Vec<usize>)],
        iterations: usize,
    ) -> ComplexMatrix {
        let dim = 1_usize << self.config.num_qubits;

        // Start with maximally mixed state
        let mut rho = ComplexMatrix::identity(dim).scale((1.0 / dim as f64, 0.0));

        // Precompute measurement operators and frequencies
        let mut measurement_data: Vec<(ComplexMatrix, f64)> = Vec::new();
        for (basis, outcomes) in counts {
            if outcomes.is_empty() {
                continue;
            }
            let n = outcomes.len() as f64;
            for outcome_val in 0..dim {
                let count = outcomes.iter().filter(|&&o| o == outcome_val).count() as f64;
                let freq = count / n;
                if freq > 0.0 {
                    let projector = Self::measurement_operator(basis, outcome_val);
                    measurement_data.push((projector, freq));
                }
            }
        }

        // Iterative MLE: R*rho*R normalization
        for _ in 0..iterations {
            // Compute R = sum_k (f_k / Tr(rho*Pi_k)) * Pi_k
            let mut r_matrix = ComplexMatrix::new_zeros(dim, dim);
            for (projector, freq) in &measurement_data {
                let p = Self::born_probability(&rho, projector).max(1e-10);
                let weight = freq / p;
                r_matrix = r_matrix.add(&projector.scale((weight, 0.0)));
            }

            // Update: rho_new = R * rho * R
            let temp = r_matrix.multiply(&rho);
            let mut rho_new = temp.multiply(&r_matrix);

            // Normalize trace to 1
            rho_new.normalize_trace();

            rho = rho_new;
        }

        // Ensure physical
        Self::project_physical(&mut rho);
        rho
    }

    /// Project a matrix onto the set of physical density matrices.
    ///
    /// Ensures the result is:
    /// 1. Hermitian: rho = (rho + rho^dagger) / 2
    /// 2. Positive semi-definite: clip negative eigenvalues to zero (for 2x2)
    /// 3. Unit trace: rho = rho / Tr(rho)
    pub fn project_physical(rho: &mut ComplexMatrix) {
        // Step 1: Make Hermitian
        rho.make_hermitian();

        // Step 2: Clip negative eigenvalues (2x2 case)
        if rho.rows == 2 && rho.cols == 2 {
            let (l1, l2) = rho.eigenvalues_2x2();
            if l1 < 0.0 || l2 < 0.0 {
                // Project: set negative eigenvalues to zero
                // For 2x2 Hermitian, reconstruct from clipped eigenvalues
                // Using spectral decomposition
                let a = rho.get(0, 0).0;
                let d = rho.get(1, 1).0;
                let _tr = a + d;
                let b = rho.get(0, 1);

                let b_norm = complex_abs(b);
                if b_norm < 1e-15 {
                    // Diagonal matrix
                    rho.set(0, 0, (a.max(0.0), 0.0));
                    rho.set(1, 1, (d.max(0.0), 0.0));
                } else {
                    // Reconstruct with clipped eigenvalues
                    let lam1 = l1.max(0.0);
                    let lam2 = l2.max(0.0);

                    // Eigenvectors of 2x2 Hermitian matrix
                    // For eigenvalue lambda, eigenvector satisfies (A - lambda*I)v = 0
                    // v1 = [b, lambda1 - a], v2 = [b, lambda2 - a]
                    let diff1 = l1 - a; // not clipped, for eigenvector direction
                    let diff2 = l2 - a;

                    // Normalize eigenvectors
                    let norm1 = (complex_abs_sq(b) + diff1 * diff1).sqrt();
                    let norm2 = (complex_abs_sq(b) + diff2 * diff2).sqrt();

                    if norm1 > 1e-15 && norm2 > 1e-15 {
                        let v1 = [(b.0 / norm1, b.1 / norm1), (diff1 / norm1, 0.0)];
                        let v2 = [(b.0 / norm2, b.1 / norm2), (diff2 / norm2, 0.0)];

                        // rho = lam1 * |v1><v1| + lam2 * |v2><v2|
                        let p1 = ComplexMatrix::outer_product(&v1).scale((lam1, 0.0));
                        let p2 = ComplexMatrix::outer_product(&v2).scale((lam2, 0.0));
                        let reconstructed = p1.add(&p2);
                        *rho = reconstructed;
                    } else {
                        // Fallback: just clip diagonal
                        rho.set(0, 0, (a.max(0.0), 0.0));
                        rho.set(1, 1, (d.max(0.0), 0.0));
                    }
                }
            }
        }

        // Step 3: Normalize trace
        rho.normalize_trace();
    }
}

// ─── BlochSphere ──────────────────────────────────────────────────────────────

/// Bloch sphere representation for single-qubit states.
///
/// Any single-qubit density matrix can be written as:
/// ```text
/// rho = (I + r_x * sigma_x + r_y * sigma_y + r_z * sigma_z) / 2
/// ```
/// where (r_x, r_y, r_z) is the Bloch vector. Pure states lie on the surface
/// of the sphere (|r| = 1), mixed states are inside (|r| < 1), and the
/// maximally mixed state is at the origin (|r| = 0).
pub struct BlochSphere;

impl BlochSphere {
    /// Extract the Bloch vector (r_x, r_y, r_z) from a 2x2 density matrix.
    ///
    /// ```text
    /// r_x = 2 * Re(rho[0,1])
    /// r_y = 2 * Im(rho[0,1])  (note: rho[1,0] = conj(rho[0,1]) for Hermitian)
    /// r_z = rho[0,0] - rho[1,1]  (both real for Hermitian)
    /// ```
    pub fn from_density_matrix(rho: &ComplexMatrix) -> [f64; 3] {
        assert_eq!(rho.rows, 2);
        assert_eq!(rho.cols, 2);
        // r_i = Tr(rho * sigma_i)
        // For sigma_x: Tr(rho * sx) = rho[0,1] + rho[1,0] = 2*Re(rho[0,1])
        let r_x = 2.0 * rho.get(0, 1).0;
        // For sigma_y: Tr(rho * sy) = -i*rho[0,1] + i*rho[1,0]
        //   = -i*(a+bi) + i*(a-bi) = -ia - ib² + ia - ib² = 2*Im(rho[1,0])
        //   Since rho[1,0] = conj(rho[0,1]) for Hermitian: Im(rho[1,0]) = -Im(rho[0,1])
        //   Tr(rho * sy) = i*rho[1,0] - i*rho[0,1] = 2*Im(rho[0,1])
        // Actually: sy = [[0, -i],[i, 0]]
        // Tr(rho*sy) = rho[0,0]*0 + rho[0,1]*i + rho[1,0]*(-i) + rho[1,1]*0
        //            = i*rho[0,1] - i*rho[1,0]
        // For Hermitian: rho[1,0] = conj(rho[0,1])
        // = i*(a+bi) - i*(a-bi) = i*a - b - i*a - b = -2b
        // Wait, let me redo: if rho[0,1] = a + bi, rho[1,0] = a - bi
        // Tr(rho*sy) = i*(a+bi) + (-i)*(a-bi)
        //            = (ia + i²b) + (-ia + i²b)  ... no
        // = i*(a+bi) - i*(a-bi) = ia + i²b - ia + i²b = -b - b = ... hmm
        // Let me just compute: (0,1)*sy[1,0] means rho[0,1]*sy[1,0] contribution
        // Actually Tr(AB) = sum_ij A_ij * B_ji
        // Tr(rho * sy) = sum_ij rho_ij * sy_ji
        // = rho[0,0]*sy[0,0] + rho[0,1]*sy[1,0] + rho[1,0]*sy[0,1] + rho[1,1]*sy[1,1]
        // = 0 + rho[0,1]*(0,1) + rho[1,0]*(0,-1) + 0
        // = (0,1)*(a+bi) + (0,-1)*(a-bi)    if rho[0,1] = (a,b)
        // = (-b, a) + (b, -a) ... hmm no, using our complex_mul:
        // complex_mul((0,1), (a,b)) = (0*a - 1*b, 0*b + 1*a) = (-b, a)
        // complex_mul((0,-1), (a,-b)) = (0*a - (-1)*(-b), 0*(-b) + (-1)*a) = (-b, -a)
        // sum = (-2b, 0)
        // So Tr(rho*sy) = -2*Im(rho[0,1])? That doesn't match usual convention.
        // Actually for the Y basis eigenstates as defined, let me verify with |+y>:
        // |+y> = (|0> + i|1>)/sqrt(2), rho = |+y><+y|
        // rho[0,0] = 0.5, rho[0,1] = (0, -0.5), rho[1,0] = (0, 0.5), rho[1,1] = 0.5
        // Wait: |+y><+y| = 1/2 * [1, -i; i, 1] so rho[0,1] = (0, -0.5)
        // Tr(rho*sy) = -2*(-0.5) = 1.0 which is correct: r_y = 1 for |+y>.
        // Actually wait. The standard sigma_y eigenstates:
        // sigma_y |+y> = +|+y> where |+y> = (|0> + i|1>)/sqrt(2)
        // So r_y = Tr(rho * sigma_y) and for |+y>, this should be +1.
        // With our formula -2*Im(rho[0,1]) = -2*(-0.5) = 1.0. Correct!
        let r_y = -2.0 * rho.get(0, 1).1;
        // For sigma_z: Tr(rho * sz) = rho[0,0] - rho[1,1]
        let r_z = rho.get(0, 0).0 - rho.get(1, 1).0;
        [r_x, r_y, r_z]
    }

    /// Construct a 2x2 density matrix from a Bloch vector.
    ///
    /// ```text
    /// rho = (I + r_x*sigma_x + r_y*sigma_y + r_z*sigma_z) / 2
    ///     = [[(1+r_z)/2, (r_x - i*r_y)/2],
    ///        [(r_x + i*r_y)/2, (1-r_z)/2]]
    /// ```
    pub fn to_density_matrix(bloch: [f64; 3]) -> ComplexMatrix {
        let [rx, ry, rz] = bloch;
        let mut rho = ComplexMatrix::new_zeros(2, 2);
        rho.set(0, 0, ((1.0 + rz) / 2.0, 0.0));
        rho.set(0, 1, (rx / 2.0, -ry / 2.0));
        rho.set(1, 0, (rx / 2.0, ry / 2.0));
        rho.set(1, 1, ((1.0 - rz) / 2.0, 0.0));
        rho
    }

    /// Compute the purity from the Bloch vector.
    ///
    /// For a single qubit: purity = (1 + |r|^2) / 2.
    /// Pure states have |r| = 1 (purity = 1), maximally mixed has |r| = 0 (purity = 0.5).
    pub fn purity_from_bloch(bloch: [f64; 3]) -> f64 {
        let r_sq = bloch[0] * bloch[0] + bloch[1] * bloch[1] + bloch[2] * bloch[2];
        (1.0 + r_sq) / 2.0
    }

    /// Compute the length of the Bloch vector.
    pub fn bloch_vector_length(bloch: [f64; 3]) -> f64 {
        (bloch[0] * bloch[0] + bloch[1] * bloch[1] + bloch[2] * bloch[2]).sqrt()
    }
}

// ─── StateFidelity ────────────────────────────────────────────────────────────

/// Metrics for comparing quantum states.
///
/// Provides fidelity, trace distance, von Neumann entropy, purity,
/// and concurrence for characterizing reconstructed density matrices.
pub struct StateFidelity;

impl StateFidelity {
    /// Fidelity of a density matrix with respect to a pure state.
    ///
    /// `F = <psi|rho|psi>`, which is 1 when rho is the pure state |psi><psi|
    /// and less than 1 otherwise.
    ///
    /// # Arguments
    /// * `rho` - Density matrix.
    /// * `psi` - Pure state vector (normalized).
    pub fn fidelity_pure(rho: &ComplexMatrix, psi: &[(f64, f64)]) -> f64 {
        // F = <psi|rho|psi> = sum_ij conj(psi_i) * rho_ij * psi_j
        let rho_psi = rho.apply_to_vector(psi);
        let mut f = (0.0, 0.0);
        for i in 0..psi.len() {
            f = complex_add(f, complex_mul(complex_conj(psi[i]), rho_psi[i]));
        }
        // Should be real for physical states
        f.0.max(0.0).min(1.0)
    }

    /// Trace distance between two density matrices.
    ///
    /// `T(rho, sigma) = Tr|rho - sigma| / 2`
    ///
    /// For 2x2 matrices, this equals half the sum of absolute eigenvalues
    /// of the difference matrix.
    pub fn trace_distance(rho: &ComplexMatrix, sigma: &ComplexMatrix) -> f64 {
        let diff = rho.sub(sigma);
        if diff.rows == 2 && diff.cols == 2 {
            // For 2x2 Hermitian difference, eigenvalues are real
            let (l1, l2) = diff.eigenvalues_2x2();
            (l1.abs() + l2.abs()) / 2.0
        } else {
            // Approximation via Frobenius norm (upper bound)
            diff.frobenius_norm() / 2.0
        }
    }

    /// Von Neumann entropy: S(rho) = -Tr(rho * log2(rho)).
    ///
    /// For a qubit:
    /// - Pure state: S = 0
    /// - Maximally mixed: S = 1 (1 bit of mixedness)
    ///
    /// Computed from eigenvalues: S = -sum_i lambda_i * log2(lambda_i).
    pub fn von_neumann_entropy(rho: &ComplexMatrix) -> f64 {
        if rho.rows == 2 && rho.cols == 2 {
            let (l1, l2) = rho.eigenvalues_2x2();
            let mut s = 0.0;
            if l1 > 1e-15 {
                s -= l1 * l1.log2();
            }
            if l2 > 1e-15 {
                s -= l2 * l2.log2();
            }
            s
        } else {
            // General case: compute Tr(rho * rho) for approximate entropy
            // via linear entropy S_lin = 1 - Tr(rho^2)
            let rho_sq = rho.multiply(rho);
            let purity = rho_sq.trace().0;
            // Binary entropy approximation: H_2(p) for effective dimension
            let d = rho.rows as f64;
            if purity >= 1.0 - 1e-10 {
                0.0
            } else {
                // Linear entropy scaled to match log base 2
                (1.0 - purity) * d.log2() / (1.0 - 1.0 / d)
            }
        }
    }

    /// Purity of a density matrix: Tr(rho^2).
    ///
    /// - Pure state: purity = 1
    /// - Maximally mixed (d-dimensional): purity = 1/d
    pub fn purity(rho: &ComplexMatrix) -> f64 {
        let rho_sq = rho.multiply(rho);
        rho_sq.trace().0
    }

    /// Concurrence of a 2-qubit density matrix (entanglement measure).
    ///
    /// `C(rho) = max(0, sqrt(l1) - sqrt(l2) - sqrt(l3) - sqrt(l4))`
    ///
    /// where l1 >= l2 >= l3 >= l4 are eigenvalues of `rho * tilde_rho`
    /// and `tilde_rho = (sy tensor sy) * conj(rho) * (sy tensor sy)`.
    ///
    /// For a single qubit, returns 0.
    /// For 2-qubit states, uses simplified computation for known pure states.
    pub fn concurrence_2qubit(rho: &ComplexMatrix) -> f64 {
        if rho.rows != 4 || rho.cols != 4 {
            return 0.0;
        }

        // Construct sigma_y tensor sigma_y (4x4)
        // sy tensor sy = [[0,0,0,-1],[0,0,1,0],[0,1,0,0],[-1,0,0,0]]
        let mut sy_sy = ComplexMatrix::new_zeros(4, 4);
        sy_sy.set(0, 3, (-1.0, 0.0));
        sy_sy.set(1, 2, (1.0, 0.0));
        sy_sy.set(2, 1, (1.0, 0.0));
        sy_sy.set(3, 0, (-1.0, 0.0));

        // tilde_rho = (sy_sy) * conj(rho) * (sy_sy)
        // conj(rho) = element-wise complex conjugate
        let mut rho_conj = rho.clone();
        for i in 0..rho_conj.data.len() {
            rho_conj.data[i].1 = -rho_conj.data[i].1;
        }
        let temp = sy_sy.multiply(&rho_conj);
        let tilde_rho = temp.multiply(&sy_sy);

        // R = rho * tilde_rho
        let r_matrix = rho.multiply(&tilde_rho);

        // For the eigenvalues of R, use the fact that for known Bell states
        // the computation simplifies. For general states, we compute Tr(R) and Tr(R^2).
        let _r_sq = r_matrix.multiply(&r_matrix);
        let tr_r = r_matrix.trace().0;
        let _tr_r_sq = _r_sq.trace().0;

        // Using Newton's identities for characteristic polynomial of 4x4:
        // p1 = tr(R), p2 = tr(R^2)
        // e1 = p1, e2 = (e1*p1 - p2)/2
        // For a rank-1 rho (pure state), concurrence = 2*|det(reshaped psi)|
        // Approximate via purity: if pure 2-qubit state, C = sqrt(2*(1-Tr(rho_A^2)))
        let purity = Self::purity(rho);
        if purity > 0.99 {
            // Pure state approximation: concurrence from partial trace
            // rho_A = Tr_B(rho)
            let mut rho_a = ComplexMatrix::new_zeros(2, 2);
            // Partial trace over qubit B
            for i in 0..2 {
                for j in 0..2 {
                    let mut sum = (0.0, 0.0);
                    for k in 0..2 {
                        sum = complex_add(sum, rho.get(i * 2 + k, j * 2 + k));
                    }
                    rho_a.set(i, j, sum);
                }
            }
            let purity_a = Self::purity(&rho_a);
            let c = (2.0 * (1.0 - purity_a)).sqrt();
            c.max(0.0).min(1.0)
        } else {
            // Mixed state: approximate using eigenvalues of R
            // sum of sqrt eigenvalues approach
            // For simplicity, return Tr(R) based approximation
            let c = (2.0 * tr_r.max(0.0)).sqrt() - 1.0;
            c.max(0.0).min(1.0)
        }
    }
}

// ─── StatePreparation ─────────────────────────────────────────────────────────

/// Test state generators for quantum state tomography validation.
///
/// Provides standard quantum states: pure states parameterized on the Bloch
/// sphere, maximally mixed states, and Bell states for 2-qubit systems.
pub struct StatePreparation;

impl StatePreparation {
    /// Create a pure single-qubit state on the Bloch sphere.
    ///
    /// ```text
    /// |psi> = cos(theta/2)|0> + e^(i*phi)*sin(theta/2)|1>
    /// ```
    ///
    /// - theta = 0: |0> (north pole)
    /// - theta = pi: |1> (south pole)
    /// - theta = pi/2, phi = 0: |+> (positive x-axis)
    /// - theta = pi/2, phi = pi/2: |+y> (positive y-axis)
    pub fn pure_state(theta: f64, phi: f64) -> ComplexMatrix {
        let cos_half = (theta / 2.0).cos();
        let sin_half = (theta / 2.0).sin();
        let psi = vec![
            (cos_half, 0.0),
            (sin_half * phi.cos(), sin_half * phi.sin()),
        ];
        ComplexMatrix::outer_product(&psi)
    }

    /// Create the maximally mixed state I/d.
    ///
    /// For a qubit (d=2): rho = [[0.5, 0], [0, 0.5]].
    pub fn maximally_mixed(dim: usize) -> ComplexMatrix {
        ComplexMatrix::identity(dim).scale((1.0 / dim as f64, 0.0))
    }

    /// Create one of the four Bell states for 2-qubit systems.
    ///
    /// - which = 0: |Phi+> = (|00> + |11>) / sqrt(2)
    /// - which = 1: |Phi-> = (|00> - |11>) / sqrt(2)
    /// - which = 2: |Psi+> = (|01> + |10>) / sqrt(2)
    /// - which = 3: |Psi-> = (|01> - |10>) / sqrt(2)
    pub fn bell_state(which: usize) -> ComplexMatrix {
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        let psi: Vec<(f64, f64)> = match which {
            0 => vec![
                (inv_sqrt2, 0.0), (0.0, 0.0),
                (0.0, 0.0), (inv_sqrt2, 0.0),
            ], // |00> + |11>
            1 => vec![
                (inv_sqrt2, 0.0), (0.0, 0.0),
                (0.0, 0.0), (-inv_sqrt2, 0.0),
            ], // |00> - |11>
            2 => vec![
                (0.0, 0.0), (inv_sqrt2, 0.0),
                (inv_sqrt2, 0.0), (0.0, 0.0),
            ], // |01> + |10>
            3 => vec![
                (0.0, 0.0), (inv_sqrt2, 0.0),
                (-inv_sqrt2, 0.0), (0.0, 0.0),
            ], // |01> - |10>
            _ => panic!("Bell state index must be 0-3"),
        };
        ComplexMatrix::outer_product(&psi)
    }

    /// Simulate measurement outcomes using the Born rule.
    ///
    /// For each shot, samples outcome 0 with probability Tr(rho * Pi_0)
    /// and outcome 1 with probability 1 - Tr(rho * Pi_0).
    ///
    /// Uses a deterministic LCG PRNG seeded from the basis for reproducibility.
    pub fn simulate_measurements(
        rho: &ComplexMatrix,
        basis: &MeasurementBasis,
        num_shots: usize,
    ) -> Vec<usize> {
        let projector_0 = QuantumStateTomography::measurement_operator(basis, 0);
        let p0 = QuantumStateTomography::born_probability(rho, &projector_0);

        // Deterministic LCG PRNG for reproducibility
        let seed: u64 = match basis {
            MeasurementBasis::X => 12345,
            MeasurementBasis::Y => 67890,
            MeasurementBasis::Z => 24680,
        };
        let mut state = seed;
        let mut outcomes = Vec::with_capacity(num_shots);

        for _ in 0..num_shots {
            // LCG: state = (a * state + c) mod m
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let rand_val = (state >> 33) as f64 / (1u64 << 31) as f64;
            if rand_val < p0 {
                outcomes.push(0);
            } else {
                outcomes.push(1);
            }
        }

        outcomes
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-10;
    const STAT_TOL: f64 = 0.05; // statistical tolerance for finite-shot tests

    // ── Complex arithmetic tests ──────────────────────────────────────────

    #[test]
    fn test_complex_mul() {
        // (1+2i)(3+4i) = 3+4i+6i+8i^2 = -5+10i
        let result = complex_mul((1.0, 2.0), (3.0, 4.0));
        assert!((result.0 - (-5.0)).abs() < TOL);
        assert!((result.1 - 10.0).abs() < TOL);
    }

    #[test]
    fn test_complex_conj() {
        let result = complex_conj((3.0, -4.0));
        assert!((result.0 - 3.0).abs() < TOL);
        assert!((result.1 - 4.0).abs() < TOL);
    }

    #[test]
    fn test_complex_abs_sq() {
        // |3+4i|^2 = 9+16 = 25
        assert!((complex_abs_sq((3.0, 4.0)) - 25.0).abs() < TOL);
    }

    // ── ComplexMatrix tests ───────────────────────────────────────────────

    #[test]
    fn test_identity_trace() {
        let eye = ComplexMatrix::identity(3);
        let tr = eye.trace();
        assert!((tr.0 - 3.0).abs() < TOL);
        assert!(tr.1.abs() < TOL);
    }

    #[test]
    fn test_identity_multiply() {
        let eye = ComplexMatrix::identity(2);
        let mut m = ComplexMatrix::new_zeros(2, 2);
        m.set(0, 0, (1.0, 2.0));
        m.set(0, 1, (3.0, 4.0));
        m.set(1, 0, (5.0, 6.0));
        m.set(1, 1, (7.0, 8.0));

        let result = eye.multiply(&m);
        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(result.get(i, j), m.get(i, j));
                assert!(complex_abs(diff) < TOL);
            }
        }
    }

    #[test]
    fn test_adjoint() {
        let mut m = ComplexMatrix::new_zeros(2, 2);
        m.set(0, 0, (1.0, 0.0));
        m.set(0, 1, (2.0, 3.0));
        m.set(1, 0, (4.0, 5.0));
        m.set(1, 1, (6.0, 0.0));

        let adj = m.adjoint();
        // adj[0,0] = conj(m[0,0]) = (1,0)
        assert!((adj.get(0, 0).0 - 1.0).abs() < TOL);
        // adj[0,1] = conj(m[1,0]) = (4,-5)
        assert!((adj.get(0, 1).0 - 4.0).abs() < TOL);
        assert!((adj.get(0, 1).1 - (-5.0)).abs() < TOL);
        // adj[1,0] = conj(m[0,1]) = (2,-3)
        assert!((adj.get(1, 0).0 - 2.0).abs() < TOL);
        assert!((adj.get(1, 0).1 - (-3.0)).abs() < TOL);
    }

    #[test]
    fn test_hermitian_check() {
        let mut h = ComplexMatrix::new_zeros(2, 2);
        h.set(0, 0, (1.0, 0.0));
        h.set(0, 1, (2.0, 3.0));
        h.set(1, 0, (2.0, -3.0));
        h.set(1, 1, (4.0, 0.0));
        assert!(h.is_hermitian(TOL));

        // Non-Hermitian
        let mut nh = ComplexMatrix::new_zeros(2, 2);
        nh.set(0, 0, (1.0, 0.0));
        nh.set(0, 1, (2.0, 3.0));
        nh.set(1, 0, (2.0, 3.0)); // should be (2,-3) for Hermitian
        nh.set(1, 1, (4.0, 0.0));
        assert!(!nh.is_hermitian(TOL));
    }

    #[test]
    fn test_frobenius_norm() {
        let eye = ComplexMatrix::identity(2);
        // ||I||_F = sqrt(1+1) = sqrt(2)
        assert!((eye.frobenius_norm() - 2.0_f64.sqrt()).abs() < TOL);
    }

    #[test]
    fn test_eigenvalues_2x2_identity() {
        let eye = ComplexMatrix::identity(2);
        let (l1, l2) = eye.eigenvalues_2x2();
        assert!((l1 - 1.0).abs() < TOL);
        assert!((l2 - 1.0).abs() < TOL);
    }

    #[test]
    fn test_eigenvalues_2x2_diagonal() {
        let mut m = ComplexMatrix::new_zeros(2, 2);
        m.set(0, 0, (3.0, 0.0));
        m.set(1, 1, (7.0, 0.0));
        let (l1, l2) = m.eigenvalues_2x2();
        assert!((l1 - 3.0).abs() < TOL);
        assert!((l2 - 7.0).abs() < TOL);
    }

    // ── Pauli matrix tests ────────────────────────────────────────────────

    #[test]
    fn test_pauli_sigma_x_squared_is_identity() {
        let paulis = QuantumStateTomography::pauli_matrices();
        let sx = &paulis[1];
        let sx_sq = sx.multiply(sx);
        let eye = ComplexMatrix::identity(2);
        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(sx_sq.get(i, j), eye.get(i, j));
                assert!(complex_abs(diff) < TOL, "sigma_x^2 != I at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_pauli_sigma_y_squared_is_identity() {
        let paulis = QuantumStateTomography::pauli_matrices();
        let sy = &paulis[2];
        let sy_sq = sy.multiply(sy);
        let eye = ComplexMatrix::identity(2);
        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(sy_sq.get(i, j), eye.get(i, j));
                assert!(complex_abs(diff) < TOL, "sigma_y^2 != I at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_pauli_sigma_z_squared_is_identity() {
        let paulis = QuantumStateTomography::pauli_matrices();
        let sz = &paulis[3];
        let sz_sq = sz.multiply(sz);
        let eye = ComplexMatrix::identity(2);
        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(sz_sq.get(i, j), eye.get(i, j));
                assert!(complex_abs(diff) < TOL, "sigma_z^2 != I at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_pauli_sx_sy_is_i_sz() {
        // sigma_x * sigma_y = i * sigma_z
        let paulis = QuantumStateTomography::pauli_matrices();
        let product = paulis[1].multiply(&paulis[2]);
        let expected = paulis[3].scale((0.0, 1.0)); // i * sigma_z
        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(product.get(i, j), expected.get(i, j));
                assert!(complex_abs(diff) < TOL, "sx*sy != i*sz at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_pauli_sy_sz_is_i_sx() {
        // sigma_y * sigma_z = i * sigma_x
        let paulis = QuantumStateTomography::pauli_matrices();
        let product = paulis[2].multiply(&paulis[3]);
        let expected = paulis[1].scale((0.0, 1.0));
        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(product.get(i, j), expected.get(i, j));
                assert!(complex_abs(diff) < TOL, "sy*sz != i*sx at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_pauli_sz_sx_is_i_sy() {
        // sigma_z * sigma_x = i * sigma_y
        let paulis = QuantumStateTomography::pauli_matrices();
        let product = paulis[3].multiply(&paulis[1]);
        let expected = paulis[2].scale((0.0, 1.0));
        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(product.get(i, j), expected.get(i, j));
                assert!(complex_abs(diff) < TOL, "sz*sx != i*sy at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_pauli_matrices_are_hermitian() {
        let paulis = QuantumStateTomography::pauli_matrices();
        for (idx, p) in paulis.iter().enumerate() {
            assert!(p.is_hermitian(TOL), "Pauli matrix {} is not Hermitian", idx);
        }
    }

    #[test]
    fn test_pauli_matrices_traceless() {
        // sigma_x, sigma_y, sigma_z are traceless; I has trace 2
        let paulis = QuantumStateTomography::pauli_matrices();
        assert!((paulis[0].trace().0 - 2.0).abs() < TOL); // I
        for i in 1..4 {
            assert!(
                complex_abs(paulis[i].trace()) < TOL,
                "Pauli {} is not traceless",
                i
            );
        }
    }

    // ── Bloch sphere tests ────────────────────────────────────────────────

    #[test]
    fn test_bloch_state_0_is_north_pole() {
        // |0> -> theta=0 -> Bloch (0,0,1)
        let rho = StatePreparation::pure_state(0.0, 0.0);
        let bloch = BlochSphere::from_density_matrix(&rho);
        assert!(bloch[0].abs() < TOL, "r_x should be 0, got {}", bloch[0]);
        assert!(bloch[1].abs() < TOL, "r_y should be 0, got {}", bloch[1]);
        assert!((bloch[2] - 1.0).abs() < TOL, "r_z should be 1, got {}", bloch[2]);
    }

    #[test]
    fn test_bloch_state_1_is_south_pole() {
        // |1> -> theta=pi -> Bloch (0,0,-1)
        let rho = StatePreparation::pure_state(PI, 0.0);
        let bloch = BlochSphere::from_density_matrix(&rho);
        assert!(bloch[0].abs() < TOL);
        assert!(bloch[1].abs() < TOL);
        assert!((bloch[2] - (-1.0)).abs() < TOL);
    }

    #[test]
    fn test_bloch_state_plus_is_positive_x() {
        // |+> -> theta=pi/2, phi=0 -> Bloch (1,0,0)
        let rho = StatePreparation::pure_state(PI / 2.0, 0.0);
        let bloch = BlochSphere::from_density_matrix(&rho);
        assert!((bloch[0] - 1.0).abs() < TOL, "r_x should be 1, got {}", bloch[0]);
        assert!(bloch[1].abs() < TOL, "r_y should be 0, got {}", bloch[1]);
        assert!(bloch[2].abs() < TOL, "r_z should be 0, got {}", bloch[2]);
    }

    #[test]
    fn test_bloch_state_plus_y() {
        // |+y> -> theta=pi/2, phi=pi/2 -> Bloch (0,1,0)
        let rho = StatePreparation::pure_state(PI / 2.0, PI / 2.0);
        let bloch = BlochSphere::from_density_matrix(&rho);
        assert!(bloch[0].abs() < TOL, "r_x should be 0, got {}", bloch[0]);
        assert!((bloch[1] - 1.0).abs() < TOL, "r_y should be 1, got {}", bloch[1]);
        assert!(bloch[2].abs() < TOL, "r_z should be 0, got {}", bloch[2]);
    }

    #[test]
    fn test_bloch_maximally_mixed_is_origin() {
        let rho = StatePreparation::maximally_mixed(2);
        let bloch = BlochSphere::from_density_matrix(&rho);
        assert!(bloch[0].abs() < TOL);
        assert!(bloch[1].abs() < TOL);
        assert!(bloch[2].abs() < TOL);
    }

    #[test]
    fn test_bloch_roundtrip() {
        // Convert Bloch -> density matrix -> Bloch and verify identity
        let original = [0.3, -0.5, 0.7];
        let rho = BlochSphere::to_density_matrix(original);
        let recovered = BlochSphere::from_density_matrix(&rho);
        for i in 0..3 {
            assert!(
                (recovered[i] - original[i]).abs() < TOL,
                "Bloch component {} mismatch: {} vs {}",
                i,
                recovered[i],
                original[i]
            );
        }
    }

    #[test]
    fn test_bloch_purity_pure_state() {
        let bloch = [0.0, 0.0, 1.0]; // |0>
        assert!((BlochSphere::purity_from_bloch(bloch) - 1.0).abs() < TOL);
    }

    #[test]
    fn test_bloch_purity_maximally_mixed() {
        let bloch = [0.0, 0.0, 0.0];
        assert!((BlochSphere::purity_from_bloch(bloch) - 0.5).abs() < TOL);
    }

    #[test]
    fn test_bloch_vector_length_pure() {
        let bloch = [1.0 / 3.0_f64.sqrt(), 1.0 / 3.0_f64.sqrt(), 1.0 / 3.0_f64.sqrt()];
        assert!((BlochSphere::bloch_vector_length(bloch) - 1.0).abs() < TOL);
    }

    // ── Born probability tests ────────────────────────────────────────────

    #[test]
    fn test_born_probability_state_0_measure_z() {
        let rho = StatePreparation::pure_state(0.0, 0.0); // |0>
        let proj_0 = QuantumStateTomography::measurement_operator(&MeasurementBasis::Z, 0);
        let proj_1 = QuantumStateTomography::measurement_operator(&MeasurementBasis::Z, 1);
        let p0 = QuantumStateTomography::born_probability(&rho, &proj_0);
        let p1 = QuantumStateTomography::born_probability(&rho, &proj_1);
        assert!((p0 - 1.0).abs() < TOL, "P(0|0>) should be 1, got {}", p0);
        assert!(p1.abs() < TOL, "P(1|0>) should be 0, got {}", p1);
    }

    #[test]
    fn test_born_probability_state_plus_measure_x() {
        let rho = StatePreparation::pure_state(PI / 2.0, 0.0); // |+>
        let proj_0 = QuantumStateTomography::measurement_operator(&MeasurementBasis::X, 0);
        let p0 = QuantumStateTomography::born_probability(&rho, &proj_0);
        assert!((p0 - 1.0).abs() < TOL, "P(+|+>) should be 1, got {}", p0);
    }

    #[test]
    fn test_born_probability_state_0_measure_x() {
        // |0> measured in X basis should give 50/50
        let rho = StatePreparation::pure_state(0.0, 0.0);
        let proj_0 = QuantumStateTomography::measurement_operator(&MeasurementBasis::X, 0);
        let p0 = QuantumStateTomography::born_probability(&rho, &proj_0);
        assert!((p0 - 0.5).abs() < TOL, "P(+|0>) should be 0.5, got {}", p0);
    }

    #[test]
    fn test_born_probability_maximally_mixed() {
        // Maximally mixed should give 0.5 for any measurement
        let rho = StatePreparation::maximally_mixed(2);
        for basis in &[MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z] {
            let proj = QuantumStateTomography::measurement_operator(basis, 0);
            let p = QuantumStateTomography::born_probability(&rho, &proj);
            assert!((p - 0.5).abs() < TOL, "Maximally mixed should give 0.5 in {:?} basis, got {}", basis, p);
        }
    }

    // ── Fidelity and entropy tests ────────────────────────────────────────

    #[test]
    fn test_fidelity_identical_pure_states() {
        let rho = StatePreparation::pure_state(0.0, 0.0); // |0>
        let psi = vec![(1.0, 0.0), (0.0, 0.0)]; // |0>
        let f = StateFidelity::fidelity_pure(&rho, &psi);
        assert!((f - 1.0).abs() < TOL);
    }

    #[test]
    fn test_fidelity_orthogonal_states() {
        let rho = StatePreparation::pure_state(0.0, 0.0); // |0>
        let psi = vec![(0.0, 0.0), (1.0, 0.0)]; // |1>
        let f = StateFidelity::fidelity_pure(&rho, &psi);
        assert!(f.abs() < TOL);
    }

    #[test]
    fn test_von_neumann_entropy_pure_state() {
        let rho = StatePreparation::pure_state(0.0, 0.0);
        let s = StateFidelity::von_neumann_entropy(&rho);
        assert!(s.abs() < TOL, "Pure state entropy should be 0, got {}", s);
    }

    #[test]
    fn test_von_neumann_entropy_maximally_mixed() {
        let rho = StatePreparation::maximally_mixed(2);
        let s = StateFidelity::von_neumann_entropy(&rho);
        assert!((s - 1.0).abs() < TOL, "Maximally mixed qubit entropy should be 1 bit, got {}", s);
    }

    #[test]
    fn test_purity_pure_state() {
        let rho = StatePreparation::pure_state(PI / 4.0, PI / 3.0);
        let p = StateFidelity::purity(&rho);
        assert!((p - 1.0).abs() < TOL, "Pure state purity should be 1, got {}", p);
    }

    #[test]
    fn test_purity_maximally_mixed() {
        let rho = StatePreparation::maximally_mixed(2);
        let p = StateFidelity::purity(&rho);
        assert!((p - 0.5).abs() < TOL, "Maximally mixed qubit purity should be 0.5, got {}", p);
    }

    #[test]
    fn test_trace_distance_identical() {
        let rho = StatePreparation::pure_state(0.0, 0.0);
        let sigma = StatePreparation::pure_state(0.0, 0.0);
        let td = StateFidelity::trace_distance(&rho, &sigma);
        assert!(td.abs() < TOL, "Trace distance of identical states should be 0, got {}", td);
    }

    #[test]
    fn test_trace_distance_orthogonal() {
        let rho = StatePreparation::pure_state(0.0, 0.0); // |0>
        let sigma = StatePreparation::pure_state(PI, 0.0); // |1>
        let td = StateFidelity::trace_distance(&rho, &sigma);
        assert!((td - 1.0).abs() < TOL, "Trace distance of orthogonal states should be 1, got {}", td);
    }

    // ── Linear inversion tests ────────────────────────────────────────────

    #[test]
    fn test_linear_inversion_state_0() {
        let config = TomographyConfig {
            num_qubits: 1,
            measurement_bases: vec![MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z],
            num_shots: 100_000,
            mle_iterations: 0,
        };
        let qst = QuantumStateTomography::new(config.clone());
        let rho_true = StatePreparation::pure_state(0.0, 0.0); // |0>

        let counts: Vec<(MeasurementBasis, Vec<usize>)> = config
            .measurement_bases
            .iter()
            .map(|basis| {
                let outcomes = StatePreparation::simulate_measurements(&rho_true, basis, config.num_shots);
                (*basis, outcomes)
            })
            .collect();

        let rho_recon = qst.linear_inversion(&counts);
        let bloch = BlochSphere::from_density_matrix(&rho_recon);
        // |0> should give Bloch vector close to (0, 0, 1)
        assert!(bloch[0].abs() < STAT_TOL, "r_x should be ~0, got {}", bloch[0]);
        assert!(bloch[1].abs() < STAT_TOL, "r_y should be ~0, got {}", bloch[1]);
        assert!((bloch[2] - 1.0).abs() < STAT_TOL, "r_z should be ~1, got {}", bloch[2]);
    }

    #[test]
    fn test_linear_inversion_state_plus() {
        let config = TomographyConfig {
            num_qubits: 1,
            measurement_bases: vec![MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z],
            num_shots: 100_000,
            mle_iterations: 0,
        };
        let qst = QuantumStateTomography::new(config.clone());
        let rho_true = StatePreparation::pure_state(PI / 2.0, 0.0); // |+>

        let counts: Vec<(MeasurementBasis, Vec<usize>)> = config
            .measurement_bases
            .iter()
            .map(|basis| {
                let outcomes = StatePreparation::simulate_measurements(&rho_true, basis, config.num_shots);
                (*basis, outcomes)
            })
            .collect();

        let rho_recon = qst.linear_inversion(&counts);
        let bloch = BlochSphere::from_density_matrix(&rho_recon);
        assert!((bloch[0] - 1.0).abs() < STAT_TOL, "r_x should be ~1, got {}", bloch[0]);
        assert!(bloch[1].abs() < STAT_TOL, "r_y should be ~0, got {}", bloch[1]);
        assert!(bloch[2].abs() < STAT_TOL, "r_z should be ~0, got {}", bloch[2]);
    }

    // ── MLE tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_mle_produces_physical_state() {
        let config = TomographyConfig {
            num_qubits: 1,
            measurement_bases: vec![MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z],
            num_shots: 10_000,
            mle_iterations: 50,
        };
        let qst = QuantumStateTomography::new(config.clone());
        let rho_true = StatePreparation::pure_state(PI / 3.0, PI / 4.0);

        let counts: Vec<(MeasurementBasis, Vec<usize>)> = config
            .measurement_bases
            .iter()
            .map(|basis| {
                let outcomes = StatePreparation::simulate_measurements(&rho_true, basis, config.num_shots);
                (*basis, outcomes)
            })
            .collect();

        let rho_mle = qst.maximum_likelihood(&counts, config.mle_iterations);

        // Check physicality
        assert!(rho_mle.is_hermitian(1e-8), "MLE result should be Hermitian");
        assert!(
            rho_mle.is_positive_semidefinite(1e-8),
            "MLE result should be positive semi-definite"
        );
        let tr = rho_mle.trace();
        assert!((tr.0 - 1.0).abs() < 1e-8, "MLE result should have unit trace, got {}", tr.0);
        assert!(tr.1.abs() < 1e-8);
    }

    #[test]
    fn test_mle_recovers_state_0() {
        let config = TomographyConfig {
            num_qubits: 1,
            measurement_bases: vec![MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z],
            num_shots: 50_000,
            mle_iterations: 100,
        };
        let qst = QuantumStateTomography::new(config.clone());
        let rho_true = StatePreparation::pure_state(0.0, 0.0); // |0>

        let counts: Vec<(MeasurementBasis, Vec<usize>)> = config
            .measurement_bases
            .iter()
            .map(|basis| {
                let outcomes = StatePreparation::simulate_measurements(&rho_true, basis, config.num_shots);
                (*basis, outcomes)
            })
            .collect();

        let rho_mle = qst.maximum_likelihood(&counts, config.mle_iterations);
        let f = StateFidelity::fidelity_pure(&rho_mle, &[(1.0, 0.0), (0.0, 0.0)]);
        assert!(f > 0.95, "MLE fidelity with |0> should be >0.95, got {}", f);
    }

    // ── State preparation tests ───────────────────────────────────────────

    #[test]
    fn test_pure_state_is_valid_density_matrix() {
        for theta in [0.0, PI / 4.0, PI / 2.0, PI] {
            for phi in [0.0, PI / 4.0, PI / 2.0, PI] {
                let rho = StatePreparation::pure_state(theta, phi);
                assert!(rho.is_hermitian(TOL), "Pure state should be Hermitian");
                assert!(
                    rho.is_positive_semidefinite(TOL),
                    "Pure state should be PSD"
                );
                let tr = rho.trace();
                assert!((tr.0 - 1.0).abs() < TOL, "Pure state should have trace 1");
                let p = StateFidelity::purity(&rho);
                assert!((p - 1.0).abs() < TOL, "Pure state should have purity 1");
            }
        }
    }

    #[test]
    fn test_maximally_mixed_properties() {
        let rho = StatePreparation::maximally_mixed(2);
        assert!(rho.is_hermitian(TOL));
        assert!(rho.is_positive_semidefinite(TOL));
        let tr = rho.trace();
        assert!((tr.0 - 1.0).abs() < TOL);
        // Off-diagonal should be zero
        assert!(complex_abs(rho.get(0, 1)) < TOL);
        assert!(complex_abs(rho.get(1, 0)) < TOL);
    }

    #[test]
    fn test_bell_state_phi_plus() {
        let rho = StatePreparation::bell_state(0); // |Phi+>
        assert!(rho.is_hermitian(TOL));
        let tr = rho.trace();
        assert!((tr.0 - 1.0).abs() < TOL);
        let p = StateFidelity::purity(&rho);
        assert!((p - 1.0).abs() < TOL, "Bell state should be pure");
    }

    #[test]
    fn test_bell_states_are_orthogonal() {
        for i in 0..4 {
            for j in 0..4 {
                if i == j {
                    continue;
                }
                let rho_i = StatePreparation::bell_state(i);
                let rho_j = StatePreparation::bell_state(j);
                let product = rho_i.multiply(&rho_j);
                let overlap = product.trace().0;
                assert!(
                    overlap.abs() < TOL,
                    "Bell states {} and {} should be orthogonal, overlap = {}",
                    i,
                    j,
                    overlap
                );
            }
        }
    }

    // ── Measurement operator tests ────────────────────────────────────────

    #[test]
    fn test_measurement_operators_are_projectors() {
        for basis in &[MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z] {
            for outcome in 0..2 {
                let proj = QuantumStateTomography::measurement_operator(basis, outcome);
                // Projector: P^2 = P
                let proj_sq = proj.multiply(&proj);
                for i in 0..2 {
                    for j in 0..2 {
                        let diff = complex_sub(proj_sq.get(i, j), proj.get(i, j));
                        assert!(
                            complex_abs(diff) < TOL,
                            "Projector^2 != Projector for {:?} outcome {} at ({},{})",
                            basis, outcome, i, j
                        );
                    }
                }
                // Projector: Tr(P) = 1 (rank 1)
                let tr = proj.trace().0;
                assert!(
                    (tr - 1.0).abs() < TOL,
                    "Projector trace should be 1, got {} for {:?} outcome {}",
                    tr, basis, outcome
                );
            }
        }
    }

    #[test]
    fn test_measurement_operators_completeness() {
        // For each basis, P_0 + P_1 = I
        for basis in &[MeasurementBasis::X, MeasurementBasis::Y, MeasurementBasis::Z] {
            let p0 = QuantumStateTomography::measurement_operator(basis, 0);
            let p1 = QuantumStateTomography::measurement_operator(basis, 1);
            let sum = p0.add(&p1);
            let eye = ComplexMatrix::identity(2);
            for i in 0..2 {
                for j in 0..2 {
                    let diff = complex_sub(sum.get(i, j), eye.get(i, j));
                    assert!(
                        complex_abs(diff) < TOL,
                        "P0 + P1 != I for {:?} at ({},{})",
                        basis, i, j
                    );
                }
            }
        }
    }

    // ── Simulate measurements test ────────────────────────────────────────

    #[test]
    fn test_simulate_measurements_statistics() {
        let rho = StatePreparation::pure_state(0.0, 0.0); // |0>
        let outcomes = StatePreparation::simulate_measurements(&rho, &MeasurementBasis::Z, 10_000);
        let count_0 = outcomes.iter().filter(|&&o| o == 0).count();
        // Should be all 0s (or very close)
        assert!(
            count_0 > 9900,
            "Measuring |0> in Z should give almost all 0s, got {} out of 10000",
            count_0
        );
    }

    // ── Concurrence test ──────────────────────────────────────────────────

    #[test]
    fn test_concurrence_bell_state() {
        let rho = StatePreparation::bell_state(0); // |Phi+> - maximally entangled
        let c = StateFidelity::concurrence_2qubit(&rho);
        assert!(
            (c - 1.0).abs() < 0.1,
            "Bell state concurrence should be ~1, got {}",
            c
        );
    }

    #[test]
    fn test_concurrence_product_state() {
        // |00> is a product state with zero entanglement
        let psi = vec![
            (1.0, 0.0), (0.0, 0.0),
            (0.0, 0.0), (0.0, 0.0),
        ];
        let rho = ComplexMatrix::outer_product(&psi);
        let c = StateFidelity::concurrence_2qubit(&rho);
        assert!(
            c < 0.1,
            "Product state concurrence should be ~0, got {}",
            c
        );
    }

    // ── project_physical test ─────────────────────────────────────────────

    #[test]
    fn test_project_physical_unphysical_matrix() {
        // Create an unphysical "density matrix" with negative eigenvalue
        let mut rho = ComplexMatrix::new_zeros(2, 2);
        rho.set(0, 0, (1.5, 0.0));
        rho.set(1, 1, (-0.5, 0.0));
        // This has eigenvalues 1.5 and -0.5 (not physical)
        QuantumStateTomography::project_physical(&mut rho);

        assert!(rho.is_hermitian(1e-8));
        assert!(rho.is_positive_semidefinite(1e-8));
        let tr = rho.trace();
        assert!((tr.0 - 1.0).abs() < 1e-8);
    }

    // ── Density matrix roundtrip via Bloch sphere ─────────────────────────

    #[test]
    fn test_density_matrix_bloch_roundtrip() {
        // Create density matrix, convert to Bloch, convert back
        let rho_orig = StatePreparation::pure_state(PI / 3.0, PI / 5.0);
        let bloch = BlochSphere::from_density_matrix(&rho_orig);
        let rho_recon = BlochSphere::to_density_matrix(bloch);

        for i in 0..2 {
            for j in 0..2 {
                let diff = complex_sub(rho_orig.get(i, j), rho_recon.get(i, j));
                assert!(
                    complex_abs(diff) < TOL,
                    "Density matrix Bloch roundtrip failed at ({},{}): orig={:?}, recon={:?}",
                    i, j, rho_orig.get(i, j), rho_recon.get(i, j)
                );
            }
        }
    }

    #[test]
    fn test_bloch_to_density_matrix_mixed_state() {
        // A mixed state inside the Bloch sphere
        let bloch = [0.3, 0.4, 0.5];
        let rho = BlochSphere::to_density_matrix(bloch);
        assert!(rho.is_hermitian(TOL));
        let tr = rho.trace();
        assert!((tr.0 - 1.0).abs() < TOL);
        let p = StateFidelity::purity(&rho);
        let expected_purity = BlochSphere::purity_from_bloch(bloch);
        assert!(
            (p - expected_purity).abs() < TOL,
            "Purity mismatch: {} vs {}",
            p,
            expected_purity
        );
    }
}

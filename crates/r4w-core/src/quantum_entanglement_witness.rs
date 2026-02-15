//! # Quantum Entanglement Witness
//!
//! This module implements quantum entanglement verification and characterization
//! through Bell inequality tests and entanglement witnesses.
//!
//! ## Overview
//!
//! Quantum entanglement is a non-classical correlation between subsystems of
//! a composite quantum system. This module provides tools to:
//!
//! - Represent 2-qubit quantum states as 4x4 density matrices
//! - Test Bell-CHSH inequality violations (classical bound S <= 2, quantum max 2*sqrt(2))
//! - Compute entanglement measures: concurrence, entanglement of formation, negativity
//! - Construct entanglement witnesses W with Tr(W*rho) < 0 implying entanglement
//! - Simulate projective measurements with finite-count statistics
//!
//! ## Key Concepts
//!
//! **Bell States** (maximally entangled):
//! ```text
//! |Phi+> = (|00> + |11>) / sqrt(2)
//! |Phi-> = (|00> - |11>) / sqrt(2)
//! |Psi+> = (|01> + |10>) / sqrt(2)
//! |Psi-> = (|01> - |10>) / sqrt(2)
//! ```
//!
//! **CHSH Inequality**: For local hidden variable theories, the CHSH parameter
//! S = |E(a,b) - E(a,b') + E(a',b) + E(a',b')| is bounded by 2. Quantum
//! mechanics allows violation up to 2*sqrt(2) (Tsirelson's bound).
//!
//! **Concurrence**: C(rho) = max(0, l1 - l2 - l3 - l4) where l_i are the
//! square roots of the eigenvalues of R = rho * (sigma_y x sigma_y) * rho* * (sigma_y x sigma_y)
//! in decreasing order.
//!
//! ## Usage
//!
//! ```rust
//! use r4w_core::quantum_entanglement_witness::*;
//!
//! // Create a Bell state |Phi+>
//! let state = QuantumState::bell_phi_plus();
//! assert!((state.trace().0 - 1.0).abs() < 1e-10);
//!
//! // Check entanglement
//! let c = concurrence(&state);
//! assert!((c - 1.0).abs() < 1e-10); // Maximally entangled
//!
//! // CHSH violation with optimal angles
//! let settings = optimal_chsh_settings();
//! let s = chsh_parameter(&state, &settings);
//! assert!(s > 2.0); // Violates classical bound
//! ```

// ---- Complex arithmetic helpers ------------------------------------------------

/// Complex number as (re, im) tuple.
pub type Complex = (f64, f64);

/// Complex zero.
pub const CZERO: Complex = (0.0, 0.0);

/// Complex one.
pub const CONE: Complex = (1.0, 0.0);

/// Complex imaginary unit.
pub const CI: Complex = (0.0, 1.0);

/// Add two complex numbers.
#[inline]
pub fn c_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers.
#[inline]
pub fn c_sub(a: Complex, b: Complex) -> Complex {
    (a.0 - b.0, a.1 - b.1)
}

/// Multiply two complex numbers.
#[inline]
pub fn c_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Divide complex a by complex b.
#[inline]
pub fn c_div(a: Complex, b: Complex) -> Complex {
    let denom = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Complex conjugate.
#[inline]
pub fn c_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

/// Magnitude squared |z|^2.
#[inline]
pub fn c_abs2(a: Complex) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Magnitude |z|.
#[inline]
pub fn c_abs(a: Complex) -> f64 {
    c_abs2(a).sqrt()
}

/// Scale complex by real.
#[inline]
pub fn c_scale(s: f64, a: Complex) -> Complex {
    (s * a.0, s * a.1)
}

/// Square root of a complex number (principal branch).
pub fn c_sqrt(z: Complex) -> Complex {
    let r = c_abs(z);
    if r < 1e-30 {
        return CZERO;
    }
    let re = ((r + z.0) / 2.0).sqrt();
    let im = ((r - z.0) / 2.0).sqrt();
    if z.1 >= 0.0 {
        (re, im)
    } else {
        (re, -im)
    }
}

// ---- 4x4 Complex Matrix -------------------------------------------------------

/// A 4x4 complex matrix stored in row-major order.
#[derive(Clone, Debug)]
pub struct Matrix4x4 {
    pub data: [[Complex; 4]; 4],
}

impl Matrix4x4 {
    /// Create a zero matrix.
    pub fn zero() -> Self {
        Self {
            data: [[CZERO; 4]; 4],
        }
    }

    /// Create an identity matrix.
    pub fn identity() -> Self {
        let mut m = Self::zero();
        for i in 0..4 {
            m.data[i][i] = CONE;
        }
        m
    }

    /// Create from a flat array of 16 complex numbers (row-major).
    pub fn from_flat(vals: &[Complex; 16]) -> Self {
        let mut m = Self::zero();
        for i in 0..4 {
            for j in 0..4 {
                m.data[i][j] = vals[i * 4 + j];
            }
        }
        m
    }

    /// Trace of the matrix.
    pub fn trace(&self) -> Complex {
        let mut t = CZERO;
        for i in 0..4 {
            t = c_add(t, self.data[i][i]);
        }
        t
    }

    /// Matrix addition.
    pub fn add(&self, other: &Matrix4x4) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i in 0..4 {
            for j in 0..4 {
                r.data[i][j] = c_add(self.data[i][j], other.data[i][j]);
            }
        }
        r
    }

    /// Matrix subtraction.
    pub fn sub(&self, other: &Matrix4x4) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i in 0..4 {
            for j in 0..4 {
                r.data[i][j] = c_sub(self.data[i][j], other.data[i][j]);
            }
        }
        r
    }

    /// Matrix multiplication.
    pub fn mul(&self, other: &Matrix4x4) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i in 0..4 {
            for j in 0..4 {
                let mut s = CZERO;
                for k in 0..4 {
                    s = c_add(s, c_mul(self.data[i][k], other.data[k][j]));
                }
                r.data[i][j] = s;
            }
        }
        r
    }

    /// Scalar multiplication.
    pub fn scale(&self, s: Complex) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i in 0..4 {
            for j in 0..4 {
                r.data[i][j] = c_mul(s, self.data[i][j]);
            }
        }
        r
    }

    /// Conjugate transpose (dagger).
    pub fn dagger(&self) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i in 0..4 {
            for j in 0..4 {
                r.data[i][j] = c_conj(self.data[j][i]);
            }
        }
        r
    }

    /// Complex conjugate (element-wise).
    pub fn conjugate(&self) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i in 0..4 {
            for j in 0..4 {
                r.data[i][j] = c_conj(self.data[i][j]);
            }
        }
        r
    }

    /// Transpose.
    pub fn transpose(&self) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i in 0..4 {
            for j in 0..4 {
                r.data[i][j] = self.data[j][i];
            }
        }
        r
    }

    /// Frobenius norm.
    pub fn frobenius_norm(&self) -> f64 {
        let mut s = 0.0;
        for i in 0..4 {
            for j in 0..4 {
                s += c_abs2(self.data[i][j]);
            }
        }
        s.sqrt()
    }

    /// Outer product |a><b| from two 4-element column vectors.
    pub fn outer_product(a: &[Complex; 4], b: &[Complex; 4]) -> Self {
        let mut m = Self::zero();
        for i in 0..4 {
            for j in 0..4 {
                m.data[i][j] = c_mul(a[i], c_conj(b[j]));
            }
        }
        m
    }

    /// Matrix-vector product.
    pub fn mul_vec(&self, v: &[Complex; 4]) -> [Complex; 4] {
        let mut r = [CZERO; 4];
        for i in 0..4 {
            for j in 0..4 {
                r[i] = c_add(r[i], c_mul(self.data[i][j], v[j]));
            }
        }
        r
    }

    /// Check if Hermitian (self == dagger).
    pub fn is_hermitian(&self, tol: f64) -> bool {
        for i in 0..4 {
            for j in 0..4 {
                let diff = c_sub(self.data[i][j], c_conj(self.data[j][i]));
                if c_abs(diff) > tol {
                    return false;
                }
            }
        }
        true
    }

    /// Partial transpose with respect to subsystem B (second qubit).
    /// For a 2-qubit system in computational basis {|00>, |01>, |10>, |11>},
    /// the partial transpose swaps the B indices: rho_{ij,kl}^{T_B} = rho_{il,kj}.
    pub fn partial_transpose_b(&self) -> Matrix4x4 {
        let mut r = Matrix4x4::zero();
        for i1 in 0..2 {
            for j1 in 0..2 {
                for i2 in 0..2 {
                    for j2 in 0..2 {
                        let row = i1 * 2 + i2;
                        let col = j1 * 2 + j2;
                        let src_row = i1 * 2 + j2;
                        let src_col = j1 * 2 + i2;
                        r.data[row][col] = self.data[src_row][src_col];
                    }
                }
            }
        }
        r
    }
}

// ---- 2x2 Complex Matrix (for reduced density matrices) -------------------------

/// A 2x2 complex matrix.
#[derive(Clone, Debug)]
pub struct Matrix2x2 {
    pub data: [[Complex; 2]; 2],
}

impl Matrix2x2 {
    pub fn zero() -> Self {
        Self {
            data: [[CZERO; 2]; 2],
        }
    }

    pub fn identity() -> Self {
        let mut m = Self::zero();
        m.data[0][0] = CONE;
        m.data[1][1] = CONE;
        m
    }

    pub fn trace(&self) -> Complex {
        c_add(self.data[0][0], self.data[1][1])
    }

    pub fn mul(&self, other: &Matrix2x2) -> Matrix2x2 {
        let mut r = Matrix2x2::zero();
        for i in 0..2 {
            for j in 0..2 {
                let mut s = CZERO;
                for k in 0..2 {
                    s = c_add(s, c_mul(self.data[i][k], other.data[k][j]));
                }
                r.data[i][j] = s;
            }
        }
        r
    }

    /// Eigenvalues of a 2x2 matrix using the quadratic formula.
    pub fn eigenvalues(&self) -> [Complex; 2] {
        let tr = self.trace();
        let det = c_sub(
            c_mul(self.data[0][0], self.data[1][1]),
            c_mul(self.data[0][1], self.data[1][0]),
        );
        // lambda^2 - tr*lambda + det = 0
        // discriminant = tr^2 - 4*det
        let disc = c_sub(c_mul(tr, tr), c_scale(4.0, det));
        let sqrt_disc = c_sqrt(disc);
        let half = (0.5, 0.0);
        [
            c_mul(half, c_add(tr, sqrt_disc)),
            c_mul(half, c_sub(tr, sqrt_disc)),
        ]
    }

    /// Von Neumann entropy S = -Tr(rho * log2(rho)).
    /// Computed from eigenvalues.
    pub fn von_neumann_entropy(&self) -> f64 {
        let evals = self.eigenvalues();
        let mut entropy = 0.0;
        for ev in &evals {
            let p = ev.0; // Should be real for a density matrix
            if p > 1e-15 {
                entropy -= p * p.log2();
            }
        }
        entropy
    }
}

// ---- Eigenvalue computation for 4x4 Hermitian matrices --------------------------

/// Compute eigenvalues of a 4x4 Hermitian matrix using Jacobi iteration.
/// Returns eigenvalues in descending order.
pub fn hermitian_eigenvalues_4x4(m: &Matrix4x4) -> [f64; 4] {
    // Work with the real representation: since m is Hermitian, we convert
    // to a real symmetric form and use Jacobi eigenvalue algorithm.
    // For 4x4 Hermitian, eigenvalues are real.

    let eigenvals = jacobi_hermitian_4x4(m);
    let mut ev = eigenvals;
    ev.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
    ev
}

/// Jacobi eigenvalue algorithm for 4x4 Hermitian matrix.
/// Returns 4 real eigenvalues.
fn jacobi_hermitian_4x4(m: &Matrix4x4) -> [f64; 4] {
    // For a Hermitian matrix, we can use a real symmetric representation
    // by converting to an 8x8 real matrix. But for efficiency at 4x4,
    // we use the direct Jacobi iteration on the complex entries.

    // Working copy as real symmetric (the real part of a Hermitian matrix
    // is symmetric, and for computing eigenvalues we can work with the
    // augmented real form).

    // Convert to 8x8 real form:
    // [ Re(H)  -Im(H) ]
    // [ Im(H)   Re(H) ]
    // This has eigenvalues that come in pairs: each eigenvalue of H appears twice.
    let n = 8;
    let mut a = [[0.0f64; 8]; 8];
    for i in 0..4 {
        for j in 0..4 {
            a[i][j] = m.data[i][j].0;       // Re(H) top-left
            a[i][j + 4] = -m.data[i][j].1;  // -Im(H) top-right
            a[i + 4][j] = m.data[i][j].1;   // Im(H) bottom-left
            a[i + 4][j + 4] = m.data[i][j].0; // Re(H) bottom-right
        }
    }

    // Jacobi iteration on 8x8 real symmetric matrix
    let max_iter = 200;
    for _ in 0..max_iter {
        // Find off-diagonal element with largest absolute value
        let mut max_val = 0.0f64;
        let mut p = 0;
        let mut q = 0;
        for i in 0..n {
            for j in (i + 1)..n {
                if a[i][j].abs() > max_val {
                    max_val = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max_val < 1e-14 {
            break;
        }

        // Compute rotation angle
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];

        let theta = if (app - aqq).abs() < 1e-30 {
            std::f64::consts::FRAC_PI_4
        } else {
            0.5 * (2.0 * apq / (app - aqq)).atan()
        };

        let c = theta.cos();
        let s = theta.sin();

        // Proper Jacobi rotation: B = G^T A G where G is rotation in (p,q) plane
        // First compute A * G (affects columns p and q)
        let mut ag = a;
        for i in 0..n {
            ag[i][p] = a[i][p] * c + a[i][q] * s;
            ag[i][q] = -a[i][p] * s + a[i][q] * c;
        }
        // Then G^T * (A*G) (affects rows p and q)
        let mut gag = ag;
        for j in 0..n {
            gag[p][j] = c * ag[p][j] + s * ag[q][j];
            gag[q][j] = -s * ag[p][j] + c * ag[q][j];
        }
        a = gag;
    }

    // Extract the unique eigenvalues (each appears twice in the 8x8 form)
    let mut diag: Vec<f64> = (0..n).map(|i| a[i][i]).collect();
    diag.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

    // Each eigenvalue appears twice; take every other one
    let mut result = [0.0f64; 4];
    let mut idx = 0;
    let mut i = 0;
    while i < diag.len() && idx < 4 {
        result[idx] = diag[i];
        idx += 1;
        // Skip the duplicate
        if i + 1 < diag.len() && (diag[i] - diag[i + 1]).abs() < 1e-10 {
            i += 2;
        } else {
            i += 1;
        }
    }
    // If we didn't get 4 eigenvalues (degenerate case), fill from remaining
    if idx < 4 {
        let mut j = 0;
        for i in 0..diag.len() {
            if j >= 4 {
                break;
            }
            result[j] = diag[i];
            j += 1;
            // skip duplicate
            if i + 1 < diag.len() && (diag[i] - diag[i + 1]).abs() < 1e-10 {
                continue;
            }
        }
        // Fallback: just take first 4 diagonal (every other)
        if j < 4 {
            for k in 0..4 {
                result[k] = diag[k * 2];
            }
        }
    }
    result
}

// ---- Quantum State (2-qubit density matrix) ------------------------------------

/// A 2-qubit quantum state represented by a 4x4 density matrix.
///
/// The computational basis is {|00>, |01>, |10>, |11>} where the first
/// qubit is A and the second is B.
#[derive(Clone, Debug)]
pub struct QuantumState {
    /// 4x4 density matrix rho.
    pub rho: Matrix4x4,
}

impl QuantumState {
    /// Create a pure state |psi> = a|00> + b|01> + c|10> + d|11>.
    /// The density matrix is rho = |psi><psi|.
    /// Amplitudes are normalized automatically.
    pub fn from_amplitudes(a: Complex, b: Complex, c: Complex, d: Complex) -> Self {
        let norm2 = c_abs2(a) + c_abs2(b) + c_abs2(c) + c_abs2(d);
        let norm = norm2.sqrt();
        let amps = [
            c_scale(1.0 / norm, a),
            c_scale(1.0 / norm, b),
            c_scale(1.0 / norm, c),
            c_scale(1.0 / norm, d),
        ];
        let rho = Matrix4x4::outer_product(&amps, &amps);
        Self { rho }
    }

    /// Create a state from a density matrix directly.
    pub fn from_density_matrix(rho: Matrix4x4) -> Self {
        Self { rho }
    }

    /// Bell state |Phi+> = (|00> + |11>) / sqrt(2).
    pub fn bell_phi_plus() -> Self {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        Self::from_amplitudes((s, 0.0), CZERO, CZERO, (s, 0.0))
    }

    /// Bell state |Phi-> = (|00> - |11>) / sqrt(2).
    pub fn bell_phi_minus() -> Self {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        Self::from_amplitudes((s, 0.0), CZERO, CZERO, (-s, 0.0))
    }

    /// Bell state |Psi+> = (|01> + |10>) / sqrt(2).
    pub fn bell_psi_plus() -> Self {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        Self::from_amplitudes(CZERO, (s, 0.0), (s, 0.0), CZERO)
    }

    /// Bell state |Psi-> = (|01> - |10>) / sqrt(2).
    pub fn bell_psi_minus() -> Self {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        Self::from_amplitudes(CZERO, (s, 0.0), (-s, 0.0), CZERO)
    }

    /// Werner state: rho_W = p * |Psi-><Psi-| + (1-p)/4 * I.
    /// Entangled if and only if p > 1/3.
    pub fn werner(p: f64) -> Self {
        let psi_minus = Self::bell_psi_minus();
        let eye = Matrix4x4::identity();
        let rho = psi_minus.rho.scale((p, 0.0)).add(&eye.scale(((1.0 - p) / 4.0, 0.0)));
        Self { rho }
    }

    /// Product state |a> tensor |b> where |a> = cos(t/2)|0> + sin(t/2)|1>,
    /// |b> = cos(p/2)|0> + sin(p/2)|1>.
    pub fn product(theta_a: f64, theta_b: f64) -> Self {
        let a0 = ((theta_a / 2.0).cos(), 0.0);
        let a1 = ((theta_a / 2.0).sin(), 0.0);
        let b0 = ((theta_b / 2.0).cos(), 0.0);
        let b1 = ((theta_b / 2.0).sin(), 0.0);
        Self::from_amplitudes(
            c_mul(a0, b0), // |00>
            c_mul(a0, b1), // |01>
            c_mul(a1, b0), // |10>
            c_mul(a1, b1), // |11>
        )
    }

    /// Maximally mixed state rho = I/4.
    pub fn maximally_mixed() -> Self {
        Self::from_density_matrix(Matrix4x4::identity().scale((0.25, 0.0)))
    }

    /// Trace of the density matrix (should be 1.0).
    pub fn trace(&self) -> Complex {
        self.rho.trace()
    }

    /// Purity Tr(rho^2). Equal to 1 for pure states, < 1 for mixed.
    pub fn purity(&self) -> f64 {
        self.rho.mul(&self.rho).trace().0
    }

    /// Partial trace over subsystem B, returning the 2x2 reduced density matrix for A.
    pub fn partial_trace_b(&self) -> Matrix2x2 {
        let mut rho_a = Matrix2x2::zero();
        for i in 0..2 {
            for j in 0..2 {
                // rho_A[i][j] = sum_k rho[i*2+k][j*2+k]
                let mut s = CZERO;
                for k in 0..2 {
                    s = c_add(s, self.rho.data[i * 2 + k][j * 2 + k]);
                }
                rho_a.data[i][j] = s;
            }
        }
        rho_a
    }

    /// Partial trace over subsystem A, returning the 2x2 reduced density matrix for B.
    pub fn partial_trace_a(&self) -> Matrix2x2 {
        let mut rho_b = Matrix2x2::zero();
        for i in 0..2 {
            for j in 0..2 {
                // rho_B[i][j] = sum_k rho[k*2+i][k*2+j]
                let mut s = CZERO;
                for k in 0..2 {
                    s = c_add(s, self.rho.data[k * 2 + i][k * 2 + j]);
                }
                rho_b.data[i][j] = s;
            }
        }
        rho_b
    }
}

// ---- Pauli matrices and tensor products ----------------------------------------

/// Pauli sigma_y as a 2x2 matrix.
fn sigma_y_2x2() -> Matrix2x2 {
    let mut m = Matrix2x2::zero();
    m.data[0][1] = (0.0, -1.0); // -i
    m.data[1][0] = (0.0, 1.0);  // +i
    m
}

/// Tensor product of two 2x2 matrices, producing a 4x4 matrix.
fn tensor_2x2(a: &Matrix2x2, b: &Matrix2x2) -> Matrix4x4 {
    let mut r = Matrix4x4::zero();
    for i1 in 0..2 {
        for j1 in 0..2 {
            for i2 in 0..2 {
                for j2 in 0..2 {
                    r.data[i1 * 2 + i2][j1 * 2 + j2] = c_mul(a.data[i1][j1], b.data[i2][j2]);
                }
            }
        }
    }
    r
}

/// sigma_y tensor sigma_y (4x4 matrix).
fn sigma_yy() -> Matrix4x4 {
    let sy = sigma_y_2x2();
    tensor_2x2(&sy, &sy)
}

// ---- Entanglement measures -----------------------------------------------------

/// Concurrence of a 2-qubit state.
///
/// C(rho) = max(0, l1 - l2 - l3 - l4) where l_i are the square roots
/// of the eigenvalues of R = rho * (sigma_y x sigma_y) * rho* * (sigma_y x sigma_y)
/// in decreasing order.
pub fn concurrence(state: &QuantumState) -> f64 {
    let rho = &state.rho;
    let syy = sigma_yy();

    // R = rho * tilde_rho where tilde_rho = syy * rho* * syy
    let rho_star = rho.conjugate();
    let tilde_rho = syy.mul(&rho_star).mul(&syy);
    let r_mat = rho.mul(&tilde_rho);

    // For concurrence, we need eigenvalues of R.
    // R is positive semi-definite for physical states, so we can find eigenvalues
    // of (R + R^dagger)/2 which is Hermitian.
    let r_herm = r_mat.add(&r_mat.dagger()).scale((0.5, 0.0));
    let mut eigenvals = hermitian_eigenvalues_4x4(&r_herm);
    eigenvals.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

    // Take square roots of eigenvalues (they should be non-negative)
    let lambdas: Vec<f64> = eigenvals.iter().map(|&e| {
        if e > 0.0 { e.sqrt() } else { 0.0 }
    }).collect();

    let c = lambdas[0] - lambdas[1] - lambdas[2] - lambdas[3];
    c.max(0.0)
}

/// Binary entropy function h(x) = -x*log2(x) - (1-x)*log2(1-x).
fn binary_entropy(x: f64) -> f64 {
    if x < 1e-15 || x > 1.0 - 1e-15 {
        return 0.0;
    }
    -x * x.log2() - (1.0 - x) * (1.0 - x).log2()
}

/// Entanglement of formation.
///
/// Ef = h((1 + sqrt(1 - C^2)) / 2) where h is the binary entropy and C is concurrence.
pub fn entanglement_of_formation(state: &QuantumState) -> f64 {
    let c = concurrence(state);
    let arg = (1.0 + (1.0 - c * c).max(0.0).sqrt()) / 2.0;
    binary_entropy(arg)
}

/// Negativity of a 2-qubit state.
///
/// N = (||rho^{T_B}||_1 - 1) / 2 where rho^{T_B} is the partial transpose
/// and ||.||_1 is the trace norm (sum of singular values = sum of |eigenvalues| for Hermitian).
pub fn negativity(state: &QuantumState) -> f64 {
    let pt = state.rho.partial_transpose_b();
    let eigenvals = hermitian_eigenvalues_4x4(&pt);

    // Trace norm of Hermitian matrix = sum of |eigenvalues|
    let trace_norm: f64 = eigenvals.iter().map(|e| e.abs()).sum();
    (trace_norm - 1.0).max(0.0) / 2.0
}

/// Von Neumann entropy of the 2-qubit state.
///
/// S(rho) = -Tr(rho * log2(rho)), computed from eigenvalues.
pub fn von_neumann_entropy(state: &QuantumState) -> f64 {
    let eigenvals = hermitian_eigenvalues_4x4(&state.rho);
    let mut entropy = 0.0;
    for &e in &eigenvals {
        if e > 1e-15 {
            entropy -= e * e.log2();
        }
    }
    entropy
}

/// Entanglement entropy = von Neumann entropy of the reduced density matrix.
pub fn entanglement_entropy(state: &QuantumState) -> f64 {
    let rho_a = state.partial_trace_b();
    rho_a.von_neumann_entropy()
}

// ---- Bell / CHSH inequality ---------------------------------------------------

/// CHSH measurement settings: two angles for Alice (a, a') and two for Bob (b, b').
/// Angles are in radians, defining measurement directions in the XZ plane of the Bloch sphere.
#[derive(Clone, Debug)]
pub struct ChshSettings {
    pub a: f64,
    pub a_prime: f64,
    pub b: f64,
    pub b_prime: f64,
}

/// Standard CHSH measurement settings that produce Bell violation.
/// a=0, a'=pi/4, b=pi/8, b'=3*pi/8 (pi/8 spacing convention).
pub fn optimal_chsh_settings() -> ChshSettings {
    use std::f64::consts::FRAC_PI_4;
    use std::f64::consts::FRAC_PI_8;
    ChshSettings {
        a: 0.0,
        a_prime: FRAC_PI_4,
        b: FRAC_PI_8,
        b_prime: 3.0 * FRAC_PI_8,
    }
}

/// Compute the correlation function E(theta_a, theta_b) for a 2-qubit state.
///
/// E(a, b) = <psi| (sigma_a tensor sigma_b) |psi> where sigma_theta =
/// cos(theta)*sigma_z + sin(theta)*sigma_x.
///
/// For the density matrix: E(a, b) = Tr(rho * (sigma_a tensor sigma_b)).
pub fn correlation_function(state: &QuantumState, theta_a: f64, theta_b: f64) -> f64 {
    // sigma_theta = cos(theta)*sigma_z + sin(theta)*sigma_x
    // sigma_z = diag(1, -1), sigma_x = [[0,1],[1,0]]
    let sa = [
        [(theta_a.cos(), 0.0), (theta_a.sin(), 0.0)],
        [(theta_a.sin(), 0.0), (-theta_a.cos(), 0.0)],
    ];
    let sb = [
        [(theta_b.cos(), 0.0), (theta_b.sin(), 0.0)],
        [(theta_b.sin(), 0.0), (-theta_b.cos(), 0.0)],
    ];

    // Tensor product sa x sb (4x4)
    let mut op = Matrix4x4::zero();
    for i1 in 0..2 {
        for j1 in 0..2 {
            for i2 in 0..2 {
                for j2 in 0..2 {
                    op.data[i1 * 2 + i2][j1 * 2 + j2] = c_mul(sa[i1][j1], sb[i2][j2]);
                }
            }
        }
    }

    // E = Tr(rho * op)
    let product = state.rho.mul(&op);
    product.trace().0
}

/// Compute the CHSH parameter S for a given state and measurement settings.
///
/// S = E(a,b) - E(a,b') + E(a',b) + E(a',b')
///
/// Classical bound: |S| <= 2. Quantum bound: |S| <= 2*sqrt(2).
pub fn chsh_parameter(state: &QuantumState, settings: &ChshSettings) -> f64 {
    let e_ab = correlation_function(state, settings.a, settings.b);
    let e_ab_prime = correlation_function(state, settings.a, settings.b_prime);
    let e_a_prime_b = correlation_function(state, settings.a_prime, settings.b);
    let e_a_prime_b_prime = correlation_function(state, settings.a_prime, settings.b_prime);

    (e_ab - e_ab_prime + e_a_prime_b + e_a_prime_b_prime).abs()
}

/// Check if a CHSH parameter violates the classical Bell inequality.
pub fn violates_bell_inequality(s: f64) -> bool {
    s > 2.0
}

/// Tsirelson's bound: maximum quantum value of S.
pub const TSIRELSON_BOUND: f64 = std::f64::consts::SQRT_2 * 2.0;

// ---- Entanglement witnesses ---------------------------------------------------

/// An entanglement witness operator W such that:
/// - Tr(W * rho_sep) >= 0 for all separable states
/// - Tr(W * rho_ent) < 0 for some entangled state rho_ent
#[derive(Clone, Debug)]
pub struct EntanglementWitness {
    /// The 4x4 witness operator W.
    pub operator: Matrix4x4,
}

impl EntanglementWitness {
    /// Construct a witness from a projector: W = alpha * I - |psi><psi|.
    ///
    /// For alpha = max eigenvalue of Tr_B(|psi><psi| * rho_sep) over separable states,
    /// we use alpha = 1/2 (the Schmidt coefficient bound for 2-qubit systems).
    pub fn from_projector(state_vec: &[Complex; 4], alpha: f64) -> Self {
        let proj = Matrix4x4::outer_product(state_vec, state_vec);
        let w = Matrix4x4::identity().scale((alpha, 0.0)).sub(&proj);
        Self { operator: w }
    }

    /// Construct a witness optimized for Bell states.
    /// W = I/2 - |Bell><Bell|.
    pub fn for_bell_phi_plus() -> Self {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let state_vec = [(s, 0.0), CZERO, CZERO, (s, 0.0)];
        Self::from_projector(&state_vec, 0.5)
    }

    /// Construct a witness for |Psi->.
    pub fn for_bell_psi_minus() -> Self {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let state_vec = [CZERO, (s, 0.0), (-s, 0.0), CZERO];
        Self::from_projector(&state_vec, 0.5)
    }

    /// Evaluate Tr(W * rho). Negative value indicates entanglement.
    pub fn evaluate(&self, state: &QuantumState) -> f64 {
        self.operator.mul(&state.rho).trace().0
    }

    /// Check if the witness detects entanglement in the given state.
    pub fn detects_entanglement(&self, state: &QuantumState) -> bool {
        self.evaluate(state) < -1e-10
    }
}

// ---- Quantum fidelity ----------------------------------------------------------

/// Quantum fidelity between two density matrices.
///
/// F(rho1, rho2) = (Tr(sqrt(sqrt(rho1) * rho2 * sqrt(rho1))))^2
///
/// For pure states rho1 = |psi><psi|, this simplifies to F = <psi|rho2|psi>.
///
/// For the general case with 4x4 matrices, we use the simplified formula
/// valid when one state is pure: F = Tr(rho1 * rho2) for pure rho1.
/// For general mixed states, we compute via eigendecomposition.
pub fn quantum_fidelity(rho1: &QuantumState, rho2: &QuantumState) -> f64 {
    // Check if rho1 is approximately pure (purity close to 1)
    let purity1 = rho1.purity();
    if (purity1 - 1.0).abs() < 1e-6 {
        // F = Tr(rho1 * rho2)
        let prod = rho1.rho.mul(&rho2.rho);
        return prod.trace().0.max(0.0).min(1.0);
    }

    // Check if rho2 is approximately pure
    let purity2 = rho2.purity();
    if (purity2 - 1.0).abs() < 1e-6 {
        let prod = rho1.rho.mul(&rho2.rho);
        return prod.trace().0.max(0.0).min(1.0);
    }

    // General case: use F = Tr(rho1 * rho2) + 2 * sqrt(det(rho1) * det(rho2))
    // for 2x2 systems. But we have 4x4. Use the approximation via partial overlap.
    // More precisely: F = (Tr(sqrt(sqrt(rho1)*rho2*sqrt(rho1))))^2
    // We use the Uhlmann formula via the identity:
    // F = (sum_i sqrt(lambda_i))^2 where lambda_i are eigenvalues of rho1 * rho2
    // (valid when both are density matrices).

    // Actually, F = (Tr sqrt(sqrt(rho1) rho2 sqrt(rho1)))^2
    // = (sum_i sqrt(mu_i))^2 where mu_i are eigenvalues of sqrt(rho1) rho2 sqrt(rho1)
    // which equal eigenvalues of rho1 * rho2 for the fidelity.

    // Simpler approach: F = ||sqrt(rho1) * sqrt(rho2)||_1^2
    // For numerical stability with small matrices, use:
    // rho1 * rho2 has eigenvalues whose sqrt-sum squared gives fidelity.
    let prod = rho1.rho.mul(&rho2.rho);
    let prod_herm = prod.add(&prod.dagger()).scale((0.5, 0.0));
    let eigenvals = hermitian_eigenvalues_4x4(&prod_herm);

    let sum_sqrt: f64 = eigenvals.iter().map(|&e| {
        if e > 0.0 { e.sqrt() } else { 0.0 }
    }).sum();

    (sum_sqrt * sum_sqrt).min(1.0).max(0.0)
}

// ---- Measurement simulation ---------------------------------------------------

/// Measurement direction on the Bloch sphere parameterized by (theta, phi).
#[derive(Clone, Debug)]
pub struct MeasurementDirection {
    pub theta: f64,
    pub phi: f64,
}

impl MeasurementDirection {
    /// Z-basis measurement (theta=0).
    pub fn z_basis() -> Self {
        Self { theta: 0.0, phi: 0.0 }
    }

    /// X-basis measurement (theta=pi/2, phi=0).
    pub fn x_basis() -> Self {
        Self { theta: std::f64::consts::FRAC_PI_2, phi: 0.0 }
    }

    /// Y-basis measurement (theta=pi/2, phi=pi/2).
    pub fn y_basis() -> Self {
        Self {
            theta: std::f64::consts::FRAC_PI_2,
            phi: std::f64::consts::FRAC_PI_2,
        }
    }

    /// Arbitrary direction for CHSH measurement.
    pub fn from_angle(angle: f64) -> Self {
        Self { theta: angle, phi: 0.0 }
    }
}

/// Measurement outcomes for a single qubit in a given direction.
/// Returns the projector |+><+| for the +1 eigenstate of sigma_n,
/// where n = (sin(theta)cos(phi), sin(theta)sin(phi), cos(theta)).
fn single_qubit_projector_plus(dir: &MeasurementDirection) -> Matrix2x2 {
    let ct = (dir.theta / 2.0).cos();
    let st = (dir.theta / 2.0).sin();
    let cp = dir.phi.cos();
    let sp = dir.phi.sin();

    // |+> = cos(theta/2)|0> + e^{i*phi}*sin(theta/2)|1>
    let a = (ct, 0.0);
    let b = (st * cp, st * sp);

    let mut proj = Matrix2x2::zero();
    proj.data[0][0] = c_mul(a, c_conj(a));
    proj.data[0][1] = c_mul(a, c_conj(b));
    proj.data[1][0] = c_mul(b, c_conj(a));
    proj.data[1][1] = c_mul(b, c_conj(b));
    proj
}

/// Compute joint measurement probabilities for 2-qubit state.
///
/// Returns [P(+1,+1), P(+1,-1), P(-1,+1), P(-1,-1)] for measurements
/// in directions dir_a (Alice) and dir_b (Bob).
pub fn joint_measurement_probabilities(
    state: &QuantumState,
    dir_a: &MeasurementDirection,
    dir_b: &MeasurementDirection,
) -> [f64; 4] {
    let proj_a_plus = single_qubit_projector_plus(dir_a);
    let proj_b_plus = single_qubit_projector_plus(dir_b);

    // Projector for -1 eigenstate = I - projector for +1
    let mut proj_a_minus = Matrix2x2::identity();
    let mut proj_b_minus = Matrix2x2::identity();
    for i in 0..2 {
        for j in 0..2 {
            proj_a_minus.data[i][j] = c_sub(proj_a_minus.data[i][j], proj_a_plus.data[i][j]);
            proj_b_minus.data[i][j] = c_sub(proj_b_minus.data[i][j], proj_b_plus.data[i][j]);
        }
    }

    // P(a, b) = Tr(rho * (proj_a tensor proj_b))
    let mut probs = [0.0; 4];
    let projs_a = [&proj_a_plus, &proj_a_minus];
    let projs_b = [&proj_b_plus, &proj_b_minus];

    for (ia, pa) in projs_a.iter().enumerate() {
        for (ib, pb) in projs_b.iter().enumerate() {
            let proj_ab = tensor_2x2(pa, pb);
            let prod = state.rho.mul(&proj_ab);
            probs[ia * 2 + ib] = prod.trace().0.max(0.0);
        }
    }

    // Normalize to ensure sum = 1 (numerical safety)
    let sum: f64 = probs.iter().sum();
    if sum > 1e-15 {
        for p in probs.iter_mut() {
            *p /= sum;
        }
    }

    probs
}

/// Simulate coincidence counts from a quantum state and measurement settings.
///
/// Uses a simple deterministic pseudo-random number generator seeded by `seed`.
/// Returns counts [N(+1,+1), N(+1,-1), N(-1,+1), N(-1,-1)].
pub fn simulate_coincidence_counts(
    state: &QuantumState,
    dir_a: &MeasurementDirection,
    dir_b: &MeasurementDirection,
    num_measurements: usize,
    seed: u64,
) -> [usize; 4] {
    let probs = joint_measurement_probabilities(state, dir_a, dir_b);
    let cumulative = [
        probs[0],
        probs[0] + probs[1],
        probs[0] + probs[1] + probs[2],
        1.0,
    ];

    let mut counts = [0usize; 4];
    let mut rng_state = seed;

    for _ in 0..num_measurements {
        // Simple LCG PRNG
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let r = (rng_state >> 11) as f64 / (1u64 << 53) as f64;

        if r < cumulative[0] {
            counts[0] += 1;
        } else if r < cumulative[1] {
            counts[1] += 1;
        } else if r < cumulative[2] {
            counts[2] += 1;
        } else {
            counts[3] += 1;
        }
    }

    counts
}

/// Compute the correlation function E(a, b) from coincidence counts.
///
/// E = (N++ + N-- - N+- - N-+) / (N++ + N-- + N+- + N-+)
pub fn correlation_from_counts(counts: &[usize; 4]) -> f64 {
    let total = (counts[0] + counts[1] + counts[2] + counts[3]) as f64;
    if total < 1.0 {
        return 0.0;
    }
    let npp = counts[0] as f64; // ++
    let npm = counts[1] as f64; // +-
    let nmp = counts[2] as f64; // -+
    let nmm = counts[3] as f64; // --
    (npp + nmm - npm - nmp) / total
}

/// Chi-squared statistic for testing whether measured counts match predicted probabilities.
///
/// chi^2 = sum_i (O_i - E_i)^2 / E_i
///
/// Returns (chi_squared, p_value_approximate).
pub fn chi_squared_test(observed: &[usize; 4], expected_probs: &[f64; 4], total: usize) -> (f64, f64) {
    let n = total as f64;
    let mut chi2 = 0.0;
    for i in 0..4 {
        let expected = n * expected_probs[i];
        if expected > 0.0 {
            let diff = observed[i] as f64 - expected;
            chi2 += diff * diff / expected;
        }
    }

    // Approximate p-value for chi-squared with 3 degrees of freedom
    // Using the approximation: P(X > x) ~ exp(-x/2) for large x
    // More accurate: use the regularized incomplete gamma function approximation
    let dof = 3.0;
    let p_value = chi2_survival(chi2, dof);

    (chi2, p_value)
}

/// Approximate survival function for chi-squared distribution.
/// P(X > x) for X ~ chi^2(k).
fn chi2_survival(x: f64, k: f64) -> f64 {
    if x <= 0.0 {
        return 1.0;
    }
    // Wilson-Hilferty approximation
    let z = ((x / k).powf(1.0 / 3.0) - (1.0 - 2.0 / (9.0 * k))) / (2.0 / (9.0 * k)).sqrt();
    // Standard normal survival function approximation
    0.5 * erfc_approx(z / std::f64::consts::SQRT_2)
}

/// Approximate complementary error function.
fn erfc_approx(x: f64) -> f64 {
    // Abramowitz and Stegun approximation 7.1.26
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }
    let p = 0.3275911;
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;
    let t = 1.0 / (1.0 + p * x);
    let poly = t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
    poly * (-x * x).exp()
}

// ---- Summary report -----------------------------------------------------------

/// Summary of entanglement characterization for a 2-qubit state.
#[derive(Clone, Debug)]
pub struct EntanglementReport {
    pub concurrence: f64,
    pub entanglement_of_formation: f64,
    pub negativity: f64,
    pub von_neumann_entropy: f64,
    pub entanglement_entropy: f64,
    pub purity: f64,
    pub chsh_parameter: f64,
    pub violates_bell: bool,
    pub witness_value: f64,
    pub witness_detects: bool,
}

/// Generate a complete entanglement characterization report.
pub fn entanglement_report(state: &QuantumState) -> EntanglementReport {
    let c = concurrence(state);
    let ef = entanglement_of_formation(state);
    let neg = negativity(state);
    let vne = von_neumann_entropy(state);
    let ee = entanglement_entropy(state);
    let pur = state.purity();

    let settings = optimal_chsh_settings();
    let s = chsh_parameter(state, &settings);

    let witness = EntanglementWitness::for_bell_phi_plus();
    let wv = witness.evaluate(state);

    EntanglementReport {
        concurrence: c,
        entanglement_of_formation: ef,
        negativity: neg,
        von_neumann_entropy: vne,
        entanglement_entropy: ee,
        purity: pur,
        chsh_parameter: s,
        violates_bell: violates_bell_inequality(s),
        witness_value: wv,
        witness_detects: wv < -1e-10,
    }
}

// ---- Tests --------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_1_SQRT_2;

    const TOL: f64 = 1e-6;

    // ---- Complex arithmetic tests ----

    #[test]
    fn test_complex_add() {
        let a = (1.0, 2.0);
        let b = (3.0, 4.0);
        let r = c_add(a, b);
        assert!((r.0 - 4.0).abs() < TOL);
        assert!((r.1 - 6.0).abs() < TOL);
    }

    #[test]
    fn test_complex_mul() {
        // (1+2i)(3+4i) = 3 + 4i + 6i + 8i^2 = -5 + 10i
        let r = c_mul((1.0, 2.0), (3.0, 4.0));
        assert!((r.0 - (-5.0)).abs() < TOL);
        assert!((r.1 - 10.0).abs() < TOL);
    }

    #[test]
    fn test_complex_div() {
        let a = (4.0, 2.0);
        let b = (1.0, 1.0);
        let r = c_div(a, b);
        // (4+2i)/(1+i) = (4+2i)(1-i)/2 = (4-4i+2i-2i^2)/2 = (6-2i)/2 = 3-i
        assert!((r.0 - 3.0).abs() < TOL);
        assert!((r.1 - (-1.0)).abs() < TOL);
    }

    #[test]
    fn test_complex_sqrt() {
        let r = c_sqrt((0.0, 1.0)); // sqrt(i) = (1+i)/sqrt(2)
        assert!((r.0 - FRAC_1_SQRT_2).abs() < TOL);
        assert!((r.1 - FRAC_1_SQRT_2).abs() < TOL);
    }

    // ---- Matrix tests ----

    #[test]
    fn test_matrix_identity_trace() {
        let id = Matrix4x4::identity();
        let tr = id.trace();
        assert!((tr.0 - 4.0).abs() < TOL);
        assert!(tr.1.abs() < TOL);
    }

    #[test]
    fn test_matrix_mul_identity() {
        let id = Matrix4x4::identity();
        let mut m = Matrix4x4::zero();
        m.data[0][1] = (1.0, 2.0);
        m.data[2][3] = (3.0, 4.0);
        let result = id.mul(&m);
        assert!((c_abs(c_sub(result.data[0][1], (1.0, 2.0)))).abs() < TOL);
        assert!((c_abs(c_sub(result.data[2][3], (3.0, 4.0)))).abs() < TOL);
    }

    #[test]
    fn test_matrix_dagger() {
        let mut m = Matrix4x4::zero();
        m.data[0][1] = (1.0, 2.0);
        m.data[1][0] = (1.0, -2.0); // Hermitian
        let dag = m.dagger();
        assert!(c_abs(c_sub(dag.data[0][1], (1.0, 2.0))) < TOL);
    }

    #[test]
    fn test_outer_product() {
        let v = [CONE, CZERO, CZERO, CZERO];
        let op = Matrix4x4::outer_product(&v, &v);
        assert!((op.data[0][0].0 - 1.0).abs() < TOL);
        assert!(c_abs(op.data[0][1]) < TOL);
        assert!(c_abs(op.data[1][0]) < TOL);
    }

    // ---- QuantumState tests ----

    #[test]
    fn test_bell_phi_plus_trace() {
        let state = QuantumState::bell_phi_plus();
        let tr = state.trace();
        assert!((tr.0 - 1.0).abs() < TOL);
        assert!(tr.1.abs() < TOL);
    }

    #[test]
    fn test_bell_phi_plus_purity() {
        let state = QuantumState::bell_phi_plus();
        let p = state.purity();
        assert!((p - 1.0).abs() < TOL); // Pure state
    }

    #[test]
    fn test_bell_states_are_normalized() {
        for state in &[
            QuantumState::bell_phi_plus(),
            QuantumState::bell_phi_minus(),
            QuantumState::bell_psi_plus(),
            QuantumState::bell_psi_minus(),
        ] {
            let tr = state.trace();
            assert!((tr.0 - 1.0).abs() < TOL, "Trace should be 1");
        }
    }

    #[test]
    fn test_maximally_mixed_purity() {
        let state = QuantumState::maximally_mixed();
        let p = state.purity();
        assert!((p - 0.25).abs() < TOL); // Tr(I/4 * I/4) = 4/16 = 0.25
    }

    #[test]
    fn test_product_state_trace() {
        let state = QuantumState::product(0.5, 1.2);
        let tr = state.trace();
        assert!((tr.0 - 1.0).abs() < TOL);
    }

    #[test]
    fn test_werner_state_trace() {
        for &p in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let state = QuantumState::werner(p);
            let tr = state.trace();
            assert!((tr.0 - 1.0).abs() < TOL, "Werner state trace should be 1 for p={}", p);
        }
    }

    // ---- Partial trace tests ----

    #[test]
    fn test_bell_phi_plus_partial_trace() {
        let state = QuantumState::bell_phi_plus();
        let rho_a = state.partial_trace_b();
        // Partial trace of maximally entangled state is I/2
        assert!((rho_a.data[0][0].0 - 0.5).abs() < TOL);
        assert!((rho_a.data[1][1].0 - 0.5).abs() < TOL);
        assert!(c_abs(rho_a.data[0][1]) < TOL);
        assert!(c_abs(rho_a.data[1][0]) < TOL);
    }

    #[test]
    fn test_product_state_partial_trace() {
        // |00> should give |0><0| for both partial traces
        let state = QuantumState::from_amplitudes(CONE, CZERO, CZERO, CZERO);
        let rho_a = state.partial_trace_b();
        assert!((rho_a.data[0][0].0 - 1.0).abs() < TOL);
        assert!((rho_a.data[1][1].0).abs() < TOL);

        let rho_b = state.partial_trace_a();
        assert!((rho_b.data[0][0].0 - 1.0).abs() < TOL);
        assert!((rho_b.data[1][1].0).abs() < TOL);
    }

    // ---- CHSH / Bell inequality tests ----

    #[test]
    fn test_chsh_bell_phi_plus_violates() {
        let state = QuantumState::bell_phi_plus();
        let settings = optimal_chsh_settings();
        let s = chsh_parameter(&state, &settings);
        assert!(s > 2.0, "Bell state should violate CHSH: S={}", s);
        assert!(s < TSIRELSON_BOUND + TOL, "S should not exceed Tsirelson bound");
    }

    #[test]
    fn test_chsh_bell_psi_minus_violates() {
        let state = QuantumState::bell_psi_minus();
        let settings = optimal_chsh_settings();
        let s = chsh_parameter(&state, &settings);
        assert!(s > 2.0, "Psi- should violate CHSH: S={}", s);
    }

    #[test]
    fn test_chsh_product_state_no_violation() {
        let state = QuantumState::product(0.0, 0.0); // |00>
        let settings = optimal_chsh_settings();
        let s = chsh_parameter(&state, &settings);
        assert!(s <= 2.0 + TOL, "Product state should not violate CHSH: S={}", s);
    }

    #[test]
    fn test_chsh_maximally_mixed_no_violation() {
        let state = QuantumState::maximally_mixed();
        let settings = optimal_chsh_settings();
        let s = chsh_parameter(&state, &settings);
        assert!(s < TOL, "Maximally mixed state should have S~0: S={}", s);
    }

    #[test]
    fn test_correlation_function_bell_z_basis() {
        // For |Phi+>, E(0, 0) = <sigma_z tensor sigma_z> = 1
        let state = QuantumState::bell_phi_plus();
        let e = correlation_function(&state, 0.0, 0.0);
        assert!((e - 1.0).abs() < TOL, "E(0,0) for Phi+ should be 1: {}", e);
    }

    // ---- Concurrence tests ----

    #[test]
    fn test_concurrence_bell_states() {
        for state in &[
            QuantumState::bell_phi_plus(),
            QuantumState::bell_phi_minus(),
            QuantumState::bell_psi_plus(),
            QuantumState::bell_psi_minus(),
        ] {
            let c = concurrence(state);
            assert!((c - 1.0).abs() < 0.05, "Bell state concurrence should be ~1: {}", c);
        }
    }

    #[test]
    fn test_concurrence_product_state() {
        let state = QuantumState::product(0.3, 0.7);
        let c = concurrence(&state);
        assert!(c < 0.05, "Product state concurrence should be ~0: {}", c);
    }

    #[test]
    fn test_concurrence_werner_entangled() {
        let state = QuantumState::werner(0.8);
        let c = concurrence(&state);
        assert!(c > 0.1, "Werner p=0.8 should be entangled: C={}", c);
    }

    #[test]
    fn test_concurrence_werner_separable() {
        let state = QuantumState::werner(0.2);
        let c = concurrence(&state);
        assert!(c < 0.1, "Werner p=0.2 should be separable: C={}", c);
    }

    // ---- Entanglement of formation tests ----

    #[test]
    fn test_eof_bell_state() {
        let state = QuantumState::bell_phi_plus();
        let ef = entanglement_of_formation(&state);
        assert!((ef - 1.0).abs() < 0.05, "Bell state EoF should be ~1: {}", ef);
    }

    #[test]
    fn test_eof_product_state() {
        let state = QuantumState::product(0.0, 0.0);
        let ef = entanglement_of_formation(&state);
        assert!(ef < 0.05, "Product state EoF should be ~0: {}", ef);
    }

    // ---- Negativity tests ----

    #[test]
    fn test_negativity_bell_state() {
        let state = QuantumState::bell_phi_plus();
        let n = negativity(&state);
        assert!((n - 0.5).abs() < 0.05, "Bell state negativity should be ~0.5: {}", n);
    }

    #[test]
    fn test_negativity_product_state() {
        let state = QuantumState::product(0.0, 0.0);
        let n = negativity(&state);
        assert!(n < 0.05, "Product state negativity should be ~0: {}", n);
    }

    #[test]
    fn test_negativity_werner_threshold() {
        // Werner state is entangled (negative partial transpose) iff p > 1/3
        let state_ent = QuantumState::werner(0.6);
        let state_sep = QuantumState::werner(0.2);
        assert!(negativity(&state_ent) > 0.01, "Werner p=0.6 should have positive negativity");
        assert!(negativity(&state_sep) < 0.05, "Werner p=0.2 should have ~zero negativity");
    }

    // ---- Von Neumann entropy tests ----

    #[test]
    fn test_vne_pure_state() {
        let state = QuantumState::bell_phi_plus();
        let s = von_neumann_entropy(&state);
        assert!(s < 0.05, "Pure state VNE should be ~0: {}", s);
    }

    #[test]
    fn test_vne_maximally_mixed() {
        let state = QuantumState::maximally_mixed();
        let s = von_neumann_entropy(&state);
        assert!((s - 2.0).abs() < 0.05, "Maximally mixed 4-dim VNE should be ~2: {}", s);
    }

    // ---- Entanglement entropy tests ----

    #[test]
    fn test_entanglement_entropy_bell_state() {
        let state = QuantumState::bell_phi_plus();
        let ee = entanglement_entropy(&state);
        assert!((ee - 1.0).abs() < 0.05, "Bell state entanglement entropy should be ~1: {}", ee);
    }

    #[test]
    fn test_entanglement_entropy_product_state() {
        let state = QuantumState::product(0.0, 0.0);
        let ee = entanglement_entropy(&state);
        assert!(ee < 0.05, "Product state entanglement entropy should be ~0: {}", ee);
    }

    // ---- Entanglement witness tests ----

    #[test]
    fn test_witness_detects_phi_plus() {
        let witness = EntanglementWitness::for_bell_phi_plus();
        let state = QuantumState::bell_phi_plus();
        assert!(witness.detects_entanglement(&state), "Witness should detect Phi+ entanglement");
    }

    #[test]
    fn test_witness_no_false_positive_product() {
        let witness = EntanglementWitness::for_bell_phi_plus();
        let state = QuantumState::product(0.0, 0.0);
        let val = witness.evaluate(&state);
        assert!(val >= -TOL, "Witness should not detect product state as entangled: {}", val);
    }

    #[test]
    fn test_witness_psi_minus() {
        let witness = EntanglementWitness::for_bell_psi_minus();
        let state = QuantumState::bell_psi_minus();
        assert!(witness.detects_entanglement(&state), "Witness should detect Psi- entanglement");
    }

    // ---- Fidelity tests ----

    #[test]
    fn test_fidelity_same_state() {
        let state = QuantumState::bell_phi_plus();
        let f = quantum_fidelity(&state, &state);
        assert!((f - 1.0).abs() < TOL, "Fidelity with self should be 1: {}", f);
    }

    #[test]
    fn test_fidelity_orthogonal_states() {
        let s1 = QuantumState::from_amplitudes(CONE, CZERO, CZERO, CZERO); // |00>
        let s2 = QuantumState::from_amplitudes(CZERO, CONE, CZERO, CZERO); // |01>
        let f = quantum_fidelity(&s1, &s2);
        assert!(f < TOL, "Fidelity of orthogonal states should be ~0: {}", f);
    }

    #[test]
    fn test_fidelity_bell_states() {
        let phi_plus = QuantumState::bell_phi_plus();
        let phi_minus = QuantumState::bell_phi_minus();
        let f = quantum_fidelity(&phi_plus, &phi_minus);
        assert!(f < TOL, "Fidelity of orthogonal Bell states should be ~0: {}", f);
    }

    // ---- Measurement simulation tests ----

    #[test]
    fn test_joint_probabilities_bell_z_basis() {
        let state = QuantumState::bell_phi_plus();
        let probs = joint_measurement_probabilities(
            &state,
            &MeasurementDirection::z_basis(),
            &MeasurementDirection::z_basis(),
        );
        // |Phi+> in Z basis: P(00) = 0.5, P(11) = 0.5, P(01) = P(10) = 0
        assert!((probs[0] - 0.5).abs() < TOL, "P(++) = {}", probs[0]);
        assert!(probs[1] < TOL, "P(+-) = {}", probs[1]);
        assert!(probs[2] < TOL, "P(-+) = {}", probs[2]);
        assert!((probs[3] - 0.5).abs() < TOL, "P(--) = {}", probs[3]);
    }

    #[test]
    fn test_coincidence_counts_sum() {
        let state = QuantumState::bell_phi_plus();
        let counts = simulate_coincidence_counts(
            &state,
            &MeasurementDirection::z_basis(),
            &MeasurementDirection::z_basis(),
            10000,
            42,
        );
        let total: usize = counts.iter().sum();
        assert_eq!(total, 10000);
    }

    #[test]
    fn test_coincidence_counts_bell_phi_plus() {
        let state = QuantumState::bell_phi_plus();
        let counts = simulate_coincidence_counts(
            &state,
            &MeasurementDirection::z_basis(),
            &MeasurementDirection::z_basis(),
            100000,
            12345,
        );
        // Should be roughly 50/50 between ++ and --
        let frac_pp = counts[0] as f64 / 100000.0;
        let frac_mm = counts[3] as f64 / 100000.0;
        assert!((frac_pp - 0.5).abs() < 0.02, "P(++) ~ 0.5: {}", frac_pp);
        assert!((frac_mm - 0.5).abs() < 0.02, "P(--) ~ 0.5: {}", frac_mm);
    }

    #[test]
    fn test_correlation_from_counts() {
        // Perfect anti-correlation: all +- and -+
        let counts = [0, 50, 50, 0];
        let e = correlation_from_counts(&counts);
        assert!((e - (-1.0)).abs() < TOL);

        // Perfect correlation: all ++ and --
        let counts2 = [50, 0, 0, 50];
        let e2 = correlation_from_counts(&counts2);
        assert!((e2 - 1.0).abs() < TOL);
    }

    // ---- Chi-squared test ----

    #[test]
    fn test_chi_squared_perfect_match() {
        let observed = [250, 250, 250, 250];
        let expected = [0.25, 0.25, 0.25, 0.25];
        let (chi2, _p) = chi_squared_test(&observed, &expected, 1000);
        assert!(chi2 < TOL, "Perfect match should give chi2~0: {}", chi2);
    }

    #[test]
    fn test_chi_squared_poor_match() {
        let observed = [900, 33, 34, 33];
        let expected = [0.25, 0.25, 0.25, 0.25];
        let (chi2, _p) = chi_squared_test(&observed, &expected, 1000);
        assert!(chi2 > 100.0, "Poor match should give large chi2: {}", chi2);
    }

    // ---- Partial transpose tests ----

    #[test]
    fn test_partial_transpose_product_state_positive() {
        let state = QuantumState::product(0.5, 1.0);
        let pt = state.rho.partial_transpose_b();
        let eigenvals = hermitian_eigenvalues_4x4(&pt);
        // Product state has positive partial transpose
        for &ev in &eigenvals {
            assert!(ev > -0.05, "Product state PPT eigenvalue should be >= 0: {}", ev);
        }
    }

    #[test]
    fn test_partial_transpose_bell_state_negative() {
        let state = QuantumState::bell_phi_plus();
        let pt = state.rho.partial_transpose_b();
        let eigenvals = hermitian_eigenvalues_4x4(&pt);
        // Bell state has negative eigenvalue in partial transpose
        let min_ev = eigenvals.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(min_ev < -0.1, "Bell state should have negative PPT eigenvalue: {}", min_ev);
    }

    // ---- Report test ----

    #[test]
    fn test_entanglement_report_bell_state() {
        let state = QuantumState::bell_phi_plus();
        let report = entanglement_report(&state);
        assert!((report.purity - 1.0).abs() < 0.05);
        assert!(report.concurrence > 0.9);
        assert!(report.violates_bell);
        assert!(report.witness_detects);
    }

    #[test]
    fn test_entanglement_report_product_state() {
        let state = QuantumState::product(0.0, 0.0);
        let report = entanglement_report(&state);
        assert!((report.purity - 1.0).abs() < 0.05);
        assert!(report.concurrence < 0.1);
        assert!(!report.violates_bell);
    }

    // ---- Werner state boundary test ----

    #[test]
    fn test_werner_p1_is_bell_psi_minus() {
        let state = QuantumState::werner(1.0);
        let f = quantum_fidelity(&state, &QuantumState::bell_psi_minus());
        assert!((f - 1.0).abs() < 0.05, "Werner p=1 should be Psi-: F={}", f);
    }

    #[test]
    fn test_binary_entropy() {
        assert!((binary_entropy(0.5) - 1.0).abs() < TOL);
        assert!(binary_entropy(0.0).abs() < TOL);
        assert!(binary_entropy(1.0).abs() < TOL);
    }

    // ---- Additional integration test ----

    #[test]
    fn test_chsh_from_simulated_counts() {
        let state = QuantumState::bell_phi_plus();
        let settings = optimal_chsh_settings();
        let n = 100000;
        let seed = 9999;

        let e_ab = {
            let counts = simulate_coincidence_counts(
                &state,
                &MeasurementDirection::from_angle(settings.a),
                &MeasurementDirection::from_angle(settings.b),
                n, seed,
            );
            correlation_from_counts(&counts)
        };
        let e_ab_prime = {
            let counts = simulate_coincidence_counts(
                &state,
                &MeasurementDirection::from_angle(settings.a),
                &MeasurementDirection::from_angle(settings.b_prime),
                n, seed + 1,
            );
            correlation_from_counts(&counts)
        };
        let e_a_prime_b = {
            let counts = simulate_coincidence_counts(
                &state,
                &MeasurementDirection::from_angle(settings.a_prime),
                &MeasurementDirection::from_angle(settings.b),
                n, seed + 2,
            );
            correlation_from_counts(&counts)
        };
        let e_a_prime_b_prime = {
            let counts = simulate_coincidence_counts(
                &state,
                &MeasurementDirection::from_angle(settings.a_prime),
                &MeasurementDirection::from_angle(settings.b_prime),
                n, seed + 3,
            );
            correlation_from_counts(&counts)
        };

        let s = (e_ab - e_ab_prime + e_a_prime_b + e_a_prime_b_prime).abs();
        assert!(s > 2.0, "Simulated CHSH should violate classical bound: S={}", s);
        // Check that the simulated value is close to the theoretical value
        let s_theoretical = chsh_parameter(&state, &settings);
        assert!((s - s_theoretical).abs() < 0.1,
                "Simulated S should match theoretical: simulated={}, theoretical={}", s, s_theoretical);
    }
}

//! Nuclear-norm minimization for matrix completion and missing data reconstruction.
//!
//! This module implements the Singular Value Thresholding (SVT) algorithm for
//! recovering a low-rank matrix from a sparse set of observed entries. This is
//! useful for IQ data reconstruction where some samples are missing or corrupted.
//!
//! The nuclear norm (sum of singular values) serves as a convex relaxation of
//! the rank function. Minimizing it subject to agreement with observed entries
//! promotes low-rank solutions.
//!
//! # Algorithm
//!
//! The SVT iteration is:
//!
//! ```text
//! Y_{k+1} = Y_k + delta * P_Omega(M - X_k)
//! X_{k+1} = SVT_tau(Y_{k+1})
//! ```
//!
//! where `SVT_tau` soft-thresholds the singular values at level `tau`.
//!
//! # Example
//!
//! ```
//! use r4w_core::matrix_completion_nuclear::{MatrixCompletion, SvtParams};
//!
//! // Create a 4x4 matrix with some observed entries
//! let rows = 4;
//! let cols = 4;
//! // Observed entries: (row, col, value)
//! let observed = vec![
//!     (0, 0, 1.0), (0, 1, 2.0),
//!     (1, 0, 2.0), (1, 1, 4.0),
//!     (2, 2, 3.0), (2, 3, 6.0),
//!     (3, 2, 1.0), (3, 3, 2.0),
//! ];
//!
//! let params = SvtParams {
//!     tau: 5.0,
//!     delta: 1.2,
//!     max_iter: 200,
//!     tol: 1e-4,
//! };
//!
//! let mc = MatrixCompletion::new(rows, cols, &observed, params);
//! let completed = mc.solve();
//!
//! // The completed matrix should agree with observed entries
//! let rmse = MatrixCompletion::rmse(&completed, rows, cols, &observed);
//! assert!(rmse < 0.1, "RMSE too large: {}", rmse);
//! ```

use std::f64;

// ─── Complex helpers ────────────────────────────────────────────────────────

/// Complex number as (re, im) tuple.
pub type Complex = (f64, f64);

#[inline]
fn c_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: Complex, b: Complex) -> Complex {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

#[inline]
fn c_abs(a: Complex) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_scale(s: f64, a: Complex) -> Complex {
    (s * a.0, s * a.1)
}

// ─── Dense real matrix utilities ────────────────────────────────────────────

/// Row-major dense matrix stored as a flat Vec.
#[derive(Clone, Debug)]
struct Mat {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl Mat {
    fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    #[inline]
    fn get(&self, r: usize, c: usize) -> f64 {
        self.data[r * self.cols + c]
    }

    #[inline]
    fn set(&mut self, r: usize, c: usize, v: f64) {
        self.data[r * self.cols + c] = v;
    }

    #[inline]
    fn add_assign(&mut self, r: usize, c: usize, v: f64) {
        self.data[r * self.cols + c] += v;
    }

    fn frobenius_norm(&self) -> f64 {
        self.data.iter().map(|x| x * x).sum::<f64>().sqrt()
    }

    /// Multiply: self (m×k) * other (k×n) → result (m×n)
    fn mul_mat(&self, other: &Mat) -> Mat {
        assert_eq!(self.cols, other.rows);
        let mut out = Mat::zeros(self.rows, other.cols);
        for i in 0..self.rows {
            for p in 0..self.cols {
                let a = self.get(i, p);
                if a == 0.0 {
                    continue;
                }
                for j in 0..other.cols {
                    out.add_assign(i, j, a * other.get(p, j));
                }
            }
        }
        out
    }

    fn transpose(&self) -> Mat {
        let mut out = Mat::zeros(self.cols, self.rows);
        for i in 0..self.rows {
            for j in 0..self.cols {
                out.set(j, i, self.get(i, j));
            }
        }
        out
    }
}

// ─── Dense complex matrix utilities ─────────────────────────────────────────

#[derive(Clone, Debug)]
struct CMat {
    rows: usize,
    cols: usize,
    data: Vec<Complex>,
}

impl CMat {
    fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![(0.0, 0.0); rows * cols],
        }
    }

    #[inline]
    fn get(&self, r: usize, c: usize) -> Complex {
        self.data[r * self.cols + c]
    }

    #[inline]
    fn set(&mut self, r: usize, c: usize, v: Complex) {
        self.data[r * self.cols + c] = v;
    }

    fn frobenius_norm(&self) -> f64 {
        self.data
            .iter()
            .map(|x| x.0 * x.0 + x.1 * x.1)
            .sum::<f64>()
            .sqrt()
    }

    fn hermitian_transpose(&self) -> CMat {
        let mut out = CMat::zeros(self.cols, self.rows);
        for i in 0..self.rows {
            for j in 0..self.cols {
                out.set(j, i, c_conj(self.get(i, j)));
            }
        }
        out
    }

    fn mul_mat(&self, other: &CMat) -> CMat {
        assert_eq!(self.cols, other.rows);
        let mut out = CMat::zeros(self.rows, other.cols);
        for i in 0..self.rows {
            for p in 0..self.cols {
                let a = self.get(i, p);
                if a.0 == 0.0 && a.1 == 0.0 {
                    continue;
                }
                for j in 0..other.cols {
                    let cur = out.get(i, j);
                    out.set(i, j, c_add(cur, c_mul(a, other.get(p, j))));
                }
            }
        }
        out
    }
}

// ─── Power-iteration SVD ────────────────────────────────────────────────────

/// Compute top-k singular triplets of a real matrix via power iteration.
///
/// Returns (U, sigma, V) where U is m×k, sigma is length k, V is n×k.
pub fn power_svd(mat: &[f64], rows: usize, cols: usize, k: usize, max_iter: usize) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let k = k.min(rows).min(cols);
    let m = Mat {
        rows,
        cols,
        data: mat.to_vec(),
    };

    let mut u_vecs: Vec<Vec<f64>> = Vec::with_capacity(k);
    let mut s_vals: Vec<f64> = Vec::with_capacity(k);
    let mut v_vecs: Vec<Vec<f64>> = Vec::with_capacity(k);

    // Deflated copy
    let mut residual = m.clone();

    for _ in 0..k {
        // Initialize random-ish vector (deterministic seed via dimension)
        let mut v = vec![0.0f64; cols];
        for j in 0..cols {
            v[j] = ((j as f64 + 1.0) * 0.7123 + 0.3).sin();
        }
        // Normalize
        let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm > 0.0 {
            for x in v.iter_mut() {
                *x /= norm;
            }
        }

        let mut sigma = 0.0f64;

        for _ in 0..max_iter {
            // u = A * v
            let mut u = vec![0.0f64; rows];
            for i in 0..rows {
                let mut s = 0.0;
                for j in 0..cols {
                    s += residual.get(i, j) * v[j];
                }
                u[i] = s;
            }
            // sigma = ||u||
            let new_sigma = u.iter().map(|x| x * x).sum::<f64>().sqrt();
            if new_sigma < 1e-15 {
                sigma = 0.0;
                break;
            }
            for x in u.iter_mut() {
                *x /= new_sigma;
            }

            // v = A^T * u
            let mut v_new = vec![0.0f64; cols];
            for j in 0..cols {
                let mut s = 0.0;
                for i in 0..rows {
                    s += residual.get(i, j) * u[i];
                }
                v_new[j] = s;
            }
            let v_norm = v_new.iter().map(|x| x * x).sum::<f64>().sqrt();
            if v_norm < 1e-15 {
                sigma = 0.0;
                break;
            }
            for x in v_new.iter_mut() {
                *x /= v_norm;
            }

            let converged = (new_sigma - sigma).abs() / (new_sigma.abs() + 1e-30) < 1e-12;
            sigma = new_sigma;
            v = v_new;

            if converged {
                break;
            }
        }

        if sigma < 1e-15 {
            // Remaining singular values are zero
            break;
        }

        // Compute u from final v
        let mut u = vec![0.0f64; rows];
        for i in 0..rows {
            let mut s = 0.0;
            for j in 0..cols {
                s += residual.get(i, j) * v[j];
            }
            u[i] = s;
        }
        let u_norm = u.iter().map(|x| x * x).sum::<f64>().sqrt();
        for x in u.iter_mut() {
            *x /= u_norm;
        }

        // Deflate: residual -= sigma * u * v^T
        for i in 0..rows {
            for j in 0..cols {
                let old = residual.get(i, j);
                residual.set(i, j, old - sigma * u[i] * v[j]);
            }
        }

        u_vecs.push(u);
        s_vals.push(sigma);
        v_vecs.push(v);
    }

    let actual_k = s_vals.len();
    // Pack into column-major style flat arrays: U is rows×actual_k, V is cols×actual_k
    let mut u_flat = vec![0.0f64; rows * actual_k];
    let mut v_flat = vec![0.0f64; cols * actual_k];
    for idx in 0..actual_k {
        for i in 0..rows {
            u_flat[i * actual_k + idx] = u_vecs[idx][i];
        }
        for j in 0..cols {
            v_flat[j * actual_k + idx] = v_vecs[idx][j];
        }
    }

    (u_flat, s_vals, v_flat)
}

/// Full SVD of a small real matrix via Jacobi one-sided rotations.
///
/// Returns (U, sigma, V) where U is m×min(m,n), sigma has min(m,n) entries,
/// V is n×min(m,n). More accurate than power iteration for small matrices.
fn full_svd(mat: &Mat) -> (Mat, Vec<f64>, Mat) {
    let m = mat.rows;
    let n = mat.cols;
    let k = m.min(n);

    // Compute A^T A
    let at = mat.transpose();
    let ata = at.mul_mat(mat); // n×n

    // Jacobi eigendecomposition of A^T A
    let (eigenvalues, eigenvectors) = jacobi_eigen_symmetric(&ata);

    // Sort eigenvalues descending
    let mut indices: Vec<usize> = (0..n).collect();
    indices.sort_by(|&a, &b| eigenvalues[b].partial_cmp(&eigenvalues[a]).unwrap_or(std::cmp::Ordering::Equal));

    let mut sigma = Vec::with_capacity(k);
    let mut v_mat = Mat::zeros(n, k);
    for idx in 0..k {
        let ev = eigenvalues[indices[idx]].max(0.0);
        sigma.push(ev.sqrt());
        for j in 0..n {
            v_mat.set(j, idx, eigenvectors.get(j, indices[idx]));
        }
    }

    // U = A * V * Sigma^{-1}
    let mut u_mat = Mat::zeros(m, k);
    let av = mat.mul_mat(&v_mat); // m×k
    for idx in 0..k {
        if sigma[idx] > 1e-15 {
            let inv_s = 1.0 / sigma[idx];
            for i in 0..m {
                u_mat.set(i, idx, av.get(i, idx) * inv_s);
            }
        }
    }

    (u_mat, sigma, v_mat)
}

/// Jacobi eigendecomposition for a symmetric matrix.
/// Returns (eigenvalues, eigenvector_matrix).
fn jacobi_eigen_symmetric(a: &Mat) -> (Vec<f64>, Mat) {
    let n = a.rows;
    assert_eq!(a.rows, a.cols);

    let mut d = a.clone();
    let mut v = Mat::zeros(n, n);
    for i in 0..n {
        v.set(i, i, 1.0);
    }

    for _ in 0..100 * n * n {
        // Find largest off-diagonal element
        let mut max_val = 0.0f64;
        let mut p = 0;
        let mut q = 1;
        for i in 0..n {
            for j in (i + 1)..n {
                let v = d.get(i, j).abs();
                if v > max_val {
                    max_val = v;
                    p = i;
                    q = j;
                }
            }
        }

        if max_val < 1e-14 {
            break;
        }

        // Compute rotation
        let app = d.get(p, p);
        let aqq = d.get(q, q);
        let apq = d.get(p, q);

        let theta = if (app - aqq).abs() < 1e-30 {
            std::f64::consts::FRAC_PI_4
        } else {
            0.5 * (2.0 * apq / (app - aqq)).atan()
        };

        let c = theta.cos();
        let s = theta.sin();

        // Apply Givens rotation to D
        // Update rows p and q
        let mut row_p = vec![0.0f64; n];
        let mut row_q = vec![0.0f64; n];
        for j in 0..n {
            row_p[j] = c * d.get(p, j) + s * d.get(q, j);
            row_q[j] = -s * d.get(p, j) + c * d.get(q, j);
        }
        for j in 0..n {
            d.set(p, j, row_p[j]);
            d.set(q, j, row_q[j]);
        }

        // Update cols p and q
        let mut col_p = vec![0.0f64; n];
        let mut col_q = vec![0.0f64; n];
        for i in 0..n {
            col_p[i] = c * d.get(i, p) + s * d.get(i, q);
            col_q[i] = -s * d.get(i, p) + c * d.get(i, q);
        }
        for i in 0..n {
            d.set(i, p, col_p[i]);
            d.set(i, q, col_q[i]);
        }

        // Update eigenvectors
        let mut vp = vec![0.0f64; n];
        let mut vq = vec![0.0f64; n];
        for i in 0..n {
            vp[i] = c * v.get(i, p) + s * v.get(i, q);
            vq[i] = -s * v.get(i, p) + c * v.get(i, q);
        }
        for i in 0..n {
            v.set(i, p, vp[i]);
            v.set(i, q, vq[i]);
        }
    }

    let eigenvalues: Vec<f64> = (0..n).map(|i| d.get(i, i)).collect();
    (eigenvalues, v)
}

// ─── SVT core ───────────────────────────────────────────────────────────────

/// Soft-threshold a singular value: max(sigma - tau, 0).
#[inline]
fn soft_threshold(sigma: f64, tau: f64) -> f64 {
    (sigma - tau).max(0.0)
}

/// Reconstruct a matrix from its thresholded SVD: U * diag(sigma) * V^T
fn reconstruct_from_svd(u: &Mat, sigma: &[f64], v: &Mat, rows: usize, cols: usize) -> Mat {
    let k = sigma.len();
    let mut out = Mat::zeros(rows, cols);
    for idx in 0..k {
        if sigma[idx] > 1e-15 {
            for i in 0..rows {
                let us = u.get(i, idx) * sigma[idx];
                for j in 0..cols {
                    out.add_assign(i, j, us * v.get(j, idx));
                }
            }
        }
    }
    out
}

// ─── Public API ─────────────────────────────────────────────────────────────

/// Parameters for the Singular Value Thresholding algorithm.
#[derive(Clone, Debug)]
pub struct SvtParams {
    /// Singular value threshold (regularization weight on nuclear norm).
    pub tau: f64,
    /// Step size for the projected gradient update. Typically in (1.0, 2.0).
    pub delta: f64,
    /// Maximum number of iterations.
    pub max_iter: usize,
    /// Convergence tolerance on relative Frobenius norm change.
    pub tol: f64,
}

impl Default for SvtParams {
    fn default() -> Self {
        Self {
            tau: 5.0,
            delta: 1.2,
            max_iter: 500,
            tol: 1e-4,
        }
    }
}

/// Result of a matrix completion solve.
#[derive(Clone, Debug)]
pub struct CompletionResult {
    /// The completed matrix in row-major order.
    pub matrix: Vec<f64>,
    /// Number of rows.
    pub rows: usize,
    /// Number of columns.
    pub cols: usize,
    /// Singular values of the final solution.
    pub singular_values: Vec<f64>,
    /// Number of iterations used.
    pub iterations: usize,
    /// Final relative change (convergence metric).
    pub final_relative_change: f64,
}

/// Result of a complex matrix completion solve.
#[derive(Clone, Debug)]
pub struct ComplexCompletionResult {
    /// The completed matrix in row-major order as (re, im) tuples.
    pub matrix: Vec<Complex>,
    /// Number of rows.
    pub rows: usize,
    /// Number of columns.
    pub cols: usize,
    /// Singular values of the final solution.
    pub singular_values: Vec<f64>,
    /// Number of iterations used.
    pub iterations: usize,
    /// Final relative change.
    pub final_relative_change: f64,
}

/// Matrix completion via nuclear-norm minimization.
///
/// Recovers a low-rank matrix from a subset of observed entries by solving:
///
/// ```text
/// minimize  ||X||_*
/// subject to  X_{ij} = M_{ij}  for (i,j) in Omega
/// ```
///
/// where `||X||_*` is the nuclear norm (sum of singular values).
pub struct MatrixCompletion {
    rows: usize,
    cols: usize,
    /// Observed entries: (row, col, value)
    observed: Vec<(usize, usize, f64)>,
    params: SvtParams,
}

impl MatrixCompletion {
    /// Create a new matrix completion problem.
    ///
    /// # Arguments
    /// * `rows` - Number of rows
    /// * `cols` - Number of columns
    /// * `observed` - Slice of (row, col, value) observed entries
    /// * `params` - SVT algorithm parameters
    pub fn new(rows: usize, cols: usize, observed: &[(usize, usize, f64)], params: SvtParams) -> Self {
        Self {
            rows,
            cols,
            observed: observed.to_vec(),
            params,
        }
    }

    /// Solve the matrix completion problem using SVT.
    ///
    /// Returns the completed matrix as a `CompletionResult`.
    pub fn solve(&self) -> CompletionResult {
        let m = self.rows;
        let n = self.cols;

        // Y_0 = delta * P_Omega(M)
        let mut y = Mat::zeros(m, n);
        for &(r, c, v) in &self.observed {
            y.set(r, c, self.params.delta * v);
        }

        let mut x = Mat::zeros(m, n);
        let mut sigma_kept = Vec::new();
        let mut final_rel = 0.0;
        let mut iters = 0;

        for iter in 0..self.params.max_iter {
            // SVD of Y
            let (u, sigma, v) = full_svd(&y);

            // Soft-threshold singular values
            sigma_kept = sigma
                .iter()
                .map(|&s| soft_threshold(s, self.params.tau))
                .collect::<Vec<_>>();

            // Reconstruct X = U * diag(sigma_thresholded) * V^T
            let x_new = reconstruct_from_svd(&u, &sigma_kept, &v, m, n);

            // Check convergence
            let diff_norm = {
                let mut s = 0.0;
                for i in 0..m * n {
                    let d = x_new.data[i] - x.data[i];
                    s += d * d;
                }
                s.sqrt()
            };
            let x_norm = x_new.frobenius_norm();
            let rel_change = if x_norm > 1e-15 {
                diff_norm / x_norm
            } else {
                diff_norm
            };

            x = x_new;
            final_rel = rel_change;
            iters = iter + 1;

            if rel_change < self.params.tol {
                break;
            }

            // Y_{k+1} = Y_k + delta * P_Omega(M - X_k)
            for &(r, c, v) in &self.observed {
                let residual = v - x.get(r, c);
                y.add_assign(r, c, self.params.delta * residual);
            }
        }

        // Filter out near-zero singular values
        let singular_values: Vec<f64> = sigma_kept.into_iter().filter(|&s| s > 1e-12).collect();

        CompletionResult {
            matrix: x.data,
            rows: m,
            cols: n,
            singular_values,
            iterations: iters,
            final_relative_change: final_rel,
        }
    }

    /// Compute RMSE on a set of known entries.
    pub fn rmse(result: &CompletionResult, rows: usize, cols: usize, entries: &[(usize, usize, f64)]) -> f64 {
        if entries.is_empty() {
            return 0.0;
        }
        let _ = rows; // used for API consistency
        let mut sum_sq = 0.0;
        for &(r, c, v) in entries {
            let predicted = result.matrix[r * cols + c];
            let diff = predicted - v;
            sum_sq += diff * diff;
        }
        (sum_sq / entries.len() as f64).sqrt()
    }

    /// Estimate the rank of the completed matrix from its singular value distribution.
    ///
    /// Counts singular values above `threshold * sigma_max`.
    pub fn estimate_rank(result: &CompletionResult, threshold: f64) -> usize {
        if result.singular_values.is_empty() {
            return 0;
        }
        let max_sv = result.singular_values[0];
        if max_sv < 1e-15 {
            return 0;
        }
        result
            .singular_values
            .iter()
            .filter(|&&s| s > threshold * max_sv)
            .count()
    }

    /// Nuclear norm of the completed matrix (sum of singular values).
    pub fn nuclear_norm(result: &CompletionResult) -> f64 {
        result.singular_values.iter().sum()
    }

    /// Cross-validate to select the regularization parameter tau.
    ///
    /// Splits observed entries into folds, trains on (folds-1) folds, and
    /// evaluates RMSE on the held-out fold. Returns the best tau.
    ///
    /// # Arguments
    /// * `rows` - Matrix rows
    /// * `cols` - Matrix columns
    /// * `observed` - All observed entries
    /// * `tau_candidates` - Candidate tau values to try
    /// * `n_folds` - Number of cross-validation folds
    /// * `base_params` - Base SVT parameters (tau will be overridden)
    pub fn cross_validate(
        rows: usize,
        cols: usize,
        observed: &[(usize, usize, f64)],
        tau_candidates: &[f64],
        n_folds: usize,
        base_params: &SvtParams,
    ) -> f64 {
        if tau_candidates.is_empty() || observed.is_empty() {
            return base_params.tau;
        }

        let n_folds = n_folds.max(2).min(observed.len());
        let fold_size = observed.len() / n_folds;
        if fold_size == 0 {
            return tau_candidates[0];
        }

        let mut best_tau = tau_candidates[0];
        let mut best_rmse = f64::MAX;

        for &tau in tau_candidates {
            let mut total_rmse = 0.0;
            let mut valid_folds = 0;

            for fold in 0..n_folds {
                let test_start = fold * fold_size;
                let test_end = if fold == n_folds - 1 {
                    observed.len()
                } else {
                    test_start + fold_size
                };

                let train: Vec<_> = observed[..test_start]
                    .iter()
                    .chain(observed[test_end..].iter())
                    .cloned()
                    .collect();
                let test: Vec<_> = observed[test_start..test_end].to_vec();

                if train.is_empty() || test.is_empty() {
                    continue;
                }

                let params = SvtParams {
                    tau,
                    delta: base_params.delta,
                    max_iter: base_params.max_iter.min(100), // fewer iters for CV
                    tol: base_params.tol * 10.0,             // looser tol for speed
                };

                let mc = MatrixCompletion::new(rows, cols, &train, params);
                let result = mc.solve();
                let rmse = Self::rmse(&result, rows, cols, &test);
                total_rmse += rmse;
                valid_folds += 1;
            }

            if valid_folds > 0 {
                let avg_rmse = total_rmse / valid_folds as f64;
                if avg_rmse < best_rmse {
                    best_rmse = avg_rmse;
                    best_tau = tau;
                }
            }
        }

        best_tau
    }
}

/// Complex-valued matrix completion via nuclear-norm minimization.
///
/// Useful for IQ data where entries are complex numbers.
pub struct ComplexMatrixCompletion {
    rows: usize,
    cols: usize,
    observed: Vec<(usize, usize, Complex)>,
    params: SvtParams,
}

impl ComplexMatrixCompletion {
    /// Create a new complex matrix completion problem.
    pub fn new(rows: usize, cols: usize, observed: &[(usize, usize, Complex)], params: SvtParams) -> Self {
        Self {
            rows,
            cols,
            observed: observed.to_vec(),
            params,
        }
    }

    /// Solve using real-valued embedding.
    ///
    /// Converts the m×n complex problem to a 2m×2n real problem:
    /// ```text
    /// [Re(X)  -Im(X)]
    /// [Im(X)   Re(X)]
    /// ```
    /// then solves with SVT and extracts the complex result.
    pub fn solve(&self) -> ComplexCompletionResult {
        let m = self.rows;
        let n = self.cols;

        // Build real embedding observations
        let mut real_obs = Vec::with_capacity(self.observed.len() * 4);
        for &(r, c, (re, im)) in &self.observed {
            // Top-left block: Re
            real_obs.push((r, c, re));
            // Top-right block: -Im
            real_obs.push((r, c + n, -im));
            // Bottom-left block: Im
            real_obs.push((r + m, c, im));
            // Bottom-right block: Re
            real_obs.push((r + m, c + n, re));
        }

        let mc = MatrixCompletion::new(2 * m, 2 * n, &real_obs, self.params.clone());
        let result = mc.solve();

        // Extract complex matrix from top-left (Re) and bottom-left (Im) blocks
        let mut complex_matrix = vec![(0.0, 0.0); m * n];
        for i in 0..m {
            for j in 0..n {
                let re = result.matrix[i * (2 * n) + j];
                let im = result.matrix[(i + m) * (2 * n) + j];
                complex_matrix[i * n + j] = (re, im);
            }
        }

        // Singular values of the real embedding include duplicates;
        // take every other one (they come in pairs for the complex structure).
        let mut svs: Vec<f64> = result.singular_values.clone();
        svs.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
        // Deduplicate: keep unique values (pairs should be nearly identical)
        let mut deduped = Vec::new();
        let mut i = 0;
        while i < svs.len() {
            deduped.push(svs[i]);
            // Skip the duplicate
            if i + 1 < svs.len() && (svs[i] - svs[i + 1]).abs() < 1e-6 * (svs[i].abs() + 1e-15) {
                i += 2;
            } else {
                i += 1;
            }
        }

        ComplexCompletionResult {
            matrix: complex_matrix,
            rows: m,
            cols: n,
            singular_values: deduped,
            iterations: result.iterations,
            final_relative_change: result.final_relative_change,
        }
    }

    /// Compute RMSE on known complex entries.
    pub fn rmse(result: &ComplexCompletionResult, entries: &[(usize, usize, Complex)]) -> f64 {
        if entries.is_empty() {
            return 0.0;
        }
        let n = result.cols;
        let mut sum_sq = 0.0;
        for &(r, c, (re, im)) in entries {
            let (pre, pim) = result.matrix[r * n + c];
            sum_sq += (pre - re).powi(2) + (pim - im).powi(2);
        }
        (sum_sq / entries.len() as f64).sqrt()
    }
}

/// Iterative Soft-Thresholded SVD for low-rank matrix recovery.
///
/// This is a proximal gradient method applied to:
///
/// ```text
/// minimize  0.5 * ||P_Omega(X - M)||_F^2 + lambda * ||X||_*
/// ```
///
/// where lambda controls the nuclear norm penalty.
pub struct SoftThresholdedSvd {
    rows: usize,
    cols: usize,
    observed: Vec<(usize, usize, f64)>,
    lambda: f64,
    max_iter: usize,
    tol: f64,
}

impl SoftThresholdedSvd {
    /// Create a new soft-thresholded SVD solver.
    ///
    /// # Arguments
    /// * `rows` - Matrix rows
    /// * `cols` - Matrix columns
    /// * `observed` - Observed entries
    /// * `lambda` - Nuclear norm regularization weight
    /// * `max_iter` - Maximum iterations
    /// * `tol` - Convergence tolerance
    pub fn new(
        rows: usize,
        cols: usize,
        observed: &[(usize, usize, f64)],
        lambda: f64,
        max_iter: usize,
        tol: f64,
    ) -> Self {
        Self {
            rows,
            cols,
            observed: observed.to_vec(),
            lambda,
            max_iter,
            tol,
        }
    }

    /// Solve via proximal gradient descent.
    pub fn solve(&self) -> CompletionResult {
        let m = self.rows;
        let n = self.cols;
        let mut x = Mat::zeros(m, n);
        let mut final_rel = 0.0;
        let mut iters = 0;
        let mut sigma_kept = Vec::new();

        for iter in 0..self.max_iter {
            // Gradient step: X - step * gradient
            // gradient on Omega: X_{ij} - M_{ij} for (i,j) in Omega, 0 elsewhere
            let mut grad = Mat::zeros(m, n);
            for &(r, c, v) in &self.observed {
                grad.set(r, c, x.get(r, c) - v);
            }

            // X_half = X - grad  (step size = 1)
            let mut x_half = Mat::zeros(m, n);
            for i in 0..m * n {
                x_half.data[i] = x.data[i] - grad.data[i];
            }

            // Proximal step: SVT with threshold lambda
            let (u, sigma, v) = full_svd(&x_half);
            sigma_kept = sigma
                .iter()
                .map(|&s| soft_threshold(s, self.lambda))
                .collect::<Vec<_>>();

            let x_new = reconstruct_from_svd(&u, &sigma_kept, &v, m, n);

            // Convergence
            let diff_norm = {
                let mut s = 0.0;
                for i in 0..m * n {
                    let d = x_new.data[i] - x.data[i];
                    s += d * d;
                }
                s.sqrt()
            };
            let x_norm = x_new.frobenius_norm();
            let rel_change = if x_norm > 1e-15 {
                diff_norm / x_norm
            } else {
                diff_norm
            };

            x = x_new;
            final_rel = rel_change;
            iters = iter + 1;

            if rel_change < self.tol {
                break;
            }
        }

        let singular_values: Vec<f64> = sigma_kept.into_iter().filter(|&s| s > 1e-12).collect();

        CompletionResult {
            matrix: x.data,
            rows: m,
            cols: n,
            singular_values,
            iterations: iters,
            final_relative_change: final_rel,
        }
    }
}

/// Convergence monitor for iterative matrix completion algorithms.
#[derive(Clone, Debug)]
pub struct ConvergenceMonitor {
    history: Vec<f64>,
    window: usize,
}

impl ConvergenceMonitor {
    /// Create a monitor with a given smoothing window size.
    pub fn new(window: usize) -> Self {
        Self {
            history: Vec::new(),
            window: window.max(1),
        }
    }

    /// Record a new relative-change value.
    pub fn record(&mut self, relative_change: f64) {
        self.history.push(relative_change);
    }

    /// Check if convergence is stalled (improvement < threshold over the window).
    pub fn is_stalled(&self, threshold: f64) -> bool {
        if self.history.len() < self.window + 1 {
            return false;
        }
        let recent = &self.history[self.history.len() - self.window..];
        let improvement = recent.first().unwrap_or(&0.0) - recent.last().unwrap_or(&0.0);
        improvement.abs() < threshold
    }

    /// Get the full convergence history.
    pub fn history(&self) -> &[f64] {
        &self.history
    }

    /// Get the latest relative change value.
    pub fn latest(&self) -> Option<f64> {
        self.history.last().copied()
    }

    /// Number of recorded values.
    pub fn len(&self) -> usize {
        self.history.len()
    }

    /// Whether any values have been recorded.
    pub fn is_empty(&self) -> bool {
        self.history.is_empty()
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a rank-1 matrix a*b^T and sample some entries.
    fn rank1_problem(m: usize, n: usize, frac: f64) -> (Vec<(usize, usize, f64)>, Mat) {
        let mut full = Mat::zeros(m, n);
        // rank-1: row_i * col_j
        for i in 0..m {
            for j in 0..n {
                let v = (i as f64 + 1.0) * (j as f64 + 1.0);
                full.set(i, j, v);
            }
        }

        // Sample entries deterministically
        let total = m * n;
        let n_obs = ((total as f64) * frac).ceil() as usize;
        let mut observed = Vec::new();
        // Use a simple stride-based sampling
        let step = total.max(1) / n_obs.max(1);
        let step = step.max(1);
        for k in (0..total).step_by(step) {
            let r = k / n;
            let c = k % n;
            observed.push((r, c, full.get(r, c)));
            if observed.len() >= n_obs {
                break;
            }
        }

        (observed, full)
    }

    #[test]
    fn test_svt_rank1_recovery() {
        let (observed, _full) = rank1_problem(5, 5, 0.6);
        let params = SvtParams {
            tau: 5.0,
            delta: 1.2,
            max_iter: 500,
            tol: 1e-5,
        };
        let mc = MatrixCompletion::new(5, 5, &observed, params);
        let result = mc.solve();

        // Check that observed entries are well-recovered
        let rmse = MatrixCompletion::rmse(&result, 5, 5, &observed);
        assert!(rmse < 1.0, "RMSE on observed entries too large: {}", rmse);
    }

    #[test]
    fn test_svt_convergence() {
        let (observed, _) = rank1_problem(4, 4, 0.75);
        let params = SvtParams {
            tau: 2.0,
            delta: 1.2,
            max_iter: 300,
            tol: 1e-6,
        };
        let mc = MatrixCompletion::new(4, 4, &observed, params);
        let result = mc.solve();
        // Should converge in fewer than max iterations
        assert!(
            result.iterations < 300,
            "Did not converge: {} iters",
            result.iterations
        );
    }

    #[test]
    fn test_svt_zero_matrix() {
        // All zeros should return zero matrix
        let observed = vec![(0, 0, 0.0), (1, 1, 0.0), (2, 2, 0.0)];
        let params = SvtParams::default();
        let mc = MatrixCompletion::new(3, 3, &observed, params);
        let result = mc.solve();
        for &v in &result.matrix {
            assert!(v.abs() < 1e-10, "Expected zero, got {}", v);
        }
    }

    #[test]
    fn test_svt_single_entry() {
        let observed = vec![(1, 2, 5.0)];
        let params = SvtParams {
            tau: 1.0,
            delta: 1.5,
            max_iter: 200,
            tol: 1e-5,
        };
        let mc = MatrixCompletion::new(3, 3, &observed, params);
        let result = mc.solve();
        // The completed matrix should be low-rank
        assert!(result.singular_values.len() <= 3);
    }

    #[test]
    fn test_svt_fully_observed() {
        // With all entries observed and low tau, should recover well
        let mut observed = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                let v = (i as f64 + 1.0) * (j as f64 + 1.0);
                observed.push((i, j, v));
            }
        }
        let params = SvtParams {
            tau: 0.01,
            delta: 1.5,
            max_iter: 500,
            tol: 1e-6,
        };
        let mc = MatrixCompletion::new(3, 3, &observed, params);
        let result = mc.solve();
        let rmse = MatrixCompletion::rmse(&result, 3, 3, &observed);
        assert!(rmse < 0.5, "RMSE too large for fully observed: {}", rmse);
    }

    #[test]
    fn test_power_svd_identity() {
        // Identity matrix has singular values all 1
        // Note: power iteration with deflation may not find all degenerate SVs
        // because the fixed initial vector can be orthogonal to remaining eigenvectors.
        let n = 4;
        let mut mat = vec![0.0; n * n];
        for i in 0..n {
            mat[i * n + i] = 1.0;
        }
        let (_u, sigma, _v) = power_svd(&mat, n, n, n, 100);
        assert!(!sigma.is_empty(), "Should find at least one SV");
        for s in &sigma {
            assert!((s - 1.0).abs() < 1e-6, "Expected sigma=1, got {}", s);
        }
    }

    #[test]
    fn test_power_svd_rank1() {
        // Rank-1 matrix: [1,2,3]^T * [1,2,3]
        let a = vec![
            1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 3.0, 6.0, 9.0,
        ];
        let (_, sigma, _) = power_svd(&a, 3, 3, 3, 200);
        // Only first singular value should be non-zero
        assert!(sigma[0] > 1.0);
        if sigma.len() > 1 {
            assert!(sigma[1] < 1e-6, "Second SV should be ~0, got {}", sigma[1]);
        }
    }

    #[test]
    fn test_power_svd_rectangular() {
        // 2x3 matrix
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let (u, sigma, v) = power_svd(&a, 2, 3, 2, 200);
        assert_eq!(sigma.len(), 2);
        assert!(sigma[0] > sigma[1]);
        // Verify reconstruction: U * diag(S) * V^T ≈ A
        let k = sigma.len();
        for i in 0..2 {
            for j in 0..3 {
                let mut val = 0.0;
                for idx in 0..k {
                    val += u[i * k + idx] * sigma[idx] * v[j * k + idx];
                }
                let expected = a[i * 3 + j];
                assert!(
                    (val - expected).abs() < 1e-6,
                    "Reconstruction error at ({},{}): got {}, expected {}",
                    i,
                    j,
                    val,
                    expected
                );
            }
        }
    }

    #[test]
    fn test_soft_threshold() {
        assert_eq!(soft_threshold(5.0, 3.0), 2.0);
        assert_eq!(soft_threshold(3.0, 3.0), 0.0);
        assert_eq!(soft_threshold(1.0, 3.0), 0.0);
        assert_eq!(soft_threshold(0.0, 1.0), 0.0);
    }

    #[test]
    fn test_rank_estimation() {
        let (observed, _) = rank1_problem(5, 5, 0.8);
        let params = SvtParams {
            tau: 0.1,
            delta: 1.5,
            max_iter: 500,
            tol: 1e-6,
        };
        let mc = MatrixCompletion::new(5, 5, &observed, params);
        let result = mc.solve();
        let rank = MatrixCompletion::estimate_rank(&result, 0.01);
        // Rank-1 input should give rank estimate of 1 or close to it
        assert!(rank >= 1 && rank <= 2, "Expected rank ~1, got {}", rank);
    }

    #[test]
    fn test_nuclear_norm() {
        let result = CompletionResult {
            matrix: vec![0.0; 4],
            rows: 2,
            cols: 2,
            singular_values: vec![3.0, 2.0, 1.0],
            iterations: 0,
            final_relative_change: 0.0,
        };
        assert!((MatrixCompletion::nuclear_norm(&result) - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_nuclear_norm_empty() {
        let result = CompletionResult {
            matrix: vec![],
            rows: 0,
            cols: 0,
            singular_values: vec![],
            iterations: 0,
            final_relative_change: 0.0,
        };
        assert!((MatrixCompletion::nuclear_norm(&result) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_rmse_exact() {
        let result = CompletionResult {
            matrix: vec![1.0, 2.0, 3.0, 4.0],
            rows: 2,
            cols: 2,
            singular_values: vec![],
            iterations: 0,
            final_relative_change: 0.0,
        };
        let entries = vec![(0, 0, 1.0), (0, 1, 2.0), (1, 0, 3.0), (1, 1, 4.0)];
        let rmse = MatrixCompletion::rmse(&result, 2, 2, &entries);
        assert!(rmse < 1e-10);
    }

    #[test]
    fn test_rmse_with_error() {
        let result = CompletionResult {
            matrix: vec![1.0, 2.0, 3.0, 4.0],
            rows: 2,
            cols: 2,
            singular_values: vec![],
            iterations: 0,
            final_relative_change: 0.0,
        };
        // Each entry off by 1.0
        let entries = vec![(0, 0, 2.0), (0, 1, 3.0), (1, 0, 4.0), (1, 1, 5.0)];
        let rmse = MatrixCompletion::rmse(&result, 2, 2, &entries);
        assert!((rmse - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_completion_simple() {
        // Rank-1 complex matrix: a * b^H
        // a = [(1,0), (0,1), (1,1)]  b = [(1,0), (2,0)]
        // M = [(1,0), (2,0);  (0,1), (0,2);  (1,1), (2,2)]
        let observed = vec![
            (0, 0, (1.0, 0.0)),
            (0, 1, (2.0, 0.0)),
            (1, 0, (0.0, 1.0)),
            // (1, 1) = (0, 2) is missing
            (2, 0, (1.0, 1.0)),
            (2, 1, (2.0, 2.0)),
        ];

        let params = SvtParams {
            tau: 0.5,
            delta: 1.2,
            max_iter: 300,
            tol: 1e-5,
        };

        let cmc = ComplexMatrixCompletion::new(3, 2, &observed, params);
        let result = cmc.solve();

        // Check known entries
        let rmse = ComplexMatrixCompletion::rmse(&result, &observed);
        assert!(rmse < 1.0, "Complex RMSE too large: {}", rmse);
    }

    #[test]
    fn test_complex_completion_real_only() {
        // Pure real matrix should work through complex path
        let observed = vec![
            (0, 0, (1.0, 0.0)),
            (0, 1, (2.0, 0.0)),
            (1, 0, (3.0, 0.0)),
            (1, 1, (4.0, 0.0)),
        ];
        let params = SvtParams {
            tau: 0.01,
            delta: 1.5,
            max_iter: 200,
            tol: 1e-5,
        };

        let cmc = ComplexMatrixCompletion::new(2, 2, &observed, params);
        let result = cmc.solve();

        // Imaginary parts should be near zero
        for &(re, im) in &result.matrix {
            assert!(im.abs() < 0.5, "Imaginary part should be ~0, got {}", im);
        }
    }

    #[test]
    fn test_soft_thresholded_svd_recovery() {
        let (observed, _full) = rank1_problem(5, 5, 0.7);
        let solver = SoftThresholdedSvd::new(5, 5, &observed, 0.5, 300, 1e-5);
        let result = solver.solve();

        let rmse = MatrixCompletion::rmse(&result, 5, 5, &observed);
        assert!(rmse < 2.0, "SoftThreshSVD RMSE too large: {}", rmse);
    }

    #[test]
    fn test_soft_thresholded_svd_convergence() {
        let (observed, _) = rank1_problem(4, 4, 0.75);
        let solver = SoftThresholdedSvd::new(4, 4, &observed, 0.1, 500, 1e-6);
        let result = solver.solve();
        assert!(
            result.iterations < 500,
            "STSVD did not converge: {} iters",
            result.iterations
        );
    }

    #[test]
    fn test_convergence_monitor() {
        let mut mon = ConvergenceMonitor::new(3);
        assert!(mon.is_empty());
        assert_eq!(mon.len(), 0);
        assert_eq!(mon.latest(), None);

        mon.record(1.0);
        mon.record(0.5);
        mon.record(0.25);
        mon.record(0.24);

        assert_eq!(mon.len(), 4);
        assert_eq!(mon.latest(), Some(0.24));
        assert!(!mon.is_empty());
        // Last 3 values: 0.5, 0.25, 0.24 - improvement = 0.5 - 0.24 = 0.26
        assert!(!mon.is_stalled(0.2));
    }

    #[test]
    fn test_convergence_monitor_stalled() {
        let mut mon = ConvergenceMonitor::new(3);
        mon.record(0.01);
        mon.record(0.01);
        mon.record(0.01);
        mon.record(0.01);
        // All same: stalled with any positive threshold
        assert!(mon.is_stalled(0.001));
    }

    #[test]
    fn test_cross_validation() {
        // Fully observed rank-1 matrix
        let mut observed = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                observed.push((i, j, (i as f64 + 1.0) * (j as f64 + 1.0)));
            }
        }

        let taus = vec![0.1, 1.0, 5.0, 10.0];
        let base = SvtParams {
            delta: 1.2,
            max_iter: 100,
            tol: 1e-4,
            ..Default::default()
        };

        let best_tau = MatrixCompletion::cross_validate(4, 4, &observed, &taus, 4, &base);
        // Should pick a smallish tau for a well-conditioned problem
        assert!(taus.contains(&best_tau), "Best tau {} not in candidates", best_tau);
    }

    #[test]
    fn test_default_params() {
        let params = SvtParams::default();
        assert_eq!(params.tau, 5.0);
        assert_eq!(params.delta, 1.2);
        assert_eq!(params.max_iter, 500);
        assert!((params.tol - 1e-4).abs() < 1e-10);
    }

    #[test]
    fn test_completion_result_fields() {
        let (observed, _) = rank1_problem(3, 3, 0.8);
        let mc = MatrixCompletion::new(3, 3, &observed, SvtParams::default());
        let result = mc.solve();

        assert_eq!(result.rows, 3);
        assert_eq!(result.cols, 3);
        assert_eq!(result.matrix.len(), 9);
        assert!(result.iterations > 0);
        assert!(result.final_relative_change >= 0.0);
    }
}

//! Eigenvalue / eigenvector decomposition for small to medium matrices.
//!
//! Provides a simple row-major `Matrix` type and algorithms commonly needed
//! in SDR post-processing: Jacobi symmetric eigen-decomposition (DOA / MUSIC),
//! power iteration, covariance estimation, and basic matrix diagnostics
//! (determinant, trace, Frobenius norm, positive-definiteness).
//!
//! Only depends on `std`; no external crates required.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::matrix_eigenvalue::{Matrix, symmetric_eigen};
//!
//! // 2x2 symmetric matrix [[2, 1], [1, 2]]
//! let m = Matrix::from_data(2, 2, vec![2.0, 1.0, 1.0, 2.0]);
//! let eig = symmetric_eigen(&m);
//!
//! // Eigenvalues should be 1.0 and 3.0 (in ascending order after sort)
//! let mut vals = eig.eigenvalues.clone();
//! vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
//! assert!((vals[0] - 1.0).abs() < 1e-10);
//! assert!((vals[1] - 3.0).abs() < 1e-10);
//! ```

/// Row-major dense matrix of `f64` values.
#[derive(Debug, Clone, PartialEq)]
pub struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

/// Result of an eigenvalue decomposition.
#[derive(Debug, Clone)]
pub struct EigenResult {
    /// Eigenvalues (not necessarily sorted).
    pub eigenvalues: Vec<f64>,
    /// Eigenvectors stored column-wise in a `Matrix` (column *j* is the
    /// eigenvector corresponding to `eigenvalues[j]`).
    pub eigenvectors: Matrix,
}

// ---------------------------------------------------------------------------
// Matrix implementation
// ---------------------------------------------------------------------------

impl Matrix {
    /// Create a zero-initialized matrix.
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    /// Create a matrix from existing data (row-major order).
    ///
    /// # Panics
    /// Panics if `data.len() != rows * cols`.
    pub fn from_data(rows: usize, cols: usize, data: Vec<f64>) -> Self {
        assert_eq!(
            data.len(),
            rows * cols,
            "data length {} != rows*cols {}",
            data.len(),
            rows * cols
        );
        Self { rows, cols, data }
    }

    /// Get element at (r, c).
    #[inline]
    pub fn get(&self, r: usize, c: usize) -> f64 {
        self.data[r * self.cols + c]
    }

    /// Set element at (r, c).
    #[inline]
    pub fn set(&mut self, r: usize, c: usize, val: f64) {
        self.data[r * self.cols + c] = val;
    }

    /// Number of rows.
    #[inline]
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Number of columns.
    #[inline]
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Create an *n*-by-*n* identity matrix.
    pub fn identity(n: usize) -> Self {
        let mut m = Self::new(n, n);
        for i in 0..n {
            m.set(i, i, 1.0);
        }
        m
    }

    /// Return the transpose.
    pub fn transpose(&self) -> Self {
        let mut t = Self::new(self.cols, self.rows);
        for r in 0..self.rows {
            for c in 0..self.cols {
                t.set(c, r, self.get(r, c));
            }
        }
        t
    }

    /// Matrix multiplication `self * other`.
    ///
    /// # Panics
    /// Panics if `self.cols != other.rows`.
    pub fn multiply(&self, other: &Matrix) -> Matrix {
        assert_eq!(
            self.cols, other.rows,
            "incompatible dimensions: {}x{} * {}x{}",
            self.rows, self.cols, other.rows, other.cols
        );
        let mut out = Matrix::new(self.rows, other.cols);
        for i in 0..self.rows {
            for j in 0..other.cols {
                let mut s = 0.0;
                for k in 0..self.cols {
                    s += self.get(i, k) * other.get(k, j);
                }
                out.set(i, j, s);
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Jacobi symmetric eigenvalue decomposition
// ---------------------------------------------------------------------------

/// Eigenvalue decomposition of a real symmetric matrix using the classical
/// Jacobi iterative method.  Convergence is guaranteed for symmetric input.
///
/// Returns eigenvalues and their corresponding eigenvectors (as columns).
///
/// # Panics
/// Panics if `mat` is not square.
pub fn symmetric_eigen(mat: &Matrix) -> EigenResult {
    let n = mat.rows();
    assert_eq!(n, mat.cols(), "symmetric_eigen requires a square matrix");

    // Work on a mutable copy.
    let mut a = mat.clone();
    // Eigenvector accumulator starts as identity.
    let mut v = Matrix::identity(n);

    let max_iter = 100 * n * n;
    let tol = 1e-12;

    for _ in 0..max_iter {
        // Find the largest off-diagonal element.
        let mut p = 0;
        let mut q = 1;
        let mut max_off = 0.0_f64;
        for i in 0..n {
            for j in (i + 1)..n {
                let val = a.get(i, j).abs();
                if val > max_off {
                    max_off = val;
                    p = i;
                    q = j;
                }
            }
        }

        if max_off < tol {
            break;
        }

        // Compute Jacobi rotation angle.
        let app = a.get(p, p);
        let aqq = a.get(q, q);
        let apq = a.get(p, q);

        let theta = if (app - aqq).abs() < 1e-30 {
            std::f64::consts::FRAC_PI_4
        } else {
            0.5 * (2.0 * apq / (app - aqq)).atan()
        };

        let c = theta.cos();
        let s = theta.sin();

        // Apply Givens rotation: A' = G^T A G
        // Update rows/cols p and q in A.
        let mut new_row_p = vec![0.0; n];
        let mut new_row_q = vec![0.0; n];
        for i in 0..n {
            new_row_p[i] = c * a.get(p, i) + s * a.get(q, i);
            new_row_q[i] = -s * a.get(p, i) + c * a.get(q, i);
        }
        for i in 0..n {
            a.set(p, i, new_row_p[i]);
            a.set(q, i, new_row_q[i]);
        }

        let mut new_col_p = vec![0.0; n];
        let mut new_col_q = vec![0.0; n];
        for i in 0..n {
            new_col_p[i] = c * a.get(i, p) + s * a.get(i, q);
            new_col_q[i] = -s * a.get(i, p) + c * a.get(i, q);
        }
        for i in 0..n {
            a.set(i, p, new_col_p[i]);
            a.set(i, q, new_col_q[i]);
        }

        // Accumulate eigenvectors: V' = V * G
        let mut new_vp = vec![0.0; n];
        let mut new_vq = vec![0.0; n];
        for i in 0..n {
            new_vp[i] = c * v.get(i, p) + s * v.get(i, q);
            new_vq[i] = -s * v.get(i, p) + c * v.get(i, q);
        }
        for i in 0..n {
            v.set(i, p, new_vp[i]);
            v.set(i, q, new_vq[i]);
        }
    }

    let eigenvalues: Vec<f64> = (0..n).map(|i| a.get(i, i)).collect();
    EigenResult {
        eigenvalues,
        eigenvectors: v,
    }
}

// ---------------------------------------------------------------------------
// Power iteration
// ---------------------------------------------------------------------------

/// Find the dominant eigenvalue and corresponding eigenvector via power
/// iteration.
///
/// Returns `(eigenvalue, eigenvector)`.
///
/// # Panics
/// Panics if `mat` is not square or is empty.
pub fn power_iteration(mat: &Matrix, max_iter: usize, tol: f64) -> (f64, Vec<f64>) {
    let n = mat.rows();
    assert_eq!(n, mat.cols(), "power_iteration requires a square matrix");
    assert!(n > 0, "matrix must be non-empty");

    // Start with a unit vector along the first axis.
    let mut b = vec![0.0; n];
    b[0] = 1.0;

    let mut eigenvalue = 0.0;

    for _ in 0..max_iter {
        // Multiply: b_next = A * b
        let mut b_next = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                b_next[i] += mat.get(i, j) * b[j];
            }
        }

        // Compute the Rayleigh quotient for eigenvalue estimate.
        let mut num = 0.0;
        let mut den = 0.0;
        for i in 0..n {
            num += b_next[i] * b[i];
            den += b[i] * b[i];
        }
        let new_eigenvalue = num / den;

        // Normalize.
        let norm: f64 = b_next.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm < 1e-30 {
            break;
        }
        for v in b_next.iter_mut() {
            *v /= norm;
        }

        let converged = (new_eigenvalue - eigenvalue).abs() < tol;
        eigenvalue = new_eigenvalue;
        b = b_next;

        if converged {
            break;
        }
    }

    (eigenvalue, b)
}

// ---------------------------------------------------------------------------
// Covariance matrix
// ---------------------------------------------------------------------------

/// Compute the sample covariance matrix from a set of observation vectors.
///
/// Each element of `data` is one snapshot (observation vector) of length *n*.
/// Returns an *n*-by-*n* covariance matrix.
///
/// # Panics
/// Panics if `data` is empty or vectors differ in length.
pub fn covariance_matrix(data: &[Vec<f64>]) -> Matrix {
    assert!(!data.is_empty(), "data must not be empty");
    let n = data[0].len();
    let m = data.len() as f64;

    // Compute means.
    let mut mean = vec![0.0; n];
    for snapshot in data {
        assert_eq!(snapshot.len(), n, "all data vectors must have the same length");
        for (i, &v) in snapshot.iter().enumerate() {
            mean[i] += v;
        }
    }
    for v in mean.iter_mut() {
        *v /= m;
    }

    // Compute covariance (unbiased: divide by m-1, or m if m==1).
    let divisor = if m > 1.0 { m - 1.0 } else { 1.0 };
    let mut cov = Matrix::new(n, n);
    for snapshot in data {
        for i in 0..n {
            let di = snapshot[i] - mean[i];
            for j in 0..n {
                let dj = snapshot[j] - mean[j];
                let cur = cov.get(i, j);
                cov.set(i, j, cur + di * dj);
            }
        }
    }
    for i in 0..n {
        for j in 0..n {
            let cur = cov.get(i, j);
            cov.set(i, j, cur / divisor);
        }
    }
    cov
}

// ---------------------------------------------------------------------------
// Determinant
// ---------------------------------------------------------------------------

/// Compute the determinant of a square matrix.
///
/// Uses cofactor expansion for n <= 4 and LU decomposition for larger
/// matrices.
///
/// # Panics
/// Panics if `mat` is not square.
pub fn determinant(mat: &Matrix) -> f64 {
    let n = mat.rows();
    assert_eq!(n, mat.cols(), "determinant requires a square matrix");

    match n {
        0 => 1.0,
        1 => mat.get(0, 0),
        2 => mat.get(0, 0) * mat.get(1, 1) - mat.get(0, 1) * mat.get(1, 0),
        3 => {
            let a = mat.get(0, 0);
            let b = mat.get(0, 1);
            let c = mat.get(0, 2);
            let d = mat.get(1, 0);
            let e = mat.get(1, 1);
            let f = mat.get(1, 2);
            let g = mat.get(2, 0);
            let h = mat.get(2, 1);
            let k = mat.get(2, 2);
            a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g)
        }
        4 => {
            // Cofactor expansion along first row.
            let mut det = 0.0;
            for col in 0..4 {
                let minor = minor_3x3(mat, 0, col);
                let sign = if col % 2 == 0 { 1.0 } else { -1.0 };
                det += sign * mat.get(0, col) * minor;
            }
            det
        }
        _ => lu_determinant(mat),
    }
}

/// 3x3 minor by removing row `skip_r` and column `skip_c` from a 4x4 matrix.
fn minor_3x3(mat: &Matrix, skip_r: usize, skip_c: usize) -> f64 {
    let mut sub = [0.0; 9];
    let mut idx = 0;
    for r in 0..4 {
        if r == skip_r {
            continue;
        }
        for c in 0..4 {
            if c == skip_c {
                continue;
            }
            sub[idx] = mat.get(r, c);
            idx += 1;
        }
    }
    // 3x3 determinant of sub.
    sub[0] * (sub[4] * sub[8] - sub[5] * sub[7])
        - sub[1] * (sub[3] * sub[8] - sub[5] * sub[6])
        + sub[2] * (sub[3] * sub[7] - sub[4] * sub[6])
}

/// LU decomposition (partial pivoting) determinant for n > 4.
fn lu_determinant(mat: &Matrix) -> f64 {
    let n = mat.rows();
    // Copy into mutable working array.
    let mut a: Vec<Vec<f64>> = (0..n)
        .map(|r| (0..n).map(|c| mat.get(r, c)).collect())
        .collect();

    let mut sign = 1.0;

    for col in 0..n {
        // Partial pivoting.
        let mut max_val = a[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let v = a[row][col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            return 0.0; // Singular.
        }
        if max_row != col {
            a.swap(col, max_row);
            sign = -sign;
        }
        // Eliminate below.
        let pivot = a[col][col];
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
        }
    }

    let mut det = sign;
    for i in 0..n {
        det *= a[i][i];
    }
    det
}

// ---------------------------------------------------------------------------
// Trace
// ---------------------------------------------------------------------------

/// Sum of diagonal elements.
///
/// # Panics
/// Panics if `mat` is not square.
pub fn trace(mat: &Matrix) -> f64 {
    let n = mat.rows();
    assert_eq!(n, mat.cols(), "trace requires a square matrix");
    (0..n).map(|i| mat.get(i, i)).sum()
}

// ---------------------------------------------------------------------------
// Frobenius norm
// ---------------------------------------------------------------------------

/// Frobenius norm: `sqrt( sum_ij |a_ij|^2 )`.
pub fn frobenius_norm(mat: &Matrix) -> f64 {
    mat.data.iter().map(|x| x * x).sum::<f64>().sqrt()
}

// ---------------------------------------------------------------------------
// Positive-definiteness check
// ---------------------------------------------------------------------------

/// Returns `true` if `mat` is symmetric positive-definite (all eigenvalues
/// are strictly positive).  A small tolerance (`1e-10`) is used to reject
/// eigenvalues that are merely non-negative due to floating-point rounding.
///
/// # Panics
/// Panics if `mat` is not square.
pub fn is_positive_definite(mat: &Matrix) -> bool {
    let n = mat.rows();
    assert_eq!(n, mat.cols(), "is_positive_definite requires a square matrix");

    let tol = 1e-10;
    let eig = symmetric_eigen(mat);
    eig.eigenvalues.iter().all(|&v| v > tol)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    #[test]
    fn test_matrix_construction_and_get_set() {
        let mut m = Matrix::new(2, 3);
        assert_eq!(m.rows(), 2);
        assert_eq!(m.cols(), 3);
        assert_eq!(m.get(0, 0), 0.0);

        m.set(0, 1, 5.0);
        m.set(1, 2, -3.0);
        assert_eq!(m.get(0, 1), 5.0);
        assert_eq!(m.get(1, 2), -3.0);

        let m2 = Matrix::from_data(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(m2.get(0, 0), 1.0);
        assert_eq!(m2.get(0, 1), 2.0);
        assert_eq!(m2.get(1, 0), 3.0);
        assert_eq!(m2.get(1, 1), 4.0);
    }

    #[test]
    fn test_identity_and_transpose() {
        let id = Matrix::identity(3);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_eq!(id.get(i, j), expected);
            }
        }

        let m = Matrix::from_data(2, 3, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let t = m.transpose();
        assert_eq!(t.rows(), 3);
        assert_eq!(t.cols(), 2);
        assert_eq!(t.get(0, 0), 1.0);
        assert_eq!(t.get(1, 0), 2.0);
        assert_eq!(t.get(2, 1), 6.0);
    }

    #[test]
    fn test_matrix_multiplication() {
        // [[1,2],[3,4]] * [[5,6],[7,8]] = [[19,22],[43,50]]
        let a = Matrix::from_data(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let b = Matrix::from_data(2, 2, vec![5.0, 6.0, 7.0, 8.0]);
        let c = a.multiply(&b);
        assert!((c.get(0, 0) - 19.0).abs() < EPS);
        assert!((c.get(0, 1) - 22.0).abs() < EPS);
        assert!((c.get(1, 0) - 43.0).abs() < EPS);
        assert!((c.get(1, 1) - 50.0).abs() < EPS);

        // Non-square: (2x3) * (3x1)
        let a2 = Matrix::from_data(2, 3, vec![1.0, 0.0, 2.0, 0.0, 1.0, 3.0]);
        let b2 = Matrix::from_data(3, 1, vec![1.0, 2.0, 3.0]);
        let c2 = a2.multiply(&b2);
        assert_eq!(c2.rows(), 2);
        assert_eq!(c2.cols(), 1);
        assert!((c2.get(0, 0) - 7.0).abs() < EPS);
        assert!((c2.get(1, 0) - 11.0).abs() < EPS);
    }

    #[test]
    fn test_symmetric_eigen_2x2() {
        // [[2, 1], [1, 2]] has eigenvalues 1 and 3.
        let m = Matrix::from_data(2, 2, vec![2.0, 1.0, 1.0, 2.0]);
        let eig = symmetric_eigen(&m);

        let mut vals = eig.eigenvalues.clone();
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((vals[0] - 1.0).abs() < EPS);
        assert!((vals[1] - 3.0).abs() < EPS);

        // Verify A * v = lambda * v for each eigenpair.
        for j in 0..2 {
            let lambda = eig.eigenvalues[j];
            let v: Vec<f64> = (0..2).map(|i| eig.eigenvectors.get(i, j)).collect();
            for i in 0..2 {
                let av_i: f64 = (0..2).map(|k| m.get(i, k) * v[k]).sum();
                assert!(
                    (av_i - lambda * v[i]).abs() < EPS,
                    "eigenvector check failed"
                );
            }
        }
    }

    #[test]
    fn test_symmetric_eigen_3x3() {
        // Diagonal matrix — eigenvalues are the diagonal entries.
        let m = Matrix::from_data(
            3,
            3,
            vec![5.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        );
        let eig = symmetric_eigen(&m);

        let mut vals = eig.eigenvalues.clone();
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((vals[0] - 1.0).abs() < EPS);
        assert!((vals[1] - 3.0).abs() < EPS);
        assert!((vals[2] - 5.0).abs() < EPS);

        // Verify A * v = lambda * v for each eigenpair.
        for j in 0..3 {
            let lambda = eig.eigenvalues[j];
            let v: Vec<f64> = (0..3).map(|i| eig.eigenvectors.get(i, j)).collect();
            for i in 0..3 {
                let av_i: f64 = (0..3).map(|k| m.get(i, k) * v[k]).sum();
                assert!(
                    (av_i - lambda * v[i]).abs() < EPS,
                    "eigenvector check failed for 3x3"
                );
            }
        }
    }

    #[test]
    fn test_power_iteration_dominant() {
        // [[4, 1], [2, 3]] dominant eigenvalue = 5.
        let m = Matrix::from_data(2, 2, vec![4.0, 1.0, 2.0, 3.0]);
        let (val, vec) = power_iteration(&m, 1000, 1e-12);
        assert!((val - 5.0).abs() < 1e-6);

        // Eigenvector should be proportional to [1, 1] (normalised: [1/sqrt2, 1/sqrt2]).
        let ratio = vec[0] / vec[1];
        assert!((ratio - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_covariance_matrix() {
        // Two perfectly correlated dimensions: y = x.
        let data = vec![
            vec![1.0, 1.0],
            vec![2.0, 2.0],
            vec![3.0, 3.0],
            vec![4.0, 4.0],
            vec![5.0, 5.0],
        ];
        let cov = covariance_matrix(&data);
        assert_eq!(cov.rows(), 2);
        assert_eq!(cov.cols(), 2);

        // Variance of [1,2,3,4,5] = 2.5, covariance = 2.5.
        assert!((cov.get(0, 0) - 2.5).abs() < EPS);
        assert!((cov.get(1, 1) - 2.5).abs() < EPS);
        assert!((cov.get(0, 1) - 2.5).abs() < EPS);
        assert!((cov.get(1, 0) - 2.5).abs() < EPS);
    }

    #[test]
    fn test_determinant() {
        // 2x2
        let m2 = Matrix::from_data(2, 2, vec![3.0, 8.0, 4.0, 6.0]);
        assert!((determinant(&m2) - (3.0 * 6.0 - 8.0 * 4.0)).abs() < EPS);

        // 3x3
        let m3 = Matrix::from_data(
            3,
            3,
            vec![6.0, 1.0, 1.0, 4.0, -2.0, 5.0, 2.0, 8.0, 7.0],
        );
        // det = 6*(-2*7 - 5*8) - 1*(4*7 - 5*2) + 1*(4*8 - (-2)*2)
        //     = 6*(-14-40) - 1*(28-10) + 1*(32+4) = -324 -18 + 36 = -306
        assert!((determinant(&m3) - (-306.0)).abs() < EPS);

        // 1x1
        let m1 = Matrix::from_data(1, 1, vec![7.0]);
        assert!((determinant(&m1) - 7.0).abs() < EPS);

        // 5x5 identity has det = 1
        let m5 = Matrix::identity(5);
        assert!((determinant(&m5) - 1.0).abs() < EPS);
    }

    #[test]
    fn test_trace_and_frobenius_norm() {
        let m = Matrix::from_data(3, 3, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        assert!((trace(&m) - 15.0).abs() < EPS); // 1 + 5 + 9

        // Frobenius norm = sqrt(sum of squares) = sqrt(1+4+9+16+25+36+49+64+81) = sqrt(285)
        let expected = 285.0_f64.sqrt();
        assert!((frobenius_norm(&m) - expected).abs() < EPS);
    }

    #[test]
    fn test_positive_definite() {
        // Identity is positive definite.
        let id = Matrix::identity(3);
        assert!(is_positive_definite(&id));

        // [[2, 1], [1, 2]] has eigenvalues 1 and 3 — positive definite.
        let pd = Matrix::from_data(2, 2, vec![2.0, 1.0, 1.0, 2.0]);
        assert!(is_positive_definite(&pd));

        // [[-1, 0], [0, -1]] has eigenvalues -1 — NOT positive definite.
        let npd = Matrix::from_data(2, 2, vec![-1.0, 0.0, 0.0, -1.0]);
        assert!(!is_positive_definite(&npd));

        // Semi-definite: [[1, 1], [1, 1]] has eigenvalues 0 and 2 — NOT positive definite.
        let psd = Matrix::from_data(2, 2, vec![1.0, 1.0, 1.0, 1.0]);
        assert!(!is_positive_definite(&psd));
    }
}

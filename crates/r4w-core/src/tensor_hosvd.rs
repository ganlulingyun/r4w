//! Higher-Order SVD (HOSVD) tensor decomposition for multi-dimensional signal analysis.
//!
//! This module provides a [`Tensor3`] type representing 3-dimensional real-valued tensors
//! along with routines for mode-n unfolding, mode-n products, SVD via power iteration,
//! and the full HOSVD (Tucker) decomposition.
//!
//! # Example
//!
//! ```
//! use r4w_core::tensor_hosvd::{Tensor3, HosvdResult};
//!
//! // Create a small 2x3x2 tensor
//! let mut t = Tensor3::zeros(2, 3, 2);
//! t.set(0, 0, 0, 1.0);
//! t.set(0, 1, 0, 2.0);
//! t.set(1, 0, 1, 3.0);
//! t.set(1, 2, 1, 4.0);
//!
//! // Full HOSVD decomposition
//! let result = t.hosvd();
//! assert_eq!(result.core.shape(), (2, 3, 2));
//!
//! // Reconstruct and verify Frobenius norm is close
//! let reconstructed = result.reconstruct();
//! let diff = t.sub(&reconstructed);
//! assert!(diff.frobenius_norm() < 1e-6);
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// Dense matrix helper (row-major, real-valued)
// ---------------------------------------------------------------------------

/// A simple dense matrix stored in row-major order.
#[derive(Clone, Debug)]
pub struct DenseMatrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl DenseMatrix {
    /// Create a zero matrix of size `rows x cols`.
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    /// Create an identity matrix of size `n x n`.
    pub fn eye(n: usize) -> Self {
        let mut m = Self::zeros(n, n);
        for i in 0..n {
            m.data[i * n + i] = 1.0;
        }
        m
    }

    /// Dimensions `(rows, cols)`.
    pub fn shape(&self) -> (usize, usize) {
        (self.rows, self.cols)
    }

    #[inline]
    pub fn get(&self, r: usize, c: usize) -> f64 {
        self.data[r * self.cols + c]
    }

    #[inline]
    pub fn set(&mut self, r: usize, c: usize, v: f64) {
        self.data[r * self.cols + c] = v;
    }

    /// Matrix-matrix multiply `self * other`.
    pub fn matmul(&self, other: &DenseMatrix) -> DenseMatrix {
        assert_eq!(self.cols, other.rows, "matmul dimension mismatch");
        let mut out = DenseMatrix::zeros(self.rows, other.cols);
        for i in 0..self.rows {
            for k in 0..self.cols {
                let a = self.get(i, k);
                if a == 0.0 {
                    continue;
                }
                for j in 0..other.cols {
                    let cur = out.get(i, j);
                    out.set(i, j, cur + a * other.get(k, j));
                }
            }
        }
        out
    }

    /// Transpose.
    pub fn transpose(&self) -> DenseMatrix {
        let mut out = DenseMatrix::zeros(self.cols, self.rows);
        for i in 0..self.rows {
            for j in 0..self.cols {
                out.set(j, i, self.get(i, j));
            }
        }
        out
    }

    /// Frobenius norm of the matrix.
    pub fn frobenius_norm(&self) -> f64 {
        self.data.iter().map(|x| x * x).sum::<f64>().sqrt()
    }

    /// Extract column `c` as a vector.
    pub fn col(&self, c: usize) -> Vec<f64> {
        (0..self.rows).map(|r| self.get(r, c)).collect()
    }

    /// Extract submatrix: first `ncols` columns.
    pub fn truncate_cols(&self, ncols: usize) -> DenseMatrix {
        assert!(ncols <= self.cols);
        let mut out = DenseMatrix::zeros(self.rows, ncols);
        for i in 0..self.rows {
            for j in 0..ncols {
                out.set(i, j, self.get(i, j));
            }
        }
        out
    }
}

impl fmt::Display for DenseMatrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for i in 0..self.rows {
            for j in 0..self.cols {
                write!(f, "{:10.4} ", self.get(i, j))?;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// SVD via power iteration (for real matrices)
// ---------------------------------------------------------------------------

/// Result of a thin SVD: `A = U * diag(sigma) * V^T`.
#[derive(Clone, Debug)]
pub struct SvdResult {
    /// Left singular vectors (m x k), columns are vectors.
    pub u: DenseMatrix,
    /// Singular values in descending order.
    pub sigma: Vec<f64>,
    /// Right singular vectors (n x k), columns are vectors.
    pub v: DenseMatrix,
}

/// Compute the thin SVD of `a` using repeated power iteration + deflation.
///
/// Returns up to `min(rows, cols)` singular triplets.
/// `max_iter` controls iterations per singular value; `tol` convergence threshold.
pub fn svd_power(a: &DenseMatrix, max_iter: usize, tol: f64) -> SvdResult {
    let k = a.rows.min(a.cols);
    let mut u_cols: Vec<Vec<f64>> = Vec::with_capacity(k);
    let mut v_cols: Vec<Vec<f64>> = Vec::with_capacity(k);
    let mut sigmas: Vec<f64> = Vec::with_capacity(k);

    // Work on a deflated copy
    let mut residual = a.clone();

    for _ in 0..k {
        // Power iteration on A^T A to get right singular vector
        let ata = residual.transpose().matmul(&residual);
        let n = ata.rows;
        // initial vector with broken symmetry to handle repeated singular values
        let mut v_vec: Vec<f64> = (0..n).map(|i| 1.0 + 0.1 * (i as f64)).collect();
        // Orthogonalize against previously found v vectors (Gram-Schmidt)
        for prev in &v_cols {
            let dot: f64 = v_vec.iter().zip(prev.iter()).map(|(a, b)| a * b).sum();
            for (vi, pi) in v_vec.iter_mut().zip(prev.iter()) {
                *vi -= dot * pi;
            }
        }
        let norm = vec_norm(&v_vec);
        if norm > 0.0 {
            vec_scale(&mut v_vec, 1.0 / norm);
        }

        for _ in 0..max_iter {
            let mut new_v = vec![0.0; n];
            for i in 0..n {
                let mut s = 0.0;
                for j in 0..n {
                    s += ata.get(i, j) * v_vec[j];
                }
                new_v[i] = s;
            }
            let nrm = vec_norm(&new_v);
            if nrm < 1e-30 {
                break;
            }
            vec_scale(&mut new_v, 1.0 / nrm);

            // convergence check
            let dot: f64 = new_v.iter().zip(v_vec.iter()).map(|(a, b)| a * b).sum();
            v_vec = new_v;
            if (dot.abs() - 1.0).abs() < tol {
                break;
            }
        }

        // u = A * v / sigma
        let mut u_vec = vec![0.0; residual.rows];
        for i in 0..residual.rows {
            let mut s = 0.0;
            for j in 0..residual.cols {
                s += residual.get(i, j) * v_vec[j];
            }
            u_vec[i] = s;
        }
        let sigma = vec_norm(&u_vec);
        if sigma < 1e-30 {
            // Remaining singular values are effectively zero
            sigmas.push(0.0);
            u_cols.push(vec![0.0; residual.rows]);
            v_cols.push(vec![0.0; residual.cols]);
            continue;
        }
        vec_scale(&mut u_vec, 1.0 / sigma);

        // Deflate: residual -= sigma * u * v^T
        for i in 0..residual.rows {
            for j in 0..residual.cols {
                let cur = residual.get(i, j);
                residual.set(i, j, cur - sigma * u_vec[i] * v_vec[j]);
            }
        }

        sigmas.push(sigma);
        u_cols.push(u_vec);
        v_cols.push(v_vec);
    }

    // Assemble matrices
    let mut u = DenseMatrix::zeros(a.rows, k);
    let mut v = DenseMatrix::zeros(a.cols, k);
    for c in 0..k {
        for r in 0..a.rows {
            u.set(r, c, u_cols[c][r]);
        }
        for r in 0..a.cols {
            v.set(r, c, v_cols[c][r]);
        }
    }

    SvdResult {
        u,
        sigma: sigmas,
        v,
    }
}

#[inline]
fn vec_norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

#[inline]
fn vec_scale(v: &mut [f64], s: f64) {
    for x in v.iter_mut() {
        *x *= s;
    }
}

// ---------------------------------------------------------------------------
// 3-D Tensor
// ---------------------------------------------------------------------------

/// A 3-dimensional real-valued tensor stored in row-major order
/// with dimensions `(d0, d1, d2)`.
///
/// Indexing convention: `data[i * d1 * d2 + j * d2 + k]` corresponds to
/// element `(i, j, k)`.
#[derive(Clone, Debug)]
pub struct Tensor3 {
    d0: usize,
    d1: usize,
    d2: usize,
    data: Vec<f64>,
}

impl Tensor3 {
    /// Create a zero tensor of shape `(d0, d1, d2)`.
    pub fn zeros(d0: usize, d1: usize, d2: usize) -> Self {
        Self {
            d0,
            d1,
            d2,
            data: vec![0.0; d0 * d1 * d2],
        }
    }

    /// Create a tensor from a flat data vector (row-major order).
    pub fn from_vec(d0: usize, d1: usize, d2: usize, data: Vec<f64>) -> Self {
        assert_eq!(data.len(), d0 * d1 * d2, "data length must match d0*d1*d2");
        Self { d0, d1, d2, data }
    }

    /// Shape as a tuple `(d0, d1, d2)`.
    pub fn shape(&self) -> (usize, usize, usize) {
        (self.d0, self.d1, self.d2)
    }

    #[inline]
    fn idx(&self, i: usize, j: usize, k: usize) -> usize {
        i * self.d1 * self.d2 + j * self.d2 + k
    }

    /// Get element at `(i, j, k)`.
    #[inline]
    pub fn get(&self, i: usize, j: usize, k: usize) -> f64 {
        self.data[self.idx(i, j, k)]
    }

    /// Set element at `(i, j, k)`.
    #[inline]
    pub fn set(&mut self, i: usize, j: usize, k: usize, v: f64) {
        let idx = self.idx(i, j, k);
        self.data[idx] = v;
    }

    /// Frobenius norm: square root of sum of squared elements.
    pub fn frobenius_norm(&self) -> f64 {
        self.data.iter().map(|x| x * x).sum::<f64>().sqrt()
    }

    /// Element-wise subtraction: `self - other`.
    pub fn sub(&self, other: &Tensor3) -> Tensor3 {
        assert_eq!(self.shape(), other.shape(), "tensor shapes must match for subtraction");
        let data: Vec<f64> = self
            .data
            .iter()
            .zip(other.data.iter())
            .map(|(a, b)| a - b)
            .collect();
        Tensor3 {
            d0: self.d0,
            d1: self.d1,
            d2: self.d2,
            data,
        }
    }

    /// Element-wise addition: `self + other`.
    pub fn add(&self, other: &Tensor3) -> Tensor3 {
        assert_eq!(self.shape(), other.shape(), "tensor shapes must match for addition");
        let data: Vec<f64> = self
            .data
            .iter()
            .zip(other.data.iter())
            .map(|(a, b)| a + b)
            .collect();
        Tensor3 {
            d0: self.d0,
            d1: self.d1,
            d2: self.d2,
            data,
        }
    }

    /// Scale every element by `s`.
    pub fn scale(&self, s: f64) -> Tensor3 {
        let data: Vec<f64> = self.data.iter().map(|x| x * s).collect();
        Tensor3 {
            d0: self.d0,
            d1: self.d1,
            d2: self.d2,
            data,
        }
    }

    // -----------------------------------------------------------------------
    // Mode-n unfolding (matricization)
    // -----------------------------------------------------------------------

    /// Mode-0 unfolding: rows indexed by dimension 0, columns by (dim1, dim2).
    ///
    /// Result is a `d0 x (d1*d2)` matrix.
    pub fn unfold_mode0(&self) -> DenseMatrix {
        let rows = self.d0;
        let cols = self.d1 * self.d2;
        let mut m = DenseMatrix::zeros(rows, cols);
        for i in 0..self.d0 {
            for j in 0..self.d1 {
                for k in 0..self.d2 {
                    m.set(i, j * self.d2 + k, self.get(i, j, k));
                }
            }
        }
        m
    }

    /// Mode-1 unfolding: rows indexed by dimension 1, columns by (dim0, dim2).
    ///
    /// Result is a `d1 x (d0*d2)` matrix.
    pub fn unfold_mode1(&self) -> DenseMatrix {
        let rows = self.d1;
        let cols = self.d0 * self.d2;
        let mut m = DenseMatrix::zeros(rows, cols);
        for i in 0..self.d0 {
            for j in 0..self.d1 {
                for k in 0..self.d2 {
                    m.set(j, i * self.d2 + k, self.get(i, j, k));
                }
            }
        }
        m
    }

    /// Mode-2 unfolding: rows indexed by dimension 2, columns by (dim0, dim1).
    ///
    /// Result is a `d2 x (d0*d1)` matrix.
    pub fn unfold_mode2(&self) -> DenseMatrix {
        let rows = self.d2;
        let cols = self.d0 * self.d1;
        let mut m = DenseMatrix::zeros(rows, cols);
        for i in 0..self.d0 {
            for j in 0..self.d1 {
                for k in 0..self.d2 {
                    m.set(k, i * self.d1 + j, self.get(i, j, k));
                }
            }
        }
        m
    }

    /// Fold a mode-0 matrix back into a tensor of shape `(d0, d1, d2)`.
    pub fn fold_mode0(mat: &DenseMatrix, d0: usize, d1: usize, d2: usize) -> Tensor3 {
        assert_eq!(mat.rows, d0);
        assert_eq!(mat.cols, d1 * d2);
        let mut t = Tensor3::zeros(d0, d1, d2);
        for i in 0..d0 {
            for j in 0..d1 {
                for k in 0..d2 {
                    t.set(i, j, k, mat.get(i, j * d2 + k));
                }
            }
        }
        t
    }

    // -----------------------------------------------------------------------
    // Mode-n product
    // -----------------------------------------------------------------------

    /// Mode-0 product: multiply tensor along dimension 0 by matrix `u` of shape `(p, d0)`.
    ///
    /// Returns a tensor of shape `(p, d1, d2)`.
    pub fn mode0_product(&self, u: &DenseMatrix) -> Tensor3 {
        assert_eq!(u.cols, self.d0, "mode-0 product requires u.cols == d0");
        let p = u.rows;
        let mut out = Tensor3::zeros(p, self.d1, self.d2);
        for ip in 0..p {
            for j in 0..self.d1 {
                for k in 0..self.d2 {
                    let mut s = 0.0;
                    for i in 0..self.d0 {
                        s += u.get(ip, i) * self.get(i, j, k);
                    }
                    out.set(ip, j, k, s);
                }
            }
        }
        out
    }

    /// Mode-1 product: multiply tensor along dimension 1 by matrix `u` of shape `(p, d1)`.
    ///
    /// Returns a tensor of shape `(d0, p, d2)`.
    pub fn mode1_product(&self, u: &DenseMatrix) -> Tensor3 {
        assert_eq!(u.cols, self.d1, "mode-1 product requires u.cols == d1");
        let p = u.rows;
        let mut out = Tensor3::zeros(self.d0, p, self.d2);
        for i in 0..self.d0 {
            for jp in 0..p {
                for k in 0..self.d2 {
                    let mut s = 0.0;
                    for j in 0..self.d1 {
                        s += u.get(jp, j) * self.get(i, j, k);
                    }
                    out.set(i, jp, k, s);
                }
            }
        }
        out
    }

    /// Mode-2 product: multiply tensor along dimension 2 by matrix `u` of shape `(p, d2)`.
    ///
    /// Returns a tensor of shape `(d0, d1, p)`.
    pub fn mode2_product(&self, u: &DenseMatrix) -> Tensor3 {
        assert_eq!(u.cols, self.d2, "mode-2 product requires u.cols == d2");
        let p = u.rows;
        let mut out = Tensor3::zeros(self.d0, self.d1, p);
        for i in 0..self.d0 {
            for j in 0..self.d1 {
                for kp in 0..p {
                    let mut s = 0.0;
                    for k in 0..self.d2 {
                        s += u.get(kp, k) * self.get(i, j, k);
                    }
                    out.set(i, j, kp, s);
                }
            }
        }
        out
    }

    // -----------------------------------------------------------------------
    // HOSVD
    // -----------------------------------------------------------------------

    /// Perform the Higher-Order SVD (HOSVD) decomposition.
    ///
    /// Decomposes `self` as `core x_0 U0 x_1 U1 x_2 U2`, where `U0`, `U1`, `U2`
    /// are orthogonal factor matrices obtained from the SVD of each mode unfolding.
    /// The core tensor has the same shape as `self`.
    pub fn hosvd(&self) -> HosvdResult {
        let svd0 = svd_power(&self.unfold_mode0(), 500, 1e-14);
        let svd1 = svd_power(&self.unfold_mode1(), 500, 1e-14);
        let svd2 = svd_power(&self.unfold_mode2(), 500, 1e-14);

        let u0 = svd0.u;
        let u1 = svd1.u;
        let u2 = svd2.u;

        // Core = T x_0 U0^T x_1 U1^T x_2 U2^T
        let core = self
            .mode0_product(&u0.transpose())
            .mode1_product(&u1.transpose())
            .mode2_product(&u2.transpose());

        HosvdResult {
            core,
            u0,
            u1,
            u2,
            sigma0: svd0.sigma,
            sigma1: svd1.sigma,
            sigma2: svd2.sigma,
        }
    }

    /// Tucker decomposition with specified multi-linear rank `(r0, r1, r2)`.
    ///
    /// This is a truncated HOSVD: only the leading `r_n` singular vectors are
    /// kept for each mode, yielding a compact core of shape `(r0, r1, r2)`.
    pub fn tucker(&self, r0: usize, r1: usize, r2: usize) -> HosvdResult {
        let svd0 = svd_power(&self.unfold_mode0(), 500, 1e-14);
        let svd1 = svd_power(&self.unfold_mode1(), 500, 1e-14);
        let svd2 = svd_power(&self.unfold_mode2(), 500, 1e-14);

        let r0 = r0.min(svd0.u.cols);
        let r1 = r1.min(svd1.u.cols);
        let r2 = r2.min(svd2.u.cols);

        let u0 = svd0.u.truncate_cols(r0);
        let u1 = svd1.u.truncate_cols(r1);
        let u2 = svd2.u.truncate_cols(r2);

        let core = self
            .mode0_product(&u0.transpose())
            .mode1_product(&u1.transpose())
            .mode2_product(&u2.transpose());

        HosvdResult {
            core,
            u0,
            u1,
            u2,
            sigma0: svd0.sigma[..r0.min(svd0.sigma.len())].to_vec(),
            sigma1: svd1.sigma[..r1.min(svd1.sigma.len())].to_vec(),
            sigma2: svd2.sigma[..r2.min(svd2.sigma.len())].to_vec(),
        }
    }

    /// Estimate the multi-linear rank based on a relative energy threshold.
    ///
    /// For each mode, the rank is the smallest number of singular values whose
    /// squared sum exceeds `threshold` fraction of the total squared sum.
    /// `threshold` should be in `(0, 1]`, e.g., 0.99 for 99% energy.
    pub fn estimate_rank(&self, threshold: f64) -> (usize, usize, usize) {
        let svd0 = svd_power(&self.unfold_mode0(), 500, 1e-14);
        let svd1 = svd_power(&self.unfold_mode1(), 500, 1e-14);
        let svd2 = svd_power(&self.unfold_mode2(), 500, 1e-14);

        let rank_from_sigma = |sigmas: &[f64]| -> usize {
            let total: f64 = sigmas.iter().map(|s| s * s).sum();
            if total < 1e-30 {
                return 1;
            }
            let mut accum = 0.0;
            for (i, s) in sigmas.iter().enumerate() {
                accum += s * s;
                if accum / total >= threshold {
                    return i + 1;
                }
            }
            sigmas.len().max(1)
        };

        (
            rank_from_sigma(&svd0.sigma),
            rank_from_sigma(&svd1.sigma),
            rank_from_sigma(&svd2.sigma),
        )
    }
}

impl fmt::Display for Tensor3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Tensor3({}x{}x{})", self.d0, self.d1, self.d2)?;
        for i in 0..self.d0 {
            writeln!(f, "\n  slice [{},:,:]:", i)?;
            for j in 0..self.d1 {
                write!(f, "    ")?;
                for k in 0..self.d2 {
                    write!(f, "{:10.4} ", self.get(i, j, k))?;
                }
                writeln!(f)?;
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// HOSVD result
// ---------------------------------------------------------------------------

/// Result of a HOSVD or Tucker decomposition.
///
/// The original tensor can be reconstructed as:
/// `T approx core x_0 U0 x_1 U1 x_2 U2`
#[derive(Clone, Debug)]
pub struct HosvdResult {
    /// Core tensor.
    pub core: Tensor3,
    /// Factor matrix for mode 0 (shape: `d0 x r0`).
    pub u0: DenseMatrix,
    /// Factor matrix for mode 1 (shape: `d1 x r1`).
    pub u1: DenseMatrix,
    /// Factor matrix for mode 2 (shape: `d2 x r2`).
    pub u2: DenseMatrix,
    /// Singular values from mode-0 unfolding.
    pub sigma0: Vec<f64>,
    /// Singular values from mode-1 unfolding.
    pub sigma1: Vec<f64>,
    /// Singular values from mode-2 unfolding.
    pub sigma2: Vec<f64>,
}

impl HosvdResult {
    /// Reconstruct the tensor from its decomposition:
    /// `T approx core x_0 U0 x_1 U1 x_2 U2`.
    pub fn reconstruct(&self) -> Tensor3 {
        self.core
            .mode0_product(&self.u0)
            .mode1_product(&self.u1)
            .mode2_product(&self.u2)
    }

    /// Multi-linear rank of the decomposition `(r0, r1, r2)`.
    pub fn rank(&self) -> (usize, usize, usize) {
        (self.u0.cols, self.u1.cols, self.u2.cols)
    }

    /// Compute the compression ratio relative to the original tensor size.
    ///
    /// Returns the ratio of stored elements (core + factor matrices) to the
    /// original number of elements.
    pub fn compression_ratio(&self, original_shape: (usize, usize, usize)) -> f64 {
        let (d0, d1, d2) = original_shape;
        let original = (d0 * d1 * d2) as f64;
        let (r0, r1, r2) = self.rank();
        let stored = (r0 * r1 * r2 + d0 * r0 + d1 * r1 + d2 * r2) as f64;
        stored / original
    }
}

// ---------------------------------------------------------------------------
// Complex helper (f64, f64) for potential spectral analysis use
// ---------------------------------------------------------------------------

/// Multiply two complex numbers represented as `(re, im)` tuples.
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Magnitude of a complex number `(re, im)`.
pub fn complex_abs(c: (f64, f64)) -> f64 {
    (c.0 * c.0 + c.1 * c.1).sqrt()
}

/// Complex conjugate.
pub fn complex_conj(c: (f64, f64)) -> (f64, f64) {
    (c.0, -c.1)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- Tensor3 basics --

    #[test]
    fn test_tensor_zeros() {
        let t = Tensor3::zeros(3, 4, 5);
        assert_eq!(t.shape(), (3, 4, 5));
        assert!(t.frobenius_norm() == 0.0);
    }

    #[test]
    fn test_tensor_set_get() {
        let mut t = Tensor3::zeros(2, 3, 4);
        t.set(1, 2, 3, 42.0);
        assert_eq!(t.get(1, 2, 3), 42.0);
        assert_eq!(t.get(0, 0, 0), 0.0);
    }

    #[test]
    fn test_tensor_from_vec() {
        let data: Vec<f64> = (0..24).map(|i| i as f64).collect();
        let t = Tensor3::from_vec(2, 3, 4, data);
        assert_eq!(t.get(0, 0, 0), 0.0);
        assert_eq!(t.get(0, 0, 3), 3.0);
        assert_eq!(t.get(1, 2, 3), 23.0);
    }

    #[test]
    fn test_frobenius_norm() {
        let mut t = Tensor3::zeros(2, 2, 2);
        t.set(0, 0, 0, 3.0);
        t.set(1, 1, 1, 4.0);
        assert!(approx_eq(t.frobenius_norm(), 5.0, 1e-12));
    }

    #[test]
    fn test_tensor_sub_add() {
        let mut a = Tensor3::zeros(2, 2, 2);
        let mut b = Tensor3::zeros(2, 2, 2);
        a.set(0, 0, 0, 5.0);
        b.set(0, 0, 0, 3.0);
        let diff = a.sub(&b);
        assert!(approx_eq(diff.get(0, 0, 0), 2.0, 1e-12));
        let sum = a.add(&b);
        assert!(approx_eq(sum.get(0, 0, 0), 8.0, 1e-12));
    }

    #[test]
    fn test_tensor_scale() {
        let mut t = Tensor3::zeros(2, 2, 1);
        t.set(0, 0, 0, 3.0);
        t.set(1, 1, 0, 4.0);
        let s = t.scale(2.0);
        assert!(approx_eq(s.get(0, 0, 0), 6.0, 1e-12));
        assert!(approx_eq(s.get(1, 1, 0), 8.0, 1e-12));
    }

    // -- Unfolding --

    #[test]
    fn test_unfold_mode0_shape() {
        let t = Tensor3::zeros(3, 4, 5);
        let m = t.unfold_mode0();
        assert_eq!(m.shape(), (3, 20));
    }

    #[test]
    fn test_unfold_mode1_shape() {
        let t = Tensor3::zeros(3, 4, 5);
        let m = t.unfold_mode1();
        assert_eq!(m.shape(), (4, 15));
    }

    #[test]
    fn test_unfold_mode2_shape() {
        let t = Tensor3::zeros(3, 4, 5);
        let m = t.unfold_mode2();
        assert_eq!(m.shape(), (5, 12));
    }

    #[test]
    fn test_unfold_preserves_frobenius() {
        // The Frobenius norm of any unfolding equals the tensor Frobenius norm
        let data: Vec<f64> = (0..24).map(|i| (i as f64) * 0.1).collect();
        let t = Tensor3::from_vec(2, 3, 4, data);
        let f_t = t.frobenius_norm();
        let f_0 = t.unfold_mode0().frobenius_norm();
        let f_1 = t.unfold_mode1().frobenius_norm();
        let f_2 = t.unfold_mode2().frobenius_norm();
        assert!(approx_eq(f_t, f_0, 1e-10));
        assert!(approx_eq(f_t, f_1, 1e-10));
        assert!(approx_eq(f_t, f_2, 1e-10));
    }

    #[test]
    fn test_fold_unfold_roundtrip() {
        let data: Vec<f64> = (0..24).map(|i| (i as f64) + 0.5).collect();
        let t = Tensor3::from_vec(2, 3, 4, data);
        let unfolded = t.unfold_mode0();
        let refolded = Tensor3::fold_mode0(&unfolded, 2, 3, 4);
        let diff = t.sub(&refolded);
        assert!(diff.frobenius_norm() < 1e-12);
    }

    // -- Mode-n products --

    #[test]
    fn test_mode0_product_identity() {
        let data: Vec<f64> = (0..12).map(|i| i as f64).collect();
        let t = Tensor3::from_vec(2, 3, 2, data);
        let eye = DenseMatrix::eye(2);
        let result = t.mode0_product(&eye);
        let diff = t.sub(&result);
        assert!(diff.frobenius_norm() < 1e-12);
    }

    #[test]
    fn test_mode1_product_shape() {
        let t = Tensor3::zeros(3, 4, 5);
        let u = DenseMatrix::zeros(2, 4); // maps 4 -> 2
        let result = t.mode1_product(&u);
        assert_eq!(result.shape(), (3, 2, 5));
    }

    #[test]
    fn test_mode2_product_shape() {
        let t = Tensor3::zeros(3, 4, 5);
        let u = DenseMatrix::zeros(7, 5); // maps 5 -> 7
        let result = t.mode2_product(&u);
        assert_eq!(result.shape(), (3, 4, 7));
    }

    // -- SVD --

    #[test]
    fn test_svd_identity() {
        let m = DenseMatrix::eye(3);
        let svd = svd_power(&m, 200, 1e-12);
        // All singular values should be 1.0
        for s in &svd.sigma {
            assert!(approx_eq(*s, 1.0, 1e-8));
        }
    }

    #[test]
    fn test_svd_reconstruction() {
        // Build a simple matrix and check U * diag(sigma) * V^T ~ A
        let mut a = DenseMatrix::zeros(3, 2);
        a.set(0, 0, 1.0);
        a.set(0, 1, 2.0);
        a.set(1, 0, 3.0);
        a.set(1, 1, 4.0);
        a.set(2, 0, 5.0);
        a.set(2, 1, 6.0);

        let svd = svd_power(&a, 300, 1e-14);
        // Reconstruct: U * diag(sigma) * V^T
        let k = svd.sigma.len();
        let mut reconstructed = DenseMatrix::zeros(a.rows, a.cols);
        for c in 0..k {
            let s = svd.sigma[c];
            for i in 0..a.rows {
                for j in 0..a.cols {
                    let cur = reconstructed.get(i, j);
                    reconstructed.set(i, j, cur + s * svd.u.get(i, c) * svd.v.get(j, c));
                }
            }
        }
        // Check Frobenius norm of difference
        let mut diff_norm = 0.0;
        for i in 0..a.rows {
            for j in 0..a.cols {
                let d = a.get(i, j) - reconstructed.get(i, j);
                diff_norm += d * d;
            }
        }
        assert!(diff_norm.sqrt() < 1e-8, "SVD reconstruction error: {}", diff_norm.sqrt());
    }

    // -- HOSVD --

    #[test]
    fn test_hosvd_reconstruction() {
        // Create a tensor with known values
        let data: Vec<f64> = (0..24).map(|i| ((i * 7 + 3) % 13) as f64).collect();
        let t = Tensor3::from_vec(2, 3, 4, data);
        let result = t.hosvd();
        let recon = result.reconstruct();
        let diff = t.sub(&recon);
        assert!(
            diff.frobenius_norm() < 1e-4,
            "HOSVD reconstruction error: {}",
            diff.frobenius_norm()
        );
    }

    #[test]
    fn test_hosvd_core_shape() {
        let t = Tensor3::zeros(3, 4, 5);
        let result = t.hosvd();
        assert_eq!(result.core.shape(), (3, 4, 5));
    }

    // -- Tucker decomposition --

    #[test]
    fn test_tucker_rank_reduction() {
        let data: Vec<f64> = (0..60).map(|i| ((i * 11 + 5) % 17) as f64).collect();
        let t = Tensor3::from_vec(3, 4, 5, data);

        let result = t.tucker(2, 2, 2);
        assert_eq!(result.core.shape(), (2, 2, 2));
        assert_eq!(result.rank(), (2, 2, 2));

        // The approximation error should be nonnegative
        let recon = result.reconstruct();
        let err = t.sub(&recon).frobenius_norm();
        assert!(err >= 0.0);
        // Approximation should be better than original norm
        // (at least captures some energy)
        assert!(err < t.frobenius_norm());
    }

    #[test]
    fn test_tucker_full_rank_reconstruction() {
        // Tucker with full rank should reconstruct perfectly
        let data: Vec<f64> = (0..24).map(|i| ((i * 7 + 3) % 13) as f64).collect();
        let t = Tensor3::from_vec(2, 3, 4, data);
        let result = t.tucker(2, 3, 4);
        let recon = result.reconstruct();
        let diff = t.sub(&recon);
        assert!(
            diff.frobenius_norm() < 1e-4,
            "Full-rank Tucker reconstruction error: {}",
            diff.frobenius_norm()
        );
    }

    // -- Rank estimation --

    #[test]
    fn test_estimate_rank() {
        // Rank-1 tensor: outer product a (x) b (x) c
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![4.0, 5.0];
        let c = vec![6.0, 7.0, 8.0, 9.0];
        let mut t = Tensor3::zeros(3, 2, 4);
        for i in 0..3 {
            for j in 0..2 {
                for k in 0..4 {
                    t.set(i, j, k, a[i] * b[j] * c[k]);
                }
            }
        }
        let (r0, r1, r2) = t.estimate_rank(0.999);
        assert_eq!(r0, 1, "mode-0 rank of rank-1 tensor should be 1, got {}", r0);
        assert_eq!(r1, 1, "mode-1 rank of rank-1 tensor should be 1, got {}", r1);
        assert_eq!(r2, 1, "mode-2 rank of rank-1 tensor should be 1, got {}", r2);
    }

    // -- Compression ratio --

    #[test]
    fn test_compression_ratio() {
        let t = Tensor3::zeros(10, 10, 10);
        let result = t.tucker(2, 2, 2);
        let cr = result.compression_ratio((10, 10, 10));
        // stored = 2*2*2 + 10*2 + 10*2 + 10*2 = 8 + 60 = 68
        // original = 1000
        assert!(approx_eq(cr, 68.0 / 1000.0, 1e-10));
    }

    // -- Complex helpers --

    #[test]
    fn test_complex_mul() {
        let a = (3.0, 4.0);
        let b = (1.0, -2.0);
        let c = complex_mul(a, b);
        // (3+4i)(1-2i) = 3 -6i + 4i - 8i^2 = 3 - 2i + 8 = 11 - 2i
        assert!(approx_eq(c.0, 11.0, 1e-12));
        assert!(approx_eq(c.1, -2.0, 1e-12));
    }

    #[test]
    fn test_complex_abs() {
        assert!(approx_eq(complex_abs((3.0, 4.0)), 5.0, 1e-12));
    }

    #[test]
    fn test_complex_conj() {
        let c = complex_conj((3.0, 4.0));
        assert_eq!(c, (3.0, -4.0));
    }

    // -- Display --

    #[test]
    fn test_tensor_display() {
        let mut t = Tensor3::zeros(1, 2, 2);
        t.set(0, 0, 0, 1.0);
        t.set(0, 1, 1, 2.0);
        let s = format!("{}", t);
        assert!(s.contains("Tensor3(1x2x2)"));
        assert!(s.contains("1.0000"));
        assert!(s.contains("2.0000"));
    }

    #[test]
    fn test_matrix_display() {
        let mut m = DenseMatrix::zeros(2, 2);
        m.set(0, 0, 1.5);
        m.set(1, 1, 2.5);
        let s = format!("{}", m);
        assert!(s.contains("1.5000"));
        assert!(s.contains("2.5000"));
    }
}

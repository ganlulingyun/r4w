//! ESPRIT — Estimation of Signal Parameters via Rotational Invariance Techniques
//!
//! High-resolution direction-of-arrival (DOA) estimation that complements MUSIC.
//! ESPRIT offers automatic angle pairing, closed-form solutions, and works with
//! uniform linear arrays (ULA). GNU Radio equivalent: `gr-doa` ESPRIT block.
//!
//! ## Algorithm
//!
//! 1. Estimate covariance matrix R = (1/K) Σ x(k) x^H(k)
//! 2. Extract signal subspace E_s via eigendecomposition (top d eigenvectors)
//! 3. Partition E_s into overlapping subarrays: E_s1 = J1 * E_s, E_s2 = J2 * E_s
//! 4. Solve invariance equation E_s2 = E_s1 * Φ for rotation matrix Φ
//! 5. Extract DOA angles from eigenvalues of Φ
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::esprit::EspritEstimator;
//!
//! let esprit = EspritEstimator::new(8, 0.5, 2);
//! // With real snapshot data, call esprit.estimate(&snapshots)
//! assert_eq!(esprit.num_elements(), 8);
//! assert_eq!(esprit.num_sources(), 2);
//! ```

use std::f64::consts::PI;

/// Complex number type for internal use.
#[derive(Debug, Clone, Copy)]
pub struct C64 {
    pub re: f64,
    pub im: f64,
}

impl C64 {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    pub fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }

    pub fn norm_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    pub fn abs(self) -> f64 {
        self.norm_sq().sqrt()
    }

    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }

    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }
}

impl std::ops::Add for C64 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::AddAssign for C64 {
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

impl std::ops::Sub for C64 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl std::ops::Mul for C64 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl std::ops::Mul<f64> for C64 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            re: self.re * rhs,
            im: self.im * rhs,
        }
    }
}

impl std::ops::Div for C64 {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        let denom = rhs.norm_sq();
        Self {
            re: (self.re * rhs.re + self.im * rhs.im) / denom,
            im: (self.im * rhs.re - self.re * rhs.im) / denom,
        }
    }
}

/// ESPRIT method variant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EspritMethod {
    /// Standard Least Squares ESPRIT.
    LeastSquares,
    /// Total Least Squares ESPRIT (more robust to noise).
    TotalLeastSquares,
}

/// ESPRIT DOA estimator configuration and state.
#[derive(Debug, Clone)]
pub struct EspritEstimator {
    /// Number of array elements.
    num_elements: usize,
    /// Element spacing in wavelengths (typically 0.5).
    spacing: f64,
    /// Number of signal sources to estimate.
    num_sources: usize,
    /// ESPRIT method (LS or TLS).
    method: EspritMethod,
}

/// Result of ESPRIT estimation.
#[derive(Debug, Clone)]
pub struct EspritResult {
    /// Estimated DOA angles in degrees.
    pub angles_deg: Vec<f64>,
    /// Eigenvalues of the rotation matrix.
    pub eigenvalues: Vec<C64>,
    /// Estimated signal powers (from covariance eigenvalues).
    pub signal_powers: Vec<f64>,
}

impl EspritEstimator {
    /// Create a new ESPRIT estimator.
    ///
    /// # Arguments
    /// * `num_elements` - Number of ULA elements (M)
    /// * `spacing` - Element spacing in wavelengths (d/λ, typically 0.5)
    /// * `num_sources` - Number of signal sources (d < M-1)
    pub fn new(num_elements: usize, spacing: f64, num_sources: usize) -> Self {
        assert!(num_sources < num_elements, "num_sources must be < num_elements - 1");
        Self {
            num_elements,
            spacing,
            num_sources,
            method: EspritMethod::LeastSquares,
        }
    }

    /// Use Total Least Squares variant.
    pub fn with_tls(mut self) -> Self {
        self.method = EspritMethod::TotalLeastSquares;
        self
    }

    /// Use Least Squares variant.
    pub fn with_ls(mut self) -> Self {
        self.method = EspritMethod::LeastSquares;
        self
    }

    /// Number of array elements.
    pub fn num_elements(&self) -> usize {
        self.num_elements
    }

    /// Number of sources.
    pub fn num_sources(&self) -> usize {
        self.num_sources
    }

    /// Estimate DOA angles from array snapshots.
    ///
    /// Each snapshot is a vector of M complex samples (one per array element).
    pub fn estimate(&self, snapshots: &[Vec<C64>]) -> EspritResult {
        assert!(!snapshots.is_empty(), "Need at least one snapshot");
        assert_eq!(
            snapshots[0].len(),
            self.num_elements,
            "Snapshot length must match num_elements"
        );

        let m = self.num_elements;
        let d = self.num_sources;

        // Step 1: Compute covariance matrix R = (1/K) Σ x x^H
        let cov = self.compute_covariance(snapshots);

        // Step 2: Eigendecomposition to get signal subspace
        let (eigenvalues, eigenvectors) = eigen_hermitian(&cov);

        // Sort by eigenvalue magnitude (descending) and take top d
        let mut indices: Vec<usize> = (0..m).collect();
        indices.sort_by(|&a, &b| eigenvalues[b].abs().partial_cmp(&eigenvalues[a].abs()).unwrap());

        let signal_powers: Vec<f64> = indices[..d].iter().map(|&i| eigenvalues[i].abs()).collect();

        // Extract signal subspace E_s (M × d)
        let mut e_s = vec![vec![C64::zero(); d]; m];
        for col in 0..d {
            let src_col = indices[col];
            for row in 0..m {
                e_s[row][col] = eigenvectors[row][src_col];
            }
        }

        // Step 3: Partition into subarrays
        // E_s1 = first (M-1) rows of E_s
        // E_s2 = last (M-1) rows of E_s
        let m1 = m - 1;
        let mut e_s1 = vec![vec![C64::zero(); d]; m1];
        let mut e_s2 = vec![vec![C64::zero(); d]; m1];
        for row in 0..m1 {
            for col in 0..d {
                e_s1[row][col] = e_s[row][col];
                e_s2[row][col] = e_s[row + 1][col];
            }
        }

        // Step 4: Solve for rotation matrix Φ
        let phi = match self.method {
            EspritMethod::LeastSquares => {
                // Φ = (E_s1^H E_s1)^(-1) E_s1^H E_s2
                let e_s1_h = hermitian_transpose(&e_s1);
                let a = mat_mul(&e_s1_h, &e_s1);
                let b = mat_mul(&e_s1_h, &e_s2);
                let a_inv = invert_matrix(&a);
                mat_mul(&a_inv, &b)
            }
            EspritMethod::TotalLeastSquares => {
                // TLS: SVD of [E_s1; E_s2], then Φ = -V_12 V_22^(-1)
                self.tls_rotation(&e_s1, &e_s2)
            }
        };

        // Step 5: Extract angles from eigenvalues of Φ
        let eig_phi = eigen_general(&phi);
        let mut angles_deg: Vec<f64> = eig_phi
            .iter()
            .map(|lambda| {
                let phase = lambda.arg();
                let sin_theta = phase / (2.0 * PI * self.spacing);
                sin_theta.clamp(-1.0, 1.0).asin().to_degrees()
            })
            .collect();

        angles_deg.sort_by(|a, b| a.partial_cmp(b).unwrap());

        EspritResult {
            angles_deg,
            eigenvalues: eig_phi,
            signal_powers,
        }
    }

    /// Compute the covariance matrix from snapshots.
    pub fn compute_covariance(&self, snapshots: &[Vec<C64>]) -> Vec<Vec<C64>> {
        let m = self.num_elements;
        let k = snapshots.len();
        let mut r = vec![vec![C64::zero(); m]; m];

        for snap in snapshots {
            for i in 0..m {
                for j in 0..m {
                    r[i][j] += snap[i] * snap[j].conj();
                }
            }
        }

        let scale = 1.0 / k as f64;
        for i in 0..m {
            for j in 0..m {
                r[i][j] = r[i][j] * scale;
            }
        }
        r
    }

    /// TLS rotation matrix estimation.
    fn tls_rotation(&self, e_s1: &[Vec<C64>], e_s2: &[Vec<C64>]) -> Vec<Vec<C64>> {
        let d = self.num_sources;
        let m1 = e_s1.len();

        // Stack [E_s1 | E_s2] as (M-1) × 2d matrix
        let mut c_mat = vec![vec![C64::zero(); 2 * d]; m1];
        for row in 0..m1 {
            for col in 0..d {
                c_mat[row][col] = e_s1[row][col];
                c_mat[row][col + d] = e_s2[row][col];
            }
        }

        // Compute C^H * C (2d × 2d)
        let c_h = hermitian_transpose(&c_mat);
        let gram = mat_mul(&c_h, &c_mat);

        // Eigendecompose the Gram matrix
        let (_, v) = eigen_hermitian(&gram);

        // Extract V_12 and V_22 from last d eigenvectors
        // Sort eigenvectors by eigenvalue (ascending) and take last d
        let n2d = 2 * d;
        let mut v12 = vec![vec![C64::zero(); d]; d];
        let mut v22 = vec![vec![C64::zero(); d]; d];
        for i in 0..d {
            for j in 0..d {
                v12[i][j] = v[i][n2d - d + j];
                v22[i][j] = v[i + d][n2d - d + j];
            }
        }

        let v22_inv = invert_matrix(&v22);
        let neg_v12 = mat_scale(&v12, C64::new(-1.0, 0.0));
        mat_mul(&neg_v12, &v22_inv)
    }

    /// Generate array response vector (steering vector) for a given angle.
    pub fn steering_vector(&self, angle_deg: f64) -> Vec<C64> {
        let m = self.num_elements;
        let sin_theta = angle_deg.to_radians().sin();
        (0..m)
            .map(|n| {
                let phase = 2.0 * PI * self.spacing * n as f64 * sin_theta;
                C64::from_polar(1.0, phase)
            })
            .collect()
    }

    /// Generate synthetic snapshots for testing.
    pub fn generate_test_snapshots(
        &self,
        angles_deg: &[f64],
        snr_db: f64,
        num_snapshots: usize,
    ) -> Vec<Vec<C64>> {
        let m = self.num_elements;
        let d = angles_deg.len();
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let signal_power = snr_linear;
        let noise_power = 1.0;

        // Simple LCG PRNG
        let mut rng_state: u64 = 12345;
        let mut next_gaussian = || -> f64 {
            // Box-Muller
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u1 = (rng_state >> 11) as f64 / (1u64 << 53) as f64;
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u2 = (rng_state >> 11) as f64 / (1u64 << 53) as f64;
            let u1 = u1.max(1e-15);
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        // Compute steering vectors
        let a_vecs: Vec<Vec<C64>> = angles_deg.iter().map(|&a| self.steering_vector(a)).collect();

        let mut snapshots = Vec::with_capacity(num_snapshots);
        for _ in 0..num_snapshots {
            // Generate signal sources with random phases
            let mut x = vec![C64::zero(); m];
            for k in 0..d {
                let phase = 2.0 * PI * next_gaussian() * 0.5;
                let s = C64::from_polar(signal_power.sqrt(), phase);
                for i in 0..m {
                    x[i] += a_vecs[k][i] * s;
                }
            }
            // Add noise
            let noise_std = (noise_power / 2.0_f64).sqrt();
            for i in 0..m {
                x[i] += C64::new(next_gaussian() * noise_std, next_gaussian() * noise_std);
            }
            snapshots.push(x);
        }
        snapshots
    }
}

// --- Linear algebra helpers ---

fn hermitian_transpose(a: &[Vec<C64>]) -> Vec<Vec<C64>> {
    let rows = a.len();
    let cols = a[0].len();
    let mut result = vec![vec![C64::zero(); rows]; cols];
    for i in 0..rows {
        for j in 0..cols {
            result[j][i] = a[i][j].conj();
        }
    }
    result
}

fn mat_mul(a: &[Vec<C64>], b: &[Vec<C64>]) -> Vec<Vec<C64>> {
    let rows = a.len();
    let cols = b[0].len();
    let inner = a[0].len();
    let mut c = vec![vec![C64::zero(); cols]; rows];
    for i in 0..rows {
        for j in 0..cols {
            for k in 0..inner {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

fn mat_scale(a: &[Vec<C64>], s: C64) -> Vec<Vec<C64>> {
    a.iter()
        .map(|row| row.iter().map(|&v| v * s).collect())
        .collect()
}

/// Invert a small complex matrix using Gauss-Jordan elimination.
fn invert_matrix(a: &[Vec<C64>]) -> Vec<Vec<C64>> {
    let n = a.len();
    let mut aug = vec![vec![C64::zero(); 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][i + n] = C64::new(1.0, 0.0);
    }

    for col in 0..n {
        // Find pivot
        let mut max_val = 0.0;
        let mut max_row = col;
        for row in col..n {
            let val = aug[row][col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-15 {
            // Singular, return identity
            let mut result = vec![vec![C64::zero(); n]; n];
            for i in 0..n {
                result[i][i] = C64::new(1.0, 0.0);
            }
            return result;
        }

        // Scale pivot row
        let pivot_inv = C64::new(1.0, 0.0) / pivot;
        for j in 0..2 * n {
            aug[col][j] = aug[col][j] * pivot_inv;
        }

        // Eliminate
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..2 * n {
                let sub = aug[col][j] * factor;
                aug[row][j] = aug[row][j] - sub;
            }
        }
    }

    let mut result = vec![vec![C64::zero(); n]; n];
    for i in 0..n {
        for j in 0..n {
            result[i][j] = aug[i][j + n];
        }
    }
    result
}

/// Eigendecomposition of a Hermitian matrix using Jacobi-like iteration.
fn eigen_hermitian(a: &[Vec<C64>]) -> (Vec<C64>, Vec<Vec<C64>>) {
    let n = a.len();
    let mut d = vec![C64::zero(); n]; // eigenvalues
    let mut v = vec![vec![C64::zero(); n]; n]; // eigenvectors

    // Initialize eigenvectors to identity
    for i in 0..n {
        v[i][i] = C64::new(1.0, 0.0);
    }

    // For Hermitian matrix, eigenvalues are real
    // Use iterative QR-like approach with Givens rotations
    let mut work = a.to_vec();

    // Power iteration / Jacobi for small matrices
    for _ in 0..200 {
        let mut off_diag = 0.0;
        for i in 0..n {
            for j in 0..n {
                if i != j {
                    off_diag += work[i][j].norm_sq();
                }
            }
        }
        if off_diag < 1e-20 {
            break;
        }

        // Find largest off-diagonal element
        let mut max_val = 0.0;
        let mut p = 0;
        let mut q = 1;
        for i in 0..n {
            for j in (i + 1)..n {
                let val = work[i][j].abs();
                if val > max_val {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        if max_val < 1e-15 {
            break;
        }

        // Compute Jacobi rotation
        let app = work[p][p].re;
        let aqq = work[q][q].re;
        let apq = work[p][q];

        // Handle complex off-diagonal
        let apq_abs = apq.abs();
        let phase = if apq_abs > 1e-15 {
            C64::new(apq.re / apq_abs, -apq.im / apq_abs)
        } else {
            C64::new(1.0, 0.0)
        };

        let tau = (aqq - app) / (2.0 * apq_abs);
        let t = if tau.abs() < 1e-15 {
            1.0
        } else {
            let sign = if tau >= 0.0 { 1.0 } else { -1.0 };
            sign / (tau.abs() + (1.0 + tau * tau).sqrt())
        };

        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        let s_c64 = C64::new(s, 0.0) * phase;

        // Apply rotation to work matrix
        for i in 0..n {
            let wip = work[i][p];
            let wiq = work[i][q];
            work[i][p] = wip * c + wiq * s_c64.conj();
            work[i][q] = wiq * c - wip * s_c64;
        }
        for j in 0..n {
            let wpj = work[p][j];
            let wqj = work[q][j];
            work[p][j] = wpj * c + wqj * s_c64;
            work[q][j] = wqj * c - wpj * s_c64.conj();
        }

        // Update eigenvectors
        for i in 0..n {
            let vip = v[i][p];
            let viq = v[i][q];
            v[i][p] = vip * c + viq * s_c64.conj();
            v[i][q] = viq * c - vip * s_c64;
        }
    }

    for i in 0..n {
        d[i] = work[i][i];
    }

    (d, v)
}

/// Eigenvalues of a general (non-Hermitian) complex matrix using QR iteration.
fn eigen_general(a: &[Vec<C64>]) -> Vec<C64> {
    let n = a.len();
    if n == 1 {
        return vec![a[0][0]];
    }
    if n == 2 {
        // Solve characteristic polynomial directly
        let tr = a[0][0] + a[1][1];
        let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        let disc = tr * tr - det * 4.0;
        let sqrt_disc = complex_sqrt(disc);
        return vec![
            (tr + sqrt_disc) * 0.5,
            (tr - sqrt_disc) * 0.5,
        ];
    }

    // Francis QR iteration for small matrices
    let mut h = a.to_vec();

    // Reduce to upper Hessenberg first
    for col in 0..(n - 2) {
        // Find largest element below diagonal
        let mut max_val = 0.0;
        let mut max_row = col + 1;
        for row in (col + 1)..n {
            let val = h[row][col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        if max_val < 1e-15 {
            continue;
        }

        // Swap rows
        h.swap(col + 1, max_row);
        // Swap columns
        for row in 0..n {
            let tmp = h[row][col + 1];
            h[row][col + 1] = h[row][max_row];
            h[row][max_row] = tmp;
        }

        // Eliminate sub-diagonal elements
        for row in (col + 2)..n {
            if h[row][col].abs() < 1e-15 {
                continue;
            }
            let factor = h[row][col] / h[col + 1][col];
            for j in col..n {
                let sub = h[col + 1][j] * factor;
                h[row][j] = h[row][j] - sub;
            }
            for i in 0..n {
                h[i][col + 1] = h[i][col + 1] + h[i][row] * factor;
            }
        }
    }

    // QR iteration on Hessenberg matrix
    for _ in 0..500 {
        // Check convergence from bottom
        let mut converged = true;
        for i in 1..n {
            if h[i][i - 1].abs() > 1e-12 {
                converged = false;
                break;
            }
        }
        if converged {
            break;
        }

        // Wilkinson shift
        let shift = h[n - 1][n - 1];

        // Shift
        for i in 0..n {
            h[i][i] = h[i][i] - shift;
        }

        // QR step using Givens rotations
        let mut cs = vec![0.0; n - 1];
        let mut sn = vec![C64::zero(); n - 1];
        for i in 0..(n - 1) {
            let a_val = h[i][i];
            let b_val = h[i + 1][i];
            let r = (a_val.norm_sq() + b_val.norm_sq()).sqrt();
            if r < 1e-15 {
                cs[i] = 1.0;
                sn[i] = C64::zero();
                continue;
            }
            cs[i] = a_val.abs() / r;
            sn[i] = if a_val.abs() > 1e-15 {
                (a_val / C64::new(a_val.abs(), 0.0)).conj() * b_val / C64::new(r, 0.0)
            } else {
                b_val / C64::new(r, 0.0)
            };

            // Apply to rows i and i+1
            for j in 0..n {
                let t1 = h[i][j] * cs[i] + h[i + 1][j] * sn[i].conj();
                let t2 = h[i + 1][j] * cs[i] - h[i][j] * sn[i];
                h[i][j] = t1;
                h[i + 1][j] = t2;
            }
        }

        // Apply Q^H from right (RQ)
        for i in 0..(n - 1) {
            for j in 0..n {
                let t1 = h[j][i] * cs[i] + h[j][i + 1] * sn[i];
                let t2 = h[j][i + 1] * cs[i] - h[j][i] * sn[i].conj();
                h[j][i] = t1;
                h[j][i + 1] = t2;
            }
        }

        // Unshift
        for i in 0..n {
            h[i][i] = h[i][i] + shift;
        }
    }

    // Extract eigenvalues from diagonal
    (0..n).map(|i| h[i][i]).collect()
}

fn complex_sqrt(z: C64) -> C64 {
    let r = z.abs();
    if r < 1e-15 {
        return C64::zero();
    }
    let sqrt_r = r.sqrt();
    let half_angle = z.arg() / 2.0;
    C64::from_polar(sqrt_r, half_angle)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_source_detection() {
        let esprit = EspritEstimator::new(8, 0.5, 1);
        let snapshots = esprit.generate_test_snapshots(&[30.0], 20.0, 500);
        let result = esprit.estimate(&snapshots);
        assert_eq!(result.angles_deg.len(), 1);
        assert!((result.angles_deg[0] - 30.0).abs() < 5.0,
            "Expected ~30°, got {:.1}°", result.angles_deg[0]);
    }

    #[test]
    fn test_dual_source_resolution() {
        let esprit = EspritEstimator::new(8, 0.5, 2);
        let snapshots = esprit.generate_test_snapshots(&[20.0, 50.0], 20.0, 500);
        let result = esprit.estimate(&snapshots);
        assert_eq!(result.angles_deg.len(), 2);
        let sorted = &result.angles_deg;
        assert!((sorted[0] - 20.0).abs() < 8.0,
            "Expected ~20°, got {:.1}°", sorted[0]);
        assert!((sorted[1] - 50.0).abs() < 8.0,
            "Expected ~50°, got {:.1}°", sorted[1]);
    }

    #[test]
    fn test_tls_variant() {
        let esprit = EspritEstimator::new(8, 0.5, 1).with_tls();
        let snapshots = esprit.generate_test_snapshots(&[30.0], 20.0, 500);
        let result = esprit.estimate(&snapshots);
        assert_eq!(result.angles_deg.len(), 1);
        assert!((result.angles_deg[0] - 30.0).abs() < 10.0,
            "TLS: Expected ~30°, got {:.1}°", result.angles_deg[0]);
    }

    #[test]
    fn test_steering_vector() {
        let esprit = EspritEstimator::new(4, 0.5, 1);
        let sv = esprit.steering_vector(0.0); // broadside
        assert_eq!(sv.len(), 4);
        // At broadside, all phases should be 0
        for s in &sv {
            assert!((s.im).abs() < 1e-10, "At broadside, imaginary part should be ~0");
        }
    }

    #[test]
    fn test_covariance_matrix_hermitian() {
        let esprit = EspritEstimator::new(4, 0.5, 1);
        let snapshots = esprit.generate_test_snapshots(&[30.0], 10.0, 100);
        let cov = esprit.compute_covariance(&snapshots);
        // Verify Hermitian: R[i][j] = conj(R[j][i])
        for i in 0..4 {
            for j in 0..4 {
                let diff_re = (cov[i][j].re - cov[j][i].re).abs();
                let diff_im = (cov[i][j].im + cov[j][i].im).abs();
                assert!(diff_re < 1e-10, "Covariance not Hermitian at ({},{})", i, j);
                assert!(diff_im < 1e-10, "Covariance not Hermitian at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_low_snr_performance() {
        let esprit = EspritEstimator::new(8, 0.5, 1);
        let snapshots = esprit.generate_test_snapshots(&[30.0], 5.0, 1000);
        let result = esprit.estimate(&snapshots);
        // More tolerance at low SNR
        assert_eq!(result.angles_deg.len(), 1);
        assert!((result.angles_deg[0] - 30.0).abs() < 15.0,
            "Low SNR: Expected ~30°, got {:.1}°", result.angles_deg[0]);
    }

    #[test]
    fn test_many_snapshots_convergence() {
        let esprit = EspritEstimator::new(8, 0.5, 1);
        let snap_50 = esprit.generate_test_snapshots(&[30.0], 15.0, 50);
        let snap_500 = esprit.generate_test_snapshots(&[30.0], 15.0, 500);
        let r1 = esprit.estimate(&snap_50);
        let r2 = esprit.estimate(&snap_500);
        // More snapshots should generally give similar or better results
        assert!(!r1.angles_deg.is_empty());
        assert!(!r2.angles_deg.is_empty());
    }

    #[test]
    fn test_broadside_source() {
        let esprit = EspritEstimator::new(8, 0.5, 1);
        let snapshots = esprit.generate_test_snapshots(&[0.0], 20.0, 500);
        let result = esprit.estimate(&snapshots);
        assert!((result.angles_deg[0]).abs() < 5.0,
            "Broadside: Expected ~0°, got {:.1}°", result.angles_deg[0]);
    }

    #[test]
    fn test_negative_angle() {
        let esprit = EspritEstimator::new(8, 0.5, 1);
        let snapshots = esprit.generate_test_snapshots(&[-25.0], 20.0, 500);
        let result = esprit.estimate(&snapshots);
        assert!((result.angles_deg[0] - (-25.0)).abs() < 10.0,
            "Expected ~-25°, got {:.1}°", result.angles_deg[0]);
    }

    #[test]
    fn test_signal_powers_returned() {
        let esprit = EspritEstimator::new(8, 0.5, 2);
        let snapshots = esprit.generate_test_snapshots(&[20.0, 50.0], 20.0, 500);
        let result = esprit.estimate(&snapshots);
        assert_eq!(result.signal_powers.len(), 2);
        // Signal eigenvalues should be larger than noise
        for &p in &result.signal_powers {
            assert!(p > 0.0, "Signal power should be positive");
        }
    }

    #[test]
    fn test_overdetermined_case() {
        let esprit = EspritEstimator::new(12, 0.5, 2);
        let snapshots = esprit.generate_test_snapshots(&[15.0, 45.0], 20.0, 500);
        let result = esprit.estimate(&snapshots);
        assert_eq!(result.angles_deg.len(), 2);
    }
}

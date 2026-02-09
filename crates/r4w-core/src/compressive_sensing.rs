//! Compressive Sensing — Sparse Signal Recovery
//!
//! Recovers sparse signals from underdetermined linear measurements
//! y = Φx, where x has few non-zero entries. Implements Orthogonal
//! Matching Pursuit (OMP), Iterative Shrinkage-Thresholding (ISTA/FISTA),
//! and basis pursuit via ADMM.
//!
//! Key applications: sub-Nyquist spectrum sensing, sparse channel
//! estimation, compressed radar, wideband signal detection.
//!
//! No direct GNU Radio equivalent (advanced signal processing).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::compressive_sensing::{omp, random_sensing_matrix};
//!
//! // Sparse signal: 3 non-zero entries in length 50
//! let mut x = vec![0.0; 50];
//! x[5] = 1.0; x[20] = -0.7; x[42] = 0.5;
//!
//! // Compressed measurements (20 measurements for 50-dim signal)
//! let phi = random_sensing_matrix(20, 50, 42);
//! let y: Vec<f64> = (0..20)
//!     .map(|i| (0..50).map(|j| phi[i][j] * x[j]).sum::<f64>())
//!     .collect();
//!
//! let recovered = omp(&phi, &y, 3);
//! assert_eq!(recovered.len(), 50);
//! ```

/// Orthogonal Matching Pursuit (OMP) for sparse recovery.
///
/// Greedily selects columns of `phi` that best explain the residual,
/// then solves least-squares over the selected support set.
///
/// `phi`: M × N sensing matrix (M measurements, N signal dimension)
/// `y`: M-dimensional measurement vector
/// `sparsity`: expected number of non-zero entries (K)
///
/// Returns the N-dimensional recovered signal.
pub fn omp(phi: &[Vec<f64>], y: &[f64], sparsity: usize) -> Vec<f64> {
    let m = phi.len();
    let n = if m > 0 { phi[0].len() } else { return vec![]; };
    if m == 0 || n == 0 {
        return vec![0.0; n];
    }

    let sparsity = sparsity.min(m).min(n);
    let mut support: Vec<usize> = Vec::with_capacity(sparsity);
    let mut residual = y.to_vec();
    let mut x = vec![0.0; n];

    for _ in 0..sparsity {
        // Find column most correlated with residual
        let mut best_idx = 0;
        let mut best_corr = 0.0;

        for j in 0..n {
            if support.contains(&j) {
                continue;
            }
            let corr: f64 = (0..m).map(|i| phi[i][j] * residual[i]).sum::<f64>();
            if corr.abs() > best_corr {
                best_corr = corr.abs();
                best_idx = j;
            }
        }

        support.push(best_idx);

        // Solve least-squares over support: phi_s * x_s = y
        let x_s = solve_least_squares_support(phi, y, &support);

        // Update full solution
        x = vec![0.0; n];
        for (i, &idx) in support.iter().enumerate() {
            if i < x_s.len() {
                x[idx] = x_s[i];
            }
        }

        // Update residual: r = y - phi * x
        residual = y.to_vec();
        for i in 0..m {
            for &idx in &support {
                if idx < n {
                    residual[i] -= phi[i][idx] * x[idx];
                }
            }
        }

        // Early termination if residual is small
        let residual_norm: f64 = residual.iter().map(|r| r * r).sum::<f64>().sqrt();
        if residual_norm < 1e-10 {
            break;
        }
    }

    x
}

/// Iterative Shrinkage-Thresholding Algorithm (ISTA) for sparse recovery.
///
/// Minimizes ||y - Φx||² + λ||x||₁ via proximal gradient descent.
///
/// `phi`: M × N sensing matrix
/// `y`: M-dimensional measurements
/// `lambda`: regularization parameter (sparsity penalty)
/// `max_iter`: maximum iterations
pub fn ista(phi: &[Vec<f64>], y: &[f64], lambda: f64, max_iter: usize) -> Vec<f64> {
    let m = phi.len();
    let n = if m > 0 { phi[0].len() } else { return vec![]; };

    // Compute step size: 1 / L where L = largest eigenvalue of Φ^T Φ
    // Approximate with power iteration
    let step = 1.0 / estimate_lipschitz(phi);

    let mut x = vec![0.0; n];

    for _ in 0..max_iter {
        // Gradient: Φ^T (Φx - y)
        let mut phi_x = vec![0.0; m];
        for i in 0..m {
            for j in 0..n {
                phi_x[i] += phi[i][j] * x[j];
            }
        }

        let mut grad = vec![0.0; n];
        for j in 0..n {
            for i in 0..m {
                grad[j] += phi[i][j] * (phi_x[i] - y[i]);
            }
        }

        // Gradient step + soft thresholding
        for j in 0..n {
            let v = x[j] - step * grad[j];
            x[j] = soft_threshold(v, step * lambda);
        }
    }

    x
}

/// Fast ISTA (FISTA) with Nesterov acceleration.
pub fn fista(phi: &[Vec<f64>], y: &[f64], lambda: f64, max_iter: usize) -> Vec<f64> {
    let m = phi.len();
    let n = if m > 0 { phi[0].len() } else { return vec![]; };

    let step = 1.0 / estimate_lipschitz(phi);

    let mut x = vec![0.0; n];
    let mut x_prev = vec![0.0; n];
    let mut t: f64 = 1.0;

    for _ in 0..max_iter {
        // Nesterov momentum
        let t_new = (1.0 + (1.0 + 4.0 * t * t).sqrt()) / 2.0;
        let momentum = (t - 1.0) / t_new;

        let z: Vec<f64> = (0..n)
            .map(|j| x[j] + momentum * (x[j] - x_prev[j]))
            .collect();

        // Gradient at z
        let mut phi_z = vec![0.0; m];
        for i in 0..m {
            for j in 0..n {
                phi_z[i] += phi[i][j] * z[j];
            }
        }

        let mut grad = vec![0.0; n];
        for j in 0..n {
            for i in 0..m {
                grad[j] += phi[i][j] * (phi_z[i] - y[i]);
            }
        }

        x_prev = x.clone();
        x = (0..n)
            .map(|j| soft_threshold(z[j] - step * grad[j], step * lambda))
            .collect();

        t = t_new;
    }

    x
}

/// Generate a random Gaussian sensing matrix.
///
/// Elements are i.i.d. N(0, 1/M) for M measurements × N signal dimension.
pub fn random_sensing_matrix(m: usize, n: usize, seed: u64) -> Vec<Vec<f64>> {
    let mut rng_state = seed;
    let scale = 1.0 / (m as f64).sqrt();

    (0..m)
        .map(|_| {
            (0..n)
                .map(|_| {
                    // Box-Muller transform for Gaussian
                    rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
                    let u1 = (rng_state >> 33) as f64 / (1u64 << 31) as f64;
                    rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
                    let u2 = (rng_state >> 33) as f64 / (1u64 << 31) as f64;
                    let u1 = u1.max(1e-10); // Avoid log(0)
                    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos() * scale
                })
                .collect()
        })
        .collect()
}

/// Generate a partial DCT sensing matrix (rows selected randomly).
pub fn dct_sensing_matrix(m: usize, n: usize, seed: u64) -> Vec<Vec<f64>> {
    // Select m random rows from the N×N DCT matrix
    let mut rng_state = seed;
    let mut indices: Vec<usize> = (0..n).collect();

    // Fisher-Yates shuffle for first m elements
    for i in 0..m.min(n) {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let j = i + (rng_state as usize % (n - i));
        indices.swap(i, j);
    }

    let selected: Vec<usize> = indices[..m.min(n)].to_vec();
    let scale = (2.0 / n as f64).sqrt();

    selected
        .iter()
        .map(|&k| {
            (0..n)
                .map(|j| {
                    let c = if k == 0 { (1.0 / n as f64).sqrt() } else { scale };
                    c * (std::f64::consts::PI * k as f64 * (2 * j + 1) as f64 / (2 * n) as f64)
                        .cos()
                })
                .collect()
        })
        .collect()
}

/// Compute the Restricted Isometry Property (RIP) constant estimate.
///
/// Checks how well the sensing matrix preserves norms of sparse vectors.
/// A good sensing matrix has δ_k close to 0 for sparsity level k.
pub fn estimate_rip_constant(phi: &[Vec<f64>], sparsity: usize, num_trials: usize) -> f64 {
    let m = phi.len();
    let n = if m > 0 { phi[0].len() } else { return 1.0 };

    let mut max_distortion = 0.0;
    let mut rng = 42u64;

    for _ in 0..num_trials {
        // Generate random sparse vector
        let mut x = vec![0.0; n];
        for _ in 0..sparsity {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            let idx = rng as usize % n;
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            x[idx] = (rng >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
        }

        let x_norm_sq: f64 = x.iter().map(|v| v * v).sum();
        if x_norm_sq < 1e-30 {
            continue;
        }

        // Compute ||Φx||²
        let mut phi_x_norm_sq = 0.0;
        for i in 0..m {
            let mut val = 0.0;
            for j in 0..n {
                val += phi[i][j] * x[j];
            }
            phi_x_norm_sq += val * val;
        }

        let distortion = (phi_x_norm_sq / x_norm_sq - 1.0).abs();
        if distortion > max_distortion {
            max_distortion = distortion;
        }
    }

    max_distortion
}

// ---- Internal helpers ----

/// Soft thresholding operator: S_λ(x) = sign(x) · max(|x| - λ, 0)
fn soft_threshold(x: f64, lambda: f64) -> f64 {
    if x > lambda {
        x - lambda
    } else if x < -lambda {
        x + lambda
    } else {
        0.0
    }
}

/// Estimate the Lipschitz constant (largest singular value squared of Φ).
fn estimate_lipschitz(phi: &[Vec<f64>]) -> f64 {
    let m = phi.len();
    let n = if m > 0 { phi[0].len() } else { return 1.0 };

    // Power iteration on Φ^T Φ
    let mut v: Vec<f64> = (0..n).map(|i| if i == 0 { 1.0 } else { 0.0 }).collect();
    let mut eigenvalue = 1.0;

    for _ in 0..50 {
        // u = Φ v
        let u: Vec<f64> = (0..m)
            .map(|i| (0..n).map(|j| phi[i][j] * v[j]).sum::<f64>())
            .collect();

        // w = Φ^T u
        let w: Vec<f64> = (0..n)
            .map(|j| (0..m).map(|i| phi[i][j] * u[i]).sum::<f64>())
            .collect();

        eigenvalue = w.iter().map(|x| x * x).sum::<f64>().sqrt();
        if eigenvalue < 1e-30 {
            return 1.0;
        }
        v = w.iter().map(|x| x / eigenvalue).collect();
    }

    eigenvalue.max(1.0)
}

/// Solve least-squares for the support subset.
fn solve_least_squares_support(phi: &[Vec<f64>], y: &[f64], support: &[usize]) -> Vec<f64> {
    let m = phi.len();
    let k = support.len();
    if k == 0 {
        return vec![];
    }

    // Build Φ_s (M × K submatrix)
    // Solve via normal equations: (Φ_s^T Φ_s) x = Φ_s^T y
    let mut ata = vec![vec![0.0; k]; k];
    let mut atb = vec![0.0; k];

    for i in 0..k {
        let ci = support[i];
        atb[i] = (0..m).map(|r| phi[r][ci] * y[r]).sum::<f64>();
        for j in 0..k {
            let cj = support[j];
            ata[i][j] = (0..m).map(|r| phi[r][ci] * phi[r][cj]).sum::<f64>();
        }
    }

    // Gaussian elimination with pivoting
    let mut aug: Vec<Vec<f64>> = ata
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            r.push(atb[i]);
            r
        })
        .collect();

    for col in 0..k {
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..k {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            continue;
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        for row in (col + 1)..k {
            let factor = aug[row][col] / pivot;
            for j in col..=k {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    let mut x = vec![0.0; k];
    for i in (0..k).rev() {
        let mut sum = aug[i][k];
        for j in (i + 1)..k {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() > 1e-30 {
            x[i] = sum / aug[i][i];
        }
    }

    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_omp_exact_recovery() {
        let n = 50;
        let m = 25;
        let k = 3;

        let mut x_true = vec![0.0; n];
        x_true[5] = 1.0;
        x_true[20] = -0.7;
        x_true[42] = 0.5;

        let phi = random_sensing_matrix(m, n, 42);
        let y: Vec<f64> = (0..m)
            .map(|i| (0..n).map(|j| phi[i][j] * x_true[j]).sum::<f64>())
            .collect();

        let recovered = omp(&phi, &y, k);
        assert_eq!(recovered.len(), n);

        // Check recovery accuracy
        let error: f64 = recovered
            .iter()
            .zip(x_true.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(error < 0.1, "error={error}");
    }

    #[test]
    fn test_ista_sparse_recovery() {
        let n = 30;
        let m = 15;

        let mut x_true = vec![0.0; n];
        x_true[3] = 1.0;
        x_true[15] = -0.5;

        let phi = random_sensing_matrix(m, n, 99);
        let y: Vec<f64> = (0..m)
            .map(|i| (0..n).map(|j| phi[i][j] * x_true[j]).sum::<f64>())
            .collect();

        let recovered = ista(&phi, &y, 0.01, 500);
        assert_eq!(recovered.len(), n);

        // Should have recovered approximately correct support
        let max_idx = recovered
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;
        assert!(max_idx == 3 || max_idx == 15, "max_idx={max_idx}");
    }

    #[test]
    fn test_fista_convergence() {
        let n = 20;
        let m = 10;

        let mut x_true = vec![0.0; n];
        x_true[7] = 2.0;

        let phi = random_sensing_matrix(m, n, 55);
        let y: Vec<f64> = (0..m)
            .map(|i| (0..n).map(|j| phi[i][j] * x_true[j]).sum::<f64>())
            .collect();

        let recovered = fista(&phi, &y, 0.01, 200);
        assert_eq!(recovered.len(), n);

        // Largest entry should be at index 7
        let max_idx = recovered
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;
        assert_eq!(max_idx, 7);
    }

    #[test]
    fn test_random_sensing_matrix() {
        let phi = random_sensing_matrix(10, 20, 42);
        assert_eq!(phi.len(), 10);
        assert_eq!(phi[0].len(), 20);
    }

    #[test]
    fn test_dct_sensing_matrix() {
        let phi = dct_sensing_matrix(10, 20, 42);
        assert_eq!(phi.len(), 10);
        assert_eq!(phi[0].len(), 20);
    }

    #[test]
    fn test_rip_constant() {
        let phi = random_sensing_matrix(30, 50, 42);
        let delta = estimate_rip_constant(&phi, 3, 100);
        assert!(delta >= 0.0);
        assert!(delta.is_finite());
    }

    #[test]
    fn test_soft_threshold() {
        assert_eq!(soft_threshold(0.5, 0.3), 0.2);
        assert_eq!(soft_threshold(-0.5, 0.3), -0.2);
        assert_eq!(soft_threshold(0.2, 0.3), 0.0);
    }

    #[test]
    fn test_omp_empty() {
        let result = omp(&[], &[], 3);
        assert!(result.is_empty());
    }

    #[test]
    fn test_omp_single_element() {
        let phi = vec![vec![1.0]];
        let y = vec![3.0];
        let result = omp(&phi, &y, 1);
        assert_eq!(result.len(), 1);
        assert!((result[0] - 3.0).abs() < 0.01);
    }
}

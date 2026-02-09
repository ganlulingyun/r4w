//! Subspace Tracking — PAST/OPAST Algorithms
//!
//! Online tracking of the principal subspace of a signal's covariance
//! matrix. Uses the Projection Approximation Subspace Tracking (PAST)
//! algorithm and its orthogonal variant (OPAST) for adaptive DOA
//! estimation, channel tracking, and signal detection.
//!
//! Computational cost is O(N·d) per sample (vs O(N³) for batch
//! eigendecomposition), enabling real-time subspace updates.
//!
//! No direct GNU Radio equivalent (adaptive array processing).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::subspace_tracker::SubspaceTracker;
//!
//! // Track 2D subspace in 4D space
//! let mut tracker = SubspaceTracker::new(4, 2, 0.99);
//! for i in 0..100 {
//!     let x = vec![
//!         (i as f64 * 0.1).sin(),
//!         (i as f64 * 0.1).cos(),
//!         (i as f64 * 0.2).sin() * 0.1,
//!         (i as f64 * 0.2).cos() * 0.1,
//!     ];
//!     tracker.update(&x);
//! }
//! let basis = tracker.subspace();
//! assert_eq!(basis.len(), 2);
//! assert_eq!(basis[0].len(), 4);
//! ```

/// Subspace Tracking via PAST/OPAST algorithm.
///
/// Tracks the `d`-dimensional principal subspace of an `n`-dimensional
/// data stream using rank-one updates.
#[derive(Debug, Clone)]
pub struct SubspaceTracker {
    /// Data dimension (N).
    dim: usize,
    /// Subspace dimension (d).
    rank: usize,
    /// Forgetting factor (λ, typically 0.95-0.999).
    forgetting_factor: f64,
    /// Subspace basis matrix W (d vectors of length N).
    basis: Vec<Vec<f64>>,
    /// Inverse correlation approximation D (d × d diagonal).
    inv_corr: Vec<f64>,
    /// Use orthogonal variant (OPAST).
    orthogonal: bool,
    /// Number of samples processed.
    count: usize,
}

impl SubspaceTracker {
    /// Create a new subspace tracker.
    ///
    /// `dim`: data dimension (N).
    /// `rank`: subspace dimension (d ≤ N).
    /// `forgetting_factor`: exponential weighting (λ, 0.95-0.999).
    pub fn new(dim: usize, rank: usize, forgetting_factor: f64) -> Self {
        let rank = rank.min(dim).max(1);
        let ff = forgetting_factor.clamp(0.5, 1.0);

        // Initialize with random orthogonal basis
        let mut basis: Vec<Vec<f64>> = Vec::with_capacity(rank);
        let mut seed = 42u64;
        for i in 0..rank {
            let mut v: Vec<f64> = (0..dim)
                .map(|j| {
                    seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5
                })
                .collect();
            // Orthogonalize against previous vectors
            for j in 0..i {
                let dot: f64 = v.iter().zip(basis[j].iter()).map(|(a, b)| a * b).sum();
                for k in 0..dim {
                    v[k] -= dot * basis[j][k];
                }
            }
            // Normalize
            let norm: f64 = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            if norm > 1e-10 {
                for x in v.iter_mut() {
                    *x /= norm;
                }
            }
            basis.push(v);
        }

        Self {
            dim,
            rank,
            forgetting_factor: ff,
            basis,
            inv_corr: vec![1.0; rank],
            orthogonal: true,
            count: 0,
        }
    }

    /// Use standard PAST (non-orthogonal, faster).
    pub fn standard(mut self) -> Self {
        self.orthogonal = false;
        self
    }

    /// Use OPAST (orthogonal variant, more accurate).
    pub fn orthogonal(mut self) -> Self {
        self.orthogonal = true;
        self
    }

    /// Update the subspace estimate with a new data vector.
    ///
    /// `x` must have length equal to `dim`.
    pub fn update(&mut self, x: &[f64]) {
        if x.len() != self.dim {
            return;
        }
        self.count += 1;

        if self.orthogonal {
            self.opast_update(x);
        } else {
            self.past_update(x);
        }
    }

    /// PAST update (Projection Approximation Subspace Tracking).
    fn past_update(&mut self, x: &[f64]) {
        let lambda = self.forgetting_factor;

        // y = W^T · x (projection onto current subspace)
        let y: Vec<f64> = self
            .basis
            .iter()
            .map(|w| w.iter().zip(x.iter()).map(|(a, b)| a * b).sum::<f64>())
            .collect();

        // Update inverse correlation: D = (1/λ)(D - D·y·y^T·D / (λ + y^T·D·y))
        let mut dy: Vec<f64> = (0..self.rank).map(|i| self.inv_corr[i] * y[i]).collect();
        let ytdy: f64 = y.iter().zip(dy.iter()).map(|(a, b)| a * b).sum();
        let denom = lambda + ytdy;

        if denom.abs() > 1e-30 {
            for i in 0..self.rank {
                self.inv_corr[i] = (self.inv_corr[i] - dy[i] * dy[i] / denom) / lambda;
            }
        }

        // e = x - W·y (projection error)
        let mut e = x.to_vec();
        for (i, w) in self.basis.iter().enumerate() {
            for j in 0..self.dim {
                e[j] -= w[j] * y[i];
            }
        }

        // Update basis: W = W + e · (D·y)^T
        dy = (0..self.rank).map(|i| self.inv_corr[i] * y[i]).collect();
        for i in 0..self.rank {
            for j in 0..self.dim {
                self.basis[i][j] += e[j] * dy[i];
            }
        }
    }

    /// OPAST update (Orthogonal PAST).
    fn opast_update(&mut self, x: &[f64]) {
        let lambda = self.forgetting_factor;

        // y = W^T · x
        let y: Vec<f64> = self
            .basis
            .iter()
            .map(|w| w.iter().zip(x.iter()).map(|(a, b)| a * b).sum::<f64>())
            .collect();

        // Update inverse correlation
        let dy: Vec<f64> = (0..self.rank).map(|i| self.inv_corr[i] * y[i]).collect();
        let ytdy: f64 = y.iter().zip(dy.iter()).map(|(a, b)| a * b).sum();
        let denom = lambda + ytdy;

        if denom.abs() > 1e-30 {
            for i in 0..self.rank {
                self.inv_corr[i] = (self.inv_corr[i] - dy[i] * dy[i] / denom) / lambda;
            }
        }

        // h = D · y / (λ + y^T·D·y)
        let h: Vec<f64> = (0..self.rank)
            .map(|i| self.inv_corr[i] * y[i])
            .collect();

        // e = x - W · y
        let mut e = x.to_vec();
        for (i, w) in self.basis.iter().enumerate() {
            for j in 0..self.dim {
                e[j] -= w[j] * y[i];
            }
        }

        // W = W + e · h^T
        for i in 0..self.rank {
            for j in 0..self.dim {
                self.basis[i][j] += e[j] * h[i];
            }
        }

        // Orthogonalization via modified Gram-Schmidt
        for i in 0..self.rank {
            for j in 0..i {
                let dot: f64 = self.basis[i]
                    .iter()
                    .zip(self.basis[j].iter())
                    .map(|(a, b)| a * b)
                    .sum();
                for k in 0..self.dim {
                    self.basis[i][k] -= dot * self.basis[j][k];
                }
            }
            // Normalize
            let norm: f64 = self.basis[i].iter().map(|x| x * x).sum::<f64>().sqrt();
            if norm > 1e-10 {
                for x in self.basis[i].iter_mut() {
                    *x /= norm;
                }
            }
        }
    }

    /// Process a block of data vectors.
    pub fn process(&mut self, data: &[Vec<f64>]) {
        for x in data {
            self.update(x);
        }
    }

    /// Get the current subspace basis (d orthonormal vectors of dimension N).
    pub fn subspace(&self) -> &[Vec<f64>] {
        &self.basis
    }

    /// Project a vector onto the current subspace.
    pub fn project(&self, x: &[f64]) -> Vec<f64> {
        let mut proj = vec![0.0; self.dim];
        for w in &self.basis {
            let coeff: f64 = w.iter().zip(x.iter()).map(|(a, b)| a * b).sum();
            for j in 0..self.dim {
                proj[j] += coeff * w[j];
            }
        }
        proj
    }

    /// Compute the projection error (distance from subspace).
    pub fn projection_error(&self, x: &[f64]) -> f64 {
        let proj = self.project(x);
        x.iter()
            .zip(proj.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    /// Estimate the signal subspace dimension using a threshold.
    ///
    /// Processes the data and returns the number of significant
    /// eigenvalues above `threshold` times the noise floor.
    pub fn estimate_dimension(data: &[Vec<f64>], max_rank: usize, threshold: f64) -> usize {
        if data.is_empty() {
            return 0;
        }
        let dim = data[0].len();
        let max_rank = max_rank.min(dim);

        // Track with maximum rank
        let mut tracker = SubspaceTracker::new(dim, max_rank, 0.99);
        tracker.process(data);

        // Compute projection errors for each rank
        let mut significant = 0;
        let total_energy: f64 = data
            .iter()
            .map(|x| x.iter().map(|v| v * v).sum::<f64>())
            .sum::<f64>()
            / data.len() as f64;

        for r in 1..=max_rank {
            let mut sub_tracker = SubspaceTracker::new(dim, r, 0.99);
            sub_tracker.process(data);

            let proj_energy: f64 = data
                .iter()
                .map(|x| {
                    let proj = sub_tracker.project(x);
                    proj.iter().map(|v| v * v).sum::<f64>()
                })
                .sum::<f64>()
                / data.len() as f64;

            if proj_energy > threshold * total_energy {
                significant = r;
                break;
            }
        }

        significant.max(1)
    }

    /// Get the data dimension.
    pub fn dim(&self) -> usize {
        self.dim
    }

    /// Get the subspace rank.
    pub fn rank(&self) -> usize {
        self.rank
    }

    /// Get the number of samples processed.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Get the forgetting factor.
    pub fn forgetting_factor(&self) -> f64 {
        self.forgetting_factor
    }

    /// Reset the tracker to initial state.
    pub fn reset(&mut self) {
        *self = SubspaceTracker::new(self.dim, self.rank, self.forgetting_factor);
    }
}

/// Compute the angle (in degrees) between two subspaces.
///
/// Uses the principal angles between subspace bases.
pub fn subspace_angle(basis_a: &[Vec<f64>], basis_b: &[Vec<f64>]) -> f64 {
    if basis_a.is_empty() || basis_b.is_empty() {
        return 90.0;
    }
    let dim = basis_a[0].len();

    // Compute the inner product matrix: M = A^T · B
    let ra = basis_a.len();
    let rb = basis_b.len();

    let mut max_cos = 0.0;
    for i in 0..ra {
        for j in 0..rb {
            let dot: f64 = basis_a[i]
                .iter()
                .zip(basis_b[j].iter())
                .map(|(a, b)| a * b)
                .sum();
            if dot.abs() > max_cos {
                max_cos = dot.abs();
            }
        }
    }

    max_cos.clamp(0.0, 1.0).acos() * 180.0 / std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_tracking() {
        let mut tracker = SubspaceTracker::new(4, 2, 0.99);
        for i in 0..200 {
            let t = i as f64 * 0.1;
            let x = vec![t.sin(), t.cos(), t.sin() * 0.01, t.cos() * 0.01];
            tracker.update(&x);
        }
        let basis = tracker.subspace();
        assert_eq!(basis.len(), 2);
        assert_eq!(basis[0].len(), 4);
        assert_eq!(tracker.count(), 200);
    }

    #[test]
    fn test_projection() {
        let mut tracker = SubspaceTracker::new(3, 1, 0.99);
        // Feed data along [1, 0, 0] direction
        for i in 0..100 {
            let amp = (i as f64 * 0.1).sin();
            tracker.update(&[amp, 0.0, 0.0]);
        }

        // Project a vector along that direction
        let proj = tracker.project(&[1.0, 0.0, 0.0]);
        let proj_norm: f64 = proj.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(proj_norm > 0.8, "proj_norm={proj_norm}");

        // Project orthogonal vector - should have small projection
        let proj_orth = tracker.project(&[0.0, 1.0, 0.0]);
        let orth_norm: f64 = proj_orth.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(orth_norm < 0.3, "orth_norm={orth_norm}");
    }

    #[test]
    fn test_projection_error() {
        let mut tracker = SubspaceTracker::new(3, 1, 0.99);
        for i in 0..100 {
            tracker.update(&[(i as f64 * 0.1).sin(), 0.0, 0.0]);
        }

        let err_in = tracker.projection_error(&[1.0, 0.0, 0.0]);
        let err_out = tracker.projection_error(&[0.0, 1.0, 0.0]);
        assert!(err_in < err_out, "err_in={err_in}, err_out={err_out}");
    }

    #[test]
    fn test_opast_vs_past() {
        // Both should converge to similar subspace
        let data: Vec<Vec<f64>> = (0..200)
            .map(|i| {
                let t = i as f64 * 0.05;
                vec![t.sin(), t.cos(), 0.01 * (2.0 * t).sin()]
            })
            .collect();

        let mut opast = SubspaceTracker::new(3, 1, 0.99).orthogonal();
        let mut past = SubspaceTracker::new(3, 1, 0.99).standard();

        opast.process(&data);
        past.process(&data);

        // Both should capture the main direction
        let angle = subspace_angle(opast.subspace(), past.subspace());
        assert!(angle < 30.0, "angle={angle}°");
    }

    #[test]
    fn test_subspace_angle() {
        let basis_a = vec![vec![1.0, 0.0, 0.0]];
        let basis_b = vec![vec![0.0, 1.0, 0.0]];
        let angle = subspace_angle(&basis_a, &basis_b);
        assert!((angle - 90.0).abs() < 1.0, "angle={angle}");

        let basis_same = vec![vec![1.0, 0.0, 0.0]];
        let angle_same = subspace_angle(&basis_a, &basis_same);
        assert!(angle_same < 1.0, "angle_same={angle_same}");
    }

    #[test]
    fn test_reset() {
        let mut tracker = SubspaceTracker::new(4, 2, 0.99);
        for i in 0..50 {
            tracker.update(&[1.0, 0.0, 0.0, 0.0]);
        }
        assert_eq!(tracker.count(), 50);
        tracker.reset();
        assert_eq!(tracker.count(), 0);
    }

    #[test]
    fn test_forgetting_factor() {
        let tracker = SubspaceTracker::new(4, 2, 0.95);
        assert!((tracker.forgetting_factor() - 0.95).abs() < 1e-10);
    }

    #[test]
    fn test_dimensions() {
        let tracker = SubspaceTracker::new(8, 3, 0.99);
        assert_eq!(tracker.dim(), 8);
        assert_eq!(tracker.rank(), 3);
    }

    #[test]
    fn test_process_block() {
        let mut tracker = SubspaceTracker::new(3, 1, 0.99);
        let data: Vec<Vec<f64>> = (0..50)
            .map(|i| vec![(i as f64 * 0.1).sin(), 0.0, 0.0])
            .collect();
        tracker.process(&data);
        assert_eq!(tracker.count(), 50);
    }

    #[test]
    fn test_rank_clamping() {
        let tracker = SubspaceTracker::new(3, 10, 0.99);
        assert_eq!(tracker.rank(), 3); // Clamped to dim
    }
}

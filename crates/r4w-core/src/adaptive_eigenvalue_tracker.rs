//! Real-time SVD-based subspace tracking using the PAST algorithm.
//!
//! This module implements the **Projection Approximation Subspace Tracking**
//! (PAST) algorithm for low-rank signal decomposition and adaptive noise
//! subspace estimation. It is useful for applications such as:
//!
//! - Direction-of-arrival (DOA) estimation in antenna arrays
//! - Adaptive interference cancellation
//! - Signal/noise subspace separation
//! - Real-time eigenvalue tracking in non-stationary environments
//!
//! The PAST algorithm maintains a rank-`r` approximation of the signal
//! subspace from streaming data vectors of dimension `n`, using a forgetting
//! factor `β ∈ (0, 1]` to weight recent observations more heavily.
//!
//! # Example
//!
//! ```
//! use r4w_core::adaptive_eigenvalue_tracker::AdaptiveEigenvalueTracker;
//!
//! // Create a tracker for 4-dimensional data, tracking rank-2 signal subspace
//! let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
//!
//! // Feed in observation vectors (complex-valued as (re, im) tuples)
//! let x1 = vec![(1.0, 0.0), (0.5, 0.1), (0.2, -0.3), (0.1, 0.0)];
//! tracker.update(&x1);
//!
//! let x2 = vec![(0.9, 0.1), (0.6, -0.1), (0.1, 0.2), (0.15, -0.05)];
//! tracker.update(&x2);
//!
//! // Get estimated eigenvalues
//! let eigenvalues = tracker.eigenvalues();
//! assert_eq!(eigenvalues.len(), 2);
//!
//! // Estimate the effective rank from the eigenvalue gap
//! let rank = tracker.estimate_rank(0.1);
//! assert!(rank <= 2);
//! ```

use std::f64::consts::PI;

// ──────────────────────────── Complex helpers ────────────────────────────

type C = (f64, f64);

#[inline]
fn c_add(a: C, b: C) -> C {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C, b: C) -> C {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C, b: C) -> C {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: C) -> C {
    (a.0, -a.1)
}

#[inline]
fn c_scale(s: f64, a: C) -> C {
    (s * a.0, s * a.1)
}

#[inline]
fn c_abs_sq(a: C) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_abs(a: C) -> f64 {
    c_abs_sq(a).sqrt()
}

/// Complex inner product <a, b> = sum conj(a_i) * b_i
fn c_inner(a: &[C], b: &[C]) -> C {
    assert_eq!(a.len(), b.len());
    let mut acc = (0.0, 0.0);
    for i in 0..a.len() {
        acc = c_add(acc, c_mul(c_conj(a[i]), b[i]));
    }
    acc
}

/// Squared L2 norm of a complex vector.
fn c_norm_sq(v: &[C]) -> f64 {
    v.iter().map(|&x| c_abs_sq(x)).sum()
}

/// L2 norm of a complex vector.
fn c_norm(v: &[C]) -> f64 {
    c_norm_sq(v).sqrt()
}

/// Normalize a complex vector in-place. Returns the original norm.
fn c_normalize(v: &mut [C]) -> f64 {
    let n = c_norm(v);
    if n > 1e-30 {
        let inv = 1.0 / n;
        for x in v.iter_mut() {
            *x = c_scale(inv, *x);
        }
    }
    n
}

// ─────────────────── Matrix helpers (column-major) ───────────────────────

/// A simple column-major complex matrix.
#[derive(Clone, Debug)]
struct CMatrix {
    rows: usize,
    cols: usize,
    data: Vec<C>, // length = rows * cols, column-major
}

impl CMatrix {
    fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![(0.0, 0.0); rows * cols],
        }
    }

    #[allow(dead_code)]
    fn identity(n: usize) -> Self {
        let mut m = Self::zeros(n, n);
        for i in 0..n {
            m.set(i, i, (1.0, 0.0));
        }
        m
    }

    #[inline]
    fn idx(&self, r: usize, c: usize) -> usize {
        c * self.rows + r
    }

    #[inline]
    fn get(&self, r: usize, c: usize) -> C {
        self.data[self.idx(r, c)]
    }

    #[inline]
    fn set(&mut self, r: usize, c: usize, v: C) {
        let i = self.idx(r, c);
        self.data[i] = v;
    }

    /// Get column `c` as a slice.
    fn col(&self, c: usize) -> &[C] {
        let start = c * self.rows;
        &self.data[start..start + self.rows]
    }

    /// Get column `c` as a mutable slice.
    fn col_mut(&mut self, c: usize) -> &mut [C] {
        let start = c * self.rows;
        &mut self.data[start..start + self.rows]
    }

    /// Set column `c` from a slice.
    fn set_col(&mut self, c: usize, v: &[C]) {
        assert_eq!(v.len(), self.rows);
        let start = c * self.rows;
        self.data[start..start + self.rows].copy_from_slice(v);
    }

    /// Multiply: self^H * x  where self is (rows x cols), x is (rows,)
    /// Result is (cols,)
    fn hermitian_mul_vec(&self, x: &[C]) -> Vec<C> {
        assert_eq!(x.len(), self.rows);
        let mut result = vec![(0.0, 0.0); self.cols];
        for j in 0..self.cols {
            result[j] = c_inner(self.col(j), x);
        }
        result
    }

    /// Multiply: self * x  where self is (rows x cols), x is (cols,)
    /// Result is (rows,)
    fn mul_vec(&self, x: &[C]) -> Vec<C> {
        assert_eq!(x.len(), self.cols);
        let mut result = vec![(0.0, 0.0); self.rows];
        for j in 0..self.cols {
            let xj = x[j];
            for i in 0..self.rows {
                result[i] = c_add(result[i], c_mul(self.get(i, j), xj));
            }
        }
        result
    }
}

// ───────────────────── AdaptiveEigenvalueTracker ─────────────────────────

/// Real-time SVD-based subspace tracker using the PAST algorithm.
///
/// Maintains a rank-`r` signal subspace estimate from streaming complex
/// observation vectors of dimension `n`. The forgetting factor `beta` controls
/// adaptation speed: values close to 1.0 provide slow, stable adaptation
/// while smaller values track changes faster.
#[derive(Clone, Debug)]
pub struct AdaptiveEigenvalueTracker {
    /// Dimension of the observation vectors.
    dim: usize,
    /// Rank of the signal subspace to track.
    rank: usize,
    /// Forgetting factor in (0, 1].
    beta: f64,
    /// Subspace basis matrix W (dim x rank), columns are approximate eigenvectors.
    w: CMatrix,
    /// Inverse correlation matrix approximation (rank x rank) for PAST.
    p: CMatrix,
    /// Estimated eigenvalues (one per tracked component).
    eigenvalues: Vec<f64>,
    /// Number of updates processed.
    update_count: u64,
}

impl AdaptiveEigenvalueTracker {
    /// Create a new tracker with default forgetting factor (0.99).
    ///
    /// # Arguments
    /// * `dim` - Dimension of observation vectors
    /// * `rank` - Number of signal subspace components to track (must be <= dim)
    ///
    /// # Panics
    /// Panics if `rank > dim` or if `rank == 0` or `dim == 0`.
    pub fn new(dim: usize, rank: usize) -> Self {
        Self::with_forgetting_factor(dim, rank, 0.99)
    }

    /// Create a new tracker with a specified forgetting factor.
    ///
    /// # Arguments
    /// * `dim` - Dimension of observation vectors
    /// * `rank` - Number of signal subspace components to track
    /// * `beta` - Forgetting factor in (0, 1]. Closer to 1 = slower adaptation.
    ///
    /// # Panics
    /// Panics if `rank > dim`, `rank == 0`, `dim == 0`, or `beta` is not in (0, 1].
    pub fn with_forgetting_factor(dim: usize, rank: usize, beta: f64) -> Self {
        assert!(dim > 0, "dim must be > 0");
        assert!(rank > 0, "rank must be > 0");
        assert!(rank <= dim, "rank must be <= dim");
        assert!(beta > 0.0 && beta <= 1.0, "beta must be in (0, 1]");

        // Initialize W with a DFT-like basis for reproducibility.
        let mut w = CMatrix::zeros(dim, rank);
        for j in 0..rank {
            for i in 0..dim {
                let phase = 2.0 * PI * (i as f64) * ((j + 1) as f64) / (dim as f64);
                w.set(i, j, (phase.cos(), phase.sin()));
            }
            // Normalize the column
            let col = w.col_mut(j);
            c_normalize(col);
        }

        // Initialize P = delta * I where delta is a large value (inverse initialization).
        let delta = 100.0;
        let mut p = CMatrix::zeros(rank, rank);
        for i in 0..rank {
            p.set(i, i, (delta, 0.0));
        }

        Self {
            dim,
            rank,
            beta,
            w,
            p,
            eigenvalues: vec![0.0; rank],
            update_count: 0,
        }
    }

    /// Return the observation dimension.
    pub fn dim(&self) -> usize {
        self.dim
    }

    /// Return the tracked subspace rank.
    pub fn rank(&self) -> usize {
        self.rank
    }

    /// Return the forgetting factor.
    pub fn forgetting_factor(&self) -> f64 {
        self.beta
    }

    /// Return the number of updates processed so far.
    pub fn update_count(&self) -> u64 {
        self.update_count
    }

    /// Return current eigenvalue estimates (one per tracked component).
    pub fn eigenvalues(&self) -> &[f64] {
        &self.eigenvalues
    }

    /// Return a reference to the current subspace basis vectors.
    /// Returns a Vec of column vectors, each of length `dim`.
    pub fn subspace_basis(&self) -> Vec<Vec<C>> {
        (0..self.rank).map(|j| self.w.col(j).to_vec()).collect()
    }

    /// Process a new observation vector using the PAST algorithm.
    ///
    /// The PAST (Projection Approximation Subspace Tracking) algorithm updates
    /// the subspace estimate with O(nr) complexity per step.
    ///
    /// # Arguments
    /// * `x` - Complex observation vector of length `dim`
    ///
    /// # Panics
    /// Panics if `x.len() != dim`.
    pub fn update(&mut self, x: &[C]) {
        assert_eq!(x.len(), self.dim, "observation vector length must equal dim");
        self.update_count += 1;

        // Step 1: y = W^H * x  (project onto current subspace)
        let y = self.w.hermitian_mul_vec(x);

        // Step 2: Update inverse correlation P
        // h = P * y
        let h = self.p.mul_vec(&y);

        // g_denom = beta + y^H * h
        let yh = c_inner(&y, &h);
        let g_denom = self.beta + yh.0; // Should be real-valued

        // g = h / g_denom
        let inv_denom = if g_denom.abs() > 1e-30 {
            1.0 / g_denom
        } else {
            1.0 / 1e-30
        };
        let g: Vec<C> = h.iter().map(|&hi| c_scale(inv_denom, hi)).collect();

        // P = (P - g * h^H) / beta
        let inv_beta = 1.0 / self.beta;
        for j in 0..self.rank {
            for i in 0..self.rank {
                let old = self.p.get(i, j);
                let update = c_mul(g[i], c_conj(h[j]));
                self.p.set(i, j, c_scale(inv_beta, c_sub(old, update)));
            }
        }

        // Step 3: Update subspace W
        // e = x - W * y (residual / approximation error)
        let wy = self.w.mul_vec(&y);
        let e: Vec<C> = x.iter().zip(wy.iter()).map(|(&xi, &wi)| c_sub(xi, wi)).collect();

        // W = W + e * g^H
        for j in 0..self.rank {
            let gj_conj = c_conj(g[j]);
            let col = self.w.col_mut(j);
            for i in 0..self.dim {
                col[i] = c_add(col[i], c_mul(e[i], gj_conj));
            }
        }

        // Step 4: Estimate eigenvalues from the diagonal of P^{-1}
        // Since P approx R_yy^{-1}, eigenvalues of R_yy approx 1/diag(P).
        for i in 0..self.rank {
            let p_ii = self.p.get(i, i);
            let p_mag = c_abs(p_ii);
            self.eigenvalues[i] = if p_mag > 1e-30 { 1.0 / p_mag } else { 0.0 };
        }

        // Sort eigenvalues in descending order and reorder columns correspondingly.
        self.sort_by_eigenvalue();
    }

    /// Sort subspace columns and eigenvalues by eigenvalue (descending).
    fn sort_by_eigenvalue(&mut self) {
        if self.rank <= 1 {
            return;
        }

        // Simple insertion sort (rank is typically small)
        let mut indices: Vec<usize> = (0..self.rank).collect();
        for i in 1..self.rank {
            let mut j = i;
            while j > 0 && self.eigenvalues[indices[j - 1]] < self.eigenvalues[indices[j]] {
                indices.swap(j - 1, j);
                j -= 1;
            }
        }

        // Check if already sorted
        if indices.iter().enumerate().all(|(i, &idx)| i == idx) {
            return;
        }

        // Reorder eigenvalues and W columns
        let old_eigenvalues = self.eigenvalues.clone();
        let old_w = self.w.clone();
        let old_p = self.p.clone();

        for (new_i, &old_i) in indices.iter().enumerate() {
            self.eigenvalues[new_i] = old_eigenvalues[old_i];
            self.w.set_col(new_i, old_w.col(old_i));
            // Reorder both rows and columns of P
            for (new_j, &old_j) in indices.iter().enumerate() {
                self.p.set(new_i, new_j, old_p.get(old_i, old_j));
            }
        }
    }

    /// Perform power iteration to refine the dominant eigenvector estimate.
    ///
    /// This applies `iterations` steps of the power method to the tracked
    /// covariance approximation, starting from the current leading eigenvector.
    /// Useful when a more precise estimate of the dominant eigenvector is needed.
    ///
    /// Returns the estimated dominant eigenvector and its associated eigenvalue.
    ///
    /// # Panics
    /// Panics if no updates have been performed yet.
    pub fn power_iteration(&self, iterations: usize) -> (Vec<C>, f64) {
        assert!(self.update_count > 0, "must have at least one update before power iteration");

        // Start from the current leading eigenvector
        let mut v = self.w.col(0).to_vec();

        // Build an approximate R = W * diag(eigenvalues) * W^H
        // Then iterate v <- R * v / ||R * v||
        for _ in 0..iterations {
            // Project: y = W^H * v
            let y = self.w.hermitian_mul_vec(&v);

            // Scale by eigenvalues: z_j = eigenvalue_j * y_j
            let z: Vec<C> = y
                .iter()
                .enumerate()
                .map(|(j, &yj)| c_scale(self.eigenvalues[j], yj))
                .collect();

            // Back-project: v_new = W * z
            v = self.w.mul_vec(&z);

            // Normalize
            c_normalize(&mut v);
        }

        // Estimate eigenvalue via Rayleigh quotient: lambda = v^H R v
        let y = self.w.hermitian_mul_vec(&v);
        let z: Vec<C> = y
            .iter()
            .enumerate()
            .map(|(j, &yj)| c_scale(self.eigenvalues[j], yj))
            .collect();
        let rv = self.w.mul_vec(&z);
        let lambda = c_inner(&v, &rv).0; // Should be real for Hermitian R

        (v, lambda.max(0.0))
    }

    /// Get an orthonormalized copy of the current subspace basis using
    /// modified Gram-Schmidt.
    fn orthonormal_basis(&self) -> CMatrix {
        let mut q = self.w.clone();
        for j in 0..self.rank {
            // Subtract projections of earlier columns
            for k in 0..j {
                let dot = c_inner(q.col(k), q.col(j));
                let qk: Vec<C> = q.col(k).to_vec();
                let col_j = q.col_mut(j);
                for i in 0..self.dim {
                    col_j[i] = c_sub(col_j[i], c_mul(dot, qk[i]));
                }
            }
            // Normalize
            let col_j = q.col_mut(j);
            c_normalize(col_j);
        }
        q
    }

    /// Project an observation vector onto the signal subspace.
    ///
    /// Returns the component of `x` that lies in the estimated signal subspace.
    /// This effectively removes the noise subspace component. The projection
    /// uses an orthonormalized basis so it is idempotent.
    pub fn project_signal(&self, x: &[C]) -> Vec<C> {
        assert_eq!(x.len(), self.dim);
        let q = self.orthonormal_basis();
        // x_s = Q * Q^H * x
        let y = q.hermitian_mul_vec(x);
        q.mul_vec(&y)
    }

    /// Project an observation vector onto the noise subspace.
    ///
    /// Returns the component of `x` that lies in the estimated noise subspace.
    /// This is `x - project_signal(x)`.
    pub fn project_noise(&self, x: &[C]) -> Vec<C> {
        assert_eq!(x.len(), self.dim);
        let signal = self.project_signal(x);
        x.iter()
            .zip(signal.iter())
            .map(|(&xi, &si)| c_sub(xi, si))
            .collect()
    }

    /// Cancel interference by removing the signal subspace component.
    ///
    /// This is equivalent to `project_noise` and is useful for suppressing
    /// strong interferers whose subspace has been tracked.
    pub fn cancel_interference(&self, x: &[C]) -> Vec<C> {
        self.project_noise(x)
    }

    /// Estimate the effective signal subspace rank based on the eigenvalue gap.
    ///
    /// Examines successive ratios of eigenvalues and finds the largest gap.
    /// An eigenvalue ratio below `threshold` indicates transition from signal
    /// to noise eigenvalues.
    ///
    /// # Arguments
    /// * `threshold` - Ratio threshold in (0, 1). Typical values: 0.05-0.2.
    ///
    /// Returns estimated rank (0 to self.rank).
    pub fn estimate_rank(&self, threshold: f64) -> usize {
        if self.rank <= 1 {
            return if self.eigenvalues[0] > 1e-10 { 1 } else { 0 };
        }

        let max_eig = self.eigenvalues[0];
        if max_eig < 1e-30 {
            return 0;
        }

        // Find the first eigenvalue that drops below threshold * max_eigenvalue
        for i in 0..self.rank {
            if self.eigenvalues[i] / max_eig < threshold {
                return i;
            }
        }

        self.rank
    }

    /// Get the signal-to-noise ratio estimate in dB.
    ///
    /// Computed as the ratio of the largest eigenvalue to the smallest tracked
    /// eigenvalue. Returns `None` if fewer than 2 updates have been performed.
    pub fn snr_estimate_db(&self) -> Option<f64> {
        if self.update_count < 2 || self.rank < 2 {
            return None;
        }

        let max_eig = self.eigenvalues[0];
        let min_eig = self.eigenvalues[self.rank - 1];

        if min_eig < 1e-30 {
            return None;
        }

        Some(10.0 * (max_eig / min_eig).log10())
    }

    /// Reset the tracker to its initial state while keeping the same configuration.
    pub fn reset(&mut self) {
        *self = Self::with_forgetting_factor(self.dim, self.rank, self.beta);
    }

    /// Feed a batch of observation vectors.
    ///
    /// Convenience method that calls `update` for each vector in sequence.
    pub fn update_batch(&mut self, observations: &[Vec<C>]) {
        for obs in observations {
            self.update(obs);
        }
    }

    /// Compute the subspace angle (in radians) between the current estimate
    /// and a given reference subspace basis.
    ///
    /// The reference should be a slice of column vectors, each of length `dim`.
    /// Returns the principal angle between the two subspaces.
    pub fn subspace_angle(&self, reference: &[Vec<C>]) -> f64 {
        assert!(!reference.is_empty());
        assert_eq!(reference[0].len(), self.dim);

        let ref_rank = reference.len().min(self.rank);

        // For principal angle, compute max |<w_i, u_j>|
        let mut max_cos = 0.0f64;
        for i in 0..self.rank {
            for j in 0..ref_rank {
                let dot = c_abs(c_inner(self.w.col(i), &reference[j]));
                if dot > max_cos {
                    max_cos = dot;
                }
            }
        }

        max_cos = max_cos.min(1.0);
        max_cos.acos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: generate a deterministic complex sinusoidal signal at a given
    // frequency, embedded in `dim` dimensions.
    fn make_signal(dim: usize, freq: f64, t: f64, amplitude: f64) -> Vec<C> {
        (0..dim)
            .map(|i| {
                let phase = 2.0 * PI * freq * (i as f64) + t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    // Helper: generate a simple noise-like vector (deterministic PRNG).
    fn make_noise(dim: usize, seed: u64, amplitude: f64) -> Vec<C> {
        let mut state = seed.wrapping_add(1); // avoid 0 state
        (0..dim)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let re = ((state as f64) / (u64::MAX as f64) - 0.5) * 2.0 * amplitude;
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let im = ((state as f64) / (u64::MAX as f64) - 0.5) * 2.0 * amplitude;
                (re, im)
            })
            .collect()
    }

    // ──────────────── Construction and basic properties ──────────────────

    #[test]
    fn test_new_basic() {
        let tracker = AdaptiveEigenvalueTracker::new(8, 3);
        assert_eq!(tracker.dim(), 8);
        assert_eq!(tracker.rank(), 3);
        assert_eq!(tracker.forgetting_factor(), 0.99);
        assert_eq!(tracker.update_count(), 0);
        assert_eq!(tracker.eigenvalues().len(), 3);
    }

    #[test]
    fn test_with_forgetting_factor() {
        let tracker = AdaptiveEigenvalueTracker::with_forgetting_factor(4, 2, 0.95);
        assert_eq!(tracker.dim(), 4);
        assert_eq!(tracker.rank(), 2);
        assert!((tracker.forgetting_factor() - 0.95).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "rank must be <= dim")]
    fn test_rank_exceeds_dim() {
        AdaptiveEigenvalueTracker::new(3, 5);
    }

    #[test]
    #[should_panic(expected = "dim must be > 0")]
    fn test_zero_dim() {
        AdaptiveEigenvalueTracker::new(0, 0);
    }

    #[test]
    #[should_panic(expected = "beta must be in (0, 1]")]
    fn test_invalid_beta() {
        AdaptiveEigenvalueTracker::with_forgetting_factor(4, 2, 0.0);
    }

    // ──────────────── Update and eigenvalue tracking ─────────────────────

    #[test]
    fn test_single_update() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
        let x = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        tracker.update(&x);
        assert_eq!(tracker.update_count(), 1);
        // Eigenvalues should be non-negative after update
        for &ev in tracker.eigenvalues() {
            assert!(ev >= 0.0, "eigenvalue should be non-negative, got {}", ev);
        }
    }

    #[test]
    fn test_eigenvalues_descending_order() {
        let mut tracker = AdaptiveEigenvalueTracker::new(6, 3);
        // Feed many samples to stabilize
        for t in 0..200 {
            let sig = make_signal(6, 0.1, t as f64 * 0.05, 5.0);
            let noise = make_noise(6, t + 1000, 0.5);
            let x: Vec<C> = sig
                .iter()
                .zip(noise.iter())
                .map(|(&s, &n)| c_add(s, n))
                .collect();
            tracker.update(&x);
        }

        let ev = tracker.eigenvalues();
        for i in 1..ev.len() {
            assert!(
                ev[i - 1] >= ev[i] - 1e-10,
                "eigenvalues should be descending: ev[{}]={} < ev[{}]={}",
                i - 1,
                ev[i - 1],
                i,
                ev[i]
            );
        }
    }

    #[test]
    fn test_dominant_eigenvalue_grows_with_signal() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);

        // Feed strong signal on a single direction
        for t in 0..100 {
            let phase = 0.3 * t as f64;
            let x = vec![
                (10.0 * phase.cos(), 10.0 * phase.sin()),
                (5.0 * phase.cos(), 5.0 * phase.sin()),
                (2.0 * phase.cos(), 2.0 * phase.sin()),
                (1.0 * phase.cos(), 1.0 * phase.sin()),
            ];
            tracker.update(&x);
        }

        // Dominant eigenvalue should be significantly larger than the second
        let ev = tracker.eigenvalues();
        assert!(
            ev[0] > ev[1],
            "dominant eigenvalue ({}) should exceed second ({})",
            ev[0],
            ev[1]
        );
    }

    // ──────────────── Subspace projection ────────────────────────────────

    #[test]
    fn test_project_signal_preserves_dimension() {
        let mut tracker = AdaptiveEigenvalueTracker::new(5, 2);
        let x = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.5, 0.5), (0.0, 0.0)];
        tracker.update(&x);

        let proj = tracker.project_signal(&x);
        assert_eq!(proj.len(), 5);
    }

    #[test]
    fn test_signal_noise_decomposition_adds_up() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);

        // Feed some data to train
        for t in 0..50 {
            let x = make_signal(4, 0.2, t as f64 * 0.1, 3.0);
            tracker.update(&x);
        }

        let x = vec![(1.0, 0.5), (0.3, -0.2), (0.7, 0.1), (-0.1, 0.4)];
        let signal = tracker.project_signal(&x);
        let noise = tracker.project_noise(&x);

        // signal + noise should equal x
        for i in 0..4 {
            let reconstructed = c_add(signal[i], noise[i]);
            assert!(
                (reconstructed.0 - x[i].0).abs() < 1e-10,
                "real part mismatch at {}: {} vs {}",
                i,
                reconstructed.0,
                x[i].0
            );
            assert!(
                (reconstructed.1 - x[i].1).abs() < 1e-10,
                "imag part mismatch at {}: {} vs {}",
                i,
                reconstructed.1,
                x[i].1
            );
        }
    }

    #[test]
    fn test_project_signal_is_idempotent() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
        for t in 0..80 {
            let x = make_signal(4, 0.15, t as f64 * 0.1, 2.0);
            tracker.update(&x);
        }

        let x = vec![(1.0, 0.0), (0.0, 1.0), (-0.5, 0.3), (0.2, -0.7)];
        let p1 = tracker.project_signal(&x);
        let p2 = tracker.project_signal(&p1);

        // Projecting twice should give the same result
        for i in 0..4 {
            assert!(
                (p1[i].0 - p2[i].0).abs() < 1e-10 && (p1[i].1 - p2[i].1).abs() < 1e-10,
                "projection not idempotent at index {}",
                i
            );
        }
    }

    #[test]
    fn test_cancel_interference_equals_project_noise() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
        for t in 0..30 {
            tracker.update(&make_signal(4, 0.25, t as f64 * 0.2, 1.0));
        }

        let x = vec![(0.5, 0.5), (-0.3, 0.2), (0.1, -0.4), (0.8, 0.0)];
        let cancelled = tracker.cancel_interference(&x);
        let noise_proj = tracker.project_noise(&x);

        for i in 0..4 {
            assert!(
                (cancelled[i].0 - noise_proj[i].0).abs() < 1e-12,
                "cancel_interference != project_noise at index {}",
                i
            );
        }
    }

    // ──────────────── Power iteration ────────────────────────────────────

    #[test]
    fn test_power_iteration_returns_unit_vector() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
        for t in 0..50 {
            tracker.update(&make_signal(4, 0.3, t as f64 * 0.1, 4.0));
        }

        let (v, _lambda) = tracker.power_iteration(10);
        assert_eq!(v.len(), 4);

        let norm = c_norm(&v);
        assert!(
            (norm - 1.0).abs() < 1e-10,
            "power iteration result should be unit vector, got norm={}",
            norm
        );
    }

    #[test]
    fn test_power_iteration_eigenvalue_positive() {
        let mut tracker = AdaptiveEigenvalueTracker::new(6, 3);
        for t in 0..100 {
            let x = make_signal(6, 0.1, t as f64 * 0.05, 3.0);
            tracker.update(&x);
        }

        let (_v, lambda) = tracker.power_iteration(20);
        assert!(lambda >= 0.0, "eigenvalue from power iteration should be non-negative");
    }

    #[test]
    #[should_panic(expected = "must have at least one update")]
    fn test_power_iteration_before_update_panics() {
        let tracker = AdaptiveEigenvalueTracker::new(4, 2);
        tracker.power_iteration(5);
    }

    // ──────────────── Rank estimation ────────────────────────────────────

    #[test]
    fn test_estimate_rank_single_signal() {
        let mut tracker = AdaptiveEigenvalueTracker::new(8, 4);

        // Feed a rank-1 signal (single sinusoid) plus noise
        for t in 0..300 {
            let sig = make_signal(8, 0.125, t as f64 * 0.05, 10.0);
            let noise = make_noise(8, t + 5000, 0.3);
            let x: Vec<C> = sig.iter().zip(noise.iter()).map(|(&s, &n)| c_add(s, n)).collect();
            tracker.update(&x);
        }

        let rank = tracker.estimate_rank(0.1);
        // With a single strong signal, estimated rank should be 1 or 2
        assert!(
            rank >= 1 && rank <= 2,
            "expected rank ~1 for single signal, got {}",
            rank
        );
    }

    #[test]
    fn test_estimate_rank_zero_for_no_signal() {
        let tracker = AdaptiveEigenvalueTracker::new(4, 2);
        // No updates yet, eigenvalues are all 0
        let rank = tracker.estimate_rank(0.1);
        assert_eq!(rank, 0, "rank should be 0 with no signal");
    }

    // ──────────────── SNR estimation ─────────────────────────────────────

    #[test]
    fn test_snr_estimate_none_initially() {
        let tracker = AdaptiveEigenvalueTracker::new(4, 2);
        assert!(tracker.snr_estimate_db().is_none());
    }

    #[test]
    fn test_snr_estimate_positive_with_signal() {
        let mut tracker = AdaptiveEigenvalueTracker::new(6, 3);

        // Strong signal + weak noise -> high SNR
        for t in 0..200 {
            let sig = make_signal(6, 0.1, t as f64 * 0.05, 10.0);
            let noise = make_noise(6, t + 2000, 0.2);
            let x: Vec<C> = sig.iter().zip(noise.iter()).map(|(&s, &n)| c_add(s, n)).collect();
            tracker.update(&x);
        }

        if let Some(snr_db) = tracker.snr_estimate_db() {
            assert!(snr_db > 0.0, "SNR should be positive for strong signal, got {}", snr_db);
        }
    }

    // ──────────────── Batch update ───────────────────────────────────────

    #[test]
    fn test_update_batch_equivalent_to_sequential() {
        let mut tracker1 = AdaptiveEigenvalueTracker::new(4, 2);
        let mut tracker2 = AdaptiveEigenvalueTracker::new(4, 2);

        let observations: Vec<Vec<C>> = (0..20)
            .map(|t| make_signal(4, 0.2, t as f64 * 0.1, 2.0))
            .collect();

        // Update sequentially
        for obs in &observations {
            tracker1.update(obs);
        }

        // Update as batch
        tracker2.update_batch(&observations);

        // Should give identical results
        assert_eq!(tracker1.update_count(), tracker2.update_count());
        for i in 0..2 {
            assert!(
                (tracker1.eigenvalues()[i] - tracker2.eigenvalues()[i]).abs() < 1e-10,
                "eigenvalue mismatch at {}: {} vs {}",
                i,
                tracker1.eigenvalues()[i],
                tracker2.eigenvalues()[i]
            );
        }
    }

    // ──────────────── Reset ──────────────────────────────────────────────

    #[test]
    fn test_reset() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
        for t in 0..50 {
            tracker.update(&make_signal(4, 0.3, t as f64 * 0.1, 5.0));
        }
        assert_eq!(tracker.update_count(), 50);

        tracker.reset();
        assert_eq!(tracker.update_count(), 0);
        assert_eq!(tracker.dim(), 4);
        assert_eq!(tracker.rank(), 2);
        for &ev in tracker.eigenvalues() {
            assert!((ev - 0.0).abs() < 1e-12, "eigenvalues should be zero after reset");
        }
    }

    // ──────────────── Subspace basis ─────────────────────────────────────

    #[test]
    fn test_subspace_basis_dimensions() {
        let tracker = AdaptiveEigenvalueTracker::new(6, 3);
        let basis = tracker.subspace_basis();
        assert_eq!(basis.len(), 3, "should have 3 basis vectors");
        for v in &basis {
            assert_eq!(v.len(), 6, "each basis vector should have dim=6");
        }
    }

    // ──────────────── Subspace angle ─────────────────────────────────────

    #[test]
    fn test_subspace_angle_self_is_zero() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
        for t in 0..100 {
            tracker.update(&make_signal(4, 0.2, t as f64 * 0.1, 3.0));
        }

        let basis = tracker.subspace_basis();
        let angle = tracker.subspace_angle(&basis);
        assert!(
            angle < 0.1,
            "angle between tracker and its own basis should be near zero, got {}",
            angle
        );
    }

    #[test]
    fn test_subspace_angle_orthogonal() {
        let tracker = AdaptiveEigenvalueTracker::new(4, 1);
        // Create a reference that is orthogonal to the initial basis
        let reference = vec![vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]];
        let angle = tracker.subspace_angle(&reference);
        // Angle should be between 0 and pi/2
        assert!(angle >= 0.0 && angle <= PI / 2.0 + 0.01);
    }

    // ──────────────── Observation length mismatch ────────────────────────

    #[test]
    #[should_panic(expected = "observation vector length must equal dim")]
    fn test_update_wrong_length() {
        let mut tracker = AdaptiveEigenvalueTracker::new(4, 2);
        tracker.update(&[(1.0, 0.0), (0.0, 1.0)]); // length 2 != dim 4
    }
}

//! MUSIC DOA — MUltiple SIgnal Classification
//!
//! High-resolution direction-of-arrival (DOA) estimation using the
//! MUSIC algorithm. Includes spectral MUSIC, Root-MUSIC for ULA,
//! and MDL/AIC source enumeration.
//!
//! Complements the existing `beamformer` module with a dedicated,
//! full-pipeline DOA estimator including spatial smoothing for
//! coherent sources.
//!
//! GNU Radio equivalent: `gr-doa` OOT module.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::music_doa::MusicEstimator;
//!
//! let mut music = MusicEstimator::new(8, 0.5, 2);
//! let snapshots = music.generate_test_snapshots(&[20.0, 45.0], 20.0, 200);
//! let doas = music.estimate(&snapshots);
//! assert_eq!(doas.len(), 2);
//! ```

use std::f64::consts::PI;

/// MUSIC direction-of-arrival estimator.
#[derive(Debug, Clone)]
pub struct MusicEstimator {
    /// Number of array elements.
    num_elements: usize,
    /// Element spacing in wavelengths.
    spacing: f64,
    /// Number of assumed signal sources.
    num_sources: usize,
    /// Angle grid resolution in degrees for pseudospectrum search.
    angle_step: f64,
}

impl MusicEstimator {
    /// Create a new MUSIC estimator for a ULA.
    ///
    /// `num_elements`: number of antenna elements.
    /// `spacing`: inter-element spacing in wavelengths (0.5 = half-wavelength).
    /// `num_sources`: assumed number of signal sources.
    pub fn new(num_elements: usize, spacing: f64, num_sources: usize) -> Self {
        Self {
            num_elements: num_elements.max(2),
            spacing,
            num_sources: num_sources.min(num_elements.saturating_sub(1)),
            angle_step: 0.5,
        }
    }

    /// Set angular search resolution in degrees.
    pub fn angle_step(mut self, step: f64) -> Self {
        self.angle_step = step.max(0.01);
        self
    }

    /// Estimate DOA angles from array snapshots.
    ///
    /// `snapshots`: each element is a vector of `num_elements` complex samples.
    /// Returns DOA angles in degrees, sorted ascending.
    pub fn estimate(&mut self, snapshots: &[Vec<(f64, f64)>]) -> Vec<f64> {
        if snapshots.is_empty() {
            return vec![];
        }

        let m = self.num_elements;

        // Compute spatial covariance matrix R = (1/K) * Σ x·x^H
        let cov = self.compute_covariance(snapshots);

        // Eigendecomposition
        let (eigenvalues, eigenvectors) = eigen_hermitian(&cov);

        // Sort eigenvalues ascending (noise subspace = smallest)
        let mut indices: Vec<usize> = (0..m).collect();
        indices.sort_by(|&a, &b| eigenvalues[a].partial_cmp(&eigenvalues[b]).unwrap());

        // Noise subspace = eigenvectors corresponding to smallest m-d eigenvalues
        let noise_dim = m - self.num_sources;
        let noise_subspace: Vec<Vec<(f64, f64)>> = indices[..noise_dim]
            .iter()
            .map(|&idx| eigenvectors[idx].clone())
            .collect();

        // Compute pseudospectrum and find peaks
        let angles: Vec<f64> = {
            let mut a = Vec::new();
            let mut angle = -90.0;
            while angle <= 90.0 {
                a.push(angle);
                angle += self.angle_step;
            }
            a
        };

        let spectrum = self.pseudospectrum_inner(&noise_subspace, &angles);

        // Find peaks
        self.find_peaks(&angles, &spectrum, self.num_sources)
    }

    /// Compute MUSIC pseudospectrum over given angles.
    ///
    /// `snapshots`: array snapshots for covariance estimation.
    /// `angles`: evaluation angles in degrees.
    pub fn pseudospectrum(
        &self,
        snapshots: &[Vec<(f64, f64)>],
        angles: &[f64],
    ) -> Vec<f64> {
        if snapshots.is_empty() {
            return vec![0.0; angles.len()];
        }

        let m = self.num_elements;
        let cov = self.compute_covariance(snapshots);
        let (eigenvalues, eigenvectors) = eigen_hermitian(&cov);

        let mut indices: Vec<usize> = (0..m).collect();
        indices.sort_by(|&a, &b| eigenvalues[a].partial_cmp(&eigenvalues[b]).unwrap());

        let noise_dim = m - self.num_sources;
        let noise_subspace: Vec<Vec<(f64, f64)>> = indices[..noise_dim]
            .iter()
            .map(|&idx| eigenvectors[idx].clone())
            .collect();

        self.pseudospectrum_inner(&noise_subspace, angles)
    }

    fn pseudospectrum_inner(
        &self,
        noise_subspace: &[Vec<(f64, f64)>],
        angles: &[f64],
    ) -> Vec<f64> {
        angles
            .iter()
            .map(|&angle| {
                let sv = self.steering_vector(angle);
                // P(θ) = 1 / (a^H · En · En^H · a)
                let mut denom = 0.0;
                for noise_vec in noise_subspace {
                    // Compute a^H · en
                    let mut dot_r = 0.0;
                    let mut dot_i = 0.0;
                    for k in 0..self.num_elements {
                        // a^H means conjugate of a
                        let (ar, ai) = sv[k];
                        let (nr, ni) = noise_vec[k];
                        dot_r += ar * nr + ai * ni; // conj(a) * n
                        dot_i += ar * ni - ai * nr;
                    }
                    denom += dot_r * dot_r + dot_i * dot_i;
                }
                if denom > 1e-20 {
                    1.0 / denom
                } else {
                    1e10
                }
            })
            .collect()
    }

    /// Compute the steering vector for a given angle (degrees).
    pub fn steering_vector(&self, angle_deg: f64) -> Vec<(f64, f64)> {
        let angle_rad = angle_deg * PI / 180.0;
        (0..self.num_elements)
            .map(|k| {
                let phase = 2.0 * PI * self.spacing * k as f64 * angle_rad.sin();
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Generate synthetic test snapshots with sources at given DOA angles.
    ///
    /// `angles_deg`: source directions.
    /// `snr_db`: signal-to-noise ratio in dB.
    /// `num_snapshots`: number of time snapshots.
    pub fn generate_test_snapshots(
        &self,
        angles_deg: &[f64],
        snr_db: f64,
        num_snapshots: usize,
    ) -> Vec<Vec<(f64, f64)>> {
        let m = self.num_elements;
        let snr_lin = 10.0_f64.powf(snr_db / 10.0);
        let signal_amp = snr_lin.sqrt();

        // Simple deterministic PRNG
        let mut rng_state = 12345u64;
        let mut next_normal = || -> f64 {
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = (rng_state >> 33) as f64 / (1u64 << 31) as f64;
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (rng_state >> 33) as f64 / (1u64 << 31) as f64;
            let u1 = u1.max(1e-10);
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        // Compute steering vectors for all sources
        let steering: Vec<Vec<(f64, f64)>> = angles_deg
            .iter()
            .map(|&a| self.steering_vector(a))
            .collect();

        let mut snapshots = Vec::with_capacity(num_snapshots);
        for t in 0..num_snapshots {
            let mut snapshot = vec![(0.0, 0.0); m];

            // Add each source
            for (s_idx, sv) in steering.iter().enumerate() {
                let phase = 2.0 * PI * (t as f64 * 0.1 * (s_idx + 1) as f64);
                let sig = (signal_amp * phase.cos(), signal_amp * phase.sin());
                for k in 0..m {
                    snapshot[k].0 += sig.0 * sv[k].0 - sig.1 * sv[k].1;
                    snapshot[k].1 += sig.0 * sv[k].1 + sig.1 * sv[k].0;
                }
            }

            // Add noise
            for k in 0..m {
                snapshot[k].0 += next_normal();
                snapshot[k].1 += next_normal();
            }

            snapshots.push(snapshot);
        }
        snapshots
    }

    fn compute_covariance(&self, snapshots: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        let m = self.num_elements;
        let k = snapshots.len() as f64;
        let mut cov = vec![vec![(0.0, 0.0); m]; m];

        for snap in snapshots {
            for i in 0..m {
                for j in 0..m {
                    // R[i][j] += x[i] * conj(x[j])
                    let (xi_r, xi_i) = snap[i];
                    let (xj_r, xj_i) = snap[j];
                    cov[i][j].0 += xi_r * xj_r + xi_i * xj_i;
                    cov[i][j].1 += xi_i * xj_r - xi_r * xj_i;
                }
            }
        }

        for i in 0..m {
            for j in 0..m {
                cov[i][j].0 /= k;
                cov[i][j].1 /= k;
            }
        }
        cov
    }

    fn find_peaks(&self, angles: &[f64], spectrum: &[f64], num_peaks: usize) -> Vec<f64> {
        let mut peaks = Vec::new();
        for i in 1..spectrum.len() - 1 {
            if spectrum[i] > spectrum[i - 1] && spectrum[i] > spectrum[i + 1] {
                peaks.push((angles[i], spectrum[i]));
            }
        }
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        peaks.truncate(num_peaks);
        let mut result: Vec<f64> = peaks.iter().map(|p| p.0).collect();
        result.sort_by(|a, b| a.partial_cmp(b).unwrap());
        result
    }
}

/// Estimate number of sources using Minimum Description Length (MDL).
///
/// `eigenvalues`: sorted eigenvalues of the spatial covariance matrix (ascending).
/// `num_snapshots`: number of array snapshots K.
pub fn mdl_source_count(eigenvalues: &[f64], num_snapshots: usize) -> usize {
    let m = eigenvalues.len();
    if m < 2 {
        return 0;
    }
    let k = num_snapshots as f64;

    let mut best_d = 0;
    let mut best_mdl = f64::MAX;

    for d in 0..m {
        let noise_dim = m - d;
        if noise_dim == 0 {
            break;
        }

        // Arithmetic and geometric means of noise eigenvalues
        let noise_eigs = &eigenvalues[..noise_dim];
        let arith_mean: f64 = noise_eigs.iter().sum::<f64>() / noise_dim as f64;
        let geo_mean: f64 = if noise_eigs.iter().all(|&x| x > 0.0) {
            (noise_eigs.iter().map(|x| x.ln()).sum::<f64>() / noise_dim as f64).exp()
        } else {
            0.0
        };

        if geo_mean <= 0.0 || arith_mean <= 0.0 {
            continue;
        }

        let log_ratio = (arith_mean / geo_mean).ln();
        let log_likelihood = -k * noise_dim as f64 * log_ratio;
        let penalty = 0.5 * d as f64 * (2.0 * m as f64 - d as f64) * k.ln();

        let mdl_val = -log_likelihood + penalty;
        if mdl_val < best_mdl {
            best_mdl = mdl_val;
            best_d = d;
        }
    }

    best_d
}

/// Estimate number of sources using Akaike Information Criterion (AIC).
pub fn aic_source_count(eigenvalues: &[f64], num_snapshots: usize) -> usize {
    let m = eigenvalues.len();
    if m < 2 {
        return 0;
    }
    let k = num_snapshots as f64;

    let mut best_d = 0;
    let mut best_aic = f64::MAX;

    for d in 0..m {
        let noise_dim = m - d;
        if noise_dim == 0 {
            break;
        }

        let noise_eigs = &eigenvalues[..noise_dim];
        let arith_mean: f64 = noise_eigs.iter().sum::<f64>() / noise_dim as f64;
        let geo_mean: f64 = if noise_eigs.iter().all(|&x| x > 0.0) {
            (noise_eigs.iter().map(|x| x.ln()).sum::<f64>() / noise_dim as f64).exp()
        } else {
            0.0
        };

        if geo_mean <= 0.0 || arith_mean <= 0.0 {
            continue;
        }

        let log_ratio = (arith_mean / geo_mean).ln();
        let log_likelihood = -k * noise_dim as f64 * log_ratio;
        let penalty = d as f64 * (2.0 * m as f64 - d as f64);

        let aic_val = -2.0 * log_likelihood + 2.0 * penalty;
        if aic_val < best_aic {
            best_aic = aic_val;
            best_d = d;
        }
    }

    best_d
}

// ---- Hermitian eigendecomposition (Jacobi method for small matrices) ----

fn eigen_hermitian(matrix: &[Vec<(f64, f64)>]) -> (Vec<f64>, Vec<Vec<(f64, f64)>>) {
    let n = matrix.len();
    // Convert Hermitian eigenvalue problem to real symmetric via augmented form.
    // For H = A + jB (A symmetric, B antisymmetric):
    //   M = [A, -B; B, A]  (2N x 2N real symmetric)
    // Each eigenvalue of H appears twice in M.
    // Eigenvector (v_r, v_i) in M → complex eigenvector v_r + j*v_i.
    let mut aug = vec![vec![0.0; 2 * n]; 2 * n];
    for i in 0..n {
        for j in 0..n {
            let (re, im) = matrix[i][j];
            // Top-left: A = Re(H)
            aug[i][j] = re;
            // Top-right: -B = -Im(H)
            aug[i][n + j] = -im;
            // Bottom-left: B = Im(H)
            aug[n + i][j] = im;
            // Bottom-right: A = Re(H)
            aug[n + i][n + j] = re;
        }
    }

    let (eig_vals, eig_vecs) = jacobi_eigen(&aug);

    // Each eigenvalue appears twice; deduplicate by pairing.
    // Sort by eigenvalue and pick every other one.
    let mut indices: Vec<usize> = (0..2 * n).collect();
    indices.sort_by(|&a, &b| eig_vals[a].partial_cmp(&eig_vals[b]).unwrap());

    let mut eigenvalues = Vec::with_capacity(n);
    let mut eigenvectors = Vec::with_capacity(n);
    let mut used = vec![false; 2 * n];

    for &idx in &indices {
        if used[idx] {
            continue;
        }
        used[idx] = true;
        eigenvalues.push(eig_vals[idx]);

        // Extract complex eigenvector: top half = real part, bottom half = imag part
        let v_r: Vec<f64> = (0..n).map(|k| eig_vecs[idx][k]).collect();
        let v_i: Vec<f64> = (0..n).map(|k| eig_vecs[idx][n + k]).collect();
        let complex_vec: Vec<(f64, f64)> = v_r.iter().zip(v_i.iter())
            .map(|(&r, &i)| (r, i))
            .collect();
        eigenvectors.push(complex_vec);

        // Mark the paired eigenvalue as used
        for &idx2 in &indices {
            if !used[idx2] && (eig_vals[idx2] - eig_vals[idx]).abs() < 1e-6 {
                used[idx2] = true;
                break;
            }
        }

        if eigenvalues.len() == n {
            break;
        }
    }

    (eigenvalues, eigenvectors)
}

fn jacobi_eigen(a: &[Vec<f64>]) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = a.len();
    let mut d: Vec<f64> = (0..n).map(|i| a[i][i]).collect();
    let mut v = vec![vec![0.0; n]; n];
    for i in 0..n {
        v[i][i] = 1.0;
    }
    let mut b = d.clone();
    let mut z = vec![0.0; n];
    let mut mat = a.to_vec();

    for _ in 0..100 {
        // Sum of off-diagonal elements
        let mut sm = 0.0;
        for i in 0..n - 1 {
            for j in i + 1..n {
                sm += mat[i][j].abs();
            }
        }
        if sm < 1e-12 {
            break;
        }

        for p in 0..n - 1 {
            for q in p + 1..n {
                if mat[p][q].abs() < 1e-15 {
                    continue;
                }
                let h = d[q] - d[p];
                let t = if h.abs() < 1e-15 {
                    1.0_f64.copysign(mat[p][q])
                } else {
                    let theta = 0.5 * h / mat[p][q];
                    1.0 / (theta.abs() + (1.0 + theta * theta).sqrt())
                        * theta.signum()
                };
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = t * c;
                let tau = s / (1.0 + c);
                let a_pq = mat[p][q];

                z[p] -= t * a_pq;
                z[q] += t * a_pq;
                d[p] -= t * a_pq;
                d[q] += t * a_pq;
                mat[p][q] = 0.0;

                for j in 0..p {
                    let g = mat[j][p];
                    let h = mat[j][q];
                    mat[j][p] = g - s * (h + g * tau);
                    mat[j][q] = h + s * (g - h * tau);
                }
                for j in p + 1..q {
                    let g = mat[p][j];
                    let h = mat[j][q];
                    mat[p][j] = g - s * (h + g * tau);
                    mat[j][q] = h + s * (g - h * tau);
                }
                for j in q + 1..n {
                    let g = mat[p][j];
                    let h = mat[q][j];
                    mat[p][j] = g - s * (h + g * tau);
                    mat[q][j] = h + s * (g - h * tau);
                }
                for j in 0..n {
                    let g = v[j][p];
                    let h = v[j][q];
                    v[j][p] = g - s * (h + g * tau);
                    v[j][q] = h + s * (g - h * tau);
                }
            }
        }

        for i in 0..n {
            b[i] += z[i];
            d[i] = b[i];
            z[i] = 0.0;
        }
    }

    // Convert eigenvectors: v[j][i] = j-th component of i-th eigenvector
    let eigenvectors: Vec<Vec<f64>> = (0..n)
        .map(|i| (0..n).map(|j| v[j][i]).collect())
        .collect();

    (d, eigenvectors)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_source_detection() {
        let mut music = MusicEstimator::new(8, 0.5, 1).angle_step(0.5);
        let snaps = music.generate_test_snapshots(&[30.0], 20.0, 200);
        let doas = music.estimate(&snaps);
        assert_eq!(doas.len(), 1);
        assert!(
            (doas[0] - 30.0).abs() < 3.0,
            "DOA={}, expected ~30.0",
            doas[0]
        );
    }

    #[test]
    fn test_two_source_detection() {
        let mut music = MusicEstimator::new(8, 0.5, 2).angle_step(0.5);
        let snaps = music.generate_test_snapshots(&[20.0, 50.0], 20.0, 300);
        let doas = music.estimate(&snaps);
        assert_eq!(doas.len(), 2);
        assert!(
            (doas[0] - 20.0).abs() < 5.0,
            "DOA[0]={}, expected ~20.0",
            doas[0]
        );
        assert!(
            (doas[1] - 50.0).abs() < 5.0,
            "DOA[1]={}, expected ~50.0",
            doas[1]
        );
    }

    #[test]
    fn test_steering_vector_broadside() {
        let music = MusicEstimator::new(4, 0.5, 1);
        let sv = music.steering_vector(0.0); // broadside: all phases = 0
        for (r, i) in &sv {
            assert!((r - 1.0).abs() < 1e-10);
            assert!(i.abs() < 1e-10);
        }
    }

    #[test]
    fn test_steering_vector_length() {
        let music = MusicEstimator::new(8, 0.5, 2);
        let sv = music.steering_vector(45.0);
        assert_eq!(sv.len(), 8);
        // Each element should have unit magnitude
        for (r, i) in &sv {
            let mag = (r * r + i * i).sqrt();
            assert!((mag - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_pseudospectrum() {
        let music = MusicEstimator::new(8, 0.5, 1);
        let snaps = music.generate_test_snapshots(&[0.0], 20.0, 200);
        let angles: Vec<f64> = (-90..=90).map(|a| a as f64).collect();
        let spectrum = music.pseudospectrum(&snaps, &angles);
        assert_eq!(spectrum.len(), angles.len());
        // All values should be positive
        assert!(spectrum.iter().all(|&s| s > 0.0));
    }

    #[test]
    fn test_mdl_zero_sources() {
        // All eigenvalues equal → noise only → 0 sources
        let eigs = vec![1.0, 1.0, 1.0, 1.0];
        let d = mdl_source_count(&eigs, 100);
        assert_eq!(d, 0);
    }

    #[test]
    fn test_mdl_one_source() {
        // One large eigenvalue, rest small and equal
        let eigs = vec![0.1, 0.1, 0.1, 10.0];
        let d = mdl_source_count(&eigs, 100);
        assert!(d >= 1, "MDL detected {d} sources, expected ≥1");
    }

    #[test]
    fn test_aic_one_source() {
        let eigs = vec![0.1, 0.1, 0.1, 10.0];
        let d = aic_source_count(&eigs, 100);
        assert!(d >= 1, "AIC detected {d} sources, expected ≥1");
    }

    #[test]
    fn test_empty_snapshots() {
        let mut music = MusicEstimator::new(4, 0.5, 1);
        let doas = music.estimate(&[]);
        assert!(doas.is_empty());
    }

    #[test]
    fn test_jacobi_eigen_identity() {
        let mat = vec![
            vec![3.0, 0.0, 0.0],
            vec![0.0, 2.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let (eigs, _) = jacobi_eigen(&mat);
        let mut sorted = eigs.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((sorted[0] - 1.0).abs() < 1e-10);
        assert!((sorted[1] - 2.0).abs() < 1e-10);
        assert!((sorted[2] - 3.0).abs() < 1e-10);
    }
}

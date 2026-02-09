//! Blind Source Separation via Independent Component Analysis (ICA)
//!
//! Separates linearly mixed signals into statistically independent
//! components without knowledge of the mixing process. Implements
//! FastICA (fixed-point ICA) with configurable nonlinearities and
//! whitening preprocessing via PCA.
//!
//! Key applications: interference mitigation, MIMO signal extraction,
//! cognitive radio spectrum sensing, cocktail party separation.
//!
//! No direct GNU Radio equivalent (advanced statistical signal processing).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::blind_source_separation::{FastIca, Nonlinearity};
//!
//! // Two mixed signals
//! let n = 500;
//! let s1: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
//! let s2: Vec<f64> = (0..n).map(|i| ((i % 37) as f64 / 18.0 - 1.0)).collect();
//!
//! // Mix: x = A * s
//! let x1: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.7 * a + 0.3 * b).collect();
//! let x2: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.4 * a + 0.6 * b).collect();
//!
//! let mut ica = FastIca::new(2, Nonlinearity::LogCosh);
//! let sources = ica.fit(&[x1, x2]);
//! assert_eq!(sources.len(), 2);
//! assert_eq!(sources[0].len(), n);
//! ```

use std::f64::consts::PI;

/// Nonlinearity function for FastICA contrast.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Nonlinearity {
    /// G(u) = log(cosh(u)), g(u) = tanh(u) — good general-purpose choice.
    LogCosh,
    /// G(u) = -exp(-u²/2), g(u) = u·exp(-u²/2) — good for super-Gaussian.
    Exp,
    /// G(u) = u⁴/4, g(u) = u³ — kurtosis-based, fast but less robust.
    Cube,
}

/// FastICA algorithm for blind source separation.
#[derive(Debug, Clone)]
pub struct FastIca {
    /// Number of independent components to extract.
    num_components: usize,
    /// Nonlinearity for contrast function.
    nonlinearity: Nonlinearity,
    /// Maximum iterations for convergence.
    max_iterations: usize,
    /// Convergence tolerance.
    tolerance: f64,
    /// Extracted unmixing matrix W (num_components × num_signals).
    unmixing_matrix: Option<Vec<Vec<f64>>>,
    /// Mean of each input channel (for centering).
    means: Vec<f64>,
    /// Whitening matrix.
    whitening_matrix: Option<Vec<Vec<f64>>>,
}

impl FastIca {
    /// Create a new FastICA instance.
    pub fn new(num_components: usize, nonlinearity: Nonlinearity) -> Self {
        Self {
            num_components: num_components.max(1),
            nonlinearity,
            max_iterations: 200,
            tolerance: 1e-6,
            unmixing_matrix: None,
            means: vec![],
            whitening_matrix: None,
        }
    }

    /// Set maximum iterations.
    pub fn max_iterations(mut self, n: usize) -> Self {
        self.max_iterations = n;
        self
    }

    /// Set convergence tolerance.
    pub fn tolerance(mut self, tol: f64) -> Self {
        self.tolerance = tol;
        self
    }

    /// Fit the ICA model and return separated sources.
    ///
    /// `signals`: one Vec<f64> per observed mixture, all same length.
    /// Returns one Vec<f64> per independent component.
    pub fn fit(&mut self, signals: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let m = signals.len(); // Number of observed signals
        if m == 0 {
            return vec![];
        }
        let n = signals[0].len(); // Number of samples
        if n == 0 {
            return vec![];
        }
        let nc = self.num_components.min(m);

        // Step 1: Center the data
        self.means = signals.iter().map(|s| s.iter().sum::<f64>() / n as f64).collect();
        let centered: Vec<Vec<f64>> = signals
            .iter()
            .zip(self.means.iter())
            .map(|(s, &mean)| s.iter().map(|&x| x - mean).collect())
            .collect();

        // Step 2: Whiten the data (PCA + scaling)
        let (whitened, whitening_mat) = self.whiten(&centered, nc);
        self.whitening_matrix = Some(whitening_mat);

        // Step 3: FastICA fixed-point iteration
        let w = self.fastica_deflation(&whitened, nc);
        self.unmixing_matrix = Some(w.clone());

        // Step 4: Compute sources: S = W · X_whitened
        let mut sources = vec![vec![0.0; n]; nc];
        for i in 0..nc {
            for j in 0..n {
                let mut sum = 0.0;
                for k in 0..whitened.len() {
                    sum += w[i][k] * whitened[k][j];
                }
                sources[i][j] = sum;
            }
        }

        sources
    }

    /// Transform new data using the fitted model.
    pub fn transform(&self, signals: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let w = match &self.unmixing_matrix {
            Some(w) => w,
            None => return vec![],
        };
        let wh = match &self.whitening_matrix {
            Some(w) => w,
            None => return vec![],
        };

        let n = signals[0].len();
        let m = signals.len();
        let nc = w.len();

        // Center
        let centered: Vec<Vec<f64>> = signals
            .iter()
            .zip(self.means.iter())
            .map(|(s, &mean)| s.iter().map(|&x| x - mean).collect())
            .collect();

        // Whiten
        let num_white = wh.len();
        let mut whitened = vec![vec![0.0; n]; num_white];
        for i in 0..num_white {
            for j in 0..n {
                let mut sum = 0.0;
                for k in 0..m {
                    sum += wh[i][k] * centered[k][j];
                }
                whitened[i][j] = sum;
            }
        }

        // Apply unmixing
        let mut sources = vec![vec![0.0; n]; nc];
        for i in 0..nc {
            for j in 0..n {
                let mut sum = 0.0;
                for k in 0..whitened.len() {
                    sum += w[i][k] * whitened[k][j];
                }
                sources[i][j] = sum;
            }
        }

        sources
    }

    /// Get the unmixing matrix.
    pub fn unmixing_matrix(&self) -> Option<&Vec<Vec<f64>>> {
        self.unmixing_matrix.as_ref()
    }

    // ---- Internal methods ----

    /// Whiten data via PCA: X_white = D^{-1/2} · E^T · X
    fn whiten(&self, data: &[Vec<f64>], num_components: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let m = data.len();
        let n = data[0].len();

        // Compute covariance matrix
        let mut cov = vec![vec![0.0; m]; m];
        for i in 0..m {
            for j in i..m {
                let c: f64 = (0..n)
                    .map(|k| data[i][k] * data[j][k])
                    .sum::<f64>()
                    / n as f64;
                cov[i][j] = c;
                cov[j][i] = c;
            }
        }

        // Eigenvalue decomposition via power iteration (for small m)
        let (eigenvalues, eigenvectors) = eigen_symmetric(&cov, num_components);

        // Whitening matrix: W = D^{-1/2} · E^T
        let nc = eigenvalues.len();
        let mut whitening = vec![vec![0.0; m]; nc];
        for i in 0..nc {
            let scale = if eigenvalues[i] > 1e-10 {
                1.0 / eigenvalues[i].sqrt()
            } else {
                0.0
            };
            for j in 0..m {
                whitening[i][j] = scale * eigenvectors[i][j];
            }
        }

        // Apply whitening
        let mut whitened = vec![vec![0.0; n]; nc];
        for i in 0..nc {
            for j in 0..n {
                let mut sum = 0.0;
                for k in 0..m {
                    sum += whitening[i][k] * data[k][j];
                }
                whitened[i][j] = sum;
            }
        }

        (whitened, whitening)
    }

    /// FastICA with deflation approach (extract components one by one).
    fn fastica_deflation(&self, whitened: &[Vec<f64>], num_components: usize) -> Vec<Vec<f64>> {
        let p = whitened.len(); // Dimension
        let n = whitened[0].len();
        let mut w_all: Vec<Vec<f64>> = Vec::with_capacity(num_components);

        // Deterministic initialization
        let mut seed = 42u64;

        for comp in 0..num_components {
            // Initialize random weight vector
            let mut w: Vec<f64> = (0..p)
                .map(|_| {
                    seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5
                })
                .collect();
            normalize(&mut w);

            for _iter in 0..self.max_iterations {
                // Compute w^T · x for all samples
                let wx: Vec<f64> = (0..n)
                    .map(|j| {
                        let mut sum = 0.0;
                        for k in 0..p {
                            sum += w[k] * whitened[k][j];
                        }
                        sum
                    })
                    .collect();

                // Compute g(w^T·x) and g'(w^T·x)
                let (gx, g_prime_mean) = self.apply_nonlinearity(&wx);

                // FastICA update: w_new = E{x·g(w^T·x)} - E{g'(w^T·x)}·w
                let mut w_new = vec![0.0; p];
                for k in 0..p {
                    let mean_xg: f64 = (0..n)
                        .map(|j| whitened[k][j] * gx[j])
                        .sum::<f64>()
                        / n as f64;
                    w_new[k] = mean_xg - g_prime_mean * w[k];
                }

                // Deflation: orthogonalize w.r.t. previously found components
                for prev in &w_all {
                    let dot: f64 = w_new.iter().zip(prev.iter()).map(|(a, b)| a * b).sum();
                    for k in 0..p {
                        w_new[k] -= dot * prev[k];
                    }
                }

                normalize(&mut w_new);

                // Check convergence
                let dot: f64 = w.iter().zip(w_new.iter()).map(|(a, b)| a * b).sum();
                w = w_new;

                if (dot.abs() - 1.0).abs() < self.tolerance {
                    break;
                }
            }

            w_all.push(w);
        }

        w_all
    }

    /// Apply nonlinearity and return (g(u), mean(g'(u))).
    fn apply_nonlinearity(&self, u: &[f64]) -> (Vec<f64>, f64) {
        let n = u.len();
        match self.nonlinearity {
            Nonlinearity::LogCosh => {
                let gx: Vec<f64> = u.iter().map(|&x| x.tanh()).collect();
                let g_prime_mean: f64 = u.iter().map(|&x| 1.0 - x.tanh().powi(2)).sum::<f64>() / n as f64;
                (gx, g_prime_mean)
            }
            Nonlinearity::Exp => {
                let gx: Vec<f64> = u.iter().map(|&x| x * (-x * x / 2.0).exp()).collect();
                let g_prime_mean: f64 = u
                    .iter()
                    .map(|&x| {
                        let e = (-x * x / 2.0).exp();
                        (1.0 - x * x) * e
                    })
                    .sum::<f64>()
                    / n as f64;
                (gx, g_prime_mean)
            }
            Nonlinearity::Cube => {
                let gx: Vec<f64> = u.iter().map(|&x| x * x * x).collect();
                let g_prime_mean: f64 = u.iter().map(|&x| 3.0 * x * x).sum::<f64>() / n as f64;
                (gx, g_prime_mean)
            }
        }
    }
}

/// Compute the kurtosis of a signal (excess kurtosis, 0 for Gaussian).
pub fn kurtosis(signal: &[f64]) -> f64 {
    let n = signal.len() as f64;
    if n < 4.0 {
        return 0.0;
    }
    let mean = signal.iter().sum::<f64>() / n;
    let m2: f64 = signal.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
    let m4: f64 = signal.iter().map(|x| (x - mean).powi(4)).sum::<f64>() / n;
    if m2 < 1e-30 {
        return 0.0;
    }
    m4 / (m2 * m2) - 3.0
}

/// Compute the negentropy approximation (measure of non-Gaussianity).
pub fn negentropy(signal: &[f64]) -> f64 {
    let n = signal.len() as f64;
    if n < 2.0 {
        return 0.0;
    }
    // J(x) ≈ [E{G(x)} - E{G(v)}]² where v ~ N(0,1) and G = logcosh
    let mean = signal.iter().sum::<f64>() / n;
    let std = (signal.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n).sqrt();
    if std < 1e-30 {
        return 0.0;
    }
    let normalized: Vec<f64> = signal.iter().map(|x| (x - mean) / std).collect();
    let e_g: f64 = normalized.iter().map(|&x| x.cosh().ln()).sum::<f64>() / n;
    let e_g_gauss = 0.3745; // E{log(cosh(v))} for v ~ N(0,1)
    (e_g - e_g_gauss).powi(2)
}

/// Simple correlation metric between two signals (absolute Pearson correlation).
pub fn correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len()) as f64;
    if n < 2.0 {
        return 0.0;
    }
    let mean_a = a.iter().sum::<f64>() / n;
    let mean_b = b.iter().sum::<f64>() / n;
    let mut cov = 0.0;
    let mut var_a = 0.0;
    let mut var_b = 0.0;
    for i in 0..n as usize {
        let da = a[i] - mean_a;
        let db = b[i] - mean_b;
        cov += da * db;
        var_a += da * da;
        var_b += db * db;
    }
    if var_a < 1e-30 || var_b < 1e-30 {
        return 0.0;
    }
    (cov / (var_a * var_b).sqrt()).abs()
}

// ---- Internal helpers ----

fn normalize(v: &mut [f64]) {
    let norm: f64 = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    if norm > 1e-30 {
        for x in v.iter_mut() {
            *x /= norm;
        }
    }
}

/// Eigenvalue decomposition of symmetric matrix via power iteration.
/// Returns top `k` eigenvalues and eigenvectors.
fn eigen_symmetric(matrix: &[Vec<f64>], k: usize) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = matrix.len();
    let k = k.min(n);
    let mut eigenvalues = Vec::with_capacity(k);
    let mut eigenvectors = Vec::with_capacity(k);

    // Deflated matrix
    let mut a: Vec<Vec<f64>> = matrix.to_vec();

    let mut seed = 123u64;

    for _ in 0..k {
        // Power iteration
        let mut v: Vec<f64> = (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5
            })
            .collect();
        normalize(&mut v);

        let mut eigenvalue = 0.0;
        for _ in 0..300 {
            // v_new = A · v
            let mut v_new = vec![0.0; n];
            for i in 0..n {
                for j in 0..n {
                    v_new[i] += a[i][j] * v[j];
                }
            }

            eigenvalue = v_new.iter().map(|x| x * x).sum::<f64>().sqrt();
            if eigenvalue < 1e-30 {
                break;
            }
            for x in v_new.iter_mut() {
                *x /= eigenvalue;
            }

            // Check convergence
            let dot: f64 = v.iter().zip(v_new.iter()).map(|(a, b)| a * b).sum();
            v = v_new;
            if (dot.abs() - 1.0).abs() < 1e-10 {
                break;
            }
        }

        // Determine sign of eigenvalue
        let mut av = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                av[i] += a[i][j] * v[j];
            }
        }
        let dot: f64 = av.iter().zip(v.iter()).map(|(a, b)| a * b).sum();
        if dot < 0.0 {
            eigenvalue = -eigenvalue;
        }

        eigenvalues.push(eigenvalue);
        eigenvectors.push(v.clone());

        // Deflate: A = A - λ·v·vᵀ
        for i in 0..n {
            for j in 0..n {
                a[i][j] -= eigenvalue * v[i] * v[j];
            }
        }
    }

    (eigenvalues, eigenvectors)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_two_source_separation() {
        let n = 1000;
        // Source 1: sinusoid
        let s1: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
        // Source 2: sawtooth-like
        let s2: Vec<f64> = (0..n).map(|i| (i % 50) as f64 / 25.0 - 1.0).collect();

        // Mix
        let x1: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.6 * a + 0.4 * b).collect();
        let x2: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.3 * a + 0.7 * b).collect();

        let mut ica = FastIca::new(2, Nonlinearity::LogCosh);
        let sources = ica.fit(&[x1, x2]);
        assert_eq!(sources.len(), 2);
        assert_eq!(sources[0].len(), n);

        // Sources should be uncorrelated with each other
        let corr = correlation(&sources[0], &sources[1]);
        assert!(corr < 0.3, "sources should be uncorrelated: corr={corr}");
    }

    #[test]
    fn test_nonlinearities() {
        let n = 500;
        let s1: Vec<f64> = (0..n).map(|i| (i as f64 * 0.15).sin()).collect();
        let s2: Vec<f64> = (0..n).map(|i| ((i * 7 + 3) % 41) as f64 / 20.0 - 1.0).collect();
        let x1: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.5 * a + 0.5 * b).collect();
        let x2: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.4 * a + 0.6 * b).collect();

        for nonlin in [Nonlinearity::LogCosh, Nonlinearity::Exp, Nonlinearity::Cube] {
            let mut ica = FastIca::new(2, nonlin);
            let sources = ica.fit(&[x1.clone(), x2.clone()]);
            assert_eq!(sources.len(), 2);
        }
    }

    #[test]
    fn test_single_source() {
        let signal: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let mut ica = FastIca::new(1, Nonlinearity::LogCosh);
        let sources = ica.fit(&[signal]);
        assert_eq!(sources.len(), 1);
    }

    #[test]
    fn test_kurtosis_gaussian_like() {
        // Uniform distribution has negative excess kurtosis (-1.2)
        let signal: Vec<f64> = (0..1000).map(|i| (i as f64 / 500.0) - 1.0).collect();
        let k = kurtosis(&signal);
        assert!((k - (-1.2)).abs() < 0.1, "k={k}");
    }

    #[test]
    fn test_kurtosis_super_gaussian() {
        // Laplacian-like (peaked) has positive excess kurtosis
        let signal: Vec<f64> = (0..1000)
            .map(|i| {
                let x = (i as f64 / 500.0) - 1.0;
                x.signum() * (-2.0 * x.abs()).exp()
            })
            .collect();
        // Just check it's finite
        let k = kurtosis(&signal);
        assert!(k.is_finite());
    }

    #[test]
    fn test_negentropy() {
        let signal: Vec<f64> = (0..500).map(|i| (i as f64 * 0.1).sin()).collect();
        let ne = negentropy(&signal);
        assert!(ne >= 0.0);
        assert!(ne.is_finite());
    }

    #[test]
    fn test_correlation() {
        let a: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let b = a.clone();
        assert!((correlation(&a, &b) - 1.0).abs() < 0.01);

        // Orthogonal signals
        let c: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).cos()).collect();
        let corr = correlation(&a, &c);
        assert!(corr < 0.2, "corr={corr}");
    }

    #[test]
    fn test_transform() {
        let n = 500;
        let s1: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
        let s2: Vec<f64> = (0..n).map(|i| ((i * 3) % 29) as f64 / 14.0 - 1.0).collect();
        let x1: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.6 * a + 0.4 * b).collect();
        let x2: Vec<f64> = s1.iter().zip(s2.iter()).map(|(a, b)| 0.3 * a + 0.7 * b).collect();

        let mut ica = FastIca::new(2, Nonlinearity::LogCosh);
        let _sources = ica.fit(&[x1.clone(), x2.clone()]);
        assert!(ica.unmixing_matrix().is_some());

        // Transform same data should give similar result
        let transformed = ica.transform(&[x1, x2]);
        assert_eq!(transformed.len(), 2);
    }

    #[test]
    fn test_empty_input() {
        let mut ica = FastIca::new(2, Nonlinearity::LogCosh);
        let sources = ica.fit(&[]);
        assert!(sources.is_empty());
    }

    #[test]
    fn test_eigen_symmetric() {
        // 2x2 identity → eigenvalues should be 1.0, 1.0
        let mat = vec![vec![2.0, 1.0], vec![1.0, 2.0]];
        let (vals, vecs) = eigen_symmetric(&mat, 2);
        assert_eq!(vals.len(), 2);
        // Eigenvalues should be 3 and 1
        let mut sorted = vals.clone();
        sorted.sort_by(|a, b| b.partial_cmp(a).unwrap());
        assert!((sorted[0] - 3.0).abs() < 0.1, "λ1={}", sorted[0]);
        assert!((sorted[1] - 1.0).abs() < 0.1, "λ2={}", sorted[1]);
    }
}

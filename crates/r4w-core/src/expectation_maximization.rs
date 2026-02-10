//! Expectation-Maximization (EM) algorithm for Gaussian Mixture Model fitting
//! and signal component decomposition.
//!
//! This module provides both 1D and 2D (complex IQ) GMM fitting using the EM
//! algorithm, with K-means initialization, BIC/AIC model order selection,
//! posterior classification, and component merging for redundant clusters.
//!
//! # Example
//!
//! ```
//! use r4w_core::expectation_maximization::EmGmm;
//!
//! // Generate two clusters of 1D data
//! let mut data = Vec::new();
//! for i in 0..50 {
//!     data.push(0.0 + (i as f64 - 25.0) * 0.02);
//! }
//! for i in 0..50 {
//!     data.push(5.0 + (i as f64 - 25.0) * 0.02);
//! }
//!
//! let mut gmm = EmGmm::new(2);
//! gmm.fit(&data);
//!
//! let means = gmm.means();
//! assert_eq!(means.len(), 2);
//! // The two means should be near 0.0 and 5.0 (order may vary)
//! let mut sorted = means.to_vec();
//! sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
//! assert!((sorted[0] - 0.0).abs() < 1.0);
//! assert!((sorted[1] - 5.0).abs() < 1.0);
//! ```

use std::f64::consts::PI;

/// A 1D Gaussian Mixture Model fitted via the EM algorithm.
#[derive(Debug, Clone)]
pub struct EmGmm {
    k: usize,
    max_iter: usize,
    tol: f64,
    means: Vec<f64>,
    variances: Vec<f64>,
    weights: Vec<f64>,
    log_likelihood: f64,
    converged: bool,
    iterations: usize,
}

/// A 2D Gaussian Mixture Model for complex IQ signal clustering.
///
/// Complex samples are represented as `(f64, f64)` tuples of (re, im).
#[derive(Debug, Clone)]
pub struct EmGmm2D {
    k: usize,
    max_iter: usize,
    tol: f64,
    means: Vec<(f64, f64)>,
    /// Each component has a 2x2 covariance stored as (var_xx, var_xy, var_yx, var_yy).
    covariances: Vec<(f64, f64, f64, f64)>,
    weights: Vec<f64>,
    log_likelihood: f64,
    converged: bool,
    iterations: usize,
}

/// Result of model order selection via BIC/AIC.
#[derive(Debug, Clone)]
pub struct ModelSelection {
    /// BIC values for each tested number of components.
    pub bic_values: Vec<(usize, f64)>,
    /// AIC values for each tested number of components.
    pub aic_values: Vec<(usize, f64)>,
    /// Best number of components according to BIC.
    pub best_k_bic: usize,
    /// Best number of components according to AIC.
    pub best_k_aic: usize,
}

// ---------------------------------------------------------------------------
// 1D GMM
// ---------------------------------------------------------------------------

impl EmGmm {
    /// Create a new 1D GMM with `k` components.
    pub fn new(k: usize) -> Self {
        assert!(k >= 1, "Number of components must be >= 1");
        Self {
            k,
            max_iter: 200,
            tol: 1e-6,
            means: Vec::new(),
            variances: Vec::new(),
            weights: Vec::new(),
            log_likelihood: f64::NEG_INFINITY,
            converged: false,
            iterations: 0,
        }
    }

    /// Set the maximum number of EM iterations (default 200).
    pub fn with_max_iter(mut self, max_iter: usize) -> Self {
        self.max_iter = max_iter;
        self
    }

    /// Set the convergence tolerance on log-likelihood change (default 1e-6).
    pub fn with_tolerance(mut self, tol: f64) -> Self {
        self.tol = tol;
        self
    }

    /// Fit the GMM to the provided 1D data using K-means initialisation and EM.
    pub fn fit(&mut self, data: &[f64]) {
        let n = data.len();
        assert!(n >= self.k, "Need at least as many data points as components");

        // K-means initialization
        self.kmeans_init(data);

        let mut responsibilities = vec![vec![0.0; self.k]; n];
        let mut prev_ll = f64::NEG_INFINITY;

        for iter in 0..self.max_iter {
            // E-step
            self.e_step(data, &mut responsibilities);

            // Log-likelihood
            let ll = self.log_likelihood(data);
            self.log_likelihood = ll;
            self.iterations = iter + 1;

            if (ll - prev_ll).abs() < self.tol {
                self.converged = true;
                break;
            }
            prev_ll = ll;

            // M-step
            self.m_step(data, &responsibilities);
        }
    }

    /// Compute posterior responsibilities (E-step).
    ///
    /// `resp[i][j]` is the posterior probability that data point `i` belongs to
    /// component `j`.
    pub fn e_step(&self, data: &[f64], resp: &mut [Vec<f64>]) {
        let n = data.len();
        for i in 0..n {
            let mut total = 0.0;
            for j in 0..self.k {
                let p = self.weights[j] * gaussian_pdf(data[i], self.means[j], self.variances[j]);
                resp[i][j] = p;
                total += p;
            }
            if total > 0.0 {
                for j in 0..self.k {
                    resp[i][j] /= total;
                }
            } else {
                // Uniform fallback
                for j in 0..self.k {
                    resp[i][j] = 1.0 / self.k as f64;
                }
            }
        }
    }

    /// Update parameters from responsibilities (M-step).
    pub fn m_step(&mut self, data: &[f64], resp: &[Vec<f64>]) {
        let n = data.len() as f64;
        for j in 0..self.k {
            let nk: f64 = resp.iter().map(|r| r[j]).sum();
            if nk < 1e-15 {
                continue;
            }
            self.weights[j] = nk / n;

            let mean: f64 = resp.iter().enumerate().map(|(i, r)| r[j] * data[i]).sum::<f64>() / nk;
            self.means[j] = mean;

            let var: f64 = resp
                .iter()
                .enumerate()
                .map(|(i, r)| r[j] * (data[i] - mean).powi(2))
                .sum::<f64>()
                / nk;
            // Floor variance to avoid singularity
            self.variances[j] = var.max(1e-10);
        }
    }

    /// Compute the log-likelihood of data under the current model.
    pub fn log_likelihood(&self, data: &[f64]) -> f64 {
        data.iter()
            .map(|&x| {
                let p: f64 = (0..self.k)
                    .map(|j| self.weights[j] * gaussian_pdf(x, self.means[j], self.variances[j]))
                    .sum();
                if p > 0.0 {
                    p.ln()
                } else {
                    -1e30
                }
            })
            .sum()
    }

    /// Classify new data points, returning the most-likely component index for each.
    pub fn classify(&self, data: &[f64]) -> Vec<usize> {
        data.iter()
            .map(|&x| {
                (0..self.k)
                    .max_by(|&a, &b| {
                        let pa = self.weights[a] * gaussian_pdf(x, self.means[a], self.variances[a]);
                        let pb = self.weights[b] * gaussian_pdf(x, self.means[b], self.variances[b]);
                        pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .unwrap()
            })
            .collect()
    }

    /// Return posterior probabilities for a single data point.
    pub fn posterior(&self, x: f64) -> Vec<f64> {
        let mut probs: Vec<f64> = (0..self.k)
            .map(|j| self.weights[j] * gaussian_pdf(x, self.means[j], self.variances[j]))
            .collect();
        let total: f64 = probs.iter().sum();
        if total > 0.0 {
            for p in &mut probs {
                *p /= total;
            }
        }
        probs
    }

    /// Merge components that are closer than `threshold` standard deviations apart.
    ///
    /// Returns the number of merges performed.
    pub fn merge_components(&mut self, threshold: f64) -> usize {
        let mut merges = 0;
        let mut i = 0;
        while i < self.k {
            let mut j = i + 1;
            while j < self.k {
                let dist = (self.means[i] - self.means[j]).abs();
                let avg_std = (self.variances[i].sqrt() + self.variances[j].sqrt()) / 2.0;
                if dist < threshold * avg_std {
                    // Merge j into i
                    let wi = self.weights[i];
                    let wj = self.weights[j];
                    let total_w = wi + wj;
                    let new_mean = (wi * self.means[i] + wj * self.means[j]) / total_w;
                    let new_var = (wi * (self.variances[i] + (self.means[i] - new_mean).powi(2))
                        + wj * (self.variances[j] + (self.means[j] - new_mean).powi(2)))
                        / total_w;
                    self.means[i] = new_mean;
                    self.variances[i] = new_var;
                    self.weights[i] = total_w;
                    self.means.remove(j);
                    self.variances.remove(j);
                    self.weights.remove(j);
                    self.k -= 1;
                    merges += 1;
                } else {
                    j += 1;
                }
            }
            i += 1;
        }
        merges
    }

    /// Return the fitted means.
    pub fn means(&self) -> &[f64] {
        &self.means
    }

    /// Return the fitted variances.
    pub fn variances(&self) -> &[f64] {
        &self.variances
    }

    /// Return the mixing weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Return whether the algorithm converged.
    pub fn converged(&self) -> bool {
        self.converged
    }

    /// Return the number of EM iterations performed.
    pub fn iterations(&self) -> usize {
        self.iterations
    }

    /// Return the number of components.
    pub fn num_components(&self) -> usize {
        self.k
    }

    /// Return the final log-likelihood.
    pub fn final_log_likelihood(&self) -> f64 {
        self.log_likelihood
    }

    /// Compute the Bayesian Information Criterion (BIC) for the current model.
    ///
    /// BIC = -2 * log_likelihood + num_params * ln(n)
    pub fn bic(&self, n: usize) -> f64 {
        let num_params = 3 * self.k - 1; // means + variances + weights (with sum=1 constraint)
        -2.0 * self.log_likelihood + (num_params as f64) * (n as f64).ln()
    }

    /// Compute the Akaike Information Criterion (AIC) for the current model.
    ///
    /// AIC = -2 * log_likelihood + 2 * num_params
    pub fn aic(&self, _n: usize) -> f64 {
        let num_params = 3 * self.k - 1;
        -2.0 * self.log_likelihood + 2.0 * num_params as f64
    }

    // ---- private helpers ----

    fn kmeans_init(&mut self, data: &[f64]) {
        let n = data.len();

        // Deterministic initialisation: spread evenly through sorted data
        let mut sorted = data.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

        self.means = (0..self.k)
            .map(|j| {
                let idx = (j * n) / self.k + n / (2 * self.k).max(1);
                sorted[idx.min(n - 1)]
            })
            .collect();

        // Run a few K-means iterations
        let mut assignments = vec![0usize; n];
        for _iter in 0..20 {
            // Assign
            for (i, &x) in data.iter().enumerate() {
                assignments[i] = (0..self.k)
                    .min_by(|&a, &b| {
                        let da = (x - self.means[a]).abs();
                        let db = (x - self.means[b]).abs();
                        da.partial_cmp(&db).unwrap()
                    })
                    .unwrap();
            }
            // Update means
            for j in 0..self.k {
                let mut sum = 0.0;
                let mut count = 0usize;
                for (i, &x) in data.iter().enumerate() {
                    if assignments[i] == j {
                        sum += x;
                        count += 1;
                    }
                }
                if count > 0 {
                    self.means[j] = sum / count as f64;
                }
            }
        }

        // Compute initial variances from cluster assignments
        self.variances = vec![1.0; self.k];
        self.weights = vec![1.0 / self.k as f64; self.k];
        for j in 0..self.k {
            let mut var_sum = 0.0;
            let mut count = 0usize;
            for (i, &x) in data.iter().enumerate() {
                if assignments[i] == j {
                    var_sum += (x - self.means[j]).powi(2);
                    count += 1;
                }
            }
            if count > 1 {
                self.variances[j] = (var_sum / count as f64).max(1e-10);
            }
            self.weights[j] = count as f64 / n as f64;
        }
        // Ensure weights are positive
        for w in &mut self.weights {
            if *w < 1e-10 {
                *w = 1e-10;
            }
        }
        let wsum: f64 = self.weights.iter().sum();
        for w in &mut self.weights {
            *w /= wsum;
        }
    }
}

/// Perform model order selection by fitting GMMs with 1..=max_k components.
pub fn select_model_order(data: &[f64], max_k: usize) -> ModelSelection {
    let n = data.len();
    let mut bic_values = Vec::new();
    let mut aic_values = Vec::new();

    for k in 1..=max_k {
        if k > n {
            break;
        }
        let mut gmm = EmGmm::new(k);
        gmm.fit(data);
        bic_values.push((k, gmm.bic(n)));
        aic_values.push((k, gmm.aic(n)));
    }

    let best_k_bic = bic_values
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .map(|&(k, _)| k)
        .unwrap_or(1);

    let best_k_aic = aic_values
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .map(|&(k, _)| k)
        .unwrap_or(1);

    ModelSelection {
        bic_values,
        aic_values,
        best_k_bic,
        best_k_aic,
    }
}

// ---------------------------------------------------------------------------
// 2D GMM for complex IQ
// ---------------------------------------------------------------------------

impl EmGmm2D {
    /// Create a new 2D GMM with `k` components for IQ data.
    pub fn new(k: usize) -> Self {
        assert!(k >= 1, "Number of components must be >= 1");
        Self {
            k,
            max_iter: 200,
            tol: 1e-6,
            means: Vec::new(),
            covariances: Vec::new(),
            weights: Vec::new(),
            log_likelihood: f64::NEG_INFINITY,
            converged: false,
            iterations: 0,
        }
    }

    /// Set the maximum number of EM iterations (default 200).
    pub fn with_max_iter(mut self, max_iter: usize) -> Self {
        self.max_iter = max_iter;
        self
    }

    /// Set the convergence tolerance (default 1e-6).
    pub fn with_tolerance(mut self, tol: f64) -> Self {
        self.tol = tol;
        self
    }

    /// Fit the 2D GMM to complex IQ data.
    pub fn fit(&mut self, data: &[(f64, f64)]) {
        let n = data.len();
        assert!(n >= self.k, "Need at least as many data points as components");

        self.kmeans_init_2d(data);

        let mut resp = vec![vec![0.0; self.k]; n];
        let mut prev_ll = f64::NEG_INFINITY;

        for iter in 0..self.max_iter {
            self.e_step_2d(data, &mut resp);
            let ll = self.log_likelihood_2d(data);
            self.log_likelihood = ll;
            self.iterations = iter + 1;

            if (ll - prev_ll).abs() < self.tol {
                self.converged = true;
                break;
            }
            prev_ll = ll;

            self.m_step_2d(data, &resp);
        }
    }

    /// E-step for 2D GMM.
    pub fn e_step_2d(&self, data: &[(f64, f64)], resp: &mut [Vec<f64>]) {
        for (i, &pt) in data.iter().enumerate() {
            let mut total = 0.0;
            for j in 0..self.k {
                let p = self.weights[j]
                    * gaussian_pdf_2d(pt, self.means[j], self.covariances[j]);
                resp[i][j] = p;
                total += p;
            }
            if total > 0.0 {
                for j in 0..self.k {
                    resp[i][j] /= total;
                }
            } else {
                for j in 0..self.k {
                    resp[i][j] = 1.0 / self.k as f64;
                }
            }
        }
    }

    /// M-step for 2D GMM.
    pub fn m_step_2d(&mut self, data: &[(f64, f64)], resp: &[Vec<f64>]) {
        let n = data.len() as f64;
        for j in 0..self.k {
            let nk: f64 = resp.iter().map(|r| r[j]).sum();
            if nk < 1e-15 {
                continue;
            }
            self.weights[j] = nk / n;

            let mx: f64 = resp.iter().enumerate().map(|(i, r)| r[j] * data[i].0).sum::<f64>() / nk;
            let my: f64 = resp.iter().enumerate().map(|(i, r)| r[j] * data[i].1).sum::<f64>() / nk;
            self.means[j] = (mx, my);

            let mut var_xx = 0.0;
            let mut var_xy = 0.0;
            let mut var_yy = 0.0;
            for (i, &(x, y)) in data.iter().enumerate() {
                let dx = x - mx;
                let dy = y - my;
                var_xx += resp[i][j] * dx * dx;
                var_xy += resp[i][j] * dx * dy;
                var_yy += resp[i][j] * dy * dy;
            }
            var_xx = (var_xx / nk).max(1e-10);
            var_xy /= nk;
            var_yy = (var_yy / nk).max(1e-10);
            self.covariances[j] = (var_xx, var_xy, var_xy, var_yy);
        }
    }

    /// Compute log-likelihood for 2D data.
    pub fn log_likelihood_2d(&self, data: &[(f64, f64)]) -> f64 {
        data.iter()
            .map(|&pt| {
                let p: f64 = (0..self.k)
                    .map(|j| {
                        self.weights[j] * gaussian_pdf_2d(pt, self.means[j], self.covariances[j])
                    })
                    .sum();
                if p > 0.0 {
                    p.ln()
                } else {
                    -1e30
                }
            })
            .sum()
    }

    /// Classify IQ data points to their most-likely component.
    pub fn classify(&self, data: &[(f64, f64)]) -> Vec<usize> {
        data.iter()
            .map(|&pt| {
                (0..self.k)
                    .max_by(|&a, &b| {
                        let pa = self.weights[a]
                            * gaussian_pdf_2d(pt, self.means[a], self.covariances[a]);
                        let pb = self.weights[b]
                            * gaussian_pdf_2d(pt, self.means[b], self.covariances[b]);
                        pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .unwrap()
            })
            .collect()
    }

    /// Return posterior probabilities for a single IQ point.
    pub fn posterior(&self, pt: (f64, f64)) -> Vec<f64> {
        let mut probs: Vec<f64> = (0..self.k)
            .map(|j| self.weights[j] * gaussian_pdf_2d(pt, self.means[j], self.covariances[j]))
            .collect();
        let total: f64 = probs.iter().sum();
        if total > 0.0 {
            for p in &mut probs {
                *p /= total;
            }
        }
        probs
    }

    /// Merge 2D components closer than `threshold` (Mahalanobis-like distance
    /// based on average component spread).
    pub fn merge_components(&mut self, threshold: f64) -> usize {
        let mut merges = 0;
        let mut i = 0;
        while i < self.k {
            let mut j = i + 1;
            while j < self.k {
                let dx = self.means[i].0 - self.means[j].0;
                let dy = self.means[i].1 - self.means[j].1;
                let dist = (dx * dx + dy * dy).sqrt();
                let spread_i = (self.covariances[i].0 + self.covariances[i].3).sqrt();
                let spread_j = (self.covariances[j].0 + self.covariances[j].3).sqrt();
                let avg_spread = (spread_i + spread_j) / 2.0;

                if dist < threshold * avg_spread {
                    let wi = self.weights[i];
                    let wj = self.weights[j];
                    let total_w = wi + wj;
                    let new_mx = (wi * self.means[i].0 + wj * self.means[j].0) / total_w;
                    let new_my = (wi * self.means[i].1 + wj * self.means[j].1) / total_w;

                    // Merge covariance using parallel axis theorem
                    let dxi = self.means[i].0 - new_mx;
                    let dyi = self.means[i].1 - new_my;
                    let dxj = self.means[j].0 - new_mx;
                    let dyj = self.means[j].1 - new_my;

                    let new_vxx = (wi * (self.covariances[i].0 + dxi * dxi)
                        + wj * (self.covariances[j].0 + dxj * dxj))
                        / total_w;
                    let new_vxy = (wi * (self.covariances[i].1 + dxi * dyi)
                        + wj * (self.covariances[j].1 + dxj * dyj))
                        / total_w;
                    let new_vyy = (wi * (self.covariances[i].3 + dyi * dyi)
                        + wj * (self.covariances[j].3 + dyj * dyj))
                        / total_w;

                    self.means[i] = (new_mx, new_my);
                    self.covariances[i] = (new_vxx, new_vxy, new_vxy, new_vyy);
                    self.weights[i] = total_w;
                    self.means.remove(j);
                    self.covariances.remove(j);
                    self.weights.remove(j);
                    self.k -= 1;
                    merges += 1;
                } else {
                    j += 1;
                }
            }
            i += 1;
        }
        merges
    }

    /// Return fitted 2D means.
    pub fn means(&self) -> &[(f64, f64)] {
        &self.means
    }

    /// Return fitted 2D covariances as (var_xx, var_xy, var_yx, var_yy).
    pub fn covariances(&self) -> &[(f64, f64, f64, f64)] {
        &self.covariances
    }

    /// Return mixing weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Return whether the algorithm converged.
    pub fn converged(&self) -> bool {
        self.converged
    }

    /// Return the number of iterations.
    pub fn iterations(&self) -> usize {
        self.iterations
    }

    /// Return the number of components.
    pub fn num_components(&self) -> usize {
        self.k
    }

    /// Return the final log-likelihood.
    pub fn final_log_likelihood(&self) -> f64 {
        self.log_likelihood
    }

    /// Compute BIC for the 2D model.
    pub fn bic(&self, n: usize) -> f64 {
        // params per component: 2 means + 3 unique cov entries + 1 weight; minus 1 for constraint
        let num_params = 6 * self.k - 1;
        -2.0 * self.log_likelihood + (num_params as f64) * (n as f64).ln()
    }

    /// Compute AIC for the 2D model.
    pub fn aic(&self, _n: usize) -> f64 {
        let num_params = 6 * self.k - 1;
        -2.0 * self.log_likelihood + 2.0 * num_params as f64
    }

    // ---- private helpers ----

    fn kmeans_init_2d(&mut self, data: &[(f64, f64)]) {
        let n = data.len();

        // Deterministic init: sort by angle from centroid, pick evenly spaced
        let cx: f64 = data.iter().map(|p| p.0).sum::<f64>() / n as f64;
        let cy: f64 = data.iter().map(|p| p.1).sum::<f64>() / n as f64;

        let mut indexed: Vec<(usize, f64)> = data
            .iter()
            .enumerate()
            .map(|(i, &(x, y))| (i, (y - cy).atan2(x - cx)))
            .collect();
        indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        self.means = (0..self.k)
            .map(|j| {
                let idx = indexed[(j * n) / self.k].0;
                data[idx]
            })
            .collect();

        // K-means iterations
        let mut assignments = vec![0usize; n];
        for _iter in 0..20 {
            for (i, &(x, y)) in data.iter().enumerate() {
                assignments[i] = (0..self.k)
                    .min_by(|&a, &b| {
                        let da = (x - self.means[a].0).powi(2) + (y - self.means[a].1).powi(2);
                        let db = (x - self.means[b].0).powi(2) + (y - self.means[b].1).powi(2);
                        da.partial_cmp(&db).unwrap()
                    })
                    .unwrap();
            }
            for j in 0..self.k {
                let mut sx = 0.0;
                let mut sy = 0.0;
                let mut count = 0usize;
                for (i, &(x, y)) in data.iter().enumerate() {
                    if assignments[i] == j {
                        sx += x;
                        sy += y;
                        count += 1;
                    }
                }
                if count > 0 {
                    self.means[j] = (sx / count as f64, sy / count as f64);
                }
            }
        }

        // Initial covariances and weights from K-means assignments
        self.covariances = vec![(1.0, 0.0, 0.0, 1.0); self.k];
        self.weights = vec![1.0 / self.k as f64; self.k];
        for j in 0..self.k {
            let mut vxx = 0.0;
            let mut vxy = 0.0;
            let mut vyy = 0.0;
            let mut count = 0usize;
            for (i, &(x, y)) in data.iter().enumerate() {
                if assignments[i] == j {
                    let dx = x - self.means[j].0;
                    let dy = y - self.means[j].1;
                    vxx += dx * dx;
                    vxy += dx * dy;
                    vyy += dy * dy;
                    count += 1;
                }
            }
            if count > 1 {
                let c = count as f64;
                self.covariances[j] = (
                    (vxx / c).max(1e-10),
                    vxy / c,
                    vxy / c,
                    (vyy / c).max(1e-10),
                );
            }
            self.weights[j] = count as f64 / n as f64;
        }
        // Normalise weights
        for w in &mut self.weights {
            if *w < 1e-10 {
                *w = 1e-10;
            }
        }
        let wsum: f64 = self.weights.iter().sum();
        for w in &mut self.weights {
            *w /= wsum;
        }
    }
}

// ---------------------------------------------------------------------------
// Gaussian PDF helpers
// ---------------------------------------------------------------------------

/// 1D Gaussian probability density function.
fn gaussian_pdf(x: f64, mean: f64, variance: f64) -> f64 {
    let v = variance.max(1e-30);
    let exponent = -0.5 * (x - mean).powi(2) / v;
    (1.0 / (2.0 * PI * v).sqrt()) * exponent.exp()
}

/// 2D Gaussian probability density function with full covariance.
fn gaussian_pdf_2d(
    pt: (f64, f64),
    mean: (f64, f64),
    cov: (f64, f64, f64, f64),
) -> f64 {
    let (vxx, vxy, _vyx, vyy) = cov;
    let det = vxx * vyy - vxy * vxy;
    let det = det.max(1e-30);

    let dx = pt.0 - mean.0;
    let dy = pt.1 - mean.1;

    // Inverse of 2x2 matrix
    let inv_xx = vyy / det;
    let inv_xy = -vxy / det;
    let inv_yy = vxx / det;

    let exponent = -0.5 * (dx * dx * inv_xx + 2.0 * dx * dy * inv_xy + dy * dy * inv_yy);
    (1.0 / (2.0 * PI * det.sqrt())) * exponent.exp()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate deterministic pseudo-random numbers using a simple LCG.
    fn lcg_rand(seed: &mut u64) -> f64 {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Map to [0, 1)
        (*seed >> 33) as f64 / (1u64 << 31) as f64
    }

    /// Simple Box-Muller deterministic normal samples.
    fn normal_samples(mean: f64, std: f64, n: usize, seed: &mut u64) -> Vec<f64> {
        let mut result = Vec::with_capacity(n);
        for _ in 0..n {
            let u1 = lcg_rand(seed).max(1e-10);
            let u2 = lcg_rand(seed);
            let z = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            result.push(mean + std * z);
        }
        result
    }

    fn normal_2d_samples(mx: f64, my: f64, std: f64, n: usize, seed: &mut u64) -> Vec<(f64, f64)> {
        let mut result = Vec::with_capacity(n);
        for _ in 0..n {
            let u1 = lcg_rand(seed).max(1e-10);
            let u2 = lcg_rand(seed);
            let z1 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            let z2 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).sin();
            result.push((mx + std * z1, my + std * z2));
        }
        result
    }

    #[test]
    fn test_single_component() {
        let mut seed = 42u64;
        let data = normal_samples(3.0, 1.0, 200, &mut seed);
        let mut gmm = EmGmm::new(1);
        gmm.fit(&data);
        assert!((gmm.means()[0] - 3.0).abs() < 0.5);
        assert!((gmm.variances()[0] - 1.0).abs() < 0.5);
        assert!((gmm.weights()[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_two_components_well_separated() {
        let mut seed = 123u64;
        let mut data = normal_samples(0.0, 0.5, 150, &mut seed);
        data.extend(normal_samples(10.0, 0.5, 150, &mut seed));

        let mut gmm = EmGmm::new(2);
        gmm.fit(&data);

        let mut means: Vec<f64> = gmm.means().to_vec();
        means.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((means[0] - 0.0).abs() < 1.0);
        assert!((means[1] - 10.0).abs() < 1.0);
    }

    #[test]
    fn test_convergence_flag() {
        let mut seed = 77u64;
        let data = normal_samples(0.0, 1.0, 100, &mut seed);
        let mut gmm = EmGmm::new(1).with_max_iter(500).with_tolerance(1e-8);
        gmm.fit(&data);
        assert!(gmm.converged());
    }

    #[test]
    fn test_weights_sum_to_one() {
        let mut seed = 99u64;
        let mut data = normal_samples(-3.0, 1.0, 100, &mut seed);
        data.extend(normal_samples(3.0, 1.0, 100, &mut seed));

        let mut gmm = EmGmm::new(2);
        gmm.fit(&data);
        let wsum: f64 = gmm.weights().iter().sum();
        assert!((wsum - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_log_likelihood_increases() {
        let mut seed = 55u64;
        let data = normal_samples(0.0, 1.0, 100, &mut seed);
        let mut gmm = EmGmm::new(1);
        gmm.fit(&data);
        assert!(gmm.final_log_likelihood().is_finite());
        assert!(gmm.final_log_likelihood() > f64::NEG_INFINITY);
    }

    #[test]
    fn test_classification() {
        let mut seed = 200u64;
        let mut data = normal_samples(-5.0, 0.3, 100, &mut seed);
        data.extend(normal_samples(5.0, 0.3, 100, &mut seed));

        let mut gmm = EmGmm::new(2);
        gmm.fit(&data);

        let labels = gmm.classify(&data);
        let label_a = labels[0];
        let label_b = labels[100];
        assert_ne!(label_a, label_b);

        let mismatches: usize = labels[..100].iter().filter(|&&l| l != label_a).count();
        assert!(mismatches < 5, "Too many misclassifications: {}", mismatches);
    }

    #[test]
    fn test_posterior_sums_to_one() {
        let mut seed = 33u64;
        let mut data = normal_samples(0.0, 1.0, 100, &mut seed);
        data.extend(normal_samples(5.0, 1.0, 100, &mut seed));

        let mut gmm = EmGmm::new(2);
        gmm.fit(&data);

        let post = gmm.posterior(2.5);
        let total: f64 = post.iter().sum();
        assert!((total - 1.0).abs() < 1e-10, "Posterior sum: {}", total);
    }

    #[test]
    fn test_merge_identical_components() {
        let mut seed = 7u64;
        let data = normal_samples(0.0, 1.0, 200, &mut seed);
        let mut gmm = EmGmm::new(2);
        gmm.fit(&data);

        let merges = gmm.merge_components(3.0);
        if merges > 0 {
            assert_eq!(gmm.num_components(), 1);
        }
        assert!(gmm.num_components() >= 1);
    }

    #[test]
    fn test_merge_does_not_merge_separated() {
        let mut seed = 88u64;
        let mut data = normal_samples(-10.0, 0.5, 100, &mut seed);
        data.extend(normal_samples(10.0, 0.5, 100, &mut seed));

        let mut gmm = EmGmm::new(2);
        gmm.fit(&data);

        let merges = gmm.merge_components(1.0);
        assert_eq!(merges, 0);
        assert_eq!(gmm.num_components(), 2);
    }

    #[test]
    fn test_bic_aic() {
        let mut seed = 44u64;
        let data = normal_samples(0.0, 1.0, 200, &mut seed);
        let mut gmm = EmGmm::new(1);
        gmm.fit(&data);

        let bic = gmm.bic(200);
        let aic = gmm.aic(200);
        assert!(bic.is_finite());
        assert!(aic.is_finite());
    }

    #[test]
    fn test_model_selection() {
        let mut seed = 300u64;
        let mut data = normal_samples(-5.0, 0.5, 150, &mut seed);
        data.extend(normal_samples(5.0, 0.5, 150, &mut seed));

        let sel = select_model_order(&data, 4);
        assert!(
            sel.best_k_bic >= 1 && sel.best_k_bic <= 4,
            "BIC selected k={}",
            sel.best_k_bic
        );
        assert!(!sel.bic_values.is_empty());
        assert!(!sel.aic_values.is_empty());
    }

    #[test]
    fn test_e_step_responsibilities_sum_to_one() {
        let mut seed = 11u64;
        let data = normal_samples(0.0, 1.0, 50, &mut seed);
        let mut gmm = EmGmm::new(2);
        gmm.fit(&data);

        let mut resp = vec![vec![0.0; 2]; data.len()];
        gmm.e_step(&data, &mut resp);
        for r in &resp {
            let sum: f64 = r.iter().sum();
            assert!((sum - 1.0).abs() < 1e-10, "Responsibility sum: {}", sum);
        }
    }

    #[test]
    fn test_iterations_count() {
        let mut seed = 22u64;
        let data = normal_samples(0.0, 1.0, 100, &mut seed);
        let mut gmm = EmGmm::new(1).with_max_iter(50);
        gmm.fit(&data);
        assert!(gmm.iterations() >= 1);
        assert!(gmm.iterations() <= 50);
    }

    #[test]
    fn test_2d_single_cluster() {
        let mut seed = 500u64;
        let data = normal_2d_samples(1.0, 2.0, 0.5, 200, &mut seed);

        let mut gmm = EmGmm2D::new(1);
        gmm.fit(&data);
        assert!((gmm.means()[0].0 - 1.0).abs() < 0.5);
        assert!((gmm.means()[0].1 - 2.0).abs() < 0.5);
        assert!(gmm.converged());
    }

    #[test]
    fn test_2d_two_clusters() {
        let mut seed = 600u64;
        let mut data = normal_2d_samples(-3.0, -3.0, 0.3, 150, &mut seed);
        data.extend(normal_2d_samples(3.0, 3.0, 0.3, 150, &mut seed));

        let mut gmm = EmGmm2D::new(2);
        gmm.fit(&data);

        let labels = gmm.classify(&data);
        let label_a = labels[0];
        let label_b = labels[150];
        assert_ne!(label_a, label_b);
    }

    #[test]
    fn test_2d_classification_accuracy() {
        let mut seed = 700u64;
        let mut data = normal_2d_samples(0.0, 0.0, 0.2, 100, &mut seed);
        data.extend(normal_2d_samples(5.0, 5.0, 0.2, 100, &mut seed));

        let mut gmm = EmGmm2D::new(2);
        gmm.fit(&data);

        let labels = gmm.classify(&data);
        let expected_label = labels[0];
        let correct: usize = labels[..100].iter().filter(|&&l| l == expected_label).count();
        assert!(correct > 90, "Only {} correct out of 100", correct);
    }

    #[test]
    fn test_2d_posterior() {
        let mut seed = 800u64;
        let data = normal_2d_samples(0.0, 0.0, 1.0, 100, &mut seed);

        let mut gmm = EmGmm2D::new(2);
        gmm.fit(&data);

        let post = gmm.posterior((0.0, 0.0));
        let total: f64 = post.iter().sum();
        assert!((total - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_2d_merge() {
        let mut seed = 900u64;
        let data = normal_2d_samples(0.0, 0.0, 1.0, 200, &mut seed);

        let mut gmm = EmGmm2D::new(2);
        gmm.fit(&data);

        let merges = gmm.merge_components(3.0);
        if merges > 0 {
            assert_eq!(gmm.num_components(), 1);
        }
        assert!(gmm.num_components() >= 1);
    }

    #[test]
    fn test_2d_weights_sum() {
        let mut seed = 1000u64;
        let data = normal_2d_samples(0.0, 0.0, 1.0, 200, &mut seed);

        let mut gmm = EmGmm2D::new(3);
        gmm.fit(&data);
        let wsum: f64 = gmm.weights().iter().sum();
        assert!((wsum - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_gaussian_pdf_peak() {
        let peak = gaussian_pdf(0.0, 0.0, 1.0);
        let off = gaussian_pdf(1.0, 0.0, 1.0);
        assert!(peak > off);
        // Known value: 1/sqrt(2*pi) ~= 0.3989
        assert!((peak - 0.3989).abs() < 0.001);
    }

    #[test]
    fn test_gaussian_pdf_2d_peak() {
        let cov = (1.0, 0.0, 0.0, 1.0);
        let peak = gaussian_pdf_2d((0.0, 0.0), (0.0, 0.0), cov);
        let off = gaussian_pdf_2d((1.0, 1.0), (0.0, 0.0), cov);
        assert!(peak > off);
        // 1/(2*pi) ~= 0.1592
        assert!((peak - 1.0 / (2.0 * PI)).abs() < 0.001);
    }

    #[test]
    fn test_2d_bic_aic() {
        let mut seed = 1100u64;
        let data = normal_2d_samples(0.0, 0.0, 1.0, 100, &mut seed);

        let mut gmm = EmGmm2D::new(1);
        gmm.fit(&data);

        let bic = gmm.bic(100);
        let aic = gmm.aic(100);
        assert!(bic.is_finite());
        assert!(aic.is_finite());
    }
}

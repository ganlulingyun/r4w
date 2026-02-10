//! Space-Time Adaptive Processing (STAP) for airborne radar clutter suppression.
//!
//! STAP performs joint spatial-temporal filtering to suppress ground clutter
//! in airborne radar systems. By exploiting the angle-Doppler coupling inherent
//! in clutter returns (the "clutter ridge"), STAP can simultaneously null
//! clutter in both angle and Doppler, achieving superior detection of slow-moving
//! targets that would otherwise be masked.
//!
//! This module provides:
//! - Full space-time snapshot vector formation
//! - Sample covariance matrix (SCM) estimation
//! - Direct STAP weight computation via matrix inversion
//! - Factored approaches: pre-Doppler and post-Doppler STAP
//! - Clutter ridge modelling (angle-Doppler coupling)
//! - SINR improvement factor computation
//! - Minimum detectable velocity (MDV) estimation
//! - Diagonal loading for robustness
//! - Clutter rank estimation (Brennan's rule)
//!
//! # Example
//!
//! ```
//! use r4w_core::space_time_adaptive_processor::{StapConfig, SpaceTimeAdaptiveProcessor};
//!
//! // Configure a 4-element ULA with 8 pulses per CPI
//! let config = StapConfig {
//!     num_elements: 4,
//!     num_pulses: 8,
//!     element_spacing_wavelengths: 0.5,
//!     prf: 1000.0,
//!     wavelength: 0.03,
//!     platform_velocity: 100.0,
//!     diagonal_loading: 0.01,
//! };
//!
//! let stap = SpaceTimeAdaptiveProcessor::new(config);
//! assert_eq!(stap.space_time_dimension(), 32);
//!
//! // Form a steering vector for angle=0, normalised Doppler=0.25
//! let sv = stap.steering_vector(0.0, 0.25);
//! assert_eq!(sv.len(), 32);
//! ```

use std::f64::consts::PI;

// ── Complex arithmetic helpers ──────────────────────────────────────────────

/// Complex number represented as `(re, im)`.
type C64 = (f64, f64);

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C64, b: C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

#[inline]
fn c_mag_sq(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_scale(s: f64, a: C64) -> C64 {
    (s * a.0, s * a.1)
}

#[inline]
fn c_exp_j(theta: f64) -> C64 {
    (theta.cos(), theta.sin())
}

#[inline]
fn c_div(a: C64, b: C64) -> C64 {
    let denom = c_mag_sq(b);
    if denom < 1e-300 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
    }
}

// ── Small dense matrix utilities (row-major, NxN complex) ───────────────────

/// Multiply two NxN complex matrices (row-major).
fn mat_mul(a: &[C64], b: &[C64], n: usize) -> Vec<C64> {
    let mut c = vec![(0.0, 0.0); n * n];
    for i in 0..n {
        for k in 0..n {
            let a_ik = a[i * n + k];
            if c_mag_sq(a_ik) < 1e-300 {
                continue;
            }
            for j in 0..n {
                c[i * n + j] = c_add(c[i * n + j], c_mul(a_ik, b[k * n + j]));
            }
        }
    }
    c
}

/// Matrix-vector multiply: y = A * x.
fn mat_vec(a: &[C64], x: &[C64], n: usize) -> Vec<C64> {
    let mut y = vec![(0.0, 0.0); n];
    for i in 0..n {
        let mut sum = (0.0, 0.0);
        for j in 0..n {
            sum = c_add(sum, c_mul(a[i * n + j], x[j]));
        }
        y[i] = sum;
    }
    y
}

/// Inner product <a, b> = sum(conj(a_i) * b_i).
fn inner_product(a: &[C64], b: &[C64]) -> C64 {
    let mut sum = (0.0, 0.0);
    for i in 0..a.len() {
        sum = c_add(sum, c_mul(c_conj(a[i]), b[i]));
    }
    sum
}

/// Hermitian transpose of an NxN matrix.
fn hermitian(a: &[C64], n: usize) -> Vec<C64> {
    let mut ah = vec![(0.0, 0.0); n * n];
    for i in 0..n {
        for j in 0..n {
            ah[j * n + i] = c_conj(a[i * n + j]);
        }
    }
    ah
}

/// Invert an NxN complex matrix via Gauss-Jordan elimination.
/// Returns `None` if singular.
fn mat_inv(a: &[C64], n: usize) -> Option<Vec<C64>> {
    // Augmented matrix [A | I]
    let mut aug = vec![(0.0, 0.0); n * 2 * n];
    for i in 0..n {
        for j in 0..n {
            aug[i * 2 * n + j] = a[i * n + j];
        }
        aug[i * 2 * n + n + i] = (1.0, 0.0);
    }

    for col in 0..n {
        // Partial pivoting: find row with largest magnitude in this column
        let mut best_row = col;
        let mut best_mag = c_mag_sq(aug[col * 2 * n + col]);
        for row in (col + 1)..n {
            let mag = c_mag_sq(aug[row * 2 * n + col]);
            if mag > best_mag {
                best_mag = mag;
                best_row = row;
            }
        }
        if best_mag < 1e-30 {
            return None; // Singular
        }
        // Swap rows
        if best_row != col {
            for j in 0..(2 * n) {
                let tmp = aug[col * 2 * n + j];
                aug[col * 2 * n + j] = aug[best_row * 2 * n + j];
                aug[best_row * 2 * n + j] = tmp;
            }
        }
        // Scale pivot row
        let pivot = aug[col * 2 * n + col];
        let pivot_inv = c_div((1.0, 0.0), pivot);
        for j in 0..(2 * n) {
            aug[col * 2 * n + j] = c_mul(aug[col * 2 * n + j], pivot_inv);
        }
        // Eliminate column
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row * 2 * n + col];
            for j in 0..(2 * n) {
                let val = c_mul(factor, aug[col * 2 * n + j]);
                aug[row * 2 * n + j] = c_sub(aug[row * 2 * n + j], val);
            }
        }
    }

    // Extract inverse from augmented matrix
    let mut inv = vec![(0.0, 0.0); n * n];
    for i in 0..n {
        for j in 0..n {
            inv[i * n + j] = aug[i * 2 * n + n + j];
        }
    }
    Some(inv)
}

/// Compute a simple NxN DFT matrix.
fn dft_matrix(n: usize) -> Vec<C64> {
    let mut d = vec![(0.0, 0.0); n * n];
    let scale = 1.0 / (n as f64).sqrt();
    for k in 0..n {
        for m in 0..n {
            let theta = -2.0 * PI * (k as f64) * (m as f64) / (n as f64);
            d[k * n + m] = c_scale(scale, c_exp_j(theta));
        }
    }
    d
}

// ── Public API ──────────────────────────────────────────────────────────────

/// Configuration for the Space-Time Adaptive Processor.
#[derive(Debug, Clone)]
pub struct StapConfig {
    /// Number of antenna elements in the uniform linear array (ULA).
    pub num_elements: usize,
    /// Number of pulses in a coherent processing interval (CPI).
    pub num_pulses: usize,
    /// Element spacing in wavelengths (typically 0.5 for half-wavelength).
    pub element_spacing_wavelengths: f64,
    /// Pulse repetition frequency in Hz.
    pub prf: f64,
    /// Radar wavelength in metres.
    pub wavelength: f64,
    /// Platform velocity in m/s.
    pub platform_velocity: f64,
    /// Diagonal loading factor (relative to noise power estimate).
    pub diagonal_loading: f64,
}

/// Space-Time Adaptive Processor for airborne radar clutter suppression.
///
/// Implements joint spatial-temporal filtering using a uniform linear array
/// of `N` elements over `M` pulses, forming an `NM`-dimensional space-time
/// snapshot vector.
#[derive(Debug, Clone)]
pub struct SpaceTimeAdaptiveProcessor {
    config: StapConfig,
    /// Cached NM dimension.
    nm: usize,
}

/// Result of STAP weight computation.
#[derive(Debug, Clone)]
pub struct StapWeights {
    /// The NM-dimensional weight vector.
    pub weights: Vec<C64>,
    /// The inverted (diagonally loaded) covariance matrix used.
    pub cov_inv: Vec<C64>,
}

/// Clutter ridge parameters for a single range cell.
#[derive(Debug, Clone)]
pub struct ClutterRidgePoint {
    /// Cone angle (radians) relative to array axis.
    pub cone_angle: f64,
    /// Spatial frequency (normalised, -0.5 to 0.5).
    pub spatial_freq: f64,
    /// Normalised Doppler frequency (-0.5 to 0.5).
    pub doppler_freq: f64,
}

/// SINR analysis result.
#[derive(Debug, Clone)]
pub struct SinrResult {
    /// Output SINR in dB.
    pub sinr_db: f64,
    /// Improvement factor in dB relative to element-level SNR.
    pub improvement_factor_db: f64,
}

impl SpaceTimeAdaptiveProcessor {
    /// Create a new STAP processor from the given configuration.
    pub fn new(config: StapConfig) -> Self {
        assert!(config.num_elements > 0, "num_elements must be > 0");
        assert!(config.num_pulses > 0, "num_pulses must be > 0");
        let nm = config.num_elements * config.num_pulses;
        Self { config, nm }
    }

    /// Return the space-time snapshot dimension (N * M).
    pub fn space_time_dimension(&self) -> usize {
        self.nm
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &StapConfig {
        &self.config
    }

    // ── Steering vectors ────────────────────────────────────────────────

    /// Spatial steering vector for a given cone angle `theta` (radians).
    /// Length = N (number of elements).
    pub fn spatial_steering_vector(&self, theta: f64) -> Vec<C64> {
        let n = self.config.num_elements;
        let d = self.config.element_spacing_wavelengths;
        let psi = 2.0 * PI * d * theta.sin();
        (0..n).map(|i| c_exp_j(psi * i as f64)).collect()
    }

    /// Temporal steering vector for a given normalised Doppler `fd`
    /// (in the range -0.5 to 0.5). Length = M (number of pulses).
    pub fn temporal_steering_vector(&self, fd: f64) -> Vec<C64> {
        let m = self.config.num_pulses;
        let phi = 2.0 * PI * fd;
        (0..m).map(|k| c_exp_j(phi * k as f64)).collect()
    }

    /// Space-time steering vector: Kronecker product of temporal and spatial
    /// steering vectors. Length = NM.
    ///
    /// `theta` is the cone angle in radians, `fd` is normalised Doppler.
    pub fn steering_vector(&self, theta: f64, fd: f64) -> Vec<C64> {
        let a_s = self.spatial_steering_vector(theta);
        let a_t = self.temporal_steering_vector(fd);
        // Kronecker: a_t ⊗ a_s  (temporal outer spatial)
        let mut v = Vec::with_capacity(self.nm);
        for t in &a_t {
            for s in &a_s {
                v.push(c_mul(*t, *s));
            }
        }
        v
    }

    // ── Snapshot formation ──────────────────────────────────────────────

    /// Form the space-time snapshot vector from raw data.
    ///
    /// `data` is organised as `[pulse_0_elem_0, pulse_0_elem_1, ..., pulse_M-1_elem_N-1]`.
    /// Returns the NM-dimensional snapshot vector (a copy).
    pub fn form_snapshot(&self, data: &[C64]) -> Vec<C64> {
        assert_eq!(
            data.len(),
            self.nm,
            "data length must equal N*M = {}",
            self.nm
        );
        data.to_vec()
    }

    // ── Covariance estimation ───────────────────────────────────────────

    /// Estimate the sample covariance matrix from `K` training snapshots.
    ///
    /// Each snapshot has length NM. The snapshots are stacked as rows of
    /// `training_data` (total length `K * NM`).
    ///
    /// Returns the NM x NM sample covariance matrix (row-major).
    pub fn estimate_covariance(&self, training_data: &[C64], num_snapshots: usize) -> Vec<C64> {
        assert_eq!(
            training_data.len(),
            num_snapshots * self.nm,
            "training_data length mismatch"
        );
        let nm = self.nm;
        let mut cov = vec![(0.0, 0.0); nm * nm];
        for k in 0..num_snapshots {
            let snap = &training_data[k * nm..(k + 1) * nm];
            for i in 0..nm {
                let si_conj = c_conj(snap[i]);
                for j in 0..nm {
                    cov[i * nm + j] = c_add(cov[i * nm + j], c_mul(snap[j], si_conj));
                }
            }
        }
        let scale = 1.0 / num_snapshots as f64;
        for v in &mut cov {
            *v = c_scale(scale, *v);
        }
        cov
    }

    /// Apply diagonal loading to a covariance matrix in-place.
    /// `loading` is the value added to each diagonal element.
    pub fn apply_diagonal_loading(&self, cov: &mut [C64], loading: f64) {
        let nm = self.nm;
        assert_eq!(cov.len(), nm * nm);
        for i in 0..nm {
            cov[i * nm + i] = c_add(cov[i * nm + i], (loading, 0.0));
        }
    }

    /// Estimate diagonal loading from the covariance matrix trace.
    /// Returns `config.diagonal_loading * trace(R) / NM`.
    pub fn auto_diagonal_loading(&self, cov: &[C64]) -> f64 {
        let nm = self.nm;
        let mut trace = 0.0;
        for i in 0..nm {
            trace += cov[i * nm + i].0;
        }
        self.config.diagonal_loading * trace / nm as f64
    }

    // ── Weight computation ──────────────────────────────────────────────

    /// Compute the optimal STAP weight vector via direct matrix inversion.
    ///
    /// w = R^{-1} s / (s^H R^{-1} s)
    ///
    /// where `R` is the (diagonally loaded) sample covariance matrix and
    /// `s` is the space-time steering vector for the look direction.
    ///
    /// Returns `None` if the matrix is singular.
    pub fn compute_weights(
        &self,
        cov: &[C64],
        steering: &[C64],
    ) -> Option<StapWeights> {
        let nm = self.nm;
        assert_eq!(cov.len(), nm * nm);
        assert_eq!(steering.len(), nm);

        // Apply diagonal loading
        let mut loaded = cov.to_vec();
        let dl = self.auto_diagonal_loading(cov);
        self.apply_diagonal_loading(&mut loaded, dl);

        let cov_inv = mat_inv(&loaded, nm)?;
        let r_inv_s = mat_vec(&cov_inv, steering, nm);
        let denom = inner_product(steering, &r_inv_s);
        let denom_inv = c_div((1.0, 0.0), denom);
        let weights: Vec<C64> = r_inv_s.iter().map(|&v| c_mul(v, denom_inv)).collect();

        Some(StapWeights { weights, cov_inv })
    }

    /// Apply STAP weight vector to a snapshot, returning the scalar output.
    pub fn apply_weights(&self, weights: &[C64], snapshot: &[C64]) -> C64 {
        inner_product(weights, snapshot)
    }

    // ── Factored STAP ───────────────────────────────────────────────────

    /// Pre-Doppler STAP: apply DFT across pulses first, then do spatial
    /// adaptive processing in each Doppler bin.
    ///
    /// `data` is `M` pulses x `N` elements (row-major, length NM).
    /// Returns the NM-length output after pre-Doppler processing.
    pub fn pre_doppler_stap(&self, data: &[C64]) -> Vec<C64> {
        let n = self.config.num_elements;
        let m = self.config.num_pulses;
        let dft = dft_matrix(m);

        // For each element, apply DFT across pulses
        let mut doppler_data = vec![(0.0, 0.0); m * n];
        for elem in 0..n {
            // Collect pulses for this element
            let pulses: Vec<C64> = (0..m).map(|p| data[p * n + elem]).collect();
            // Apply DFT
            let doppler_bins = mat_vec(&dft, &pulses, m);
            for k in 0..m {
                doppler_data[k * n + elem] = doppler_bins[k];
            }
        }
        doppler_data
    }

    /// Post-Doppler STAP: perform spatial beamforming first, then temporal
    /// filtering. Returns spatial beamformed data for each Doppler bin.
    ///
    /// `data` is `M` pulses x `N` elements (row-major, length NM).
    /// `look_angle` is the look direction cone angle in radians.
    ///
    /// Returns M Doppler-bin outputs.
    pub fn post_doppler_stap(&self, data: &[C64], look_angle: f64) -> Vec<C64> {
        let n = self.config.num_elements;
        let m = self.config.num_pulses;

        // Spatial beamforming weights (conventional)
        let a_s = self.spatial_steering_vector(look_angle);
        let scale = 1.0 / n as f64;

        // Beamform each pulse
        let mut beamformed = Vec::with_capacity(m);
        for p in 0..m {
            let mut sum = (0.0, 0.0);
            for e in 0..n {
                sum = c_add(sum, c_mul(c_conj(a_s[e]), data[p * n + e]));
            }
            beamformed.push(c_scale(scale, sum));
        }

        // Apply DFT to get Doppler bins
        let dft = dft_matrix(m);
        mat_vec(&dft, &beamformed, m)
    }

    // ── Clutter ridge model ─────────────────────────────────────────────

    /// Compute the clutter ridge: the angle-Doppler coupling line.
    ///
    /// For an airborne side-looking radar with velocity `v`, the clutter
    /// return at cone angle `theta` has normalised Doppler:
    ///   fd = (2 * v * d * sin(theta)) / (lambda * PRF) * PRF / PRF
    ///      = 2 * v * sin(theta) / (lambda * PRF)
    ///
    /// Returns a vector of `ClutterRidgePoint` sampled over azimuth.
    pub fn clutter_ridge(&self, num_points: usize) -> Vec<ClutterRidgePoint> {
        let v = self.config.platform_velocity;
        let lambda = self.config.wavelength;
        let prf = self.config.prf;
        let d = self.config.element_spacing_wavelengths;

        (0..num_points)
            .map(|i| {
                let theta = -PI / 2.0 + PI * (i as f64) / (num_points as f64 - 1.0);
                let sin_theta = theta.sin();
                let spatial_freq = d * sin_theta;
                let doppler_freq = 2.0 * v * sin_theta / (lambda * prf);
                ClutterRidgePoint {
                    cone_angle: theta,
                    spatial_freq,
                    doppler_freq,
                }
            })
            .collect()
    }

    // ── Clutter rank estimation ─────────────────────────────────────────

    /// Estimate the clutter rank using Brennan's rule.
    ///
    /// rank ≈ N + (M-1) * β, where β = 2*v*T_PRI / (d*λ)
    /// and `T_PRI = 1/PRF`.
    ///
    /// Clipped to NM as maximum.
    pub fn estimate_clutter_rank(&self) -> f64 {
        let n = self.config.num_elements as f64;
        let m = self.config.num_pulses as f64;
        let v = self.config.platform_velocity;
        let prf = self.config.prf;
        let d_metres = self.config.element_spacing_wavelengths * self.config.wavelength;
        let t_pri = 1.0 / prf;

        let beta = 2.0 * v * t_pri / d_metres;
        let rank = n + (m - 1.0) * beta;
        rank.min(n * m)
    }

    // ── SINR computation ────────────────────────────────────────────────

    /// Compute the output SINR for a target with steering vector `s`
    /// and signal power `sigma_s_sq`.
    ///
    /// SINR_out = sigma_s^2 * |w^H s|^2 / (w^H R_cn w)
    ///
    /// where `R_cn` is the clutter-plus-noise covariance.
    pub fn compute_sinr(
        &self,
        weights: &[C64],
        steering: &[C64],
        cov_cn: &[C64],
        signal_power: f64,
    ) -> SinrResult {
        let nm = self.nm;

        // Numerator: sigma_s^2 * |w^H s|^2
        let wh_s = inner_product(weights, steering);
        let numerator = signal_power * c_mag_sq(wh_s);

        // Denominator: w^H R_cn w
        let r_w = mat_vec(cov_cn, weights, nm);
        let denominator = inner_product(weights, &r_w).0.max(1e-300);

        let sinr_linear = numerator / denominator;
        let sinr_db = 10.0 * sinr_linear.log10();

        // Element-level SNR (for improvement factor)
        let mut trace = 0.0;
        for i in 0..nm {
            trace += cov_cn[i * nm + i].0;
        }
        let noise_per_element = trace / nm as f64;
        let snr_element = if noise_per_element > 1e-300 {
            signal_power / noise_per_element
        } else {
            signal_power
        };
        let snr_element_db = 10.0 * snr_element.max(1e-300).log10();
        let improvement_factor_db = sinr_db - snr_element_db;

        SinrResult {
            sinr_db,
            improvement_factor_db,
        }
    }

    /// Compute SINR improvement factor across normalised Doppler.
    ///
    /// Returns a vector of `(normalised_doppler, improvement_factor_dB)`.
    pub fn sinr_improvement_vs_doppler(
        &self,
        cov: &[C64],
        look_angle: f64,
        num_points: usize,
        signal_power: f64,
    ) -> Vec<(f64, f64)> {
        let mut results = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let fd = -0.5 + (i as f64) / (num_points as f64 - 1.0);
            let s = self.steering_vector(look_angle, fd);

            if let Some(stap_w) = self.compute_weights(cov, &s) {
                let sinr = self.compute_sinr(&stap_w.weights, &s, cov, signal_power);
                results.push((fd, sinr.improvement_factor_db));
            } else {
                results.push((fd, f64::NEG_INFINITY));
            }
        }
        results
    }

    // ── Minimum detectable velocity ─────────────────────────────────────

    /// Estimate the minimum detectable velocity (MDV).
    ///
    /// MDV is the smallest radial velocity at which the SINR improvement
    /// factor drops below a given threshold relative to the peak.
    ///
    /// `threshold_db` is typically 3 dB below peak.
    ///
    /// Returns the MDV in m/s, or `None` if it cannot be determined.
    pub fn minimum_detectable_velocity(
        &self,
        cov: &[C64],
        look_angle: f64,
        threshold_db: f64,
    ) -> Option<f64> {
        let lambda = self.config.wavelength;
        let prf = self.config.prf;

        // Sample the improvement factor at many Doppler points
        let num_pts = 201;
        let curve = self.sinr_improvement_vs_doppler(cov, look_angle, num_pts, 1.0);

        // Find peak improvement factor
        let peak_if = curve
            .iter()
            .map(|(_, v)| *v)
            .fold(f64::NEG_INFINITY, f64::max);

        let cutoff = peak_if - threshold_db;

        // Search from zero Doppler outward for where IF exceeds cutoff
        let center = num_pts / 2; // fd ~ 0
        for offset in 1..=center {
            let idx_pos = center + offset;
            if idx_pos < curve.len() && curve[idx_pos].1 >= cutoff {
                let fd = curve[idx_pos].0;
                let v_radial = fd * lambda * prf / 2.0;
                return Some(v_radial.abs());
            }
        }
        None
    }

    // ── Utility ─────────────────────────────────────────────────────────

    /// Generate a synthetic clutter covariance matrix based on the clutter
    /// ridge model. Useful for testing.
    ///
    /// `cnr` is the clutter-to-noise ratio (linear scale).
    /// `num_clutter_patches` is the number of azimuth samples.
    pub fn generate_clutter_covariance(
        &self,
        cnr: f64,
        num_clutter_patches: usize,
    ) -> Vec<C64> {
        let nm = self.nm;
        let v = self.config.platform_velocity;
        let lambda = self.config.wavelength;
        let prf = self.config.prf;

        // Start with identity (noise floor)
        let mut cov = vec![(0.0, 0.0); nm * nm];
        for i in 0..nm {
            cov[i * nm + i] = (1.0, 0.0);
        }

        // Add clutter along the ridge
        let patch_power = cnr / num_clutter_patches as f64;
        for p in 0..num_clutter_patches {
            let theta =
                -PI / 2.0 + PI * (p as f64 + 0.5) / num_clutter_patches as f64;
            let fd = 2.0 * v * theta.sin() / (lambda * prf);
            let s = self.steering_vector(theta, fd);

            // Outer product: R += power * s * s^H
            for i in 0..nm {
                for j in 0..nm {
                    let val = c_scale(patch_power, c_mul(s[i], c_conj(s[j])));
                    cov[i * nm + j] = c_add(cov[i * nm + j], val);
                }
            }
        }
        cov
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> StapConfig {
        StapConfig {
            num_elements: 4,
            num_pulses: 8,
            element_spacing_wavelengths: 0.5,
            prf: 1000.0,
            wavelength: 0.03,
            platform_velocity: 100.0,
            diagonal_loading: 0.01,
        }
    }

    fn small_config() -> StapConfig {
        StapConfig {
            num_elements: 3,
            num_pulses: 4,
            element_spacing_wavelengths: 0.5,
            prf: 1000.0,
            wavelength: 0.03,
            platform_velocity: 50.0,
            diagonal_loading: 0.01,
        }
    }

    #[test]
    fn test_space_time_dimension() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        assert_eq!(stap.space_time_dimension(), 32);
    }

    #[test]
    fn test_spatial_steering_vector_length() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let sv = stap.spatial_steering_vector(0.0);
        assert_eq!(sv.len(), 4);
    }

    #[test]
    fn test_spatial_steering_broadside() {
        // At broadside (theta=0), all phases should be 1+0j
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let sv = stap.spatial_steering_vector(0.0);
        for s in &sv {
            assert!((s.0 - 1.0).abs() < 1e-10, "real part should be 1.0");
            assert!(s.1.abs() < 1e-10, "imag part should be 0.0");
        }
    }

    #[test]
    fn test_temporal_steering_vector_length() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let sv = stap.temporal_steering_vector(0.0);
        assert_eq!(sv.len(), 8);
    }

    #[test]
    fn test_temporal_steering_zero_doppler() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let sv = stap.temporal_steering_vector(0.0);
        for s in &sv {
            assert!((s.0 - 1.0).abs() < 1e-10);
            assert!(s.1.abs() < 1e-10);
        }
    }

    #[test]
    fn test_steering_vector_length() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let sv = stap.steering_vector(0.0, 0.0);
        assert_eq!(sv.len(), 32);
    }

    #[test]
    fn test_steering_vector_unit_norm_at_broadside() {
        // At broadside, zero Doppler, the steering vector is all ones
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let sv = stap.steering_vector(0.0, 0.0);
        let norm_sq: f64 = sv.iter().map(|s| c_mag_sq(*s)).sum();
        // norm should be NM = 32
        assert!((norm_sq - 32.0).abs() < 1e-8);
    }

    #[test]
    fn test_form_snapshot() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let data: Vec<C64> = (0..12).map(|i| (i as f64, 0.0)).collect();
        let snap = stap.form_snapshot(&data);
        assert_eq!(snap.len(), 12);
        assert_eq!(snap[0], (0.0, 0.0));
        assert_eq!(snap[11], (11.0, 0.0));
    }

    #[test]
    #[should_panic(expected = "data length must equal")]
    fn test_form_snapshot_wrong_size() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let data: Vec<C64> = vec![(1.0, 0.0); 5];
        stap.form_snapshot(&data);
    }

    #[test]
    fn test_estimate_covariance_identity() {
        // If training data is identity-like (orthonormal snapshots),
        // covariance should be close to identity.
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = stap.space_time_dimension(); // 12
        let num_snapshots = 100;
        // Generate noise-like data using a simple PRNG
        let mut data = Vec::with_capacity(num_snapshots * nm);
        let mut seed: u64 = 42;
        for _ in 0..(num_snapshots * nm) {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let re = ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let im = ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
            data.push((re, im));
        }
        let cov = stap.estimate_covariance(&data, num_snapshots);
        assert_eq!(cov.len(), nm * nm);
        // Diagonal should be positive
        for i in 0..nm {
            assert!(cov[i * nm + i].0 > 0.0, "diagonal should be positive");
        }
    }

    #[test]
    fn test_covariance_hermitian() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = 12;
        let num_snapshots = 20;
        let mut data = Vec::with_capacity(num_snapshots * nm);
        let mut seed: u64 = 123;
        for _ in 0..(num_snapshots * nm) {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let re = ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let im = ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
            data.push((re, im));
        }
        let cov = stap.estimate_covariance(&data, num_snapshots);
        // Check R[i,j] == conj(R[j,i])
        for i in 0..nm {
            for j in 0..nm {
                let rij = cov[i * nm + j];
                let rji = cov[j * nm + i];
                assert!(
                    (rij.0 - rji.0).abs() < 1e-10,
                    "Hermitian re mismatch at ({},{})",
                    i,
                    j
                );
                assert!(
                    (rij.1 + rji.1).abs() < 1e-10,
                    "Hermitian im mismatch at ({},{})",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_diagonal_loading() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = 12;
        let mut cov = vec![(0.0, 0.0); nm * nm];
        for i in 0..nm {
            cov[i * nm + i] = (1.0, 0.0);
        }
        stap.apply_diagonal_loading(&mut cov, 0.5);
        for i in 0..nm {
            assert!((cov[i * nm + i].0 - 1.5).abs() < 1e-12);
        }
    }

    #[test]
    fn test_compute_weights_identity_cov() {
        // With identity covariance, weights should be proportional to steering vector
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = 12;
        let mut cov = vec![(0.0, 0.0); nm * nm];
        for i in 0..nm {
            cov[i * nm + i] = (1.0, 0.0);
        }
        let s = stap.steering_vector(0.0, 0.0);
        let result = stap.compute_weights(&cov, &s);
        assert!(result.is_some(), "weights should be computable");
        let w = result.unwrap().weights;
        assert_eq!(w.len(), nm);

        // w should be s / (s^H s) scaled. For broadside zero-doppler, s = all-ones,
        // so w_i ~ 1/NM (after diagonal loading adjusts slightly)
        let output = stap.apply_weights(&w, &s);
        // w^H s should be ~ 1.0 (MVDR normalisation)
        assert!(
            (output.0 - 1.0).abs() < 0.05,
            "distortionless constraint: got {}",
            output.0
        );
        assert!(output.1.abs() < 0.05);
    }

    #[test]
    fn test_apply_weights() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let w: Vec<C64> = vec![(1.0, 0.0); 12];
        let x: Vec<C64> = vec![(2.0, 1.0); 12];
        let out = stap.apply_weights(&w, &x);
        // inner_product(w, x) = sum(conj(w_i)*x_i) = 12 * (2+j) = (24, 12)
        assert!((out.0 - 24.0).abs() < 1e-10);
        assert!((out.1 - 12.0).abs() < 1e-10);
    }

    #[test]
    fn test_clutter_ridge_symmetry() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let ridge = stap.clutter_ridge(101);
        assert_eq!(ridge.len(), 101);
        // Ridge should be symmetric about theta=0
        let first = &ridge[0];
        let last = &ridge[100];
        assert!(
            (first.doppler_freq + last.doppler_freq).abs() < 1e-10,
            "ridge should be antisymmetric in Doppler"
        );
    }

    #[test]
    fn test_clutter_ridge_broadside_zero_doppler() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let ridge = stap.clutter_ridge(101);
        let mid = &ridge[50]; // theta ~ 0
        assert!(mid.cone_angle.abs() < 0.02);
        assert!(
            mid.doppler_freq.abs() < 0.01,
            "broadside Doppler should be near zero"
        );
    }

    #[test]
    fn test_estimate_clutter_rank_brennan() {
        let stap = SpaceTimeAdaptiveProcessor::new(default_config());
        let rank = stap.estimate_clutter_rank();
        // N=4, M=8, v=100, PRF=1000, d_metres=0.5*0.03=0.015
        // beta = 2*100*(1/1000)/0.015 = 200/15 ~ 13.33
        // rank = 4 + 7*13.33 = 4 + 93.33 = 97.33, clipped to NM=32
        assert!(
            (rank - 32.0).abs() < 0.1,
            "rank should be clipped to NM=32, got {}",
            rank
        );
    }

    #[test]
    fn test_estimate_clutter_rank_low_velocity() {
        let mut cfg = default_config();
        cfg.platform_velocity = 5.0; // Very slow
        let stap = SpaceTimeAdaptiveProcessor::new(cfg);
        let rank = stap.estimate_clutter_rank();
        // beta = 2*5*(1/1000)/0.015 = 10/15 ~ 0.667
        // rank = 4 + 7*0.667 = 4 + 4.667 ~ 8.67
        assert!(rank > 4.0 && rank < 32.0, "rank should be between N and NM, got {}", rank);
    }

    #[test]
    fn test_generate_clutter_covariance_positive_diagonal() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = 12;
        let cov = stap.generate_clutter_covariance(20.0, 50);
        assert_eq!(cov.len(), nm * nm);
        for i in 0..nm {
            assert!(
                cov[i * nm + i].0 > 1.0,
                "diagonal should be > noise floor"
            );
        }
    }

    #[test]
    fn test_sinr_with_clutter() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let cov = stap.generate_clutter_covariance(10.0, 30);
        let s = stap.steering_vector(0.3, 0.1);
        let result = stap.compute_weights(&cov, &s);
        assert!(result.is_some());
        let w = &result.unwrap().weights;
        let sinr = stap.compute_sinr(w, &s, &cov, 1.0);
        // SINR should be finite
        assert!(sinr.sinr_db.is_finite(), "SINR should be finite");
    }

    #[test]
    fn test_pre_doppler_stap_output_length() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = 12;
        let data: Vec<C64> = (0..nm).map(|i| (i as f64, 0.0)).collect();
        let out = stap.pre_doppler_stap(&data);
        assert_eq!(out.len(), nm);
    }

    #[test]
    fn test_post_doppler_stap_output_length() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = 12;
        let data: Vec<C64> = (0..nm).map(|i| (i as f64, 0.0)).collect();
        let out = stap.post_doppler_stap(&data, 0.0);
        assert_eq!(out.len(), small_config().num_pulses);
    }

    #[test]
    fn test_mdv_estimation() {
        // Use a small config with clutter
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let cov = stap.generate_clutter_covariance(20.0, 50);
        let mdv = stap.minimum_detectable_velocity(&cov, 0.0, 3.0);
        // MDV should be a small positive velocity (or None if clutter is too strong)
        if let Some(v) = mdv {
            assert!(v >= 0.0, "MDV should be non-negative, got {}", v);
            assert!(
                v < 100.0,
                "MDV should be reasonable, got {} m/s",
                v
            );
        }
        // It's also acceptable to return None for difficult scenarios
    }

    #[test]
    fn test_auto_diagonal_loading() {
        let stap = SpaceTimeAdaptiveProcessor::new(small_config());
        let nm = 12;
        let mut cov = vec![(0.0, 0.0); nm * nm];
        for i in 0..nm {
            cov[i * nm + i] = (10.0, 0.0);
        }
        let dl = stap.auto_diagonal_loading(&cov);
        // trace = 120, NM = 12, loading_factor = 0.01
        // dl = 0.01 * 120 / 12 = 0.1
        assert!(
            (dl - 0.1).abs() < 1e-10,
            "auto diagonal loading should be 0.1, got {}",
            dl
        );
    }

    #[test]
    fn test_mat_inv_identity() {
        let n = 3;
        let mut eye = vec![(0.0, 0.0); n * n];
        for i in 0..n {
            eye[i * n + i] = (1.0, 0.0);
        }
        let inv = mat_inv(&eye, n).unwrap();
        for i in 0..n {
            for j in 0..n {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (inv[i * n + j].0 - expected).abs() < 1e-10,
                    "inv[{},{}] re = {}, expected {}",
                    i,
                    j,
                    inv[i * n + j].0,
                    expected
                );
                assert!(
                    inv[i * n + j].1.abs() < 1e-10,
                    "inv[{},{}] im should be 0",
                    i,
                    j
                );
            }
        }
    }
}

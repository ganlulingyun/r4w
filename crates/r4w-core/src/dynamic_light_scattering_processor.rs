//! Dynamic Light Scattering (DLS) / Photon Correlation Spectroscopy signal processor.
//!
//! This module implements the complete signal processing pipeline for Dynamic Light
//! Scattering experiments, from raw photon count time series to particle size
//! distributions. DLS measures the temporal fluctuations of scattered laser light
//! caused by Brownian motion of particles in suspension, enabling non-invasive
//! determination of hydrodynamic diameter.
//!
//! # Background
//!
//! When a coherent laser beam illuminates a colloidal suspension, each particle
//! scatters light. Because the particles undergo Brownian motion, the scattered
//! intensity fluctuates. The timescale of these fluctuations is directly related
//! to the diffusion coefficient and hence the particle size via the
//! Stokes-Einstein equation.
//!
//! The key measurable is the intensity autocorrelation function g2(tau), which
//! is related to the field autocorrelation g1(tau) through the Siegert relation:
//!
//! ```text
//!   g2(tau) = 1 + beta * |g1(tau)|^2
//! ```
//!
//! where beta is the coherence factor (instrument-dependent, typically 0.5-0.95).
//!
//! For monodisperse spheres: g1(tau) = exp(-Gamma * tau), where Gamma = D * Q^2
//! is the decay rate, D is the translational diffusion coefficient, and Q is the
//! scattering vector magnitude.
//!
//! # Physics
//!
//! - Scattering vector: Q = (4 * pi * n / lambda) * sin(theta / 2)
//! - Diffusion coefficient: D = Gamma / Q^2
//! - Stokes-Einstein: D = k_B * T / (3 * pi * eta * d_H)
//! - Hence: d_H = k_B * T / (3 * pi * eta * D)
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`DlsConfig`] | Laser wavelength, scattering angle, temperature, viscosity |
//! | [`AutocorrelationCalculator`] | Compute g2(tau) from photon count time series |
//! | [`SiegertRelation`] | Convert intensity to field correlation g2 -> g1 |
//! | [`CumulantAnalyzer`] | Cumulant expansion for Z-average size and PDI |
//! | [`StokesEinsteinEquation`] | Hydrodynamic diameter from diffusion coefficient |
//! | [`ContiRegularization`] | CONTIN-like regularized inversion for size distribution |
//! | [`NumberDistribution`] | Intensity-weighted to number-weighted conversion |
//! | [`MultiAngleAnalyzer`] | Angular dependence of Gamma/Q^2 for shape analysis |
//! | [`PolydispersityEstimator`] | PDI assessment and monodispersity quality |
//! | [`BaselineChecker`] | Verify g2(tau) baseline convergence |
//! | [`TemperatureCorrector`] | Viscosity-temperature correction for water |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::dynamic_light_scattering_processor::{
//!     DlsConfig, AutocorrelationCalculator, SiegertRelation,
//!     CumulantAnalyzer, StokesEinsteinEquation,
//! };
//!
//! // Configure DLS experiment: 633 nm HeNe laser, 173 deg backscatter, 25 C water
//! let config = DlsConfig::new(633e-9, 173.0_f64.to_radians(), 298.15, 0.89e-3, 1.33);
//!
//! // Compute scattering vector
//! let q = config.scattering_vector();
//! assert!(q > 0.0);
//!
//! // From a measured decay rate, compute size
//! let gamma = 5000.0; // 1/s
//! let d = gamma / (q * q);
//! let dh = StokesEinsteinEquation::hydrodynamic_diameter(config.temperature, config.viscosity, d);
//! assert!(dh > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Boltzmann constant in J/K.
const K_B: f64 = 1.380649e-23;

// ---------------------------------------------------------------------------
// DlsConfig
// ---------------------------------------------------------------------------

/// Configuration for a Dynamic Light Scattering experiment.
///
/// Encapsulates all physical parameters needed to convert measured
/// autocorrelation data into particle size information.
#[derive(Debug, Clone, PartialEq)]
pub struct DlsConfig {
    /// Laser wavelength in vacuum (m). Typical: 633e-9 (HeNe), 532e-9 (Nd:YAG).
    pub wavelength: f64,
    /// Scattering angle in radians. Common: 173 deg (backscatter), 90 deg.
    pub scattering_angle: f64,
    /// Absolute temperature in Kelvin.
    pub temperature: f64,
    /// Solvent dynamic viscosity in Pa*s. Water at 25C: 0.89e-3.
    pub viscosity: f64,
    /// Solvent refractive index. Water: 1.33.
    pub refractive_index: f64,
}

impl DlsConfig {
    /// Create a new DLS configuration.
    ///
    /// # Arguments
    /// - `wavelength` - Laser wavelength in vacuum (m)
    /// - `scattering_angle` - Scattering angle (radians)
    /// - `temperature` - Absolute temperature (K)
    /// - `viscosity` - Solvent dynamic viscosity (Pa*s)
    /// - `refractive_index` - Solvent refractive index
    pub fn new(
        wavelength: f64,
        scattering_angle: f64,
        temperature: f64,
        viscosity: f64,
        refractive_index: f64,
    ) -> Self {
        Self {
            wavelength,
            scattering_angle,
            temperature,
            viscosity,
            refractive_index,
        }
    }

    /// Standard DLS configuration: 633 nm HeNe, 173 deg backscatter, 25 C water.
    pub fn standard_backscatter() -> Self {
        Self::new(633e-9, 173.0_f64.to_radians(), 298.15, 0.89e-3, 1.33)
    }

    /// Standard DLS configuration: 633 nm HeNe, 90 deg, 25 C water.
    pub fn standard_90deg() -> Self {
        Self::new(633e-9, 90.0_f64.to_radians(), 298.15, 0.89e-3, 1.33)
    }

    /// Compute the scattering vector magnitude Q = (4*pi*n/lambda)*sin(theta/2).
    ///
    /// Units: 1/m (inverse metres).
    pub fn scattering_vector(&self) -> f64 {
        (4.0 * PI * self.refractive_index / self.wavelength)
            * (self.scattering_angle / 2.0).sin()
    }
}

// ---------------------------------------------------------------------------
// AutocorrelationCalculator
// ---------------------------------------------------------------------------

/// Computes the intensity autocorrelation function g2(tau) from a photon count
/// time series.
///
/// Uses the multi-tau algorithm concept: computes normalized autocorrelation at
/// specified lag times. The output is the normalized g2(tau) = <I(t)*I(t+tau)> / <I>^2.
#[derive(Debug, Clone)]
pub struct AutocorrelationCalculator {
    /// Lag channel indices for which to compute g2.
    lag_channels: Vec<usize>,
}

impl AutocorrelationCalculator {
    /// Create calculator with logarithmically-spaced lag channels.
    ///
    /// Generates `n_channels` lag indices from 1 up to `max_lag`, spaced
    /// approximately logarithmically (as is standard in DLS correlators).
    pub fn new_log_spaced(n_channels: usize, max_lag: usize) -> Self {
        let mut channels = Vec::with_capacity(n_channels);
        if n_channels == 0 || max_lag == 0 {
            return Self { lag_channels: channels };
        }
        if n_channels == 1 {
            channels.push(1);
            return Self { lag_channels: channels };
        }
        let log_min = 0.0_f64; // log2(1)
        let log_max = (max_lag as f64).log2();
        for i in 0..n_channels {
            let frac = i as f64 / (n_channels - 1) as f64;
            let lag = 2.0_f64.powf(log_min + frac * (log_max - log_min)).round() as usize;
            let lag = lag.max(1).min(max_lag);
            if channels.last().map_or(true, |&prev| prev != lag) {
                channels.push(lag);
            }
        }
        Self { lag_channels: channels }
    }

    /// Create calculator with linearly-spaced lag channels.
    pub fn new_linear(n_channels: usize, max_lag: usize) -> Self {
        let mut channels = Vec::with_capacity(n_channels);
        if n_channels == 0 || max_lag == 0 {
            return Self { lag_channels: channels };
        }
        for i in 0..n_channels {
            let lag = if n_channels == 1 {
                1
            } else {
                1 + i * (max_lag - 1) / (n_channels - 1)
            };
            if channels.last().map_or(true, |&prev| prev != lag) {
                channels.push(lag);
            }
        }
        Self { lag_channels: channels }
    }

    /// Create calculator with explicitly specified lag channels.
    pub fn new_custom(lag_channels: Vec<usize>) -> Self {
        Self { lag_channels }
    }

    /// Return the lag channel indices.
    pub fn lag_channels(&self) -> &[usize] {
        &self.lag_channels
    }

    /// Compute the normalized intensity autocorrelation g2(tau).
    ///
    /// Returns a vector of (lag_index, g2_value) pairs.
    ///
    /// g2(tau) = <I(t) * I(t+tau)> / <I>^2
    pub fn compute_g2(&self, intensities: &[f64]) -> Vec<(usize, f64)> {
        let n = intensities.len();
        if n < 2 {
            return Vec::new();
        }

        // Compute mean intensity
        let mean_i: f64 = intensities.iter().sum::<f64>() / n as f64;
        if mean_i.abs() < 1e-30 {
            return Vec::new();
        }
        let mean_i_sq = mean_i * mean_i;

        let mut result = Vec::with_capacity(self.lag_channels.len());

        for &lag in &self.lag_channels {
            if lag >= n {
                continue;
            }
            let count = n - lag;
            let mut sum = 0.0;
            for t in 0..count {
                sum += intensities[t] * intensities[t + lag];
            }
            let g2 = (sum / count as f64) / mean_i_sq;
            result.push((lag, g2));
        }

        result
    }

    /// Compute the unnormalized autocorrelation (raw product sum).
    pub fn compute_raw(&self, intensities: &[f64]) -> Vec<(usize, f64)> {
        let n = intensities.len();
        if n < 2 {
            return Vec::new();
        }

        let mut result = Vec::with_capacity(self.lag_channels.len());
        for &lag in &self.lag_channels {
            if lag >= n {
                continue;
            }
            let count = n - lag;
            let mut sum = 0.0;
            for t in 0..count {
                sum += intensities[t] * intensities[t + lag];
            }
            result.push((lag, sum / count as f64));
        }
        result
    }
}

// ---------------------------------------------------------------------------
// SiegertRelation
// ---------------------------------------------------------------------------

/// Converts between intensity autocorrelation g2(tau) and field autocorrelation
/// g1(tau) via the Siegert relation.
///
/// g2(tau) = 1 + beta * |g1(tau)|^2
///
/// where beta is the coherence factor (spatial coherence, detector area, etc.).
/// Typical beta: 0.5 - 0.95 for single-mode fiber optics.
#[derive(Debug, Clone)]
pub struct SiegertRelation {
    /// Coherence factor (0 < beta <= 1).
    pub beta: f64,
}

impl SiegertRelation {
    /// Create a new Siegert relation converter.
    ///
    /// # Arguments
    /// - `beta` - Coherence factor (spatial coherence parameter), typically 0.5-0.95
    pub fn new(beta: f64) -> Self {
        Self { beta: beta.clamp(1e-10, 1.0) }
    }

    /// Convert g2(tau) values to |g1(tau)| values.
    ///
    /// |g1(tau)| = sqrt((g2(tau) - 1) / beta)
    ///
    /// Returns None if g2 < 1 (unphysical) or beta <= 0.
    pub fn g2_to_g1(&self, g2: f64) -> Option<f64> {
        let arg = (g2 - 1.0) / self.beta;
        if arg < 0.0 {
            None
        } else {
            Some(arg.sqrt())
        }
    }

    /// Convert |g1(tau)| to g2(tau).
    pub fn g1_to_g2(&self, g1_abs: f64) -> f64 {
        1.0 + self.beta * g1_abs * g1_abs
    }

    /// Convert a vector of (lag, g2) to (lag, |g1|), filtering unphysical values.
    pub fn convert_g2_to_g1(&self, g2_data: &[(usize, f64)]) -> Vec<(usize, f64)> {
        g2_data
            .iter()
            .filter_map(|&(lag, g2)| {
                self.g2_to_g1(g2).map(|g1| (lag, g1))
            })
            .collect()
    }

    /// Estimate beta from the intercept of g2(tau) at tau -> 0.
    ///
    /// beta = g2(0) - 1 (since |g1(0)| = 1 for normalized correlation).
    pub fn estimate_beta(g2_at_zero: f64) -> f64 {
        (g2_at_zero - 1.0).max(0.0).min(1.0)
    }
}

// ---------------------------------------------------------------------------
// CumulantAnalyzer
// ---------------------------------------------------------------------------

/// Cumulant analysis of the field autocorrelation function.
///
/// Fits the logarithm of |g1(tau)| to a polynomial:
///
/// ```text
///   ln|g1(tau)| = -Gamma*tau + (mu2/2)*tau^2 - (mu3/6)*tau^3 + ...
/// ```
///
/// This yields:
/// - Gamma: mean decay rate -> Z-average diffusion coefficient -> Z-average size
/// - mu2: second cumulant -> polydispersity index PDI = mu2/Gamma^2
/// - mu3: third cumulant -> skewness of distribution
#[derive(Debug, Clone)]
pub struct CumulantAnalyzer {
    /// Maximum polynomial order for the fit (2 = quadratic, 3 = cubic).
    pub max_order: usize,
}

/// Result of a cumulant analysis.
#[derive(Debug, Clone, PartialEq)]
pub struct CumulantResult {
    /// Mean decay rate Gamma (1/s or 1/sample, depending on tau units).
    pub gamma: f64,
    /// Second cumulant mu2 (variance of decay rate distribution).
    pub mu2: f64,
    /// Third cumulant mu3 (skewness, only valid if max_order >= 3).
    pub mu3: f64,
    /// Polydispersity index PDI = mu2 / Gamma^2.
    pub pdi: f64,
    /// Residual of the fit (sum of squared errors).
    pub residual: f64,
}

impl CumulantAnalyzer {
    /// Create a cumulant analyzer.
    ///
    /// # Arguments
    /// - `max_order` - Maximum polynomial order (2 or 3 recommended)
    pub fn new(max_order: usize) -> Self {
        Self {
            max_order: max_order.max(1).min(4),
        }
    }

    /// Perform cumulant analysis on |g1(tau)| data.
    ///
    /// # Arguments
    /// - `g1_data` - Vector of (tau, |g1(tau)|) pairs. tau should be in seconds
    ///   (or sample intervals); |g1| should be in (0, 1].
    ///
    /// Returns `None` if insufficient data or all g1 values are non-positive.
    pub fn analyze(&self, g1_data: &[(f64, f64)]) -> Option<CumulantResult> {
        // Filter valid points (g1 > 0 for log)
        let valid: Vec<(f64, f64)> = g1_data
            .iter()
            .filter(|&&(_, g1)| g1 > 0.0 && g1 <= 1.0)
            .copied()
            .collect();

        let n = valid.len();
        let order = self.max_order.min(n.saturating_sub(1));
        if n < 2 || order < 1 {
            return None;
        }

        // Build ln|g1| data
        let y: Vec<f64> = valid.iter().map(|&(_, g1)| g1.ln()).collect();
        let tau: Vec<f64> = valid.iter().map(|&(t, _)| t).collect();

        // Polynomial least-squares fit: y = c0 + c1*tau + c2*tau^2 + c3*tau^3 + ...
        // We expect c0 ~ 0, c1 = -Gamma, c2 = mu2/2, c3 = -mu3/6
        let coeffs = polynomial_least_squares(&tau, &y, order);

        // Extract cumulants
        let gamma = -coeffs.get(1).copied().unwrap_or(0.0);
        let mu2 = 2.0 * coeffs.get(2).copied().unwrap_or(0.0);
        let mu3 = -6.0 * coeffs.get(3).copied().unwrap_or(0.0);

        let pdi = if gamma.abs() > 1e-30 {
            mu2 / (gamma * gamma)
        } else {
            0.0
        };

        // Compute residual
        let mut residual = 0.0;
        for i in 0..n {
            let mut y_fit = 0.0;
            for (k, &c) in coeffs.iter().enumerate() {
                y_fit += c * tau[i].powi(k as i32);
            }
            let err = y[i] - y_fit;
            residual += err * err;
        }

        Some(CumulantResult {
            gamma,
            mu2,
            mu3,
            pdi: pdi.abs(),
            residual,
        })
    }
}

// ---------------------------------------------------------------------------
// Polynomial least squares (internal)
// ---------------------------------------------------------------------------

/// Fit y = c0 + c1*x + c2*x^2 + ... + cn*x^n using normal equations.
///
/// Returns coefficient vector [c0, c1, ..., cn].
fn polynomial_least_squares(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len();
    let m = order + 1; // number of coefficients

    // Build Vandermonde matrix and solve normal equations A^T A c = A^T y
    // For small order (2-4) this is perfectly fine numerically.

    // Compute A^T A (m x m)
    let mut ata = vec![0.0; m * m];
    for i in 0..m {
        for j in 0..m {
            let mut sum = 0.0;
            for k in 0..n {
                sum += x[k].powi((i + j) as i32);
            }
            ata[i * m + j] = sum;
        }
    }

    // Compute A^T y (m)
    let mut aty = vec![0.0; m];
    for i in 0..m {
        let mut sum = 0.0;
        for k in 0..n {
            sum += x[k].powi(i as i32) * y[k];
        }
        aty[i] = sum;
    }

    // Solve via Gaussian elimination with partial pivoting
    solve_linear_system(&mut ata, &mut aty, m)
}

/// Solve Ax = b in-place using Gaussian elimination with partial pivoting.
/// Returns x. The input arrays are modified.
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = a[col * n + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for j in 0..n {
                a.swap(col * n + j, max_row * n + j);
            }
            b.swap(col, max_row);
        }

        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        // Eliminate below
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * x[j];
        }
        let diag = a[i * n + i];
        if diag.abs() > 1e-30 {
            x[i] = sum / diag;
        }
    }
    x
}

// ---------------------------------------------------------------------------
// StokesEinsteinEquation
// ---------------------------------------------------------------------------

/// Implements the Stokes-Einstein equation for converting between diffusion
/// coefficients and hydrodynamic diameters.
///
/// D = k_B * T / (3 * pi * eta * d_H)
/// d_H = k_B * T / (3 * pi * eta * D)
pub struct StokesEinsteinEquation;

impl StokesEinsteinEquation {
    /// Compute hydrodynamic diameter from diffusion coefficient.
    ///
    /// # Arguments
    /// - `temperature` - Absolute temperature (K)
    /// - `viscosity` - Solvent dynamic viscosity (Pa*s)
    /// - `diffusion_coeff` - Translational diffusion coefficient (m^2/s)
    ///
    /// Returns diameter in metres.
    pub fn hydrodynamic_diameter(temperature: f64, viscosity: f64, diffusion_coeff: f64) -> f64 {
        if diffusion_coeff.abs() < 1e-30 {
            return 0.0;
        }
        K_B * temperature / (3.0 * PI * viscosity * diffusion_coeff)
    }

    /// Compute diffusion coefficient from hydrodynamic diameter.
    ///
    /// # Arguments
    /// - `temperature` - Absolute temperature (K)
    /// - `viscosity` - Solvent dynamic viscosity (Pa*s)
    /// - `diameter` - Hydrodynamic diameter (m)
    ///
    /// Returns diffusion coefficient in m^2/s.
    pub fn diffusion_coefficient(temperature: f64, viscosity: f64, diameter: f64) -> f64 {
        if diameter.abs() < 1e-30 {
            return 0.0;
        }
        K_B * temperature / (3.0 * PI * viscosity * diameter)
    }

    /// Compute decay rate Gamma from diameter and DLS configuration.
    ///
    /// Gamma = D * Q^2, where D = k_B*T/(3*pi*eta*d_H)
    pub fn decay_rate_from_diameter(config: &DlsConfig, diameter: f64) -> f64 {
        let d = Self::diffusion_coefficient(config.temperature, config.viscosity, diameter);
        let q = config.scattering_vector();
        d * q * q
    }

    /// Compute diameter from measured decay rate and DLS configuration.
    pub fn diameter_from_decay_rate(config: &DlsConfig, gamma: f64) -> f64 {
        let q = config.scattering_vector();
        if q.abs() < 1e-30 || gamma.abs() < 1e-30 {
            return 0.0;
        }
        let d = gamma / (q * q);
        Self::hydrodynamic_diameter(config.temperature, config.viscosity, d)
    }
}

// ---------------------------------------------------------------------------
// ContiRegularization
// ---------------------------------------------------------------------------

/// CONTIN-like regularized inversion for recovering the size distribution from
/// the measured field autocorrelation function.
///
/// Solves: min_f ||g1_measured - A * f||^2 + alpha * ||L * f||^2
///
/// where A is the kernel matrix (exponential decays for each size bin),
/// f is the (non-negative) distribution, and L is a regularization operator
/// (identity or second derivative for smoothness).
///
/// The regularization parameter alpha controls the trade-off between fitting
/// fidelity and smoothness. A larger alpha produces smoother distributions
/// but may lose fine detail.
#[derive(Debug, Clone)]
pub struct ContiRegularization {
    /// Regularization parameter alpha.
    pub alpha: f64,
    /// Number of size bins for the distribution.
    pub n_bins: usize,
    /// Minimum diameter (m).
    pub d_min: f64,
    /// Maximum diameter (m).
    pub d_max: f64,
    /// Maximum iterations for non-negative least squares.
    pub max_iterations: usize,
}

/// Result of CONTIN-like inversion.
#[derive(Debug, Clone)]
pub struct SizeDistribution {
    /// Diameter bin centres (m).
    pub diameters: Vec<f64>,
    /// Intensity-weighted distribution amplitudes (arbitrary units).
    pub amplitudes: Vec<f64>,
    /// Residual norm of the fit.
    pub residual: f64,
}

impl ContiRegularization {
    /// Create a new CONTIN regularizer.
    pub fn new(alpha: f64, n_bins: usize, d_min: f64, d_max: f64) -> Self {
        Self {
            alpha: alpha.max(0.0),
            n_bins: n_bins.max(2),
            d_min,
            d_max,
            max_iterations: 200,
        }
    }

    /// Generate logarithmically-spaced diameter bins.
    pub fn diameter_bins(&self) -> Vec<f64> {
        let log_min = self.d_min.ln();
        let log_max = self.d_max.ln();
        (0..self.n_bins)
            .map(|i| {
                let frac = i as f64 / (self.n_bins - 1).max(1) as f64;
                (log_min + frac * (log_max - log_min)).exp()
            })
            .collect()
    }

    /// Perform regularized inversion.
    ///
    /// # Arguments
    /// - `config` - DLS configuration
    /// - `g1_data` - Measured |g1(tau)| as (tau_seconds, g1_value) pairs
    ///
    /// Returns the size distribution.
    pub fn invert(&self, config: &DlsConfig, g1_data: &[(f64, f64)]) -> SizeDistribution {
        let diameters = self.diameter_bins();
        let n_tau = g1_data.len();
        let n_d = diameters.len();

        if n_tau == 0 || n_d == 0 {
            return SizeDistribution {
                diameters,
                amplitudes: vec![0.0; n_d],
                residual: 0.0,
            };
        }

        // Build kernel matrix A[i][j] = exp(-Gamma_j * tau_i)
        // where Gamma_j = D_j * Q^2 and D_j = kT/(3*pi*eta*d_j)
        let q = config.scattering_vector();
        let q2 = q * q;

        let gammas: Vec<f64> = diameters
            .iter()
            .map(|&d| {
                let diff = StokesEinsteinEquation::diffusion_coefficient(
                    config.temperature,
                    config.viscosity,
                    d,
                );
                diff * q2
            })
            .collect();

        // A is n_tau x n_d
        let mut a_mat = vec![0.0; n_tau * n_d];
        for i in 0..n_tau {
            let tau = g1_data[i].0;
            for j in 0..n_d {
                a_mat[i * n_d + j] = (-gammas[j] * tau).exp();
            }
        }

        // Measured g1 vector
        let g1_vec: Vec<f64> = g1_data.iter().map(|&(_, g1)| g1).collect();

        // Solve (A^T A + alpha * I) * f = A^T * g1 with non-negativity constraint
        // Build normal equations
        let mut ata = vec![0.0; n_d * n_d];
        for i in 0..n_d {
            for j in 0..n_d {
                let mut sum = 0.0;
                for k in 0..n_tau {
                    sum += a_mat[k * n_d + i] * a_mat[k * n_d + j];
                }
                ata[i * n_d + j] = sum;
            }
            // Add regularization
            ata[i * n_d + i] += self.alpha;
        }

        let mut atg = vec![0.0; n_d];
        for i in 0..n_d {
            let mut sum = 0.0;
            for k in 0..n_tau {
                sum += a_mat[k * n_d + i] * g1_vec[k];
            }
            atg[i] = sum;
        }

        // Non-negative least squares using active set method (simplified)
        let amplitudes = self.nnls_solve(&ata, &atg, n_d);

        // Compute residual
        let mut residual = 0.0;
        for i in 0..n_tau {
            let mut fitted = 0.0;
            for j in 0..n_d {
                fitted += a_mat[i * n_d + j] * amplitudes[j];
            }
            let err = g1_vec[i] - fitted;
            residual += err * err;
        }

        SizeDistribution {
            diameters,
            amplitudes,
            residual: residual.sqrt(),
        }
    }

    /// Simplified non-negative least squares solver.
    ///
    /// Solves: min ||Hx - g||^2 subject to x >= 0
    /// using iterative projection (Lawson-Hanson style).
    fn nnls_solve(&self, h: &[f64], g: &[f64], n: usize) -> Vec<f64> {
        let mut x = vec![0.0; n];

        // Start with unconstrained solution, then project
        let mut h_copy = h.to_vec();
        let mut g_copy = g.to_vec();
        let unconstrained = solve_linear_system(&mut h_copy, &mut g_copy, n);

        // If all non-negative, we're done
        if unconstrained.iter().all(|&v| v >= 0.0) {
            return unconstrained;
        }

        // Iterative: set negative components to zero and re-solve
        let mut active: Vec<bool> = vec![true; n]; // true = free, false = fixed at 0

        for _ in 0..self.max_iterations {
            // Solve reduced system
            let mut x_trial = vec![0.0; n];
            let free_indices: Vec<usize> = (0..n).filter(|&i| active[i]).collect();
            let nf = free_indices.len();

            if nf == 0 {
                break;
            }

            // Build reduced normal equations
            let mut h_red = vec![0.0; nf * nf];
            let mut g_red = vec![0.0; nf];
            for (ri, &i) in free_indices.iter().enumerate() {
                g_red[ri] = g[i];
                for (ci, &j) in free_indices.iter().enumerate() {
                    h_red[ri * nf + ci] = h[i * n + j];
                }
            }

            let sol = solve_linear_system(&mut h_red, &mut g_red, nf);
            for (ri, &i) in free_indices.iter().enumerate() {
                x_trial[i] = sol[ri];
            }

            // Check for negative values
            let mut all_nonneg = true;
            for &i in &free_indices {
                if x_trial[i] < 0.0 {
                    active[i] = false;
                    all_nonneg = false;
                }
            }

            if all_nonneg {
                x = x_trial;
                break;
            }

            // Project negative to zero
            for i in 0..n {
                x[i] = if active[i] { x_trial[i].max(0.0) } else { 0.0 };
            }
        }

        x
    }
}

impl SizeDistribution {
    /// Find the peak diameter (mode of the distribution).
    pub fn peak_diameter(&self) -> f64 {
        self.diameters
            .iter()
            .zip(self.amplitudes.iter())
            .max_by(|(_, a1), (_, a2)| a1.partial_cmp(a2).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(&d, _)| d)
            .unwrap_or(0.0)
    }

    /// Compute the intensity-weighted mean diameter.
    pub fn mean_diameter(&self) -> f64 {
        let total: f64 = self.amplitudes.iter().sum();
        if total < 1e-30 {
            return 0.0;
        }
        let weighted: f64 = self
            .diameters
            .iter()
            .zip(self.amplitudes.iter())
            .map(|(&d, &a)| d * a)
            .sum();
        weighted / total
    }

    /// Compute the D10, D50, D90 percentile diameters.
    pub fn percentiles(&self) -> (f64, f64, f64) {
        let total: f64 = self.amplitudes.iter().sum();
        if total < 1e-30 {
            return (0.0, 0.0, 0.0);
        }

        let mut cumulative = 0.0;
        let mut d10 = 0.0;
        let mut d50 = 0.0;
        let mut d90 = 0.0;

        for (&d, &a) in self.diameters.iter().zip(self.amplitudes.iter()) {
            cumulative += a;
            let frac = cumulative / total;
            if d10 == 0.0 && frac >= 0.1 {
                d10 = d;
            }
            if d50 == 0.0 && frac >= 0.5 {
                d50 = d;
            }
            if d90 == 0.0 && frac >= 0.9 {
                d90 = d;
            }
        }

        (d10, d50, d90)
    }
}

// ---------------------------------------------------------------------------
// NumberDistribution
// ---------------------------------------------------------------------------

/// Converts an intensity-weighted size distribution to a number-weighted
/// distribution using Mie scattering correction.
///
/// For particles much smaller than the wavelength (Rayleigh regime),
/// scattered intensity scales as d^6, so:
///
/// ```text
///   f_number(d) = f_intensity(d) / d^6
///   f_volume(d) = f_intensity(d) / d^3
/// ```
///
/// For larger particles, the full Mie correction is more complex, but the
/// d^6 approximation is widely used in DLS instruments.
pub struct NumberDistribution;

impl NumberDistribution {
    /// Convert intensity distribution to number distribution (Rayleigh approximation).
    ///
    /// Divides each amplitude by d^6, then renormalizes.
    pub fn from_intensity_rayleigh(dist: &SizeDistribution) -> SizeDistribution {
        let mut number_amps: Vec<f64> = dist
            .diameters
            .iter()
            .zip(dist.amplitudes.iter())
            .map(|(&d, &a)| {
                if d > 1e-30 {
                    a / d.powi(6)
                } else {
                    0.0
                }
            })
            .collect();

        // Normalize to sum to 1
        let total: f64 = number_amps.iter().sum();
        if total > 1e-30 {
            for a in &mut number_amps {
                *a /= total;
            }
        }

        SizeDistribution {
            diameters: dist.diameters.clone(),
            amplitudes: number_amps,
            residual: dist.residual,
        }
    }

    /// Convert intensity distribution to volume distribution.
    ///
    /// Divides each amplitude by d^3, then renormalizes.
    pub fn from_intensity_volume(dist: &SizeDistribution) -> SizeDistribution {
        let mut vol_amps: Vec<f64> = dist
            .diameters
            .iter()
            .zip(dist.amplitudes.iter())
            .map(|(&d, &a)| {
                if d > 1e-30 {
                    a / d.powi(3)
                } else {
                    0.0
                }
            })
            .collect();

        let total: f64 = vol_amps.iter().sum();
        if total > 1e-30 {
            for a in &mut vol_amps {
                *a /= total;
            }
        }

        SizeDistribution {
            diameters: dist.diameters.clone(),
            amplitudes: vol_amps,
            residual: dist.residual,
        }
    }
}

// ---------------------------------------------------------------------------
// MultiAngleAnalyzer
// ---------------------------------------------------------------------------

/// Analyzes DLS measurements at multiple scattering angles to distinguish
/// between translational and rotational diffusion, or to detect shape
/// anisotropy.
///
/// For a sphere: Gamma / Q^2 = D (constant, independent of angle).
/// For anisotropic particles: Gamma / Q^2 varies with angle, revealing
/// rotational contributions.
#[derive(Debug, Clone)]
pub struct MultiAngleAnalyzer;

/// Result of multi-angle analysis.
#[derive(Debug, Clone)]
pub struct MultiAngleResult {
    /// Angles in radians.
    pub angles: Vec<f64>,
    /// Q values for each angle.
    pub q_values: Vec<f64>,
    /// Q^2 values.
    pub q2_values: Vec<f64>,
    /// Measured decay rates.
    pub gammas: Vec<f64>,
    /// Apparent diffusion coefficient D_app = Gamma / Q^2 at each angle.
    pub d_apparent: Vec<f64>,
    /// Extrapolated D at Q^2 -> 0 (translational diffusion).
    pub d_extrapolated: f64,
    /// Slope of D_app vs Q^2 (related to rotational diffusion or structure).
    pub slope: f64,
    /// Is the particle isotropic? (slope ~ 0 within tolerance)
    pub is_isotropic: bool,
}

impl MultiAngleAnalyzer {
    /// Analyze decay rates measured at multiple angles.
    ///
    /// # Arguments
    /// - `base_config` - Base DLS configuration (wavelength, temperature, etc.)
    /// - `angle_gamma_pairs` - Vector of (angle_rad, gamma) measurements
    /// - `isotropy_tolerance` - Fractional tolerance for isotropy test (e.g. 0.1)
    pub fn analyze(
        base_config: &DlsConfig,
        angle_gamma_pairs: &[(f64, f64)],
        isotropy_tolerance: f64,
    ) -> MultiAngleResult {
        let n = angle_gamma_pairs.len();
        let mut angles = Vec::with_capacity(n);
        let mut q_values = Vec::with_capacity(n);
        let mut q2_values = Vec::with_capacity(n);
        let mut gammas = Vec::with_capacity(n);
        let mut d_apparent = Vec::with_capacity(n);

        for &(angle, gamma) in angle_gamma_pairs {
            let mut cfg = base_config.clone();
            cfg.scattering_angle = angle;
            let q = cfg.scattering_vector();
            let q2 = q * q;
            let d_app = if q2 > 1e-30 { gamma / q2 } else { 0.0 };

            angles.push(angle);
            q_values.push(q);
            q2_values.push(q2);
            gammas.push(gamma);
            d_apparent.push(d_app);
        }

        // Linear regression: D_app = D0 + slope * Q^2
        let (d_extrapolated, slope) = if n >= 2 {
            linear_regression(&q2_values, &d_apparent)
        } else if n == 1 {
            (d_apparent[0], 0.0)
        } else {
            (0.0, 0.0)
        };

        // Isotropy test: check if the relative variation in D_app is small
        let d_mean = if !d_apparent.is_empty() {
            d_apparent.iter().sum::<f64>() / d_apparent.len() as f64
        } else {
            0.0
        };
        let is_isotropic = if d_mean.abs() > 1e-30 && n >= 2 {
            let max_dev = d_apparent
                .iter()
                .map(|&d| ((d - d_mean) / d_mean).abs())
                .fold(0.0_f64, f64::max);
            max_dev < isotropy_tolerance
        } else {
            true
        };

        MultiAngleResult {
            angles,
            q_values,
            q2_values,
            gammas,
            d_apparent,
            d_extrapolated,
            slope,
            is_isotropic,
        }
    }
}

/// Simple linear regression: y = a + b*x. Returns (intercept, slope).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    if n < 2.0 {
        return (y.first().copied().unwrap_or(0.0), 0.0);
    }
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(&xi, &yi)| xi * yi).sum();
    let sxx: f64 = x.iter().map(|&xi| xi * xi).sum();

    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (sy / n, 0.0);
    }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    (intercept, slope)
}

// ---------------------------------------------------------------------------
// PolydispersityEstimator
// ---------------------------------------------------------------------------

/// Estimates and classifies the polydispersity of a sample.
///
/// PDI = mu2 / Gamma^2, where mu2 and Gamma come from cumulant analysis.
///
/// Classification (ISO 22412):
/// - PDI < 0.05: monodisperse
/// - 0.05 <= PDI < 0.1: nearly monodisperse
/// - 0.1 <= PDI < 0.3: moderately polydisperse
/// - 0.3 <= PDI < 0.7: broadly polydisperse
/// - PDI >= 0.7: very polydisperse (cumulant analysis unreliable)
#[derive(Debug, Clone, PartialEq)]
pub enum PolydispersityClass {
    /// PDI < 0.05
    Monodisperse,
    /// 0.05 <= PDI < 0.1
    NearlyMonodisperse,
    /// 0.1 <= PDI < 0.3
    ModeratelyPolydisperse,
    /// 0.3 <= PDI < 0.7
    BroadlyPolydisperse,
    /// PDI >= 0.7
    VeryPolydisperse,
}

/// Polydispersity estimation results.
#[derive(Debug, Clone)]
pub struct PolydispersityEstimate {
    /// Polydispersity index.
    pub pdi: f64,
    /// Classification.
    pub class: PolydispersityClass,
    /// Is the cumulant analysis reliable? (PDI < 0.7)
    pub cumulant_reliable: bool,
    /// Recommended analysis method.
    pub recommended_method: &'static str,
}

pub struct PolydispersityEstimator;

impl PolydispersityEstimator {
    /// Classify a PDI value.
    pub fn classify(pdi: f64) -> PolydispersityEstimate {
        let (class, method) = if pdi < 0.05 {
            (PolydispersityClass::Monodisperse, "cumulant (single exponential)")
        } else if pdi < 0.1 {
            (PolydispersityClass::NearlyMonodisperse, "cumulant (2nd order)")
        } else if pdi < 0.3 {
            (PolydispersityClass::ModeratelyPolydisperse, "cumulant or CONTIN")
        } else if pdi < 0.7 {
            (PolydispersityClass::BroadlyPolydisperse, "CONTIN or NNLS")
        } else {
            (PolydispersityClass::VeryPolydisperse, "CONTIN (cumulant unreliable)")
        };

        PolydispersityEstimate {
            pdi,
            class,
            cumulant_reliable: pdi < 0.7,
            recommended_method: method,
        }
    }

    /// Compute PDI from second cumulant and mean decay rate.
    pub fn compute_pdi(mu2: f64, gamma: f64) -> f64 {
        if gamma.abs() < 1e-30 {
            return 0.0;
        }
        (mu2 / (gamma * gamma)).abs()
    }
}

// ---------------------------------------------------------------------------
// BaselineChecker
// ---------------------------------------------------------------------------

/// Checks the baseline quality of the autocorrelation function.
///
/// A well-converged g2(tau) should asymptote to 1.0 at long lag times.
/// Deviations indicate:
/// - Dust or large aggregates (baseline > 1)
/// - Insufficient measurement time
/// - Mechanical vibration or other artefacts
#[derive(Debug, Clone)]
pub struct BaselineChecker {
    /// Fraction of the correlation function tail to use for baseline estimation.
    /// E.g. 0.1 means use the last 10% of channels.
    pub tail_fraction: f64,
    /// Maximum acceptable deviation from 1.0 in g2 baseline.
    pub tolerance: f64,
}

/// Baseline quality assessment.
#[derive(Debug, Clone)]
pub struct BaselineQuality {
    /// Estimated baseline value of g2.
    pub baseline: f64,
    /// Standard deviation of g2 in the tail region.
    pub noise: f64,
    /// Is the baseline acceptable?
    pub is_acceptable: bool,
    /// Deviation from ideal baseline of 1.0.
    pub deviation: f64,
}

impl BaselineChecker {
    /// Create a baseline checker.
    ///
    /// # Arguments
    /// - `tail_fraction` - Fraction of data to use for baseline (0.05 to 0.3 typical)
    /// - `tolerance` - Maximum acceptable deviation from 1.0
    pub fn new(tail_fraction: f64, tolerance: f64) -> Self {
        Self {
            tail_fraction: tail_fraction.clamp(0.01, 0.5),
            tolerance: tolerance.max(0.0),
        }
    }

    /// Default checker: 10% tail, 0.01 tolerance.
    pub fn default_checker() -> Self {
        Self::new(0.1, 0.01)
    }

    /// Assess the baseline of g2(tau) data.
    pub fn check(&self, g2_data: &[(usize, f64)]) -> BaselineQuality {
        let n = g2_data.len();
        if n < 3 {
            return BaselineQuality {
                baseline: 1.0,
                noise: 0.0,
                is_acceptable: false,
                deviation: f64::NAN,
            };
        }

        let tail_start = (n as f64 * (1.0 - self.tail_fraction)).ceil() as usize;
        let tail_start = tail_start.min(n - 1);
        let tail = &g2_data[tail_start..];

        if tail.is_empty() {
            return BaselineQuality {
                baseline: 1.0,
                noise: 0.0,
                is_acceptable: false,
                deviation: f64::NAN,
            };
        }

        let baseline: f64 = tail.iter().map(|&(_, g2)| g2).sum::<f64>() / tail.len() as f64;
        let variance: f64 = tail
            .iter()
            .map(|&(_, g2)| (g2 - baseline).powi(2))
            .sum::<f64>()
            / tail.len() as f64;
        let noise = variance.sqrt();
        let deviation = (baseline - 1.0).abs();

        BaselineQuality {
            baseline,
            noise,
            is_acceptable: deviation < self.tolerance,
            deviation,
        }
    }
}

// ---------------------------------------------------------------------------
// TemperatureCorrector
// ---------------------------------------------------------------------------

/// Temperature-dependent viscosity correction for common solvents.
///
/// For water, uses the empirical Vogel-Fulcher-Tammann (VFT) equation:
///
/// ```text
///   eta(T) = A * exp(B / (T - C))
/// ```
///
/// where for water: A = 2.414e-5 Pa*s, B = 247.8 K, C = 140.0 K
/// (valid approximately 0-100 C).
pub struct TemperatureCorrector;

impl TemperatureCorrector {
    /// Water dynamic viscosity at a given temperature (K).
    ///
    /// Uses the Vogel-Fulcher-Tammann equation.
    /// Valid range: approximately 273-373 K (0-100 C).
    ///
    /// Returns viscosity in Pa*s (SI units).
    pub fn water_viscosity(temperature_k: f64) -> f64 {
        // VFT parameters for water
        let a = 2.414e-5; // Pa*s
        let b = 247.8; // K
        let c = 140.0; // K
        let denom = temperature_k - c;
        if denom <= 0.0 {
            return 1.0e-3; // fallback to ~room temperature
        }
        a * (b / denom).exp()
    }

    /// Correct a hydrodynamic diameter measured at one temperature to another.
    ///
    /// Since D = kT/(3*pi*eta*d_H), and the particle doesn't change size,
    /// the measured d_H changes with T and eta. This function computes what
    /// the measurement at T_ref would give if actually measured at T_meas.
    ///
    /// d_H(T_ref) = d_H(T_meas) * (T_ref * eta_meas) / (T_meas * eta_ref)
    pub fn correct_diameter(
        d_h_measured: f64,
        t_meas: f64,
        eta_meas: f64,
        t_ref: f64,
        eta_ref: f64,
    ) -> f64 {
        if t_meas.abs() < 1e-10 || eta_ref.abs() < 1e-30 {
            return d_h_measured;
        }
        d_h_measured * (t_ref * eta_meas) / (t_meas * eta_ref)
    }

    /// Correct diffusion coefficient from measurement temperature to reference.
    ///
    /// D_ref = D_meas * (T_ref / T_meas) * (eta_meas / eta_ref)
    pub fn correct_diffusion(
        d_meas: f64,
        t_meas: f64,
        eta_meas: f64,
        t_ref: f64,
        eta_ref: f64,
    ) -> f64 {
        if t_meas.abs() < 1e-10 || eta_ref.abs() < 1e-30 {
            return d_meas;
        }
        d_meas * (t_ref / t_meas) * (eta_meas / eta_ref)
    }
}

// ---------------------------------------------------------------------------
// Utility: generate synthetic DLS data for testing
// ---------------------------------------------------------------------------

/// Generate a synthetic monodisperse autocorrelation function for testing.
///
/// Returns (tau_values, g2_values) for a monodisperse sample.
pub fn generate_synthetic_g2(
    config: &DlsConfig,
    diameter: f64,
    beta: f64,
    n_channels: usize,
    tau_min: f64,
    tau_max: f64,
) -> (Vec<f64>, Vec<f64>) {
    let gamma = StokesEinsteinEquation::decay_rate_from_diameter(config, diameter);
    let mut taus = Vec::with_capacity(n_channels);
    let mut g2s = Vec::with_capacity(n_channels);

    let log_min = tau_min.ln();
    let log_max = tau_max.ln();

    for i in 0..n_channels {
        let frac = i as f64 / (n_channels - 1).max(1) as f64;
        let tau = (log_min + frac * (log_max - log_min)).exp();
        let g1 = (-gamma * tau).exp();
        let g2 = 1.0 + beta * g1 * g1;
        taus.push(tau);
        g2s.push(g2);
    }

    (taus, g2s)
}

/// Generate a synthetic bimodal autocorrelation function for testing.
///
/// Returns (tau_values, g2_values) for a bimodal sample.
pub fn generate_synthetic_bimodal_g2(
    config: &DlsConfig,
    diameter1: f64,
    diameter2: f64,
    fraction1: f64,
    beta: f64,
    n_channels: usize,
    tau_min: f64,
    tau_max: f64,
) -> (Vec<f64>, Vec<f64>) {
    let gamma1 = StokesEinsteinEquation::decay_rate_from_diameter(config, diameter1);
    let gamma2 = StokesEinsteinEquation::decay_rate_from_diameter(config, diameter2);
    let f1 = fraction1.clamp(0.0, 1.0);
    let f2 = 1.0 - f1;

    let mut taus = Vec::with_capacity(n_channels);
    let mut g2s = Vec::with_capacity(n_channels);

    let log_min = tau_min.ln();
    let log_max = tau_max.ln();

    for i in 0..n_channels {
        let frac = i as f64 / (n_channels - 1).max(1) as f64;
        let tau = (log_min + frac * (log_max - log_min)).exp();
        let g1 = f1 * (-gamma1 * tau).exp() + f2 * (-gamma2 * tau).exp();
        let g2 = 1.0 + beta * g1 * g1;
        taus.push(tau);
        g2s.push(g2);
    }

    (taus, g2s)
}

// =========================================================================
// Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    // --- DlsConfig tests ---

    #[test]
    fn test_config_creation() {
        let cfg = DlsConfig::new(633e-9, 90.0_f64.to_radians(), 298.15, 0.89e-3, 1.33);
        assert!((cfg.wavelength - 633e-9).abs() < EPS);
        assert!((cfg.temperature - 298.15).abs() < EPS);
        assert!((cfg.viscosity - 0.89e-3).abs() < EPS);
    }

    #[test]
    fn test_config_presets() {
        let back = DlsConfig::standard_backscatter();
        assert!((back.wavelength - 633e-9).abs() < EPS);
        assert!(back.scattering_angle > 3.0); // ~173 deg in radians

        let ninety = DlsConfig::standard_90deg();
        assert!((ninety.scattering_angle - PI / 2.0).abs() < 0.01);
    }

    #[test]
    fn test_scattering_vector_90deg() {
        let cfg = DlsConfig::new(633e-9, PI / 2.0, 298.15, 0.89e-3, 1.33);
        let q = cfg.scattering_vector();
        // Q = (4*pi*1.33/633e-9) * sin(45deg) = (4*pi*1.33/633e-9) * 0.7071
        let expected = (4.0 * PI * 1.33 / 633e-9) * (PI / 4.0).sin();
        assert!((q - expected).abs() / expected < 1e-10);
    }

    #[test]
    fn test_scattering_vector_backscatter() {
        let cfg = DlsConfig::standard_backscatter();
        let q = cfg.scattering_vector();
        // For 173 deg, sin(86.5 deg) ~ 0.9981
        // Q should be close to 4*pi*n/lambda
        let q_max = 4.0 * PI * 1.33 / 633e-9;
        assert!(q > 0.99 * q_max); // very close to maximum Q
    }

    #[test]
    fn test_scattering_vector_forward() {
        let cfg = DlsConfig::new(633e-9, 0.1, 298.15, 0.89e-3, 1.33); // ~5.7 deg
        let q = cfg.scattering_vector();
        assert!(q > 0.0);
        // Small angle = small Q
        let cfg_back = DlsConfig::standard_backscatter();
        assert!(q < cfg_back.scattering_vector());
    }

    // --- AutocorrelationCalculator tests ---

    #[test]
    fn test_autocorrelation_constant_signal() {
        let calc = AutocorrelationCalculator::new_linear(5, 10);
        let signal = vec![1.0; 100];
        let g2 = calc.compute_g2(&signal);
        // Constant signal: g2(tau) = 1.0 for all tau
        for &(_, g2_val) in &g2 {
            assert!((g2_val - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_autocorrelation_log_spacing() {
        let calc = AutocorrelationCalculator::new_log_spaced(10, 1000);
        let channels = calc.lag_channels();
        assert!(!channels.is_empty());
        assert_eq!(channels[0], 1);
        // Check increasing
        for i in 1..channels.len() {
            assert!(channels[i] >= channels[i - 1]);
        }
    }

    #[test]
    fn test_autocorrelation_custom_lags() {
        let calc = AutocorrelationCalculator::new_custom(vec![1, 5, 10, 50]);
        let signal: Vec<f64> = (0..200).map(|i| (i as f64 * 0.1).sin() + 2.0).collect();
        let g2 = calc.compute_g2(&signal);
        assert_eq!(g2.len(), 4);
        // g2 at lag 0 should be highest
        // Since all g2 values should be >= 1 for a positive signal
        for &(_, val) in &g2 {
            assert!(val > 0.0);
        }
    }

    #[test]
    fn test_autocorrelation_raw() {
        let calc = AutocorrelationCalculator::new_linear(3, 5);
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let raw = calc.compute_raw(&signal);
        assert!(!raw.is_empty());
        // Raw autocorrelation at lag 0 would be mean of x^2
        // But lag starts at 1
    }

    #[test]
    fn test_autocorrelation_empty() {
        let calc = AutocorrelationCalculator::new_linear(5, 10);
        let g2 = calc.compute_g2(&[]);
        assert!(g2.is_empty());
        let g2 = calc.compute_g2(&[1.0]);
        assert!(g2.is_empty());
    }

    // --- SiegertRelation tests ---

    #[test]
    fn test_siegert_roundtrip() {
        let sr = SiegertRelation::new(0.8);
        let g1_orig = 0.5;
        let g2 = sr.g1_to_g2(g1_orig);
        let g1_recovered = sr.g2_to_g1(g2).unwrap();
        assert!((g1_recovered - g1_orig).abs() < 1e-12);
    }

    #[test]
    fn test_siegert_g1_unity() {
        let sr = SiegertRelation::new(0.9);
        // At tau=0, g1=1, so g2 = 1 + beta
        let g2 = sr.g1_to_g2(1.0);
        assert!((g2 - 1.9).abs() < 1e-12);
    }

    #[test]
    fn test_siegert_g1_zero() {
        let sr = SiegertRelation::new(0.8);
        // At tau->inf, g1=0, so g2 = 1
        let g2 = sr.g1_to_g2(0.0);
        assert!((g2 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_siegert_unphysical() {
        let sr = SiegertRelation::new(0.8);
        // g2 < 1 is unphysical
        assert!(sr.g2_to_g1(0.5).is_none());
    }

    #[test]
    fn test_siegert_estimate_beta() {
        let beta_est = SiegertRelation::estimate_beta(1.85);
        assert!((beta_est - 0.85).abs() < 1e-12);
    }

    #[test]
    fn test_siegert_convert_vector() {
        let sr = SiegertRelation::new(0.9);
        let g2_data = vec![(1, 1.8), (2, 1.5), (3, 1.2), (4, 1.05)];
        let g1_data = sr.convert_g2_to_g1(&g2_data);
        assert_eq!(g1_data.len(), 4);
        // g1 values should be decreasing
        for i in 1..g1_data.len() {
            assert!(g1_data[i].1 <= g1_data[i - 1].1 + 1e-10);
        }
    }

    // --- StokesEinsteinEquation tests ---

    #[test]
    fn test_stokes_einstein_roundtrip() {
        let temp = 298.15;
        let eta = 0.89e-3;
        let d_h = 100e-9; // 100 nm
        let d = StokesEinsteinEquation::diffusion_coefficient(temp, eta, d_h);
        let d_h_recovered = StokesEinsteinEquation::hydrodynamic_diameter(temp, eta, d);
        assert!((d_h_recovered - d_h).abs() / d_h < 1e-10);
    }

    #[test]
    fn test_stokes_einstein_known_value() {
        // 100 nm particle in water at 25 C
        // D = kT/(3*pi*eta*d) = 1.38e-23 * 298.15 / (3*pi*0.89e-3*100e-9)
        let temp = 298.15;
        let eta = 0.89e-3;
        let d_h = 100e-9;
        let d = StokesEinsteinEquation::diffusion_coefficient(temp, eta, d_h);
        // Expected: ~4.9e-12 m^2/s
        assert!(d > 4.0e-12 && d < 6.0e-12);
    }

    #[test]
    fn test_larger_particles_slower_diffusion() {
        let temp = 298.15;
        let eta = 0.89e-3;
        let d_small = StokesEinsteinEquation::diffusion_coefficient(temp, eta, 10e-9);
        let d_large = StokesEinsteinEquation::diffusion_coefficient(temp, eta, 1000e-9);
        assert!(d_small > d_large);
        // D ratio should be ~100x (inverse of diameter ratio)
        let ratio = d_small / d_large;
        assert!((ratio - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_decay_rate_from_diameter() {
        let cfg = DlsConfig::standard_backscatter();
        let gamma_small = StokesEinsteinEquation::decay_rate_from_diameter(&cfg, 10e-9);
        let gamma_large = StokesEinsteinEquation::decay_rate_from_diameter(&cfg, 1000e-9);
        assert!(gamma_small > gamma_large); // small particles relax faster
    }

    #[test]
    fn test_diameter_from_decay_rate_roundtrip() {
        let cfg = DlsConfig::standard_backscatter();
        let d_orig = 200e-9;
        let gamma = StokesEinsteinEquation::decay_rate_from_diameter(&cfg, d_orig);
        let d_recovered = StokesEinsteinEquation::diameter_from_decay_rate(&cfg, gamma);
        assert!((d_recovered - d_orig).abs() / d_orig < 1e-10);
    }

    // --- CumulantAnalyzer tests ---

    #[test]
    fn test_cumulant_monodisperse() {
        let cfg = DlsConfig::standard_backscatter();
        let diameter = 100e-9;
        let gamma_true = StokesEinsteinEquation::decay_rate_from_diameter(&cfg, diameter);

        // Generate perfect exponential g1 data
        let n = 50;
        let mut g1_data = Vec::new();
        for i in 0..n {
            let tau = 1e-6 + (i as f64) * 5e-6;
            let g1 = (-gamma_true * tau).exp();
            g1_data.push((tau, g1));
        }

        let analyzer = CumulantAnalyzer::new(2);
        let result = analyzer.analyze(&g1_data).unwrap();

        // Gamma should match
        assert!(
            (result.gamma - gamma_true).abs() / gamma_true < 0.01,
            "Gamma: expected {}, got {}",
            gamma_true,
            result.gamma
        );
        // PDI should be ~0 for monodisperse
        assert!(result.pdi < 0.01, "PDI should be ~0, got {}", result.pdi);
    }

    #[test]
    fn test_cumulant_polydisperse() {
        // Bimodal -> higher PDI
        let cfg = DlsConfig::standard_backscatter();
        let g1 = StokesEinsteinEquation::decay_rate_from_diameter(&cfg, 50e-9);
        let g2 = StokesEinsteinEquation::decay_rate_from_diameter(&cfg, 200e-9);

        let n = 80;
        let mut data = Vec::new();
        for i in 0..n {
            let tau = 1e-6 + (i as f64) * 3e-6;
            let g1_val = 0.5 * (-g1 * tau).exp() + 0.5 * (-g2 * tau).exp();
            data.push((tau, g1_val));
        }

        let analyzer = CumulantAnalyzer::new(3);
        let result = analyzer.analyze(&data).unwrap();
        // PDI should be significant for bimodal
        assert!(result.pdi > 0.05, "PDI should be significant for bimodal, got {}", result.pdi);
    }

    #[test]
    fn test_cumulant_insufficient_data() {
        let analyzer = CumulantAnalyzer::new(2);
        assert!(analyzer.analyze(&[]).is_none());
        assert!(analyzer.analyze(&[(1.0, 0.5)]).is_none());
    }

    // --- ContiRegularization tests ---

    #[test]
    fn test_contin_monodisperse_recovery() {
        let cfg = DlsConfig::standard_backscatter();
        let diameter = 100e-9;
        let beta = 0.9;

        let (taus, g2s) = generate_synthetic_g2(&cfg, diameter, beta, 50, 1e-6, 1e-2);
        let sr = SiegertRelation::new(beta);

        let g1_data: Vec<(f64, f64)> = taus
            .iter()
            .zip(g2s.iter())
            .filter_map(|(&tau, &g2)| sr.g2_to_g1(g2).map(|g1| (tau, g1)))
            .collect();

        let contin = ContiRegularization::new(1e-3, 30, 10e-9, 1000e-9);
        let dist = contin.invert(&cfg, &g1_data);

        // Peak should be near 100 nm
        let peak = dist.peak_diameter();
        assert!(
            (peak - diameter).abs() / diameter < 0.5,
            "Peak at {} nm, expected ~100 nm",
            peak * 1e9
        );
    }

    #[test]
    fn test_contin_diameter_bins_log_spaced() {
        let contin = ContiRegularization::new(0.01, 20, 1e-9, 10e-6);
        let bins = contin.diameter_bins();
        assert_eq!(bins.len(), 20);
        assert!((bins[0] - 1e-9).abs() / 1e-9 < 0.01);
        // Check logarithmic spacing: ratio between consecutive bins should be ~constant
        let ratio1 = bins[1] / bins[0];
        let ratio2 = bins[2] / bins[1];
        assert!((ratio1 - ratio2).abs() / ratio1 < 0.01);
    }

    #[test]
    fn test_contin_empty_data() {
        let cfg = DlsConfig::standard_backscatter();
        let contin = ContiRegularization::new(0.01, 10, 1e-9, 1e-6);
        let dist = contin.invert(&cfg, &[]);
        assert!(dist.amplitudes.iter().all(|&a| a == 0.0));
    }

    // --- NumberDistribution tests ---

    #[test]
    fn test_number_distribution_shift() {
        // Intensity distribution peaked at large sizes should shift to smaller
        // sizes in number distribution
        let dist = SizeDistribution {
            diameters: vec![10e-9, 50e-9, 100e-9, 500e-9],
            amplitudes: vec![0.1, 0.2, 0.3, 0.4],
            residual: 0.0,
        };

        let number = NumberDistribution::from_intensity_rayleigh(&dist);
        // Number distribution should be dominated by small particles
        // (large particles contribute d^6 more intensity per particle)
        assert!(
            number.amplitudes[0] > number.amplitudes[3],
            "Small particles should dominate number dist"
        );
    }

    #[test]
    fn test_volume_distribution() {
        let dist = SizeDistribution {
            diameters: vec![10e-9, 100e-9, 1000e-9],
            amplitudes: vec![0.1, 0.3, 0.6],
            residual: 0.0,
        };

        let volume = NumberDistribution::from_intensity_volume(&dist);
        let total: f64 = volume.amplitudes.iter().sum();
        assert!((total - 1.0).abs() < 1e-10, "Volume distribution should be normalized");
    }

    // --- MultiAngleAnalyzer tests ---

    #[test]
    fn test_multi_angle_isotropic_sphere() {
        let cfg = DlsConfig::standard_backscatter();
        let diameter = 100e-9;

        // Generate Gamma at multiple angles for a sphere (D is constant)
        let angles: Vec<f64> = vec![30.0_f64, 60.0, 90.0, 120.0, 150.0, 173.0]
            .into_iter()
            .map(|deg: f64| deg.to_radians())
            .collect();

        let d_true = StokesEinsteinEquation::diffusion_coefficient(298.15, 0.89e-3, diameter);

        let mut pairs = Vec::new();
        for &angle in &angles {
            let mut c = cfg.clone();
            c.scattering_angle = angle;
            let q = c.scattering_vector();
            let gamma = d_true * q * q;
            pairs.push((angle, gamma));
        }

        let result = MultiAngleAnalyzer::analyze(&cfg, &pairs, 0.1);
        assert!(result.is_isotropic, "Sphere should be detected as isotropic");

        // D_apparent should be approximately constant
        let d_max = result.d_apparent.iter().cloned().fold(0.0_f64, f64::max);
        let d_min = result.d_apparent.iter().cloned().fold(f64::INFINITY, f64::min);
        let variation = (d_max - d_min) / d_max;
        assert!(variation < 0.01, "D_apparent variation should be small: {}", variation);
    }

    #[test]
    fn test_multi_angle_anisotropic() {
        let cfg = DlsConfig::standard_backscatter();

        // Simulate anisotropic particle: D_app = D0 + D_rot * Q^2
        let d0 = 4.0e-12;
        let d_rot_coeff = 1e-29;

        let angles: Vec<f64> = vec![30.0_f64, 60.0, 90.0, 120.0, 150.0]
            .into_iter()
            .map(|deg: f64| deg.to_radians())
            .collect();

        let mut pairs = Vec::new();
        for &angle in &angles {
            let mut c = cfg.clone();
            c.scattering_angle = angle;
            let q = c.scattering_vector();
            let q2 = q * q;
            let d_app = d0 + d_rot_coeff * q2;
            let gamma = d_app * q2;
            pairs.push((angle, gamma));
        }

        let result = MultiAngleAnalyzer::analyze(&cfg, &pairs, 0.01);
        // With d_rot_coeff > 0, the slope should be positive
        assert!(result.slope > 0.0, "Slope should be positive for anisotropic particle");
    }

    // --- PolydispersityEstimator tests ---

    #[test]
    fn test_pdi_classification_monodisperse() {
        let est = PolydispersityEstimator::classify(0.02);
        assert_eq!(est.class, PolydispersityClass::Monodisperse);
        assert!(est.cumulant_reliable);
    }

    #[test]
    fn test_pdi_classification_moderate() {
        let est = PolydispersityEstimator::classify(0.2);
        assert_eq!(est.class, PolydispersityClass::ModeratelyPolydisperse);
        assert!(est.cumulant_reliable);
    }

    #[test]
    fn test_pdi_classification_broad() {
        let est = PolydispersityEstimator::classify(0.5);
        assert_eq!(est.class, PolydispersityClass::BroadlyPolydisperse);
        assert!(est.cumulant_reliable);
    }

    #[test]
    fn test_pdi_classification_very_poly() {
        let est = PolydispersityEstimator::classify(0.8);
        assert_eq!(est.class, PolydispersityClass::VeryPolydisperse);
        assert!(!est.cumulant_reliable);
    }

    #[test]
    fn test_pdi_classification_nearly_mono() {
        let est = PolydispersityEstimator::classify(0.07);
        assert_eq!(est.class, PolydispersityClass::NearlyMonodisperse);
    }

    #[test]
    fn test_compute_pdi() {
        let gamma = 1000.0;
        let mu2 = 10000.0;
        let pdi = PolydispersityEstimator::compute_pdi(mu2, gamma);
        assert!((pdi - 0.01).abs() < 1e-10);
    }

    // --- BaselineChecker tests ---

    #[test]
    fn test_baseline_good() {
        // g2 that converges to 1.0
        let data: Vec<(usize, f64)> = (0..100)
            .map(|i| {
                let tau = i + 1;
                let g2 = 1.0 + 0.9 * (-(i as f64) * 0.05).exp();
                (tau, g2)
            })
            .collect();

        let checker = BaselineChecker::new(0.1, 0.02);
        let quality = checker.check(&data);
        assert!(
            quality.is_acceptable,
            "Baseline should be acceptable, deviation: {}",
            quality.deviation
        );
    }

    #[test]
    fn test_baseline_bad_high() {
        // g2 with elevated baseline (dust contamination)
        let data: Vec<(usize, f64)> = (0..100)
            .map(|i| {
                let tau = i + 1;
                let g2 = 1.05 + 0.9 * (-(i as f64) * 0.05).exp();
                (tau, g2)
            })
            .collect();

        let checker = BaselineChecker::new(0.1, 0.01);
        let quality = checker.check(&data);
        assert!(!quality.is_acceptable, "Elevated baseline should fail check");
        assert!(quality.deviation > 0.04);
    }

    #[test]
    fn test_baseline_insufficient_data() {
        let checker = BaselineChecker::default_checker();
        let quality = checker.check(&[(1, 1.5)]);
        assert!(!quality.is_acceptable);
    }

    // --- TemperatureCorrector tests ---

    #[test]
    fn test_water_viscosity_25c() {
        let eta = TemperatureCorrector::water_viscosity(298.15);
        // Should be approximately 0.89e-3 Pa*s at 25C
        assert!(
            (eta - 0.89e-3).abs() < 0.1e-3,
            "Water viscosity at 25C: {} Pa*s",
            eta
        );
    }

    #[test]
    fn test_water_viscosity_temperature_dependence() {
        let eta_cold = TemperatureCorrector::water_viscosity(278.15); // 5C
        let eta_room = TemperatureCorrector::water_viscosity(298.15); // 25C
        let eta_hot = TemperatureCorrector::water_viscosity(333.15); // 60C

        // Viscosity decreases with temperature
        assert!(eta_cold > eta_room);
        assert!(eta_room > eta_hot);
    }

    #[test]
    fn test_temperature_diameter_correction() {
        let d_meas = 100e-9;
        let t_meas = 298.15;
        let eta_meas = 0.89e-3;
        let t_ref = 293.15;
        let eta_ref = 1.0e-3;

        let d_corr =
            TemperatureCorrector::correct_diameter(d_meas, t_meas, eta_meas, t_ref, eta_ref);
        // Different temperature/viscosity should give different apparent diameter
        assert!(d_corr > 0.0);
        assert!((d_corr - d_meas).abs() > 1e-12); // should be different
    }

    #[test]
    fn test_diffusion_correction() {
        let d = 4.9e-12;
        let t1 = 298.15;
        let eta1 = 0.89e-3;
        let t2 = 310.0;
        let eta2 = 0.69e-3;

        let d_corrected =
            TemperatureCorrector::correct_diffusion(d, t1, eta1, t2, eta2);
        // Higher T and lower viscosity -> larger D
        assert!(d_corrected > d);
    }

    // --- Synthetic data generation tests ---

    #[test]
    fn test_synthetic_g2_shape() {
        let cfg = DlsConfig::standard_backscatter();
        let (taus, g2s) = generate_synthetic_g2(&cfg, 100e-9, 0.9, 50, 1e-6, 1e-2);

        assert_eq!(taus.len(), 50);
        assert_eq!(g2s.len(), 50);

        // g2 should start near 1 + beta and decay to 1
        assert!(g2s[0] > 1.5, "g2 at short tau should be high: {}", g2s[0]);
        assert!((g2s[49] - 1.0).abs() < 0.1, "g2 at long tau should be ~1: {}", g2s[49]);

        // Should be monotonically decreasing
        for i in 1..g2s.len() {
            assert!(g2s[i] <= g2s[i - 1] + 1e-10);
        }
    }

    #[test]
    fn test_synthetic_bimodal() {
        let cfg = DlsConfig::standard_backscatter();
        let (taus, g2s) =
            generate_synthetic_bimodal_g2(&cfg, 50e-9, 500e-9, 0.5, 0.9, 60, 1e-6, 1e-2);

        assert_eq!(taus.len(), 60);
        assert!(g2s[0] > 1.0);
        assert!((g2s[59] - 1.0).abs() < 0.2);
    }

    // --- SizeDistribution helper tests ---

    #[test]
    fn test_size_distribution_mean() {
        let dist = SizeDistribution {
            diameters: vec![10e-9, 20e-9, 30e-9],
            amplitudes: vec![0.25, 0.50, 0.25],
            residual: 0.0,
        };
        let mean = dist.mean_diameter();
        assert!((mean - 20e-9).abs() < 1e-12);
    }

    #[test]
    fn test_size_distribution_peak() {
        let dist = SizeDistribution {
            diameters: vec![10e-9, 20e-9, 30e-9, 40e-9],
            amplitudes: vec![0.1, 0.2, 0.5, 0.2],
            residual: 0.0,
        };
        let peak = dist.peak_diameter();
        assert!((peak - 30e-9).abs() < 1e-12);
    }

    #[test]
    fn test_size_distribution_percentiles() {
        let dist = SizeDistribution {
            diameters: vec![10e-9, 20e-9, 30e-9, 40e-9, 50e-9],
            amplitudes: vec![0.2, 0.2, 0.2, 0.2, 0.2],
            residual: 0.0,
        };
        let (d10, d50, d90) = dist.percentiles();
        assert!(d10 > 0.0);
        assert!(d50 >= d10);
        assert!(d90 >= d50);
    }

    // --- Integration / end-to-end tests ---

    #[test]
    fn test_full_pipeline_monodisperse() {
        // Full DLS analysis pipeline: config -> synthetic data -> g2 -> g1 -> cumulant -> size
        let cfg = DlsConfig::standard_backscatter();
        let true_diameter = 150e-9; // 150 nm
        let beta = 0.85;

        // Generate synthetic g2
        let (taus, g2s) = generate_synthetic_g2(&cfg, true_diameter, beta, 80, 1e-6, 5e-3);

        // Convert g2 -> g1 via Siegert
        let sr = SiegertRelation::new(beta);
        let g1_data: Vec<(f64, f64)> = taus
            .iter()
            .zip(g2s.iter())
            .filter_map(|(&tau, &g2)| sr.g2_to_g1(g2).map(|g1| (tau, g1)))
            .collect();

        assert!(!g1_data.is_empty());

        // Cumulant analysis
        let analyzer = CumulantAnalyzer::new(2);
        let result = analyzer.analyze(&g1_data).unwrap();

        // Recover diameter
        let recovered = StokesEinsteinEquation::diameter_from_decay_rate(&cfg, result.gamma);
        let error_pct = ((recovered - true_diameter) / true_diameter).abs() * 100.0;
        assert!(
            error_pct < 5.0,
            "Recovered diameter {:.1} nm vs true {:.1} nm ({:.1}% error)",
            recovered * 1e9,
            true_diameter * 1e9,
            error_pct
        );

        // PDI should be very small
        assert!(result.pdi < 0.01, "PDI for monodisperse: {}", result.pdi);
    }

    #[test]
    fn test_polynomial_least_squares_linear() {
        // y = 2 + 3x
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![2.0, 5.0, 8.0, 11.0, 14.0];
        let coeffs = polynomial_least_squares(&x, &y, 1);
        assert!((coeffs[0] - 2.0).abs() < 1e-8);
        assert!((coeffs[1] - 3.0).abs() < 1e-8);
    }

    #[test]
    fn test_polynomial_least_squares_quadratic() {
        // y = 1 + 2x + 0.5x^2
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| 1.0 + 2.0 * xi + 0.5 * xi * xi).collect();
        let coeffs = polynomial_least_squares(&x, &y, 2);
        assert!((coeffs[0] - 1.0).abs() < 1e-6, "c0: {}", coeffs[0]);
        assert!((coeffs[1] - 2.0).abs() < 1e-6, "c1: {}", coeffs[1]);
        assert!((coeffs[2] - 0.5).abs() < 1e-6, "c2: {}", coeffs[2]);
    }

    #[test]
    fn test_linear_regression() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.1, 4.0, 5.9, 8.1, 9.9];
        let (intercept, slope) = linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < 0.2);
        assert!(intercept.abs() < 0.5);
    }
}

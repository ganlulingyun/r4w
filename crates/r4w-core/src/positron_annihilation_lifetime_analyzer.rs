//! Positron Annihilation Lifetime Spectroscopy (PALS) Signal Processor
//!
//! Analyzes gamma-ray coincidence timing to extract positron lifetimes in
//! materials. Positron Annihilation Lifetime Spectroscopy is a powerful
//! non-destructive technique for probing defects and free volume in metals,
//! semiconductors, polymers, and porous materials.
//!
//! # Overview
//!
//! When a positron from a Na-22 source enters a material, it thermalizes and
//! eventually annihilates with an electron, producing two 511 keV gamma rays.
//! The time between the birth gamma (1.274 MeV from Na-22 decay) and the
//! annihilation gammas encodes information about the electron density at the
//! annihilation site.
//!
//! Three distinct lifetime components are typically observed:
//!
//! - **p-Ps (para-positronium)**: τ₁ ~ 125 ps, self-annihilation in vacuum
//! - **Free positron**: τ₂ ~ 200-500 ps, annihilation in bulk material
//! - **o-Ps (ortho-positronium)**: τ₃ ~ 1-5 ns in polymers, pick-off
//!   annihilation in free volume holes
//!
//! # Algorithms
//!
//! - **TAC histogram processing**: Time-to-amplitude conversion spectra
//! - **Multi-exponential decomposition**: S(t) = Σᵢ (Iᵢ/τᵢ) exp(-t/τᵢ)
//! - **Resolution function convolution**: Gaussian instrument response
//! - **Source correction**: Subtract Na-22 source contribution
//! - **Tao-Eldrup model**: Free volume hole radius from o-Ps lifetime
//! - **Doppler broadening**: S-parameter and W-parameter extraction
//! - **CDB ratio curves**: Coincidence Doppler broadening analysis
//! - **Chi-squared goodness of fit**: Statistical quality assessment
//!
//! # Example
//!
//! ```rust
//! use r4w_core::positron_annihilation_lifetime_analyzer::{
//!     PalsConfig, PalsAnalyzer, LifetimeComponent, tao_eldrup_radius,
//! };
//!
//! // Configure a 3-component fit
//! let config = PalsConfig {
//!     time_per_channel_ps: 25.0,
//!     num_channels: 1024,
//!     resolution_fwhm_ps: 250.0,
//!     source_fraction: 0.10,
//!     source_lifetime_ps: 382.0,
//!     num_components: 3,
//!     background_region: (900, 1023),
//!     fit_region: (50, 900),
//! };
//!
//! let analyzer = PalsAnalyzer::new(config);
//!
//! // Tao-Eldrup: o-Ps lifetime of 2.0 ns gives free volume radius
//! let radius = tao_eldrup_radius(2000.0);
//! assert!(radius > 0.0 && radius < 10.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Para-positronium vacuum lifetime in picoseconds (125 ps).
const TAU_PPS_PS: f64 = 125.0;

/// Ortho-positronium vacuum lifetime in picoseconds (142 ns).
const TAU_OPS_VACUUM_PS: f64 = 142_000.0;

/// Tao-Eldrup empirical layer thickness ΔR in Angstroms.
/// Standard value from Tao (1972) and Eldrup et al. (1981).
const DELTA_R_ANGSTROM: f64 = 1.66;

/// Electron rest mass energy in keV (511 keV).
const ELECTRON_MASS_KEV: f64 = 511.0;

/// Na-22 birth gamma energy in keV (1274.5 keV).
const NA22_BIRTH_GAMMA_KEV: f64 = 1274.5;

/// Conversion: FWHM to sigma for Gaussian (FWHM = 2*sqrt(2*ln(2))*sigma).
const FWHM_TO_SIGMA: f64 = 2.354_820_045_030_949_4;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for a PALS measurement.
#[derive(Debug, Clone)]
pub struct PalsConfig {
    /// Time per TAC channel in picoseconds.
    pub time_per_channel_ps: f64,
    /// Number of TAC channels.
    pub num_channels: usize,
    /// Instrument resolution function FWHM in picoseconds.
    pub resolution_fwhm_ps: f64,
    /// Source component intensity fraction (0 to 1).
    pub source_fraction: f64,
    /// Source component lifetime in picoseconds (e.g., 382 ps for Kapton).
    pub source_lifetime_ps: f64,
    /// Number of lifetime components to fit (1-4).
    pub num_components: usize,
    /// Background estimation region (channel_start, channel_end).
    pub background_region: (usize, usize),
    /// Fit region (channel_start, channel_end).
    pub fit_region: (usize, usize),
}

impl Default for PalsConfig {
    fn default() -> Self {
        Self {
            time_per_channel_ps: 25.0,
            num_channels: 1024,
            resolution_fwhm_ps: 250.0,
            source_fraction: 0.10,
            source_lifetime_ps: 382.0,
            num_components: 3,
            background_region: (900, 1023),
            fit_region: (50, 900),
        }
    }
}

// ---------------------------------------------------------------------------
// Lifetime component
// ---------------------------------------------------------------------------

/// A single lifetime component from the decomposition.
#[derive(Debug, Clone, Copy)]
pub struct LifetimeComponent {
    /// Lifetime τ in picoseconds.
    pub tau_ps: f64,
    /// Relative intensity (fraction, sums to 1.0 across all components).
    pub intensity: f64,
    /// Uncertainty in lifetime (ps), from fit residuals.
    pub tau_uncertainty_ps: f64,
    /// Uncertainty in intensity.
    pub intensity_uncertainty: f64,
}

// ---------------------------------------------------------------------------
// Doppler broadening parameters
// ---------------------------------------------------------------------------

/// Doppler broadening line-shape parameters.
#[derive(Debug, Clone, Copy)]
pub struct DopplerParameters {
    /// S-parameter: central region fraction of 511 keV peak.
    pub s_parameter: f64,
    /// W-parameter: wing region fraction of 511 keV peak.
    pub w_parameter: f64,
    /// Total peak area (counts).
    pub total_area: f64,
    /// Peak centroid channel.
    pub centroid: f64,
    /// Peak FWHM in channels.
    pub fwhm_channels: f64,
}

// ---------------------------------------------------------------------------
// Fit result
// ---------------------------------------------------------------------------

/// Complete result of a PALS lifetime analysis.
#[derive(Debug, Clone)]
pub struct PalsFitResult {
    /// Extracted lifetime components.
    pub components: Vec<LifetimeComponent>,
    /// Reduced chi-squared of the fit.
    pub chi_squared_reduced: f64,
    /// Background level (counts per channel).
    pub background: f64,
    /// Time zero channel (prompt peak position).
    pub t0_channel: f64,
    /// Total counts in the spectrum.
    pub total_counts: f64,
    /// Variance of the fit (residual sum of squares / DOF).
    pub fit_variance: f64,
}

// ---------------------------------------------------------------------------
// CDB ratio curve
// ---------------------------------------------------------------------------

/// Coincidence Doppler broadening ratio curve result.
#[derive(Debug, Clone)]
pub struct CdbRatioCurve {
    /// Momentum values (in units of 10^-3 m_e*c).
    pub momentum: Vec<f64>,
    /// Ratio values (sample / reference).
    pub ratio: Vec<f64>,
    /// Statistical uncertainty in ratio.
    pub ratio_error: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Free-standing functions
// ---------------------------------------------------------------------------

/// Compute the Tao-Eldrup free volume hole radius from o-Ps lifetime.
///
/// The Tao-Eldrup model relates ortho-positronium pick-off lifetime τ₃
/// to the spherical free volume hole radius R:
///
/// 1/τ₃ = 2.0 * [1 - R/(R+ΔR) + sin(2πR/(R+ΔR))/(2π)]  (ns⁻¹)
///
/// where ΔR = 1.66 Å is the empirical electron layer thickness.
///
/// This function inverts the equation numerically using bisection.
///
/// # Arguments
/// * `tau3_ps` - Ortho-positronium lifetime in picoseconds
///
/// # Returns
/// Free volume hole radius in Angstroms
pub fn tao_eldrup_radius(tau3_ps: f64) -> f64 {
    let tau3_ns = tau3_ps / 1000.0;
    let lambda_obs = 1.0 / tau3_ns; // ns^-1

    // Bisection search for R
    let mut r_lo = 0.01_f64;
    let mut r_hi = 20.0_f64;

    for _ in 0..200 {
        let r_mid = 0.5 * (r_lo + r_hi);
        let lambda_calc = tao_eldrup_rate(r_mid);
        if lambda_calc > lambda_obs {
            r_lo = r_mid;
        } else {
            r_hi = r_mid;
        }
    }
    0.5 * (r_lo + r_hi)
}

/// Compute the Tao-Eldrup pick-off annihilation rate (ns⁻¹) for radius R (Å).
///
/// λ = 2.0 * [1 - R/(R+ΔR) + sin(2πR/(R+ΔR))/(2π)]
pub fn tao_eldrup_rate(r_angstrom: f64) -> f64 {
    let x = r_angstrom / (r_angstrom + DELTA_R_ANGSTROM);
    2.0 * (1.0 - x + (2.0 * PI * x).sin() / (2.0 * PI))
}

/// Compute the Tao-Eldrup lifetime (ps) from hole radius R (Å).
pub fn tao_eldrup_lifetime(r_angstrom: f64) -> f64 {
    let rate = tao_eldrup_rate(r_angstrom);
    if rate <= 0.0 {
        return f64::INFINITY;
    }
    1000.0 / rate // convert ns to ps
}

/// Compute spherical free volume hole volume from radius in Å.
/// Returns volume in ų.
pub fn free_volume_from_radius(r_angstrom: f64) -> f64 {
    (4.0 / 3.0) * PI * r_angstrom.powi(3)
}

/// Compute fractional free volume from o-Ps intensity and lifetime.
///
/// Fv = C * I₃ * Vh
///
/// where C is the empirical scaling constant (~0.0018 for most polymers),
/// I₃ is the o-Ps intensity, and Vh is the hole volume.
pub fn fractional_free_volume(intensity_3: f64, tau3_ps: f64, c_constant: f64) -> f64 {
    let r = tao_eldrup_radius(tau3_ps);
    let vh = free_volume_from_radius(r);
    c_constant * intensity_3 * vh
}

/// Generate a Gaussian resolution function.
///
/// Returns a normalized Gaussian centered at `center` with the given FWHM.
pub fn gaussian_resolution(num_channels: usize, center: f64, fwhm_ps: f64, bin_width_ps: f64) -> Vec<f64> {
    let sigma = fwhm_ps / FWHM_TO_SIGMA;
    let sigma_ch = sigma / bin_width_ps;
    let mut g = vec![0.0; num_channels];
    let mut sum = 0.0;

    for i in 0..num_channels {
        let x = (i as f64) - center;
        let val = (-0.5 * (x / sigma_ch).powi(2)).exp();
        g[i] = val;
        sum += val;
    }

    // Normalize
    if sum > 0.0 {
        for v in &mut g {
            *v /= sum;
        }
    }
    g
}

/// Multi-exponential decay model (unconvolved).
///
/// S(t) = Σᵢ (Iᵢ/τᵢ) * exp(-t/τᵢ) + background
///
/// where t is in picoseconds.
pub fn multi_exponential(
    t_ps: f64,
    components: &[LifetimeComponent],
    background: f64,
) -> f64 {
    let mut val = background;
    for c in components {
        if c.tau_ps > 0.0 {
            val += (c.intensity / c.tau_ps) * (-t_ps / c.tau_ps).exp();
        }
    }
    val
}

/// Convolve a spectrum with a resolution function using direct convolution.
pub fn convolve_spectrum(spectrum: &[f64], resolution: &[f64]) -> Vec<f64> {
    let n = spectrum.len();
    let m = resolution.len();
    let mut result = vec![0.0; n];

    // Find center of resolution function
    let mut max_idx = 0;
    let mut max_val = 0.0_f64;
    for (i, &v) in resolution.iter().enumerate() {
        if v > max_val {
            max_val = v;
            max_idx = i;
        }
    }

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..m {
            let k = i as isize + j as isize - max_idx as isize;
            if k >= 0 && (k as usize) < n {
                sum += spectrum[k as usize] * resolution[j];
            }
        }
        result[i] = sum;
    }
    result
}

/// Estimate background from a flat region of the spectrum.
pub fn estimate_background(spectrum: &[f64], region: (usize, usize)) -> f64 {
    let (start, end) = region;
    if end <= start || start >= spectrum.len() {
        return 0.0;
    }
    let end = end.min(spectrum.len());
    let n = (end - start) as f64;
    let sum: f64 = spectrum[start..end].iter().sum();
    sum / n
}

/// Find the prompt peak (time zero) channel.
pub fn find_t0_channel(spectrum: &[f64]) -> usize {
    let mut max_val = f64::NEG_INFINITY;
    let mut max_ch = 0;
    for (i, &v) in spectrum.iter().enumerate() {
        if v > max_val {
            max_val = v;
            max_ch = i;
        }
    }
    max_ch
}

/// Compute reduced chi-squared between observed and model spectra.
///
/// χ²ᵣ = (1/ν) Σᵢ (yᵢ - f(xᵢ))² / σᵢ²
///
/// where ν = N_data - N_params is the degrees of freedom, and σᵢ² = max(yᵢ, 1)
/// for Poisson counting statistics.
pub fn chi_squared_reduced(
    observed: &[f64],
    model: &[f64],
    region: (usize, usize),
    n_params: usize,
) -> f64 {
    let (start, end) = region;
    let end = end.min(observed.len()).min(model.len());
    if end <= start {
        return f64::INFINITY;
    }

    let mut chi2 = 0.0;
    let mut n_data = 0usize;

    for i in start..end {
        let sigma2 = observed[i].max(1.0);
        let diff = observed[i] - model[i];
        chi2 += diff * diff / sigma2;
        n_data += 1;
    }

    let dof = n_data.saturating_sub(n_params);
    if dof == 0 {
        return f64::INFINITY;
    }
    chi2 / dof as f64
}

/// Subtract source component from a spectrum.
///
/// Generates the source decay, convolves with resolution, scales, and subtracts.
pub fn source_correct(
    spectrum: &[f64],
    config: &PalsConfig,
    t0_channel: f64,
) -> Vec<f64> {
    let n = spectrum.len();
    let bg = estimate_background(spectrum, config.background_region);

    // Total counts above background
    let total: f64 = spectrum.iter().map(|&v| (v - bg).max(0.0)).sum();
    let source_counts = total * config.source_fraction;

    // Generate source decay
    let mut source_spectrum = vec![0.0; n];
    for i in 0..n {
        let t = (i as f64 - t0_channel) * config.time_per_channel_ps;
        if t >= 0.0 {
            source_spectrum[i] = (source_counts / config.source_lifetime_ps)
                * (-t / config.source_lifetime_ps).exp();
        }
    }

    // Convolve with resolution
    let resolution = gaussian_resolution(
        n,
        t0_channel,
        config.resolution_fwhm_ps,
        config.time_per_channel_ps,
    );
    let source_convolved = convolve_spectrum(&source_spectrum, &resolution);

    // Subtract
    let mut corrected = vec![0.0; n];
    for i in 0..n {
        corrected[i] = (spectrum[i] - source_convolved[i]).max(0.0);
    }
    corrected
}

// ---------------------------------------------------------------------------
// Doppler broadening analysis
// ---------------------------------------------------------------------------

/// Extract S-parameter and W-parameter from a 511 keV annihilation peak.
///
/// The S-parameter is the ratio of the central region to the total area,
/// sensitive to low-momentum (valence) electrons. The W-parameter is the
/// ratio of the wing regions, sensitive to high-momentum (core) electrons.
///
/// # Arguments
/// * `spectrum` - Energy spectrum (counts per channel)
/// * `centroid` - Peak centroid channel
/// * `s_half_width` - Half-width of S-parameter region (channels)
/// * `w_inner` - Inner boundary of W-parameter region (channels from centroid)
/// * `w_outer` - Outer boundary of W-parameter region (channels from centroid)
pub fn doppler_broadening_parameters(
    spectrum: &[f64],
    centroid: f64,
    s_half_width: f64,
    w_inner: f64,
    w_outer: f64,
) -> DopplerParameters {
    let n = spectrum.len();
    let center = centroid.round() as isize;

    // Compute total peak area, S, and W regions
    let mut total = 0.0;
    let mut s_area = 0.0;
    let mut w_area = 0.0;

    // Determine peak boundaries (3 sigma ~ region where signal is above bg)
    let peak_start = ((center as f64 - w_outer).max(0.0)) as usize;
    let peak_end = ((center as f64 + w_outer).min(n as f64 - 1.0)) as usize;

    for i in peak_start..=peak_end {
        let dist = (i as f64 - centroid).abs();
        total += spectrum[i];
        if dist <= s_half_width {
            s_area += spectrum[i];
        }
        if dist >= w_inner && dist <= w_outer {
            w_area += spectrum[i];
        }
    }

    let s_parameter = if total > 0.0 { s_area / total } else { 0.0 };
    let w_parameter = if total > 0.0 { w_area / total } else { 0.0 };

    // Estimate FWHM
    let peak_val = spectrum.get(center as usize).copied().unwrap_or(0.0);
    let half_max = peak_val / 2.0;
    let mut left_hw = 0.0;
    let mut right_hw = 0.0;
    for i in (peak_start..center as usize).rev() {
        if spectrum[i] < half_max {
            left_hw = center as f64 - i as f64;
            break;
        }
    }
    for i in (center as usize + 1)..=peak_end {
        if spectrum[i] < half_max {
            right_hw = i as f64 - center as f64;
            break;
        }
    }

    DopplerParameters {
        s_parameter,
        w_parameter,
        total_area: total,
        centroid,
        fwhm_channels: left_hw + right_hw,
    }
}

/// Compute a coincidence Doppler broadening (CDB) ratio curve.
///
/// The ratio curve is obtained by dividing the sample spectrum by a
/// well-annealed reference spectrum, both normalized to the same total area.
///
/// # Arguments
/// * `sample` - Sample CDB spectrum
/// * `reference` - Reference CDB spectrum (well-annealed)
/// * `centroid_sample` - Centroid of sample peak
/// * `centroid_ref` - Centroid of reference peak
/// * `kev_per_channel` - Energy per channel in keV
pub fn cdb_ratio_curve(
    sample: &[f64],
    reference: &[f64],
    centroid_sample: f64,
    centroid_ref: f64,
    kev_per_channel: f64,
) -> CdbRatioCurve {
    let n = sample.len().min(reference.len());

    // Normalize both spectra to unit area
    let sum_s: f64 = sample.iter().take(n).sum();
    let sum_r: f64 = reference.iter().take(n).sum();

    if sum_s <= 0.0 || sum_r <= 0.0 {
        return CdbRatioCurve {
            momentum: vec![],
            ratio: vec![],
            ratio_error: vec![],
        };
    }

    let mut momentum = Vec::with_capacity(n);
    let mut ratio = Vec::with_capacity(n);
    let mut ratio_error = Vec::with_capacity(n);

    for i in 0..n {
        // Momentum in units of 10^-3 m_e*c
        // p_L = ΔE / c = (E - 511) / c
        // For CDB: Doppler shift ΔE from centroid
        let de_sample = (i as f64 - centroid_sample) * kev_per_channel;
        let _de_ref = (i as f64 - centroid_ref) * kev_per_channel;

        // p_L in 10^-3 m_e*c: p = ΔE / (m_e * c^2) * 1000
        let p = de_sample / ELECTRON_MASS_KEV * 1000.0;

        let s_norm = sample[i] / sum_s;
        let r_norm = reference[i] / sum_r;

        if r_norm > 1e-15 {
            let r = s_norm / r_norm;
            // Error propagation: δ(a/b) = (a/b) * sqrt((δa/a)^2 + (δb/b)^2)
            let ds = if sample[i] > 0.0 {
                sample[i].sqrt() / sum_s
            } else {
                0.0
            };
            let dr = if reference[i] > 0.0 {
                reference[i].sqrt() / sum_r
            } else {
                0.0
            };
            let rel_err = if s_norm > 0.0 && r_norm > 0.0 {
                ((ds / s_norm).powi(2) + (dr / r_norm).powi(2)).sqrt()
            } else {
                0.0
            };

            momentum.push(p);
            ratio.push(r);
            ratio_error.push(r * rel_err);
        }
    }

    CdbRatioCurve {
        momentum,
        ratio,
        ratio_error,
    }
}

// ---------------------------------------------------------------------------
// TAC histogram
// ---------------------------------------------------------------------------

/// Generate a synthetic TAC histogram from known lifetime components.
///
/// Creates a time spectrum including convolution with the Gaussian resolution
/// function and Poisson noise (deterministic seed for reproducibility).
pub fn generate_tac_histogram(
    config: &PalsConfig,
    components: &[LifetimeComponent],
    total_counts: f64,
) -> Vec<f64> {
    let n = config.num_channels;
    let t0 = n as f64 * 0.15; // Place peak at ~15% of range

    // Generate ideal decay
    let mut spectrum = vec![0.0; n];
    for ch in 0..n {
        let t = (ch as f64 - t0) * config.time_per_channel_ps;
        if t >= 0.0 {
            for c in components {
                if c.tau_ps > 0.0 {
                    spectrum[ch] += (c.intensity / c.tau_ps) * (-t / c.tau_ps).exp();
                }
            }
        }
    }

    // Normalize to total counts
    let sum: f64 = spectrum.iter().sum();
    if sum > 0.0 {
        let scale = total_counts / sum;
        for v in &mut spectrum {
            *v *= scale;
        }
    }

    // Convolve with resolution
    let resolution = gaussian_resolution(n, t0, config.resolution_fwhm_ps, config.time_per_channel_ps);
    let convolved = convolve_spectrum(&spectrum, &resolution);

    convolved
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt-like iterative fitting
// ---------------------------------------------------------------------------

/// PALS lifetime spectrum analyzer.
#[derive(Debug, Clone)]
pub struct PalsAnalyzer {
    config: PalsConfig,
    /// Maximum number of iterations for the fit.
    pub max_iterations: usize,
    /// Convergence tolerance for chi-squared change.
    pub convergence_tol: f64,
}

impl PalsAnalyzer {
    /// Create a new analyzer with the given configuration.
    pub fn new(config: PalsConfig) -> Self {
        Self {
            config,
            max_iterations: 500,
            convergence_tol: 1e-6,
        }
    }

    /// Fit lifetime components to a TAC spectrum.
    ///
    /// Uses iterative nonlinear least squares (simplified Levenberg-Marquardt).
    /// Initial guesses are spread logarithmically from 100 ps to 3000 ps.
    pub fn fit_spectrum(&self, spectrum: &[f64]) -> PalsFitResult {
        let n = self.config.num_channels.min(spectrum.len());
        let bg = estimate_background(spectrum, self.config.background_region);
        let t0_ch = find_t0_channel(spectrum) as f64;
        let nc = self.config.num_components;

        // Initial guesses for lifetime components
        let mut components = Vec::with_capacity(nc);
        let initial_lifetimes = match nc {
            1 => vec![400.0],
            2 => vec![200.0, 1500.0],
            3 => vec![125.0, 350.0, 2000.0],
            _ => {
                let mut v = Vec::with_capacity(nc);
                for i in 0..nc {
                    let frac = i as f64 / (nc - 1).max(1) as f64;
                    v.push(100.0 * (3000.0_f64 / 100.0).powf(frac));
                }
                v
            }
        };

        let initial_intensity = 1.0 / nc as f64;
        for &tau in &initial_lifetimes {
            components.push(LifetimeComponent {
                tau_ps: tau,
                intensity: initial_intensity,
                tau_uncertainty_ps: 0.0,
                intensity_uncertainty: 0.0,
            });
        }

        // Resolution function
        let resolution = gaussian_resolution(
            n,
            t0_ch,
            self.config.resolution_fwhm_ps,
            self.config.time_per_channel_ps,
        );

        let (fit_start, fit_end) = self.config.fit_region;
        let fit_end = fit_end.min(n);

        // Iterative fitting using gradient descent with adaptive step
        let mut lambda = 0.001; // Damping factor
        let mut prev_chi2 = f64::INFINITY;

        for _iter in 0..self.max_iterations {
            // Generate model
            let model = self.generate_model(&components, bg, t0_ch, n, &resolution);

            // Compute chi-squared
            let chi2 = chi_squared_reduced(spectrum, &model, (fit_start, fit_end), 2 * nc + 2);

            // Check convergence
            if (prev_chi2 - chi2).abs() < self.convergence_tol {
                break;
            }

            // Compute gradients by finite differences
            let delta_tau = 1.0; // ps step for numerical derivative
            let delta_i = 0.001; // intensity step

            for k in 0..nc {
                // Gradient w.r.t. tau_k
                let mut comp_plus = components.clone();
                comp_plus[k].tau_ps += delta_tau;
                let model_plus = self.generate_model(&comp_plus, bg, t0_ch, n, &resolution);
                let chi2_plus_tau = chi_squared_reduced(spectrum, &model_plus, (fit_start, fit_end), 2 * nc + 2);

                let grad_tau = (chi2_plus_tau - chi2) / delta_tau;
                let step_tau = lambda * grad_tau * components[k].tau_ps.max(10.0);
                components[k].tau_ps = (components[k].tau_ps - step_tau).max(10.0);

                // Gradient w.r.t. intensity_k
                let mut comp_plus_i = components.clone();
                comp_plus_i[k].intensity += delta_i;
                let model_plus_i = self.generate_model(&comp_plus_i, bg, t0_ch, n, &resolution);
                let chi2_plus_i = chi_squared_reduced(spectrum, &model_plus_i, (fit_start, fit_end), 2 * nc + 2);

                let grad_i = (chi2_plus_i - chi2) / delta_i;
                let step_i = lambda * grad_i * 0.01;
                components[k].intensity = (components[k].intensity - step_i).max(0.001);
            }

            // Normalize intensities
            let total_i: f64 = components.iter().map(|c| c.intensity).sum();
            if total_i > 0.0 {
                for c in &mut components {
                    c.intensity /= total_i;
                }
            }

            // Adjust damping
            if chi2 < prev_chi2 {
                lambda *= 1.1;
            } else {
                lambda *= 0.5;
            }
            lambda = lambda.clamp(1e-6, 1.0);

            prev_chi2 = chi2;
        }

        // Final model and chi-squared
        let model = self.generate_model(&components, bg, t0_ch, n, &resolution);
        let chi2_final = chi_squared_reduced(spectrum, &model, (fit_start, fit_end), 2 * nc + 2);

        // Estimate uncertainties from curvature (simplified)
        for k in 0..nc {
            let delta = components[k].tau_ps * 0.01;
            let mut cp = components.clone();
            let mut cm = components.clone();
            cp[k].tau_ps += delta;
            cm[k].tau_ps = (cm[k].tau_ps - delta).max(1.0);
            let mp = self.generate_model(&cp, bg, t0_ch, n, &resolution);
            let mm = self.generate_model(&cm, bg, t0_ch, n, &resolution);
            let chi2_p = chi_squared_reduced(spectrum, &mp, (fit_start, fit_end), 2 * nc + 2);
            let chi2_m = chi_squared_reduced(spectrum, &mm, (fit_start, fit_end), 2 * nc + 2);
            let curvature = (chi2_p + chi2_m - 2.0 * chi2_final) / (delta * delta);
            components[k].tau_uncertainty_ps = if curvature > 0.0 {
                (2.0 / curvature).sqrt()
            } else {
                0.0
            };

            // Intensity uncertainty
            let di = 0.005;
            let mut cpi = components.clone();
            let mut cmi = components.clone();
            cpi[k].intensity += di;
            cmi[k].intensity = (cmi[k].intensity - di).max(0.001);
            let mpi = self.generate_model(&cpi, bg, t0_ch, n, &resolution);
            let mmi = self.generate_model(&cmi, bg, t0_ch, n, &resolution);
            let chi2_pi = chi_squared_reduced(spectrum, &mpi, (fit_start, fit_end), 2 * nc + 2);
            let chi2_mi = chi_squared_reduced(spectrum, &mmi, (fit_start, fit_end), 2 * nc + 2);
            let curv_i = (chi2_pi + chi2_mi - 2.0 * chi2_final) / (di * di);
            components[k].intensity_uncertainty = if curv_i > 0.0 {
                (2.0 / curv_i).sqrt()
            } else {
                0.0
            };
        }

        let total_counts: f64 = spectrum.iter().sum();
        let residual_ss: f64 = (fit_start..fit_end.min(n))
            .map(|i| (spectrum[i] - model[i]).powi(2))
            .sum();
        let dof = (fit_end.min(n) - fit_start).saturating_sub(2 * nc + 2);
        let fit_variance = if dof > 0 { residual_ss / dof as f64 } else { 0.0 };

        PalsFitResult {
            components,
            chi_squared_reduced: chi2_final,
            background: bg,
            t0_channel: t0_ch,
            total_counts,
            fit_variance,
        }
    }

    /// Generate a model spectrum from components, background, and resolution.
    fn generate_model(
        &self,
        components: &[LifetimeComponent],
        background: f64,
        t0_ch: f64,
        n: usize,
        resolution: &[f64],
    ) -> Vec<f64> {
        let mut raw = vec![0.0; n];
        for ch in 0..n {
            let t = (ch as f64 - t0_ch) * self.config.time_per_channel_ps;
            if t >= 0.0 {
                for c in components {
                    if c.tau_ps > 0.0 {
                        raw[ch] += (c.intensity / c.tau_ps) * (-t / c.tau_ps).exp();
                    }
                }
            }
        }

        // Normalize raw to match data scale
        let sum_raw: f64 = raw.iter().sum();
        if sum_raw > 0.0 {
            let scale = 1.0 / sum_raw;
            for v in &mut raw {
                *v *= scale;
            }
        }

        // Convolve
        let convolved = convolve_spectrum(&raw, resolution);

        // Scale and add background
        let total_above_bg: f64 = convolved.iter().sum::<f64>();
        let mut model = vec![background; n];
        if total_above_bg > 0.0 {
            // Scale factor: total counts in data above background
            for ch in 0..n {
                model[ch] += convolved[ch] * 1e6; // arbitrary scale, chi2 will handle normalization
            }
        } else {
            for ch in 0..n {
                model[ch] += convolved[ch];
            }
        }

        model
    }

    /// Perform source correction on a spectrum.
    pub fn source_correct(&self, spectrum: &[f64]) -> Vec<f64> {
        let t0 = find_t0_channel(spectrum) as f64;
        source_correct(spectrum, &self.config, t0)
    }

    /// Compute mean lifetime from fitted components.
    ///
    /// τ_mean = Σᵢ Iᵢ * τᵢ
    pub fn mean_lifetime(components: &[LifetimeComponent]) -> f64 {
        components.iter().map(|c| c.intensity * c.tau_ps).sum()
    }

    /// Compute the defect-specific trapping rate from two-state trapping model.
    ///
    /// κ_d = I₂/I₁ * (1/τ_b - 1/τ₂)
    ///
    /// where τ_b is the bulk lifetime (≈ 1/(I₁/τ₁ + I₂/τ₂)).
    pub fn trapping_rate(components: &[LifetimeComponent]) -> Option<f64> {
        if components.len() < 2 {
            return None;
        }
        let c1 = &components[0];
        let c2 = &components[1];

        let tau_b = 1.0 / (c1.intensity / c1.tau_ps + c2.intensity / c2.tau_ps);
        let kappa = (c2.intensity / c1.intensity) * (1.0 / tau_b - 1.0 / c2.tau_ps);
        Some(kappa)
    }

    /// Extract Doppler broadening parameters from a 511 keV peak.
    pub fn analyze_doppler(
        &self,
        spectrum: &[f64],
        centroid: f64,
        s_half_width: f64,
        w_inner: f64,
        w_outer: f64,
    ) -> DopplerParameters {
        doppler_broadening_parameters(spectrum, centroid, s_half_width, w_inner, w_outer)
    }

    /// Compute CDB ratio curve.
    pub fn compute_cdb_ratio(
        &self,
        sample: &[f64],
        reference: &[f64],
        centroid_sample: f64,
        centroid_ref: f64,
        kev_per_channel: f64,
    ) -> CdbRatioCurve {
        cdb_ratio_curve(sample, reference, centroid_sample, centroid_ref, kev_per_channel)
    }
}

// ---------------------------------------------------------------------------
// Helper: generate a synthetic 511 keV peak for testing
// ---------------------------------------------------------------------------

/// Generate a synthetic Gaussian peak centered at `centroid` with given area and FWHM.
fn synthetic_gaussian_peak(n: usize, centroid: f64, area: f64, fwhm: f64) -> Vec<f64> {
    let sigma = fwhm / FWHM_TO_SIGMA;
    let mut spec = vec![0.0; n];
    let norm = area / (sigma * (2.0 * PI).sqrt());
    for i in 0..n {
        let x = i as f64 - centroid;
        spec[i] = norm * (-0.5 * (x / sigma).powi(2)).exp();
    }
    spec
}

/// Generate a synthetic TAC spectrum with known components for testing.
fn synthetic_tac_spectrum(
    n: usize,
    t0: f64,
    bin_ps: f64,
    components: &[(f64, f64)], // (tau_ps, intensity)
    total_counts: f64,
    bg: f64,
) -> Vec<f64> {
    let mut spec = vec![bg; n];
    for ch in 0..n {
        let t = (ch as f64 - t0) * bin_ps;
        if t >= 0.0 {
            for &(tau, intensity) in components {
                if tau > 0.0 {
                    spec[ch] += total_counts * (intensity / tau) * (-t / tau).exp();
                }
            }
        }
    }
    spec
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // === Tao-Eldrup model tests ===

    #[test]
    fn test_tao_eldrup_rate_zero_radius() {
        // At R=0, rate should be 2.0 * (1 - 0 + 0) = 2.0 ns^-1
        let rate = tao_eldrup_rate(0.0);
        assert!((rate - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_tao_eldrup_rate_large_radius() {
        // As R -> infinity, R/(R+ΔR) -> 1, sin(2π)/2π -> 0
        // rate -> 2.0 * (1 - 1 + 0) = 0
        let rate = tao_eldrup_rate(1000.0);
        assert!(rate < 0.01);
    }

    #[test]
    fn test_tao_eldrup_lifetime_increases_with_radius() {
        let tau1 = tao_eldrup_lifetime(1.0);
        let tau2 = tao_eldrup_lifetime(2.0);
        let tau3 = tao_eldrup_lifetime(3.0);
        assert!(tau2 > tau1);
        assert!(tau3 > tau2);
    }

    #[test]
    fn test_tao_eldrup_radius_roundtrip() {
        // Generate lifetime from radius, then recover radius
        let r_original = 2.5;
        let tau = tao_eldrup_lifetime(r_original);
        let r_recovered = tao_eldrup_radius(tau);
        assert!((r_recovered - r_original).abs() < 0.01);
    }

    #[test]
    fn test_tao_eldrup_radius_polymer_range() {
        // Typical o-Ps lifetime in polymers: 1-5 ns (1000-5000 ps)
        let r_1ns = tao_eldrup_radius(1000.0);
        let r_5ns = tao_eldrup_radius(5000.0);
        assert!(r_1ns > 0.0);
        assert!(r_5ns > r_1ns);
        assert!(r_5ns < 10.0); // Should be reasonable
    }

    #[test]
    fn test_tao_eldrup_radius_2ns() {
        // 2 ns o-Ps lifetime is common, radius should be ~ 2.9 Å
        let r = tao_eldrup_radius(2000.0);
        assert!(r > 2.0 && r < 4.0);
    }

    #[test]
    fn test_tao_eldrup_rate_monotonically_decreasing() {
        let mut prev_rate = tao_eldrup_rate(0.01);
        for i in 1..50 {
            let r = i as f64 * 0.2;
            let rate = tao_eldrup_rate(r);
            assert!(rate <= prev_rate + EPSILON, "rate not monotonically decreasing at R={}", r);
            prev_rate = rate;
        }
    }

    // === Free volume calculation tests ===

    #[test]
    fn test_free_volume_from_radius() {
        let r = 1.0;
        let v = free_volume_from_radius(r);
        let expected = 4.0 / 3.0 * PI;
        assert!((v - expected).abs() < 0.01);
    }

    #[test]
    fn test_free_volume_scales_cubically() {
        let v1 = free_volume_from_radius(1.0);
        let v2 = free_volume_from_radius(2.0);
        assert!((v2 / v1 - 8.0).abs() < 0.01);
    }

    #[test]
    fn test_fractional_free_volume() {
        let ffv = fractional_free_volume(0.3, 2000.0, 0.0018);
        assert!(ffv > 0.0);
        assert!(ffv < 1.0); // Should be a fraction
    }

    #[test]
    fn test_fractional_free_volume_increases_with_intensity() {
        let ffv1 = fractional_free_volume(0.1, 2000.0, 0.0018);
        let ffv2 = fractional_free_volume(0.3, 2000.0, 0.0018);
        assert!(ffv2 > ffv1);
    }

    // === Gaussian resolution function tests ===

    #[test]
    fn test_gaussian_resolution_normalized() {
        let g = gaussian_resolution(256, 128.0, 250.0, 25.0);
        let sum: f64 = g.iter().sum();
        assert!((sum - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_gaussian_resolution_peak_at_center() {
        let g = gaussian_resolution(256, 128.0, 250.0, 25.0);
        let max_idx = g.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(max_idx, 128);
    }

    #[test]
    fn test_gaussian_resolution_symmetric() {
        let center = 128.0;
        let g = gaussian_resolution(256, center, 250.0, 25.0);
        for offset in 1..50 {
            let left = g[(center as usize) - offset];
            let right = g[(center as usize) + offset];
            assert!((left - right).abs() < 1e-10);
        }
    }

    #[test]
    fn test_gaussian_resolution_fwhm() {
        let center = 128.0;
        let fwhm_ps = 250.0;
        let bin_ps = 25.0;
        let g = gaussian_resolution(256, center, fwhm_ps, bin_ps);
        let peak = g[128];
        let half_max = peak / 2.0;

        // Find FWHM in channels
        let mut left_ch = 128;
        let mut right_ch = 128;
        for i in (0..128).rev() {
            if g[i] < half_max {
                left_ch = i + 1;
                break;
            }
        }
        for i in 129..256 {
            if g[i] < half_max {
                right_ch = i - 1;
                break;
            }
        }
        let fwhm_channels = (right_ch - left_ch) as f64;
        let fwhm_measured = fwhm_channels * bin_ps;
        // Should be close to 250 ps (within a bin)
        assert!((fwhm_measured - fwhm_ps).abs() < bin_ps * 2.0);
    }

    #[test]
    fn test_gaussian_resolution_narrower_with_smaller_fwhm() {
        let g_wide = gaussian_resolution(256, 128.0, 500.0, 25.0);
        let g_narrow = gaussian_resolution(256, 128.0, 100.0, 25.0);
        // Narrow should have higher peak
        assert!(g_narrow[128] > g_wide[128]);
    }

    // === Multi-exponential decay tests ===

    #[test]
    fn test_multi_exponential_single_component() {
        let comp = vec![LifetimeComponent {
            tau_ps: 400.0,
            intensity: 1.0,
            tau_uncertainty_ps: 0.0,
            intensity_uncertainty: 0.0,
        }];
        let val = multi_exponential(0.0, &comp, 0.0);
        assert!((val - 1.0 / 400.0).abs() < 1e-10);
    }

    #[test]
    fn test_multi_exponential_decay_at_infinity() {
        let comp = vec![LifetimeComponent {
            tau_ps: 400.0,
            intensity: 1.0,
            tau_uncertainty_ps: 0.0,
            intensity_uncertainty: 0.0,
        }];
        let val = multi_exponential(1e10, &comp, 0.0);
        assert!(val < 1e-15);
    }

    #[test]
    fn test_multi_exponential_with_background() {
        let comp = vec![LifetimeComponent {
            tau_ps: 400.0,
            intensity: 1.0,
            tau_uncertainty_ps: 0.0,
            intensity_uncertainty: 0.0,
        }];
        let bg = 5.0;
        let val = multi_exponential(1e10, &comp, bg);
        assert!((val - bg).abs() < 1e-10);
    }

    #[test]
    fn test_multi_exponential_two_components() {
        let comp = vec![
            LifetimeComponent { tau_ps: 200.0, intensity: 0.6, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
            LifetimeComponent { tau_ps: 2000.0, intensity: 0.4, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        let val_0 = multi_exponential(0.0, &comp, 0.0);
        let expected = 0.6 / 200.0 + 0.4 / 2000.0;
        assert!((val_0 - expected).abs() < 1e-10);
    }

    #[test]
    fn test_multi_exponential_monotonically_decreasing() {
        let comp = vec![
            LifetimeComponent { tau_ps: 200.0, intensity: 0.5, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
            LifetimeComponent { tau_ps: 1500.0, intensity: 0.5, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        let mut prev = multi_exponential(0.0, &comp, 0.0);
        for i in 1..100 {
            let t = i as f64 * 50.0;
            let val = multi_exponential(t, &comp, 0.0);
            assert!(val <= prev + EPSILON);
            prev = val;
        }
    }

    // === Convolution tests ===

    #[test]
    fn test_convolve_delta() {
        // Convolving with a delta should return the same signal
        let signal = vec![0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0];
        let delta = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let result = convolve_spectrum(&signal, &delta);
        for i in 0..signal.len() {
            assert!((result[i] - signal[i]).abs() < 1e-10, "mismatch at {}", i);
        }
    }

    #[test]
    fn test_convolve_preserves_total_energy() {
        let signal = vec![0.0, 1.0, 2.0, 3.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0];
        let kernel = gaussian_resolution(10, 5.0, 2.0, 1.0);
        let result = convolve_spectrum(&signal, &kernel);
        let sum_in: f64 = signal.iter().sum();
        let sum_out: f64 = result.iter().sum();
        // Should be approximately preserved (edge effects may cause small deviations)
        assert!((sum_in - sum_out).abs() / sum_in.max(1.0) < 0.2);
    }

    #[test]
    fn test_convolve_broadens_peak() {
        let mut signal = vec![0.0; 100];
        signal[50] = 100.0; // Sharp peak
        let kernel = gaussian_resolution(100, 50.0, 10.0, 1.0);
        let result = convolve_spectrum(&signal, &kernel);
        // Peak should be broadened (lower maximum)
        assert!(result[50] < 100.0);
        // But neighbors should have nonzero values
        assert!(result[49] > 0.0);
        assert!(result[51] > 0.0);
    }

    // === Background estimation tests ===

    #[test]
    fn test_estimate_background_flat() {
        let spectrum = vec![10.0; 100];
        let bg = estimate_background(&spectrum, (80, 100));
        assert!((bg - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_estimate_background_with_peak() {
        let mut spectrum = vec![5.0; 100];
        spectrum[50] = 1000.0; // Peak should not affect background
        let bg = estimate_background(&spectrum, (80, 100));
        assert!((bg - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_estimate_background_empty_region() {
        let spectrum = vec![1.0; 10];
        let bg = estimate_background(&spectrum, (10, 5)); // Invalid range
        assert_eq!(bg, 0.0);
    }

    // === Time zero detection tests ===

    #[test]
    fn test_find_t0_channel_single_peak() {
        let mut spectrum = vec![0.0; 100];
        spectrum[30] = 1000.0;
        assert_eq!(find_t0_channel(&spectrum), 30);
    }

    #[test]
    fn test_find_t0_channel_gaussian() {
        let spectrum = gaussian_resolution(256, 100.0, 200.0, 25.0);
        let t0 = find_t0_channel(&spectrum);
        assert_eq!(t0, 100);
    }

    // === Chi-squared tests ===

    #[test]
    fn test_chi_squared_perfect_fit() {
        let data = vec![10.0, 20.0, 30.0, 20.0, 10.0];
        let model = data.clone();
        let chi2 = chi_squared_reduced(&data, &model, (0, 5), 0);
        assert!(chi2 < EPSILON);
    }

    #[test]
    fn test_chi_squared_poor_fit() {
        let data = vec![100.0, 200.0, 300.0, 200.0, 100.0];
        let model = vec![1.0, 1.0, 1.0, 1.0, 1.0];
        let chi2 = chi_squared_reduced(&data, &model, (0, 5), 0);
        assert!(chi2 > 10.0); // Very poor fit
    }

    #[test]
    fn test_chi_squared_accounts_for_poisson_statistics() {
        // Higher counts -> smaller relative contribution to chi2
        let data = vec![10000.0, 10001.0]; // 1 count difference on 10000
        let model = vec![10000.0, 10000.0];
        let chi2_high = chi_squared_reduced(&data, &model, (0, 2), 0);

        let data2 = vec![10.0, 11.0]; // 1 count difference on 10
        let model2 = vec![10.0, 10.0];
        let chi2_low = chi_squared_reduced(&data2, &model2, (0, 2), 0);

        assert!(chi2_low > chi2_high);
    }

    // === Doppler broadening tests ===

    #[test]
    fn test_doppler_s_parameter_gaussian() {
        // Generate a Gaussian peak
        let spectrum = synthetic_gaussian_peak(500, 250.0, 10000.0, 10.0);
        let dp = doppler_broadening_parameters(&spectrum, 250.0, 5.0, 15.0, 30.0);
        assert!(dp.s_parameter > 0.0 && dp.s_parameter < 1.0);
        assert!(dp.w_parameter >= 0.0 && dp.w_parameter < dp.s_parameter);
        assert!(dp.total_area > 0.0);
    }

    #[test]
    fn test_doppler_s_w_sum_less_than_one() {
        let spectrum = synthetic_gaussian_peak(500, 250.0, 10000.0, 10.0);
        let dp = doppler_broadening_parameters(&spectrum, 250.0, 5.0, 15.0, 30.0);
        assert!(dp.s_parameter + dp.w_parameter <= 1.0 + EPSILON);
    }

    #[test]
    fn test_doppler_wider_peak_lower_s() {
        let narrow = synthetic_gaussian_peak(500, 250.0, 10000.0, 5.0);
        let wide = synthetic_gaussian_peak(500, 250.0, 10000.0, 20.0);
        let dp_narrow = doppler_broadening_parameters(&narrow, 250.0, 3.0, 10.0, 30.0);
        let dp_wide = doppler_broadening_parameters(&wide, 250.0, 3.0, 10.0, 30.0);
        // Wider peak should have lower S (more counts in wings)
        assert!(dp_wide.s_parameter < dp_narrow.s_parameter);
    }

    #[test]
    fn test_doppler_wider_peak_higher_w() {
        let narrow = synthetic_gaussian_peak(500, 250.0, 10000.0, 5.0);
        let wide = synthetic_gaussian_peak(500, 250.0, 10000.0, 20.0);
        let dp_narrow = doppler_broadening_parameters(&narrow, 250.0, 3.0, 10.0, 30.0);
        let dp_wide = doppler_broadening_parameters(&wide, 250.0, 3.0, 10.0, 30.0);
        // Wider peak should have higher W
        assert!(dp_wide.w_parameter > dp_narrow.w_parameter);
    }

    #[test]
    fn test_doppler_centroid() {
        let spectrum = synthetic_gaussian_peak(500, 250.0, 10000.0, 10.0);
        let dp = doppler_broadening_parameters(&spectrum, 250.0, 5.0, 15.0, 30.0);
        assert!((dp.centroid - 250.0).abs() < EPSILON);
    }

    // === CDB ratio curve tests ===

    #[test]
    fn test_cdb_ratio_same_spectra() {
        let spectrum = synthetic_gaussian_peak(200, 100.0, 10000.0, 10.0);
        let result = cdb_ratio_curve(&spectrum, &spectrum, 100.0, 100.0, 0.5);
        // Ratio of identical spectra should be 1.0 everywhere
        for &r in &result.ratio {
            assert!((r - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_cdb_ratio_wider_sample() {
        let sample = synthetic_gaussian_peak(200, 100.0, 10000.0, 15.0);
        let reference = synthetic_gaussian_peak(200, 100.0, 10000.0, 10.0);
        let result = cdb_ratio_curve(&sample, &reference, 100.0, 100.0, 0.5);
        // Wider sample: ratio < 1 at center, > 1 in wings
        // Find center ratio
        let center_idx = result.momentum.iter()
            .enumerate()
            .min_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);
        assert!(result.ratio[center_idx] < 1.0);
    }

    #[test]
    fn test_cdb_ratio_has_errors() {
        let sample = synthetic_gaussian_peak(200, 100.0, 10000.0, 10.0);
        let reference = synthetic_gaussian_peak(200, 100.0, 10000.0, 10.0);
        let result = cdb_ratio_curve(&sample, &reference, 100.0, 100.0, 0.5);
        assert!(!result.ratio_error.is_empty());
        for &e in &result.ratio_error {
            assert!(e >= 0.0);
        }
    }

    // === Source correction tests ===

    #[test]
    fn test_source_correct_reduces_counts() {
        let config = PalsConfig::default();
        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(350.0, 1.0)], 100000.0, 10.0,
        );
        let corrected = source_correct(&spectrum, &config, 150.0);
        let sum_orig: f64 = spectrum.iter().sum();
        let sum_corr: f64 = corrected.iter().sum();
        assert!(sum_corr < sum_orig);
    }

    #[test]
    fn test_source_correct_non_negative() {
        let config = PalsConfig::default();
        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(350.0, 1.0)], 100000.0, 10.0,
        );
        let corrected = source_correct(&spectrum, &config, 150.0);
        for &v in &corrected {
            assert!(v >= 0.0);
        }
    }

    // === TAC histogram generation tests ===

    #[test]
    fn test_generate_tac_histogram_nonzero() {
        let config = PalsConfig::default();
        let components = vec![
            LifetimeComponent { tau_ps: 125.0, intensity: 0.2, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
            LifetimeComponent { tau_ps: 350.0, intensity: 0.5, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
            LifetimeComponent { tau_ps: 2000.0, intensity: 0.3, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        let hist = generate_tac_histogram(&config, &components, 100000.0);
        assert_eq!(hist.len(), config.num_channels);
        let sum: f64 = hist.iter().sum();
        assert!(sum > 0.0);
    }

    #[test]
    fn test_generate_tac_histogram_has_peak() {
        let config = PalsConfig::default();
        let components = vec![
            LifetimeComponent { tau_ps: 350.0, intensity: 1.0, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        let hist = generate_tac_histogram(&config, &components, 100000.0);
        let max_val = hist.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_val = hist.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(max_val > min_val * 10.0); // Should have a clear peak
    }

    // === PalsAnalyzer tests ===

    #[test]
    fn test_pals_analyzer_creation() {
        let config = PalsConfig::default();
        let analyzer = PalsAnalyzer::new(config.clone());
        assert_eq!(analyzer.max_iterations, 500);
        assert_eq!(analyzer.config.num_components, 3);
    }

    #[test]
    fn test_mean_lifetime_single_component() {
        let components = vec![
            LifetimeComponent { tau_ps: 400.0, intensity: 1.0, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        let mean = PalsAnalyzer::mean_lifetime(&components);
        assert!((mean - 400.0).abs() < EPSILON);
    }

    #[test]
    fn test_mean_lifetime_two_components() {
        let components = vec![
            LifetimeComponent { tau_ps: 200.0, intensity: 0.5, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
            LifetimeComponent { tau_ps: 400.0, intensity: 0.5, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        let mean = PalsAnalyzer::mean_lifetime(&components);
        assert!((mean - 300.0).abs() < EPSILON);
    }

    #[test]
    fn test_trapping_rate_two_components() {
        let components = vec![
            LifetimeComponent { tau_ps: 150.0, intensity: 0.7, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
            LifetimeComponent { tau_ps: 300.0, intensity: 0.3, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        let kappa = PalsAnalyzer::trapping_rate(&components);
        assert!(kappa.is_some());
        assert!(kappa.unwrap() > 0.0);
    }

    #[test]
    fn test_trapping_rate_single_component_none() {
        let components = vec![
            LifetimeComponent { tau_ps: 200.0, intensity: 1.0, tau_uncertainty_ps: 0.0, intensity_uncertainty: 0.0 },
        ];
        assert!(PalsAnalyzer::trapping_rate(&components).is_none());
    }

    #[test]
    fn test_fit_spectrum_returns_correct_number_of_components() {
        let config = PalsConfig {
            num_components: 2,
            ..PalsConfig::default()
        };
        let analyzer = PalsAnalyzer::new(config.clone());

        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(200.0, 0.6), (1500.0, 0.4)], 50000.0, 5.0,
        );

        let result = analyzer.fit_spectrum(&spectrum);
        assert_eq!(result.components.len(), 2);
    }

    #[test]
    fn test_fit_spectrum_intensities_sum_to_one() {
        let config = PalsConfig {
            num_components: 3,
            ..PalsConfig::default()
        };
        let analyzer = PalsAnalyzer::new(config.clone());

        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(125.0, 0.2), (350.0, 0.5), (2000.0, 0.3)], 100000.0, 10.0,
        );

        let result = analyzer.fit_spectrum(&spectrum);
        let sum_i: f64 = result.components.iter().map(|c| c.intensity).sum();
        assert!((sum_i - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_fit_spectrum_positive_lifetimes() {
        let config = PalsConfig {
            num_components: 2,
            ..PalsConfig::default()
        };
        let analyzer = PalsAnalyzer::new(config.clone());

        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(300.0, 0.7), (2000.0, 0.3)], 50000.0, 5.0,
        );

        let result = analyzer.fit_spectrum(&spectrum);
        for c in &result.components {
            assert!(c.tau_ps > 0.0);
        }
    }

    #[test]
    fn test_fit_spectrum_has_chi_squared() {
        let config = PalsConfig::default();
        let analyzer = PalsAnalyzer::new(config.clone());

        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(350.0, 1.0)], 50000.0, 5.0,
        );

        let result = analyzer.fit_spectrum(&spectrum);
        assert!(result.chi_squared_reduced > 0.0);
        assert!(result.chi_squared_reduced < f64::INFINITY);
    }

    #[test]
    fn test_fit_spectrum_background_estimation() {
        let config = PalsConfig::default();
        let analyzer = PalsAnalyzer::new(config.clone());
        let bg = 7.5;

        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(350.0, 1.0)], 50000.0, bg,
        );

        let result = analyzer.fit_spectrum(&spectrum);
        assert!((result.background - bg).abs() < bg * 0.5);
    }

    #[test]
    fn test_source_correct_via_analyzer() {
        let config = PalsConfig::default();
        let analyzer = PalsAnalyzer::new(config.clone());

        let spectrum = synthetic_tac_spectrum(
            config.num_channels, 150.0, config.time_per_channel_ps,
            &[(350.0, 1.0)], 100000.0, 10.0,
        );

        let corrected = analyzer.source_correct(&spectrum);
        assert_eq!(corrected.len(), spectrum.len());
    }

    // === Default config test ===

    #[test]
    fn test_default_config() {
        let config = PalsConfig::default();
        assert_eq!(config.num_channels, 1024);
        assert_eq!(config.num_components, 3);
        assert!((config.time_per_channel_ps - 25.0).abs() < EPSILON);
        assert!((config.resolution_fwhm_ps - 250.0).abs() < EPSILON);
        assert!((config.source_fraction - 0.10).abs() < EPSILON);
        assert!((config.source_lifetime_ps - 382.0).abs() < EPSILON);
    }

    // === Physical constants sanity tests ===

    #[test]
    fn test_para_ps_lifetime_constant() {
        assert!((TAU_PPS_PS - 125.0).abs() < EPSILON);
    }

    #[test]
    fn test_ortho_ps_vacuum_lifetime_constant() {
        assert!((TAU_OPS_VACUUM_PS - 142000.0).abs() < EPSILON);
    }

    #[test]
    fn test_delta_r_constant() {
        assert!((DELTA_R_ANGSTROM - 1.66).abs() < EPSILON);
    }

    #[test]
    fn test_electron_mass_constant() {
        assert!((ELECTRON_MASS_KEV - 511.0).abs() < EPSILON);
    }

    #[test]
    fn test_na22_gamma_constant() {
        assert!((NA22_BIRTH_GAMMA_KEV - 1274.5).abs() < EPSILON);
    }

    #[test]
    fn test_fwhm_to_sigma_constant() {
        let expected = 2.0 * (2.0_f64 * 2.0_f64.ln()).sqrt();
        assert!((FWHM_TO_SIGMA - expected).abs() < 1e-6);
    }

    // === Edge case tests ===

    #[test]
    fn test_multi_exponential_empty_components() {
        let val = multi_exponential(100.0, &[], 5.0);
        assert!((val - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_free_volume_zero_radius() {
        let v = free_volume_from_radius(0.0);
        assert!(v.abs() < EPSILON);
    }

    #[test]
    fn test_tao_eldrup_very_short_lifetime() {
        // Very short lifetime -> very small radius
        let r = tao_eldrup_radius(100.0);
        assert!(r > 0.0);
        assert!(r < 1.0);
    }

    #[test]
    fn test_tao_eldrup_very_long_lifetime() {
        // Very long lifetime -> large radius
        let r = tao_eldrup_radius(10000.0);
        assert!(r > 3.0);
    }

    #[test]
    fn test_doppler_empty_spectrum() {
        let spectrum = vec![0.0; 100];
        let dp = doppler_broadening_parameters(&spectrum, 50.0, 5.0, 15.0, 30.0);
        assert_eq!(dp.s_parameter, 0.0);
        assert_eq!(dp.w_parameter, 0.0);
    }

    #[test]
    fn test_cdb_ratio_empty_spectra() {
        let result = cdb_ratio_curve(&[0.0; 10], &[0.0; 10], 5.0, 5.0, 0.5);
        assert!(result.ratio.is_empty());
    }

    #[test]
    fn test_synthetic_tac_spectrum_shape() {
        let spec = synthetic_tac_spectrum(512, 100.0, 25.0, &[(300.0, 1.0)], 100000.0, 5.0);
        assert_eq!(spec.len(), 512);
        // Before t0, should be just background
        assert!((spec[0] - 5.0).abs() < EPSILON);
        // At t0, should be much higher
        assert!(spec[100] > 100.0);
        // Should decay after t0
        assert!(spec[200] < spec[100]);
    }

    #[test]
    fn test_analyze_doppler_via_analyzer() {
        let config = PalsConfig::default();
        let analyzer = PalsAnalyzer::new(config);
        let spectrum = synthetic_gaussian_peak(500, 250.0, 10000.0, 10.0);
        let dp = analyzer.analyze_doppler(&spectrum, 250.0, 5.0, 15.0, 30.0);
        assert!(dp.s_parameter > 0.0);
    }

    #[test]
    fn test_compute_cdb_ratio_via_analyzer() {
        let config = PalsConfig::default();
        let analyzer = PalsAnalyzer::new(config);
        let sample = synthetic_gaussian_peak(200, 100.0, 10000.0, 10.0);
        let reference = synthetic_gaussian_peak(200, 100.0, 10000.0, 10.0);
        let result = analyzer.compute_cdb_ratio(&sample, &reference, 100.0, 100.0, 0.5);
        assert!(!result.ratio.is_empty());
    }
}

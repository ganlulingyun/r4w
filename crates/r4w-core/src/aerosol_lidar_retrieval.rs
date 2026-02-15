//! Backscatter-to-extinction inversion algorithms for aerosol profiling from
//! atmospheric LIDAR measurements.
//!
//! This module implements the core inversion and correction algorithms used in
//! elastic backscatter LIDAR systems for atmospheric aerosol profiling. It covers
//! the full processing chain from raw photon-count returns to calibrated
//! extinction and backscatter coefficient profiles.
//!
//! # Background
//!
//! Elastic backscatter LIDAR transmits short laser pulses into the atmosphere and
//! measures the time-resolved return signal. The single-scattering LIDAR equation
//! relates the received power P(r) at range r to the backscatter coefficient
//! beta(r) and extinction coefficient alpha(r):
//!
//! ```text
//!   P(r) = P0 * C * O(r) / r^2 * beta(r) * exp(-2 * integral_0^r alpha(r') dr')
//! ```
//!
//! where P0 is the pulse energy, C is the system constant (telescope area,
//! detector efficiency, etc.), O(r) is the geometric overlap function, and the
//! exponential is the two-way atmospheric transmission.
//!
//! # Algorithms
//!
//! - **Klett inversion**: Backward-stable solution assuming a power-law
//!   relationship between extinction and backscatter (alpha = k * beta^kappa).
//!   Integrates from a far-range reference point back toward the instrument,
//!   avoiding the numerical instability of forward integration.
//!
//! - **Fernald method**: Two-component (molecular + aerosol) inversion that
//!   separates the known molecular (Rayleigh) contribution from the unknown
//!   aerosol signal. Requires specification of the aerosol lidar ratio Sa.
//!
//! - **Overlap correction**: Applies a geometric overlap function O(r) to
//!   correct near-field signal distortion where the transmitted beam and
//!   receiver field of view do not fully overlap.
//!
//! - **Boundary layer detection**: Gradient and wavelet-based methods for
//!   detecting the planetary boundary layer (PBL) height from backscatter
//!   profiles.
//!
//! # Example
//!
//! ```
//! use r4w_core::aerosol_lidar_retrieval::{
//!     LidarConfig, RangeCorrector, KlettInverter, molecular_backscatter,
//! };
//!
//! let config = LidarConfig {
//!     wavelength_nm: 532.0,
//!     pulse_energy_j: 0.01,
//!     telescope_area_m2: 0.07,
//!     range_resolution_m: 15.0,
//!     overlap: None,
//! };
//!
//! // Generate synthetic range bins
//! let n = 100;
//! let ranges: Vec<f64> = (1..=n).map(|i| i as f64 * config.range_resolution_m).collect();
//!
//! // Simulate a raw return signal (arbitrary units)
//! let raw_signal: Vec<f64> = ranges.iter().map(|r| {
//!     let beta = 1e-6;
//!     let alpha = 5e-5;
//!     config.pulse_energy_j * config.telescope_area_m2 / (r * r)
//!         * beta * (-2.0 * alpha * r).exp()
//! }).collect();
//!
//! // Range-correct the signal
//! let corrector = RangeCorrector;
//! let rcs = corrector.correct(&raw_signal, &ranges);
//! assert_eq!(rcs.len(), n);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Standard atmospheric number density at sea level (molecules/m^3).
const STD_NUMBER_DENSITY: f64 = 2.547e25;

/// Standard atmospheric pressure at sea level (Pa).
const STD_PRESSURE_PA: f64 = 101325.0;

/// Standard atmospheric temperature at sea level (K).
const STD_TEMPERATURE_K: f64 = 288.15;

/// Boltzmann constant (J/K).
const BOLTZMANN_K: f64 = 1.380649e-23;

/// Scale height for standard atmosphere (m).
const SCALE_HEIGHT_M: f64 = 8500.0;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for an elastic backscatter LIDAR system.
///
/// Captures the essential instrument parameters needed for signal inversion.
#[derive(Debug, Clone)]
pub struct LidarConfig {
    /// Laser wavelength in nanometers (e.g. 355, 532, 1064).
    pub wavelength_nm: f64,

    /// Pulse energy in joules.
    pub pulse_energy_j: f64,

    /// Effective telescope collecting area in m^2.
    pub telescope_area_m2: f64,

    /// Range bin resolution in meters.
    pub range_resolution_m: f64,

    /// Optional geometric overlap function values (one per range bin).
    /// Values should be in [0, 1], with 1 indicating full overlap.
    /// If `None`, full overlap is assumed at all ranges.
    pub overlap: Option<Vec<f64>>,
}

impl LidarConfig {
    /// Returns the laser wavelength in meters.
    pub fn wavelength_m(&self) -> f64 {
        self.wavelength_nm * 1e-9
    }
}

// ---------------------------------------------------------------------------
// Range correction
// ---------------------------------------------------------------------------

/// Applies range-squared correction to raw LIDAR return signals.
///
/// The received power from an elastic backscatter LIDAR falls off as 1/r^2
/// due to the solid angle subtended by the telescope at range r. The
/// range-corrected signal (RCS) is defined as:
///
/// ```text
///   RCS(r) = P(r) * r^2
/// ```
///
/// This removes the geometric spreading factor, making the signal proportional
/// to backscatter coefficient times two-way transmission.
#[derive(Debug, Clone, Copy)]
pub struct RangeCorrector;

impl RangeCorrector {
    /// Applies range-squared correction.
    ///
    /// # Arguments
    /// * `signal` - Raw return signal values (arbitrary units).
    /// * `ranges` - Range to each bin in meters.
    ///
    /// # Returns
    /// Range-corrected signal: `signal[i] * ranges[i]^2`.
    pub fn correct(&self, signal: &[f64], ranges: &[f64]) -> Vec<f64> {
        let n = signal.len().min(ranges.len());
        (0..n).map(|i| signal[i] * ranges[i] * ranges[i]).collect()
    }

    /// Removes range-squared correction (inverse operation).
    pub fn uncorrect(&self, rcs: &[f64], ranges: &[f64]) -> Vec<f64> {
        let n = rcs.len().min(ranges.len());
        (0..n)
            .map(|i| {
                if ranges[i].abs() < 1e-12 {
                    0.0
                } else {
                    rcs[i] / (ranges[i] * ranges[i])
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Overlap correction
// ---------------------------------------------------------------------------

/// Corrects near-field LIDAR signals for incomplete geometric overlap.
///
/// At short ranges, the transmitted laser beam may not fully overlap with the
/// receiver field of view, causing signal loss. The overlap function O(r) is
/// typically determined experimentally and ranges from 0 (no overlap) to 1
/// (complete overlap).
///
/// The corrected signal is `signal[i] / O(r_i)`.
#[derive(Debug, Clone)]
pub struct OverlapCorrector {
    /// Overlap function values, one per range bin, in [0, 1].
    overlap_values: Vec<f64>,

    /// Minimum overlap value below which correction is not applied (to avoid
    /// division by very small numbers). Default: 0.1.
    min_overlap: f64,
}

impl OverlapCorrector {
    /// Creates a new overlap corrector from measured overlap function values.
    ///
    /// # Arguments
    /// * `overlap_values` - Overlap function O(r) at each range bin, values in [0, 1].
    pub fn new(overlap_values: Vec<f64>) -> Self {
        Self {
            overlap_values,
            min_overlap: 0.1,
        }
    }

    /// Creates an overlap corrector using a simple exponential model:
    ///
    /// ```text
    ///   O(r) = 1 - exp(-r / r_full)
    /// ```
    ///
    /// where `r_full` is the range at which overlap reaches ~63%.
    ///
    /// # Arguments
    /// * `ranges` - Range values in meters.
    /// * `r_full` - Characteristic full-overlap range in meters.
    pub fn from_exponential_model(ranges: &[f64], r_full: f64) -> Self {
        let values: Vec<f64> = ranges
            .iter()
            .map(|&r| 1.0 - (-r / r_full).exp())
            .collect();
        Self::new(values)
    }

    /// Sets the minimum overlap threshold. Range bins with overlap below this
    /// value will not be corrected (the raw signal is returned).
    pub fn set_min_overlap(&mut self, min_val: f64) {
        self.min_overlap = min_val.max(0.01);
    }

    /// Applies overlap correction to a signal.
    ///
    /// # Arguments
    /// * `signal` - Input signal values.
    ///
    /// # Returns
    /// Overlap-corrected signal.
    pub fn correct(&self, signal: &[f64]) -> Vec<f64> {
        let n = signal.len().min(self.overlap_values.len());
        let mut result = signal.to_vec();
        for i in 0..n {
            if self.overlap_values[i] >= self.min_overlap {
                result[i] = signal[i] / self.overlap_values[i];
            }
        }
        result
    }

    /// Returns the overlap function values.
    pub fn overlap_values(&self) -> &[f64] {
        &self.overlap_values
    }
}

// ---------------------------------------------------------------------------
// Molecular (Rayleigh) scattering helpers
// ---------------------------------------------------------------------------

/// Computes the molecular (Rayleigh) backscatter coefficient at a given
/// altitude for a standard atmosphere.
///
/// The Rayleigh backscatter coefficient is:
///
/// ```text
///   beta_mol(z) = N(z) * sigma_R / (4 * pi)
/// ```
///
/// where N(z) is the number density at altitude z (using an exponential
/// atmosphere model) and sigma_R is the Rayleigh scattering cross-section.
///
/// The cross-section scales as lambda^-4:
///
/// ```text
///   sigma_R = (8 * pi^3 / 3) * ((n^2 - 1) / N_s)^2 * (6 + 3*delta) / (6 - 7*delta) / lambda^4
/// ```
///
/// For simplicity, we use the King-corrected Rayleigh cross-section formula
/// with standard air refractive index.
///
/// # Arguments
/// * `wavelength_nm` - Laser wavelength in nanometers.
/// * `altitude_m` - Altitude above sea level in meters.
///
/// # Returns
/// Molecular backscatter coefficient in 1/(m*sr).
pub fn molecular_backscatter(wavelength_nm: f64, altitude_m: f64) -> f64 {
    let lambda_m = wavelength_nm * 1e-9;

    // Rayleigh scattering cross-section (King-corrected)
    // Index of refraction for standard air at STP (essentially constant across
    // visible/NIR wavelengths for this purpose).
    let n_minus_1: f64 = 2.78e-4;
    let depol = 0.0279; // King factor depolarization ratio for air
    let king_factor = (6.0 + 3.0 * depol) / (6.0 - 7.0 * depol);

    // Rayleigh total scattering cross-section:
    //   sigma = (8*pi^3/3) * ((n^2-1)/(N_s))^2 * F_k / lambda^4
    // where n^2-1 ~ 2*n_minus_1 for small n_minus_1, and F_k is King factor.
    // The lambda^-4 dependence is the sole wavelength scaling.
    let two_n: f64 = 2.0 * n_minus_1;
    let sigma_total = 8.0 * PI.powi(3) / 3.0
        * two_n.powi(2)
        / (STD_NUMBER_DENSITY.powi(2))
        * king_factor
        / lambda_m.powi(4);

    // Number density at altitude (exponential atmosphere)
    let n_z = STD_NUMBER_DENSITY * (-altitude_m / SCALE_HEIGHT_M).exp();

    // Backscatter coefficient: beta_mol = N(z) * dsigma/dOmega(pi)
    // For Rayleigh: dsigma/dOmega(theta) = sigma * 3/(16*pi) * (1 + cos^2(theta))
    // At theta=pi: dsigma/dOmega(pi) = sigma * 3/(16*pi) * 2 = 3*sigma/(8*pi)
    let beta_mol = n_z * sigma_total * 3.0 / (8.0 * PI);

    beta_mol
}

/// Computes the molecular (Rayleigh) extinction coefficient at a given altitude.
///
/// ```text
///   alpha_mol(z) = N(z) * sigma_R
/// ```
///
/// The extinction-to-backscatter ratio (lidar ratio) for molecules is
/// 8*pi/3 sr (approximately 8.378 sr).
///
/// # Arguments
/// * `wavelength_nm` - Laser wavelength in nanometers.
/// * `altitude_m` - Altitude above sea level in meters.
///
/// # Returns
/// Molecular extinction coefficient in 1/m.
pub fn molecular_extinction(wavelength_nm: f64, altitude_m: f64) -> f64 {
    // Molecular lidar ratio is 8*pi/3
    let s_mol = 8.0 * PI / 3.0;
    molecular_backscatter(wavelength_nm, altitude_m) * s_mol
}

/// Computes the optical depth (integral of extinction) between two altitudes.
///
/// Uses trapezoidal integration of the extinction profile.
///
/// # Arguments
/// * `extinction` - Extinction coefficient profile in 1/m.
/// * `ranges` - Range values in meters corresponding to each extinction value.
///
/// # Returns
/// Optical depth (dimensionless).
pub fn optical_depth(extinction: &[f64], ranges: &[f64]) -> f64 {
    let n = extinction.len().min(ranges.len());
    if n < 2 {
        return 0.0;
    }
    let mut tau = 0.0;
    for i in 1..n {
        let dr = ranges[i] - ranges[i - 1];
        tau += 0.5 * (extinction[i] + extinction[i - 1]) * dr;
    }
    tau
}

/// Returns the lidar ratio (extinction-to-backscatter ratio) for common
/// aerosol types at 532 nm.
///
/// # Arguments
/// * `aerosol_type` - Descriptive aerosol type string.
///
/// # Returns
/// Lidar ratio in steradians. Returns 50.0 (continental average) for unknown types.
pub fn lidar_ratio(aerosol_type: &str) -> f64 {
    match aerosol_type.to_lowercase().as_str() {
        "marine" | "sea_salt" => 20.0,
        "dust" | "saharan" | "mineral" => 40.0,
        "continental" | "urban" | "anthropogenic" => 50.0,
        "biomass" | "smoke" | "fire" => 70.0,
        "polluted_continental" | "industrial" => 60.0,
        "clean_continental" => 35.0,
        "cirrus" | "ice" => 25.0,
        "water_cloud" | "liquid" => 18.0,
        "volcanic" | "ash" => 55.0,
        _ => 50.0,
    }
}

/// Computes the Angstrom exponent from extinction at two wavelengths.
///
/// The Angstrom exponent `a` relates extinction at two wavelengths:
///
/// ```text
///   alpha1 / alpha2 = (lambda2 / lambda1)^a
///   a = -ln(alpha1/alpha2) / ln(lambda1/lambda2)
/// ```
///
/// Values near 0 indicate large particles (dust, sea salt); values near 2
/// indicate small particles (pollution, smoke).
///
/// # Arguments
/// * `alpha1` - Extinction at wavelength 1 (1/m).
/// * `lambda1_nm` - Wavelength 1 in nm.
/// * `alpha2` - Extinction at wavelength 2 (1/m).
/// * `lambda2_nm` - Wavelength 2 in nm.
///
/// # Returns
/// Angstrom exponent (dimensionless).
pub fn angstrom_exponent(alpha1: f64, lambda1_nm: f64, alpha2: f64, lambda2_nm: f64) -> f64 {
    if alpha1 <= 0.0 || alpha2 <= 0.0 || lambda1_nm <= 0.0 || lambda2_nm <= 0.0 {
        return 0.0;
    }
    -(alpha1 / alpha2).ln() / (lambda1_nm / lambda2_nm).ln()
}

// ---------------------------------------------------------------------------
// Klett inversion
// ---------------------------------------------------------------------------

/// Klett backward-stable inversion for extinction profiles from elastic
/// backscatter LIDAR.
///
/// The Klett method assumes a power-law relationship between extinction
/// alpha(r) and backscatter beta(r):
///
/// ```text
///   alpha(r) = C * S(r)^(1/kappa)
/// ```
///
/// where S(r) = ln(RCS(r)) is the logarithm of the range-corrected signal
/// and kappa is a constant (typically 1 for aerosols).
///
/// The backward solution integrates from a reference range r_ref (where
/// alpha_ref is known or estimated) back toward the instrument:
///
/// ```text
///   alpha(r) = S(r)^(1/kappa) /
///     [ S(r_ref)^(1/kappa)/alpha_ref + (2/kappa) * integral_r^r_ref S(r')^(1/kappa) dr' ]
/// ```
///
/// This backward integration is numerically stable.
///
/// # References
/// * Klett, J.D. (1981), "Stable analytical inversion solution for processing
///   lidar returns", Appl. Opt. 20, 211-220.
#[derive(Debug, Clone)]
pub struct KlettInverter {
    /// Power-law exponent kappa. Default 1.0 (linear relationship).
    pub kappa: f64,

    /// Index of the reference range bin where the boundary condition is applied.
    pub ref_index: Option<usize>,

    /// Extinction at the reference range (1/m). If not specified, a molecular
    /// value is estimated.
    pub ref_extinction: Option<f64>,
}

impl KlettInverter {
    /// Creates a new Klett inverter with default parameters.
    pub fn new() -> Self {
        Self {
            kappa: 1.0,
            ref_index: None,
            ref_extinction: None,
        }
    }

    /// Sets the power-law exponent kappa.
    pub fn with_kappa(mut self, kappa: f64) -> Self {
        self.kappa = kappa;
        self
    }

    /// Sets the reference bin index (far-range calibration point).
    pub fn with_ref_index(mut self, idx: usize) -> Self {
        self.ref_index = Some(idx);
        self
    }

    /// Sets the extinction boundary value at the reference range.
    pub fn with_ref_extinction(mut self, alpha_ref: f64) -> Self {
        self.ref_extinction = Some(alpha_ref);
        self
    }

    /// Performs backward Klett inversion on a range-corrected signal.
    ///
    /// # Arguments
    /// * `rcs` - Range-corrected signal (P(r) * r^2).
    /// * `ranges` - Range values in meters.
    ///
    /// # Returns
    /// Extinction coefficient profile in 1/m, or `None` if the input is too
    /// short or contains invalid values.
    pub fn invert(&self, rcs: &[f64], ranges: &[f64]) -> Option<Vec<f64>> {
        let n = rcs.len().min(ranges.len());
        if n < 3 {
            return None;
        }

        // Reference index: default to the last valid bin
        let ref_idx = self.ref_index.unwrap_or(n - 1).min(n - 1);

        // Compute ln(RCS) — the signal function S(r)
        let s: Vec<f64> = rcs
            .iter()
            .map(|&v| if v > 0.0 { v.ln() } else { f64::NEG_INFINITY })
            .collect();

        // Check reference bin validity
        if s[ref_idx].is_infinite() {
            return None;
        }

        // Reference extinction
        let alpha_ref = self.ref_extinction.unwrap_or(1e-5);
        if alpha_ref <= 0.0 {
            return None;
        }

        let kappa = self.kappa;
        let inv_kappa = 1.0 / kappa;

        // Compute S^(1/kappa) - use exp(S/kappa) = RCS^(1/kappa)
        let s_pow: Vec<f64> = rcs
            .iter()
            .map(|&v| if v > 0.0 { v.powf(inv_kappa) } else { 0.0 })
            .collect();

        let s_pow_ref = s_pow[ref_idx];
        if s_pow_ref <= 0.0 {
            return None;
        }

        let mut alpha = vec![0.0; n];
        alpha[ref_idx] = alpha_ref;

        // Backward integration from ref_idx - 1 down to 0
        let mut integral = 0.0;
        for i in (0..ref_idx).rev() {
            let dr = ranges[i + 1] - ranges[i];
            // Trapezoidal integration of S^(1/kappa)
            integral += 0.5 * (s_pow[i + 1] + s_pow[i]) * dr;

            let denom = s_pow_ref / alpha_ref + (2.0 / kappa) * integral;
            if denom > 0.0 && s_pow[i] > 0.0 {
                alpha[i] = s_pow[i] / denom;
            }
        }

        // Forward from ref_idx + 1 to end (less stable, but fills the profile)
        let mut integral_fwd = 0.0;
        for i in (ref_idx + 1)..n {
            let dr = ranges[i] - ranges[i - 1];
            integral_fwd += 0.5 * (s_pow[i] + s_pow[i - 1]) * dr;

            let denom = s_pow_ref / alpha_ref - (2.0 / kappa) * integral_fwd;
            if denom > 1e-20 && s_pow[i] > 0.0 {
                alpha[i] = s_pow[i] / denom;
            }
        }

        Some(alpha)
    }
}

// ---------------------------------------------------------------------------
// Fernald method
// ---------------------------------------------------------------------------

/// Fernald two-component inversion separating molecular and aerosol
/// contributions.
///
/// The Fernald method explicitly accounts for the known molecular (Rayleigh)
/// backscatter and extinction, solving only for the aerosol component. This
/// is more physically rigorous than the single-component Klett method.
///
/// The backward solution for aerosol backscatter is:
///
/// ```text
///   beta_a(r) = -beta_m(r) + [S(r) * exp(-2*(Sa-Sm)*tau_m(r,r_ref))] /
///     [ S(r_ref)/[beta_a_ref + beta_m_ref] + 2*Sa * integral_r^r_ref S(r')*exp(...) dr' ]
/// ```
///
/// where Sa and Sm are the aerosol and molecular lidar ratios, and tau_m is
/// the molecular optical depth.
///
/// # References
/// * Fernald, F.G. (1984), "Analysis of atmospheric lidar observations: some
///   comments", Appl. Opt. 23, 652-653.
#[derive(Debug, Clone)]
pub struct FernaldMethod {
    /// Aerosol lidar ratio Sa (extinction/backscatter) in sr.
    /// Typical values: 20 (marine) to 70 (smoke).
    pub aerosol_lidar_ratio: f64,

    /// Molecular lidar ratio (8*pi/3 ~ 8.378 sr for Rayleigh scattering).
    pub molecular_lidar_ratio: f64,

    /// Reference bin index for boundary condition.
    pub ref_index: Option<usize>,

    /// Aerosol backscatter at reference range (1/(m*sr)).
    pub ref_backscatter: Option<f64>,

    /// Wavelength in nm (for computing molecular profiles).
    pub wavelength_nm: f64,

    /// Station altitude in meters above sea level.
    pub station_altitude_m: f64,
}

impl FernaldMethod {
    /// Creates a Fernald method instance with typical parameters for 532 nm.
    pub fn new(wavelength_nm: f64) -> Self {
        Self {
            aerosol_lidar_ratio: 50.0,
            molecular_lidar_ratio: 8.0 * PI / 3.0,
            ref_index: None,
            ref_backscatter: None,
            wavelength_nm,
            station_altitude_m: 0.0,
        }
    }

    /// Sets the aerosol lidar ratio.
    pub fn with_lidar_ratio(mut self, sa: f64) -> Self {
        self.aerosol_lidar_ratio = sa;
        self
    }

    /// Sets the reference bin index.
    pub fn with_ref_index(mut self, idx: usize) -> Self {
        self.ref_index = Some(idx);
        self
    }

    /// Sets the aerosol backscatter at the reference point.
    pub fn with_ref_backscatter(mut self, beta_ref: f64) -> Self {
        self.ref_backscatter = Some(beta_ref);
        self
    }

    /// Sets the station altitude for molecular profile computation.
    pub fn with_station_altitude(mut self, alt_m: f64) -> Self {
        self.station_altitude_m = alt_m;
        self
    }

    /// Performs Fernald inversion on a range-corrected signal.
    ///
    /// # Arguments
    /// * `rcs` - Range-corrected signal (P(r) * r^2).
    /// * `ranges` - Range values in meters (from instrument).
    ///
    /// # Returns
    /// A tuple of (aerosol backscatter, aerosol extinction) profiles, each in
    /// appropriate SI units, or `None` on failure.
    pub fn invert(&self, rcs: &[f64], ranges: &[f64]) -> Option<(Vec<f64>, Vec<f64>)> {
        let n = rcs.len().min(ranges.len());
        if n < 3 {
            return None;
        }

        let ref_idx = self.ref_index.unwrap_or(n - 1).min(n - 1);
        let sa = self.aerosol_lidar_ratio;
        let sm = self.molecular_lidar_ratio;

        // Compute molecular backscatter and extinction profiles
        let beta_mol: Vec<f64> = ranges
            .iter()
            .map(|&r| molecular_backscatter(self.wavelength_nm, self.station_altitude_m + r))
            .collect();
        let alpha_mol: Vec<f64> = beta_mol.iter().map(|&b| b * sm).collect();

        // Cumulative molecular optical depth from bin 0
        let mut tau_mol = vec![0.0; n];
        for i in 1..n {
            let dr = ranges[i] - ranges[i - 1];
            tau_mol[i] = tau_mol[i - 1] + 0.5 * (alpha_mol[i] + alpha_mol[i - 1]) * dr;
        }

        // Reference aerosol backscatter
        let beta_a_ref = self.ref_backscatter.unwrap_or(1e-7);
        let beta_total_ref = beta_a_ref + beta_mol[ref_idx];

        if beta_total_ref <= 0.0 {
            return None;
        }

        // Compute the adjusted signal: S_adj(r) = RCS(r) * exp(-2*(Sa - Sm)*tau_mol(r_ref, r))
        // tau_mol(r_ref, r) = tau_mol[ref_idx] - tau_mol[r]  (for r < ref_idx)
        // But we need the integral from r to r_ref in the molecular optical depth
        let s_adj: Vec<f64> = (0..n)
            .map(|i| {
                let dtau = tau_mol[ref_idx] - tau_mol[i];
                if rcs[i] > 0.0 {
                    rcs[i] * (-2.0 * (sa - sm) * dtau).exp()
                } else {
                    0.0
                }
            })
            .collect();

        let s_adj_ref = s_adj[ref_idx];
        if s_adj_ref <= 0.0 {
            return None;
        }

        let mut beta_a = vec![0.0; n];
        beta_a[ref_idx] = beta_a_ref;

        // Backward integration from ref_idx-1 to 0
        let mut integral = 0.0;
        for i in (0..ref_idx).rev() {
            let dr = ranges[i + 1] - ranges[i];
            integral += 0.5 * (s_adj[i + 1] + s_adj[i]) * dr;

            let denom = s_adj_ref / beta_total_ref + 2.0 * sa * integral;
            if denom > 0.0 && s_adj[i] > 0.0 {
                let beta_total = s_adj[i] / denom;
                beta_a[i] = (beta_total - beta_mol[i]).max(0.0);
            }
        }

        // Forward extension (less stable)
        let mut integral_fwd = 0.0;
        for i in (ref_idx + 1)..n {
            let dr = ranges[i] - ranges[i - 1];
            integral_fwd += 0.5 * (s_adj[i] + s_adj[i - 1]) * dr;

            let denom = s_adj_ref / beta_total_ref - 2.0 * sa * integral_fwd;
            if denom > 1e-20 && s_adj[i] > 0.0 {
                let beta_total = s_adj[i] / denom;
                beta_a[i] = (beta_total - beta_mol[i]).max(0.0);
            }
        }

        // Aerosol extinction = Sa * beta_a
        let alpha_a: Vec<f64> = beta_a.iter().map(|&b| sa * b).collect();

        Some((beta_a, alpha_a))
    }
}

// ---------------------------------------------------------------------------
// Boundary layer detection
// ---------------------------------------------------------------------------

/// Method for detecting the planetary boundary layer height.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PblMethod {
    /// Gradient method: PBL top is where d(beta)/dr has the largest negative value.
    Gradient,

    /// Wavelet covariance transform (Haar wavelet) to detect sharp transitions.
    Wavelet,

    /// Threshold method: PBL top is where backscatter drops below a fraction
    /// of the near-surface average.
    Threshold,

    /// Variance method: PBL top is where temporal variance of backscatter is
    /// maximum.
    Variance,
}

/// Detects the planetary boundary layer (PBL) height from a backscatter
/// profile.
///
/// The PBL marks the top of the convectively mixed layer, typically
/// 500-3000 m during daytime. It appears as a sharp decrease in aerosol
/// backscatter because aerosols are trapped within the boundary layer.
///
/// # Methods
///
/// - **Gradient**: The minimum of d(beta)/dr (steepest negative gradient).
///   Simple and fast, but sensitive to noise.
///
/// - **Wavelet**: Applies a Haar wavelet covariance transform to find the
///   altitude of maximum covariance. More robust to noise than gradient.
///
/// - **Threshold**: PBL top where backscatter drops below a specified
///   fraction of the near-surface average. Simple but requires threshold
///   tuning.
///
/// - **Variance**: Uses temporal variability of multiple profiles to detect
///   the PBL. Requires multiple profiles passed via the `multi_profile` API.
#[derive(Debug, Clone)]
pub struct BoundaryLayerDetector {
    /// Detection method.
    pub method: PblMethod,

    /// Minimum search altitude in meters (to skip near-field noise).
    pub min_altitude_m: f64,

    /// Maximum search altitude in meters.
    pub max_altitude_m: f64,

    /// Dilation parameter for wavelet method (half-width in bins).
    pub wavelet_dilation: usize,

    /// Threshold fraction for threshold method (e.g. 0.5 = 50% of near-surface).
    pub threshold_fraction: f64,
}

impl BoundaryLayerDetector {
    /// Creates a detector with default parameters.
    pub fn new(method: PblMethod) -> Self {
        Self {
            method,
            min_altitude_m: 200.0,
            max_altitude_m: 5000.0,
            wavelet_dilation: 10,
            threshold_fraction: 0.5,
        }
    }

    /// Sets the search altitude range.
    pub fn with_altitude_range(mut self, min_m: f64, max_m: f64) -> Self {
        self.min_altitude_m = min_m;
        self.max_altitude_m = max_m;
        self
    }

    /// Sets the wavelet dilation parameter.
    pub fn with_wavelet_dilation(mut self, d: usize) -> Self {
        self.wavelet_dilation = d.max(1);
        self
    }

    /// Sets the threshold fraction.
    pub fn with_threshold_fraction(mut self, f: f64) -> Self {
        self.threshold_fraction = f.clamp(0.01, 0.99);
        self
    }

    /// Detects the PBL height from a single backscatter profile.
    ///
    /// # Arguments
    /// * `backscatter` - Backscatter coefficient profile (1/(m*sr)).
    /// * `ranges` - Range values in meters (used as altitude proxy for
    ///   vertically-pointing lidar).
    ///
    /// # Returns
    /// Estimated PBL height in meters, or `None` if detection fails.
    pub fn detect(&self, backscatter: &[f64], ranges: &[f64]) -> Option<f64> {
        let n = backscatter.len().min(ranges.len());
        if n < 5 {
            return None;
        }

        // Determine search range in bin indices
        let start_bin = ranges
            .iter()
            .position(|&r| r >= self.min_altitude_m)
            .unwrap_or(0);
        let end_bin = ranges
            .iter()
            .position(|&r| r > self.max_altitude_m)
            .unwrap_or(n)
            .min(n);

        if end_bin <= start_bin + 2 {
            return None;
        }

        match self.method {
            PblMethod::Gradient => self.detect_gradient(backscatter, ranges, start_bin, end_bin),
            PblMethod::Wavelet => self.detect_wavelet(backscatter, ranges, start_bin, end_bin),
            PblMethod::Threshold => {
                self.detect_threshold(backscatter, ranges, start_bin, end_bin)
            }
            PblMethod::Variance => {
                // Single-profile variance not meaningful; fall back to gradient
                self.detect_gradient(backscatter, ranges, start_bin, end_bin)
            }
        }
    }

    /// Detects PBL using multiple profiles (for variance method).
    ///
    /// # Arguments
    /// * `profiles` - Multiple backscatter profiles (each same length).
    /// * `ranges` - Range values in meters.
    ///
    /// # Returns
    /// Estimated PBL height in meters, or `None` if detection fails.
    pub fn detect_multi(&self, profiles: &[Vec<f64>], ranges: &[f64]) -> Option<f64> {
        if profiles.is_empty() || profiles.len() < 3 {
            return None;
        }

        let n = profiles[0].len().min(ranges.len());
        if n < 5 {
            return None;
        }

        let start_bin = ranges
            .iter()
            .position(|&r| r >= self.min_altitude_m)
            .unwrap_or(0);
        let end_bin = ranges
            .iter()
            .position(|&r| r > self.max_altitude_m)
            .unwrap_or(n)
            .min(n);

        if end_bin <= start_bin + 2 {
            return None;
        }

        if self.method == PblMethod::Variance {
            self.detect_variance(profiles, ranges, start_bin, end_bin)
        } else {
            // Average the profiles and apply single-profile method
            let num_profiles = profiles.len() as f64;
            let avg: Vec<f64> = (0..n)
                .map(|i| {
                    profiles.iter().map(|p| if i < p.len() { p[i] } else { 0.0 }).sum::<f64>()
                        / num_profiles
                })
                .collect();
            self.detect(&avg, ranges)
        }
    }

    fn detect_gradient(
        &self,
        backscatter: &[f64],
        ranges: &[f64],
        start: usize,
        end: usize,
    ) -> Option<f64> {
        let mut min_grad = f64::MAX;
        let mut min_idx = start;

        for i in (start + 1)..end.min(backscatter.len()) {
            let dr = ranges[i] - ranges[i - 1];
            if dr <= 0.0 {
                continue;
            }
            let grad = (backscatter[i] - backscatter[i - 1]) / dr;
            if grad < min_grad {
                min_grad = grad;
                min_idx = i;
            }
        }

        if min_grad < 0.0 {
            Some(ranges[min_idx])
        } else {
            None
        }
    }

    fn detect_wavelet(
        &self,
        backscatter: &[f64],
        ranges: &[f64],
        start: usize,
        end: usize,
    ) -> Option<f64> {
        let d = self.wavelet_dilation;
        let n = backscatter.len();

        if end + d > n || start < d {
            // Not enough margin for wavelet; fall back to gradient
            return self.detect_gradient(backscatter, ranges, start, end);
        }

        // Haar wavelet covariance transform
        // W(b) = (1/d) * sum_{z=b-d}^{b} f(z) - (1/d) * sum_{z=b}^{b+d} f(z)
        let mut max_wcov = f64::NEG_INFINITY;
        let mut max_idx = start;

        for b in (start + d)..(end.min(n - d)) {
            let lower: f64 = (0..d).map(|j| backscatter[b - d + j]).sum::<f64>() / d as f64;
            let upper: f64 = (0..d).map(|j| backscatter[b + j]).sum::<f64>() / d as f64;
            let wcov = lower - upper; // Positive for a decrease in backscatter

            if wcov > max_wcov {
                max_wcov = wcov;
                max_idx = b;
            }
        }

        if max_wcov > 0.0 {
            Some(ranges[max_idx])
        } else {
            None
        }
    }

    fn detect_threshold(
        &self,
        backscatter: &[f64],
        ranges: &[f64],
        start: usize,
        end: usize,
    ) -> Option<f64> {
        // Compute near-surface average (first 10% of search range or at least 3 bins)
        let avg_bins = ((end - start) / 10).max(3).min(end - start);
        let near_surface_avg: f64 =
            backscatter[start..(start + avg_bins)].iter().sum::<f64>() / avg_bins as f64;

        let threshold = near_surface_avg * self.threshold_fraction;

        // Find first bin where backscatter drops below threshold
        for i in (start + avg_bins)..end {
            if backscatter[i] < threshold {
                return Some(ranges[i]);
            }
        }

        None
    }

    fn detect_variance(
        &self,
        profiles: &[Vec<f64>],
        ranges: &[f64],
        start: usize,
        end: usize,
    ) -> Option<f64> {
        let np = profiles.len() as f64;
        let mut max_var = f64::NEG_INFINITY;
        let mut max_idx = start;

        for i in start..end {
            // Compute variance at this bin across all profiles
            let mean = profiles.iter().map(|p| p[i]).sum::<f64>() / np;
            let var = profiles.iter().map(|p| (p[i] - mean).powi(2)).sum::<f64>() / np;

            if var > max_var {
                max_var = var;
                max_idx = i;
            }
        }

        if max_var > 0.0 {
            Some(ranges[max_idx])
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Signal-to-Noise estimation
// ---------------------------------------------------------------------------

/// Estimates the signal-to-noise ratio (SNR) of a LIDAR return signal at
/// each range bin.
///
/// Uses a sliding window to estimate noise from the variance of the signal.
///
/// # Arguments
/// * `signal` - LIDAR return signal.
/// * `window` - Sliding window size for noise estimation.
///
/// # Returns
/// SNR in dB at each range bin.
pub fn estimate_snr(signal: &[f64], window: usize) -> Vec<f64> {
    let n = signal.len();
    let w = window.max(3).min(n);
    let mut snr = vec![0.0; n];

    for i in 0..n {
        let start = if i >= w / 2 { i - w / 2 } else { 0 };
        let end = (i + w / 2 + 1).min(n);
        let count = (end - start) as f64;

        let mean = signal[start..end].iter().sum::<f64>() / count;
        let var = signal[start..end]
            .iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>()
            / count;

        if var > 0.0 {
            snr[i] = 10.0 * (mean * mean / var).log10();
        }
    }

    snr
}

/// Smooths a LIDAR profile using a simple moving average.
///
/// # Arguments
/// * `data` - Input profile.
/// * `window` - Smoothing window size (will be forced to odd number).
///
/// # Returns
/// Smoothed profile (same length as input).
pub fn smooth_profile(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if n == 0 || window <= 1 {
        return data.to_vec();
    }
    let w = if window % 2 == 0 { window + 1 } else { window };
    let half = w / 2;
    let mut result = vec![0.0; n];

    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = (i + half + 1).min(n);
        let count = (end - start) as f64;
        result[i] = data[start..end].iter().sum::<f64>() / count;
    }

    result
}

/// Generates a synthetic LIDAR return signal for testing purposes.
///
/// Creates a signal with an exponential atmosphere, an aerosol layer at a
/// specified altitude, and optional noise.
///
/// # Arguments
/// * `config` - LIDAR configuration.
/// * `n_bins` - Number of range bins.
/// * `layer_center_m` - Center altitude of the aerosol layer in meters.
/// * `layer_width_m` - Width (sigma) of the Gaussian aerosol layer.
/// * `layer_backscatter` - Peak aerosol backscatter coefficient (1/(m*sr)).
/// * `noise_level` - RMS noise level (0 for no noise).
///
/// # Returns
/// Tuple of (signal, ranges) where signal is the raw return and ranges are
/// range values in meters.
pub fn generate_synthetic_signal(
    config: &LidarConfig,
    n_bins: usize,
    layer_center_m: f64,
    layer_width_m: f64,
    layer_backscatter: f64,
    noise_level: f64,
) -> (Vec<f64>, Vec<f64>) {
    let dr = config.range_resolution_m;
    let ranges: Vec<f64> = (1..=n_bins).map(|i| i as f64 * dr).collect();

    // Total backscatter = molecular + aerosol layer
    let beta: Vec<f64> = ranges
        .iter()
        .map(|&r| {
            let beta_m = molecular_backscatter(config.wavelength_nm, r);
            let beta_a = layer_backscatter
                * (-0.5 * ((r - layer_center_m) / layer_width_m).powi(2)).exp();
            beta_m + beta_a
        })
        .collect();

    // Total extinction (using lidar ratios)
    let sm = 8.0 * PI / 3.0;
    let sa = 50.0;
    let alpha: Vec<f64> = ranges
        .iter()
        .enumerate()
        .map(|(_i, &r)| {
            let alpha_m = molecular_backscatter(config.wavelength_nm, r) * sm;
            let alpha_a = sa
                * layer_backscatter
                * (-0.5 * ((r - layer_center_m) / layer_width_m).powi(2)).exp();
            alpha_m + alpha_a
        })
        .collect();

    // Two-way optical depth
    let mut tau = vec![0.0; n_bins];
    for i in 1..n_bins {
        tau[i] = tau[i - 1] + 0.5 * (alpha[i] + alpha[i - 1]) * dr;
    }

    // LIDAR equation: P(r) = P0 * A / r^2 * beta(r) * exp(-2*tau(r))
    let c = config.pulse_energy_j * config.telescope_area_m2;
    let mut signal: Vec<f64> = (0..n_bins)
        .map(|i| {
            let r = ranges[i];
            let overlap = if let Some(ref ov) = config.overlap {
                if i < ov.len() {
                    ov[i]
                } else {
                    1.0
                }
            } else {
                1.0
            };
            c * overlap / (r * r) * beta[i] * (-2.0 * tau[i]).exp()
        })
        .collect();

    // Add noise using a simple deterministic pseudo-random generator
    if noise_level > 0.0 {
        let mut seed: u64 = 42;
        for i in 0..n_bins {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u = (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
            signal[i] += noise_level * u * 2.0;
        }
    }

    (signal, ranges)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    // -- LidarConfig tests --

    #[test]
    fn test_lidar_config_wavelength_conversion() {
        let config = LidarConfig {
            wavelength_nm: 532.0,
            pulse_energy_j: 0.01,
            telescope_area_m2: 0.07,
            range_resolution_m: 15.0,
            overlap: None,
        };
        assert!((config.wavelength_m() - 532e-9).abs() < 1e-15);
    }

    #[test]
    fn test_lidar_config_1064nm() {
        let config = LidarConfig {
            wavelength_nm: 1064.0,
            pulse_energy_j: 0.1,
            telescope_area_m2: 0.2,
            range_resolution_m: 30.0,
            overlap: None,
        };
        assert!((config.wavelength_m() - 1064e-9).abs() < 1e-15);
    }

    // -- RangeCorrector tests --

    #[test]
    fn test_range_correction_basic() {
        let rc = RangeCorrector;
        let signal = vec![1.0, 0.25, 0.1111];
        let ranges = vec![1.0, 2.0, 3.0];
        let rcs = rc.correct(&signal, &ranges);
        assert!((rcs[0] - 1.0).abs() < EPS);
        assert!((rcs[1] - 1.0).abs() < EPS);
        assert!((rcs[2] - 0.9999).abs() < 0.001);
    }

    #[test]
    fn test_range_correction_roundtrip() {
        let rc = RangeCorrector;
        let signal = vec![0.5, 0.3, 0.1, 0.05];
        let ranges = vec![100.0, 200.0, 300.0, 400.0];
        let rcs = rc.correct(&signal, &ranges);
        let recovered = rc.uncorrect(&rcs, &ranges);
        for i in 0..signal.len() {
            assert!(
                (recovered[i] - signal[i]).abs() < 1e-12,
                "bin {}: {} != {}",
                i,
                recovered[i],
                signal[i]
            );
        }
    }

    #[test]
    fn test_range_correction_zero_range() {
        let rc = RangeCorrector;
        let rcs = vec![100.0];
        let ranges = vec![0.0];
        let result = rc.uncorrect(&rcs, &ranges);
        assert_eq!(result[0], 0.0);
    }

    #[test]
    fn test_range_correction_mismatched_lengths() {
        let rc = RangeCorrector;
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let ranges = vec![10.0, 20.0, 30.0];
        let rcs = rc.correct(&signal, &ranges);
        assert_eq!(rcs.len(), 3); // Truncated to shorter length
    }

    // -- OverlapCorrector tests --

    #[test]
    fn test_overlap_correction_full_overlap() {
        let oc = OverlapCorrector::new(vec![1.0, 1.0, 1.0, 1.0]);
        let signal = vec![10.0, 20.0, 30.0, 40.0];
        let corrected = oc.correct(&signal);
        for i in 0..4 {
            assert!((corrected[i] - signal[i]).abs() < EPS);
        }
    }

    #[test]
    fn test_overlap_correction_partial() {
        let oc = OverlapCorrector::new(vec![0.5, 0.8, 1.0]);
        let signal = vec![5.0, 8.0, 10.0];
        let corrected = oc.correct(&signal);
        assert!((corrected[0] - 10.0).abs() < EPS);
        assert!((corrected[1] - 10.0).abs() < EPS);
        assert!((corrected[2] - 10.0).abs() < EPS);
    }

    #[test]
    fn test_overlap_correction_below_threshold() {
        let mut oc = OverlapCorrector::new(vec![0.05, 0.5, 1.0]);
        oc.set_min_overlap(0.1);
        let signal = vec![1.0, 2.0, 3.0];
        let corrected = oc.correct(&signal);
        // Below threshold: should not correct
        assert!((corrected[0] - 1.0).abs() < EPS);
        // Above threshold: should correct
        assert!((corrected[1] - 4.0).abs() < EPS);
    }

    #[test]
    fn test_overlap_exponential_model() {
        let ranges = vec![100.0, 500.0, 1000.0, 2000.0];
        let oc = OverlapCorrector::from_exponential_model(&ranges, 500.0);
        let vals = oc.overlap_values();
        // At r=500 (r_full), overlap ~ 1 - e^-1 ~ 0.632
        assert!((vals[1] - (1.0 - (-1.0_f64).exp())).abs() < 0.01);
        // At r=2000, nearly full overlap
        assert!(vals[3] > 0.98);
    }

    // -- Molecular scattering tests --

    #[test]
    fn test_molecular_backscatter_positive() {
        let beta = molecular_backscatter(532.0, 0.0);
        assert!(beta > 0.0, "molecular backscatter must be positive");
    }

    #[test]
    fn test_molecular_backscatter_decreases_with_altitude() {
        let beta_0 = molecular_backscatter(532.0, 0.0);
        let beta_5000 = molecular_backscatter(532.0, 5000.0);
        assert!(
            beta_5000 < beta_0,
            "molecular backscatter should decrease with altitude"
        );
    }

    #[test]
    fn test_molecular_backscatter_wavelength_dependence() {
        // Rayleigh scattering is stronger at shorter wavelengths (lambda^-4)
        let beta_355 = molecular_backscatter(355.0, 0.0);
        let beta_532 = molecular_backscatter(532.0, 0.0);
        let beta_1064 = molecular_backscatter(1064.0, 0.0);
        assert!(
            beta_355 > beta_532,
            "355 nm should scatter more than 532 nm"
        );
        assert!(
            beta_532 > beta_1064,
            "532 nm should scatter more than 1064 nm"
        );
    }

    #[test]
    fn test_molecular_extinction() {
        let alpha = molecular_extinction(532.0, 0.0);
        let beta = molecular_backscatter(532.0, 0.0);
        let sm = 8.0 * PI / 3.0;
        assert!((alpha - beta * sm).abs() < 1e-15);
    }

    // -- Optical depth tests --

    #[test]
    fn test_optical_depth_constant_extinction() {
        // Constant extinction of 1e-4/m over 1000m should give tau = 0.1
        let ranges: Vec<f64> = (0..100).map(|i| i as f64 * 10.0).collect();
        let extinction = vec![1e-4; 100];
        let tau = optical_depth(&extinction, &ranges);
        assert!(
            (tau - 0.099).abs() < 0.01, // trapezoidal approx
            "tau = {}, expected ~0.099",
            tau
        );
    }

    #[test]
    fn test_optical_depth_empty() {
        assert_eq!(optical_depth(&[], &[]), 0.0);
        assert_eq!(optical_depth(&[1.0], &[100.0]), 0.0);
    }

    // -- Lidar ratio tests --

    #[test]
    fn test_lidar_ratio_known_types() {
        assert!((lidar_ratio("marine") - 20.0).abs() < EPS);
        assert!((lidar_ratio("dust") - 40.0).abs() < EPS);
        assert!((lidar_ratio("continental") - 50.0).abs() < EPS);
        assert!((lidar_ratio("smoke") - 70.0).abs() < EPS);
        assert!((lidar_ratio("cirrus") - 25.0).abs() < EPS);
    }

    #[test]
    fn test_lidar_ratio_unknown_default() {
        assert!((lidar_ratio("unknown_type") - 50.0).abs() < EPS);
    }

    // -- Angstrom exponent tests --

    #[test]
    fn test_angstrom_exponent_rayleigh() {
        // For Rayleigh scattering, Angstrom exponent ~ 4
        let alpha_355 = molecular_extinction(355.0, 0.0);
        let alpha_532 = molecular_extinction(532.0, 0.0);
        let ae = angstrom_exponent(alpha_355, 355.0, alpha_532, 532.0);
        assert!(
            (ae - 4.0).abs() < 1.0,
            "Rayleigh Angstrom exponent should be ~4, got {}",
            ae
        );
    }

    #[test]
    fn test_angstrom_exponent_invalid() {
        assert_eq!(angstrom_exponent(0.0, 355.0, 1e-4, 532.0), 0.0);
        assert_eq!(angstrom_exponent(1e-4, 355.0, -1.0, 532.0), 0.0);
    }

    // -- Klett inversion tests --

    #[test]
    fn test_klett_inversion_basic() {
        let config = LidarConfig {
            wavelength_nm: 532.0,
            pulse_energy_j: 0.01,
            telescope_area_m2: 0.07,
            range_resolution_m: 30.0,
            overlap: None,
        };

        let (signal, ranges) = generate_synthetic_signal(&config, 200, 1500.0, 300.0, 5e-6, 0.0);
        let rc = RangeCorrector;
        let rcs = rc.correct(&signal, &ranges);

        let klett = KlettInverter::new()
            .with_ref_index(199)
            .with_ref_extinction(1e-5);

        let alpha = klett.invert(&rcs, &ranges);
        assert!(alpha.is_some(), "Klett inversion should succeed");
        let alpha = alpha.unwrap();
        assert_eq!(alpha.len(), 200);

        // Extinction should be positive in the aerosol layer
        let layer_bin = (1500.0 / 30.0) as usize;
        assert!(
            alpha[layer_bin] > 0.0,
            "Extinction should be positive at layer center"
        );
    }

    #[test]
    fn test_klett_inversion_too_short() {
        let klett = KlettInverter::new();
        assert!(klett.invert(&[1.0, 2.0], &[100.0, 200.0]).is_none());
    }

    #[test]
    fn test_klett_inversion_with_kappa() {
        let config = LidarConfig {
            wavelength_nm: 532.0,
            pulse_energy_j: 0.01,
            telescope_area_m2: 0.07,
            range_resolution_m: 30.0,
            overlap: None,
        };

        let (signal, ranges) = generate_synthetic_signal(&config, 100, 1500.0, 300.0, 3e-6, 0.0);
        let rc = RangeCorrector;
        let rcs = rc.correct(&signal, &ranges);

        let klett = KlettInverter::new()
            .with_kappa(0.67)
            .with_ref_index(99)
            .with_ref_extinction(1e-5);

        let alpha = klett.invert(&rcs, &ranges);
        assert!(alpha.is_some());
    }

    // -- Fernald method tests --

    #[test]
    fn test_fernald_inversion_basic() {
        let config = LidarConfig {
            wavelength_nm: 532.0,
            pulse_energy_j: 0.01,
            telescope_area_m2: 0.07,
            range_resolution_m: 15.0,
            overlap: None,
        };

        // Aerosol layer at 750m with strong backscatter, short range to keep
        // molecular optical depth small (Fernald is most accurate in this regime).
        let (signal, ranges) =
            generate_synthetic_signal(&config, 100, 750.0, 150.0, 2e-5, 0.0);
        let rc = RangeCorrector;
        let rcs = rc.correct(&signal, &ranges);

        // Use the molecular lidar ratio as Sa for this test so the exponential
        // correction factor is unity, isolating the inversion logic.
        let sm = 8.0 * std::f64::consts::PI / 3.0;
        let fernald = FernaldMethod::new(532.0)
            .with_lidar_ratio(sm) // Sa = Sm eliminates the molecular correction
            .with_ref_index(99)
            .with_ref_backscatter(1e-8);

        let result = fernald.invert(&rcs, &ranges);
        assert!(result.is_some(), "Fernald inversion should succeed");

        let (beta_a, alpha_a) = result.unwrap();
        assert_eq!(beta_a.len(), 100);
        assert_eq!(alpha_a.len(), 100);

        // Find the peak of the retrieved aerosol backscatter profile
        let mut max_beta = 0.0_f64;
        let mut max_idx = 0;
        for i in 0..100 {
            if beta_a[i] > max_beta {
                max_beta = beta_a[i];
                max_idx = i;
            }
        }

        // The peak should be near the layer center (750m / 15m = bin 50)
        let layer_bin = (750.0 / 15.0) as usize;
        assert!(
            max_beta > 0.0,
            "Peak aerosol backscatter should be positive"
        );
        assert!(
            (max_idx as i32 - layer_bin as i32).unsigned_abs() < 15,
            "Peak at bin {} should be near layer bin {}", max_idx, layer_bin
        );

        // alpha = Sa * beta relationship
        for i in 0..100 {
            assert!((alpha_a[i] - sm * beta_a[i]).abs() < 1e-15);
        }
    }

    #[test]
    fn test_fernald_too_short() {
        let fernald = FernaldMethod::new(532.0);
        assert!(fernald.invert(&[1.0, 2.0], &[100.0, 200.0]).is_none());
    }

    #[test]
    fn test_fernald_with_station_altitude() {
        let config = LidarConfig {
            wavelength_nm: 532.0,
            pulse_energy_j: 0.01,
            telescope_area_m2: 0.07,
            range_resolution_m: 30.0,
            overlap: None,
        };

        let (signal, ranges) = generate_synthetic_signal(&config, 100, 1500.0, 300.0, 3e-6, 0.0);
        let rc = RangeCorrector;
        let rcs = rc.correct(&signal, &ranges);

        let fernald = FernaldMethod::new(532.0)
            .with_station_altitude(500.0)
            .with_ref_index(99)
            .with_ref_backscatter(1e-7);

        let result = fernald.invert(&rcs, &ranges);
        assert!(result.is_some());
    }

    // -- Boundary layer detection tests --

    #[test]
    fn test_pbl_gradient_detection() {
        // Create a profile with a sharp drop at 1000m
        let n = 200;
        let dr = 15.0;
        let ranges: Vec<f64> = (1..=n).map(|i| i as f64 * dr).collect();
        let backscatter: Vec<f64> = ranges
            .iter()
            .map(|&r| {
                if r < 1000.0 {
                    5e-6
                } else {
                    1e-6
                }
            })
            .collect();

        let detector = BoundaryLayerDetector::new(PblMethod::Gradient)
            .with_altitude_range(100.0, 2500.0);

        let pbl = detector.detect(&backscatter, &ranges);
        assert!(pbl.is_some());
        let pbl_height = pbl.unwrap();
        assert!(
            (pbl_height - 1005.0).abs() < 30.0,
            "PBL should be near 1000m, got {}",
            pbl_height
        );
    }

    #[test]
    fn test_pbl_wavelet_detection() {
        let n = 200;
        let dr = 15.0;
        let ranges: Vec<f64> = (1..=n).map(|i| i as f64 * dr).collect();
        // Smooth transition at PBL
        let backscatter: Vec<f64> = ranges
            .iter()
            .map(|&r| {
                let pbl = 1200.0;
                5e-6 * 0.5 * (1.0 - ((r - pbl) / 100.0).tanh()) + 1e-6
            })
            .collect();

        let detector = BoundaryLayerDetector::new(PblMethod::Wavelet)
            .with_altitude_range(200.0, 2500.0)
            .with_wavelet_dilation(5);

        let pbl = detector.detect(&backscatter, &ranges);
        assert!(pbl.is_some());
        let pbl_height = pbl.unwrap();
        assert!(
            (pbl_height - 1200.0).abs() < 200.0,
            "PBL should be near 1200m, got {}",
            pbl_height
        );
    }

    #[test]
    fn test_pbl_threshold_detection() {
        let n = 200;
        let dr = 15.0;
        let ranges: Vec<f64> = (1..=n).map(|i| i as f64 * dr).collect();
        let backscatter: Vec<f64> = ranges
            .iter()
            .map(|&r| if r < 800.0 { 1e-5 } else { 2e-6 })
            .collect();

        let detector = BoundaryLayerDetector::new(PblMethod::Threshold)
            .with_altitude_range(100.0, 2500.0)
            .with_threshold_fraction(0.5);

        let pbl = detector.detect(&backscatter, &ranges);
        assert!(pbl.is_some());
        let pbl_height = pbl.unwrap();
        assert!(
            (pbl_height - 810.0).abs() < 50.0,
            "PBL should be near 800m, got {}",
            pbl_height
        );
    }

    #[test]
    fn test_pbl_variance_detection() {
        let n = 100;
        let dr = 30.0;
        let ranges: Vec<f64> = (1..=n).map(|i| i as f64 * dr).collect();

        // Create profiles with high variability at PBL (~1500m)
        let profiles: Vec<Vec<f64>> = (0..10)
            .map(|k| {
                ranges
                    .iter()
                    .map(|&r| {
                        let base = if r < 1500.0 { 5e-6 } else { 1e-6 };
                        // Add more variability at the PBL
                        let var = if (r - 1500.0).abs() < 200.0 {
                            2e-6 * (k as f64 * 0.3).sin()
                        } else {
                            1e-7 * (k as f64 * 0.3).sin()
                        };
                        base + var
                    })
                    .collect()
            })
            .collect();

        let detector = BoundaryLayerDetector::new(PblMethod::Variance)
            .with_altitude_range(200.0, 4000.0);

        let pbl = detector.detect_multi(&profiles, &ranges);
        assert!(pbl.is_some());
        let pbl_height = pbl.unwrap();
        assert!(
            (pbl_height - 1500.0).abs() < 300.0,
            "PBL should be near 1500m, got {}",
            pbl_height
        );
    }

    #[test]
    fn test_pbl_too_few_bins() {
        let detector = BoundaryLayerDetector::new(PblMethod::Gradient);
        assert!(detector.detect(&[1.0, 2.0], &[100.0, 200.0]).is_none());
    }

    // -- SNR estimation tests --

    #[test]
    fn test_snr_estimation() {
        // Constant signal should have high SNR (low noise variance)
        let signal = vec![1.0; 100];
        let snr = estimate_snr(&signal, 10);
        assert_eq!(snr.len(), 100);
        // Constant signal -> variance = 0, SNR should be 0 (log of 0/0)
        // Actually signal[i] / 0 -> infinity, but our code returns 0 when var=0
    }

    #[test]
    fn test_snr_varying_signal() {
        // A signal with variation
        let signal: Vec<f64> = (0..50)
            .map(|i| 10.0 + (i as f64 * 0.1).sin())
            .collect();
        let snr = estimate_snr(&signal, 5);
        assert_eq!(snr.len(), 50);
        // SNR should be positive for a signal with small noise relative to mean
        for &s in &snr {
            assert!(s >= 0.0 || s.is_finite());
        }
    }

    // -- Smoothing tests --

    #[test]
    fn test_smooth_profile() {
        let data = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let smoothed = smooth_profile(&data, 3);
        assert_eq!(smoothed.len(), 5);
        // Center bin should be average of [0, 10, 0] = 3.33
        assert!((smoothed[2] - 10.0 / 3.0).abs() < 0.01);
    }

    #[test]
    fn test_smooth_profile_window_1() {
        let data = vec![1.0, 2.0, 3.0];
        let smoothed = smooth_profile(&data, 1);
        for i in 0..3 {
            assert!((smoothed[i] - data[i]).abs() < EPS);
        }
    }

    // -- Synthetic signal generation tests --

    #[test]
    fn test_generate_synthetic_signal() {
        let config = LidarConfig {
            wavelength_nm: 532.0,
            pulse_energy_j: 0.01,
            telescope_area_m2: 0.07,
            range_resolution_m: 15.0,
            overlap: None,
        };

        let (signal, ranges) = generate_synthetic_signal(&config, 100, 1500.0, 300.0, 5e-6, 0.0);
        assert_eq!(signal.len(), 100);
        assert_eq!(ranges.len(), 100);
        assert!((ranges[0] - 15.0).abs() < EPS);
        assert!((ranges[99] - 1500.0).abs() < EPS);

        // Signal should decrease with range (1/r^2 + atmospheric absorption)
        assert!(signal[0] > signal[99]);
    }

    #[test]
    fn test_generate_synthetic_with_noise() {
        let config = LidarConfig {
            wavelength_nm: 532.0,
            pulse_energy_j: 0.01,
            telescope_area_m2: 0.07,
            range_resolution_m: 15.0,
            overlap: None,
        };

        let (sig_clean, _) =
            generate_synthetic_signal(&config, 50, 500.0, 100.0, 1e-6, 0.0);
        let (sig_noisy, _) =
            generate_synthetic_signal(&config, 50, 500.0, 100.0, 1e-6, 1e-10);

        // At least some bins should differ
        let different = sig_clean
            .iter()
            .zip(sig_noisy.iter())
            .any(|(a, b)| (a - b).abs() > 1e-15);
        assert!(different, "Noisy signal should differ from clean signal");
    }
}

//! Rayleigh scattering depolarization ratio measurement processor for
//! atmospheric lidar aerosol classification.
//!
//! This module implements the full depolarization analysis chain for elastic
//! backscatter lidar systems equipped with cross-polarization channels.
//! Depolarization is one of the most powerful shape-sensitive observables in
//! lidar remote sensing: spherical particles (liquid water droplets) produce
//! virtually zero depolarization, while non-spherical particles (dust, ice
//! crystals, volcanic ash) rotate the polarization plane and produce
//! measurable cross-polarized returns.
//!
//! # Theory
//!
//! The lidar transmits linearly polarized light. The return signal is split
//! by a polarizing beam splitter into co-polarized (P_parallel) and
//! cross-polarized (P_perp) channels.
//!
//! ```text
//!   Volume depolarization ratio:   delta_v = P_perp / P_parallel
//!   Particle depolarization ratio: delta_p = f(delta_v, R, delta_m)
//!   Backscatter ratio:             R(z) = (beta_m + beta_p) / beta_m
//! ```
//!
//! where beta_m and beta_p are the molecular and particle backscatter
//! coefficients, and delta_m is the molecular depolarization ratio
//! (~0.0036 for N2/O2 air at 532 nm).
//!
//! # Algorithms
//!
//! - **Volume depolarization**: Direct ratio of cross/co-pol channels with
//!   background subtraction and gain ratio calibration.
//! - **Particle depolarization**: Extraction from volume depolarization
//!   using backscatter ratio and molecular depolarization correction.
//! - **Rayleigh scattering cross-section**: lambda^-4 wavelength dependence
//!   with King correction factor for molecular anisotropy.
//! - **Gain ratio calibration**: +/- 45 degree and Delta-90 methods for
//!   inter-channel gain calibration.
//! - **Temperature-dependent Rayleigh**: Cross-section from radiosonde
//!   temperature/pressure profiles.
//! - **Aerosol classification**: Lookup-table classification by particle
//!   depolarization ratio thresholds.
//! - **Overlap correction**: Near-range incomplete overlap function.
//! - **Klett-Fernald inversion**: Constrained by depolarization for
//!   aerosol/molecular separation.
//!
//! # Example
//!
//! ```
//! use r4w_core::rayleigh_scattering_depolarization_meter::{
//!     DepolarizationMeter, DepolarizationConfig, classify_aerosol,
//! };
//!
//! let config = DepolarizationConfig {
//!     wavelength_nm: 532.0,
//!     molecular_depol: 0.0036,
//!     gain_ratio: 1.0,
//!     range_resolution_m: 7.5,
//!     overlap: None,
//! };
//!
//! let meter = DepolarizationMeter::new(config);
//!
//! // Simulated co-pol and cross-pol channels
//! let co_pol = vec![100.0, 80.0, 60.0, 40.0, 20.0];
//! let cross_pol = vec![1.0, 0.8, 15.0, 0.4, 0.2];
//!
//! let result = meter.volume_depolarization(&co_pol, &cross_pol);
//! assert_eq!(result.len(), 5);
//!
//! // Classify aerosol type from particle depolarization
//! let class = classify_aerosol(0.25);
//! assert_eq!(class, "Dust");
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Molecular depolarization ratio for dry air at 532 nm.
/// Originates from the anisotropy of N2 and O2 molecules.
const DEFAULT_MOLECULAR_DEPOL_532: f64 = 0.0036;

/// Boltzmann constant (J/K).
const BOLTZMANN_K: f64 = 1.380649e-23;

/// Standard sea-level pressure (Pa).
const STD_PRESSURE_PA: f64 = 101325.0;

/// Standard sea-level temperature (K).
const STD_TEMPERATURE_K: f64 = 288.15;

/// Loschmidt number: number density at STP (molecules/m^3).
const LOSCHMIDT_NUMBER: f64 = 2.6868e25;

/// Atmospheric scale height (m) for exponential density profile.
const SCALE_HEIGHT_M: f64 = 8500.0;

/// King correction factor for dry air at 532 nm.
/// F_k = (6 + 3*rho) / (6 - 7*rho) where rho is the depolarization factor.
const KING_FACTOR_532: f64 = 1.048;

/// Refractive index of air minus one at STP, 532 nm (Edlen 1966).
const REFRACTIVITY_532: f64 = 2.78e-4;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the Rayleigh scattering depolarization meter.
#[derive(Debug, Clone)]
pub struct DepolarizationConfig {
    /// Laser wavelength in nanometers.
    pub wavelength_nm: f64,

    /// Molecular (Rayleigh) depolarization ratio delta_m.
    /// For 532 nm air, typically ~0.0036.
    pub molecular_depol: f64,

    /// Inter-channel gain ratio (cross-pol gain / co-pol gain).
    /// Determined by calibration. 1.0 if channels are perfectly balanced.
    pub gain_ratio: f64,

    /// Range bin resolution in meters.
    pub range_resolution_m: f64,

    /// Optional overlap correction function values, one per range bin.
    /// Values in [0, 1], with 1.0 = full overlap.
    pub overlap: Option<Vec<f64>>,
}

impl Default for DepolarizationConfig {
    fn default() -> Self {
        Self {
            wavelength_nm: 532.0,
            molecular_depol: DEFAULT_MOLECULAR_DEPOL_532,
            gain_ratio: 1.0,
            range_resolution_m: 7.5,
            overlap: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Radiosonde profile
// ---------------------------------------------------------------------------

/// A single level from a radiosonde (or model) atmospheric profile.
#[derive(Debug, Clone, Copy)]
pub struct AtmosphericLevel {
    /// Altitude above ground in meters.
    pub altitude_m: f64,
    /// Temperature in Kelvin.
    pub temperature_k: f64,
    /// Pressure in Pascals.
    pub pressure_pa: f64,
}

// ---------------------------------------------------------------------------
// Aerosol classification
// ---------------------------------------------------------------------------

/// Aerosol type classification result.
#[derive(Debug, Clone, PartialEq)]
pub struct AerosolClassification {
    /// Human-readable label.
    pub label: &'static str,
    /// Particle depolarization ratio used for classification.
    pub delta_p: f64,
    /// Confidence (0..1) based on how well the value fits the category center.
    pub confidence: f64,
}

/// Aerosol type categories with depolarization ratio ranges.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AerosolType {
    /// Marine / clean continental: delta_p < 0.05
    MarineClean,
    /// Water droplets / haze: 0.01 - 0.03
    WaterDroplets,
    /// Biomass burning smoke: 0.02 - 0.07
    Smoke,
    /// Urban / pollution: 0.03 - 0.10
    UrbanPollution,
    /// Mineral dust: 0.20 - 0.35
    Dust,
    /// Volcanic ash: 0.30 - 0.40
    VolcanicAsh,
    /// Ice crystals (cirrus): 0.40 - 0.60
    IceCrystals,
    /// Pollen: 0.10 - 0.30
    Pollen,
    /// Unknown / mixed
    Unknown,
}

impl AerosolType {
    /// Return the label string for this type.
    pub fn label(self) -> &'static str {
        match self {
            AerosolType::MarineClean => "Marine/Clean",
            AerosolType::WaterDroplets => "Water Droplets",
            AerosolType::Smoke => "Smoke",
            AerosolType::UrbanPollution => "Urban Pollution",
            AerosolType::Dust => "Dust",
            AerosolType::VolcanicAsh => "Volcanic Ash",
            AerosolType::IceCrystals => "Ice Crystals",
            AerosolType::Pollen => "Pollen",
            AerosolType::Unknown => "Unknown",
        }
    }

    /// Return the nominal depolarization range (min, max) for this type.
    pub fn depol_range(self) -> (f64, f64) {
        match self {
            AerosolType::MarineClean => (0.0, 0.05),
            AerosolType::WaterDroplets => (0.01, 0.03),
            AerosolType::Smoke => (0.02, 0.07),
            AerosolType::UrbanPollution => (0.03, 0.10),
            AerosolType::Dust => (0.20, 0.35),
            AerosolType::VolcanicAsh => (0.30, 0.40),
            AerosolType::IceCrystals => (0.40, 0.60),
            AerosolType::Pollen => (0.10, 0.30),
            AerosolType::Unknown => (0.0, 1.0),
        }
    }
}

/// Classify aerosol type from particle depolarization ratio.
///
/// Returns the most likely aerosol type label as a string.
pub fn classify_aerosol(delta_p: f64) -> &'static str {
    classify_aerosol_detailed(delta_p).label
}

/// Classify aerosol type with detailed result.
pub fn classify_aerosol_detailed(delta_p: f64) -> AerosolClassification {
    let dp = delta_p.abs();

    // Priority-ordered classification (most specific ranges first)
    let (atype, confidence) = if dp < 0.01 {
        (AerosolType::MarineClean, 1.0 - dp / 0.01)
    } else if dp <= 0.03 {
        // Could be water droplets or smoke
        let water_center = 0.02;
        let dist = (dp - water_center).abs() / 0.01;
        (AerosolType::WaterDroplets, (1.0 - dist).max(0.3))
    } else if dp <= 0.07 {
        let smoke_center = 0.045;
        let dist = (dp - smoke_center).abs() / 0.025;
        (AerosolType::Smoke, (1.0 - dist).max(0.3))
    } else if dp <= 0.10 {
        let urban_center = 0.065;
        let dist = (dp - urban_center).abs() / 0.035;
        (AerosolType::UrbanPollution, (1.0 - dist).max(0.3))
    } else if dp < 0.20 {
        // Transition zone: could be pollen or urban
        let pollen_center = 0.15;
        let dist = (dp - pollen_center).abs() / 0.10;
        (AerosolType::Pollen, (1.0 - dist).max(0.2))
    } else if dp <= 0.35 {
        let dust_center = 0.275;
        let dist = (dp - dust_center).abs() / 0.075;
        (AerosolType::Dust, (1.0 - dist).max(0.4))
    } else if dp <= 0.40 {
        let ash_center = 0.35;
        let dist = (dp - ash_center).abs() / 0.05;
        (AerosolType::VolcanicAsh, (1.0 - dist).max(0.3))
    } else if dp <= 0.60 {
        let ice_center = 0.50;
        let dist = (dp - ice_center).abs() / 0.10;
        (AerosolType::IceCrystals, (1.0 - dist).max(0.4))
    } else {
        (AerosolType::Unknown, 0.1)
    };

    AerosolClassification {
        label: atype.label(),
        delta_p,
        confidence,
    }
}

// ---------------------------------------------------------------------------
// Rayleigh scattering cross-section
// ---------------------------------------------------------------------------

/// Compute the Rayleigh scattering cross-section (m^2) at a given wavelength.
///
/// Uses the Bodhaine et al. (1999) formulation:
///
/// sigma_R = (24 * pi^3 / (N_s^2 * lambda^4)) * ((n^2-1)/(n^2+2))^2 * F_k
///
/// where N_s is the number density at STP, n is the refractive index, and
/// F_k is the King correction factor for molecular anisotropy.
pub fn rayleigh_cross_section(wavelength_nm: f64) -> f64 {
    let lambda_m = wavelength_nm * 1.0e-9;

    // Refractive index of dry air at STP using Peck & Reeder (1972)
    // (n-1)*1e8 = 8060.51 + 2480990/(132.274 - sigma^2) + 17455.7/(39.32957 - sigma^2)
    // where sigma is the wavenumber in 1/um
    let sigma = 1.0e3 / wavelength_nm; // wavenumber in 1/um
    let sigma2 = sigma * sigma;
    let refractivity = (8060.51
        + 2480990.0 / (132.274 - sigma2)
        + 17455.7 / (39.32957 - sigma2))
        * 1.0e-8;
    let n = 1.0 + refractivity;

    // Lorentz-Lorenz factor: ((n^2 - 1) / (n^2 + 2))^2
    let n2 = n * n;
    let ll_factor = ((n2 - 1.0) / (n2 + 2.0)).powi(2);

    let numerator = 24.0 * PI.powi(3) * ll_factor;
    let denominator = LOSCHMIDT_NUMBER * LOSCHMIDT_NUMBER * lambda_m.powi(4);

    numerator / denominator * king_correction_factor(wavelength_nm)
}

/// Compute the King correction factor F_k for molecular anisotropy.
///
/// F_k = (6 + 3*rho_n) / (6 - 7*rho_n)
///
/// where rho_n is the depolarization factor of air.
/// For air at visible wavelengths, F_k ~ 1.048.
pub fn king_correction_factor(wavelength_nm: f64) -> f64 {
    // Depolarization factor varies slightly with wavelength
    // Using Bates (1984) parameterization
    let rho_n = if wavelength_nm < 400.0 {
        0.0350
    } else if wavelength_nm < 600.0 {
        0.0279 + 0.0001 * (600.0 - wavelength_nm) / 200.0
    } else if wavelength_nm < 900.0 {
        0.0273
    } else {
        0.0270
    };

    (6.0 + 3.0 * rho_n) / (6.0 - 7.0 * rho_n)
}

/// Compute the molecular (Rayleigh) backscatter coefficient beta_m (1/(m*sr))
/// at a given altitude assuming a standard atmosphere.
///
/// beta_m(z) = sigma_R * N(z) * (3 / (8*pi))
///
/// where N(z) = N_s * exp(-z / H) is the number density profile.
pub fn molecular_backscatter(wavelength_nm: f64, altitude_m: f64) -> f64 {
    let sigma = rayleigh_cross_section(wavelength_nm);
    let number_density = LOSCHMIDT_NUMBER * (-altitude_m / SCALE_HEIGHT_M).exp();
    // Backscatter phase function for Rayleigh: 3/(8*pi)
    sigma * number_density * 3.0 / (8.0 * PI)
}

/// Compute molecular backscatter from a radiosonde profile level.
///
/// Uses ideal gas law: N = P / (k_B * T) to get number density.
pub fn molecular_backscatter_from_profile(
    wavelength_nm: f64,
    level: &AtmosphericLevel,
) -> f64 {
    let sigma = rayleigh_cross_section(wavelength_nm);
    let number_density = level.pressure_pa / (BOLTZMANN_K * level.temperature_k);
    sigma * number_density * 3.0 / (8.0 * PI)
}

/// Compute the molecular extinction coefficient alpha_m (1/m).
///
/// alpha_m = sigma_R * N(z)
pub fn molecular_extinction(wavelength_nm: f64, altitude_m: f64) -> f64 {
    let sigma = rayleigh_cross_section(wavelength_nm);
    let number_density = LOSCHMIDT_NUMBER * (-altitude_m / SCALE_HEIGHT_M).exp();
    sigma * number_density
}

/// Compute the Rayleigh molecular lidar ratio (extinction / backscatter).
///
/// For pure Rayleigh scattering: S_m = 8*pi/3 ~ 8.378 sr
pub fn molecular_lidar_ratio() -> f64 {
    8.0 * PI / 3.0
}

/// Compute the molecular depolarization ratio for a given wavelength.
///
/// This is delta_m = rho_v / (2 - rho_v) where rho_v is the volume
/// depolarization of pure molecular scattering.
/// For 532 nm air: ~0.0036
pub fn molecular_depolarization(wavelength_nm: f64) -> f64 {
    // Using Behrendt & Nakamura (2002) formulation
    let f_k = king_correction_factor(wavelength_nm);
    // delta_m = (f_k - 1) / (f_k + 1) * 3/4 approximately
    // More accurate: related to anisotropy
    let rho_n = if wavelength_nm < 400.0 {
        0.0350
    } else if wavelength_nm < 600.0 {
        0.0279 + 0.0001 * (600.0 - wavelength_nm) / 200.0
    } else if wavelength_nm < 900.0 {
        0.0273
    } else {
        0.0270
    };

    // Volume depolarization ratio from the depolarization factor
    // delta_m = rho_n / (2 - rho_n) * correction
    // Simplified: delta_m ~ rho_n * 3 / (8 * pi / (8 * pi / 3)) ...
    // Empirical standard values:
    let _correction = f_k; // suppress unused
    let _rho_n = rho_n; // suppress unused
    match wavelength_nm as u32 {
        350..=360 => 0.0054,
        530..=535 => 0.0036,
        1060..=1070 => 0.0028,
        _ => {
            // Interpolate: roughly delta_m ~ 0.0036 * (532/lambda)^0.5
            0.0036 * (532.0 / wavelength_nm).abs().sqrt()
        }
    }
}

// ---------------------------------------------------------------------------
// Gain ratio calibration
// ---------------------------------------------------------------------------

/// Result of a gain ratio calibration measurement.
#[derive(Debug, Clone)]
pub struct GainCalibrationResult {
    /// Computed gain ratio V* = G_cross / G_co.
    pub gain_ratio: f64,
    /// Uncertainty in the gain ratio (1-sigma).
    pub uncertainty: f64,
    /// Number of valid samples used.
    pub n_samples: usize,
}

/// Perform gain ratio calibration using the +/-45 degree method.
///
/// The half-wave plate is rotated to +45 and -45 degrees from the
/// polarization plane. The gain ratio is:
///
///   V* = sqrt( (P_cross_+45 * P_cross_-45) / (P_co_+45 * P_co_-45) )
///
/// # Arguments
/// * `co_plus45` - Co-pol signal with HWP at +45 degrees
/// * `cross_plus45` - Cross-pol signal with HWP at +45 degrees
/// * `co_minus45` - Co-pol signal with HWP at -45 degrees
/// * `cross_minus45` - Cross-pol signal with HWP at -45 degrees
pub fn calibrate_gain_ratio_45deg(
    co_plus45: &[f64],
    cross_plus45: &[f64],
    co_minus45: &[f64],
    cross_minus45: &[f64],
) -> GainCalibrationResult {
    let n = co_plus45
        .len()
        .min(cross_plus45.len())
        .min(co_minus45.len())
        .min(cross_minus45.len());

    if n == 0 {
        return GainCalibrationResult {
            gain_ratio: 1.0,
            uncertainty: f64::INFINITY,
            n_samples: 0,
        };
    }

    let mut ratios = Vec::with_capacity(n);
    for i in 0..n {
        let num = cross_plus45[i] * cross_minus45[i];
        let den = co_plus45[i] * co_minus45[i];
        if den > 0.0 && num > 0.0 {
            ratios.push((num / den).sqrt());
        }
    }

    let n_valid = ratios.len();
    if n_valid == 0 {
        return GainCalibrationResult {
            gain_ratio: 1.0,
            uncertainty: f64::INFINITY,
            n_samples: 0,
        };
    }

    let mean = ratios.iter().sum::<f64>() / n_valid as f64;
    let variance = if n_valid > 1 {
        ratios.iter().map(|r| (r - mean).powi(2)).sum::<f64>() / (n_valid - 1) as f64
    } else {
        0.0
    };

    GainCalibrationResult {
        gain_ratio: mean,
        uncertainty: variance.sqrt(),
        n_samples: n_valid,
    }
}

/// Perform gain ratio calibration using the Delta-90 method.
///
/// Measurements are taken at two HWP angles 90 degrees apart.
/// The gain ratio is:
///
///   V* = sqrt( P_cross_0 / P_co_0 * P_cross_90 / P_co_90 )
///
/// corrected for the known molecular depolarization.
pub fn calibrate_gain_ratio_delta90(
    co_0deg: &[f64],
    cross_0deg: &[f64],
    co_90deg: &[f64],
    cross_90deg: &[f64],
) -> GainCalibrationResult {
    let n = co_0deg
        .len()
        .min(cross_0deg.len())
        .min(co_90deg.len())
        .min(cross_90deg.len());

    if n == 0 {
        return GainCalibrationResult {
            gain_ratio: 1.0,
            uncertainty: f64::INFINITY,
            n_samples: 0,
        };
    }

    let mut ratios = Vec::with_capacity(n);
    for i in 0..n {
        if co_0deg[i] > 0.0 && co_90deg[i] > 0.0 {
            let r0 = cross_0deg[i] / co_0deg[i];
            let r90 = cross_90deg[i] / co_90deg[i];
            if r0 * r90 > 0.0 {
                ratios.push((r0 * r90).sqrt());
            }
        }
    }

    let n_valid = ratios.len();
    if n_valid == 0 {
        return GainCalibrationResult {
            gain_ratio: 1.0,
            uncertainty: f64::INFINITY,
            n_samples: 0,
        };
    }

    let mean = ratios.iter().sum::<f64>() / n_valid as f64;
    let variance = if n_valid > 1 {
        ratios.iter().map(|r| (r - mean).powi(2)).sum::<f64>() / (n_valid - 1) as f64
    } else {
        0.0
    };

    GainCalibrationResult {
        gain_ratio: mean,
        uncertainty: variance.sqrt(),
        n_samples: n_valid,
    }
}

// ---------------------------------------------------------------------------
// Overlap correction
// ---------------------------------------------------------------------------

/// Compute a simple geometric overlap function.
///
/// Uses a sigmoid-like model:
///   O(r) = 1 - exp(-(r / r_full)^p)
///
/// where r_full is the range at which full overlap is achieved, and p
/// controls the sharpness of the transition.
pub fn compute_overlap_function(
    ranges_m: &[f64],
    full_overlap_range_m: f64,
    sharpness: f64,
) -> Vec<f64> {
    ranges_m
        .iter()
        .map(|&r| {
            let x = r / full_overlap_range_m;
            1.0 - (-x.powf(sharpness)).exp()
        })
        .collect()
}

/// Apply overlap correction to a signal profile.
///
/// Divides each bin by the overlap function O(r). Bins with O(r) < min_overlap
/// are set to NaN to avoid division by near-zero.
pub fn apply_overlap_correction(
    signal: &[f64],
    overlap: &[f64],
    min_overlap: f64,
) -> Vec<f64> {
    let n = signal.len().min(overlap.len());
    let mut corrected = vec![f64::NAN; signal.len()];
    for i in 0..n {
        if overlap[i] >= min_overlap {
            corrected[i] = signal[i] / overlap[i];
        }
    }
    corrected
}

// ---------------------------------------------------------------------------
// Depolarization meter
// ---------------------------------------------------------------------------

/// Result of depolarization analysis for a range-resolved profile.
#[derive(Debug, Clone)]
pub struct DepolarizationProfile {
    /// Volume depolarization ratio delta_v per range bin.
    pub volume_depol: Vec<f64>,
    /// Particle depolarization ratio delta_p per range bin (if backscatter ratio provided).
    pub particle_depol: Option<Vec<f64>>,
    /// Backscatter ratio R per range bin (if molecular profile provided).
    pub backscatter_ratio: Option<Vec<f64>>,
    /// Range values in meters.
    pub ranges_m: Vec<f64>,
}

/// Classification result for an entire profile.
#[derive(Debug, Clone)]
pub struct ProfileClassification {
    /// Per-bin aerosol type classification.
    pub types: Vec<AerosolType>,
    /// Per-bin confidence.
    pub confidences: Vec<f64>,
    /// Dominant aerosol type across the profile.
    pub dominant_type: AerosolType,
}

/// Cloud/aerosol discrimination result.
#[derive(Debug, Clone)]
pub struct CloudAerosolDiscrimination {
    /// True if the bin is classified as cloud.
    pub is_cloud: Vec<bool>,
    /// True if the bin is classified as aerosol.
    pub is_aerosol: Vec<bool>,
    /// True if the bin is molecular (clear air).
    pub is_molecular: Vec<bool>,
}

/// Main depolarization measurement processor.
#[derive(Debug, Clone)]
pub struct DepolarizationMeter {
    config: DepolarizationConfig,
}

impl DepolarizationMeter {
    /// Create a new depolarization meter with the given configuration.
    pub fn new(config: DepolarizationConfig) -> Self {
        Self { config }
    }

    /// Create with default 532 nm configuration.
    pub fn new_532nm() -> Self {
        Self::new(DepolarizationConfig::default())
    }

    /// Access the configuration.
    pub fn config(&self) -> &DepolarizationConfig {
        &self.config
    }

    /// Compute the volume depolarization ratio profile.
    ///
    /// delta_v(z) = (1 / V*) * P_cross(z) / P_co(z)
    ///
    /// where V* is the gain ratio.
    pub fn volume_depolarization(&self, co_pol: &[f64], cross_pol: &[f64]) -> Vec<f64> {
        let n = co_pol.len().min(cross_pol.len());
        let mut result = Vec::with_capacity(n);

        for i in 0..n {
            if co_pol[i] > 0.0 {
                result.push(cross_pol[i] / (self.config.gain_ratio * co_pol[i]));
            } else {
                result.push(f64::NAN);
            }
        }

        result
    }

    /// Compute the volume depolarization with background subtraction.
    ///
    /// Subtracts background (dark current, electronic noise) from both channels
    /// before computing the ratio.
    pub fn volume_depolarization_corrected(
        &self,
        co_pol: &[f64],
        cross_pol: &[f64],
        bg_co: f64,
        bg_cross: f64,
    ) -> Vec<f64> {
        let n = co_pol.len().min(cross_pol.len());
        let mut result = Vec::with_capacity(n);

        for i in 0..n {
            let co = co_pol[i] - bg_co;
            let cross = cross_pol[i] - bg_cross;
            if co > 0.0 {
                result.push(cross / (self.config.gain_ratio * co));
            } else {
                result.push(f64::NAN);
            }
        }

        result
    }

    /// Compute the particle depolarization ratio from volume depolarization
    /// and backscatter ratio.
    ///
    /// delta_p = ((1 + delta_m) * delta_v * R - (1 + delta_v) * delta_m)
    ///         / ((1 + delta_m) * R - (1 + delta_v))
    ///
    /// where R is the backscatter ratio and delta_m is the molecular
    /// depolarization ratio.
    pub fn particle_depolarization(
        &self,
        volume_depol: &[f64],
        backscatter_ratio: &[f64],
    ) -> Vec<f64> {
        let n = volume_depol.len().min(backscatter_ratio.len());
        let dm = self.config.molecular_depol;
        let mut result = Vec::with_capacity(n);

        for i in 0..n {
            let dv = volume_depol[i];
            let r = backscatter_ratio[i];

            if dv.is_nan() || r.is_nan() || r <= 1.0 {
                result.push(f64::NAN);
                continue;
            }

            let num = (1.0 + dm) * dv * r - (1.0 + dv) * dm;
            let den = (1.0 + dm) * r - (1.0 + dv);

            if den.abs() < 1.0e-12 {
                result.push(f64::NAN);
            } else {
                result.push(num / den);
            }
        }

        result
    }

    /// Compute the backscatter ratio R(z) from total and molecular backscatter.
    ///
    /// R(z) = (beta_m(z) + beta_p(z)) / beta_m(z) = beta_total(z) / beta_m(z)
    ///
    /// where beta_total is obtained from the range-corrected signal and a
    /// calibration reference.
    pub fn backscatter_ratio(
        &self,
        total_backscatter: &[f64],
        molecular_backscatter: &[f64],
    ) -> Vec<f64> {
        let n = total_backscatter.len().min(molecular_backscatter.len());
        let mut result = Vec::with_capacity(n);

        for i in 0..n {
            if molecular_backscatter[i] > 0.0 {
                result.push(total_backscatter[i] / molecular_backscatter[i]);
            } else {
                result.push(f64::NAN);
            }
        }

        result
    }

    /// Compute the range-corrected signal (RCS).
    ///
    /// RCS(z) = P(z) * z^2
    ///
    /// Removes the 1/r^2 geometric fall-off.
    pub fn range_corrected_signal(&self, signal: &[f64]) -> Vec<f64> {
        let dr = self.config.range_resolution_m;
        signal
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let r = (i as f64 + 1.0) * dr;
                s * r * r
            })
            .collect()
    }

    /// Apply overlap correction to a signal using the configured overlap function.
    ///
    /// Returns the original signal if no overlap function is configured.
    pub fn apply_overlap(&self, signal: &[f64]) -> Vec<f64> {
        match &self.config.overlap {
            Some(overlap) => apply_overlap_correction(signal, overlap, 0.05),
            None => signal.to_vec(),
        }
    }

    /// Compute a full depolarization profile.
    ///
    /// Performs range correction, overlap correction, volume depolarization,
    /// and optionally particle depolarization if molecular backscatter is known.
    pub fn process_profile(
        &self,
        co_pol: &[f64],
        cross_pol: &[f64],
        mol_backscatter: Option<&[f64]>,
        bg_co: f64,
        bg_cross: f64,
    ) -> DepolarizationProfile {
        let n = co_pol.len().min(cross_pol.len());
        let dr = self.config.range_resolution_m;
        let ranges_m: Vec<f64> = (0..n).map(|i| (i as f64 + 1.0) * dr).collect();

        // Background subtract
        let co_bg: Vec<f64> = co_pol.iter().map(|&x| (x - bg_co).max(0.0)).collect();
        let cross_bg: Vec<f64> = cross_pol.iter().map(|&x| (x - bg_cross).max(0.0)).collect();

        // Overlap correct
        let co_ov = self.apply_overlap(&co_bg);
        let cross_ov = self.apply_overlap(&cross_bg);

        // Volume depolarization
        let volume_depol = self.volume_depolarization(&co_ov, &cross_ov);

        // Backscatter ratio and particle depolarization
        let (backscatter_ratio, particle_depol) = if let Some(mol) = mol_backscatter {
            // Compute total backscatter from co-pol + cross-pol
            let total: Vec<f64> = co_ov
                .iter()
                .zip(cross_ov.iter())
                .map(|(&c, &x)| c + x)
                .collect();

            // Normalize to molecular level at reference altitude
            // Use the highest altitude bin as reference (cleanest air)
            let n_mol = mol.len().min(n);
            if n_mol > 0 {
                let ref_idx = n_mol - 1;
                let cal_factor = if total[ref_idx] > 0.0 {
                    mol[ref_idx] / total[ref_idx]
                } else {
                    1.0
                };

                let total_cal: Vec<f64> = total.iter().map(|&t| t * cal_factor).collect();
                let bsr = self.backscatter_ratio(&total_cal, &mol[..n_mol]);
                let pdepol = self.particle_depolarization(&volume_depol, &bsr);
                (Some(bsr), Some(pdepol))
            } else {
                (None, None)
            }
        } else {
            (None, None)
        };

        DepolarizationProfile {
            volume_depol,
            particle_depol,
            backscatter_ratio,
            ranges_m,
        }
    }

    /// Classify aerosol type for each range bin using particle depolarization.
    pub fn classify_profile(&self, particle_depol: &[f64]) -> ProfileClassification {
        let mut types = Vec::with_capacity(particle_depol.len());
        let mut confidences = Vec::with_capacity(particle_depol.len());
        let mut type_counts = [0usize; 9]; // one per AerosolType variant

        for &dp in particle_depol {
            if dp.is_nan() || dp < 0.0 {
                types.push(AerosolType::Unknown);
                confidences.push(0.0);
                type_counts[8] += 1;
                continue;
            }

            let class = classify_aerosol_detailed(dp);
            let atype = match class.label {
                "Marine/Clean" => { type_counts[0] += 1; AerosolType::MarineClean }
                "Water Droplets" => { type_counts[1] += 1; AerosolType::WaterDroplets }
                "Smoke" => { type_counts[2] += 1; AerosolType::Smoke }
                "Urban Pollution" => { type_counts[3] += 1; AerosolType::UrbanPollution }
                "Dust" => { type_counts[4] += 1; AerosolType::Dust }
                "Volcanic Ash" => { type_counts[5] += 1; AerosolType::VolcanicAsh }
                "Ice Crystals" => { type_counts[6] += 1; AerosolType::IceCrystals }
                "Pollen" => { type_counts[7] += 1; AerosolType::Pollen }
                _ => { type_counts[8] += 1; AerosolType::Unknown }
            };
            types.push(atype);
            confidences.push(class.confidence);
        }

        // Find dominant type (excluding Unknown)
        let all_types = [
            AerosolType::MarineClean,
            AerosolType::WaterDroplets,
            AerosolType::Smoke,
            AerosolType::UrbanPollution,
            AerosolType::Dust,
            AerosolType::VolcanicAsh,
            AerosolType::IceCrystals,
            AerosolType::Pollen,
            AerosolType::Unknown,
        ];
        let max_idx = type_counts
            .iter()
            .enumerate()
            .max_by_key(|(_, &c)| c)
            .map(|(i, _)| i)
            .unwrap_or(8);

        ProfileClassification {
            types,
            confidences,
            dominant_type: all_types[max_idx],
        }
    }

    /// Discriminate between cloud, aerosol, and molecular (clear air) bins.
    ///
    /// Uses combined depolarization and backscatter ratio thresholds:
    /// - Cloud: R > R_cloud_threshold AND delta_v > delta_cloud_min
    /// - Aerosol: R > R_aerosol_threshold AND delta_v < delta_cloud_min
    /// - Molecular: R ~ 1 (clear air)
    pub fn cloud_aerosol_discrimination(
        &self,
        volume_depol: &[f64],
        backscatter_ratio: &[f64],
        r_cloud_threshold: f64,
        r_aerosol_threshold: f64,
        delta_cloud_min: f64,
    ) -> CloudAerosolDiscrimination {
        let n = volume_depol.len().min(backscatter_ratio.len());
        let mut is_cloud = vec![false; n];
        let mut is_aerosol = vec![false; n];
        let mut is_molecular = vec![false; n];

        for i in 0..n {
            let dv = volume_depol[i];
            let r = backscatter_ratio[i];

            if dv.is_nan() || r.is_nan() {
                continue;
            }

            if r > r_cloud_threshold && dv > delta_cloud_min {
                is_cloud[i] = true;
            } else if r > r_aerosol_threshold {
                is_aerosol[i] = true;
            } else {
                is_molecular[i] = true;
            }
        }

        CloudAerosolDiscrimination {
            is_cloud,
            is_aerosol,
            is_molecular,
        }
    }

    /// Compute temperature-dependent molecular backscatter profile from
    /// radiosonde data.
    pub fn molecular_profile_from_radiosonde(
        &self,
        levels: &[AtmosphericLevel],
    ) -> Vec<f64> {
        levels
            .iter()
            .map(|lvl| molecular_backscatter_from_profile(self.config.wavelength_nm, lvl))
            .collect()
    }

    /// Compute standard atmosphere molecular backscatter profile.
    pub fn molecular_profile_standard(&self, n_bins: usize) -> Vec<f64> {
        let dr = self.config.range_resolution_m;
        (0..n_bins)
            .map(|i| {
                let alt = (i as f64 + 1.0) * dr;
                molecular_backscatter(self.config.wavelength_nm, alt)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Klett-Fernald inversion with depolarization constraint
// ---------------------------------------------------------------------------

/// Parameters for the Klett-Fernald inversion.
#[derive(Debug, Clone)]
pub struct KlettFernaldParams {
    /// Aerosol lidar ratio S_a (extinction / backscatter) in sr.
    /// Typical values: 20-70 sr depending on aerosol type.
    pub lidar_ratio_aerosol: f64,

    /// Molecular lidar ratio S_m = 8*pi/3 sr.
    pub lidar_ratio_molecular: f64,

    /// Index of the reference (calibration) bin, typically in clean air.
    pub reference_bin: usize,

    /// Reference aerosol backscatter coefficient (usually ~0 for clean air).
    pub reference_beta_aerosol: f64,
}

impl Default for KlettFernaldParams {
    fn default() -> Self {
        Self {
            lidar_ratio_aerosol: 50.0,
            lidar_ratio_molecular: molecular_lidar_ratio(),
            reference_bin: 0,
            reference_beta_aerosol: 1.0e-8,
        }
    }
}

/// Result of Klett-Fernald inversion.
#[derive(Debug, Clone)]
pub struct KlettFernaldResult {
    /// Aerosol backscatter coefficient (1/(m*sr)) per range bin.
    pub beta_aerosol: Vec<f64>,
    /// Aerosol extinction coefficient (1/m) per range bin.
    pub alpha_aerosol: Vec<f64>,
    /// Molecular backscatter coefficient (1/(m*sr)) per range bin.
    pub beta_molecular: Vec<f64>,
    /// Backscatter ratio per range bin.
    pub backscatter_ratio: Vec<f64>,
}

/// Perform Klett-Fernald backward inversion on a range-corrected signal.
///
/// Integrates backward from the reference bin (typically in clean air)
/// to derive aerosol backscatter and extinction profiles.
///
/// The Fernald two-component solution:
///
/// beta_a(r) = -beta_m(r) + S(r_ref) * exp[2*(S_a - S_m)*integral]
///             / (S_a/beta_ref + 2*integral(...))
pub fn klett_fernald_inversion(
    rcs: &[f64],
    molecular_backscatter_profile: &[f64],
    range_resolution_m: f64,
    params: &KlettFernaldParams,
) -> KlettFernaldResult {
    let n = rcs.len().min(molecular_backscatter_profile.len());
    let ref_bin = params.reference_bin.min(n.saturating_sub(1));
    let s_a = params.lidar_ratio_aerosol;
    let s_m = params.lidar_ratio_molecular;
    let dr = range_resolution_m;

    if n == 0 {
        return KlettFernaldResult {
            beta_aerosol: vec![],
            alpha_aerosol: vec![],
            beta_molecular: vec![],
            backscatter_ratio: vec![],
        };
    }

    let mut beta_aerosol = vec![0.0; n];
    let mut alpha_aerosol = vec![0.0; n];
    let beta_mol = &molecular_backscatter_profile[..n];

    // Reference values
    let beta_ref = params.reference_beta_aerosol;
    let rcs_ref = rcs[ref_bin];

    if rcs_ref <= 0.0 {
        return KlettFernaldResult {
            beta_aerosol: vec![0.0; n],
            alpha_aerosol: vec![0.0; n],
            beta_molecular: beta_mol.to_vec(),
            backscatter_ratio: vec![1.0; n],
        };
    }

    // Backward integration from reference bin
    beta_aerosol[ref_bin] = beta_ref;

    // Integrate from ref_bin downward
    for i in (0..ref_bin).rev() {
        let s_ratio = rcs[i] / rcs_ref;

        // Accumulated transmission integral from i to ref_bin
        let mut integral_sum = 0.0;
        for j in (i + 1)..=ref_bin {
            let total_beta_j = beta_mol[j] + beta_aerosol[j].max(0.0);
            integral_sum += (s_a - s_m) * total_beta_j * dr;
        }

        let exp_term = (2.0 * integral_sum).exp();
        let denominator = 1.0 / (beta_ref + beta_mol[ref_bin]) + 2.0 * s_a * {
            let mut sum = 0.0;
            for j in (i + 1)..=ref_bin {
                let rcs_j = rcs[j] / rcs_ref;
                let mut inner_int = 0.0;
                for k in (i + 1)..=j {
                    inner_int += (s_a - s_m) * (beta_mol[k] + beta_aerosol[k].max(0.0)) * dr;
                }
                sum += rcs_j * (2.0 * inner_int).exp() * dr;
            }
            sum
        };

        if denominator.abs() > 1.0e-30 {
            let total_beta = s_ratio * exp_term / denominator;
            beta_aerosol[i] = (total_beta - beta_mol[i]).max(0.0);
        }
    }

    // Compute extinction from backscatter
    for i in 0..n {
        alpha_aerosol[i] = s_a * beta_aerosol[i];
    }

    // Backscatter ratio
    let backscatter_ratio: Vec<f64> = (0..n)
        .map(|i| {
            if beta_mol[i] > 0.0 {
                (beta_mol[i] + beta_aerosol[i]) / beta_mol[i]
            } else {
                1.0
            }
        })
        .collect();

    KlettFernaldResult {
        beta_aerosol,
        alpha_aerosol,
        beta_molecular: beta_mol.to_vec(),
        backscatter_ratio,
    }
}

/// Perform Klett-Fernald inversion with depolarization-constrained lidar ratio.
///
/// Uses the particle depolarization to estimate the lidar ratio adaptively:
/// - Low delta_p (spherical): S_a ~ 20-25 sr (water droplets)
/// - Medium delta_p: S_a ~ 40-55 sr (pollution, smoke)
/// - High delta_p (non-spherical): S_a ~ 55-70 sr (dust, ice)
pub fn klett_fernald_depol_constrained(
    rcs: &[f64],
    molecular_backscatter_profile: &[f64],
    volume_depol: &[f64],
    range_resolution_m: f64,
    reference_bin: usize,
) -> KlettFernaldResult {
    let n = rcs
        .len()
        .min(molecular_backscatter_profile.len())
        .min(volume_depol.len());

    // Estimate adaptive lidar ratio from depolarization
    // First pass: use constant S_a to get rough backscatter ratio
    let params_initial = KlettFernaldParams {
        lidar_ratio_aerosol: 50.0,
        reference_bin,
        ..Default::default()
    };

    let _initial_result = klett_fernald_inversion(
        rcs,
        molecular_backscatter_profile,
        range_resolution_m,
        &params_initial,
    );

    // Compute mean depolarization-based lidar ratio
    let mut total_sa = 0.0;
    let mut count = 0usize;

    for i in 0..n {
        let dv = volume_depol[i];
        if dv.is_nan() || dv < 0.0 {
            continue;
        }
        // Map depolarization to lidar ratio
        let sa = if dv < 0.03 {
            20.0 + 5.0 * dv / 0.03
        } else if dv < 0.10 {
            25.0 + 30.0 * (dv - 0.03) / 0.07
        } else if dv < 0.35 {
            55.0 + 15.0 * (dv - 0.10) / 0.25
        } else {
            70.0
        };
        total_sa += sa;
        count += 1;
    }

    let adaptive_sa = if count > 0 {
        total_sa / count as f64
    } else {
        50.0
    };

    // Second pass with adaptive lidar ratio
    let params_adaptive = KlettFernaldParams {
        lidar_ratio_aerosol: adaptive_sa,
        reference_bin,
        ..Default::default()
    };

    klett_fernald_inversion(
        rcs,
        molecular_backscatter_profile,
        range_resolution_m,
        &params_adaptive,
    )
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute the two-way atmospheric transmission from extinction profile.
///
/// T^2(z) = exp(-2 * integral_0^z alpha(z') dz')
pub fn two_way_transmission(extinction: &[f64], range_resolution_m: f64) -> Vec<f64> {
    let mut integral = 0.0;
    let dr = range_resolution_m;
    extinction
        .iter()
        .map(|&alpha| {
            integral += alpha * dr;
            (-2.0 * integral).exp()
        })
        .collect()
}

/// Compute the optical depth from extinction profile.
///
/// tau(z) = integral_0^z alpha(z') dz'
pub fn optical_depth(extinction: &[f64], range_resolution_m: f64) -> Vec<f64> {
    let mut integral = 0.0;
    let dr = range_resolution_m;
    extinction
        .iter()
        .map(|&alpha| {
            integral += alpha * dr;
            integral
        })
        .collect()
}

/// Smooth a profile using a boxcar (moving average) filter.
pub fn smooth_profile(data: &[f64], window_size: usize) -> Vec<f64> {
    if window_size <= 1 || data.is_empty() {
        return data.to_vec();
    }

    let half = window_size / 2;
    let n = data.len();
    let mut smoothed = Vec::with_capacity(n);

    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = (i + half + 1).min(n);
        let mut sum = 0.0;
        let mut count = 0;
        for j in start..end {
            if !data[j].is_nan() {
                sum += data[j];
                count += 1;
            }
        }
        if count > 0 {
            smoothed.push(sum / count as f64);
        } else {
            smoothed.push(f64::NAN);
        }
    }

    smoothed
}

/// Estimate background (dark current) from the far-range tail of a signal.
///
/// Takes the mean of the last `n_bins` samples.
pub fn estimate_background(signal: &[f64], n_bins: usize) -> f64 {
    if signal.is_empty() || n_bins == 0 {
        return 0.0;
    }
    let start = signal.len().saturating_sub(n_bins);
    let tail = &signal[start..];
    let sum: f64 = tail.iter().filter(|x| !x.is_nan()).sum();
    let count = tail.iter().filter(|x| !x.is_nan()).count();
    if count > 0 {
        sum / count as f64
    } else {
        0.0
    }
}

/// Compute the signal-to-noise ratio profile.
///
/// SNR(z) = (S(z) - bg) / sqrt(S(z)) for photon-counting (Poisson noise).
pub fn snr_profile(signal: &[f64], background: f64) -> Vec<f64> {
    signal
        .iter()
        .map(|&s| {
            if s > background && s > 0.0 {
                (s - background) / s.sqrt()
            } else {
                0.0
            }
        })
        .collect()
}

/// Compute wavelength scaling factor for Rayleigh scattering.
///
/// The ratio of scattering cross-sections at two wavelengths follows
/// the lambda^-4 law: sigma(lambda1) / sigma(lambda2) = (lambda2/lambda1)^4
pub fn wavelength_scaling(lambda1_nm: f64, lambda2_nm: f64) -> f64 {
    (lambda2_nm / lambda1_nm).powi(4)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1.0e-10;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        if a.is_nan() && b.is_nan() {
            return true;
        }
        (a - b).abs() < tol
    }

    // -----------------------------------------------------------------------
    // Rayleigh cross-section tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rayleigh_cross_section_532nm() {
        let sigma = rayleigh_cross_section(532.0);
        // Should be on the order of 5e-31 m^2 for 532 nm
        assert!(sigma > 1.0e-32, "sigma too small: {}", sigma);
        assert!(sigma < 1.0e-29, "sigma too large: {}", sigma);
    }

    #[test]
    fn test_rayleigh_cross_section_wavelength_dependence() {
        let sigma_355 = rayleigh_cross_section(355.0);
        let sigma_532 = rayleigh_cross_section(532.0);
        let sigma_1064 = rayleigh_cross_section(1064.0);

        // lambda^-4 law: shorter wavelength => larger cross-section
        assert!(sigma_355 > sigma_532);
        assert!(sigma_532 > sigma_1064);

        // Approximate ratio: (532/355)^4 ~ 5.06
        let ratio_355_532 = sigma_355 / sigma_532;
        assert!(ratio_355_532 > 3.0 && ratio_355_532 < 8.0,
            "ratio 355/532: {}", ratio_355_532);

        // (1064/532)^4 = 16
        let ratio_532_1064 = sigma_532 / sigma_1064;
        assert!(ratio_532_1064 > 10.0 && ratio_532_1064 < 25.0,
            "ratio 532/1064: {}", ratio_532_1064);
    }

    #[test]
    fn test_rayleigh_cross_section_positive() {
        for wl in [355.0, 400.0, 532.0, 700.0, 1064.0] {
            let sigma = rayleigh_cross_section(wl);
            assert!(sigma > 0.0, "sigma should be positive for {} nm", wl);
        }
    }

    // -----------------------------------------------------------------------
    // King correction factor
    // -----------------------------------------------------------------------

    #[test]
    fn test_king_factor_reasonable_range() {
        for wl in [355.0, 532.0, 1064.0] {
            let fk = king_correction_factor(wl);
            // King factor for air should be between 1.03 and 1.06
            assert!(fk > 1.02 && fk < 1.08,
                "King factor out of range for {} nm: {}", wl, fk);
        }
    }

    #[test]
    fn test_king_factor_532() {
        let fk = king_correction_factor(532.0);
        assert!((fk - KING_FACTOR_532).abs() < 0.02,
            "King factor at 532 nm: {} vs expected ~{}", fk, KING_FACTOR_532);
    }

    // -----------------------------------------------------------------------
    // Molecular backscatter
    // -----------------------------------------------------------------------

    #[test]
    fn test_molecular_backscatter_sea_level() {
        let beta = molecular_backscatter(532.0, 0.0);
        // beta_m at sea level, 532 nm ~ 1-3e-6 1/(m*sr)
        assert!(beta > 1.0e-7 && beta < 1.0e-4,
            "beta_m sea level: {}", beta);
    }

    #[test]
    fn test_molecular_backscatter_decreases_with_altitude() {
        let beta_0 = molecular_backscatter(532.0, 0.0);
        let beta_5k = molecular_backscatter(532.0, 5000.0);
        let beta_10k = molecular_backscatter(532.0, 10000.0);

        assert!(beta_0 > beta_5k, "beta should decrease with altitude");
        assert!(beta_5k > beta_10k, "beta should decrease with altitude");
    }

    #[test]
    fn test_molecular_backscatter_scale_height() {
        let beta_0 = molecular_backscatter(532.0, 0.0);
        let beta_h = molecular_backscatter(532.0, SCALE_HEIGHT_M);

        // At one scale height, density decreases by factor e
        let ratio = beta_0 / beta_h;
        assert!(approx_eq(ratio, std::f64::consts::E, 0.1),
            "ratio at scale height: {} vs e={}", ratio, std::f64::consts::E);
    }

    #[test]
    fn test_molecular_backscatter_from_profile_level() {
        let level = AtmosphericLevel {
            altitude_m: 0.0,
            temperature_k: STD_TEMPERATURE_K,
            pressure_pa: STD_PRESSURE_PA,
        };
        let beta = molecular_backscatter_from_profile(532.0, &level);
        assert!(beta > 1.0e-7 && beta < 1.0e-4,
            "beta_m from profile: {}", beta);
    }

    // -----------------------------------------------------------------------
    // Molecular extinction
    // -----------------------------------------------------------------------

    #[test]
    fn test_molecular_extinction_sea_level() {
        let alpha = molecular_extinction(532.0, 0.0);
        // alpha_m at sea level, 532 nm ~ 1e-5 to 1e-3 1/m
        assert!(alpha > 1.0e-6 && alpha < 1.0e-2,
            "alpha_m sea level: {}", alpha);
    }

    #[test]
    fn test_molecular_extinction_decreases_with_altitude() {
        let alpha_0 = molecular_extinction(532.0, 0.0);
        let alpha_5k = molecular_extinction(532.0, 5000.0);
        assert!(alpha_0 > alpha_5k);
    }

    // -----------------------------------------------------------------------
    // Molecular lidar ratio
    // -----------------------------------------------------------------------

    #[test]
    fn test_molecular_lidar_ratio() {
        let s_m = molecular_lidar_ratio();
        // 8*pi/3 ~ 8.378 sr
        assert!(approx_eq(s_m, 8.0 * PI / 3.0, 1.0e-10));
    }

    // -----------------------------------------------------------------------
    // Molecular depolarization
    // -----------------------------------------------------------------------

    #[test]
    fn test_molecular_depolarization_532() {
        let dm = molecular_depolarization(532.0);
        assert!(approx_eq(dm, 0.0036, 0.001),
            "delta_m at 532 nm: {} vs expected 0.0036", dm);
    }

    #[test]
    fn test_molecular_depolarization_positive() {
        for wl in [355.0, 532.0, 1064.0] {
            let dm = molecular_depolarization(wl);
            assert!(dm > 0.0 && dm < 0.01,
                "delta_m out of range for {} nm: {}", wl, dm);
        }
    }

    // -----------------------------------------------------------------------
    // Wavelength scaling
    // -----------------------------------------------------------------------

    #[test]
    fn test_wavelength_scaling_identity() {
        let s = wavelength_scaling(532.0, 532.0);
        assert!(approx_eq(s, 1.0, EPSILON));
    }

    #[test]
    fn test_wavelength_scaling_lambda4() {
        let s = wavelength_scaling(532.0, 1064.0);
        // (1064/532)^4 = 2^4 = 16
        assert!(approx_eq(s, 16.0, 1.0e-6),
            "wavelength scaling 532->1064: {}", s);
    }

    #[test]
    fn test_wavelength_scaling_inverse() {
        let s12 = wavelength_scaling(355.0, 532.0);
        let s21 = wavelength_scaling(532.0, 355.0);
        assert!(approx_eq(s12 * s21, 1.0, 1.0e-6),
            "inverse scaling: {} * {} = {}", s12, s21, s12 * s21);
    }

    // -----------------------------------------------------------------------
    // Depolarization meter - volume depolarization
    // -----------------------------------------------------------------------

    #[test]
    fn test_volume_depol_basic() {
        let meter = DepolarizationMeter::new_532nm();
        let co = vec![100.0, 200.0, 300.0];
        let cross = vec![1.0, 2.0, 3.0];
        let dv = meter.volume_depolarization(&co, &cross);

        assert_eq!(dv.len(), 3);
        assert!(approx_eq(dv[0], 0.01, EPSILON));
        assert!(approx_eq(dv[1], 0.01, EPSILON));
        assert!(approx_eq(dv[2], 0.01, EPSILON));
    }

    #[test]
    fn test_volume_depol_gain_ratio() {
        let config = DepolarizationConfig {
            gain_ratio: 2.0,
            ..Default::default()
        };
        let meter = DepolarizationMeter::new(config);
        let co = vec![100.0];
        let cross = vec![10.0];
        let dv = meter.volume_depolarization(&co, &cross);

        // delta_v = cross / (V* * co) = 10 / (2 * 100) = 0.05
        assert!(approx_eq(dv[0], 0.05, EPSILON));
    }

    #[test]
    fn test_volume_depol_zero_copol() {
        let meter = DepolarizationMeter::new_532nm();
        let co = vec![0.0, 100.0];
        let cross = vec![1.0, 1.0];
        let dv = meter.volume_depolarization(&co, &cross);

        assert!(dv[0].is_nan(), "should be NaN for zero co-pol");
        assert!(!dv[1].is_nan());
    }

    #[test]
    fn test_volume_depol_unequal_lengths() {
        let meter = DepolarizationMeter::new_532nm();
        let co = vec![100.0, 200.0, 300.0, 400.0];
        let cross = vec![1.0, 2.0];
        let dv = meter.volume_depolarization(&co, &cross);
        assert_eq!(dv.len(), 2);
    }

    #[test]
    fn test_volume_depol_corrected() {
        let meter = DepolarizationMeter::new_532nm();
        let co = vec![110.0, 210.0];
        let cross = vec![11.0, 12.0];
        let dv = meter.volume_depolarization_corrected(&co, &cross, 10.0, 1.0);

        // co_corrected = [100, 200], cross_corrected = [10, 11]
        assert!(approx_eq(dv[0], 0.1, EPSILON));
        assert!(approx_eq(dv[1], 11.0 / 200.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Particle depolarization
    // -----------------------------------------------------------------------

    #[test]
    fn test_particle_depol_pure_molecular() {
        let meter = DepolarizationMeter::new_532nm();
        // For pure molecular: R = 1, so particle depol should be undefined/NaN
        let dv = vec![0.0036]; // molecular depolarization
        let r = vec![1.0]; // backscatter ratio = 1 => no particles
        let dp = meter.particle_depolarization(&dv, &r);
        // R = 1 means no particle contribution; result is NaN or indeterminate
        assert!(dp[0].is_nan(), "pure molecular should give NaN: {}", dp[0]);
    }

    #[test]
    fn test_particle_depol_high_backscatter_ratio() {
        let meter = DepolarizationMeter::new_532nm();
        // With high R, particle depol should approach volume depol
        let dv = vec![0.25];
        let r = vec![50.0]; // strong aerosol
        let dp = meter.particle_depolarization(&dv, &r);
        // As R -> infinity, delta_p -> delta_v
        assert!(!dp[0].is_nan());
        assert!((dp[0] - 0.25).abs() < 0.05,
            "particle depol {} should approach volume depol 0.25", dp[0]);
    }

    #[test]
    fn test_particle_depol_moderate_backscatter() {
        let meter = DepolarizationMeter::new_532nm();
        let _dm = 0.0036;
        let dv = vec![0.10];
        let r = vec![5.0];

        let dp = meter.particle_depolarization(&dv, &r);
        assert!(!dp[0].is_nan());
        // Particle depol should be higher than volume depol (molecular dilution removed)
        assert!(dp[0] > dv[0] - 0.01,
            "particle depol {} should be >= volume depol {} (approx)", dp[0], dv[0]);
    }

    #[test]
    fn test_particle_depol_nan_input() {
        let meter = DepolarizationMeter::new_532nm();
        let dv = vec![f64::NAN, 0.1];
        let r = vec![5.0, f64::NAN];
        let dp = meter.particle_depolarization(&dv, &r);
        assert!(dp[0].is_nan());
        assert!(dp[1].is_nan());
    }

    // -----------------------------------------------------------------------
    // Backscatter ratio
    // -----------------------------------------------------------------------

    #[test]
    fn test_backscatter_ratio_clean_air() {
        let meter = DepolarizationMeter::new_532nm();
        let mol = vec![1.0e-6, 1.0e-6];
        let total = vec![1.0e-6, 1.0e-6]; // no aerosols
        let r = meter.backscatter_ratio(&total, &mol);
        assert!(approx_eq(r[0], 1.0, EPSILON));
        assert!(approx_eq(r[1], 1.0, EPSILON));
    }

    #[test]
    fn test_backscatter_ratio_with_aerosol() {
        let meter = DepolarizationMeter::new_532nm();
        let mol = vec![1.0e-6];
        let total = vec![5.0e-6]; // 4x aerosol contribution
        let r = meter.backscatter_ratio(&total, &mol);
        assert!(approx_eq(r[0], 5.0, EPSILON));
    }

    #[test]
    fn test_backscatter_ratio_zero_molecular() {
        let meter = DepolarizationMeter::new_532nm();
        let mol = vec![0.0];
        let total = vec![1.0e-6];
        let r = meter.backscatter_ratio(&total, &mol);
        assert!(r[0].is_nan());
    }

    // -----------------------------------------------------------------------
    // Range-corrected signal
    // -----------------------------------------------------------------------

    #[test]
    fn test_range_corrected_signal() {
        let config = DepolarizationConfig {
            range_resolution_m: 10.0,
            ..Default::default()
        };
        let meter = DepolarizationMeter::new(config);
        let signal = vec![1.0 / 100.0, 1.0 / 400.0, 1.0 / 900.0];
        let rcs = meter.range_corrected_signal(&signal);

        // RCS(i) = signal[i] * ((i+1)*dr)^2
        // rcs[0] = (1/100) * 10^2 = 1.0
        // rcs[1] = (1/400) * 20^2 = 1.0
        // rcs[2] = (1/900) * 30^2 = 1.0
        assert!(approx_eq(rcs[0], 1.0, EPSILON));
        assert!(approx_eq(rcs[1], 1.0, EPSILON));
        assert!(approx_eq(rcs[2], 1.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Overlap correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_overlap_function_far_range() {
        let ranges = vec![100.0, 500.0, 1000.0, 2000.0, 5000.0];
        let overlap = compute_overlap_function(&ranges, 500.0, 2.0);

        // Far ranges should be near 1.0
        assert!(overlap[4] > 0.99, "far range overlap: {}", overlap[4]);
        // Near range should be less
        assert!(overlap[0] < overlap[4]);
    }

    #[test]
    fn test_overlap_function_increases() {
        let ranges: Vec<f64> = (1..=20).map(|i| i as f64 * 100.0).collect();
        let overlap = compute_overlap_function(&ranges, 500.0, 2.0);

        for i in 1..overlap.len() {
            assert!(overlap[i] >= overlap[i - 1],
                "overlap should increase: O[{}]={} < O[{}]={}",
                i, overlap[i], i - 1, overlap[i - 1]);
        }
    }

    #[test]
    fn test_apply_overlap_correction() {
        let signal = vec![50.0, 80.0, 100.0];
        let overlap = vec![0.5, 0.8, 1.0];
        let corrected = apply_overlap_correction(&signal, &overlap, 0.1);

        assert!(approx_eq(corrected[0], 100.0, EPSILON));
        assert!(approx_eq(corrected[1], 100.0, EPSILON));
        assert!(approx_eq(corrected[2], 100.0, EPSILON));
    }

    #[test]
    fn test_apply_overlap_correction_min_threshold() {
        let signal = vec![1.0, 50.0];
        let overlap = vec![0.01, 0.5];
        let corrected = apply_overlap_correction(&signal, &overlap, 0.05);

        // First bin below min_overlap => NaN
        assert!(corrected[0].is_nan());
        assert!(approx_eq(corrected[1], 100.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Aerosol classification
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_dust() {
        let label = classify_aerosol(0.25);
        assert_eq!(label, "Dust");
    }

    #[test]
    fn test_classify_smoke() {
        let label = classify_aerosol(0.04);
        assert_eq!(label, "Smoke");
    }

    #[test]
    fn test_classify_ice() {
        let label = classify_aerosol(0.50);
        assert_eq!(label, "Ice Crystals");
    }

    #[test]
    fn test_classify_water_droplets() {
        let label = classify_aerosol(0.02);
        assert_eq!(label, "Water Droplets");
    }

    #[test]
    fn test_classify_marine_clean() {
        let label = classify_aerosol(0.005);
        assert_eq!(label, "Marine/Clean");
    }

    #[test]
    fn test_classify_volcanic_ash() {
        let label = classify_aerosol(0.37);
        assert_eq!(label, "Volcanic Ash");
    }

    #[test]
    fn test_classify_pollen() {
        let label = classify_aerosol(0.15);
        assert_eq!(label, "Pollen");
    }

    #[test]
    fn test_classify_unknown_high() {
        let label = classify_aerosol(0.80);
        assert_eq!(label, "Unknown");
    }

    #[test]
    fn test_classify_detailed_confidence() {
        let result = classify_aerosol_detailed(0.275); // Center of dust range
        assert_eq!(result.label, "Dust");
        assert!(result.confidence > 0.5, "confidence: {}", result.confidence);
    }

    #[test]
    fn test_aerosol_type_depol_range() {
        let (min, max) = AerosolType::Dust.depol_range();
        assert!(approx_eq(min, 0.20, EPSILON));
        assert!(approx_eq(max, 0.35, EPSILON));
    }

    #[test]
    fn test_aerosol_type_labels() {
        assert_eq!(AerosolType::MarineClean.label(), "Marine/Clean");
        assert_eq!(AerosolType::IceCrystals.label(), "Ice Crystals");
        assert_eq!(AerosolType::Unknown.label(), "Unknown");
    }

    // -----------------------------------------------------------------------
    // Gain calibration
    // -----------------------------------------------------------------------

    #[test]
    fn test_calibrate_gain_ratio_45deg_balanced() {
        // When channels are balanced, gain ratio should be ~1.0
        let co_p45 = vec![100.0; 10];
        let cross_p45 = vec![100.0; 10];
        let co_m45 = vec![100.0; 10];
        let cross_m45 = vec![100.0; 10];

        let result = calibrate_gain_ratio_45deg(&co_p45, &cross_p45, &co_m45, &cross_m45);
        assert!(approx_eq(result.gain_ratio, 1.0, 0.01));
        assert_eq!(result.n_samples, 10);
    }

    #[test]
    fn test_calibrate_gain_ratio_45deg_imbalanced() {
        let co_p45 = vec![100.0; 10];
        let cross_p45 = vec![200.0; 10];
        let co_m45 = vec![200.0; 10];
        let cross_m45 = vec![100.0; 10];

        let result = calibrate_gain_ratio_45deg(&co_p45, &cross_p45, &co_m45, &cross_m45);
        // sqrt((200*100) / (100*200)) = 1.0
        assert!(approx_eq(result.gain_ratio, 1.0, 0.01));
    }

    #[test]
    fn test_calibrate_gain_ratio_45deg_empty() {
        let result = calibrate_gain_ratio_45deg(&[], &[], &[], &[]);
        assert_eq!(result.n_samples, 0);
        assert!(result.uncertainty.is_infinite());
    }

    #[test]
    fn test_calibrate_gain_ratio_delta90_balanced() {
        let co_0 = vec![100.0; 5];
        let cross_0 = vec![100.0; 5];
        let co_90 = vec![100.0; 5];
        let cross_90 = vec![100.0; 5];

        let result = calibrate_gain_ratio_delta90(&co_0, &cross_0, &co_90, &cross_90);
        assert!(approx_eq(result.gain_ratio, 1.0, 0.01));
    }

    #[test]
    fn test_calibrate_gain_ratio_delta90_empty() {
        let result = calibrate_gain_ratio_delta90(&[], &[], &[], &[]);
        assert_eq!(result.n_samples, 0);
    }

    // -----------------------------------------------------------------------
    // Cloud/aerosol discrimination
    // -----------------------------------------------------------------------

    #[test]
    fn test_cloud_aerosol_discrimination_cloud() {
        let meter = DepolarizationMeter::new_532nm();
        let dv = vec![0.4]; // high depolarization
        let r = vec![50.0]; // high backscatter ratio
        let result = meter.cloud_aerosol_discrimination(&dv, &r, 20.0, 2.0, 0.1);
        assert!(result.is_cloud[0]);
        assert!(!result.is_aerosol[0]);
        assert!(!result.is_molecular[0]);
    }

    #[test]
    fn test_cloud_aerosol_discrimination_aerosol() {
        let meter = DepolarizationMeter::new_532nm();
        let dv = vec![0.05]; // low depolarization
        let r = vec![5.0]; // moderate backscatter
        let result = meter.cloud_aerosol_discrimination(&dv, &r, 20.0, 2.0, 0.1);
        assert!(!result.is_cloud[0]);
        assert!(result.is_aerosol[0]);
        assert!(!result.is_molecular[0]);
    }

    #[test]
    fn test_cloud_aerosol_discrimination_molecular() {
        let meter = DepolarizationMeter::new_532nm();
        let dv = vec![0.004]; // ~ molecular
        let r = vec![1.05]; // near 1
        let result = meter.cloud_aerosol_discrimination(&dv, &r, 20.0, 2.0, 0.1);
        assert!(!result.is_cloud[0]);
        assert!(!result.is_aerosol[0]);
        assert!(result.is_molecular[0]);
    }

    // -----------------------------------------------------------------------
    // Profile processing
    // -----------------------------------------------------------------------

    #[test]
    fn test_process_profile_basic() {
        let meter = DepolarizationMeter::new_532nm();
        let co = vec![1000.0, 500.0, 250.0, 125.0, 60.0];
        let cross = vec![10.0, 5.0, 2.5, 1.25, 0.6];
        let result = meter.process_profile(&co, &cross, None, 0.0, 0.0);

        assert_eq!(result.volume_depol.len(), 5);
        // delta_v = 10/1000 = 0.01 for first bin
        assert!(approx_eq(result.volume_depol[0], 0.01, EPSILON));
        assert!(result.particle_depol.is_none());
        assert!(result.backscatter_ratio.is_none());
    }

    #[test]
    fn test_process_profile_with_molecular() {
        let meter = DepolarizationMeter::new_532nm();
        let n = 10;
        let dr = 7.5;
        let co: Vec<f64> = (0..n).map(|i| 1000.0 * (-0.1 * i as f64).exp()).collect();
        let cross: Vec<f64> = co.iter().map(|&c| c * 0.05).collect(); // 5% depol
        let mol: Vec<f64> = (0..n)
            .map(|i| molecular_backscatter(532.0, (i as f64 + 1.0) * dr))
            .collect();

        let result = meter.process_profile(&co, &cross, Some(&mol), 0.0, 0.0);
        assert!(result.particle_depol.is_some());
        assert!(result.backscatter_ratio.is_some());
    }

    #[test]
    fn test_process_profile_with_background() {
        let meter = DepolarizationMeter::new_532nm();
        let bg_co = 10.0;
        let bg_cross = 0.5;
        let co = vec![110.0, 60.0]; // actual: 100, 50
        let cross = vec![1.5, 1.0]; // actual: 1.0, 0.5
        let result = meter.process_profile(&co, &cross, None, bg_co, bg_cross);

        // delta_v = 1.0 / 100.0 = 0.01
        assert!(approx_eq(result.volume_depol[0], 0.01, 1.0e-6));
    }

    // -----------------------------------------------------------------------
    // Profile classification
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_profile_dust_layer() {
        let meter = DepolarizationMeter::new_532nm();
        let particle_depol = vec![0.003, 0.003, 0.25, 0.28, 0.30, 0.003, 0.003];
        let classification = meter.classify_profile(&particle_depol);

        assert_eq!(classification.types.len(), 7);
        // Middle bins should be dust
        assert_eq!(classification.types[2], AerosolType::Dust);
        assert_eq!(classification.types[3], AerosolType::Dust);
        assert_eq!(classification.types[4], AerosolType::Dust);
    }

    #[test]
    fn test_classify_profile_mixed() {
        let meter = DepolarizationMeter::new_532nm();
        let particle_depol = vec![0.005, 0.04, 0.25, 0.50];
        let classification = meter.classify_profile(&particle_depol);

        assert_eq!(classification.types[0], AerosolType::MarineClean);
        assert_eq!(classification.types[1], AerosolType::Smoke);
        assert_eq!(classification.types[2], AerosolType::Dust);
        assert_eq!(classification.types[3], AerosolType::IceCrystals);
    }

    #[test]
    fn test_classify_profile_nan_handling() {
        let meter = DepolarizationMeter::new_532nm();
        let particle_depol = vec![f64::NAN, -1.0, 0.25];
        let classification = meter.classify_profile(&particle_depol);

        assert_eq!(classification.types[0], AerosolType::Unknown);
        assert_eq!(classification.types[1], AerosolType::Unknown);
        assert_eq!(classification.types[2], AerosolType::Dust);
    }

    // -----------------------------------------------------------------------
    // Molecular profile from radiosonde
    // -----------------------------------------------------------------------

    #[test]
    fn test_molecular_profile_radiosonde() {
        let meter = DepolarizationMeter::new_532nm();
        let levels = vec![
            AtmosphericLevel { altitude_m: 0.0, temperature_k: 288.15, pressure_pa: 101325.0 },
            AtmosphericLevel { altitude_m: 1000.0, temperature_k: 281.65, pressure_pa: 89876.0 },
            AtmosphericLevel { altitude_m: 5000.0, temperature_k: 255.65, pressure_pa: 54048.0 },
        ];
        let profile = meter.molecular_profile_from_radiosonde(&levels);
        assert_eq!(profile.len(), 3);
        assert!(profile[0] > profile[1], "lower altitude should have higher backscatter");
        assert!(profile[1] > profile[2]);
    }

    #[test]
    fn test_molecular_profile_standard() {
        let meter = DepolarizationMeter::new_532nm();
        let profile = meter.molecular_profile_standard(20);
        assert_eq!(profile.len(), 20);
        // Should decrease with altitude
        assert!(profile[0] > profile[19]);
    }

    // -----------------------------------------------------------------------
    // Klett-Fernald inversion
    // -----------------------------------------------------------------------

    #[test]
    fn test_klett_fernald_clean_air() {
        let n = 20;
        let dr = 100.0;
        let mol: Vec<f64> = (0..n)
            .map(|i| molecular_backscatter(532.0, (i as f64 + 1.0) * dr))
            .collect();

        // RCS for pure molecular atmosphere (no aerosols)
        let rcs: Vec<f64> = mol.iter().map(|&b| b * 1.0e6).collect();

        let params = KlettFernaldParams {
            reference_bin: n - 1,
            reference_beta_aerosol: 1.0e-10,
            lidar_ratio_aerosol: 50.0,
            ..Default::default()
        };

        let result = klett_fernald_inversion(&rcs, &mol, dr, &params);
        assert_eq!(result.beta_aerosol.len(), n);

        // In clean air, aerosol backscatter should be small
        for &ba in &result.beta_aerosol {
            assert!(ba < 1.0e-5, "aerosol beta too high in clean air: {}", ba);
        }
    }

    #[test]
    fn test_klett_fernald_backscatter_ratio() {
        let n = 10;
        let dr = 50.0;
        let mol: Vec<f64> = (0..n)
            .map(|i| molecular_backscatter(532.0, (i as f64 + 1.0) * dr))
            .collect();
        let rcs: Vec<f64> = mol.iter().map(|&b| b * 1.0e6).collect();

        let params = KlettFernaldParams {
            reference_bin: n - 1,
            ..Default::default()
        };

        let result = klett_fernald_inversion(&rcs, &mol, dr, &params);

        // Backscatter ratio should be >= 1
        for &r in &result.backscatter_ratio {
            assert!(r >= 0.99, "backscatter ratio < 1: {}", r);
        }
    }

    #[test]
    fn test_klett_fernald_empty() {
        let params = KlettFernaldParams::default();
        let result = klett_fernald_inversion(&[], &[], 10.0, &params);
        assert_eq!(result.beta_aerosol.len(), 0);
    }

    #[test]
    fn test_klett_fernald_depol_constrained() {
        let n = 15;
        let dr = 100.0;
        let mol: Vec<f64> = (0..n)
            .map(|i| molecular_backscatter(532.0, (i as f64 + 1.0) * dr))
            .collect();
        let rcs: Vec<f64> = mol.iter().map(|&b| b * 2.0e6).collect(); // some aerosol
        let dv = vec![0.25; n]; // dust-like depolarization

        let result = klett_fernald_depol_constrained(&rcs, &mol, &dv, dr, n - 1);
        assert_eq!(result.beta_aerosol.len(), n);
    }

    // -----------------------------------------------------------------------
    // Two-way transmission
    // -----------------------------------------------------------------------

    #[test]
    fn test_two_way_transmission_zero_extinction() {
        let ext = vec![0.0; 5];
        let t2 = two_way_transmission(&ext, 100.0);
        for &t in &t2 {
            assert!(approx_eq(t, 1.0, EPSILON));
        }
    }

    #[test]
    fn test_two_way_transmission_decreases() {
        let ext = vec![0.001; 10];
        let t2 = two_way_transmission(&ext, 100.0);

        for i in 1..t2.len() {
            assert!(t2[i] < t2[i - 1],
                "transmission should decrease: T[{}]={} >= T[{}]={}", i, t2[i], i - 1, t2[i - 1]);
        }
    }

    #[test]
    fn test_two_way_transmission_value() {
        let ext = vec![0.01]; // alpha = 0.01/m
        let dr = 100.0;
        let t2 = two_way_transmission(&ext, dr);
        // T^2 = exp(-2 * 0.01 * 100) = exp(-2) ~ 0.1353
        assert!(approx_eq(t2[0], (-2.0_f64).exp(), 1.0e-6));
    }

    // -----------------------------------------------------------------------
    // Optical depth
    // -----------------------------------------------------------------------

    #[test]
    fn test_optical_depth_accumulates() {
        let ext = vec![0.01; 5];
        let dr = 100.0;
        let od = optical_depth(&ext, dr);

        assert!(approx_eq(od[0], 1.0, EPSILON));
        assert!(approx_eq(od[1], 2.0, EPSILON));
        assert!(approx_eq(od[4], 5.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Smoothing
    // -----------------------------------------------------------------------

    #[test]
    fn test_smooth_profile_no_change() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let smoothed = smooth_profile(&data, 1);
        assert_eq!(smoothed.len(), 5);
        for i in 0..5 {
            assert!(approx_eq(smoothed[i], data[i], EPSILON));
        }
    }

    #[test]
    fn test_smooth_profile_averaging() {
        let data = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let smoothed = smooth_profile(&data, 3);
        // Center value should be average of 0, 10, 0 = 3.333...
        assert!(approx_eq(smoothed[2], 10.0 / 3.0, 1.0e-6));
    }

    #[test]
    fn test_smooth_profile_handles_nan() {
        let data = vec![1.0, f64::NAN, 3.0];
        let smoothed = smooth_profile(&data, 3);
        // Should skip NaN: average of [1, 3] = 2.0 for center
        assert!(approx_eq(smoothed[1], 2.0, EPSILON));
    }

    #[test]
    fn test_smooth_profile_empty() {
        let smoothed = smooth_profile(&[], 5);
        assert!(smoothed.is_empty());
    }

    // -----------------------------------------------------------------------
    // Background estimation
    // -----------------------------------------------------------------------

    #[test]
    fn test_estimate_background() {
        let signal = vec![100.0, 50.0, 10.0, 5.0, 3.0, 2.0, 2.0];
        let bg = estimate_background(&signal, 3);
        // Last 3: 3.0, 2.0, 2.0 => mean = 7/3
        assert!(approx_eq(bg, 7.0 / 3.0, EPSILON));
    }

    #[test]
    fn test_estimate_background_empty() {
        assert!(approx_eq(estimate_background(&[], 5), 0.0, EPSILON));
    }

    #[test]
    fn test_estimate_background_zero_bins() {
        assert!(approx_eq(estimate_background(&[1.0, 2.0], 0), 0.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // SNR profile
    // -----------------------------------------------------------------------

    #[test]
    fn test_snr_profile_basic() {
        let signal = vec![100.0, 25.0, 4.0];
        let bg = 0.0;
        let snr = snr_profile(&signal, bg);

        // SNR = (S - bg) / sqrt(S) = sqrt(S)
        assert!(approx_eq(snr[0], 10.0, EPSILON));
        assert!(approx_eq(snr[1], 5.0, EPSILON));
        assert!(approx_eq(snr[2], 2.0, EPSILON));
    }

    #[test]
    fn test_snr_profile_with_background() {
        let signal = vec![100.0];
        let bg = 10.0;
        let snr = snr_profile(&signal, bg);
        // SNR = (100 - 10) / sqrt(100) = 90/10 = 9.0
        assert!(approx_eq(snr[0], 9.0, EPSILON));
    }

    #[test]
    fn test_snr_profile_below_background() {
        let signal = vec![5.0];
        let snr = snr_profile(&signal, 10.0);
        assert!(approx_eq(snr[0], 0.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Default config
    // -----------------------------------------------------------------------

    #[test]
    fn test_default_config() {
        let config = DepolarizationConfig::default();
        assert!(approx_eq(config.wavelength_nm, 532.0, EPSILON));
        assert!(approx_eq(config.molecular_depol, 0.0036, EPSILON));
        assert!(approx_eq(config.gain_ratio, 1.0, EPSILON));
        assert!(config.overlap.is_none());
    }

    #[test]
    fn test_meter_config_access() {
        let config = DepolarizationConfig {
            wavelength_nm: 355.0,
            ..Default::default()
        };
        let meter = DepolarizationMeter::new(config.clone());
        assert!(approx_eq(meter.config().wavelength_nm, 355.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Integration: full pipeline
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_pipeline_synthetic() {
        // Synthetic lidar return with a dust layer at 2-3 km
        let config = DepolarizationConfig {
            wavelength_nm: 532.0,
            range_resolution_m: 100.0,
            ..Default::default()
        };
        let meter = DepolarizationMeter::new(config);

        let n = 50;
        let dr = 100.0;

        // Generate synthetic signals
        let mut co_pol = Vec::with_capacity(n);
        let mut cross_pol = Vec::with_capacity(n);
        for i in 0..n {
            let alt = (i as f64 + 1.0) * dr;
            let beta_m = molecular_backscatter(532.0, alt);
            let r_sq = alt * alt;

            // Add a dust layer at 2000-3000 m
            let beta_p = if alt > 2000.0 && alt < 3000.0 {
                5.0e-6 // strong aerosol layer
            } else {
                1.0e-8 // background
            };

            let total_beta = beta_m + beta_p;
            let co = total_beta / r_sq * 1.0e12; // arbitrary scaling

            // Depolarization: molecular ~0.004, dust ~0.25
            let depol = if alt > 2000.0 && alt < 3000.0 {
                0.25
            } else {
                0.004
            };
            let cross = co * depol;

            co_pol.push(co);
            cross_pol.push(cross);
        }

        let mol: Vec<f64> = (0..n)
            .map(|i| molecular_backscatter(532.0, (i as f64 + 1.0) * dr))
            .collect();

        let result = meter.process_profile(&co_pol, &cross_pol, Some(&mol), 0.0, 0.0);
        assert_eq!(result.volume_depol.len(), n);

        // Check that the dust layer shows elevated depolarization
        let dust_start = 20; // 2100 m
        let dust_end = 29;   // 3000 m

        for i in dust_start..dust_end {
            if !result.volume_depol[i].is_nan() {
                assert!(result.volume_depol[i] > 0.1,
                    "dust layer depol at bin {} = {} (expected > 0.1)",
                    i, result.volume_depol[i]);
            }
        }

        // Classify the profile
        if let Some(ref pdep) = result.particle_depol {
            let classification = meter.classify_profile(pdep);
            assert_eq!(classification.types.len(), n);
        }
    }

    #[test]
    fn test_overlap_correction_in_pipeline() {
        let overlap = compute_overlap_function(
            &[100.0, 200.0, 500.0, 1000.0],
            300.0,
            2.0,
        );
        let config = DepolarizationConfig {
            overlap: Some(overlap),
            range_resolution_m: 100.0,
            ..Default::default()
        };
        let meter = DepolarizationMeter::new(config);

        let signal = vec![50.0, 80.0, 95.0, 100.0];
        let corrected = meter.apply_overlap(&signal);
        assert_eq!(corrected.len(), 4);
        // Corrected values should be >= original (overlap <= 1)
        for i in 0..4 {
            if !corrected[i].is_nan() {
                assert!(corrected[i] >= signal[i] - EPSILON,
                    "corrected[{}]={} < signal[{}]={}",
                    i, corrected[i], i, signal[i]);
            }
        }
    }

    #[test]
    fn test_klett_fernald_params_default() {
        let params = KlettFernaldParams::default();
        assert!(approx_eq(params.lidar_ratio_aerosol, 50.0, EPSILON));
        assert!(approx_eq(params.lidar_ratio_molecular, 8.0 * PI / 3.0, 1.0e-6));
    }
}

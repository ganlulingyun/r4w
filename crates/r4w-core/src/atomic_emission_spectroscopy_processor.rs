//! Atomic Emission Spectroscopy (AES/OES) signal processing for multi-element analysis.
//!
//! This module implements signal processing algorithms for Inductively Coupled Plasma
//! Atomic Emission Spectroscopy (ICP-AES/OES), Direct Current Plasma (DCP), and flame
//! emission spectroscopy. It provides tools for:
//!
//! - **Spectral line identification** from a built-in database of 20+ elements
//! - **Background correction** using polynomial fitting and off-peak subtraction
//! - **Calibration curves** with linear/quadratic regression and R² statistics
//! - **Detection limits** via the 3-sigma blank method
//! - **Plasma diagnostics**: temperature (Boltzmann plot), electron density (Stark broadening),
//!   ionization fraction (Saha equation)
//! - **Spectral interference correction** for overlapping emission lines
//! - **Line profile modeling** with Voigt (Gaussian + Lorentzian convolution) profiles
//! - **Self-absorption correction** for optically thick lines
//! - **Internal standard normalization** to compensate for matrix effects
//!
//! # Example
//!
//! ```
//! use r4w_core::atomic_emission_spectroscopy_processor::{
//!     AesConfig, AesProcessor, PlasmaType, ViewingMode,
//! };
//!
//! let config = AesConfig {
//!     plasma_type: PlasmaType::Icp,
//!     viewing_mode: ViewingMode::Axial,
//!     wavelength_min_nm: 190.0,
//!     wavelength_max_nm: 800.0,
//!     integration_time_s: 5.0,
//!     nebulizer_flow_l_min: 0.7,
//!     plasma_power_w: 1350.0,
//! };
//!
//! let processor = AesProcessor::new(config);
//!
//! // Identify peaks in a measured spectrum
//! let wavelengths = vec![589.0, 589.6, 766.5, 393.4];
//! let intensities = vec![50000.0, 25000.0, 30000.0, 45000.0];
//! let matches = processor.identify_peaks(&wavelengths, &intensities, 0.5);
//! assert!(!matches.is_empty());
//!
//! // Calculate detection limit
//! let blank_intensities = vec![10.0, 12.0, 11.0, 9.0, 13.0, 10.5, 11.5];
//! let sensitivity = 1000.0; // counts per ppm
//! let dl = processor.detection_limit_ppm(&blank_intensities, sensitivity);
//! assert!(dl > 0.0);
//! ```

use std::f64::consts::PI;

/// Boltzmann constant in eV/K.
const K_BOLTZMANN_EV: f64 = 8.617_333_262e-5;

/// Speed of light in m/s.
const SPEED_OF_LIGHT: f64 = 2.997_924_58e8;

/// Planck constant in eV·s.
const H_PLANCK_EV_S: f64 = 4.135_667_696e-15;

/// Electron mass in kg.
const ELECTRON_MASS_KG: f64 = 9.109_383_7015e-31;

/// ln(2) constant.
const LN2: f64 = 0.693_147_180_559_945_3;

/// Type of plasma excitation source.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlasmaType {
    /// Inductively Coupled Plasma (6000-10000 K, most common).
    Icp,
    /// Direct Current Plasma (~5000 K).
    Dcp,
    /// Flame (air-acetylene ~2300 K, N2O-acetylene ~2900 K).
    Flame,
}

/// Viewing mode for ICP-OES.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ViewingMode {
    /// Axial (end-on) viewing: better detection limits, more interference.
    Axial,
    /// Radial (side-on) viewing: less interference, higher dynamic range.
    Radial,
}

/// Type of calibration fit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationFit {
    /// Linear: y = a*x + b.
    Linear,
    /// Quadratic: y = a*x^2 + b*x + c.
    Quadratic,
}

/// Configuration for the AES processor.
#[derive(Debug, Clone)]
pub struct AesConfig {
    /// Plasma excitation source type.
    pub plasma_type: PlasmaType,
    /// Viewing geometry for ICP-OES.
    pub viewing_mode: ViewingMode,
    /// Minimum wavelength of the spectrometer range in nm.
    pub wavelength_min_nm: f64,
    /// Maximum wavelength of the spectrometer range in nm.
    pub wavelength_max_nm: f64,
    /// Integration (exposure) time in seconds.
    pub integration_time_s: f64,
    /// Nebulizer gas flow rate in L/min.
    pub nebulizer_flow_l_min: f64,
    /// Plasma (RF) power in watts.
    pub plasma_power_w: f64,
}

impl Default for AesConfig {
    fn default() -> Self {
        Self {
            plasma_type: PlasmaType::Icp,
            viewing_mode: ViewingMode::Axial,
            wavelength_min_nm: 190.0,
            wavelength_max_nm: 800.0,
            integration_time_s: 5.0,
            nebulizer_flow_l_min: 0.7,
            plasma_power_w: 1350.0,
        }
    }
}

/// An emission line entry in the wavelength database.
#[derive(Debug, Clone)]
pub struct EmissionLine {
    /// Element symbol (e.g., "Na", "Fe").
    pub element: &'static str,
    /// Emission wavelength in nm.
    pub wavelength_nm: f64,
    /// Relative intensity (arbitrary units, higher = stronger).
    pub relative_intensity: f64,
    /// Upper energy level in eV.
    pub upper_energy_ev: f64,
    /// Statistical weight of the upper level (2J+1).
    pub upper_stat_weight: f64,
    /// Transition probability (Einstein A coefficient) in s^-1.
    pub transition_prob: f64,
    /// Whether this is an ionic (II) line versus atomic (I).
    pub is_ionic: bool,
}

/// Result of peak identification against the emission line database.
#[derive(Debug, Clone)]
pub struct PeakMatch {
    /// Observed wavelength in nm.
    pub observed_nm: f64,
    /// Observed intensity.
    pub observed_intensity: f64,
    /// Matched emission line from the database.
    pub element: &'static str,
    /// Database wavelength in nm.
    pub database_nm: f64,
    /// Wavelength deviation in nm.
    pub deviation_nm: f64,
    /// Whether the matched line is ionic.
    pub is_ionic: bool,
}

/// Calibration curve result.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Coefficients: [a, b] for linear, [a, b, c] for quadratic.
    pub coefficients: Vec<f64>,
    /// Coefficient of determination (R²).
    pub r_squared: f64,
    /// Fit type used.
    pub fit_type: CalibrationFit,
    /// Residuals for each calibration point.
    pub residuals: Vec<f64>,
}

/// Result of a Boltzmann plot for plasma temperature determination.
#[derive(Debug, Clone)]
pub struct BoltzmannPlotResult {
    /// Derived plasma temperature in Kelvin.
    pub temperature_k: f64,
    /// R² of the linear fit to ln(I*lambda/(g*A)) vs E_upper.
    pub r_squared: f64,
    /// Slope of the Boltzmann plot (= -1/(kT)).
    pub slope: f64,
    /// Intercept of the Boltzmann plot.
    pub intercept: f64,
}

/// Built-in database of common emission lines for AES analysis.
///
/// Contains lines for 20+ elements commonly analyzed by ICP-AES/OES.
/// Wavelengths, energies, statistical weights, and transition probabilities
/// are from NIST Atomic Spectra Database reference values.
const EMISSION_DATABASE: &[EmissionLine] = &[
    // Sodium (Na)
    EmissionLine { element: "Na", wavelength_nm: 589.0, relative_intensity: 1000.0, upper_energy_ev: 2.104, upper_stat_weight: 4.0, transition_prob: 6.16e7, is_ionic: false },
    EmissionLine { element: "Na", wavelength_nm: 589.6, relative_intensity: 500.0, upper_energy_ev: 2.102, upper_stat_weight: 2.0, transition_prob: 6.14e7, is_ionic: false },
    EmissionLine { element: "Na", wavelength_nm: 330.2, relative_intensity: 100.0, upper_energy_ev: 3.753, upper_stat_weight: 4.0, transition_prob: 2.77e7, is_ionic: false },
    // Potassium (K)
    EmissionLine { element: "K", wavelength_nm: 766.5, relative_intensity: 800.0, upper_energy_ev: 1.617, upper_stat_weight: 4.0, transition_prob: 3.87e7, is_ionic: false },
    EmissionLine { element: "K", wavelength_nm: 769.9, relative_intensity: 400.0, upper_energy_ev: 1.610, upper_stat_weight: 2.0, transition_prob: 3.82e7, is_ionic: false },
    // Calcium (Ca)
    EmissionLine { element: "Ca", wavelength_nm: 393.4, relative_intensity: 900.0, upper_energy_ev: 3.151, upper_stat_weight: 4.0, transition_prob: 1.47e8, is_ionic: true },
    EmissionLine { element: "Ca", wavelength_nm: 396.8, relative_intensity: 450.0, upper_energy_ev: 3.123, upper_stat_weight: 2.0, transition_prob: 1.40e8, is_ionic: true },
    EmissionLine { element: "Ca", wavelength_nm: 422.7, relative_intensity: 600.0, upper_energy_ev: 2.933, upper_stat_weight: 3.0, transition_prob: 2.18e8, is_ionic: false },
    EmissionLine { element: "Ca", wavelength_nm: 317.9, relative_intensity: 700.0, upper_energy_ev: 3.900, upper_stat_weight: 6.0, transition_prob: 3.60e8, is_ionic: true },
    // Iron (Fe)
    EmissionLine { element: "Fe", wavelength_nm: 238.2, relative_intensity: 850.0, upper_energy_ev: 5.204, upper_stat_weight: 9.0, transition_prob: 3.07e8, is_ionic: true },
    EmissionLine { element: "Fe", wavelength_nm: 259.9, relative_intensity: 750.0, upper_energy_ev: 4.769, upper_stat_weight: 11.0, transition_prob: 2.21e8, is_ionic: true },
    EmissionLine { element: "Fe", wavelength_nm: 371.9, relative_intensity: 500.0, upper_energy_ev: 3.332, upper_stat_weight: 7.0, transition_prob: 1.62e7, is_ionic: false },
    EmissionLine { element: "Fe", wavelength_nm: 248.3, relative_intensity: 600.0, upper_energy_ev: 4.992, upper_stat_weight: 7.0, transition_prob: 5.49e7, is_ionic: false },
    EmissionLine { element: "Fe", wavelength_nm: 302.1, relative_intensity: 400.0, upper_energy_ev: 4.103, upper_stat_weight: 5.0, transition_prob: 7.59e7, is_ionic: false },
    // Magnesium (Mg)
    EmissionLine { element: "Mg", wavelength_nm: 279.6, relative_intensity: 950.0, upper_energy_ev: 4.434, upper_stat_weight: 4.0, transition_prob: 2.60e8, is_ionic: true },
    EmissionLine { element: "Mg", wavelength_nm: 280.3, relative_intensity: 475.0, upper_energy_ev: 4.422, upper_stat_weight: 2.0, transition_prob: 2.57e8, is_ionic: true },
    EmissionLine { element: "Mg", wavelength_nm: 285.2, relative_intensity: 400.0, upper_energy_ev: 4.346, upper_stat_weight: 3.0, transition_prob: 4.91e8, is_ionic: false },
    // Aluminum (Al)
    EmissionLine { element: "Al", wavelength_nm: 396.2, relative_intensity: 700.0, upper_energy_ev: 3.143, upper_stat_weight: 4.0, transition_prob: 9.89e7, is_ionic: false },
    EmissionLine { element: "Al", wavelength_nm: 394.4, relative_intensity: 350.0, upper_energy_ev: 3.143, upper_stat_weight: 2.0, transition_prob: 4.93e7, is_ionic: false },
    EmissionLine { element: "Al", wavelength_nm: 309.3, relative_intensity: 500.0, upper_energy_ev: 4.022, upper_stat_weight: 3.0, transition_prob: 5.83e7, is_ionic: false },
    // Zinc (Zn)
    EmissionLine { element: "Zn", wavelength_nm: 213.9, relative_intensity: 800.0, upper_energy_ev: 5.796, upper_stat_weight: 3.0, transition_prob: 7.09e8, is_ionic: false },
    EmissionLine { element: "Zn", wavelength_nm: 206.2, relative_intensity: 400.0, upper_energy_ev: 6.012, upper_stat_weight: 5.0, transition_prob: 3.33e8, is_ionic: true },
    // Copper (Cu)
    EmissionLine { element: "Cu", wavelength_nm: 324.8, relative_intensity: 900.0, upper_energy_ev: 3.817, upper_stat_weight: 4.0, transition_prob: 1.39e8, is_ionic: false },
    EmissionLine { element: "Cu", wavelength_nm: 327.4, relative_intensity: 450.0, upper_energy_ev: 3.786, upper_stat_weight: 2.0, transition_prob: 1.37e8, is_ionic: false },
    EmissionLine { element: "Cu", wavelength_nm: 224.7, relative_intensity: 600.0, upper_energy_ev: 5.517, upper_stat_weight: 5.0, transition_prob: 3.07e8, is_ionic: true },
    // Manganese (Mn)
    EmissionLine { element: "Mn", wavelength_nm: 257.6, relative_intensity: 850.0, upper_energy_ev: 4.812, upper_stat_weight: 9.0, transition_prob: 2.70e8, is_ionic: true },
    EmissionLine { element: "Mn", wavelength_nm: 259.4, relative_intensity: 700.0, upper_energy_ev: 4.778, upper_stat_weight: 7.0, transition_prob: 2.65e8, is_ionic: true },
    EmissionLine { element: "Mn", wavelength_nm: 403.1, relative_intensity: 500.0, upper_energy_ev: 3.075, upper_stat_weight: 8.0, transition_prob: 1.70e7, is_ionic: false },
    // Chromium (Cr)
    EmissionLine { element: "Cr", wavelength_nm: 267.7, relative_intensity: 800.0, upper_energy_ev: 4.631, upper_stat_weight: 9.0, transition_prob: 1.28e8, is_ionic: true },
    EmissionLine { element: "Cr", wavelength_nm: 283.6, relative_intensity: 600.0, upper_energy_ev: 4.370, upper_stat_weight: 7.0, transition_prob: 1.07e8, is_ionic: true },
    EmissionLine { element: "Cr", wavelength_nm: 357.9, relative_intensity: 700.0, upper_energy_ev: 3.464, upper_stat_weight: 9.0, transition_prob: 1.48e7, is_ionic: false },
    // Nickel (Ni)
    EmissionLine { element: "Ni", wavelength_nm: 231.6, relative_intensity: 750.0, upper_energy_ev: 5.354, upper_stat_weight: 7.0, transition_prob: 2.00e8, is_ionic: true },
    EmissionLine { element: "Ni", wavelength_nm: 341.5, relative_intensity: 500.0, upper_energy_ev: 3.635, upper_stat_weight: 5.0, transition_prob: 5.50e7, is_ionic: false },
    // Lead (Pb)
    EmissionLine { element: "Pb", wavelength_nm: 220.4, relative_intensity: 700.0, upper_energy_ev: 5.625, upper_stat_weight: 3.0, transition_prob: 1.20e8, is_ionic: true },
    EmissionLine { element: "Pb", wavelength_nm: 405.8, relative_intensity: 500.0, upper_energy_ev: 3.057, upper_stat_weight: 5.0, transition_prob: 8.90e7, is_ionic: false },
    EmissionLine { element: "Pb", wavelength_nm: 283.3, relative_intensity: 600.0, upper_energy_ev: 4.375, upper_stat_weight: 3.0, transition_prob: 5.80e7, is_ionic: false },
    // Cadmium (Cd)
    EmissionLine { element: "Cd", wavelength_nm: 228.8, relative_intensity: 800.0, upper_energy_ev: 5.417, upper_stat_weight: 3.0, transition_prob: 5.30e8, is_ionic: false },
    EmissionLine { element: "Cd", wavelength_nm: 214.4, relative_intensity: 600.0, upper_energy_ev: 5.779, upper_stat_weight: 5.0, transition_prob: 1.50e8, is_ionic: true },
    // Arsenic (As)
    EmissionLine { element: "As", wavelength_nm: 193.7, relative_intensity: 700.0, upper_energy_ev: 6.400, upper_stat_weight: 4.0, transition_prob: 2.00e8, is_ionic: false },
    EmissionLine { element: "As", wavelength_nm: 197.2, relative_intensity: 400.0, upper_energy_ev: 6.289, upper_stat_weight: 6.0, transition_prob: 1.20e8, is_ionic: false },
    // Mercury (Hg)
    EmissionLine { element: "Hg", wavelength_nm: 253.7, relative_intensity: 900.0, upper_energy_ev: 4.888, upper_stat_weight: 3.0, transition_prob: 8.00e6, is_ionic: false },
    EmissionLine { element: "Hg", wavelength_nm: 194.2, relative_intensity: 500.0, upper_energy_ev: 6.385, upper_stat_weight: 3.0, transition_prob: 1.02e7, is_ionic: false },
    // Barium (Ba)
    EmissionLine { element: "Ba", wavelength_nm: 455.4, relative_intensity: 800.0, upper_energy_ev: 2.722, upper_stat_weight: 4.0, transition_prob: 1.19e8, is_ionic: true },
    EmissionLine { element: "Ba", wavelength_nm: 493.4, relative_intensity: 500.0, upper_energy_ev: 2.512, upper_stat_weight: 2.0, transition_prob: 9.53e7, is_ionic: true },
    // Strontium (Sr)
    EmissionLine { element: "Sr", wavelength_nm: 407.8, relative_intensity: 850.0, upper_energy_ev: 3.040, upper_stat_weight: 4.0, transition_prob: 1.41e8, is_ionic: true },
    EmissionLine { element: "Sr", wavelength_nm: 421.6, relative_intensity: 400.0, upper_energy_ev: 2.941, upper_stat_weight: 2.0, transition_prob: 1.27e8, is_ionic: true },
    EmissionLine { element: "Sr", wavelength_nm: 460.7, relative_intensity: 600.0, upper_energy_ev: 2.690, upper_stat_weight: 3.0, transition_prob: 2.01e8, is_ionic: false },
    // Lithium (Li)
    EmissionLine { element: "Li", wavelength_nm: 670.8, relative_intensity: 900.0, upper_energy_ev: 1.848, upper_stat_weight: 6.0, transition_prob: 3.69e7, is_ionic: false },
    // Titanium (Ti)
    EmissionLine { element: "Ti", wavelength_nm: 334.9, relative_intensity: 700.0, upper_energy_ev: 3.702, upper_stat_weight: 11.0, transition_prob: 1.73e8, is_ionic: true },
    EmissionLine { element: "Ti", wavelength_nm: 336.1, relative_intensity: 650.0, upper_energy_ev: 3.688, upper_stat_weight: 9.0, transition_prob: 1.58e8, is_ionic: true },
    // Vanadium (V)
    EmissionLine { element: "V", wavelength_nm: 309.3, relative_intensity: 600.0, upper_energy_ev: 4.007, upper_stat_weight: 8.0, transition_prob: 1.30e8, is_ionic: true },
    EmissionLine { element: "V", wavelength_nm: 311.1, relative_intensity: 550.0, upper_energy_ev: 3.986, upper_stat_weight: 10.0, transition_prob: 1.05e8, is_ionic: true },
    // Cobalt (Co)
    EmissionLine { element: "Co", wavelength_nm: 228.6, relative_intensity: 700.0, upper_energy_ev: 5.424, upper_stat_weight: 8.0, transition_prob: 1.70e8, is_ionic: true },
    EmissionLine { element: "Co", wavelength_nm: 238.9, relative_intensity: 550.0, upper_energy_ev: 5.189, upper_stat_weight: 6.0, transition_prob: 1.10e8, is_ionic: true },
    // Silver (Ag)
    EmissionLine { element: "Ag", wavelength_nm: 328.1, relative_intensity: 800.0, upper_energy_ev: 3.779, upper_stat_weight: 4.0, transition_prob: 1.47e8, is_ionic: false },
    EmissionLine { element: "Ag", wavelength_nm: 338.3, relative_intensity: 400.0, upper_energy_ev: 3.665, upper_stat_weight: 2.0, transition_prob: 1.34e8, is_ionic: false },
    // Beryllium (Be)
    EmissionLine { element: "Be", wavelength_nm: 313.0, relative_intensity: 800.0, upper_energy_ev: 3.962, upper_stat_weight: 4.0, transition_prob: 1.15e8, is_ionic: true },
    EmissionLine { element: "Be", wavelength_nm: 234.9, relative_intensity: 500.0, upper_energy_ev: 5.278, upper_stat_weight: 3.0, transition_prob: 5.54e8, is_ionic: false },
    // Selenium (Se)
    EmissionLine { element: "Se", wavelength_nm: 196.1, relative_intensity: 700.0, upper_energy_ev: 6.323, upper_stat_weight: 5.0, transition_prob: 1.50e8, is_ionic: false },
    // Boron (B)
    EmissionLine { element: "B", wavelength_nm: 249.7, relative_intensity: 700.0, upper_energy_ev: 4.964, upper_stat_weight: 6.0, transition_prob: 2.10e8, is_ionic: false },
    EmissionLine { element: "B", wavelength_nm: 249.8, relative_intensity: 350.0, upper_energy_ev: 4.964, upper_stat_weight: 4.0, transition_prob: 1.67e8, is_ionic: false },
    // Phosphorus (P)
    EmissionLine { element: "P", wavelength_nm: 213.6, relative_intensity: 600.0, upper_energy_ev: 5.803, upper_stat_weight: 4.0, transition_prob: 6.21e7, is_ionic: false },
    EmissionLine { element: "P", wavelength_nm: 214.9, relative_intensity: 400.0, upper_energy_ev: 5.770, upper_stat_weight: 6.0, transition_prob: 3.18e7, is_ionic: false },
    // Sulfur (S)
    EmissionLine { element: "S", wavelength_nm: 180.7, relative_intensity: 600.0, upper_energy_ev: 6.860, upper_stat_weight: 5.0, transition_prob: 4.20e8, is_ionic: false },
    EmissionLine { element: "S", wavelength_nm: 182.0, relative_intensity: 400.0, upper_energy_ev: 6.813, upper_stat_weight: 3.0, transition_prob: 3.40e8, is_ionic: false },
];

/// The Hydrogen-beta line wavelength in nm, used for Stark broadening electron density.
const H_BETA_WAVELENGTH_NM: f64 = 486.1;

/// The main AES/OES signal processor.
///
/// Processes spectral data from ICP-AES/OES, DCP, and flame emission instruments.
/// Provides peak identification, calibration, detection limits, plasma diagnostics,
/// and spectral interference correction.
///
/// # Example
///
/// ```
/// use r4w_core::atomic_emission_spectroscopy_processor::{AesProcessor, AesConfig};
///
/// let processor = AesProcessor::new(AesConfig::default());
///
/// // Get emission lines for iron
/// let fe_lines = processor.get_lines_for_element("Fe");
/// assert!(fe_lines.len() >= 3);
/// ```
#[derive(Debug, Clone)]
pub struct AesProcessor {
    config: AesConfig,
}

impl AesProcessor {
    /// Create a new AES processor with the given configuration.
    pub fn new(config: AesConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &AesConfig {
        &self.config
    }

    // -----------------------------------------------------------------------
    // Wavelength Database
    // -----------------------------------------------------------------------

    /// Return all emission lines in the built-in database.
    pub fn wavelength_database(&self) -> &[EmissionLine] {
        EMISSION_DATABASE
    }

    /// Return emission lines for a specific element.
    pub fn get_lines_for_element(&self, element: &str) -> Vec<&EmissionLine> {
        EMISSION_DATABASE
            .iter()
            .filter(|line| line.element == element)
            .collect()
    }

    /// Return all unique element symbols in the database.
    pub fn elements_in_database(&self) -> Vec<&'static str> {
        let mut elements: Vec<&'static str> = Vec::new();
        for line in EMISSION_DATABASE {
            if !elements.contains(&line.element) {
                elements.push(line.element);
            }
        }
        elements
    }

    // -----------------------------------------------------------------------
    // Spectral Line Intensity (Boltzmann distribution)
    // -----------------------------------------------------------------------

    /// Calculate theoretical emission line intensity from the Boltzmann distribution.
    ///
    /// The intensity of an emission line is proportional to:
    ///   I = C * g_k * A_ki * exp(-E_k / (k_B * T))
    ///
    /// where:
    /// - `g_k` is the statistical weight of the upper level
    /// - `A_ki` is the transition probability (Einstein A coefficient)
    /// - `E_k` is the upper level energy in eV
    /// - `T` is the plasma temperature in Kelvin
    ///
    /// Returns the relative intensity (arbitrary units).
    pub fn spectral_line_intensity(
        &self,
        upper_energy_ev: f64,
        stat_weight: f64,
        transition_prob: f64,
        temperature_k: f64,
    ) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        let exponent = -upper_energy_ev / (K_BOLTZMANN_EV * temperature_k);
        stat_weight * transition_prob * exponent.exp()
    }

    // -----------------------------------------------------------------------
    // Peak Identification
    // -----------------------------------------------------------------------

    /// Identify observed peaks by matching against the emission line database.
    ///
    /// For each observed wavelength, finds the closest database emission line within
    /// the specified tolerance. Returns all matches sorted by deviation.
    ///
    /// # Arguments
    /// * `observed_wavelengths` - Observed peak wavelengths in nm
    /// * `observed_intensities` - Corresponding peak intensities
    /// * `tolerance_nm` - Maximum allowed deviation in nm for a match
    pub fn identify_peaks(
        &self,
        observed_wavelengths: &[f64],
        observed_intensities: &[f64],
        tolerance_nm: f64,
    ) -> Vec<PeakMatch> {
        let mut matches = Vec::new();
        let n = observed_wavelengths.len().min(observed_intensities.len());

        for i in 0..n {
            let obs_wl = observed_wavelengths[i];
            let obs_int = observed_intensities[i];

            // Find best matching database line
            let mut best: Option<&EmissionLine> = None;
            let mut best_dev = f64::MAX;

            for line in EMISSION_DATABASE {
                let dev = (obs_wl - line.wavelength_nm).abs();
                if dev < tolerance_nm && dev < best_dev {
                    best_dev = dev;
                    best = Some(line);
                }
            }

            if let Some(line) = best {
                matches.push(PeakMatch {
                    observed_nm: obs_wl,
                    observed_intensity: obs_int,
                    element: line.element,
                    database_nm: line.wavelength_nm,
                    deviation_nm: best_dev,
                    is_ionic: line.is_ionic,
                });
            }
        }

        matches.sort_by(|a, b| a.deviation_nm.partial_cmp(&b.deviation_nm).unwrap());
        matches
    }

    // -----------------------------------------------------------------------
    // Background Correction
    // -----------------------------------------------------------------------

    /// Apply polynomial background correction to a spectrum.
    ///
    /// Fits a polynomial of the specified order to background regions and
    /// subtracts it from the signal. Background regions are identified as
    /// points below the median intensity.
    ///
    /// # Arguments
    /// * `wavelengths` - Wavelength array in nm
    /// * `intensities` - Measured intensity array (modified in place conceptually)
    /// * `poly_order` - Polynomial order (1 = linear, 2 = quadratic, etc.)
    ///
    /// Returns the background-corrected intensities.
    pub fn background_correction(
        &self,
        wavelengths: &[f64],
        intensities: &[f64],
        poly_order: usize,
    ) -> Vec<f64> {
        let n = wavelengths.len().min(intensities.len());
        if n == 0 {
            return Vec::new();
        }

        // Find median intensity to identify background points
        let mut sorted_int: Vec<f64> = intensities[..n].to_vec();
        sorted_int.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = sorted_int[sorted_int.len() / 2];

        // Collect background points (below median)
        let mut bg_wl = Vec::new();
        let mut bg_int = Vec::new();
        for i in 0..n {
            if intensities[i] <= median {
                bg_wl.push(wavelengths[i]);
                bg_int.push(intensities[i]);
            }
        }

        if bg_wl.len() <= poly_order {
            // Not enough background points, return as-is
            return intensities[..n].to_vec();
        }

        // Fit polynomial to background using least squares
        let coeffs = poly_fit(&bg_wl, &bg_int, poly_order);

        // Subtract fitted background
        let mut corrected = Vec::with_capacity(n);
        for i in 0..n {
            let bg = poly_eval(&coeffs, wavelengths[i]);
            let val = intensities[i] - bg;
            corrected.push(if val > 0.0 { val } else { 0.0 });
        }
        corrected
    }

    /// Apply off-peak background subtraction.
    ///
    /// Uses the average of two off-peak measurement points (one on each side
    /// of the analyte line) to estimate and subtract background.
    ///
    /// # Arguments
    /// * `signal_intensity` - Intensity at the analyte wavelength
    /// * `left_bg` - Background intensity on the short-wavelength side
    /// * `right_bg` - Background intensity on the long-wavelength side
    ///
    /// Returns the net (background-corrected) analyte intensity.
    pub fn off_peak_background_correction(
        &self,
        signal_intensity: f64,
        left_bg: f64,
        right_bg: f64,
    ) -> f64 {
        let bg = (left_bg + right_bg) / 2.0;
        let net = signal_intensity - bg;
        if net > 0.0 { net } else { 0.0 }
    }

    // -----------------------------------------------------------------------
    // Internal Standard Correction
    // -----------------------------------------------------------------------

    /// Apply internal standard correction to compensate for matrix effects.
    ///
    /// The analyte intensity is normalized to the internal standard line intensity:
    ///   corrected = analyte_intensity / standard_intensity
    ///
    /// Common internal standards: Y 371.0 nm, Sc 361.4 nm, In 325.6 nm.
    ///
    /// # Arguments
    /// * `analyte_intensities` - Measured analyte line intensities
    /// * `standard_intensities` - Corresponding internal standard line intensities
    ///
    /// Returns the ratio-corrected intensities.
    pub fn internal_standard_correction(
        &self,
        analyte_intensities: &[f64],
        standard_intensities: &[f64],
    ) -> Vec<f64> {
        let n = analyte_intensities.len().min(standard_intensities.len());
        let mut corrected = Vec::with_capacity(n);
        for i in 0..n {
            if standard_intensities[i] > 0.0 {
                corrected.push(analyte_intensities[i] / standard_intensities[i]);
            } else {
                corrected.push(0.0);
            }
        }
        corrected
    }

    // -----------------------------------------------------------------------
    // Inter-Element (Spectral Interference) Correction
    // -----------------------------------------------------------------------

    /// Correct for spectral interference from overlapping emission lines.
    ///
    /// Uses inter-element correction (IEC) factors:
    ///   corrected = measured - sum_j(k_j * interferent_j)
    ///
    /// where k_j is the interference coefficient for element j at the analyte
    /// wavelength, determined from single-element standards.
    ///
    /// # Arguments
    /// * `analyte_measured` - Measured intensity at the analyte wavelength
    /// * `interferent_intensities` - Intensities of interfering elements at their own lines
    /// * `correction_factors` - IEC factors (k_j) for each interferent
    ///
    /// Returns the corrected analyte intensity.
    pub fn inter_element_correction(
        &self,
        analyte_measured: f64,
        interferent_intensities: &[f64],
        correction_factors: &[f64],
    ) -> f64 {
        let n = interferent_intensities.len().min(correction_factors.len());
        let mut correction = 0.0;
        for i in 0..n {
            correction += correction_factors[i] * interferent_intensities[i];
        }
        let result = analyte_measured - correction;
        if result > 0.0 { result } else { 0.0 }
    }

    // -----------------------------------------------------------------------
    // Calibration Curve
    // -----------------------------------------------------------------------

    /// Build a calibration curve from concentration-intensity data.
    ///
    /// Fits a linear (y = a*x + b) or quadratic (y = a*x^2 + b*x + c) model
    /// using least-squares regression. Reports R² coefficient of determination.
    ///
    /// # Arguments
    /// * `concentrations` - Known concentrations (ppm)
    /// * `intensities` - Measured net intensities for each standard
    /// * `fit_type` - Linear or quadratic fit
    ///
    /// Returns the calibration result with coefficients, R², and residuals.
    pub fn calibration_curve(
        &self,
        concentrations: &[f64],
        intensities: &[f64],
        fit_type: CalibrationFit,
    ) -> CalibrationResult {
        let n = concentrations.len().min(intensities.len());
        assert!(n >= 2, "Need at least 2 calibration points");

        let order = match fit_type {
            CalibrationFit::Linear => 1,
            CalibrationFit::Quadratic => 2,
        };

        let coeffs = poly_fit(&concentrations[..n], &intensities[..n], order);

        // Calculate R²
        let y_mean = intensities[..n].iter().sum::<f64>() / n as f64;
        let ss_tot: f64 = intensities[..n]
            .iter()
            .map(|&y| (y - y_mean).powi(2))
            .sum();
        let mut residuals = Vec::with_capacity(n);
        let mut ss_res = 0.0;
        for i in 0..n {
            let y_pred = poly_eval(&coeffs, concentrations[i]);
            let r = intensities[i] - y_pred;
            residuals.push(r);
            ss_res += r * r;
        }

        let r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        CalibrationResult {
            coefficients: coeffs,
            r_squared,
            fit_type,
            residuals,
        }
    }

    /// Predict concentration from a calibration curve and measured intensity.
    ///
    /// For linear calibration y = a*x + b, solves for x = (y - b) / a.
    /// For quadratic, uses the quadratic formula.
    ///
    /// Returns the predicted concentration in ppm.
    pub fn predict_concentration(&self, cal: &CalibrationResult, intensity: f64) -> f64 {
        match cal.fit_type {
            CalibrationFit::Linear => {
                // y = a*x + b => x = (y - b) / a
                let a = cal.coefficients[0];
                let b = cal.coefficients[1];
                if a.abs() < 1e-30 {
                    return 0.0;
                }
                (intensity - b) / a
            }
            CalibrationFit::Quadratic => {
                // y = a*x^2 + b*x + c => a*x^2 + b*x + (c - y) = 0
                let a = cal.coefficients[0];
                let b = cal.coefficients[1];
                let c = cal.coefficients[2] - intensity;
                if a.abs() < 1e-30 {
                    // Degenerate to linear
                    if b.abs() < 1e-30 {
                        return 0.0;
                    }
                    return -c / b;
                }
                let disc = b * b - 4.0 * a * c;
                if disc < 0.0 {
                    return 0.0;
                }
                // Take the positive root
                let x1 = (-b + disc.sqrt()) / (2.0 * a);
                let x2 = (-b - disc.sqrt()) / (2.0 * a);
                if x1 >= 0.0 { x1 } else { x2 }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Detection Limit (3-sigma blank method)
    // -----------------------------------------------------------------------

    /// Calculate the method detection limit using the 3-sigma blank method.
    ///
    /// MDL = 3 * sigma_blank / sensitivity
    ///
    /// where sigma_blank is the standard deviation of repeated blank measurements
    /// and sensitivity is the slope of the calibration curve (counts/ppm).
    ///
    /// # Arguments
    /// * `blank_intensities` - Repeated measurements of a blank solution
    /// * `sensitivity` - Calibration sensitivity (intensity units per ppm)
    ///
    /// Returns the detection limit in ppm.
    pub fn detection_limit_ppm(&self, blank_intensities: &[f64], sensitivity: f64) -> f64 {
        if blank_intensities.len() < 2 || sensitivity.abs() < 1e-30 {
            return f64::MAX;
        }

        let n = blank_intensities.len() as f64;
        let mean = blank_intensities.iter().sum::<f64>() / n;
        let variance = blank_intensities
            .iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>()
            / (n - 1.0);
        let sigma = variance.sqrt();

        3.0 * sigma / sensitivity
    }

    // -----------------------------------------------------------------------
    // Plasma Temperature (Boltzmann Plot)
    // -----------------------------------------------------------------------

    /// Determine plasma excitation temperature from a Boltzmann plot.
    ///
    /// Uses multiple emission lines of the same element. Plots:
    ///   ln(I * lambda / (g * A)) vs. E_upper
    ///
    /// The slope of the linear fit gives -1/(k_B * T), so T = -1/(k_B * slope).
    ///
    /// # Arguments
    /// * `lines` - Emission line data from the database (same element)
    /// * `measured_intensities` - Corresponding measured intensities
    ///
    /// Returns temperature and fit quality.
    pub fn plasma_temperature(
        &self,
        lines: &[&EmissionLine],
        measured_intensities: &[f64],
    ) -> BoltzmannPlotResult {
        let n = lines.len().min(measured_intensities.len());
        assert!(n >= 2, "Need at least 2 lines for Boltzmann plot");

        let mut x_vals = Vec::with_capacity(n); // E_upper
        let mut y_vals = Vec::with_capacity(n); // ln(I * lambda / (g * A))

        for i in 0..n {
            let line = lines[i];
            let intensity = measured_intensities[i];
            if intensity <= 0.0 || line.transition_prob <= 0.0 || line.upper_stat_weight <= 0.0 {
                continue;
            }
            let y = (intensity * line.wavelength_nm
                / (line.upper_stat_weight * line.transition_prob))
                .ln();
            x_vals.push(line.upper_energy_ev);
            y_vals.push(y);
        }

        if x_vals.len() < 2 {
            return BoltzmannPlotResult {
                temperature_k: 0.0,
                r_squared: 0.0,
                slope: 0.0,
                intercept: 0.0,
            };
        }

        // Linear fit: y = slope * x + intercept
        let coeffs = poly_fit(&x_vals, &y_vals, 1);
        let slope = coeffs[0];
        let intercept = coeffs[1];

        // R²
        let mean_y = y_vals.iter().sum::<f64>() / y_vals.len() as f64;
        let ss_tot: f64 = y_vals.iter().map(|&y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = x_vals
            .iter()
            .zip(y_vals.iter())
            .map(|(&x, &y)| {
                let pred = slope * x + intercept;
                (y - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        // T = -1 / (k_B * slope)
        let temperature_k = if slope.abs() > 1e-30 {
            -1.0 / (K_BOLTZMANN_EV * slope)
        } else {
            0.0
        };

        BoltzmannPlotResult {
            temperature_k,
            r_squared,
            slope,
            intercept,
        }
    }

    // -----------------------------------------------------------------------
    // Electron Density (Stark Broadening)
    // -----------------------------------------------------------------------

    /// Estimate electron density from Stark broadening of the H-beta line.
    ///
    /// The full width at half maximum (FWHM) of the H-beta line at 486.1 nm
    /// is related to electron density by:
    ///   n_e = C(T) * (FWHM / alpha_1/2)^(3/2)
    ///
    /// Using the simplified Griem approximation for ICP conditions (~8000 K):
    ///   n_e ≈ (FWHM_nm / 4.8)^(3/2) * 1e23 m^-3
    ///
    /// # Arguments
    /// * `fwhm_nm` - Measured FWHM of the H-beta line in nm
    ///
    /// Returns electron density in m^-3.
    pub fn electron_density(&self, fwhm_nm: f64) -> f64 {
        if fwhm_nm <= 0.0 {
            return 0.0;
        }
        // Simplified Griem formula for H-beta Stark broadening
        // n_e = (FWHM / w_ref)^(3/2) * n_ref
        // At ~8000 K: w_ref ≈ 4.8 nm corresponds to n_e ≈ 1e23 m^-3
        let ratio = fwhm_nm / 4.8;
        ratio.powf(1.5) * 1e23
    }

    // -----------------------------------------------------------------------
    // Ionization Fraction (Saha Equation)
    // -----------------------------------------------------------------------

    /// Calculate the ionization fraction using the Saha equation.
    ///
    /// The Saha-Eggert equation for single ionization:
    ///   n_i * n_e / n_a = (2 * Z_i / Z_a) * (2*pi*m_e*k_B*T / h^2)^(3/2) * exp(-E_ion / (k_B*T))
    ///
    /// where n_i, n_e, n_a are ion, electron, and atom number densities.
    /// Returns the ionization fraction alpha = n_i / (n_a + n_i).
    ///
    /// # Arguments
    /// * `temperature_k` - Plasma temperature in Kelvin
    /// * `electron_density` - Electron density in m^-3
    /// * `ionization_energy_ev` - First ionization energy of the element in eV
    /// * `z_ion` - Partition function of the ion
    /// * `z_atom` - Partition function of the neutral atom
    pub fn ionization_fraction(
        &self,
        temperature_k: f64,
        electron_density_m3: f64,
        ionization_energy_ev: f64,
        z_ion: f64,
        z_atom: f64,
    ) -> f64 {
        if temperature_k <= 0.0 || electron_density_m3 <= 0.0 || z_atom <= 0.0 {
            return 0.0;
        }

        let kt = K_BOLTZMANN_EV * temperature_k; // in eV

        // (2*pi*m_e*k_B*T / h^2)^(3/2) using SI units
        // k_B in J/K = 1.380649e-23
        let k_b_si = 1.380_649e-23;
        let h_si = 6.626_070_15e-34;
        let thermal = 2.0 * PI * ELECTRON_MASS_KG * k_b_si * temperature_k / (h_si * h_si);
        let thermal_32 = thermal.powf(1.5);

        // Saha constant K_s
        let k_saha =
            2.0 * (z_ion / z_atom) * thermal_32 * (-ionization_energy_ev / kt).exp();

        // Quadratic: n_e * alpha / (1 - alpha) = K_s  (with n_e = n_total * alpha for pure element)
        // For given n_e: alpha / (1 - alpha) = K_s / n_e
        let ratio = k_saha / electron_density_m3;
        // alpha = ratio / (1 + ratio)
        let alpha = ratio / (1.0 + ratio);
        alpha.clamp(0.0, 1.0)
    }

    // -----------------------------------------------------------------------
    // Self-Absorption Correction
    // -----------------------------------------------------------------------

    /// Correct measured intensity for self-absorption effects.
    ///
    /// At high concentrations, the outer cooler zones of the plasma absorb
    /// radiation from the hot core, leading to a non-linear response.
    /// The curve-of-growth correction uses the parameter b:
    ///
    ///   I_corrected = I_measured * (k * l) / (1 - exp(-k * l))
    ///
    /// where k*l is the optical depth at line center.
    ///
    /// For small optical depth (k*l << 1), I_corrected ≈ I_measured (optically thin).
    /// For large optical depth, the correction becomes significant.
    ///
    /// # Arguments
    /// * `measured_intensity` - Observed line intensity
    /// * `optical_depth` - Estimated optical depth (k*l) at line center
    ///
    /// Returns the corrected intensity.
    pub fn self_absorption_correction(
        &self,
        measured_intensity: f64,
        optical_depth: f64,
    ) -> f64 {
        if optical_depth.abs() < 1e-10 {
            return measured_intensity;
        }
        // Growth factor: tau / (1 - exp(-tau))
        let factor = optical_depth / (1.0 - (-optical_depth).exp());
        measured_intensity * factor
    }

    // -----------------------------------------------------------------------
    // Voigt Profile
    // -----------------------------------------------------------------------

    /// Calculate a Voigt profile (convolution of Gaussian and Lorentzian).
    ///
    /// The Voigt profile is the spectral line shape resulting from the combined
    /// effects of Doppler broadening (Gaussian, temperature-dependent) and
    /// pressure/Stark broadening (Lorentzian).
    ///
    /// Uses the pseudo-Voigt approximation:
    ///   V(x) ≈ eta * L(x) + (1 - eta) * G(x)
    ///
    /// where eta depends on the ratio of Lorentzian to total FWHM.
    ///
    /// # Arguments
    /// * `wavelengths` - Array of wavelength points in nm
    /// * `center_nm` - Line center wavelength in nm
    /// * `gaussian_fwhm_nm` - Gaussian (Doppler) FWHM in nm
    /// * `lorentzian_fwhm_nm` - Lorentzian (pressure) FWHM in nm
    /// * `amplitude` - Peak amplitude
    ///
    /// Returns the Voigt profile values at each wavelength point.
    pub fn voigt_profile(
        &self,
        wavelengths: &[f64],
        center_nm: f64,
        gaussian_fwhm_nm: f64,
        lorentzian_fwhm_nm: f64,
        amplitude: f64,
    ) -> Vec<f64> {
        // Total FWHM using Thompson et al. approximation
        let fg = gaussian_fwhm_nm;
        let fl = lorentzian_fwhm_nm;
        let f_v = (fg.powi(5)
            + 2.69269 * fg.powi(4) * fl
            + 2.42843 * fg.powi(3) * fl.powi(2)
            + 4.47163 * fg.powi(2) * fl.powi(3)
            + 0.07842 * fg * fl.powi(4)
            + fl.powi(5))
        .powf(0.2);

        // Mixing parameter eta (fraction of Lorentzian)
        let eta = if f_v > 0.0 {
            let r = fl / f_v;
            1.36603 * r - 0.47719 * r * r + 0.11116 * r * r * r
        } else {
            0.0
        }
        .clamp(0.0, 1.0);

        let sigma_g = if fg > 0.0 {
            fg / (2.0 * (2.0 * LN2).sqrt())
        } else {
            1e-10
        };
        let gamma_l = fl / 2.0;

        wavelengths
            .iter()
            .map(|&wl| {
                let dx = wl - center_nm;

                // Gaussian component
                let g = (-dx * dx / (2.0 * sigma_g * sigma_g)).exp()
                    / (sigma_g * (2.0 * PI).sqrt());

                // Lorentzian component
                let l = (gamma_l / PI) / (dx * dx + gamma_l * gamma_l);

                // Pseudo-Voigt
                let v = eta * l + (1.0 - eta) * g;

                // Normalize so peak ≈ amplitude
                // Peak of pure Gaussian = 1/(sigma*sqrt(2*pi))
                // Peak of pure Lorentzian = 1/(pi*gamma)
                let peak_g = 1.0 / (sigma_g * (2.0 * PI).sqrt());
                let peak_l = 1.0 / (PI * gamma_l);
                let peak_v = eta * peak_l + (1.0 - eta) * peak_g;

                if peak_v > 0.0 {
                    amplitude * v / peak_v
                } else {
                    0.0
                }
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Spectral Resolution
    // -----------------------------------------------------------------------

    /// Calculate the spectral resolution based on the Rayleigh criterion.
    ///
    /// Two spectral lines are considered resolved when the maximum of one
    /// coincides with the first minimum of the other. For a grating:
    ///   R = lambda / delta_lambda = m * N
    ///
    /// where m is the diffraction order and N is the number of illuminated grooves.
    ///
    /// # Arguments
    /// * `wavelength_nm` - Center wavelength in nm
    /// * `grating_order` - Diffraction order (typically 1 or 2)
    /// * `num_grooves` - Number of illuminated grating grooves
    ///
    /// Returns the minimum resolvable wavelength difference (delta_lambda) in nm.
    pub fn spectral_resolution(
        &self,
        wavelength_nm: f64,
        grating_order: u32,
        num_grooves: u32,
    ) -> f64 {
        let resolving_power = (grating_order as f64) * (num_grooves as f64);
        if resolving_power > 0.0 {
            wavelength_nm / resolving_power
        } else {
            f64::MAX
        }
    }

    // -----------------------------------------------------------------------
    // Signal-to-Background Ratio
    // -----------------------------------------------------------------------

    /// Calculate the signal-to-background ratio (SBR).
    ///
    /// SBR = (I_signal - I_background) / I_background
    ///
    /// A higher SBR indicates better analytical sensitivity for that line.
    ///
    /// # Arguments
    /// * `signal_intensity` - Gross (total) intensity at the analyte line
    /// * `background_intensity` - Background intensity near the analyte line
    pub fn signal_to_background_ratio(
        &self,
        signal_intensity: f64,
        background_intensity: f64,
    ) -> f64 {
        if background_intensity <= 0.0 {
            if signal_intensity > 0.0 {
                return f64::MAX;
            } else {
                return 0.0;
            }
        }
        (signal_intensity - background_intensity) / background_intensity
    }

    // -----------------------------------------------------------------------
    // Doppler Broadening Width
    // -----------------------------------------------------------------------

    /// Calculate Doppler (Gaussian) broadening FWHM for a given element.
    ///
    /// Doppler FWHM (nm) = wavelength * sqrt(8 * k_B * T * ln2 / (m * c^2))
    ///
    /// # Arguments
    /// * `wavelength_nm` - Line wavelength in nm
    /// * `temperature_k` - Plasma temperature in Kelvin
    /// * `atomic_mass_amu` - Atomic mass of the element in amu
    ///
    /// Returns the Doppler FWHM in nm.
    pub fn doppler_broadening_fwhm(
        &self,
        wavelength_nm: f64,
        temperature_k: f64,
        atomic_mass_amu: f64,
    ) -> f64 {
        if temperature_k <= 0.0 || atomic_mass_amu <= 0.0 {
            return 0.0;
        }
        // k_B in J/K, mass in kg
        let k_b_si = 1.380_649e-23;
        let amu_to_kg = 1.660_539_066_6e-27;
        let mass_kg = atomic_mass_amu * amu_to_kg;

        let factor = (8.0 * k_b_si * temperature_k * LN2 / (mass_kg * SPEED_OF_LIGHT * SPEED_OF_LIGHT)).sqrt();
        wavelength_nm * factor
    }

    // -----------------------------------------------------------------------
    // Typical Plasma Temperature
    // -----------------------------------------------------------------------

    /// Return the typical excitation temperature for the configured plasma type.
    ///
    /// - ICP: ~7500 K (6000-10000 K)
    /// - DCP: ~5000 K
    /// - Flame (air-acetylene): ~2300 K
    pub fn typical_temperature_k(&self) -> f64 {
        match self.config.plasma_type {
            PlasmaType::Icp => 7500.0,
            PlasmaType::Dcp => 5000.0,
            PlasmaType::Flame => 2300.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Polynomial least-squares fit.
///
/// Fits y = c[0]*x^0 + c[1]*x^1 + ... + c[order]*x^order
/// but we store as: coefficients[0] = slope for order 1, coefficients[1] = intercept
/// To match calibration convention: coefficients[0]*x + coefficients[1] for linear.
///
/// Actually, we use the convention: y = coefficients[0]*x^order + ... + coefficients[order]
/// i.e., highest power first.
fn poly_fit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len();
    let m = order + 1; // number of coefficients

    // Build normal equations: A^T A c = A^T y
    // where A is the Vandermonde matrix [1, x, x^2, ...]

    // Compute sums of powers of x
    let mut x_powers = vec![0.0_f64; 2 * order + 1];
    for k in 0..x_powers.len() {
        for i in 0..n {
            x_powers[k] += x[i].powi(k as i32);
        }
    }

    // Compute sums of y * x^k
    let mut xy_sums = vec![0.0_f64; m];
    for k in 0..m {
        for i in 0..n {
            xy_sums[k] += y[i] * x[i].powi(k as i32);
        }
    }

    // Build the normal equation matrix (m x m+1 augmented)
    let mut mat = vec![vec![0.0_f64; m + 1]; m];
    for row in 0..m {
        for col in 0..m {
            mat[row][col] = x_powers[row + col];
        }
        mat[row][m] = xy_sums[row];
    }

    // Gauss elimination with partial pivoting
    for col in 0..m {
        // Find pivot
        let mut max_row = col;
        let mut max_val = mat[col][col].abs();
        for row in (col + 1)..m {
            if mat[row][col].abs() > max_val {
                max_val = mat[row][col].abs();
                max_row = row;
            }
        }
        mat.swap(col, max_row);

        let pivot = mat[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..m {
            let factor = mat[row][col] / pivot;
            for j in col..=m {
                mat[row][j] -= factor * mat[col][j];
            }
        }
    }

    // Back substitution
    let mut coeffs_ascending = vec![0.0_f64; m];
    for row in (0..m).rev() {
        let mut sum = mat[row][m];
        for col in (row + 1)..m {
            sum -= mat[row][col] * coeffs_ascending[col];
        }
        if mat[row][row].abs() > 1e-30 {
            coeffs_ascending[row] = sum / mat[row][row];
        }
    }

    // Reverse to highest-power-first convention
    let mut coeffs = coeffs_ascending;
    coeffs.reverse();
    coeffs
}

/// Evaluate a polynomial with coefficients in highest-power-first order.
///
/// coeffs = [a_n, a_{n-1}, ..., a_1, a_0]
/// result = a_n * x^n + ... + a_1 * x + a_0
fn poly_eval(coeffs: &[f64], x: f64) -> f64 {
    // Horner's method
    let mut result = 0.0;
    for &c in coeffs {
        result = result * x + c;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> AesProcessor {
        AesProcessor::new(AesConfig::default())
    }

    // --- Database tests ---

    #[test]
    fn test_database_has_elements() {
        let proc = default_processor();
        let elements = proc.elements_in_database();
        assert!(elements.len() >= 20, "Should have at least 20 elements, got {}", elements.len());
    }

    #[test]
    fn test_database_contains_sodium() {
        let proc = default_processor();
        let na_lines = proc.get_lines_for_element("Na");
        assert!(!na_lines.is_empty());
        // Na D1/D2 lines around 589 nm
        let has_d_line = na_lines.iter().any(|l| (l.wavelength_nm - 589.0).abs() < 1.0);
        assert!(has_d_line, "Na should have D line near 589 nm");
    }

    #[test]
    fn test_database_contains_iron() {
        let proc = default_processor();
        let fe_lines = proc.get_lines_for_element("Fe");
        assert!(fe_lines.len() >= 3, "Fe should have multiple lines");
    }

    #[test]
    fn test_database_contains_calcium() {
        let proc = default_processor();
        let ca_lines = proc.get_lines_for_element("Ca");
        assert!(ca_lines.len() >= 2);
        // Ca II 393.4 nm (H line)
        let has_h_line = ca_lines.iter().any(|l| (l.wavelength_nm - 393.4).abs() < 1.0);
        assert!(has_h_line);
    }

    #[test]
    fn test_database_ionic_lines() {
        let proc = default_processor();
        let db = proc.wavelength_database();
        let ionic_count = db.iter().filter(|l| l.is_ionic).count();
        let atomic_count = db.iter().filter(|l| !l.is_ionic).count();
        assert!(ionic_count > 0, "Should have some ionic lines");
        assert!(atomic_count > 0, "Should have some atomic lines");
    }

    #[test]
    fn test_elements_unique() {
        let proc = default_processor();
        let elements = proc.elements_in_database();
        let mut seen = Vec::new();
        for e in &elements {
            assert!(!seen.contains(e), "Duplicate element: {}", e);
            seen.push(*e);
        }
    }

    // --- Spectral line intensity ---

    #[test]
    fn test_spectral_line_intensity_positive() {
        let proc = default_processor();
        let intensity = proc.spectral_line_intensity(3.0, 4.0, 1e8, 7500.0);
        assert!(intensity > 0.0);
    }

    #[test]
    fn test_spectral_line_intensity_temperature_dependence() {
        let proc = default_processor();
        // Higher temperature => more population in upper levels => higher intensity
        let i_low = proc.spectral_line_intensity(5.0, 4.0, 1e8, 5000.0);
        let i_high = proc.spectral_line_intensity(5.0, 4.0, 1e8, 10000.0);
        assert!(i_high > i_low, "Higher T should give higher intensity for high energy level");
    }

    #[test]
    fn test_spectral_line_intensity_zero_temp() {
        let proc = default_processor();
        let intensity = proc.spectral_line_intensity(3.0, 4.0, 1e8, 0.0);
        assert_eq!(intensity, 0.0);
    }

    // --- Peak identification ---

    #[test]
    fn test_identify_sodium_d_lines() {
        let proc = default_processor();
        let wls = vec![589.0, 589.6];
        let ints = vec![50000.0, 25000.0];
        let matches = proc.identify_peaks(&wls, &ints, 0.5);
        assert_eq!(matches.len(), 2);
        assert_eq!(matches[0].element, "Na");
        assert_eq!(matches[1].element, "Na");
    }

    #[test]
    fn test_identify_no_match() {
        let proc = default_processor();
        // Wavelength far from any database entry
        let wls = vec![999.9];
        let ints = vec![100.0];
        let matches = proc.identify_peaks(&wls, &ints, 0.1);
        assert!(matches.is_empty());
    }

    #[test]
    fn test_identify_potassium() {
        let proc = default_processor();
        let wls = vec![766.5];
        let ints = vec![30000.0];
        let matches = proc.identify_peaks(&wls, &ints, 0.5);
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].element, "K");
    }

    #[test]
    fn test_identify_tolerance() {
        let proc = default_processor();
        // 591.5 nm is 2.5 nm off from Na 589.0 and 1.9 nm from Na 589.6
        let wls = vec![591.5];
        let ints = vec![1000.0];
        let tight = proc.identify_peaks(&wls, &ints, 0.5);
        let loose = proc.identify_peaks(&wls, &ints, 3.0);
        assert!(tight.is_empty(), "Tight tolerance should not match");
        assert!(!loose.is_empty(), "Loose tolerance should match Na");
    }

    // --- Background correction ---

    #[test]
    fn test_polynomial_background_correction() {
        let proc = default_processor();
        let wl: Vec<f64> = (0..100).map(|i| 200.0 + i as f64 * 6.0).collect();
        // Background = linear slope + a peak at index 50
        let int: Vec<f64> = wl
            .iter()
            .enumerate()
            .map(|(i, &w)| {
                let bg = 100.0 + 0.5 * (w - 200.0);
                if (i as i32 - 50).unsigned_abs() < 5 {
                    bg + 5000.0 // peak
                } else {
                    bg
                }
            })
            .collect();

        let corrected = proc.background_correction(&wl, &int, 1);
        assert_eq!(corrected.len(), 100);
        // Peak region should still have signal
        assert!(corrected[50] > 1000.0, "Peak should remain after bg correction");
        // Far-from-peak background region should be near zero
        assert!(corrected[10] < 200.0, "Background should be near zero");
    }

    #[test]
    fn test_off_peak_background() {
        let proc = default_processor();
        let net = proc.off_peak_background_correction(1000.0, 100.0, 120.0);
        assert!((net - 890.0).abs() < 1e-6);
    }

    #[test]
    fn test_off_peak_background_negative_clamp() {
        let proc = default_processor();
        let net = proc.off_peak_background_correction(50.0, 100.0, 100.0);
        assert_eq!(net, 0.0, "Negative net intensity should clamp to 0");
    }

    // --- Internal standard correction ---

    #[test]
    fn test_internal_standard_correction() {
        let proc = default_processor();
        let analyte = vec![1000.0, 2000.0, 3000.0];
        let standard = vec![500.0, 500.0, 500.0];
        let corrected = proc.internal_standard_correction(&analyte, &standard);
        assert_eq!(corrected.len(), 3);
        assert!((corrected[0] - 2.0).abs() < 1e-10);
        assert!((corrected[1] - 4.0).abs() < 1e-10);
        assert!((corrected[2] - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_internal_standard_zero_standard() {
        let proc = default_processor();
        let corrected = proc.internal_standard_correction(&[100.0], &[0.0]);
        assert_eq!(corrected[0], 0.0);
    }

    // --- Inter-element correction ---

    #[test]
    fn test_inter_element_correction() {
        let proc = default_processor();
        let corrected = proc.inter_element_correction(
            1000.0,
            &[500.0, 200.0],
            &[0.1, 0.05],
        );
        // 1000 - (0.1*500 + 0.05*200) = 1000 - 50 - 10 = 940
        assert!((corrected - 940.0).abs() < 1e-6);
    }

    #[test]
    fn test_inter_element_correction_clamped() {
        let proc = default_processor();
        let corrected = proc.inter_element_correction(10.0, &[500.0], &[1.0]);
        assert_eq!(corrected, 0.0);
    }

    // --- Calibration curve ---

    #[test]
    fn test_linear_calibration() {
        let proc = default_processor();
        // Perfect linear: y = 100*x + 50
        let conc = vec![0.0, 1.0, 2.0, 5.0, 10.0];
        let ints = vec![50.0, 150.0, 250.0, 550.0, 1050.0];
        let cal = proc.calibration_curve(&conc, &ints, CalibrationFit::Linear);
        assert!(cal.r_squared > 0.999, "R² should be ~1.0 for perfect linear, got {}", cal.r_squared);
        assert_eq!(cal.coefficients.len(), 2);
        // Predict concentration
        let predicted = proc.predict_concentration(&cal, 550.0);
        assert!((predicted - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_quadratic_calibration() {
        let proc = default_processor();
        // y = 5*x^2 + 10*x + 20
        let conc = vec![0.0, 1.0, 2.0, 5.0, 10.0, 20.0];
        let ints: Vec<f64> = conc.iter().map(|&x| 5.0 * x * x + 10.0 * x + 20.0).collect();
        let cal = proc.calibration_curve(&conc, &ints, CalibrationFit::Quadratic);
        assert!(cal.r_squared > 0.999, "R² should be ~1.0 for perfect quadratic, got {}", cal.r_squared);

        // Predict concentration for intensity at x=3: 5*9 + 30 + 20 = 95
        let predicted = proc.predict_concentration(&cal, 95.0);
        assert!((predicted - 3.0).abs() < 0.5, "Predicted {} should be ~3.0", predicted);
    }

    // --- Detection limit ---

    #[test]
    fn test_detection_limit() {
        let proc = default_processor();
        let blanks = vec![10.0, 12.0, 11.0, 9.0, 13.0, 10.5, 11.5];
        let sensitivity = 1000.0;
        let dl = proc.detection_limit_ppm(&blanks, sensitivity);
        assert!(dl > 0.0);
        assert!(dl < 0.1, "DL should be small for these values, got {}", dl);
    }

    #[test]
    fn test_detection_limit_high_noise() {
        let proc = default_processor();
        let blanks = vec![10.0, 50.0, 5.0, 80.0, 15.0];
        let sensitivity = 100.0;
        let dl = proc.detection_limit_ppm(&blanks, sensitivity);
        assert!(dl > 0.5, "High noise should give higher DL");
    }

    #[test]
    fn test_detection_limit_insufficient_data() {
        let proc = default_processor();
        let dl = proc.detection_limit_ppm(&[10.0], 100.0);
        assert_eq!(dl, f64::MAX);
    }

    // --- Plasma temperature ---

    #[test]
    fn test_plasma_temperature_boltzmann() {
        let proc = default_processor();
        // Use Fe lines with known intensities consistent with ~7500 K
        let fe_lines = proc.get_lines_for_element("Fe");
        assert!(fe_lines.len() >= 3);

        // Generate synthetic intensities from Boltzmann at 7500 K
        let temp = 7500.0;
        let intensities: Vec<f64> = fe_lines
            .iter()
            .map(|line| {
                proc.spectral_line_intensity(
                    line.upper_energy_ev,
                    line.upper_stat_weight,
                    line.transition_prob,
                    temp,
                )
            })
            .collect();

        let result = proc.plasma_temperature(&fe_lines, &intensities);
        assert!(result.r_squared > 0.90, "R² should be high for consistent data, got {}", result.r_squared);
        assert!(
            (result.temperature_k - temp).abs() < 2000.0,
            "Temperature should be near 7500 K, got {} K",
            result.temperature_k
        );
    }

    // --- Electron density ---

    #[test]
    fn test_electron_density() {
        let proc = default_processor();
        // Typical ICP H-beta FWHM ~0.05 nm
        let ne = proc.electron_density(0.05);
        assert!(ne > 0.0);
        // Expected ~1e20 m^-3 range for ICP
        assert!(ne > 1e18 && ne < 1e22, "n_e should be in ICP range, got {:.2e}", ne);
    }

    #[test]
    fn test_electron_density_zero_fwhm() {
        let proc = default_processor();
        assert_eq!(proc.electron_density(0.0), 0.0);
    }

    // --- Ionization fraction ---

    #[test]
    fn test_ionization_fraction() {
        let proc = default_processor();
        // Ca ionization energy ~6.11 eV, at 7500 K in ICP
        let alpha = proc.ionization_fraction(
            7500.0,
            1e21, // typical ICP electron density
            6.11, // Ca ionization energy
            1.0,  // Z_ion
            1.0,  // Z_atom
        );
        assert!(alpha > 0.0 && alpha < 1.0, "Ionization fraction should be 0-1, got {}", alpha);
    }

    #[test]
    fn test_ionization_fraction_zero_temp() {
        let proc = default_processor();
        let alpha = proc.ionization_fraction(0.0, 1e21, 6.0, 1.0, 1.0);
        assert_eq!(alpha, 0.0);
    }

    #[test]
    fn test_ionization_fraction_high_temp() {
        let proc = default_processor();
        // Very high temperature should give high ionization
        let alpha = proc.ionization_fraction(20000.0, 1e19, 5.0, 2.0, 1.0);
        assert!(alpha > 0.5, "High T should give high ionization, got {}", alpha);
    }

    // --- Self-absorption correction ---

    #[test]
    fn test_self_absorption_thin() {
        let proc = default_processor();
        // Optically thin: correction factor ≈ 1
        let corrected = proc.self_absorption_correction(1000.0, 0.01);
        assert!((corrected - 1000.0).abs() / 1000.0 < 0.01);
    }

    #[test]
    fn test_self_absorption_thick() {
        let proc = default_processor();
        // Optically thick: correction factor > 1
        let corrected = proc.self_absorption_correction(1000.0, 5.0);
        assert!(corrected > 1000.0, "Self-absorption correction should increase intensity");
    }

    #[test]
    fn test_self_absorption_zero_depth() {
        let proc = default_processor();
        let corrected = proc.self_absorption_correction(500.0, 0.0);
        assert_eq!(corrected, 500.0);
    }

    // --- Voigt profile ---

    #[test]
    fn test_voigt_profile_peak_at_center() {
        let proc = default_processor();
        let wls: Vec<f64> = (0..100).map(|i| 588.0 + i as f64 * 0.03).collect();
        let profile = proc.voigt_profile(&wls, 589.5, 0.02, 0.01, 1.0);
        assert_eq!(profile.len(), 100);

        // Find peak
        let max_idx = profile
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_wl = wls[max_idx];
        assert!(
            (peak_wl - 589.5).abs() < 0.1,
            "Voigt peak should be near 589.5 nm, found at {} nm",
            peak_wl
        );
    }

    #[test]
    fn test_voigt_profile_gaussian_limit() {
        let proc = default_processor();
        // Pure Gaussian (no Lorentzian)
        let wls: Vec<f64> = (0..200).map(|i| 589.0 + i as f64 * 0.005).collect();
        let profile = proc.voigt_profile(&wls, 589.5, 0.05, 0.0001, 1.0);
        // Should still produce a peak
        let max_val = profile.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_val > 0.0);
    }

    #[test]
    fn test_voigt_profile_lorentzian_limit() {
        let proc = default_processor();
        // Dominant Lorentzian
        let wls: Vec<f64> = (0..200).map(|i| 589.0 + i as f64 * 0.005).collect();
        let profile = proc.voigt_profile(&wls, 589.5, 0.001, 0.05, 1.0);
        let max_val = profile.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_val > 0.0);
    }

    // --- Spectral resolution ---

    #[test]
    fn test_spectral_resolution() {
        let proc = default_processor();
        // 3600 grooves, order 1, at 500 nm
        let delta = proc.spectral_resolution(500.0, 1, 3600);
        // R = 3600, delta = 500/3600 ≈ 0.139 nm
        assert!((delta - 500.0 / 3600.0).abs() < 0.001);
    }

    #[test]
    fn test_spectral_resolution_higher_order() {
        let proc = default_processor();
        let delta_1 = proc.spectral_resolution(400.0, 1, 2400);
        let delta_2 = proc.spectral_resolution(400.0, 2, 2400);
        assert!(delta_2 < delta_1, "Higher order should give better resolution");
    }

    // --- Signal-to-background ratio ---

    #[test]
    fn test_sbr() {
        let proc = default_processor();
        let sbr = proc.signal_to_background_ratio(1100.0, 100.0);
        assert!((sbr - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_sbr_zero_background() {
        let proc = default_processor();
        let sbr = proc.signal_to_background_ratio(100.0, 0.0);
        assert_eq!(sbr, f64::MAX);
    }

    // --- Doppler broadening ---

    #[test]
    fn test_doppler_broadening() {
        let proc = default_processor();
        // Fe (55.845 amu) at 7500 K, 259.9 nm
        let fwhm = proc.doppler_broadening_fwhm(259.9, 7500.0, 55.845);
        assert!(fwhm > 0.0);
        // Expected ~0.003-0.01 nm for ICP conditions
        assert!(fwhm < 0.1, "Doppler FWHM should be small, got {} nm", fwhm);
    }

    #[test]
    fn test_doppler_broadening_temperature_dependence() {
        let proc = default_processor();
        let fwhm_low = proc.doppler_broadening_fwhm(300.0, 3000.0, 40.0);
        let fwhm_high = proc.doppler_broadening_fwhm(300.0, 10000.0, 40.0);
        assert!(fwhm_high > fwhm_low, "Higher T should give broader Doppler");
    }

    #[test]
    fn test_doppler_broadening_mass_dependence() {
        let proc = default_processor();
        // Light element (Li, 6.941 amu) vs heavy (Pb, 207.2 amu)
        let fwhm_light = proc.doppler_broadening_fwhm(400.0, 7500.0, 6.941);
        let fwhm_heavy = proc.doppler_broadening_fwhm(400.0, 7500.0, 207.2);
        assert!(
            fwhm_light > fwhm_heavy,
            "Lighter element should have broader Doppler: Li={}, Pb={}",
            fwhm_light,
            fwhm_heavy
        );
    }

    // --- Typical temperature ---

    #[test]
    fn test_typical_temperature_icp() {
        let proc = AesProcessor::new(AesConfig {
            plasma_type: PlasmaType::Icp,
            ..AesConfig::default()
        });
        assert_eq!(proc.typical_temperature_k(), 7500.0);
    }

    #[test]
    fn test_typical_temperature_flame() {
        let proc = AesProcessor::new(AesConfig {
            plasma_type: PlasmaType::Flame,
            ..AesConfig::default()
        });
        assert_eq!(proc.typical_temperature_k(), 2300.0);
    }

    #[test]
    fn test_typical_temperature_dcp() {
        let proc = AesProcessor::new(AesConfig {
            plasma_type: PlasmaType::Dcp,
            ..AesConfig::default()
        });
        assert_eq!(proc.typical_temperature_k(), 5000.0);
    }

    // --- Polynomial helpers ---

    #[test]
    fn test_poly_eval_linear() {
        // y = 2*x + 3 => coeffs = [2, 3]
        let val = poly_eval(&[2.0, 3.0], 5.0);
        assert!((val - 13.0).abs() < 1e-10);
    }

    #[test]
    fn test_poly_eval_quadratic() {
        // y = x^2 - 2x + 1 => coeffs = [1, -2, 1]
        let val = poly_eval(&[1.0, -2.0, 1.0], 3.0);
        assert!((val - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_poly_fit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![1.0, 3.0, 5.0, 7.0, 9.0]; // y = 2x + 1
        let coeffs = poly_fit(&x, &y, 1);
        assert_eq!(coeffs.len(), 2);
        assert!((coeffs[0] - 2.0).abs() < 1e-6, "slope should be 2, got {}", coeffs[0]);
        assert!((coeffs[1] - 1.0).abs() < 1e-6, "intercept should be 1, got {}", coeffs[1]);
    }

    // --- Config defaults ---

    #[test]
    fn test_default_config() {
        let config = AesConfig::default();
        assert_eq!(config.plasma_type, PlasmaType::Icp);
        assert_eq!(config.viewing_mode, ViewingMode::Axial);
        assert!((config.wavelength_min_nm - 190.0).abs() < 1e-6);
        assert!((config.wavelength_max_nm - 800.0).abs() < 1e-6);
        assert!((config.integration_time_s - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_predict_concentration_linear() {
        let proc = default_processor();
        let conc = vec![0.0, 5.0, 10.0];
        let ints = vec![100.0, 600.0, 1100.0]; // y = 100*x + 100
        let cal = proc.calibration_curve(&conc, &ints, CalibrationFit::Linear);
        let predicted = proc.predict_concentration(&cal, 350.0);
        assert!((predicted - 2.5).abs() < 0.5, "Should predict ~2.5 ppm, got {}", predicted);
    }
}

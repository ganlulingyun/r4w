// trace:FR-FLAME-PHOTOMETRY | ai:claude
//! # Flame Photometry Emission Processor
//!
//! Implements flame photometry (flame emission spectroscopy) signal processing
//! for alkali and alkaline earth metal analysis. Flame photometry is a quantitative
//! analytical technique for determining concentrations of alkali metals (Li, Na, K,
//! Rb, Cs) and some alkaline earth metals (Ca, Sr, Ba) by measuring the intensity
//! of light emitted when a metal salt solution is introduced into a flame.
//!
//! ## Key Components
//!
//! - **FlameModel** - Temperature models for air-propane, air-acetylene, N2O-acetylene
//! - **EmissionLines** - Characteristic wavelengths for alkali/alkaline earth metals
//! - **InternalStandard** - Li or Cs normalization for nebulization correction
//! - **CalibrationCurve** - Linear calibration with self-absorption correction
//! - **InterferenceCorrection** - Ionization, spectral, and chemical interference
//! - **NebulizerModel** - Nebulizer efficiency and aerosol transport modeling
//! - **SignalProcessing** - Background subtraction, drift correction, averaging
//! - **ClinicalApplication** - Serum electrolyte analysis (Na+, K+, Li+)
//! - **FlamePhotometerSession** - Complete measurement workflow

use std::f64::consts::PI;

// ============================================================================
// Physical constants
// ============================================================================

/// Boltzmann constant in eV/K
const K_BOLTZMANN_EV: f64 = 8.617333262e-5;

/// Boltzmann constant in J/K
const K_BOLTZMANN_J: f64 = 1.380649e-23;

/// Planck constant in J*s
const H_PLANCK: f64 = 6.62607015e-34;

/// Speed of light in m/s
const C_LIGHT: f64 = 2.99792458e8;

/// Electron charge in C
const Q_ELECTRON: f64 = 1.602176634e-19;

/// Electron mass in kg
const M_ELECTRON: f64 = 9.1093837015e-31;

// ============================================================================
// Flame Model
// ============================================================================

/// Types of flame used in flame photometry
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlameType {
    /// Air-propane flame, ~1925 degC
    AirPropane,
    /// Air-acetylene flame, ~2300 degC
    AirAcetylene,
    /// Nitrous oxide - acetylene flame, ~2950 degC
    N2OAcetylene,
}

impl FlameType {
    /// Returns the approximate flame temperature in Kelvin
    pub fn temperature_k(&self) -> f64 {
        match self {
            FlameType::AirPropane => 2198.0,     // 1925 C
            FlameType::AirAcetylene => 2573.0,   // 2300 C
            FlameType::N2OAcetylene => 3223.0,    // 2950 C
        }
    }

    /// Returns the approximate flame temperature in Celsius
    pub fn temperature_c(&self) -> f64 {
        self.temperature_k() - 273.15
    }
}

/// Flame model for emission spectroscopy
#[derive(Debug, Clone)]
pub struct FlameModel {
    /// The flame type
    pub flame_type: FlameType,
    /// Temperature in Kelvin (can be adjusted from default)
    pub temperature_k: f64,
    /// Flame path length in cm
    pub path_length_cm: f64,
    /// Flame cross-section area in cm^2
    pub cross_section_cm2: f64,
}

impl FlameModel {
    /// Create a new flame model with default parameters
    pub fn new(flame_type: FlameType) -> Self {
        Self {
            flame_type,
            temperature_k: flame_type.temperature_k(),
            path_length_cm: 5.0,
            cross_section_cm2: 1.0,
        }
    }

    /// Create with custom temperature
    pub fn with_temperature(flame_type: FlameType, temp_k: f64) -> Self {
        Self {
            flame_type,
            temperature_k: temp_k,
            path_length_cm: 5.0,
            cross_section_cm2: 1.0,
        }
    }

    /// Compute the Boltzmann fraction of atoms in the excited state
    pub fn boltzmann_fraction(&self, g_excited: f64, g_ground: f64, delta_e_ev: f64) -> f64 {
        boltzmann_fraction(g_excited, g_ground, delta_e_ev, self.temperature_k)
    }

    /// Compute emission intensity given number of excited atoms and transition probability
    pub fn emission_intensity(&self, n_excited: f64, transition_prob: f64, wavelength_nm: f64) -> f64 {
        let h_nu: f64 = H_PLANCK * C_LIGHT / (wavelength_nm * 1.0e-9);
        emission_intensity(n_excited, transition_prob, h_nu)
    }

    /// Compute ionization fraction using Saha equation
    pub fn ionization_fraction(&self, ip_ev: f64, ne: f64) -> f64 {
        ionization_fraction(self.temperature_k, ip_ev, ne)
    }

    /// Estimate the total atom number density in the flame from a concentration
    /// in ppm (mg/L) and nebulizer efficiency
    pub fn atom_density(&self, concentration_ppm: f64, nebulizer_efficiency: f64) -> f64 {
        // Rough estimate: at flame conditions, using ideal gas approximation
        // n = concentration * nebulizer_eff * transport_factor
        // For a typical flame photometer, about 1e10 atoms/cm^3 per ppm
        let base_density: f64 = 1.0e10; // atoms/cm^3 per ppm (approximate)
        concentration_ppm * nebulizer_efficiency * base_density
    }
}

// ============================================================================
// Emission Lines
// ============================================================================

/// Element identifiers for flame photometry
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Element {
    /// Sodium - Na D-line doublet at 589 nm
    Sodium,
    /// Potassium at 766 nm
    Potassium,
    /// Lithium at 671 nm
    Lithium,
    /// Calcium at 622 nm
    Calcium,
    /// Barium at 554 nm
    Barium,
    /// Strontium at 461 nm
    Strontium,
    /// Cesium at 852 nm
    Cesium,
    /// Rubidium at 780 nm
    Rubidium,
}

/// Emission line data for a specific element
#[derive(Debug, Clone)]
pub struct EmissionLine {
    /// Element
    pub element: Element,
    /// Primary emission wavelength in nm
    pub wavelength_nm: f64,
    /// Secondary wavelength in nm (if doublet)
    pub secondary_wavelength_nm: Option<f64>,
    /// Energy of upper level in eV
    pub upper_level_ev: f64,
    /// Statistical weight of upper (excited) level
    pub g_upper: f64,
    /// Statistical weight of lower (ground) level
    pub g_lower: f64,
    /// Transition probability (Einstein A coefficient) in s^-1
    pub transition_prob: f64,
    /// First ionization potential in eV
    pub ionization_potential_ev: f64,
    /// Color of emission
    pub color: &'static str,
}

/// Database of emission lines used in flame photometry
pub struct EmissionLines;

impl EmissionLines {
    /// Get emission line data for a specific element
    pub fn get(element: Element) -> EmissionLine {
        match element {
            Element::Sodium => EmissionLine {
                element: Element::Sodium,
                wavelength_nm: 589.0,
                secondary_wavelength_nm: Some(589.6), // D1 and D2 lines
                upper_level_ev: 2.104,
                g_upper: 4.0,  // 3p 2P3/2
                g_lower: 2.0,  // 3s 2S1/2
                transition_prob: 6.16e7,
                ionization_potential_ev: 5.139,
                color: "yellow",
            },
            Element::Potassium => EmissionLine {
                element: Element::Potassium,
                wavelength_nm: 766.5,
                secondary_wavelength_nm: Some(769.9),
                upper_level_ev: 1.617,
                g_upper: 4.0,
                g_lower: 2.0,
                transition_prob: 3.87e7,
                ionization_potential_ev: 4.341,
                color: "violet/red",
            },
            Element::Lithium => EmissionLine {
                element: Element::Lithium,
                wavelength_nm: 670.8,
                secondary_wavelength_nm: None,
                upper_level_ev: 1.848,
                g_upper: 6.0,  // 2p 2P
                g_lower: 2.0,  // 2s 2S1/2
                transition_prob: 3.69e7,
                ionization_potential_ev: 5.392,
                color: "red",
            },
            Element::Calcium => EmissionLine {
                element: Element::Calcium,
                wavelength_nm: 622.0,
                secondary_wavelength_nm: Some(553.0), // CaOH band
                upper_level_ev: 2.933,
                g_upper: 3.0,
                g_lower: 1.0,
                transition_prob: 2.18e8,
                ionization_potential_ev: 6.113,
                color: "orange-red",
            },
            Element::Barium => EmissionLine {
                element: Element::Barium,
                wavelength_nm: 553.6,
                secondary_wavelength_nm: None,
                upper_level_ev: 2.239,
                g_upper: 3.0,
                g_lower: 1.0,
                transition_prob: 1.19e8,
                ionization_potential_ev: 5.212,
                color: "green",
            },
            Element::Strontium => EmissionLine {
                element: Element::Strontium,
                wavelength_nm: 460.7,
                secondary_wavelength_nm: None,
                upper_level_ev: 2.690,
                g_upper: 3.0,
                g_lower: 1.0,
                transition_prob: 2.01e8,
                ionization_potential_ev: 5.695,
                color: "red",
            },
            Element::Cesium => EmissionLine {
                element: Element::Cesium,
                wavelength_nm: 852.1,
                secondary_wavelength_nm: Some(894.3),
                upper_level_ev: 1.455,
                g_upper: 4.0,
                g_lower: 2.0,
                transition_prob: 3.28e7,
                ionization_potential_ev: 3.894,
                color: "blue",
            },
            Element::Rubidium => EmissionLine {
                element: Element::Rubidium,
                wavelength_nm: 780.0,
                secondary_wavelength_nm: Some(794.8),
                upper_level_ev: 1.589,
                g_upper: 4.0,
                g_lower: 2.0,
                transition_prob: 3.81e7,
                ionization_potential_ev: 4.177,
                color: "red",
            },
        }
    }

    /// Get all supported elements
    pub fn all_elements() -> Vec<Element> {
        vec![
            Element::Sodium,
            Element::Potassium,
            Element::Lithium,
            Element::Calcium,
            Element::Barium,
            Element::Strontium,
            Element::Cesium,
            Element::Rubidium,
        ]
    }

    /// Calculate the energy of a photon at the given wavelength (in eV)
    pub fn photon_energy_ev(wavelength_nm: f64) -> f64 {
        let energy_j: f64 = H_PLANCK * C_LIGHT / (wavelength_nm * 1.0e-9);
        energy_j / Q_ELECTRON
    }

    /// Calculate wavelength from energy in eV
    pub fn wavelength_from_energy(energy_ev: f64) -> f64 {
        let energy_j: f64 = energy_ev * Q_ELECTRON;
        let wavelength_m: f64 = H_PLANCK * C_LIGHT / energy_j;
        wavelength_m * 1.0e9
    }
}

// ============================================================================
// Internal Standard
// ============================================================================

/// Internal standard element choice
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InternalStandardElement {
    /// Lithium as internal standard (commonly used for Na, K analysis)
    Lithium,
    /// Cesium as internal standard
    Cesium,
}

/// Internal standard method for signal normalization
#[derive(Debug, Clone)]
pub struct InternalStandard {
    /// The internal standard element
    pub element: InternalStandardElement,
    /// Concentration of internal standard added (ppm)
    pub concentration_ppm: f64,
    /// Reference signal intensity of the internal standard
    reference_signal: f64,
}

impl InternalStandard {
    /// Create a new internal standard
    pub fn new(element: InternalStandardElement, concentration_ppm: f64) -> Self {
        Self {
            element,
            concentration_ppm,
            reference_signal: 0.0,
        }
    }

    /// Set the reference signal intensity (measured from calibration blank + IS)
    pub fn set_reference(&mut self, signal: f64) {
        self.reference_signal = signal;
    }

    /// Get the emission line for the internal standard
    pub fn emission_line(&self) -> EmissionLine {
        match self.element {
            InternalStandardElement::Lithium => EmissionLines::get(Element::Lithium),
            InternalStandardElement::Cesium => EmissionLines::get(Element::Cesium),
        }
    }

    /// Normalize an analyte signal using the internal standard ratio method
    /// Returns the ratio of analyte signal to internal standard signal
    pub fn normalize(&self, analyte_signal: f64, is_signal: f64) -> f64 {
        if is_signal.abs() < 1.0e-15 {
            return 0.0;
        }
        analyte_signal / is_signal
    }

    /// Compute the corrected signal using internal standard ratio and reference
    pub fn corrected_signal(&self, analyte_signal: f64, is_signal: f64) -> f64 {
        if is_signal.abs() < 1.0e-15 || self.reference_signal.abs() < 1.0e-15 {
            return 0.0;
        }
        (analyte_signal / is_signal) * self.reference_signal
    }

    /// Compute the correction factor for a given IS measurement vs reference
    pub fn correction_factor(&self, is_signal: f64) -> f64 {
        if is_signal.abs() < 1.0e-15 {
            return 1.0;
        }
        self.reference_signal / is_signal
    }
}

// ============================================================================
// Calibration Curve
// ============================================================================

/// A calibration point (concentration, signal)
#[derive(Debug, Clone, Copy)]
pub struct CalibrationPoint {
    /// Concentration in ppm (mg/L) or mmol/L
    pub concentration: f64,
    /// Measured emission signal intensity
    pub signal: f64,
}

/// Calibration curve for quantitative analysis
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    /// Calibration points
    points: Vec<CalibrationPoint>,
    /// Slope of linear regression
    slope: f64,
    /// Intercept of linear regression
    intercept: f64,
    /// Correlation coefficient r^2
    r_squared: f64,
    /// Self-absorption coefficient (for high-concentration curvature)
    self_absorption_coeff: f64,
    /// Whether the curve has been fitted
    fitted: bool,
}

impl CalibrationCurve {
    /// Create a new empty calibration curve
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            slope: 0.0,
            intercept: 0.0,
            r_squared: 0.0,
            self_absorption_coeff: 0.0,
            fitted: false,
        }
    }

    /// Add a calibration point
    pub fn add_point(&mut self, concentration: f64, signal: f64) {
        self.points.push(CalibrationPoint { concentration, signal });
        self.fitted = false;
    }

    /// Add multiple calibration points
    pub fn add_points(&mut self, points: &[(f64, f64)]) {
        for &(conc, sig) in points {
            self.points.push(CalibrationPoint {
                concentration: conc,
                signal: sig,
            });
        }
        self.fitted = false;
    }

    /// Fit a linear calibration curve using least-squares regression
    pub fn fit_linear(&mut self) -> bool {
        let n: f64 = self.points.len() as f64;
        if n < 2.0 {
            return false;
        }

        let sum_x: f64 = self.points.iter().map(|p| p.concentration).sum();
        let sum_y: f64 = self.points.iter().map(|p| p.signal).sum();
        let sum_xy: f64 = self.points.iter().map(|p| p.concentration * p.signal).sum();
        let sum_x2: f64 = self.points.iter().map(|p| p.concentration * p.concentration).sum();

        let denom: f64 = n * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1.0e-30 {
            return false;
        }

        self.slope = (n * sum_xy - sum_x * sum_y) / denom;
        self.intercept = (sum_y - self.slope * sum_x) / n;

        // Calculate R-squared
        let mean_y: f64 = sum_y / n;
        let ss_tot: f64 = self.points.iter().map(|p| {
            let d: f64 = p.signal - mean_y;
            d * d
        }).sum();
        let ss_res: f64 = self.points.iter().map(|p| {
            let predicted: f64 = self.slope * p.concentration + self.intercept;
            let d: f64 = p.signal - predicted;
            d * d
        }).sum();

        self.r_squared = if ss_tot > 1.0e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
        self.fitted = true;
        true
    }

    /// Set self-absorption coefficient for high-concentration correction
    pub fn set_self_absorption(&mut self, coeff: f64) {
        self.self_absorption_coeff = coeff;
    }

    /// Predict signal from concentration using linear model
    pub fn predict_signal(&self, concentration: f64) -> f64 {
        if !self.fitted {
            return 0.0;
        }
        let linear: f64 = self.slope * concentration + self.intercept;
        if self.self_absorption_coeff > 0.0 {
            // Self-absorption reduces signal at high concentrations
            // I = I_linear * exp(-k * C)
            linear * (-self.self_absorption_coeff * concentration).exp()
        } else {
            linear
        }
    }

    /// Calculate concentration from measured signal using linear model
    pub fn concentration_from_signal(&self, signal: f64) -> f64 {
        if !self.fitted || self.slope.abs() < 1.0e-30 {
            return 0.0;
        }
        if self.self_absorption_coeff > 0.0 {
            // Need iterative solution for self-absorption case
            self.concentration_iterative(signal)
        } else {
            (signal - self.intercept) / self.slope
        }
    }

    /// Iterative solution for concentration when self-absorption is present
    fn concentration_iterative(&self, signal: f64) -> f64 {
        // Newton-Raphson iteration
        let mut c: f64 = (signal - self.intercept) / self.slope; // Initial guess
        if c < 0.0 {
            c = 0.0;
        }
        let k: f64 = self.self_absorption_coeff;

        for _ in 0..50 {
            let f_val: f64 = (self.slope * c + self.intercept) * (-k * c).exp() - signal;
            let f_deriv: f64 = (self.slope - k * (self.slope * c + self.intercept)) * (-k * c).exp();
            if f_deriv.abs() < 1.0e-30 {
                break;
            }
            let delta: f64 = f_val / f_deriv;
            c -= delta;
            if c < 0.0 {
                c = 0.0;
            }
            if delta.abs() < 1.0e-10 {
                break;
            }
        }
        c
    }

    /// Get the slope
    pub fn slope(&self) -> f64 {
        self.slope
    }

    /// Get the intercept
    pub fn intercept(&self) -> f64 {
        self.intercept
    }

    /// Get R-squared value
    pub fn r_squared(&self) -> f64 {
        self.r_squared
    }

    /// Check if the curve is fitted
    pub fn is_fitted(&self) -> bool {
        self.fitted
    }

    /// Determine the linear working range (where residuals are < threshold %)
    pub fn working_range(&self, max_residual_pct: f64) -> Option<(f64, f64)> {
        if !self.fitted || self.points.is_empty() {
            return None;
        }

        let mut valid_concs: Vec<f64> = Vec::new();
        for p in &self.points {
            let predicted: f64 = self.slope * p.concentration + self.intercept;
            if predicted.abs() > 1.0e-15 {
                let residual_pct: f64 = ((p.signal - predicted) / predicted).abs() * 100.0;
                if residual_pct < max_residual_pct {
                    valid_concs.push(p.concentration);
                }
            } else if p.signal.abs() < 1.0e-15 {
                valid_concs.push(p.concentration);
            }
        }

        if valid_concs.is_empty() {
            return None;
        }

        let min_c: f64 = valid_concs.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_c: f64 = valid_concs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        Some((min_c, max_c))
    }

    /// Number of calibration points
    pub fn num_points(&self) -> usize {
        self.points.len()
    }

    /// Limit of detection (3*sigma of blank / slope)
    pub fn limit_of_detection(&self, blank_std: f64) -> f64 {
        if self.slope.abs() < 1.0e-30 {
            return f64::INFINITY;
        }
        3.0 * blank_std / self.slope
    }

    /// Limit of quantification (10*sigma of blank / slope)
    pub fn limit_of_quantification(&self, blank_std: f64) -> f64 {
        if self.slope.abs() < 1.0e-30 {
            return f64::INFINITY;
        }
        10.0 * blank_std / self.slope
    }
}

// ============================================================================
// Interference Correction
// ============================================================================

/// Types of interference in flame photometry
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterferenceType {
    /// Ionization interference - easily ionized elements
    Ionization,
    /// Spectral interference - overlapping emission lines
    Spectral,
    /// Chemical interference - refractory compound formation
    Chemical,
    /// Matrix interference - physical property changes
    Matrix,
}

/// Interference correction parameters
#[derive(Debug, Clone)]
pub struct InterferenceCorrection {
    /// Ionization buffer element (typically Cs or K)
    pub ionization_buffer: Option<Element>,
    /// Ionization buffer concentration in ppm
    pub buffer_concentration_ppm: f64,
    /// Spectral bandpass of monochromator in nm
    pub spectral_bandpass_nm: f64,
    /// Chemical releasing agent (e.g., La or Sr for Ca interference)
    pub releasing_agent: bool,
    /// Matrix matching applied
    pub matrix_matched: bool,
}

impl InterferenceCorrection {
    /// Create default correction parameters
    pub fn new() -> Self {
        Self {
            ionization_buffer: None,
            buffer_concentration_ppm: 0.0,
            spectral_bandpass_nm: 2.0,
            releasing_agent: false,
            matrix_matched: false,
        }
    }

    /// Add ionization buffer (Cs or K at high concentration to suppress ionization)
    pub fn with_ionization_buffer(mut self, element: Element, concentration_ppm: f64) -> Self {
        self.ionization_buffer = Some(element);
        self.buffer_concentration_ppm = concentration_ppm;
        self
    }

    /// Set spectral bandpass
    pub fn with_spectral_bandpass(mut self, bandpass_nm: f64) -> Self {
        self.spectral_bandpass_nm = bandpass_nm;
        self
    }

    /// Enable releasing agent
    pub fn with_releasing_agent(mut self) -> Self {
        self.releasing_agent = true;
        self
    }

    /// Enable matrix matching
    pub fn with_matrix_matching(mut self) -> Self {
        self.matrix_matched = true;
        self
    }

    /// Estimate ionization suppression factor with buffer present
    /// Returns fraction of neutral atoms remaining (1.0 = no ionization)
    pub fn ionization_suppression_factor(
        &self,
        analyte_ip_ev: f64,
        temperature_k: f64,
    ) -> f64 {
        // Without buffer, calculate natural ionization fraction
        let ne_natural: f64 = 1.0e12; // typical electron density in flame
        let alpha_natural: f64 = ionization_fraction(temperature_k, analyte_ip_ev, ne_natural);
        let neutral_without: f64 = 1.0 - alpha_natural;

        // With ionization buffer, electron density is much higher
        if self.ionization_buffer.is_some() && self.buffer_concentration_ppm > 0.0 {
            // Buffer increases electron density, suppressing analyte ionization
            let ne_buffered: f64 = ne_natural + self.buffer_concentration_ppm * 1.0e9;
            let alpha_buffered: f64 = ionization_fraction(temperature_k, analyte_ip_ev, ne_buffered);
            let neutral_with: f64 = 1.0 - alpha_buffered;
            if neutral_without > 1.0e-15 {
                neutral_with / neutral_without
            } else {
                1.0
            }
        } else {
            1.0
        }
    }

    /// Check for spectral overlap between two elements
    pub fn has_spectral_overlap(&self, elem1: Element, elem2: Element) -> bool {
        let line1: EmissionLine = EmissionLines::get(elem1);
        let line2: EmissionLine = EmissionLines::get(elem2);
        let diff: f64 = (line1.wavelength_nm - line2.wavelength_nm).abs();
        diff < self.spectral_bandpass_nm
    }

    /// Calculate spectral overlap correction factor
    /// Returns the fraction of signal that is interference
    pub fn spectral_overlap_fraction(
        &self,
        analyte: Element,
        interferent: Element,
        interferent_concentration: f64,
        analyte_sensitivity: f64,
    ) -> f64 {
        let line_a: EmissionLine = EmissionLines::get(analyte);
        let line_i: EmissionLine = EmissionLines::get(interferent);

        let separation: f64 = (line_a.wavelength_nm - line_i.wavelength_nm).abs();
        if separation >= self.spectral_bandpass_nm {
            return 0.0;
        }

        // Gaussian overlap model
        let sigma: f64 = self.spectral_bandpass_nm / 2.355; // FWHM to sigma
        let overlap: f64 = (-0.5 * (separation / sigma).powi(2)).exp();

        // Fraction depends on interferent signal relative to analyte
        let interferent_signal: f64 = interferent_concentration * analyte_sensitivity * overlap;
        if interferent_signal + analyte_sensitivity > 1.0e-15 {
            interferent_signal / (interferent_signal + analyte_sensitivity)
        } else {
            0.0
        }
    }

    /// Apply chemical interference correction factor
    pub fn chemical_correction_factor(&self, base_signal: f64) -> f64 {
        if self.releasing_agent {
            // Releasing agent recovers ~95% of lost signal
            base_signal * 1.0
        } else {
            // Without releasing agent, Ca can lose 10-30% due to phosphate
            base_signal * 0.80
        }
    }
}

// ============================================================================
// Nebulizer Model
// ============================================================================

/// Nebulizer types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NebulizerType {
    /// Concentric (pneumatic) nebulizer
    Concentric,
    /// Cross-flow nebulizer
    CrossFlow,
    /// Ultrasonic nebulizer
    Ultrasonic,
}

/// Model of the nebulizer and spray chamber
#[derive(Debug, Clone)]
pub struct NebulizerModel {
    /// Nebulizer type
    pub nebulizer_type: NebulizerType,
    /// Sample uptake rate in mL/min
    pub uptake_rate_ml_min: f64,
    /// Nebulization efficiency (fraction of sample reaching flame)
    pub efficiency: f64,
    /// Drain fraction (lost to waste)
    pub drain_fraction: f64,
    /// Mean droplet diameter in micrometers
    pub mean_droplet_um: f64,
}

impl NebulizerModel {
    /// Create a nebulizer model with typical parameters
    pub fn new(nebulizer_type: NebulizerType) -> Self {
        match nebulizer_type {
            NebulizerType::Concentric => Self {
                nebulizer_type,
                uptake_rate_ml_min: 5.0,
                efficiency: 0.05,  // 5% typical
                drain_fraction: 0.95,
                mean_droplet_um: 20.0,
            },
            NebulizerType::CrossFlow => Self {
                nebulizer_type,
                uptake_rate_ml_min: 3.0,
                efficiency: 0.03,
                drain_fraction: 0.97,
                mean_droplet_um: 30.0,
            },
            NebulizerType::Ultrasonic => Self {
                nebulizer_type,
                uptake_rate_ml_min: 2.0,
                efficiency: 0.15,
                drain_fraction: 0.85,
                mean_droplet_um: 5.0,
            },
        }
    }

    /// Calculate the mass flow rate of analyte reaching the flame (ug/min)
    pub fn analyte_mass_flow(&self, concentration_ppm: f64) -> f64 {
        // ppm = mg/L = ug/mL
        concentration_ppm * self.uptake_rate_ml_min * self.efficiency
    }

    /// Calculate the effective sample transport rate
    pub fn effective_transport_rate(&self) -> f64 {
        self.uptake_rate_ml_min * self.efficiency
    }

    /// Drain correction factor for signal normalization
    pub fn drain_correction_factor(&self, reference_drain: f64) -> f64 {
        if reference_drain.abs() < 1.0e-15 {
            return 1.0;
        }
        self.drain_fraction / reference_drain
    }

    /// Calculate the Sauter mean diameter using the Nukiyama-Tanasawa equation
    /// gas_velocity in m/s, surface_tension in N/m, viscosity in Pa*s, density in kg/m^3
    pub fn sauter_mean_diameter(
        &self,
        gas_velocity: f64,
        surface_tension: f64,
        viscosity: f64,
        density: f64,
    ) -> f64 {
        if gas_velocity.abs() < 1.0e-15 || density.abs() < 1.0e-15 {
            return self.mean_droplet_um;
        }
        // Simplified Nukiyama-Tanasawa: d = 585/v * sqrt(sigma/rho) + 597*(mu/sqrt(sigma*rho))^0.45
        let term1: f64 = 585.0 / gas_velocity * (surface_tension / density).sqrt();
        let denom: f64 = (surface_tension * density).sqrt();
        let term2: f64 = if denom > 1.0e-15 {
            597.0 * (viscosity / denom).powf(0.45)
        } else {
            0.0
        };
        (term1 + term2) * 1.0e6 // Convert to micrometers
    }
}

// ============================================================================
// Signal Processing
// ============================================================================

/// Signal processing for flame photometry measurements
#[derive(Debug, Clone)]
pub struct SignalProcessor {
    /// Background signal level
    background: f64,
    /// Drift reference signal
    drift_reference: f64,
    /// Signal history for averaging
    history: Vec<f64>,
    /// Maximum history length
    max_history: usize,
}

impl SignalProcessor {
    /// Create a new signal processor
    pub fn new(max_history: usize) -> Self {
        Self {
            background: 0.0,
            drift_reference: 0.0,
            history: Vec::new(),
            max_history,
        }
    }

    /// Set background signal (measured with blank)
    pub fn set_background(&mut self, bg: f64) {
        self.background = bg;
    }

    /// Set drift reference signal
    pub fn set_drift_reference(&mut self, ref_signal: f64) {
        self.drift_reference = ref_signal;
    }

    /// Subtract background from a raw signal
    pub fn background_subtract(&self, raw_signal: f64) -> f64 {
        raw_signal - self.background
    }

    /// Apply drift correction using a reference standard measurement
    pub fn drift_correct(&self, signal: f64, current_reference: f64) -> f64 {
        if current_reference.abs() < 1.0e-15 || self.drift_reference.abs() < 1.0e-15 {
            return signal;
        }
        signal * (self.drift_reference / current_reference)
    }

    /// Add a measurement to history and return the running average
    pub fn add_measurement(&mut self, signal: f64) -> f64 {
        self.history.push(signal);
        if self.history.len() > self.max_history {
            self.history.remove(0);
        }
        self.average()
    }

    /// Calculate the average of stored measurements
    pub fn average(&self) -> f64 {
        if self.history.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.history.iter().sum();
        sum / self.history.len() as f64
    }

    /// Calculate the standard deviation of stored measurements
    pub fn std_deviation(&self) -> f64 {
        if self.history.len() < 2 {
            return 0.0;
        }
        let mean: f64 = self.average();
        let n: f64 = self.history.len() as f64;
        let variance: f64 = self.history.iter().map(|&x| {
            let d: f64 = x - mean;
            d * d
        }).sum::<f64>() / (n - 1.0);
        variance.sqrt()
    }

    /// Calculate the coefficient of variation (% RSD)
    pub fn coefficient_of_variation(&self) -> f64 {
        let mean: f64 = self.average();
        if mean.abs() < 1.0e-15 {
            return 0.0;
        }
        (self.std_deviation() / mean) * 100.0
    }

    /// Calculate signal-to-noise ratio
    pub fn signal_to_noise(&self, signal: f64, noise_std: f64) -> f64 {
        if noise_std.abs() < 1.0e-15 {
            return f64::INFINITY;
        }
        signal / noise_std
    }

    /// Check stability: CV should be < threshold (typically 1-2%)
    pub fn is_stable(&self, max_cv_pct: f64) -> bool {
        self.coefficient_of_variation() < max_cv_pct
    }

    /// Perform a median filter on the history for outlier removal
    pub fn median_filtered(&self) -> f64 {
        if self.history.is_empty() {
            return 0.0;
        }
        let mut sorted: Vec<f64> = self.history.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n: usize = sorted.len();
        if n % 2 == 0 {
            (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
        } else {
            sorted[n / 2]
        }
    }

    /// Clear history
    pub fn clear(&mut self) {
        self.history.clear();
    }

    /// Number of measurements stored
    pub fn count(&self) -> usize {
        self.history.len()
    }

    /// Apply a moving average filter to a signal vector
    pub fn moving_average(data: &[f64], window: usize) -> Vec<f64> {
        if window == 0 || data.is_empty() {
            return data.to_vec();
        }
        let mut result: Vec<f64> = Vec::with_capacity(data.len());
        for i in 0..data.len() {
            let start: usize = if i >= window / 2 { i - window / 2 } else { 0 };
            let end: usize = if i + window / 2 + 1 <= data.len() {
                i + window / 2 + 1
            } else {
                data.len()
            };
            let count: f64 = (end - start) as f64;
            let sum: f64 = data[start..end].iter().sum();
            result.push(sum / count);
        }
        result
    }

    /// Compute the signal-to-noise ratio of a data series
    pub fn compute_snr(signal_data: &[f64], noise_data: &[f64]) -> f64 {
        if noise_data.is_empty() || signal_data.is_empty() {
            return 0.0;
        }
        let signal_mean: f64 = signal_data.iter().sum::<f64>() / signal_data.len() as f64;
        let noise_std: f64 = {
            let noise_mean: f64 = noise_data.iter().sum::<f64>() / noise_data.len() as f64;
            let n: f64 = noise_data.len() as f64;
            let var: f64 = noise_data.iter().map(|&x| {
                let d: f64 = x - noise_mean;
                d * d
            }).sum::<f64>() / n;
            var.sqrt()
        };
        if noise_std.abs() < 1.0e-15 {
            return f64::INFINITY;
        }
        signal_mean / noise_std
    }
}

// ============================================================================
// Clinical Application
// ============================================================================

/// Clinical reference ranges for serum electrolytes
#[derive(Debug, Clone, Copy)]
pub struct ClinicalRange {
    /// Lower normal limit (mmol/L)
    pub low: f64,
    /// Upper normal limit (mmol/L)
    pub high: f64,
    /// Critical low value
    pub critical_low: f64,
    /// Critical high value
    pub critical_high: f64,
}

/// Clinical status of a result
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClinicalStatus {
    /// Within normal reference range
    Normal,
    /// Below normal reference range
    Low,
    /// Above normal reference range
    High,
    /// Critically low - immediate action needed
    CriticalLow,
    /// Critically high - immediate action needed
    CriticalHigh,
}

/// Clinical application of flame photometry for serum electrolytes
#[derive(Debug, Clone)]
pub struct ClinicalApplication {
    /// Dilution factor applied to serum samples
    pub dilution_factor: f64,
    /// Internal standard concentration in diluent (ppm)
    pub is_concentration_ppm: f64,
}

impl ClinicalApplication {
    /// Create a new clinical application with standard parameters
    pub fn new() -> Self {
        Self {
            dilution_factor: 200.0, // 1:200 typical serum dilution
            is_concentration_ppm: 15.0, // 15 ppm Li in diluent
        }
    }

    /// Create with custom dilution
    pub fn with_dilution(dilution_factor: f64) -> Self {
        Self {
            dilution_factor,
            is_concentration_ppm: 15.0,
        }
    }

    /// Get clinical reference range for serum sodium (mmol/L)
    pub fn sodium_range() -> ClinicalRange {
        ClinicalRange {
            low: 135.0,
            high: 145.0,
            critical_low: 120.0,
            critical_high: 160.0,
        }
    }

    /// Get clinical reference range for serum potassium (mmol/L)
    pub fn potassium_range() -> ClinicalRange {
        ClinicalRange {
            low: 3.5,
            high: 5.0,
            critical_low: 2.5,
            critical_high: 6.5,
        }
    }

    /// Get clinical reference range for serum lithium therapeutic level (mmol/L)
    pub fn lithium_therapeutic_range() -> ClinicalRange {
        ClinicalRange {
            low: 0.6,
            high: 1.2,
            critical_low: 0.0,
            critical_high: 2.0,
        }
    }

    /// Evaluate clinical status of a result
    pub fn evaluate_status(value: f64, range: &ClinicalRange) -> ClinicalStatus {
        if value <= range.critical_low {
            ClinicalStatus::CriticalLow
        } else if value >= range.critical_high {
            ClinicalStatus::CriticalHigh
        } else if value < range.low {
            ClinicalStatus::Low
        } else if value > range.high {
            ClinicalStatus::High
        } else {
            ClinicalStatus::Normal
        }
    }

    /// Convert measured concentration (ppm in diluted sample) to serum mmol/L
    pub fn to_serum_mmol_per_l(&self, measured_ppm: f64, atomic_weight: f64) -> f64 {
        // ppm = mg/L, multiply by dilution factor to get serum mg/L
        // then convert to mmol/L: divide by atomic weight (mg/mmol)
        let serum_mg_l: f64 = measured_ppm * self.dilution_factor;
        serum_mg_l / atomic_weight
    }

    /// Convert serum mmol/L to expected ppm in diluted sample
    pub fn from_serum_mmol_per_l(&self, serum_mmol_l: f64, atomic_weight: f64) -> f64 {
        let serum_mg_l: f64 = serum_mmol_l * atomic_weight;
        serum_mg_l / self.dilution_factor
    }

    /// Atomic weight for sodium (g/mol = mg/mmol)
    pub fn sodium_atomic_weight() -> f64 {
        22.99
    }

    /// Atomic weight for potassium (g/mol)
    pub fn potassium_atomic_weight() -> f64 {
        39.10
    }

    /// Atomic weight for lithium (g/mol)
    pub fn lithium_atomic_weight() -> f64 {
        6.941
    }

    /// Check if sample needs auto-dilution (concentration too high for linear range)
    pub fn needs_dilution(&self, measured_ppm: f64, max_linear_ppm: f64) -> bool {
        measured_ppm > max_linear_ppm
    }

    /// Calculate additional dilution factor needed
    pub fn additional_dilution_factor(&self, measured_ppm: f64, target_ppm: f64) -> f64 {
        if target_ppm.abs() < 1.0e-15 {
            return 1.0;
        }
        measured_ppm / target_ppm
    }

    /// Total dilution factor after additional dilution
    pub fn total_dilution(&self, additional_factor: f64) -> f64 {
        self.dilution_factor * additional_factor
    }
}

// ============================================================================
// Flame Photometer Session
// ============================================================================

/// Quality control result
#[derive(Debug, Clone)]
pub struct QcResult {
    /// Expected value
    pub expected: f64,
    /// Measured value
    pub measured: f64,
    /// Percent error
    pub percent_error: f64,
    /// Pass/fail (typically < 5% or < 10% error)
    pub passed: bool,
}

/// Measurement result from the flame photometer
#[derive(Debug, Clone)]
pub struct MeasurementResult {
    /// Element measured
    pub element: Element,
    /// Raw signal intensity
    pub raw_signal: f64,
    /// Background-corrected signal
    pub corrected_signal: f64,
    /// Concentration in ppm (in diluted sample)
    pub concentration_ppm: f64,
    /// Concentration in original sample units
    pub original_concentration: f64,
    /// Number of replicates
    pub replicates: usize,
    /// Standard deviation
    pub std_dev: f64,
    /// Coefficient of variation (%)
    pub cv_pct: f64,
}

/// State of the session
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SessionState {
    /// Not started
    Idle,
    /// Calibration phase
    Calibrating,
    /// Ready for measurements
    Ready,
    /// Measuring samples
    Measuring,
    /// QC check in progress
    QcCheck,
    /// Session complete
    Complete,
}

/// Complete flame photometer measurement session
#[derive(Debug, Clone)]
pub struct FlamePhotometerSession {
    /// Session state
    pub state: SessionState,
    /// Flame model
    pub flame: FlameModel,
    /// Nebulizer model
    pub nebulizer: NebulizerModel,
    /// Internal standard
    pub internal_standard: Option<InternalStandard>,
    /// Calibration curves (one per element)
    calibrations: Vec<(Element, CalibrationCurve)>,
    /// Signal processor
    processor: SignalProcessor,
    /// Interference corrections
    pub corrections: InterferenceCorrection,
    /// Clinical application settings (if clinical mode)
    pub clinical: Option<ClinicalApplication>,
    /// Measurement results
    results: Vec<MeasurementResult>,
    /// QC results
    qc_results: Vec<QcResult>,
}

impl FlamePhotometerSession {
    /// Create a new session with specified flame type
    pub fn new(flame_type: FlameType) -> Self {
        Self {
            state: SessionState::Idle,
            flame: FlameModel::new(flame_type),
            nebulizer: NebulizerModel::new(NebulizerType::Concentric),
            internal_standard: None,
            calibrations: Vec::new(),
            processor: SignalProcessor::new(10),
            corrections: InterferenceCorrection::new(),
            clinical: None,
            results: Vec::new(),
            qc_results: Vec::new(),
        }
    }

    /// Create a clinical analysis session
    pub fn clinical(flame_type: FlameType) -> Self {
        let mut session = Self::new(flame_type);
        session.clinical = Some(ClinicalApplication::new());
        session.internal_standard = Some(InternalStandard::new(
            InternalStandardElement::Lithium,
            15.0,
        ));
        session.corrections = InterferenceCorrection::new()
            .with_ionization_buffer(Element::Cesium, 1000.0);
        session
    }

    /// Set the nebulizer model
    pub fn set_nebulizer(&mut self, nebulizer: NebulizerModel) {
        self.nebulizer = nebulizer;
    }

    /// Set the internal standard
    pub fn set_internal_standard(&mut self, is: InternalStandard) {
        self.internal_standard = Some(is);
    }

    /// Begin calibration phase
    pub fn begin_calibration(&mut self) {
        self.state = SessionState::Calibrating;
        self.calibrations.clear();
    }

    /// Add calibration data for an element
    pub fn add_calibration(&mut self, element: Element, points: &[(f64, f64)]) {
        let mut curve = CalibrationCurve::new();
        curve.add_points(points);
        curve.fit_linear();
        self.calibrations.push((element, curve));
    }

    /// Finish calibration and check quality
    pub fn finish_calibration(&mut self) -> bool {
        let all_good: bool = self.calibrations.iter().all(|(_, c)| c.r_squared() > 0.99);
        if all_good || !self.calibrations.is_empty() {
            self.state = SessionState::Ready;
        }
        all_good
    }

    /// Set background from blank measurement
    pub fn set_background(&mut self, background: f64) {
        self.processor.set_background(background);
    }

    /// Set drift reference
    pub fn set_drift_reference(&mut self, reference: f64) {
        self.processor.set_drift_reference(reference);
    }

    /// Measure a sample - returns concentration
    pub fn measure(
        &mut self,
        element: Element,
        raw_signals: &[f64],
        is_signals: Option<&[f64]>,
    ) -> Option<MeasurementResult> {
        if self.state != SessionState::Ready && self.state != SessionState::Measuring {
            return None;
        }
        self.state = SessionState::Measuring;

        // Find calibration curve for element
        let cal_idx: Option<usize> = self.calibrations.iter().position(|(e, _)| *e == element);
        let cal_idx: usize = cal_idx?;

        // Process each replicate
        let mut corrected_signals: Vec<f64> = Vec::new();
        for (i, &raw) in raw_signals.iter().enumerate() {
            let bg_corrected: f64 = self.processor.background_subtract(raw);

            // Apply internal standard correction if available
            let corrected: f64 = if let (Some(ref is), Some(is_sigs)) =
                (&self.internal_standard, is_signals)
            {
                if i < is_sigs.len() {
                    is.normalize(bg_corrected, is_sigs[i])
                } else {
                    bg_corrected
                }
            } else {
                bg_corrected
            };

            corrected_signals.push(corrected);
        }

        // Calculate statistics
        let n: f64 = corrected_signals.len() as f64;
        if n < 1.0 {
            return None;
        }
        let mean: f64 = corrected_signals.iter().sum::<f64>() / n;
        let std_dev: f64 = if n > 1.0 {
            let var: f64 = corrected_signals
                .iter()
                .map(|&x| {
                    let d: f64 = x - mean;
                    d * d
                })
                .sum::<f64>()
                / (n - 1.0);
            var.sqrt()
        } else {
            0.0
        };
        let cv: f64 = if mean.abs() > 1.0e-15 {
            (std_dev / mean) * 100.0
        } else {
            0.0
        };

        // Convert to concentration using calibration curve
        let concentration: f64 = self.calibrations[cal_idx].1.concentration_from_signal(mean);

        // Calculate original concentration (accounting for dilution)
        let original: f64 = if let Some(ref clin) = self.clinical {
            // For clinical mode, convert to mmol/L
            let aw: f64 = match element {
                Element::Sodium => ClinicalApplication::sodium_atomic_weight(),
                Element::Potassium => ClinicalApplication::potassium_atomic_weight(),
                Element::Lithium => ClinicalApplication::lithium_atomic_weight(),
                _ => 1.0,
            };
            clin.to_serum_mmol_per_l(concentration, aw)
        } else {
            concentration
        };

        let result = MeasurementResult {
            element,
            raw_signal: raw_signals.iter().sum::<f64>() / raw_signals.len() as f64,
            corrected_signal: mean,
            concentration_ppm: concentration,
            original_concentration: original,
            replicates: raw_signals.len(),
            std_dev,
            cv_pct: cv,
        };

        self.results.push(result.clone());
        Some(result)
    }

    /// Perform a QC check
    pub fn qc_check(
        &mut self,
        element: Element,
        expected_concentration: f64,
        measured_signals: &[f64],
        max_error_pct: f64,
    ) -> Option<QcResult> {
        let result = self.measure(element, measured_signals, None)?;
        self.state = SessionState::QcCheck;
        let percent_error: f64 = if expected_concentration.abs() > 1.0e-15 {
            ((result.concentration_ppm - expected_concentration) / expected_concentration).abs()
                * 100.0
        } else {
            0.0
        };

        let qc = QcResult {
            expected: expected_concentration,
            measured: result.concentration_ppm,
            percent_error,
            passed: percent_error < max_error_pct,
        };

        self.qc_results.push(qc.clone());
        self.state = SessionState::Ready;
        Some(qc)
    }

    /// Get all measurement results
    pub fn results(&self) -> &[MeasurementResult] {
        &self.results
    }

    /// Get all QC results
    pub fn qc_results(&self) -> &[QcResult] {
        &self.qc_results
    }

    /// Complete the session
    pub fn complete(&mut self) {
        self.state = SessionState::Complete;
    }

    /// Get calibration curve for an element
    pub fn get_calibration(&self, element: Element) -> Option<&CalibrationCurve> {
        self.calibrations.iter().find(|(e, _)| *e == element).map(|(_, c)| c)
    }

    /// Check if auto-dilution is needed for a sample
    pub fn needs_auto_dilution(&self, element: Element, raw_signal: f64) -> bool {
        if let Some(cal) = self.get_calibration(element) {
            let conc: f64 = cal.concentration_from_signal(raw_signal);
            if let Some((_, max_c)) = cal.working_range(5.0) {
                return conc > max_c;
            }
        }
        false
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Calculate the Boltzmann fraction of atoms in an excited state
///
/// N_excited/N_total = (g_excited/g_ground) * exp(-delta_E / kT)
///
/// # Arguments
/// * `g_excited` - Statistical weight of excited state
/// * `g_ground` - Statistical weight of ground state
/// * `delta_e_ev` - Energy difference in eV
/// * `temp_k` - Temperature in Kelvin
pub fn boltzmann_fraction(g_excited: f64, g_ground: f64, delta_e_ev: f64, temp_k: f64) -> f64 {
    if g_ground.abs() < 1.0e-15 || temp_k.abs() < 1.0e-15 {
        return 0.0;
    }
    let ratio: f64 = g_excited / g_ground;
    let exponent: f64 = -delta_e_ev / (K_BOLTZMANN_EV * temp_k);
    let boltz: f64 = ratio * exponent.exp();
    // N_excited / N_total = boltz / (1 + boltz)
    boltz / (1.0 + boltz)
}

/// Calculate emission intensity
///
/// I = N_excited * A * h*nu
///
/// # Arguments
/// * `n_excited` - Number of atoms in excited state
/// * `transition_prob` - Einstein A coefficient (spontaneous emission rate, s^-1)
/// * `h_nu` - Photon energy in Joules
pub fn emission_intensity(n_excited: f64, transition_prob: f64, h_nu: f64) -> f64 {
    n_excited * transition_prob * h_nu
}

/// Calculate ionization fraction using the Saha equation
///
/// n_i * n_e / n_0 = (2 * g_i / g_0) * (2*pi*m_e*k*T/h^2)^(3/2) * exp(-IP/kT)
///
/// Returns fraction ionized (alpha = n_i / (n_0 + n_i))
///
/// # Arguments
/// * `temp_k` - Temperature in Kelvin
/// * `ip_ev` - Ionization potential in eV
/// * `ne` - Electron density in cm^-3
pub fn ionization_fraction(temp_k: f64, ip_ev: f64, ne: f64) -> f64 {
    if temp_k < 1.0 || ne.abs() < 1.0e-5 {
        return 0.0;
    }

    // Saha equation: S = (2 * gi/g0) * (2*pi*me*kT/h^2)^(3/2) * exp(-IP/kT)
    // For alkali metals, gi/g0 ~ 1 (assuming similar statistical weights)
    let kt_j: f64 = K_BOLTZMANN_J * temp_k;
    let factor: f64 = 2.0 * PI * M_ELECTRON * kt_j / (H_PLANCK * H_PLANCK);
    let saha_const: f64 = 2.0 * factor.powf(1.5) * (-ip_ev / (K_BOLTZMANN_EV * temp_k)).exp();

    // Convert ne from cm^-3 to m^-3
    let ne_m3: f64 = ne * 1.0e6;

    // alpha^2 / (1 - alpha) = S / ne
    // Let x = S / ne
    let x: f64 = saha_const / ne_m3;

    // Solve alpha^2 + x*alpha - x = 0
    // alpha = (-x + sqrt(x^2 + 4*x)) / 2
    let discriminant: f64 = x * x + 4.0 * x;
    if discriminant < 0.0 {
        return 0.0;
    }
    let alpha: f64 = (-x + discriminant.sqrt()) / 2.0;

    // Clamp to [0, 1]
    if alpha < 0.0 {
        0.0
    } else if alpha > 1.0 {
        1.0
    } else {
        alpha
    }
}

/// Calculate the wavelength separation needed to resolve two lines
/// using the Rayleigh criterion for a diffraction grating
pub fn rayleigh_resolution(wavelength_nm: f64, grating_lines: usize, order: usize) -> f64 {
    if grating_lines == 0 || order == 0 {
        return f64::INFINITY;
    }
    wavelength_nm / (grating_lines * order) as f64
}

/// Convert wavelength in nm to frequency in Hz
pub fn wavelength_to_frequency(wavelength_nm: f64) -> f64 {
    if wavelength_nm.abs() < 1.0e-15 {
        return 0.0;
    }
    C_LIGHT / (wavelength_nm * 1.0e-9)
}

/// Convert frequency in Hz to wavelength in nm
pub fn frequency_to_wavelength(frequency_hz: f64) -> f64 {
    if frequency_hz.abs() < 1.0e-15 {
        return 0.0;
    }
    (C_LIGHT / frequency_hz) * 1.0e9
}

/// Calculate blackbody spectral radiance (Planck function) at given wavelength and temperature
/// Returns W / (m^2 * sr * nm)
pub fn planck_radiance(wavelength_nm: f64, temp_k: f64) -> f64 {
    if wavelength_nm <= 0.0 || temp_k <= 0.0 {
        return 0.0;
    }
    let lambda_m: f64 = wavelength_nm * 1.0e-9;
    let c1: f64 = 2.0 * H_PLANCK * C_LIGHT * C_LIGHT;
    let c2: f64 = H_PLANCK * C_LIGHT / (K_BOLTZMANN_J * temp_k);
    let exponent: f64 = c2 / lambda_m;

    // Avoid overflow
    if exponent > 500.0 {
        return 0.0;
    }

    let denominator: f64 = lambda_m.powi(5) * (exponent.exp() - 1.0);
    if denominator.abs() < 1.0e-300 {
        return 0.0;
    }
    // Result in W/(m^2 * sr * m), convert last m to nm by * 1e-9
    (c1 / denominator) * 1.0e-9
}

/// Wien's displacement law: peak wavelength for blackbody
pub fn wien_peak_wavelength(temp_k: f64) -> f64 {
    if temp_k <= 0.0 {
        return f64::INFINITY;
    }
    // Wien's displacement constant: b = 2.897771955e-3 m*K
    let b: f64 = 2.897771955e-3;
    (b / temp_k) * 1.0e9 // Convert m to nm
}

/// Calculate the Doppler broadening (FWHM) of an emission line
/// Returns FWHM in nm
pub fn doppler_broadening(wavelength_nm: f64, temp_k: f64, atomic_mass_amu: f64) -> f64 {
    if atomic_mass_amu <= 0.0 || temp_k <= 0.0 || wavelength_nm <= 0.0 {
        return 0.0;
    }
    // FWHM = lambda * sqrt(8 * kT * ln(2) / (m * c^2))
    let mass_kg: f64 = atomic_mass_amu * 1.66053906660e-27;
    let factor: f64 = 8.0 * K_BOLTZMANN_J * temp_k * (2.0_f64.ln()) / (mass_kg * C_LIGHT * C_LIGHT);
    wavelength_nm * factor.sqrt()
}

/// Lorentzian line profile
pub fn lorentzian_profile(wavelength_nm: f64, center_nm: f64, fwhm_nm: f64) -> f64 {
    let gamma: f64 = fwhm_nm / 2.0;
    let delta: f64 = wavelength_nm - center_nm;
    gamma / (PI * (delta * delta + gamma * gamma))
}

/// Gaussian line profile
pub fn gaussian_profile(wavelength_nm: f64, center_nm: f64, fwhm_nm: f64) -> f64 {
    let sigma: f64 = fwhm_nm / (2.0 * (2.0_f64.ln()).sqrt());
    let delta: f64 = wavelength_nm - center_nm;
    let norm: f64 = 1.0 / (sigma * (2.0 * PI).sqrt());
    norm * (-0.5 * (delta / sigma).powi(2)).exp()
}

/// Voigt profile approximation (convolution of Gaussian and Lorentzian)
/// Using the pseudo-Voigt approximation
pub fn voigt_profile(wavelength_nm: f64, center_nm: f64, fwhm_g: f64, fwhm_l: f64) -> f64 {
    // Pseudo-Voigt: eta * L + (1-eta) * G
    // fwhm_v approx = (fwhm_g^5 + 2.69*fwhm_g^4*fwhm_l + 2.43*fwhm_g^3*fwhm_l^2
    //                  + 4.47*fwhm_g^2*fwhm_l^3 + 0.07*fwhm_g*fwhm_l^4 + fwhm_l^5)^(1/5)
    let fg5: f64 = fwhm_g.powi(5);
    let fl5: f64 = fwhm_l.powi(5);
    let fwhm_v: f64 = (fg5
        + 2.69 * fwhm_g.powi(4) * fwhm_l
        + 2.43 * fwhm_g.powi(3) * fwhm_l.powi(2)
        + 4.47 * fwhm_g.powi(2) * fwhm_l.powi(3)
        + 0.07 * fwhm_g * fwhm_l.powi(4)
        + fl5)
        .powf(0.2);

    // eta = 1.36603*(fl/fv) - 0.47719*(fl/fv)^2 + 0.11116*(fl/fv)^3
    let ratio: f64 = if fwhm_v > 1.0e-15 { fwhm_l / fwhm_v } else { 0.5 };
    let eta: f64 = 1.36603 * ratio - 0.47719 * ratio * ratio + 0.11116 * ratio * ratio * ratio;
    let eta: f64 = eta.max(0.0).min(1.0);

    let l: f64 = lorentzian_profile(wavelength_nm, center_nm, fwhm_v);
    let g: f64 = gaussian_profile(wavelength_nm, center_nm, fwhm_v);

    eta * l + (1.0 - eta) * g
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1.0e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if a.abs() < 1.0e-15 && b.abs() < 1.0e-15 {
            return true;
        }
        let max_abs: f64 = a.abs().max(b.abs());
        (a - b).abs() / max_abs < rel_tol
    }

    // ---- FlameType Tests ----

    #[test]
    fn test_flame_type_temperatures() {
        let ap: f64 = FlameType::AirPropane.temperature_k();
        let aa: f64 = FlameType::AirAcetylene.temperature_k();
        let n2o: f64 = FlameType::N2OAcetylene.temperature_k();

        assert!(ap < aa, "Air-propane should be cooler than air-acetylene");
        assert!(aa < n2o, "Air-acetylene should be cooler than N2O-acetylene");
        assert!(ap > 2000.0);
        assert!(aa > 2400.0);
        assert!(n2o > 3000.0);
    }

    #[test]
    fn test_flame_type_celsius() {
        let temp_c: f64 = FlameType::AirPropane.temperature_c();
        assert!(approx_eq(temp_c, 1924.85, 1.0));
    }

    // ---- FlameModel Tests ----

    #[test]
    fn test_flame_model_creation() {
        let model = FlameModel::new(FlameType::AirAcetylene);
        assert_eq!(model.flame_type, FlameType::AirAcetylene);
        assert!(approx_eq(model.temperature_k, 2573.0, 1.0));
        assert!(model.path_length_cm > 0.0);
    }

    #[test]
    fn test_flame_model_custom_temperature() {
        let model = FlameModel::with_temperature(FlameType::AirPropane, 2100.0);
        assert!(approx_eq(model.temperature_k, 2100.0, 0.1));
    }

    #[test]
    fn test_flame_model_boltzmann() {
        let model = FlameModel::new(FlameType::AirAcetylene);
        let na_line = EmissionLines::get(Element::Sodium);
        let frac: f64 = model.boltzmann_fraction(
            na_line.g_upper,
            na_line.g_lower,
            na_line.upper_level_ev,
        );
        // At 2573 K, Na 589nm (2.1 eV) should have very small excited fraction
        assert!(frac > 0.0);
        assert!(frac < 0.01); // Much less than 1%
    }

    #[test]
    fn test_flame_model_emission_intensity() {
        let model = FlameModel::new(FlameType::AirAcetylene);
        let intensity: f64 = model.emission_intensity(1.0e8, 6.16e7, 589.0);
        assert!(intensity > 0.0);
    }

    #[test]
    fn test_flame_model_ionization_fraction() {
        let model = FlameModel::new(FlameType::AirAcetylene);
        // Na has ionization potential 5.139 eV - should be low at 2573K
        let alpha_na: f64 = model.ionization_fraction(5.139, 1.0e12);
        assert!(alpha_na >= 0.0 && alpha_na <= 1.0);

        // Cs has lower IP (3.894 eV) - should be more ionized
        let alpha_cs: f64 = model.ionization_fraction(3.894, 1.0e12);
        assert!(alpha_cs > alpha_na, "Cs should be more ionized than Na");
    }

    #[test]
    fn test_flame_model_atom_density() {
        let model = FlameModel::new(FlameType::AirAcetylene);
        let density: f64 = model.atom_density(10.0, 0.05);
        assert!(density > 0.0);
        // 10 ppm * 0.05 efficiency * 1e10 = 5e9
        assert!(approx_eq(density, 5.0e9, 1.0e7));
    }

    // ---- EmissionLines Tests ----

    #[test]
    fn test_emission_line_sodium() {
        let na = EmissionLines::get(Element::Sodium);
        assert!(approx_eq(na.wavelength_nm, 589.0, 1.0));
        assert!(na.secondary_wavelength_nm.is_some());
        assert!(approx_eq(na.ionization_potential_ev, 5.139, 0.01));
        assert_eq!(na.color, "yellow");
    }

    #[test]
    fn test_emission_line_potassium() {
        let k = EmissionLines::get(Element::Potassium);
        assert!(approx_eq(k.wavelength_nm, 766.5, 1.0));
        assert!(approx_eq(k.ionization_potential_ev, 4.341, 0.01));
    }

    #[test]
    fn test_emission_line_lithium() {
        let li = EmissionLines::get(Element::Lithium);
        assert!(approx_eq(li.wavelength_nm, 670.8, 1.0));
        assert!(approx_eq(li.ionization_potential_ev, 5.392, 0.01));
    }

    #[test]
    fn test_emission_line_calcium() {
        let ca = EmissionLines::get(Element::Calcium);
        assert!(approx_eq(ca.wavelength_nm, 622.0, 1.0));
    }

    #[test]
    fn test_emission_line_barium() {
        let ba = EmissionLines::get(Element::Barium);
        assert!(approx_eq(ba.wavelength_nm, 553.6, 1.0));
        assert_eq!(ba.color, "green");
    }

    #[test]
    fn test_emission_line_strontium() {
        let sr = EmissionLines::get(Element::Strontium);
        assert!(approx_eq(sr.wavelength_nm, 460.7, 1.0));
    }

    #[test]
    fn test_emission_line_cesium() {
        let cs = EmissionLines::get(Element::Cesium);
        assert!(approx_eq(cs.wavelength_nm, 852.1, 1.0));
        assert!(approx_eq(cs.ionization_potential_ev, 3.894, 0.01));
    }

    #[test]
    fn test_emission_line_rubidium() {
        let rb = EmissionLines::get(Element::Rubidium);
        assert!(approx_eq(rb.wavelength_nm, 780.0, 1.0));
    }

    #[test]
    fn test_all_elements() {
        let elements = EmissionLines::all_elements();
        assert_eq!(elements.len(), 8);
    }

    #[test]
    fn test_photon_energy() {
        // Na 589nm -> ~2.1 eV
        let e: f64 = EmissionLines::photon_energy_ev(589.0);
        assert!(approx_eq(e, 2.104, 0.02));
    }

    #[test]
    fn test_wavelength_from_energy() {
        let wl: f64 = EmissionLines::wavelength_from_energy(2.104);
        assert!(approx_eq(wl, 589.0, 2.0));
    }

    #[test]
    fn test_photon_energy_roundtrip() {
        let wl_orig: f64 = 589.0;
        let energy: f64 = EmissionLines::photon_energy_ev(wl_orig);
        let wl_back: f64 = EmissionLines::wavelength_from_energy(energy);
        assert!(approx_eq(wl_orig, wl_back, 0.1));
    }

    // ---- InternalStandard Tests ----

    #[test]
    fn test_internal_standard_creation() {
        let is = InternalStandard::new(InternalStandardElement::Lithium, 15.0);
        assert_eq!(is.element, InternalStandardElement::Lithium);
        assert!(approx_eq(is.concentration_ppm, 15.0, 0.01));
    }

    #[test]
    fn test_internal_standard_normalize() {
        let is = InternalStandard::new(InternalStandardElement::Lithium, 15.0);
        let ratio: f64 = is.normalize(100.0, 50.0);
        assert!(approx_eq(ratio, 2.0, EPSILON));
    }

    #[test]
    fn test_internal_standard_normalize_zero() {
        let is = InternalStandard::new(InternalStandardElement::Lithium, 15.0);
        let ratio: f64 = is.normalize(100.0, 0.0);
        assert!(approx_eq(ratio, 0.0, EPSILON));
    }

    #[test]
    fn test_internal_standard_corrected_signal() {
        let mut is = InternalStandard::new(InternalStandardElement::Lithium, 15.0);
        is.set_reference(200.0);
        // If IS signal is 200, correction factor = 1.0
        let corrected: f64 = is.corrected_signal(100.0, 200.0);
        assert!(approx_eq(corrected, 100.0, EPSILON));

        // If IS signal drops to 100 (half), correction doubles analyte
        let corrected2: f64 = is.corrected_signal(100.0, 100.0);
        assert!(approx_eq(corrected2, 200.0, EPSILON));
    }

    #[test]
    fn test_internal_standard_correction_factor() {
        let mut is = InternalStandard::new(InternalStandardElement::Cesium, 10.0);
        is.set_reference(500.0);
        let factor: f64 = is.correction_factor(250.0);
        assert!(approx_eq(factor, 2.0, EPSILON));
    }

    #[test]
    fn test_internal_standard_emission_line() {
        let is_li = InternalStandard::new(InternalStandardElement::Lithium, 15.0);
        let line = is_li.emission_line();
        assert!(approx_eq(line.wavelength_nm, 670.8, 1.0));

        let is_cs = InternalStandard::new(InternalStandardElement::Cesium, 10.0);
        let line2 = is_cs.emission_line();
        assert!(approx_eq(line2.wavelength_nm, 852.1, 1.0));
    }

    // ---- CalibrationCurve Tests ----

    #[test]
    fn test_calibration_curve_linear() {
        let mut cal = CalibrationCurve::new();
        // Perfect linear data: signal = 2 * concentration + 5
        cal.add_point(0.0, 5.0);
        cal.add_point(10.0, 25.0);
        cal.add_point(20.0, 45.0);
        cal.add_point(30.0, 65.0);
        assert!(cal.fit_linear());
        assert!(approx_eq(cal.slope(), 2.0, 0.01));
        assert!(approx_eq(cal.intercept(), 5.0, 0.01));
        assert!(cal.r_squared() > 0.999);
    }

    #[test]
    fn test_calibration_curve_predict() {
        let mut cal = CalibrationCurve::new();
        cal.add_points(&[(0.0, 0.0), (10.0, 100.0), (20.0, 200.0)]);
        cal.fit_linear();
        let predicted: f64 = cal.predict_signal(15.0);
        assert!(approx_eq(predicted, 150.0, 1.0));
    }

    #[test]
    fn test_calibration_curve_concentration() {
        let mut cal = CalibrationCurve::new();
        cal.add_points(&[(0.0, 0.0), (10.0, 50.0), (20.0, 100.0)]);
        cal.fit_linear();
        let conc: f64 = cal.concentration_from_signal(75.0);
        assert!(approx_eq(conc, 15.0, 0.1));
    }

    #[test]
    fn test_calibration_curve_self_absorption() {
        let mut cal = CalibrationCurve::new();
        cal.add_points(&[(0.0, 0.0), (10.0, 100.0), (20.0, 200.0)]);
        cal.fit_linear();
        cal.set_self_absorption(0.01);
        // With self-absorption, signal at high concentration should be reduced
        let signal_linear: f64 = 10.0 * 200.0; // Without self-absorption
        let signal_sa: f64 = cal.predict_signal(200.0);
        assert!(signal_sa < signal_linear);
    }

    #[test]
    fn test_calibration_curve_not_fitted() {
        let cal = CalibrationCurve::new();
        assert!(!cal.is_fitted());
        assert!(approx_eq(cal.predict_signal(10.0), 0.0, EPSILON));
    }

    #[test]
    fn test_calibration_curve_too_few_points() {
        let mut cal = CalibrationCurve::new();
        cal.add_point(1.0, 10.0);
        assert!(!cal.fit_linear());
    }

    #[test]
    fn test_calibration_lod_loq() {
        let mut cal = CalibrationCurve::new();
        cal.add_points(&[(0.0, 0.0), (10.0, 100.0), (20.0, 200.0)]);
        cal.fit_linear();
        let lod: f64 = cal.limit_of_detection(1.0); // 3*1.0/10.0 = 0.3
        let loq: f64 = cal.limit_of_quantification(1.0); // 10*1.0/10.0 = 1.0
        assert!(approx_eq(lod, 0.3, 0.01));
        assert!(approx_eq(loq, 1.0, 0.01));
    }

    #[test]
    fn test_calibration_working_range() {
        let mut cal = CalibrationCurve::new();
        cal.add_points(&[(0.0, 0.0), (5.0, 50.0), (10.0, 100.0), (15.0, 150.0), (20.0, 200.0)]);
        cal.fit_linear();
        let range = cal.working_range(5.0);
        assert!(range.is_some());
        let (min_c, max_c) = range.unwrap();
        assert!(min_c <= 5.0);
        assert!(max_c >= 15.0);
    }

    #[test]
    fn test_calibration_num_points() {
        let mut cal = CalibrationCurve::new();
        cal.add_points(&[(1.0, 10.0), (2.0, 20.0), (3.0, 30.0)]);
        assert_eq!(cal.num_points(), 3);
    }

    #[test]
    fn test_calibration_self_absorption_roundtrip() {
        let mut cal = CalibrationCurve::new();
        cal.add_points(&[(0.0, 0.0), (10.0, 100.0), (20.0, 200.0)]);
        cal.fit_linear();
        cal.set_self_absorption(0.005);

        // Predict signal from a known concentration, then recover concentration
        let conc: f64 = 15.0;
        let signal: f64 = cal.predict_signal(conc);
        let recovered: f64 = cal.concentration_from_signal(signal);
        assert!(approx_eq(recovered, conc, 0.1));
    }

    // ---- InterferenceCorrection Tests ----

    #[test]
    fn test_interference_correction_default() {
        let ic = InterferenceCorrection::new();
        assert!(ic.ionization_buffer.is_none());
        assert!(approx_eq(ic.spectral_bandpass_nm, 2.0, 0.1));
    }

    #[test]
    fn test_interference_with_buffer() {
        let ic = InterferenceCorrection::new()
            .with_ionization_buffer(Element::Cesium, 1000.0);
        assert_eq!(ic.ionization_buffer, Some(Element::Cesium));
        assert!(approx_eq(ic.buffer_concentration_ppm, 1000.0, 0.1));
    }

    #[test]
    fn test_spectral_overlap_detection() {
        let ic = InterferenceCorrection::new().with_spectral_bandpass(2.0);
        // Na D1 and D2 lines are 0.6 nm apart - should overlap
        // But we are checking element-to-element overlap
        // Ba (553.6) and Na (589.0) are 35 nm apart - no overlap
        assert!(!ic.has_spectral_overlap(Element::Barium, Element::Sodium));

        // With very wide bandpass
        let ic_wide = InterferenceCorrection::new().with_spectral_bandpass(40.0);
        assert!(ic_wide.has_spectral_overlap(Element::Barium, Element::Sodium));
    }

    #[test]
    fn test_ionization_suppression_without_buffer() {
        let ic = InterferenceCorrection::new();
        let factor: f64 = ic.ionization_suppression_factor(5.139, 2573.0);
        assert!(approx_eq(factor, 1.0, EPSILON));
    }

    #[test]
    fn test_ionization_suppression_with_buffer() {
        let ic = InterferenceCorrection::new()
            .with_ionization_buffer(Element::Cesium, 1000.0);
        let factor: f64 = ic.ionization_suppression_factor(4.341, 2573.0);
        // Factor should be > 1 (more neutral atoms with buffer)
        assert!(factor >= 1.0);
    }

    #[test]
    fn test_chemical_correction_with_releasing_agent() {
        let ic = InterferenceCorrection::new().with_releasing_agent();
        let corrected: f64 = ic.chemical_correction_factor(100.0);
        assert!(approx_eq(corrected, 100.0, 0.1));
    }

    #[test]
    fn test_chemical_correction_without_releasing_agent() {
        let ic = InterferenceCorrection::new();
        let corrected: f64 = ic.chemical_correction_factor(100.0);
        assert!(approx_eq(corrected, 80.0, 0.1));
    }

    #[test]
    fn test_spectral_overlap_fraction() {
        let ic = InterferenceCorrection::new().with_spectral_bandpass(2.0);
        // Two elements far apart - zero overlap
        let frac: f64 = ic.spectral_overlap_fraction(
            Element::Sodium,
            Element::Potassium,
            10.0,
            1.0,
        );
        assert!(frac < 0.01, "Na and K should have negligible spectral overlap");
    }

    // ---- NebulizerModel Tests ----

    #[test]
    fn test_nebulizer_concentric() {
        let neb = NebulizerModel::new(NebulizerType::Concentric);
        assert!(approx_eq(neb.efficiency, 0.05, 0.01));
        assert!(approx_eq(neb.uptake_rate_ml_min, 5.0, 0.1));
    }

    #[test]
    fn test_nebulizer_crossflow() {
        let neb = NebulizerModel::new(NebulizerType::CrossFlow);
        assert!(approx_eq(neb.efficiency, 0.03, 0.01));
    }

    #[test]
    fn test_nebulizer_ultrasonic() {
        let neb = NebulizerModel::new(NebulizerType::Ultrasonic);
        assert!(neb.efficiency > 0.10, "Ultrasonic should be more efficient");
    }

    #[test]
    fn test_nebulizer_mass_flow() {
        let neb = NebulizerModel::new(NebulizerType::Concentric);
        let flow: f64 = neb.analyte_mass_flow(10.0); // 10 ppm
        // 10 ug/mL * 5 mL/min * 0.05 = 2.5 ug/min
        assert!(approx_eq(flow, 2.5, 0.01));
    }

    #[test]
    fn test_nebulizer_transport_rate() {
        let neb = NebulizerModel::new(NebulizerType::Concentric);
        let rate: f64 = neb.effective_transport_rate();
        assert!(approx_eq(rate, 0.25, 0.01)); // 5 * 0.05
    }

    #[test]
    fn test_nebulizer_drain_correction() {
        let neb = NebulizerModel::new(NebulizerType::Concentric);
        let factor: f64 = neb.drain_correction_factor(0.90);
        // 0.95 / 0.90 = 1.0556
        assert!(approx_eq(factor, 1.0556, 0.001));
    }

    #[test]
    fn test_nebulizer_sauter_mean_diameter() {
        let neb = NebulizerModel::new(NebulizerType::Concentric);
        let d: f64 = neb.sauter_mean_diameter(100.0, 0.072, 0.001, 1000.0);
        assert!(d > 0.0);
    }

    // ---- SignalProcessor Tests ----

    #[test]
    fn test_signal_processor_background_subtract() {
        let mut sp = SignalProcessor::new(10);
        sp.set_background(5.0);
        let result: f64 = sp.background_subtract(105.0);
        assert!(approx_eq(result, 100.0, EPSILON));
    }

    #[test]
    fn test_signal_processor_drift_correct() {
        let mut sp = SignalProcessor::new(10);
        sp.set_drift_reference(100.0);
        // If reference has drifted to 110, actual signal should be corrected down
        let corrected: f64 = sp.drift_correct(50.0, 110.0);
        assert!(approx_eq(corrected, 50.0 * 100.0 / 110.0, EPSILON));
    }

    #[test]
    fn test_signal_processor_averaging() {
        let mut sp = SignalProcessor::new(5);
        sp.add_measurement(10.0);
        sp.add_measurement(12.0);
        sp.add_measurement(11.0);
        let avg: f64 = sp.average();
        assert!(approx_eq(avg, 11.0, EPSILON));
    }

    #[test]
    fn test_signal_processor_std_deviation() {
        let mut sp = SignalProcessor::new(10);
        sp.add_measurement(10.0);
        sp.add_measurement(12.0);
        sp.add_measurement(14.0);
        let std: f64 = sp.std_deviation();
        assert!(std > 0.0);
        assert!(approx_eq(std, 2.0, 0.01));
    }

    #[test]
    fn test_signal_processor_cv() {
        let mut sp = SignalProcessor::new(10);
        sp.add_measurement(100.0);
        sp.add_measurement(102.0);
        sp.add_measurement(98.0);
        let cv: f64 = sp.coefficient_of_variation();
        assert!(cv > 0.0 && cv < 5.0);
    }

    #[test]
    fn test_signal_processor_stability() {
        let mut sp = SignalProcessor::new(10);
        for _ in 0..5 {
            sp.add_measurement(100.0);
        }
        assert!(sp.is_stable(1.0));
    }

    #[test]
    fn test_signal_processor_snr() {
        let sp = SignalProcessor::new(10);
        let snr: f64 = sp.signal_to_noise(100.0, 1.0);
        assert!(approx_eq(snr, 100.0, EPSILON));
    }

    #[test]
    fn test_signal_processor_median() {
        let mut sp = SignalProcessor::new(10);
        sp.add_measurement(10.0);
        sp.add_measurement(100.0); // outlier
        sp.add_measurement(11.0);
        sp.add_measurement(12.0);
        sp.add_measurement(9.0);
        let median: f64 = sp.median_filtered();
        assert!(approx_eq(median, 11.0, EPSILON));
    }

    #[test]
    fn test_signal_processor_clear() {
        let mut sp = SignalProcessor::new(10);
        sp.add_measurement(10.0);
        sp.add_measurement(20.0);
        assert_eq!(sp.count(), 2);
        sp.clear();
        assert_eq!(sp.count(), 0);
    }

    #[test]
    fn test_signal_processor_moving_average() {
        let data: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = SignalProcessor::moving_average(&data, 3);
        assert_eq!(result.len(), 5);
        // Middle value should be average of 3 neighbors
        assert!(approx_eq(result[2], 3.0, EPSILON));
    }

    #[test]
    fn test_signal_processor_compute_snr() {
        let signal: Vec<f64> = vec![100.0, 100.0, 100.0];
        let noise: Vec<f64> = vec![1.0, -1.0, 1.0, -1.0];
        let snr: f64 = SignalProcessor::compute_snr(&signal, &noise);
        assert!(snr > 50.0);
    }

    #[test]
    fn test_signal_processor_max_history() {
        let mut sp = SignalProcessor::new(3);
        sp.add_measurement(1.0);
        sp.add_measurement(2.0);
        sp.add_measurement(3.0);
        sp.add_measurement(4.0);
        assert_eq!(sp.count(), 3);
        // Should have 2, 3, 4
        assert!(approx_eq(sp.average(), 3.0, EPSILON));
    }

    // ---- ClinicalApplication Tests ----

    #[test]
    fn test_clinical_sodium_range() {
        let range = ClinicalApplication::sodium_range();
        assert!(approx_eq(range.low, 135.0, 0.1));
        assert!(approx_eq(range.high, 145.0, 0.1));
    }

    #[test]
    fn test_clinical_potassium_range() {
        let range = ClinicalApplication::potassium_range();
        assert!(approx_eq(range.low, 3.5, 0.1));
        assert!(approx_eq(range.high, 5.0, 0.1));
    }

    #[test]
    fn test_clinical_lithium_range() {
        let range = ClinicalApplication::lithium_therapeutic_range();
        assert!(approx_eq(range.low, 0.6, 0.1));
        assert!(approx_eq(range.high, 1.2, 0.1));
    }

    #[test]
    fn test_clinical_status_normal() {
        let range = ClinicalApplication::sodium_range();
        let status = ClinicalApplication::evaluate_status(140.0, &range);
        assert_eq!(status, ClinicalStatus::Normal);
    }

    #[test]
    fn test_clinical_status_low() {
        let range = ClinicalApplication::sodium_range();
        let status = ClinicalApplication::evaluate_status(130.0, &range);
        assert_eq!(status, ClinicalStatus::Low);
    }

    #[test]
    fn test_clinical_status_high() {
        let range = ClinicalApplication::sodium_range();
        let status = ClinicalApplication::evaluate_status(150.0, &range);
        assert_eq!(status, ClinicalStatus::High);
    }

    #[test]
    fn test_clinical_status_critical_low() {
        let range = ClinicalApplication::sodium_range();
        let status = ClinicalApplication::evaluate_status(115.0, &range);
        assert_eq!(status, ClinicalStatus::CriticalLow);
    }

    #[test]
    fn test_clinical_status_critical_high() {
        let range = ClinicalApplication::potassium_range();
        let status = ClinicalApplication::evaluate_status(7.0, &range);
        assert_eq!(status, ClinicalStatus::CriticalHigh);
    }

    #[test]
    fn test_clinical_conversion_to_serum() {
        let clin = ClinicalApplication::new(); // 1:200 dilution
        // If measured 0.69 ppm Na in diluted sample:
        // serum = 0.69 * 200 = 138 mg/L = 138/22.99 = 6.0 mmol/L... wait
        // Actually, 138 mg/L / 22.99 g/mol = 6.0 mmol/L... that's wrong
        // Let's recalculate: typical serum Na is 140 mmol/L = 140 * 22.99 = 3218 mg/L
        // Diluted 1:200 = 16.1 ppm
        let measured_ppm: f64 = 16.1;
        let serum: f64 = clin.to_serum_mmol_per_l(measured_ppm, 22.99);
        assert!(approx_eq(serum, 140.0, 1.0));
    }

    #[test]
    fn test_clinical_conversion_from_serum() {
        let clin = ClinicalApplication::new();
        let expected_ppm: f64 = clin.from_serum_mmol_per_l(140.0, 22.99);
        // 140 * 22.99 / 200 = 16.09 ppm
        assert!(approx_eq(expected_ppm, 16.09, 0.1));
    }

    #[test]
    fn test_clinical_needs_dilution() {
        let clin = ClinicalApplication::new();
        assert!(clin.needs_dilution(25.0, 20.0));
        assert!(!clin.needs_dilution(15.0, 20.0));
    }

    #[test]
    fn test_clinical_additional_dilution() {
        let clin = ClinicalApplication::new();
        let factor: f64 = clin.additional_dilution_factor(40.0, 20.0);
        assert!(approx_eq(factor, 2.0, EPSILON));
    }

    #[test]
    fn test_clinical_total_dilution() {
        let clin = ClinicalApplication::new();
        let total: f64 = clin.total_dilution(2.0);
        assert!(approx_eq(total, 400.0, EPSILON));
    }

    // ---- Helper Function Tests ----

    #[test]
    fn test_boltzmann_fraction_basic() {
        // At very high T, fraction should approach g_e/(g_e+g_g)
        let frac: f64 = boltzmann_fraction(2.0, 2.0, 0.001, 100000.0);
        assert!(approx_eq(frac, 0.5, 0.01));
    }

    #[test]
    fn test_boltzmann_fraction_low_temp() {
        // At low temperature with high energy gap, fraction should be near zero
        let frac: f64 = boltzmann_fraction(2.0, 2.0, 5.0, 300.0);
        assert!(frac < 1.0e-30);
    }

    #[test]
    fn test_boltzmann_fraction_zero_temp() {
        let frac: f64 = boltzmann_fraction(2.0, 2.0, 2.0, 0.0);
        assert!(approx_eq(frac, 0.0, EPSILON));
    }

    #[test]
    fn test_emission_intensity_basic() {
        let i: f64 = emission_intensity(1.0e10, 1.0e8, 3.37e-19);
        assert!(i > 0.0);
        assert!(approx_eq(i, 1.0e10 * 1.0e8 * 3.37e-19, 1.0e-5));
    }

    #[test]
    fn test_emission_intensity_zero() {
        let i: f64 = emission_intensity(0.0, 1.0e8, 3.37e-19);
        assert!(approx_eq(i, 0.0, EPSILON));
    }

    #[test]
    fn test_ionization_fraction_basic() {
        // High IP at moderate temp should give low ionization
        let alpha: f64 = ionization_fraction(2573.0, 5.139, 1.0e12);
        assert!(alpha >= 0.0 && alpha <= 1.0);
    }

    #[test]
    fn test_ionization_fraction_low_ip() {
        // Low IP should give higher ionization
        let alpha_low: f64 = ionization_fraction(2573.0, 3.894, 1.0e12);
        let alpha_high: f64 = ionization_fraction(2573.0, 5.392, 1.0e12);
        assert!(alpha_low > alpha_high);
    }

    #[test]
    fn test_ionization_fraction_bounds() {
        let alpha: f64 = ionization_fraction(5000.0, 3.0, 1.0e10);
        assert!(alpha >= 0.0 && alpha <= 1.0);
    }

    #[test]
    fn test_rayleigh_resolution() {
        let res: f64 = rayleigh_resolution(589.0, 1000, 1);
        assert!(approx_eq(res, 0.589, 0.001));
    }

    #[test]
    fn test_wavelength_to_frequency() {
        let freq: f64 = wavelength_to_frequency(589.0);
        // c/lambda = 3e8 / 589e-9 ~= 5.09e14 Hz
        assert!(relative_eq(freq, 5.09e14, 0.01));
    }

    #[test]
    fn test_frequency_to_wavelength() {
        let wl: f64 = frequency_to_wavelength(5.09e14);
        assert!(approx_eq(wl, 589.0, 1.0));
    }

    #[test]
    fn test_wavelength_frequency_roundtrip() {
        let wl_orig: f64 = 670.8;
        let freq: f64 = wavelength_to_frequency(wl_orig);
        let wl_back: f64 = frequency_to_wavelength(freq);
        assert!(approx_eq(wl_orig, wl_back, 0.01));
    }

    #[test]
    fn test_planck_radiance_positive() {
        let rad: f64 = planck_radiance(589.0, 2573.0);
        assert!(rad > 0.0);
    }

    #[test]
    fn test_planck_radiance_peak() {
        // Planck function should peak near Wien's peak
        let temp: f64 = 2573.0;
        let peak_wl: f64 = wien_peak_wavelength(temp);
        let rad_peak: f64 = planck_radiance(peak_wl, temp);
        let rad_off: f64 = planck_radiance(peak_wl * 2.0, temp);
        assert!(rad_peak > rad_off, "Peak should have higher radiance");
    }

    #[test]
    fn test_planck_radiance_zero() {
        let rad: f64 = planck_radiance(0.0, 2573.0);
        assert!(approx_eq(rad, 0.0, EPSILON));
    }

    #[test]
    fn test_wien_peak_wavelength() {
        // At ~2573 K, peak should be around 1126 nm (near-IR)
        let peak: f64 = wien_peak_wavelength(2573.0);
        assert!(peak > 1000.0 && peak < 1300.0);
    }

    #[test]
    fn test_doppler_broadening() {
        let fwhm: f64 = doppler_broadening(589.0, 2573.0, 22.99);
        // Should be small - order of pm to pm's
        assert!(fwhm > 0.0);
        assert!(fwhm < 0.1); // Less than 0.1 nm
    }

    #[test]
    fn test_doppler_broadening_heavier_atom() {
        let fwhm_na: f64 = doppler_broadening(589.0, 2573.0, 22.99);
        let fwhm_cs: f64 = doppler_broadening(589.0, 2573.0, 132.9);
        assert!(fwhm_na > fwhm_cs, "Lighter atom should have broader Doppler");
    }

    #[test]
    fn test_lorentzian_profile_peak() {
        let peak: f64 = lorentzian_profile(589.0, 589.0, 0.01);
        let off: f64 = lorentzian_profile(590.0, 589.0, 0.01);
        assert!(peak > off);
    }

    #[test]
    fn test_gaussian_profile_peak() {
        let peak: f64 = gaussian_profile(589.0, 589.0, 0.01);
        let off: f64 = gaussian_profile(590.0, 589.0, 0.01);
        assert!(peak > off);
    }

    #[test]
    fn test_gaussian_profile_symmetric() {
        let left: f64 = gaussian_profile(588.0, 589.0, 0.5);
        let right: f64 = gaussian_profile(590.0, 589.0, 0.5);
        assert!(approx_eq(left, right, 1.0e-10));
    }

    #[test]
    fn test_voigt_profile_peak() {
        let peak: f64 = voigt_profile(589.0, 589.0, 0.01, 0.01);
        let off: f64 = voigt_profile(590.0, 589.0, 0.01, 0.01);
        assert!(peak > off);
    }

    #[test]
    fn test_voigt_profile_limits() {
        // When Lorentzian FWHM >> Gaussian, should approach Lorentzian
        let v1: f64 = voigt_profile(589.0, 589.0, 0.001, 1.0);
        let l1: f64 = lorentzian_profile(589.0, 589.0, 1.0);
        // Not exact due to pseudo-Voigt, but should be in same ballpark
        assert!(relative_eq(v1, l1, 0.5));
    }

    // ---- FlamePhotometerSession Tests ----

    #[test]
    fn test_session_creation() {
        let session = FlamePhotometerSession::new(FlameType::AirPropane);
        assert_eq!(session.state, SessionState::Idle);
    }

    #[test]
    fn test_session_clinical_creation() {
        let session = FlamePhotometerSession::clinical(FlameType::AirPropane);
        assert!(session.clinical.is_some());
        assert!(session.internal_standard.is_some());
    }

    #[test]
    fn test_session_calibration_workflow() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        session.begin_calibration();
        assert_eq!(session.state, SessionState::Calibrating);

        session.add_calibration(
            Element::Sodium,
            &[(0.0, 0.0), (5.0, 50.0), (10.0, 100.0), (15.0, 150.0)],
        );

        let ok: bool = session.finish_calibration();
        assert!(ok);
        assert_eq!(session.state, SessionState::Ready);
    }

    #[test]
    fn test_session_measurement() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        session.begin_calibration();
        session.add_calibration(
            Element::Sodium,
            &[(0.0, 0.0), (5.0, 50.0), (10.0, 100.0), (15.0, 150.0)],
        );
        session.finish_calibration();

        let result = session.measure(
            Element::Sodium,
            &[75.0, 74.0, 76.0],
            None,
        );
        assert!(result.is_some());
        let r = result.unwrap();
        assert!(approx_eq(r.concentration_ppm, 7.5, 0.5));
        assert_eq!(r.replicates, 3);
    }

    #[test]
    fn test_session_qc_check() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        session.begin_calibration();
        session.add_calibration(
            Element::Sodium,
            &[(0.0, 0.0), (5.0, 50.0), (10.0, 100.0)],
        );
        session.finish_calibration();

        let qc = session.qc_check(Element::Sodium, 7.5, &[75.0, 75.0], 5.0);
        assert!(qc.is_some());
        let q = qc.unwrap();
        assert!(q.passed);
    }

    #[test]
    fn test_session_complete() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        session.complete();
        assert_eq!(session.state, SessionState::Complete);
    }

    #[test]
    fn test_session_results_collection() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        session.begin_calibration();
        session.add_calibration(
            Element::Sodium,
            &[(0.0, 0.0), (10.0, 100.0), (20.0, 200.0)],
        );
        session.finish_calibration();

        session.measure(Element::Sodium, &[50.0], None);
        session.measure(Element::Sodium, &[100.0], None);

        assert_eq!(session.results().len(), 2);
    }

    #[test]
    fn test_session_measure_not_ready() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        let result = session.measure(Element::Sodium, &[50.0], None);
        assert!(result.is_none());
    }

    #[test]
    fn test_session_get_calibration() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        session.begin_calibration();
        session.add_calibration(
            Element::Sodium,
            &[(0.0, 0.0), (10.0, 100.0)],
        );
        session.finish_calibration();

        assert!(session.get_calibration(Element::Sodium).is_some());
        assert!(session.get_calibration(Element::Potassium).is_none());
    }

    #[test]
    fn test_session_auto_dilution_check() {
        let mut session = FlamePhotometerSession::new(FlameType::AirPropane);
        session.begin_calibration();
        session.add_calibration(
            Element::Sodium,
            &[(0.0, 0.0), (5.0, 50.0), (10.0, 100.0), (15.0, 150.0), (20.0, 200.0)],
        );
        session.finish_calibration();

        // Very high signal should suggest auto-dilution
        let needs: bool = session.needs_auto_dilution(Element::Sodium, 500.0);
        assert!(needs);
    }

    #[test]
    fn test_session_set_nebulizer() {
        let mut session = FlamePhotometerSession::new(FlameType::AirAcetylene);
        session.set_nebulizer(NebulizerModel::new(NebulizerType::Ultrasonic));
        assert_eq!(session.nebulizer.nebulizer_type, NebulizerType::Ultrasonic);
    }

    #[test]
    fn test_session_set_internal_standard() {
        let mut session = FlamePhotometerSession::new(FlameType::AirAcetylene);
        session.set_internal_standard(InternalStandard::new(
            InternalStandardElement::Cesium,
            20.0,
        ));
        assert!(session.internal_standard.is_some());
    }
}

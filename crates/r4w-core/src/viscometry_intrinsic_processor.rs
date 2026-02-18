//! # Viscometry and Intrinsic Viscosity Processor
//!
//! This module implements viscometry processing for polymer characterization,
//! providing tools for capillary viscometry analysis, viscosity calculations,
//! and molecular weight estimation.
//!
//! ## Overview
//!
//! Viscometry is a fundamental technique in polymer science for characterizing
//! macromolecular solutions. The intrinsic viscosity `[η]` is a measure of a
//! polymer's contribution to the viscosity of a solution, extrapolated to
//! infinite dilution where polymer-polymer interactions vanish.
//!
//! ## Viscosity Types
//!
//! - **Relative viscosity**: η_rel = η_solution / η_solvent ≈ t_solution / t_solvent
//! - **Specific viscosity**: η_sp = η_rel - 1
//! - **Reduced viscosity**: η_red = η_sp / c
//! - **Inherent viscosity**: η_inh = ln(η_rel) / c
//! - **Intrinsic viscosity**: [η] = lim(c→0) η_red = lim(c→0) η_inh
//!
//! ## Extrapolation Methods
//!
//! - **Huggins equation**: η_sp/c = [η] + k_H·[η]²·c
//! - **Kraemer equation**: ln(η_rel)/c = [η] + k_K·[η]²·c
//! - Both should yield the same [η] intercept; k_H + k_K ≈ 0.5
//!
//! ## Mark-Houwink-Sakurada Equation
//!
//! Relates intrinsic viscosity to molecular weight:
//!
//! ```text
//! [η] = K · M^a
//! ```
//!
//! where K and a are polymer-solvent-temperature-specific constants.
//!
//! ## Temperature Models
//!
//! - **Arrhenius**: η = η_ref · exp(E_a/R · (1/T - 1/T_ref))
//! - **VFT (Vogel-Fulcher-Tammann)**: η = A · exp(B / (T - T_0))
//! - **WLF (Williams-Landel-Ferry)**: log(η/η_ref) = -C1·(T-Tref) / (C2 + T - Tref)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::viscometry_intrinsic_processor::*;
//!
//! // Measure relative and specific viscosity
//! let eta_rel = relative_viscosity(120.5, 100.0);
//! let eta_sp = specific_viscosity(eta_rel);
//!
//! // Compute reduced and inherent viscosities
//! let c = 0.005; // g/mL
//! let eta_red = reduced_viscosity(eta_sp, c);
//! let eta_inh = inherent_viscosity(eta_rel, c);
//!
//! // Determine intrinsic viscosity from multiple concentrations
//! let concentrations = vec![0.002, 0.004, 0.006, 0.008, 0.01];
//! let eta_rels = vec![1.08, 1.17, 1.27, 1.38, 1.50];
//! let determiner = IntrinsicViscosityDeterminer::new(&concentrations, &eta_rels);
//! let result = determiner.huggins_extrapolation();
//! println!("Intrinsic viscosity [η] = {:.2} mL/g", result.intrinsic_viscosity);
//! ```

use std::f64::consts::PI;

// ============================================================================
// Physical Constants
// ============================================================================

/// Gas constant in J/(mol·K)
pub const GAS_CONSTANT_R: f64 = 8.314;

/// Water viscosity at 20°C in mPa·s (centipoise)
pub const WATER_VISCOSITY_20C: f64 = 1.002;

/// Water viscosity at 25°C in mPa·s (centipoise)
pub const WATER_VISCOSITY_25C: f64 = 0.8901;

/// Water density at 20°C in g/mL
pub const WATER_DENSITY_20C: f64 = 0.99821;

/// Water density at 25°C in g/mL
pub const WATER_DENSITY_25C: f64 = 0.99705;

// ============================================================================
// Capillary Viscometer Types
// ============================================================================

/// Type of capillary viscometer
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CapillaryType {
    /// Ubbelohde (suspended-level) viscometer — flow time independent of sample volume
    Ubbelohde,
    /// Ostwald (U-tube) viscometer — requires precise volume control
    Ostwald,
}

/// Calibration parameters for a capillary viscometer
#[derive(Debug, Clone)]
pub struct CapillaryCalibration {
    /// Viscometer constant K (mm²/s²) relating kinematic viscosity to flow time
    pub constant_k: f64,
    /// Hagenbach correction coefficient (s) for kinetic energy losses
    pub hagenbach_coeff: f64,
    /// Capillary radius in meters
    pub radius: f64,
    /// Capillary length in meters
    pub length: f64,
    /// Mean hydrostatic head in meters
    pub head_height: f64,
}

impl CapillaryCalibration {
    /// Create a new calibration with typical Ubbelohde parameters
    pub fn ubbelohde_typical() -> Self {
        Self {
            constant_k: 0.01,
            hagenbach_coeff: 1.12,
            radius: 0.0003, // 0.3 mm
            length: 0.10,   // 100 mm
            head_height: 0.10,
        }
    }

    /// Create a new calibration with typical Ostwald parameters
    pub fn ostwald_typical() -> Self {
        Self {
            constant_k: 0.015,
            hagenbach_coeff: 1.5,
            radius: 0.0004, // 0.4 mm
            length: 0.12,   // 120 mm
            head_height: 0.12,
        }
    }
}

// ============================================================================
// Solvent Specification
// ============================================================================

/// Common solvents for viscometry
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Solvent {
    /// Water
    Water,
    /// Tetrahydrofuran (THF)
    THF,
    /// Dimethylformamide (DMF)
    DMF,
    /// Toluene
    Toluene,
    /// Chloroform
    Chloroform,
    /// Custom solvent with specified viscosity (mPa·s) and density (g/mL)
    Custom,
}

/// Solvent properties at a given temperature
#[derive(Debug, Clone)]
pub struct SolventProperties {
    pub solvent: Solvent,
    /// Dynamic viscosity in mPa·s (centipoise)
    pub viscosity: f64,
    /// Density in g/mL
    pub density: f64,
    /// Temperature in Kelvin
    pub temperature: f64,
}

impl SolventProperties {
    /// Water at 25°C
    pub fn water_25c() -> Self {
        Self {
            solvent: Solvent::Water,
            viscosity: WATER_VISCOSITY_25C,
            density: WATER_DENSITY_25C,
            temperature: 298.15,
        }
    }

    /// Water at 20°C
    pub fn water_20c() -> Self {
        Self {
            solvent: Solvent::Water,
            viscosity: WATER_VISCOSITY_20C,
            density: WATER_DENSITY_20C,
            temperature: 293.15,
        }
    }

    /// THF at 25°C
    pub fn thf_25c() -> Self {
        Self {
            solvent: Solvent::THF,
            viscosity: 0.456,
            density: 0.889,
            temperature: 298.15,
        }
    }

    /// Custom solvent
    pub fn custom(viscosity: f64, density: f64, temperature: f64) -> Self {
        Self {
            solvent: Solvent::Custom,
            viscosity,
            density,
            temperature,
        }
    }
}

// ============================================================================
// Mark-Houwink-Sakurada Parameters
// ============================================================================

/// Mark-Houwink-Sakurada (MHS) parameters for a polymer-solvent system
///
/// The relationship is: [η] = K · M^a
///
/// where:
/// - [η] is the intrinsic viscosity in mL/g (or dL/g depending on convention)
/// - K is a constant that depends on polymer-solvent-temperature
/// - M is the viscosity-average molecular weight
/// - a is the exponent (0.5 for theta solvent, 0.5-0.8 for good solvents)
#[derive(Debug, Clone)]
pub struct MarkHouwinkParams {
    /// K constant in mL/g
    pub k: f64,
    /// Exponent a (dimensionless)
    pub a: f64,
    /// Description of the polymer-solvent system
    pub description: &'static str,
}

impl MarkHouwinkParams {
    /// PEO (polyethylene oxide) in water at 25°C
    pub fn peo_water_25c() -> Self {
        Self {
            k: 0.0125,
            a: 0.78,
            description: "PEO in water at 25°C",
        }
    }

    /// Polystyrene in THF at 25°C
    pub fn ps_thf_25c() -> Self {
        Self {
            k: 0.0141,
            a: 0.70,
            description: "Polystyrene in THF at 25°C",
        }
    }

    /// Polystyrene in toluene at 25°C
    pub fn ps_toluene_25c() -> Self {
        Self {
            k: 0.0096,
            a: 0.73,
            description: "Polystyrene in toluene at 25°C",
        }
    }

    /// PVA (polyvinyl alcohol) in water at 25°C
    pub fn pva_water_25c() -> Self {
        Self {
            k: 0.030,
            a: 0.50,
            description: "PVA in water at 25°C (theta conditions)",
        }
    }

    /// Custom parameters
    pub fn custom(k: f64, a: f64, description: &'static str) -> Self {
        Self { k, a, description }
    }
}

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for the viscometry processor
#[derive(Debug, Clone)]
pub struct ViscometryConfig {
    /// Type of capillary viscometer
    pub capillary_type: CapillaryType,
    /// Calibration parameters
    pub calibration: CapillaryCalibration,
    /// Solvent properties
    pub solvent: SolventProperties,
    /// Mark-Houwink parameters (optional)
    pub mark_houwink: Option<MarkHouwinkParams>,
    /// Whether to apply Hagenbach kinetic energy correction
    pub apply_hagenbach: bool,
}

impl ViscometryConfig {
    /// Standard configuration for Ubbelohde viscometer with water at 25°C
    pub fn ubbelohde_water_25c() -> Self {
        Self {
            capillary_type: CapillaryType::Ubbelohde,
            calibration: CapillaryCalibration::ubbelohde_typical(),
            solvent: SolventProperties::water_25c(),
            mark_houwink: None,
            apply_hagenbach: true,
        }
    }

    /// Standard configuration for Ostwald viscometer with water at 25°C
    pub fn ostwald_water_25c() -> Self {
        Self {
            capillary_type: CapillaryType::Ostwald,
            calibration: CapillaryCalibration::ostwald_typical(),
            solvent: SolventProperties::water_25c(),
            mark_houwink: None,
            apply_hagenbach: true,
        }
    }

    /// Set Mark-Houwink parameters
    pub fn with_mark_houwink(mut self, params: MarkHouwinkParams) -> Self {
        self.mark_houwink = Some(params);
        self
    }
}

// ============================================================================
// Basic Viscosity Functions
// ============================================================================

/// Calculate relative viscosity from flow times
///
/// η_rel = t_solution / t_solvent
///
/// For a capillary viscometer under identical conditions, the ratio of
/// flow times equals the ratio of kinematic viscosities.
///
/// # Arguments
/// * `t_solution` - Flow time of solution in seconds
/// * `t_solvent` - Flow time of solvent in seconds
///
/// # Returns
/// Relative viscosity (dimensionless, > 1 for polymer solutions)
pub fn relative_viscosity(t_solution: f64, t_solvent: f64) -> f64 {
    assert!(t_solvent > 0.0, "Solvent flow time must be positive");
    assert!(t_solution > 0.0, "Solution flow time must be positive");
    t_solution / t_solvent
}

/// Calculate specific viscosity from relative viscosity
///
/// η_sp = η_rel - 1
///
/// Represents the fractional increase in viscosity due to the solute.
///
/// # Arguments
/// * `eta_rel` - Relative viscosity (dimensionless)
///
/// # Returns
/// Specific viscosity (dimensionless)
pub fn specific_viscosity(eta_rel: f64) -> f64 {
    eta_rel - 1.0
}

/// Calculate reduced viscosity
///
/// η_red = η_sp / c
///
/// Also known as viscosity number. Has units of inverse concentration (mL/g).
///
/// # Arguments
/// * `eta_sp` - Specific viscosity (dimensionless)
/// * `concentration` - Polymer concentration in g/mL
///
/// # Returns
/// Reduced viscosity in mL/g
pub fn reduced_viscosity(eta_sp: f64, concentration: f64) -> f64 {
    assert!(concentration > 0.0, "Concentration must be positive");
    eta_sp / concentration
}

/// Calculate inherent viscosity
///
/// η_inh = ln(η_rel) / c
///
/// Also known as logarithmic viscosity number. Has units of inverse concentration (mL/g).
///
/// # Arguments
/// * `eta_rel` - Relative viscosity (dimensionless)
/// * `concentration` - Polymer concentration in g/mL
///
/// # Returns
/// Inherent viscosity in mL/g
pub fn inherent_viscosity(eta_rel: f64, concentration: f64) -> f64 {
    assert!(concentration > 0.0, "Concentration must be positive");
    assert!(eta_rel > 0.0, "Relative viscosity must be positive");
    eta_rel.ln() / concentration
}

/// Convert kinematic viscosity to dynamic viscosity
///
/// η = ν · ρ
///
/// # Arguments
/// * `kinematic_viscosity` - Kinematic viscosity in mm²/s (centistokes)
/// * `density` - Density in g/mL (g/cm³)
///
/// # Returns
/// Dynamic viscosity in mPa·s (centipoise)
pub fn kinematic_to_dynamic(kinematic_viscosity: f64, density: f64) -> f64 {
    kinematic_viscosity * density
}

/// Convert dynamic viscosity to kinematic viscosity
///
/// ν = η / ρ
///
/// # Arguments
/// * `dynamic_viscosity` - Dynamic viscosity in mPa·s (centipoise)
/// * `density` - Density in g/mL (g/cm³)
///
/// # Returns
/// Kinematic viscosity in mm²/s (centistokes)
pub fn dynamic_to_kinematic(dynamic_viscosity: f64, density: f64) -> f64 {
    assert!(density > 0.0, "Density must be positive");
    dynamic_viscosity / density
}

// ============================================================================
// Hagenbach Correction
// ============================================================================

/// Apply Hagenbach kinetic energy correction to flow time
///
/// The corrected time accounts for the kinetic energy of the fluid
/// entering and leaving the capillary:
///
/// t_corrected = t_measured - (m · V) / (8π · L · ν)
///
/// Simplified using the calibration's Hagenbach coefficient:
///
/// t_corrected = t_measured - hagenbach_coeff / t_measured
///
/// # Arguments
/// * `flow_time` - Measured flow time in seconds
/// * `calibration` - Capillary calibration parameters
///
/// # Returns
/// Corrected flow time in seconds
pub fn hagenbach_correction(flow_time: f64, calibration: &CapillaryCalibration) -> f64 {
    assert!(flow_time > 0.0, "Flow time must be positive");
    let correction = calibration.hagenbach_coeff / flow_time;
    let corrected = flow_time - correction;
    // Ensure corrected time is positive
    if corrected > 0.0 {
        corrected
    } else {
        flow_time
    }
}

// ============================================================================
// Poiseuille Flow
// ============================================================================

/// Calculate volumetric flow rate using Hagen-Poiseuille equation
///
/// Q = (π · r⁴ · ΔP) / (8 · η · L)
///
/// Valid for steady, laminar, incompressible, Newtonian flow in a
/// straight cylindrical tube of constant cross section.
///
/// # Arguments
/// * `radius` - Capillary radius in meters
/// * `length` - Capillary length in meters
/// * `pressure_drop` - Pressure drop across capillary in Pa
/// * `viscosity` - Dynamic viscosity in Pa·s
///
/// # Returns
/// Volumetric flow rate in m³/s
pub fn poiseuille_flow(radius: f64, length: f64, pressure_drop: f64, viscosity: f64) -> f64 {
    assert!(radius > 0.0, "Radius must be positive");
    assert!(length > 0.0, "Length must be positive");
    assert!(viscosity > 0.0, "Viscosity must be positive");
    let r4 = radius * radius * radius * radius;
    (PI * r4 * pressure_drop) / (8.0 * viscosity * length)
}

/// Calculate Reynolds number for capillary flow
///
/// Re = ρ · v · d / η = 2 · ρ · Q / (π · r · η)
///
/// # Arguments
/// * `flow_rate` - Volumetric flow rate in m³/s
/// * `radius` - Capillary radius in meters
/// * `density` - Fluid density in kg/m³
/// * `viscosity` - Dynamic viscosity in Pa·s
///
/// # Returns
/// Reynolds number (dimensionless). Flow is laminar if Re < 2300.
pub fn reynolds_number(flow_rate: f64, radius: f64, density: f64, viscosity: f64) -> f64 {
    assert!(radius > 0.0, "Radius must be positive");
    assert!(viscosity > 0.0, "Viscosity must be positive");
    (2.0 * density * flow_rate.abs()) / (PI * radius * viscosity)
}

/// Calculate wall shear rate in a capillary
///
/// γ_w = 4Q / (π · r³)
///
/// # Arguments
/// * `flow_rate` - Volumetric flow rate in m³/s
/// * `radius` - Capillary radius in meters
///
/// # Returns
/// Wall shear rate in s⁻¹
pub fn wall_shear_rate(flow_rate: f64, radius: f64) -> f64 {
    assert!(radius > 0.0, "Radius must be positive");
    let r3 = radius * radius * radius;
    (4.0 * flow_rate.abs()) / (PI * r3)
}

/// Calculate wall shear stress in a capillary
///
/// τ_w = (r · ΔP) / (2 · L)
///
/// # Arguments
/// * `radius` - Capillary radius in meters
/// * `length` - Capillary length in meters
/// * `pressure_drop` - Pressure drop in Pa
///
/// # Returns
/// Wall shear stress in Pa
pub fn wall_shear_stress(radius: f64, length: f64, pressure_drop: f64) -> f64 {
    assert!(radius > 0.0 && length > 0.0);
    (radius * pressure_drop) / (2.0 * length)
}

// ============================================================================
// Temperature Dependence Models
// ============================================================================

/// Temperature dependence model for viscosity
#[derive(Debug, Clone)]
pub enum TemperatureDependence {
    /// Arrhenius model: η = η_ref · exp(E_a/R · (1/T - 1/T_ref))
    Arrhenius {
        /// Reference viscosity in mPa·s
        eta_ref: f64,
        /// Activation energy in J/mol
        activation_energy: f64,
        /// Reference temperature in Kelvin
        temp_ref: f64,
    },
    /// Vogel-Fulcher-Tammann: η = A · exp(B / (T - T_0))
    VFT {
        /// Pre-exponential factor A in mPa·s
        a: f64,
        /// Activation parameter B in K
        b: f64,
        /// Vogel temperature T_0 in K
        t0: f64,
    },
    /// Williams-Landel-Ferry: log(η/η_ref) = -C1·(T-Tref) / (C2 + T - Tref)
    WLF {
        /// Reference viscosity in mPa·s
        eta_ref: f64,
        /// Reference temperature in K
        temp_ref: f64,
        /// WLF constant C1 (dimensionless)
        c1: f64,
        /// WLF constant C2 in K
        c2: f64,
    },
}

impl TemperatureDependence {
    /// Calculate viscosity at a given temperature
    ///
    /// # Arguments
    /// * `temperature` - Temperature in Kelvin
    ///
    /// # Returns
    /// Dynamic viscosity in mPa·s
    pub fn viscosity_at(&self, temperature: f64) -> f64 {
        match self {
            TemperatureDependence::Arrhenius {
                eta_ref,
                activation_energy,
                temp_ref,
            } => arrhenius_viscosity(*eta_ref, *activation_energy, temperature, *temp_ref),

            TemperatureDependence::VFT { a, b, t0 } => vft_viscosity(*a, *b, *t0, temperature),

            TemperatureDependence::WLF {
                eta_ref,
                temp_ref,
                c1,
                c2,
            } => wlf_viscosity(*eta_ref, *temp_ref, *c1, *c2, temperature),
        }
    }
}

/// Arrhenius viscosity model
///
/// η(T) = η_ref · exp(E_a/R · (1/T - 1/T_ref))
///
/// # Arguments
/// * `eta_ref` - Reference viscosity in mPa·s
/// * `activation_energy` - Flow activation energy in J/mol
/// * `temp` - Temperature in Kelvin
/// * `temp_ref` - Reference temperature in Kelvin
///
/// # Returns
/// Viscosity at the specified temperature in mPa·s
pub fn arrhenius_viscosity(
    eta_ref: f64,
    activation_energy: f64,
    temp: f64,
    temp_ref: f64,
) -> f64 {
    assert!(temp > 0.0 && temp_ref > 0.0, "Temperatures must be positive");
    let exponent = (activation_energy / GAS_CONSTANT_R) * (1.0 / temp - 1.0 / temp_ref);
    eta_ref * exponent.exp()
}

/// Vogel-Fulcher-Tammann (VFT) viscosity model
///
/// η(T) = A · exp(B / (T - T_0))
///
/// More accurate than Arrhenius for glass-forming liquids and polymers near Tg.
///
/// # Arguments
/// * `a` - Pre-exponential factor in mPa·s
/// * `b` - Activation parameter in K
/// * `t0` - Vogel temperature in K (typically 30-70 K below Tg)
/// * `temp` - Temperature in Kelvin
///
/// # Returns
/// Viscosity in mPa·s
pub fn vft_viscosity(a: f64, b: f64, t0: f64, temp: f64) -> f64 {
    assert!(
        temp > t0,
        "Temperature must be above Vogel temperature T0"
    );
    a * (b / (temp - t0)).exp()
}

/// Williams-Landel-Ferry (WLF) viscosity model
///
/// log10(η/η_ref) = -C1 · (T - T_ref) / (C2 + T - T_ref)
///
/// Most commonly used for polymers in the range Tg to Tg+100K.
/// Universal constants: C1 ≈ 17.44, C2 ≈ 51.6 K (when T_ref = Tg).
///
/// # Arguments
/// * `eta_ref` - Reference viscosity at T_ref in mPa·s
/// * `temp_ref` - Reference temperature in K
/// * `c1` - WLF constant C1 (dimensionless)
/// * `c2` - WLF constant C2 in K
/// * `temp` - Temperature in K
///
/// # Returns
/// Viscosity in mPa·s
pub fn wlf_viscosity(eta_ref: f64, temp_ref: f64, c1: f64, c2: f64, temp: f64) -> f64 {
    let dt = temp - temp_ref;
    let denom = c2 + dt;
    assert!(denom.abs() > 1e-15, "WLF denominator (C2 + T - Tref) must not be zero");
    let log_shift = -c1 * dt / denom;
    // log10(η/η_ref) = log_shift => η = η_ref · 10^log_shift
    eta_ref * (10.0_f64).powf(log_shift)
}

// ============================================================================
// Linear Regression
// ============================================================================

/// Result of a linear regression fit
#[derive(Debug, Clone)]
pub struct LinearFitResult {
    /// Slope of the fit line
    pub slope: f64,
    /// Y-intercept of the fit line
    pub intercept: f64,
    /// Coefficient of determination (R²)
    pub r_squared: f64,
    /// Number of data points
    pub n: usize,
}

/// Perform ordinary least-squares linear regression: y = slope·x + intercept
///
/// Uses the standard formulas:
/// - slope = (N·Σxy - Σx·Σy) / (N·Σx² - (Σx)²)
/// - intercept = (Σy - slope·Σx) / N
///
/// # Arguments
/// * `x` - Independent variable values
/// * `y` - Dependent variable values
///
/// # Returns
/// `LinearFitResult` with slope, intercept, and R²
pub fn linear_regression(x: &[f64], y: &[f64]) -> LinearFitResult {
    let n = x.len();
    assert!(n >= 2, "Need at least 2 data points for linear regression");
    assert_eq!(n, y.len(), "x and y must have the same length");

    let n_f = n as f64;
    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sum_x2: f64 = x.iter().map(|xi| xi * xi).sum();
    let sum_y2: f64 = y.iter().map(|yi| yi * yi).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    assert!(denom.abs() > 1e-30, "Degenerate data: all x values are identical");

    let slope = (n_f * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n_f;

    // Coefficient of determination R²
    let ss_tot = sum_y2 - sum_y * sum_y / n_f;
    let r_squared = if ss_tot.abs() < 1e-30 {
        1.0 // Perfect fit if all y values are identical
    } else {
        let ss_res: f64 = x
            .iter()
            .zip(y.iter())
            .map(|(xi, yi)| {
                let residual = yi - (slope * xi + intercept);
                residual * residual
            })
            .sum();
        1.0 - ss_res / ss_tot
    };

    LinearFitResult {
        slope,
        intercept,
        r_squared,
        n,
    }
}

// ============================================================================
// Capillary Viscometer
// ============================================================================

/// Capillary viscometer model for kinematic viscosity measurement
///
/// Computes kinematic viscosity from flow time using:
///
/// ν = K·t - H/t
///
/// where K is the viscometer constant and H is the Hagenbach correction.
#[derive(Debug, Clone)]
pub struct CapillaryViscometer {
    pub capillary_type: CapillaryType,
    pub calibration: CapillaryCalibration,
    pub apply_hagenbach: bool,
}

impl CapillaryViscometer {
    /// Create a new capillary viscometer
    pub fn new(capillary_type: CapillaryType, calibration: CapillaryCalibration) -> Self {
        Self {
            capillary_type,
            calibration,
            apply_hagenbach: true,
        }
    }

    /// Ubbelohde viscometer with typical parameters
    pub fn ubbelohde() -> Self {
        Self::new(CapillaryType::Ubbelohde, CapillaryCalibration::ubbelohde_typical())
    }

    /// Ostwald viscometer with typical parameters
    pub fn ostwald() -> Self {
        Self::new(CapillaryType::Ostwald, CapillaryCalibration::ostwald_typical())
    }

    /// Disable Hagenbach correction
    pub fn without_hagenbach(mut self) -> Self {
        self.apply_hagenbach = false;
        self
    }

    /// Calculate kinematic viscosity from flow time
    ///
    /// ν = K·t (without Hagenbach correction)
    /// ν = K·t_corrected (with Hagenbach correction)
    ///
    /// # Arguments
    /// * `flow_time` - Measured flow time in seconds
    ///
    /// # Returns
    /// Kinematic viscosity in mm²/s (centistokes)
    pub fn kinematic_viscosity(&self, flow_time: f64) -> f64 {
        let t = if self.apply_hagenbach {
            hagenbach_correction(flow_time, &self.calibration)
        } else {
            flow_time
        };
        self.calibration.constant_k * t
    }

    /// Calculate dynamic viscosity from flow time and density
    ///
    /// η = ν · ρ
    ///
    /// # Arguments
    /// * `flow_time` - Measured flow time in seconds
    /// * `density` - Fluid density in g/mL
    ///
    /// # Returns
    /// Dynamic viscosity in mPa·s (centipoise)
    pub fn dynamic_viscosity(&self, flow_time: f64, density: f64) -> f64 {
        kinematic_to_dynamic(self.kinematic_viscosity(flow_time), density)
    }

    /// Calculate relative viscosity from solution and solvent flow times
    ///
    /// # Arguments
    /// * `t_solution` - Solution flow time in seconds
    /// * `t_solvent` - Solvent flow time in seconds
    ///
    /// # Returns
    /// Relative viscosity (dimensionless)
    pub fn relative_viscosity(&self, t_solution: f64, t_solvent: f64) -> f64 {
        let ts = if self.apply_hagenbach {
            hagenbach_correction(t_solution, &self.calibration)
        } else {
            t_solution
        };
        let tv = if self.apply_hagenbach {
            hagenbach_correction(t_solvent, &self.calibration)
        } else {
            t_solvent
        };
        ts / tv
    }
}

// ============================================================================
// Intrinsic Viscosity Determination
// ============================================================================

/// Result of intrinsic viscosity determination via extrapolation
#[derive(Debug, Clone)]
pub struct IntrinsicViscosityResult {
    /// Intrinsic viscosity [η] in mL/g
    pub intrinsic_viscosity: f64,
    /// Huggins constant k_H (should be ~0.3-0.8)
    pub huggins_k: f64,
    /// R² of the linear fit
    pub r_squared: f64,
    /// Slope of the extrapolation line
    pub slope: f64,
}

/// Determines intrinsic viscosity by extrapolation to zero concentration
///
/// Stores sets of (concentration, η_rel) measurements and provides
/// Huggins and Kraemer extrapolation methods.
#[derive(Debug)]
pub struct IntrinsicViscosityDeterminer {
    /// Concentrations in g/mL
    concentrations: Vec<f64>,
    /// Relative viscosities (dimensionless)
    relative_viscosities: Vec<f64>,
}

impl IntrinsicViscosityDeterminer {
    /// Create a new determiner from concentration and relative viscosity data
    ///
    /// # Arguments
    /// * `concentrations` - Polymer concentrations in g/mL
    /// * `relative_viscosities` - Corresponding relative viscosities
    pub fn new(concentrations: &[f64], relative_viscosities: &[f64]) -> Self {
        assert!(
            concentrations.len() >= 2,
            "Need at least 2 concentration points"
        );
        assert_eq!(
            concentrations.len(),
            relative_viscosities.len(),
            "Concentrations and viscosities must have the same length"
        );

        Self {
            concentrations: concentrations.to_vec(),
            relative_viscosities: relative_viscosities.to_vec(),
        }
    }

    /// Create from flow times (solution and solvent)
    ///
    /// # Arguments
    /// * `concentrations` - Polymer concentrations in g/mL
    /// * `t_solutions` - Flow times of solutions in seconds
    /// * `t_solvent` - Flow time of pure solvent in seconds
    pub fn from_flow_times(
        concentrations: &[f64],
        t_solutions: &[f64],
        t_solvent: f64,
    ) -> Self {
        let rel_viscs: Vec<f64> = t_solutions
            .iter()
            .map(|&t| relative_viscosity(t, t_solvent))
            .collect();
        Self::new(concentrations, &rel_viscs)
    }

    /// Huggins extrapolation: η_sp/c = [η] + k_H·[η]²·c
    ///
    /// Plots reduced viscosity (η_sp/c) vs concentration and extrapolates
    /// to c = 0 to find [η].
    ///
    /// The Huggins constant k_H is calculated from:
    /// slope = k_H · [η]²
    /// k_H = slope / [η]²
    pub fn huggins_extrapolation(&self) -> IntrinsicViscosityResult {
        let y: Vec<f64> = self
            .concentrations
            .iter()
            .zip(self.relative_viscosities.iter())
            .map(|(&c, &eta_rel)| reduced_viscosity(specific_viscosity(eta_rel), c))
            .collect();

        let fit = linear_regression(&self.concentrations, &y);

        let intrinsic_viscosity = fit.intercept;
        let huggins_k = if intrinsic_viscosity.abs() > 1e-15 {
            fit.slope / (intrinsic_viscosity * intrinsic_viscosity)
        } else {
            0.0
        };

        IntrinsicViscosityResult {
            intrinsic_viscosity,
            huggins_k,
            r_squared: fit.r_squared,
            slope: fit.slope,
        }
    }

    /// Kraemer extrapolation: ln(η_rel)/c = [η] + k_K·[η]²·c
    ///
    /// Plots inherent viscosity (ln(η_rel)/c) vs concentration and extrapolates
    /// to c = 0 to find [η].
    ///
    /// Note: k_H + k_K ≈ 0.5 is expected for consistent data.
    pub fn kraemer_extrapolation(&self) -> IntrinsicViscosityResult {
        let y: Vec<f64> = self
            .concentrations
            .iter()
            .zip(self.relative_viscosities.iter())
            .map(|(&c, &eta_rel)| inherent_viscosity(eta_rel, c))
            .collect();

        let fit = linear_regression(&self.concentrations, &y);

        let intrinsic_viscosity = fit.intercept;
        let kraemer_k = if intrinsic_viscosity.abs() > 1e-15 {
            fit.slope / (intrinsic_viscosity * intrinsic_viscosity)
        } else {
            0.0
        };

        IntrinsicViscosityResult {
            intrinsic_viscosity,
            huggins_k: kraemer_k, // Stored in same field for simplicity
            r_squared: fit.r_squared,
            slope: fit.slope,
        }
    }

    /// Combined Huggins-Kraemer determination
    ///
    /// Returns results from both methods. The intrinsic viscosity
    /// values should agree within experimental error. The average
    /// is also returned.
    pub fn combined_extrapolation(&self) -> (IntrinsicViscosityResult, IntrinsicViscosityResult, f64) {
        let huggins = self.huggins_extrapolation();
        let kraemer = self.kraemer_extrapolation();
        let avg = (huggins.intrinsic_viscosity + kraemer.intrinsic_viscosity) / 2.0;
        (huggins, kraemer, avg)
    }

    /// Get reduced viscosities for all concentration points
    pub fn reduced_viscosities(&self) -> Vec<f64> {
        self.concentrations
            .iter()
            .zip(self.relative_viscosities.iter())
            .map(|(&c, &eta_rel)| reduced_viscosity(specific_viscosity(eta_rel), c))
            .collect()
    }

    /// Get inherent viscosities for all concentration points
    pub fn inherent_viscosities(&self) -> Vec<f64> {
        self.concentrations
            .iter()
            .zip(self.relative_viscosities.iter())
            .map(|(&c, &eta_rel)| inherent_viscosity(eta_rel, c))
            .collect()
    }
}

// ============================================================================
// Huggins Extrapolation (standalone)
// ============================================================================

/// Standalone Huggins extrapolation from reduced viscosity data
///
/// η_sp/c = [η] + k_H·[η]²·c
///
/// # Arguments
/// * `concentrations` - Polymer concentrations in g/mL
/// * `reduced_viscosities` - Corresponding reduced viscosities in mL/g
///
/// # Returns
/// (intrinsic_viscosity, huggins_k, r_squared)
pub fn huggins_extrapolation(
    concentrations: &[f64],
    reduced_viscosities: &[f64],
) -> (f64, f64, f64) {
    let fit = linear_regression(concentrations, reduced_viscosities);
    let iv = fit.intercept;
    let kh = if iv.abs() > 1e-15 {
        fit.slope / (iv * iv)
    } else {
        0.0
    };
    (iv, kh, fit.r_squared)
}

/// Standalone Kraemer extrapolation from inherent viscosity data
///
/// ln(η_rel)/c = [η] + k_K·[η]²·c
///
/// # Arguments
/// * `concentrations` - Polymer concentrations in g/mL
/// * `inherent_viscosities` - Corresponding inherent viscosities in mL/g
///
/// # Returns
/// (intrinsic_viscosity, kraemer_k, r_squared)
pub fn kraemer_extrapolation(
    concentrations: &[f64],
    inherent_viscosities: &[f64],
) -> (f64, f64, f64) {
    let fit = linear_regression(concentrations, inherent_viscosities);
    let iv = fit.intercept;
    let kk = if iv.abs() > 1e-15 {
        fit.slope / (iv * iv)
    } else {
        0.0
    };
    (iv, kk, fit.r_squared)
}

// ============================================================================
// Mark-Houwink-Sakurada
// ============================================================================

/// Mark-Houwink-Sakurada calculator for molecular weight estimation
///
/// Uses the relationship: [η] = K · M^a
#[derive(Debug, Clone)]
pub struct MarkHouwinkCalculator {
    params: MarkHouwinkParams,
}

impl MarkHouwinkCalculator {
    /// Create a new calculator with the given MHS parameters
    pub fn new(params: MarkHouwinkParams) -> Self {
        Self { params }
    }

    /// PEO in water at 25°C
    pub fn peo_water() -> Self {
        Self::new(MarkHouwinkParams::peo_water_25c())
    }

    /// Polystyrene in THF at 25°C
    pub fn ps_thf() -> Self {
        Self::new(MarkHouwinkParams::ps_thf_25c())
    }

    /// Estimate molecular weight from intrinsic viscosity
    ///
    /// M = ([η] / K)^(1/a)
    ///
    /// # Arguments
    /// * `intrinsic_viscosity` - Intrinsic viscosity [η] in mL/g
    ///
    /// # Returns
    /// Viscosity-average molecular weight in g/mol (Da)
    pub fn molecular_weight(&self, intrinsic_viscosity: f64) -> f64 {
        mark_houwink_mw(intrinsic_viscosity, self.params.k, self.params.a)
    }

    /// Calculate intrinsic viscosity from molecular weight
    ///
    /// [η] = K · M^a
    ///
    /// # Arguments
    /// * `mw` - Molecular weight in g/mol
    ///
    /// # Returns
    /// Intrinsic viscosity in mL/g
    pub fn intrinsic_viscosity(&self, mw: f64) -> f64 {
        self.params.k * mw.powf(self.params.a)
    }

    /// Determine MHS parameters from multiple ([η], M) data pairs
    ///
    /// Uses log-log linear regression:
    /// log([η]) = log(K) + a·log(M)
    ///
    /// # Arguments
    /// * `intrinsic_viscosities` - [η] values in mL/g
    /// * `molecular_weights` - Corresponding MW values in g/mol
    ///
    /// # Returns
    /// Fitted MarkHouwinkParams
    pub fn fit_parameters(
        intrinsic_viscosities: &[f64],
        molecular_weights: &[f64],
    ) -> MarkHouwinkParams {
        let log_iv: Vec<f64> = intrinsic_viscosities.iter().map(|&v| v.ln()).collect();
        let log_mw: Vec<f64> = molecular_weights.iter().map(|&m| m.ln()).collect();

        let fit = linear_regression(&log_mw, &log_iv);
        let a = fit.slope;
        let k = fit.intercept.exp();

        MarkHouwinkParams {
            k,
            a,
            description: "Fitted from experimental data",
        }
    }
}

/// Estimate molecular weight from intrinsic viscosity using Mark-Houwink equation
///
/// M = ([η] / K)^(1/a)
///
/// # Arguments
/// * `intrinsic_visc` - Intrinsic viscosity [η] in mL/g
/// * `k` - Mark-Houwink K constant
/// * `a` - Mark-Houwink exponent
///
/// # Returns
/// Molecular weight in g/mol
pub fn mark_houwink_mw(intrinsic_visc: f64, k: f64, a: f64) -> f64 {
    assert!(k > 0.0, "K must be positive");
    assert!(a > 0.0, "Exponent a must be positive");
    assert!(intrinsic_visc > 0.0, "Intrinsic viscosity must be positive");
    (intrinsic_visc / k).powf(1.0 / a)
}

/// Calculate intrinsic viscosity from molecular weight using Mark-Houwink equation
///
/// [η] = K · M^a
pub fn mark_houwink_iv(mw: f64, k: f64, a: f64) -> f64 {
    assert!(k > 0.0 && mw > 0.0 && a > 0.0);
    k * mw.powf(a)
}

// ============================================================================
// Solomon-Ciuta Single-Point Method
// ============================================================================

/// Solomon-Ciuta single-point intrinsic viscosity estimation
///
/// [η] = (1/c) · √(2 · (η_sp - ln(η_rel)))
///
/// This approximation is valid when the Huggins constant k_H ≈ 0.5,
/// eliminating the need for multi-concentration measurements.
///
/// # Arguments
/// * `eta_rel` - Relative viscosity
/// * `concentration` - Polymer concentration in g/mL
///
/// # Returns
/// Estimated intrinsic viscosity in mL/g
pub fn solomon_ciuta(eta_rel: f64, concentration: f64) -> f64 {
    assert!(concentration > 0.0, "Concentration must be positive");
    assert!(eta_rel > 1.0, "Relative viscosity must be > 1 for a polymer solution");
    let eta_sp = specific_viscosity(eta_rel);
    let ln_eta_rel = eta_rel.ln();
    let arg = 2.0 * (eta_sp - ln_eta_rel);
    if arg >= 0.0 {
        arg.sqrt() / concentration
    } else {
        // Fallback: use reduced viscosity as approximation
        eta_sp / concentration
    }
}

// ============================================================================
// Schulz-Blaschke Single-Point Method
// ============================================================================

/// Schulz-Blaschke single-point intrinsic viscosity estimation
///
/// [η] = η_sp / (c · (1 + k_SB · η_sp))
///
/// where k_SB ≈ 0.28 is the Schulz-Blaschke constant.
///
/// # Arguments
/// * `eta_rel` - Relative viscosity
/// * `concentration` - Polymer concentration in g/mL
/// * `k_sb` - Schulz-Blaschke constant (typically 0.28)
///
/// # Returns
/// Estimated intrinsic viscosity in mL/g
pub fn schulz_blaschke(eta_rel: f64, concentration: f64, k_sb: f64) -> f64 {
    assert!(concentration > 0.0);
    let eta_sp = specific_viscosity(eta_rel);
    eta_sp / (concentration * (1.0 + k_sb * eta_sp))
}

// ============================================================================
// Viscosity-Average Degree of Polymerization
// ============================================================================

/// Calculate viscosity-average degree of polymerization
///
/// DP_v = M_v / M_0
///
/// where M_v is the viscosity-average molecular weight and M_0 is the
/// monomer molecular weight.
///
/// # Arguments
/// * `mw_viscosity_avg` - Viscosity-average molecular weight in g/mol
/// * `monomer_mw` - Monomer molecular weight in g/mol
///
/// # Returns
/// Degree of polymerization (dimensionless)
pub fn degree_of_polymerization(mw_viscosity_avg: f64, monomer_mw: f64) -> f64 {
    assert!(monomer_mw > 0.0, "Monomer MW must be positive");
    mw_viscosity_avg / monomer_mw
}

// ============================================================================
// Huggins-Kraemer Consistency Check
// ============================================================================

/// Check consistency between Huggins and Kraemer extrapolation results
///
/// For consistent data: k_H + k_K ≈ 0.5
///
/// # Arguments
/// * `k_h` - Huggins constant
/// * `k_k` - Kraemer constant
///
/// # Returns
/// (sum, is_consistent) where is_consistent is true if |sum - 0.5| < 0.1
pub fn huggins_kraemer_consistency(k_h: f64, k_k: f64) -> (f64, bool) {
    let sum = k_h + k_k;
    let is_consistent = (sum - 0.5).abs() < 0.1;
    (sum, is_consistent)
}

// ============================================================================
// Concentration Conversion
// ============================================================================

/// Convert concentration from g/dL to g/mL
pub fn g_per_dl_to_g_per_ml(c_g_per_dl: f64) -> f64 {
    c_g_per_dl / 100.0
}

/// Convert concentration from g/mL to g/dL
pub fn g_per_ml_to_g_per_dl(c_g_per_ml: f64) -> f64 {
    c_g_per_ml * 100.0
}

/// Convert intrinsic viscosity from dL/g to mL/g
pub fn dl_per_g_to_ml_per_g(iv_dl_per_g: f64) -> f64 {
    iv_dl_per_g * 100.0
}

/// Convert intrinsic viscosity from mL/g to dL/g
pub fn ml_per_g_to_dl_per_g(iv_ml_per_g: f64) -> f64 {
    iv_ml_per_g / 100.0
}

// ============================================================================
// Viscosity Blending
// ============================================================================

/// Calculate the viscosity of a polymer blend using the log mixing rule
///
/// ln(η_blend) = w1·ln(η1) + w2·ln(η2)
///
/// # Arguments
/// * `viscosities` - Component viscosities
/// * `weight_fractions` - Corresponding weight fractions (must sum to ~1)
///
/// # Returns
/// Blend viscosity
pub fn blend_viscosity_log_rule(viscosities: &[f64], weight_fractions: &[f64]) -> f64 {
    assert_eq!(viscosities.len(), weight_fractions.len());
    let ln_blend: f64 = viscosities
        .iter()
        .zip(weight_fractions.iter())
        .map(|(&v, &w)| w * v.ln())
        .sum();
    ln_blend.exp()
}

/// Calculate the viscosity of a polymer blend using Arrhenius mixing rule
///
/// ln(η_blend) = Σ x_i · ln(η_i)
///
/// Same as log rule but typically used with mole fractions.
pub fn blend_viscosity_arrhenius(viscosities: &[f64], mole_fractions: &[f64]) -> f64 {
    blend_viscosity_log_rule(viscosities, mole_fractions)
}

// ============================================================================
// Relative Viscosity from Dynamic Viscosities
// ============================================================================

/// Calculate relative viscosity from dynamic viscosity values
///
/// η_rel = η_solution / η_solvent
///
/// # Arguments
/// * `eta_solution` - Dynamic viscosity of solution in mPa·s
/// * `eta_solvent` - Dynamic viscosity of solvent in mPa·s
///
/// # Returns
/// Relative viscosity (dimensionless)
pub fn relative_viscosity_from_dynamic(eta_solution: f64, eta_solvent: f64) -> f64 {
    assert!(eta_solvent > 0.0, "Solvent viscosity must be positive");
    eta_solution / eta_solvent
}

// ============================================================================
// Hydrodynamic Radius
// ============================================================================

/// Estimate hydrodynamic radius from intrinsic viscosity and molecular weight
///
/// Using the Einstein-Simha relation for hard spheres:
///
/// [η] = (10π/3) · N_A · R_h³ / M
///
/// Solving for R_h:
///
/// R_h = (3·[η]·M / (10π·N_A))^(1/3)
///
/// # Arguments
/// * `intrinsic_viscosity` - [η] in mL/g
/// * `molecular_weight` - M in g/mol
///
/// # Returns
/// Hydrodynamic radius in cm (note: mL/g = cm³/g)
pub fn hydrodynamic_radius(intrinsic_viscosity: f64, molecular_weight: f64) -> f64 {
    const AVOGADRO: f64 = 6.022e23;
    let numerator = 3.0 * intrinsic_viscosity * molecular_weight;
    let denominator = 10.0 * PI * AVOGADRO;
    (numerator / denominator).cbrt()
}

// ============================================================================
// Kinetic Energy Correction Factor
// ============================================================================

/// Calculate the Couette correction factor for capillary exit effects
///
/// The correction accounts for the kinetic energy dissipated at the
/// capillary exit and entry. For well-designed viscometers, this is
/// typically small (< 2% of flow time).
///
/// # Arguments
/// * `flow_time` - Measured flow time in seconds
/// * `capillary_volume` - Volume of liquid that flows through in m³
/// * `capillary_radius` - Capillary radius in meters
///
/// # Returns
/// Correction factor (multiply measured time by this to get corrected time)
pub fn couette_correction_factor(
    flow_time: f64,
    capillary_volume: f64,
    capillary_radius: f64,
) -> f64 {
    assert!(flow_time > 0.0 && capillary_radius > 0.0);
    let v_avg = capillary_volume / (PI * capillary_radius * capillary_radius * flow_time);
    let correction_time = v_avg * v_avg / (2.0 * 9.81 * 0.1); // approximate
    1.0 - correction_time / flow_time
}

// ============================================================================
// Batch Processor
// ============================================================================

/// Result of a complete viscometry analysis run
#[derive(Debug, Clone)]
pub struct ViscometryAnalysisResult {
    /// Concentrations used (g/mL)
    pub concentrations: Vec<f64>,
    /// Relative viscosities
    pub relative_viscosities: Vec<f64>,
    /// Specific viscosities
    pub specific_viscosities: Vec<f64>,
    /// Reduced viscosities (mL/g)
    pub reduced_viscosities: Vec<f64>,
    /// Inherent viscosities (mL/g)
    pub inherent_viscosities: Vec<f64>,
    /// Huggins extrapolation result
    pub huggins_result: IntrinsicViscosityResult,
    /// Kraemer extrapolation result
    pub kraemer_result: IntrinsicViscosityResult,
    /// Average intrinsic viscosity from both methods
    pub intrinsic_viscosity_avg: f64,
    /// Estimated molecular weight (if MHS params provided)
    pub molecular_weight: Option<f64>,
}

/// Full viscometry analysis from flow time data
///
/// Performs a complete viscometric analysis given flow times at multiple
/// concentrations.
///
/// # Arguments
/// * `config` - Viscometry configuration
/// * `concentrations` - Polymer concentrations in g/mL
/// * `t_solutions` - Solution flow times in seconds
/// * `t_solvent` - Solvent flow time in seconds
///
/// # Returns
/// Complete analysis result
pub fn full_analysis(
    config: &ViscometryConfig,
    concentrations: &[f64],
    t_solutions: &[f64],
    t_solvent: f64,
) -> ViscometryAnalysisResult {
    assert_eq!(concentrations.len(), t_solutions.len());
    assert!(concentrations.len() >= 2);

    // Apply Hagenbach correction if configured
    let corrected_solutions: Vec<f64> = if config.apply_hagenbach {
        t_solutions
            .iter()
            .map(|&t| hagenbach_correction(t, &config.calibration))
            .collect()
    } else {
        t_solutions.to_vec()
    };
    let corrected_solvent = if config.apply_hagenbach {
        hagenbach_correction(t_solvent, &config.calibration)
    } else {
        t_solvent
    };

    let rel_viscs: Vec<f64> = corrected_solutions
        .iter()
        .map(|&t| relative_viscosity(t, corrected_solvent))
        .collect();
    let spec_viscs: Vec<f64> = rel_viscs.iter().map(|&r| specific_viscosity(r)).collect();
    let red_viscs: Vec<f64> = spec_viscs
        .iter()
        .zip(concentrations.iter())
        .map(|(&sp, &c)| reduced_viscosity(sp, c))
        .collect();
    let inh_viscs: Vec<f64> = rel_viscs
        .iter()
        .zip(concentrations.iter())
        .map(|(&r, &c)| inherent_viscosity(r, c))
        .collect();

    let determiner = IntrinsicViscosityDeterminer::new(concentrations, &rel_viscs);
    let huggins = determiner.huggins_extrapolation();
    let kraemer = determiner.kraemer_extrapolation();
    let avg_iv = (huggins.intrinsic_viscosity + kraemer.intrinsic_viscosity) / 2.0;

    let mw = config.mark_houwink.as_ref().map(|mhs| {
        mark_houwink_mw(avg_iv, mhs.k, mhs.a)
    });

    ViscometryAnalysisResult {
        concentrations: concentrations.to_vec(),
        relative_viscosities: rel_viscs,
        specific_viscosities: spec_viscs,
        reduced_viscosities: red_viscs,
        inherent_viscosities: inh_viscs,
        huggins_result: huggins,
        kraemer_result: kraemer,
        intrinsic_viscosity_avg: avg_iv,
        molecular_weight: mw,
    }
}

// ============================================================================
// Polymer Solution Quality Indicators
// ============================================================================

/// Determine solvent quality from the Mark-Houwink exponent `a`
///
/// - a < 0.5: poor solvent (collapsed coil)
/// - a ≈ 0.5: theta solvent (ideal, unperturbed dimensions)
/// - 0.5 < a < 0.8: good solvent (expanded coil)
/// - a ≈ 1.0: rigid rod (stiff chain)
///
/// # Arguments
/// * `a` - Mark-Houwink exponent
///
/// # Returns
/// Description of solvent quality
pub fn solvent_quality(a: f64) -> &'static str {
    if a < 0.45 {
        "poor solvent (collapsed coil)"
    } else if a < 0.55 {
        "theta solvent (unperturbed dimensions)"
    } else if a < 0.85 {
        "good solvent (expanded coil)"
    } else {
        "rigid rod / stiff chain"
    }
}

/// Calculate the expansion factor α from Mark-Houwink exponent
///
/// For a flexible chain: a = 3ν - 1, where ν is the Flory exponent.
/// The expansion factor relates to the excluded volume effect.
///
/// - ν = 0.5: theta conditions (α = 1)
/// - ν = 0.588: good solvent (Flory prediction)
///
/// # Arguments
/// * `a` - Mark-Houwink exponent
///
/// # Returns
/// Flory exponent ν
pub fn flory_exponent(a: f64) -> f64 {
    (a + 1.0) / 3.0
}

// ============================================================================
// Viscosity Index
// ============================================================================

/// Calculate the Viscosity Index (VI) per ASTM D2270
///
/// VI = (L - U) / (L - H) × 100
///
/// where:
/// - U = kinematic viscosity of sample at 40°C
/// - L = kinematic viscosity of a 0 VI reference oil at 40°C with same viscosity at 100°C
/// - H = kinematic viscosity of a 100 VI reference oil at 40°C with same viscosity at 100°C
///
/// Simplified calculation using empirical correlations for mineral oils.
///
/// # Arguments
/// * `visc_40` - Kinematic viscosity at 40°C in mm²/s (cSt)
/// * `visc_100` - Kinematic viscosity at 100°C in mm²/s (cSt)
///
/// # Returns
/// Viscosity index (dimensionless)
pub fn viscosity_index(visc_40: f64, visc_100: f64) -> f64 {
    assert!(visc_40 > 0.0 && visc_100 > 0.0);
    // Dean and Davis empirical correlation (simplified)
    let y = visc_100;
    // Approximate L and H reference oils
    let l = 0.8353 * y * y + 14.67 * y - 216.0;
    let h = 0.1684 * y * y + 11.85 * y - 97.0;

    if (l - h).abs() < 1e-10 {
        100.0
    } else {
        ((l - visc_40) / (l - h)) * 100.0
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;
    const EPSILON_LOOSE: f64 = 1e-3;

    // --- Basic viscosity calculations ---

    #[test]
    fn test_relative_viscosity_basic() {
        let eta_rel = relative_viscosity(120.0, 100.0);
        assert!((eta_rel - 1.2).abs() < EPSILON);
    }

    #[test]
    fn test_relative_viscosity_equal_times() {
        let eta_rel = relative_viscosity(100.0, 100.0);
        assert!((eta_rel - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_specific_viscosity_basic() {
        let eta_sp = specific_viscosity(1.2);
        assert!((eta_sp - 0.2).abs() < EPSILON);
    }

    #[test]
    fn test_specific_viscosity_pure_solvent() {
        let eta_sp = specific_viscosity(1.0);
        assert!(eta_sp.abs() < EPSILON);
    }

    #[test]
    fn test_reduced_viscosity_basic() {
        let eta_red = reduced_viscosity(0.2, 0.005);
        assert!((eta_red - 40.0).abs() < EPSILON);
    }

    #[test]
    fn test_inherent_viscosity_basic() {
        let eta_rel = 1.2;
        let c = 0.005;
        let eta_inh = inherent_viscosity(eta_rel, c);
        let expected = (1.2_f64).ln() / 0.005;
        assert!((eta_inh - expected).abs() < EPSILON);
    }

    #[test]
    fn test_inherent_viscosity_nearly_pure() {
        let eta_inh = inherent_viscosity(1.001, 0.001);
        // ln(1.001)/0.001 ≈ 0.9995
        assert!((eta_inh - 0.9995).abs() < 0.01);
    }

    #[test]
    fn test_reduced_vs_inherent_at_low_concentration() {
        // At very dilute solutions, η_red ≈ η_inh
        let eta_rel = 1.005;
        let c = 0.001;
        let eta_sp = specific_viscosity(eta_rel);
        let eta_red = reduced_viscosity(eta_sp, c);
        let eta_inh = inherent_viscosity(eta_rel, c);
        // Should be close for dilute solutions
        assert!((eta_red - eta_inh).abs() < 0.1);
    }

    // --- Kinematic-dynamic conversion ---

    #[test]
    fn test_kinematic_to_dynamic() {
        let eta = kinematic_to_dynamic(1.004, 0.998);
        assert!((eta - 1.002).abs() < 0.001);
    }

    #[test]
    fn test_dynamic_to_kinematic() {
        let nu = dynamic_to_kinematic(1.002, 0.998);
        assert!((nu - 1.004).abs() < 0.001);
    }

    #[test]
    fn test_kinematic_dynamic_roundtrip() {
        let nu_orig = 1.5;
        let rho = 0.95;
        let eta = kinematic_to_dynamic(nu_orig, rho);
        let nu_back = dynamic_to_kinematic(eta, rho);
        assert!((nu_orig - nu_back).abs() < EPSILON);
    }

    // --- Water constants ---

    #[test]
    fn test_water_viscosity_constants() {
        assert!((WATER_VISCOSITY_20C - 1.002).abs() < EPSILON);
        assert!((WATER_VISCOSITY_25C - 0.8901).abs() < EPSILON);
    }

    #[test]
    fn test_water_density_constants() {
        assert!((WATER_DENSITY_20C - 0.99821).abs() < EPSILON);
        assert!((WATER_DENSITY_25C - 0.99705).abs() < EPSILON);
    }

    #[test]
    fn test_gas_constant() {
        assert!((GAS_CONSTANT_R - 8.314).abs() < EPSILON);
    }

    // --- Hagenbach correction ---

    #[test]
    fn test_hagenbach_correction_basic() {
        let cal = CapillaryCalibration::ubbelohde_typical();
        let corrected = hagenbach_correction(200.0, &cal);
        // t_corrected = 200 - 1.12/200 = 200 - 0.0056 = 199.9944
        assert!((corrected - 199.9944).abs() < EPSILON);
    }

    #[test]
    fn test_hagenbach_correction_short_time() {
        let cal = CapillaryCalibration {
            hagenbach_coeff: 100.0,
            ..CapillaryCalibration::ubbelohde_typical()
        };
        // For very short flow time, correction might make it negative
        // The function should clamp to positive
        let corrected = hagenbach_correction(5.0, &cal);
        // 5 - 100/5 = 5 - 20 = -15 → clamped to 5.0
        assert!((corrected - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_hagenbach_correction_large_time() {
        let cal = CapillaryCalibration::ostwald_typical();
        let corrected = hagenbach_correction(500.0, &cal);
        let expected = 500.0 - 1.5 / 500.0;
        assert!((corrected - expected).abs() < EPSILON);
    }

    // --- Poiseuille flow ---

    #[test]
    fn test_poiseuille_flow_basic() {
        let q = poiseuille_flow(0.001, 0.1, 1000.0, 0.001);
        // Q = π × (0.001)^4 × 1000 / (8 × 0.001 × 0.1)
        let expected = PI * 1e-12 * 1000.0 / (8.0 * 0.001 * 0.1);
        assert!((q - expected).abs() / expected < EPSILON);
    }

    #[test]
    fn test_poiseuille_doubles_with_pressure() {
        let q1 = poiseuille_flow(0.001, 0.1, 1000.0, 0.001);
        let q2 = poiseuille_flow(0.001, 0.1, 2000.0, 0.001);
        assert!((q2 / q1 - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_poiseuille_r4_dependence() {
        let q1 = poiseuille_flow(0.001, 0.1, 1000.0, 0.001);
        let q2 = poiseuille_flow(0.002, 0.1, 1000.0, 0.001);
        // Doubling radius should increase flow by 16x
        assert!((q2 / q1 - 16.0).abs() < EPSILON);
    }

    #[test]
    fn test_poiseuille_inverse_length() {
        let q1 = poiseuille_flow(0.001, 0.1, 1000.0, 0.001);
        let q2 = poiseuille_flow(0.001, 0.2, 1000.0, 0.001);
        assert!((q1 / q2 - 2.0).abs() < EPSILON);
    }

    // --- Reynolds number ---

    #[test]
    fn test_reynolds_number_laminar() {
        let re = reynolds_number(1e-7, 0.0005, 1000.0, 0.001);
        // Should be laminar (< 2300)
        assert!(re < 2300.0);
    }

    #[test]
    fn test_reynolds_number_proportional_to_flow() {
        let re1 = reynolds_number(1e-7, 0.0005, 1000.0, 0.001);
        let re2 = reynolds_number(2e-7, 0.0005, 1000.0, 0.001);
        assert!((re2 / re1 - 2.0).abs() < EPSILON);
    }

    // --- Wall shear rate and stress ---

    #[test]
    fn test_wall_shear_rate() {
        let gamma = wall_shear_rate(1e-6, 0.001);
        let expected = 4.0 * 1e-6 / (PI * 1e-9);
        assert!((gamma - expected).abs() / expected < EPSILON);
    }

    #[test]
    fn test_wall_shear_stress() {
        let tau = wall_shear_stress(0.001, 0.1, 10000.0);
        let expected = 0.001 * 10000.0 / (2.0 * 0.1);
        assert!((tau - expected).abs() < EPSILON);
    }

    // --- Temperature models ---

    #[test]
    fn test_arrhenius_at_reference() {
        let eta = arrhenius_viscosity(1.0, 15000.0, 298.15, 298.15);
        assert!((eta - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_arrhenius_higher_temp_lower_viscosity() {
        let eta = arrhenius_viscosity(1.0, 15000.0, 308.15, 298.15);
        assert!(eta < 1.0);
    }

    #[test]
    fn test_arrhenius_lower_temp_higher_viscosity() {
        let eta = arrhenius_viscosity(1.0, 15000.0, 288.15, 298.15);
        assert!(eta > 1.0);
    }

    #[test]
    fn test_arrhenius_symmetry() {
        let eta_ref = 1.0;
        let ea = 15000.0;
        let t_ref = 300.0;
        let eta_high = arrhenius_viscosity(eta_ref, ea, t_ref + 10.0, t_ref);
        let eta_low = arrhenius_viscosity(eta_ref, ea, t_ref - 10.0, t_ref);
        // Product should be close to eta_ref^2 for small temperature differences
        assert!(eta_high < eta_ref);
        assert!(eta_low > eta_ref);
    }

    #[test]
    fn test_vft_basic() {
        let eta = vft_viscosity(0.01, 500.0, 200.0, 350.0);
        let expected = 0.01_f64 * (500.0_f64 / (350.0 - 200.0)).exp();
        assert!((eta - expected).abs() / expected < EPSILON);
    }

    #[test]
    fn test_vft_higher_temp_lower_viscosity() {
        let eta1 = vft_viscosity(0.01, 500.0, 200.0, 350.0);
        let eta2 = vft_viscosity(0.01, 500.0, 200.0, 400.0);
        assert!(eta2 < eta1);
    }

    #[test]
    fn test_wlf_at_reference() {
        let eta = wlf_viscosity(1000.0, 373.0, 17.44, 51.6, 373.0);
        assert!((eta - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_wlf_above_tref() {
        let eta = wlf_viscosity(1000.0, 373.0, 17.44, 51.6, 383.0);
        assert!(eta < 1000.0);
    }

    #[test]
    fn test_temperature_dependence_enum_arrhenius() {
        let model = TemperatureDependence::Arrhenius {
            eta_ref: 1.0,
            activation_energy: 15000.0,
            temp_ref: 298.15,
        };
        let eta = model.viscosity_at(298.15);
        assert!((eta - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_temperature_dependence_enum_vft() {
        let model = TemperatureDependence::VFT {
            a: 0.01,
            b: 500.0,
            t0: 200.0,
        };
        let eta = model.viscosity_at(350.0);
        let expected = vft_viscosity(0.01, 500.0, 200.0, 350.0);
        assert!((eta - expected).abs() < EPSILON);
    }

    #[test]
    fn test_temperature_dependence_enum_wlf() {
        let model = TemperatureDependence::WLF {
            eta_ref: 1000.0,
            temp_ref: 373.0,
            c1: 17.44,
            c2: 51.6,
        };
        let eta = model.viscosity_at(373.0);
        assert!((eta - 1000.0).abs() < EPSILON);
    }

    // --- Linear regression ---

    #[test]
    fn test_linear_regression_perfect_fit() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let fit = linear_regression(&x, &y);
        assert!((fit.slope - 2.0).abs() < EPSILON);
        assert!(fit.intercept.abs() < EPSILON);
        assert!((fit.r_squared - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_linear_regression_with_intercept() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![5.0, 7.0, 9.0, 11.0, 13.0];
        let fit = linear_regression(&x, &y);
        assert!((fit.slope - 2.0).abs() < EPSILON);
        assert!((fit.intercept - 5.0).abs() < EPSILON);
        assert!((fit.r_squared - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_linear_regression_horizontal_line() {
        let x = vec![1.0, 2.0, 3.0, 4.0];
        let y = vec![5.0, 5.0, 5.0, 5.0];
        let fit = linear_regression(&x, &y);
        assert!(fit.slope.abs() < EPSILON);
        assert!((fit.intercept - 5.0).abs() < EPSILON);
        assert!((fit.r_squared - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_linear_regression_two_points() {
        let x = vec![0.0, 10.0];
        let y = vec![3.0, 13.0];
        let fit = linear_regression(&x, &y);
        assert!((fit.slope - 1.0).abs() < EPSILON);
        assert!((fit.intercept - 3.0).abs() < EPSILON);
        assert!((fit.r_squared - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_linear_regression_negative_slope() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![10.0, 8.0, 6.0, 4.0, 2.0];
        let fit = linear_regression(&x, &y);
        assert!((fit.slope - (-2.0)).abs() < EPSILON);
        assert!((fit.intercept - 12.0).abs() < EPSILON);
    }

    // --- Capillary viscometer ---

    #[test]
    fn test_capillary_ubbelohde() {
        let visc = CapillaryViscometer::ubbelohde();
        assert_eq!(visc.capillary_type, CapillaryType::Ubbelohde);
    }

    #[test]
    fn test_capillary_ostwald() {
        let visc = CapillaryViscometer::ostwald();
        assert_eq!(visc.capillary_type, CapillaryType::Ostwald);
    }

    #[test]
    fn test_capillary_kinematic_viscosity() {
        let visc = CapillaryViscometer::ubbelohde().without_hagenbach();
        let nu = visc.kinematic_viscosity(100.0);
        // K * t = 0.01 * 100 = 1.0
        assert!((nu - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_capillary_dynamic_viscosity() {
        let visc = CapillaryViscometer::ubbelohde().without_hagenbach();
        let eta = visc.dynamic_viscosity(100.0, 0.998);
        let expected = 0.01 * 100.0 * 0.998;
        assert!((eta - expected).abs() < EPSILON);
    }

    #[test]
    fn test_capillary_relative_viscosity() {
        let visc = CapillaryViscometer::ubbelohde().without_hagenbach();
        let eta_rel = visc.relative_viscosity(120.0, 100.0);
        assert!((eta_rel - 1.2).abs() < EPSILON);
    }

    #[test]
    fn test_capillary_with_hagenbach() {
        let visc = CapillaryViscometer::ubbelohde();
        assert!(visc.apply_hagenbach);
        // With Hagenbach, kinematic viscosity should differ slightly
        let nu_with = visc.kinematic_viscosity(200.0);
        let nu_without = CapillaryViscometer::ubbelohde()
            .without_hagenbach()
            .kinematic_viscosity(200.0);
        assert!(nu_with < nu_without);
    }

    // --- Intrinsic viscosity determination ---

    #[test]
    fn test_huggins_extrapolation_synthetic() {
        // Synthetic data with known [η] = 40.0 mL/g, k_H = 0.35
        let iv = 40.0;
        let kh = 0.35;
        let concentrations = vec![0.002, 0.004, 0.006, 0.008, 0.010];
        let eta_rels: Vec<f64> = concentrations
            .iter()
            .map(|&c| {
                // From Huggins: η_sp/c = [η] + k_H·[η]²·c
                // η_sp = c * ([η] + k_H·[η]²·c)
                let eta_sp = c * (iv + kh * iv * iv * c);
                1.0 + eta_sp // η_rel = 1 + η_sp
            })
            .collect();

        let det = IntrinsicViscosityDeterminer::new(&concentrations, &eta_rels);
        let result = det.huggins_extrapolation();
        assert!((result.intrinsic_viscosity - iv).abs() < 0.1);
        assert!((result.huggins_k - kh).abs() < 0.1);
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_kraemer_extrapolation_synthetic() {
        // For small η_sp, ln(η_rel) ≈ η_sp - η_sp²/2
        // So Kraemer intercept should be close to Huggins intercept
        let concentrations = vec![0.002, 0.004, 0.006, 0.008, 0.010];
        let eta_rels = vec![1.08, 1.17, 1.27, 1.38, 1.50];

        let det = IntrinsicViscosityDeterminer::new(&concentrations, &eta_rels);
        let huggins = det.huggins_extrapolation();
        let kraemer = det.kraemer_extrapolation();

        // Both should give similar intrinsic viscosity
        let diff = (huggins.intrinsic_viscosity - kraemer.intrinsic_viscosity).abs();
        assert!(diff < 5.0, "Huggins and Kraemer should agree within 5 mL/g, got diff = {}", diff);
    }

    #[test]
    fn test_combined_extrapolation() {
        let concentrations = vec![0.002, 0.004, 0.006, 0.008, 0.010];
        let eta_rels = vec![1.08, 1.17, 1.27, 1.38, 1.50];

        let det = IntrinsicViscosityDeterminer::new(&concentrations, &eta_rels);
        let (huggins, kraemer, avg) = det.combined_extrapolation();
        assert!((avg - (huggins.intrinsic_viscosity + kraemer.intrinsic_viscosity) / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_from_flow_times() {
        let concentrations = vec![0.002, 0.004, 0.006];
        let t_solutions = vec![105.0, 110.0, 116.0];
        let t_solvent = 100.0;
        let det = IntrinsicViscosityDeterminer::from_flow_times(&concentrations, &t_solutions, t_solvent);
        let result = det.huggins_extrapolation();
        assert!(result.intrinsic_viscosity > 0.0);
    }

    #[test]
    fn test_reduced_viscosities_method() {
        let concentrations = vec![0.002, 0.004];
        let eta_rels = vec![1.08, 1.17];
        let det = IntrinsicViscosityDeterminer::new(&concentrations, &eta_rels);
        let reds = det.reduced_viscosities();
        assert_eq!(reds.len(), 2);
        assert!((reds[0] - 0.08 / 0.002).abs() < EPSILON);
    }

    #[test]
    fn test_inherent_viscosities_method() {
        let concentrations = vec![0.002, 0.004];
        let eta_rels = vec![1.08, 1.17];
        let det = IntrinsicViscosityDeterminer::new(&concentrations, &eta_rels);
        let inhs = det.inherent_viscosities();
        assert_eq!(inhs.len(), 2);
        let expected = (1.08_f64).ln() / 0.002;
        assert!((inhs[0] - expected).abs() < EPSILON);
    }

    // --- Standalone Huggins/Kraemer ---

    #[test]
    fn test_standalone_huggins() {
        let c = vec![0.002, 0.004, 0.006, 0.008, 0.010];
        let red = vec![42.0, 44.0, 46.0, 48.0, 50.0];
        let (iv, _kh, r2) = huggins_extrapolation(&c, &red);
        assert!(iv > 38.0 && iv < 44.0);
        assert!(r2 > 0.99);
    }

    #[test]
    fn test_standalone_kraemer() {
        let c = vec![0.002, 0.004, 0.006, 0.008, 0.010];
        let inh = vec![38.0, 37.5, 37.0, 36.5, 36.0];
        let (iv, _kk, r2) = kraemer_extrapolation(&c, &inh);
        assert!(iv > 37.0 && iv < 40.0);
        assert!(r2 > 0.99);
    }

    // --- Mark-Houwink-Sakurada ---

    #[test]
    fn test_mark_houwink_mw_basic() {
        // [η] = K × M^a => M = ([η]/K)^(1/a)
        let k = 0.0125;
        let a = 0.78;
        let iv = 50.0; // mL/g
        let mw = mark_houwink_mw(iv, k, a);
        assert!(mw > 1e4); // Should give a reasonable MW
    }

    #[test]
    fn test_mark_houwink_roundtrip() {
        let k = 0.0125;
        let a = 0.78;
        let mw_orig = 100_000.0;
        let iv = mark_houwink_iv(mw_orig, k, a);
        let mw_back = mark_houwink_mw(iv, k, a);
        assert!((mw_back - mw_orig).abs() / mw_orig < 1e-10);
    }

    #[test]
    fn test_mark_houwink_calculator_peo() {
        let calc = MarkHouwinkCalculator::peo_water();
        let iv = calc.intrinsic_viscosity(100_000.0);
        let mw = calc.molecular_weight(iv);
        assert!((mw - 100_000.0).abs() / 100_000.0 < 1e-10);
    }

    #[test]
    fn test_mark_houwink_calculator_ps() {
        let calc = MarkHouwinkCalculator::ps_thf();
        let mw = 200_000.0;
        let iv = calc.intrinsic_viscosity(mw);
        assert!(iv > 0.0);
        let mw_back = calc.molecular_weight(iv);
        assert!((mw_back - mw).abs() / mw < 1e-10);
    }

    #[test]
    fn test_mark_houwink_fit_parameters() {
        let k_true = 0.0125;
        let a_true = 0.78;
        let mws = vec![10_000.0, 50_000.0, 100_000.0, 500_000.0, 1_000_000.0];
        let ivs: Vec<f64> = mws.iter().map(|&m: &f64| k_true * m.powf(a_true)).collect();

        let params = MarkHouwinkCalculator::fit_parameters(&ivs, &mws);
        assert!((params.k - k_true).abs() / k_true < 0.01);
        assert!((params.a - a_true).abs() < 0.01);
    }

    #[test]
    fn test_mark_houwink_higher_mw_higher_iv() {
        let calc = MarkHouwinkCalculator::peo_water();
        let iv1 = calc.intrinsic_viscosity(50_000.0);
        let iv2 = calc.intrinsic_viscosity(100_000.0);
        assert!(iv2 > iv1);
    }

    // --- Solomon-Ciuta ---

    #[test]
    fn test_solomon_ciuta_basic() {
        let eta_rel = 1.3;
        let c = 0.005;
        let iv = solomon_ciuta(eta_rel, c);
        assert!(iv > 0.0);
    }

    #[test]
    fn test_solomon_ciuta_consistent_with_huggins() {
        // For k_H ≈ 0.5, Solomon-Ciuta should match Huggins result
        let eta_rel = 1.15;
        let c = 0.005;
        let iv_sc = solomon_ciuta(eta_rel, c);
        let eta_sp = specific_viscosity(eta_rel);
        let eta_red = reduced_viscosity(eta_sp, c);
        // At finite concentration, reduced viscosity overestimates [η]
        // Solomon-Ciuta should give a better estimate
        assert!(iv_sc < eta_red);
    }

    // --- Schulz-Blaschke ---

    #[test]
    fn test_schulz_blaschke_basic() {
        let eta_rel = 1.2;
        let c = 0.005;
        let iv = schulz_blaschke(eta_rel, c, 0.28);
        assert!(iv > 0.0);
        // Should be less than reduced viscosity
        let eta_sp = specific_viscosity(eta_rel);
        let eta_red = reduced_viscosity(eta_sp, c);
        assert!(iv < eta_red);
    }

    #[test]
    fn test_schulz_blaschke_zero_k() {
        // With k_SB = 0, it should equal reduced viscosity
        let eta_rel = 1.2;
        let c = 0.005;
        let iv = schulz_blaschke(eta_rel, c, 0.0);
        let expected = specific_viscosity(eta_rel) / c;
        assert!((iv - expected).abs() < EPSILON);
    }

    // --- Concentration conversions ---

    #[test]
    fn test_g_per_dl_to_g_per_ml() {
        assert!((g_per_dl_to_g_per_ml(1.0) - 0.01).abs() < EPSILON);
    }

    #[test]
    fn test_g_per_ml_to_g_per_dl() {
        assert!((g_per_ml_to_g_per_dl(0.01) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_dl_per_g_to_ml_per_g() {
        assert!((dl_per_g_to_ml_per_g(1.0) - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_ml_per_g_to_dl_per_g() {
        assert!((ml_per_g_to_dl_per_g(100.0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_concentration_roundtrip() {
        let c = 0.005;
        let c2 = g_per_dl_to_g_per_ml(g_per_ml_to_g_per_dl(c));
        assert!((c - c2).abs() < EPSILON);
    }

    // --- Blend viscosity ---

    #[test]
    fn test_blend_viscosity_single_component() {
        let v = blend_viscosity_log_rule(&[5.0], &[1.0]);
        assert!((v - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_blend_viscosity_equal_components() {
        let v = blend_viscosity_log_rule(&[1.0, 1.0], &[0.5, 0.5]);
        assert!((v - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_blend_viscosity_between_pure() {
        let v1 = 2.0;
        let v2 = 8.0;
        let blend = blend_viscosity_log_rule(&[v1, v2], &[0.5, 0.5]);
        assert!(blend > v1 && blend < v2);
    }

    // --- Solvent properties ---

    #[test]
    fn test_solvent_water_25c() {
        let sp = SolventProperties::water_25c();
        assert!((sp.viscosity - WATER_VISCOSITY_25C).abs() < EPSILON);
        assert!((sp.density - WATER_DENSITY_25C).abs() < EPSILON);
    }

    #[test]
    fn test_solvent_water_20c() {
        let sp = SolventProperties::water_20c();
        assert!((sp.viscosity - WATER_VISCOSITY_20C).abs() < EPSILON);
    }

    #[test]
    fn test_solvent_thf_25c() {
        let sp = SolventProperties::thf_25c();
        assert!(sp.viscosity > 0.0 && sp.viscosity < 1.0);
    }

    #[test]
    fn test_solvent_custom() {
        let sp = SolventProperties::custom(1.5, 1.05, 300.0);
        assert_eq!(sp.solvent, Solvent::Custom);
        assert!((sp.viscosity - 1.5).abs() < EPSILON);
    }

    // --- Configuration ---

    #[test]
    fn test_config_ubbelohde() {
        let config = ViscometryConfig::ubbelohde_water_25c();
        assert_eq!(config.capillary_type, CapillaryType::Ubbelohde);
        assert!(config.apply_hagenbach);
    }

    #[test]
    fn test_config_ostwald() {
        let config = ViscometryConfig::ostwald_water_25c();
        assert_eq!(config.capillary_type, CapillaryType::Ostwald);
    }

    #[test]
    fn test_config_with_mark_houwink() {
        let config = ViscometryConfig::ubbelohde_water_25c()
            .with_mark_houwink(MarkHouwinkParams::peo_water_25c());
        assert!(config.mark_houwink.is_some());
    }

    // --- Full analysis ---

    #[test]
    fn test_full_analysis_basic() {
        let config = ViscometryConfig::ubbelohde_water_25c()
            .with_mark_houwink(MarkHouwinkParams::peo_water_25c());

        let concentrations = vec![0.002, 0.004, 0.006, 0.008, 0.010];
        let t_solvent = 200.0;
        let t_solutions = vec![216.0, 234.0, 254.0, 276.0, 300.0];

        let result = full_analysis(&config, &concentrations, &t_solutions, t_solvent);
        assert_eq!(result.concentrations.len(), 5);
        assert!(result.intrinsic_viscosity_avg > 0.0);
        assert!(result.molecular_weight.is_some());
        assert!(result.molecular_weight.unwrap() > 0.0);
    }

    #[test]
    fn test_full_analysis_viscosity_types() {
        let config = ViscometryConfig::ubbelohde_water_25c();
        let concentrations = vec![0.002, 0.004, 0.006];
        let t_solutions = vec![210.0, 220.0, 232.0];
        let t_solvent = 200.0;

        let result = full_analysis(&config, &concentrations, &t_solutions, t_solvent);

        // All relative viscosities should be > 1
        for &r in &result.relative_viscosities {
            assert!(r > 1.0);
        }
        // All specific viscosities should be > 0
        for &s in &result.specific_viscosities {
            assert!(s > 0.0);
        }
        // Reduced and inherent should be positive
        for &r in &result.reduced_viscosities {
            assert!(r > 0.0);
        }
        for &i in &result.inherent_viscosities {
            assert!(i > 0.0);
        }
    }

    // --- Huggins-Kraemer consistency ---

    #[test]
    fn test_huggins_kraemer_consistency_good() {
        let (sum, consistent) = huggins_kraemer_consistency(0.35, 0.15);
        assert!((sum - 0.5).abs() < EPSILON);
        assert!(consistent);
    }

    #[test]
    fn test_huggins_kraemer_consistency_bad() {
        let (_, consistent) = huggins_kraemer_consistency(0.8, 0.5);
        assert!(!consistent);
    }

    // --- Degree of polymerization ---

    #[test]
    fn test_degree_of_polymerization() {
        let dp = degree_of_polymerization(100_000.0, 44.05);
        // PEO monomer MW = 44.05
        assert!((dp - 100_000.0 / 44.05).abs() < 0.01);
    }

    // --- Solvent quality ---

    #[test]
    fn test_solvent_quality_poor() {
        assert_eq!(solvent_quality(0.3), "poor solvent (collapsed coil)");
    }

    #[test]
    fn test_solvent_quality_theta() {
        assert_eq!(solvent_quality(0.5), "theta solvent (unperturbed dimensions)");
    }

    #[test]
    fn test_solvent_quality_good() {
        assert_eq!(solvent_quality(0.7), "good solvent (expanded coil)");
    }

    #[test]
    fn test_solvent_quality_rigid() {
        assert_eq!(solvent_quality(1.0), "rigid rod / stiff chain");
    }

    // --- Flory exponent ---

    #[test]
    fn test_flory_exponent_theta() {
        let nu = flory_exponent(0.5);
        assert!((nu - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_flory_exponent_good_solvent() {
        let nu = flory_exponent(0.764);
        assert!((nu - 0.588).abs() < 0.001);
    }

    // --- Hydrodynamic radius ---

    #[test]
    fn test_hydrodynamic_radius_positive() {
        let r = hydrodynamic_radius(50.0, 100_000.0);
        assert!(r > 0.0);
    }

    #[test]
    fn test_hydrodynamic_radius_increases_with_mw() {
        let r1 = hydrodynamic_radius(50.0, 50_000.0);
        let r2 = hydrodynamic_radius(50.0, 200_000.0);
        assert!(r2 > r1);
    }

    // --- Viscosity index ---

    #[test]
    fn test_viscosity_index_computed() {
        // Just verify it returns a finite number
        let vi = viscosity_index(100.0, 15.0);
        assert!(vi.is_finite());
    }

    // --- MHS presets ---

    #[test]
    fn test_mhs_peo_water() {
        let p = MarkHouwinkParams::peo_water_25c();
        assert!((p.k - 0.0125).abs() < EPSILON);
        assert!((p.a - 0.78).abs() < EPSILON);
    }

    #[test]
    fn test_mhs_ps_thf() {
        let p = MarkHouwinkParams::ps_thf_25c();
        assert!((p.k - 0.0141).abs() < EPSILON);
        assert!((p.a - 0.70).abs() < EPSILON);
    }

    #[test]
    fn test_mhs_ps_toluene() {
        let p = MarkHouwinkParams::ps_toluene_25c();
        assert!((p.k - 0.0096).abs() < EPSILON);
        assert!((p.a - 0.73).abs() < EPSILON);
    }

    #[test]
    fn test_mhs_pva_water() {
        let p = MarkHouwinkParams::pva_water_25c();
        assert!((p.a - 0.50).abs() < EPSILON);
    }

    // --- Relative viscosity from dynamic ---

    #[test]
    fn test_relative_viscosity_from_dynamic() {
        let rel = relative_viscosity_from_dynamic(1.5, 1.0);
        assert!((rel - 1.5).abs() < EPSILON);
    }

    // --- Couette correction ---

    #[test]
    fn test_couette_correction_factor_reasonable() {
        let f = couette_correction_factor(200.0, 1e-6, 0.0003);
        // Factor should be close to 1.0 for long flow times
        assert!(f > 0.9 && f <= 1.0);
    }

    // --- CapillaryCalibration presets ---

    #[test]
    fn test_ubbelohde_calibration() {
        let cal = CapillaryCalibration::ubbelohde_typical();
        assert!(cal.constant_k > 0.0);
        assert!(cal.hagenbach_coeff > 0.0);
    }

    #[test]
    fn test_ostwald_calibration() {
        let cal = CapillaryCalibration::ostwald_typical();
        assert!(cal.constant_k > 0.0);
        assert!(cal.hagenbach_coeff > 0.0);
    }
}

//! # Vapor Pressure Osmometry (VPO) Analyzer
//!
//! Implements vapor pressure osmometry for molecular weight determination of small
//! molecules and oligomers. VPO is a thermoelectric technique that measures the vapor
//! pressure lowering of a solvent caused by the presence of a dissolved solute.
//!
//! ## Measurement Principle
//!
//! Two matched thermistors are placed in a saturated solvent vapor atmosphere. One
//! thermistor carries a drop of pure solvent, the other a drop of solution. The
//! solvent condenses onto the solution drop (lower vapor pressure), releasing heat
//! of condensation. The resulting temperature difference ΔT is proportional to the
//! solute's molality and inversely proportional to its molecular weight.
//!
//! ## Theory
//!
//! From Raoult's law for dilute solutions:
//!
//! ```text
//! ΔP/P° = x_solute ≈ n_solute / n_solvent = (m · M_solvent) / 1000
//! ```
//!
//! where m is molality. Combined with the Clausius-Clapeyron equation, the
//! steady-state temperature difference is:
//!
//! ```text
//! ΔT = K_cal · c / M_n
//! ```
//!
//! where K_cal is the instrument calibration constant (determined using a standard
//! of known molecular weight), c is concentration (g/L), and M_n is number-average
//! molecular weight.
//!
//! ## Virial Expansion
//!
//! For non-ideal solutions, ΔT/c is expanded as:
//!
//! ```text
//! ΔT/c = K_cal · (1/M_n + A₂·c + A₃·c² + ...)
//! ```
//!
//! where A₂ is the second virial coefficient characterizing solute-solvent
//! interactions.
//!
//! ## Flory-Huggins Analysis
//!
//! For polymer solutions, the Flory-Huggins interaction parameter χ can be
//! determined from VPO solvent activity measurements:
//!
//! ```text
//! ln(a₁) = ln(1-φ₂) + (1 - 1/r)·φ₂ + χ·φ₂²
//! ```
//!
//! where a₁ is solvent activity, φ₂ is polymer volume fraction, and r is the
//! degree of polymerization.
//!
//! ## Typical Applications
//!
//! - Number-average molecular weight (Mn) of polymers (50-20,000 g/mol)
//! - Second virial coefficient (A₂) determination
//! - Solvent-polymer interaction parameter (χ)
//! - Association/aggregation studies
//! - Purity assessment of small molecules
//!
//! ## References
//!
//! - Bonnar, Dimbat, Stross, "Number Average Molecular Weights", 1958
//! - Flory, "Principles of Polymer Chemistry", Cornell University Press, 1953
//! - Kamide, Dobashi, "Physical Chemistry of Polymer Solutions", Elsevier, 2000

use std::f64::consts::PI;

/// Universal gas constant in J/(mol·K)
const R_GAS: f64 = 8.314;

/// Benzil molecular weight (common VPO calibration standard) in g/mol
const BENZIL_MW: f64 = 210.23;

/// Sucrose octaacetate molecular weight (another VPO standard) in g/mol
const SUCROSE_OCTAACETATE_MW: f64 = 678.60;

/// VPO typical molecular weight range lower bound in g/mol
const VPO_MW_MIN: f64 = 50.0;

/// VPO typical molecular weight range upper bound in g/mol
const VPO_MW_MAX: f64 = 20000.0;

// ────────────────────────────── Solvent Database ──────────────────────────────

/// Properties of a solvent used in VPO measurements.
#[derive(Debug, Clone)]
pub struct SolventProperties {
    /// Solvent name
    pub name: &'static str,
    /// Molar mass in g/mol
    pub mw: f64,
    /// Boiling point in °C
    pub boiling_point_c: f64,
    /// Enthalpy of vaporization in J/mol
    pub delta_h_vap: f64,
    /// Ebullioscopic constant Kb in K·kg/mol
    pub kb: f64,
    /// Density at 25 °C in g/mL
    pub density: f64,
    /// Molar volume in mL/mol (calculated from mw/density)
    pub molar_volume: f64,
}

/// Database of common VPO solvents.
pub struct SolventDatabase;

impl SolventDatabase {
    /// Toluene (methylbenzene) — most common VPO solvent for organic polymers.
    pub fn toluene() -> SolventProperties {
        SolventProperties {
            name: "Toluene",
            mw: 92.14,
            boiling_point_c: 110.6,
            delta_h_vap: 33180.0,
            kb: 3.40,
            density: 0.867,
            molar_volume: 92.14 / 0.867,
        }
    }

    /// Chloroform (trichloromethane).
    pub fn chloroform() -> SolventProperties {
        SolventProperties {
            name: "Chloroform",
            mw: 119.38,
            boiling_point_c: 61.2,
            delta_h_vap: 29240.0,
            kb: 3.63,
            density: 1.489,
            molar_volume: 119.38 / 1.489,
        }
    }

    /// Tetrahydrofuran (THF).
    pub fn thf() -> SolventProperties {
        SolventProperties {
            name: "THF",
            mw: 72.11,
            boiling_point_c: 66.0,
            delta_h_vap: 31800.0,
            kb: 1.72,
            density: 0.889,
            molar_volume: 72.11 / 0.889,
        }
    }

    /// N,N-Dimethylformamide (DMF).
    pub fn dmf() -> SolventProperties {
        SolventProperties {
            name: "DMF",
            mw: 73.09,
            boiling_point_c: 153.0,
            delta_h_vap: 46680.0,
            kb: 2.08,
            density: 0.944,
            molar_volume: 73.09 / 0.944,
        }
    }

    /// Water.
    pub fn water() -> SolventProperties {
        SolventProperties {
            name: "Water",
            mw: 18.015,
            boiling_point_c: 100.0,
            delta_h_vap: 40660.0,
            kb: 0.512,
            density: 1.000,
            molar_volume: 18.015,
        }
    }

    /// Benzene.
    pub fn benzene() -> SolventProperties {
        SolventProperties {
            name: "Benzene",
            mw: 78.11,
            boiling_point_c: 80.1,
            delta_h_vap: 30720.0,
            kb: 2.53,
            density: 0.879,
            molar_volume: 78.11 / 0.879,
        }
    }

    /// Acetone.
    pub fn acetone() -> SolventProperties {
        SolventProperties {
            name: "Acetone",
            mw: 58.08,
            boiling_point_c: 56.05,
            delta_h_vap: 31300.0,
            kb: 1.72,
            density: 0.784,
            molar_volume: 58.08 / 0.784,
        }
    }

    /// Return all available solvents.
    pub fn all() -> Vec<SolventProperties> {
        vec![
            Self::toluene(),
            Self::chloroform(),
            Self::thf(),
            Self::dmf(),
            Self::water(),
            Self::benzene(),
            Self::acetone(),
        ]
    }

    /// Lookup a solvent by name (case-insensitive).
    pub fn lookup(name: &str) -> Option<SolventProperties> {
        let lower = name.to_lowercase();
        Self::all()
            .into_iter()
            .find(|s| s.name.to_lowercase() == lower)
    }
}

// ────────────────────────────── Calibration Standards ─────────────────────────

/// Known calibration standards for VPO.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationStandard {
    /// Benzil (diphenyl-1,2-ethanedione), MW = 210.23 g/mol
    Benzil,
    /// Sucrose octaacetate, MW = 678.60 g/mol
    SucroseOctaacetate,
    /// Custom standard with known molecular weight
    Custom(f64),
}

impl CalibrationStandard {
    /// Return the molecular weight of the standard in g/mol.
    pub fn molecular_weight(&self) -> f64 {
        match self {
            CalibrationStandard::Benzil => BENZIL_MW,
            CalibrationStandard::SucroseOctaacetate => SUCROSE_OCTAACETATE_MW,
            CalibrationStandard::Custom(mw) => *mw,
        }
    }
}

// ────────────────────────────── Configuration ────────────────────────────────

/// Configuration for a VPO measurement session.
#[derive(Debug, Clone)]
pub struct VpoConfig {
    /// Solvent used in the measurement
    pub solvent: SolventProperties,
    /// Measurement temperature in °C
    pub temperature_c: f64,
    /// Calibration standard used
    pub calibration_standard: CalibrationStandard,
    /// Thermistor sensitivity in mV/°C
    pub thermistor_sensitivity: f64,
    /// Wheatstone bridge excitation voltage in V
    pub bridge_excitation_v: f64,
    /// Number of replicate measurements per concentration
    pub replicates: usize,
}

impl VpoConfig {
    /// Create a new VPO configuration with default thermistor parameters.
    pub fn new(solvent: SolventProperties, temperature_c: f64) -> Self {
        Self {
            solvent,
            temperature_c,
            calibration_standard: CalibrationStandard::Benzil,
            thermistor_sensitivity: 100.0, // mV/°C
            bridge_excitation_v: 5.0,
            replicates: 3,
        }
    }

    /// Set the calibration standard.
    pub fn with_standard(mut self, standard: CalibrationStandard) -> Self {
        self.calibration_standard = standard;
        self
    }

    /// Set the thermistor sensitivity.
    pub fn with_sensitivity(mut self, sensitivity_mv_per_c: f64) -> Self {
        self.thermistor_sensitivity = sensitivity_mv_per_c;
        self
    }

    /// Set the bridge excitation voltage.
    pub fn with_bridge_voltage(mut self, voltage_v: f64) -> Self {
        self.bridge_excitation_v = voltage_v;
        self
    }

    /// Set the number of replicates.
    pub fn with_replicates(mut self, n: usize) -> Self {
        self.replicates = n;
        self
    }

    /// Measurement temperature in Kelvin.
    pub fn temperature_k(&self) -> f64 {
        self.temperature_c + 273.15
    }

    /// Calculate the theoretical ebullioscopic constant from thermodynamic data.
    /// Kb = R·T²·M_solvent / (1000·ΔH_vap)
    pub fn theoretical_kb(&self) -> f64 {
        let t = self.temperature_k();
        R_GAS * t * t * self.solvent.mw / (1000.0 * self.solvent.delta_h_vap)
    }
}

// ────────────────────────────── Measurement Data ─────────────────────────────

/// A single VPO measurement data point.
#[derive(Debug, Clone)]
pub struct VpoMeasurement {
    /// Solute concentration in g/L
    pub concentration: f64,
    /// Measured temperature difference ΔT in °C (or mK if specified)
    pub delta_t: f64,
    /// Bridge voltage difference ΔV in mV (if available)
    pub delta_v: Option<f64>,
    /// Measurement time in seconds (for equilibration tracking)
    pub time_s: Option<f64>,
}

impl VpoMeasurement {
    /// Create a new measurement with concentration and ΔT.
    pub fn new(concentration: f64, delta_t: f64) -> Self {
        Self {
            concentration,
            delta_t,
            delta_v: None,
            time_s: None,
        }
    }

    /// Create a measurement from bridge voltage ΔV and thermistor sensitivity.
    pub fn from_bridge_voltage(
        concentration: f64,
        delta_v_mv: f64,
        sensitivity_mv_per_c: f64,
    ) -> Self {
        Self {
            concentration,
            delta_t: delta_v_mv / sensitivity_mv_per_c,
            delta_v: Some(delta_v_mv),
            time_s: None,
        }
    }

    /// Set the measurement time.
    pub fn with_time(mut self, time_s: f64) -> Self {
        self.time_s = Some(time_s);
        self
    }

    /// Calculate ΔT/c (reduced signal).
    /// Returns None if concentration is zero.
    pub fn reduced_signal(&self) -> Option<f64> {
        if self.concentration.abs() < 1e-15 {
            None
        } else {
            Some(self.delta_t / self.concentration)
        }
    }
}

// ────────────────────────────── Calibration Curve ─────────────────────────────

/// Calibration result from VPO measurements of a known standard.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    /// Instrument calibration constant K_cal in (°C·L/g)·(g/mol) = °C·L·mol⁻¹
    pub k_cal: f64,
    /// Slope of ΔT vs. concentration (°C·L/g)
    pub slope: f64,
    /// Intercept of linear fit (should be near zero)
    pub intercept: f64,
    /// R² of the calibration fit
    pub r_squared: f64,
    /// Standard molecular weight used for calibration
    pub standard_mw: f64,
    /// Number of calibration points used
    pub n_points: usize,
}

impl CalibrationCurve {
    /// Build a calibration curve from measurements of a known standard.
    ///
    /// `measurements`: VPO data for the calibration standard at various concentrations.
    /// `standard`: the calibration standard used.
    ///
    /// K_cal = slope × M_standard
    ///
    /// where slope = d(ΔT)/d(c) from linear regression of ΔT vs c.
    pub fn from_measurements(
        measurements: &[VpoMeasurement],
        standard: CalibrationStandard,
    ) -> Option<Self> {
        if measurements.len() < 2 {
            return None;
        }

        let conc: Vec<f64> = measurements.iter().map(|m| m.concentration).collect();
        let dt: Vec<f64> = measurements.iter().map(|m| m.delta_t).collect();

        let (slope, intercept, r_squared) = linear_regression(&conc, &dt)?;

        let mw = standard.molecular_weight();
        let k_cal = slope * mw;

        Some(CalibrationCurve {
            k_cal,
            slope,
            intercept,
            r_squared,
            standard_mw: mw,
            n_points: measurements.len(),
        })
    }

    /// Build calibration from ΔT/c extrapolation (more accurate for non-ideal).
    ///
    /// Plots ΔT/c vs c and extrapolates to c→0. The intercept gives K_cal/M_std.
    pub fn from_extrapolation(
        measurements: &[VpoMeasurement],
        standard: CalibrationStandard,
    ) -> Option<Self> {
        if measurements.len() < 2 {
            return None;
        }

        let conc: Vec<f64> = measurements.iter().map(|m| m.concentration).collect();
        let dt_over_c: Vec<f64> = measurements
            .iter()
            .filter_map(|m| m.reduced_signal())
            .collect();

        if dt_over_c.len() < 2 {
            return None;
        }

        let conc_filtered: Vec<f64> = measurements
            .iter()
            .filter(|m| m.concentration.abs() > 1e-15)
            .map(|m| m.concentration)
            .collect();

        let (slope_virial, intercept_val, r_sq) =
            linear_regression(&conc_filtered, &dt_over_c)?;

        let mw = standard.molecular_weight();
        let k_cal = intercept_val * mw;

        Some(CalibrationCurve {
            k_cal,
            slope: intercept_val, // This is the ΔT/c at c→0
            intercept: slope_virial, // slope of ΔT/c vs c (virial term)
            r_squared: r_sq,
            standard_mw: mw,
            n_points: conc_filtered.len(),
        })
    }
}

// ────────────────────────────── Molecular Weight Calculator ──────────────────

/// Calculator for number-average molecular weight from VPO data.
#[derive(Debug, Clone)]
pub struct MolecularWeightCalculator {
    /// Calibration constant K_cal
    pub k_cal: f64,
}

impl MolecularWeightCalculator {
    /// Create a calculator from a calibration curve.
    pub fn new(k_cal: f64) -> Self {
        Self { k_cal }
    }

    /// Create from a calibration curve result.
    pub fn from_calibration(cal: &CalibrationCurve) -> Self {
        Self { k_cal: cal.k_cal }
    }

    /// Calculate Mn from a single ΔT/c measurement (assumes ideal dilute solution).
    ///
    /// Mn = K_cal / (ΔT/c)
    pub fn mn_from_single(&self, measurement: &VpoMeasurement) -> Option<f64> {
        let reduced = measurement.reduced_signal()?;
        if reduced.abs() < 1e-15 {
            return None;
        }
        Some(self.k_cal / reduced)
    }

    /// Calculate Mn from a series of measurements by extrapolation to c→0.
    ///
    /// Plots ΔT/c vs c, extrapolates to c=0: intercept = K_cal/Mn
    pub fn mn_from_series(&self, measurements: &[VpoMeasurement]) -> Option<MwResult> {
        if measurements.len() < 2 {
            return None;
        }

        let mut conc = Vec::new();
        let mut dt_over_c = Vec::new();

        for m in measurements {
            if let Some(rs) = m.reduced_signal() {
                conc.push(m.concentration);
                dt_over_c.push(rs);
            }
        }

        if conc.len() < 2 {
            return None;
        }

        let (slope, intercept, r_squared) = linear_regression(&conc, &dt_over_c)?;

        if intercept.abs() < 1e-15 {
            return None;
        }

        let mn = self.k_cal / intercept;

        Some(MwResult {
            mn,
            intercept,
            slope,
            r_squared,
            n_points: conc.len(),
            in_range: mn >= VPO_MW_MIN && mn <= VPO_MW_MAX,
        })
    }

    /// Calculate Mn from the intercept of ΔT/c vs c directly.
    pub fn mn_from_intercept(&self, intercept: f64) -> Option<f64> {
        if intercept.abs() < 1e-15 {
            return None;
        }
        Some(self.k_cal / intercept)
    }
}

/// Result of molecular weight determination.
#[derive(Debug, Clone)]
pub struct MwResult {
    /// Number-average molecular weight in g/mol
    pub mn: f64,
    /// Intercept of ΔT/c vs c (= K_cal/Mn)
    pub intercept: f64,
    /// Slope of ΔT/c vs c (related to A₂)
    pub slope: f64,
    /// R² of the linear fit
    pub r_squared: f64,
    /// Number of data points used
    pub n_points: usize,
    /// Whether Mn falls within typical VPO range (50-20,000 g/mol)
    pub in_range: bool,
}

// ────────────────────────────── Virial Expansion ─────────────────────────────

/// Second virial coefficient analysis from VPO data.
///
/// The virial expansion of osmotic pressure / VPO signal:
///
/// ```text
/// ΔT/c = K_cal · (1/Mn + A₂·c + A₃·c² + ...)
/// ```
#[derive(Debug, Clone)]
pub struct VirialExpansion {
    /// Calibration constant
    pub k_cal: f64,
}

impl VirialExpansion {
    /// Create a new virial expansion analyzer.
    pub fn new(k_cal: f64) -> Self {
        Self { k_cal }
    }

    /// Determine Mn and A₂ from a concentration series.
    ///
    /// Linear fit of ΔT/c vs c:
    /// - intercept = K_cal / Mn
    /// - slope = K_cal · A₂
    pub fn analyze(&self, measurements: &[VpoMeasurement]) -> Option<VirialResult> {
        if measurements.len() < 3 {
            return None;
        }

        let mut conc = Vec::new();
        let mut dt_over_c = Vec::new();

        for m in measurements {
            if let Some(rs) = m.reduced_signal() {
                conc.push(m.concentration);
                dt_over_c.push(rs);
            }
        }

        if conc.len() < 3 {
            return None;
        }

        let (slope, intercept, r_squared) = linear_regression(&conc, &dt_over_c)?;

        if intercept.abs() < 1e-15 {
            return None;
        }

        let mn = self.k_cal / intercept;
        let a2 = slope / self.k_cal;

        Some(VirialResult {
            mn,
            a2,
            r_squared,
            n_points: conc.len(),
            theta_condition: a2.abs() < 1e-6,
        })
    }

    /// Perform quadratic virial fit to extract A₂ and A₃.
    ///
    /// ΔT/c = b₀ + b₁·c + b₂·c²
    /// where b₀ = K_cal/Mn, b₁ = K_cal·A₂, b₂ = K_cal·A₃
    pub fn analyze_quadratic(&self, measurements: &[VpoMeasurement]) -> Option<VirialResultQuadratic> {
        if measurements.len() < 4 {
            return None;
        }

        let mut conc = Vec::new();
        let mut dt_over_c = Vec::new();

        for m in measurements {
            if let Some(rs) = m.reduced_signal() {
                conc.push(m.concentration);
                dt_over_c.push(rs);
            }
        }

        if conc.len() < 4 {
            return None;
        }

        let (b0, b1, b2, r_sq) = quadratic_regression(&conc, &dt_over_c)?;

        if b0.abs() < 1e-15 {
            return None;
        }

        let mn = self.k_cal / b0;
        let a2 = b1 / self.k_cal;
        let a3 = b2 / self.k_cal;

        Some(VirialResultQuadratic {
            mn,
            a2,
            a3,
            r_squared: r_sq,
            n_points: conc.len(),
        })
    }

    /// Calculate reduced osmotic pressure Π/(cRT) from VPO data.
    /// Π/(cRT) = 1/Mn + A₂·c + A₃·c² + ...
    pub fn reduced_osmotic_pressure(&self, mn: f64, a2: f64, concentration: f64) -> f64 {
        1.0 / mn + a2 * concentration
    }

    /// Predict ΔT for a given concentration using virial parameters.
    pub fn predict_delta_t(&self, mn: f64, a2: f64, concentration: f64) -> f64 {
        self.k_cal * concentration * (1.0 / mn + a2 * concentration)
    }
}

/// Result of linear virial analysis.
#[derive(Debug, Clone)]
pub struct VirialResult {
    /// Number-average molecular weight in g/mol
    pub mn: f64,
    /// Second virial coefficient A₂ in mol·mL/g²
    pub a2: f64,
    /// R² of the linear fit
    pub r_squared: f64,
    /// Number of data points
    pub n_points: usize,
    /// Whether A₂ ≈ 0 (theta condition / theta solvent)
    pub theta_condition: bool,
}

/// Result of quadratic virial analysis.
#[derive(Debug, Clone)]
pub struct VirialResultQuadratic {
    /// Number-average molecular weight in g/mol
    pub mn: f64,
    /// Second virial coefficient A₂
    pub a2: f64,
    /// Third virial coefficient A₃
    pub a3: f64,
    /// R² of the quadratic fit
    pub r_squared: f64,
    /// Number of data points
    pub n_points: usize,
}

// ────────────────────────────── Raoult's Law ─────────────────────────────────

/// Raoult's law calculations for ideal solutions.
pub struct RaoultsLaw;

impl RaoultsLaw {
    /// Calculate vapor pressure lowering ΔP from Raoult's law.
    ///
    /// ΔP = P° × x_solute
    ///
    /// where P° is pure solvent vapor pressure and x_solute is solute mole fraction.
    pub fn delta_p(p_pure: f64, mole_fraction_solute: f64) -> f64 {
        p_pure * mole_fraction_solute
    }

    /// Calculate solution vapor pressure from Raoult's law.
    ///
    /// P = P° × (1 - x_solute) = P° × x_solvent
    pub fn solution_vapor_pressure(p_pure: f64, mole_fraction_solute: f64) -> f64 {
        p_pure * (1.0 - mole_fraction_solute)
    }

    /// Calculate relative vapor pressure lowering.
    ///
    /// ΔP/P° = x_solute
    pub fn relative_lowering(mole_fraction_solute: f64) -> f64 {
        mole_fraction_solute
    }

    /// Calculate boiling point elevation.
    ///
    /// ΔTb = Kb × m
    ///
    /// where Kb is the ebullioscopic constant and m is molality.
    pub fn boiling_point_elevation(kb: f64, molality: f64) -> f64 {
        kb * molality
    }

    /// Calculate expected ΔT for a given solute in a solvent.
    ///
    /// ΔT = Kb × (mass_solute / mw_solute) / (mass_solvent_kg)
    pub fn expected_delta_t(
        kb: f64,
        mass_solute_g: f64,
        mw_solute: f64,
        mass_solvent_g: f64,
    ) -> f64 {
        let molality = (mass_solute_g / mw_solute) / (mass_solvent_g / 1000.0);
        kb * molality
    }
}

// ────────────────────────────── Flory-Huggins ────────────────────────────────

/// Flory-Huggins interaction parameter analysis from VPO solvent activity data.
///
/// The Flory-Huggins equation for solvent activity:
///
/// ```text
/// ln(a₁) = ln(1-φ₂) + (1 - 1/r)·φ₂ + χ·φ₂²
/// ```
pub struct FloryHugginsAnalyzer;

impl FloryHugginsAnalyzer {
    /// Calculate χ from a single solvent activity measurement.
    ///
    /// χ = [ln(a₁) - ln(1-φ₂) - (1 - 1/r)·φ₂] / φ₂²
    pub fn chi_from_activity(
        activity: f64,
        volume_fraction_polymer: f64,
        degree_of_polymerization: f64,
    ) -> Option<f64> {
        if volume_fraction_polymer.abs() < 1e-15 {
            return None;
        }
        if activity <= 0.0 || activity > 1.0 {
            return None;
        }

        let phi = volume_fraction_polymer;
        let r = degree_of_polymerization;

        let ln_a = activity.ln();
        let ln_1_minus_phi = (1.0 - phi).ln();
        let combinatorial = (1.0 - 1.0 / r) * phi;

        let chi = (ln_a - ln_1_minus_phi - combinatorial) / (phi * phi);

        Some(chi)
    }

    /// Calculate solvent activity from Flory-Huggins equation.
    ///
    /// a₁ = exp[ln(1-φ₂) + (1 - 1/r)·φ₂ + χ·φ₂²]
    pub fn activity_from_chi(
        chi: f64,
        volume_fraction_polymer: f64,
        degree_of_polymerization: f64,
    ) -> f64 {
        let phi = volume_fraction_polymer;
        let r = degree_of_polymerization;

        let exponent = (1.0 - phi).ln() + (1.0 - 1.0 / r) * phi + chi * phi * phi;
        exponent.exp()
    }

    /// Determine χ from multiple VPO measurements at different concentrations.
    ///
    /// Uses linear regression of [ln(a₁) - ln(1-φ₂) - (1-1/r)·φ₂] vs φ₂²
    /// where the slope gives χ.
    pub fn chi_from_series(
        activities: &[f64],
        volume_fractions: &[f64],
        degree_of_polymerization: f64,
    ) -> Option<FloryHugginsResult> {
        if activities.len() != volume_fractions.len() || activities.len() < 2 {
            return None;
        }

        let r = degree_of_polymerization;
        let mut x_vals = Vec::new();
        let mut y_vals = Vec::new();

        for (a, phi) in activities.iter().zip(volume_fractions.iter()) {
            if *a <= 0.0 || *a > 1.0 || *phi <= 0.0 || *phi >= 1.0 {
                continue;
            }
            let ln_a = a.ln();
            let ln_1_minus_phi = (1.0 - phi).ln();
            let combinatorial = (1.0 - 1.0 / r) * *phi;

            x_vals.push(phi * phi);
            y_vals.push(ln_a - ln_1_minus_phi - combinatorial);
        }

        if x_vals.len() < 2 {
            return None;
        }

        let (slope, intercept, r_squared) = linear_regression(&x_vals, &y_vals)?;

        Some(FloryHugginsResult {
            chi: slope,
            intercept,
            r_squared,
            n_points: x_vals.len(),
            good_solvent: slope < 0.5,
        })
    }

    /// Calculate the osmotic pressure from Flory-Huggins theory.
    ///
    /// Π = -(RT/V₁) · [ln(1-φ₂) + (1 - 1/r)·φ₂ + χ·φ₂²]
    pub fn osmotic_pressure(
        temperature_k: f64,
        molar_volume_solvent: f64,  // in mL/mol
        chi: f64,
        volume_fraction_polymer: f64,
        degree_of_polymerization: f64,
    ) -> f64 {
        let phi = volume_fraction_polymer;
        let r = degree_of_polymerization;
        let v1 = molar_volume_solvent * 1e-6; // convert mL to m³/mol (approximate for units)

        let bracket = (1.0 - phi).ln() + (1.0 - 1.0 / r) * phi + chi * phi * phi;
        -(R_GAS * temperature_k / v1) * bracket
    }

    /// Calculate the critical chi parameter for phase separation.
    ///
    /// χ_c = 0.5 × (1 + 1/√r)²
    pub fn critical_chi(degree_of_polymerization: f64) -> f64 {
        let r = degree_of_polymerization;
        let term = 1.0 + 1.0 / r.sqrt();
        0.5 * term * term
    }

    /// Calculate the critical volume fraction for phase separation.
    ///
    /// φ_c = 1 / (1 + √r)
    pub fn critical_volume_fraction(degree_of_polymerization: f64) -> f64 {
        1.0 / (1.0 + degree_of_polymerization.sqrt())
    }
}

/// Result of Flory-Huggins analysis.
#[derive(Debug, Clone)]
pub struct FloryHugginsResult {
    /// Flory-Huggins interaction parameter χ
    pub chi: f64,
    /// Intercept of the linear fit (should be near zero)
    pub intercept: f64,
    /// R² of the fit
    pub r_squared: f64,
    /// Number of data points
    pub n_points: usize,
    /// Whether χ < 0.5 (good solvent condition)
    pub good_solvent: bool,
}

// ────────────────────────────── Thermodynamic Functions ──────────────────────

/// Calculate vapor pressure at temperature T₂ using the Clausius-Clapeyron equation.
///
/// ```text
/// ln(P₂/P₁) = -ΔH_vap/R × (1/T₂ - 1/T₁)
/// ```
///
/// Returns P₂ in the same units as P₁.
pub fn clausius_clapeyron(delta_h_vap: f64, t1_k: f64, p1: f64, t2_k: f64) -> f64 {
    let exponent = -delta_h_vap / R_GAS * (1.0 / t2_k - 1.0 / t1_k);
    p1 * exponent.exp()
}

/// Calculate mole fraction of solute.
///
/// ```text
/// x_solute = n_solute / (n_solute + n_solvent)
///          = (m_solute/MW_solute) / (m_solute/MW_solute + m_solvent/MW_solvent)
/// ```
pub fn mole_fraction(
    mass_solute: f64,
    mw_solute: f64,
    mass_solvent: f64,
    mw_solvent: f64,
) -> f64 {
    let n_solute = mass_solute / mw_solute;
    let n_solvent = mass_solvent / mw_solvent;
    n_solute / (n_solute + n_solvent)
}

/// Calculate molality (moles of solute per kg of solvent).
///
/// ```text
/// m = (mass_solute / MW_solute) / (mass_solvent / 1000)
/// ```
pub fn molality(mass_solute_g: f64, mw_solute: f64, mass_solvent_g: f64) -> f64 {
    (mass_solute_g / mw_solute) / (mass_solvent_g / 1000.0)
}

/// Calculate molarity (moles of solute per liter of solution).
///
/// Approximate for dilute solutions: c ≈ (mass_solute / MW_solute) / V_solution_L
pub fn molarity(mass_solute_g: f64, mw_solute: f64, volume_solution_l: f64) -> f64 {
    (mass_solute_g / mw_solute) / volume_solution_l
}

/// Calculate VPO signal (ΔT) to molecular weight.
///
/// ```text
/// Mn = K_cal / (ΔT/c) at c→0
/// ```
pub fn vpo_signal_to_mw(delta_t_over_c_intercept: f64, k_cal: f64) -> Option<f64> {
    if delta_t_over_c_intercept.abs() < 1e-15 {
        return None;
    }
    Some(k_cal / delta_t_over_c_intercept)
}

/// Raoult's law vapor pressure lowering.
///
/// ΔP = P° × x_solute
pub fn raoults_law_delta_p(p_pure: f64, mole_fraction_solute: f64) -> f64 {
    p_pure * mole_fraction_solute
}

/// Calculate the Flory-Huggins interaction parameter χ from activity data.
///
/// χ = [ln(a₁) - ln(1-φ₂) - (1 - 1/r)·φ₂] / φ₂²
pub fn flory_huggins_chi(
    activity: f64,
    volume_fraction: f64,
    degree_of_polymerization: f64,
) -> Option<f64> {
    FloryHugginsAnalyzer::chi_from_activity(activity, volume_fraction, degree_of_polymerization)
}

/// Calculate activity coefficient from osmotic coefficient.
///
/// For electrolyte solutions: ln(γ) = φ - 1 + integral...
/// Simplified for non-electrolyte VPO: γ = a / x
///
/// For molality-based: a₁ = γ₁ × x₁, so γ₁ = a₁ / x₁
pub fn activity_coefficient(activity: f64, mole_fraction_solvent: f64) -> Option<f64> {
    if mole_fraction_solvent.abs() < 1e-15 {
        return None;
    }
    Some(activity / mole_fraction_solvent)
}

/// Calculate the osmotic coefficient from activity.
///
/// φ = -ln(a₁) / (ν × m × M₁/1000)
///
/// For non-electrolytes (ν=1): φ = -1000 × ln(a₁) / (m × M₁)
pub fn osmotic_coefficient(
    activity: f64,
    molality_val: f64,
    mw_solvent: f64,
) -> Option<f64> {
    if molality_val.abs() < 1e-15 || activity <= 0.0 {
        return None;
    }
    Some(-1000.0 * activity.ln() / (molality_val * mw_solvent))
}

/// Calculate volume fraction from mass fraction and densities.
///
/// φ₂ = (w₂/ρ₂) / (w₂/ρ₂ + (1-w₂)/ρ₁)
pub fn volume_fraction(
    mass_fraction_polymer: f64,
    density_polymer: f64,
    density_solvent: f64,
) -> f64 {
    let w = mass_fraction_polymer;
    let v2 = w / density_polymer;
    let v1 = (1.0 - w) / density_solvent;
    v2 / (v2 + v1)
}

/// Convert concentration in g/L to molality.
///
/// Approximate for dilute solutions:
/// m ≈ c / (MW × ρ_solvent) × 1000
pub fn concentration_to_molality(
    conc_g_per_l: f64,
    mw_solute: f64,
    density_solvent: f64,
) -> f64 {
    // mass of solvent in 1 L: approximately density_solvent × 1000 g (since ρ in g/mL)
    let mass_solvent_kg = density_solvent; // g/mL × 1000 mL / 1000 g/kg = g/mL
    (conc_g_per_l / mw_solute) / mass_solvent_kg
}

/// Calculate the theoretical ebullioscopic constant from thermodynamic data.
///
/// Kb = R × Tb² × M_solvent / (1000 × ΔH_vap)
pub fn ebullioscopic_constant(tb_k: f64, mw_solvent: f64, delta_h_vap: f64) -> f64 {
    R_GAS * tb_k * tb_k * mw_solvent / (1000.0 * delta_h_vap)
}

/// Antoine equation for vapor pressure estimation.
///
/// log₁₀(P) = A - B/(C + T)
///
/// where P is in mmHg and T is in °C.
pub fn antoine_pressure(a: f64, b: f64, c: f64, temperature_c: f64) -> f64 {
    let log_p = a - b / (c + temperature_c);
    10.0_f64.powf(log_p)
}

/// Calculate number of moles from mass and molecular weight.
pub fn moles(mass_g: f64, mw: f64) -> f64 {
    mass_g / mw
}

/// Convert Celsius to Kelvin.
pub fn celsius_to_kelvin(t_c: f64) -> f64 {
    t_c + 273.15
}

/// Convert Kelvin to Celsius.
pub fn kelvin_to_celsius(t_k: f64) -> f64 {
    t_k - 273.15
}

// ────────────────────────────── Wheatstone Bridge Model ──────────────────────

/// Model of the Wheatstone bridge thermistor circuit used in VPO.
#[derive(Debug, Clone)]
pub struct WheatstoneThermistorBridge {
    /// Reference resistance in Ohms (thermistor at measurement temperature)
    pub r_ref: f64,
    /// Thermistor B-parameter (material constant) in Kelvin
    pub b_param: f64,
    /// Reference temperature in K (usually measurement temperature)
    pub t_ref_k: f64,
    /// Bridge excitation voltage in V
    pub v_excitation: f64,
    /// Bridge arm resistances (R1, R2, R3) in Ohms (R4 is the thermistor)
    pub r_arms: [f64; 3],
}

impl WheatstoneThermistorBridge {
    /// Create a balanced bridge with all arms equal to r_ref.
    pub fn balanced(r_ref: f64, b_param: f64, t_ref_k: f64, v_excitation: f64) -> Self {
        Self {
            r_ref,
            b_param,
            t_ref_k,
            v_excitation,
            r_arms: [r_ref, r_ref, r_ref],
        }
    }

    /// Calculate thermistor resistance at temperature T using the B-parameter model.
    ///
    /// R(T) = R_ref × exp[B × (1/T - 1/T_ref)]
    pub fn thermistor_resistance(&self, temperature_k: f64) -> f64 {
        self.r_ref * (self.b_param * (1.0 / temperature_k - 1.0 / self.t_ref_k)).exp()
    }

    /// Calculate bridge output voltage for a given temperature difference.
    ///
    /// ΔV = V_exc × [R3/(R3+R4) - R2/(R1+R2)]
    ///
    /// For a balanced bridge with ΔT → ΔR4:
    /// ΔV ≈ V_exc × ΔR / (4R) for small imbalance
    pub fn bridge_output(&self, delta_t: f64) -> f64 {
        let t2 = self.t_ref_k + delta_t;
        let r4 = self.thermistor_resistance(t2);
        let r3 = self.r_arms[2];
        let r2 = self.r_arms[1];
        let r1 = self.r_arms[0];

        self.v_excitation * (r3 / (r3 + r4) - r2 / (r1 + r2))
    }

    /// Sensitivity: dV/dT at the reference temperature.
    pub fn sensitivity(&self) -> f64 {
        let dt = 0.001; // 1 mK perturbation
        let v_plus = self.bridge_output(dt);
        let v_minus = self.bridge_output(-dt);
        (v_plus - v_minus) / (2.0 * dt)
    }
}

// ────────────────────────────── Equilibration Analysis ───────────────────────

/// Analyzer for VPO equilibration curves (signal vs time).
pub struct EquilibrationAnalyzer;

impl EquilibrationAnalyzer {
    /// Fit an exponential approach to steady state.
    ///
    /// ΔT(t) = ΔT_∞ × (1 - exp(-t/τ))
    ///
    /// Returns (delta_t_infinity, time_constant_tau).
    pub fn fit_exponential(times: &[f64], signals: &[f64]) -> Option<(f64, f64)> {
        if times.len() < 3 || signals.len() < 3 || times.len() != signals.len() {
            return None;
        }

        // Estimate ΔT_∞ as the last measured value (or average of last few)
        let n = signals.len();
        let dt_inf = if n >= 3 {
            (signals[n - 3] + signals[n - 2] + signals[n - 1]) / 3.0
        } else {
            signals[n - 1]
        };

        if dt_inf.abs() < 1e-15 {
            return None;
        }

        // Linearize: ln(1 - ΔT/ΔT_∞) = -t/τ
        // Fit slope to get -1/τ
        let mut x_vals = Vec::new();
        let mut y_vals = Vec::new();

        for i in 0..n {
            let ratio = 1.0 - signals[i] / dt_inf;
            if ratio > 0.01 && ratio < 0.99 {
                x_vals.push(times[i]);
                y_vals.push(ratio.ln());
            }
        }

        if x_vals.len() < 2 {
            return None;
        }

        let (slope, _intercept, _r_sq) = linear_regression(&x_vals, &y_vals)?;

        if slope >= 0.0 {
            return None; // Should be negative for approach to equilibrium
        }

        let tau = -1.0 / slope;
        Some((dt_inf, tau))
    }

    /// Check if measurement has reached steady state.
    ///
    /// Criterion: relative change over last n_window points < threshold.
    pub fn is_steady_state(
        signals: &[f64],
        n_window: usize,
        threshold: f64,
    ) -> bool {
        if signals.len() < n_window || n_window < 2 {
            return false;
        }

        let start = signals.len() - n_window;
        let window = &signals[start..];

        let mean = window.iter().sum::<f64>() / window.len() as f64;
        if mean.abs() < 1e-15 {
            return true; // Zero signal is trivially steady
        }

        let max_dev = window
            .iter()
            .map(|&v| ((v - mean) / mean).abs())
            .fold(0.0_f64, |a, b| a.max(b));

        max_dev < threshold
    }
}

// ────────────────────────────── Association Analysis ─────────────────────────

/// Analysis of molecular association/aggregation from apparent MW data.
///
/// If measured Mn varies with concentration, it may indicate self-association.
/// For monomer-dimer equilibrium: 1/M_app = 1/M_1 × [1/(1+K·c)]
pub struct AssociationAnalyzer;

impl AssociationAnalyzer {
    /// Calculate the apparent degree of association from apparent and true MW.
    ///
    /// n_app = M_apparent / M_monomer
    pub fn degree_of_association(m_apparent: f64, m_monomer: f64) -> f64 {
        m_apparent / m_monomer
    }

    /// Estimate monomer-dimer equilibrium constant from concentration-dependent MW.
    ///
    /// For simple dimerization: 2A ⇌ A₂
    /// M_app = M₁ × (1 + 4K·c·M₁)^(1/2) / (1 + ... ) simplified
    ///
    /// Uses linear regression of 1/M_app vs 1/c
    pub fn dimerization_constant(
        concentrations: &[f64],
        apparent_mws: &[f64],
        monomer_mw: f64,
    ) -> Option<f64> {
        if concentrations.len() != apparent_mws.len() || concentrations.len() < 3 {
            return None;
        }

        // For weak association: M_app ≈ M₁ × (1 + 2K·c)
        // So M_app/M₁ - 1 = 2K·c
        // Linear fit of (M_app/M₁ - 1) vs c gives slope = 2K

        let mut x = Vec::new();
        let mut y = Vec::new();

        for i in 0..concentrations.len() {
            if concentrations[i] > 0.0 && apparent_mws[i] > 0.0 {
                x.push(concentrations[i]);
                y.push(apparent_mws[i] / monomer_mw - 1.0);
            }
        }

        if x.len() < 2 {
            return None;
        }

        let (slope, _intercept, _r_sq) = linear_regression(&x, &y)?;
        let k = slope / 2.0;
        if k > 0.0 {
            Some(k)
        } else {
            None
        }
    }
}

// ────────────────────────────── Data Processing ─────────────────────────────

/// Process a set of replicate measurements by averaging.
pub fn average_replicates(measurements: &[VpoMeasurement]) -> Option<VpoMeasurement> {
    if measurements.is_empty() {
        return None;
    }

    let n = measurements.len() as f64;
    let avg_conc = measurements.iter().map(|m| m.concentration).sum::<f64>() / n;
    let avg_dt = measurements.iter().map(|m| m.delta_t).sum::<f64>() / n;

    let avg_dv = if measurements.iter().all(|m| m.delta_v.is_some()) {
        Some(
            measurements
                .iter()
                .map(|m| m.delta_v.unwrap())
                .sum::<f64>()
                / n,
        )
    } else {
        None
    };

    Some(VpoMeasurement {
        concentration: avg_conc,
        delta_t: avg_dt,
        delta_v: avg_dv,
        time_s: None,
    })
}

/// Calculate standard deviation of replicate ΔT measurements.
pub fn replicate_std_dev(measurements: &[VpoMeasurement]) -> Option<f64> {
    if measurements.len() < 2 {
        return None;
    }

    let n = measurements.len() as f64;
    let mean = measurements.iter().map(|m| m.delta_t).sum::<f64>() / n;
    let variance = measurements
        .iter()
        .map(|m| (m.delta_t - mean) * (m.delta_t - mean))
        .sum::<f64>()
        / (n - 1.0);

    Some(variance.sqrt())
}

/// Remove outliers from replicate measurements using Grubbs' test (simplified).
///
/// Removes values more than `z_threshold` standard deviations from the mean.
pub fn remove_outliers(measurements: &[VpoMeasurement], z_threshold: f64) -> Vec<VpoMeasurement> {
    if measurements.len() < 3 {
        return measurements.to_vec();
    }

    let n = measurements.len() as f64;
    let mean = measurements.iter().map(|m| m.delta_t).sum::<f64>() / n;
    let std_dev = replicate_std_dev(measurements).unwrap_or(0.0);

    if std_dev < 1e-15 {
        return measurements.to_vec();
    }

    measurements
        .iter()
        .filter(|m| ((m.delta_t - mean) / std_dev).abs() < z_threshold)
        .cloned()
        .collect()
}

/// Interpolate ΔT/c at c=0 from a sorted concentration series using Lagrange interpolation.
pub fn extrapolate_to_zero(concentrations: &[f64], dt_over_c: &[f64]) -> Option<f64> {
    if concentrations.len() != dt_over_c.len() || concentrations.len() < 2 {
        return None;
    }

    // Use linear extrapolation from the two lowest concentration points
    let (slope, intercept, _) = linear_regression(concentrations, dt_over_c)?;
    let _ = slope; // slope is used implicitly in the linear_regression intercept
    Some(intercept)
}

// ────────────────────────────── Statistical Functions ────────────────────────

/// Perform linear regression y = slope × x + intercept.
///
/// Returns (slope, intercept, r_squared).
pub fn linear_regression(x: &[f64], y: &[f64]) -> Option<(f64, f64, f64)> {
    let n = x.len();
    if n != y.len() || n < 2 {
        return None;
    }

    let n_f = n as f64;
    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xx: f64 = x.iter().map(|&xi| xi * xi).sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(&xi, &yi)| xi * yi).sum();
    let sum_yy: f64 = y.iter().map(|&yi| yi * yi).sum();

    let denom = n_f * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return None;
    }

    let slope = (n_f * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n_f;

    // R² calculation
    let ss_tot = sum_yy - sum_y * sum_y / n_f;
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(&xi, &yi)| {
            let pred = slope * xi + intercept;
            (yi - pred) * (yi - pred)
        })
        .sum();

    let r_squared = if ss_tot.abs() < 1e-30 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };

    Some((slope, intercept, r_squared))
}

/// Perform quadratic regression y = b0 + b1*x + b2*x².
///
/// Returns (b0, b1, b2, r_squared).
pub fn quadratic_regression(x: &[f64], y: &[f64]) -> Option<(f64, f64, f64, f64)> {
    let n = x.len();
    if n != y.len() || n < 3 {
        return None;
    }

    // Solve normal equations using Cramer's rule for 3x3 system
    // [n     Σx    Σx²  ] [b0]   [Σy   ]
    // [Σx    Σx²   Σx³  ] [b1] = [Σxy  ]
    // [Σx²   Σx³   Σx⁴  ] [b2]   [Σx²y ]

    let n_f = n as f64;
    let mut s = [0.0_f64; 5]; // s[k] = Σ x^k
    s[0] = n_f;
    for &xi in x.iter() {
        let mut xp = xi;
        s[1] += xp;
        xp *= xi;
        s[2] += xp;
        xp *= xi;
        s[3] += xp;
        xp *= xi;
        s[4] += xp;
    }

    let mut t = [0.0_f64; 3]; // t[k] = Σ x^k * y
    for (&xi, &yi) in x.iter().zip(y.iter()) {
        t[0] += yi;
        t[1] += xi * yi;
        t[2] += xi * xi * yi;
    }

    // 3x3 determinant
    let det = |a: [[f64; 3]; 3]| -> f64 {
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    };

    let d = det([
        [s[0], s[1], s[2]],
        [s[1], s[2], s[3]],
        [s[2], s[3], s[4]],
    ]);

    if d.abs() < 1e-30 {
        return None;
    }

    let d0 = det([
        [t[0], s[1], s[2]],
        [t[1], s[2], s[3]],
        [t[2], s[3], s[4]],
    ]);

    let d1 = det([
        [s[0], t[0], s[2]],
        [s[1], t[1], s[3]],
        [s[2], t[2], s[4]],
    ]);

    let d2 = det([
        [s[0], s[1], t[0]],
        [s[1], s[2], t[1]],
        [s[2], s[3], t[2]],
    ]);

    let b0 = d0 / d;
    let b1 = d1 / d;
    let b2 = d2 / d;

    // R² calculation
    let sum_yy: f64 = y.iter().map(|&yi| yi * yi).sum();
    let mean_y = t[0] / n_f;
    let ss_tot = sum_yy - n_f * mean_y * mean_y;
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(&xi, &yi)| {
            let pred = b0 + b1 * xi + b2 * xi * xi;
            (yi - pred) * (yi - pred)
        })
        .sum();

    let r_squared = if ss_tot.abs() < 1e-30 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };

    Some((b0, b1, b2, r_squared))
}

/// Calculate Pearson correlation coefficient.
pub fn correlation_coefficient(x: &[f64], y: &[f64]) -> Option<f64> {
    let (slope, _intercept, r_squared) = linear_regression(x, y)?;
    let sign = if slope >= 0.0 { 1.0 } else { -1.0 };
    if r_squared < 0.0 {
        Some(0.0)
    } else {
        Some(sign * r_squared.sqrt())
    }
}

/// Calculate mean of a slice.
pub fn mean(data: &[f64]) -> Option<f64> {
    if data.is_empty() {
        return None;
    }
    Some(data.iter().sum::<f64>() / data.len() as f64)
}

/// Calculate variance of a slice (sample variance, N-1 denominator).
pub fn variance(data: &[f64]) -> Option<f64> {
    if data.len() < 2 {
        return None;
    }
    let m = mean(data)?;
    let n = data.len() as f64;
    Some(data.iter().map(|&x| (x - m) * (x - m)).sum::<f64>() / (n - 1.0))
}

/// Calculate standard deviation of a slice.
pub fn std_dev(data: &[f64]) -> Option<f64> {
    variance(data).map(|v| v.sqrt())
}

/// Calculate relative standard deviation (coefficient of variation) in percent.
pub fn relative_std_dev(data: &[f64]) -> Option<f64> {
    let m = mean(data)?;
    let s = std_dev(data)?;
    if m.abs() < 1e-15 {
        return None;
    }
    Some(100.0 * s / m.abs())
}

// ────────────────────────────── Complete VPO Session ─────────────────────────

/// Complete VPO analysis session handling calibration and unknown measurement.
#[derive(Debug, Clone)]
pub struct VpoSession {
    /// Configuration for this session
    pub config: VpoConfig,
    /// Calibration curve (if established)
    pub calibration: Option<CalibrationCurve>,
}

impl VpoSession {
    /// Create a new VPO session.
    pub fn new(config: VpoConfig) -> Self {
        Self {
            config,
            calibration: None,
        }
    }

    /// Perform calibration using standard measurements.
    pub fn calibrate(&mut self, measurements: &[VpoMeasurement]) -> Option<&CalibrationCurve> {
        let cal = CalibrationCurve::from_measurements(
            measurements,
            self.config.calibration_standard,
        )?;
        self.calibration = Some(cal);
        self.calibration.as_ref()
    }

    /// Calculate molecular weight of unknown using established calibration.
    pub fn determine_mw(&self, measurements: &[VpoMeasurement]) -> Option<MwResult> {
        let cal = self.calibration.as_ref()?;
        let calc = MolecularWeightCalculator::from_calibration(cal);
        calc.mn_from_series(measurements)
    }

    /// Perform virial analysis on unknown measurements.
    pub fn virial_analysis(&self, measurements: &[VpoMeasurement]) -> Option<VirialResult> {
        let cal = self.calibration.as_ref()?;
        let virial = VirialExpansion::new(cal.k_cal);
        virial.analyze(measurements)
    }

    /// Check if calibration is valid (good R² and reasonable K_cal).
    pub fn is_calibration_valid(&self) -> bool {
        match &self.calibration {
            Some(cal) => cal.r_squared > 0.99 && cal.k_cal > 0.0,
            None => false,
        }
    }

    /// Get the calibration constant K_cal.
    pub fn k_cal(&self) -> Option<f64> {
        self.calibration.as_ref().map(|c| c.k_cal)
    }
}

// ────────────────────────────── Solvent Activity ─────────────────────────────

/// Calculate solvent activity from VPO ΔT measurement.
///
/// a₁ = 1 - (ΔT × M_solvent) / (Kb × 1000 × T_ref)
///
/// More precisely: a₁ ≈ exp(-ΔT × M_solvent / (Kb × 1000))
/// For dilute solutions: a₁ ≈ 1 - x_solute
pub fn solvent_activity_from_vpo(
    delta_t: f64,
    solvent: &SolventProperties,
) -> f64 {
    // From Raoult's law: ΔT = Kb × m, and a₁ ≈ 1 - x₂ ≈ exp(-m × M₁/1000)
    let m = delta_t / solvent.kb; // molality
    (-m * solvent.mw / 1000.0).exp()
}

/// Calculate solvent activity from mole fraction (ideal solution).
///
/// a₁ = x₁ = 1 - x₂
pub fn ideal_solvent_activity(mole_fraction_solute: f64) -> f64 {
    1.0 - mole_fraction_solute
}

// ────────────────────────────── Error Estimation ─────────────────────────────

/// Estimate the uncertainty in Mn from measurement uncertainties.
///
/// Using error propagation: Mn = K_cal / (ΔT/c)
/// σ_Mn/Mn = √[(σ_Kcal/K_cal)² + (σ_(ΔT/c) / (ΔT/c))²]
pub fn mn_uncertainty(
    mn: f64,
    k_cal: f64,
    sigma_k_cal: f64,
    dt_over_c: f64,
    sigma_dt_over_c: f64,
) -> f64 {
    let rel_k = sigma_k_cal / k_cal;
    let rel_dtc = sigma_dt_over_c / dt_over_c;
    mn * (rel_k * rel_k + rel_dtc * rel_dtc).sqrt()
}

/// Calculate the precision of Mn determination as relative standard deviation.
pub fn mn_precision_rsd(mn_values: &[f64]) -> Option<f64> {
    relative_std_dev(mn_values)
}

// ──────────────────────────────────── Tests ──────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if b.abs() < 1e-15 {
            a.abs() < rel_tol
        } else {
            ((a - b) / b).abs() < rel_tol
        }
    }

    // ── Solvent Database Tests ──

    #[test]
    fn test_solvent_toluene_properties() {
        let s = SolventDatabase::toluene();
        assert_eq!(s.name, "Toluene");
        assert!(approx_eq(s.mw, 92.14, TOL));
        assert!(approx_eq(s.boiling_point_c, 110.6, TOL));
        assert!(approx_eq(s.delta_h_vap, 33180.0, TOL));
        assert!(approx_eq(s.kb, 3.40, TOL));
    }

    #[test]
    fn test_solvent_water_properties() {
        let s = SolventDatabase::water();
        assert!(approx_eq(s.mw, 18.015, TOL));
        assert!(approx_eq(s.boiling_point_c, 100.0, TOL));
        assert!(approx_eq(s.delta_h_vap, 40660.0, TOL));
        assert!(approx_eq(s.kb, 0.512, TOL));
    }

    #[test]
    fn test_solvent_chloroform_properties() {
        let s = SolventDatabase::chloroform();
        assert!(approx_eq(s.mw, 119.38, TOL));
        assert!(approx_eq(s.boiling_point_c, 61.2, TOL));
    }

    #[test]
    fn test_solvent_all_returns_seven() {
        let all = SolventDatabase::all();
        assert_eq!(all.len(), 7);
    }

    #[test]
    fn test_solvent_lookup_toluene() {
        let s = SolventDatabase::lookup("Toluene").unwrap();
        assert_eq!(s.name, "Toluene");
    }

    #[test]
    fn test_solvent_lookup_case_insensitive() {
        let s = SolventDatabase::lookup("toluene").unwrap();
        assert_eq!(s.name, "Toluene");
    }

    #[test]
    fn test_solvent_lookup_nonexistent() {
        assert!(SolventDatabase::lookup("Xylene").is_none());
    }

    #[test]
    fn test_solvent_molar_volume() {
        let s = SolventDatabase::toluene();
        let expected = 92.14 / 0.867;
        assert!(approx_eq(s.molar_volume, expected, 0.01));
    }

    // ── Calibration Standard Tests ──

    #[test]
    fn test_benzil_mw() {
        let std = CalibrationStandard::Benzil;
        assert!(approx_eq(std.molecular_weight(), 210.23, TOL));
    }

    #[test]
    fn test_sucrose_octaacetate_mw() {
        let std = CalibrationStandard::SucroseOctaacetate;
        assert!(approx_eq(std.molecular_weight(), 678.60, TOL));
    }

    #[test]
    fn test_custom_standard_mw() {
        let std = CalibrationStandard::Custom(500.0);
        assert!(approx_eq(std.molecular_weight(), 500.0, TOL));
    }

    // ── VpoConfig Tests ──

    #[test]
    fn test_vpo_config_defaults() {
        let config = VpoConfig::new(SolventDatabase::toluene(), 37.0);
        assert!(approx_eq(config.temperature_c, 37.0, TOL));
        assert!(approx_eq(config.thermistor_sensitivity, 100.0, TOL));
        assert!(approx_eq(config.bridge_excitation_v, 5.0, TOL));
        assert_eq!(config.replicates, 3);
    }

    #[test]
    fn test_vpo_config_temperature_k() {
        let config = VpoConfig::new(SolventDatabase::toluene(), 37.0);
        assert!(approx_eq(config.temperature_k(), 310.15, TOL));
    }

    #[test]
    fn test_vpo_config_builder() {
        let config = VpoConfig::new(SolventDatabase::water(), 25.0)
            .with_standard(CalibrationStandard::SucroseOctaacetate)
            .with_sensitivity(200.0)
            .with_bridge_voltage(10.0)
            .with_replicates(5);
        assert!(approx_eq(config.thermistor_sensitivity, 200.0, TOL));
        assert!(approx_eq(config.bridge_excitation_v, 10.0, TOL));
        assert_eq!(config.replicates, 5);
    }

    #[test]
    fn test_theoretical_kb() {
        // For water at 100°C: Kb = R × T² × M / (1000 × ΔH)
        let config = VpoConfig::new(SolventDatabase::water(), 100.0);
        let kb = config.theoretical_kb();
        // Expected ~ 0.512 K·kg/mol
        assert!(relative_eq(kb, 0.512, 0.05));
    }

    // ── VpoMeasurement Tests ──

    #[test]
    fn test_measurement_new() {
        let m = VpoMeasurement::new(10.0, 0.5);
        assert!(approx_eq(m.concentration, 10.0, TOL));
        assert!(approx_eq(m.delta_t, 0.5, TOL));
        assert!(m.delta_v.is_none());
    }

    #[test]
    fn test_measurement_from_bridge_voltage() {
        let m = VpoMeasurement::from_bridge_voltage(10.0, 50.0, 100.0);
        assert!(approx_eq(m.delta_t, 0.5, TOL));
        assert!(approx_eq(m.delta_v.unwrap(), 50.0, TOL));
    }

    #[test]
    fn test_measurement_reduced_signal() {
        let m = VpoMeasurement::new(10.0, 0.5);
        let rs = m.reduced_signal().unwrap();
        assert!(approx_eq(rs, 0.05, TOL));
    }

    #[test]
    fn test_measurement_reduced_signal_zero_conc() {
        let m = VpoMeasurement::new(0.0, 0.5);
        assert!(m.reduced_signal().is_none());
    }

    #[test]
    fn test_measurement_with_time() {
        let m = VpoMeasurement::new(10.0, 0.5).with_time(120.0);
        assert!(approx_eq(m.time_s.unwrap(), 120.0, TOL));
    }

    // ── Calibration Curve Tests ──

    #[test]
    fn test_calibration_from_measurements() {
        // Benzil (MW=210.23) calibration data
        // ΔT = (K_cal/MW) × c, so slope = K_cal/MW
        let measurements = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(15.0, 0.75),
            VpoMeasurement::new(20.0, 1.00),
        ];
        let cal =
            CalibrationCurve::from_measurements(&measurements, CalibrationStandard::Benzil)
                .unwrap();
        // slope = 0.05 °C·L/g, K_cal = 0.05 × 210.23 = 10.5115
        assert!(relative_eq(cal.slope, 0.05, 0.01));
        assert!(relative_eq(cal.k_cal, 10.5115, 0.01));
        assert!(cal.r_squared > 0.99);
    }

    #[test]
    fn test_calibration_insufficient_points() {
        let measurements = vec![VpoMeasurement::new(5.0, 0.25)];
        let cal =
            CalibrationCurve::from_measurements(&measurements, CalibrationStandard::Benzil);
        assert!(cal.is_none());
    }

    #[test]
    fn test_calibration_extrapolation() {
        // ΔT/c should be constant for ideal solutions
        let measurements = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(15.0, 0.75),
        ];
        let cal = CalibrationCurve::from_extrapolation(
            &measurements,
            CalibrationStandard::Benzil,
        )
        .unwrap();
        // ΔT/c = 0.05 for all, intercept = 0.05, K_cal = 0.05 × 210.23
        assert!(relative_eq(cal.k_cal, 10.5115, 0.02));
    }

    // ── Molecular Weight Calculator Tests ──

    #[test]
    fn test_mw_from_single() {
        let calc = MolecularWeightCalculator::new(10.5);
        let m = VpoMeasurement::new(10.0, 0.5);
        let mn = calc.mn_from_single(&m).unwrap();
        // K_cal / (ΔT/c) = 10.5 / 0.05 = 210
        assert!(relative_eq(mn, 210.0, 0.01));
    }

    #[test]
    fn test_mw_from_series() {
        let calc = MolecularWeightCalculator::new(10.5);
        let measurements = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(15.0, 0.75),
            VpoMeasurement::new(20.0, 1.00),
        ];
        let result = calc.mn_from_series(&measurements).unwrap();
        // ΔT/c = 0.05 constant, intercept = 0.05, Mn = 10.5/0.05 = 210
        assert!(relative_eq(result.mn, 210.0, 0.01));
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_mw_from_intercept() {
        let calc = MolecularWeightCalculator::new(10.5);
        let mn = calc.mn_from_intercept(0.05).unwrap();
        assert!(relative_eq(mn, 210.0, 0.01));
    }

    #[test]
    fn test_mw_from_intercept_zero() {
        let calc = MolecularWeightCalculator::new(10.5);
        assert!(calc.mn_from_intercept(0.0).is_none());
    }

    #[test]
    fn test_mw_in_range() {
        let calc = MolecularWeightCalculator::new(10.5);
        let measurements = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(15.0, 0.75),
        ];
        let result = calc.mn_from_series(&measurements).unwrap();
        assert!(result.in_range); // 210 is in 50-20000
    }

    // ── Virial Expansion Tests ──

    #[test]
    fn test_virial_ideal_solution() {
        let virial = VirialExpansion::new(10.5);
        let measurements = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(15.0, 0.75),
            VpoMeasurement::new(20.0, 1.00),
        ];
        let result = virial.analyze(&measurements).unwrap();
        // For ideal: A₂ ≈ 0
        assert!(result.a2.abs() < 0.01);
        assert!(relative_eq(result.mn, 210.0, 0.01));
    }

    #[test]
    fn test_virial_nonideal_positive_a2() {
        // ΔT/c = 0.05 + 0.001 × c (positive A₂ = good solvent)
        let k_cal = 10.5;
        let virial = VirialExpansion::new(k_cal);
        let measurements = vec![
            VpoMeasurement::new(5.0, 5.0 * (0.05 + 0.001 * 5.0)),
            VpoMeasurement::new(10.0, 10.0 * (0.05 + 0.001 * 10.0)),
            VpoMeasurement::new(15.0, 15.0 * (0.05 + 0.001 * 15.0)),
            VpoMeasurement::new(20.0, 20.0 * (0.05 + 0.001 * 20.0)),
        ];
        let result = virial.analyze(&measurements).unwrap();
        assert!(result.a2 > 0.0);
        assert!(relative_eq(result.mn, 210.0, 0.02));
    }

    #[test]
    fn test_virial_insufficient_data() {
        let virial = VirialExpansion::new(10.5);
        let measurements = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
        ];
        assert!(virial.analyze(&measurements).is_none());
    }

    #[test]
    fn test_virial_predict_delta_t() {
        let virial = VirialExpansion::new(10.5);
        let dt = virial.predict_delta_t(210.0, 0.001, 10.0);
        // dt = 10.5 × 10 × (1/210 + 0.001 × 10) = 10.5 × 10 × (0.00476 + 0.01)
        let expected = 10.5 * 10.0 * (1.0 / 210.0 + 0.001 * 10.0);
        assert!(approx_eq(dt, expected, 0.001));
    }

    #[test]
    fn test_virial_reduced_osmotic_pressure() {
        let virial = VirialExpansion::new(10.5);
        let rop = virial.reduced_osmotic_pressure(210.0, 0.001, 10.0);
        let expected = 1.0 / 210.0 + 0.001 * 10.0;
        assert!(approx_eq(rop, expected, TOL));
    }

    #[test]
    fn test_virial_quadratic() {
        let k_cal = 10.5;
        let virial = VirialExpansion::new(k_cal);
        // ΔT/c = 0.05 + 0.001×c + 0.00005×c²
        let measurements = vec![
            VpoMeasurement::new(5.0, 5.0 * (0.05 + 0.001 * 5.0 + 0.00005 * 25.0)),
            VpoMeasurement::new(10.0, 10.0 * (0.05 + 0.001 * 10.0 + 0.00005 * 100.0)),
            VpoMeasurement::new(15.0, 15.0 * (0.05 + 0.001 * 15.0 + 0.00005 * 225.0)),
            VpoMeasurement::new(20.0, 20.0 * (0.05 + 0.001 * 20.0 + 0.00005 * 400.0)),
            VpoMeasurement::new(25.0, 25.0 * (0.05 + 0.001 * 25.0 + 0.00005 * 625.0)),
        ];
        let result = virial.analyze_quadratic(&measurements).unwrap();
        assert!(relative_eq(result.mn, k_cal / 0.05, 0.02));
        assert!(result.r_squared > 0.99);
    }

    // ── Raoult's Law Tests ──

    #[test]
    fn test_raoults_law_delta_p() {
        let dp = RaoultsLaw::delta_p(760.0, 0.01);
        assert!(approx_eq(dp, 7.6, TOL));
    }

    #[test]
    fn test_raoults_law_solution_vp() {
        let p = RaoultsLaw::solution_vapor_pressure(760.0, 0.01);
        assert!(approx_eq(p, 752.4, TOL));
    }

    #[test]
    fn test_raoults_law_relative_lowering() {
        let rl = RaoultsLaw::relative_lowering(0.05);
        assert!(approx_eq(rl, 0.05, TOL));
    }

    #[test]
    fn test_boiling_point_elevation() {
        let dt = RaoultsLaw::boiling_point_elevation(0.512, 1.0);
        assert!(approx_eq(dt, 0.512, TOL));
    }

    #[test]
    fn test_expected_delta_t() {
        // 1g of MW=100 in 100g water: m=0.1, ΔT=0.512×0.1=0.0512
        let dt = RaoultsLaw::expected_delta_t(0.512, 1.0, 100.0, 100.0);
        assert!(approx_eq(dt, 0.0512, 0.001));
    }

    #[test]
    fn test_raoults_law_free_function() {
        let dp = raoults_law_delta_p(760.0, 0.01);
        assert!(approx_eq(dp, 7.6, TOL));
    }

    // ── Flory-Huggins Tests ──

    #[test]
    fn test_fh_chi_from_activity() {
        // Known: for χ = 0.4, φ₂ = 0.1, r = 100
        // ln(a₁) = ln(0.9) + (1-0.01)×0.1 + 0.4×0.01
        let phi = 0.1;
        let r = 100.0;
        let chi_expected = 0.4;
        let a1 = FloryHugginsAnalyzer::activity_from_chi(chi_expected, phi, r);
        let chi_calc = FloryHugginsAnalyzer::chi_from_activity(a1, phi, r).unwrap();
        assert!(approx_eq(chi_calc, chi_expected, 0.001));
    }

    #[test]
    fn test_fh_activity_from_chi() {
        let a1 = FloryHugginsAnalyzer::activity_from_chi(0.0, 0.0, 100.0);
        assert!(approx_eq(a1, 1.0, TOL)); // Pure solvent
    }

    #[test]
    fn test_fh_chi_invalid_activity() {
        assert!(FloryHugginsAnalyzer::chi_from_activity(0.0, 0.1, 100.0).is_none());
        assert!(FloryHugginsAnalyzer::chi_from_activity(1.5, 0.1, 100.0).is_none());
    }

    #[test]
    fn test_fh_chi_zero_volume_fraction() {
        assert!(FloryHugginsAnalyzer::chi_from_activity(0.9, 0.0, 100.0).is_none());
    }

    #[test]
    fn test_fh_chi_from_series() {
        let r = 100.0;
        let chi_true = 0.4;
        let phis = vec![0.05, 0.10, 0.15, 0.20, 0.25];
        let activities: Vec<f64> = phis
            .iter()
            .map(|&phi| FloryHugginsAnalyzer::activity_from_chi(chi_true, phi, r))
            .collect();

        let result =
            FloryHugginsAnalyzer::chi_from_series(&activities, &phis, r).unwrap();
        assert!(relative_eq(result.chi, chi_true, 0.05));
        assert!(result.r_squared > 0.99);
        assert!(result.good_solvent); // χ=0.4 < 0.5
    }

    #[test]
    fn test_fh_critical_chi() {
        // For r→∞, χ_c → 0.5
        let chi_c = FloryHugginsAnalyzer::critical_chi(1e6);
        assert!(relative_eq(chi_c, 0.5, 0.01));
    }

    #[test]
    fn test_fh_critical_volume_fraction() {
        // For r=100: φ_c = 1/(1+10) ≈ 0.0909
        let phi_c = FloryHugginsAnalyzer::critical_volume_fraction(100.0);
        assert!(relative_eq(phi_c, 1.0 / 11.0, 0.001));
    }

    #[test]
    fn test_fh_osmotic_pressure() {
        // Zero polymer → zero osmotic pressure
        let pi = FloryHugginsAnalyzer::osmotic_pressure(300.0, 100.0, 0.4, 0.0, 100.0);
        assert!(approx_eq(pi, 0.0, 0.01));
    }

    #[test]
    fn test_flory_huggins_chi_free_function() {
        let phi = 0.1;
        let r = 100.0;
        let chi_expected = 0.3;
        let a = FloryHugginsAnalyzer::activity_from_chi(chi_expected, phi, r);
        let chi = flory_huggins_chi(a, phi, r).unwrap();
        assert!(approx_eq(chi, chi_expected, 0.001));
    }

    // ── Clausius-Clapeyron Tests ──

    #[test]
    fn test_clausius_clapeyron_same_temp() {
        let p = clausius_clapeyron(40660.0, 373.15, 760.0, 373.15);
        assert!(approx_eq(p, 760.0, TOL));
    }

    #[test]
    fn test_clausius_clapeyron_higher_temp() {
        // Water: higher temp → higher pressure
        let p = clausius_clapeyron(40660.0, 373.15, 760.0, 383.15);
        assert!(p > 760.0);
    }

    #[test]
    fn test_clausius_clapeyron_lower_temp() {
        let p = clausius_clapeyron(40660.0, 373.15, 760.0, 363.15);
        assert!(p < 760.0);
    }

    // ── Mole Fraction / Molality Tests ──

    #[test]
    fn test_mole_fraction_calculation() {
        // 18.015 g water (1 mol) + 58.44 g NaCl (1 mol)
        let x = mole_fraction(58.44, 58.44, 18.015, 18.015);
        assert!(approx_eq(x, 0.5, TOL));
    }

    #[test]
    fn test_mole_fraction_dilute() {
        // 1g of MW=100 in 1000g water
        let x = mole_fraction(1.0, 100.0, 1000.0, 18.015);
        // x = 0.01 / (0.01 + 55.51) ≈ 0.00018
        assert!(x < 0.001);
        assert!(x > 0.0);
    }

    #[test]
    fn test_molality_calculation() {
        // 1 mol of solute (MW=100, mass=100g) in 1 kg solvent
        let m = molality(100.0, 100.0, 1000.0);
        assert!(approx_eq(m, 1.0, TOL));
    }

    #[test]
    fn test_molarity_calculation() {
        // 1 mol (100g of MW=100) in 1 L
        let c = molarity(100.0, 100.0, 1.0);
        assert!(approx_eq(c, 1.0, TOL));
    }

    // ── VPO Signal to MW Tests ──

    #[test]
    fn test_vpo_signal_to_mw() {
        let mn = vpo_signal_to_mw(0.05, 10.5).unwrap();
        assert!(relative_eq(mn, 210.0, 0.01));
    }

    #[test]
    fn test_vpo_signal_to_mw_zero() {
        assert!(vpo_signal_to_mw(0.0, 10.5).is_none());
    }

    // ── Activity Coefficient Tests ──

    #[test]
    fn test_activity_coefficient_ideal() {
        // Ideal solution: γ = 1
        let gamma = activity_coefficient(0.99, 0.99).unwrap();
        assert!(approx_eq(gamma, 1.0, TOL));
    }

    #[test]
    fn test_activity_coefficient_nonideal() {
        let gamma = activity_coefficient(0.95, 0.99).unwrap();
        // γ = 0.95/0.99 ≈ 0.9596
        assert!(relative_eq(gamma, 0.95 / 0.99, 0.001));
    }

    #[test]
    fn test_activity_coefficient_zero_fraction() {
        assert!(activity_coefficient(0.9, 0.0).is_none());
    }

    // ── Osmotic Coefficient Tests ──

    #[test]
    fn test_osmotic_coefficient_calculation() {
        // φ = -1000 × ln(a₁) / (m × M₁)
        let phi = osmotic_coefficient(0.98, 0.5, 18.015).unwrap();
        let expected = -1000.0 * (0.98_f64).ln() / (0.5 * 18.015);
        assert!(approx_eq(phi, expected, 0.001));
    }

    #[test]
    fn test_osmotic_coefficient_zero_molality() {
        assert!(osmotic_coefficient(0.98, 0.0, 18.015).is_none());
    }

    // ── Volume Fraction Tests ──

    #[test]
    fn test_volume_fraction_pure_polymer() {
        let vf = volume_fraction(1.0, 1.2, 0.867);
        assert!(approx_eq(vf, 1.0, TOL));
    }

    #[test]
    fn test_volume_fraction_pure_solvent() {
        let vf = volume_fraction(0.0, 1.2, 0.867);
        assert!(approx_eq(vf, 0.0, TOL));
    }

    #[test]
    fn test_volume_fraction_half() {
        // Equal mass, same density → φ = 0.5
        let vf = volume_fraction(0.5, 1.0, 1.0);
        assert!(approx_eq(vf, 0.5, TOL));
    }

    // ── Ebullioscopic Constant Tests ──

    #[test]
    fn test_ebullioscopic_constant_water() {
        let kb = ebullioscopic_constant(373.15, 18.015, 40660.0);
        // Should be close to 0.512
        assert!(relative_eq(kb, 0.512, 0.05));
    }

    #[test]
    fn test_ebullioscopic_constant_toluene() {
        let kb = ebullioscopic_constant(383.75, 92.14, 33180.0);
        // Should be near 3.40
        assert!(relative_eq(kb, 3.40, 0.1));
    }

    // ── Antoine Equation Tests ──

    #[test]
    fn test_antoine_pressure_water_100c() {
        // Antoine coefficients for water (T in °C, P in mmHg)
        let p = antoine_pressure(8.07131, 1730.63, 233.426, 100.0);
        assert!(relative_eq(p, 760.0, 0.02));
    }

    // ── Unit Conversion Tests ──

    #[test]
    fn test_celsius_to_kelvin() {
        assert!(approx_eq(celsius_to_kelvin(0.0), 273.15, TOL));
        assert!(approx_eq(celsius_to_kelvin(100.0), 373.15, TOL));
    }

    #[test]
    fn test_kelvin_to_celsius() {
        assert!(approx_eq(kelvin_to_celsius(273.15), 0.0, TOL));
        assert!(approx_eq(kelvin_to_celsius(373.15), 100.0, TOL));
    }

    #[test]
    fn test_moles() {
        assert!(approx_eq(moles(18.015, 18.015), 1.0, TOL));
        assert!(approx_eq(moles(36.03, 18.015), 2.0, TOL));
    }

    // ── Wheatstone Bridge Tests ──

    #[test]
    fn test_bridge_balanced_zero_output() {
        let bridge = WheatstoneThermistorBridge::balanced(10000.0, 3950.0, 310.15, 5.0);
        let v = bridge.bridge_output(0.0);
        assert!(approx_eq(v, 0.0, 1e-10));
    }

    #[test]
    fn test_bridge_nonzero_delta_t() {
        let bridge = WheatstoneThermistorBridge::balanced(10000.0, 3950.0, 310.15, 5.0);
        let v = bridge.bridge_output(0.01);
        assert!(v.abs() > 0.0);
    }

    #[test]
    fn test_bridge_sensitivity_nonzero() {
        let bridge = WheatstoneThermistorBridge::balanced(10000.0, 3950.0, 310.15, 5.0);
        let sens = bridge.sensitivity();
        assert!(sens.abs() > 0.0);
    }

    #[test]
    fn test_thermistor_resistance_at_ref() {
        let bridge = WheatstoneThermistorBridge::balanced(10000.0, 3950.0, 310.15, 5.0);
        let r = bridge.thermistor_resistance(310.15);
        assert!(approx_eq(r, 10000.0, 0.01));
    }

    // ── Equilibration Analyzer Tests ──

    #[test]
    fn test_equilibration_fit() {
        // Simulate ΔT(t) = 0.5 × (1 - exp(-t/30))
        let dt_inf = 0.5;
        let tau = 30.0;
        let times: Vec<f64> = (0..20).map(|i| i as f64 * 5.0).collect();
        let signals: Vec<f64> = times
            .iter()
            .map(|&t| dt_inf * (1.0 - (-t / tau).exp()))
            .collect();

        let (est_inf, est_tau) = EquilibrationAnalyzer::fit_exponential(&times, &signals).unwrap();
        assert!(relative_eq(est_tau, tau, 0.30));
        assert!(relative_eq(est_inf, dt_inf, 0.10));
    }

    #[test]
    fn test_steady_state_check_stable() {
        let signals = vec![0.50, 0.50, 0.50, 0.50, 0.50];
        assert!(EquilibrationAnalyzer::is_steady_state(&signals, 3, 0.01));
    }

    #[test]
    fn test_steady_state_check_unstable() {
        let signals = vec![0.40, 0.42, 0.44, 0.46, 0.48];
        assert!(!EquilibrationAnalyzer::is_steady_state(&signals, 3, 0.01));
    }

    // ── Association Analyzer Tests ──

    #[test]
    fn test_degree_of_association() {
        let n = AssociationAnalyzer::degree_of_association(400.0, 200.0);
        assert!(approx_eq(n, 2.0, TOL));
    }

    #[test]
    fn test_dimerization_constant() {
        // Simulate weak association: M_app = M₁ × (1 + 2K·c)
        let m1 = 200.0;
        let k = 0.01; // L/g
        let conc = vec![5.0, 10.0, 15.0, 20.0];
        let mw_app: Vec<f64> = conc.iter().map(|&c| m1 * (1.0 + 2.0 * k * c)).collect();

        let k_est = AssociationAnalyzer::dimerization_constant(&conc, &mw_app, m1).unwrap();
        assert!(relative_eq(k_est, k, 0.01));
    }

    // ── Data Processing Tests ──

    #[test]
    fn test_average_replicates() {
        let measurements = vec![
            VpoMeasurement::new(10.0, 0.48),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(10.0, 0.52),
        ];
        let avg = average_replicates(&measurements).unwrap();
        assert!(approx_eq(avg.delta_t, 0.50, TOL));
        assert!(approx_eq(avg.concentration, 10.0, TOL));
    }

    #[test]
    fn test_average_replicates_empty() {
        assert!(average_replicates(&[]).is_none());
    }

    #[test]
    fn test_replicate_std_dev() {
        let measurements = vec![
            VpoMeasurement::new(10.0, 0.48),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(10.0, 0.52),
        ];
        let sd = replicate_std_dev(&measurements).unwrap();
        assert!(approx_eq(sd, 0.02, 0.001));
    }

    #[test]
    fn test_remove_outliers() {
        // Need enough points so the outlier z-score exceeds threshold
        // (with n=4, max z-score is sqrt(n-1)=1.73, can't exceed 2.0)
        let measurements = vec![
            VpoMeasurement::new(10.0, 0.49),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(10.0, 0.51),
            VpoMeasurement::new(10.0, 0.49),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(10.0, 0.51),
            VpoMeasurement::new(10.0, 1.50), // outlier
        ];
        let filtered = remove_outliers(&measurements, 2.0);
        assert_eq!(filtered.len(), 6);
    }

    #[test]
    fn test_extrapolate_to_zero() {
        let conc = vec![5.0, 10.0, 15.0, 20.0];
        let dt_c = vec![0.050, 0.051, 0.052, 0.053]; // slightly increasing
        let intercept = extrapolate_to_zero(&conc, &dt_c).unwrap();
        assert!(relative_eq(intercept, 0.0490, 0.02));
    }

    // ── Linear Regression Tests ──

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![1.0, 2.0, 3.0, 4.0];
        let y = vec![2.0, 4.0, 6.0, 8.0];
        let (slope, intercept, r_sq) = linear_regression(&x, &y).unwrap();
        assert!(approx_eq(slope, 2.0, TOL));
        assert!(approx_eq(intercept, 0.0, TOL));
        assert!(approx_eq(r_sq, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_with_intercept() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let (slope, intercept, r_sq) = linear_regression(&x, &y).unwrap();
        assert!(approx_eq(slope, 2.0, TOL));
        assert!(approx_eq(intercept, 1.0, TOL));
        assert!(approx_eq(r_sq, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_insufficient_data() {
        let x = vec![1.0];
        let y = vec![2.0];
        assert!(linear_regression(&x, &y).is_none());
    }

    #[test]
    fn test_linear_regression_mismatched_length() {
        let x = vec![1.0, 2.0];
        let y = vec![1.0];
        assert!(linear_regression(&x, &y).is_none());
    }

    // ── Quadratic Regression Tests ──

    #[test]
    fn test_quadratic_regression_perfect() {
        // y = 1 + 2x + 3x²
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|&xi| 1.0 + 2.0 * xi + 3.0 * xi * xi).collect();
        let (b0, b1, b2, r_sq) = quadratic_regression(&x, &y).unwrap();
        assert!(approx_eq(b0, 1.0, 0.001));
        assert!(approx_eq(b1, 2.0, 0.001));
        assert!(approx_eq(b2, 3.0, 0.001));
        assert!(approx_eq(r_sq, 1.0, 0.001));
    }

    #[test]
    fn test_quadratic_regression_insufficient_data() {
        let x = vec![1.0, 2.0];
        let y = vec![1.0, 2.0];
        assert!(quadratic_regression(&x, &y).is_none());
    }

    // ── Statistical Function Tests ──

    #[test]
    fn test_correlation_coefficient() {
        let x = vec![1.0, 2.0, 3.0, 4.0];
        let y = vec![2.0, 4.0, 6.0, 8.0];
        let r = correlation_coefficient(&x, &y).unwrap();
        assert!(approx_eq(r, 1.0, TOL));
    }

    #[test]
    fn test_correlation_coefficient_negative() {
        let x = vec![1.0, 2.0, 3.0, 4.0];
        let y = vec![8.0, 6.0, 4.0, 2.0];
        let r = correlation_coefficient(&x, &y).unwrap();
        assert!(approx_eq(r, -1.0, TOL));
    }

    #[test]
    fn test_mean() {
        assert!(approx_eq(mean(&[1.0, 2.0, 3.0]).unwrap(), 2.0, TOL));
    }

    #[test]
    fn test_mean_empty() {
        assert!(mean(&[]).is_none());
    }

    #[test]
    fn test_variance() {
        // Sample variance of [1, 2, 3] = 1.0
        let v = variance(&[1.0, 2.0, 3.0]).unwrap();
        assert!(approx_eq(v, 1.0, TOL));
    }

    #[test]
    fn test_std_dev() {
        let s = std_dev(&[1.0, 2.0, 3.0]).unwrap();
        assert!(approx_eq(s, 1.0, TOL));
    }

    #[test]
    fn test_relative_std_dev() {
        let rsd = relative_std_dev(&[100.0, 101.0, 99.0]).unwrap();
        assert!(rsd > 0.0);
        assert!(rsd < 5.0); // Should be ~ 1%
    }

    // ── Solvent Activity Tests ──

    #[test]
    fn test_solvent_activity_zero_dt() {
        let a = solvent_activity_from_vpo(0.0, &SolventDatabase::toluene());
        assert!(approx_eq(a, 1.0, TOL));
    }

    #[test]
    fn test_solvent_activity_positive_dt() {
        let a = solvent_activity_from_vpo(0.1, &SolventDatabase::toluene());
        assert!(a < 1.0);
        assert!(a > 0.9);
    }

    #[test]
    fn test_ideal_solvent_activity() {
        let a = ideal_solvent_activity(0.01);
        assert!(approx_eq(a, 0.99, TOL));
    }

    // ── VPO Session Tests ──

    #[test]
    fn test_vpo_session_calibrate_and_measure() {
        let config = VpoConfig::new(SolventDatabase::toluene(), 37.0);
        let mut session = VpoSession::new(config);

        // Calibration with benzil
        let cal_data = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(15.0, 0.75),
        ];
        session.calibrate(&cal_data);
        assert!(session.is_calibration_valid());

        // Measure unknown
        let unknown = vec![
            VpoMeasurement::new(5.0, 0.125),
            VpoMeasurement::new(10.0, 0.25),
            VpoMeasurement::new(15.0, 0.375),
        ];
        let result = session.determine_mw(&unknown).unwrap();
        // Unknown has ΔT/c = 0.025, so Mn = K_cal/0.025
        // K_cal = 0.05 × 210.23 ≈ 10.51
        // Mn ≈ 10.51/0.025 ≈ 420
        assert!(relative_eq(result.mn, 420.0, 0.05));
    }

    #[test]
    fn test_vpo_session_no_calibration() {
        let config = VpoConfig::new(SolventDatabase::toluene(), 37.0);
        let session = VpoSession::new(config);
        assert!(!session.is_calibration_valid());

        let unknown = vec![VpoMeasurement::new(10.0, 0.25)];
        assert!(session.determine_mw(&unknown).is_none());
    }

    #[test]
    fn test_vpo_session_virial_analysis() {
        let config = VpoConfig::new(SolventDatabase::toluene(), 37.0);
        let mut session = VpoSession::new(config);

        let cal_data = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
            VpoMeasurement::new(15.0, 0.75),
            VpoMeasurement::new(20.0, 1.00),
        ];
        session.calibrate(&cal_data);

        let unknown = vec![
            VpoMeasurement::new(5.0, 0.125),
            VpoMeasurement::new(10.0, 0.25),
            VpoMeasurement::new(15.0, 0.375),
            VpoMeasurement::new(20.0, 0.50),
        ];
        let result = session.virial_analysis(&unknown).unwrap();
        assert!(result.mn > 0.0);
    }

    #[test]
    fn test_vpo_session_k_cal() {
        let config = VpoConfig::new(SolventDatabase::toluene(), 37.0);
        let mut session = VpoSession::new(config);
        assert!(session.k_cal().is_none());

        let cal_data = vec![
            VpoMeasurement::new(5.0, 0.25),
            VpoMeasurement::new(10.0, 0.50),
        ];
        session.calibrate(&cal_data);
        assert!(session.k_cal().is_some());
    }

    // ── Concentration Conversion Tests ──

    #[test]
    fn test_concentration_to_molality() {
        // 10 g/L of MW=100 in toluene (ρ=0.867)
        let m = concentration_to_molality(10.0, 100.0, 0.867);
        // m = (10/100) / 0.867 ≈ 0.1153
        assert!(relative_eq(m, 0.1 / 0.867, 0.01));
    }

    // ── Error Estimation Tests ──

    #[test]
    fn test_mn_uncertainty() {
        let sigma = mn_uncertainty(200.0, 10.0, 0.5, 0.05, 0.002);
        // σ_Mn/Mn = √[(0.5/10)² + (0.002/0.05)²] = √[0.0025 + 0.0016] ≈ 0.064
        // σ_Mn ≈ 200 × 0.064 ≈ 12.8
        assert!(relative_eq(sigma, 200.0 * (0.0025 + 0.0016_f64).sqrt(), 0.01));
    }

    #[test]
    fn test_mn_precision_rsd() {
        let values = vec![200.0, 202.0, 198.0, 201.0, 199.0];
        let rsd = mn_precision_rsd(&values).unwrap();
        assert!(rsd > 0.0);
        assert!(rsd < 5.0); // Should be about 0.8%
    }
}

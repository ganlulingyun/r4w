//! # Osmometry and Colligative Property Analyzer
//!
//! Implements osmometry techniques and colligative property analysis for
//! molecular weight determination, solution characterization, and pharmaceutical
//! isotonicity calculations.
//!
//! ## Key Components
//!
//! - **MembraneOsmometer** - Measures osmotic pressure pi vs concentration, extrapolates to c->0 for Mn
//! - **VaporPressureOsmometer** - VPO thermoelectric measurement for small molecule MW
//! - **CryoscopicAnalyzer** - Freezing point depression: Delta_Tf = Kf * m * i
//! - **EbullioscopicAnalyzer** - Boiling point elevation: Delta_Tb = Kb * m * i
//! - **VantHoffAnalyzer** - Virial expansion: pi = cRT(1/Mn + A2*c + A3*c^2 + ...)
//! - **SecondVirialCoefficient** - A2 from pi/c vs c linear extrapolation
//! - **IsotonicityCalculator** - Pharmaceutical isotonicity, NaCl equivalents
//! - **OsmoticCoefficient** - Activity coefficient corrections for non-ideal solutions
//!
//! ## Physics
//!
//! - Ideal osmotic pressure: pi = cRT/Mn (van't Hoff equation)
//! - Freezing point depression: Delta_Tf = Kf * m * i
//! - Boiling point elevation: Delta_Tb = Kb * m * i
//! - Virial expansion: pi/c = RT(1/Mn + A2*c + A3*c^2)
//! - Raoult's law: Delta_P/P0 = x_solute (ideal dilute)
//! - Osmolality: mOsm/kg = sum(phi * n_i * m_i) * 1000
//! - R = 8.314462618 J/(mol*K)
//!
//! ## Solvent Constants
//!
//! | Solvent  | Kf (K*kg/mol) | Kb (K*kg/mol) | Tf (C) | Tb (C) |
//! |----------|---------------|---------------|--------|--------|
//! | Water    | 1.86          | 0.512         | 0.0    | 100.0  |
//! | Benzene  | 5.12          | 2.53          | 5.5    | 80.1   |
//! | Camphor  | 40.0          | 5.61          | 179.8  | 204.0  |
//! | Cyclohexane | 20.0       | 2.79          | 6.6    | 80.7   |

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Universal gas constant in J/(mol*K)
const R_GAS: f64 = 8.314462618;

/// Water cryoscopic constant in K*kg/mol
const WATER_KF: f64 = 1.86;

/// Water ebullioscopic constant in K*kg/mol
const WATER_KB: f64 = 0.512;

/// Benzene cryoscopic constant in K*kg/mol
const BENZENE_KF: f64 = 5.12;

/// Benzene ebullioscopic constant in K*kg/mol
const BENZENE_KB: f64 = 2.53;

/// Camphor cryoscopic constant in K*kg/mol
const CAMPHOR_KF: f64 = 40.0;

/// Cyclohexane cryoscopic constant in K*kg/mol
const CYCLOHEXANE_KF: f64 = 20.0;

/// Normal blood osmolality lower bound in mOsm/kg
const BLOOD_OSMOLALITY_LOW: f64 = 285.0;

/// Normal blood osmolality upper bound in mOsm/kg
const BLOOD_OSMOLALITY_HIGH: f64 = 295.0;

/// Isotonic NaCl concentration in mol/L (0.9% w/v = 154 mM)
const ISOTONIC_NACL_MOLAR: f64 = 0.154;

/// Isotonic NaCl percent w/v
const ISOTONIC_NACL_PERCENT: f64 = 0.9;

/// Molar mass of NaCl in g/mol
const NACL_MOLAR_MASS: f64 = 58.44;

/// Molar mass of water in g/mol
const WATER_MOLAR_MASS: f64 = 18.015;

/// Water freezing point in K
const WATER_FP_K: f64 = 273.15;

/// Water boiling point in K
const WATER_BP_K: f64 = 373.15;

/// Density of water in kg/L at 25 C (approximation)
const WATER_DENSITY: f64 = 0.997;

// ---------------------------------------------------------------------------
// Linear regression utility
// ---------------------------------------------------------------------------

/// Result of a linear regression fit y = slope * x + intercept.
#[derive(Debug, Clone, Copy)]
pub struct LinearFit {
    /// Slope of the best-fit line.
    pub slope: f64,
    /// Y-intercept of the best-fit line.
    pub intercept: f64,
    /// Coefficient of determination (R-squared).
    pub r_squared: f64,
}

/// Perform ordinary least-squares linear regression on (x, y) data.
///
/// Returns (slope, intercept, r_squared).
///
/// # Panics
/// Panics if arrays differ in length or have fewer than 2 points.
pub fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    assert_eq!(x.len(), y.len(), "x and y must have the same length");
    let n = x.len();
    assert!(n >= 2, "need at least 2 data points for linear regression");

    let n_f = n as f64;
    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sum_x2: f64 = x.iter().map(|xi| xi * xi).sum();
    let sum_y2: f64 = y.iter().map(|yi| yi * yi).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / n_f, 0.0);
    }

    let slope = (n_f * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n_f;

    // R-squared
    let ss_tot = sum_y2 - sum_y * sum_y / n_f;
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| {
            let pred = slope * xi + intercept;
            (yi - pred) * (yi - pred)
        })
        .sum();

    let r_squared = if ss_tot.abs() < 1e-30 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };

    (slope, intercept, r_squared)
}

/// Perform linear regression returning a `LinearFit` struct.
pub fn linear_regression_fit(x: &[f64], y: &[f64]) -> LinearFit {
    let (slope, intercept, r_squared) = linear_regression(x, y);
    LinearFit {
        slope,
        intercept,
        r_squared,
    }
}

// ---------------------------------------------------------------------------
// Standalone colligative property functions
// ---------------------------------------------------------------------------

/// Calculate ideal osmotic pressure using van't Hoff equation.
///
/// pi = c * R * T / M
///
/// # Arguments
/// * `concentration` - Mass concentration in g/L
/// * `molar_mass` - Molar mass in g/mol
/// * `temperature` - Temperature in K
///
/// # Returns
/// Osmotic pressure in Pa
pub fn osmotic_pressure(concentration: f64, molar_mass: f64, temperature: f64) -> f64 {
    assert!(molar_mass > 0.0, "molar_mass must be positive");
    assert!(temperature > 0.0, "temperature must be positive");
    // c in g/L, M in g/mol => c/M = mol/L = molar concentration
    // pi = (c/M) * R * T * 1000 (convert L to m^3)
    (concentration / molar_mass) * R_GAS * temperature * 1000.0
}

/// Calculate osmotic pressure for an electrolyte with van't Hoff factor.
///
/// pi = i * c * R * T / M
pub fn osmotic_pressure_electrolyte(
    concentration: f64,
    molar_mass: f64,
    temperature: f64,
    van_hoff_i: f64,
) -> f64 {
    van_hoff_i * osmotic_pressure(concentration, molar_mass, temperature)
}

/// Calculate freezing point depression.
///
/// Delta_Tf = Kf * m * i
///
/// # Arguments
/// * `kf` - Cryoscopic constant in K*kg/mol
/// * `molality` - Molality in mol/kg
/// * `van_hoff_i` - van't Hoff factor (1 for non-electrolytes)
///
/// # Returns
/// Freezing point depression in K (always positive)
pub fn freezing_point_depression(kf: f64, molality: f64, van_hoff_i: f64) -> f64 {
    assert!(kf > 0.0, "kf must be positive");
    assert!(molality >= 0.0, "molality must be non-negative");
    assert!(van_hoff_i > 0.0, "van_hoff_i must be positive");
    kf * molality * van_hoff_i
}

/// Calculate boiling point elevation.
///
/// Delta_Tb = Kb * m * i
///
/// # Arguments
/// * `kb` - Ebullioscopic constant in K*kg/mol
/// * `molality` - Molality in mol/kg
/// * `van_hoff_i` - van't Hoff factor (1 for non-electrolytes)
///
/// # Returns
/// Boiling point elevation in K (always positive)
pub fn boiling_point_elevation(kb: f64, molality: f64, van_hoff_i: f64) -> f64 {
    assert!(kb > 0.0, "kb must be positive");
    assert!(molality >= 0.0, "molality must be non-negative");
    assert!(van_hoff_i > 0.0, "van_hoff_i must be positive");
    kb * molality * van_hoff_i
}

/// Calculate number-average molecular weight from pi/c intercept.
///
/// At c -> 0: pi/c = RT/Mn, so Mn = RT / (pi/c intercept)
///
/// # Arguments
/// * `pi_over_c_intercept` - Intercept of pi/c vs c plot in Pa*L/g
/// * `temperature` - Temperature in K
///
/// # Returns
/// Number-average molecular weight Mn in g/mol
pub fn number_avg_mw(pi_over_c_intercept: f64, temperature: f64) -> f64 {
    assert!(
        pi_over_c_intercept > 0.0,
        "pi_over_c intercept must be positive"
    );
    assert!(temperature > 0.0, "temperature must be positive");
    // pi/c = RT * 1000 / Mn  (factor 1000 for L->m^3)
    R_GAS * temperature * 1000.0 / pi_over_c_intercept
}

/// Calculate osmolality in mOsm/kg.
///
/// # Arguments
/// * `osmoles_per_kg` - Osmoles per kg of solvent
///
/// # Returns
/// Osmolality in mOsm/kg
pub fn osmolality(osmoles_per_kg: f64) -> f64 {
    osmoles_per_kg * 1000.0
}

/// Calculate osmolality from molality and van't Hoff factor.
///
/// osmolality = phi * i * m (in Osm/kg), returned as mOsm/kg
///
/// # Arguments
/// * `molality` - Molality in mol/kg
/// * `van_hoff_i` - van't Hoff factor
/// * `osmotic_coefficient` - Osmotic coefficient phi (1.0 for ideal)
pub fn osmolality_from_molality(
    molality: f64,
    van_hoff_i: f64,
    osmotic_coefficient: f64,
) -> f64 {
    osmotic_coefficient * van_hoff_i * molality * 1000.0
}

/// Calculate Raoult's law vapor pressure lowering.
///
/// Delta_P = P0 * x_solute  (for ideal dilute solutions)
///
/// # Arguments
/// * `p0` - Pure solvent vapor pressure (any units)
/// * `x_solute` - Mole fraction of solute
///
/// # Returns
/// Vapor pressure lowering in same units as p0
pub fn raoult_vapor_pressure_lowering(p0: f64, x_solute: f64) -> f64 {
    assert!(
        (0.0..=1.0).contains(&x_solute),
        "mole fraction must be in [0, 1]"
    );
    p0 * x_solute
}

/// Convert mass concentration (g/L) to molality (mol/kg solvent).
///
/// Assumes dilute solution where solvent density is approximately known.
pub fn concentration_to_molality(
    conc_g_per_l: f64,
    molar_mass: f64,
    solvent_density_kg_per_l: f64,
) -> f64 {
    assert!(molar_mass > 0.0, "molar_mass must be positive");
    assert!(
        solvent_density_kg_per_l > 0.0,
        "solvent density must be positive"
    );
    // n = conc / M (mol/L)
    // solvent mass per L = density (kg/L), approx ignoring solute volume for dilute
    (conc_g_per_l / molar_mass) / solvent_density_kg_per_l
}

/// Convert molality (mol/kg) to mass concentration (g/L).
pub fn molality_to_concentration(
    molality: f64,
    molar_mass: f64,
    solvent_density_kg_per_l: f64,
) -> f64 {
    molality * molar_mass * solvent_density_kg_per_l
}

/// Calculate mole fraction of solute from molality.
///
/// x_solute = m * M_solvent / (1000 + m * M_solvent)
/// where M_solvent is in g/mol and molality m is in mol/kg.
pub fn molality_to_mole_fraction(molality: f64, solvent_molar_mass: f64) -> f64 {
    let numerator = molality * solvent_molar_mass / 1000.0;
    numerator / (1.0 + numerator)
}

// ---------------------------------------------------------------------------
// SolventConstants
// ---------------------------------------------------------------------------

/// Cryoscopic and ebullioscopic constants for a solvent.
#[derive(Debug, Clone, Copy)]
pub struct SolventConstants {
    /// Solvent name
    pub name: &'static str,
    /// Cryoscopic constant Kf in K*kg/mol
    pub kf: f64,
    /// Ebullioscopic constant Kb in K*kg/mol
    pub kb: f64,
    /// Freezing point in degrees Celsius
    pub fp_celsius: f64,
    /// Boiling point in degrees Celsius
    pub bp_celsius: f64,
    /// Molar mass in g/mol
    pub molar_mass: f64,
}

impl SolventConstants {
    /// Water constants.
    pub fn water() -> Self {
        Self {
            name: "Water",
            kf: WATER_KF,
            kb: WATER_KB,
            fp_celsius: 0.0,
            bp_celsius: 100.0,
            molar_mass: WATER_MOLAR_MASS,
        }
    }

    /// Benzene constants.
    pub fn benzene() -> Self {
        Self {
            name: "Benzene",
            kf: BENZENE_KF,
            kb: BENZENE_KB,
            fp_celsius: 5.5,
            bp_celsius: 80.1,
            molar_mass: 78.11,
        }
    }

    /// Camphor constants.
    pub fn camphor() -> Self {
        Self {
            name: "Camphor",
            kf: CAMPHOR_KF,
            kb: 5.61,
            fp_celsius: 179.8,
            bp_celsius: 204.0,
            molar_mass: 152.23,
        }
    }

    /// Cyclohexane constants.
    pub fn cyclohexane() -> Self {
        Self {
            name: "Cyclohexane",
            kf: CYCLOHEXANE_KF,
            kb: 2.79,
            fp_celsius: 6.6,
            bp_celsius: 80.7,
            molar_mass: 84.16,
        }
    }

    /// Freezing point in Kelvin.
    pub fn fp_kelvin(&self) -> f64 {
        self.fp_celsius + 273.15
    }

    /// Boiling point in Kelvin.
    pub fn bp_kelvin(&self) -> f64 {
        self.bp_celsius + 273.15
    }
}

// ---------------------------------------------------------------------------
// MembraneOsmometer
// ---------------------------------------------------------------------------

/// Membrane osmometry for number-average molecular weight determination.
///
/// Measures osmotic pressure pi at multiple concentrations and extrapolates
/// pi/c vs c to c=0 to obtain Mn via the van't Hoff equation.
///
/// The virial expansion is:
///   pi/c = RT/Mn * (1 + A2*Mn*c + A3*Mn*c^2 + ...)
///
/// So a plot of pi/c vs c has intercept RT/Mn and slope RT*A2.
#[derive(Debug, Clone)]
pub struct MembraneOsmometer {
    /// Temperature in Kelvin
    temperature: f64,
    /// Concentrations in g/L
    concentrations: Vec<f64>,
    /// Measured osmotic pressures in Pa
    pressures: Vec<f64>,
}

/// Result from membrane osmometry analysis.
#[derive(Debug, Clone)]
pub struct MembraneOsmometryResult {
    /// Number-average molecular weight in g/mol
    pub mn: f64,
    /// Second virial coefficient A2 in mol*mL/g^2
    pub a2: f64,
    /// Intercept of pi/c vs c in Pa*L/g
    pub intercept: f64,
    /// Slope of pi/c vs c
    pub slope: f64,
    /// R-squared of the linear fit
    pub r_squared: f64,
}

impl MembraneOsmometer {
    /// Create a new membrane osmometer at given temperature (K).
    pub fn new(temperature: f64) -> Self {
        assert!(temperature > 0.0, "temperature must be positive");
        Self {
            temperature,
            concentrations: Vec::new(),
            pressures: Vec::new(),
        }
    }

    /// Add a measurement point (concentration in g/L, pressure in Pa).
    pub fn add_measurement(&mut self, concentration: f64, pressure: f64) {
        assert!(concentration > 0.0, "concentration must be positive");
        assert!(pressure > 0.0, "pressure must be positive");
        self.concentrations.push(concentration);
        self.pressures.push(pressure);
    }

    /// Add multiple measurement points.
    pub fn add_measurements(&mut self, concentrations: &[f64], pressures: &[f64]) {
        assert_eq!(
            concentrations.len(),
            pressures.len(),
            "concentrations and pressures must have the same length"
        );
        for (&c, &p) in concentrations.iter().zip(pressures.iter()) {
            self.add_measurement(c, p);
        }
    }

    /// Number of measurements.
    pub fn num_measurements(&self) -> usize {
        self.concentrations.len()
    }

    /// Calculate pi/c values.
    pub fn pi_over_c(&self) -> Vec<f64> {
        self.concentrations
            .iter()
            .zip(self.pressures.iter())
            .map(|(c, p)| p / c)
            .collect()
    }

    /// Analyze the data: extrapolate pi/c vs c to get Mn and A2.
    ///
    /// Requires at least 2 data points.
    pub fn analyze(&self) -> MembraneOsmometryResult {
        assert!(
            self.concentrations.len() >= 2,
            "need at least 2 data points"
        );

        let pi_c: Vec<f64> = self.pi_over_c();
        let (slope, intercept, r_squared) =
            linear_regression(&self.concentrations, &pi_c);

        // intercept = RT * 1000 / Mn => Mn = RT * 1000 / intercept
        let mn = if intercept > 0.0 {
            R_GAS * self.temperature * 1000.0 / intercept
        } else {
            f64::NAN
        };

        // slope = RT * A2 * 1000 => A2 = slope / (RT * 1000)
        // A2 in mol*L/g^2 => convert to mol*mL/g^2 by *1000
        let a2 = slope / (R_GAS * self.temperature);

        MembraneOsmometryResult {
            mn,
            a2,
            intercept,
            slope,
            r_squared,
        }
    }

    /// Predict osmotic pressure at a given concentration using fitted parameters.
    pub fn predict_pressure(&self, concentration: f64) -> f64 {
        let result = self.analyze();
        (result.intercept + result.slope * concentration) * concentration
    }

    /// Get the temperature in Kelvin.
    pub fn temperature(&self) -> f64 {
        self.temperature
    }
}

// ---------------------------------------------------------------------------
// VaporPressureOsmometer
// ---------------------------------------------------------------------------

/// Vapor pressure osmometry (VPO) for molecular weight determination of
/// small molecules (typically < 20,000 g/mol).
///
/// Measures thermoelectric response (voltage) proportional to vapor pressure
/// lowering, calibrated against a known standard.
#[derive(Debug, Clone)]
pub struct VaporPressureOsmometer {
    /// Temperature in Kelvin
    temperature: f64,
    /// Calibration constant K_cal (V*kg/mol) from standard
    calibration_constant: f64,
    /// Solvent constants
    solvent: SolventConstants,
}

/// VPO measurement result.
#[derive(Debug, Clone)]
pub struct VpoResult {
    /// Measured molecular weight in g/mol
    pub molar_mass: f64,
    /// Osmolality in mOsm/kg
    pub osmolality: f64,
    /// Vapor pressure lowering (relative)
    pub delta_p_relative: f64,
}

impl VaporPressureOsmometer {
    /// Create a new VPO with given temperature, calibration constant, and solvent.
    pub fn new(temperature: f64, calibration_constant: f64, solvent: SolventConstants) -> Self {
        assert!(temperature > 0.0, "temperature must be positive");
        assert!(
            calibration_constant > 0.0,
            "calibration_constant must be positive"
        );
        Self {
            temperature,
            calibration_constant,
            solvent,
        }
    }

    /// Create VPO for water at 37 C with default calibration.
    pub fn water_37c() -> Self {
        Self {
            temperature: 310.15,
            calibration_constant: 1.0,
            solvent: SolventConstants::water(),
        }
    }

    /// Calibrate the instrument using a standard of known molar mass.
    ///
    /// # Arguments
    /// * `standard_conc` - Standard concentration in g/L
    /// * `standard_mw` - Standard molar mass in g/mol
    /// * `voltage` - Measured voltage for the standard
    pub fn calibrate(
        &mut self,
        standard_conc: f64,
        standard_mw: f64,
        voltage: f64,
    ) {
        assert!(standard_conc > 0.0);
        assert!(standard_mw > 0.0);
        assert!(voltage > 0.0);
        // voltage = K_cal * (c / M) => K_cal = voltage * M / c
        self.calibration_constant = voltage * standard_mw / standard_conc;
    }

    /// Measure molecular weight from a voltage reading and concentration.
    ///
    /// # Arguments
    /// * `concentration` - Sample concentration in g/L
    /// * `voltage` - Measured thermoelectric voltage
    pub fn measure(&self, concentration: f64, voltage: f64) -> VpoResult {
        assert!(concentration > 0.0, "concentration must be positive");
        assert!(voltage > 0.0, "voltage must be positive");

        // voltage = K_cal * (c / M) => M = K_cal * c / voltage... wait
        // Actually: voltage = K_cal * molality
        // molality ~ c / (M * rho_solvent) for dilute
        // So M = K_cal * c / (voltage * rho_solvent * M_solvent / 1000)
        // Simplified: M = K_cal * c / voltage
        let molar_mass = self.calibration_constant * concentration / voltage;

        // Osmolality
        let molality = concentration_to_molality(
            concentration,
            molar_mass,
            WATER_DENSITY,
        );
        let osm = osmolality(molality);

        // Raoult's law relative vapor pressure lowering
        let x_solute = molality_to_mole_fraction(molality, self.solvent.molar_mass);
        let delta_p_rel = x_solute;

        VpoResult {
            molar_mass,
            osmolality: osm,
            delta_p_relative: delta_p_rel,
        }
    }

    /// Get calibration constant.
    pub fn calibration_constant(&self) -> f64 {
        self.calibration_constant
    }

    /// Get the temperature.
    pub fn temperature(&self) -> f64 {
        self.temperature
    }
}

// ---------------------------------------------------------------------------
// CryoscopicAnalyzer
// ---------------------------------------------------------------------------

/// Freezing point depression analyzer (cryoscopy).
///
/// Determines molecular weight or solution properties from the depression
/// of the freezing point: Delta_Tf = Kf * m * i
#[derive(Debug, Clone)]
pub struct CryoscopicAnalyzer {
    /// Solvent constants
    solvent: SolventConstants,
}

/// Result of cryoscopic analysis.
#[derive(Debug, Clone)]
pub struct CryoscopicResult {
    /// Freezing point depression in K
    pub delta_tf: f64,
    /// New freezing point in Celsius
    pub new_fp_celsius: f64,
    /// Molality in mol/kg
    pub molality: f64,
    /// Estimated molar mass in g/mol (if unknown)
    pub estimated_mw: Option<f64>,
}

impl CryoscopicAnalyzer {
    /// Create a new cryoscopic analyzer for the given solvent.
    pub fn new(solvent: SolventConstants) -> Self {
        Self { solvent }
    }

    /// Create analyzer for water.
    pub fn water() -> Self {
        Self::new(SolventConstants::water())
    }

    /// Create analyzer for benzene.
    pub fn benzene() -> Self {
        Self::new(SolventConstants::benzene())
    }

    /// Create analyzer for camphor.
    pub fn camphor() -> Self {
        Self::new(SolventConstants::camphor())
    }

    /// Calculate freezing point depression for known molality and i factor.
    pub fn depression(&self, molality: f64, van_hoff_i: f64) -> CryoscopicResult {
        let delta_tf = freezing_point_depression(self.solvent.kf, molality, van_hoff_i);
        CryoscopicResult {
            delta_tf,
            new_fp_celsius: self.solvent.fp_celsius - delta_tf,
            molality,
            estimated_mw: None,
        }
    }

    /// Determine molecular weight from measured freezing point depression.
    ///
    /// # Arguments
    /// * `delta_tf` - Measured freezing point depression in K
    /// * `mass_solute_g` - Mass of solute in grams
    /// * `mass_solvent_kg` - Mass of solvent in kg
    /// * `van_hoff_i` - van't Hoff factor
    pub fn determine_mw(
        &self,
        delta_tf: f64,
        mass_solute_g: f64,
        mass_solvent_kg: f64,
        van_hoff_i: f64,
    ) -> CryoscopicResult {
        assert!(delta_tf > 0.0, "delta_tf must be positive");
        assert!(mass_solute_g > 0.0);
        assert!(mass_solvent_kg > 0.0);
        assert!(van_hoff_i > 0.0);

        // Delta_Tf = Kf * m * i = Kf * (mass_solute / (MW * mass_solvent)) * i
        // MW = Kf * i * mass_solute / (delta_tf * mass_solvent)
        let mw = self.solvent.kf * van_hoff_i * mass_solute_g
            / (delta_tf * mass_solvent_kg);
        let molality = mass_solute_g / (mw * mass_solvent_kg);

        CryoscopicResult {
            delta_tf,
            new_fp_celsius: self.solvent.fp_celsius - delta_tf,
            molality,
            estimated_mw: Some(mw),
        }
    }

    /// Calculate molality from measured freezing point depression.
    pub fn molality_from_depression(&self, delta_tf: f64, van_hoff_i: f64) -> f64 {
        assert!(delta_tf > 0.0);
        assert!(van_hoff_i > 0.0);
        delta_tf / (self.solvent.kf * van_hoff_i)
    }

    /// Get the solvent Kf.
    pub fn kf(&self) -> f64 {
        self.solvent.kf
    }

    /// Get the solvent reference.
    pub fn solvent(&self) -> &SolventConstants {
        &self.solvent
    }
}

// ---------------------------------------------------------------------------
// EbullioscopicAnalyzer
// ---------------------------------------------------------------------------

/// Boiling point elevation analyzer (ebulliometry).
///
/// Determines molecular weight or solution properties from the elevation
/// of the boiling point: Delta_Tb = Kb * m * i
#[derive(Debug, Clone)]
pub struct EbullioscopicAnalyzer {
    /// Solvent constants
    solvent: SolventConstants,
}

/// Result of ebullioscopic analysis.
#[derive(Debug, Clone)]
pub struct EbullioscopicResult {
    /// Boiling point elevation in K
    pub delta_tb: f64,
    /// New boiling point in Celsius
    pub new_bp_celsius: f64,
    /// Molality in mol/kg
    pub molality: f64,
    /// Estimated molar mass in g/mol (if unknown)
    pub estimated_mw: Option<f64>,
}

impl EbullioscopicAnalyzer {
    /// Create a new ebullioscopic analyzer for the given solvent.
    pub fn new(solvent: SolventConstants) -> Self {
        Self { solvent }
    }

    /// Create analyzer for water.
    pub fn water() -> Self {
        Self::new(SolventConstants::water())
    }

    /// Calculate boiling point elevation for known molality and i factor.
    pub fn elevation(&self, molality: f64, van_hoff_i: f64) -> EbullioscopicResult {
        let delta_tb = boiling_point_elevation(self.solvent.kb, molality, van_hoff_i);
        EbullioscopicResult {
            delta_tb,
            new_bp_celsius: self.solvent.bp_celsius + delta_tb,
            molality,
            estimated_mw: None,
        }
    }

    /// Determine molecular weight from measured boiling point elevation.
    pub fn determine_mw(
        &self,
        delta_tb: f64,
        mass_solute_g: f64,
        mass_solvent_kg: f64,
        van_hoff_i: f64,
    ) -> EbullioscopicResult {
        assert!(delta_tb > 0.0, "delta_tb must be positive");
        assert!(mass_solute_g > 0.0);
        assert!(mass_solvent_kg > 0.0);
        assert!(van_hoff_i > 0.0);

        let mw = self.solvent.kb * van_hoff_i * mass_solute_g
            / (delta_tb * mass_solvent_kg);
        let molality = mass_solute_g / (mw * mass_solvent_kg);

        EbullioscopicResult {
            delta_tb,
            new_bp_celsius: self.solvent.bp_celsius + delta_tb,
            molality,
            estimated_mw: Some(mw),
        }
    }

    /// Calculate molality from measured boiling point elevation.
    pub fn molality_from_elevation(&self, delta_tb: f64, van_hoff_i: f64) -> f64 {
        assert!(delta_tb > 0.0);
        assert!(van_hoff_i > 0.0);
        delta_tb / (self.solvent.kb * van_hoff_i)
    }

    /// Get the solvent Kb.
    pub fn kb(&self) -> f64 {
        self.solvent.kb
    }

    /// Get the solvent reference.
    pub fn solvent(&self) -> &SolventConstants {
        &self.solvent
    }
}

// ---------------------------------------------------------------------------
// VantHoffAnalyzer
// ---------------------------------------------------------------------------

/// Van't Hoff analysis for osmotic pressure with virial expansion.
///
/// pi = cRT (1/Mn + A2*c + A3*c^2 + ...)
///
/// Fits pi/c vs c data to polynomial to extract Mn, A2, and optionally A3.
#[derive(Debug, Clone)]
pub struct VantHoffAnalyzer {
    /// Temperature in Kelvin
    temperature: f64,
    /// Concentrations in g/L
    concentrations: Vec<f64>,
    /// Measured pi/c values in Pa*L/g
    pi_over_c: Vec<f64>,
}

/// Result from van't Hoff virial analysis.
#[derive(Debug, Clone)]
pub struct VantHoffResult {
    /// Number-average molecular weight in g/mol
    pub mn: f64,
    /// Second virial coefficient A2 in mol*mL/g^2
    pub a2: f64,
    /// Third virial coefficient A3 (from quadratic fit, may be None)
    pub a3: Option<f64>,
    /// Linear R-squared
    pub r_squared: f64,
}

impl VantHoffAnalyzer {
    /// Create a new van't Hoff analyzer at given temperature (K).
    pub fn new(temperature: f64) -> Self {
        assert!(temperature > 0.0, "temperature must be positive");
        Self {
            temperature,
            concentrations: Vec::new(),
            pi_over_c: Vec::new(),
        }
    }

    /// Add a data point (concentration in g/L, pi/c in Pa*L/g).
    pub fn add_point(&mut self, concentration: f64, pi_over_c: f64) {
        assert!(concentration > 0.0);
        assert!(pi_over_c > 0.0);
        self.concentrations.push(concentration);
        self.pi_over_c.push(pi_over_c);
    }

    /// Add data from concentration and pressure arrays.
    pub fn add_from_pressures(&mut self, concentrations: &[f64], pressures: &[f64]) {
        assert_eq!(concentrations.len(), pressures.len());
        for (&c, &p) in concentrations.iter().zip(pressures.iter()) {
            self.add_point(c, p / c);
        }
    }

    /// Perform linear virial analysis (1st order: pi/c = a + b*c).
    pub fn analyze_linear(&self) -> VantHoffResult {
        assert!(self.concentrations.len() >= 2, "need at least 2 points");

        let (slope, intercept, r_squared) =
            linear_regression(&self.concentrations, &self.pi_over_c);

        let rt_1000 = R_GAS * self.temperature * 1000.0;
        let mn = if intercept > 0.0 {
            rt_1000 / intercept
        } else {
            f64::NAN
        };

        // slope = RT * 1000 * A2  => A2 = slope / (RT * 1000)
        let a2 = slope / rt_1000;

        VantHoffResult {
            mn,
            a2,
            a3: None,
            r_squared,
        }
    }

    /// Perform quadratic virial analysis (2nd order: pi/c = a + b*c + d*c^2).
    ///
    /// Uses a simple least-squares quadratic fit.
    pub fn analyze_quadratic(&self) -> VantHoffResult {
        let n = self.concentrations.len();
        assert!(n >= 3, "need at least 3 points for quadratic fit");

        // Solve normal equations for y = a + b*x + d*x^2
        // Using Cramer's rule for 3x3 system
        let mut s = [0.0f64; 5]; // sum of x^0..x^4
        let mut t = [0.0f64; 3]; // sum of y*x^0..y*x^2

        for i in 0..n {
            let x = self.concentrations[i];
            let y = self.pi_over_c[i];
            let mut xp = 1.0;
            for j in 0..5 {
                s[j] += xp;
                xp *= x;
            }
            t[0] += y;
            t[1] += y * x;
            t[2] += y * x * x;
        }

        // Normal equations:
        // [s0 s1 s2] [a]   [t0]
        // [s1 s2 s3] [b] = [t1]
        // [s2 s3 s4] [d]   [t2]
        let det = s[0] * (s[2] * s[4] - s[3] * s[3])
            - s[1] * (s[1] * s[4] - s[3] * s[2])
            + s[2] * (s[1] * s[3] - s[2] * s[2]);

        if det.abs() < 1e-30 {
            return self.analyze_linear();
        }

        let a = (t[0] * (s[2] * s[4] - s[3] * s[3])
            - s[1] * (t[1] * s[4] - s[3] * t[2])
            + s[2] * (t[1] * s[3] - s[2] * t[2]))
            / det;

        let b = (s[0] * (t[1] * s[4] - s[3] * t[2])
            - t[0] * (s[1] * s[4] - s[3] * s[2])
            + s[2] * (s[1] * t[2] - t[1] * s[2]))
            / det;

        let d = (s[0] * (s[2] * t[2] - t[1] * s[3])
            - s[1] * (s[1] * t[2] - t[1] * s[2])
            + t[0] * (s[1] * s[3] - s[2] * s[2]))
            / det;

        let rt_1000 = R_GAS * self.temperature * 1000.0;
        let mn = if a > 0.0 { rt_1000 / a } else { f64::NAN };
        let a2 = b / rt_1000;
        let a3 = Some(d / rt_1000);

        // Calculate R-squared for quadratic
        let y_mean: f64 = self.pi_over_c.iter().sum::<f64>() / n as f64;
        let ss_tot: f64 = self.pi_over_c.iter().map(|y| (y - y_mean).powi(2)).sum();
        let ss_res: f64 = self
            .concentrations
            .iter()
            .zip(self.pi_over_c.iter())
            .map(|(x, y)| {
                let pred = a + b * x + d * x * x;
                (y - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot.abs() < 1e-30 {
            1.0
        } else {
            1.0 - ss_res / ss_tot
        };

        VantHoffResult {
            mn,
            a2,
            a3,
            r_squared,
        }
    }

    /// Get temperature.
    pub fn temperature(&self) -> f64 {
        self.temperature
    }

    /// Get number of data points.
    pub fn num_points(&self) -> usize {
        self.concentrations.len()
    }
}

// ---------------------------------------------------------------------------
// SecondVirialCoefficient
// ---------------------------------------------------------------------------

/// Determines the second virial coefficient A2 from osmotic pressure data.
///
/// A positive A2 indicates good solvent conditions (polymer-solvent interactions
/// dominate). A negative A2 indicates poor solvent. A2 = 0 at the theta
/// temperature.
#[derive(Debug, Clone)]
pub struct SecondVirialCoefficient {
    /// Temperature in Kelvin
    temperature: f64,
    /// Concentration values in g/L
    concentrations: Vec<f64>,
    /// Corresponding pi/c values in Pa*L/g
    pi_over_c_values: Vec<f64>,
}

/// Result from A2 determination.
#[derive(Debug, Clone)]
pub struct A2Result {
    /// Second virial coefficient in mol*mL/g^2
    pub a2: f64,
    /// Number-average molecular weight from intercept
    pub mn: f64,
    /// R-squared of the linear fit
    pub r_squared: f64,
    /// Solvent quality assessment
    pub solvent_quality: SolventQuality,
}

/// Classification of solvent quality from A2 sign.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SolventQuality {
    /// A2 > 0: good solvent (chains expanded)
    Good,
    /// A2 = 0: theta solvent (chains ideal)
    Theta,
    /// A2 < 0: poor solvent (chains collapsed)
    Poor,
}

impl SecondVirialCoefficient {
    /// Create a new A2 analyzer at given temperature.
    pub fn new(temperature: f64) -> Self {
        assert!(temperature > 0.0);
        Self {
            temperature,
            concentrations: Vec::new(),
            pi_over_c_values: Vec::new(),
        }
    }

    /// Add a data point (concentration in g/L, pi/c in Pa*L/g).
    pub fn add_point(&mut self, concentration: f64, pi_over_c: f64) {
        self.concentrations.push(concentration);
        self.pi_over_c_values.push(pi_over_c);
    }

    /// Add data from raw pressure measurements.
    pub fn add_from_pressures(&mut self, concentrations: &[f64], pressures: &[f64]) {
        assert_eq!(concentrations.len(), pressures.len());
        for (&c, &p) in concentrations.iter().zip(pressures.iter()) {
            self.add_point(c, p / c);
        }
    }

    /// Determine A2 via linear extrapolation of pi/c vs c.
    pub fn determine(&self) -> A2Result {
        assert!(
            self.concentrations.len() >= 2,
            "need at least 2 data points"
        );

        let (slope, intercept, r_squared) =
            linear_regression(&self.concentrations, &self.pi_over_c_values);

        let rt_1000 = R_GAS * self.temperature * 1000.0;
        let mn = if intercept > 0.0 {
            rt_1000 / intercept
        } else {
            f64::NAN
        };
        let a2 = slope / rt_1000;

        let solvent_quality = if a2.abs() < 1e-10 {
            SolventQuality::Theta
        } else if a2 > 0.0 {
            SolventQuality::Good
        } else {
            SolventQuality::Poor
        };

        A2Result {
            a2,
            mn,
            r_squared,
            solvent_quality,
        }
    }

    /// Predict whether a theta temperature exists between two measurements.
    ///
    /// Returns estimated theta temperature from linear interpolation of A2
    /// vs 1/T data from two different temperatures. Requires A2 values at
    /// two different temperatures.
    pub fn estimate_theta_temperature(
        a2_t1: f64,
        t1: f64,
        a2_t2: f64,
        t2: f64,
    ) -> Option<f64> {
        if a2_t1.signum() == a2_t2.signum() {
            return None; // No sign change, can't interpolate theta
        }
        // Linear interpolation: A2 = 0 at T_theta
        // a2_t1 + (a2_t2 - a2_t1) * (T_theta - t1) / (t2 - t1) = 0
        let t_theta = t1 - a2_t1 * (t2 - t1) / (a2_t2 - a2_t1);
        if t_theta > 0.0 {
            Some(t_theta)
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// IsotonicityCalculator
// ---------------------------------------------------------------------------

/// Pharmaceutical isotonicity calculator.
///
/// Calculates osmolality/osmolarity for drug solutions and determines
/// the amount of NaCl (or other tonicity agent) needed for isotonicity
/// (290 mOsm/kg for blood).
#[derive(Debug, Clone)]
pub struct IsotonicityCalculator {
    /// Target osmolality in mOsm/kg
    target_osmolality: f64,
}

/// Drug or solute contribution to isotonicity.
#[derive(Debug, Clone)]
pub struct SoluteContribution {
    /// Name of the solute
    pub name: String,
    /// Concentration in g/L
    pub concentration: f64,
    /// Molar mass in g/mol
    pub molar_mass: f64,
    /// Van't Hoff dissociation factor
    pub van_hoff_i: f64,
    /// Osmotic coefficient (deviation from ideal)
    pub osmotic_coefficient: f64,
}

/// Result of isotonicity calculation.
#[derive(Debug, Clone)]
pub struct IsotonicityResult {
    /// Total osmolality from all solutes in mOsm/kg
    pub total_osmolality: f64,
    /// Whether the solution is isotonic
    pub is_isotonic: bool,
    /// Whether the solution is hypertonic
    pub is_hypertonic: bool,
    /// Whether the solution is hypotonic
    pub is_hypotonic: bool,
    /// NaCl equivalent concentration needed to make isotonic (g/L, 0 if already isotonic)
    pub nacl_to_add_g_per_l: f64,
    /// Osmolality contribution from each solute
    pub contributions: Vec<f64>,
}

impl SoluteContribution {
    /// Create a new solute contribution.
    pub fn new(
        name: &str,
        concentration: f64,
        molar_mass: f64,
        van_hoff_i: f64,
    ) -> Self {
        Self {
            name: name.to_string(),
            concentration,
            molar_mass,
            van_hoff_i,
            osmotic_coefficient: 1.0,
        }
    }

    /// Set the osmotic coefficient.
    pub fn with_osmotic_coefficient(mut self, phi: f64) -> Self {
        self.osmotic_coefficient = phi;
        self
    }

    /// Calculate osmolality contribution in mOsm/kg.
    pub fn osmolality_contribution(&self) -> f64 {
        let molarity = self.concentration / self.molar_mass; // mol/L
        // For dilute aqueous, molarity ~ molality
        let molality = molarity / WATER_DENSITY;
        self.osmotic_coefficient * self.van_hoff_i * molality * 1000.0
    }

    /// Calculate NaCl equivalent (E value).
    ///
    /// E = (58.44 * i_drug) / (i_NaCl * MW_drug) = 58.44 * i_drug / (1.8 * MW_drug)
    pub fn nacl_equivalent(&self) -> f64 {
        let nacl_i = 1.8; // NaCl van't Hoff factor (slightly less than 2 due to ion pairing)
        NACL_MOLAR_MASS * self.van_hoff_i / (nacl_i * self.molar_mass)
    }
}

impl IsotonicityCalculator {
    /// Create a new isotonicity calculator with blood-equivalent target (290 mOsm/kg).
    pub fn new() -> Self {
        Self {
            target_osmolality: 290.0,
        }
    }

    /// Create with a custom target osmolality.
    pub fn with_target(target_osmolality: f64) -> Self {
        Self { target_osmolality }
    }

    /// Get the target osmolality.
    pub fn target_osmolality(&self) -> f64 {
        self.target_osmolality
    }

    /// Calculate isotonicity for a set of solutes.
    pub fn calculate(&self, solutes: &[SoluteContribution]) -> IsotonicityResult {
        let contributions: Vec<f64> = solutes
            .iter()
            .map(|s| s.osmolality_contribution())
            .collect();
        let total_osmolality: f64 = contributions.iter().sum();

        let tolerance = 10.0; // mOsm/kg tolerance for isotonicity
        let is_isotonic = (total_osmolality - self.target_osmolality).abs() <= tolerance;
        let is_hypertonic = total_osmolality > self.target_osmolality + tolerance;
        let is_hypotonic = total_osmolality < self.target_osmolality - tolerance;

        // NaCl needed to reach target
        let nacl_to_add_g_per_l = if total_osmolality < self.target_osmolality - tolerance {
            let deficit_mosm = self.target_osmolality - total_osmolality;
            // NaCl: n_ions=2, phi=0.93
            let nacl_phi = 0.93;
            let n_ions = 2.0;
            // deficit = phi * n_ions * molality * 1000
            // molality = deficit / (phi * n_ions * 1000)
            let molality_needed = deficit_mosm / (nacl_phi * n_ions * 1000.0);
            // Convert molality to g/L: g/L = molality * MW * rho_solvent
            molality_needed * NACL_MOLAR_MASS * WATER_DENSITY
        } else {
            0.0
        };

        IsotonicityResult {
            total_osmolality,
            is_isotonic,
            is_hypertonic,
            is_hypotonic,
            nacl_to_add_g_per_l,
            contributions,
        }
    }

    /// Check if a given NaCl percent w/v concentration is isotonic.
    pub fn is_nacl_isotonic(percent_wv: f64) -> bool {
        let osm = nacl_osmolality(percent_wv);
        (osm - 290.0).abs() < 20.0
    }

    /// Calculate the NaCl percentage that produces the target osmolality.
    pub fn isotonic_nacl_percent(&self) -> f64 {
        nacl_percent_for_osmolality(self.target_osmolality)
    }
}

// ---------------------------------------------------------------------------
// OsmoticCoefficient
// ---------------------------------------------------------------------------

/// Osmotic coefficient calculator for non-ideal solution corrections.
///
/// The osmotic coefficient phi relates real osmotic pressure to ideal:
///   pi_real = phi * pi_ideal
///
/// For electrolytes, phi deviates from 1.0 due to ion-ion interactions
/// and can be estimated from the Debye-Huckel theory at low concentrations.
#[derive(Debug, Clone)]
pub struct OsmoticCoefficient {
    /// Reference data: molality values
    molalities: Vec<f64>,
    /// Reference data: corresponding phi values
    phi_values: Vec<f64>,
}

impl OsmoticCoefficient {
    /// Create a new osmotic coefficient calculator from reference data.
    pub fn new(molalities: Vec<f64>, phi_values: Vec<f64>) -> Self {
        assert_eq!(
            molalities.len(),
            phi_values.len(),
            "molalities and phi_values must have the same length"
        );
        assert!(!molalities.is_empty(), "need at least one data point");
        Self {
            molalities,
            phi_values,
        }
    }

    /// Create an ideal solution (phi = 1.0 everywhere).
    pub fn ideal() -> Self {
        Self {
            molalities: vec![0.0, 10.0],
            phi_values: vec![1.0, 1.0],
        }
    }

    /// Approximate NaCl osmotic coefficient using a simplified model.
    ///
    /// phi_NaCl ~ 0.93 for typical physiological concentrations.
    /// Uses a simplified Pitzer equation fit.
    pub fn nacl_approximate() -> Self {
        // Simplified NaCl phi data from 0 to 6 mol/kg
        Self {
            molalities: vec![0.0, 0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            phi_values: vec![1.0, 0.932, 0.921, 0.936, 0.984, 1.045, 1.116, 1.191, 1.270],
        }
    }

    /// Interpolate phi at a given molality using linear interpolation.
    pub fn phi_at(&self, molality: f64) -> f64 {
        if molality <= self.molalities[0] {
            return self.phi_values[0];
        }
        let last = self.molalities.len() - 1;
        if molality >= self.molalities[last] {
            return self.phi_values[last];
        }

        // Find bracketing interval
        for i in 0..last {
            if molality >= self.molalities[i] && molality <= self.molalities[i + 1] {
                let frac = (molality - self.molalities[i])
                    / (self.molalities[i + 1] - self.molalities[i]);
                return self.phi_values[i]
                    + frac * (self.phi_values[i + 1] - self.phi_values[i]);
            }
        }
        self.phi_values[last]
    }

    /// Calculate the Debye-Huckel limiting law osmotic coefficient.
    ///
    /// phi = 1 - (A_phi / 3) * |z+ * z-| * sqrt(I)
    ///
    /// where A_phi = 0.3915 for water at 25 C, I = ionic strength.
    pub fn debye_huckel_phi(ionic_strength: f64, z_plus: f64, z_minus: f64) -> f64 {
        let a_phi = 0.3915; // Water at 25 C
        1.0 - (a_phi / 3.0) * (z_plus * z_minus).abs() * ionic_strength.sqrt()
    }

    /// Calculate ionic strength from molality for a 1:1 electrolyte.
    pub fn ionic_strength_11(molality: f64) -> f64 {
        // I = 0.5 * sum(m_i * z_i^2) = 0.5 * (m * 1 + m * 1) = m
        molality
    }

    /// Calculate ionic strength from molality for a z+:z- electrolyte.
    pub fn ionic_strength(molality: f64, z_plus: f64, z_minus: f64, nu_plus: f64, nu_minus: f64) -> f64 {
        0.5 * molality * (nu_plus * z_plus * z_plus + nu_minus * z_minus * z_minus)
    }

    /// Get the number of reference data points.
    pub fn num_points(&self) -> usize {
        self.molalities.len()
    }
}

// ---------------------------------------------------------------------------
// ColligativePropertySuite - convenience for running all analyses
// ---------------------------------------------------------------------------

/// Comprehensive colligative property analysis suite.
///
/// Runs freezing point depression, boiling point elevation, and osmotic
/// pressure calculations simultaneously for a given solution.
#[derive(Debug, Clone)]
pub struct ColligativePropertySuite {
    /// Solvent constants
    solvent: SolventConstants,
    /// Temperature for osmotic pressure in K
    temperature: f64,
}

/// Combined colligative property results.
#[derive(Debug, Clone)]
pub struct ColligativeResults {
    /// Freezing point depression in K
    pub delta_tf: f64,
    /// New freezing point in Celsius
    pub new_fp_celsius: f64,
    /// Boiling point elevation in K
    pub delta_tb: f64,
    /// New boiling point in Celsius
    pub new_bp_celsius: f64,
    /// Osmotic pressure in Pa
    pub osmotic_pressure_pa: f64,
    /// Osmotic pressure in atm
    pub osmotic_pressure_atm: f64,
    /// Vapor pressure lowering (relative)
    pub vapor_pressure_lowering: f64,
    /// Osmolality in mOsm/kg
    pub osmolality_mosm: f64,
}

impl ColligativePropertySuite {
    /// Create a new suite for the given solvent and temperature.
    pub fn new(solvent: SolventConstants, temperature: f64) -> Self {
        Self {
            solvent,
            temperature,
        }
    }

    /// Create a water suite at 25 C.
    pub fn water_25c() -> Self {
        Self::new(SolventConstants::water(), 298.15)
    }

    /// Analyze colligative properties for a given molality and van't Hoff factor.
    pub fn analyze(
        &self,
        molality: f64,
        van_hoff_i: f64,
        molar_mass: f64,
    ) -> ColligativeResults {
        let delta_tf = freezing_point_depression(self.solvent.kf, molality, van_hoff_i);
        let delta_tb = boiling_point_elevation(self.solvent.kb, molality, van_hoff_i);

        // Osmotic pressure: pi = i * m * R * T * rho_solvent * 1000
        // (where m is molality, and we convert appropriately)
        let conc_g_l = molality * molar_mass * WATER_DENSITY;
        let pi_pa = osmotic_pressure_electrolyte(
            conc_g_l,
            molar_mass,
            self.temperature,
            van_hoff_i,
        );

        // Vapor pressure lowering
        let x_solute = molality_to_mole_fraction(molality, self.solvent.molar_mass);
        let vp_lowering = x_solute;

        // Osmolality
        let osm = van_hoff_i * molality * 1000.0; // mOsm/kg

        ColligativeResults {
            delta_tf,
            new_fp_celsius: self.solvent.fp_celsius - delta_tf,
            delta_tb,
            new_bp_celsius: self.solvent.bp_celsius + delta_tb,
            osmotic_pressure_pa: pi_pa,
            osmotic_pressure_atm: pi_pa / 101325.0,
            vapor_pressure_lowering: vp_lowering,
            osmolality_mosm: osm,
        }
    }
}

// ---------------------------------------------------------------------------
// Polynomial molecular weight analysis
// ---------------------------------------------------------------------------

/// Determine Mn from multiple pi measurements at different concentrations
/// using weighted regression where lower concentrations get higher weight.
pub fn weighted_mn_determination(
    concentrations: &[f64],
    pressures: &[f64],
    temperature: f64,
) -> (f64, f64, f64) {
    assert_eq!(concentrations.len(), pressures.len());
    let n = concentrations.len();
    assert!(n >= 2);

    // Weight by 1/c^2 to emphasize low-concentration points
    let pi_over_c: Vec<f64> = concentrations
        .iter()
        .zip(pressures.iter())
        .map(|(c, p)| p / c)
        .collect();

    let weights: Vec<f64> = concentrations
        .iter()
        .map(|c| 1.0 / (c * c))
        .collect();

    let w_sum: f64 = weights.iter().sum();

    // Weighted linear regression
    let w_x: f64 = weights
        .iter()
        .zip(concentrations.iter())
        .map(|(w, x)| w * x)
        .sum::<f64>()
        / w_sum;
    let w_y: f64 = weights
        .iter()
        .zip(pi_over_c.iter())
        .map(|(w, y)| w * y)
        .sum::<f64>()
        / w_sum;
    let w_xy: f64 = weights
        .iter()
        .zip(concentrations.iter().zip(pi_over_c.iter()))
        .map(|(w, (x, y))| w * x * y)
        .sum::<f64>()
        / w_sum;
    let w_xx: f64 = weights
        .iter()
        .zip(concentrations.iter())
        .map(|(w, x)| w * x * x)
        .sum::<f64>()
        / w_sum;

    let slope = (w_xy - w_x * w_y) / (w_xx - w_x * w_x);
    let intercept = w_y - slope * w_x;

    let rt_1000 = R_GAS * temperature * 1000.0;
    let mn = if intercept > 0.0 {
        rt_1000 / intercept
    } else {
        f64::NAN
    };
    let a2 = slope / rt_1000;

    // R-squared with weights
    let ss_tot: f64 = weights
        .iter()
        .zip(pi_over_c.iter())
        .map(|(w, y)| w * (y - w_y).powi(2))
        .sum();
    let ss_res: f64 = weights
        .iter()
        .zip(concentrations.iter().zip(pi_over_c.iter()))
        .map(|(w, (x, y))| {
            let pred = slope * x + intercept;
            w * (y - pred).powi(2)
        })
        .sum();
    let r_squared = if ss_tot.abs() < 1e-30 {
        1.0
    } else {
        1.0 - ss_res / ss_tot
    };

    (mn, a2, r_squared)
}

// ---------------------------------------------------------------------------
// Utility: tonicity classification
// ---------------------------------------------------------------------------

/// Classify a solution's tonicity relative to blood.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Tonicity {
    /// < 285 mOsm/kg
    Hypotonic,
    /// 285-295 mOsm/kg
    Isotonic,
    /// > 295 mOsm/kg
    Hypertonic,
}

/// Classify tonicity based on osmolality in mOsm/kg.
pub fn classify_tonicity(osmolality_mosm: f64) -> Tonicity {
    if osmolality_mosm < BLOOD_OSMOLALITY_LOW {
        Tonicity::Hypotonic
    } else if osmolality_mosm > BLOOD_OSMOLALITY_HIGH {
        Tonicity::Hypertonic
    } else {
        Tonicity::Isotonic
    }
}

/// Calculate the osmolality of a NaCl solution given w/v percent.
///
/// Uses n_ions=2 (Na+ + Cl-) and osmotic coefficient phi=0.93.
/// For 0.9% NaCl: ~286 mOsm/kg (physiological isotonic reference).
pub fn nacl_osmolality(percent_wv: f64) -> f64 {
    let conc_g_l = percent_wv * 10.0;
    let molarity = conc_g_l / NACL_MOLAR_MASS;
    let molality = molarity / WATER_DENSITY;
    // n_ions = 2 (Na+ + Cl-), phi = 0.93 (osmotic coefficient already
    // accounts for non-ideal behavior; do not also use i<2)
    let n_ions = 2.0;
    let phi = 0.93;
    phi * n_ions * molality * 1000.0
}

/// Calculate the equivalent NaCl percent for a given target osmolality.
pub fn nacl_percent_for_osmolality(target_mosm: f64) -> f64 {
    // Inverse of nacl_osmolality: osm = phi * n_ions * molality * 1000
    let n_ions = 2.0;
    let phi = 0.93;
    let molality = target_mosm / (phi * n_ions * 1000.0);
    let molarity = molality * WATER_DENSITY;
    let conc_g_l = molarity * NACL_MOLAR_MASS;
    conc_g_l / 10.0
}

// ---------------------------------------------------------------------------
// Multi-solute osmolality calculator
// ---------------------------------------------------------------------------

/// Simple osmolality calculator for a mixture of solutes.
pub fn mixture_osmolality(solutes: &[(f64, f64, f64, f64)]) -> f64 {
    // Each tuple: (concentration_g_l, molar_mass, van_hoff_i, osmotic_coeff)
    solutes
        .iter()
        .map(|(conc, mw, i, phi)| {
            let molarity = conc / mw;
            let molality = molarity / WATER_DENSITY;
            phi * i * molality * 1000.0
        })
        .sum()
}

/// Estimate van't Hoff factor from measured and theoretical freezing point depression.
///
/// i = delta_tf_measured / (Kf * m)
pub fn estimate_van_hoff_factor(kf: f64, molality: f64, delta_tf_measured: f64) -> f64 {
    assert!(kf > 0.0);
    assert!(molality > 0.0);
    delta_tf_measured / (kf * molality)
}

/// Calculate the degree of dissociation alpha from the van't Hoff factor.
///
/// For an electrolyte that dissociates into n ions:
///   i = 1 + alpha * (n - 1)
///   alpha = (i - 1) / (n - 1)
pub fn degree_of_dissociation(van_hoff_i: f64, n_ions: u32) -> f64 {
    assert!(n_ions >= 2, "n_ions must be >= 2 for electrolytes");
    let n = n_ions as f64;
    let alpha = (van_hoff_i - 1.0) / (n - 1.0);
    alpha.clamp(0.0, 1.0)
}

/// Calculate the van't Hoff factor from degree of dissociation.
///
/// i = 1 + alpha * (n - 1)
pub fn van_hoff_from_dissociation(alpha: f64, n_ions: u32) -> f64 {
    let n = n_ions as f64;
    1.0 + alpha * (n - 1.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 0.01;
    const TOL_PERCENT: f64 = 0.05; // 5% tolerance for physical calculations

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if b.abs() < 1e-30 {
            a.abs() < 1e-30
        } else {
            ((a - b) / b).abs() < rel_tol
        }
    }

    // === Linear regression tests ===

    #[test]
    fn test_linear_regression_perfect_line() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, TOL));
        assert!(approx_eq(intercept, 0.0, TOL));
        assert!(approx_eq(r2, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_with_offset() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![3.0, 5.0, 7.0, 9.0];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, TOL));
        assert!(approx_eq(intercept, 3.0, TOL));
        assert!(approx_eq(r2, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_negative_slope() {
        let x = vec![1.0, 2.0, 3.0, 4.0];
        let y = vec![10.0, 7.0, 4.0, 1.0];
        let (slope, intercept, _r2) = linear_regression(&x, &y);
        assert!(approx_eq(slope, -3.0, TOL));
        assert!(approx_eq(intercept, 13.0, TOL));
    }

    #[test]
    fn test_linear_regression_two_points() {
        let x = vec![0.0, 10.0];
        let y = vec![5.0, 15.0];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 1.0, TOL));
        assert!(approx_eq(intercept, 5.0, TOL));
        assert!(approx_eq(r2, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_fit_struct() {
        let x = vec![1.0, 2.0, 3.0];
        let y = vec![2.0, 4.0, 6.0];
        let fit = linear_regression_fit(&x, &y);
        assert!(approx_eq(fit.slope, 2.0, TOL));
        assert!(approx_eq(fit.intercept, 0.0, TOL));
        assert!(approx_eq(fit.r_squared, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_noisy() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.1, 3.9, 6.2, 7.8, 10.1];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(relative_eq(slope, 2.0, 0.1));
        assert!(r2 > 0.99);
        assert!(intercept.abs() < 0.5);
    }

    // === Osmotic pressure tests ===

    #[test]
    fn test_osmotic_pressure_glucose() {
        // 1 g/L glucose (MW 180.16) at 25 C
        let pi = osmotic_pressure(1.0, 180.16, 298.15);
        // pi = (1/180.16) * 8.314 * 298.15 * 1000 = ~13765 Pa
        assert!(pi > 13000.0 && pi < 14500.0);
    }

    #[test]
    fn test_osmotic_pressure_protein() {
        // 10 g/L BSA (MW 66430) at 25 C
        let pi = osmotic_pressure(10.0, 66430.0, 298.15);
        // Much lower pressure due to high MW
        assert!(pi > 0.0 && pi < 500.0);
    }

    #[test]
    fn test_osmotic_pressure_temperature_dependence() {
        let pi_cold = osmotic_pressure(1.0, 100.0, 273.15);
        let pi_hot = osmotic_pressure(1.0, 100.0, 373.15);
        assert!(pi_hot > pi_cold);
        assert!(relative_eq(pi_hot / pi_cold, 373.15 / 273.15, 0.001));
    }

    #[test]
    fn test_osmotic_pressure_electrolyte() {
        let pi_nonelec = osmotic_pressure(1.0, 58.44, 298.15);
        let pi_nacl = osmotic_pressure_electrolyte(1.0, 58.44, 298.15, 1.8);
        assert!(relative_eq(pi_nacl / pi_nonelec, 1.8, 0.001));
    }

    #[test]
    fn test_osmotic_pressure_zero_concentration() {
        let pi = osmotic_pressure(0.0, 100.0, 298.15);
        assert!(approx_eq(pi, 0.0, TOL));
    }

    // === Freezing point depression tests ===

    #[test]
    fn test_fpd_water_sucrose() {
        // 1 mol/kg sucrose in water (non-electrolyte, i=1)
        let delta = freezing_point_depression(WATER_KF, 1.0, 1.0);
        assert!(approx_eq(delta, 1.86, TOL));
    }

    #[test]
    fn test_fpd_water_nacl() {
        // 1 mol/kg NaCl in water (i ~ 1.8)
        let delta = freezing_point_depression(WATER_KF, 1.0, 1.8);
        assert!(approx_eq(delta, 1.86 * 1.8, TOL_LOOSE));
    }

    #[test]
    fn test_fpd_benzene() {
        // 1 mol/kg solute in benzene
        let delta = freezing_point_depression(BENZENE_KF, 1.0, 1.0);
        assert!(approx_eq(delta, 5.12, TOL));
    }

    #[test]
    fn test_fpd_camphor() {
        // 0.5 mol/kg in camphor
        let delta = freezing_point_depression(CAMPHOR_KF, 0.5, 1.0);
        assert!(approx_eq(delta, 20.0, TOL));
    }

    #[test]
    fn test_fpd_zero_molality() {
        let delta = freezing_point_depression(WATER_KF, 0.0, 1.0);
        assert!(approx_eq(delta, 0.0, TOL));
    }

    #[test]
    fn test_fpd_dilute() {
        // 0.01 mol/kg
        let delta = freezing_point_depression(WATER_KF, 0.01, 1.0);
        assert!(approx_eq(delta, 0.0186, TOL));
    }

    // === Boiling point elevation tests ===

    #[test]
    fn test_bpe_water() {
        let delta = boiling_point_elevation(WATER_KB, 1.0, 1.0);
        assert!(approx_eq(delta, 0.512, TOL));
    }

    #[test]
    fn test_bpe_water_nacl() {
        let delta = boiling_point_elevation(WATER_KB, 1.0, 1.8);
        assert!(approx_eq(delta, 0.512 * 1.8, TOL_LOOSE));
    }

    #[test]
    fn test_bpe_benzene() {
        let delta = boiling_point_elevation(BENZENE_KB, 2.0, 1.0);
        assert!(approx_eq(delta, 5.06, TOL));
    }

    #[test]
    fn test_bpe_proportional_to_molality() {
        let d1 = boiling_point_elevation(WATER_KB, 1.0, 1.0);
        let d2 = boiling_point_elevation(WATER_KB, 2.0, 1.0);
        assert!(relative_eq(d2 / d1, 2.0, 0.001));
    }

    // === Number average MW tests ===

    #[test]
    fn test_number_avg_mw() {
        // If intercept = RT*1000/Mn, for Mn=50000 at T=298.15
        let intercept = R_GAS * 298.15 * 1000.0 / 50000.0;
        let mn = number_avg_mw(intercept, 298.15);
        assert!(relative_eq(mn, 50000.0, 0.001));
    }

    #[test]
    fn test_number_avg_mw_small_molecule() {
        // Mn = 100
        let intercept = R_GAS * 298.15 * 1000.0 / 100.0;
        let mn = number_avg_mw(intercept, 298.15);
        assert!(relative_eq(mn, 100.0, 0.001));
    }

    // === Osmolality tests ===

    #[test]
    fn test_osmolality_conversion() {
        assert!(approx_eq(osmolality(0.290), 290.0, TOL));
        assert!(approx_eq(osmolality(0.0), 0.0, TOL));
        assert!(approx_eq(osmolality(1.0), 1000.0, TOL));
    }

    #[test]
    fn test_osmolality_from_molality() {
        // 0.154 mol/kg NaCl, i=1.8, phi=0.93
        let osm = osmolality_from_molality(0.154, 1.8, 0.93);
        // Should be ~257 mOsm/kg
        assert!(osm > 250.0 && osm < 270.0);
    }

    #[test]
    fn test_osmolality_from_molality_ideal() {
        let osm = osmolality_from_molality(0.3, 1.0, 1.0);
        assert!(approx_eq(osm, 300.0, TOL));
    }

    // === Raoult's law tests ===

    #[test]
    fn test_raoult_pure_solvent() {
        assert!(approx_eq(raoult_vapor_pressure_lowering(100.0, 0.0), 0.0, TOL));
    }

    #[test]
    fn test_raoult_dilute() {
        let dp = raoult_vapor_pressure_lowering(23.8, 0.01);
        assert!(approx_eq(dp, 0.238, TOL));
    }

    #[test]
    fn test_raoult_half_mole_fraction() {
        let dp = raoult_vapor_pressure_lowering(100.0, 0.5);
        assert!(approx_eq(dp, 50.0, TOL));
    }

    // === Concentration conversions ===

    #[test]
    fn test_concentration_to_molality() {
        // 18 g/L of MW=180 solute in water
        let m = concentration_to_molality(18.0, 180.0, WATER_DENSITY);
        // 18/180 = 0.1 mol/L / 0.997 kg/L ~ 0.1003 mol/kg
        assert!(relative_eq(m, 0.1003, 0.01));
    }

    #[test]
    fn test_molality_to_concentration_roundtrip() {
        let conc_orig = 10.0;
        let mw = 100.0;
        let m = concentration_to_molality(conc_orig, mw, WATER_DENSITY);
        let conc_back = molality_to_concentration(m, mw, WATER_DENSITY);
        assert!(relative_eq(conc_back, conc_orig, 0.001));
    }

    #[test]
    fn test_molality_to_mole_fraction() {
        // 1 mol/kg in water (18.015 g/mol)
        // n_solute = 1 mol, n_solvent = 1000/18.015 = 55.508 mol
        // x = 1 / (1 + 55.508) = 0.01770
        let x = molality_to_mole_fraction(1.0, 18.015);
        let expected = 1.0 / (1.0 + 1000.0 / 18.015);
        assert!(relative_eq(x, expected, 0.001));
    }

    // === Solvent constants tests ===

    #[test]
    fn test_water_constants() {
        let w = SolventConstants::water();
        assert!(approx_eq(w.kf, 1.86, TOL));
        assert!(approx_eq(w.kb, 0.512, TOL));
        assert!(approx_eq(w.fp_celsius, 0.0, TOL));
        assert!(approx_eq(w.bp_celsius, 100.0, TOL));
    }

    #[test]
    fn test_benzene_constants() {
        let b = SolventConstants::benzene();
        assert!(approx_eq(b.kf, 5.12, TOL));
        assert!(approx_eq(b.kb, 2.53, TOL));
    }

    #[test]
    fn test_camphor_constants() {
        let c = SolventConstants::camphor();
        assert!(approx_eq(c.kf, 40.0, TOL));
    }

    #[test]
    fn test_cyclohexane_constants() {
        let c = SolventConstants::cyclohexane();
        assert!(approx_eq(c.kf, 20.0, TOL));
    }

    #[test]
    fn test_kelvin_conversions() {
        let w = SolventConstants::water();
        assert!(approx_eq(w.fp_kelvin(), 273.15, TOL));
        assert!(approx_eq(w.bp_kelvin(), 373.15, TOL));
    }

    // === Membrane osmometer tests ===

    #[test]
    fn test_membrane_osmometer_basic() {
        let mut osm = MembraneOsmometer::new(298.15);
        // Generate synthetic data for Mn=50000 polymer
        let mn_true = 50000.0;
        let a2_true = 3.0e-4; // mol*mL/g^2
        let rt_1000 = R_GAS * 298.15 * 1000.0;

        for &c in &[1.0, 2.0, 5.0, 10.0, 20.0] {
            let pi_c = rt_1000 / mn_true + rt_1000 * a2_true * c;
            let pi = pi_c * c;
            osm.add_measurement(c, pi);
        }

        let result = osm.analyze();
        assert!(relative_eq(result.mn, mn_true, 0.01));
    }

    #[test]
    fn test_membrane_osmometer_num_measurements() {
        let mut osm = MembraneOsmometer::new(298.15);
        assert_eq!(osm.num_measurements(), 0);
        osm.add_measurement(1.0, 100.0);
        assert_eq!(osm.num_measurements(), 1);
        osm.add_measurement(2.0, 200.0);
        assert_eq!(osm.num_measurements(), 2);
    }

    #[test]
    fn test_membrane_osmometer_pi_over_c() {
        let mut osm = MembraneOsmometer::new(298.15);
        osm.add_measurement(2.0, 100.0);
        osm.add_measurement(4.0, 240.0);
        let pi_c = osm.pi_over_c();
        assert!(approx_eq(pi_c[0], 50.0, TOL));
        assert!(approx_eq(pi_c[1], 60.0, TOL));
    }

    #[test]
    fn test_membrane_osmometer_predict() {
        let mut osm = MembraneOsmometer::new(298.15);
        let mn_true = 50000.0;
        let rt_1000 = R_GAS * 298.15 * 1000.0;

        for &c in &[1.0, 5.0, 10.0] {
            let pi_c = rt_1000 / mn_true;
            let pi = pi_c * c;
            osm.add_measurement(c, pi);
        }

        let pred = osm.predict_pressure(5.0);
        let expected = (rt_1000 / mn_true) * 5.0;
        assert!(relative_eq(pred, expected, 0.01));
    }

    #[test]
    fn test_membrane_osmometer_temperature() {
        let osm = MembraneOsmometer::new(310.15);
        assert!(approx_eq(osm.temperature(), 310.15, TOL));
    }

    #[test]
    fn test_membrane_osmometer_add_measurements() {
        let mut osm = MembraneOsmometer::new(298.15);
        osm.add_measurements(&[1.0, 2.0], &[100.0, 200.0]);
        assert_eq!(osm.num_measurements(), 2);
    }

    // === VPO tests ===

    #[test]
    fn test_vpo_calibration() {
        let mut vpo = VaporPressureOsmometer::water_37c();
        // Calibrate with sucrose (MW 342.3)
        vpo.calibrate(10.0, 342.3, 5.0); // 10 g/L, 342.3 g/mol, 5V
        assert!(approx_eq(vpo.calibration_constant(), 5.0 * 342.3 / 10.0, TOL));
    }

    #[test]
    fn test_vpo_measure() {
        let mut vpo = VaporPressureOsmometer::water_37c();
        vpo.calibrate(10.0, 342.3, 5.0);
        let result = vpo.measure(10.0, 5.0);
        // Should recover calibration MW
        assert!(relative_eq(result.molar_mass, 342.3, 0.01));
    }

    #[test]
    fn test_vpo_temperature() {
        let vpo = VaporPressureOsmometer::water_37c();
        assert!(approx_eq(vpo.temperature(), 310.15, TOL));
    }

    #[test]
    fn test_vpo_osmolality() {
        let mut vpo = VaporPressureOsmometer::water_37c();
        vpo.calibrate(10.0, 180.0, 1.0);
        let result = vpo.measure(10.0, 1.0);
        assert!(result.osmolality > 0.0);
    }

    // === Cryoscopic analyzer tests ===

    #[test]
    fn test_cryoscopic_depression_water() {
        let cryo = CryoscopicAnalyzer::water();
        let result = cryo.depression(1.0, 1.0);
        assert!(approx_eq(result.delta_tf, 1.86, TOL));
        assert!(approx_eq(result.new_fp_celsius, -1.86, TOL));
    }

    #[test]
    fn test_cryoscopic_depression_nacl() {
        let cryo = CryoscopicAnalyzer::water();
        let result = cryo.depression(0.5, 1.8);
        let expected = 1.86 * 0.5 * 1.8;
        assert!(approx_eq(result.delta_tf, expected, TOL));
    }

    #[test]
    fn test_cryoscopic_determine_mw() {
        let cryo = CryoscopicAnalyzer::water();
        // 5 g solute in 0.1 kg water, delta_tf = 1.86 K
        // MW = Kf * i * m_solute / (delta_tf * m_solvent)
        //    = 1.86 * 1 * 5 / (1.86 * 0.1) = 50
        let result = cryo.determine_mw(1.86, 5.0, 0.1, 1.0);
        assert!(relative_eq(result.estimated_mw.unwrap(), 50.0, 0.01));
    }

    #[test]
    fn test_cryoscopic_benzene() {
        let cryo = CryoscopicAnalyzer::benzene();
        let result = cryo.depression(1.0, 1.0);
        assert!(approx_eq(result.delta_tf, 5.12, TOL));
        assert!(approx_eq(result.new_fp_celsius, 5.5 - 5.12, TOL_LOOSE));
    }

    #[test]
    fn test_cryoscopic_camphor() {
        let cryo = CryoscopicAnalyzer::camphor();
        assert!(approx_eq(cryo.kf(), 40.0, TOL));
    }

    #[test]
    fn test_cryoscopic_molality_from_depression() {
        let cryo = CryoscopicAnalyzer::water();
        let m = cryo.molality_from_depression(3.72, 1.0);
        assert!(relative_eq(m, 2.0, 0.01));
    }

    // === Ebullioscopic analyzer tests ===

    #[test]
    fn test_ebullioscopic_elevation_water() {
        let ebull = EbullioscopicAnalyzer::water();
        let result = ebull.elevation(1.0, 1.0);
        assert!(approx_eq(result.delta_tb, 0.512, TOL));
        assert!(approx_eq(result.new_bp_celsius, 100.512, TOL));
    }

    #[test]
    fn test_ebullioscopic_determine_mw() {
        let ebull = EbullioscopicAnalyzer::water();
        // Similar to cryoscopic: MW = Kb * i * m_solute / (delta_tb * m_solvent)
        // 5 g in 0.1 kg water, delta_tb = 0.512
        // MW = 0.512 * 1 * 5 / (0.512 * 0.1) = 50
        let result = ebull.determine_mw(0.512, 5.0, 0.1, 1.0);
        assert!(relative_eq(result.estimated_mw.unwrap(), 50.0, 0.01));
    }

    #[test]
    fn test_ebullioscopic_molality_from_elevation() {
        let ebull = EbullioscopicAnalyzer::water();
        let m = ebull.molality_from_elevation(0.512, 1.0);
        assert!(relative_eq(m, 1.0, 0.01));
    }

    #[test]
    fn test_ebullioscopic_kb() {
        let ebull = EbullioscopicAnalyzer::water();
        assert!(approx_eq(ebull.kb(), 0.512, TOL));
    }

    // === Van't Hoff analyzer tests ===

    #[test]
    fn test_vanthoff_linear_analysis() {
        let mut vh = VantHoffAnalyzer::new(298.15);
        let mn_true = 30000.0;
        let a2_true = 2.0e-4;
        let rt_1000 = R_GAS * 298.15 * 1000.0;

        for &c in &[1.0, 2.0, 5.0, 10.0] {
            let pi_c = rt_1000 / mn_true + rt_1000 * a2_true * c;
            vh.add_point(c, pi_c);
        }

        let result = vh.analyze_linear();
        assert!(relative_eq(result.mn, mn_true, 0.01));
        assert!(relative_eq(result.a2, a2_true, 0.01));
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_vanthoff_quadratic_analysis() {
        let mut vh = VantHoffAnalyzer::new(298.15);
        let mn_true = 30000.0;
        let a2_true = 2.0e-4;
        let a3_true = 1.0e-7;
        let rt_1000 = R_GAS * 298.15 * 1000.0;

        for &c in &[0.5, 1.0, 2.0, 5.0, 10.0, 20.0] {
            let pi_c = rt_1000 / mn_true
                + rt_1000 * a2_true * c
                + rt_1000 * a3_true * c * c;
            vh.add_point(c, pi_c);
        }

        let result = vh.analyze_quadratic();
        assert!(relative_eq(result.mn, mn_true, 0.05));
        assert!(result.a3.is_some());
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_vanthoff_from_pressures() {
        let mut vh = VantHoffAnalyzer::new(298.15);
        let concs = vec![1.0, 2.0, 3.0];
        let pressures = vec![100.0, 210.0, 330.0];
        vh.add_from_pressures(&concs, &pressures);
        assert_eq!(vh.num_points(), 3);
    }

    #[test]
    fn test_vanthoff_temperature() {
        let vh = VantHoffAnalyzer::new(310.15);
        assert!(approx_eq(vh.temperature(), 310.15, TOL));
    }

    // === Second virial coefficient tests ===

    #[test]
    fn test_a2_good_solvent() {
        let mut a2_calc = SecondVirialCoefficient::new(298.15);
        let rt_1000 = R_GAS * 298.15 * 1000.0;
        let mn = 50000.0;
        let a2_true = 5.0e-4; // Positive => good solvent

        for &c in &[1.0, 2.0, 5.0, 10.0] {
            let pi_c = rt_1000 / mn + rt_1000 * a2_true * c;
            a2_calc.add_point(c, pi_c);
        }

        let result = a2_calc.determine();
        assert_eq!(result.solvent_quality, SolventQuality::Good);
        assert!(result.a2 > 0.0);
    }

    #[test]
    fn test_a2_poor_solvent() {
        let mut a2_calc = SecondVirialCoefficient::new(298.15);
        let rt_1000 = R_GAS * 298.15 * 1000.0;
        let mn = 50000.0;
        let a2_true = -2.0e-4; // Negative => poor solvent

        for &c in &[1.0, 2.0, 5.0, 10.0] {
            let pi_c = rt_1000 / mn + rt_1000 * a2_true * c;
            a2_calc.add_point(c, pi_c);
        }

        let result = a2_calc.determine();
        assert_eq!(result.solvent_quality, SolventQuality::Poor);
        assert!(result.a2 < 0.0);
    }

    #[test]
    fn test_a2_theta_estimation() {
        let theta = SecondVirialCoefficient::estimate_theta_temperature(
            1.0e-4, 300.0,
            -1.0e-4, 280.0,
        );
        assert!(theta.is_some());
        let t = theta.unwrap();
        assert!(t > 280.0 && t < 300.0);
        // Linear interpolation: should be 290
        assert!(relative_eq(t, 290.0, 0.01));
    }

    #[test]
    fn test_a2_theta_no_sign_change() {
        let theta = SecondVirialCoefficient::estimate_theta_temperature(
            1.0e-4, 300.0,
            2.0e-4, 280.0,
        );
        assert!(theta.is_none());
    }

    #[test]
    fn test_a2_from_pressures() {
        let mut a2_calc = SecondVirialCoefficient::new(298.15);
        a2_calc.add_from_pressures(&[1.0, 2.0], &[100.0, 210.0]);
        let result = a2_calc.determine();
        assert!(result.mn > 0.0);
    }

    // === Isotonicity calculator tests ===

    #[test]
    fn test_isotonicity_isotonic_nacl() {
        assert!(IsotonicityCalculator::is_nacl_isotonic(0.9));
    }

    #[test]
    fn test_isotonicity_hypotonic_nacl() {
        assert!(!IsotonicityCalculator::is_nacl_isotonic(0.3));
    }

    #[test]
    fn test_isotonicity_calculator_pure_water() {
        let calc = IsotonicityCalculator::new();
        let result = calc.calculate(&[]);
        assert!(approx_eq(result.total_osmolality, 0.0, TOL));
        assert!(result.is_hypotonic);
        assert!(!result.is_isotonic);
        assert!(result.nacl_to_add_g_per_l > 0.0);
    }

    #[test]
    fn test_isotonicity_calculator_glucose_5_percent() {
        let calc = IsotonicityCalculator::new();
        // 5% w/v glucose = 50 g/L, MW=180.16, i=1
        let glucose = SoluteContribution::new("Glucose", 50.0, 180.16, 1.0);
        let result = calc.calculate(&[glucose]);
        // 50/180.16 = 0.2775 mol/L ~ 278 mOsm/kg => close to isotonic
        assert!(result.total_osmolality > 250.0 && result.total_osmolality < 310.0);
    }

    #[test]
    fn test_isotonicity_target() {
        let calc = IsotonicityCalculator::with_target(300.0);
        assert!(approx_eq(calc.target_osmolality(), 300.0, TOL));
    }

    #[test]
    fn test_isotonicity_nacl_percent() {
        let calc = IsotonicityCalculator::new();
        let pct = calc.isotonic_nacl_percent();
        // Should be close to 0.9%
        assert!(pct > 0.7 && pct < 1.1);
    }

    #[test]
    fn test_isotonicity_hypertonic_detection() {
        let calc = IsotonicityCalculator::new();
        // 100 g/L NaCl => very hypertonic
        let nacl = SoluteContribution::new("NaCl", 100.0, 58.44, 1.8);
        let result = calc.calculate(&[nacl]);
        assert!(result.is_hypertonic);
    }

    #[test]
    fn test_solute_nacl_equivalent() {
        // For NaCl itself: E = 58.44 * 1.8 / (1.8 * 58.44) = 1.0
        let nacl = SoluteContribution::new("NaCl", 9.0, 58.44, 1.8);
        assert!(relative_eq(nacl.nacl_equivalent(), 1.0, 0.001));
    }

    #[test]
    fn test_solute_with_osmotic_coefficient() {
        let s = SoluteContribution::new("Test", 10.0, 100.0, 1.0)
            .with_osmotic_coefficient(0.9);
        assert!(approx_eq(s.osmotic_coefficient, 0.9, TOL));
    }

    // === Osmotic coefficient tests ===

    #[test]
    fn test_osmotic_coefficient_ideal() {
        let oc = OsmoticCoefficient::ideal();
        assert!(approx_eq(oc.phi_at(0.0), 1.0, TOL));
        assert!(approx_eq(oc.phi_at(5.0), 1.0, TOL));
    }

    #[test]
    fn test_osmotic_coefficient_nacl() {
        let oc = OsmoticCoefficient::nacl_approximate();
        let phi = oc.phi_at(0.15);
        // NaCl at 0.15 mol/kg should be ~ 0.93
        assert!(phi > 0.92 && phi < 0.94);
    }

    #[test]
    fn test_osmotic_coefficient_interpolation() {
        let oc = OsmoticCoefficient::new(
            vec![0.0, 1.0, 2.0],
            vec![1.0, 0.9, 0.8],
        );
        let phi = oc.phi_at(0.5);
        assert!(approx_eq(phi, 0.95, TOL));
    }

    #[test]
    fn test_osmotic_coefficient_extrapolation() {
        let oc = OsmoticCoefficient::new(
            vec![0.0, 1.0],
            vec![1.0, 0.9],
        );
        // Below range
        assert!(approx_eq(oc.phi_at(-1.0), 1.0, TOL));
        // Above range
        assert!(approx_eq(oc.phi_at(5.0), 0.9, TOL));
    }

    #[test]
    fn test_debye_huckel_phi() {
        let phi = OsmoticCoefficient::debye_huckel_phi(0.01, 1.0, -1.0);
        // phi = 1 - 0.3915/3 * 1 * sqrt(0.01) = 1 - 0.01305 ~ 0.987
        assert!(phi > 0.98 && phi < 1.0);
    }

    #[test]
    fn test_ionic_strength_11() {
        let i = OsmoticCoefficient::ionic_strength_11(0.1);
        assert!(approx_eq(i, 0.1, TOL));
    }

    #[test]
    fn test_ionic_strength_21() {
        // CaCl2: z+=2, z-=1, nu+=1, nu-=2
        let i = OsmoticCoefficient::ionic_strength(0.1, 2.0, 1.0, 1.0, 2.0);
        // I = 0.5 * 0.1 * (1*4 + 2*1) = 0.5 * 0.1 * 6 = 0.3
        assert!(approx_eq(i, 0.3, TOL));
    }

    #[test]
    fn test_osmotic_coefficient_num_points() {
        let oc = OsmoticCoefficient::nacl_approximate();
        assert_eq!(oc.num_points(), 9);
    }

    // === Colligative property suite tests ===

    #[test]
    fn test_suite_water_sucrose() {
        let suite = ColligativePropertySuite::water_25c();
        let result = suite.analyze(1.0, 1.0, 342.3);

        // FPD
        assert!(approx_eq(result.delta_tf, 1.86, TOL));
        assert!(approx_eq(result.new_fp_celsius, -1.86, TOL));

        // BPE
        assert!(approx_eq(result.delta_tb, 0.512, TOL));
        assert!(approx_eq(result.new_bp_celsius, 100.512, TOL));

        // Osmotic pressure should be positive
        assert!(result.osmotic_pressure_pa > 0.0);
        assert!(result.osmotic_pressure_atm > 0.0);

        // Osmolality
        assert!(approx_eq(result.osmolality_mosm, 1000.0, TOL));
    }

    #[test]
    fn test_suite_nacl_isotonic() {
        let suite = ColligativePropertySuite::water_25c();
        // 0.154 mol/kg NaCl with i=1.8
        let result = suite.analyze(0.154, 1.8, NACL_MOLAR_MASS);
        // Osmolality should be near blood range
        let osm = result.osmolality_mosm;
        assert!(osm > 250.0 && osm < 310.0);
    }

    // === Weighted Mn determination tests ===

    #[test]
    fn test_weighted_mn_determination() {
        let mn_true = 50000.0;
        let rt_1000 = R_GAS * 298.15 * 1000.0;
        let concs = vec![1.0, 2.0, 5.0, 10.0, 20.0];
        let pressures: Vec<f64> = concs
            .iter()
            .map(|&c| (rt_1000 / mn_true) * c)
            .collect();

        let (mn, _a2, r2) = weighted_mn_determination(&concs, &pressures, 298.15);
        assert!(relative_eq(mn, mn_true, 0.05));
        assert!(r2 > 0.99);
    }

    // === Tonicity classification tests ===

    #[test]
    fn test_classify_tonicity_isotonic() {
        assert_eq!(classify_tonicity(290.0), Tonicity::Isotonic);
        assert_eq!(classify_tonicity(285.0), Tonicity::Isotonic);
        assert_eq!(classify_tonicity(295.0), Tonicity::Isotonic);
    }

    #[test]
    fn test_classify_tonicity_hypotonic() {
        assert_eq!(classify_tonicity(200.0), Tonicity::Hypotonic);
        assert_eq!(classify_tonicity(284.0), Tonicity::Hypotonic);
    }

    #[test]
    fn test_classify_tonicity_hypertonic() {
        assert_eq!(classify_tonicity(400.0), Tonicity::Hypertonic);
        assert_eq!(classify_tonicity(296.0), Tonicity::Hypertonic);
    }

    // === NaCl osmolality tests ===

    #[test]
    fn test_nacl_osmolality_isotonic() {
        let osm = nacl_osmolality(0.9);
        // 0.9% NaCl ~ 286-290 mOsm/kg
        assert!(osm > 270.0 && osm < 310.0);
    }

    #[test]
    fn test_nacl_osmolality_zero() {
        let osm = nacl_osmolality(0.0);
        assert!(approx_eq(osm, 0.0, TOL));
    }

    #[test]
    fn test_nacl_percent_for_osmolality_roundtrip() {
        let target = 290.0;
        let pct = nacl_percent_for_osmolality(target);
        let osm = nacl_osmolality(pct);
        assert!(relative_eq(osm, target, 0.01));
    }

    // === Mixture osmolality tests ===

    #[test]
    fn test_mixture_osmolality_single() {
        // Use n_ions=2.0, phi=0.93 for NaCl (9 g/L = 0.9% w/v)
        let osm = mixture_osmolality(&[(9.0, NACL_MOLAR_MASS, 2.0, 0.93)]);
        assert!(osm > 270.0 && osm < 310.0);
    }

    #[test]
    fn test_mixture_osmolality_multiple() {
        let osm = mixture_osmolality(&[
            (5.0, NACL_MOLAR_MASS, 2.0, 0.93),   // partial NaCl
            (25.0, 180.16, 1.0, 1.0),              // glucose
        ]);
        // Should be sum of both contributions
        assert!(osm > 200.0);
    }

    #[test]
    fn test_mixture_osmolality_empty() {
        let osm = mixture_osmolality(&[]);
        assert!(approx_eq(osm, 0.0, TOL));
    }

    // === Van't Hoff factor tests ===

    #[test]
    fn test_estimate_van_hoff_factor() {
        let i = estimate_van_hoff_factor(WATER_KF, 1.0, 3.348);
        // 3.348 / 1.86 = 1.8
        assert!(relative_eq(i, 1.8, 0.01));
    }

    #[test]
    fn test_degree_of_dissociation_full() {
        let alpha = degree_of_dissociation(2.0, 2);
        assert!(approx_eq(alpha, 1.0, TOL));
    }

    #[test]
    fn test_degree_of_dissociation_partial() {
        // NaCl i=1.8, n=2: alpha = (1.8-1)/(2-1) = 0.8
        let alpha = degree_of_dissociation(1.8, 2);
        assert!(approx_eq(alpha, 0.8, TOL));
    }

    #[test]
    fn test_degree_of_dissociation_no_dissociation() {
        let alpha = degree_of_dissociation(1.0, 2);
        assert!(approx_eq(alpha, 0.0, TOL));
    }

    #[test]
    fn test_van_hoff_from_dissociation_roundtrip() {
        let i = van_hoff_from_dissociation(0.8, 2);
        assert!(approx_eq(i, 1.8, TOL));
        let alpha = degree_of_dissociation(i, 2);
        assert!(approx_eq(alpha, 0.8, TOL));
    }

    #[test]
    fn test_van_hoff_from_dissociation_trivalent() {
        // CaCl2: n=3 ions, alpha=0.7
        let i = van_hoff_from_dissociation(0.7, 3);
        // i = 1 + 0.7*2 = 2.4
        assert!(approx_eq(i, 2.4, TOL));
    }
}

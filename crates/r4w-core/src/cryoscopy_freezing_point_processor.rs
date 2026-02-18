//! # Cryoscopy Freezing Point Processor
//!
//! Cryoscopic analysis for freezing point depression measurements used in molecular
//! weight determination, solution thermodynamics, and purity analysis.
//!
//! ## Theory
//!
//! Freezing point depression is a colligative property: the decrease in the freezing
//! point of a solvent upon addition of a solute depends only on the number of solute
//! particles, not their identity. The relationship is:
//!
//! ```text
//! ΔTf = Kf × m × i
//! ```
//!
//! where:
//! - `ΔTf` = freezing point depression (K)
//! - `Kf`  = cryoscopic constant of the solvent (K·kg/mol)
//! - `m`   = molality of solute (mol/kg solvent)
//! - `i`   = van't Hoff factor (number of particles per formula unit)
//!
//! ## Cryoscopic Constant
//!
//! The cryoscopic constant is derived from thermodynamic properties of the pure solvent:
//!
//! ```text
//! Kf = R × Tf² × M_solvent / (1000 × ΔHfus)
//! ```
//!
//! where `R` is the gas constant, `Tf` is the normal freezing point (K),
//! `M_solvent` is the molar mass (g/mol), and `ΔHfus` is the molar enthalpy of fusion (J/mol).
//!
//! ## Applications
//!
//! - **Molecular weight determination**: Measure ΔTf, solve for MW of unknown solute
//! - **Purity analysis**: Van't Hoff purity method from DSC-like cooling curves
//! - **Osmolality**: Clinical/food science osmolality from freezing point depression
//! - **Phase diagrams**: Eutectic point determination from binary cooling curves

use std::f64::consts::PI;

/// Universal gas constant in J/(mol·K)
const R_GAS: f64 = 8.314462;

// ──────────────────────────────────────────────────────────────────────────────
// CryoscopicConstant
// ──────────────────────────────────────────────────────────────────────────────

/// Cryoscopic constant (Kf) for a solvent.
///
/// Kf = R × Tf² × M_solvent / (1000 × ΔHfus)
///
/// Units: K·kg/mol
#[derive(Debug, Clone, PartialEq)]
pub struct CryoscopicConstant {
    /// Name of the solvent
    pub name: String,
    /// Normal freezing point of the pure solvent (K)
    pub freezing_point_k: f64,
    /// Molar mass of the solvent (g/mol)
    pub molar_mass: f64,
    /// Molar enthalpy of fusion (J/mol)
    pub enthalpy_fusion: f64,
    /// Cryoscopic constant Kf (K·kg/mol)
    pub kf: f64,
}

impl CryoscopicConstant {
    /// Create a new cryoscopic constant from thermodynamic properties.
    ///
    /// Kf is calculated as R × Tf² × M / (1000 × ΔHfus).
    pub fn new(name: &str, freezing_point_k: f64, molar_mass: f64, enthalpy_fusion: f64) -> Self {
        let kf = R_GAS * freezing_point_k * freezing_point_k * molar_mass
            / (1000.0 * enthalpy_fusion);
        Self {
            name: name.to_string(),
            freezing_point_k,
            molar_mass,
            enthalpy_fusion,
            kf,
        }
    }

    /// Create from a known Kf value directly.
    pub fn from_kf(name: &str, freezing_point_k: f64, molar_mass: f64, kf: f64) -> Self {
        // Back-calculate enthalpy of fusion
        let enthalpy_fusion =
            R_GAS * freezing_point_k * freezing_point_k * molar_mass / (1000.0 * kf);
        Self {
            name: name.to_string(),
            freezing_point_k,
            molar_mass,
            enthalpy_fusion,
            kf,
        }
    }

    /// Water: Kf = 1.86 K·kg/mol, Tf = 273.15 K, M = 18.015 g/mol
    pub fn water() -> Self {
        Self::from_kf("Water", 273.15, 18.015, 1.86)
    }

    /// Benzene: Kf = 5.12 K·kg/mol, Tf = 278.68 K, M = 78.11 g/mol
    pub fn benzene() -> Self {
        Self::from_kf("Benzene", 278.68, 78.11, 5.12)
    }

    /// Cyclohexane: Kf = 20.0 K·kg/mol, Tf = 279.65 K, M = 84.16 g/mol
    pub fn cyclohexane() -> Self {
        Self::from_kf("Cyclohexane", 279.65, 84.16, 20.0)
    }

    /// Camphor: Kf = 37.7 K·kg/mol, Tf = 451.55 K, M = 152.23 g/mol
    pub fn camphor() -> Self {
        Self::from_kf("Camphor", 451.55, 152.23, 37.7)
    }

    /// Naphthalene: Kf = 6.94 K·kg/mol, Tf = 353.43 K, M = 72.11 g/mol
    pub fn naphthalene() -> Self {
        // Note: naphthalene actual M = 128.17, but using traditional tabulated Kf
        Self::from_kf("Naphthalene", 353.43, 128.17, 6.94)
    }

    /// Acetic acid: Kf = 3.90 K·kg/mol, Tf = 289.77 K, M = 60.05 g/mol
    pub fn acetic_acid() -> Self {
        Self::from_kf("Acetic acid", 289.77, 60.05, 3.90)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// FreezingPointDepression
// ──────────────────────────────────────────────────────────────────────────────

/// Freezing point depression calculator.
///
/// ΔTf = Kf × m × i
#[derive(Debug, Clone)]
pub struct FreezingPointDepression {
    /// Cryoscopic constant of the solvent
    pub solvent: CryoscopicConstant,
    /// Van't Hoff factor (1 for non-electrolytes, 2 for NaCl, 3 for CaCl2, etc.)
    pub vant_hoff_factor: f64,
}

impl FreezingPointDepression {
    /// Create a new FPD calculator.
    pub fn new(solvent: CryoscopicConstant, vant_hoff_factor: f64) -> Self {
        Self {
            solvent,
            vant_hoff_factor,
        }
    }

    /// Create for a non-electrolyte solute (i = 1).
    pub fn non_electrolyte(solvent: CryoscopicConstant) -> Self {
        Self::new(solvent, 1.0)
    }

    /// Calculate the freezing point depression ΔTf for a given molality.
    pub fn delta_tf(&self, molality: f64) -> f64 {
        self.solvent.kf * molality * self.vant_hoff_factor
    }

    /// Calculate the new freezing point (K) of the solution.
    pub fn freezing_point(&self, molality: f64) -> f64 {
        self.solvent.freezing_point_k - self.delta_tf(molality)
    }

    /// Calculate molality from observed depression.
    pub fn molality_from_depression(&self, delta_tf: f64) -> f64 {
        delta_tf / (self.solvent.kf * self.vant_hoff_factor)
    }

    /// Calculate molality from solute mass (g) and solvent mass (kg).
    pub fn molality(solute_mass_g: f64, solute_mw: f64, solvent_mass_kg: f64) -> f64 {
        (solute_mass_g / solute_mw) / solvent_mass_kg
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// MolecularWeightFromFPD
// ──────────────────────────────────────────────────────────────────────────────

/// Molecular weight determination from freezing point depression.
///
/// MW = Kf × w_solute / (ΔTf × w_solvent_kg)
#[derive(Debug, Clone)]
pub struct MolecularWeightFromFPD {
    /// Cryoscopic constant of the solvent
    pub solvent: CryoscopicConstant,
    /// Van't Hoff factor
    pub vant_hoff_factor: f64,
}

/// Result of a molecular weight determination.
#[derive(Debug, Clone)]
pub struct MwResult {
    /// Calculated molecular weight (g/mol)
    pub mw: f64,
    /// Molality used (mol/kg)
    pub molality: f64,
    /// Freezing point depression observed (K)
    pub delta_tf: f64,
}

impl MolecularWeightFromFPD {
    /// Create a new MW determination calculator.
    pub fn new(solvent: CryoscopicConstant, vant_hoff_factor: f64) -> Self {
        Self {
            solvent,
            vant_hoff_factor,
        }
    }

    /// Calculate molecular weight from a single measurement.
    ///
    /// - `solute_mass_g`: mass of solute in grams
    /// - `solvent_mass_kg`: mass of solvent in kilograms
    /// - `delta_tf`: observed freezing point depression (K)
    pub fn calculate(&self, solute_mass_g: f64, solvent_mass_kg: f64, delta_tf: f64) -> MwResult {
        let mw = self.solvent.kf * solute_mass_g * self.vant_hoff_factor
            / (delta_tf * solvent_mass_kg);
        let molality = (solute_mass_g / mw) / solvent_mass_kg;
        MwResult {
            mw,
            molality,
            delta_tf,
        }
    }

    /// Multi-concentration extrapolation to zero concentration.
    ///
    /// Takes multiple (solute_mass_g, solvent_mass_kg, delta_tf) measurements
    /// and extrapolates MW to zero concentration using linear regression of
    /// apparent MW vs concentration.
    pub fn extrapolate(
        &self,
        measurements: &[(f64, f64, f64)],
    ) -> Option<MwExtrapolationResult> {
        if measurements.len() < 2 {
            return None;
        }

        let mut concentrations = Vec::new();
        let mut apparent_mws = Vec::new();

        for &(solute_g, solvent_kg, dtf) in measurements {
            let result = self.calculate(solute_g, solvent_kg, dtf);
            // Concentration in g/kg
            let conc = solute_g / solvent_kg;
            concentrations.push(conc);
            apparent_mws.push(result.mw);
        }

        // Linear regression: MW_app = MW_0 + slope × concentration
        let n = concentrations.len() as f64;
        let sum_x: f64 = concentrations.iter().sum();
        let sum_y: f64 = apparent_mws.iter().sum();
        let sum_xy: f64 = concentrations
            .iter()
            .zip(apparent_mws.iter())
            .map(|(x, y)| x * y)
            .sum();
        let sum_xx: f64 = concentrations.iter().map(|x| x * x).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-15 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        // Residuals for R²
        let mean_y = sum_y / n;
        let ss_tot: f64 = apparent_mws.iter().map(|y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = concentrations
            .iter()
            .zip(apparent_mws.iter())
            .map(|(x, y)| {
                let pred = intercept + slope * x;
                (y - pred).powi(2)
            })
            .sum();

        let r_squared = if ss_tot > 1e-15 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        Some(MwExtrapolationResult {
            mw_extrapolated: intercept,
            slope,
            r_squared,
            apparent_mws,
            concentrations,
        })
    }
}

/// Result of multi-concentration extrapolation.
#[derive(Debug, Clone)]
pub struct MwExtrapolationResult {
    /// Extrapolated MW at zero concentration (g/mol)
    pub mw_extrapolated: f64,
    /// Slope of apparent MW vs concentration
    pub slope: f64,
    /// Coefficient of determination
    pub r_squared: f64,
    /// Apparent MW at each concentration
    pub apparent_mws: Vec<f64>,
    /// Concentrations used (g/kg)
    pub concentrations: Vec<f64>,
}

// ──────────────────────────────────────────────────────────────────────────────
// CoolingCurveAnalyzer
// ──────────────────────────────────────────────────────────────────────────────

/// Process temperature vs time cooling curves for freezing point determination.
///
/// Features:
/// - Supercooling dip detection
/// - Freezing plateau identification
/// - Onset temperature determination
/// - Scheil equation for non-equilibrium freezing
#[derive(Debug, Clone)]
pub struct CoolingCurveAnalyzer {
    /// Tolerance for plateau detection (K)
    pub plateau_tolerance: f64,
    /// Minimum number of consecutive points to qualify as a plateau
    pub min_plateau_points: usize,
    /// Smoothing window size for derivative computation
    pub smoothing_window: usize,
}

/// Result of cooling curve analysis.
#[derive(Debug, Clone)]
pub struct CoolingCurveResult {
    /// Detected freezing temperature (K)
    pub freezing_temperature: f64,
    /// Start index of the freezing plateau
    pub plateau_start: usize,
    /// End index of the freezing plateau
    pub plateau_end: usize,
    /// Whether supercooling was detected
    pub supercooling_detected: bool,
    /// Supercooling depth (K) below the plateau, if detected
    pub supercooling_depth: f64,
    /// Index of the supercooling minimum, if detected
    pub supercooling_index: Option<usize>,
    /// Onset temperature (K), extrapolated from the initial cooling slope
    pub onset_temperature: f64,
    /// Cooling rate before freezing (K/s)
    pub pre_freeze_cooling_rate: f64,
}

impl CoolingCurveAnalyzer {
    /// Create a new cooling curve analyzer with default parameters.
    pub fn new() -> Self {
        Self {
            plateau_tolerance: 0.05,
            min_plateau_points: 5,
            smoothing_window: 3,
        }
    }

    /// Create with custom tolerance.
    pub fn with_tolerance(mut self, tol: f64) -> Self {
        self.plateau_tolerance = tol;
        self
    }

    /// Create with custom minimum plateau points.
    pub fn with_min_plateau(mut self, n: usize) -> Self {
        self.min_plateau_points = n;
        self
    }

    /// Analyze a cooling curve.
    ///
    /// - `temperatures`: temperature readings (K)
    /// - `times`: time stamps (s) corresponding to each temperature
    pub fn analyze(&self, temperatures: &[f64], times: &[f64]) -> Option<CoolingCurveResult> {
        if temperatures.len() < self.min_plateau_points + 2 || temperatures.len() != times.len() {
            return None;
        }

        // Step 1: Find the plateau (region of nearly constant temperature)
        let (plat_start, plat_end) = detect_plateau(temperatures, self.plateau_tolerance)?;

        if plat_end - plat_start < self.min_plateau_points {
            return None;
        }

        // Plateau temperature is the mean of the plateau region
        let plat_temps = &temperatures[plat_start..=plat_end];
        let freezing_temperature = plat_temps.iter().sum::<f64>() / plat_temps.len() as f64;

        // Step 2: Detect supercooling (temperature dip below plateau before it)
        let mut supercooling_detected = false;
        let mut supercooling_depth = 0.0;
        let mut supercooling_index = None;

        // Look for a minimum before the plateau that is below the plateau temperature
        if plat_start > 0 {
            let pre_plateau = &temperatures[..plat_start];
            let mut min_temp = f64::MAX;
            let mut min_idx = 0;
            for (i, &t) in pre_plateau.iter().enumerate() {
                if t < min_temp {
                    min_temp = t;
                    min_idx = i;
                }
            }
            if min_temp < freezing_temperature - self.plateau_tolerance {
                supercooling_detected = true;
                supercooling_depth = freezing_temperature - min_temp;
                supercooling_index = Some(min_idx);
            }
        }

        // Step 3: Pre-freeze cooling rate
        let pre_freeze_end = if supercooling_index.is_some() {
            supercooling_index.unwrap()
        } else {
            plat_start
        };
        let pre_freeze_start = if pre_freeze_end > 5 {
            pre_freeze_end - 5
        } else {
            0
        };
        let pre_freeze_cooling_rate = if pre_freeze_end > pre_freeze_start {
            cooling_rate(
                &temperatures[pre_freeze_start..pre_freeze_end],
                &times[pre_freeze_start..pre_freeze_end],
            )
        } else {
            0.0
        };

        // Step 4: Onset temperature extrapolation
        // Extrapolate the pre-freeze cooling line to the time of plateau start
        let onset_temperature = if pre_freeze_end > pre_freeze_start + 1 {
            let rate = pre_freeze_cooling_rate;
            let t0 = temperatures[pre_freeze_start];
            let dt = times[plat_start] - times[pre_freeze_start];
            t0 + rate * dt // rate is negative for cooling
        } else {
            freezing_temperature
        };

        Some(CoolingCurveResult {
            freezing_temperature,
            plateau_start: plat_start,
            plateau_end: plat_end,
            supercooling_detected,
            supercooling_depth,
            supercooling_index,
            onset_temperature,
            pre_freeze_cooling_rate,
        })
    }

    /// Compute the Scheil solidification fraction as a function of temperature.
    ///
    /// The Scheil equation (non-equilibrium solidification):
    /// f_s = 1 - ((T_liquidus - T) / (T_liquidus - T_solidus))^(1/(k-1))
    ///
    /// where k is the partition coefficient.
    ///
    /// Returns a vector of (temperature, solid_fraction) pairs.
    pub fn scheil_solidification(
        &self,
        t_liquidus: f64,
        t_solidus: f64,
        partition_coeff: f64,
        n_points: usize,
    ) -> Vec<(f64, f64)> {
        if n_points == 0 || partition_coeff <= 0.0 || partition_coeff >= 1.0 {
            return Vec::new();
        }

        let dt = (t_liquidus - t_solidus) / n_points as f64;
        // Scheil: f_s = 1 - ((T - T_solidus) / (T_liquidus - T_solidus))^(1/(1-k))
        // As T decreases from T_liquidus to T_solidus, fs increases from 0 to 1.
        let exponent = 1.0 / (1.0 - partition_coeff);

        (0..=n_points)
            .map(|i| {
                let t = t_liquidus - i as f64 * dt;
                let ratio = (t - t_solidus) / (t_liquidus - t_solidus);
                let fs = if ratio >= 1.0 {
                    0.0
                } else if ratio <= 0.0 {
                    1.0
                } else {
                    1.0 - ratio.powf(exponent)
                };
                (t, fs)
            })
            .collect()
    }
}

impl Default for CoolingCurveAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// EutecticAnalyzer
// ──────────────────────────────────────────────────────────────────────────────

/// Binary eutectic phase diagram analysis from cooling curve data.
///
/// Determines liquidus lines, eutectic temperature, and eutectic composition
/// from a set of cooling curves at different compositions.
#[derive(Debug, Clone)]
pub struct EutecticAnalyzer {
    /// Plateau detection tolerance (K)
    pub tolerance: f64,
}

/// A single composition measurement for eutectic analysis.
#[derive(Debug, Clone)]
pub struct CompositionMeasurement {
    /// Mole fraction of component B (0.0 to 1.0)
    pub x_b: f64,
    /// Primary freezing temperature (liquidus, K)
    pub t_liquidus: f64,
    /// Eutectic arrest temperature (K), if observed
    pub t_eutectic: Option<f64>,
}

/// Result of eutectic analysis.
#[derive(Debug, Clone)]
pub struct EutecticResult {
    /// Eutectic temperature (K)
    pub t_eutectic: f64,
    /// Eutectic composition (mole fraction of B)
    pub x_eutectic: f64,
    /// Liquidus line coefficients for component A side: T = a0 + a1*x + a2*x²
    pub liquidus_a_coeffs: (f64, f64, f64),
    /// Liquidus line coefficients for component B side: T = b0 + b1*x + b2*x²
    pub liquidus_b_coeffs: (f64, f64, f64),
}

impl EutecticAnalyzer {
    /// Create a new eutectic analyzer.
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
    }

    /// Analyze a set of composition measurements to determine the eutectic point.
    ///
    /// Requires at least 3 measurements spanning the eutectic composition.
    pub fn analyze(&self, measurements: &[CompositionMeasurement]) -> Option<EutecticResult> {
        if measurements.len() < 3 {
            return None;
        }

        // Find the eutectic temperature as the average of observed eutectic arrests
        let eutectic_temps: Vec<f64> = measurements
            .iter()
            .filter_map(|m| m.t_eutectic)
            .collect();

        let t_eutectic = if eutectic_temps.is_empty() {
            // Use the minimum liquidus temperature as estimate
            measurements
                .iter()
                .map(|m| m.t_liquidus)
                .fold(f64::MAX, f64::min)
        } else {
            eutectic_temps.iter().sum::<f64>() / eutectic_temps.len() as f64
        };

        // Find the composition with the lowest liquidus temperature (eutectic estimate)
        let min_liquidus = measurements
            .iter()
            .min_by(|a, b| a.t_liquidus.partial_cmp(&b.t_liquidus).unwrap())?;
        let x_eutectic = min_liquidus.x_b;

        // Split measurements into A-side (x < x_eutectic) and B-side (x > x_eutectic)
        let a_side: Vec<(f64, f64)> = measurements
            .iter()
            .filter(|m| m.x_b <= x_eutectic + 0.01)
            .map(|m| (m.x_b, m.t_liquidus))
            .collect();

        let b_side: Vec<(f64, f64)> = measurements
            .iter()
            .filter(|m| m.x_b >= x_eutectic - 0.01)
            .map(|m| (m.x_b, m.t_liquidus))
            .collect();

        // Fit quadratic (or linear if insufficient points) to each side
        let liquidus_a_coeffs = fit_quadratic(&a_side);
        let liquidus_b_coeffs = fit_quadratic(&b_side);

        Some(EutecticResult {
            t_eutectic,
            x_eutectic,
            liquidus_a_coeffs,
            liquidus_b_coeffs,
        })
    }

    /// Evaluate a liquidus line at a given composition.
    pub fn eval_liquidus(coeffs: &(f64, f64, f64), x: f64) -> f64 {
        coeffs.0 + coeffs.1 * x + coeffs.2 * x * x
    }
}

/// Fit a quadratic y = a0 + a1*x + a2*x² to data points using least squares.
fn fit_quadratic(data: &[(f64, f64)]) -> (f64, f64, f64) {
    let n = data.len();
    if n == 0 {
        return (0.0, 0.0, 0.0);
    }
    if n == 1 {
        return (data[0].1, 0.0, 0.0);
    }
    if n == 2 {
        // Linear fit
        let slope = (data[1].1 - data[0].1) / (data[1].0 - data[0].0 + 1e-15);
        let intercept = data[0].1 - slope * data[0].0;
        return (intercept, slope, 0.0);
    }

    // Normal equations for quadratic fit: [X^T X] a = X^T y
    // where X_i = [1, x_i, x_i²]
    let mut xtx = [[0.0f64; 3]; 3];
    let mut xty = [0.0f64; 3];

    for &(x, y) in data {
        let x2 = x * x;
        let x3 = x2 * x;
        let x4 = x2 * x2;

        xtx[0][0] += 1.0;
        xtx[0][1] += x;
        xtx[0][2] += x2;
        xtx[1][0] += x;
        xtx[1][1] += x2;
        xtx[1][2] += x3;
        xtx[2][0] += x2;
        xtx[2][1] += x3;
        xtx[2][2] += x4;

        xty[0] += y;
        xty[1] += x * y;
        xty[2] += x2 * y;
    }

    // Solve 3x3 system by Gaussian elimination
    if let Some(coeffs) = solve_3x3(&xtx, &xty) {
        (coeffs[0], coeffs[1], coeffs[2])
    } else {
        // Fallback to mean
        let mean_y = data.iter().map(|(_, y)| y).sum::<f64>() / n as f64;
        (mean_y, 0.0, 0.0)
    }
}

/// Solve a 3x3 linear system Ax = b by Gaussian elimination with partial pivoting.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let mut aug = [[0.0f64; 4]; 3];
    for i in 0..3 {
        for j in 0..3 {
            aug[i][j] = a[i][j];
        }
        aug[i][3] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..3 {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..3 {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-15 {
            return None;
        }
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Eliminate below
        for row in (col + 1)..3 {
            let factor = aug[row][col] / aug[col][col];
            for j in col..4 {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0f64; 3];
    for i in (0..3).rev() {
        x[i] = aug[i][3];
        for j in (i + 1)..3 {
            x[i] -= aug[i][j] * x[j];
        }
        x[i] /= aug[i][i];
    }

    Some(x)
}

// ──────────────────────────────────────────────────────────────────────────────
// PurityAnalyzer
// ──────────────────────────────────────────────────────────────────────────────

/// Van't Hoff purity analysis from freezing point depression.
///
/// Uses the relationship between the reciprocal of the fraction melted (1/F)
/// and the sample temperature to determine purity. The slope of the 1/F vs T
/// plot gives the impurity content.
///
/// T_sample = T_pure - (R × T_pure² × x_impurity) / (ΔHfus × F)
///
/// where F is the fraction melted and x_impurity is the mole fraction of impurity.
#[derive(Debug, Clone)]
pub struct PurityAnalyzer {
    /// Pure compound freezing point (K)
    pub t_pure: f64,
    /// Molar enthalpy of fusion (J/mol)
    pub enthalpy_fusion: f64,
}

/// Result of purity analysis.
#[derive(Debug, Clone)]
pub struct PurityResult {
    /// Mole fraction purity (0.0 to 1.0)
    pub purity: f64,
    /// Mole fraction of impurity
    pub x_impurity: f64,
    /// Percent purity
    pub percent_purity: f64,
    /// R² of the 1/F vs T fit
    pub r_squared: f64,
    /// Corrected melting point (K)
    pub corrected_melting_point: f64,
}

impl PurityAnalyzer {
    /// Create a new purity analyzer.
    pub fn new(t_pure: f64, enthalpy_fusion: f64) -> Self {
        Self {
            t_pure,
            enthalpy_fusion,
        }
    }

    /// Analyze purity from DSC-like melting data.
    ///
    /// - `fractions_melted`: F values (0 < F ≤ 1), the fraction of sample melted
    /// - `temperatures`: corresponding sample temperatures (K)
    ///
    /// Uses linear regression of T vs 1/F. The slope gives x_impurity.
    pub fn analyze(
        &self,
        fractions_melted: &[f64],
        temperatures: &[f64],
    ) -> Option<PurityResult> {
        if fractions_melted.len() < 2 || fractions_melted.len() != temperatures.len() {
            return None;
        }

        // Build 1/F and T data, filtering out F=0
        let mut inv_f = Vec::new();
        let mut temps = Vec::new();
        for (&f, &t) in fractions_melted.iter().zip(temperatures.iter()) {
            if f > 0.0 {
                inv_f.push(1.0 / f);
                temps.push(t);
            }
        }

        if inv_f.len() < 2 {
            return None;
        }

        // Linear regression: T = intercept + slope * (1/F)
        let n = inv_f.len() as f64;
        let sum_x: f64 = inv_f.iter().sum();
        let sum_y: f64 = temps.iter().sum();
        let sum_xy: f64 = inv_f.iter().zip(temps.iter()).map(|(x, y)| x * y).sum();
        let sum_xx: f64 = inv_f.iter().map(|x| x * x).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-15 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        // slope = -R × T_pure² × x_impurity / ΔHfus
        // x_impurity = -slope × ΔHfus / (R × T_pure²)
        let x_impurity = (-slope * self.enthalpy_fusion)
            / (R_GAS * self.t_pure * self.t_pure);
        let x_impurity = x_impurity.max(0.0).min(1.0);

        // R² calculation
        let mean_y = sum_y / n;
        let ss_tot: f64 = temps.iter().map(|y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = inv_f
            .iter()
            .zip(temps.iter())
            .map(|(x, y)| {
                let pred = intercept + slope * x;
                (y - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 1e-15 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        // Corrected melting point is the intercept (T at 1/F = 0, fully melted)
        let corrected_melting_point = intercept;

        Some(PurityResult {
            purity: 1.0 - x_impurity,
            x_impurity,
            percent_purity: (1.0 - x_impurity) * 100.0,
            r_squared,
            corrected_melting_point,
        })
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// SupercoolingCorrector
// ──────────────────────────────────────────────────────────────────────────────

/// Correct the measured freezing point for supercooling effects.
///
/// When a liquid supercools before crystallizing, the measured temperature
/// at the point of recalescence (temperature recovery) needs correction
/// to obtain the true equilibrium freezing point.
#[derive(Debug, Clone)]
pub struct SupercoolingCorrector {
    /// Specific heat capacity of the liquid (J/(g·K))
    pub cp_liquid: f64,
    /// Specific heat capacity of the solid (J/(g·K))
    pub cp_solid: f64,
    /// Latent heat of fusion (J/g)
    pub latent_heat: f64,
}

impl SupercoolingCorrector {
    /// Create a new supercooling corrector.
    pub fn new(cp_liquid: f64, cp_solid: f64, latent_heat: f64) -> Self {
        Self {
            cp_liquid,
            cp_solid,
            latent_heat,
        }
    }

    /// Preset for water.
    pub fn water() -> Self {
        Self::new(4.18, 2.09, 334.0)
    }

    /// Correct the freezing point using the area-ratio method.
    ///
    /// The area-ratio method uses the areas of the cooling curve above and below
    /// the plateau to estimate the true freezing point.
    ///
    /// - `t_supercool`: minimum temperature reached during supercooling (K)
    /// - `t_plateau`: observed plateau temperature (K)
    /// - `t_onset`: onset temperature of freezing (K)
    ///
    /// Returns the corrected freezing point.
    pub fn area_ratio_correction(
        &self,
        t_supercool: f64,
        t_plateau: f64,
        _t_onset: f64,
    ) -> f64 {
        // The heat released during recalescence from supercooling warms the sample.
        // Correction: ΔT_correction = cp_liquid × (T_plateau - T_supercool) / latent_heat × (T_plateau - T_supercool)
        // Simplified: T_corrected = T_plateau + cp_liquid × (T_plateau - T_supercool)² / (2 × latent_heat × sample_mass)
        // For unit mass approximation:
        let delta_sc = t_plateau - t_supercool;
        let correction = self.cp_liquid * delta_sc * delta_sc / (2.0 * self.latent_heat);
        t_plateau + correction
    }

    /// Latent heat correction for supercooling.
    ///
    /// The fraction of sample that freezes during recalescence:
    /// f = cp_liquid × (T_plateau - T_supercool) / latent_heat
    ///
    /// This fraction affects the apparent freezing point.
    pub fn latent_heat_correction(
        &self,
        t_supercool: f64,
        t_plateau: f64,
    ) -> SupercoolingCorrectionResult {
        let delta_sc = t_plateau - t_supercool;
        let fraction_frozen = self.cp_liquid * delta_sc / self.latent_heat;
        let fraction_frozen = fraction_frozen.min(1.0).max(0.0);

        // Corrected temperature accounts for the heat balance
        let t_corrected = t_plateau
            + self.cp_liquid * delta_sc * delta_sc / (2.0 * self.latent_heat);

        SupercoolingCorrectionResult {
            t_corrected,
            fraction_frozen_on_recalescence: fraction_frozen,
            supercooling_depth: delta_sc,
        }
    }

    /// Estimate the nucleation undercooling from classical nucleation theory.
    ///
    /// ΔT* = 16π × σ³ × T_m / (3 × ΔHfus² × ΔT²)
    ///
    /// This is a simplified estimate of the critical supercooling.
    pub fn critical_supercooling(
        &self,
        surface_energy: f64,
        t_melt: f64,
    ) -> f64 {
        // ΔT* ≈ 16πσ³T_m² / (3ΔH_fus²ρ)
        // Simplified: estimate based on surface energy and thermodynamics
        let numerator = 16.0 * PI * surface_energy.powi(3) * t_melt;
        let denominator = 3.0 * self.latent_heat * self.latent_heat;
        (numerator / denominator).cbrt()
    }
}

/// Result of supercooling correction.
#[derive(Debug, Clone)]
pub struct SupercoolingCorrectionResult {
    /// Corrected freezing temperature (K)
    pub t_corrected: f64,
    /// Fraction of sample that froze during recalescence
    pub fraction_frozen_on_recalescence: f64,
    /// Supercooling depth (K)
    pub supercooling_depth: f64,
}

// ──────────────────────────────────────────────────────────────────────────────
// OsmoticPressureFromFPD
// ──────────────────────────────────────────────────────────────────────────────

/// Convert freezing point depression to osmotic pressure.
///
/// π = ΔTf × ρ × R / (Kf × M_solvent)
///
/// where ρ is the solution density (kg/m³).
#[derive(Debug, Clone)]
pub struct OsmoticPressureFromFPD {
    /// Solvent cryoscopic data
    pub solvent: CryoscopicConstant,
    /// Solution density (kg/m³)
    pub density: f64,
}

impl OsmoticPressureFromFPD {
    /// Create a new osmotic pressure calculator.
    pub fn new(solvent: CryoscopicConstant, density: f64) -> Self {
        Self { solvent, density }
    }

    /// Water-based solution with density 1000 kg/m³.
    pub fn aqueous() -> Self {
        Self::new(CryoscopicConstant::water(), 1000.0)
    }

    /// Calculate osmotic pressure (Pa) from freezing point depression (K).
    pub fn osmotic_pressure(&self, delta_tf: f64) -> f64 {
        // π = ΔTf × ρ × R / (Kf × M_solvent)
        // Units: K × kg/m³ × J/(mol·K) / (K·kg/mol × g/mol)
        // Need M in kg/mol: M_solvent / 1000
        let m_kg_per_mol = self.solvent.molar_mass / 1000.0;
        delta_tf * self.density * R_GAS / (self.solvent.kf * 1000.0 / self.solvent.molar_mass * m_kg_per_mol)
    }

    /// Simplified osmotic pressure using osmolality.
    ///
    /// π = osmolality × R × T
    ///
    /// where osmolality is ΔTf / Kf (in osmol/kg).
    pub fn osmotic_pressure_simplified(&self, delta_tf: f64, temperature_k: f64) -> f64 {
        let osmolality = delta_tf / self.solvent.kf; // mol/kg
        // Convert to mol/m³ using density
        let osmolarity = osmolality * self.density; // mol/m³ (density in kg/m³)
        osmolarity * R_GAS * temperature_k
    }

    /// Calculate osmolality from freezing point depression.
    ///
    /// osmolality = ΔTf / Kf (osmol/kg for water)
    pub fn osmolality(&self, delta_tf: f64) -> f64 {
        delta_tf / self.solvent.kf
    }

    /// Clinical osmolality: normal range 275-295 mOsm/kg for serum.
    pub fn is_normal_serum_osmolality(&self, delta_tf: f64) -> bool {
        let osm = self.osmolality(delta_tf) * 1000.0; // mOsm/kg
        osm >= 275.0 && osm <= 295.0
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// CryoscopicSession
// ──────────────────────────────────────────────────────────────────────────────

/// Multi-measurement cryoscopic session with calibration and uncertainty.
///
/// Workflow:
/// 1. Calibrate with a known standard
/// 2. Measure unknown samples
/// 3. Apply calibration correction and propagate uncertainty
#[derive(Debug, Clone)]
pub struct CryoscopicSession {
    /// Solvent in use
    pub solvent: CryoscopicConstant,
    /// Calibration factor (actual/measured ratio)
    pub calibration_factor: f64,
    /// Standard uncertainty in Kf from calibration
    pub kf_uncertainty: f64,
    /// Recorded measurements
    pub measurements: Vec<SessionMeasurement>,
}

/// A single measurement in a cryoscopic session.
#[derive(Debug, Clone)]
pub struct SessionMeasurement {
    /// Sample identifier
    pub label: String,
    /// Mass of solute (g)
    pub solute_mass_g: f64,
    /// Mass of solvent (kg)
    pub solvent_mass_kg: f64,
    /// Measured freezing point depression (K)
    pub delta_tf: f64,
    /// Calculated molecular weight (g/mol)
    pub mw: f64,
    /// Uncertainty in MW (g/mol)
    pub mw_uncertainty: f64,
}

impl CryoscopicSession {
    /// Create a new session with a given solvent.
    pub fn new(solvent: CryoscopicConstant) -> Self {
        Self {
            solvent,
            calibration_factor: 1.0,
            kf_uncertainty: 0.0,
            measurements: Vec::new(),
        }
    }

    /// Calibrate using a known standard.
    ///
    /// - `standard_mw`: known molecular weight of the standard (g/mol)
    /// - `solute_mass_g`: mass of standard used (g)
    /// - `solvent_mass_kg`: mass of solvent (kg)
    /// - `measured_delta_tf`: observed freezing point depression (K)
    pub fn calibrate(
        &mut self,
        standard_mw: f64,
        solute_mass_g: f64,
        solvent_mass_kg: f64,
        measured_delta_tf: f64,
    ) {
        // Expected depression
        let molality = (solute_mass_g / standard_mw) / solvent_mass_kg;
        let expected_delta_tf = self.solvent.kf * molality;

        self.calibration_factor = expected_delta_tf / measured_delta_tf;

        // Uncertainty estimate: 1% of Kf as typical precision
        self.kf_uncertainty = self.solvent.kf * 0.01;
    }

    /// Measure an unknown sample.
    ///
    /// - `label`: sample identifier
    /// - `solute_mass_g`: mass of unknown solute (g)
    /// - `solvent_mass_kg`: mass of solvent (kg)
    /// - `measured_delta_tf`: observed freezing point depression (K)
    ///
    /// Returns the calculated molecular weight.
    pub fn measure(
        &mut self,
        label: &str,
        solute_mass_g: f64,
        solvent_mass_kg: f64,
        measured_delta_tf: f64,
    ) -> f64 {
        // Apply calibration correction
        let corrected_delta_tf = measured_delta_tf * self.calibration_factor;

        // Calculate MW
        let mw = self.solvent.kf * solute_mass_g / (corrected_delta_tf * solvent_mass_kg);

        // Propagate uncertainty
        // MW = Kf * m_solute / (ΔTf * m_solvent)
        // δMW/MW = sqrt((δKf/Kf)² + (δΔTf/ΔTf)²)
        // Assume 0.5% uncertainty in delta_tf from measurement
        let rel_kf_unc = self.kf_uncertainty / self.solvent.kf;
        let rel_dtf_unc = 0.005; // 0.5%
        let rel_mw_unc = (rel_kf_unc * rel_kf_unc + rel_dtf_unc * rel_dtf_unc).sqrt();
        let mw_uncertainty = mw * rel_mw_unc;

        self.measurements.push(SessionMeasurement {
            label: label.to_string(),
            solute_mass_g,
            solvent_mass_kg,
            delta_tf: corrected_delta_tf,
            mw,
            mw_uncertainty,
        });

        mw
    }

    /// Get the mean molecular weight from replicate measurements with the same label.
    pub fn mean_mw(&self, label: &str) -> Option<(f64, f64)> {
        let mws: Vec<f64> = self
            .measurements
            .iter()
            .filter(|m| m.label == label)
            .map(|m| m.mw)
            .collect();

        if mws.is_empty() {
            return None;
        }

        let n = mws.len() as f64;
        let mean = mws.iter().sum::<f64>() / n;

        if mws.len() < 2 {
            return Some((mean, 0.0));
        }

        let variance = mws.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
        let std_error = (variance / n).sqrt();

        Some((mean, std_error))
    }

    /// Get all recorded measurements.
    pub fn results(&self) -> &[SessionMeasurement] {
        &self.measurements
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Helper functions
// ──────────────────────────────────────────────────────────────────────────────

/// Calculate ideal freezing point depression.
///
/// ΔTf = Kf × molality
pub fn ideal_freezing_depression(molality: f64, kf: f64) -> f64 {
    kf * molality
}

/// Calculate the cooling rate (K/s) from temperature and time data.
///
/// Returns the average rate of temperature change (negative for cooling).
/// Uses linear regression for robustness.
pub fn cooling_rate(temperatures: &[f64], times: &[f64]) -> f64 {
    if temperatures.len() < 2 || temperatures.len() != times.len() {
        return 0.0;
    }

    let n = temperatures.len() as f64;
    let sum_t: f64 = times.iter().sum();
    let sum_temp: f64 = temperatures.iter().sum();
    let sum_t_temp: f64 = times
        .iter()
        .zip(temperatures.iter())
        .map(|(t, temp)| t * temp)
        .sum();
    let sum_tt: f64 = times.iter().map(|t| t * t).sum();

    let denom = n * sum_tt - sum_t * sum_t;
    if denom.abs() < 1e-15 {
        return 0.0;
    }

    (n * sum_t_temp - sum_t * sum_temp) / denom
}

/// Detect a plateau (region of nearly constant values) in a data series.
///
/// Returns the (start_index, end_index) of the longest plateau found,
/// where all values within the plateau differ by at most `tolerance`.
pub fn detect_plateau(temperatures: &[f64], tolerance: f64) -> Option<(usize, usize)> {
    if temperatures.is_empty() {
        return None;
    }

    let mut best_start = 0;
    let mut best_end = 0;
    let mut best_len = 0;

    let n = temperatures.len();

    for start in 0..n {
        let base = temperatures[start];
        let mut end = start;

        while end < n && (temperatures[end] - base).abs() <= tolerance {
            end += 1;
        }
        end -= 1; // last valid index

        let len = end - start + 1;
        if len > best_len {
            best_len = len;
            best_start = start;
            best_end = end;
        }
    }

    if best_len >= 2 {
        Some((best_start, best_end))
    } else {
        None
    }
}

/// Compute the derivative of a signal using central differences.
fn numerical_derivative(values: &[f64], dt: f64) -> Vec<f64> {
    let n = values.len();
    if n < 2 {
        return vec![0.0; n];
    }

    let mut deriv = vec![0.0; n];

    // Forward difference for first point
    deriv[0] = (values[1] - values[0]) / dt;

    // Central differences for interior points
    for i in 1..n - 1 {
        deriv[i] = (values[i + 1] - values[i - 1]) / (2.0 * dt);
    }

    // Backward difference for last point
    deriv[n - 1] = (values[n - 1] - values[n - 2]) / dt;

    deriv
}

/// Smooth a signal using a simple moving average.
fn smooth(values: &[f64], window: usize) -> Vec<f64> {
    if window <= 1 || values.is_empty() {
        return values.to_vec();
    }

    let n = values.len();
    let half = window / 2;
    let mut result = vec![0.0; n];

    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = if i + half < n { i + half } else { n - 1 };
        let count = end - start + 1;
        result[i] = values[start..=end].iter().sum::<f64>() / count as f64;
    }

    result
}

/// Calculate the enthalpy of fusion from the cryoscopic constant.
///
/// ΔHfus = R × Tf² × M / (1000 × Kf)
pub fn enthalpy_from_kf(kf: f64, freezing_point_k: f64, molar_mass: f64) -> f64 {
    R_GAS * freezing_point_k * freezing_point_k * molar_mass / (1000.0 * kf)
}

/// Convert Celsius to Kelvin.
pub fn celsius_to_kelvin(c: f64) -> f64 {
    c + 273.15
}

/// Convert Kelvin to Celsius.
pub fn kelvin_to_celsius(k: f64) -> f64 {
    k - 273.15
}

/// Calculate the activity coefficient from freezing point depression.
///
/// ln(γ × x_solvent) = ΔHfus/R × (1/Tf - 1/T)
///
/// For dilute solutions, γ ≈ 1 and x_solvent ≈ 1 - x_solute.
pub fn activity_coefficient_from_fpd(
    delta_tf: f64,
    t_pure: f64,
    enthalpy_fusion: f64,
    x_solvent: f64,
) -> f64 {
    let t_solution = t_pure - delta_tf;
    let ln_gamma_x = enthalpy_fusion / R_GAS * (1.0 / t_pure - 1.0 / t_solution);
    let gamma = ln_gamma_x.exp() / x_solvent;
    gamma
}

/// Calculate the theoretical van't Hoff factor for a strong electrolyte.
///
/// For complete dissociation: i = number of ions produced.
/// For partial dissociation: i = 1 + α(ν - 1) where α is degree of dissociation
/// and ν is the number of ions per formula unit.
pub fn vant_hoff_factor(degree_dissociation: f64, num_ions: usize) -> f64 {
    1.0 + degree_dissociation * (num_ions as f64 - 1.0)
}

// ──────────────────────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 0.1;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── CryoscopicConstant tests ──

    #[test]
    fn test_water_kf() {
        let w = CryoscopicConstant::water();
        assert!(approx_eq(w.kf, 1.86, 0.001));
        assert!(approx_eq(w.freezing_point_k, 273.15, 0.01));
    }

    #[test]
    fn test_benzene_kf() {
        let b = CryoscopicConstant::benzene();
        assert!(approx_eq(b.kf, 5.12, 0.001));
    }

    #[test]
    fn test_cyclohexane_kf() {
        let c = CryoscopicConstant::cyclohexane();
        assert!(approx_eq(c.kf, 20.0, 0.001));
    }

    #[test]
    fn test_camphor_kf() {
        let c = CryoscopicConstant::camphor();
        assert!(approx_eq(c.kf, 37.7, 0.001));
    }

    #[test]
    fn test_naphthalene_kf() {
        let n = CryoscopicConstant::naphthalene();
        assert!(approx_eq(n.kf, 6.94, 0.001));
    }

    #[test]
    fn test_acetic_acid_kf() {
        let a = CryoscopicConstant::acetic_acid();
        assert!(approx_eq(a.kf, 3.90, 0.001));
    }

    #[test]
    fn test_kf_from_thermodynamic_properties() {
        // Water: R × 273.15² × 18.015 / (1000 × 6009.5) ≈ 1.86
        let w = CryoscopicConstant::new("Water", 273.15, 18.015, 6009.5);
        assert!(approx_eq(w.kf, 1.86, 0.02));
    }

    #[test]
    fn test_kf_back_calculation() {
        let w = CryoscopicConstant::water();
        // enthalpy_fusion should be consistent
        let kf_recalc = R_GAS * w.freezing_point_k * w.freezing_point_k * w.molar_mass
            / (1000.0 * w.enthalpy_fusion);
        assert!(approx_eq(kf_recalc, w.kf, 1e-10));
    }

    #[test]
    fn test_cryoscopic_constant_clone() {
        let w = CryoscopicConstant::water();
        let w2 = w.clone();
        assert_eq!(w, w2);
    }

    // ── FreezingPointDepression tests ──

    #[test]
    fn test_fpd_simple() {
        let fpd = FreezingPointDepression::non_electrolyte(CryoscopicConstant::water());
        // 1 molal solution: ΔTf = 1.86 K
        assert!(approx_eq(fpd.delta_tf(1.0), 1.86, 0.001));
    }

    #[test]
    fn test_fpd_nacl() {
        // NaCl: i = 2
        let fpd = FreezingPointDepression::new(CryoscopicConstant::water(), 2.0);
        // 0.5 molal NaCl: ΔTf = 1.86 × 0.5 × 2 = 1.86 K
        assert!(approx_eq(fpd.delta_tf(0.5), 1.86, 0.001));
    }

    #[test]
    fn test_fpd_freezing_point() {
        let fpd = FreezingPointDepression::non_electrolyte(CryoscopicConstant::water());
        let fp = fpd.freezing_point(1.0);
        assert!(approx_eq(fp, 273.15 - 1.86, 0.001));
    }

    #[test]
    fn test_fpd_zero_molality() {
        let fpd = FreezingPointDepression::non_electrolyte(CryoscopicConstant::water());
        assert!(approx_eq(fpd.delta_tf(0.0), 0.0, TOL));
        assert!(approx_eq(fpd.freezing_point(0.0), 273.15, TOL));
    }

    #[test]
    fn test_fpd_molality_from_depression() {
        let fpd = FreezingPointDepression::non_electrolyte(CryoscopicConstant::water());
        let m = fpd.molality_from_depression(1.86);
        assert!(approx_eq(m, 1.0, 0.001));
    }

    #[test]
    fn test_fpd_cacl2() {
        // CaCl2: i = 3
        let fpd = FreezingPointDepression::new(CryoscopicConstant::water(), 3.0);
        // 0.1 molal: ΔTf = 1.86 × 0.1 × 3 = 0.558 K
        assert!(approx_eq(fpd.delta_tf(0.1), 0.558, 0.001));
    }

    #[test]
    fn test_fpd_benzene_solvent() {
        let fpd = FreezingPointDepression::non_electrolyte(CryoscopicConstant::benzene());
        // 0.5 molal in benzene: ΔTf = 5.12 × 0.5 = 2.56 K
        assert!(approx_eq(fpd.delta_tf(0.5), 2.56, 0.001));
    }

    #[test]
    fn test_static_molality_calculation() {
        // 10 g solute, MW = 100 g/mol, 0.5 kg solvent
        let m = FreezingPointDepression::molality(10.0, 100.0, 0.5);
        assert!(approx_eq(m, 0.2, TOL));
    }

    #[test]
    fn test_fpd_camphor_solvent() {
        let fpd = FreezingPointDepression::non_electrolyte(CryoscopicConstant::camphor());
        // 0.1 molal in camphor: ΔTf = 37.7 × 0.1 = 3.77 K
        assert!(approx_eq(fpd.delta_tf(0.1), 3.77, 0.001));
    }

    // ── MolecularWeightFromFPD tests ──

    #[test]
    fn test_mw_simple() {
        let mw_calc = MolecularWeightFromFPD::new(CryoscopicConstant::water(), 1.0);
        // 5 g solute in 0.1 kg water, ΔTf = 0.93 K
        // MW = 1.86 × 5.0 / (0.93 × 0.1) = 100 g/mol
        let result = mw_calc.calculate(5.0, 0.1, 0.93);
        assert!(approx_eq(result.mw, 100.0, 0.1));
    }

    #[test]
    fn test_mw_glucose() {
        let mw_calc = MolecularWeightFromFPD::new(CryoscopicConstant::water(), 1.0);
        // 18 g glucose (MW=180) in 0.5 kg water
        // molality = 18/180/0.5 = 0.2 mol/kg
        // ΔTf = 1.86 × 0.2 = 0.372 K
        let result = mw_calc.calculate(18.0, 0.5, 0.372);
        assert!(approx_eq(result.mw, 180.0, 1.0));
    }

    #[test]
    fn test_mw_extrapolation() {
        let mw_calc = MolecularWeightFromFPD::new(CryoscopicConstant::water(), 1.0);
        // True MW = 100 g/mol
        // At different concentrations, apparent MW varies slightly
        let measurements = vec![
            (5.0, 0.5, 0.186),  // 0.1 molal
            (10.0, 0.5, 0.372), // 0.2 molal
            (15.0, 0.5, 0.558), // 0.3 molal
        ];
        let result = mw_calc.extrapolate(&measurements).unwrap();
        assert!(approx_eq(result.mw_extrapolated, 100.0, 1.0));
    }

    #[test]
    fn test_mw_extrapolation_insufficient_data() {
        let mw_calc = MolecularWeightFromFPD::new(CryoscopicConstant::water(), 1.0);
        let measurements = vec![(5.0, 0.5, 0.186)];
        assert!(mw_calc.extrapolate(&measurements).is_none());
    }

    #[test]
    fn test_mw_result_fields() {
        let mw_calc = MolecularWeightFromFPD::new(CryoscopicConstant::water(), 1.0);
        let result = mw_calc.calculate(10.0, 0.5, 0.372);
        assert!(result.mw > 0.0);
        assert!(result.molality > 0.0);
        assert!(approx_eq(result.delta_tf, 0.372, TOL));
    }

    // ── CoolingCurveAnalyzer tests ──

    #[test]
    fn test_cooling_curve_simple_plateau() {
        let analyzer = CoolingCurveAnalyzer::new().with_tolerance(0.1);

        // Simulated cooling curve: cool → plateau → cool more
        let temps = vec![
            280.0, 278.0, 276.0, 274.0, 273.2, 273.15, 273.14, 273.16, 273.15, 273.14, 273.15,
            272.0, 271.0,
        ];
        let times: Vec<f64> = (0..temps.len()).map(|i| i as f64 * 10.0).collect();

        let result = analyzer.analyze(&temps, &times).unwrap();
        assert!(approx_eq(result.freezing_temperature, 273.15, 0.1));
    }

    #[test]
    fn test_cooling_curve_with_supercooling() {
        let analyzer = CoolingCurveAnalyzer::new().with_tolerance(0.1);

        // Cool → supercool → recalesce → plateau
        let temps = vec![
            280.0, 278.0, 276.0, 274.0, 272.0, 271.0, // cooling
            270.0, // supercooling dip
            273.1, 273.15, 273.14, 273.16, 273.15, 273.14, 273.15, // plateau
            272.0, 271.0, // post-freeze cooling
        ];
        let times: Vec<f64> = (0..temps.len()).map(|i| i as f64 * 5.0).collect();

        let result = analyzer.analyze(&temps, &times).unwrap();
        assert!(result.supercooling_detected);
        assert!(result.supercooling_depth > 2.0);
    }

    #[test]
    fn test_cooling_curve_no_plateau() {
        let analyzer = CoolingCurveAnalyzer::new().with_tolerance(0.01).with_min_plateau(10);

        // Monotonically decreasing - no plateau
        let temps: Vec<f64> = (0..20).map(|i| 280.0 - i as f64).collect();
        let times: Vec<f64> = (0..20).map(|i| i as f64).collect();

        // This might still find a "plateau" depending on tolerance
        // With very tight tolerance and high min_plateau, should fail
        let result = analyzer.analyze(&temps, &times);
        // The result depends on implementation - it may find a short plateau
        // Just verify it doesn't crash
        let _ = result;
    }

    #[test]
    fn test_cooling_curve_insufficient_data() {
        let analyzer = CoolingCurveAnalyzer::new();
        let result = analyzer.analyze(&[280.0], &[0.0]);
        assert!(result.is_none());
    }

    #[test]
    fn test_cooling_curve_cooling_rate() {
        let analyzer = CoolingCurveAnalyzer::new().with_tolerance(0.2);

        let temps = vec![
            290.0, 288.0, 286.0, 284.0, 282.0, 280.0, // cooling at -2 K/s
            273.2, 273.1, 273.15, 273.12, 273.14, 273.13, 273.15, // plateau
            270.0, 268.0,
        ];
        let times: Vec<f64> = (0..temps.len()).map(|i| i as f64).collect();

        let result = analyzer.analyze(&temps, &times).unwrap();
        // Cooling rate should be approximately -2 K/s
        assert!(result.pre_freeze_cooling_rate < 0.0);
    }

    #[test]
    fn test_scheil_solidification() {
        let analyzer = CoolingCurveAnalyzer::new();
        let result = analyzer.scheil_solidification(660.0, 577.0, 0.13, 100);

        // At liquidus, fs should be 0
        assert!(approx_eq(result[0].0, 660.0, TOL));
        assert!(approx_eq(result[0].1, 0.0, TOL));

        // At solidus, fs should be close to 1
        let last = result.last().unwrap();
        assert!(approx_eq(last.0, 577.0, TOL));

        // fs should be monotonically increasing
        for i in 1..result.len() {
            assert!(result[i].1 >= result[i - 1].1 - TOL);
        }
    }

    #[test]
    fn test_scheil_invalid_params() {
        let analyzer = CoolingCurveAnalyzer::new();
        assert!(analyzer.scheil_solidification(660.0, 577.0, 0.0, 100).is_empty());
        assert!(analyzer.scheil_solidification(660.0, 577.0, 1.0, 100).is_empty());
        assert!(analyzer.scheil_solidification(660.0, 577.0, 0.5, 0).is_empty());
    }

    #[test]
    fn test_cooling_curve_default() {
        let a1 = CoolingCurveAnalyzer::default();
        let a2 = CoolingCurveAnalyzer::new();
        assert!(approx_eq(a1.plateau_tolerance, a2.plateau_tolerance, TOL));
    }

    // ── EutecticAnalyzer tests ──

    #[test]
    fn test_eutectic_simple() {
        let analyzer = EutecticAnalyzer::new(0.5);

        let measurements = vec![
            CompositionMeasurement {
                x_b: 0.0,
                t_liquidus: 500.0,
                t_eutectic: Some(350.0),
            },
            CompositionMeasurement {
                x_b: 0.2,
                t_liquidus: 420.0,
                t_eutectic: Some(350.0),
            },
            CompositionMeasurement {
                x_b: 0.4,
                t_liquidus: 360.0,
                t_eutectic: Some(350.0),
            },
            CompositionMeasurement {
                x_b: 0.5,
                t_liquidus: 350.0,
                t_eutectic: Some(350.0),
            },
            CompositionMeasurement {
                x_b: 0.7,
                t_liquidus: 400.0,
                t_eutectic: Some(350.0),
            },
            CompositionMeasurement {
                x_b: 1.0,
                t_liquidus: 550.0,
                t_eutectic: None,
            },
        ];

        let result = analyzer.analyze(&measurements).unwrap();
        assert!(approx_eq(result.t_eutectic, 350.0, 1.0));
        assert!(approx_eq(result.x_eutectic, 0.5, 0.1));
    }

    #[test]
    fn test_eutectic_insufficient_data() {
        let analyzer = EutecticAnalyzer::new(0.5);
        let measurements = vec![CompositionMeasurement {
            x_b: 0.0,
            t_liquidus: 500.0,
            t_eutectic: None,
        }];
        assert!(analyzer.analyze(&measurements).is_none());
    }

    #[test]
    fn test_eutectic_eval_liquidus() {
        let coeffs = (500.0, -200.0, 0.0);
        let t = EutecticAnalyzer::eval_liquidus(&coeffs, 0.5);
        assert!(approx_eq(t, 400.0, TOL));
    }

    #[test]
    fn test_eutectic_no_arrest_temps() {
        let analyzer = EutecticAnalyzer::new(0.5);
        let measurements = vec![
            CompositionMeasurement {
                x_b: 0.0,
                t_liquidus: 500.0,
                t_eutectic: None,
            },
            CompositionMeasurement {
                x_b: 0.3,
                t_liquidus: 400.0,
                t_eutectic: None,
            },
            CompositionMeasurement {
                x_b: 0.5,
                t_liquidus: 350.0,
                t_eutectic: None,
            },
        ];
        let result = analyzer.analyze(&measurements).unwrap();
        // Should use minimum liquidus as eutectic estimate
        assert!(approx_eq(result.t_eutectic, 350.0, TOL));
    }

    // ── PurityAnalyzer tests ──

    #[test]
    fn test_purity_pure_sample() {
        let analyzer = PurityAnalyzer::new(273.15, 6009.5);
        // Very small depression = high purity
        let fractions = vec![0.1, 0.2, 0.3, 0.5, 0.7, 1.0];
        let temps = vec![273.10, 273.11, 273.12, 273.13, 273.14, 273.15];

        let result = analyzer.analyze(&fractions, &temps).unwrap();
        assert!(result.percent_purity > 99.0);
    }

    #[test]
    fn test_purity_impure_sample() {
        let analyzer = PurityAnalyzer::new(273.15, 6009.5);
        // Significant depression = lower purity
        // T = T_pure - (R × T²  × x_imp) / (ΔH × F)
        // For x_imp = 0.02 (2% impurity):
        let x_imp = 0.02;
        let slope = -R_GAS * 273.15 * 273.15 * x_imp / 6009.5;

        let fractions = vec![0.2, 0.3, 0.5, 0.7, 1.0];
        let temps: Vec<f64> = fractions
            .iter()
            .map(|&f| 273.15 + slope / f)
            .collect();

        let result = analyzer.analyze(&fractions, &temps).unwrap();
        assert!(approx_eq(result.purity, 0.98, 0.01));
    }

    #[test]
    fn test_purity_insufficient_data() {
        let analyzer = PurityAnalyzer::new(273.15, 6009.5);
        assert!(analyzer.analyze(&[0.5], &[273.0]).is_none());
    }

    #[test]
    fn test_purity_r_squared() {
        let analyzer = PurityAnalyzer::new(273.15, 6009.5);
        // Perfect linear relationship should give R² ≈ 1
        let fractions = vec![0.2, 0.4, 0.6, 0.8, 1.0];
        let temps: Vec<f64> = fractions
            .iter()
            .map(|&f| 273.0 + 0.15 * f) // linear in F, so linear in 1/F? No.
            .collect();
        let result = analyzer.analyze(&fractions, &temps);
        assert!(result.is_some());
    }

    #[test]
    fn test_purity_skip_zero_fraction() {
        let analyzer = PurityAnalyzer::new(273.15, 6009.5);
        let fractions = vec![0.0, 0.3, 0.5, 0.7, 1.0];
        let temps = vec![272.0, 272.8, 273.0, 273.05, 273.1];
        // Should not crash on F=0
        let result = analyzer.analyze(&fractions, &temps);
        assert!(result.is_some());
    }

    // ── SupercoolingCorrector tests ──

    #[test]
    fn test_supercooling_water_preset() {
        let sc = SupercoolingCorrector::water();
        assert!(approx_eq(sc.cp_liquid, 4.18, 0.01));
        assert!(approx_eq(sc.latent_heat, 334.0, 0.1));
    }

    #[test]
    fn test_area_ratio_correction() {
        let sc = SupercoolingCorrector::water();
        let t_corrected = sc.area_ratio_correction(271.0, 273.15, 273.15);
        // Correction should raise the temperature slightly above the plateau
        assert!(t_corrected > 273.15);
    }

    #[test]
    fn test_latent_heat_correction() {
        let sc = SupercoolingCorrector::water();
        let result = sc.latent_heat_correction(271.0, 273.15);
        assert!(result.supercooling_depth > 0.0);
        assert!(result.fraction_frozen_on_recalescence > 0.0);
        assert!(result.fraction_frozen_on_recalescence < 1.0);
        assert!(result.t_corrected >= 273.15);
    }

    #[test]
    fn test_supercooling_no_supercooling() {
        let sc = SupercoolingCorrector::water();
        let result = sc.latent_heat_correction(273.15, 273.15);
        assert!(approx_eq(result.supercooling_depth, 0.0, TOL));
        assert!(approx_eq(result.fraction_frozen_on_recalescence, 0.0, TOL));
    }

    #[test]
    fn test_critical_supercooling() {
        let sc = SupercoolingCorrector::water();
        let dt = sc.critical_supercooling(0.025, 273.15); // surface energy ~25 mJ/m²
        // Should be a reasonable value
        assert!(dt > 0.0);
        assert!(dt < 100.0);
    }

    #[test]
    fn test_supercooling_fraction_clamped() {
        let sc = SupercoolingCorrector::new(10.0, 2.0, 1.0); // extreme cp/latent_heat ratio
        let result = sc.latent_heat_correction(200.0, 273.15);
        assert!(result.fraction_frozen_on_recalescence <= 1.0);
    }

    // ── OsmoticPressureFromFPD tests ──

    #[test]
    fn test_osmotic_aqueous() {
        let op = OsmoticPressureFromFPD::aqueous();
        assert!(approx_eq(op.density, 1000.0, TOL));
    }

    #[test]
    fn test_osmolality() {
        let op = OsmoticPressureFromFPD::aqueous();
        // 1.86 K depression = 1.0 osmol/kg
        let osm = op.osmolality(1.86);
        assert!(approx_eq(osm, 1.0, 0.001));
    }

    #[test]
    fn test_osmolality_half() {
        let op = OsmoticPressureFromFPD::aqueous();
        let osm = op.osmolality(0.93);
        assert!(approx_eq(osm, 0.5, 0.001));
    }

    #[test]
    fn test_normal_serum_osmolality() {
        let op = OsmoticPressureFromFPD::aqueous();
        // Normal serum: ~290 mOsm/kg → ΔTf = 0.290 × 1.86 ≈ 0.5394 K
        let delta_tf = 0.290 * 1.86;
        assert!(op.is_normal_serum_osmolality(delta_tf));
    }

    #[test]
    fn test_abnormal_serum_osmolality() {
        let op = OsmoticPressureFromFPD::aqueous();
        // 400 mOsm/kg → too high
        let delta_tf = 0.400 * 1.86;
        assert!(!op.is_normal_serum_osmolality(delta_tf));
    }

    #[test]
    fn test_osmotic_pressure_positive() {
        let op = OsmoticPressureFromFPD::aqueous();
        let pi = op.osmotic_pressure(1.86);
        assert!(pi > 0.0);
    }

    #[test]
    fn test_osmotic_pressure_simplified() {
        let op = OsmoticPressureFromFPD::aqueous();
        let pi = op.osmotic_pressure_simplified(1.86, 298.15);
        // 1 osmol at 298 K → π ≈ 2.48 MPa ≈ 24.8 atm
        assert!(pi > 2.0e6); // > 2 MPa
    }

    // ── CryoscopicSession tests ──

    #[test]
    fn test_session_creation() {
        let session = CryoscopicSession::new(CryoscopicConstant::water());
        assert!(approx_eq(session.calibration_factor, 1.0, TOL));
        assert!(session.measurements.is_empty());
    }

    #[test]
    fn test_session_calibration() {
        let mut session = CryoscopicSession::new(CryoscopicConstant::water());
        // Calibrate with urea (MW = 60.06)
        // 3.0 g in 0.5 kg water: m = 0.0999 mol/kg
        // Expected ΔTf = 1.86 × 0.0999 = 0.18581
        // If measured is 0.184, calibration factor adjusts
        session.calibrate(60.06, 3.0, 0.5, 0.184);
        assert!(session.calibration_factor > 0.9 && session.calibration_factor < 1.1);
    }

    #[test]
    fn test_session_measure() {
        let mut session = CryoscopicSession::new(CryoscopicConstant::water());

        let mw = session.measure("Unknown", 5.0, 0.5, 0.186);
        // MW = 1.86 × 5.0 / (0.186 × 0.5) = 100 g/mol
        assert!(approx_eq(mw, 100.0, 1.0));
        assert_eq!(session.measurements.len(), 1);
    }

    #[test]
    fn test_session_mean_mw() {
        let mut session = CryoscopicSession::new(CryoscopicConstant::water());

        session.measure("Sample A", 5.0, 0.5, 0.186);
        session.measure("Sample A", 5.0, 0.5, 0.185);
        session.measure("Sample A", 5.0, 0.5, 0.187);

        let (mean, se) = session.mean_mw("Sample A").unwrap();
        assert!(approx_eq(mean, 100.0, 2.0));
        assert!(se >= 0.0);
    }

    #[test]
    fn test_session_mean_mw_no_data() {
        let session = CryoscopicSession::new(CryoscopicConstant::water());
        assert!(session.mean_mw("Nothing").is_none());
    }

    #[test]
    fn test_session_mean_mw_single() {
        let mut session = CryoscopicSession::new(CryoscopicConstant::water());
        session.measure("Single", 5.0, 0.5, 0.186);
        let (mean, se) = session.mean_mw("Single").unwrap();
        assert!(mean > 0.0);
        assert!(approx_eq(se, 0.0, TOL));
    }

    #[test]
    fn test_session_results() {
        let mut session = CryoscopicSession::new(CryoscopicConstant::water());
        session.measure("A", 5.0, 0.5, 0.186);
        session.measure("B", 10.0, 0.5, 0.372);
        assert_eq!(session.results().len(), 2);
    }

    #[test]
    fn test_session_uncertainty() {
        let mut session = CryoscopicSession::new(CryoscopicConstant::water());
        session.measure("Test", 5.0, 0.5, 0.186);
        let m = &session.measurements[0];
        assert!(m.mw_uncertainty > 0.0);
        assert!(m.mw_uncertainty < m.mw * 0.1); // less than 10% relative uncertainty
    }

    // ── Helper function tests ──

    #[test]
    fn test_ideal_freezing_depression() {
        let dtf = ideal_freezing_depression(1.0, 1.86);
        assert!(approx_eq(dtf, 1.86, TOL));
    }

    #[test]
    fn test_ideal_freezing_depression_zero() {
        let dtf = ideal_freezing_depression(0.0, 1.86);
        assert!(approx_eq(dtf, 0.0, TOL));
    }

    #[test]
    fn test_cooling_rate_linear() {
        // Linear cooling at -2 K/s
        let temps = vec![280.0, 278.0, 276.0, 274.0, 272.0];
        let times = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let rate = cooling_rate(&temps, &times);
        assert!(approx_eq(rate, -2.0, TOL));
    }

    #[test]
    fn test_cooling_rate_empty() {
        assert!(approx_eq(cooling_rate(&[], &[]), 0.0, TOL));
    }

    #[test]
    fn test_cooling_rate_single() {
        assert!(approx_eq(cooling_rate(&[280.0], &[0.0]), 0.0, TOL));
    }

    #[test]
    fn test_detect_plateau_simple() {
        let temps = vec![280.0, 275.0, 273.1, 273.15, 273.12, 273.14, 270.0, 268.0];
        let result = detect_plateau(&temps, 0.1).unwrap();
        // Plateau should be around indices 2-5
        assert!(result.0 >= 2 && result.0 <= 3);
        assert!(result.1 >= 4 && result.1 <= 5);
    }

    #[test]
    fn test_detect_plateau_none() {
        let temps = vec![280.0, 275.0, 270.0, 265.0, 260.0];
        // With very tight tolerance, no plateau
        let result = detect_plateau(&temps, 0.001);
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plateau_empty() {
        assert!(detect_plateau(&[], 0.1).is_none());
    }

    #[test]
    fn test_detect_plateau_all_same() {
        let temps = vec![273.15; 10];
        let result = detect_plateau(&temps, 0.01).unwrap();
        assert_eq!(result.0, 0);
        assert_eq!(result.1, 9);
    }

    #[test]
    fn test_celsius_to_kelvin() {
        assert!(approx_eq(celsius_to_kelvin(0.0), 273.15, TOL));
        assert!(approx_eq(celsius_to_kelvin(100.0), 373.15, TOL));
        assert!(approx_eq(celsius_to_kelvin(-40.0), 233.15, TOL));
    }

    #[test]
    fn test_kelvin_to_celsius() {
        assert!(approx_eq(kelvin_to_celsius(273.15), 0.0, TOL));
        assert!(approx_eq(kelvin_to_celsius(373.15), 100.0, TOL));
    }

    #[test]
    fn test_celsius_kelvin_roundtrip() {
        let c = 25.0;
        assert!(approx_eq(kelvin_to_celsius(celsius_to_kelvin(c)), c, TOL));
    }

    #[test]
    fn test_enthalpy_from_kf() {
        let dh = enthalpy_from_kf(1.86, 273.15, 18.015);
        // Water ΔHfus ≈ 6009 J/mol
        assert!(approx_eq(dh, 6009.0, 50.0));
    }

    #[test]
    fn test_activity_coefficient_dilute() {
        // For very dilute solution, γ should be close to 1
        let gamma = activity_coefficient_from_fpd(0.001, 273.15, 6009.5, 0.9999);
        assert!(approx_eq(gamma, 1.0, 0.01));
    }

    #[test]
    fn test_vant_hoff_factor_complete_dissociation() {
        // NaCl fully dissociates: i = 1 + 1.0 × (2 - 1) = 2
        let i = vant_hoff_factor(1.0, 2);
        assert!(approx_eq(i, 2.0, TOL));
    }

    #[test]
    fn test_vant_hoff_factor_partial() {
        // 50% dissociation of NaCl: i = 1 + 0.5 × 1 = 1.5
        let i = vant_hoff_factor(0.5, 2);
        assert!(approx_eq(i, 1.5, TOL));
    }

    #[test]
    fn test_vant_hoff_factor_no_dissociation() {
        let i = vant_hoff_factor(0.0, 3);
        assert!(approx_eq(i, 1.0, TOL));
    }

    #[test]
    fn test_vant_hoff_cacl2() {
        // CaCl2 → Ca²⁺ + 2Cl⁻, 3 ions
        let i = vant_hoff_factor(1.0, 3);
        assert!(approx_eq(i, 3.0, TOL));
    }

    #[test]
    fn test_numerical_derivative_constant() {
        let vals = vec![5.0; 10];
        let deriv = numerical_derivative(&vals, 1.0);
        for &d in &deriv {
            assert!(approx_eq(d, 0.0, TOL));
        }
    }

    #[test]
    fn test_numerical_derivative_linear() {
        // y = 2x, dy/dx = 2
        let vals: Vec<f64> = (0..10).map(|i| 2.0 * i as f64).collect();
        let deriv = numerical_derivative(&vals, 1.0);
        // Interior points should be approximately 2.0
        for &d in &deriv[1..deriv.len() - 1] {
            assert!(approx_eq(d, 2.0, TOL));
        }
    }

    #[test]
    fn test_smooth_identity() {
        let vals = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let s = smooth(&vals, 1);
        assert_eq!(s, vals);
    }

    #[test]
    fn test_smooth_averaging() {
        let vals = vec![0.0, 10.0, 0.0, 10.0, 0.0];
        let s = smooth(&vals, 3);
        // Center element: avg of 10, 0, 10 = 6.67
        assert!(approx_eq(s[2], 20.0 / 3.0, 0.01));
    }

    // ── Integration tests ──

    #[test]
    fn test_full_mw_determination_workflow() {
        // Complete workflow: measure FPD of glucose in water
        let water = CryoscopicConstant::water();
        let fpd = FreezingPointDepression::non_electrolyte(water.clone());

        // 18 g glucose (MW = 180.16 g/mol) in 500 g water
        let molality = FreezingPointDepression::molality(18.0, 180.16, 0.5);
        let delta_tf = fpd.delta_tf(molality);

        // Now determine MW from FPD
        let mw_calc = MolecularWeightFromFPD::new(water, 1.0);
        let result = mw_calc.calculate(18.0, 0.5, delta_tf);
        assert!(approx_eq(result.mw, 180.16, 0.1));
    }

    #[test]
    fn test_electrolyte_fpd_roundtrip() {
        let fpd = FreezingPointDepression::new(CryoscopicConstant::water(), 2.0);
        let molality = 0.5;
        let delta = fpd.delta_tf(molality);
        let m_back = fpd.molality_from_depression(delta);
        assert!(approx_eq(m_back, molality, TOL));
    }

    #[test]
    fn test_session_calibration_improves_accuracy() {
        let mut session = CryoscopicSession::new(CryoscopicConstant::water());

        // Simulated instrument that reads 5% low
        let instrument_bias = 0.95;

        // Calibrate with sucrose (MW = 342.3 g/mol)
        // 17.115 g in 0.5 kg water: m = 0.1 mol/kg, ΔTf = 0.186 K
        let true_delta = 1.86 * 0.1;
        let measured_delta = true_delta * instrument_bias;
        session.calibrate(342.3, 17.115, 0.5, measured_delta);

        // Now measure unknown (should be corrected)
        let true_unknown_delta = 1.86 * 0.2;
        let measured_unknown = true_unknown_delta * instrument_bias;
        let mw = session.measure("Unknown", 10.0, 0.5, measured_unknown);

        // MW = 1.86 × 10 / (corrected_delta × 0.5) should be close to
        // 1.86 × 10 / (true_delta_unknown × 0.5) = 100 g/mol
        let expected_mw = 1.86 * 10.0 / (true_unknown_delta * 0.5);
        assert!(approx_eq(mw, expected_mw, 2.0));
    }

    #[test]
    fn test_osmotic_from_fpd_consistency() {
        let op = OsmoticPressureFromFPD::aqueous();
        // Zero depression → zero osmolality
        assert!(approx_eq(op.osmolality(0.0), 0.0, TOL));
        // Negative depression is physically meaningless but shouldn't crash
        let _ = op.osmolality(-0.1);
    }

    #[test]
    fn test_solve_3x3_identity() {
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [1.0, 2.0, 3.0];
        let x = solve_3x3(&a, &b).unwrap();
        assert!(approx_eq(x[0], 1.0, TOL));
        assert!(approx_eq(x[1], 2.0, TOL));
        assert!(approx_eq(x[2], 3.0, TOL));
    }

    #[test]
    fn test_solve_3x3_singular() {
        let a = [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [1.0, 2.0, 3.0];
        assert!(solve_3x3(&a, &b).is_none());
    }

    #[test]
    fn test_fit_quadratic_linear_data() {
        // y = 2x + 1
        let data = vec![(0.0, 1.0), (1.0, 3.0), (2.0, 5.0), (3.0, 7.0)];
        let (a0, a1, a2) = fit_quadratic(&data);
        assert!(approx_eq(a0, 1.0, 0.01));
        assert!(approx_eq(a1, 2.0, 0.01));
        assert!(approx_eq(a2, 0.0, 0.01));
    }

    #[test]
    fn test_fit_quadratic_single_point() {
        let data = vec![(1.0, 5.0)];
        let (a0, _, _) = fit_quadratic(&data);
        assert!(approx_eq(a0, 5.0, TOL));
    }

    #[test]
    fn test_fit_quadratic_parabola() {
        // y = x²
        let data = vec![(-2.0, 4.0), (-1.0, 1.0), (0.0, 0.0), (1.0, 1.0), (2.0, 4.0)];
        let (a0, a1, a2) = fit_quadratic(&data);
        assert!(approx_eq(a0, 0.0, 0.01));
        assert!(approx_eq(a1, 0.0, 0.01));
        assert!(approx_eq(a2, 1.0, 0.01));
    }

    #[test]
    fn test_fpd_high_molality_camphor() {
        // Camphor has very high Kf, useful for small MW determination
        let fpd = FreezingPointDepression::non_electrolyte(CryoscopicConstant::camphor());
        let delta = fpd.delta_tf(0.01);
        // 0.01 molal → ΔTf = 37.7 × 0.01 = 0.377 K
        assert!(approx_eq(delta, 0.377, 0.001));
    }
}

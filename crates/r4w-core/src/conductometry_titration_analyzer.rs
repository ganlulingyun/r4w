//! # Conductometry Titration Analyzer
//!
//! Conductometric analysis for electrical conductivity measurements of solutions,
//! conductometric titrations, and ion mobility studies.
//!
//! ## Physics
//!
//! - Cell constant: K = l/A  (cm⁻¹)
//! - Specific conductivity: κ = G × K  (S/cm)
//! - Molar conductivity: Λm = κ/c  (S·cm²/mol)
//! - Kohlrausch's law: Λ°m = Σν+λ°+ + Σν-λ°-
//! - Debye-Hückel-Onsager: Λm = Λ°m - (A + BΛ°m)√c
//! - Ostwald dilution law: Ka = c·α²/(1-α)
//! - Temperature compensation: κ(T) = κ(Tref) × [1 + α(T - Tref)]

use std::f64::consts::PI;

/// Faraday constant (C/mol)
const FARADAY: f64 = 96485.33212;

/// Gas constant (J/(mol·K))
const R_GAS: f64 = 8.314462618;

/// Avogadro's number (1/mol)
const AVOGADRO: f64 = 6.02214076e23;

/// Elementary charge (C)
const ELEM_CHARGE: f64 = 1.602176634e-19;

/// Boltzmann constant (J/K)
const K_BOLTZMANN: f64 = 1.380649e-23;

/// Permittivity of free space (F/m)
const EPSILON_0: f64 = 8.854187817e-12;

/// Viscosity of water at 25°C (Pa·s)
const WATER_VISCOSITY_25C: f64 = 8.9e-4;

/// Relative permittivity of water at 25°C
const WATER_EPSILON_R_25C: f64 = 78.36;

// ─── Limiting Ionic Conductivities ──────────────────────────────────────────

/// Limiting ionic conductivities at 25°C in S·cm²/mol
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Ion {
    /// H⁺ (hydrogen / hydronium) - 349.8 S·cm²/mol
    HPlus,
    /// OH⁻ (hydroxide) - 198.0 S·cm²/mol
    OHMinus,
    /// Na⁺ (sodium) - 50.1 S·cm²/mol
    NaPlus,
    /// K⁺ (potassium) - 73.5 S·cm²/mol
    KPlus,
    /// Li⁺ (lithium) - 38.7 S·cm²/mol
    LiPlus,
    /// Cl⁻ (chloride) - 76.3 S·cm²/mol
    ClMinus,
    /// Br⁻ (bromide) - 78.1 S·cm²/mol
    BrMinus,
    /// NO₃⁻ (nitrate) - 71.4 S·cm²/mol
    NO3Minus,
    /// CH₃COO⁻ (acetate) - 40.9 S·cm²/mol
    AcetateMinus,
    /// SO₄²⁻ (sulfate) - 160.0 S·cm²/mol
    SO4TwoMinus,
    /// Ca²⁺ (calcium) - 119.0 S·cm²/mol
    CaTwoPlus,
    /// Mg²⁺ (magnesium) - 106.0 S·cm²/mol
    MgTwoPlus,
    /// Ba²⁺ (barium) - 127.2 S·cm²/mol
    BaTwoPlus,
    /// Ag⁺ (silver) - 61.9 S·cm²/mol
    AgPlus,
    /// NH₄⁺ (ammonium) - 73.4 S·cm²/mol
    NH4Plus,
    /// F⁻ (fluoride) - 55.4 S·cm²/mol
    FMinus,
    /// I⁻ (iodide) - 76.8 S·cm²/mol
    IMinus,
    /// CO₃²⁻ (carbonate) - 138.6 S·cm²/mol
    CO3TwoMinus,
    /// Custom ion with given conductivity and charge
    Custom {
        lambda0: f64,
        charge: i32,
    },
}

impl Ion {
    /// Limiting ionic conductivity λ° at 25°C in S·cm²/mol
    pub fn lambda0(&self) -> f64 {
        match self {
            Ion::HPlus => 349.8,
            Ion::OHMinus => 198.0,
            Ion::NaPlus => 50.1,
            Ion::KPlus => 73.5,
            Ion::LiPlus => 38.7,
            Ion::ClMinus => 76.3,
            Ion::BrMinus => 78.1,
            Ion::NO3Minus => 71.4,
            Ion::AcetateMinus => 40.9,
            Ion::SO4TwoMinus => 160.0,
            Ion::CaTwoPlus => 119.0,
            Ion::MgTwoPlus => 106.0,
            Ion::BaTwoPlus => 127.2,
            Ion::AgPlus => 61.9,
            Ion::NH4Plus => 73.4,
            Ion::FMinus => 55.4,
            Ion::IMinus => 76.8,
            Ion::CO3TwoMinus => 138.6,
            Ion::Custom { lambda0, .. } => *lambda0,
        }
    }

    /// Absolute value of ionic charge number |z|
    pub fn charge_abs(&self) -> u32 {
        match self {
            Ion::HPlus | Ion::NaPlus | Ion::KPlus | Ion::LiPlus | Ion::AgPlus | Ion::NH4Plus => 1,
            Ion::OHMinus | Ion::ClMinus | Ion::BrMinus | Ion::NO3Minus
            | Ion::AcetateMinus | Ion::FMinus | Ion::IMinus => 1,
            Ion::SO4TwoMinus | Ion::CaTwoPlus | Ion::MgTwoPlus | Ion::BaTwoPlus
            | Ion::CO3TwoMinus => 2,
            Ion::Custom { charge, .. } => charge.unsigned_abs(),
        }
    }

    /// Sign of the charge: +1 for cations, -1 for anions
    pub fn charge_sign(&self) -> i32 {
        match self {
            Ion::HPlus | Ion::NaPlus | Ion::KPlus | Ion::LiPlus | Ion::AgPlus
            | Ion::NH4Plus | Ion::CaTwoPlus | Ion::MgTwoPlus | Ion::BaTwoPlus => 1,
            Ion::OHMinus | Ion::ClMinus | Ion::BrMinus | Ion::NO3Minus
            | Ion::AcetateMinus | Ion::SO4TwoMinus | Ion::FMinus | Ion::IMinus
            | Ion::CO3TwoMinus => -1,
            Ion::Custom { charge, .. } => if *charge >= 0 { 1 } else { -1 },
        }
    }

    /// Ionic mobility u = λ°/(|z|·F) in cm²/(V·s)
    pub fn mobility(&self) -> f64 {
        self.lambda0() / (self.charge_abs() as f64 * FARADAY)
    }
}

// ─── ConductivityCell ───────────────────────────────────────────────────────

/// Model of a conductivity cell (two-electrode or four-electrode).
///
/// The cell constant K = l/A relates the geometric properties of the cell
/// (electrode separation l and area A) to the measured conductance.
#[derive(Debug, Clone)]
pub struct ConductivityCell {
    /// Cell constant K in cm⁻¹
    pub cell_constant: f64,
    /// Temperature coefficient α (%/°C as fraction, e.g. 0.02 for 2%)
    pub temp_coefficient: f64,
    /// Reference temperature in °C (typically 25°C)
    pub ref_temperature: f64,
    /// Platinum black roughness factor (>=1.0, typical 1.0-1.5)
    pub pt_black_factor: f64,
}

impl ConductivityCell {
    /// Create a new cell from electrode separation (cm) and area (cm²).
    pub fn new(separation_cm: f64, area_cm2: f64) -> Self {
        assert!(separation_cm > 0.0, "Electrode separation must be positive");
        assert!(area_cm2 > 0.0, "Electrode area must be positive");
        Self {
            cell_constant: separation_cm / area_cm2,
            temp_coefficient: 0.02, // 2%/°C typical for aqueous
            ref_temperature: 25.0,
            pt_black_factor: 1.0,
        }
    }

    /// Create from a known cell constant.
    pub fn from_cell_constant(k: f64) -> Self {
        assert!(k > 0.0, "Cell constant must be positive");
        Self {
            cell_constant: k,
            temp_coefficient: 0.02,
            ref_temperature: 25.0,
            pt_black_factor: 1.0,
        }
    }

    /// Set temperature coefficient (fraction, e.g. 0.02 for 2%/°C).
    pub fn with_temp_coefficient(mut self, alpha: f64) -> Self {
        self.temp_coefficient = alpha;
        self
    }

    /// Set reference temperature in °C.
    pub fn with_ref_temperature(mut self, t: f64) -> Self {
        self.ref_temperature = t;
        self
    }

    /// Set Pt-black roughness factor (>= 1.0).
    pub fn with_pt_black_factor(mut self, factor: f64) -> Self {
        assert!(factor >= 1.0, "Pt-black factor must be >= 1.0");
        self.pt_black_factor = factor;
        self
    }

    /// Effective cell constant accounting for Pt-black roughness.
    /// K_eff = K / roughness_factor (rougher surface → lower effective K)
    pub fn effective_cell_constant(&self) -> f64 {
        self.cell_constant / self.pt_black_factor
    }

    /// Convert measured conductance G (S) to specific conductivity κ (S/cm).
    /// κ = G × K_eff
    pub fn conductance_to_kappa(&self, conductance_s: f64) -> f64 {
        conductance_s * self.effective_cell_constant()
    }

    /// Convert specific conductivity κ (S/cm) to measured conductance G (S).
    /// G = κ / K_eff
    pub fn kappa_to_conductance(&self, kappa: f64) -> f64 {
        kappa / self.effective_cell_constant()
    }

    /// Temperature-compensated specific conductivity at reference temperature.
    /// κ_ref = κ_meas / [1 + α(T - T_ref)]
    pub fn compensate_kappa(&self, kappa_measured: f64, temp_c: f64) -> f64 {
        temperature_compensate(kappa_measured, temp_c, self.ref_temperature, self.temp_coefficient)
    }

    /// Resistance to specific conductivity.
    /// κ = K_eff / R
    pub fn resistance_to_kappa(&self, resistance_ohm: f64) -> f64 {
        self.effective_cell_constant() / resistance_ohm
    }

    /// Calibrate the cell constant using a standard KCl solution.
    /// Returns the calibrated cell constant.
    pub fn calibrate_with_kcl(&mut self, measured_conductance_s: f64, kcl_kappa: f64) -> f64 {
        // K_cal = κ_known / G_measured
        let k_cal = kcl_kappa / measured_conductance_s;
        self.cell_constant = k_cal * self.pt_black_factor;
        k_cal
    }
}

// ─── MolarConductivity ──────────────────────────────────────────────────────

/// Molar conductivity calculations using Kohlrausch's law.
#[derive(Debug, Clone)]
pub struct MolarConductivity;

impl MolarConductivity {
    /// Calculate molar conductivity Λm = κ/c (S·cm²/mol).
    ///
    /// - `kappa`: specific conductivity in S/cm
    /// - `concentration_mol_per_l`: concentration in mol/L
    pub fn from_kappa(kappa: f64, concentration_mol_per_l: f64) -> f64 {
        assert!(concentration_mol_per_l > 0.0, "Concentration must be positive");
        // Λm = (κ in S/cm) / (c in mol/cm³)
        // c in mol/cm³ = c_mol_per_l / 1000
        kappa / (concentration_mol_per_l / 1000.0)
    }

    /// Calculate specific conductivity κ from molar conductivity.
    /// κ = Λm × c/1000
    pub fn to_kappa(lambda_m: f64, concentration_mol_per_l: f64) -> f64 {
        lambda_m * (concentration_mol_per_l / 1000.0)
    }

    /// Kohlrausch's law of independent migration of ions.
    /// Λ°m = Σ ν_i × λ°_i for all ions.
    ///
    /// `ions_and_stoich`: (ion, stoichiometric coefficient)
    pub fn kohlrausch_limiting(ions_and_stoich: &[(Ion, u32)]) -> f64 {
        ions_and_stoich
            .iter()
            .map(|(ion, nu)| *nu as f64 * ion.lambda0())
            .sum()
    }

    /// Limiting molar conductivity of a strong electrolyte from cation and anion.
    pub fn limiting_from_ions(
        cation: Ion, nu_cation: u32,
        anion: Ion, nu_anion: u32,
    ) -> f64 {
        nu_cation as f64 * cation.lambda0() + nu_anion as f64 * anion.lambda0()
    }
}

// ─── DebyeHuckelOnsager ─────────────────────────────────────────────────────

/// Debye-Hückel-Onsager equation for concentration dependence of molar conductivity
/// of strong electrolytes.
///
/// Λm = Λ°m - (A + B·Λ°m) × √c
///
/// where A and B depend on solvent properties, temperature, and ion charges.
#[derive(Debug, Clone)]
pub struct DebyeHuckelOnsager {
    /// Limiting molar conductivity Λ°m (S·cm²/mol)
    pub lambda0: f64,
    /// Onsager coefficient A (S·cm²/mol / (mol/L)^0.5)
    pub a_coeff: f64,
    /// Onsager coefficient B (dimensionless coefficient on Λ°m)
    pub b_coeff: f64,
}

impl DebyeHuckelOnsager {
    /// Create with pre-computed coefficients.
    pub fn new(lambda0: f64, a: f64, b: f64) -> Self {
        Self {
            lambda0,
            a_coeff: a,
            b_coeff: b,
        }
    }

    /// Compute Onsager coefficients for a z:z electrolyte in water at 25°C.
    ///
    /// For a z:z electrolyte:
    /// A = (z² × e × F) / (3π × η) × √(2 / (ε₀ε_r R T))
    /// B = (q × z³ × e × F) / (24π × ε₀ε_r k_B T) × √(2 / (ε₀ε_r R T))
    ///
    /// Simplified for water at 25°C (z=1):
    /// A ≈ 60.20 S·cm²·(mol/L)^(-1/2)/mol (the relaxation term)
    /// B ≈ 0.2289 (mol/L)^(-1/2) (the electrophoretic term)
    pub fn for_water_25c(lambda0: f64, z: u32) -> Self {
        // Standard values for 1:1 electrolytes in water at 25°C
        let z = z as f64;
        // A (electrophoretic term) ≈ 60.20 for z=1
        let a = 60.20 * z * z;
        // B (relaxation term) ≈ 0.2289 for z=1
        let b = 0.2289 * z * z;
        Self {
            lambda0,
            a_coeff: a,
            b_coeff: b,
        }
    }

    /// Compute molar conductivity at given concentration (mol/L).
    /// Λm = Λ°m - (A + B·Λ°m) × √c
    pub fn lambda_m(&self, concentration_mol_per_l: f64) -> f64 {
        let sqrt_c = concentration_mol_per_l.sqrt();
        self.lambda0 - (self.a_coeff + self.b_coeff * self.lambda0) * sqrt_c
    }

    /// Compute specific conductivity at given concentration.
    pub fn kappa(&self, concentration_mol_per_l: f64) -> f64 {
        let lm = self.lambda_m(concentration_mol_per_l);
        lm * concentration_mol_per_l / 1000.0
    }

    /// Onsager slope S = A + B·Λ°m
    pub fn onsager_slope(&self) -> f64 {
        self.a_coeff + self.b_coeff * self.lambda0
    }
}

// ─── ConductometricTitration ────────────────────────────────────────────────

/// Type of conductometric titration reaction
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TitrationType {
    /// Strong acid + Strong base: V-shaped curve
    StrongAcidStrongBase,
    /// Weak acid + Strong base: curved then linear
    WeakAcidStrongBase,
    /// Strong acid + Weak base
    StrongAcidWeakBase,
    /// Precipitation titration (e.g., NaCl + AgNO3)
    Precipitation,
    /// Complexometric titration
    Complexometric,
}

/// A single data point in a conductometric titration.
#[derive(Debug, Clone, Copy)]
pub struct TitrationPoint {
    /// Volume of titrant added (mL)
    pub volume_ml: f64,
    /// Measured conductivity κ (μS/cm) or conductance G (μS)
    pub conductivity: f64,
}

/// Conductometric titration curve processor.
///
/// Processes V-shaped or curved conductivity vs. volume data to find
/// equivalence point(s) from the intersection of linear segments.
#[derive(Debug, Clone)]
pub struct ConductometricTitration {
    /// Titration data points
    pub data: Vec<TitrationPoint>,
    /// Type of titration
    pub titration_type: TitrationType,
}

impl ConductometricTitration {
    /// Create a new titration from data points.
    pub fn new(data: Vec<TitrationPoint>, titration_type: TitrationType) -> Self {
        assert!(data.len() >= 4, "Need at least 4 data points for endpoint detection");
        Self { data, titration_type }
    }

    /// Create from separate volume and conductivity arrays.
    pub fn from_arrays(volumes: &[f64], conductivities: &[f64], titration_type: TitrationType) -> Self {
        assert_eq!(volumes.len(), conductivities.len(), "Arrays must have equal length");
        let data: Vec<_> = volumes.iter().zip(conductivities.iter())
            .map(|(&v, &c)| TitrationPoint { volume_ml: v, conductivity: c })
            .collect();
        Self::new(data, titration_type)
    }

    /// Find the minimum conductivity point (approximate equivalence point for V-curve).
    pub fn minimum_point(&self) -> (f64, f64) {
        let mut min_idx = 0;
        let mut min_val = f64::INFINITY;
        for (i, pt) in self.data.iter().enumerate() {
            if pt.conductivity < min_val {
                min_val = pt.conductivity;
                min_idx = i;
            }
        }
        (self.data[min_idx].volume_ml, self.data[min_idx].conductivity)
    }

    /// Correct conductivities for dilution effect.
    /// κ_corrected = κ_measured × (V_initial + V_titrant) / V_initial
    pub fn dilution_corrected(&self, initial_volume_ml: f64) -> Vec<TitrationPoint> {
        self.data.iter().map(|pt| {
            let factor = (initial_volume_ml + pt.volume_ml) / initial_volume_ml;
            TitrationPoint {
                volume_ml: pt.volume_ml,
                conductivity: pt.conductivity * factor,
            }
        }).collect()
    }

    /// Calculate first derivative of conductivity vs. volume.
    pub fn first_derivative(&self) -> Vec<(f64, f64)> {
        let mut deriv = Vec::with_capacity(self.data.len() - 1);
        for i in 0..self.data.len() - 1 {
            let dv = self.data[i + 1].volume_ml - self.data[i].volume_ml;
            if dv.abs() < 1e-15 { continue; }
            let dk = self.data[i + 1].conductivity - self.data[i].conductivity;
            let v_mid = (self.data[i].volume_ml + self.data[i + 1].volume_ml) / 2.0;
            deriv.push((v_mid, dk / dv));
        }
        deriv
    }

    /// Calculate second derivative of conductivity vs. volume.
    pub fn second_derivative(&self) -> Vec<(f64, f64)> {
        let first = self.first_derivative();
        let mut second = Vec::with_capacity(first.len().saturating_sub(1));
        for i in 0..first.len().saturating_sub(1) {
            let dv = first[i + 1].0 - first[i].0;
            if dv.abs() < 1e-15 { continue; }
            let d2k = first[i + 1].1 - first[i].1;
            let v_mid = (first[i].0 + first[i + 1].0) / 2.0;
            second.push((v_mid, d2k / dv));
        }
        second
    }
}

// ─── EndpointDetector ───────────────────────────────────────────────────────

/// Linear regression result: y = slope * x + intercept
#[derive(Debug, Clone, Copy)]
pub struct LinearFit {
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
}

/// Detected endpoint from a conductometric titration.
#[derive(Debug, Clone, Copy)]
pub struct Endpoint {
    /// Volume at the equivalence point (mL)
    pub volume_ml: f64,
    /// Conductivity at the equivalence point
    pub conductivity: f64,
    /// Pre-equivalence linear fit
    pub pre_fit: LinearFit,
    /// Post-equivalence linear fit
    pub post_fit: LinearFit,
}

/// Endpoint detection from conductometric titration curves.
#[derive(Debug, Clone)]
pub struct EndpointDetector {
    /// Minimum R² for linear segment validation
    pub min_r_squared: f64,
    /// Minimum number of points per segment
    pub min_points_per_segment: usize,
}

impl EndpointDetector {
    /// Create with default parameters.
    pub fn new() -> Self {
        Self {
            min_r_squared: 0.90,
            min_points_per_segment: 3,
        }
    }

    /// Set minimum R² threshold.
    pub fn with_min_r_squared(mut self, r2: f64) -> Self {
        self.min_r_squared = r2;
        self
    }

    /// Set minimum points per segment.
    pub fn with_min_points(mut self, n: usize) -> Self {
        self.min_points_per_segment = n;
        self
    }

    /// Detect the equivalence point using intersection of two linear segments.
    ///
    /// Splits the data at a scan point, fits linear regression to each segment,
    /// and finds the best split where both segments have good R² values.
    pub fn detect_endpoint(&self, titration: &ConductometricTitration) -> Option<Endpoint> {
        let n = titration.data.len();
        let min_pts = self.min_points_per_segment;
        if n < 2 * min_pts {
            return None;
        }

        let mut best: Option<Endpoint> = None;
        let mut best_score = f64::NEG_INFINITY;

        for split in min_pts..(n - min_pts + 1) {
            let pre_x: Vec<f64> = titration.data[..split].iter().map(|p| p.volume_ml).collect();
            let pre_y: Vec<f64> = titration.data[..split].iter().map(|p| p.conductivity).collect();
            let post_x: Vec<f64> = titration.data[split..].iter().map(|p| p.volume_ml).collect();
            let post_y: Vec<f64> = titration.data[split..].iter().map(|p| p.conductivity).collect();

            let pre_fit = linear_regression(&pre_x, &pre_y);
            let post_fit = linear_regression(&post_x, &post_y);

            if pre_fit.r_squared < self.min_r_squared || post_fit.r_squared < self.min_r_squared {
                continue;
            }

            // Lines must have different slopes to intersect
            let slope_diff = (post_fit.slope - pre_fit.slope).abs();
            if slope_diff < 1e-12 {
                continue;
            }

            // Intersection point
            let v_eq = (pre_fit.intercept - post_fit.intercept)
                / (post_fit.slope - pre_fit.slope);
            let k_eq = pre_fit.slope * v_eq + pre_fit.intercept;

            // Score: combined R² weighted by slope difference
            let score = (pre_fit.r_squared + post_fit.r_squared) * slope_diff;

            // Intersection must be within data range
            let v_min = titration.data[0].volume_ml;
            let v_max = titration.data[n - 1].volume_ml;
            if v_eq < v_min || v_eq > v_max {
                continue;
            }

            if score > best_score {
                best_score = score;
                best = Some(Endpoint {
                    volume_ml: v_eq,
                    conductivity: k_eq,
                    pre_fit,
                    post_fit,
                });
            }
        }

        best
    }

    /// Detect multiple endpoints (for polyprotic acids or sequential reactions).
    pub fn detect_multiple_endpoints(
        &self,
        titration: &ConductometricTitration,
        max_endpoints: usize,
    ) -> Vec<Endpoint> {
        let mut endpoints = Vec::new();
        let n = titration.data.len();
        let min_pts = self.min_points_per_segment;

        if max_endpoints == 0 || n < 2 * min_pts {
            return endpoints;
        }

        if max_endpoints == 1 {
            if let Some(ep) = self.detect_endpoint(titration) {
                endpoints.push(ep);
            }
            return endpoints;
        }

        // For multiple endpoints, use derivative method to find inflection points
        let deriv = titration.second_derivative();
        if deriv.is_empty() {
            return endpoints;
        }

        // Find sign changes in second derivative (inflection points)
        let mut inflection_volumes = Vec::new();
        for i in 0..deriv.len() - 1 {
            if deriv[i].1 * deriv[i + 1].1 < 0.0 {
                // Linear interpolation to find zero crossing
                let v = deriv[i].0 + (deriv[i + 1].0 - deriv[i].0)
                    * (-deriv[i].1 / (deriv[i + 1].1 - deriv[i].1));
                inflection_volumes.push(v);
            }
        }

        // For each inflection region, try to find an endpoint
        // Use a sub-range approach: split at each inflection and analyze segments
        if inflection_volumes.is_empty() {
            // Fall back to single endpoint
            if let Some(ep) = self.detect_endpoint(titration) {
                endpoints.push(ep);
            }
        } else {
            // Take up to max_endpoints inflection points
            for &v_inf in inflection_volumes.iter().take(max_endpoints) {
                // Find closest data index
                let mut closest_idx = 0;
                let mut closest_dist = f64::INFINITY;
                for (i, pt) in titration.data.iter().enumerate() {
                    let dist = (pt.volume_ml - v_inf).abs();
                    if dist < closest_dist {
                        closest_dist = dist;
                        closest_idx = i;
                    }
                }

                // Build sub-titration around this inflection
                let start = if closest_idx >= min_pts { closest_idx - min_pts } else { 0 };
                let end = if closest_idx + min_pts < n { closest_idx + min_pts + 1 } else { n };

                if end - start >= 2 * min_pts {
                    let sub_data: Vec<TitrationPoint> =
                        titration.data[start..end].to_vec();
                    let sub_titration =
                        ConductometricTitration::new(sub_data, titration.titration_type);
                    if let Some(ep) = self.detect_endpoint(&sub_titration) {
                        endpoints.push(ep);
                    }
                }
            }
        }

        endpoints.sort_by(|a, b| a.volume_ml.partial_cmp(&b.volume_ml).unwrap());
        endpoints
    }

    /// Derivative-based endpoint detection.
    /// The equivalence point is where the first derivative has the largest magnitude change.
    pub fn detect_by_derivative(&self, titration: &ConductometricTitration) -> Option<f64> {
        let deriv = titration.first_derivative();
        if deriv.len() < 3 {
            return None;
        }

        // Find maximum absolute change in first derivative
        let mut max_change = 0.0_f64;
        let mut ep_volume = 0.0;
        for i in 0..deriv.len() - 1 {
            let change = (deriv[i + 1].1 - deriv[i].1).abs();
            if change > max_change {
                max_change = change;
                ep_volume = (deriv[i].0 + deriv[i + 1].0) / 2.0;
            }
        }
        if max_change > 0.0 {
            Some(ep_volume)
        } else {
            None
        }
    }
}

impl Default for EndpointDetector {
    fn default() -> Self {
        Self::new()
    }
}

// ─── IonMobility ────────────────────────────────────────────────────────────

/// Ion mobility and transport number calculations.
#[derive(Debug, Clone)]
pub struct IonMobility;

impl IonMobility {
    /// Ionic mobility from limiting conductivity.
    /// u = λ° / (|z| × F) in cm²/(V·s)
    pub fn mobility(lambda0: f64, z: u32) -> f64 {
        assert!(z > 0, "Charge number must be positive");
        lambda0 / (z as f64 * FARADAY)
    }

    /// Transport number of cation: t+ = u+ / (u+ + u-)
    pub fn transport_number_cation(cation: &Ion, anion: &Ion) -> f64 {
        let u_plus = cation.mobility();
        let u_minus = anion.mobility();
        u_plus / (u_plus + u_minus)
    }

    /// Transport number of anion: t- = u- / (u+ + u-)
    pub fn transport_number_anion(cation: &Ion, anion: &Ion) -> f64 {
        1.0 - Self::transport_number_cation(cation, anion)
    }

    /// Hittorf method: transport number from mass change.
    /// t+ = (Δn_anode × z × F) / (I × t)
    ///
    /// - `delta_mol_anode`: change in moles at the anode compartment
    /// - `z`: charge number
    /// - `current_a`: current in amperes
    /// - `time_s`: time in seconds
    pub fn hittorf_transport_number(
        delta_mol_anode: f64,
        z: u32,
        current_a: f64,
        time_s: f64,
    ) -> f64 {
        let total_equivalents = current_a * time_s / FARADAY;
        (delta_mol_anode * z as f64) / total_equivalents
    }

    /// Stokes radius from mobility: r_s = z×e / (6π×η×u)
    /// where η is viscosity in Pa·s
    pub fn stokes_radius(ion: &Ion, viscosity_pa_s: f64) -> f64 {
        let u_si = ion.mobility() * 1e-4; // cm²/(V·s) -> m²/(V·s)
        let z = ion.charge_abs() as f64;
        z * ELEM_CHARGE / (6.0 * PI * viscosity_pa_s * u_si)
    }

    /// Walden product: Λ°m × η (should be approximately constant for a given ion)
    pub fn walden_product(lambda0: f64, viscosity_pa_s: f64) -> f64 {
        // Convert viscosity to poise (1 Pa·s = 10 P)
        lambda0 * viscosity_pa_s * 10.0
    }

    /// Diffusion coefficient from Nernst-Einstein: D = u × k_B × T / (z × e)
    pub fn diffusion_coefficient(ion: &Ion, temp_k: f64) -> f64 {
        let u_si = ion.mobility() * 1e-4; // cm²/(V·s) -> m²/(V·s)
        let z = ion.charge_abs() as f64;
        u_si * K_BOLTZMANN * temp_k / (z * ELEM_CHARGE)
    }
}

// ─── WheatstoneACBridge ─────────────────────────────────────────────────────

/// AC Wheatstone bridge model for conductivity measurement.
///
/// The bridge consists of four arms: R1, R2, R3 (known) and Z_cell (unknown).
/// Balance condition: Z_cell = R3 × R2 / R1
/// For the Jones-Josephs modification, capacitance compensation is included.
#[derive(Debug, Clone)]
pub struct WheatstoneACBridge {
    /// Known resistor R1 (Ω)
    pub r1: f64,
    /// Known resistor R2 (Ω)
    pub r2: f64,
    /// Variable resistor R3 for balance (Ω)
    pub r3: f64,
    /// Frequency of AC excitation (Hz)
    pub frequency_hz: f64,
    /// Compensating capacitor (F) - Jones-Josephs bridge
    pub compensating_cap: f64,
}

impl WheatstoneACBridge {
    /// Create a new bridge with given arm resistances and frequency.
    pub fn new(r1: f64, r2: f64, r3: f64, frequency_hz: f64) -> Self {
        Self {
            r1,
            r2,
            r3,
            frequency_hz,
            compensating_cap: 0.0,
        }
    }

    /// Set compensating capacitor for Jones-Josephs bridge.
    pub fn with_capacitor(mut self, cap_f: f64) -> Self {
        self.compensating_cap = cap_f;
        self
    }

    /// Balance condition: resistance of the unknown cell at DC balance.
    /// R_cell = R3 × R2 / R1
    pub fn cell_resistance_at_balance(&self) -> f64 {
        self.r3 * self.r2 / self.r1
    }

    /// Conductance at balance: G = 1/R_cell
    pub fn conductance_at_balance(&self) -> f64 {
        1.0 / self.cell_resistance_at_balance()
    }

    /// Impedance of cell arm with parallel capacitance.
    /// Z = R / (1 + jωRC) for a cell with resistance R and capacitance C.
    /// |Z| = R / √(1 + (ωRC)²)
    pub fn cell_impedance_magnitude(&self, cell_resistance: f64, cell_cap: f64) -> f64 {
        let omega = 2.0 * PI * self.frequency_hz;
        let wrc = omega * cell_resistance * cell_cap;
        cell_resistance / (1.0 + wrc * wrc).sqrt()
    }

    /// Phase angle of cell impedance.
    /// θ = -arctan(ωRC)
    pub fn cell_impedance_phase(&self, cell_resistance: f64, cell_cap: f64) -> f64 {
        let omega = 2.0 * PI * self.frequency_hz;
        let wrc = omega * cell_resistance * cell_cap;
        -wrc.atan()
    }

    /// Frequency-dependent error in resistance measurement.
    /// ΔR/R = (ωRC)² for small ωRC
    pub fn frequency_error_fraction(&self, cell_resistance: f64, cell_cap: f64) -> f64 {
        let omega = 2.0 * PI * self.frequency_hz;
        let wrc = omega * cell_resistance * cell_cap;
        wrc * wrc
    }

    /// Jones-Josephs bridge: corrected conductance with capacitance compensation.
    /// The compensating capacitor C_comp is adjusted to null the reactive component.
    /// At balance: G_corrected = G_balance × (1 + ω²R²C_cell(C_cell - C_comp))
    /// For perfect compensation (C_comp ≈ C_cell), error vanishes.
    pub fn compensated_conductance(&self, cell_cap: f64) -> f64 {
        let g = self.conductance_at_balance();
        let omega = 2.0 * PI * self.frequency_hz;
        let r = self.cell_resistance_at_balance();
        let correction = omega * omega * r * r * cell_cap * (cell_cap - self.compensating_cap);
        g * (1.0 + correction)
    }

    /// Optimal measurement frequency to minimize errors.
    /// Rule of thumb: f_opt ≈ 1/(2π R C) where R is cell resistance, C is cell capacitance.
    pub fn optimal_frequency(cell_resistance: f64, cell_cap: f64) -> f64 {
        if cell_cap <= 0.0 || cell_resistance <= 0.0 {
            return 1000.0; // default 1 kHz
        }
        1.0 / (2.0 * PI * cell_resistance * cell_cap)
    }
}

// ─── KohlrauschExtrapolation ────────────────────────────────────────────────

/// Extrapolation of weak electrolyte molar conductivity to infinite dilution
/// using the Ostwald dilution law and Kohlrausch's law.
///
/// For weak electrolytes, Λm cannot be directly extrapolated from a Λm vs √c plot.
/// Instead, we use the relation:
/// 1/Λm = 1/Λ°m + Λm·c / (Ka·(Λ°m)²)
///
/// Iterative procedure:
/// 1. Estimate Λ°m from Kohlrausch's law of independent migration
/// 2. Calculate α = Λm/Λ°m for each concentration
/// 3. Calculate Ka from Ostwald dilution: Ka = cα²/(1-α)
/// 4. Refine Λ°m from the 1/Λm vs Λm·c plot
/// 5. Iterate until convergence
#[derive(Debug, Clone)]
pub struct KohlrauschExtrapolation {
    /// Maximum iterations for convergence
    pub max_iterations: usize,
    /// Convergence tolerance for Λ°m
    pub tolerance: f64,
}

impl KohlrauschExtrapolation {
    /// Create with default parameters.
    pub fn new() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 1e-4,
        }
    }

    /// Extrapolate Λ°m for a weak electrolyte.
    ///
    /// - `concentrations`: concentrations in mol/L
    /// - `lambda_m_values`: measured molar conductivities in S·cm²/mol
    /// - `lambda0_initial`: initial estimate of Λ°m (e.g. from Kohlrausch's law)
    ///
    /// Returns (Λ°m, Ka) or None if convergence fails.
    pub fn extrapolate(
        &self,
        concentrations: &[f64],
        lambda_m_values: &[f64],
        lambda0_initial: f64,
    ) -> Option<(f64, f64)> {
        assert_eq!(
            concentrations.len(),
            lambda_m_values.len(),
            "Arrays must have equal length"
        );
        assert!(!concentrations.is_empty(), "Need at least one data point");

        let n = concentrations.len();
        let mut lambda0 = lambda0_initial;

        for _iter in 0..self.max_iterations {
            // Calculate Ka for each point using current Λ°m estimate
            let mut ka_sum = 0.0;
            let mut ka_count = 0;

            for i in 0..n {
                let alpha = lambda_m_values[i] / lambda0;
                if alpha > 0.0 && alpha < 1.0 {
                    let ka = concentrations[i] * alpha * alpha / (1.0 - alpha);
                    if ka > 0.0 && ka.is_finite() {
                        ka_sum += ka;
                        ka_count += 1;
                    }
                }
            }

            if ka_count == 0 {
                return None;
            }

            let ka_avg = ka_sum / ka_count as f64;

            // Use 1/Λm = 1/Λ°m + Λm·c/(Ka·Λ°m²) to refine Λ°m
            // Linear regression of 1/Λm vs Λm·c
            let x: Vec<f64> = (0..n)
                .map(|i| lambda_m_values[i] * concentrations[i])
                .collect();
            let y: Vec<f64> = (0..n)
                .map(|i| 1.0 / lambda_m_values[i])
                .collect();

            let fit = linear_regression(&x, &y);

            // intercept = 1/Λ°m, slope = 1/(Ka·Λ°m²)
            let lambda0_new = 1.0 / fit.intercept;

            if (lambda0_new - lambda0).abs() / lambda0.abs().max(1e-15) < self.tolerance {
                let ka_final = 1.0 / (fit.slope * lambda0_new * lambda0_new);
                return Some((lambda0_new, ka_final.abs()));
            }

            lambda0 = lambda0_new;
        }

        // Return best estimate even if not fully converged
        let alpha_avg: f64 = lambda_m_values.iter().map(|lm| lm / lambda0).sum::<f64>() / n as f64;
        let c_avg: f64 = concentrations.iter().sum::<f64>() / n as f64;
        let ka = c_avg * alpha_avg * alpha_avg / (1.0 - alpha_avg).max(1e-15);
        Some((lambda0, ka))
    }
}

impl Default for KohlrauschExtrapolation {
    fn default() -> Self {
        Self::new()
    }
}

// ─── WaterQualityFromConductivity ───────────────────────────────────────────

/// Water quality classification based on conductivity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WaterQuality {
    /// Ultra-pure (Type I): < 0.055 μS/cm (18.2 MΩ·cm)
    UltraPure,
    /// Purified (USP): < 1.3 μS/cm at 25°C
    Purified,
    /// Deionized: < 10 μS/cm
    Deionized,
    /// Distilled: typically 0.5 - 5 μS/cm
    Distilled,
    /// Drinking water: 50 - 1500 μS/cm
    Drinking,
    /// Freshwater: < 1500 μS/cm
    Freshwater,
    /// Brackish: 1500 - 15000 μS/cm
    Brackish,
    /// Saline: 15000 - 50000 μS/cm
    Saline,
    /// Brine: > 50000 μS/cm
    Brine,
}

/// Water quality assessment from conductivity measurements.
#[derive(Debug, Clone)]
pub struct WaterQualityFromConductivity;

impl WaterQualityFromConductivity {
    /// Classify water quality based on conductivity in μS/cm.
    pub fn classify(kappa_us_cm: f64) -> WaterQuality {
        if kappa_us_cm < 0.055 {
            WaterQuality::UltraPure
        } else if kappa_us_cm < 1.3 {
            WaterQuality::Purified
        } else if kappa_us_cm < 10.0 {
            WaterQuality::Deionized
        } else if kappa_us_cm < 50.0 {
            WaterQuality::Distilled
        } else if kappa_us_cm < 1500.0 {
            WaterQuality::Drinking
        } else if kappa_us_cm < 15000.0 {
            WaterQuality::Brackish
        } else if kappa_us_cm < 50000.0 {
            WaterQuality::Saline
        } else {
            WaterQuality::Brine
        }
    }

    /// Estimate Total Dissolved Solids (TDS) from conductivity.
    /// TDS (mg/L) ≈ kf × κ (μS/cm)
    /// Typical kf: 0.5-0.7 (default 0.64 for NaCl-dominated waters)
    pub fn estimate_tds(kappa_us_cm: f64, conversion_factor: Option<f64>) -> f64 {
        let kf = conversion_factor.unwrap_or(0.64);
        kf * kappa_us_cm
    }

    /// Check USP purified water compliance.
    /// Stage 1 (inline): κ must be ≤ 1.3 μS/cm at 25°C (USP <645>)
    pub fn is_usp_compliant(kappa_us_cm_at_25c: f64) -> bool {
        kappa_us_cm_at_25c <= 1.3
    }

    /// Check USP WFI (Water for Injection) compliance.
    /// κ must be ≤ 1.3 μS/cm at 25°C and TOC ≤ 500 ppb
    pub fn is_wfi_compliant(kappa_us_cm_at_25c: f64, toc_ppb: Option<f64>) -> bool {
        let conductivity_ok = kappa_us_cm_at_25c <= 1.3;
        let toc_ok = toc_ppb.map_or(true, |t| t <= 500.0);
        conductivity_ok && toc_ok
    }

    /// Resistivity from conductivity.
    /// ρ (MΩ·cm) = 1 / κ (μS/cm)
    pub fn resistivity_mohm_cm(kappa_us_cm: f64) -> f64 {
        if kappa_us_cm <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / kappa_us_cm
    }

    /// Estimate salinity (PSU) from conductivity at 25°C.
    /// Uses simplified UNESCO formula for practical salinity.
    /// Valid for S ≈ 2-42 PSU
    pub fn estimate_salinity_psu(kappa_ms_cm: f64) -> f64 {
        // R_t = κ / κ_standard_seawater (42.914 mS/cm at 15°C, ~53 mS/cm at 25°C)
        // Simplified: S ≈ 0.008 - 0.1692 Rt^0.5 + 25.3851 Rt + 14.0941 Rt^1.5
        //           - 7.0261 Rt^2 + 2.7081 Rt^2.5
        let kappa_standard = 53.087; // mS/cm at 25°C for S=35
        let rt = kappa_ms_cm / kappa_standard;
        let rt_sqrt = rt.sqrt();

        0.008 - 0.1692 * rt_sqrt + 25.3851 * rt + 14.0941 * rt * rt_sqrt
            - 7.0261 * rt * rt + 2.7081 * rt * rt * rt_sqrt
    }

    /// USP Stage 1 conductivity limit at a given temperature.
    /// Returns the maximum allowable conductivity (μS/cm).
    /// Based on USP <645> Table 1.
    pub fn usp_stage1_limit(temp_c: f64) -> f64 {
        // Simplified interpolation of USP <645> Table 1
        if temp_c <= 0.0 {
            0.6
        } else if temp_c <= 10.0 {
            0.6 + (temp_c / 10.0) * (0.9 - 0.6)
        } else if temp_c <= 20.0 {
            0.9 + ((temp_c - 10.0) / 10.0) * (1.1 - 0.9)
        } else if temp_c <= 25.0 {
            1.1 + ((temp_c - 20.0) / 5.0) * (1.3 - 1.1)
        } else if temp_c <= 30.0 {
            1.3 + ((temp_c - 25.0) / 5.0) * (1.4 - 1.3)
        } else if temp_c <= 40.0 {
            1.4 + ((temp_c - 30.0) / 10.0) * (1.7 - 1.4)
        } else if temp_c <= 50.0 {
            1.7 + ((temp_c - 40.0) / 10.0) * (2.1 - 1.7)
        } else if temp_c <= 60.0 {
            2.1 + ((temp_c - 50.0) / 10.0) * (2.5 - 2.1)
        } else if temp_c <= 70.0 {
            2.5 + ((temp_c - 60.0) / 10.0) * (3.1 - 2.5)
        } else if temp_c <= 80.0 {
            3.1 + ((temp_c - 70.0) / 10.0) * (3.6 - 3.1)
        } else if temp_c <= 90.0 {
            3.6 + ((temp_c - 80.0) / 10.0) * (4.2 - 3.6)
        } else {
            4.2 + ((temp_c - 90.0) / 10.0) * (4.7 - 4.2)
        }
    }
}

// ─── Helper Functions ───────────────────────────────────────────────────────

/// Temperature compensation for conductivity.
/// κ_ref = κ_meas / [1 + α(T - T_ref)]
///
/// - `kappa`: measured conductivity
/// - `temp_c`: measurement temperature (°C)
/// - `ref_temp`: reference temperature (°C), typically 25°C
/// - `alpha`: temperature coefficient (fraction, e.g. 0.02 for 2%/°C)
pub fn temperature_compensate(kappa: f64, temp_c: f64, ref_temp: f64, alpha: f64) -> f64 {
    kappa / (1.0 + alpha * (temp_c - ref_temp))
}

/// Degree of dissociation from molar conductivity.
/// α = Λm / Λ°m
pub fn degree_of_dissociation(lambda_m: f64, lambda_inf: f64) -> f64 {
    assert!(lambda_inf > 0.0, "Limiting conductivity must be positive");
    lambda_m / lambda_inf
}

/// Dissociation constant Ka from conductivity via Ostwald dilution law.
/// Ka = c × α² / (1 - α)
/// where α = Λm / Λ°m
pub fn ka_from_conductivity(c: f64, lambda_m: f64, lambda_inf: f64) -> f64 {
    let alpha = degree_of_dissociation(lambda_m, lambda_inf);
    assert!(alpha < 1.0, "Degree of dissociation must be < 1 for weak electrolytes");
    assert!(alpha > 0.0, "Degree of dissociation must be positive");
    c * alpha * alpha / (1.0 - alpha)
}

/// Ionic strength I = 0.5 × Σ c_i × z_i²
pub fn ionic_strength(concentrations: &[f64], charges: &[i32]) -> f64 {
    assert_eq!(concentrations.len(), charges.len());
    0.5 * concentrations
        .iter()
        .zip(charges.iter())
        .map(|(c, z)| c * (*z as f64) * (*z as f64))
        .sum::<f64>()
}

/// Activity coefficient from Debye-Hückel limiting law.
/// log γ± = -A × |z+·z-| × √I
/// A = 0.509 for water at 25°C
pub fn activity_coefficient_dh(z_plus: i32, z_minus: i32, ionic_strength: f64) -> f64 {
    let a = 0.509; // for water at 25°C
    let log_gamma = -a * (z_plus.abs() * z_minus.abs()) as f64 * ionic_strength.sqrt();
    10.0_f64.powf(log_gamma)
}

/// Simple linear regression: y = slope * x + intercept
fn linear_regression(x: &[f64], y: &[f64]) -> LinearFit {
    let n = x.len() as f64;
    assert!(x.len() >= 2, "Need at least 2 points for regression");
    assert_eq!(x.len(), y.len());

    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sum_x2: f64 = x.iter().map(|xi| xi * xi).sum();

    let denom = n * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return LinearFit {
            slope: 0.0,
            intercept: sum_y / n,
            r_squared: 0.0,
        };
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;

    // R²
    let y_mean = sum_y / n;
    let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean) * (yi - y_mean)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| {
            let predicted = slope * xi + intercept;
            (yi - predicted) * (yi - predicted)
        })
        .sum();

    let r_squared = if ss_tot > 1e-30 {
        1.0 - ss_res / ss_tot
    } else {
        1.0
    };

    LinearFit {
        slope,
        intercept,
        r_squared,
    }
}

/// Equivalent conductivity at concentration c for a strong acid-strong base titration.
/// Models the replacement of ions during titration.
pub fn strong_acid_base_conductivity(
    volume_acid_ml: f64,
    conc_acid: f64,
    volume_base_added_ml: f64,
    conc_base: f64,
) -> f64 {
    let mol_acid = volume_acid_ml * conc_acid / 1000.0;
    let mol_base = volume_base_added_ml * conc_base / 1000.0;
    let total_vol_l = (volume_acid_ml + volume_base_added_ml) / 1000.0;

    if mol_base < mol_acid {
        // Before equivalence: excess acid (H+ + anion) + neutralization product (cation + anion)
        let mol_h_remaining = mol_acid - mol_base;
        let mol_salt = mol_base;
        let kappa_h = (mol_h_remaining / total_vol_l) * Ion::HPlus.lambda0() / 1000.0;
        let kappa_anion = (mol_acid / total_vol_l) * Ion::ClMinus.lambda0() / 1000.0;
        let kappa_cation = (mol_salt / total_vol_l) * Ion::NaPlus.lambda0() / 1000.0;
        // Use conductivity contributions (simplified)
        kappa_h + kappa_anion.min(kappa_cation + kappa_h * 0.1)
    } else {
        // After equivalence: excess base (OH- + cation) + salt
        let mol_oh_excess = mol_base - mol_acid;
        let kappa_oh = (mol_oh_excess / total_vol_l) * Ion::OHMinus.lambda0() / 1000.0;
        let kappa_cation = (mol_base / total_vol_l) * Ion::NaPlus.lambda0() / 1000.0;
        let kappa_anion = (mol_acid / total_vol_l) * Ion::ClMinus.lambda0() / 1000.0;
        kappa_oh + kappa_anion.min(kappa_cation)
    }
}

/// Conductivity of a KCl standard solution at 25°C.
/// Returns κ in S/cm for common standard concentrations.
pub fn kcl_standard_conductivity(concentration_mol_per_l: f64) -> f64 {
    // Standard values from NIST
    if (concentration_mol_per_l - 1.0).abs() < 0.01 {
        0.11131 // 1.0 M KCl at 25°C
    } else if (concentration_mol_per_l - 0.1).abs() < 0.001 {
        0.012856 // 0.1 M KCl at 25°C
    } else if (concentration_mol_per_l - 0.01).abs() < 0.0001 {
        0.001408 // 0.01 M KCl at 25°C
    } else {
        // Estimate from Λ°m for KCl (149.8 S·cm²/mol)
        let lambda0_kcl = Ion::KPlus.lambda0() + Ion::ClMinus.lambda0();
        let dho = DebyeHuckelOnsager::for_water_25c(lambda0_kcl, 1);
        dho.kappa(concentration_mol_per_l)
    }
}

/// Solvent conductivity (background) for common solvents at 25°C in S/cm.
pub fn solvent_conductivity(solvent: &str) -> f64 {
    match solvent.to_lowercase().as_str() {
        "water" | "h2o" => 5.5e-8,         // ultra-pure water
        "methanol" | "meoh" => 1.5e-7,
        "ethanol" | "etoh" => 1.35e-7,
        "acetonitrile" | "mecn" => 6.0e-10,
        "dmso" => 2.0e-9,
        "dmf" => 6.0e-8,
        _ => 1e-7, // generic polar solvent
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 1e-3;
    const TOL_PERCENT: f64 = 0.05; // 5% tolerance for approximate values

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn approx_eq_rel(a: f64, b: f64, rel_tol: f64) -> bool {
        let denom = a.abs().max(b.abs()).max(1e-15);
        (a - b).abs() / denom < rel_tol
    }

    // ── Ion tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_ion_lambda0_h_plus() {
        assert_eq!(Ion::HPlus.lambda0(), 349.8);
    }

    #[test]
    fn test_ion_lambda0_oh_minus() {
        assert_eq!(Ion::OHMinus.lambda0(), 198.0);
    }

    #[test]
    fn test_ion_lambda0_na_plus() {
        assert_eq!(Ion::NaPlus.lambda0(), 50.1);
    }

    #[test]
    fn test_ion_lambda0_k_plus() {
        assert_eq!(Ion::KPlus.lambda0(), 73.5);
    }

    #[test]
    fn test_ion_lambda0_cl_minus() {
        assert_eq!(Ion::ClMinus.lambda0(), 76.3);
    }

    #[test]
    fn test_ion_lambda0_so4() {
        assert_eq!(Ion::SO4TwoMinus.lambda0(), 160.0);
    }

    #[test]
    fn test_ion_charge_abs_monovalent() {
        assert_eq!(Ion::HPlus.charge_abs(), 1);
        assert_eq!(Ion::ClMinus.charge_abs(), 1);
        assert_eq!(Ion::NaPlus.charge_abs(), 1);
    }

    #[test]
    fn test_ion_charge_abs_divalent() {
        assert_eq!(Ion::SO4TwoMinus.charge_abs(), 2);
        assert_eq!(Ion::CaTwoPlus.charge_abs(), 2);
    }

    #[test]
    fn test_ion_charge_sign() {
        assert_eq!(Ion::HPlus.charge_sign(), 1);
        assert_eq!(Ion::OHMinus.charge_sign(), -1);
        assert_eq!(Ion::CaTwoPlus.charge_sign(), 1);
        assert_eq!(Ion::SO4TwoMinus.charge_sign(), -1);
    }

    #[test]
    fn test_ion_mobility_h_plus() {
        // u(H+) = 349.8 / (1 × 96485.33) ≈ 3.625e-3 cm²/(V·s)
        let u = Ion::HPlus.mobility();
        assert!(approx_eq_rel(u, 3.625e-3, 0.01));
    }

    #[test]
    fn test_ion_mobility_na_plus() {
        let u = Ion::NaPlus.mobility();
        assert!(u > 0.0);
        assert!(u < Ion::HPlus.mobility()); // Na+ slower than H+
    }

    #[test]
    fn test_ion_custom() {
        let custom = Ion::Custom { lambda0: 100.0, charge: -2 };
        assert_eq!(custom.lambda0(), 100.0);
        assert_eq!(custom.charge_abs(), 2);
        assert_eq!(custom.charge_sign(), -1);
    }

    #[test]
    fn test_ion_custom_positive() {
        let custom = Ion::Custom { lambda0: 55.0, charge: 3 };
        assert_eq!(custom.charge_sign(), 1);
        assert_eq!(custom.charge_abs(), 3);
    }

    // ── ConductivityCell tests ──────────────────────────────────────────────

    #[test]
    fn test_cell_constant_from_geometry() {
        let cell = ConductivityCell::new(1.0, 1.0);
        assert_eq!(cell.cell_constant, 1.0); // K = 1/1 = 1 cm⁻¹
    }

    #[test]
    fn test_cell_constant_from_geometry_2() {
        let cell = ConductivityCell::new(2.0, 0.5);
        assert_eq!(cell.cell_constant, 4.0); // K = 2/0.5 = 4 cm⁻¹
    }

    #[test]
    fn test_cell_from_constant() {
        let cell = ConductivityCell::from_cell_constant(0.1);
        assert_eq!(cell.cell_constant, 0.1);
    }

    #[test]
    fn test_conductance_to_kappa() {
        let cell = ConductivityCell::from_cell_constant(1.0);
        assert_eq!(cell.conductance_to_kappa(0.01), 0.01); // κ = G × K
    }

    #[test]
    fn test_kappa_to_conductance() {
        let cell = ConductivityCell::from_cell_constant(2.0);
        assert!(approx_eq(cell.kappa_to_conductance(0.02), 0.01, TOL));
    }

    #[test]
    fn test_resistance_to_kappa() {
        let cell = ConductivityCell::from_cell_constant(1.0);
        assert!(approx_eq(cell.resistance_to_kappa(100.0), 0.01, TOL));
    }

    #[test]
    fn test_pt_black_factor() {
        let cell = ConductivityCell::from_cell_constant(1.0).with_pt_black_factor(1.5);
        assert!(approx_eq(cell.effective_cell_constant(), 1.0 / 1.5, TOL));
    }

    #[test]
    fn test_temp_compensation_cell() {
        let cell = ConductivityCell::from_cell_constant(1.0);
        // At 30°C, κ_meas should be higher; compensated back to 25°C
        let kappa_30c = 0.01;
        let kappa_25c = cell.compensate_kappa(kappa_30c, 30.0);
        assert!(kappa_25c < kappa_30c); // compensated value is lower
    }

    #[test]
    fn test_calibrate_with_kcl() {
        let mut cell = ConductivityCell::from_cell_constant(1.0);
        // 0.01 M KCl: κ = 0.001408 S/cm, if G_measured = 0.0014,
        // K_cal = 0.001408 / 0.0014 ≈ 1.00571
        let k = cell.calibrate_with_kcl(0.0014, 0.001408);
        assert!(approx_eq_rel(k, 1.00571, 0.01));
    }

    // ── MolarConductivity tests ─────────────────────────────────────────────

    #[test]
    fn test_molar_conductivity_from_kappa() {
        // Λm = κ / (c/1000) = 0.01 / (0.1/1000) = 100 S·cm²/mol
        let lm = MolarConductivity::from_kappa(0.01, 0.1);
        assert!(approx_eq(lm, 100.0, TOL));
    }

    #[test]
    fn test_molar_conductivity_to_kappa() {
        let kappa = MolarConductivity::to_kappa(100.0, 0.1);
        assert!(approx_eq(kappa, 0.01, TOL));
    }

    #[test]
    fn test_kohlrausch_nacl() {
        // Λ°m(NaCl) = λ°(Na+) + λ°(Cl-) = 50.1 + 76.3 = 126.4
        let lm = MolarConductivity::kohlrausch_limiting(&[(Ion::NaPlus, 1), (Ion::ClMinus, 1)]);
        assert!(approx_eq(lm, 126.4, TOL));
    }

    #[test]
    fn test_kohlrausch_caso4() {
        // Λ°m(CaSO4) = λ°(Ca²+) + λ°(SO4²-) = 119.0 + 160.0 = 279.0
        let lm = MolarConductivity::kohlrausch_limiting(&[(Ion::CaTwoPlus, 1), (Ion::SO4TwoMinus, 1)]);
        assert!(approx_eq(lm, 279.0, TOL));
    }

    #[test]
    fn test_kohlrausch_barium_chloride() {
        // Λ°m(BaCl2) = λ°(Ba²+) + 2×λ°(Cl-) = 127.2 + 2×76.3 = 279.8
        let lm = MolarConductivity::limiting_from_ions(Ion::BaTwoPlus, 1, Ion::ClMinus, 2);
        assert!(approx_eq(lm, 279.8, TOL));
    }

    #[test]
    fn test_kohlrausch_hcl() {
        // Λ°m(HCl) = λ°(H+) + λ°(Cl-) = 349.8 + 76.3 = 426.1
        let lm = MolarConductivity::kohlrausch_limiting(&[(Ion::HPlus, 1), (Ion::ClMinus, 1)]);
        assert!(approx_eq(lm, 426.1, TOL));
    }

    #[test]
    fn test_kohlrausch_naoh() {
        // Λ°m(NaOH) = λ°(Na+) + λ°(OH-) = 50.1 + 198.0 = 248.1
        let lm = MolarConductivity::kohlrausch_limiting(&[(Ion::NaPlus, 1), (Ion::OHMinus, 1)]);
        assert!(approx_eq(lm, 248.1, TOL));
    }

    // ── DebyeHuckelOnsager tests ────────────────────────────────────────────

    #[test]
    fn test_dho_at_zero_concentration() {
        let dho = DebyeHuckelOnsager::for_water_25c(126.4, 1);
        assert!(approx_eq(dho.lambda_m(0.0), 126.4, TOL));
    }

    #[test]
    fn test_dho_decreases_with_concentration() {
        let dho = DebyeHuckelOnsager::for_water_25c(126.4, 1);
        let lm_001 = dho.lambda_m(0.01);
        let lm_01 = dho.lambda_m(0.1);
        assert!(lm_001 > lm_01); // higher c → lower Λm
        assert!(lm_001 < 126.4); // less than limiting
    }

    #[test]
    fn test_dho_onsager_slope() {
        let dho = DebyeHuckelOnsager::for_water_25c(126.4, 1);
        let s = dho.onsager_slope();
        // S = 60.20 + 0.2289 × 126.4 ≈ 89.13
        assert!(approx_eq_rel(s, 89.13, 0.01));
    }

    #[test]
    fn test_dho_kappa() {
        let dho = DebyeHuckelOnsager::for_water_25c(126.4, 1);
        let kappa = dho.kappa(0.01);
        assert!(kappa > 0.0);
        // κ = Λm × c / 1000
        let lm = dho.lambda_m(0.01);
        assert!(approx_eq(kappa, lm * 0.01 / 1000.0, TOL));
    }

    #[test]
    fn test_dho_higher_charge() {
        let dho_1 = DebyeHuckelOnsager::for_water_25c(126.4, 1);
        let dho_2 = DebyeHuckelOnsager::for_water_25c(126.4, 2);
        // Higher charge → bigger Onsager slope → more decrease
        assert!(dho_2.lambda_m(0.01) < dho_1.lambda_m(0.01));
    }

    // ── Temperature compensation tests ──────────────────────────────────────

    #[test]
    fn test_temp_compensate_at_ref() {
        let result = temperature_compensate(100.0, 25.0, 25.0, 0.02);
        assert!(approx_eq(result, 100.0, TOL));
    }

    #[test]
    fn test_temp_compensate_above_ref() {
        // At 30°C: κ_ref = κ_meas / (1 + 0.02×5) = 100/1.1 ≈ 90.909
        let result = temperature_compensate(100.0, 30.0, 25.0, 0.02);
        assert!(approx_eq(result, 100.0 / 1.1, TOL));
    }

    #[test]
    fn test_temp_compensate_below_ref() {
        // At 20°C: κ_ref = κ_meas / (1 + 0.02×(-5)) = 100/0.9 ≈ 111.111
        let result = temperature_compensate(100.0, 20.0, 25.0, 0.02);
        assert!(approx_eq(result, 100.0 / 0.9, TOL));
    }

    // ── Degree of dissociation tests ────────────────────────────────────────

    #[test]
    fn test_degree_of_dissociation_full() {
        let alpha = degree_of_dissociation(126.4, 126.4);
        assert!(approx_eq(alpha, 1.0, TOL));
    }

    #[test]
    fn test_degree_of_dissociation_partial() {
        let alpha = degree_of_dissociation(63.2, 126.4);
        assert!(approx_eq(alpha, 0.5, TOL));
    }

    // ── Ka from conductivity tests ──────────────────────────────────────────

    #[test]
    fn test_ka_from_conductivity_acetic_acid() {
        // Acetic acid: c=0.1 M, Λm≈5.2, Λ°m≈390.5 (H+ + CH3COO-)
        let lambda_inf = Ion::HPlus.lambda0() + Ion::AcetateMinus.lambda0(); // 390.7
        let lambda_m = 5.2;
        let c = 0.1;
        let ka = ka_from_conductivity(c, lambda_m, lambda_inf);
        // α = 5.2/390.7 ≈ 0.01331, Ka = 0.1×0.01331²/(1-0.01331) ≈ 1.796e-5
        assert!(ka > 1e-6 && ka < 1e-4); // Reasonable range for acetic acid
    }

    #[test]
    fn test_ka_from_conductivity_weak_acid() {
        let ka = ka_from_conductivity(0.01, 10.0, 400.0);
        let alpha = 10.0 / 400.0;
        let expected = 0.01 * alpha * alpha / (1.0 - alpha);
        assert!(approx_eq(ka, expected, TOL));
    }

    // ── Ionic strength tests ────────────────────────────────────────────────

    #[test]
    fn test_ionic_strength_nacl() {
        // NaCl 0.1 M: I = 0.5 × (0.1×1² + 0.1×1²) = 0.1
        let i = ionic_strength(&[0.1, 0.1], &[1, -1]);
        assert!(approx_eq(i, 0.1, TOL));
    }

    #[test]
    fn test_ionic_strength_caso4() {
        // CaSO4 0.01 M: I = 0.5 × (0.01×4 + 0.01×4) = 0.04
        let i = ionic_strength(&[0.01, 0.01], &[2, -2]);
        assert!(approx_eq(i, 0.04, TOL));
    }

    // ── Activity coefficient tests ──────────────────────────────────────────

    #[test]
    fn test_activity_coefficient_zero_strength() {
        let gamma = activity_coefficient_dh(1, -1, 0.0);
        assert!(approx_eq(gamma, 1.0, TOL)); // ideal at zero I
    }

    #[test]
    fn test_activity_coefficient_decreases_with_i() {
        let g1 = activity_coefficient_dh(1, -1, 0.01);
        let g2 = activity_coefficient_dh(1, -1, 0.1);
        assert!(g2 < g1); // higher I → lower γ
        assert!(g1 < 1.0);
    }

    // ── ConductometricTitration tests ───────────────────────────────────────

    fn make_v_shaped_titration() -> ConductometricTitration {
        // Simulate HCl + NaOH: V-shaped conductivity curve
        // Before eq. point: conductivity decreases (H+ replaced by Na+)
        // After eq. point: conductivity increases (excess OH-)
        let volumes: Vec<f64> = (0..=20).map(|i| i as f64 * 1.0).collect();
        let eq_vol = 10.0;
        let conductivities: Vec<f64> = volumes
            .iter()
            .map(|&v| {
                if v <= eq_vol {
                    // Decreasing: 200 - 15*v (down from 200 to 50)
                    200.0 - 15.0 * v
                } else {
                    // Increasing: 50 + 12*(v-10) (up from 50)
                    50.0 + 12.0 * (v - eq_vol)
                }
            })
            .collect();
        ConductometricTitration::from_arrays(
            &volumes,
            &conductivities,
            TitrationType::StrongAcidStrongBase,
        )
    }

    #[test]
    fn test_titration_minimum_point() {
        let t = make_v_shaped_titration();
        let (v_min, _k_min) = t.minimum_point();
        assert!(approx_eq(v_min, 10.0, 0.5)); // near equivalence point
    }

    #[test]
    fn test_titration_first_derivative() {
        let t = make_v_shaped_titration();
        let deriv = t.first_derivative();
        assert!(!deriv.is_empty());
        // Before eq.: slope ≈ -15, After eq.: slope ≈ +12
        assert!(deriv[0].1 < 0.0); // decreasing initially
        assert!(deriv[deriv.len() - 1].1 > 0.0); // increasing at end
    }

    #[test]
    fn test_titration_second_derivative() {
        let t = make_v_shaped_titration();
        let second = t.second_derivative();
        assert!(!second.is_empty());
    }

    #[test]
    fn test_titration_dilution_correction() {
        let t = make_v_shaped_titration();
        let corrected = t.dilution_corrected(50.0);
        assert_eq!(corrected.len(), t.data.len());
        // At v=0, factor=1.0, so same. At v=10, factor=1.2, so higher.
        assert!(approx_eq(corrected[0].conductivity, t.data[0].conductivity, TOL));
        assert!(corrected[10].conductivity > t.data[10].conductivity);
    }

    // ── EndpointDetector tests ──────────────────────────────────────────────

    #[test]
    fn test_endpoint_detection_v_curve() {
        let t = make_v_shaped_titration();
        let detector = EndpointDetector::new().with_min_r_squared(0.95);
        let ep = detector.detect_endpoint(&t);
        assert!(ep.is_some());
        let ep = ep.unwrap();
        assert!(approx_eq(ep.volume_ml, 10.0, 1.0)); // within 1 mL of true eq. point
    }

    #[test]
    fn test_endpoint_pre_post_fit() {
        let t = make_v_shaped_titration();
        let detector = EndpointDetector::new();
        let ep = detector.detect_endpoint(&t).unwrap();
        assert!(ep.pre_fit.r_squared > 0.9);
        assert!(ep.post_fit.r_squared > 0.9);
        assert!(ep.pre_fit.slope < 0.0); // decreasing before eq.
        assert!(ep.post_fit.slope > 0.0); // increasing after eq.
    }

    #[test]
    fn test_endpoint_derivative_method() {
        let t = make_v_shaped_titration();
        let detector = EndpointDetector::new();
        let ep_vol = detector.detect_by_derivative(&t);
        assert!(ep_vol.is_some());
        assert!(approx_eq(ep_vol.unwrap(), 10.0, 2.0));
    }

    #[test]
    fn test_endpoint_default() {
        let d = EndpointDetector::default();
        assert!(approx_eq(d.min_r_squared, 0.90, TOL));
    }

    #[test]
    fn test_endpoint_with_min_points() {
        let d = EndpointDetector::new().with_min_points(5);
        assert_eq!(d.min_points_per_segment, 5);
    }

    // ── IonMobility tests ───────────────────────────────────────────────────

    #[test]
    fn test_ion_mobility_calculation() {
        let u = IonMobility::mobility(349.8, 1);
        assert!(approx_eq_rel(u, 3.625e-3, 0.01));
    }

    #[test]
    fn test_transport_number_hcl() {
        // t+(HCl) = u(H+) / (u(H+) + u(Cl-))
        let t_plus = IonMobility::transport_number_cation(&Ion::HPlus, &Ion::ClMinus);
        let expected = Ion::HPlus.lambda0() / (Ion::HPlus.lambda0() + Ion::ClMinus.lambda0());
        assert!(approx_eq(t_plus, expected, TOL));
        assert!(t_plus > 0.5); // H+ is faster
    }

    #[test]
    fn test_transport_number_sum() {
        let t_plus = IonMobility::transport_number_cation(&Ion::NaPlus, &Ion::ClMinus);
        let t_minus = IonMobility::transport_number_anion(&Ion::NaPlus, &Ion::ClMinus);
        assert!(approx_eq(t_plus + t_minus, 1.0, TOL));
    }

    #[test]
    fn test_hittorf_method() {
        // If 0.001 mol lost from anode, z=1, I=1A, t=96.485 s → total eq = 0.001
        // t+ = 0.001/0.001 = 1.0
        let t_plus = IonMobility::hittorf_transport_number(0.001, 1, 1.0, 96.485);
        assert!(approx_eq_rel(t_plus, 1.0, 0.01));
    }

    #[test]
    fn test_stokes_radius() {
        let r = IonMobility::stokes_radius(&Ion::NaPlus, WATER_VISCOSITY_25C);
        // Stokes radius of Na+ is about 1.83 Å = 1.83e-10 m
        assert!(r > 1e-10 && r < 5e-10);
    }

    #[test]
    fn test_walden_product() {
        let wp = IonMobility::walden_product(126.4, WATER_VISCOSITY_25C);
        assert!(wp > 0.0);
    }

    #[test]
    fn test_diffusion_coefficient() {
        let d = IonMobility::diffusion_coefficient(&Ion::NaPlus, 298.15);
        // D(Na+) ≈ 1.33e-9 m²/s
        assert!(d > 1e-10 && d < 1e-8);
    }

    // ── WheatstoneACBridge tests ────────────────────────────────────────────

    #[test]
    fn test_bridge_balance_condition() {
        let bridge = WheatstoneACBridge::new(100.0, 100.0, 1000.0, 1000.0);
        let r_cell = bridge.cell_resistance_at_balance();
        assert!(approx_eq(r_cell, 1000.0, TOL)); // R3 × R2 / R1
    }

    #[test]
    fn test_bridge_conductance() {
        let bridge = WheatstoneACBridge::new(100.0, 100.0, 1000.0, 1000.0);
        let g = bridge.conductance_at_balance();
        assert!(approx_eq(g, 0.001, TOL));
    }

    #[test]
    fn test_bridge_impedance_no_cap() {
        let bridge = WheatstoneACBridge::new(100.0, 100.0, 1000.0, 1000.0);
        let z = bridge.cell_impedance_magnitude(1000.0, 0.0);
        assert!(approx_eq(z, 1000.0, TOL)); // no capacitance → Z = R
    }

    #[test]
    fn test_bridge_impedance_with_cap() {
        let bridge = WheatstoneACBridge::new(100.0, 100.0, 1000.0, 1000.0);
        let z = bridge.cell_impedance_magnitude(1000.0, 1e-9);
        assert!(z < 1000.0); // capacitance reduces impedance
    }

    #[test]
    fn test_bridge_phase_no_cap() {
        let bridge = WheatstoneACBridge::new(100.0, 100.0, 1000.0, 1000.0);
        let phase = bridge.cell_impedance_phase(1000.0, 0.0);
        assert!(approx_eq(phase, 0.0, TOL)); // purely resistive
    }

    #[test]
    fn test_bridge_frequency_error() {
        let bridge = WheatstoneACBridge::new(100.0, 100.0, 1000.0, 1000.0);
        let err = bridge.frequency_error_fraction(1000.0, 1e-12);
        assert!(err >= 0.0 && err < 0.01); // small error at small capacitance
    }

    #[test]
    fn test_bridge_compensated_conductance() {
        let bridge = WheatstoneACBridge::new(100.0, 100.0, 1000.0, 1000.0)
            .with_capacitor(1e-12);
        let g = bridge.compensated_conductance(1e-12);
        // With exact compensation, should equal uncompensated
        assert!(approx_eq(g, bridge.conductance_at_balance(), 1e-4));
    }

    #[test]
    fn test_optimal_frequency() {
        let f = WheatstoneACBridge::optimal_frequency(1000.0, 1e-9);
        // f ≈ 1/(2π×1000×1e-9) ≈ 159.15 kHz
        assert!(approx_eq_rel(f, 159154.9, 0.01));
    }

    // ── KohlrauschExtrapolation tests ───────────────────────────────────────

    #[test]
    fn test_kohlrausch_extrapolation_basic() {
        let ke = KohlrauschExtrapolation::new();
        // Simulated weak acid data: Λm approaches Λ°m=390 as c→0
        let concs = vec![0.1, 0.05, 0.01, 0.005, 0.001];
        let lambda0_true = 390.7;
        let ka_true = 1.8e-5;

        // Generate synthetic Λm from Ostwald dilution: Ka = c·α²/(1-α), α=Λm/Λ°m
        let lambda_m_vals: Vec<f64> = concs.iter().map(|&c| {
            // Solve for α: Ka(1-α) = c·α² → c·α² + Ka·α - Ka = 0
            let a = c;
            let b = ka_true;
            let disc: f64 = b * b + 4.0 * a * ka_true;
            let alpha = (-b + disc.sqrt()) / (2.0 * a);
            alpha * lambda0_true
        }).collect();

        let result = ke.extrapolate(&concs, &lambda_m_vals, 400.0);
        assert!(result.is_some());
        let (l0, ka) = result.unwrap();
        assert!(approx_eq_rel(l0, lambda0_true, 0.1)); // within 10%
        // Ka might be less accurate due to the extrapolation method
        assert!(ka > 0.0);
    }

    #[test]
    fn test_kohlrausch_extrapolation_default() {
        let ke = KohlrauschExtrapolation::default();
        assert_eq!(ke.max_iterations, 100);
    }

    // ── WaterQualityFromConductivity tests ──────────────────────────────────

    #[test]
    fn test_water_quality_ultrapure() {
        assert_eq!(
            WaterQualityFromConductivity::classify(0.05),
            WaterQuality::UltraPure
        );
    }

    #[test]
    fn test_water_quality_purified() {
        assert_eq!(
            WaterQualityFromConductivity::classify(1.0),
            WaterQuality::Purified
        );
    }

    #[test]
    fn test_water_quality_deionized() {
        assert_eq!(
            WaterQualityFromConductivity::classify(5.0),
            WaterQuality::Deionized
        );
    }

    #[test]
    fn test_water_quality_drinking() {
        assert_eq!(
            WaterQualityFromConductivity::classify(500.0),
            WaterQuality::Drinking
        );
    }

    #[test]
    fn test_water_quality_brackish() {
        assert_eq!(
            WaterQualityFromConductivity::classify(5000.0),
            WaterQuality::Brackish
        );
    }

    #[test]
    fn test_water_quality_saline() {
        assert_eq!(
            WaterQualityFromConductivity::classify(30000.0),
            WaterQuality::Saline
        );
    }

    #[test]
    fn test_water_quality_brine() {
        assert_eq!(
            WaterQualityFromConductivity::classify(60000.0),
            WaterQuality::Brine
        );
    }

    #[test]
    fn test_tds_estimation_default() {
        let tds = WaterQualityFromConductivity::estimate_tds(1000.0, None);
        assert!(approx_eq(tds, 640.0, TOL)); // 0.64 × 1000
    }

    #[test]
    fn test_tds_estimation_custom_factor() {
        let tds = WaterQualityFromConductivity::estimate_tds(1000.0, Some(0.5));
        assert!(approx_eq(tds, 500.0, TOL));
    }

    #[test]
    fn test_usp_compliant() {
        assert!(WaterQualityFromConductivity::is_usp_compliant(1.0));
        assert!(WaterQualityFromConductivity::is_usp_compliant(1.3));
        assert!(!WaterQualityFromConductivity::is_usp_compliant(1.4));
    }

    #[test]
    fn test_wfi_compliant() {
        assert!(WaterQualityFromConductivity::is_wfi_compliant(1.0, Some(400.0)));
        assert!(!WaterQualityFromConductivity::is_wfi_compliant(1.0, Some(600.0)));
        assert!(!WaterQualityFromConductivity::is_wfi_compliant(1.5, Some(400.0)));
    }

    #[test]
    fn test_resistivity() {
        let r = WaterQualityFromConductivity::resistivity_mohm_cm(0.055);
        assert!(approx_eq(r, 1.0 / 0.055, TOL)); // ≈ 18.18 MΩ·cm
    }

    #[test]
    fn test_resistivity_zero() {
        let r = WaterQualityFromConductivity::resistivity_mohm_cm(0.0);
        assert!(r.is_infinite());
    }

    #[test]
    fn test_salinity_estimation() {
        // Seawater at 25°C: κ ≈ 53 mS/cm → S ≈ 35 PSU
        let s = WaterQualityFromConductivity::estimate_salinity_psu(53.087);
        assert!(approx_eq_rel(s, 35.0, 0.05)); // within 5%
    }

    #[test]
    fn test_usp_stage1_limit_25c() {
        let limit = WaterQualityFromConductivity::usp_stage1_limit(25.0);
        assert!(approx_eq(limit, 1.3, TOL));
    }

    #[test]
    fn test_usp_stage1_limit_increases_with_temp() {
        let l20 = WaterQualityFromConductivity::usp_stage1_limit(20.0);
        let l50 = WaterQualityFromConductivity::usp_stage1_limit(50.0);
        assert!(l50 > l20);
    }

    // ── Linear regression tests ─────────────────────────────────────────────

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0]; // y = 2x
        let fit = linear_regression(&x, &y);
        assert!(approx_eq(fit.slope, 2.0, TOL));
        assert!(approx_eq(fit.intercept, 0.0, TOL));
        assert!(approx_eq(fit.r_squared, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_with_intercept() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![5.0, 8.0, 11.0, 14.0]; // y = 3x + 5
        let fit = linear_regression(&x, &y);
        assert!(approx_eq(fit.slope, 3.0, TOL));
        assert!(approx_eq(fit.intercept, 5.0, TOL));
    }

    // ── KCl standard tests ──────────────────────────────────────────────────

    #[test]
    fn test_kcl_standard_1m() {
        let k = kcl_standard_conductivity(1.0);
        assert!(approx_eq(k, 0.11131, TOL));
    }

    #[test]
    fn test_kcl_standard_01m() {
        let k = kcl_standard_conductivity(0.1);
        assert!(approx_eq(k, 0.012856, TOL));
    }

    #[test]
    fn test_kcl_standard_001m() {
        let k = kcl_standard_conductivity(0.01);
        assert!(approx_eq(k, 0.001408, TOL));
    }

    // ── Solvent conductivity tests ──────────────────────────────────────────

    #[test]
    fn test_solvent_conductivity_water() {
        let k = solvent_conductivity("water");
        assert!(approx_eq(k, 5.5e-8, 1e-10));
    }

    #[test]
    fn test_solvent_conductivity_case_insensitive() {
        let k1 = solvent_conductivity("Water");
        let k2 = solvent_conductivity("WATER");
        assert_eq!(k1, k2);
    }

    #[test]
    fn test_solvent_conductivity_ethanol() {
        let k = solvent_conductivity("ethanol");
        assert!(k > 0.0 && k < 1e-5);
    }

    // ── Strong acid-base conductivity tests ─────────────────────────────────

    #[test]
    fn test_strong_acid_base_before_eq() {
        let k1 = strong_acid_base_conductivity(50.0, 0.1, 1.0, 0.1);
        let k2 = strong_acid_base_conductivity(50.0, 0.1, 5.0, 0.1);
        // Before equivalence: conductivity should generally decrease
        // (H+ replaced by Na+)
        assert!(k1 > 0.0);
        assert!(k2 > 0.0);
    }

    #[test]
    fn test_strong_acid_base_after_eq() {
        let k = strong_acid_base_conductivity(50.0, 0.1, 60.0, 0.1);
        // After equivalence: excess OH- contributes
        assert!(k > 0.0);
    }

    // ── Multiple endpoint detection tests ───────────────────────────────────

    #[test]
    fn test_multiple_endpoints_single() {
        let t = make_v_shaped_titration();
        let detector = EndpointDetector::new();
        let eps = detector.detect_multiple_endpoints(&t, 1);
        assert!(!eps.is_empty());
    }

    #[test]
    fn test_multiple_endpoints_zero() {
        let t = make_v_shaped_titration();
        let detector = EndpointDetector::new();
        let eps = detector.detect_multiple_endpoints(&t, 0);
        assert!(eps.is_empty());
    }

    // ── Integration / cross-check tests ─────────────────────────────────────

    #[test]
    fn test_kohlrausch_nacl_roundtrip() {
        // Compute Λ°m for NaCl, then use DHO to get κ at 0.01 M, then back to Λm
        let l0 = MolarConductivity::kohlrausch_limiting(&[(Ion::NaPlus, 1), (Ion::ClMinus, 1)]);
        let dho = DebyeHuckelOnsager::for_water_25c(l0, 1);
        let kappa = dho.kappa(0.01);
        let lm_back = MolarConductivity::from_kappa(kappa, 0.01);
        let lm_direct = dho.lambda_m(0.01);
        assert!(approx_eq(lm_back, lm_direct, TOL));
    }

    #[test]
    fn test_cell_roundtrip_kappa_conductance() {
        let cell = ConductivityCell::from_cell_constant(0.5);
        let kappa = 0.01;
        let g = cell.kappa_to_conductance(kappa);
        let kappa_back = cell.conductance_to_kappa(g);
        assert!(approx_eq(kappa_back, kappa, TOL));
    }

    #[test]
    fn test_transport_number_kcl_symmetry() {
        // For KCl, transport numbers should be close to 0.5
        // since K+ and Cl- have similar mobilities
        let t_plus = IonMobility::transport_number_cation(&Ion::KPlus, &Ion::ClMinus);
        assert!(t_plus > 0.4 && t_plus < 0.6);
    }

    #[test]
    fn test_water_quality_distilled() {
        assert_eq!(
            WaterQualityFromConductivity::classify(20.0),
            WaterQuality::Distilled
        );
    }

    #[test]
    fn test_ion_all_lambda0_positive() {
        let ions = [
            Ion::HPlus, Ion::OHMinus, Ion::NaPlus, Ion::KPlus, Ion::LiPlus,
            Ion::ClMinus, Ion::BrMinus, Ion::NO3Minus, Ion::AcetateMinus,
            Ion::SO4TwoMinus, Ion::CaTwoPlus, Ion::MgTwoPlus, Ion::BaTwoPlus,
            Ion::AgPlus, Ion::NH4Plus, Ion::FMinus, Ion::IMinus, Ion::CO3TwoMinus,
        ];
        for ion in &ions {
            assert!(ion.lambda0() > 0.0, "lambda0 must be positive for {:?}", ion);
        }
    }

    #[test]
    fn test_ion_mobility_ordering() {
        // H+ should have the highest mobility among monovalent cations
        assert!(Ion::HPlus.mobility() > Ion::NaPlus.mobility());
        assert!(Ion::HPlus.mobility() > Ion::KPlus.mobility());
        assert!(Ion::HPlus.mobility() > Ion::LiPlus.mobility());
    }
}

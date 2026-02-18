//! # Coulometric Titration Processor
//!
//! Coulometric titration signal processing for precise quantitative analysis
//! using Faraday's law of electrolysis. Coulometric titration generates the
//! titrant *in situ* electrochemically, yielding absolute measurements without
//! volumetric glassware.
//!
//! ## Key Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`FaradayLawProcessor`] | m = MIt/(nF) mass-charge conversion, charge integration |
//! | [`ConstantCurrentSource`] | Galvanostatic current source model with regulation precision |
//! | [`EndpointDetection`] | Potentiometric endpoint by first/second derivative or bipotentiometric dead-stop |
//! | [`KarlFischerCoulometric`] | Water determination via coulometric I2 generation with drift correction |
//! | [`AcidBaseCoulometric`] | OH-/H+ generation at Pt electrode for acid-base titrations |
//! | [`RedoxCoulometric`] | Br2 or Ce(IV) generation for redox titrations |
//! | [`ChargeIntegrator`] | High-precision trapezoidal coulomb counting with error analysis |
//! | [`DriftCompensation`] | Background current correction for Karl Fischer |
//! | [`CoulometricSession`] | Calibration, sample sequence, blank correction, QC checks |
//!
//! ## Physics
//!
//! - Faraday's law: m = Q·M / (n·F), where F = 96485.33212 C/mol
//! - Karl Fischer: 1 mol H2O ≡ 2 × F coulombs ≡ 192970.66 C
//! - 1 mg H2O = 192970.66 / 18015.0 × 1000 ≈ 10.712 C
//! - Endpoint: inflection point in E vs Q curve (dE/dQ maximum, d²E/dQ² = 0)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::coulometric_titration_processor::{
//!     FaradayLawProcessor, charge_to_mass, water_from_charge,
//! };
//!
//! let processor = FaradayLawProcessor::new(18.015, 2);
//! let mass_g = processor.charge_to_mass(192970.66);
//! assert!((mass_g - 18.015).abs() < 0.01);
//!
//! // Convenience functions
//! let mg_water = water_from_charge(10.712);
//! assert!((mg_water - 1.0).abs() < 0.01);
//! ```

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Faraday constant in C/mol.
const FARADAY_CONSTANT: f64 = 96485.33212;

/// Molar mass of water in g/mol.
const WATER_MOLAR_MASS: f64 = 18.015;

/// Electrons transferred per mole H2O in Karl Fischer reaction.
const WATER_N_ELECTRONS: u32 = 2;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Convert charge (coulombs) to mass (grams) using Faraday's law.
///
/// m = Q × M / (n × F)
///
/// * `charge_c` - charge in coulombs
/// * `molar_mass` - molar mass of analyte in g/mol
/// * `n_electrons` - number of electrons per formula unit
pub fn charge_to_mass(charge_c: f64, molar_mass: f64, n_electrons: u32) -> f64 {
    charge_c * molar_mass / (n_electrons as f64 * FARADAY_CONSTANT)
}

/// Convert mass (grams) to charge (coulombs) using Faraday's law.
///
/// Q = m × n × F / M
///
/// * `mass_g` - mass in grams
/// * `molar_mass` - molar mass of analyte in g/mol
/// * `n_electrons` - number of electrons per formula unit
pub fn mass_to_charge(mass_g: f64, molar_mass: f64, n_electrons: u32) -> f64 {
    mass_g * n_electrons as f64 * FARADAY_CONSTANT / molar_mass
}

/// Convert charge (coulombs) to milligrams of H2O.
///
/// Uses M = 18.015 g/mol, n = 2 electrons (Karl Fischer reaction).
pub fn water_from_charge(charge_c: f64) -> f64 {
    let mass_g: f64 = charge_to_mass(charge_c, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
    mass_g * 1000.0
}

// ---------------------------------------------------------------------------
// FaradayLawProcessor
// ---------------------------------------------------------------------------

/// Faraday's law processor for mass-charge conversion.
///
/// Applies m = Q·M / (n·F) and integrates current-time data to obtain charge.
#[derive(Debug, Clone)]
pub struct FaradayLawProcessor {
    /// Molar mass of analyte in g/mol.
    pub molar_mass: f64,
    /// Number of electrons per formula unit.
    pub n_electrons: u32,
    /// Current efficiency (fraction 0..1, default 1.0).
    pub current_efficiency: f64,
}

impl FaradayLawProcessor {
    /// Create a new Faraday law processor.
    pub fn new(molar_mass: f64, n_electrons: u32) -> Self {
        Self {
            molar_mass,
            n_electrons,
            current_efficiency: 1.0,
        }
    }

    /// Create with specified current efficiency.
    pub fn with_efficiency(mut self, efficiency: f64) -> Self {
        self.current_efficiency = efficiency.clamp(0.0, 1.0);
        self
    }

    /// Convert charge to mass (grams), accounting for current efficiency.
    pub fn charge_to_mass(&self, charge_c: f64) -> f64 {
        let effective_charge: f64 = charge_c * self.current_efficiency;
        charge_to_mass(effective_charge, self.molar_mass, self.n_electrons)
    }

    /// Convert mass to charge (coulombs), accounting for current efficiency.
    pub fn mass_to_charge(&self, mass_g: f64) -> f64 {
        let base_charge: f64 = mass_to_charge(mass_g, self.molar_mass, self.n_electrons);
        base_charge / self.current_efficiency
    }

    /// Integrate current over time (trapezoidal rule) and compute mass.
    ///
    /// Returns `(charge_c, mass_g)`.
    pub fn integrate_current(&self, time_s: &[f64], current_a: &[f64]) -> (f64, f64) {
        assert_eq!(time_s.len(), current_a.len(), "time and current arrays must match");
        assert!(time_s.len() >= 2, "need at least 2 data points");
        let charge_c: f64 = trapezoidal_integrate(time_s, current_a);
        let mass_g: f64 = self.charge_to_mass(charge_c);
        (charge_c, mass_g)
    }

    /// Compute coulombs needed for a target mass.
    pub fn required_charge(&self, target_mass_g: f64) -> f64 {
        self.mass_to_charge(target_mass_g)
    }

    /// Compute theoretical Faraday constant from experimental data.
    ///
    /// Given measured charge and known mass/molar mass/n, computes F_exp.
    pub fn experimental_faraday(&self, charge_c: f64, mass_g: f64) -> f64 {
        if mass_g.abs() < 1e-30 {
            return 0.0;
        }
        charge_c * self.molar_mass / (self.n_electrons as f64 * mass_g)
    }
}

// ---------------------------------------------------------------------------
// ConstantCurrentSource
// ---------------------------------------------------------------------------

/// Galvanostatic constant-current source model.
///
/// In coulometric titrations the generating current is held constant, so
/// Q = I × t. This struct models the current source including regulation
/// precision and compliance voltage limits.
#[derive(Debug, Clone)]
pub struct ConstantCurrentSource {
    /// Set current in amperes.
    pub set_current_a: f64,
    /// Regulation precision (relative, e.g. 0.001 for 0.1%).
    pub precision: f64,
    /// Maximum compliance voltage (V).
    pub compliance_v: f64,
    /// Accumulated charge in coulombs.
    accumulated_charge_c: f64,
    /// Elapsed time in seconds.
    elapsed_time_s: f64,
}

impl ConstantCurrentSource {
    /// Create a new constant-current source.
    pub fn new(set_current_a: f64, precision: f64, compliance_v: f64) -> Self {
        Self {
            set_current_a,
            precision,
            compliance_v,
            accumulated_charge_c: 0.0,
            elapsed_time_s: 0.0,
        }
    }

    /// Create a typical micro-ampere source for Karl Fischer.
    pub fn karl_fischer_preset() -> Self {
        Self::new(0.0004, 0.001, 10.0) // 400 uA, 0.1% precision, 10V compliance
    }

    /// Create a milliamp source for general coulometric titration.
    pub fn general_preset() -> Self {
        Self::new(0.050, 0.0005, 20.0) // 50 mA, 0.05% precision, 20V compliance
    }

    /// Actual current including regulation error.
    pub fn actual_current(&self) -> f64 {
        self.set_current_a * (1.0 + self.precision)
    }

    /// Worst-case current (lower bound).
    pub fn min_current(&self) -> f64 {
        self.set_current_a * (1.0 - self.precision)
    }

    /// Advance time by `dt` seconds, accumulating charge.
    pub fn advance(&mut self, dt_s: f64) {
        self.accumulated_charge_c += self.set_current_a * dt_s;
        self.elapsed_time_s += dt_s;
    }

    /// Get accumulated charge.
    pub fn charge(&self) -> f64 {
        self.accumulated_charge_c
    }

    /// Get elapsed time.
    pub fn elapsed(&self) -> f64 {
        self.elapsed_time_s
    }

    /// Charge at endpoint Q = I × t.
    pub fn charge_at_time(&self, t_s: f64) -> f64 {
        self.set_current_a * t_s
    }

    /// Charge uncertainty from regulation precision.
    pub fn charge_uncertainty(&self) -> f64 {
        self.accumulated_charge_c * self.precision
    }

    /// Reset the source.
    pub fn reset(&mut self) {
        self.accumulated_charge_c = 0.0;
        self.elapsed_time_s = 0.0;
    }

    /// Check if cell voltage exceeds compliance.
    pub fn is_compliance_limited(&self, cell_voltage: f64) -> bool {
        cell_voltage.abs() >= self.compliance_v
    }
}

// ---------------------------------------------------------------------------
// EndpointDetection
// ---------------------------------------------------------------------------

/// Endpoint detection method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EndpointMethod {
    /// First derivative maximum: dE/dV peak.
    FirstDerivative,
    /// Second derivative zero crossing: d²E/dV² = 0.
    SecondDerivative,
    /// Bipotentiometric (dead-stop) endpoint.
    Bipotentiometric,
    /// Threshold-based: potential crosses a fixed value.
    PotentialThreshold(f64),
}

/// Result of endpoint detection.
#[derive(Debug, Clone)]
pub struct EndpointResult {
    /// Index in the data arrays.
    pub index: usize,
    /// Charge at endpoint in coulombs.
    pub charge_c: f64,
    /// Potential at endpoint in volts.
    pub potential_v: f64,
    /// Confidence score (0..1).
    pub confidence: f64,
    /// Method used.
    pub method: EndpointMethod,
}

/// Endpoint detection for potentiometric coulometric titration.
///
/// Monitors the indicator electrode potential as a function of accumulated
/// charge and detects the equivalence point using derivative methods.
#[derive(Debug, Clone)]
pub struct EndpointDetection {
    /// Accumulated charge values (x-axis) in coulombs.
    pub charge_c: Vec<f64>,
    /// Indicator electrode potential values (y-axis) in volts.
    pub potential_v: Vec<f64>,
}

impl EndpointDetection {
    /// Create a new endpoint detector from charge-potential data.
    pub fn new(charge_c: Vec<f64>, potential_v: Vec<f64>) -> Self {
        assert_eq!(charge_c.len(), potential_v.len(), "arrays must match");
        Self { charge_c, potential_v }
    }

    /// Add a new data point.
    pub fn push(&mut self, charge: f64, potential: f64) {
        self.charge_c.push(charge);
        self.potential_v.push(potential);
    }

    /// Detect endpoint using first derivative method.
    ///
    /// Finds the charge value where |dE/dQ| is maximum.
    pub fn first_derivative_endpoint(&self) -> Option<EndpointResult> {
        if self.charge_c.len() < 3 {
            return None;
        }
        let derivs: Vec<f64> = self.first_derivative();
        let mut max_idx: usize = 0;
        let mut max_val: f64 = 0.0;
        for (i, &d) in derivs.iter().enumerate() {
            let abs_d: f64 = d.abs();
            if abs_d > max_val {
                max_val = abs_d;
                max_idx = i;
            }
        }
        // Map derivative index back to data index (offset by 1 for central diff)
        let data_idx: usize = max_idx + 1;
        if data_idx >= self.charge_c.len() {
            return None;
        }
        // Confidence from ratio of max derivative to mean derivative
        let mean_abs: f64 = derivs.iter().map(|d| d.abs()).sum::<f64>() / derivs.len() as f64;
        let confidence: f64 = if mean_abs > 1e-30 {
            (max_val / mean_abs / 10.0).min(1.0)
        } else {
            0.0
        };

        Some(EndpointResult {
            index: data_idx,
            charge_c: self.charge_c[data_idx],
            potential_v: self.potential_v[data_idx],
            confidence,
            method: EndpointMethod::FirstDerivative,
        })
    }

    /// Detect endpoint using second derivative zero-crossing method.
    ///
    /// Finds where d²E/dQ² crosses zero.
    pub fn second_derivative_endpoint(&self) -> Option<EndpointResult> {
        if self.charge_c.len() < 5 {
            return None;
        }
        let second_derivs: Vec<f64> = self.second_derivative();
        // Find zero crossing with the largest surrounding magnitude (most significant)
        let mut best_idx: Option<usize> = None;
        let mut best_magnitude: f64 = 0.0;
        let mut best_abs: f64 = f64::MAX;
        for i in 0..second_derivs.len().saturating_sub(1) {
            let d0: f64 = second_derivs[i];
            let d1: f64 = second_derivs[i + 1];
            if d0 * d1 <= 0.0 {
                // Zero crossing significance = max magnitude on either side
                let crossing_mag: f64 = d0.abs().max(d1.abs());
                if crossing_mag > best_magnitude {
                    best_magnitude = crossing_mag;
                    let (candidate_idx, candidate_abs) = if d0.abs() < d1.abs() {
                        (i, d0.abs())
                    } else {
                        (i + 1, d1.abs())
                    };
                    best_idx = Some(candidate_idx);
                    best_abs = candidate_abs;
                }
            }
        }
        let min_abs: f64 = best_abs;
        let idx = best_idx?;
        // Map back to data index (second derivative offset by 2)
        let data_idx: usize = idx + 2;
        if data_idx >= self.charge_c.len() {
            return None;
        }
        let max_abs: f64 = second_derivs.iter().map(|d| d.abs()).fold(0.0_f64, f64::max);
        let confidence: f64 = if max_abs > 1e-30 {
            (1.0 - min_abs / max_abs).max(0.0)
        } else {
            0.0
        };

        Some(EndpointResult {
            index: data_idx,
            charge_c: self.charge_c[data_idx],
            potential_v: self.potential_v[data_idx],
            confidence,
            method: EndpointMethod::SecondDerivative,
        })
    }

    /// Detect bipotentiometric (dead-stop) endpoint.
    ///
    /// In this method, two identical indicator electrodes are used with a small
    /// constant current. The potential difference drops to near zero at the
    /// equivalence point.
    pub fn bipotentiometric_endpoint(&self, threshold_v: f64) -> Option<EndpointResult> {
        if self.charge_c.len() < 2 {
            return None;
        }
        // Find first index where potential drops below threshold
        for i in 1..self.potential_v.len() {
            if self.potential_v[i - 1] > threshold_v && self.potential_v[i] <= threshold_v {
                // Linear interpolation for more precise endpoint
                let dv: f64 = self.potential_v[i - 1] - self.potential_v[i];
                let frac: f64 = if dv.abs() > 1e-30 {
                    (self.potential_v[i - 1] - threshold_v) / dv
                } else {
                    0.5
                };
                let charge: f64 = self.charge_c[i - 1]
                    + frac * (self.charge_c[i] - self.charge_c[i - 1]);
                return Some(EndpointResult {
                    index: i,
                    charge_c: charge,
                    potential_v: threshold_v,
                    confidence: 0.9,
                    method: EndpointMethod::Bipotentiometric,
                });
            }
        }
        None
    }

    /// Detect endpoint by potential threshold crossing.
    pub fn threshold_endpoint(&self, threshold_v: f64) -> Option<EndpointResult> {
        if self.charge_c.len() < 2 {
            return None;
        }
        for i in 1..self.potential_v.len() {
            let prev: f64 = self.potential_v[i - 1];
            let curr: f64 = self.potential_v[i];
            // Detect crossing in either direction
            if (prev < threshold_v && curr >= threshold_v)
                || (prev > threshold_v && curr <= threshold_v)
            {
                let dv: f64 = curr - prev;
                let frac: f64 = if dv.abs() > 1e-30 {
                    (threshold_v - prev) / dv
                } else {
                    0.5
                };
                let charge: f64 = self.charge_c[i - 1]
                    + frac * (self.charge_c[i] - self.charge_c[i - 1]);
                return Some(EndpointResult {
                    index: i,
                    charge_c: charge,
                    potential_v: threshold_v,
                    confidence: 0.85,
                    method: EndpointMethod::PotentialThreshold(threshold_v),
                });
            }
        }
        None
    }

    /// Compute first derivative dE/dQ via central differences.
    pub fn first_derivative(&self) -> Vec<f64> {
        let n: usize = self.charge_c.len();
        if n < 3 {
            return vec![];
        }
        let mut derivs: Vec<f64> = Vec::with_capacity(n - 2);
        for i in 1..n - 1 {
            let dq: f64 = self.charge_c[i + 1] - self.charge_c[i - 1];
            let de: f64 = self.potential_v[i + 1] - self.potential_v[i - 1];
            let d: f64 = if dq.abs() > 1e-30 { de / dq } else { 0.0 };
            derivs.push(d);
        }
        derivs
    }

    /// Compute second derivative d²E/dQ² via central differences.
    pub fn second_derivative(&self) -> Vec<f64> {
        let n: usize = self.charge_c.len();
        if n < 5 {
            return vec![];
        }
        let first: Vec<f64> = self.first_derivative();
        if first.len() < 3 {
            return vec![];
        }
        // Use original charge grid shifted for derivative midpoints
        let mut second: Vec<f64> = Vec::with_capacity(first.len() - 2);
        for i in 1..first.len() - 1 {
            // Indices in original data: i maps to charge_c[i+1]
            let dq: f64 = self.charge_c[i + 2] - self.charge_c[i];
            let dd: f64 = first[i + 1] - first[i - 1];
            let d2: f64 = if dq.abs() > 1e-30 { dd / dq } else { 0.0 };
            second.push(d2);
        }
        second
    }

    /// Detect endpoint using the specified method.
    pub fn detect(&self, method: EndpointMethod) -> Option<EndpointResult> {
        match method {
            EndpointMethod::FirstDerivative => self.first_derivative_endpoint(),
            EndpointMethod::SecondDerivative => self.second_derivative_endpoint(),
            EndpointMethod::Bipotentiometric => self.bipotentiometric_endpoint(0.01),
            EndpointMethod::PotentialThreshold(v) => self.threshold_endpoint(v),
        }
    }
}

// ---------------------------------------------------------------------------
// KarlFischerCoulometric
// ---------------------------------------------------------------------------

/// Karl Fischer coulometric water determination.
///
/// Water reacts with iodine generated at the generator electrode:
///   H2O + I2 + SO2 + 3 Base → 2 HI + Base·SO3
///
/// Since I2 is produced coulometrically from I-, the charge consumed is
/// directly proportional to water content: 1 mol H2O = 2 × F coulombs.
#[derive(Debug, Clone)]
pub struct KarlFischerCoulometric {
    /// Generator electrode current in amperes.
    pub generator_current_a: f64,
    /// Sensor electrode bipotentiometric threshold (mV).
    pub sensor_threshold_mv: f64,
    /// Pre-titration drift rate in ug/min.
    pub drift_rate_ug_per_min: f64,
    /// Accumulated charge in coulombs.
    charge_c: f64,
    /// Elapsed titration time in seconds.
    elapsed_s: f64,
    /// Whether titration is active.
    titrating: bool,
    /// Drift compensator.
    drift_comp: DriftCompensation,
}

impl KarlFischerCoulometric {
    /// Create a new Karl Fischer processor.
    pub fn new(generator_current_a: f64, sensor_threshold_mv: f64) -> Self {
        Self {
            generator_current_a,
            sensor_threshold_mv,
            drift_rate_ug_per_min: 0.0,
            charge_c: 0.0,
            elapsed_s: 0.0,
            titrating: false,
            drift_comp: DriftCompensation::new(0.0),
        }
    }

    /// Standard KF cell: 400 uA generator, 30 mV sensor threshold.
    pub fn standard_cell() -> Self {
        Self::new(0.0004, 30.0)
    }

    /// Set the pre-titration drift rate (ug/min).
    pub fn set_drift_rate(&mut self, rate_ug_per_min: f64) {
        self.drift_rate_ug_per_min = rate_ug_per_min;
        self.drift_comp = DriftCompensation::new(rate_ug_per_min);
    }

    /// Start titration.
    pub fn start(&mut self) {
        self.titrating = true;
        self.charge_c = 0.0;
        self.elapsed_s = 0.0;
    }

    /// Advance titration by dt seconds.
    pub fn advance(&mut self, dt_s: f64) {
        if self.titrating {
            self.charge_c += self.generator_current_a * dt_s;
            self.elapsed_s += dt_s;
        }
    }

    /// Stop titration.
    pub fn stop(&mut self) {
        self.titrating = false;
    }

    /// Raw water content in milligrams from accumulated charge.
    pub fn raw_water_mg(&self) -> f64 {
        water_from_charge(self.charge_c)
    }

    /// Net water content after drift correction.
    pub fn net_water_mg(&self) -> f64 {
        let raw: f64 = self.raw_water_mg();
        let drift_ug: f64 = self.drift_comp.drift_correction_ug(self.elapsed_s);
        let net: f64 = raw - drift_ug / 1000.0;
        if net < 0.0 { 0.0 } else { net }
    }

    /// Compute water concentration in ppm given sample mass.
    pub fn water_ppm(&self, sample_mass_g: f64) -> f64 {
        if sample_mass_g <= 0.0 {
            return 0.0;
        }
        let net_mg: f64 = self.net_water_mg();
        (net_mg / sample_mass_g) * 1000.0 // mg/g = ppm (w/w)
    }

    /// Compute water concentration as percent.
    pub fn water_percent(&self, sample_mass_g: f64) -> f64 {
        self.water_ppm(sample_mass_g) / 10000.0
    }

    /// Get current charge.
    pub fn charge(&self) -> f64 {
        self.charge_c
    }

    /// Get elapsed time.
    pub fn elapsed(&self) -> f64 {
        self.elapsed_s
    }

    /// Check if the sensor electrode potential indicates endpoint reached.
    pub fn is_endpoint_reached(&self, sensor_potential_mv: f64) -> bool {
        sensor_potential_mv <= self.sensor_threshold_mv
    }
}

// ---------------------------------------------------------------------------
// AcidBaseCoulometric
// ---------------------------------------------------------------------------

/// Acid-base coulometric titration.
///
/// At a Pt electrode in aqueous solution:
/// - Cathode: 2H2O + 2e- → H2↑ + 2OH-   (generates base)
/// - Anode:   H2O → 2H+ + ½O2↑ + 2e-     (generates acid)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AcidBaseMode {
    /// Generate OH- at cathode for acid titration.
    GenerateBase,
    /// Generate H+ at anode for base titration.
    GenerateAcid,
}

/// Acid-base coulometric titration processor.
#[derive(Debug, Clone)]
pub struct AcidBaseCoulometric {
    /// Operating mode.
    pub mode: AcidBaseMode,
    /// Generating current in amperes.
    pub current_a: f64,
    /// Accumulated charge in coulombs.
    charge_c: f64,
    /// Elapsed time in seconds.
    elapsed_s: f64,
}

impl AcidBaseCoulometric {
    /// Create a new acid-base coulometric processor.
    pub fn new(mode: AcidBaseMode, current_a: f64) -> Self {
        Self {
            mode,
            current_a,
            charge_c: 0.0,
            elapsed_s: 0.0,
        }
    }

    /// Advance by dt seconds.
    pub fn advance(&mut self, dt_s: f64) {
        self.charge_c += self.current_a * dt_s;
        self.elapsed_s += dt_s;
    }

    /// Moles of OH- or H+ generated.
    ///
    /// Both reactions involve 1 electron per ion:
    ///   2e- → 2OH-  or  → 2H+ + 2e-
    /// So n=1 per mole of ion.
    pub fn moles_generated(&self) -> f64 {
        self.charge_c / FARADAY_CONSTANT
    }

    /// Milliequivalents of titrant generated.
    pub fn milliequivalents(&self) -> f64 {
        self.moles_generated() * 1000.0
    }

    /// Mass of analyte titrated (grams), given analyte molar mass and n_eq.
    pub fn analyte_mass_g(&self, analyte_molar_mass: f64, n_equivalents: u32) -> f64 {
        let moles_titrant: f64 = self.moles_generated();
        let moles_analyte: f64 = moles_titrant / n_equivalents as f64;
        moles_analyte * analyte_molar_mass
    }

    /// Get charge.
    pub fn charge(&self) -> f64 {
        self.charge_c
    }

    /// Get elapsed time.
    pub fn elapsed(&self) -> f64 {
        self.elapsed_s
    }

    /// Reset for a new titration.
    pub fn reset(&mut self) {
        self.charge_c = 0.0;
        self.elapsed_s = 0.0;
    }

    /// Compute pH change from strong acid or base generation.
    ///
    /// Given volume in liters and initial concentration.
    pub fn delta_ph(&self, volume_l: f64, initial_concentration_m: f64) -> f64 {
        if volume_l <= 0.0 {
            return 0.0;
        }
        let moles_gen: f64 = self.moles_generated();
        let delta_conc: f64 = moles_gen / volume_l;
        match self.mode {
            AcidBaseMode::GenerateBase => {
                // Adding OH-: pH increases
                let new_oh: f64 = initial_concentration_m + delta_conc;
                let poh: f64 = -new_oh.log10();
                14.0 - poh
            }
            AcidBaseMode::GenerateAcid => {
                // Adding H+: pH decreases
                let new_h: f64 = initial_concentration_m + delta_conc;
                -new_h.log10()
            }
        }
    }
}

// ---------------------------------------------------------------------------
// RedoxCoulometric
// ---------------------------------------------------------------------------

/// Redox system for coulometric titration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RedoxSystem {
    /// Br2 from Br-: 2Br- → Br2 + 2e- (n=2 per Br2)
    BromineBromide,
    /// Ce(IV) from Ce(III): Ce³+ → Ce⁴+ + e- (n=1)
    CeriumIVIII,
    /// I2 from I-: 2I- → I2 + 2e- (n=2 per I2)
    IodineIodide,
    /// Ag+ from Ag: Ag → Ag+ + e- (n=1)
    SilverSilverIon,
    /// Custom system with specified n_electrons.
    Custom { n_electrons: u32 },
}

impl RedoxSystem {
    /// Electrons per mole of titrant generated.
    pub fn n_electrons(&self) -> u32 {
        match self {
            RedoxSystem::BromineBromide => 2,
            RedoxSystem::CeriumIVIII => 1,
            RedoxSystem::IodineIodide => 2,
            RedoxSystem::SilverSilverIon => 1,
            RedoxSystem::Custom { n_electrons } => *n_electrons,
        }
    }

    /// Molar mass of the electrogenerated titrant (g/mol).
    pub fn titrant_molar_mass(&self) -> f64 {
        match self {
            RedoxSystem::BromineBromide => 159.808,   // Br2
            RedoxSystem::CeriumIVIII => 140.116,      // Ce
            RedoxSystem::IodineIodide => 253.808,     // I2
            RedoxSystem::SilverSilverIon => 107.868,  // Ag+
            RedoxSystem::Custom { .. } => 1.0,        // user must interpret
        }
    }
}

/// Redox coulometric titration processor.
#[derive(Debug, Clone)]
pub struct RedoxCoulometric {
    /// Redox system in use.
    pub system: RedoxSystem,
    /// Generating current in amperes.
    pub current_a: f64,
    /// Accumulated charge in coulombs.
    charge_c: f64,
    /// Elapsed time in seconds.
    elapsed_s: f64,
}

impl RedoxCoulometric {
    /// Create a new redox coulometric processor.
    pub fn new(system: RedoxSystem, current_a: f64) -> Self {
        Self {
            system,
            current_a,
            charge_c: 0.0,
            elapsed_s: 0.0,
        }
    }

    /// Advance by dt seconds.
    pub fn advance(&mut self, dt_s: f64) {
        self.charge_c += self.current_a * dt_s;
        self.elapsed_s += dt_s;
    }

    /// Moles of titrant generated.
    pub fn moles_titrant(&self) -> f64 {
        self.charge_c / (self.system.n_electrons() as f64 * FARADAY_CONSTANT)
    }

    /// Mass of titrant generated in grams.
    pub fn mass_titrant_g(&self) -> f64 {
        self.moles_titrant() * self.system.titrant_molar_mass()
    }

    /// Mass of analyte reacted (grams).
    ///
    /// * `analyte_molar_mass` - molar mass of analyte (g/mol)
    /// * `stoichiometric_ratio` - moles analyte per mole titrant
    pub fn analyte_mass_g(&self, analyte_molar_mass: f64, stoichiometric_ratio: f64) -> f64 {
        let moles_titrant: f64 = self.moles_titrant();
        let moles_analyte: f64 = moles_titrant * stoichiometric_ratio;
        moles_analyte * analyte_molar_mass
    }

    /// Get charge.
    pub fn charge(&self) -> f64 {
        self.charge_c
    }

    /// Get elapsed time.
    pub fn elapsed(&self) -> f64 {
        self.elapsed_s
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.charge_c = 0.0;
        self.elapsed_s = 0.0;
    }

    /// Current efficiency (actual moles vs theoretical).
    pub fn current_efficiency(&self, actual_moles: f64) -> f64 {
        let theoretical: f64 = self.moles_titrant();
        if theoretical.abs() < 1e-30 {
            return 0.0;
        }
        actual_moles / theoretical
    }
}

// ---------------------------------------------------------------------------
// ChargeIntegrator
// ---------------------------------------------------------------------------

/// Integration method for charge computation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntegrationMethod {
    /// Trapezoidal rule.
    Trapezoidal,
    /// Simpson's 1/3 rule (requires odd number of intervals).
    Simpson,
    /// Rectangular (left endpoint).
    Rectangular,
}

/// High-precision charge integrator for coulometric analysis.
///
/// Integrates current i(t) over time to produce charge Q = integral i(t) dt.
#[derive(Debug, Clone)]
pub struct ChargeIntegrator {
    /// Integration method.
    pub method: IntegrationMethod,
    /// Time data points in seconds.
    time_s: Vec<f64>,
    /// Current data points in amperes.
    current_a: Vec<f64>,
    /// Running charge sum (for streaming mode).
    running_charge_c: f64,
    /// Last time and current for streaming integration.
    last_time_s: Option<f64>,
    last_current_a: Option<f64>,
}

impl ChargeIntegrator {
    /// Create a new charge integrator.
    pub fn new(method: IntegrationMethod) -> Self {
        Self {
            method,
            time_s: Vec::new(),
            current_a: Vec::new(),
            running_charge_c: 0.0,
            last_time_s: None,
            last_current_a: None,
        }
    }

    /// Create with trapezoidal method (most common).
    pub fn trapezoidal() -> Self {
        Self::new(IntegrationMethod::Trapezoidal)
    }

    /// Add a data point in streaming mode.
    pub fn push(&mut self, time_s: f64, current_a: f64) {
        self.time_s.push(time_s);
        self.current_a.push(current_a);
        if let (Some(t0), Some(i0)) = (self.last_time_s, self.last_current_a) {
            let dt: f64 = time_s - t0;
            match self.method {
                IntegrationMethod::Trapezoidal => {
                    self.running_charge_c += 0.5 * (i0 + current_a) * dt;
                }
                IntegrationMethod::Rectangular => {
                    self.running_charge_c += i0 * dt;
                }
                IntegrationMethod::Simpson => {
                    // Fall back to trapezoidal in streaming mode
                    self.running_charge_c += 0.5 * (i0 + current_a) * dt;
                }
            }
        }
        self.last_time_s = Some(time_s);
        self.last_current_a = Some(current_a);
    }

    /// Get running charge in streaming mode.
    pub fn running_charge(&self) -> f64 {
        self.running_charge_c
    }

    /// Integrate from batch data.
    pub fn integrate_batch(time_s: &[f64], current_a: &[f64], method: IntegrationMethod) -> f64 {
        assert_eq!(time_s.len(), current_a.len(), "arrays must match");
        if time_s.len() < 2 {
            return 0.0;
        }
        match method {
            IntegrationMethod::Trapezoidal => trapezoidal_integrate(time_s, current_a),
            IntegrationMethod::Simpson => simpson_integrate(time_s, current_a),
            IntegrationMethod::Rectangular => rectangular_integrate(time_s, current_a),
        }
    }

    /// Compute charge error estimate.
    ///
    /// For trapezoidal rule, error is O(h²). Estimated by comparing
    /// trapezoidal and Simpson results.
    pub fn charge_error_estimate(&self) -> f64 {
        if self.time_s.len() < 3 {
            return 0.0;
        }
        let trap: f64 = trapezoidal_integrate(&self.time_s, &self.current_a);
        let simp: f64 = simpson_integrate(&self.time_s, &self.current_a);
        (trap - simp).abs()
    }

    /// Get number of data points.
    pub fn len(&self) -> usize {
        self.time_s.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.time_s.is_empty()
    }

    /// Reset the integrator.
    pub fn reset(&mut self) {
        self.time_s.clear();
        self.current_a.clear();
        self.running_charge_c = 0.0;
        self.last_time_s = None;
        self.last_current_a = None;
    }

    /// Get the stored time data.
    pub fn time_data(&self) -> &[f64] {
        &self.time_s
    }

    /// Get the stored current data.
    pub fn current_data(&self) -> &[f64] {
        &self.current_a
    }

    /// Average current over the integration period.
    pub fn average_current(&self) -> f64 {
        if self.time_s.len() < 2 {
            return 0.0;
        }
        let total_time: f64 = self.time_s[self.time_s.len() - 1] - self.time_s[0];
        if total_time.abs() < 1e-30 {
            return 0.0;
        }
        self.running_charge_c / total_time
    }
}

// ---------------------------------------------------------------------------
// DriftCompensation
// ---------------------------------------------------------------------------

/// Background current / drift compensation for Karl Fischer coulometric titration.
///
/// In KF titrations, there is always a small background drift of water vapor
/// into the titration cell. This drift must be measured before titration and
/// subtracted from the result.
#[derive(Debug, Clone)]
pub struct DriftCompensation {
    /// Drift rate in micrograms per minute.
    pub drift_rate_ug_per_min: f64,
    /// Conditional drift stop threshold (ug/min).
    pub stop_threshold_ug_per_min: f64,
    /// Over-titration correction factor (fraction).
    pub over_titration_factor: f64,
    /// History of drift measurements for averaging.
    drift_history: Vec<f64>,
}

impl DriftCompensation {
    /// Create a new drift compensator.
    pub fn new(drift_rate_ug_per_min: f64) -> Self {
        Self {
            drift_rate_ug_per_min,
            stop_threshold_ug_per_min: 10.0,
            over_titration_factor: 0.0,
            drift_history: vec![drift_rate_ug_per_min],
        }
    }

    /// Set the conditional drift stop threshold.
    pub fn with_stop_threshold(mut self, threshold_ug_per_min: f64) -> Self {
        self.stop_threshold_ug_per_min = threshold_ug_per_min;
        self
    }

    /// Set the over-titration correction factor.
    pub fn with_over_titration_correction(mut self, factor: f64) -> Self {
        self.over_titration_factor = factor.clamp(0.0, 1.0);
        self
    }

    /// Record a new drift measurement.
    pub fn record_drift(&mut self, rate_ug_per_min: f64) {
        self.drift_history.push(rate_ug_per_min);
        // Use exponential weighted average
        let n: f64 = self.drift_history.len() as f64;
        let alpha: f64 = 2.0 / (n + 1.0);
        self.drift_rate_ug_per_min =
            alpha * rate_ug_per_min + (1.0 - alpha) * self.drift_rate_ug_per_min;
    }

    /// Get the current averaged drift rate.
    pub fn current_drift_rate(&self) -> f64 {
        self.drift_rate_ug_per_min
    }

    /// Compute drift correction in micrograms for a given titration time.
    pub fn drift_correction_ug(&self, elapsed_s: f64) -> f64 {
        let elapsed_min: f64 = elapsed_s / 60.0;
        self.drift_rate_ug_per_min * elapsed_min
    }

    /// Check if drift is low enough to proceed with titration.
    pub fn is_drift_stable(&self) -> bool {
        self.drift_rate_ug_per_min <= self.stop_threshold_ug_per_min
    }

    /// Compute over-titration correction in micrograms.
    ///
    /// When the endpoint is overshot, excess reagent is generated.
    pub fn over_titration_correction_ug(&self, elapsed_s: f64) -> f64 {
        self.drift_correction_ug(elapsed_s) * self.over_titration_factor
    }

    /// Total correction in micrograms.
    pub fn total_correction_ug(&self, elapsed_s: f64) -> f64 {
        self.drift_correction_ug(elapsed_s) + self.over_titration_correction_ug(elapsed_s)
    }

    /// Number of drift measurements recorded.
    pub fn measurement_count(&self) -> usize {
        self.drift_history.len()
    }

    /// Drift standard deviation from history.
    pub fn drift_std_dev(&self) -> f64 {
        if self.drift_history.len() < 2 {
            return 0.0;
        }
        let n: f64 = self.drift_history.len() as f64;
        let mean: f64 = self.drift_history.iter().sum::<f64>() / n;
        let variance: f64 =
            self.drift_history.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (n - 1.0);
        variance.sqrt()
    }
}

// ---------------------------------------------------------------------------
// CoulometricSession
// ---------------------------------------------------------------------------

/// Quality control check result.
#[derive(Debug, Clone)]
pub struct QcResult {
    /// Name of the check.
    pub name: String,
    /// Measured value.
    pub measured: f64,
    /// Expected value.
    pub expected: f64,
    /// Tolerance (relative, e.g. 0.02 for 2%).
    pub tolerance: f64,
    /// Whether the check passed.
    pub passed: bool,
}

impl QcResult {
    /// Compute the relative error.
    pub fn relative_error(&self) -> f64 {
        if self.expected.abs() < 1e-30 {
            return 0.0;
        }
        ((self.measured - self.expected) / self.expected).abs()
    }
}

/// A single sample measurement within a session.
#[derive(Debug, Clone)]
pub struct SampleMeasurement {
    /// Sample identifier.
    pub sample_id: String,
    /// Sample mass in grams.
    pub sample_mass_g: f64,
    /// Measured charge in coulombs.
    pub charge_c: f64,
    /// Computed analyte mass in milligrams.
    pub analyte_mass_mg: f64,
    /// Concentration (ppm).
    pub concentration_ppm: f64,
    /// Blank-corrected flag.
    pub blank_corrected: bool,
}

/// Coulometric titration session manager.
///
/// Manages calibration, sample sequence, blank corrections, QC checks,
/// and replicate analysis for a complete analytical session.
#[derive(Debug, Clone)]
pub struct CoulometricSession {
    /// Faraday processor for mass-charge conversion.
    processor: FaradayLawProcessor,
    /// Calibration factor (measured/theoretical).
    calibration_factor: f64,
    /// Blank value in milligrams.
    blank_mg: f64,
    /// Sample measurements.
    measurements: Vec<SampleMeasurement>,
    /// QC results.
    qc_results: Vec<QcResult>,
    /// Is the session calibrated?
    calibrated: bool,
}

impl CoulometricSession {
    /// Create a new session.
    pub fn new(molar_mass: f64, n_electrons: u32) -> Self {
        Self {
            processor: FaradayLawProcessor::new(molar_mass, n_electrons),
            calibration_factor: 1.0,
            blank_mg: 0.0,
            measurements: Vec::new(),
            qc_results: Vec::new(),
            calibrated: false,
        }
    }

    /// Create a Karl Fischer water determination session.
    pub fn karl_fischer() -> Self {
        Self::new(WATER_MOLAR_MASS, WATER_N_ELECTRONS)
    }

    /// Perform calibration with a known standard.
    ///
    /// * `standard_mass_mg` - known mass of standard in milligrams
    /// * `measured_charge_c` - charge consumed in coulombs
    ///
    /// Returns calibration factor.
    pub fn calibrate(&mut self, standard_mass_mg: f64, measured_charge_c: f64) -> f64 {
        let theoretical_charge: f64 =
            mass_to_charge(standard_mass_mg / 1000.0, self.processor.molar_mass, self.processor.n_electrons);
        self.calibration_factor = if theoretical_charge.abs() > 1e-30 {
            measured_charge_c / theoretical_charge
        } else {
            1.0
        };
        self.calibrated = true;
        self.calibration_factor
    }

    /// Perform a blank determination.
    ///
    /// Run the same procedure without sample to determine blank value.
    pub fn run_blank(&mut self, blank_charge_c: f64) {
        let mass_g: f64 = self.processor.charge_to_mass(blank_charge_c / self.calibration_factor);
        self.blank_mg = mass_g * 1000.0;
    }

    /// Process a sample measurement.
    pub fn measure_sample(
        &mut self,
        sample_id: &str,
        sample_mass_g: f64,
        charge_c: f64,
    ) -> SampleMeasurement {
        let corrected_charge: f64 = charge_c / self.calibration_factor;
        let mass_g: f64 = self.processor.charge_to_mass(corrected_charge);
        let raw_mg: f64 = mass_g * 1000.0;
        let corrected_mg: f64 = if raw_mg > self.blank_mg {
            raw_mg - self.blank_mg
        } else {
            0.0
        };
        let conc_ppm: f64 = if sample_mass_g > 0.0 {
            (corrected_mg / sample_mass_g) * 1000.0
        } else {
            0.0
        };
        let measurement = SampleMeasurement {
            sample_id: sample_id.to_string(),
            sample_mass_g,
            charge_c,
            analyte_mass_mg: corrected_mg,
            concentration_ppm: conc_ppm,
            blank_corrected: self.blank_mg > 0.0,
        };
        self.measurements.push(measurement.clone());
        measurement
    }

    /// Run a QC check against a known standard.
    pub fn qc_check(
        &mut self,
        name: &str,
        measured_value: f64,
        expected_value: f64,
        tolerance: f64,
    ) -> QcResult {
        let rel_err: f64 = if expected_value.abs() > 1e-30 {
            ((measured_value - expected_value) / expected_value).abs()
        } else {
            0.0
        };
        let result = QcResult {
            name: name.to_string(),
            measured: measured_value,
            expected: expected_value,
            tolerance,
            passed: rel_err <= tolerance,
        };
        self.qc_results.push(result.clone());
        result
    }

    /// Get all measurements.
    pub fn measurements(&self) -> &[SampleMeasurement] {
        &self.measurements
    }

    /// Get all QC results.
    pub fn qc_results(&self) -> &[QcResult] {
        &self.qc_results
    }

    /// Whether all QC checks passed.
    pub fn all_qc_passed(&self) -> bool {
        self.qc_results.iter().all(|r| r.passed)
    }

    /// Whether the session is calibrated.
    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Get the calibration factor.
    pub fn calibration_factor(&self) -> f64 {
        self.calibration_factor
    }

    /// Get the blank value in milligrams.
    pub fn blank_mg(&self) -> f64 {
        self.blank_mg
    }

    /// Compute statistics for replicate measurements of the same sample.
    pub fn replicate_stats(&self, sample_id: &str) -> Option<ReplicateStats> {
        let values: Vec<f64> = self
            .measurements
            .iter()
            .filter(|m| m.sample_id == sample_id)
            .map(|m| m.analyte_mass_mg)
            .collect();
        if values.is_empty() {
            return None;
        }
        let n: f64 = values.len() as f64;
        let mean: f64 = values.iter().sum::<f64>() / n;
        let std_dev: f64 = if values.len() > 1 {
            let var: f64 = values.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (n - 1.0);
            var.sqrt()
        } else {
            0.0
        };
        let rsd: f64 = if mean.abs() > 1e-30 {
            (std_dev / mean) * 100.0
        } else {
            0.0
        };
        Some(ReplicateStats {
            count: values.len(),
            mean,
            std_dev,
            rsd_percent: rsd,
            min: values.iter().cloned().fold(f64::INFINITY, f64::min),
            max: values.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
        })
    }

    /// Number of samples measured.
    pub fn sample_count(&self) -> usize {
        self.measurements.len()
    }
}

/// Replicate measurement statistics.
#[derive(Debug, Clone)]
pub struct ReplicateStats {
    /// Number of replicates.
    pub count: usize,
    /// Mean value in mg.
    pub mean: f64,
    /// Standard deviation in mg.
    pub std_dev: f64,
    /// Relative standard deviation in percent.
    pub rsd_percent: f64,
    /// Minimum value.
    pub min: f64,
    /// Maximum value.
    pub max: f64,
}

// ---------------------------------------------------------------------------
// Integration helpers
// ---------------------------------------------------------------------------

/// Trapezoidal integration of y(x).
fn trapezoidal_integrate(x: &[f64], y: &[f64]) -> f64 {
    let mut sum: f64 = 0.0;
    for i in 1..x.len() {
        let dx: f64 = x[i] - x[i - 1];
        sum += 0.5 * (y[i - 1] + y[i]) * dx;
    }
    sum
}

/// Simpson's 1/3 rule integration.
///
/// Falls back to trapezoidal for the last interval if the number
/// of intervals is odd.
fn simpson_integrate(x: &[f64], y: &[f64]) -> f64 {
    let n: usize = x.len();
    if n < 3 {
        return trapezoidal_integrate(x, y);
    }
    let mut sum: f64 = 0.0;
    let mut i: usize = 0;
    // Apply Simpson's rule to pairs of intervals
    while i + 2 < n {
        let h1: f64 = x[i + 1] - x[i];
        let h2: f64 = x[i + 2] - x[i + 1];
        let h: f64 = (h1 + h2) / 2.0;
        sum += h / 3.0 * (y[i] + 4.0 * y[i + 1] + y[i + 2]);
        i += 2;
    }
    // Handle remaining interval with trapezoidal rule
    if i + 1 < n {
        let dx: f64 = x[i + 1] - x[i];
        sum += 0.5 * (y[i] + y[i + 1]) * dx;
    }
    sum
}

/// Rectangular (left endpoint) integration.
fn rectangular_integrate(x: &[f64], y: &[f64]) -> f64 {
    let mut sum: f64 = 0.0;
    for i in 1..x.len() {
        let dx: f64 = x[i] - x[i - 1];
        sum += y[i - 1] * dx;
    }
    sum
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_charge_to_mass_basic() {
        // 1 mole of Cu²+ (n=2, M=63.546): Q = 2*F
        let charge: f64 = 2.0 * FARADAY_CONSTANT;
        let mass: f64 = charge_to_mass(charge, 63.546, 2);
        assert!((mass - 63.546).abs() < 0.01, "mass = {}", mass);
    }

    #[test]
    fn test_mass_to_charge_basic() {
        // 63.546 g Cu²+ should need 2F coulombs
        let charge: f64 = mass_to_charge(63.546, 63.546, 2);
        let expected: f64 = 2.0 * FARADAY_CONSTANT;
        assert!((charge - expected).abs() < 1.0, "charge = {}", charge);
    }

    #[test]
    fn test_charge_mass_roundtrip() {
        let mass_original: f64 = 0.5;
        let molar_mass: f64 = 107.868;
        let n: u32 = 1;
        let charge: f64 = mass_to_charge(mass_original, molar_mass, n);
        let mass_back: f64 = charge_to_mass(charge, molar_mass, n);
        assert!((mass_back - mass_original).abs() < 1e-10);
    }

    #[test]
    fn test_water_from_charge() {
        // 1 mole of H2O = 2F coulombs = 18.015 g = 18015 mg
        let charge: f64 = 2.0 * FARADAY_CONSTANT;
        let mg: f64 = water_from_charge(charge);
        assert!((mg - 18015.0).abs() < 1.0, "mg = {}", mg);
    }

    #[test]
    fn test_water_from_charge_1mg() {
        // 1 mg H2O requires ~10.712 C
        let expected_charge: f64 = mass_to_charge(0.001, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let mg: f64 = water_from_charge(expected_charge);
        assert!((mg - 1.0).abs() < 0.01, "mg = {}", mg);
    }

    #[test]
    fn test_charge_to_mass_zero() {
        assert_eq!(charge_to_mass(0.0, 100.0, 1), 0.0);
    }

    #[test]
    fn test_mass_to_charge_zero() {
        assert_eq!(mass_to_charge(0.0, 100.0, 1), 0.0);
    }

    #[test]
    fn test_water_from_charge_zero() {
        assert_eq!(water_from_charge(0.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // FaradayLawProcessor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_faraday_processor_basic() {
        let proc = FaradayLawProcessor::new(63.546, 2);
        let charge: f64 = 2.0 * FARADAY_CONSTANT;
        let mass: f64 = proc.charge_to_mass(charge);
        assert!((mass - 63.546).abs() < 0.01);
    }

    #[test]
    fn test_faraday_processor_with_efficiency() {
        let proc = FaradayLawProcessor::new(63.546, 2).with_efficiency(0.95);
        let charge: f64 = 2.0 * FARADAY_CONSTANT;
        let mass: f64 = proc.charge_to_mass(charge);
        // Should be 95% of theoretical
        assert!((mass - 63.546 * 0.95).abs() < 0.01);
    }

    #[test]
    fn test_faraday_processor_mass_to_charge() {
        let proc = FaradayLawProcessor::new(63.546, 2);
        let charge: f64 = proc.mass_to_charge(63.546);
        let expected: f64 = 2.0 * FARADAY_CONSTANT;
        assert!((charge - expected).abs() < 1.0);
    }

    #[test]
    fn test_faraday_processor_mass_to_charge_with_efficiency() {
        let proc = FaradayLawProcessor::new(63.546, 2).with_efficiency(0.95);
        let charge: f64 = proc.mass_to_charge(63.546);
        let expected: f64 = 2.0 * FARADAY_CONSTANT / 0.95;
        assert!((charge - expected).abs() < 10.0);
    }

    #[test]
    fn test_faraday_integrate_current_constant() {
        let proc = FaradayLawProcessor::new(63.546, 2);
        // Constant 1A for 100s = 100 C
        let time: Vec<f64> = (0..=100).map(|t| t as f64).collect();
        let current: Vec<f64> = vec![1.0; 101];
        let (charge, _mass) = proc.integrate_current(&time, &current);
        assert!((charge - 100.0).abs() < 0.01, "charge = {}", charge);
    }

    #[test]
    fn test_faraday_integrate_current_linear() {
        let proc = FaradayLawProcessor::new(100.0, 1);
        // Linear ramp 0 to 2A over 10s: integral = 10.0 C
        let time: Vec<f64> = (0..=100).map(|t| t as f64 * 0.1).collect();
        let current: Vec<f64> = (0..=100).map(|t| t as f64 * 0.02).collect();
        let (charge, _mass) = proc.integrate_current(&time, &current);
        assert!((charge - 10.0).abs() < 0.01, "charge = {}", charge);
    }

    #[test]
    fn test_faraday_required_charge() {
        let proc = FaradayLawProcessor::new(WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let required: f64 = proc.required_charge(WATER_MOLAR_MASS);
        let expected: f64 = 2.0 * FARADAY_CONSTANT;
        assert!((required - expected).abs() < 1.0);
    }

    #[test]
    fn test_faraday_experimental_faraday() {
        let proc = FaradayLawProcessor::new(63.546, 2);
        let charge: f64 = 2.0 * FARADAY_CONSTANT;
        let f_exp: f64 = proc.experimental_faraday(charge, 63.546);
        assert!((f_exp - FARADAY_CONSTANT).abs() < 1.0);
    }

    #[test]
    fn test_faraday_experimental_faraday_zero_mass() {
        let proc = FaradayLawProcessor::new(63.546, 2);
        assert_eq!(proc.experimental_faraday(100.0, 0.0), 0.0);
    }

    #[test]
    fn test_faraday_efficiency_clamp() {
        let proc = FaradayLawProcessor::new(100.0, 1).with_efficiency(1.5);
        assert_eq!(proc.current_efficiency, 1.0);
        let proc2 = FaradayLawProcessor::new(100.0, 1).with_efficiency(-0.5);
        assert_eq!(proc2.current_efficiency, 0.0);
    }

    // -----------------------------------------------------------------------
    // ConstantCurrentSource tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ccs_basic() {
        let mut src = ConstantCurrentSource::new(0.050, 0.001, 20.0);
        src.advance(100.0);
        let expected_charge: f64 = 0.050 * 100.0;
        assert!((src.charge() - expected_charge).abs() < 1e-10);
    }

    #[test]
    fn test_ccs_charge_at_time() {
        let src = ConstantCurrentSource::new(0.100, 0.001, 20.0);
        let q: f64 = src.charge_at_time(200.0);
        assert!((q - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_ccs_karl_fischer_preset() {
        let src = ConstantCurrentSource::karl_fischer_preset();
        assert!((src.set_current_a - 0.0004).abs() < 1e-10);
        assert!((src.precision - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_ccs_general_preset() {
        let src = ConstantCurrentSource::general_preset();
        assert!((src.set_current_a - 0.050).abs() < 1e-10);
    }

    #[test]
    fn test_ccs_actual_current() {
        let src = ConstantCurrentSource::new(1.0, 0.01, 10.0);
        assert!((src.actual_current() - 1.01).abs() < 1e-10);
        assert!((src.min_current() - 0.99).abs() < 1e-10);
    }

    #[test]
    fn test_ccs_charge_uncertainty() {
        let mut src = ConstantCurrentSource::new(1.0, 0.01, 10.0);
        src.advance(100.0);
        assert!((src.charge_uncertainty() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ccs_reset() {
        let mut src = ConstantCurrentSource::new(1.0, 0.01, 10.0);
        src.advance(100.0);
        src.reset();
        assert_eq!(src.charge(), 0.0);
        assert_eq!(src.elapsed(), 0.0);
    }

    #[test]
    fn test_ccs_compliance_limited() {
        let src = ConstantCurrentSource::new(1.0, 0.01, 10.0);
        assert!(!src.is_compliance_limited(9.0));
        assert!(src.is_compliance_limited(10.0));
        assert!(src.is_compliance_limited(15.0));
    }

    #[test]
    fn test_ccs_elapsed() {
        let mut src = ConstantCurrentSource::new(0.1, 0.001, 10.0);
        src.advance(5.0);
        src.advance(3.0);
        assert!((src.elapsed() - 8.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // EndpointDetection tests
    // -----------------------------------------------------------------------

    fn make_sigmoid_data(n: usize, midpoint: f64) -> (Vec<f64>, Vec<f64>) {
        let charge: Vec<f64> = (0..n).map(|i| i as f64 * 0.1).collect();
        let potential: Vec<f64> = charge
            .iter()
            .map(|&q| {
                let x: f64 = (q - midpoint) * 20.0;
                0.2 + 0.6 / (1.0 + (-x).exp())
            })
            .collect();
        (charge, potential)
    }

    #[test]
    fn test_endpoint_first_derivative() {
        let (charge, potential) = make_sigmoid_data(100, 5.0);
        let ed = EndpointDetection::new(charge.clone(), potential.clone());
        let result = ed.first_derivative_endpoint();
        assert!(result.is_some());
        let r = result.unwrap();
        // Endpoint should be near charge = 5.0
        assert!((r.charge_c - 5.0).abs() < 0.5, "charge = {}", r.charge_c);
        assert!(r.confidence > 0.0);
    }

    #[test]
    fn test_endpoint_second_derivative() {
        // Use more points and a wider range to avoid spurious zero-crossings in flat regions
        let (charge, potential) = make_sigmoid_data(200, 5.0);
        let ed = EndpointDetection::new(charge, potential);
        let result = ed.second_derivative_endpoint();
        assert!(result.is_some());
        let r = result.unwrap();
        assert!((r.charge_c - 5.0).abs() < 2.0, "charge = {}", r.charge_c);
    }

    #[test]
    fn test_endpoint_bipotentiometric() {
        let charge: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let potential: Vec<f64> = charge.iter().map(|&q| {
            if q < 25.0 { 0.5 - q * 0.02 } else { 0.0 }
        }).collect();
        let ed = EndpointDetection::new(charge, potential);
        let result = ed.bipotentiometric_endpoint(0.01);
        assert!(result.is_some());
    }

    #[test]
    fn test_endpoint_threshold() {
        let charge: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let potential: Vec<f64> = vec![0.1, 0.2, 0.3, 0.5, 0.7, 0.9];
        let ed = EndpointDetection::new(charge, potential);
        let result = ed.threshold_endpoint(0.4);
        assert!(result.is_some());
        let r = result.unwrap();
        assert!((r.charge_c - 2.5).abs() < 0.5);
    }

    #[test]
    fn test_endpoint_threshold_not_found() {
        let charge: Vec<f64> = vec![0.0, 1.0, 2.0];
        let potential: Vec<f64> = vec![0.1, 0.2, 0.3];
        let ed = EndpointDetection::new(charge, potential);
        assert!(ed.threshold_endpoint(0.9).is_none());
    }

    #[test]
    fn test_endpoint_first_derivative_too_few() {
        let ed = EndpointDetection::new(vec![0.0, 1.0], vec![0.1, 0.2]);
        assert!(ed.first_derivative_endpoint().is_none());
    }

    #[test]
    fn test_endpoint_second_derivative_too_few() {
        let ed = EndpointDetection::new(vec![0.0, 1.0, 2.0, 3.0], vec![0.1, 0.2, 0.3, 0.4]);
        assert!(ed.second_derivative_endpoint().is_none());
    }

    #[test]
    fn test_endpoint_push() {
        let mut ed = EndpointDetection::new(vec![], vec![]);
        for i in 0..10 {
            ed.push(i as f64, i as f64 * 0.1);
        }
        assert_eq!(ed.charge_c.len(), 10);
    }

    #[test]
    fn test_endpoint_first_derivative_values() {
        let charge: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let potential: Vec<f64> = vec![0.0, 1.0, 4.0, 9.0, 16.0]; // quadratic
        let ed = EndpointDetection::new(charge, potential);
        let derivs = ed.first_derivative();
        assert_eq!(derivs.len(), 3);
        // Central difference of quadratic: dE/dQ at q=1 = (4-0)/2 = 2.0
        assert!((derivs[0] - 2.0).abs() < 1e-10);
        // dE/dQ at q=2 = (9-1)/2 = 4.0
        assert!((derivs[1] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_endpoint_detect_method() {
        let (charge, potential) = make_sigmoid_data(100, 5.0);
        let ed = EndpointDetection::new(charge, potential);
        let r1 = ed.detect(EndpointMethod::FirstDerivative);
        assert!(r1.is_some());
        let r2 = ed.detect(EndpointMethod::SecondDerivative);
        assert!(r2.is_some());
    }

    #[test]
    fn test_endpoint_detect_threshold_method() {
        let charge: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0];
        let potential: Vec<f64> = vec![0.0, 0.3, 0.6, 0.9];
        let ed = EndpointDetection::new(charge, potential);
        let r = ed.detect(EndpointMethod::PotentialThreshold(0.5));
        assert!(r.is_some());
    }

    // -----------------------------------------------------------------------
    // KarlFischerCoulometric tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_kf_standard_cell() {
        let kf = KarlFischerCoulometric::standard_cell();
        assert!((kf.generator_current_a - 0.0004).abs() < 1e-10);
        assert!((kf.sensor_threshold_mv - 30.0).abs() < 1e-10);
    }

    #[test]
    fn test_kf_titration_1mg_water() {
        let mut kf = KarlFischerCoulometric::standard_cell();
        kf.start();
        // 1 mg H2O ≈ 10.712 C at 400 uA → t = 10.712/0.0004 = 26780 s
        let target_charge: f64 = mass_to_charge(0.001, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let time_s: f64 = target_charge / 0.0004;
        kf.advance(time_s);
        kf.stop();
        let mg: f64 = kf.raw_water_mg();
        assert!((mg - 1.0).abs() < 0.01, "mg = {}", mg);
    }

    #[test]
    fn test_kf_net_water_with_drift() {
        let mut kf = KarlFischerCoulometric::standard_cell();
        kf.set_drift_rate(0.1); // 0.1 ug/min (low drift)
        kf.start();
        // Titrate for 60 seconds
        kf.advance(60.0);
        kf.stop();
        let raw: f64 = kf.raw_water_mg();
        let net: f64 = kf.net_water_mg();
        // drift = 0.1 ug/min * 1 min = 0.1 ug = 0.0001 mg
        let expected_drift_mg: f64 = 0.1 * (60.0 / 60.0) / 1000.0;
        assert!(net < raw);
        assert!((raw - net - expected_drift_mg).abs() < 0.0001, "diff = {}", raw - net);
    }

    #[test]
    fn test_kf_water_ppm() {
        let mut kf = KarlFischerCoulometric::standard_cell();
        kf.start();
        // Generate charge for 10 mg water
        let charge_for_10mg: f64 = mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let time_s: f64 = charge_for_10mg / kf.generator_current_a;
        kf.advance(time_s);
        kf.stop();
        // 10 mg in 1 g sample = 10000 ppm
        let ppm: f64 = kf.water_ppm(1.0);
        assert!((ppm - 10000.0).abs() < 100.0, "ppm = {}", ppm);
    }

    #[test]
    fn test_kf_water_percent() {
        let mut kf = KarlFischerCoulometric::standard_cell();
        kf.start();
        let charge_for_10mg: f64 = mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let time_s: f64 = charge_for_10mg / kf.generator_current_a;
        kf.advance(time_s);
        kf.stop();
        let pct: f64 = kf.water_percent(1.0);
        assert!((pct - 1.0).abs() < 0.01, "pct = {}", pct);
    }

    #[test]
    fn test_kf_endpoint_reached() {
        let kf = KarlFischerCoulometric::standard_cell();
        assert!(!kf.is_endpoint_reached(100.0));
        assert!(kf.is_endpoint_reached(30.0));
        assert!(kf.is_endpoint_reached(20.0));
    }

    #[test]
    fn test_kf_zero_sample_mass() {
        let kf = KarlFischerCoulometric::standard_cell();
        assert_eq!(kf.water_ppm(0.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // AcidBaseCoulometric tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_acid_base_generate_base() {
        let mut ab = AcidBaseCoulometric::new(AcidBaseMode::GenerateBase, 0.050);
        ab.advance(FARADAY_CONSTANT / 0.050); // 1 mole of OH-
        let moles: f64 = ab.moles_generated();
        assert!((moles - 1.0).abs() < 0.001, "moles = {}", moles);
    }

    #[test]
    fn test_acid_base_milliequivalents() {
        let mut ab = AcidBaseCoulometric::new(AcidBaseMode::GenerateAcid, 0.100);
        ab.advance(FARADAY_CONSTANT / 0.100); // 1 mole
        let meq: f64 = ab.milliequivalents();
        assert!((meq - 1000.0).abs() < 1.0, "meq = {}", meq);
    }

    #[test]
    fn test_acid_base_analyte_mass() {
        // Titrate HCl (M=36.461, monoprotic n_eq=1) with generated OH-
        let mut ab = AcidBaseCoulometric::new(AcidBaseMode::GenerateBase, 0.050);
        let target_moles: f64 = 0.001; // 1 mmol HCl
        let time_s: f64 = target_moles * FARADAY_CONSTANT / 0.050;
        ab.advance(time_s);
        let mass: f64 = ab.analyte_mass_g(36.461, 1);
        let expected: f64 = 0.001 * 36.461;
        assert!((mass - expected).abs() < 0.001, "mass = {}", mass);
    }

    #[test]
    fn test_acid_base_reset() {
        let mut ab = AcidBaseCoulometric::new(AcidBaseMode::GenerateBase, 0.050);
        ab.advance(100.0);
        ab.reset();
        assert_eq!(ab.charge(), 0.0);
        assert_eq!(ab.elapsed(), 0.0);
    }

    #[test]
    fn test_acid_base_delta_ph_base() {
        let mut ab = AcidBaseCoulometric::new(AcidBaseMode::GenerateBase, 0.010);
        // Generate enough OH- for 0.01 M in 0.1 L
        let moles_needed: f64 = 0.01 * 0.1; // 0.001 mol
        let time_s: f64 = moles_needed * FARADAY_CONSTANT / 0.010;
        ab.advance(time_s);
        let ph: f64 = ab.delta_ph(0.1, 0.001);
        // OH- concentration = 0.001 + 0.01 = 0.011 M
        // pOH = -log10(0.011) ≈ 1.959, pH = 12.04
        assert!(ph > 11.0 && ph < 13.0, "pH = {}", ph);
    }

    #[test]
    fn test_acid_base_delta_ph_acid() {
        let mut ab = AcidBaseCoulometric::new(AcidBaseMode::GenerateAcid, 0.010);
        let moles_needed: f64 = 0.001;
        let time_s: f64 = moles_needed * FARADAY_CONSTANT / 0.010;
        ab.advance(time_s);
        let ph: f64 = ab.delta_ph(0.1, 0.001);
        // H+ concentration = 0.001 + 0.01 = 0.011 M, pH ≈ 1.96
        assert!(ph > 1.0 && ph < 3.0, "pH = {}", ph);
    }

    #[test]
    fn test_acid_base_delta_ph_zero_volume() {
        let ab = AcidBaseCoulometric::new(AcidBaseMode::GenerateBase, 0.010);
        assert_eq!(ab.delta_ph(0.0, 0.001), 0.0);
    }

    // -----------------------------------------------------------------------
    // RedoxCoulometric tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_redox_bromine_generation() {
        let mut rx = RedoxCoulometric::new(RedoxSystem::BromineBromide, 0.050);
        // Generate 1 mmol Br2: n=2, so Q = 0.001 * 2 * F
        let time_s: f64 = 0.001 * 2.0 * FARADAY_CONSTANT / 0.050;
        rx.advance(time_s);
        let moles: f64 = rx.moles_titrant();
        assert!((moles - 0.001).abs() < 1e-5, "moles = {}", moles);
    }

    #[test]
    fn test_redox_cerium_generation() {
        let mut rx = RedoxCoulometric::new(RedoxSystem::CeriumIVIII, 0.100);
        // Generate 1 mmol Ce(IV): n=1, so Q = 0.001 * F
        let time_s: f64 = 0.001 * FARADAY_CONSTANT / 0.100;
        rx.advance(time_s);
        let moles: f64 = rx.moles_titrant();
        assert!((moles - 0.001).abs() < 1e-5, "moles = {}", moles);
    }

    #[test]
    fn test_redox_mass_titrant() {
        let mut rx = RedoxCoulometric::new(RedoxSystem::IodineIodide, 0.050);
        let time_s: f64 = 0.001 * 2.0 * FARADAY_CONSTANT / 0.050;
        rx.advance(time_s);
        let mass: f64 = rx.mass_titrant_g();
        let expected: f64 = 0.001 * 253.808;
        assert!((mass - expected).abs() < 0.01, "mass = {}", mass);
    }

    #[test]
    fn test_redox_analyte_mass() {
        // Fe²+ titration with Ce⁴+: Fe²+ + Ce⁴+ → Fe³+ + Ce³+, 1:1 stoichiometry
        let mut rx = RedoxCoulometric::new(RedoxSystem::CeriumIVIII, 0.050);
        let time_s: f64 = 0.001 * FARADAY_CONSTANT / 0.050; // 1 mmol Ce(IV)
        rx.advance(time_s);
        let mass_fe: f64 = rx.analyte_mass_g(55.845, 1.0); // Fe, 1:1
        let expected: f64 = 0.001 * 55.845;
        assert!((mass_fe - expected).abs() < 0.001, "mass_fe = {}", mass_fe);
    }

    #[test]
    fn test_redox_current_efficiency() {
        let mut rx = RedoxCoulometric::new(RedoxSystem::BromineBromide, 0.050);
        let time_s: f64 = 0.001 * 2.0 * FARADAY_CONSTANT / 0.050;
        rx.advance(time_s);
        let eff: f64 = rx.current_efficiency(0.0009);
        assert!((eff - 0.9).abs() < 0.001);
    }

    #[test]
    fn test_redox_reset() {
        let mut rx = RedoxCoulometric::new(RedoxSystem::SilverSilverIon, 0.010);
        rx.advance(100.0);
        rx.reset();
        assert_eq!(rx.charge(), 0.0);
        assert_eq!(rx.elapsed(), 0.0);
    }

    #[test]
    fn test_redox_n_electrons() {
        assert_eq!(RedoxSystem::BromineBromide.n_electrons(), 2);
        assert_eq!(RedoxSystem::CeriumIVIII.n_electrons(), 1);
        assert_eq!(RedoxSystem::IodineIodide.n_electrons(), 2);
        assert_eq!(RedoxSystem::SilverSilverIon.n_electrons(), 1);
        assert_eq!(RedoxSystem::Custom { n_electrons: 3 }.n_electrons(), 3);
    }

    #[test]
    fn test_redox_titrant_molar_mass() {
        assert!((RedoxSystem::BromineBromide.titrant_molar_mass() - 159.808).abs() < 0.01);
        assert!((RedoxSystem::CeriumIVIII.titrant_molar_mass() - 140.116).abs() < 0.01);
    }

    #[test]
    fn test_redox_current_efficiency_zero_theoretical() {
        let rx = RedoxCoulometric::new(RedoxSystem::BromineBromide, 0.050);
        assert_eq!(rx.current_efficiency(1.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // ChargeIntegrator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_integrator_trapezoidal_constant() {
        let q: f64 = ChargeIntegrator::integrate_batch(
            &[0.0, 1.0, 2.0, 3.0, 4.0],
            &[1.0, 1.0, 1.0, 1.0, 1.0],
            IntegrationMethod::Trapezoidal,
        );
        assert!((q - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrator_trapezoidal_linear() {
        let q: f64 = ChargeIntegrator::integrate_batch(
            &[0.0, 1.0, 2.0, 3.0, 4.0],
            &[0.0, 1.0, 2.0, 3.0, 4.0],
            IntegrationMethod::Trapezoidal,
        );
        assert!((q - 8.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrator_simpson_quadratic() {
        // Simpson's rule is exact for quadratics
        let time: Vec<f64> = (0..=10).map(|i| i as f64 * 0.1).collect();
        let current: Vec<f64> = time.iter().map(|&t| t * t).collect();
        let q: f64 = ChargeIntegrator::integrate_batch(&time, &current, IntegrationMethod::Simpson);
        // integral of t² from 0 to 1 = 1/3
        assert!((q - 1.0 / 3.0).abs() < 0.01, "q = {}", q);
    }

    #[test]
    fn test_integrator_rectangular() {
        let q: f64 = ChargeIntegrator::integrate_batch(
            &[0.0, 1.0, 2.0, 3.0],
            &[1.0, 2.0, 3.0, 4.0],
            IntegrationMethod::Rectangular,
        );
        // Left rectangles: 1*1 + 2*1 + 3*1 = 6
        assert!((q - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrator_streaming() {
        let mut integrator = ChargeIntegrator::trapezoidal();
        integrator.push(0.0, 1.0);
        integrator.push(1.0, 1.0);
        integrator.push(2.0, 1.0);
        assert!((integrator.running_charge() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrator_streaming_matches_batch() {
        let time: Vec<f64> = (0..=20).map(|i| i as f64 * 0.5).collect();
        let current: Vec<f64> = time.iter().map(|&t| (t * 0.3).sin() + 1.0).collect();

        let batch: f64 =
            ChargeIntegrator::integrate_batch(&time, &current, IntegrationMethod::Trapezoidal);

        let mut streaming = ChargeIntegrator::trapezoidal();
        for i in 0..time.len() {
            streaming.push(time[i], current[i]);
        }
        assert!(
            (streaming.running_charge() - batch).abs() < 1e-10,
            "streaming={}, batch={}",
            streaming.running_charge(),
            batch
        );
    }

    #[test]
    fn test_integrator_error_estimate() {
        let time: Vec<f64> = (0..=10).map(|i| i as f64 * 0.1).collect();
        let current: Vec<f64> = time.iter().map(|&t| t * t * t).collect();
        let mut integrator = ChargeIntegrator::trapezoidal();
        for i in 0..time.len() {
            integrator.push(time[i], current[i]);
        }
        let err: f64 = integrator.charge_error_estimate();
        assert!(err >= 0.0);
    }

    #[test]
    fn test_integrator_len_empty() {
        let integrator = ChargeIntegrator::trapezoidal();
        assert!(integrator.is_empty());
        assert_eq!(integrator.len(), 0);
    }

    #[test]
    fn test_integrator_reset() {
        let mut integrator = ChargeIntegrator::trapezoidal();
        integrator.push(0.0, 1.0);
        integrator.push(1.0, 2.0);
        integrator.reset();
        assert!(integrator.is_empty());
        assert_eq!(integrator.running_charge(), 0.0);
    }

    #[test]
    fn test_integrator_average_current() {
        let mut integrator = ChargeIntegrator::trapezoidal();
        integrator.push(0.0, 2.0);
        integrator.push(1.0, 2.0);
        integrator.push(2.0, 2.0);
        assert!((integrator.average_current() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrator_batch_too_few() {
        let q: f64 = ChargeIntegrator::integrate_batch(&[0.0], &[1.0], IntegrationMethod::Trapezoidal);
        assert_eq!(q, 0.0);
    }

    // -----------------------------------------------------------------------
    // DriftCompensation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_drift_basic_correction() {
        let dc = DriftCompensation::new(10.0); // 10 ug/min
        let corr: f64 = dc.drift_correction_ug(120.0); // 2 minutes
        assert!((corr - 20.0).abs() < 0.01, "corr = {}", corr);
    }

    #[test]
    fn test_drift_record_and_average() {
        let mut dc = DriftCompensation::new(10.0);
        dc.record_drift(12.0);
        dc.record_drift(8.0);
        // Should average towards recent values
        let rate: f64 = dc.current_drift_rate();
        assert!(rate > 7.0 && rate < 13.0, "rate = {}", rate);
    }

    #[test]
    fn test_drift_stability() {
        let dc = DriftCompensation::new(5.0).with_stop_threshold(10.0);
        assert!(dc.is_drift_stable());
        let dc2 = DriftCompensation::new(15.0).with_stop_threshold(10.0);
        assert!(!dc2.is_drift_stable());
    }

    #[test]
    fn test_drift_over_titration() {
        let dc = DriftCompensation::new(10.0).with_over_titration_correction(0.5);
        let ot: f64 = dc.over_titration_correction_ug(60.0); // 1 min
        // drift = 10 ug, over_titration = 10 * 0.5 = 5 ug
        assert!((ot - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_drift_total_correction() {
        let dc = DriftCompensation::new(10.0).with_over_titration_correction(0.5);
        let total: f64 = dc.total_correction_ug(60.0);
        // 10 + 5 = 15
        assert!((total - 15.0).abs() < 0.01);
    }

    #[test]
    fn test_drift_std_dev() {
        let mut dc = DriftCompensation::new(10.0);
        dc.record_drift(10.0);
        dc.record_drift(10.0);
        // All same → std dev 0
        assert!(dc.drift_std_dev() < 1e-10);
    }

    #[test]
    fn test_drift_std_dev_variation() {
        let mut dc = DriftCompensation::new(5.0);
        dc.record_drift(15.0);
        dc.record_drift(10.0);
        assert!(dc.drift_std_dev() > 0.0);
    }

    #[test]
    fn test_drift_measurement_count() {
        let mut dc = DriftCompensation::new(10.0);
        assert_eq!(dc.measurement_count(), 1);
        dc.record_drift(12.0);
        assert_eq!(dc.measurement_count(), 2);
    }

    // -----------------------------------------------------------------------
    // CoulometricSession tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_session_karl_fischer() {
        let session = CoulometricSession::karl_fischer();
        assert!(!session.is_calibrated());
    }

    #[test]
    fn test_session_calibrate() {
        let mut session = CoulometricSession::karl_fischer();
        // Calibrate with 10 mg water standard
        let standard_charge: f64 = mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let factor: f64 = session.calibrate(10.0, standard_charge);
        assert!((factor - 1.0).abs() < 0.001);
        assert!(session.is_calibrated());
    }

    #[test]
    fn test_session_blank_correction() {
        let mut session = CoulometricSession::karl_fischer();
        session.calibrate(10.0, mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS));
        // Blank run with 0.1 mg water
        let blank_charge: f64 = mass_to_charge(0.0001, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        session.run_blank(blank_charge);
        assert!((session.blank_mg() - 0.1).abs() < 0.01, "blank = {}", session.blank_mg());
    }

    #[test]
    fn test_session_measure_sample() {
        let mut session = CoulometricSession::karl_fischer();
        session.calibrate(10.0, mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS));
        let charge: f64 = mass_to_charge(0.005, WATER_MOLAR_MASS, WATER_N_ELECTRONS); // 5 mg
        let m = session.measure_sample("S1", 1.0, charge);
        assert!((m.analyte_mass_mg - 5.0).abs() < 0.1, "mass = {}", m.analyte_mass_mg);
        assert!((m.concentration_ppm - 5000.0).abs() < 100.0);
    }

    #[test]
    fn test_session_qc_check_pass() {
        let mut session = CoulometricSession::karl_fischer();
        let qc = session.qc_check("Water std", 10.0, 10.0, 0.02);
        assert!(qc.passed);
    }

    #[test]
    fn test_session_qc_check_fail() {
        let mut session = CoulometricSession::karl_fischer();
        let qc = session.qc_check("Water std", 10.0, 12.0, 0.02);
        assert!(!qc.passed);
    }

    #[test]
    fn test_session_all_qc_passed() {
        let mut session = CoulometricSession::karl_fischer();
        session.qc_check("Q1", 10.0, 10.0, 0.05);
        session.qc_check("Q2", 9.9, 10.0, 0.05);
        assert!(session.all_qc_passed());
    }

    #[test]
    fn test_session_replicate_stats() {
        let mut session = CoulometricSession::karl_fischer();
        session.calibrate(10.0, mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS));
        let charge1: f64 = mass_to_charge(0.005, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let charge2: f64 = mass_to_charge(0.0052, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let charge3: f64 = mass_to_charge(0.0048, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        session.measure_sample("S1", 1.0, charge1);
        session.measure_sample("S1", 1.0, charge2);
        session.measure_sample("S1", 1.0, charge3);
        let stats = session.replicate_stats("S1");
        assert!(stats.is_some());
        let s = stats.unwrap();
        assert_eq!(s.count, 3);
        assert!(s.std_dev > 0.0);
        assert!(s.rsd_percent > 0.0 && s.rsd_percent < 10.0);
    }

    #[test]
    fn test_session_replicate_stats_not_found() {
        let session = CoulometricSession::karl_fischer();
        assert!(session.replicate_stats("nonexistent").is_none());
    }

    #[test]
    fn test_session_sample_count() {
        let mut session = CoulometricSession::karl_fischer();
        assert_eq!(session.sample_count(), 0);
        session.calibrate(10.0, mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS));
        let charge: f64 = mass_to_charge(0.005, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        session.measure_sample("S1", 1.0, charge);
        assert_eq!(session.sample_count(), 1);
    }

    #[test]
    fn test_qc_result_relative_error() {
        let qc = QcResult {
            name: "test".to_string(),
            measured: 10.5,
            expected: 10.0,
            tolerance: 0.1,
            passed: true,
        };
        assert!((qc.relative_error() - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_qc_result_relative_error_zero_expected() {
        let qc = QcResult {
            name: "test".to_string(),
            measured: 1.0,
            expected: 0.0,
            tolerance: 0.1,
            passed: false,
        };
        assert_eq!(qc.relative_error(), 0.0);
    }

    // -----------------------------------------------------------------------
    // Integration helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_trapezoidal_sine() {
        // integral of sin(x) from 0 to pi = 2.0
        let n: usize = 1000;
        let time: Vec<f64> = (0..=n).map(|i| i as f64 * std::f64::consts::PI / n as f64).collect();
        let current: Vec<f64> = time.iter().map(|&t| t.sin()).collect();
        let q: f64 = trapezoidal_integrate(&time, &current);
        assert!((q - 2.0).abs() < 0.001, "q = {}", q);
    }

    #[test]
    fn test_simpson_sine() {
        let n: usize = 100;
        let time: Vec<f64> = (0..=n).map(|i| i as f64 * std::f64::consts::PI / n as f64).collect();
        let current: Vec<f64> = time.iter().map(|&t| t.sin()).collect();
        let q: f64 = simpson_integrate(&time, &current);
        assert!((q - 2.0).abs() < 0.001, "q = {}", q);
    }

    #[test]
    fn test_rectangular_constant() {
        let q: f64 = rectangular_integrate(&[0.0, 1.0, 2.0], &[5.0, 5.0, 5.0]);
        assert!((q - 10.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Edge case and cross-component tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_karl_fischer_workflow() {
        // Complete KF analysis workflow
        let mut session = CoulometricSession::karl_fischer();

        // 1. Calibrate with 10 mg water standard
        let std_charge: f64 = mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        session.calibrate(10.0, std_charge);

        // 2. Run blank
        let blank_charge: f64 = mass_to_charge(0.00005, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        session.run_blank(blank_charge);

        // 3. Measure sample
        let sample_charge: f64 = mass_to_charge(0.002, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let m = session.measure_sample("Sample-1", 0.5, sample_charge);
        assert!(m.blank_corrected);
        assert!(m.analyte_mass_mg > 0.0);
        assert!(m.concentration_ppm > 0.0);

        // 4. QC check
        let qc = session.qc_check("Cal verify", 10.0, 10.0, 0.05);
        assert!(qc.passed);
        assert!(session.all_qc_passed());
    }

    #[test]
    fn test_redox_custom_system() {
        let sys = RedoxSystem::Custom { n_electrons: 3 };
        assert_eq!(sys.n_electrons(), 3);
        assert_eq!(sys.titrant_molar_mass(), 1.0);
    }

    #[test]
    fn test_kf_net_water_no_drift() {
        let mut kf = KarlFischerCoulometric::standard_cell();
        kf.start();
        kf.advance(100.0);
        kf.stop();
        let raw: f64 = kf.raw_water_mg();
        let net: f64 = kf.net_water_mg();
        assert!((raw - net).abs() < 1e-10);
    }

    #[test]
    fn test_ccs_multiple_advances() {
        let mut src = ConstantCurrentSource::new(0.010, 0.001, 10.0);
        for _ in 0..100 {
            src.advance(0.1);
        }
        let expected: f64 = 0.010 * 10.0;
        assert!((src.charge() - expected).abs() < 1e-8, "charge = {}", src.charge());
    }

    #[test]
    fn test_endpoint_bipotentiometric_no_crossing() {
        let charge: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0];
        let potential: Vec<f64> = vec![0.5, 0.6, 0.7, 0.8];
        let ed = EndpointDetection::new(charge, potential);
        assert!(ed.bipotentiometric_endpoint(0.01).is_none());
    }

    #[test]
    fn test_session_calibration_factor_asymmetric() {
        let mut session = CoulometricSession::karl_fischer();
        // Measured charge is 5% more than expected
        let std_charge: f64 = mass_to_charge(0.010, WATER_MOLAR_MASS, WATER_N_ELECTRONS);
        let factor: f64 = session.calibrate(10.0, std_charge * 1.05);
        assert!((factor - 1.05).abs() < 0.01);
    }

    #[test]
    fn test_integrator_data_access() {
        let mut integrator = ChargeIntegrator::trapezoidal();
        integrator.push(0.0, 1.0);
        integrator.push(1.0, 2.0);
        assert_eq!(integrator.time_data().len(), 2);
        assert_eq!(integrator.current_data().len(), 2);
    }

    #[test]
    fn test_integrator_average_current_zero_time() {
        let mut integrator = ChargeIntegrator::trapezoidal();
        integrator.push(0.0, 1.0);
        assert_eq!(integrator.average_current(), 0.0);
    }
}

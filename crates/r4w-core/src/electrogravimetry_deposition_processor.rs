//! # Electrogravimetric Deposition Processor
//!
//! Implements electrogravimetric analysis for electrochemical deposition and quantitative
//! determination of metals and ions. Covers Faraday's law of electrolysis, Nernst equation,
//! controlled-potential electrolysis, current efficiency, Butler-Volmer kinetics, selective
//! separation, mass transport, deposit quality assessment, and coulometric titration.
//!
//! ## Physics
//!
//! Electrogravimetry is an analytical technique where a metal ion is quantitatively deposited
//! onto an electrode by electrolysis, and the mass of the deposit is measured. Faraday's law
//! relates the mass deposited to the charge passed:
//!
//!   m = (M × I × t) / (n × F)
//!
//! where F = 96485.33212 C/mol (Faraday constant), M is molar mass, I is current, t is time,
//! and n is the number of electrons transferred per ion.
//!
//! The Nernst equation governs the electrode potential:
//!
//!   E = E° - (RT / nF) × ln(Q)
//!
//! Butler-Volmer kinetics describe the current-overpotential relationship:
//!
//!   i = i₀ × [exp(αₐFη/RT) - exp(-αcFη/RT)]

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Faraday constant in C/mol.
pub const FARADAY: f64 = 96485.33212;

/// Universal gas constant in J/(mol·K).
pub const R_GAS: f64 = 8.314462618;

/// Standard temperature in Kelvin (25°C).
pub const STD_TEMP_K: f64 = 298.15;

// ---------------------------------------------------------------------------
// Standard reduction potentials (V vs SHE)
// ---------------------------------------------------------------------------

/// Standard reduction potential for Cu²⁺/Cu.
pub const E0_CU: f64 = 0.34;

/// Standard reduction potential for Ag⁺/Ag.
pub const E0_AG: f64 = 0.80;

/// Standard reduction potential for Ni²⁺/Ni.
pub const E0_NI: f64 = -0.26;

/// Standard reduction potential for Zn²⁺/Zn.
pub const E0_ZN: f64 = -0.76;

/// Standard reduction potential for Pb²⁺/Pb.
pub const E0_PB: f64 = -0.13;

/// Standard reduction potential for Fe²⁺/Fe.
pub const E0_FE: f64 = -0.44;

/// Standard reduction potential for Au³⁺/Au.
pub const E0_AU: f64 = 1.50;

/// Standard reduction potential for Cd²⁺/Cd.
pub const E0_CD: f64 = -0.40;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute mass deposited from Faraday's law.
///
/// m = (M × Q) / (n × F)
///
/// - `mw`: molar mass in g/mol
/// - `charge`: total charge passed in coulombs (I × t)
/// - `n_electrons`: number of electrons per ion
///
/// Returns mass in grams.
pub fn faraday_mass(mw: f64, charge: f64, n_electrons: u32) -> f64 {
    (mw * charge) / (n_electrons as f64 * FARADAY)
}

/// Compute required deposition time from Faraday's law.
///
/// t = (m × n × F) / (M × I)
///
/// - `mass_g`: target mass in grams
/// - `mw`: molar mass in g/mol
/// - `current_a`: constant current in amperes
/// - `n_electrons`: number of electrons per ion
///
/// Returns time in seconds.
pub fn deposition_time(mass_g: f64, mw: f64, current_a: f64, n_electrons: u32) -> f64 {
    (mass_g * n_electrons as f64 * FARADAY) / (mw * current_a)
}

/// Compute Tafel overpotential for a given current.
///
/// η = (RT / αF) × ln(i / i₀)
///
/// Valid for |η| > ~50 mV (high-field approximation).
///
/// - `i`: actual current density (A/m²)
/// - `i0`: exchange current density (A/m²)
/// - `alpha`: transfer coefficient (typically 0.3-0.7)
/// - `temp_k`: temperature in Kelvin
///
/// Returns overpotential in volts.
pub fn overpotential_tafel(i: f64, i0: f64, alpha: f64, temp_k: f64) -> f64 {
    let rt_over_af = (R_GAS * temp_k) / (alpha * FARADAY);
    rt_over_af * (i / i0).ln()
}

/// Nernst potential for a half-cell.
///
/// E = E° - (RT / nF) × ln(Q)
///
/// For Mⁿ⁺ + ne⁻ → M, Q = 1 / [Mⁿ⁺], so:
/// E = E° + (RT / nF) × ln([Mⁿ⁺])
pub fn nernst_potential(e0: f64, n_electrons: u32, concentration_m: f64, temp_k: f64) -> f64 {
    let rt_nf = (R_GAS * temp_k) / (n_electrons as f64 * FARADAY);
    e0 + rt_nf * concentration_m.ln()
}

/// Charge required to deposit a given mass.
///
/// Q = (m × n × F) / M
pub fn charge_for_mass(mass_g: f64, mw: f64, n_electrons: u32) -> f64 {
    (mass_g * n_electrons as f64 * FARADAY) / mw
}

/// Cottrell current at time t.
///
/// i(t) = nFAD^(1/2)C / (πt)^(1/2)
///
/// - `n_electrons`: electron count
/// - `area_cm2`: electrode area in cm²
/// - `diff_coeff`: diffusion coefficient in cm²/s
/// - `concentration`: bulk concentration in mol/cm³
/// - `time_s`: time in seconds (must be > 0)
///
/// Returns current in amperes.
pub fn cottrell_current(
    n_electrons: u32,
    area_cm2: f64,
    diff_coeff: f64,
    concentration: f64,
    time_s: f64,
) -> f64 {
    let n = n_electrons as f64;
    n * FARADAY * area_cm2 * diff_coeff.sqrt() * concentration / (PI * time_s).sqrt()
}

/// Levich limiting current for a rotating disk electrode.
///
/// iL = 0.62 × n × F × A × D^(2/3) × ω^(1/2) × ν^(-1/6) × C
///
/// - `n_electrons`: electron count
/// - `area_cm2`: electrode area in cm²
/// - `diff_coeff`: diffusion coefficient in cm²/s
/// - `omega_rad_s`: angular velocity in rad/s
/// - `kinematic_visc`: kinematic viscosity in cm²/s
/// - `concentration`: bulk concentration in mol/cm³
///
/// Returns limiting current in amperes.
pub fn levich_current(
    n_electrons: u32,
    area_cm2: f64,
    diff_coeff: f64,
    omega_rad_s: f64,
    kinematic_visc: f64,
    concentration: f64,
) -> f64 {
    let n = n_electrons as f64;
    0.62 * n * FARADAY * area_cm2
        * diff_coeff.powf(2.0 / 3.0)
        * omega_rad_s.sqrt()
        * kinematic_visc.powf(-1.0 / 6.0)
        * concentration
}

/// Butler-Volmer current density.
///
/// i = i₀ × [exp(αₐFη/RT) - exp(-αcFη/RT)]
pub fn butler_volmer_current(
    i0: f64,
    alpha_a: f64,
    alpha_c: f64,
    eta: f64,
    temp_k: f64,
) -> f64 {
    let f_rt = FARADAY / (R_GAS * temp_k);
    i0 * ((alpha_a * f_rt * eta).exp() - (-(alpha_c * f_rt * eta)).exp())
}

// ---------------------------------------------------------------------------
// FaradayLaw
// ---------------------------------------------------------------------------

/// Faraday's law of electrolysis calculator.
///
/// Relates charge passed to mass deposited or dissolved at an electrode.
#[derive(Debug, Clone)]
pub struct FaradayLaw {
    /// Molar mass (g/mol).
    pub molar_mass: f64,
    /// Number of electrons transferred per ion.
    pub n_electrons: u32,
    /// Current efficiency (0.0 to 1.0).
    pub efficiency: f64,
}

impl FaradayLaw {
    /// Create a new Faraday law calculator.
    pub fn new(molar_mass: f64, n_electrons: u32) -> Self {
        Self {
            molar_mass,
            n_electrons,
            efficiency: 1.0,
        }
    }

    /// Set current efficiency.
    pub fn with_efficiency(mut self, eff: f64) -> Self {
        self.efficiency = eff.clamp(0.0, 1.0);
        self
    }

    /// Mass deposited for given current and time.
    ///
    /// m = η × (M × I × t) / (n × F)
    pub fn mass_deposited(&self, current_a: f64, time_s: f64) -> f64 {
        self.efficiency * (self.molar_mass * current_a * time_s)
            / (self.n_electrons as f64 * FARADAY)
    }

    /// Charge required to deposit given mass.
    pub fn charge_required(&self, mass_g: f64) -> f64 {
        (mass_g * self.n_electrons as f64 * FARADAY) / (self.molar_mass * self.efficiency)
    }

    /// Time required at a given current to deposit mass.
    pub fn time_required(&self, mass_g: f64, current_a: f64) -> f64 {
        self.charge_required(mass_g) / current_a
    }

    /// Moles deposited from charge.
    pub fn moles_deposited(&self, charge_c: f64) -> f64 {
        self.efficiency * charge_c / (self.n_electrons as f64 * FARADAY)
    }

    /// Equivalent weight = M / n.
    pub fn equivalent_weight(&self) -> f64 {
        self.molar_mass / self.n_electrons as f64
    }

    /// Coulometric analysis: mass from measured charge (for quantitative determination).
    pub fn coulometric_mass(&self, charge_c: f64) -> f64 {
        self.efficiency * self.molar_mass * charge_c / (self.n_electrons as f64 * FARADAY)
    }
}

// ---------------------------------------------------------------------------
// NernstEquation
// ---------------------------------------------------------------------------

/// Nernst equation calculator for half-cell potentials.
#[derive(Debug, Clone)]
pub struct NernstEquation {
    /// Standard reduction potential (V vs SHE).
    pub e_standard: f64,
    /// Number of electrons transferred.
    pub n_electrons: u32,
    /// Temperature in Kelvin.
    pub temperature_k: f64,
}

impl NernstEquation {
    /// Create a new Nernst equation calculator.
    pub fn new(e_standard: f64, n_electrons: u32) -> Self {
        Self {
            e_standard,
            n_electrons,
            temperature_k: STD_TEMP_K,
        }
    }

    /// Set temperature.
    pub fn with_temperature(mut self, temp_k: f64) -> Self {
        self.temperature_k = temp_k;
        self
    }

    /// Electrode potential at a given ion concentration.
    ///
    /// E = E° + (RT/nF) × ln([Mⁿ⁺])
    pub fn potential(&self, concentration_m: f64) -> f64 {
        let rt_nf = (R_GAS * self.temperature_k) / (self.n_electrons as f64 * FARADAY);
        self.e_standard + rt_nf * concentration_m.ln()
    }

    /// Potential from reaction quotient Q.
    ///
    /// E = E° - (RT/nF) × ln(Q)
    pub fn potential_from_q(&self, q: f64) -> f64 {
        let rt_nf = (R_GAS * self.temperature_k) / (self.n_electrons as f64 * FARADAY);
        self.e_standard - rt_nf * q.ln()
    }

    /// RT/nF factor at the set temperature.
    pub fn thermal_voltage_factor(&self) -> f64 {
        (R_GAS * self.temperature_k) / (self.n_electrons as f64 * FARADAY)
    }

    /// Cell voltage for a galvanic cell: E_cell = E_cathode - E_anode.
    pub fn cell_voltage(cathode: &NernstEquation, anode: &NernstEquation,
                        c_cathode: f64, c_anode: f64) -> f64 {
        cathode.potential(c_cathode) - anode.potential(c_anode)
    }

    /// Concentration at which the potential equals a target value.
    ///
    /// [Mⁿ⁺] = exp((E - E°) × nF / RT)
    pub fn concentration_at_potential(&self, target_e: f64) -> f64 {
        let nf_rt = (self.n_electrons as f64 * FARADAY) / (R_GAS * self.temperature_k);
        ((target_e - self.e_standard) * nf_rt).exp()
    }

    /// Presets for common reduction half-cells.
    pub fn copper() -> Self { Self::new(E0_CU, 2) }
    pub fn silver() -> Self { Self::new(E0_AG, 1) }
    pub fn nickel() -> Self { Self::new(E0_NI, 2) }
    pub fn zinc() -> Self { Self::new(E0_ZN, 2) }
    pub fn lead() -> Self { Self::new(E0_PB, 2) }
    pub fn iron() -> Self { Self::new(E0_FE, 2) }
    pub fn gold() -> Self { Self::new(E0_AU, 3) }
    pub fn cadmium() -> Self { Self::new(E0_CD, 2) }
}

// ---------------------------------------------------------------------------
// DepositionController
// ---------------------------------------------------------------------------

/// Controlled-potential electrolysis controller.
///
/// Models the current transient during potentiostatic electrolysis using the
/// Cottrell equation and tracks deposition progress.
#[derive(Debug, Clone)]
pub struct DepositionController {
    /// Number of electrons.
    pub n_electrons: u32,
    /// Electrode area in cm².
    pub area_cm2: f64,
    /// Diffusion coefficient in cm²/s.
    pub diff_coeff: f64,
    /// Initial bulk concentration in mol/cm³.
    pub bulk_concentration: f64,
    /// Molar mass in g/mol.
    pub molar_mass: f64,
    /// Completion fraction (current / initial current) below which deposition is considered done.
    pub completion_threshold: f64,
    /// Solution volume in cm³ (for depletion tracking).
    pub volume_cm3: f64,
}

impl DepositionController {
    /// Create a new deposition controller.
    pub fn new(
        n_electrons: u32,
        area_cm2: f64,
        diff_coeff: f64,
        bulk_concentration: f64,
        molar_mass: f64,
        volume_cm3: f64,
    ) -> Self {
        Self {
            n_electrons,
            area_cm2,
            diff_coeff,
            bulk_concentration,
            molar_mass,
            completion_threshold: 0.001, // 0.1% of initial
            volume_cm3,
        }
    }

    /// Set completion threshold.
    pub fn with_completion_threshold(mut self, threshold: f64) -> Self {
        self.completion_threshold = threshold;
        self
    }

    /// Cottrell current at time t (semi-infinite diffusion).
    pub fn current_at(&self, time_s: f64) -> f64 {
        if time_s <= 0.0 {
            return f64::INFINITY;
        }
        cottrell_current(
            self.n_electrons,
            self.area_cm2,
            self.diff_coeff,
            self.bulk_concentration,
            time_s,
        )
    }

    /// Initial current estimate at t = 1 s (reference point for Cottrell).
    pub fn initial_current(&self) -> f64 {
        self.current_at(1.0)
    }

    /// Check if deposition is essentially complete based on current ratio.
    pub fn is_complete(&self, current_ratio: f64) -> bool {
        current_ratio <= self.completion_threshold
    }

    /// Simulate current transient over a time grid.
    ///
    /// Returns Vec of (time_s, current_a).
    pub fn simulate_transient(&self, times: &[f64]) -> Vec<(f64, f64)> {
        times
            .iter()
            .map(|&t| (t, self.current_at(t)))
            .collect()
    }

    /// Charge passed from t=t1 to t=t2 by integrating Cottrell equation.
    ///
    /// Q = ∫ i(t) dt = 2 × nFAD^(1/2)C × (t2^(1/2) - t1^(1/2)) / π^(1/2)
    pub fn charge_between(&self, t1: f64, t2: f64) -> f64 {
        let n = self.n_electrons as f64;
        let factor =
            2.0 * n * FARADAY * self.area_cm2 * self.diff_coeff.sqrt() * self.bulk_concentration
                / PI.sqrt();
        factor * (t2.sqrt() - t1.sqrt())
    }

    /// Mass deposited from t=0 to t (assuming semi-infinite diffusion).
    pub fn mass_at_time(&self, time_s: f64) -> f64 {
        let charge = self.charge_between(0.001, time_s); // avoid singularity at t=0
        faraday_mass(self.molar_mass, charge, self.n_electrons)
    }

    /// Theoretical total mass available in solution.
    pub fn total_mass_available(&self) -> f64 {
        self.bulk_concentration * self.volume_cm3 * self.molar_mass
    }

    /// Fraction of analyte deposited at time t (simple model).
    pub fn fraction_deposited(&self, time_s: f64) -> f64 {
        let deposited = self.mass_at_time(time_s);
        let total = self.total_mass_available();
        if total <= 0.0 {
            return 0.0;
        }
        (deposited / total).min(1.0)
    }
}

// ---------------------------------------------------------------------------
// CurrentEfficiency
// ---------------------------------------------------------------------------

/// Current efficiency calculator.
///
/// Compares actual mass deposited to theoretical (Faraday) prediction,
/// accounting for side reactions.
#[derive(Debug, Clone)]
pub struct CurrentEfficiency {
    /// Molar mass of the deposit.
    pub molar_mass: f64,
    /// Electrons per ion.
    pub n_electrons: u32,
    /// Current passed in amperes.
    pub current_a: f64,
    /// Electrolysis time in seconds.
    pub time_s: f64,
    /// Actually measured deposit mass in grams.
    pub actual_mass_g: f64,
}

impl CurrentEfficiency {
    /// Create a new current efficiency measurement.
    pub fn new(
        molar_mass: f64,
        n_electrons: u32,
        current_a: f64,
        time_s: f64,
        actual_mass_g: f64,
    ) -> Self {
        Self {
            molar_mass,
            n_electrons,
            current_a,
            time_s,
            actual_mass_g,
        }
    }

    /// Theoretical mass from Faraday's law (100% efficiency).
    pub fn theoretical_mass(&self) -> f64 {
        faraday_mass(
            self.molar_mass,
            self.current_a * self.time_s,
            self.n_electrons,
        )
    }

    /// Current efficiency as a fraction (0.0 to 1.0+).
    pub fn efficiency(&self) -> f64 {
        let theo = self.theoretical_mass();
        if theo <= 0.0 {
            return 0.0;
        }
        self.actual_mass_g / theo
    }

    /// Current efficiency as a percentage.
    pub fn efficiency_percent(&self) -> f64 {
        self.efficiency() * 100.0
    }

    /// Charge consumed by side reactions.
    pub fn side_reaction_charge(&self) -> f64 {
        let total_charge = self.current_a * self.time_s;
        let useful_charge = charge_for_mass(self.actual_mass_g, self.molar_mass, self.n_electrons);
        (total_charge - useful_charge).max(0.0)
    }

    /// Mass lost to H₂ evolution side reaction (assuming 2e⁻ per H₂).
    pub fn h2_evolution_mass(&self) -> f64 {
        let side_q = self.side_reaction_charge();
        // Each mole of H₂ requires 2 moles of electrons
        // m_H2 = (M_H2 × Q_side) / (2 × F)
        (2.016 * side_q) / (2.0 * FARADAY)
    }

    /// Estimate efficiency as a function of current density.
    ///
    /// Simplified model: efficiency drops at very high or very low current densities.
    /// Optimal range centered on `optimal_cd` (A/cm²).
    pub fn efficiency_vs_cd(current_density: f64, optimal_cd: f64) -> f64 {
        // Gaussian-like model centered on optimal current density
        let ratio = (current_density - optimal_cd) / optimal_cd;
        let eff = (-ratio * ratio * 2.0).exp();
        eff.clamp(0.0, 1.0)
    }
}

// ---------------------------------------------------------------------------
// OverpotentialModel
// ---------------------------------------------------------------------------

/// Butler-Volmer electrode kinetics model.
///
/// Models the current-overpotential relationship at an electrode surface.
#[derive(Debug, Clone)]
pub struct OverpotentialModel {
    /// Exchange current density (A/m²).
    pub i0: f64,
    /// Anodic transfer coefficient.
    pub alpha_a: f64,
    /// Cathodic transfer coefficient.
    pub alpha_c: f64,
    /// Temperature in Kelvin.
    pub temperature_k: f64,
}

impl OverpotentialModel {
    /// Create a new Butler-Volmer model.
    ///
    /// Typically alpha_a + alpha_c = 1 (for a single electron step).
    pub fn new(i0: f64, alpha_a: f64, alpha_c: f64) -> Self {
        Self {
            i0,
            alpha_a,
            alpha_c,
            temperature_k: STD_TEMP_K,
        }
    }

    /// Set temperature.
    pub fn with_temperature(mut self, temp_k: f64) -> Self {
        self.temperature_k = temp_k;
        self
    }

    /// Full Butler-Volmer current density for a given overpotential η.
    ///
    /// i = i₀ × [exp(αₐFη/RT) - exp(-αcFη/RT)]
    pub fn current_density(&self, eta: f64) -> f64 {
        butler_volmer_current(self.i0, self.alpha_a, self.alpha_c, eta, self.temperature_k)
    }

    /// Tafel approximation for anodic branch (η >> 0).
    ///
    /// i ≈ i₀ × exp(αₐFη/RT)
    pub fn tafel_anodic(&self, eta: f64) -> f64 {
        let f_rt = FARADAY / (R_GAS * self.temperature_k);
        self.i0 * (self.alpha_a * f_rt * eta).exp()
    }

    /// Tafel approximation for cathodic branch (η << 0).
    ///
    /// i ≈ -i₀ × exp(-αcFη/RT)
    pub fn tafel_cathodic(&self, eta: f64) -> f64 {
        let f_rt = FARADAY / (R_GAS * self.temperature_k);
        -self.i0 * (-self.alpha_c * f_rt * eta).exp()
    }

    /// Tafel slope (V/decade) for the anodic branch.
    ///
    /// b = 2.303 × RT / (αₐF)
    pub fn tafel_slope_anodic(&self) -> f64 {
        2.303 * R_GAS * self.temperature_k / (self.alpha_a * FARADAY)
    }

    /// Tafel slope (V/decade) for the cathodic branch.
    pub fn tafel_slope_cathodic(&self) -> f64 {
        2.303 * R_GAS * self.temperature_k / (self.alpha_c * FARADAY)
    }

    /// Linearized Butler-Volmer for small η (|η| < ~10 mV).
    ///
    /// i ≈ i₀ × (αₐ + αc) × F × η / RT
    pub fn linearized_current(&self, eta: f64) -> f64 {
        let f_rt = FARADAY / (R_GAS * self.temperature_k);
        self.i0 * (self.alpha_a + self.alpha_c) * f_rt * eta
    }

    /// Charge transfer resistance at equilibrium.
    ///
    /// R_ct = RT / (nF × i₀) where n = αₐ + αc ≈ 1 for single electron step.
    pub fn charge_transfer_resistance(&self) -> f64 {
        (R_GAS * self.temperature_k) / ((self.alpha_a + self.alpha_c) * FARADAY * self.i0)
    }

    /// Overpotential required to achieve a given current density (anodic Tafel).
    pub fn eta_for_current(&self, i: f64) -> f64 {
        overpotential_tafel(i, self.i0, self.alpha_a, self.temperature_k)
    }

    /// Scan overpotential range and return (η, i) pairs.
    pub fn polarization_curve(&self, eta_min: f64, eta_max: f64, n_points: usize) -> Vec<(f64, f64)> {
        let step = (eta_max - eta_min) / (n_points as f64 - 1.0);
        (0..n_points)
            .map(|k| {
                let eta = eta_min + k as f64 * step;
                (eta, self.current_density(eta))
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// SelectiveSeparation
// ---------------------------------------------------------------------------

/// Entry for a metal species in a separation problem.
#[derive(Debug, Clone)]
pub struct MetalSpecies {
    /// Name of the metal.
    pub name: String,
    /// Standard reduction potential (V vs SHE).
    pub e_standard: f64,
    /// Number of electrons.
    pub n_electrons: u32,
    /// Concentration in mol/L.
    pub concentration_m: f64,
    /// Molar mass in g/mol.
    pub molar_mass: f64,
}

impl MetalSpecies {
    /// Create a new metal species.
    pub fn new(name: &str, e_standard: f64, n_electrons: u32, concentration_m: f64, molar_mass: f64) -> Self {
        Self {
            name: name.to_string(),
            e_standard,
            n_electrons,
            concentration_m,
            molar_mass,
        }
    }

    /// Nernst potential at the current concentration.
    pub fn nernst(&self, temp_k: f64) -> f64 {
        nernst_potential(self.e_standard, self.n_electrons, self.concentration_m, temp_k)
    }
}

/// Selective separation controller for depositing metals from a mixture.
#[derive(Debug, Clone)]
pub struct SelectiveSeparation {
    /// List of metal species in solution.
    pub species: Vec<MetalSpecies>,
    /// Temperature in Kelvin.
    pub temperature_k: f64,
}

impl SelectiveSeparation {
    /// Create a new selective separation problem.
    pub fn new(species: Vec<MetalSpecies>) -> Self {
        Self {
            species,
            temperature_k: STD_TEMP_K,
        }
    }

    /// Set temperature.
    pub fn with_temperature(mut self, temp_k: f64) -> Self {
        self.temperature_k = temp_k;
        self
    }

    /// Deposition order: species sorted by Nernst potential (most positive first).
    pub fn deposition_order(&self) -> Vec<&MetalSpecies> {
        let mut sorted: Vec<&MetalSpecies> = self.species.iter().collect();
        sorted.sort_by(|a, b| {
            b.nernst(self.temperature_k)
                .partial_cmp(&a.nernst(self.temperature_k))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        sorted
    }

    /// Potential window for selective deposition of the i-th species (0-based by deposition order).
    ///
    /// Returns (E_start, E_stop) where E_start is the onset potential of the target
    /// and E_stop is the onset of the next species.
    pub fn potential_window(&self, index: usize) -> Option<(f64, f64)> {
        let order = self.deposition_order();
        if index >= order.len() {
            return None;
        }
        let e_start = order[index].nernst(self.temperature_k);
        let e_stop = if index + 1 < order.len() {
            order[index + 1].nernst(self.temperature_k)
        } else {
            e_start - 0.5 // arbitrary lower bound
        };
        Some((e_start, e_stop))
    }

    /// Separation factor between two species (difference in Nernst potentials).
    ///
    /// A larger ΔE means easier separation.
    pub fn separation_factor(&self, species_a: usize, species_b: usize) -> f64 {
        if species_a >= self.species.len() || species_b >= self.species.len() {
            return 0.0;
        }
        let ea = self.species[species_a].nernst(self.temperature_k);
        let eb = self.species[species_b].nernst(self.temperature_k);
        (ea - eb).abs()
    }

    /// Check if two species can be separated with at least `min_delta_v` potential difference.
    pub fn can_separate(&self, species_a: usize, species_b: usize, min_delta_v: f64) -> bool {
        self.separation_factor(species_a, species_b) >= min_delta_v
    }

    /// Required potential (most negative Nernst) to deposit all species.
    pub fn total_deposition_potential(&self) -> f64 {
        self.species
            .iter()
            .map(|s| s.nernst(self.temperature_k))
            .fold(f64::INFINITY, f64::min)
    }

    /// Concentration at which a given species effectively "finishes" depositing.
    ///
    /// Uses Nernst equation: [M] = exp((E_stop - E°) × nF / RT)
    pub fn residual_concentration(&self, species_idx: usize, e_stop: f64) -> f64 {
        let s = &self.species[species_idx];
        let nf_rt = (s.n_electrons as f64 * FARADAY) / (R_GAS * self.temperature_k);
        ((e_stop - s.e_standard) * nf_rt).exp()
    }
}

// ---------------------------------------------------------------------------
// MassTransport
// ---------------------------------------------------------------------------

/// Mass transport model for electrode reactions.
///
/// Models diffusion-limited current under various hydrodynamic conditions.
#[derive(Debug, Clone)]
pub struct MassTransport {
    /// Number of electrons.
    pub n_electrons: u32,
    /// Electrode area in cm².
    pub area_cm2: f64,
    /// Diffusion coefficient in cm²/s.
    pub diff_coeff: f64,
    /// Bulk concentration in mol/cm³.
    pub concentration: f64,
    /// Nernst diffusion layer thickness in cm.
    pub delta_cm: f64,
    /// Kinematic viscosity in cm²/s.
    pub kinematic_visc: f64,
}

impl MassTransport {
    /// Create a new mass transport model.
    pub fn new(
        n_electrons: u32,
        area_cm2: f64,
        diff_coeff: f64,
        concentration: f64,
    ) -> Self {
        Self {
            n_electrons,
            area_cm2,
            diff_coeff,
            concentration,
            delta_cm: 0.05, // typical stagnant diffusion layer
            kinematic_visc: 0.01, // water at 25°C
        }
    }

    /// Set diffusion layer thickness.
    pub fn with_delta(mut self, delta_cm: f64) -> Self {
        self.delta_cm = delta_cm;
        self
    }

    /// Set kinematic viscosity.
    pub fn with_viscosity(mut self, visc: f64) -> Self {
        self.kinematic_visc = visc;
        self
    }

    /// Diffusion-limited (steady-state) current for stagnant solution.
    ///
    /// iL = nFDC / δ  (per unit area, then multiply by A)
    pub fn limiting_current_stagnant(&self) -> f64 {
        self.n_electrons as f64 * FARADAY * self.area_cm2 * self.diff_coeff * self.concentration
            / self.delta_cm
    }

    /// Levich equation for rotating disk electrode.
    pub fn limiting_current_rde(&self, omega_rad_s: f64) -> f64 {
        levich_current(
            self.n_electrons,
            self.area_cm2,
            self.diff_coeff,
            omega_rad_s,
            self.kinematic_visc,
            self.concentration,
        )
    }

    /// Effective diffusion layer thickness for RDE (Levich).
    ///
    /// δ = 1.61 × D^(1/3) × ω^(-1/2) × ν^(1/6)
    pub fn rde_diffusion_layer(&self, omega_rad_s: f64) -> f64 {
        1.61
            * self.diff_coeff.powf(1.0 / 3.0)
            * omega_rad_s.powf(-0.5)
            * self.kinematic_visc.powf(1.0 / 6.0)
    }

    /// Cottrell transient current at time t.
    pub fn cottrell_at(&self, time_s: f64) -> f64 {
        cottrell_current(
            self.n_electrons,
            self.area_cm2,
            self.diff_coeff,
            self.concentration,
            time_s,
        )
    }

    /// Koutecky-Levich analysis: 1/i = 1/i_k + 1/i_L.
    ///
    /// Returns kinetic current from measured current and limiting current.
    pub fn kinetic_current(&self, measured: f64, limiting: f64) -> f64 {
        if (measured - limiting).abs() < 1e-15 {
            return f64::INFINITY;
        }
        (measured * limiting) / (limiting - measured)
    }

    /// Mass flux at the electrode surface (mol/(cm²·s)).
    pub fn mass_flux(&self) -> f64 {
        self.diff_coeff * self.concentration / self.delta_cm
    }

    /// Reynolds number for rotating disk.
    pub fn reynolds_rde(&self, omega_rad_s: f64, disk_radius_cm: f64) -> f64 {
        omega_rad_s * disk_radius_cm * disk_radius_cm / self.kinematic_visc
    }
}

// ---------------------------------------------------------------------------
// DepositQuality
// ---------------------------------------------------------------------------

/// Deposit morphology classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DepositMorphology {
    /// Smooth, adherent deposit (low current density).
    Smooth,
    /// Fine-grained with some roughness.
    FineGrained,
    /// Nodular growth (moderate over-current).
    Nodular,
    /// Dendritic tree-like growth (high current density).
    Dendritic,
    /// Powdery, non-adherent (very high current density).
    Powdery,
}

/// Deposit quality assessment.
#[derive(Debug, Clone)]
pub struct DepositQuality {
    /// Limiting current density (A/cm²).
    pub limiting_cd: f64,
}

impl DepositQuality {
    /// Create a new deposit quality model.
    pub fn new(limiting_cd: f64) -> Self {
        Self { limiting_cd }
    }

    /// Classify morphology based on fraction of limiting current density.
    ///
    /// - 0..20% → Smooth
    /// - 20..50% → FineGrained
    /// - 50..80% → Nodular
    /// - 80..100% → Dendritic
    /// - >100% → Powdery
    pub fn classify(&self, current_density: f64) -> DepositMorphology {
        let ratio = current_density / self.limiting_cd;
        if ratio < 0.2 {
            DepositMorphology::Smooth
        } else if ratio < 0.5 {
            DepositMorphology::FineGrained
        } else if ratio < 0.8 {
            DepositMorphology::Nodular
        } else if ratio <= 1.0 {
            DepositMorphology::Dendritic
        } else {
            DepositMorphology::Powdery
        }
    }

    /// Throwing power index: TP = (M - K) / (M + K - 2) × 100.
    ///
    /// M = ratio of far/near cathode distances in Hull cell.
    /// K = ratio of deposit masses far/near.
    ///
    /// Higher TP means more uniform current distribution.
    pub fn throwing_power(distance_ratio: f64, mass_ratio: f64) -> f64 {
        if (distance_ratio + mass_ratio - 2.0).abs() < 1e-12 {
            return 0.0;
        }
        (distance_ratio - mass_ratio) / (distance_ratio + mass_ratio - 2.0) * 100.0
    }

    /// Hull cell current density distribution along cathode.
    ///
    /// Approximate primary current distribution: j(x) ∝ (1/x)^p
    /// where x is position along the cathode (0 = near, 1 = far) and p depends on geometry.
    ///
    /// Returns normalized current density at position x (0..1).
    pub fn hull_cell_cd(&self, x_frac: f64, power: f64) -> f64 {
        if x_frac <= 0.0 {
            return f64::INFINITY;
        }
        let x = x_frac.clamp(0.01, 1.0);
        (1.0 / x).powf(power)
    }

    /// Wagner number: Wa = (dη/di) × κ / L
    ///
    /// Ratio of polarization resistance to electrolyte resistance.
    /// Wa >> 1 → uniform current distribution.
    ///
    /// - `d_eta_di`: slope of overpotential vs current density (Ω·cm²)
    /// - `conductivity`: electrolyte conductivity (S/cm)
    /// - `length_cm`: characteristic length (cm)
    pub fn wagner_number(d_eta_di: f64, conductivity: f64, length_cm: f64) -> f64 {
        d_eta_di * conductivity / length_cm
    }

    /// Grain size estimate (empirical model).
    ///
    /// Returns approximate grain size in micrometers.
    /// Lower current density → larger grains; higher → finer (up to a point).
    pub fn grain_size_um(&self, current_density: f64) -> f64 {
        let ratio = current_density / self.limiting_cd;
        if ratio < 0.01 {
            return 100.0; // very coarse
        }
        // Empirical: grain size ~ 1/sqrt(ratio) with normalization
        10.0 / ratio.sqrt()
    }
}

// ---------------------------------------------------------------------------
// CoulometricTitration
// ---------------------------------------------------------------------------

/// Coulometric titration data point.
#[derive(Debug, Clone, Copy)]
pub struct TitrationPoint {
    /// Time in seconds.
    pub time_s: f64,
    /// Potential (indicator electrode) in volts.
    pub potential_v: f64,
    /// Cumulative charge in coulombs.
    pub charge_c: f64,
}

/// Coulometric titration processor.
///
/// Determines endpoint from potential break during constant-current coulometric titration.
#[derive(Debug, Clone)]
pub struct CoulometricTitration {
    /// Constant generating current in amperes.
    pub current_a: f64,
    /// Molar mass of analyte in g/mol.
    pub molar_mass: f64,
    /// Electrons per mole of analyte consumed.
    pub n_electrons: u32,
    /// Data points (time, potential, charge).
    pub data: Vec<TitrationPoint>,
}

impl CoulometricTitration {
    /// Create a new coulometric titration.
    pub fn new(current_a: f64, molar_mass: f64, n_electrons: u32) -> Self {
        Self {
            current_a,
            molar_mass,
            n_electrons,
            data: Vec::new(),
        }
    }

    /// Add a data point.
    pub fn add_point(&mut self, time_s: f64, potential_v: f64) {
        let charge_c = self.current_a * time_s;
        self.data.push(TitrationPoint {
            time_s,
            potential_v,
            charge_c,
        });
    }

    /// Find the endpoint by maximum |dE/dt| (first derivative).
    ///
    /// Returns the index of the endpoint in the data array.
    pub fn find_endpoint_first_derivative(&self) -> Option<usize> {
        if self.data.len() < 3 {
            return None;
        }
        let mut max_deriv = 0.0_f64;
        let mut max_idx = 0;
        for i in 1..self.data.len() - 1 {
            let dt = self.data[i + 1].time_s - self.data[i - 1].time_s;
            if dt <= 0.0 {
                continue;
            }
            let de = self.data[i + 1].potential_v - self.data[i - 1].potential_v;
            let deriv = (de / dt).abs();
            if deriv > max_deriv {
                max_deriv = deriv;
                max_idx = i;
            }
        }
        Some(max_idx)
    }

    /// Find the endpoint by second derivative zero-crossing.
    ///
    /// Returns the index closest to the zero-crossing.
    pub fn find_endpoint_second_derivative(&self) -> Option<usize> {
        if self.data.len() < 5 {
            return None;
        }
        // Compute first derivatives
        let n = self.data.len();
        let mut first_deriv = vec![0.0; n];
        for i in 1..n - 1 {
            let dt = self.data[i + 1].time_s - self.data[i - 1].time_s;
            if dt > 0.0 {
                first_deriv[i] = (self.data[i + 1].potential_v - self.data[i - 1].potential_v) / dt;
            }
        }

        // Compute second derivatives and find zero-crossing with largest first derivative
        let mut best_idx = None;
        let mut best_first_deriv = 0.0_f64;
        for i in 2..n - 2 {
            let dt = self.data[i + 1].time_s - self.data[i - 1].time_s;
            if dt <= 0.0 {
                continue;
            }
            let d2 = (first_deriv[i + 1] - first_deriv[i - 1]) / dt;
            let d2_next = if i + 2 < n - 1 {
                let dt2 = self.data[i + 2].time_s - self.data[i].time_s;
                if dt2 > 0.0 {
                    (first_deriv[i + 2] - first_deriv[i]) / dt2
                } else {
                    d2
                }
            } else {
                d2
            };
            // Zero crossing: sign change in second derivative
            if d2 * d2_next <= 0.0 && first_deriv[i].abs() > best_first_deriv {
                best_first_deriv = first_deriv[i].abs();
                best_idx = Some(i);
            }
        }
        best_idx
    }

    /// Analyte mass at the endpoint.
    pub fn mass_at_endpoint(&self) -> Option<f64> {
        let idx = self.find_endpoint_first_derivative()?;
        let charge = self.data[idx].charge_c;
        Some(faraday_mass(self.molar_mass, charge, self.n_electrons))
    }

    /// Analyte moles at the endpoint.
    pub fn moles_at_endpoint(&self) -> Option<f64> {
        let mass = self.mass_at_endpoint()?;
        Some(mass / self.molar_mass)
    }

    /// Charge at the endpoint.
    pub fn charge_at_endpoint(&self) -> Option<f64> {
        let idx = self.find_endpoint_first_derivative()?;
        Some(self.data[idx].charge_c)
    }

    /// Concentration of analyte in solution (given volume in L).
    pub fn concentration_at_endpoint(&self, volume_l: f64) -> Option<f64> {
        let moles = self.moles_at_endpoint()?;
        Some(moles / volume_l)
    }

    /// Generate a simulated titration curve.
    ///
    /// Potential follows Nernst equation, with a sharp break at the equivalence point.
    pub fn simulate_curve(
        current_a: f64,
        molar_mass: f64,
        n_electrons: u32,
        analyte_mass_g: f64,
        n_points: usize,
    ) -> Self {
        let mut titration = Self::new(current_a, molar_mass, n_electrons);
        let eq_charge = charge_for_mass(analyte_mass_g, molar_mass, n_electrons);
        let eq_time = eq_charge / current_a;
        let total_time = eq_time * 1.5;
        let dt = total_time / n_points as f64;

        for i in 0..n_points {
            let t = (i as f64 + 0.5) * dt;
            let fraction = (current_a * t) / eq_charge;
            // Nernst-like sigmoidal potential response
            let potential = if fraction < 0.999 {
                0.5 + 0.059 / n_electrons as f64 * (fraction / (1.0 - fraction)).ln()
            } else {
                0.5 + 0.059 / n_electrons as f64 * (0.999 / 0.001_f64).ln()
                    + (fraction - 0.999) * 200.0
            };
            titration.add_point(t, potential);
        }
        titration
    }

    /// Karl Fischer coulometric water equivalent.
    ///
    /// 1 mole of I₂ = 1 mole of H₂O.
    /// I₂ + SO₂ + 3 C₅H₅N + CH₃OH + H₂O → 2 C₅H₅N·HI + C₅H₅N·HSO₄CH₃
    ///
    /// Returns water mass in micrograms from charge in coulombs.
    pub fn karl_fischer_water_ug(charge_c: f64) -> f64 {
        // n=2 electrons per I₂, M(H₂O) = 18.015 g/mol
        let mass_g = faraday_mass(18.015, charge_c, 2);
        mass_g * 1e6 // convert to micrograms
    }
}

// ---------------------------------------------------------------------------
// Electrodeposition bath parameters
// ---------------------------------------------------------------------------

/// Electroplating bath composition and operating parameters.
#[derive(Debug, Clone)]
pub struct PlatingBath {
    /// Metal ion concentration in mol/L.
    pub metal_conc: f64,
    /// pH of the bath.
    pub ph: f64,
    /// Temperature in Kelvin.
    pub temperature_k: f64,
    /// Conductivity in S/cm.
    pub conductivity: f64,
    /// Agitation level (0 = stagnant, 1 = vigorous).
    pub agitation: f64,
}

impl PlatingBath {
    /// Create a new plating bath.
    pub fn new(metal_conc: f64, ph: f64) -> Self {
        Self {
            metal_conc,
            ph,
            temperature_k: STD_TEMP_K,
            conductivity: 0.1,
            agitation: 0.5,
        }
    }

    /// Set temperature.
    pub fn with_temperature(mut self, temp_k: f64) -> Self {
        self.temperature_k = temp_k;
        self
    }

    /// Copper acid sulfate bath preset.
    pub fn copper_sulfate() -> Self {
        Self {
            metal_conc: 0.25,   // ~0.25 M CuSO₄
            ph: 0.5,            // acid
            temperature_k: 298.15,
            conductivity: 0.15,
            agitation: 0.5,
        }
    }

    /// Nickel Watts bath preset.
    pub fn nickel_watts() -> Self {
        Self {
            metal_conc: 1.0,    // ~1 M NiSO₄
            ph: 4.0,
            temperature_k: 328.15, // 55°C
            conductivity: 0.08,
            agitation: 0.5,
        }
    }

    /// Hydrogen evolution potential at this pH.
    ///
    /// E(H₂) = 0 - 0.0592 × pH (at 25°C)
    pub fn h2_evolution_potential(&self) -> f64 {
        // Nernst equation for 2H⁺ + 2e⁻ → H₂
        // E = 0 - (RT/F) × ln(1/[H⁺]) = -0.0592 × pH at 25°C
        let rt_f = R_GAS * self.temperature_k / FARADAY;
        -2.303 * rt_f * self.ph
    }

    /// Oxygen evolution potential at this pH.
    ///
    /// E(O₂) = 1.229 - 0.0592 × pH (at 25°C)
    pub fn o2_evolution_potential(&self) -> f64 {
        let rt_f = R_GAS * self.temperature_k / FARADAY;
        1.229 - 2.303 * rt_f * self.ph
    }

    /// Effective diffusion layer thickness based on agitation.
    ///
    /// δ ranges from ~0.05 cm (stagnant) to ~0.005 cm (vigorous stirring).
    pub fn diffusion_layer_cm(&self) -> f64 {
        0.05 * (1.0 - 0.9 * self.agitation)
    }
}

// ---------------------------------------------------------------------------
// Pulse plating
// ---------------------------------------------------------------------------

/// Pulse plating waveform for improved deposit quality.
#[derive(Debug, Clone)]
pub struct PulsePlating {
    /// Peak current density in A/cm².
    pub peak_cd: f64,
    /// On-time in seconds.
    pub t_on: f64,
    /// Off-time in seconds.
    pub t_off: f64,
    /// Number of cycles.
    pub n_cycles: u32,
}

impl PulsePlating {
    /// Create a new pulse plating configuration.
    pub fn new(peak_cd: f64, t_on: f64, t_off: f64) -> Self {
        Self {
            peak_cd,
            t_on,
            t_off,
            n_cycles: 1000,
        }
    }

    /// Set number of cycles.
    pub fn with_cycles(mut self, n: u32) -> Self {
        self.n_cycles = n;
        self
    }

    /// Duty cycle as a fraction.
    pub fn duty_cycle(&self) -> f64 {
        self.t_on / (self.t_on + self.t_off)
    }

    /// Average current density.
    pub fn average_cd(&self) -> f64 {
        self.peak_cd * self.duty_cycle()
    }

    /// Total plating time.
    pub fn total_time(&self) -> f64 {
        self.n_cycles as f64 * (self.t_on + self.t_off)
    }

    /// Frequency of the pulse waveform.
    pub fn frequency_hz(&self) -> f64 {
        1.0 / (self.t_on + self.t_off)
    }

    /// Generate current waveform over one period.
    pub fn waveform_one_period(&self, n_points: usize) -> Vec<(f64, f64)> {
        let period = self.t_on + self.t_off;
        let dt = period / n_points as f64;
        (0..n_points)
            .map(|i| {
                let t = i as f64 * dt;
                let cd = if t < self.t_on {
                    self.peak_cd
                } else {
                    0.0
                };
                (t, cd)
            })
            .collect()
    }

    /// Charge per pulse cycle.
    pub fn charge_per_cycle(&self, area_cm2: f64) -> f64 {
        self.peak_cd * area_cm2 * self.t_on
    }

    /// Total charge for all cycles.
    pub fn total_charge(&self, area_cm2: f64) -> f64 {
        self.charge_per_cycle(area_cm2) * self.n_cycles as f64
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 1e-3;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol || (a - b).abs() / (a.abs().max(b.abs()).max(1e-15)) < tol
    }

    // =======================================================================
    // Constants
    // =======================================================================

    #[test]
    fn test_faraday_constant() {
        assert!(approx_eq(FARADAY, 96485.33212, 0.01));
    }

    #[test]
    fn test_gas_constant() {
        assert!(approx_eq(R_GAS, 8.314462618, 1e-6));
    }

    #[test]
    fn test_std_temp() {
        assert!(approx_eq(STD_TEMP_K, 298.15, 0.01));
    }

    // =======================================================================
    // Helper functions
    // =======================================================================

    #[test]
    fn test_faraday_mass_copper() {
        // 1 A for 1 hour depositing Cu²⁺ (M=63.546, n=2)
        let charge = 1.0 * 3600.0;
        let mass = faraday_mass(63.546, charge, 2);
        // Expected: 63.546 * 3600 / (2 * 96485.33) ≈ 1.1854 g
        assert!(approx_eq(mass, 1.1854, 0.01));
    }

    #[test]
    fn test_faraday_mass_silver() {
        // Silver: M=107.868, n=1
        let charge = 1.0 * 3600.0;
        let mass = faraday_mass(107.868, charge, 1);
        // Expected: 107.868 * 3600 / (1 * 96485.33) ≈ 4.025 g
        assert!(approx_eq(mass, 4.025, 0.01));
    }

    #[test]
    fn test_faraday_mass_zero_charge() {
        let mass = faraday_mass(63.546, 0.0, 2);
        assert!(approx_eq(mass, 0.0, TOL));
    }

    #[test]
    fn test_deposition_time_copper() {
        // Deposit 1g of Cu at 1A: t = (1 * 2 * 96485) / (63.546 * 1) ≈ 3036 s
        let t = deposition_time(1.0, 63.546, 1.0, 2);
        assert!(approx_eq(t, 3036.0, 5.0));
    }

    #[test]
    fn test_deposition_time_inverse_of_faraday_mass() {
        let mass = 2.0;
        let mw = 107.868;
        let current = 0.5;
        let n = 1;
        let t = deposition_time(mass, mw, current, n);
        let m_check = faraday_mass(mw, current * t, n);
        assert!(approx_eq(m_check, mass, 1e-6));
    }

    #[test]
    fn test_overpotential_tafel_basic() {
        let eta = overpotential_tafel(1.0, 0.01, 0.5, 298.15);
        // η = (RT/αF) × ln(100) = (8.314*298.15)/(0.5*96485) × ln(100)
        // = 0.05139 × 4.605 ≈ 0.2367 V
        assert!(approx_eq(eta, 0.2367, 0.005));
    }

    #[test]
    fn test_overpotential_tafel_at_i0() {
        let eta = overpotential_tafel(1.0, 1.0, 0.5, 298.15);
        assert!(approx_eq(eta, 0.0, TOL));
    }

    #[test]
    fn test_nernst_potential_standard() {
        // At 1 M concentration, E = E° (since ln(1) = 0)
        let e = nernst_potential(E0_CU, 2, 1.0, STD_TEMP_K);
        assert!(approx_eq(e, E0_CU, TOL));
    }

    #[test]
    fn test_nernst_potential_dilute() {
        // At 0.001 M Cu²⁺: E = 0.34 + (RT/2F) × ln(0.001)
        let e = nernst_potential(E0_CU, 2, 0.001, STD_TEMP_K);
        // RT/2F ≈ 0.01285, ln(0.001) ≈ -6.908
        assert!(e < E0_CU);
        assert!(approx_eq(e, 0.34 + 0.01285 * (-6.908), 0.005));
    }

    #[test]
    fn test_charge_for_mass_roundtrip() {
        let mass = 5.0;
        let q = charge_for_mass(mass, 63.546, 2);
        let m_back = faraday_mass(63.546, q, 2);
        assert!(approx_eq(m_back, mass, 1e-8));
    }

    #[test]
    fn test_cottrell_current_positive() {
        let i = cottrell_current(2, 1.0, 1e-5, 1e-5, 1.0);
        assert!(i > 0.0);
    }

    #[test]
    fn test_cottrell_current_decreases() {
        let i1 = cottrell_current(2, 1.0, 1e-5, 1e-5, 1.0);
        let i2 = cottrell_current(2, 1.0, 1e-5, 1e-5, 4.0);
        // At 4x time, current should be 1/2 (sqrt(4) = 2)
        assert!(approx_eq(i2, i1 / 2.0, 1e-10));
    }

    #[test]
    fn test_levich_current_positive() {
        let i = levich_current(2, 1.0, 1e-5, 100.0, 0.01, 1e-5);
        assert!(i > 0.0);
    }

    #[test]
    fn test_levich_increases_with_omega() {
        let i1 = levich_current(2, 1.0, 1e-5, 100.0, 0.01, 1e-5);
        let i2 = levich_current(2, 1.0, 1e-5, 400.0, 0.01, 1e-5);
        // omega × 4 → current × 2 (sqrt dependence)
        assert!(approx_eq(i2 / i1, 2.0, 0.01));
    }

    #[test]
    fn test_butler_volmer_zero_overpotential() {
        let i = butler_volmer_current(0.01, 0.5, 0.5, 0.0, STD_TEMP_K);
        assert!(approx_eq(i, 0.0, 1e-10));
    }

    #[test]
    fn test_butler_volmer_symmetry() {
        let i_pos = butler_volmer_current(0.01, 0.5, 0.5, 0.1, STD_TEMP_K);
        let i_neg = butler_volmer_current(0.01, 0.5, 0.5, -0.1, STD_TEMP_K);
        assert!(approx_eq(i_pos, -i_neg, 1e-10));
    }

    // =======================================================================
    // FaradayLaw
    // =======================================================================

    #[test]
    fn test_faraday_law_mass_deposited() {
        let fl = FaradayLaw::new(63.546, 2);
        let mass = fl.mass_deposited(1.0, 3600.0);
        assert!(approx_eq(mass, 1.1854, 0.01));
    }

    #[test]
    fn test_faraday_law_with_efficiency() {
        let fl = FaradayLaw::new(63.546, 2).with_efficiency(0.9);
        let mass = fl.mass_deposited(1.0, 3600.0);
        let expected = 0.9 * 63.546 * 3600.0 / (2.0 * FARADAY);
        assert!(approx_eq(mass, expected, 1e-4));
    }

    #[test]
    fn test_faraday_law_charge_required() {
        let fl = FaradayLaw::new(63.546, 2);
        let q = fl.charge_required(1.0);
        let expected = (1.0 * 2.0 * FARADAY) / 63.546;
        assert!(approx_eq(q, expected, 0.01));
    }

    #[test]
    fn test_faraday_law_time_required() {
        let fl = FaradayLaw::new(63.546, 2);
        let t = fl.time_required(1.0, 1.0);
        let expected = (1.0 * 2.0 * FARADAY) / (63.546 * 1.0);
        assert!(approx_eq(t, expected, 0.01));
    }

    #[test]
    fn test_faraday_law_moles_deposited() {
        let fl = FaradayLaw::new(63.546, 2);
        let q = 2.0 * FARADAY; // 2 × F charge → 1 mol electrons per eq → deposit 1 mol Cu
        let moles = fl.moles_deposited(q);
        assert!(approx_eq(moles, 1.0, TOL));
    }

    #[test]
    fn test_faraday_law_equivalent_weight() {
        let fl = FaradayLaw::new(63.546, 2);
        assert!(approx_eq(fl.equivalent_weight(), 31.773, 0.001));
    }

    #[test]
    fn test_faraday_law_coulometric_mass() {
        let fl = FaradayLaw::new(107.868, 1);
        let q = FARADAY; // 1 Faraday → 1 equiv → 107.868 g Ag
        let mass = fl.coulometric_mass(q);
        assert!(approx_eq(mass, 107.868, 0.01));
    }

    #[test]
    fn test_faraday_law_efficiency_clamped() {
        let fl = FaradayLaw::new(63.546, 2).with_efficiency(1.5);
        assert!(approx_eq(fl.efficiency, 1.0, TOL));
        let fl2 = FaradayLaw::new(63.546, 2).with_efficiency(-0.5);
        assert!(approx_eq(fl2.efficiency, 0.0, TOL));
    }

    // =======================================================================
    // NernstEquation
    // =======================================================================

    #[test]
    fn test_nernst_copper_1m() {
        let ne = NernstEquation::copper();
        let e = ne.potential(1.0);
        assert!(approx_eq(e, E0_CU, TOL));
    }

    #[test]
    fn test_nernst_silver() {
        let ne = NernstEquation::silver();
        assert!(approx_eq(ne.e_standard, E0_AG, TOL));
        assert_eq!(ne.n_electrons, 1);
    }

    #[test]
    fn test_nernst_with_temperature() {
        let ne = NernstEquation::copper().with_temperature(350.0);
        assert!(approx_eq(ne.temperature_k, 350.0, TOL));
        // Higher T → larger RT/nF → more shift for same concentration
        let e = ne.potential(0.01);
        let e_std = NernstEquation::copper().potential(0.01);
        // At higher temp the correction is larger (more negative for dilute)
        assert!(e < e_std);
    }

    #[test]
    fn test_nernst_thermal_voltage_factor() {
        let ne = NernstEquation::new(0.34, 2);
        let tvf = ne.thermal_voltage_factor();
        // RT/(nF) = 8.314*298.15/(2*96485) ≈ 0.01285 V
        assert!(approx_eq(tvf, 0.01285, 0.0001));
    }

    #[test]
    fn test_nernst_cell_voltage() {
        let cathode = NernstEquation::copper();
        let anode = NernstEquation::zinc();
        let v = NernstEquation::cell_voltage(&cathode, &anode, 1.0, 1.0);
        // 0.34 - (-0.76) = 1.10 V
        assert!(approx_eq(v, 1.10, 0.01));
    }

    #[test]
    fn test_nernst_potential_from_q() {
        let ne = NernstEquation::copper();
        // Q=1 → E = E°
        let e = ne.potential_from_q(1.0);
        assert!(approx_eq(e, E0_CU, TOL));
    }

    #[test]
    fn test_nernst_concentration_at_potential() {
        let ne = NernstEquation::copper();
        // Roundtrip: set concentration, get potential, recover concentration
        let c = 0.05;
        let e = ne.potential(c);
        let c_recovered = ne.concentration_at_potential(e);
        assert!(approx_eq(c_recovered, c, 1e-8));
    }

    #[test]
    fn test_nernst_presets() {
        assert!(approx_eq(NernstEquation::nickel().e_standard, E0_NI, TOL));
        assert!(approx_eq(NernstEquation::lead().e_standard, E0_PB, TOL));
        assert!(approx_eq(NernstEquation::iron().e_standard, E0_FE, TOL));
        assert!(approx_eq(NernstEquation::gold().e_standard, E0_AU, TOL));
        assert!(approx_eq(NernstEquation::cadmium().e_standard, E0_CD, TOL));
    }

    // =======================================================================
    // DepositionController
    // =======================================================================

    #[test]
    fn test_deposition_controller_current_at() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        let i = dc.current_at(1.0);
        assert!(i > 0.0);
    }

    #[test]
    fn test_deposition_controller_current_decreases() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        let i1 = dc.current_at(1.0);
        let i10 = dc.current_at(10.0);
        assert!(i10 < i1);
    }

    #[test]
    fn test_deposition_controller_initial_current() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        assert!(approx_eq(dc.initial_current(), dc.current_at(1.0), TOL));
    }

    #[test]
    fn test_deposition_controller_is_complete() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        assert!(!dc.is_complete(0.5));
        assert!(dc.is_complete(0.0005));
    }

    #[test]
    fn test_deposition_controller_simulate_transient() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        let times: Vec<f64> = (1..=5).map(|i| i as f64).collect();
        let transient = dc.simulate_transient(&times);
        assert_eq!(transient.len(), 5);
        // Current should decrease monotonically
        for i in 1..transient.len() {
            assert!(transient[i].1 < transient[i - 1].1);
        }
    }

    #[test]
    fn test_deposition_controller_charge_between() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        let q = dc.charge_between(1.0, 4.0);
        // Q = 2nFAD^½C/√π × (√4 - √1) = factor × 1.0
        assert!(q > 0.0);
    }

    #[test]
    fn test_deposition_controller_mass_at_time() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        let m = dc.mass_at_time(100.0);
        assert!(m > 0.0);
    }

    #[test]
    fn test_deposition_controller_total_mass() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        let total = dc.total_mass_available();
        // 1e-5 mol/cm³ × 100 cm³ × 63.546 g/mol = 0.063546 g
        assert!(approx_eq(total, 0.063546, 1e-4));
    }

    #[test]
    fn test_deposition_controller_fraction_deposited() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0);
        let f = dc.fraction_deposited(100.0);
        assert!(f >= 0.0 && f <= 1.0);
    }

    #[test]
    fn test_deposition_controller_with_threshold() {
        let dc = DepositionController::new(2, 1.0, 1e-5, 1e-5, 63.546, 100.0)
            .with_completion_threshold(0.01);
        assert!(approx_eq(dc.completion_threshold, 0.01, TOL));
    }

    // =======================================================================
    // CurrentEfficiency
    // =======================================================================

    #[test]
    fn test_current_efficiency_100_percent() {
        let fl = FaradayLaw::new(63.546, 2);
        let theo = fl.mass_deposited(1.0, 3600.0);
        let ce = CurrentEfficiency::new(63.546, 2, 1.0, 3600.0, theo);
        assert!(approx_eq(ce.efficiency(), 1.0, 1e-8));
    }

    #[test]
    fn test_current_efficiency_50_percent() {
        let fl = FaradayLaw::new(63.546, 2);
        let theo = fl.mass_deposited(1.0, 3600.0);
        let ce = CurrentEfficiency::new(63.546, 2, 1.0, 3600.0, theo * 0.5);
        assert!(approx_eq(ce.efficiency(), 0.5, 1e-6));
        assert!(approx_eq(ce.efficiency_percent(), 50.0, 1e-4));
    }

    #[test]
    fn test_current_efficiency_theoretical_mass() {
        let ce = CurrentEfficiency::new(63.546, 2, 1.0, 3600.0, 1.0);
        let theo = ce.theoretical_mass();
        assert!(approx_eq(theo, 63.546 * 3600.0 / (2.0 * FARADAY), 1e-4));
    }

    #[test]
    fn test_current_efficiency_side_reaction_charge() {
        let fl = FaradayLaw::new(63.546, 2);
        let theo = fl.mass_deposited(1.0, 3600.0);
        let ce = CurrentEfficiency::new(63.546, 2, 1.0, 3600.0, theo * 0.8);
        let side_q = ce.side_reaction_charge();
        assert!(side_q > 0.0);
        assert!(approx_eq(side_q, 0.2 * 3600.0, 1.0)); // 20% of total charge
    }

    #[test]
    fn test_current_efficiency_h2_mass() {
        let ce = CurrentEfficiency::new(63.546, 2, 1.0, 3600.0, 0.0); // 0% efficiency
        let h2_mass = ce.h2_evolution_mass();
        assert!(h2_mass > 0.0);
    }

    #[test]
    fn test_efficiency_vs_cd_optimal() {
        let eff = CurrentEfficiency::efficiency_vs_cd(5.0, 5.0);
        assert!(approx_eq(eff, 1.0, TOL));
    }

    #[test]
    fn test_efficiency_vs_cd_off_optimal() {
        let eff_opt = CurrentEfficiency::efficiency_vs_cd(5.0, 5.0);
        let eff_off = CurrentEfficiency::efficiency_vs_cd(10.0, 5.0);
        assert!(eff_off < eff_opt);
    }

    // =======================================================================
    // OverpotentialModel
    // =======================================================================

    #[test]
    fn test_bv_zero_overpotential() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let i = bv.current_density(0.0);
        assert!(approx_eq(i, 0.0, 1e-12));
    }

    #[test]
    fn test_bv_positive_anodic() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let i = bv.current_density(0.1);
        assert!(i > 0.0);
    }

    #[test]
    fn test_bv_negative_cathodic() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let i = bv.current_density(-0.1);
        assert!(i < 0.0);
    }

    #[test]
    fn test_bv_tafel_anodic_approximation() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        // At large positive η, BV ≈ Tafel anodic
        let eta = 0.3;
        let i_bv = bv.current_density(eta);
        let i_tafel = bv.tafel_anodic(eta);
        // Should be close since cathodic term is negligible
        assert!(approx_eq(i_bv / i_tafel, 1.0, 0.01));
    }

    #[test]
    fn test_bv_linearized_small_eta() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let eta = 0.001; // 1 mV, very small
        let i_bv = bv.current_density(eta);
        let i_lin = bv.linearized_current(eta);
        assert!(approx_eq(i_bv, i_lin, 1e-6));
    }

    #[test]
    fn test_bv_tafel_slope() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let slope = bv.tafel_slope_anodic();
        // b = 2.303 × RT / (0.5 × F) ≈ 0.1183 V/decade at 25°C
        assert!(approx_eq(slope, 0.1183, 0.001));
    }

    #[test]
    fn test_bv_charge_transfer_resistance() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let rct = bv.charge_transfer_resistance();
        // Rct = RT/(nF × i₀) = 8.314*298.15/(1.0*96485*0.01) ≈ 2.569 Ω
        assert!(approx_eq(rct, 2.569, 0.01));
    }

    #[test]
    fn test_bv_eta_for_current() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let eta = bv.eta_for_current(0.01);
        assert!(approx_eq(eta, 0.0, TOL));
    }

    #[test]
    fn test_bv_polarization_curve() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5);
        let curve = bv.polarization_curve(-0.2, 0.2, 41);
        assert_eq!(curve.len(), 41);
        // Current should be negative at negative η and positive at positive η
        assert!(curve[0].1 < 0.0);
        assert!(curve[40].1 > 0.0);
    }

    #[test]
    fn test_bv_with_temperature() {
        let bv = OverpotentialModel::new(0.01, 0.5, 0.5).with_temperature(350.0);
        assert!(approx_eq(bv.temperature_k, 350.0, TOL));
    }

    // =======================================================================
    // SelectiveSeparation
    // =======================================================================

    #[test]
    fn test_selective_separation_deposition_order() {
        let species = vec![
            MetalSpecies::new("Zn", E0_ZN, 2, 0.1, 65.38),
            MetalSpecies::new("Cu", E0_CU, 2, 0.1, 63.546),
            MetalSpecies::new("Ni", E0_NI, 2, 0.1, 58.693),
        ];
        let ss = SelectiveSeparation::new(species);
        let order = ss.deposition_order();
        assert_eq!(order[0].name, "Cu");
        assert_eq!(order[1].name, "Ni");
        assert_eq!(order[2].name, "Zn");
    }

    #[test]
    fn test_selective_separation_potential_window() {
        let species = vec![
            MetalSpecies::new("Cu", E0_CU, 2, 0.1, 63.546),
            MetalSpecies::new("Ni", E0_NI, 2, 0.1, 58.693),
        ];
        let ss = SelectiveSeparation::new(species);
        let (e_start, e_stop) = ss.potential_window(0).unwrap();
        // Cu deposits first (more positive), window extends to Ni onset
        assert!(e_start > e_stop);
    }

    #[test]
    fn test_selective_separation_factor() {
        let species = vec![
            MetalSpecies::new("Cu", E0_CU, 2, 0.1, 63.546),
            MetalSpecies::new("Ni", E0_NI, 2, 0.1, 58.693),
        ];
        let ss = SelectiveSeparation::new(species);
        let sf = ss.separation_factor(0, 1);
        // |0.34 - (-0.26)| + Nernst correction (both at 0.1 M)
        // Both at same concentration, so corrections are similar
        assert!(sf > 0.5); // should be around 0.6 V
    }

    #[test]
    fn test_selective_separation_can_separate() {
        let species = vec![
            MetalSpecies::new("Cu", E0_CU, 2, 0.1, 63.546),
            MetalSpecies::new("Ni", E0_NI, 2, 0.1, 58.693),
        ];
        let ss = SelectiveSeparation::new(species);
        assert!(ss.can_separate(0, 1, 0.2)); // easily separated
        assert!(!ss.can_separate(0, 1, 5.0)); // impossible
    }

    #[test]
    fn test_selective_separation_total_potential() {
        let species = vec![
            MetalSpecies::new("Cu", E0_CU, 2, 0.1, 63.546),
            MetalSpecies::new("Zn", E0_ZN, 2, 0.1, 65.38),
        ];
        let ss = SelectiveSeparation::new(species);
        let e_total = ss.total_deposition_potential();
        // Should be near Zn potential (most negative)
        assert!(e_total < -0.5);
    }

    #[test]
    fn test_selective_separation_residual_concentration() {
        let species = vec![
            MetalSpecies::new("Cu", E0_CU, 2, 0.1, 63.546),
        ];
        let ss = SelectiveSeparation::new(species);
        // At a very negative potential, concentration should be very small
        let c_res = ss.residual_concentration(0, -0.5);
        assert!(c_res < 1e-20);
    }

    #[test]
    fn test_metal_species_nernst() {
        let cu = MetalSpecies::new("Cu", E0_CU, 2, 1.0, 63.546);
        assert!(approx_eq(cu.nernst(STD_TEMP_K), E0_CU, TOL));
    }

    // =======================================================================
    // MassTransport
    // =======================================================================

    #[test]
    fn test_mass_transport_limiting_current_stagnant() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5);
        let il = mt.limiting_current_stagnant();
        // iL = 2 × 96485 × 1.0 × 1e-5 × 1e-5 / 0.05
        let expected = 2.0 * FARADAY * 1.0 * 1e-5 * 1e-5 / 0.05;
        assert!(approx_eq(il, expected, 1e-8));
    }

    #[test]
    fn test_mass_transport_rde_increases_with_omega() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5);
        let il1 = mt.limiting_current_rde(100.0);
        let il2 = mt.limiting_current_rde(400.0);
        assert!(il2 > il1);
        assert!(approx_eq(il2 / il1, 2.0, 0.01));
    }

    #[test]
    fn test_mass_transport_rde_diffusion_layer() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5);
        let delta = mt.rde_diffusion_layer(100.0);
        assert!(delta > 0.0);
        // Higher rotation should give thinner layer
        let delta2 = mt.rde_diffusion_layer(400.0);
        assert!(delta2 < delta);
    }

    #[test]
    fn test_mass_transport_cottrell() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5);
        let i = mt.cottrell_at(1.0);
        assert!(i > 0.0);
    }

    #[test]
    fn test_mass_transport_kinetic_current() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5);
        let i_measured = 0.8;
        let i_limiting = 1.0;
        let i_k = mt.kinetic_current(i_measured, i_limiting);
        // 1/i = 1/ik + 1/il → ik = i*il/(il-i) = 0.8*1/(1-0.8) = 4.0
        assert!(approx_eq(i_k, 4.0, TOL));
    }

    #[test]
    fn test_mass_transport_mass_flux() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5);
        let flux = mt.mass_flux();
        // D*C/δ = 1e-5 * 1e-5 / 0.05 = 2e-9
        assert!(approx_eq(flux, 2e-9, 1e-12));
    }

    #[test]
    fn test_mass_transport_reynolds() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5);
        let re = mt.reynolds_rde(100.0, 0.5);
        // Re = ω × r² / ν = 100 × 0.25 / 0.01 = 2500
        assert!(approx_eq(re, 2500.0, TOL));
    }

    #[test]
    fn test_mass_transport_with_delta() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5).with_delta(0.01);
        assert!(approx_eq(mt.delta_cm, 0.01, TOL));
    }

    #[test]
    fn test_mass_transport_with_viscosity() {
        let mt = MassTransport::new(2, 1.0, 1e-5, 1e-5).with_viscosity(0.02);
        assert!(approx_eq(mt.kinematic_visc, 0.02, TOL));
    }

    // =======================================================================
    // DepositQuality
    // =======================================================================

    #[test]
    fn test_deposit_quality_smooth() {
        let dq = DepositQuality::new(10.0);
        assert_eq!(dq.classify(1.0), DepositMorphology::Smooth);
    }

    #[test]
    fn test_deposit_quality_fine_grained() {
        let dq = DepositQuality::new(10.0);
        assert_eq!(dq.classify(3.0), DepositMorphology::FineGrained);
    }

    #[test]
    fn test_deposit_quality_nodular() {
        let dq = DepositQuality::new(10.0);
        assert_eq!(dq.classify(6.0), DepositMorphology::Nodular);
    }

    #[test]
    fn test_deposit_quality_dendritic() {
        let dq = DepositQuality::new(10.0);
        assert_eq!(dq.classify(9.0), DepositMorphology::Dendritic);
    }

    #[test]
    fn test_deposit_quality_powdery() {
        let dq = DepositQuality::new(10.0);
        assert_eq!(dq.classify(15.0), DepositMorphology::Powdery);
    }

    #[test]
    fn test_throwing_power_equal() {
        // Equal distribution → high throwing power
        let tp = DepositQuality::throwing_power(5.0, 1.0);
        assert!(tp > 0.0);
    }

    #[test]
    fn test_throwing_power_perfect() {
        // When K = 1 (same mass at near/far), TP = (M-1)/(M-1) × 100 = 100
        let tp = DepositQuality::throwing_power(5.0, 1.0);
        // (5-1)/(5+1-2) = 4/4 = 1 → 100%
        assert!(approx_eq(tp, 100.0, TOL));
    }

    #[test]
    fn test_hull_cell_cd_distribution() {
        let dq = DepositQuality::new(10.0);
        let cd_near = dq.hull_cell_cd(0.1, 1.0);
        let cd_far = dq.hull_cell_cd(1.0, 1.0);
        assert!(cd_near > cd_far);
    }

    #[test]
    fn test_wagner_number() {
        let wa = DepositQuality::wagner_number(10.0, 0.1, 5.0);
        // 10 × 0.1 / 5 = 0.2
        assert!(approx_eq(wa, 0.2, TOL));
    }

    #[test]
    fn test_grain_size_decreases_with_cd() {
        let dq = DepositQuality::new(10.0);
        let gs1 = dq.grain_size_um(1.0);
        let gs2 = dq.grain_size_um(4.0);
        assert!(gs2 < gs1);
    }

    // =======================================================================
    // CoulometricTitration
    // =======================================================================

    #[test]
    fn test_coulometric_titration_add_point() {
        let mut ct = CoulometricTitration::new(0.01, 63.546, 2);
        ct.add_point(10.0, 0.5);
        assert_eq!(ct.data.len(), 1);
        assert!(approx_eq(ct.data[0].charge_c, 0.1, TOL));
    }

    #[test]
    fn test_coulometric_titration_simulated_curve() {
        let ct = CoulometricTitration::simulate_curve(0.01, 63.546, 2, 0.001, 100);
        assert_eq!(ct.data.len(), 100);
        // Potential should generally increase (or have a break)
        assert!(!ct.data.is_empty());
    }

    #[test]
    fn test_coulometric_titration_endpoint_detection() {
        let ct = CoulometricTitration::simulate_curve(0.01, 63.546, 2, 0.001, 200);
        let ep = ct.find_endpoint_first_derivative();
        assert!(ep.is_some());
        let idx = ep.unwrap();
        // Endpoint should be found (max derivative near equivalence or in linear ramp)
        assert!(idx > 0 && idx < 199);
    }

    #[test]
    fn test_coulometric_titration_mass_at_endpoint() {
        let ct = CoulometricTitration::simulate_curve(0.01, 63.546, 2, 0.001, 200);
        let mass = ct.mass_at_endpoint();
        assert!(mass.is_some());
        let m = mass.unwrap();
        // Should be roughly the actual mass (within some error due to discrete sampling)
        assert!(m > 0.0);
    }

    #[test]
    fn test_coulometric_titration_moles_at_endpoint() {
        let ct = CoulometricTitration::simulate_curve(0.01, 107.868, 1, 0.01, 200);
        let moles = ct.moles_at_endpoint();
        assert!(moles.is_some());
        assert!(moles.unwrap() > 0.0);
    }

    #[test]
    fn test_coulometric_titration_charge_at_endpoint() {
        let ct = CoulometricTitration::simulate_curve(0.01, 63.546, 2, 0.001, 200);
        let q = ct.charge_at_endpoint();
        assert!(q.is_some());
        assert!(q.unwrap() > 0.0);
    }

    #[test]
    fn test_coulometric_titration_concentration() {
        let ct = CoulometricTitration::simulate_curve(0.01, 63.546, 2, 0.001, 200);
        let conc = ct.concentration_at_endpoint(0.05); // 50 mL
        assert!(conc.is_some());
        assert!(conc.unwrap() > 0.0);
    }

    #[test]
    fn test_karl_fischer_water() {
        // 1 Faraday of charge → 1 mole I₂ (2e⁻) → 1 mole H₂O = 18.015 g
        let charge = 2.0 * FARADAY; // 2 Faradays → still 1 mol (n=2)
        let water_ug = CoulometricTitration::karl_fischer_water_ug(charge);
        // Should be 18.015 g = 18015000 μg
        assert!(approx_eq(water_ug, 18.015e6, 1e3));
    }

    #[test]
    fn test_coulometric_second_derivative_endpoint() {
        let ct = CoulometricTitration::simulate_curve(0.01, 63.546, 2, 0.001, 200);
        let ep = ct.find_endpoint_second_derivative();
        // May or may not find it depending on curve shape, but should not panic
        if let Some(idx) = ep {
            assert!(idx > 0 && idx < ct.data.len());
        }
    }

    // =======================================================================
    // PlatingBath
    // =======================================================================

    #[test]
    fn test_plating_bath_copper_sulfate() {
        let bath = PlatingBath::copper_sulfate();
        assert!(approx_eq(bath.metal_conc, 0.25, TOL));
        assert!(bath.ph < 2.0); // acidic
    }

    #[test]
    fn test_plating_bath_nickel_watts() {
        let bath = PlatingBath::nickel_watts();
        assert!(approx_eq(bath.metal_conc, 1.0, TOL));
        assert!(approx_eq(bath.ph, 4.0, TOL));
        assert!(bath.temperature_k > 320.0); // elevated temperature
    }

    #[test]
    fn test_h2_evolution_potential() {
        let bath = PlatingBath::new(0.1, 0.0); // pH=0
        let e = bath.h2_evolution_potential();
        // At pH 0: E ≈ 0 V (SHE)
        assert!(approx_eq(e, 0.0, 0.01));
    }

    #[test]
    fn test_o2_evolution_potential() {
        let bath = PlatingBath::new(0.1, 0.0); // pH=0
        let e = bath.o2_evolution_potential();
        // At pH 0: E ≈ 1.229 V
        assert!(approx_eq(e, 1.229, 0.01));
    }

    #[test]
    fn test_diffusion_layer_varies_with_agitation() {
        let stagnant = PlatingBath::new(0.1, 4.0);
        let mut stirred = PlatingBath::new(0.1, 4.0);
        stirred.agitation = 1.0;
        assert!(stirred.diffusion_layer_cm() < stagnant.diffusion_layer_cm());
    }

    #[test]
    fn test_plating_bath_with_temperature() {
        let bath = PlatingBath::new(0.1, 4.0).with_temperature(350.0);
        assert!(approx_eq(bath.temperature_k, 350.0, TOL));
    }

    // =======================================================================
    // PulsePlating
    // =======================================================================

    #[test]
    fn test_pulse_plating_duty_cycle() {
        let pp = PulsePlating::new(10.0, 0.01, 0.04);
        assert!(approx_eq(pp.duty_cycle(), 0.2, TOL));
    }

    #[test]
    fn test_pulse_plating_average_cd() {
        let pp = PulsePlating::new(10.0, 0.01, 0.04);
        assert!(approx_eq(pp.average_cd(), 2.0, TOL));
    }

    #[test]
    fn test_pulse_plating_total_time() {
        let pp = PulsePlating::new(10.0, 0.01, 0.04).with_cycles(100);
        assert!(approx_eq(pp.total_time(), 5.0, TOL));
    }

    #[test]
    fn test_pulse_plating_frequency() {
        let pp = PulsePlating::new(10.0, 0.01, 0.04);
        assert!(approx_eq(pp.frequency_hz(), 20.0, TOL));
    }

    #[test]
    fn test_pulse_plating_waveform() {
        let pp = PulsePlating::new(10.0, 0.01, 0.04);
        let wf = pp.waveform_one_period(50);
        assert_eq!(wf.len(), 50);
        // First ~20% should be at peak_cd, rest at 0
        assert!(approx_eq(wf[0].1, 10.0, TOL));
        assert!(approx_eq(wf[49].1, 0.0, TOL));
    }

    #[test]
    fn test_pulse_plating_charge_per_cycle() {
        let pp = PulsePlating::new(10.0, 0.01, 0.04);
        let q = pp.charge_per_cycle(1.0);
        // 10 A/cm² × 1 cm² × 0.01 s = 0.1 C
        assert!(approx_eq(q, 0.1, TOL));
    }

    #[test]
    fn test_pulse_plating_total_charge() {
        let pp = PulsePlating::new(10.0, 0.01, 0.04).with_cycles(100);
        let q = pp.total_charge(1.0);
        assert!(approx_eq(q, 10.0, TOL));
    }
}

//! # Muon-Catalyzed Fusion Diagnostics
//!
//! Signal processing and analysis for muon-catalyzed fusion (muCF) experiments.
//! This module provides tools for modeling muon decay, tracking fusion cycles,
//! calculating sticking probabilities, detecting fusion products (neutrons, X-rays),
//! estimating cycling rates, and analyzing energy breakeven conditions.
//!
//! # Physics Background
//!
//! Muon-catalyzed fusion (muCF) exploits the fact that a negative muon (mu-) is
//! ~207 times heavier than an electron. When a muon replaces an electron in a
//! hydrogen isotope molecule, the resulting muonic molecule is ~207 times smaller,
//! bringing the nuclei close enough for quantum tunneling to cause fusion at
//! room temperature.
//!
//! The muCF cycle proceeds as follows:
//! 1. A negative muon is injected into a D-T mixture
//! 2. The muon is captured by a deuterium or tritium atom, forming a muonic atom
//! 3. The muonic atom collides with another hydrogen isotope, forming a muonic
//!    molecule (dt-mu) via the Vesman resonance mechanism
//! 4. The nuclei fuse, releasing energy (17.6 MeV for d-t)
//! 5. The muon is usually released and can catalyze another fusion
//! 6. With probability alpha_s (~0.5% for d-t), the muon sticks to the alpha
//!    particle and is lost from the cycle
//!
//! The key limitation is alpha-sticking: even at 0.5%, after ~150 cycles the
//! muon is lost or decays (lifetime 2.197 us). Breakeven requires ~300 fusions
//! per muon given the ~5-10 GeV production cost.
//!
//! # Key Parameters
//!
//! - Muon mass: 105.658 MeV/c^2, lifetime: 2.197 us
//! - d-t fusion Q-value: 17.6 MeV (14.1 MeV neutron + 3.5 MeV alpha)
//! - d-d fusion Q-value: 3.27 MeV (n + He-3) or 4.03 MeV (p + T)
//! - Alpha sticking: ~0.5% for d-t (limits cycling to ~150 fusions/muon)
//! - Muon production cost: ~5-10 GeV per muon (from pion decay)
//! - Muonic atom X-rays: E scales as m_mu/m_e ~ 207 relative to electronic atoms
//!
//! # Example
//!
//! ```rust
//! use r4w_core::muon_catalyzed_fusion_diagnostics::*;
//!
//! let config = MuonConfig::dt_standard();
//! let decay = MuonDecayModel::new(config.clone());
//!
//! // Muon population at one lifetime
//! let n = decay.population(MUON_LIFETIME_US * 1e-6);
//! assert!((n - 1.0 / std::f64::consts::E).abs() < 1e-6);
//!
//! // Alpha sticking limits effective cycles
//! let sticking = StickingCalculator::new(config.clone());
//! let n_eff = sticking.effective_cycles();
//! assert!(n_eff > 100.0 && n_eff < 300.0);
//!
//! // Energy yield analysis
//! let yield_calc = EnergyYieldCalculator::new(config);
//! let e_net = yield_calc.net_energy_mev(150.0);
//! assert!(e_net > 0.0); // 150 fusions * 17.6 MeV - production cost
//! ```

// No external dependencies needed - pure Rust with std only.

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Muon rest mass in MeV/c^2.
pub const MUON_MASS_MEV: f64 = 105.658_3755;

/// Muon rest mass in kg.
pub const MUON_MASS_KG: f64 = 1.883_531_627e-28;

/// Muon lifetime in microseconds.
pub const MUON_LIFETIME_US: f64 = 2.196_9811;

/// Muon lifetime in seconds.
pub const MUON_LIFETIME_S: f64 = 2.196_9811e-6;

/// Electron mass in MeV/c^2.
pub const ELECTRON_MASS_MEV: f64 = 0.510_998_950;

/// Muon-to-electron mass ratio.
pub const MUON_ELECTRON_MASS_RATIO: f64 = 206.768_2830;

/// Proton mass in MeV/c^2.
pub const PROTON_MASS_MEV: f64 = 938.272_088;

/// Neutron mass in MeV/c^2.
pub const NEUTRON_MASS_MEV: f64 = 939.565_420;

/// Deuteron mass in MeV/c^2.
pub const DEUTERON_MASS_MEV: f64 = 1875.612_93;

/// Triton mass in MeV/c^2.
pub const TRITON_MASS_MEV: f64 = 2808.921_12;

/// Helium-3 mass in MeV/c^2.
pub const HELIUM3_MASS_MEV: f64 = 2808.391_60;

/// Alpha particle mass in MeV/c^2.
pub const ALPHA_MASS_MEV: f64 = 3727.379_41;

/// d-t fusion Q-value in MeV: d + t -> He-4 + n.
pub const DT_Q_VALUE_MEV: f64 = 17.588;

/// d-t neutron energy in MeV.
pub const DT_NEUTRON_ENERGY_MEV: f64 = 14.1;

/// d-t alpha energy in MeV.
pub const DT_ALPHA_ENERGY_MEV: f64 = 3.5;

/// d-d fusion Q-value (n + He-3 branch) in MeV.
pub const DD_N_HE3_Q_VALUE_MEV: f64 = 3.269;

/// d-d neutron energy (n + He-3 branch) in MeV.
pub const DD_NEUTRON_ENERGY_MEV: f64 = 2.45;

/// d-d fusion Q-value (p + T branch) in MeV.
pub const DD_P_T_Q_VALUE_MEV: f64 = 4.033;

/// p-d fusion Q-value (He-3 + gamma) in MeV.
pub const PD_Q_VALUE_MEV: f64 = 5.493;

/// Hydrogen ground state energy in eV (Rydberg).
pub const HYDROGEN_GROUND_STATE_EV: f64 = 13.6;

/// Boltzmann constant in eV/K.
pub const KB_EV_PER_K: f64 = 8.617_333_262e-5;

/// Speed of light in m/s.
pub const C_M_PER_S: f64 = 2.997_924_58e8;

/// Default alpha-sticking probability for d-t fusion.
pub const DEFAULT_DT_ALPHA_STICKING: f64 = 0.005;

/// Default alpha-sticking probability for d-d fusion.
pub const DEFAULT_DD_ALPHA_STICKING: f64 = 0.12;

/// Typical muon production cost in GeV.
pub const MUON_PRODUCTION_COST_GEV: f64 = 5.0;

// ---------------------------------------------------------------------------
// Target composition
// ---------------------------------------------------------------------------

/// Target mixture composition for muCF experiments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TargetComposition {
    /// Deuterium-Tritium mixture with specified tritium fraction (0..1).
    DeuteriumTritium(f64),
    /// Pure Deuterium-Deuterium.
    DeuteriumDeuterium,
    /// Proton-Deuterium mixture with specified deuterium fraction (0..1).
    ProtonDeuterium(f64),
}

impl TargetComposition {
    /// Returns the primary fusion Q-value in MeV for this target.
    pub fn q_value_mev(&self) -> f64 {
        match self {
            TargetComposition::DeuteriumTritium(_) => DT_Q_VALUE_MEV,
            TargetComposition::DeuteriumDeuterium => {
                // Average of the two branches (equal probability)
                0.5 * (DD_N_HE3_Q_VALUE_MEV + DD_P_T_Q_VALUE_MEV)
            }
            TargetComposition::ProtonDeuterium(_) => PD_Q_VALUE_MEV,
        }
    }

    /// Returns the default alpha-sticking probability for this target.
    pub fn default_sticking(&self) -> f64 {
        match self {
            TargetComposition::DeuteriumTritium(_) => DEFAULT_DT_ALPHA_STICKING,
            TargetComposition::DeuteriumDeuterium => DEFAULT_DD_ALPHA_STICKING,
            TargetComposition::ProtonDeuterium(_) => 0.0, // No alpha produced
        }
    }

    /// Returns the primary neutron energy in MeV (0 if no neutron branch).
    pub fn neutron_energy_mev(&self) -> f64 {
        match self {
            TargetComposition::DeuteriumTritium(_) => DT_NEUTRON_ENERGY_MEV,
            TargetComposition::DeuteriumDeuterium => DD_NEUTRON_ENERGY_MEV,
            TargetComposition::ProtonDeuterium(_) => 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// MuonConfig
// ---------------------------------------------------------------------------

/// Configuration for a muon-catalyzed fusion experiment.
#[derive(Debug, Clone)]
pub struct MuonConfig {
    /// Muon lifetime in seconds (default: 2.197 us).
    pub muon_lifetime_s: f64,
    /// Target composition.
    pub target: TargetComposition,
    /// Target temperature in Kelvin.
    pub temperature_k: f64,
    /// Target density in atoms/cm^3.
    pub density_per_cm3: f64,
    /// Alpha-sticking probability (0..1). If None, uses default for target.
    pub alpha_sticking: Option<f64>,
    /// Muon production cost in GeV.
    pub muon_production_cost_gev: f64,
    /// Molecular formation rate lambda_f in 1/s. If None, uses estimate.
    pub molecular_formation_rate: Option<f64>,
}

impl MuonConfig {
    /// Standard d-t configuration at liquid hydrogen density.
    pub fn dt_standard() -> Self {
        Self {
            muon_lifetime_s: MUON_LIFETIME_S,
            target: TargetComposition::DeuteriumTritium(0.5),
            temperature_k: 300.0,
            density_per_cm3: 4.25e22, // liquid hydrogen density
            alpha_sticking: None,
            muon_production_cost_gev: MUON_PRODUCTION_COST_GEV,
            molecular_formation_rate: None,
        }
    }

    /// Standard d-d configuration.
    pub fn dd_standard() -> Self {
        Self {
            muon_lifetime_s: MUON_LIFETIME_S,
            target: TargetComposition::DeuteriumDeuterium,
            temperature_k: 300.0,
            density_per_cm3: 4.25e22,
            alpha_sticking: None,
            muon_production_cost_gev: MUON_PRODUCTION_COST_GEV,
            molecular_formation_rate: None,
        }
    }

    /// Standard p-d configuration.
    pub fn pd_standard() -> Self {
        Self {
            muon_lifetime_s: MUON_LIFETIME_S,
            target: TargetComposition::ProtonDeuterium(0.5),
            temperature_k: 300.0,
            density_per_cm3: 4.25e22,
            alpha_sticking: None,
            muon_production_cost_gev: MUON_PRODUCTION_COST_GEV,
            molecular_formation_rate: None,
        }
    }

    /// Returns the effective alpha-sticking probability.
    pub fn effective_sticking(&self) -> f64 {
        self.alpha_sticking.unwrap_or_else(|| self.target.default_sticking())
    }
}

// ---------------------------------------------------------------------------
// MuonDecayModel
// ---------------------------------------------------------------------------

/// Models muon decay: N(t) = N0 * exp(-t / tau_mu).
///
/// Also provides the Michel spectrum for decay electron energies.
/// The muon decays via: mu- -> e- + nu_mu_bar + nu_e
/// The Michel spectrum gives the energy distribution of the decay electron.
#[derive(Debug, Clone)]
pub struct MuonDecayModel {
    config: MuonConfig,
}

impl MuonDecayModel {
    /// Create a new decay model from configuration.
    pub fn new(config: MuonConfig) -> Self {
        Self { config }
    }

    /// Fractional population remaining at time t (seconds).
    /// N(t)/N0 = exp(-t / tau_mu)
    pub fn population(&self, t_seconds: f64) -> f64 {
        (-t_seconds / self.config.muon_lifetime_s).exp()
    }

    /// Decay rate (probability per second) = 1/tau.
    pub fn decay_rate(&self) -> f64 {
        1.0 / self.config.muon_lifetime_s
    }

    /// Number of muons remaining from initial population N0 at time t.
    pub fn remaining(&self, n0: f64, t_seconds: f64) -> f64 {
        n0 * self.population(t_seconds)
    }

    /// Half-life in seconds: t_1/2 = tau * ln(2).
    pub fn half_life_s(&self) -> f64 {
        self.config.muon_lifetime_s * 2.0_f64.ln()
    }

    /// Michel spectrum: differential decay rate dN/dx as a function of
    /// reduced electron energy x = E_e / E_max, where E_max = m_mu/2.
    ///
    /// For V-A interaction (Standard Model):
    /// dN/dx = 2x^2 * (3 - 2x) for 0 <= x <= 1
    ///
    /// This is the normalized probability density.
    pub fn michel_spectrum(&self, x: f64) -> f64 {
        if x < 0.0 || x > 1.0 {
            return 0.0;
        }
        2.0 * x * x * (3.0 - 2.0 * x)
    }

    /// Maximum electron energy from muon decay in MeV.
    /// E_max = m_mu / 2 (in the muon rest frame).
    pub fn max_electron_energy_mev(&self) -> f64 {
        MUON_MASS_MEV / 2.0
    }

    /// Mean electron energy from muon decay in MeV.
    /// <E> = (3/8) * m_mu (from integrating x * Michel spectrum).
    /// Actually: <x> = integral(x * 2x^2(3-2x) dx, 0..1)
    ///         = integral(6x^3 - 4x^4 dx, 0..1) = 6/4 - 4/5 = 3/2 - 4/5 = 7/10
    /// So <E> = 0.7 * E_max = 0.7 * m_mu/2 = 0.35 * m_mu
    pub fn mean_electron_energy_mev(&self) -> f64 {
        0.35 * MUON_MASS_MEV
    }

    /// Generate a time spectrum of muon decays for a given time window.
    /// Returns an array of (time_us, decay_rate) pairs.
    /// The decay rate is proportional to exp(-t/tau) / tau.
    pub fn time_spectrum(&self, t_start_us: f64, t_end_us: f64, n_points: usize) -> Vec<(f64, f64)> {
        if n_points == 0 || t_end_us <= t_start_us {
            return Vec::new();
        }
        let dt = (t_end_us - t_start_us) / n_points as f64;
        let tau_us = self.config.muon_lifetime_s * 1e6;
        (0..n_points)
            .map(|i| {
                let t = t_start_us + (i as f64 + 0.5) * dt;
                let rate = (-t / tau_us).exp() / tau_us;
                (t, rate)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// FusionCycleTracker
// ---------------------------------------------------------------------------

/// Tracks the muon cycling rate through fusion cycles.
///
/// Each cycle consists of:
/// 1. Muonic atom formation (rate lambda_a)
/// 2. Molecular formation (rate lambda_f) -- often the rate-limiting step
/// 3. Nuclear fusion (rate lambda_fus) -- very fast once molecule forms
/// 4. Muon release or sticking (probability alpha_s of loss)
///
/// The overall cycling rate is:
/// lambda_c = 1 / (1/lambda_a + 1/lambda_f + 1/lambda_fus)
/// Effective rate accounting for sticking and decay:
/// n_fusions = lambda_c * tau_mu / (1 + alpha_s * lambda_c * tau_mu)
#[derive(Debug, Clone)]
pub struct FusionCycleTracker {
    config: MuonConfig,
    /// Muonic atom formation rate (1/s).
    lambda_a: f64,
    /// Molecular formation rate (1/s).
    lambda_f: f64,
    /// Nuclear fusion rate (1/s).
    lambda_fus: f64,
    /// Number of completed cycles.
    cycles_completed: u64,
    /// Whether the muon is still active.
    muon_active: bool,
}

impl FusionCycleTracker {
    /// Create a new cycle tracker.
    ///
    /// Default rates are order-of-magnitude estimates for d-t at LHD:
    /// - Atomic formation: ~10^12/s (very fast)
    /// - Molecular formation: ~10^8/s (rate-limiting, Vesman resonance)
    /// - Fusion: ~10^12/s (very fast once molecule forms)
    pub fn new(config: MuonConfig) -> Self {
        let lambda_f = config.molecular_formation_rate.unwrap_or(1.0e8);
        Self {
            config,
            lambda_a: 1.0e12,
            lambda_f,
            lambda_fus: 1.0e12,
            cycles_completed: 0,
            muon_active: true,
        }
    }

    /// Set custom formation rates.
    pub fn with_rates(mut self, lambda_a: f64, lambda_f: f64, lambda_fus: f64) -> Self {
        self.lambda_a = lambda_a;
        self.lambda_f = lambda_f;
        self.lambda_fus = lambda_fus;
        self
    }

    /// Overall cycling rate (1/s).
    /// lambda_c = 1 / (1/lambda_a + 1/lambda_f + 1/lambda_fus)
    pub fn cycling_rate(&self) -> f64 {
        let inv = 1.0 / self.lambda_a + 1.0 / self.lambda_f + 1.0 / self.lambda_fus;
        1.0 / inv
    }

    /// Cycle time (seconds per fusion cycle).
    pub fn cycle_time_s(&self) -> f64 {
        1.0 / self.cycling_rate()
    }

    /// Expected number of fusions per muon, accounting for sticking and decay.
    /// n_fusions = lambda_c * tau / (1 + alpha_s * lambda_c * tau)
    /// This is derived from the geometric series of survival probabilities.
    pub fn expected_fusions(&self) -> f64 {
        let lambda_c = self.cycling_rate();
        let tau = self.config.muon_lifetime_s;
        let alpha_s = self.config.effective_sticking();
        let lambda_d = 1.0 / tau; // decay rate

        // Effective loss rate per cycle time
        // Probability of surviving one cycle: (1 - alpha_s) * exp(-t_cycle / tau)
        // For small t_cycle/tau: effective loss rate = alpha_s * lambda_c + lambda_d
        // Expected fusions = lambda_c / (alpha_s * lambda_c + lambda_d)
        lambda_c / (alpha_s * lambda_c + lambda_d)
    }

    /// Simulate one cycle. Returns true if muon survives (continues cycling),
    /// false if muon is lost (sticking or decay).
    /// Uses a simple deterministic model based on expected probabilities.
    pub fn step_cycle(&mut self) -> bool {
        if !self.muon_active {
            return false;
        }

        let alpha_s = self.config.effective_sticking();
        let cycle_time = self.cycle_time_s();
        let decay_prob = 1.0 - (-cycle_time / self.config.muon_lifetime_s).exp();

        // Combined loss probability per cycle
        let loss_prob = alpha_s + decay_prob * (1.0 - alpha_s);

        // Deterministic: use fractional cycle counting
        // For simulation, track whether we exceed expected cycles
        let expected = self.expected_fusions();
        if (self.cycles_completed as f64) < expected {
            self.cycles_completed += 1;
            true
        } else {
            self.muon_active = false;
            false
        }
    }

    /// Reset the tracker for a new muon.
    pub fn reset(&mut self) {
        self.cycles_completed = 0;
        self.muon_active = true;
    }

    /// Number of completed fusion cycles.
    pub fn cycles(&self) -> u64 {
        self.cycles_completed
    }

    /// Whether the muon is still actively cycling.
    pub fn is_active(&self) -> bool {
        self.muon_active
    }

    /// Loss probability per cycle: alpha_s + (1 - alpha_s) * (1 - exp(-t_cycle/tau)).
    pub fn loss_probability_per_cycle(&self) -> f64 {
        let alpha_s = self.config.effective_sticking();
        let cycle_time = self.cycle_time_s();
        let decay_prob = 1.0 - (-cycle_time / self.config.muon_lifetime_s).exp();
        alpha_s + (1.0 - alpha_s) * decay_prob
    }
}

// ---------------------------------------------------------------------------
// StickingCalculator
// ---------------------------------------------------------------------------

/// Calculates alpha-sticking probability and its effect on cycling.
///
/// When d-t fusion occurs, the products are a 3.5 MeV alpha particle and a
/// 14.1 MeV neutron. The muon can stick to the alpha (He-4), removing it
/// from the catalytic cycle. The initial sticking probability omega_s^0 is
/// about 0.9%, but some muons are stripped by collisions (reactivation),
/// giving an effective sticking of ~0.5%.
///
/// The effective number of catalyzed fusions is approximately 1/alpha_s_eff.
#[derive(Debug, Clone)]
pub struct StickingCalculator {
    config: MuonConfig,
    /// Initial sticking probability before reactivation.
    initial_sticking: f64,
    /// Reactivation fraction (fraction of stuck muons that get stripped).
    reactivation_fraction: f64,
}

impl StickingCalculator {
    /// Create a new sticking calculator with default parameters.
    pub fn new(config: MuonConfig) -> Self {
        let (initial, reactivation) = match &config.target {
            TargetComposition::DeuteriumTritium(_) => (0.009, 0.44),
            TargetComposition::DeuteriumDeuterium => (0.12, 0.0),
            TargetComposition::ProtonDeuterium(_) => (0.0, 0.0),
        };
        Self {
            config,
            initial_sticking: initial,
            reactivation_fraction: reactivation,
        }
    }

    /// Set custom sticking parameters.
    pub fn with_params(mut self, initial_sticking: f64, reactivation: f64) -> Self {
        self.initial_sticking = initial_sticking;
        self.reactivation_fraction = reactivation;
        self
    }

    /// Initial sticking probability omega_s^0 (before reactivation).
    pub fn initial_sticking(&self) -> f64 {
        self.initial_sticking
    }

    /// Effective sticking probability after reactivation.
    /// alpha_s_eff = omega_s^0 * (1 - R)
    /// where R is the reactivation fraction.
    pub fn effective_sticking(&self) -> f64 {
        self.initial_sticking * (1.0 - self.reactivation_fraction)
    }

    /// Maximum number of cycles limited by sticking alone (ignoring decay).
    /// n_max = 1 / alpha_s_eff
    pub fn effective_cycles(&self) -> f64 {
        let alpha_eff = self.effective_sticking();
        if alpha_eff <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / alpha_eff
    }

    /// Probability that the muon survives n cycles without sticking.
    /// P_survive(n) = (1 - alpha_s_eff)^n
    pub fn survival_probability(&self, n_cycles: u32) -> f64 {
        let alpha_eff = self.effective_sticking();
        (1.0 - alpha_eff).powi(n_cycles as i32)
    }

    /// Temperature dependence of sticking (simple model).
    /// At higher temperatures, the alpha is faster and stripping is more effective.
    /// R(T) = R_0 * (1 + beta * (T - T_ref) / T_ref) for T > T_ref
    pub fn temperature_corrected_sticking(&self, temperature_k: f64) -> f64 {
        let t_ref = 300.0; // reference temperature
        let beta = 0.1; // temperature sensitivity coefficient
        let r_corrected = if temperature_k > t_ref {
            (self.reactivation_fraction * (1.0 + beta * (temperature_k - t_ref) / t_ref))
                .min(0.99)
        } else {
            self.reactivation_fraction * (temperature_k / t_ref).max(0.01)
        };
        self.initial_sticking * (1.0 - r_corrected)
    }
}

// ---------------------------------------------------------------------------
// NeutronDetector
// ---------------------------------------------------------------------------

/// Detects and characterizes fusion neutrons.
///
/// d-t fusion produces 14.1 MeV neutrons; d-d produces 2.45 MeV neutrons.
/// Time-of-flight (TOF) measurement over a known flight path gives energy:
/// E_n = (1/2) * m_n * (L/t)^2
///
/// This detector processes time-tagged neutron events and determines their
/// energies, allowing identification of the fusion reaction type.
#[derive(Debug, Clone)]
pub struct NeutronDetector {
    /// Flight path length in metres.
    flight_path_m: f64,
    /// Energy resolution (fractional, e.g., 0.05 = 5%).
    energy_resolution: f64,
    /// Detected events: (time_ns, energy_mev).
    events: Vec<(f64, f64)>,
    /// Detection efficiency (0..1).
    efficiency: f64,
}

impl NeutronDetector {
    /// Create a new neutron detector.
    ///
    /// * `flight_path_m` - Distance from source to detector in metres.
    /// * `energy_resolution` - Fractional energy resolution.
    pub fn new(flight_path_m: f64, energy_resolution: f64) -> Self {
        Self {
            flight_path_m,
            energy_resolution,
            events: Vec::new(),
            efficiency: 0.1, // typical ~10% for organic scintillator
        }
    }

    /// Set detection efficiency.
    pub fn with_efficiency(mut self, eff: f64) -> Self {
        self.efficiency = eff.clamp(0.0, 1.0);
        self
    }

    /// Calculate neutron energy from time-of-flight.
    /// E_n = (1/2) * m_n * (L/t)^2, converted to MeV.
    ///
    /// * `tof_ns` - Time of flight in nanoseconds.
    /// Returns energy in MeV.
    pub fn energy_from_tof(&self, tof_ns: f64) -> f64 {
        if tof_ns <= 0.0 {
            return 0.0;
        }
        let tof_s = tof_ns * 1e-9;
        let v = self.flight_path_m / tof_s; // m/s
        let neutron_mass_kg = 1.674_927_471e-27;
        let e_joules = 0.5 * neutron_mass_kg * v * v;
        let mev_per_joule = 6.242e12;
        e_joules * mev_per_joule
    }

    /// Calculate expected time-of-flight for a given neutron energy.
    /// t = L / v, where v = sqrt(2*E/m_n).
    ///
    /// Returns TOF in nanoseconds.
    pub fn tof_from_energy(&self, energy_mev: f64) -> f64 {
        if energy_mev <= 0.0 {
            return f64::INFINITY;
        }
        let e_joules = energy_mev / 6.242e12;
        let neutron_mass_kg = 1.674_927_471e-27;
        let v = (2.0 * e_joules / neutron_mass_kg).sqrt();
        let tof_s = self.flight_path_m / v;
        tof_s * 1e9
    }

    /// Record a neutron detection event.
    pub fn record_event(&mut self, tof_ns: f64) {
        let energy = self.energy_from_tof(tof_ns);
        self.events.push((tof_ns, energy));
    }

    /// Classify a neutron event as d-t (14.1 MeV) or d-d (2.45 MeV).
    /// Returns Some("d-t"), Some("d-d"), or None if unclassified.
    pub fn classify_event(&self, energy_mev: f64) -> Option<&'static str> {
        let dt_window = DT_NEUTRON_ENERGY_MEV * self.energy_resolution;
        let dd_window = DD_NEUTRON_ENERGY_MEV * self.energy_resolution;

        if (energy_mev - DT_NEUTRON_ENERGY_MEV).abs() < 3.0 * dt_window {
            Some("d-t")
        } else if (energy_mev - DD_NEUTRON_ENERGY_MEV).abs() < 3.0 * dd_window {
            Some("d-d")
        } else {
            None
        }
    }

    /// Count events by reaction type.
    pub fn count_by_type(&self) -> (usize, usize, usize) {
        let mut dt = 0;
        let mut dd = 0;
        let mut other = 0;
        for &(_, energy) in &self.events {
            match self.classify_event(energy) {
                Some("d-t") => dt += 1,
                Some("d-d") => dd += 1,
                _ => other += 1,
            }
        }
        (dt, dd, other)
    }

    /// Total number of recorded events.
    pub fn event_count(&self) -> usize {
        self.events.len()
    }

    /// Clear all recorded events.
    pub fn clear(&mut self) {
        self.events.clear();
    }

    /// Neutron velocity for a given energy (m/s).
    pub fn neutron_velocity(energy_mev: f64) -> f64 {
        let e_joules = energy_mev / 6.242e12;
        let neutron_mass_kg = 1.674_927_471e-27;
        (2.0 * e_joules / neutron_mass_kg).sqrt()
    }
}

// ---------------------------------------------------------------------------
// XraySpectrometer
// ---------------------------------------------------------------------------

/// Models muonic atom X-ray transitions.
///
/// When a muon is captured by an atom, it cascades down through energy levels
/// emitting characteristic X-rays. The energies scale as m_mu/m_e ~ 207
/// compared to electronic transitions (with reduced mass correction).
///
/// E_n = 13.6 eV * (m_reduced/m_e) * Z^2 / n^2
///
/// where m_reduced = m_mu * m_nucleus / (m_mu + m_nucleus).
/// For heavy nuclei, m_reduced ~ m_mu, so E scales as ~207 * Z^2 / n^2.
#[derive(Debug, Clone)]
pub struct XraySpectrometer {
    /// Atomic number of the target.
    z: u32,
    /// Nuclear mass in MeV/c^2 (for reduced mass calculation).
    nuclear_mass_mev: f64,
}

impl XraySpectrometer {
    /// Create a spectrometer for a given element.
    ///
    /// * `z` - Atomic number.
    /// * `nuclear_mass_mev` - Nuclear mass in MeV/c^2.
    pub fn new(z: u32, nuclear_mass_mev: f64) -> Self {
        Self { z, nuclear_mass_mev }
    }

    /// Muonic hydrogen (Z=1, proton nucleus).
    pub fn hydrogen() -> Self {
        Self::new(1, PROTON_MASS_MEV)
    }

    /// Muonic deuterium (Z=1, deuteron nucleus).
    pub fn deuterium() -> Self {
        Self::new(1, DEUTERON_MASS_MEV)
    }

    /// Muonic helium (Z=2, alpha nucleus).
    pub fn helium() -> Self {
        Self::new(2, ALPHA_MASS_MEV)
    }

    /// Reduced mass ratio m_reduced / m_e.
    /// m_reduced = m_mu * m_nuc / (m_mu + m_nuc)
    pub fn reduced_mass_ratio(&self) -> f64 {
        let m_reduced = MUON_MASS_MEV * self.nuclear_mass_mev
            / (MUON_MASS_MEV + self.nuclear_mass_mev);
        m_reduced / ELECTRON_MASS_MEV
    }

    /// Energy level of the muonic atom for principal quantum number n.
    /// E_n = -13.6 eV * (m_reduced/m_e) * Z^2 / n^2
    pub fn energy_level_ev(&self, n: u32) -> f64 {
        if n == 0 {
            return 0.0;
        }
        let ratio = self.reduced_mass_ratio();
        -HYDROGEN_GROUND_STATE_EV * ratio * (self.z as f64).powi(2) / (n as f64).powi(2)
    }

    /// Energy level in keV.
    pub fn energy_level_kev(&self, n: u32) -> f64 {
        self.energy_level_ev(n) / 1000.0
    }

    /// Transition energy between levels n_upper -> n_lower (positive, in eV).
    /// The emitted photon energy is |E(n_lower)| - |E(n_upper)|.
    /// Since energy_level_ev returns negative values (bound states),
    /// E_photon = E(n_upper) - E(n_lower) > 0.
    pub fn transition_energy_ev(&self, n_upper: u32, n_lower: u32) -> f64 {
        if n_upper <= n_lower || n_lower == 0 {
            return 0.0;
        }
        self.energy_level_ev(n_upper) - self.energy_level_ev(n_lower)
    }

    /// Transition energy in keV.
    pub fn transition_energy_kev(&self, n_upper: u32, n_lower: u32) -> f64 {
        self.transition_energy_ev(n_upper, n_lower) / 1000.0
    }

    /// K-alpha transition (2 -> 1) energy in keV.
    pub fn k_alpha_kev(&self) -> f64 {
        self.transition_energy_kev(2, 1)
    }

    /// K-beta transition (3 -> 1) energy in keV.
    pub fn k_beta_kev(&self) -> f64 {
        self.transition_energy_kev(3, 1)
    }

    /// Lyman series energies (n -> 1) for n = 2..=n_max, in keV.
    pub fn lyman_series_kev(&self, n_max: u32) -> Vec<(u32, f64)> {
        (2..=n_max)
            .map(|n| (n, self.transition_energy_kev(n, 1)))
            .collect()
    }

    /// Balmer series energies (n -> 2) for n = 3..=n_max, in keV.
    pub fn balmer_series_kev(&self, n_max: u32) -> Vec<(u32, f64)> {
        (3..=n_max)
            .map(|n| (n, self.transition_energy_kev(n, 2)))
            .collect()
    }

    /// Ground state binding energy (positive, in keV).
    pub fn ground_state_binding_kev(&self) -> f64 {
        -self.energy_level_kev(1)
    }
}

// ---------------------------------------------------------------------------
// CyclingRateEstimator
// ---------------------------------------------------------------------------

/// Fits time spectra to multi-exponential models to extract cycling rates.
///
/// The observed time spectrum in a muCF experiment is a superposition of
/// exponential components:
/// S(t) = sum_i A_i * exp(-lambda_i * t)
///
/// Components include:
/// - Muon decay: lambda_0 = 1/tau_mu
/// - Muonic atom formation: fast component
/// - Molecular formation: intermediate component
/// - Sticking loss: modifies the effective cycling rate
///
/// This estimator fits up to N exponential components to time-binned data.
#[derive(Debug, Clone)]
pub struct CyclingRateEstimator {
    /// Time bins (us).
    time_bins_us: Vec<f64>,
    /// Counts in each bin.
    counts: Vec<f64>,
    /// Fitted components: (amplitude, rate in 1/us).
    components: Vec<(f64, f64)>,
}

impl CyclingRateEstimator {
    /// Create a new estimator with empty data.
    pub fn new() -> Self {
        Self {
            time_bins_us: Vec::new(),
            counts: Vec::new(),
            components: Vec::new(),
        }
    }

    /// Add time-binned data.
    pub fn add_data(&mut self, time_bins_us: &[f64], counts: &[f64]) {
        self.time_bins_us = time_bins_us.to_vec();
        self.counts = counts.to_vec();
    }

    /// Fit a single exponential: A * exp(-lambda * t).
    /// Uses linear regression on log(counts) vs time.
    /// Returns (amplitude, rate_per_us).
    pub fn fit_single_exponential(&mut self) -> Option<(f64, f64)> {
        if self.time_bins_us.len() < 2 || self.counts.len() < 2 {
            return None;
        }
        let n = self.time_bins_us.len().min(self.counts.len());

        // Filter positive counts for log transform
        let mut sum_t = 0.0;
        let mut sum_y = 0.0;
        let mut sum_tt = 0.0;
        let mut sum_ty = 0.0;
        let mut count = 0;

        for i in 0..n {
            if self.counts[i] > 0.0 {
                let t = self.time_bins_us[i];
                let y = self.counts[i].ln();
                sum_t += t;
                sum_y += y;
                sum_tt += t * t;
                sum_ty += t * y;
                count += 1;
            }
        }

        if count < 2 {
            return None;
        }

        let c = count as f64;
        let denom = c * sum_tt - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (c * sum_ty - sum_t * sum_y) / denom;
        let intercept = (sum_y - slope * sum_t) / c;

        let amplitude = intercept.exp();
        let rate = -slope; // lambda in 1/us

        self.components = vec![(amplitude, rate)];
        Some((amplitude, rate))
    }

    /// Fit a double exponential: A1*exp(-l1*t) + A2*exp(-l2*t).
    /// Uses iterative peeling method: fit the slow component at late times,
    /// subtract it, then fit the residual fast component.
    pub fn fit_double_exponential(&mut self) -> Option<Vec<(f64, f64)>> {
        if self.time_bins_us.len() < 4 || self.counts.len() < 4 {
            return None;
        }
        let n = self.time_bins_us.len().min(self.counts.len());

        // Fit slow component from the latter half of data
        let mid = n / 2;
        let mut sum_t = 0.0;
        let mut sum_y = 0.0;
        let mut sum_tt = 0.0;
        let mut sum_ty = 0.0;
        let mut count = 0;

        for i in mid..n {
            if self.counts[i] > 0.0 {
                let t = self.time_bins_us[i];
                let y = self.counts[i].ln();
                sum_t += t;
                sum_y += y;
                sum_tt += t * t;
                sum_ty += t * y;
                count += 1;
            }
        }

        if count < 2 {
            return None;
        }

        let c = count as f64;
        let denom = c * sum_tt - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope_slow = (c * sum_ty - sum_t * sum_y) / denom;
        let intercept_slow = (sum_y - slope_slow * sum_t) / c;
        let a_slow = intercept_slow.exp();
        let lambda_slow = -slope_slow;

        // Subtract slow component and fit fast from first half
        sum_t = 0.0;
        sum_y = 0.0;
        sum_tt = 0.0;
        sum_ty = 0.0;
        count = 0;

        for i in 0..mid {
            let residual = self.counts[i] - a_slow * (-lambda_slow * self.time_bins_us[i]).exp();
            if residual > 0.0 {
                let t = self.time_bins_us[i];
                let y = residual.ln();
                sum_t += t;
                sum_y += y;
                sum_tt += t * t;
                sum_ty += t * y;
                count += 1;
            }
        }

        if count < 2 {
            // Only one component found
            self.components = vec![(a_slow, lambda_slow)];
            return Some(self.components.clone());
        }

        let c = count as f64;
        let denom = c * sum_tt - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            self.components = vec![(a_slow, lambda_slow)];
            return Some(self.components.clone());
        }

        let slope_fast = (c * sum_ty - sum_t * sum_y) / denom;
        let intercept_fast = (sum_y - slope_fast * sum_t) / c;
        let a_fast = intercept_fast.exp();
        let lambda_fast = -slope_fast;

        self.components = vec![(a_fast, lambda_fast), (a_slow, lambda_slow)];
        Some(self.components.clone())
    }

    /// Evaluate the fitted model at time t (us).
    pub fn evaluate(&self, t_us: f64) -> f64 {
        self.components
            .iter()
            .map(|&(a, lambda)| a * (-lambda * t_us).exp())
            .sum()
    }

    /// Return the fitted components.
    pub fn components(&self) -> &[(f64, f64)] {
        &self.components
    }

    /// Compute residuals (data - model) for quality assessment.
    pub fn residuals(&self) -> Vec<f64> {
        let n = self.time_bins_us.len().min(self.counts.len());
        (0..n)
            .map(|i| self.counts[i] - self.evaluate(self.time_bins_us[i]))
            .collect()
    }

    /// Chi-squared statistic (sum of squared residuals / counts).
    pub fn chi_squared(&self) -> f64 {
        let n = self.time_bins_us.len().min(self.counts.len());
        (0..n)
            .map(|i| {
                let expected = self.evaluate(self.time_bins_us[i]);
                if expected > 0.0 {
                    let diff = self.counts[i] - expected;
                    diff * diff / expected
                } else {
                    0.0
                }
            })
            .sum()
    }
}

impl Default for CyclingRateEstimator {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// EnergyYieldCalculator
// ---------------------------------------------------------------------------

/// Calculates energy yield from muon-catalyzed fusion.
///
/// Net energy per muon:
/// E_net = n_fusions * Q_value - E_muon_production
///
/// For d-t: Q = 17.6 MeV per fusion, muon costs ~5000 MeV to produce.
/// Need n_fusions > E_cost / Q = 5000/17.6 ~ 284 just to break even.
#[derive(Debug, Clone)]
pub struct EnergyYieldCalculator {
    config: MuonConfig,
}

impl EnergyYieldCalculator {
    /// Create a new energy yield calculator.
    pub fn new(config: MuonConfig) -> Self {
        Self { config }
    }

    /// Fusion Q-value in MeV for the configured target.
    pub fn q_value_mev(&self) -> f64 {
        self.config.target.q_value_mev()
    }

    /// Muon production cost in MeV.
    pub fn production_cost_mev(&self) -> f64 {
        self.config.muon_production_cost_gev * 1000.0
    }

    /// Gross fusion energy from n fusions (MeV).
    pub fn gross_energy_mev(&self, n_fusions: f64) -> f64 {
        n_fusions * self.q_value_mev()
    }

    /// Net energy per muon: n_fusions * Q - E_production (MeV).
    pub fn net_energy_mev(&self, n_fusions: f64) -> f64 {
        self.gross_energy_mev(n_fusions) - self.production_cost_mev()
    }

    /// Energy gain factor: Q_total / E_production.
    pub fn energy_gain(&self, n_fusions: f64) -> f64 {
        self.gross_energy_mev(n_fusions) / self.production_cost_mev()
    }

    /// Minimum fusions needed for energy breakeven.
    /// n_breakeven = E_production / Q_value
    pub fn breakeven_fusions(&self) -> f64 {
        self.production_cost_mev() / self.q_value_mev()
    }

    /// Whether a given number of fusions achieves breakeven.
    pub fn is_breakeven(&self, n_fusions: f64) -> bool {
        n_fusions >= self.breakeven_fusions()
    }

    /// Energy efficiency: net_energy / production_cost.
    /// Positive means net energy gain; negative means net loss.
    pub fn efficiency(&self, n_fusions: f64) -> f64 {
        self.net_energy_mev(n_fusions) / self.production_cost_mev()
    }

    /// Neutron energy fraction of total Q for the configured target.
    pub fn neutron_energy_fraction(&self) -> f64 {
        let q = self.q_value_mev();
        if q <= 0.0 {
            return 0.0;
        }
        self.config.target.neutron_energy_mev() / q
    }
}

// ---------------------------------------------------------------------------
// MolecularFormationRate
// ---------------------------------------------------------------------------

/// Models the resonant formation rate of muonic molecules via the Vesman mechanism.
///
/// In d-t muCF, the rate-limiting step is formation of the dt-mu molecule.
/// The Vesman resonance mechanism involves the muonic atom (t-mu or d-mu)
/// colliding with a D2 or DT molecule, where the muonic molecule formation
/// is resonantly enhanced by coupling to ro-vibrational states of the host
/// molecule.
///
/// The formation rate depends strongly on:
/// - Temperature (resonance condition)
/// - Target density (collision rate)
/// - Isotopic composition (d-t vs d-d)
///
/// lambda_f = phi * c_d * lambda_f0 * F(T)
/// where phi = density / LHD, c_d = deuterium concentration,
/// lambda_f0 ~ 10^8 /s is the normalized rate, and F(T) is the
/// temperature-dependent resonance function.
#[derive(Debug, Clone)]
pub struct MolecularFormationRate {
    config: MuonConfig,
}

impl MolecularFormationRate {
    /// Create a new molecular formation rate calculator.
    pub fn new(config: MuonConfig) -> Self {
        Self { config }
    }

    /// Liquid hydrogen density (atoms/cm^3).
    pub const LHD: f64 = 4.25e22;

    /// Normalized density phi = n / n_LHD.
    pub fn normalized_density(&self) -> f64 {
        self.config.density_per_cm3 / Self::LHD
    }

    /// Temperature-dependent resonance function F(T).
    /// Simplified model: Gaussian resonance peak at ~500 K for d-t.
    /// F(T) = exp(-(T - T_res)^2 / (2 * sigma_T^2))
    pub fn resonance_function(&self, temperature_k: f64) -> f64 {
        let (t_res, sigma_t) = match &self.config.target {
            TargetComposition::DeuteriumTritium(_) => (500.0, 200.0),
            TargetComposition::DeuteriumDeuterium => (300.0, 150.0),
            TargetComposition::ProtonDeuterium(_) => (400.0, 180.0),
        };
        (-(temperature_k - t_res).powi(2) / (2.0 * sigma_t * sigma_t)).exp()
    }

    /// Base formation rate lambda_f0 (1/s) at LHD and optimal temperature.
    pub fn base_rate(&self) -> f64 {
        match &self.config.target {
            TargetComposition::DeuteriumTritium(_) => 1.2e8,
            TargetComposition::DeuteriumDeuterium => 3.0e6,
            TargetComposition::ProtonDeuterium(_) => 5.0e5,
        }
    }

    /// Deuterium (or relevant isotope) concentration in the target.
    pub fn isotope_concentration(&self) -> f64 {
        match &self.config.target {
            TargetComposition::DeuteriumTritium(ct) => 1.0 - ct, // deuterium fraction
            TargetComposition::DeuteriumDeuterium => 1.0,
            TargetComposition::ProtonDeuterium(cd) => *cd,
        }
    }

    /// Effective molecular formation rate (1/s).
    /// lambda_f = phi * c_d * lambda_f0 * F(T)
    pub fn formation_rate(&self) -> f64 {
        let phi = self.normalized_density();
        let c_d = self.isotope_concentration();
        let lambda_f0 = self.base_rate();
        let f_t = self.resonance_function(self.config.temperature_k);
        phi * c_d * lambda_f0 * f_t
    }

    /// Formation time (seconds) = 1 / lambda_f.
    pub fn formation_time_s(&self) -> f64 {
        1.0 / self.formation_rate()
    }

    /// Formation time in nanoseconds.
    pub fn formation_time_ns(&self) -> f64 {
        self.formation_time_s() * 1e9
    }
}

// ---------------------------------------------------------------------------
// BreakevenAnalyzer
// ---------------------------------------------------------------------------

/// Analyzes energy breakeven conditions for muon-catalyzed fusion.
///
/// Breakeven requires:
/// n_cycles * Q / E_muon_cost > 1
///
/// where n_cycles is the expected number of fusions per muon.
///
/// The required cycling rate lambda_c,min can be determined from:
/// lambda_c,min * tau / (1 + alpha_s * lambda_c,min * tau) >= n_breakeven
///
/// This analyzer computes the required conditions and compares them to
/// current experimental achievements.
#[derive(Debug, Clone)]
pub struct BreakevenAnalyzer {
    config: MuonConfig,
    yield_calc: EnergyYieldCalculator,
    sticking_calc: StickingCalculator,
}

impl BreakevenAnalyzer {
    /// Create a new breakeven analyzer.
    pub fn new(config: MuonConfig) -> Self {
        let yield_calc = EnergyYieldCalculator::new(config.clone());
        let sticking_calc = StickingCalculator::new(config.clone());
        Self {
            config,
            yield_calc,
            sticking_calc,
        }
    }

    /// Number of fusions needed for breakeven.
    pub fn required_fusions(&self) -> f64 {
        self.yield_calc.breakeven_fusions()
    }

    /// Maximum achievable fusions limited by sticking alone.
    pub fn sticking_limited_fusions(&self) -> f64 {
        self.sticking_calc.effective_cycles()
    }

    /// Whether breakeven is achievable with current sticking probability.
    /// True if 1/alpha_s_eff > n_breakeven.
    pub fn is_achievable(&self) -> bool {
        self.sticking_limited_fusions() > self.required_fusions()
    }

    /// Required alpha-sticking for breakeven.
    /// alpha_s,max = 1 / n_breakeven
    pub fn required_max_sticking(&self) -> f64 {
        1.0 / self.required_fusions()
    }

    /// Required cycling rate (1/s) to achieve n_target fusions per muon.
    /// From: n = lambda_c * tau / (1 + alpha_s * lambda_c * tau)
    /// Solving: lambda_c = n * lambda_d / (1 - n * alpha_s)
    /// where lambda_d = 1/tau.
    pub fn required_cycling_rate(&self, n_target: f64) -> Option<f64> {
        let alpha_s = self.sticking_calc.effective_sticking();
        let lambda_d = 1.0 / self.config.muon_lifetime_s;

        let denom = 1.0 - n_target * alpha_s;
        if denom <= 0.0 {
            // Impossible: sticking prevents reaching n_target
            return None;
        }
        Some(n_target * lambda_d / denom)
    }

    /// Required cycling rate for breakeven (1/s).
    pub fn breakeven_cycling_rate(&self) -> Option<f64> {
        self.required_cycling_rate(self.required_fusions())
    }

    /// Energy deficit: how much more energy per muon is needed (MeV).
    /// Returns negative if we have a surplus (breakeven achieved).
    pub fn energy_deficit_mev(&self, n_fusions: f64) -> f64 {
        -self.yield_calc.net_energy_mev(n_fusions)
    }

    /// Sticking reduction factor needed for breakeven.
    /// If current alpha_s is too high, returns the factor by which it must be reduced.
    /// factor < 1 means current sticking already allows breakeven.
    pub fn sticking_reduction_needed(&self) -> f64 {
        let current_alpha = self.sticking_calc.effective_sticking();
        let required_alpha = self.required_max_sticking();
        if required_alpha <= 0.0 || current_alpha <= 0.0 {
            return 0.0;
        }
        current_alpha / required_alpha
    }

    /// Summary of breakeven analysis.
    pub fn summary(&self) -> BreakevenSummary {
        let n_required = self.required_fusions();
        let n_max_sticking = self.sticking_limited_fusions();
        let achievable = self.is_achievable();
        let cycling_rate = self.breakeven_cycling_rate();
        let alpha_eff = self.sticking_calc.effective_sticking();
        let alpha_required = self.required_max_sticking();

        BreakevenSummary {
            q_value_mev: self.yield_calc.q_value_mev(),
            production_cost_mev: self.yield_calc.production_cost_mev(),
            fusions_required: n_required,
            fusions_max_sticking: n_max_sticking,
            is_achievable: achievable,
            effective_sticking: alpha_eff,
            required_sticking: alpha_required,
            breakeven_cycling_rate: cycling_rate,
        }
    }
}

/// Summary of breakeven analysis results.
#[derive(Debug, Clone)]
pub struct BreakevenSummary {
    /// Q-value per fusion (MeV).
    pub q_value_mev: f64,
    /// Muon production cost (MeV).
    pub production_cost_mev: f64,
    /// Fusions required for breakeven.
    pub fusions_required: f64,
    /// Maximum fusions from sticking limit.
    pub fusions_max_sticking: f64,
    /// Whether breakeven is physically achievable.
    pub is_achievable: bool,
    /// Current effective sticking probability.
    pub effective_sticking: f64,
    /// Required sticking for breakeven.
    pub required_sticking: f64,
    /// Required cycling rate for breakeven (1/s), None if impossible.
    pub breakeven_cycling_rate: Option<f64>,
}

// ---------------------------------------------------------------------------
// Convenience functions
// ---------------------------------------------------------------------------

/// Muon decay population fraction at time t.
/// N(t)/N0 = exp(-t / tau_mu)
pub fn muon_population(t_seconds: f64) -> f64 {
    (-t_seconds / MUON_LIFETIME_S).exp()
}

/// Michel spectrum value at reduced energy x = E/E_max.
/// dN/dx = 2*x^2*(3 - 2*x) for 0 <= x <= 1.
pub fn michel_spectrum(x: f64) -> f64 {
    if x < 0.0 || x > 1.0 {
        return 0.0;
    }
    2.0 * x * x * (3.0 - 2.0 * x)
}

/// Muonic atom energy level in eV.
/// E_n = -13.6 * (m_reduced/m_e) * Z^2 / n^2
pub fn muonic_energy_level_ev(z: u32, n: u32, nuclear_mass_mev: f64) -> f64 {
    if n == 0 {
        return 0.0;
    }
    let m_reduced = MUON_MASS_MEV * nuclear_mass_mev / (MUON_MASS_MEV + nuclear_mass_mev);
    let ratio = m_reduced / ELECTRON_MASS_MEV;
    -HYDROGEN_GROUND_STATE_EV * ratio * (z as f64).powi(2) / (n as f64).powi(2)
}

/// Neutron energy from time-of-flight measurement.
/// E_n = (1/2) * m_n * (L/t)^2
///
/// * `flight_path_m` - Flight path in metres.
/// * `tof_ns` - Time-of-flight in nanoseconds.
/// Returns energy in MeV.
pub fn neutron_energy_from_tof(flight_path_m: f64, tof_ns: f64) -> f64 {
    if tof_ns <= 0.0 {
        return 0.0;
    }
    let tof_s = tof_ns * 1e-9;
    let v = flight_path_m / tof_s;
    let neutron_mass_kg = 1.674_927_471e-27;
    let e_joules = 0.5 * neutron_mass_kg * v * v;
    let mev_per_joule = 6.242e12;
    e_joules * mev_per_joule
}

/// Effective number of fusions per muon given cycling rate and sticking.
/// n = lambda_c / (alpha_s * lambda_c + lambda_d)
pub fn expected_fusions(cycling_rate: f64, alpha_sticking: f64) -> f64 {
    let lambda_d = 1.0 / MUON_LIFETIME_S;
    cycling_rate / (alpha_sticking * cycling_rate + lambda_d)
}

/// Breakeven number of fusions for a given Q-value and production cost.
pub fn breakeven_fusions(q_value_mev: f64, production_cost_gev: f64) -> f64 {
    (production_cost_gev * 1000.0) / q_value_mev
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;
    const EPSILON_LOOSE: f64 = 1e-3;

    // --- Constants tests ---

    #[test]
    fn test_muon_mass_ratio() {
        let ratio = MUON_MASS_MEV / ELECTRON_MASS_MEV;
        assert!((ratio - MUON_ELECTRON_MASS_RATIO).abs() / MUON_ELECTRON_MASS_RATIO < 0.001);
    }

    #[test]
    fn test_dt_q_value() {
        // d-t Q-value should be approximately 17.6 MeV
        assert!((DT_Q_VALUE_MEV - 17.6).abs() < 0.1);
    }

    #[test]
    fn test_muon_lifetime() {
        assert!((MUON_LIFETIME_US - 2.197).abs() < 0.01);
        assert!((MUON_LIFETIME_S - 2.197e-6).abs() < 1e-8);
    }

    // --- MuonConfig tests ---

    #[test]
    fn test_dt_config() {
        let config = MuonConfig::dt_standard();
        assert_eq!(config.effective_sticking(), DEFAULT_DT_ALPHA_STICKING);
        assert!((config.muon_lifetime_s - MUON_LIFETIME_S).abs() < EPSILON);
    }

    #[test]
    fn test_dd_config() {
        let config = MuonConfig::dd_standard();
        assert_eq!(config.effective_sticking(), DEFAULT_DD_ALPHA_STICKING);
    }

    #[test]
    fn test_custom_sticking() {
        let mut config = MuonConfig::dt_standard();
        config.alpha_sticking = Some(0.003);
        assert_eq!(config.effective_sticking(), 0.003);
    }

    // --- Target composition tests ---

    #[test]
    fn test_target_q_values() {
        let dt = TargetComposition::DeuteriumTritium(0.5);
        assert!((dt.q_value_mev() - DT_Q_VALUE_MEV).abs() < EPSILON);

        let dd = TargetComposition::DeuteriumDeuterium;
        let expected_dd = 0.5 * (DD_N_HE3_Q_VALUE_MEV + DD_P_T_Q_VALUE_MEV);
        assert!((dd.q_value_mev() - expected_dd).abs() < EPSILON);

        let pd = TargetComposition::ProtonDeuterium(0.5);
        assert!((pd.q_value_mev() - PD_Q_VALUE_MEV).abs() < EPSILON);
    }

    #[test]
    fn test_target_neutron_energies() {
        let dt = TargetComposition::DeuteriumTritium(0.5);
        assert!((dt.neutron_energy_mev() - DT_NEUTRON_ENERGY_MEV).abs() < EPSILON);

        let dd = TargetComposition::DeuteriumDeuterium;
        assert!((dd.neutron_energy_mev() - DD_NEUTRON_ENERGY_MEV).abs() < EPSILON);

        let pd = TargetComposition::ProtonDeuterium(0.5);
        assert!((pd.neutron_energy_mev()).abs() < EPSILON);
    }

    // --- MuonDecayModel tests ---

    #[test]
    fn test_decay_at_zero() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        assert!((decay.population(0.0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_decay_at_one_lifetime() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        let n = decay.population(MUON_LIFETIME_S);
        assert!((n - 1.0 / std::f64::consts::E).abs() < EPSILON);
    }

    #[test]
    fn test_decay_at_two_lifetimes() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        let n = decay.population(2.0 * MUON_LIFETIME_S);
        let expected = (-2.0_f64).exp();
        assert!((n - expected).abs() < EPSILON);
    }

    #[test]
    fn test_remaining_population() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        let n0 = 1000.0;
        let remaining = decay.remaining(n0, MUON_LIFETIME_S);
        assert!((remaining - n0 / std::f64::consts::E).abs() < 0.01);
    }

    #[test]
    fn test_half_life() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        let t_half = decay.half_life_s();
        // At t_half, population should be 0.5
        let n = decay.population(t_half);
        assert!((n - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_decay_rate() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        let rate = decay.decay_rate();
        assert!((rate - 1.0 / MUON_LIFETIME_S).abs() < 1.0);
    }

    #[test]
    fn test_michel_spectrum_normalization() {
        // Integral of Michel spectrum should be 1
        let n = 10000;
        let dx = 1.0 / n as f64;
        let integral: f64 = (0..n)
            .map(|i| {
                let x = (i as f64 + 0.5) * dx;
                michel_spectrum(x) * dx
            })
            .sum();
        assert!((integral - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_michel_spectrum_boundaries() {
        assert!(michel_spectrum(-0.1).abs() < EPSILON);
        assert!(michel_spectrum(1.1).abs() < EPSILON);
        assert!(michel_spectrum(0.0).abs() < EPSILON);
    }

    #[test]
    fn test_michel_spectrum_peak() {
        // The Michel spectrum peaks at x = 1 with value 2
        let peak = michel_spectrum(1.0);
        assert!((peak - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_max_electron_energy() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        let e_max = decay.max_electron_energy_mev();
        assert!((e_max - MUON_MASS_MEV / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_time_spectrum() {
        let decay = MuonDecayModel::new(MuonConfig::dt_standard());
        let spectrum = decay.time_spectrum(0.0, 10.0, 100);
        assert_eq!(spectrum.len(), 100);
        // Should be monotonically decreasing
        for i in 1..spectrum.len() {
            assert!(spectrum[i].1 < spectrum[i - 1].1);
        }
    }

    // --- FusionCycleTracker tests ---

    #[test]
    fn test_cycling_rate_positive() {
        let tracker = FusionCycleTracker::new(MuonConfig::dt_standard());
        assert!(tracker.cycling_rate() > 0.0);
    }

    #[test]
    fn test_cycling_rate_limited_by_slowest() {
        let tracker = FusionCycleTracker::new(MuonConfig::dt_standard());
        // Cycling rate should be less than or equal to the slowest component
        let min_rate = tracker.cycling_rate();
        assert!(min_rate <= 1.0e8); // molecular formation is rate-limiting
    }

    #[test]
    fn test_expected_fusions_dt() {
        let config = MuonConfig::dt_standard();
        let tracker = FusionCycleTracker::new(config);
        let n = tracker.expected_fusions();
        // For d-t with alpha_s ~ 0.5%, expect ~100-200 fusions
        assert!(n > 50.0 && n < 300.0);
    }

    #[test]
    fn test_cycle_step() {
        let config = MuonConfig::dt_standard();
        let mut tracker = FusionCycleTracker::new(config);
        assert!(tracker.is_active());
        assert_eq!(tracker.cycles(), 0);

        tracker.step_cycle();
        assert_eq!(tracker.cycles(), 1);
    }

    #[test]
    fn test_cycle_reset() {
        let config = MuonConfig::dt_standard();
        let mut tracker = FusionCycleTracker::new(config);
        tracker.step_cycle();
        tracker.step_cycle();
        tracker.reset();
        assert_eq!(tracker.cycles(), 0);
        assert!(tracker.is_active());
    }

    #[test]
    fn test_loss_probability_positive() {
        let tracker = FusionCycleTracker::new(MuonConfig::dt_standard());
        let loss = tracker.loss_probability_per_cycle();
        assert!(loss > 0.0 && loss < 1.0);
    }

    // --- StickingCalculator tests ---

    #[test]
    fn test_dt_effective_sticking() {
        let calc = StickingCalculator::new(MuonConfig::dt_standard());
        let alpha_eff = calc.effective_sticking();
        // Initial 0.9% * (1 - 0.44) ~ 0.50%
        assert!(alpha_eff > 0.004 && alpha_eff < 0.006);
    }

    #[test]
    fn test_dd_sticking_higher() {
        let dt_calc = StickingCalculator::new(MuonConfig::dt_standard());
        let dd_calc = StickingCalculator::new(MuonConfig::dd_standard());
        assert!(dd_calc.effective_sticking() > dt_calc.effective_sticking());
    }

    #[test]
    fn test_effective_cycles_dt() {
        let calc = StickingCalculator::new(MuonConfig::dt_standard());
        let n = calc.effective_cycles();
        // 1/0.005 = 200 (approximately)
        assert!(n > 150.0 && n < 250.0);
    }

    #[test]
    fn test_survival_probability_decreasing() {
        let calc = StickingCalculator::new(MuonConfig::dt_standard());
        let p0 = calc.survival_probability(0);
        let p10 = calc.survival_probability(10);
        let p100 = calc.survival_probability(100);
        assert!((p0 - 1.0).abs() < EPSILON);
        assert!(p10 > p100);
        assert!(p100 > 0.0);
    }

    #[test]
    fn test_temperature_corrected_sticking() {
        let calc = StickingCalculator::new(MuonConfig::dt_standard());
        let s_300 = calc.temperature_corrected_sticking(300.0);
        let s_600 = calc.temperature_corrected_sticking(600.0);
        // Higher temperature -> more reactivation -> lower effective sticking
        assert!(s_600 < s_300);
    }

    // --- NeutronDetector tests ---

    #[test]
    fn test_neutron_tof_roundtrip() {
        let det = NeutronDetector::new(10.0, 0.05);
        let energy = 14.1; // MeV
        let tof = det.tof_from_energy(energy);
        let e_back = det.energy_from_tof(tof);
        assert!((e_back - energy).abs() / energy < 0.01);
    }

    #[test]
    fn test_neutron_classification_dt() {
        let det = NeutronDetector::new(10.0, 0.05);
        assert_eq!(det.classify_event(14.1), Some("d-t"));
        assert_eq!(det.classify_event(2.45), Some("d-d"));
    }

    #[test]
    fn test_neutron_classification_unknown() {
        let det = NeutronDetector::new(10.0, 0.05);
        assert_eq!(det.classify_event(8.0), None);
    }

    #[test]
    fn test_neutron_event_recording() {
        let mut det = NeutronDetector::new(10.0, 0.05);
        let tof_dt = det.tof_from_energy(14.1);
        let tof_dd = det.tof_from_energy(2.45);
        det.record_event(tof_dt);
        det.record_event(tof_dt);
        det.record_event(tof_dd);
        assert_eq!(det.event_count(), 3);
        let (dt, dd, other) = det.count_by_type();
        assert_eq!(dt, 2);
        assert_eq!(dd, 1);
        assert_eq!(other, 0);
    }

    #[test]
    fn test_neutron_velocity() {
        let v = NeutronDetector::neutron_velocity(14.1);
        // 14.1 MeV neutron should be ~5.2e7 m/s
        assert!(v > 4e7 && v < 6e7);
    }

    #[test]
    fn test_neutron_detector_clear() {
        let mut det = NeutronDetector::new(10.0, 0.05);
        det.record_event(100.0);
        det.clear();
        assert_eq!(det.event_count(), 0);
    }

    // --- XraySpectrometer tests ---

    #[test]
    fn test_muonic_hydrogen_ground_state() {
        let spec = XraySpectrometer::hydrogen();
        let e = spec.ground_state_binding_kev();
        // Muonic hydrogen ground state: ~2.53 keV
        assert!(e > 2.0 && e < 3.0);
    }

    #[test]
    fn test_muonic_hydrogen_k_alpha() {
        let spec = XraySpectrometer::hydrogen();
        let e = spec.k_alpha_kev();
        // K-alpha (2->1): E_1 * (1 - 1/4) = 3/4 * E_1 ~ 1.9 keV
        assert!(e > 1.5 && e < 2.5);
    }

    #[test]
    fn test_reduced_mass_ratio() {
        let spec = XraySpectrometer::hydrogen();
        let ratio = spec.reduced_mass_ratio();
        // For muonic hydrogen, reduced mass ~ 186 * m_e
        assert!(ratio > 180.0 && ratio < 210.0);
    }

    #[test]
    fn test_helium_higher_energy() {
        let h = XraySpectrometer::hydrogen();
        let he = XraySpectrometer::helium();
        // Helium (Z=2) should have ~4x the binding energy of hydrogen
        assert!(he.ground_state_binding_kev() > 3.0 * h.ground_state_binding_kev());
    }

    #[test]
    fn test_lyman_series() {
        let spec = XraySpectrometer::hydrogen();
        let series = spec.lyman_series_kev(5);
        assert_eq!(series.len(), 4); // n=2,3,4,5
        // Each transition should be positive
        for &(_, e) in &series {
            assert!(e > 0.0);
        }
        // Higher n -> higher energy (converging to ionization)
        for i in 1..series.len() {
            assert!(series[i].1 > series[i - 1].1);
        }
    }

    #[test]
    fn test_transition_energy_invalid() {
        let spec = XraySpectrometer::hydrogen();
        // n_upper <= n_lower should return 0
        assert_eq!(spec.transition_energy_ev(1, 2), 0.0);
        assert_eq!(spec.transition_energy_ev(2, 2), 0.0);
        assert_eq!(spec.transition_energy_ev(1, 0), 0.0);
    }

    // --- CyclingRateEstimator tests ---

    #[test]
    fn test_single_exponential_fit() {
        let mut est = CyclingRateEstimator::new();
        let lambda = 0.5; // 1/us
        let amplitude = 100.0;
        let times: Vec<f64> = (0..20).map(|i| i as f64 * 0.5).collect();
        let counts: Vec<f64> = times.iter().map(|&t| amplitude * (-lambda * t).exp()).collect();
        est.add_data(&times, &counts);

        let result = est.fit_single_exponential().unwrap();
        assert!((result.0 - amplitude).abs() / amplitude < 0.01);
        assert!((result.1 - lambda).abs() / lambda < 0.01);
    }

    #[test]
    fn test_estimator_evaluate() {
        let mut est = CyclingRateEstimator::new();
        let times: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let counts: Vec<f64> = times.iter().map(|&t| 50.0 * (-0.3 * t).exp()).collect();
        est.add_data(&times, &counts);
        est.fit_single_exponential();

        // Model should match data reasonably
        let val = est.evaluate(0.0);
        assert!((val - 50.0).abs() / 50.0 < 0.05);
    }

    #[test]
    fn test_estimator_residuals() {
        let mut est = CyclingRateEstimator::new();
        let times: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let counts: Vec<f64> = times.iter().map(|&t| 100.0 * (-0.2 * t).exp()).collect();
        est.add_data(&times, &counts);
        est.fit_single_exponential();

        let residuals = est.residuals();
        assert_eq!(residuals.len(), 10);
        // Residuals should be small for a perfect exponential
        let max_residual = residuals.iter().map(|r| r.abs()).fold(0.0_f64, f64::max);
        assert!(max_residual < 1.0);
    }

    #[test]
    fn test_estimator_default() {
        let est = CyclingRateEstimator::default();
        assert_eq!(est.components().len(), 0);
    }

    // --- EnergyYieldCalculator tests ---

    #[test]
    fn test_breakeven_fusions_dt() {
        let calc = EnergyYieldCalculator::new(MuonConfig::dt_standard());
        let n_be = calc.breakeven_fusions();
        // 5000 MeV / 17.6 MeV ~ 284
        assert!(n_be > 250.0 && n_be < 320.0);
    }

    #[test]
    fn test_net_energy_positive() {
        let calc = EnergyYieldCalculator::new(MuonConfig::dt_standard());
        // 300 fusions should give positive net energy for d-t
        let e_net = calc.net_energy_mev(300.0);
        assert!(e_net > 0.0);
    }

    #[test]
    fn test_net_energy_negative() {
        let calc = EnergyYieldCalculator::new(MuonConfig::dt_standard());
        // 100 fusions should give negative net energy
        let e_net = calc.net_energy_mev(100.0);
        assert!(e_net < 0.0);
    }

    #[test]
    fn test_energy_gain() {
        let calc = EnergyYieldCalculator::new(MuonConfig::dt_standard());
        let gain = calc.energy_gain(150.0);
        // 150 * 17.6 / 5000 ~ 0.53
        assert!(gain > 0.4 && gain < 0.7);
    }

    #[test]
    fn test_is_breakeven() {
        let calc = EnergyYieldCalculator::new(MuonConfig::dt_standard());
        assert!(!calc.is_breakeven(100.0));
        assert!(calc.is_breakeven(300.0));
    }

    #[test]
    fn test_neutron_energy_fraction() {
        let calc = EnergyYieldCalculator::new(MuonConfig::dt_standard());
        let frac = calc.neutron_energy_fraction();
        // 14.1 / 17.6 ~ 0.80
        assert!(frac > 0.75 && frac < 0.85);
    }

    // --- MolecularFormationRate tests ---

    #[test]
    fn test_formation_rate_positive() {
        let calc = MolecularFormationRate::new(MuonConfig::dt_standard());
        assert!(calc.formation_rate() > 0.0);
    }

    #[test]
    fn test_formation_rate_density_scaling() {
        let mut config1 = MuonConfig::dt_standard();
        config1.density_per_cm3 = MolecularFormationRate::LHD;
        let calc1 = MolecularFormationRate::new(config1);

        let mut config2 = MuonConfig::dt_standard();
        config2.density_per_cm3 = MolecularFormationRate::LHD * 2.0;
        let calc2 = MolecularFormationRate::new(config2);

        // Double density -> double formation rate
        let ratio = calc2.formation_rate() / calc1.formation_rate();
        assert!((ratio - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_dt_faster_than_dd() {
        let dt = MolecularFormationRate::new(MuonConfig::dt_standard());
        let dd = MolecularFormationRate::new(MuonConfig::dd_standard());
        assert!(dt.base_rate() > dd.base_rate());
    }

    #[test]
    fn test_formation_time_ns() {
        let calc = MolecularFormationRate::new(MuonConfig::dt_standard());
        let t_ns = calc.formation_time_ns();
        assert!(t_ns > 0.0);
        assert!(t_ns < 1e6); // should be less than 1 ms
    }

    // --- BreakevenAnalyzer tests ---

    #[test]
    fn test_breakeven_dt_not_achievable() {
        let analyzer = BreakevenAnalyzer::new(MuonConfig::dt_standard());
        // With standard parameters (5 GeV cost, 0.5% sticking),
        // sticking limit ~200 < breakeven ~284
        let summary = analyzer.summary();
        assert!(!summary.is_achievable);
    }

    #[test]
    fn test_breakeven_with_low_cost() {
        let mut config = MuonConfig::dt_standard();
        config.muon_production_cost_gev = 2.0; // Cheaper muons
        let analyzer = BreakevenAnalyzer::new(config);
        // 2000/17.6 ~ 114 < 200 sticking limit -> achievable
        assert!(analyzer.is_achievable());
    }

    #[test]
    fn test_required_cycling_rate() {
        let analyzer = BreakevenAnalyzer::new(MuonConfig::dt_standard());
        let rate = analyzer.required_cycling_rate(100.0);
        assert!(rate.is_some());
        assert!(rate.unwrap() > 0.0);
    }

    #[test]
    fn test_sticking_reduction_needed() {
        let analyzer = BreakevenAnalyzer::new(MuonConfig::dt_standard());
        let factor = analyzer.sticking_reduction_needed();
        // Current sticking allows ~200 fusions, need ~284
        // factor should be > 1 (need reduction)
        assert!(factor > 1.0);
    }

    // --- Convenience function tests ---

    #[test]
    fn test_muon_population_fn() {
        let p = muon_population(0.0);
        assert!((p - 1.0).abs() < EPSILON);

        let p_half = muon_population(MUON_LIFETIME_S * 2.0_f64.ln());
        assert!((p_half - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_muonic_energy_level_hydrogen() {
        let e1 = muonic_energy_level_ev(1, 1, PROTON_MASS_MEV);
        // Should be around -2530 eV for muonic hydrogen
        assert!(e1 < -2000.0 && e1 > -3000.0);
    }

    #[test]
    fn test_neutron_energy_from_tof_fn() {
        let e = neutron_energy_from_tof(10.0, 200.0);
        // Should give some reasonable energy
        assert!(e > 0.0);
    }

    #[test]
    fn test_expected_fusions_fn() {
        let n = expected_fusions(1e8, 0.005);
        // Should be order of ~150
        assert!(n > 50.0 && n < 300.0);
    }

    #[test]
    fn test_breakeven_fusions_fn() {
        let n = breakeven_fusions(17.6, 5.0);
        assert!((n - 5000.0 / 17.6).abs() < 0.1);
    }
}

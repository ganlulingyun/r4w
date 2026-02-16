//! Magneto-Optical Trap (MOT) controller for laser cooling and trapping of neutral atoms.
//!
//! This module implements signal processing and control algorithms used in
//! magneto-optical traps — the workhorse technology for laser cooling neutral
//! atoms to microkelvin temperatures. Applications include atomic clocks,
//! quantum computing, Bose-Einstein condensate production, precision
//! measurements, and atom interferometry.
//!
//! # Physics Background
//!
//! A MOT combines three pairs of counter-propagating laser beams (slightly red-detuned
//! from an atomic transition) with a quadrupole magnetic field from anti-Helmholtz coils.
//! The Zeeman shift of atomic energy levels in the spatially varying magnetic field
//! creates a position-dependent scattering force that both cools (friction) and traps
//! (restoring force) atoms near the field zero.
//!
//! The scattering force on a two-level atom from a single beam is:
//!
//! ```text
//! F = hbar * k * (Gamma/2) * s / (1 + s + (2*delta_eff/Gamma)^2)
//! ```
//!
//! where `s = I/I_sat` is the saturation parameter, `delta_eff` includes both
//! the laser detuning and the Doppler shift `k*v`, and `Gamma` is the natural
//! linewidth.
//!
//! # Key Results
//!
//! - **Doppler temperature**: `T_D = hbar * Gamma / (2 * k_B)` ~ 146 µK for Rb-87
//! - **Recoil temperature**: `T_recoil = (hbar*k)^2 / (m * k_B)` ~ 362 nK for Rb-87
//! - **Capture velocity**: `v_c ~ Gamma / k` ~ a few m/s
//!
//! # Example
//!
//! ```
//! use r4w_core::magneto_optical_trap_controller::{
//!     MotConfig, AtomSpecies, DopplerCoolingForce, DopplerLimit,
//!     TemperatureEstimator, AtomNumberEstimator,
//! };
//!
//! // Configure a Rb-87 MOT
//! let config = MotConfig::new(AtomSpecies::Rb87)
//!     .with_detuning_gamma(-2.0)    // -2 Gamma detuning
//!     .with_intensity_mw_cm2(5.0)   // 5 mW/cm^2 per beam
//!     .with_field_gradient_g_cm(15.0); // 15 G/cm axial gradient
//!
//! // Compute Doppler cooling force on an atom at rest
//! let force = DopplerCoolingForce::new(&config);
//! let f = force.force_single_beam(0.0); // velocity = 0
//! assert!(f > 0.0); // photon scattering pushes atom
//!
//! // Doppler limit temperature
//! let t_doppler = DopplerLimit::temperature(&config);
//! assert!((t_doppler * 1e6 - 146.0).abs() < 5.0); // ~146 µK for Rb-87
//!
//! // Estimate atom number from fluorescence
//! let n = AtomNumberEstimator::from_fluorescence(&config, 1e-6); // 1 µW
//! assert!(n > 1e4);
//! ```

use std::f64::consts::PI;

// ── Physical constants ───────────────────────────────────────────────────────

/// Reduced Planck constant in J*s.
const HBAR: f64 = 1.054_571_817e-34;

/// Boltzmann constant in J/K.
const K_B: f64 = 1.380_649e-23;

/// Speed of light in m/s.
const C: f64 = 2.997_924_58e8;

/// Atomic mass unit in kg.
const AMU: f64 = 1.660_539_066_6e-27;

/// Bohr magneton in J/T.
const MU_B: f64 = 9.274_010_078_3e-24;

// ── Atom species ─────────────────────────────────────────────────────────────

/// Atom species commonly used in magneto-optical traps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AtomSpecies {
    /// Rubidium-87, D2 line (5S1/2 -> 5P3/2), lambda = 780.241 nm.
    Rb87,
    /// Cesium-133, D2 line (6S1/2 -> 6P3/2), lambda = 852.347 nm.
    Cs133,
    /// Strontium-88, blue MOT line (1S0 -> 1P1), lambda = 460.862 nm.
    Sr88,
    /// Lithium-6, D2 line (2S1/2 -> 2P3/2), lambda = 670.977 nm.
    Li6,
}

/// Physical properties for a given atom species and transition.
#[derive(Debug, Clone, Copy)]
pub struct AtomProperties {
    /// Transition wavelength in metres.
    pub wavelength_m: f64,
    /// Natural linewidth (angular frequency) in rad/s.
    pub gamma_rad_s: f64,
    /// Saturation intensity in W/m^2.
    pub i_sat_w_m2: f64,
    /// Atomic mass in kg.
    pub mass_kg: f64,
    /// Effective Lande g-factor product (g_e * m_e - g_g * m_g) for cycling transition.
    pub effective_g_factor: f64,
}

impl AtomSpecies {
    /// Return the physical properties of the species' primary cooling transition.
    pub fn properties(self) -> AtomProperties {
        match self {
            AtomSpecies::Rb87 => AtomProperties {
                wavelength_m: 780.241_209e-9,
                gamma_rad_s: 2.0 * PI * 6.065e6, // 2*pi * 6.065 MHz
                i_sat_w_m2: 16.69,                // 1.669 mW/cm^2 = 16.69 W/m^2
                mass_kg: 86.909_180_5 * AMU,
                effective_g_factor: 1.0, // F=2 -> F'=3 cycling transition, effective ~1
            },
            AtomSpecies::Cs133 => AtomProperties {
                wavelength_m: 852.347_275e-9,
                gamma_rad_s: 2.0 * PI * 5.234e6,
                i_sat_w_m2: 11.023, // 1.1023 mW/cm^2
                mass_kg: 132.905_451_9 * AMU,
                effective_g_factor: 1.0,
            },
            AtomSpecies::Sr88 => AtomProperties {
                wavelength_m: 460.862e-9,
                gamma_rad_s: 2.0 * PI * 30.5e6, // Broad blue line
                i_sat_w_m2: 428.0,               // 42.8 mW/cm^2
                mass_kg: 87.905_612_3 * AMU,
                effective_g_factor: 1.0,
            },
            AtomSpecies::Li6 => AtomProperties {
                wavelength_m: 670.977e-9,
                gamma_rad_s: 2.0 * PI * 5.872e6,
                i_sat_w_m2: 25.4, // 2.54 mW/cm^2
                mass_kg: 6.015_122_8 * AMU,
                effective_g_factor: 1.0,
            },
        }
    }

    /// Wavevector magnitude k = 2*pi / lambda in 1/m.
    pub fn wavevector(self) -> f64 {
        2.0 * PI / self.properties().wavelength_m
    }

    /// Recoil velocity v_rec = hbar*k / m in m/s.
    pub fn recoil_velocity(self) -> f64 {
        let p = self.properties();
        HBAR * self.wavevector() / p.mass_kg
    }

    /// Recoil energy E_rec = (hbar*k)^2 / (2*m) in Joules.
    pub fn recoil_energy(self) -> f64 {
        let p = self.properties();
        let hk = HBAR * self.wavevector();
        hk * hk / (2.0 * p.mass_kg)
    }
}

// ── MOT Configuration ────────────────────────────────────────────────────────

/// Configuration for a magneto-optical trap.
#[derive(Debug, Clone)]
pub struct MotConfig {
    /// Atom species.
    pub species: AtomSpecies,
    /// Laser detuning in units of Gamma (negative = red-detuned).
    /// Typical: -1 to -3 Gamma for MOT loading, -0.5 Gamma for molasses.
    pub detuning_gamma: f64,
    /// Laser intensity per beam in W/m^2.
    pub intensity_w_m2: f64,
    /// Axial magnetic field gradient in T/m (dB/dz).
    /// Typical: 10-20 G/cm = 0.1-0.2 T/m.
    pub field_gradient_t_m: f64,
    /// Beam waist radius (1/e^2) in metres.
    pub beam_waist_m: f64,
}

impl MotConfig {
    /// Create a new MOT configuration for the given atom species with defaults.
    ///
    /// Defaults: detuning = -2 Gamma, I = 2*I_sat per beam, gradient = 10 G/cm,
    /// beam waist = 10 mm.
    pub fn new(species: AtomSpecies) -> Self {
        let p = species.properties();
        Self {
            species,
            detuning_gamma: -2.0,
            intensity_w_m2: 2.0 * p.i_sat_w_m2,
            field_gradient_t_m: 0.10, // 10 G/cm
            beam_waist_m: 0.010,      // 10 mm
        }
    }

    /// Set laser detuning in units of Gamma.
    pub fn with_detuning_gamma(mut self, delta: f64) -> Self {
        self.detuning_gamma = delta;
        self
    }

    /// Set laser intensity per beam in mW/cm^2.
    pub fn with_intensity_mw_cm2(mut self, i_mw_cm2: f64) -> Self {
        self.intensity_w_m2 = i_mw_cm2 * 10.0; // 1 mW/cm^2 = 10 W/m^2
        self
    }

    /// Set axial magnetic field gradient in G/cm.
    pub fn with_field_gradient_g_cm(mut self, grad: f64) -> Self {
        self.field_gradient_t_m = grad * 0.01; // 1 G/cm = 0.01 T/m
        self
    }

    /// Set beam waist radius in mm.
    pub fn with_beam_waist_mm(mut self, w_mm: f64) -> Self {
        self.beam_waist_m = w_mm * 1e-3;
        self
    }

    /// Saturation parameter s = I / I_sat.
    pub fn saturation_parameter(&self) -> f64 {
        self.intensity_w_m2 / self.species.properties().i_sat_w_m2
    }

    /// Absolute detuning in rad/s.
    pub fn detuning_rad_s(&self) -> f64 {
        self.detuning_gamma * self.species.properties().gamma_rad_s
    }

    /// Capture velocity estimate: v_c ~ |delta| / k.
    pub fn capture_velocity(&self) -> f64 {
        let k = self.species.wavevector();
        (self.detuning_gamma.abs() * self.species.properties().gamma_rad_s) / k
    }
}

// ── Doppler Cooling Force ────────────────────────────────────────────────────

/// Computes the radiation pressure (scattering) force on a two-level atom.
///
/// For a single beam propagating in +z direction:
///
/// ```text
/// F = hbar * k * (Gamma/2) * s / (1 + s + (2*delta_eff/Gamma)^2)
/// ```
///
/// where `delta_eff = delta_laser - k*v` (Doppler shift).
///
/// For a 1D MOT (two counter-propagating beams), the net force is the sum
/// of forces from both beams. Near v=0 this linearises to a friction force
/// `F ≈ -alpha * v`.
pub struct DopplerCoolingForce {
    hbar_k: f64,
    gamma: f64,
    s: f64,
    delta: f64, // detuning in rad/s (negative for red)
    k: f64,
}

impl DopplerCoolingForce {
    /// Create from a MOT configuration.
    pub fn new(config: &MotConfig) -> Self {
        let p = config.species.properties();
        let k = config.species.wavevector();
        Self {
            hbar_k: HBAR * k,
            gamma: p.gamma_rad_s,
            s: config.saturation_parameter(),
            delta: config.detuning_rad_s(),
            k,
        }
    }

    /// Scattering force from a single beam propagating in +z, on an atom
    /// moving with velocity `v` (positive = +z direction) in Newtons.
    pub fn force_single_beam(&self, v: f64) -> f64 {
        let delta_eff = self.delta - self.k * v;
        let denom = 1.0 + self.s + (2.0 * delta_eff / self.gamma).powi(2);
        self.hbar_k * (self.gamma / 2.0) * self.s / denom
    }

    /// Net force from counter-propagating beam pair (1D molasses).
    /// Beam 1: +z direction, Beam 2: -z direction.
    pub fn force_molasses_1d(&self, v: f64) -> f64 {
        // +z beam: pushes in +z, Doppler shift -k*v
        let f_plus = self.force_single_beam(v);
        // -z beam: pushes in -z, Doppler shift +k*v
        let delta_eff_minus = self.delta + self.k * v;
        let denom_minus =
            1.0 + self.s + (2.0 * delta_eff_minus / self.gamma).powi(2);
        let f_minus = self.hbar_k * (self.gamma / 2.0) * self.s / denom_minus;
        f_plus - f_minus
    }

    /// Scattering rate from a single beam in photons/s.
    pub fn scattering_rate(&self, v: f64) -> f64 {
        let delta_eff = self.delta - self.k * v;
        let denom = 1.0 + self.s + (2.0 * delta_eff / self.gamma).powi(2);
        (self.gamma / 2.0) * self.s / denom
    }

    /// Friction coefficient alpha from linearised molasses force: F ≈ -alpha * v.
    ///
    /// ```text
    /// alpha = -4 * hbar * k^2 * s * (2*delta/Gamma) / (1 + s + (2*delta/Gamma)^2)^2
    /// ```
    ///
    /// Returns positive alpha when delta < 0 (red detuned).
    pub fn friction_coefficient(&self) -> f64 {
        let x = 2.0 * self.delta / self.gamma;
        let denom = (1.0 + self.s + x * x).powi(2);
        -4.0 * HBAR * self.k * self.k * self.s * x / denom
    }

    /// Maximum deceleration from a single beam (saturated scattering force).
    pub fn max_deceleration(&self, mass_kg: f64) -> f64 {
        let f_max = self.hbar_k * self.gamma / 2.0; // s -> infinity limit
        f_max / mass_kg
    }
}

// ── Doppler Limit ────────────────────────────────────────────────────────────

/// Computes the Doppler cooling limit temperature.
///
/// The minimum temperature achievable in optical molasses (Doppler cooling):
///
/// ```text
/// T_D = hbar * Gamma / (2 * k_B)
/// ```
///
/// This occurs at detuning delta = -Gamma/2 and low intensity.
pub struct DopplerLimit;

impl DopplerLimit {
    /// Doppler temperature in Kelvin for the given configuration's atom species.
    pub fn temperature(config: &MotConfig) -> f64 {
        let gamma = config.species.properties().gamma_rad_s;
        HBAR * gamma / (2.0 * K_B)
    }

    /// Doppler temperature for a given linewidth (angular frequency).
    pub fn from_linewidth(gamma_rad_s: f64) -> f64 {
        HBAR * gamma_rad_s / (2.0 * K_B)
    }

    /// Doppler-limited RMS velocity in m/s.
    pub fn rms_velocity(config: &MotConfig) -> f64 {
        let t = Self::temperature(config);
        let m = config.species.properties().mass_kg;
        (K_B * t / m).sqrt()
    }
}

// ── Sub-Doppler Cooling ──────────────────────────────────────────────────────

/// Sub-Doppler (Sisyphus / polarization gradient) cooling parameters.
///
/// Polarization gradient cooling in lin-perp-lin or sigma+/sigma- configurations
/// can cool atoms below the Doppler limit down to a few recoil temperatures.
///
/// ```text
/// T_recoil = (hbar * k)^2 / (m * k_B)
/// ```
pub struct SubDopplerCooling;

impl SubDopplerCooling {
    /// Recoil temperature in Kelvin: T_rec = (hbar*k)^2 / (m*k_B).
    pub fn recoil_temperature(species: AtomSpecies) -> f64 {
        let p = species.properties();
        let k = species.wavevector();
        let hbar_k = HBAR * k;
        (hbar_k * hbar_k) / (p.mass_kg * K_B)
    }

    /// Typical achievable temperature in polarization gradient cooling,
    /// approximately a few times the recoil temperature.
    /// Uses empirical scaling: T ~ C * E_rec / k_B where C ~ 10 for typical conditions.
    pub fn estimated_temperature(species: AtomSpecies, detuning_gamma: f64) -> f64 {
        let t_rec = Self::recoil_temperature(species);
        // Empirical: T/T_rec ~ |Omega_R^2 / (delta * Gamma)| factor
        // For large detunings, T approaches a few T_rec
        let factor = 5.0 / detuning_gamma.abs().max(0.5);
        t_rec * (1.0 + factor)
    }

    /// Recoil-limited RMS velocity in m/s.
    pub fn recoil_rms_velocity(species: AtomSpecies) -> f64 {
        let t = Self::recoil_temperature(species);
        let m = species.properties().mass_kg;
        (K_B * t / m).sqrt()
    }
}

// ── Optical Molasses Model ───────────────────────────────────────────────────

/// 1D optical molasses model with friction and diffusion.
///
/// Near v=0, the molasses force is approximately:
///
/// ```text
/// F = -alpha * v
/// ```
///
/// The equilibrium temperature from balancing friction heating and cooling:
///
/// ```text
/// T = D / (alpha * k_B)
/// ```
///
/// where D is the momentum diffusion coefficient.
pub struct OpticalMolassesModel {
    alpha: f64,
    diffusion: f64,
}

impl OpticalMolassesModel {
    /// Create from MOT configuration.
    pub fn new(config: &MotConfig) -> Self {
        let force = DopplerCoolingForce::new(config);
        let alpha = force.friction_coefficient();

        // Momentum diffusion coefficient: D = (hbar*k)^2 * R_scatter
        // where R_scatter is the scattering rate from all beams
        // For 1D pair at v=0: R_total = 2 * R_single
        let r_single = force.scattering_rate(0.0);
        let hbar_k = HBAR * config.species.wavevector();
        // Factor of 2 for two beams, and additional factor for recoil randomness
        let diffusion = 2.0 * hbar_k * hbar_k * r_single;

        Self { alpha, diffusion }
    }

    /// Friction coefficient alpha in kg/s.
    pub fn friction(&self) -> f64 {
        self.alpha
    }

    /// Momentum diffusion coefficient D in kg^2*m^2/s^3.
    pub fn diffusion(&self) -> f64 {
        self.diffusion
    }

    /// Equilibrium temperature T = D / (alpha * k_B) in Kelvin.
    pub fn equilibrium_temperature(&self) -> f64 {
        if self.alpha.abs() < 1e-40 {
            return f64::INFINITY;
        }
        self.diffusion / (self.alpha * K_B)
    }

    /// Damping time constant tau = m / alpha in seconds.
    pub fn damping_time(&self, mass_kg: f64) -> f64 {
        mass_kg / self.alpha
    }

    /// Simulate 1D velocity evolution over time using Euler integration.
    /// Returns velocity at each time step.
    pub fn simulate_velocity(
        &self,
        mass_kg: f64,
        v0: f64,
        dt: f64,
        num_steps: usize,
    ) -> Vec<f64> {
        let mut v = v0;
        let mut result = Vec::with_capacity(num_steps);
        for _ in 0..num_steps {
            result.push(v);
            let a = -self.alpha * v / mass_kg;
            v += a * dt;
        }
        result
    }
}

// ── Magnetic Field Profile ───────────────────────────────────────────────────

/// Anti-Helmholtz coil quadrupole magnetic field model.
///
/// Near the center of an anti-Helmholtz pair, the field varies linearly:
///
/// ```text
/// B_z(z) = B' * z     (axial)
/// B_r(r) = -B'/2 * r  (radial, by Maxwell div B = 0)
/// ```
///
/// where `B'` is the axial field gradient in T/m.
pub struct MagneticFieldProfile {
    /// Axial gradient dB/dz in T/m.
    gradient_t_m: f64,
}

impl MagneticFieldProfile {
    /// Create from a MOT configuration.
    pub fn new(config: &MotConfig) -> Self {
        Self {
            gradient_t_m: config.field_gradient_t_m,
        }
    }

    /// Create from gradient in T/m.
    pub fn from_gradient(gradient_t_m: f64) -> Self {
        Self { gradient_t_m }
    }

    /// Axial field B_z at position z (metres) in Tesla.
    pub fn field_axial(&self, z: f64) -> f64 {
        self.gradient_t_m * z
    }

    /// Radial field B_r at position r (metres) in Tesla.
    /// By Maxwell's div(B)=0 for quadrupole: B_r = -B'/2 * r.
    pub fn field_radial(&self, r: f64) -> f64 {
        -self.gradient_t_m / 2.0 * r
    }

    /// Total field magnitude at position (r, z) in Tesla.
    pub fn field_magnitude(&self, r: f64, z: f64) -> f64 {
        let bz = self.field_axial(z);
        let br = self.field_radial(r);
        (bz * bz + br * br).sqrt()
    }

    /// Zeeman shift at position z for the cycling transition in rad/s.
    /// delta_Z = mu_B * g * m * B(z) / hbar.
    pub fn zeeman_shift(&self, z: f64, g_factor: f64) -> f64 {
        MU_B * g_factor * self.field_axial(z).abs() / HBAR
    }

    /// Trap spring constant kappa in N/m from the MOT restoring force.
    ///
    /// ```text
    /// kappa = alpha * (mu_B * g * B') / (hbar * k)
    /// ```
    ///
    /// This is a simplified estimate relating the friction and Zeeman gradient.
    pub fn spring_constant(&self, config: &MotConfig) -> f64 {
        let force = DopplerCoolingForce::new(config);
        let alpha = force.friction_coefficient();
        let k = config.species.wavevector();
        let g = config.species.properties().effective_g_factor;
        alpha * MU_B * g * self.gradient_t_m / (HBAR * k)
    }

    /// Trap frequency omega_trap = sqrt(kappa/m) in rad/s.
    pub fn trap_frequency(&self, config: &MotConfig) -> f64 {
        let kappa = self.spring_constant(config);
        let m = config.species.properties().mass_kg;
        if kappa <= 0.0 {
            return 0.0;
        }
        (kappa / m).sqrt()
    }
}

// ── Atom Number Estimator ────────────────────────────────────────────────────

/// Estimates the number of trapped atoms from fluorescence measurements.
///
/// Each atom scatters photons at rate R_scatter. The total fluorescence power
/// collected by a detector subtending solid angle Omega is:
///
/// ```text
/// P_fluor = N * hbar * omega * R_scatter * (Omega / 4*pi)
/// ```
///
/// Inverting: `N = 4*pi * P_fluor / (hbar * omega * R_scatter * Omega)`.
/// If total fluorescence power (4*pi collection) is given:
/// `N = P_total / (hbar * omega * R_scatter)`.
pub struct AtomNumberEstimator;

impl AtomNumberEstimator {
    /// Estimate atom number from total (4*pi) fluorescence power in Watts.
    pub fn from_fluorescence(config: &MotConfig, total_power_w: f64) -> f64 {
        let p = config.species.properties();
        let force = DopplerCoolingForce::new(config);
        let r_scatter = force.scattering_rate(0.0);
        let omega = 2.0 * PI * C / p.wavelength_m;
        let photon_energy = HBAR * omega;

        total_power_w / (photon_energy * r_scatter)
    }

    /// Estimate atom number given partial fluorescence collection.
    ///
    /// `power_w` is the collected power, `solid_angle_sr` is the collection
    /// solid angle in steradians.
    pub fn from_partial_fluorescence(
        config: &MotConfig,
        power_w: f64,
        solid_angle_sr: f64,
    ) -> f64 {
        let total_power = power_w * 4.0 * PI / solid_angle_sr;
        Self::from_fluorescence(config, total_power)
    }

    /// Scattering rate per atom in photons/s at v=0.
    pub fn scattering_rate(config: &MotConfig) -> f64 {
        let force = DopplerCoolingForce::new(config);
        force.scattering_rate(0.0)
    }

    /// Power radiated per atom in Watts.
    pub fn power_per_atom(config: &MotConfig) -> f64 {
        let p = config.species.properties();
        let r_scatter = Self::scattering_rate(config);
        let omega = 2.0 * PI * C / p.wavelength_m;
        HBAR * omega * r_scatter
    }
}

// ── Laser Lock Servo ─────────────────────────────────────────────────────────

/// PID servo controller for laser frequency stabilisation.
///
/// Used to lock a cooling laser to the atomic transition via modulation
/// transfer spectroscopy or saturated absorption. The error signal is
/// proportional to the frequency offset from the lock point.
#[derive(Debug, Clone)]
pub struct LaserLockServo {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
    /// Integral accumulator.
    integral: f64,
    /// Previous error for derivative.
    prev_error: f64,
    /// Output limits (min, max) in Hz.
    pub output_limits: (f64, f64),
    /// Sample interval in seconds.
    pub dt: f64,
}

impl LaserLockServo {
    /// Create a new PID servo with given gains and sample rate.
    pub fn new(kp: f64, ki: f64, kd: f64, sample_rate_hz: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            output_limits: (-1e9, 1e9), // +/- 1 GHz default
            dt: 1.0 / sample_rate_hz,
        }
    }

    /// Set output limits.
    pub fn with_output_limits(mut self, min: f64, max: f64) -> Self {
        self.output_limits = (min, max);
        self
    }

    /// Reset the integrator and state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    /// Compute control output given error signal.
    ///
    /// Returns the frequency correction in Hz to apply to the laser.
    pub fn update(&mut self, error: f64) -> f64 {
        self.integral += error * self.dt;

        // Anti-windup: clamp integral
        let max_integral = (self.output_limits.1 - self.output_limits.0) / (2.0 * self.ki.abs().max(1e-30));
        self.integral = self.integral.clamp(-max_integral, max_integral);

        let derivative = (error - self.prev_error) / self.dt;
        self.prev_error = error;

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        output.clamp(self.output_limits.0, self.output_limits.1)
    }

    /// Process a sequence of error signals, returning the control outputs.
    pub fn process(&mut self, errors: &[f64]) -> Vec<f64> {
        errors.iter().map(|&e| self.update(e)).collect()
    }
}

// ── Saturated Absorption Spectrum ────────────────────────────────────────────

/// Models the saturated absorption spectroscopy signal used for laser locking.
///
/// In saturated absorption, a pump beam saturates the transition, creating
/// Lamb dips (reduced absorption) at the line center of each hyperfine
/// transition. The sub-Doppler features appear on top of the Doppler-broadened
/// absorption profile.
pub struct SaturatedAbsorptionSpectrum;

impl SaturatedAbsorptionSpectrum {
    /// Doppler-broadened absorption profile (Gaussian).
    ///
    /// FWHM = (omega_0 / c) * sqrt(8 * k_B * T * ln(2) / m).
    pub fn doppler_profile(
        freq_offset_hz: f64,
        temperature_k: f64,
        species: AtomSpecies,
    ) -> f64 {
        let p = species.properties();
        let omega_0 = 2.0 * PI * C / p.wavelength_m;
        let fwhm = (omega_0 / (2.0 * PI * C))
            * (8.0 * K_B * temperature_k * (2.0_f64.ln()) / p.mass_kg).sqrt();
        let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt());
        let x = freq_offset_hz / sigma;
        (-0.5 * x * x).exp() / (sigma * (2.0 * PI).sqrt())
    }

    /// Lorentzian (natural linewidth) profile for Lamb dip.
    ///
    /// L(f) = (Gamma/(2*pi)) / (f^2 + (Gamma/(2*pi))^2).
    pub fn lorentzian_profile(freq_offset_hz: f64, gamma_rad_s: f64) -> f64 {
        let hw = gamma_rad_s / (2.0 * PI); // half-width in Hz
        hw / (PI * (freq_offset_hz * freq_offset_hz + hw * hw))
    }

    /// Saturated absorption signal: Doppler background with Lamb dip subtracted.
    ///
    /// Returns absorption as a function of frequency offset from line center.
    /// `dip_depth` is the relative depth of the Lamb dip (0..1).
    pub fn signal(
        freq_offset_hz: f64,
        temperature_k: f64,
        species: AtomSpecies,
        dip_depth: f64,
    ) -> f64 {
        let p = species.properties();
        let doppler = Self::doppler_profile(freq_offset_hz, temperature_k, species);
        let lamb_dip = Self::lorentzian_profile(freq_offset_hz, p.gamma_rad_s);
        // Normalise Lamb dip to peak depth relative to Doppler peak
        let doppler_peak = Self::doppler_profile(0.0, temperature_k, species);
        let lamb_peak = Self::lorentzian_profile(0.0, p.gamma_rad_s);
        let normalized_dip = dip_depth * doppler_peak * lamb_dip / lamb_peak;
        doppler - normalized_dip
    }

    /// Dispersive error signal (derivative of Lamb dip) for frequency locking.
    /// Computed by finite difference.
    pub fn error_signal(
        freq_offset_hz: f64,
        temperature_k: f64,
        species: AtomSpecies,
        dip_depth: f64,
    ) -> f64 {
        let df = 1e3; // 1 kHz step
        let s_plus = Self::signal(freq_offset_hz + df, temperature_k, species, dip_depth);
        let s_minus = Self::signal(freq_offset_hz - df, temperature_k, species, dip_depth);
        (s_plus - s_minus) / (2.0 * df)
    }
}

// ── Trap Lifetime Analyzer ───────────────────────────────────────────────────

/// Analyses exponential decay of trapped atom number to extract trap lifetime.
///
/// The atom number decays as:
///
/// ```text
/// N(t) = N0 * exp(-t / tau_trap)
/// ```
///
/// where `tau_trap` is limited by background gas collisions, light-assisted
/// collisions, and other loss mechanisms.
pub struct TrapLifetimeAnalyzer;

impl TrapLifetimeAnalyzer {
    /// Fit exponential decay N(t) = N0 * exp(-t/tau) using least-squares on log(N).
    ///
    /// Returns (N0, tau) or None if the fit fails.
    pub fn fit_exponential(times_s: &[f64], atom_numbers: &[f64]) -> Option<(f64, f64)> {
        if times_s.len() < 2 || times_s.len() != atom_numbers.len() {
            return None;
        }

        // Linear regression on ln(N) = ln(N0) - t/tau
        let mut sum_t = 0.0;
        let mut sum_ln = 0.0;
        let mut sum_t2 = 0.0;
        let mut sum_t_ln = 0.0;
        let mut count = 0usize;

        for (&t, &n) in times_s.iter().zip(atom_numbers.iter()) {
            if n <= 0.0 {
                continue;
            }
            let ln_n = n.ln();
            sum_t += t;
            sum_ln += ln_n;
            sum_t2 += t * t;
            sum_t_ln += t * ln_n;
            count += 1;
        }

        if count < 2 {
            return None;
        }
        let n = count as f64;
        let denom = n * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_t_ln - sum_t * sum_ln) / denom;
        let intercept = (sum_ln - slope * sum_t) / n;

        let tau = -1.0 / slope; // slope = -1/tau
        let n0 = intercept.exp();

        if tau > 0.0 { Some((n0, tau)) } else { None }
    }

    /// Generate a synthetic decay curve for testing.
    pub fn generate_decay(n0: f64, tau: f64, times_s: &[f64]) -> Vec<f64> {
        times_s.iter().map(|&t| n0 * (-t / tau).exp()).collect()
    }

    /// Compute the 1/e lifetime from the decay data (time to reach N0/e).
    pub fn lifetime_1e(times_s: &[f64], atom_numbers: &[f64]) -> Option<f64> {
        Self::fit_exponential(times_s, atom_numbers).map(|(_, tau)| tau)
    }

    /// Compute residuals between data and fitted exponential.
    pub fn residuals(times_s: &[f64], atom_numbers: &[f64]) -> Option<Vec<f64>> {
        let (n0, tau) = Self::fit_exponential(times_s, atom_numbers)?;
        Some(
            times_s
                .iter()
                .zip(atom_numbers.iter())
                .map(|(&t, &n)| n - n0 * (-t / tau).exp())
                .collect(),
        )
    }
}

// ── Temperature Estimator (Time-of-Flight) ───────────────────────────────────

/// Estimates atom cloud temperature from time-of-flight (TOF) expansion.
///
/// After release from the trap, the cloud expands ballistically:
///
/// ```text
/// sigma_x^2(t) = sigma_0^2 + (k_B * T / m) * t^2
/// ```
///
/// By measuring the cloud size sigma_x at multiple expansion times t,
/// a linear fit of sigma^2 vs t^2 yields the temperature.
pub struct TemperatureEstimator;

impl TemperatureEstimator {
    /// Fit temperature from TOF data.
    ///
    /// `times_s`: expansion times after release.
    /// `sigmas_m`: measured 1/e cloud radii in metres.
    /// `species`: atom species for mass.
    ///
    /// Returns (T_kelvin, sigma_0_m) or None on failure.
    pub fn fit_tof(
        times_s: &[f64],
        sigmas_m: &[f64],
        species: AtomSpecies,
    ) -> Option<(f64, f64)> {
        if times_s.len() < 2 || times_s.len() != sigmas_m.len() {
            return None;
        }

        // Linear regression: sigma^2 = sigma_0^2 + (k_B*T/m) * t^2
        // Let x = t^2, y = sigma^2. Fit y = a + b*x.
        let n = times_s.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_x2 = 0.0;
        let mut sum_xy = 0.0;

        for (&t, &s) in times_s.iter().zip(sigmas_m.iter()) {
            let x = t * t;
            let y = s * s;
            sum_x += x;
            sum_y += y;
            sum_x2 += x * x;
            sum_xy += x * y;
        }

        let denom = n * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1e-60 {
            return None;
        }

        let b = (n * sum_xy - sum_x * sum_y) / denom; // k_B*T/m
        let a = (sum_y - b * sum_x) / n; // sigma_0^2

        let m = species.properties().mass_kg;
        let temperature = b * m / K_B;
        let sigma_0 = if a >= 0.0 { a.sqrt() } else { 0.0 };

        if temperature >= 0.0 {
            Some((temperature, sigma_0))
        } else {
            None
        }
    }

    /// Generate synthetic TOF expansion data for testing.
    pub fn generate_tof_data(
        temperature_k: f64,
        sigma_0_m: f64,
        species: AtomSpecies,
        times_s: &[f64],
    ) -> Vec<f64> {
        let m = species.properties().mass_kg;
        let kbt_m = K_B * temperature_k / m;
        times_s
            .iter()
            .map(|&t| (sigma_0_m * sigma_0_m + kbt_m * t * t).sqrt())
            .collect()
    }

    /// Compute temperature from a single TOF measurement, given initial size.
    pub fn temperature_single(
        sigma_0_m: f64,
        sigma_t_m: f64,
        t_expand_s: f64,
        species: AtomSpecies,
    ) -> f64 {
        let m = species.properties().mass_kg;
        let dsq = sigma_t_m * sigma_t_m - sigma_0_m * sigma_0_m;
        if dsq < 0.0 || t_expand_s <= 0.0 {
            return 0.0;
        }
        dsq * m / (K_B * t_expand_s * t_expand_s)
    }
}

// ── MOT Loading Dynamics ─────────────────────────────────────────────────────

/// Models MOT loading from a background vapour or slowed atom beam.
///
/// The atom number evolves as:
///
/// ```text
/// dN/dt = R_load - N/tau_loss
/// N(t) = N_ss * (1 - exp(-t/tau_loss))
/// ```
///
/// where `N_ss = R_load * tau_loss`.
pub struct MotLoadingModel {
    /// Loading rate in atoms/s.
    pub loading_rate: f64,
    /// Loss time constant in seconds.
    pub tau_loss: f64,
}

impl MotLoadingModel {
    /// Create with given loading rate and loss time.
    pub fn new(loading_rate: f64, tau_loss: f64) -> Self {
        Self {
            loading_rate,
            tau_loss,
        }
    }

    /// Steady-state atom number.
    pub fn steady_state_number(&self) -> f64 {
        self.loading_rate * self.tau_loss
    }

    /// Atom number at time t (starting from empty trap).
    pub fn atom_number(&self, t: f64) -> f64 {
        self.steady_state_number() * (1.0 - (-t / self.tau_loss).exp())
    }

    /// Time to reach fraction f of steady-state (0 < f < 1).
    pub fn time_to_fraction(&self, f: f64) -> f64 {
        if f <= 0.0 || f >= 1.0 {
            return f64::INFINITY;
        }
        -self.tau_loss * (1.0 - f).ln()
    }

    /// Generate loading curve.
    pub fn generate_loading_curve(&self, times_s: &[f64]) -> Vec<f64> {
        times_s.iter().map(|&t| self.atom_number(t)).collect()
    }

    /// Fit loading curve to extract (R_load, tau_loss).
    pub fn fit_loading(times_s: &[f64], atom_numbers: &[f64]) -> Option<(f64, f64)> {
        if times_s.len() < 3 {
            return None;
        }

        // Estimate N_ss from the last few points
        let n = atom_numbers.len();
        let n_ss = atom_numbers[n - 1].max(1.0);

        // Fit 1 - N/N_ss = exp(-t/tau) => ln(1 - N/N_ss) = -t/tau
        let mut sum_t = 0.0;
        let mut sum_y = 0.0;
        let mut sum_t2 = 0.0;
        let mut sum_ty = 0.0;
        let mut count = 0usize;

        for (&t, &nn) in times_s.iter().zip(atom_numbers.iter()) {
            let ratio = nn / n_ss;
            if ratio >= 1.0 || ratio <= 0.0 {
                continue;
            }
            let y = (1.0 - ratio).ln();
            sum_t += t;
            sum_y += y;
            sum_t2 += t * t;
            sum_ty += t * y;
            count += 1;
        }

        if count < 2 {
            return None;
        }
        let nc = count as f64;
        let denom = nc * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (nc * sum_ty - sum_t * sum_y) / denom;
        let tau = -1.0 / slope;
        let loading_rate = n_ss / tau;

        if tau > 0.0 {
            Some((loading_rate, tau))
        } else {
            None
        }
    }
}

// ── Helper Functions ─────────────────────────────────────────────────────────

/// Compute the maximum scattering rate (saturated) Gamma/2.
pub fn max_scattering_rate(species: AtomSpecies) -> f64 {
    species.properties().gamma_rad_s / 2.0
}

/// Photon recoil momentum hbar*k in kg*m/s.
pub fn recoil_momentum(species: AtomSpecies) -> f64 {
    HBAR * species.wavevector()
}

/// Maximum radiation pressure acceleration a_max = hbar*k*Gamma/(2*m) in m/s^2.
pub fn max_acceleration(species: AtomSpecies) -> f64 {
    let p = species.properties();
    HBAR * species.wavevector() * p.gamma_rad_s / (2.0 * p.mass_kg)
}

/// Stopping distance for an atom at velocity v: d = v^2 / (2*a_max).
pub fn stopping_distance(species: AtomSpecies, velocity_mps: f64) -> f64 {
    let a = max_acceleration(species);
    velocity_mps * velocity_mps / (2.0 * a)
}

/// MOT capture velocity estimate for given beam size and detuning.
pub fn capture_velocity(config: &MotConfig) -> f64 {
    config.capture_velocity()
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 0.1; // 10% relative tolerance for physics comparisons

    fn approx_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if b == 0.0 {
            return a.abs() < 1e-30;
        }
        ((a - b) / b).abs() < rel_tol
    }

    // ── Atom species properties ──────────────────────────────────────────

    #[test]
    fn test_rb87_wavelength() {
        let p = AtomSpecies::Rb87.properties();
        assert!(approx_eq(p.wavelength_m, 780.241e-9, 1e-3));
    }

    #[test]
    fn test_rb87_linewidth() {
        let p = AtomSpecies::Rb87.properties();
        let gamma_mhz = p.gamma_rad_s / (2.0 * PI * 1e6);
        assert!(approx_eq(gamma_mhz, 6.065, 0.01));
    }

    #[test]
    fn test_rb87_saturation_intensity() {
        let p = AtomSpecies::Rb87.properties();
        // 1.669 mW/cm^2 = 16.69 W/m^2
        assert!(approx_eq(p.i_sat_w_m2, 16.69, 0.01));
    }

    #[test]
    fn test_cs133_wavelength() {
        let p = AtomSpecies::Cs133.properties();
        assert!(approx_eq(p.wavelength_m, 852.347e-9, 1e-3));
    }

    #[test]
    fn test_species_wavevector() {
        let k = AtomSpecies::Rb87.wavevector();
        let expected = 2.0 * PI / 780.241e-9;
        assert!(approx_eq(k, expected, 1e-3));
    }

    #[test]
    fn test_rb87_recoil_velocity() {
        let v_rec = AtomSpecies::Rb87.recoil_velocity();
        // v_rec ~ 5.88 mm/s for Rb-87
        assert!(approx_eq(v_rec, 5.88e-3, 0.05));
    }

    // ── MOT Configuration ────────────────────────────────────────────────

    #[test]
    fn test_mot_config_defaults() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        assert_eq!(config.detuning_gamma, -2.0);
        assert!(config.saturation_parameter() > 1.0);
    }

    #[test]
    fn test_mot_config_builder() {
        let config = MotConfig::new(AtomSpecies::Rb87)
            .with_detuning_gamma(-3.0)
            .with_intensity_mw_cm2(10.0)
            .with_field_gradient_g_cm(15.0)
            .with_beam_waist_mm(12.0);
        assert_eq!(config.detuning_gamma, -3.0);
        assert!(approx_eq(config.intensity_w_m2, 100.0, 1e-6));
        assert!(approx_eq(config.field_gradient_t_m, 0.15, 1e-6));
        assert!(approx_eq(config.beam_waist_m, 0.012, 1e-6));
    }

    #[test]
    fn test_saturation_parameter() {
        let config = MotConfig::new(AtomSpecies::Rb87)
            .with_intensity_mw_cm2(1.669); // I = I_sat
        assert!(approx_eq(config.saturation_parameter(), 1.0, 0.01));
    }

    #[test]
    fn test_capture_velocity() {
        let config = MotConfig::new(AtomSpecies::Rb87).with_detuning_gamma(-2.0);
        let vc = config.capture_velocity();
        // v_c ~ |2*Gamma| / k ~ a few m/s
        assert!(vc > 1.0 && vc < 50.0);
    }

    // ── Doppler cooling force ────────────────────────────────────────────

    #[test]
    fn test_single_beam_force_positive() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        let force = DopplerCoolingForce::new(&config);
        let f = force.force_single_beam(0.0);
        assert!(f > 0.0, "Single beam force should be positive (push)");
    }

    #[test]
    fn test_scattering_rate_bounded() {
        let config = MotConfig::new(AtomSpecies::Rb87)
            .with_intensity_mw_cm2(1000.0); // Very high intensity
        let force = DopplerCoolingForce::new(&config);
        let r = force.scattering_rate(0.0);
        let gamma = config.species.properties().gamma_rad_s;
        // Should be less than Gamma/2 (saturated limit)
        assert!(r < gamma / 2.0);
        assert!(r > 0.0);
    }

    #[test]
    fn test_molasses_force_decelerating() {
        // Red-detuned molasses should decelerate moving atoms
        let config = MotConfig::new(AtomSpecies::Rb87).with_detuning_gamma(-2.0);
        let force = DopplerCoolingForce::new(&config);
        let f = force.force_molasses_1d(1.0); // atom moving at 1 m/s
        assert!(f < 0.0, "Molasses should decelerate: F = {} N", f);
    }

    #[test]
    fn test_molasses_force_zero_at_rest() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        let force = DopplerCoolingForce::new(&config);
        let f = force.force_molasses_1d(0.0);
        assert!(f.abs() < 1e-30, "Molasses force at v=0 should be zero");
    }

    #[test]
    fn test_molasses_force_antisymmetric() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        let force = DopplerCoolingForce::new(&config);
        let f_pos = force.force_molasses_1d(0.5);
        let f_neg = force.force_molasses_1d(-0.5);
        assert!(
            approx_eq(f_pos, -f_neg, 1e-10),
            "Molasses force should be antisymmetric"
        );
    }

    #[test]
    fn test_friction_coefficient_positive_for_red_detuning() {
        let config = MotConfig::new(AtomSpecies::Rb87).with_detuning_gamma(-1.0);
        let force = DopplerCoolingForce::new(&config);
        let alpha = force.friction_coefficient();
        assert!(
            alpha > 0.0,
            "Friction coefficient should be positive for red detuning, got {}",
            alpha
        );
    }

    #[test]
    fn test_friction_coefficient_negative_for_blue_detuning() {
        let config = MotConfig::new(AtomSpecies::Rb87).with_detuning_gamma(1.0);
        let force = DopplerCoolingForce::new(&config);
        let alpha = force.friction_coefficient();
        assert!(
            alpha < 0.0,
            "Friction coefficient should be negative for blue detuning (heating)"
        );
    }

    #[test]
    fn test_max_deceleration() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        let force = DopplerCoolingForce::new(&config);
        let m = config.species.properties().mass_kg;
        let a_max = force.max_deceleration(m);
        // For Rb-87: a_max ~ 1.1e5 m/s^2
        assert!(a_max > 1e4 && a_max < 1e7);
    }

    // ── Doppler limit ────────────────────────────────────────────────────

    #[test]
    fn test_doppler_temperature_rb87() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        let t_d = DopplerLimit::temperature(&config);
        // T_D ~ 146 µK for Rb-87
        let t_uk = t_d * 1e6;
        assert!(
            approx_eq(t_uk, 146.0, TOLERANCE),
            "Doppler temperature should be ~146 µK, got {} µK",
            t_uk
        );
    }

    #[test]
    fn test_doppler_temperature_cs133() {
        let config = MotConfig::new(AtomSpecies::Cs133);
        let t_d = DopplerLimit::temperature(&config);
        // T_D ~ 125 µK for Cs-133
        let t_uk = t_d * 1e6;
        assert!(
            approx_eq(t_uk, 125.0, TOLERANCE),
            "Cs Doppler temp ~ 125 µK, got {} µK",
            t_uk
        );
    }

    #[test]
    fn test_doppler_rms_velocity() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        let v_rms = DopplerLimit::rms_velocity(&config);
        // v_rms = sqrt(k_B * T_D / m) ~ 0.12 m/s for Rb-87
        assert!(v_rms > 0.05 && v_rms < 0.5);
    }

    // ── Sub-Doppler cooling ──────────────────────────────────────────────

    #[test]
    fn test_recoil_temperature_rb87() {
        let t_rec = SubDopplerCooling::recoil_temperature(AtomSpecies::Rb87);
        // T_rec ~ 362 nK for Rb-87
        let t_nk = t_rec * 1e9;
        assert!(
            approx_eq(t_nk, 362.0, TOLERANCE),
            "Recoil temperature ~ 362 nK, got {} nK",
            t_nk
        );
    }

    #[test]
    fn test_recoil_temperature_below_doppler() {
        let t_rec = SubDopplerCooling::recoil_temperature(AtomSpecies::Rb87);
        let config = MotConfig::new(AtomSpecies::Rb87);
        let t_doppler = DopplerLimit::temperature(&config);
        assert!(
            t_rec < t_doppler,
            "Recoil temperature should be below Doppler limit"
        );
    }

    #[test]
    fn test_sub_doppler_estimated_temperature() {
        let t_est = SubDopplerCooling::estimated_temperature(AtomSpecies::Rb87, -5.0);
        let t_rec = SubDopplerCooling::recoil_temperature(AtomSpecies::Rb87);
        assert!(t_est > t_rec, "Estimated T should be above T_recoil");
        let config = MotConfig::new(AtomSpecies::Rb87);
        let t_doppler = DopplerLimit::temperature(&config);
        assert!(
            t_est < t_doppler,
            "Sub-Doppler T should be below Doppler limit"
        );
    }

    // ── Optical molasses model ───────────────────────────────────────────

    #[test]
    fn test_optical_molasses_friction_positive() {
        let config = MotConfig::new(AtomSpecies::Rb87).with_detuning_gamma(-2.0);
        let molasses = OpticalMolassesModel::new(&config);
        assert!(molasses.friction() > 0.0);
    }

    #[test]
    fn test_optical_molasses_equilibrium_temperature() {
        let config = MotConfig::new(AtomSpecies::Rb87)
            .with_detuning_gamma(-0.5)
            .with_intensity_mw_cm2(0.5);
        let molasses = OpticalMolassesModel::new(&config);
        let t = molasses.equilibrium_temperature();
        let t_doppler = DopplerLimit::temperature(&config);
        // At optimal detuning (-Gamma/2), T should be near Doppler limit
        assert!(t > 0.0, "Equilibrium temperature should be positive");
        // Allow factor of 10 for this simplified model
        assert!(
            t < 10.0 * t_doppler,
            "T = {} K should be within 10x of T_D = {} K",
            t,
            t_doppler
        );
    }

    #[test]
    fn test_molasses_velocity_damping() {
        let config = MotConfig::new(AtomSpecies::Rb87).with_detuning_gamma(-2.0);
        let molasses = OpticalMolassesModel::new(&config);
        let m = config.species.properties().mass_kg;
        let velocities = molasses.simulate_velocity(m, 1.0, 1e-6, 1000);
        // Velocity should decrease monotonically for red detuning
        assert!(velocities.last().unwrap().abs() < velocities[0].abs());
    }

    // ── Magnetic field profile ───────────────────────────────────────────

    #[test]
    fn test_field_zero_at_origin() {
        let config = MotConfig::new(AtomSpecies::Rb87);
        let field = MagneticFieldProfile::new(&config);
        assert_eq!(field.field_axial(0.0), 0.0);
        assert_eq!(field.field_radial(0.0), 0.0);
    }

    #[test]
    fn test_field_linear_axial() {
        let field = MagneticFieldProfile::from_gradient(0.10); // 10 G/cm
        assert!(approx_eq(field.field_axial(0.01), 0.001, 1e-6)); // 1 cm -> 1 G = 0.001 T? No. 10 G/cm * 1 cm = 10 G = 0.001 T
        // Actually: 0.10 T/m * 0.01 m = 0.001 T = 10 Gauss. Correct.
        assert!(approx_eq(field.field_axial(0.01), 0.001, 1e-6));
    }

    #[test]
    fn test_field_radial_half_gradient() {
        let field = MagneticFieldProfile::from_gradient(0.10);
        let bz = field.field_axial(0.01);
        let br = field.field_radial(0.01);
        // Radial gradient is half of axial for quadrupole
        assert!(approx_eq(br.abs(), bz.abs() / 2.0, 1e-10));
    }

    #[test]
    fn test_trap_frequency_positive() {
        let config = MotConfig::new(AtomSpecies::Rb87)
            .with_detuning_gamma(-2.0)
            .with_field_gradient_g_cm(15.0);
        let field = MagneticFieldProfile::new(&config);
        let omega = field.trap_frequency(&config);
        assert!(omega > 0.0, "Trap frequency should be positive");
        // Typical MOT frequency: tens of Hz to ~100 Hz -> omega ~ 60-600 rad/s
        let f_hz = omega / (2.0 * PI);
        assert!(
            f_hz > 0.1 && f_hz < 1e4,
            "Trap frequency {} Hz out of range",
            f_hz
        );
    }

    // ── Atom number estimator ────────────────────────────────────────────

    #[test]
    fn test_atom_number_from_fluorescence() {
        let config = MotConfig::new(AtomSpecies::Rb87)
            .with_intensity_mw_cm2(5.0)
            .with_detuning_gamma(-2.0);
        let n = AtomNumberEstimator::from_fluorescence(&config, 1e-6); // 1 µW
        // Should be ~ 10^4 to 10^7 atoms for 1 µW total fluorescence
        assert!(
            n > 1e3 && n < 1e8,
            "Atom number {} out of reasonable range for 1 µW",
            n
        );
    }

    #[test]
    fn test_atom_number_partial_collection() {
        let config = MotConfig::new(AtomSpecies::Rb87).with_intensity_mw_cm2(5.0);
        let n_total = AtomNumberEstimator::from_fluorescence(&config, 1e-6);
        let solid_angle = 0.01; // ~0.01 sr lens
        let n_partial =
            AtomNumberEstimator::from_partial_fluorescence(&config, 1e-6, solid_angle);
        // n_partial should be much larger (collecting less light -> infer more atoms)
        assert!(n_partial > n_total);
    }

    #[test]
    fn test_power_per_atom() {
        let config = MotConfig::new(AtomSpecies::Rb87).with_intensity_mw_cm2(5.0);
        let p = AtomNumberEstimator::power_per_atom(&config);
        // Each atom scatters ~ 10^6-10^7 photons/s at ~1.6 eV each
        // P ~ 10^-13 to 10^-12 W per atom
        assert!(
            p > 1e-15 && p < 1e-10,
            "Power per atom {} out of range",
            p
        );
    }

    // ── Laser lock servo ─────────────────────────────────────────────────

    #[test]
    fn test_servo_proportional_response() {
        let mut servo = LaserLockServo::new(1.0, 0.0, 0.0, 1000.0);
        let out = servo.update(5.0);
        assert!(approx_eq(out, 5.0, 1e-10));
    }

    #[test]
    fn test_servo_integral_accumulation() {
        let mut servo = LaserLockServo::new(0.0, 1000.0, 0.0, 1000.0);
        let _ = servo.update(1.0); // integral = 1.0 * 0.001
        let out = servo.update(1.0); // integral = 2.0 * 0.001
        assert!(out > 0.0, "Integral should produce positive output");
    }

    #[test]
    fn test_servo_reset() {
        let mut servo = LaserLockServo::new(1.0, 100.0, 0.0, 1000.0);
        let _ = servo.update(5.0);
        servo.reset();
        assert_eq!(servo.integral, 0.0);
        assert_eq!(servo.prev_error, 0.0);
    }

    #[test]
    fn test_servo_output_limits() {
        let mut servo = LaserLockServo::new(1e12, 0.0, 0.0, 1000.0)
            .with_output_limits(-100.0, 100.0);
        let out = servo.update(1.0);
        assert!(out <= 100.0 && out >= -100.0);
    }

    #[test]
    fn test_servo_process_batch() {
        let mut servo = LaserLockServo::new(1.0, 0.0, 0.0, 1000.0);
        let errors = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let outputs = servo.process(&errors);
        assert_eq!(outputs.len(), 5);
    }

    // ── Saturated absorption spectrum ────────────────────────────────────

    #[test]
    fn test_doppler_profile_peak_at_zero() {
        let p0 = SaturatedAbsorptionSpectrum::doppler_profile(0.0, 300.0, AtomSpecies::Rb87);
        let p1 = SaturatedAbsorptionSpectrum::doppler_profile(1e9, 300.0, AtomSpecies::Rb87);
        assert!(p0 > p1, "Doppler profile should peak at zero offset");
    }

    #[test]
    fn test_lorentzian_profile_peak() {
        let gamma = AtomSpecies::Rb87.properties().gamma_rad_s;
        let l0 = SaturatedAbsorptionSpectrum::lorentzian_profile(0.0, gamma);
        let l1 = SaturatedAbsorptionSpectrum::lorentzian_profile(1e7, gamma);
        assert!(l0 > l1, "Lorentzian should peak at zero");
    }

    #[test]
    fn test_saturated_absorption_dip() {
        // At line center, the Lamb dip should reduce absorption
        let s_center = SaturatedAbsorptionSpectrum::signal(
            0.0, 300.0, AtomSpecies::Rb87, 0.5,
        );
        let s_wing = SaturatedAbsorptionSpectrum::signal(
            5e6, 300.0, AtomSpecies::Rb87, 0.5,
        );
        // Dip means absorption at center is less than nearby Doppler profile
        let doppler_center =
            SaturatedAbsorptionSpectrum::doppler_profile(0.0, 300.0, AtomSpecies::Rb87);
        assert!(
            s_center < doppler_center,
            "Lamb dip should reduce absorption at center"
        );
    }

    #[test]
    fn test_error_signal_zero_crossing() {
        // Error signal should be near zero at line center (antisymmetric)
        let e0 = SaturatedAbsorptionSpectrum::error_signal(
            0.0, 300.0, AtomSpecies::Rb87, 0.5,
        );
        assert!(
            e0.abs() < 1e-10,
            "Error signal at resonance should be near zero, got {}",
            e0
        );
    }

    // ── Trap lifetime analyzer ───────────────────────────────────────────

    #[test]
    fn test_exponential_decay_fit() {
        let n0 = 1e6;
        let tau = 10.0;
        let times: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let data = TrapLifetimeAnalyzer::generate_decay(n0, tau, &times);
        let (fit_n0, fit_tau) = TrapLifetimeAnalyzer::fit_exponential(&times, &data).unwrap();
        assert!(approx_eq(fit_n0, n0, 0.01));
        assert!(approx_eq(fit_tau, tau, 0.01));
    }

    #[test]
    fn test_lifetime_1e() {
        let times: Vec<f64> = (0..50).map(|i| i as f64 * 0.5).collect();
        let data = TrapLifetimeAnalyzer::generate_decay(1e6, 5.0, &times);
        let tau = TrapLifetimeAnalyzer::lifetime_1e(&times, &data).unwrap();
        assert!(approx_eq(tau, 5.0, 0.01));
    }

    #[test]
    fn test_decay_residuals_small() {
        let times: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let data = TrapLifetimeAnalyzer::generate_decay(1e6, 10.0, &times);
        let residuals = TrapLifetimeAnalyzer::residuals(&times, &data).unwrap();
        let max_res = residuals.iter().map(|r| r.abs()).fold(0.0_f64, f64::max);
        assert!(max_res < 1.0, "Residuals should be tiny for perfect data");
    }

    // ── Temperature estimator (TOF) ─────────────────────────────────────

    #[test]
    fn test_tof_temperature_fit() {
        let species = AtomSpecies::Rb87;
        let t_actual = 150e-6; // 150 µK
        let sigma_0 = 0.5e-3; // 0.5 mm initial size
        let times: Vec<f64> = (1..11).map(|i| i as f64 * 1e-3).collect(); // 1-10 ms
        let sigmas =
            TemperatureEstimator::generate_tof_data(t_actual, sigma_0, species, &times);
        let (t_fit, s0_fit) =
            TemperatureEstimator::fit_tof(&times, &sigmas, species).unwrap();
        assert!(
            approx_eq(t_fit, t_actual, 0.02),
            "Fitted T = {} K, expected {} K",
            t_fit,
            t_actual
        );
        assert!(
            approx_eq(s0_fit, sigma_0, 0.02),
            "Fitted sigma_0 = {} m, expected {} m",
            s0_fit,
            sigma_0
        );
    }

    #[test]
    fn test_tof_single_measurement() {
        let species = AtomSpecies::Rb87;
        let t_actual = 100e-6; // 100 µK
        let sigma_0 = 1e-3;
        let t_expand = 5e-3; // 5 ms
        let m = species.properties().mass_kg;
        let sigma_t = (sigma_0 * sigma_0 + K_B * t_actual / m * t_expand * t_expand).sqrt();
        let t_meas =
            TemperatureEstimator::temperature_single(sigma_0, sigma_t, t_expand, species);
        assert!(approx_eq(t_meas, t_actual, 0.01));
    }

    // ── MOT loading model ────────────────────────────────────────────────

    #[test]
    fn test_loading_steady_state() {
        let model = MotLoadingModel::new(1e8, 10.0); // 1e8 atoms/s, 10 s lifetime
        let n_ss = model.steady_state_number();
        assert!(approx_eq(n_ss, 1e9, 1e-10));
    }

    #[test]
    fn test_loading_starts_at_zero() {
        let model = MotLoadingModel::new(1e8, 10.0);
        let n0 = model.atom_number(0.0);
        assert!(n0.abs() < 1.0);
    }

    #[test]
    fn test_loading_approaches_steady_state() {
        let model = MotLoadingModel::new(1e8, 10.0);
        let n_ss = model.steady_state_number();
        let n_long = model.atom_number(100.0); // 10 tau
        assert!(approx_eq(n_long, n_ss, 0.001));
    }

    #[test]
    fn test_loading_time_to_half() {
        let model = MotLoadingModel::new(1e8, 10.0);
        let t_half = model.time_to_fraction(0.5);
        // t = -tau * ln(0.5) = tau * ln(2) ~ 6.93 s
        assert!(approx_eq(t_half, 10.0 * 2.0_f64.ln(), 0.01));
    }

    #[test]
    fn test_loading_curve_monotonic() {
        let model = MotLoadingModel::new(1e8, 10.0);
        let times: Vec<f64> = (0..50).map(|i| i as f64 * 0.5).collect();
        let curve = model.generate_loading_curve(&times);
        for i in 1..curve.len() {
            assert!(curve[i] >= curve[i - 1], "Loading curve should be monotonic");
        }
    }

    // ── Helper functions ─────────────────────────────────────────────────

    #[test]
    fn test_max_scattering_rate_value() {
        let r_max = max_scattering_rate(AtomSpecies::Rb87);
        let expected = AtomSpecies::Rb87.properties().gamma_rad_s / 2.0;
        assert!(approx_eq(r_max, expected, 1e-10));
    }

    #[test]
    fn test_max_acceleration_rb87() {
        let a = max_acceleration(AtomSpecies::Rb87);
        // a_max ~ 1.1e5 m/s^2 for Rb-87
        assert!(a > 5e4 && a < 5e5);
    }

    #[test]
    fn test_stopping_distance() {
        let d = stopping_distance(AtomSpecies::Rb87, 300.0); // typical thermal velocity
        // Zeeman slower tube: ~0.3-1 m for Rb at 300 m/s
        assert!(d > 0.01 && d < 10.0, "Stopping distance {} m", d);
    }

    #[test]
    fn test_recoil_momentum() {
        let p = recoil_momentum(AtomSpecies::Rb87);
        let expected = HBAR * AtomSpecies::Rb87.wavevector();
        assert!(approx_eq(p, expected, 1e-10));
    }

    #[test]
    fn test_all_species_doppler_temperatures() {
        // All species should have Doppler temperatures in µK range
        for species in &[
            AtomSpecies::Rb87,
            AtomSpecies::Cs133,
            AtomSpecies::Sr88,
            AtomSpecies::Li6,
        ] {
            let config = MotConfig::new(*species);
            let t_d = DopplerLimit::temperature(&config);
            let t_uk = t_d * 1e6;
            assert!(
                t_uk > 1.0 && t_uk < 1000.0,
                "{:?} Doppler temp {} µK out of range",
                species,
                t_uk
            );
        }
    }

    #[test]
    fn test_all_species_recoil_temperatures() {
        for species in &[
            AtomSpecies::Rb87,
            AtomSpecies::Cs133,
            AtomSpecies::Sr88,
            AtomSpecies::Li6,
        ] {
            let t_rec = SubDopplerCooling::recoil_temperature(*species);
            let t_nk = t_rec * 1e9;
            assert!(
                t_nk > 1.0 && t_nk < 10_000.0,
                "{:?} recoil temp {} nK out of range",
                species,
                t_nk
            );
        }
    }
}

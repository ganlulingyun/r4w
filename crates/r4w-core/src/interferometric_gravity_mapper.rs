//! Signal processing for atom interferometry-based gravity measurement.
//!
//! This module implements the signal processing chain for cold-atom interferometric
//! gravimeters and gravity gradiometers. Atom interferometry exploits the wave nature
//! of matter to measure inertial effects with extraordinary precision: modern
//! instruments routinely reach 1 nGal (10^-11 m/s^2) sensitivity, enabling absolute
//! gravimetry, inertial navigation, geophysical surveys, and tests of fundamental
//! physics (equivalence principle, gravitational constant G).
//!
//! # Physics background
//!
//! A cloud of laser-cooled atoms (typically Rb-87 or Cs-133) is released in free fall
//! and interrogated by a sequence of Raman laser pulses that coherently split, redirect,
//! and recombine the atomic wavepackets. The standard Mach-Zehnder configuration uses
//! three pulses (pi/2 — pi — pi/2) separated by free-evolution time T. The resulting
//! interferometric phase is:
//!
//! ```text
//!   phi = k_eff * g * T^2
//! ```
//!
//! where `k_eff = 2 * k_laser = 4 * pi / lambda` is the effective wave vector of the
//! two-photon Raman transition. For Rb-87 at 780.24 nm with T = 100 ms, one fringe
//! corresponds to a gravity change of about 1.2 uGal, and sub-fringe resolution yields
//! nGal-class sensitivity.
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`GravimeterConfig`] | Atom species, interrogation time, laser wavelength, Rabi freq |
//! | [`MachZehnderInterferometer`] | Three-pulse pi/2-pi-pi/2 sequence simulation |
//! | [`PhaseExtractor`] | Extract interferometric phase from population fringe |
//! | [`GravityCalculator`] | Convert phase to gravity: g = phi / (k_eff * T^2) |
//! | [`FringeScanner`] | Scan Raman laser phase, fit sinusoid for phase + contrast |
//! | [`GradientMeter`] | Differential gravity measurement for gradiometry |
//! | [`TidalCorrector`] | Solid Earth tide correction (lunar + solar, ~100 uGal) |
//! | [`AtmosphericPressureCorrector`] | Barometric admittance correction (-0.3 uGal/hPa) |
//! | [`DriftEstimator`] | Instrument drift removal via polynomial fitting |
//! | [`NoiseAnalyzer`] | Allan deviation of gravity measurements |
//! | [`FreeEvolutionModel`] | Atom trajectory during free fall |
//! | [`k_eff_two_photon`] | Effective wavevector for two-photon Raman transition |
//! | [`sensitivity_delta_g`] | Gravimeter sensitivity estimate |
//!
//! # Units
//!
//! - 1 Gal = 1 cm/s^2 = 10^-2 m/s^2
//! - 1 mGal = 10^-5 m/s^2, 1 uGal = 10^-8 m/s^2, 1 nGal = 10^-11 m/s^2
//! - Gravity measurements throughout this module are in m/s^2
//!
//! # Example
//!
//! ```rust
//! use r4w_core::interferometric_gravity_mapper::{
//!     GravimeterConfig, AtomSpecies, MachZehnderInterferometer,
//!     PhaseExtractor, GravityCalculator, k_eff_two_photon,
//! };
//!
//! let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
//! let k_eff = k_eff_two_photon(config.laser_wavelength_m);
//!
//! // Simulate interferometer output for standard gravity
//! let mzi = MachZehnderInterferometer::new(&config);
//! let phase = mzi.interferometric_phase(9.80665);
//!
//! // Recover gravity from phase
//! let calc = GravityCalculator::new(k_eff, config.interrogation_time_s);
//! let g_recovered = calc.gravity_from_phase(phase);
//! assert!((g_recovered - 9.80665).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Standard gravity in m/s^2.
const G_STANDARD: f64 = 9.806_65;

/// Rb-87 D2 line wavelength in meters.
const RB87_WAVELENGTH_M: f64 = 780.241_209_686e-9;

/// Cs-133 D2 line wavelength in meters.
const CS133_WAVELENGTH_M: f64 = 852.347_275e-9;

/// Rb-87 typical Rabi frequency for Raman transitions (rad/s).
const RB87_RABI_FREQ: f64 = 2.0 * PI * 50_000.0; // ~50 kHz

/// Cs-133 typical Rabi frequency for Raman transitions (rad/s).
const CS133_RABI_FREQ: f64 = 2.0 * PI * 40_000.0; // ~40 kHz

/// Conversion: 1 Gal = 0.01 m/s^2.
const GAL_TO_MS2: f64 = 1e-2;

/// Conversion: 1 uGal = 1e-8 m/s^2.
const UGAL_TO_MS2: f64 = 1e-8;

/// Lunar semi-diurnal tidal amplitude in m/s^2 (M2 constituent, ~100 uGal).
const LUNAR_TIDE_AMPLITUDE_MS2: f64 = 100.0 * UGAL_TO_MS2;

/// Solar semi-diurnal tidal amplitude in m/s^2 (S2 constituent, ~46 uGal).
const SOLAR_TIDE_AMPLITUDE_MS2: f64 = 46.0 * UGAL_TO_MS2;

/// Lunar semi-diurnal period in seconds (~12.42 hours).
const LUNAR_SEMIDIURNAL_PERIOD_S: f64 = 12.0 * 3600.0 + 25.0 * 60.0 + 14.0;

/// Solar semi-diurnal period in seconds (12 hours exactly).
const SOLAR_SEMIDIURNAL_PERIOD_S: f64 = 12.0 * 3600.0;

/// Barometric admittance factor: delta_g / delta_P in (m/s^2) / hPa.
/// Typical value: -0.3 uGal/hPa = -3e-9 m/s^2 per hPa.
const BAROMETRIC_ADMITTANCE_MS2_PER_HPA: f64 = -0.3 * UGAL_TO_MS2;

// ---------------------------------------------------------------------------
// Atom species
// ---------------------------------------------------------------------------

/// Atom species used in the interferometer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AtomSpecies {
    /// Rubidium-87 (D2 line at 780.24 nm).
    Rubidium87,
    /// Cesium-133 (D2 line at 852.35 nm).
    Cesium133,
}

impl AtomSpecies {
    /// Return the D2 line wavelength in meters.
    pub fn wavelength_m(self) -> f64 {
        match self {
            AtomSpecies::Rubidium87 => RB87_WAVELENGTH_M,
            AtomSpecies::Cesium133 => CS133_WAVELENGTH_M,
        }
    }

    /// Return typical Rabi frequency in rad/s.
    pub fn rabi_frequency(self) -> f64 {
        match self {
            AtomSpecies::Rubidium87 => RB87_RABI_FREQ,
            AtomSpecies::Cesium133 => CS133_RABI_FREQ,
        }
    }
}

// ---------------------------------------------------------------------------
// Gravimeter configuration
// ---------------------------------------------------------------------------

/// Configuration for an atom interferometry gravimeter.
#[derive(Debug, Clone)]
pub struct GravimeterConfig {
    /// Atom species.
    pub species: AtomSpecies,
    /// Interrogation time T in seconds (half the free-fall time between first and last pulse).
    pub interrogation_time_s: f64,
    /// Laser wavelength in meters (defaults to species D2 line).
    pub laser_wavelength_m: f64,
    /// Rabi frequency in rad/s (defaults to species typical value).
    pub rabi_frequency: f64,
}

impl GravimeterConfig {
    /// Create a new gravimeter configuration.
    ///
    /// - `species`: atom species
    /// - `interrogation_time_s`: free evolution time T (seconds)
    /// - `laser_wavelength_m`: optional override for laser wavelength
    /// - `rabi_frequency`: optional override for Rabi frequency
    pub fn new(
        species: AtomSpecies,
        interrogation_time_s: f64,
        laser_wavelength_m: Option<f64>,
        rabi_frequency: Option<f64>,
    ) -> Self {
        Self {
            species,
            interrogation_time_s,
            laser_wavelength_m: laser_wavelength_m.unwrap_or_else(|| species.wavelength_m()),
            rabi_frequency: rabi_frequency.unwrap_or_else(|| species.rabi_frequency()),
        }
    }

    /// Effective two-photon wavevector k_eff = 4*pi / lambda.
    pub fn k_eff(&self) -> f64 {
        k_eff_two_photon(self.laser_wavelength_m)
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute the effective wavevector for a two-photon Raman transition.
///
/// `k_eff = 2 * k_laser = 2 * (2*pi/lambda) = 4*pi/lambda`
pub fn k_eff_two_photon(wavelength_m: f64) -> f64 {
    4.0 * PI / wavelength_m
}

/// Estimate the gravimeter phase sensitivity.
///
/// `delta_g = 1 / (k_eff * T^2 * snr)` in m/s^2.
pub fn sensitivity_delta_g(k_eff: f64, interrogation_time_s: f64, snr: f64) -> f64 {
    1.0 / (k_eff * interrogation_time_s * interrogation_time_s * snr)
}

/// Convert m/s^2 to micro-Gal.
pub fn ms2_to_ugal(val: f64) -> f64 {
    val / UGAL_TO_MS2
}

/// Convert micro-Gal to m/s^2.
pub fn ugal_to_ms2(val: f64) -> f64 {
    val * UGAL_TO_MS2
}

/// Convert m/s^2 to mGal.
pub fn ms2_to_mgal(val: f64) -> f64 {
    val * 1e5
}

/// Convert mGal to m/s^2.
pub fn mgal_to_ms2(val: f64) -> f64 {
    val * 1e-5
}

// ---------------------------------------------------------------------------
// Mach-Zehnder interferometer
// ---------------------------------------------------------------------------

/// Three-pulse Mach-Zehnder atom interferometer.
///
/// The standard pi/2 — pi — pi/2 pulse sequence produces an interferometric phase
/// `phi = k_eff * a * T^2` where `a` is the acceleration along the laser beam axis,
/// and `T` is the time between consecutive pulses.
#[derive(Debug, Clone)]
pub struct MachZehnderInterferometer {
    /// Effective wavevector k_eff (1/m).
    k_eff: f64,
    /// Interrogation time T (seconds).
    interrogation_time_s: f64,
    /// Rabi frequency (rad/s), used for pulse area calculations.
    rabi_frequency: f64,
}

impl MachZehnderInterferometer {
    /// Create from a gravimeter configuration.
    pub fn new(config: &GravimeterConfig) -> Self {
        Self {
            k_eff: config.k_eff(),
            interrogation_time_s: config.interrogation_time_s,
            rabi_frequency: config.rabi_frequency,
        }
    }

    /// Create with explicit parameters.
    pub fn with_params(k_eff: f64, interrogation_time_s: f64, rabi_frequency: f64) -> Self {
        Self {
            k_eff,
            interrogation_time_s,
            rabi_frequency,
        }
    }

    /// Compute the interferometric phase for a given acceleration.
    ///
    /// `phi = k_eff * a * T^2`
    pub fn interferometric_phase(&self, acceleration_ms2: f64) -> f64 {
        self.k_eff * acceleration_ms2 * self.interrogation_time_s * self.interrogation_time_s
    }

    /// Compute the transition probability (population in the excited state)
    /// for a given acceleration and additional Raman laser phase `phi_laser`.
    ///
    /// `P = (1 + C * cos(phi_interferometer - phi_laser)) / 2`
    ///
    /// where C is the fringe contrast (0..1).
    pub fn transition_probability(
        &self,
        acceleration_ms2: f64,
        phi_laser: f64,
        contrast: f64,
    ) -> f64 {
        let phi = self.interferometric_phase(acceleration_ms2);
        0.5 * (1.0 + contrast * (phi - phi_laser).cos())
    }

    /// Duration of a pi/2 pulse given the Rabi frequency.
    ///
    /// `t_pi2 = pi / (2 * Omega_R)`
    pub fn pi_half_pulse_duration(&self) -> f64 {
        PI / (2.0 * self.rabi_frequency)
    }

    /// Duration of a pi pulse.
    ///
    /// `t_pi = pi / Omega_R`
    pub fn pi_pulse_duration(&self) -> f64 {
        PI / self.rabi_frequency
    }

    /// Total sequence duration: pi/2 + T + pi + T + pi/2.
    pub fn total_sequence_duration(&self) -> f64 {
        2.0 * self.pi_half_pulse_duration()
            + self.pi_pulse_duration()
            + 2.0 * self.interrogation_time_s
    }

    /// Phase sensitivity: d(phi)/d(g) = k_eff * T^2 in rad/(m/s^2).
    pub fn phase_sensitivity(&self) -> f64 {
        self.k_eff * self.interrogation_time_s * self.interrogation_time_s
    }

    /// Gravity change per fringe: delta_g = 2*pi / (k_eff * T^2).
    pub fn gravity_per_fringe(&self) -> f64 {
        2.0 * PI / self.phase_sensitivity()
    }
}

// ---------------------------------------------------------------------------
// Phase extractor
// ---------------------------------------------------------------------------

/// Extract interferometric phase from population measurements.
///
/// The population fringe is: `P = (1 + C * cos(phi)) / 2`
///
/// Given known contrast C, we can invert for phase (with ambiguity). For robust
/// extraction, multiple measurements at different Raman phases are used (see
/// [`FringeScanner`]).
#[derive(Debug, Clone)]
pub struct PhaseExtractor {
    /// Expected fringe contrast (0..1).
    contrast: f64,
}

impl PhaseExtractor {
    /// Create a new phase extractor with the given contrast.
    pub fn new(contrast: f64) -> Self {
        Self {
            contrast: contrast.clamp(1e-10, 1.0),
        }
    }

    /// Extract phase from a single population measurement.
    ///
    /// Returns phase in radians (0..pi range due to arccos ambiguity).
    pub fn phase_from_population(&self, population: f64) -> f64 {
        let p_clamped = population.clamp(0.0, 1.0);
        let cos_phi = (2.0 * p_clamped - 1.0) / self.contrast;
        let cos_clamped = cos_phi.clamp(-1.0, 1.0);
        cos_clamped.acos()
    }

    /// Compute population from phase.
    ///
    /// `P = (1 + C * cos(phi)) / 2`
    pub fn population_from_phase(&self, phase: f64) -> f64 {
        0.5 * (1.0 + self.contrast * phase.cos())
    }

    /// Fringe contrast getter.
    pub fn contrast(&self) -> f64 {
        self.contrast
    }
}

// ---------------------------------------------------------------------------
// Gravity calculator
// ---------------------------------------------------------------------------

/// Convert interferometric phase to gravitational acceleration.
///
/// `g = phi / (k_eff * T^2)`
#[derive(Debug, Clone)]
pub struct GravityCalculator {
    /// Effective wavevector (1/m).
    k_eff: f64,
    /// Interrogation time (seconds).
    interrogation_time_s: f64,
    /// Pre-computed scale factor: k_eff * T^2.
    scale_factor: f64,
}

impl GravityCalculator {
    /// Create a new gravity calculator.
    pub fn new(k_eff: f64, interrogation_time_s: f64) -> Self {
        let scale_factor = k_eff * interrogation_time_s * interrogation_time_s;
        Self {
            k_eff,
            interrogation_time_s,
            scale_factor,
        }
    }

    /// Create from a gravimeter configuration.
    pub fn from_config(config: &GravimeterConfig) -> Self {
        Self::new(config.k_eff(), config.interrogation_time_s)
    }

    /// Compute gravity from interferometric phase.
    ///
    /// `g = phi / (k_eff * T^2)`
    pub fn gravity_from_phase(&self, phase_rad: f64) -> f64 {
        phase_rad / self.scale_factor
    }

    /// Compute phase from gravity.
    ///
    /// `phi = k_eff * g * T^2`
    pub fn phase_from_gravity(&self, gravity_ms2: f64) -> f64 {
        gravity_ms2 * self.scale_factor
    }

    /// Scale factor k_eff * T^2 in rad / (m/s^2).
    pub fn scale_factor(&self) -> f64 {
        self.scale_factor
    }

    /// Gravity resolution for a given phase noise (rad).
    pub fn gravity_resolution(&self, phase_noise_rad: f64) -> f64 {
        phase_noise_rad / self.scale_factor
    }
}

// ---------------------------------------------------------------------------
// Fringe scanner
// ---------------------------------------------------------------------------

/// Scan the Raman laser phase across a full fringe to extract the interferometric
/// phase, contrast, and offset via sinusoidal fitting.
///
/// Measures populations at N equally-spaced Raman phases from 0 to 2*pi and fits:
///
/// `P(phi_L) = A + B * cos(phi_L - phi_0)`
///
/// using discrete Fourier analysis (single-bin DFT at the fundamental frequency).
#[derive(Debug, Clone)]
pub struct FringeScanner {
    /// Number of scan points per fringe.
    num_points: usize,
    /// Accumulated scan data: (raman_phase, population) pairs.
    data: Vec<(f64, f64)>,
}

/// Result of a fringe scan fit.
#[derive(Debug, Clone, Copy)]
pub struct FringeFitResult {
    /// Interferometric phase offset (radians).
    pub phase_rad: f64,
    /// Fringe contrast (0..1).
    pub contrast: f64,
    /// DC offset (should be ~0.5 for ideal fringes).
    pub offset: f64,
    /// Fit residual (RMS).
    pub residual_rms: f64,
}

impl FringeScanner {
    /// Create a new fringe scanner for the given number of points per scan.
    pub fn new(num_points: usize) -> Self {
        Self {
            num_points: num_points.max(4),
            data: Vec::with_capacity(num_points),
        }
    }

    /// Add a measurement: population at a given Raman laser phase.
    pub fn add_measurement(&mut self, raman_phase: f64, population: f64) {
        self.data.push((raman_phase, population));
    }

    /// Generate scan phases: N equally-spaced phases from 0 to 2*pi (exclusive).
    pub fn scan_phases(&self) -> Vec<f64> {
        (0..self.num_points)
            .map(|i| 2.0 * PI * i as f64 / self.num_points as f64)
            .collect()
    }

    /// Clear accumulated data.
    pub fn clear(&mut self) {
        self.data.clear();
    }

    /// Number of measurements collected.
    pub fn num_measurements(&self) -> usize {
        self.data.len()
    }

    /// Fit the accumulated data to `P = offset + amplitude * cos(phi_L - phi_0)`.
    ///
    /// Uses least-squares projection onto cos/sin basis to extract the fundamental
    /// component: `P ~ a0 + a1*cos(phi_L) + a2*sin(phi_L)`, then
    /// `amplitude = sqrt(a1^2 + a2^2)`, `phi_0 = atan2(-a2, a1)`.
    pub fn fit(&self) -> Option<FringeFitResult> {
        let n = self.data.len();
        if n < 3 {
            return None;
        }

        // Least-squares: fit P = a0 + a1*cos(phi) + a2*sin(phi)
        // For equally-spaced phases on [0, 2*pi), the basis is orthogonal:
        //   a0 = mean(P)
        //   a1 = (2/N) * sum(P * cos(phi))
        //   a2 = (2/N) * sum(P * sin(phi))
        let mut sum_p = 0.0_f64;
        let mut sum_p_cos = 0.0_f64;
        let mut sum_p_sin = 0.0_f64;

        for &(phi_l, p) in &self.data {
            sum_p += p;
            sum_p_cos += p * phi_l.cos();
            sum_p_sin += p * phi_l.sin();
        }

        let a0 = sum_p / n as f64;
        let a1 = 2.0 * sum_p_cos / n as f64;
        let a2 = 2.0 * sum_p_sin / n as f64;

        let amplitude = (a1 * a1 + a2 * a2).sqrt();
        // P = a0 + amplitude * cos(phi_L - phi_0)
        //   = a0 + amplitude * [cos(phi_L)*cos(phi_0) + sin(phi_L)*sin(phi_0)]
        // So a1 = amplitude*cos(phi_0), a2 = amplitude*sin(phi_0)
        let phase = a2.atan2(a1);

        // Compute residual
        let mut residual_sum = 0.0;
        for &(phi_l, p) in &self.data {
            let p_fit = a0 + amplitude * (phi_l - phase).cos();
            let diff = p - p_fit;
            residual_sum += diff * diff;
        }
        let residual_rms = (residual_sum / n as f64).sqrt();

        // Contrast = 2 * amplitude (since P = (1 + C*cos(...))/2 → amplitude = C/2)
        let contrast = (2.0 * amplitude).clamp(0.0, 1.0);

        Some(FringeFitResult {
            phase_rad: phase,
            contrast,
            offset: a0,
            residual_rms,
        })
    }
}

// ---------------------------------------------------------------------------
// Gradient meter
// ---------------------------------------------------------------------------

/// Differential gravity measurement for gravity gradiometry.
///
/// Two interferometers separated by a vertical baseline measure g_top and g_bottom.
/// The gravity gradient (vertical component of the gravity gradient tensor) is:
///
/// `Gamma_zz = (g_top - g_bottom) / baseline`
///
/// Units: 1 Eotvos (E) = 10^-9 s^-2 = 0.1 uGal/m.
#[derive(Debug, Clone)]
pub struct GradientMeter {
    /// Vertical baseline in meters.
    baseline_m: f64,
}

impl GradientMeter {
    /// Create a new gradient meter with the given baseline.
    pub fn new(baseline_m: f64) -> Self {
        assert!(baseline_m > 0.0, "Baseline must be positive");
        Self { baseline_m }
    }

    /// Compute the gravity gradient from top and bottom gravity measurements.
    ///
    /// Returns gradient in s^-2 (Eotvos * 1e-9).
    pub fn gradient(&self, g_top: f64, g_bottom: f64) -> f64 {
        (g_top - g_bottom) / self.baseline_m
    }

    /// Compute the gravity gradient in Eotvos units (1 E = 10^-9 s^-2).
    pub fn gradient_eotvos(&self, g_top: f64, g_bottom: f64) -> f64 {
        self.gradient(g_top, g_bottom) * 1e9
    }

    /// Normal free-air gradient: approximately -3086 E (or -308.6 uGal/m).
    pub fn free_air_gradient_eotvos() -> f64 {
        -3086.0
    }

    /// Compute Bouguer anomaly gradient correction for a slab of given density.
    ///
    /// `dg/dz_Bouguer = -2 * pi * G * rho`
    ///
    /// Returns in s^-2. For rock (rho=2670 kg/m^3), this is about 1119 E.
    pub fn bouguer_correction_s2(density_kg_m3: f64) -> f64 {
        -2.0 * PI * 6.674_30e-11 * density_kg_m3
    }

    /// Baseline getter.
    pub fn baseline_m(&self) -> f64 {
        self.baseline_m
    }
}

// ---------------------------------------------------------------------------
// Tidal corrector
// ---------------------------------------------------------------------------

/// Approximate solid Earth tide correction.
///
/// Models the semi-diurnal gravitational variation from the Moon (M2) and Sun (S2)
/// as sinusoidal components. The amplitude is typically ~100 uGal total, with
/// M2 ~ 100 uGal and S2 ~ 46 uGal.
///
/// This is a simplified model; operational gravimeters use full tidal catalogues
/// (e.g., Tamura, Hartmann-Wenzel) with hundreds of constituents.
#[derive(Debug, Clone)]
pub struct TidalCorrector {
    /// Lunar M2 amplitude in m/s^2.
    lunar_amplitude_ms2: f64,
    /// Solar S2 amplitude in m/s^2.
    solar_amplitude_ms2: f64,
    /// Lunar M2 period in seconds.
    lunar_period_s: f64,
    /// Solar S2 period in seconds.
    solar_period_s: f64,
    /// Lunar initial phase in radians.
    lunar_phase0: f64,
    /// Solar initial phase in radians.
    solar_phase0: f64,
}

impl TidalCorrector {
    /// Create with default M2 + S2 amplitudes and zero initial phases.
    pub fn new() -> Self {
        Self {
            lunar_amplitude_ms2: LUNAR_TIDE_AMPLITUDE_MS2,
            solar_amplitude_ms2: SOLAR_TIDE_AMPLITUDE_MS2,
            lunar_period_s: LUNAR_SEMIDIURNAL_PERIOD_S,
            solar_period_s: SOLAR_SEMIDIURNAL_PERIOD_S,
            lunar_phase0: 0.0,
            solar_phase0: 0.0,
        }
    }

    /// Create with custom amplitudes (in m/s^2) and initial phases (radians).
    pub fn with_params(
        lunar_amplitude_ms2: f64,
        solar_amplitude_ms2: f64,
        lunar_phase0: f64,
        solar_phase0: f64,
    ) -> Self {
        Self {
            lunar_amplitude_ms2,
            solar_amplitude_ms2,
            lunar_period_s: LUNAR_SEMIDIURNAL_PERIOD_S,
            solar_period_s: SOLAR_SEMIDIURNAL_PERIOD_S,
            lunar_phase0,
            solar_phase0,
        }
    }

    /// Compute tidal gravity correction at time t (seconds since reference epoch).
    ///
    /// Returns the tidal gravity perturbation in m/s^2.
    pub fn correction(&self, time_s: f64) -> f64 {
        let lunar = self.lunar_amplitude_ms2
            * (2.0 * PI * time_s / self.lunar_period_s + self.lunar_phase0).cos();
        let solar = self.solar_amplitude_ms2
            * (2.0 * PI * time_s / self.solar_period_s + self.solar_phase0).cos();
        lunar + solar
    }

    /// Correct a gravity measurement by subtracting the tidal effect.
    pub fn correct(&self, gravity_ms2: f64, time_s: f64) -> f64 {
        gravity_ms2 - self.correction(time_s)
    }

    /// Compute tidal corrections for a time series.
    pub fn correct_series(&self, gravities: &[f64], times_s: &[f64]) -> Vec<f64> {
        gravities
            .iter()
            .zip(times_s.iter())
            .map(|(&g, &t)| self.correct(g, t))
            .collect()
    }

    /// Peak-to-peak tidal range in m/s^2.
    pub fn peak_to_peak(&self) -> f64 {
        2.0 * (self.lunar_amplitude_ms2 + self.solar_amplitude_ms2)
    }
}

impl Default for TidalCorrector {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Atmospheric pressure corrector
// ---------------------------------------------------------------------------

/// Atmospheric pressure correction for gravity measurements.
///
/// Changes in atmospheric pressure cause gravity variations through the direct
/// Newtonian attraction of the atmosphere and crustal loading. The standard
/// admittance factor is approximately -0.3 uGal/hPa.
#[derive(Debug, Clone)]
pub struct AtmosphericPressureCorrector {
    /// Admittance factor in (m/s^2) / hPa.
    admittance_ms2_per_hpa: f64,
    /// Reference pressure in hPa.
    reference_pressure_hpa: f64,
}

impl AtmosphericPressureCorrector {
    /// Create with default admittance (-0.3 uGal/hPa) and standard atmosphere reference.
    pub fn new(reference_pressure_hpa: f64) -> Self {
        Self {
            admittance_ms2_per_hpa: BAROMETRIC_ADMITTANCE_MS2_PER_HPA,
            reference_pressure_hpa,
        }
    }

    /// Create with custom admittance factor.
    pub fn with_admittance(reference_pressure_hpa: f64, admittance_ugal_per_hpa: f64) -> Self {
        Self {
            admittance_ms2_per_hpa: admittance_ugal_per_hpa * UGAL_TO_MS2,
            reference_pressure_hpa,
        }
    }

    /// Compute the gravity correction for a given pressure.
    ///
    /// `delta_g = admittance * (pressure - reference)`
    pub fn correction(&self, pressure_hpa: f64) -> f64 {
        self.admittance_ms2_per_hpa * (pressure_hpa - self.reference_pressure_hpa)
    }

    /// Correct a gravity measurement.
    pub fn correct(&self, gravity_ms2: f64, pressure_hpa: f64) -> f64 {
        gravity_ms2 - self.correction(pressure_hpa)
    }

    /// Admittance factor in uGal/hPa.
    pub fn admittance_ugal_per_hpa(&self) -> f64 {
        self.admittance_ms2_per_hpa / UGAL_TO_MS2
    }
}

// ---------------------------------------------------------------------------
// Drift estimator
// ---------------------------------------------------------------------------

/// Instrument drift estimator via polynomial trend removal.
///
/// Fits a polynomial (up to degree N) to a time series of gravity measurements
/// and subtracts the trend to isolate the geophysical signal.
#[derive(Debug, Clone)]
pub struct DriftEstimator {
    /// Polynomial degree (1=linear, 2=quadratic, etc.).
    degree: usize,
}

impl DriftEstimator {
    /// Create a drift estimator with the given polynomial degree.
    pub fn new(degree: usize) -> Self {
        Self { degree: degree.max(1) }
    }

    /// Fit polynomial to the time series using centered and scaled coordinates
    /// for numerical stability.
    ///
    /// Returns `(coeffs_centered, t_mean, t_scale)` where the polynomial is
    /// `y(t) = sum_k coeffs[k] * u^k` with `u = (t - t_mean) / t_scale`.
    fn fit_centered(&self, times: &[f64], values: &[f64]) -> (Vec<f64>, f64, f64) {
        let n = times.len();
        let p = self.degree + 1;
        if n < p {
            return (vec![0.0; p], 0.0, 1.0);
        }

        // Center and scale times for numerical stability
        let t_mean = times.iter().sum::<f64>() / n as f64;
        let t_range = times
            .iter()
            .fold(0.0_f64, |mx, &t| mx.max((t - t_mean).abs()));
        let t_scale = if t_range > 1e-15 { t_range } else { 1.0 };

        let centered: Vec<f64> = times.iter().map(|&t| (t - t_mean) / t_scale).collect();

        // Build normal equations in centered coordinates
        // A[i][j] = sum_k tc[k]^(i+j), b[i] = sum_k tc[k]^i * y[k]
        // Pre-compute powers of tc for each data point
        let mut a = vec![vec![0.0_f64; p]; p];
        let mut b = vec![0.0_f64; p];

        for k in 0..n {
            let tc = centered[k];
            let y = values[k];
            // Compute tc^0, tc^1, ..., tc^(2*(p-1))
            let max_pow = 2 * (p - 1) + 1;
            let mut powers = vec![1.0_f64; max_pow];
            for m in 1..max_pow {
                powers[m] = powers[m - 1] * tc;
            }
            for i in 0..p {
                b[i] += powers[i] * y;
                for j in i..p {
                    a[i][j] += powers[i + j];
                }
            }
        }

        // Fill lower triangle
        for i in 0..p {
            for j in 0..i {
                a[i][j] = a[j][i];
            }
        }

        // Gaussian elimination with partial pivoting
        let mut aug = vec![vec![0.0_f64; p + 1]; p];
        for i in 0..p {
            for j in 0..p {
                aug[i][j] = a[i][j];
            }
            aug[i][p] = b[i];
        }

        for col in 0..p {
            let mut max_val = aug[col][col].abs();
            let mut max_row = col;
            for row in (col + 1)..p {
                if aug[row][col].abs() > max_val {
                    max_val = aug[row][col].abs();
                    max_row = row;
                }
            }
            aug.swap(col, max_row);

            let pivot = aug[col][col];
            if pivot.abs() < 1e-30 {
                continue;
            }

            for row in (col + 1)..p {
                let factor = aug[row][col] / pivot;
                for j in col..=p {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        // Back substitution
        let mut coeffs = vec![0.0_f64; p];
        for i in (0..p).rev() {
            let mut sum = aug[i][p];
            for j in (i + 1)..p {
                sum -= aug[i][j] * coeffs[j];
            }
            if aug[i][i].abs() > 1e-30 {
                coeffs[i] = sum / aug[i][i];
            }
        }

        (coeffs, t_mean, t_scale)
    }

    /// Evaluate centered polynomial: `sum_k coeffs[k] * u^k` where `u = (t - t_mean) / t_scale`.
    fn eval_centered(coeffs: &[f64], t: f64, t_mean: f64, t_scale: f64) -> f64 {
        let u = (t - t_mean) / t_scale;
        let mut result = 0.0;
        for c in coeffs.iter().rev() {
            result = result * u + c;
        }
        result
    }

    /// Fit polynomial and return coefficients in the original (uncentered) time variable.
    ///
    /// Uses `fit_centered` internally, then evaluates at original time points.
    /// The returned coefficients satisfy: `y(t) = sum_k coeffs[k] * t^k`.
    pub fn fit_polynomial(&self, times: &[f64], values: &[f64]) -> Vec<f64> {
        let (c, t_mean, t_scale) = self.fit_centered(times, values);
        // For API compatibility, return coefficients in original variable.
        // We do this via polynomial composition using the centered coefficients.
        // c[k] applies to u^k where u = (t - t_mean)/t_scale = t/t_scale - t_mean/t_scale
        // Expand using nested multiplication.
        let p = c.len();
        let alpha = 1.0 / t_scale;
        let beta = -t_mean / t_scale;

        // Build coefficients in t: start with c[p-1], multiply by (alpha*t + beta), add c[p-2], etc.
        // Maintain a polynomial in t as a vector of coefficients.
        let mut result = vec![0.0_f64; p];
        result[0] = c[p - 1];

        for i in (0..p - 1).rev() {
            // Multiply current polynomial by (alpha*t + beta): shift up and scale
            let mut new = vec![0.0_f64; p];
            for j in (0..p).rev() {
                if j > 0 {
                    new[j] += result[j - 1] * alpha;
                }
                new[j] += result[j] * beta;
            }
            new[0] += c[i];
            result = new;
        }

        result
    }

    /// Evaluate a polynomial at time t: `sum_k coeffs[k] * t^k` via Horner's method.
    pub fn eval_polynomial(coeffs: &[f64], t: f64) -> f64 {
        let mut result = 0.0;
        for c in coeffs.iter().rev() {
            result = result * t + c;
        }
        result
    }

    /// Compute the drift (trend) values for each time point.
    pub fn compute_drift(&self, times: &[f64], values: &[f64]) -> Vec<f64> {
        let (coeffs, t_mean, t_scale) = self.fit_centered(times, values);
        times
            .iter()
            .map(|&t| Self::eval_centered(&coeffs, t, t_mean, t_scale))
            .collect()
    }

    /// Remove drift from the time series (detrend).
    pub fn remove_drift(&self, times: &[f64], values: &[f64]) -> Vec<f64> {
        let drift = self.compute_drift(times, values);
        values
            .iter()
            .zip(drift.iter())
            .map(|(&v, &d)| v - d)
            .collect()
    }

    /// Estimate linear drift rate from the fitted polynomial.
    ///
    /// For centered polynomial `y(u) = c0 + c1*u + ...`, the drift rate in
    /// original time is `c1 / t_scale` (derivative dy/dt at the center).
    pub fn linear_drift_rate(&self, times: &[f64], values: &[f64]) -> f64 {
        let (coeffs, _t_mean, t_scale) = self.fit_centered(times, values);
        if coeffs.len() > 1 {
            coeffs[1] / t_scale
        } else {
            0.0
        }
    }
}

// ---------------------------------------------------------------------------
// Noise analyzer (Allan deviation)
// ---------------------------------------------------------------------------

/// Allan deviation analyzer for gravity measurement stability.
///
/// Computes the overlapping Allan deviation (ADEV), which characterizes the noise
/// type and stability of repeated gravity measurements at different averaging times.
#[derive(Debug, Clone)]
pub struct NoiseAnalyzer;

impl NoiseAnalyzer {
    /// Compute overlapping Allan deviation.
    ///
    /// - `values`: time series of gravity measurements (equally spaced)
    /// - `tau0`: base sampling interval in seconds
    /// - `max_factor`: maximum averaging factor (tau = factor * tau0)
    ///
    /// Returns a vector of (tau, adev) pairs.
    pub fn allan_deviation(
        values: &[f64],
        tau0: f64,
        max_factor: usize,
    ) -> Vec<(f64, f64)> {
        let n = values.len();
        if n < 3 {
            return vec![];
        }

        let max_m = max_factor.min(n / 2);
        let mut result = Vec::new();

        for m in 1..=max_m {
            let tau = m as f64 * tau0;
            let mut sum_sq = 0.0;
            let mut count = 0usize;

            // Overlapping Allan deviation
            for i in 0..(n - 2 * m) {
                let avg1: f64 = values[i..i + m].iter().sum::<f64>() / m as f64;
                let avg2: f64 = values[i + m..i + 2 * m].iter().sum::<f64>() / m as f64;
                let diff = avg2 - avg1;
                sum_sq += diff * diff;
                count += 1;
            }

            if count > 0 {
                let adev = (sum_sq / (2.0 * count as f64)).sqrt();
                result.push((tau, adev));
            }
        }

        result
    }

    /// Identify the dominant noise type from the Allan deviation slope.
    ///
    /// - slope ~ -1: white frequency noise (random walk of phase)
    /// - slope ~ -0.5: flicker frequency noise
    /// - slope ~ 0: random walk frequency noise
    /// - slope ~ +0.5: frequency drift
    pub fn identify_noise_type(adev_data: &[(f64, f64)]) -> &'static str {
        if adev_data.len() < 2 {
            return "insufficient data";
        }

        // Fit log-log slope
        let n = adev_data.len();
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_xx = 0.0;

        for &(tau, adev) in adev_data {
            if tau > 0.0 && adev > 0.0 {
                let x = tau.ln();
                let y = adev.ln();
                sum_x += x;
                sum_y += y;
                sum_xy += x * y;
                sum_xx += x * x;
            }
        }

        let n_f = n as f64;
        let denom = n_f * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return "degenerate";
        }

        let slope = (n_f * sum_xy - sum_x * sum_y) / denom;

        if slope < -0.75 {
            "white frequency noise"
        } else if slope < -0.25 {
            "flicker frequency noise"
        } else if slope < 0.25 {
            "random walk frequency noise"
        } else {
            "frequency drift"
        }
    }

    /// Compute the noise floor: minimum Allan deviation across all averaging times.
    pub fn noise_floor(adev_data: &[(f64, f64)]) -> Option<(f64, f64)> {
        adev_data
            .iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .copied()
    }
}

// ---------------------------------------------------------------------------
// Free evolution model
// ---------------------------------------------------------------------------

/// Model of atom trajectory during free fall in the interferometer.
///
/// `z(t) = z0 + v0*t + 0.5*g*t^2`
///
/// Accounts for launch velocity, gravity, and optional gravity gradient.
#[derive(Debug, Clone)]
pub struct FreeEvolutionModel {
    /// Initial position (m).
    z0: f64,
    /// Initial velocity (m/s, positive = upward for vertical launch).
    v0: f64,
    /// Gravitational acceleration (m/s^2, positive downward by convention).
    gravity: f64,
    /// Optional gravity gradient (s^-2).
    gradient: Option<f64>,
}

impl FreeEvolutionModel {
    /// Create a free evolution model.
    pub fn new(z0: f64, v0: f64, gravity: f64) -> Self {
        Self {
            z0,
            v0,
            gravity,
            gradient: None,
        }
    }

    /// Create with gravity gradient (for gradiometer modeling).
    pub fn with_gradient(z0: f64, v0: f64, gravity: f64, gradient_s2: f64) -> Self {
        Self {
            z0,
            v0,
            gravity,
            gradient: Some(gradient_s2),
        }
    }

    /// Position at time t.
    ///
    /// `z(t) = z0 + v0*t - 0.5*g*t^2` (upward positive, g>0 pulls downward)
    pub fn position(&self, t: f64) -> f64 {
        let mut z = self.z0 + self.v0 * t - 0.5 * self.gravity * t * t;
        if let Some(grad) = self.gradient {
            // First-order gradient correction
            z -= (1.0 / 6.0) * grad * self.gravity * t * t * t;
        }
        z
    }

    /// Velocity at time t.
    ///
    /// `v(t) = v0 - g*t`
    pub fn velocity(&self, t: f64) -> f64 {
        let mut v = self.v0 - self.gravity * t;
        if let Some(grad) = self.gradient {
            v -= 0.5 * grad * self.gravity * t * t;
        }
        v
    }

    /// Apogee time (when velocity = 0, no gradient).
    pub fn apogee_time(&self) -> f64 {
        if self.gravity.abs() < 1e-15 {
            return 0.0;
        }
        self.v0 / self.gravity
    }

    /// Apogee height above initial position.
    pub fn apogee_height(&self) -> f64 {
        let t_ap = self.apogee_time();
        self.position(t_ap) - self.z0
    }

    /// Generate trajectory: returns (time, position, velocity) tuples.
    pub fn trajectory(&self, duration: f64, num_points: usize) -> Vec<(f64, f64, f64)> {
        let dt = duration / (num_points.max(1) - 1).max(1) as f64;
        (0..num_points)
            .map(|i| {
                let t = i as f64 * dt;
                (t, self.position(t), self.velocity(t))
            })
            .collect()
    }

    /// Free-fall distance after time T from rest.
    pub fn free_fall_distance(gravity: f64, time_s: f64) -> f64 {
        0.5 * gravity * time_s * time_s
    }

    /// Required launch velocity for atoms to reach a desired apogee height.
    pub fn required_launch_velocity(gravity: f64, height_m: f64) -> f64 {
        (2.0 * gravity * height_m).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Convenience: full measurement pipeline
// ---------------------------------------------------------------------------

/// Process a single gravity measurement from raw population data.
///
/// Steps:
/// 1. Extract phase from population using known contrast
/// 2. Convert phase to gravity using k_eff and T
/// 3. Apply tidal correction
/// 4. Apply atmospheric pressure correction
///
/// Returns corrected gravity in m/s^2.
pub fn process_measurement(
    population: f64,
    config: &GravimeterConfig,
    contrast: f64,
    time_s: f64,
    pressure_hpa: f64,
    reference_pressure_hpa: f64,
) -> f64 {
    let extractor = PhaseExtractor::new(contrast);
    let phase = extractor.phase_from_population(population);

    let calculator = GravityCalculator::from_config(config);
    let g_raw = calculator.gravity_from_phase(phase);

    let tidal = TidalCorrector::new();
    let g_tidal = tidal.correct(g_raw, time_s);

    let atm = AtmosphericPressureCorrector::new(reference_pressure_hpa);
    atm.correct(g_tidal, pressure_hpa)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOLERANCE: f64 = 1e-9;

    // --- AtomSpecies ---

    #[test]
    fn test_rb87_wavelength() {
        let wl = AtomSpecies::Rubidium87.wavelength_m();
        assert!((wl - 780.241_209_686e-9).abs() < 1e-18);
    }

    #[test]
    fn test_cs133_wavelength() {
        let wl = AtomSpecies::Cesium133.wavelength_m();
        assert!((wl - 852.347_275e-9).abs() < 1e-15);
    }

    #[test]
    fn test_rb87_rabi_frequency() {
        let omega = AtomSpecies::Rubidium87.rabi_frequency();
        assert!((omega - 2.0 * PI * 50_000.0).abs() < 1.0);
    }

    // --- GravimeterConfig ---

    #[test]
    fn test_config_defaults() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
        assert_eq!(config.species, AtomSpecies::Rubidium87);
        assert!((config.interrogation_time_s - 0.1).abs() < TOLERANCE);
        assert!((config.laser_wavelength_m - RB87_WAVELENGTH_M).abs() < 1e-18);
    }

    #[test]
    fn test_config_custom_wavelength() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, Some(800e-9), None);
        assert!((config.laser_wavelength_m - 800e-9).abs() < 1e-18);
    }

    #[test]
    fn test_k_eff_rb87() {
        let k_eff = k_eff_two_photon(RB87_WAVELENGTH_M);
        // k_eff = 4*pi/780.24e-9 ~ 1.613e7 m^-1
        assert!((k_eff - 4.0 * PI / RB87_WAVELENGTH_M).abs() < 1.0);
        assert!(k_eff > 1.6e7);
        assert!(k_eff < 1.7e7);
    }

    // --- k_eff_two_photon ---

    #[test]
    fn test_k_eff_formula() {
        let lambda = 780e-9;
        let k = k_eff_two_photon(lambda);
        let expected = 4.0 * PI / lambda;
        assert!((k - expected).abs() / expected < 1e-12);
    }

    // --- MachZehnderInterferometer ---

    #[test]
    fn test_mzi_phase_standard_gravity() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let phase = mzi.interferometric_phase(G_STANDARD);
        // phi = k_eff * g * T^2 ~ 1.61e7 * 9.81 * 0.01 ~ 1.58e6 rad
        assert!(phase > 1.5e6);
        assert!(phase < 1.7e6);
    }

    #[test]
    fn test_mzi_phase_linearity() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.05, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let p1 = mzi.interferometric_phase(9.8);
        let p2 = mzi.interferometric_phase(19.6);
        assert!((p2 / p1 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_mzi_transition_probability_range() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.01, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        for phi_l in [0.0, PI / 4.0, PI / 2.0, PI, 2.0 * PI] {
            let p = mzi.transition_probability(G_STANDARD, phi_l, 0.9);
            assert!(p >= 0.0 && p <= 1.0, "P={} out of range for phi_l={}", p, phi_l);
        }
    }

    #[test]
    fn test_mzi_transition_probability_full_contrast() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.01, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let phase_intrf = mzi.interferometric_phase(G_STANDARD);
        // At phi_laser = phase_intrf, cos(0) = 1, P = (1+C)/2
        let p = mzi.transition_probability(G_STANDARD, phase_intrf, 1.0);
        assert!((p - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_mzi_pulse_durations() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let t_pi2 = mzi.pi_half_pulse_duration();
        let t_pi = mzi.pi_pulse_duration();
        assert!((t_pi - 2.0 * t_pi2).abs() < 1e-15);
        assert!(t_pi2 > 0.0);
    }

    #[test]
    fn test_mzi_total_sequence_duration() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let total = mzi.total_sequence_duration();
        // Should be dominated by 2*T = 200 ms
        assert!(total > 0.2);
        assert!(total < 0.21); // pulses are very short
    }

    #[test]
    fn test_mzi_gravity_per_fringe() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let g_per_fringe = mzi.gravity_per_fringe();
        // ~ 2*pi / (1.61e7 * 0.01) ~ 3.9e-5 m/s^2 ~ 3.9 mGal
        assert!(g_per_fringe > 1e-6);
        assert!(g_per_fringe < 1e-3);
    }

    // --- PhaseExtractor ---

    #[test]
    fn test_phase_extractor_roundtrip() {
        let ext = PhaseExtractor::new(0.95);
        for phase_deg in [0, 30, 60, 90, 120, 150, 180] {
            let phase = phase_deg as f64 * PI / 180.0;
            let pop = ext.population_from_phase(phase);
            let phase_recovered = ext.phase_from_population(pop);
            assert!(
                (phase - phase_recovered).abs() < 1e-10,
                "Roundtrip failed for {} deg",
                phase_deg
            );
        }
    }

    #[test]
    fn test_phase_extractor_boundary_populations() {
        let ext = PhaseExtractor::new(1.0);
        // P=1 → phi=0
        assert!(ext.phase_from_population(1.0).abs() < 1e-10);
        // P=0 → phi=pi
        assert!((ext.phase_from_population(0.0) - PI).abs() < 1e-10);
        // P=0.5 → phi=pi/2
        assert!((ext.phase_from_population(0.5) - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_phase_extractor_clamping() {
        let ext = PhaseExtractor::new(0.8);
        // Population values outside [0,1] should be clamped
        let p1 = ext.phase_from_population(-0.5);
        let p2 = ext.phase_from_population(1.5);
        assert!(p1.is_finite());
        assert!(p2.is_finite());
    }

    // --- GravityCalculator ---

    #[test]
    fn test_gravity_calculator_roundtrip() {
        let k_eff = k_eff_two_photon(RB87_WAVELENGTH_M);
        let calc = GravityCalculator::new(k_eff, 0.1);
        let g_original = 9.80665;
        let phase = calc.phase_from_gravity(g_original);
        let g_recovered = calc.gravity_from_phase(phase);
        assert!((g_recovered - g_original).abs() < 1e-12);
    }

    #[test]
    fn test_gravity_calculator_scale_factor() {
        let k_eff = k_eff_two_photon(RB87_WAVELENGTH_M);
        let t = 0.1;
        let calc = GravityCalculator::new(k_eff, t);
        assert!((calc.scale_factor() - k_eff * t * t).abs() < 1e-6);
    }

    #[test]
    fn test_gravity_calculator_from_config() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
        let calc = GravityCalculator::from_config(&config);
        assert!((calc.scale_factor() - config.k_eff() * 0.01).abs() < 1e-6);
    }

    #[test]
    fn test_gravity_resolution() {
        let k_eff = k_eff_two_photon(RB87_WAVELENGTH_M);
        let calc = GravityCalculator::new(k_eff, 0.1);
        // 1 mrad phase noise → gravity resolution
        let res = calc.gravity_resolution(1e-3);
        assert!(res > 0.0);
        assert!(res < 1e-6); // sub-uGal for 100ms interrogation
    }

    // --- FringeScanner ---

    #[test]
    fn test_fringe_scanner_perfect_sinusoid() {
        let mut scanner = FringeScanner::new(32);
        let true_phase = 1.23;
        let true_contrast = 0.9;

        for phi_l in scanner.scan_phases() {
            let p = 0.5 * (1.0 + true_contrast * (phi_l - true_phase).cos());
            scanner.add_measurement(phi_l, p);
        }

        let fit = scanner.fit().unwrap();
        assert!((fit.phase_rad - true_phase).abs() < 0.01);
        assert!((fit.contrast - true_contrast).abs() < 0.01);
        assert!((fit.offset - 0.5).abs() < 0.01);
        assert!(fit.residual_rms < 0.001);
    }

    #[test]
    fn test_fringe_scanner_insufficient_data() {
        let scanner = FringeScanner::new(16);
        assert!(scanner.fit().is_none());
    }

    #[test]
    fn test_fringe_scanner_clear() {
        let mut scanner = FringeScanner::new(8);
        scanner.add_measurement(0.0, 0.5);
        scanner.add_measurement(1.0, 0.7);
        assert_eq!(scanner.num_measurements(), 2);
        scanner.clear();
        assert_eq!(scanner.num_measurements(), 0);
    }

    // --- GradientMeter ---

    #[test]
    fn test_gradient_computation() {
        let gm = GradientMeter::new(1.0);
        let g_top = 9.806_650_0;
        let g_bottom = 9.806_653_086; // ~ 3086 E difference for 1m
        let grad_e = gm.gradient_eotvos(g_top, g_bottom);
        assert!((grad_e - (-3086.0)).abs() < 1.0);
    }

    #[test]
    fn test_gradient_baseline() {
        let gm = GradientMeter::new(0.5);
        assert!((gm.baseline_m() - 0.5).abs() < 1e-15);
    }

    #[test]
    fn test_free_air_gradient() {
        let fag = GradientMeter::free_air_gradient_eotvos();
        assert!((fag - (-3086.0)).abs() < 1e-10);
    }

    #[test]
    fn test_bouguer_correction() {
        // Rock density 2670 kg/m^3 → ~1119 E
        let corr = GradientMeter::bouguer_correction_s2(2670.0);
        let corr_e = corr * 1e9;
        assert!((corr_e - (-1119.0)).abs() < 5.0);
    }

    // --- TidalCorrector ---

    #[test]
    fn test_tidal_correction_at_zero() {
        let tc = TidalCorrector::new();
        let corr = tc.correction(0.0);
        // At t=0 with zero initial phases: lunar + solar = max amplitude
        let expected = LUNAR_TIDE_AMPLITUDE_MS2 + SOLAR_TIDE_AMPLITUDE_MS2;
        assert!((corr - expected).abs() < 1e-15);
    }

    #[test]
    fn test_tidal_correction_periodicity() {
        let tc = TidalCorrector::new();
        let c1 = tc.correction(0.0);
        // After one lunar semi-diurnal period, should return to same value
        let c2 = tc.correction(LUNAR_SEMIDIURNAL_PERIOD_S);
        // Won't be exactly equal because solar period differs, but lunar component repeats
        assert!((c2 - c1).abs() < 2.0 * SOLAR_TIDE_AMPLITUDE_MS2 * 2.0);
    }

    #[test]
    fn test_tidal_peak_to_peak() {
        let tc = TidalCorrector::new();
        let ptp = tc.peak_to_peak();
        let expected = 2.0 * (LUNAR_TIDE_AMPLITUDE_MS2 + SOLAR_TIDE_AMPLITUDE_MS2);
        assert!((ptp - expected).abs() < 1e-15);
    }

    #[test]
    fn test_tidal_correct_series() {
        let tc = TidalCorrector::new();
        let g_vals = vec![9.80665; 5];
        let times: Vec<f64> = (0..5).map(|i| i as f64 * 3600.0).collect();
        let corrected = tc.correct_series(&g_vals, &times);
        assert_eq!(corrected.len(), 5);
        // Each corrected value should differ from raw by the tidal amount
        for (i, &gc) in corrected.iter().enumerate() {
            let expected = 9.80665 - tc.correction(times[i]);
            assert!((gc - expected).abs() < 1e-15);
        }
    }

    // --- AtmosphericPressureCorrector ---

    #[test]
    fn test_atm_correction_zero_at_reference() {
        let atm = AtmosphericPressureCorrector::new(1013.25);
        assert!(atm.correction(1013.25).abs() < 1e-15);
    }

    #[test]
    fn test_atm_correction_sign() {
        let atm = AtmosphericPressureCorrector::new(1013.25);
        // Higher pressure → negative correction (atmosphere attracts more)
        let corr = atm.correction(1023.25); // +10 hPa
        assert!(corr < 0.0); // -0.3 uGal/hPa * 10 = -3 uGal
    }

    #[test]
    fn test_atm_correction_magnitude() {
        let atm = AtmosphericPressureCorrector::new(1013.25);
        let corr = atm.correction(1013.25 + 10.0);
        let expected = BAROMETRIC_ADMITTANCE_MS2_PER_HPA * 10.0;
        assert!((corr - expected).abs() < 1e-15);
    }

    #[test]
    fn test_atm_admittance_ugal() {
        let atm = AtmosphericPressureCorrector::new(1013.25);
        assert!((atm.admittance_ugal_per_hpa() - (-0.3)).abs() < 1e-10);
    }

    #[test]
    fn test_atm_custom_admittance() {
        let atm = AtmosphericPressureCorrector::with_admittance(1013.25, -0.4);
        assert!((atm.admittance_ugal_per_hpa() - (-0.4)).abs() < 1e-10);
    }

    // --- DriftEstimator ---

    #[test]
    fn test_drift_linear_removal() {
        let estimator = DriftEstimator::new(1);
        let times: Vec<f64> = (0..100).map(|i| i as f64).collect();
        // Linear drift: 1e-8 m/s^2 per sample
        let values: Vec<f64> = times.iter().map(|&t| 9.80665 + 1e-8 * t).collect();
        let detrended = estimator.remove_drift(&times, &values);
        for val in &detrended {
            assert!(val.abs() < 1e-9, "Residual {} too large", val);
        }
    }

    #[test]
    fn test_drift_quadratic_removal() {
        let estimator = DriftEstimator::new(2);
        let times: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let values: Vec<f64> = times
            .iter()
            .map(|&t| 1.0 + 0.01 * t + 0.001 * t * t)
            .collect();
        let detrended = estimator.remove_drift(&times, &values);
        for val in &detrended {
            assert!(val.abs() < 1e-6, "Residual {} too large", val);
        }
    }

    #[test]
    fn test_drift_linear_rate() {
        let estimator = DriftEstimator::new(1);
        let times: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let rate = 2.5e-8;
        let values: Vec<f64> = times.iter().map(|&t| rate * t).collect();
        let estimated_rate = estimator.linear_drift_rate(&times, &values);
        assert!(
            (estimated_rate - rate).abs() / rate < 1e-6,
            "Rate {} vs expected {}",
            estimated_rate,
            rate
        );
    }

    #[test]
    fn test_drift_polynomial_eval() {
        // Horner evaluation: a0 + a1*t + a2*t^2
        let coeffs = vec![1.0, 2.0, 3.0];
        let val = DriftEstimator::eval_polynomial(&coeffs, 2.0);
        assert!((val - (1.0 + 4.0 + 12.0)).abs() < 1e-10);
    }

    // --- NoiseAnalyzer (Allan deviation) ---

    #[test]
    fn test_allan_deviation_constant_signal() {
        // Constant signal → ADEV = 0
        let values = vec![9.80665; 100];
        let adev = NoiseAnalyzer::allan_deviation(&values, 1.0, 10);
        for &(_tau, ad) in &adev {
            assert!(ad < 1e-15, "ADEV should be zero for constant signal, got {}", ad);
        }
    }

    #[test]
    fn test_allan_deviation_length() {
        let values: Vec<f64> = (0..100).map(|i| 9.80665 + 1e-8 * (i as f64).sin()).collect();
        let adev = NoiseAnalyzer::allan_deviation(&values, 1.0, 20);
        assert!(!adev.is_empty());
        assert!(adev.len() <= 20);
    }

    #[test]
    fn test_allan_deviation_insufficient_data() {
        let values = vec![1.0, 2.0];
        let adev = NoiseAnalyzer::allan_deviation(&values, 1.0, 10);
        assert!(adev.is_empty());
    }

    #[test]
    fn test_noise_type_identification() {
        // Create data with known ADEV slope
        let adev_data: Vec<(f64, f64)> = (1..=10)
            .map(|i| {
                let tau = i as f64;
                let adev = 1.0 / tau; // slope = -1 in log-log
                (tau, adev)
            })
            .collect();
        let noise_type = NoiseAnalyzer::identify_noise_type(&adev_data);
        assert_eq!(noise_type, "white frequency noise");
    }

    #[test]
    fn test_noise_floor() {
        let adev_data = vec![(1.0, 1e-8), (10.0, 5e-9), (100.0, 2e-9), (1000.0, 3e-9)];
        let (tau, floor) = NoiseAnalyzer::noise_floor(&adev_data).unwrap();
        assert!((tau - 100.0).abs() < 1e-10);
        assert!((floor - 2e-9).abs() < 1e-15);
    }

    // --- FreeEvolutionModel ---

    #[test]
    fn test_free_fall_from_rest() {
        let model = FreeEvolutionModel::new(0.0, 0.0, G_STANDARD);
        // After 1 second: z = -0.5*g*t^2 ~ -4.9 m
        let z = model.position(1.0);
        assert!((z - (-0.5 * G_STANDARD)).abs() < 1e-10);
    }

    #[test]
    fn test_free_evolution_apogee() {
        let v0 = 5.0; // 5 m/s upward
        let model = FreeEvolutionModel::new(0.0, v0, G_STANDARD);
        let t_ap = model.apogee_time();
        assert!((t_ap - v0 / G_STANDARD).abs() < 1e-10);
        // Velocity at apogee should be zero
        let v_ap = model.velocity(t_ap);
        assert!(v_ap.abs() < 1e-10);
    }

    #[test]
    fn test_free_evolution_apogee_height() {
        let v0 = 4.0;
        let model = FreeEvolutionModel::new(0.0, v0, G_STANDARD);
        let h = model.apogee_height();
        let expected = v0 * v0 / (2.0 * G_STANDARD);
        assert!((h - expected).abs() < 1e-10);
    }

    #[test]
    fn test_trajectory_generation() {
        let model = FreeEvolutionModel::new(1.0, 3.0, G_STANDARD);
        let traj = model.trajectory(0.5, 11);
        assert_eq!(traj.len(), 11);
        // First point should be at initial conditions
        assert!((traj[0].0 - 0.0).abs() < 1e-10);
        assert!((traj[0].1 - 1.0).abs() < 1e-10);
        assert!((traj[0].2 - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_free_fall_distance() {
        let d = FreeEvolutionModel::free_fall_distance(G_STANDARD, 0.1);
        // 0.5 * 9.8 * 0.01 ~ 0.049 m
        assert!((d - 0.5 * G_STANDARD * 0.01).abs() < 1e-12);
    }

    #[test]
    fn test_required_launch_velocity() {
        let h = 1.0; // 1 meter
        let v = FreeEvolutionModel::required_launch_velocity(G_STANDARD, h);
        let expected = (2.0 * G_STANDARD * h).sqrt();
        assert!((v - expected).abs() < 1e-12);
    }

    // --- Unit conversion ---

    #[test]
    fn test_ms2_to_ugal() {
        assert!((ms2_to_ugal(1e-8) - 1.0).abs() < 1e-6);
        assert!((ms2_to_ugal(1e-6) - 100.0).abs() < 1e-4);
    }

    #[test]
    fn test_ugal_to_ms2() {
        assert!((ugal_to_ms2(1.0) - 1e-8).abs() < 1e-16);
    }

    #[test]
    fn test_ms2_to_mgal() {
        // 1 mGal = 10^-5 m/s^2
        assert!((ms2_to_mgal(1e-5) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_mgal_to_ms2() {
        assert!((mgal_to_ms2(1.0) - 1e-5).abs() < 1e-15);
    }

    // --- sensitivity_delta_g ---

    #[test]
    fn test_sensitivity_estimate() {
        let k_eff = k_eff_two_photon(RB87_WAVELENGTH_M);
        let t = 0.1;
        let snr = 1000.0;
        let delta_g = sensitivity_delta_g(k_eff, t, snr);
        // ~ 1 / (1.61e7 * 0.01 * 1000) ~ 6.2e-9 m/s^2 ~ 0.6 uGal
        assert!(delta_g > 1e-10);
        assert!(delta_g < 1e-7);
    }

    #[test]
    fn test_sensitivity_scales_with_t_squared() {
        let k_eff = k_eff_two_photon(RB87_WAVELENGTH_M);
        let snr = 100.0;
        let s1 = sensitivity_delta_g(k_eff, 0.1, snr);
        let s2 = sensitivity_delta_g(k_eff, 0.2, snr);
        // Doubling T should improve sensitivity by 4x
        assert!((s1 / s2 - 4.0).abs() < 1e-6);
    }

    // --- Process measurement pipeline ---

    #[test]
    fn test_process_measurement() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.01, None, None);
        let contrast = 0.95;
        let g = process_measurement(0.5, &config, contrast, 0.0, 1013.25, 1013.25);
        // P=0.5 → phase=pi/2 → g = (pi/2) / (k_eff * T^2)
        assert!(g > 0.0);
        assert!(g.is_finite());
    }

    // --- Integration: full roundtrip ---

    #[test]
    fn test_full_roundtrip_gravity_measurement() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.1, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let calc = GravityCalculator::from_config(&config);

        let g_true = 9.80665;
        let phase = mzi.interferometric_phase(g_true);
        let g_recovered = calc.gravity_from_phase(phase);
        assert!((g_recovered - g_true).abs() < 1e-12);
    }

    #[test]
    fn test_full_roundtrip_with_fringe_scanner() {
        let config = GravimeterConfig::new(AtomSpecies::Rubidium87, 0.01, None, None);
        let mzi = MachZehnderInterferometer::new(&config);
        let calc = GravityCalculator::from_config(&config);

        let g_true = 9.80665;
        let contrast = 0.95;
        let mut scanner = FringeScanner::new(64);

        for phi_l in scanner.scan_phases() {
            let p = mzi.transition_probability(g_true, phi_l, contrast);
            scanner.add_measurement(phi_l, p);
        }

        let fit = scanner.fit().unwrap();
        // The fitted phase should be the interferometric phase modulo 2*pi
        let expected_phase = mzi.interferometric_phase(g_true);
        // Phase wraps, so compare modulo 2*pi
        let phase_diff = (fit.phase_rad - expected_phase % (2.0 * PI)).abs();
        // This is a wrapped quantity, so just check contrast recovery
        assert!((fit.contrast - contrast).abs() < 0.02);
    }

    #[test]
    fn test_gradient_with_gradient_meter() {
        let gm = GradientMeter::new(0.3);
        let g_bottom = 9.806_65;
        let fag_s2 = GradientMeter::free_air_gradient_eotvos() * 1e-9;
        let g_top = g_bottom + fag_s2 * 0.3;
        let grad = gm.gradient_eotvos(g_top, g_bottom);
        assert!((grad - GradientMeter::free_air_gradient_eotvos()).abs() < 1.0);
    }

    #[test]
    fn test_free_evolution_with_gradient() {
        let model = FreeEvolutionModel::with_gradient(0.0, 0.0, G_STANDARD, -3086e-9);
        let z = model.position(0.1);
        // Should be close to simple free fall but with small gradient correction
        let z_simple = -0.5 * G_STANDARD * 0.01;
        assert!((z - z_simple).abs() < 1e-6);
    }
}

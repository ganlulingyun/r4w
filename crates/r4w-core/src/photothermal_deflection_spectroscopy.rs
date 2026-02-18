//! Photothermal Deflection Spectroscopy (PDS) Signal Processing
//!
//! Implements signal processing algorithms for Photothermal Deflection Spectroscopy,
//! a highly sensitive optical technique for measuring absorption in materials.
//!
//! # Principle
//!
//! A modulated pump beam heats the sample via optical absorption. The resulting
//! temperature gradient in the surrounding medium deflects a probe beam (mirage effect).
//! The deflection angle is:
//!
//! ```text
//! θ(ω) = (dn/dT) * (P_abs * L) / (κ * w) * F(ω)
//! ```
//!
//! where:
//! - `dn/dT` = thermooptic coefficient of the surrounding medium [K⁻¹]
//! - `P_abs`  = absorbed power [W]
//! - `L`      = probe-sample interaction length [m]
//! - `κ`      = thermal conductivity [W/(m·K)]
//! - `w`      = probe beam waist [m]
//! - `F(ω)`   = frequency response function containing thermal diffusion length
//!
//! # Thermal Diffusion Length
//!
//! ```text
//! μ(ω) = √(2α/ω)
//! ```
//!
//! where α is the thermal diffusivity [m²/s] and ω = 2πf is the angular modulation frequency.
//!
//! # References
//! - Boccara et al., Appl. Phys. Lett. 36, 130 (1980)
//! - Jackson et al., Appl. Phys. Lett. 32, 438 (1978)

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Stefan-Boltzmann constant [W/(m²·K⁴)] - for radiative loss corrections
const STEFAN_BOLTZMANN: f64 = 5.670374e-8;

// ---------------------------------------------------------------------------
// Material presets
// ---------------------------------------------------------------------------

/// Material with thermal and optical properties for PDS modelling
#[derive(Debug, Clone)]
pub struct Material {
    /// Name of the material
    pub name: &'static str,
    /// Thermal diffusivity α [m²/s]
    pub thermal_diffusivity: f64,
    /// Thermal conductivity κ [W/(m·K)]
    pub thermal_conductivity: f64,
    /// Thermooptic coefficient dn/dT [K⁻¹] of coupling medium (usually CCl₄ or air)
    pub dn_dt_medium: f64,
    /// Optical absorption coefficient α_abs [cm⁻¹] at reference wavelength
    pub absorption_coeff: f64,
    /// Urbach energy E_U [eV] for sub-bandgap exponential tail
    pub urbach_energy: f64,
    /// Bandgap energy E_g [eV]
    pub bandgap: f64,
    /// Refractive index (real part)
    pub refractive_index: f64,
}

impl Material {
    /// Amorphous silicon (a-Si:H) - common thin-film solar cell material
    pub fn amorphous_silicon() -> Self {
        Self {
            name: "a-Si:H",
            thermal_diffusivity: 1.0e-7,       // m²/s
            thermal_conductivity: 1.0,           // W/(m·K)
            dn_dt_medium: -4.5e-4,              // CCl₄ dn/dT
            absorption_coeff: 1e4,              // cm⁻¹ at 600 nm
            urbach_energy: 0.05,                // eV
            bandgap: 1.75,                      // eV
            refractive_index: 4.0,
        }
    }

    /// Crystalline silicon
    pub fn crystalline_silicon() -> Self {
        Self {
            name: "c-Si",
            thermal_diffusivity: 8.8e-5,        // m²/s
            thermal_conductivity: 148.0,          // W/(m·K)
            dn_dt_medium: -4.5e-4,              // CCl₄ dn/dT
            absorption_coeff: 1e2,              // cm⁻¹ at 800 nm
            urbach_energy: 0.01,                // eV (very sharp band edge)
            bandgap: 1.12,                      // eV
            refractive_index: 3.5,
        }
    }

    /// Gallium arsenide (GaAs)
    pub fn gaas() -> Self {
        Self {
            name: "GaAs",
            thermal_diffusivity: 2.5e-5,        // m²/s
            thermal_conductivity: 46.0,           // W/(m·K)
            dn_dt_medium: -4.5e-4,              // CCl₄ dn/dT
            absorption_coeff: 1e4,              // cm⁻¹ at 800 nm
            urbach_energy: 0.007,               // eV
            bandgap: 1.42,                      // eV
            refractive_index: 3.6,
        }
    }

    /// Perovskite solar cell (MAPbI₃)
    pub fn perovskite() -> Self {
        Self {
            name: "MAPbI3",
            thermal_diffusivity: 3.0e-7,        // m²/s
            thermal_conductivity: 0.5,            // W/(m·K)
            dn_dt_medium: -4.5e-4,              // CCl₄ dn/dT
            absorption_coeff: 1e4,              // cm⁻¹ at 550 nm
            urbach_energy: 0.015,               // eV
            bandgap: 1.6,                       // eV
            refractive_index: 2.5,
        }
    }

    /// Organic semiconductor (P3HT:PCBM blend)
    pub fn organic_semiconductor() -> Self {
        Self {
            name: "P3HT:PCBM",
            thermal_diffusivity: 5.0e-8,        // m²/s
            thermal_conductivity: 0.2,            // W/(m·K)
            dn_dt_medium: -4.5e-4,              // CCl₄ dn/dT
            absorption_coeff: 5e3,              // cm⁻¹ at 500 nm
            urbach_energy: 0.1,                 // eV (broad tail)
            bandgap: 1.9,                       // eV
            refractive_index: 1.8,
        }
    }
}

// ---------------------------------------------------------------------------
// Thermal diffusion model
// ---------------------------------------------------------------------------

/// Compute thermal diffusion length μ [m] at angular frequency ω [rad/s]
///
/// μ = √(2α/ω)
///
/// # Arguments
/// * `thermal_diffusivity` - α [m²/s]
/// * `omega` - angular modulation frequency [rad/s]
///
/// # Returns
/// Thermal diffusion length in metres. Returns f64::INFINITY for ω = 0.
pub fn thermal_diffusion_length(thermal_diffusivity: f64, omega: f64) -> f64 {
    if omega <= 0.0 {
        return f64::INFINITY;
    }
    (2.0 * thermal_diffusivity / omega).sqrt()
}

/// Complex thermal wave propagation factor at distance z from source
///
/// exp(-(1+i) * z / μ)  – attenuated exponential wave
///
/// Returns (real, imag) tuple.
pub fn thermal_wave_factor(z: f64, mu: f64) -> (f64, f64) {
    if mu.is_infinite() {
        return (1.0, 0.0);
    }
    let exponent = -z / mu;
    let magnitude = exponent.exp();
    // phase = -z/μ  (same as real part exponent for (1+i)/μ factor)
    (magnitude * exponent.cos(), magnitude * exponent.sin())
}

// ---------------------------------------------------------------------------
// Beam deflection
// ---------------------------------------------------------------------------

/// Configuration for a PDS measurement geometry
#[derive(Debug, Clone)]
pub struct PdsGeometry {
    /// Probe beam waist (1/e² radius) [m]
    pub probe_waist: f64,
    /// Probe beam height above sample surface [m] (transverse PDS)
    pub probe_height: f64,
    /// Probe-sample interaction length [m]
    pub interaction_length: f64,
    /// Thermooptic coefficient of surrounding medium dn/dT [K⁻¹]
    pub dn_dt: f64,
    /// Pump beam spot radius at sample [m]
    pub pump_spot_radius: f64,
    /// Modality: transverse or collinear
    pub mode: PdsMode,
}

/// PDS measurement modality
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PdsMode {
    /// Probe beam parallel to sample surface (mirage effect) – surface sensitive
    Transverse,
    /// Probe beam perpendicular to sample – transparent samples, bulk absorption
    Collinear,
}

impl Default for PdsGeometry {
    fn default() -> Self {
        Self {
            probe_waist: 100e-6,         // 100 µm
            probe_height: 200e-6,        // 200 µm above surface
            interaction_length: 5e-3,    // 5 mm
            dn_dt: -4.5e-4,             // CCl₄ at 633 nm [K⁻¹]
            pump_spot_radius: 500e-6,    // 500 µm
            mode: PdsMode::Transverse,
        }
    }
}

/// Compute probe beam deflection angle θ [rad] for transverse PDS
///
/// θ = (dn/dT) * (P_abs * L) / (κ_eff * w) * exp(-z/μ)
///
/// Returns (in-phase, quadrature) deflection angle components [rad].
///
/// # Arguments
/// * `absorbed_power`   - P_abs [W] = P_pump * (1 - exp(-α*d)) for film of thickness d
/// * `geometry`         - measurement geometry parameters
/// * `material`         - sample material properties
/// * `omega`            - pump modulation angular frequency [rad/s]
pub fn beam_deflection_transverse(
    absorbed_power: f64,
    geometry: &PdsGeometry,
    material: &Material,
    omega: f64,
) -> (f64, f64) {
    let mu = thermal_diffusion_length(material.thermal_diffusivity, omega);
    let (tw_re, tw_im) = thermal_wave_factor(geometry.probe_height, mu);

    // Temperature gradient amplitude at probe position (simplified 1D model)
    let delta_t_amplitude = absorbed_power
        / (material.thermal_conductivity * geometry.pump_spot_radius * 2.0 * PI);

    // Deflection angle magnitude
    let scale = geometry.dn_dt.abs() * delta_t_amplitude * geometry.interaction_length
        / geometry.probe_waist;

    (scale * tw_re, scale * tw_im)
}

/// Compute probe beam deflection angle θ [rad] for collinear PDS
///
/// For collinear geometry the probe traverses the sample perpendicularly.
/// Deflection is caused by the transverse refractive index gradient.
///
/// Returns (in-phase, quadrature) deflection angle components [rad].
pub fn beam_deflection_collinear(
    absorbed_power: f64,
    geometry: &PdsGeometry,
    material: &Material,
    omega: f64,
    sample_thickness: f64,
) -> (f64, f64) {
    let mu = thermal_diffusion_length(material.thermal_diffusivity, omega);

    // Effective temperature rise in sample bulk
    let delta_t = absorbed_power
        / (material.thermal_conductivity
            * PI
            * geometry.pump_spot_radius
            * geometry.pump_spot_radius);

    // Thermal wave attenuation over sample thickness
    let (tw_re, tw_im) = thermal_wave_factor(sample_thickness / 2.0, mu);

    let scale = geometry.dn_dt.abs() * delta_t * sample_thickness;

    (scale * tw_re, scale * tw_im)
}

/// Compute deflection signal amplitude and phase from (I, Q) components
pub fn deflection_amplitude_phase(in_phase: f64, quadrature: f64) -> (f64, f64) {
    let amplitude = (in_phase * in_phase + quadrature * quadrature).sqrt();
    let phase = quadrature.atan2(in_phase);
    (amplitude, phase)
}

// ---------------------------------------------------------------------------
// Lock-in amplifier detection
// ---------------------------------------------------------------------------

/// Lock-in amplifier parameters
#[derive(Debug, Clone)]
pub struct LockInAmplifier {
    /// Reference frequency [Hz]
    pub reference_freq: f64,
    /// Low-pass filter time constant [s]
    pub time_constant: f64,
    /// Phase offset applied to reference [rad]
    pub phase_offset: f64,
    /// Integration/averaging samples
    pub integration_samples: usize,
}

impl LockInAmplifier {
    /// Create a new lock-in amplifier
    pub fn new(reference_freq: f64, time_constant: f64) -> Self {
        Self {
            reference_freq,
            time_constant,
            phase_offset: 0.0,
            integration_samples: 1024,
        }
    }

    /// Demodulate a signal at the reference frequency
    ///
    /// Returns (X component, Y component) where:
    /// - X = in-phase (real) component
    /// - Y = quadrature (imaginary) component
    ///
    /// # Arguments
    /// * `signal`      - time-domain input signal
    /// * `sample_rate` - sampling rate [Hz]
    pub fn demodulate(&self, signal: &[f64], sample_rate: f64) -> (f64, f64) {
        if signal.is_empty() {
            return (0.0, 0.0);
        }

        let omega_ref = 2.0 * PI * self.reference_freq;
        let dt = 1.0 / sample_rate;

        // IQ demodulation: multiply by reference sinusoids then lowpass filter
        let mut sum_x = 0.0f64;
        let mut sum_y = 0.0f64;

        for (n, &s) in signal.iter().enumerate() {
            let t = n as f64 * dt;
            let phase = omega_ref * t + self.phase_offset;
            sum_x += s * phase.cos();
            sum_y += s * (-phase.sin());
        }

        let n = signal.len() as f64;
        // Apply RC-equivalent averaging weight
        let norm = 2.0 / n;
        (sum_x * norm, sum_y * norm)
    }

    /// Demodulate complex (I+jQ) signal and return amplitude + phase
    pub fn demodulate_amplitude_phase(&self, signal: &[f64], sample_rate: f64) -> (f64, f64) {
        let (x, y) = self.demodulate(signal, sample_rate);
        deflection_amplitude_phase(x, y)
    }

    /// Apply IIR low-pass filter (single-pole RC) to a sequence of demodulated values
    ///
    /// α = dt / (τ + dt)
    pub fn lowpass_filter(&self, values: &[f64], sample_rate: f64) -> Vec<f64> {
        let dt = 1.0 / sample_rate;
        let alpha = dt / (self.time_constant + dt);
        let mut output = Vec::with_capacity(values.len());
        let mut state = 0.0f64;
        for &v in values {
            state = state + alpha * (v - state);
            output.push(state);
        }
        output
    }
}

// ---------------------------------------------------------------------------
// Frequency response
// ---------------------------------------------------------------------------

/// Compute PDS amplitude frequency response at a set of modulation frequencies
///
/// The thermal roll-off follows f^(-1/2) above the thermal cut-off frequency.
///
/// Returns (frequencies_hz, normalised_amplitudes) where amplitude[0] = 1.0.
///
/// # Arguments
/// * `frequencies_hz`   - modulation frequencies to evaluate [Hz]
/// * `thermal_diffusivity` - α [m²/s]
/// * `probe_height`     - probe beam height above sample [m]
pub fn frequency_response(
    frequencies_hz: &[f64],
    thermal_diffusivity: f64,
    probe_height: f64,
) -> Vec<f64> {
    let amplitudes: Vec<f64> = frequencies_hz
        .iter()
        .map(|&f| {
            if f <= 0.0 {
                return 1.0;
            }
            let omega = 2.0 * PI * f;
            let mu = thermal_diffusion_length(thermal_diffusivity, omega);
            let (re, im) = thermal_wave_factor(probe_height, mu);
            (re * re + im * im).sqrt()
        })
        .collect();

    // Normalise to first non-zero frequency
    let norm = amplitudes.first().copied().unwrap_or(1.0).max(1e-30);
    amplitudes.iter().map(|&a| a / norm).collect()
}

/// Compute theoretical f^(-1/2) slope reference for frequency response verification
pub fn half_power_slope(frequencies_hz: &[f64], f_ref: f64) -> Vec<f64> {
    frequencies_hz
        .iter()
        .map(|&f| {
            if f <= 0.0 || f_ref <= 0.0 {
                1.0
            } else {
                (f_ref / f).sqrt()
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Absorption coefficient extraction
// ---------------------------------------------------------------------------

/// PDS calibration dataset
#[derive(Debug, Clone)]
pub struct PdsCalibration {
    /// Reference sample absorption coefficient [cm⁻¹]
    pub reference_absorption: f64,
    /// Deflection signal from reference sample [V or rad]
    pub reference_signal: f64,
    /// Pump power during calibration [W]
    pub pump_power: f64,
    /// Sample thickness [m]
    pub sample_thickness: f64,
}

/// Extract absorption coefficient from PDS deflection signal
///
/// α = (S / S_ref) * α_ref
///
/// where S is the normalised deflection signal.
///
/// # Arguments
/// * `deflection_signal` - measured deflection amplitude
/// * `pump_power`        - pump power [W]
/// * `calibration`       - calibration parameters
pub fn extract_absorption_coefficient(
    deflection_signal: f64,
    pump_power: f64,
    calibration: &PdsCalibration,
) -> f64 {
    if calibration.reference_signal <= 0.0
        || pump_power <= 0.0
        || calibration.pump_power <= 0.0
    {
        return 0.0;
    }
    // Normalise for pump power differences
    let signal_norm = deflection_signal / pump_power;
    let ref_norm = calibration.reference_signal / calibration.pump_power;
    (signal_norm / ref_norm) * calibration.reference_absorption
}

/// Compute absorption spectrum from wavelength-scanned PDS data
///
/// Normalises deflection vs wavelength by the lamp/filter spectral response.
///
/// # Arguments
/// * `deflection_vs_wavelength` - measured deflection at each wavelength
/// * `lamp_spectrum`            - lamp spectral power at each wavelength (same length)
/// * `calibration_factor`       - scalar to convert to cm⁻¹
///
/// # Returns
/// Absorption coefficient spectrum [cm⁻¹]
pub fn absorption_spectrum(
    deflection_vs_wavelength: &[f64],
    lamp_spectrum: &[f64],
    calibration_factor: f64,
) -> Vec<f64> {
    deflection_vs_wavelength
        .iter()
        .zip(lamp_spectrum.iter())
        .map(|(&d, &l)| {
            if l <= 0.0 {
                0.0
            } else {
                calibration_factor * d / l
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Sub-bandgap absorption and Urbach tail
// ---------------------------------------------------------------------------

/// Compute Urbach tail absorption coefficient
///
/// α(E) = α₀ * exp((E - E₀) / E_U)   for E < E_g
///
/// # Arguments
/// * `energy_ev`     - photon energy [eV]
/// * `alpha0`        - pre-exponential [cm⁻¹]
/// * `e0`            - reference energy [eV] (often E_g)
/// * `urbach_energy` - Urbach energy E_U [eV]
pub fn urbach_tail_absorption(energy_ev: f64, alpha0: f64, e0: f64, urbach_energy: f64) -> f64 {
    if urbach_energy <= 0.0 {
        return 0.0;
    }
    alpha0 * ((energy_ev - e0) / urbach_energy).exp()
}

/// Fit Urbach energy E_U from a set of (energy, alpha) data points
///
/// Uses linear regression on ln(α) vs E to extract slope = 1/E_U.
///
/// Returns (E_U [eV], alpha0 [cm⁻¹], e0 reference energy [eV])
pub fn fit_urbach_energy(energies_ev: &[f64], absorption_cm: &[f64]) -> (f64, f64, f64) {
    let n = energies_ev.len().min(absorption_cm.len());
    if n < 2 {
        return (0.0, 0.0, 0.0);
    }

    // Filter out non-positive absorption values
    let pairs: Vec<(f64, f64)> = energies_ev[..n]
        .iter()
        .zip(absorption_cm[..n].iter())
        .filter(|(_, &a)| a > 0.0)
        .map(|(&e, &a)| (e, a.ln()))
        .collect();

    if pairs.len() < 2 {
        return (0.0, 0.0, 0.0);
    }

    let m = pairs.len() as f64;
    let sum_e: f64 = pairs.iter().map(|(e, _)| e).sum();
    let sum_ln_a: f64 = pairs.iter().map(|(_, la)| la).sum();
    let sum_e2: f64 = pairs.iter().map(|(e, _)| e * e).sum();
    let sum_e_ln_a: f64 = pairs.iter().map(|(e, la)| e * la).sum();

    let denom = m * sum_e2 - sum_e * sum_e;
    if denom.abs() < 1e-30 {
        return (0.0, 0.0, 0.0);
    }

    let slope = (m * sum_e_ln_a - sum_e * sum_ln_a) / denom; // = 1/E_U
    let intercept = (sum_ln_a - slope * sum_e) / m;

    let e_urbach = if slope.abs() > 1e-30 { 1.0 / slope } else { 0.0 };
    let alpha0 = intercept.exp();
    let e0 = -intercept / slope.max(1e-30); // energy where α = 1 cm⁻¹ reference

    (e_urbach.abs(), alpha0, e0)
}

/// Classify defect state from absorption energy below bandgap
///
/// Returns a string description of the likely defect type.
pub fn classify_defect_state(
    absorption_energy_ev: f64,
    bandgap_ev: f64,
    urbach_energy_ev: f64,
) -> DefectClass {
    let depth = bandgap_ev - absorption_energy_ev;
    if depth < 0.0 {
        return DefectClass::AboveBandgap;
    }
    if depth < 3.0 * urbach_energy_ev {
        DefectClass::UrbachTail
    } else if depth < 0.3 * bandgap_ev {
        DefectClass::ShallowDefect
    } else if depth < 0.6 * bandgap_ev {
        DefectClass::MidgapDefect
    } else {
        DefectClass::DeepDefect
    }
}

/// Classification of defect states from sub-bandgap PDS
#[derive(Debug, Clone, PartialEq)]
pub enum DefectClass {
    AboveBandgap,
    UrbachTail,
    ShallowDefect,
    MidgapDefect,
    DeepDefect,
}

// ---------------------------------------------------------------------------
// Thin film model
// ---------------------------------------------------------------------------

/// Thin film parameters for PDS modelling with multiple reflections
#[derive(Debug, Clone)]
pub struct ThinFilm {
    /// Film thickness [m]
    pub thickness: f64,
    /// Film refractive index (real part)
    pub n_film: f64,
    /// Substrate refractive index (real part)
    pub n_substrate: f64,
    /// Film absorption coefficient [cm⁻¹]
    pub absorption_coeff: f64,
}

impl ThinFilm {
    /// Fresnel reflectance at normal incidence from medium 1 to medium 2
    pub fn fresnel_reflectance(n1: f64, n2: f64) -> f64 {
        let r = (n1 - n2) / (n1 + n2);
        r * r
    }

    /// Compute fraction of pump power absorbed in thin film with multiple reflections
    ///
    /// Uses simple Fabry-Perot model neglecting interference (incoherent limit):
    /// P_abs / P_inc = (1 - R_01) * (1 - exp(-α*d)) / (1 - R_12 * exp(-2*α*d))
    ///
    /// # Arguments
    /// * `pump_power` - incident pump power [W]
    pub fn absorbed_power(&self, pump_power: f64) -> f64 {
        let r01 = Self::fresnel_reflectance(1.0, self.n_film); // air-film
        let r12 = Self::fresnel_reflectance(self.n_film, self.n_substrate); // film-substrate

        // Convert absorption coefficient cm⁻¹ → m⁻¹
        let alpha_m = self.absorption_coeff * 100.0;
        let exp_term = (-alpha_m * self.thickness).exp();
        let denom = 1.0 - r12 * exp_term * exp_term;

        if denom.abs() < 1e-10 {
            return pump_power * (1.0 - r01) * (1.0 - exp_term);
        }

        pump_power * (1.0 - r01) * (1.0 - exp_term) / denom
    }

    /// Round-trip phase in the film for coherent interference calculations [rad]
    pub fn round_trip_phase(&self, wavelength_m: f64) -> f64 {
        4.0 * PI * self.n_film * self.thickness / wavelength_m
    }

    /// Estimate absorption coefficient from PDS signal and geometry (thin film)
    ///
    /// # Arguments
    /// * `pds_signal`        - measured deflection signal
    /// * `ref_signal`        - reference signal (no sample or known reference)
    /// * `ref_absorption`    - reference absorption coefficient [cm⁻¹]
    pub fn estimate_absorption(&self, pds_signal: f64, ref_signal: f64, ref_absorption: f64) -> f64 {
        if ref_signal <= 0.0 {
            return 0.0;
        }
        pds_signal / ref_signal * ref_absorption
    }
}

// ---------------------------------------------------------------------------
// PDS Processor: full measurement pipeline
// ---------------------------------------------------------------------------

/// Complete PDS signal processor combining all components
#[derive(Debug)]
pub struct PdsProcessor {
    /// Measurement geometry
    pub geometry: PdsGeometry,
    /// Lock-in amplifier settings
    pub lock_in: LockInAmplifier,
    /// Background signal (dark/no pump) for subtraction [V]
    pub background_signal: f64,
}

impl PdsProcessor {
    /// Create a new PDS processor
    pub fn new(geometry: PdsGeometry, modulation_freq: f64) -> Self {
        let lock_in = LockInAmplifier::new(modulation_freq, 0.1);
        Self {
            geometry,
            lock_in,
            background_signal: 0.0,
        }
    }

    /// Process a raw photodetector signal and return deflection amplitude [rad]
    ///
    /// 1. Lock-in demodulation at pump modulation frequency
    /// 2. Background subtraction
    /// 3. Amplitude extraction
    pub fn process_signal(&self, raw_signal: &[f64], sample_rate: f64) -> (f64, f64) {
        let (x, y) = self.lock_in.demodulate(raw_signal, sample_rate);
        let x_corrected = x - self.background_signal;
        deflection_amplitude_phase(x_corrected, y)
    }

    /// Full spectral measurement: scan pump wavelength, return absorption spectrum
    ///
    /// # Arguments
    /// * `deflections`    - measured deflection at each wavelength (background subtracted)
    /// * `lamp_power`     - lamp power at each wavelength
    /// * `calibration`    - calibration parameters
    pub fn compute_absorption_spectrum(
        &self,
        deflections: &[f64],
        lamp_power: &[f64],
        calibration: &PdsCalibration,
    ) -> Vec<f64> {
        let cal_factor = calibration.reference_absorption / calibration.reference_signal;
        absorption_spectrum(deflections, lamp_power, cal_factor)
    }
}

// ---------------------------------------------------------------------------
// Sensitivity and noise floor analysis
// ---------------------------------------------------------------------------

/// Estimate minimum detectable absorption coefficient [cm⁻¹]
///
/// # Arguments
/// * `noise_floor`       - RMS noise on deflection signal [rad]
/// * `pump_power`        - pump power [W]
/// * `sample_length`     - interaction length [m]
/// * `sensitivity_coeff` - dθ/dα [rad/(cm⁻¹)] from calibration
pub fn minimum_detectable_absorption(
    noise_floor: f64,
    sensitivity_coeff: f64,
) -> f64 {
    if sensitivity_coeff <= 0.0 {
        return f64::INFINITY;
    }
    noise_floor / sensitivity_coeff
}

/// Compute signal-to-noise ratio for a PDS measurement
pub fn snr_pds(signal_amplitude: f64, noise_rms: f64) -> f64 {
    if noise_rms <= 0.0 {
        return f64::INFINITY;
    }
    signal_amplitude / noise_rms
}

/// Theoretical shot-noise-limited deflection noise [rad/√Hz]
///
/// For a detector current I_d [A] and optical power P [W]:
/// δθ_shot = √(2eI_d) / (dI/dθ)
///
/// Here we use the sensitivity dI/dθ = I_d / θ_0 to give a normalised estimate.
///
/// # Arguments
/// * `detector_current_a` - mean photocurrent [A]
/// * `sensitivity_rad`    - deflection for unit photocurrent change [rad/A]
pub fn shot_noise_deflection(detector_current_a: f64, sensitivity_rad: f64) -> f64 {
    const ELECTRON_CHARGE: f64 = 1.602e-19;
    if detector_current_a <= 0.0 || sensitivity_rad <= 0.0 {
        return 0.0;
    }
    let current_noise = (2.0 * ELECTRON_CHARGE * detector_current_a).sqrt();
    current_noise * sensitivity_rad
}

// ---------------------------------------------------------------------------
// Radiative correction (optional)
// ---------------------------------------------------------------------------

/// Radiative cooling power from heated sample surface [W/m²]
///
/// P_rad = ε * σ * (T⁴ - T_amb⁴)
///
/// Small correction for low pump powers but relevant for high-power calibration.
pub fn radiative_cooling_power(
    emissivity: f64,
    temperature_k: f64,
    ambient_temp_k: f64,
) -> f64 {
    emissivity * STEFAN_BOLTZMANN * (temperature_k.powi(4) - ambient_temp_k.powi(4))
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    // -----------------------------------------------------------------------
    // Thermal diffusion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_thermal_diffusion_length_basic() {
        // μ = √(2α/ω) = √(2*1e-6/62.83) ≈ 1.784e-4 m for α=1e-6 m²/s, f=10 Hz
        let alpha = 1.0e-6;
        let omega = 2.0 * PI * 10.0;
        let mu = thermal_diffusion_length(alpha, omega);
        let expected = (2.0 * alpha / omega).sqrt();
        assert!((mu - expected).abs() < TOL, "μ mismatch: {mu} vs {expected}");
    }

    #[test]
    fn test_thermal_diffusion_length_zero_freq() {
        let mu = thermal_diffusion_length(1e-6, 0.0);
        assert!(mu.is_infinite(), "μ at ω=0 should be infinity");
    }

    #[test]
    fn test_thermal_diffusion_length_high_freq() {
        // Higher frequency → shorter thermal diffusion length
        let alpha = 1e-6;
        let mu_low = thermal_diffusion_length(alpha, 2.0 * PI * 1.0);
        let mu_high = thermal_diffusion_length(alpha, 2.0 * PI * 100.0);
        assert!(mu_high < mu_low, "Higher freq should give shorter μ");
    }

    #[test]
    fn test_thermal_diffusion_scales_with_alpha() {
        let omega = 2.0 * PI * 10.0;
        let mu1 = thermal_diffusion_length(1e-6, omega);
        let mu4 = thermal_diffusion_length(4e-6, omega); // 4x diffusivity
        assert!(
            (mu4 / mu1 - 2.0).abs() < 1e-10,
            "μ should scale as √α: ratio={}", mu4 / mu1
        );
    }

    // -----------------------------------------------------------------------
    // Thermal wave factor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_thermal_wave_factor_zero_depth() {
        let (re, im) = thermal_wave_factor(0.0, 100e-6);
        assert!((re - 1.0).abs() < TOL);
        assert!(im.abs() < TOL);
    }

    #[test]
    fn test_thermal_wave_factor_infinite_mu() {
        let (re, im) = thermal_wave_factor(1e-3, f64::INFINITY);
        assert!((re - 1.0).abs() < TOL);
        assert!(im.abs() < TOL);
    }

    #[test]
    fn test_thermal_wave_factor_attenuation() {
        let mu = 200e-6;
        let (re1, im1) = thermal_wave_factor(mu, mu);
        let amplitude = (re1 * re1 + im1 * im1).sqrt();
        // At z = μ, amplitude = exp(-1) ≈ 0.3679
        assert!((amplitude - (-1.0f64).exp()).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Beam deflection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_beam_deflection_transverse_zero_power() {
        let geom = PdsGeometry::default();
        let mat = Material::amorphous_silicon();
        let (re, im) = beam_deflection_transverse(0.0, &geom, &mat, 2.0 * PI * 10.0);
        assert!(re.abs() < TOL);
        assert!(im.abs() < TOL);
    }

    #[test]
    fn test_beam_deflection_transverse_positive() {
        let geom = PdsGeometry::default();
        let mat = Material::amorphous_silicon();
        let (re, im) = beam_deflection_transverse(1e-3, &geom, &mat, 2.0 * PI * 10.0);
        let amplitude = (re * re + im * im).sqrt();
        assert!(amplitude > 0.0, "Deflection should be positive for non-zero power");
    }

    #[test]
    fn test_beam_deflection_collinear_scales_with_thickness() {
        let geom = PdsGeometry {
            mode: PdsMode::Collinear,
            ..PdsGeometry::default()
        };
        let mat = Material::crystalline_silicon();
        let (re1, im1) = beam_deflection_collinear(1e-3, &geom, &mat, 2.0 * PI * 5.0, 100e-9);
        let (re2, im2) = beam_deflection_collinear(1e-3, &geom, &mat, 2.0 * PI * 5.0, 200e-9);
        let amp1 = (re1 * re1 + im1 * im1).sqrt();
        let amp2 = (re2 * re2 + im2 * im2).sqrt();
        // Thicker sample → larger deflection (roughly linear)
        assert!(amp2 > amp1, "Thicker film should give larger deflection");
    }

    #[test]
    fn test_deflection_amplitude_phase() {
        let (amp, phase) = deflection_amplitude_phase(3.0, 4.0);
        assert!((amp - 5.0).abs() < TOL, "Amplitude should be 5");
        assert!((phase - (4.0f64).atan2(3.0)).abs() < TOL);
    }

    #[test]
    fn test_deflection_amplitude_zero_signal() {
        let (amp, _phase) = deflection_amplitude_phase(0.0, 0.0);
        assert!(amp.abs() < TOL);
    }

    // -----------------------------------------------------------------------
    // Lock-in amplifier tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lock_in_pure_sine_in_phase() {
        let freq = 100.0_f64;
        let sample_rate = 10_000.0;
        let n = 10_000usize;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).cos())
            .collect();

        let lia = LockInAmplifier::new(freq, 0.01);
        let (x, y) = lia.demodulate(&signal, sample_rate);
        // In-phase should be ~1.0, quadrature ~0.0
        assert!(x.abs() > 0.9, "X component should be near 1: {x}");
        assert!(y.abs() < 0.05, "Y component should be near 0: {y}");
    }

    #[test]
    fn test_lock_in_pure_sine_quadrature() {
        let freq = 100.0_f64;
        let sample_rate = 10_000.0;
        let n = 10_000usize;
        // 90° phase-shifted (sine)
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect();

        let lia = LockInAmplifier::new(freq, 0.01);
        let (x, y) = lia.demodulate(&signal, sample_rate);
        // Quadrature should dominate
        assert!(y.abs() > 0.9, "Y component should be near 1: {y}");
        assert!(x.abs() < 0.05, "X component should be near 0: {x}");
    }

    #[test]
    fn test_lock_in_amplitude_phase() {
        let freq = 50.0_f64;
        let sample_rate = 5_000.0;
        let n = 5_000usize;
        let signal: Vec<f64> = (0..n)
            .map(|i| 2.0 * (2.0 * PI * freq * i as f64 / sample_rate).cos())
            .collect();

        let lia = LockInAmplifier::new(freq, 0.01);
        let (amp, _phase) = lia.demodulate_amplitude_phase(&signal, sample_rate);
        // Amplitude of cosine with magnitude 2
        assert!((amp - 2.0).abs() < 0.1, "Amplitude should be ~2: {amp}");
    }

    #[test]
    fn test_lock_in_empty_signal() {
        let lia = LockInAmplifier::new(100.0, 0.01);
        let (x, y) = lia.demodulate(&[], 10_000.0);
        assert!(x.abs() < TOL);
        assert!(y.abs() < TOL);
    }

    #[test]
    fn test_lock_in_lowpass_filter() {
        let lia = LockInAmplifier::new(100.0, 0.5); // τ = 0.5 s
        let values = vec![0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let filtered = lia.lowpass_filter(&values, 10.0);
        // Should rise towards 1.0
        assert!(filtered.last().copied().unwrap_or(0.0) > 0.0);
        assert!(filtered.last().copied().unwrap_or(0.0) < 1.01);
        // Monotonically increasing
        for w in filtered.windows(2) {
            assert!(w[1] >= w[0] - 1e-10);
        }
    }

    // -----------------------------------------------------------------------
    // Frequency response tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_frequency_response_normalised_at_first() {
        let freqs = vec![1.0, 10.0, 100.0, 1000.0];
        let resp = frequency_response(&freqs, 1e-6, 100e-6);
        assert!((resp[0] - 1.0).abs() < TOL, "First value should be 1.0");
    }

    #[test]
    fn test_frequency_response_decreasing() {
        let freqs: Vec<f64> = (1..=10).map(|i| i as f64 * 10.0).collect();
        let resp = frequency_response(&freqs, 1e-6, 100e-6);
        for i in 1..resp.len() {
            assert!(resp[i] <= resp[i - 1] + 1e-10, "Response should decrease with frequency");
        }
    }

    #[test]
    fn test_half_power_slope_reference() {
        let freqs = vec![1.0, 4.0, 9.0, 16.0];
        let slope = half_power_slope(&freqs, 1.0);
        // slope[i] = √(f_ref / f[i]) = 1/√f[i]
        assert!((slope[0] - 1.0).abs() < TOL);
        assert!((slope[1] - 0.5).abs() < TOL); // √(1/4)
        assert!((slope[2] - 1.0 / 3.0).abs() < 1e-10); // √(1/9)
    }

    // -----------------------------------------------------------------------
    // Absorption coefficient extraction tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_extract_absorption_coefficient_basic() {
        let cal = PdsCalibration {
            reference_absorption: 100.0,  // cm⁻¹
            reference_signal: 1.0,
            pump_power: 1.0,
            sample_thickness: 100e-9,
        };
        // Same signal and power → same absorption as reference
        let alpha = extract_absorption_coefficient(1.0, 1.0, &cal);
        assert!((alpha - 100.0).abs() < 1e-10, "Should return reference absorption: {alpha}");
    }

    #[test]
    fn test_extract_absorption_coefficient_scaling() {
        let cal = PdsCalibration {
            reference_absorption: 100.0,
            reference_signal: 1.0,
            pump_power: 1.0,
            sample_thickness: 100e-9,
        };
        // Double the signal → double the absorption
        let alpha = extract_absorption_coefficient(2.0, 1.0, &cal);
        assert!((alpha - 200.0).abs() < 1e-10);
    }

    #[test]
    fn test_extract_absorption_zero_inputs() {
        let cal = PdsCalibration {
            reference_absorption: 100.0,
            reference_signal: 0.0,  // zero ref signal
            pump_power: 1.0,
            sample_thickness: 100e-9,
        };
        let alpha = extract_absorption_coefficient(1.0, 1.0, &cal);
        assert_eq!(alpha, 0.0, "Should return 0 for zero reference signal");
    }

    #[test]
    fn test_absorption_spectrum_normalisation() {
        let deflections = vec![1.0, 2.0, 3.0];
        let lamp = vec![1.0, 2.0, 1.0];
        let spec = absorption_spectrum(&deflections, &lamp, 100.0);
        // spec = 100 * d / l
        assert!((spec[0] - 100.0).abs() < TOL);
        assert!((spec[1] - 100.0).abs() < TOL); // 2/2 * 100
        assert!((spec[2] - 300.0).abs() < TOL); // 3/1 * 100
    }

    #[test]
    fn test_absorption_spectrum_zero_lamp() {
        let deflections = vec![1.0, 2.0];
        let lamp = vec![0.0, 1.0];
        let spec = absorption_spectrum(&deflections, &lamp, 10.0);
        assert_eq!(spec[0], 0.0, "Zero lamp power should yield zero absorption");
        assert!((spec[1] - 20.0).abs() < TOL);
    }

    // -----------------------------------------------------------------------
    // Urbach tail / sub-bandgap tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_urbach_tail_absorption_at_e0() {
        // At E = E0, α = α0
        let alpha = urbach_tail_absorption(1.5, 100.0, 1.5, 0.05);
        assert!((alpha - 100.0).abs() < TOL, "At E=E0, α should equal α0: {alpha}");
    }

    #[test]
    fn test_urbach_tail_absorption_exponential_growth() {
        let alpha1 = urbach_tail_absorption(1.5, 1.0, 1.0, 0.05);
        let alpha2 = urbach_tail_absorption(1.6, 1.0, 1.0, 0.05);
        assert!(alpha2 > alpha1, "Absorption should increase with energy in Urbach tail");
    }

    #[test]
    fn test_urbach_tail_zero_eu() {
        let alpha = urbach_tail_absorption(1.5, 100.0, 1.5, 0.0);
        assert_eq!(alpha, 0.0, "Zero Urbach energy should return 0");
    }

    #[test]
    fn test_fit_urbach_energy_synthetic() {
        // Generate synthetic Urbach tail data with E_U = 0.05 eV
        let e_u_true = 0.05;
        let alpha0_true = 1.0;
        let e0 = 1.5;
        let energies: Vec<f64> = (0..20).map(|i| 1.2 + i as f64 * 0.02).collect();
        let absorptions: Vec<f64> = energies
            .iter()
            .map(|&e| urbach_tail_absorption(e, alpha0_true, e0, e_u_true))
            .collect();

        let (e_u_fit, _alpha0_fit, _e0_fit) = fit_urbach_energy(&energies, &absorptions);
        assert!(
            (e_u_fit - e_u_true).abs() < 1e-6,
            "Urbach energy fit error: {e_u_fit} vs {e_u_true}"
        );
    }

    #[test]
    fn test_fit_urbach_energy_too_few_points() {
        let (e_u, _, _) = fit_urbach_energy(&[1.5], &[100.0]);
        assert_eq!(e_u, 0.0, "Single point should return 0");
    }

    #[test]
    fn test_classify_defect_state_urbach() {
        let mat = Material::amorphous_silicon();
        let class = classify_defect_state(
            mat.bandgap - mat.urbach_energy,
            mat.bandgap,
            mat.urbach_energy,
        );
        assert_eq!(class, DefectClass::UrbachTail);
    }

    #[test]
    fn test_classify_defect_state_deep() {
        let mat = Material::amorphous_silicon(); // E_g = 1.75, E_U = 0.05
        let energy_deep = mat.bandgap * 0.3; // Deep in gap
        let class = classify_defect_state(energy_deep, mat.bandgap, mat.urbach_energy);
        assert_eq!(class, DefectClass::DeepDefect);
    }

    #[test]
    fn test_classify_defect_state_above_bandgap() {
        let class = classify_defect_state(2.0, 1.5, 0.05);
        assert_eq!(class, DefectClass::AboveBandgap);
    }

    // -----------------------------------------------------------------------
    // Thin film tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fresnel_reflectance_air_silicon() {
        // n_air=1, n_Si=3.5 → R = ((1-3.5)/(1+3.5))^2 = (2.5/4.5)^2
        let r = ThinFilm::fresnel_reflectance(1.0, 3.5);
        let expected = (2.5 / 4.5) * (2.5 / 4.5);
        assert!((r - expected).abs() < TOL);
    }

    #[test]
    fn test_fresnel_reflectance_identical_media() {
        let r = ThinFilm::fresnel_reflectance(1.5, 1.5);
        assert!(r.abs() < TOL, "Same n should give zero reflectance");
    }

    #[test]
    fn test_thin_film_absorbed_power_zero_alpha() {
        let film = ThinFilm {
            thickness: 100e-9,
            n_film: 2.0,
            n_substrate: 1.5,
            absorption_coeff: 0.0, // non-absorbing
        };
        let p_abs = film.absorbed_power(1.0);
        assert!(p_abs.abs() < 1e-10, "Non-absorbing film absorbs zero power: {p_abs}");
    }

    #[test]
    fn test_thin_film_absorbed_power_high_alpha() {
        let film = ThinFilm {
            thickness: 1e-6, // 1 µm
            n_film: 2.0,
            n_substrate: 1.5,
            absorption_coeff: 1e6, // extremely absorbing
        };
        let p_abs = film.absorbed_power(1.0);
        // Should absorb almost all incident power (minus surface reflection)
        let r01 = ThinFilm::fresnel_reflectance(1.0, 2.0);
        assert!(p_abs > 0.9 * (1.0 - r01), "High alpha film should absorb most power: {p_abs}");
    }

    #[test]
    fn test_thin_film_round_trip_phase() {
        let film = ThinFilm {
            thickness: 500e-9, // half-wave film at 2 µm for n=2
            n_film: 2.0,
            n_substrate: 1.5,
            absorption_coeff: 0.0,
        };
        // φ = 4π * n * d / λ = 4π * 2 * 500e-9 / 2000e-9 = 2π
        let phase = film.round_trip_phase(2000e-9);
        assert!((phase - 2.0 * PI).abs() < 1e-10, "Round-trip phase should be 2π: {phase}");
    }

    // -----------------------------------------------------------------------
    // Sensitivity and noise tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_minimum_detectable_absorption() {
        let mda = minimum_detectable_absorption(1e-9, 1e-5); // noise=1e-9 rad, sens=1e-5 rad/cm⁻¹
        assert!((mda - 1e-4).abs() < 1e-15, "MDA mismatch: {mda}");
    }

    #[test]
    fn test_minimum_detectable_absorption_zero_sensitivity() {
        let mda = minimum_detectable_absorption(1e-9, 0.0);
        assert!(mda.is_infinite());
    }

    #[test]
    fn test_snr_pds() {
        let snr = snr_pds(10.0, 0.1);
        assert!((snr - 100.0).abs() < TOL);
    }

    #[test]
    fn test_snr_pds_zero_noise() {
        let snr = snr_pds(10.0, 0.0);
        assert!(snr.is_infinite());
    }

    #[test]
    fn test_shot_noise_deflection_basic() {
        let noise = shot_noise_deflection(1e-9, 1e-6); // 1 nA, 1 µrad/A sensitivity
        assert!(noise > 0.0, "Shot noise should be positive");
    }

    #[test]
    fn test_shot_noise_zero_current() {
        let noise = shot_noise_deflection(0.0, 1e-6);
        assert!(noise.abs() < TOL);
    }

    // -----------------------------------------------------------------------
    // Radiative correction test
    // -----------------------------------------------------------------------

    #[test]
    fn test_radiative_cooling_power_room_temp() {
        // At T=300 K, T_amb=300 K → net = 0
        let p = radiative_cooling_power(1.0, 300.0, 300.0);
        assert!(p.abs() < TOL);
    }

    #[test]
    fn test_radiative_cooling_power_positive_for_hot_surface() {
        let p = radiative_cooling_power(0.9, 400.0, 300.0);
        assert!(p > 0.0, "Hot surface should have positive cooling power");
    }

    // -----------------------------------------------------------------------
    // Material preset tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_material_amorphous_silicon_properties() {
        let mat = Material::amorphous_silicon();
        assert_eq!(mat.name, "a-Si:H");
        assert!(mat.bandgap > 1.5 && mat.bandgap < 2.0, "a-Si:H bandgap should be ~1.75 eV");
        assert!(mat.urbach_energy > 0.0);
        assert!(mat.thermal_conductivity > 0.0);
    }

    #[test]
    fn test_material_crystalline_silicon_high_conductivity() {
        let c_si = Material::crystalline_silicon();
        let a_si = Material::amorphous_silicon();
        assert!(
            c_si.thermal_conductivity > a_si.thermal_conductivity,
            "c-Si should have higher thermal conductivity than a-Si:H"
        );
    }

    #[test]
    fn test_material_gaas_direct_bandgap() {
        let mat = Material::gaas();
        // GaAs has a direct bandgap of ~1.42 eV
        assert!((mat.bandgap - 1.42).abs() < 0.1);
    }

    #[test]
    fn test_all_materials_positive_diffusivity() {
        let materials = [
            Material::amorphous_silicon(),
            Material::crystalline_silicon(),
            Material::gaas(),
            Material::perovskite(),
            Material::organic_semiconductor(),
        ];
        for mat in &materials {
            assert!(mat.thermal_diffusivity > 0.0, "{} diffusivity must be positive", mat.name);
            assert!(mat.thermal_conductivity > 0.0, "{} conductivity must be positive", mat.name);
            assert!(mat.bandgap > 0.0, "{} bandgap must be positive", mat.name);
        }
    }

    // -----------------------------------------------------------------------
    // PDS Processor integration test
    // -----------------------------------------------------------------------

    #[test]
    fn test_pds_processor_integration() {
        let geom = PdsGeometry::default();
        let processor = PdsProcessor::new(geom, 100.0);

        let sample_rate = 10_000.0;
        let n = 10_000usize;
        let signal: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * PI * 100.0 * i as f64 / sample_rate).cos())
            .collect();

        let (amp, _phase) = processor.process_signal(&signal, sample_rate);
        assert!(amp > 0.0, "Processor should detect signal amplitude");
    }

    #[test]
    fn test_pds_processor_absorption_spectrum() {
        let geom = PdsGeometry::default();
        let processor = PdsProcessor::new(geom, 100.0);

        let deflections = vec![0.5, 1.0, 1.5, 2.0];
        let lamp = vec![1.0, 1.0, 1.0, 1.0];
        let cal = PdsCalibration {
            reference_absorption: 100.0,
            reference_signal: 1.0,
            pump_power: 1.0,
            sample_thickness: 100e-9,
        };

        let spec = processor.compute_absorption_spectrum(&deflections, &lamp, &cal);
        // Should scale linearly with deflection
        assert_eq!(spec.len(), 4);
        assert!(spec[3] > spec[0], "Spectrum should increase with deflection");
    }
}

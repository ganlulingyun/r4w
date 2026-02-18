//! # Terahertz Time-Domain Spectroscopy (THz-TDS) Processor
//!
//! Comprehensive THz-TDS signal processing for material characterization in the
//! 0.1-10 THz frequency range. Implements the complete extraction pipeline from
//! raw time-domain waveforms to complex optical constants (refractive index and
//! extinction coefficient), absorption spectra, and material identification.
//!
//! ## Processing Pipeline
//!
//! ```text
//! Reference pulse ──┐
//!                    ├─> Transfer Function H(w) ──> Phase Unwrap ──> n(w), k(w)
//! Sample pulse   ──┘                                              ──> alpha(w)
//! ```
//!
//! ## Key Equations
//!
//! - **Transfer function**: H(w) = E_sam(w) / E_ref(w)
//! - **Refractive index**: n(w) = 1 + c * phi(w) / (w * d)
//! - **Extinction coefficient**: k(w) = -c * ln(|H(w)| * T_FP) / (w * d)
//! - **Absorption coefficient**: alpha(w) = 2 * w * k(w) / c  (cm^-1)
//! - **Drude conductivity**: sigma(w) = sigma_dc / (1 - j*w*tau)
//! - **Lorentz oscillator**: eps(w) = eps_inf + S*w0^2 / (w0^2 - w^2 - j*gamma*w)
//! - **Kramers-Kronig**: n(w) - 1 = (2/pi) * P.V. integral[ w'*k(w') / (w'^2 - w^2) dw' ]
//! - **Fabry-Perot**: FP(w) = 1 / (1 - r^2 * exp(-2*j*n*w*d/c))
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::terahertz_time_domain_spectroscopy::{ThzTdsConfig, ThzTdsProcessor};
//!
//! let config = ThzTdsConfig {
//!     sampling_rate_thz: 20.0,
//!     time_window_ps: 50.0,
//!     sample_thickness_m: 1.0e-3,
//!     substrate_thickness_m: 0.0,
//!     temperature_k: 295.0,
//! };
//!
//! let n = 512;
//! let dt = 1.0 / config.sampling_rate_thz;
//! let ref_pulse: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = (i as f64 - n as f64 / 2.0) * dt;
//!         (-t * t / 0.5).exp()
//!     })
//!     .collect();
//!
//! let processor = ThzTdsProcessor::new(config.clone());
//! let spectrum = processor.fft(&ref_pulse);
//! assert_eq!(spectrum.len(), n);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Speed of light in vacuum (m/s).
const C_LIGHT: f64 = 2.997_924_58e8;

/// Two pi.
const TWO_PI: f64 = 2.0 * PI;

/// Boltzmann constant (J/K).
const K_BOLTZMANN: f64 = 1.380_649e-23;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers using (f64, f64) tuples
// ---------------------------------------------------------------------------

/// Complex addition.
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex division.
#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-60 {
        return (0.0, 0.0);
    }
    (
        (a.0 * b.0 + a.1 * b.1) / denom,
        (a.1 * b.0 - a.0 * b.1) / denom,
    )
}

/// Complex magnitude.
#[inline]
fn c_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex magnitude squared.
#[inline]
fn c_abs_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex phase (argument).
#[inline]
fn c_arg(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

/// Complex exponential: exp(a + jb) = exp(a) * (cos(b) + j*sin(b)).
#[inline]
fn c_exp(a: (f64, f64)) -> (f64, f64) {
    let r = a.0.exp();
    (r * a.1.cos(), r * a.1.sin())
}

/// Complex conjugate.
#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex natural logarithm: ln(|z|) + j*arg(z).
#[inline]
fn c_ln(a: (f64, f64)) -> (f64, f64) {
    (c_abs(a).ln(), c_arg(a))
}

/// Complex square root via polar form.
#[inline]
fn c_sqrt(a: (f64, f64)) -> (f64, f64) {
    let r = c_abs(a).sqrt();
    let theta = c_arg(a) / 2.0;
    (r * theta.cos(), r * theta.sin())
}

/// Real-valued complex number.
#[inline]
fn c_real(r: f64) -> (f64, f64) {
    (r, 0.0)
}

/// Imaginary-valued complex number.
#[inline]
fn c_imag(i: f64) -> (f64, f64) {
    (0.0, i)
}

/// Scale complex by real.
#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

// ---------------------------------------------------------------------------
// FFT implementation (radix-2 Cooley-Tukey)
// ---------------------------------------------------------------------------

/// In-place radix-2 Cooley-Tukey FFT.
///
/// `data` must have power-of-2 length.
/// If `inverse` is true, computes IFFT (with 1/N normalization).
fn fft_in_place(data: &mut [(f64, f64)], inverse: bool) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    debug_assert!(n.is_power_of_two(), "FFT length must be a power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            data.swap(i, j);
        }
    }

    // Butterfly passes
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * TWO_PI / len as f64;
        let wn = (angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = data[start + k];
                let t = c_mul(w, data[start + k + half]);
                data[start + k] = c_add(u, t);
                data[start + k + half] = c_sub(u, t);
                w = c_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    // IFFT normalization
    if inverse {
        let inv_n = 1.0 / n as f64;
        for sample in data.iter_mut() {
            sample.0 *= inv_n;
            sample.1 *= inv_n;
        }
    }
}

/// Zero-pad to next power of 2 and compute FFT.
fn fft_padded(signal: &[f64]) -> Vec<(f64, f64)> {
    let n = signal.len().next_power_of_two();
    let mut data: Vec<(f64, f64)> = Vec::with_capacity(n);
    for &s in signal {
        data.push((s, 0.0));
    }
    data.resize(n, (0.0, 0.0));
    fft_in_place(&mut data, false);
    data
}

/// Compute IFFT (inverse).
fn ifft(spectrum: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut data = spectrum.to_vec();
    fft_in_place(&mut data, true);
    data
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for THz-TDS processing.
#[derive(Debug, Clone)]
pub struct ThzTdsConfig {
    /// Sampling rate in THz (e.g., 20.0 THz = 50 fs steps).
    pub sampling_rate_thz: f64,
    /// Time window duration in picoseconds.
    pub time_window_ps: f64,
    /// Sample thickness in meters.
    pub sample_thickness_m: f64,
    /// Substrate thickness in meters (0 if no substrate).
    pub substrate_thickness_m: f64,
    /// Sample temperature in Kelvin.
    pub temperature_k: f64,
}

impl Default for ThzTdsConfig {
    fn default() -> Self {
        Self {
            sampling_rate_thz: 20.0,
            time_window_ps: 50.0,
            sample_thickness_m: 1.0e-3,
            substrate_thickness_m: 0.0,
            temperature_k: 295.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Optical constants result
// ---------------------------------------------------------------------------

/// Extracted optical constants at a set of frequencies.
#[derive(Debug, Clone)]
pub struct OpticalConstants {
    /// Frequencies in THz.
    pub freq_thz: Vec<f64>,
    /// Real part of complex refractive index n(w).
    pub n: Vec<f64>,
    /// Extinction coefficient k(w) (imaginary part of complex refractive index).
    pub k: Vec<f64>,
    /// Absorption coefficient alpha(w) in cm^-1.
    pub alpha_cm: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Material database entry
// ---------------------------------------------------------------------------

/// THz properties of a material.
#[derive(Debug, Clone)]
pub struct MaterialProperties {
    /// Material name.
    pub name: &'static str,
    /// Refractive index at 1 THz.
    pub n_1thz: f64,
    /// Absorption coefficient at 1 THz in cm^-1.
    pub alpha_1thz: f64,
    /// Characteristic absorption line frequencies in THz (if any).
    pub absorption_lines_thz: &'static [f64],
    /// Relative permittivity (real part at low frequency).
    pub epsilon_r: f64,
}

/// Built-in THz material database.
pub const MATERIAL_DATABASE: &[MaterialProperties] = &[
    MaterialProperties {
        name: "Silicon (high-resistivity)",
        n_1thz: 3.418,
        alpha_1thz: 0.05,
        absorption_lines_thz: &[],
        epsilon_r: 11.68,
    },
    MaterialProperties {
        name: "GaAs (semi-insulating)",
        n_1thz: 3.60,
        alpha_1thz: 0.5,
        absorption_lines_thz: &[8.02],
        epsilon_r: 12.9,
    },
    MaterialProperties {
        name: "HDPE (polyethylene)",
        n_1thz: 1.524,
        alpha_1thz: 0.3,
        absorption_lines_thz: &[2.22],
        epsilon_r: 2.37,
    },
    MaterialProperties {
        name: "PTFE (Teflon)",
        n_1thz: 1.434,
        alpha_1thz: 0.2,
        absorption_lines_thz: &[6.13],
        epsilon_r: 2.10,
    },
    MaterialProperties {
        name: "Water (liquid)",
        n_1thz: 2.12,
        alpha_1thz: 220.0,
        absorption_lines_thz: &[],
        epsilon_r: 78.3,
    },
    MaterialProperties {
        name: "Lactose (alpha-monohydrate)",
        n_1thz: 1.81,
        alpha_1thz: 5.0,
        absorption_lines_thz: &[0.53, 1.37, 1.82],
        epsilon_r: 3.30,
    },
    MaterialProperties {
        name: "Quartz (crystalline, ordinary)",
        n_1thz: 2.108,
        alpha_1thz: 0.1,
        absorption_lines_thz: &[3.85, 8.05],
        epsilon_r: 4.52,
    },
    MaterialProperties {
        name: "Sapphire (Al2O3, ordinary)",
        n_1thz: 3.07,
        alpha_1thz: 0.1,
        absorption_lines_thz: &[],
        epsilon_r: 9.39,
    },
];

/// Look up a material by name (case-insensitive substring match).
pub fn lookup_material(name: &str) -> Option<&'static MaterialProperties> {
    let lower = name.to_lowercase();
    MATERIAL_DATABASE.iter().find(|m| {
        m.name.to_lowercase().contains(&lower)
    })
}

// ---------------------------------------------------------------------------
// Water vapor absorption lines
// ---------------------------------------------------------------------------

/// Water vapor absorption line (frequency in THz, relative strength).
#[derive(Debug, Clone, Copy)]
pub struct WaterVaporLine {
    /// Line center frequency in THz.
    pub freq_thz: f64,
    /// Relative absorption strength (arbitrary units, normalized to strongest).
    pub strength: f64,
    /// Approximate linewidth (FWHM) in THz.
    pub linewidth_thz: f64,
}

/// Built-in database of atmospheric water vapor absorption lines in the 0.5-3 THz range.
///
/// These are the dominant rotational lines that affect THz-TDS measurements
/// in ambient atmosphere. Data from HITRAN database.
pub const WATER_VAPOR_LINES: &[WaterVaporLine] = &[
    WaterVaporLine { freq_thz: 0.557, strength: 0.15, linewidth_thz: 0.010 },
    WaterVaporLine { freq_thz: 0.752, strength: 0.35, linewidth_thz: 0.012 },
    WaterVaporLine { freq_thz: 0.988, strength: 0.10, linewidth_thz: 0.008 },
    WaterVaporLine { freq_thz: 1.097, strength: 0.30, linewidth_thz: 0.015 },
    WaterVaporLine { freq_thz: 1.113, strength: 0.20, linewidth_thz: 0.012 },
    WaterVaporLine { freq_thz: 1.163, strength: 0.25, linewidth_thz: 0.010 },
    WaterVaporLine { freq_thz: 1.229, strength: 0.20, linewidth_thz: 0.010 },
    WaterVaporLine { freq_thz: 1.411, strength: 0.15, linewidth_thz: 0.008 },
    WaterVaporLine { freq_thz: 1.602, strength: 0.10, linewidth_thz: 0.008 },
    WaterVaporLine { freq_thz: 1.661, strength: 0.45, linewidth_thz: 0.015 },
    WaterVaporLine { freq_thz: 1.670, strength: 0.50, linewidth_thz: 0.015 },
    WaterVaporLine { freq_thz: 1.717, strength: 0.35, linewidth_thz: 0.012 },
    WaterVaporLine { freq_thz: 1.794, strength: 0.20, linewidth_thz: 0.010 },
    WaterVaporLine { freq_thz: 1.868, strength: 0.60, linewidth_thz: 0.018 },
    WaterVaporLine { freq_thz: 1.920, strength: 0.15, linewidth_thz: 0.008 },
    WaterVaporLine { freq_thz: 2.164, strength: 0.25, linewidth_thz: 0.012 },
    WaterVaporLine { freq_thz: 2.196, strength: 0.20, linewidth_thz: 0.010 },
    WaterVaporLine { freq_thz: 2.255, strength: 0.10, linewidth_thz: 0.008 },
    WaterVaporLine { freq_thz: 2.390, strength: 0.15, linewidth_thz: 0.010 },
    WaterVaporLine { freq_thz: 2.531, strength: 0.20, linewidth_thz: 0.012 },
    WaterVaporLine { freq_thz: 2.674, strength: 1.00, linewidth_thz: 0.020 },
    WaterVaporLine { freq_thz: 2.774, strength: 0.30, linewidth_thz: 0.012 },
    WaterVaporLine { freq_thz: 2.873, strength: 0.15, linewidth_thz: 0.008 },
];

/// Get water vapor lines within a frequency range.
pub fn water_vapor_lines(freq_min_thz: f64, freq_max_thz: f64) -> Vec<WaterVaporLine> {
    WATER_VAPOR_LINES
        .iter()
        .filter(|l| l.freq_thz >= freq_min_thz && l.freq_thz <= freq_max_thz)
        .copied()
        .collect()
}

/// Compute water vapor absorption spectrum (Lorentzian line shapes).
///
/// Returns absorption coefficient in cm^-1 at each frequency in `freq_thz`.
/// `humidity_factor` scales line strengths (1.0 = standard conditions, ~50% RH at 295 K).
pub fn water_vapor_absorption(freq_thz: &[f64], humidity_factor: f64) -> Vec<f64> {
    freq_thz
        .iter()
        .map(|&f| {
            let mut alpha = 0.0;
            for line in WATER_VAPOR_LINES {
                let df = f - line.freq_thz;
                let hw = line.linewidth_thz / 2.0;
                // Lorentzian line shape
                alpha += humidity_factor * line.strength * 100.0 * hw * hw
                    / (df * df + hw * hw);
            }
            alpha
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Drude model
// ---------------------------------------------------------------------------

/// Drude model parameters for free carrier absorption.
#[derive(Debug, Clone)]
pub struct DrudeParams {
    /// DC conductivity (S/m).
    pub sigma_dc: f64,
    /// Scattering time / momentum relaxation time (seconds).
    pub tau: f64,
    /// High-frequency permittivity (unitless).
    pub eps_inf: f64,
}

/// Compute complex permittivity from Drude model.
///
/// eps(w) = eps_inf - w_p^2 / (w^2 + j*w*gamma)
///
/// Equivalently via conductivity:
/// eps(w) = eps_inf + j * sigma(w) / (eps_0 * w)
///
/// where sigma(w) = sigma_dc / (1 - j*w*tau)
///
/// Returns (Re[eps], Im[eps]) at each frequency.
pub fn drude_model(params: &DrudeParams, freq_thz: &[f64]) -> Vec<(f64, f64)> {
    let eps_0 = 8.854_187_8128e-12; // F/m
    freq_thz
        .iter()
        .map(|&f| {
            let omega = TWO_PI * f * 1e12;
            if omega.abs() < 1e-20 {
                return (params.eps_inf, 0.0);
            }
            // Complex conductivity: sigma(w) = sigma_dc / (1 - j*w*tau)
            let denom = c_add(c_real(1.0), c_imag(-omega * params.tau));
            let sigma = c_div(c_real(params.sigma_dc), denom);
            // eps(w) = eps_inf + j * sigma / (eps_0 * omega)
            let j_sigma_over_eps0_omega = c_mul(c_imag(1.0), c_scale(sigma, 1.0 / (eps_0 * omega)));
            c_add(c_real(params.eps_inf), j_sigma_over_eps0_omega)
        })
        .collect()
}

/// Compute complex refractive index from Drude model: n_complex = sqrt(eps).
pub fn drude_refractive_index(params: &DrudeParams, freq_thz: &[f64]) -> Vec<(f64, f64)> {
    drude_model(params, freq_thz)
        .iter()
        .map(|&eps| c_sqrt(eps))
        .collect()
}

// ---------------------------------------------------------------------------
// Lorentz oscillator model
// ---------------------------------------------------------------------------

/// Lorentz oscillator parameters for phonon/vibrational resonances.
#[derive(Debug, Clone)]
pub struct LorentzOscillator {
    /// Resonance frequency in THz.
    pub freq_0_thz: f64,
    /// Oscillator strength (dimensionless).
    pub strength: f64,
    /// Damping rate / linewidth in THz.
    pub gamma_thz: f64,
}

/// Compute complex permittivity from one or more Lorentz oscillators.
///
/// eps(w) = eps_inf + sum_j [ S_j * w0_j^2 / (w0_j^2 - w^2 - j*gamma_j*w) ]
///
/// Returns (Re[eps], Im[eps]) at each frequency.
pub fn lorentz_oscillator(
    eps_inf: f64,
    oscillators: &[LorentzOscillator],
    freq_thz: &[f64],
) -> Vec<(f64, f64)> {
    freq_thz
        .iter()
        .map(|&f| {
            let omega = TWO_PI * f * 1e12;
            let omega_sq = omega * omega;
            let mut eps = c_real(eps_inf);
            for osc in oscillators {
                let omega_0 = TWO_PI * osc.freq_0_thz * 1e12;
                let omega_0_sq = omega_0 * omega_0;
                let gamma = TWO_PI * osc.gamma_thz * 1e12;
                // Numerator: S * w0^2
                let num = c_real(osc.strength * omega_0_sq);
                // Denominator: w0^2 - w^2 - j*gamma*w
                let den = c_add(c_real(omega_0_sq - omega_sq), c_imag(-gamma * omega));
                eps = c_add(eps, c_div(num, den));
            }
            eps
        })
        .collect()
}

/// Compute complex refractive index from Lorentz oscillator model: n_complex = sqrt(eps).
pub fn lorentz_refractive_index(
    eps_inf: f64,
    oscillators: &[LorentzOscillator],
    freq_thz: &[f64],
) -> Vec<(f64, f64)> {
    lorentz_oscillator(eps_inf, oscillators, freq_thz)
        .iter()
        .map(|&eps| c_sqrt(eps))
        .collect()
}

// ---------------------------------------------------------------------------
// ThzTdsProcessor
// ---------------------------------------------------------------------------

/// THz Time-Domain Spectroscopy processor.
///
/// Provides methods for the complete THz-TDS extraction pipeline:
/// pulse generation, FFT, transfer function, phase unwrapping,
/// optical constant extraction, deconvolution, and Fabry-Perot removal.
#[derive(Debug, Clone)]
pub struct ThzTdsProcessor {
    config: ThzTdsConfig,
}

impl ThzTdsProcessor {
    /// Create a new THz-TDS processor with the given configuration.
    pub fn new(config: ThzTdsConfig) -> Self {
        Self { config }
    }

    /// Get the configuration.
    pub fn config(&self) -> &ThzTdsConfig {
        &self.config
    }

    // -------------------------------------------------------------------
    // Pulse generation
    // -------------------------------------------------------------------

    /// Generate a THz pulse as the derivative of a Gaussian (monocycle).
    ///
    /// `t0_ps` is the pulse center in picoseconds, `tau_ps` is the pulse
    /// width parameter in picoseconds. The number of samples is determined
    /// by the time window and sampling rate.
    ///
    /// The monocycle is: E(t) = -(t - t0) / tau^2 * exp(-(t - t0)^2 / (2 * tau^2))
    pub fn generate_thz_pulse(&self, t0_ps: f64, tau_ps: f64) -> Vec<f64> {
        let n = self.num_samples();
        let dt = self.dt_ps();
        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let dt_from_center = t - t0_ps;
                let norm = dt_from_center / (tau_ps * tau_ps);
                -norm * (-dt_from_center * dt_from_center / (2.0 * tau_ps * tau_ps)).exp()
            })
            .collect()
    }

    /// Generate a Gaussian envelope pulse (not monocycle).
    ///
    /// E(t) = exp(-(t - t0)^2 / (2 * tau^2))
    pub fn generate_gaussian_pulse(&self, t0_ps: f64, tau_ps: f64) -> Vec<f64> {
        let n = self.num_samples();
        let dt = self.dt_ps();
        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let dt_from_center = t - t0_ps;
                (-dt_from_center * dt_from_center / (2.0 * tau_ps * tau_ps)).exp()
            })
            .collect()
    }

    // -------------------------------------------------------------------
    // FFT / IFFT
    // -------------------------------------------------------------------

    /// Compute the FFT of a real-valued time-domain signal.
    ///
    /// The input is zero-padded to the next power of 2.
    /// Returns complex spectrum as (Re, Im) tuples.
    pub fn fft(&self, signal: &[f64]) -> Vec<(f64, f64)> {
        fft_padded(signal)
    }

    /// Compute the IFFT of a complex spectrum.
    ///
    /// Returns complex time-domain signal as (Re, Im) tuples.
    pub fn ifft(&self, spectrum: &[(f64, f64)]) -> Vec<(f64, f64)> {
        ifft(spectrum)
    }

    /// Frequency axis for a given FFT length (in THz).
    pub fn frequency_axis(&self, fft_len: usize) -> Vec<f64> {
        let df = self.config.sampling_rate_thz / fft_len as f64;
        (0..fft_len).map(|i| i as f64 * df).collect()
    }

    // -------------------------------------------------------------------
    // Transfer function
    // -------------------------------------------------------------------

    /// Compute the transfer function H(w) = sample_spectrum / reference_spectrum.
    ///
    /// Both spectra must have the same length.
    /// Returns complex transfer function as (Re, Im) tuples.
    pub fn transfer_function(
        &self,
        reference_spectrum: &[(f64, f64)],
        sample_spectrum: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        assert_eq!(
            reference_spectrum.len(),
            sample_spectrum.len(),
            "Reference and sample spectra must have the same length"
        );
        reference_spectrum
            .iter()
            .zip(sample_spectrum.iter())
            .map(|(&r, &s)| c_div(s, r))
            .collect()
    }

    // -------------------------------------------------------------------
    // Phase unwrapping
    // -------------------------------------------------------------------

    /// Unwrap phase from an array of wrapped phase values (radians).
    ///
    /// Detects jumps greater than pi and adds/subtracts 2*pi to maintain
    /// continuity. This is essential for extracting refractive index
    /// from the transfer function phase.
    pub fn phase_unwrapping(&self, wrapped_phase: &[f64]) -> Vec<f64> {
        phase_unwrap(wrapped_phase)
    }

    // -------------------------------------------------------------------
    // Optical constant extraction
    // -------------------------------------------------------------------

    /// Extract complex refractive index n(w) + j*k(w) from reference and sample
    /// time-domain waveforms.
    ///
    /// Uses the standard THz-TDS extraction procedure:
    /// 1. FFT both reference and sample
    /// 2. Compute transfer function H(w) = S(w)/R(w)
    /// 3. Unwrap phase of H(w)
    /// 4. n(w) = 1 + c * phi(w) / (w * d)
    /// 5. k(w) = -c * ln(|H(w)| * FP_correction) / (w * d)
    ///
    /// Returns `OpticalConstants` with n, k, and alpha at each frequency.
    pub fn extract_optical_constants(
        &self,
        reference: &[f64],
        sample: &[f64],
    ) -> OpticalConstants {
        let ref_spec = self.fft(reference);
        let sam_spec = self.fft(sample);
        let fft_len = ref_spec.len();

        let h = self.transfer_function(&ref_spec, &sam_spec);
        let freq_axis = self.frequency_axis(fft_len);

        // Extract magnitude and phase
        let magnitude: Vec<f64> = h.iter().map(|&z| c_abs(z)).collect();
        let phase: Vec<f64> = h.iter().map(|&z| c_arg(z)).collect();
        let unwrapped = self.phase_unwrapping(&phase);

        let d = self.config.sample_thickness_m;
        let usable = fft_len / 2; // Only positive frequencies are physically meaningful

        let mut n_vals = Vec::with_capacity(usable);
        let mut k_vals = Vec::with_capacity(usable);
        let mut alpha_vals = Vec::with_capacity(usable);
        let mut freq_out = Vec::with_capacity(usable);

        for i in 1..usable {
            let f = freq_axis[i];
            let omega = TWO_PI * f * 1e12; // Convert THz to rad/s
            if omega.abs() < 1e-20 || d < 1e-15 {
                continue;
            }

            // Fresnel transmission coefficient correction for air-sample interface
            // T = 4*n / (n+1)^2 -- will be iteratively refined in practice
            // For first pass, use |H| directly with T ~ 1
            let n_f = 1.0 + C_LIGHT * unwrapped[i] / (omega * d);

            // Extinction coefficient from magnitude
            // Account for Fresnel losses: T_12 * T_21 = 4n/(n+1)^2
            let fresnel_t = if n_f > 0.1 {
                4.0 * n_f / ((n_f + 1.0) * (n_f + 1.0))
            } else {
                1.0
            };

            let k_f = if magnitude[i] > 1e-30 && fresnel_t > 1e-30 {
                -C_LIGHT * (magnitude[i] / fresnel_t).ln() / (omega * d)
            } else {
                0.0
            };

            // Absorption coefficient: alpha = 2*omega*k/c (in m^-1), convert to cm^-1
            let alpha = 2.0 * omega * k_f.abs() / C_LIGHT * 0.01; // m^-1 -> cm^-1

            freq_out.push(f);
            n_vals.push(n_f);
            k_vals.push(k_f);
            alpha_vals.push(alpha);
        }

        OpticalConstants {
            freq_thz: freq_out,
            n: n_vals,
            k: k_vals,
            alpha_cm: alpha_vals,
        }
    }

    // -------------------------------------------------------------------
    // Absorption coefficient
    // -------------------------------------------------------------------

    /// Compute absorption coefficient alpha(w) = 2*w*k/c in cm^-1.
    ///
    /// `k` is the extinction coefficient, `freq_thz` are frequencies in THz.
    pub fn absorption_coefficient(&self, k: &[f64], freq_thz: &[f64]) -> Vec<f64> {
        k.iter()
            .zip(freq_thz.iter())
            .map(|(&ki, &f)| {
                let omega = TWO_PI * f * 1e12;
                2.0 * omega * ki.abs() / C_LIGHT * 0.01 // m^-1 to cm^-1
            })
            .collect()
    }

    // -------------------------------------------------------------------
    // Deconvolution
    // -------------------------------------------------------------------

    /// Deconvolve instrument response from measured signal.
    ///
    /// Uses Wiener deconvolution: D(w) = S(w) * conj(R(w)) / (|R(w)|^2 + epsilon)
    ///
    /// `epsilon` is the regularization parameter to avoid noise amplification.
    /// Returns deconvolved time-domain signal.
    pub fn deconvolution(
        &self,
        measured: &[f64],
        instrument_response: &[f64],
        epsilon: f64,
    ) -> Vec<f64> {
        let meas_spec = self.fft(measured);
        let inst_spec = self.fft(instrument_response);
        let n = meas_spec.len();

        let mut deconv_spec = Vec::with_capacity(n);
        for i in 0..n {
            let s = meas_spec[i];
            let r = inst_spec[i];
            let r_conj = c_conj(r);
            let num = c_mul(s, r_conj);
            let denom = c_abs_sq(r) + epsilon;
            deconv_spec.push(c_scale(num, 1.0 / denom));
        }

        let time_domain = ifft(&deconv_spec);
        time_domain.iter().map(|&(re, _im)| re).collect()
    }

    // -------------------------------------------------------------------
    // Fabry-Perot removal
    // -------------------------------------------------------------------

    /// Remove Fabry-Perot etalon artifacts from the transfer function.
    ///
    /// For a parallel-sided sample of thickness d and refractive index n,
    /// internal reflections create periodic oscillations (etalon fringes).
    ///
    /// FP(w) = 1 / (1 - r^2 * exp(-2*j*n*w*d/c))
    ///
    /// where r = (n-1)/(n+1) is the Fresnel reflection coefficient.
    ///
    /// The corrected transfer function is H_corrected = H * FP.
    pub fn fabry_perot_removal(
        &self,
        transfer_func: &[(f64, f64)],
        n_est: &[f64],
        freq_thz: &[f64],
    ) -> Vec<(f64, f64)> {
        let d = self.config.sample_thickness_m;
        transfer_func
            .iter()
            .enumerate()
            .map(|(i, &h)| {
                if i >= n_est.len() || i >= freq_thz.len() {
                    return h;
                }
                let n = n_est[i];
                if n.abs() < 1e-10 {
                    return h;
                }
                let r = (n - 1.0) / (n + 1.0);
                let omega = TWO_PI * freq_thz[i] * 1e12;
                // FP correction factor
                let phase = -2.0 * n * omega * d / C_LIGHT;
                let r_sq_exp = c_scale(c_exp((0.0, phase)), r * r);
                let fp_denom = c_sub(c_real(1.0), r_sq_exp);
                let fp = c_div(c_real(1.0), fp_denom);
                // H_corrected = H * FP (divide out etalon)
                c_mul(h, fp)
            })
            .collect()
    }

    // -------------------------------------------------------------------
    // Thickness extraction
    // -------------------------------------------------------------------

    /// Determine sample thickness from time-of-flight difference between
    /// reference and sample pulses.
    ///
    /// d = c * delta_t / (2 * (n - 1))  for transmission mode
    ///
    /// `n_approx` is an approximate refractive index of the sample material.
    /// Returns thickness in meters.
    pub fn thickness_extraction(
        &self,
        reference: &[f64],
        sample: &[f64],
        n_approx: f64,
    ) -> f64 {
        let dt_ps = self.dt_ps();

        // Find peak positions
        let ref_peak = find_peak_index(reference);
        let sam_peak = find_peak_index(sample);

        // Time delay in seconds
        let delta_samples = sam_peak as f64 - ref_peak as f64;
        let delta_t = delta_samples * dt_ps * 1e-12; // ps to seconds

        // d = c * delta_t / (2 * (n - 1)) for double-pass (reflection)
        // d = c * delta_t / (n - 1) for single-pass (transmission)
        // Standard THz-TDS is transmission mode:
        if n_approx > 1.0 + 1e-10 {
            C_LIGHT * delta_t / (n_approx - 1.0)
        } else {
            // Fallback: assume n ~ 1.5
            C_LIGHT * delta_t / 0.5
        }
    }

    // -------------------------------------------------------------------
    // Dynamic range
    // -------------------------------------------------------------------

    /// Compute frequency-dependent dynamic range from the reference spectrum
    /// and noise floor.
    ///
    /// DR(f) = 20 * log10(|E_ref(f)| / noise_floor(f))  in dB
    ///
    /// `noise` should be a measurement with no THz signal (e.g., blocked beam).
    /// Returns DR in dB at each frequency bin.
    pub fn dynamic_range(
        &self,
        reference: &[f64],
        noise: &[f64],
    ) -> Vec<f64> {
        let ref_spec = self.fft(reference);
        let noise_spec = self.fft(noise);
        let n = ref_spec.len().min(noise_spec.len());

        (0..n)
            .map(|i| {
                let ref_mag = c_abs(ref_spec[i]);
                let noise_mag = c_abs(noise_spec[i]).max(1e-30);
                20.0 * (ref_mag / noise_mag).log10()
            })
            .collect()
    }

    // -------------------------------------------------------------------
    // SNR spectrum
    // -------------------------------------------------------------------

    /// Compute frequency-dependent signal-to-noise ratio.
    ///
    /// SNR(f) = |E_signal(f)|^2 / |E_noise(f)|^2  (linear)
    ///
    /// Returns SNR in dB at each frequency bin.
    pub fn snr_spectrum(
        &self,
        signal: &[f64],
        noise: &[f64],
    ) -> Vec<f64> {
        let sig_spec = self.fft(signal);
        let noise_spec = self.fft(noise);
        let n = sig_spec.len().min(noise_spec.len());

        (0..n)
            .map(|i| {
                let sig_power = c_abs_sq(sig_spec[i]);
                let noise_power = c_abs_sq(noise_spec[i]).max(1e-60);
                10.0 * (sig_power / noise_power).log10()
            })
            .collect()
    }

    // -------------------------------------------------------------------
    // Kramers-Kronig consistency check
    // -------------------------------------------------------------------

    /// Verify consistency of extracted n and k via Kramers-Kronig relations.
    ///
    /// The KK relation connects n(w) and k(w):
    /// n(w) - 1 = (2/pi) * P.V. integral[ w' * k(w') / (w'^2 - w^2) dw' ]
    ///
    /// Returns the KK-reconstructed n(w) for comparison with measured n(w).
    /// A small discrepancy indicates self-consistent optical constants.
    pub fn kramers_kronig(
        &self,
        k: &[f64],
        freq_thz: &[f64],
    ) -> Vec<f64> {
        let n_freq = freq_thz.len();
        if n_freq < 3 {
            return vec![1.0; n_freq];
        }

        let df = if n_freq > 1 {
            (freq_thz[n_freq - 1] - freq_thz[0]) / (n_freq - 1) as f64
        } else {
            1.0
        };

        let mut n_kk = Vec::with_capacity(n_freq);
        for i in 0..n_freq {
            let omega = freq_thz[i];
            let omega_sq = omega * omega;
            let mut integral = 0.0;

            // Cauchy principal value integration using trapezoidal rule
            // skipping the singular point
            for j in 0..n_freq {
                if j == i {
                    continue;
                }
                let omega_prime = freq_thz[j];
                let omega_prime_sq = omega_prime * omega_prime;
                let denom = omega_prime_sq - omega_sq;
                if denom.abs() < 1e-30 {
                    continue;
                }
                integral += omega_prime * k[j] / denom;
            }

            let n_val = 1.0 + (2.0 / PI) * integral * df;
            n_kk.push(n_val);
        }

        n_kk
    }

    // -------------------------------------------------------------------
    // Time windowing
    // -------------------------------------------------------------------

    /// Apply a time window to isolate pulses and suppress reflections.
    ///
    /// Supported window types: "tukey" (default, cosine taper parameter `alpha`
    /// controls the taper fraction 0..1), "hann" (equivalent to Tukey alpha=1).
    ///
    /// `center_sample` and `width_samples` define the window position and extent.
    pub fn time_windowing(
        &self,
        signal: &[f64],
        window_type: &str,
        center_sample: usize,
        width_samples: usize,
        alpha: f64,
    ) -> Vec<f64> {
        let n = signal.len();
        let half_width = width_samples / 2;
        let start = center_sample.saturating_sub(half_width);
        let end = (center_sample + half_width).min(n);

        let window = match window_type.to_lowercase().as_str() {
            "hann" => generate_hann_window(end - start),
            _ => generate_tukey_window(end - start, alpha),
        };

        let mut result = vec![0.0; n];
        for (i, w) in window.iter().enumerate() {
            let idx = start + i;
            if idx < n {
                result[idx] = signal[idx] * w;
            }
        }
        result
    }

    // -------------------------------------------------------------------
    // Helper methods
    // -------------------------------------------------------------------

    /// Number of time samples based on config.
    fn num_samples(&self) -> usize {
        let dt_ps = self.dt_ps();
        if dt_ps <= 0.0 {
            return 0;
        }
        (self.config.time_window_ps / dt_ps).ceil() as usize
    }

    /// Time step in picoseconds.
    fn dt_ps(&self) -> f64 {
        if self.config.sampling_rate_thz <= 0.0 {
            return 0.0;
        }
        1.0 / self.config.sampling_rate_thz
    }
}

// ---------------------------------------------------------------------------
// Free-standing utility functions
// ---------------------------------------------------------------------------

/// Unwrap phase to remove 2*pi discontinuities.
pub fn phase_unwrap(wrapped: &[f64]) -> Vec<f64> {
    if wrapped.is_empty() {
        return vec![];
    }
    let mut unwrapped = vec![0.0; wrapped.len()];
    unwrapped[0] = wrapped[0];
    for i in 1..wrapped.len() {
        let mut diff = wrapped[i] - wrapped[i - 1];
        // Bring diff into (-pi, pi]
        while diff > PI {
            diff -= TWO_PI;
        }
        while diff <= -PI {
            diff += TWO_PI;
        }
        unwrapped[i] = unwrapped[i - 1] + diff;
    }
    unwrapped
}

/// Find the index of the peak (maximum absolute value) in a signal.
fn find_peak_index(signal: &[f64]) -> usize {
    signal
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0)
}

/// Generate a Tukey (cosine-tapered) window.
///
/// `alpha` = 0 gives a rectangular window, `alpha` = 1 gives a Hann window.
fn generate_tukey_window(n: usize, alpha: f64) -> Vec<f64> {
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![1.0];
    }
    let alpha = alpha.clamp(0.0, 1.0);
    let nm1 = (n - 1) as f64;
    (0..n)
        .map(|i| {
            let x = i as f64 / nm1;
            if alpha < 1e-10 {
                1.0
            } else if x < alpha / 2.0 {
                0.5 * (1.0 + (TWO_PI * x / alpha - PI).cos())
            } else if x > 1.0 - alpha / 2.0 {
                0.5 * (1.0 + (TWO_PI * (x - 1.0) / alpha + PI).cos())
            } else {
                1.0
            }
        })
        .collect()
}

/// Generate a Hann window.
fn generate_hann_window(n: usize) -> Vec<f64> {
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![1.0];
    }
    let nm1 = (n - 1) as f64;
    (0..n)
        .map(|i| 0.5 * (1.0 - (TWO_PI * i as f64 / nm1).cos()))
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> ThzTdsProcessor {
        ThzTdsProcessor::new(ThzTdsConfig::default())
    }

    fn make_config(sr: f64, tw: f64, thick: f64) -> ThzTdsConfig {
        ThzTdsConfig {
            sampling_rate_thz: sr,
            time_window_ps: tw,
            sample_thickness_m: thick,
            substrate_thickness_m: 0.0,
            temperature_k: 295.0,
        }
    }

    // ---------------------------------------------------------------
    // FFT tests
    // ---------------------------------------------------------------

    #[test]
    fn test_fft_dc_signal() {
        let proc = default_processor();
        let signal = vec![1.0; 8];
        let spectrum = proc.fft(&signal);
        assert_eq!(spectrum.len(), 8);
        // DC bin should equal N
        assert!((spectrum[0].0 - 8.0).abs() < 1e-10);
        // All other bins should be near zero
        for i in 1..8 {
            assert!(c_abs(spectrum[i]) < 1e-10, "Bin {} should be zero", i);
        }
    }

    #[test]
    fn test_fft_pure_tone() {
        let proc = default_processor();
        let n = 64;
        // Single frequency at bin 3
        let signal: Vec<f64> = (0..n)
            .map(|i| (TWO_PI * 3.0 * i as f64 / n as f64).cos())
            .collect();
        let spectrum = proc.fft(&signal);
        assert_eq!(spectrum.len(), n);
        // Bin 3 and bin N-3 should have the energy
        let mag3 = c_abs(spectrum[3]);
        let mag_n3 = c_abs(spectrum[n - 3]);
        assert!(mag3 > 20.0, "Expected energy at bin 3, got {}", mag3);
        assert!(mag_n3 > 20.0);
        // Other bins should be small
        for i in [0, 1, 2, 4, 5, 6] {
            assert!(c_abs(spectrum[i]) < 1e-10, "Bin {} should be near zero", i);
        }
    }

    #[test]
    fn test_fft_ifft_roundtrip() {
        let proc = default_processor();
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let spectrum = proc.fft(&signal);
        let reconstructed = proc.ifft(&spectrum);
        for (i, &(re, im)) in reconstructed.iter().enumerate() {
            assert!(
                (re - signal[i]).abs() < 1e-10,
                "Sample {} mismatch: expected {}, got {}",
                i, signal[i], re
            );
            assert!(im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_fft_parseval() {
        let proc = default_processor();
        let signal: Vec<f64> = (0..16)
            .map(|i| (0.5 * i as f64).sin() + 0.3 * (1.2 * i as f64).cos())
            .collect();
        let spectrum = proc.fft(&signal);

        let time_energy: f64 = signal.iter().map(|x| x * x).sum();
        let freq_energy: f64 = spectrum.iter().map(|z| c_abs_sq(*z)).sum::<f64>() / signal.len() as f64;
        assert!(
            (time_energy - freq_energy).abs() < 1e-8,
            "Parseval's theorem: {} vs {}",
            time_energy,
            freq_energy
        );
    }

    // ---------------------------------------------------------------
    // Pulse generation tests
    // ---------------------------------------------------------------

    #[test]
    fn test_generate_thz_pulse_monocycle() {
        let config = make_config(20.0, 10.0, 1e-3);
        let proc = ThzTdsProcessor::new(config);
        let pulse = proc.generate_thz_pulse(5.0, 0.5);
        assert!(!pulse.is_empty());
        // Monocycle should be bipolar: has both positive and negative values
        let has_pos = pulse.iter().any(|&x| x > 0.01);
        let has_neg = pulse.iter().any(|&x| x < -0.01);
        assert!(has_pos, "Monocycle should have positive values");
        assert!(has_neg, "Monocycle should have negative values");
    }

    #[test]
    fn test_generate_thz_pulse_peak_location() {
        let config = make_config(20.0, 10.0, 1e-3);
        let proc = ThzTdsProcessor::new(config);
        let pulse = proc.generate_thz_pulse(5.0, 0.5);
        // The pulse should have its extrema near t0 = 5 ps
        let peak_idx = find_peak_index(&pulse);
        let dt = 1.0 / 20.0; // 0.05 ps
        let peak_time = peak_idx as f64 * dt;
        assert!(
            (peak_time - 5.0).abs() < 1.0,
            "Peak should be near 5 ps, got {} ps",
            peak_time
        );
    }

    #[test]
    fn test_generate_gaussian_pulse() {
        let config = make_config(20.0, 10.0, 1e-3);
        let proc = ThzTdsProcessor::new(config);
        let pulse = proc.generate_gaussian_pulse(5.0, 0.5);
        assert!(!pulse.is_empty());
        // Gaussian should be non-negative
        assert!(pulse.iter().all(|&x| x >= -1e-15));
        // Peak should be near 1.0
        let peak = pulse.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            (peak - 1.0).abs() < 0.01,
            "Gaussian peak should be ~1.0, got {}",
            peak
        );
    }

    // ---------------------------------------------------------------
    // Transfer function tests
    // ---------------------------------------------------------------

    #[test]
    fn test_transfer_function_unity() {
        let proc = default_processor();
        let spectrum = vec![(1.0, 0.0), (2.0, 1.0), (0.5, -0.3)];
        let h = proc.transfer_function(&spectrum, &spectrum);
        for &(re, im) in &h {
            assert!((re - 1.0).abs() < 1e-10);
            assert!(im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_transfer_function_half() {
        let proc = default_processor();
        let ref_spec = vec![(2.0, 0.0), (4.0, 0.0)];
        let sam_spec = vec![(1.0, 0.0), (2.0, 0.0)];
        let h = proc.transfer_function(&ref_spec, &sam_spec);
        assert!((h[0].0 - 0.5).abs() < 1e-10);
        assert!((h[1].0 - 0.5).abs() < 1e-10);
    }

    // ---------------------------------------------------------------
    // Phase unwrapping tests
    // ---------------------------------------------------------------

    #[test]
    fn test_phase_unwrapping_linear() {
        let proc = default_processor();
        // A linearly increasing phase that wraps
        let n = 100;
        let wrapped: Vec<f64> = (0..n)
            .map(|i| {
                let phase = 0.1 * i as f64;
                // Wrap to [-pi, pi)
                let mut p = phase % TWO_PI;
                if p > PI {
                    p -= TWO_PI;
                }
                p
            })
            .collect();
        let unwrapped = proc.phase_unwrapping(&wrapped);
        // Check that unwrapped phase is approximately linear
        for i in 2..n {
            let diff1 = unwrapped[i] - unwrapped[i - 1];
            let diff2 = unwrapped[i - 1] - unwrapped[i - 2];
            assert!(
                (diff1 - diff2).abs() < 0.1,
                "Phase should be linear at index {}: diff1={}, diff2={}",
                i, diff1, diff2
            );
        }
    }

    #[test]
    fn test_phase_unwrapping_no_wrap() {
        let proc = default_processor();
        let phase = vec![0.1, 0.2, 0.3, 0.4, 0.5];
        let unwrapped = proc.phase_unwrapping(&phase);
        for (i, (&orig, &unw)) in phase.iter().zip(unwrapped.iter()).enumerate() {
            assert!(
                (orig - unw).abs() < 1e-10,
                "No wrapping needed at index {}",
                i
            );
        }
    }

    #[test]
    fn test_phase_unwrapping_single_wrap() {
        let proc = default_processor();
        // Phase jumps from ~PI to ~-PI (a wrap)
        let wrapped = vec![0.0, 1.0, 2.0, 3.0, -3.0, -2.0, -1.0];
        let unwrapped = proc.phase_unwrapping(&wrapped);
        // After unwrapping, the phase at index 4 should be ~3.28 (continuous)
        assert!(
            unwrapped[4] > unwrapped[3],
            "Phase should continue increasing after unwrap"
        );
    }

    // ---------------------------------------------------------------
    // Optical constant extraction tests
    // ---------------------------------------------------------------

    #[test]
    fn test_extract_optical_constants_basic() {
        let config = make_config(20.0, 25.6, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        let n = 512;
        let dt = 1.0 / 20.0;
        // Reference pulse
        let reference: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i as f64 - n as f64 / 2.0) * dt;
                (-t * t / 0.5).exp()
            })
            .collect();
        // Sample pulse: slightly delayed and attenuated
        let sample: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i as f64 - n as f64 / 2.0 - 2.0) * dt;
                0.8 * (-t * t / 0.5).exp()
            })
            .collect();

        let oc = proc.extract_optical_constants(&reference, &sample);
        assert!(!oc.freq_thz.is_empty());
        assert_eq!(oc.n.len(), oc.freq_thz.len());
        assert_eq!(oc.k.len(), oc.freq_thz.len());
        assert_eq!(oc.alpha_cm.len(), oc.freq_thz.len());
    }

    #[test]
    fn test_extract_optical_constants_n_positive() {
        let config = make_config(20.0, 25.6, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        let n = 256;
        let dt = 1.0 / 20.0;
        let reference: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i as f64 - n as f64 / 2.0) * dt;
                (-t * t / 0.3).exp()
            })
            .collect();
        let sample: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i as f64 - n as f64 / 2.0 - 1.0) * dt;
                0.9 * (-t * t / 0.3).exp()
            })
            .collect();

        let oc = proc.extract_optical_constants(&reference, &sample);
        // Most n values should be positive (physical materials)
        let positive_count = oc.n.iter().filter(|&&n| n > 0.0).count();
        assert!(
            positive_count > oc.n.len() / 2,
            "Most n values should be positive"
        );
    }

    // ---------------------------------------------------------------
    // Absorption coefficient tests
    // ---------------------------------------------------------------

    #[test]
    fn test_absorption_coefficient_zero_k() {
        let proc = default_processor();
        let k = vec![0.0, 0.0, 0.0];
        let freq = vec![0.5, 1.0, 2.0];
        let alpha = proc.absorption_coefficient(&k, &freq);
        assert!(alpha.iter().all(|&a| a.abs() < 1e-15));
    }

    #[test]
    fn test_absorption_coefficient_proportional_to_frequency() {
        let proc = default_processor();
        let k = vec![0.01, 0.01, 0.01];
        let freq = vec![0.5, 1.0, 2.0];
        let alpha = proc.absorption_coefficient(&k, &freq);
        // alpha ~ omega * k, so should scale with frequency
        assert!(alpha[1] > alpha[0]);
        assert!(alpha[2] > alpha[1]);
        // Check ratios
        assert!((alpha[1] / alpha[0] - 2.0).abs() < 1e-6);
        assert!((alpha[2] / alpha[1] - 2.0).abs() < 1e-6);
    }

    // ---------------------------------------------------------------
    // Drude model tests
    // ---------------------------------------------------------------

    #[test]
    fn test_drude_model_dc_limit() {
        let params = DrudeParams {
            sigma_dc: 1000.0,
            tau: 1e-14,
            eps_inf: 10.0,
        };
        let freq = vec![0.001]; // Near DC
        let eps = drude_model(&params, &freq);
        // At low frequency, real part should be large due to conductivity
        // eps_r should be dominated by eps_inf + sigma/(eps_0 * omega)
        assert!(eps[0].0.is_finite());
    }

    #[test]
    fn test_drude_model_high_frequency() {
        let params = DrudeParams {
            sigma_dc: 100.0,
            tau: 1e-14,
            eps_inf: 5.0,
        };
        let freq = vec![10.0]; // 10 THz, w*tau >> 1
        let eps = drude_model(&params, &freq);
        // At high frequency, should approach eps_inf
        assert!(
            (eps[0].0 - params.eps_inf).abs() < params.eps_inf * 0.5,
            "High-freq eps should approach eps_inf: got {}",
            eps[0].0
        );
    }

    #[test]
    fn test_drude_refractive_index_positive() {
        let params = DrudeParams {
            sigma_dc: 10.0,
            tau: 1e-13,
            eps_inf: 12.0,
        };
        let freq = vec![0.5, 1.0, 2.0, 5.0];
        let n_complex = drude_refractive_index(&params, &freq);
        // Real part of n should be positive for dielectric
        for (i, &(n_re, _)) in n_complex.iter().enumerate() {
            assert!(
                n_re > 0.0,
                "n at {} THz should be positive, got {}",
                freq[i], n_re
            );
        }
    }

    // ---------------------------------------------------------------
    // Lorentz oscillator tests
    // ---------------------------------------------------------------

    #[test]
    fn test_lorentz_oscillator_single_resonance() {
        let oscillators = vec![LorentzOscillator {
            freq_0_thz: 1.0,
            strength: 0.5,
            gamma_thz: 0.1,
        }];
        let freq: Vec<f64> = (1..200).map(|i| i as f64 * 0.02).collect();
        let eps = lorentz_oscillator(2.0, &oscillators, &freq);

        // At resonance, imaginary part should peak
        let at_res_idx = freq.iter().position(|&f| (f - 1.0).abs() < 0.03).unwrap();
        let im_at_res = eps[at_res_idx].1.abs();

        // Far from resonance, imaginary part should be smaller
        let far_idx = freq.iter().position(|&f| (f - 3.0).abs() < 0.03).unwrap();
        let im_far = eps[far_idx].1.abs();

        assert!(
            im_at_res > im_far,
            "Im(eps) should peak at resonance: {} vs {}",
            im_at_res, im_far
        );
    }

    #[test]
    fn test_lorentz_oscillator_off_resonance() {
        let oscillators = vec![LorentzOscillator {
            freq_0_thz: 5.0,
            strength: 1.0,
            gamma_thz: 0.5,
        }];
        let freq = vec![0.5]; // Well below resonance
        let eps = lorentz_oscillator(3.0, &oscillators, &freq);
        // Far below resonance: Re(eps) > eps_inf, Im(eps) small
        assert!(eps[0].0 > 3.0, "Below resonance, Re(eps) > eps_inf");
    }

    #[test]
    fn test_lorentz_refractive_index() {
        let oscillators = vec![LorentzOscillator {
            freq_0_thz: 2.0,
            strength: 0.3,
            gamma_thz: 0.2,
        }];
        let freq = vec![0.5, 1.0, 3.0];
        let n = lorentz_refractive_index(2.0, &oscillators, &freq);
        // n should be finite and positive (real part)
        for &(n_re, _) in &n {
            assert!(n_re.is_finite() && n_re > 0.0);
        }
    }

    // ---------------------------------------------------------------
    // Deconvolution tests
    // ---------------------------------------------------------------

    #[test]
    fn test_deconvolution_delta() {
        let config = make_config(20.0, 0.8, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        // Instrument response: delta function (shifted)
        let n = 16;
        let mut instrument = vec![0.0; n];
        instrument[0] = 1.0;

        // Measured signal: should equal true signal after deconvolution with delta
        let measured: Vec<f64> = (0..n)
            .map(|i| (TWO_PI * 2.0 * i as f64 / n as f64).cos())
            .collect();

        let deconvolved = proc.deconvolution(&measured, &instrument, 1e-10);
        // Should approximately recover the input
        for i in 0..n {
            assert!(
                (deconvolved[i] - measured[i]).abs() < 0.1,
                "Sample {}: expected {}, got {}",
                i,
                measured[i],
                deconvolved[i]
            );
        }
    }

    // ---------------------------------------------------------------
    // Fabry-Perot removal tests
    // ---------------------------------------------------------------

    #[test]
    fn test_fabry_perot_removal_n1() {
        let config = make_config(20.0, 10.0, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        // For n = 1 (vacuum), FP correction should be identity (r = 0)
        let h = vec![(0.5, 0.1), (0.3, -0.2), (0.8, 0.0)];
        let n_est = vec![1.0, 1.0, 1.0];
        let freq = vec![0.5, 1.0, 2.0];

        let h_corrected = proc.fabry_perot_removal(&h, &n_est, &freq);
        for (i, (&orig, &corr)) in h.iter().zip(h_corrected.iter()).enumerate() {
            assert!(
                (orig.0 - corr.0).abs() < 1e-10 && (orig.1 - corr.1).abs() < 1e-10,
                "FP correction for n=1 should be identity at index {}",
                i
            );
        }
    }

    #[test]
    fn test_fabry_perot_removal_changes_transfer_function() {
        let config = make_config(20.0, 10.0, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        let h = vec![(0.5, 0.1); 10];
        let n_est = vec![3.0; 10];
        let freq: Vec<f64> = (0..10).map(|i| 0.5 + i as f64 * 0.3).collect();

        let h_corrected = proc.fabry_perot_removal(&h, &n_est, &freq);
        // With n=3, there should be some correction
        let any_different = h.iter().zip(h_corrected.iter()).any(|(&a, &b)| {
            (a.0 - b.0).abs() > 1e-6 || (a.1 - b.1).abs() > 1e-6
        });
        assert!(any_different, "FP correction should modify H for n=3");
    }

    // ---------------------------------------------------------------
    // Thickness extraction tests
    // ---------------------------------------------------------------

    #[test]
    fn test_thickness_extraction() {
        let config = make_config(20.0, 25.6, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        let n = 512;
        let dt = 1.0 / 20.0;
        // Reference pulse centered at 12.8 ps
        let ref_center = 12.8;
        let reference: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                (-(t - ref_center).powi(2) / 0.5).exp()
            })
            .collect();

        // Sample pulse delayed by delta_t corresponding to thickness
        // d = c * delta_t / (n - 1), delta_t = d * (n-1) / c
        let target_d = 1e-3; // 1 mm
        let n_mat = 1.5;
        let delta_t_ps = target_d * (n_mat - 1.0) / C_LIGHT * 1e12;
        let sam_center = ref_center + delta_t_ps;
        let sample: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                (-(t - sam_center).powi(2) / 0.5).exp()
            })
            .collect();

        let extracted_d = proc.thickness_extraction(&reference, &sample, n_mat);
        let error = (extracted_d - target_d).abs();
        assert!(
            error < 0.1e-3,
            "Extracted thickness {} should be near {} (error={})",
            extracted_d, target_d, error
        );
    }

    // ---------------------------------------------------------------
    // Dynamic range tests
    // ---------------------------------------------------------------

    #[test]
    fn test_dynamic_range_positive() {
        let proc = default_processor();
        // Signal much larger than noise
        let signal: Vec<f64> = (0..64).map(|i| (0.3 * i as f64).sin()).collect();
        let noise: Vec<f64> = (0..64).map(|i| 0.001 * (7.1 * i as f64).sin()).collect();

        let dr = proc.dynamic_range(&signal, &noise);
        // Most bins should have positive DR
        let positive_bins = dr.iter().filter(|&&d| d > 0.0).count();
        assert!(
            positive_bins > dr.len() / 4,
            "Expected mostly positive DR values"
        );
    }

    // ---------------------------------------------------------------
    // SNR spectrum tests
    // ---------------------------------------------------------------

    #[test]
    fn test_snr_spectrum_high_snr() {
        let proc = default_processor();
        let signal: Vec<f64> = (0..64).map(|i| (0.5 * i as f64).sin()).collect();
        let noise: Vec<f64> = (0..64).map(|i| 0.001 * (13.3 * i as f64).sin()).collect();

        let snr = proc.snr_spectrum(&signal, &noise);
        // Should have some bins with high SNR
        let max_snr = snr.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_snr > 10.0, "Expected high SNR bins, max was {} dB", max_snr);
    }

    #[test]
    fn test_snr_spectrum_equal_signal_noise() {
        let proc = default_processor();
        let signal: Vec<f64> = (0..32).map(|i| (0.5 * i as f64).sin()).collect();
        let snr = proc.snr_spectrum(&signal, &signal);
        // SNR should be 0 dB everywhere
        for (i, &s) in snr.iter().enumerate() {
            assert!(
                s.abs() < 1e-6,
                "SNR at bin {} should be 0 dB, got {}",
                i, s
            );
        }
    }

    // ---------------------------------------------------------------
    // Kramers-Kronig tests
    // ---------------------------------------------------------------

    #[test]
    fn test_kramers_kronig_zero_k() {
        let proc = default_processor();
        let k = vec![0.0; 50];
        let freq: Vec<f64> = (1..51).map(|i| i as f64 * 0.1).collect();
        let n_kk = proc.kramers_kronig(&k, &freq);
        // With k=0, KK should give n ~ 1
        for &n in &n_kk {
            assert!(
                (n - 1.0).abs() < 1e-6,
                "KK with k=0 should give n=1, got {}",
                n
            );
        }
    }

    #[test]
    fn test_kramers_kronig_finite() {
        let proc = default_processor();
        // Lorentzian-shaped k
        let n_pts = 100;
        let freq: Vec<f64> = (1..=n_pts).map(|i| i as f64 * 0.05).collect();
        let k: Vec<f64> = freq
            .iter()
            .map(|&f| {
                let f0 = 2.0;
                let gamma = 0.3;
                0.1 * gamma / ((f - f0) * (f - f0) + gamma * gamma)
            })
            .collect();

        let n_kk = proc.kramers_kronig(&k, &freq);
        // n_kk should be finite and have some structure
        assert!(n_kk.iter().all(|&n| n.is_finite()));
        // Anomalous dispersion: n should increase then decrease around resonance
        let max_n = n_kk.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_n = n_kk.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            max_n > min_n,
            "KK should produce dispersion variation"
        );
    }

    // ---------------------------------------------------------------
    // Time windowing tests
    // ---------------------------------------------------------------

    #[test]
    fn test_time_windowing_tukey() {
        let config = make_config(20.0, 25.6, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        let n = 512;
        let signal = vec![1.0; n];
        let windowed = proc.time_windowing(&signal, "tukey", 256, 100, 0.5);

        assert_eq!(windowed.len(), n);
        // Outside window region should be zero
        assert!((windowed[0]).abs() < 1e-10);
        assert!((windowed[n - 1]).abs() < 1e-10);
        // Center of window should be 1.0
        assert!((windowed[256] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_time_windowing_hann() {
        let config = make_config(20.0, 5.0, 1e-3);
        let proc = ThzTdsProcessor::new(config);

        let n = 100;
        let signal = vec![1.0; n];
        let windowed = proc.time_windowing(&signal, "hann", 50, 40, 0.0);

        // Hann window edges should be zero
        assert_eq!(windowed.len(), n);
        // Outside window should be zero
        assert!((windowed[0]).abs() < 1e-10);
    }

    #[test]
    fn test_time_windowing_preserves_length() {
        let proc = default_processor();
        let signal = vec![1.0; 200];
        let windowed = proc.time_windowing(&signal, "tukey", 100, 50, 0.3);
        assert_eq!(windowed.len(), signal.len());
    }

    // ---------------------------------------------------------------
    // Water vapor lines tests
    // ---------------------------------------------------------------

    #[test]
    fn test_water_vapor_lines_range() {
        let lines = water_vapor_lines(1.0, 2.0);
        assert!(!lines.is_empty());
        for line in &lines {
            assert!(line.freq_thz >= 1.0 && line.freq_thz <= 2.0);
        }
    }

    #[test]
    fn test_water_vapor_lines_full_range() {
        let lines = water_vapor_lines(0.0, 10.0);
        assert_eq!(lines.len(), WATER_VAPOR_LINES.len());
    }

    #[test]
    fn test_water_vapor_absorption_peaks() {
        let freq: Vec<f64> = (0..600).map(|i| 0.5 + i as f64 * 0.005).collect();
        let alpha = water_vapor_absorption(&freq, 1.0);
        assert_eq!(alpha.len(), freq.len());
        // Should have peaks at water vapor line positions
        let max_alpha = alpha.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_alpha > 1.0, "Should have significant absorption peaks");
    }

    #[test]
    fn test_water_vapor_absorption_humidity_scaling() {
        let freq = vec![1.661]; // Strong water line
        let alpha_normal = water_vapor_absorption(&freq, 1.0);
        let alpha_double = water_vapor_absorption(&freq, 2.0);
        assert!(
            (alpha_double[0] / alpha_normal[0] - 2.0).abs() < 1e-6,
            "Humidity scaling should be linear"
        );
    }

    // ---------------------------------------------------------------
    // Material database tests
    // ---------------------------------------------------------------

    #[test]
    fn test_material_database_lookup() {
        let si = lookup_material("silicon");
        assert!(si.is_some());
        let si = si.unwrap();
        assert!((si.n_1thz - 3.418).abs() < 0.01);
    }

    #[test]
    fn test_material_database_lookup_case_insensitive() {
        let teflon = lookup_material("TEFLON");
        assert!(teflon.is_some());
        assert!(teflon.unwrap().name.contains("PTFE"));
    }

    #[test]
    fn test_material_database_not_found() {
        assert!(lookup_material("unobtanium").is_none());
    }

    #[test]
    fn test_material_database_lactose_lines() {
        let lac = lookup_material("lactose").unwrap();
        assert!(!lac.absorption_lines_thz.is_empty());
        // Lactose has well-known absorption features
        assert!(lac.absorption_lines_thz.contains(&0.53));
        assert!(lac.absorption_lines_thz.contains(&1.37));
    }

    // ---------------------------------------------------------------
    // Window function tests
    // ---------------------------------------------------------------

    #[test]
    fn test_tukey_window_rectangular() {
        let w = generate_tukey_window(10, 0.0);
        assert!(w.iter().all(|&v| (v - 1.0).abs() < 1e-10));
    }

    #[test]
    fn test_tukey_window_hann_equivalent() {
        let tukey = generate_tukey_window(64, 1.0);
        let hann = generate_hann_window(64);
        for (i, (&t, &h)) in tukey.iter().zip(hann.iter()).enumerate() {
            assert!(
                (t - h).abs() < 1e-10,
                "Tukey(1.0) should equal Hann at index {}: {} vs {}",
                i, t, h
            );
        }
    }

    #[test]
    fn test_hann_window_endpoints() {
        let w = generate_hann_window(32);
        assert!(w[0].abs() < 1e-10, "Hann window should start at 0");
        assert!(w[31].abs() < 1e-10, "Hann window should end at 0");
        // Peak at center
        let mid = w[15];
        assert!(mid > 0.95, "Hann window should peak near center");
    }

    // ---------------------------------------------------------------
    // Complex arithmetic tests
    // ---------------------------------------------------------------

    #[test]
    fn test_complex_div_by_self() {
        let z = (3.0, 4.0);
        let result = c_div(z, z);
        assert!((result.0 - 1.0).abs() < 1e-10);
        assert!(result.1.abs() < 1e-10);
    }

    #[test]
    fn test_complex_exp_euler() {
        // e^(j*pi) = -1
        let result = c_exp((0.0, PI));
        assert!((result.0 - (-1.0)).abs() < 1e-10);
        assert!(result.1.abs() < 1e-10);
    }

    #[test]
    fn test_complex_sqrt_positive_real() {
        let result = c_sqrt((4.0, 0.0));
        assert!((result.0 - 2.0).abs() < 1e-10);
        assert!(result.1.abs() < 1e-10);
    }

    // ---------------------------------------------------------------
    // Edge case tests
    // ---------------------------------------------------------------

    #[test]
    fn test_fft_single_sample() {
        let proc = default_processor();
        let signal = vec![42.0];
        let spectrum = proc.fft(&signal);
        assert_eq!(spectrum.len(), 1);
        assert!((spectrum[0].0 - 42.0).abs() < 1e-10);
    }

    #[test]
    fn test_phase_unwrap_empty() {
        let result = phase_unwrap(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_frequency_axis() {
        let config = make_config(10.0, 10.0, 1e-3);
        let proc = ThzTdsProcessor::new(config);
        let freq = proc.frequency_axis(100);
        assert_eq!(freq.len(), 100);
        assert!((freq[0]).abs() < 1e-10);
        // df = 10.0 / 100 = 0.1 THz
        assert!((freq[1] - 0.1).abs() < 1e-10);
        assert!((freq[50] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_default_config() {
        let config = ThzTdsConfig::default();
        assert!((config.sampling_rate_thz - 20.0).abs() < 1e-10);
        assert!((config.time_window_ps - 50.0).abs() < 1e-10);
        assert!((config.temperature_k - 295.0).abs() < 1e-10);
    }
}

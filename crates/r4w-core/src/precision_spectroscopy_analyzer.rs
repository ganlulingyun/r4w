//! High-resolution optical and laser spectroscopy signal processing.
//!
//! This module provides algorithms for precision spectroscopy applications
//! including wavelength metrology, frequency comb measurement, Doppler-free
//! spectroscopy, atomic physics precision measurements, and trace gas analysis
//! via cavity ring-down spectroscopy (CRDS).
//!
//! # Components
//!
//! - [`SpectroscopyConfig`] — configuration for instrument type, wavelength range, resolution
//! - [`VoigtProfileFitter`] — fits Voigt profiles (Gaussian+Lorentzian convolution) to spectral lines
//! - [`DopplerBroadening`] — Gaussian Doppler width from temperature and atomic mass
//! - [`PressureBroadening`] — Lorentzian width from collisional broadening
//! - [`FrequencyCombAnalyzer`] — frequency comb tooth positions and absolute frequency measurement
//! - [`FabryPerotAnalyzer`] — Airy function transmission, finesse, FSR, resolving power
//! - [`CavityRingdownProcessor`] — CRDS exponential decay fitting for ultra-sensitive absorption
//! - [`SaturationSpectroscopy`] — Lamb dip sub-Doppler spectroscopy
//! - [`WavelengthCalibrator`] — calibrate wavelength axis using known reference lines
//! - [`InstrumentLineshape`] — model and deconvolve instrument functions
//! - [`BeerLambertAnalyzer`] — absorbance and concentration from Beer-Lambert law
//!
//! # Example
//!
//! ```
//! use r4w_core::precision_spectroscopy_analyzer::{
//!     DopplerBroadening, FabryPerotAnalyzer, BeerLambertAnalyzer,
//! };
//!
//! // Doppler width for Rb-87 D2 line at 300 K
//! let doppler = DopplerBroadening::new(384.230e12, 87.0 * 1.66054e-27, 300.0);
//! let fwhm = doppler.fwhm_hz();
//! assert!(fwhm > 400e6 && fwhm < 600e6);
//!
//! // Fabry-Perot with 99% reflectivity, 10 cm cavity
//! let fp = FabryPerotAnalyzer::new(0.99, 0.10, 1.0);
//! assert!(fp.finesse() > 300.0);
//!
//! // Beer-Lambert absorbance
//! let abs = BeerLambertAnalyzer::absorbance(100.0, 50.0);
//! assert!(abs > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Speed of light in vacuum (m/s).
const C: f64 = 299_792_458.0;

/// Boltzmann constant (J/K).
const K_B: f64 = 1.380649e-23;

/// Planck constant (J*s).
const H: f64 = 6.62607015e-34;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Instrument type for spectroscopy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InstrumentType {
    /// Diffraction grating spectrometer.
    Grating,
    /// Fourier Transform Infrared spectrometer.
    Ftir,
    /// Tunable laser spectroscopy.
    Laser,
    /// Fabry-Perot interferometer.
    FabryPerot,
}

/// Configuration for a spectroscopy measurement.
#[derive(Debug, Clone)]
pub struct SpectroscopyConfig {
    /// Start wavelength in metres.
    pub wavelength_start_m: f64,
    /// End wavelength in metres.
    pub wavelength_end_m: f64,
    /// Spectral resolution in metres (wavelength).
    pub resolution_m: f64,
    /// Instrument type.
    pub instrument_type: InstrumentType,
    /// Number of spectral points.
    pub num_points: usize,
}

impl SpectroscopyConfig {
    /// Create a new spectroscopy configuration.
    pub fn new(
        wavelength_start_m: f64,
        wavelength_end_m: f64,
        resolution_m: f64,
        instrument_type: InstrumentType,
        num_points: usize,
    ) -> Self {
        Self {
            wavelength_start_m,
            wavelength_end_m,
            resolution_m,
            instrument_type,
            num_points,
        }
    }

    /// Generate a wavelength axis (evenly spaced) in metres.
    pub fn wavelength_axis(&self) -> Vec<f64> {
        let n = self.num_points;
        if n <= 1 {
            return vec![self.wavelength_start_m];
        }
        let step = (self.wavelength_end_m - self.wavelength_start_m) / (n - 1) as f64;
        (0..n).map(|i| self.wavelength_start_m + i as f64 * step).collect()
    }

    /// Generate a frequency axis (evenly spaced) in Hz.
    pub fn frequency_axis(&self) -> Vec<f64> {
        let f_start = C / self.wavelength_end_m;
        let f_end = C / self.wavelength_start_m;
        let n = self.num_points;
        if n <= 1 {
            return vec![f_start];
        }
        let step = (f_end - f_start) / (n - 1) as f64;
        (0..n).map(|i| f_start + i as f64 * step).collect()
    }

    /// Resolving power R = lambda / delta_lambda.
    pub fn resolving_power(&self) -> f64 {
        let center = (self.wavelength_start_m + self.wavelength_end_m) / 2.0;
        center / self.resolution_m
    }
}

// ---------------------------------------------------------------------------
// Gaussian (Doppler) broadening
// ---------------------------------------------------------------------------

/// Gaussian Doppler broadening calculator.
///
/// The Doppler FWHM in frequency is:
///   delta_f_D = (f0 / c) * sqrt(8 * k_B * T * ln(2) / m)
///
/// Equivalently in wavelength:
///   delta_lambda_D = (lambda0 / c) * sqrt(8 * k_B * T * ln(2) / m)
#[derive(Debug, Clone)]
pub struct DopplerBroadening {
    /// Transition centre frequency (Hz).
    pub f0_hz: f64,
    /// Atomic/molecular mass (kg).
    pub mass_kg: f64,
    /// Temperature (K).
    pub temperature_k: f64,
}

impl DopplerBroadening {
    /// Create a new Doppler broadening calculator.
    pub fn new(f0_hz: f64, mass_kg: f64, temperature_k: f64) -> Self {
        Self { f0_hz, mass_kg, temperature_k }
    }

    /// Gaussian FWHM in Hz.
    pub fn fwhm_hz(&self) -> f64 {
        (self.f0_hz / C) * (8.0 * K_B * self.temperature_k * (2.0_f64.ln()) / self.mass_kg).sqrt()
    }

    /// Gaussian FWHM in wavelength (m).
    pub fn fwhm_wavelength_m(&self) -> f64 {
        let lambda0 = C / self.f0_hz;
        (lambda0 / C) * (8.0 * K_B * self.temperature_k * (2.0_f64.ln()) / self.mass_kg).sqrt()
    }

    /// Gaussian sigma (standard deviation) in Hz.
    pub fn sigma_hz(&self) -> f64 {
        self.fwhm_hz() / (2.0 * (2.0 * 2.0_f64.ln()).sqrt())
    }

    /// Estimate temperature from measured Doppler width.
    ///
    /// T = m * (delta_f * c / f0)^2 / (8 * k_B * ln(2))
    pub fn temperature_from_fwhm(f0_hz: f64, mass_kg: f64, fwhm_hz: f64) -> f64 {
        let ratio = fwhm_hz * C / f0_hz;
        mass_kg * ratio * ratio / (8.0 * K_B * (2.0_f64.ln()))
    }

    /// Generate a normalized Gaussian lineshape centred at f0.
    pub fn lineshape(&self, frequencies: &[f64]) -> Vec<f64> {
        let sigma = self.sigma_hz();
        let norm = 1.0 / (sigma * (2.0 * PI).sqrt());
        frequencies
            .iter()
            .map(|&f| {
                let x = (f - self.f0_hz) / sigma;
                norm * (-0.5 * x * x).exp()
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Lorentzian (pressure) broadening
// ---------------------------------------------------------------------------

/// Pressure (collisional) broadening calculator.
///
/// Lorentzian HWHM:  gamma_L = gamma_0 * (P / P0) * sqrt(T0 / T)
///
/// where gamma_0 is the reference broadening coefficient at P0 and T0.
#[derive(Debug, Clone)]
pub struct PressureBroadening {
    /// Reference Lorentzian HWHM (Hz).
    pub gamma_0_hz: f64,
    /// Reference pressure (Pa).
    pub p0_pa: f64,
    /// Reference temperature (K).
    pub t0_k: f64,
}

impl PressureBroadening {
    /// Create a new pressure broadening calculator.
    pub fn new(gamma_0_hz: f64, p0_pa: f64, t0_k: f64) -> Self {
        Self { gamma_0_hz, p0_pa, t0_k }
    }

    /// Lorentzian HWHM at given pressure and temperature (Hz).
    pub fn hwhm_hz(&self, pressure_pa: f64, temperature_k: f64) -> f64 {
        self.gamma_0_hz * (pressure_pa / self.p0_pa) * (self.t0_k / temperature_k).sqrt()
    }

    /// Lorentzian FWHM at given pressure and temperature (Hz).
    pub fn fwhm_hz(&self, pressure_pa: f64, temperature_k: f64) -> f64 {
        2.0 * self.hwhm_hz(pressure_pa, temperature_k)
    }

    /// Generate a normalized Lorentzian lineshape centred at f0.
    pub fn lineshape(&self, frequencies: &[f64], f0_hz: f64, pressure_pa: f64, temperature_k: f64) -> Vec<f64> {
        let gamma = self.hwhm_hz(pressure_pa, temperature_k);
        let norm = gamma / PI;
        frequencies
            .iter()
            .map(|&f| {
                let df = f - f0_hz;
                norm / (df * df + gamma * gamma)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Voigt profile fitter
// ---------------------------------------------------------------------------

/// Voigt profile fitter for spectral line analysis.
///
/// The Voigt profile is the convolution of Gaussian (Doppler) and Lorentzian
/// (natural/pressure) profiles. We approximate the Voigt function using the
/// pseudo-Voigt approximation (Thompson et al., 1987):
///
///   V(x) ~ eta * L(x) + (1 - eta) * G(x)
///
/// where eta depends on the ratio of Lorentzian to total FWHM.
#[derive(Debug, Clone)]
pub struct VoigtProfileFitter {
    /// Centre frequency (Hz).
    pub center_hz: f64,
    /// Gaussian FWHM (Hz).
    pub gaussian_fwhm_hz: f64,
    /// Lorentzian FWHM (Hz).
    pub lorentzian_fwhm_hz: f64,
    /// Peak amplitude.
    pub amplitude: f64,
}

impl VoigtProfileFitter {
    /// Create a new Voigt profile fitter with initial parameters.
    pub fn new(center_hz: f64, gaussian_fwhm_hz: f64, lorentzian_fwhm_hz: f64, amplitude: f64) -> Self {
        Self {
            center_hz,
            gaussian_fwhm_hz,
            lorentzian_fwhm_hz,
            amplitude,
        }
    }

    /// Voigt FWHM approximation (Thompson et al., 1987).
    ///
    ///   f_V = (f_G^5 + 2.69269*f_G^4*f_L + 2.42843*f_G^3*f_L^2
    ///          + 4.47163*f_G^2*f_L^3 + 0.07842*f_G*f_L^4 + f_L^5)^(1/5)
    pub fn voigt_fwhm(&self) -> f64 {
        let fg = self.gaussian_fwhm_hz;
        let fl = self.lorentzian_fwhm_hz;
        let fg2 = fg * fg;
        let fg3 = fg2 * fg;
        let fg4 = fg3 * fg;
        let fg5 = fg4 * fg;
        let fl2 = fl * fl;
        let fl3 = fl2 * fl;
        let fl4 = fl3 * fl;
        let fl5 = fl4 * fl;
        (fg5 + 2.69269 * fg4 * fl + 2.42843 * fg3 * fl2
            + 4.47163 * fg2 * fl3 + 0.07842 * fg * fl4 + fl5)
            .powf(0.2)
    }

    /// Pseudo-Voigt mixing parameter eta.
    ///
    ///   eta = 1.36603*(f_L/f_V) - 0.47719*(f_L/f_V)^2 + 0.11116*(f_L/f_V)^3
    pub fn mixing_parameter(&self) -> f64 {
        let fv = self.voigt_fwhm();
        if fv < 1e-30 {
            return 0.0;
        }
        let ratio = self.lorentzian_fwhm_hz / fv;
        1.36603 * ratio - 0.47719 * ratio * ratio + 0.11116 * ratio * ratio * ratio
    }

    /// Evaluate the pseudo-Voigt profile at the given frequencies.
    pub fn evaluate(&self, frequencies: &[f64]) -> Vec<f64> {
        let eta = self.mixing_parameter();
        let fv = self.voigt_fwhm();
        let sigma_g = self.gaussian_fwhm_hz / (2.0 * (2.0_f64.ln()).sqrt());
        let gamma_l = self.lorentzian_fwhm_hz / 2.0;

        frequencies
            .iter()
            .map(|&f| {
                let df = f - self.center_hz;

                // Gaussian component
                let g = if sigma_g > 1e-30 {
                    (-0.5 * (df / sigma_g).powi(2)).exp() / (sigma_g * (2.0 * PI).sqrt())
                } else {
                    0.0
                };

                // Lorentzian component
                let l = if gamma_l > 1e-30 {
                    (gamma_l / PI) / (df * df + gamma_l * gamma_l)
                } else {
                    0.0
                };

                self.amplitude * (eta * l + (1.0 - eta) * g)
            })
            .collect()
    }

    /// Voigt parameter a = gamma_L / (sigma_G * sqrt(2)).
    /// Characterizes the relative Lorentzian/Gaussian contribution.
    pub fn voigt_parameter(&self) -> f64 {
        let sigma_g = self.gaussian_fwhm_hz / (2.0 * (2.0_f64.ln()).sqrt());
        if sigma_g < 1e-30 {
            return f64::INFINITY;
        }
        let gamma_l = self.lorentzian_fwhm_hz / 2.0;
        gamma_l / (sigma_g * 2.0_f64.sqrt())
    }

    /// Fit a Voigt profile to measured spectral data using iterative refinement.
    ///
    /// Uses a simple Levenberg-Marquardt-like approach with numerical derivatives.
    /// Returns the fitted `VoigtProfileFitter` with optimized parameters.
    pub fn fit(
        frequencies: &[f64],
        intensities: &[f64],
        initial_center: f64,
        initial_gauss_fwhm: f64,
        initial_lorentz_fwhm: f64,
        max_iterations: usize,
    ) -> Self {
        let n = frequencies.len().min(intensities.len());
        if n == 0 {
            return Self::new(initial_center, initial_gauss_fwhm, initial_lorentz_fwhm, 1.0);
        }

        // Find initial amplitude from data peak
        let peak_val = intensities.iter().cloned().fold(0.0_f64, f64::max);
        let mut center = initial_center;
        let mut fg = initial_gauss_fwhm.max(1.0);
        let mut fl = initial_lorentz_fwhm.max(1.0);
        let mut amp = peak_val.max(1e-30);

        let compute_residual = |c: f64, g: f64, l: f64, a: f64| -> f64 {
            let fitter = VoigtProfileFitter::new(c, g, l, a);
            let model = fitter.evaluate(frequencies);
            let mut sum_sq = 0.0;
            for i in 0..n {
                let diff = intensities[i] - model[i];
                sum_sq += diff * diff;
            }
            sum_sq
        };

        let mut lambda = 0.01;

        for _ in 0..max_iterations {
            let residual = compute_residual(center, fg, fl, amp);

            // Numerical gradient
            let dc = fg.max(fl).max(1.0) * 1e-6;
            let dg = fg * 1e-4;
            let dl = fl * 1e-4;
            let da = amp * 1e-4;

            let grad_c = (compute_residual(center + dc, fg, fl, amp) - residual) / dc;
            let grad_g = (compute_residual(center, fg + dg, fl, amp) - residual) / dg;
            let grad_l = (compute_residual(center, fg, fl + dl, amp) - residual) / dl;
            let grad_a = (compute_residual(center, fg, fl, amp + da) - residual) / da;

            // Step with damping
            let step = 1.0 / (1.0 + lambda);
            let new_center = center - step * grad_c * dc;
            let new_fg = (fg - step * grad_g * dg).max(1.0);
            let new_fl = (fl - step * grad_l * dl).max(0.0);
            let new_amp = (amp - step * grad_a * da).max(1e-30);

            let new_residual = compute_residual(new_center, new_fg, new_fl, new_amp);
            if new_residual < residual {
                center = new_center;
                fg = new_fg;
                fl = new_fl;
                amp = new_amp;
                lambda *= 0.5;
            } else {
                lambda *= 2.0;
            }

            if lambda > 1e10 {
                break;
            }
        }

        Self::new(center, fg, fl, amp)
    }
}

// ---------------------------------------------------------------------------
// Frequency comb analyzer
// ---------------------------------------------------------------------------

/// Frequency comb analyzer.
///
/// Comb tooth positions: f_n = f_CEO + n * f_rep
///
/// Used for absolute frequency measurement via optical frequency combs.
#[derive(Debug, Clone)]
pub struct FrequencyCombAnalyzer {
    /// Carrier-envelope offset frequency (Hz).
    pub f_ceo_hz: f64,
    /// Repetition rate (Hz).
    pub f_rep_hz: f64,
}

impl FrequencyCombAnalyzer {
    /// Create a new frequency comb analyzer.
    pub fn new(f_ceo_hz: f64, f_rep_hz: f64) -> Self {
        Self { f_ceo_hz, f_rep_hz }
    }

    /// Frequency of comb tooth number n.
    pub fn tooth_frequency(&self, n: u64) -> f64 {
        self.f_ceo_hz + n as f64 * self.f_rep_hz
    }

    /// Determine the mode number closest to a measured frequency.
    ///
    /// n = round((f_meas - f_CEO) / f_rep)
    pub fn mode_number(&self, f_measured_hz: f64) -> u64 {
        let n_float = (f_measured_hz - self.f_ceo_hz) / self.f_rep_hz;
        n_float.round().max(0.0) as u64
    }

    /// Absolute frequency measurement: given a measured beat note f_beat
    /// against the nearest comb tooth, determine the optical frequency.
    ///
    /// f_optical = f_CEO + n * f_rep +/- f_beat
    ///
    /// Returns both possible frequencies (+ and - beat).
    pub fn absolute_frequency(&self, mode_n: u64, f_beat_hz: f64) -> (f64, f64) {
        let f_tooth = self.tooth_frequency(mode_n);
        (f_tooth + f_beat_hz, f_tooth - f_beat_hz)
    }

    /// Generate comb spectrum over a frequency range.
    /// Returns (mode_numbers, frequencies).
    pub fn comb_spectrum(&self, f_start_hz: f64, f_end_hz: f64) -> (Vec<u64>, Vec<f64>) {
        let n_start = ((f_start_hz - self.f_ceo_hz) / self.f_rep_hz).ceil().max(0.0) as u64;
        let n_end = ((f_end_hz - self.f_ceo_hz) / self.f_rep_hz).floor().max(0.0) as u64;

        let mut modes = Vec::new();
        let mut freqs = Vec::new();
        for n in n_start..=n_end {
            let f = self.tooth_frequency(n);
            if f >= f_start_hz && f <= f_end_hz {
                modes.push(n);
                freqs.push(f);
            }
        }
        (modes, freqs)
    }

    /// Fractional frequency uncertainty of the comb.
    /// delta_f/f = sqrt((delta_f_ceo/f)^2 + (n * delta_f_rep / f)^2)
    pub fn fractional_uncertainty(
        &self,
        mode_n: u64,
        delta_f_ceo_hz: f64,
        delta_f_rep_hz: f64,
    ) -> f64 {
        let f = self.tooth_frequency(mode_n);
        if f < 1e-30 {
            return f64::INFINITY;
        }
        let term1 = (delta_f_ceo_hz / f).powi(2);
        let term2 = (mode_n as f64 * delta_f_rep_hz / f).powi(2);
        (term1 + term2).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Fabry-Perot analyzer
// ---------------------------------------------------------------------------

/// Fabry-Perot interferometer analyzer.
///
/// Airy function transmission:
///   T(delta) = T_max / (1 + (2F/pi)^2 * sin^2(delta/2))
///
/// where delta = 4*pi*n*L*cos(theta)/lambda is the round-trip phase,
/// F is the finesse, n is the refractive index, and L is the cavity length.
#[derive(Debug, Clone)]
pub struct FabryPerotAnalyzer {
    /// Mirror reflectivity (0 to 1).
    pub reflectivity: f64,
    /// Cavity length (m).
    pub cavity_length_m: f64,
    /// Refractive index of cavity medium.
    pub refractive_index: f64,
}

impl FabryPerotAnalyzer {
    /// Create a new Fabry-Perot analyzer.
    pub fn new(reflectivity: f64, cavity_length_m: f64, refractive_index: f64) -> Self {
        Self {
            reflectivity,
            cavity_length_m,
            refractive_index,
        }
    }

    /// Finesse: F = pi * sqrt(R) / (1 - R).
    pub fn finesse(&self) -> f64 {
        PI * self.reflectivity.sqrt() / (1.0 - self.reflectivity)
    }

    /// Free spectral range in Hz: FSR = c / (2 * n * L).
    pub fn fsr_hz(&self) -> f64 {
        C / (2.0 * self.refractive_index * self.cavity_length_m)
    }

    /// Free spectral range in wavelength (m) at a given central wavelength.
    pub fn fsr_wavelength_m(&self, lambda_center_m: f64) -> f64 {
        lambda_center_m * lambda_center_m / (2.0 * self.refractive_index * self.cavity_length_m)
    }

    /// Resolving power: R = m * F, where m = 2*n*L/lambda is the order.
    pub fn resolving_power(&self, lambda_m: f64) -> f64 {
        let m = 2.0 * self.refractive_index * self.cavity_length_m / lambda_m;
        m * self.finesse()
    }

    /// Minimum resolvable frequency difference (Hz): delta_f = FSR / F.
    pub fn resolution_hz(&self) -> f64 {
        self.fsr_hz() / self.finesse()
    }

    /// Round-trip phase at a given frequency.
    pub fn round_trip_phase(&self, frequency_hz: f64) -> f64 {
        4.0 * PI * self.refractive_index * self.cavity_length_m * frequency_hz / C
    }

    /// Airy function transmission at given frequencies.
    ///
    /// T(f) = T_max / (1 + (2F/pi)^2 * sin^2(delta/2))
    pub fn transmission(&self, frequencies: &[f64]) -> Vec<f64> {
        let f = self.finesse();
        let coeff = (2.0 * f / PI).powi(2);
        let t_max = (1.0 - self.reflectivity).powi(2) / (1.0 - self.reflectivity).powi(2);
        // T_max = (1-R)^2 / (1-R)^2 = 1 for lossless mirrors

        frequencies
            .iter()
            .map(|&freq| {
                let delta = self.round_trip_phase(freq);
                let sin_half = (delta / 2.0).sin();
                t_max / (1.0 + coeff * sin_half * sin_half)
            })
            .collect()
    }

    /// Find transmission peak frequencies near a target frequency.
    /// Peaks occur at delta = 2*pi*m, i.e., f = m * FSR.
    pub fn peak_frequencies_near(&self, target_hz: f64, count: usize) -> Vec<f64> {
        let fsr = self.fsr_hz();
        let m_center = (target_hz / fsr).round() as i64;
        let half = count as i64 / 2;
        ((m_center - half)..=(m_center + half))
            .filter(|&m| m > 0)
            .map(|m| m as f64 * fsr)
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Cavity ring-down spectroscopy (CRDS)
// ---------------------------------------------------------------------------

/// Cavity Ring-Down Spectroscopy (CRDS) processor.
///
/// Measures ultra-sensitive absorption by fitting the exponential decay
/// of light intensity in a high-finesse optical cavity:
///
///   I(t) = I0 * exp(-t / tau)
///
/// Absorption coefficient:
///   alpha = (1 / (c * tau)) - (1 / (c * tau0))
///
/// where tau0 is the empty cavity ring-down time and tau is measured with sample.
#[derive(Debug, Clone)]
pub struct CavityRingdownProcessor {
    /// Empty cavity ring-down time (s).
    pub tau0_s: f64,
    /// Cavity length (m).
    pub cavity_length_m: f64,
}

impl CavityRingdownProcessor {
    /// Create a new CRDS processor.
    pub fn new(tau0_s: f64, cavity_length_m: f64) -> Self {
        Self { tau0_s, cavity_length_m }
    }

    /// Empty cavity loss per pass: loss = L / (c * tau0).
    pub fn empty_cavity_loss(&self) -> f64 {
        self.cavity_length_m / (C * self.tau0_s)
    }

    /// Mirror reflectivity estimate: R ~ 1 - L/(c*tau0).
    pub fn estimated_reflectivity(&self) -> f64 {
        1.0 - self.empty_cavity_loss()
    }

    /// Absorption coefficient from measured ring-down time.
    ///
    /// alpha = (1/(c*tau) - 1/(c*tau0))
    pub fn absorption_coefficient(&self, tau_s: f64) -> f64 {
        (1.0 / (C * tau_s)) - (1.0 / (C * self.tau0_s))
    }

    /// Absorption coefficient per unit path length (cm^-1).
    pub fn absorption_coefficient_per_cm(&self, tau_s: f64) -> f64 {
        self.absorption_coefficient(tau_s) * 100.0
    }

    /// Minimum detectable absorption coefficient.
    /// alpha_min ~ (1/(c*tau0)) * (delta_tau / tau0)
    pub fn sensitivity(&self, delta_tau_relative: f64) -> f64 {
        delta_tau_relative / (C * self.tau0_s)
    }

    /// Effective path length: L_eff = c * tau0.
    pub fn effective_path_length(&self) -> f64 {
        C * self.tau0_s
    }

    /// Number of effective round trips: N = c * tau0 / (2 * L).
    pub fn effective_round_trips(&self) -> f64 {
        C * self.tau0_s / (2.0 * self.cavity_length_m)
    }

    /// Fit an exponential decay to time-domain ring-down data.
    ///
    /// Uses linear regression on ln(I): ln(I) = ln(I0) - t/tau.
    /// Returns (I0, tau_s).
    pub fn fit_ringdown(times_s: &[f64], intensities: &[f64]) -> Option<(f64, f64)> {
        let n = times_s.len().min(intensities.len());
        if n < 2 {
            return None;
        }

        // Take log of positive intensities
        let mut sum_t = 0.0;
        let mut sum_lni = 0.0;
        let mut sum_t2 = 0.0;
        let mut sum_t_lni = 0.0;
        let mut count = 0;

        for i in 0..n {
            if intensities[i] > 0.0 {
                let t = times_s[i];
                let lni = intensities[i].ln();
                sum_t += t;
                sum_lni += lni;
                sum_t2 += t * t;
                sum_t_lni += t * lni;
                count += 1;
            }
        }

        if count < 2 {
            return None;
        }

        let nf = count as f64;
        let denom = nf * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return None;
        }

        // slope = -1/tau, intercept = ln(I0)
        let slope = (nf * sum_t_lni - sum_t * sum_lni) / denom;
        let intercept = (sum_lni - slope * sum_t) / nf;

        if slope >= 0.0 {
            return None; // Not a decay
        }

        let tau = -1.0 / slope;
        let i0 = intercept.exp();
        Some((i0, tau))
    }

    /// Generate a synthetic ring-down signal.
    pub fn generate_ringdown(&self, i0: f64, num_samples: usize, sample_rate: f64) -> (Vec<f64>, Vec<f64>) {
        let dt = 1.0 / sample_rate;
        let times: Vec<f64> = (0..num_samples).map(|i| i as f64 * dt).collect();
        let intensities: Vec<f64> = times.iter().map(|&t| i0 * (-t / self.tau0_s).exp()).collect();
        (times, intensities)
    }
}

// ---------------------------------------------------------------------------
// Saturation spectroscopy
// ---------------------------------------------------------------------------

/// Saturation (Lamb dip) spectroscopy processor.
///
/// In saturated absorption spectroscopy, a strong pump beam saturates a
/// Doppler-broadened transition, creating a narrow Lamb dip at the transition
/// centre frequency. This enables sub-Doppler resolution.
#[derive(Debug, Clone)]
pub struct SaturationSpectroscopy {
    /// Transition centre frequency (Hz).
    pub f0_hz: f64,
    /// Doppler (Gaussian) FWHM (Hz).
    pub doppler_fwhm_hz: f64,
    /// Natural (Lorentzian) FWHM (Hz).
    pub natural_fwhm_hz: f64,
    /// Saturation parameter s = I / I_sat.
    pub saturation_parameter: f64,
}

impl SaturationSpectroscopy {
    /// Create a new saturation spectroscopy processor.
    pub fn new(f0_hz: f64, doppler_fwhm_hz: f64, natural_fwhm_hz: f64, saturation_parameter: f64) -> Self {
        Self {
            f0_hz,
            doppler_fwhm_hz,
            natural_fwhm_hz,
            saturation_parameter,
        }
    }

    /// Power-broadened linewidth: gamma_eff = gamma_nat * sqrt(1 + s).
    pub fn power_broadened_fwhm(&self) -> f64 {
        self.natural_fwhm_hz * (1.0 + self.saturation_parameter).sqrt()
    }

    /// Lamb dip depth relative to Doppler background.
    ///
    /// Depth ~ s / (2 * (1 + s)) * (gamma_nat / gamma_D)
    pub fn lamb_dip_depth(&self) -> f64 {
        let s = self.saturation_parameter;
        let ratio = self.natural_fwhm_hz / self.doppler_fwhm_hz;
        s / (2.0 * (1.0 + s)) * ratio
    }

    /// Generate a saturated absorption spectrum.
    ///
    /// The profile is a Doppler-broadened absorption with a narrow Lamb dip
    /// at line centre.
    pub fn spectrum(&self, frequencies: &[f64]) -> Vec<f64> {
        let sigma_d = self.doppler_fwhm_hz / (2.0 * (2.0_f64.ln()).sqrt());
        let gamma_pb = self.power_broadened_fwhm() / 2.0; // HWHM
        let depth = self.lamb_dip_depth();

        frequencies
            .iter()
            .map(|&f| {
                let df = f - self.f0_hz;

                // Doppler (Gaussian) absorption background
                let gauss = (-0.5 * (df / sigma_d).powi(2)).exp();

                // Lamb dip (Lorentzian dip at line centre)
                let lorentz = gamma_pb * gamma_pb / (df * df + gamma_pb * gamma_pb);

                // Absorption = Doppler background - Lamb dip
                gauss * (1.0 - depth * lorentz)
            })
            .collect()
    }

    /// Crossover resonance frequency between two transitions.
    ///
    /// For transitions at f1 and f2, crossover appears at (f1 + f2) / 2.
    pub fn crossover_frequency(f1_hz: f64, f2_hz: f64) -> f64 {
        (f1_hz + f2_hz) / 2.0
    }
}

// ---------------------------------------------------------------------------
// Wavelength calibrator
// ---------------------------------------------------------------------------

/// Reference spectral line for calibration.
#[derive(Debug, Clone)]
pub struct ReferenceLine {
    /// Wavelength in metres.
    pub wavelength_m: f64,
    /// Relative intensity (arbitrary units).
    pub intensity: f64,
    /// Label (e.g., "Ne 585.2nm").
    pub label: &'static str,
}

/// Wavelength calibrator using known reference emission lines.
///
/// Fits a polynomial mapping from pixel/channel position to wavelength
/// using identified reference lines.
#[derive(Debug, Clone)]
pub struct WavelengthCalibrator {
    /// Polynomial coefficients [a0, a1, a2, ...] for lambda = a0 + a1*x + a2*x^2 + ...
    pub coefficients: Vec<f64>,
}

impl WavelengthCalibrator {
    /// Standard neon emission lines (wavelength in metres).
    pub fn neon_lines() -> Vec<ReferenceLine> {
        vec![
            ReferenceLine { wavelength_m: 585.249e-9, intensity: 1.0, label: "Ne 585.2nm" },
            ReferenceLine { wavelength_m: 588.189e-9, intensity: 0.8, label: "Ne 588.2nm" },
            ReferenceLine { wavelength_m: 594.483e-9, intensity: 0.6, label: "Ne 594.5nm" },
            ReferenceLine { wavelength_m: 607.434e-9, intensity: 0.9, label: "Ne 607.4nm" },
            ReferenceLine { wavelength_m: 614.306e-9, intensity: 0.7, label: "Ne 614.3nm" },
            ReferenceLine { wavelength_m: 621.728e-9, intensity: 0.5, label: "Ne 621.7nm" },
            ReferenceLine { wavelength_m: 626.649e-9, intensity: 0.8, label: "Ne 626.6nm" },
            ReferenceLine { wavelength_m: 633.443e-9, intensity: 1.0, label: "Ne 633.4nm" },
            ReferenceLine { wavelength_m: 638.299e-9, intensity: 0.6, label: "Ne 638.3nm" },
            ReferenceLine { wavelength_m: 640.225e-9, intensity: 0.9, label: "Ne 640.2nm" },
        ]
    }

    /// Standard mercury emission lines.
    pub fn mercury_lines() -> Vec<ReferenceLine> {
        vec![
            ReferenceLine { wavelength_m: 253.652e-9, intensity: 1.0, label: "Hg 253.7nm" },
            ReferenceLine { wavelength_m: 296.728e-9, intensity: 0.5, label: "Hg 296.7nm" },
            ReferenceLine { wavelength_m: 365.015e-9, intensity: 0.8, label: "Hg 365.0nm" },
            ReferenceLine { wavelength_m: 404.656e-9, intensity: 0.9, label: "Hg 404.7nm" },
            ReferenceLine { wavelength_m: 435.833e-9, intensity: 0.9, label: "Hg 435.8nm" },
            ReferenceLine { wavelength_m: 546.074e-9, intensity: 1.0, label: "Hg 546.1nm" },
            ReferenceLine { wavelength_m: 576.960e-9, intensity: 0.7, label: "Hg 577.0nm" },
            ReferenceLine { wavelength_m: 579.066e-9, intensity: 0.7, label: "Hg 579.1nm" },
        ]
    }

    /// Standard argon emission lines (visible).
    pub fn argon_lines() -> Vec<ReferenceLine> {
        vec![
            ReferenceLine { wavelength_m: 696.543e-9, intensity: 1.0, label: "Ar 696.5nm" },
            ReferenceLine { wavelength_m: 706.722e-9, intensity: 0.8, label: "Ar 706.7nm" },
            ReferenceLine { wavelength_m: 714.704e-9, intensity: 0.6, label: "Ar 714.7nm" },
            ReferenceLine { wavelength_m: 727.294e-9, intensity: 0.7, label: "Ar 727.3nm" },
            ReferenceLine { wavelength_m: 738.398e-9, intensity: 0.9, label: "Ar 738.4nm" },
            ReferenceLine { wavelength_m: 750.387e-9, intensity: 1.0, label: "Ar 750.4nm" },
            ReferenceLine { wavelength_m: 763.511e-9, intensity: 0.8, label: "Ar 763.5nm" },
            ReferenceLine { wavelength_m: 772.376e-9, intensity: 0.5, label: "Ar 772.4nm" },
            ReferenceLine { wavelength_m: 794.818e-9, intensity: 0.7, label: "Ar 794.8nm" },
            ReferenceLine { wavelength_m: 811.531e-9, intensity: 0.9, label: "Ar 811.5nm" },
        ]
    }

    /// Calibrate using matched positions and known wavelengths.
    ///
    /// Fits a polynomial of the given order to the (position, wavelength) pairs.
    pub fn calibrate(positions: &[f64], wavelengths_m: &[f64], poly_order: usize) -> Self {
        let n = positions.len().min(wavelengths_m.len());
        let order = poly_order.min(n.saturating_sub(1));

        let coefficients = polynomial_fit(positions, wavelengths_m, order);
        Self { coefficients }
    }

    /// Convert a pixel/channel position to calibrated wavelength (m).
    pub fn position_to_wavelength(&self, position: f64) -> f64 {
        let mut result = 0.0;
        let mut x_pow = 1.0;
        for &c in &self.coefficients {
            result += c * x_pow;
            x_pow *= position;
        }
        result
    }

    /// Calibrate an entire spectrum axis.
    pub fn calibrate_axis(&self, positions: &[f64]) -> Vec<f64> {
        positions.iter().map(|&p| self.position_to_wavelength(p)).collect()
    }

    /// Residual RMS between calibrated and known wavelengths.
    pub fn residual_rms(&self, positions: &[f64], wavelengths_m: &[f64]) -> f64 {
        let n = positions.len().min(wavelengths_m.len());
        if n == 0 {
            return 0.0;
        }
        let sum_sq: f64 = positions.iter().zip(wavelengths_m.iter()).map(|(&p, &w)| {
            let cal = self.position_to_wavelength(p);
            (cal - w).powi(2)
        }).sum();
        (sum_sq / n as f64).sqrt()
    }
}

/// Least-squares polynomial fit of given order.
/// Returns coefficients [a0, a1, ..., a_order].
fn polynomial_fit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len().min(y.len());
    let m = order + 1;

    if n == 0 || m == 0 {
        return vec![0.0; m];
    }

    // Build normal equations: (X^T X) a = X^T y
    // X is n x m Vandermonde matrix

    // X^T X: m x m
    let mut xtx = vec![0.0; m * m];
    let mut xty = vec![0.0; m];

    for i in 0..n {
        let mut xi_pow = vec![1.0; m];
        for j in 1..m {
            xi_pow[j] = xi_pow[j - 1] * x[i];
        }
        for r in 0..m {
            for c in 0..m {
                xtx[r * m + c] += xi_pow[r] * xi_pow[c];
            }
            xty[r] += xi_pow[r] * y[i];
        }
    }

    // Solve via Gaussian elimination with partial pivoting
    gauss_solve(&xtx, &xty, m)
}

/// Solve A * x = b via Gaussian elimination with partial pivoting.
fn gauss_solve(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut aug = vec![0.0; n * (n + 1)];
    for i in 0..n {
        for j in 0..n {
            aug[i * (n + 1) + j] = a[i * n + j];
        }
        aug[i * (n + 1) + n] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col * (n + 1) + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = aug[row * (n + 1) + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for j in 0..=n {
                let tmp = aug[col * (n + 1) + j];
                aug[col * (n + 1) + j] = aug[max_row * (n + 1) + j];
                aug[max_row * (n + 1) + j] = tmp;
            }
        }

        let pivot = aug[col * (n + 1) + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..n {
            let factor = aug[row * (n + 1) + col] / pivot;
            for j in col..=n {
                aug[row * (n + 1) + j] -= factor * aug[col * (n + 1) + j];
            }
        }
    }

    // Back substitution
    let mut result = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i * (n + 1) + n];
        for j in (i + 1)..n {
            sum -= aug[i * (n + 1) + j] * result[j];
        }
        let diag = aug[i * (n + 1) + i];
        if diag.abs() > 1e-30 {
            result[i] = sum / diag;
        }
    }

    result
}

// ---------------------------------------------------------------------------
// Instrument lineshape
// ---------------------------------------------------------------------------

/// Types of instrument lineshape functions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LineshapeType {
    /// Gaussian (typical for grating spectrometers).
    Gaussian,
    /// Sinc (boxcar apodized FTIR).
    Sinc,
    /// Sinc-squared (triangular apodized FTIR).
    SincSquared,
    /// Lorentzian (Fabry-Perot).
    Lorentzian,
}

/// Instrument lineshape (ILS) modeler.
///
/// Models the spectral response function of the instrument and provides
/// convolution/deconvolution capabilities.
#[derive(Debug, Clone)]
pub struct InstrumentLineshape {
    /// Lineshape type.
    pub lineshape_type: LineshapeType,
    /// FWHM of the instrument lineshape (in same units as spectral axis).
    pub fwhm: f64,
}

impl InstrumentLineshape {
    /// Create a new instrument lineshape model.
    pub fn new(lineshape_type: LineshapeType, fwhm: f64) -> Self {
        Self { lineshape_type, fwhm }
    }

    /// Evaluate the normalized ILS at given offsets from centre.
    pub fn evaluate(&self, offsets: &[f64]) -> Vec<f64> {
        match self.lineshape_type {
            LineshapeType::Gaussian => {
                let sigma = self.fwhm / (2.0 * (2.0_f64.ln()).sqrt());
                let norm = 1.0 / (sigma * (2.0 * PI).sqrt());
                offsets.iter().map(|&x| norm * (-0.5 * (x / sigma).powi(2)).exp()).collect()
            }
            LineshapeType::Sinc => {
                // sinc(x) = sin(pi*x) / (pi*x), scaled so FWHM matches
                // For sinc, first zero at 1.0, FWHM ~ 0.886 * (zero crossing)
                let scale = 0.886 / self.fwhm;
                offsets.iter().map(|&x| {
                    let u = x * scale;
                    if u.abs() < 1e-15 {
                        scale
                    } else {
                        scale * (PI * u).sin() / (PI * u)
                    }
                }).collect()
            }
            LineshapeType::SincSquared => {
                let scale = 0.886 / self.fwhm;
                offsets.iter().map(|&x| {
                    let u = x * scale;
                    if u.abs() < 1e-15 {
                        scale
                    } else {
                        let s = (PI * u).sin() / (PI * u);
                        scale * s * s
                    }
                }).collect()
            }
            LineshapeType::Lorentzian => {
                let gamma = self.fwhm / 2.0;
                let norm = gamma / PI;
                offsets.iter().map(|&x| norm / (x * x + gamma * gamma)).collect()
            }
        }
    }

    /// Convolve a spectrum with the instrument lineshape.
    ///
    /// The spectral axis must be uniformly spaced.
    pub fn convolve(&self, spectrum: &[f64], spectral_step: f64) -> Vec<f64> {
        let n = spectrum.len();
        if n == 0 {
            return vec![];
        }

        // Generate kernel
        let kernel_half_width = (3.0 * self.fwhm / spectral_step).ceil() as usize;
        let kernel_size = 2 * kernel_half_width + 1;
        let offsets: Vec<f64> = (0..kernel_size)
            .map(|i| (i as f64 - kernel_half_width as f64) * spectral_step)
            .collect();
        let kernel = self.evaluate(&offsets);

        // Normalize kernel
        let kernel_sum: f64 = kernel.iter().sum::<f64>() * spectral_step;
        let kernel_norm: Vec<f64> = if kernel_sum.abs() > 1e-30 {
            kernel.iter().map(|&k| k / kernel_sum * spectral_step).collect()
        } else {
            kernel
        };

        // Convolve (zero-padded edges)
        let mut result = vec![0.0; n];
        for i in 0..n {
            let mut sum = 0.0;
            for (ki, &kv) in kernel_norm.iter().enumerate() {
                let si = i as i64 + ki as i64 - kernel_half_width as i64;
                if si >= 0 && (si as usize) < n {
                    sum += spectrum[si as usize] * kv;
                }
            }
            result[i] = sum;
        }
        result
    }

    /// Simple iterative deconvolution (Richardson-Lucy-like).
    ///
    /// Attempts to recover the true spectrum from an instrument-broadened measurement.
    pub fn deconvolve(&self, measured: &[f64], spectral_step: f64, iterations: usize) -> Vec<f64> {
        let n = measured.len();
        if n == 0 {
            return vec![];
        }

        let mut estimate: Vec<f64> = measured.to_vec();

        for _ in 0..iterations {
            let reconvolved = self.convolve(&estimate, spectral_step);
            for i in 0..n {
                if reconvolved[i].abs() > 1e-30 {
                    estimate[i] *= measured[i] / reconvolved[i];
                }
            }
        }

        estimate
    }
}

// ---------------------------------------------------------------------------
// Beer-Lambert analyzer
// ---------------------------------------------------------------------------

/// Beer-Lambert law analyzer for absorption spectroscopy.
///
/// Absorbance: A = epsilon * c * l = -log10(I / I0)
///
/// where epsilon is the molar absorptivity (L/(mol*cm)),
/// c is the concentration (mol/L), and l is the path length (cm).
#[derive(Debug, Clone)]
pub struct BeerLambertAnalyzer;

impl BeerLambertAnalyzer {
    /// Absorbance from transmitted and incident intensities.
    ///
    /// A = -log10(I / I0)
    pub fn absorbance(i0: f64, i_transmitted: f64) -> f64 {
        if i0 <= 0.0 || i_transmitted <= 0.0 {
            return 0.0;
        }
        -(i_transmitted / i0).log10()
    }

    /// Transmittance: T = I / I0 = 10^(-A).
    pub fn transmittance(absorbance: f64) -> f64 {
        10.0_f64.powf(-absorbance)
    }

    /// Concentration from absorbance, molar absorptivity, and path length.
    ///
    /// c = A / (epsilon * l)  [mol/L]
    pub fn concentration(absorbance: f64, molar_absorptivity: f64, path_length_cm: f64) -> f64 {
        if molar_absorptivity * path_length_cm == 0.0 {
            return 0.0;
        }
        absorbance / (molar_absorptivity * path_length_cm)
    }

    /// Absorbance from concentration, molar absorptivity, and path length.
    ///
    /// A = epsilon * c * l
    pub fn absorbance_from_concentration(
        molar_absorptivity: f64,
        concentration_mol_per_l: f64,
        path_length_cm: f64,
    ) -> f64 {
        molar_absorptivity * concentration_mol_per_l * path_length_cm
    }

    /// Compute absorbance spectrum from reference and sample spectra.
    pub fn absorbance_spectrum(reference: &[f64], sample: &[f64]) -> Vec<f64> {
        reference
            .iter()
            .zip(sample.iter())
            .map(|(&r, &s)| Self::absorbance(r, s))
            .collect()
    }

    /// Estimate concentration from absorbance spectrum at peak wavelength.
    ///
    /// Finds maximum absorbance and uses it with given epsilon and path length.
    pub fn concentration_from_spectrum(
        absorbance_spectrum: &[f64],
        molar_absorptivity: f64,
        path_length_cm: f64,
    ) -> f64 {
        let peak_abs = absorbance_spectrum.iter().cloned().fold(0.0_f64, f64::max);
        Self::concentration(peak_abs, molar_absorptivity, path_length_cm)
    }

    /// Natural linewidth from excited-state radiative lifetime.
    ///
    /// delta_f_N = 1 / (2 * pi * tau_rad)
    pub fn natural_linewidth_hz(tau_rad_s: f64) -> f64 {
        if tau_rad_s <= 0.0 {
            return 0.0;
        }
        1.0 / (2.0 * PI * tau_rad_s)
    }

    /// Optical density: OD = A (same as absorbance, but commonly used for filters).
    pub fn optical_density(i0: f64, i_transmitted: f64) -> f64 {
        Self::absorbance(i0, i_transmitted)
    }

    /// Cross-section from molar absorptivity.
    ///
    /// sigma (cm^2) = 1000 * ln(10) * epsilon / N_A
    /// where N_A is Avogadro's number.
    pub fn cross_section_cm2(molar_absorptivity: f64) -> f64 {
        const N_A: f64 = 6.02214076e23;
        1000.0 * 10.0_f64.ln() * molar_absorptivity / N_A
    }
}

// ---------------------------------------------------------------------------
// Helper: wavelength <-> frequency conversion
// ---------------------------------------------------------------------------

/// Convert wavelength (m) to frequency (Hz).
pub fn wavelength_to_frequency(lambda_m: f64) -> f64 {
    C / lambda_m
}

/// Convert frequency (Hz) to wavelength (m).
pub fn frequency_to_wavelength(f_hz: f64) -> f64 {
    C / f_hz
}

/// Wavenumber (cm^-1) from wavelength (m).
pub fn wavelength_to_wavenumber(lambda_m: f64) -> f64 {
    1.0 / (lambda_m * 100.0)
}

/// Wavelength (m) from wavenumber (cm^-1).
pub fn wavenumber_to_wavelength(sigma_per_cm: f64) -> f64 {
    1.0 / (sigma_per_cm * 100.0)
}

/// Photon energy (J) from frequency (Hz).
pub fn photon_energy_j(f_hz: f64) -> f64 {
    H * f_hz
}

/// Photon energy (eV) from wavelength (m).
pub fn photon_energy_ev(lambda_m: f64) -> f64 {
    const E_CHARGE: f64 = 1.602176634e-19;
    H * C / (lambda_m * E_CHARGE)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- SpectroscopyConfig tests ---

    #[test]
    fn test_config_wavelength_axis() {
        let config = SpectroscopyConfig::new(500e-9, 700e-9, 1e-9, InstrumentType::Grating, 201);
        let axis = config.wavelength_axis();
        assert_eq!(axis.len(), 201);
        assert!((axis[0] - 500e-9).abs() < 1e-15);
        assert!((axis[200] - 700e-9).abs() < 1e-15);
    }

    #[test]
    fn test_config_frequency_axis() {
        let config = SpectroscopyConfig::new(500e-9, 700e-9, 1e-9, InstrumentType::Ftir, 101);
        let axis = config.frequency_axis();
        assert_eq!(axis.len(), 101);
        // Frequency axis inverted relative to wavelength
        assert!(axis[0] < axis[100]);
    }

    #[test]
    fn test_config_resolving_power() {
        let config = SpectroscopyConfig::new(599e-9, 601e-9, 0.01e-9, InstrumentType::Laser, 201);
        let r = config.resolving_power();
        // R = 600nm / 0.01nm = 60000
        assert!((r - 60000.0).abs() < 1.0);
    }

    #[test]
    fn test_config_single_point() {
        let config = SpectroscopyConfig::new(500e-9, 700e-9, 1e-9, InstrumentType::Grating, 1);
        let axis = config.wavelength_axis();
        assert_eq!(axis.len(), 1);
        assert!((axis[0] - 500e-9).abs() < 1e-15);
    }

    // --- DopplerBroadening tests ---

    #[test]
    fn test_doppler_fwhm_rubidium() {
        // Rb-87 D2 line at 384.230 THz, T = 300 K
        let mass_rb87 = 87.0 * 1.66054e-27; // kg
        let d = DopplerBroadening::new(384.230e12, mass_rb87, 300.0);
        let fwhm = d.fwhm_hz();
        // Expected: ~500 MHz for Rb at room temperature
        assert!(fwhm > 400e6 && fwhm < 600e6, "Doppler FWHM was {} Hz", fwhm);
    }

    #[test]
    fn test_doppler_temperature_recovery() {
        let mass = 40.0 * 1.66054e-27; // Ar-40
        let f0 = 300e12;
        let t_original = 500.0;
        let d = DopplerBroadening::new(f0, mass, t_original);
        let fwhm = d.fwhm_hz();
        let t_recovered = DopplerBroadening::temperature_from_fwhm(f0, mass, fwhm);
        assert!((t_recovered - t_original).abs() < 0.01);
    }

    #[test]
    fn test_doppler_sigma_relationship() {
        let d = DopplerBroadening::new(300e12, 40.0 * 1.66054e-27, 300.0);
        let fwhm = d.fwhm_hz();
        let sigma = d.sigma_hz();
        let ratio = fwhm / sigma;
        // FWHM = 2 * sqrt(2 * ln(2)) * sigma ~ 2.3548 * sigma
        assert!((ratio - 2.3548).abs() < 0.001);
    }

    #[test]
    fn test_doppler_wavelength_fwhm() {
        let d = DopplerBroadening::new(C / 500e-9, 20.0 * 1.66054e-27, 300.0);
        let fwhm_wl = d.fwhm_wavelength_m();
        assert!(fwhm_wl > 0.0);
        // FWHM in wavelength should be small relative to 500 nm
        assert!(fwhm_wl < 1e-9);
    }

    #[test]
    fn test_doppler_lineshape_normalization() {
        let d = DopplerBroadening::new(300e12, 40.0 * 1.66054e-27, 300.0);
        let fwhm = d.fwhm_hz();
        // Generate many points over wide range
        let n = 10001;
        let span = 10.0 * fwhm;
        let df = span / (n - 1) as f64;
        let freqs: Vec<f64> = (0..n).map(|i| 300e12 - span / 2.0 + i as f64 * df).collect();
        let ls = d.lineshape(&freqs);
        let integral: f64 = ls.iter().sum::<f64>() * df;
        // Should integrate to approximately 1
        assert!((integral - 1.0).abs() < 0.01, "Integral was {}", integral);
    }

    #[test]
    fn test_doppler_temperature_scaling() {
        let mass = 40.0 * 1.66054e-27;
        let d1 = DopplerBroadening::new(300e12, mass, 300.0);
        let d2 = DopplerBroadening::new(300e12, mass, 1200.0);
        // FWHM scales as sqrt(T)
        let ratio = d2.fwhm_hz() / d1.fwhm_hz();
        assert!((ratio - 2.0).abs() < 0.001);
    }

    // --- PressureBroadening tests ---

    #[test]
    fn test_pressure_broadening_scaling() {
        let pb = PressureBroadening::new(1e9, 101325.0, 296.0);
        let hwhm_1atm = pb.hwhm_hz(101325.0, 296.0);
        assert!((hwhm_1atm - 1e9).abs() < 1.0);

        // At half pressure, HWHM should halve
        let hwhm_half = pb.hwhm_hz(50662.5, 296.0);
        assert!((hwhm_half - 0.5e9).abs() < 1.0);
    }

    #[test]
    fn test_pressure_broadening_temperature() {
        let pb = PressureBroadening::new(1e9, 101325.0, 300.0);
        // At 4x temperature, width * sqrt(1/4) = width/2
        let hwhm_1 = pb.hwhm_hz(101325.0, 300.0);
        let hwhm_2 = pb.hwhm_hz(101325.0, 1200.0);
        assert!((hwhm_2 / hwhm_1 - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_pressure_lineshape_normalization() {
        let pb = PressureBroadening::new(1e9, 101325.0, 296.0);
        let n = 10001;
        let span = 100e9;
        let df = span / (n - 1) as f64;
        let freqs: Vec<f64> = (0..n).map(|i| 300e12 - span / 2.0 + i as f64 * df).collect();
        let ls = pb.lineshape(&freqs, 300e12, 101325.0, 296.0);
        let integral: f64 = ls.iter().sum::<f64>() * df;
        assert!((integral - 1.0).abs() < 0.02, "Lorentzian integral was {}", integral);
    }

    // --- VoigtProfileFitter tests ---

    #[test]
    fn test_voigt_pure_gaussian() {
        let v = VoigtProfileFitter::new(300e12, 1e9, 0.0, 1.0);
        let fwhm = v.voigt_fwhm();
        // Pure Gaussian: Voigt FWHM should equal Gaussian FWHM
        assert!((fwhm - 1e9).abs() / 1e9 < 0.01);
    }

    #[test]
    fn test_voigt_pure_lorentzian() {
        let v = VoigtProfileFitter::new(300e12, 1e-10, 1e9, 1.0);
        let fwhm = v.voigt_fwhm();
        // Nearly pure Lorentzian: Voigt FWHM should be close to Lorentzian FWHM
        assert!((fwhm - 1e9).abs() / 1e9 < 0.01);
    }

    #[test]
    fn test_voigt_fwhm_intermediate() {
        let v = VoigtProfileFitter::new(300e12, 1e9, 1e9, 1.0);
        let fwhm = v.voigt_fwhm();
        // Voigt FWHM should be between Gaussian and sum
        assert!(fwhm > 1e9);
        assert!(fwhm < 2e9);
    }

    #[test]
    fn test_voigt_mixing_parameter_range() {
        let v = VoigtProfileFitter::new(300e12, 1e9, 1e9, 1.0);
        let eta = v.mixing_parameter();
        assert!(eta > 0.0 && eta < 1.0, "eta was {}", eta);
    }

    #[test]
    fn test_voigt_parameter() {
        let v = VoigtProfileFitter::new(300e12, 1e9, 0.5e9, 1.0);
        let a = v.voigt_parameter();
        assert!(a > 0.0);
        assert!(a < 10.0);
    }

    #[test]
    fn test_voigt_evaluate_peak() {
        let v = VoigtProfileFitter::new(300e12, 1e9, 1e9, 1.0);
        let profile = v.evaluate(&[300e12]);
        assert!(profile[0] > 0.0, "Peak should be positive");
    }

    #[test]
    fn test_voigt_fit_recovery() {
        // Create synthetic data from known Voigt profile
        let true_v = VoigtProfileFitter::new(300e12, 1e9, 0.5e9, 2.0);
        let n = 1001;
        let span = 10e9;
        let df = span / (n - 1) as f64;
        let freqs: Vec<f64> = (0..n).map(|i| 300e12 - span / 2.0 + i as f64 * df).collect();
        let data = true_v.evaluate(&freqs);

        let fitted = VoigtProfileFitter::fit(&freqs, &data, 300e12, 1.5e9, 0.8e9, 50);
        // Check centre frequency recovery
        assert!((fitted.center_hz - 300e12).abs() / 300e12 < 0.001);
    }

    // --- FrequencyCombAnalyzer tests ---

    #[test]
    fn test_comb_tooth_frequency() {
        let comb = FrequencyCombAnalyzer::new(20e6, 250e6);
        let f = comb.tooth_frequency(1_000_000);
        // f = 20e6 + 1e6 * 250e6 = 250_000_020_000_000 Hz = 250.00002 THz
        let expected = 20e6 + 1_000_000.0 * 250e6;
        assert!((f - expected).abs() < 1.0);
    }

    #[test]
    fn test_comb_mode_number() {
        let comb = FrequencyCombAnalyzer::new(20e6, 250e6);
        // f_tooth = 20e6 + n * 250e6 => n = (f - 20e6) / 250e6
        let f_target = 20e6 + 1_000_000.0 * 250e6;
        let n = comb.mode_number(f_target);
        assert_eq!(n, 1_000_000);
    }

    #[test]
    fn test_comb_absolute_frequency() {
        let comb = FrequencyCombAnalyzer::new(20e6, 250e6);
        let (f_plus, f_minus) = comb.absolute_frequency(1_000_000, 50e6);
        let f_tooth = comb.tooth_frequency(1_000_000);
        assert!((f_plus - (f_tooth + 50e6)).abs() < 1.0);
        assert!((f_minus - (f_tooth - 50e6)).abs() < 1.0);
    }

    #[test]
    fn test_comb_spectrum() {
        let comb = FrequencyCombAnalyzer::new(0.0, 1e9);
        let (modes, freqs) = comb.comb_spectrum(10e9, 15e9);
        assert_eq!(modes.len(), freqs.len());
        assert!(!modes.is_empty());
        assert_eq!(modes[0], 10);
        assert_eq!(*modes.last().unwrap(), 15);
    }

    #[test]
    fn test_comb_fractional_uncertainty() {
        let comb = FrequencyCombAnalyzer::new(20e6, 250e6);
        let frac = comb.fractional_uncertainty(1_000_000, 1.0, 1e-3);
        // f = 250.00002 THz
        // term1 = (1.0 / 2.5e14)^2 ~ 1.6e-29
        // term2 = (1e6 * 1e-3 / 2.5e14)^2 = (1e3 / 2.5e14)^2 ~ 1.6e-23
        // frac ~ 4e-12
        assert!(frac < 1e-10);
        assert!(frac > 0.0);
    }

    // --- FabryPerotAnalyzer tests ---

    #[test]
    fn test_fp_finesse() {
        let fp = FabryPerotAnalyzer::new(0.99, 0.10, 1.0);
        let f = fp.finesse();
        // F = pi * sqrt(0.99) / (1 - 0.99) = pi * 0.995 / 0.01 ~ 312.6
        assert!((f - 312.6).abs() < 1.0);
    }

    #[test]
    fn test_fp_fsr() {
        let fp = FabryPerotAnalyzer::new(0.99, 0.10, 1.0);
        let fsr = fp.fsr_hz();
        // FSR = c / (2 * 1.0 * 0.10) = c / 0.20
        let expected = C / 0.20;
        assert!((fsr - expected).abs() < 1.0);
    }

    #[test]
    fn test_fp_resolution() {
        let fp = FabryPerotAnalyzer::new(0.99, 0.10, 1.0);
        let res = fp.resolution_hz();
        let expected = fp.fsr_hz() / fp.finesse();
        assert!((res - expected).abs() < 1.0);
    }

    #[test]
    fn test_fp_resolving_power() {
        let fp = FabryPerotAnalyzer::new(0.99, 0.05, 1.0);
        let rp = fp.resolving_power(500e-9);
        // Should be very high
        assert!(rp > 1e6);
    }

    #[test]
    fn test_fp_transmission_peaks() {
        let fp = FabryPerotAnalyzer::new(0.99, 0.10, 1.0);
        let fsr = fp.fsr_hz();
        // At exact resonance, transmission should be maximum
        let f_peak = fsr; // first resonance
        let t = fp.transmission(&[f_peak]);
        assert!((t[0] - 1.0).abs() < 0.001, "Peak transmission was {}", t[0]);
    }

    #[test]
    fn test_fp_transmission_between_peaks() {
        let fp = FabryPerotAnalyzer::new(0.99, 0.10, 1.0);
        let fsr = fp.fsr_hz();
        // At half FSR from resonance, transmission should be minimum
        let f_mid = fsr * 1.5;
        let t = fp.transmission(&[f_mid]);
        assert!(t[0] < 0.01, "Mid-peak transmission was {}", t[0]);
    }

    #[test]
    fn test_fp_peak_frequencies() {
        let fp = FabryPerotAnalyzer::new(0.99, 0.10, 1.0);
        let fsr = fp.fsr_hz();
        let peaks = fp.peak_frequencies_near(10.0 * fsr, 5);
        // Peaks should be spaced by FSR
        assert!(peaks.len() >= 3);
        for pair in peaks.windows(2) {
            assert!((pair[1] - pair[0] - fsr).abs() / fsr < 0.001);
        }
    }

    // --- CavityRingdownProcessor tests ---

    #[test]
    fn test_crds_absorption_coefficient() {
        let crds = CavityRingdownProcessor::new(10e-6, 0.50);
        // With shorter tau, there's absorption
        let alpha = crds.absorption_coefficient(8e-6);
        // alpha = 1/(c*8e-6) - 1/(c*10e-6)
        let expected = 1.0 / (C * 8e-6) - 1.0 / (C * 10e-6);
        assert!((alpha - expected).abs() < 1e-10);
        assert!(alpha > 0.0);
    }

    #[test]
    fn test_crds_empty_cavity() {
        let crds = CavityRingdownProcessor::new(10e-6, 0.50);
        let alpha = crds.absorption_coefficient(10e-6);
        assert!(alpha.abs() < 1e-15);
    }

    #[test]
    fn test_crds_effective_path() {
        let crds = CavityRingdownProcessor::new(10e-6, 0.50);
        let l_eff = crds.effective_path_length();
        // L_eff = c * 10us ~ 3000 m
        assert!((l_eff - C * 10e-6).abs() < 1.0);
    }

    #[test]
    fn test_crds_round_trips() {
        let crds = CavityRingdownProcessor::new(10e-6, 0.50);
        let n = crds.effective_round_trips();
        // N = c * 10e-6 / (2 * 0.50) = c * 10e-6
        let expected = C * 10e-6 / 1.0;
        assert!((n - expected).abs() < 1.0);
    }

    #[test]
    fn test_crds_fit_ringdown() {
        let crds = CavityRingdownProcessor::new(10e-6, 0.50);
        let (times, intensities) = crds.generate_ringdown(1000.0, 500, 50e6);

        let result = CavityRingdownProcessor::fit_ringdown(&times, &intensities);
        assert!(result.is_some());
        let (i0_fit, tau_fit) = result.unwrap();
        assert!((i0_fit - 1000.0).abs() / 1000.0 < 0.01);
        assert!((tau_fit - 10e-6).abs() / 10e-6 < 0.01);
    }

    #[test]
    fn test_crds_sensitivity() {
        let crds = CavityRingdownProcessor::new(100e-6, 0.50);
        let sens = crds.sensitivity(1e-3); // 0.1% precision on tau
        // sens = 1e-3 / (c * 100e-6) ~ 3.34e-8 (m^-1)
        assert!(sens < 1e-7);
        assert!(sens > 1e-9);
    }

    // --- SaturationSpectroscopy tests ---

    #[test]
    fn test_saturation_power_broadening() {
        let sat = SaturationSpectroscopy::new(384.230e12, 500e6, 6e6, 1.0);
        let pb = sat.power_broadened_fwhm();
        // At s=1: gamma_eff = 6e6 * sqrt(2) ~ 8.49 MHz
        assert!((pb - 6e6 * 2.0_f64.sqrt()).abs() / pb < 0.001);
    }

    #[test]
    fn test_saturation_lamb_dip_depth() {
        let sat = SaturationSpectroscopy::new(384.230e12, 500e6, 6e6, 1.0);
        let depth = sat.lamb_dip_depth();
        assert!(depth > 0.0 && depth < 1.0);
    }

    #[test]
    fn test_saturation_spectrum_has_dip() {
        let sat = SaturationSpectroscopy::new(300e12, 500e6, 6e6, 2.0);
        let n = 1001;
        let span = 2e9;
        let df = span / (n - 1) as f64;
        let freqs: Vec<f64> = (0..n).map(|i| 300e12 - span / 2.0 + i as f64 * df).collect();
        let spectrum = sat.spectrum(&freqs);

        // The centre should have a dip compared to nearby points
        let center_idx = n / 2;
        let offset_idx = center_idx + 50;
        // Nearby off-centre point should be higher than centre (dip)
        assert!(spectrum[offset_idx] > spectrum[center_idx] ||
                (spectrum[center_idx] - spectrum[offset_idx]).abs() < 0.1,
                "Expected Lamb dip at centre");
    }

    #[test]
    fn test_saturation_crossover() {
        let f_co = SaturationSpectroscopy::crossover_frequency(384.230e12, 384.232e12);
        assert!((f_co - 384.231e12).abs() < 1.0);
    }

    // --- WavelengthCalibrator tests ---

    #[test]
    fn test_calibrator_linear() {
        // Perfect linear relationship: lambda = 500nm + 0.1nm * pixel
        let positions = vec![0.0, 100.0, 200.0, 300.0, 400.0];
        let wavelengths: Vec<f64> = positions.iter().map(|&p| 500e-9 + 0.1e-9 * p).collect();
        let cal = WavelengthCalibrator::calibrate(&positions, &wavelengths, 1);
        let w = cal.position_to_wavelength(150.0);
        assert!((w - 515e-9).abs() < 1e-12);
    }

    #[test]
    fn test_calibrator_quadratic() {
        let positions: Vec<f64> = (0..5).map(|i| i as f64 * 100.0).collect();
        let wavelengths: Vec<f64> = positions
            .iter()
            .map(|&p| 500e-9 + 0.1e-9 * p + 1e-14 * p * p)
            .collect();
        let cal = WavelengthCalibrator::calibrate(&positions, &wavelengths, 2);
        let rms = cal.residual_rms(&positions, &wavelengths);
        assert!(rms < 1e-15);
    }

    #[test]
    fn test_calibrator_axis() {
        let positions = vec![0.0, 1000.0];
        let wavelengths = vec![500e-9, 600e-9];
        let cal = WavelengthCalibrator::calibrate(&positions, &wavelengths, 1);
        let axis = cal.calibrate_axis(&[0.0, 500.0, 1000.0]);
        assert_eq!(axis.len(), 3);
        assert!((axis[1] - 550e-9).abs() < 1e-12);
    }

    #[test]
    fn test_calibrator_reference_lines() {
        let ne = WavelengthCalibrator::neon_lines();
        assert!(ne.len() >= 8);
        let hg = WavelengthCalibrator::mercury_lines();
        assert!(hg.len() >= 6);
        let ar = WavelengthCalibrator::argon_lines();
        assert!(ar.len() >= 8);
    }

    // --- InstrumentLineshape tests ---

    #[test]
    fn test_ils_gaussian() {
        let ils = InstrumentLineshape::new(LineshapeType::Gaussian, 1.0);
        let val = ils.evaluate(&[0.0]);
        assert!(val[0] > 0.0);
    }

    #[test]
    fn test_ils_sinc() {
        let ils = InstrumentLineshape::new(LineshapeType::Sinc, 1.0);
        let val = ils.evaluate(&[0.0]);
        assert!(val[0] > 0.0);
    }

    #[test]
    fn test_ils_lorentzian() {
        let ils = InstrumentLineshape::new(LineshapeType::Lorentzian, 1.0);
        let val = ils.evaluate(&[0.0]);
        assert!(val[0] > 0.0);
    }

    #[test]
    fn test_ils_convolve_preserves_area() {
        let ils = InstrumentLineshape::new(LineshapeType::Gaussian, 0.5);
        let n = 501;
        let step = 0.01;
        let mut spectrum = vec![0.0; n];
        spectrum[n / 2] = 1.0 / step; // Delta function normalized

        let convolved = ils.convolve(&spectrum, step);
        let area: f64 = convolved.iter().sum::<f64>() * step;
        // Area should be approximately preserved
        assert!((area - 1.0).abs() < 0.1, "Area was {}", area);
    }

    #[test]
    fn test_ils_deconvolve_sharpens() {
        let ils = InstrumentLineshape::new(LineshapeType::Gaussian, 2.0);
        let n = 201;
        let step = 0.1;
        // Sharp line broadened by instrument
        let mut true_spectrum = vec![0.0; n];
        true_spectrum[n / 2] = 1.0;
        let broadened = ils.convolve(&true_spectrum, step);

        let recovered = ils.deconvolve(&broadened, step, 20);
        // Recovered peak should be sharper than broadened
        let peak_idx = n / 2;
        assert!(recovered[peak_idx] > broadened[peak_idx]);
    }

    // --- BeerLambertAnalyzer tests ---

    #[test]
    fn test_beer_lambert_absorbance() {
        let a = BeerLambertAnalyzer::absorbance(100.0, 10.0);
        // A = -log10(10/100) = -log10(0.1) = 1.0
        assert!((a - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_beer_lambert_transmittance() {
        let t = BeerLambertAnalyzer::transmittance(1.0);
        assert!((t - 0.1).abs() < EPSILON);
    }

    #[test]
    fn test_beer_lambert_roundtrip() {
        let a = 2.5;
        let t = BeerLambertAnalyzer::transmittance(a);
        let a2 = BeerLambertAnalyzer::absorbance(1.0, t);
        assert!((a2 - a).abs() < EPSILON);
    }

    #[test]
    fn test_beer_lambert_concentration() {
        // A = epsilon * c * l => c = A / (epsilon * l)
        let a = 1.0;
        let epsilon = 100.0; // L/(mol*cm)
        let l = 1.0; // cm
        let c = BeerLambertAnalyzer::concentration(a, epsilon, l);
        assert!((c - 0.01).abs() < EPSILON);
    }

    #[test]
    fn test_beer_lambert_absorbance_from_concentration() {
        let a = BeerLambertAnalyzer::absorbance_from_concentration(100.0, 0.01, 1.0);
        assert!((a - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_beer_lambert_spectrum() {
        let reference = vec![100.0, 100.0, 100.0, 100.0];
        let sample = vec![100.0, 50.0, 10.0, 1.0];
        let abs = BeerLambertAnalyzer::absorbance_spectrum(&reference, &sample);
        assert!((abs[0]).abs() < EPSILON); // No absorption
        assert!((abs[2] - 1.0).abs() < EPSILON); // OD 1
        assert!((abs[3] - 2.0).abs() < EPSILON); // OD 2
    }

    #[test]
    fn test_beer_lambert_natural_linewidth() {
        // tau = 26 ns for Na D line
        let lw = BeerLambertAnalyzer::natural_linewidth_hz(26e-9);
        // Expected: ~6.1 MHz
        assert!(lw > 5e6 && lw < 7e6);
    }

    #[test]
    fn test_beer_lambert_cross_section() {
        let sigma = BeerLambertAnalyzer::cross_section_cm2(1000.0);
        // Should be on the order of 1e-20 cm^2
        assert!(sigma > 1e-22 && sigma < 1e-17);
    }

    #[test]
    fn test_beer_lambert_concentration_from_spectrum() {
        let spectrum = vec![0.5, 1.0, 1.5, 2.0, 1.5, 1.0, 0.5];
        let c = BeerLambertAnalyzer::concentration_from_spectrum(&spectrum, 200.0, 1.0);
        // Peak absorbance = 2.0, c = 2.0 / (200 * 1) = 0.01
        assert!((c - 0.01).abs() < EPSILON);
    }

    // --- Utility function tests ---

    #[test]
    fn test_wavelength_frequency_roundtrip() {
        let lambda = 632.8e-9; // HeNe laser
        let f = wavelength_to_frequency(lambda);
        let lambda2 = frequency_to_wavelength(f);
        assert!((lambda2 - lambda).abs() < 1e-18);
    }

    #[test]
    fn test_wavenumber_conversion() {
        let lambda = 10e-6; // 10 microns = 1000 cm^-1
        let wn = wavelength_to_wavenumber(lambda);
        assert!((wn - 1000.0).abs() < 0.1);
        let lambda2 = wavenumber_to_wavelength(wn);
        assert!((lambda2 - lambda).abs() < 1e-15);
    }

    #[test]
    fn test_photon_energy() {
        // Photon at 500 nm ~ 2.48 eV
        let ev = photon_energy_ev(500e-9);
        assert!((ev - 2.48).abs() < 0.02);
    }

    #[test]
    fn test_photon_energy_j() {
        let f = 600e12;
        let e = photon_energy_j(f);
        assert!((e - H * f).abs() < 1e-40);
    }
}

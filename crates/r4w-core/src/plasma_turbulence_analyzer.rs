//! # MHD Turbulence Spectrum Analyzer for Plasma Physics
//!
//! This module implements magnetohydrodynamic (MHD) turbulence spectrum analysis
//! for fusion reactor diagnostics and space plasma physics. Turbulence plays a
//! critical role in plasma confinement within tokamaks and stellarators, driving
//! anomalous energy transport that degrades performance.
//!
//! ## Physical Background
//!
//! MHD turbulence in plasmas differs fundamentally from hydrodynamic turbulence
//! due to the presence of a magnetic field, which introduces anisotropy and wave
//! phenomena (Alfven waves). The key turbulence models are:
//!
//! - **Kolmogorov (1941)**: Classical fluid cascade with spectral index -5/3,
//!   applicable when magnetic fields are weak (high plasma beta).
//! - **Iroshnikov-Kraichnan (IK)**: Weak MHD turbulence where Alfven wave
//!   interactions slow the cascade, yielding spectral index -3/2.
//! - **Goldreich-Sridhar (GS95)**: Critical balance theory for strong MHD
//!   turbulence, predicting -5/3 spectrum perpendicular to the field.
//! - **Bohm**: Empirical diffusion scaling D ~ kT/(eB), historically observed
//!   in early confinement experiments.
//! - **GyroBohm**: Improved diffusion scaling D ~ (rho_i/a) * D_Bohm, where
//!   rho_i is the ion gyroradius and a is the device minor radius.
//!
//! ## Usage
//!
//! ```rust
//! use r4w_core::plasma_turbulence_analyzer::*;
//!
//! let config = TurbulenceConfig {
//!     sample_rate_hz: 1.0e6,
//!     magnetic_field_tesla: 5.0,
//!     plasma_density_m3: 1.0e20,
//!     ion_mass_kg: PROTON_MASS,
//!     temperature_ev: 1000.0,
//!     turbulence_model: TurbulenceModel::GoldreichSridhar,
//! };
//!
//! let analyzer = PlasmaTurbulenceAnalyzer::new(config);
//! let v_a = PlasmaTurbulenceAnalyzer::alfven_speed(5.0, 1.0e20, PROTON_MASS);
//! assert!((v_a - 1.09e7).abs() / v_a < 0.05); // ~10^7 m/s for tokamak conditions
//! ```

use std::f64::consts::PI;

// ──────────────────────────────────────────────────────────────────────────────
// Physical Constants (SI units)
// ──────────────────────────────────────────────────────────────────────────────

/// Permeability of free space (H/m)
pub const MU_0: f64 = 4.0 * PI * 1.0e-7; // 1.2566370614...e-6

/// Permittivity of free space (F/m)
pub const EPSILON_0: f64 = 8.854_187_817e-12;

/// Electron rest mass (kg)
pub const ELECTRON_MASS: f64 = 9.109_383_56e-31;

/// Proton rest mass (kg)
pub const PROTON_MASS: f64 = 1.672_621_898e-27;

/// Elementary charge (C)
pub const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;

/// Boltzmann constant (J/K)
pub const BOLTZMANN_K: f64 = 1.380_649e-23;

// ──────────────────────────────────────────────────────────────────────────────
// Turbulence Model Enum
// ──────────────────────────────────────────────────────────────────────────────

/// MHD turbulence cascade models.
///
/// Each model predicts a different power-law spectral index for the inertial
/// range of the turbulence energy spectrum E(k) ~ k^alpha.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TurbulenceModel {
    /// Kolmogorov (1941) isotropic fluid turbulence.
    /// Spectral index: -5/3. Applies when magnetic effects are subdominant.
    Kolmogorov,

    /// Iroshnikov (1964) - Kraichnan (1965) weak MHD turbulence.
    /// Spectral index: -3/2. Alfven wave interactions slow the cascade.
    IroshnikovKraichnan,

    /// Goldreich-Sridhar (1995) critical-balance strong MHD turbulence.
    /// Spectral index: -5/3 perpendicular to B0, -2 parallel.
    GoldreichSridhar,

    /// Bohm diffusion model. Empirical scaling D ~ kT/(16eB).
    Bohm,

    /// GyroBohm diffusion model. Scaling D ~ (rho_i/a) * D_Bohm.
    GyroBohm,
}

// ──────────────────────────────────────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────────────────────────────────────

/// Configuration for the plasma turbulence analyzer.
#[derive(Debug, Clone)]
pub struct TurbulenceConfig {
    /// Sampling rate of the diagnostic signal (Hz).
    pub sample_rate_hz: f64,

    /// Background magnetic field strength (Tesla).
    pub magnetic_field_tesla: f64,

    /// Plasma number density (particles per m^3).
    pub plasma_density_m3: f64,

    /// Ion mass (kg). Defaults to proton mass for hydrogen plasmas.
    pub ion_mass_kg: f64,

    /// Plasma temperature (eV). 1 eV = 11604.5 K.
    pub temperature_ev: f64,

    /// Turbulence cascade model to use for predictions.
    pub turbulence_model: TurbulenceModel,
}

impl Default for TurbulenceConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 1.0e6,
            magnetic_field_tesla: 5.0,
            plasma_density_m3: 1.0e20,
            ion_mass_kg: PROTON_MASS,
            temperature_ev: 1000.0,
            turbulence_model: TurbulenceModel::Kolmogorov,
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// PlasmaTurbulenceAnalyzer
// ──────────────────────────────────────────────────────────────────────────────

/// Core analyzer for MHD turbulence spectra and plasma parameters.
///
/// Provides methods for computing fundamental plasma parameters (Alfven speed,
/// gyroradius, Debye length, plasma beta, etc.), power spectra, spectral index
/// fitting, structure functions, and correlation lengths.
#[derive(Debug, Clone)]
pub struct PlasmaTurbulenceAnalyzer {
    config: TurbulenceConfig,
}

impl PlasmaTurbulenceAnalyzer {
    /// Create a new analyzer from the given configuration.
    pub fn new(config: TurbulenceConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &TurbulenceConfig {
        &self.config
    }

    // ── Fundamental plasma parameters ────────────────────────────────────

    /// Alfven speed v_A = B / sqrt(mu_0 * n * m_i).
    ///
    /// The characteristic propagation speed of shear Alfven waves along the
    /// magnetic field. For tokamak conditions (B=5T, n=1e20, hydrogen),
    /// v_A ~ 10^7 m/s.
    ///
    /// # Arguments
    /// * `b_tesla` - Magnetic field strength (T)
    /// * `density_m3` - Plasma number density (m^-3)
    /// * `ion_mass` - Ion mass (kg)
    pub fn alfven_speed(b_tesla: f64, density_m3: f64, ion_mass: f64) -> f64 {
        b_tesla / (MU_0 * density_m3 * ion_mass).sqrt()
    }

    /// Ion cyclotron (gyro) frequency omega_ci = q * B / m.
    ///
    /// # Arguments
    /// * `b_tesla` - Magnetic field strength (T)
    /// * `mass_kg` - Ion mass (kg)
    /// * `charge_c` - Ion charge (C), typically ELEMENTARY_CHARGE
    pub fn ion_gyrofrequency(b_tesla: f64, mass_kg: f64, charge_c: f64) -> f64 {
        charge_c * b_tesla / mass_kg
    }

    /// Ion gyroradius (Larmor radius) rho_i = v_th / omega_ci.
    ///
    /// The thermal speed is v_th = sqrt(kT/m_i), where T is in eV
    /// (converted via T_joules = T_eV * e).
    ///
    /// For tokamak conditions (1 keV, B=5T, hydrogen), rho_i ~ few mm.
    ///
    /// # Arguments
    /// * `temp_ev` - Ion temperature (eV)
    /// * `b_tesla` - Magnetic field strength (T)
    /// * `mass_kg` - Ion mass (kg)
    pub fn ion_gyroradius(temp_ev: f64, b_tesla: f64, mass_kg: f64) -> f64 {
        let t_joules = temp_ev * ELEMENTARY_CHARGE;
        let v_th = (t_joules / mass_kg).sqrt();
        let omega_ci = ELEMENTARY_CHARGE * b_tesla / mass_kg;
        v_th / omega_ci
    }

    /// Debye length lambda_D = sqrt(epsilon_0 * kT / (n * e^2)).
    ///
    /// The characteristic shielding distance in a plasma. For tokamak
    /// conditions (1 keV, n=1e20), lambda_D ~ tens of micrometers.
    ///
    /// # Arguments
    /// * `temp_ev` - Electron temperature (eV)
    /// * `density_m3` - Electron density (m^-3)
    pub fn debye_length(temp_ev: f64, density_m3: f64) -> f64 {
        let t_joules = temp_ev * ELEMENTARY_CHARGE;
        (EPSILON_0 * t_joules / (density_m3 * ELEMENTARY_CHARGE * ELEMENTARY_CHARGE)).sqrt()
    }

    /// Electron plasma frequency omega_pe = sqrt(n * e^2 / (epsilon_0 * m_e)).
    ///
    /// # Arguments
    /// * `density_m3` - Electron density (m^-3)
    pub fn plasma_frequency(density_m3: f64) -> f64 {
        (density_m3 * ELEMENTARY_CHARGE * ELEMENTARY_CHARGE
            / (EPSILON_0 * ELECTRON_MASS))
            .sqrt()
    }

    /// Plasma beta: ratio of thermal to magnetic pressure.
    ///
    /// beta = 2 * mu_0 * n * k * T / B^2
    ///
    /// Beta < 1 means magnetically dominated (typical for tokamaks).
    /// Beta > 1 means thermally dominated (some astrophysical plasmas).
    ///
    /// # Arguments
    /// * `density_m3` - Plasma density (m^-3)
    /// * `temp_ev` - Temperature (eV)
    /// * `b_tesla` - Magnetic field (T)
    pub fn plasma_beta(density_m3: f64, temp_ev: f64, b_tesla: f64) -> f64 {
        let t_joules = temp_ev * ELEMENTARY_CHARGE;
        2.0 * MU_0 * density_m3 * t_joules / (b_tesla * b_tesla)
    }

    // ── Spectral analysis ────────────────────────────────────────────────

    /// Compute the power spectrum of a real-valued signal.
    ///
    /// Uses a simple DFT (Cooley-Tukey for power-of-two, or direct DFT)
    /// and returns (frequency_hz, power) pairs for positive frequencies.
    ///
    /// # Arguments
    /// * `signal` - Time-domain signal samples
    pub fn power_spectrum(&self, signal: &[f64]) -> Vec<(f64, f64)> {
        let n = signal.len();
        if n == 0 {
            return vec![];
        }

        // Compute DFT magnitudes squared
        let spectrum = dft_magnitude_squared(signal);
        let n_pos = n / 2 + 1;
        let df = self.config.sample_rate_hz / n as f64;

        (0..n_pos)
            .map(|k| {
                let freq = k as f64 * df;
                let power = spectrum[k] / (n as f64 * n as f64);
                (freq, power)
            })
            .collect()
    }

    /// Fit a power-law spectral index alpha to P(f) ~ f^alpha
    /// over the frequency range [f_min, f_max].
    ///
    /// Uses least-squares linear regression in log-log space:
    /// log(P) = alpha * log(f) + const.
    ///
    /// # Arguments
    /// * `spectrum` - (frequency, power) pairs
    /// * `f_min` - Lower bound of fit range (Hz)
    /// * `f_max` - Upper bound of fit range (Hz)
    pub fn spectral_index(spectrum: &[(f64, f64)], f_min: f64, f_max: f64) -> f64 {
        let points: Vec<(f64, f64)> = spectrum
            .iter()
            .filter(|&&(f, p)| f >= f_min && f <= f_max && f > 0.0 && p > 0.0)
            .map(|&(f, p)| (f.ln(), p.ln()))
            .collect();

        if points.len() < 2 {
            return 0.0;
        }

        // Linear regression: y = a*x + b, where y=ln(P), x=ln(f)
        let n = points.len() as f64;
        let sum_x: f64 = points.iter().map(|(x, _)| x).sum();
        let sum_y: f64 = points.iter().map(|(_, y)| y).sum();
        let sum_xy: f64 = points.iter().map(|(x, y)| x * y).sum();
        let sum_xx: f64 = points.iter().map(|(x, _)| x * x).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1.0e-30 {
            return 0.0;
        }

        (n * sum_xy - sum_x * sum_y) / denom
    }

    /// Return the theoretical spectral index for the given turbulence model.
    ///
    /// - Kolmogorov: -5/3
    /// - Iroshnikov-Kraichnan: -3/2
    /// - Goldreich-Sridhar: -5/3 (perpendicular cascade)
    /// - Bohm: -1 (1/f-like empirical)
    /// - GyroBohm: -1 (similar empirical)
    pub fn expected_spectral_index(model: &TurbulenceModel) -> f64 {
        match model {
            TurbulenceModel::Kolmogorov => -5.0 / 3.0,
            TurbulenceModel::IroshnikovKraichnan => -3.0 / 2.0,
            TurbulenceModel::GoldreichSridhar => -5.0 / 3.0,
            TurbulenceModel::Bohm => -1.0,
            TurbulenceModel::GyroBohm => -1.0,
        }
    }

    /// Compute the p-th order structure function S_p(tau) = <|delta_B(tau)|^p>.
    ///
    /// Structure functions reveal intermittency and scaling properties of
    /// the turbulence. For Kolmogorov turbulence, S_2(tau) ~ tau^(2/3).
    ///
    /// # Arguments
    /// * `signal` - Time-domain signal
    /// * `order` - Structure function order p (typically 1-6)
    /// * `lags` - Array of lag values (in samples) to evaluate
    pub fn structure_function(
        signal: &[f64],
        order: usize,
        lags: &[usize],
    ) -> Vec<(usize, f64)> {
        let n = signal.len();
        lags.iter()
            .filter_map(|&lag| {
                if lag == 0 || lag >= n {
                    return None;
                }
                let count = n - lag;
                let sum: f64 = (0..count)
                    .map(|i| (signal[i + lag] - signal[i]).abs().powi(order as i32))
                    .sum();
                Some((lag, sum / count as f64))
            })
            .collect()
    }

    /// Compute the correlation length of a signal as the e-folding distance
    /// of the normalized autocorrelation function.
    ///
    /// The correlation length L_c is defined as the smallest lag tau where
    /// R(tau)/R(0) <= 1/e.
    ///
    /// # Arguments
    /// * `signal` - Time-domain signal
    pub fn correlation_length(signal: &[f64]) -> f64 {
        let n = signal.len();
        if n < 2 {
            return 0.0;
        }

        let mean: f64 = signal.iter().sum::<f64>() / n as f64;
        let r0: f64 = signal.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        if r0 < 1.0e-30 {
            return 0.0;
        }

        let threshold = 1.0 / std::f64::consts::E;
        let max_lag = n / 2;

        for lag in 1..=max_lag {
            let r: f64 = (0..n - lag)
                .map(|i| (signal[i] - mean) * (signal[i + lag] - mean))
                .sum::<f64>()
                / n as f64;
            if r / r0 <= threshold {
                // Linear interpolation for sub-sample precision
                if lag == 1 {
                    return lag as f64;
                }
                let r_prev: f64 = (0..n - (lag - 1))
                    .map(|i| (signal[i] - mean) * (signal[i + lag - 1] - mean))
                    .sum::<f64>()
                    / n as f64;
                let r_prev_norm = r_prev / r0;
                let r_norm = r / r0;
                let frac = (r_prev_norm - threshold) / (r_prev_norm - r_norm);
                return (lag - 1) as f64 + frac;
            }
        }

        max_lag as f64
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// EnergyTransportAnalyzer
// ──────────────────────────────────────────────────────────────────────────────

/// Analyzer for turbulent energy transport in confined plasmas.
///
/// Computes diffusion coefficients, energy confinement times, and cross-phase
/// analysis for understanding anomalous transport driven by turbulence.
pub struct EnergyTransportAnalyzer;

impl EnergyTransportAnalyzer {
    /// Bohm diffusion coefficient D_B = kT / (16 * e * B).
    ///
    /// An empirical upper bound on cross-field diffusion. Historically observed
    /// in many early plasma experiments, now understood as anomalous transport
    /// driven by turbulence.
    ///
    /// # Arguments
    /// * `temp_ev` - Temperature (eV)
    /// * `b_tesla` - Magnetic field (T)
    pub fn diffusion_coefficient_bohm(temp_ev: f64, b_tesla: f64) -> f64 {
        let t_joules = temp_ev * ELEMENTARY_CHARGE;
        t_joules / (16.0 * ELEMENTARY_CHARGE * b_tesla)
    }

    /// GyroBohm diffusion coefficient D_GB = (rho_i / a) * D_Bohm.
    ///
    /// Refined scaling that accounts for the ratio of ion gyroradius to
    /// device size, providing better agreement with modern tokamak data.
    ///
    /// # Arguments
    /// * `temp_ev` - Temperature (eV)
    /// * `b_tesla` - Magnetic field (T)
    /// * `a_m` - Device minor radius (m)
    pub fn diffusion_coefficient_gyrobohm(temp_ev: f64, b_tesla: f64, a_m: f64) -> f64 {
        let d_bohm = Self::diffusion_coefficient_bohm(temp_ev, b_tesla);
        let rho_i = PlasmaTurbulenceAnalyzer::ion_gyroradius(temp_ev, b_tesla, PROTON_MASS);
        rho_i / a_m * d_bohm
    }

    /// Energy confinement time tau_E = W / P.
    ///
    /// Fundamental figure of merit for fusion devices. The ratio of stored
    /// plasma energy to input heating power. ITER targets tau_E ~ 3-4 s.
    ///
    /// # Arguments
    /// * `volume_m3` - Plasma volume (m^3) [unused in simple model, kept for API]
    /// * `stored_energy_j` - Total stored thermal energy (J)
    /// * `heating_power_w` - Total heating power (W)
    pub fn energy_confinement_time(
        _volume_m3: f64,
        stored_energy_j: f64,
        heating_power_w: f64,
    ) -> f64 {
        if heating_power_w.abs() < 1.0e-30 {
            return f64::INFINITY;
        }
        stored_energy_j / heating_power_w
    }

    /// Relative fluctuation level: delta_B/B0 or delta_n/n0.
    ///
    /// RMS of (signal - mean) divided by mean. A key turbulence diagnostic;
    /// typical tokamak edge fluctuation levels are 10-30%, core levels ~1%.
    ///
    /// # Arguments
    /// * `signal` - Time-domain signal (e.g., density or magnetic field)
    pub fn fluctuation_level(signal: &[f64]) -> f64 {
        let n = signal.len();
        if n == 0 {
            return 0.0;
        }
        let mean: f64 = signal.iter().sum::<f64>() / n as f64;
        if mean.abs() < 1.0e-30 {
            return 0.0;
        }
        let variance: f64 = signal.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        variance.sqrt() / mean.abs()
    }

    /// Cross-phase spectrum between two signals.
    ///
    /// Computes the frequency-dependent phase angle between two signals,
    /// useful for transport analysis (e.g., density-potential cross-phase
    /// determines particle flux direction).
    ///
    /// Returns (frequency_hz, phase_radians) pairs.
    ///
    /// # Arguments
    /// * `signal1` - First time-domain signal
    /// * `signal2` - Second time-domain signal (same length)
    pub fn cross_phase(signal1: &[f64], signal2: &[f64]) -> Vec<(f64, f64)> {
        let n = signal1.len().min(signal2.len());
        if n == 0 {
            return vec![];
        }

        let fft1 = dft_complex(signal1);
        let fft2 = dft_complex(signal2);

        let n_pos = n / 2 + 1;
        (0..n_pos)
            .map(|k| {
                let freq = k as f64 / n as f64; // normalized frequency
                // Cross-spectrum: S12 = X1* . X2
                let cross_re = fft1[k].0 * fft2[k].0 + fft1[k].1 * fft2[k].1;
                let cross_im = fft1[k].0 * fft2[k].1 - fft1[k].1 * fft2[k].0;
                let phase = cross_im.atan2(cross_re);
                (freq, phase)
            })
            .collect()
    }

    /// Magnitude-squared coherence spectrum gamma^2(f).
    ///
    /// Measures the linear correlation between two signals as a function
    /// of frequency. gamma^2 = 1.0 means perfect linear relationship,
    /// gamma^2 = 0.0 means no linear relationship.
    ///
    /// gamma^2(f) = |S_12(f)|^2 / (S_11(f) * S_22(f))
    ///
    /// Returns (frequency_normalized, coherence) pairs.
    ///
    /// # Arguments
    /// * `signal1` - First time-domain signal
    /// * `signal2` - Second time-domain signal (same length)
    pub fn coherence_spectrum(signal1: &[f64], signal2: &[f64]) -> Vec<(f64, f64)> {
        let n = signal1.len().min(signal2.len());
        if n == 0 {
            return vec![];
        }

        let fft1 = dft_complex(signal1);
        let fft2 = dft_complex(signal2);

        let n_pos = n / 2 + 1;
        (0..n_pos)
            .map(|k| {
                let freq = k as f64 / n as f64;
                // Auto-spectra
                let s11 = fft1[k].0 * fft1[k].0 + fft1[k].1 * fft1[k].1;
                let s22 = fft2[k].0 * fft2[k].0 + fft2[k].1 * fft2[k].1;
                // Cross-spectrum magnitude squared
                let cross_re = fft1[k].0 * fft2[k].0 + fft1[k].1 * fft2[k].1;
                let cross_im = fft1[k].0 * fft2[k].1 - fft1[k].1 * fft2[k].0;
                let s12_mag_sq = cross_re * cross_re + cross_im * cross_im;

                let denom = s11 * s22;
                let coh = if denom > 1.0e-30 {
                    (s12_mag_sq / denom).min(1.0)
                } else {
                    0.0
                };
                (freq, coh)
            })
            .collect()
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// WaveletTurbulenceAnalyzer
// ──────────────────────────────────────────────────────────────────────────────

/// Wavelet-based turbulence analysis for intermittency characterization.
///
/// Continuous wavelet transforms provide time-scale decomposition that reveals
/// intermittent structures (coherent vortices, current sheets) not visible in
/// Fourier spectra.
pub struct WaveletTurbulenceAnalyzer;

impl WaveletTurbulenceAnalyzer {
    /// Compute a scalogram (continuous wavelet transform magnitudes).
    ///
    /// Uses a Morlet-like wavelet (modulated Gaussian) at `num_scales`
    /// logarithmically spaced scales. Returns a 2D array [scale][time].
    ///
    /// # Arguments
    /// * `signal` - Time-domain signal
    /// * `num_scales` - Number of wavelet scales (rows in output)
    pub fn scalogram(signal: &[f64], num_scales: usize) -> Vec<Vec<f64>> {
        let n = signal.len();
        if n == 0 || num_scales == 0 {
            return vec![];
        }

        let s_min = 2.0;
        let s_max = (n as f64) / 2.0;
        let log_ratio = (s_max / s_min).ln();

        (0..num_scales)
            .map(|si| {
                let scale =
                    s_min * (log_ratio * si as f64 / (num_scales.max(2) - 1) as f64).exp();
                morlet_cwt(signal, scale)
            })
            .collect()
    }

    /// Intermittency measure via flatness (kurtosis) of increments.
    ///
    /// For Gaussian (non-intermittent) turbulence, flatness = 3. Higher
    /// values indicate intermittency (fat tails from coherent structures).
    ///
    /// F_p = <|delta x|^p> / <|delta x|^2>^(p/2)
    ///
    /// For p=4, this is the standard flatness/kurtosis.
    ///
    /// # Arguments
    /// * `signal` - Time-domain signal
    /// * `order` - Moment order p (typically 4 for kurtosis)
    pub fn intermittency_measure(signal: &[f64], order: usize) -> f64 {
        let n = signal.len();
        if n < 2 {
            return 0.0;
        }

        // Compute increments (lag-1 differences)
        let increments: Vec<f64> = (1..n).map(|i| signal[i] - signal[i - 1]).collect();
        let m = increments.len() as f64;

        let moment_p: f64 = increments
            .iter()
            .map(|&dx| dx.abs().powi(order as i32))
            .sum::<f64>()
            / m;

        let moment_2: f64 = increments.iter().map(|&dx| dx * dx).sum::<f64>() / m;

        if moment_2 < 1.0e-30 {
            return 0.0;
        }

        moment_p / moment_2.powf(order as f64 / 2.0)
    }

    /// Local Intermittency Measure (LIM) from wavelet coefficients.
    ///
    /// LIM(s,t) = |W(s,t)|^2 / <|W(s,t)|^2>_t
    ///
    /// Values >> 1 indicate localized intense events at that scale and time.
    /// Useful for identifying coherent structures in turbulent flows.
    ///
    /// # Arguments
    /// * `wavelet_coeffs` - 2D wavelet coefficients [scale][time] (e.g. from scalogram)
    pub fn local_intermittency_measure(wavelet_coeffs: &[Vec<f64>]) -> Vec<Vec<f64>> {
        wavelet_coeffs
            .iter()
            .map(|row| {
                let n = row.len() as f64;
                if n < 1.0 {
                    return vec![];
                }
                let mean_sq: f64 = row.iter().map(|&w| w * w).sum::<f64>() / n;
                if mean_sq < 1.0e-30 {
                    return vec![0.0; row.len()];
                }
                row.iter().map(|&w| (w * w) / mean_sq).collect()
            })
            .collect()
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// MHDModeAnalyzer
// ──────────────────────────────────────────────────────────────────────────────

/// Analyzer for identifying MHD wave modes from dispersion relations.
///
/// In a magnetized plasma, perturbations propagate as Alfven waves (shear),
/// magnetosonic waves (fast/slow), and various instabilities. This analyzer
/// helps classify observed modes and compute growth rates.
pub struct MHDModeAnalyzer;

impl MHDModeAnalyzer {
    /// Check if a (frequency, wavenumber) pair is consistent with a shear
    /// Alfven wave: omega = k_parallel * v_alfven.
    ///
    /// # Arguments
    /// * `freq_hz` - Observed frequency (Hz)
    /// * `k_parallel` - Parallel wavenumber (rad/m)
    /// * `v_alfven` - Alfven speed (m/s)
    ///
    /// Returns true if the relative error is within 10%.
    pub fn identify_alfven_wave(freq_hz: f64, k_parallel: f64, v_alfven: f64) -> bool {
        let omega_obs = 2.0 * PI * freq_hz;
        let omega_alfven = k_parallel.abs() * v_alfven;
        if omega_alfven < 1.0e-30 {
            return false;
        }
        let rel_error = ((omega_obs - omega_alfven) / omega_alfven).abs();
        rel_error < 0.1
    }

    /// Check if a (frequency, wavenumber) pair is consistent with a
    /// magnetosonic wave (fast or slow mode).
    ///
    /// Fast mode: omega^2 = k^2 * (v_A^2 + v_s^2)
    /// Slow mode: omega^2 = k^2 * v_A^2 * v_s^2 / (v_A^2 + v_s^2)
    ///
    /// Returns true if either mode matches within 10%.
    ///
    /// # Arguments
    /// * `freq_hz` - Observed frequency (Hz)
    /// * `k` - Total wavenumber (rad/m)
    /// * `v_alfven` - Alfven speed (m/s)
    /// * `v_sound` - Sound speed (m/s)
    pub fn identify_magnetosonic(
        freq_hz: f64,
        k: f64,
        v_alfven: f64,
        v_sound: f64,
    ) -> bool {
        let omega_obs = 2.0 * PI * freq_hz;
        let omega_sq = omega_obs * omega_obs;

        // Fast mode
        let omega_fast_sq = k * k * (v_alfven * v_alfven + v_sound * v_sound);
        if omega_fast_sq > 1.0e-30 {
            let rel_error = ((omega_sq - omega_fast_sq) / omega_fast_sq).abs();
            if rel_error < 0.1 {
                return true;
            }
        }

        // Slow mode
        let v_a2 = v_alfven * v_alfven;
        let v_s2 = v_sound * v_sound;
        let sum = v_a2 + v_s2;
        if sum > 1.0e-30 {
            let omega_slow_sq = k * k * v_a2 * v_s2 / sum;
            if omega_slow_sq > 1.0e-30 {
                let rel_error = ((omega_sq - omega_slow_sq) / omega_slow_sq).abs();
                if rel_error < 0.1 {
                    return true;
                }
            }
        }

        false
    }

    /// Shear Alfven wave dispersion relation: omega(k) = k_parallel * v_A.
    ///
    /// # Arguments
    /// * `k` - Parallel wavenumber (rad/m)
    /// * `v_alfven` - Alfven speed (m/s)
    pub fn dispersion_relation_alfven(k: f64, v_alfven: f64) -> f64 {
        k.abs() * v_alfven
    }

    /// Tearing mode growth rate (resistive MHD instability).
    ///
    /// gamma ~ (eta * Delta')^(2/5) * tau_A^(-3/5)
    ///
    /// where tau_A = a / v_A is the Alfven transit time.
    ///
    /// Simplified formula: gamma = (eta * delta_prime^2)^(2/5) / a^(3/5)
    ///
    /// # Arguments
    /// * `eta` - Resistivity (Ohm*m)
    /// * `delta_prime` - Tearing stability parameter (1/m)
    /// * `a` - Characteristic length scale, e.g. minor radius (m)
    pub fn tearing_mode_growth_rate(eta: f64, delta_prime: f64, a: f64) -> f64 {
        // gamma = (eta * delta_prime^2)^(2/5) / a^(3/5)
        let numerator = (eta * delta_prime * delta_prime).powf(2.0 / 5.0);
        let denominator = a.powf(3.0 / 5.0);
        if denominator < 1.0e-30 {
            return 0.0;
        }
        numerator / denominator
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Internal helper functions
// ──────────────────────────────────────────────────────────────────────────────

/// Compute |DFT(x)|^2 for each frequency bin using direct DFT.
fn dft_magnitude_squared(x: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut result = vec![0.0; n];

    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (j, &xj) in x.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            re += xj * angle.cos();
            im += xj * angle.sin();
        }
        result[k] = re * re + im * im;
    }

    result
}

/// Compute complex DFT coefficients (re, im) for each frequency bin.
fn dft_complex(x: &[f64]) -> Vec<(f64, f64)> {
    let n = x.len();
    let mut result = Vec::with_capacity(n);

    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (j, &xj) in x.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            re += xj * angle.cos();
            im += xj * angle.sin();
        }
        result.push((re, im));
    }

    result
}

/// Morlet continuous wavelet transform at a given scale.
///
/// Returns the magnitude of the CWT coefficients for each time point.
fn morlet_cwt(signal: &[f64], scale: f64) -> Vec<f64> {
    let n = signal.len();
    let omega_0 = 6.0; // central frequency of Morlet wavelet
    let half_width = (3.0 * scale).ceil() as usize;

    (0..n)
        .map(|t| {
            let mut re = 0.0;
            let mut im = 0.0;
            let start = if t > half_width { t - half_width } else { 0 };
            let end = (t + half_width + 1).min(n);

            for i in start..end {
                let tau = (i as f64 - t as f64) / scale;
                // Morlet wavelet: psi(t) = pi^(-1/4) * exp(i*omega_0*t) * exp(-t^2/2)
                let envelope = (-tau * tau / 2.0).exp();
                let cos_part = (omega_0 * tau).cos() * envelope;
                let sin_part = (omega_0 * tau).sin() * envelope;
                re += signal[i] * cos_part;
                im += signal[i] * sin_part;
            }

            let norm = 1.0 / scale.sqrt();
            ((re * norm).powi(2) + (im * norm).powi(2)).sqrt()
        })
        .collect()
}

// ──────────────────────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> TurbulenceConfig {
        TurbulenceConfig::default()
    }

    fn tokamak_config() -> TurbulenceConfig {
        TurbulenceConfig {
            sample_rate_hz: 1.0e6,
            magnetic_field_tesla: 5.0,
            plasma_density_m3: 1.0e20,
            ion_mass_kg: PROTON_MASS,
            temperature_ev: 1000.0,
            turbulence_model: TurbulenceModel::GoldreichSridhar,
        }
    }

    // ── Alfven speed ─────────────────────────────────────────────────────

    #[test]
    fn test_alfven_speed_tokamak() {
        // B=5T, n=1e20 m^-3, proton mass => v_A ~ 10^7 m/s
        let v_a = PlasmaTurbulenceAnalyzer::alfven_speed(5.0, 1.0e20, PROTON_MASS);
        // Expected: 5 / sqrt(4pi*1e-7 * 1e20 * 1.67e-27) ~ 1.09e7
        assert!(v_a > 1.0e6, "Alfven speed should be > 1e6 m/s: {}", v_a);
        assert!(v_a < 1.0e8, "Alfven speed should be < 1e8 m/s: {}", v_a);
        // More precise check
        let expected = 5.0 / (MU_0 * 1.0e20 * PROTON_MASS).sqrt();
        assert!((v_a - expected).abs() / expected < 1.0e-10);
    }

    #[test]
    fn test_alfven_speed_solar_wind() {
        // Solar wind: B ~ 5 nT, n ~ 5 cm^-3 = 5e6 m^-3
        let v_a = PlasmaTurbulenceAnalyzer::alfven_speed(5.0e-9, 5.0e6, PROTON_MASS);
        // Expected: ~50 km/s
        assert!(v_a > 1.0e4, "Solar wind v_A should be > 10 km/s");
        assert!(v_a < 1.0e6, "Solar wind v_A should be < 1000 km/s");
    }

    #[test]
    fn test_alfven_speed_zero_density() {
        let v_a = PlasmaTurbulenceAnalyzer::alfven_speed(5.0, 0.0, PROTON_MASS);
        assert!(v_a.is_infinite());
    }

    // ── Ion gyrofrequency ────────────────────────────────────────────────

    #[test]
    fn test_ion_gyrofrequency() {
        let omega_ci =
            PlasmaTurbulenceAnalyzer::ion_gyrofrequency(5.0, PROTON_MASS, ELEMENTARY_CHARGE);
        // Expected: e*B/m_p = 1.6e-19 * 5 / 1.67e-27 ~ 4.79e8 rad/s
        assert!(omega_ci > 1.0e8, "omega_ci should be > 1e8 rad/s");
        assert!(omega_ci < 1.0e9, "omega_ci should be < 1e9 rad/s");
        let expected = ELEMENTARY_CHARGE * 5.0 / PROTON_MASS;
        assert!((omega_ci - expected).abs() / expected < 1.0e-10);
    }

    // ── Ion gyroradius ───────────────────────────────────────────────────

    #[test]
    fn test_ion_gyroradius_tokamak() {
        // 1 keV, B=5T, proton => rho_i ~ few mm
        let rho_i = PlasmaTurbulenceAnalyzer::ion_gyroradius(1000.0, 5.0, PROTON_MASS);
        assert!(rho_i > 1.0e-4, "Gyroradius should be > 0.1 mm: {:.3e}", rho_i);
        assert!(rho_i < 1.0e-2, "Gyroradius should be < 10 mm: {:.3e}", rho_i);
    }

    #[test]
    fn test_ion_gyroradius_scales_with_temperature() {
        let rho_low = PlasmaTurbulenceAnalyzer::ion_gyroradius(100.0, 5.0, PROTON_MASS);
        let rho_high = PlasmaTurbulenceAnalyzer::ion_gyroradius(10000.0, 5.0, PROTON_MASS);
        assert!(rho_high > rho_low, "Higher T should give larger gyroradius");
        // rho ~ sqrt(T), so ratio should be sqrt(100) = 10
        let ratio = rho_high / rho_low;
        assert!((ratio - 10.0).abs() < 0.1, "Ratio should be ~10: {}", ratio);
    }

    // ── Debye length ─────────────────────────────────────────────────────

    #[test]
    fn test_debye_length_tokamak() {
        // 1 keV, n=1e20 => lambda_D ~ tens of micrometers
        let lambda_d = PlasmaTurbulenceAnalyzer::debye_length(1000.0, 1.0e20);
        assert!(lambda_d > 1.0e-6, "Debye length should be > 1 um: {:.3e}", lambda_d);
        assert!(
            lambda_d < 1.0e-3,
            "Debye length should be < 1 mm: {:.3e}",
            lambda_d
        );
    }

    #[test]
    fn test_debye_length_scales_inversely_with_density() {
        let ld_low = PlasmaTurbulenceAnalyzer::debye_length(1000.0, 1.0e18);
        let ld_high = PlasmaTurbulenceAnalyzer::debye_length(1000.0, 1.0e20);
        assert!(
            ld_low > ld_high,
            "Lower density should give larger Debye length"
        );
        // lambda_D ~ 1/sqrt(n), so ratio should be sqrt(100) = 10
        let ratio = ld_low / ld_high;
        assert!((ratio - 10.0).abs() < 0.1, "Ratio should be ~10: {}", ratio);
    }

    // ── Plasma frequency ─────────────────────────────────────────────────

    #[test]
    fn test_plasma_frequency() {
        // n=1e20 => omega_pe ~ 5.6e11 rad/s
        let omega_pe = PlasmaTurbulenceAnalyzer::plasma_frequency(1.0e20);
        assert!(omega_pe > 1.0e11, "omega_pe too low: {:.3e}", omega_pe);
        assert!(omega_pe < 1.0e12, "omega_pe too high: {:.3e}", omega_pe);
    }

    #[test]
    fn test_plasma_frequency_scales_with_sqrt_density() {
        let wp1 = PlasmaTurbulenceAnalyzer::plasma_frequency(1.0e18);
        let wp2 = PlasmaTurbulenceAnalyzer::plasma_frequency(1.0e20);
        let ratio = wp2 / wp1;
        assert!((ratio - 10.0).abs() < 0.01, "Ratio should be 10: {}", ratio);
    }

    // ── Plasma beta ──────────────────────────────────────────────────────

    #[test]
    fn test_plasma_beta_tokamak_less_than_one() {
        // Tokamak: n=1e20, T=1keV, B=5T => beta < 1
        let beta = PlasmaTurbulenceAnalyzer::plasma_beta(1.0e20, 1000.0, 5.0);
        assert!(beta > 0.0, "Beta should be positive");
        assert!(beta < 1.0, "Tokamak beta should be < 1: {:.4}", beta);
    }

    #[test]
    fn test_plasma_beta_high_beta() {
        // Low field, hot dense plasma
        let beta = PlasmaTurbulenceAnalyzer::plasma_beta(1.0e21, 10000.0, 0.1);
        assert!(beta > 1.0, "High-beta regime expected: {:.2e}", beta);
    }

    // ── Spectral index ───────────────────────────────────────────────────

    #[test]
    fn test_expected_spectral_index_kolmogorov() {
        let idx = PlasmaTurbulenceAnalyzer::expected_spectral_index(&TurbulenceModel::Kolmogorov);
        assert!((idx - (-5.0 / 3.0)).abs() < 1.0e-10);
    }

    #[test]
    fn test_expected_spectral_index_ik() {
        let idx = PlasmaTurbulenceAnalyzer::expected_spectral_index(
            &TurbulenceModel::IroshnikovKraichnan,
        );
        assert!((idx - (-1.5)).abs() < 1.0e-10);
    }

    #[test]
    fn test_expected_spectral_index_gs() {
        let idx =
            PlasmaTurbulenceAnalyzer::expected_spectral_index(&TurbulenceModel::GoldreichSridhar);
        assert!((idx - (-5.0 / 3.0)).abs() < 1.0e-10);
    }

    #[test]
    fn test_spectral_index_from_power_law() {
        // Generate synthetic spectrum P(f) = f^(-5/3)
        let spectrum: Vec<(f64, f64)> = (1..=100)
            .map(|k| {
                let f = k as f64 * 10.0;
                let p = f.powf(-5.0 / 3.0);
                (f, p)
            })
            .collect();

        let alpha = PlasmaTurbulenceAnalyzer::spectral_index(&spectrum, 10.0, 1000.0);
        assert!(
            (alpha - (-5.0 / 3.0)).abs() < 0.01,
            "Expected ~-5/3, got {}",
            alpha
        );
    }

    #[test]
    fn test_spectral_index_ik_spectrum() {
        // Synthetic IK spectrum P(f) = f^(-3/2)
        let spectrum: Vec<(f64, f64)> = (1..=100)
            .map(|k| {
                let f = k as f64 * 10.0;
                let p = f.powf(-1.5);
                (f, p)
            })
            .collect();

        let alpha = PlasmaTurbulenceAnalyzer::spectral_index(&spectrum, 10.0, 1000.0);
        assert!(
            (alpha - (-1.5)).abs() < 0.01,
            "Expected ~-1.5, got {}",
            alpha
        );
    }

    #[test]
    fn test_spectral_index_empty() {
        let alpha = PlasmaTurbulenceAnalyzer::spectral_index(&[], 1.0, 10.0);
        assert_eq!(alpha, 0.0);
    }

    // ── Power spectrum ───────────────────────────────────────────────────

    #[test]
    fn test_power_spectrum_single_tone() {
        let config = TurbulenceConfig {
            sample_rate_hz: 1000.0,
            ..default_config()
        };
        let analyzer = PlasmaTurbulenceAnalyzer::new(config);

        // Generate 100 Hz tone at 1000 Hz sample rate
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 1000.0).sin())
            .collect();

        let spectrum = analyzer.power_spectrum(&signal);
        assert!(!spectrum.is_empty());

        // Find peak (should be near 100 Hz)
        let (peak_freq, _peak_power) = spectrum
            .iter()
            .skip(1) // skip DC
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();

        // Resolution is ~3.9 Hz, so peak should be within ~4 Hz of 100 Hz
        assert!(
            (peak_freq - 100.0).abs() < 5.0,
            "Peak at {} Hz, expected ~100 Hz",
            peak_freq
        );
    }

    #[test]
    fn test_power_spectrum_empty() {
        let analyzer = PlasmaTurbulenceAnalyzer::new(default_config());
        let spectrum = analyzer.power_spectrum(&[]);
        assert!(spectrum.is_empty());
    }

    // ── Structure function ───────────────────────────────────────────────

    #[test]
    fn test_structure_function_linear() {
        // For a linear signal y = x, increments are constant,
        // so S_2(lag) = lag^2 * c^2 for step c.
        let signal: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let lags: Vec<usize> = vec![1, 2, 5, 10];
        let sf = PlasmaTurbulenceAnalyzer::structure_function(&signal, 2, &lags);

        for &(lag, val) in &sf {
            let expected = (lag as f64).powi(2);
            assert!(
                (val - expected).abs() / expected < 0.02,
                "S2({}) = {}, expected {}",
                lag,
                val,
                expected
            );
        }
    }

    #[test]
    fn test_structure_function_order_1() {
        let signal: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let lags: Vec<usize> = vec![1, 5];
        let sf = PlasmaTurbulenceAnalyzer::structure_function(&signal, 1, &lags);

        // For linear signal, |x[i+lag] - x[i]| = lag, so S_1 = lag
        for &(lag, val) in &sf {
            let expected = lag as f64;
            assert!(
                (val - expected).abs() / expected < 0.02,
                "S1({}) = {}, expected {}",
                lag,
                val,
                expected
            );
        }
    }

    #[test]
    fn test_structure_function_zero_lag() {
        let signal = vec![1.0, 2.0, 3.0];
        let sf = PlasmaTurbulenceAnalyzer::structure_function(&signal, 2, &[0]);
        assert!(sf.is_empty(), "Zero lag should be filtered out");
    }

    // ── Correlation length ───────────────────────────────────────────────

    #[test]
    fn test_correlation_length_exponential_decay() {
        // Generate exponentially correlated signal: x[n] = alpha*x[n-1] + noise
        let n = 10000;
        let alpha = 0.95; // correlation ~ exp(-n/tau), tau ~ -1/ln(alpha) ~ 19.5
        let mut signal = vec![0.0; n];
        let mut state: u64 = 12345;
        for i in 1..n {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let noise = ((state >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.1;
            signal[i] = alpha * signal[i - 1] + noise;
        }

        let lc = PlasmaTurbulenceAnalyzer::correlation_length(&signal);
        let expected_tau = -1.0 / alpha.ln(); // ~19.5
        assert!(
            (lc - expected_tau).abs() / expected_tau < 0.3,
            "Correlation length {:.1} should be near {:.1}",
            lc,
            expected_tau
        );
    }

    #[test]
    fn test_correlation_length_white_noise() {
        // White noise should have very short correlation length (~1)
        let n = 1000;
        let mut state: u64 = 98765;
        let signal: Vec<f64> = (0..n)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                (state >> 33) as f64 / (1u64 << 31) as f64 - 0.5
            })
            .collect();

        let lc = PlasmaTurbulenceAnalyzer::correlation_length(&signal);
        assert!(lc < 5.0, "White noise should have short correlation: {}", lc);
    }

    #[test]
    fn test_correlation_length_constant_signal() {
        let signal = vec![5.0; 100];
        let lc = PlasmaTurbulenceAnalyzer::correlation_length(&signal);
        assert_eq!(lc, 0.0, "Constant signal has zero variance");
    }

    // ── Bohm diffusion ───────────────────────────────────────────────────

    #[test]
    fn test_bohm_diffusion_coefficient() {
        // D_B = kT/(16eB) = T_eV/(16*B) in natural units
        let d_b = EnergyTransportAnalyzer::diffusion_coefficient_bohm(1000.0, 5.0);
        // Expected: 1000 / (16 * 5) = 12.5 m^2/s ... but in SI:
        // D_B = 1000 * 1.6e-19 / (16 * 1.6e-19 * 5) = 1000/(16*5) = 12.5
        let expected = 1000.0 / (16.0 * 5.0);
        assert!(
            (d_b - expected).abs() / expected < 1.0e-10,
            "D_Bohm = {}, expected {}",
            d_b,
            expected
        );
    }

    #[test]
    fn test_bohm_diffusion_scales_with_temperature() {
        let d1 = EnergyTransportAnalyzer::diffusion_coefficient_bohm(100.0, 5.0);
        let d2 = EnergyTransportAnalyzer::diffusion_coefficient_bohm(200.0, 5.0);
        assert!(
            (d2 / d1 - 2.0).abs() < 0.01,
            "D_Bohm should scale linearly with T"
        );
    }

    // ── GyroBohm diffusion ───────────────────────────────────────────────

    #[test]
    fn test_gyrobohm_smaller_than_bohm() {
        let a_m = 0.5; // typical tokamak minor radius
        let d_bohm = EnergyTransportAnalyzer::diffusion_coefficient_bohm(1000.0, 5.0);
        let d_gyro = EnergyTransportAnalyzer::diffusion_coefficient_gyrobohm(1000.0, 5.0, a_m);
        assert!(
            d_gyro < d_bohm,
            "GyroBohm ({:.3e}) should be smaller than Bohm ({:.3e})",
            d_gyro,
            d_bohm
        );
    }

    // ── Energy confinement time ──────────────────────────────────────────

    #[test]
    fn test_energy_confinement_time() {
        let tau = EnergyTransportAnalyzer::energy_confinement_time(10.0, 1.0e6, 1.0e5);
        assert!((tau - 10.0).abs() < 1.0e-10, "tau_E should be 10 s");
    }

    #[test]
    fn test_energy_confinement_zero_power() {
        let tau = EnergyTransportAnalyzer::energy_confinement_time(10.0, 1.0e6, 0.0);
        assert!(tau.is_infinite());
    }

    // ── Fluctuation level ────────────────────────────────────────────────

    #[test]
    fn test_fluctuation_level() {
        // Signal = constant + 10% fluctuation
        let base = 100.0;
        let signal: Vec<f64> = (0..1000)
            .map(|i| base + 10.0 * (2.0 * PI * i as f64 / 100.0).sin())
            .collect();

        let fl = EnergyTransportAnalyzer::fluctuation_level(&signal);
        // RMS of sine = amplitude / sqrt(2) = 10/sqrt(2) ~ 7.07
        // Fluctuation level = 7.07 / 100 = 0.0707
        assert!(
            (fl - 0.0707).abs() < 0.005,
            "Fluctuation level = {}, expected ~0.0707",
            fl
        );
    }

    #[test]
    fn test_fluctuation_level_constant() {
        let signal = vec![42.0; 100];
        let fl = EnergyTransportAnalyzer::fluctuation_level(&signal);
        assert_eq!(fl, 0.0, "Constant signal has zero fluctuation");
    }

    // ── Cross-phase ──────────────────────────────────────────────────────

    #[test]
    fn test_cross_phase_identical_signals() {
        let signal: Vec<f64> = (0..64)
            .map(|i| (2.0 * PI * 5.0 * i as f64 / 64.0).sin())
            .collect();

        let cp = EnergyTransportAnalyzer::cross_phase(&signal, &signal);
        // Cross-phase of identical signals should be 0
        for &(_, phase) in &cp {
            assert!(
                phase.abs() < 1.0e-10,
                "Cross-phase of identical signals should be 0: {}",
                phase
            );
        }
    }

    // ── Coherence spectrum ───────────────────────────────────────────────

    #[test]
    fn test_coherence_identical_signals() {
        // Use a broadband signal (pseudo-random) so all frequency bins have energy.
        // For identical signals, coherence = |S12|^2 / (S11 * S22) = 1.0 everywhere.
        let n = 64;
        let mut state: u64 = 11111;
        let signal: Vec<f64> = (0..n)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                (state >> 33) as f64 / (1u64 << 31) as f64 - 0.5
            })
            .collect();

        let coh = EnergyTransportAnalyzer::coherence_spectrum(&signal, &signal);
        for &(_, gamma_sq) in &coh {
            assert!(
                (gamma_sq - 1.0).abs() < 1.0e-6,
                "Coherence of identical signals should be 1.0: {}",
                gamma_sq
            );
        }
    }

    #[test]
    fn test_coherence_empty_signal() {
        let coh = EnergyTransportAnalyzer::coherence_spectrum(&[], &[]);
        assert!(coh.is_empty());
    }

    // ── Wavelet scalogram ────────────────────────────────────────────────

    #[test]
    fn test_scalogram_dimensions() {
        let signal: Vec<f64> = (0..128)
            .map(|i| (2.0 * PI * 10.0 * i as f64 / 128.0).sin())
            .collect();

        let sg = WaveletTurbulenceAnalyzer::scalogram(&signal, 8);
        assert_eq!(sg.len(), 8, "Should have 8 scales");
        for row in &sg {
            assert_eq!(row.len(), 128, "Each scale should have 128 time points");
        }
    }

    #[test]
    fn test_scalogram_non_negative() {
        let signal: Vec<f64> = (0..64)
            .map(|i| (2.0 * PI * 5.0 * i as f64 / 64.0).sin())
            .collect();

        let sg = WaveletTurbulenceAnalyzer::scalogram(&signal, 4);
        for row in &sg {
            for &val in row {
                assert!(val >= 0.0, "Scalogram magnitudes must be non-negative");
            }
        }
    }

    // ── Intermittency ────────────────────────────────────────────────────

    #[test]
    fn test_intermittency_gaussian() {
        // For Gaussian increments, flatness (order=4) should be close to 3
        // Use pseudo-Gaussian via central limit (sum of uniforms)
        let n = 10000;
        let mut state: u64 = 54321;
        let signal: Vec<f64> = (0..n)
            .map(|_| {
                let mut sum = 0.0;
                for _ in 0..12 {
                    state = state
                        .wrapping_mul(6364136223846793005)
                        .wrapping_add(1442695040888963407);
                    sum += (state >> 33) as f64 / (1u64 << 31) as f64;
                }
                sum - 6.0 // centered
            })
            .collect();

        let flatness = WaveletTurbulenceAnalyzer::intermittency_measure(&signal, 4);
        // Gaussian flatness = 3
        assert!(
            (flatness - 3.0).abs() < 1.0,
            "Gaussian flatness should be ~3: {}",
            flatness
        );
    }

    // ── Local intermittency measure ──────────────────────────────────────

    #[test]
    fn test_local_intermittency_measure_mean_one() {
        // LIM should average to 1.0 at each scale
        let coeffs = vec![vec![1.0, 2.0, 3.0, 4.0, 5.0]];
        let lim = WaveletTurbulenceAnalyzer::local_intermittency_measure(&coeffs);
        let mean: f64 = lim[0].iter().sum::<f64>() / lim[0].len() as f64;
        assert!(
            (mean - 1.0).abs() < 0.01,
            "Mean LIM should be ~1.0: {}",
            mean
        );
    }

    // ── MHD mode identification ──────────────────────────────────────────

    #[test]
    fn test_identify_alfven_wave_exact() {
        let v_a = 1.0e7; // 10^7 m/s
        let k = 100.0; // rad/m
        let omega = k * v_a;
        let freq = omega / (2.0 * PI);
        assert!(MHDModeAnalyzer::identify_alfven_wave(freq, k, v_a));
    }

    #[test]
    fn test_identify_alfven_wave_reject() {
        let v_a = 1.0e7;
        let k = 100.0;
        // Frequency 50% too high
        let freq = 1.5 * k * v_a / (2.0 * PI);
        assert!(!MHDModeAnalyzer::identify_alfven_wave(freq, k, v_a));
    }

    #[test]
    fn test_identify_magnetosonic_fast() {
        let v_a = 1.0e6;
        let v_s = 5.0e5;
        let k = 10.0;
        let v_fast = ((v_a * v_a + v_s * v_s) as f64).sqrt();
        let omega = k * v_fast;
        let freq = omega / (2.0 * PI);
        assert!(MHDModeAnalyzer::identify_magnetosonic(freq, k, v_a, v_s));
    }

    #[test]
    fn test_identify_magnetosonic_slow() {
        let v_a = 1.0e6;
        let v_s = 5.0e5;
        let k = 10.0;
        let v_slow = ((v_a * v_a * v_s * v_s / (v_a * v_a + v_s * v_s)) as f64).sqrt();
        let omega = k * v_slow;
        let freq = omega / (2.0 * PI);
        assert!(MHDModeAnalyzer::identify_magnetosonic(freq, k, v_a, v_s));
    }

    #[test]
    fn test_dispersion_relation_alfven() {
        let v_a = 1.0e7;
        let k = 50.0;
        let omega = MHDModeAnalyzer::dispersion_relation_alfven(k, v_a);
        assert!((omega - k * v_a).abs() < 1.0e-6);
    }

    #[test]
    fn test_dispersion_relation_alfven_negative_k() {
        let v_a = 1.0e7;
        let omega = MHDModeAnalyzer::dispersion_relation_alfven(-50.0, v_a);
        assert!(omega > 0.0, "Frequency should be positive for negative k");
    }

    // ── Tearing mode ─────────────────────────────────────────────────────

    #[test]
    fn test_tearing_mode_growth_rate_positive() {
        let gamma = MHDModeAnalyzer::tearing_mode_growth_rate(1.0e-6, 10.0, 0.5);
        assert!(gamma > 0.0, "Growth rate should be positive");
    }

    #[test]
    fn test_tearing_mode_growth_rate_scales_with_resistivity() {
        let g1 = MHDModeAnalyzer::tearing_mode_growth_rate(1.0e-6, 10.0, 0.5);
        let g2 = MHDModeAnalyzer::tearing_mode_growth_rate(1.0e-5, 10.0, 0.5);
        assert!(
            g2 > g1,
            "Higher resistivity should increase tearing growth rate"
        );
    }

    // ── Constructor and config ───────────────────────────────────────────

    #[test]
    fn test_constructor_and_config() {
        let config = tokamak_config();
        let analyzer = PlasmaTurbulenceAnalyzer::new(config.clone());
        assert_eq!(analyzer.config().magnetic_field_tesla, 5.0);
        assert_eq!(
            analyzer.config().turbulence_model,
            TurbulenceModel::GoldreichSridhar
        );
    }

    #[test]
    fn test_default_config() {
        let config = TurbulenceConfig::default();
        assert_eq!(config.ion_mass_kg, PROTON_MASS);
        assert_eq!(config.magnetic_field_tesla, 5.0);
    }

    // ── Constants sanity checks ──────────────────────────────────────────

    #[test]
    fn test_constants_reasonable() {
        assert!((MU_0 - 1.2566e-6).abs() < 1.0e-9);
        assert!((EPSILON_0 - 8.854e-12).abs() < 1.0e-14);
        assert!((ELECTRON_MASS - 9.109e-31).abs() < 1.0e-33);
        assert!((PROTON_MASS - 1.673e-27).abs() < 1.0e-29);
        assert!((ELEMENTARY_CHARGE - 1.602e-19).abs() < 1.0e-22);
        assert!((BOLTZMANN_K - 1.381e-23).abs() < 1.0e-25);
    }

    #[test]
    fn test_turbulence_model_enum() {
        let models = [
            TurbulenceModel::Kolmogorov,
            TurbulenceModel::IroshnikovKraichnan,
            TurbulenceModel::GoldreichSridhar,
            TurbulenceModel::Bohm,
            TurbulenceModel::GyroBohm,
        ];
        // Ensure all models have distinct expected spectral indices (except K and GS)
        let idx_k = PlasmaTurbulenceAnalyzer::expected_spectral_index(&models[0]);
        let idx_ik = PlasmaTurbulenceAnalyzer::expected_spectral_index(&models[1]);
        assert!((idx_k - idx_ik).abs() > 0.1, "K and IK should differ");
    }
}

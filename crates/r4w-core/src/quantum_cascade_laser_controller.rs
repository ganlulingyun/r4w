//! # Quantum Cascade Laser Controller for Mid-IR Gas Spectroscopy
//!
//! This module implements Quantum Cascade Laser (QCL) control and spectroscopy
//! signal processing for mid-infrared gas sensing applications. QCLs are
//! semiconductor lasers that emit in the mid-infrared (3–25 µm), a region rich
//! in fundamental molecular absorption lines ("fingerprint region"), making them
//! ideal for trace gas detection.
//!
//! ## Background
//!
//! A QCL works by electron transitions between quantised sub-band states in a
//! series of coupled quantum wells (the "cascade").  By tuning the injection
//! current, the emission wavenumber shifts via the tuning coefficient, allowing
//! the laser to be swept across a target absorption line.  Combined with
//! Beer-Lambert absorption, lock-in detection, and Allan variance analysis, ppm
//! and sub-ppb detection limits are routinely achieved.
//!
//! ## Components
//!
//! - [`QclConfig`] -- laser physical parameters (wavelength, tuning, power).
//! - [`QclController`] -- current-to-wavenumber/power model, temperature tuning.
//! - [`GasLine`] -- HITRAN-style spectral line parameters for target gases.
//! - [`AbsorptionModel`] -- Beer-Lambert law, Voigt line profile, transmittance.
//! - [`WmsProcessor`] -- Wavelength Modulation Spectroscopy (1f / 2f lock-in).
//! - [`DasProcessor`] -- Direct Absorption Spectroscopy with baseline fitting.
//! - [`ConcentrationCalculator`] -- absorbance → ppm/ppb, minimum detectable.
//! - [`AllanVariance`] -- optimal integration time and detection limit analysis.
//! - [`EtalonSuppressor`] -- Fabry-Perot fringe removal by notch filtering.
//! - [`TecController`] -- PID-based thermoelectric cooler simulation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::quantum_cascade_laser_controller::*;
//!
//! // Configure a QCL targeting the CO2 asymmetric stretch at 4.26 µm
//! let config = QclConfig {
//!     center_wavelength_um: 4.26,
//!     tuning_range_cm1: 5.0,
//!     max_power_mw: 100.0,
//!     threshold_current_ma: 250.0,
//!     slope_efficiency_mw_per_ma: 0.5,
//!     tuning_coefficient_cm1_per_ma: 0.01,
//! };
//! let ctrl = QclController::new(config);
//!
//! // Compute wavenumber at 300 mA
//! let nu = ctrl.wavenumber_at_current(300.0);
//! assert!((nu - 2347.5).abs() < 1.0); // near CO2 line center
//!
//! // Compute output power
//! let p = ctrl.power_at_current(300.0);
//! assert!(p > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// QCL Configuration
// ---------------------------------------------------------------------------

/// Physical parameters of a quantum cascade laser.
#[derive(Debug, Clone)]
pub struct QclConfig {
    /// Center emission wavelength in micrometres.
    pub center_wavelength_um: f64,
    /// Total wavenumber tuning range (cm⁻¹).
    pub tuning_range_cm1: f64,
    /// Maximum CW output power (mW).
    pub max_power_mw: f64,
    /// Threshold current (mA).
    pub threshold_current_ma: f64,
    /// Slope efficiency above threshold (mW / mA).
    pub slope_efficiency_mw_per_ma: f64,
    /// Current tuning coefficient (cm⁻¹ / mA).
    pub tuning_coefficient_cm1_per_ma: f64,
}

impl QclConfig {
    /// Preset for a CO₂-targeting QCL at 4.26 µm.
    pub fn co2_4um() -> Self {
        Self {
            center_wavelength_um: 4.26,
            tuning_range_cm1: 5.0,
            max_power_mw: 100.0,
            threshold_current_ma: 250.0,
            slope_efficiency_mw_per_ma: 0.5,
            tuning_coefficient_cm1_per_ma: 0.01,
        }
    }

    /// Preset for a CO-targeting QCL at 4.67 µm.
    pub fn co_4um() -> Self {
        Self {
            center_wavelength_um: 4.67,
            tuning_range_cm1: 4.0,
            max_power_mw: 80.0,
            threshold_current_ma: 220.0,
            slope_efficiency_mw_per_ma: 0.4,
            tuning_coefficient_cm1_per_ma: 0.008,
        }
    }

    /// Preset for a CH₄-targeting QCL at 3.31 µm.
    pub fn ch4_3um() -> Self {
        Self {
            center_wavelength_um: 3.31,
            tuning_range_cm1: 6.0,
            max_power_mw: 50.0,
            threshold_current_ma: 200.0,
            slope_efficiency_mw_per_ma: 0.3,
            tuning_coefficient_cm1_per_ma: 0.012,
        }
    }

    /// Center wavenumber in cm⁻¹ derived from center wavelength.
    pub fn center_wavenumber_cm1(&self) -> f64 {
        1.0e4 / self.center_wavelength_um
    }
}

// ---------------------------------------------------------------------------
// QCL Controller
// ---------------------------------------------------------------------------

/// QCL current/temperature-to-optical-output model.
///
/// Models the relationship between injection current, chip temperature,
/// output wavenumber, and optical power.
#[derive(Debug, Clone)]
pub struct QclController {
    config: QclConfig,
    /// Center wavenumber ν₀ (cm⁻¹).
    nu0: f64,
    /// Operating temperature (K).
    temperature_k: f64,
    /// Reference temperature for thermal tuning (K).
    reference_temperature_k: f64,
    /// Thermal tuning coefficient (cm⁻¹/K), typically −0.06 for mid-IR QCL.
    thermal_tuning_coeff: f64,
}

impl QclController {
    /// Create a new controller from the given config, defaulting to 293 K.
    pub fn new(config: QclConfig) -> Self {
        let nu0 = config.center_wavenumber_cm1();
        Self {
            config,
            nu0,
            temperature_k: 293.0,
            reference_temperature_k: 293.0,
            thermal_tuning_coeff: -0.06,
        }
    }

    /// Set operating temperature (K).
    pub fn set_temperature(&mut self, temp_k: f64) {
        self.temperature_k = temp_k;
    }

    /// Set thermal tuning coefficient (cm⁻¹/K).
    pub fn set_thermal_tuning_coeff(&mut self, coeff: f64) {
        self.thermal_tuning_coeff = coeff;
    }

    /// Current at which the laser emits at the center wavenumber.
    pub fn center_current_ma(&self) -> f64 {
        // The center current is defined as the midpoint of the tuning range
        // above threshold.
        self.config.threshold_current_ma
            + self.config.tuning_range_cm1
                / (2.0 * self.config.tuning_coefficient_cm1_per_ma)
    }

    /// Wavenumber (cm⁻¹) at a given injection current (mA), including
    /// thermal shift from the reference temperature.
    ///
    /// ν = ν₀ + k_tune · (I − I_center) + (dν/dT) · (T − T_ref)
    pub fn wavenumber_at_current(&self, current_ma: f64) -> f64 {
        let i_center = self.center_current_ma();
        let thermal_shift =
            self.thermal_tuning_coeff * (self.temperature_k - self.reference_temperature_k);
        self.nu0 + self.config.tuning_coefficient_cm1_per_ma * (current_ma - i_center)
            + thermal_shift
    }

    /// Optical power (mW) at a given injection current (mA).
    ///
    /// P = η · (I − I_th)  for I > I_th, else 0.
    /// Clamped to max_power_mw.
    pub fn power_at_current(&self, current_ma: f64) -> f64 {
        if current_ma <= self.config.threshold_current_ma {
            return 0.0;
        }
        let p = self.config.slope_efficiency_mw_per_ma
            * (current_ma - self.config.threshold_current_ma);
        p.min(self.config.max_power_mw)
    }

    /// Generate a current ramp from `i_start` to `i_end` (mA) with `n`
    /// equally-spaced points.  Returns (currents, wavenumbers, powers).
    pub fn scan(
        &self,
        i_start: f64,
        i_end: f64,
        n: usize,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut currents = Vec::with_capacity(n);
        let mut wavenumbers = Vec::with_capacity(n);
        let mut powers = Vec::with_capacity(n);
        for k in 0..n {
            let frac = if n > 1 { k as f64 / (n - 1) as f64 } else { 0.0 };
            let i = i_start + frac * (i_end - i_start);
            currents.push(i);
            wavenumbers.push(self.wavenumber_at_current(i));
            powers.push(self.power_at_current(i));
        }
        (currents, wavenumbers, powers)
    }

    /// Access to the underlying config.
    pub fn config(&self) -> &QclConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// Gas Line (HITRAN-style parameters)
// ---------------------------------------------------------------------------

/// HITRAN-style spectral line parameters for a gas absorption feature.
#[derive(Debug, Clone)]
pub struct GasLine {
    /// Gas species name.
    pub species: &'static str,
    /// Line center wavenumber (cm⁻¹).
    pub nu0: f64,
    /// Line strength S (cm⁻¹ / (molecule · cm⁻²)) at reference T (296 K).
    pub line_strength: f64,
    /// Air-broadened half-width at half-maximum γ_air (cm⁻¹ / atm) at 296 K.
    pub gamma_air: f64,
    /// Self-broadened half-width γ_self (cm⁻¹ / atm).
    pub gamma_self: f64,
    /// Lower state energy (cm⁻¹), for temperature scaling.
    pub lower_state_energy: f64,
}

impl GasLine {
    /// CO₂ asymmetric stretch ν₃ near 2349 cm⁻¹.
    pub fn co2_2349() -> Self {
        Self {
            species: "CO2",
            nu0: 2349.14,
            line_strength: 4.16e-19,
            gamma_air: 0.0685,
            gamma_self: 0.0980,
            lower_state_energy: 667.38,
        }
    }

    /// CO fundamental R(6) near 2143 cm⁻¹.
    pub fn co_2143() -> Self {
        Self {
            species: "CO",
            nu0: 2143.27,
            line_strength: 1.08e-19,
            gamma_air: 0.0578,
            gamma_self: 0.0600,
            lower_state_energy: 89.59,
        }
    }

    /// CH₄ ν₃ band near 3019 cm⁻¹.
    pub fn ch4_3019() -> Self {
        Self {
            species: "CH4",
            nu0: 3019.49,
            line_strength: 9.84e-20,
            gamma_air: 0.0604,
            gamma_self: 0.0774,
            lower_state_energy: 62.88,
        }
    }

    /// N₂O ν₃ asymmetric stretch near 2224 cm⁻¹.
    pub fn n2o_2224() -> Self {
        Self {
            species: "N2O",
            nu0: 2223.76,
            line_strength: 4.83e-19,
            gamma_air: 0.0728,
            gamma_self: 0.0830,
            lower_state_energy: 556.16,
        }
    }

    /// NO fundamental near 1876 cm⁻¹.
    pub fn no_1876() -> Self {
        Self {
            species: "NO",
            nu0: 1876.08,
            line_strength: 2.21e-20,
            gamma_air: 0.0530,
            gamma_self: 0.0540,
            lower_state_energy: 36.44,
        }
    }

    /// Pressure-broadened half-width (cm⁻¹) at given total and partial
    /// pressures (atm) and temperature (K), relative to 296 K.
    ///
    /// γ(p,T) = (T_ref/T)^0.75 · (γ_air · (p − p_self) + γ_self · p_self)
    pub fn broadened_halfwidth(
        &self,
        total_pressure_atm: f64,
        partial_pressure_atm: f64,
        temperature_k: f64,
    ) -> f64 {
        let t_ratio = (296.0 / temperature_k).powf(0.75);
        t_ratio
            * (self.gamma_air * (total_pressure_atm - partial_pressure_atm)
                + self.gamma_self * partial_pressure_atm)
    }
}

// ---------------------------------------------------------------------------
// Absorption Model
// ---------------------------------------------------------------------------

/// Beer-Lambert absorption and Voigt line-shape calculations.
pub struct AbsorptionModel;

impl AbsorptionModel {
    /// Beer-Lambert transmitted intensity.
    ///
    /// I_out = I_0 · exp(−α · C · L)
    ///
    /// * `i0` -- incident intensity (arbitrary units)
    /// * `alpha` -- absorption coefficient (cm⁻¹ per (mol/cm³))
    /// * `concentration` -- concentration (mol/cm³)
    /// * `path_length_cm` -- optical path length (cm)
    pub fn beer_lambert(i0: f64, alpha: f64, concentration: f64, path_length_cm: f64) -> f64 {
        i0 * (-alpha * concentration * path_length_cm).exp()
    }

    /// Absorbance A = −ln(I/I₀) = α · C · L.
    pub fn absorbance(alpha: f64, concentration: f64, path_length_cm: f64) -> f64 {
        alpha * concentration * path_length_cm
    }

    /// Transmittance T = I/I₀ = exp(−A).
    pub fn transmittance(absorbance: f64) -> f64 {
        (-absorbance).exp()
    }

    /// Lorentzian (pressure-broadened) line shape (cm).
    ///
    /// g_L(ν) = (γ / π) / ((ν − ν₀)² + γ²)
    pub fn lorentzian(nu: f64, nu0: f64, gamma: f64) -> f64 {
        gamma / (PI * ((nu - nu0).powi(2) + gamma.powi(2)))
    }

    /// Gaussian (Doppler-broadened) line shape (cm).
    ///
    /// g_D(ν) = (1 / (γ_D √π)) · exp(−((ν − ν₀)/γ_D)²)
    ///
    /// where γ_D is the Doppler half-width (1/e).
    pub fn gaussian(nu: f64, nu0: f64, gamma_d: f64) -> f64 {
        let x = (nu - nu0) / gamma_d;
        (1.0 / (gamma_d * PI.sqrt())) * (-x * x).exp()
    }

    /// Doppler half-width γ_D (cm⁻¹) for a gas at temperature T.
    ///
    /// γ_D = (ν₀ / c) · √(2 k_B T ln2 / m)
    ///
    /// With ν₀ in cm⁻¹, T in K, molar_mass in g/mol.
    pub fn doppler_halfwidth(nu0: f64, temperature_k: f64, molar_mass_g: f64) -> f64 {
        // k_B = 1.380649e-23 J/K,  N_A = 6.02214076e23 /mol
        // k_B / m = R / M where R = 8.314462 J/(mol·K), M in kg/mol
        let r = 8.314462;
        let m_kg = molar_mass_g * 1.0e-3;
        let c_cm_s = 2.99792458e10; // speed of light in cm/s
        nu0 / c_cm_s * (2.0 * r * temperature_k * (2.0_f64.ln()) / m_kg).sqrt()
    }

    /// Voigt profile approximation using the pseudo-Voigt method
    /// (Thompson, Cox & Hastings, 1987).
    ///
    /// The Voigt profile is the convolution of Lorentzian and Gaussian shapes.
    /// The pseudo-Voigt approximation computes the Voigt FWHM and mixing
    /// parameter η, then returns η·L + (1−η)·G, which is accurate to
    /// better than 1% for all parameter regimes.
    ///
    /// * `gamma_l` -- Lorentzian HWHM (cm⁻¹)
    /// * `gamma_d` -- Gaussian HWHM (1/e half-width, cm⁻¹)
    ///
    /// Returns the line shape value (cm).
    pub fn voigt(nu: f64, nu0: f64, gamma_l: f64, gamma_d: f64) -> f64 {
        if gamma_d < 1.0e-30 {
            return Self::lorentzian(nu, nu0, gamma_l);
        }
        if gamma_l < 1.0e-30 {
            return Self::gaussian(nu, nu0, gamma_d);
        }

        // Convert to FWHM values:
        //   Lorentzian FWHM = 2 * gamma_l
        //   Gaussian FWHM = 2 * gamma_d * sqrt(ln2) (from 1/e to HWHM)
        // Actually gamma_d as defined in our gaussian() is the 1/e half-width.
        // The Gaussian FWHM = 2 * gamma_d * sqrt(ln 2).
        let f_l = 2.0 * gamma_l;
        let f_g = 2.0 * gamma_d * (2.0_f64.ln()).sqrt();

        // Thompson-Cox-Hastings approximate Voigt FWHM:
        // f_V^5 = f_G^5 + 2.69269*f_G^4*f_L + 2.42843*f_G^3*f_L^2
        //       + 4.47163*f_G^2*f_L^3 + 0.07842*f_G*f_L^4 + f_L^5
        let fg2 = f_g * f_g;
        let fg3 = fg2 * f_g;
        let fg4 = fg3 * f_g;
        let fg5 = fg4 * f_g;
        let fl2 = f_l * f_l;
        let fl3 = fl2 * f_l;
        let fl4 = fl3 * f_l;
        let fl5 = fl4 * f_l;

        let fv5 = fg5
            + 2.69269 * fg4 * f_l
            + 2.42843 * fg3 * fl2
            + 4.47163 * fg2 * fl3
            + 0.07842 * f_g * fl4
            + fl5;
        let f_v = fv5.powf(0.2);

        if f_v < 1.0e-30 {
            return 0.0;
        }

        // Mixing parameter η (Lorentzian fraction):
        let d = f_l / f_v;
        let eta = 1.36603 * d - 0.47719 * d * d + 0.11116 * d * d * d;
        let eta = eta.clamp(0.0, 1.0);

        // Compute Lorentzian and Gaussian with the Voigt FWHM
        let hwhm_v = f_v / 2.0;
        let l_val = hwhm_v / (PI * ((nu - nu0).powi(2) + hwhm_v * hwhm_v));
        // For the Gaussian component with FWHM = f_v:
        // sigma = f_v / (2*sqrt(2*ln2)), HWHM_G_1e = f_v / (2*sqrt(ln2))
        let sigma_v = f_v / (2.0 * (2.0 * (2.0_f64.ln())).sqrt());
        let g_val = (1.0 / (sigma_v * (2.0 * PI).sqrt()))
            * (-0.5 * ((nu - nu0) / sigma_v).powi(2)).exp();

        eta * l_val + (1.0 - eta) * g_val
    }

    /// Compute absorption spectrum α(ν) (cm⁻¹) for a gas line at given
    /// conditions. Uses the Voigt profile.
    ///
    /// α(ν) = S · N · g_V(ν − ν₀)
    ///
    /// where N = number density (molecule/cm³), S = line strength.
    pub fn absorption_spectrum(
        wavenumbers: &[f64],
        line: &GasLine,
        number_density: f64,
        pressure_atm: f64,
        temperature_k: f64,
        molar_mass_g: f64,
    ) -> Vec<f64> {
        let gamma_l = line.broadened_halfwidth(pressure_atm, 0.0, temperature_k);
        let gamma_d = Self::doppler_halfwidth(line.nu0, temperature_k, molar_mass_g);
        wavenumbers
            .iter()
            .map(|&nu| {
                let g = Self::voigt(nu, line.nu0, gamma_l, gamma_d);
                line.line_strength * number_density * g
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Wavelength Modulation Spectroscopy (WMS)
// ---------------------------------------------------------------------------

/// Wavelength Modulation Spectroscopy processor.
///
/// The laser current is modulated sinusoidally at frequency f_m.  The
/// transmitted signal is demodulated at 1f and 2f harmonics using lock-in
/// detection.  The 2f/1f ratio provides a calibration-free, baseline-immune
/// concentration measurement.
#[derive(Debug, Clone)]
pub struct WmsProcessor {
    /// Modulation frequency (Hz).
    pub modulation_freq_hz: f64,
    /// Modulation depth (wavenumber amplitude, cm⁻¹).
    pub modulation_depth_cm1: f64,
    /// Sample rate (Hz).
    pub sample_rate_hz: f64,
}

impl WmsProcessor {
    /// Create a new WMS processor.
    pub fn new(modulation_freq_hz: f64, modulation_depth_cm1: f64, sample_rate_hz: f64) -> Self {
        Self {
            modulation_freq_hz,
            modulation_depth_cm1,
            sample_rate_hz,
        }
    }

    /// Generate sinusoidal current modulation waveform.
    ///
    /// Returns the wavenumber offset Δν(t) = a · sin(2πf_m t) for `n` samples.
    pub fn generate_modulation(&self, n: usize) -> Vec<f64> {
        let dt = 1.0 / self.sample_rate_hz;
        (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                self.modulation_depth_cm1 * (2.0 * PI * self.modulation_freq_hz * t).sin()
            })
            .collect()
    }

    /// Perform lock-in detection (digital demodulation) at harmonic `n_harm`
    /// of the modulation frequency.
    ///
    /// Returns (in-phase, quadrature) averaged over the signal.
    pub fn lock_in_detect(&self, signal: &[f64], n_harm: u32) -> (f64, f64) {
        let dt = 1.0 / self.sample_rate_hz;
        let omega = 2.0 * PI * self.modulation_freq_hz * n_harm as f64;
        let mut sum_i = 0.0;
        let mut sum_q = 0.0;
        for (k, &s) in signal.iter().enumerate() {
            let t = k as f64 * dt;
            sum_i += s * (omega * t).cos();
            sum_q += s * (omega * t).sin();
        }
        let n = signal.len() as f64;
        (2.0 * sum_i / n, 2.0 * sum_q / n)
    }

    /// Extract the 1f amplitude from a detector signal.
    pub fn detect_1f(&self, signal: &[f64]) -> f64 {
        let (i, q) = self.lock_in_detect(signal, 1);
        (i * i + q * q).sqrt()
    }

    /// Extract the 2f amplitude from a detector signal.
    pub fn detect_2f(&self, signal: &[f64]) -> f64 {
        let (i, q) = self.lock_in_detect(signal, 2);
        (i * i + q * q).sqrt()
    }

    /// Compute the 2f/1f ratio (calibration-free normalised signal).
    pub fn ratio_2f_1f(&self, signal: &[f64]) -> f64 {
        let f1 = self.detect_1f(signal);
        let f2 = self.detect_2f(signal);
        if f1 < 1.0e-30 {
            return 0.0;
        }
        f2 / f1
    }

    /// Approximate 2f signal shape for a Lorentzian line with half-width γ
    /// and modulation depth a.
    ///
    /// For small modulation index m = a/γ, the 2f signal near line center is
    /// proportional to:
    ///   H₂ ∝ m² / (1 + m²)^(5/2)  (peak of second harmonic Fourier coefficient)
    pub fn lorentzian_2f_peak(gamma: f64, modulation_depth: f64) -> f64 {
        let m = modulation_depth / gamma;
        let denom = (1.0 + m * m).powf(2.5);
        2.0 * m * m / denom
    }
}

// ---------------------------------------------------------------------------
// Direct Absorption Spectroscopy (DAS)
// ---------------------------------------------------------------------------

/// Direct Absorption Spectroscopy processor.
///
/// Scans the laser across an absorption feature, fits and removes the
/// baseline, then integrates the peak area for concentration.
#[derive(Debug, Clone)]
pub struct DasProcessor {
    /// Number of passes through the gas cell.
    pub num_passes: usize,
    /// Cell length per pass (cm).
    pub cell_length_cm: f64,
}

impl DasProcessor {
    /// Create a new DAS processor.
    pub fn new(cell_length_cm: f64, num_passes: usize) -> Self {
        Self {
            num_passes,
            cell_length_cm,
        }
    }

    /// Effective optical path length (cm).
    pub fn effective_path_length(&self) -> f64 {
        self.num_passes as f64 * self.cell_length_cm
    }

    /// Compute absorbance from transmitted and reference intensities.
    ///
    /// A(ν) = −ln(I_trans(ν) / I_ref(ν))
    pub fn absorbance(transmitted: &[f64], reference: &[f64]) -> Vec<f64> {
        transmitted
            .iter()
            .zip(reference.iter())
            .map(|(&it, &ir)| {
                if ir <= 0.0 || it <= 0.0 {
                    0.0
                } else {
                    -(it / ir).ln()
                }
            })
            .collect()
    }

    /// Fit and subtract a polynomial baseline of given order.
    ///
    /// Uses edge regions (first and last `edge_frac` of data) to fit the
    /// polynomial, then subtracts it from the entire spectrum.
    pub fn remove_baseline(absorbance: &[f64], order: usize, edge_frac: f64) -> Vec<f64> {
        let n = absorbance.len();
        if n < 2 {
            return absorbance.to_vec();
        }
        let edge = ((n as f64 * edge_frac) as usize).max(1);

        // Collect edge points for fitting
        let mut x_fit = Vec::new();
        let mut y_fit = Vec::new();
        for i in 0..edge {
            x_fit.push(i as f64 / n as f64);
            y_fit.push(absorbance[i]);
        }
        for i in (n - edge)..n {
            x_fit.push(i as f64 / n as f64);
            y_fit.push(absorbance[i]);
        }

        // Least-squares polynomial fit
        let coeffs = polyfit(&x_fit, &y_fit, order);

        // Subtract baseline
        (0..n)
            .map(|i| {
                let x = i as f64 / n as f64;
                absorbance[i] - polyeval(&coeffs, x)
            })
            .collect()
    }

    /// Integrate absorbance peak (trapezoidal rule) over the given wavenumber
    /// grid.  Returns integrated absorbance (cm⁻¹).
    pub fn integrated_absorbance(wavenumbers: &[f64], absorbance: &[f64]) -> f64 {
        if wavenumbers.len() < 2 || absorbance.len() < 2 {
            return 0.0;
        }
        let n = wavenumbers.len().min(absorbance.len());
        let mut sum = 0.0;
        for i in 1..n {
            let dnu = (wavenumbers[i] - wavenumbers[i - 1]).abs();
            sum += 0.5 * (absorbance[i] + absorbance[i - 1]) * dnu;
        }
        sum
    }

    /// Concentration from integrated absorbance.
    ///
    /// C = A_int / (S · L)
    ///
    /// where S is the integrated line strength and L is the path length.
    pub fn concentration_from_integrated(
        &self,
        integrated_abs: f64,
        line_strength: f64,
    ) -> f64 {
        let l = self.effective_path_length();
        if l * line_strength < 1.0e-40 {
            return 0.0;
        }
        integrated_abs / (line_strength * l)
    }
}

// ---------------------------------------------------------------------------
// Concentration Calculator
// ---------------------------------------------------------------------------

/// Converts absorbance measurements to concentrations in various units.
pub struct ConcentrationCalculator;

impl ConcentrationCalculator {
    /// Concentration from absorbance A = α·C·L.
    ///
    /// Returns C in (mol/cm³) if α is in cm⁻¹/(mol/cm³).
    pub fn from_absorbance(absorbance: f64, alpha: f64, path_length_cm: f64) -> f64 {
        if alpha * path_length_cm < 1.0e-40 {
            return 0.0;
        }
        absorbance / (alpha * path_length_cm)
    }

    /// Convert number density (molecule/cm³) to ppm at given temperature
    /// and pressure using the ideal gas law.
    ///
    /// ppm = (N / N_total) × 1e6
    /// N_total = P / (k_B T)
    pub fn number_density_to_ppm(
        number_density: f64,
        temperature_k: f64,
        pressure_atm: f64,
    ) -> f64 {
        let k_b = 1.380649e-23; // J/K
        let pressure_pa = pressure_atm * 101325.0;
        let n_total = pressure_pa / (k_b * temperature_k) * 1.0e-6; // per cm³
        if n_total < 1.0e-10 {
            return 0.0;
        }
        (number_density / n_total) * 1.0e6
    }

    /// Convert ppm to number density (molecule/cm³).
    pub fn ppm_to_number_density(
        ppm: f64,
        temperature_k: f64,
        pressure_atm: f64,
    ) -> f64 {
        let k_b = 1.380649e-23;
        let pressure_pa = pressure_atm * 101325.0;
        let n_total = pressure_pa / (k_b * temperature_k) * 1.0e-6;
        ppm * n_total / 1.0e6
    }

    /// Convert ppm to ppb.
    pub fn ppm_to_ppb(ppm: f64) -> f64 {
        ppm * 1000.0
    }

    /// Convert ppb to ppm.
    pub fn ppb_to_ppm(ppb: f64) -> f64 {
        ppb / 1000.0
    }

    /// Minimum detectable concentration (molecule/cm³) from noise floor.
    ///
    /// C_min = noise_absorbance / (S · L)
    ///
    /// where noise_absorbance is the 1-σ absorbance noise (NEA).
    pub fn minimum_detectable(
        noise_absorbance: f64,
        line_strength: f64,
        path_length_cm: f64,
    ) -> f64 {
        if line_strength * path_length_cm < 1.0e-40 {
            return f64::INFINITY;
        }
        noise_absorbance / (line_strength * path_length_cm)
    }
}

// ---------------------------------------------------------------------------
// Allan Variance
// ---------------------------------------------------------------------------

/// Allan variance analysis for optimal integration time determination.
///
/// The Allan variance σ²(τ) quantifies frequency/signal stability as a
/// function of averaging time τ.  For white noise, σ²(τ) ∝ 1/τ.  When
/// drift dominates, σ²(τ) increases.  The minimum indicates the optimal
/// integration time.
pub struct AllanVariance;

impl AllanVariance {
    /// Compute overlapping Allan variance for a time series at given
    /// averaging factors (multiples of the base sample period).
    ///
    /// Returns (tau, allan_variance) pairs.
    pub fn compute(data: &[f64], base_period: f64, factors: &[usize]) -> Vec<(f64, f64)> {
        let n = data.len();
        let mut result = Vec::new();
        for &m in factors {
            if m == 0 || 2 * m > n {
                continue;
            }
            let tau = m as f64 * base_period;
            // Compute non-overlapping averaged blocks
            let mut avgs = Vec::new();
            let mut i = 0;
            while i + m <= n {
                let sum: f64 = data[i..i + m].iter().sum();
                avgs.push(sum / m as f64);
                i += m;
            }
            if avgs.len() < 2 {
                continue;
            }
            // Allan variance = 0.5 * mean((y_{k+1} - y_k)^2)
            let mut sum_sq = 0.0;
            for k in 0..(avgs.len() - 1) {
                let diff = avgs[k + 1] - avgs[k];
                sum_sq += diff * diff;
            }
            let avar = sum_sq / (2.0 * (avgs.len() - 1) as f64);
            result.push((tau, avar));
        }
        result
    }

    /// Allan deviation (square root of Allan variance).
    pub fn deviation(data: &[f64], base_period: f64, factors: &[usize]) -> Vec<(f64, f64)> {
        Self::compute(data, base_period, factors)
            .into_iter()
            .map(|(tau, avar)| (tau, avar.sqrt()))
            .collect()
    }

    /// Find the optimal integration time (minimum Allan deviation).
    pub fn optimal_integration_time(
        data: &[f64],
        base_period: f64,
        factors: &[usize],
    ) -> Option<(f64, f64)> {
        let adevs = Self::deviation(data, base_period, factors);
        adevs
            .into_iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    }

    /// Determine whether a given averaging factor is in the white noise
    /// regime (σ²(τ) decreasing) or drift regime (increasing).
    pub fn noise_type(avar_prev: f64, avar_curr: f64) -> &'static str {
        if avar_curr < avar_prev {
            "white_noise"
        } else {
            "drift"
        }
    }
}

// ---------------------------------------------------------------------------
// Etalon (Fringe) Suppressor
// ---------------------------------------------------------------------------

/// Fabry-Perot etalon fringe suppression.
///
/// Optical surfaces separated by distance d with refractive index n produce
/// sinusoidal fringes with period Δν = 1/(2nd) in wavenumber space.
#[derive(Debug, Clone)]
pub struct EtalonSuppressor {
    /// Fringe period in cm⁻¹.
    pub fringe_period_cm1: f64,
}

impl EtalonSuppressor {
    /// Create from optical element parameters.
    ///
    /// * `gap_cm` -- physical gap between reflective surfaces (cm).
    /// * `refractive_index` -- refractive index of the medium in the gap.
    pub fn from_gap(gap_cm: f64, refractive_index: f64) -> Self {
        let period = 1.0 / (2.0 * refractive_index * gap_cm);
        Self {
            fringe_period_cm1: period,
        }
    }

    /// Create directly from a known fringe period.
    pub fn new(fringe_period_cm1: f64) -> Self {
        Self { fringe_period_cm1 }
    }

    /// Remove etalon fringes by fitting and subtracting a sinusoidal model.
    ///
    /// Fits the model: s(ν) = a₀ + a₁·cos(ων) + a₂·sin(ων) using
    /// least-squares over all data points, then subtracts the sinusoidal
    /// components (keeping the DC offset a₀).
    ///
    /// `edge_frac` is reserved for future use (e.g. weighting edges).
    pub fn remove_fringes(
        &self,
        wavenumbers: &[f64],
        spectrum: &[f64],
        _edge_frac: f64,
    ) -> Vec<f64> {
        let n = wavenumbers.len().min(spectrum.len());
        if n < 4 {
            return spectrum.to_vec();
        }

        let omega = 2.0 * PI / self.fringe_period_cm1;

        // Least-squares fit: s = a0 + a1*cos(omega*nu) + a2*sin(omega*nu)
        // Normal equations: [sum 1, sum cos, sum sin;
        //                    sum cos, sum cos^2, sum cos*sin;
        //                    sum sin, sum cos*sin, sum sin^2] * [a0;a1;a2] = [sum s; sum s*cos; sum s*sin]
        let mut sc = 0.0;
        let mut ss = 0.0;
        let mut scc = 0.0;
        let mut sss = 0.0;
        let mut scs = 0.0;
        let mut sy = 0.0;
        let mut syc = 0.0;
        let mut sys = 0.0;

        for i in 0..n {
            let phi = omega * wavenumbers[i];
            let c = phi.cos();
            let s = phi.sin();
            let y = spectrum[i];
            sc += c;
            ss += s;
            scc += c * c;
            sss += s * s;
            scs += c * s;
            sy += y;
            syc += y * c;
            sys += y * s;
        }

        let nn = n as f64;
        // Solve 3x3 system
        let mut a = [
            nn, sc, ss,
            sc, scc, scs,
            ss, scs, sss,
        ];
        let mut b = [sy, syc, sys];
        let coeffs = gauss_solve(&mut a, &mut b, 3);

        let a0 = coeffs[0];
        let a1 = coeffs[1];
        let a2 = coeffs[2];

        // Subtract only the sinusoidal components (keep DC / baseline)
        (0..n)
            .map(|i| {
                let phi = omega * wavenumbers[i];
                let fringe = a1 * phi.cos() + a2 * phi.sin();
                spectrum[i] - fringe
            })
            .collect()
    }

    /// Generate synthetic etalon fringes for testing.
    pub fn generate_fringes(
        &self,
        wavenumbers: &[f64],
        amplitude: f64,
        phase: f64,
    ) -> Vec<f64> {
        let omega = 2.0 * PI / self.fringe_period_cm1;
        wavenumbers
            .iter()
            .map(|&nu| amplitude * (omega * nu + phase).sin())
            .collect()
    }
}

// ---------------------------------------------------------------------------
// TEC (Thermoelectric Cooler) PID Controller
// ---------------------------------------------------------------------------

/// PID-based thermoelectric cooler simulation for laser temperature control.
#[derive(Debug, Clone)]
pub struct TecController {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
    /// Temperature setpoint (K).
    pub setpoint_k: f64,
    /// Current temperature (K).
    pub temperature_k: f64,
    /// Thermal time constant (s).
    pub thermal_time_constant: f64,
    /// Ambient temperature (K).
    pub ambient_k: f64,
    /// Max TEC power (W).
    pub max_power_w: f64,
    /// Thermal mass (J/K).
    pub thermal_mass: f64,
    // Internal state
    integral: f64,
    prev_error: f64,
}

impl TecController {
    /// Create a new TEC controller.
    pub fn new(setpoint_k: f64) -> Self {
        Self {
            kp: 5.0,
            ki: 1.0,
            kd: 0.5,
            setpoint_k,
            temperature_k: 293.0,
            thermal_time_constant: 2.0,
            ambient_k: 298.0,
            max_power_w: 10.0,
            thermal_mass: 0.5, // J/K
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    /// Set PID gains.
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Set temperature setpoint.
    pub fn set_setpoint(&mut self, setpoint_k: f64) {
        self.setpoint_k = setpoint_k;
    }

    /// Reset internal PID state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    /// Step the simulation by `dt` seconds.  Returns (temperature, tec_power).
    ///
    /// The thermal model is:
    ///   C · dT/dt = P_tec − (T − T_amb) / R_th
    ///
    /// where R_th = thermal_time_constant / thermal_mass.
    pub fn step(&mut self, dt: f64) -> (f64, f64) {
        let error = self.setpoint_k - self.temperature_k;

        // PID output
        self.integral += error * dt;
        // Anti-windup: clamp integral
        let max_integral = self.max_power_w / self.ki.abs().max(1.0e-10);
        self.integral = self.integral.clamp(-max_integral, max_integral);

        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        self.prev_error = error;

        let mut power = self.kp * error + self.ki * self.integral + self.kd * derivative;
        power = power.clamp(-self.max_power_w, self.max_power_w);

        // Thermal dynamics
        let r_th = self.thermal_time_constant / self.thermal_mass;
        let heat_leak = (self.temperature_k - self.ambient_k) / r_th;
        let d_temp = (power - heat_leak) / self.thermal_mass * dt;
        self.temperature_k += d_temp;

        (self.temperature_k, power)
    }

    /// Run the simulation for `duration` seconds at timestep `dt`.
    /// Returns (times, temperatures, powers).
    pub fn simulate(
        &mut self,
        duration: f64,
        dt: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let n = (duration / dt) as usize;
        let mut times = Vec::with_capacity(n);
        let mut temps = Vec::with_capacity(n);
        let mut powers = Vec::with_capacity(n);
        for k in 0..n {
            let (t, p) = self.step(dt);
            times.push(k as f64 * dt);
            temps.push(t);
            powers.push(p);
        }
        (times, temps, powers)
    }

    /// Temperature stability metric: standard deviation of temperature
    /// over the last `window` samples.
    pub fn stability(temperatures: &[f64], window: usize) -> f64 {
        let n = temperatures.len();
        if window == 0 || n == 0 {
            return f64::INFINITY;
        }
        let start = if n > window { n - window } else { 0 };
        let slice = &temperatures[start..];
        let mean = slice.iter().sum::<f64>() / slice.len() as f64;
        let var = slice.iter().map(|&t| (t - mean).powi(2)).sum::<f64>() / slice.len() as f64;
        var.sqrt()
    }

    /// Current temperature.
    pub fn current_temperature(&self) -> f64 {
        self.temperature_k
    }
}

// ---------------------------------------------------------------------------
// Helper: polynomial fitting
// ---------------------------------------------------------------------------

/// Least-squares polynomial fit of order `order` to (x, y) data.
/// Returns coefficients [a0, a1, ..., a_order] such that
/// p(x) = a0 + a1*x + a2*x² + ...
fn polyfit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len();
    let m = order + 1;
    if n < m {
        return vec![0.0; m];
    }

    // Build normal equations: (Xᵀ X) a = Xᵀ y
    // where X is the Vandermonde matrix.
    let mut ata = vec![0.0; m * m];
    let mut aty = vec![0.0; m];

    for k in 0..n {
        let mut xi = 1.0;
        for i in 0..m {
            let mut xj = 1.0;
            for j in 0..m {
                ata[i * m + j] += xi * xj;
                xj *= x[k];
            }
            aty[i] += xi * y[k];
            xi *= x[k];
        }
    }

    // Solve via Gauss elimination with partial pivoting
    gauss_solve(&mut ata, &mut aty, m)
}

/// Gauss elimination with partial pivoting.
fn gauss_solve(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Forward elimination
    for col in 0..n {
        // Find pivot
        let mut max_val = a[col * n + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let v = a[row * n + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        // Swap rows
        if max_row != col {
            for j in 0..n {
                let tmp = a[col * n + j];
                a[col * n + j] = a[max_row * n + j];
                a[max_row * n + j] = tmp;
            }
            let tmp = b[col];
            b[col] = b[max_row];
            b[max_row] = tmp;
        }
        let pivot = a[col * n + col];
        if pivot.abs() < 1.0e-30 {
            continue;
        }
        // Eliminate below
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * x[j];
        }
        let diag = a[i * n + i];
        x[i] = if diag.abs() > 1.0e-30 { sum / diag } else { 0.0 };
    }
    x
}

/// Evaluate polynomial with coefficients [a0, a1, ...] at point x.
fn polyeval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xi = 1.0;
    for &c in coeffs {
        result += c * xi;
        xi *= x;
    }
    result
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1.0e-6;

    // --- QclConfig ---

    #[test]
    fn test_qcl_config_center_wavenumber() {
        let cfg = QclConfig::co2_4um();
        let nu = cfg.center_wavenumber_cm1();
        // 1e4 / 4.26 ≈ 2347.42 cm⁻¹
        assert!((nu - 2347.42).abs() < 0.1);
    }

    #[test]
    fn test_qcl_config_presets() {
        let co2 = QclConfig::co2_4um();
        assert!(co2.center_wavelength_um > 4.0 && co2.center_wavelength_um < 5.0);
        let co = QclConfig::co_4um();
        assert!(co.center_wavelength_um > 4.5 && co.center_wavelength_um < 5.0);
        let ch4 = QclConfig::ch4_3um();
        assert!(ch4.center_wavelength_um > 3.0 && ch4.center_wavelength_um < 4.0);
    }

    // --- QclController ---

    #[test]
    fn test_controller_power_below_threshold() {
        let ctrl = QclController::new(QclConfig::co2_4um());
        assert_eq!(ctrl.power_at_current(100.0), 0.0);
        assert_eq!(ctrl.power_at_current(250.0), 0.0); // at threshold
    }

    #[test]
    fn test_controller_power_above_threshold() {
        let ctrl = QclController::new(QclConfig::co2_4um());
        let p = ctrl.power_at_current(300.0);
        // 0.5 * (300 - 250) = 25 mW
        assert!((p - 25.0).abs() < TOL);
    }

    #[test]
    fn test_controller_power_clamped() {
        let ctrl = QclController::new(QclConfig::co2_4um());
        let p = ctrl.power_at_current(1000.0);
        assert!((p - 100.0).abs() < TOL); // max_power_mw
    }

    #[test]
    fn test_controller_wavenumber_at_center_current() {
        let ctrl = QclController::new(QclConfig::co2_4um());
        let ic = ctrl.center_current_ma();
        let nu = ctrl.wavenumber_at_current(ic);
        let nu0 = ctrl.config().center_wavenumber_cm1();
        assert!((nu - nu0).abs() < 0.01);
    }

    #[test]
    fn test_controller_wavenumber_tuning() {
        let ctrl = QclController::new(QclConfig::co2_4um());
        let ic = ctrl.center_current_ma();
        let nu_center = ctrl.wavenumber_at_current(ic);
        let nu_up = ctrl.wavenumber_at_current(ic + 10.0);
        // Δν = 0.01 * 10 = 0.1 cm⁻¹
        assert!((nu_up - nu_center - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_controller_thermal_tuning() {
        let mut ctrl = QclController::new(QclConfig::co2_4um());
        let ic = ctrl.center_current_ma();
        let nu_ref = ctrl.wavenumber_at_current(ic);
        ctrl.set_temperature(303.0); // +10 K
        let nu_hot = ctrl.wavenumber_at_current(ic);
        // Δν = -0.06 * 10 = -0.6 cm⁻¹
        assert!((nu_hot - nu_ref + 0.6).abs() < 0.01);
    }

    #[test]
    fn test_controller_scan() {
        let ctrl = QclController::new(QclConfig::co2_4um());
        let (currents, wns, pows) = ctrl.scan(260.0, 300.0, 5);
        assert_eq!(currents.len(), 5);
        assert_eq!(wns.len(), 5);
        assert_eq!(pows.len(), 5);
        // Wavenumbers should be monotonically increasing
        for i in 1..wns.len() {
            assert!(wns[i] > wns[i - 1]);
        }
        // Powers should all be above zero (above threshold)
        for &p in &pows {
            assert!(p > 0.0);
        }
    }

    // --- GasLine ---

    #[test]
    fn test_gas_line_presets() {
        let co2 = GasLine::co2_2349();
        assert!((co2.nu0 - 2349.14).abs() < 0.01);
        let co = GasLine::co_2143();
        assert!((co.nu0 - 2143.27).abs() < 0.01);
        let ch4 = GasLine::ch4_3019();
        assert!((ch4.nu0 - 3019.49).abs() < 0.01);
        let n2o = GasLine::n2o_2224();
        assert!((n2o.nu0 - 2223.76).abs() < 0.01);
        let no = GasLine::no_1876();
        assert!((no.nu0 - 1876.08).abs() < 0.01);
    }

    #[test]
    fn test_broadened_halfwidth() {
        let line = GasLine::co2_2349();
        let gamma = line.broadened_halfwidth(1.0, 0.0, 296.0);
        // At reference conditions, should equal gamma_air
        assert!((gamma - line.gamma_air).abs() < 1.0e-6);
    }

    #[test]
    fn test_broadened_halfwidth_temperature() {
        let line = GasLine::co2_2349();
        let gamma_296 = line.broadened_halfwidth(1.0, 0.0, 296.0);
        let gamma_400 = line.broadened_halfwidth(1.0, 0.0, 400.0);
        // Higher T => narrower pressure broadening
        assert!(gamma_400 < gamma_296);
    }

    // --- Absorption Model ---

    #[test]
    fn test_beer_lambert_no_absorption() {
        let i = AbsorptionModel::beer_lambert(1.0, 0.0, 1.0, 100.0);
        assert!((i - 1.0).abs() < TOL);
    }

    #[test]
    fn test_beer_lambert_some_absorption() {
        let i = AbsorptionModel::beer_lambert(1.0, 0.1, 1.0, 10.0);
        // exp(-0.1 * 1.0 * 10) = exp(-1) ≈ 0.36788
        assert!((i - (-1.0_f64).exp()).abs() < 1.0e-5);
    }

    #[test]
    fn test_absorbance_transmittance_roundtrip() {
        let a = AbsorptionModel::absorbance(0.5, 2.0, 10.0);
        // A = 0.5 * 2.0 * 10 = 10
        assert!((a - 10.0).abs() < TOL);
        let t = AbsorptionModel::transmittance(a);
        assert!((t - (-10.0_f64).exp()).abs() < 1.0e-10);
    }

    #[test]
    fn test_lorentzian_peak() {
        let peak = AbsorptionModel::lorentzian(100.0, 100.0, 0.05);
        // At line center: γ / (π γ²) = 1/(π γ) ≈ 6.366
        assert!((peak - 1.0 / (PI * 0.05)).abs() < 0.01);
    }

    #[test]
    fn test_lorentzian_symmetry() {
        let l_plus = AbsorptionModel::lorentzian(100.1, 100.0, 0.05);
        let l_minus = AbsorptionModel::lorentzian(99.9, 100.0, 0.05);
        assert!((l_plus - l_minus).abs() < 1.0e-10);
    }

    #[test]
    fn test_gaussian_peak() {
        let peak = AbsorptionModel::gaussian(100.0, 100.0, 0.01);
        let expected = 1.0 / (0.01 * PI.sqrt());
        assert!((peak - expected).abs() < 0.01);
    }

    #[test]
    fn test_doppler_halfwidth_positive() {
        let gd = AbsorptionModel::doppler_halfwidth(2349.0, 296.0, 44.0);
        assert!(gd > 0.0);
        assert!(gd < 0.01); // Should be small (order 0.002 cm⁻¹ for CO₂)
    }

    #[test]
    fn test_voigt_reduces_to_lorentzian() {
        // When Doppler width is negligible, Voigt ≈ Lorentzian
        let gamma_l = 0.05;
        let gamma_d = 1.0e-35; // negligible
        let v = AbsorptionModel::voigt(100.0, 100.0, gamma_l, gamma_d);
        let l = AbsorptionModel::lorentzian(100.0, 100.0, gamma_l);
        assert!((v - l).abs() / l < 0.01);
    }

    #[test]
    fn test_voigt_broader_than_components() {
        // Voigt profile should be broader than pure Gaussian, and its FWHM
        // should be at least as wide as the Lorentzian HWHM contribution.
        let gamma_l = 0.05;
        let gamma_d = 0.003;
        let peak = AbsorptionModel::voigt(100.0, 100.0, gamma_l, gamma_d);
        let half_peak = peak / 2.0;

        // Find half-width by scanning
        let mut hw = 0.0;
        for i in 1..2000 {
            let dnu = i as f64 * 0.0001;
            let v = AbsorptionModel::voigt(100.0 + dnu, 100.0, gamma_l, gamma_d);
            if v < half_peak {
                hw = dnu;
                break;
            }
        }
        // The Voigt HWHM should be positive and in a reasonable range.
        // For gamma_l >> gamma_d, it should approach gamma_l.
        assert!(hw > 0.01, "Voigt half-width should be positive, got {}", hw);
        assert!(hw < 0.2, "Voigt half-width should be reasonable, got {}", hw);
        // The Voigt profile at +/- gamma_l should still have significant amplitude
        let v_at_gl = AbsorptionModel::voigt(100.0 + gamma_l, 100.0, gamma_l, gamma_d);
        assert!(v_at_gl > peak * 0.2, "Voigt should have significant wing amplitude");
    }

    #[test]
    fn test_absorption_spectrum() {
        let line = GasLine::co2_2349();
        let wns: Vec<f64> = (0..100).map(|i| 2348.0 + i as f64 * 0.02).collect();
        let spec =
            AbsorptionModel::absorption_spectrum(&wns, &line, 2.5e16, 1.0, 296.0, 44.0);
        assert_eq!(spec.len(), 100);
        // Should have a peak near the line center
        let max_idx = spec
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_wn = wns[max_idx];
        assert!((peak_wn - line.nu0).abs() < 0.2);
    }

    // --- WMS Processor ---

    #[test]
    fn test_wms_modulation_waveform() {
        let wms = WmsProcessor::new(10e3, 0.05, 1e6);
        let mod_wf = wms.generate_modulation(1000);
        assert_eq!(mod_wf.len(), 1000);
        let max = mod_wf.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = mod_wf.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((max - 0.05).abs() < 0.001);
        assert!((min + 0.05).abs() < 0.001);
    }

    #[test]
    fn test_wms_lock_in_1f() {
        let wms = WmsProcessor::new(1000.0, 0.05, 100000.0);
        let n = 10000;
        let dt = 1.0 / 100000.0;
        // Pure 1f signal
        let signal: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * 1000.0 * k as f64 * dt).sin())
            .collect();
        let amp_1f = wms.detect_1f(&signal);
        assert!((amp_1f - 1.0).abs() < 0.05);
        let amp_2f = wms.detect_2f(&signal);
        assert!(amp_2f < 0.1); // Should be small for pure 1f
    }

    #[test]
    fn test_wms_lock_in_2f() {
        let wms = WmsProcessor::new(1000.0, 0.05, 100000.0);
        let n = 10000;
        let dt = 1.0 / 100000.0;
        // Pure 2f signal
        let signal: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * 2000.0 * k as f64 * dt).sin())
            .collect();
        let amp_2f = wms.detect_2f(&signal);
        assert!((amp_2f - 1.0).abs() < 0.05);
    }

    #[test]
    fn test_wms_2f_1f_ratio() {
        let wms = WmsProcessor::new(1000.0, 0.05, 100000.0);
        let n = 10000;
        let dt = 1.0 / 100000.0;
        // Mixed signal with both 1f and 2f components
        let signal: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                2.0 * (2.0 * PI * 1000.0 * t).sin() + 0.5 * (2.0 * PI * 2000.0 * t).sin()
            })
            .collect();
        let ratio = wms.ratio_2f_1f(&signal);
        assert!((ratio - 0.25).abs() < 0.05);
    }

    #[test]
    fn test_lorentzian_2f_peak() {
        let peak = WmsProcessor::lorentzian_2f_peak(0.05, 0.05);
        assert!(peak > 0.0 && peak < 1.0);
        // For m=1: 2*1/(1+1)^2.5 = 2/2^2.5 ≈ 0.3536
        assert!((peak - 2.0 / (2.0_f64).powf(2.5)).abs() < 0.01);
    }

    // --- DAS Processor ---

    #[test]
    fn test_das_effective_path_length() {
        let das = DasProcessor::new(50.0, 10);
        assert!((das.effective_path_length() - 500.0).abs() < TOL);
    }

    #[test]
    fn test_das_absorbance_calculation() {
        let trans = vec![0.5, 0.8, 1.0];
        let refer = vec![1.0, 1.0, 1.0];
        let abs = DasProcessor::absorbance(&trans, &refer);
        assert!((abs[0] - (2.0_f64).ln()).abs() < 1.0e-10);
        assert!((abs[1] - (1.25_f64).ln()).abs() < 1.0e-10);
        assert!(abs[2].abs() < 1.0e-10);
    }

    #[test]
    fn test_das_baseline_removal() {
        // Linear baseline + Gaussian peak
        let n = 100;
        let mut data = Vec::with_capacity(n);
        for i in 0..n {
            let x = i as f64 / n as f64;
            let baseline = 0.5 + 2.0 * x;
            let peak = 1.0 * (-(((x - 0.5) / 0.05).powi(2))).exp();
            data.push(baseline + peak);
        }
        let corrected = DasProcessor::remove_baseline(&data, 1, 0.1);
        // The peak should remain; edges should be near zero
        assert!(corrected[0].abs() < 0.3);
        assert!(corrected[n - 1].abs() < 0.3);
        // Peak near center should still be substantial
        let peak_val = corrected[50];
        assert!(peak_val > 0.5);
    }

    #[test]
    fn test_das_integrated_absorbance() {
        let wns: Vec<f64> = (0..100).map(|i| 100.0 + i as f64 * 0.01).collect();
        let abs: Vec<f64> = wns
            .iter()
            .map(|&nu| {
                let x = (nu - 100.5) / 0.05;
                (-(x * x)).exp()
            })
            .collect();
        let area = DasProcessor::integrated_absorbance(&wns, &abs);
        // Gaussian integral ≈ sqrt(π) * σ ≈ 1.7725 * 0.05 ≈ 0.0886
        assert!((area - 0.0886).abs() < 0.01);
    }

    // --- Concentration Calculator ---

    #[test]
    fn test_concentration_from_absorbance() {
        let c = ConcentrationCalculator::from_absorbance(1.0, 0.5, 100.0);
        // C = 1.0 / (0.5 * 100) = 0.02
        assert!((c - 0.02).abs() < TOL);
    }

    #[test]
    fn test_ppm_ppb_conversion() {
        assert!((ConcentrationCalculator::ppm_to_ppb(1.0) - 1000.0).abs() < TOL);
        assert!((ConcentrationCalculator::ppb_to_ppm(1000.0) - 1.0).abs() < TOL);
    }

    #[test]
    fn test_number_density_ppm_roundtrip() {
        let nd = ConcentrationCalculator::ppm_to_number_density(400.0, 296.0, 1.0);
        let ppm = ConcentrationCalculator::number_density_to_ppm(nd, 296.0, 1.0);
        assert!((ppm - 400.0).abs() < 0.1);
    }

    #[test]
    fn test_minimum_detectable() {
        let c_min = ConcentrationCalculator::minimum_detectable(1.0e-5, 1.0e-19, 1000.0);
        // C_min = 1e-5 / (1e-19 * 1000) = 1e-5 / 1e-16 = 1e11
        assert!((c_min - 1.0e11).abs() / 1.0e11 < 0.01);
    }

    // --- Allan Variance ---

    #[test]
    fn test_allan_variance_white_noise() {
        // For white noise, AVAR(τ) should decrease as 1/τ
        let n = 10000;
        // Deterministic pseudo-random using simple LCG
        let mut rng_state: u64 = 12345;
        let data: Vec<f64> = (0..n)
            .map(|_| {
                rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
                (rng_state >> 33) as f64 / (1u64 << 31) as f64 - 0.5
            })
            .collect();
        let factors = vec![1, 2, 4, 8, 16, 32];
        let avars = AllanVariance::compute(&data, 1.0, &factors);
        // Check that AVAR decreases (at least first few)
        assert!(avars.len() >= 3);
        assert!(avars[1].1 < avars[0].1 * 1.5); // Should roughly halve
    }

    #[test]
    fn test_allan_deviation() {
        let data = vec![1.0, 2.0, 1.0, 2.0, 1.0, 2.0, 1.0, 2.0];
        let adevs = AllanVariance::deviation(&data, 1.0, &[1, 2]);
        assert!(!adevs.is_empty());
        for &(tau, adev) in &adevs {
            assert!(tau > 0.0);
            assert!(adev >= 0.0);
        }
    }

    #[test]
    fn test_noise_type() {
        assert_eq!(AllanVariance::noise_type(1.0, 0.5), "white_noise");
        assert_eq!(AllanVariance::noise_type(0.5, 1.0), "drift");
    }

    // --- Etalon Suppressor ---

    #[test]
    fn test_etalon_from_gap() {
        let es = EtalonSuppressor::from_gap(1.0, 1.0);
        // Period = 1/(2*1*1) = 0.5 cm⁻¹
        assert!((es.fringe_period_cm1 - 0.5).abs() < TOL);
    }

    #[test]
    fn test_etalon_generate_fringes() {
        let es = EtalonSuppressor::new(0.5);
        let wns: Vec<f64> = (0..100).map(|i| 2349.0 + i as f64 * 0.01).collect();
        let fringes = es.generate_fringes(&wns, 0.01, 0.0);
        assert_eq!(fringes.len(), 100);
        // Should be sinusoidal with max ≈ 0.01
        let max = fringes.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!((max - 0.01).abs() < 0.002);
    }

    #[test]
    fn test_etalon_fringe_removal() {
        let es = EtalonSuppressor::new(0.5);
        let wns: Vec<f64> = (0..200).map(|i| 2348.0 + i as f64 * 0.01).collect();
        let fringes = es.generate_fringes(&wns, 0.01, 0.3);

        // Signal = constant + fringes
        let signal: Vec<f64> = fringes.iter().map(|&f| 1.0 + f).collect();
        let cleaned = es.remove_fringes(&wns, &signal, 0.2);

        // After removal, RMS of residual should be much smaller than fringe amplitude
        let rms: f64 = (cleaned.iter().map(|&x| (x - 1.0).powi(2)).sum::<f64>()
            / cleaned.len() as f64)
            .sqrt();
        assert!(rms < 0.005); // Original fringes had amplitude 0.01
    }

    // --- TEC Controller ---

    #[test]
    fn test_tec_settles_to_setpoint() {
        let mut tec = TecController::new(280.0);
        tec.temperature_k = 293.0;
        let (_times, temps, _powers) = tec.simulate(20.0, 0.01);
        // Should settle near setpoint after 20 s
        let final_temp = *temps.last().unwrap();
        assert!((final_temp - 280.0).abs() < 1.0);
    }

    #[test]
    fn test_tec_stability_metric() {
        let temps = vec![280.01, 280.02, 279.99, 280.00, 280.01];
        let stab = TecController::stability(&temps, 5);
        assert!(stab < 0.02); // Very stable
    }

    #[test]
    fn test_tec_pid_reset() {
        let mut tec = TecController::new(280.0);
        tec.step(0.1);
        tec.step(0.1);
        tec.reset();
        assert_eq!(tec.integral, 0.0);
        assert_eq!(tec.prev_error, 0.0);
    }

    // --- Polyfit helpers ---

    #[test]
    fn test_polyfit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0]; // y = 1 + 2x
        let c = polyfit(&x, &y, 1);
        assert!((c[0] - 1.0).abs() < 0.01);
        assert!((c[1] - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_polyeval() {
        let coeffs = vec![1.0, 2.0, 3.0]; // 1 + 2x + 3x²
        let y = polyeval(&coeffs, 2.0);
        // 1 + 4 + 12 = 17
        assert!((y - 17.0).abs() < TOL);
    }
}

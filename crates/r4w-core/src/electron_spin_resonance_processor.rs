//! Electron Spin Resonance (ESR) / Electron Paramagnetic Resonance (EPR) signal processor.
//!
//! This module implements the complete ESR/EPR signal processing pipeline for
//! unpaired electron detection and characterization. It provides:
//!
//! - **Resonance condition** calculation: h*nu = g*mu_B*B0
//! - **Lineshape models**: Lorentzian, Gaussian, Voigt (convolution approx), and first derivatives
//! - **Peak analysis**: peak-to-peak linewidth, g-factor, spin counting via double integral
//! - **Hyperfine splitting**: patterns for N equivalent nuclei (nitrogen triplet, hydrogen multiplet)
//! - **Spin Hamiltonian parameters**: isotropic/axial g and A, zero-field splitting D/E
//! - **Signal processing**: phase-sensitive detection, baseline correction, Savitzky-Golay smoothing
//! - **Spin quantitation**: double integral, standard comparison (DPPH, Mn2+)
//! - **Relaxation times**: T1 from saturation, T2 from linewidth, saturation curves
//!
//! # Physical Background
//!
//! ESR detects unpaired electrons by sweeping the external magnetic field B0 while
//! irradiating the sample with fixed microwave frequency. At resonance, the Zeeman
//! splitting matches the microwave photon energy. Field modulation at ~100 kHz and
//! phase-sensitive detection yield the first derivative of the absorption lineshape.
//!
//! # Example
//!
//! ```
//! use r4w_core::electron_spin_resonance_processor::{
//!     EsrConfig, EsrSpectrum, resonant_field_mt, g_factor_from_field,
//!     lorentzian_derivative, PLANCK_CONSTANT, BOHR_MAGNETON, FREE_ELECTRON_G,
//! };
//!
//! // X-band ESR at 9.5 GHz
//! let config = EsrConfig::x_band();
//! let b_res = resonant_field_mt(config.microwave_frequency_ghz, FREE_ELECTRON_G);
//! assert!((b_res - 339.0).abs() < 1.0); // ~339 mT for free electron
//!
//! let g = g_factor_from_field(config.microwave_frequency_ghz, b_res);
//! assert!((g - FREE_ELECTRON_G).abs() < 0.001);
//! ```

use std::f64::consts::PI;

// ─── Physical constants ─────────────────────────────────────────────────────

/// Planck constant in J*s.
pub const PLANCK_CONSTANT: f64 = 6.62607015e-34;

/// Bohr magneton in J/T.
pub const BOHR_MAGNETON: f64 = 9.2740100783e-24;

/// Free electron g-factor (dimensionless).
pub const FREE_ELECTRON_G: f64 = 2.00231930436256;

/// Electron gyromagnetic ratio in rad/(s*T).
pub const GAMMA_ELECTRON: f64 = 1.76085963023e11;

/// Speed of light in m/s (for frequency-wavelength conversion).
const SPEED_OF_LIGHT: f64 = 2.99792458e8;

// ─── Configuration ──────────────────────────────────────────────────────────

/// ESR spectrometer configuration.
#[derive(Debug, Clone)]
pub struct EsrConfig {
    /// Microwave excitation frequency in GHz.
    pub microwave_frequency_ghz: f64,
    /// Magnetic field sweep range in mT: (start, end).
    pub magnetic_field_range_mt: (f64, f64),
    /// Field modulation amplitude in mT.
    pub modulation_amplitude_mt: f64,
    /// Field modulation frequency in kHz (typically 100 kHz).
    pub modulation_frequency_khz: f64,
    /// Number of field points in the sweep.
    pub num_points: usize,
}

impl EsrConfig {
    /// Standard X-band configuration (~9.5 GHz).
    pub fn x_band() -> Self {
        Self {
            microwave_frequency_ghz: 9.5,
            magnetic_field_range_mt: (300.0, 380.0),
            modulation_amplitude_mt: 0.1,
            modulation_frequency_khz: 100.0,
            num_points: 1024,
        }
    }

    /// Q-band configuration (~34 GHz).
    pub fn q_band() -> Self {
        Self {
            microwave_frequency_ghz: 34.0,
            magnetic_field_range_mt: (1100.0, 1300.0),
            modulation_amplitude_mt: 0.05,
            modulation_frequency_khz: 100.0,
            num_points: 1024,
        }
    }

    /// W-band configuration (~94 GHz).
    pub fn w_band() -> Self {
        Self {
            microwave_frequency_ghz: 94.0,
            magnetic_field_range_mt: (3300.0, 3400.0),
            modulation_amplitude_mt: 0.02,
            modulation_frequency_khz: 100.0,
            num_points: 1024,
        }
    }

    /// Generate the magnetic field axis in mT.
    pub fn field_axis(&self) -> Vec<f64> {
        let (b_start, b_end) = self.magnetic_field_range_mt;
        let n = self.num_points;
        if n <= 1 {
            return vec![b_start];
        }
        (0..n)
            .map(|i| b_start + (b_end - b_start) * (i as f64) / ((n - 1) as f64))
            .collect()
    }
}

// ─── ESR Spectrum ───────────────────────────────────────────────────────────

/// An ESR spectrum: arrays of magnetic field (mT) and signal (first derivative of absorption).
#[derive(Debug, Clone)]
pub struct EsrSpectrum {
    /// Magnetic field values in mT.
    pub magnetic_field_mt: Vec<f64>,
    /// Signal values (first derivative of absorption by default).
    pub signal: Vec<f64>,
}

impl EsrSpectrum {
    /// Create a new spectrum from field and signal arrays.
    pub fn new(magnetic_field_mt: Vec<f64>, signal: Vec<f64>) -> Self {
        assert_eq!(magnetic_field_mt.len(), signal.len(), "field and signal must have same length");
        Self { magnetic_field_mt, signal }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.magnetic_field_mt.len()
    }

    /// Check if spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.magnetic_field_mt.is_empty()
    }

    /// Convert first-derivative signal to absorption by integration (trapezoidal rule).
    pub fn to_absorption(&self) -> Vec<f64> {
        if self.signal.len() < 2 {
            return self.signal.clone();
        }
        let mut absorption = vec![0.0; self.signal.len()];
        for i in 1..self.signal.len() {
            let db = self.magnetic_field_mt[i] - self.magnetic_field_mt[i - 1];
            absorption[i] = absorption[i - 1] + 0.5 * (self.signal[i] + self.signal[i - 1]) * db;
        }
        absorption
    }

    /// Convert first-derivative signal to second derivative by numerical differentiation.
    pub fn to_second_derivative(&self) -> Vec<f64> {
        if self.signal.len() < 3 {
            return vec![0.0; self.signal.len()];
        }
        let n = self.signal.len();
        let mut deriv = vec![0.0; n];
        // Central difference for interior points
        for i in 1..n - 1 {
            let db = self.magnetic_field_mt[i + 1] - self.magnetic_field_mt[i - 1];
            if db.abs() > 1e-30 {
                deriv[i] = (self.signal[i + 1] - self.signal[i - 1]) / db;
            }
        }
        // Forward/backward difference for endpoints
        let db0 = self.magnetic_field_mt[1] - self.magnetic_field_mt[0];
        if db0.abs() > 1e-30 {
            deriv[0] = (self.signal[1] - self.signal[0]) / db0;
        }
        let dbn = self.magnetic_field_mt[n - 1] - self.magnetic_field_mt[n - 2];
        if dbn.abs() > 1e-30 {
            deriv[n - 1] = (self.signal[n - 1] - self.signal[n - 2]) / dbn;
        }
        deriv
    }

    /// Compute the double integral of the first-derivative signal.
    /// The double integral is proportional to the number of spins.
    pub fn double_integral(&self) -> f64 {
        let absorption = self.to_absorption();
        if absorption.len() < 2 {
            return 0.0;
        }
        let mut integral = 0.0;
        for i in 1..absorption.len() {
            let db = self.magnetic_field_mt[i] - self.magnetic_field_mt[i - 1];
            integral += 0.5 * (absorption[i] + absorption[i - 1]) * db;
        }
        integral
    }

    /// Estimate signal-to-noise ratio.
    /// Uses the peak-to-peak amplitude divided by the RMS noise in the baseline region.
    /// `noise_fraction` is the fraction of points at each end used for noise estimation.
    pub fn snr(&self, noise_fraction: f64) -> f64 {
        if self.signal.is_empty() {
            return 0.0;
        }
        let n = self.signal.len();
        let noise_points = ((n as f64) * noise_fraction).max(2.0) as usize;
        let noise_points = noise_points.min(n / 4);

        // Collect noise from first and last noise_points
        let mut noise_vals: Vec<f64> = Vec::new();
        for i in 0..noise_points {
            noise_vals.push(self.signal[i]);
        }
        for i in (n - noise_points)..n {
            noise_vals.push(self.signal[i]);
        }

        let mean = noise_vals.iter().sum::<f64>() / noise_vals.len() as f64;
        let variance = noise_vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>()
            / noise_vals.len() as f64;
        let rms_noise = variance.sqrt();

        if rms_noise < 1e-30 {
            return f64::INFINITY;
        }

        let pp = peak_to_peak_amplitude(&self.signal);
        pp / rms_noise
    }
}

// ─── Resonance Condition ────────────────────────────────────────────────────

/// Calculate the resonant magnetic field in mT for a given microwave frequency and g-factor.
///
/// B0 = h * nu / (g * mu_B), where nu is in Hz, B0 in Tesla.
/// Returns B0 in milliTesla.
pub fn resonant_field_mt(frequency_ghz: f64, g_factor: f64) -> f64 {
    let freq_hz = frequency_ghz * 1e9;
    let b0_tesla = PLANCK_CONSTANT * freq_hz / (g_factor * BOHR_MAGNETON);
    b0_tesla * 1000.0 // convert to mT
}

/// Calculate the g-factor from microwave frequency and resonant field.
///
/// g = h * nu / (mu_B * B0)
pub fn g_factor_from_field(frequency_ghz: f64, field_mt: f64) -> f64 {
    let freq_hz = frequency_ghz * 1e9;
    let b0_tesla = field_mt / 1000.0;
    PLANCK_CONSTANT * freq_hz / (BOHR_MAGNETON * b0_tesla)
}

/// Convert microwave frequency in GHz to wavelength in mm.
pub fn frequency_to_wavelength_mm(frequency_ghz: f64) -> f64 {
    (SPEED_OF_LIGHT / (frequency_ghz * 1e9)) * 1000.0
}

// ─── Lineshape Models ───────────────────────────────────────────────────────

/// Lorentzian absorption lineshape.
///
/// A(B) = A0 / (1 + ((B - B0) / delta_B)^2)
///
/// - `b`: magnetic field in mT
/// - `b0`: center field in mT
/// - `delta_b`: half-width at half-maximum (HWHM) in mT
/// - `amplitude`: peak amplitude A0
pub fn lorentzian_absorption(b: f64, b0: f64, delta_b: f64, amplitude: f64) -> f64 {
    let x = (b - b0) / delta_b;
    amplitude / (1.0 + x * x)
}

/// First derivative of Lorentzian absorption (field-modulated ESR signal).
///
/// dA/dB = -2 * A0 * (B - B0) / (delta_B^2 * (1 + ((B-B0)/delta_B)^2)^2)
pub fn lorentzian_derivative(b: f64, b0: f64, delta_b: f64, amplitude: f64) -> f64 {
    let x = (b - b0) / delta_b;
    let denom = (1.0 + x * x).powi(2);
    -2.0 * amplitude * x / (delta_b * denom)
}

/// Gaussian absorption lineshape.
///
/// A(B) = A0 * exp(-((B - B0)^2) / (2 * sigma^2))
///
/// - `sigma`: standard deviation in mT (FWHM = 2*sqrt(2*ln(2))*sigma ~ 2.3548*sigma)
pub fn gaussian_absorption(b: f64, b0: f64, sigma: f64, amplitude: f64) -> f64 {
    let x = (b - b0) / sigma;
    amplitude * (-0.5 * x * x).exp()
}

/// First derivative of Gaussian absorption.
///
/// dA/dB = -A0 * (B - B0) / sigma^2 * exp(-((B-B0)^2)/(2*sigma^2))
pub fn gaussian_derivative(b: f64, b0: f64, sigma: f64, amplitude: f64) -> f64 {
    let x = (b - b0) / sigma;
    -amplitude * x / sigma * (-0.5 * x * x).exp()
}

/// Voigt absorption lineshape (pseudo-Voigt approximation).
///
/// V(B) = eta * L(B) + (1 - eta) * G(B)
///
/// The mixing parameter eta is approximated from the Lorentzian (gamma_l) and
/// Gaussian (gamma_g) FWHM contributions using the Thompson-Cox-Hastings formula.
///
/// - `gamma_l`: Lorentzian HWHM in mT
/// - `gamma_g`: Gaussian sigma in mT
/// - `eta_override`: if Some(eta), use this mixing parameter directly (0=pure Gaussian, 1=pure Lorentzian)
pub fn voigt_absorption(
    b: f64,
    b0: f64,
    gamma_l: f64,
    gamma_g: f64,
    amplitude: f64,
    eta_override: Option<f64>,
) -> f64 {
    let eta = match eta_override {
        Some(e) => e.clamp(0.0, 1.0),
        None => {
            // Thompson-Cox-Hastings pseudo-Voigt mixing parameter
            let fwhm_l = 2.0 * gamma_l;
            let fwhm_g = 2.3548 * gamma_g; // 2*sqrt(2*ln2)*sigma
            let f5 = fwhm_g.powi(5)
                + 2.69269 * fwhm_g.powi(4) * fwhm_l
                + 2.42843 * fwhm_g.powi(3) * fwhm_l.powi(2)
                + 4.47163 * fwhm_g.powi(2) * fwhm_l.powi(3)
                + 0.07842 * fwhm_g * fwhm_l.powi(4)
                + fwhm_l.powi(5);
            let fwhm_v = f5.powf(0.2);
            if fwhm_v.abs() < 1e-30 {
                0.5
            } else {
                let ratio = fwhm_l / fwhm_v;
                1.36603 * ratio - 0.47719 * ratio * ratio + 0.11116 * ratio * ratio * ratio
            }
        }
    };
    let lor = lorentzian_absorption(b, b0, gamma_l, amplitude);
    let gau = gaussian_absorption(b, b0, gamma_g, amplitude);
    eta * lor + (1.0 - eta) * gau
}

/// First derivative of pseudo-Voigt lineshape.
pub fn voigt_derivative(
    b: f64,
    b0: f64,
    gamma_l: f64,
    gamma_g: f64,
    amplitude: f64,
    eta: f64,
) -> f64 {
    let eta_c = eta.clamp(0.0, 1.0);
    let lor_d = lorentzian_derivative(b, b0, gamma_l, amplitude);
    let gau_d = gaussian_derivative(b, b0, gamma_g, amplitude);
    eta_c * lor_d + (1.0 - eta_c) * gau_d
}

// ─── Peak Analysis ──────────────────────────────────────────────────────────

/// Find peak-to-peak amplitude of a signal (max - min).
pub fn peak_to_peak_amplitude(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let mut max_val = f64::NEG_INFINITY;
    let mut min_val = f64::INFINITY;
    for &v in signal {
        if v > max_val { max_val = v; }
        if v < min_val { min_val = v; }
    }
    max_val - min_val
}

/// Find the peak-to-peak linewidth (delta_Bpp) from a first-derivative spectrum.
///
/// delta_Bpp is the field separation between the maximum and minimum of the
/// first derivative signal.
pub fn peak_to_peak_linewidth(field_mt: &[f64], derivative_signal: &[f64]) -> f64 {
    if field_mt.len() != derivative_signal.len() || field_mt.len() < 2 {
        return 0.0;
    }
    let mut max_idx = 0;
    let mut min_idx = 0;
    let mut max_val = f64::NEG_INFINITY;
    let mut min_val = f64::INFINITY;
    for (i, &v) in derivative_signal.iter().enumerate() {
        if v > max_val {
            max_val = v;
            max_idx = i;
        }
        if v < min_val {
            min_val = v;
            min_idx = i;
        }
    }
    (field_mt[max_idx] - field_mt[min_idx]).abs()
}

/// Determine the center field (B0) from a first-derivative spectrum.
///
/// The center field is at the zero crossing between the max and min of the derivative.
pub fn center_field_mt(field_mt: &[f64], derivative_signal: &[f64]) -> f64 {
    if field_mt.len() != derivative_signal.len() || field_mt.len() < 2 {
        return 0.0;
    }
    // Find max and min positions
    let mut max_idx = 0;
    let mut min_idx = 0;
    let mut max_val = f64::NEG_INFINITY;
    let mut min_val = f64::INFINITY;
    for (i, &v) in derivative_signal.iter().enumerate() {
        if v > max_val { max_val = v; max_idx = i; }
        if v < min_val { min_val = v; min_idx = i; }
    }
    let (lo, hi) = if max_idx < min_idx {
        (max_idx, min_idx)
    } else {
        (min_idx, max_idx)
    };

    // Find zero crossing between lo and hi
    for i in lo..hi {
        if derivative_signal[i] * derivative_signal[i + 1] <= 0.0 {
            // Linear interpolation to find zero crossing
            let d0 = derivative_signal[i];
            let d1 = derivative_signal[i + 1];
            if (d1 - d0).abs() < 1e-30 {
                return 0.5 * (field_mt[i] + field_mt[i + 1]);
            }
            let frac = -d0 / (d1 - d0);
            return field_mt[i] + frac * (field_mt[i + 1] - field_mt[i]);
        }
    }
    // Fallback: midpoint between max and min
    0.5 * (field_mt[max_idx] + field_mt[min_idx])
}

/// Determine g-factor from a first-derivative ESR spectrum.
pub fn g_factor_from_spectrum(
    frequency_ghz: f64,
    field_mt: &[f64],
    derivative_signal: &[f64],
) -> f64 {
    let b0 = center_field_mt(field_mt, derivative_signal);
    g_factor_from_field(frequency_ghz, b0)
}

// ─── Hyperfine Splitting ────────────────────────────────────────────────────

/// Nuclear spin quantum number.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NuclearSpin {
    /// I = 1/2 (e.g., 1H, 13C, 19F, 31P)
    Half,
    /// I = 1 (e.g., 14N, 2H)
    One,
    /// I = 3/2 (e.g., 11B, 35Cl, 37Cl)
    ThreeHalves,
    /// I = 5/2 (e.g., 55Mn, 17O)
    FiveHalves,
}

impl NuclearSpin {
    /// Get the spin quantum number as f64.
    pub fn value(&self) -> f64 {
        match self {
            NuclearSpin::Half => 0.5,
            NuclearSpin::One => 1.0,
            NuclearSpin::ThreeHalves => 1.5,
            NuclearSpin::FiveHalves => 2.5,
        }
    }
}

/// Calculate the number of hyperfine lines for N equivalent nuclei with spin I.
///
/// Number of lines = 2*N*I + 1
pub fn hyperfine_line_count(n_nuclei: usize, spin: NuclearSpin) -> usize {
    let val = 2.0 * (n_nuclei as f64) * spin.value() + 1.0;
    val.round() as usize
}

/// Calculate relative intensities for N equivalent nuclei with spin I = 1/2.
///
/// Binomial coefficients: C(N, k) for k = 0..N
pub fn binomial_intensities(n_nuclei: usize) -> Vec<f64> {
    let n = n_nuclei;
    let mut coeffs = vec![0.0; n + 1];
    coeffs[0] = 1.0;
    for i in 1..=n {
        // C(n, i) = C(n, i-1) * (n - i + 1) / i
        coeffs[i] = coeffs[i - 1] * ((n - i + 1) as f64) / (i as f64);
    }
    coeffs
}

/// Calculate relative intensities for N equivalent nuclei with spin I = 1.
///
/// For N nuclei with I=1, intensities follow the trinomial distribution.
/// Each nucleus contributes m_I = -1, 0, +1 with equal probability.
pub fn trinomial_intensities(n_nuclei: usize) -> Vec<f64> {
    // Convolve {1, 1, 1} with itself N times
    let mut current = vec![1.0_f64];
    let kernel = vec![1.0, 1.0, 1.0];
    for _ in 0..n_nuclei {
        let new_len = current.len() + kernel.len() - 1;
        let mut next = vec![0.0; new_len];
        for (i, &c) in current.iter().enumerate() {
            for (j, &k) in kernel.iter().enumerate() {
                next[i + j] += c * k;
            }
        }
        current = next;
    }
    current
}

/// Generate hyperfine splitting pattern.
///
/// Returns (field_offsets_mt, relative_intensities) for lines centered at 0.
///
/// - `coupling_constant_mt`: hyperfine coupling constant A in mT
/// - `n_nuclei`: number of equivalent nuclei
/// - `spin`: nuclear spin
pub fn hyperfine_pattern(
    coupling_constant_mt: f64,
    n_nuclei: usize,
    spin: NuclearSpin,
) -> (Vec<f64>, Vec<f64>) {
    let num_lines = hyperfine_line_count(n_nuclei, spin);
    let intensities = match spin {
        NuclearSpin::Half => binomial_intensities(n_nuclei),
        NuclearSpin::One => trinomial_intensities(n_nuclei),
        _ => {
            // General case: uniform intensities as approximation
            vec![1.0; num_lines]
        }
    };

    let n_i = intensities.len();
    let center = (n_i as f64 - 1.0) / 2.0;
    let offsets: Vec<f64> = (0..n_i)
        .map(|i| (i as f64 - center) * coupling_constant_mt)
        .collect();

    (offsets, intensities)
}

/// Generate a simulated ESR spectrum with hyperfine splitting.
///
/// Creates a first-derivative spectrum with the specified hyperfine pattern.
pub fn simulate_hyperfine_spectrum(
    config: &EsrConfig,
    center_field_mt: f64,
    coupling_constant_mt: f64,
    n_nuclei: usize,
    spin: NuclearSpin,
    linewidth_mt: f64,
    amplitude: f64,
) -> EsrSpectrum {
    let field = config.field_axis();
    let (offsets, intensities) = hyperfine_pattern(coupling_constant_mt, n_nuclei, spin);
    let max_intensity = intensities.iter().cloned().fold(0.0_f64, f64::max);

    let signal: Vec<f64> = field
        .iter()
        .map(|&b| {
            let mut total = 0.0;
            for (offset, intensity) in offsets.iter().zip(intensities.iter()) {
                let b0 = center_field_mt + offset;
                let rel = intensity / max_intensity;
                total += rel * lorentzian_derivative(b, b0, linewidth_mt, amplitude);
            }
            total
        })
        .collect();

    EsrSpectrum::new(field, signal)
}

// ─── Spin Hamiltonian Parameters ────────────────────────────────────────────

/// Spin Hamiltonian parameters for an ESR center.
#[derive(Debug, Clone)]
pub struct SpinHamiltonian {
    /// Isotropic g-factor (average of principal values).
    pub g_iso: f64,
    /// Axial g-factor components: (g_parallel, g_perpendicular).
    /// None if isotropic.
    pub g_axial: Option<(f64, f64)>,
    /// Isotropic hyperfine coupling constant in mT.
    pub a_iso_mt: f64,
    /// Axial hyperfine components: (A_parallel, A_perpendicular) in mT.
    /// None if isotropic.
    pub a_axial_mt: Option<(f64, f64)>,
    /// Zero-field splitting parameter D in mT (for S > 1/2).
    pub d_mt: f64,
    /// Zero-field splitting parameter E in mT (rhombic distortion).
    pub e_mt: f64,
    /// Total electron spin quantum number S.
    pub spin_s: f64,
}

impl SpinHamiltonian {
    /// Create an isotropic spin-1/2 system.
    pub fn isotropic(g: f64, a_mt: f64) -> Self {
        Self {
            g_iso: g,
            g_axial: None,
            a_iso_mt: a_mt,
            a_axial_mt: None,
            d_mt: 0.0,
            e_mt: 0.0,
            spin_s: 0.5,
        }
    }

    /// Create an axially symmetric system.
    pub fn axial(g_par: f64, g_perp: f64, a_par_mt: f64, a_perp_mt: f64) -> Self {
        let g_iso = (g_par + 2.0 * g_perp) / 3.0;
        let a_iso = (a_par_mt + 2.0 * a_perp_mt) / 3.0;
        Self {
            g_iso,
            g_axial: Some((g_par, g_perp)),
            a_iso_mt: a_iso,
            a_axial_mt: Some((a_par_mt, a_perp_mt)),
            d_mt: 0.0,
            e_mt: 0.0,
            spin_s: 0.5,
        }
    }

    /// Create a system with zero-field splitting (S > 1/2).
    pub fn high_spin(g_iso: f64, spin_s: f64, d_mt: f64, e_mt: f64) -> Self {
        Self {
            g_iso,
            g_axial: None,
            a_iso_mt: 0.0,
            a_axial_mt: None,
            d_mt,
            e_mt,
            spin_s,
        }
    }

    /// Effective g-factor at angle theta (radians) from the symmetry axis.
    /// Only valid for axial symmetry.
    pub fn g_effective(&self, theta: f64) -> f64 {
        match self.g_axial {
            Some((g_par, g_perp)) => {
                let cos_t = theta.cos();
                let sin_t = theta.sin();
                (g_par * g_par * cos_t * cos_t + g_perp * g_perp * sin_t * sin_t).sqrt()
            }
            None => self.g_iso,
        }
    }

    /// Effective hyperfine coupling at angle theta (radians) from the symmetry axis.
    pub fn a_effective_mt(&self, theta: f64) -> f64 {
        match self.a_axial_mt {
            Some((a_par, a_perp)) => {
                let cos_t = theta.cos();
                let sin_t = theta.sin();
                (a_par * a_par * cos_t * cos_t + a_perp * a_perp * sin_t * sin_t).sqrt()
            }
            None => self.a_iso_mt,
        }
    }

    /// Rhombicity parameter E/D.
    pub fn rhombicity(&self) -> f64 {
        if self.d_mt.abs() < 1e-30 {
            0.0
        } else {
            self.e_mt / self.d_mt
        }
    }
}

// ─── Signal Processing ──────────────────────────────────────────────────────

/// Phase-sensitive detection (lock-in amplifier simulation).
///
/// Multiply signal by reference at modulation frequency and low-pass filter.
/// - `signal`: input time-domain signal
/// - `sample_rate_hz`: sampling rate
/// - `reference_freq_hz`: lock-in reference frequency
/// - `time_constant_s`: low-pass filter time constant
pub fn phase_sensitive_detection(
    signal: &[f64],
    sample_rate_hz: f64,
    reference_freq_hz: f64,
    time_constant_s: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    let dt = 1.0 / sample_rate_hz;
    let alpha = dt / (time_constant_s + dt);

    // Demodulate: multiply by cos(2*pi*f*t)
    let mut demod: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 * dt;
            signal[i] * (2.0 * PI * reference_freq_hz * t).cos() * 2.0
        })
        .collect();

    // Single-pole IIR low-pass filter
    for i in 1..n {
        demod[i] = alpha * demod[i] + (1.0 - alpha) * demod[i - 1];
    }

    demod
}

/// Polynomial baseline correction.
///
/// Fits a polynomial of specified degree to the baseline regions at the edges
/// and subtracts it from the signal.
///
/// - `signal`: input signal
/// - `degree`: polynomial degree (1 = linear, 2 = quadratic, etc.)
/// - `edge_fraction`: fraction of points at each end used for baseline fitting
pub fn baseline_correction(signal: &[f64], degree: usize, edge_fraction: f64) -> Vec<f64> {
    let n = signal.len();
    if n < 4 {
        return signal.to_vec();
    }

    let edge_pts = ((n as f64) * edge_fraction).max(2.0) as usize;
    let edge_pts = edge_pts.min(n / 4);

    // Collect baseline points (indices and values)
    let mut x_pts = Vec::new();
    let mut y_pts = Vec::new();

    for i in 0..edge_pts {
        x_pts.push(i as f64 / n as f64);
        y_pts.push(signal[i]);
    }
    for i in (n - edge_pts)..n {
        x_pts.push(i as f64 / n as f64);
        y_pts.push(signal[i]);
    }

    // Fit polynomial using least squares (Vandermonde approach)
    let m = x_pts.len();
    let d = degree + 1;
    let d = d.min(m); // can't have more coefficients than points

    // Build normal equations: A^T * A * c = A^T * y
    let mut ata = vec![0.0; d * d];
    let mut aty = vec![0.0; d];

    for k in 0..m {
        let mut xi_powers = vec![1.0; d];
        for j in 1..d {
            xi_powers[j] = xi_powers[j - 1] * x_pts[k];
        }
        for i in 0..d {
            for j in 0..d {
                ata[i * d + j] += xi_powers[i] * xi_powers[j];
            }
            aty[i] += xi_powers[i] * y_pts[k];
        }
    }

    // Solve via Gaussian elimination with partial pivoting
    let coeffs = solve_linear_system(&ata, &aty, d);

    // Subtract polynomial from signal
    let mut corrected = signal.to_vec();
    for i in 0..n {
        let x = i as f64 / n as f64;
        let mut baseline = 0.0;
        let mut x_pow = 1.0;
        for c in &coeffs {
            baseline += c * x_pow;
            x_pow *= x;
        }
        corrected[i] -= baseline;
    }

    corrected
}

/// Solve a linear system Ax = b via Gaussian elimination with partial pivoting.
fn solve_linear_system(a_flat: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut a = vec![0.0; n * n];
    a.copy_from_slice(&a_flat[..n * n]);
    let mut rhs = b[..n].to_vec();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = a[col * n + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for j in 0..n {
                a.swap(col * n + j, max_row * n + j);
            }
            rhs.swap(col, max_row);
        }

        let pivot = a[col * n + col];
        if pivot.abs() < 1e-15 {
            continue;
        }

        // Eliminate below
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            rhs[row] -= factor * rhs[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = rhs[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * x[j];
        }
        let diag = a[i * n + i];
        if diag.abs() > 1e-15 {
            x[i] = sum / diag;
        }
    }

    x
}

/// Savitzky-Golay smoothing filter.
///
/// - `signal`: input signal
/// - `window_size`: must be odd, >= 3
/// - `poly_order`: polynomial order (typically 2 or 3), must be < window_size
pub fn savitzky_golay_smooth(signal: &[f64], window_size: usize, poly_order: usize) -> Vec<f64> {
    let n = signal.len();
    if n < window_size || window_size < 3 || window_size % 2 == 0 || poly_order >= window_size {
        return signal.to_vec();
    }

    let half = (window_size / 2) as i64;
    let d = poly_order + 1;

    // Precompute SG coefficients for the smoothing case (0th derivative at center)
    // Build Vandermonde-like matrix for positions -half..half
    let m = window_size;
    let mut ata = vec![0.0; d * d];
    let mut atb_col = vec![0.0; d]; // We want row 0 of (A^T A)^-1 A^T, which are the SG coefficients

    // Normal equations for each basis function
    for k in 0..m {
        let xk = (k as i64 - half) as f64;
        let mut xi_powers = vec![1.0; d];
        for j in 1..d {
            xi_powers[j] = xi_powers[j - 1] * xk;
        }
        for i in 0..d {
            for j in 0..d {
                ata[i * d + j] += xi_powers[i] * xi_powers[j];
            }
        }
    }

    // Compute (A^T A)^-1 first column (since we want smoothing = 0th derivative)
    let mut identity_col = vec![0.0; d];
    identity_col[0] = 1.0;
    let inv_col = solve_linear_system(&ata, &identity_col, d);

    // Compute SG coefficients: c_k = sum_j (A^T A)^-1_{0,j} * x_k^j
    let mut sg_coeffs = vec![0.0; m];
    for k in 0..m {
        let xk = (k as i64 - half) as f64;
        let mut x_pow = 1.0;
        for j in 0..d {
            sg_coeffs[k] += inv_col[j] * x_pow;
            x_pow *= xk;
        }
    }

    // Apply filter
    let mut result = signal.to_vec();
    for i in (half as usize)..(n - half as usize) {
        let mut val = 0.0;
        for k in 0..m {
            let idx = (i as i64 - half + k as i64) as usize;
            val += sg_coeffs[k] * signal[idx];
        }
        result[i] = val;
    }

    result
}

// ─── Spin Quantitation ──────────────────────────────────────────────────────

/// Reference standard for spin quantitation.
#[derive(Debug, Clone)]
pub struct SpinStandard {
    /// Name of the standard.
    pub name: String,
    /// Number of spins in the standard.
    pub num_spins: f64,
    /// Double integral of the standard's ESR spectrum.
    pub double_integral: f64,
    /// g-factor of the standard.
    pub g_factor: f64,
}

impl SpinStandard {
    /// DPPH (2,2-diphenyl-1-picrylhydrazyl) standard.
    /// Typical values for a standard sample with known spin count.
    pub fn dpph(num_spins: f64, double_integral: f64) -> Self {
        Self {
            name: "DPPH".to_string(),
            num_spins,
            double_integral,
            g_factor: 2.0036,
        }
    }

    /// Mn2+ standard (6 lines, S=5/2, I=5/2 for 55Mn).
    pub fn mn2_plus(num_spins: f64, double_integral: f64) -> Self {
        Self {
            name: "Mn2+".to_string(),
            num_spins,
            double_integral,
            g_factor: 2.0010,
        }
    }
}

/// Calculate number of spins from double integral, comparing with a standard.
///
/// N_sample = N_standard * (DI_sample / DI_standard) * (g_standard / g_sample)^2
pub fn calculate_num_spins(
    sample_double_integral: f64,
    sample_g: f64,
    standard: &SpinStandard,
) -> f64 {
    if standard.double_integral.abs() < 1e-30 {
        return 0.0;
    }
    standard.num_spins
        * (sample_double_integral / standard.double_integral)
        * (standard.g_factor / sample_g).powi(2)
}

/// Calculate spin concentration from number of spins and sample volume.
///
/// Returns concentration in spins/mL.
pub fn spin_concentration(num_spins: f64, volume_ml: f64) -> f64 {
    if volume_ml.abs() < 1e-30 {
        return 0.0;
    }
    num_spins / volume_ml
}

/// Convert spins/mL to molar concentration.
///
/// Returns concentration in mol/L (M).
pub fn spins_to_molar(spins_per_ml: f64) -> f64 {
    let avogadro = 6.02214076e23;
    (spins_per_ml * 1000.0) / avogadro // * 1000 to convert mL to L
}

// ─── Relaxation Times ───────────────────────────────────────────────────────

/// Calculate T2 (spin-spin relaxation time) from Lorentzian linewidth.
///
/// T2 = 1 / (gamma_e * delta_Bpp * sqrt(3) / 2)
///
/// For Lorentzian line: HWHM = sqrt(3) * delta_Bpp / 2, and T2 = 1/(gamma * HWHM).
///
/// - `delta_bpp_mt`: peak-to-peak linewidth in mT
///
/// Returns T2 in seconds.
pub fn t2_from_linewidth(delta_bpp_mt: f64) -> f64 {
    let delta_bpp_tesla = delta_bpp_mt / 1000.0;
    // For Lorentzian: HWHM = delta_Bpp * sqrt(3) / 2
    let hwhm_tesla = delta_bpp_tesla * (3.0_f64).sqrt() / 2.0;
    if hwhm_tesla.abs() < 1e-30 {
        return f64::INFINITY;
    }
    1.0 / (GAMMA_ELECTRON * hwhm_tesla)
}

/// Calculate signal amplitude under saturation conditions.
///
/// A(P) = A0 / sqrt(1 + gamma^2 * B1^2 * T1 * T2)
///
/// The microwave B1 field is related to power: B1 ~ sqrt(P).
///
/// - `a0`: unsaturated amplitude
/// - `b1_mt`: microwave B1 field amplitude in mT
/// - `t1`: spin-lattice relaxation time in seconds
/// - `t2`: spin-spin relaxation time in seconds
pub fn saturation_amplitude(a0: f64, b1_mt: f64, t1: f64, t2: f64) -> f64 {
    let b1_tesla = b1_mt / 1000.0;
    let gamma_sq = GAMMA_ELECTRON * GAMMA_ELECTRON;
    let s_factor = gamma_sq * b1_tesla * b1_tesla * t1 * t2;
    a0 / (1.0 + s_factor).sqrt()
}

/// Fit T1 from a saturation curve: amplitude vs sqrt(power).
///
/// Given arrays of microwave power levels and measured amplitudes,
/// fit the saturation model: A = A0 / sqrt(1 + k * P) where k ~ gamma^2 * T1 * T2 / Q.
///
/// Returns (a0, saturation_parameter_k) using linearized fit:
/// 1/A^2 = 1/A0^2 + k/A0^2 * P
///
/// - `powers`: relative microwave power levels
/// - `amplitudes`: measured peak-to-peak amplitudes
pub fn fit_saturation_curve(powers: &[f64], amplitudes: &[f64]) -> (f64, f64) {
    if powers.len() != amplitudes.len() || powers.len() < 2 {
        return (0.0, 0.0);
    }

    // Linearize: y = 1/A^2, x = P
    // y = intercept + slope * x, where intercept = 1/A0^2, slope = k/A0^2
    let n = powers.len();
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;
    let mut count = 0;

    for i in 0..n {
        if amplitudes[i].abs() < 1e-30 {
            continue;
        }
        let x = powers[i];
        let y = 1.0 / (amplitudes[i] * amplitudes[i]);
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        count += 1;
    }

    if count < 2 {
        return (0.0, 0.0);
    }

    let cf = count as f64;
    let denom = cf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, 0.0);
    }

    let slope = (cf * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / cf;

    if intercept <= 0.0 {
        return (0.0, 0.0);
    }

    let a0 = (1.0 / intercept).sqrt();
    let k = slope / intercept;

    (a0, k)
}

/// Estimate T1 from saturation parameter and T2.
///
/// From k = gamma^2 * T1 * T2 / Q (simplified), if we know T2 and k in appropriate units:
/// T1 = k * conversion_factor / (gamma^2 * T2)
///
/// This is a simplified estimation -- in practice, the conversion depends on
/// the resonator quality factor and B1 calibration.
///
/// - `saturation_k`: saturation parameter from fit_saturation_curve
/// - `t2`: spin-spin relaxation time in seconds
/// - `b1_per_sqrt_p`: B1 field per sqrt(power) calibration factor (mT/sqrt(mW))
///
/// Returns T1 in seconds.
pub fn estimate_t1(saturation_k: f64, t2: f64, b1_per_sqrt_p: f64) -> f64 {
    let b1_cal = b1_per_sqrt_p / 1000.0; // convert mT to T
    let gamma_sq = GAMMA_ELECTRON * GAMMA_ELECTRON;
    if (gamma_sq * t2 * b1_cal * b1_cal).abs() < 1e-30 {
        return 0.0;
    }
    saturation_k / (gamma_sq * t2 * b1_cal * b1_cal)
}

/// Generate a saturation curve: amplitude vs B1 field.
///
/// Returns (b1_values_mt, amplitude_values).
pub fn saturation_curve(
    a0: f64,
    t1: f64,
    t2: f64,
    b1_min_mt: f64,
    b1_max_mt: f64,
    num_points: usize,
) -> (Vec<f64>, Vec<f64>) {
    let mut b1_vals = Vec::with_capacity(num_points);
    let mut amp_vals = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let b1 = if num_points <= 1 {
            b1_min_mt
        } else {
            b1_min_mt + (b1_max_mt - b1_min_mt) * (i as f64) / ((num_points - 1) as f64)
        };
        b1_vals.push(b1);
        amp_vals.push(saturation_amplitude(a0, b1, t1, t2));
    }

    (b1_vals, amp_vals)
}

// ─── Simulation ─────────────────────────────────────────────────────────────

/// Simulate a simple Lorentzian first-derivative ESR spectrum.
pub fn simulate_lorentzian_spectrum(
    config: &EsrConfig,
    center_field: f64,
    linewidth_mt: f64,
    amplitude: f64,
) -> EsrSpectrum {
    let field = config.field_axis();
    let signal: Vec<f64> = field
        .iter()
        .map(|&b| lorentzian_derivative(b, center_field, linewidth_mt, amplitude))
        .collect();
    EsrSpectrum::new(field, signal)
}

/// Simulate a Gaussian first-derivative ESR spectrum.
pub fn simulate_gaussian_spectrum(
    config: &EsrConfig,
    center_field: f64,
    sigma_mt: f64,
    amplitude: f64,
) -> EsrSpectrum {
    let field = config.field_axis();
    let signal: Vec<f64> = field
        .iter()
        .map(|&b| gaussian_derivative(b, center_field, sigma_mt, amplitude))
        .collect();
    EsrSpectrum::new(field, signal)
}

/// Add Gaussian noise to a spectrum.
///
/// Uses a simple Box-Muller transform for Gaussian random numbers.
pub fn add_noise(spectrum: &mut EsrSpectrum, noise_amplitude: f64, seed: u64) {
    let mut rng_state = seed;
    for s in spectrum.signal.iter_mut() {
        // Simple LCG for uniform random
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u1 = (rng_state >> 11) as f64 / (1u64 << 53) as f64;
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u2 = (rng_state >> 11) as f64 / (1u64 << 53) as f64;

        let u1_safe = u1.max(1e-15);
        let z = (-2.0 * u1_safe.ln()).sqrt() * (2.0 * PI * u2).cos();
        *s += noise_amplitude * z;
    }
}

// ─── Common Radicals and Standards ──────────────────────────────────────────

/// Known g-factors for common paramagnetic species.
pub mod known_species {
    /// Free electron.
    pub const FREE_ELECTRON: f64 = 2.00231930436256;
    /// DPPH (2,2-diphenyl-1-picrylhydrazyl).
    pub const DPPH: f64 = 2.0036;
    /// Organic free radical (typical).
    pub const ORGANIC_RADICAL: f64 = 2.003;
    /// Nitroxide spin label (typical).
    pub const NITROXIDE: f64 = 2.006;
    /// Cu2+ (typical).
    pub const CU2_PLUS: f64 = 2.12;
    /// Fe3+ high-spin (typical).
    pub const FE3_PLUS_HIGH: f64 = 4.3;
    /// Mn2+ (typical).
    pub const MN2_PLUS: f64 = 2.001;
    /// Cr3+ (typical).
    pub const CR3_PLUS: f64 = 1.98;
    /// V4+ (VO2+, vanadyl, typical).
    pub const V4_PLUS: f64 = 1.97;
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // --- Resonance condition tests ---

    #[test]
    fn test_resonant_field_free_electron_x_band() {
        // X-band at 9.5 GHz: B0 = h*nu/(g*mu_B)
        let b0 = resonant_field_mt(9.5, FREE_ELECTRON_G);
        // Expected: h * 9.5e9 / (2.0023 * 9.274e-24) = ~0.3390 T = 339.0 mT
        assert!((b0 - 339.0).abs() < 1.0, "Resonant field = {} mT", b0);
    }

    #[test]
    fn test_resonant_field_q_band() {
        let b0 = resonant_field_mt(34.0, FREE_ELECTRON_G);
        // ~34 GHz -> ~1213 mT
        assert!((b0 - 1213.0).abs() < 5.0, "Q-band resonant field = {} mT", b0);
    }

    #[test]
    fn test_g_factor_roundtrip() {
        let freq = 9.5;
        let g_in = 2.0023;
        let b0 = resonant_field_mt(freq, g_in);
        let g_out = g_factor_from_field(freq, b0);
        assert!((g_out - g_in).abs() < 1e-4, "g roundtrip: {} -> {}", g_in, g_out);
    }

    #[test]
    fn test_g_factor_from_field() {
        // Known: at 9.5 GHz, B0 = 339.0 mT gives g ~ 2.0023
        let g = g_factor_from_field(9.5, 339.0);
        assert!((g - 2.0023).abs() < 0.01, "g = {}", g);
    }

    #[test]
    fn test_frequency_to_wavelength() {
        let wl = frequency_to_wavelength_mm(9.5);
        // c / 9.5e9 ~ 31.6 mm
        assert!((wl - 31.6).abs() < 0.5, "wavelength = {} mm", wl);
    }

    // --- Lineshape tests ---

    #[test]
    fn test_lorentzian_absorption_peak() {
        let amp = lorentzian_absorption(340.0, 340.0, 0.5, 1.0);
        assert!((amp - 1.0).abs() < TOL, "Lorentzian peak = {}", amp);
    }

    #[test]
    fn test_lorentzian_absorption_hwhm() {
        // At B = B0 + delta_B, A should be 0.5 * A0
        let amp = lorentzian_absorption(340.5, 340.0, 0.5, 1.0);
        assert!((amp - 0.5).abs() < TOL, "Lorentzian at HWHM = {}", amp);
    }

    #[test]
    fn test_lorentzian_derivative_zero_at_center() {
        let d = lorentzian_derivative(340.0, 340.0, 0.5, 1.0);
        assert!(d.abs() < TOL, "Derivative at center = {}", d);
    }

    #[test]
    fn test_lorentzian_derivative_antisymmetric() {
        let d_plus = lorentzian_derivative(340.3, 340.0, 0.5, 1.0);
        let d_minus = lorentzian_derivative(339.7, 340.0, 0.5, 1.0);
        assert!((d_plus + d_minus).abs() < 1e-10, "Not antisymmetric: {} vs {}", d_plus, d_minus);
    }

    #[test]
    fn test_gaussian_absorption_peak() {
        let amp = gaussian_absorption(340.0, 340.0, 0.5, 1.0);
        assert!((amp - 1.0).abs() < TOL, "Gaussian peak = {}", amp);
    }

    #[test]
    fn test_gaussian_derivative_zero_at_center() {
        let d = gaussian_derivative(340.0, 340.0, 0.5, 1.0);
        assert!(d.abs() < TOL, "Gaussian derivative at center = {}", d);
    }

    #[test]
    fn test_gaussian_derivative_antisymmetric() {
        let d_plus = gaussian_derivative(340.3, 340.0, 0.5, 1.0);
        let d_minus = gaussian_derivative(339.7, 340.0, 0.5, 1.0);
        assert!((d_plus + d_minus).abs() < 1e-10);
    }

    #[test]
    fn test_voigt_pure_lorentzian() {
        let v = voigt_absorption(340.0, 340.0, 0.5, 0.5, 1.0, Some(1.0));
        let l = lorentzian_absorption(340.0, 340.0, 0.5, 1.0);
        assert!((v - l).abs() < TOL, "Voigt(eta=1) = {} vs Lorentzian = {}", v, l);
    }

    #[test]
    fn test_voigt_pure_gaussian() {
        let v = voigt_absorption(340.0, 340.0, 0.5, 0.5, 1.0, Some(0.0));
        let g = gaussian_absorption(340.0, 340.0, 0.5, 1.0);
        assert!((v - g).abs() < TOL, "Voigt(eta=0) = {} vs Gaussian = {}", v, g);
    }

    #[test]
    fn test_voigt_derivative() {
        let vd = voigt_derivative(340.0, 340.0, 0.5, 0.5, 1.0, 0.5);
        // At center, derivative should be zero for symmetric lineshape
        assert!(vd.abs() < TOL, "Voigt derivative at center = {}", vd);
    }

    // --- Peak analysis tests ---

    #[test]
    fn test_peak_to_peak_amplitude() {
        let signal = vec![-3.0, -1.0, 0.0, 2.0, 5.0, 1.0, -2.0];
        let pp = peak_to_peak_amplitude(&signal);
        assert!((pp - 8.0).abs() < TOL);
    }

    #[test]
    fn test_peak_to_peak_linewidth_lorentzian() {
        let config = EsrConfig::x_band();
        let spectrum = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 1.0);
        let dbpp = peak_to_peak_linewidth(&spectrum.magnetic_field_mt, &spectrum.signal);
        // For Lorentzian: delta_Bpp = 2 * HWHM / sqrt(3) = 2 * 0.5 / sqrt(3) ~ 0.577 mT
        let expected = 2.0 * 0.5 / (3.0_f64).sqrt();
        assert!((dbpp - expected).abs() < 0.1, "delta_Bpp = {} (expected ~{})", dbpp, expected);
    }

    #[test]
    fn test_center_field_determination() {
        let config = EsrConfig::x_band();
        let b0_true = 340.0;
        let spectrum = simulate_lorentzian_spectrum(&config, b0_true, 0.5, 1.0);
        let b0_meas = center_field_mt(&spectrum.magnetic_field_mt, &spectrum.signal);
        assert!((b0_meas - b0_true).abs() < 0.1, "Center field = {} (expected {})", b0_meas, b0_true);
    }

    #[test]
    fn test_g_factor_from_spectrum() {
        let config = EsrConfig::x_band();
        let b0_true = resonant_field_mt(config.microwave_frequency_ghz, 2.0023);
        let spectrum = simulate_lorentzian_spectrum(&config, b0_true, 0.5, 1.0);
        let g = g_factor_from_spectrum(
            config.microwave_frequency_ghz,
            &spectrum.magnetic_field_mt,
            &spectrum.signal,
        );
        assert!((g - 2.0023).abs() < 0.01, "g from spectrum = {}", g);
    }

    // --- Spectrum operations tests ---

    #[test]
    fn test_absorption_from_derivative() {
        let config = EsrConfig::x_band();
        let spectrum = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 1.0);
        let absorption = spectrum.to_absorption();
        // Absorption should be positive near the center
        let center_idx = config.num_points / 2;
        assert!(absorption[center_idx] > 0.0, "Absorption at center should be positive");
    }

    #[test]
    fn test_double_integral_positive() {
        let config = EsrConfig::x_band();
        let spectrum = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 1.0);
        let di = spectrum.double_integral();
        assert!(di > 0.0, "Double integral should be positive: {}", di);
    }

    #[test]
    fn test_second_derivative() {
        let config = EsrConfig::x_band();
        let spectrum = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 1.0);
        let second_deriv = spectrum.to_second_derivative();
        assert_eq!(second_deriv.len(), spectrum.len());
        // Second derivative should have a negative peak at center for Lorentzian
    }

    #[test]
    fn test_snr_estimation() {
        let config = EsrConfig::x_band();
        let mut spectrum = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 1.0);
        let snr_clean = spectrum.snr(0.1);
        add_noise(&mut spectrum, 0.01, 42);
        let snr_noisy = spectrum.snr(0.1);
        // Clean signal should have higher SNR than noisy
        assert!(snr_clean > snr_noisy, "Clean SNR ({}) should > noisy SNR ({})", snr_clean, snr_noisy);
    }

    // --- Hyperfine splitting tests ---

    #[test]
    fn test_hyperfine_line_count_nitrogen_triplet() {
        // 1 nitrogen (I=1): 2*1*1 + 1 = 3 lines
        let count = hyperfine_line_count(1, NuclearSpin::One);
        assert_eq!(count, 3);
    }

    #[test]
    fn test_hyperfine_line_count_hydrogen_doublet() {
        // 1 hydrogen (I=1/2): 2*1*0.5 + 1 = 2 lines
        let count = hyperfine_line_count(1, NuclearSpin::Half);
        assert_eq!(count, 2);
    }

    #[test]
    fn test_hyperfine_line_count_three_hydrogens() {
        // 3 hydrogens: 2*3*0.5 + 1 = 4 lines (quartet)
        let count = hyperfine_line_count(3, NuclearSpin::Half);
        assert_eq!(count, 4);
    }

    #[test]
    fn test_binomial_intensities_quartet() {
        // 3 equivalent I=1/2 nuclei: 1:3:3:1
        let intensities = binomial_intensities(3);
        assert_eq!(intensities.len(), 4);
        assert!((intensities[0] - 1.0).abs() < TOL);
        assert!((intensities[1] - 3.0).abs() < TOL);
        assert!((intensities[2] - 3.0).abs() < TOL);
        assert!((intensities[3] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_trinomial_intensities_nitrogen() {
        // 1 nitrogen (I=1): 3 lines with 1:1:1
        let intensities = trinomial_intensities(1);
        assert_eq!(intensities.len(), 3);
        assert!((intensities[0] - 1.0).abs() < TOL);
        assert!((intensities[1] - 1.0).abs() < TOL);
        assert!((intensities[2] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_trinomial_intensities_two_nitrogens() {
        // 2 nitrogens (I=1): 5 lines with 1:2:3:2:1
        let intensities = trinomial_intensities(2);
        assert_eq!(intensities.len(), 5);
        assert!((intensities[0] - 1.0).abs() < TOL);
        assert!((intensities[1] - 2.0).abs() < TOL);
        assert!((intensities[2] - 3.0).abs() < TOL);
        assert!((intensities[3] - 2.0).abs() < TOL);
        assert!((intensities[4] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_hyperfine_pattern_spacing() {
        let a_mt = 1.5; // 1.5 mT coupling constant
        let (offsets, _) = hyperfine_pattern(a_mt, 1, NuclearSpin::One);
        assert_eq!(offsets.len(), 3);
        // Spacing between adjacent lines should equal coupling constant
        let spacing = (offsets[1] - offsets[0]).abs();
        assert!((spacing - a_mt).abs() < TOL, "Spacing = {} (expected {})", spacing, a_mt);
    }

    #[test]
    fn test_simulate_hyperfine_spectrum() {
        let config = EsrConfig::x_band();
        let spectrum = simulate_hyperfine_spectrum(
            &config, 340.0, 1.5, 1, NuclearSpin::One, 0.2, 1.0,
        );
        assert_eq!(spectrum.len(), config.num_points);
        // Should have signal content (not all zeros)
        let pp = peak_to_peak_amplitude(&spectrum.signal);
        assert!(pp > 0.0, "Spectrum should have non-zero amplitude");
    }

    // --- Spin Hamiltonian tests ---

    #[test]
    fn test_isotropic_hamiltonian() {
        let h = SpinHamiltonian::isotropic(2.0023, 1.5);
        assert!((h.g_iso - 2.0023).abs() < TOL);
        assert!((h.a_iso_mt - 1.5).abs() < TOL);
        assert!(h.g_axial.is_none());
        assert!((h.spin_s - 0.5).abs() < TOL);
    }

    #[test]
    fn test_axial_hamiltonian() {
        let h = SpinHamiltonian::axial(2.40, 2.05, 18.0, 6.0);
        // g_iso = (g_par + 2*g_perp) / 3
        let expected_g_iso = (2.40 + 2.0 * 2.05) / 3.0;
        assert!((h.g_iso - expected_g_iso).abs() < TOL);
        // a_iso = (A_par + 2*A_perp) / 3
        let expected_a_iso = (18.0 + 2.0 * 6.0) / 3.0;
        assert!((h.a_iso_mt - expected_a_iso).abs() < TOL);
    }

    #[test]
    fn test_g_effective_parallel() {
        let h = SpinHamiltonian::axial(2.40, 2.05, 18.0, 6.0);
        let g_par = h.g_effective(0.0); // theta = 0 -> g_parallel
        assert!((g_par - 2.40).abs() < TOL, "g_effective(0) = {}", g_par);
    }

    #[test]
    fn test_g_effective_perpendicular() {
        let h = SpinHamiltonian::axial(2.40, 2.05, 18.0, 6.0);
        let g_perp = h.g_effective(PI / 2.0); // theta = 90 -> g_perpendicular
        assert!((g_perp - 2.05).abs() < TOL, "g_effective(pi/2) = {}", g_perp);
    }

    #[test]
    fn test_rhombicity() {
        let h = SpinHamiltonian::high_spin(2.0, 1.0, 10.0, 3.0);
        assert!((h.rhombicity() - 0.3).abs() < TOL);
    }

    // --- Signal processing tests ---

    #[test]
    fn test_phase_sensitive_detection() {
        let n = 10000;
        let fs = 1e6; // 1 MHz sampling
        let fmod = 100e3; // 100 kHz modulation
        let dt = 1.0 / fs;

        // Signal: cosine at modulation frequency
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * fmod * i as f64 * dt).cos())
            .collect();

        let psd = phase_sensitive_detection(&signal, fs, fmod, 1e-4);
        assert_eq!(psd.len(), n);
        // After demodulation and filtering, should converge to ~1.0
        let last_val = psd[n - 1];
        assert!(last_val > 0.5, "PSD output = {} (expected near 1.0)", last_val);
    }

    #[test]
    fn test_baseline_correction() {
        let n = 100;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let x = i as f64 / n as f64;
                5.0 * x + 2.0 // linear baseline
            })
            .collect();

        let corrected = baseline_correction(&signal, 1, 0.2);
        // After correction, mean should be near zero
        let mean = corrected.iter().sum::<f64>() / n as f64;
        assert!(mean.abs() < 1.0, "Corrected mean = {}", mean);
    }

    #[test]
    fn test_savitzky_golay_smooth() {
        let n = 101;
        let signal: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
        let smoothed = savitzky_golay_smooth(&signal, 5, 2);
        assert_eq!(smoothed.len(), n);
        // Smoothed signal should be close to original for smooth input
        let mid = n / 2;
        assert!((smoothed[mid] - signal[mid]).abs() < 0.01, "SG deviation at midpoint");
    }

    // --- Spin quantitation tests ---

    #[test]
    fn test_spin_count_with_standard() {
        let standard = SpinStandard::dpph(1e15, 100.0);
        let sample_di = 200.0;
        let sample_g = 2.0036;
        let n_spins = calculate_num_spins(sample_di, sample_g, &standard);
        // Same g-factor, double DI -> double spins
        assert!((n_spins - 2e15).abs() < 1e12, "Spin count = {}", n_spins);
    }

    #[test]
    fn test_spin_concentration() {
        let conc = spin_concentration(6.022e23, 1000.0); // 1 mole in 1 liter
        let molar = spins_to_molar(conc);
        assert!((molar - 1.0).abs() < 0.01, "Molar concentration = {}", molar);
    }

    // --- Relaxation time tests ---

    #[test]
    fn test_t2_from_linewidth() {
        let delta_bpp = 0.5; // 0.5 mT
        let t2 = t2_from_linewidth(delta_bpp);
        // T2 should be on the order of nanoseconds for 0.5 mT linewidth
        assert!(t2 > 0.0 && t2 < 1e-6, "T2 = {} s", t2);
    }

    #[test]
    fn test_saturation_no_power() {
        let amp = saturation_amplitude(1.0, 0.0, 1e-6, 1e-8);
        assert!((amp - 1.0).abs() < TOL, "At zero power, amplitude should equal A0");
    }

    #[test]
    fn test_saturation_decreases_with_power() {
        let a_low = saturation_amplitude(1.0, 0.001, 1e-6, 1e-8);
        let a_high = saturation_amplitude(1.0, 0.01, 1e-6, 1e-8);
        assert!(a_high < a_low, "Higher power should give lower amplitude: {} vs {}", a_high, a_low);
    }

    #[test]
    fn test_saturation_curve_generation() {
        let (b1, amps) = saturation_curve(1.0, 1e-6, 1e-8, 0.001, 0.1, 50);
        assert_eq!(b1.len(), 50);
        assert_eq!(amps.len(), 50);
        // First amplitude should be largest
        assert!(amps[0] > amps[49], "Amplitude should decrease with B1");
    }

    #[test]
    fn test_fit_saturation_curve() {
        // Generate synthetic saturation data
        let t1 = 1e-6;
        let t2 = 1e-8;
        let a0_true = 100.0;
        let b1_per_sqrt_p = 0.01; // mT/sqrt(mW)

        let powers: Vec<f64> = (1..=20).map(|i| i as f64 * 0.5).collect();
        let amplitudes: Vec<f64> = powers
            .iter()
            .map(|&p| {
                let b1 = b1_per_sqrt_p * p.sqrt();
                saturation_amplitude(a0_true, b1, t1, t2)
            })
            .collect();

        let (a0_fit, _k) = fit_saturation_curve(&powers, &amplitudes);
        // Fitted A0 should be close to true value
        assert!((a0_fit - a0_true).abs() / a0_true < 0.1, "Fitted A0 = {} (true = {})", a0_fit, a0_true);
    }

    // --- Configuration tests ---

    #[test]
    fn test_x_band_config() {
        let config = EsrConfig::x_band();
        assert!((config.microwave_frequency_ghz - 9.5).abs() < TOL);
        assert_eq!(config.num_points, 1024);
    }

    #[test]
    fn test_field_axis() {
        let config = EsrConfig::x_band();
        let axis = config.field_axis();
        assert_eq!(axis.len(), 1024);
        assert!((axis[0] - 300.0).abs() < TOL);
        assert!((axis[1023] - 380.0).abs() < TOL);
    }

    #[test]
    fn test_known_species_g_factors() {
        assert!((known_species::DPPH - 2.0036).abs() < TOL);
        assert!((known_species::FREE_ELECTRON - FREE_ELECTRON_G).abs() < TOL);
        assert!(known_species::CU2_PLUS > 2.0);
        assert!(known_species::FE3_PLUS_HIGH > 4.0);
    }

    // --- Noise and simulation tests ---

    #[test]
    fn test_add_noise() {
        let config = EsrConfig::x_band();
        let mut spectrum = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 1.0);
        let pp_before = peak_to_peak_amplitude(&spectrum.signal);
        add_noise(&mut spectrum, 0.001, 12345);
        // Signal should be slightly different but not wildly
        let pp_after = peak_to_peak_amplitude(&spectrum.signal);
        assert!((pp_after - pp_before).abs() < 0.1 * pp_before);
    }

    #[test]
    fn test_gaussian_spectrum_simulation() {
        let config = EsrConfig::x_band();
        let spectrum = simulate_gaussian_spectrum(&config, 340.0, 0.3, 1.0);
        assert_eq!(spectrum.len(), config.num_points);
        let pp = peak_to_peak_amplitude(&spectrum.signal);
        assert!(pp > 0.0);
    }

    #[test]
    fn test_spectrum_empty() {
        let spec = EsrSpectrum::new(vec![], vec![]);
        assert!(spec.is_empty());
        assert_eq!(spec.len(), 0);
    }

    #[test]
    fn test_double_integral_proportional_to_amplitude() {
        let config = EsrConfig::x_band();
        let spec1 = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 1.0);
        let spec2 = simulate_lorentzian_spectrum(&config, 340.0, 0.5, 2.0);
        let di1 = spec1.double_integral();
        let di2 = spec2.double_integral();
        // Double integral should scale with amplitude
        assert!((di2 / di1 - 2.0).abs() < 0.1, "DI ratio = {} (expected 2.0)", di2 / di1);
    }
}

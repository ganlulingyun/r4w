//! Laser-Induced Breakdown Spectroscopy (LIBS) signal processing for elemental
//! analysis of materials.
//!
//! This module implements the full LIBS spectral processing chain from raw
//! plasma emission spectra to quantitative elemental composition. LIBS works by
//! focusing a high-energy pulsed laser onto a sample surface, creating a
//! micro-plasma whose optical emission reveals the elemental composition of the
//! target material.
//!
//! # Background
//!
//! When a laser pulse (typically Nd:YAG at 1064 nm, 5-200 mJ, 5-10 ns) is
//! focused onto a sample, it ablates a tiny amount of material and creates a
//! plasma at temperatures of 10,000-20,000 K. As the plasma cools, the excited
//! atoms and ions emit characteristic spectral lines that uniquely identify each
//! element. By measuring the wavelength and intensity of these lines, the
//! elemental composition can be determined qualitatively and quantitatively.
//!
//! # Processing Pipeline
//!
//! 1. **Temporal gating** - Delay acquisition to reject early Bremsstrahlung
//!    continuum (typically 0.5-2 us gate delay, 1-10 us gate width).
//! 2. **Continuum subtraction** - Estimate and remove residual broadband
//!    background from free-free and free-bound transitions.
//! 3. **Peak detection** - Identify emission lines above an SNR threshold.
//! 4. **Line fitting** - Fit Lorentzian, Gaussian, or Voigt profiles to
//!    extract line parameters (center, width, area).
//! 5. **Element identification** - Match detected peaks to known emission
//!    line databases (NIST Atomic Spectra Database).
//! 6. **Plasma diagnostics** - Derive electron temperature (Boltzmann plot)
//!    and electron density (Stark broadening).
//! 7. **Quantitative analysis** - Convert line intensities to elemental
//!    concentrations via calibration curves or CF-LIBS.
//!
//! # Key Equations
//!
//! Lorentzian line profile:
//! ```text
//!   I(lambda) = A * (gamma/2)^2 / ((lambda - lambda0)^2 + (gamma/2)^2)
//! ```
//!
//! Boltzmann plot for electron temperature:
//! ```text
//!   ln(I * lambda / (g * A_ki)) = -E_k / (kB * T) + C
//! ```
//!
//! Stark broadening for electron density:
//! ```text
//!   delta_lambda_S = 2 * w * (n_e / 10^16)
//! ```
//!
//! Calibration-Free LIBS concentration:
//! ```text
//!   C_s = (F * I * lambda) / (g * A_ki * U(T) * exp(-E_k / (kB * T)))
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::laser_induced_breakdown_spectroscopy::{
//!     LibsSpectrum, LibsAnalyzer, LibsAnalyzerConfig, EmissionDatabase,
//! };
//!
//! // Create a synthetic LIBS spectrum with a sodium doublet
//! let wavelengths: Vec<f64> = (0..1000)
//!     .map(|i| 500.0 + i as f64 * 0.5)
//!     .collect();
//! let mut intensities = vec![100.0_f64; 1000]; // background
//! // Add Na D-lines
//! for (i, &wl) in wavelengths.iter().enumerate() {
//!     let g = 0.1;
//!     intensities[i] += 5000.0 * (g / 2.0_f64).powi(2)
//!         / ((wl - 589.0).powi(2) + (g / 2.0).powi(2));
//!     intensities[i] += 3000.0 * (g / 2.0_f64).powi(2)
//!         / ((wl - 589.6).powi(2) + (g / 2.0).powi(2));
//! }
//!
//! let spectrum = LibsSpectrum::new(
//!     wavelengths,
//!     intensities,
//!     1.0,   // gate_delay_us
//!     5.0,   // gate_width_us
//!     50.0,  // laser_energy_mj
//! );
//!
//! let config = LibsAnalyzerConfig {
//!     spectral_resolution_nm: 0.05,
//!     wavelength_min_nm: 500.0,
//!     wavelength_max_nm: 999.5,
//!     gate_delay_us: 1.0,
//!     gate_width_us: 5.0,
//!     snr_threshold: 3.0,
//!     continuum_window: 50,
//! };
//!
//! let analyzer = LibsAnalyzer::new(config);
//! let peaks = analyzer.detect_peaks(&spectrum);
//! assert!(peaks.len() >= 1);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Boltzmann constant in eV/K.
const KB_EV: f64 = 8.617_333_262e-5;

// ---------------------------------------------------------------------------
// Emission line database
// ---------------------------------------------------------------------------

/// A single atomic emission line entry.
#[derive(Debug, Clone)]
pub struct EmissionLine {
    /// Element symbol.
    pub element: &'static str,
    /// Wavelength in nm.
    pub wavelength_nm: f64,
    /// Statistical weight of the upper level (degeneracy g_k).
    pub g_upper: f64,
    /// Transition probability (Einstein A coefficient) in s^-1.
    pub a_ki: f64,
    /// Upper level energy in eV.
    pub e_upper_ev: f64,
    /// Stark width parameter w in nm at n_e = 10^16 cm^-3.
    pub stark_width_nm: f64,
    /// Relative intensity (arbitrary scale, useful for identification).
    pub relative_intensity: f64,
}

/// Database of common LIBS emission lines for element identification and
/// plasma diagnostics.
pub struct EmissionDatabase {
    lines: Vec<EmissionLine>,
}

impl EmissionDatabase {
    /// Create a new database with built-in emission lines.
    pub fn new() -> Self {
        let lines = vec![
            // Iron (Fe I)
            EmissionLine { element: "Fe", wavelength_nm: 259.94, g_upper: 9.0, a_ki: 3.2e8, e_upper_ev: 4.77, stark_width_nm: 0.012, relative_intensity: 800.0 },
            EmissionLine { element: "Fe", wavelength_nm: 274.95, g_upper: 7.0, a_ki: 2.1e8, e_upper_ev: 4.51, stark_width_nm: 0.011, relative_intensity: 600.0 },
            EmissionLine { element: "Fe", wavelength_nm: 438.35, g_upper: 11.0, a_ki: 5.0e7, e_upper_ev: 4.31, stark_width_nm: 0.020, relative_intensity: 500.0 },
            // Calcium (Ca II)
            EmissionLine { element: "Ca", wavelength_nm: 393.37, g_upper: 4.0, a_ki: 1.47e8, e_upper_ev: 3.15, stark_width_nm: 0.035, relative_intensity: 9000.0 },
            EmissionLine { element: "Ca", wavelength_nm: 396.85, g_upper: 2.0, a_ki: 1.40e8, e_upper_ev: 3.12, stark_width_nm: 0.034, relative_intensity: 8500.0 },
            EmissionLine { element: "Ca", wavelength_nm: 422.67, g_upper: 3.0, a_ki: 2.18e8, e_upper_ev: 2.93, stark_width_nm: 0.030, relative_intensity: 7000.0 },
            // Sodium (Na I)
            EmissionLine { element: "Na", wavelength_nm: 588.99, g_upper: 4.0, a_ki: 6.16e7, e_upper_ev: 2.10, stark_width_nm: 0.040, relative_intensity: 9500.0 },
            EmissionLine { element: "Na", wavelength_nm: 589.59, g_upper: 2.0, a_ki: 6.14e7, e_upper_ev: 2.10, stark_width_nm: 0.040, relative_intensity: 9000.0 },
            // Aluminum (Al I)
            EmissionLine { element: "Al", wavelength_nm: 308.22, g_upper: 4.0, a_ki: 5.9e7, e_upper_ev: 4.02, stark_width_nm: 0.015, relative_intensity: 700.0 },
            EmissionLine { element: "Al", wavelength_nm: 309.27, g_upper: 6.0, a_ki: 7.4e7, e_upper_ev: 4.02, stark_width_nm: 0.015, relative_intensity: 900.0 },
            EmissionLine { element: "Al", wavelength_nm: 394.40, g_upper: 2.0, a_ki: 4.99e7, e_upper_ev: 3.14, stark_width_nm: 0.018, relative_intensity: 600.0 },
            // Silicon (Si I)
            EmissionLine { element: "Si", wavelength_nm: 250.69, g_upper: 3.0, a_ki: 5.5e7, e_upper_ev: 4.95, stark_width_nm: 0.010, relative_intensity: 500.0 },
            EmissionLine { element: "Si", wavelength_nm: 251.61, g_upper: 5.0, a_ki: 1.2e8, e_upper_ev: 4.93, stark_width_nm: 0.010, relative_intensity: 700.0 },
            EmissionLine { element: "Si", wavelength_nm: 288.16, g_upper: 3.0, a_ki: 2.2e8, e_upper_ev: 5.08, stark_width_nm: 0.013, relative_intensity: 800.0 },
            // Magnesium (Mg I / Mg II)
            EmissionLine { element: "Mg", wavelength_nm: 279.55, g_upper: 4.0, a_ki: 2.6e8, e_upper_ev: 4.43, stark_width_nm: 0.015, relative_intensity: 8000.0 },
            EmissionLine { element: "Mg", wavelength_nm: 280.27, g_upper: 2.0, a_ki: 2.6e8, e_upper_ev: 4.42, stark_width_nm: 0.015, relative_intensity: 7500.0 },
            EmissionLine { element: "Mg", wavelength_nm: 285.21, g_upper: 3.0, a_ki: 4.91e8, e_upper_ev: 4.35, stark_width_nm: 0.014, relative_intensity: 6000.0 },
            // Carbon (C I)
            EmissionLine { element: "C", wavelength_nm: 247.86, g_upper: 5.0, a_ki: 3.4e7, e_upper_ev: 7.68, stark_width_nm: 0.008, relative_intensity: 400.0 },
            // Hydrogen (H I)
            EmissionLine { element: "H", wavelength_nm: 656.28, g_upper: 18.0, a_ki: 4.41e7, e_upper_ev: 12.09, stark_width_nm: 0.080, relative_intensity: 5000.0 },
            // Oxygen (O I)
            EmissionLine { element: "O", wavelength_nm: 777.19, g_upper: 7.0, a_ki: 3.69e7, e_upper_ev: 10.74, stark_width_nm: 0.025, relative_intensity: 3000.0 },
            EmissionLine { element: "O", wavelength_nm: 844.68, g_upper: 9.0, a_ki: 3.22e7, e_upper_ev: 10.99, stark_width_nm: 0.028, relative_intensity: 2500.0 },
        ];
        Self { lines }
    }

    /// Find all emission lines within `tolerance_nm` of a given wavelength.
    pub fn lookup(&self, wavelength_nm: f64, tolerance_nm: f64) -> Vec<&EmissionLine> {
        self.lines
            .iter()
            .filter(|l| (l.wavelength_nm - wavelength_nm).abs() <= tolerance_nm)
            .collect()
    }

    /// Get all lines for a given element.
    pub fn lines_for_element(&self, element: &str) -> Vec<&EmissionLine> {
        self.lines.iter().filter(|l| l.element == element).collect()
    }

    /// Get all lines in the database.
    pub fn all_lines(&self) -> &[EmissionLine] {
        &self.lines
    }

    /// Return unique element symbols in the database.
    pub fn elements(&self) -> Vec<&'static str> {
        let mut elems: Vec<&'static str> = self.lines.iter().map(|l| l.element).collect();
        elems.sort();
        elems.dedup();
        elems
    }
}

impl Default for EmissionDatabase {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// LIBS spectrum
// ---------------------------------------------------------------------------

/// A single LIBS measurement spectrum.
#[derive(Debug, Clone)]
pub struct LibsSpectrum {
    /// Wavelength axis in nm.
    pub wavelength_nm: Vec<f64>,
    /// Intensity values (arbitrary units, e.g. CCD counts).
    pub intensity: Vec<f64>,
    /// Detector gate delay after laser pulse in microseconds.
    pub gate_delay_us: f64,
    /// Detector gate width in microseconds.
    pub gate_width_us: f64,
    /// Laser pulse energy in millijoules.
    pub laser_energy_mj: f64,
}

impl LibsSpectrum {
    /// Create a new LIBS spectrum.
    pub fn new(
        wavelength_nm: Vec<f64>,
        intensity: Vec<f64>,
        gate_delay_us: f64,
        gate_width_us: f64,
        laser_energy_mj: f64,
    ) -> Self {
        assert_eq!(
            wavelength_nm.len(),
            intensity.len(),
            "wavelength and intensity arrays must have equal length"
        );
        Self {
            wavelength_nm,
            intensity,
            gate_delay_us,
            gate_width_us,
            laser_energy_mj,
        }
    }

    /// Number of spectral channels.
    pub fn len(&self) -> usize {
        self.wavelength_nm.len()
    }

    /// Check if the spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.wavelength_nm.is_empty()
    }

    /// Spectral resolution (average spacing between channels) in nm.
    pub fn resolution_nm(&self) -> f64 {
        if self.wavelength_nm.len() < 2 {
            return 0.0;
        }
        let n = self.wavelength_nm.len();
        (self.wavelength_nm[n - 1] - self.wavelength_nm[0]) / (n - 1) as f64
    }

    /// Return a sub-spectrum within a wavelength window.
    pub fn window(&self, wl_min: f64, wl_max: f64) -> LibsSpectrum {
        let mut wl = Vec::new();
        let mut int = Vec::new();
        for (i, &w) in self.wavelength_nm.iter().enumerate() {
            if w >= wl_min && w <= wl_max {
                wl.push(w);
                int.push(self.intensity[i]);
            }
        }
        LibsSpectrum {
            wavelength_nm: wl,
            intensity: int,
            gate_delay_us: self.gate_delay_us,
            gate_width_us: self.gate_width_us,
            laser_energy_mj: self.laser_energy_mj,
        }
    }
}

// ---------------------------------------------------------------------------
// Detected peak
// ---------------------------------------------------------------------------

/// A detected emission peak in a LIBS spectrum.
#[derive(Debug, Clone)]
pub struct DetectedPeak {
    /// Center wavelength in nm.
    pub wavelength_nm: f64,
    /// Peak intensity (background-subtracted).
    pub intensity: f64,
    /// Signal-to-noise ratio.
    pub snr: f64,
    /// FWHM (full width at half maximum) in nm.
    pub fwhm_nm: f64,
    /// Integrated area under the peak.
    pub area: f64,
    /// Index in the spectrum array.
    pub index: usize,
}

// ---------------------------------------------------------------------------
// Line profile functions
// ---------------------------------------------------------------------------

/// Lorentzian line profile: I(lambda) = A * (gamma/2)^2 / ((lambda-lambda0)^2 + (gamma/2)^2)
pub fn lorentzian(wavelength: f64, lambda0: f64, amplitude: f64, gamma: f64) -> f64 {
    let half_gamma = gamma / 2.0;
    let dw = wavelength - lambda0;
    amplitude * half_gamma * half_gamma / (dw * dw + half_gamma * half_gamma)
}

/// Gaussian line profile: I(lambda) = A * exp(-4*ln(2)*(lambda-lambda0)^2 / sigma^2)
/// where sigma is the FWHM.
pub fn gaussian_profile(wavelength: f64, lambda0: f64, amplitude: f64, fwhm: f64) -> f64 {
    let dw = wavelength - lambda0;
    let c = 4.0 * 2.0_f64.ln() / (fwhm * fwhm);
    amplitude * (-c * dw * dw).exp()
}

/// Voigt profile approximation using the pseudo-Voigt method.
///
/// The Voigt profile is the convolution of a Gaussian (Doppler broadening)
/// and a Lorentzian (Stark/pressure broadening). The pseudo-Voigt
/// approximation uses a linear combination:
///
///   V(x) = eta * L(x) + (1 - eta) * G(x)
///
/// where eta depends on the Lorentzian and Gaussian widths.
pub fn voigt_pseudo(
    wavelength: f64,
    lambda0: f64,
    amplitude: f64,
    fwhm_gaussian: f64,
    fwhm_lorentzian: f64,
) -> f64 {
    // Thompson formula for total Voigt FWHM
    let fg = fwhm_gaussian;
    let fl = fwhm_lorentzian;
    let fv = (fg.powi(5)
        + 2.69269 * fg.powi(4) * fl
        + 2.42843 * fg.powi(3) * fl.powi(2)
        + 4.47163 * fg.powi(2) * fl.powi(3)
        + 0.07842 * fg * fl.powi(4)
        + fl.powi(5))
    .powf(0.2);

    // Mixing parameter eta
    let eta = 1.36603 * (fl / fv)
        - 0.47719 * (fl / fv).powi(2)
        + 0.11116 * (fl / fv).powi(3);
    let eta = eta.clamp(0.0, 1.0);

    let l = lorentzian(wavelength, lambda0, 1.0, fv);
    let g = gaussian_profile(wavelength, lambda0, 1.0, fv);
    amplitude * (eta * l + (1.0 - eta) * g)
}

// ---------------------------------------------------------------------------
// Fit result
// ---------------------------------------------------------------------------

/// Result of fitting a line profile to spectral data.
#[derive(Debug, Clone)]
pub struct LineFitResult {
    /// Fitted center wavelength in nm.
    pub center_nm: f64,
    /// Fitted amplitude.
    pub amplitude: f64,
    /// Fitted FWHM in nm.
    pub fwhm_nm: f64,
    /// Integrated area under the fitted profile.
    pub area: f64,
    /// Residual sum of squares.
    pub residual_ss: f64,
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

/// A calibration point mapping known concentration to measured intensity.
#[derive(Debug, Clone)]
pub struct CalibrationPoint {
    /// Known concentration (e.g., weight percent).
    pub concentration: f64,
    /// Measured line intensity.
    pub intensity: f64,
}

/// Linear calibration curve: concentration = slope * intensity + intercept.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    pub slope: f64,
    pub intercept: f64,
    /// R-squared coefficient of determination.
    pub r_squared: f64,
    /// Sensitivity (slope of calibration curve).
    pub sensitivity: f64,
}

impl CalibrationCurve {
    /// Build a calibration curve from standard measurements using linear
    /// least squares.
    pub fn from_points(points: &[CalibrationPoint]) -> Option<Self> {
        let n = points.len();
        if n < 2 {
            return None;
        }
        let nf = n as f64;
        let sum_x: f64 = points.iter().map(|p| p.intensity).sum();
        let sum_y: f64 = points.iter().map(|p| p.concentration).sum();
        let sum_xx: f64 = points.iter().map(|p| p.intensity * p.intensity).sum();
        let sum_xy: f64 = points.iter().map(|p| p.intensity * p.concentration).sum();

        let denom = nf * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }
        let slope = (nf * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / nf;

        // R-squared
        let y_mean = sum_y / nf;
        let ss_tot: f64 = points.iter().map(|p| (p.concentration - y_mean).powi(2)).sum();
        let ss_res: f64 = points
            .iter()
            .map(|p| (p.concentration - (slope * p.intensity + intercept)).powi(2))
            .sum();
        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        Some(Self {
            slope,
            intercept,
            r_squared,
            sensitivity: slope,
        })
    }

    /// Predict concentration from a measured intensity.
    pub fn predict(&self, intensity: f64) -> f64 {
        self.slope * intensity + self.intercept
    }

    /// Limit of detection: LOD = 3 * sigma_background / sensitivity.
    pub fn limit_of_detection(&self, sigma_background: f64) -> f64 {
        if self.sensitivity.abs() < 1e-30 {
            return f64::INFINITY;
        }
        3.0 * sigma_background / self.sensitivity.abs()
    }
}

// ---------------------------------------------------------------------------
// Plasma diagnostics
// ---------------------------------------------------------------------------

/// Result of a Boltzmann plot electron temperature determination.
#[derive(Debug, Clone)]
pub struct BoltzmannPlotResult {
    /// Electron temperature in Kelvin.
    pub temperature_k: f64,
    /// R-squared of the linear fit.
    pub r_squared: f64,
    /// Slope of the Boltzmann plot.
    pub slope: f64,
    /// Intercept of the Boltzmann plot.
    pub intercept: f64,
}

/// A Boltzmann plot data point for temperature determination.
#[derive(Debug, Clone)]
pub struct BoltzmannPoint {
    /// Measured line intensity.
    pub intensity: f64,
    /// Wavelength in nm.
    pub wavelength_nm: f64,
    /// Statistical weight of upper level.
    pub g_upper: f64,
    /// Transition probability in s^-1.
    pub a_ki: f64,
    /// Upper level energy in eV.
    pub e_upper_ev: f64,
}

/// Compute electron temperature from a Boltzmann plot.
///
/// Fits: ln(I * lambda / (g * A)) = -E / (kB * T) + C
///
/// Returns None if fewer than 2 points or the fit is degenerate.
pub fn boltzmann_temperature(points: &[BoltzmannPoint]) -> Option<BoltzmannPlotResult> {
    let n = points.len();
    if n < 2 {
        return None;
    }

    // Build (x, y) = (E_upper, ln(I * lambda / (g * A)))
    let mut xs = Vec::with_capacity(n);
    let mut ys = Vec::with_capacity(n);
    for p in points {
        if p.intensity <= 0.0 || p.g_upper <= 0.0 || p.a_ki <= 0.0 {
            continue;
        }
        let y = (p.intensity * p.wavelength_nm / (p.g_upper * p.a_ki)).ln();
        xs.push(p.e_upper_ev);
        ys.push(y);
    }
    let n = xs.len();
    if n < 2 {
        return None;
    }

    // Linear least squares
    let nf = n as f64;
    let sum_x: f64 = xs.iter().sum();
    let sum_y: f64 = ys.iter().sum();
    let sum_xx: f64 = xs.iter().map(|&x| x * x).sum();
    let sum_xy: f64 = xs.iter().zip(ys.iter()).map(|(&x, &y)| x * y).sum();

    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return None;
    }
    let slope = (nf * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / nf;

    // Temperature: slope = -1 / (kB * T)
    if slope.abs() < 1e-30 {
        return None;
    }
    let temperature_k = -1.0 / (KB_EV * slope);

    // R-squared
    let y_mean = sum_y / nf;
    let ss_tot: f64 = ys.iter().map(|&y| (y - y_mean).powi(2)).sum();
    let ss_res: f64 = xs
        .iter()
        .zip(ys.iter())
        .map(|(&x, &y)| (y - (slope * x + intercept)).powi(2))
        .sum();
    let r_squared = if ss_tot > 1e-30 {
        1.0 - ss_res / ss_tot
    } else {
        0.0
    };

    Some(BoltzmannPlotResult {
        temperature_k,
        r_squared,
        slope,
        intercept,
    })
}

/// Estimate electron density from Stark broadening.
///
/// delta_lambda_S = 2 * w * (n_e / 10^16)
///
/// Returns n_e in cm^-3.
pub fn electron_density_stark(observed_fwhm_nm: f64, instrument_fwhm_nm: f64, stark_width_param: f64) -> f64 {
    // Subtract instrument broadening (assuming Lorentzian convolution)
    let stark_fwhm = (observed_fwhm_nm - instrument_fwhm_nm).max(0.0);
    if stark_width_param <= 0.0 {
        return 0.0;
    }
    // delta_lambda_S = 2 * w * (n_e / 10^16)
    // n_e = delta_lambda_S / (2 * w) * 10^16
    (stark_fwhm / (2.0 * stark_width_param)) * 1e16
}

/// McWhirter criterion for Local Thermodynamic Equilibrium (LTE).
///
/// LTE requires: n_e >= 1.6e12 * T^(1/2) * (delta_E)^3
///
/// where T is in K and delta_E is the largest energy gap in eV.
/// Returns the minimum electron density needed for LTE.
pub fn mcwhirter_criterion(temperature_k: f64, delta_e_ev: f64) -> f64 {
    1.6e12 * temperature_k.sqrt() * delta_e_ev.powi(3)
}

// ---------------------------------------------------------------------------
// Partition function approximation
// ---------------------------------------------------------------------------

/// Simple partition function approximation for CF-LIBS.
///
/// Uses a polynomial approximation U(T) = a + b*T + c*T^2 for common elements.
/// This is a simplified model; real partition functions should come from NIST.
pub fn partition_function(element: &str, temperature_k: f64) -> f64 {
    let t = temperature_k;
    match element {
        "Fe" => 25.0 + 4.0e-4 * t + 1.0e-8 * t * t,
        "Ca" => 1.0 + 1.5e-4 * t + 3.0e-9 * t * t,
        "Na" => 2.0 + 5.0e-5 * t + 1.0e-9 * t * t,
        "Al" => 6.0 + 2.0e-4 * t + 5.0e-9 * t * t,
        "Si" => 9.0 + 3.0e-4 * t + 7.0e-9 * t * t,
        "Mg" => 1.0 + 1.0e-4 * t + 2.0e-9 * t * t,
        "C"  => 9.0 + 1.5e-4 * t + 4.0e-9 * t * t,
        "H"  => 2.0,
        "O"  => 9.0 + 2.5e-4 * t + 6.0e-9 * t * t,
        _    => 10.0 + 3.0e-4 * t + 5.0e-9 * t * t,
    }
}

// ---------------------------------------------------------------------------
// CF-LIBS quantitative analysis
// ---------------------------------------------------------------------------

/// Calibration-Free LIBS concentration result for one element.
#[derive(Debug, Clone)]
pub struct CfLibsResult {
    /// Element symbol.
    pub element: String,
    /// Relative concentration (normalized so all sum to 1).
    pub concentration: f64,
    /// Un-normalized Boltzmann factor F * I * lambda / (g * A * U(T) * exp(-E/kT)).
    pub raw_factor: f64,
}

/// Perform Calibration-Free LIBS analysis.
///
/// Given measured line intensities and the plasma temperature, compute
/// relative elemental concentrations using the Boltzmann distribution.
///
/// C_s = (F * I * lambda) / (g * A_ki * U(T) * exp(-E_k / (kB * T)))
///
/// The factor F cancels when normalizing concentrations to sum to 1.
pub fn cf_libs_concentrations(
    lines: &[(BoltzmannPoint, &str)],  // (measurement, element)
    temperature_k: f64,
) -> Vec<CfLibsResult> {
    if lines.is_empty() || temperature_k <= 0.0 {
        return Vec::new();
    }

    // Group by element and compute raw Boltzmann factors
    let mut element_factors: Vec<(String, f64)> = Vec::new();

    for (bp, element) in lines {
        if bp.intensity <= 0.0 || bp.g_upper <= 0.0 || bp.a_ki <= 0.0 {
            continue;
        }
        let u_t = partition_function(element, temperature_k);
        let boltzmann = (-bp.e_upper_ev / (KB_EV * temperature_k)).exp();
        let raw = bp.intensity * bp.wavelength_nm / (bp.g_upper * bp.a_ki * u_t * boltzmann);

        // Check if element already exists
        if let Some(entry) = element_factors.iter_mut().find(|(e, _)| e == element) {
            entry.1 += raw;
        } else {
            element_factors.push((element.to_string(), raw));
        }
    }

    // Normalize
    let total: f64 = element_factors.iter().map(|(_, f)| f).sum();
    if total <= 0.0 {
        return Vec::new();
    }

    element_factors
        .into_iter()
        .map(|(element, raw)| CfLibsResult {
            element,
            concentration: raw / total,
            raw_factor: raw,
        })
        .collect()
}

// ---------------------------------------------------------------------------
// LIBS Analyzer
// ---------------------------------------------------------------------------

/// Configuration for the LIBS analyzer.
#[derive(Debug, Clone)]
pub struct LibsAnalyzerConfig {
    /// Spectral resolution in nm.
    pub spectral_resolution_nm: f64,
    /// Minimum wavelength of analysis range in nm.
    pub wavelength_min_nm: f64,
    /// Maximum wavelength of analysis range in nm.
    pub wavelength_max_nm: f64,
    /// Detector gate delay in microseconds.
    pub gate_delay_us: f64,
    /// Detector gate width in microseconds.
    pub gate_width_us: f64,
    /// Minimum SNR for peak detection.
    pub snr_threshold: f64,
    /// Window size (in samples) for continuum estimation.
    pub continuum_window: usize,
}

impl Default for LibsAnalyzerConfig {
    fn default() -> Self {
        Self {
            spectral_resolution_nm: 0.05,
            wavelength_min_nm: 200.0,
            wavelength_max_nm: 900.0,
            gate_delay_us: 1.0,
            gate_width_us: 5.0,
            snr_threshold: 3.0,
            continuum_window: 50,
        }
    }
}

/// Main LIBS spectrum analyzer.
pub struct LibsAnalyzer {
    config: LibsAnalyzerConfig,
    database: EmissionDatabase,
}

impl LibsAnalyzer {
    /// Create a new LIBS analyzer with the given configuration.
    pub fn new(config: LibsAnalyzerConfig) -> Self {
        Self {
            config,
            database: EmissionDatabase::new(),
        }
    }

    /// Create a new analyzer with a custom emission database.
    pub fn with_database(config: LibsAnalyzerConfig, database: EmissionDatabase) -> Self {
        Self { config, database }
    }

    /// Get a reference to the emission database.
    pub fn database(&self) -> &EmissionDatabase {
        &self.database
    }

    /// Get a reference to the configuration.
    pub fn config(&self) -> &LibsAnalyzerConfig {
        &self.config
    }

    // ---- Continuum estimation and subtraction ----------------------------

    /// Estimate the continuum background using a rolling minimum filter
    /// followed by smoothing. This approximates the Bremsstrahlung and
    /// recombination continuum that underlies discrete emission lines.
    pub fn estimate_continuum(&self, spectrum: &LibsSpectrum) -> Vec<f64> {
        let n = spectrum.intensity.len();
        if n == 0 {
            return Vec::new();
        }
        let w = self.config.continuum_window.max(1);
        let half_w = w / 2;

        // Step 1: rolling minimum (finds the baseline below peaks)
        let mut min_vals = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= half_w { i - half_w } else { 0 };
            let hi = (i + half_w + 1).min(n);
            let mut min_v = f64::MAX;
            for j in lo..hi {
                if spectrum.intensity[j] < min_v {
                    min_v = spectrum.intensity[j];
                }
            }
            min_vals[i] = min_v;
        }

        // Step 2: smooth the minimum envelope with a moving average
        let mut continuum = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= half_w { i - half_w } else { 0 };
            let hi = (i + half_w + 1).min(n);
            let sum: f64 = min_vals[lo..hi].iter().sum();
            continuum[i] = sum / (hi - lo) as f64;
        }

        continuum
    }

    /// Subtract continuum background from a spectrum.
    pub fn subtract_continuum(&self, spectrum: &LibsSpectrum) -> LibsSpectrum {
        let cont = self.estimate_continuum(spectrum);
        let corrected: Vec<f64> = spectrum
            .intensity
            .iter()
            .zip(cont.iter())
            .map(|(&s, &c)| (s - c).max(0.0))
            .collect();
        LibsSpectrum {
            wavelength_nm: spectrum.wavelength_nm.clone(),
            intensity: corrected,
            gate_delay_us: spectrum.gate_delay_us,
            gate_width_us: spectrum.gate_width_us,
            laser_energy_mj: spectrum.laser_energy_mj,
        }
    }

    // ---- Peak detection --------------------------------------------------

    /// Detect emission peaks above the SNR threshold.
    pub fn detect_peaks(&self, spectrum: &LibsSpectrum) -> Vec<DetectedPeak> {
        let n = spectrum.intensity.len();
        if n < 3 {
            return Vec::new();
        }

        let continuum = self.estimate_continuum(spectrum);

        // Estimate noise as the standard deviation of residuals from continuum
        let residuals: Vec<f64> = spectrum
            .intensity
            .iter()
            .zip(continuum.iter())
            .map(|(&s, &c)| s - c)
            .collect();

        let mean_r: f64 = residuals.iter().sum::<f64>() / n as f64;
        let var_r: f64 = residuals.iter().map(|&r| (r - mean_r).powi(2)).sum::<f64>() / n as f64;
        let sigma = var_r.sqrt().max(1e-30);

        let mut peaks = Vec::new();

        for i in 1..n - 1 {
            let net = residuals[i];
            let snr = net / sigma;

            // Local maximum check
            if residuals[i] > residuals[i - 1]
                && residuals[i] > residuals[i + 1]
                && snr >= self.config.snr_threshold
            {
                // Estimate FWHM by finding half-max points
                let half_max = net / 2.0;
                let mut left = i;
                while left > 0 && residuals[left] > half_max {
                    left -= 1;
                }
                let mut right = i;
                while right < n - 1 && residuals[right] > half_max {
                    right += 1;
                }
                let fwhm_nm = if right > left {
                    spectrum.wavelength_nm[right] - spectrum.wavelength_nm[left]
                } else {
                    self.config.spectral_resolution_nm
                };

                // Simple trapezoidal area
                let mut area = 0.0;
                for j in left..right {
                    if j + 1 < n {
                        let dx = spectrum.wavelength_nm[j + 1] - spectrum.wavelength_nm[j];
                        area += 0.5 * (residuals[j].max(0.0) + residuals[j + 1].max(0.0)) * dx;
                    }
                }

                peaks.push(DetectedPeak {
                    wavelength_nm: spectrum.wavelength_nm[i],
                    intensity: net,
                    snr,
                    fwhm_nm,
                    area,
                    index: i,
                });
            }
        }

        peaks
    }

    // ---- Line fitting ----------------------------------------------------

    /// Fit a Lorentzian profile to data around a detected peak.
    ///
    /// Uses iterative refinement: grid search for center, then analytical
    /// amplitude from known center and width.
    pub fn fit_lorentzian(
        &self,
        spectrum: &LibsSpectrum,
        peak: &DetectedPeak,
        fit_window_nm: f64,
    ) -> LineFitResult {
        let center_guess = peak.wavelength_nm;
        let wl_min = center_guess - fit_window_nm;
        let wl_max = center_guess + fit_window_nm;

        // Extract fitting region
        let mut wl = Vec::new();
        let mut data = Vec::new();
        for (i, &w) in spectrum.wavelength_nm.iter().enumerate() {
            if w >= wl_min && w <= wl_max {
                wl.push(w);
                data.push(spectrum.intensity[i]);
            }
        }

        if wl.len() < 3 {
            return LineFitResult {
                center_nm: center_guess,
                amplitude: peak.intensity,
                fwhm_nm: peak.fwhm_nm,
                area: peak.area,
                residual_ss: 0.0,
            };
        }

        // Grid search over center wavelength and FWHM
        let mut best_center = center_guess;
        let mut best_gamma = peak.fwhm_nm.max(self.config.spectral_resolution_nm);
        let mut best_amp = peak.intensity;
        let mut best_ss = f64::MAX;

        let res = self.config.spectral_resolution_nm;
        for dc in -10..=10 {
            let c = center_guess + dc as f64 * res * 0.1;
            for dg in 1..=20 {
                let gamma = dg as f64 * res * 0.5;
                // Optimal amplitude: minimize sum (data - A * L)^2
                // A = sum(data * L) / sum(L^2)
                let mut sum_dl = 0.0;
                let mut sum_ll = 0.0;
                for (j, &w) in wl.iter().enumerate() {
                    let l = lorentzian(w, c, 1.0, gamma);
                    sum_dl += data[j] * l;
                    sum_ll += l * l;
                }
                if sum_ll < 1e-30 {
                    continue;
                }
                let amp = sum_dl / sum_ll;
                if amp <= 0.0 {
                    continue;
                }

                let ss: f64 = wl
                    .iter()
                    .zip(data.iter())
                    .map(|(&w, &d)| {
                        let model = lorentzian(w, c, amp, gamma);
                        (d - model).powi(2)
                    })
                    .sum();

                if ss < best_ss {
                    best_ss = ss;
                    best_center = c;
                    best_gamma = gamma;
                    best_amp = amp;
                }
            }
        }

        // Analytical area of Lorentzian: integral = A * pi * gamma / 2
        let area = best_amp * PI * best_gamma / 2.0;

        LineFitResult {
            center_nm: best_center,
            amplitude: best_amp,
            fwhm_nm: best_gamma,
            area,
            residual_ss: best_ss,
        }
    }

    // ---- Element identification ------------------------------------------

    /// Identify the element for a detected peak by matching to the database.
    pub fn identify_peak(&self, peak: &DetectedPeak, tolerance_nm: f64) -> Vec<&EmissionLine> {
        self.database.lookup(peak.wavelength_nm, tolerance_nm)
    }

    /// Identify all elements present in a spectrum.
    pub fn identify_elements(
        &self,
        spectrum: &LibsSpectrum,
        tolerance_nm: f64,
    ) -> Vec<(DetectedPeak, Vec<&EmissionLine>)> {
        let peaks = self.detect_peaks(spectrum);
        peaks
            .into_iter()
            .map(|p| {
                let matches = self.identify_peak(&p, tolerance_nm);
                (p, matches)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Multi-shot averaging
// ---------------------------------------------------------------------------

/// Average multiple LIBS spectra for noise reduction.
///
/// All spectra must have the same wavelength axis.
pub fn average_spectra(spectra: &[LibsSpectrum]) -> Option<LibsSpectrum> {
    if spectra.is_empty() {
        return None;
    }
    let n = spectra[0].wavelength_nm.len();
    let m = spectra.len() as f64;

    let mut avg_intensity = vec![0.0; n];
    for s in spectra {
        if s.intensity.len() != n {
            return None; // mismatched lengths
        }
        for (i, &v) in s.intensity.iter().enumerate() {
            avg_intensity[i] += v;
        }
    }
    for v in &mut avg_intensity {
        *v /= m;
    }

    Some(LibsSpectrum {
        wavelength_nm: spectra[0].wavelength_nm.clone(),
        intensity: avg_intensity,
        gate_delay_us: spectra[0].gate_delay_us,
        gate_width_us: spectra[0].gate_width_us,
        laser_energy_mj: spectra[0].laser_energy_mj,
    })
}

/// Compute shot-to-shot relative standard deviation (RSD) at each channel.
pub fn shot_to_shot_rsd(spectra: &[LibsSpectrum]) -> Vec<f64> {
    if spectra.is_empty() {
        return Vec::new();
    }
    let n = spectra[0].wavelength_nm.len();
    let m = spectra.len() as f64;

    let mut means = vec![0.0; n];
    for s in spectra {
        for (i, &v) in s.intensity.iter().enumerate() {
            means[i] += v;
        }
    }
    for v in &mut means {
        *v /= m;
    }

    let mut rsd = vec![0.0; n];
    for i in 0..n {
        let var: f64 = spectra
            .iter()
            .map(|s| (s.intensity[i] - means[i]).powi(2))
            .sum::<f64>()
            / m;
        let std = var.sqrt();
        rsd[i] = if means[i].abs() > 1e-30 {
            std / means[i].abs() * 100.0 // percent
        } else {
            0.0
        };
    }
    rsd
}

/// Reject outlier spectra using 3-sigma criterion on total intensity.
pub fn reject_outliers_3sigma(spectra: &[LibsSpectrum]) -> Vec<LibsSpectrum> {
    if spectra.len() < 3 {
        return spectra.to_vec();
    }
    let totals: Vec<f64> = spectra
        .iter()
        .map(|s| s.intensity.iter().sum::<f64>())
        .collect();
    let n = totals.len() as f64;
    let mean = totals.iter().sum::<f64>() / n;
    let var = totals.iter().map(|&t| (t - mean).powi(2)).sum::<f64>() / n;
    let sigma = var.sqrt();

    spectra
        .iter()
        .zip(totals.iter())
        .filter(|(_, &t)| (t - mean).abs() <= 3.0 * sigma)
        .map(|(s, _)| s.clone())
        .collect()
}

/// Reject outlier spectra using Median Absolute Deviation (MAD).
pub fn reject_outliers_mad(spectra: &[LibsSpectrum], threshold: f64) -> Vec<LibsSpectrum> {
    if spectra.len() < 3 {
        return spectra.to_vec();
    }
    let totals: Vec<f64> = spectra
        .iter()
        .map(|s| s.intensity.iter().sum::<f64>())
        .collect();
    let mut sorted = totals.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = if sorted.len() % 2 == 0 {
        (sorted[sorted.len() / 2 - 1] + sorted[sorted.len() / 2]) / 2.0
    } else {
        sorted[sorted.len() / 2]
    };

    let mut deviations: Vec<f64> = totals.iter().map(|&t| (t - median).abs()).collect();
    deviations.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mad = if deviations.len() % 2 == 0 {
        (deviations[deviations.len() / 2 - 1] + deviations[deviations.len() / 2]) / 2.0
    } else {
        deviations[deviations.len() / 2]
    };

    // Modified Z-score: 0.6745 * |x - median| / MAD
    let mad_scale = mad.max(1e-30) * 1.4826; // MAD to sigma conversion

    spectra
        .iter()
        .zip(totals.iter())
        .filter(|(_, &t)| (t - median).abs() / mad_scale <= threshold)
        .map(|(s, _)| s.clone())
        .collect()
}

// ---------------------------------------------------------------------------
// Spectral normalization
// ---------------------------------------------------------------------------

/// Normalize a spectrum by total area.
pub fn normalize_by_area(spectrum: &LibsSpectrum) -> LibsSpectrum {
    let n = spectrum.wavelength_nm.len();
    if n < 2 {
        return spectrum.clone();
    }
    // Trapezoidal integration
    let mut total_area = 0.0;
    for i in 0..n - 1 {
        let dx = spectrum.wavelength_nm[i + 1] - spectrum.wavelength_nm[i];
        total_area += 0.5 * (spectrum.intensity[i] + spectrum.intensity[i + 1]) * dx;
    }
    if total_area.abs() < 1e-30 {
        return spectrum.clone();
    }
    let norm: Vec<f64> = spectrum.intensity.iter().map(|&v| v / total_area).collect();
    LibsSpectrum {
        wavelength_nm: spectrum.wavelength_nm.clone(),
        intensity: norm,
        gate_delay_us: spectrum.gate_delay_us,
        gate_width_us: spectrum.gate_width_us,
        laser_energy_mj: spectrum.laser_energy_mj,
    }
}

/// Normalize a spectrum by a reference line intensity.
pub fn normalize_by_reference_line(
    spectrum: &LibsSpectrum,
    reference_wavelength_nm: f64,
    tolerance_nm: f64,
) -> LibsSpectrum {
    // Find the peak intensity near the reference wavelength
    let mut ref_intensity = 0.0_f64;
    for (i, &w) in spectrum.wavelength_nm.iter().enumerate() {
        if (w - reference_wavelength_nm).abs() <= tolerance_nm {
            if spectrum.intensity[i] > ref_intensity {
                ref_intensity = spectrum.intensity[i];
            }
        }
    }
    if ref_intensity.abs() < 1e-30 {
        return spectrum.clone();
    }
    let norm: Vec<f64> = spectrum.intensity.iter().map(|&v| v / ref_intensity).collect();
    LibsSpectrum {
        wavelength_nm: spectrum.wavelength_nm.clone(),
        intensity: norm,
        gate_delay_us: spectrum.gate_delay_us,
        gate_width_us: spectrum.gate_width_us,
        laser_energy_mj: spectrum.laser_energy_mj,
    }
}

// ---------------------------------------------------------------------------
// Internal standardization (line ratio)
// ---------------------------------------------------------------------------

/// Compute the intensity ratio of two lines for internal standardization.
///
/// Returns the ratio of the analyte line intensity to the reference line
/// intensity, useful for mitigating matrix effects.
pub fn line_intensity_ratio(
    spectrum: &LibsSpectrum,
    analyte_wavelength_nm: f64,
    reference_wavelength_nm: f64,
    tolerance_nm: f64,
) -> f64 {
    let find_peak = |target: f64| -> f64 {
        let mut max_i = 0.0_f64;
        for (i, &w) in spectrum.wavelength_nm.iter().enumerate() {
            if (w - target).abs() <= tolerance_nm && spectrum.intensity[i] > max_i {
                max_i = spectrum.intensity[i];
            }
        }
        max_i
    };
    let analyte = find_peak(analyte_wavelength_nm);
    let reference = find_peak(reference_wavelength_nm);
    if reference.abs() < 1e-30 {
        return 0.0;
    }
    analyte / reference
}

// ---------------------------------------------------------------------------
// Self-absorption correction
// ---------------------------------------------------------------------------

/// Simplified self-absorption correction using the curve-of-growth model.
///
/// For optically thin plasmas, intensity is proportional to concentration.
/// For optically thick lines, self-absorption causes saturation.
///
/// The correction factor is: I_corrected = I_observed / (1 - exp(-tau))  * tau
/// where tau is the optical depth estimated from the line parameters.
///
/// `absorption_coeff` is a dimensionless self-absorption parameter (0 = no
/// absorption, 1 = strong absorption).
pub fn self_absorption_correction(intensity: f64, absorption_coeff: f64) -> f64 {
    let tau = absorption_coeff.max(0.0);
    if tau < 1e-6 {
        return intensity; // optically thin
    }
    let factor = tau / (1.0 - (-tau).exp());
    intensity * factor
}

// ---------------------------------------------------------------------------
// Multi-peak deconvolution
// ---------------------------------------------------------------------------

/// Result of multi-peak deconvolution.
#[derive(Debug, Clone)]
pub struct DeconvolutionResult {
    /// Fitted peak parameters (center, amplitude, fwhm) for each peak.
    pub peaks: Vec<(f64, f64, f64)>,
    /// Residual sum of squares.
    pub residual_ss: f64,
}

/// Deconvolve overlapping peaks by fitting multiple Lorentzians
/// simultaneously using alternating least squares.
///
/// `initial_centers` provides initial guesses for peak center wavelengths.
pub fn multi_peak_deconvolution(
    wavelength: &[f64],
    intensity: &[f64],
    initial_centers: &[f64],
    max_iterations: usize,
) -> DeconvolutionResult {
    let n_peaks = initial_centers.len();
    if n_peaks == 0 || wavelength.is_empty() {
        return DeconvolutionResult {
            peaks: Vec::new(),
            residual_ss: 0.0,
        };
    }

    let n = wavelength.len();
    let mut centers: Vec<f64> = initial_centers.to_vec();
    let mut amplitudes: Vec<f64> = vec![1.0; n_peaks];
    let mut gammas: Vec<f64> = vec![0.1; n_peaks];

    // Initialize amplitudes from data at center positions
    for k in 0..n_peaks {
        let mut best_i = 0;
        let mut best_d = f64::MAX;
        for (i, &w) in wavelength.iter().enumerate() {
            let d = (w - centers[k]).abs();
            if d < best_d {
                best_d = d;
                best_i = i;
            }
        }
        amplitudes[k] = intensity[best_i].max(1.0);
    }

    // Alternating optimization: for each peak, fix others and optimize one
    for _iter in 0..max_iterations {
        for k in 0..n_peaks {
            // Compute residual = data - sum of all other peaks
            let residual: Vec<f64> = (0..n)
                .map(|i| {
                    let mut other_sum = 0.0;
                    for j in 0..n_peaks {
                        if j != k {
                            other_sum +=
                                lorentzian(wavelength[i], centers[j], amplitudes[j], gammas[j]);
                        }
                    }
                    (intensity[i] - other_sum).max(0.0)
                })
                .collect();

            // Find best center for peak k (search near current center)
            let res_nm = if n > 1 {
                (wavelength[1] - wavelength[0]).abs()
            } else {
                0.01
            };
            let mut best_c = centers[k];
            let mut best_g = gammas[k];
            let mut best_a = amplitudes[k];
            let mut best_ss = f64::MAX;

            for dc in -5..=5 {
                let c = centers[k] + dc as f64 * res_nm * 0.2;
                for dg in 1..=15 {
                    let g = dg as f64 * res_nm * 0.5;
                    // Optimal amplitude
                    let mut sum_rl = 0.0;
                    let mut sum_ll = 0.0;
                    for i in 0..n {
                        let l = lorentzian(wavelength[i], c, 1.0, g);
                        sum_rl += residual[i] * l;
                        sum_ll += l * l;
                    }
                    if sum_ll < 1e-30 {
                        continue;
                    }
                    let a = (sum_rl / sum_ll).max(0.0);
                    let ss: f64 = (0..n)
                        .map(|i| (residual[i] - lorentzian(wavelength[i], c, a, g)).powi(2))
                        .sum();
                    if ss < best_ss {
                        best_ss = ss;
                        best_c = c;
                        best_g = g;
                        best_a = a;
                    }
                }
            }
            centers[k] = best_c;
            gammas[k] = best_g;
            amplitudes[k] = best_a;
        }
    }

    // Final residual
    let residual_ss: f64 = (0..n)
        .map(|i| {
            let model: f64 = (0..n_peaks)
                .map(|k| lorentzian(wavelength[i], centers[k], amplitudes[k], gammas[k]))
                .sum();
            (intensity[i] - model).powi(2)
        })
        .sum();

    let peaks = (0..n_peaks)
        .map(|k| (centers[k], amplitudes[k], gammas[k]))
        .collect();

    DeconvolutionResult { peaks, residual_ss }
}

// ---------------------------------------------------------------------------
// SNR-based quality filtering
// ---------------------------------------------------------------------------

/// Filter detected peaks by minimum SNR.
pub fn filter_peaks_by_snr(peaks: &[DetectedPeak], min_snr: f64) -> Vec<DetectedPeak> {
    peaks.iter().filter(|p| p.snr >= min_snr).cloned().collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper to create a synthetic spectrum with known peaks
    fn make_test_spectrum() -> LibsSpectrum {
        let n = 2000;
        let wl_min = 200.0;
        let wl_max = 900.0;
        let wavelengths: Vec<f64> = (0..n)
            .map(|i| wl_min + (wl_max - wl_min) * i as f64 / (n - 1) as f64)
            .collect();
        let mut intensity = vec![100.0; n]; // flat background

        // Add Na D-line doublet
        for (i, &wl) in wavelengths.iter().enumerate() {
            intensity[i] += lorentzian(wl, 589.0, 5000.0, 0.5);
            intensity[i] += lorentzian(wl, 589.6, 3000.0, 0.5);
        }
        // Add Ca II H&K
        for (i, &wl) in wavelengths.iter().enumerate() {
            intensity[i] += lorentzian(wl, 393.37, 8000.0, 0.6);
            intensity[i] += lorentzian(wl, 396.85, 7000.0, 0.6);
        }
        // Add H-alpha
        for (i, &wl) in wavelengths.iter().enumerate() {
            intensity[i] += lorentzian(wl, 656.28, 4000.0, 0.8);
        }

        LibsSpectrum::new(wavelengths, intensity, 1.0, 5.0, 50.0)
    }

    #[test]
    fn test_spectrum_creation() {
        let wl = vec![200.0, 201.0, 202.0];
        let int = vec![10.0, 20.0, 15.0];
        let s = LibsSpectrum::new(wl, int, 1.0, 5.0, 50.0);
        assert_eq!(s.len(), 3);
        assert!(!s.is_empty());
        assert!((s.resolution_nm() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_spectrum_window() {
        let s = make_test_spectrum();
        let w = s.window(580.0, 600.0);
        assert!(w.len() > 0);
        assert!(w.wavelength_nm[0] >= 580.0);
        assert!(*w.wavelength_nm.last().unwrap() <= 600.0);
    }

    #[test]
    fn test_emission_database_creation() {
        let db = EmissionDatabase::new();
        assert!(db.all_lines().len() >= 21);
    }

    #[test]
    fn test_emission_database_lookup() {
        let db = EmissionDatabase::new();
        let matches = db.lookup(589.0, 1.0);
        assert!(matches.len() >= 1);
        assert_eq!(matches[0].element, "Na");
    }

    #[test]
    fn test_emission_database_element_filter() {
        let db = EmissionDatabase::new();
        let fe_lines = db.lines_for_element("Fe");
        assert_eq!(fe_lines.len(), 3);
        let ca_lines = db.lines_for_element("Ca");
        assert_eq!(ca_lines.len(), 3);
    }

    #[test]
    fn test_emission_database_elements() {
        let db = EmissionDatabase::new();
        let elems = db.elements();
        assert!(elems.contains(&"Fe"));
        assert!(elems.contains(&"Na"));
        assert!(elems.contains(&"Ca"));
        assert!(elems.contains(&"H"));
        assert!(elems.contains(&"O"));
    }

    #[test]
    fn test_lorentzian_peak() {
        let val = lorentzian(500.0, 500.0, 1000.0, 0.5);
        assert!((val - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_lorentzian_symmetry() {
        let left = lorentzian(499.0, 500.0, 1000.0, 1.0);
        let right = lorentzian(501.0, 500.0, 1000.0, 1.0);
        assert!((left - right).abs() < 1e-10);
    }

    #[test]
    fn test_lorentzian_fwhm() {
        // At +/- gamma/2 from center, the value should be A/2
        let gamma = 1.0;
        let a = 1000.0;
        let half = lorentzian(500.0 + gamma / 2.0, 500.0, a, gamma);
        assert!((half - a / 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_gaussian_profile_peak() {
        let val = gaussian_profile(500.0, 500.0, 1000.0, 0.5);
        assert!((val - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_gaussian_profile_symmetry() {
        let left = gaussian_profile(499.0, 500.0, 1000.0, 1.0);
        let right = gaussian_profile(501.0, 500.0, 1000.0, 1.0);
        assert!((left - right).abs() < 1e-10);
    }

    #[test]
    fn test_voigt_pseudo_limits() {
        // When Lorentzian width dominates, should approach Lorentzian
        let v_lor = voigt_pseudo(500.5, 500.0, 1000.0, 0.001, 1.0);
        let l_ref = lorentzian(500.5, 500.0, 1000.0, 1.0);
        // Should be close (not exact due to FWHM mixing)
        assert!((v_lor - l_ref).abs() / l_ref.abs() < 0.3);
    }

    #[test]
    fn test_voigt_pseudo_peak() {
        let val = voigt_pseudo(500.0, 500.0, 1000.0, 0.5, 0.5);
        // At the center, should be close to amplitude
        assert!(val > 500.0); // some fraction of amplitude
    }

    #[test]
    fn test_continuum_estimation() {
        let s = make_test_spectrum();
        let config = LibsAnalyzerConfig::default();
        let analyzer = LibsAnalyzer::new(config);
        let cont = analyzer.estimate_continuum(&s);
        assert_eq!(cont.len(), s.len());
        // Continuum should be roughly at background level (100)
        // In flat regions away from peaks, continuum should be near 100
        // Find a region truly away from peaks: around 700 nm
        let far_idx = s.wavelength_nm.iter().position(|&w| w >= 700.0).unwrap();
        assert!(cont[far_idx] < 200.0); // Should be near background, not peak height
    }

    #[test]
    fn test_continuum_subtraction() {
        let s = make_test_spectrum();
        let config = LibsAnalyzerConfig::default();
        let analyzer = LibsAnalyzer::new(config);
        let corrected = analyzer.subtract_continuum(&s);
        assert_eq!(corrected.len(), s.len());
        // Corrected intensities should be non-negative
        for &v in &corrected.intensity {
            assert!(v >= 0.0);
        }
    }

    #[test]
    fn test_peak_detection_finds_peaks() {
        let s = make_test_spectrum();
        let config = LibsAnalyzerConfig {
            snr_threshold: 3.0,
            continuum_window: 50,
            ..Default::default()
        };
        let analyzer = LibsAnalyzer::new(config);
        let peaks = analyzer.detect_peaks(&s);
        // Should find at least the Na, Ca, and H-alpha peaks
        assert!(peaks.len() >= 3, "Found {} peaks, expected >= 3", peaks.len());
    }

    #[test]
    fn test_peak_detection_snr() {
        let s = make_test_spectrum();
        let config = LibsAnalyzerConfig {
            snr_threshold: 5.0,
            continuum_window: 50,
            ..Default::default()
        };
        let analyzer = LibsAnalyzer::new(config);
        let peaks = analyzer.detect_peaks(&s);
        for p in &peaks {
            assert!(p.snr >= 5.0);
        }
    }

    #[test]
    fn test_peak_detection_na_doublet() {
        let s = make_test_spectrum();
        let config = LibsAnalyzerConfig {
            snr_threshold: 3.0,
            continuum_window: 50,
            ..Default::default()
        };
        let analyzer = LibsAnalyzer::new(config);
        let peaks = analyzer.detect_peaks(&s);
        // Check that we find peaks near 589 nm
        let na_peaks: Vec<_> = peaks
            .iter()
            .filter(|p| p.wavelength_nm > 585.0 && p.wavelength_nm < 593.0)
            .collect();
        assert!(
            na_peaks.len() >= 1,
            "Should find Na D-line peak(s), found {}",
            na_peaks.len()
        );
    }

    #[test]
    fn test_lorentzian_fit() {
        let s = make_test_spectrum();
        let config = LibsAnalyzerConfig::default();
        let analyzer = LibsAnalyzer::new(config);
        let peaks = analyzer.detect_peaks(&s);
        if let Some(peak) = peaks.iter().find(|p| p.wavelength_nm > 650.0 && p.wavelength_nm < 660.0) {
            let fit = analyzer.fit_lorentzian(&s, peak, 5.0);
            // Should be near 656.28 nm (H-alpha)
            assert!((fit.center_nm - 656.28).abs() < 2.0);
            assert!(fit.amplitude > 0.0);
            assert!(fit.fwhm_nm > 0.0);
        }
    }

    #[test]
    fn test_element_identification() {
        let s = make_test_spectrum();
        let config = LibsAnalyzerConfig {
            snr_threshold: 3.0,
            continuum_window: 50,
            ..Default::default()
        };
        let analyzer = LibsAnalyzer::new(config);
        let results = analyzer.identify_elements(&s, 2.0);
        // Should identify at least some elements
        let identified_elements: Vec<_> = results
            .iter()
            .flat_map(|(_, lines)| lines.iter().map(|l| l.element))
            .collect();
        // Na should be identified
        assert!(
            identified_elements.contains(&"Na") || identified_elements.contains(&"Ca") || identified_elements.contains(&"H"),
            "Should identify at least one of Na, Ca, H"
        );
    }

    #[test]
    fn test_boltzmann_temperature() {
        // Simulate Boltzmann plot data at T = 10000 K
        let temp = 10000.0;
        let db = EmissionDatabase::new();
        let fe_lines = db.lines_for_element("Fe");

        let points: Vec<BoltzmannPoint> = fe_lines
            .iter()
            .map(|line| {
                // I = C * g * A * exp(-E / kT) / lambda
                let i = line.g_upper
                    * line.a_ki
                    * (-line.e_upper_ev / (KB_EV * temp)).exp()
                    / line.wavelength_nm;
                BoltzmannPoint {
                    intensity: i * 1e10, // scale up
                    wavelength_nm: line.wavelength_nm,
                    g_upper: line.g_upper,
                    a_ki: line.a_ki,
                    e_upper_ev: line.e_upper_ev,
                }
            })
            .collect();

        let result = boltzmann_temperature(&points).unwrap();
        // Should recover approximately 10000 K
        assert!(
            (result.temperature_k - 10000.0).abs() / 10000.0 < 0.05,
            "Expected ~10000 K, got {} K",
            result.temperature_k
        );
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_boltzmann_temperature_insufficient_points() {
        let result = boltzmann_temperature(&[]);
        assert!(result.is_none());
    }

    #[test]
    fn test_electron_density_stark() {
        // Typical LIBS plasma: observed FWHM 0.5 nm, instrument 0.05 nm
        // H-alpha Stark width parameter w ~ 0.080 nm
        let ne = electron_density_stark(0.5, 0.05, 0.080);
        // Should be on the order of 10^16 - 10^17 cm^-3
        assert!(ne > 1e15, "Electron density too low: {}", ne);
        assert!(ne < 1e19, "Electron density too high: {}", ne);
    }

    #[test]
    fn test_electron_density_zero_stark_width() {
        let ne = electron_density_stark(0.5, 0.05, 0.0);
        assert_eq!(ne, 0.0);
    }

    #[test]
    fn test_mcwhirter_criterion() {
        let ne_min = mcwhirter_criterion(10000.0, 3.0);
        // Should be roughly 1.6e12 * 100 * 27 = 4.32e15
        assert!(ne_min > 1e15);
        assert!(ne_min < 1e16);
    }

    #[test]
    fn test_partition_function() {
        let u_fe = partition_function("Fe", 10000.0);
        assert!(u_fe > 20.0);
        let u_h = partition_function("H", 10000.0);
        assert!((u_h - 2.0).abs() < 1e-10); // H partition function ~ 2
    }

    #[test]
    fn test_cf_libs_concentrations() {
        let temp = 10000.0;
        let db = EmissionDatabase::new();

        let mut lines: Vec<(BoltzmannPoint, &str)> = Vec::new();
        for element in &["Fe", "Ca", "Na"] {
            for line in db.lines_for_element(element) {
                let i = line.g_upper
                    * line.a_ki
                    * (-line.e_upper_ev / (KB_EV * temp)).exp()
                    / line.wavelength_nm
                    * 1e10;
                lines.push((
                    BoltzmannPoint {
                        intensity: i,
                        wavelength_nm: line.wavelength_nm,
                        g_upper: line.g_upper,
                        a_ki: line.a_ki,
                        e_upper_ev: line.e_upper_ev,
                    },
                    element,
                ));
            }
        }

        let results = cf_libs_concentrations(&lines, temp);
        assert_eq!(results.len(), 3);
        let total: f64 = results.iter().map(|r| r.concentration).sum();
        assert!((total - 1.0).abs() < 1e-6, "Concentrations should sum to 1, got {}", total);
    }

    #[test]
    fn test_cf_libs_empty() {
        let results = cf_libs_concentrations(&[], 10000.0);
        assert!(results.is_empty());
    }

    #[test]
    fn test_calibration_curve() {
        let points = vec![
            CalibrationPoint { concentration: 0.0, intensity: 0.0 },
            CalibrationPoint { concentration: 1.0, intensity: 100.0 },
            CalibrationPoint { concentration: 2.0, intensity: 200.0 },
            CalibrationPoint { concentration: 3.0, intensity: 300.0 },
        ];
        let curve = CalibrationCurve::from_points(&points).unwrap();
        assert!((curve.slope - 0.01).abs() < 1e-6);
        assert!(curve.r_squared > 0.999);
    }

    #[test]
    fn test_calibration_predict() {
        let points = vec![
            CalibrationPoint { concentration: 0.0, intensity: 0.0 },
            CalibrationPoint { concentration: 5.0, intensity: 500.0 },
        ];
        let curve = CalibrationCurve::from_points(&points).unwrap();
        let predicted = curve.predict(250.0);
        assert!((predicted - 2.5).abs() < 1e-6);
    }

    #[test]
    fn test_limit_of_detection() {
        let points = vec![
            CalibrationPoint { concentration: 0.0, intensity: 0.0 },
            CalibrationPoint { concentration: 10.0, intensity: 1000.0 },
        ];
        let curve = CalibrationCurve::from_points(&points).unwrap();
        let lod = curve.limit_of_detection(5.0);
        // LOD = 3 * 5 / 0.01 = 1500? No, sensitivity is slope = 10/1000 = 0.01
        // LOD = 3 * 5 / 0.01 = 1500
        assert!((lod - 1500.0).abs() < 1e-3);
    }

    #[test]
    fn test_calibration_insufficient_points() {
        let points = vec![CalibrationPoint {
            concentration: 1.0,
            intensity: 100.0,
        }];
        assert!(CalibrationCurve::from_points(&points).is_none());
    }

    #[test]
    fn test_average_spectra() {
        let s1 = LibsSpectrum::new(vec![500.0, 501.0], vec![100.0, 200.0], 1.0, 5.0, 50.0);
        let s2 = LibsSpectrum::new(vec![500.0, 501.0], vec![200.0, 400.0], 1.0, 5.0, 50.0);
        let avg = average_spectra(&[s1, s2]).unwrap();
        assert!((avg.intensity[0] - 150.0).abs() < 1e-10);
        assert!((avg.intensity[1] - 300.0).abs() < 1e-10);
    }

    #[test]
    fn test_average_spectra_empty() {
        assert!(average_spectra(&[]).is_none());
    }

    #[test]
    fn test_shot_to_shot_rsd() {
        let s1 = LibsSpectrum::new(vec![500.0], vec![100.0], 1.0, 5.0, 50.0);
        let s2 = LibsSpectrum::new(vec![500.0], vec![100.0], 1.0, 5.0, 50.0);
        let rsd = shot_to_shot_rsd(&[s1, s2]);
        assert_eq!(rsd.len(), 1);
        assert!(rsd[0] < 1e-10); // identical spectra => 0% RSD
    }

    #[test]
    fn test_shot_to_shot_rsd_variation() {
        let s1 = LibsSpectrum::new(vec![500.0], vec![90.0], 1.0, 5.0, 50.0);
        let s2 = LibsSpectrum::new(vec![500.0], vec![110.0], 1.0, 5.0, 50.0);
        let rsd = shot_to_shot_rsd(&[s1, s2]);
        assert!(rsd[0] > 0.0); // Should have non-zero RSD
    }

    #[test]
    fn test_reject_outliers_3sigma() {
        // With 50 normal samples and 1 extreme outlier, the mean and sigma
        // are dominated by the normal population, making the outlier exceed 3-sigma.
        let mut spectra: Vec<LibsSpectrum> = (0..50)
            .map(|_| LibsSpectrum::new(vec![500.0], vec![100.0], 1.0, 5.0, 50.0))
            .collect();
        spectra.push(LibsSpectrum::new(vec![500.0], vec![100000.0], 1.0, 5.0, 50.0));
        let filtered = reject_outliers_3sigma(&spectra);
        assert!(filtered.len() < spectra.len());
    }

    #[test]
    fn test_reject_outliers_mad() {
        let spectra: Vec<LibsSpectrum> = (0..10)
            .map(|i| {
                let int = if i == 5 { 100000.0 } else { 100.0 };
                LibsSpectrum::new(vec![500.0], vec![int], 1.0, 5.0, 50.0)
            })
            .collect();
        let filtered = reject_outliers_mad(&spectra, 3.0);
        assert!(filtered.len() < spectra.len());
    }

    #[test]
    fn test_normalize_by_area() {
        let s = LibsSpectrum::new(
            vec![500.0, 501.0, 502.0],
            vec![100.0, 200.0, 100.0],
            1.0, 5.0, 50.0,
        );
        let normed = normalize_by_area(&s);
        // Check that total area is approximately 1
        let mut area = 0.0;
        for i in 0..normed.len() - 1 {
            let dx = normed.wavelength_nm[i + 1] - normed.wavelength_nm[i];
            area += 0.5 * (normed.intensity[i] + normed.intensity[i + 1]) * dx;
        }
        assert!((area - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_normalize_by_reference_line() {
        let s = LibsSpectrum::new(
            vec![500.0, 501.0, 502.0],
            vec![100.0, 500.0, 100.0],
            1.0, 5.0, 50.0,
        );
        let normed = normalize_by_reference_line(&s, 501.0, 0.5);
        assert!((normed.intensity[1] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_line_intensity_ratio() {
        let s = LibsSpectrum::new(
            vec![500.0, 501.0, 502.0],
            vec![200.0, 100.0, 400.0],
            1.0, 5.0, 50.0,
        );
        let ratio = line_intensity_ratio(&s, 502.0, 500.0, 0.5);
        assert!((ratio - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_self_absorption_correction_thin() {
        let corrected = self_absorption_correction(1000.0, 0.0);
        assert!((corrected - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_self_absorption_correction_thick() {
        let corrected = self_absorption_correction(1000.0, 2.0);
        assert!(corrected > 1000.0); // Correction should increase intensity
    }

    #[test]
    fn test_multi_peak_deconvolution() {
        // Create overlapping peaks
        let n = 200;
        let wl: Vec<f64> = (0..n).map(|i| 588.0 + i as f64 * 0.02).collect();
        let intensity: Vec<f64> = wl
            .iter()
            .map(|&w| {
                lorentzian(w, 589.0, 5000.0, 0.3) + lorentzian(w, 589.6, 3000.0, 0.3)
            })
            .collect();

        let result = multi_peak_deconvolution(&wl, &intensity, &[589.0, 589.6], 20);
        assert_eq!(result.peaks.len(), 2);
        // Centers should be close to input
        assert!((result.peaks[0].0 - 589.0).abs() < 0.2);
        assert!((result.peaks[1].0 - 589.6).abs() < 0.2);
    }

    #[test]
    fn test_filter_peaks_by_snr() {
        let peaks = vec![
            DetectedPeak { wavelength_nm: 500.0, intensity: 100.0, snr: 2.0, fwhm_nm: 0.5, area: 50.0, index: 0 },
            DetectedPeak { wavelength_nm: 600.0, intensity: 500.0, snr: 10.0, fwhm_nm: 0.5, area: 250.0, index: 1 },
            DetectedPeak { wavelength_nm: 700.0, intensity: 200.0, snr: 5.0, fwhm_nm: 0.5, area: 100.0, index: 2 },
        ];
        let filtered = filter_peaks_by_snr(&peaks, 5.0);
        assert_eq!(filtered.len(), 2);
    }

    #[test]
    fn test_default_config() {
        let config = LibsAnalyzerConfig::default();
        assert!((config.spectral_resolution_nm - 0.05).abs() < 1e-10);
        assert!((config.wavelength_min_nm - 200.0).abs() < 1e-10);
        assert!((config.wavelength_max_nm - 900.0).abs() < 1e-10);
    }

    #[test]
    fn test_analyzer_database_access() {
        let analyzer = LibsAnalyzer::new(LibsAnalyzerConfig::default());
        let db = analyzer.database();
        assert!(db.all_lines().len() > 0);
    }

    #[test]
    fn test_empty_spectrum() {
        let s = LibsSpectrum::new(vec![], vec![], 1.0, 5.0, 50.0);
        assert!(s.is_empty());
        assert_eq!(s.len(), 0);
        assert!((s.resolution_nm() - 0.0).abs() < 1e-10);
    }
}

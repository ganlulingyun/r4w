//! Fiber Bragg Grating (FBG) interrogation system for optical fiber sensing.
//!
//! This module implements a complete FBG interrogation pipeline for measuring
//! strain and temperature from reflected wavelength shifts in optical fiber
//! sensors. Fiber Bragg Gratings are periodic refractive index modulations
//! inscribed in an optical fiber core that reflect a narrow band of light
//! centered at the Bragg wavelength λ_B = 2 · n_eff · Λ.
//!
//! # Features
//!
//! - **Peak detection** with configurable threshold in reflected spectra
//! - **Gaussian/parabolic peak fitting** for sub-sample wavelength resolution
//! - **Strain measurement** from wavelength shifts (typ. 1.2 pm/με)
//! - **Temperature measurement** from wavelength shifts (typ. 10 pm/°C)
//! - **Spectrum generation** for simulation of Gaussian-shaped FBG reflections
//! - **Wavelength-division multiplexing** of multiple gratings
//! - **Strain-optic coefficient** computation from photoelastic constants
//!
//! # Physical Background
//!
//! When an FBG is subjected to strain or temperature change, the Bragg
//! wavelength shifts according to:
//!
//! ```text
//! Δλ_B / λ_B = (1 - p_e) · ε + (α + ξ) · ΔT
//! ```
//!
//! where p_e is the strain-optic coefficient, ε is the applied strain,
//! α is the thermal expansion coefficient, and ξ is the thermo-optic
//! coefficient.
//!
//! # Example
//!
//! ```
//! use r4w_core::fiber_bragg_grating_interrogator::{
//!     FbgConfig, FbgInterrogator, generate_fbg_spectrum, bragg_wavelength,
//! };
//!
//! // Define a sensor array with two gratings
//! let config = FbgConfig {
//!     num_gratings: 2,
//!     nominal_wavelengths_nm: vec![1545.0, 1555.0],
//!     sample_rate_hz: 1000.0,
//!     strain_sensitivity_pm_per_ue: 1.2,
//!     temp_sensitivity_pm_per_c: 10.0,
//! };
//!
//! let mut interrogator = FbgInterrogator::new(config);
//!
//! // Generate a synthetic spectrum with two peaks
//! let wavelengths: Vec<f64> = (0..10000)
//!     .map(|i| 1540.0 + i as f64 * 0.002)
//!     .collect();
//!
//! let spec1 = generate_fbg_spectrum(1545.05, 200.0, 0.9, &wavelengths);
//! let spec2 = generate_fbg_spectrum(1555.10, 200.0, 0.85, &wavelengths);
//! let power: Vec<f64> = spec1.iter().zip(&spec2)
//!     .map(|(a, b)| a + b)
//!     .collect();
//! // Convert to dBm for processing
//! let power_dbm: Vec<f64> = power.iter()
//!     .map(|&p| if p > 1e-15 { 10.0 * p.log10() } else { -150.0 })
//!     .collect();
//!
//! let readings = interrogator.process_spectrum(&wavelengths, &power_dbm);
//! assert_eq!(readings.len(), 2);
//! ```

// ---------------------------------------------------------------------------
// Core data structures
// ---------------------------------------------------------------------------

/// Configuration for an FBG interrogation system.
///
/// Describes the sensor array geometry and measurement sensitivities used to
/// convert detected wavelength shifts into physical quantities.
#[derive(Debug, Clone)]
pub struct FbgConfig {
    /// Number of gratings in the sensor array.
    pub num_gratings: usize,
    /// Nominal (unstrained, reference) Bragg wavelength of each grating in nm.
    pub nominal_wavelengths_nm: Vec<f64>,
    /// Interrogation sampling rate in Hz (how often spectra are acquired).
    pub sample_rate_hz: f64,
    /// Strain sensitivity in pm per microstrain (typical: 1.2 pm/με at 1550 nm).
    pub strain_sensitivity_pm_per_ue: f64,
    /// Temperature sensitivity in pm per °C (typical: 10 pm/°C at 1550 nm).
    pub temp_sensitivity_pm_per_c: f64,
}

impl Default for FbgConfig {
    fn default() -> Self {
        Self {
            num_gratings: 1,
            nominal_wavelengths_nm: vec![1550.0],
            sample_rate_hz: 1000.0,
            strain_sensitivity_pm_per_ue: 1.2,
            temp_sensitivity_pm_per_c: 10.0,
        }
    }
}

/// A single measurement reading from one FBG sensor.
#[derive(Debug, Clone)]
pub struct FbgReading {
    /// Index of the grating in the sensor array (0-based).
    pub grating_index: usize,
    /// Measured peak wavelength in nm.
    pub peak_wavelength_nm: f64,
    /// Wavelength shift from nominal in pm (1 pm = 0.001 nm).
    pub wavelength_shift_pm: f64,
    /// Estimated strain in microstrain (με), assuming pure strain loading.
    pub estimated_strain_ue: f64,
    /// Estimated temperature change in °C, assuming pure thermal loading.
    pub estimated_temp_delta_c: f64,
    /// Peak reflected power in dBm.
    pub peak_power_dbm: f64,
    /// Full-width at half-maximum of the reflection peak in pm.
    pub fwhm_pm: f64,
}

/// A detected peak in an optical spectrum.
#[derive(Debug, Clone)]
pub struct SpectralPeak {
    /// Fitted center wavelength in nm.
    pub center_wavelength_nm: f64,
    /// Peak power level in dBm.
    pub peak_power_dbm: f64,
    /// Full-width at half-maximum in pm.
    pub fwhm_pm: f64,
    /// Left index of the peak region in the spectrum array.
    pub left_idx: usize,
    /// Right index of the peak region in the spectrum array.
    pub right_idx: usize,
}

// ---------------------------------------------------------------------------
// FBG Interrogator
// ---------------------------------------------------------------------------

/// FBG interrogation engine that processes optical spectra into strain/temperature
/// measurements.
///
/// The interrogator maintains configuration for the sensor array and provides
/// spectrum-to-reading conversion through peak detection, Gaussian fitting,
/// and calibrated wavelength-to-physical-quantity mapping.
#[derive(Debug, Clone)]
pub struct FbgInterrogator {
    config: FbgConfig,
    /// Detection threshold in dB below the spectrum maximum for peak finding.
    threshold_db: f64,
}

impl FbgInterrogator {
    /// Create a new interrogator from the given configuration.
    ///
    /// The default peak detection threshold is 20 dB below the maximum spectral
    /// power level.
    pub fn new(config: FbgConfig) -> Self {
        Self {
            config,
            threshold_db: 20.0,
        }
    }

    /// Set the peak detection threshold in dB below the spectrum maximum.
    pub fn set_threshold_db(&mut self, threshold_db: f64) {
        self.threshold_db = threshold_db;
    }

    /// Process a reflected optical spectrum and produce readings for each grating.
    ///
    /// `wavelength_nm` and `power_dbm` must have the same length and represent
    /// the interrogated reflection spectrum. The wavelength axis must be
    /// monotonically increasing.
    ///
    /// Returns a vector of `FbgReading`, one per detected grating. Gratings
    /// are matched to detected peaks by proximity to their nominal wavelengths.
    pub fn process_spectrum(
        &mut self,
        wavelength_nm: &[f64],
        power_dbm: &[f64],
    ) -> Vec<FbgReading> {
        assert_eq!(
            wavelength_nm.len(),
            power_dbm.len(),
            "wavelength and power arrays must have equal length"
        );
        if wavelength_nm.is_empty() {
            return Vec::new();
        }

        // Detect peaks in the spectrum.
        let peaks = find_peaks_in_spectrum(wavelength_nm, power_dbm, self.threshold_db);

        // Match each nominal grating to the nearest detected peak.
        let mut readings = Vec::with_capacity(self.config.num_gratings);
        for (gi, &nominal_wl) in self.config.nominal_wavelengths_nm.iter().enumerate() {
            if gi >= self.config.num_gratings {
                break;
            }
            // Find the closest peak to this grating's nominal wavelength.
            let best_peak = peaks.iter().min_by(|a, b| {
                let da = (a.center_wavelength_nm - nominal_wl).abs();
                let db = (b.center_wavelength_nm - nominal_wl).abs();
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            });

            if let Some(peak) = best_peak {
                let shift_pm = (peak.center_wavelength_nm - nominal_wl) * 1000.0; // nm -> pm
                readings.push(FbgReading {
                    grating_index: gi,
                    peak_wavelength_nm: peak.center_wavelength_nm,
                    wavelength_shift_pm: shift_pm,
                    estimated_strain_ue: wavelength_to_strain(
                        shift_pm,
                        self.config.strain_sensitivity_pm_per_ue,
                    ),
                    estimated_temp_delta_c: wavelength_to_temperature(
                        shift_pm,
                        self.config.temp_sensitivity_pm_per_c,
                    ),
                    peak_power_dbm: peak.peak_power_dbm,
                    fwhm_pm: peak.fwhm_pm,
                });
            }
        }

        readings
    }
}

// ---------------------------------------------------------------------------
// Peak detection
// ---------------------------------------------------------------------------

/// Find peaks in an optical spectrum above a relative threshold.
///
/// Peaks are detected as local maxima in the dBm-domain power spectrum that
/// exceed `max_power - threshold_db`. For each detected peak, a Gaussian fit
/// refines the center wavelength and FWHM.
///
/// # Arguments
///
/// * `wavelength` - Wavelength axis in nm (monotonically increasing).
/// * `power` - Power in dBm corresponding to each wavelength sample.
/// * `threshold_db` - Detection threshold: peaks must be within this many dB
///   of the spectrum maximum.
pub fn find_peaks_in_spectrum(
    wavelength: &[f64],
    power: &[f64],
    threshold_db: f64,
) -> Vec<SpectralPeak> {
    if wavelength.len() < 3 || power.len() < 3 {
        return Vec::new();
    }
    let n = wavelength.len().min(power.len());

    // Find the global maximum power level.
    let max_power = power[..n]
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    let floor = max_power - threshold_db;

    // Identify local maxima above the threshold.
    let mut peak_indices: Vec<usize> = Vec::new();
    for i in 1..n - 1 {
        if power[i] > power[i - 1] && power[i] > power[i + 1] && power[i] >= floor {
            peak_indices.push(i);
        }
    }

    // Also check endpoints if they are the absolute maximum.
    if n >= 2 && power[0] > power[1] && power[0] >= floor {
        peak_indices.insert(0, 0);
    }
    if n >= 2 && power[n - 1] > power[n - 2] && power[n - 1] >= floor {
        peak_indices.push(n - 1);
    }

    // Convert power from dBm to linear for Gaussian fitting.
    let power_linear: Vec<f64> = power[..n].iter().map(|&p| dbm_to_linear(p)).collect();

    let mut peaks = Vec::with_capacity(peak_indices.len());
    for &pi in &peak_indices {
        // Gaussian fit for sub-sample center wavelength.
        let (center_wl, _amplitude, sigma) =
            gaussian_peak_fit(wavelength, &power_linear, pi);

        // FWHM from Gaussian sigma: FWHM = 2 * sqrt(2 * ln(2)) * sigma
        let fwhm_nm = 2.0 * (2.0_f64 * 2.0_f64.ln()).sqrt() * sigma.abs();
        let fwhm_pm = fwhm_nm * 1000.0;

        // Determine peak region bounds (where power drops below half-max in linear).
        let half_max_linear = power_linear[pi] / 2.0;
        let mut left = pi;
        while left > 0 && power_linear[left - 1] > half_max_linear {
            left -= 1;
        }
        let mut right = pi;
        while right < n - 1 && power_linear[right + 1] > half_max_linear {
            right += 1;
        }

        peaks.push(SpectralPeak {
            center_wavelength_nm: center_wl,
            peak_power_dbm: power[pi],
            fwhm_pm,
            left_idx: left,
            right_idx: right,
        });
    }

    peaks
}

// ---------------------------------------------------------------------------
// Gaussian / parabolic peak interpolation
// ---------------------------------------------------------------------------

/// Perform parabolic (Gaussian in log-domain) interpolation around a peak index
/// to find the sub-sample center wavelength, amplitude, and Gaussian sigma.
///
/// Uses the three points at `peak_idx - 1`, `peak_idx`, `peak_idx + 1` in the
/// log-amplitude domain to fit a parabola, which is equivalent to fitting a
/// Gaussian in the linear domain.
///
/// # Returns
///
/// `(center_wavelength_nm, amplitude_linear, sigma_nm)` where sigma is the
/// Gaussian standard deviation of the peak shape.
pub fn gaussian_peak_fit(
    wavelength: &[f64],
    power_linear: &[f64],
    peak_idx: usize,
) -> (f64, f64, f64) {
    let n = wavelength.len().min(power_linear.len());
    if peak_idx == 0 || peak_idx >= n - 1 {
        // Cannot interpolate at the boundary; return raw values.
        let amp = if peak_idx < n {
            power_linear[peak_idx]
        } else {
            0.0
        };
        let wl = if peak_idx < wavelength.len() {
            wavelength[peak_idx]
        } else {
            0.0
        };
        // Estimate sigma from neighboring points if available.
        let sigma = if n >= 2 {
            (wavelength[1] - wavelength[0]).abs()
        } else {
            0.001
        };
        return (wl, amp, sigma);
    }

    let y_m1 = safe_ln(power_linear[peak_idx - 1]);
    let y_0 = safe_ln(power_linear[peak_idx]);
    let y_p1 = safe_ln(power_linear[peak_idx + 1]);

    let wl_m1 = wavelength[peak_idx - 1];
    let wl_0 = wavelength[peak_idx];
    let wl_p1 = wavelength[peak_idx + 1];

    // Parabolic interpolation in log domain.
    // For equally spaced points: delta = 0.5 * (y_p1 - y_m1) / (2*y_0 - y_m1 - y_p1)
    let denom = 2.0 * y_0 - y_m1 - y_p1;
    if denom.abs() < 1e-30 {
        // Flat peak; no interpolation possible.
        return (wl_0, power_linear[peak_idx], (wl_p1 - wl_m1).abs() / 2.0);
    }

    let step = (wl_p1 - wl_m1) / 2.0;
    let fractional_offset = 0.5 * (y_p1 - y_m1) / denom;
    let center_wl = wl_0 + fractional_offset * step;

    // Amplitude at the interpolated center.
    let log_amp = y_0 + 0.25 * (y_p1 - y_m1) * fractional_offset;
    let amplitude = log_amp.exp();

    // Gaussian sigma from the curvature of the parabola.
    // In log domain: y = a - (x - x0)^2 / (2*sigma^2)
    // Curvature: d^2y/dx^2 = -1/sigma^2
    // For equally spaced points: curvature ≈ (y_m1 - 2*y_0 + y_p1) / step^2
    let curvature = (y_m1 - 2.0 * y_0 + y_p1) / (step * step);
    let sigma = if curvature < -1e-30 {
        (-1.0 / curvature).sqrt()
    } else {
        step // fallback
    };

    (center_wl, amplitude, sigma)
}

// ---------------------------------------------------------------------------
// Physical conversion functions
// ---------------------------------------------------------------------------

/// Convert a wavelength shift to microstrain.
///
/// # Arguments
///
/// * `shift_pm` - Wavelength shift in picometers.
/// * `sensitivity_pm_per_ue` - Strain sensitivity in pm/με (typical: 1.2).
///
/// # Returns
///
/// Strain in microstrain (με).
pub fn wavelength_to_strain(shift_pm: f64, sensitivity_pm_per_ue: f64) -> f64 {
    if sensitivity_pm_per_ue.abs() < 1e-30 {
        return 0.0;
    }
    shift_pm / sensitivity_pm_per_ue
}

/// Convert a wavelength shift to temperature change.
///
/// # Arguments
///
/// * `shift_pm` - Wavelength shift in picometers.
/// * `sensitivity_pm_per_c` - Temperature sensitivity in pm/°C (typical: 10.0).
///
/// # Returns
///
/// Temperature change in °C.
pub fn wavelength_to_temperature(shift_pm: f64, sensitivity_pm_per_c: f64) -> f64 {
    if sensitivity_pm_per_c.abs() < 1e-30 {
        return 0.0;
    }
    shift_pm / sensitivity_pm_per_c
}

/// Compute the Bragg wavelength from effective refractive index and grating period.
///
/// The fundamental Bragg condition for first-order reflection is:
///
/// ```text
/// λ_B = 2 · n_eff · Λ
/// ```
///
/// # Arguments
///
/// * `n_eff` - Effective refractive index of the fiber mode (typically ~1.465).
/// * `period_nm` - Grating period Λ in nm (typically ~530 nm for 1550 nm Bragg).
///
/// # Returns
///
/// Bragg wavelength in nm.
pub fn bragg_wavelength(n_eff: f64, period_nm: f64) -> f64 {
    2.0 * n_eff * period_nm
}

/// Compute the effective strain-optic coefficient from photoelastic constants.
///
/// ```text
/// p_e = (n² / 2) · [p12 - ν · (p11 + p12)]
/// ```
///
/// For standard silica fiber: p11 ≈ 0.121, p12 ≈ 0.270, ν ≈ 0.17, n ≈ 1.465
/// giving p_e ≈ 0.216.
///
/// # Arguments
///
/// * `p11` - Pockel's coefficient p11.
/// * `p12` - Pockel's coefficient p12.
/// * `nu` - Poisson's ratio ν of the fiber material.
///
/// # Returns
///
/// Effective strain-optic coefficient p_e (dimensionless, typically ~0.22).
pub fn strain_optic_coefficient(p11: f64, p12: f64, nu: f64) -> f64 {
    // Standard silica fiber refractive index.
    let n = 1.465;
    (n * n / 2.0) * (p12 - nu * (p11 + p12))
}

/// Compute the strain-optic coefficient with an explicit refractive index.
pub fn strain_optic_coefficient_with_n(p11: f64, p12: f64, nu: f64, n: f64) -> f64 {
    (n * n / 2.0) * (p12 - nu * (p11 + p12))
}

// ---------------------------------------------------------------------------
// Spectrum generation and multiplexing
// ---------------------------------------------------------------------------

/// Generate a Gaussian-shaped FBG reflection spectrum.
///
/// Models the reflected power as a Gaussian function of wavelength centered
/// at `center_nm` with the given `fwhm_pm` and peak `reflectivity` (0..1).
///
/// # Arguments
///
/// * `center_nm` - Center (Bragg) wavelength of the grating in nm.
/// * `fwhm_pm` - Full-width at half-maximum of the reflection peak in pm.
/// * `reflectivity` - Peak reflectivity (0.0 to 1.0).
/// * `wavelength_range` - Array of wavelength points in nm at which to evaluate.
///
/// # Returns
///
/// Vector of linear power values (reflectivity) at each wavelength point.
pub fn generate_fbg_spectrum(
    center_nm: f64,
    fwhm_pm: f64,
    reflectivity: f64,
    wavelength_range: &[f64],
) -> Vec<f64> {
    // Convert FWHM from pm to nm.
    let fwhm_nm = fwhm_pm / 1000.0;
    // Gaussian sigma from FWHM: sigma = FWHM / (2 * sqrt(2 * ln(2)))
    let sigma = fwhm_nm / (2.0 * (2.0_f64 * 2.0_f64.ln()).sqrt());

    wavelength_range
        .iter()
        .map(|&wl| {
            let delta = wl - center_nm;
            reflectivity * (-0.5 * (delta / sigma).powi(2)).exp()
        })
        .collect()
}

/// Combine reflection spectra from multiple FBG sensors.
///
/// Each grating is described by a tuple `(center_nm, fwhm_pm, reflectivity)`.
/// The combined spectrum is the sum of individual Gaussian reflections, which
/// models an incoherent superposition (valid when gratings are spectrally
/// well-separated).
///
/// # Arguments
///
/// * `gratings` - Slice of `(center_nm, fwhm_pm, reflectivity)` tuples.
/// * `wavelengths` - Wavelength axis in nm.
///
/// # Returns
///
/// Combined linear power spectrum.
pub fn multiplex_fbg_spectrum(
    gratings: &[(f64, f64, f64)],
    wavelengths: &[f64],
) -> Vec<f64> {
    let mut combined = vec![0.0_f64; wavelengths.len()];
    for &(center, fwhm, refl) in gratings {
        let spec = generate_fbg_spectrum(center, fwhm, refl, wavelengths);
        for (c, s) in combined.iter_mut().zip(spec.iter()) {
            *c += s;
        }
    }
    combined
}

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

/// Safe natural logarithm that clamps very small / negative values.
fn safe_ln(x: f64) -> f64 {
    if x > 1e-30 {
        x.ln()
    } else {
        (-30.0_f64) * std::f64::consts::LN_10 // ≈ -69
    }
}

/// Convert dBm to linear power (milliwatts).
fn dbm_to_linear(dbm: f64) -> f64 {
    10.0_f64.powf(dbm / 10.0)
}

/// Convert linear power (milliwatts) to dBm.
fn linear_to_dbm(linear: f64) -> f64 {
    if linear > 1e-30 {
        10.0 * linear.log10()
    } else {
        -300.0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // -- Bragg wavelength --

    #[test]
    fn test_bragg_wavelength_standard_fiber() {
        // n_eff=1.465, Λ=529.0 nm → λ_B ≈ 1549.97 nm
        let lambda = bragg_wavelength(1.465, 529.0);
        assert!((lambda - 1549.97).abs() < 0.01, "λ_B = {lambda}");
    }

    #[test]
    fn test_bragg_wavelength_1550() {
        // Reverse-engineer period for 1550 nm.
        let period = 1550.0 / (2.0 * 1.465);
        let lambda = bragg_wavelength(1.465, period);
        assert!((lambda - 1550.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_bragg_wavelength_zero_index() {
        assert_eq!(bragg_wavelength(0.0, 530.0), 0.0);
    }

    #[test]
    fn test_bragg_wavelength_zero_period() {
        assert_eq!(bragg_wavelength(1.465, 0.0), 0.0);
    }

    // -- Strain / temperature conversion --

    #[test]
    fn test_wavelength_to_strain_positive() {
        // 1.2 pm shift at 1.2 pm/με → 1 με
        let strain = wavelength_to_strain(1.2, 1.2);
        assert!((strain - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_wavelength_to_strain_negative() {
        // Negative shift → compressive strain.
        let strain = wavelength_to_strain(-2.4, 1.2);
        assert!((strain - (-2.0)).abs() < TOLERANCE);
    }

    #[test]
    fn test_wavelength_to_strain_zero_sensitivity() {
        assert_eq!(wavelength_to_strain(5.0, 0.0), 0.0);
    }

    #[test]
    fn test_wavelength_to_strain_large() {
        // 1200 pm → 1000 με
        let strain = wavelength_to_strain(1200.0, 1.2);
        assert!((strain - 1000.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_wavelength_to_temperature_positive() {
        // 10 pm shift at 10 pm/°C → 1 °C
        let temp = wavelength_to_temperature(10.0, 10.0);
        assert!((temp - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_wavelength_to_temperature_negative() {
        let temp = wavelength_to_temperature(-50.0, 10.0);
        assert!((temp - (-5.0)).abs() < TOLERANCE);
    }

    #[test]
    fn test_wavelength_to_temperature_zero_sensitivity() {
        assert_eq!(wavelength_to_temperature(5.0, 0.0), 0.0);
    }

    // -- Strain-optic coefficient --

    #[test]
    fn test_strain_optic_coefficient_silica() {
        // Standard silica: p11=0.121, p12=0.270, ν=0.17 → p_e ≈ 0.216
        let pe = strain_optic_coefficient(0.121, 0.270, 0.17);
        assert!(
            (pe - 0.216).abs() < 0.01,
            "p_e = {pe}, expected ~0.216"
        );
    }

    #[test]
    fn test_strain_optic_coefficient_zero_poisson() {
        // ν=0 → p_e = n²/2 * p12
        let pe = strain_optic_coefficient(0.121, 0.270, 0.0);
        let expected = 1.465_f64.powi(2) / 2.0 * 0.270;
        assert!((pe - expected).abs() < TOLERANCE);
    }

    #[test]
    fn test_strain_optic_coefficient_with_explicit_n() {
        let pe = strain_optic_coefficient_with_n(0.121, 0.270, 0.17, 1.5);
        let expected = (1.5_f64.powi(2) / 2.0) * (0.270 - 0.17 * (0.121 + 0.270));
        assert!((pe - expected).abs() < TOLERANCE);
    }

    // -- Spectrum generation --

    #[test]
    fn test_generate_fbg_spectrum_peak_at_center() {
        let wl: Vec<f64> = (0..1000).map(|i| 1549.0 + i as f64 * 0.002).collect();
        let spec = generate_fbg_spectrum(1550.0, 200.0, 0.95, &wl);
        // Find the maximum.
        let max_idx = spec
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_wl = wl[max_idx];
        assert!(
            (peak_wl - 1550.0).abs() < 0.01,
            "Peak at {peak_wl}, expected 1550.0"
        );
        assert!((spec[max_idx] - 0.95).abs() < 0.01);
    }

    #[test]
    fn test_generate_fbg_spectrum_fwhm() {
        let wl: Vec<f64> = (0..10000).map(|i| 1549.5 + i as f64 * 0.0001).collect();
        let spec = generate_fbg_spectrum(1550.0, 200.0, 1.0, &wl);
        let half = 0.5;
        // Find crossings of the half-maximum level.
        let mut left_cross = 0;
        let mut right_cross = wl.len() - 1;
        for i in 0..spec.len() - 1 {
            if spec[i] < half && spec[i + 1] >= half {
                left_cross = i;
            }
            if spec[i] >= half && spec[i + 1] < half {
                right_cross = i;
            }
        }
        let measured_fwhm_nm = wl[right_cross] - wl[left_cross];
        let measured_fwhm_pm = measured_fwhm_nm * 1000.0;
        assert!(
            (measured_fwhm_pm - 200.0).abs() < 5.0,
            "FWHM = {measured_fwhm_pm} pm, expected ~200 pm"
        );
    }

    #[test]
    fn test_generate_fbg_spectrum_zero_reflectivity() {
        let wl: Vec<f64> = (0..100).map(|i| 1549.0 + i as f64 * 0.02).collect();
        let spec = generate_fbg_spectrum(1550.0, 200.0, 0.0, &wl);
        assert!(spec.iter().all(|&v| v.abs() < TOLERANCE));
    }

    #[test]
    fn test_generate_fbg_spectrum_empty_wavelength() {
        let spec = generate_fbg_spectrum(1550.0, 200.0, 0.9, &[]);
        assert!(spec.is_empty());
    }

    // -- Multiplexed spectrum --

    #[test]
    fn test_multiplex_two_gratings() {
        let wl: Vec<f64> = (0..10000).map(|i| 1540.0 + i as f64 * 0.002).collect();
        let gratings = vec![(1545.0, 200.0, 0.9), (1555.0, 200.0, 0.85)];
        let combined = multiplex_fbg_spectrum(&gratings, &wl);
        // Find two peaks by scanning for local maxima.
        let mut peaks = Vec::new();
        for i in 1..combined.len() - 1 {
            if combined[i] > combined[i - 1]
                && combined[i] > combined[i + 1]
                && combined[i] > 0.5
            {
                peaks.push(wl[i]);
            }
        }
        assert!(peaks.len() >= 2, "Expected at least 2 peaks, found {}", peaks.len());
        // Verify the first peak is near 1545 and the second near 1555.
        assert!((peaks[0] - 1545.0).abs() < 0.1);
        assert!((peaks[peaks.len() - 1] - 1555.0).abs() < 0.1);
    }

    #[test]
    fn test_multiplex_single_grating() {
        let wl: Vec<f64> = (0..1000).map(|i| 1549.0 + i as f64 * 0.002).collect();
        let gratings = vec![(1550.0, 200.0, 0.9)];
        let combined = multiplex_fbg_spectrum(&gratings, &wl);
        let direct = generate_fbg_spectrum(1550.0, 200.0, 0.9, &wl);
        for (c, d) in combined.iter().zip(direct.iter()) {
            assert!((c - d).abs() < TOLERANCE);
        }
    }

    #[test]
    fn test_multiplex_no_gratings() {
        let wl: Vec<f64> = (0..100).map(|i| 1549.0 + i as f64 * 0.02).collect();
        let combined = multiplex_fbg_spectrum(&[], &wl);
        assert!(combined.iter().all(|&v| v.abs() < TOLERANCE));
    }

    // -- Peak finding --

    #[test]
    fn test_find_peaks_single() {
        let wl: Vec<f64> = (0..10000).map(|i| 1549.0 + i as f64 * 0.0002).collect();
        let spec = generate_fbg_spectrum(1550.0, 200.0, 0.9, &wl);
        let power_dbm: Vec<f64> = spec.iter().map(|&p| linear_to_dbm(p)).collect();
        let peaks = find_peaks_in_spectrum(&wl, &power_dbm, 30.0);
        assert!(!peaks.is_empty(), "Should detect at least one peak");
        assert!(
            (peaks[0].center_wavelength_nm - 1550.0).abs() < 0.01,
            "Peak center = {}",
            peaks[0].center_wavelength_nm
        );
    }

    #[test]
    fn test_find_peaks_two_separated() {
        let wl: Vec<f64> = (0..20000).map(|i| 1540.0 + i as f64 * 0.001).collect();
        let s1 = generate_fbg_spectrum(1545.0, 200.0, 0.9, &wl);
        let s2 = generate_fbg_spectrum(1555.0, 200.0, 0.85, &wl);
        let combined: Vec<f64> = s1.iter().zip(&s2).map(|(a, b)| a + b).collect();
        let power_dbm: Vec<f64> = combined.iter().map(|&p| linear_to_dbm(p)).collect();
        let peaks = find_peaks_in_spectrum(&wl, &power_dbm, 30.0);
        assert!(peaks.len() >= 2, "Found {} peaks, expected >= 2", peaks.len());
    }

    #[test]
    fn test_find_peaks_below_threshold() {
        let wl: Vec<f64> = (0..1000).map(|i| 1549.0 + i as f64 * 0.002).collect();
        let spec = generate_fbg_spectrum(1550.0, 200.0, 0.0001, &wl);
        // Add a strong peak.
        let strong = generate_fbg_spectrum(1549.5, 200.0, 1.0, &wl);
        let combined: Vec<f64> = spec.iter().zip(&strong).map(|(a, b)| a + b).collect();
        let power_dbm: Vec<f64> = combined.iter().map(|&p| linear_to_dbm(p)).collect();
        // With a tight 3 dB threshold, only the strong peak should pass.
        let peaks = find_peaks_in_spectrum(&wl, &power_dbm, 3.0);
        // All detected peaks should be near 1549.5.
        for p in &peaks {
            assert!(
                (p.center_wavelength_nm - 1549.5).abs() < 1.0,
                "Unexpected peak at {}",
                p.center_wavelength_nm
            );
        }
    }

    #[test]
    fn test_find_peaks_empty_spectrum() {
        let peaks = find_peaks_in_spectrum(&[], &[], 20.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_find_peaks_short_spectrum() {
        let peaks = find_peaks_in_spectrum(&[1550.0, 1550.1], &[-10.0, -20.0], 20.0);
        // Only 2 points; the first is a local max.
        // With our implementation, 2-point spectra are too short for interior peaks.
        // Endpoint check might detect one.
        assert!(peaks.len() <= 1);
    }

    // -- Gaussian peak fit --

    #[test]
    fn test_gaussian_fit_symmetric() {
        // Three-point symmetric Gaussian: peak at index 1.
        let wl: [f64; 3] = [1549.9, 1550.0, 1550.1];
        let sigma: f64 = 0.1; // nm
        let pl: Vec<f64> = wl
            .iter()
            .map(|&w| (-0.5f64 * ((w - 1550.0) / sigma).powi(2)).exp())
            .collect();
        let (center, amp, _) = gaussian_peak_fit(&wl, &pl, 1);
        assert!(
            (center - 1550.0).abs() < 0.001,
            "Fit center = {center}"
        );
        assert!((amp - 1.0).abs() < 0.01, "Fit amplitude = {amp}");
    }

    #[test]
    fn test_gaussian_fit_offset_peak() {
        // Peak slightly off-center between indices 1 and 2.
        let true_center: f64 = 1550.03;
        let sigma: f64 = 0.1;
        let wl: [f64; 3] = [1549.9, 1550.0, 1550.1];
        let pl: Vec<f64> = wl
            .iter()
            .map(|&w| (-0.5f64 * ((w - true_center) / sigma).powi(2)).exp())
            .collect();
        let peak_idx = pl
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let (center, _, _) = gaussian_peak_fit(&wl, &pl, peak_idx);
        assert!(
            (center - true_center).abs() < 0.02,
            "Fit center = {center}, true = {true_center}"
        );
    }

    #[test]
    fn test_gaussian_fit_at_boundary() {
        let wl = [1549.9, 1550.0, 1550.1];
        let pl = [1.0, 0.5, 0.2];
        let (center, amp, _) = gaussian_peak_fit(&wl, &pl, 0);
        // At boundary, just returns raw values.
        assert!((center - 1549.9).abs() < TOLERANCE);
        assert!((amp - 1.0).abs() < TOLERANCE);
    }

    // -- FbgInterrogator end-to-end --

    #[test]
    fn test_interrogator_single_grating_no_shift() {
        let config = FbgConfig {
            num_gratings: 1,
            nominal_wavelengths_nm: vec![1550.0],
            sample_rate_hz: 1000.0,
            strain_sensitivity_pm_per_ue: 1.2,
            temp_sensitivity_pm_per_c: 10.0,
        };
        let mut interr = FbgInterrogator::new(config);

        let wl: Vec<f64> = (0..10000).map(|i| 1549.0 + i as f64 * 0.0002).collect();
        let spec = generate_fbg_spectrum(1550.0, 200.0, 0.9, &wl);
        let power_dbm: Vec<f64> = spec.iter().map(|&p| linear_to_dbm(p)).collect();

        let readings = interr.process_spectrum(&wl, &power_dbm);
        assert_eq!(readings.len(), 1);
        assert_eq!(readings[0].grating_index, 0);
        assert!(
            readings[0].wavelength_shift_pm.abs() < 5.0,
            "shift = {} pm, expected ~0",
            readings[0].wavelength_shift_pm
        );
        assert!(readings[0].estimated_strain_ue.abs() < 5.0);
    }

    #[test]
    fn test_interrogator_single_grating_with_strain() {
        let config = FbgConfig {
            num_gratings: 1,
            nominal_wavelengths_nm: vec![1550.0],
            sample_rate_hz: 1000.0,
            strain_sensitivity_pm_per_ue: 1.2,
            temp_sensitivity_pm_per_c: 10.0,
        };
        let mut interr = FbgInterrogator::new(config);

        // Apply 100 με → shift = 120 pm = 0.12 nm.
        let shifted_center = 1550.0 + 0.12;
        let wl: Vec<f64> = (0..10000).map(|i| 1549.0 + i as f64 * 0.0002).collect();
        let spec = generate_fbg_spectrum(shifted_center, 200.0, 0.9, &wl);
        let power_dbm: Vec<f64> = spec.iter().map(|&p| linear_to_dbm(p)).collect();

        let readings = interr.process_spectrum(&wl, &power_dbm);
        assert_eq!(readings.len(), 1);
        assert!(
            (readings[0].wavelength_shift_pm - 120.0).abs() < 10.0,
            "shift = {} pm, expected ~120",
            readings[0].wavelength_shift_pm
        );
        assert!(
            (readings[0].estimated_strain_ue - 100.0).abs() < 10.0,
            "strain = {} με, expected ~100",
            readings[0].estimated_strain_ue
        );
    }

    #[test]
    fn test_interrogator_two_gratings() {
        let config = FbgConfig {
            num_gratings: 2,
            nominal_wavelengths_nm: vec![1545.0, 1555.0],
            sample_rate_hz: 1000.0,
            strain_sensitivity_pm_per_ue: 1.2,
            temp_sensitivity_pm_per_c: 10.0,
        };
        let mut interr = FbgInterrogator::new(config);

        let wl: Vec<f64> = (0..20000).map(|i| 1540.0 + i as f64 * 0.001).collect();
        let gratings = vec![(1545.05, 200.0, 0.9), (1555.10, 200.0, 0.85)];
        let combined = multiplex_fbg_spectrum(&gratings, &wl);
        let power_dbm: Vec<f64> = combined.iter().map(|&p| linear_to_dbm(p)).collect();

        let readings = interr.process_spectrum(&wl, &power_dbm);
        assert_eq!(readings.len(), 2);
        assert_eq!(readings[0].grating_index, 0);
        assert_eq!(readings[1].grating_index, 1);
        // Grating 0: shift ≈ 50 pm.
        assert!(
            (readings[0].wavelength_shift_pm - 50.0).abs() < 10.0,
            "g0 shift = {} pm",
            readings[0].wavelength_shift_pm
        );
        // Grating 1: shift ≈ 100 pm.
        assert!(
            (readings[1].wavelength_shift_pm - 100.0).abs() < 10.0,
            "g1 shift = {} pm",
            readings[1].wavelength_shift_pm
        );
    }

    #[test]
    fn test_interrogator_empty_spectrum() {
        let config = FbgConfig::default();
        let mut interr = FbgInterrogator::new(config);
        let readings = interr.process_spectrum(&[], &[]);
        assert!(readings.is_empty());
    }

    #[test]
    fn test_interrogator_temperature_shift() {
        let config = FbgConfig {
            num_gratings: 1,
            nominal_wavelengths_nm: vec![1550.0],
            sample_rate_hz: 1000.0,
            strain_sensitivity_pm_per_ue: 1.2,
            temp_sensitivity_pm_per_c: 10.0,
        };
        let mut interr = FbgInterrogator::new(config);

        // +50°C → shift = 500 pm = 0.5 nm
        let shifted = 1550.0 + 0.5;
        let wl: Vec<f64> = (0..10000).map(|i| 1549.0 + i as f64 * 0.0004).collect();
        let spec = generate_fbg_spectrum(shifted, 200.0, 0.9, &wl);
        let power_dbm: Vec<f64> = spec.iter().map(|&p| linear_to_dbm(p)).collect();

        let readings = interr.process_spectrum(&wl, &power_dbm);
        assert_eq!(readings.len(), 1);
        assert!(
            (readings[0].estimated_temp_delta_c - 50.0).abs() < 5.0,
            "temp = {} °C, expected ~50",
            readings[0].estimated_temp_delta_c
        );
    }

    // -- dBm conversion --

    #[test]
    fn test_dbm_linear_roundtrip() {
        let original = 0.5; // 0.5 mW
        let dbm = linear_to_dbm(original);
        let back = dbm_to_linear(dbm);
        assert!((back - original).abs() < 1e-10);
    }

    #[test]
    fn test_dbm_to_linear_0dbm() {
        let lin = dbm_to_linear(0.0);
        assert!((lin - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_linear_to_dbm_zero() {
        let dbm = linear_to_dbm(0.0);
        assert!(dbm < -100.0); // Very negative.
    }

    // -- Default config --

    #[test]
    fn test_default_config() {
        let cfg = FbgConfig::default();
        assert_eq!(cfg.num_gratings, 1);
        assert!((cfg.strain_sensitivity_pm_per_ue - 1.2).abs() < TOLERANCE);
        assert!((cfg.temp_sensitivity_pm_per_c - 10.0).abs() < TOLERANCE);
        assert!((cfg.sample_rate_hz - 1000.0).abs() < TOLERANCE);
    }

    // -- FWHM measurement accuracy --

    #[test]
    fn test_peak_fwhm_measurement() {
        let wl: Vec<f64> = (0..50000).map(|i| 1549.5 + i as f64 * 0.00002).collect();
        let spec = generate_fbg_spectrum(1550.0, 100.0, 1.0, &wl);
        let power_dbm: Vec<f64> = spec.iter().map(|&p| linear_to_dbm(p)).collect();
        let peaks = find_peaks_in_spectrum(&wl, &power_dbm, 30.0);
        assert!(!peaks.is_empty());
        // The detected FWHM should be close to 100 pm.
        assert!(
            (peaks[0].fwhm_pm - 100.0).abs() < 20.0,
            "Measured FWHM = {} pm, expected ~100 pm",
            peaks[0].fwhm_pm
        );
    }

    // -- Edge cases / robustness --

    #[test]
    fn test_set_threshold() {
        let config = FbgConfig::default();
        let mut interr = FbgInterrogator::new(config);
        interr.set_threshold_db(10.0);
        assert!((interr.threshold_db - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_safe_ln_negative_input() {
        let val = safe_ln(-1.0);
        assert!(val.is_finite());
        assert!(val < -60.0); // Should be very negative.
    }
}

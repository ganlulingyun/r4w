//! Laser Flash Analysis (LFA) for thermal property measurement of materials.
//!
//! This module implements the complete laser flash analysis (LFA) signal
//! processing chain, from raw thermogram data to thermal diffusivity, thermal
//! conductivity, and specific heat capacity. LFA (ASTM E1461, ISO 18755) is the
//! most widely used technique for measuring thermal diffusivity of solids over a
//! broad temperature range.
//!
//! # Background
//!
//! In a laser flash experiment, a short laser pulse (typically 0.3–2 ms) heats
//! the front face of a disc-shaped sample uniformly and instantaneously. An
//! infrared detector monitors the resulting temperature rise on the rear face.
//! From the shape of this thermogram, especially the time for the rear-face
//! temperature to reach half its maximum value (t½), the thermal diffusivity
//! can be calculated.
//!
//! # Processing Pipeline
//!
//! 1. **Pre-processing** - Baseline correction, noise filtering, signal normalization.
//! 2. **Half-time extraction** - Interpolate the normalized thermogram to find t½.
//! 3. **Parker equation** - Compute raw thermal diffusivity from t½.
//! 4. **Cowan correction** - Correct for radiative heat losses using ratio method.
//! 5. **Cape-Lehman correction** - Correct for finite laser pulse duration.
//! 6. **Thermal conductivity** - Combine with density and Cp to get k.
//! 7. **Specific heat** - Comparison method against reference material.
//! 8. **Temperature-dependence** - Compile alpha(T) and k(T) curves.
//!
//! # Key Equations
//!
//! **Parker equation (adiabatic, instantaneous pulse):**
//! ```text
//!   alpha = 0.1388 * L^2 / t_half
//! ```
//! where L = sample thickness (m), t_half = time to 50% of T_max (s).
//!
//! **Rear-face temperature response (adiabatic):**
//! ```text
//!   T(L,t)/T_max = 1 + 2*sum_{n=1}^{inf} (-1)^n * exp(-n^2 * pi^2 * alpha * t / L^2)
//! ```
//!
//! **Cowan ratio correction:**
//! ```text
//!   Kp = f(t_0.75 / t_0.25)   or   Kp = f(t_0.80 / t_0.20)
//! ```
//!
//! **Thermal conductivity:**
//! ```text
//!   k = alpha * rho * Cp
//! ```
//!
//! **Specific heat by comparison:**
//! ```text
//!   Cp_sample = Cp_ref * (DeltaT_ref * m_ref) / (DeltaT_sample * m_sample)
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::laser_flash_thermal_analyzer::{
//!     LaserFlashAnalyzer, SampleParams, PulseParams, Thermogram,
//! };
//!
//! // Sample: 3 mm thick copper disc
//! let params = SampleParams {
//!     thickness_mm: 3.0,
//!     density_kg_m3: 8960.0,
//!     specific_heat_j_per_kgk: 385.0,
//!     mass_mg: 500.0,
//! };
//!
//! let pulse = PulseParams {
//!     duration_us: 400.0,
//!     energy_j: 0.05,
//! };
//!
//! let analyzer = LaserFlashAnalyzer::new(params, pulse);
//!
//! // Generate a synthetic thermogram (adiabatic model)
//! let n_points = 512;
//! let dt = 0.0005_f64;
//! let alpha_true = 1.17e-4_f64;
//! let thickness = 3e-3_f64;
//! let mut time_s: Vec<f64> = (0..n_points).map(|i| i as f64 * dt).collect();
//! let temperature: Vec<f64> = time_s.iter().map(|&t| {
//!     if t <= 0.0 { return 0.0; }
//!     let norm_t = alpha_true * t / (thickness * thickness);
//!     let mut val = 1.0_f64;
//!     for n in 1..=20 {
//!         let sign = if n % 2 == 0 { 1.0 } else { -1.0 };
//!         val += 2.0 * sign * (-((n * n) as f64) * std::f64::consts::PI.powi(2) * norm_t).exp();
//!     }
//!     val.max(0.0) * 10.0 // 10 degrees rise
//! }).collect();
//!
//! let thermogram = Thermogram {
//!     time_s,
//!     temperature,
//!     trigger_index: 10,
//! };
//!
//! let result = analyzer.analyze(&thermogram);
//! // Thermal diffusivity should be close to 1.17e-4 m²/s
//! assert!(result.alpha_m2_per_s > 0.5e-4 && result.alpha_m2_per_s < 2.0e-4);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Raw rear-face temperature vs. time data from a single laser flash shot.
#[derive(Debug, Clone)]
pub struct Thermogram {
    /// Time axis (seconds), evenly or unevenly spaced.
    pub time_s: Vec<f64>,
    /// Rear-face temperature (°C or K; units are relative for diffusivity).
    pub temperature: Vec<f64>,
    /// Index in the arrays corresponding to the laser trigger event.
    /// Data before this index is used for baseline estimation.
    pub trigger_index: usize,
}

/// Physical parameters of the sample disc.
#[derive(Debug, Clone, Copy)]
pub struct SampleParams {
    /// Sample thickness in millimetres.
    pub thickness_mm: f64,
    /// Bulk density in kg/m³.
    pub density_kg_m3: f64,
    /// Specific heat capacity in J/(kg·K). Set to 0 to skip k calculation.
    pub specific_heat_j_per_kgk: f64,
    /// Sample mass in milligrams (used for comparative Cp measurement).
    pub mass_mg: f64,
}

/// Laser pulse parameters.
#[derive(Debug, Clone, Copy)]
pub struct PulseParams {
    /// Laser pulse duration (FWHM or full width) in microseconds.
    pub duration_us: f64,
    /// Laser pulse energy in joules.
    pub energy_j: f64,
}

/// Cowan heat-loss correction factors derived from a thermogram.
#[derive(Debug, Clone, Copy)]
pub struct CowanCorrection {
    /// Ratio t_0.75 / t_0.25 (time to 75% T_max over time to 25% T_max).
    pub ratio_75_25: f64,
    /// Ratio t_0.80 / t_0.20.
    pub ratio_80_20: f64,
    /// Dimensionless correction factor Kp applied to alpha_parker.
    pub correction_factor: f64,
}

/// Full result of a single laser flash shot analysis.
#[derive(Debug, Clone, Copy)]
pub struct FlashResult {
    /// Thermal diffusivity from Parker equation (m²/s).
    pub alpha_parker_m2_per_s: f64,
    /// Thermal diffusivity after Cowan heat-loss correction (m²/s).
    pub alpha_cowan_m2_per_s: f64,
    /// Thermal diffusivity after Cape-Lehman finite-pulse correction (m²/s).
    pub alpha_corrected_m2_per_s: f64,
    /// Final recommended thermal diffusivity (m²/s).
    pub alpha_m2_per_s: f64,
    /// Time to 50% of T_max (seconds) after trigger.
    pub t_half_s: f64,
    /// Thermal conductivity k = alpha * rho * Cp (W/(m·K)). NaN if Cp not provided.
    pub k_w_per_mk: f64,
    /// Specific heat capacity as provided (J/(kg·K)).
    pub cp_j_per_kgk: f64,
    /// Cowan correction details.
    pub cowan: CowanCorrection,
    /// Maximum temperature rise (normalised units).
    pub delta_t_max: f64,
    /// Cape-Lehman dimensionless pulse parameter tau = t_pulse / t_half.
    pub tau_cape_lehman: f64,
}

/// Temperature-dependent thermal property curve from multiple shots.
#[derive(Debug, Clone)]
pub struct ThermalCurve {
    /// Measurement temperatures (°C).
    pub temperatures_c: Vec<f64>,
    /// Thermal diffusivity at each temperature (m²/s).
    pub diffusivity: Vec<f64>,
    /// Thermal conductivity at each temperature (W/(m·K)).
    pub conductivity: Vec<f64>,
    /// Specific heat at each temperature (J/(kg·K)).
    pub specific_heat: Vec<f64>,
}

/// Material thermal properties database entry.
#[derive(Debug, Clone)]
pub struct MaterialThermalProps {
    /// Material name.
    pub name: &'static str,
    /// Reference thermal diffusivity at 25°C (m²/s).
    pub alpha_ref: f64,
    /// Thermal conductivity at 25°C (W/(m·K)).
    pub k_ref: f64,
    /// Specific heat at 25°C (J/(kg·K)).
    pub cp_ref: f64,
    /// Density (kg/m³).
    pub density: f64,
    /// Reference temperature (°C).
    pub temp_c: f64,
}

/// Configuration options for the LFA analyzer.
#[derive(Debug, Clone, Copy)]
pub struct LfaConfig {
    /// Number of Fourier-series terms for rear-face model (default 20).
    pub n_series_terms: usize,
    /// Enable Cowan heat-loss correction (default true).
    pub apply_cowan: bool,
    /// Enable Cape-Lehman finite-pulse correction (default true).
    pub apply_cape_lehman: bool,
    /// Moving-average window for noise filtering (0 = no filter).
    pub smoothing_window: usize,
    /// Minimum fraction of T_max to consider as valid data (default 0.001).
    pub noise_floor_fraction: f64,
}

impl Default for LfaConfig {
    fn default() -> Self {
        Self {
            n_series_terms: 20,
            apply_cowan: true,
            apply_cape_lehman: true,
            smoothing_window: 3,
            noise_floor_fraction: 0.001,
        }
    }
}

// ---------------------------------------------------------------------------
// Material database
// ---------------------------------------------------------------------------

/// Return a static slice of built-in material thermal property presets at 25°C.
pub fn material_presets() -> &'static [MaterialThermalProps] {
    &MATERIAL_DB
}

static MATERIAL_DB: [MaterialThermalProps; 6] = [
    MaterialThermalProps {
        name: "Copper",
        alpha_ref: 1.17e-4,
        k_ref: 401.0,
        cp_ref: 385.0,
        density: 8960.0,
        temp_c: 25.0,
    },
    MaterialThermalProps {
        name: "Aluminum",
        alpha_ref: 9.7e-5,
        k_ref: 237.0,
        cp_ref: 897.0,
        density: 2700.0,
        temp_c: 25.0,
    },
    MaterialThermalProps {
        name: "Iron",
        alpha_ref: 2.3e-5,
        k_ref: 80.0,
        cp_ref: 449.0,
        density: 7874.0,
        temp_c: 25.0,
    },
    MaterialThermalProps {
        name: "Graphite",
        alpha_ref: 1.22e-4,
        k_ref: 168.0,
        cp_ref: 710.0,
        density: 1940.0,
        temp_c: 25.0,
    },
    MaterialThermalProps {
        name: "Fused Silica",
        alpha_ref: 8.3e-7,
        k_ref: 1.38,
        cp_ref: 740.0,
        density: 2200.0,
        temp_c: 25.0,
    },
    MaterialThermalProps {
        name: "Alumina (Al2O3)",
        alpha_ref: 1.2e-5,
        k_ref: 35.0,
        cp_ref: 765.0,
        density: 3900.0,
        temp_c: 25.0,
    },
];

// ---------------------------------------------------------------------------
// Core mathematical functions
// ---------------------------------------------------------------------------

/// Evaluate the adiabatic rear-face temperature response.
///
/// Returns T(L,t)/T_max using the series:
/// ```text
///   T(L,t)/T_max = 1 + 2*sum_{n=1}^{N} (-1)^n * exp(-n^2 * pi^2 * Fo)
/// ```
/// where `Fo = alpha * t / L^2` is the Fourier number.
///
/// # Arguments
/// * `fourier` - Dimensionless Fourier number Fo = alpha * t / L².
/// * `n_terms` - Number of series terms (20 is typically sufficient).
pub fn adiabatic_rear_face(fourier: f64, n_terms: usize) -> f64 {
    if fourier <= 0.0 {
        return 0.0;
    }
    let mut sum = 1.0_f64;
    for n in 1..=n_terms {
        let sign = if n % 2 == 0 { 1.0_f64 } else { -1.0_f64 };
        let exponent = -(n as f64).powi(2) * PI * PI * fourier;
        sum += 2.0 * sign * exponent.exp();
    }
    sum.max(0.0)
}

/// Parker (1961) thermal diffusivity from rear-face t-half.
///
/// ```text
///   alpha = 0.1388 * L^2 / t_half
/// ```
///
/// # Arguments
/// * `thickness_m` - Sample thickness in metres.
/// * `t_half_s` - Time from laser trigger to 50% of T_max (seconds).
pub fn parker_diffusivity(thickness_m: f64, t_half_s: f64) -> f64 {
    if t_half_s <= 0.0 || thickness_m <= 0.0 {
        return f64::NAN;
    }
    0.1388_f64 * thickness_m * thickness_m / t_half_s
}

/// Derive the adiabatic t-half from Parker equation (inverse).
///
/// ```text
///   t_half = 0.1388 * L^2 / alpha
/// ```
pub fn parker_t_half(thickness_m: f64, alpha: f64) -> f64 {
    if alpha <= 0.0 || thickness_m <= 0.0 {
        return f64::NAN;
    }
    0.1388_f64 * thickness_m * thickness_m / alpha
}

/// Cowan correction factor lookup from ratio t_0.75 / t_0.25.
///
/// The correction factor Kp is derived from the Cowan (1963) polynomial fit
/// to tabulated values. The polynomial approximation used here is valid for
/// ratios in the range [1.5, 10].
///
/// Returns the dimensionless factor Kp such that alpha_corrected = alpha_parker * Kp.
pub fn cowan_correction_from_ratio(ratio_75_25: f64) -> f64 {
    // Cowan (1963) tabulated values and polynomial fit.
    // Kp(u) where u = t_0.75 / t_0.25
    // For an ideal adiabatic case u = 2.684 and Kp = 1.0.
    // Larger ratios indicate heat loss, requiring upward correction.
    let u = ratio_75_25;
    if u <= 0.0 {
        return 1.0;
    }
    // Piecewise polynomial fit based on Cowan (1963) Table 1.
    // Valid range approximately 1.5 <= u <= 12.
    if u < 2.0 {
        // Near-adiabatic region — slight correction
        1.0 + 0.0081 * (u - 2.684)
    } else if u <= 4.0 {
        // Main working range
        let du = u - 2.684;
        1.0 + 0.0281 * du + 0.00124 * du * du
    } else if u <= 8.0 {
        let du = u - 2.684;
        1.0 + 0.0281 * du + 0.00124 * du * du - 1.5e-5 * du * du * du
    } else {
        // Heavy heat-loss regime — linear extrapolation
        1.0 + 0.0281 * (u - 2.684)
    }
}

/// Cowan correction factor lookup from ratio t_0.80 / t_0.20.
///
/// For the adiabatic case, t_0.80/t_0.20 ≈ 3.812.
pub fn cowan_correction_from_ratio_8020(ratio_80_20: f64) -> f64 {
    // Map to equivalent 75/25 ratio and apply standard lookup.
    // Approximate linear relationship based on numerical tables.
    // For adiabatic: ratio_75_25 ≈ 2.684, ratio_80_20 ≈ 3.812 => scale ~1.421
    let equivalent_75_25 = ratio_80_20 / 1.421;
    cowan_correction_from_ratio(equivalent_75_25)
}

/// Cape-Lehman finite-pulse correction to thermal diffusivity.
///
/// When the pulse duration is not truly instantaneous, the apparent t½ is
/// shifted by approximately t_pulse/2. The correction factor is:
/// ```text
///   tau = t_pulse / t_half
///   Kp_CL = 1 + A1*tau + A2*tau^2 + A3*tau^3
/// ```
/// Coefficients from Cape & Lehman (1963), valid for tau < 0.3.
///
/// # Arguments
/// * `t_half_s` - Apparent (uncorrected) t½ in seconds.
/// * `pulse_duration_s` - Laser pulse duration in seconds.
///
/// Returns the corrected diffusivity scaling factor (multiply alpha by this).
pub fn cape_lehman_correction(t_half_s: f64, pulse_duration_s: f64) -> f64 {
    if t_half_s <= 0.0 || pulse_duration_s <= 0.0 {
        return 1.0;
    }
    let tau = pulse_duration_s / t_half_s;
    if tau < 1e-4 {
        return 1.0; // negligible correction
    }
    // Cape-Lehman polynomial coefficients
    // alpha_true / alpha_apparent = 1 / (1 - A1*tau - A2*tau^2 ...)
    // Equivalently: corrected = apparent * Kp where Kp > 1
    let a1 = 0.3333_f64;
    let a2 = -0.0833_f64;
    let a3 = 0.0222_f64;
    let denom = 1.0 - a1 * tau - a2 * tau * tau - a3 * tau * tau * tau;
    if denom <= 0.0 {
        return 1.0;
    }
    1.0 / denom
}

/// Thermal conductivity from diffusivity, density, and specific heat.
///
/// ```text
///   k = alpha * rho * Cp
/// ```
pub fn thermal_conductivity(alpha_m2_per_s: f64, density_kg_m3: f64, cp_j_per_kgk: f64) -> f64 {
    alpha_m2_per_s * density_kg_m3 * cp_j_per_kgk
}

/// Specific heat by comparative LFA method.
///
/// ```text
///   Cp_sample = Cp_ref * (DeltaT_ref * m_ref) / (DeltaT_sample * m_sample)
/// ```
///
/// # Arguments
/// * `cp_ref` - Known Cp of the reference sample (J/(kg·K)).
/// * `delta_t_ref` - Maximum temperature rise of reference (normalised or K).
/// * `mass_ref_mg` - Mass of reference sample (mg).
/// * `delta_t_sample` - Maximum temperature rise of sample.
/// * `mass_sample_mg` - Mass of sample (mg).
pub fn specific_heat_comparative(
    cp_ref: f64,
    delta_t_ref: f64,
    mass_ref_mg: f64,
    delta_t_sample: f64,
    mass_sample_mg: f64,
) -> f64 {
    if delta_t_sample <= 0.0 || mass_sample_mg <= 0.0 {
        return f64::NAN;
    }
    cp_ref * (delta_t_ref * mass_ref_mg) / (delta_t_sample * mass_sample_mg)
}

// ---------------------------------------------------------------------------
// Signal processing helpers
// ---------------------------------------------------------------------------

/// Apply a simple moving-average filter (boxcar) of given half-width.
///
/// Edge samples are replicated to preserve array length.
pub fn moving_average_filter(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if window == 0 || n == 0 {
        return data.to_vec();
    }
    let hw = window / 2;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let start = if i >= hw { i - hw } else { 0 };
        let end = (i + hw + 1).min(n);
        let sum: f64 = data[start..end].iter().sum();
        out.push(sum / (end - start) as f64);
    }
    out
}

/// Fit a linear baseline to `data[0..end_idx]` and return baseline-corrected
/// data where the pre-trigger trend is subtracted from the full array.
///
/// Returns `(corrected, slope, intercept)`.
pub fn baseline_correction(
    time_s: &[f64],
    temperature: &[f64],
    end_idx: usize,
) -> (Vec<f64>, f64, f64) {
    assert_eq!(time_s.len(), temperature.len(), "time and temperature must have equal length");
    let n = time_s.len();
    let end = end_idx.min(n);

    if end < 2 {
        return (temperature.to_vec(), 0.0, 0.0);
    }

    // Linear regression on pre-trigger data
    let n_f = end as f64;
    let sum_x: f64 = time_s[..end].iter().sum();
    let sum_y: f64 = temperature[..end].iter().sum();
    let sum_xy: f64 = time_s[..end]
        .iter()
        .zip(temperature[..end].iter())
        .map(|(&x, &y)| x * y)
        .sum();
    let sum_x2: f64 = time_s[..end].iter().map(|&x| x * x).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    let (slope, intercept) = if denom.abs() < 1e-30 {
        (0.0, sum_y / n_f)
    } else {
        let s = (n_f * sum_xy - sum_x * sum_y) / denom;
        let b = (sum_y - s * sum_x) / n_f;
        (s, b)
    };

    let corrected: Vec<f64> = time_s
        .iter()
        .zip(temperature.iter())
        .map(|(&t, &y)| y - (slope * t + intercept))
        .collect();

    (corrected, slope, intercept)
}

/// Normalize a signal to the range [0, 1] using its maximum value.
///
/// Returns `(normalized, t_max_index, max_value)`.
pub fn normalize_signal(data: &[f64]) -> (Vec<f64>, usize, f64) {
    if data.is_empty() {
        return (vec![], 0, 0.0);
    }
    let max_val = data
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    let max_idx = data
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);

    if max_val <= 0.0 {
        return (vec![0.0; data.len()], 0, max_val);
    }
    let normalized: Vec<f64> = data.iter().map(|&v| v / max_val).collect();
    (normalized, max_idx, max_val)
}

/// Find the time at which normalized signal crosses a given fraction via
/// linear interpolation. Searches in `data[start_idx..end_idx]`.
///
/// Returns `None` if the fraction is never reached.
pub fn find_crossing_time(
    time_s: &[f64],
    normalized: &[f64],
    fraction: f64,
    start_idx: usize,
    end_idx: usize,
) -> Option<f64> {
    let end = end_idx.min(normalized.len()).min(time_s.len());
    if start_idx >= end {
        return None;
    }
    for i in start_idx..end - 1 {
        let y0 = normalized[i];
        let y1 = normalized[i + 1];
        if (y0 <= fraction && y1 >= fraction) || (y0 >= fraction && y1 <= fraction) {
            // Linear interpolation
            let t = if (y1 - y0).abs() < 1e-30 {
                time_s[i]
            } else {
                time_s[i] + (fraction - y0) / (y1 - y0) * (time_s[i + 1] - time_s[i])
            };
            return Some(t);
        }
    }
    None
}

/// Extract t_half (time to 50% of T_max) relative to the trigger time.
///
/// # Arguments
/// * `time_s` - Full time axis.
/// * `normalized` - Signal normalized to 0..1.
/// * `trigger_idx` - Index of laser trigger.
/// * `max_idx` - Index of maximum temperature.
///
/// Returns `None` if the crossing cannot be found.
pub fn extract_t_half(
    time_s: &[f64],
    normalized: &[f64],
    trigger_idx: usize,
    max_idx: usize,
) -> Option<f64> {
    let t_trigger = time_s.get(trigger_idx).copied().unwrap_or(0.0);
    let t50 = find_crossing_time(time_s, normalized, 0.5, trigger_idx, max_idx + 1)?;
    Some(t50 - t_trigger)
}

// ---------------------------------------------------------------------------
// Main analyzer
// ---------------------------------------------------------------------------

/// Laser Flash Analyzer — the primary entry point.
///
/// Encapsulates sample parameters, pulse parameters, and analysis configuration.
/// Call [`LaserFlashAnalyzer::analyze`] on a [`Thermogram`] to obtain a
/// [`FlashResult`].
#[derive(Debug, Clone)]
pub struct LaserFlashAnalyzer {
    /// Sample physical parameters.
    pub sample: SampleParams,
    /// Laser pulse parameters.
    pub pulse: PulseParams,
    /// Analysis configuration.
    pub config: LfaConfig,
}

impl LaserFlashAnalyzer {
    /// Create a new analyzer with default configuration.
    pub fn new(sample: SampleParams, pulse: PulseParams) -> Self {
        Self {
            sample,
            pulse,
            config: LfaConfig::default(),
        }
    }

    /// Create a new analyzer with custom configuration.
    pub fn with_config(sample: SampleParams, pulse: PulseParams, config: LfaConfig) -> Self {
        Self { sample, pulse, config }
    }

    /// Run the full LFA analysis on a thermogram.
    ///
    /// Steps performed:
    /// 1. Baseline correction using pre-trigger data.
    /// 2. Optional smoothing.
    /// 3. Signal normalization.
    /// 4. t½ extraction via interpolation.
    /// 5. Parker diffusivity.
    /// 6. Cowan heat-loss correction.
    /// 7. Cape-Lehman finite-pulse correction.
    /// 8. Thermal conductivity calculation.
    pub fn analyze(&self, gram: &Thermogram) -> FlashResult {
        let thickness_m = self.sample.thickness_mm * 1e-3;

        // ---- 1. Baseline correction ----------------------------------------
        let (corrected, _, _) =
            baseline_correction(&gram.time_s, &gram.temperature, gram.trigger_index);

        // ---- 2. Smoothing ---------------------------------------------------
        let smoothed = if self.config.smoothing_window > 1 {
            moving_average_filter(&corrected, self.config.smoothing_window)
        } else {
            corrected
        };

        // ---- 3. Normalise ---------------------------------------------------
        // Only consider data after trigger for normalisation
        let post_trigger: Vec<f64> = smoothed[gram.trigger_index..].to_vec();
        let time_post: Vec<f64> = gram.time_s[gram.trigger_index..].to_vec();

        let (normalized, rel_max_idx, delta_t_max) = normalize_signal(&post_trigger);

        // ---- 4. Extract t½, t_0.25, t_0.75, t_0.80, t_0.20 ----------------
        let t_trigger = gram.time_s.get(gram.trigger_index).copied().unwrap_or(0.0);

        let t_half_opt = find_crossing_time(&time_post, &normalized, 0.5, 0, rel_max_idx + 1)
            .map(|t| t - t_trigger);

        let t25 = find_crossing_time(&time_post, &normalized, 0.25, 0, rel_max_idx + 1)
            .map(|t| t - t_trigger);
        let t75 = find_crossing_time(&time_post, &normalized, 0.75, 0, rel_max_idx + 1)
            .map(|t| t - t_trigger);
        let t20 = find_crossing_time(&time_post, &normalized, 0.20, 0, rel_max_idx + 1)
            .map(|t| t - t_trigger);
        let t80 = find_crossing_time(&time_post, &normalized, 0.80, 0, rel_max_idx + 1)
            .map(|t| t - t_trigger);

        let t_half_s = t_half_opt.unwrap_or(f64::NAN);

        // ---- 5. Parker diffusivity -----------------------------------------
        let alpha_parker = parker_diffusivity(thickness_m, t_half_s);

        // ---- 6. Cowan correction --------------------------------------------
        let (ratio_75_25, ratio_80_20, cowan_kp) = match (t25, t75, t20, t80) {
            (Some(t25v), Some(t75v), Some(t20v), Some(t80v)) if t25v > 0.0 && t20v > 0.0 => {
                let r7525 = t75v / t25v;
                let r8020 = t80v / t20v;
                let kp = cowan_correction_from_ratio(r7525);
                (r7525, r8020, kp)
            }
            (Some(t25v), Some(t75v), _, _) if t25v > 0.0 => {
                let r7525 = t75v / t25v;
                let kp = cowan_correction_from_ratio(r7525);
                (r7525, f64::NAN, kp)
            }
            _ => (f64::NAN, f64::NAN, 1.0),
        };

        let alpha_cowan = if self.config.apply_cowan {
            alpha_parker * cowan_kp
        } else {
            alpha_parker
        };

        // ---- 7. Cape-Lehman correction --------------------------------------
        let pulse_s = self.pulse.duration_us * 1e-6;
        let cl_factor = if self.config.apply_cape_lehman {
            cape_lehman_correction(t_half_s, pulse_s)
        } else {
            1.0
        };
        let alpha_corrected = alpha_cowan * cl_factor;
        let tau_cl = if t_half_s > 0.0 { pulse_s / t_half_s } else { f64::NAN };

        // ---- 8. Thermal conductivity ----------------------------------------
        let k = if self.sample.specific_heat_j_per_kgk > 0.0 {
            thermal_conductivity(
                alpha_corrected,
                self.sample.density_kg_m3,
                self.sample.specific_heat_j_per_kgk,
            )
        } else {
            f64::NAN
        };

        FlashResult {
            alpha_parker_m2_per_s: alpha_parker,
            alpha_cowan_m2_per_s: alpha_cowan,
            alpha_corrected_m2_per_s: alpha_corrected,
            alpha_m2_per_s: alpha_corrected,
            t_half_s,
            k_w_per_mk: k,
            cp_j_per_kgk: self.sample.specific_heat_j_per_kgk,
            cowan: CowanCorrection {
                ratio_75_25,
                ratio_80_20,
                correction_factor: cowan_kp,
            },
            delta_t_max,
            tau_cape_lehman: tau_cl,
        }
    }

    /// Analyze multiple shots at different temperatures and assemble a curve.
    ///
    /// Each `(temperature_c, thermogram)` pair yields one data point.
    pub fn analyze_temperature_series(
        &self,
        shots: &[(f64, Thermogram)],
    ) -> ThermalCurve {
        let mut temperatures_c = Vec::with_capacity(shots.len());
        let mut diffusivity = Vec::with_capacity(shots.len());
        let mut conductivity = Vec::with_capacity(shots.len());
        let mut specific_heat = Vec::with_capacity(shots.len());

        for (temp, gram) in shots {
            let result = self.analyze(gram);
            temperatures_c.push(*temp);
            diffusivity.push(result.alpha_m2_per_s);
            conductivity.push(result.k_w_per_mk);
            specific_heat.push(result.cp_j_per_kgk);
        }

        ThermalCurve {
            temperatures_c,
            diffusivity,
            conductivity,
            specific_heat,
        }
    }

    /// Generate a synthetic adiabatic thermogram for testing.
    ///
    /// Uses the exact rear-face model with `n_terms` Fourier terms.
    pub fn generate_synthetic_thermogram(
        &self,
        alpha_true: f64,
        t_max_rise: f64,
        n_points: usize,
        dt_s: f64,
        pre_trigger_points: usize,
    ) -> Thermogram {
        let thickness_m = self.sample.thickness_mm * 1e-3;
        let t_half_expected = parker_t_half(thickness_m, alpha_true);
        // Total time: 5× t_half to capture plateau
        let total_pts = n_points;
        let time_s: Vec<f64> = (0..total_pts)
            .map(|i| i as f64 * dt_s - pre_trigger_points as f64 * dt_s)
            .collect();

        let temperature: Vec<f64> = time_s
            .iter()
            .map(|&t| {
                if t <= 0.0 {
                    0.0
                } else {
                    let fo = alpha_true * t / (thickness_m * thickness_m);
                    adiabatic_rear_face(fo, self.config.n_series_terms) * t_max_rise
                }
            })
            .collect();

        let _ = t_half_expected; // suppress unused warning

        Thermogram {
            time_s,
            temperature,
            trigger_index: pre_trigger_points,
        }
    }
}

// ---------------------------------------------------------------------------
// Multi-layer analysis
// ---------------------------------------------------------------------------

/// Parameters for one layer in a multi-layer stack.
#[derive(Debug, Clone, Copy)]
pub struct LayerParams {
    /// Thickness of this layer (metres).
    pub thickness_m: f64,
    /// Thermal diffusivity of this layer (m²/s).
    pub alpha_m2_per_s: f64,
    /// Thermal conductivity (W/(m·K)).
    pub k_w_per_mk: f64,
}

/// Estimate effective diffusivity for a two-layer composite using the
/// effective medium approximation (thermal resistance in series).
///
/// ```text
///   1/k_eff = sum(d_i / k_i) / L_total
///   alpha_eff = k_eff / (rho_eff * Cp_eff)
/// ```
/// where rho_eff and Cp_eff are volume-weighted averages.
///
/// Returns `(alpha_eff, k_eff)`.
pub fn two_layer_effective_properties(
    layer1: LayerParams,
    layer2: LayerParams,
    density1: f64,
    density2: f64,
    cp1: f64,
    cp2: f64,
) -> (f64, f64) {
    let l_total = layer1.thickness_m + layer2.thickness_m;
    // Thermal resistance in series: R_total = d1/k1 + d2/k2
    let r_total = layer1.thickness_m / layer1.k_w_per_mk
        + layer2.thickness_m / layer2.k_w_per_mk;
    let k_eff = if r_total > 0.0 { l_total / r_total } else { f64::NAN };

    // Volume-weighted rho and Cp
    let v1 = layer1.thickness_m / l_total;
    let v2 = layer2.thickness_m / l_total;
    let rho_eff = v1 * density1 + v2 * density2;
    let cp_eff = v1 * cp1 + v2 * cp2;

    let alpha_eff = if rho_eff > 0.0 && cp_eff > 0.0 {
        k_eff / (rho_eff * cp_eff)
    } else {
        f64::NAN
    };

    (alpha_eff, k_eff)
}

// ---------------------------------------------------------------------------
// Utility: Savitzky-Golay smoothing (5-point, polynomial order 2)
// ---------------------------------------------------------------------------

/// Apply a 5-point Savitzky-Golay smoothing filter (polynomial order 2).
///
/// Coefficients: [-3, 12, 17, 12, -3] / 35 (standard 5-pt quadratic/quartic).
/// Edge points are handled by replication.
pub fn savitzky_golay_5pt(data: &[f64]) -> Vec<f64> {
    let n = data.len();
    if n < 5 {
        return data.to_vec();
    }
    let coeffs = [-3.0_f64, 12.0, 17.0, 12.0, -3.0];
    let norm = 35.0_f64;
    let mut out = Vec::with_capacity(n);

    // Edge: replicate first/last two points
    out.push(data[0]);
    out.push(data[1]);

    for i in 2..n - 2 {
        let val = coeffs[0] * data[i - 2]
            + coeffs[1] * data[i - 1]
            + coeffs[2] * data[i]
            + coeffs[3] * data[i + 1]
            + coeffs[4] * data[i + 2];
        out.push(val / norm);
    }

    out.push(data[n - 2]);
    out.push(data[n - 1]);
    out
}

// ---------------------------------------------------------------------------
// Helper: build thermogram from raw sensor data
// ---------------------------------------------------------------------------

/// Helper to construct a `Thermogram` from raw acquisition data where the
/// trigger occurs at time zero.
///
/// # Arguments
/// * `pre_trigger_s` - Duration before the trigger (seconds). Will be prepended.
/// * `post_trigger_data` - `(time_after_trigger_s, temperature)` pairs.
/// * `baseline_temp` - Nominal baseline temperature to fill pre-trigger samples.
/// * `dt_pre_s` - Sampling interval for pre-trigger synthetic baseline.
pub fn build_thermogram(
    pre_trigger_duration_s: f64,
    post_trigger_data: &[(f64, f64)],
    baseline_temp: f64,
    dt_pre_s: f64,
) -> Thermogram {
    let n_pre = if dt_pre_s > 0.0 {
        (pre_trigger_duration_s / dt_pre_s).round() as usize
    } else {
        0
    };

    let mut time_s = Vec::with_capacity(n_pre + post_trigger_data.len());
    let mut temperature = Vec::with_capacity(n_pre + post_trigger_data.len());

    for i in 0..n_pre {
        let t = -(pre_trigger_duration_s - i as f64 * dt_pre_s);
        time_s.push(t);
        temperature.push(baseline_temp);
    }

    let trigger_index = n_pre;
    for &(t, temp) in post_trigger_data {
        time_s.push(t);
        temperature.push(temp);
    }

    Thermogram {
        time_s,
        temperature,
        trigger_index,
    }
}

// ---------------------------------------------------------------------------
// Signal quality metrics
// ---------------------------------------------------------------------------

/// Compute signal-to-noise ratio of a thermogram after baseline correction.
///
/// SNR = max_signal / rms_noise, where noise is estimated from the
/// pre-trigger standard deviation.
pub fn thermogram_snr(gram: &Thermogram) -> f64 {
    let n_pre = gram.trigger_index;
    if n_pre < 2 {
        return f64::NAN;
    }

    let pre = &gram.temperature[..n_pre];
    let mean: f64 = pre.iter().sum::<f64>() / n_pre as f64;
    let var: f64 = pre.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n_pre - 1) as f64;
    let rms_noise = var.sqrt();

    if rms_noise < 1e-30 {
        return f64::INFINITY;
    }

    let post = &gram.temperature[n_pre..];
    let max_signal = post.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - mean;
    max_signal / rms_noise
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // Helper: build an ideal adiabatic thermogram for thickness L and alpha.
    fn ideal_thermogram(thickness_m: f64, alpha: f64, n_points: usize, dt: f64) -> Thermogram {
        let pre = 10usize;
        let mut time_s = Vec::with_capacity(pre + n_points);
        let mut temp = Vec::with_capacity(pre + n_points);
        for i in 0..pre {
            time_s.push(-(pre as f64 - i as f64) * dt);
            temp.push(0.0_f64);
        }
        for i in 0..n_points {
            let t = i as f64 * dt;
            let fo = alpha * t / (thickness_m * thickness_m);
            time_s.push(t);
            temp.push(adiabatic_rear_face(fo, 20) * 100.0); // 100 K rise
        }
        Thermogram {
            time_s,
            temperature: temp,
            trigger_index: pre,
        }
    }

    // ---- Parker equation ---------------------------------------------------

    #[test]
    fn test_parker_forward_inverse() {
        let l = 3e-3_f64; // 3 mm
        let alpha = 1.17e-4_f64;
        let t_half = parker_t_half(l, alpha);
        let alpha_back = parker_diffusivity(l, t_half);
        assert!((alpha_back - alpha).abs() / alpha < 1e-10);
    }

    #[test]
    fn test_parker_copper_typical() {
        // Copper 3 mm: t_half ~ 1.07 ms
        let l = 3e-3_f64;
        let alpha = 1.17e-4_f64;
        let t_half = parker_t_half(l, alpha);
        assert!(t_half > 0.005 && t_half < 0.020, "t_half = {t_half:.4e}");
    }

    #[test]
    fn test_parker_silica_typical() {
        // Fused silica 2 mm: t_half ~ several seconds
        let l = 2e-3_f64;
        let alpha = 8.3e-7_f64;
        let t_half = parker_t_half(l, alpha);
        let alpha_back = parker_diffusivity(l, t_half);
        assert!((alpha_back - alpha).abs() / alpha < 1e-10);
    }

    #[test]
    fn test_parker_nan_on_zero_input() {
        assert!(parker_diffusivity(0.0, 0.001).is_nan());
        assert!(parker_diffusivity(0.003, 0.0).is_nan());
        assert!(parker_t_half(0.0, 1e-4).is_nan());
        assert!(parker_t_half(0.003, 0.0).is_nan());
    }

    #[test]
    fn test_parker_thickness_scaling() {
        // alpha ∝ L² for fixed t_half
        let t_half = 0.001_f64;
        let alpha1 = parker_diffusivity(0.002, t_half);
        let alpha2 = parker_diffusivity(0.004, t_half);
        assert!((alpha2 / alpha1 - 4.0).abs() < 1e-10);
    }

    // ---- Rear-face model --------------------------------------------------

    #[test]
    fn test_adiabatic_model_zero_time() {
        assert_eq!(adiabatic_rear_face(0.0, 20), 0.0);
    }

    #[test]
    fn test_adiabatic_model_plateau() {
        // For large Fourier number, should approach 1.0
        let val = adiabatic_rear_face(5.0, 30);
        assert!((val - 1.0).abs() < 1e-6, "val at Fo=5: {val}");
    }

    #[test]
    fn test_adiabatic_model_half_time() {
        // At Fo corresponding to t_half, model should be near 0.5.
        // From Parker: Fo_half = 0.1388
        let fo_half = 0.1388_f64;
        let val = adiabatic_rear_face(fo_half, 30);
        assert!((val - 0.5).abs() < 0.01, "rear face at Fo_half = {val}");
    }

    #[test]
    fn test_adiabatic_model_monotone_increasing() {
        let fos = [0.01, 0.05, 0.1388, 0.3, 0.5, 1.0, 2.0];
        let mut prev = 0.0_f64;
        for &fo in &fos {
            let val = adiabatic_rear_face(fo, 30);
            assert!(val > prev, "not monotone at Fo={fo}");
            prev = val;
        }
    }

    #[test]
    fn test_adiabatic_model_convergence() {
        // 20 vs 50 terms should give very similar results
        let fo = 0.2;
        let v20 = adiabatic_rear_face(fo, 20);
        let v50 = adiabatic_rear_face(fo, 50);
        assert!((v20 - v50).abs() < 1e-8);
    }

    // ---- Cowan correction -------------------------------------------------

    #[test]
    fn test_cowan_adiabatic_ratio() {
        // For u = 2.684 (adiabatic), correction should be very close to 1.0
        let kp = cowan_correction_from_ratio(2.684);
        assert!((kp - 1.0).abs() < 0.01, "Kp(2.684) = {kp}");
    }

    #[test]
    fn test_cowan_increases_with_ratio() {
        // More heat loss => larger ratio => larger correction
        let kp1 = cowan_correction_from_ratio(3.0);
        let kp2 = cowan_correction_from_ratio(5.0);
        let kp3 = cowan_correction_from_ratio(8.0);
        assert!(kp2 > kp1, "kp2={kp2} should be > kp1={kp1}");
        assert!(kp3 > kp2, "kp3={kp3} should be > kp2={kp2}");
    }

    #[test]
    fn test_cowan_zero_ratio_returns_one() {
        let kp = cowan_correction_from_ratio(0.0);
        assert!((kp - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_cowan_8020_adiabatic() {
        // Adiabatic ratio 80/20 ~ 3.812 should give Kp close to 1
        let kp = cowan_correction_from_ratio_8020(3.812);
        assert!((kp - 1.0).abs() < 0.05, "Kp(3.812) = {kp}");
    }

    // ---- Cape-Lehman correction -------------------------------------------

    #[test]
    fn test_cape_lehman_zero_pulse() {
        // No pulse => factor = 1
        let kp = cape_lehman_correction(0.001, 0.0);
        assert_eq!(kp, 1.0);
    }

    #[test]
    fn test_cape_lehman_long_pulse_increases_alpha() {
        // Longer pulse => larger correction factor
        let kp_short = cape_lehman_correction(0.001, 1e-6);
        let kp_long = cape_lehman_correction(0.001, 100e-6);
        assert!(kp_long > kp_short);
    }

    #[test]
    fn test_cape_lehman_small_tau() {
        // tau << 0.02 => factor ~ 1
        let kp = cape_lehman_correction(0.010, 1e-7); // tau = 1e-5
        assert!((kp - 1.0).abs() < 1e-3, "kp={kp}");
    }

    #[test]
    fn test_cape_lehman_typical_copper() {
        // Copper 3 mm, t_half ~ 1 ms, pulse 400 us => tau = 0.4
        // Should give correction > 1
        let kp = cape_lehman_correction(0.001, 400e-6);
        assert!(kp > 1.0 && kp < 3.0, "kp={kp}");
    }

    // ---- Thermal conductivity ---------------------------------------------

    #[test]
    fn test_thermal_conductivity_copper() {
        let k = thermal_conductivity(1.17e-4, 8960.0, 385.0);
        // Expected ~401 W/mK
        assert!((k - 401.0).abs() / 401.0 < 0.05, "k = {k}");
    }

    #[test]
    fn test_thermal_conductivity_aluminum() {
        let k = thermal_conductivity(9.7e-5, 2700.0, 897.0);
        assert!((k - 237.0).abs() / 237.0 < 0.05, "k = {k}");
    }

    #[test]
    fn test_thermal_conductivity_proportional() {
        // Doubling alpha should double k
        let k1 = thermal_conductivity(1e-5, 3000.0, 800.0);
        let k2 = thermal_conductivity(2e-5, 3000.0, 800.0);
        assert!((k2 / k1 - 2.0).abs() < 1e-10);
    }

    // ---- Specific heat comparison method ----------------------------------

    #[test]
    fn test_specific_heat_comparison_identity() {
        // Same material: should return cp_ref
        let cp = specific_heat_comparative(500.0, 10.0, 200.0, 10.0, 200.0);
        assert!((cp - 500.0).abs() < 1e-10);
    }

    #[test]
    fn test_specific_heat_comparison_scaling() {
        // Sample has twice the temperature rise => sample Cp is half
        let cp = specific_heat_comparative(500.0, 10.0, 200.0, 20.0, 200.0);
        assert!((cp - 250.0).abs() < 1e-10);
    }

    #[test]
    fn test_specific_heat_comparison_mass_ratio() {
        // Reference is twice the mass => sample Cp is double
        let cp = specific_heat_comparative(500.0, 10.0, 400.0, 10.0, 200.0);
        assert!((cp - 1000.0).abs() < 1e-10);
    }

    #[test]
    fn test_specific_heat_comparison_nan_on_zero() {
        let cp = specific_heat_comparative(500.0, 10.0, 200.0, 0.0, 200.0);
        assert!(cp.is_nan());
    }

    // ---- Baseline correction ----------------------------------------------

    #[test]
    fn test_baseline_correction_flat() {
        // Flat baseline at 100 should be removed
        let time = vec![0.0, 0.001, 0.002, 0.003, 0.004];
        let temp = vec![100.0, 100.0, 100.0, 101.0, 102.0];
        let (corr, slope, intercept) = baseline_correction(&time, &temp, 3);
        assert!(slope.abs() < 1e-6);
        assert!((intercept - 100.0).abs() < 0.1);
        // Pre-trigger region should be near zero
        for i in 0..3 {
            assert!(corr[i].abs() < 0.5, "corr[{i}] = {}", corr[i]);
        }
    }

    #[test]
    fn test_baseline_correction_drifting() {
        // Linear drift of 10 K/s
        let time: Vec<f64> = (0..10).map(|i| i as f64 * 0.001).collect();
        let temp: Vec<f64> = time.iter().map(|&t| 10.0 * t + 20.0).collect();
        let (corr, slope, intercept) = baseline_correction(&time, &temp, 5);
        assert!((slope - 10.0).abs() < 0.1, "slope={slope}");
        assert!((intercept - 20.0).abs() < 0.1, "intercept={intercept}");
        for &c in corr.iter().take(5) {
            assert!(c.abs() < 0.05, "corr={c}");
        }
    }

    #[test]
    fn test_baseline_correction_empty_pre_trigger() {
        let time = vec![0.0, 0.001, 0.002];
        let temp = vec![0.0, 1.0, 2.0];
        let (corr, _, _) = baseline_correction(&time, &temp, 1);
        assert_eq!(corr.len(), 3);
    }

    // ---- Signal normalization ---------------------------------------------

    #[test]
    fn test_normalize_signal_basic() {
        let data = vec![0.0, 0.5, 1.0, 0.8, 0.6];
        let (norm, idx, max) = normalize_signal(&data);
        assert_eq!(idx, 2);
        assert!((max - 1.0).abs() < 1e-10);
        assert!((norm[2] - 1.0).abs() < 1e-10);
        assert!((norm[0] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_signal_all_zeros() {
        let data = vec![0.0, 0.0, 0.0];
        let (norm, _, max) = normalize_signal(&data);
        assert!(max <= 0.0);
        assert!(norm.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_normalize_signal_max_is_one() {
        let data = vec![2.0, 5.0, 3.0, 1.0];
        let (norm, _, _) = normalize_signal(&data);
        let max_norm = norm.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!((max_norm - 1.0).abs() < 1e-10);
    }

    // ---- Half-time extraction ---------------------------------------------

    #[test]
    fn test_extract_t_half_linear() {
        // Signal linearly rising from 0 to 1 between t=0 and t=1 s
        let n = 101;
        let time_s: Vec<f64> = (0..n).map(|i| i as f64 * 0.01).collect();
        let normalized: Vec<f64> = time_s.iter().map(|&t| t).collect();
        let t50 = find_crossing_time(&time_s, &normalized, 0.5, 0, n).unwrap();
        assert!((t50 - 0.5).abs() < 0.015, "t50={t50}");
    }

    #[test]
    fn test_extract_t_half_ideal_thermogram() {
        let l = 3e-3_f64;
        let alpha = 1.17e-4_f64;
        let t_half_theory = parker_t_half(l, alpha);
        let dt = t_half_theory / 200.0;
        let gram = ideal_thermogram(l, alpha, 2000, dt);

        let post: Vec<f64> = gram.temperature[gram.trigger_index..].to_vec();
        let time_post: Vec<f64> = gram.time_s[gram.trigger_index..].to_vec();
        let (normalized, rel_max, _) = normalize_signal(&post);
        let t_trigger = gram.time_s[gram.trigger_index];
        let t50 = find_crossing_time(&time_post, &normalized, 0.5, 0, rel_max + 1)
            .map(|t| t - t_trigger)
            .unwrap();

        let error = (t50 - t_half_theory).abs() / t_half_theory;
        assert!(error < 0.03, "t50 error = {:.1}%", error * 100.0);
    }

    #[test]
    fn test_find_crossing_returns_none_below_threshold() {
        let time_s = vec![0.0, 0.1, 0.2];
        let normalized = vec![0.1, 0.2, 0.3]; // never reaches 0.5
        let result = find_crossing_time(&time_s, &normalized, 0.5, 0, 3);
        assert!(result.is_none());
    }

    // ---- Moving average filter -------------------------------------------

    #[test]
    fn test_moving_average_length_preserved() {
        let data: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let smoothed = moving_average_filter(&data, 5);
        assert_eq!(smoothed.len(), data.len());
    }

    #[test]
    fn test_moving_average_constant_signal() {
        let data = vec![5.0_f64; 50];
        let smoothed = moving_average_filter(&data, 7);
        for &v in &smoothed {
            assert!((v - 5.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_moving_average_zero_window() {
        let data = vec![1.0, 2.0, 3.0];
        let smoothed = moving_average_filter(&data, 0);
        assert_eq!(smoothed, data);
    }

    // ---- Savitzky-Golay ---------------------------------------------------

    #[test]
    fn test_savitzky_golay_preserves_length() {
        let data: Vec<f64> = (0..20).map(|i| i as f64 * 0.1).collect();
        let sg = savitzky_golay_5pt(&data);
        assert_eq!(sg.len(), data.len());
    }

    #[test]
    fn test_savitzky_golay_smooth_polynomial() {
        // Quadratic input should be reproduced nearly exactly by SG
        let data: Vec<f64> = (0..20).map(|i| (i as f64).powi(2) * 0.01).collect();
        let sg = savitzky_golay_5pt(&data);
        // Interior points should match original
        for i in 2..18 {
            assert!((sg[i] - data[i]).abs() < 1e-8, "i={i} sg={} data={}", sg[i], data[i]);
        }
    }

    #[test]
    fn test_savitzky_golay_short_data() {
        let data = vec![1.0, 2.0, 3.0];
        let sg = savitzky_golay_5pt(&data);
        assert_eq!(sg, data); // falls through to identity for n < 5
    }

    // ---- Material presets -------------------------------------------------

    #[test]
    fn test_material_presets_count() {
        let presets = material_presets();
        assert_eq!(presets.len(), 6);
    }

    #[test]
    fn test_material_presets_copper() {
        let cu = material_presets().iter().find(|m| m.name == "Copper").unwrap();
        assert!((cu.alpha_ref - 1.17e-4).abs() / 1.17e-4 < 0.01);
        assert!((cu.k_ref - 401.0).abs() < 5.0);
        assert!((cu.cp_ref - 385.0).abs() < 5.0);
        assert!((cu.density - 8960.0).abs() < 10.0);
    }

    #[test]
    fn test_material_presets_consistency() {
        // For each preset: k = alpha * rho * Cp should be consistent
        for mat in material_presets() {
            let k_computed = mat.alpha_ref * mat.density * mat.cp_ref;
            let error = (k_computed - mat.k_ref).abs() / mat.k_ref;
            assert!(error < 0.15, "Inconsistency for {}: computed k={k_computed:.2}, k_ref={}", mat.name, mat.k_ref);
        }
    }

    #[test]
    fn test_material_presets_all_positive() {
        for mat in material_presets() {
            assert!(mat.alpha_ref > 0.0, "{} alpha", mat.name);
            assert!(mat.k_ref > 0.0, "{} k", mat.name);
            assert!(mat.cp_ref > 0.0, "{} Cp", mat.name);
            assert!(mat.density > 0.0, "{} rho", mat.name);
        }
    }

    #[test]
    fn test_material_presets_alumina() {
        let al2o3 = material_presets()
            .iter()
            .find(|m| m.name == "Alumina (Al2O3)")
            .unwrap();
        assert!((al2o3.alpha_ref - 1.2e-5).abs() / 1.2e-5 < 0.01);
    }

    // ---- Full analyzer integration tests ----------------------------------

    #[test]
    fn test_analyzer_copper_parker() {
        let alpha_true = 1.17e-4_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);

        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.05 }; // near-instantaneous
        let mut config = LfaConfig::default();
        config.apply_cowan = false;
        config.apply_cape_lehman = false;
        config.smoothing_window = 0;

        let analyzer = LaserFlashAnalyzer::with_config(params, pulse, config);
        let dt = t_half_theory / 200.0;
        let gram = ideal_thermogram(thickness_m, alpha_true, 600, dt);
        let result = analyzer.analyze(&gram);

        let error = (result.alpha_parker_m2_per_s - alpha_true).abs() / alpha_true;
        assert!(error < 0.05, "Parker error = {:.1}%", error * 100.0);
    }

    #[test]
    fn test_analyzer_thermal_conductivity_copper() {
        let alpha_true = 1.17e-4_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);

        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.05 };
        let mut config = LfaConfig::default();
        config.apply_cowan = false;
        config.apply_cape_lehman = false;
        config.smoothing_window = 0;

        let analyzer = LaserFlashAnalyzer::with_config(params, pulse, config);
        let dt = t_half_theory / 200.0;
        let gram = ideal_thermogram(thickness_m, alpha_true, 600, dt);
        let result = analyzer.analyze(&gram);

        // k should be close to ~401 W/mK
        assert!(result.k_w_per_mk > 300.0 && result.k_w_per_mk < 500.0,
            "k = {} W/mK", result.k_w_per_mk);
    }

    #[test]
    fn test_analyzer_silica_slow() {
        let alpha_true = 8.3e-7_f64;
        let thickness_m = 2e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);

        let params = SampleParams {
            thickness_mm: 2.0,
            density_kg_m3: 2200.0,
            specific_heat_j_per_kgk: 740.0,
            mass_mg: 200.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.02 };
        let mut config = LfaConfig::default();
        config.apply_cowan = false;
        config.apply_cape_lehman = false;
        config.smoothing_window = 0;

        let analyzer = LaserFlashAnalyzer::with_config(params, pulse, config);
        let dt = t_half_theory / 200.0;
        let gram = ideal_thermogram(thickness_m, alpha_true, 600, dt);
        let result = analyzer.analyze(&gram);

        let error = (result.alpha_parker_m2_per_s - alpha_true).abs() / alpha_true;
        assert!(error < 0.03, "Silica error = {:.1}%", error * 100.0);
    }

    #[test]
    fn test_analyzer_t_half_extracted() {
        let alpha_true = 9.7e-5_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);

        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 2700.0,
            specific_heat_j_per_kgk: 897.0,
            mass_mg: 300.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.03 };
        let mut config = LfaConfig::default();
        config.apply_cowan = false;
        config.apply_cape_lehman = false;
        config.smoothing_window = 0;

        let analyzer = LaserFlashAnalyzer::with_config(params, pulse, config);
        let dt = t_half_theory / 200.0;
        let gram = ideal_thermogram(thickness_m, alpha_true, 600, dt);
        let result = analyzer.analyze(&gram);

        let error = (result.t_half_s - t_half_theory).abs() / t_half_theory;
        assert!(error < 0.05, "t_half error = {:.1}%", error * 100.0);
    }

    #[test]
    fn test_analyzer_cowan_applied() {
        // Verify that Cowan correction changes alpha
        let alpha_true = 1.17e-4_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);

        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.05 };
        let dt = t_half_theory / 200.0;
        let gram = ideal_thermogram(thickness_m, alpha_true, 600, dt);

        let mut config_no_cowan = LfaConfig::default();
        config_no_cowan.apply_cowan = false;
        config_no_cowan.apply_cape_lehman = false;
        config_no_cowan.smoothing_window = 0;

        let mut config_cowan = config_no_cowan;
        config_cowan.apply_cowan = true;

        let analyzer_no = LaserFlashAnalyzer::with_config(params, pulse, config_no_cowan);
        let analyzer_co = LaserFlashAnalyzer::with_config(params, pulse, config_cowan);

        let res_no = analyzer_no.analyze(&gram);
        let res_co = analyzer_co.analyze(&gram);

        // For near-adiabatic input, Cowan factor ~ 1 (within ±10%)
        let ratio = res_co.alpha_cowan_m2_per_s / res_no.alpha_parker_m2_per_s;
        assert!(ratio > 0.9 && ratio < 1.1, "Cowan ratio = {ratio}");
    }

    #[test]
    fn test_analyzer_cape_lehman_correction_nonzero() {
        let alpha_true = 1.17e-4_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);
        let dt = t_half_theory / 200.0;
        let gram = ideal_thermogram(thickness_m, alpha_true, 600, dt);

        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        // Very long pulse so Cape-Lehman actually matters
        let pulse = PulseParams { duration_us: 200.0, energy_j: 0.05 };
        let mut cfg = LfaConfig::default();
        cfg.apply_cowan = false;
        cfg.apply_cape_lehman = true;
        cfg.smoothing_window = 0;

        let analyzer = LaserFlashAnalyzer::with_config(params, pulse, cfg);
        let result = analyzer.analyze(&gram);
        // With 200 us pulse and ~1 ms t_half, tau ~ 0.2, correction > 1
        assert!(result.alpha_corrected_m2_per_s >= result.alpha_cowan_m2_per_s,
            "CL should increase alpha for long pulse");
    }

    #[test]
    fn test_analyzer_delta_t_max_positive() {
        let alpha_true = 1.17e-4_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);
        let dt = t_half_theory / 100.0;
        let gram = ideal_thermogram(thickness_m, alpha_true, 300, dt);

        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.05 };
        let analyzer = LaserFlashAnalyzer::new(params, pulse);
        let result = analyzer.analyze(&gram);
        assert!(result.delta_t_max > 0.0, "delta_t_max should be positive");
    }

    // ---- Multi-layer analysis ---------------------------------------------

    #[test]
    fn test_two_layer_single_layer_identity() {
        // Two identical layers => effective = individual
        let layer = LayerParams {
            thickness_m: 1e-3,
            alpha_m2_per_s: 1e-4,
            k_w_per_mk: 100.0,
        };
        let (alpha_eff, k_eff) = two_layer_effective_properties(
            layer, layer, 2700.0, 2700.0, 900.0, 900.0,
        );
        assert!((k_eff - 100.0).abs() < 0.01, "k_eff = {k_eff}");
    }

    #[test]
    fn test_two_layer_series_resistance() {
        // Thermal resistance in series
        let l1 = LayerParams { thickness_m: 1e-3, alpha_m2_per_s: 1e-4, k_w_per_mk: 100.0 };
        let l2 = LayerParams { thickness_m: 1e-3, alpha_m2_per_s: 1e-4, k_w_per_mk: 50.0 };
        let (_, k_eff) = two_layer_effective_properties(l1, l2, 2700.0, 2700.0, 900.0, 900.0);
        // R_total = 1e-3/100 + 1e-3/50 = 1e-5 + 2e-5 = 3e-5 m²K/W
        // k_eff = 2e-3 / 3e-5 = 66.67 W/mK
        assert!((k_eff - 66.67).abs() < 0.1, "k_eff = {k_eff}");
    }

    #[test]
    fn test_two_layer_positive_result() {
        let l1 = LayerParams { thickness_m: 2e-3, alpha_m2_per_s: 1e-4, k_w_per_mk: 200.0 };
        let l2 = LayerParams { thickness_m: 1e-3, alpha_m2_per_s: 5e-6, k_w_per_mk: 10.0 };
        let (alpha_eff, k_eff) = two_layer_effective_properties(
            l1, l2, 7000.0, 2000.0, 500.0, 700.0,
        );
        assert!(alpha_eff > 0.0 && k_eff > 0.0);
    }

    // ---- SNR measurement --------------------------------------------------

    #[test]
    fn test_snr_positive_for_real_signal() {
        let alpha = 1.17e-4_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha);
        let dt = t_half_theory / 100.0;
        let gram = ideal_thermogram(thickness_m, alpha, 300, dt);
        let snr = thermogram_snr(&gram);
        // Ideal thermogram with zero noise => SNR = infinity
        assert!(snr.is_infinite() || snr > 1000.0, "snr = {snr}");
    }

    #[test]
    fn test_snr_nan_for_no_pre_trigger() {
        let gram = Thermogram {
            time_s: vec![0.0, 0.001, 0.002],
            temperature: vec![0.0, 1.0, 2.0],
            trigger_index: 0,
        };
        let snr = thermogram_snr(&gram);
        assert!(snr.is_nan());
    }

    // ---- build_thermogram helper ------------------------------------------

    #[test]
    fn test_build_thermogram_trigger_index() {
        let post = vec![(0.0, 0.0), (0.001, 5.0), (0.002, 10.0)];
        let gram = build_thermogram(0.005, &post, 20.0, 0.001);
        assert_eq!(gram.trigger_index, 5); // 0.005 / 0.001 = 5 pre-trigger points
    }

    #[test]
    fn test_build_thermogram_length() {
        let post: Vec<(f64, f64)> = (0..100).map(|i| (i as f64 * 0.001, i as f64)).collect();
        let gram = build_thermogram(0.01, &post, 0.0, 0.001);
        assert_eq!(gram.time_s.len(), gram.temperature.len());
        assert_eq!(gram.time_s.len(), 110); // 10 pre + 100 post
    }

    // ---- Temperature series analysis --------------------------------------

    #[test]
    fn test_temperature_series_length() {
        let alpha_true = 1.17e-4_f64;
        let thickness_m = 3e-3_f64;
        let t_half_theory = parker_t_half(thickness_m, alpha_true);
        let dt = t_half_theory / 100.0;

        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.05 };
        let mut cfg = LfaConfig::default();
        cfg.apply_cowan = false;
        cfg.apply_cape_lehman = false;
        cfg.smoothing_window = 0;
        let analyzer = LaserFlashAnalyzer::with_config(params, pulse, cfg);

        let shots: Vec<(f64, Thermogram)> = vec![25.0, 100.0, 200.0, 300.0]
            .into_iter()
            .map(|t| (t, ideal_thermogram(thickness_m, alpha_true, 300, dt)))
            .collect();

        let curve = analyzer.analyze_temperature_series(&shots);
        assert_eq!(curve.temperatures_c.len(), 4);
        assert_eq!(curve.diffusivity.len(), 4);
        assert_eq!(curve.conductivity.len(), 4);
    }

    // ---- Synthetic thermogram generation ----------------------------------

    #[test]
    fn test_synthetic_thermogram_shape() {
        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.05 };
        let analyzer = LaserFlashAnalyzer::new(params, pulse);

        let gram = analyzer.generate_synthetic_thermogram(1.17e-4, 10.0, 500, 5e-5, 10);
        assert_eq!(gram.trigger_index, 10);
        // Pre-trigger should be near zero
        for &t in gram.temperature[..10].iter() {
            assert!(t.abs() < 1e-10, "pre-trigger not zero: {t}");
        }
        // Post trigger should rise and plateau
        let post_max = gram.temperature[10..].iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(post_max > 4.0, "max rise = {post_max}");
    }

    #[test]
    fn test_synthetic_thermogram_approaches_t_max() {
        let params = SampleParams {
            thickness_mm: 3.0,
            density_kg_m3: 8960.0,
            specific_heat_j_per_kgk: 385.0,
            mass_mg: 500.0,
        };
        let pulse = PulseParams { duration_us: 1.0, energy_j: 0.05 };
        let analyzer = LaserFlashAnalyzer::new(params, pulse);

        // Large time window to reach near-plateau
        let gram = analyzer.generate_synthetic_thermogram(1.17e-4, 10.0, 500, 5e-5, 10);
        let last = gram.temperature[gram.temperature.len() - 1];
        assert!(last > 9.0, "plateau not reached: last = {last}");
    }
}

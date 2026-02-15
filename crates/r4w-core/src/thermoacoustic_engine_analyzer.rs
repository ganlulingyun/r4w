//! # Thermoacoustic Engine Analyzer
//!
//! Signal processing for thermoacoustic engine and refrigerator characterization.
//! Implements Rott's linear thermoacoustic theory, standing-wave acoustic field
//! analysis, two-microphone impedance measurement, and performance metrics for
//! both prime movers and heat pumps.
//!
//! ## Key Equations
//!
//! - **Standing wave frequency**: f_n = n*c/(2*L) (closed-closed), f_n = n*c/(4*L) (closed-open)
//! - **Thermal penetration depth**: delta_kappa = sqrt(2*kappa/omega)
//! - **Viscous penetration depth**: delta_nu = sqrt(2*nu/omega)
//! - **Acoustic power**: W_dot = 0.5 * Re(p1 * conj(u1)) * A
//! - **Drive ratio**: DR = |p1| / p_mean
//! - **Carnot efficiency**: eta_C = 1 - T_cold/T_hot
//! - **Carnot COP**: COP_C = T_cold / (T_hot - T_cold)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::thermoacoustic_engine_analyzer::{
//!     GasProperties, ResonatorConfig, ResonatorType, StackConfig,
//!     ThermoacousticAnalyzer,
//! };
//!
//! let gas = GasProperties::air();
//! let resonator = ResonatorConfig {
//!     length_m: 1.0,
//!     diameter_m: 0.05,
//!     mean_pressure_pa: 101325.0,
//!     mean_temperature_k: 300.0,
//!     resonator_type: ResonatorType::ClosedClosed,
//! };
//! let stack = StackConfig {
//!     plate_spacing_m: 0.4e-3,
//!     plate_thickness_m: 0.1e-3,
//!     length_m: 0.05,
//!     position_from_closed_end_m: 0.15,
//! };
//! let analyzer = ThermoacousticAnalyzer::new(gas, resonator, stack);
//! let freq = analyzer.resonant_frequency(1);
//! assert!((freq - 171.5).abs() < 1.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Reference atmospheric pressure (Pa).
const ATM_PA: f64 = 101_325.0;

// ---------------------------------------------------------------------------
// GasProperties
// ---------------------------------------------------------------------------

/// Thermophysical properties of the working gas.
#[derive(Debug, Clone, Copy)]
pub struct GasProperties {
    /// Ratio of specific heats (Cp/Cv), dimensionless.
    pub gamma: f64,
    /// Mean density (kg/m^3) at operating conditions.
    pub density_kgm3: f64,
    /// Speed of sound (m/s).
    pub speed_of_sound_ms: f64,
    /// Dynamic viscosity (Pa*s).
    pub viscosity_pas: f64,
    /// Thermal conductivity (W/(m*K)).
    pub thermal_conductivity: f64,
    /// Prandtl number, dimensionless.
    pub prandtl_number: f64,
    /// Specific heat at constant pressure (J/(kg*K)).
    pub specific_heat_cp: f64,
}

impl GasProperties {
    /// Air at 300 K, 1 atm.
    pub fn air() -> Self {
        Self {
            gamma: 1.4,
            density_kgm3: 1.177,
            speed_of_sound_ms: 343.0,
            viscosity_pas: 1.846e-5,
            thermal_conductivity: 0.0263,
            prandtl_number: 0.713,
            specific_heat_cp: 1005.0,
        }
    }

    /// Helium at 300 K, 1 atm.
    pub fn helium() -> Self {
        Self {
            gamma: 1.667,
            density_kgm3: 0.164,
            speed_of_sound_ms: 1020.0,
            viscosity_pas: 1.96e-5,
            thermal_conductivity: 0.152,
            prandtl_number: 0.68,
            specific_heat_cp: 5193.0,
        }
    }

    /// Argon at 300 K, 1 atm.
    pub fn argon() -> Self {
        Self {
            gamma: 1.667,
            density_kgm3: 1.634,
            speed_of_sound_ms: 323.0,
            viscosity_pas: 2.27e-5,
            thermal_conductivity: 0.0177,
            prandtl_number: 0.667,
            specific_heat_cp: 520.0,
        }
    }

    /// Kinematic viscosity nu = mu / rho (m^2/s).
    pub fn kinematic_viscosity(&self) -> f64 {
        self.viscosity_pas / self.density_kgm3
    }

    /// Thermal diffusivity kappa = k / (rho * cp) (m^2/s).
    pub fn thermal_diffusivity(&self) -> f64 {
        self.thermal_conductivity / (self.density_kgm3 * self.specific_heat_cp)
    }
}

// ---------------------------------------------------------------------------
// ResonatorConfig
// ---------------------------------------------------------------------------

/// Resonator boundary condition type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResonatorType {
    /// Both ends closed (pressure antinodes at both ends).
    ClosedClosed,
    /// One end closed, one end open (quarter-wave).
    ClosedOpen,
}

/// Acoustic resonator geometry and operating conditions.
#[derive(Debug, Clone, Copy)]
pub struct ResonatorConfig {
    /// Resonator length (m).
    pub length_m: f64,
    /// Resonator inner diameter (m).
    pub diameter_m: f64,
    /// Mean (static) pressure (Pa).
    pub mean_pressure_pa: f64,
    /// Mean temperature (K).
    pub mean_temperature_k: f64,
    /// Boundary condition type.
    pub resonator_type: ResonatorType,
}

impl ResonatorConfig {
    /// Cross-sectional area (m^2).
    pub fn cross_section_area(&self) -> f64 {
        PI * (self.diameter_m / 2.0).powi(2)
    }
}

// ---------------------------------------------------------------------------
// StackConfig
// ---------------------------------------------------------------------------

/// Thermoacoustic stack/regenerator geometry.
#[derive(Debug, Clone, Copy)]
pub struct StackConfig {
    /// Spacing between parallel plates (m).
    pub plate_spacing_m: f64,
    /// Plate thickness (m).
    pub plate_thickness_m: f64,
    /// Stack length along resonator axis (m).
    pub length_m: f64,
    /// Position of stack centre from closed end (m).
    pub position_from_closed_end_m: f64,
}

impl StackConfig {
    /// Blockage ratio: open area fraction of the stack cross section.
    pub fn blockage_ratio(&self) -> f64 {
        self.plate_spacing_m / (self.plate_spacing_m + self.plate_thickness_m)
    }

    /// Hydraulic radius of a parallel-plate gap: r_h = y0/2 = spacing/2.
    pub fn hydraulic_radius(&self) -> f64 {
        self.plate_spacing_m / 2.0
    }
}

// ---------------------------------------------------------------------------
// Penetration depths
// ---------------------------------------------------------------------------

/// Thermal penetration depth delta_kappa = sqrt(2 * kappa / omega).
pub fn thermal_penetration_depth(gas: &GasProperties, freq_hz: f64) -> f64 {
    let omega = 2.0 * PI * freq_hz;
    let kappa = gas.thermal_diffusivity();
    (2.0 * kappa / omega).sqrt()
}

/// Viscous penetration depth delta_nu = sqrt(2 * nu / omega).
pub fn viscous_penetration_depth(gas: &GasProperties, freq_hz: f64) -> f64 {
    let omega = 2.0 * PI * freq_hz;
    let nu = gas.kinematic_viscosity();
    (2.0 * nu / omega).sqrt()
}

/// Plate spacing ratio y0 / delta_kappa (optimal ~ 2-4).
pub fn spacing_ratio(stack: &StackConfig, gas: &GasProperties, freq_hz: f64) -> f64 {
    let delta_k = thermal_penetration_depth(gas, freq_hz);
    (stack.plate_spacing_m / 2.0) / delta_k
}

// ---------------------------------------------------------------------------
// Acoustic field (standing wave)
// ---------------------------------------------------------------------------

/// Standing-wave pressure amplitude at position x from the closed end.
/// p1(x) = P_A * cos(k*x).
pub fn pressure_amplitude(p_a: f64, k: f64, x: f64) -> f64 {
    p_a * (k * x).cos()
}

/// Standing-wave velocity amplitude at position x from the closed end.
/// u1(x) = (P_A / (rho*c)) * sin(k*x).
pub fn velocity_amplitude(p_a: f64, k: f64, x: f64, rho: f64, c: f64) -> f64 {
    (p_a / (rho * c)) * (k * x).sin()
}

/// Time-averaged acoustic power at position x (W).
/// W_dot = 0.5 * p1 * u1 * A (for in-phase standing wave components at x).
/// In a pure standing wave p and u are 90 degrees out of phase, so net power
/// is zero everywhere. For a *travelling-wave component* the power is:
/// W_dot = 0.5 * |p_forward|^2 / (rho*c) * A  (forward)
///       minus 0.5 * |p_backward|^2 / (rho*c) * A  (backward).
///
/// This function returns the travelling-wave acoustic power given forward and
/// backward pressure amplitudes.
pub fn acoustic_power_travelling(
    p_forward: f64,
    p_backward: f64,
    rho: f64,
    c: f64,
    area: f64,
) -> f64 {
    0.5 * (p_forward.powi(2) - p_backward.powi(2)) / (rho * c) * area
}

/// Acoustic power for a pure travelling wave with pressure amplitude p_a.
pub fn acoustic_power_simple(p_a: f64, rho: f64, c: f64, area: f64) -> f64 {
    0.5 * p_a.powi(2) / (rho * c) * area
}

/// Drive ratio: DR = |p1| / p_mean.
pub fn drive_ratio(p1_amplitude: f64, p_mean: f64) -> f64 {
    p1_amplitude.abs() / p_mean
}

// ---------------------------------------------------------------------------
// Performance metrics
// ---------------------------------------------------------------------------

/// Carnot efficiency for a prime mover: eta_C = 1 - T_cold / T_hot.
pub fn carnot_efficiency(t_hot_k: f64, t_cold_k: f64) -> f64 {
    1.0 - t_cold_k / t_hot_k
}

/// Carnot COP for a refrigerator: COP_C = T_cold / (T_hot - T_cold).
pub fn carnot_cop(t_hot_k: f64, t_cold_k: f64) -> f64 {
    t_cold_k / (t_hot_k - t_cold_k)
}

/// Actual engine efficiency: W_output / Q_hot.
pub fn engine_efficiency(w_output: f64, q_hot: f64) -> f64 {
    if q_hot.abs() < 1e-30 {
        return 0.0;
    }
    w_output / q_hot
}

/// Refrigerator COP: Q_cold / W_input.
pub fn refrigerator_cop(q_cold: f64, w_input: f64) -> f64 {
    if w_input.abs() < 1e-30 {
        return 0.0;
    }
    q_cold / w_input
}

/// Relative efficiency or COP: actual / Carnot.
pub fn relative_performance(actual: f64, carnot: f64) -> f64 {
    if carnot.abs() < 1e-30 {
        return 0.0;
    }
    actual / carnot
}

// ---------------------------------------------------------------------------
// Engine/refrigerator performance result
// ---------------------------------------------------------------------------

/// Collected performance metrics for a thermoacoustic device.
#[derive(Debug, Clone, Copy)]
pub struct PerformanceMetrics {
    /// Acoustic power produced (engine) or consumed (refrigerator) (W).
    pub acoustic_power_w: f64,
    /// Hot-side heat flow (W).
    pub q_hot_w: f64,
    /// Cold-side heat flow (W).
    pub q_cold_w: f64,
    /// Actual efficiency (engine) or COP (refrigerator).
    pub actual_performance: f64,
    /// Carnot limit.
    pub carnot_limit: f64,
    /// Relative performance (actual / Carnot).
    pub relative_perf: f64,
}

// ---------------------------------------------------------------------------
// Signal processing: FFT and harmonic analysis
// ---------------------------------------------------------------------------

/// Radix-2 DIT FFT (in-place). `re` and `im` are real/imaginary parts.
/// Length must be a power of two.
fn fft_inplace(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert_eq!(n, im.len());
    assert!(n.is_power_of_two(), "FFT length must be power of two");

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
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = -2.0 * PI / len as f64;
        let w_re = angle.cos();
        let w_im = angle.sin();
        let mut i = 0;
        while i < n {
            let mut cur_re = 1.0;
            let mut cur_im = 0.0;
            for k in 0..half {
                let u_re = re[i + k];
                let u_im = im[i + k];
                let v_re = re[i + k + half] * cur_re - im[i + k + half] * cur_im;
                let v_im = re[i + k + half] * cur_im + im[i + k + half] * cur_re;
                re[i + k] = u_re + v_re;
                im[i + k] = u_im + v_im;
                re[i + k + half] = u_re - v_re;
                im[i + k + half] = u_im - v_im;
                let new_re = cur_re * w_re - cur_im * w_im;
                let new_im = cur_re * w_im + cur_im * w_re;
                cur_re = new_re;
                cur_im = new_im;
            }
            i += len;
        }
        len <<= 1;
    }
}

/// Compute magnitude spectrum from real signal. Returns magnitudes for bins 0..N/2+1.
pub fn magnitude_spectrum(signal: &[f64]) -> Vec<f64> {
    let n = signal.len().next_power_of_two();
    let mut re = vec![0.0; n];
    let mut im = vec![0.0; n];
    for (i, &s) in signal.iter().enumerate() {
        re[i] = s;
    }
    fft_inplace(&mut re, &mut im);
    let scale = 2.0 / signal.len() as f64;
    let mut mags = Vec::with_capacity(n / 2 + 1);
    for i in 0..=n / 2 {
        let mag = (re[i].powi(2) + im[i].powi(2)).sqrt() * scale;
        mags.push(mag);
    }
    // DC and Nyquist should not be doubled
    if !mags.is_empty() {
        mags[0] /= 2.0;
    }
    if mags.len() > 1 {
        let last = mags.len() - 1;
        mags[last] /= 2.0;
    }
    mags
}

/// Total Harmonic Distortion from a pressure waveform.
/// THD = sqrt(sum of H2^2 + H3^2 + ... + Hn^2) / H1.
pub fn thd(signal: &[f64], num_harmonics: usize) -> f64 {
    let mags = magnitude_spectrum(signal);
    if mags.len() < 2 {
        return 0.0;
    }
    // Find fundamental peak (skip DC bin 0)
    let mut fund_bin = 1;
    let mut fund_mag = 0.0_f64;
    for (i, &m) in mags.iter().enumerate().skip(1) {
        if m > fund_mag {
            fund_mag = m;
            fund_bin = i;
        }
    }
    if fund_mag < 1e-30 {
        return 0.0;
    }
    let mut sum_sq = 0.0;
    for h in 2..=num_harmonics {
        let bin = fund_bin * h;
        if bin < mags.len() {
            sum_sq += mags[bin].powi(2);
        }
    }
    sum_sq.sqrt() / fund_mag
}

/// Drive ratio from a pressure waveform and mean pressure.
/// Extracts fundamental amplitude via FFT.
pub fn drive_ratio_from_waveform(pressure_signal: &[f64], p_mean: f64) -> f64 {
    let mags = magnitude_spectrum(pressure_signal);
    // Fundamental is the largest non-DC bin
    let fund_mag = mags.iter().skip(1).cloned().fold(0.0_f64, f64::max);
    drive_ratio(fund_mag, p_mean)
}

/// Phase between pressure and velocity signals (radians) estimated via FFT
/// at the fundamental frequency.
pub fn phase_between_signals(signal_a: &[f64], signal_b: &[f64]) -> f64 {
    let n = signal_a.len().max(signal_b.len()).next_power_of_two();
    let mut re_a = vec![0.0; n];
    let mut im_a = vec![0.0; n];
    let mut re_b = vec![0.0; n];
    let mut im_b = vec![0.0; n];
    for (i, &s) in signal_a.iter().enumerate() {
        re_a[i] = s;
    }
    for (i, &s) in signal_b.iter().enumerate() {
        re_b[i] = s;
    }
    fft_inplace(&mut re_a, &mut im_a);
    fft_inplace(&mut re_b, &mut im_b);

    // Find fundamental (largest non-DC bin in signal_a)
    let mut fund_bin = 1;
    let mut fund_mag = 0.0_f64;
    for i in 1..n / 2 {
        let m = (re_a[i].powi(2) + im_a[i].powi(2)).sqrt();
        if m > fund_mag {
            fund_mag = m;
            fund_bin = i;
        }
    }

    let phase_a = im_a[fund_bin].atan2(re_a[fund_bin]);
    let phase_b = im_b[fund_bin].atan2(re_b[fund_bin]);
    let mut diff = phase_b - phase_a;
    // Wrap to [-pi, pi]
    while diff > PI {
        diff -= 2.0 * PI;
    }
    while diff < -PI {
        diff += 2.0 * PI;
    }
    diff
}

// ---------------------------------------------------------------------------
// Two-microphone method
// ---------------------------------------------------------------------------

/// Result of two-microphone decomposition.
#[derive(Debug, Clone, Copy)]
pub struct TwoMicResult {
    /// Forward (incident) wave pressure amplitude.
    pub p_forward_re: f64,
    pub p_forward_im: f64,
    /// Backward (reflected) wave pressure amplitude.
    pub p_backward_re: f64,
    pub p_backward_im: f64,
    /// Reflection coefficient magnitude |R|.
    pub reflection_coeff_mag: f64,
    /// Reflection coefficient phase (radians).
    pub reflection_coeff_phase: f64,
    /// Acoustic impedance magnitude |Z| at reference position (Pa*s/m).
    pub impedance_mag: f64,
    /// Acoustic impedance phase (radians).
    pub impedance_phase: f64,
}

/// Two-microphone wave decomposition.
///
/// Given complex pressure measurements p1, p2 at positions x1, x2 along the
/// duct, and the wavenumber k, decompose into forward and backward travelling
/// waves.
///
/// p_forward = (p1*e^{-jkx2} - p2*e^{-jkx1}) / (2j*sin(k*(x2-x1)))
/// p_backward = (p2*e^{jkx1} - p1*e^{jkx2}) / (2j*sin(k*(x2-x1)))
pub fn two_microphone_decomposition(
    p1_re: f64,
    p1_im: f64,
    p2_re: f64,
    p2_im: f64,
    x1: f64,
    x2: f64,
    k: f64,
    rho: f64,
    c: f64,
) -> TwoMicResult {
    // e^{-jkx} = cos(kx) - j*sin(kx)
    let e_neg_jkx1_re = (k * x1).cos();
    let e_neg_jkx1_im = -(k * x1).sin();
    let e_neg_jkx2_re = (k * x2).cos();
    let e_neg_jkx2_im = -(k * x2).sin();

    // e^{jkx} = cos(kx) + j*sin(kx)
    let e_pos_jkx1_re = (k * x1).cos();
    let e_pos_jkx1_im = (k * x1).sin();
    let e_pos_jkx2_re = (k * x2).cos();
    let e_pos_jkx2_im = (k * x2).sin();

    // Denominator: 2j*sin(k*(x2-x1))
    let sin_val = (k * (x2 - x1)).sin();
    let denom_re = 0.0; // 2j*sin => pure imaginary: (0, 2*sin)
    let denom_im = 2.0 * sin_val;

    // p1 * e^{-jkx2}
    let a_re = p1_re * e_neg_jkx2_re - p1_im * e_neg_jkx2_im;
    let a_im = p1_re * e_neg_jkx2_im + p1_im * e_neg_jkx2_re;

    // p2 * e^{-jkx1}
    let b_re = p2_re * e_neg_jkx1_re - p2_im * e_neg_jkx1_im;
    let b_im = p2_re * e_neg_jkx1_im + p2_im * e_neg_jkx1_re;

    // Numerator for p_forward: a - b
    let num_fwd_re = a_re - b_re;
    let num_fwd_im = a_im - b_im;

    // Complex division: (num) / (denom) where denom = (denom_re, denom_im)
    let denom_mag_sq = denom_re * denom_re + denom_im * denom_im;
    let (pf_re, pf_im) = if denom_mag_sq > 1e-60 {
        (
            (num_fwd_re * denom_re + num_fwd_im * denom_im) / denom_mag_sq,
            (num_fwd_im * denom_re - num_fwd_re * denom_im) / denom_mag_sq,
        )
    } else {
        (0.0, 0.0)
    };

    // Numerator for p_backward: p2*e^{jkx1} - p1*e^{jkx2}
    let c_re = p2_re * e_pos_jkx1_re - p2_im * e_pos_jkx1_im;
    let c_im = p2_re * e_pos_jkx1_im + p2_im * e_pos_jkx1_re;
    let d_re = p1_re * e_pos_jkx2_re - p1_im * e_pos_jkx2_im;
    let d_im = p1_re * e_pos_jkx2_im + p1_im * e_pos_jkx2_re;

    let num_bwd_re = c_re - d_re;
    let num_bwd_im = c_im - d_im;

    let (pb_re, pb_im) = if denom_mag_sq > 1e-60 {
        (
            (num_bwd_re * denom_re + num_bwd_im * denom_im) / denom_mag_sq,
            (num_bwd_im * denom_re - num_bwd_re * denom_im) / denom_mag_sq,
        )
    } else {
        (0.0, 0.0)
    };

    // Reflection coefficient R = p_backward / p_forward
    let pf_mag_sq = pf_re * pf_re + pf_im * pf_im;
    let (r_re, r_im) = if pf_mag_sq > 1e-60 {
        (
            (pb_re * pf_re + pb_im * pf_im) / pf_mag_sq,
            (pb_im * pf_re - pb_re * pf_im) / pf_mag_sq,
        )
    } else {
        (0.0, 0.0)
    };

    let r_mag = (r_re * r_re + r_im * r_im).sqrt();
    let r_phase = r_im.atan2(r_re);

    // Acoustic impedance Z = p / u at x = 0
    // p(0) = p_forward + p_backward
    // u(0) = (p_forward - p_backward) / (rho*c)
    // Z(0) = rho*c * (p_forward + p_backward) / (p_forward - p_backward)
    let p_sum_re = pf_re + pb_re;
    let p_sum_im = pf_im + pb_im;
    let p_diff_re = pf_re - pb_re;
    let p_diff_im = pf_im - pb_im;
    let pd_mag_sq = p_diff_re * p_diff_re + p_diff_im * p_diff_im;

    let (z_re, z_im) = if pd_mag_sq > 1e-60 {
        let ratio_re =
            (p_sum_re * p_diff_re + p_sum_im * p_diff_im) / pd_mag_sq;
        let ratio_im =
            (p_sum_im * p_diff_re - p_sum_re * p_diff_im) / pd_mag_sq;
        (rho * c * ratio_re, rho * c * ratio_im)
    } else {
        (f64::INFINITY, 0.0)
    };

    let z_mag = (z_re * z_re + z_im * z_im).sqrt();
    let z_phase = z_im.atan2(z_re);

    TwoMicResult {
        p_forward_re: pf_re,
        p_forward_im: pf_im,
        p_backward_re: pb_re,
        p_backward_im: pb_im,
        reflection_coeff_mag: r_mag,
        reflection_coeff_phase: r_phase,
        impedance_mag: z_mag,
        impedance_phase: z_phase,
    }
}

// ---------------------------------------------------------------------------
// Onset detection
// ---------------------------------------------------------------------------

/// Result of onset detection.
#[derive(Debug, Clone)]
pub struct OnsetResult {
    /// Index in the amplitude series where onset occurs.
    pub onset_index: Option<usize>,
    /// Critical temperature ratio T_hot / T_cold at onset.
    pub critical_temp_ratio: Option<f64>,
    /// Exponential growth rate (1/sample) once onset detected.
    pub growth_rate: Option<f64>,
}

/// Detect onset of thermoacoustic oscillation.
///
/// Monitors acoustic amplitude vs temperature difference. Onset is defined as
/// the point where amplitude exceeds `threshold` and shows sustained growth
/// over `sustain_count` consecutive samples.
///
/// `amplitudes` - time series of acoustic pressure amplitude.
/// `temp_ratios` - corresponding T_hot/T_cold ratios (same length).
/// `threshold` - amplitude threshold above noise floor.
/// `sustain_count` - number of consecutive growing samples to confirm onset.
pub fn detect_onset(
    amplitudes: &[f64],
    temp_ratios: &[f64],
    threshold: f64,
    sustain_count: usize,
) -> OnsetResult {
    if amplitudes.len() < sustain_count + 1 || amplitudes.len() != temp_ratios.len() {
        return OnsetResult {
            onset_index: None,
            critical_temp_ratio: None,
            growth_rate: None,
        };
    }

    let mut consecutive_growth = 0usize;
    for i in 1..amplitudes.len() {
        if amplitudes[i] > threshold && amplitudes[i] > amplitudes[i - 1] {
            consecutive_growth += 1;
            if consecutive_growth >= sustain_count {
                let onset_idx = i - sustain_count + 1;
                // Estimate exponential growth rate from onset region
                let start = onset_idx;
                let end = i;
                let growth_rate = if end > start && amplitudes[start] > 1e-30 {
                    let ratio = amplitudes[end] / amplitudes[start];
                    ratio.ln() / (end - start) as f64
                } else {
                    0.0
                };
                return OnsetResult {
                    onset_index: Some(onset_idx),
                    critical_temp_ratio: Some(temp_ratios[onset_idx]),
                    growth_rate: Some(growth_rate),
                };
            }
        } else {
            consecutive_growth = 0;
        }
    }

    OnsetResult {
        onset_index: None,
        critical_temp_ratio: None,
        growth_rate: None,
    }
}

// ---------------------------------------------------------------------------
// Streaming losses
// ---------------------------------------------------------------------------

/// Estimate Gedeon (DC) streaming mass flux in a looped (travelling-wave) engine.
///
/// For a looped tube, Gedeon streaming produces a net mass flow that carries
/// enthalpy from hot to cold, reducing efficiency. The streaming velocity is
/// approximately: U_DC ~ (|p1|^2) / (4 * rho * c * p_mean) for a simple torus.
///
/// Returns mass flux (kg/(m^2*s)).
pub fn gedeon_streaming_flux(p1_amplitude: f64, rho: f64, c: f64, p_mean: f64) -> f64 {
    // U_DC = |p1|^2 / (4 * rho * c * p_mean)
    // mass flux = rho * U_DC
    rho * p1_amplitude.powi(2) / (4.0 * rho * c * p_mean)
}

/// Estimate Rayleigh streaming velocity in a standing-wave resonator.
///
/// Rayleigh streaming is a second-order steady flow driven by boundary-layer
/// effects near walls. Typical magnitude:
/// u_streaming ~ (3/16) * |u1|^2 / (omega * R) for a cylindrical resonator
/// where R is the tube radius and u1 is the first-order velocity amplitude.
///
/// Returns streaming velocity (m/s).
pub fn rayleigh_streaming_velocity(u1_amplitude: f64, freq_hz: f64, tube_radius: f64) -> f64 {
    let omega = 2.0 * PI * freq_hz;
    if omega * tube_radius < 1e-30 {
        return 0.0;
    }
    (3.0 / 16.0) * u1_amplitude.powi(2) / (omega * tube_radius)
}

// ---------------------------------------------------------------------------
// Rott's linear theory: simplified stack work and losses
// ---------------------------------------------------------------------------

/// Simplified acoustic power production in the stack (Rott's theory).
///
/// For a parallel-plate stack in a standing-wave engine, the net acoustic
/// power production is approximately:
///
/// W_dot ~ -(delta_kappa * B * A * omega / (8 * gamma * p_mean))
///         * |p1|^2 * sin(2*k*x_s) * (dT_mean/dx - dT_crit/dx)
///
/// where B is the blockage ratio, x_s is the stack position, and dT_crit is
/// the critical temperature gradient for onset.
///
/// Positive result means the stack is producing acoustic power (engine mode).
/// Negative means absorbing (below onset or refrigerator).
pub fn stack_acoustic_power(
    gas: &GasProperties,
    resonator: &ResonatorConfig,
    stack: &StackConfig,
    freq_hz: f64,
    p1_amplitude: f64,
    dt_mean_dx: f64,
) -> f64 {
    let omega = 2.0 * PI * freq_hz;
    let k = omega / gas.speed_of_sound_ms;
    let delta_k = thermal_penetration_depth(gas, freq_hz);
    let b = stack.blockage_ratio();
    let area = resonator.cross_section_area();
    let x_s = stack.position_from_closed_end_m;

    // Critical temperature gradient: dT_crit/dx ~ (gamma - 1) * T_mean * k * tan(k*x_s) / (1 + Pr)
    // This is a linearized Rott estimate.
    let tan_kx = (k * x_s).tan();
    let dt_crit_dx = (gas.gamma - 1.0) * resonator.mean_temperature_k * k * tan_kx
        / (1.0 + gas.prandtl_number);

    let geometry_factor = (2.0 * k * x_s).sin();

    -(delta_k * b * area * omega / (8.0 * gas.gamma * resonator.mean_pressure_pa))
        * p1_amplitude.powi(2)
        * geometry_factor
        * (dt_mean_dx - dt_crit_dx)
}

/// Viscous losses in the stack (W).
///
/// W_visc ~ (delta_nu * B * A * omega * rho / 4) * |u1(x_s)|^2 * L_stack
pub fn stack_viscous_losses(
    gas: &GasProperties,
    resonator: &ResonatorConfig,
    stack: &StackConfig,
    freq_hz: f64,
    p1_amplitude: f64,
) -> f64 {
    let omega = 2.0 * PI * freq_hz;
    let k = omega / gas.speed_of_sound_ms;
    let delta_nu = viscous_penetration_depth(gas, freq_hz);
    let b = stack.blockage_ratio();
    let area = resonator.cross_section_area();

    let u1 = velocity_amplitude(
        p1_amplitude,
        k,
        stack.position_from_closed_end_m,
        gas.density_kgm3,
        gas.speed_of_sound_ms,
    );

    delta_nu * b * area * omega * gas.density_kgm3 / 4.0 * u1.powi(2) * stack.length_m
}

// ---------------------------------------------------------------------------
// ThermoacousticAnalyzer
// ---------------------------------------------------------------------------

/// Main analyzer combining gas, resonator, and stack for thermoacoustic
/// engine/refrigerator design and measurement.
#[derive(Debug, Clone)]
pub struct ThermoacousticAnalyzer {
    pub gas: GasProperties,
    pub resonator: ResonatorConfig,
    pub stack: StackConfig,
}

impl ThermoacousticAnalyzer {
    /// Create a new analyzer.
    pub fn new(gas: GasProperties, resonator: ResonatorConfig, stack: StackConfig) -> Self {
        Self {
            gas,
            resonator,
            stack,
        }
    }

    /// Resonant frequency for mode number n (n >= 1).
    pub fn resonant_frequency(&self, n: u32) -> f64 {
        let c = self.gas.speed_of_sound_ms;
        let l = self.resonator.length_m;
        match self.resonator.resonator_type {
            ResonatorType::ClosedClosed => n as f64 * c / (2.0 * l),
            ResonatorType::ClosedOpen => n as f64 * c / (4.0 * l),
        }
    }

    /// Wavenumber k = omega / c for mode n.
    pub fn wavenumber(&self, n: u32) -> f64 {
        2.0 * PI * self.resonant_frequency(n) / self.gas.speed_of_sound_ms
    }

    /// Thermal penetration depth at mode n.
    pub fn thermal_depth(&self, n: u32) -> f64 {
        thermal_penetration_depth(&self.gas, self.resonant_frequency(n))
    }

    /// Viscous penetration depth at mode n.
    pub fn viscous_depth(&self, n: u32) -> f64 {
        viscous_penetration_depth(&self.gas, self.resonant_frequency(n))
    }

    /// Plate spacing ratio y0/delta_kappa at mode n.
    pub fn spacing_ratio(&self, n: u32) -> f64 {
        spacing_ratio(&self.stack, &self.gas, self.resonant_frequency(n))
    }

    /// Evaluate engine performance given temperatures and operating conditions.
    ///
    /// `p1_amplitude` - pressure amplitude at the pressure antinode (Pa).
    /// `t_hot_k` - hot heat exchanger temperature (K).
    /// `t_cold_k` - cold heat exchanger temperature (K).
    pub fn evaluate_engine(
        &self,
        p1_amplitude: f64,
        t_hot_k: f64,
        t_cold_k: f64,
    ) -> PerformanceMetrics {
        let freq = self.resonant_frequency(1);
        let dt_mean_dx = (t_hot_k - t_cold_k) / self.stack.length_m;

        let w_stack = stack_acoustic_power(
            &self.gas,
            &self.resonator,
            &self.stack,
            freq,
            p1_amplitude,
            dt_mean_dx,
        );
        let w_visc = stack_viscous_losses(
            &self.gas,
            &self.resonator,
            &self.stack,
            freq,
            p1_amplitude,
        );

        let w_net = w_stack - w_visc;

        let eta_carnot = carnot_efficiency(t_hot_k, t_cold_k);

        // Approximate Q_hot for efficiency calculation
        // Q_hot ~ W_net / eta (use ~30% of Carnot as typical)
        // For a more physical estimate: Q_hot = W_net + Q_cold
        // Here we compute a simplified version:
        let q_hot = if w_net > 0.0 { w_net / (0.3 * eta_carnot).max(1e-10) } else { 0.0 };
        let q_cold = q_hot - w_net;

        let actual_eff = engine_efficiency(w_net, q_hot);

        PerformanceMetrics {
            acoustic_power_w: w_net,
            q_hot_w: q_hot,
            q_cold_w: q_cold,
            actual_performance: actual_eff,
            carnot_limit: eta_carnot,
            relative_perf: relative_performance(actual_eff, eta_carnot),
        }
    }

    /// Evaluate refrigerator performance.
    pub fn evaluate_refrigerator(
        &self,
        p1_amplitude: f64,
        t_hot_k: f64,
        t_cold_k: f64,
    ) -> PerformanceMetrics {
        let freq = self.resonant_frequency(1);
        // In refrigerator mode, acoustic power drives heat from cold to hot
        let w_visc = stack_viscous_losses(
            &self.gas,
            &self.resonator,
            &self.stack,
            freq,
            p1_amplitude,
        );

        // Input acoustic power
        let area = self.resonator.cross_section_area();
        let w_input = acoustic_power_simple(
            p1_amplitude,
            self.gas.density_kgm3,
            self.gas.speed_of_sound_ms,
            area,
        );

        let cop_carnot = carnot_cop(t_hot_k, t_cold_k);
        let q_cold = (w_input - w_visc) * cop_carnot * 0.3; // ~30% of Carnot COP
        let q_hot = q_cold + w_input;

        let actual_cop = refrigerator_cop(q_cold, w_input);

        PerformanceMetrics {
            acoustic_power_w: w_input,
            q_hot_w: q_hot,
            q_cold_w: q_cold,
            actual_performance: actual_cop,
            carnot_limit: cop_carnot,
            relative_perf: relative_performance(actual_cop, cop_carnot),
        }
    }

    /// Analyze a measured pressure waveform for harmonic content and drive ratio.
    pub fn analyze_waveform(&self, pressure_signal: &[f64]) -> WaveformAnalysis {
        let mags = magnitude_spectrum(pressure_signal);
        let fund_mag = mags.iter().skip(1).cloned().fold(0.0_f64, f64::max);
        let dr = drive_ratio(fund_mag, self.resonator.mean_pressure_pa);
        let thd_val = thd(pressure_signal, 10);

        WaveformAnalysis {
            fundamental_amplitude: fund_mag,
            drive_ratio: dr,
            thd: thd_val,
            harmonic_magnitudes: mags,
        }
    }
}

/// Analysis results for a measured pressure waveform.
#[derive(Debug, Clone)]
pub struct WaveformAnalysis {
    /// Fundamental pressure amplitude (Pa).
    pub fundamental_amplitude: f64,
    /// Drive ratio |p1|/p_mean.
    pub drive_ratio: f64,
    /// Total harmonic distortion (fraction, not percent).
    pub thd: f64,
    /// FFT magnitude spectrum bins.
    pub harmonic_magnitudes: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -----------------------------------------------------------------------
    // GasProperties
    // -----------------------------------------------------------------------

    #[test]
    fn test_air_preset() {
        let air = GasProperties::air();
        assert!(approx_eq(air.gamma, 1.4, 0.01));
        assert!(approx_eq(air.speed_of_sound_ms, 343.0, 1.0));
        assert!(air.density_kgm3 > 1.0 && air.density_kgm3 < 1.3);
        assert!(air.prandtl_number > 0.7 && air.prandtl_number < 0.72);
    }

    #[test]
    fn test_helium_preset() {
        let he = GasProperties::helium();
        assert!(approx_eq(he.gamma, 1.667, 0.01));
        assert!(approx_eq(he.speed_of_sound_ms, 1020.0, 5.0));
        assert!(he.density_kgm3 < 0.2);
    }

    #[test]
    fn test_argon_preset() {
        let ar = GasProperties::argon();
        assert!(approx_eq(ar.gamma, 1.667, 0.01));
        assert!(approx_eq(ar.speed_of_sound_ms, 323.0, 2.0));
        assert!(ar.density_kgm3 > 1.5);
    }

    #[test]
    fn test_kinematic_viscosity() {
        let air = GasProperties::air();
        let nu = air.kinematic_viscosity();
        // nu ~ 1.846e-5 / 1.177 ~ 1.568e-5 m^2/s
        assert!(nu > 1.5e-5 && nu < 1.7e-5);
    }

    #[test]
    fn test_thermal_diffusivity() {
        let air = GasProperties::air();
        let kappa = air.thermal_diffusivity();
        // kappa = 0.0263 / (1.177 * 1005) ~ 2.22e-5 m^2/s
        assert!(kappa > 2.0e-5 && kappa < 2.5e-5);
    }

    #[test]
    fn test_prandtl_consistency() {
        // Pr = nu / kappa should match the stored value approximately
        let air = GasProperties::air();
        let pr_computed = air.kinematic_viscosity() / air.thermal_diffusivity();
        assert!(approx_eq(pr_computed, air.prandtl_number, 0.05));
    }

    // -----------------------------------------------------------------------
    // Resonator
    // -----------------------------------------------------------------------

    #[test]
    fn test_cross_section_area() {
        let res = ResonatorConfig {
            length_m: 1.0,
            diameter_m: 0.1,
            mean_pressure_pa: ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedClosed,
        };
        let area = res.cross_section_area();
        let expected = PI * 0.05_f64.powi(2);
        assert!(approx_eq(area, expected, 1e-10));
    }

    #[test]
    fn test_resonant_freq_closed_closed() {
        let gas = GasProperties::air();
        let res = ResonatorConfig {
            length_m: 1.0,
            diameter_m: 0.05,
            mean_pressure_pa: ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedClosed,
        };
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        let analyzer = ThermoacousticAnalyzer::new(gas, res, stack);
        // f1 = 1*343/(2*1) = 171.5 Hz
        let f1 = analyzer.resonant_frequency(1);
        assert!(approx_eq(f1, 171.5, 0.5));
        // f2 = 343 Hz
        let f2 = analyzer.resonant_frequency(2);
        assert!(approx_eq(f2, 343.0, 0.5));
    }

    #[test]
    fn test_resonant_freq_closed_open() {
        let gas = GasProperties::air();
        let res = ResonatorConfig {
            length_m: 1.0,
            diameter_m: 0.05,
            mean_pressure_pa: ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedOpen,
        };
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        let analyzer = ThermoacousticAnalyzer::new(gas, res, stack);
        // f1 = 1*343/(4*1) = 85.75 Hz
        let f1 = analyzer.resonant_frequency(1);
        assert!(approx_eq(f1, 85.75, 0.5));
    }

    // -----------------------------------------------------------------------
    // Stack
    // -----------------------------------------------------------------------

    #[test]
    fn test_blockage_ratio() {
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        // BR = 0.4 / (0.4 + 0.1) = 0.8
        assert!(approx_eq(stack.blockage_ratio(), 0.8, 1e-10));
    }

    #[test]
    fn test_hydraulic_radius() {
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        assert!(approx_eq(stack.hydraulic_radius(), 0.2e-3, 1e-10));
    }

    // -----------------------------------------------------------------------
    // Penetration depths
    // -----------------------------------------------------------------------

    #[test]
    fn test_thermal_penetration_depth_air() {
        let air = GasProperties::air();
        let delta_k = thermal_penetration_depth(&air, 171.5);
        // delta_k = sqrt(2 * kappa / omega), kappa ~ 2.22e-5
        // omega = 2*pi*171.5 ~ 1078
        // delta_k = sqrt(2 * 2.22e-5 / 1078) ~ sqrt(4.12e-8) ~ 2.03e-4 m
        assert!(delta_k > 1.5e-4 && delta_k < 2.5e-4);
    }

    #[test]
    fn test_viscous_penetration_depth_air() {
        let air = GasProperties::air();
        let delta_nu = viscous_penetration_depth(&air, 171.5);
        // delta_nu = sqrt(2 * nu / omega), nu ~ 1.568e-5
        // delta_nu = sqrt(2 * 1.568e-5 / 1078) ~ sqrt(2.91e-8) ~ 1.70e-4 m
        assert!(delta_nu > 1.3e-4 && delta_nu < 2.0e-4);
    }

    #[test]
    fn test_spacing_ratio() {
        let air = GasProperties::air();
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        let ratio = spacing_ratio(&stack, &air, 171.5);
        // y0 = 0.2e-3, delta_k ~ 2.03e-4
        // ratio ~ 0.2e-3 / 2.03e-4 ~ 0.99
        assert!(ratio > 0.5 && ratio < 2.0);
    }

    #[test]
    fn test_helium_penetration_depth_smaller() {
        // Helium has higher kappa -> larger thermal penetration depth for same freq
        let air = GasProperties::air();
        let he = GasProperties::helium();
        let freq = 500.0;
        let dk_air = thermal_penetration_depth(&air, freq);
        let dk_he = thermal_penetration_depth(&he, freq);
        // Helium kappa = 0.152 / (0.164*5193) ~ 1.78e-4, much larger than air's ~2.22e-5
        assert!(dk_he > dk_air);
    }

    // -----------------------------------------------------------------------
    // Acoustic field
    // -----------------------------------------------------------------------

    #[test]
    fn test_pressure_amplitude_at_closed_end() {
        // At x=0 (closed end), cos(0)=1, so p1 = P_A
        let p = pressure_amplitude(1000.0, 1.0, 0.0);
        assert!(approx_eq(p, 1000.0, TOL));
    }

    #[test]
    fn test_pressure_amplitude_at_quarter_wave() {
        // At x = lambda/4, k*x = pi/2, cos(pi/2) = 0
        let k = 2.0 * PI; // lambda = 1 m
        let x = 0.25; // quarter wavelength
        let p = pressure_amplitude(1000.0, k, x);
        assert!(p.abs() < 1e-10);
    }

    #[test]
    fn test_velocity_amplitude_at_closed_end() {
        // At x=0, sin(0)=0, so u1 = 0
        let u = velocity_amplitude(1000.0, 1.0, 0.0, 1.2, 343.0);
        assert!(u.abs() < TOL);
    }

    #[test]
    fn test_velocity_amplitude_at_quarter_wave() {
        // At x = lambda/4, sin(pi/2) = 1
        let k = 2.0 * PI;
        let x = 0.25;
        let rho = 1.2;
        let c = 343.0;
        let p_a = 1000.0;
        let u = velocity_amplitude(p_a, k, x, rho, c);
        let expected = p_a / (rho * c);
        assert!(approx_eq(u, expected, 1e-6));
    }

    #[test]
    fn test_drive_ratio() {
        let dr = drive_ratio(1013.25, ATM_PA);
        assert!(approx_eq(dr, 0.01, 1e-6));
    }

    #[test]
    fn test_acoustic_power_simple() {
        let p_a = 1000.0;
        let rho = 1.2;
        let c = 343.0;
        let area = PI * 0.025_f64.powi(2);
        let w = acoustic_power_simple(p_a, rho, c, area);
        // W = 0.5 * 1e6 / (1.2*343) * pi*6.25e-4 = 0.5 * 2430 * 1.96e-3 ~ 2.38 W
        assert!(w > 1.0 && w < 5.0);
    }

    // -----------------------------------------------------------------------
    // Performance metrics
    // -----------------------------------------------------------------------

    #[test]
    fn test_carnot_efficiency() {
        let eta = carnot_efficiency(600.0, 300.0);
        assert!(approx_eq(eta, 0.5, TOL));
    }

    #[test]
    fn test_carnot_cop() {
        let cop = carnot_cop(350.0, 250.0);
        // COP = 250/(350-250) = 2.5
        assert!(approx_eq(cop, 2.5, TOL));
    }

    #[test]
    fn test_engine_efficiency() {
        let eff = engine_efficiency(100.0, 500.0);
        assert!(approx_eq(eff, 0.2, TOL));
    }

    #[test]
    fn test_refrigerator_cop_calc() {
        let cop = refrigerator_cop(250.0, 100.0);
        assert!(approx_eq(cop, 2.5, TOL));
    }

    #[test]
    fn test_relative_performance() {
        let rel = relative_performance(0.15, 0.5);
        assert!(approx_eq(rel, 0.3, TOL));
    }

    #[test]
    fn test_zero_division_protection() {
        assert!(approx_eq(engine_efficiency(100.0, 0.0), 0.0, TOL));
        assert!(approx_eq(refrigerator_cop(100.0, 0.0), 0.0, TOL));
        assert!(approx_eq(relative_performance(0.5, 0.0), 0.0, TOL));
    }

    // -----------------------------------------------------------------------
    // FFT and signal processing
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_dc() {
        let mut re = vec![1.0; 8];
        let mut im = vec![0.0; 8];
        fft_inplace(&mut re, &mut im);
        // DC bin should be 8.0, all others 0
        assert!(approx_eq(re[0], 8.0, TOL));
        for i in 1..8 {
            assert!(re[i].abs() < TOL);
            assert!(im[i].abs() < TOL);
        }
    }

    #[test]
    fn test_fft_sine() {
        let n = 64;
        let freq_bin = 4; // 4 cycles in 64 samples
        let mut re: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq_bin as f64 * i as f64 / n as f64).sin())
            .collect();
        let mut im = vec![0.0; n];
        fft_inplace(&mut re, &mut im);
        // Bin 4 and bin 60 (N-4) should have magnitude N/2 = 32
        let mag4 = (re[4].powi(2) + im[4].powi(2)).sqrt();
        assert!(approx_eq(mag4, 32.0, 0.5));
    }

    #[test]
    fn test_magnitude_spectrum_pure_tone() {
        let n = 256;
        let freq_bin = 10;
        let amplitude = 3.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| amplitude * (2.0 * PI * freq_bin as f64 * i as f64 / n as f64).sin())
            .collect();
        let mags = magnitude_spectrum(&signal);
        // Fundamental bin should be near 3.0
        assert!(approx_eq(mags[freq_bin], amplitude, 0.1));
        // Other bins (not DC or harmonic) should be small
        assert!(mags[freq_bin + 3] < 0.1);
    }

    #[test]
    fn test_thd_pure_sine() {
        let n = 1024;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 10.0 * i as f64 / n as f64).sin())
            .collect();
        let distortion = thd(&signal, 5);
        // Pure sine should have near-zero THD
        assert!(distortion < 0.01);
    }

    #[test]
    fn test_thd_with_harmonics() {
        let n = 1024;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                (2.0 * PI * 10.0 * t).sin() + 0.1 * (2.0 * PI * 20.0 * t).sin()
            })
            .collect();
        let distortion = thd(&signal, 5);
        // THD ~ 0.1 (10% second harmonic)
        assert!(distortion > 0.05 && distortion < 0.15);
    }

    #[test]
    fn test_drive_ratio_from_waveform() {
        let n = 1024;
        let p_mean = ATM_PA;
        let p_amplitude = 1000.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| p_mean + p_amplitude * (2.0 * PI * 8.0 * i as f64 / n as f64).sin())
            .collect();
        let dr = drive_ratio_from_waveform(&signal, p_mean);
        // DR should be ~ 1000/101325 ~ 0.00987
        assert!(dr > 0.008 && dr < 0.012);
    }

    #[test]
    fn test_phase_between_in_phase_signals() {
        let n = 256;
        let a: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 5.0 * i as f64 / n as f64).sin())
            .collect();
        let b = a.clone();
        let phase = phase_between_signals(&a, &b);
        assert!(phase.abs() < 0.1);
    }

    #[test]
    fn test_phase_between_quadrature_signals() {
        let n = 256;
        let a: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 5.0 * i as f64 / n as f64).sin())
            .collect();
        let b: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 5.0 * i as f64 / n as f64).cos())
            .collect();
        let phase = phase_between_signals(&a, &b);
        // cos leads sin by pi/2
        assert!((phase - (-PI / 2.0)).abs() < 0.2 || (phase - (3.0 * PI / 2.0)).abs() < 0.2);
    }

    // -----------------------------------------------------------------------
    // Two-microphone method
    // -----------------------------------------------------------------------

    #[test]
    fn test_two_mic_pure_forward_wave() {
        // Forward wave only: p(x) = P_A * exp(-jkx)
        // At x1 and x2: p1 = P_A * exp(-jkx1), p2 = P_A * exp(-jkx2)
        let p_a: f64 = 100.0;
        let k: f64 = 2.0;
        let x1: f64 = 0.1;
        let x2: f64 = 0.3;

        let p1_re = p_a * (k * x1).cos();
        let p1_im = -p_a * (k * x1).sin();
        let p2_re = p_a * (k * x2).cos();
        let p2_im = -p_a * (k * x2).sin();

        let result = two_microphone_decomposition(
            p1_re, p1_im, p2_re, p2_im, x1, x2, k, 1.2, 343.0,
        );

        // Forward wave magnitude should be close to P_A
        let pf_mag = (result.p_forward_re.powi(2) + result.p_forward_im.powi(2)).sqrt();
        assert!(
            approx_eq(pf_mag, p_a, 1.0),
            "Forward magnitude {pf_mag} should be near {p_a}"
        );

        // Reflection coefficient should be near zero
        assert!(
            result.reflection_coeff_mag < 0.1,
            "R = {} should be near 0",
            result.reflection_coeff_mag
        );
    }

    #[test]
    fn test_two_mic_impedance_plane_wave() {
        // For a pure forward wave, Z = rho*c
        let p_a: f64 = 100.0;
        let k: f64 = 2.0;
        let x1: f64 = 0.1;
        let x2: f64 = 0.3;
        let rho: f64 = 1.2;
        let c: f64 = 343.0;

        let p1_re = p_a * (k * x1).cos();
        let p1_im = -p_a * (k * x1).sin();
        let p2_re = p_a * (k * x2).cos();
        let p2_im = -p_a * (k * x2).sin();

        let result = two_microphone_decomposition(
            p1_re, p1_im, p2_re, p2_im, x1, x2, k, rho, c,
        );

        let z_expected = rho * c; // 411.6
        assert!(
            (result.impedance_mag - z_expected).abs() / z_expected < 0.1,
            "|Z| = {} should be near rho*c = {}",
            result.impedance_mag,
            z_expected
        );
    }

    // -----------------------------------------------------------------------
    // Onset detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_onset_detection_growing_signal() {
        let n = 50;
        let amplitudes: Vec<f64> = (0..n).map(|i| 0.01 * (0.1 * i as f64).exp()).collect();
        let temp_ratios: Vec<f64> = (0..n).map(|i| 1.0 + 0.02 * i as f64).collect();
        let result = detect_onset(&amplitudes, &temp_ratios, 0.02, 3);
        assert!(result.onset_index.is_some());
        assert!(result.growth_rate.unwrap() > 0.0);
    }

    #[test]
    fn test_onset_detection_no_onset() {
        let amplitudes = vec![0.001; 20];
        let temp_ratios = vec![1.5; 20];
        let result = detect_onset(&amplitudes, &temp_ratios, 0.01, 3);
        assert!(result.onset_index.is_none());
    }

    #[test]
    fn test_onset_detection_critical_ratio() {
        let n = 30;
        // Below onset for first 15, then exponential growth
        let mut amplitudes = vec![0.005; n];
        let mut temp_ratios = vec![1.0; n];
        for i in 15..n {
            amplitudes[i] = 0.005 * (0.3 * (i - 15) as f64).exp();
            temp_ratios[i] = 1.0 + 0.05 * i as f64;
        }
        let result = detect_onset(&amplitudes, &temp_ratios, 0.006, 3);
        assert!(result.onset_index.is_some());
        let idx = result.onset_index.unwrap();
        assert!(idx >= 15 && idx < 25);
        assert!(result.critical_temp_ratio.unwrap() > 1.0);
    }

    // -----------------------------------------------------------------------
    // Streaming losses
    // -----------------------------------------------------------------------

    #[test]
    fn test_gedeon_streaming_positive() {
        let flux = gedeon_streaming_flux(1000.0, 1.2, 343.0, ATM_PA);
        assert!(flux > 0.0);
        // Should be small: ~1e6 / (4*1.2*343*101325) ~ 0.006 kg/(m^2*s)
        assert!(flux < 0.1);
    }

    #[test]
    fn test_rayleigh_streaming() {
        let u1 = 1.0; // 1 m/s velocity amplitude
        let freq = 171.5;
        let radius = 0.025;
        let u_s = rayleigh_streaming_velocity(u1, freq, radius);
        // ~(3/16)*1^2 / (2*pi*171.5*0.025) ~ 0.1875 / 26.95 ~ 0.007 m/s
        assert!(u_s > 0.0 && u_s < 0.05);
    }

    // -----------------------------------------------------------------------
    // Rott's theory: stack power and losses
    // -----------------------------------------------------------------------

    #[test]
    fn test_stack_viscous_losses_positive() {
        let gas = GasProperties::air();
        let res = ResonatorConfig {
            length_m: 1.0,
            diameter_m: 0.05,
            mean_pressure_pa: ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedClosed,
        };
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        let losses = stack_viscous_losses(&gas, &res, &stack, 171.5, 1000.0);
        assert!(losses > 0.0, "Viscous losses must be positive");
    }

    // -----------------------------------------------------------------------
    // Analyzer integration
    // -----------------------------------------------------------------------

    #[test]
    fn test_analyzer_evaluate_engine() {
        let gas = GasProperties::air();
        let res = ResonatorConfig {
            length_m: 1.0,
            diameter_m: 0.05,
            mean_pressure_pa: ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedClosed,
        };
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        let analyzer = ThermoacousticAnalyzer::new(gas, res, stack);
        let perf = analyzer.evaluate_engine(3000.0, 600.0, 300.0);
        // Carnot efficiency = 0.5
        assert!(approx_eq(perf.carnot_limit, 0.5, 0.01));
        // Relative performance should be <= 1
        assert!(perf.relative_perf <= 1.01);
    }

    #[test]
    fn test_analyzer_evaluate_refrigerator() {
        let gas = GasProperties::helium();
        let res = ResonatorConfig {
            length_m: 0.5,
            diameter_m: 0.03,
            mean_pressure_pa: 10.0 * ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedClosed,
        };
        let stack = StackConfig {
            plate_spacing_m: 0.3e-3,
            plate_thickness_m: 0.05e-3,
            length_m: 0.04,
            position_from_closed_end_m: 0.1,
        };
        let analyzer = ThermoacousticAnalyzer::new(gas, res, stack);
        let perf = analyzer.evaluate_refrigerator(5000.0, 350.0, 250.0);
        // Carnot COP = 250/(350-250) = 2.5
        assert!(approx_eq(perf.carnot_limit, 2.5, 0.01));
        assert!(perf.q_cold_w >= 0.0);
    }

    #[test]
    fn test_analyzer_wavenumber() {
        let gas = GasProperties::air();
        let res = ResonatorConfig {
            length_m: 1.0,
            diameter_m: 0.05,
            mean_pressure_pa: ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedClosed,
        };
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        let analyzer = ThermoacousticAnalyzer::new(gas, res, stack);
        let k = analyzer.wavenumber(1);
        // k = 2*pi*f / c = 2*pi*171.5 / 343 = pi
        assert!(approx_eq(k, PI, 0.01));
    }

    #[test]
    fn test_analyzer_thermal_and_viscous_depth() {
        let gas = GasProperties::air();
        let res = ResonatorConfig {
            length_m: 1.0,
            diameter_m: 0.05,
            mean_pressure_pa: ATM_PA,
            mean_temperature_k: 300.0,
            resonator_type: ResonatorType::ClosedClosed,
        };
        let stack = StackConfig {
            plate_spacing_m: 0.4e-3,
            plate_thickness_m: 0.1e-3,
            length_m: 0.05,
            position_from_closed_end_m: 0.15,
        };
        let analyzer = ThermoacousticAnalyzer::new(gas, res, stack);
        let dk = analyzer.thermal_depth(1);
        let dv = analyzer.viscous_depth(1);
        // Both should be on order of 0.1-0.3 mm
        assert!(dk > 1e-4 && dk < 1e-3);
        assert!(dv > 1e-4 && dv < 1e-3);
        // For air, Pr < 1, so delta_nu < delta_kappa
        assert!(dv < dk);
    }

    #[test]
    fn test_acoustic_power_travelling_net() {
        let rho = 1.2;
        let c = 343.0;
        let area = PI * 0.025_f64.powi(2);
        // Forward = 100 Pa, backward = 50 Pa
        let w = acoustic_power_travelling(100.0, 50.0, rho, c, area);
        // W = 0.5 * (10000 - 2500) / (411.6) * area
        assert!(w > 0.0);
        // Forward > backward => net positive power
        let w_rev = acoustic_power_travelling(50.0, 100.0, rho, c, area);
        assert!(w_rev < 0.0);
    }
}

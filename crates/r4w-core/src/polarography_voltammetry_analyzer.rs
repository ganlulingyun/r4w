//! # Polarography & Voltammetry Analyzer
//!
//! Implements polarographic and voltammetric analysis techniques for electrochemistry.
//! Supports Linear Sweep Voltammetry (LSV), Cyclic Voltammetry (CV), Differential Pulse
//! Voltammetry (DPV), Square Wave Voltammetry (SWV), DC Polarography, Tafel analysis,
//! Butler-Volmer kinetics, Randles-Sevcik, Cottrell, Levich, and Koutecky-Levich equations.
//!
//! ## Physics
//!
//! Voltammetry measures current as a function of applied potential. The peak current,
//! half-wave potential, and peak shape encode information about analyte concentration,
//! diffusion coefficient, electron transfer kinetics, and reaction mechanism.
//!
//! - Randles-Sevcik: ip = 2.69e5 * n^(3/2) * A * D^(1/2) * C * v^(1/2)
//! - Cottrell: i(t) = nFAD^(1/2)C / (pi*t)^(1/2)
//! - Ilkovic: id = 607 * n * D^(1/2) * m^(2/3) * t^(1/6) * C
//! - Butler-Volmer: i = i0 * [exp(alpha*n*F*eta/(RT)) - exp(-(1-alpha)*n*F*eta/(RT))]

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Faraday constant (C/mol).
const F_CONST: f64 = 96485.33212;
/// Gas constant (J/(mol·K)).
const R_CONST: f64 = 8.314462;
/// Standard temperature (K).
const T_STD: f64 = 298.15;
/// Thermal voltage at 298.15 K: RT/F.
const VT: f64 = R_CONST * T_STD / F_CONST;

// ---------------------------------------------------------------------------
// VoltammetricSweep
// ---------------------------------------------------------------------------

/// Stores potential (V) and current (A) arrays for a voltammogram.
#[derive(Debug, Clone)]
pub struct VoltammetricSweep {
    /// Applied potential in Volts.
    pub potential: Vec<f64>,
    /// Measured current in Amperes.
    pub current: Vec<f64>,
}

impl VoltammetricSweep {
    /// Create a new sweep from equal-length vectors.
    pub fn new(potential: Vec<f64>, current: Vec<f64>) -> Self {
        assert_eq!(potential.len(), current.len());
        Self { potential, current }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.potential.len()
    }

    /// Whether the sweep is empty.
    pub fn is_empty(&self) -> bool {
        self.potential.is_empty()
    }
}

// ---------------------------------------------------------------------------
// PeakResult
// ---------------------------------------------------------------------------

/// Result of peak analysis on a voltammogram.
#[derive(Debug, Clone)]
pub struct PeakResult {
    /// Peak potential (V).
    pub peak_potential: f64,
    /// Peak current (A).
    pub peak_current: f64,
    /// Half-peak width (V), if determinable.
    pub half_width: Option<f64>,
    /// Index in the sweep array.
    pub index: usize,
}

// ---------------------------------------------------------------------------
// HalfWavePotential
// ---------------------------------------------------------------------------

/// Result of half-wave potential analysis.
#[derive(Debug, Clone)]
pub struct HalfWavePotential {
    /// E½ in Volts.
    pub e_half: f64,
    /// Limiting diffusion current (A).
    pub i_d: f64,
    /// Number of electrons from log-plot slope.
    pub n_electrons: f64,
}

// ---------------------------------------------------------------------------
// DiffusionCoefficient
// ---------------------------------------------------------------------------

/// Result of diffusion coefficient analysis.
#[derive(Debug, Clone)]
pub struct DiffusionCoefficient {
    /// Diffusion coefficient in cm²/s.
    pub d_cm2_s: f64,
    /// Method used for determination.
    pub method: String,
}

// ---------------------------------------------------------------------------
// TafelResult
// ---------------------------------------------------------------------------

/// Result of Tafel analysis.
#[derive(Debug, Clone)]
pub struct TafelResult {
    /// Tafel slope (V/decade).
    pub slope: f64,
    /// Exchange current density (A/cm²).
    pub i0: f64,
    /// Transfer coefficient alpha.
    pub alpha: f64,
    /// Number of electrons (assumed or fitted).
    pub n: f64,
    /// R² of the linear fit.
    pub r_squared: f64,
}

// ---------------------------------------------------------------------------
// CvResult
// ---------------------------------------------------------------------------

/// Result of cyclic voltammetry analysis.
#[derive(Debug, Clone)]
pub struct CvResult {
    /// Anodic peak potential (V).
    pub epa: f64,
    /// Cathodic peak potential (V).
    pub epc: f64,
    /// Anodic peak current (A).
    pub ipa: f64,
    /// Cathodic peak current (A).
    pub ipc: f64,
    /// Peak separation ΔEp = Epa - Epc (V).
    pub delta_ep: f64,
    /// Formal potential E°' = (Epa + Epc) / 2 (V).
    pub e_formal: f64,
    /// Peak current ratio |ipa/ipc|.
    pub current_ratio: f64,
    /// Whether reaction appears reversible (ΔEp ≈ 59/n mV).
    pub reversible: bool,
}

// ---------------------------------------------------------------------------
// BaselineMethod
// ---------------------------------------------------------------------------

/// Baseline correction method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BaselineMethod {
    /// Linear baseline between first and last points.
    Linear,
    /// Moving minimum subtraction.
    MovingMinimum(usize),
    /// Polynomial fit of given degree.
    Polynomial(usize),
}

// ---------------------------------------------------------------------------
// PolarographyProcessor
// ---------------------------------------------------------------------------

/// Configuration for the processor.
#[derive(Debug, Clone)]
pub struct PolarographyConfig {
    /// Temperature in Kelvin.
    pub temperature_k: f64,
    /// Electrode area in cm².
    pub electrode_area_cm2: f64,
    /// Number of electrons for the reaction.
    pub n_electrons: u32,
    /// Concentration in mol/cm³.
    pub concentration_mol_cm3: f64,
}

impl Default for PolarographyConfig {
    fn default() -> Self {
        Self {
            temperature_k: T_STD,
            electrode_area_cm2: 0.0707, // 3mm diameter disk
            n_electrons: 1,
            concentration_mol_cm3: 1.0e-6,
        }
    }
}

/// Polarography and voltammetry processor.
#[derive(Debug, Clone)]
pub struct PolarographyProcessor {
    /// Configuration.
    pub config: PolarographyConfig,
}

impl PolarographyProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: PolarographyConfig) -> Self {
        Self { config }
    }

    /// Create with default configuration.
    pub fn default_processor() -> Self {
        Self::new(PolarographyConfig::default())
    }

    /// Analyze a linear sweep voltammogram.
    pub fn analyze_lsv(&self, sweep: &VoltammetricSweep) -> Option<PeakResult> {
        linear_sweep_analyze(sweep)
    }

    /// Analyze cyclic voltammetry data.
    pub fn analyze_cv(
        &self,
        forward: &VoltammetricSweep,
        reverse: &VoltammetricSweep,
    ) -> Option<CvResult> {
        cyclic_voltammetry_analyze(forward, reverse, self.config.n_electrons)
    }

    /// Calculate diffusion coefficient from peak current and scan rate.
    pub fn diffusion_from_randles_sevcik(&self, ip: f64, scan_rate: f64) -> DiffusionCoefficient {
        let d: f64 = randles_sevcik(
            ip,
            self.config.n_electrons,
            self.config.electrode_area_cm2,
            self.config.concentration_mol_cm3,
            scan_rate,
        );
        DiffusionCoefficient {
            d_cm2_s: d,
            method: "Randles-Sevcik".into(),
        }
    }
}

// ---------------------------------------------------------------------------
// Linear Sweep Voltammetry
// ---------------------------------------------------------------------------

/// Find peak in a linear sweep voltammogram using first derivative zero-crossing.
pub fn linear_sweep_analyze(sweep: &VoltammetricSweep) -> Option<PeakResult> {
    if sweep.len() < 3 {
        return None;
    }
    // Find the index of maximum absolute current
    let mut best_idx: usize = 0;
    let mut best_abs: f64 = 0.0;
    for (i, &c) in sweep.current.iter().enumerate() {
        let ac: f64 = c.abs();
        if ac > best_abs {
            best_abs = ac;
            best_idx = i;
        }
    }

    // Refine using first derivative zero-crossing near the peak
    let deriv: Vec<f64> = numerical_derivative(&sweep.potential, &sweep.current);
    let mut refined_idx: usize = best_idx;
    let search_start: usize = if best_idx > 3 { best_idx - 3 } else { 0 };
    let search_end: usize = (best_idx + 4).min(deriv.len().saturating_sub(1));
    for i in search_start..search_end {
        if i + 1 < deriv.len() && deriv[i] * deriv[i + 1] <= 0.0 {
            refined_idx = if deriv[i].abs() < deriv[i + 1].abs() { i } else { i + 1 };
            break;
        }
    }

    let hw: Option<f64> = half_peak_width(sweep, refined_idx);

    Some(PeakResult {
        peak_potential: sweep.potential[refined_idx],
        peak_current: sweep.current[refined_idx],
        half_width: hw,
        index: refined_idx,
    })
}

// ---------------------------------------------------------------------------
// Cyclic Voltammetry
// ---------------------------------------------------------------------------

/// Analyze cyclic voltammetry data (forward and reverse sweeps).
pub fn cyclic_voltammetry_analyze(
    forward: &VoltammetricSweep,
    reverse: &VoltammetricSweep,
    n: u32,
) -> Option<CvResult> {
    let fwd_peak: PeakResult = linear_sweep_analyze(forward)?;
    let rev_peak: PeakResult = linear_sweep_analyze(reverse)?;

    let epa: f64 = fwd_peak.peak_potential;
    let epc: f64 = rev_peak.peak_potential;
    let ipa: f64 = fwd_peak.peak_current;
    let ipc: f64 = rev_peak.peak_current;
    let delta_ep: f64 = (epa - epc).abs();
    let e_formal: f64 = (epa + epc) / 2.0;
    let current_ratio: f64 = if ipc.abs() > 1e-30 {
        (ipa / ipc).abs()
    } else {
        f64::INFINITY
    };

    // Reversibility criterion: ΔEp ≈ 59.2/n mV at 25°C
    let ideal_sep: f64 = 0.0592 / (n as f64);
    let reversible: bool = (delta_ep - ideal_sep).abs() < 0.020; // within 20 mV

    Some(CvResult {
        epa,
        epc,
        ipa,
        ipc,
        delta_ep,
        e_formal,
        current_ratio,
        reversible,
    })
}

// ---------------------------------------------------------------------------
// Differential Pulse Voltammetry
// ---------------------------------------------------------------------------

/// Analyze a differential pulse voltammogram.
/// DPV measures the difference current between two pulses, giving enhanced peak shape.
pub fn differential_pulse_analyze(sweep: &VoltammetricSweep) -> Option<PeakResult> {
    // DPV data is already difference current; find peak
    linear_sweep_analyze(sweep)
}

// ---------------------------------------------------------------------------
// Square Wave Voltammetry
// ---------------------------------------------------------------------------

/// SWV result with net, forward, and reverse currents.
#[derive(Debug, Clone)]
pub struct SwvResult {
    /// Peak from net current.
    pub peak: PeakResult,
    /// Net current (forward - reverse).
    pub net_current: Vec<f64>,
}

/// Analyze square wave voltammetry data.
pub fn square_wave_analyze(
    forward_current: &[f64],
    reverse_current: &[f64],
    potentials: &[f64],
) -> Option<SwvResult> {
    let n: usize = forward_current.len().min(reverse_current.len()).min(potentials.len());
    if n < 3 {
        return None;
    }
    let net: Vec<f64> = (0..n).map(|i| forward_current[i] - reverse_current[i]).collect();
    let sweep = VoltammetricSweep::new(potentials[..n].to_vec(), net.clone());
    let peak: PeakResult = linear_sweep_analyze(&sweep)?;
    Some(SwvResult {
        peak,
        net_current: net,
    })
}

// ---------------------------------------------------------------------------
// DC Polarography
// ---------------------------------------------------------------------------

/// Analyze a DC polarogram to find the half-wave potential and diffusion current.
pub fn dc_polarography_analyze(sweep: &VoltammetricSweep) -> Option<HalfWavePotential> {
    if sweep.len() < 5 {
        return None;
    }
    // Find the diffusion-limited plateau current (id)
    // Use the average of the last 10% of points as the limiting current
    let n: usize = sweep.len();
    let tail_start: usize = n - (n / 10).max(2);
    let mut id_sum: f64 = 0.0;
    let tail_count: f64 = (n - tail_start) as f64;
    for i in tail_start..n {
        id_sum += sweep.current[i];
    }
    let id: f64 = id_sum / tail_count;

    // Subtract residual current (first few points)
    let head_end: usize = (n / 10).max(2);
    let mut res_sum: f64 = 0.0;
    for i in 0..head_end {
        res_sum += sweep.current[i];
    }
    let i_residual: f64 = res_sum / (head_end as f64);
    let id_corrected: f64 = id - i_residual;

    if id_corrected.abs() < 1e-15 {
        return None;
    }

    // E½ is where i = id/2
    let half_current: f64 = i_residual + id_corrected / 2.0;
    let mut e_half: f64 = sweep.potential[n / 2];
    for i in 0..n - 1 {
        let i0: f64 = sweep.current[i];
        let i1: f64 = sweep.current[i + 1];
        if (i0 - half_current) * (i1 - half_current) <= 0.0 {
            // Linear interpolation
            let frac: f64 = (half_current - i0) / (i1 - i0);
            e_half = sweep.potential[i] + frac * (sweep.potential[i + 1] - sweep.potential[i]);
            break;
        }
    }

    // Determine n from Heyrovsky-Ilkovic log plot
    let n_e: f64 = heyrovsky_ilkovic_n(sweep, id_corrected, i_residual);

    Some(HalfWavePotential {
        e_half,
        i_d: id_corrected,
        n_electrons: n_e,
    })
}

/// Heyrovsky-Ilkovic equation: plot log((id-i)/(i-ir)) vs E.
/// Slope = -nF/(2.303RT), so n = -slope * 2.303 * R * T / F.
fn heyrovsky_ilkovic_n(
    sweep: &VoltammetricSweep,
    id: f64,
    i_residual: f64,
) -> f64 {
    let mut sum_x: f64 = 0.0;
    let mut sum_y: f64 = 0.0;
    let mut sum_xx: f64 = 0.0;
    let mut sum_xy: f64 = 0.0;
    let mut count: f64 = 0.0;

    for i in 0..sweep.len() {
        let curr: f64 = sweep.current[i] - i_residual;
        let ratio_num: f64 = id - curr;
        let ratio_den: f64 = curr;
        if ratio_num > 1e-15 && ratio_den > 1e-15 {
            let log_ratio: f64 = (ratio_num / ratio_den).ln() / (10.0_f64).ln();
            let e: f64 = sweep.potential[i];
            sum_x += e;
            sum_y += log_ratio;
            sum_xx += e * e;
            sum_xy += e * log_ratio;
            count += 1.0;
        }
    }

    if count < 3.0 {
        return 1.0;
    }

    let slope: f64 = (count * sum_xy - sum_x * sum_y) / (count * sum_xx - sum_x * sum_x);
    // slope = -n*F / (2.303*R*T)
    let n_est: f64 = (-slope * 2.303 * R_CONST * T_STD / F_CONST).abs();
    // Round to nearest integer
    if n_est < 0.5 {
        1.0
    } else {
        n_est.round()
    }
}

// ---------------------------------------------------------------------------
// Ilkovic Equation
// ---------------------------------------------------------------------------

/// Ilkovic equation for dropping mercury electrode diffusion current.
///
/// id = 607 * n * D^(1/2) * m^(2/3) * t^(1/6) * C
///
/// - `n`: number of electrons
/// - `d_cm2_s`: diffusion coefficient (cm²/s)
/// - `m_mg_s`: mercury flow rate (mg/s)
/// - `t_s`: drop time (s)
/// - `c_mmol_l`: concentration (mmol/L)
///
/// Returns current in µA.
pub fn ilkovic_current(n: u32, d_cm2_s: f64, m_mg_s: f64, t_s: f64, c_mmol_l: f64) -> f64 {
    607.0
        * (n as f64)
        * d_cm2_s.sqrt()
        * m_mg_s.powf(2.0 / 3.0)
        * t_s.powf(1.0 / 6.0)
        * c_mmol_l
}

// ---------------------------------------------------------------------------
// Randles-Sevcik
// ---------------------------------------------------------------------------

/// Randles-Sevcik equation: compute diffusion coefficient from peak current.
///
/// ip = 2.69e5 * n^(3/2) * A * D^(1/2) * C * v^(1/2)
///
/// Returns D in cm²/s.
pub fn randles_sevcik(
    ip_a: f64,
    n: u32,
    area_cm2: f64,
    c_mol_cm3: f64,
    scan_rate_v_s: f64,
) -> f64 {
    let nf: f64 = n as f64;
    let denom: f64 = 2.69e5 * nf.powf(1.5) * area_cm2 * c_mol_cm3 * scan_rate_v_s.sqrt();
    if denom.abs() < 1e-30 {
        return 0.0;
    }
    let ratio: f64 = ip_a.abs() / denom;
    ratio * ratio // D = (ip / (2.69e5 * n^1.5 * A * C * v^0.5))^2
}

/// Compute peak current from Randles-Sevcik equation.
pub fn randles_sevcik_ip(
    n: u32,
    area_cm2: f64,
    d_cm2_s: f64,
    c_mol_cm3: f64,
    scan_rate_v_s: f64,
) -> f64 {
    let nf: f64 = n as f64;
    2.69e5 * nf.powf(1.5) * area_cm2 * d_cm2_s.sqrt() * c_mol_cm3 * scan_rate_v_s.sqrt()
}

// ---------------------------------------------------------------------------
// Cottrell Equation
// ---------------------------------------------------------------------------

/// Cottrell equation result.
#[derive(Debug, Clone)]
pub struct CottrellResult {
    /// Slope of i vs t^(-1/2) plot.
    pub slope: f64,
    /// Diffusion coefficient (cm²/s).
    pub d_cm2_s: f64,
    /// R² of linear fit.
    pub r_squared: f64,
}

/// Cottrell analysis: i(t) = nFAD^(1/2)C / (π*t)^(1/2).
/// Plots i vs t^(-1/2), fits linear, extracts D.
pub fn cottrell_analysis(
    time_s: &[f64],
    current_a: &[f64],
    n: u32,
    area_cm2: f64,
    c_mol_cm3: f64,
) -> Option<CottrellResult> {
    let len: usize = time_s.len().min(current_a.len());
    if len < 3 {
        return None;
    }

    // Build i vs t^(-1/2) data, skipping t=0
    let mut x_vals: Vec<f64> = Vec::new();
    let mut y_vals: Vec<f64> = Vec::new();
    for i in 0..len {
        if time_s[i] > 1e-10 {
            x_vals.push(1.0 / time_s[i].sqrt());
            y_vals.push(current_a[i]);
        }
    }

    if x_vals.len() < 2 {
        return None;
    }

    let (slope, _intercept, r2) = linear_regression(&x_vals, &y_vals);

    // slope = nFAD^(1/2)C / π^(1/2)
    let nf: f64 = n as f64;
    let denom: f64 = nf * F_CONST * area_cm2 * c_mol_cm3 / PI.sqrt();
    let d: f64 = if denom.abs() > 1e-30 {
        let d_sqrt: f64 = slope.abs() / denom;
        d_sqrt * d_sqrt
    } else {
        0.0
    };

    Some(CottrellResult {
        slope,
        d_cm2_s: d,
        r_squared: r2,
    })
}

/// Cottrell current at time t.
pub fn cottrell_current(n: u32, area_cm2: f64, d_cm2_s: f64, c_mol_cm3: f64, t_s: f64) -> f64 {
    if t_s <= 0.0 {
        return 0.0;
    }
    let nf: f64 = n as f64;
    nf * F_CONST * area_cm2 * d_cm2_s.sqrt() * c_mol_cm3 / (PI * t_s).sqrt()
}

// ---------------------------------------------------------------------------
// Levich Equation
// ---------------------------------------------------------------------------

/// Levich equation result.
#[derive(Debug, Clone)]
pub struct LevichResult {
    /// Slope of il vs ω^(1/2) plot.
    pub slope: f64,
    /// Diffusion coefficient (cm²/s).
    pub d_cm2_s: f64,
    /// R² of fit.
    pub r_squared: f64,
}

/// Levich analysis for rotating disk electrode.
/// il = 0.62 * n * F * A * D^(2/3) * ω^(1/2) * ν^(-1/6) * C
///
/// Plots il vs ω^(1/2), extracts D from slope.
pub fn levich_analysis(
    currents_a: &[f64],
    rotation_rad_s: &[f64],
    n: u32,
    area_cm2: f64,
    c_mol_cm3: f64,
    kinematic_viscosity: f64,
) -> Option<LevichResult> {
    let len: usize = currents_a.len().min(rotation_rad_s.len());
    if len < 2 {
        return None;
    }

    let x: Vec<f64> = rotation_rad_s[..len].iter().map(|&w| w.sqrt()).collect();
    let y: Vec<f64> = currents_a[..len].to_vec();
    let (slope, _int, r2) = linear_regression(&x, &y);

    // slope = 0.62 * n * F * A * D^(2/3) * ν^(-1/6) * C
    let nf: f64 = n as f64;
    let coeff: f64 = 0.62 * nf * F_CONST * area_cm2 * kinematic_viscosity.powf(-1.0 / 6.0) * c_mol_cm3;
    let d: f64 = if coeff.abs() > 1e-30 {
        (slope.abs() / coeff).powf(1.5)
    } else {
        0.0
    };

    Some(LevichResult {
        slope,
        d_cm2_s: d,
        r_squared: r2,
    })
}

/// Levich limiting current.
pub fn levich_current(
    n: u32,
    area_cm2: f64,
    d_cm2_s: f64,
    omega_rad_s: f64,
    nu_cm2_s: f64,
    c_mol_cm3: f64,
) -> f64 {
    let nf: f64 = n as f64;
    0.62 * nf * F_CONST * area_cm2 * d_cm2_s.powf(2.0 / 3.0) * omega_rad_s.sqrt()
        * nu_cm2_s.powf(-1.0 / 6.0)
        * c_mol_cm3
}

// ---------------------------------------------------------------------------
// Koutecky-Levich
// ---------------------------------------------------------------------------

/// Koutecky-Levich result.
#[derive(Debug, Clone)]
pub struct KouteckyLevichResult {
    /// Kinetic current (A).
    pub i_kinetic: f64,
    /// Y-intercept of 1/i vs 1/ω^(1/2) plot.
    pub intercept: f64,
    /// Slope.
    pub slope: f64,
    /// R² of fit.
    pub r_squared: f64,
}

/// Koutecky-Levich analysis: 1/i = 1/ik + 1/(B*ω^(1/2))
/// where B = 0.62*n*F*A*D^(2/3)*ν^(-1/6)*C.
pub fn koutecky_levich_analysis(
    currents_a: &[f64],
    rotation_rad_s: &[f64],
) -> Option<KouteckyLevichResult> {
    let len: usize = currents_a.len().min(rotation_rad_s.len());
    if len < 2 {
        return None;
    }

    let mut x: Vec<f64> = Vec::new();
    let mut y: Vec<f64> = Vec::new();
    for i in 0..len {
        if rotation_rad_s[i] > 1e-10 && currents_a[i].abs() > 1e-15 {
            x.push(1.0 / rotation_rad_s[i].sqrt());
            y.push(1.0 / currents_a[i]);
        }
    }

    if x.len() < 2 {
        return None;
    }

    let (slope, intercept, r2) = linear_regression(&x, &y);
    let ik: f64 = if intercept.abs() > 1e-30 {
        1.0 / intercept
    } else {
        f64::INFINITY
    };

    Some(KouteckyLevichResult {
        i_kinetic: ik,
        intercept,
        slope,
        r_squared: r2,
    })
}

// ---------------------------------------------------------------------------
// Tafel Analysis
// ---------------------------------------------------------------------------

/// Tafel analysis: extract kinetic parameters from overpotential vs log|i| data.
pub fn tafel_analysis(
    overpotential_v: &[f64],
    current_a: &[f64],
    n: f64,
    temperature_k: f64,
) -> Option<TafelResult> {
    let len: usize = overpotential_v.len().min(current_a.len());
    if len < 3 {
        return None;
    }

    // Build log|i| vs η data for anodic branch (η > threshold)
    let mut x: Vec<f64> = Vec::new();
    let mut y: Vec<f64> = Vec::new();
    for i in 0..len {
        let abs_i: f64 = current_a[i].abs();
        if abs_i > 1e-15 && overpotential_v[i].abs() > 0.03 {
            x.push(overpotential_v[i]);
            y.push(abs_i.log10());
        }
    }

    if x.len() < 2 {
        return None;
    }

    let (slope, intercept, r2) = linear_regression(&x, &y);

    // Tafel slope = 2.303*R*T / (alpha*n*F)
    // slope (of log|i| vs η) = alpha*n*F / (2.303*R*T)
    let alpha: f64 = slope.abs() * 2.303 * R_CONST * temperature_k / (n * F_CONST);
    let alpha_clamped: f64 = alpha.clamp(0.01, 0.99);
    let i0: f64 = 10.0_f64.powf(intercept);
    let tafel_slope: f64 = 2.303 * R_CONST * temperature_k / (alpha_clamped * n * F_CONST);

    Some(TafelResult {
        slope: tafel_slope,
        i0,
        alpha: alpha_clamped,
        n,
        r_squared: r2,
    })
}

// ---------------------------------------------------------------------------
// Butler-Volmer
// ---------------------------------------------------------------------------

/// Butler-Volmer equation for electrode kinetics.
///
/// i = i0 * [exp(α*n*F*η/(RT)) - exp(-(1-α)*n*F*η/(RT))]
pub fn butler_volmer(i0: f64, alpha: f64, n: f64, eta_v: f64, temperature_k: f64) -> f64 {
    let f_rt: f64 = F_CONST / (R_CONST * temperature_k);
    let anodic: f64 = (alpha * n * f_rt * eta_v).exp();
    let cathodic: f64 = (-(1.0 - alpha) * n * f_rt * eta_v).exp();
    i0 * (anodic - cathodic)
}

/// Butler-Volmer at standard temperature.
pub fn butler_volmer_std(i0: f64, alpha: f64, n: f64, eta_v: f64) -> f64 {
    butler_volmer(i0, alpha, n, eta_v, T_STD)
}

// ---------------------------------------------------------------------------
// Baseline Correction
// ---------------------------------------------------------------------------

/// Apply baseline correction to a sweep.
pub fn baseline_correct(
    sweep: &VoltammetricSweep,
    method: BaselineMethod,
) -> VoltammetricSweep {
    let corrected: Vec<f64> = match method {
        BaselineMethod::Linear => {
            if sweep.len() < 2 {
                return sweep.clone();
            }
            let n: usize = sweep.len();
            let i0: f64 = sweep.current[0];
            let i1: f64 = sweep.current[n - 1];
            (0..n)
                .map(|k| {
                    let frac: f64 = k as f64 / (n - 1) as f64;
                    sweep.current[k] - (i0 + frac * (i1 - i0))
                })
                .collect()
        }
        BaselineMethod::MovingMinimum(window) => {
            let hw: usize = window / 2;
            let n: usize = sweep.len();
            let baseline: Vec<f64> = (0..n)
                .map(|i| {
                    let start: usize = if i >= hw { i - hw } else { 0 };
                    let end: usize = (i + hw + 1).min(n);
                    let mut min_val: f64 = f64::MAX;
                    for j in start..end {
                        if sweep.current[j] < min_val {
                            min_val = sweep.current[j];
                        }
                    }
                    min_val
                })
                .collect();
            (0..n).map(|i| sweep.current[i] - baseline[i]).collect()
        }
        BaselineMethod::Polynomial(degree) => {
            let coeffs: Vec<f64> = polyfit(&sweep.potential, &sweep.current, degree);
            let n: usize = sweep.len();
            (0..n)
                .map(|i| {
                    let bl: f64 = polyeval(&coeffs, sweep.potential[i]);
                    sweep.current[i] - bl
                })
                .collect()
        }
    };

    VoltammetricSweep::new(sweep.potential.clone(), corrected)
}

// ---------------------------------------------------------------------------
// Savitzky-Golay smoothing (simple 5/7-point)
// ---------------------------------------------------------------------------

/// Simple 5-point Savitzky-Golay smoothing.
pub fn sg_smooth_5(data: &[f64]) -> Vec<f64> {
    let n: usize = data.len();
    if n < 5 {
        return data.to_vec();
    }
    let mut out: Vec<f64> = vec![0.0; n];
    out[0] = data[0];
    out[1] = data[1];
    out[n - 2] = data[n - 2];
    out[n - 1] = data[n - 1];
    for i in 2..n - 2 {
        out[i] = (-3.0 * data[i - 2] + 12.0 * data[i - 1] + 17.0 * data[i]
            + 12.0 * data[i + 1] - 3.0 * data[i + 2])
            / 35.0;
    }
    out
}

/// Simple 7-point Savitzky-Golay smoothing.
pub fn sg_smooth_7(data: &[f64]) -> Vec<f64> {
    let n: usize = data.len();
    if n < 7 {
        return data.to_vec();
    }
    let mut out: Vec<f64> = vec![0.0; n];
    for i in 0..3 {
        out[i] = data[i];
    }
    for i in n - 3..n {
        out[i] = data[i];
    }
    for i in 3..n - 3 {
        out[i] = (-2.0 * data[i - 3] + 3.0 * data[i - 2] + 6.0 * data[i - 1]
            + 7.0 * data[i] + 6.0 * data[i + 1] + 3.0 * data[i + 2]
            - 2.0 * data[i + 3])
            / 21.0;
    }
    out
}

// ---------------------------------------------------------------------------
// Peak Deconvolution
// ---------------------------------------------------------------------------

/// Gaussian peak: A * exp(-(x - mu)^2 / (2*sigma^2)).
pub fn gaussian_peak(x: f64, amplitude: f64, center: f64, sigma: f64) -> f64 {
    amplitude * (-((x - center).powi(2)) / (2.0 * sigma * sigma)).exp()
}

/// Fit a sum of n_peaks Gaussians to the data.
/// Returns Vec of (amplitude, center, sigma) for each peak.
pub fn deconvolve_peaks(
    potentials: &[f64],
    currents: &[f64],
    n_peaks: usize,
) -> Vec<(f64, f64, f64)> {
    if potentials.is_empty() || n_peaks == 0 {
        return vec![];
    }

    let n: usize = potentials.len().min(currents.len());
    let e_min: f64 = potentials[0];
    let e_max: f64 = potentials[n - 1];
    let e_range: f64 = (e_max - e_min).abs().max(0.01);
    let i_max: f64 = currents.iter().copied().fold(0.0_f64, f64::max);

    // Initialize peaks evenly spaced
    let mut params: Vec<(f64, f64, f64)> = (0..n_peaks)
        .map(|k| {
            let center: f64 = e_min + e_range * (k as f64 + 0.5) / (n_peaks as f64);
            let sigma: f64 = e_range / (n_peaks as f64 * 4.0);
            let amp: f64 = i_max / (n_peaks as f64);
            (amp, center, sigma)
        })
        .collect();

    // Simple iterative refinement
    for _ in 0..100 {
        let mut residuals: Vec<f64> = vec![0.0; n];
        for i in 0..n {
            let mut model: f64 = 0.0;
            for &(a, c, s) in &params {
                model += gaussian_peak(potentials[i], a, c, s);
            }
            residuals[i] = currents[i] - model;
        }

        // Update each peak
        for pk in 0..n_peaks {
            let (a, c, s) = params[pk];
            let step: f64 = 0.001;

            // Gradient for amplitude
            let da: f64 = residual_change(&potentials[..n], &currents[..n], &params, pk, a + step, c, s)
                - residual_change(&potentials[..n], &currents[..n], &params, pk, a, c, s);
            // Gradient for center
            let dc: f64 = residual_change(&potentials[..n], &currents[..n], &params, pk, a, c + step, s)
                - residual_change(&potentials[..n], &currents[..n], &params, pk, a, c, s);
            // Gradient for sigma
            let ds: f64 = residual_change(&potentials[..n], &currents[..n], &params, pk, a, c, s + step)
                - residual_change(&potentials[..n], &currents[..n], &params, pk, a, c, s);

            let grad_norm: f64 = (da * da + dc * dc + ds * ds).sqrt();
            if grad_norm < 1e-15 {
                continue;
            }
            let lr: f64 = 0.01 / grad_norm;
            params[pk].0 = (a - lr * da).max(0.0);
            params[pk].1 = c - lr * dc;
            params[pk].2 = (s - lr * ds).max(0.001);
        }
    }

    params
}

fn residual_change(
    potentials: &[f64],
    currents: &[f64],
    params: &[(f64, f64, f64)],
    pk_idx: usize,
    a: f64,
    c: f64,
    s: f64,
) -> f64 {
    let mut sse: f64 = 0.0;
    for i in 0..potentials.len() {
        let mut model: f64 = 0.0;
        for (k, &(ak, ck, sk)) in params.iter().enumerate() {
            if k == pk_idx {
                model += gaussian_peak(potentials[i], a, c, s);
            } else {
                model += gaussian_peak(potentials[i], ak, ck, sk);
            }
        }
        let r: f64 = currents[i] - model;
        sse += r * r;
    }
    sse
}

// ---------------------------------------------------------------------------
// Nernst Equation
// ---------------------------------------------------------------------------

/// Nernst equation: E = E0 + (RT/nF) * ln(Ox/Red)
pub fn nernst_potential(e0: f64, n: u32, ox_red_ratio: f64, temperature_k: f64) -> f64 {
    if ox_red_ratio <= 0.0 {
        return e0;
    }
    let nf: f64 = n as f64;
    e0 + (R_CONST * temperature_k / (nf * F_CONST)) * ox_red_ratio.ln()
}

/// Nernst at standard temperature.
pub fn nernst_std(e0: f64, n: u32, ox_red_ratio: f64) -> f64 {
    nernst_potential(e0, n, ox_red_ratio, T_STD)
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Numerical first derivative using central differences.
fn numerical_derivative(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n: usize = x.len().min(y.len());
    if n < 2 {
        return vec![];
    }
    let mut d: Vec<f64> = Vec::with_capacity(n);
    d.push((y[1] - y[0]) / (x[1] - x[0]).max(1e-30));
    for i in 1..n - 1 {
        let dx: f64 = x[i + 1] - x[i - 1];
        let dy: f64 = y[i + 1] - y[i - 1];
        d.push(if dx.abs() > 1e-30 { dy / dx } else { 0.0 });
    }
    d.push((y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]).max(1e-30));
    d
}

/// Half-peak width estimation.
fn half_peak_width(sweep: &VoltammetricSweep, peak_idx: usize) -> Option<f64> {
    let ip: f64 = sweep.current[peak_idx];
    let half_val: f64 = ip / 2.0;
    let n: usize = sweep.len();

    // Search left
    let mut left_e: Option<f64> = None;
    for i in (0..peak_idx).rev() {
        if (sweep.current[i] - half_val) * (sweep.current[i + 1] - half_val) <= 0.0 {
            let frac: f64 = (half_val - sweep.current[i]) / (sweep.current[i + 1] - sweep.current[i]);
            left_e = Some(sweep.potential[i] + frac * (sweep.potential[i + 1] - sweep.potential[i]));
            break;
        }
    }

    // Search right
    let mut right_e: Option<f64> = None;
    for i in peak_idx..n - 1 {
        if (sweep.current[i] - half_val) * (sweep.current[i + 1] - half_val) <= 0.0 {
            let frac: f64 = (half_val - sweep.current[i]) / (sweep.current[i + 1] - sweep.current[i]);
            right_e = Some(sweep.potential[i] + frac * (sweep.potential[i + 1] - sweep.potential[i]));
            break;
        }
    }

    match (left_e, right_e) {
        (Some(l), Some(r)) => Some((r - l).abs()),
        _ => None,
    }
}

/// Simple linear regression: y = slope*x + intercept. Returns (slope, intercept, r²).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n: f64 = x.len() as f64;
    if n < 2.0 {
        return (0.0, 0.0, 0.0);
    }
    let mut sx: f64 = 0.0;
    let mut sy: f64 = 0.0;
    let mut sxx: f64 = 0.0;
    let mut sxy: f64 = 0.0;
    let mut syy: f64 = 0.0;
    for i in 0..x.len() {
        sx += x[i];
        sy += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
        syy += y[i] * y[i];
    }
    let denom: f64 = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / n, 0.0);
    }
    let slope: f64 = (n * sxy - sx * sy) / denom;
    let intercept: f64 = (sy - slope * sx) / n;

    let ss_tot: f64 = syy - sy * sy / n;
    let ss_res: f64 = {
        let mut s: f64 = 0.0;
        for i in 0..x.len() {
            let r: f64 = y[i] - (slope * x[i] + intercept);
            s += r * r;
        }
        s
    };
    let r2: f64 = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 1.0 };
    (slope, intercept, r2)
}

/// Polynomial fit via normal equations. Returns coefficients [a0, a1, ..., a_deg].
fn polyfit(x: &[f64], y: &[f64], degree: usize) -> Vec<f64> {
    let n: usize = x.len().min(y.len());
    let p: usize = degree + 1;
    if n < p {
        return vec![0.0; p];
    }

    // Build normal equations: (X^T X) a = X^T y
    let mut ata = vec![vec![0.0; p]; p];
    let mut aty = vec![0.0; p];

    for i in 0..n {
        let mut xi_pow = vec![1.0_f64; p];
        for j in 1..p {
            xi_pow[j] = xi_pow[j - 1] * x[i];
        }
        for j in 0..p {
            aty[j] += xi_pow[j] * y[i];
            for k in 0..p {
                ata[j][k] += xi_pow[j] * xi_pow[k];
            }
        }
    }

    // Gaussian elimination
    gauss_solve(&mut ata, &mut aty)
}

fn gauss_solve(a: &mut Vec<Vec<f64>>, b: &mut Vec<f64>) -> Vec<f64> {
    let n: usize = b.len();
    for col in 0..n {
        // Partial pivoting
        let mut max_row: usize = col;
        let mut max_val: f64 = a[col][col].abs();
        for row in col + 1..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot: f64 = a[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in col + 1..n {
            let factor: f64 = a[row][col] / pivot;
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x: Vec<f64> = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum: f64 = b[i];
        for j in i + 1..n {
            sum -= a[i][j] * x[j];
        }
        x[i] = if a[i][i].abs() > 1e-30 { sum / a[i][i] } else { 0.0 };
    }
    x
}

fn polyeval(coeffs: &[f64], x: f64) -> f64 {
    let mut result: f64 = 0.0;
    let mut x_pow: f64 = 1.0;
    for &c in coeffs {
        result += c * x_pow;
        x_pow *= x;
    }
    result
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // === VoltammetricSweep ===

    #[test]
    fn test_sweep_new() {
        let s = VoltammetricSweep::new(vec![0.0, 0.1, 0.2], vec![1.0, 2.0, 3.0]);
        assert_eq!(s.len(), 3);
        assert!(!s.is_empty());
    }

    #[test]
    fn test_sweep_empty() {
        let s = VoltammetricSweep::new(vec![], vec![]);
        assert!(s.is_empty());
    }

    // === Linear Sweep Analysis ===

    #[test]
    fn test_lsv_peak_detection() {
        let e: Vec<f64> = (0..100).map(|i| -0.5 + i as f64 * 0.01).collect();
        let i: Vec<f64> = e.iter().map(|&x| {
            // Gaussian peak centered at 0.0 V
            1e-4 * (-((x * 10.0).powi(2))).exp()
        }).collect();
        let sweep = VoltammetricSweep::new(e, i);
        let peak = linear_sweep_analyze(&sweep);
        assert!(peak.is_some());
        let p = peak.unwrap();
        assert!(p.peak_potential.abs() < 0.05, "peak at {}", p.peak_potential);
        assert!(p.peak_current > 5e-5);
    }

    #[test]
    fn test_lsv_peak_at_edge() {
        // Monotonically increasing - peak at end
        let e: Vec<f64> = vec![0.0, 0.1, 0.2, 0.3, 0.4];
        let i: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let sweep = VoltammetricSweep::new(e, i);
        let peak = linear_sweep_analyze(&sweep);
        assert!(peak.is_some());
        assert_eq!(peak.unwrap().index, 4);
    }

    #[test]
    fn test_lsv_too_few_points() {
        let sweep = VoltammetricSweep::new(vec![0.0, 0.1], vec![1.0, 2.0]);
        assert!(linear_sweep_analyze(&sweep).is_none());
    }

    // === Cyclic Voltammetry ===

    #[test]
    fn test_cv_reversible() {
        // Reversible 1-electron system: ΔEp ≈ 59 mV
        let n_pts: usize = 100;
        let fwd_e: Vec<f64> = (0..n_pts).map(|i| -0.3 + i as f64 * 0.006).collect();
        let fwd_i: Vec<f64> = fwd_e.iter().map(|&x| {
            1e-4 * (-((x + 0.03) * 20.0).powi(2)).exp()
        }).collect();
        let fwd = VoltammetricSweep::new(fwd_e, fwd_i);

        let rev_e: Vec<f64> = (0..n_pts).map(|i| 0.3 - i as f64 * 0.006).collect();
        let rev_i: Vec<f64> = rev_e.iter().map(|&x| {
            -1e-4 * (-((x + 0.03 - 0.059) * 20.0).powi(2)).exp()
        }).collect();
        let rev = VoltammetricSweep::new(rev_e, rev_i);

        let result = cyclic_voltammetry_analyze(&fwd, &rev, 1);
        assert!(result.is_some());
        let cv = result.unwrap();
        assert!(cv.delta_ep < 0.15, "ΔEp = {}", cv.delta_ep);
        assert!(cv.current_ratio > 0.5);
    }

    #[test]
    fn test_cv_irreversible() {
        let n_pts: usize = 50;
        let fwd_e: Vec<f64> = (0..n_pts).map(|i| -0.5 + i as f64 * 0.02).collect();
        let fwd_i: Vec<f64> = fwd_e.iter().map(|&x| {
            1e-4 * (-(x * 10.0).powi(2)).exp()
        }).collect();
        let fwd = VoltammetricSweep::new(fwd_e, fwd_i);

        let rev_e: Vec<f64> = (0..n_pts).map(|i| 0.5 - i as f64 * 0.02).collect();
        let rev_i: Vec<f64> = rev_e.iter().map(|&x| {
            -1e-4 * (-((x - 0.3) * 10.0).powi(2)).exp()
        }).collect();
        let rev = VoltammetricSweep::new(rev_e, rev_i);

        let result = cyclic_voltammetry_analyze(&fwd, &rev, 1);
        assert!(result.is_some());
        let cv = result.unwrap();
        assert!(cv.delta_ep > 0.1, "ΔEp = {} should indicate irreversible", cv.delta_ep);
    }

    // === DPV ===

    #[test]
    fn test_dpv_analysis() {
        let e: Vec<f64> = (0..50).map(|i| -0.3 + i as f64 * 0.01).collect();
        let i: Vec<f64> = e.iter().map(|&x| {
            5e-5 * (-(x * 15.0).powi(2)).exp()
        }).collect();
        let sweep = VoltammetricSweep::new(e, i);
        let peak = differential_pulse_analyze(&sweep);
        assert!(peak.is_some());
        let p = peak.unwrap();
        assert!(p.peak_potential.abs() < 0.05);
    }

    // === SWV ===

    #[test]
    fn test_swv_analysis() {
        let potentials: Vec<f64> = (0..50).map(|i| -0.3 + i as f64 * 0.01).collect();
        let fwd: Vec<f64> = potentials.iter().map(|&x| {
            3e-5 * (-(x * 15.0).powi(2)).exp()
        }).collect();
        let rev: Vec<f64> = potentials.iter().map(|&x| {
            -1e-5 * (-(x * 15.0).powi(2)).exp()
        }).collect();
        let result = square_wave_analyze(&fwd, &rev, &potentials);
        assert!(result.is_some());
        let swv = result.unwrap();
        assert_eq!(swv.net_current.len(), 50);
        assert!(swv.peak.peak_potential.abs() < 0.05);
    }

    #[test]
    fn test_swv_too_few() {
        let result = square_wave_analyze(&[1.0, 2.0], &[0.5, 1.0], &[0.0, 0.1]);
        assert!(result.is_none());
    }

    // === DC Polarography ===

    #[test]
    fn test_dc_polarography() {
        let n_pts: usize = 100;
        let e: Vec<f64> = (0..n_pts).map(|i| -0.2 - i as f64 * 0.008).collect();
        let i: Vec<f64> = e.iter().map(|&x| {
            // Sigmoidal: id / (1 + exp(-(E - E½) * nF/RT))
            let id: f64 = 5e-6;
            let e_half: f64 = -0.5;
            id / (1.0 + ((x - e_half) / VT).exp())
        }).collect();
        let sweep = VoltammetricSweep::new(e, i);
        let result = dc_polarography_analyze(&sweep);
        assert!(result.is_some());
        let hwp = result.unwrap();
        assert!(hwp.i_d > 0.0);
        assert!((hwp.e_half - (-0.5)).abs() < 0.1, "E½ = {}", hwp.e_half);
    }

    #[test]
    fn test_dc_too_few() {
        let sweep = VoltammetricSweep::new(vec![0.0, 0.1, 0.2], vec![1.0, 2.0, 3.0]);
        assert!(dc_polarography_analyze(&sweep).is_none());
    }

    // === Ilkovic ===

    #[test]
    fn test_ilkovic_current() {
        let id: f64 = ilkovic_current(2, 7.2e-6, 2.0, 4.0, 1.0);
        // id = 607 * 2 * sqrt(7.2e-6) * 2^(2/3) * 4^(1/6) * 1.0
        assert!(id > 0.0);
        let expected: f64 = 607.0 * 2.0 * (7.2e-6_f64).sqrt() * 2.0_f64.powf(2.0/3.0) * 4.0_f64.powf(1.0/6.0) * 1.0;
        assert!(approx(id, expected, 1e-6));
    }

    #[test]
    fn test_ilkovic_zero_concentration() {
        assert!(approx(ilkovic_current(1, 1e-5, 1.0, 1.0, 0.0), 0.0, TOL));
    }

    #[test]
    fn test_ilkovic_proportional_to_n() {
        let i1: f64 = ilkovic_current(1, 1e-5, 1.0, 1.0, 1.0);
        let i2: f64 = ilkovic_current(2, 1e-5, 1.0, 1.0, 1.0);
        assert!(approx(i2 / i1, 2.0, 0.01));
    }

    // === Randles-Sevcik ===

    #[test]
    fn test_randles_sevcik_roundtrip() {
        let n: u32 = 1;
        let a: f64 = 0.0707;
        let d: f64 = 7.6e-6;
        let c: f64 = 1e-6;
        let v: f64 = 0.1;
        let ip: f64 = randles_sevcik_ip(n, a, d, c, v);
        let d_calc: f64 = randles_sevcik(ip, n, a, c, v);
        assert!(approx(d_calc, d, 1e-8), "D = {} expected {}", d_calc, d);
    }

    #[test]
    fn test_randles_sevcik_ip_proportional_sqrt_v() {
        let ip1: f64 = randles_sevcik_ip(1, 0.07, 1e-5, 1e-6, 0.01);
        let ip4: f64 = randles_sevcik_ip(1, 0.07, 1e-5, 1e-6, 0.04);
        assert!(approx(ip4 / ip1, 2.0, 0.01));
    }

    #[test]
    fn test_randles_sevcik_zero_scan_rate() {
        let d: f64 = randles_sevcik(1e-5, 1, 0.07, 1e-6, 0.0);
        assert!(d.is_finite());
    }

    // === Cottrell ===

    #[test]
    fn test_cottrell_current() {
        let i: f64 = cottrell_current(1, 0.07, 1e-5, 1e-6, 1.0);
        // nFAD^(1/2)C / (πt)^(1/2)
        let expected: f64 = 1.0 * F_CONST * 0.07 * (1e-5_f64).sqrt() * 1e-6 / (PI * 1.0).sqrt();
        assert!(approx(i, expected, 1e-15));
    }

    #[test]
    fn test_cottrell_current_at_zero() {
        assert_eq!(cottrell_current(1, 0.07, 1e-5, 1e-6, 0.0), 0.0);
    }

    #[test]
    fn test_cottrell_analysis() {
        let n: u32 = 1;
        let a: f64 = 0.07;
        let d: f64 = 1e-5;
        let c: f64 = 1e-6;
        let times: Vec<f64> = (1..=20).map(|i| i as f64 * 0.1).collect();
        let currents: Vec<f64> = times.iter().map(|&t| cottrell_current(n, a, d, c, t)).collect();

        let result = cottrell_analysis(&times, &currents, n, a, c);
        assert!(result.is_some());
        let r = result.unwrap();
        assert!(r.r_squared > 0.99, "R² = {}", r.r_squared);
        assert!(approx(r.d_cm2_s, d, 1e-7), "D = {} expected {}", r.d_cm2_s, d);
    }

    #[test]
    fn test_cottrell_analysis_too_few() {
        assert!(cottrell_analysis(&[1.0], &[1.0], 1, 0.07, 1e-6).is_none());
    }

    // === Levich ===

    #[test]
    fn test_levich_current() {
        let il: f64 = levich_current(1, 0.07, 1e-5, 100.0, 0.01, 1e-6);
        assert!(il > 0.0);
    }

    #[test]
    fn test_levich_proportional_sqrt_omega() {
        let i1: f64 = levich_current(1, 0.07, 1e-5, 100.0, 0.01, 1e-6);
        let i4: f64 = levich_current(1, 0.07, 1e-5, 400.0, 0.01, 1e-6);
        assert!(approx(i4 / i1, 2.0, 0.01));
    }

    #[test]
    fn test_levich_analysis() {
        let omegas: Vec<f64> = vec![100.0, 200.0, 400.0, 800.0, 1600.0];
        let currents: Vec<f64> = omegas.iter().map(|&w| levich_current(1, 0.07, 1e-5, w, 0.01, 1e-6)).collect();
        let result = levich_analysis(&currents, &omegas, 1, 0.07, 1e-6, 0.01);
        assert!(result.is_some());
        let r = result.unwrap();
        assert!(r.r_squared > 0.99);
        assert!(approx(r.d_cm2_s, 1e-5, 1e-7), "D = {}", r.d_cm2_s);
    }

    // === Koutecky-Levich ===

    #[test]
    fn test_koutecky_levich() {
        // Pure diffusion: ik → ∞, so 1/ik → 0
        let omegas: Vec<f64> = vec![100.0, 200.0, 400.0, 800.0];
        let currents: Vec<f64> = omegas.iter().map(|&w| levich_current(1, 0.07, 1e-5, w, 0.01, 1e-6)).collect();
        let result = koutecky_levich_analysis(&currents, &omegas);
        assert!(result.is_some());
        let kl = result.unwrap();
        assert!(kl.r_squared > 0.99);
    }

    #[test]
    fn test_koutecky_levich_too_few() {
        assert!(koutecky_levich_analysis(&[1.0], &[100.0]).is_none());
    }

    // === Tafel ===

    #[test]
    fn test_tafel_analysis() {
        // Generate Tafel data: log|i| = log(i0) + (alpha*n*F/(2.303*RT)) * eta
        let i0: f64 = 1e-6;
        let alpha: f64 = 0.5;
        let n: f64 = 1.0;
        let etas: Vec<f64> = (1..=20).map(|i| i as f64 * 0.01 + 0.05).collect();
        let currents: Vec<f64> = etas.iter().map(|&eta| {
            butler_volmer_std(i0, alpha, n, eta)
        }).collect();

        let result = tafel_analysis(&etas, &currents, n, T_STD);
        assert!(result.is_some());
        let t = result.unwrap();
        assert!(t.alpha > 0.3 && t.alpha < 0.7, "alpha = {}", t.alpha);
        assert!(t.i0 > 1e-8 && t.i0 < 1e-4, "i0 = {}", t.i0);
    }

    #[test]
    fn test_tafel_too_few() {
        assert!(tafel_analysis(&[0.1], &[1e-4], 1.0, T_STD).is_none());
    }

    // === Butler-Volmer ===

    #[test]
    fn test_butler_volmer_at_equilibrium() {
        let i: f64 = butler_volmer_std(1e-3, 0.5, 1.0, 0.0);
        assert!(i.abs() < 1e-15, "i at equilibrium = {}", i);
    }

    #[test]
    fn test_butler_volmer_anodic() {
        let i: f64 = butler_volmer_std(1e-3, 0.5, 1.0, 0.1);
        assert!(i > 0.0, "anodic current should be positive");
    }

    #[test]
    fn test_butler_volmer_cathodic() {
        let i: f64 = butler_volmer_std(1e-3, 0.5, 1.0, -0.1);
        assert!(i < 0.0, "cathodic current should be negative");
    }

    #[test]
    fn test_butler_volmer_symmetry() {
        let ia: f64 = butler_volmer_std(1e-3, 0.5, 1.0, 0.1);
        let ic: f64 = butler_volmer_std(1e-3, 0.5, 1.0, -0.1);
        // With alpha=0.5, should be antisymmetric
        assert!(approx(ia, -ic, 1e-10));
    }

    #[test]
    fn test_butler_volmer_limiting() {
        // Large anodic overpotential: i ≈ i0 * exp(alpha*n*F*eta/RT)
        let i: f64 = butler_volmer_std(1e-3, 0.5, 1.0, 0.5);
        let i_approx: f64 = 1e-3 * (0.5 * 1.0 * F_CONST * 0.5 / (R_CONST * T_STD)).exp();
        assert!((i - i_approx) / i_approx < 0.01);
    }

    // === Nernst ===

    #[test]
    fn test_nernst_at_unity() {
        let e: f64 = nernst_std(0.771, 1, 1.0);
        assert!(approx(e, 0.771, TOL));
    }

    #[test]
    fn test_nernst_ten_fold() {
        // 10:1 ratio, n=1: shift of +59.2 mV
        let e: f64 = nernst_std(0.0, 1, 10.0);
        assert!(approx(e, VT * (10.0_f64).ln(), 1e-4));
    }

    #[test]
    fn test_nernst_two_electron() {
        let e: f64 = nernst_std(0.0, 2, 10.0);
        let expected: f64 = VT / 2.0 * (10.0_f64).ln();
        assert!(approx(e, expected, 1e-4));
    }

    // === Baseline Correction ===

    #[test]
    fn test_baseline_linear() {
        let sweep = VoltammetricSweep::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![1.0, 3.0, 5.0, 3.0, 1.0], // V-shape + linear baseline
        );
        let corrected = baseline_correct(&sweep, BaselineMethod::Linear);
        // First and last should be 0 after correction
        assert!(corrected.current[0].abs() < TOL);
        assert!(corrected.current[4].abs() < TOL);
    }

    #[test]
    fn test_baseline_moving_minimum() {
        let sweep = VoltammetricSweep::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![1.0, 2.0, 5.0, 2.0, 1.0],
        );
        let corrected = baseline_correct(&sweep, BaselineMethod::MovingMinimum(3));
        assert!(corrected.current[2] > 0.0); // Peak should remain positive
    }

    #[test]
    fn test_baseline_polynomial() {
        let sweep = VoltammetricSweep::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 1.0, 4.0, 9.0, 16.0], // Quadratic
        );
        let corrected = baseline_correct(&sweep, BaselineMethod::Polynomial(2));
        // After subtracting a perfect quadratic fit, residuals should be near zero
        for &c in &corrected.current {
            assert!(c.abs() < 0.1, "residual = {}", c);
        }
    }

    // === Smoothing ===

    #[test]
    fn test_sg_smooth_5() {
        let data: Vec<f64> = vec![1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0];
        let smoothed = sg_smooth_5(&data);
        assert_eq!(smoothed.len(), data.len());
        // The spike at index 2 should be reduced
        assert!(smoothed[2] < data[2]);
    }

    #[test]
    fn test_sg_smooth_7() {
        let data: Vec<f64> = vec![1.0, 1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        let smoothed = sg_smooth_7(&data);
        assert_eq!(smoothed.len(), data.len());
        assert!(smoothed[3] < data[3]);
    }

    #[test]
    fn test_sg_smooth_5_short() {
        let data: Vec<f64> = vec![1.0, 2.0, 3.0];
        let smoothed = sg_smooth_5(&data);
        assert_eq!(smoothed, data); // Too short, returned as-is
    }

    // === Peak Deconvolution ===

    #[test]
    fn test_gaussian_peak() {
        let y: f64 = gaussian_peak(0.0, 1.0, 0.0, 1.0);
        assert!(approx(y, 1.0, TOL));
    }

    #[test]
    fn test_gaussian_peak_offset() {
        let y: f64 = gaussian_peak(1.0, 1.0, 1.0, 0.5);
        assert!(approx(y, 1.0, TOL));
    }

    #[test]
    fn test_deconvolve_single_peak() {
        let e: Vec<f64> = (0..100).map(|i| i as f64 * 0.01).collect();
        let i: Vec<f64> = e.iter().map(|&x| gaussian_peak(x, 5.0, 0.5, 0.1)).collect();
        let peaks = deconvolve_peaks(&e, &i, 1);
        assert_eq!(peaks.len(), 1);
        let (amp, center, _sigma) = peaks[0];
        assert!(amp > 2.0, "amplitude = {}", amp);
        assert!((center - 0.5).abs() < 0.2, "center = {}", center);
    }

    #[test]
    fn test_deconvolve_no_peaks() {
        let peaks = deconvolve_peaks(&[], &[], 0);
        assert!(peaks.is_empty());
    }

    // === Processor ===

    #[test]
    fn test_processor_default() {
        let proc = PolarographyProcessor::default_processor();
        assert!(approx(proc.config.temperature_k, 298.15, 0.01));
        assert_eq!(proc.config.n_electrons, 1);
    }

    #[test]
    fn test_processor_lsv() {
        let e: Vec<f64> = (0..50).map(|i| -0.3 + i as f64 * 0.01).collect();
        let i: Vec<f64> = e.iter().map(|&x| 1e-4 * (-(x * 10.0).powi(2)).exp()).collect();
        let sweep = VoltammetricSweep::new(e, i);
        let proc = PolarographyProcessor::default_processor();
        assert!(proc.analyze_lsv(&sweep).is_some());
    }

    #[test]
    fn test_processor_randles_sevcik() {
        let proc = PolarographyProcessor::new(PolarographyConfig {
            n_electrons: 1,
            electrode_area_cm2: 0.07,
            concentration_mol_cm3: 1e-6,
            ..PolarographyConfig::default()
        });
        let d = proc.diffusion_from_randles_sevcik(1e-5, 0.1);
        assert!(d.d_cm2_s > 0.0);
        assert_eq!(d.method, "Randles-Sevcik");
    }

    // === Numerical derivative ===

    #[test]
    fn test_numerical_derivative_linear() {
        let x: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = vec![0.0, 2.0, 4.0, 6.0, 8.0];
        let d = numerical_derivative(&x, &y);
        assert_eq!(d.len(), 5);
        for &di in &d {
            assert!(approx(di, 2.0, 0.01));
        }
    }

    #[test]
    fn test_numerical_derivative_quadratic() {
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| xi * xi).collect();
        let d = numerical_derivative(&x, &y);
        // dy/dx = 2x, at x=5: d'≈10
        assert!(approx(d[5], 10.0, 0.5));
    }

    // === Linear regression ===

    #[test]
    fn test_linear_regression_perfect() {
        let x: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y: Vec<f64> = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(approx(slope, 2.0, TOL));
        assert!(approx(intercept, 0.0, TOL));
        assert!(approx(r2, 1.0, TOL));
    }

    #[test]
    fn test_linear_regression_offset() {
        let x: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0];
        let y: Vec<f64> = vec![5.0, 7.0, 9.0, 11.0];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(approx(slope, 2.0, TOL));
        assert!(approx(intercept, 5.0, TOL));
        assert!(approx(r2, 1.0, TOL));
    }

    // === Polyfit ===

    #[test]
    fn test_polyfit_linear() {
        let x: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0];
        let y: Vec<f64> = vec![1.0, 3.0, 5.0, 7.0];
        let c = polyfit(&x, &y, 1);
        assert_eq!(c.len(), 2);
        assert!(approx(c[0], 1.0, 0.01));
        assert!(approx(c[1], 2.0, 0.01));
    }

    #[test]
    fn test_polyfit_quadratic() {
        let x: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|&xi| 1.0 + 2.0 * xi + 3.0 * xi * xi).collect();
        let c = polyfit(&x, &y, 2);
        assert!(approx(c[0], 1.0, 0.01));
        assert!(approx(c[1], 2.0, 0.01));
        assert!(approx(c[2], 3.0, 0.01));
    }

    #[test]
    fn test_polyeval() {
        let c: Vec<f64> = vec![1.0, 2.0, 3.0]; // 1 + 2x + 3x²
        assert!(approx(polyeval(&c, 0.0), 1.0, TOL));
        assert!(approx(polyeval(&c, 1.0), 6.0, TOL));
        assert!(approx(polyeval(&c, 2.0), 17.0, TOL));
    }

    // === Constants ===

    #[test]
    fn test_thermal_voltage() {
        // VT ≈ 25.7 mV at 25°C
        assert!(VT > 0.025 && VT < 0.026, "VT = {}", VT);
    }

    #[test]
    fn test_faraday_constant() {
        assert!((F_CONST - 96485.0).abs() < 1.0);
    }

    // === Half-peak width ===

    #[test]
    fn test_half_peak_width() {
        let e: Vec<f64> = (0..100).map(|i| i as f64 * 0.01).collect();
        let i: Vec<f64> = e.iter().map(|&x| gaussian_peak(x, 1.0, 0.5, 0.1)).collect();
        let sweep = VoltammetricSweep::new(e, i);
        let hw = half_peak_width(&sweep, 50);
        assert!(hw.is_some());
        // FWHM of Gaussian: 2*sqrt(2*ln(2))*sigma ≈ 2.355*sigma ≈ 0.2355
        let fwhm: f64 = hw.unwrap();
        assert!(approx(fwhm, 0.2355, 0.02), "FWHM = {}", fwhm);
    }

    // === Heyrovsky-Ilkovic ===

    #[test]
    fn test_heyrovsky_ilkovic_one_electron() {
        let n_pts: usize = 100;
        let id: f64 = 5e-6;
        let e_half: f64 = -0.5;
        let e: Vec<f64> = (0..n_pts).map(|i| -0.2 - i as f64 * 0.008).collect();
        let i: Vec<f64> = e.iter().map(|&x| {
            id / (1.0 + ((x - e_half) / VT).exp())
        }).collect();
        let sweep = VoltammetricSweep::new(e, i);
        let n_e: f64 = heyrovsky_ilkovic_n(&sweep, id, 0.0);
        assert!(n_e >= 0.5 && n_e <= 1.5, "n = {}", n_e);
    }

    // === Edge cases ===

    #[test]
    fn test_empty_sweep_analysis() {
        let sweep = VoltammetricSweep::new(vec![], vec![]);
        assert!(linear_sweep_analyze(&sweep).is_none());
    }

    #[test]
    fn test_randles_sevcik_zero_area() {
        let d: f64 = randles_sevcik(1e-5, 1, 0.0, 1e-6, 0.1);
        assert!(d == 0.0 || d.is_finite());
    }

    #[test]
    fn test_butler_volmer_temperature_dependence() {
        let i_300: f64 = butler_volmer(1e-3, 0.5, 1.0, 0.1, 300.0);
        let i_350: f64 = butler_volmer(1e-3, 0.5, 1.0, 0.1, 350.0);
        // Higher temperature → less current (reduced driving force relative to thermal energy)
        assert!(i_300 > i_350, "i_300={}, i_350={}", i_300, i_350);
    }

    #[test]
    fn test_nernst_zero_ratio() {
        let e: f64 = nernst_std(0.5, 1, 0.0);
        assert_eq!(e, 0.5); // Falls through to e0
    }

    #[test]
    fn test_cv_analysis_returns_formal_potential() {
        let fwd = VoltammetricSweep::new(
            vec![0.0, 0.1, 0.2, 0.3, 0.4],
            vec![0.0, 0.5, 1.0, 0.5, 0.0],
        );
        let rev = VoltammetricSweep::new(
            vec![0.4, 0.3, 0.2, 0.1, 0.0],
            vec![0.0, -0.5, -1.0, -0.5, 0.0],
        );
        let result = cyclic_voltammetry_analyze(&fwd, &rev, 1);
        assert!(result.is_some());
        let cv = result.unwrap();
        assert!(cv.e_formal > 0.0);
    }

    #[test]
    fn test_config_default() {
        let cfg = PolarographyConfig::default();
        assert_eq!(cfg.n_electrons, 1);
        assert!(cfg.electrode_area_cm2 > 0.0);
    }

    #[test]
    fn test_levich_current_zero_omega() {
        let il: f64 = levich_current(1, 0.07, 1e-5, 0.0, 0.01, 1e-6);
        assert!(approx(il, 0.0, TOL));
    }

    #[test]
    fn test_ilkovic_n_proportional() {
        let i1: f64 = ilkovic_current(1, 1e-5, 1.0, 1.0, 1.0);
        let i3: f64 = ilkovic_current(3, 1e-5, 1.0, 1.0, 1.0);
        assert!(approx(i3 / i1, 3.0, 0.01));
    }

    #[test]
    fn test_cottrell_decreasing_with_time() {
        let i1: f64 = cottrell_current(1, 0.07, 1e-5, 1e-6, 1.0);
        let i4: f64 = cottrell_current(1, 0.07, 1e-5, 1e-6, 4.0);
        assert!(i1 > i4); // Current decreases as t^(-1/2)
        assert!(approx(i1 / i4, 2.0, 0.01));
    }

    #[test]
    fn test_koutecky_levich_with_kinetic_limit() {
        // Mixed kinetic-diffusion case: 1/i = 1/ik + 1/(B*sqrt(w))
        let ik: f64 = 1e-4;
        let b: f64 = 5e-6;
        let omegas: Vec<f64> = vec![100.0, 200.0, 400.0, 800.0, 1600.0];
        let currents: Vec<f64> = omegas.iter().map(|&w| {
            1.0 / (1.0 / ik + 1.0 / (b * w.sqrt()))
        }).collect();
        let result = koutecky_levich_analysis(&currents, &omegas);
        assert!(result.is_some());
        let kl = result.unwrap();
        assert!(kl.r_squared > 0.99);
        assert!((kl.i_kinetic - ik).abs() / ik < 0.1, "ik = {}", kl.i_kinetic);
    }

    #[test]
    fn test_sg_smooth_5_flat() {
        // Flat data should be unchanged
        let data: Vec<f64> = vec![5.0; 10];
        let smoothed = sg_smooth_5(&data);
        for (i, &s) in smoothed.iter().enumerate() {
            assert!(approx(s, 5.0, 0.01), "index {} = {}", i, s);
        }
    }

    #[test]
    fn test_sg_smooth_7_flat() {
        let data: Vec<f64> = vec![3.0; 12];
        let smoothed = sg_smooth_7(&data);
        for (i, &s) in smoothed.iter().enumerate() {
            assert!(approx(s, 3.0, 0.01), "index {} = {}", i, s);
        }
    }

    #[test]
    fn test_baseline_empty() {
        let sweep = VoltammetricSweep::new(vec![0.0], vec![5.0]);
        let corrected = baseline_correct(&sweep, BaselineMethod::Linear);
        assert_eq!(corrected.len(), 1);
    }

    #[test]
    fn test_gaussian_peak_at_distance() {
        let y: f64 = gaussian_peak(3.0, 1.0, 0.0, 1.0);
        let expected: f64 = (-4.5_f64).exp();
        assert!(approx(y, expected, TOL));
    }

    #[test]
    fn test_processor_cv() {
        let fwd = VoltammetricSweep::new(
            vec![0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
            vec![0.0, 0.2, 0.8, 0.5, 0.2, 0.0],
        );
        let rev = VoltammetricSweep::new(
            vec![0.5, 0.4, 0.3, 0.2, 0.1, 0.0],
            vec![0.0, -0.2, -0.8, -0.5, -0.2, 0.0],
        );
        let proc = PolarographyProcessor::default_processor();
        let result = proc.analyze_cv(&fwd, &rev);
        assert!(result.is_some());
    }

    #[test]
    fn test_swv_net_current() {
        let fwd: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let rev: Vec<f64> = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let pot: Vec<f64> = vec![0.0, 0.1, 0.2, 0.3, 0.4];
        let result = square_wave_analyze(&fwd, &rev, &pot).unwrap();
        assert_eq!(result.net_current.len(), 5);
        assert!(approx(result.net_current[0], 0.5, TOL));
        assert!(approx(result.net_current[4], 2.5, TOL));
    }

    #[test]
    fn test_randles_sevcik_proportional_to_concentration() {
        let ip1: f64 = randles_sevcik_ip(1, 0.07, 1e-5, 1e-6, 0.1);
        let ip2: f64 = randles_sevcik_ip(1, 0.07, 1e-5, 2e-6, 0.1);
        assert!(approx(ip2 / ip1, 2.0, 0.01));
    }

    #[test]
    fn test_butler_volmer_large_cathodic() {
        // Large cathodic: i ≈ -i0 * exp((1-alpha)*n*F*|eta|/(RT))
        let i: f64 = butler_volmer_std(1e-3, 0.5, 1.0, -0.5);
        assert!(i < 0.0);
    }
}

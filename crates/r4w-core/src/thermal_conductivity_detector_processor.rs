//! # Thermal Conductivity Detector (TCD) Signal Processor
//!
//! Signal processing module for gas chromatography thermal conductivity detectors
//! (katharometers). Implements the full TCD signal processing chain from raw
//! Wheatstone bridge measurements to gas identification.
//!
//! ## Overview
//!
//! A TCD measures differences in thermal conductivity between a carrier gas and
//! sample gas components as they elute from a GC column. The detector uses a heated
//! filament (or thermistor) in a Wheatstone bridge configuration. When a sample gas
//! with different thermal conductivity passes over the sensing filament, the bridge
//! becomes unbalanced, producing a measurable voltage signal.
//!
//! ## Processing Pipeline
//!
//! ```text
//! Raw Bridge Signal → Wheatstone Conditioning → Baseline Drift Correction
//!     → Peak Detection → Peak Integration → Thermal Conductivity Calculation
//!     → Gas Identification
//! ```
//!
//! ## Key Features
//!
//! - **Wheatstone bridge conditioning**: Balance, offset removal, amplification
//! - **Baseline drift correction**: Polynomial fitting and subtraction for long GC runs
//! - **Peak detection**: Onset, apex, and end detection with slope-based thresholds
//! - **Peak integration**: Trapezoidal and Simpson's 1/3 rule for area calculation
//! - **Gaussian/EMG fitting**: Least-squares peak shape analysis
//! - **Thermal conductivity calculation**: Convert bridge voltage to W/(m·K)
//! - **Gas identification**: Match measured TC values against reference database
//! - **Response factor tables**: Carrier gas correction factors (He, Ar, N2)
//! - **Sensitivity and noise analysis**: SNR, detection limits per ASTM E685
//!
//! ## Physical Constants
//!
//! Thermal conductivity values at 25°C, 1 atm in W/(m·K):
//!
//! | Gas           | TC (W/(m·K)) |
//! |---------------|-------------|
//! | Helium        | 0.1513      |
//! | Hydrogen      | 0.1805      |
//! | Nitrogen      | 0.0259      |
//! | Argon         | 0.0177      |
//! | Air           | 0.0262      |
//! | CO2           | 0.0166      |
//! | Methane       | 0.0343      |
//! | Water vapor   | 0.0186      |
//!
//! ## References
//!
//! - ASTM E685: Standard Practice for Testing Fixed-Wavelength Photometric Detectors
//!   Used in Liquid Chromatography (adapted for GC-TCD)
//! - David, D. J. "Gas Chromatographic Detectors", Wiley-Interscience, 1974
//! - McNair, H. M. & Miller, J. M. "Basic Gas Chromatography", Wiley, 2009

use std::f64::consts::PI;

// ─── Physical Constants: Thermal Conductivity at 25°C, 1 atm [W/(m·K)] ─────

/// Thermal conductivity of helium at 25°C, 1 atm [W/(m·K)]
pub const TC_HELIUM: f64 = 0.1513;
/// Thermal conductivity of hydrogen at 25°C, 1 atm [W/(m·K)]
pub const TC_HYDROGEN: f64 = 0.1805;
/// Thermal conductivity of nitrogen at 25°C, 1 atm [W/(m·K)]
pub const TC_NITROGEN: f64 = 0.0259;
/// Thermal conductivity of argon at 25°C, 1 atm [W/(m·K)]
pub const TC_ARGON: f64 = 0.0177;
/// Thermal conductivity of air at 25°C, 1 atm [W/(m·K)]
pub const TC_AIR: f64 = 0.0262;
/// Thermal conductivity of carbon dioxide at 25°C, 1 atm [W/(m·K)]
pub const TC_CO2: f64 = 0.0166;
/// Thermal conductivity of methane at 25°C, 1 atm [W/(m·K)]
pub const TC_METHANE: f64 = 0.0343;
/// Thermal conductivity of water vapor at 25°C, 1 atm [W/(m·K)]
pub const TC_WATER_VAPOR: f64 = 0.0186;

// ─── Enumerations ────────────────────────────────────────────────────────────

/// Carrier gas type used in the GC system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CarrierGas {
    Helium,
    Hydrogen,
    Nitrogen,
    Argon,
}

impl CarrierGas {
    /// Return the thermal conductivity of this carrier gas at 25°C [W/(m·K)].
    pub fn thermal_conductivity(&self) -> f64 {
        match self {
            CarrierGas::Helium => TC_HELIUM,
            CarrierGas::Hydrogen => TC_HYDROGEN,
            CarrierGas::Nitrogen => TC_NITROGEN,
            CarrierGas::Argon => TC_ARGON,
        }
    }
}

/// Known gas species in the reference database.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GasSpecies {
    Helium,
    Hydrogen,
    Nitrogen,
    Argon,
    Air,
    CarbonDioxide,
    Methane,
    WaterVapor,
}

impl GasSpecies {
    /// Return thermal conductivity at 25°C [W/(m·K)].
    pub fn thermal_conductivity(&self) -> f64 {
        match self {
            GasSpecies::Helium => TC_HELIUM,
            GasSpecies::Hydrogen => TC_HYDROGEN,
            GasSpecies::Nitrogen => TC_NITROGEN,
            GasSpecies::Argon => TC_ARGON,
            GasSpecies::Air => TC_AIR,
            GasSpecies::CarbonDioxide => TC_CO2,
            GasSpecies::Methane => TC_METHANE,
            GasSpecies::WaterVapor => TC_WATER_VAPOR,
        }
    }

    /// Return the name of this gas species.
    pub fn name(&self) -> &'static str {
        match self {
            GasSpecies::Helium => "Helium",
            GasSpecies::Hydrogen => "Hydrogen",
            GasSpecies::Nitrogen => "Nitrogen",
            GasSpecies::Argon => "Argon",
            GasSpecies::Air => "Air",
            GasSpecies::CarbonDioxide => "Carbon Dioxide",
            GasSpecies::Methane => "Methane",
            GasSpecies::WaterVapor => "Water Vapor",
        }
    }

    /// Return all known gas species.
    pub fn all() -> &'static [GasSpecies] {
        &[
            GasSpecies::Helium,
            GasSpecies::Hydrogen,
            GasSpecies::Nitrogen,
            GasSpecies::Argon,
            GasSpecies::Air,
            GasSpecies::CarbonDioxide,
            GasSpecies::Methane,
            GasSpecies::WaterVapor,
        ]
    }
}

/// Peak integration method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntegrationMethod {
    /// Trapezoidal rule (linear interpolation between points).
    Trapezoidal,
    /// Simpson's 1/3 rule (quadratic interpolation, requires odd number of points).
    Simpsons,
}

/// Peak shape model for fitting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PeakModel {
    /// Symmetric Gaussian: A * exp(-(x - mu)^2 / (2 * sigma^2))
    Gaussian,
    /// Exponentially Modified Gaussian (asymmetric tailing peak)
    ExponentiallyModifiedGaussian,
}

// ─── Configuration ───────────────────────────────────────────────────────────

/// TCD detector configuration.
#[derive(Debug, Clone)]
pub struct TcdConfig {
    /// Bridge current in milliamps (typical: 100-300 mA).
    pub bridge_current_ma: f64,
    /// Carrier gas type.
    pub carrier_gas: CarrierGas,
    /// Cell temperature in °C.
    pub cell_temperature_c: f64,
    /// Carrier gas flow rate in mL/min.
    pub flow_rate_ml_min: f64,
    /// Sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// Supply voltage of the Wheatstone bridge in volts.
    pub bridge_supply_v: f64,
}

impl Default for TcdConfig {
    fn default() -> Self {
        Self {
            bridge_current_ma: 200.0,
            carrier_gas: CarrierGas::Helium,
            cell_temperature_c: 150.0,
            flow_rate_ml_min: 30.0,
            sample_rate_hz: 10.0,
            bridge_supply_v: 10.0,
        }
    }
}

// ─── Standalone Functions ────────────────────────────────────────────────────

/// Compute Wheatstone bridge output voltage.
///
/// ```text
///       R1        R2
///  V+ ──┤├──┬──┤├── GND
///        │  Vout  │
///  V+ ──┤├──┴──┤├── GND
///       R3        R4
/// ```
///
/// V_out = V_supply * (R3/(R1+R3) - R4/(R2+R4))
///
/// # Arguments
/// - `r1`, `r2`, `r3`, `r4` — Bridge resistances in ohms
/// - `v_supply` — Supply voltage in volts
///
/// # Returns
/// Bridge output voltage in volts.
pub fn wheatstone_voltage(r1: f64, r2: f64, r3: f64, r4: f64, v_supply: f64) -> f64 {
    let v_a = v_supply * r3 / (r1 + r3);
    let v_b = v_supply * r4 / (r2 + r4);
    v_a - v_b
}

/// Convert a bridge signal (voltage) to thermal conductivity using a linear calibration.
///
/// TC = calibration_slope * signal + calibration_intercept
///
/// # Arguments
/// - `signal` — Bridge output voltage (V)
/// - `calibration_slope` — Calibration slope [W/(m·K·V)]
/// - `calibration_intercept` — Calibration intercept [W/(m·K)]
///
/// # Returns
/// Thermal conductivity in W/(m·K).
pub fn thermal_conductivity_from_signal(
    signal: f64,
    calibration_slope: f64,
    calibration_intercept: f64,
) -> f64 {
    calibration_slope * signal + calibration_intercept
}

/// Compute the integrated peak area using the trapezoidal rule.
///
/// Area = sum_{i=0}^{N-2} 0.5 * (y[i] + y[i+1]) * dt
///
/// # Arguments
/// - `signal` — Signal values (baseline-corrected peak data)
/// - `dt` — Time step between samples (seconds)
///
/// # Returns
/// Integrated area (signal_units * seconds).
pub fn peak_area_trapezoidal(signal: &[f64], dt: f64) -> f64 {
    if signal.len() < 2 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..signal.len() - 1 {
        area += 0.5 * (signal[i] + signal[i + 1]) * dt;
    }
    area
}

/// Compute the integrated peak area using Simpson's 1/3 rule.
///
/// For N points (N must be odd, i.e., even number of intervals):
/// Area = (dt/3) * [y[0] + 4*y[1] + 2*y[2] + 4*y[3] + ... + 4*y[N-2] + y[N-1]]
///
/// If the number of points is even, the last interval uses the trapezoidal rule.
///
/// # Arguments
/// - `signal` — Signal values (baseline-corrected peak data)
/// - `dt` — Time step between samples (seconds)
///
/// # Returns
/// Integrated area (signal_units * seconds).
pub fn peak_area_simpsons(signal: &[f64], dt: f64) -> f64 {
    if signal.len() < 2 {
        return 0.0;
    }
    if signal.len() == 2 {
        return 0.5 * (signal[0] + signal[1]) * dt;
    }

    let n = signal.len();
    // Number of full Simpson intervals (groups of 2 intervals each)
    let pairs = (n - 1) / 2;
    let simpson_points = pairs * 2 + 1;

    let mut area = 0.0;

    // Simpson's 1/3 rule over complete pairs
    if simpson_points >= 3 {
        area += signal[0];
        for i in (1..simpson_points - 1).step_by(2) {
            area += 4.0 * signal[i];
        }
        for i in (2..simpson_points - 1).step_by(2) {
            area += 2.0 * signal[i];
        }
        area += signal[simpson_points - 1];
        area *= dt / 3.0;
    }

    // If even number of points, add last interval with trapezoidal rule
    if n > simpson_points {
        area += 0.5 * (signal[simpson_points - 1] + signal[n - 1]) * dt;
    }

    area
}

/// Fit a Gaussian curve to data points using iterative least-squares.
///
/// Model: y = A * exp(-(x - mu)^2 / (2 * sigma^2))
///
/// Uses a linearized approach: ln(y) = ln(A) - (x - mu)^2 / (2*sigma^2)
/// which becomes a quadratic fit on log-transformed data.
///
/// # Arguments
/// - `x` — Independent variable values (e.g., time in seconds)
/// - `y` — Dependent variable values (must be positive for log transform)
///
/// # Returns
/// `Some((amplitude, center, sigma))` if fit succeeds, `None` otherwise.
pub fn gaussian_fit(x: &[f64], y: &[f64]) -> Option<(f64, f64, f64)> {
    if x.len() != y.len() || x.len() < 3 {
        return None;
    }

    // Filter out non-positive y values for log transform
    let mut xf = Vec::new();
    let mut lny = Vec::new();
    for i in 0..x.len() {
        if y[i] > 0.0 {
            xf.push(x[i]);
            lny.push(y[i].ln());
        }
    }

    if xf.len() < 3 {
        return None;
    }

    // Fit ln(y) = a + b*x + c*x^2 using normal equations
    // where a = ln(A) - mu^2/(2*sigma^2), b = mu/sigma^2, c = -1/(2*sigma^2)
    let n = xf.len() as f64;
    let sum_x: f64 = xf.iter().sum();
    let sum_x2: f64 = xf.iter().map(|&xi| xi * xi).sum();
    let sum_x3: f64 = xf.iter().map(|&xi| xi * xi * xi).sum();
    let sum_x4: f64 = xf.iter().map(|&xi| xi * xi * xi * xi).sum();
    let sum_y: f64 = lny.iter().sum();
    let sum_xy: f64 = xf.iter().zip(lny.iter()).map(|(&xi, &yi)| xi * yi).sum();
    let sum_x2y: f64 = xf
        .iter()
        .zip(lny.iter())
        .map(|(&xi, &yi)| xi * xi * yi)
        .sum();

    // Solve 3x3 system via Cramer's rule:
    // | n      sum_x   sum_x2 | | a |   | sum_y   |
    // | sum_x  sum_x2  sum_x3 | | b | = | sum_xy  |
    // | sum_x2 sum_x3  sum_x4 | | c |   | sum_x2y |

    let det = n * (sum_x2 * sum_x4 - sum_x3 * sum_x3)
        - sum_x * (sum_x * sum_x4 - sum_x3 * sum_x2)
        + sum_x2 * (sum_x * sum_x3 - sum_x2 * sum_x2);

    if det.abs() < 1e-30 {
        return None;
    }

    let det_a = sum_y * (sum_x2 * sum_x4 - sum_x3 * sum_x3)
        - sum_x * (sum_xy * sum_x4 - sum_x3 * sum_x2y)
        + sum_x2 * (sum_xy * sum_x3 - sum_x2 * sum_x2y);

    let det_b = n * (sum_xy * sum_x4 - sum_x3 * sum_x2y)
        - sum_y * (sum_x * sum_x4 - sum_x3 * sum_x2)
        + sum_x2 * (sum_x * sum_x2y - sum_xy * sum_x2);

    let det_c = n * (sum_x2 * sum_x2y - sum_xy * sum_x3)
        - sum_x * (sum_x * sum_x2y - sum_xy * sum_x2)
        + sum_y * (sum_x * sum_x3 - sum_x2 * sum_x2);

    let a = det_a / det;
    let b = det_b / det;
    let c = det_c / det;

    // c must be negative for a valid Gaussian
    if c >= 0.0 {
        return None;
    }

    let sigma = (-1.0 / (2.0 * c)).sqrt();
    let mu = b * sigma * sigma;
    let amplitude = (a + mu * mu / (2.0 * sigma * sigma)).exp();

    if amplitude.is_finite() && mu.is_finite() && sigma.is_finite() && sigma > 0.0 {
        Some((amplitude, mu, sigma))
    } else {
        None
    }
}

/// Compute signal-to-noise ratio.
///
/// SNR = peak_height / noise_rms
///
/// # Arguments
/// - `peak_height` — Peak signal amplitude above baseline
/// - `noise_rms` — RMS noise level of the baseline
///
/// # Returns
/// SNR (dimensionless ratio).
pub fn signal_to_noise(peak_height: f64, noise_rms: f64) -> f64 {
    if noise_rms <= 0.0 {
        return f64::INFINITY;
    }
    peak_height / noise_rms
}

/// Compute the minimum detectable quantity (detection limit).
///
/// Based on ASTM E685 / ICH guidelines:
/// LOD = (confidence * noise_rms) / slope
///
/// Typical `confidence` values:
/// - 3.0 for detection limit (LOD)
/// - 10.0 for quantitation limit (LOQ)
///
/// # Arguments
/// - `noise_rms` — RMS noise level of the baseline
/// - `slope` — Calibration curve slope (response per unit concentration)
/// - `confidence` — Confidence multiplier (3.0 for LOD, 10.0 for LOQ)
///
/// # Returns
/// Minimum detectable quantity in concentration units.
pub fn detection_limit(noise_rms: f64, slope: f64, confidence: f64) -> f64 {
    if slope.abs() < 1e-30 {
        return f64::INFINITY;
    }
    confidence * noise_rms / slope.abs()
}

/// Compute the RMS noise level of a signal segment.
///
/// RMS = sqrt(mean(x^2 - mean(x)^2))
///
/// # Arguments
/// - `signal` — Signal values (should be a baseline region)
///
/// # Returns
/// RMS noise level.
pub fn noise_rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let n = signal.len() as f64;
    let mean = signal.iter().sum::<f64>() / n;
    let variance = signal.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / n;
    variance.sqrt()
}

/// Compute the relative response factor of a sample gas vs. carrier gas.
///
/// RF = (TC_carrier - TC_sample) / TC_carrier
///
/// # Arguments
/// - `tc_carrier` — Thermal conductivity of the carrier gas [W/(m·K)]
/// - `tc_sample` — Thermal conductivity of the sample gas [W/(m·K)]
///
/// # Returns
/// Relative response factor (dimensionless).
pub fn relative_response_factor(tc_carrier: f64, tc_sample: f64) -> f64 {
    if tc_carrier.abs() < 1e-30 {
        return 0.0;
    }
    (tc_carrier - tc_sample) / tc_carrier
}

/// Apply temperature correction to thermal conductivity.
///
/// TC(T) = TC(T_ref) * (T / T_ref)^n
///
/// where n is approximately 0.7-0.8 for most gases.
///
/// # Arguments
/// - `tc_ref` — TC at reference temperature [W/(m·K)]
/// - `t_ref_k` — Reference temperature [K]
/// - `t_k` — Target temperature [K]
/// - `exponent` — Temperature exponent (typically 0.75)
///
/// # Returns
/// Temperature-corrected thermal conductivity [W/(m·K)].
pub fn temperature_correct_tc(tc_ref: f64, t_ref_k: f64, t_k: f64, exponent: f64) -> f64 {
    if t_ref_k <= 0.0 {
        return tc_ref;
    }
    tc_ref * (t_k / t_ref_k).powf(exponent)
}

// ─── Wheatstone Bridge Conditioner ───────────────────────────────────────────

/// Wheatstone bridge signal conditioner.
///
/// Performs zero offset removal, gain adjustment, and optional low-pass filtering
/// of the raw bridge output voltage.
#[derive(Debug, Clone)]
pub struct WheatstoneConditioner {
    /// Zero offset voltage to subtract (V).
    pub zero_offset: f64,
    /// Amplifier gain (dimensionless).
    pub gain: f64,
    /// Reference resistance values (R1, R2, R3, R4) in ohms.
    pub reference_resistances: [f64; 4],
    /// Supply voltage (V).
    pub supply_voltage: f64,
    /// Optional single-pole IIR low-pass filter coefficient (0..1, smaller = more filtering).
    pub lpf_alpha: Option<f64>,
    /// Low-pass filter state.
    lpf_state: f64,
    /// Whether the filter state has been initialized.
    lpf_initialized: bool,
}

impl WheatstoneConditioner {
    /// Create a new Wheatstone bridge conditioner.
    ///
    /// # Arguments
    /// - `supply_voltage` — Bridge supply voltage (V)
    /// - `gain` — Amplifier gain
    pub fn new(supply_voltage: f64, gain: f64) -> Self {
        Self {
            zero_offset: 0.0,
            gain,
            reference_resistances: [100.0, 100.0, 100.0, 100.0],
            supply_voltage,
            lpf_alpha: None,
            lpf_state: 0.0,
            lpf_initialized: false,
        }
    }

    /// Set the zero offset from a balanced bridge reading.
    pub fn set_zero_offset(&mut self, offset: f64) {
        self.zero_offset = offset;
    }

    /// Auto-zero by computing the offset from reference resistances.
    pub fn auto_zero(&mut self) {
        let r = &self.reference_resistances;
        self.zero_offset = wheatstone_voltage(r[0], r[1], r[2], r[3], self.supply_voltage);
    }

    /// Enable low-pass filtering with given alpha coefficient.
    /// alpha = dt / (RC + dt), where smaller alpha = more smoothing.
    pub fn set_lowpass(&mut self, alpha: f64) {
        self.lpf_alpha = Some(alpha.clamp(0.0, 1.0));
        self.lpf_initialized = false;
    }

    /// Condition a single raw bridge voltage sample.
    ///
    /// Processing: (raw - zero_offset) * gain → optional LPF
    pub fn condition(&mut self, raw_voltage: f64) -> f64 {
        let mut signal = (raw_voltage - self.zero_offset) * self.gain;

        if let Some(alpha) = self.lpf_alpha {
            if !self.lpf_initialized {
                self.lpf_state = signal;
                self.lpf_initialized = true;
            } else {
                self.lpf_state = alpha * signal + (1.0 - alpha) * self.lpf_state;
            }
            signal = self.lpf_state;
        }

        signal
    }

    /// Condition a batch of raw voltage samples.
    pub fn condition_batch(&mut self, raw: &[f64]) -> Vec<f64> {
        raw.iter().map(|&v| self.condition(v)).collect()
    }

    /// Reset the conditioner state.
    pub fn reset(&mut self) {
        self.lpf_state = 0.0;
        self.lpf_initialized = false;
    }
}

// ─── Baseline Drift Corrector ────────────────────────────────────────────────

/// Polynomial baseline drift corrector.
///
/// Fits a polynomial of configurable degree to user-specified baseline anchor
/// points and subtracts it from the signal. Used to remove slow drift caused by
/// temperature changes, column bleed, or detector aging during long GC runs.
#[derive(Debug, Clone)]
pub struct BaselineDriftCorrector {
    /// Polynomial degree for baseline fitting (0=constant, 1=linear, 2=quadratic, etc.).
    pub degree: usize,
}

impl BaselineDriftCorrector {
    /// Create a new baseline drift corrector with the specified polynomial degree.
    pub fn new(degree: usize) -> Self {
        Self { degree }
    }

    /// Fit a polynomial baseline to the entire signal using least-squares.
    ///
    /// Fits polynomial p(t) = c0 + c1*t + c2*t^2 + ... to the given signal,
    /// interpreting indices as time coordinates.
    ///
    /// # Arguments
    /// - `signal` — Full signal vector
    ///
    /// # Returns
    /// Polynomial coefficients [c0, c1, ..., c_degree].
    pub fn fit_baseline(&self, signal: &[f64]) -> Vec<f64> {
        if signal.is_empty() {
            return vec![0.0; self.degree + 1];
        }
        let n = signal.len();
        let x: Vec<f64> = (0..n).map(|i| i as f64).collect();
        polyfit(&x, signal, self.degree)
    }

    /// Fit a polynomial baseline using only specified anchor point indices.
    ///
    /// # Arguments
    /// - `signal` — Full signal vector
    /// - `anchor_indices` — Indices of baseline anchor points (known baseline regions)
    ///
    /// # Returns
    /// Polynomial coefficients [c0, c1, ..., c_degree].
    pub fn fit_baseline_anchored(&self, signal: &[f64], anchor_indices: &[usize]) -> Vec<f64> {
        if anchor_indices.is_empty() {
            return vec![0.0; self.degree + 1];
        }
        let x: Vec<f64> = anchor_indices.iter().map(|&i| i as f64).collect();
        let y: Vec<f64> = anchor_indices
            .iter()
            .map(|&i| if i < signal.len() { signal[i] } else { 0.0 })
            .collect();
        polyfit(&x, &y, self.degree)
    }

    /// Subtract a polynomial baseline from the signal.
    ///
    /// # Arguments
    /// - `signal` — Signal to correct
    /// - `coefficients` — Polynomial coefficients from `fit_baseline`
    ///
    /// # Returns
    /// Baseline-corrected signal.
    pub fn subtract_baseline(&self, signal: &[f64], coefficients: &[f64]) -> Vec<f64> {
        signal
            .iter()
            .enumerate()
            .map(|(i, &s)| s - polyeval(coefficients, i as f64))
            .collect()
    }

    /// Perform full baseline correction: fit and subtract in one step.
    pub fn correct(&self, signal: &[f64]) -> Vec<f64> {
        let coeffs = self.fit_baseline(signal);
        self.subtract_baseline(signal, &coeffs)
    }

    /// Perform anchored baseline correction: fit to anchor points and subtract.
    pub fn correct_anchored(&self, signal: &[f64], anchor_indices: &[usize]) -> Vec<f64> {
        let coeffs = self.fit_baseline_anchored(signal, anchor_indices);
        self.subtract_baseline(signal, &coeffs)
    }
}

/// Evaluate a polynomial at x: p(x) = c0 + c1*x + c2*x^2 + ...
fn polyeval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut x_pow = 1.0;
    for &c in coeffs {
        result += c * x_pow;
        x_pow *= x;
    }
    result
}

/// Fit a polynomial of given degree to (x, y) data using normal equations.
///
/// Solves the Vandermonde system A^T A c = A^T y where A is the Vandermonde matrix.
fn polyfit(x: &[f64], y: &[f64], degree: usize) -> Vec<f64> {
    let n = x.len();
    let m = degree + 1;

    if n < m {
        return vec![0.0; m];
    }

    // Build A^T A (m x m) and A^T y (m x 1)
    let mut ata = vec![vec![0.0; m]; m];
    let mut aty = vec![0.0; m];

    for k in 0..n {
        let mut xi_pow = vec![1.0; 2 * m];
        for p in 1..2 * m {
            xi_pow[p] = xi_pow[p - 1] * x[k];
        }
        for i in 0..m {
            for j in 0..m {
                ata[i][j] += xi_pow[i + j];
            }
            aty[i] += xi_pow[i] * y[k];
        }
    }

    // Solve using Gaussian elimination with partial pivoting
    gauss_solve(&mut ata, &mut aty)
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
fn gauss_solve(a: &mut Vec<Vec<f64>>, b: &mut Vec<f64>) -> Vec<f64> {
    let n = b.len();
    if n == 0 {
        return vec![];
    }

    // Forward elimination
    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in col + 1..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            continue;
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot = a[col][col];
        for row in col + 1..n {
            let factor = a[row][col] / pivot;
            for j in col..n {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        if a[i][i].abs() < 1e-30 {
            x[i] = 0.0;
            continue;
        }
        let mut sum = b[i];
        for j in i + 1..n {
            sum -= a[i][j] * x[j];
        }
        x[i] = sum / a[i][i];
    }
    x
}

// ─── Peak Detector ───────────────────────────────────────────────────────────

/// Detected chromatographic peak.
#[derive(Debug, Clone)]
pub struct DetectedPeak {
    /// Index of peak onset (start).
    pub onset_index: usize,
    /// Index of peak apex (maximum).
    pub apex_index: usize,
    /// Index of peak end.
    pub end_index: usize,
    /// Peak height above baseline at apex.
    pub height: f64,
    /// Width at half maximum (in sample indices).
    pub fwhm: f64,
    /// Asymmetry factor (tailing ratio at 10% height).
    pub asymmetry: f64,
}

/// Chromatographic peak detector using slope-based detection.
///
/// Detects peaks by monitoring the first derivative (slope) of the signal:
/// 1. **Onset**: slope exceeds positive threshold
/// 2. **Apex**: slope crosses zero (positive to negative)
/// 3. **End**: slope returns to within threshold of zero
#[derive(Debug, Clone)]
pub struct PeakDetector {
    /// Minimum slope threshold for onset detection.
    pub slope_threshold: f64,
    /// Minimum peak height to report.
    pub min_height: f64,
    /// Minimum peak width (in samples) to report.
    pub min_width: usize,
}

impl PeakDetector {
    /// Create a new peak detector.
    ///
    /// # Arguments
    /// - `slope_threshold` — Minimum slope for peak onset detection
    /// - `min_height` — Minimum peak height to report
    /// - `min_width` — Minimum peak width in samples
    pub fn new(slope_threshold: f64, min_height: f64, min_width: usize) -> Self {
        Self {
            slope_threshold,
            min_height,
            min_width,
        }
    }

    /// Detect peaks in a baseline-corrected signal.
    ///
    /// # Arguments
    /// - `signal` — Baseline-corrected signal
    ///
    /// # Returns
    /// Vector of detected peaks.
    pub fn detect(&self, signal: &[f64]) -> Vec<DetectedPeak> {
        if signal.len() < 3 {
            return vec![];
        }

        let mut peaks = Vec::new();

        // Compute first derivative (slope)
        let mut slopes = vec![0.0; signal.len()];
        for i in 1..signal.len() {
            slopes[i] = signal[i] - signal[i - 1];
        }

        #[derive(PartialEq)]
        enum State {
            Baseline,
            Rising,
            Falling,
        }

        let mut state = State::Baseline;
        let mut onset = 0usize;
        let mut apex = 0usize;
        let mut apex_val = 0.0f64;

        for i in 1..signal.len() {
            match state {
                State::Baseline => {
                    if slopes[i] > self.slope_threshold {
                        onset = i - 1;
                        state = State::Rising;
                    }
                }
                State::Rising => {
                    if signal[i] > apex_val {
                        apex = i;
                        apex_val = signal[i];
                    }
                    if slopes[i] <= 0.0 {
                        state = State::Falling;
                    }
                }
                State::Falling => {
                    if slopes[i].abs() < self.slope_threshold || i == signal.len() - 1 {
                        let end = i;
                        let width = end - onset;
                        let height = apex_val;

                        if height >= self.min_height && width >= self.min_width {
                            let fwhm = compute_fwhm(signal, onset, apex, end);
                            let asymmetry = compute_asymmetry(signal, onset, apex, end);
                            peaks.push(DetectedPeak {
                                onset_index: onset,
                                apex_index: apex,
                                end_index: end,
                                height,
                                fwhm,
                                asymmetry,
                            });
                        }

                        state = State::Baseline;
                        apex_val = 0.0;
                    }
                }
            }
        }

        // Handle peak that extends to end of signal
        if state == State::Falling || state == State::Rising {
            let end = signal.len() - 1;
            let width = end - onset;
            if apex_val >= self.min_height && width >= self.min_width {
                let fwhm = compute_fwhm(signal, onset, apex, end);
                let asymmetry = compute_asymmetry(signal, onset, apex, end);
                peaks.push(DetectedPeak {
                    onset_index: onset,
                    apex_index: apex,
                    end_index: end,
                    height: apex_val,
                    fwhm,
                    asymmetry,
                });
            }
        }

        peaks
    }
}

/// Compute full width at half maximum from peak data.
fn compute_fwhm(signal: &[f64], onset: usize, apex: usize, end: usize) -> f64 {
    let half_height = signal[apex] / 2.0;

    // Find left half-height crossing
    let mut left = onset as f64;
    for i in onset..apex {
        if signal[i] <= half_height && signal[i + 1] > half_height {
            // Linear interpolation
            let frac = (half_height - signal[i]) / (signal[i + 1] - signal[i]);
            left = i as f64 + frac;
            break;
        }
    }

    // Find right half-height crossing
    let mut right = end as f64;
    for i in apex..end {
        if signal[i] >= half_height && i + 1 < signal.len() && signal[i + 1] < half_height {
            let frac = (signal[i] - half_height) / (signal[i] - signal[i + 1]);
            right = i as f64 + frac;
            break;
        }
    }

    right - left
}

/// Compute peak asymmetry factor (tailing factor at 10% height).
///
/// Asymmetry = B / A where A is front width and B is back width at 10% height.
fn compute_asymmetry(signal: &[f64], onset: usize, apex: usize, end: usize) -> f64 {
    let threshold = signal[apex] * 0.1;

    let mut left = onset as f64;
    for i in onset..apex {
        if signal[i] <= threshold && i + 1 < signal.len() && signal[i + 1] > threshold {
            let frac = (threshold - signal[i]) / (signal[i + 1] - signal[i]);
            left = i as f64 + frac;
            break;
        }
    }

    let mut right = end as f64;
    for i in apex..end {
        if signal[i] >= threshold && i + 1 < signal.len() && signal[i + 1] < threshold {
            let frac = (signal[i] - threshold) / (signal[i] - signal[i + 1]);
            right = i as f64 + frac;
            break;
        }
    }

    let a = apex as f64 - left;
    let b = right - apex as f64;

    if a.abs() < 1e-10 {
        return 1.0;
    }
    b / a
}

// ─── Peak Integrator ─────────────────────────────────────────────────────────

/// Chromatographic peak integrator.
///
/// Computes peak area using trapezoidal or Simpson's rule on baseline-corrected
/// signal data extracted between detected peak boundaries.
#[derive(Debug, Clone)]
pub struct PeakIntegrator {
    /// Integration method to use.
    pub method: IntegrationMethod,
    /// Time step between samples (seconds).
    pub dt: f64,
}

impl PeakIntegrator {
    /// Create a new peak integrator.
    ///
    /// # Arguments
    /// - `method` — Integration method (Trapezoidal or Simpsons)
    /// - `dt` — Time step between samples (seconds)
    pub fn new(method: IntegrationMethod, dt: f64) -> Self {
        Self { method, dt }
    }

    /// Integrate a single detected peak.
    ///
    /// Extracts the signal region between onset and end indices,
    /// then integrates using the configured method.
    ///
    /// # Returns
    /// Peak area in (signal_units * seconds).
    pub fn integrate_peak(&self, signal: &[f64], peak: &DetectedPeak) -> f64 {
        let start = peak.onset_index;
        let end = (peak.end_index + 1).min(signal.len());
        if end <= start {
            return 0.0;
        }
        let segment = &signal[start..end];
        match self.method {
            IntegrationMethod::Trapezoidal => peak_area_trapezoidal(segment, self.dt),
            IntegrationMethod::Simpsons => peak_area_simpsons(segment, self.dt),
        }
    }

    /// Integrate all detected peaks in a signal.
    ///
    /// # Returns
    /// Vector of (peak_index, area) tuples.
    pub fn integrate_all(&self, signal: &[f64], peaks: &[DetectedPeak]) -> Vec<(usize, f64)> {
        peaks
            .iter()
            .enumerate()
            .map(|(i, p)| (i, self.integrate_peak(signal, p)))
            .collect()
    }

    /// Compute peak area with baseline interpolation between onset and end.
    ///
    /// Uses a linear baseline between the signal values at onset and end,
    /// then integrates the signal above this interpolated baseline.
    pub fn integrate_peak_with_baseline(&self, signal: &[f64], peak: &DetectedPeak) -> f64 {
        let start = peak.onset_index;
        let end = peak.end_index.min(signal.len() - 1);
        if end <= start {
            return 0.0;
        }

        let n = end - start + 1;
        let y_start = signal[start];
        let y_end = signal[end];
        let slope = (y_end - y_start) / (end - start) as f64;

        let corrected: Vec<f64> = (0..n)
            .map(|i| signal[start + i] - (y_start + slope * i as f64))
            .collect();

        match self.method {
            IntegrationMethod::Trapezoidal => peak_area_trapezoidal(&corrected, self.dt),
            IntegrationMethod::Simpsons => peak_area_simpsons(&corrected, self.dt),
        }
    }
}

// ─── Thermal Conductivity Calculator ─────────────────────────────────────────

/// Converts bridge signal to thermal conductivity values.
///
/// Uses a two-point calibration (or multi-point linear regression) to convert
/// the conditioned Wheatstone bridge voltage to thermal conductivity in W/(m·K).
#[derive(Debug, Clone)]
pub struct ThermalConductivityCalculator {
    /// Calibration slope [W/(m·K) per V].
    pub calibration_slope: f64,
    /// Calibration intercept [W/(m·K)].
    pub calibration_intercept: f64,
    /// Carrier gas TC for relative calculations.
    pub carrier_tc: f64,
}

impl ThermalConductivityCalculator {
    /// Create a calculator with two-point calibration.
    ///
    /// # Arguments
    /// - `v1`, `tc1` — First calibration point (voltage, thermal conductivity)
    /// - `v2`, `tc2` — Second calibration point (voltage, thermal conductivity)
    pub fn from_two_point(v1: f64, tc1: f64, v2: f64, tc2: f64) -> Self {
        let slope = if (v2 - v1).abs() > 1e-30 {
            (tc2 - tc1) / (v2 - v1)
        } else {
            0.0
        };
        let intercept = tc1 - slope * v1;
        Self {
            calibration_slope: slope,
            calibration_intercept: intercept,
            carrier_tc: TC_HELIUM,
        }
    }

    /// Create a calculator with known calibration constants.
    pub fn new(slope: f64, intercept: f64, carrier_gas: CarrierGas) -> Self {
        Self {
            calibration_slope: slope,
            calibration_intercept: intercept,
            carrier_tc: carrier_gas.thermal_conductivity(),
        }
    }

    /// Convert a single voltage sample to thermal conductivity.
    pub fn convert(&self, voltage: f64) -> f64 {
        thermal_conductivity_from_signal(voltage, self.calibration_slope, self.calibration_intercept)
    }

    /// Convert a batch of voltage samples to thermal conductivity.
    pub fn convert_batch(&self, voltages: &[f64]) -> Vec<f64> {
        voltages.iter().map(|&v| self.convert(v)).collect()
    }

    /// Compute the relative response of a sample vs. carrier gas.
    pub fn relative_response(&self, sample_tc: f64) -> f64 {
        relative_response_factor(self.carrier_tc, sample_tc)
    }

    /// Apply multi-point calibration via linear regression.
    ///
    /// # Arguments
    /// - `voltages` — Calibration voltages
    /// - `tc_values` — Corresponding thermal conductivity values
    pub fn calibrate_multipoint(&mut self, voltages: &[f64], tc_values: &[f64]) {
        if voltages.len() < 2 || voltages.len() != tc_values.len() {
            return;
        }
        let coeffs = polyfit(voltages, tc_values, 1);
        if coeffs.len() >= 2 {
            self.calibration_intercept = coeffs[0];
            self.calibration_slope = coeffs[1];
        }
    }
}

// ─── Gas Identifier ──────────────────────────────────────────────────────────

/// Gas identification result.
#[derive(Debug, Clone)]
pub struct GasMatch {
    /// Identified gas species.
    pub species: GasSpecies,
    /// Reference thermal conductivity [W/(m·K)].
    pub reference_tc: f64,
    /// Measured thermal conductivity [W/(m·K)].
    pub measured_tc: f64,
    /// Absolute error [W/(m·K)].
    pub error: f64,
    /// Relative error (fraction).
    pub relative_error: f64,
}

/// Gas identification by thermal conductivity matching.
///
/// Compares a measured TC value against a database of known gases and returns
/// the best match. Supports custom tolerance thresholds and temperature
/// correction.
#[derive(Debug, Clone)]
pub struct GasIdentifier {
    /// Database of known gases with their TC values.
    pub database: Vec<(GasSpecies, f64)>,
    /// Maximum relative error for a valid match.
    pub tolerance: f64,
}

impl GasIdentifier {
    /// Create a new gas identifier with default database (all known gases at 25°C).
    pub fn new(tolerance: f64) -> Self {
        let database = GasSpecies::all()
            .iter()
            .map(|&g| (g, g.thermal_conductivity()))
            .collect();
        Self {
            database,
            tolerance,
        }
    }

    /// Create with a custom database.
    pub fn with_database(database: Vec<(GasSpecies, f64)>, tolerance: f64) -> Self {
        Self {
            database,
            tolerance,
        }
    }

    /// Identify a gas from its measured thermal conductivity.
    ///
    /// Returns the best match if within tolerance, or None.
    pub fn identify(&self, measured_tc: f64) -> Option<GasMatch> {
        let mut best: Option<GasMatch> = None;
        let mut best_error = f64::MAX;

        for &(species, ref_tc) in &self.database {
            let error = (measured_tc - ref_tc).abs();
            let rel_error = if ref_tc.abs() > 1e-30 {
                error / ref_tc
            } else {
                f64::MAX
            };

            if error < best_error && rel_error <= self.tolerance {
                best_error = error;
                best = Some(GasMatch {
                    species,
                    reference_tc: ref_tc,
                    measured_tc,
                    error,
                    relative_error: rel_error,
                });
            }
        }

        best
    }

    /// Rank all database gases by closeness to measured TC.
    pub fn rank_matches(&self, measured_tc: f64) -> Vec<GasMatch> {
        let mut matches: Vec<GasMatch> = self
            .database
            .iter()
            .map(|&(species, ref_tc)| {
                let error = (measured_tc - ref_tc).abs();
                let relative_error = if ref_tc.abs() > 1e-30 {
                    error / ref_tc
                } else {
                    f64::MAX
                };
                GasMatch {
                    species,
                    reference_tc: ref_tc,
                    measured_tc,
                    error,
                    relative_error,
                }
            })
            .collect();

        matches.sort_by(|a, b| a.error.partial_cmp(&b.error).unwrap_or(std::cmp::Ordering::Equal));
        matches
    }
}

// ─── Response Factor Table ───────────────────────────────────────────────────

/// Response factor table entry.
#[derive(Debug, Clone)]
pub struct ResponseFactorEntry {
    /// Gas species.
    pub species: GasSpecies,
    /// Relative response factor vs. carrier gas.
    pub response_factor: f64,
    /// Molar mass (g/mol).
    pub molar_mass: f64,
}

/// Table of relative response factors for common gases.
///
/// Response factors relate peak area to concentration and depend on the carrier gas.
/// They are computed from thermal conductivity differences:
/// RF = (TC_carrier - TC_sample) / (TC_carrier - TC_reference)
#[derive(Debug, Clone)]
pub struct ResponseFactorTable {
    /// Carrier gas used.
    pub carrier_gas: CarrierGas,
    /// Entries in the table.
    pub entries: Vec<ResponseFactorEntry>,
}

impl ResponseFactorTable {
    /// Create a response factor table for a given carrier gas.
    ///
    /// Automatically populates response factors for all known gases.
    pub fn new(carrier_gas: CarrierGas) -> Self {
        let tc_carrier = carrier_gas.thermal_conductivity();

        // Use nitrogen as the reference gas (RF = 1.0 for N2)
        let tc_ref = TC_NITROGEN;
        let denom = tc_carrier - tc_ref;

        let molar_masses = [
            (GasSpecies::Helium, 4.003),
            (GasSpecies::Hydrogen, 2.016),
            (GasSpecies::Nitrogen, 28.014),
            (GasSpecies::Argon, 39.948),
            (GasSpecies::Air, 28.97),
            (GasSpecies::CarbonDioxide, 44.01),
            (GasSpecies::Methane, 16.04),
            (GasSpecies::WaterVapor, 18.015),
        ];

        let entries = molar_masses
            .iter()
            .map(|&(species, mm)| {
                let tc_sample = species.thermal_conductivity();
                let rf = if denom.abs() > 1e-30 {
                    (tc_carrier - tc_sample) / denom
                } else {
                    1.0
                };
                ResponseFactorEntry {
                    species,
                    response_factor: rf,
                    molar_mass: mm,
                }
            })
            .collect();

        Self {
            carrier_gas,
            entries,
        }
    }

    /// Look up the response factor for a given gas species.
    pub fn get_factor(&self, species: GasSpecies) -> Option<f64> {
        self.entries
            .iter()
            .find(|e| e.species == species)
            .map(|e| e.response_factor)
    }

    /// Correct a measured peak area by the response factor.
    ///
    /// Corrected_area = measured_area / response_factor
    pub fn correct_area(&self, species: GasSpecies, measured_area: f64) -> Option<f64> {
        self.get_factor(species).map(|rf| {
            if rf.abs() > 1e-30 {
                measured_area / rf
            } else {
                measured_area
            }
        })
    }
}

// ─── Sensitivity / Noise Analysis ────────────────────────────────────────────

/// TCD sensitivity and noise analysis results.
#[derive(Debug, Clone)]
pub struct SensitivityAnalysis {
    /// Baseline noise RMS (V).
    pub noise_rms: f64,
    /// Peak-to-peak noise (V).
    pub noise_pp: f64,
    /// Detector sensitivity [V / (W/(m·K))].
    pub sensitivity: f64,
    /// Signal-to-noise ratio for a reference peak.
    pub snr: f64,
    /// Limit of detection (LOD) at 3-sigma [W/(m·K)].
    pub lod: f64,
    /// Limit of quantitation (LOQ) at 10-sigma [W/(m·K)].
    pub loq: f64,
    /// Noise drift rate [V/min].
    pub drift_rate: f64,
}

/// TCD sensitivity and noise analyzer per ASTM E685.
#[derive(Debug, Clone)]
pub struct SensitivityAnalyzer {
    /// Calibration slope for sensitivity calculation [W/(m·K) per V].
    pub calibration_slope: f64,
}

impl SensitivityAnalyzer {
    /// Create a new sensitivity analyzer.
    pub fn new(calibration_slope: f64) -> Self {
        Self { calibration_slope }
    }

    /// Analyze a baseline signal segment for noise characteristics.
    ///
    /// # Arguments
    /// - `baseline` — Signal segment from a blank (no peaks) region
    /// - `dt` — Time step between samples (seconds)
    ///
    /// # Returns
    /// Sensitivity analysis results.
    pub fn analyze(&self, baseline: &[f64], dt: f64) -> SensitivityAnalysis {
        let rms = noise_rms(baseline);

        let (min, max) = baseline.iter().fold((f64::MAX, f64::MIN), |(lo, hi), &x| {
            (lo.min(x), hi.max(x))
        });
        let pp = if baseline.is_empty() {
            0.0
        } else {
            max - min
        };

        let sensitivity = if self.calibration_slope.abs() > 1e-30 {
            1.0 / self.calibration_slope
        } else {
            0.0
        };

        let lod = detection_limit(rms, self.calibration_slope.abs(), 3.0);
        let loq = detection_limit(rms, self.calibration_slope.abs(), 10.0);

        // Compute drift rate by linear regression
        let drift_rate = if baseline.len() >= 2 {
            let x: Vec<f64> = (0..baseline.len()).map(|i| i as f64 * dt / 60.0).collect();
            let coeffs = polyfit(&x, baseline, 1);
            if coeffs.len() >= 2 {
                coeffs[1] // slope in V/min
            } else {
                0.0
            }
        } else {
            0.0
        };

        let snr = if rms > 0.0 { pp / rms } else { f64::INFINITY };

        SensitivityAnalysis {
            noise_rms: rms,
            noise_pp: pp,
            sensitivity,
            snr,
            lod,
            loq,
            drift_rate,
        }
    }

    /// Analyze with a known reference peak for SNR calculation.
    pub fn analyze_with_reference(
        &self,
        baseline: &[f64],
        peak_height: f64,
        dt: f64,
    ) -> SensitivityAnalysis {
        let mut result = self.analyze(baseline, dt);
        result.snr = signal_to_noise(peak_height, result.noise_rms);
        result
    }
}

// ─── Full TCD Processor ─────────────────────────────────────────────────────

/// Complete TCD signal processing pipeline.
///
/// Combines all processing stages into a single convenient processor.
#[derive(Debug, Clone)]
pub struct TcdProcessor {
    /// Detector configuration.
    pub config: TcdConfig,
    /// Wheatstone bridge conditioner.
    pub conditioner: WheatstoneConditioner,
    /// Baseline drift corrector.
    pub baseline_corrector: BaselineDriftCorrector,
    /// Peak detector.
    pub peak_detector: PeakDetector,
    /// Peak integrator.
    pub integrator: PeakIntegrator,
    /// Thermal conductivity calculator.
    pub tc_calculator: ThermalConductivityCalculator,
    /// Gas identifier.
    pub gas_identifier: GasIdentifier,
}

/// Result of processing a complete chromatogram.
#[derive(Debug, Clone)]
pub struct ChromatogramResult {
    /// Conditioned signal.
    pub conditioned_signal: Vec<f64>,
    /// Baseline-corrected signal.
    pub corrected_signal: Vec<f64>,
    /// Detected peaks.
    pub peaks: Vec<DetectedPeak>,
    /// Peak areas.
    pub peak_areas: Vec<f64>,
    /// Thermal conductivity at each peak apex.
    pub peak_tc: Vec<f64>,
    /// Gas identification for each peak.
    pub identifications: Vec<Option<GasMatch>>,
}

impl TcdProcessor {
    /// Create a new TCD processor with default settings.
    pub fn new(config: TcdConfig) -> Self {
        let dt = 1.0 / config.sample_rate_hz;
        let conditioner = WheatstoneConditioner::new(config.bridge_supply_v, 1000.0);
        let baseline_corrector = BaselineDriftCorrector::new(2);
        let peak_detector = PeakDetector::new(0.001, 0.01, 3);
        let integrator = PeakIntegrator::new(IntegrationMethod::Simpsons, dt);
        let tc_calculator = ThermalConductivityCalculator::new(
            0.01, // slope
            config.carrier_gas.thermal_conductivity(),
            config.carrier_gas,
        );
        let gas_identifier = GasIdentifier::new(0.2); // 20% tolerance

        Self {
            config,
            conditioner,
            baseline_corrector,
            peak_detector,
            integrator,
            tc_calculator,
            gas_identifier,
        }
    }

    /// Process a complete raw chromatogram.
    ///
    /// Runs the full pipeline: conditioning → baseline correction → peak detection
    /// → integration → TC calculation → gas identification.
    pub fn process(&mut self, raw_signal: &[f64]) -> ChromatogramResult {
        // Step 1: Condition
        let conditioned = self.conditioner.condition_batch(raw_signal);

        // Step 2: Baseline correction
        let corrected = self.baseline_corrector.correct(&conditioned);

        // Step 3: Peak detection
        let peaks = self.peak_detector.detect(&corrected);

        // Step 4: Integration
        let peak_areas: Vec<f64> = peaks
            .iter()
            .map(|p| self.integrator.integrate_peak(&corrected, p))
            .collect();

        // Step 5: TC calculation at each peak apex
        let peak_tc: Vec<f64> = peaks
            .iter()
            .map(|p| self.tc_calculator.convert(conditioned[p.apex_index]))
            .collect();

        // Step 6: Gas identification
        let identifications: Vec<Option<GasMatch>> =
            peak_tc.iter().map(|&tc| self.gas_identifier.identify(tc)).collect();

        ChromatogramResult {
            conditioned_signal: conditioned,
            corrected_signal: corrected,
            peaks,
            peak_areas,
            peak_tc,
            identifications,
        }
    }
}

// ─── EMG Peak Fitting ────────────────────────────────────────────────────────

/// Exponentially Modified Gaussian (EMG) parameters.
///
/// EMG models asymmetric (tailing) peaks common in chromatography:
/// h(t) = (A * sigma / tau) * sqrt(pi/2) * exp((sigma/(sqrt(2)*tau))^2 - (t-mu)/tau)
///        * erfc((sigma/(sqrt(2)*tau)) - (t-mu)/(sqrt(2)*sigma))
#[derive(Debug, Clone)]
pub struct EmgParams {
    /// Amplitude.
    pub amplitude: f64,
    /// Center (mean) of the Gaussian component.
    pub center: f64,
    /// Standard deviation of the Gaussian component.
    pub sigma: f64,
    /// Exponential time constant (tailing parameter).
    pub tau: f64,
}

impl EmgParams {
    /// Evaluate the EMG function at time t.
    ///
    /// Uses a simplified approximation for the erfc term.
    pub fn evaluate(&self, t: f64) -> f64 {
        let z = (t - self.center) / self.sigma;
        let lambda = self.sigma / self.tau;

        // Simplified EMG: Gaussian * exponential tail correction
        let gaussian = self.amplitude * (-0.5 * z * z).exp();
        let tail = if z > 0.0 {
            let decay = (-z / lambda).exp();
            gaussian * (1.0 + (1.0 - decay) * 0.5 * lambda)
        } else {
            gaussian
        };
        tail
    }

    /// Estimate EMG parameters from peak data.
    ///
    /// Uses the Gaussian fit as a starting point and estimates tau from asymmetry.
    pub fn estimate(x: &[f64], y: &[f64]) -> Option<Self> {
        let (amplitude, center, sigma) = gaussian_fit(x, y)?;

        // Estimate tau from peak asymmetry
        // Find the peak maximum index
        let mut max_idx = 0;
        let mut max_val = y[0];
        for i in 1..y.len() {
            if y[i] > max_val {
                max_val = y[i];
                max_idx = i;
            }
        }

        // Measure asymmetry from the difference between centroid and maximum
        let total_area: f64 = y.iter().sum();
        let centroid = if total_area > 0.0 {
            x.iter().zip(y.iter()).map(|(&xi, &yi)| xi * yi).sum::<f64>() / total_area
        } else {
            center
        };

        let shift = centroid - x[max_idx];
        let tau = if shift.abs() > 1e-10 {
            shift.abs().max(sigma * 0.1)
        } else {
            sigma * 0.5 // Default tau if symmetric
        };

        Some(Self {
            amplitude,
            center,
            sigma,
            tau,
        })
    }
}

// ─── Helper: approximate erfc for completeness ──────────────────────────────

/// Approximate complementary error function using Horner's method.
///
/// Abramowitz and Stegun approximation (7.1.26), max error ~1.5e-7.
#[allow(dead_code)]
fn erfc_approx(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }

    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;
    const APPROX: f64 = 1e-6;

    // ── Physical Constants ───────────────────────────────────────────────

    #[test]
    fn test_tc_helium_value() {
        assert!((TC_HELIUM - 0.1513).abs() < EPSILON);
    }

    #[test]
    fn test_tc_hydrogen_value() {
        assert!((TC_HYDROGEN - 0.1805).abs() < EPSILON);
    }

    #[test]
    fn test_tc_nitrogen_value() {
        assert!((TC_NITROGEN - 0.0259).abs() < EPSILON);
    }

    #[test]
    fn test_tc_argon_value() {
        assert!((TC_ARGON - 0.0177).abs() < EPSILON);
    }

    #[test]
    fn test_tc_air_value() {
        assert!((TC_AIR - 0.0262).abs() < EPSILON);
    }

    #[test]
    fn test_tc_co2_value() {
        assert!((TC_CO2 - 0.0166).abs() < EPSILON);
    }

    #[test]
    fn test_tc_methane_value() {
        assert!((TC_METHANE - 0.0343).abs() < EPSILON);
    }

    #[test]
    fn test_tc_water_vapor_value() {
        assert!((TC_WATER_VAPOR - 0.0186).abs() < EPSILON);
    }

    // ── CarrierGas ───────────────────────────────────────────────────────

    #[test]
    fn test_carrier_gas_helium_tc() {
        assert!((CarrierGas::Helium.thermal_conductivity() - TC_HELIUM).abs() < EPSILON);
    }

    #[test]
    fn test_carrier_gas_argon_tc() {
        assert!((CarrierGas::Argon.thermal_conductivity() - TC_ARGON).abs() < EPSILON);
    }

    // ── GasSpecies ───────────────────────────────────────────────────────

    #[test]
    fn test_gas_species_all_count() {
        assert_eq!(GasSpecies::all().len(), 8);
    }

    #[test]
    fn test_gas_species_name_helium() {
        assert_eq!(GasSpecies::Helium.name(), "Helium");
    }

    #[test]
    fn test_gas_species_name_co2() {
        assert_eq!(GasSpecies::CarbonDioxide.name(), "Carbon Dioxide");
    }

    #[test]
    fn test_gas_species_tc_methane() {
        assert!((GasSpecies::Methane.thermal_conductivity() - TC_METHANE).abs() < EPSILON);
    }

    // ── Wheatstone Bridge ────────────────────────────────────────────────

    #[test]
    fn test_wheatstone_balanced() {
        // Equal resistances → zero output
        let v = wheatstone_voltage(100.0, 100.0, 100.0, 100.0, 10.0);
        assert!(v.abs() < EPSILON);
    }

    #[test]
    fn test_wheatstone_unbalanced() {
        // R4 changed → nonzero output
        let v = wheatstone_voltage(100.0, 100.0, 100.0, 110.0, 10.0);
        assert!(v < 0.0); // V_A < V_B since R4 > R2
    }

    #[test]
    fn test_wheatstone_known_values() {
        // R1=R2=R3=100, R4=200, Vs=10
        // VA = 10 * 100/200 = 5.0
        // VB = 10 * 200/300 = 6.667
        let v = wheatstone_voltage(100.0, 100.0, 100.0, 200.0, 10.0);
        let expected = 5.0 - 10.0 * 200.0 / 300.0;
        assert!((v - expected).abs() < APPROX);
    }

    #[test]
    fn test_wheatstone_zero_supply() {
        let v = wheatstone_voltage(100.0, 100.0, 100.0, 200.0, 0.0);
        assert!(v.abs() < EPSILON);
    }

    #[test]
    fn test_wheatstone_symmetry() {
        // Swapping R1/R3 with R2/R4 inverts the sign
        let v1 = wheatstone_voltage(100.0, 200.0, 100.0, 200.0, 10.0);
        // Equal ratio → zero
        assert!(v1.abs() < EPSILON);
    }

    // ── Thermal Conductivity from Signal ─────────────────────────────────

    #[test]
    fn test_tc_from_signal_zero() {
        let tc = thermal_conductivity_from_signal(0.0, 0.01, 0.05);
        assert!((tc - 0.05).abs() < EPSILON);
    }

    #[test]
    fn test_tc_from_signal_linear() {
        let tc = thermal_conductivity_from_signal(1.0, 0.01, 0.0);
        assert!((tc - 0.01).abs() < EPSILON);
    }

    #[test]
    fn test_tc_from_signal_negative() {
        let tc = thermal_conductivity_from_signal(-1.0, 0.01, 0.1);
        assert!((tc - 0.09).abs() < EPSILON);
    }

    // ── Peak Area: Trapezoidal ───────────────────────────────────────────

    #[test]
    fn test_trap_area_constant() {
        // Constant signal of 1.0 over 10 samples at dt=0.1 → area = 0.9
        let signal = vec![1.0; 10];
        let area = peak_area_trapezoidal(&signal, 0.1);
        assert!((area - 0.9).abs() < APPROX);
    }

    #[test]
    fn test_trap_area_triangle() {
        // Triangle: 0,1,2,3,4,3,2,1,0 at dt=1
        let signal = vec![0.0, 1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.0];
        let area = peak_area_trapezoidal(&signal, 1.0);
        assert!((area - 16.0).abs() < APPROX);
    }

    #[test]
    fn test_trap_area_single_point() {
        let area = peak_area_trapezoidal(&[5.0], 1.0);
        assert!(area.abs() < EPSILON);
    }

    #[test]
    fn test_trap_area_empty() {
        let area = peak_area_trapezoidal(&[], 1.0);
        assert!(area.abs() < EPSILON);
    }

    #[test]
    fn test_trap_area_two_points() {
        let area = peak_area_trapezoidal(&[1.0, 3.0], 0.5);
        assert!((area - 1.0).abs() < APPROX); // 0.5 * (1+3) * 0.5
    }

    // ── Peak Area: Simpson's ─────────────────────────────────────────────

    #[test]
    fn test_simpsons_area_constant() {
        let signal = vec![2.0; 5]; // 4 intervals
        let area = peak_area_simpsons(&signal, 1.0);
        assert!((area - 8.0).abs() < APPROX);
    }

    #[test]
    fn test_simpsons_area_quadratic() {
        // x^2 from 0..4 → exact integral = 64/3 = 21.333...
        // Simpson's is exact for polynomials up to degree 3
        let signal: Vec<f64> = (0..=4).map(|i| (i * i) as f64).collect();
        let area = peak_area_simpsons(&signal, 1.0);
        assert!((area - 64.0 / 3.0).abs() < APPROX);
    }

    #[test]
    fn test_simpsons_area_two_points() {
        let area = peak_area_simpsons(&[1.0, 3.0], 0.5);
        assert!((area - 1.0).abs() < APPROX);
    }

    #[test]
    fn test_simpsons_area_empty() {
        let area = peak_area_simpsons(&[], 1.0);
        assert!(area.abs() < EPSILON);
    }

    #[test]
    fn test_simpsons_vs_trapezoidal_linear() {
        // For linear data, both should give the same result
        let signal: Vec<f64> = (0..11).map(|i| i as f64).collect();
        let trap = peak_area_trapezoidal(&signal, 1.0);
        let simp = peak_area_simpsons(&signal, 1.0);
        assert!((trap - simp).abs() < APPROX);
    }

    // ── Gaussian Fit ─────────────────────────────────────────────────────

    #[test]
    fn test_gaussian_fit_perfect() {
        let amp = 10.0;
        let mu = 5.0;
        let sigma = 1.0;
        let x: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let y: Vec<f64> = x
            .iter()
            .map(|&xi| amp * (-0.5 * ((xi - mu) / sigma).powi(2)).exp())
            .collect();

        let result = gaussian_fit(&x, &y);
        assert!(result.is_some());
        let (a, c, s) = result.unwrap();
        assert!((a - amp).abs() < 0.1);
        assert!((c - mu).abs() < 0.1);
        assert!((s - sigma).abs() < 0.1);
    }

    #[test]
    fn test_gaussian_fit_too_few_points() {
        let result = gaussian_fit(&[0.0, 1.0], &[1.0, 2.0]);
        assert!(result.is_none());
    }

    #[test]
    fn test_gaussian_fit_mismatched_lengths() {
        let result = gaussian_fit(&[0.0, 1.0, 2.0], &[1.0, 2.0]);
        assert!(result.is_none());
    }

    #[test]
    fn test_gaussian_fit_zero_data() {
        // All zeros → no positive values for log transform
        let result = gaussian_fit(&[0.0, 1.0, 2.0], &[0.0, 0.0, 0.0]);
        assert!(result.is_none());
    }

    // ── SNR & Detection Limit ────────────────────────────────────────────

    #[test]
    fn test_snr_basic() {
        assert!((signal_to_noise(10.0, 2.0) - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_snr_zero_noise() {
        assert!(signal_to_noise(10.0, 0.0).is_infinite());
    }

    #[test]
    fn test_detection_limit_lod() {
        let lod = detection_limit(0.001, 0.01, 3.0);
        assert!((lod - 0.3).abs() < APPROX);
    }

    #[test]
    fn test_detection_limit_loq() {
        let loq = detection_limit(0.001, 0.01, 10.0);
        assert!((loq - 1.0).abs() < APPROX);
    }

    #[test]
    fn test_detection_limit_zero_slope() {
        let lod = detection_limit(0.001, 0.0, 3.0);
        assert!(lod.is_infinite());
    }

    // ── Noise RMS ────────────────────────────────────────────────────────

    #[test]
    fn test_noise_rms_constant() {
        let rms = noise_rms(&[5.0, 5.0, 5.0, 5.0]);
        assert!(rms < EPSILON);
    }

    #[test]
    fn test_noise_rms_known_values() {
        // Values centered at 0: [-1, 1, -1, 1]
        let rms = noise_rms(&[-1.0, 1.0, -1.0, 1.0]);
        assert!((rms - 1.0).abs() < APPROX);
    }

    #[test]
    fn test_noise_rms_empty() {
        assert!(noise_rms(&[]).abs() < EPSILON);
    }

    // ── Relative Response Factor ─────────────────────────────────────────

    #[test]
    fn test_rrf_same_gas() {
        let rf = relative_response_factor(TC_HELIUM, TC_HELIUM);
        assert!(rf.abs() < EPSILON);
    }

    #[test]
    fn test_rrf_helium_nitrogen() {
        let rf = relative_response_factor(TC_HELIUM, TC_NITROGEN);
        let expected = (TC_HELIUM - TC_NITROGEN) / TC_HELIUM;
        assert!((rf - expected).abs() < EPSILON);
    }

    #[test]
    fn test_rrf_positive_for_lower_tc() {
        // Sample has lower TC than carrier → positive response
        let rf = relative_response_factor(TC_HELIUM, TC_CO2);
        assert!(rf > 0.0);
    }

    // ── Temperature Correction ───────────────────────────────────────────

    #[test]
    fn test_temp_correction_same_temp() {
        let tc = temperature_correct_tc(TC_HELIUM, 298.15, 298.15, 0.75);
        assert!((tc - TC_HELIUM).abs() < EPSILON);
    }

    #[test]
    fn test_temp_correction_higher_temp() {
        // TC increases with temperature for most gases
        let tc = temperature_correct_tc(TC_HELIUM, 298.15, 373.15, 0.75);
        assert!(tc > TC_HELIUM);
    }

    #[test]
    fn test_temp_correction_zero_ref() {
        let tc = temperature_correct_tc(TC_HELIUM, 0.0, 300.0, 0.75);
        assert!((tc - TC_HELIUM).abs() < EPSILON);
    }

    // ── Wheatstone Conditioner ───────────────────────────────────────────

    #[test]
    fn test_conditioner_basic() {
        let mut cond = WheatstoneConditioner::new(10.0, 100.0);
        let result = cond.condition(0.005);
        assert!((result - 0.5).abs() < APPROX);
    }

    #[test]
    fn test_conditioner_with_offset() {
        let mut cond = WheatstoneConditioner::new(10.0, 100.0);
        cond.set_zero_offset(0.001);
        let result = cond.condition(0.005);
        assert!((result - 0.4).abs() < APPROX);
    }

    #[test]
    fn test_conditioner_auto_zero() {
        let mut cond = WheatstoneConditioner::new(10.0, 100.0);
        cond.reference_resistances = [100.0, 100.0, 100.0, 100.0];
        cond.auto_zero();
        assert!(cond.zero_offset.abs() < EPSILON);
    }

    #[test]
    fn test_conditioner_lowpass() {
        let mut cond = WheatstoneConditioner::new(10.0, 1.0);
        cond.set_lowpass(0.1);
        // First sample initializes
        let v1 = cond.condition(1.0);
        assert!((v1 - 1.0).abs() < APPROX);
        // Second sample is smoothed
        let v2 = cond.condition(2.0);
        assert!(v2 > 1.0 && v2 < 2.0);
    }

    #[test]
    fn test_conditioner_batch() {
        let mut cond = WheatstoneConditioner::new(10.0, 1.0);
        let raw = vec![0.1, 0.2, 0.3, 0.4];
        let result = cond.condition_batch(&raw);
        assert_eq!(result.len(), 4);
        assert!((result[0] - 0.1).abs() < APPROX);
    }

    #[test]
    fn test_conditioner_reset() {
        let mut cond = WheatstoneConditioner::new(10.0, 1.0);
        cond.set_lowpass(0.5);
        cond.condition(1.0);
        cond.condition(2.0);
        cond.reset();
        // After reset, next sample should initialize the filter
        let v = cond.condition(5.0);
        assert!((v - 5.0).abs() < APPROX);
    }

    // ── Baseline Drift Corrector ─────────────────────────────────────────

    #[test]
    fn test_baseline_constant_removal() {
        let corrector = BaselineDriftCorrector::new(0);
        let signal = vec![5.0, 5.1, 4.9, 5.0, 5.05];
        let corrected = corrector.correct(&signal);
        let mean: f64 = corrected.iter().sum::<f64>() / corrected.len() as f64;
        assert!(mean.abs() < 0.1);
    }

    #[test]
    fn test_baseline_linear_removal() {
        let corrector = BaselineDriftCorrector::new(1);
        // Linear drift: y = 0.01*x + 5
        let signal: Vec<f64> = (0..100).map(|i| 0.01 * i as f64 + 5.0).collect();
        let corrected = corrector.correct(&signal);
        // All values should be near zero after removing linear drift
        let max_abs = corrected.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
        assert!(max_abs < 0.01);
    }

    #[test]
    fn test_baseline_anchored() {
        let corrector = BaselineDriftCorrector::new(1);
        let mut signal: Vec<f64> = (0..100).map(|i| 0.01 * i as f64).collect();
        // Add a peak in the middle
        for i in 40..60 {
            signal[i] += 5.0;
        }
        // Anchor to baseline regions (before and after peak)
        let anchors: Vec<usize> = (0..30).chain(70..100).collect();
        let corrected = corrector.correct_anchored(&signal, &anchors);
        // Baseline regions should be near zero
        assert!(corrected[10].abs() < 0.5);
        // Peak should still be visible
        assert!(corrected[50] > 3.0);
    }

    #[test]
    fn test_baseline_empty_signal() {
        let corrector = BaselineDriftCorrector::new(2);
        let corrected = corrector.correct(&[]);
        assert!(corrected.is_empty());
    }

    // ── Peak Detector ────────────────────────────────────────────────────

    #[test]
    fn test_peak_detect_single() {
        let mut signal = vec![0.0; 100];
        // Add a Gaussian peak centered at 50
        for i in 0..100 {
            let x = (i as f64 - 50.0) / 5.0;
            signal[i] += 10.0 * (-0.5 * x * x).exp();
        }
        let detector = PeakDetector::new(0.01, 1.0, 3);
        let peaks = detector.detect(&signal);
        assert!(!peaks.is_empty());
        assert!((peaks[0].apex_index as f64 - 50.0).abs() < 3.0);
    }

    #[test]
    fn test_peak_detect_multiple() {
        let mut signal = vec![0.0; 200];
        // Two peaks
        for i in 0..200 {
            let x1 = (i as f64 - 50.0) / 5.0;
            let x2 = (i as f64 - 150.0) / 5.0;
            signal[i] += 10.0 * (-0.5 * x1 * x1).exp();
            signal[i] += 8.0 * (-0.5 * x2 * x2).exp();
        }
        let detector = PeakDetector::new(0.01, 1.0, 3);
        let peaks = detector.detect(&signal);
        assert!(peaks.len() >= 2);
    }

    #[test]
    fn test_peak_detect_below_threshold() {
        let mut signal = vec![0.0; 100];
        // Very small peak
        for i in 0..100 {
            let x = (i as f64 - 50.0) / 5.0;
            signal[i] += 0.001 * (-0.5 * x * x).exp();
        }
        let detector = PeakDetector::new(0.01, 1.0, 3);
        let peaks = detector.detect(&signal);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_peak_detect_flat_signal() {
        let signal = vec![1.0; 100];
        let detector = PeakDetector::new(0.01, 0.1, 3);
        let peaks = detector.detect(&signal);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_peak_detect_short_signal() {
        let detector = PeakDetector::new(0.01, 0.1, 3);
        let peaks = detector.detect(&[1.0, 2.0]);
        assert!(peaks.is_empty());
    }

    // ── Peak Integrator ──────────────────────────────────────────────────

    #[test]
    fn test_integrator_trapezoidal() {
        let integrator = PeakIntegrator::new(IntegrationMethod::Trapezoidal, 1.0);
        let signal = vec![0.0, 1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.0];
        let peak = DetectedPeak {
            onset_index: 0,
            apex_index: 4,
            end_index: 8,
            height: 4.0,
            fwhm: 4.0,
            asymmetry: 1.0,
        };
        let area = integrator.integrate_peak(&signal, &peak);
        assert!((area - 16.0).abs() < APPROX);
    }

    #[test]
    fn test_integrator_simpsons() {
        let integrator = PeakIntegrator::new(IntegrationMethod::Simpsons, 1.0);
        let signal = vec![0.0, 1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.0];
        let peak = DetectedPeak {
            onset_index: 0,
            apex_index: 4,
            end_index: 8,
            height: 4.0,
            fwhm: 4.0,
            asymmetry: 1.0,
        };
        let area = integrator.integrate_peak(&signal, &peak);
        assert!(area > 15.0 && area < 17.0);
    }

    #[test]
    fn test_integrator_all() {
        let integrator = PeakIntegrator::new(IntegrationMethod::Trapezoidal, 1.0);
        let signal = vec![0.0; 10];
        let peaks = vec![
            DetectedPeak {
                onset_index: 0,
                apex_index: 2,
                end_index: 4,
                height: 1.0,
                fwhm: 2.0,
                asymmetry: 1.0,
            },
            DetectedPeak {
                onset_index: 5,
                apex_index: 7,
                end_index: 9,
                height: 1.0,
                fwhm: 2.0,
                asymmetry: 1.0,
            },
        ];
        let results = integrator.integrate_all(&signal, &peaks);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_integrator_with_baseline() {
        let integrator = PeakIntegrator::new(IntegrationMethod::Trapezoidal, 1.0);
        // Signal with a sloped baseline + peak
        let signal = vec![1.0, 2.0, 5.0, 8.0, 5.0, 2.0, 1.0];
        let peak = DetectedPeak {
            onset_index: 0,
            apex_index: 3,
            end_index: 6,
            height: 8.0,
            fwhm: 4.0,
            asymmetry: 1.0,
        };
        let area = integrator.integrate_peak_with_baseline(&signal, &peak);
        // Baseline runs from signal[0]=1.0 to signal[6]=1.0 (flat at 1.0)
        // Peak above baseline should be positive
        assert!(area > 0.0);
    }

    // ── Thermal Conductivity Calculator ──────────────────────────────────

    #[test]
    fn test_tc_calc_two_point() {
        let calc = ThermalConductivityCalculator::from_two_point(0.0, TC_HELIUM, 1.0, TC_NITROGEN);
        let tc = calc.convert(0.0);
        assert!((tc - TC_HELIUM).abs() < APPROX);
        let tc2 = calc.convert(1.0);
        assert!((tc2 - TC_NITROGEN).abs() < APPROX);
    }

    #[test]
    fn test_tc_calc_new() {
        let calc = ThermalConductivityCalculator::new(0.01, 0.05, CarrierGas::Helium);
        let tc = calc.convert(1.0);
        assert!((tc - 0.06).abs() < APPROX);
    }

    #[test]
    fn test_tc_calc_batch() {
        let calc = ThermalConductivityCalculator::from_two_point(0.0, 0.0, 1.0, 1.0);
        let result = calc.convert_batch(&[0.0, 0.5, 1.0]);
        assert_eq!(result.len(), 3);
        assert!((result[1] - 0.5).abs() < APPROX);
    }

    #[test]
    fn test_tc_calc_relative_response() {
        let calc = ThermalConductivityCalculator::new(0.01, 0.0, CarrierGas::Helium);
        let rr = calc.relative_response(TC_NITROGEN);
        assert!(rr > 0.0 && rr < 1.0);
    }

    #[test]
    fn test_tc_calc_multipoint() {
        let mut calc = ThermalConductivityCalculator::new(0.0, 0.0, CarrierGas::Helium);
        let voltages = vec![0.0, 1.0, 2.0, 3.0];
        let tc_vals = vec![0.0, 0.05, 0.10, 0.15];
        calc.calibrate_multipoint(&voltages, &tc_vals);
        let tc = calc.convert(2.0);
        assert!((tc - 0.10).abs() < 0.01);
    }

    // ── Gas Identifier ───────────────────────────────────────────────────

    #[test]
    fn test_gas_id_helium() {
        let id = GasIdentifier::new(0.1);
        let result = id.identify(TC_HELIUM);
        assert!(result.is_some());
        assert_eq!(result.unwrap().species, GasSpecies::Helium);
    }

    #[test]
    fn test_gas_id_co2() {
        let id = GasIdentifier::new(0.1);
        let result = id.identify(TC_CO2);
        assert!(result.is_some());
        assert_eq!(result.unwrap().species, GasSpecies::CarbonDioxide);
    }

    #[test]
    fn test_gas_id_no_match() {
        let id = GasIdentifier::new(0.001); // Very tight tolerance
        let result = id.identify(0.5); // No gas with this TC
        assert!(result.is_none());
    }

    #[test]
    fn test_gas_id_rank() {
        let id = GasIdentifier::new(1.0);
        let ranked = id.rank_matches(TC_METHANE);
        assert!(!ranked.is_empty());
        assert_eq!(ranked[0].species, GasSpecies::Methane);
    }

    #[test]
    fn test_gas_id_custom_database() {
        let db = vec![
            (GasSpecies::Helium, 0.1513),
            (GasSpecies::Nitrogen, 0.0259),
        ];
        let id = GasIdentifier::with_database(db, 0.1);
        let result = id.identify(0.15);
        assert!(result.is_some());
        assert_eq!(result.unwrap().species, GasSpecies::Helium);
    }

    // ── Response Factor Table ────────────────────────────────────────────

    #[test]
    fn test_rft_helium_carrier() {
        let table = ResponseFactorTable::new(CarrierGas::Helium);
        assert_eq!(table.entries.len(), 8);
    }

    #[test]
    fn test_rft_nitrogen_rf_is_one() {
        let table = ResponseFactorTable::new(CarrierGas::Helium);
        let rf = table.get_factor(GasSpecies::Nitrogen);
        assert!(rf.is_some());
        assert!((rf.unwrap() - 1.0).abs() < APPROX);
    }

    #[test]
    fn test_rft_correct_area() {
        let table = ResponseFactorTable::new(CarrierGas::Helium);
        let corrected = table.correct_area(GasSpecies::Nitrogen, 100.0);
        assert!(corrected.is_some());
        assert!((corrected.unwrap() - 100.0).abs() < APPROX);
    }

    #[test]
    fn test_rft_unknown_gas() {
        let table = ResponseFactorTable::new(CarrierGas::Helium);
        // All gases are in the table, so this tests get_factor returning Some
        let rf = table.get_factor(GasSpecies::Methane);
        assert!(rf.is_some());
    }

    #[test]
    fn test_rft_argon_carrier() {
        let table = ResponseFactorTable::new(CarrierGas::Argon);
        // With argon carrier, helium should have a large positive response factor
        // RF = (TC_Ar - TC_He) / (TC_Ar - TC_N2), both numerator and denominator negative
        let rf = table.get_factor(GasSpecies::Helium);
        assert!(rf.is_some());
        assert!(rf.unwrap() > 1.0, "RF for He with Ar carrier should be > 1.0, got {}", rf.unwrap());
    }

    // ── Sensitivity Analyzer ─────────────────────────────────────────────

    #[test]
    fn test_sensitivity_clean_baseline() {
        let analyzer = SensitivityAnalyzer::new(0.01);
        let baseline = vec![0.001, 0.0012, 0.0009, 0.0011, 0.001];
        let result = analyzer.analyze(&baseline, 0.1);
        assert!(result.noise_rms > 0.0);
        assert!(result.noise_pp > 0.0);
        assert!(result.lod > 0.0);
        assert!(result.loq > result.lod);
    }

    #[test]
    fn test_sensitivity_with_reference() {
        let analyzer = SensitivityAnalyzer::new(0.01);
        let baseline = vec![0.0; 10];
        let result = analyzer.analyze_with_reference(&baseline, 1.0, 0.1);
        assert!(result.snr.is_infinite()); // Zero noise
    }

    #[test]
    fn test_sensitivity_drift_rate() {
        let analyzer = SensitivityAnalyzer::new(0.01);
        // Linear drift: 0.001 V/sample at dt=1s = 0.06 V/min
        let baseline: Vec<f64> = (0..60).map(|i| 0.001 * i as f64).collect();
        let result = analyzer.analyze(&baseline, 1.0);
        assert!(result.drift_rate > 0.0);
    }

    // ── Polynomial Helpers ───────────────────────────────────────────────

    #[test]
    fn test_polyeval_constant() {
        let val = polyeval(&[5.0], 10.0);
        assert!((val - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_polyeval_linear() {
        // p(x) = 2 + 3*x → p(4) = 14
        let val = polyeval(&[2.0, 3.0], 4.0);
        assert!((val - 14.0).abs() < EPSILON);
    }

    #[test]
    fn test_polyeval_quadratic() {
        // p(x) = 1 + 0*x + 1*x^2 → p(3) = 10
        let val = polyeval(&[1.0, 0.0, 1.0], 3.0);
        assert!((val - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_polyfit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![1.0, 3.0, 5.0, 7.0, 9.0]; // y = 1 + 2*x
        let coeffs = polyfit(&x, &y, 1);
        assert!((coeffs[0] - 1.0).abs() < APPROX);
        assert!((coeffs[1] - 2.0).abs() < APPROX);
    }

    #[test]
    fn test_polyfit_quadratic() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|&xi| xi * xi).collect(); // y = x^2
        let coeffs = polyfit(&x, &y, 2);
        assert!(coeffs[0].abs() < 0.1);
        assert!(coeffs[1].abs() < 0.1);
        assert!((coeffs[2] - 1.0).abs() < 0.1);
    }

    // ── EMG Fitting ──────────────────────────────────────────────────────

    #[test]
    fn test_emg_evaluate_center() {
        let emg = EmgParams {
            amplitude: 10.0,
            center: 5.0,
            sigma: 1.0,
            tau: 0.5,
        };
        let val = emg.evaluate(5.0);
        assert!(val > 9.0); // Near the peak
    }

    #[test]
    fn test_emg_evaluate_far() {
        let emg = EmgParams {
            amplitude: 10.0,
            center: 5.0,
            sigma: 1.0,
            tau: 0.5,
        };
        let val = emg.evaluate(100.0);
        assert!(val.abs() < 0.01);
    }

    #[test]
    fn test_emg_estimate_gaussian() {
        let x: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let y: Vec<f64> = x
            .iter()
            .map(|&xi| 10.0 * (-0.5 * ((xi - 5.0) / 1.0f64).powi(2)).exp())
            .collect();
        let params = EmgParams::estimate(&x, &y);
        assert!(params.is_some());
        let p = params.unwrap();
        assert!(p.amplitude > 5.0);
    }

    // ── erfc approximation ───────────────────────────────────────────────

    #[test]
    fn test_erfc_at_zero() {
        let val = erfc_approx(0.0);
        assert!((val - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_erfc_at_large() {
        let val = erfc_approx(5.0);
        assert!(val < 1e-6);
    }

    #[test]
    fn test_erfc_negative() {
        let val = erfc_approx(-1.0);
        assert!(val > 1.0 && val < 2.0);
    }

    // ── TCD Config ───────────────────────────────────────────────────────

    #[test]
    fn test_config_default() {
        let config = TcdConfig::default();
        assert!((config.bridge_current_ma - 200.0).abs() < EPSILON);
        assert_eq!(config.carrier_gas, CarrierGas::Helium);
        assert!((config.cell_temperature_c - 150.0).abs() < EPSILON);
    }

    // ── TCD Processor ────────────────────────────────────────────────────

    #[test]
    fn test_processor_creation() {
        let config = TcdConfig::default();
        let processor = TcdProcessor::new(config);
        assert!((processor.config.bridge_current_ma - 200.0).abs() < EPSILON);
    }

    #[test]
    fn test_processor_flat_signal() {
        let config = TcdConfig::default();
        let mut processor = TcdProcessor::new(config);
        let signal = vec![0.001; 100];
        let result = processor.process(&signal);
        assert_eq!(result.conditioned_signal.len(), 100);
        assert!(result.peaks.is_empty());
    }

    #[test]
    fn test_processor_with_peak() {
        let config = TcdConfig {
            sample_rate_hz: 10.0,
            ..TcdConfig::default()
        };
        let mut processor = TcdProcessor::new(config);
        processor.conditioner.gain = 1.0;
        processor.peak_detector.slope_threshold = 0.001;
        processor.peak_detector.min_height = 0.1;

        // Create signal with a clear peak
        let mut signal = vec![0.0; 200];
        for i in 0..200 {
            let x = (i as f64 - 100.0) / 10.0;
            signal[i] = 5.0 * (-0.5 * x * x).exp();
        }
        let result = processor.process(&signal);
        assert!(!result.peaks.is_empty());
    }

    // ── FWHM and Asymmetry helpers ───────────────────────────────────────

    #[test]
    fn test_fwhm_symmetric_peak() {
        let mut signal = vec![0.0; 100];
        for i in 0..100 {
            let x = (i as f64 - 50.0) / 5.0;
            signal[i] = 10.0 * (-0.5 * x * x).exp();
        }
        let fwhm = compute_fwhm(&signal, 0, 50, 99);
        // FWHM of Gaussian = 2 * sqrt(2 * ln(2)) * sigma ≈ 2.355 * sigma
        // With sigma = 5 samples → FWHM ≈ 11.77 samples
        assert!(fwhm > 10.0 && fwhm < 14.0);
    }

    #[test]
    fn test_asymmetry_symmetric() {
        let mut signal = vec![0.0; 100];
        for i in 0..100 {
            let x = (i as f64 - 50.0) / 5.0;
            signal[i] = 10.0 * (-0.5 * x * x).exp();
        }
        let asym = compute_asymmetry(&signal, 0, 50, 99);
        // Symmetric peak → asymmetry ≈ 1.0
        assert!((asym - 1.0).abs() < 0.3);
    }

    // ── Integration: edge cases ──────────────────────────────────────────

    #[test]
    fn test_simpsons_three_points() {
        // f(x) = x^2 from 0 to 2: integral = 8/3
        let signal = vec![0.0, 1.0, 4.0];
        let area = peak_area_simpsons(&signal, 1.0);
        assert!((area - 8.0 / 3.0).abs() < APPROX);
    }

    #[test]
    fn test_simpsons_four_points() {
        // Even number: 3 Simpson + 1 trapezoidal
        let signal = vec![0.0, 1.0, 4.0, 9.0]; // x^2 at 0,1,2,3
        let area = peak_area_simpsons(&signal, 1.0);
        // Simpson on 0..2 = 8/3, trap on 2..3 = 0.5*(4+9) = 6.5
        let expected = 8.0 / 3.0 + 6.5;
        assert!((area - expected).abs() < APPROX);
    }

    // ── Comprehensive pipeline test ──────────────────────────────────────

    #[test]
    fn test_full_pipeline_roundtrip() {
        // Create a synthetic chromatogram
        let n = 500;
        let dt = 0.1; // 10 Hz
        let mut signal = vec![0.0; n];

        // Add baseline drift
        for i in 0..n {
            signal[i] += 0.001 * i as f64;
        }

        // Add two Gaussian peaks
        for i in 0..n {
            let x1 = (i as f64 - 150.0) / 10.0;
            let x2 = (i as f64 - 350.0) / 15.0;
            signal[i] += 5.0 * (-0.5 * x1 * x1).exp();
            signal[i] += 3.0 * (-0.5 * x2 * x2).exp();
        }

        // Baseline correction
        let corrector = BaselineDriftCorrector::new(1);
        let corrected = corrector.correct(&signal);

        // Peak detection
        let detector = PeakDetector::new(0.01, 0.5, 5);
        let peaks = detector.detect(&corrected);
        assert!(!peaks.is_empty(), "Should detect at least one peak");

        // Integration
        let integrator = PeakIntegrator::new(IntegrationMethod::Trapezoidal, dt);
        for peak in &peaks {
            let area = integrator.integrate_peak(&corrected, peak);
            assert!(area > 0.0, "Peak area should be positive");
        }
    }

    // ── Gauss solve ──────────────────────────────────────────────────────

    #[test]
    fn test_gauss_solve_identity() {
        let mut a = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let mut b = vec![3.0, 5.0];
        let x = gauss_solve(&mut a, &mut b);
        assert!((x[0] - 3.0).abs() < EPSILON);
        assert!((x[1] - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_gauss_solve_2x2() {
        // x + 2y = 5, 3x + 4y = 11 → x=1, y=2
        let mut a = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let mut b = vec![5.0, 11.0];
        let x = gauss_solve(&mut a, &mut b);
        assert!((x[0] - 1.0).abs() < APPROX);
        assert!((x[1] - 2.0).abs() < APPROX);
    }

    #[test]
    fn test_gauss_solve_empty() {
        let mut a: Vec<Vec<f64>> = vec![];
        let mut b: Vec<f64> = vec![];
        let x = gauss_solve(&mut a, &mut b);
        assert!(x.is_empty());
    }
}

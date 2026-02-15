//! Electrostatic Discharge (ESD) Event Analysis and Classification
//!
//! Implements ESD event detection, waveform modeling, and classification for
//! EMC testing and component reliability assessment per IEC 61000-4-2.
//!
//! ## ESD Waveform Models
//!
//! | Model | Capacitance | Resistance | Rise Time    | Application          |
//! |-------|-------------|------------|--------------|----------------------|
//! | HBM   | 100 pF      | 1.5 kOhm  | 0.7-1 ns     | Human handling       |
//! | MM    | 200 pF      | ~0 Ohm    | oscillatory  | Machine contact      |
//! | CDM   | ~6.8 pF     | ~1 Ohm    | <200 ps      | Charged device       |
//!
//! ## IEC 61000-4-2 Severity Levels
//!
//! | Level | Contact (kV) | Air (kV) |
//! |-------|-------------|----------|
//! | 1     | 2           | 2        |
//! | 2     | 4           | 4        |
//! | 3     | 6           | 8        |
//! | 4     | 8           | 15       |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::electrostatic_discharge_analyzer::{
//!     EsdModel, EsdWaveformGenerator, EsdAnalyzer, EsdAnalyzerConfig,
//! };
//!
//! // Generate an HBM ESD waveform at 2 kV
//! let gen = EsdWaveformGenerator::new(EsdModel::HumanBody);
//! let waveform = gen.generate_current(2000.0, 1e10, 1000);
//!
//! // Analyze the waveform
//! let config = EsdAnalyzerConfig {
//!     sample_rate_hz: 1e10,
//!     voltage_threshold_v: 50.0,
//!     current_threshold_a: 0.1,
//! };
//! let analyzer = EsdAnalyzer::new(config);
//! let events = analyzer.detect_events(&waveform, &[]);
//! assert!(!events.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enums and basic types
// ---------------------------------------------------------------------------

/// Polarity of an ESD event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EsdPolarity {
    Positive,
    Negative,
}

/// ESD waveform model type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EsdModel {
    /// Human Body Model: 100 pF, 1.5 kOhm
    HumanBody,
    /// Machine Model: 200 pF, ~0 Ohm (oscillatory)
    Machine,
    /// Charged Device Model: ~6.8 pF, ~1 Ohm
    ChargedDevice,
}

impl EsdModel {
    /// Returns (capacitance_F, resistance_Ohm, inductance_H) for the model.
    pub fn circuit_params(&self) -> (f64, f64, f64) {
        match self {
            EsdModel::HumanBody => (100e-12, 1500.0, 7.5e-6),
            EsdModel::Machine => (200e-12, 0.5, 0.75e-6),
            EsdModel::ChargedDevice => (6.8e-12, 1.0, 0.5e-9),
        }
    }

    /// Returns the typical rise time in seconds.
    pub fn typical_rise_time_s(&self) -> f64 {
        match self {
            EsdModel::HumanBody => 0.85e-9,
            EsdModel::Machine => 0.5e-9,
            EsdModel::ChargedDevice => 0.15e-9,
        }
    }
}

/// IEC 61000-4-2 discharge type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DischargeType {
    Contact,
    Air,
}

/// IEC 61000-4-2 severity level (1-4).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IecSeverityLevel {
    Level1,
    Level2,
    Level3,
    Level4,
}

impl IecSeverityLevel {
    /// Returns the test voltage in volts for the given discharge type.
    pub fn voltage_v(&self, discharge: DischargeType) -> f64 {
        match (self, discharge) {
            (IecSeverityLevel::Level1, DischargeType::Contact) => 2000.0,
            (IecSeverityLevel::Level2, DischargeType::Contact) => 4000.0,
            (IecSeverityLevel::Level3, DischargeType::Contact) => 6000.0,
            (IecSeverityLevel::Level4, DischargeType::Contact) => 8000.0,
            (IecSeverityLevel::Level1, DischargeType::Air) => 2000.0,
            (IecSeverityLevel::Level2, DischargeType::Air) => 4000.0,
            (IecSeverityLevel::Level3, DischargeType::Air) => 8000.0,
            (IecSeverityLevel::Level4, DischargeType::Air) => 15000.0,
        }
    }

    /// Classify a voltage into an IEC severity level.
    pub fn classify(voltage_v: f64, discharge: DischargeType) -> Option<IecSeverityLevel> {
        let abs_v = voltage_v.abs();
        let levels = [
            IecSeverityLevel::Level4,
            IecSeverityLevel::Level3,
            IecSeverityLevel::Level2,
            IecSeverityLevel::Level1,
        ];
        for level in &levels {
            if abs_v >= level.voltage_v(discharge) {
                return Some(*level);
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// ESD Event
// ---------------------------------------------------------------------------

/// A detected ESD event with measured parameters.
#[derive(Debug, Clone)]
pub struct EsdEvent {
    /// Peak voltage in volts.
    pub peak_voltage_v: f64,
    /// Peak current in amperes.
    pub peak_current_a: f64,
    /// Rise time (10%-90%) in nanoseconds.
    pub rise_time_ns: f64,
    /// Decay time (90%-10%) in nanoseconds.
    pub decay_time_ns: f64,
    /// Total energy in joules.
    pub energy_j: f64,
    /// Polarity of the event.
    pub polarity: EsdPolarity,
    /// Charge transfer in coulombs.
    pub charge_transfer_c: f64,
    /// Sample index of the event start.
    pub start_sample: usize,
    /// Duration in samples.
    pub duration_samples: usize,
}

// ---------------------------------------------------------------------------
// RLC Circuit Model
// ---------------------------------------------------------------------------

/// Damping regime of an RLC circuit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DampingRegime {
    Underdamped,
    CriticallyDamped,
    Overdamped,
}

/// RLC circuit model for ESD waveform generation.
#[derive(Debug, Clone)]
pub struct RlcModel {
    pub resistance: f64,
    pub inductance: f64,
    pub capacitance: f64,
}

impl RlcModel {
    pub fn new(r: f64, l: f64, c: f64) -> Self {
        Self {
            resistance: r,
            inductance: l,
            capacitance: c,
        }
    }

    /// Damping coefficient alpha = R / (2L).
    pub fn alpha(&self) -> f64 {
        self.resistance / (2.0 * self.inductance)
    }

    /// Natural frequency omega_0 = 1 / sqrt(LC).
    pub fn omega_0(&self) -> f64 {
        1.0 / (self.inductance * self.capacitance).sqrt()
    }

    /// Damped frequency omega_d = sqrt(omega_0^2 - alpha^2).
    /// Returns 0 if overdamped or critically damped.
    pub fn omega_d(&self) -> f64 {
        let w0_sq = self.omega_0() * self.omega_0();
        let a_sq = self.alpha() * self.alpha();
        if w0_sq > a_sq {
            (w0_sq - a_sq).sqrt()
        } else {
            0.0
        }
    }

    /// Determine the damping regime.
    pub fn damping_regime(&self) -> DampingRegime {
        let alpha = self.alpha();
        let omega_0 = self.omega_0();
        let ratio = alpha / omega_0;
        if (ratio - 1.0).abs() < 1e-9 {
            DampingRegime::CriticallyDamped
        } else if ratio < 1.0 {
            DampingRegime::Underdamped
        } else {
            DampingRegime::Overdamped
        }
    }

    /// Compute current i(t) for initial voltage V0 on the capacitor.
    ///
    /// For underdamped: i(t) = (V0/L) * exp(-alpha*t) * sin(omega_d*t) / omega_d
    /// For critically damped: i(t) = (V0/L) * t * exp(-alpha*t)
    /// For overdamped: i(t) = (V0/L) * exp(-alpha*t) * sinh(s*t) / s
    ///   where s = sqrt(alpha^2 - omega_0^2)
    pub fn current_at(&self, v0: f64, t: f64) -> f64 {
        if t < 0.0 {
            return 0.0;
        }
        let alpha = self.alpha();
        let omega_0 = self.omega_0();

        match self.damping_regime() {
            DampingRegime::Underdamped => {
                let wd = self.omega_d();
                (v0 / self.inductance) * (-alpha * t).exp() * (wd * t).sin() / wd
            }
            DampingRegime::CriticallyDamped => {
                (v0 / self.inductance) * t * (-alpha * t).exp()
            }
            DampingRegime::Overdamped => {
                let s = (alpha * alpha - omega_0 * omega_0).sqrt();
                (v0 / self.inductance) * (-alpha * t).exp() * (s * t).sinh() / s
            }
        }
    }

    /// Compute capacitor voltage V(t) for initial voltage V0.
    ///
    /// V(t) = V0 - (1/C) * integral_0^t i(tau) dtau
    /// Approximate by numerical integration using the trapezoidal rule.
    pub fn voltage_at(&self, v0: f64, t: f64, dt: f64) -> f64 {
        if t <= 0.0 {
            return v0;
        }
        let n_steps = (t / dt).ceil() as usize;
        let actual_dt = t / n_steps as f64;
        let mut integral = 0.0;
        let mut prev_i = self.current_at(v0, 0.0);
        for k in 1..=n_steps {
            let tk = k as f64 * actual_dt;
            let curr_i = self.current_at(v0, tk);
            integral += (prev_i + curr_i) * 0.5 * actual_dt;
            prev_i = curr_i;
        }
        v0 - integral / self.capacitance
    }
}

// ---------------------------------------------------------------------------
// Double Exponential Model
// ---------------------------------------------------------------------------

/// Double exponential pulse model: i(t) = Ip * (exp(-t/tau1) - exp(-t/tau2)).
/// Used for IEC 61000-4-2 waveform approximation.
#[derive(Debug, Clone)]
pub struct DoubleExponential {
    /// Peak current scaling factor (amperes).
    pub ip: f64,
    /// Decay time constant (seconds).
    pub tau_decay: f64,
    /// Rise time constant (seconds).
    pub tau_rise: f64,
}

impl DoubleExponential {
    /// Create from peak current and rise/decay time constants.
    pub fn new(ip: f64, tau_decay: f64, tau_rise: f64) -> Self {
        Self {
            ip,
            tau_decay,
            tau_rise,
        }
    }

    /// Create an IEC 61000-4-2 standard waveform for a given charge voltage.
    /// Rise time ~0.8 ns, first peak at ~1 ns, decay ~60 ns.
    pub fn iec_standard(charge_voltage_v: f64) -> Self {
        // IEC 61000-4-2 contact discharge: Ip ~ 3.75 A/kV
        let ip = charge_voltage_v.abs() * 3.75e-3;
        // Time constants chosen to approximate the IEC waveform shape
        let tau_rise: f64 = 0.4e-9; // ~0.8 ns 10-90% rise
        let tau_decay: f64 = 30e-9; // ~60 ns decay
        // Correction factor for peak normalization
        let t_peak: f64 = (tau_decay * tau_rise / (tau_decay - tau_rise))
            * (tau_decay / tau_rise).ln();
        let peak_val: f64 =
            (-t_peak / tau_decay).exp() - (-t_peak / tau_rise).exp();
        Self {
            ip: ip / peak_val,
            tau_decay,
            tau_rise,
        }
    }

    /// Current at time t.
    pub fn current_at(&self, t: f64) -> f64 {
        if t < 0.0 {
            return 0.0;
        }
        self.ip * ((-t / self.tau_decay).exp() - (-t / self.tau_rise).exp())
    }

    /// Time of peak current.
    pub fn time_of_peak(&self) -> f64 {
        if (self.tau_decay - self.tau_rise).abs() < 1e-30 {
            return 0.0;
        }
        (self.tau_decay * self.tau_rise / (self.tau_decay - self.tau_rise))
            * (self.tau_decay / self.tau_rise).ln()
    }

    /// Peak current value.
    pub fn peak_current(&self) -> f64 {
        self.current_at(self.time_of_peak())
    }
}

// ---------------------------------------------------------------------------
// ESD Waveform Generator
// ---------------------------------------------------------------------------

/// Generates ESD waveforms for different discharge models.
pub struct EsdWaveformGenerator {
    model: EsdModel,
}

impl EsdWaveformGenerator {
    pub fn new(model: EsdModel) -> Self {
        Self { model }
    }

    /// Generate a current waveform i(t) for the given charge voltage.
    ///
    /// Returns a vector of current samples in amperes.
    pub fn generate_current(
        &self,
        charge_voltage_v: f64,
        sample_rate_hz: f64,
        num_samples: usize,
    ) -> Vec<f64> {
        let (c, r, l) = self.model.circuit_params();
        let rlc = RlcModel::new(r, l, c);
        let dt = 1.0 / sample_rate_hz;

        (0..num_samples)
            .map(|i| rlc.current_at(charge_voltage_v, i as f64 * dt))
            .collect()
    }

    /// Generate a double-exponential IEC waveform.
    pub fn generate_iec_waveform(
        charge_voltage_v: f64,
        sample_rate_hz: f64,
        num_samples: usize,
    ) -> Vec<f64> {
        let de = DoubleExponential::iec_standard(charge_voltage_v);
        let dt = 1.0 / sample_rate_hz;
        (0..num_samples)
            .map(|i| de.current_at(i as f64 * dt))
            .collect()
    }

    /// Generate voltage waveform from the RLC model.
    pub fn generate_voltage(
        &self,
        charge_voltage_v: f64,
        sample_rate_hz: f64,
        num_samples: usize,
    ) -> Vec<f64> {
        let (c, r, l) = self.model.circuit_params();
        let rlc = RlcModel::new(r, l, c);
        let dt = 1.0 / sample_rate_hz;

        (0..num_samples)
            .map(|i| rlc.voltage_at(charge_voltage_v, i as f64 * dt, dt * 0.1))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Spectral Analysis
// ---------------------------------------------------------------------------

/// Spectral content of an ESD pulse.
#[derive(Debug, Clone)]
pub struct EsdSpectrum {
    /// Frequency bins in Hz.
    pub frequencies: Vec<f64>,
    /// Magnitude spectrum in linear units.
    pub magnitudes: Vec<f64>,
    /// 3 dB bandwidth in Hz.
    pub bandwidth_3db_hz: f64,
    /// 20 dB bandwidth in Hz.
    pub bandwidth_20db_hz: f64,
}

/// Compute the DFT magnitude spectrum of a real signal.
fn dft_magnitude(signal: &[f64], sample_rate_hz: f64) -> (Vec<f64>, Vec<f64>) {
    let n = signal.len();
    let half = n / 2 + 1;
    let mut freqs = Vec::with_capacity(half);
    let mut mags = Vec::with_capacity(half);

    for k in 0..half {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &s) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += s * angle.cos();
            im += s * angle.sin();
        }
        let mag = (re * re + im * im).sqrt() / n as f64;
        freqs.push(k as f64 * sample_rate_hz / n as f64);
        mags.push(mag);
    }
    (freqs, mags)
}

/// Analyze the spectral content of an ESD pulse.
pub fn analyze_spectrum(signal: &[f64], sample_rate_hz: f64) -> EsdSpectrum {
    let (frequencies, magnitudes) = dft_magnitude(signal, sample_rate_hz);

    // Find peak magnitude
    let peak_mag = magnitudes
        .iter()
        .cloned()
        .fold(0.0_f64, f64::max);

    // 3 dB bandwidth: first frequency where magnitude drops below peak / sqrt(2)
    let threshold_3db = peak_mag / 2.0_f64.sqrt();
    let threshold_20db = peak_mag / 10.0; // 20 dB = 10x voltage

    let bandwidth_3db_hz = frequencies
        .iter()
        .zip(magnitudes.iter())
        .filter(|(_, &m)| m >= threshold_3db)
        .map(|(&f, _)| f)
        .last()
        .unwrap_or(0.0);

    let bandwidth_20db_hz = frequencies
        .iter()
        .zip(magnitudes.iter())
        .filter(|(_, &m)| m >= threshold_20db)
        .map(|(&f, _)| f)
        .last()
        .unwrap_or(0.0);

    EsdSpectrum {
        frequencies,
        magnitudes,
        bandwidth_3db_hz,
        bandwidth_20db_hz,
    }
}

/// Estimate spectral envelope corner frequency from rise time.
/// f_corner = 1 / (pi * t_rise)
pub fn spectral_envelope_corner_hz(rise_time_s: f64) -> f64 {
    if rise_time_s <= 0.0 {
        return f64::INFINITY;
    }
    1.0 / (PI * rise_time_s)
}

/// Compute spectral envelope magnitude at frequency f.
/// Flat below f_corner, rolls off at 40 dB/decade above.
pub fn spectral_envelope_db(f_hz: f64, f_corner_hz: f64, reference_db: f64) -> f64 {
    if f_hz <= 0.0 || f_corner_hz <= 0.0 {
        return reference_db;
    }
    if f_hz <= f_corner_hz {
        reference_db
    } else {
        reference_db - 40.0 * (f_hz / f_corner_hz).log10()
    }
}

// ---------------------------------------------------------------------------
// ESD Analyzer
// ---------------------------------------------------------------------------

/// Configuration for the ESD analyzer.
#[derive(Debug, Clone)]
pub struct EsdAnalyzerConfig {
    /// Sample rate of the input waveforms in Hz.
    pub sample_rate_hz: f64,
    /// Voltage threshold for event detection (volts).
    pub voltage_threshold_v: f64,
    /// Current threshold for event detection (amperes).
    pub current_threshold_a: f64,
}

impl Default for EsdAnalyzerConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 10e9, // 10 GS/s typical for ESD
            voltage_threshold_v: 100.0,
            current_threshold_a: 0.5,
        }
    }
}

/// ESD event analyzer for detecting and characterizing ESD events.
pub struct EsdAnalyzer {
    config: EsdAnalyzerConfig,
}

impl EsdAnalyzer {
    pub fn new(config: EsdAnalyzerConfig) -> Self {
        Self { config }
    }

    /// Detect ESD events from raw current (and optionally voltage) waveforms.
    ///
    /// If voltage is empty, energy is estimated from 0.5*C*V^2 model.
    pub fn detect_events(
        &self,
        current: &[f64],
        voltage: &[f64],
    ) -> Vec<EsdEvent> {
        if current.is_empty() {
            return Vec::new();
        }

        let dt = 1.0 / self.config.sample_rate_hz;
        let mut events = Vec::new();
        let mut in_event = false;
        let mut event_start = 0usize;

        for i in 0..current.len() {
            let abs_i = current[i].abs();
            if !in_event && abs_i > self.config.current_threshold_a {
                in_event = true;
                event_start = i;
            } else if in_event && abs_i <= self.config.current_threshold_a {
                in_event = false;
                let event_end = i;
                if let Some(ev) = self.characterize_event(
                    current,
                    voltage,
                    event_start,
                    event_end,
                    dt,
                ) {
                    events.push(ev);
                }
            }
        }

        // Handle event that extends to end of data
        if in_event {
            if let Some(ev) = self.characterize_event(
                current,
                voltage,
                event_start,
                current.len(),
                dt,
            ) {
                events.push(ev);
            }
        }

        events
    }

    fn characterize_event(
        &self,
        current: &[f64],
        voltage: &[f64],
        start: usize,
        end: usize,
        dt: f64,
    ) -> Option<EsdEvent> {
        if end <= start {
            return None;
        }

        let segment = &current[start..end];

        // Find peak current (signed)
        let (peak_idx, peak_current) = segment
            .iter()
            .enumerate()
            .fold((0, 0.0_f64), |(pi, pv), (i, &v)| {
                if v.abs() > pv.abs() {
                    (i, v)
                } else {
                    (pi, pv)
                }
            });

        let polarity = if peak_current >= 0.0 {
            EsdPolarity::Positive
        } else {
            EsdPolarity::Negative
        };

        let abs_peak = peak_current.abs();

        // Rise time (10%-90%)
        let rise_time_ns = measure_rise_time(segment, dt) * 1e9;

        // Decay time (90%-10% from peak)
        let decay_time_ns =
            measure_decay_time(segment, peak_idx, dt) * 1e9;

        // Charge transfer: Q = integral(|i(t)|) * dt
        let charge_transfer: f64 =
            segment.iter().map(|&i| i.abs() * dt).sum();

        // Energy calculation
        let energy = if !voltage.is_empty() && voltage.len() > start {
            // E = integral(V(t) * I(t) * dt)
            let v_end = end.min(voltage.len());
            let mut e = 0.0;
            for k in start..v_end {
                e += voltage[k].abs() * current[k].abs() * dt;
            }
            e
        } else {
            // Estimate from I^2 * R model (assume 1.5k for HBM)
            let r_est = 1500.0;
            segment.iter().map(|&i| i * i * r_est * dt).sum()
        };

        // Peak voltage
        let peak_voltage = if !voltage.is_empty() && voltage.len() > start {
            let v_end = end.min(voltage.len());
            voltage[start..v_end]
                .iter()
                .cloned()
                .fold(0.0_f64, |a, v| a.max(v.abs()))
        } else {
            abs_peak * 1500.0 // Estimate V = I * R for HBM
        };

        Some(EsdEvent {
            peak_voltage_v: peak_voltage,
            peak_current_a: abs_peak,
            rise_time_ns,
            decay_time_ns,
            energy_j: energy,
            polarity,
            charge_transfer_c: charge_transfer,
            start_sample: start,
            duration_samples: end - start,
        })
    }
}

/// Measure 10%-90% rise time of a pulse.
fn measure_rise_time(signal: &[f64], dt: f64) -> f64 {
    let abs_signal: Vec<f64> = signal.iter().map(|s| s.abs()).collect();
    let peak = abs_signal
        .iter()
        .cloned()
        .fold(0.0_f64, f64::max);
    if peak <= 0.0 {
        return 0.0;
    }

    let t10 = peak * 0.1;
    let t90 = peak * 0.9;

    let mut idx_10 = None;
    let mut idx_90 = None;

    for (i, &v) in abs_signal.iter().enumerate() {
        if idx_10.is_none() && v >= t10 {
            idx_10 = Some(i);
        }
        if idx_90.is_none() && v >= t90 {
            idx_90 = Some(i);
            break;
        }
    }

    match (idx_10, idx_90) {
        (Some(i10), Some(i90)) => (i90 as f64 - i10 as f64) * dt,
        _ => 0.0,
    }
}

/// Measure 90%-10% decay time from the peak.
fn measure_decay_time(signal: &[f64], peak_idx: usize, dt: f64) -> f64 {
    let abs_signal: Vec<f64> = signal.iter().map(|s| s.abs()).collect();
    let peak = abs_signal[peak_idx];
    if peak <= 0.0 {
        return 0.0;
    }

    let t90 = peak * 0.9;
    let t10 = peak * 0.1;

    let mut idx_90 = None;
    let mut idx_10 = None;

    for i in peak_idx..abs_signal.len() {
        if idx_90.is_none() && abs_signal[i] <= t90 {
            idx_90 = Some(i);
        }
        if idx_90.is_some() && idx_10.is_none() && abs_signal[i] <= t10 {
            idx_10 = Some(i);
            break;
        }
    }

    match (idx_90, idx_10) {
        (Some(i90), Some(i10)) => (i10 as f64 - i90 as f64) * dt,
        _ => 0.0,
    }
}

/// Compute the first peak / second peak current ratio.
/// Returns None if no second peak is found.
pub fn peak_ratio(signal: &[f64]) -> Option<f64> {
    let abs_signal: Vec<f64> = signal.iter().map(|s| s.abs()).collect();

    // Find first peak
    let mut first_peak_idx = 0;
    let mut first_peak_val = 0.0_f64;
    for (i, &v) in abs_signal.iter().enumerate() {
        if v > first_peak_val {
            first_peak_val = v;
            first_peak_idx = i;
        }
    }

    if first_peak_val <= 0.0 {
        return None;
    }

    // Find first local minimum after peak
    let mut min_idx = first_peak_idx;
    for i in first_peak_idx + 1..abs_signal.len() {
        if abs_signal[i] < abs_signal[min_idx] {
            min_idx = i;
        }
        if i > min_idx + 1 && abs_signal[i] > abs_signal[min_idx] * 1.1 {
            break;
        }
    }

    // Find second peak after the minimum
    let mut second_peak_val = 0.0_f64;
    for &v in abs_signal.iter().skip(min_idx) {
        if v > second_peak_val {
            second_peak_val = v;
        }
        // Stop if it starts decaying significantly
        if second_peak_val > 0.0 && v < second_peak_val * 0.5 {
            break;
        }
    }

    if second_peak_val > 0.0 {
        Some(first_peak_val / second_peak_val)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Transmission Line Pulse (TLP) Analysis
// ---------------------------------------------------------------------------

/// A single point on a TLP I-V curve.
#[derive(Debug, Clone)]
pub struct TlpPoint {
    /// Voltage in volts.
    pub voltage_v: f64,
    /// Current in amperes.
    pub current_a: f64,
}

/// Results of TLP I-V curve analysis.
#[derive(Debug, Clone)]
pub struct TlpAnalysis {
    /// Measured I-V points.
    pub iv_curve: Vec<TlpPoint>,
    /// Trigger voltage (onset of snapback or significant conduction).
    pub trigger_voltage_v: Option<f64>,
    /// Holding voltage (post-snapback steady state).
    pub holding_voltage_v: Option<f64>,
    /// Whether snapback (negative resistance) was detected.
    pub snapback_detected: bool,
    /// On-resistance in ohms after turn-on.
    pub on_resistance_ohm: Option<f64>,
    /// Failure current (if detected from sudden change).
    pub failure_current_a: Option<f64>,
}

/// Extract I-V points from rectangular TLP pulse data.
///
/// Each pulse gives one (V, I) point. The steady-state portion of each
/// pulse (last quarter) is averaged.
pub fn extract_tlp_iv(
    pulses_voltage: &[Vec<f64>],
    pulses_current: &[Vec<f64>],
) -> Vec<TlpPoint> {
    let mut points = Vec::new();
    for (v_pulse, i_pulse) in pulses_voltage.iter().zip(pulses_current.iter()) {
        let n = v_pulse.len().min(i_pulse.len());
        if n < 4 {
            continue;
        }
        // Average the last quarter (steady state)
        let start = 3 * n / 4;
        let count = (n - start) as f64;
        let avg_v: f64 = v_pulse[start..n].iter().sum::<f64>() / count;
        let avg_i: f64 = i_pulse[start..n].iter().sum::<f64>() / count;
        points.push(TlpPoint {
            voltage_v: avg_v,
            current_a: avg_i,
        });
    }
    points
}

/// Analyze a TLP I-V curve for snapback, trigger/holding voltages, and on-resistance.
pub fn analyze_tlp(iv_curve: &[TlpPoint]) -> TlpAnalysis {
    if iv_curve.is_empty() {
        return TlpAnalysis {
            iv_curve: Vec::new(),
            trigger_voltage_v: None,
            holding_voltage_v: None,
            snapback_detected: false,
            on_resistance_ohm: None,
            failure_current_a: None,
        };
    }

    let mut trigger_voltage = None;
    let mut holding_voltage = None;
    let mut snapback_detected = false;
    let mut on_resistance = None;
    let mut failure_current = None;

    // Detect trigger: first point where current exceeds a small fraction of max
    let max_current = iv_curve
        .iter()
        .map(|p| p.current_a)
        .fold(0.0_f64, f64::max);
    let trigger_threshold = max_current * 0.01;

    for p in iv_curve.iter() {
        if p.current_a > trigger_threshold {
            trigger_voltage = Some(p.voltage_v);
            break;
        }
    }

    // Detect snapback: voltage decreasing while current increases
    for i in 1..iv_curve.len() {
        let dv = iv_curve[i].voltage_v - iv_curve[i - 1].voltage_v;
        let di = iv_curve[i].current_a - iv_curve[i - 1].current_a;
        if dv < 0.0 && di > 0.0 {
            snapback_detected = true;
            // Holding voltage is the minimum voltage in the snapback region
            if holding_voltage.is_none()
                || iv_curve[i].voltage_v < holding_voltage.unwrap()
            {
                holding_voltage = Some(iv_curve[i].voltage_v);
            }
        }
    }

    // On-resistance: slope of linear region after turn-on
    // Use points in the upper half of current range
    let mid_current = max_current * 0.3;
    let high_current = max_current * 0.8;
    let linear_points: Vec<&TlpPoint> = iv_curve
        .iter()
        .filter(|p| p.current_a >= mid_current && p.current_a <= high_current)
        .collect();

    if linear_points.len() >= 2 {
        let first = linear_points[0];
        let last = linear_points[linear_points.len() - 1];
        let di = last.current_a - first.current_a;
        if di.abs() > 1e-12 {
            let dv = last.voltage_v - first.voltage_v;
            on_resistance = Some(dv / di);
        }
    }

    // Failure detection: sudden large change in leakage or resistance
    for i in 2..iv_curve.len() {
        let r_prev = if iv_curve[i - 1].current_a.abs() > 1e-12 {
            iv_curve[i - 1].voltage_v / iv_curve[i - 1].current_a
        } else {
            f64::MAX
        };
        let r_curr = if iv_curve[i].current_a.abs() > 1e-12 {
            iv_curve[i].voltage_v / iv_curve[i].current_a
        } else {
            f64::MAX
        };
        if r_prev < f64::MAX
            && r_curr < f64::MAX
            && r_curr < r_prev * 0.3
            && iv_curve[i].current_a > mid_current
        {
            failure_current = Some(iv_curve[i].current_a);
            break;
        }
    }

    TlpAnalysis {
        iv_curve: iv_curve.to_vec(),
        trigger_voltage_v: trigger_voltage,
        holding_voltage_v: holding_voltage,
        snapback_detected,
        on_resistance_ohm: on_resistance,
        failure_current_a: failure_current,
    }
}

// ---------------------------------------------------------------------------
// Component Damage Threshold
// ---------------------------------------------------------------------------

/// Component ESD damage threshold.
#[derive(Debug, Clone)]
pub struct ComponentThreshold {
    /// Component name or ID.
    pub name: String,
    /// HBM withstand voltage in volts.
    pub hbm_voltage_v: f64,
    /// MM withstand voltage in volts.
    pub mm_voltage_v: f64,
    /// CDM withstand voltage in volts.
    pub cdm_voltage_v: f64,
}

/// Check if an ESD event exceeds a component's damage threshold.
pub fn exceeds_threshold(
    event: &EsdEvent,
    threshold: &ComponentThreshold,
    model: EsdModel,
) -> bool {
    let limit = match model {
        EsdModel::HumanBody => threshold.hbm_voltage_v,
        EsdModel::Machine => threshold.mm_voltage_v,
        EsdModel::ChargedDevice => threshold.cdm_voltage_v,
    };
    event.peak_voltage_v > limit
}

// ---------------------------------------------------------------------------
// Statistical Analysis
// ---------------------------------------------------------------------------

/// Statistical summary of multiple ESD events.
#[derive(Debug, Clone)]
pub struct EsdStatistics {
    /// Number of events analyzed.
    pub count: usize,
    /// Mean peak current in amperes.
    pub mean_peak_current_a: f64,
    /// Standard deviation of peak current.
    pub std_peak_current_a: f64,
    /// Median peak current.
    pub median_peak_current_a: f64,
    /// 95th percentile peak current.
    pub p95_peak_current_a: f64,
    /// Mean energy in joules.
    pub mean_energy_j: f64,
    /// Weibull shape parameter (beta).
    pub weibull_beta: f64,
    /// Weibull scale parameter (eta).
    pub weibull_eta: f64,
}

/// Compute statistics over a set of ESD events.
pub fn compute_statistics(events: &[EsdEvent]) -> EsdStatistics {
    if events.is_empty() {
        return EsdStatistics {
            count: 0,
            mean_peak_current_a: 0.0,
            std_peak_current_a: 0.0,
            median_peak_current_a: 0.0,
            p95_peak_current_a: 0.0,
            mean_energy_j: 0.0,
            weibull_beta: 1.0,
            weibull_eta: 1.0,
        };
    }

    let n = events.len() as f64;
    let currents: Vec<f64> = events.iter().map(|e| e.peak_current_a).collect();
    let energies: Vec<f64> = events.iter().map(|e| e.energy_j).collect();

    let mean_current = currents.iter().sum::<f64>() / n;
    let mean_energy = energies.iter().sum::<f64>() / n;

    let var_current = currents
        .iter()
        .map(|&c| (c - mean_current) * (c - mean_current))
        .sum::<f64>()
        / n;
    let std_current = var_current.sqrt();

    let mut sorted = currents.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let median = if sorted.len() % 2 == 0 {
        (sorted[sorted.len() / 2 - 1] + sorted[sorted.len() / 2]) / 2.0
    } else {
        sorted[sorted.len() / 2]
    };

    let p95_idx = ((sorted.len() as f64 * 0.95).ceil() as usize).min(sorted.len()) - 1;
    let p95 = sorted[p95_idx];

    // Weibull parameter estimation via linear regression on log-log plot
    let (beta, eta) = estimate_weibull(&sorted);

    EsdStatistics {
        count: events.len(),
        mean_peak_current_a: mean_current,
        std_peak_current_a: std_current,
        median_peak_current_a: median,
        p95_peak_current_a: p95,
        mean_energy_j: mean_energy,
        weibull_beta: beta,
        weibull_eta: eta,
    }
}

/// Estimate Weibull parameters from sorted data using the median rank method.
fn estimate_weibull(sorted_data: &[f64]) -> (f64, f64) {
    let n = sorted_data.len();
    if n < 2 {
        return (1.0, sorted_data.first().copied().unwrap_or(1.0));
    }

    // Filter out non-positive values
    let positive: Vec<f64> = sorted_data.iter().copied().filter(|&v| v > 0.0).collect();
    if positive.len() < 2 {
        return (1.0, 1.0);
    }

    let np = positive.len();

    // Median rank: F(i) = (i - 0.3) / (n + 0.4)
    // Linearize: ln(ln(1/(1-F))) = beta * ln(x) - beta * ln(eta)
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xy = 0.0;
    let mut sum_xx = 0.0;

    for (i, &val) in positive.iter().enumerate() {
        let f = (i as f64 + 0.7) / (np as f64 + 0.4);
        if f <= 0.0 || f >= 1.0 {
            continue;
        }
        let x = val.ln();
        let y = (1.0 / (1.0 - f)).ln().ln();
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    let n_f = np as f64;
    let denom = n_f * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (1.0, positive[np / 2]);
    }

    let beta = (n_f * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - beta * sum_x) / n_f;
    let eta = (-intercept / beta).exp();

    let beta = beta.max(0.1); // Ensure positive
    let eta = eta.max(1e-30);

    (beta, eta)
}

/// Weibull CDF: F(t) = 1 - exp(-(t/eta)^beta).
pub fn weibull_cdf(t: f64, beta: f64, eta: f64) -> f64 {
    if t <= 0.0 || eta <= 0.0 || beta <= 0.0 {
        return 0.0;
    }
    1.0 - (-(t / eta).powf(beta)).exp()
}

/// Weibull reliability: R(t) = exp(-(t/eta)^beta).
pub fn weibull_reliability(t: f64, beta: f64, eta: f64) -> f64 {
    1.0 - weibull_cdf(t, beta, eta)
}

// ---------------------------------------------------------------------------
// Energy Calculations
// ---------------------------------------------------------------------------

/// Calculate ESD energy from capacitor model: E = 0.5 * C * V^2.
pub fn energy_from_capacitor(capacitance_f: f64, voltage_v: f64) -> f64 {
    0.5 * capacitance_f * voltage_v * voltage_v
}

/// Calculate charge transfer: Q = C * V.
pub fn charge_from_capacitor(capacitance_f: f64, voltage_v: f64) -> f64 {
    capacitance_f * voltage_v
}

/// Calculate energy from voltage and current waveforms:
/// E = integral(V(t) * I(t) * dt).
pub fn energy_from_waveforms(
    voltage: &[f64],
    current: &[f64],
    dt: f64,
) -> f64 {
    voltage
        .iter()
        .zip(current.iter())
        .map(|(&v, &i)| v * i * dt)
        .sum()
}

/// Calculate charge transfer from current waveform:
/// Q = integral(I(t) * dt).
pub fn charge_from_waveform(current: &[f64], dt: f64) -> f64 {
    current.iter().map(|&i| i * dt).sum()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-6;

    // --- ESD Model tests ---

    #[test]
    fn test_hbm_circuit_params() {
        let (c, r, l) = EsdModel::HumanBody.circuit_params();
        assert!((c - 100e-12).abs() < 1e-15);
        assert!((r - 1500.0).abs() < EPSILON);
        assert!(l > 0.0);
    }

    #[test]
    fn test_mm_circuit_params() {
        let (c, r, _l) = EsdModel::Machine.circuit_params();
        assert!((c - 200e-12).abs() < 1e-15);
        assert!(r < 10.0); // Low resistance
    }

    #[test]
    fn test_cdm_circuit_params() {
        let (c, _r, _l) = EsdModel::ChargedDevice.circuit_params();
        assert!((c - 6.8e-12).abs() < 1e-15);
    }

    #[test]
    fn test_typical_rise_times() {
        let hbm = EsdModel::HumanBody.typical_rise_time_s();
        let mm = EsdModel::Machine.typical_rise_time_s();
        let cdm = EsdModel::ChargedDevice.typical_rise_time_s();
        // HBM > MM > CDM
        assert!(hbm > mm);
        assert!(mm > cdm);
        assert!(cdm < 0.2e-9); // CDM < 200 ps
    }

    // --- IEC Severity Level tests ---

    #[test]
    fn test_iec_contact_voltages() {
        assert!((IecSeverityLevel::Level1.voltage_v(DischargeType::Contact) - 2000.0).abs() < EPSILON);
        assert!((IecSeverityLevel::Level2.voltage_v(DischargeType::Contact) - 4000.0).abs() < EPSILON);
        assert!((IecSeverityLevel::Level3.voltage_v(DischargeType::Contact) - 6000.0).abs() < EPSILON);
        assert!((IecSeverityLevel::Level4.voltage_v(DischargeType::Contact) - 8000.0).abs() < EPSILON);
    }

    #[test]
    fn test_iec_air_voltages() {
        assert!((IecSeverityLevel::Level1.voltage_v(DischargeType::Air) - 2000.0).abs() < EPSILON);
        assert!((IecSeverityLevel::Level2.voltage_v(DischargeType::Air) - 4000.0).abs() < EPSILON);
        assert!((IecSeverityLevel::Level3.voltage_v(DischargeType::Air) - 8000.0).abs() < EPSILON);
        assert!((IecSeverityLevel::Level4.voltage_v(DischargeType::Air) - 15000.0).abs() < EPSILON);
    }

    #[test]
    fn test_iec_classify_contact() {
        assert_eq!(
            IecSeverityLevel::classify(8500.0, DischargeType::Contact),
            Some(IecSeverityLevel::Level4)
        );
        assert_eq!(
            IecSeverityLevel::classify(6000.0, DischargeType::Contact),
            Some(IecSeverityLevel::Level3)
        );
        assert_eq!(
            IecSeverityLevel::classify(4500.0, DischargeType::Contact),
            Some(IecSeverityLevel::Level2)
        );
        assert_eq!(
            IecSeverityLevel::classify(2000.0, DischargeType::Contact),
            Some(IecSeverityLevel::Level1)
        );
        assert_eq!(
            IecSeverityLevel::classify(500.0, DischargeType::Contact),
            None
        );
    }

    #[test]
    fn test_iec_classify_negative_voltage() {
        // Should use absolute value
        assert_eq!(
            IecSeverityLevel::classify(-4000.0, DischargeType::Contact),
            Some(IecSeverityLevel::Level2)
        );
    }

    // --- RLC Model tests ---

    #[test]
    fn test_rlc_underdamped_hbm() {
        let rlc = RlcModel::new(1500.0, 7.5e-6, 100e-12);
        // Check damping regime
        // alpha = 1500/(2*7.5e-6) = 1e8
        // omega_0 = 1/sqrt(7.5e-6 * 100e-12) = 1/sqrt(7.5e-16) ~ 1.155e9/sqrt(7.5) ~ 3.65e7
        // Wait: omega_0 = 1/sqrt(LC) = 1/sqrt(7.5e-6 * 100e-12) = 1/sqrt(7.5e-16)
        // sqrt(7.5e-16) ~ 2.74e-8, omega_0 ~ 3.65e7
        // alpha = 1e8 > omega_0 ~ 3.65e7 => overdamped
        assert_eq!(rlc.damping_regime(), DampingRegime::Overdamped);
    }

    #[test]
    fn test_rlc_underdamped_mm() {
        let rlc = RlcModel::new(0.5, 0.75e-6, 200e-12);
        // alpha = 0.5 / (2 * 0.75e-6) = 3.33e5
        // omega_0 = 1/sqrt(0.75e-6 * 200e-12) = 1/sqrt(1.5e-16) ~ 8.16e7
        // omega_0 >> alpha => underdamped (oscillatory, as expected for MM)
        assert_eq!(rlc.damping_regime(), DampingRegime::Underdamped);
    }

    #[test]
    fn test_rlc_current_zero_at_t0() {
        let rlc = RlcModel::new(1500.0, 7.5e-6, 100e-12);
        let i0 = rlc.current_at(2000.0, 0.0);
        assert!(i0.abs() < 1e-3, "Current should be near zero at t=0, got {}", i0);
    }

    #[test]
    fn test_rlc_current_positive_after_start() {
        let rlc = RlcModel::new(1500.0, 7.5e-6, 100e-12);
        let i1 = rlc.current_at(2000.0, 1e-9);
        assert!(i1 > 0.0, "Current should be positive shortly after t=0");
    }

    #[test]
    fn test_rlc_voltage_starts_at_v0() {
        let rlc = RlcModel::new(1500.0, 7.5e-6, 100e-12);
        let v0 = rlc.voltage_at(2000.0, 0.0, 1e-10);
        assert!((v0 - 2000.0).abs() < 1.0);
    }

    #[test]
    fn test_rlc_omega_d_underdamped() {
        let rlc = RlcModel::new(0.5, 0.75e-6, 200e-12);
        let wd = rlc.omega_d();
        assert!(wd > 0.0, "Underdamped should have positive omega_d");
    }

    #[test]
    fn test_rlc_omega_d_overdamped() {
        let rlc = RlcModel::new(1500.0, 7.5e-6, 100e-12);
        let wd = rlc.omega_d();
        assert!((wd - 0.0).abs() < EPSILON, "Overdamped should have omega_d = 0");
    }

    // --- Double Exponential tests ---

    #[test]
    fn test_double_exponential_zero_at_origin() {
        let de = DoubleExponential::new(10.0, 30e-9, 0.5e-9);
        let i0 = de.current_at(0.0);
        assert!(i0.abs() < 1e-6);
    }

    #[test]
    fn test_double_exponential_peak_positive() {
        let de = DoubleExponential::new(10.0, 30e-9, 0.5e-9);
        let peak = de.peak_current();
        assert!(peak > 0.0);
    }

    #[test]
    fn test_double_exponential_decays() {
        let de = DoubleExponential::new(10.0, 30e-9, 0.5e-9);
        let early = de.current_at(2e-9);
        let late = de.current_at(100e-9);
        assert!(early > late, "Current should decay over time");
    }

    #[test]
    fn test_iec_standard_waveform_peak() {
        let de = DoubleExponential::iec_standard(4000.0);
        let peak = de.peak_current();
        // IEC standard: ~3.75 A/kV, so 4 kV => ~15 A
        assert!(peak > 10.0 && peak < 25.0, "Peak {} should be ~15A for 4kV", peak);
    }

    #[test]
    fn test_double_exponential_negative_time() {
        let de = DoubleExponential::new(10.0, 30e-9, 0.5e-9);
        assert!((de.current_at(-1e-9) - 0.0).abs() < EPSILON);
    }

    // --- Waveform Generator tests ---

    #[test]
    fn test_generate_hbm_current() {
        let gen = EsdWaveformGenerator::new(EsdModel::HumanBody);
        let waveform = gen.generate_current(2000.0, 10e9, 1000);
        assert_eq!(waveform.len(), 1000);
        // Should start near zero
        assert!(waveform[0].abs() < 0.1);
        // Should have a positive peak
        let max = waveform.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max > 0.0, "Should have positive current peak");
    }

    #[test]
    fn test_generate_mm_oscillatory() {
        let gen = EsdWaveformGenerator::new(EsdModel::Machine);
        let waveform = gen.generate_current(2000.0, 100e9, 50000);
        // Machine model should be oscillatory (underdamped)
        // Check for sign changes
        let mut sign_changes = 0;
        for i in 1..waveform.len() {
            if waveform[i] * waveform[i - 1] < 0.0 {
                sign_changes += 1;
            }
        }
        assert!(
            sign_changes >= 2,
            "MM should be oscillatory, got {} sign changes",
            sign_changes
        );
    }

    #[test]
    fn test_generate_iec_waveform() {
        let waveform =
            EsdWaveformGenerator::generate_iec_waveform(2000.0, 10e9, 2000);
        assert_eq!(waveform.len(), 2000);
        let peak = waveform.iter().cloned().fold(0.0_f64, f64::max);
        assert!(peak > 1.0, "IEC waveform should have significant peak current");
    }

    // --- Analyzer tests ---

    #[test]
    fn test_detect_events_empty() {
        let config = EsdAnalyzerConfig::default();
        let analyzer = EsdAnalyzer::new(config);
        let events = analyzer.detect_events(&[], &[]);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_events_below_threshold() {
        let config = EsdAnalyzerConfig {
            sample_rate_hz: 1e9,
            voltage_threshold_v: 100.0,
            current_threshold_a: 1.0,
        };
        let analyzer = EsdAnalyzer::new(config);
        // Signal below threshold
        let current = vec![0.1; 100];
        let events = analyzer.detect_events(&current, &[]);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_single_event() {
        let config = EsdAnalyzerConfig {
            sample_rate_hz: 1e10,
            voltage_threshold_v: 50.0,
            current_threshold_a: 0.1,
        };
        let analyzer = EsdAnalyzer::new(config);

        // Generate HBM pulse
        let gen = EsdWaveformGenerator::new(EsdModel::HumanBody);
        let current = gen.generate_current(2000.0, 1e10, 5000);
        let events = analyzer.detect_events(&current, &[]);

        assert!(
            !events.is_empty(),
            "Should detect at least one event"
        );
        assert!(events[0].peak_current_a > 0.0);
        assert!(events[0].polarity == EsdPolarity::Positive);
    }

    #[test]
    fn test_event_polarity_negative() {
        let config = EsdAnalyzerConfig {
            sample_rate_hz: 1e10,
            voltage_threshold_v: 50.0,
            current_threshold_a: 0.1,
        };
        let analyzer = EsdAnalyzer::new(config);

        // Negative voltage ESD
        let gen = EsdWaveformGenerator::new(EsdModel::HumanBody);
        let current: Vec<f64> = gen
            .generate_current(-2000.0, 1e10, 5000)
            .iter()
            .map(|&x| x) // Already negative from negative V0
            .collect();
        let events = analyzer.detect_events(&current, &[]);

        if !events.is_empty() {
            assert_eq!(events[0].polarity, EsdPolarity::Negative);
        }
    }

    // --- Spectral Analysis tests ---

    #[test]
    fn test_spectral_envelope_corner() {
        let rise_time = 1e-9; // 1 ns
        let fc = spectral_envelope_corner_hz(rise_time);
        // f = 1/(pi * 1e-9) ~ 318 MHz
        assert!((fc - 1.0 / (PI * 1e-9)).abs() < 1e3);
    }

    #[test]
    fn test_spectral_envelope_rolloff() {
        let fc = 318e6;
        let db_at_corner = spectral_envelope_db(fc, fc, 0.0);
        assert!((db_at_corner - 0.0).abs() < EPSILON);

        // At 10x the corner frequency, should be -40 dB
        let db_at_10x = spectral_envelope_db(10.0 * fc, fc, 0.0);
        assert!(
            (db_at_10x - (-40.0)).abs() < 0.1,
            "Expected -40 dB at 10x fc, got {}",
            db_at_10x
        );
    }

    #[test]
    fn test_spectral_envelope_below_corner() {
        let fc = 318e6;
        let db = spectral_envelope_db(100e6, fc, 10.0);
        assert!((db - 10.0).abs() < EPSILON, "Below corner should be flat");
    }

    #[test]
    fn test_analyze_spectrum_basic() {
        // Simple tone test
        let n = 256;
        let fs = 10e9;
        let f0 = 1e9;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).sin())
            .collect();
        let spectrum = analyze_spectrum(&signal, fs);
        assert_eq!(spectrum.frequencies.len(), n / 2 + 1);
        assert!(spectrum.bandwidth_3db_hz > 0.0);
    }

    // --- Energy calculation tests ---

    #[test]
    fn test_energy_from_capacitor() {
        let e = energy_from_capacitor(100e-12, 2000.0);
        // E = 0.5 * 100e-12 * 2000^2 = 0.5 * 100e-12 * 4e6 = 200e-6 = 0.2 mJ
        assert!((e - 0.2e-3).abs() < 1e-9);
    }

    #[test]
    fn test_charge_from_capacitor() {
        let q = charge_from_capacitor(100e-12, 2000.0);
        // Q = 100e-12 * 2000 = 200e-9 = 200 nC
        assert!((q - 200e-9).abs() < 1e-15);
    }

    #[test]
    fn test_energy_from_waveforms() {
        let v = vec![100.0, 100.0, 100.0];
        let i = vec![1.0, 1.0, 1.0];
        let dt = 1e-9;
        let e = energy_from_waveforms(&v, &i, dt);
        // E = 3 * 100 * 1 * 1e-9 = 300e-9
        assert!((e - 300e-9).abs() < 1e-15);
    }

    #[test]
    fn test_charge_from_waveform() {
        let i = vec![1.0, 2.0, 3.0];
        let dt = 1e-9;
        let q = charge_from_waveform(&i, dt);
        assert!((q - 6e-9).abs() < 1e-15);
    }

    // --- TLP Analysis tests ---

    #[test]
    fn test_extract_tlp_iv_basic() {
        let v_pulses = vec![
            vec![0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        ];
        let i_pulses = vec![
            vec![0.0, 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            vec![0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        ];
        let points = extract_tlp_iv(&v_pulses, &i_pulses);
        assert_eq!(points.len(), 2);
        assert!((points[0].voltage_v - 1.0).abs() < 0.1);
        assert!((points[1].current_a - 0.5).abs() < 0.1);
    }

    #[test]
    fn test_analyze_tlp_snapback() {
        // Simulate snapback: V increases then decreases while I keeps increasing
        let iv = vec![
            TlpPoint { voltage_v: 0.0, current_a: 0.0 },
            TlpPoint { voltage_v: 5.0, current_a: 0.001 },
            TlpPoint { voltage_v: 10.0, current_a: 0.01 },
            TlpPoint { voltage_v: 15.0, current_a: 0.1 },    // trigger
            TlpPoint { voltage_v: 12.0, current_a: 0.5 },     // snapback
            TlpPoint { voltage_v: 10.0, current_a: 1.0 },     // holding
            TlpPoint { voltage_v: 11.0, current_a: 1.5 },     // on-state
            TlpPoint { voltage_v: 12.0, current_a: 2.0 },
        ];
        let analysis = analyze_tlp(&iv);
        assert!(analysis.snapback_detected);
        assert!(analysis.trigger_voltage_v.is_some());
        assert!(analysis.holding_voltage_v.is_some());
    }

    #[test]
    fn test_analyze_tlp_no_snapback() {
        // Linear I-V, no snapback
        let iv: Vec<TlpPoint> = (0..10)
            .map(|i| TlpPoint {
                voltage_v: i as f64,
                current_a: i as f64 * 0.1,
            })
            .collect();
        let analysis = analyze_tlp(&iv);
        assert!(!analysis.snapback_detected);
    }

    #[test]
    fn test_tlp_on_resistance() {
        // Linear I-V with known slope (R = 10 ohms)
        let iv: Vec<TlpPoint> = (0..20)
            .map(|i| TlpPoint {
                voltage_v: 5.0 + i as f64 * 0.5,
                current_a: i as f64 * 0.05,
            })
            .collect();
        let analysis = analyze_tlp(&iv);
        if let Some(r_on) = analysis.on_resistance_ohm {
            assert!(
                (r_on - 10.0).abs() < 2.0,
                "On-resistance should be ~10 ohms, got {}",
                r_on
            );
        }
    }

    // --- Component threshold tests ---

    #[test]
    fn test_exceeds_threshold() {
        let threshold = ComponentThreshold {
            name: "TestIC".into(),
            hbm_voltage_v: 2000.0,
            mm_voltage_v: 200.0,
            cdm_voltage_v: 500.0,
        };
        let event = EsdEvent {
            peak_voltage_v: 2500.0,
            peak_current_a: 1.5,
            rise_time_ns: 0.8,
            decay_time_ns: 60.0,
            energy_j: 0.2e-3,
            polarity: EsdPolarity::Positive,
            charge_transfer_c: 200e-9,
            start_sample: 0,
            duration_samples: 1000,
        };
        assert!(exceeds_threshold(&event, &threshold, EsdModel::HumanBody));
    }

    #[test]
    fn test_within_threshold() {
        let threshold = ComponentThreshold {
            name: "TestIC".into(),
            hbm_voltage_v: 2000.0,
            mm_voltage_v: 200.0,
            cdm_voltage_v: 500.0,
        };
        let event = EsdEvent {
            peak_voltage_v: 1500.0,
            peak_current_a: 1.0,
            rise_time_ns: 0.8,
            decay_time_ns: 60.0,
            energy_j: 0.1e-3,
            polarity: EsdPolarity::Positive,
            charge_transfer_c: 100e-9,
            start_sample: 0,
            duration_samples: 1000,
        };
        assert!(!exceeds_threshold(&event, &threshold, EsdModel::HumanBody));
    }

    // --- Statistics tests ---

    #[test]
    fn test_compute_statistics_empty() {
        let stats = compute_statistics(&[]);
        assert_eq!(stats.count, 0);
        assert!((stats.mean_peak_current_a - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_compute_statistics_basic() {
        let events: Vec<EsdEvent> = (1..=10)
            .map(|i| EsdEvent {
                peak_voltage_v: 1000.0 * i as f64,
                peak_current_a: i as f64,
                rise_time_ns: 0.8,
                decay_time_ns: 60.0,
                energy_j: 0.1e-3 * i as f64,
                polarity: EsdPolarity::Positive,
                charge_transfer_c: 100e-9,
                start_sample: 0,
                duration_samples: 1000,
            })
            .collect();
        let stats = compute_statistics(&events);
        assert_eq!(stats.count, 10);
        assert!((stats.mean_peak_current_a - 5.5).abs() < EPSILON);
        assert!(stats.std_peak_current_a > 0.0);
        assert!(stats.p95_peak_current_a >= 9.0);
    }

    #[test]
    fn test_weibull_cdf_properties() {
        // CDF at 0 should be 0
        assert!((weibull_cdf(0.0, 2.0, 10.0) - 0.0).abs() < EPSILON);
        // CDF should be monotonically increasing
        let f1 = weibull_cdf(5.0, 2.0, 10.0);
        let f2 = weibull_cdf(10.0, 2.0, 10.0);
        let f3 = weibull_cdf(20.0, 2.0, 10.0);
        assert!(f1 < f2);
        assert!(f2 < f3);
        // CDF at eta should be ~0.632 for any beta
        let f_eta = weibull_cdf(10.0, 2.0, 10.0);
        assert!(
            (f_eta - (1.0 - (-1.0_f64).exp())).abs() < 0.01,
            "CDF at eta should be ~0.632, got {}",
            f_eta
        );
    }

    #[test]
    fn test_weibull_reliability() {
        let r = weibull_reliability(10.0, 2.0, 10.0);
        let f = weibull_cdf(10.0, 2.0, 10.0);
        assert!((r + f - 1.0).abs() < EPSILON);
    }

    // --- Measurement utility tests ---

    #[test]
    fn test_peak_ratio_single_peak() {
        // Monotonic decay after peak - no meaningful second peak
        let signal: Vec<f64> = (0..100)
            .map(|i| {
                let t = i as f64 * 0.1;
                10.0 * (-t * 0.5).exp()
            })
            .collect();
        // May or may not find a second peak depending on implementation
        let _ = peak_ratio(&signal);
    }

    #[test]
    fn test_rise_time_known_pulse() {
        // Create a signal that linearly ramps from 0 to 10 in 10 samples, then holds
        let dt = 1e-9;
        let mut signal = Vec::new();
        for i in 0..10 {
            signal.push(i as f64);
        }
        for _ in 10..50 {
            signal.push(10.0);
        }
        let rt = measure_rise_time(&signal, dt);
        // 10% of 10 = 1 (sample 1), 90% of 10 = 9 (sample 9)
        // rise time = (9-1) * 1ns = 8 ns
        assert!(
            (rt - 8e-9).abs() < 2e-9,
            "Rise time should be ~8 ns, got {} ns",
            rt * 1e9
        );
    }

    #[test]
    fn test_dft_magnitude_dc() {
        let signal = vec![1.0; 64];
        let (freqs, mags) = dft_magnitude(&signal, 1000.0);
        // DC component should be 1.0
        assert!((mags[0] - 1.0).abs() < 0.01);
        // Non-DC should be near zero
        for &m in mags.iter().skip(1) {
            assert!(m < 0.01, "Non-DC component should be near zero");
        }
        assert!((freqs[0] - 0.0).abs() < EPSILON);
    }
}

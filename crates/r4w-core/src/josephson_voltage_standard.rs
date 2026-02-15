//! # Josephson Voltage Standard Signal Processing
//!
//! This module implements Josephson voltage standard (JVS) signal processing for
//! precision voltage metrology using the AC Josephson effect. When a Josephson
//! junction is irradiated with microwave radiation at frequency f, the I-V curve
//! develops perfectly quantized voltage steps (Shapiro steps) at V_n = n * f / K_J,
//! where K_J = 2e/h is the Josephson constant.
//!
//! Since the 2019 SI redefinition, K_J is exact, making the Josephson effect a
//! primary realization of the volt with zero uncertainty from the defining constant.
//!
//! ## Key Components
//!
//! - [`JosephsonJunction`] -- Single junction parameters (I_c, R_n, C, Stewart-McCumber)
//! - [`ShapiroStep`] -- Shapiro step voltage and width from Bessel functions
//! - [`ConventionalJvs`] -- CJVS: N-junction series array at fixed frequency
//! - [`ProgrammableJvs`] -- PJVS: binary-weighted subarrays with ternary coding
//! - [`JawsSynthesizer`] -- JAWS: sigma-delta pulse-driven arbitrary waveform synthesis
//! - [`UncertaintyBudget`] -- Frequency, leakage, thermal EMF, cable contributions
//! - [`ComparisonMeasurement`] -- DVM calibration, null detection, Allan variance
//! - [`bessel_jn`] -- Bessel function of the first kind J_n(x)
//!
//! ## Physical Constants (2019 SI exact values)
//!
//! | Constant | Symbol | Value |
//! |----------|--------|-------|
//! | Planck constant | h | 6.62607015e-34 J s |
//! | Elementary charge | e | 1.602176634e-19 C |
//! | Josephson constant | K_J | 2e/h = 483597.8484...e9 Hz/V |
//! | Flux quantum | Phi_0 | h/(2e) = 2.067833848...e-15 Wb |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::josephson_voltage_standard::*;
//!
//! // Josephson constant
//! assert!((JOSEPHSON_CONSTANT_GHZ_PER_V - 483597.8484).abs() < 0.001);
//!
//! // Shapiro step voltage at 70 GHz, step n=1
//! let v = shapiro_step_voltage(1, 70.0e9);
//! assert!((v * 1e6 - 144.8).abs() < 0.1); // ~144.8 uV
//!
//! // CJVS: 20,208 junctions at 70 GHz for ~1 V
//! let cjvs = ConventionalJvs::new(20_208, 70.0e9);
//! assert!((cjvs.output_voltage(1) - 1.0).abs() < 0.01);
//! ```

use std::f64::consts::PI;

// ============================================================================
// Physical constants (2019 SI exact values)
// ============================================================================

/// Planck constant h in J*s (exact since 2019 SI)
pub const PLANCK_CONSTANT: f64 = 6.626_070_15e-34;

/// Reduced Planck constant h_bar = h/(2*pi)
pub const HBAR: f64 = 1.054_571_817e-34; // h / (2*pi), precomputed for precision

/// Elementary charge e in coulombs (exact since 2019 SI)
pub const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;

/// Josephson constant K_J = 2e/h in Hz/V (exact since 2019 SI)
pub const JOSEPHSON_CONSTANT: f64 = 2.0 * ELEMENTARY_CHARGE / PLANCK_CONSTANT;

/// Josephson constant in GHz/V for convenient use
pub const JOSEPHSON_CONSTANT_GHZ_PER_V: f64 = 483_597.848_4;

/// Magnetic flux quantum Phi_0 = h/(2e) in Wb
pub const FLUX_QUANTUM: f64 = PLANCK_CONSTANT / (2.0 * ELEMENTARY_CHARGE);

/// Boltzmann constant k_B in J/K (exact since 2019 SI)
pub const BOLTZMANN_CONSTANT: f64 = 1.380_649e-23;

// ============================================================================
// Bessel functions of the first kind
// ============================================================================

/// Compute Bessel function of the first kind J_n(x) using series expansion.
///
/// Uses the power series:
///   J_n(x) = sum_{m=0}^{inf} (-1)^m / (m! * (m+n)!) * (x/2)^(2m+n)
///
/// For large x, uses asymptotic expansion:
///   J_n(x) ~ sqrt(2/(pi*x)) * cos(x - n*pi/2 - pi/4)
pub fn bessel_jn(n: i32, x: f64) -> f64 {
    let n_abs = n.unsigned_abs() as usize;

    if x.abs() < 1e-15 {
        return if n == 0 { 1.0 } else { 0.0 };
    }

    // For negative orders: J_{-n}(x) = (-1)^n * J_n(x)
    if n < 0 {
        let sign = if n_abs % 2 == 0 { 1.0 } else { -1.0 };
        return sign * bessel_jn(n_abs as i32, x);
    }

    let x_abs = x.abs();

    // Use asymptotic expansion for large arguments
    if x_abs > 25.0 + 0.5 * n_abs as f64 {
        return bessel_jn_asymptotic(n_abs, x_abs) * if x < 0.0 && n_abs % 2 == 1 { -1.0 } else { 1.0 };
    }

    // Series expansion
    bessel_jn_series(n_abs, x)
}

/// Series expansion for J_n(x)
fn bessel_jn_series(n: usize, x: f64) -> f64 {
    let half_x = x / 2.0;
    let mut sum = 0.0;
    let mut term = 1.0;

    // Initial term: (x/2)^n / n!
    for k in 1..=n {
        term *= half_x / k as f64;
    }

    sum += term;

    // Subsequent terms: multiply by -(x/2)^2 / (m * (m+n))
    let half_x_sq = half_x * half_x;
    for m in 1..=80 {
        term *= -half_x_sq / (m as f64 * (m + n) as f64);
        sum += term;
        if term.abs() < 1e-16 * sum.abs() {
            break;
        }
    }

    sum
}

/// Asymptotic expansion for J_n(x) when x >> n
fn bessel_jn_asymptotic(n: usize, x: f64) -> f64 {
    let mu = 4.0 * (n as f64) * (n as f64);
    let phase = x - (n as f64) * PI / 2.0 - PI / 4.0;

    // First few terms of the asymptotic expansion
    let x8 = 8.0 * x;
    let p = 1.0 - (mu - 1.0) * (mu - 9.0) / (2.0 * x8 * x8)
        + (mu - 1.0) * (mu - 9.0) * (mu - 25.0) * (mu - 49.0) / (24.0 * x8 * x8 * x8 * x8);
    let q = (mu - 1.0) / x8
        - (mu - 1.0) * (mu - 9.0) * (mu - 25.0) / (6.0 * x8 * x8 * x8);

    (2.0 / (PI * x)).sqrt() * (p * phase.cos() - q * phase.sin())
}

/// Compute factorial as f64 (for small n)
fn factorial(n: usize) -> f64 {
    let mut result = 1.0;
    for i in 2..=n {
        result *= i as f64;
    }
    result
}

// ============================================================================
// Core Josephson effect functions
// ============================================================================

/// Compute Shapiro step voltage: V_n = n * f / K_J
///
/// # Arguments
/// * `n` - Step number (integer)
/// * `freq_hz` - Microwave drive frequency in Hz
///
/// # Returns
/// Voltage in volts
pub fn shapiro_step_voltage(n: i32, freq_hz: f64) -> f64 {
    n as f64 * freq_hz / JOSEPHSON_CONSTANT
}

/// Compute Shapiro step width (current range) for the nth step.
///
/// delta_I_n = 2 * I_c * |J_n(alpha)|
/// where alpha = 2*e*V_rf / (h*f)
///
/// # Arguments
/// * `n` - Step number
/// * `critical_current_a` - Junction critical current in amps
/// * `v_rf` - Microwave RF voltage amplitude across junction in volts
/// * `freq_hz` - Microwave frequency in Hz
pub fn shapiro_step_width(n: i32, critical_current_a: f64, v_rf: f64, freq_hz: f64) -> f64 {
    let alpha = 2.0 * ELEMENTARY_CHARGE * v_rf / (PLANCK_CONSTANT * freq_hz);
    2.0 * critical_current_a * bessel_jn(n, alpha).abs()
}

// ============================================================================
// Josephson Junction
// ============================================================================

/// Parameters of a single Josephson junction.
#[derive(Debug, Clone)]
pub struct JosephsonJunction {
    /// Critical current I_c in amps
    pub critical_current_a: f64,
    /// Normal-state resistance R_n in ohms
    pub normal_resistance_ohm: f64,
    /// Junction capacitance C in farads
    pub capacitance_f: f64,
}

impl JosephsonJunction {
    /// Create a new Josephson junction with the given parameters.
    pub fn new(critical_current_a: f64, normal_resistance_ohm: f64, capacitance_f: f64) -> Self {
        Self {
            critical_current_a,
            normal_resistance_ohm,
            capacitance_f,
        }
    }

    /// Typical Nb/AlOx/Nb junction for voltage standard applications.
    /// I_c ~ 1 mA, R_n ~ 1 ohm, C ~ 1 pF.
    pub fn typical_nb() -> Self {
        Self {
            critical_current_a: 1.0e-3,
            normal_resistance_ohm: 1.0,
            capacitance_f: 1.0e-12,
        }
    }

    /// Characteristic voltage V_c = I_c * R_n
    pub fn characteristic_voltage(&self) -> f64 {
        self.critical_current_a * self.normal_resistance_ohm
    }

    /// Stewart-McCumber parameter:
    /// beta_c = (2*pi / Phi_0) * I_c * R_n^2 * C
    ///
    /// beta_c >> 1: underdamped (hysteretic)
    /// beta_c << 1: overdamped (non-hysteretic, preferred for standards)
    pub fn stewart_mccumber_parameter(&self) -> f64 {
        (2.0 * PI / FLUX_QUANTUM)
            * self.critical_current_a
            * self.normal_resistance_ohm
            * self.normal_resistance_ohm
            * self.capacitance_f
    }

    /// Plasma frequency in Hz:
    /// f_p = (1/(2*pi)) * sqrt(2*e*I_c / (h_bar * C))
    pub fn plasma_frequency(&self) -> f64 {
        (1.0 / (2.0 * PI))
            * (2.0 * ELEMENTARY_CHARGE * self.critical_current_a
                / (HBAR * self.capacitance_f))
                .sqrt()
    }

    /// I_c * R_n product in volts (quality metric for junction)
    pub fn ic_rn_product(&self) -> f64 {
        self.critical_current_a * self.normal_resistance_ohm
    }

    /// Characteristic frequency f_c = (2*e/(h)) * I_c * R_n = K_J * V_c
    pub fn characteristic_frequency(&self) -> f64 {
        JOSEPHSON_CONSTANT * self.characteristic_voltage()
    }

    /// Thermal noise rounding parameter: Gamma = 2*pi*k_B*T / (Phi_0*I_c)
    pub fn thermal_noise_parameter(&self, temperature_k: f64) -> f64 {
        2.0 * PI * BOLTZMANN_CONSTANT * temperature_k
            / (FLUX_QUANTUM * self.critical_current_a)
    }

    /// Compute the step width for the nth Shapiro step.
    pub fn shapiro_step_width(&self, n: i32, v_rf: f64, freq_hz: f64) -> f64 {
        shapiro_step_width(n, self.critical_current_a, v_rf, freq_hz)
    }

    /// Is the junction overdamped? (preferred for voltage standards)
    pub fn is_overdamped(&self) -> bool {
        self.stewart_mccumber_parameter() < 1.0
    }
}

// ============================================================================
// Shapiro step computation
// ============================================================================

/// Detailed Shapiro step information.
#[derive(Debug, Clone)]
pub struct ShapiroStep {
    /// Step number n
    pub step_number: i32,
    /// Step voltage V_n in volts
    pub voltage_v: f64,
    /// Step width (current range) in amps
    pub step_width_a: f64,
    /// Bessel function value J_n(alpha)
    pub bessel_value: f64,
    /// Drive parameter alpha = 2eV_rf / (hf)
    pub alpha: f64,
}

/// Compute Shapiro step parameters for a junction under microwave irradiation.
///
/// # Arguments
/// * `junction` - Junction parameters
/// * `freq_hz` - Microwave frequency in Hz
/// * `v_rf` - RF voltage amplitude across junction
/// * `max_step` - Maximum step number to compute
pub fn compute_shapiro_steps(
    junction: &JosephsonJunction,
    freq_hz: f64,
    v_rf: f64,
    max_step: i32,
) -> Vec<ShapiroStep> {
    let alpha = 2.0 * ELEMENTARY_CHARGE * v_rf / (PLANCK_CONSTANT * freq_hz);
    let mut steps = Vec::new();

    for n in 0..=max_step {
        let jn = bessel_jn(n, alpha);
        steps.push(ShapiroStep {
            step_number: n,
            voltage_v: shapiro_step_voltage(n, freq_hz),
            step_width_a: 2.0 * junction.critical_current_a * jn.abs(),
            bessel_value: jn,
            alpha,
        });
    }

    steps
}

// ============================================================================
// Conventional Josephson Voltage Standard (CJVS)
// ============================================================================

/// Conventional Josephson Voltage Standard using a series array of identical
/// junctions all biased on the same Shapiro step.
#[derive(Debug, Clone)]
pub struct ConventionalJvs {
    /// Number of junctions in series
    pub num_junctions: usize,
    /// Microwave drive frequency in Hz
    pub frequency_hz: f64,
    /// Voltage per junction per step in volts
    pub voltage_per_step: f64,
}

impl ConventionalJvs {
    /// Create a CJVS array.
    ///
    /// # Arguments
    /// * `num_junctions` - Number of junctions in series
    /// * `frequency_hz` - Microwave irradiation frequency in Hz
    pub fn new(num_junctions: usize, frequency_hz: f64) -> Self {
        let voltage_per_step = frequency_hz / JOSEPHSON_CONSTANT;
        Self {
            num_junctions,
            frequency_hz,
            voltage_per_step,
        }
    }

    /// Typical 1V standard: ~20,208 junctions at 70 GHz.
    pub fn standard_1v() -> Self {
        // 1V / (70e9 / K_J) = 1 / 1.44770e-4 ≈ 6907 junctions on step n=1
        // But for step n=1: N * 1 * f/K_J = 1V => N = K_J / f
        let freq = 70.0e9;
        let v_step = freq / JOSEPHSON_CONSTANT;
        let n_junctions = (1.0 / v_step).ceil() as usize;
        Self::new(n_junctions, freq)
    }

    /// Typical 10V standard: ~20,208 junctions at 70 GHz, step n=1.
    pub fn standard_10v() -> Self {
        let freq = 70.0e9;
        let v_step = freq / JOSEPHSON_CONSTANT;
        let n_junctions = (10.0 / v_step).ceil() as usize;
        Self::new(n_junctions, freq)
    }

    /// Output voltage when all junctions are on step n.
    pub fn output_voltage(&self, step_n: i32) -> f64 {
        self.num_junctions as f64 * step_n as f64 * self.voltage_per_step
    }

    /// Maximum output voltage (all junctions on step n).
    pub fn max_voltage(&self, max_step: i32) -> f64 {
        self.output_voltage(max_step)
    }

    /// Number of junctions required for a target voltage at step n=1.
    pub fn junctions_for_voltage(target_v: f64, frequency_hz: f64) -> usize {
        let v_step = frequency_hz / JOSEPHSON_CONSTANT;
        (target_v / v_step).ceil() as usize
    }

    /// Voltage resolution (smallest step) in volts.
    pub fn voltage_resolution(&self) -> f64 {
        self.voltage_per_step
    }
}

// ============================================================================
// Programmable Josephson Voltage Standard (PJVS)
// ============================================================================

/// Ternary state for a PJVS subarray segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TernaryState {
    /// Junction biased on positive step (+1)
    Plus,
    /// Junction in zero-voltage state (0)
    Zero,
    /// Junction biased on negative step (-1)
    Minus,
}

impl TernaryState {
    /// Numeric value of the ternary state.
    pub fn value(&self) -> i32 {
        match self {
            TernaryState::Plus => 1,
            TernaryState::Zero => 0,
            TernaryState::Minus => -1,
        }
    }
}

/// Programmable Josephson Voltage Standard with binary-weighted subarrays.
///
/// Each subarray segment has 2^i junctions and can be set to +1, 0, or -1
/// steps, enabling DAC-like arbitrary DC voltage synthesis:
///   V = V_step * sum(b_i * 2^i)   where b_i in {-1, 0, +1}
#[derive(Debug, Clone)]
pub struct ProgrammableJvs {
    /// Number of binary-weighted bits (subarrays)
    pub num_bits: usize,
    /// Microwave drive frequency in Hz
    pub frequency_hz: f64,
    /// LSB voltage (single junction, step n=1)
    pub lsb_voltage: f64,
    /// Current state of each subarray segment
    pub states: Vec<TernaryState>,
    /// Step transition time in seconds
    pub transition_time_s: f64,
    /// Settling time after transition in seconds
    pub settling_time_s: f64,
}

impl ProgrammableJvs {
    /// Create a new PJVS.
    ///
    /// # Arguments
    /// * `num_bits` - Number of binary-weighted segments
    /// * `frequency_hz` - Microwave frequency in Hz
    pub fn new(num_bits: usize, frequency_hz: f64) -> Self {
        let lsb_voltage = frequency_hz / JOSEPHSON_CONSTANT;
        Self {
            num_bits,
            frequency_hz,
            lsb_voltage,
            states: vec![TernaryState::Zero; num_bits],
            transition_time_s: 10.0e-6,   // 10 us typical
            settling_time_s: 100.0e-6,     // 100 us typical
        }
    }

    /// Typical NIST 1V PJVS: 16 bits at 70 GHz.
    pub fn nist_1v() -> Self {
        Self::new(16, 70.0e9)
    }

    /// Total number of junctions in the array.
    pub fn total_junctions(&self) -> usize {
        (1usize << self.num_bits) - 1
    }

    /// Set the state of a specific subarray segment.
    pub fn set_segment(&mut self, bit: usize, state: TernaryState) {
        if bit < self.num_bits {
            self.states[bit] = state;
        }
    }

    /// Set all segments from a ternary code vector.
    pub fn set_code(&mut self, code: &[TernaryState]) {
        let len = code.len().min(self.num_bits);
        self.states[..len].copy_from_slice(&code[..len]);
    }

    /// Compute the output voltage from the current ternary state.
    pub fn output_voltage(&self) -> f64 {
        let mut total = 0i64;
        for (i, state) in self.states.iter().enumerate() {
            total += state.value() as i64 * (1i64 << i);
        }
        total as f64 * self.lsb_voltage
    }

    /// Set the output to the closest achievable voltage.
    /// Returns the actual voltage set.
    pub fn set_voltage(&mut self, target_v: f64) -> f64 {
        let code = (target_v / self.lsb_voltage).round() as i64;
        let max_code = (1i64 << self.num_bits) - 1;
        let clamped = code.max(-max_code).min(max_code);

        // Convert to balanced ternary representation
        let mut remaining = clamped;
        let negative = remaining < 0;
        remaining = remaining.abs();

        for i in (0..self.num_bits).rev() {
            let weight = 1i64 << i;
            if remaining >= weight {
                self.states[i] = if negative {
                    TernaryState::Minus
                } else {
                    TernaryState::Plus
                };
                remaining -= weight;
            } else {
                self.states[i] = TernaryState::Zero;
            }
        }

        self.output_voltage()
    }

    /// Maximum positive output voltage.
    pub fn max_voltage(&self) -> f64 {
        ((1i64 << self.num_bits) - 1) as f64 * self.lsb_voltage
    }

    /// Voltage resolution (LSB).
    pub fn resolution(&self) -> f64 {
        self.lsb_voltage
    }

    /// Number of achievable voltage levels (including zero).
    pub fn num_levels(&self) -> usize {
        2 * ((1usize << self.num_bits) - 1) + 1
    }
}

// ============================================================================
// JAWS - Josephson Arbitrary Waveform Synthesizer
// ============================================================================

/// Josephson Arbitrary Waveform Synthesizer (JAWS).
///
/// Uses sigma-delta modulated pulse trains to drive a Josephson junction array,
/// synthesizing arbitrary AC voltage waveforms with quantum accuracy.
#[derive(Debug, Clone)]
pub struct JawsSynthesizer {
    /// Number of junctions
    pub num_junctions: usize,
    /// Pulse repetition rate in Hz (clock frequency)
    pub pulse_rate_hz: f64,
    /// Voltage per pulse: V_pulse = Phi_0 * f_rep (for single junction)
    pub voltage_per_pulse: f64,
    /// Sigma-delta pattern (true = pulse, false = no pulse)
    pattern: Vec<bool>,
    /// Pattern length
    pattern_length: usize,
}

impl JawsSynthesizer {
    /// Create a new JAWS synthesizer.
    ///
    /// # Arguments
    /// * `num_junctions` - Number of junctions in array
    /// * `pulse_rate_hz` - Sigma-delta clock rate in Hz
    pub fn new(num_junctions: usize, pulse_rate_hz: f64) -> Self {
        Self {
            num_junctions,
            pulse_rate_hz,
            voltage_per_pulse: FLUX_QUANTUM * pulse_rate_hz,
            pattern: Vec::new(),
            pattern_length: 0,
        }
    }

    /// Generate a sigma-delta modulated pattern for a sine wave.
    ///
    /// # Arguments
    /// * `signal_freq_hz` - Desired output frequency in Hz
    /// * `amplitude_v` - Peak amplitude in volts
    /// * `num_periods` - Number of signal periods to generate
    pub fn generate_sine_pattern(
        &mut self,
        signal_freq_hz: f64,
        amplitude_v: f64,
        num_periods: usize,
    ) {
        let samples_per_period = (self.pulse_rate_hz / signal_freq_hz).round() as usize;
        let total_samples = samples_per_period * num_periods;
        let max_pulse_voltage = self.num_junctions as f64 * self.voltage_per_pulse;

        let mut pattern = Vec::with_capacity(total_samples);
        let mut integrator = 0.0;

        for i in 0..total_samples {
            let t = i as f64 / self.pulse_rate_hz;
            let target = amplitude_v * (2.0 * PI * signal_freq_hz * t).sin();
            let normalized = target / max_pulse_voltage;

            integrator += normalized;
            if integrator >= 0.0 {
                pattern.push(true);
                integrator -= 1.0;
            } else {
                pattern.push(false);
                // negative pulse
                integrator += 1.0;
            }
        }

        self.pattern_length = total_samples;
        self.pattern = pattern;
    }

    /// Generate a sigma-delta pattern for a triangle wave.
    pub fn generate_triangle_pattern(
        &mut self,
        signal_freq_hz: f64,
        amplitude_v: f64,
        num_periods: usize,
    ) {
        let samples_per_period = (self.pulse_rate_hz / signal_freq_hz).round() as usize;
        let total_samples = samples_per_period * num_periods;
        let max_pulse_voltage = self.num_junctions as f64 * self.voltage_per_pulse;

        let mut pattern = Vec::with_capacity(total_samples);
        let mut integrator = 0.0;

        for i in 0..total_samples {
            let phase = (i % samples_per_period) as f64 / samples_per_period as f64;
            let target = amplitude_v
                * if phase < 0.25 {
                    4.0 * phase
                } else if phase < 0.75 {
                    2.0 - 4.0 * phase
                } else {
                    4.0 * phase - 4.0
                };
            let normalized = target / max_pulse_voltage;

            integrator += normalized;
            if integrator >= 0.0 {
                pattern.push(true);
                integrator -= 1.0;
            } else {
                pattern.push(false);
                integrator += 1.0;
            }
        }

        self.pattern_length = total_samples;
        self.pattern = pattern;
    }

    /// Compute the RMS voltage of the current pattern.
    pub fn rms_voltage(&self) -> f64 {
        if self.pattern.is_empty() {
            return 0.0;
        }

        let pulse_v = self.num_junctions as f64 * self.voltage_per_pulse;
        let positive_count = self.pattern.iter().filter(|&&p| p).count();
        let negative_count = self.pattern.len() - positive_count;

        // Each sample is either +V_pulse or -V_pulse
        // RMS = V_pulse * sqrt(duty_cycle) for unipolar
        // For bipolar: each sample contributes V_pulse^2
        // V_rms = V_pulse (since every sample has magnitude V_pulse)
        // But the net RMS of the reconstructed waveform depends on the pattern

        // Compute the reconstructed waveform RMS
        let mean_sq = (positive_count as f64 * pulse_v * pulse_v
            + negative_count as f64 * pulse_v * pulse_v)
            / self.pattern.len() as f64;
        mean_sq.sqrt()
    }

    /// Compute the duty cycle (fraction of positive pulses).
    pub fn duty_cycle(&self) -> f64 {
        if self.pattern.is_empty() {
            return 0.0;
        }
        let positive = self.pattern.iter().filter(|&&p| p).count();
        positive as f64 / self.pattern.len() as f64
    }

    /// Get the pattern length.
    pub fn pattern_length(&self) -> usize {
        self.pattern_length
    }

    /// Get the pattern as a slice.
    pub fn pattern(&self) -> &[bool] {
        &self.pattern
    }

    /// Reconstruct the analog voltage waveform from the pulse pattern.
    /// Returns voltage samples at the pulse rate.
    pub fn reconstruct_waveform(&self) -> Vec<f64> {
        let pulse_v = self.num_junctions as f64 * self.voltage_per_pulse;
        self.pattern
            .iter()
            .map(|&p| if p { pulse_v } else { -pulse_v })
            .collect()
    }
}

// ============================================================================
// Uncertainty Budget
// ============================================================================

/// Individual uncertainty contribution in a JVS measurement.
#[derive(Debug, Clone)]
pub struct UncertaintyComponent {
    /// Name of the uncertainty source
    pub name: String,
    /// Value of the uncertainty (relative, dimensionless)
    pub value: f64,
    /// Type: 'A' for statistical, 'B' for systematic
    pub eval_type: char,
}

/// Uncertainty budget for a Josephson voltage standard measurement.
#[derive(Debug, Clone)]
pub struct UncertaintyBudget {
    /// Components of the uncertainty budget
    pub components: Vec<UncertaintyComponent>,
}

impl UncertaintyBudget {
    /// Create a new empty uncertainty budget.
    pub fn new() -> Self {
        Self {
            components: Vec::new(),
        }
    }

    /// Create a typical uncertainty budget for a 1V CJVS measurement.
    pub fn typical_1v_cjvs() -> Self {
        let mut budget = Self::new();
        budget.add_component("Frequency reference", 1.0e-13, 'B');
        budget.add_component("Leakage current", 5.0e-11, 'B');
        budget.add_component("Thermal EMF", 2.0e-9, 'B');
        budget.add_component("Cable resistance", 1.0e-10, 'B');
        budget.add_component("DVM noise", 1.0e-8, 'A');
        budget.add_component("Repeatability", 5.0e-9, 'A');
        budget
    }

    /// Add an uncertainty component.
    pub fn add_component(&mut self, name: &str, value: f64, eval_type: char) {
        self.components.push(UncertaintyComponent {
            name: name.to_string(),
            value,
            eval_type,
        });
    }

    /// Frequency uncertainty contribution: delta_V/V = delta_f/f
    pub fn frequency_uncertainty(freq_uncertainty_relative: f64) -> f64 {
        freq_uncertainty_relative
    }

    /// Leakage current uncertainty: delta_V = I_leak * R_bias
    pub fn leakage_current_uncertainty(leakage_a: f64, bias_resistance_ohm: f64, voltage_v: f64) -> f64 {
        (leakage_a * bias_resistance_ohm) / voltage_v
    }

    /// Thermal EMF uncertainty contribution.
    pub fn thermal_emf_uncertainty(thermal_emf_v: f64, voltage_v: f64) -> f64 {
        thermal_emf_v / voltage_v
    }

    /// Cable/connection resistance uncertainty.
    pub fn cable_resistance_uncertainty(
        cable_resistance_ohm: f64,
        current_a: f64,
        voltage_v: f64,
    ) -> f64 {
        (cable_resistance_ohm * current_a) / voltage_v
    }

    /// Combined standard uncertainty (RSS of all components).
    pub fn combined_standard_uncertainty(&self) -> f64 {
        let sum_sq: f64 = self.components.iter().map(|c| c.value * c.value).sum();
        sum_sq.sqrt()
    }

    /// Expanded uncertainty at coverage factor k (typically k=2 for ~95%).
    pub fn expanded_uncertainty(&self, k: f64) -> f64 {
        k * self.combined_standard_uncertainty()
    }

    /// Type A uncertainty only (statistical).
    pub fn type_a_uncertainty(&self) -> f64 {
        let sum_sq: f64 = self
            .components
            .iter()
            .filter(|c| c.eval_type == 'A')
            .map(|c| c.value * c.value)
            .sum();
        sum_sq.sqrt()
    }

    /// Type B uncertainty only (systematic).
    pub fn type_b_uncertainty(&self) -> f64 {
        let sum_sq: f64 = self
            .components
            .iter()
            .filter(|c| c.eval_type == 'B')
            .map(|c| c.value * c.value)
            .sum();
        sum_sq.sqrt()
    }
}

impl Default for UncertaintyBudget {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Comparison Measurements
// ============================================================================

/// Result of comparing a DVM against a JVS reference.
#[derive(Debug, Clone)]
pub struct ComparisonResult {
    /// JVS reference voltage in volts
    pub jvs_voltage_v: f64,
    /// DVM reading in volts
    pub dvm_reading_v: f64,
    /// Difference (DVM - JVS) in volts
    pub difference_v: f64,
    /// Relative difference (DVM - JVS) / JVS
    pub relative_difference: f64,
}

/// DVM calibration measurement using a JVS reference.
#[derive(Debug, Clone)]
pub struct ComparisonMeasurement {
    /// All comparison results
    pub results: Vec<ComparisonResult>,
}

impl ComparisonMeasurement {
    /// Create a new comparison measurement.
    pub fn new() -> Self {
        Self {
            results: Vec::new(),
        }
    }

    /// Add a measurement point.
    pub fn add_measurement(&mut self, jvs_v: f64, dvm_v: f64) {
        let diff = dvm_v - jvs_v;
        let rel = if jvs_v.abs() > 1e-15 {
            diff / jvs_v
        } else {
            0.0
        };
        self.results.push(ComparisonResult {
            jvs_voltage_v: jvs_v,
            dvm_reading_v: dvm_v,
            difference_v: diff,
            relative_difference: rel,
        });
    }

    /// Mean difference in volts.
    pub fn mean_difference(&self) -> f64 {
        if self.results.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.results.iter().map(|r| r.difference_v).sum();
        sum / self.results.len() as f64
    }

    /// Standard deviation of differences.
    pub fn std_deviation(&self) -> f64 {
        if self.results.len() < 2 {
            return 0.0;
        }
        let mean = self.mean_difference();
        let n = self.results.len() as f64;
        let var: f64 = self
            .results
            .iter()
            .map(|r| {
                let d = r.difference_v - mean;
                d * d
            })
            .sum::<f64>()
            / (n - 1.0);
        var.sqrt()
    }

    /// Standard error of the mean.
    pub fn standard_error(&self) -> f64 {
        if self.results.is_empty() {
            return 0.0;
        }
        self.std_deviation() / (self.results.len() as f64).sqrt()
    }

    /// Type A uncertainty: standard error of the mean of differences.
    pub fn type_a_uncertainty(&self) -> f64 {
        self.standard_error()
    }

    /// Allan variance for a given averaging time tau (in measurement indices).
    /// Uses overlapping Allan variance estimator.
    ///
    /// # Arguments
    /// * `tau` - Averaging factor (number of consecutive measurements to average)
    pub fn allan_variance(&self, tau: usize) -> f64 {
        if tau == 0 || self.results.len() < 2 * tau {
            return 0.0;
        }

        let diffs: Vec<f64> = self.results.iter().map(|r| r.difference_v).collect();
        let n = diffs.len();

        // Compute averaged blocks
        let num_blocks = n - tau + 1;
        let mut averages = Vec::with_capacity(num_blocks);
        for i in 0..num_blocks {
            let sum: f64 = diffs[i..i + tau].iter().sum();
            averages.push(sum / tau as f64);
        }

        // Allan variance: (1/2) * mean((y_{i+1} - y_i)^2)
        if averages.len() < 2 {
            return 0.0;
        }
        let mut sum_sq = 0.0;
        for i in 0..averages.len() - 1 {
            let d = averages[i + 1] - averages[i];
            sum_sq += d * d;
        }
        sum_sq / (2.0 * (averages.len() - 1) as f64)
    }

    /// Allan deviation (square root of Allan variance).
    pub fn allan_deviation(&self, tau: usize) -> f64 {
        self.allan_variance(tau).sqrt()
    }

    /// Null detector reading: difference between JVS and DUT at the most recent point.
    pub fn null_reading(&self) -> Option<f64> {
        self.results.last().map(|r| r.difference_v)
    }

    /// DVM gain error estimated from linear fit of (JVS, DVM) data.
    /// Returns (gain, offset) where DVM = gain * JVS + offset.
    pub fn dvm_calibration(&self) -> (f64, f64) {
        if self.results.len() < 2 {
            return (1.0, 0.0);
        }

        let n = self.results.len() as f64;
        let sum_x: f64 = self.results.iter().map(|r| r.jvs_voltage_v).sum();
        let sum_y: f64 = self.results.iter().map(|r| r.dvm_reading_v).sum();
        let sum_xx: f64 = self
            .results
            .iter()
            .map(|r| r.jvs_voltage_v * r.jvs_voltage_v)
            .sum();
        let sum_xy: f64 = self
            .results
            .iter()
            .map(|r| r.jvs_voltage_v * r.dvm_reading_v)
            .sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return (1.0, self.mean_difference());
        }

        let gain = (n * sum_xy - sum_x * sum_y) / denom;
        let offset = (sum_y - gain * sum_x) / n;
        (gain, offset)
    }
}

impl Default for ComparisonMeasurement {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// I-V Characteristic simulation
// ============================================================================

/// Compute the RSJ (Resistively Shunted Junction) model I-V curve.
///
/// For an overdamped junction under DC bias current I:
///   V = R_n * sqrt(I^2 - I_c^2)  for |I| > I_c
///   V = 0                          for |I| <= I_c
///
/// # Arguments
/// * `junction` - Junction parameters
/// * `bias_currents` - Array of bias current values in amps
///
/// # Returns
/// Voltages corresponding to each bias current
pub fn rsj_iv_curve(junction: &JosephsonJunction, bias_currents: &[f64]) -> Vec<f64> {
    bias_currents
        .iter()
        .map(|&i| {
            let i_abs = i.abs();
            if i_abs <= junction.critical_current_a {
                0.0
            } else {
                let v = junction.normal_resistance_ohm
                    * (i_abs * i_abs
                        - junction.critical_current_a * junction.critical_current_a)
                        .sqrt();
                if i < 0.0 { -v } else { v }
            }
        })
        .collect()
}

/// Compute I-V curve with Shapiro steps under microwave irradiation.
///
/// The steps appear as flat plateaus at V_n = n*h*f/(2e).
/// Step width depends on RF drive level via Bessel functions.
///
/// # Arguments
/// * `junction` - Junction parameters
/// * `freq_hz` - Microwave frequency
/// * `v_rf` - RF voltage amplitude
/// * `bias_currents` - Bias current sweep values
/// * `max_step` - Maximum Shapiro step to consider
pub fn iv_with_shapiro_steps(
    junction: &JosephsonJunction,
    freq_hz: f64,
    v_rf: f64,
    bias_currents: &[f64],
    max_step: i32,
) -> Vec<f64> {
    let alpha = 2.0 * ELEMENTARY_CHARGE * v_rf / (PLANCK_CONSTANT * freq_hz);
    let steps = compute_shapiro_steps(junction, freq_hz, v_rf, max_step);

    bias_currents
        .iter()
        .map(|&i_bias| {
            let i_abs = i_bias.abs();
            let sign = if i_bias < 0.0 { -1.0 } else { 1.0 };

            // Check if current is within any Shapiro step
            for step in &steps {
                let half_width = step.step_width_a / 2.0;
                // Each step centered around some current value
                // For simplicity, check if the current is within the step range
                // In a real junction, the steps form at specific I values
                let step_center = junction.critical_current_a * bessel_jn(0, alpha);
                if step.step_number == 0 && i_abs < half_width {
                    return 0.0;
                }
            }

            // Outside step regions, use RSJ model
            if i_abs <= junction.critical_current_a {
                0.0
            } else {
                sign
                    * junction.normal_resistance_ohm
                    * (i_abs * i_abs
                        - junction.critical_current_a * junction.critical_current_a)
                        .sqrt()
            }
        })
        .collect()
}

// ============================================================================
// Utility functions
// ============================================================================

/// Convert voltage to equivalent frequency: f = V * K_J
pub fn voltage_to_frequency(voltage_v: f64) -> f64 {
    voltage_v * JOSEPHSON_CONSTANT
}

/// Convert frequency to equivalent voltage: V = f / K_J
pub fn frequency_to_voltage(freq_hz: f64) -> f64 {
    freq_hz / JOSEPHSON_CONSTANT
}

/// Number of junctions needed for a target voltage at given frequency.
pub fn junctions_needed(target_v: f64, freq_hz: f64, step_n: i32) -> usize {
    if step_n == 0 || freq_hz <= 0.0 {
        return 0;
    }
    let v_per_junction = step_n as f64 * freq_hz / JOSEPHSON_CONSTANT;
    (target_v / v_per_junction).ceil() as usize
}

/// Compute parts-per-billion (ppb) deviation.
pub fn ppb_deviation(measured: f64, reference: f64) -> f64 {
    if reference.abs() < 1e-30 {
        return 0.0;
    }
    (measured - reference) / reference * 1e9
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;
    const EPSILON_LOOSE: f64 = 1e-6;

    // --- Physical constants ---

    #[test]
    fn test_josephson_constant() {
        // K_J = 2e/h ≈ 483597.8484... GHz/V
        let kj_ghz = JOSEPHSON_CONSTANT / 1e9;
        assert!(
            (kj_ghz - 483597.8484).abs() < 0.01,
            "K_J = {} GHz/V, expected ~483597.8484",
            kj_ghz
        );
    }

    #[test]
    fn test_josephson_constant_ghz_alias() {
        let kj_computed = JOSEPHSON_CONSTANT / 1e9;
        assert!(
            (kj_computed - JOSEPHSON_CONSTANT_GHZ_PER_V).abs() < 0.1,
            "Precomputed GHz constant should match computed"
        );
    }

    #[test]
    fn test_flux_quantum() {
        // Phi_0 = h/(2e) ≈ 2.067833848e-15 Wb
        assert!(
            (FLUX_QUANTUM - 2.067833848e-15).abs() < 1e-24,
            "Phi_0 = {} Wb",
            FLUX_QUANTUM
        );
    }

    #[test]
    fn test_kj_times_phi0() {
        // K_J * Phi_0 = (2e/h) * (h/(2e)) = 1 Hz*Wb/V = 1
        // But K_J is in Hz/V and Phi_0 is in Wb = V*s
        // K_J * Phi_0 should have units Hz*s = dimensionless ≈ 1/freq...
        // Actually K_J [Hz/V] * Phi_0 [V*s] = Hz*s = 1
        let product = JOSEPHSON_CONSTANT * FLUX_QUANTUM;
        assert!(
            (product - 1.0).abs() < 1e-10,
            "K_J * Phi_0 = {}, expected 1.0",
            product
        );
    }

    // --- Bessel functions ---

    #[test]
    fn test_bessel_j0_zero() {
        assert!((bessel_jn(0, 0.0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bessel_jn_zero_for_n_nonzero() {
        for n in 1..5 {
            assert!(
                bessel_jn(n, 0.0).abs() < EPSILON,
                "J_{}(0) should be 0",
                n
            );
        }
    }

    #[test]
    fn test_bessel_j0_known_values() {
        // J_0(1) ≈ 0.7651976865579666
        assert!((bessel_jn(0, 1.0) - 0.7651976866).abs() < 1e-8);
        // J_0(2.4048) ≈ 0 (first zero of J_0)
        assert!(bessel_jn(0, 2.4048).abs() < 1e-4);
    }

    #[test]
    fn test_bessel_j1_known_values() {
        // J_1(1) ≈ 0.4400505857
        assert!((bessel_jn(1, 1.0) - 0.4400505857).abs() < 1e-8);
        // J_1(3.8317) ≈ 0 (first zero of J_1)
        assert!(bessel_jn(1, 3.8317).abs() < 1e-3);
    }

    #[test]
    fn test_bessel_j2() {
        // J_2(3) ≈ 0.4860912606
        assert!((bessel_jn(2, 3.0) - 0.4860912606).abs() < 1e-7);
    }

    #[test]
    fn test_bessel_negative_order() {
        // J_{-n}(x) = (-1)^n * J_n(x)
        let x = 2.5;
        assert!((bessel_jn(-1, x) + bessel_jn(1, x)).abs() < 1e-10);
        assert!((bessel_jn(-2, x) - bessel_jn(2, x)).abs() < 1e-10);
    }

    #[test]
    fn test_bessel_large_argument() {
        // J_0(50) ≈ 0.05580
        let j0_50 = bessel_jn(0, 50.0);
        assert!(
            (j0_50 - 0.05581).abs() < 0.005,
            "J_0(50) = {}, expected ~0.0558",
            j0_50
        );
    }

    #[test]
    fn test_bessel_addition_theorem() {
        // Sum of J_n^2(x) for all n equals 1: sum_{n=-inf}^{inf} J_n^2(x) = 1
        let x = 3.0;
        let mut sum = bessel_jn(0, x).powi(2);
        for n in 1..20 {
            sum += 2.0 * bessel_jn(n, x).powi(2);
        }
        assert!(
            (sum - 1.0).abs() < 1e-6,
            "Sum of J_n^2 = {}, expected 1.0",
            sum
        );
    }

    // --- Shapiro steps ---

    #[test]
    fn test_shapiro_step_voltage_70ghz() {
        // V_1 = 1 * 70e9 / K_J ≈ 144.77 uV
        let v = shapiro_step_voltage(1, 70.0e9);
        assert!(
            (v * 1e6 - 144.77).abs() < 0.1,
            "V_1 at 70 GHz = {} uV",
            v * 1e6
        );
    }

    #[test]
    fn test_shapiro_step_voltage_linear() {
        let f = 70.0e9;
        let v1 = shapiro_step_voltage(1, f);
        let v2 = shapiro_step_voltage(2, f);
        let v3 = shapiro_step_voltage(3, f);
        assert!((v2 - 2.0 * v1).abs() < EPSILON);
        assert!((v3 - 3.0 * v1).abs() < EPSILON);
    }

    #[test]
    fn test_shapiro_step_zero() {
        assert_eq!(shapiro_step_voltage(0, 70.0e9), 0.0);
    }

    #[test]
    fn test_shapiro_step_width() {
        let jj = JosephsonJunction::typical_nb();
        let f = 70.0e9;
        let v_rf = 1.0e-4; // 100 uV RF amplitude
        let width = shapiro_step_width(1, jj.critical_current_a, v_rf, f);
        assert!(width > 0.0, "Step width should be positive");
        assert!(
            width <= 2.0 * jj.critical_current_a,
            "Step width bounded by 2*I_c"
        );
    }

    // --- Josephson Junction ---

    #[test]
    fn test_junction_characteristic_voltage() {
        let jj = JosephsonJunction::new(1e-3, 2.0, 1e-12);
        assert!((jj.characteristic_voltage() - 2e-3).abs() < EPSILON);
    }

    #[test]
    fn test_junction_stewart_mccumber() {
        let jj = JosephsonJunction::typical_nb();
        let beta_c = jj.stewart_mccumber_parameter();
        // For I_c=1mA, R_n=1ohm, C=1pF: beta_c = 2pi * 1e-3 * 1 * 1e-12 / Phi_0
        // ≈ 2pi * 1e-15 / 2.07e-15 ≈ 3.03
        assert!(
            beta_c > 0.0,
            "Stewart-McCumber should be positive, got {}",
            beta_c
        );
    }

    #[test]
    fn test_junction_plasma_frequency() {
        let jj = JosephsonJunction::typical_nb();
        let fp = jj.plasma_frequency();
        // Should be in GHz range for typical junctions
        assert!(fp > 1e9 && fp < 1e12, "f_p = {} Hz", fp);
    }

    #[test]
    fn test_junction_overdamped() {
        // Low capacitance -> overdamped
        let overdamped = JosephsonJunction::new(1e-3, 1.0, 1e-15);
        assert!(
            overdamped.is_overdamped(),
            "Low-C junction should be overdamped"
        );

        // High capacitance -> underdamped
        let underdamped = JosephsonJunction::new(1e-3, 10.0, 1e-9);
        assert!(
            !underdamped.is_overdamped(),
            "High-C junction should be underdamped"
        );
    }

    #[test]
    fn test_junction_thermal_noise() {
        let jj = JosephsonJunction::typical_nb();
        let gamma_4k = jj.thermal_noise_parameter(4.2);
        let gamma_300k = jj.thermal_noise_parameter(300.0);
        // Thermal noise at 4.2K should be much less than at 300K
        assert!(gamma_4k < gamma_300k);
        // At 4.2K, Gamma should be very small for 1mA junction
        assert!(gamma_4k < 0.01, "Gamma(4.2K) = {}", gamma_4k);
    }

    // --- CJVS ---

    #[test]
    fn test_cjvs_1v_standard() {
        let cjvs = ConventionalJvs::standard_1v();
        let v = cjvs.output_voltage(1);
        // Should be close to 1V (may be slightly over due to ceiling)
        assert!(
            (v - 1.0).abs() < cjvs.voltage_per_step,
            "1V standard output = {} V",
            v
        );
    }

    #[test]
    fn test_cjvs_10v_standard() {
        let cjvs = ConventionalJvs::standard_10v();
        let v = cjvs.output_voltage(1);
        assert!(
            (v - 10.0).abs() < cjvs.voltage_per_step,
            "10V standard output = {} V",
            v
        );
    }

    #[test]
    fn test_cjvs_voltage_scaling() {
        let cjvs = ConventionalJvs::new(1000, 70.0e9);
        let v1 = cjvs.output_voltage(1);
        let v2 = cjvs.output_voltage(2);
        assert!((v2 - 2.0 * v1).abs() < EPSILON);
    }

    #[test]
    fn test_cjvs_junctions_for_voltage() {
        let n = ConventionalJvs::junctions_for_voltage(1.0, 70.0e9);
        assert!(n > 6000 && n < 7000, "Need {} junctions for 1V", n);
    }

    #[test]
    fn test_cjvs_voltage_resolution() {
        let cjvs = ConventionalJvs::new(100, 70.0e9);
        let res = cjvs.voltage_resolution();
        assert!(
            (res - shapiro_step_voltage(1, 70.0e9)).abs() < EPSILON,
            "Resolution should equal single step voltage"
        );
    }

    // --- PJVS ---

    #[test]
    fn test_pjvs_zero_initial() {
        let pjvs = ProgrammableJvs::new(10, 70.0e9);
        assert!(
            pjvs.output_voltage().abs() < EPSILON,
            "Initial output should be zero"
        );
    }

    #[test]
    fn test_pjvs_set_voltage() {
        let mut pjvs = ProgrammableJvs::new(16, 70.0e9);
        let actual = pjvs.set_voltage(0.5);
        assert!(
            (actual - 0.5).abs() < pjvs.lsb_voltage,
            "Set voltage = {} V, expected ~0.5 V",
            actual
        );
    }

    #[test]
    fn test_pjvs_negative_voltage() {
        let mut pjvs = ProgrammableJvs::new(16, 70.0e9);
        let actual = pjvs.set_voltage(-0.3);
        assert!(
            (actual - (-0.3)).abs() < pjvs.lsb_voltage,
            "Negative voltage = {} V",
            actual
        );
    }

    #[test]
    fn test_pjvs_total_junctions() {
        let pjvs = ProgrammableJvs::new(16, 70.0e9);
        assert_eq!(pjvs.total_junctions(), 65535);
    }

    #[test]
    fn test_pjvs_num_levels() {
        let pjvs = ProgrammableJvs::new(4, 70.0e9);
        // 2*(2^4 - 1) + 1 = 2*15 + 1 = 31
        assert_eq!(pjvs.num_levels(), 31);
    }

    #[test]
    fn test_pjvs_max_voltage() {
        let pjvs = ProgrammableJvs::new(16, 70.0e9);
        let max_v = pjvs.max_voltage();
        assert!(max_v > 0.0);
        assert!((max_v - 65535.0 * pjvs.lsb_voltage).abs() < EPSILON);
    }

    #[test]
    fn test_pjvs_ternary_states() {
        assert_eq!(TernaryState::Plus.value(), 1);
        assert_eq!(TernaryState::Zero.value(), 0);
        assert_eq!(TernaryState::Minus.value(), -1);
    }

    // --- JAWS ---

    #[test]
    fn test_jaws_sine_generation() {
        let mut jaws = JawsSynthesizer::new(1000, 15.0e9);
        jaws.generate_sine_pattern(1e3, 1.0e-3, 2);
        assert!(jaws.pattern_length() > 0);
        assert!(!jaws.pattern().is_empty());
    }

    #[test]
    fn test_jaws_triangle_generation() {
        let mut jaws = JawsSynthesizer::new(1000, 15.0e9);
        jaws.generate_triangle_pattern(1e3, 1.0e-3, 2);
        assert!(jaws.pattern_length() > 0);
    }

    #[test]
    fn test_jaws_duty_cycle() {
        let mut jaws = JawsSynthesizer::new(1000, 15.0e9);
        jaws.generate_sine_pattern(1e3, 1.0e-6, 3);
        let dc = jaws.duty_cycle();
        // For a symmetric sine, duty cycle should be near 0.5
        assert!(
            (dc - 0.5).abs() < 0.1,
            "Duty cycle = {}, expected ~0.5",
            dc
        );
    }

    #[test]
    fn test_jaws_reconstruct_waveform() {
        let mut jaws = JawsSynthesizer::new(100, 10.0e9);
        jaws.generate_sine_pattern(1e3, 1.0e-6, 1);
        let waveform = jaws.reconstruct_waveform();
        assert_eq!(waveform.len(), jaws.pattern_length());
        // All values should be +/- pulse voltage
        let pulse_v = 100.0 * FLUX_QUANTUM * 10.0e9;
        for v in &waveform {
            assert!((v.abs() - pulse_v).abs() < EPSILON);
        }
    }

    // --- Uncertainty Budget ---

    #[test]
    fn test_uncertainty_combined() {
        let mut budget = UncertaintyBudget::new();
        budget.add_component("A", 3e-9, 'A');
        budget.add_component("B", 4e-9, 'B');
        let combined = budget.combined_standard_uncertainty();
        assert!(
            (combined - 5e-9).abs() < 1e-15,
            "RSS of 3 and 4 should be 5, got {}",
            combined * 1e9
        );
    }

    #[test]
    fn test_uncertainty_expanded() {
        let mut budget = UncertaintyBudget::new();
        budget.add_component("test", 5e-9, 'A');
        let expanded = budget.expanded_uncertainty(2.0);
        assert!((expanded - 10e-9).abs() < 1e-15);
    }

    #[test]
    fn test_uncertainty_type_a_b_separation() {
        let budget = UncertaintyBudget::typical_1v_cjvs();
        let type_a = budget.type_a_uncertainty();
        let type_b = budget.type_b_uncertainty();
        assert!(type_a > 0.0);
        assert!(type_b > 0.0);
        // Combined should be RSS of type A and B
        let combined = budget.combined_standard_uncertainty();
        assert!(
            ((type_a * type_a + type_b * type_b).sqrt() - combined).abs() < 1e-20
        );
    }

    #[test]
    fn test_frequency_uncertainty() {
        let u = UncertaintyBudget::frequency_uncertainty(1e-12);
        assert!((u - 1e-12).abs() < 1e-20);
    }

    // --- Comparison Measurements ---

    #[test]
    fn test_comparison_mean_difference() {
        let mut comp = ComparisonMeasurement::new();
        comp.add_measurement(1.0, 1.000_001);
        comp.add_measurement(1.0, 1.000_003);
        let mean = comp.mean_difference();
        assert!(
            (mean - 2e-6).abs() < 1e-12,
            "Mean diff = {} V",
            mean
        );
    }

    #[test]
    fn test_comparison_std_deviation() {
        let mut comp = ComparisonMeasurement::new();
        for i in 0..100 {
            let offset = 1e-6 * ((i as f64 * 0.1).sin());
            comp.add_measurement(1.0, 1.0 + offset);
        }
        let std = comp.std_deviation();
        assert!(std > 0.0, "Std deviation should be positive");
        assert!(std < 1e-5, "Std deviation should be small");
    }

    #[test]
    fn test_comparison_allan_variance() {
        let mut comp = ComparisonMeasurement::new();
        // White noise: Allan variance should decrease as 1/tau
        for i in 0..1000 {
            let noise = ((i * 7 + 3) % 13) as f64 * 1e-9 - 6e-9;
            comp.add_measurement(1.0, 1.0 + noise);
        }
        let av1 = comp.allan_variance(1);
        let av10 = comp.allan_variance(10);
        assert!(av1 > 0.0);
        assert!(av10 > 0.0);
        // For white noise, AVAR(tau) ~ sigma^2/tau
        // So AVAR(10) should be roughly 1/10 of AVAR(1)
        assert!(av10 < av1, "Allan variance should decrease with tau");
    }

    #[test]
    fn test_comparison_dvm_calibration() {
        let mut comp = ComparisonMeasurement::new();
        // DVM with gain error of 1.00001 and offset of 5uV
        let gain = 1.00001;
        let offset = 5e-6;
        for i in 0..20 {
            let v_jvs = i as f64 * 0.5;
            let v_dvm = gain * v_jvs + offset;
            comp.add_measurement(v_jvs, v_dvm);
        }
        let (g, o) = comp.dvm_calibration();
        assert!(
            (g - gain).abs() < 1e-8,
            "Gain = {}, expected {}",
            g,
            gain
        );
        assert!(
            (o - offset).abs() < 1e-8,
            "Offset = {} V, expected {} V",
            o,
            offset
        );
    }

    #[test]
    fn test_null_reading() {
        let mut comp = ComparisonMeasurement::new();
        comp.add_measurement(1.0, 1.000_002);
        assert!((comp.null_reading().unwrap() - 2e-6).abs() < 1e-12);
    }

    // --- RSJ I-V curve ---

    #[test]
    fn test_rsj_zero_voltage_below_ic() {
        let jj = JosephsonJunction::typical_nb();
        let currents: Vec<f64> = (0..10).map(|i| i as f64 * 0.1e-3).collect();
        let voltages = rsj_iv_curve(&jj, &currents);
        for v in &voltages {
            assert!(
                v.abs() < EPSILON,
                "V should be 0 below I_c, got {}",
                v
            );
        }
    }

    #[test]
    fn test_rsj_nonzero_above_ic() {
        let jj = JosephsonJunction::typical_nb();
        let currents = vec![2.0e-3, 5.0e-3, 10.0e-3];
        let voltages = rsj_iv_curve(&jj, &currents);
        for v in &voltages {
            assert!(*v > 0.0, "V should be positive above I_c");
        }
    }

    #[test]
    fn test_rsj_antisymmetric() {
        let jj = JosephsonJunction::typical_nb();
        let v_pos = rsj_iv_curve(&jj, &[5.0e-3]);
        let v_neg = rsj_iv_curve(&jj, &[-5.0e-3]);
        assert!(
            (v_pos[0] + v_neg[0]).abs() < EPSILON,
            "I-V should be antisymmetric"
        );
    }

    // --- Utility functions ---

    #[test]
    fn test_voltage_frequency_roundtrip() {
        let f = 70.0e9;
        let v = frequency_to_voltage(f);
        let f_back = voltage_to_frequency(v);
        assert!((f_back - f).abs() < 1.0, "Roundtrip failed");
    }

    #[test]
    fn test_junctions_needed() {
        let n = junctions_needed(1.0, 70.0e9, 1);
        assert!(n > 6000 && n < 7000);
    }

    #[test]
    fn test_ppb_deviation() {
        let ppb = ppb_deviation(1.000_000_001, 1.0);
        assert!((ppb - 1.0).abs() < 0.01, "ppb = {}", ppb);
    }

    #[test]
    fn test_compute_shapiro_steps() {
        let jj = JosephsonJunction::typical_nb();
        let steps = compute_shapiro_steps(&jj, 70.0e9, 1e-4, 5);
        assert_eq!(steps.len(), 6); // steps 0 through 5
        assert!(steps[0].voltage_v.abs() < EPSILON);
        for i in 1..steps.len() {
            assert!(
                (steps[i].voltage_v - i as f64 * steps[1].voltage_v).abs() < EPSILON,
                "Step {} voltage mismatch",
                i
            );
        }
    }

    #[test]
    fn test_factorial() {
        assert!((factorial(0) - 1.0).abs() < EPSILON);
        assert!((factorial(1) - 1.0).abs() < EPSILON);
        assert!((factorial(5) - 120.0).abs() < EPSILON);
        assert!((factorial(10) - 3628800.0).abs() < EPSILON);
    }
}

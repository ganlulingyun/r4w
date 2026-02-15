//! # Superconducting Magnetometer (SQUID) Signal Processing
//!
//! This module implements SQUID (Superconducting QUantum Interference Device) magnetometer
//! signal processing for ultra-sensitive magnetic field measurement. SQUIDs exploit the
//! macroscopic quantum phenomena of flux quantization and Josephson tunneling to achieve
//! magnetic field sensitivities approaching the quantum limit.
//!
//! ## Physical Background
//!
//! A DC SQUID consists of two Josephson junctions connected in parallel within a
//! superconducting loop. The critical current of the loop is modulated by the applied
//! magnetic flux threading the loop:
//!
//! ```text
//! I_c(Phi) = 2 * I_c0 * |cos(pi * Phi / Phi_0)|
//! ```
//!
//! where `Phi_0 = h/(2e) ~ 2.0678e-15 Wb` is the magnetic flux quantum, `I_c0` is
//! the single-junction critical current, and `Phi` is the total flux through the loop.
//!
//! When biased above the critical current, the SQUID develops a voltage that is
//! periodic in the applied flux with period `Phi_0`:
//!
//! ```text
//! V(Phi) = R_n * sqrt(I_bias^2 - (2*I_c*cos(pi*Phi/Phi_0))^2)
//! ```
//!
//! ## Flux-Locked Loop (FLL)
//!
//! ```text
//! Phi_signal --> [SQUID] --> [Preamp] --> [Integrator] --> V_out
//!                  ^                          |
//!                  |      [Feedback coil] <---+
//!                  +-- Phi_feedback -----------+
//! ```
//!
//! The FLL maintains the SQUID at its optimal operating point by applying feedback
//! flux equal and opposite to the signal flux, extending the linear range far beyond
//! one flux quantum.
//!
//! ## Applications
//!
//! - **Magnetoencephalography (MEG)**: Brain magnetic fields ~100 fT
//! - **Geomagnetic surveys**: Anomaly detection at nT scale
//! - **Non-Destructive Evaluation (NDE)**: Defect detection via flux leakage
//! - **Fundamental physics**: Magnetic monopole searches, gravity wave antennas
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::superconducting_magnetometer::*;
//!
//! let config = SquidConfig {
//!     critical_current_a: 10.0e-6,
//!     loop_inductance_h: 100.0e-12,
//!     shunt_resistance_ohm: 5.0,
//!     junction_capacitance_f: 1.0e-12,
//! };
//!
//! let squid = DcSquid::new(config);
//! let v = squid.voltage_flux_characteristic(15.0e-6, 0.5 * PHI_0);
//! assert!(v > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Planck constant [J*s]
pub const PLANCK_H: f64 = 6.62607015e-34;

/// Elementary charge [C]
pub const ELEMENTARY_CHARGE: f64 = 1.602176634e-19;

/// Boltzmann constant [J/K]
pub const BOLTZMANN_K: f64 = 1.380649e-23;

/// Magnetic flux quantum Phi_0 = h/(2e) [Wb]
pub const PHI_0: f64 = 2.067833848e-15; // PLANCK_H / (2.0 * ELEMENTARY_CHARGE)

// ---------------------------------------------------------------------------
// SQUID Configuration
// ---------------------------------------------------------------------------

/// Configuration for a DC SQUID sensor.
#[derive(Debug, Clone)]
pub struct SquidConfig {
    /// Critical current of a single Josephson junction [A] (typ. 1-50 uA).
    pub critical_current_a: f64,
    /// SQUID loop inductance [H] (typ. 10-500 pH).
    pub loop_inductance_h: f64,
    /// Shunt resistance per junction [Ohm] (typ. 1-20 Ohm).
    pub shunt_resistance_ohm: f64,
    /// Junction capacitance [F] (typ. 0.1-5 pF).
    pub junction_capacitance_f: f64,
}

impl SquidConfig {
    /// Stewart-McCumber parameter: beta_c = 2*pi*I_c*R_n^2*C / Phi_0.
    /// Non-hysteretic operation requires beta_c < 1.
    pub fn stewart_mccumber_parameter(&self) -> f64 {
        2.0 * PI * self.critical_current_a
            * self.shunt_resistance_ohm.powi(2)
            * self.junction_capacitance_f
            / PHI_0
    }

    /// Screening parameter: beta_L = 2*L*I_c / Phi_0.
    /// Optimal modulation depth at beta_L ~ 1.
    pub fn screening_parameter(&self) -> f64 {
        2.0 * self.loop_inductance_h * self.critical_current_a / PHI_0
    }

    /// Characteristic voltage V_c = I_c * R_n [V].
    pub fn characteristic_voltage(&self) -> f64 {
        self.critical_current_a * self.shunt_resistance_ohm
    }

    /// Josephson frequency for one junction at characteristic voltage [Hz].
    /// f_J = V_c / Phi_0 = 2*e*V / h
    pub fn josephson_frequency(&self) -> f64 {
        self.characteristic_voltage() / PHI_0
    }

    /// Plasma frequency of the junction [Hz].
    /// omega_p = sqrt(2*pi*I_c / (Phi_0*C))
    pub fn plasma_frequency(&self) -> f64 {
        (2.0 * PI * self.critical_current_a
            / (PHI_0 * self.junction_capacitance_f))
            .sqrt()
            / (2.0 * PI)
    }
}

// ---------------------------------------------------------------------------
// DC SQUID
// ---------------------------------------------------------------------------

/// DC SQUID sensor model.
#[derive(Debug, Clone)]
pub struct DcSquid {
    pub config: SquidConfig,
}

impl DcSquid {
    /// Create a new DC SQUID from configuration.
    pub fn new(config: SquidConfig) -> Self {
        Self { config }
    }

    /// Effective critical current as a function of applied flux.
    ///
    /// I_c(Phi) = 2 * I_c0 * |cos(pi * Phi / Phi_0)|
    pub fn critical_current(&self, flux_wb: f64) -> f64 {
        2.0 * self.config.critical_current_a * (PI * flux_wb / PHI_0).cos().abs()
    }

    /// DC SQUID voltage-flux characteristic.
    ///
    /// V(Phi) = R_n * sqrt(I_bias^2 - (2*I_c*cos(pi*Phi/Phi_0))^2)
    ///
    /// Returns 0 if `I_bias < I_c(Phi)` (sub-critical bias).
    pub fn voltage_flux_characteristic(&self, i_bias_a: f64, flux_wb: f64) -> f64 {
        let ic_eff = self.critical_current(flux_wb);
        let diff = i_bias_a * i_bias_a - ic_eff * ic_eff;
        if diff <= 0.0 {
            return 0.0;
        }
        self.config.shunt_resistance_ohm * diff.sqrt()
    }

    /// Transfer function dV/dPhi at operating point [V/Phi_0].
    ///
    /// Computed by numerical differentiation.
    pub fn transfer_function(&self, i_bias_a: f64, flux_wb: f64) -> f64 {
        let delta = PHI_0 * 1.0e-6;
        let v_plus = self.voltage_flux_characteristic(i_bias_a, flux_wb + delta);
        let v_minus = self.voltage_flux_characteristic(i_bias_a, flux_wb - delta);
        (v_plus - v_minus) / (2.0 * delta) * PHI_0
    }

    /// Maximum transfer function (peak dV/dPhi) for a given bias current.
    /// Scans flux from 0 to Phi_0 and returns the maximum |dV/dPhi|.
    pub fn max_transfer_function(&self, i_bias_a: f64) -> f64 {
        let n = 1000;
        let mut max_tf = 0.0_f64;
        for i in 0..n {
            let phi = PHI_0 * (i as f64) / (n as f64);
            let tf = self.transfer_function(i_bias_a, phi).abs();
            if tf > max_tf {
                max_tf = tf;
            }
        }
        max_tf
    }

    /// Optimal bias current (just above the maximum critical current 2*I_c).
    /// Returns 1.1 * 2 * I_c as a practical operating point.
    pub fn optimal_bias_current(&self) -> f64 {
        2.0 * self.config.critical_current_a * 1.1
    }

    /// Voltage swing (peak-to-peak) vs flux at given bias current.
    pub fn voltage_swing(&self, i_bias_a: f64) -> f64 {
        let n = 1000;
        let mut v_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;
        for i in 0..=n {
            let phi = PHI_0 * (i as f64) / (n as f64);
            let v = self.voltage_flux_characteristic(i_bias_a, phi);
            if v > v_max {
                v_max = v;
            }
            if v < v_min {
                v_min = v;
            }
        }
        v_max - v_min
    }

    /// Generate voltage-flux curve samples for one period.
    pub fn voltage_flux_curve(&self, i_bias_a: f64, num_points: usize) -> Vec<(f64, f64)> {
        (0..num_points)
            .map(|i| {
                let phi = PHI_0 * (i as f64) / (num_points as f64);
                let v = self.voltage_flux_characteristic(i_bias_a, phi);
                (phi, v)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Noise characterisation
// ---------------------------------------------------------------------------

/// SQUID noise parameters.
#[derive(Debug, Clone)]
pub struct SquidNoise {
    /// White flux noise power spectral density [Phi_0^2 / Hz].
    pub white_flux_noise_phi0_sq_per_hz: f64,
    /// Energy sensitivity [J/Hz].
    pub energy_sensitivity_j_per_hz: f64,
    /// Spectral density in fT/sqrt(Hz) (for a given effective area).
    pub field_noise_ft_per_rt_hz: f64,
    /// 1/f noise corner frequency [Hz].
    pub one_over_f_corner_hz: f64,
    /// Noise temperature [K].
    pub noise_temperature_k: f64,
}

/// Compute SQUID noise characteristics.
///
/// - `config`: SQUID configuration
/// - `temperature_k`: operating temperature [K] (typ. 4.2 for LTS, 77 for HTS)
/// - `effective_area_m2`: effective pickup area [m^2]
/// - `one_over_f_corner_hz`: empirical 1/f corner frequency
pub fn compute_squid_noise(
    config: &SquidConfig,
    temperature_k: f64,
    effective_area_m2: f64,
    one_over_f_corner_hz: f64,
) -> SquidNoise {
    // White flux noise: S_Phi = 16 * k_B * T * L^2 / R_n [Wb^2/Hz]
    let s_phi_wb2 = 16.0 * BOLTZMANN_K * temperature_k
        * config.loop_inductance_h.powi(2)
        / config.shunt_resistance_ohm;

    // Convert to [Phi_0^2 / Hz]
    let s_phi_phi0 = s_phi_wb2 / (PHI_0 * PHI_0);

    // Energy sensitivity: epsilon = S_Phi / (2*L)
    let epsilon = s_phi_wb2 / (2.0 * config.loop_inductance_h);

    // Field noise: S_B = S_Phi / A_eff^2,  sqrt(S_B) in T/sqrt(Hz) -> fT/sqrt(Hz)
    let field_noise = if effective_area_m2 > 0.0 {
        (s_phi_wb2.sqrt() / effective_area_m2) * 1.0e15 // T -> fT
    } else {
        0.0
    };

    // Voltage noise: S_V = S_Phi * (dV/dPhi)^2 ~ 16*k_B*T*R_n (simplified)
    let s_v = 16.0 * BOLTZMANN_K * temperature_k * config.shunt_resistance_ohm;

    // Noise temperature: T_N = S_V / (4*k_B*R_n)
    let noise_temp = s_v / (4.0 * BOLTZMANN_K * config.shunt_resistance_ohm);

    SquidNoise {
        white_flux_noise_phi0_sq_per_hz: s_phi_phi0,
        energy_sensitivity_j_per_hz: epsilon,
        field_noise_ft_per_rt_hz: field_noise,
        one_over_f_corner_hz,
        noise_temperature_k: noise_temp,
    }
}

/// Total noise spectral density at a given frequency, including 1/f.
///
/// S(f) = S_white * (1 + f_corner / f)
pub fn noise_spectral_density(noise: &SquidNoise, frequency_hz: f64) -> f64 {
    if frequency_hz <= 0.0 {
        return f64::INFINITY;
    }
    noise.white_flux_noise_phi0_sq_per_hz
        * (1.0 + noise.one_over_f_corner_hz / frequency_hz)
}

// ---------------------------------------------------------------------------
// Flux-Locked Loop (FLL)
// ---------------------------------------------------------------------------

/// Flux-locked loop configuration.
#[derive(Debug, Clone)]
pub struct FllConfig {
    /// Integrator gain [V/(Phi_0*s)].
    pub integrator_gain: f64,
    /// Feedback mutual inductance [H] (flux per amp in feedback coil).
    pub feedback_mutual_inductance_h: f64,
    /// Feedback resistance [Ohm] (converts integrator voltage to current).
    pub feedback_resistance_ohm: f64,
    /// Maximum slew rate [Phi_0/s].
    pub max_slew_rate_phi0_per_s: f64,
    /// Bandwidth [Hz].
    pub bandwidth_hz: f64,
}

/// Flux-locked loop state machine.
#[derive(Debug, Clone)]
pub struct FluxLockedLoop {
    pub config: FllConfig,
    /// Current integrator output [V].
    pub integrator_output_v: f64,
    /// Accumulated feedback flux [Wb].
    pub total_feedback_flux_wb: f64,
    /// Number of flux quanta tracked.
    pub flux_quanta_count: i64,
    /// Lock status.
    pub is_locked: bool,
    /// Sample rate [Hz].
    sample_rate_hz: f64,
}

impl FluxLockedLoop {
    /// Create a new FLL.
    pub fn new(config: FllConfig, sample_rate_hz: f64) -> Self {
        Self {
            config,
            integrator_output_v: 0.0,
            total_feedback_flux_wb: 0.0,
            flux_quanta_count: 0,
            is_locked: false,
            sample_rate_hz,
        }
    }

    /// Reset the FLL state.
    pub fn reset(&mut self) {
        self.integrator_output_v = 0.0;
        self.total_feedback_flux_wb = 0.0;
        self.flux_quanta_count = 0;
        self.is_locked = false;
    }

    /// Process one sample from the SQUID error signal.
    ///
    /// `error_v`: SQUID voltage deviation from the operating point.
    ///
    /// Returns the linearised output voltage proportional to flux.
    pub fn process_sample(&mut self, error_v: f64) -> f64 {
        let dt = 1.0 / self.sample_rate_hz;

        // Integrator update
        let delta_v = self.config.integrator_gain * error_v * dt;
        self.integrator_output_v += delta_v;

        // Slew rate limiting
        let max_delta_flux = self.config.max_slew_rate_phi0_per_s * PHI_0 * dt;
        let delta_flux = (self.integrator_output_v / self.config.feedback_resistance_ohm)
            * self.config.feedback_mutual_inductance_h
            - self.total_feedback_flux_wb;

        let clamped_delta = delta_flux.clamp(-max_delta_flux, max_delta_flux);
        self.total_feedback_flux_wb += clamped_delta;

        // Track flux quanta
        let quanta = (self.total_feedback_flux_wb / PHI_0).round() as i64;
        if quanta != self.flux_quanta_count {
            self.flux_quanta_count = quanta;
        }

        // Lock detection: small error implies lock
        self.is_locked = error_v.abs() < 0.1 * PHI_0 * self.config.integrator_gain;

        self.integrator_output_v
    }

    /// Process a block of error samples, returning linearised output.
    pub fn process_block(&mut self, errors: &[f64]) -> Vec<f64> {
        errors.iter().map(|&e| self.process_sample(e)).collect()
    }

    /// Feedback flux in units of Phi_0.
    pub fn feedback_flux_phi0(&self) -> f64 {
        self.total_feedback_flux_wb / PHI_0
    }

    /// Maximum trackable flux change rate [Phi_0/s].
    pub fn max_trackable_rate(&self) -> f64 {
        self.config.max_slew_rate_phi0_per_s
    }
}

// ---------------------------------------------------------------------------
// Gradiometer configurations
// ---------------------------------------------------------------------------

/// Gradiometer type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GradiometerType {
    /// First-order axial: dBz/dz.
    FirstOrderAxial,
    /// Second-order axial: d^2Bz/dz^2.
    SecondOrderAxial,
    /// First-order planar: dBz/dx.
    FirstOrderPlanar,
}

/// Gradiometer configuration.
#[derive(Debug, Clone)]
pub struct Gradiometer {
    /// Gradiometer type.
    pub grad_type: GradiometerType,
    /// Baseline (separation between coils) [m].
    pub baseline_m: f64,
    /// Coil area [m^2].
    pub coil_area_m2: f64,
    /// Number of turns per coil.
    pub turns: u32,
}

impl Gradiometer {
    /// Create a new gradiometer.
    pub fn new(grad_type: GradiometerType, baseline_m: f64, coil_area_m2: f64, turns: u32) -> Self {
        Self {
            grad_type,
            baseline_m,
            coil_area_m2,
            turns,
        }
    }

    /// Compute gradient from field measurements at the coil positions.
    ///
    /// For first-order axial: fields = [B_top, B_bottom]
    /// For second-order axial: fields = [B_top, B_middle, B_bottom]
    /// For first-order planar: fields = [B_left, B_right]
    ///
    /// Returns the gradient in T/m (first-order) or T/m^2 (second-order).
    pub fn compute_gradient(&self, fields_t: &[f64]) -> Option<f64> {
        match self.grad_type {
            GradiometerType::FirstOrderAxial | GradiometerType::FirstOrderPlanar => {
                if fields_t.len() < 2 {
                    return None;
                }
                Some((fields_t[0] - fields_t[1]) / self.baseline_m)
            }
            GradiometerType::SecondOrderAxial => {
                if fields_t.len() < 3 {
                    return None;
                }
                // Weights: 1, -2, 1
                let d2b = fields_t[0] - 2.0 * fields_t[1] + fields_t[2];
                Some(d2b / (self.baseline_m * self.baseline_m))
            }
        }
    }

    /// Compute the net flux from a set of field values using the gradiometer weights.
    ///
    /// Returns flux in [Wb].
    pub fn compute_flux(&self, fields_t: &[f64]) -> Option<f64> {
        let weights: Vec<f64> = match self.grad_type {
            GradiometerType::FirstOrderAxial | GradiometerType::FirstOrderPlanar => {
                if fields_t.len() < 2 {
                    return None;
                }
                vec![1.0, -1.0]
            }
            GradiometerType::SecondOrderAxial => {
                if fields_t.len() < 3 {
                    return None;
                }
                vec![1.0, -2.0, 1.0]
            }
        };

        let net_flux: f64 = fields_t
            .iter()
            .zip(weights.iter())
            .map(|(&b, &w)| b * w * self.coil_area_m2 * self.turns as f64)
            .sum();

        Some(net_flux)
    }

    /// Common-mode rejection ratio (CMRR) in dB.
    ///
    /// Ideal CMRR is infinite. This estimates the CMRR from baseline and
    /// source distance assuming a dipolar far-field:
    ///
    /// CMRR = 20*log10(2*d / baseline) for first-order,
    ///        where d is distance to the noise source [m].
    pub fn cmrr_db(&self, source_distance_m: f64) -> f64 {
        if self.baseline_m <= 0.0 || source_distance_m <= 0.0 {
            return 0.0;
        }
        match self.grad_type {
            GradiometerType::FirstOrderAxial | GradiometerType::FirstOrderPlanar => {
                20.0 * (2.0 * source_distance_m / self.baseline_m).log10()
            }
            GradiometerType::SecondOrderAxial => {
                // Second-order rejects uniform gradient as well
                40.0 * (source_distance_m / self.baseline_m).log10()
            }
        }
    }

    /// Number of coils in this gradiometer.
    pub fn num_coils(&self) -> usize {
        match self.grad_type {
            GradiometerType::FirstOrderAxial | GradiometerType::FirstOrderPlanar => 2,
            GradiometerType::SecondOrderAxial => 3,
        }
    }
}

// ---------------------------------------------------------------------------
// Flux transformer / pickup coil
// ---------------------------------------------------------------------------

/// Flux transformer coupling a pickup coil to the SQUID input.
#[derive(Debug, Clone)]
pub struct FluxTransformer {
    /// Pickup coil area [m^2].
    pub pickup_area_m2: f64,
    /// Number of turns in the pickup coil.
    pub pickup_turns: u32,
    /// Pickup coil inductance [H].
    pub pickup_inductance_h: f64,
    /// Input coil inductance [H] (wound on the SQUID).
    pub input_inductance_h: f64,
    /// Mutual inductance between input coil and SQUID loop [H].
    pub mutual_inductance_h: f64,
    /// SQUID loop inductance [H].
    pub squid_inductance_h: f64,
}

impl FluxTransformer {
    /// Coupling factor k^2 = M^2 / (L_input * L_squid).
    pub fn coupling_factor(&self) -> f64 {
        self.mutual_inductance_h.powi(2)
            / (self.input_inductance_h * self.squid_inductance_h)
    }

    /// Effective area A_eff = A_pickup * N * M / L_input [m^2].
    pub fn effective_area(&self) -> f64 {
        self.pickup_area_m2
            * self.pickup_turns as f64
            * self.mutual_inductance_h
            / self.input_inductance_h
    }

    /// Flux coupled to the SQUID from an applied field B [Wb].
    ///
    /// Phi_squid = B * A_pickup * N * M / (L_pickup + L_input)
    pub fn coupled_flux(&self, field_t: f64) -> f64 {
        field_t * self.pickup_area_m2 * self.pickup_turns as f64
            * self.mutual_inductance_h
            / (self.pickup_inductance_h + self.input_inductance_h)
    }

    /// Field-to-flux sensitivity: dPhi/dB [Wb/T].
    pub fn field_to_flux_sensitivity(&self) -> f64 {
        self.pickup_area_m2 * self.pickup_turns as f64
            * self.mutual_inductance_h
            / (self.pickup_inductance_h + self.input_inductance_h)
    }

    /// Impedance match ratio L_pickup / L_input.
    /// Optimal when this equals 1 (matched inductances).
    pub fn impedance_match_ratio(&self) -> f64 {
        self.pickup_inductance_h / self.input_inductance_h
    }
}

// ---------------------------------------------------------------------------
// Signal processing
// ---------------------------------------------------------------------------

/// Lock-in detection at a modulation frequency.
///
/// Multiplies `signal` by sine and cosine references at `mod_freq_hz`,
/// then applies a boxcar low-pass filter of `integration_samples`.
///
/// Returns (in-phase, quadrature) components.
pub fn lockin_detect(
    signal: &[f64],
    sample_rate_hz: f64,
    mod_freq_hz: f64,
    integration_samples: usize,
) -> (Vec<f64>, Vec<f64>) {
    let n = signal.len();
    if n == 0 || integration_samples == 0 {
        return (vec![], vec![]);
    }

    // Multiply by references
    let mut x_i = vec![0.0; n];
    let mut x_q = vec![0.0; n];
    for i in 0..n {
        let t = i as f64 / sample_rate_hz;
        let phase = 2.0 * PI * mod_freq_hz * t;
        x_i[i] = signal[i] * phase.cos() * 2.0;
        x_q[i] = signal[i] * phase.sin() * 2.0;
    }

    // Boxcar low-pass
    let out_len = if n >= integration_samples {
        n - integration_samples + 1
    } else {
        0
    };

    let mut out_i = Vec::with_capacity(out_len);
    let mut out_q = Vec::with_capacity(out_len);

    if out_len > 0 {
        let mut sum_i: f64 = x_i[..integration_samples].iter().sum();
        let mut sum_q: f64 = x_q[..integration_samples].iter().sum();
        let inv = 1.0 / integration_samples as f64;

        out_i.push(sum_i * inv);
        out_q.push(sum_q * inv);

        for k in 1..out_len {
            sum_i += x_i[k + integration_samples - 1] - x_i[k - 1];
            sum_q += x_q[k + integration_samples - 1] - x_q[k - 1];
            out_i.push(sum_i * inv);
            out_q.push(sum_q * inv);
        }
    }

    (out_i, out_q)
}

/// Simple bandpass filter using cascaded first-order IIR sections.
///
/// Center frequency `f_center`, bandwidth `bw`, both in Hz.
/// Returns filtered signal.
pub fn bandpass_filter(
    signal: &[f64],
    sample_rate_hz: f64,
    f_center_hz: f64,
    bandwidth_hz: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // Heterodyne to baseband, lowpass, heterodyne back
    let half_bw = bandwidth_hz / 2.0;
    let alpha = if sample_rate_hz > 0.0 {
        let rc = 1.0 / (2.0 * PI * half_bw);
        let dt = 1.0 / sample_rate_hz;
        dt / (rc + dt)
    } else {
        1.0
    };

    // Mix to baseband
    let mut bb_i = vec![0.0; n];
    let mut bb_q = vec![0.0; n];
    for i in 0..n {
        let t = i as f64 / sample_rate_hz;
        let phase = 2.0 * PI * f_center_hz * t;
        bb_i[i] = signal[i] * phase.cos();
        bb_q[i] = signal[i] * phase.sin();
    }

    // Single-pole IIR lowpass
    let mut filt_i = vec![0.0; n];
    let mut filt_q = vec![0.0; n];
    filt_i[0] = alpha * bb_i[0];
    filt_q[0] = alpha * bb_q[0];
    for i in 1..n {
        filt_i[i] = alpha * bb_i[i] + (1.0 - alpha) * filt_i[i - 1];
        filt_q[i] = alpha * bb_q[i] + (1.0 - alpha) * filt_q[i - 1];
    }

    // Mix back up
    let mut output = vec![0.0; n];
    for i in 0..n {
        let t = i as f64 / sample_rate_hz;
        let phase = 2.0 * PI * f_center_hz * t;
        output[i] = 2.0 * (filt_i[i] * phase.cos() + filt_q[i] * phase.sin());
    }

    output
}

/// Environmental noise cancellation using a reference channel.
///
/// Adaptive LMS cancellation: subtracts the correlated component from `signal`
/// using `reference` with `num_taps` filter taps and learning rate `mu`.
pub fn noise_cancellation(
    signal: &[f64],
    reference: &[f64],
    num_taps: usize,
    mu: f64,
) -> Vec<f64> {
    let n = signal.len().min(reference.len());
    if n == 0 || num_taps == 0 {
        return vec![];
    }

    let mut weights = vec![0.0; num_taps];
    let mut output = vec![0.0; n];

    for i in 0..n {
        // FIR output from reference
        let mut y = 0.0;
        for j in 0..num_taps {
            if i >= j {
                y += weights[j] * reference[i - j];
            }
        }

        // Error = signal - estimate of noise
        let error = signal[i] - y;
        output[i] = error;

        // LMS weight update
        let mut ref_power = 0.0;
        for j in 0..num_taps {
            if i >= j {
                ref_power += reference[i - j] * reference[i - j];
            }
        }
        let norm = if ref_power > 1.0e-30 {
            mu / ref_power
        } else {
            mu
        };

        for j in 0..num_taps {
            if i >= j {
                weights[j] += norm * error * reference[i - j];
            }
        }
    }

    output
}

/// Power spectral density estimation via Welch's method (simplified).
///
/// Divides the signal into overlapping segments, applies a Hann window,
/// computes FFT magnitude squared, and averages.
///
/// Returns (frequencies_hz, psd) where psd is in signal_units^2/Hz.
pub fn power_spectral_density(
    signal: &[f64],
    sample_rate_hz: f64,
    segment_size: usize,
) -> (Vec<f64>, Vec<f64>) {
    if signal.is_empty() || segment_size == 0 {
        return (vec![], vec![]);
    }

    let half = segment_size / 2;
    let overlap = half;
    let step = segment_size - overlap;

    // Count segments
    let num_segments = if signal.len() >= segment_size {
        (signal.len() - segment_size) / step + 1
    } else {
        0
    };

    if num_segments == 0 {
        return (vec![], vec![]);
    }

    // Hann window
    let window: Vec<f64> = (0..segment_size)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / segment_size as f64).cos()))
        .collect();
    let window_power: f64 = window.iter().map(|w| w * w).sum::<f64>() / segment_size as f64;

    let output_bins = half + 1;
    let mut psd = vec![0.0; output_bins];

    for seg in 0..num_segments {
        let start = seg * step;
        // Apply window and compute DFT (real input -> half spectrum)
        let windowed: Vec<f64> = (0..segment_size)
            .map(|i| signal[start + i] * window[i])
            .collect();

        for k in 0..output_bins {
            let mut re = 0.0;
            let mut im = 0.0;
            for n in 0..segment_size {
                let angle = 2.0 * PI * k as f64 * n as f64 / segment_size as f64;
                re += windowed[n] * angle.cos();
                im -= windowed[n] * angle.sin();
            }
            psd[k] += (re * re + im * im) / (num_segments as f64);
        }
    }

    // Normalise: PSD = |X(f)|^2 / (fs * N * window_power)
    let norm = sample_rate_hz * segment_size as f64 * window_power;
    for val in &mut psd {
        *val /= norm;
    }

    // Double-sided to single-sided (multiply by 2 except DC and Nyquist)
    for k in 1..output_bins - 1 {
        psd[k] *= 2.0;
    }

    let freqs: Vec<f64> = (0..output_bins)
        .map(|k| k as f64 * sample_rate_hz / segment_size as f64)
        .collect();

    (freqs, psd)
}

// ---------------------------------------------------------------------------
// Application presets
// ---------------------------------------------------------------------------

/// MEG (Magnetoencephalography) system parameters.
#[derive(Debug, Clone)]
pub struct MegSystem {
    /// Number of channels (sensor positions).
    pub num_channels: usize,
    /// Sensor noise floor [fT/sqrt(Hz)].
    pub noise_floor_ft_per_rt_hz: f64,
    /// Bandwidth [Hz].
    pub bandwidth_hz: f64,
    /// Helmet radius [m].
    pub helmet_radius_m: f64,
}

impl MegSystem {
    /// Create a typical whole-head MEG system.
    pub fn typical_whole_head() -> Self {
        Self {
            num_channels: 306,
            noise_floor_ft_per_rt_hz: 3.0,
            bandwidth_hz: 1000.0,
            helmet_radius_m: 0.12,
        }
    }

    /// Minimum detectable field [fT] for a given measurement time [s].
    pub fn min_detectable_field_ft(&self, measurement_time_s: f64) -> f64 {
        if measurement_time_s <= 0.0 {
            return f64::INFINITY;
        }
        self.noise_floor_ft_per_rt_hz / measurement_time_s.sqrt()
    }

    /// SNR for a brain signal of given amplitude [fT].
    pub fn signal_snr_db(&self, signal_ft: f64, measurement_time_s: f64) -> f64 {
        let min_field = self.min_detectable_field_ft(measurement_time_s);
        if min_field <= 0.0 {
            return f64::INFINITY;
        }
        20.0 * (signal_ft / min_field).log10()
    }
}

/// Geomagnetic survey parameters.
#[derive(Debug, Clone)]
pub struct GeomagneticSurvey {
    /// Earth's background field magnitude [T].
    pub earth_field_t: f64,
    /// Anomaly threshold [T].
    pub anomaly_threshold_t: f64,
    /// Survey line spacing [m].
    pub line_spacing_m: f64,
    /// Sensor height above ground [m].
    pub sensor_height_m: f64,
}

impl GeomagneticSurvey {
    /// Default parameters for archaeological survey.
    pub fn archaeological() -> Self {
        Self {
            earth_field_t: 50.0e-6,
            anomaly_threshold_t: 1.0e-9,
            line_spacing_m: 0.5,
            sensor_height_m: 0.3,
        }
    }

    /// Detect anomalies in a field profile.
    /// Returns indices where |field - mean| > threshold.
    pub fn detect_anomalies(&self, field_data_t: &[f64]) -> Vec<usize> {
        if field_data_t.is_empty() {
            return vec![];
        }
        let mean = field_data_t.iter().sum::<f64>() / field_data_t.len() as f64;
        field_data_t
            .iter()
            .enumerate()
            .filter(|(_, &v)| (v - mean).abs() > self.anomaly_threshold_t)
            .map(|(i, _)| i)
            .collect()
    }

    /// Anomaly field magnitude relative to Earth's field [ppm].
    pub fn anomaly_ppm(&self, anomaly_t: f64) -> f64 {
        if self.earth_field_t <= 0.0 {
            return 0.0;
        }
        (anomaly_t / self.earth_field_t) * 1.0e6
    }
}

/// NDE (Non-Destructive Evaluation) defect detection.
#[derive(Debug, Clone)]
pub struct NdeInspection {
    /// Excitation field strength [T].
    pub excitation_field_t: f64,
    /// Scan resolution [m].
    pub scan_resolution_m: f64,
    /// Minimum detectable defect size [m].
    pub min_defect_size_m: f64,
}

impl NdeInspection {
    /// Default configuration for steel pipe inspection.
    pub fn pipe_inspection() -> Self {
        Self {
            excitation_field_t: 0.01,
            scan_resolution_m: 1.0e-3,
            min_defect_size_m: 0.5e-3,
        }
    }

    /// Estimate flux leakage signal from a surface defect.
    ///
    /// Simplified dipole model: B_leak ~ B_exc * (depth/width) * exp(-pi*h/width)
    /// where h is the sensor-to-surface distance.
    pub fn estimate_flux_leakage(
        &self,
        defect_depth_m: f64,
        defect_width_m: f64,
        sensor_distance_m: f64,
    ) -> f64 {
        if defect_width_m <= 0.0 {
            return 0.0;
        }
        self.excitation_field_t
            * (defect_depth_m / defect_width_m)
            * (-PI * sensor_distance_m / defect_width_m).exp()
    }

    /// Detect defects in a scan line (flux leakage profile).
    /// Returns (position_index, peak_amplitude) pairs.
    pub fn detect_defects(&self, scan_data_t: &[f64], threshold_t: f64) -> Vec<(usize, f64)> {
        let mut defects = Vec::new();
        let n = scan_data_t.len();
        if n < 3 {
            return defects;
        }

        // Find local maxima above threshold
        for i in 1..n - 1 {
            let v = scan_data_t[i].abs();
            if v > threshold_t
                && v >= scan_data_t[i - 1].abs()
                && v >= scan_data_t[i + 1].abs()
            {
                defects.push((i, scan_data_t[i]));
            }
        }
        defects
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> SquidConfig {
        SquidConfig {
            critical_current_a: 10.0e-6,
            loop_inductance_h: 100.0e-12,
            shunt_resistance_ohm: 5.0,
            junction_capacitance_f: 1.0e-12,
        }
    }

    // --- Physical constants ---

    #[test]
    fn test_flux_quantum_value() {
        // Phi_0 = h/(2e) ~ 2.0678e-15 Wb
        let expected = PLANCK_H / (2.0 * ELEMENTARY_CHARGE);
        assert!((PHI_0 - expected).abs() < 1.0e-20);
    }

    #[test]
    fn test_constants_positive() {
        assert!(PLANCK_H > 0.0);
        assert!(ELEMENTARY_CHARGE > 0.0);
        assert!(BOLTZMANN_K > 0.0);
        assert!(PHI_0 > 0.0);
    }

    // --- SquidConfig ---

    #[test]
    fn test_stewart_mccumber_parameter() {
        let config = default_config();
        let beta_c = config.stewart_mccumber_parameter();
        // Non-hysteretic requires beta_c < 1
        assert!(beta_c > 0.0);
    }

    #[test]
    fn test_screening_parameter() {
        let config = default_config();
        let beta_l = config.screening_parameter();
        assert!(beta_l > 0.0);
        // beta_L = 2*L*Ic/Phi_0
        let expected = 2.0 * 100.0e-12 * 10.0e-6 / PHI_0;
        assert!((beta_l - expected).abs() / expected < 1.0e-10);
    }

    #[test]
    fn test_characteristic_voltage() {
        let config = default_config();
        let vc = config.characteristic_voltage();
        assert!((vc - 50.0e-6).abs() < 1.0e-15);
    }

    #[test]
    fn test_josephson_frequency() {
        let config = default_config();
        let fj = config.josephson_frequency();
        assert!(fj > 1.0e9); // GHz range for typical SQUIDs
    }

    #[test]
    fn test_plasma_frequency() {
        let config = default_config();
        let fp = config.plasma_frequency();
        assert!(fp > 0.0);
    }

    // --- DC SQUID ---

    #[test]
    fn test_critical_current_at_zero_flux() {
        let squid = DcSquid::new(default_config());
        let ic = squid.critical_current(0.0);
        assert!((ic - 2.0 * 10.0e-6).abs() < 1.0e-15);
    }

    #[test]
    fn test_critical_current_at_half_phi0() {
        let squid = DcSquid::new(default_config());
        let ic = squid.critical_current(0.5 * PHI_0);
        // cos(pi/2) = 0 => Ic = 0
        assert!(ic.abs() < 1.0e-15);
    }

    #[test]
    fn test_critical_current_periodic() {
        let squid = DcSquid::new(default_config());
        let ic0 = squid.critical_current(0.0);
        let ic1 = squid.critical_current(PHI_0);
        assert!((ic0 - ic1).abs() < 1.0e-15);
    }

    #[test]
    fn test_voltage_subcritical_bias() {
        let squid = DcSquid::new(default_config());
        // Bias below critical current -> V = 0
        let v = squid.voltage_flux_characteristic(5.0e-6, 0.0);
        assert_eq!(v, 0.0);
    }

    #[test]
    fn test_voltage_supercritical_bias() {
        let squid = DcSquid::new(default_config());
        let i_bias = 25.0e-6; // above 2*Ic = 20uA
        let v = squid.voltage_flux_characteristic(i_bias, 0.25 * PHI_0);
        assert!(v > 0.0);
    }

    #[test]
    fn test_voltage_periodic_in_phi0() {
        let squid = DcSquid::new(default_config());
        let i_bias = 25.0e-6;
        let v0 = squid.voltage_flux_characteristic(i_bias, 0.3 * PHI_0);
        let v1 = squid.voltage_flux_characteristic(i_bias, 1.3 * PHI_0);
        assert!((v0 - v1).abs() < 1.0e-15);
    }

    #[test]
    fn test_transfer_function_nonzero() {
        let squid = DcSquid::new(default_config());
        let i_bias = 25.0e-6;
        let tf = squid.transfer_function(i_bias, 0.25 * PHI_0);
        assert!(tf.abs() > 0.0);
    }

    #[test]
    fn test_max_transfer_function() {
        let squid = DcSquid::new(default_config());
        let i_bias = 25.0e-6;
        let max_tf = squid.max_transfer_function(i_bias);
        assert!(max_tf > 0.0);
    }

    #[test]
    fn test_optimal_bias_current() {
        let squid = DcSquid::new(default_config());
        let i_opt = squid.optimal_bias_current();
        assert!((i_opt - 22.0e-6).abs() < 1.0e-12);
    }

    #[test]
    fn test_voltage_swing() {
        let squid = DcSquid::new(default_config());
        let i_bias = 25.0e-6;
        let swing = squid.voltage_swing(i_bias);
        assert!(swing > 0.0);
    }

    #[test]
    fn test_voltage_flux_curve_length() {
        let squid = DcSquid::new(default_config());
        let curve = squid.voltage_flux_curve(25.0e-6, 100);
        assert_eq!(curve.len(), 100);
    }

    // --- Noise ---

    #[test]
    fn test_noise_white_flux() {
        let config = default_config();
        let noise = compute_squid_noise(&config, 4.2, 1.0e-4, 1.0);
        assert!(noise.white_flux_noise_phi0_sq_per_hz > 0.0);
    }

    #[test]
    fn test_noise_energy_sensitivity() {
        let config = default_config();
        let noise = compute_squid_noise(&config, 4.2, 1.0e-4, 1.0);
        assert!(noise.energy_sensitivity_j_per_hz > 0.0);
    }

    #[test]
    fn test_noise_temperature() {
        let config = default_config();
        let noise = compute_squid_noise(&config, 4.2, 1.0e-4, 1.0);
        // Noise temperature T_N = S_V/(4*k_B*R_n) = 16*k_B*T*R/(4*k_B*R) = 4*T
        assert!((noise.noise_temperature_k - 4.0 * 4.2).abs() < 0.01);
    }

    #[test]
    fn test_noise_field_ft() {
        let config = default_config();
        let noise = compute_squid_noise(&config, 4.2, 1.0e-4, 1.0);
        assert!(noise.field_noise_ft_per_rt_hz > 0.0);
        assert!(noise.field_noise_ft_per_rt_hz < 1.0e6); // reasonable range
    }

    #[test]
    fn test_noise_spectral_density_1_over_f() {
        let config = default_config();
        let noise = compute_squid_noise(&config, 4.2, 1.0e-4, 10.0);
        let s_low = noise_spectral_density(&noise, 1.0);
        let s_high = noise_spectral_density(&noise, 1000.0);
        // Low frequency should have higher noise due to 1/f
        assert!(s_low > s_high);
    }

    #[test]
    fn test_noise_spectral_density_at_corner() {
        let config = default_config();
        let corner = 10.0;
        let noise = compute_squid_noise(&config, 4.2, 1.0e-4, corner);
        let s_at_corner = noise_spectral_density(&noise, corner);
        // At the corner, S(f) = 2 * S_white
        let expected = 2.0 * noise.white_flux_noise_phi0_sq_per_hz;
        assert!((s_at_corner - expected).abs() / expected < 1.0e-10);
    }

    // --- FLL ---

    #[test]
    fn test_fll_creation() {
        let fll_config = FllConfig {
            integrator_gain: 1.0e6,
            feedback_mutual_inductance_h: 10.0e-9,
            feedback_resistance_ohm: 100.0,
            max_slew_rate_phi0_per_s: 1.0e6,
            bandwidth_hz: 10000.0,
        };
        let fll = FluxLockedLoop::new(fll_config, 1.0e6);
        assert_eq!(fll.integrator_output_v, 0.0);
        assert_eq!(fll.flux_quanta_count, 0);
    }

    #[test]
    fn test_fll_process_zero_error() {
        let fll_config = FllConfig {
            integrator_gain: 1.0e6,
            feedback_mutual_inductance_h: 10.0e-9,
            feedback_resistance_ohm: 100.0,
            max_slew_rate_phi0_per_s: 1.0e6,
            bandwidth_hz: 10000.0,
        };
        let mut fll = FluxLockedLoop::new(fll_config, 1.0e6);
        let out = fll.process_sample(0.0);
        assert_eq!(out, 0.0);
    }

    #[test]
    fn test_fll_integrates_error() {
        let fll_config = FllConfig {
            integrator_gain: 1.0e6,
            feedback_mutual_inductance_h: 10.0e-9,
            feedback_resistance_ohm: 100.0,
            max_slew_rate_phi0_per_s: 1.0e6,
            bandwidth_hz: 10000.0,
        };
        let mut fll = FluxLockedLoop::new(fll_config, 1.0e6);
        // Apply constant positive error
        for _ in 0..100 {
            fll.process_sample(1.0e-6);
        }
        assert!(fll.integrator_output_v > 0.0);
    }

    #[test]
    fn test_fll_process_block() {
        let fll_config = FllConfig {
            integrator_gain: 1.0e6,
            feedback_mutual_inductance_h: 10.0e-9,
            feedback_resistance_ohm: 100.0,
            max_slew_rate_phi0_per_s: 1.0e6,
            bandwidth_hz: 10000.0,
        };
        let mut fll = FluxLockedLoop::new(fll_config, 1.0e6);
        let errors = vec![1.0e-6; 50];
        let output = fll.process_block(&errors);
        assert_eq!(output.len(), 50);
        // Output should be monotonically increasing
        for i in 1..output.len() {
            assert!(output[i] >= output[i - 1]);
        }
    }

    #[test]
    fn test_fll_reset() {
        let fll_config = FllConfig {
            integrator_gain: 1.0e6,
            feedback_mutual_inductance_h: 10.0e-9,
            feedback_resistance_ohm: 100.0,
            max_slew_rate_phi0_per_s: 1.0e6,
            bandwidth_hz: 10000.0,
        };
        let mut fll = FluxLockedLoop::new(fll_config, 1.0e6);
        fll.process_sample(1.0e-3);
        fll.reset();
        assert_eq!(fll.integrator_output_v, 0.0);
        assert_eq!(fll.flux_quanta_count, 0);
    }

    // --- Gradiometer ---

    #[test]
    fn test_first_order_axial_gradient() {
        let grad = Gradiometer::new(GradiometerType::FirstOrderAxial, 0.05, 1.0e-4, 1);
        let g = grad.compute_gradient(&[50.001e-6, 50.000e-6]);
        assert!(g.is_some());
        let g = g.unwrap();
        assert!((g - 0.02e-6).abs() < 1.0e-12);
    }

    #[test]
    fn test_second_order_gradient() {
        let grad = Gradiometer::new(GradiometerType::SecondOrderAxial, 0.05, 1.0e-4, 1);
        // Uniform gradient -> second-order should give zero
        let g = grad.compute_gradient(&[3.0, 2.0, 1.0]);
        assert!(g.is_some());
        let g = g.unwrap();
        // 3 - 2*2 + 1 = 0 -> g = 0
        assert!(g.abs() < 1.0e-15);
    }

    #[test]
    fn test_second_order_gradient_nonzero() {
        let grad = Gradiometer::new(GradiometerType::SecondOrderAxial, 0.05, 1.0e-4, 1);
        // Quadratic field: B = x^2 -> d2B/dx2 = 2
        let g = grad.compute_gradient(&[1.0, 0.0, 1.0]); // 1 - 0 + 1 = 2
        assert!(g.is_some());
        let expected = 2.0 / (0.05 * 0.05);
        assert!((g.unwrap() - expected).abs() < 1.0e-10);
    }

    #[test]
    fn test_planar_gradient() {
        let grad = Gradiometer::new(GradiometerType::FirstOrderPlanar, 0.02, 1.0e-4, 1);
        let g = grad.compute_gradient(&[50.0e-6, 49.9e-6]);
        assert!(g.is_some());
    }

    #[test]
    fn test_gradient_insufficient_fields() {
        let grad = Gradiometer::new(GradiometerType::SecondOrderAxial, 0.05, 1.0e-4, 1);
        assert!(grad.compute_gradient(&[1.0, 2.0]).is_none());
    }

    #[test]
    fn test_gradiometer_flux() {
        let grad = Gradiometer::new(GradiometerType::FirstOrderAxial, 0.05, 1.0e-4, 10);
        let flux = grad.compute_flux(&[50.001e-6, 50.000e-6]);
        assert!(flux.is_some());
        assert!(flux.unwrap() > 0.0);
    }

    #[test]
    fn test_cmrr_first_order() {
        let grad = Gradiometer::new(GradiometerType::FirstOrderAxial, 0.05, 1.0e-4, 1);
        let cmrr = grad.cmrr_db(10.0);
        assert!(cmrr > 40.0); // Should be high for distant source
    }

    #[test]
    fn test_cmrr_second_order_higher() {
        let grad1 = Gradiometer::new(GradiometerType::FirstOrderAxial, 0.05, 1.0e-4, 1);
        let grad2 = Gradiometer::new(GradiometerType::SecondOrderAxial, 0.05, 1.0e-4, 1);
        let cmrr1 = grad1.cmrr_db(10.0);
        let cmrr2 = grad2.cmrr_db(10.0);
        assert!(cmrr2 > cmrr1);
    }

    #[test]
    fn test_num_coils() {
        let g1 = Gradiometer::new(GradiometerType::FirstOrderAxial, 0.05, 1.0e-4, 1);
        let g2 = Gradiometer::new(GradiometerType::SecondOrderAxial, 0.05, 1.0e-4, 1);
        assert_eq!(g1.num_coils(), 2);
        assert_eq!(g2.num_coils(), 3);
    }

    // --- Flux Transformer ---

    #[test]
    fn test_coupling_factor() {
        let ft = FluxTransformer {
            pickup_area_m2: 1.0e-4,
            pickup_turns: 50,
            pickup_inductance_h: 1.0e-6,
            input_inductance_h: 1.0e-6,
            mutual_inductance_h: 5.0e-9,
            squid_inductance_h: 100.0e-12,
        };
        let k2 = ft.coupling_factor();
        assert!(k2 > 0.0);
        assert!(k2 <= 1.0);
    }

    #[test]
    fn test_effective_area() {
        let ft = FluxTransformer {
            pickup_area_m2: 1.0e-4,
            pickup_turns: 50,
            pickup_inductance_h: 1.0e-6,
            input_inductance_h: 1.0e-6,
            mutual_inductance_h: 5.0e-9,
            squid_inductance_h: 100.0e-12,
        };
        let a_eff = ft.effective_area();
        assert!(a_eff > 0.0);
    }

    #[test]
    fn test_coupled_flux() {
        let ft = FluxTransformer {
            pickup_area_m2: 1.0e-4,
            pickup_turns: 50,
            pickup_inductance_h: 1.0e-6,
            input_inductance_h: 1.0e-6,
            mutual_inductance_h: 5.0e-9,
            squid_inductance_h: 100.0e-12,
        };
        let flux = ft.coupled_flux(50.0e-6); // Earth's field
        assert!(flux > 0.0);
    }

    #[test]
    fn test_field_to_flux_sensitivity() {
        let ft = FluxTransformer {
            pickup_area_m2: 1.0e-4,
            pickup_turns: 50,
            pickup_inductance_h: 1.0e-6,
            input_inductance_h: 1.0e-6,
            mutual_inductance_h: 5.0e-9,
            squid_inductance_h: 100.0e-12,
        };
        let sens = ft.field_to_flux_sensitivity();
        assert!(sens > 0.0);
        // Coupled flux should equal sens * B
        let b = 1.0e-9;
        let phi = ft.coupled_flux(b);
        assert!((phi - sens * b).abs() / phi < 1.0e-10);
    }

    #[test]
    fn test_impedance_match_ratio() {
        let ft = FluxTransformer {
            pickup_area_m2: 1.0e-4,
            pickup_turns: 50,
            pickup_inductance_h: 1.0e-6,
            input_inductance_h: 1.0e-6,
            mutual_inductance_h: 5.0e-9,
            squid_inductance_h: 100.0e-12,
        };
        assert!((ft.impedance_match_ratio() - 1.0).abs() < 1.0e-15);
    }

    // --- Signal processing ---

    #[test]
    fn test_lockin_detect_dc() {
        // DC signal at the modulation frequency should yield a non-zero output
        let fs = 10000.0;
        let fmod = 100.0;
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * fmod * i as f64 / fs).cos())
            .collect();
        let (ip, _qp) = lockin_detect(&signal, fs, fmod, 100);
        assert!(!ip.is_empty());
        // In-phase component should be significant
        let max_ip = ip.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_ip > 0.1);
    }

    #[test]
    fn test_lockin_detect_empty() {
        let (ip, qp) = lockin_detect(&[], 1000.0, 100.0, 10);
        assert!(ip.is_empty());
        assert!(qp.is_empty());
    }

    #[test]
    fn test_bandpass_filter_passes_center() {
        let fs = 10000.0;
        let fc = 500.0;
        let n = 2000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * fc * i as f64 / fs).cos())
            .collect();
        let filtered = bandpass_filter(&signal, fs, fc, 200.0);
        assert_eq!(filtered.len(), n);
        // Signal at center frequency should be preserved
        let power: f64 = filtered[n / 2..].iter().map(|x| x * x).sum::<f64>()
            / (n / 2) as f64;
        assert!(power > 0.01);
    }

    #[test]
    fn test_bandpass_filter_rejects_offband() {
        let fs = 10000.0;
        let fc = 500.0;
        let n = 2000;
        // Signal far from the passband
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 4000.0 * i as f64 / fs).cos())
            .collect();
        let filtered = bandpass_filter(&signal, fs, fc, 100.0);
        let power: f64 =
            filtered[n / 2..].iter().map(|x| x * x).sum::<f64>() / (n / 2) as f64;
        // Off-band signal should be attenuated (though simple filter has limited rejection)
        let input_power: f64 =
            signal[n / 2..].iter().map(|x| x * x).sum::<f64>() / (n / 2) as f64;
        assert!(power < input_power);
    }

    #[test]
    fn test_noise_cancellation_reduces_noise() {
        let n = 500;
        // Noise source (reference)
        let reference: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 60.0 * i as f64 / 1000.0).sin())
            .collect();
        // Signal = desired + 0.8 * noise
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                0.1 * (2.0 * PI * 10.0 * i as f64 / 1000.0).sin()
                    + 0.8 * reference[i]
            })
            .collect();
        let cleaned = noise_cancellation(&signal, &reference, 8, 0.01);
        assert_eq!(cleaned.len(), n);
        // RMS of cleaned should be less than signal (noise removed)
        let rms_signal = (signal.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt();
        let rms_cleaned =
            (cleaned[n / 2..].iter().map(|x| x * x).sum::<f64>() / (n / 2) as f64).sqrt();
        assert!(rms_cleaned < rms_signal);
    }

    #[test]
    fn test_psd_estimation() {
        let fs = 1000.0;
        let n = 2048;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / fs).sin())
            .collect();
        let (freqs, psd) = power_spectral_density(&signal, fs, 256);
        assert!(!freqs.is_empty());
        assert_eq!(freqs.len(), psd.len());
        // Peak should be near 100 Hz
        let peak_idx = psd
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_freq = freqs[peak_idx];
        assert!((peak_freq - 100.0).abs() < 10.0);
    }

    // --- Application: MEG ---

    #[test]
    fn test_meg_typical() {
        let meg = MegSystem::typical_whole_head();
        assert_eq!(meg.num_channels, 306);
        assert!(meg.noise_floor_ft_per_rt_hz < 5.0);
    }

    #[test]
    fn test_meg_min_detectable_field() {
        let meg = MegSystem::typical_whole_head();
        let mdf = meg.min_detectable_field_ft(1.0);
        assert!((mdf - meg.noise_floor_ft_per_rt_hz).abs() < 1.0e-10);
        // Longer measurement -> lower detection limit
        let mdf_long = meg.min_detectable_field_ft(100.0);
        assert!(mdf_long < mdf);
    }

    #[test]
    fn test_meg_snr() {
        let meg = MegSystem::typical_whole_head();
        let snr = meg.signal_snr_db(100.0, 1.0);
        assert!(snr > 0.0); // 100 fT signal should be above 3 fT noise floor
    }

    // --- Application: Geomagnetic ---

    #[test]
    fn test_geomagnetic_archaeological() {
        let survey = GeomagneticSurvey::archaeological();
        assert!((survey.earth_field_t - 50.0e-6).abs() < 1.0e-12);
    }

    #[test]
    fn test_anomaly_detection() {
        let survey = GeomagneticSurvey::archaeological();
        let data = vec![
            50.0e-6, 50.0e-6, 50.0e-6, 50.005e-6, 50.0e-6, 50.0e-6,
        ];
        let anomalies = survey.detect_anomalies(&data);
        assert!(!anomalies.is_empty());
        assert!(anomalies.contains(&3));
    }

    #[test]
    fn test_anomaly_ppm() {
        let survey = GeomagneticSurvey::archaeological();
        let ppm = survey.anomaly_ppm(1.0e-9);
        // 1 nT / 50 uT * 1e6 = 20 ppm
        assert!((ppm - 20.0).abs() < 1.0e-6);
    }

    // --- Application: NDE ---

    #[test]
    fn test_nde_flux_leakage() {
        let nde = NdeInspection::pipe_inspection();
        let leak = nde.estimate_flux_leakage(1.0e-3, 2.0e-3, 0.5e-3);
        assert!(leak > 0.0);
    }

    #[test]
    fn test_nde_flux_leakage_decays_with_distance() {
        let nde = NdeInspection::pipe_inspection();
        let leak_close = nde.estimate_flux_leakage(1.0e-3, 2.0e-3, 0.5e-3);
        let leak_far = nde.estimate_flux_leakage(1.0e-3, 2.0e-3, 5.0e-3);
        assert!(leak_close > leak_far);
    }

    #[test]
    fn test_nde_detect_defects() {
        let nde = NdeInspection::pipe_inspection();
        let scan = vec![0.0, 0.0, 0.0, 0.0, 0.5e-3, 0.0, 0.0, 0.0];
        let defects = nde.detect_defects(&scan, 0.1e-3);
        assert!(!defects.is_empty());
        assert_eq!(defects[0].0, 4);
    }
}

//! Electric vehicle motor phase control and commutation for BLDC and PMSM drives.
//!
//! This module implements six-step (trapezoidal) commutation, space vector PWM (SVPWM)
//! generation, Clarke and Park transforms (and their inverses), PI current control,
//! field-oriented control (FOC), Hall sensor decoding, back-EMF zero-crossing detection
//! for sensorless control, and speed/torque estimation.
//!
//! # Example
//!
//! ```
//! use r4w_core::ev_motor_commutation_controller::{
//!     MotorConfig, MotorController, clarke_transform, park_transform,
//!     svpwm, six_step_commute, decode_hall_sensors,
//! };
//!
//! let config = MotorConfig {
//!     pole_pairs: 4,
//!     rated_rpm: 3000.0,
//!     rated_current_a: 10.0,
//!     dc_bus_voltage_v: 48.0,
//!     pwm_freq_hz: 20_000.0,
//! };
//!
//! let mut ctrl = MotorController::new(config);
//!
//! // Measure three-phase currents
//! let (ia, ib, ic) = (5.0, -2.5, -2.5);
//!
//! // Clarke transform: 3-phase -> alpha-beta
//! let (alpha, beta) = clarke_transform(ia, ib, ic);
//!
//! // Park transform: alpha-beta -> d-q (at electrical angle 0)
//! let (id, iq) = park_transform(alpha, beta, 0.0);
//!
//! // Hall sensor decoding
//! let sector = decode_hall_sensors(true, false, true);
//! assert!(sector >= 1 && sector <= 6);
//!
//! // Six-step commutation from Hall state
//! let (pa, pb, pc) = six_step_commute(sector);
//!
//! // SVPWM duty cycles
//! let (da, db, dc) = svpwm(alpha, beta, 48.0);
//! assert!(da >= 0.0 && da <= 1.0);
//! assert!(db >= 0.0 && db <= 1.0);
//! assert!(dc >= 0.0 && dc <= 1.0);
//! ```

use std::f64::consts::PI;

// ──────────────────────────────────────────────────────────────────────────────
// Constants
// ──────────────────────────────────────────────────────────────────────────────

const TWO_PI: f64 = 2.0 * PI;
const SQRT3: f64 = 1.732_050_808_068_872;
const TWO_OVER_THREE: f64 = 2.0 / 3.0;

// ──────────────────────────────────────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────────────────────────────────────

/// Motor electrical and mechanical parameters.
#[derive(Debug, Clone)]
pub struct MotorConfig {
    /// Number of magnetic pole pairs (e.g., 4 for an 8-pole motor).
    pub pole_pairs: u8,
    /// Rated speed in revolutions per minute.
    pub rated_rpm: f64,
    /// Rated phase current in amperes.
    pub rated_current_a: f64,
    /// DC bus voltage in volts.
    pub dc_bus_voltage_v: f64,
    /// PWM switching frequency in hertz.
    pub pwm_freq_hz: f64,
}

// ──────────────────────────────────────────────────────────────────────────────
// State
// ──────────────────────────────────────────────────────────────────────────────

/// Instantaneous commutation state of the motor.
#[derive(Debug, Clone)]
pub struct CommutationState {
    /// Electrical angle in radians (0 .. 2pi).
    pub electrical_angle_rad: f64,
    /// Mechanical speed in RPM.
    pub speed_rpm: f64,
    /// Direct-axis current in amperes.
    pub id_a: f64,
    /// Quadrature-axis current in amperes.
    pub iq_a: f64,
    /// Current Hall sector (1-6).
    pub sector: u8,
}

// ──────────────────────────────────────────────────────────────────────────────
// Motor controller
// ──────────────────────────────────────────────────────────────────────────────

/// Main motor controller holding configuration, PI integrator states, and
/// the latest commutation state.
#[derive(Debug, Clone)]
pub struct MotorController {
    /// Motor configuration parameters.
    pub config: MotorConfig,
    /// Current commutation state.
    pub state: CommutationState,
    /// PI d-axis integrator accumulator.
    pub pi_d_integral: f64,
    /// PI q-axis integrator accumulator.
    pub pi_q_integral: f64,
    /// PI proportional gain.
    pub kp: f64,
    /// PI integral gain.
    pub ki: f64,
}

impl MotorController {
    /// Create a new controller with the given motor configuration.
    ///
    /// Default PI gains are `kp = 1.0`, `ki = 100.0`. Adjust after
    /// construction for your specific motor.
    pub fn new(config: MotorConfig) -> Self {
        Self {
            config,
            state: CommutationState {
                electrical_angle_rad: 0.0,
                speed_rpm: 0.0,
                id_a: 0.0,
                iq_a: 0.0,
                sector: 1,
            },
            pi_d_integral: 0.0,
            pi_q_integral: 0.0,
            kp: 1.0,
            ki: 100.0,
        }
    }

    /// Run one FOC iteration: measure currents, update state, return duty cycles.
    ///
    /// # Arguments
    /// * `id_ref` - desired d-axis current (typically 0 for max torque/amp)
    /// * `iq_ref` - desired q-axis current (torque command)
    /// * `ia`, `ib`, `ic` - measured three-phase currents (amperes)
    /// * `theta` - electrical angle (radians)
    /// * `dt` - control loop period (seconds)
    ///
    /// # Returns
    /// `(duty_a, duty_b, duty_c)` in the range `[0, 1]`.
    pub fn foc_update(
        &mut self,
        id_ref: f64,
        iq_ref: f64,
        ia: f64,
        ib: f64,
        ic: f64,
        theta: f64,
        dt: f64,
    ) -> (f64, f64, f64) {
        let vdc = self.config.dc_bus_voltage_v;
        let (da, db, dc) = foc_step(
            id_ref,
            iq_ref,
            ia,
            ib,
            ic,
            theta,
            vdc,
            &mut self.pi_d_integral,
            &mut self.pi_q_integral,
            self.kp,
            self.ki,
            dt,
        );

        // Update state
        let (alpha, beta) = clarke_transform(ia, ib, ic);
        let (id, iq) = park_transform(alpha, beta, theta);
        self.state.electrical_angle_rad = theta % TWO_PI;
        self.state.id_a = id;
        self.state.iq_a = iq;

        (da, db, dc)
    }

    /// Update the commutation state from Hall sensor readings.
    pub fn update_from_hall(&mut self, ha: bool, hb: bool, hc: bool) {
        let sector = decode_hall_sensors(ha, hb, hc);
        self.state.sector = sector;
        self.state.electrical_angle_rad =
            estimate_electrical_angle(sector, self.config.pole_pairs);
    }

    /// Estimate mechanical speed from consecutive Hall transitions.
    ///
    /// `delta_time_s` is the time between two consecutive Hall edges.
    pub fn estimate_speed_from_hall(&mut self, delta_time_s: f64) {
        if delta_time_s > 0.0 {
            // 6 Hall edges per electrical revolution
            let electrical_freq = 1.0 / (6.0 * delta_time_s);
            let mech_freq = electrical_freq / self.config.pole_pairs as f64;
            self.state.speed_rpm = mech_freq * 60.0;
        }
    }

    /// Estimate torque from q-axis current.
    ///
    /// Uses simplified model: `T = 1.5 * pole_pairs * flux * iq`.
    /// `flux_wb` is the permanent-magnet flux linkage in Weber.
    pub fn estimate_torque(&self, flux_wb: f64) -> f64 {
        estimate_torque(self.config.pole_pairs, flux_wb, self.state.iq_a)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Clarke / Park transforms
// ──────────────────────────────────────────────────────────────────────────────

/// Clarke (a-b-c to alpha-beta) transform using the amplitude-invariant form.
///
/// Converts balanced three-phase quantities into a two-axis stationary
/// reference frame.
///
/// # Arguments
/// * `ia`, `ib`, `ic` - three-phase currents (or voltages)
///
/// # Returns
/// `(alpha, beta)` components.
pub fn clarke_transform(ia: f64, ib: f64, ic: f64) -> (f64, f64) {
    let alpha = TWO_OVER_THREE * (ia - 0.5 * ib - 0.5 * ic);
    let beta = TWO_OVER_THREE * (SQRT3 * 0.5 * (ib - ic));
    (alpha, beta)
}

/// Inverse Clarke (alpha-beta to a-b-c) transform.
///
/// Converts two-axis stationary frame back to three-phase quantities.
pub fn inverse_clarke(alpha: f64, beta: f64) -> (f64, f64, f64) {
    let a = alpha;
    let b = -0.5 * alpha + SQRT3 * 0.5 * beta;
    let c = -0.5 * alpha - SQRT3 * 0.5 * beta;
    (a, b, c)
}

/// Park (alpha-beta to d-q) transform.
///
/// Rotates the stationary alpha-beta frame into the rotor-aligned d-q frame.
///
/// # Arguments
/// * `alpha`, `beta` - stationary frame components
/// * `theta` - electrical angle in radians
///
/// # Returns
/// `(d, q)` rotor-frame components.
pub fn park_transform(alpha: f64, beta: f64, theta: f64) -> (f64, f64) {
    let cos_t = theta.cos();
    let sin_t = theta.sin();
    let d = alpha * cos_t + beta * sin_t;
    let q = -alpha * sin_t + beta * cos_t;
    (d, q)
}

/// Inverse Park (d-q to alpha-beta) transform.
///
/// Rotates rotor-frame quantities back to the stationary frame.
pub fn inverse_park(d: f64, q: f64, theta: f64) -> (f64, f64) {
    let cos_t = theta.cos();
    let sin_t = theta.sin();
    let alpha = d * cos_t - q * sin_t;
    let beta = d * sin_t + q * cos_t;
    (alpha, beta)
}

// ──────────────────────────────────────────────────────────────────────────────
// Space Vector PWM
// ──────────────────────────────────────────────────────────────────────────────

/// Space Vector Pulse Width Modulation.
///
/// Computes per-phase duty cycles from alpha-beta voltage commands and DC bus voltage.
///
/// The implementation determines the voltage vector sector, calculates the
/// dwell times for the two adjacent active vectors, distributes the zero-vector
/// time equally, and maps to phase duty cycles via a min-shift (center-aligned)
/// approach.
///
/// # Returns
/// `(duty_a, duty_b, duty_c)` each clamped to `[0, 1]`.
pub fn svpwm(alpha: f64, beta: f64, vdc: f64) -> (f64, f64, f64) {
    if vdc <= 0.0 {
        return (0.5, 0.5, 0.5);
    }

    // Inverse Clarke to get three reference voltages (for sector determination)
    let v_ref1 = alpha;
    let v_ref2 = -0.5 * alpha + SQRT3 * 0.5 * beta;
    let v_ref3 = -0.5 * alpha - SQRT3 * 0.5 * beta;

    // Determine sector (1-6) from the signs of the three references
    let sector = {
        let a = if v_ref1 > 0.0 { 1u8 } else { 0 };
        let b = if v_ref2 > 0.0 { 2u8 } else { 0 };
        let c = if v_ref3 > 0.0 { 4u8 } else { 0 };
        match a | b | c {
            1 => 1u8,
            3 => 2,
            2 => 3,
            6 => 4,
            4 => 5,
            5 => 6,
            _ => 1, // fallback
        }
    };

    // Normalized voltage magnitude
    let ts = 1.0; // normalised PWM period
    let k = SQRT3 * ts / vdc;

    // Dwell times for the two active vectors
    let (t1, t2) = match sector {
        1 => (k * (SQRT3 * 0.5 * alpha - 0.5 * beta), k * beta),
        2 => (
            k * (SQRT3 * 0.5 * alpha + 0.5 * beta),
            k * (-SQRT3 * 0.5 * alpha + 0.5 * beta),
        ),
        3 => (k * beta, k * (-SQRT3 * 0.5 * alpha - 0.5 * beta)),
        4 => (k * (-SQRT3 * 0.5 * alpha + 0.5 * beta), k * (-beta)),
        5 => (
            k * (-SQRT3 * 0.5 * alpha - 0.5 * beta),
            k * (SQRT3 * 0.5 * alpha - 0.5 * beta),
        ),
        6 => (k * (-beta), k * (SQRT3 * 0.5 * alpha + 0.5 * beta)),
        _ => (0.0, 0.0),
    };

    // Clamp if over-modulated
    let t_sum = t1 + t2;
    let (t1, t2) = if t_sum > ts {
        let scale = ts / t_sum;
        (t1 * scale, t2 * scale)
    } else {
        (t1, t2)
    };

    let t0 = (ts - t1 - t2) * 0.5; // half zero-vector time at start/end

    // Map dwell times to phase duty cycles (center-aligned)
    let (ta, tb, tc) = match sector {
        1 => (t0 + t1 + t2, t0 + t2, t0),
        2 => (t0 + t1, t0 + t1 + t2, t0),
        3 => (t0, t0 + t1 + t2, t0 + t2),
        4 => (t0, t0 + t1, t0 + t1 + t2),
        5 => (t0 + t2, t0, t0 + t1 + t2),
        6 => (t0 + t1 + t2, t0, t0 + t1),
        _ => (0.5, 0.5, 0.5),
    };

    // Normalise to [0,1]
    let clamp01 = |x: f64| x.clamp(0.0, 1.0);
    (clamp01(ta / ts), clamp01(tb / ts), clamp01(tc / ts))
}

// ──────────────────────────────────────────────────────────────────────────────
// Six-step commutation
// ──────────────────────────────────────────────────────────────────────────────

/// Six-step (trapezoidal) commutation from a Hall sensor sector.
///
/// Returns the phase drive state for phases A, B, C as `(-1, 0, 1)`:
/// - `1`  = high-side ON (positive current into winding)
/// - `-1` = low-side ON (current out of winding / freewheeling)
/// - `0`  = floating (high-Z)
///
/// `hall_state` should be in the range 1..=6.
pub fn six_step_commute(hall_state: u8) -> (i8, i8, i8) {
    match hall_state {
        1 => (1, -1, 0),  // A+ B-
        2 => (1, 0, -1),  // A+ C-
        3 => (0, 1, -1),  // B+ C-
        4 => (-1, 1, 0),  // B+ A-
        5 => (-1, 0, 1),  // C+ A-
        6 => (0, -1, 1),  // C+ B-
        _ => (0, 0, 0),   // invalid: all floating
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Hall sensor decoding
// ──────────────────────────────────────────────────────────────────────────────

/// Decode three Hall-effect sensor signals into a commutation sector (1-6).
///
/// Hall sensors are typically spaced 120 degrees apart. The 3-bit pattern maps to
/// one of six valid sectors. Invalid patterns (000, 111) return 0.
pub fn decode_hall_sensors(ha: bool, hb: bool, hc: bool) -> u8 {
    let code = (ha as u8) | ((hb as u8) << 1) | ((hc as u8) << 2);
    match code {
        0b001 => 1,
        0b011 => 2,
        0b010 => 3,
        0b110 => 4,
        0b100 => 5,
        0b101 => 6,
        _ => 0, // invalid (000 or 111)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Electrical angle estimation
// ──────────────────────────────────────────────────────────────────────────────

/// Estimate the electrical angle from a Hall sector and pole-pair count.
///
/// Each sector spans 60 degrees of electrical angle. The returned angle is the
/// centre of the sector. This is a coarse estimate; real controllers
/// interpolate between Hall transitions.
///
/// Returns radians in `[0, 2*pi)`.
pub fn estimate_electrical_angle(hall_sector: u8, _pole_pairs: u8) -> f64 {
    if hall_sector == 0 || hall_sector > 6 {
        return 0.0;
    }
    // Centre of each 60-degree sector: sector 1 -> 30deg, sector 2 -> 90deg, etc.
    let sector_centre_deg = (hall_sector as f64 - 0.5) * 60.0;
    sector_centre_deg.to_radians() % TWO_PI
}

// ──────────────────────────────────────────────────────────────────────────────
// Back-EMF zero-crossing detection (sensorless)
// ──────────────────────────────────────────────────────────────────────────────

/// Detect a back-EMF zero-crossing in a sample buffer.
///
/// Scans the `bemf` slice for the first sign change and returns its index.
/// A zero-crossing occurs where `bemf[i]` and `bemf[i+1]` have opposite
/// signs (or one is exactly zero).
///
/// Returns `None` if no crossing is found.
pub fn detect_bemf_zero_crossing(bemf: &[f64]) -> Option<usize> {
    if bemf.len() < 2 {
        return None;
    }
    for i in 0..bemf.len() - 1 {
        let a = bemf[i];
        let b = bemf[i + 1];
        // Sign change: a and b have different signs, or b is exactly zero
        if (a > 0.0 && b < 0.0) || (a < 0.0 && b > 0.0) || (a != 0.0 && b == 0.0) {
            return Some(i);
        }
    }
    None
}

// ──────────────────────────────────────────────────────────────────────────────
// PI controller
// ──────────────────────────────────────────────────────────────────────────────

/// Proportional-Integral controller with anti-windup clamping.
///
/// # Arguments
/// * `error` - current error signal
/// * `integral` - mutable reference to the integrator accumulator
/// * `kp` - proportional gain
/// * `ki` - integral gain
/// * `dt` - time step (seconds)
/// * `limit` - symmetric output saturation limit (plus/minus limit)
///
/// # Returns
/// Controller output clamped to `[-limit, +limit]`.
pub fn pi_controller(
    error: f64,
    integral: &mut f64,
    kp: f64,
    ki: f64,
    dt: f64,
    limit: f64,
) -> f64 {
    // Accumulate integral term
    *integral += error * dt;

    // Anti-windup: clamp integral contribution
    let i_term = ki * *integral;
    let i_clamped = i_term.clamp(-limit, limit);
    if (i_term - i_clamped).abs() > 1e-15 {
        // Back-calculate integral to prevent windup
        if ki.abs() > 1e-15 {
            *integral = i_clamped / ki;
        }
    }

    let output = kp * error + i_clamped;
    output.clamp(-limit, limit)
}

// ──────────────────────────────────────────────────────────────────────────────
// Field-Oriented Control (FOC) step
// ──────────────────────────────────────────────────────────────────────────────

/// Execute one full field-oriented control iteration.
///
/// 1. Clarke transform on measured currents.
/// 2. Park transform to get d-q currents.
/// 3. PI controllers for d-axis and q-axis.
/// 4. Inverse Park to get alpha-beta voltages.
/// 5. SVPWM to get per-phase duty cycles.
///
/// # Arguments
/// * `id_ref`, `iq_ref` - reference d/q currents
/// * `ia`, `ib`, `ic` - measured three-phase currents
/// * `theta` - electrical angle (radians)
/// * `vdc` - DC bus voltage
/// * `pi_d`, `pi_q` - mutable PI integrator states
/// * `kp`, `ki` - PI gains (shared for both axes)
/// * `dt` - control period (seconds)
///
/// # Returns
/// `(duty_a, duty_b, duty_c)` in `[0, 1]`.
pub fn foc_step(
    id_ref: f64,
    iq_ref: f64,
    ia: f64,
    ib: f64,
    ic: f64,
    theta: f64,
    vdc: f64,
    pi_d: &mut f64,
    pi_q: &mut f64,
    kp: f64,
    ki: f64,
    dt: f64,
) -> (f64, f64, f64) {
    let limit = vdc * 0.5;

    // Step 1: Clarke
    let (alpha_i, beta_i) = clarke_transform(ia, ib, ic);

    // Step 2: Park
    let (id_meas, iq_meas) = park_transform(alpha_i, beta_i, theta);

    // Step 3: PI controllers
    let vd = pi_controller(id_ref - id_meas, pi_d, kp, ki, dt, limit);
    let vq = pi_controller(iq_ref - iq_meas, pi_q, kp, ki, dt, limit);

    // Step 4: Inverse Park
    let (v_alpha, v_beta) = inverse_park(vd, vq, theta);

    // Step 5: SVPWM
    svpwm(v_alpha, v_beta, vdc)
}

// ──────────────────────────────────────────────────────────────────────────────
// Speed & torque estimation
// ──────────────────────────────────────────────────────────────────────────────

/// Estimate mechanical speed from Hall sensor transition period.
///
/// `delta_time_s` is the time between two consecutive Hall edges.
/// There are 6 edges per electrical revolution.
///
/// Returns speed in RPM.
pub fn estimate_speed_rpm(delta_time_s: f64, pole_pairs: u8) -> f64 {
    if delta_time_s <= 0.0 || pole_pairs == 0 {
        return 0.0;
    }
    let electrical_freq = 1.0 / (6.0 * delta_time_s);
    let mech_freq = electrical_freq / pole_pairs as f64;
    mech_freq * 60.0
}

/// Estimate electromagnetic torque from q-axis current and flux linkage.
///
/// `T = 1.5 * P * lambda_m * iq` where P is pole pairs, lambda_m is PM flux in Weber.
pub fn estimate_torque(pole_pairs: u8, flux_wb: f64, iq_a: f64) -> f64 {
    1.5 * pole_pairs as f64 * flux_wb * iq_a
}

// ──────────────────────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPS: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- Clarke transform -------------------------------------------------

    #[test]
    fn test_clarke_balanced_positive_a() {
        // When ia = 1, ib = -0.5, ic = -0.5 (balanced), alpha should be 1, beta should be 0
        let (alpha, beta) = clarke_transform(1.0, -0.5, -0.5);
        assert!(approx_eq(alpha, 1.0, EPS), "alpha = {alpha}");
        assert!(approx_eq(beta, 0.0, EPS), "beta = {beta}");
    }

    #[test]
    fn test_clarke_balanced_positive_b() {
        // When ib is positive and ia, ic are negative halves
        let (alpha, beta) = clarke_transform(-0.5, 1.0, -0.5);
        assert!(approx_eq(alpha, -0.5, EPS), "alpha = {alpha}");
        assert!(approx_eq(beta, SQRT3 / 2.0, EPS), "beta = {beta}");
    }

    #[test]
    fn test_clarke_zero_input() {
        let (alpha, beta) = clarke_transform(0.0, 0.0, 0.0);
        assert!(approx_eq(alpha, 0.0, EPS));
        assert!(approx_eq(beta, 0.0, EPS));
    }

    // -- Inverse Clarke ---------------------------------------------------

    #[test]
    fn test_inverse_clarke_roundtrip() {
        let ia = 3.0;
        let ib = -1.5;
        let ic = -1.5;
        let (alpha, beta) = clarke_transform(ia, ib, ic);
        let (ra, rb, rc) = inverse_clarke(alpha, beta);
        assert!(approx_eq(ra, ia, EPS), "a: {ra} vs {ia}");
        assert!(approx_eq(rb, ib, EPS), "b: {rb} vs {ib}");
        assert!(approx_eq(rc, ic, EPS), "c: {rc} vs {ic}");
    }

    #[test]
    fn test_inverse_clarke_sums_to_zero() {
        let (a, b, c) = inverse_clarke(5.0, 3.0);
        assert!(approx_eq(a + b + c, 0.0, EPS));
    }

    // -- Park transform ---------------------------------------------------

    #[test]
    fn test_park_at_zero_angle() {
        let (d, q) = park_transform(1.0, 0.0, 0.0);
        assert!(approx_eq(d, 1.0, EPS), "d = {d}");
        assert!(approx_eq(q, 0.0, EPS), "q = {q}");
    }

    #[test]
    fn test_park_at_90_degrees() {
        let theta = PI / 2.0;
        let (d, q) = park_transform(1.0, 0.0, theta);
        assert!(approx_eq(d, 0.0, EPS), "d = {d}");
        assert!(approx_eq(q, -1.0, EPS), "q = {q}");
    }

    // -- Inverse Park -----------------------------------------------------

    #[test]
    fn test_inverse_park_roundtrip() {
        let alpha_orig = 2.5;
        let beta_orig = -1.3;
        let theta = 1.23;
        let (d, q) = park_transform(alpha_orig, beta_orig, theta);
        let (alpha, beta) = inverse_park(d, q, theta);
        assert!(approx_eq(alpha, alpha_orig, EPS), "alpha: {alpha}");
        assert!(approx_eq(beta, beta_orig, EPS), "beta: {beta}");
    }

    #[test]
    fn test_park_inverse_park_identity() {
        let d = 4.0;
        let q = -2.0;
        let theta = 0.7;
        let (alpha, beta) = inverse_park(d, q, theta);
        let (d2, q2) = park_transform(alpha, beta, theta);
        assert!(approx_eq(d2, d, EPS));
        assert!(approx_eq(q2, q, EPS));
    }

    // -- SVPWM ------------------------------------------------------------

    #[test]
    fn test_svpwm_zero_reference() {
        let (da, db, dc) = svpwm(0.0, 0.0, 48.0);
        assert!(approx_eq(da, 0.5, EPS), "da = {da}");
        assert!(approx_eq(db, 0.5, EPS), "db = {db}");
        assert!(approx_eq(dc, 0.5, EPS), "dc = {dc}");
    }

    #[test]
    fn test_svpwm_duties_in_range() {
        // Sweep through angles
        for deg in 0..360 {
            let theta = (deg as f64).to_radians();
            let alpha = 10.0 * theta.cos();
            let beta = 10.0 * theta.sin();
            let (da, db, dc) = svpwm(alpha, beta, 48.0);
            assert!(da >= 0.0 && da <= 1.0, "da={da} at {deg} deg");
            assert!(db >= 0.0 && db <= 1.0, "db={db} at {deg} deg");
            assert!(dc >= 0.0 && dc <= 1.0, "dc={dc} at {deg} deg");
        }
    }

    #[test]
    fn test_svpwm_zero_bus_voltage() {
        let (da, db, dc) = svpwm(10.0, 5.0, 0.0);
        assert!(approx_eq(da, 0.5, EPS));
        assert!(approx_eq(db, 0.5, EPS));
        assert!(approx_eq(dc, 0.5, EPS));
    }

    // -- Six-step commutation ---------------------------------------------

    #[test]
    fn test_six_step_all_sectors() {
        let expected: [(u8, (i8, i8, i8)); 6] = [
            (1, (1, -1, 0)),
            (2, (1, 0, -1)),
            (3, (0, 1, -1)),
            (4, (-1, 1, 0)),
            (5, (-1, 0, 1)),
            (6, (0, -1, 1)),
        ];
        for (sector, expected_phases) in &expected {
            let phases = six_step_commute(*sector);
            assert_eq!(phases, *expected_phases, "sector {sector}");
        }
    }

    #[test]
    fn test_six_step_invalid_sector() {
        assert_eq!(six_step_commute(0), (0, 0, 0));
        assert_eq!(six_step_commute(7), (0, 0, 0));
        assert_eq!(six_step_commute(255), (0, 0, 0));
    }

    // -- Hall sensor decoding ---------------------------------------------

    #[test]
    fn test_decode_hall_all_valid() {
        assert_eq!(decode_hall_sensors(true, false, false), 1);
        assert_eq!(decode_hall_sensors(true, true, false), 2);
        assert_eq!(decode_hall_sensors(false, true, false), 3);
        assert_eq!(decode_hall_sensors(false, true, true), 4);
        assert_eq!(decode_hall_sensors(false, false, true), 5);
        assert_eq!(decode_hall_sensors(true, false, true), 6);
    }

    #[test]
    fn test_decode_hall_invalid() {
        assert_eq!(decode_hall_sensors(false, false, false), 0);
        assert_eq!(decode_hall_sensors(true, true, true), 0);
    }

    // -- Electrical angle estimation --------------------------------------

    #[test]
    fn test_estimate_electrical_angle_sectors() {
        for sector in 1..=6u8 {
            let angle = estimate_electrical_angle(sector, 4);
            assert!(angle >= 0.0 && angle < TWO_PI, "angle={angle} at sector {sector}");
        }
        // Sector 1 -> centre at 30 degrees = pi/6
        let a1 = estimate_electrical_angle(1, 4);
        assert!(approx_eq(a1, (30.0_f64).to_radians(), EPS));
    }

    #[test]
    fn test_estimate_electrical_angle_invalid() {
        assert!(approx_eq(estimate_electrical_angle(0, 4), 0.0, EPS));
        assert!(approx_eq(estimate_electrical_angle(7, 4), 0.0, EPS));
    }

    // -- Back-EMF zero-crossing -------------------------------------------

    #[test]
    fn test_bemf_zero_crossing_found() {
        let bemf = [1.0, 0.5, -0.3, -1.0];
        assert_eq!(detect_bemf_zero_crossing(&bemf), Some(1));
    }

    #[test]
    fn test_bemf_zero_crossing_not_found() {
        let bemf = [1.0, 2.0, 3.0, 4.0];
        assert_eq!(detect_bemf_zero_crossing(&bemf), None);
    }

    #[test]
    fn test_bemf_zero_crossing_exact_zero() {
        // If signal goes from nonzero to exactly zero, that counts
        let bemf = [1.0, 0.0, -1.0];
        assert_eq!(detect_bemf_zero_crossing(&bemf), Some(0));
    }

    #[test]
    fn test_bemf_zero_crossing_empty() {
        assert_eq!(detect_bemf_zero_crossing(&[]), None);
        assert_eq!(detect_bemf_zero_crossing(&[1.0]), None);
    }

    // -- PI controller ----------------------------------------------------

    #[test]
    fn test_pi_controller_proportional_only() {
        let mut integral = 0.0;
        let out = pi_controller(1.0, &mut integral, 2.0, 0.0, 0.001, 100.0);
        assert!(approx_eq(out, 2.0, EPS));
    }

    #[test]
    fn test_pi_controller_clamping() {
        let mut integral = 0.0;
        let out = pi_controller(100.0, &mut integral, 10.0, 0.0, 0.001, 5.0);
        assert!(approx_eq(out, 5.0, EPS), "output should be clamped to 5.0, got {out}");
    }

    #[test]
    fn test_pi_controller_integral_accumulation() {
        let mut integral = 0.0;
        let ki = 100.0;
        let dt = 0.001;
        // Run 10 steps with constant error
        for _ in 0..10 {
            pi_controller(1.0, &mut integral, 0.0, ki, dt, 1000.0);
        }
        // integral should be approximately 10 * 1.0 * 0.001 = 0.01
        assert!(approx_eq(integral, 0.01, 1e-6), "integral = {integral}");
    }

    // -- FOC step ---------------------------------------------------------

    #[test]
    fn test_foc_step_zero_current() {
        let mut pi_d = 0.0;
        let mut pi_q = 0.0;
        let (da, db, dc) = foc_step(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 48.0, &mut pi_d, &mut pi_q, 1.0, 100.0, 0.001,
        );
        // With zero references and zero currents, duties should be around 0.5
        assert!(da >= 0.0 && da <= 1.0);
        assert!(db >= 0.0 && db <= 1.0);
        assert!(dc >= 0.0 && dc <= 1.0);
    }

    // -- Speed estimation -------------------------------------------------

    #[test]
    fn test_estimate_speed_rpm_basic() {
        // 4 pole-pairs, 1ms between Hall edges
        // electrical freq = 1/(6*0.001) = 166.67 Hz
        // mech freq = 166.67 / 4 = 41.667 Hz
        // RPM = 41.667 * 60 = 2500
        let rpm = estimate_speed_rpm(0.001, 4);
        assert!(approx_eq(rpm, 2500.0, 0.1), "rpm = {rpm}");
    }

    #[test]
    fn test_estimate_speed_zero_time() {
        assert!(approx_eq(estimate_speed_rpm(0.0, 4), 0.0, EPS));
        assert!(approx_eq(estimate_speed_rpm(-1.0, 4), 0.0, EPS));
    }

    // -- Torque estimation ------------------------------------------------

    #[test]
    fn test_estimate_torque_basic() {
        // T = 1.5 * 4 * 0.1 * 5.0 = 3.0 Nm
        let t = estimate_torque(4, 0.1, 5.0);
        assert!(approx_eq(t, 3.0, EPS), "torque = {t}");
    }

    // -- MotorController integration --------------------------------------

    #[test]
    fn test_motor_controller_new() {
        let config = MotorConfig {
            pole_pairs: 4,
            rated_rpm: 3000.0,
            rated_current_a: 10.0,
            dc_bus_voltage_v: 48.0,
            pwm_freq_hz: 20_000.0,
        };
        let ctrl = MotorController::new(config);
        assert_eq!(ctrl.state.sector, 1);
        assert!(approx_eq(ctrl.state.speed_rpm, 0.0, EPS));
        assert!(approx_eq(ctrl.pi_d_integral, 0.0, EPS));
    }

    #[test]
    fn test_motor_controller_hall_update() {
        let config = MotorConfig {
            pole_pairs: 4,
            rated_rpm: 3000.0,
            rated_current_a: 10.0,
            dc_bus_voltage_v: 48.0,
            pwm_freq_hz: 20_000.0,
        };
        let mut ctrl = MotorController::new(config);
        ctrl.update_from_hall(false, true, false);
        assert_eq!(ctrl.state.sector, 3);
        let expected_angle = (2.5_f64 * 60.0).to_radians();
        assert!(
            approx_eq(ctrl.state.electrical_angle_rad, expected_angle, EPS),
            "angle = {}",
            ctrl.state.electrical_angle_rad
        );
    }

    #[test]
    fn test_motor_controller_speed_estimation() {
        let config = MotorConfig {
            pole_pairs: 4,
            rated_rpm: 3000.0,
            rated_current_a: 10.0,
            dc_bus_voltage_v: 48.0,
            pwm_freq_hz: 20_000.0,
        };
        let mut ctrl = MotorController::new(config);
        ctrl.estimate_speed_from_hall(0.001);
        assert!(approx_eq(ctrl.state.speed_rpm, 2500.0, 0.1));
    }

    #[test]
    fn test_motor_controller_foc_update() {
        let config = MotorConfig {
            pole_pairs: 4,
            rated_rpm: 3000.0,
            rated_current_a: 10.0,
            dc_bus_voltage_v: 48.0,
            pwm_freq_hz: 20_000.0,
        };
        let mut ctrl = MotorController::new(config);
        let (da, db, dc) = ctrl.foc_update(0.0, 5.0, 1.0, -0.5, -0.5, 0.5, 0.001);
        assert!(da >= 0.0 && da <= 1.0, "da = {da}");
        assert!(db >= 0.0 && db <= 1.0, "db = {db}");
        assert!(dc >= 0.0 && dc <= 1.0, "dc = {dc}");
    }

    #[test]
    fn test_motor_controller_torque() {
        let config = MotorConfig {
            pole_pairs: 4,
            rated_rpm: 3000.0,
            rated_current_a: 10.0,
            dc_bus_voltage_v: 48.0,
            pwm_freq_hz: 20_000.0,
        };
        let mut ctrl = MotorController::new(config);
        ctrl.state.iq_a = 5.0;
        let t = ctrl.estimate_torque(0.1);
        assert!(approx_eq(t, 3.0, EPS), "torque = {t}");
    }
}

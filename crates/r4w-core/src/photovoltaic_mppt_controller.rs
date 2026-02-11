//! # Photovoltaic MPPT Controller
//!
//! Maximum Power Point Tracking (MPPT) algorithms for solar photovoltaic systems.
//!
//! This module implements several MPPT methods used in solar charge controllers
//! and grid-tied inverters to extract maximum power from photovoltaic panels
//! under varying irradiance and temperature conditions.
//!
//! ## Algorithms
//!
//! - **Perturb & Observe (P&O)**: Hill-climbing method that periodically perturbs
//!   the operating voltage and observes the resulting power change.
//! - **Incremental Conductance (IC)**: Compares instantaneous conductance (I/V)
//!   with incremental conductance (dI/dV) to determine MPP direction.
//! - **Fractional Open-Circuit Voltage (Frac V_oc)**: Approximates V_mpp as a
//!   fixed fraction (typically 0.72-0.78) of the open-circuit voltage.
//! - **Constant Voltage**: Maintains a fixed reference voltage (simplest method).
//!
//! ## Solar Cell Model
//!
//! The single-diode model computes current from voltage:
//!
//! ```text
//! I = I_sc - I_sc * (exp(V / (n * V_t)) - 1) / (exp(V_oc / (n * V_t)) - 1)
//! ```
//!
//! where `V_t = kT/q` is the thermal voltage, `n` is the ideality factor,
//! and `I_sc`, `V_oc` are the short-circuit current and open-circuit voltage.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::photovoltaic_mppt_controller::{
//!     MpptController, MpptConfig, MpptMethod,
//! };
//!
//! let config = MpptConfig {
//!     v_oc_nominal: 40.0,
//!     i_sc_nominal: 10.0,
//!     num_cells_series: 60,
//!     step_size_v: 0.5,
//!     method: MpptMethod::PerturbAndObserve,
//! };
//!
//! let mut controller = MpptController::new(config);
//! let output = controller.update(32.0, 9.0);
//! assert!(output.power_w > 0.0);
//! ```

/// Boltzmann constant in J/K.
const K_BOLTZMANN: f64 = 1.380_649e-23;

/// Elementary charge in C.
const Q_ELECTRON: f64 = 1.602_176_634e-19;

/// Standard Test Conditions temperature in Kelvin (25 C).
const STC_TEMP_K: f64 = 298.15;

/// Fractional V_oc coefficient (typical for crystalline silicon).
const FRAC_VOC_COEFFICIENT: f64 = 0.76;

/// Tolerance for floating-point comparisons in operating point detection.
const CONDUCTANCE_TOLERANCE: f64 = 1e-6;

/// MPPT algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MpptMethod {
    /// Perturb & Observe: hill-climbing with periodic voltage perturbation.
    PerturbAndObserve,
    /// Incremental Conductance: dI/dV + I/V comparison for MPP detection.
    IncrementalConductance,
    /// Fractional Open-Circuit Voltage: V_ref = k * V_oc.
    FractionalVoc,
    /// Constant Voltage: maintain a fixed reference voltage.
    ConstantVoltage,
}

impl Default for MpptMethod {
    fn default() -> Self {
        MpptMethod::PerturbAndObserve
    }
}

/// Operating point classification relative to the Maximum Power Point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperatingPoint {
    /// Voltage is below the MPP voltage (left side of P-V curve).
    BelowMpp,
    /// Voltage is at or very near the MPP (peak of P-V curve).
    AtMpp,
    /// Voltage is above the MPP voltage (right side of P-V curve).
    AboveMpp,
}

/// Configuration for the MPPT controller.
#[derive(Debug, Clone)]
pub struct MpptConfig {
    /// Nominal open-circuit voltage of the PV module in volts.
    pub v_oc_nominal: f64,
    /// Nominal short-circuit current of the PV module in amps.
    pub i_sc_nominal: f64,
    /// Number of cells connected in series.
    pub num_cells_series: usize,
    /// Voltage perturbation step size in volts.
    pub step_size_v: f64,
    /// MPPT algorithm to use.
    pub method: MpptMethod,
}

impl Default for MpptConfig {
    fn default() -> Self {
        Self {
            v_oc_nominal: 40.0,
            i_sc_nominal: 10.0,
            num_cells_series: 60,
            step_size_v: 0.5,
            method: MpptMethod::PerturbAndObserve,
        }
    }
}

/// Output of an MPPT controller update step.
#[derive(Debug, Clone)]
pub struct MpptOutput {
    /// Reference voltage command for the converter in volts.
    pub reference_voltage: f64,
    /// Instantaneous power in watts (V * I).
    pub power_w: f64,
    /// Tracking efficiency as a percentage of estimated maximum power.
    pub efficiency_percent: f64,
    /// Duty cycle command for the DC-DC converter (0.0 to 1.0).
    pub duty_cycle: f64,
    /// Classification of the current operating point.
    pub operating_point: OperatingPoint,
}

/// MPPT controller state machine.
///
/// Tracks the PV module operating point and generates voltage reference
/// commands to converge on the maximum power point.
#[derive(Debug, Clone)]
pub struct MpptController {
    config: MpptConfig,
    /// Previous voltage measurement.
    v_prev: f64,
    /// Previous current measurement.
    i_prev: f64,
    /// Previous power measurement.
    p_prev: f64,
    /// Current voltage reference command.
    v_ref: f64,
    /// Direction of last perturbation (+1 or -1).
    direction: f64,
    /// Whether this is the first update (no previous data).
    first_update: bool,
    /// Estimated maximum power for efficiency calculation.
    p_max_estimate: f64,
}

impl MpptController {
    /// Create a new MPPT controller with the given configuration.
    ///
    /// The initial voltage reference is set to 76% of V_oc (typical MPP ratio
    /// for crystalline silicon).
    pub fn new(config: MpptConfig) -> Self {
        let v_ref = config.v_oc_nominal * FRAC_VOC_COEFFICIENT;
        let p_max_estimate = config.v_oc_nominal * FRAC_VOC_COEFFICIENT
            * config.i_sc_nominal * 0.92; // typical fill factor estimate
        Self {
            v_ref,
            v_prev: 0.0,
            i_prev: 0.0,
            p_prev: 0.0,
            direction: 1.0,
            first_update: true,
            p_max_estimate,
            config,
        }
    }

    /// Process a new voltage/current measurement and return the updated
    /// MPPT output with a new voltage reference command.
    ///
    /// # Arguments
    ///
    /// * `voltage` - Measured PV module voltage in volts.
    /// * `current` - Measured PV module current in amps.
    ///
    /// # Returns
    ///
    /// An [`MpptOutput`] containing the new reference voltage, power,
    /// efficiency estimate, duty cycle, and operating point classification.
    pub fn update(&mut self, voltage: f64, current: f64) -> MpptOutput {
        let power = voltage * current;

        if self.first_update {
            self.v_prev = voltage;
            self.i_prev = current;
            self.p_prev = power;
            self.first_update = false;
            // On first call, keep the initial reference
        } else {
            match self.config.method {
                MpptMethod::PerturbAndObserve => {
                    self.v_ref = perturb_and_observe(
                        self.v_prev,
                        self.p_prev,
                        voltage,
                        power,
                        self.config.step_size_v,
                    );
                }
                MpptMethod::IncrementalConductance => {
                    let dv = voltage - self.v_prev;
                    let di = current - self.i_prev;
                    let op = incremental_conductance(voltage, current, dv, di);
                    match op {
                        OperatingPoint::BelowMpp => {
                            self.v_ref += self.config.step_size_v;
                        }
                        OperatingPoint::AtMpp => {
                            // Hold current reference
                        }
                        OperatingPoint::AboveMpp => {
                            self.v_ref -= self.config.step_size_v;
                        }
                    }
                }
                MpptMethod::FractionalVoc => {
                    // Use measured V_oc estimate (scale from current operating point)
                    // In practice, V_oc is measured by briefly disconnecting the load.
                    // Here we use the nominal value.
                    self.v_ref = self.config.v_oc_nominal * FRAC_VOC_COEFFICIENT;
                }
                MpptMethod::ConstantVoltage => {
                    // v_ref stays at the initial value
                }
            }

            self.v_prev = voltage;
            self.i_prev = current;
            self.p_prev = power;
        }

        // Clamp reference voltage to valid range
        self.v_ref = self.v_ref.clamp(0.0, self.config.v_oc_nominal);

        // Update max power estimate if we found higher power
        if power > self.p_max_estimate {
            self.p_max_estimate = power;
        }

        let efficiency = if self.p_max_estimate > 0.0 {
            (power / self.p_max_estimate * 100.0).min(100.0)
        } else {
            0.0
        };

        // Determine operating point based on incremental conductance
        let dv = voltage - self.v_prev;
        let di = current - self.i_prev;
        let operating_point = if self.first_update || dv.abs() < 1e-12 {
            // Cannot determine without previous data or no voltage change
            OperatingPoint::AtMpp
        } else {
            incremental_conductance(voltage, current, dv, di)
        };

        // Compute duty cycle assuming a buck converter with V_out = V_battery (nominal 24V)
        let v_battery = self.config.v_oc_nominal * 0.6; // typical battery voltage
        let duty = duty_cycle_for_voltage(voltage, v_battery);

        MpptOutput {
            reference_voltage: self.v_ref,
            power_w: power,
            efficiency_percent: efficiency,
            duty_cycle: duty,
            operating_point,
        }
    }
}

/// Perturb & Observe algorithm: determine the next reference voltage.
///
/// Compares current power with previous power and adjusts the voltage
/// in the direction that increased power.
///
/// # Arguments
///
/// * `v_prev` - Previous operating voltage in volts.
/// * `p_prev` - Previous power in watts.
/// * `v_now` - Current operating voltage in volts.
/// * `p_now` - Current power in watts.
/// * `step` - Voltage perturbation step size in volts.
///
/// # Returns
///
/// The next voltage reference in volts.
pub fn perturb_and_observe(v_prev: f64, p_prev: f64, v_now: f64, p_now: f64, step: f64) -> f64 {
    let dp = p_now - p_prev;
    let dv = v_now - v_prev;

    if dp.abs() < 1e-12 {
        // No power change; hold current voltage
        v_now
    } else if dp > 0.0 {
        // Power increased: continue in the same direction
        if dv > 0.0 {
            v_now + step
        } else {
            v_now - step
        }
    } else {
        // Power decreased: reverse direction
        if dv > 0.0 {
            v_now - step
        } else {
            v_now + step
        }
    }
}

/// Incremental Conductance method: classify the operating point.
///
/// Compares the incremental conductance dI/dV with the instantaneous
/// conductance -I/V to determine if the operating point is at, below,
/// or above the maximum power point.
///
/// At MPP: dI/dV = -I/V (slope of P-V curve is zero).
///
/// # Arguments
///
/// * `v` - Current voltage in volts (must be > 0).
/// * `i` - Current current in amps.
/// * `dv` - Voltage change since last sample.
/// * `di` - Current change since last sample.
///
/// # Returns
///
/// The [`OperatingPoint`] classification.
pub fn incremental_conductance(v: f64, i: f64, dv: f64, di: f64) -> OperatingPoint {
    if v.abs() < 1e-12 {
        return OperatingPoint::BelowMpp;
    }

    if dv.abs() < 1e-12 {
        // No voltage change; use current change to decide
        if di.abs() < CONDUCTANCE_TOLERANCE {
            return OperatingPoint::AtMpp;
        } else if di > 0.0 {
            return OperatingPoint::BelowMpp;
        } else {
            return OperatingPoint::AboveMpp;
        }
    }

    let inc_conductance = di / dv;
    let inst_conductance = -i / v;
    let diff = inc_conductance - inst_conductance;

    if diff.abs() < CONDUCTANCE_TOLERANCE {
        OperatingPoint::AtMpp
    } else if diff > 0.0 {
        OperatingPoint::BelowMpp
    } else {
        OperatingPoint::AboveMpp
    }
}

/// Single-diode solar cell/module I-V model.
///
/// Computes the output current for a given voltage using the simplified
/// single-diode equation with module-level parameters. The model uses a
/// normalized voltage ratio to handle multi-cell modules correctly:
///
/// ```text
/// I = I_sc * (1 - C1 * (exp(V / (C2 * V_oc)) - 1))
/// ```
///
/// where `C1 = (1 - I_mpp/I_sc) * exp(-V_mpp/(C2*V_oc))` and
/// `C2 = (V_mpp/V_oc - 1) / ln(1 - I_mpp/I_sc)`.
///
/// This simplified form uses the ideality factor `n` and temperature `temp_k`
/// to shape the curve via the thermal voltage ratio.
///
/// # Arguments
///
/// * `voltage` - Module voltage in volts.
/// * `i_sc` - Short-circuit current in amps.
/// * `v_oc` - Open-circuit voltage in volts.
/// * `n` - Diode ideality factor (typically 1.0 - 2.0). Controls the
///   sharpness of the knee in the I-V curve.
/// * `temp_k` - Cell temperature in Kelvin. Higher temperatures produce
///   a more gradual curve.
///
/// # Returns
///
/// The module current in amps. Returns 0.0 if voltage >= V_oc.
pub fn solar_cell_iv(voltage: f64, i_sc: f64, v_oc: f64, n: f64, temp_k: f64) -> f64 {
    if voltage <= 0.0 {
        return i_sc;
    }
    if voltage >= v_oc {
        return 0.0;
    }
    if v_oc <= 0.0 || i_sc <= 0.0 {
        return 0.0;
    }

    // Temperature-dependent shape factor: higher temperature -> more gradual knee
    let v_t = thermal_voltage(temp_k);
    let v_t_stc = thermal_voltage(STC_TEMP_K);
    let temp_ratio = v_t / v_t_stc;

    // Shape parameter 'a' controls the knee sharpness of the I-V curve.
    // For crystalline silicon modules, typical values of a = 10..30 produce
    // realistic fill factors (0.70 - 0.85). The ideality factor n (1.0-2.0)
    // inversely scales a: lower n -> sharper knee -> higher fill factor.
    // Temperature increases soften the knee (lower a -> lower FF).
    //
    // Base shape: a_base ~ Q*Voc / (n * Ns * k * T) for a real module, but
    // since Voc here is already the module-level voltage, we use:
    let a = 15.0 / (n * temp_ratio);

    // Normalized voltage
    let v_norm = voltage / v_oc;

    // Module-level single-diode I-V:
    //   I = Isc * (1 - (exp(a * v_norm) - 1) / (exp(a) - 1))
    //
    // Boundary conditions:
    //   I(0)   = Isc * (1 - 0/(exp(a)-1)) = Isc
    //   I(Voc) = Isc * (1 - (exp(a)-1)/(exp(a)-1)) = 0
    let exp_a_v = (a * v_norm).min(700.0).exp();
    let exp_a = a.min(700.0).exp();
    let denom = exp_a - 1.0;

    if denom.abs() < 1e-30 {
        // a is very small; linear approximation
        return i_sc * (1.0 - v_norm);
    }

    let current = i_sc * (1.0 - (exp_a_v - 1.0) / denom);
    current.max(0.0)
}

/// Apply temperature derating to STC-rated power.
///
/// PV module power decreases with temperature above STC (25 C).
/// Typical temperature coefficient for crystalline silicon is -0.4%/C.
///
/// # Arguments
///
/// * `power_stc` - Power at Standard Test Conditions in watts.
/// * `temp_cell_c` - Actual cell temperature in degrees Celsius.
/// * `temp_coeff_pct_per_c` - Temperature coefficient in %/C (negative value, e.g., -0.4).
///
/// # Returns
///
/// Derated power in watts.
pub fn temperature_derating(power_stc: f64, temp_cell_c: f64, temp_coeff_pct_per_c: f64) -> f64 {
    let delta_t = temp_cell_c - 25.0; // STC is 25 C
    let derating_factor = 1.0 + (temp_coeff_pct_per_c / 100.0) * delta_t;
    (power_stc * derating_factor).max(0.0)
}

/// Compute the fill factor of a PV cell or module.
///
/// Fill Factor (FF) is the ratio of the maximum obtainable power to the
/// product of the open-circuit voltage and short-circuit current:
///
/// ```text
/// FF = (V_mpp * I_mpp) / (V_oc * I_sc)
/// ```
///
/// A higher fill factor indicates a more "square" I-V curve and better
/// cell quality. Typical values: 0.70 - 0.85 for crystalline silicon.
///
/// # Arguments
///
/// * `v_oc` - Open-circuit voltage in volts.
/// * `i_sc` - Short-circuit current in amps.
/// * `v_mpp` - Voltage at maximum power point in volts.
/// * `i_mpp` - Current at maximum power point in amps.
///
/// # Returns
///
/// Fill factor as a dimensionless ratio (0.0 to 1.0).
pub fn fill_factor(v_oc: f64, i_sc: f64, v_mpp: f64, i_mpp: f64) -> f64 {
    let denominator = v_oc * i_sc;
    if denominator.abs() < 1e-12 {
        return 0.0;
    }
    let ff = (v_mpp * i_mpp) / denominator;
    ff.clamp(0.0, 1.0)
}

/// Estimate cell temperature from ambient conditions and irradiance.
///
/// Uses the Nominal Operating Cell Temperature (NOCT) model:
///
/// ```text
/// T_cell = T_ambient + (NOCT - 20) * G / 800
/// ```
///
/// where G is irradiance in W/m^2 and NOCT is typically 45 C.
///
/// # Arguments
///
/// * `ambient_c` - Ambient temperature in degrees Celsius.
/// * `irradiance_w_m2` - Solar irradiance in W/m^2 (typically 0 - 1000).
/// * `noct_c` - Nominal Operating Cell Temperature in degrees Celsius.
///
/// # Returns
///
/// Estimated cell temperature in degrees Celsius.
pub fn estimate_cell_temp(ambient_c: f64, irradiance_w_m2: f64, noct_c: f64) -> f64 {
    ambient_c + (noct_c - 20.0) * irradiance_w_m2 / 800.0
}

/// Compute the duty cycle for a buck converter.
///
/// For an ideal buck (step-down) converter:
///
/// ```text
/// D = V_out / V_in
/// ```
///
/// # Arguments
///
/// * `v_in` - Input voltage in volts (must be > 0).
/// * `v_out` - Desired output voltage in volts.
///
/// # Returns
///
/// Duty cycle as a ratio (0.0 to 1.0). Returns 0.0 if v_in <= 0.
pub fn duty_cycle_for_voltage(v_in: f64, v_out: f64) -> f64 {
    if v_in <= 0.0 {
        return 0.0;
    }
    (v_out / v_in).clamp(0.0, 1.0)
}

/// Generate a complete I-V curve for a solar cell.
///
/// Sweeps voltage from 0 to V_oc and computes the corresponding current
/// at each point using the single-diode model.
///
/// # Arguments
///
/// * `i_sc` - Short-circuit current in amps.
/// * `v_oc` - Open-circuit voltage in volts.
/// * `n` - Diode ideality factor.
/// * `temp_k` - Cell temperature in Kelvin.
/// * `num_points` - Number of points in the curve.
///
/// # Returns
///
/// A vector of `(voltage, current)` pairs.
pub fn generate_iv_curve(
    i_sc: f64,
    v_oc: f64,
    n: f64,
    temp_k: f64,
    num_points: usize,
) -> Vec<(f64, f64)> {
    if num_points == 0 {
        return Vec::new();
    }
    if num_points == 1 {
        return vec![(0.0, i_sc)];
    }

    let mut curve = Vec::with_capacity(num_points);
    for i in 0..num_points {
        let v = v_oc * (i as f64) / ((num_points - 1) as f64);
        let current = solar_cell_iv(v, i_sc, v_oc, n, temp_k);
        curve.push((v, current));
    }
    curve
}

/// Compute the thermal voltage V_t = kT/q at a given temperature.
///
/// # Arguments
///
/// * `temp_k` - Temperature in Kelvin.
///
/// # Returns
///
/// Thermal voltage in volts.
pub fn thermal_voltage(temp_k: f64) -> f64 {
    K_BOLTZMANN * temp_k / Q_ELECTRON
}

/// Find the maximum power point on a given I-V curve.
///
/// # Arguments
///
/// * `curve` - Slice of (voltage, current) pairs.
///
/// # Returns
///
/// `(v_mpp, i_mpp, p_mpp)` - Voltage, current, and power at the MPP.
pub fn find_mpp(curve: &[(f64, f64)]) -> (f64, f64, f64) {
    let mut best_v = 0.0;
    let mut best_i = 0.0;
    let mut best_p = 0.0;

    for &(v, i) in curve {
        let p = v * i;
        if p > best_p {
            best_p = p;
            best_v = v;
            best_i = i;
        }
    }

    (best_v, best_i, best_p)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // -----------------------------------------------------------------------
    // Perturb & Observe tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_po_power_increasing_voltage_increasing() {
        // Power went up while voltage went up -> continue increasing
        let v_next = perturb_and_observe(30.0, 200.0, 31.0, 210.0, 0.5);
        assert!(
            v_next > 31.0,
            "Should increase voltage when power increases with voltage"
        );
        assert!((v_next - 31.5).abs() < EPSILON);
    }

    #[test]
    fn test_po_power_increasing_voltage_decreasing() {
        // Power went up while voltage went down -> continue decreasing
        let v_next = perturb_and_observe(31.0, 200.0, 30.0, 210.0, 0.5);
        assert!(
            v_next < 30.0,
            "Should decrease voltage when power increases with lower voltage"
        );
        assert!((v_next - 29.5).abs() < EPSILON);
    }

    #[test]
    fn test_po_power_decreasing_voltage_increasing() {
        // Power went down while voltage went up -> reverse (decrease voltage)
        let v_next = perturb_and_observe(30.0, 210.0, 31.0, 200.0, 0.5);
        assert!(
            v_next < 31.0,
            "Should reverse direction when power decreases"
        );
        assert!((v_next - 30.5).abs() < EPSILON);
    }

    #[test]
    fn test_po_power_decreasing_voltage_decreasing() {
        // Power went down while voltage went down -> reverse (increase voltage)
        let v_next = perturb_and_observe(31.0, 210.0, 30.0, 200.0, 0.5);
        assert!(
            v_next > 30.0,
            "Should reverse direction when power decreases"
        );
        assert!((v_next - 30.5).abs() < EPSILON);
    }

    #[test]
    fn test_po_no_power_change() {
        // No power change -> hold voltage
        let v_next = perturb_and_observe(30.0, 200.0, 31.0, 200.0, 0.5);
        assert!(
            (v_next - 31.0).abs() < EPSILON,
            "Should hold voltage when power unchanged"
        );
    }

    #[test]
    fn test_po_various_step_sizes() {
        let v1 = perturb_and_observe(30.0, 200.0, 31.0, 210.0, 1.0);
        assert!((v1 - 32.0).abs() < EPSILON);

        let v2 = perturb_and_observe(30.0, 200.0, 31.0, 210.0, 0.1);
        assert!((v2 - 31.1).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Incremental Conductance tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ic_at_mpp() {
        // At MPP: dI/dV = -I/V
        // If V=30, I=9, then -I/V = -0.3
        // Need dI/dV = -0.3, so with dV=1, dI=-0.3
        let op = incremental_conductance(30.0, 9.0, 1.0, -0.3);
        assert_eq!(op, OperatingPoint::AtMpp);
    }

    #[test]
    fn test_ic_below_mpp() {
        // Below MPP: dI/dV > -I/V (inc. conductance > instantaneous)
        // V=20, I=9, -I/V = -0.45
        // dI/dV = -0.1 (which is > -0.45)
        let op = incremental_conductance(20.0, 9.0, 1.0, -0.1);
        assert_eq!(op, OperatingPoint::BelowMpp);
    }

    #[test]
    fn test_ic_above_mpp() {
        // Above MPP: dI/dV < -I/V
        // V=35, I=5, -I/V = -0.1429
        // dI/dV = -0.5 (which is < -0.1429)
        let op = incremental_conductance(35.0, 5.0, 1.0, -0.5);
        assert_eq!(op, OperatingPoint::AboveMpp);
    }

    #[test]
    fn test_ic_zero_voltage() {
        let op = incremental_conductance(0.0, 9.0, 1.0, -0.3);
        assert_eq!(op, OperatingPoint::BelowMpp);
    }

    #[test]
    fn test_ic_zero_dv_positive_di() {
        let op = incremental_conductance(30.0, 9.0, 0.0, 0.5);
        assert_eq!(op, OperatingPoint::BelowMpp);
    }

    #[test]
    fn test_ic_zero_dv_negative_di() {
        let op = incremental_conductance(30.0, 9.0, 0.0, -0.5);
        assert_eq!(op, OperatingPoint::AboveMpp);
    }

    #[test]
    fn test_ic_zero_dv_zero_di() {
        let op = incremental_conductance(30.0, 9.0, 0.0, 0.0);
        assert_eq!(op, OperatingPoint::AtMpp);
    }

    // -----------------------------------------------------------------------
    // Solar cell I-V model tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_iv_at_zero_voltage() {
        let i = solar_cell_iv(0.0, 10.0, 40.0, 1.3, STC_TEMP_K);
        assert!(
            (i - 10.0).abs() < EPSILON,
            "Current at V=0 should equal I_sc"
        );
    }

    #[test]
    fn test_iv_at_voc() {
        let i = solar_cell_iv(40.0, 10.0, 40.0, 1.3, STC_TEMP_K);
        assert!(i.abs() < EPSILON, "Current at V_oc should be zero");
    }

    #[test]
    fn test_iv_midpoint() {
        let i = solar_cell_iv(20.0, 10.0, 40.0, 1.3, STC_TEMP_K);
        assert!(
            i > 0.0 && i < 10.0,
            "Current at V_oc/2 should be between 0 and I_sc, got {}",
            i
        );
    }

    #[test]
    fn test_iv_negative_voltage() {
        let i = solar_cell_iv(-5.0, 10.0, 40.0, 1.3, STC_TEMP_K);
        assert!(
            (i - 10.0).abs() < EPSILON,
            "Current at negative voltage should equal I_sc"
        );
    }

    #[test]
    fn test_iv_beyond_voc() {
        let i = solar_cell_iv(45.0, 10.0, 40.0, 1.3, STC_TEMP_K);
        assert!(i.abs() < EPSILON, "Current beyond V_oc should be zero");
    }

    #[test]
    fn test_iv_temperature_dependence() {
        let i_cold = solar_cell_iv(20.0, 10.0, 40.0, 1.3, 273.15); // 0 C
        let i_hot = solar_cell_iv(20.0, 10.0, 40.0, 1.3, 348.15); // 75 C
        // At higher temperature, the exponential curve is more gradual,
        // so current at the same voltage should differ
        assert!(
            (i_cold - i_hot).abs() > 1e-3,
            "Temperature should affect I-V curve"
        );
    }

    // -----------------------------------------------------------------------
    // Fill factor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fill_factor_typical() {
        let ff = fill_factor(40.0, 10.0, 32.0, 9.2);
        let expected = (32.0 * 9.2) / (40.0 * 10.0);
        assert!(
            (ff - expected).abs() < EPSILON,
            "Fill factor should be {}, got {}",
            expected,
            ff
        );
        assert!(ff > 0.7 && ff < 0.8, "Typical FF should be 0.7-0.8");
    }

    #[test]
    fn test_fill_factor_perfect() {
        // Ideal square I-V curve: V_mpp = V_oc, I_mpp = I_sc
        let ff = fill_factor(40.0, 10.0, 40.0, 10.0);
        assert!((ff - 1.0).abs() < EPSILON, "Perfect FF should be 1.0");
    }

    #[test]
    fn test_fill_factor_zero_voc() {
        let ff = fill_factor(0.0, 10.0, 0.0, 0.0);
        assert!(ff.abs() < EPSILON, "FF with zero V_oc should be 0.0");
    }

    #[test]
    fn test_fill_factor_clamping() {
        // Even if input values are out of range, FF is clamped to [0, 1]
        let ff = fill_factor(10.0, 1.0, 5.0, 0.5);
        assert!(ff >= 0.0 && ff <= 1.0);
    }

    // -----------------------------------------------------------------------
    // Temperature derating tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_derating_at_stc() {
        let p = temperature_derating(300.0, 25.0, -0.4);
        assert!(
            (p - 300.0).abs() < EPSILON,
            "No derating at STC (25 C)"
        );
    }

    #[test]
    fn test_derating_above_stc() {
        // At 50 C with -0.4%/C: derating = 1 + (-0.4/100) * 25 = 0.90
        let p = temperature_derating(300.0, 50.0, -0.4);
        let expected = 300.0 * 0.90;
        assert!(
            (p - expected).abs() < 0.01,
            "Expected {} W at 50 C, got {} W",
            expected,
            p
        );
    }

    #[test]
    fn test_derating_below_stc() {
        // At 0 C with -0.4%/C: derating = 1 + (-0.4/100) * (-25) = 1.10
        let p = temperature_derating(300.0, 0.0, -0.4);
        let expected = 300.0 * 1.10;
        assert!(
            (p - expected).abs() < 0.01,
            "Expected {} W at 0 C, got {} W",
            expected,
            p
        );
    }

    #[test]
    fn test_derating_extreme_heat() {
        // Very high temperature should not produce negative power
        let p = temperature_derating(300.0, 300.0, -0.4);
        assert!(p >= 0.0, "Power should never be negative");
    }

    // -----------------------------------------------------------------------
    // Cell temperature estimation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_cell_temp_no_irradiance() {
        let t = estimate_cell_temp(25.0, 0.0, 45.0);
        assert!(
            (t - 25.0).abs() < EPSILON,
            "Cell temp should equal ambient with no irradiance"
        );
    }

    #[test]
    fn test_cell_temp_stc_irradiance() {
        // At 1000 W/m^2, NOCT=45: T_cell = 25 + (45-20) * 1000/800 = 56.25
        let t = estimate_cell_temp(25.0, 1000.0, 45.0);
        let expected = 25.0 + 25.0 * 1000.0 / 800.0;
        assert!(
            (t - expected).abs() < 0.01,
            "Expected {} C, got {} C",
            expected,
            t
        );
    }

    #[test]
    fn test_cell_temp_noct_conditions() {
        // At NOCT conditions: 800 W/m^2, 20 C ambient, T_cell should be NOCT
        let t = estimate_cell_temp(20.0, 800.0, 45.0);
        assert!(
            (t - 45.0).abs() < 0.01,
            "At NOCT conditions, T_cell should equal NOCT (45 C), got {} C",
            t
        );
    }

    // -----------------------------------------------------------------------
    // Duty cycle tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_duty_cycle_normal() {
        let d = duty_cycle_for_voltage(40.0, 24.0);
        assert!(
            (d - 0.6).abs() < EPSILON,
            "D = 24/40 = 0.6, got {}",
            d
        );
    }

    #[test]
    fn test_duty_cycle_unity() {
        let d = duty_cycle_for_voltage(24.0, 24.0);
        assert!((d - 1.0).abs() < EPSILON, "D should be 1.0 for V_in = V_out");
    }

    #[test]
    fn test_duty_cycle_clamped_high() {
        let d = duty_cycle_for_voltage(10.0, 30.0);
        assert!(
            (d - 1.0).abs() < EPSILON,
            "D should be clamped to 1.0 for V_out > V_in"
        );
    }

    #[test]
    fn test_duty_cycle_zero_input() {
        let d = duty_cycle_for_voltage(0.0, 24.0);
        assert!(d.abs() < EPSILON, "D should be 0.0 for zero V_in");
    }

    #[test]
    fn test_duty_cycle_negative_input() {
        let d = duty_cycle_for_voltage(-5.0, 24.0);
        assert!(d.abs() < EPSILON, "D should be 0.0 for negative V_in");
    }

    // -----------------------------------------------------------------------
    // I-V curve generation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_iv_curve_length() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 100);
        assert_eq!(curve.len(), 100);
    }

    #[test]
    fn test_iv_curve_endpoints() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 50);
        let (v_first, i_first) = curve[0];
        let (v_last, i_last) = curve[curve.len() - 1];

        assert!(v_first.abs() < EPSILON, "First voltage should be 0");
        assert!(
            (i_first - 10.0).abs() < EPSILON,
            "First current should be I_sc"
        );
        assert!(
            (v_last - 40.0).abs() < EPSILON,
            "Last voltage should be V_oc"
        );
        assert!(i_last.abs() < EPSILON, "Last current should be ~0");
    }

    #[test]
    fn test_iv_curve_monotonic_voltage() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 50);
        for i in 1..curve.len() {
            assert!(
                curve[i].0 >= curve[i - 1].0,
                "Voltage should be monotonically increasing"
            );
        }
    }

    #[test]
    fn test_iv_curve_monotonic_current() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 100);
        for i in 1..curve.len() {
            assert!(
                curve[i].1 <= curve[i - 1].1 + 1e-10,
                "Current should be monotonically decreasing: I[{}]={} > I[{}]={}",
                i,
                curve[i].1,
                i - 1,
                curve[i - 1].1,
            );
        }
    }

    #[test]
    fn test_iv_curve_empty() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 0);
        assert!(curve.is_empty());
    }

    #[test]
    fn test_iv_curve_single_point() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 1);
        assert_eq!(curve.len(), 1);
        assert!((curve[0].0).abs() < EPSILON);
        assert!((curve[0].1 - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_iv_curve_power_has_maximum() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 200);
        let powers: Vec<f64> = curve.iter().map(|&(v, i)| v * i).collect();
        // Power at endpoints should be near zero
        assert!(powers[0].abs() < EPSILON, "Power at V=0 should be ~0");
        assert!(
            powers[powers.len() - 1].abs() < EPSILON,
            "Power at V_oc should be ~0"
        );
        // Maximum power should be in the middle somewhere
        let max_p = powers.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_p > 100.0, "Max power should be substantial, got {}", max_p);
    }

    // -----------------------------------------------------------------------
    // MPPT Controller tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_controller_creation() {
        let config = MpptConfig::default();
        let controller = MpptController::new(config);
        assert!(controller.v_ref > 0.0, "Initial reference should be positive");
    }

    #[test]
    fn test_controller_first_update() {
        let config = MpptConfig::default();
        let mut controller = MpptController::new(config);
        let output = controller.update(30.0, 9.0);
        assert!(
            (output.power_w - 270.0).abs() < EPSILON,
            "Power should be V*I = 270 W"
        );
        assert!(output.reference_voltage > 0.0);
    }

    #[test]
    fn test_controller_po_convergence() {
        let config = MpptConfig {
            v_oc_nominal: 40.0,
            i_sc_nominal: 10.0,
            num_cells_series: 60,
            step_size_v: 0.5,
            method: MpptMethod::PerturbAndObserve,
        };
        let mut controller = MpptController::new(config);

        // Generate a realistic I-V curve
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 1000);
        let (v_mpp, _i_mpp, p_mpp) = find_mpp(&curve);

        // Simulate P&O tracking over several iterations
        let mut v_op = 20.0; // start far from MPP
        let mut last_power = 0.0;
        for _ in 0..100 {
            let i_op = solar_cell_iv(v_op, 10.0, 40.0, 1.3, STC_TEMP_K);
            let output = controller.update(v_op, i_op);
            v_op = output.reference_voltage;
            last_power = output.power_w;
        }

        // Should converge near the MPP (within a few step sizes)
        assert!(
            (v_op - v_mpp).abs() < 5.0,
            "P&O should converge near MPP voltage: v_op={}, v_mpp={}",
            v_op,
            v_mpp
        );
    }

    #[test]
    fn test_controller_ic_method() {
        let config = MpptConfig {
            method: MpptMethod::IncrementalConductance,
            step_size_v: 0.3,
            ..MpptConfig::default()
        };
        let mut controller = MpptController::new(config);

        // Run a few updates
        let v_start = 25.0;
        let i1 = solar_cell_iv(v_start, 10.0, 40.0, 1.3, STC_TEMP_K);
        let output1 = controller.update(v_start, i1);
        assert!(output1.power_w > 0.0);

        let v2 = output1.reference_voltage;
        let i2 = solar_cell_iv(v2, 10.0, 40.0, 1.3, STC_TEMP_K);
        let output2 = controller.update(v2, i2);
        assert!(output2.reference_voltage > 0.0);
    }

    #[test]
    fn test_controller_fractional_voc() {
        let config = MpptConfig {
            v_oc_nominal: 40.0,
            method: MpptMethod::FractionalVoc,
            ..MpptConfig::default()
        };
        let mut controller = MpptController::new(config);

        let output = controller.update(30.0, 9.0);
        // FractionalVoc should set reference to ~76% of V_oc
        let expected_vref = 40.0 * FRAC_VOC_COEFFICIENT;

        // After the first update, the second should compute the reference
        let output2 = controller.update(30.0, 9.0);
        assert!(
            (output2.reference_voltage - expected_vref).abs() < EPSILON,
            "Fractional V_oc reference should be {}, got {}",
            expected_vref,
            output2.reference_voltage
        );
    }

    #[test]
    fn test_controller_constant_voltage() {
        let config = MpptConfig {
            v_oc_nominal: 40.0,
            method: MpptMethod::ConstantVoltage,
            ..MpptConfig::default()
        };
        let mut controller = MpptController::new(config);
        let initial_ref = controller.v_ref;

        let _ = controller.update(30.0, 9.0);
        let output = controller.update(32.0, 8.5);
        assert!(
            (output.reference_voltage - initial_ref).abs() < EPSILON,
            "Constant voltage should maintain initial reference"
        );
    }

    #[test]
    fn test_controller_output_efficiency() {
        let config = MpptConfig::default();
        let mut controller = MpptController::new(config);
        let output = controller.update(32.0, 9.0);
        assert!(
            output.efficiency_percent >= 0.0 && output.efficiency_percent <= 100.0,
            "Efficiency should be 0-100%, got {}",
            output.efficiency_percent
        );
    }

    #[test]
    fn test_controller_output_duty_cycle() {
        let config = MpptConfig::default();
        let mut controller = MpptController::new(config);
        let output = controller.update(35.0, 8.0);
        assert!(
            output.duty_cycle >= 0.0 && output.duty_cycle <= 1.0,
            "Duty cycle should be 0-1, got {}",
            output.duty_cycle
        );
    }

    // -----------------------------------------------------------------------
    // Thermal voltage test
    // -----------------------------------------------------------------------

    #[test]
    fn test_thermal_voltage_stc() {
        let vt = thermal_voltage(STC_TEMP_K);
        // Expected: 1.38e-23 * 298.15 / 1.602e-19 ~ 0.02569 V
        assert!(
            (vt - 0.02569).abs() < 0.001,
            "V_t at STC should be ~25.7 mV, got {} V",
            vt
        );
    }

    // -----------------------------------------------------------------------
    // find_mpp tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_find_mpp_on_curve() {
        let curve = generate_iv_curve(10.0, 40.0, 1.3, STC_TEMP_K, 500);
        let (v_mpp, i_mpp, p_mpp) = find_mpp(&curve);

        assert!(v_mpp > 0.0 && v_mpp < 40.0, "V_mpp should be between 0 and V_oc");
        assert!(i_mpp > 0.0 && i_mpp < 10.0, "I_mpp should be between 0 and I_sc");
        assert!(p_mpp > 200.0, "P_mpp should be substantial");

        // Verify fill factor is reasonable
        let ff = fill_factor(40.0, 10.0, v_mpp, i_mpp);
        assert!(
            ff > 0.5 && ff < 1.0,
            "Fill factor should be reasonable, got {}",
            ff
        );
    }

    #[test]
    fn test_find_mpp_empty_curve() {
        let (v, i, p) = find_mpp(&[]);
        assert!(v.abs() < EPSILON);
        assert!(i.abs() < EPSILON);
        assert!(p.abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Edge case tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_po_zero_step() {
        // With zero step, voltage should not change
        let v = perturb_and_observe(30.0, 200.0, 31.0, 210.0, 0.0);
        assert!((v - 31.0).abs() < EPSILON);
    }

    #[test]
    fn test_derating_zero_coefficient() {
        let p = temperature_derating(300.0, 60.0, 0.0);
        assert!(
            (p - 300.0).abs() < EPSILON,
            "No derating with zero coefficient"
        );
    }

    #[test]
    fn test_controller_voltage_clamping() {
        let config = MpptConfig {
            v_oc_nominal: 40.0,
            step_size_v: 50.0, // huge step
            method: MpptMethod::PerturbAndObserve,
            ..MpptConfig::default()
        };
        let mut controller = MpptController::new(config);
        let _ = controller.update(39.0, 0.5);
        let output = controller.update(39.5, 0.3);

        assert!(
            output.reference_voltage >= 0.0 && output.reference_voltage <= 40.0,
            "Reference voltage should be clamped to [0, V_oc], got {}",
            output.reference_voltage
        );
    }

    #[test]
    fn test_default_config() {
        let config = MpptConfig::default();
        assert!((config.v_oc_nominal - 40.0).abs() < EPSILON);
        assert!((config.i_sc_nominal - 10.0).abs() < EPSILON);
        assert_eq!(config.num_cells_series, 60);
        assert!((config.step_size_v - 0.5).abs() < EPSILON);
        assert_eq!(config.method, MpptMethod::PerturbAndObserve);
    }

    #[test]
    fn test_default_method() {
        let method = MpptMethod::default();
        assert_eq!(method, MpptMethod::PerturbAndObserve);
    }
}

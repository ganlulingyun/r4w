//! # Cryogenic Thermometry Processor
//!
//! Signal processing for ultra-low temperature measurement in cryogenic experiments
//! such as dilution refrigerators, superconducting devices, and quantum computing systems.
//!
//! ## Sensor Types
//!
//! - **Cernox**: Zirconium oxy-nitride thin film, useful from 100 mK to 420 K
//! - **RuO2**: Ruthenium oxide thick film, excellent below 1 K
//! - **Germanium**: Doped germanium, gold standard for 0.05–100 K
//! - **Silicon Diode**: Forward-voltage thermometer, 1.4–475 K
//! - **Thermocouple**: Type E, K, T etc., broad range but low sensitivity at cryogenic temps
//! - **Platinum**: Pt-100/Pt-1000 RTD, IEC 60751, 14–873 K
//!
//! ## Challenges at Millikelvin Temperatures
//!
//! - **Self-heating**: Excitation current dissipates power P = I²R in the sensor,
//!   raising its temperature above the environment. Must use nanoamp-level excitation.
//! - **Thermal contact**: Kapitza resistance at interfaces grows as T⁻³, making
//!   thermal anchoring critical below 1 K.
//! - **Johnson noise**: Thermal voltage noise V_rms = sqrt(4kTRB) sets a fundamental
//!   measurement floor.
//! - **Signal conditioning**: Lock-in amplifiers and AC bridges are used to extract
//!   the tiny resistance signal from noise.
//!
//! ## Calibration
//!
//! Resistance thermometers require individual calibration curves, typically expressed
//! as Chebyshev polynomial expansions of log(T) vs log(R) over defined ranges.
//! The Chebyshev representation provides uniform approximation error and numerical
//! stability compared to power-series polynomials.

use std::f64::consts::PI;

/// Boltzmann constant in J/K
const K_BOLTZMANN: f64 = 1.380649e-23;

/// Type of cryogenic temperature sensor.
#[derive(Debug, Clone, PartialEq)]
pub enum SensorType {
    /// Zirconium oxy-nitride thin film (Lake Shore), 100 mK – 420 K
    Cernox,
    /// Ruthenium oxide thick film, excellent below 1 K
    RuO2,
    /// Doped germanium, 50 mK – 100 K
    Germanium,
    /// Forward-voltage silicon diode, 1.4 – 475 K
    SiliconDiode,
    /// Thermocouple with IEC type code (E, K, T, etc.)
    Thermocouple {
        /// IEC 60584 type code character
        type_code: char,
    },
    /// Platinum RTD (Pt-100, Pt-1000), 14 – 873 K
    Platinum,
}

/// Chebyshev polynomial calibration curve for a resistance thermometer.
///
/// The calibration maps sensor resistance to temperature (and vice versa)
/// using a Chebyshev polynomial expansion. The variable is normalized from
/// the physical range to [-1, 1] before evaluation.
///
/// Typical usage: coefficients are fit to log(R) vs log(T) data from a
/// calibration laboratory, with the ranges specifying the valid domain.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    /// Chebyshev polynomial coefficients [a0, a1, a2, ...] where
    /// T(x) = a0*T0(x) + a1*T1(x) + a2*T2(x) + ...
    pub coefficients: Vec<f64>,
    /// Valid temperature range (min_K, max_K)
    pub temp_range: (f64, f64),
    /// Valid resistance range (min_ohm, max_ohm)
    pub resistance_range: (f64, f64),
}

/// Configuration for a cryogenic thermometry measurement channel.
#[derive(Debug, Clone)]
pub struct ThermometryConfig {
    /// Type of sensor on this channel
    pub sensor_type: SensorType,
    /// Excitation current in microamperes (typical: 0.001 – 10 uA)
    pub excitation_current_ua: f64,
    /// ADC/readout sample rate in Hz
    pub sample_rate_hz: f64,
    /// Integration / averaging time in seconds
    pub averaging_time_s: f64,
    /// Calibration curve for resistance-to-temperature conversion
    pub calibration: CalibrationCurve,
}

/// Cryogenic thermometer processor for resistance-to-temperature conversion,
/// self-heating correction, and noise analysis.
#[derive(Debug)]
pub struct CryogenicThermometer {
    config: ThermometryConfig,
}

impl CryogenicThermometer {
    /// Create a new cryogenic thermometer processor with the given configuration.
    pub fn new(config: ThermometryConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the current configuration.
    pub fn config(&self) -> &ThermometryConfig {
        &self.config
    }

    /// Convert sensor resistance (ohms) to temperature (K) using the calibration curve.
    ///
    /// Uses Chebyshev polynomial interpolation. The resistance value is first
    /// normalized to the [-1, 1] interval over the calibration resistance range,
    /// then the Chebyshev expansion is evaluated.
    ///
    /// # Arguments
    /// * `resistance_ohm` - Measured sensor resistance in ohms
    /// * `cal` - Calibration curve to use
    ///
    /// # Returns
    /// Temperature in Kelvin
    pub fn resistance_to_temperature(resistance_ohm: f64, cal: &CalibrationCurve) -> f64 {
        let x = Self::normalize_to_chebyshev(resistance_ohm, cal.resistance_range);
        Self::chebyshev_eval(x, &cal.coefficients)
    }

    /// Convert temperature (K) to sensor resistance (ohms) using inverse lookup.
    ///
    /// Performs a bisection search over the resistance range to find the resistance
    /// value that maps to the target temperature within a tight tolerance.
    ///
    /// # Arguments
    /// * `temp_k` - Target temperature in Kelvin
    /// * `cal` - Calibration curve to use
    ///
    /// # Returns
    /// Estimated sensor resistance in ohms
    pub fn temperature_to_resistance(temp_k: f64, cal: &CalibrationCurve) -> f64 {
        // Bisection search over resistance range
        let mut lo = cal.resistance_range.0;
        let mut hi = cal.resistance_range.1;

        // Determine if calibration is monotonically increasing or decreasing
        let t_lo = Self::resistance_to_temperature(lo, cal);
        let t_hi = Self::resistance_to_temperature(hi, cal);
        let increasing = t_hi > t_lo;

        for _ in 0..100 {
            let mid = 0.5 * (lo + hi);
            let t_mid = Self::resistance_to_temperature(mid, cal);

            if (t_mid - temp_k).abs() < 1e-12 {
                return mid;
            }

            if (t_mid < temp_k) == increasing {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        0.5 * (lo + hi)
    }

    /// Evaluate a Chebyshev polynomial expansion at a normalized point.
    ///
    /// Uses the Clenshaw recurrence algorithm for stable and efficient evaluation:
    ///   T(x) = sum_{k=0}^{N-1} c_k * T_k(x)
    ///
    /// where T_k(x) are Chebyshev polynomials of the first kind.
    ///
    /// # Arguments
    /// * `x_normalized` - Point in [-1, 1] at which to evaluate
    /// * `coefficients` - Chebyshev coefficients [c0, c1, c2, ...]
    ///
    /// # Returns
    /// Value of the Chebyshev expansion
    pub fn chebyshev_eval(x_normalized: f64, coefficients: &[f64]) -> f64 {
        if coefficients.is_empty() {
            return 0.0;
        }
        if coefficients.len() == 1 {
            return coefficients[0];
        }

        // Clenshaw recurrence: evaluate sum c_k * T_k(x)
        let n = coefficients.len();
        let mut b_k_plus1 = 0.0;
        let mut b_k_plus2 = 0.0;

        for k in (1..n).rev() {
            let b_k = 2.0 * x_normalized * b_k_plus1 - b_k_plus2 + coefficients[k];
            b_k_plus2 = b_k_plus1;
            b_k_plus1 = b_k;
        }

        // Final step: T_0(x) = 1, so c_0 + x * b_1 - b_2
        coefficients[0] + x_normalized * b_k_plus1 - b_k_plus2
    }

    /// Normalize a physical value to the Chebyshev interval [-1, 1].
    ///
    /// Maps `value` from [range.0, range.1] to [-1, 1] linearly:
    ///   x_norm = 2 * (value - min) / (max - min) - 1
    ///
    /// # Arguments
    /// * `value` - Physical value to normalize
    /// * `range` - (min, max) of the physical domain
    ///
    /// # Returns
    /// Normalized value in [-1, 1]
    pub fn normalize_to_chebyshev(value: f64, range: (f64, f64)) -> f64 {
        let (min, max) = range;
        if (max - min).abs() < f64::EPSILON {
            return 0.0;
        }
        2.0 * (value - min) / (max - min) - 1.0
    }

    /// Correct measured temperature for sensor self-heating.
    ///
    /// Self-heating occurs because the excitation current dissipates power P = I²R
    /// in the sensor, raising it above the true environment temperature:
    ///   T_true = T_measured - P / G
    ///
    /// where G is the thermal conductance between the sensor and the thermal bath.
    ///
    /// # Arguments
    /// * `measured_temp` - Measured temperature in Kelvin (includes self-heating)
    /// * `excitation_power_w` - Electrical power dissipated in the sensor (watts)
    /// * `thermal_conductance_w_per_k` - Thermal link conductance (W/K)
    ///
    /// # Returns
    /// Corrected (true) temperature in Kelvin
    pub fn self_heating_correction(
        measured_temp: f64,
        excitation_power_w: f64,
        thermal_conductance_w_per_k: f64,
    ) -> f64 {
        if thermal_conductance_w_per_k <= 0.0 {
            return measured_temp;
        }
        measured_temp - excitation_power_w / thermal_conductance_w_per_k
    }

    /// Calculate the RMS Johnson (thermal) noise voltage across a resistor.
    ///
    /// Johnson-Nyquist noise is the fundamental thermal noise in a resistor:
    ///   V_rms = sqrt(4 * k_B * T * R * B)
    ///
    /// where k_B is Boltzmann's constant, T is temperature, R is resistance,
    /// and B is the measurement bandwidth.
    ///
    /// # Arguments
    /// * `resistance_ohm` - Resistance in ohms
    /// * `temperature_k` - Temperature in Kelvin
    /// * `bandwidth_hz` - Measurement bandwidth in Hz
    ///
    /// # Returns
    /// RMS noise voltage in volts
    pub fn johnson_noise_voltage(
        resistance_ohm: f64,
        temperature_k: f64,
        bandwidth_hz: f64,
    ) -> f64 {
        if temperature_k <= 0.0 || resistance_ohm <= 0.0 || bandwidth_hz <= 0.0 {
            return 0.0;
        }
        (4.0 * K_BOLTZMANN * temperature_k * resistance_ohm * bandwidth_hz).sqrt()
    }
}

/// Signal conditioning routines for cryogenic measurement systems.
///
/// Includes lock-in amplifier detection, AC bridge balancing, and various
/// filtering and noise analysis methods commonly used in low-temperature
/// instrumentation.
pub struct SignalConditioner;

impl SignalConditioner {
    /// Phase-sensitive (lock-in) detection of a signal against a reference.
    ///
    /// Multiplies the signal by the reference (mixing), then applies a
    /// single-pole low-pass filter with the given time constant to extract
    /// the in-phase component. This rejects noise at frequencies away from
    /// the reference frequency.
    ///
    /// # Arguments
    /// * `signal` - Input signal samples
    /// * `reference` - Reference oscillator samples (same length as signal)
    /// * `time_constant_s` - Low-pass filter time constant in seconds
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    /// Filtered (demodulated) signal
    pub fn lock_in_amplifier(
        signal: &[f64],
        reference: &[f64],
        time_constant_s: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        let n = signal.len().min(reference.len());
        if n == 0 {
            return vec![];
        }

        // Single-pole IIR coefficient
        let dt = 1.0 / sample_rate;
        let alpha = dt / (time_constant_s + dt);

        // Mix signal with reference, then low-pass filter
        let mut output = vec![0.0; n];
        let mut filtered = 0.0;

        for i in 0..n {
            let mixed = signal[i] * reference[i];
            filtered += alpha * (mixed - filtered);
            output[i] = filtered;
        }

        output
    }

    /// Calculate the imbalance voltage of a Wheatstone AC bridge.
    ///
    /// In a balanced bridge, V_out = 0. The imbalance voltage is proportional
    /// to the deviation of the sensor resistance from balance:
    ///   V_imbalance = V_excitation * (R_sensor / (R_sensor + R_ref) - ratio)
    ///
    /// where `ratio` is the target balance ratio (typically 0.5 for equal arms).
    ///
    /// # Arguments
    /// * `r_sensor` - Sensor resistance in ohms
    /// * `r_ref` - Reference arm resistance in ohms
    /// * `ratio` - Target voltage divider ratio at balance (typically 0.5)
    ///
    /// # Returns
    /// Normalized imbalance voltage (dimensionless, multiply by excitation voltage)
    pub fn ac_bridge_balance(r_sensor: f64, r_ref: f64, ratio: f64) -> f64 {
        if (r_sensor + r_ref).abs() < f64::EPSILON {
            return 0.0;
        }
        r_sensor / (r_sensor + r_ref) - ratio
    }

    /// Apply a moving-average (boxcar) filter to the input signal.
    ///
    /// # Arguments
    /// * `signal` - Input samples
    /// * `window_size` - Number of samples in the averaging window
    ///
    /// # Returns
    /// Filtered signal (same length as input, causal)
    pub fn moving_average_filter(signal: &[f64], window_size: usize) -> Vec<f64> {
        if signal.is_empty() || window_size == 0 {
            return vec![];
        }

        let n = signal.len();
        let mut output = vec![0.0; n];
        let mut sum = 0.0;
        let w = window_size;

        for i in 0..n {
            sum += signal[i];
            if i >= w {
                sum -= signal[i - w];
            }
            let count = (i + 1).min(w);
            output[i] = sum / count as f64;
        }

        output
    }

    /// Apply an exponential (IIR) smoothing filter.
    ///
    /// y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    ///
    /// Smaller alpha gives heavier smoothing (longer time constant).
    ///
    /// # Arguments
    /// * `signal` - Input samples
    /// * `alpha` - Smoothing factor in (0, 1]
    ///
    /// # Returns
    /// Filtered signal
    pub fn exponential_filter(signal: &[f64], alpha: f64) -> Vec<f64> {
        if signal.is_empty() {
            return vec![];
        }

        let alpha = alpha.clamp(0.0, 1.0);
        let mut output = vec![0.0; signal.len()];
        output[0] = signal[0];

        for i in 1..signal.len() {
            output[i] = alpha * signal[i] + (1.0 - alpha) * output[i - 1];
        }

        output
    }

    /// Compute the RMS (root-mean-square) noise level of a signal.
    ///
    /// Removes the DC component (mean) before computing RMS to measure
    /// only the AC noise power.
    ///
    /// # Arguments
    /// * `signal` - Input samples
    ///
    /// # Returns
    /// RMS noise amplitude
    pub fn noise_rms(signal: &[f64]) -> f64 {
        if signal.is_empty() {
            return 0.0;
        }

        let n = signal.len() as f64;
        let mean = signal.iter().sum::<f64>() / n;
        let variance = signal.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        variance.sqrt()
    }

    /// Compute the Allan variance for a given averaging time.
    ///
    /// The Allan variance σ²_y(τ) characterizes frequency stability and
    /// is widely used to assess thermometer stability. For white noise,
    /// Allan variance decreases as 1/τ; for flicker noise, it is constant.
    ///
    /// σ²_y(τ) = (1 / 2(M-1)) * Σ (y_{i+1} - y_i)²
    ///
    /// where y_i are non-overlapping averages of `tau_samples` consecutive samples.
    ///
    /// # Arguments
    /// * `signal` - Input time series
    /// * `tau_samples` - Number of samples per averaging bin
    ///
    /// # Returns
    /// Allan variance for the specified τ
    pub fn allan_variance(signal: &[f64], tau_samples: usize) -> f64 {
        if tau_samples == 0 || signal.len() < 2 * tau_samples {
            return 0.0;
        }

        // Compute non-overlapping bin averages
        let num_bins = signal.len() / tau_samples;
        let mut averages = Vec::with_capacity(num_bins);

        for i in 0..num_bins {
            let start = i * tau_samples;
            let end = start + tau_samples;
            let avg: f64 = signal[start..end].iter().sum::<f64>() / tau_samples as f64;
            averages.push(avg);
        }

        if averages.len() < 2 {
            return 0.0;
        }

        // Allan variance: mean of squared successive differences / 2
        let m = averages.len() - 1;
        let sum_sq_diff: f64 = averages
            .windows(2)
            .map(|w| (w[1] - w[0]).powi(2))
            .sum();

        sum_sq_diff / (2.0 * m as f64)
    }
}

/// Temperature controller for cryogenic systems.
///
/// Provides PID heater control, thermal modeling, and heat capacity calculations
/// needed for temperature regulation in dilution refrigerators and cryostats.
pub struct TemperatureController;

impl TemperatureController {
    /// Compute PID heater output for temperature regulation.
    ///
    /// Implements a standard PID controller with integral and derivative terms:
    ///   output = Kp * e + Ki * ∫e dt + Kd * de/dt
    ///
    /// Output is clamped to [0, 1] (fractional heater power).
    ///
    /// # Arguments
    /// * `setpoint_k` - Target temperature in Kelvin
    /// * `measured_k` - Current measured temperature in Kelvin
    /// * `integral` - Accumulated integral error (updated in place)
    /// * `prev_error` - Previous error for derivative (updated in place)
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `kd` - Derivative gain
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// Heater output power fraction in [0, 1]
    pub fn pid_heater_output(
        setpoint_k: f64,
        measured_k: f64,
        integral: &mut f64,
        prev_error: &mut f64,
        kp: f64,
        ki: f64,
        kd: f64,
        dt: f64,
    ) -> f64 {
        let error = setpoint_k - measured_k;
        *integral += error * dt;
        let derivative = if dt > 0.0 {
            (error - *prev_error) / dt
        } else {
            0.0
        };
        *prev_error = error;

        let output = kp * error + ki * (*integral) + kd * derivative;
        output.clamp(0.0, 1.0)
    }

    /// Calculate the thermal time constant of a body.
    ///
    /// The thermal time constant τ = C / G determines how quickly a body
    /// equilibrates with its environment, where C is heat capacity and G
    /// is thermal conductance.
    ///
    /// # Arguments
    /// * `heat_capacity_j_per_k` - Heat capacity in J/K
    /// * `thermal_conductance_w_per_k` - Thermal conductance in W/K
    ///
    /// # Returns
    /// Time constant τ in seconds
    pub fn thermal_time_constant(
        heat_capacity_j_per_k: f64,
        thermal_conductance_w_per_k: f64,
    ) -> f64 {
        if thermal_conductance_w_per_k <= 0.0 {
            return f64::INFINITY;
        }
        heat_capacity_j_per_k / thermal_conductance_w_per_k
    }

    /// Model exponential cooling toward a base temperature.
    ///
    /// T(t) = T_base + (T_initial - T_base) * exp(-t / τ)
    ///
    /// # Arguments
    /// * `initial_temp` - Starting temperature in Kelvin
    /// * `base_temp` - Asymptotic base temperature in Kelvin
    /// * `time_constant` - Thermal time constant τ in seconds
    /// * `time` - Elapsed time in seconds
    ///
    /// # Returns
    /// Temperature at time `t` in Kelvin
    pub fn cooling_curve(
        initial_temp: f64,
        base_temp: f64,
        time_constant: f64,
        time: f64,
    ) -> f64 {
        if time_constant <= 0.0 {
            return base_temp;
        }
        base_temp + (initial_temp - base_temp) * (-time / time_constant).exp()
    }

    /// Electronic heat capacity at low temperatures.
    ///
    /// In metals, C_e = γT (Sommerfeld model), where γ is the Sommerfeld
    /// coefficient in J/(mol·K²). Dominates below ~1 K.
    ///
    /// # Arguments
    /// * `temperature_k` - Temperature in Kelvin
    /// * `sommerfeld_coeff` - Sommerfeld coefficient γ in J/(mol·K²)
    ///
    /// # Returns
    /// Electronic heat capacity in J/(mol·K)
    pub fn heat_capacity_electron(temperature_k: f64, sommerfeld_coeff: f64) -> f64 {
        sommerfeld_coeff * temperature_k
    }

    /// Phonon (lattice) heat capacity at low temperatures via Debye T³ law.
    ///
    /// At T << Θ_D (Debye temperature):
    ///   C_ph = (12/5) π⁴ N k_B (T/Θ_D)³
    ///
    /// For 1 mole, N k_B = R = 8.314 J/(mol·K).
    ///
    /// # Arguments
    /// * `temperature_k` - Temperature in Kelvin
    /// * `debye_temp` - Debye temperature Θ_D in Kelvin
    ///
    /// # Returns
    /// Phonon heat capacity in J/(mol·K)
    pub fn heat_capacity_phonon(temperature_k: f64, debye_temp: f64) -> f64 {
        if debye_temp <= 0.0 {
            return 0.0;
        }
        let r = 8.314; // Gas constant J/(mol·K)
        let ratio = temperature_k / debye_temp;
        (12.0 / 5.0) * PI.powi(4) * r * ratio.powi(3)
    }
}

/// Cryostat monitoring and dilution refrigerator diagnostics.
///
/// Provides models for dilution refrigerator cooling power, still operation,
/// helium-3 vapor pressure (ITS-90), and base temperature estimation.
pub struct CryostatMonitor;

impl CryostatMonitor {
    /// Estimate the cooling power of a dilution refrigerator mixing chamber.
    ///
    /// For an ideal dilution refrigerator, the cooling power scales as T²:
    ///   Q̇ = K * ṅ₃ * T²
    ///
    /// Using typical values for a circulation rate of ~100 µmol/s:
    ///   Q̇ ≈ 84 * ṅ₃ * T² (W, with T in K, ṅ₃ in mol/s)
    ///
    /// This function uses a simplified model with T in millikelvin and
    /// returns power in microwatts.
    ///
    /// # Arguments
    /// * `temperature_mk` - Mixing chamber temperature in millikelvin
    ///
    /// # Returns
    /// Cooling power in microwatts (µW)
    pub fn mix_chamber_cooling_power(temperature_mk: f64) -> f64 {
        if temperature_mk <= 0.0 {
            return 0.0;
        }
        // Q_dot = K * T_mk^2 [uW], K ~ 0.01 uW/mK^2 for a typical fridge
        // with ~100 umol/s circulation rate
        let k = 0.01; // uW / mK^2
        k * temperature_mk * temperature_mk
    }

    /// Estimate the power load on the still of a dilution refrigerator.
    ///
    /// The still operates at ~0.6–0.7 K and drives the He-3 circulation.
    /// Power is approximately proportional to T⁴ (radiative + conductive):
    ///   P_still ∝ T⁴
    ///
    /// # Arguments
    /// * `temperature_k` - Still temperature in Kelvin
    ///
    /// # Returns
    /// Still power in milliwatts
    pub fn still_power(temperature_k: f64) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        // Typical still dissipation model: P ~ 10 * T^4 [mW]
        10.0 * temperature_k.powi(4)
    }

    /// Calculate He-3 vapor pressure using the ITS-90 scale approximation.
    ///
    /// Uses a simplified Antoine-type equation for the He-3 vapor pressure
    /// curve between 0.2 K and 3.2 K:
    ///   log10(P) = A - B / T
    ///
    /// This is a low-order approximation; real ITS-90 uses higher-order
    /// polynomial fits.
    ///
    /// # Arguments
    /// * `temperature_k` - Temperature in Kelvin
    ///
    /// # Returns
    /// Vapor pressure in Pascals (Pa)
    pub fn helium3_vapor_pressure(temperature_k: f64) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        // Antoine-like coefficients for He-3 (approximate)
        // Gives roughly correct behavior: ~0 at 0.3 K, ~100 kPa at 3.2 K
        let a = 4.80; // log10(Pa) intercept
        let b = 2.49; // K (related to latent heat / R)
        let log10_p = a - b / temperature_k;
        10.0_f64.powf(log10_p)
    }

    /// Estimate the base temperature achievable given cooling power and heat leak.
    ///
    /// In steady state, Q̇_cool(T_base) = Q̇_leak:
    ///   K * T_base² = Q_leak
    ///   T_base = sqrt(Q_leak / K)
    ///
    /// Both cooling power and heat leak should be in the same units (e.g., µW).
    ///
    /// # Arguments
    /// * `cooling_power_uw` - Available cooling power at reference T, in µW
    /// * `heat_leak_uw` - Total parasitic heat leak in µW
    ///
    /// # Returns
    /// Estimated base temperature in millikelvin
    pub fn estimated_base_temperature(cooling_power_uw: f64, heat_leak_uw: f64) -> f64 {
        if cooling_power_uw <= 0.0 || heat_leak_uw < 0.0 {
            return 0.0;
        }
        // From Q = K * T^2, with K = cooling_power_uw / T_ref^2
        // At base: K * T_base^2 = heat_leak  =>  T_base = sqrt(heat_leak / K)
        // Using a reference: if we know Q at some T, K = Q/T^2
        // For this simplified model: T_base = sqrt(heat_leak * T_ref^2 / cooling_power)
        // Use 100 mK reference point
        let t_ref_mk = 100.0; // mK reference temperature
        let k = cooling_power_uw / (t_ref_mk * t_ref_mk);
        if k <= 0.0 {
            return 0.0;
        }
        (heat_leak_uw / k).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a simple linear calibration curve T = a0 + a1 * x_norm
    /// where x_norm = normalize(R, resistance_range).
    fn make_linear_cal() -> CalibrationCurve {
        // Chebyshev: T(x) = c0*T0(x) + c1*T1(x) = c0 + c1*x
        // Map R in [100, 10000] -> T in [1, 300] approximately
        // At R=100 (x=-1): T = 150.5 + (-149.5)*(-1) = 300
        // At R=10000 (x=+1): T = 150.5 + (-149.5)*(+1) = 1
        CalibrationCurve {
            coefficients: vec![150.5, -149.5],
            temp_range: (1.0, 300.0),
            resistance_range: (100.0, 10000.0),
        }
    }

    /// Helper: create a quadratic calibration curve with Chebyshev coefficients
    fn make_quadratic_cal() -> CalibrationCurve {
        // T(x) = c0*T0(x) + c1*T1(x) + c2*T2(x) = c0 + c1*x + c2*(2x²-1)
        // c0 = 50, c1 = -30, c2 = 10
        CalibrationCurve {
            coefficients: vec![50.0, -30.0, 10.0],
            temp_range: (10.0, 90.0),
            resistance_range: (500.0, 5000.0),
        }
    }

    fn make_config() -> ThermometryConfig {
        ThermometryConfig {
            sensor_type: SensorType::Cernox,
            excitation_current_ua: 1.0,
            sample_rate_hz: 10.0,
            averaging_time_s: 1.0,
            calibration: make_linear_cal(),
        }
    }

    // ---- Chebyshev evaluation tests ----

    #[test]
    fn test_chebyshev_eval_constant() {
        // T0(x) = 1, so coefficients = [5.0] => always 5.0
        assert!((CryogenicThermometer::chebyshev_eval(0.0, &[5.0]) - 5.0).abs() < 1e-12);
        assert!((CryogenicThermometer::chebyshev_eval(0.5, &[5.0]) - 5.0).abs() < 1e-12);
        assert!((CryogenicThermometer::chebyshev_eval(-1.0, &[5.0]) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn test_chebyshev_eval_linear() {
        // T(x) = 3 + 2*x
        let c = [3.0, 2.0];
        assert!((CryogenicThermometer::chebyshev_eval(0.0, &c) - 3.0).abs() < 1e-12);
        assert!((CryogenicThermometer::chebyshev_eval(1.0, &c) - 5.0).abs() < 1e-12);
        assert!((CryogenicThermometer::chebyshev_eval(-1.0, &c) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_chebyshev_eval_quadratic() {
        // T(x) = c0 + c1*T1(x) + c2*T2(x), where T2(x) = 2x² - 1
        // c0=1, c1=0, c2=1 => T(x) = 1 + 0 + (2x²-1) = 2x²
        let c = [1.0, 0.0, 1.0];
        assert!((CryogenicThermometer::chebyshev_eval(0.0, &c) - 0.0).abs() < 1e-12);
        assert!((CryogenicThermometer::chebyshev_eval(1.0, &c) - 2.0).abs() < 1e-12);
        assert!((CryogenicThermometer::chebyshev_eval(-1.0, &c) - 2.0).abs() < 1e-12);
        assert!((CryogenicThermometer::chebyshev_eval(0.5, &c) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn test_chebyshev_eval_empty_coefficients() {
        assert_eq!(CryogenicThermometer::chebyshev_eval(0.5, &[]), 0.0);
    }

    #[test]
    fn test_chebyshev_eval_cubic() {
        // T3(x) = 4x³ - 3x
        // c0=0, c1=0, c2=0, c3=1 => T(x) = 4x³ - 3x
        let c = [0.0, 0.0, 0.0, 1.0];
        // At x=1: T3(1) = 4-3 = 1
        assert!((CryogenicThermometer::chebyshev_eval(1.0, &c) - 1.0).abs() < 1e-12);
        // At x=-1: T3(-1) = -4+3 = -1
        assert!((CryogenicThermometer::chebyshev_eval(-1.0, &c) - (-1.0)).abs() < 1e-12);
        // At x=0: T3(0) = 0
        assert!((CryogenicThermometer::chebyshev_eval(0.0, &c) - 0.0).abs() < 1e-12);
    }

    // ---- Normalization tests ----

    #[test]
    fn test_normalize_to_chebyshev_endpoints() {
        assert!((CryogenicThermometer::normalize_to_chebyshev(0.0, (0.0, 10.0)) - (-1.0)).abs() < 1e-12);
        assert!((CryogenicThermometer::normalize_to_chebyshev(10.0, (0.0, 10.0)) - 1.0).abs() < 1e-12);
        assert!((CryogenicThermometer::normalize_to_chebyshev(5.0, (0.0, 10.0)) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_normalize_to_chebyshev_degenerate_range() {
        assert_eq!(CryogenicThermometer::normalize_to_chebyshev(5.0, (5.0, 5.0)), 0.0);
    }

    // ---- Resistance to temperature roundtrip ----

    #[test]
    fn test_resistance_to_temperature_linear() {
        let cal = make_linear_cal();
        // At R=100 (x=-1): T = 150.5 - (-149.5) = 300
        let t = CryogenicThermometer::resistance_to_temperature(100.0, &cal);
        assert!((t - 300.0).abs() < 1e-6);

        // At R=10000 (x=+1): T = 150.5 - 149.5 = 1
        let t = CryogenicThermometer::resistance_to_temperature(10000.0, &cal);
        assert!((t - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_roundtrip_resistance_temperature_linear() {
        let cal = make_linear_cal();
        let original_temp = 150.0;
        let r = CryogenicThermometer::temperature_to_resistance(original_temp, &cal);
        let t_back = CryogenicThermometer::resistance_to_temperature(r, &cal);
        assert!(
            (t_back - original_temp).abs() < 1e-6,
            "Roundtrip failed: {} -> R={} -> {}",
            original_temp,
            r,
            t_back
        );
    }

    #[test]
    fn test_roundtrip_resistance_temperature_quadratic() {
        let cal = make_quadratic_cal();
        // Pick a temperature in range
        // At x=0 (R=2750): T = 50 + 0 + 10*(0-1) = 40
        let original_temp = 40.0;
        let r = CryogenicThermometer::temperature_to_resistance(original_temp, &cal);
        let t_back = CryogenicThermometer::resistance_to_temperature(r, &cal);
        assert!(
            (t_back - original_temp).abs() < 1e-4,
            "Roundtrip failed: {} -> R={} -> {}",
            original_temp,
            r,
            t_back
        );
    }

    // ---- Self-heating correction ----

    #[test]
    fn test_self_heating_correction_reduces_temperature() {
        let measured = 4.2;
        let power = 1e-6; // 1 µW
        let conductance = 1e-3; // 1 mW/K
        let corrected =
            CryogenicThermometer::self_heating_correction(measured, power, conductance);
        assert!(corrected < measured, "Corrected temp should be lower than measured");
        assert!((corrected - (4.2 - 0.001)).abs() < 1e-9);
    }

    #[test]
    fn test_self_heating_zero_power() {
        let measured = 0.1;
        let corrected = CryogenicThermometer::self_heating_correction(measured, 0.0, 1e-3);
        assert!((corrected - measured).abs() < 1e-12);
    }

    #[test]
    fn test_self_heating_zero_conductance() {
        // With zero conductance, should return measured (no correction possible)
        let measured = 0.3;
        let corrected = CryogenicThermometer::self_heating_correction(measured, 1e-6, 0.0);
        assert!((corrected - measured).abs() < 1e-12);
    }

    // ---- Johnson noise ----

    #[test]
    fn test_johnson_noise_increases_with_temperature() {
        let v1 = CryogenicThermometer::johnson_noise_voltage(1000.0, 4.2, 100.0);
        let v2 = CryogenicThermometer::johnson_noise_voltage(1000.0, 300.0, 100.0);
        assert!(v2 > v1, "Noise should increase with temperature");
    }

    #[test]
    fn test_johnson_noise_zero_at_zero_k() {
        let v = CryogenicThermometer::johnson_noise_voltage(1000.0, 0.0, 100.0);
        assert_eq!(v, 0.0, "Johnson noise must be zero at 0 K");
    }

    #[test]
    fn test_johnson_noise_known_value() {
        // V_rms = sqrt(4 * 1.38e-23 * 300 * 1000 * 1e6)
        // = sqrt(4 * 1.38e-23 * 300 * 1e9)
        // = sqrt(4 * 1.38e-23 * 3e11)
        // = sqrt(1.656e-11) ≈ 4.07e-6 V
        let v = CryogenicThermometer::johnson_noise_voltage(1000.0, 300.0, 1e6);
        assert!(
            (v - 4.07e-6).abs() < 0.1e-6,
            "Expected ~4.07 µV, got {} V",
            v
        );
    }

    #[test]
    fn test_johnson_noise_increases_with_resistance() {
        let v1 = CryogenicThermometer::johnson_noise_voltage(100.0, 4.2, 100.0);
        let v2 = CryogenicThermometer::johnson_noise_voltage(10000.0, 4.2, 100.0);
        assert!(v2 > v1);
    }

    #[test]
    fn test_johnson_noise_increases_with_bandwidth() {
        let v1 = CryogenicThermometer::johnson_noise_voltage(1000.0, 4.2, 10.0);
        let v2 = CryogenicThermometer::johnson_noise_voltage(1000.0, 4.2, 1000.0);
        assert!(v2 > v1);
    }

    // ---- Lock-in amplifier ----

    #[test]
    fn test_lock_in_amplifier_recovers_in_phase_signal() {
        // Create a signal that is in-phase with the reference
        let sample_rate = 1000.0;
        let freq = 50.0;
        let n = 2000;
        let amplitude = 1.0;

        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                amplitude * (2.0 * PI * freq * t).sin()
            })
            .collect();

        let reference: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * freq * t).sin()
            })
            .collect();

        let output = SignalConditioner::lock_in_amplifier(
            &signal,
            &reference,
            0.1, // 100 ms time constant
            sample_rate,
        );

        // After settling, the output should converge to ~0.5 * amplitude
        // (because sin²(x) averages to 0.5)
        let last = output[n - 1];
        assert!(
            (last - 0.5).abs() < 0.05,
            "Lock-in output should converge to ~0.5, got {}",
            last
        );
    }

    #[test]
    fn test_lock_in_amplifier_rejects_quadrature() {
        // Signal 90° out of phase with reference => low output
        let sample_rate = 1000.0;
        let freq = 50.0;
        let n = 2000;

        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * freq * t).cos() // cosine = 90° from sine
            })
            .collect();

        let reference: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * freq * t).sin()
            })
            .collect();

        let output = SignalConditioner::lock_in_amplifier(&signal, &reference, 0.1, sample_rate);

        // sin * cos averages to 0 -> output should be near 0
        let last = output[n - 1];
        assert!(
            last.abs() < 0.05,
            "Lock-in should reject quadrature signal, got {}",
            last
        );
    }

    // ---- AC bridge balance ----

    #[test]
    fn test_ac_bridge_balanced() {
        // R_sensor = R_ref, ratio = 0.5 => balanced
        let v = SignalConditioner::ac_bridge_balance(1000.0, 1000.0, 0.5);
        assert!(v.abs() < 1e-12, "Balanced bridge should give zero imbalance");
    }

    #[test]
    fn test_ac_bridge_imbalanced() {
        // R_sensor > R_ref => positive imbalance
        let v = SignalConditioner::ac_bridge_balance(2000.0, 1000.0, 0.5);
        assert!(v > 0.0);
    }

    // ---- Moving average filter ----

    #[test]
    fn test_moving_average_constant_signal() {
        let signal = vec![5.0; 10];
        let filtered = SignalConditioner::moving_average_filter(&signal, 3);
        for &v in &filtered {
            assert!((v - 5.0).abs() < 1e-12);
        }
    }

    #[test]
    fn test_moving_average_reduces_noise() {
        // Signal with noise
        let clean = 10.0;
        let signal: Vec<f64> = (0..100)
            .map(|i| clean + if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();

        let filtered = SignalConditioner::moving_average_filter(&signal, 4);

        // After window fills, filtered should be closer to clean
        let noise_filtered: f64 = filtered[10..]
            .iter()
            .map(|&v| (v - clean).powi(2))
            .sum::<f64>()
            / (filtered.len() - 10) as f64;

        let noise_raw: f64 = signal[10..]
            .iter()
            .map(|&v| (v - clean).powi(2))
            .sum::<f64>()
            / (signal.len() - 10) as f64;

        assert!(
            noise_filtered < noise_raw,
            "Moving average should reduce noise"
        );
    }

    // ---- Exponential filter ----

    #[test]
    fn test_exponential_filter_alpha_one() {
        // alpha=1 => no filtering, output = input
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let filtered = SignalConditioner::exponential_filter(&signal, 1.0);
        for (a, b) in signal.iter().zip(filtered.iter()) {
            assert!((a - b).abs() < 1e-12);
        }
    }

    #[test]
    fn test_exponential_filter_smoothing() {
        // Generate a longer oscillating signal so the filter can settle
        let n = 200;
        let signal: Vec<f64> = (0..n).map(|i| if i % 2 == 0 { 10.0 } else { 0.0 }).collect();
        let filtered = SignalConditioner::exponential_filter(&signal, 0.1);

        // Compare variance of raw vs filtered (skip initial transient)
        let raw_var: f64 = signal[100..].iter().map(|&v| (v - 5.0).powi(2)).sum::<f64>()
            / (n - 100) as f64;
        let filt_var: f64 = filtered[100..].iter().map(|&v| (v - 5.0).powi(2)).sum::<f64>()
            / (n - 100) as f64;

        assert!(
            filt_var < raw_var,
            "Filtered variance ({}) should be less than raw variance ({})",
            filt_var,
            raw_var
        );
    }

    // ---- Noise RMS ----

    #[test]
    fn test_noise_rms_zero_for_constant() {
        let signal = vec![7.0; 100];
        let rms = SignalConditioner::noise_rms(&signal);
        assert!(rms < 1e-12, "Constant signal has zero noise");
    }

    #[test]
    fn test_noise_rms_known_value() {
        // Alternating +1/-1 has mean 0 and RMS 1
        let signal: Vec<f64> = (0..100).map(|i| if i % 2 == 0 { 1.0 } else { -1.0 }).collect();
        let rms = SignalConditioner::noise_rms(&signal);
        assert!((rms - 1.0).abs() < 1e-12, "Expected RMS=1, got {}", rms);
    }

    // ---- Allan variance ----

    #[test]
    fn test_allan_variance_white_noise_decreases_with_averaging() {
        // White noise Allan variance ∝ 1/τ
        // Use a simple deterministic pseudo-noise sequence
        let n = 10000;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                // Simple LCG-like pseudo-noise
                let x = ((i as u64).wrapping_mul(6364136223846793005).wrapping_add(1)) as f64;
                (x / u64::MAX as f64) * 2.0 - 1.0
            })
            .collect();

        let av_short = SignalConditioner::allan_variance(&signal, 10);
        let av_long = SignalConditioner::allan_variance(&signal, 100);

        assert!(
            av_long < av_short,
            "Allan variance should decrease with longer averaging: short={}, long={}",
            av_short,
            av_long
        );
    }

    #[test]
    fn test_allan_variance_constant_is_zero() {
        let signal = vec![3.14; 1000];
        let av = SignalConditioner::allan_variance(&signal, 10);
        assert!(av < 1e-20, "Constant signal should have zero Allan variance");
    }

    // ---- PID controller ----

    #[test]
    fn test_pid_zero_error_zero_output() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let output = TemperatureController::pid_heater_output(
            4.2, 4.2, &mut integral, &mut prev_error, 1.0, 0.1, 0.01, 0.1,
        );
        assert!(
            output.abs() < 1e-12,
            "Zero error should give zero output, got {}",
            output
        );
    }

    #[test]
    fn test_pid_positive_error_positive_output() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let output = TemperatureController::pid_heater_output(
            4.2,
            3.0, // below setpoint
            &mut integral,
            &mut prev_error,
            1.0,
            0.0,
            0.0,
            0.1,
        );
        assert!(output > 0.0, "Below setpoint should produce positive heater output");
    }

    #[test]
    fn test_pid_output_clamped() {
        let mut integral = 0.0;
        let mut prev_error = 0.0;
        let output = TemperatureController::pid_heater_output(
            300.0,
            0.01, // huge error
            &mut integral,
            &mut prev_error,
            100.0,
            0.0,
            0.0,
            0.1,
        );
        assert!(
            (output - 1.0).abs() < 1e-12,
            "Output should be clamped to 1.0"
        );
    }

    // ---- Thermal time constant ----

    #[test]
    fn test_thermal_time_constant() {
        let tau = TemperatureController::thermal_time_constant(0.01, 0.001);
        assert!((tau - 10.0).abs() < 1e-12, "τ = C/G = 0.01/0.001 = 10 s");
    }

    #[test]
    fn test_thermal_time_constant_zero_conductance() {
        let tau = TemperatureController::thermal_time_constant(1.0, 0.0);
        assert!(tau.is_infinite());
    }

    // ---- Cooling curve ----

    #[test]
    fn test_cooling_curve_initial() {
        let t = TemperatureController::cooling_curve(300.0, 4.2, 100.0, 0.0);
        assert!((t - 300.0).abs() < 1e-12, "At t=0, should be initial temp");
    }

    #[test]
    fn test_cooling_curve_asymptotic() {
        let t = TemperatureController::cooling_curve(300.0, 4.2, 100.0, 1e6);
        assert!(
            (t - 4.2).abs() < 1e-6,
            "At t>>τ, should approach base temp, got {}",
            t
        );
    }

    #[test]
    fn test_cooling_curve_one_time_constant() {
        // At t=τ, T = base + (init-base) * exp(-1) ≈ base + 0.368*(init-base)
        let init = 300.0;
        let base = 0.0;
        let tau = 50.0;
        let t = TemperatureController::cooling_curve(init, base, tau, tau);
        let expected = base + (init - base) * (-1.0_f64).exp();
        assert!(
            (t - expected).abs() < 1e-10,
            "At t=τ, expected {}, got {}",
            expected,
            t
        );
    }

    // ---- Heat capacity ----

    #[test]
    fn test_heat_capacity_electron() {
        // C_e = γT
        let gamma = 0.001; // J/(mol·K²), typical for copper
        let c = TemperatureController::heat_capacity_electron(4.2, gamma);
        assert!((c - 0.0042).abs() < 1e-10);
    }

    #[test]
    fn test_heat_capacity_phonon_debye_t3() {
        // Debye T³ law: C ∝ T³ at low T
        let theta_d = 343.0; // Copper Debye temperature
        let c1 = TemperatureController::heat_capacity_phonon(1.0, theta_d);
        let c2 = TemperatureController::heat_capacity_phonon(2.0, theta_d);
        let ratio = c2 / c1;
        // Should be (2/1)^3 = 8
        assert!(
            (ratio - 8.0).abs() < 1e-6,
            "Debye T³ ratio should be 8, got {}",
            ratio
        );
    }

    #[test]
    fn test_heat_capacity_phonon_zero_temp() {
        let c = TemperatureController::heat_capacity_phonon(0.0, 343.0);
        assert!(c.abs() < 1e-20, "C_ph should be zero at T=0");
    }

    // ---- Dilution fridge cooling power ----

    #[test]
    fn test_mix_chamber_cooling_power_proportional_t_squared() {
        let q1 = CryostatMonitor::mix_chamber_cooling_power(100.0);
        let q2 = CryostatMonitor::mix_chamber_cooling_power(200.0);
        let ratio = q2 / q1;
        assert!(
            (ratio - 4.0).abs() < 1e-6,
            "Cooling power should scale as T², ratio should be 4, got {}",
            ratio
        );
    }

    #[test]
    fn test_mix_chamber_cooling_power_zero_temp() {
        let q = CryostatMonitor::mix_chamber_cooling_power(0.0);
        assert_eq!(q, 0.0);
    }

    #[test]
    fn test_still_power_positive() {
        let p = CryostatMonitor::still_power(0.7);
        assert!(p > 0.0, "Still power should be positive at 0.7 K");
    }

    // ---- He-3 vapor pressure ----

    #[test]
    fn test_helium3_vapor_pressure_increases_with_temperature() {
        let p1 = CryostatMonitor::helium3_vapor_pressure(1.0);
        let p2 = CryostatMonitor::helium3_vapor_pressure(2.0);
        assert!(
            p2 > p1,
            "Vapor pressure should increase with T: p(1K)={}, p(2K)={}",
            p1,
            p2
        );
    }

    #[test]
    fn test_helium3_vapor_pressure_zero_at_zero_k() {
        let p = CryostatMonitor::helium3_vapor_pressure(0.0);
        assert_eq!(p, 0.0);
    }

    // ---- Base temperature estimation ----

    #[test]
    fn test_estimated_base_temperature() {
        // More cooling power relative to heat leak => lower base temp
        let t1 = CryostatMonitor::estimated_base_temperature(100.0, 1.0);
        let t2 = CryostatMonitor::estimated_base_temperature(100.0, 10.0);
        assert!(
            t2 > t1,
            "More heat leak should give higher base temp: t1={}, t2={}",
            t1,
            t2
        );
    }

    #[test]
    fn test_estimated_base_temperature_zero_heat_leak() {
        let t = CryostatMonitor::estimated_base_temperature(100.0, 0.0);
        assert!((t - 0.0).abs() < 1e-12, "Zero heat leak => zero base temp");
    }

    // ---- Sensor type equality ----

    #[test]
    fn test_sensor_type_equality() {
        assert_eq!(SensorType::Cernox, SensorType::Cernox);
        assert_ne!(SensorType::Cernox, SensorType::RuO2);
        assert_eq!(
            SensorType::Thermocouple { type_code: 'K' },
            SensorType::Thermocouple { type_code: 'K' }
        );
        assert_ne!(
            SensorType::Thermocouple { type_code: 'K' },
            SensorType::Thermocouple { type_code: 'T' }
        );
    }

    // ---- Config constructor ----

    #[test]
    fn test_thermometer_constructor() {
        let config = make_config();
        let therm = CryogenicThermometer::new(config);
        assert_eq!(therm.config().sensor_type, SensorType::Cernox);
        assert!((therm.config().excitation_current_ua - 1.0).abs() < 1e-12);
    }
}

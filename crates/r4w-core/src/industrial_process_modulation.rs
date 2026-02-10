//! Industrial process control signal analysis for predictive maintenance and anomaly detection.
//!
//! This module provides tools for analyzing PID controller outputs, process variable trends,
//! setpoint tracking metrics, oscillation detection (limit cycles, hunting), valve stiction
//! detection, control loop performance indices (Harris index, Desborough-Harris), and alarm
//! rate analysis.
//!
//! # Example
//!
//! ```
//! use r4w_core::industrial_process_modulation::{ProcessAnalyzer, compute_ise, compute_iae};
//!
//! let analyzer = ProcessAnalyzer::new(10.0, 50.0);
//! let pv = vec![0.0, 10.0, 30.0, 45.0, 52.0, 51.0, 50.5, 50.1, 50.0, 50.0];
//! let metrics = analyzer.analyze(&pv);
//! assert!(metrics.ise >= 0.0);
//! assert!(metrics.iae >= 0.0);
//!
//! let ise = compute_ise(&pv, 50.0, 0.1);
//! assert!(ise > 0.0);
//!
//! let iae = compute_iae(&pv, 50.0, 0.1);
//! assert!(iae > 0.0);
//! ```

use std::f64::consts::PI;

/// Analyzer for industrial process control signals.
///
/// Holds the sample rate and setpoint for a control loop, and provides
/// methods to compute performance metrics from process variable data.
#[derive(Debug, Clone)]
pub struct ProcessAnalyzer {
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Desired setpoint for the process variable.
    pub setpoint: f64,
}

/// Performance metrics for a control loop.
///
/// Contains integral error metrics, overshoot, settling time, rise time,
/// and the Harris minimum-variance benchmark index.
#[derive(Debug, Clone)]
pub struct ControlLoopMetrics {
    /// Integral of squared error.
    pub ise: f64,
    /// Integral of absolute error.
    pub iae: f64,
    /// Percentage overshoot beyond setpoint.
    pub overshoot_pct: f64,
    /// Time in seconds for the process variable to settle within tolerance of setpoint.
    pub settling_time: f64,
    /// Time in seconds for the process variable to rise from 10% to 90% of the setpoint.
    pub rise_time: f64,
    /// Harris minimum-variance benchmark index (0.0 = optimal, 1.0 = poor).
    pub harris_index: f64,
}

impl ProcessAnalyzer {
    /// Create a new `ProcessAnalyzer` with the given sample rate and setpoint.
    ///
    /// # Arguments
    ///
    /// * `sample_rate_hz` - Sampling rate of the process variable in Hz.
    /// * `setpoint` - Desired process variable value.
    pub fn new(sample_rate_hz: f64, setpoint: f64) -> Self {
        Self {
            sample_rate_hz,
            setpoint,
        }
    }

    /// Analyze a process variable time series and return control loop metrics.
    ///
    /// Uses a default settling tolerance of 2% and computes ISE, IAE, overshoot,
    /// settling time, rise time, and Harris index.
    ///
    /// # Arguments
    ///
    /// * `pv` - Slice of process variable samples.
    pub fn analyze(&self, pv: &[f64]) -> ControlLoopMetrics {
        let dt = 1.0 / self.sample_rate_hz;
        let tolerance_pct = 2.0;

        ControlLoopMetrics {
            ise: compute_ise(pv, self.setpoint, dt),
            iae: compute_iae(pv, self.setpoint, dt),
            overshoot_pct: detect_overshoot(pv, self.setpoint),
            settling_time: settling_time(pv, self.setpoint, tolerance_pct, dt),
            rise_time: compute_rise_time(pv, self.setpoint, dt),
            harris_index: compute_harris_index(pv, self.setpoint),
        }
    }
}

/// Compute the Integral of Squared Error (ISE).
///
/// ISE = sum((pv[i] - setpoint)^2 * dt) for all samples.
///
/// # Arguments
///
/// * `pv` - Process variable samples.
/// * `setpoint` - Desired value.
/// * `dt` - Time step between samples in seconds.
pub fn compute_ise(pv: &[f64], setpoint: f64, dt: f64) -> f64 {
    pv.iter().map(|&v| (v - setpoint).powi(2) * dt).sum()
}

/// Compute the Integral of Absolute Error (IAE).
///
/// IAE = sum(|pv[i] - setpoint| * dt) for all samples.
///
/// # Arguments
///
/// * `pv` - Process variable samples.
/// * `setpoint` - Desired value.
/// * `dt` - Time step between samples in seconds.
pub fn compute_iae(pv: &[f64], setpoint: f64, dt: f64) -> f64 {
    pv.iter().map(|&v| (v - setpoint).abs() * dt).sum()
}

/// Detect the percentage overshoot of the process variable beyond the setpoint.
///
/// Returns the overshoot as a percentage of the setpoint. If the setpoint is zero,
/// returns the absolute maximum deviation. Returns 0.0 if there is no overshoot.
///
/// # Arguments
///
/// * `pv` - Process variable samples.
/// * `setpoint` - Desired value.
pub fn detect_overshoot(pv: &[f64], setpoint: f64) -> f64 {
    if pv.is_empty() {
        return 0.0;
    }

    // Determine the direction of the step response.
    let initial = pv[0];
    let max_val = pv.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min_val = pv.iter().cloned().fold(f64::INFINITY, f64::min);

    let overshoot = if setpoint >= initial {
        // Rising response: overshoot is above setpoint
        let peak = max_val;
        if peak > setpoint {
            peak - setpoint
        } else {
            0.0
        }
    } else {
        // Falling response: overshoot is below setpoint
        let valley = min_val;
        if valley < setpoint {
            setpoint - valley
        } else {
            0.0
        }
    };

    if setpoint.abs() < 1e-12 {
        overshoot
    } else {
        (overshoot / setpoint.abs()) * 100.0
    }
}

/// Compute the settling time of the process variable.
///
/// Settling time is the time after which the process variable remains within
/// `tolerance_pct`% of the setpoint for the rest of the signal.
///
/// # Arguments
///
/// * `pv` - Process variable samples.
/// * `setpoint` - Desired value.
/// * `tolerance_pct` - Tolerance band as a percentage (e.g., 2.0 for 2%).
/// * `dt` - Time step between samples in seconds.
pub fn settling_time(pv: &[f64], setpoint: f64, tolerance_pct: f64, dt: f64) -> f64 {
    if pv.is_empty() {
        return 0.0;
    }

    let band = if setpoint.abs() < 1e-12 {
        tolerance_pct / 100.0
    } else {
        setpoint.abs() * tolerance_pct / 100.0
    };

    // Walk backwards to find the last sample outside the tolerance band.
    let mut last_outside: Option<usize> = None;
    for (i, &v) in pv.iter().enumerate().rev() {
        if (v - setpoint).abs() > band {
            last_outside = Some(i);
            break;
        }
    }

    match last_outside {
        Some(idx) => (idx as f64 + 1.0) * dt,
        None => 0.0, // All samples within band
    }
}

/// Compute the rise time of the process variable (10% to 90% of setpoint).
///
/// # Arguments
///
/// * `pv` - Process variable samples.
/// * `setpoint` - Desired value.
/// * `dt` - Time step between samples in seconds.
fn compute_rise_time(pv: &[f64], setpoint: f64, dt: f64) -> f64 {
    if pv.is_empty() || pv.len() < 2 {
        return 0.0;
    }

    let initial = pv[0];
    let span = setpoint - initial;
    if span.abs() < 1e-12 {
        return 0.0;
    }

    let level_10 = initial + 0.1 * span;
    let level_90 = initial + 0.9 * span;

    let mut t10: Option<usize> = None;
    let mut t90: Option<usize> = None;

    if span > 0.0 {
        for (i, &v) in pv.iter().enumerate() {
            if t10.is_none() && v >= level_10 {
                t10 = Some(i);
            }
            if t90.is_none() && v >= level_90 {
                t90 = Some(i);
            }
            if t10.is_some() && t90.is_some() {
                break;
            }
        }
    } else {
        for (i, &v) in pv.iter().enumerate() {
            if t10.is_none() && v <= level_10 {
                t10 = Some(i);
            }
            if t90.is_none() && v <= level_90 {
                t90 = Some(i);
            }
            if t10.is_some() && t90.is_some() {
                break;
            }
        }
    }

    match (t10, t90) {
        (Some(a), Some(b)) => {
            if b > a {
                (b - a) as f64 * dt
            } else {
                0.0
            }
        }
        _ => 0.0,
    }
}

/// Detect oscillation in a process variable signal.
///
/// Uses zero-crossing analysis of the de-meaned signal to estimate oscillation
/// frequency and amplitude. Returns `(is_oscillating, frequency_hz, amplitude)`.
///
/// A signal is considered oscillating if at least 4 zero crossings are found
/// (i.e., at least 2 full cycles).
///
/// # Arguments
///
/// * `pv` - Process variable samples.
/// * `sample_rate_hz` - Sampling rate in Hz.
pub fn detect_oscillation(pv: &[f64], sample_rate_hz: f64) -> (bool, f64, f64) {
    if pv.len() < 4 {
        return (false, 0.0, 0.0);
    }

    // Remove DC (mean)
    let mean = pv.iter().sum::<f64>() / pv.len() as f64;
    let centered: Vec<f64> = pv.iter().map(|&v| v - mean).collect();

    // Count zero crossings
    let mut crossings = Vec::new();
    for i in 1..centered.len() {
        if centered[i - 1] * centered[i] < 0.0 {
            // Linear interpolation for sub-sample crossing point
            let frac = centered[i - 1].abs() / (centered[i - 1].abs() + centered[i].abs());
            crossings.push((i - 1) as f64 + frac);
        }
    }

    if crossings.len() < 4 {
        return (false, 0.0, 0.0);
    }

    // Estimate frequency from average half-period between consecutive crossings
    let mut half_periods = Vec::new();
    for i in 1..crossings.len() {
        half_periods.push(crossings[i] - crossings[i - 1]);
    }
    let avg_half_period = half_periods.iter().sum::<f64>() / half_periods.len() as f64;
    let frequency_hz = sample_rate_hz / (2.0 * avg_half_period);

    // Estimate amplitude as RMS * sqrt(2) for a sinusoidal signal
    let rms = (centered.iter().map(|&v| v * v).sum::<f64>() / centered.len() as f64).sqrt();
    let amplitude = rms * std::f64::consts::SQRT_2;

    (true, frequency_hz, amplitude)
}

/// Detect valve stiction from controller output and process variable data.
///
/// Stiction causes a characteristic pattern where the controller output (OP)
/// changes without corresponding changes in the process variable (PV), followed
/// by sudden jumps. This function estimates the stiction deadband.
///
/// Returns `(has_stiction, deadband_estimate)`.
///
/// # Arguments
///
/// * `op` - Controller output (manipulated variable) samples.
/// * `pv` - Process variable samples.
pub fn detect_stiction(op: &[f64], pv: &[f64]) -> (bool, f64) {
    let n = op.len().min(pv.len());
    if n < 3 {
        return (false, 0.0);
    }

    // Compute incremental changes
    let d_op: Vec<f64> = (1..n).map(|i| op[i] - op[i - 1]).collect();
    let d_pv: Vec<f64> = (1..n).map(|i| pv[i] - pv[i - 1]).collect();

    // Detect stiction: OP is changing but PV is stuck (then PV jumps suddenly)
    let op_threshold = {
        let max_d_op = d_op.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        max_d_op * 0.05 // 5% of max OP change as threshold for "OP is moving"
    };

    let pv_threshold = {
        let max_d_pv = d_pv.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        max_d_pv * 0.05 // 5% of max PV change as threshold for "PV is stuck"
    };

    if op_threshold < 1e-15 || pv_threshold < 1e-15 {
        return (false, 0.0);
    }

    // Count samples where OP is changing but PV is stuck
    let mut stuck_count = 0usize;
    let mut deadband_estimates = Vec::new();
    let mut in_stuck_region = false;
    let mut region_op_accum = 0.0_f64;

    for i in 0..d_op.len() {
        let op_moving = d_op[i].abs() > op_threshold;
        let pv_stuck = d_pv[i].abs() <= pv_threshold;

        if op_moving && pv_stuck {
            stuck_count += 1;
            region_op_accum += d_op[i].abs();
            in_stuck_region = true;
        } else if in_stuck_region {
            // Leaving a stuck region: record the accumulated OP change as a deadband estimate
            if region_op_accum > 0.0 {
                deadband_estimates.push(region_op_accum);
            }
            region_op_accum = 0.0;
            in_stuck_region = false;
        }
    }

    // Catch final stuck region
    if in_stuck_region && region_op_accum > 0.0 {
        deadband_estimates.push(region_op_accum);
    }

    // Stiction detected if a significant fraction of samples show stuck behavior
    let stuck_ratio = stuck_count as f64 / d_op.len() as f64;
    let has_stiction = stuck_ratio > 0.1 && !deadband_estimates.is_empty();

    let deadband = if deadband_estimates.is_empty() {
        0.0
    } else {
        deadband_estimates.iter().sum::<f64>() / deadband_estimates.len() as f64
    };

    (has_stiction, deadband)
}

/// Compute the Harris index (minimum variance benchmark) for a control loop.
///
/// The Harris index compares the variance of the process variable error against
/// the minimum achievable variance (estimated from the first-lag autocorrelation).
/// A value near 0 indicates the loop is performing near the theoretical optimum,
/// while a value near 1 indicates poor performance.
///
/// # Arguments
///
/// * `pv` - Process variable samples.
/// * `setpoint` - Desired value.
pub fn compute_harris_index(pv: &[f64], setpoint: f64) -> f64 {
    if pv.len() < 3 {
        return 0.0;
    }

    let errors: Vec<f64> = pv.iter().map(|&v| v - setpoint).collect();
    let n = errors.len();

    // Compute variance of the error signal
    let mean_error = errors.iter().sum::<f64>() / n as f64;
    let variance = errors.iter().map(|&e| (e - mean_error).powi(2)).sum::<f64>() / n as f64;

    if variance < 1e-15 {
        return 0.0;
    }

    // Estimate minimum variance from first-difference variance
    // The minimum variance controller would produce white noise residuals.
    // We approximate minimum variance as the variance of first differences / 2.
    let diffs: Vec<f64> = (1..n).map(|i| errors[i] - errors[i - 1]).collect();
    let diff_variance = diffs.iter().map(|&d| d * d).sum::<f64>() / diffs.len() as f64;
    let min_variance = diff_variance / 2.0;

    // Harris index: 1 - (min_variance / actual_variance)
    // Clamped to [0, 1]
    let index = 1.0 - (min_variance / variance);
    index.clamp(0.0, 1.0)
}

/// Compute a sliding-window alarm rate from a boolean alarm series.
///
/// Returns a vector of alarm rates (fraction of `true` values) computed over
/// a sliding window of the given size.
///
/// # Arguments
///
/// * `alarms` - Boolean slice where `true` indicates an alarm condition.
/// * `window_size` - Number of samples in the sliding window.
pub fn alarm_rate(alarms: &[bool], window_size: usize) -> Vec<f64> {
    if alarms.is_empty() || window_size == 0 || window_size > alarms.len() {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(alarms.len() - window_size + 1);

    // Initial window count
    let mut count: usize = alarms[..window_size].iter().filter(|&&a| a).count();
    result.push(count as f64 / window_size as f64);

    // Slide the window
    for i in window_size..alarms.len() {
        if alarms[i] {
            count += 1;
        }
        if alarms[i - window_size] {
            count -= 1;
        }
        result.push(count as f64 / window_size as f64);
    }

    result
}

/// Extract a linear trend from a process variable time series.
///
/// Fits a least-squares line y = slope * x + intercept where x is the sample index.
/// Returns `(slope, intercept)`.
///
/// # Arguments
///
/// * `pv` - Process variable samples.
pub fn trend_extract(pv: &[f64]) -> (f64, f64) {
    let n = pv.len();
    if n == 0 {
        return (0.0, 0.0);
    }
    if n == 1 {
        return (0.0, pv[0]);
    }

    let n_f = n as f64;
    let sum_x: f64 = (0..n).map(|i| i as f64).sum();
    let sum_y: f64 = pv.iter().sum();
    let sum_xy: f64 = pv.iter().enumerate().map(|(i, &y)| i as f64 * y).sum();
    let sum_x2: f64 = (0..n).map(|i| (i as f64).powi(2)).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-15 {
        return (0.0, sum_y / n_f);
    }

    let slope = (n_f * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n_f;

    (slope, intercept)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn test_process_analyzer_new() {
        let pa = ProcessAnalyzer::new(100.0, 50.0);
        assert_eq!(pa.sample_rate_hz, 100.0);
        assert_eq!(pa.setpoint, 50.0);
    }

    #[test]
    fn test_process_analyzer_analyze() {
        let pa = ProcessAnalyzer::new(10.0, 50.0);
        let pv = vec![0.0, 10.0, 25.0, 40.0, 48.0, 52.0, 51.0, 50.5, 50.1, 50.0];
        let metrics = pa.analyze(&pv);
        assert!(metrics.ise > 0.0);
        assert!(metrics.iae > 0.0);
        assert!(metrics.overshoot_pct > 0.0);
        assert!(metrics.settling_time >= 0.0);
        assert!(metrics.rise_time >= 0.0);
        assert!(metrics.harris_index >= 0.0 && metrics.harris_index <= 1.0);
    }

    #[test]
    fn test_compute_ise_zero_error() {
        let pv = vec![50.0; 10];
        let ise = compute_ise(&pv, 50.0, 0.1);
        assert!(approx_eq(ise, 0.0, EPSILON));
    }

    #[test]
    fn test_compute_ise_known_value() {
        // Error of 1.0 for 5 samples at dt=0.1 -> ISE = 5 * 1.0^2 * 0.1 = 0.5
        let pv = vec![51.0; 5];
        let ise = compute_ise(&pv, 50.0, 0.1);
        assert!(approx_eq(ise, 0.5, EPSILON));
    }

    #[test]
    fn test_compute_iae_zero_error() {
        let pv = vec![50.0; 10];
        let iae = compute_iae(&pv, 50.0, 0.1);
        assert!(approx_eq(iae, 0.0, EPSILON));
    }

    #[test]
    fn test_compute_iae_known_value() {
        // Error of 2.0 for 4 samples at dt=0.5 -> IAE = 4 * 2.0 * 0.5 = 4.0
        let pv = vec![52.0; 4];
        let iae = compute_iae(&pv, 50.0, 0.5);
        assert!(approx_eq(iae, 4.0, EPSILON));
    }

    #[test]
    fn test_detect_overshoot_no_overshoot() {
        // Approaching from below, never exceeds setpoint
        let pv = vec![0.0, 10.0, 20.0, 30.0, 40.0, 49.0];
        let os = detect_overshoot(&pv, 50.0);
        assert!(approx_eq(os, 0.0, EPSILON));
    }

    #[test]
    fn test_detect_overshoot_with_overshoot() {
        // Setpoint = 100, max = 120 -> 20% overshoot
        let pv = vec![0.0, 50.0, 100.0, 120.0, 105.0, 100.0];
        let os = detect_overshoot(&pv, 100.0);
        assert!(approx_eq(os, 20.0, EPSILON));
    }

    #[test]
    fn test_detect_overshoot_empty() {
        let pv: Vec<f64> = vec![];
        let os = detect_overshoot(&pv, 50.0);
        assert_eq!(os, 0.0);
    }

    #[test]
    fn test_settling_time_already_settled() {
        let pv = vec![50.0; 20];
        let st = settling_time(&pv, 50.0, 2.0, 0.1);
        assert!(approx_eq(st, 0.0, EPSILON));
    }

    #[test]
    fn test_settling_time_known() {
        // Outside tolerance at index 4, within from index 5 onward
        // setpoint=100, 2% tolerance band = [98, 102]
        let pv = vec![0.0, 50.0, 80.0, 95.0, 110.0, 100.5, 99.8, 100.1, 100.0];
        let st = settling_time(&pv, 100.0, 2.0, 1.0);
        // Last outside sample is index 4 (110.0 is outside [98,102])
        assert!(approx_eq(st, 5.0, EPSILON));
    }

    #[test]
    fn test_detect_oscillation_sine_wave() {
        let sample_rate = 1000.0;
        let freq = 10.0;
        let n = 1000;
        let pv: Vec<f64> = (0..n)
            .map(|i| 5.0 * (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect();

        let (oscillating, detected_freq, amplitude) = detect_oscillation(&pv, sample_rate);
        assert!(oscillating);
        assert!(approx_eq(detected_freq, freq, 1.0)); // Within 1 Hz
        assert!(approx_eq(amplitude, 5.0, 0.5)); // Within 0.5 of true amplitude
    }

    #[test]
    fn test_detect_oscillation_constant() {
        let pv = vec![50.0; 100];
        let (oscillating, _, _) = detect_oscillation(&pv, 100.0);
        assert!(!oscillating);
    }

    #[test]
    fn test_detect_oscillation_short_signal() {
        let pv = vec![1.0, 2.0];
        let (oscillating, _, _) = detect_oscillation(&pv, 100.0);
        assert!(!oscillating);
    }

    #[test]
    fn test_detect_stiction_no_stiction() {
        // OP and PV change together proportionally
        let n = 100;
        let op: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let pv: Vec<f64> = (0..n).map(|i| 0.5 * i as f64).collect();
        let (has_stiction, _) = detect_stiction(&op, &pv);
        assert!(!has_stiction);
    }

    #[test]
    fn test_detect_stiction_with_stiction() {
        // Simulate stiction: OP ramps but PV stays stuck, then jumps
        let mut op = Vec::new();
        let mut pv = Vec::new();
        for i in 0..100 {
            op.push(i as f64);
            if i < 30 {
                pv.push(0.0); // PV stuck while OP ramps
            } else if i < 31 {
                pv.push(25.0); // PV jumps
            } else if i < 60 {
                pv.push(25.0); // PV stuck again while OP continues
            } else if i < 61 {
                pv.push(50.0); // PV jumps again
            } else {
                pv.push(50.0);
            }
        }
        let (has_stiction, deadband) = detect_stiction(&op, &pv);
        assert!(has_stiction);
        assert!(deadband > 0.0);
    }

    #[test]
    fn test_detect_stiction_short_signal() {
        let op = vec![1.0, 2.0];
        let pv = vec![1.0, 2.0];
        let (has_stiction, _) = detect_stiction(&op, &pv);
        assert!(!has_stiction);
    }

    #[test]
    fn test_harris_index_perfect_control() {
        // If the PV is exactly at setpoint, Harris index should be 0
        let pv = vec![50.0; 100];
        let hi = compute_harris_index(&pv, 50.0);
        assert!(approx_eq(hi, 0.0, EPSILON));
    }

    #[test]
    fn test_harris_index_bounded() {
        // Harris index should always be in [0, 1]
        let pv: Vec<f64> = (0..100)
            .map(|i| 50.0 + 10.0 * (0.1 * i as f64).sin())
            .collect();
        let hi = compute_harris_index(&pv, 50.0);
        assert!(hi >= 0.0 && hi <= 1.0, "Harris index {} out of bounds", hi);
    }

    #[test]
    fn test_alarm_rate_all_alarms() {
        let alarms = vec![true; 10];
        let rates = alarm_rate(&alarms, 5);
        assert_eq!(rates.len(), 6);
        for &r in &rates {
            assert!(approx_eq(r, 1.0, EPSILON));
        }
    }

    #[test]
    fn test_alarm_rate_no_alarms() {
        let alarms = vec![false; 10];
        let rates = alarm_rate(&alarms, 5);
        assert_eq!(rates.len(), 6);
        for &r in &rates {
            assert!(approx_eq(r, 0.0, EPSILON));
        }
    }

    #[test]
    fn test_alarm_rate_mixed() {
        // [true, false, true, false, true] with window 3
        // Windows: [T,F,T]=2/3, [F,T,F]=1/3, [T,F,T]=2/3
        let alarms = vec![true, false, true, false, true];
        let rates = alarm_rate(&alarms, 3);
        assert_eq!(rates.len(), 3);
        assert!(approx_eq(rates[0], 2.0 / 3.0, EPSILON));
        assert!(approx_eq(rates[1], 1.0 / 3.0, EPSILON));
        assert!(approx_eq(rates[2], 2.0 / 3.0, EPSILON));
    }

    #[test]
    fn test_alarm_rate_empty() {
        let alarms: Vec<bool> = vec![];
        let rates = alarm_rate(&alarms, 5);
        assert!(rates.is_empty());
    }

    #[test]
    fn test_trend_extract_flat() {
        let pv = vec![5.0; 10];
        let (slope, intercept) = trend_extract(&pv);
        assert!(approx_eq(slope, 0.0, EPSILON));
        assert!(approx_eq(intercept, 5.0, EPSILON));
    }

    #[test]
    fn test_trend_extract_linear() {
        // y = 2*x + 1 for x = 0..9
        let pv: Vec<f64> = (0..10).map(|i| 2.0 * i as f64 + 1.0).collect();
        let (slope, intercept) = trend_extract(&pv);
        assert!(approx_eq(slope, 2.0, EPSILON));
        assert!(approx_eq(intercept, 1.0, EPSILON));
    }

    #[test]
    fn test_trend_extract_empty() {
        let pv: Vec<f64> = vec![];
        let (slope, intercept) = trend_extract(&pv);
        assert_eq!(slope, 0.0);
        assert_eq!(intercept, 0.0);
    }

    #[test]
    fn test_trend_extract_single() {
        let pv = vec![42.0];
        let (slope, intercept) = trend_extract(&pv);
        assert!(approx_eq(slope, 0.0, EPSILON));
        assert!(approx_eq(intercept, 42.0, EPSILON));
    }
}

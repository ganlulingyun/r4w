//! Optical Time-Domain Reflectometry (OTDR) signal processing for fiber optic fault localization.
//!
//! This module provides tools for analyzing OTDR traces to detect and classify
//! fiber optic events such as connectors, splices, bends, and breaks. It includes
//! Rayleigh backscatter trace analysis, event detection with loss measurement,
//! fiber attenuation coefficient calculation, optical return loss (ORL),
//! two-point loss measurement, dead zone identification, trace averaging for
//! noise reduction, and distance-to-fault calculation.
//!
//! # Example
//!
//! ```
//! use r4w_core::otdr_pulse_analyzer::{
//!     OtdrConfig, OtdrAnalyzer, EventType, generate_otdr_trace,
//! };
//!
//! // Define fiber events: (distance_m, loss_db, reflectance_db)
//! let events = vec![
//!     (500.0, 0.5, -40.0),   // connector at 500 m
//!     (2000.0, 0.1, -60.0),  // splice at 2 km
//! ];
//!
//! // Generate a synthetic OTDR trace
//! let (trace_db, distance_m) = generate_otdr_trace(5000.0, 0.2, &events, 1.4681);
//!
//! // Create analyzer and detect events
//! let config = OtdrConfig {
//!     wavelength_nm: 1550.0,
//!     pulse_width_ns: 100.0,
//!     ior: 1.4681,
//!     sample_rate_hz: 1e9,
//!     averaging_count: 16,
//! };
//! let analyzer = OtdrAnalyzer::new(config);
//! let detected = analyzer.detect_events(&trace_db, &distance_m, 0.3);
//! assert!(detected.len() >= 2);
//! ```

use std::f64;

/// Speed of light in vacuum (m/s).
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Configuration parameters for an OTDR measurement.
#[derive(Debug, Clone)]
pub struct OtdrConfig {
    /// Laser wavelength in nanometers (e.g., 1310.0 or 1550.0).
    pub wavelength_nm: f64,
    /// Pulse width in nanoseconds.
    pub pulse_width_ns: f64,
    /// Index of refraction of the fiber core (typically ~1.4681 for SMF-28).
    pub ior: f64,
    /// Sample rate of the OTDR receiver in Hz.
    pub sample_rate_hz: f64,
    /// Number of traces to average for noise reduction.
    pub averaging_count: u32,
}

/// Classification of a detected event on the fiber.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EventType {
    /// Mechanical or fusion connector (high reflectance, moderate loss).
    Connector,
    /// Fusion splice (low reflectance, low loss).
    Splice,
    /// Macro-bend in the fiber (loss without significant reflectance).
    Bend,
    /// Fiber break or open end (very high loss/reflectance).
    Break,
    /// End of fiber (Fresnel reflection at cleaved or polished end).
    End,
    /// Unclassified event.
    Unknown,
}

/// A detected event on the OTDR trace.
#[derive(Debug, Clone)]
pub struct OtdrEvent {
    /// Distance from the OTDR to the event in meters.
    pub distance_m: f64,
    /// Insertion loss at the event in dB (positive means loss).
    pub loss_db: f64,
    /// Reflectance of the event in dB (negative, closer to 0 means higher reflectance).
    pub reflectance_db: f64,
    /// Classification of the event.
    pub event_type: EventType,
}

/// Main OTDR trace analyzer.
///
/// Provides methods for event detection, attenuation measurement,
/// ORL calculation, and other OTDR analysis functions.
#[derive(Debug, Clone)]
pub struct OtdrAnalyzer {
    /// Configuration for this analyzer instance.
    pub config: OtdrConfig,
}

impl OtdrAnalyzer {
    /// Create a new analyzer with the given configuration.
    pub fn new(config: OtdrConfig) -> Self {
        Self { config }
    }

    /// Detect events in an OTDR trace.
    ///
    /// Scans the trace for abrupt changes (drops or spikes) that exceed
    /// `threshold_db`. Returns a list of detected events sorted by distance.
    ///
    /// # Arguments
    ///
    /// * `trace_db` - OTDR trace power values in dB.
    /// * `distance_m` - Corresponding distance axis in meters.
    /// * `threshold_db` - Minimum deviation in dB to register as an event.
    pub fn detect_events(
        &self,
        trace_db: &[f64],
        distance_m: &[f64],
        threshold_db: f64,
    ) -> Vec<OtdrEvent> {
        detect_events(trace_db, distance_m, threshold_db)
    }

    /// Calculate fiber attenuation between two indices in dB/km.
    pub fn fiber_attenuation(
        &self,
        trace_db: &[f64],
        distance_m: &[f64],
        start_idx: usize,
        end_idx: usize,
    ) -> f64 {
        fiber_attenuation(trace_db, distance_m, start_idx, end_idx)
    }

    /// Calculate two-point loss between two indices in dB.
    pub fn two_point_loss(
        &self,
        trace_db: &[f64],
        distance_m: &[f64],
        point_a: usize,
        point_b: usize,
    ) -> f64 {
        two_point_loss(trace_db, distance_m, point_a, point_b)
    }

    /// Calculate optical return loss (ORL) in dB.
    pub fn optical_return_loss(&self, trace_db: &[f64]) -> f64 {
        optical_return_loss(trace_db)
    }

    /// Calculate the dead zone length for the configured pulse width.
    pub fn dead_zone_length(&self) -> f64 {
        dead_zone_length(self.config.pulse_width_ns, self.config.ior)
    }

    /// Calculate distance to a fault given a round-trip time in nanoseconds.
    pub fn distance_to_fault(&self, time_ns: f64) -> f64 {
        distance_to_fault(time_ns, self.config.ior)
    }
}

/// Detect events in an OTDR trace.
///
/// This function examines the derivative of the trace to find abrupt level
/// changes. Events are identified where the local drop or spike exceeds
/// `threshold_db`. Loss is estimated from the level change, and reflectance
/// from any spike above the backscatter baseline.
///
/// # Arguments
///
/// * `trace_db` - OTDR trace power values in dB.
/// * `distance_m` - Corresponding distance axis in meters.
/// * `threshold_db` - Minimum deviation in dB to register as an event.
///
/// # Returns
///
/// A vector of `OtdrEvent` sorted by distance.
pub fn detect_events(
    trace_db: &[f64],
    distance_m: &[f64],
    threshold_db: f64,
) -> Vec<OtdrEvent> {
    if trace_db.len() < 3 || distance_m.len() < 3 {
        return Vec::new();
    }
    let n = trace_db.len().min(distance_m.len());

    // Compute the local derivative (difference) of the trace
    let mut derivatives: Vec<f64> = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        derivatives.push(trace_db[i + 1] - trace_db[i]);
    }

    // Estimate the baseline backscatter slope using the median derivative
    let baseline_slope = median_value(&derivatives);

    // Look for events: points where the derivative deviates significantly from
    // the baseline slope. We use a small window to measure loss and reflectance.
    let mut events = Vec::new();
    let mut i = 1;
    let window = 5.min(n / 4).max(1);

    while i < n - 1 {
        let deriv = derivatives[i - 1]; // derivative from i-1 to i
        let deviation = deriv - baseline_slope;

        if deviation.abs() > threshold_db {
            // Found a potential event at index i
            // Measure loss: compare average level before and after the event
            let pre_start = if i > window { i - window } else { 0 };
            let post_end = (i + window + 1).min(n);
            let pre_avg = mean_slice(&trace_db[pre_start..i]);
            let post_avg = mean_slice(&trace_db[i + 1..post_end]);
            let loss_db = pre_avg - post_avg;

            // Measure reflectance: if there is a spike at the event location
            // above the backscatter baseline, it indicates reflectance
            let expected_level = pre_avg + baseline_slope;
            let reflectance_db = if trace_db[i] > expected_level + 0.5 {
                // Reflective event: the spike height relative to baseline
                -(trace_db[i] - expected_level).max(1.0)
            } else {
                // Non-reflective event: assign low reflectance
                -65.0
            };

            let event_type = classify_event(loss_db, reflectance_db);

            events.push(OtdrEvent {
                distance_m: distance_m[i],
                loss_db,
                reflectance_db,
                event_type,
            });

            // Skip past the event region to avoid double-detection
            i += window.max(2);
        } else {
            i += 1;
        }
    }

    events.sort_by(|a, b| {
        a.distance_m
            .partial_cmp(&b.distance_m)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    events
}

/// Calculate fiber attenuation coefficient in dB/km between two trace indices.
///
/// Performs a least-squares linear fit on the trace segment to determine the
/// slope, then converts to dB/km.
///
/// # Arguments
///
/// * `trace_db` - OTDR trace power values in dB.
/// * `distance_m` - Corresponding distance axis in meters.
/// * `start_idx` - Starting index of the segment.
/// * `end_idx` - Ending index of the segment (exclusive).
///
/// # Returns
///
/// Attenuation coefficient in dB/km (positive value).
pub fn fiber_attenuation(
    trace_db: &[f64],
    distance_m: &[f64],
    start_idx: usize,
    end_idx: usize,
) -> f64 {
    if start_idx >= end_idx || end_idx > trace_db.len() || end_idx > distance_m.len() {
        return 0.0;
    }

    let segment_trace = &trace_db[start_idx..end_idx];
    let segment_dist = &distance_m[start_idx..end_idx];
    let n = segment_trace.len();
    if n < 2 {
        return 0.0;
    }

    // Least-squares linear fit: trace_db = slope * distance_m + intercept
    let mean_d = mean_slice(segment_dist);
    let mean_t = mean_slice(segment_trace);

    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..n {
        let dd = segment_dist[i] - mean_d;
        let dt = segment_trace[i] - mean_t;
        num += dd * dt;
        den += dd * dd;
    }

    if den.abs() < 1e-30 {
        return 0.0;
    }

    let slope_db_per_m = num / den; // dB/m (will be negative for loss)
    // Convert to dB/km and return as positive value
    // OTDR trace shows backscatter, so the one-way attenuation is half the
    // round-trip slope
    let one_way_slope = slope_db_per_m / 2.0;
    (-one_way_slope * 1000.0).max(0.0)
}

/// Calculate two-point loss between two indices in dB.
///
/// Simply computes the difference in trace power between point A and point B.
/// A positive result means loss (signal is weaker at point B).
///
/// # Arguments
///
/// * `trace_db` - OTDR trace power values in dB.
/// * `distance_m` - Corresponding distance axis in meters (unused but included for API consistency).
/// * `point_a` - Index of the first measurement point.
/// * `point_b` - Index of the second measurement point.
///
/// # Returns
///
/// Loss in dB (positive means signal decreased from A to B).
pub fn two_point_loss(
    trace_db: &[f64],
    _distance_m: &[f64],
    point_a: usize,
    point_b: usize,
) -> f64 {
    if point_a >= trace_db.len() || point_b >= trace_db.len() {
        return 0.0;
    }
    trace_db[point_a] - trace_db[point_b]
}

/// Calculate the optical return loss (ORL) of the entire trace.
///
/// ORL is defined as the ratio of input power to total reflected power,
/// expressed in dB. A higher ORL (positive) indicates less total reflection.
///
/// For an OTDR trace in dB, ORL is approximated by computing the integrated
/// backscatter power relative to the launch power.
///
/// # Arguments
///
/// * `trace_db` - OTDR trace power values in dB.
///
/// # Returns
///
/// ORL in dB (positive value; higher is better).
pub fn optical_return_loss(trace_db: &[f64]) -> f64 {
    if trace_db.is_empty() {
        return 0.0;
    }

    // The launch power is approximately the first sample
    let launch_power_db = trace_db[0];

    // Integrate the total reflected power (sum in linear domain)
    let mut total_reflected_linear = 0.0;
    for &val in trace_db {
        total_reflected_linear += 10.0_f64.powf(val / 10.0);
    }

    let n = trace_db.len() as f64;
    // Average reflected power level
    let avg_reflected_db = 10.0 * (total_reflected_linear / n).log10();

    // ORL = launch_power - avg_reflected (all reflected power)
    (launch_power_db - avg_reflected_db).abs()
}

/// Calculate the one-way distance to a fault from the round-trip time.
///
/// Uses the relationship: distance = (c * time) / (2 * n), where c is the
/// speed of light, time is the round-trip time, and n is the index of refraction.
///
/// # Arguments
///
/// * `time_ns` - Round-trip time in nanoseconds.
/// * `ior` - Index of refraction of the fiber.
///
/// # Returns
///
/// One-way distance in meters.
pub fn distance_to_fault(time_ns: f64, ior: f64) -> f64 {
    if ior <= 0.0 {
        return 0.0;
    }
    let time_s = time_ns * 1e-9;
    (SPEED_OF_LIGHT * time_s) / (2.0 * ior)
}

/// Perform coherent trace averaging across multiple OTDR traces.
///
/// Averages corresponding samples point-by-point, reducing noise by
/// approximately `sqrt(N)` where N is the number of traces.
///
/// # Arguments
///
/// * `traces` - Slice of trace vectors, each of the same length.
///
/// # Returns
///
/// The averaged trace. Returns an empty vector if input is empty.
pub fn average_traces(traces: &[Vec<f64>]) -> Vec<f64> {
    if traces.is_empty() {
        return Vec::new();
    }

    let len = traces[0].len();
    if len == 0 {
        return Vec::new();
    }

    let count = traces.len() as f64;
    let mut result = vec![0.0; len];

    for trace in traces {
        let trace_len = trace.len().min(len);
        for i in 0..trace_len {
            result[i] += trace[i];
        }
    }

    for val in &mut result {
        *val /= count;
    }

    result
}

/// Calculate the event dead zone length for a given pulse width.
///
/// The dead zone is the minimum distance after a reflective event within
/// which the OTDR cannot detect another event. It is primarily determined
/// by the pulse width.
///
/// dead_zone = (c * pulse_width) / (2 * n)
///
/// # Arguments
///
/// * `pulse_width_ns` - OTDR pulse width in nanoseconds.
/// * `ior` - Index of refraction.
///
/// # Returns
///
/// Dead zone length in meters.
pub fn dead_zone_length(pulse_width_ns: f64, ior: f64) -> f64 {
    if ior <= 0.0 {
        return 0.0;
    }
    let pulse_width_s = pulse_width_ns * 1e-9;
    (SPEED_OF_LIGHT * pulse_width_s) / (2.0 * ior)
}

/// Classify an event based on its loss and reflectance characteristics.
///
/// Classification heuristics:
/// - **Break**: loss > 10 dB
/// - **Connector**: reflectance > -50 dB and loss 0.2-10 dB
/// - **Splice**: reflectance <= -50 dB and loss < 0.5 dB
/// - **Bend**: non-reflective (reflectance <= -55 dB) with loss 0.1-10 dB
/// - **End**: high reflectance (> -30 dB) with high loss (> 10 dB)
/// - **Unknown**: anything else
///
/// # Arguments
///
/// * `loss_db` - Event insertion loss in dB (positive = loss).
/// * `reflectance_db` - Event reflectance in dB (negative, closer to 0 = more reflective).
///
/// # Returns
///
/// The classified `EventType`.
pub fn classify_event(loss_db: f64, reflectance_db: f64) -> EventType {
    // Break: very high loss
    if loss_db > 10.0 {
        if reflectance_db > -30.0 {
            return EventType::End;
        }
        return EventType::Break;
    }

    // High reflectance with significant loss: connector
    if reflectance_db > -50.0 && loss_db >= 0.2 {
        return EventType::Connector;
    }

    // High reflectance, low loss: also connector (e.g., APC connector)
    if reflectance_db > -50.0 && loss_db >= 0.0 {
        return EventType::Connector;
    }

    // Non-reflective with loss: bend
    if reflectance_db <= -55.0 && loss_db >= 0.2 {
        return EventType::Bend;
    }

    // Low reflectance, low loss: splice
    if reflectance_db <= -50.0 && loss_db < 0.5 && loss_db >= -0.5 {
        return EventType::Splice;
    }

    EventType::Unknown
}

/// Generate a synthetic OTDR trace for testing and simulation.
///
/// Creates a trace with linear Rayleigh backscatter attenuation and
/// superimposed events at specified locations.
///
/// # Arguments
///
/// * `fiber_length_m` - Total fiber length in meters.
/// * `attenuation_dbkm` - Fiber attenuation coefficient in dB/km.
/// * `events` - Slice of (distance_m, loss_db, reflectance_db) tuples.
/// * `ior` - Index of refraction.
///
/// # Returns
///
/// A tuple `(trace_db, distance_m)` where `trace_db` is the power trace and
/// `distance_m` is the corresponding distance axis.
pub fn generate_otdr_trace(
    fiber_length_m: f64,
    attenuation_dbkm: f64,
    events: &[(f64, f64, f64)],
    ior: f64,
) -> (Vec<f64>, Vec<f64>) {
    // Number of samples: 1 sample per meter, minimum 10
    let n_samples = (fiber_length_m as usize).max(10);
    let dx = fiber_length_m / (n_samples as f64 - 1.0);

    let mut trace_db = Vec::with_capacity(n_samples);
    let mut distance_m = Vec::with_capacity(n_samples);

    // Starting power level (launch power at OTDR output)
    let launch_power_db = 0.0;

    // Attenuation in dB/m (round-trip, so 2x one-way)
    let atten_db_per_m = 2.0 * attenuation_dbkm / 1000.0;

    // Sort events by distance for proper cumulative loss tracking
    let mut sorted_events: Vec<(f64, f64, f64)> = events.to_vec();
    sorted_events.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Pre-compute cumulative event loss at each sample point
    // and spike contributions
    for i in 0..n_samples {
        let d = i as f64 * dx;
        distance_m.push(d);

        // Cumulative loss from all events before this distance
        let mut cum_loss = 0.0;
        let mut spike = 0.0;
        for &(event_dist, event_loss, event_reflectance) in &sorted_events {
            if d > event_dist + dx {
                cum_loss += event_loss;
            } else if (d - event_dist).abs() < dx * 0.75 {
                // At the event location: add spike for reflectance
                spike = (-event_reflectance - 20.0).max(0.0);
                // Also add half the event loss (transition point)
                cum_loss += event_loss * 0.5;
            }
        }

        // Base Rayleigh backscatter level
        let base_level = launch_power_db - atten_db_per_m * d - cum_loss;

        // After the fiber end, signal drops to noise floor
        let noise_floor = launch_power_db - 40.0;
        if d > fiber_length_m * 0.98 {
            trace_db.push(noise_floor);
        } else {
            trace_db.push(base_level + spike);
        }
    }

    // Add the end-of-fiber Fresnel reflection
    if n_samples > 2 {
        let end_idx = (fiber_length_m * 0.97 / dx) as usize;
        if end_idx < n_samples {
            let _ = ior; // ior affects dead zone but not trace shape directly
            trace_db[end_idx] = trace_db[end_idx.saturating_sub(1)] + 15.0;
        }
    }

    (trace_db, distance_m)
}

// ---------------------------------------------------------------------------
// Internal helper functions
// ---------------------------------------------------------------------------

/// Compute the mean of a slice of f64 values.
fn mean_slice(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let sum: f64 = data.iter().sum();
    sum / data.len() as f64
}

/// Compute the median of a slice of f64 values (non-destructive).
fn median_value(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f64> = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEFAULT_IOR: f64 = 1.4681;
    const TOLERANCE: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_distance_to_fault_basic() {
        // 10 ns round-trip in fiber with IOR 1.4681
        let dist = distance_to_fault(10.0, DEFAULT_IOR);
        let expected = (SPEED_OF_LIGHT * 10e-9) / (2.0 * DEFAULT_IOR);
        assert!(
            approx_eq(dist, expected, 1e-6),
            "dist={dist}, expected={expected}"
        );
    }

    #[test]
    fn test_distance_to_fault_zero_time() {
        let dist = distance_to_fault(0.0, DEFAULT_IOR);
        assert!(approx_eq(dist, 0.0, TOLERANCE));
    }

    #[test]
    fn test_distance_to_fault_invalid_ior() {
        assert!(approx_eq(distance_to_fault(100.0, 0.0), 0.0, TOLERANCE));
        assert!(approx_eq(distance_to_fault(100.0, -1.0), 0.0, TOLERANCE));
    }

    #[test]
    fn test_dead_zone_length_basic() {
        // 100 ns pulse
        let dz = dead_zone_length(100.0, DEFAULT_IOR);
        let expected = (SPEED_OF_LIGHT * 100e-9) / (2.0 * DEFAULT_IOR);
        assert!(
            approx_eq(dz, expected, 1e-3),
            "dz={dz}, expected={expected}"
        );
        // Should be roughly 10.2 meters
        assert!(dz > 9.0 && dz < 12.0, "dead zone={dz} m");
    }

    #[test]
    fn test_dead_zone_length_short_pulse() {
        let dz = dead_zone_length(10.0, DEFAULT_IOR);
        // 10 ns pulse -> ~1.02 m dead zone
        assert!(dz > 0.5 && dz < 2.0, "dead zone={dz} m");
    }

    #[test]
    fn test_dead_zone_length_invalid_ior() {
        assert!(approx_eq(dead_zone_length(100.0, 0.0), 0.0, TOLERANCE));
    }

    #[test]
    fn test_classify_event_connector() {
        let et = classify_event(0.5, -40.0);
        assert_eq!(et, EventType::Connector);
    }

    #[test]
    fn test_classify_event_splice() {
        let et = classify_event(0.1, -55.0);
        assert_eq!(et, EventType::Splice);
    }

    #[test]
    fn test_classify_event_bend() {
        let et = classify_event(0.5, -60.0);
        assert_eq!(et, EventType::Bend);
    }

    #[test]
    fn test_classify_event_break() {
        let et = classify_event(15.0, -45.0);
        assert_eq!(et, EventType::Break);
    }

    #[test]
    fn test_classify_event_end() {
        let et = classify_event(15.0, -20.0);
        assert_eq!(et, EventType::End);
    }

    #[test]
    fn test_classify_event_unknown() {
        // Negative loss, low reflectance => Unknown
        let et = classify_event(-1.0, -70.0);
        assert_eq!(et, EventType::Unknown);
    }

    #[test]
    fn test_average_traces_single() {
        let traces = vec![vec![1.0, 2.0, 3.0]];
        let avg = average_traces(&traces);
        assert_eq!(avg, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_average_traces_multiple() {
        let traces = vec![vec![1.0, 2.0, 3.0], vec![3.0, 4.0, 5.0]];
        let avg = average_traces(&traces);
        assert!(approx_eq(avg[0], 2.0, TOLERANCE));
        assert!(approx_eq(avg[1], 3.0, TOLERANCE));
        assert!(approx_eq(avg[2], 4.0, TOLERANCE));
    }

    #[test]
    fn test_average_traces_empty() {
        let traces: Vec<Vec<f64>> = Vec::new();
        let avg = average_traces(&traces);
        assert!(avg.is_empty());
    }

    #[test]
    fn test_average_traces_noise_reduction() {
        // Multiple noisy traces should average to a smoother result
        let base = 5.0;
        let offsets = [0.5, -0.3, 0.2, -0.4, 0.1, -0.1, 0.3, -0.2];
        let traces: Vec<Vec<f64>> = offsets.iter().map(|&o| vec![base + o]).collect();
        let avg = average_traces(&traces);
        let expected = base + offsets.iter().sum::<f64>() / offsets.len() as f64;
        assert!(approx_eq(avg[0], expected, 0.01));
    }

    #[test]
    fn test_two_point_loss_basic() {
        let trace_db = vec![0.0, -1.0, -2.0, -3.0, -4.0];
        let distance_m = vec![0.0, 100.0, 200.0, 300.0, 400.0];
        let loss = two_point_loss(&trace_db, &distance_m, 0, 4);
        assert!(approx_eq(loss, 4.0, TOLERANCE));
    }

    #[test]
    fn test_two_point_loss_reverse() {
        let trace_db = vec![0.0, -1.0, -2.0, -3.0, -4.0];
        let distance_m = vec![0.0, 100.0, 200.0, 300.0, 400.0];
        let loss = two_point_loss(&trace_db, &distance_m, 4, 0);
        assert!(approx_eq(loss, -4.0, TOLERANCE), "loss={loss}");
    }

    #[test]
    fn test_two_point_loss_out_of_bounds() {
        let trace_db = vec![0.0, -1.0];
        let distance_m = vec![0.0, 100.0];
        let loss = two_point_loss(&trace_db, &distance_m, 0, 10);
        assert!(approx_eq(loss, 0.0, TOLERANCE));
    }

    #[test]
    fn test_fiber_attenuation_uniform() {
        // Generate a trace with known attenuation (round-trip 0.4 dB/km -> one-way 0.2 dB/km)
        let n = 1000;
        let atten_one_way = 0.2; // dB/km
        let atten_roundtrip = 2.0 * atten_one_way / 1000.0; // dB/m
        let distance_m: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let trace_db: Vec<f64> = distance_m.iter().map(|&d| -atten_roundtrip * d).collect();

        let measured = fiber_attenuation(&trace_db, &distance_m, 10, 990);
        assert!(
            approx_eq(measured, atten_one_way, 0.01),
            "measured={measured}, expected={atten_one_way}"
        );
    }

    #[test]
    fn test_fiber_attenuation_invalid_indices() {
        let trace_db = vec![0.0, -0.1, -0.2];
        let distance_m = vec![0.0, 100.0, 200.0];
        // start >= end
        assert!(approx_eq(
            fiber_attenuation(&trace_db, &distance_m, 2, 1),
            0.0,
            TOLERANCE
        ));
    }

    #[test]
    fn test_optical_return_loss_basic() {
        // A flat trace at 0 dB should give ORL = 0
        let trace_db = vec![0.0; 100];
        let orl = optical_return_loss(&trace_db);
        assert!(approx_eq(orl, 0.0, 0.1), "orl={orl}");
    }

    #[test]
    fn test_optical_return_loss_decaying_trace() {
        // A decaying trace has lower average reflected power -> higher ORL
        let trace_db: Vec<f64> = (0..100).map(|i| -(i as f64) * 0.1).collect();
        let orl = optical_return_loss(&trace_db);
        assert!(
            orl > 0.0,
            "ORL should be positive for a decaying trace: orl={orl}"
        );
    }

    #[test]
    fn test_optical_return_loss_empty() {
        let orl = optical_return_loss(&[]);
        assert!(approx_eq(orl, 0.0, TOLERANCE));
    }

    #[test]
    fn test_generate_otdr_trace_length() {
        let (trace, dist) = generate_otdr_trace(1000.0, 0.2, &[], DEFAULT_IOR);
        assert_eq!(trace.len(), dist.len());
        assert_eq!(trace.len(), 1000);
    }

    #[test]
    fn test_generate_otdr_trace_with_events() {
        let events = vec![(200.0, 0.5, -40.0), (600.0, 0.1, -60.0)];
        let (trace, dist) = generate_otdr_trace(1000.0, 0.2, &events, DEFAULT_IOR);
        assert_eq!(trace.len(), dist.len());

        // The trace should not be monotonically decreasing (events cause spikes/drops)
        let mut has_increase = false;
        for i in 1..trace.len() {
            if trace[i] > trace[i - 1] {
                has_increase = true;
                break;
            }
        }
        assert!(
            has_increase,
            "Trace with events should have at least one increase (reflective spike)"
        );
    }

    #[test]
    fn test_detect_events_on_synthetic_trace() {
        let events_input = vec![
            (500.0, 0.5, -40.0),  // connector
            (2000.0, 0.1, -60.0), // splice
        ];
        let (trace, dist) = generate_otdr_trace(5000.0, 0.2, &events_input, DEFAULT_IOR);
        let detected = detect_events(&trace, &dist, 0.3);

        // Should detect at least 1 event (the connector is the most prominent)
        assert!(
            !detected.is_empty(),
            "Should detect at least one event, got none"
        );

        // The first detected event should be near 500 m
        let first = &detected[0];
        assert!(
            (first.distance_m - 500.0).abs() < 50.0,
            "First event at {}, expected ~500 m",
            first.distance_m
        );
    }

    #[test]
    fn test_detect_events_empty_trace() {
        let events = detect_events(&[], &[], 1.0);
        assert!(events.is_empty());
    }

    #[test]
    fn test_analyzer_struct_integration() {
        let config = OtdrConfig {
            wavelength_nm: 1550.0,
            pulse_width_ns: 100.0,
            ior: DEFAULT_IOR,
            sample_rate_hz: 1e9,
            averaging_count: 16,
        };
        let analyzer = OtdrAnalyzer::new(config);

        // Test dead zone calculation via analyzer
        let dz = analyzer.dead_zone_length();
        assert!(dz > 9.0 && dz < 12.0, "dead zone={dz}");

        // Test distance-to-fault via analyzer
        let dist = analyzer.distance_to_fault(100.0);
        let expected = distance_to_fault(100.0, DEFAULT_IOR);
        assert!(approx_eq(dist, expected, TOLERANCE));

        // Generate trace and detect events
        let events_input = vec![(1000.0, 0.5, -35.0)];
        let (trace, dist_axis) = generate_otdr_trace(3000.0, 0.2, &events_input, DEFAULT_IOR);
        let detected = analyzer.detect_events(&trace, &dist_axis, 0.3);
        assert!(!detected.is_empty(), "Analyzer should detect events");

        // Test ORL via analyzer
        let orl = analyzer.optical_return_loss(&trace);
        assert!(orl > 0.0, "ORL should be positive");
    }

    #[test]
    fn test_mean_slice_helper() {
        assert!(approx_eq(mean_slice(&[1.0, 2.0, 3.0]), 2.0, TOLERANCE));
        assert!(approx_eq(mean_slice(&[]), 0.0, TOLERANCE));
        assert!(approx_eq(mean_slice(&[5.0]), 5.0, TOLERANCE));
    }

    #[test]
    fn test_median_value_helper() {
        assert!(approx_eq(median_value(&[1.0, 3.0, 2.0]), 2.0, TOLERANCE));
        assert!(approx_eq(
            median_value(&[1.0, 2.0, 3.0, 4.0]),
            2.5,
            TOLERANCE
        ));
        assert!(approx_eq(median_value(&[]), 0.0, TOLERANCE));
    }
}

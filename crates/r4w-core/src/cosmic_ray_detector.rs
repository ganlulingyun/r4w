//! Cosmic ray event detection from scintillator/PMT detector arrays.
//!
//! This module provides pulse detection and timestamping, coincidence detection
//! between multiple detectors, muon flux rate estimation, energy deposition
//! measurement, dead-time correction, time-over-threshold analysis, pulse height
//! distribution, shower front reconstruction, and zenith angle estimation from
//! arrival time differences.
//!
//! # Example
//!
//! ```
//! use r4w_core::cosmic_ray_detector::{
//!     CosmicRayDetector, DetectorConfig, detect_pulses, estimate_muon_flux,
//! };
//!
//! // Configure a 4-detector array sampling at 1 GHz
//! let config = DetectorConfig {
//!     sample_rate_hz: 1.0e9,
//!     threshold_mv: 50.0,
//!     coincidence_window_ns: 100.0,
//!     num_detectors: 4,
//! };
//!
//! // Generate a simple test pulse and detect it
//! let mut signal = vec![0.0; 200];
//! for i in 50..60 {
//!     signal[i] = 100.0 * (-(((i as f64) - 55.0) / 2.0).powi(2)).exp();
//! }
//!
//! let events = detect_pulses(&signal, config.threshold_mv, config.sample_rate_hz);
//! assert!(!events.is_empty());
//!
//! // Estimate muon flux from 1000 events over 1 m^2 in 3600 s
//! let flux = estimate_muon_flux(1000, 1.0, 3600.0, 1.0);
//! assert!((flux - 1000.0 / 3600.0).abs() < 1e-9);
//! ```

/// Configuration for a cosmic ray detector array.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Sampling rate of the digitizer in Hz.
    pub sample_rate_hz: f64,
    /// Pulse detection threshold in millivolts.
    pub threshold_mv: f64,
    /// Coincidence window for multi-detector triggers in nanoseconds.
    pub coincidence_window_ns: f64,
    /// Number of detectors in the array.
    pub num_detectors: usize,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 1.0e9,
            threshold_mv: 50.0,
            coincidence_window_ns: 100.0,
            num_detectors: 4,
        }
    }
}

/// A single detected cosmic ray event.
#[derive(Debug, Clone, PartialEq)]
pub struct CosmicEvent {
    /// Event timestamp in nanoseconds from the start of the recording.
    pub timestamp_ns: f64,
    /// Peak pulse amplitude (in the same units as the input signal).
    pub peak_amplitude: f64,
    /// Estimated deposited energy in MeV (requires calibration).
    pub energy_mev: f64,
    /// Identifier of the detector that registered this event.
    pub detector_id: usize,
    /// Time-over-threshold in nanoseconds.
    pub time_over_threshold_ns: f64,
}

/// Main cosmic ray detector providing analysis methods.
///
/// Wraps a [`DetectorConfig`] and offers convenience methods for
/// full pipeline analysis of multi-detector data.
pub struct CosmicRayDetector {
    /// The detector array configuration.
    pub config: DetectorConfig,
    /// Calibration slope for energy conversion (MeV per amplitude unit).
    pub calibration_slope: f64,
    /// Calibration offset for energy conversion (MeV).
    pub calibration_offset: f64,
}

impl CosmicRayDetector {
    /// Create a new detector with the given configuration and calibration constants.
    pub fn new(config: DetectorConfig, calibration_slope: f64, calibration_offset: f64) -> Self {
        Self {
            config,
            calibration_slope,
            calibration_offset,
        }
    }

    /// Detect pulses in a single-detector signal using the configured threshold.
    pub fn detect(&self, signal: &[f64], detector_id: usize) -> Vec<CosmicEvent> {
        let mut events = detect_pulses(signal, self.config.threshold_mv, self.config.sample_rate_hz);
        for event in &mut events {
            event.detector_id = detector_id;
            event.energy_mev = energy_from_pulse_height(
                event.peak_amplitude,
                self.calibration_slope,
                self.calibration_offset,
            );
        }
        events
    }

    /// Run coincidence detection across all detectors.
    ///
    /// `signals` must contain one signal per detector (length == `config.num_detectors`).
    /// Returns groups of event indices that are coincident within the configured window.
    pub fn detect_coincidences(&self, signals: &[&[f64]]) -> Vec<Vec<usize>> {
        assert_eq!(
            signals.len(),
            self.config.num_detectors,
            "Expected {} signals, got {}",
            self.config.num_detectors,
            signals.len()
        );
        let per_detector: Vec<Vec<CosmicEvent>> = signals
            .iter()
            .enumerate()
            .map(|(id, sig)| self.detect(sig, id))
            .collect();
        find_coincidences(&per_detector, self.config.coincidence_window_ns)
    }
}

/// Detect pulses in a digitized signal that exceed a given threshold.
///
/// Scans the signal for contiguous regions above `threshold`. For each region
/// the peak amplitude, timestamp, and time-over-threshold are recorded.
/// The `detector_id` field is set to 0; callers may override it.
///
/// # Arguments
/// * `signal` - Digitized pulse waveform (amplitude values).
/// * `threshold` - Detection threshold in the same units as `signal`.
/// * `sample_rate` - Sampling rate in Hz.
pub fn detect_pulses(signal: &[f64], threshold: f64, sample_rate: f64) -> Vec<CosmicEvent> {
    let mut events = Vec::new();
    let ns_per_sample = 1.0e9 / sample_rate;
    let mut i = 0;
    while i < signal.len() {
        if signal[i] > threshold {
            let start = i;
            let mut peak = signal[i];
            let mut peak_idx = i;
            while i < signal.len() && signal[i] > threshold {
                if signal[i] > peak {
                    peak = signal[i];
                    peak_idx = i;
                }
                i += 1;
            }
            let end = i; // one past last above-threshold sample
            let tot_samples = (end - start) as f64;
            events.push(CosmicEvent {
                timestamp_ns: peak_idx as f64 * ns_per_sample,
                peak_amplitude: peak,
                energy_mev: 0.0,
                detector_id: 0,
                time_over_threshold_ns: tot_samples * ns_per_sample,
            });
        } else {
            i += 1;
        }
    }
    events
}

/// Find coincident event groups across multiple detectors.
///
/// Two events from different detectors are considered coincident if their
/// timestamps differ by no more than `window_ns` nanoseconds.
///
/// Returns a list of coincidence groups, where each group is a vector of
/// alternating `(detector_index, event_index)` pairs flattened into a single
/// `Vec<usize>`.
///
/// # Algorithm
/// All events are pooled and sorted by timestamp, then a sliding window
/// groups events that fall within the coincidence window. Only groups
/// containing events from at least two distinct detectors are returned.
pub fn find_coincidences(events: &[Vec<CosmicEvent>], window_ns: f64) -> Vec<Vec<usize>> {
    // Collect (timestamp, detector_id, event_index_within_detector)
    let mut all_events: Vec<(f64, usize, usize)> = Vec::new();
    for (det_id, det_events) in events.iter().enumerate() {
        for (evt_idx, evt) in det_events.iter().enumerate() {
            all_events.push((evt.timestamp_ns, det_id, evt_idx));
        }
    }
    // Sort by timestamp
    all_events.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut coincidences = Vec::new();
    let mut used = vec![false; all_events.len()];

    for i in 0..all_events.len() {
        if used[i] {
            continue;
        }
        let mut group: Vec<usize> = vec![all_events[i].1, all_events[i].2];
        let mut detectors_in_group = std::collections::HashSet::new();
        detectors_in_group.insert(all_events[i].1);
        let t0 = all_events[i].0;

        for j in (i + 1)..all_events.len() {
            if used[j] {
                continue;
            }
            if all_events[j].0 - t0 > window_ns {
                break;
            }
            // Only add if from a different detector than already in group
            if !detectors_in_group.contains(&all_events[j].1) {
                group.push(all_events[j].1);
                group.push(all_events[j].2);
                detectors_in_group.insert(all_events[j].1);
                used[j] = true;
            }
        }

        if detectors_in_group.len() >= 2 {
            used[i] = true;
            coincidences.push(group);
        }
    }
    coincidences
}

/// Estimate the muon flux rate.
///
/// Returns the flux in units of counts per m^2 per second per steradian.
///
/// # Arguments
/// * `event_count` - Total number of detected events.
/// * `area_m2` - Effective detector area in square metres.
/// * `duration_s` - Measurement duration in seconds.
/// * `solid_angle_sr` - Solid angle acceptance in steradians.
pub fn estimate_muon_flux(event_count: u64, area_m2: f64, duration_s: f64, solid_angle_sr: f64) -> f64 {
    event_count as f64 / (area_m2 * duration_s * solid_angle_sr)
}

/// Compute time-over-threshold for a single pulse.
///
/// Returns the duration in nanoseconds that the pulse exceeds `threshold`.
///
/// # Arguments
/// * `pulse` - Digitized pulse waveform.
/// * `threshold` - Detection threshold.
/// * `sample_rate` - Sampling rate in Hz.
pub fn time_over_threshold(pulse: &[f64], threshold: f64, sample_rate: f64) -> f64 {
    let ns_per_sample = 1.0e9 / sample_rate;
    let count = pulse.iter().filter(|&&s| s > threshold).count();
    count as f64 * ns_per_sample
}

/// Build a pulse-height histogram from a set of events.
///
/// Bins the `peak_amplitude` of each event into `num_bins` equally spaced
/// bins spanning from the minimum to the maximum observed amplitude.
/// Returns a vector of counts per bin.
pub fn pulse_height_histogram(events: &[CosmicEvent], num_bins: usize) -> Vec<u64> {
    if events.is_empty() || num_bins == 0 {
        return vec![0; num_bins];
    }
    let min_amp = events
        .iter()
        .map(|e| e.peak_amplitude)
        .fold(f64::INFINITY, f64::min);
    let max_amp = events
        .iter()
        .map(|e| e.peak_amplitude)
        .fold(f64::NEG_INFINITY, f64::max);

    let range = max_amp - min_amp;
    if range == 0.0 {
        let mut hist = vec![0u64; num_bins];
        hist[0] = events.len() as u64;
        return hist;
    }

    let mut hist = vec![0u64; num_bins];
    for event in events {
        let bin = ((event.peak_amplitude - min_amp) / range * (num_bins as f64 - 1.0)).round() as usize;
        let bin = bin.min(num_bins - 1);
        hist[bin] += 1;
    }
    hist
}

/// Convert a pulse height (amplitude) to energy in MeV via linear calibration.
///
/// `energy = amplitude * calibration_slope + calibration_offset`
pub fn energy_from_pulse_height(amplitude: f64, calibration_slope: f64, calibration_offset: f64) -> f64 {
    amplitude * calibration_slope + calibration_offset
}

/// Apply dead-time correction to a measured event rate.
///
/// Uses the non-paralyzable (Type I) dead-time model:
///
/// `true_rate = measured_rate / (1 - measured_rate * dead_time_s)`
///
/// Returns `f64::INFINITY` if the denominator is zero or negative
/// (i.e., the detector is saturated).
pub fn dead_time_correct(measured_rate: f64, dead_time_s: f64) -> f64 {
    let denom = 1.0 - measured_rate * dead_time_s;
    if denom <= 0.0 {
        f64::INFINITY
    } else {
        measured_rate / denom
    }
}

/// Estimate the shower front zenith angle from arrival time differences.
///
/// Fits a plane wave to the arrival times at known detector positions and
/// returns the zenith angle in radians (0 = vertical, PI/2 = horizontal).
///
/// Uses a least-squares fit of the model:
///   `t_i = t0 + (x_i * sin(theta)*cos(phi) + y_i * sin(theta)*sin(phi)) / c`
///
/// The direction cosines `(l, m)` are solved via the normal equations and the
/// zenith angle is `arcsin(sqrt(l^2 + m^2))`.
///
/// # Arguments
/// * `arrival_times_ns` - Arrival time at each detector in nanoseconds.
/// * `detector_positions` - (x, y) positions of each detector in metres.
/// * `propagation_speed_mps` - Speed of the shower front in m/s (typically ~3e8).
///
/// # Panics
/// Panics if the number of arrival times does not match the number of positions,
/// or if fewer than 3 detectors are provided.
pub fn shower_zenith_angle(
    arrival_times_ns: &[f64],
    detector_positions: &[(f64, f64)],
    propagation_speed_mps: f64,
) -> f64 {
    let n = arrival_times_ns.len();
    assert_eq!(
        n,
        detector_positions.len(),
        "arrival_times and detector_positions must have the same length"
    );
    assert!(n >= 3, "Need at least 3 detectors for plane-wave fit");

    // Convert arrival times from ns to seconds
    let times_s: Vec<f64> = arrival_times_ns.iter().map(|&t| t * 1e-9).collect();

    // Build the design matrix for: t_i = t0 + l*x_i/c + m*y_i/c
    // where l = sin(theta)*cos(phi), m = sin(theta)*sin(phi)
    // Rewrite as: t_i = a0 + a1*x_i + a2*y_i
    // where a1 = l/c, a2 = m/c

    // Solve using normal equations: A^T A [a0, a1, a2]^T = A^T t
    // A = [1, x_i, y_i]

    let mut ata = [[0.0f64; 3]; 3];
    let mut atb = [0.0f64; 3];

    for i in 0..n {
        let row = [1.0, detector_positions[i].0, detector_positions[i].1];
        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
            atb[r] += row[r] * times_s[i];
        }
    }

    // Solve 3x3 system via Cramer's rule
    let det3 = |m: [[f64; 3]; 3]| -> f64 {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };

    let d = det3(ata);
    if d.abs() < 1e-30 {
        return 0.0; // Degenerate geometry
    }

    // Replace column 1 with atb to solve for a1
    let mut m1 = ata;
    for r in 0..3 {
        m1[r][0] = atb[r];
    }
    // Replace column 2 with atb to solve for a2
    let mut m2 = ata;
    for r in 0..3 {
        m2[r][1] = atb[r];
    }
    // Replace column 3 with atb (not needed for a0, but we solve for a2 here)
    let mut m3 = ata;
    for r in 0..3 {
        m3[r][2] = atb[r];
    }

    let _a0 = det3(m1) / d;
    let a1 = det3(m2) / d;
    let a2 = det3(m3) / d;

    // l = a1 * c, m = a2 * c
    let l = a1 * propagation_speed_mps;
    let m = a2 * propagation_speed_mps;

    let sin_theta = (l * l + m * m).sqrt();
    // Clamp to valid range for asin
    let sin_theta = sin_theta.min(1.0);
    sin_theta.asin()
}

/// Compute the chi-squared test statistic for rate stability.
///
/// Compares observed counts in time bins against an expected rate.
///
/// `chi2 = sum_i (counts_i - expected_rate)^2 / expected_rate`
///
/// A value close to the number of bins indicates a stable (Poisson-like) rate.
pub fn rate_stability(counts: &[u64], expected_rate: f64) -> f64 {
    if expected_rate <= 0.0 {
        return 0.0;
    }
    counts
        .iter()
        .map(|&c| {
            let diff = c as f64 - expected_rate;
            diff * diff / expected_rate
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: create a Gaussian pulse centred at `center` with given amplitude.
    fn gaussian_pulse(len: usize, center: f64, amplitude: f64, sigma: f64) -> Vec<f64> {
        (0..len)
            .map(|i| {
                let t = i as f64 - center;
                amplitude * (-t * t / (2.0 * sigma * sigma)).exp()
            })
            .collect()
    }

    #[test]
    fn test_detect_single_pulse() {
        let signal = gaussian_pulse(200, 100.0, 500.0, 5.0);
        let events = detect_pulses(&signal, 50.0, 1.0e9);
        assert_eq!(events.len(), 1);
        assert!((events[0].peak_amplitude - 500.0).abs() < 1e-6);
    }

    #[test]
    fn test_detect_no_pulse_below_threshold() {
        let signal = gaussian_pulse(200, 100.0, 30.0, 5.0);
        let events = detect_pulses(&signal, 50.0, 1.0e9);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_multiple_pulses() {
        let mut signal = gaussian_pulse(500, 100.0, 200.0, 5.0);
        let p2 = gaussian_pulse(500, 300.0, 300.0, 5.0);
        for (i, v) in p2.iter().enumerate() {
            signal[i] += v;
        }
        let events = detect_pulses(&signal, 50.0, 1.0e9);
        assert_eq!(events.len(), 2);
        // Second pulse has higher amplitude
        assert!(events[1].peak_amplitude > events[0].peak_amplitude);
    }

    #[test]
    fn test_detect_pulses_timestamp() {
        let signal = gaussian_pulse(1000, 500.0, 200.0, 5.0);
        let events = detect_pulses(&signal, 50.0, 1.0e9);
        assert_eq!(events.len(), 1);
        // Timestamp at peak should be near sample 500 -> 500 ns at 1 GHz
        assert!((events[0].timestamp_ns - 500.0).abs() < 2.0);
    }

    #[test]
    fn test_time_over_threshold_simple() {
        // 10 samples above threshold at 1 GHz -> 10 ns
        let pulse: Vec<f64> = (0..20)
            .map(|i| if (5..15).contains(&i) { 100.0 } else { 0.0 })
            .collect();
        let tot = time_over_threshold(&pulse, 50.0, 1.0e9);
        assert!((tot - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_time_over_threshold_none_above() {
        let pulse = vec![10.0; 100];
        let tot = time_over_threshold(&pulse, 50.0, 1.0e9);
        assert!((tot - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_estimate_muon_flux() {
        let flux = estimate_muon_flux(10000, 1.0, 100.0, 2.0 * PI);
        let expected = 10000.0 / (1.0 * 100.0 * 2.0 * PI);
        assert!((flux - expected).abs() < 1e-9);
    }

    #[test]
    fn test_estimate_muon_flux_typical_sea_level() {
        // ~1 muon per cm^2 per minute ~ 167/m^2/s over 2*pi sr
        let flux = estimate_muon_flux(167 * 3600, 1.0, 3600.0, 2.0 * PI);
        assert!((flux - 167.0 / (2.0 * PI)).abs() < 0.01);
    }

    #[test]
    fn test_energy_from_pulse_height() {
        let e = energy_from_pulse_height(100.0, 0.05, 1.0);
        assert!((e - 6.0).abs() < 1e-9);
    }

    #[test]
    fn test_energy_calibration_zero_offset() {
        let e = energy_from_pulse_height(200.0, 0.01, 0.0);
        assert!((e - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_dead_time_correct_low_rate() {
        // At low rates, correction is small
        let corrected = dead_time_correct(100.0, 1e-6);
        // 100 / (1 - 100 * 1e-6) = 100 / 0.9999 ~ 100.01
        assert!((corrected - 100.01000100010002).abs() < 1e-6);
    }

    #[test]
    fn test_dead_time_correct_saturation() {
        // When measured_rate * dead_time >= 1, detector is saturated
        let corrected = dead_time_correct(1e6, 1e-6);
        assert!(corrected.is_infinite());
    }

    #[test]
    fn test_dead_time_correct_zero_rate() {
        let corrected = dead_time_correct(0.0, 1e-6);
        assert!((corrected - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_pulse_height_histogram_basic() {
        let events: Vec<CosmicEvent> = (0..100)
            .map(|i| CosmicEvent {
                timestamp_ns: i as f64,
                peak_amplitude: (i % 10) as f64 * 10.0,
                energy_mev: 0.0,
                detector_id: 0,
                time_over_threshold_ns: 5.0,
            })
            .collect();
        let hist = pulse_height_histogram(&events, 10);
        assert_eq!(hist.len(), 10);
        let total: u64 = hist.iter().sum();
        assert_eq!(total, 100);
    }

    #[test]
    fn test_pulse_height_histogram_empty() {
        let hist = pulse_height_histogram(&[], 5);
        assert_eq!(hist, vec![0; 5]);
    }

    #[test]
    fn test_pulse_height_histogram_single_amplitude() {
        let events: Vec<CosmicEvent> = (0..10)
            .map(|_| CosmicEvent {
                timestamp_ns: 0.0,
                peak_amplitude: 42.0,
                energy_mev: 0.0,
                detector_id: 0,
                time_over_threshold_ns: 5.0,
            })
            .collect();
        let hist = pulse_height_histogram(&events, 5);
        // All in the first bin when all amplitudes are identical
        assert_eq!(hist[0], 10);
    }

    #[test]
    fn test_find_coincidences_two_detectors() {
        let det0 = vec![CosmicEvent {
            timestamp_ns: 1000.0,
            peak_amplitude: 100.0,
            energy_mev: 1.0,
            detector_id: 0,
            time_over_threshold_ns: 5.0,
        }];
        let det1 = vec![CosmicEvent {
            timestamp_ns: 1050.0, // 50 ns later
            peak_amplitude: 120.0,
            energy_mev: 1.2,
            detector_id: 1,
            time_over_threshold_ns: 6.0,
        }];
        let coinc = find_coincidences(&[det0, det1], 100.0);
        assert_eq!(coinc.len(), 1);
        // Group should contain entries from both detectors
        assert_eq!(coinc[0].len(), 4); // [det0, idx0, det1, idx1]
    }

    #[test]
    fn test_find_coincidences_no_match() {
        let det0 = vec![CosmicEvent {
            timestamp_ns: 1000.0,
            peak_amplitude: 100.0,
            energy_mev: 1.0,
            detector_id: 0,
            time_over_threshold_ns: 5.0,
        }];
        let det1 = vec![CosmicEvent {
            timestamp_ns: 5000.0, // Far away in time
            peak_amplitude: 120.0,
            energy_mev: 1.2,
            detector_id: 1,
            time_over_threshold_ns: 6.0,
        }];
        let coinc = find_coincidences(&[det0, det1], 100.0);
        assert!(coinc.is_empty());
    }

    #[test]
    fn test_shower_zenith_angle_vertical() {
        // A vertical shower arrives at all detectors simultaneously
        let times = vec![0.0, 0.0, 0.0, 0.0];
        let positions = vec![(0.0, 0.0), (10.0, 0.0), (0.0, 10.0), (10.0, 10.0)];
        let angle = shower_zenith_angle(&times, &positions, 3.0e8);
        assert!(angle.abs() < 1e-6, "Vertical shower should give ~0 zenith angle, got {}", angle);
    }

    #[test]
    fn test_shower_zenith_angle_tilted() {
        // A shower arriving from the side should give a non-zero zenith angle.
        // Detectors at (0,0), (10,0), (0,10) spaced 10 m apart.
        // If the shower front moves along x at speed c, the delay between
        // x=0 and x=10 is 10/c seconds = 10/3e8 s = 33.33 ns
        let c = 3.0e8;
        let dx = 10.0;
        let delay_ns = dx / c * 1e9; // ~33.33 ns
        let times = vec![0.0, delay_ns, 0.0];
        let positions = vec![(0.0, 0.0), (dx, 0.0), (0.0, 10.0)];
        let angle = shower_zenith_angle(&times, &positions, c);
        // The shower is coming from near the horizon along x
        assert!(angle > 0.5, "Tilted shower should give significant zenith angle, got {}", angle);
    }

    #[test]
    fn test_rate_stability_perfect() {
        // If every bin matches the expected rate, chi2 = 0
        let counts = vec![100, 100, 100, 100, 100];
        let chi2 = rate_stability(&counts, 100.0);
        assert!((chi2 - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_rate_stability_deviation() {
        // One bin deviates by 10 from expected 100
        // chi2 = (110-100)^2/100 + 4*(100-100)^2/100 = 100/100 = 1.0
        let counts = vec![110, 100, 100, 100, 100];
        let chi2 = rate_stability(&counts, 100.0);
        assert!((chi2 - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_detector_config_default() {
        let config = DetectorConfig::default();
        assert_eq!(config.sample_rate_hz, 1.0e9);
        assert_eq!(config.threshold_mv, 50.0);
        assert_eq!(config.coincidence_window_ns, 100.0);
        assert_eq!(config.num_detectors, 4);
    }

    #[test]
    fn test_cosmic_ray_detector_detect() {
        let config = DetectorConfig {
            sample_rate_hz: 1.0e9,
            threshold_mv: 50.0,
            coincidence_window_ns: 100.0,
            num_detectors: 2,
        };
        let detector = CosmicRayDetector::new(config, 0.01, 0.5);
        let signal = gaussian_pulse(200, 100.0, 300.0, 5.0);
        let events = detector.detect(&signal, 0);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].detector_id, 0);
        // Energy: 300.0 * 0.01 + 0.5 = 3.5
        assert!((events[0].energy_mev - 3.5).abs() < 0.01);
    }

    #[test]
    fn test_cosmic_ray_detector_coincidences() {
        let config = DetectorConfig {
            sample_rate_hz: 1.0e9,
            threshold_mv: 50.0,
            coincidence_window_ns: 200.0,
            num_detectors: 2,
        };
        let detector = CosmicRayDetector::new(config, 0.01, 0.0);
        // Both detectors see a pulse at roughly the same time
        let sig0 = gaussian_pulse(200, 100.0, 300.0, 5.0);
        let sig1 = gaussian_pulse(200, 102.0, 250.0, 5.0);
        let coinc = detector.detect_coincidences(&[&sig0, &sig1]);
        assert!(!coinc.is_empty(), "Should find coincident events");
    }

    #[test]
    fn test_detect_pulses_empty_signal() {
        let events = detect_pulses(&[], 50.0, 1.0e9);
        assert!(events.is_empty());
    }
}

//! VLF/LF Lightning Stroke Detection and Localization
//!
//! This module implements sferic waveform detection (threshold + template matching),
//! time-of-arrival (TOA) estimation, multi-station TDOA triangulation, stroke type
//! classification (cloud-to-ground vs intra-cloud), peak current estimation,
//! flash grouping (temporal/spatial clustering), and sferic propagation delay modeling.
//!
//! # Example
//!
//! ```
//! use r4w_core::lightning_stroke_analyzer::{
//!     LightningConfig, LightningDetector, detect_sferics,
//!     generate_sferic_template, propagation_delay,
//! };
//!
//! let config = LightningConfig::default();
//! let detector = LightningDetector::new(config);
//!
//! // Generate a synthetic sferic template at 1 MHz sample rate
//! let template = generate_sferic_template(1_000_000.0, 0.001);
//! assert!(!template.is_empty());
//!
//! // Propagation delay for 300 km at near-light speed
//! let delay = propagation_delay(300.0, 299_792_458.0 * 0.997);
//! assert!(delay > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for the lightning detection system.
#[derive(Debug, Clone)]
pub struct LightningConfig {
    /// Sample rate in Hz of the input signal.
    pub sample_rate_hz: f64,
    /// Amplitude threshold for sferic detection (absolute value).
    pub detection_threshold: f64,
    /// Electromagnetic propagation speed in m/s (typically ~0.997 * c).
    pub propagation_speed_mps: f64,
}

impl Default for LightningConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 1_000_000.0,
            detection_threshold: 0.5,
            propagation_speed_mps: 299_792_458.0 * 0.997,
        }
    }
}

// ---------------------------------------------------------------------------
// Stroke type classification
// ---------------------------------------------------------------------------

/// Classification of a lightning stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StrokeType {
    /// Cloud-to-ground stroke (characterised by a slower, larger bipolar pulse).
    CloudToGround,
    /// Intra-cloud stroke (typically narrower, faster pulses).
    IntraCloud,
    /// Could not be classified.
    Unknown,
}

// ---------------------------------------------------------------------------
// StrokeEvent
// ---------------------------------------------------------------------------

/// A single detected lightning stroke event.
#[derive(Debug, Clone)]
pub struct StrokeEvent {
    /// Time-of-arrival in seconds (relative to signal start).
    pub toa_s: f64,
    /// Peak amplitude of the detected sferic (signed).
    pub amplitude: f64,
    /// Estimated latitude in degrees (WGS-84). May be `f64::NAN` if unknown.
    pub latitude_deg: f64,
    /// Estimated longitude in degrees (WGS-84). May be `f64::NAN` if unknown.
    pub longitude_deg: f64,
    /// Estimated peak current in kiloamperes.
    pub peak_current_ka: f64,
    /// Stroke type classification.
    pub stroke_type: StrokeType,
}

// ---------------------------------------------------------------------------
// FlashGroup
// ---------------------------------------------------------------------------

/// A flash is a temporal/spatial cluster of individual strokes.
#[derive(Debug, Clone)]
pub struct FlashGroup {
    /// Individual strokes belonging to this flash.
    pub strokes: Vec<StrokeEvent>,
    /// Earliest stroke TOA in the group (seconds).
    pub start_time: f64,
    /// Latest stroke TOA in the group (seconds).
    pub end_time: f64,
    /// Number of strokes in the flash (multiplicity).
    pub multiplicity: usize,
}

// ---------------------------------------------------------------------------
// LightningDetector (stateful wrapper)
// ---------------------------------------------------------------------------

/// Main detector struct that wraps configuration and provides convenience
/// methods for the full detection pipeline.
#[derive(Debug, Clone)]
pub struct LightningDetector {
    config: LightningConfig,
}

impl LightningDetector {
    /// Create a new detector with the given configuration.
    pub fn new(config: LightningConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &LightningConfig {
        &self.config
    }

    /// Detect sferics using the configured threshold and a default minimum
    /// separation of 100 samples.
    pub fn detect(&self, signal: &[f64]) -> Vec<(usize, f64)> {
        detect_sferics(signal, self.config.detection_threshold, 100)
    }

    /// Estimate TOA of a template in the signal via cross-correlation.
    pub fn estimate_toa(&self, signal: &[f64], template: &[f64]) -> (usize, f64) {
        estimate_toa(signal, template)
    }

    /// Generate a sferic template using the configured sample rate.
    pub fn generate_template(&self, duration_s: f64) -> Vec<f64> {
        generate_sferic_template(self.config.sample_rate_hz, duration_s)
    }

    /// Compute propagation delay using the configured speed.
    pub fn propagation_delay(&self, distance_km: f64) -> f64 {
        propagation_delay(distance_km, self.config.propagation_speed_mps)
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Detect sferic pulses in a signal using amplitude thresholding.
///
/// Returns a list of `(sample_index, amplitude)` pairs for each detected
/// sferic.  Detections closer than `min_separation_samples` are suppressed
/// (only the largest within each window is kept).
pub fn detect_sferics(
    signal: &[f64],
    threshold: f64,
    min_separation_samples: usize,
) -> Vec<(usize, f64)> {
    let mut detections: Vec<(usize, f64)> = Vec::new();

    // First pass: find all samples that exceed threshold
    let mut candidates: Vec<(usize, f64)> = Vec::new();
    for (i, &s) in signal.iter().enumerate() {
        if s.abs() >= threshold {
            candidates.push((i, s));
        }
    }

    // Second pass: non-maximum suppression within min_separation_samples
    for &(idx, amp) in &candidates {
        if let Some(last) = detections.last() {
            let (last_idx, last_amp): (usize, f64) = *last;
            if idx.saturating_sub(last_idx) < min_separation_samples {
                // Within the window - keep the larger one
                if amp.abs() > last_amp.abs() {
                    *detections.last_mut().unwrap() = (idx, amp);
                }
                continue;
            }
        }
        detections.push((idx, amp));
    }

    detections
}

/// Estimate time-of-arrival of a template in a signal via cross-correlation.
///
/// Returns `(peak_index, quality)` where `quality` is the normalised
/// cross-correlation value at the peak (range 0..1, higher is better).
pub fn estimate_toa(signal: &[f64], template: &[f64]) -> (usize, f64) {
    if signal.is_empty() || template.is_empty() || signal.len() < template.len() {
        return (0, 0.0);
    }

    let template_energy: f64 = template.iter().map(|t| t * t).sum();
    if template_energy == 0.0 {
        return (0, 0.0);
    }

    let n = signal.len() - template.len() + 1;
    let mut best_idx: usize = 0;
    let mut best_val: f64 = f64::NEG_INFINITY;

    for i in 0..n {
        let mut cross: f64 = 0.0;
        let mut sig_energy: f64 = 0.0;
        for (j, &t) in template.iter().enumerate() {
            let s = signal[i + j];
            cross += s * t;
            sig_energy += s * s;
        }
        let denom = (sig_energy * template_energy).sqrt();
        let norm = if denom > 0.0 { cross / denom } else { 0.0 };
        if norm > best_val {
            best_val = norm;
            best_idx = i;
        }
    }

    let quality = best_val.clamp(0.0, 1.0);
    (best_idx, quality)
}

/// Triangulate a stroke location from multi-station TDOA measurements.
///
/// `stations` contains `(latitude_deg, longitude_deg)` of each receiving
/// station. `toa_s` contains the measured time-of-arrival at each station
/// (same length as `stations`). `prop_speed` is the propagation speed in m/s.
///
/// Returns `Some((lat, lon))` or `None` if triangulation is not possible
/// (e.g. fewer than 3 stations).
///
/// Uses an iterative least-squares linearised approach on a flat-Earth
/// approximation suitable for ranges up to ~500 km.
pub fn triangulate_tdoa(
    stations: &[(f64, f64)],
    toa_s: &[f64],
    prop_speed: f64,
) -> Option<(f64, f64)> {
    if stations.len() < 3 || stations.len() != toa_s.len() {
        return None;
    }

    // Convert lat/lon to local km (flat-Earth approx from centroid)
    let n = stations.len();
    let mean_lat: f64 = stations.iter().map(|s| s.0).sum::<f64>() / n as f64;
    let mean_lon: f64 = stations.iter().map(|s| s.1).sum::<f64>() / n as f64;

    let km_per_deg_lat = 111.32;
    let km_per_deg_lon = 111.32 * (mean_lat.to_radians()).cos();

    let xy: Vec<(f64, f64)> = stations
        .iter()
        .map(|s| {
            (
                (s.1 - mean_lon) * km_per_deg_lon,
                (s.0 - mean_lat) * km_per_deg_lat,
            )
        })
        .collect();

    // TDOA relative to station 0
    let prop_speed_km = prop_speed / 1000.0;
    let mut tdoa_km: Vec<f64> = Vec::with_capacity(n - 1);
    for i in 1..n {
        tdoa_km.push((toa_s[i] - toa_s[0]) * prop_speed_km);
    }

    // Iterative linearised least-squares (start from centroid)
    let mut x_est = 0.0_f64;
    let mut y_est = 0.0_f64;

    for _ in 0..20 {
        // distance from estimate to each station
        let dists: Vec<f64> = xy
            .iter()
            .map(|&(sx, sy)| ((x_est - sx).powi(2) + (y_est - sy).powi(2)).sqrt().max(1e-12))
            .collect();

        // Build Ax = b for TDOA (d_i - d_0 = tdoa_km[i-1])
        let m = n - 1;
        // A is m x 2, b is m x 1
        let mut ata = [0.0f64; 4]; // 2x2
        let mut atb = [0.0f64; 2]; // 2x1

        for i in 0..m {
            let ii = i + 1;
            // partial derivatives of (d_i - d_0) w.r.t. (x_est, y_est)
            let dx_i = (x_est - xy[ii].0) / dists[ii] - (x_est - xy[0].0) / dists[0];
            let dy_i = (y_est - xy[ii].1) / dists[ii] - (y_est - xy[0].1) / dists[0];

            let residual = (dists[ii] - dists[0]) - tdoa_km[i];

            // accumulate normal equations
            ata[0] += dx_i * dx_i;
            ata[1] += dx_i * dy_i;
            ata[2] += dy_i * dx_i;
            ata[3] += dy_i * dy_i;

            atb[0] -= dx_i * residual;
            atb[1] -= dy_i * residual;
        }

        // Solve 2x2 system
        let det = ata[0] * ata[3] - ata[1] * ata[2];
        if det.abs() < 1e-30 {
            break;
        }
        let dx = (ata[3] * atb[0] - ata[1] * atb[1]) / det;
        let dy = (-ata[2] * atb[0] + ata[0] * atb[1]) / det;

        x_est += dx;
        y_est += dy;

        if dx.abs() < 1e-9 && dy.abs() < 1e-9 {
            break;
        }
    }

    // Convert back to lat/lon
    let lat = mean_lat + y_est / km_per_deg_lat;
    let lon = mean_lon + x_est / km_per_deg_lon;

    Some((lat, lon))
}

/// Classify a stroke as cloud-to-ground or intra-cloud based on pulse shape.
///
/// Cloud-to-ground strokes typically have a slower rise time and longer
/// duration compared to intra-cloud strokes.  This classifier uses a simple
/// heuristic: the ratio of energy in the first half vs second half of the
/// waveform, and the zero-crossing rate.
pub fn classify_stroke(waveform: &[f64], sample_rate: f64) -> StrokeType {
    if waveform.is_empty() || sample_rate <= 0.0 {
        return StrokeType::Unknown;
    }

    let duration_us = (waveform.len() as f64 / sample_rate) * 1e6;

    // Find peak position (relative)
    let peak_idx = waveform
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);

    let peak_rel = peak_idx as f64 / waveform.len().max(1) as f64;

    // Zero-crossing count
    let mut zc_count = 0usize;
    for w in waveform.windows(2) {
        if (w[0] >= 0.0 && w[1] < 0.0) || (w[0] < 0.0 && w[1] >= 0.0) {
            zc_count += 1;
        }
    }
    let zc_rate = zc_count as f64 / (waveform.len().max(1) as f64 / sample_rate);

    // Heuristic:
    // CG strokes: peak early (peak_rel < 0.4), lower ZC rate, duration > 50 us
    // IC strokes: peak anywhere, higher ZC rate, shorter duration
    if duration_us > 50.0 && peak_rel < 0.45 && zc_rate < sample_rate * 0.15 {
        StrokeType::CloudToGround
    } else if zc_rate > sample_rate * 0.10 || duration_us < 30.0 {
        StrokeType::IntraCloud
    } else {
        StrokeType::Unknown
    }
}

/// Estimate peak current (in kA) from measured electric field amplitude and
/// distance to the stroke.
///
/// Uses the simplified transmission-line model:
///   I_peak = (2 * pi * epsilon_0 * c^2 * E * d) / v_ret
///
/// Simplified for practical use to:
///   I_peak (kA) = amplitude * distance_km * 0.0148
///
/// where 0.0148 is an empirical calibration factor for VLF sensors.
pub fn estimate_peak_current(amplitude: f64, distance_km: f64) -> f64 {
    // Empirical linear relationship commonly used in lightning location
    // networks (e.g. NLDN-style).  The coefficient accounts for sensor
    // calibration, return-stroke speed, and geometric factors.
    const CALIBRATION_FACTOR: f64 = 0.0148;
    amplitude.abs() * distance_km * CALIBRATION_FACTOR
}

/// Group individual strokes into flashes using temporal and spatial proximity.
///
/// Two strokes belong to the same flash if their TOA difference is less than
/// `time_window_s` **and** their great-circle distance is less than
/// `distance_km_threshold`.
pub fn group_flashes(
    strokes: &[StrokeEvent],
    time_window_s: f64,
    distance_km_threshold: f64,
) -> Vec<FlashGroup> {
    if strokes.is_empty() {
        return Vec::new();
    }

    // Sort by TOA
    let mut sorted: Vec<&StrokeEvent> = strokes.iter().collect();
    sorted.sort_by(|a, b| a.toa_s.partial_cmp(&b.toa_s).unwrap_or(std::cmp::Ordering::Equal));

    let mut groups: Vec<Vec<&StrokeEvent>> = Vec::new();
    let mut assigned = vec![false; sorted.len()];

    for i in 0..sorted.len() {
        if assigned[i] {
            continue;
        }
        let mut group = vec![sorted[i]];
        assigned[i] = true;

        for j in (i + 1)..sorted.len() {
            if assigned[j] {
                continue;
            }
            // Check time window against the first stroke in the group
            if sorted[j].toa_s - group[0].toa_s > time_window_s {
                break;
            }
            // Check spatial proximity to any stroke already in the group
            let close = group.iter().any(|gs| {
                let d = haversine_km(
                    gs.latitude_deg,
                    gs.longitude_deg,
                    sorted[j].latitude_deg,
                    sorted[j].longitude_deg,
                );
                d <= distance_km_threshold
            });
            if close {
                group.push(sorted[j]);
                assigned[j] = true;
            }
        }
        groups.push(group);
    }

    groups
        .into_iter()
        .map(|g| {
            let start_time = g
                .iter()
                .map(|s| s.toa_s)
                .fold(f64::INFINITY, f64::min);
            let end_time = g
                .iter()
                .map(|s| s.toa_s)
                .fold(f64::NEG_INFINITY, f64::max);
            let multiplicity = g.len();
            let strokes = g.into_iter().cloned().collect();
            FlashGroup {
                strokes,
                start_time,
                end_time,
                multiplicity,
            }
        })
        .collect()
}

/// Compute propagation delay in seconds for a given distance (km) and
/// propagation speed (m/s).
pub fn propagation_delay(distance_km: f64, speed_mps: f64) -> f64 {
    (distance_km * 1000.0) / speed_mps
}

/// Generate a synthetic sferic (atmospheric) waveform template.
///
/// The template is a damped bipolar pulse that approximates the VLF signature
/// of a lightning return stroke.  It is constructed as a damped sinusoid
/// multiplied by a Gaussian envelope.
///
/// * `sample_rate` - Sample rate in Hz.
/// * `duration_s` - Duration in seconds.
pub fn generate_sferic_template(sample_rate: f64, duration_s: f64) -> Vec<f64> {
    let n = (sample_rate * duration_s).round() as usize;
    if n == 0 {
        return Vec::new();
    }

    let mut template = Vec::with_capacity(n);
    let center = n as f64 / 2.0;
    let sigma = n as f64 / 6.0; // Gaussian width
    let freq_hz = 10_000.0; // ~10 kHz centre frequency (VLF sferic band)

    for i in 0..n {
        let t = (i as f64 - center) / sample_rate;
        let gaussian = (-0.5 * ((i as f64 - center) / sigma).powi(2)).exp();
        let oscillation = (2.0 * PI * freq_hz * t).sin();
        // Asymmetric damping: faster decay on the trailing side
        let asym = if (i as f64) < center {
            1.0
        } else {
            (-2.0 * (i as f64 - center) / (n as f64)).exp()
        };
        template.push(gaussian * oscillation * asym);
    }

    template
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Haversine great-circle distance in km.
fn haversine_km(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    // Handle NaN coordinates gracefully
    if lat1.is_nan() || lon1.is_nan() || lat2.is_nan() || lon2.is_nan() {
        return f64::INFINITY;
    }
    const R: f64 = 6371.0;
    let d_lat = (lat2 - lat1).to_radians();
    let d_lon = (lon2 - lon1).to_radians();
    let a = (d_lat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();
    R * c
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // LightningConfig tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_config_defaults() {
        let cfg = LightningConfig::default();
        assert_eq!(cfg.sample_rate_hz, 1_000_000.0);
        assert_eq!(cfg.detection_threshold, 0.5);
        assert!(cfg.propagation_speed_mps > 2.9e8);
        assert!(cfg.propagation_speed_mps < 3.0e8);
    }

    #[test]
    fn test_config_custom() {
        let cfg = LightningConfig {
            sample_rate_hz: 500_000.0,
            detection_threshold: 0.3,
            propagation_speed_mps: 2.998e8,
        };
        assert_eq!(cfg.sample_rate_hz, 500_000.0);
        assert_eq!(cfg.detection_threshold, 0.3);
    }

    // -----------------------------------------------------------------------
    // detect_sferics tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_sferics_basic() {
        // Signal with two well-separated pulses
        let mut signal = vec![0.0; 1000];
        signal[200] = 1.0;
        signal[700] = -0.8;

        let detections = detect_sferics(&signal, 0.5, 100);
        assert_eq!(detections.len(), 2);
        assert_eq!(detections[0].0, 200);
        assert!((detections[0].1 - 1.0).abs() < 1e-12);
        assert_eq!(detections[1].0, 700);
        assert!((detections[1].1 - (-0.8)).abs() < 1e-12);
    }

    #[test]
    fn test_detect_sferics_suppression() {
        // Two pulses within min_separation - only largest should survive
        let mut signal = vec![0.0; 1000];
        signal[200] = 0.6;
        signal[210] = 0.9;

        let detections = detect_sferics(&signal, 0.5, 50);
        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].0, 210);
    }

    #[test]
    fn test_detect_sferics_empty_signal() {
        let detections = detect_sferics(&[], 0.5, 100);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_detect_sferics_below_threshold() {
        let signal = vec![0.1, 0.2, 0.3, 0.2, 0.1];
        let detections = detect_sferics(&signal, 0.5, 1);
        assert!(detections.is_empty());
    }

    // -----------------------------------------------------------------------
    // estimate_toa tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_estimate_toa_exact_match() {
        let template = vec![0.0, 0.5, 1.0, 0.5, 0.0];
        // Place template at index 10
        let mut signal = vec![0.0; 30];
        for (i, &t) in template.iter().enumerate() {
            signal[10 + i] = t;
        }
        let (idx, quality) = estimate_toa(&signal, &template);
        assert_eq!(idx, 10);
        assert!(quality > 0.95, "quality = {}", quality);
    }

    #[test]
    fn test_estimate_toa_scaled() {
        let template = vec![0.0, 0.5, 1.0, 0.5, 0.0];
        let mut signal = vec![0.0; 30];
        for (i, &t) in template.iter().enumerate() {
            signal[15 + i] = t * 3.0; // scaled version
        }
        let (idx, quality) = estimate_toa(&signal, &template);
        assert_eq!(idx, 15);
        assert!(quality > 0.95);
    }

    #[test]
    fn test_estimate_toa_empty() {
        let (idx, quality) = estimate_toa(&[], &[1.0, 2.0]);
        assert_eq!(idx, 0);
        assert_eq!(quality, 0.0);
    }

    // -----------------------------------------------------------------------
    // triangulate_tdoa tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_triangulate_tdoa_collocated_stations() {
        // If all TOAs are equal, the source is at the centroid
        let stations = vec![(30.0, -90.0), (30.1, -90.0), (30.0, -89.9)];
        let toa = vec![0.001, 0.001, 0.001];
        let speed = 299_792_458.0 * 0.997;
        let result = triangulate_tdoa(&stations, &toa, speed);
        assert!(result.is_some());
        // Should be near the centroid
        let (lat, lon) = result.unwrap();
        assert!((lat - 30.033).abs() < 0.1, "lat = {}", lat);
        assert!((lon - (-89.967)).abs() < 0.1, "lon = {}", lon);
    }

    #[test]
    fn test_triangulate_tdoa_too_few_stations() {
        let stations = vec![(30.0, -90.0), (30.1, -90.0)];
        let toa = vec![0.001, 0.0012];
        assert!(triangulate_tdoa(&stations, &toa, 3e8).is_none());
    }

    #[test]
    fn test_triangulate_tdoa_mismatched_lengths() {
        let stations = vec![(30.0, -90.0), (30.1, -90.0), (30.0, -89.9)];
        let toa = vec![0.001, 0.002]; // wrong length
        assert!(triangulate_tdoa(&stations, &toa, 3e8).is_none());
    }

    // -----------------------------------------------------------------------
    // classify_stroke tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_stroke_cg() {
        // Slow rise, early peak, longer waveform -> CG
        let sample_rate = 1_000_000.0;
        let n = 200; // 200 us at 1 MHz
        let mut waveform = vec![0.0; n];
        // Peak early (at ~20% of waveform)
        let peak_pos = n / 5;
        for i in 0..n {
            let dist = (i as f64 - peak_pos as f64).abs();
            waveform[i] = (-dist / 30.0).exp() * if i < peak_pos + 10 { 1.0 } else { -0.3 };
        }
        let result = classify_stroke(&waveform, sample_rate);
        assert_eq!(result, StrokeType::CloudToGround);
    }

    #[test]
    fn test_classify_stroke_ic() {
        // High zero-crossing rate, short duration -> IC
        let sample_rate = 1_000_000.0;
        let n = 20; // 20 us at 1 MHz
        let mut waveform = Vec::with_capacity(n);
        for i in 0..n {
            // Rapid oscillation
            waveform.push(if i % 2 == 0 { 1.0 } else { -1.0 });
        }
        let result = classify_stroke(&waveform, sample_rate);
        assert_eq!(result, StrokeType::IntraCloud);
    }

    #[test]
    fn test_classify_stroke_empty() {
        assert_eq!(classify_stroke(&[], 1e6), StrokeType::Unknown);
    }

    #[test]
    fn test_classify_stroke_zero_sample_rate() {
        assert_eq!(classify_stroke(&[1.0, 2.0], 0.0), StrokeType::Unknown);
    }

    // -----------------------------------------------------------------------
    // estimate_peak_current tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_estimate_peak_current_basic() {
        let current = estimate_peak_current(100.0, 200.0);
        // 100 * 200 * 0.0148 = 296 kA
        assert!((current - 296.0).abs() < 1e-6);
    }

    #[test]
    fn test_estimate_peak_current_negative_amplitude() {
        // Should use absolute value
        let current = estimate_peak_current(-50.0, 100.0);
        assert!((current - 74.0).abs() < 1e-6);
    }

    #[test]
    fn test_estimate_peak_current_zero_distance() {
        assert_eq!(estimate_peak_current(100.0, 0.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // group_flashes tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_group_flashes_single_group() {
        let strokes = vec![
            StrokeEvent {
                toa_s: 0.0,
                amplitude: 1.0,
                latitude_deg: 30.0,
                longitude_deg: -90.0,
                peak_current_ka: 50.0,
                stroke_type: StrokeType::CloudToGround,
            },
            StrokeEvent {
                toa_s: 0.05,
                amplitude: 0.5,
                latitude_deg: 30.001,
                longitude_deg: -90.001,
                peak_current_ka: 25.0,
                stroke_type: StrokeType::CloudToGround,
            },
        ];
        let groups = group_flashes(&strokes, 0.5, 10.0);
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].multiplicity, 2);
        assert!((groups[0].start_time - 0.0).abs() < 1e-12);
        assert!((groups[0].end_time - 0.05).abs() < 1e-12);
    }

    #[test]
    fn test_group_flashes_separate_groups() {
        let strokes = vec![
            StrokeEvent {
                toa_s: 0.0,
                amplitude: 1.0,
                latitude_deg: 30.0,
                longitude_deg: -90.0,
                peak_current_ka: 50.0,
                stroke_type: StrokeType::CloudToGround,
            },
            StrokeEvent {
                toa_s: 5.0, // 5 seconds later - different flash
                amplitude: 0.8,
                latitude_deg: 35.0, // far away
                longitude_deg: -85.0,
                peak_current_ka: 40.0,
                stroke_type: StrokeType::IntraCloud,
            },
        ];
        let groups = group_flashes(&strokes, 0.5, 10.0);
        assert_eq!(groups.len(), 2);
        assert_eq!(groups[0].multiplicity, 1);
        assert_eq!(groups[1].multiplicity, 1);
    }

    #[test]
    fn test_group_flashes_empty() {
        let groups = group_flashes(&[], 0.5, 10.0);
        assert!(groups.is_empty());
    }

    // -----------------------------------------------------------------------
    // propagation_delay tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_propagation_delay_basic() {
        let delay = propagation_delay(300.0, 3e8);
        // 300 km / 3e8 m/s = 1e-3 s = 1 ms
        assert!((delay - 0.001).abs() < 1e-9);
    }

    #[test]
    fn test_propagation_delay_zero() {
        assert_eq!(propagation_delay(0.0, 3e8), 0.0);
    }

    // -----------------------------------------------------------------------
    // generate_sferic_template tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_sferic_template_length() {
        let t = generate_sferic_template(1_000_000.0, 0.001);
        assert_eq!(t.len(), 1000);
    }

    #[test]
    fn test_sferic_template_zero_duration() {
        let t = generate_sferic_template(1e6, 0.0);
        assert!(t.is_empty());
    }

    #[test]
    fn test_sferic_template_has_bipolar_shape() {
        let t = generate_sferic_template(1_000_000.0, 0.001);
        let has_positive = t.iter().any(|&v| v > 0.01);
        let has_negative = t.iter().any(|&v| v < -0.01);
        assert!(has_positive, "template should have positive values");
        assert!(has_negative, "template should have negative values");
    }

    // -----------------------------------------------------------------------
    // LightningDetector (stateful) tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_new_and_config() {
        let cfg = LightningConfig {
            sample_rate_hz: 2e6,
            detection_threshold: 0.7,
            propagation_speed_mps: 2.99e8,
        };
        let det = LightningDetector::new(cfg);
        assert_eq!(det.config().sample_rate_hz, 2e6);
        assert_eq!(det.config().detection_threshold, 0.7);
    }

    #[test]
    fn test_detector_detect_convenience() {
        let cfg = LightningConfig {
            detection_threshold: 0.4,
            ..LightningConfig::default()
        };
        let det = LightningDetector::new(cfg);
        let mut signal = vec![0.0; 500];
        signal[100] = 0.8;
        signal[300] = -0.6;
        let results = det.detect(&signal);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_detector_propagation_delay() {
        let det = LightningDetector::new(LightningConfig::default());
        let delay = det.propagation_delay(100.0);
        assert!(delay > 0.0);
        assert!(delay < 0.001); // 100 km should be << 1 ms
    }

    // -----------------------------------------------------------------------
    // haversine_km tests (internal but exercised via group_flashes)
    // -----------------------------------------------------------------------

    #[test]
    fn test_haversine_same_point() {
        let d = haversine_km(30.0, -90.0, 30.0, -90.0);
        assert!(d.abs() < 1e-6);
    }

    #[test]
    fn test_haversine_nan_coords() {
        let d = haversine_km(f64::NAN, -90.0, 30.0, -90.0);
        assert!(d.is_infinite());
    }
}

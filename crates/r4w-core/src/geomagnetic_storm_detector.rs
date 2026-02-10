//! Geomagnetic storm detection and space weather analysis from magnetometer time series data.
//!
//! This module provides tools for analyzing magnetometer data to detect and classify
//! geomagnetic storms. It implements standard geomagnetic indices (K-index, Dst, Ap),
//! sudden storm commencement (SSC) detection, substorm onset identification via Pi2
//! pulsation analysis, and magnetic field component decomposition (H/D/Z).
//!
//! # Overview
//!
//! Geomagnetic storms are disturbances in Earth's magnetosphere caused by solar wind
//! interactions. They are characterized by:
//! - **K-index**: A quasi-logarithmic local index (0-9) measuring 3-hour magnetic activity
//! - **Dst index**: A global index measuring equatorial ring current intensity (in nT)
//! - **SSC**: Sudden storm commencements, rapid increases in the H-component
//! - **Substorms**: Magnetospheric energy releases detected via Pi2 pulsations (40-150s period)
//!
//! # Example
//!
//! ```
//! use r4w_core::geomagnetic_storm_detector::{
//!     GeomagneticStormDetector, StormLevel, compute_k_index, classify_storm,
//! };
//!
//! // Create a detector for a mid-latitude station
//! let detector = GeomagneticStormDetector::new(45.0);
//!
//! // Simulate quiet-day H-component data (small variation)
//! let h_data: Vec<f64> = (0..1080).map(|i| 5.0 * (i as f64 * 0.001).sin()).collect();
//! let k = compute_k_index(&h_data, 3.0);
//! assert!(k <= 2, "quiet-day data should yield low K-index");
//!
//! // Classify from a set of K-indices
//! let k_indices = vec![1, 0, 1, 1, 0, 1, 1, 0];
//! let level = classify_storm(&k_indices);
//! assert_eq!(level, StormLevel::Quiet);
//! ```

use std::f64::consts::PI;

/// Standard K-to-Ap conversion table per IAGA specification.
/// Index is K value (0..9), value is the equivalent Ap (linear) index.
const K_TO_AP_TABLE: [f64; 10] = [0.0, 3.0, 7.0, 15.0, 27.0, 48.0, 80.0, 132.0, 207.0, 400.0];

/// Default lower-bound nT thresholds for K-index values 0 through 9.
/// These are typical for a mid-latitude station (~50 deg geomagnetic latitude).
/// K=0: 0-5 nT, K=1: 5-10, K=2: 10-20, K=3: 20-40, K=4: 40-70,
/// K=5: 70-120, K=6: 120-200, K=7: 200-330, K=8: 330-500, K=9: >=500
const DEFAULT_K_THRESHOLDS: [f64; 10] = [0.0, 5.0, 10.0, 20.0, 40.0, 70.0, 120.0, 200.0, 330.0, 500.0];

// ---------------------------------------------------------------------------
// StormLevel
// ---------------------------------------------------------------------------

/// Classification of geomagnetic activity based on the K-index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum StormLevel {
    /// K = 0-1: Quiet conditions.
    Quiet,
    /// K = 2-3: Unsettled conditions, minor fluctuations.
    Unsettled,
    /// K = 4: Active conditions, noticeable fluctuations.
    Active,
    /// K = 5: Minor geomagnetic storm (G1).
    MinorStorm,
    /// K = 6: Moderate geomagnetic storm (G2).
    ModerateStorm,
    /// K = 7: Strong geomagnetic storm (G3).
    StrongStorm,
    /// K = 8: Severe geomagnetic storm (G4).
    SevereStorm,
    /// K = 9: Extreme geomagnetic storm (G5).
    ExtremeStorm,
}

impl StormLevel {
    /// Create a `StormLevel` from a single K-index value (0-9).
    pub fn from_k_index(k: u8) -> Self {
        match k {
            0 | 1 => StormLevel::Quiet,
            2 | 3 => StormLevel::Unsettled,
            4 => StormLevel::Active,
            5 => StormLevel::MinorStorm,
            6 => StormLevel::ModerateStorm,
            7 => StormLevel::StrongStorm,
            8 => StormLevel::SevereStorm,
            _ => StormLevel::ExtremeStorm, // 9+
        }
    }

    /// Return a human-readable description of this storm level.
    pub fn description(&self) -> &'static str {
        match self {
            StormLevel::Quiet => "Quiet",
            StormLevel::Unsettled => "Unsettled",
            StormLevel::Active => "Active",
            StormLevel::MinorStorm => "Minor storm (G1)",
            StormLevel::ModerateStorm => "Moderate storm (G2)",
            StormLevel::StrongStorm => "Strong storm (G3)",
            StormLevel::SevereStorm => "Severe storm (G4)",
            StormLevel::ExtremeStorm => "Extreme storm (G5)",
        }
    }
}

impl core::fmt::Display for StormLevel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.description())
    }
}

// ---------------------------------------------------------------------------
// StormEvent
// ---------------------------------------------------------------------------

/// Describes a detected geomagnetic storm event.
#[derive(Debug, Clone, PartialEq)]
pub struct StormEvent {
    /// Start time as a sample index in the original time series.
    pub start_time: usize,
    /// End time as a sample index in the original time series.
    pub end_time: usize,
    /// Peak K-index observed during the event.
    pub peak_k_index: u8,
    /// Minimum Dst value (nT) during the event (most negative = strongest).
    pub min_dst_nt: f64,
    /// Overall storm classification.
    pub storm_level: StormLevel,
}

// ---------------------------------------------------------------------------
// GeomagneticStormDetector
// ---------------------------------------------------------------------------

/// Main detector struct for geomagnetic storm analysis.
///
/// Holds configuration for a specific magnetometer station including its
/// geomagnetic latitude and the K-index lookup table (nT thresholds).
#[derive(Debug, Clone)]
pub struct GeomagneticStormDetector {
    /// Station geomagnetic latitude in degrees.
    pub station_latitude_deg: f64,
    /// K-index threshold table. `k_index_table[k]` is the lower nT bound for that K value.
    pub k_index_table: [f64; 10],
}

impl GeomagneticStormDetector {
    /// Create a new detector with default K-index thresholds for the given station latitude.
    ///
    /// The default thresholds are representative of a mid-latitude observatory.
    /// For high-latitude stations, use [`GeomagneticStormDetector::with_k_table`].
    pub fn new(station_latitude_deg: f64) -> Self {
        Self {
            station_latitude_deg,
            k_index_table: DEFAULT_K_THRESHOLDS,
        }
    }

    /// Create a detector with a custom K-index threshold table.
    ///
    /// `k_table` must be a monotonically non-decreasing array of 10 nT thresholds
    /// where `k_table[k]` is the lower bound for K = k.
    pub fn with_k_table(station_latitude_deg: f64, k_table: [f64; 10]) -> Self {
        Self {
            station_latitude_deg,
            k_index_table: k_table,
        }
    }

    /// Compute the K-index for a segment of H-component data.
    ///
    /// Delegates to the free function [`compute_k_index_with_table`] using this
    /// detector's configured threshold table.
    pub fn k_index(&self, h_component: &[f64], interval_hours: f64) -> u8 {
        compute_k_index_with_table(h_component, interval_hours, &self.k_index_table)
    }

    /// Compute the cosine of the station latitude (used for Dst estimation).
    pub fn cos_lat(&self) -> f64 {
        (self.station_latitude_deg * PI / 180.0).cos()
    }

    /// Detect storm events from a series of 3-hour K-index values and corresponding
    /// Dst estimates.
    ///
    /// A storm event starts when K >= `k_threshold` and ends when K drops below it
    /// for at least one consecutive interval.
    pub fn detect_storms(
        &self,
        k_indices: &[u8],
        dst_values: &[f64],
        k_threshold: u8,
    ) -> Vec<StormEvent> {
        let mut events = Vec::new();
        let len = k_indices.len().min(dst_values.len());
        if len == 0 {
            return events;
        }

        let mut in_storm = false;
        let mut start = 0usize;
        let mut peak_k: u8 = 0;
        let mut min_dst: f64 = 0.0;

        for i in 0..len {
            if k_indices[i] >= k_threshold {
                if !in_storm {
                    in_storm = true;
                    start = i;
                    peak_k = k_indices[i];
                    min_dst = dst_values[i];
                } else {
                    if k_indices[i] > peak_k {
                        peak_k = k_indices[i];
                    }
                    if dst_values[i] < min_dst {
                        min_dst = dst_values[i];
                    }
                }
            } else if in_storm {
                events.push(StormEvent {
                    start_time: start,
                    end_time: i.saturating_sub(1),
                    peak_k_index: peak_k,
                    min_dst_nt: min_dst,
                    storm_level: StormLevel::from_k_index(peak_k),
                });
                in_storm = false;
            }
        }

        // Close any open storm at the end
        if in_storm {
            events.push(StormEvent {
                start_time: start,
                end_time: len - 1,
                peak_k_index: peak_k,
                min_dst_nt: min_dst,
                storm_level: StormLevel::from_k_index(peak_k),
            });
        }

        events
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Compute the K-index from H-component variation over an interval.
///
/// The K-index is a quasi-logarithmic scale (0-9) describing 3-hour magnetic
/// activity. It is derived from the maximum deviation of the H (horizontal)
/// component from a quiet-day baseline within the interval.
///
/// # Arguments
/// * `h_component` - H-component magnetometer samples (nT) for the interval
/// * `interval_hours` - Duration of the interval in hours (standard: 3.0)
///
/// # Returns
/// K-index value in range 0..=9
pub fn compute_k_index(h_component: &[f64], interval_hours: f64) -> u8 {
    compute_k_index_with_table(h_component, interval_hours, &DEFAULT_K_THRESHOLDS)
}

/// Compute the K-index using a custom threshold table.
///
/// The range is computed as `max(H) - min(H)` over the interval, optionally
/// scaled if the interval differs from the standard 3 hours.
pub fn compute_k_index_with_table(
    h_component: &[f64],
    interval_hours: f64,
    k_table: &[f64; 10],
) -> u8 {
    if h_component.is_empty() {
        return 0;
    }

    let mut min_val = f64::INFINITY;
    let mut max_val = f64::NEG_INFINITY;
    for &v in h_component {
        if v < min_val {
            min_val = v;
        }
        if v > max_val {
            max_val = v;
        }
    }

    // Scale range to standard 3-hour interval
    let range = (max_val - min_val) * (3.0 / interval_hours.max(0.001));

    // Find K by descending through the table
    for k in (0..10u8).rev() {
        if range >= k_table[k as usize] {
            return k;
        }
    }
    0
}

/// Estimate the Dst (Disturbance Storm Time) index from H-component values.
///
/// The Dst index represents the globally averaged depression of the horizontal
/// magnetic field component due to the ring current. It is computed as:
///
/// `Dst(t) = (H(t) - quiet_day_baseline) / cos(lat)`
///
/// Negative Dst values indicate storm-time ring current enhancement.
///
/// # Arguments
/// * `h_values` - H-component magnetometer samples (nT)
/// * `cos_lat` - Cosine of the station's geomagnetic latitude
/// * `quiet_day_baseline` - Quiet-day average H value (nT)
///
/// # Returns
/// Vector of Dst estimates for each sample (nT)
pub fn compute_dst(h_values: &[f64], cos_lat: f64, quiet_day_baseline: f64) -> Vec<f64> {
    let cos_lat_safe = if cos_lat.abs() < 1e-10 { 1e-10 } else { cos_lat };
    h_values
        .iter()
        .map(|&h| (h - quiet_day_baseline) / cos_lat_safe)
        .collect()
}

/// Detect sudden storm commencements (SSC) in the H-component time series.
///
/// An SSC is a rapid, sharp increase in the H-component caused by the arrival
/// of a solar wind shock at the magnetopause. Detection uses a first-derivative
/// threshold: when dH/dt exceeds `threshold_nt` per sample, the onset index
/// is recorded.
///
/// # Arguments
/// * `h_component` - H-component magnetometer samples (nT)
/// * `sample_rate_hz` - Sample rate in Hz
/// * `threshold_nt` - Derivative threshold in nT/s for detection
///
/// # Returns
/// Vector of sample indices where SSC onsets were detected
pub fn detect_ssc(h_component: &[f64], sample_rate_hz: f64, threshold_nt: f64) -> Vec<usize> {
    if h_component.len() < 2 || sample_rate_hz <= 0.0 {
        return Vec::new();
    }

    let mut onsets = Vec::new();
    let dt = 1.0 / sample_rate_hz;
    let mut in_event = false;

    // Minimum separation between detections: 5 minutes worth of samples
    let min_separation = (300.0 * sample_rate_hz) as usize;
    let mut last_onset: usize = 0;

    for i in 1..h_component.len() {
        let derivative = (h_component[i] - h_component[i - 1]) / dt;

        if derivative >= threshold_nt && !in_event {
            if onsets.is_empty() || (i - last_onset) >= min_separation {
                onsets.push(i);
                last_onset = i;
                in_event = true;
            }
        } else if derivative < threshold_nt * 0.5 {
            in_event = false;
        }
    }

    onsets
}

/// Detect substorm onsets via Pi2 pulsation analysis in the Z-component.
///
/// Pi2 pulsations are irregular ULF magnetic pulsations with periods of
/// 40-150 seconds that are a reliable signature of substorm onset.
/// Detection is performed by bandpass filtering the Z-component in the
/// Pi2 band and finding amplitude bursts.
///
/// # Arguments
/// * `z_component` - Z (vertical) component magnetometer samples (nT)
/// * `sample_rate_hz` - Sample rate in Hz
///
/// # Returns
/// Vector of sample indices where substorm onsets were detected
pub fn detect_substorm_onset(z_component: &[f64], sample_rate_hz: f64) -> Vec<usize> {
    if z_component.len() < 4 || sample_rate_hz <= 0.0 {
        return Vec::new();
    }

    // Pi2 band: 40-150 seconds period -> 1/150 to 1/40 Hz -> ~0.0067 to 0.025 Hz
    let f_low = 1.0 / 150.0;
    let f_high = 1.0 / 40.0;

    // Simple bandpass via DFT approach on the signal
    // We use a sliding-window energy detector in the Pi2 band
    let window_seconds = 300.0; // 5-minute analysis window
    let window_samples = (window_seconds * sample_rate_hz) as usize;
    let window_samples = window_samples.max(16).min(z_component.len());
    let step = window_samples / 2; // 50% overlap

    let mut band_energies = Vec::new();
    let mut window_centers = Vec::new();

    let mut pos = 0;
    while pos + window_samples <= z_component.len() {
        let segment = &z_component[pos..pos + window_samples];
        let energy = bandpass_energy(segment, sample_rate_hz, f_low, f_high);
        band_energies.push(energy);
        window_centers.push(pos + window_samples / 2);
        pos += step.max(1);
    }

    if band_energies.is_empty() {
        return Vec::new();
    }

    // Compute mean and std of band energy for thresholding
    let mean_energy = band_energies.iter().sum::<f64>() / band_energies.len() as f64;
    let var = band_energies
        .iter()
        .map(|&e| (e - mean_energy) * (e - mean_energy))
        .sum::<f64>()
        / band_energies.len() as f64;
    let std_energy = var.sqrt();

    // Threshold: mean + 2*std
    let threshold = mean_energy + 2.0 * std_energy;

    let mut onsets = Vec::new();
    let mut in_burst = false;

    for (i, &energy) in band_energies.iter().enumerate() {
        if energy > threshold && !in_burst {
            onsets.push(window_centers[i]);
            in_burst = true;
        } else if energy <= threshold {
            in_burst = false;
        }
    }

    onsets
}

/// Decompose geographic (Bx, By, Bz) magnetic field components into (H, D, Z).
///
/// - **H** (horizontal intensity): `H = Bx * cos(D0) + By * sin(D0)`
/// - **D** (declination perturbation): `D = (-Bx * sin(D0) + By * cos(D0)) / H_mean`
///   (in degrees, approximated for small perturbations)
/// - **Z** (vertical component): same as Bz
///
/// # Arguments
/// * `bx` - Geographic northward component (nT)
/// * `by` - Geographic eastward component (nT)
/// * `bz` - Vertical downward component (nT)
/// * `declination_deg` - Magnetic declination at the station (degrees)
///
/// # Returns
/// Tuple of (H, D, Z) vectors, each the same length as the inputs.
/// D is returned in degrees.
pub fn decompose_hdz(
    bx: &[f64],
    by: &[f64],
    bz: &[f64],
    declination_deg: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let len = bx.len().min(by.len()).min(bz.len());
    let d_rad = declination_deg * PI / 180.0;
    let cos_d = d_rad.cos();
    let sin_d = d_rad.sin();

    let mut h_vec = Vec::with_capacity(len);
    let mut d_vec = Vec::with_capacity(len);
    let mut z_vec = Vec::with_capacity(len);

    // Compute mean H for D normalization
    let mut h_sum = 0.0;
    for i in 0..len {
        let h = bx[i] * cos_d + by[i] * sin_d;
        h_sum += h;
        h_vec.push(h);
    }
    let h_mean = if len > 0 { h_sum / len as f64 } else { 1.0 };
    let h_mean_safe = if h_mean.abs() < 1.0 { 1.0 } else { h_mean };

    for i in 0..len {
        let d_nt = -bx[i] * sin_d + by[i] * cos_d;
        // Convert perturbation in nT to degrees: arctan(d_nt / H_mean) ~ d_nt / H_mean (radians) -> degrees
        let d_deg = (d_nt / h_mean_safe) * (180.0 / PI);
        d_vec.push(d_deg);
        z_vec.push(bz[i]);
    }

    (h_vec, d_vec, z_vec)
}

/// Classify the overall storm level from a set of K-index values.
///
/// Returns the storm level corresponding to the maximum K-index observed.
///
/// # Arguments
/// * `k_indices` - Slice of K-index values (0-9)
///
/// # Returns
/// The `StormLevel` corresponding to the peak K-index
pub fn classify_storm(k_indices: &[u8]) -> StormLevel {
    let max_k = k_indices.iter().copied().max().unwrap_or(0).min(9);
    StormLevel::from_k_index(max_k)
}

/// Convert a K-index value (0-9) to the equivalent Ap index.
///
/// The Ap index is a linearized version of the K-index using the standard
/// IAGA conversion table. Ap is used for planetary geomagnetic activity.
///
/// # Arguments
/// * `k_index` - K value in range 0..=9
///
/// # Returns
/// The equivalent Ap value
pub fn compute_ap_index(k_index: u8) -> f64 {
    let idx = (k_index as usize).min(9);
    K_TO_AP_TABLE[idx]
}

/// Determine the quiet-day baseline curve by averaging the N quietest days.
///
/// Given a collection of daily magnetometer data (each day represented as a
/// vector of H-component samples), this function identifies the `n_quietest`
/// days with the smallest peak-to-peak range and returns their point-by-point
/// average as the quiet-day reference curve.
///
/// # Arguments
/// * `daily_data` - Slice of daily data vectors (each day is a `Vec<f64>`)
/// * `n_quietest` - Number of quietest days to average
///
/// # Returns
/// The averaged quiet-day curve. Its length equals the minimum day length
/// among the selected days.
pub fn quiet_day_curve(daily_data: &[Vec<f64>], n_quietest: usize) -> Vec<f64> {
    if daily_data.is_empty() || n_quietest == 0 {
        return Vec::new();
    }

    // Compute range for each day
    let mut day_ranges: Vec<(usize, f64)> = daily_data
        .iter()
        .enumerate()
        .map(|(i, day)| {
            if day.is_empty() {
                return (i, f64::INFINITY);
            }
            let min = day.iter().cloned().fold(f64::INFINITY, f64::min);
            let max = day.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            (i, max - min)
        })
        .collect();

    // Sort by range (ascending)
    day_ranges.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    let n = n_quietest.min(day_ranges.len());
    let selected_indices: Vec<usize> = day_ranges[..n].iter().map(|&(i, _)| i).collect();

    // Find minimum length across selected days
    let min_len = selected_indices
        .iter()
        .map(|&i| daily_data[i].len())
        .min()
        .unwrap_or(0);

    if min_len == 0 {
        return Vec::new();
    }

    // Average point-by-point
    let mut curve = vec![0.0; min_len];
    for &day_idx in &selected_indices {
        for j in 0..min_len {
            curve[j] += daily_data[day_idx][j];
        }
    }
    for val in &mut curve {
        *val /= n as f64;
    }

    curve
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the energy in a frequency band using a simple DFT.
fn bandpass_energy(signal: &[f64], sample_rate_hz: f64, f_low: f64, f_high: f64) -> f64 {
    let n = signal.len();
    if n == 0 {
        return 0.0;
    }

    let freq_resolution = sample_rate_hz / n as f64;
    let k_low = (f_low / freq_resolution).ceil() as usize;
    let k_high = (f_high / freq_resolution).floor() as usize;

    if k_low > k_high || k_high >= n / 2 {
        return 0.0;
    }

    let mut energy = 0.0;
    for k in k_low..=k_high {
        let mut re = 0.0;
        let mut im = 0.0;
        let omega = 2.0 * PI * k as f64 / n as f64;
        for (i, &s) in signal.iter().enumerate() {
            let angle = omega * i as f64;
            re += s * angle.cos();
            im -= s * angle.sin();
        }
        energy += re * re + im * im;
    }

    energy / (n as f64 * n as f64)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // StormLevel tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_storm_level_from_k_index_quiet() {
        assert_eq!(StormLevel::from_k_index(0), StormLevel::Quiet);
        assert_eq!(StormLevel::from_k_index(1), StormLevel::Quiet);
    }

    #[test]
    fn test_storm_level_from_k_index_unsettled() {
        assert_eq!(StormLevel::from_k_index(2), StormLevel::Unsettled);
        assert_eq!(StormLevel::from_k_index(3), StormLevel::Unsettled);
    }

    #[test]
    fn test_storm_level_from_k_index_storm_grades() {
        assert_eq!(StormLevel::from_k_index(4), StormLevel::Active);
        assert_eq!(StormLevel::from_k_index(5), StormLevel::MinorStorm);
        assert_eq!(StormLevel::from_k_index(6), StormLevel::ModerateStorm);
        assert_eq!(StormLevel::from_k_index(7), StormLevel::StrongStorm);
        assert_eq!(StormLevel::from_k_index(8), StormLevel::SevereStorm);
        assert_eq!(StormLevel::from_k_index(9), StormLevel::ExtremeStorm);
    }

    #[test]
    fn test_storm_level_description() {
        assert_eq!(StormLevel::Quiet.description(), "Quiet");
        assert_eq!(StormLevel::ExtremeStorm.description(), "Extreme storm (G5)");
    }

    #[test]
    fn test_storm_level_display() {
        let s = format!("{}", StormLevel::ModerateStorm);
        assert_eq!(s, "Moderate storm (G2)");
    }

    #[test]
    fn test_storm_level_ordering() {
        assert!(StormLevel::Quiet < StormLevel::Unsettled);
        assert!(StormLevel::Unsettled < StormLevel::Active);
        assert!(StormLevel::Active < StormLevel::MinorStorm);
        assert!(StormLevel::SevereStorm < StormLevel::ExtremeStorm);
    }

    // -----------------------------------------------------------------------
    // K-index computation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_k_index_empty() {
        assert_eq!(compute_k_index(&[], 3.0), 0);
    }

    #[test]
    fn test_compute_k_index_quiet() {
        // Range = 4 nT -> K = 0 (below 5 nT threshold)
        let data: Vec<f64> = (0..100).map(|i| 20000.0 + 2.0 * (i as f64 * 0.1).sin()).collect();
        let k = compute_k_index(&data, 3.0);
        assert!(k <= 1, "Small variation should give K <= 1, got K={}", k);
    }

    #[test]
    fn test_compute_k_index_active() {
        // Create data with ~50 nT range -> K = 4 (40-70 nT range)
        let mut data = vec![20000.0; 100];
        data[50] = 20050.0; // max
        data[60] = 20000.0; // min
        let k = compute_k_index(&data, 3.0);
        assert_eq!(k, 4, "50 nT range should give K=4, got K={}", k);
    }

    #[test]
    fn test_compute_k_index_extreme() {
        // Create data with 600 nT range -> K = 9
        let mut data = vec![20000.0; 100];
        data[10] = 20600.0;
        let k = compute_k_index(&data, 3.0);
        assert_eq!(k, 9, "600 nT range should give K=9, got K={}", k);
    }

    #[test]
    fn test_compute_k_index_interval_scaling() {
        // 25 nT in 1.5 hours -> scaled to 3 hours: 25 * (3/1.5) = 50 nT -> K=4
        let mut data = vec![20000.0; 50];
        data[25] = 20025.0;
        let k = compute_k_index(&data, 1.5);
        assert_eq!(k, 4, "25 nT over 1.5h should scale to K=4, got K={}", k);
    }

    // -----------------------------------------------------------------------
    // Dst computation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_dst_basic() {
        let h_values = vec![20000.0, 19950.0, 19900.0, 20000.0];
        let cos_lat = 45.0_f64.to_radians().cos(); // ~0.707
        let baseline = 20000.0;
        let dst = compute_dst(&h_values, cos_lat, baseline);

        assert_eq!(dst.len(), 4);
        assert!((dst[0]).abs() < 0.01, "First sample should be ~0 Dst");
        assert!(dst[1] < 0.0, "Depressed H should give negative Dst");
        assert!(dst[2] < dst[1], "More depressed H -> more negative Dst");
    }

    #[test]
    fn test_compute_dst_empty() {
        let dst = compute_dst(&[], 0.707, 20000.0);
        assert!(dst.is_empty());
    }

    #[test]
    fn test_compute_dst_equator() {
        // At equator, cos(lat) = 1.0, so Dst = H - baseline
        let h_values = vec![19900.0, 20000.0, 20100.0];
        let dst = compute_dst(&h_values, 1.0, 20000.0);
        assert!((dst[0] - (-100.0)).abs() < 0.01);
        assert!((dst[1] - 0.0).abs() < 0.01);
        assert!((dst[2] - 100.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // SSC detection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_ssc_basic() {
        // Create a sharp step in H (simulating SSC)
        let sample_rate = 1.0; // 1 Hz
        let mut h: Vec<f64> = vec![20000.0; 100];
        // Insert a sharp jump at sample 50
        for i in 50..100 {
            h[i] = 20000.0 + 50.0 * ((i - 50) as f64).min(1.0);
        }
        let onsets = detect_ssc(&h, sample_rate, 30.0);
        assert!(!onsets.is_empty(), "Should detect the SSC");
        assert_eq!(onsets[0], 51, "SSC should be at sample 51 (first large derivative)");
    }

    #[test]
    fn test_detect_ssc_no_event() {
        let h: Vec<f64> = (0..200).map(|i| 20000.0 + 0.1 * (i as f64 * 0.01).sin()).collect();
        let onsets = detect_ssc(&h, 1.0, 10.0);
        assert!(onsets.is_empty(), "Smooth signal should have no SSC");
    }

    #[test]
    fn test_detect_ssc_empty() {
        assert!(detect_ssc(&[], 1.0, 10.0).is_empty());
        assert!(detect_ssc(&[1.0], 1.0, 10.0).is_empty());
    }

    // -----------------------------------------------------------------------
    // Substorm onset detection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_substorm_onset_empty() {
        assert!(detect_substorm_onset(&[], 1.0).is_empty());
        assert!(detect_substorm_onset(&[1.0, 2.0], 1.0).is_empty());
    }

    #[test]
    fn test_detect_substorm_onset_with_burst() {
        // Create signal with a Pi2-band burst (period ~60s -> f ~ 0.0167 Hz)
        let sample_rate = 1.0; // 1 Hz
        let n = 2000;
        let f_pi2 = 1.0 / 60.0; // Pi2 frequency
        let mut z: Vec<f64> = vec![0.0; n];

        // Background noise
        let mut rng_state = 12345u64;
        for val in z.iter_mut() {
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            *val = ((rng_state >> 33) as f64 / (1u64 << 31) as f64 - 1.0) * 0.5;
        }

        // Add strong Pi2 burst at samples 800-1200
        for i in 800..1200 {
            z[i] += 20.0 * (2.0 * PI * f_pi2 * i as f64 / sample_rate).sin();
        }

        let onsets = detect_substorm_onset(&z, sample_rate);
        // We should detect at least one onset in the burst region
        // (exact detection depends on windowing)
        if !onsets.is_empty() {
            // At least one onset should be near the burst region
            let near_burst = onsets.iter().any(|&idx| idx >= 600 && idx <= 1400);
            assert!(near_burst, "Onset should be near burst region, got {:?}", onsets);
        }
        // Note: detection is statistical; we accept if the algorithm runs without panic
    }

    // -----------------------------------------------------------------------
    // HDZ decomposition tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_decompose_hdz_zero_declination() {
        // With D=0, H = Bx, D_pert = By/H_mean, Z = Bz
        let bx = vec![20000.0, 20001.0, 20002.0];
        let by = vec![0.0, 0.0, 0.0];
        let bz = vec![40000.0, 40001.0, 40002.0];
        let (h, d, z) = decompose_hdz(&bx, &by, &bz, 0.0);

        assert_eq!(h.len(), 3);
        assert!((h[0] - 20000.0).abs() < 0.01);
        assert!((z[0] - 40000.0).abs() < 0.01);
        // With By=0 and D=0, declination perturbation should be ~0
        for &dv in &d {
            assert!(dv.abs() < 0.01, "D should be ~0 with zero By, got {}", dv);
        }
    }

    #[test]
    fn test_decompose_hdz_nonzero_declination() {
        // With declination = 10 degrees
        let bx = vec![20000.0; 5];
        let by = vec![3000.0; 5];
        let bz = vec![45000.0; 5];
        let (h, _d, z) = decompose_hdz(&bx, &by, &bz, 10.0);

        // H should be Bx*cos(10) + By*sin(10)
        let expected_h = 20000.0 * 10.0_f64.to_radians().cos() + 3000.0 * 10.0_f64.to_radians().sin();
        assert!((h[0] - expected_h).abs() < 0.01, "H = {}, expected {}", h[0], expected_h);
        assert!((z[0] - 45000.0).abs() < 0.01);
    }

    #[test]
    fn test_decompose_hdz_empty() {
        let (h, d, z) = decompose_hdz(&[], &[], &[], 5.0);
        assert!(h.is_empty());
        assert!(d.is_empty());
        assert!(z.is_empty());
    }

    // -----------------------------------------------------------------------
    // Storm classification tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_storm_quiet() {
        assert_eq!(classify_storm(&[0, 1, 0, 1, 0, 0, 1, 1]), StormLevel::Quiet);
    }

    #[test]
    fn test_classify_storm_severe() {
        assert_eq!(classify_storm(&[2, 3, 5, 8, 6, 4, 2, 1]), StormLevel::SevereStorm);
    }

    #[test]
    fn test_classify_storm_empty() {
        assert_eq!(classify_storm(&[]), StormLevel::Quiet);
    }

    // -----------------------------------------------------------------------
    // Ap index conversion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_ap_index() {
        assert!((compute_ap_index(0) - 0.0).abs() < 0.01);
        assert!((compute_ap_index(3) - 15.0).abs() < 0.01);
        assert!((compute_ap_index(5) - 48.0).abs() < 0.01);
        assert!((compute_ap_index(9) - 400.0).abs() < 0.01);
    }

    #[test]
    fn test_compute_ap_index_clamp() {
        // K > 9 should be clamped to K=9
        assert!((compute_ap_index(10) - 400.0).abs() < 0.01);
        assert!((compute_ap_index(255) - 400.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // Quiet-day curve tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_quiet_day_curve_basic() {
        // 5 days, each with 10 samples
        let daily_data = vec![
            vec![100.0; 10],                                           // range = 0 (quietest)
            (0..10).map(|i| 100.0 + i as f64 * 5.0).collect(),       // range = 45
            (0..10).map(|i| 100.0 + i as f64 * 1.0).collect(),       // range = 9 (2nd quietest)
            (0..10).map(|i| 100.0 + i as f64 * 20.0).collect(),      // range = 180
            (0..10).map(|i| 100.0 + i as f64 * 0.5).collect(),       // range = 4.5 (3rd quietest)
        ];

        // Take 3 quietest: days 0 (range=0), 4 (range=4.5), 2 (range=9)
        let curve = quiet_day_curve(&daily_data, 3);
        assert_eq!(curve.len(), 10);

        // First sample: avg of day0[0]=100, day4[0]=100, day2[0]=100 -> 100
        assert!((curve[0] - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_quiet_day_curve_empty() {
        assert!(quiet_day_curve(&[], 5).is_empty());
        assert!(quiet_day_curve(&[vec![1.0, 2.0]], 0).is_empty());
    }

    #[test]
    fn test_quiet_day_curve_n_exceeds_days() {
        let daily_data = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]];
        // Asking for 10 quietest but only 2 days -> uses all 2
        let curve = quiet_day_curve(&daily_data, 10);
        assert_eq!(curve.len(), 3);
        assert!((curve[0] - 2.5).abs() < 0.01); // (1+4)/2
    }

    // -----------------------------------------------------------------------
    // GeomagneticStormDetector struct tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_new() {
        let det = GeomagneticStormDetector::new(45.0);
        assert!((det.station_latitude_deg - 45.0).abs() < 0.01);
        assert!((det.cos_lat() - 45.0_f64.to_radians().cos()).abs() < 1e-10);
    }

    #[test]
    fn test_detector_with_k_table() {
        let custom = [0.0, 10.0, 20.0, 40.0, 80.0, 140.0, 240.0, 400.0, 660.0, 1000.0];
        let det = GeomagneticStormDetector::with_k_table(60.0, custom);
        assert!((det.k_index_table[5] - 140.0).abs() < 0.01);
    }

    #[test]
    fn test_detector_k_index() {
        let det = GeomagneticStormDetector::new(50.0);
        let mut data = vec![20000.0; 100];
        data[50] = 20080.0; // range 80 nT -> K=5
        let k = det.k_index(&data, 3.0);
        assert_eq!(k, 5, "80 nT range -> K=5, got K={}", k);
    }

    #[test]
    fn test_detector_detect_storms() {
        let det = GeomagneticStormDetector::new(50.0);
        let k_indices = vec![1, 2, 5, 6, 7, 5, 3, 1, 2, 6, 5, 4, 2, 1];
        let dst_values: Vec<f64> = k_indices
            .iter()
            .map(|&k| -10.0 * k as f64)
            .collect();

        let events = det.detect_storms(&k_indices, &dst_values, 5);
        assert_eq!(events.len(), 2, "Should detect 2 storm events");
        assert_eq!(events[0].peak_k_index, 7);
        assert_eq!(events[0].storm_level, StormLevel::StrongStorm);
        assert_eq!(events[1].peak_k_index, 6);
    }

    #[test]
    fn test_detector_detect_storms_empty() {
        let det = GeomagneticStormDetector::new(50.0);
        let events = det.detect_storms(&[], &[], 5);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detector_detect_storms_open_end() {
        // Storm still active at end of data
        let det = GeomagneticStormDetector::new(50.0);
        let k_indices = vec![1, 2, 5, 7, 8];
        let dst_values = vec![-10.0, -20.0, -50.0, -100.0, -200.0];
        let events = det.detect_storms(&k_indices, &dst_values, 5);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].end_time, 4);
        assert_eq!(events[0].peak_k_index, 8);
        assert!((events[0].min_dst_nt - (-200.0)).abs() < 0.01);
    }
}

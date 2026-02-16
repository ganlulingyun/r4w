//! Geotechnical inclinometer data processing for subsurface deformation monitoring.
//!
//! This module processes inclinometer data used for monitoring lateral displacements
//! in slopes, embankments, retaining walls, and other geotechnical structures.
//! Inclinometers measure tilt angles at fixed intervals along a casing installed
//! in a borehole, and integration from a fixed bottom yields cumulative displacement
//! profiles.
//!
//! # Principles
//!
//! An inclinometer probe measures tilt angles at regular depth intervals along a
//! grooved casing. Two orthogonal axes are recorded:
//!
//! - **A-axis**: typically aligned with the expected direction of movement (downhill)
//! - **B-axis**: perpendicular to the A-axis
//!
//! Incremental displacement at depth `i`:
//!
//! ```text
//! delta_x_i = L * sin(theta_i)
//! ```
//!
//! where `L` is the gauge length and `theta_i` is the tilt angle at depth `i`.
//!
//! Cumulative displacement at depth `j` (integrating from fixed bottom):
//!
//! ```text
//! x_j = sum_{i=bottom}^{j} delta_x_i
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::geotechnical_inclinometer::{
//!     InclinometerConfig, InclinometerReading, InclinometerProcessor,
//! };
//!
//! let config = InclinometerConfig {
//!     gauge_length_m: 0.5,
//!     casing_depth_m: 20.0,
//!     reading_interval_m: 0.5,
//!     num_readings: 40,
//! };
//!
//! let processor = InclinometerProcessor::new(config);
//!
//! // Create a reading with small uniform tilt
//! let n = 40;
//! let depths: Vec<f64> = (0..n).map(|i| i as f64 * 0.5).collect();
//! let tilts: Vec<f64> = vec![0.001; n]; // ~0.001 rad tilt
//! let reading = InclinometerReading {
//!     depth_m: depths,
//!     tilt_a_rad: tilts.clone(),
//!     tilt_b_rad: vec![0.0; n],
//!     timestamp_s: 0.0,
//! };
//!
//! let profile = processor.displacement_profile(&reading);
//! assert!(profile.cumulative_a_mm.last().unwrap().abs() > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for an inclinometer installation.
#[derive(Debug, Clone)]
pub struct InclinometerConfig {
    /// Gauge length in metres (distance between wheels, typically 0.5 m).
    pub gauge_length_m: f64,
    /// Total casing depth in metres.
    pub casing_depth_m: f64,
    /// Interval between readings in metres.
    pub reading_interval_m: f64,
    /// Number of readings per survey traverse.
    pub num_readings: usize,
}

// ---------------------------------------------------------------------------
// Reading / data types
// ---------------------------------------------------------------------------

/// A single inclinometer survey (one traverse of the casing).
#[derive(Debug, Clone)]
pub struct InclinometerReading {
    /// Depths of each measurement point (metres from top).
    pub depth_m: Vec<f64>,
    /// A-axis tilt angles in radians (slope direction).
    pub tilt_a_rad: Vec<f64>,
    /// B-axis tilt angles in radians (perpendicular).
    pub tilt_b_rad: Vec<f64>,
    /// Timestamp in seconds (epoch or relative).
    pub timestamp_s: f64,
}

/// Displacement profile computed from a single reading.
#[derive(Debug, Clone)]
pub struct DisplacementProfile {
    /// Depths (metres from top).
    pub depth_m: Vec<f64>,
    /// Incremental A-axis displacement at each interval (mm).
    pub incremental_a_mm: Vec<f64>,
    /// Incremental B-axis displacement at each interval (mm).
    pub incremental_b_mm: Vec<f64>,
    /// Cumulative A-axis displacement from bottom (mm).
    pub cumulative_a_mm: Vec<f64>,
    /// Cumulative B-axis displacement from bottom (mm).
    pub cumulative_b_mm: Vec<f64>,
    /// Resultant displacement (mm).
    pub resultant_mm: Vec<f64>,
    /// Direction of resultant (radians, atan2(B, A)).
    pub direction_rad: Vec<f64>,
}

/// Change profile between two readings.
#[derive(Debug, Clone)]
pub struct ChangeProfile {
    pub depth_m: Vec<f64>,
    /// Change in cumulative A-axis displacement (mm).
    pub delta_a_mm: Vec<f64>,
    /// Change in cumulative B-axis displacement (mm).
    pub delta_b_mm: Vec<f64>,
    /// Change in resultant displacement (mm).
    pub delta_resultant_mm: Vec<f64>,
    /// Rate of A-axis displacement (mm/day).
    pub rate_a_mm_per_day: Vec<f64>,
    /// Rate of B-axis displacement (mm/day).
    pub rate_b_mm_per_day: Vec<f64>,
}

/// Shear zone detection result.
#[derive(Debug, Clone)]
pub struct ShearZone {
    /// Depth of detected shear zone (metres from top).
    pub depth_m: f64,
    /// Maximum incremental displacement at shear zone (mm).
    pub max_incremental_mm: f64,
    /// Axis ('A' or 'B') with dominant shear.
    pub dominant_axis: char,
    /// Displacement gradient change at the shear zone (mm/m).
    pub gradient_change: f64,
}

/// Cruden & Varnes velocity classification for landslides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VelocityClass {
    ExtremelySlow,
    VerySlow,
    Slow,
    Moderate,
    Rapid,
    VeryRapid,
    ExtremelyRapid,
}

impl VelocityClass {
    /// Classify a displacement velocity in mm/s.
    pub fn from_mm_per_s(v: f64) -> Self {
        let abs_v = v.abs();
        // Thresholds converted to mm/s:
        // Extremely slow: < 16 mm/year = 16/(365.25*86400) mm/s ≈ 5.07e-7 mm/s
        // Very slow: < 1.6 m/year = 1600/(365.25*86400) ≈ 5.07e-5 mm/s
        // Slow: < 13 m/month = 13000/(30.44*86400) ≈ 4.94e-3 mm/s
        // Moderate: < 1.8 m/hour = 1.8e6/(3600) = 0.5 mm/s
        // Rapid: < 3 m/min = 3e6/60 ≈ 50.0 mm/s  (= 50 mm/s)
        // Very rapid: < 5 m/s = 5000 mm/s
        const EXTREMELY_SLOW: f64 = 16.0 / (365.25 * 86400.0); // mm/s
        const VERY_SLOW: f64 = 1600.0 / (365.25 * 86400.0);
        const SLOW: f64 = 13000.0 / (30.44 * 86400.0);
        const MODERATE: f64 = 1800.0 / 3600.0; // 1.8 m/h = 1800 mm/h → 0.5 mm/s
        const RAPID: f64 = 3000.0 / 60.0; // 3 m/min = 3000 mm/min → 50 mm/s
        const VERY_RAPID: f64 = 5000.0; // 5 m/s = 5000 mm/s

        if abs_v < EXTREMELY_SLOW {
            VelocityClass::ExtremelySlow
        } else if abs_v < VERY_SLOW {
            VelocityClass::VerySlow
        } else if abs_v < SLOW {
            VelocityClass::Slow
        } else if abs_v < MODERATE {
            VelocityClass::Moderate
        } else if abs_v < RAPID {
            VelocityClass::Rapid
        } else if abs_v < VERY_RAPID {
            VelocityClass::VeryRapid
        } else {
            VelocityClass::ExtremelyRapid
        }
    }

    /// Return human-readable label.
    pub fn label(self) -> &'static str {
        match self {
            VelocityClass::ExtremelySlow => "Extremely slow",
            VelocityClass::VerySlow => "Very slow",
            VelocityClass::Slow => "Slow",
            VelocityClass::Moderate => "Moderate",
            VelocityClass::Rapid => "Rapid",
            VelocityClass::VeryRapid => "Very rapid",
            VelocityClass::ExtremelyRapid => "Extremely rapid",
        }
    }
}

/// Multi-level alarm thresholds.
#[derive(Debug, Clone)]
pub struct AlarmThresholds {
    /// Total displacement limits (mm) for each level.
    pub displacement_mm: [f64; 4],
    /// Velocity limits (mm/day) for each level.
    pub velocity_mm_per_day: [f64; 4],
    /// Acceleration limits (mm/day^2) for each level.
    pub acceleration_mm_per_day2: [f64; 4],
}

/// Alarm level.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlarmLevel {
    Normal,
    Attention,
    Warning,
    Alarm,
    Evacuation,
}

impl AlarmLevel {
    pub fn label(self) -> &'static str {
        match self {
            AlarmLevel::Normal => "Normal",
            AlarmLevel::Attention => "Attention",
            AlarmLevel::Warning => "Warning",
            AlarmLevel::Alarm => "Alarm",
            AlarmLevel::Evacuation => "Evacuation",
        }
    }
}

/// Checksum verification result for bi-directional readings.
#[derive(Debug, Clone)]
pub struct ChecksumResult {
    /// Depth of each reading (m).
    pub depth_m: Vec<f64>,
    /// Checksum values (A+ + A-) at each depth.
    pub checksum: Vec<f64>,
    /// Bias values (A+ + A-)/2 at each depth.
    pub bias: Vec<f64>,
    /// Sensitivity values (A+ - A-)/2 at each depth.
    pub sensitivity: Vec<f64>,
    /// Whether each reading passes the checksum tolerance.
    pub pass: Vec<bool>,
}

/// Data quality report.
#[derive(Debug, Clone)]
pub struct DataQualityReport {
    /// Fraction of readings with valid checksums (0..1).
    pub checksum_pass_rate: f64,
    /// Maximum absolute checksum deviation.
    pub max_checksum_deviation: f64,
    /// Number of missing data points detected.
    pub missing_count: usize,
    /// Whether systematic drift was detected.
    pub drift_detected: bool,
    /// Spiral correction applied (radians).
    pub spiral_correction_rad: Vec<f64>,
}

/// Time series analysis result for a specific depth.
#[derive(Debug, Clone)]
pub struct TimeSeries {
    /// Timestamps (seconds).
    pub time_s: Vec<f64>,
    /// Displacement values (mm).
    pub displacement_mm: Vec<f64>,
    /// Velocity values (mm/day).
    pub velocity_mm_per_day: Vec<f64>,
    /// Acceleration values (mm/day^2).
    pub acceleration_mm_per_day2: Vec<f64>,
    /// Linear trend slope (mm/day).
    pub trend_slope_mm_per_day: f64,
    /// Linear trend intercept (mm).
    pub trend_intercept_mm: f64,
}

/// Result of inverse velocity analysis (Fukuzono method).
#[derive(Debug, Clone)]
pub struct InverseVelocityResult {
    /// Timestamps (seconds).
    pub time_s: Vec<f64>,
    /// Inverse velocity values (day/mm).
    pub inverse_velocity: Vec<f64>,
    /// Estimated time of failure (seconds from epoch), if extrapolation reaches zero.
    pub failure_time_s: Option<f64>,
    /// Linear regression slope of 1/v vs t.
    pub regression_slope: f64,
    /// Linear regression intercept of 1/v vs t.
    pub regression_intercept: f64,
}

/// Seasonal correction parameters (sinusoidal fit).
#[derive(Debug, Clone)]
pub struct SeasonalCorrection {
    /// Amplitude of seasonal component (mm).
    pub amplitude_mm: f64,
    /// Phase offset (radians).
    pub phase_rad: f64,
    /// Period (seconds, typically ~365.25 days).
    pub period_s: f64,
    /// Mean (DC offset, mm).
    pub mean_mm: f64,
}

// ---------------------------------------------------------------------------
// Processor
// ---------------------------------------------------------------------------

/// Main inclinometer data processor.
#[derive(Debug, Clone)]
pub struct InclinometerProcessor {
    config: InclinometerConfig,
}

impl InclinometerProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: InclinometerConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &InclinometerConfig {
        &self.config
    }

    // -----------------------------------------------------------------------
    // Displacement calculation
    // -----------------------------------------------------------------------

    /// Compute incremental displacements (mm) from tilt angles.
    pub fn incremental_displacement(&self, tilts_rad: &[f64]) -> Vec<f64> {
        let l = self.config.gauge_length_m;
        tilts_rad.iter().map(|&t| l * t.sin() * 1000.0).collect()
    }

    /// Compute cumulative displacement (mm) from bottom, given incremental displacements.
    /// Index 0 = shallowest (top), last index = deepest (bottom, fixed).
    /// Integration proceeds from the bottom upward.
    pub fn cumulative_from_bottom(&self, incremental_mm: &[f64]) -> Vec<f64> {
        let n = incremental_mm.len();
        let mut cum = vec![0.0; n];
        // Start from the bottom (last element). Bottom is fixed (zero).
        // Cumulate upward: cum[i] = sum of incremental from i to n-1
        // but displacement at bottom is 0, and we accumulate going up.
        let mut running = 0.0;
        for i in (0..n).rev() {
            running += incremental_mm[i];
            cum[i] = running;
        }
        cum
    }

    /// Compute a full displacement profile from a single reading.
    pub fn displacement_profile(&self, reading: &InclinometerReading) -> DisplacementProfile {
        let inc_a = self.incremental_displacement(&reading.tilt_a_rad);
        let inc_b = self.incremental_displacement(&reading.tilt_b_rad);
        let cum_a = self.cumulative_from_bottom(&inc_a);
        let cum_b = self.cumulative_from_bottom(&inc_b);

        let n = reading.depth_m.len();
        let mut resultant = vec![0.0; n];
        let mut direction = vec![0.0; n];
        for i in 0..n {
            resultant[i] = (cum_a[i] * cum_a[i] + cum_b[i] * cum_b[i]).sqrt();
            direction[i] = cum_b[i].atan2(cum_a[i]);
        }

        DisplacementProfile {
            depth_m: reading.depth_m.clone(),
            incremental_a_mm: inc_a,
            incremental_b_mm: inc_b,
            cumulative_a_mm: cum_a,
            cumulative_b_mm: cum_b,
            resultant_mm: resultant,
            direction_rad: direction,
        }
    }

    // -----------------------------------------------------------------------
    // Change profile
    // -----------------------------------------------------------------------

    /// Compute the change profile between a current reading and an initial (baseline) reading.
    pub fn change_profile(
        &self,
        initial: &InclinometerReading,
        current: &InclinometerReading,
    ) -> ChangeProfile {
        let prof_init = self.displacement_profile(initial);
        let prof_curr = self.displacement_profile(current);

        let n = prof_init.depth_m.len().min(prof_curr.depth_m.len());
        let dt_days = (current.timestamp_s - initial.timestamp_s) / 86400.0;
        let dt_safe = if dt_days.abs() < 1e-12 { 1.0 } else { dt_days };

        let mut delta_a = vec![0.0; n];
        let mut delta_b = vec![0.0; n];
        let mut delta_r = vec![0.0; n];
        let mut rate_a = vec![0.0; n];
        let mut rate_b = vec![0.0; n];

        for i in 0..n {
            delta_a[i] = prof_curr.cumulative_a_mm[i] - prof_init.cumulative_a_mm[i];
            delta_b[i] = prof_curr.cumulative_b_mm[i] - prof_init.cumulative_b_mm[i];
            delta_r[i] = (delta_a[i] * delta_a[i] + delta_b[i] * delta_b[i]).sqrt();
            rate_a[i] = delta_a[i] / dt_safe;
            rate_b[i] = delta_b[i] / dt_safe;
        }

        ChangeProfile {
            depth_m: prof_curr.depth_m[..n].to_vec(),
            delta_a_mm: delta_a,
            delta_b_mm: delta_b,
            delta_resultant_mm: delta_r,
            rate_a_mm_per_day: rate_a,
            rate_b_mm_per_day: rate_b,
        }
    }

    /// Find depth and magnitude of maximum displacement in a profile.
    pub fn max_displacement(&self, profile: &DisplacementProfile) -> (f64, f64) {
        let mut max_val = 0.0_f64;
        let mut max_depth = 0.0_f64;
        for (i, &r) in profile.resultant_mm.iter().enumerate() {
            if r > max_val {
                max_val = r;
                max_depth = profile.depth_m[i];
            }
        }
        (max_depth, max_val)
    }

    // -----------------------------------------------------------------------
    // Shear zone detection
    // -----------------------------------------------------------------------

    /// Detect shear zones from a displacement profile.
    /// Returns zones where incremental displacement exceeds `threshold_mm`.
    pub fn detect_shear_zones(
        &self,
        profile: &DisplacementProfile,
        threshold_mm: f64,
    ) -> Vec<ShearZone> {
        let mut zones = Vec::new();
        let n = profile.depth_m.len();
        if n < 2 {
            return zones;
        }

        for i in 0..n {
            let inc_a = profile.incremental_a_mm[i].abs();
            let inc_b = profile.incremental_b_mm[i].abs();
            let inc_max = inc_a.max(inc_b);

            if inc_max > threshold_mm {
                let dominant = if inc_a >= inc_b { 'A' } else { 'B' };

                // Compute gradient change (second derivative of cumulative)
                let gradient_change = if i > 0 && i < n - 1 {
                    let interval = self.config.reading_interval_m;
                    let cum = if dominant == 'A' {
                        &profile.cumulative_a_mm
                    } else {
                        &profile.cumulative_b_mm
                    };
                    let d2 = (cum[i + 1] - 2.0 * cum[i] + cum[i - 1])
                        / (interval * interval);
                    d2.abs()
                } else {
                    0.0
                };

                zones.push(ShearZone {
                    depth_m: profile.depth_m[i],
                    max_incremental_mm: inc_max,
                    dominant_axis: dominant,
                    gradient_change,
                });
            }
        }
        zones
    }

    /// Detect inflection points in a cumulative displacement profile.
    /// Returns indices where the second derivative changes sign.
    pub fn find_inflection_points(&self, cumulative_mm: &[f64]) -> Vec<usize> {
        let n = cumulative_mm.len();
        if n < 3 {
            return Vec::new();
        }

        let mut inflections = Vec::new();
        // Compute second differences
        let mut prev_d2 = cumulative_mm[2] - 2.0 * cumulative_mm[1] + cumulative_mm[0];
        for i in 2..n - 1 {
            let d2 = cumulative_mm[i + 1] - 2.0 * cumulative_mm[i] + cumulative_mm[i - 1];
            if prev_d2 * d2 < 0.0 {
                inflections.push(i);
            }
            prev_d2 = d2;
        }
        inflections
    }

    // -----------------------------------------------------------------------
    // Checksum verification
    // -----------------------------------------------------------------------

    /// Verify bi-directional readings (A+ and A- passes).
    /// `a_plus` and `a_minus` are tilt readings in the normal and reversed orientations.
    /// `tolerance_rad` is the maximum acceptable checksum value.
    pub fn verify_checksum(
        &self,
        depths: &[f64],
        a_plus: &[f64],
        a_minus: &[f64],
        tolerance_rad: f64,
    ) -> ChecksumResult {
        let n = depths.len().min(a_plus.len()).min(a_minus.len());
        let mut checksum = vec![0.0; n];
        let mut bias = vec![0.0; n];
        let mut sensitivity = vec![0.0; n];
        let mut pass = vec![false; n];

        for i in 0..n {
            checksum[i] = a_plus[i] + a_minus[i];
            bias[i] = (a_plus[i] + a_minus[i]) / 2.0;
            sensitivity[i] = (a_plus[i] - a_minus[i]) / 2.0;
            pass[i] = checksum[i].abs() < tolerance_rad;
        }

        ChecksumResult {
            depth_m: depths[..n].to_vec(),
            checksum,
            bias,
            sensitivity,
            pass,
        }
    }

    // -----------------------------------------------------------------------
    // Data quality
    // -----------------------------------------------------------------------

    /// Perform data quality checks on a reading.
    pub fn data_quality_check(
        &self,
        reading: &InclinometerReading,
        checksum_tolerance: f64,
        a_minus: Option<&[f64]>,
    ) -> DataQualityReport {
        let n = reading.depth_m.len();

        // Checksum if A- readings are available
        let (pass_rate, max_dev) = if let Some(am) = a_minus {
            let cr = self.verify_checksum(
                &reading.depth_m,
                &reading.tilt_a_rad,
                am,
                checksum_tolerance,
            );
            let pass_count = cr.pass.iter().filter(|&&p| p).count();
            let max_cs = cr
                .checksum
                .iter()
                .map(|v| v.abs())
                .fold(0.0_f64, f64::max);
            (pass_count as f64 / n.max(1) as f64, max_cs)
        } else {
            (1.0, 0.0)
        };

        // Missing data detection (NaN or exactly 0 tilt at multiple consecutive depths)
        let mut missing = 0usize;
        for &t in &reading.tilt_a_rad {
            if t.is_nan() || t.is_infinite() {
                missing += 1;
            }
        }

        // Systematic drift detection: check if tilt monotonically increases with depth
        let drift_detected = detect_systematic_drift(&reading.tilt_a_rad);

        // Spiral correction: estimate cumulative rotation of casing
        let spiral = compute_spiral_correction(&reading.tilt_a_rad, &reading.tilt_b_rad);

        DataQualityReport {
            checksum_pass_rate: pass_rate,
            max_checksum_deviation: max_dev,
            missing_count: missing,
            drift_detected,
            spiral_correction_rad: spiral,
        }
    }

    /// Interpolate missing (NaN) values using linear interpolation.
    pub fn interpolate_missing(values: &mut [f64]) {
        let n = values.len();
        if n < 2 {
            return;
        }

        // Find runs of NaN and interpolate
        let mut i = 0;
        while i < n {
            if values[i].is_nan() {
                // Find the start of the NaN run
                let start = i;
                while i < n && values[i].is_nan() {
                    i += 1;
                }
                let end = i; // first non-NaN after run

                let v_before = if start > 0 { values[start - 1] } else { 0.0 };
                let v_after = if end < n { values[end] } else { v_before };

                let span = (end - start + 1) as f64;
                for j in start..end {
                    let frac = (j - start + 1) as f64 / span;
                    values[j] = v_before + frac * (v_after - v_before);
                }
            } else {
                i += 1;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Alarm evaluation
    // -----------------------------------------------------------------------

    /// Evaluate alarm level based on current displacement, velocity, and acceleration.
    pub fn evaluate_alarm(
        &self,
        thresholds: &AlarmThresholds,
        displacement_mm: f64,
        velocity_mm_per_day: f64,
        acceleration_mm_per_day2: f64,
    ) -> AlarmLevel {
        let d = displacement_mm.abs();
        let v = velocity_mm_per_day.abs();
        let a = acceleration_mm_per_day2.abs();

        // Check from highest level down
        if d >= thresholds.displacement_mm[3]
            || v >= thresholds.velocity_mm_per_day[3]
            || a >= thresholds.acceleration_mm_per_day2[3]
        {
            AlarmLevel::Evacuation
        } else if d >= thresholds.displacement_mm[2]
            || v >= thresholds.velocity_mm_per_day[2]
            || a >= thresholds.acceleration_mm_per_day2[2]
        {
            AlarmLevel::Alarm
        } else if d >= thresholds.displacement_mm[1]
            || v >= thresholds.velocity_mm_per_day[1]
            || a >= thresholds.acceleration_mm_per_day2[1]
        {
            AlarmLevel::Warning
        } else if d >= thresholds.displacement_mm[0]
            || v >= thresholds.velocity_mm_per_day[0]
            || a >= thresholds.acceleration_mm_per_day2[0]
        {
            AlarmLevel::Attention
        } else {
            AlarmLevel::Normal
        }
    }

    // -----------------------------------------------------------------------
    // Time series analysis
    // -----------------------------------------------------------------------

    /// Extract time series of displacement at a specific depth index across multiple readings.
    pub fn time_series_at_depth(
        &self,
        readings: &[InclinometerReading],
        depth_index: usize,
    ) -> TimeSeries {
        let mut times = Vec::with_capacity(readings.len());
        let mut displacements = Vec::with_capacity(readings.len());

        for r in readings {
            if depth_index >= r.depth_m.len() {
                continue;
            }
            let prof = self.displacement_profile(r);
            let resultant = prof.resultant_mm[depth_index];
            times.push(r.timestamp_s);
            displacements.push(resultant);
        }

        // Compute velocity (mm/day) using central differences
        let n = times.len();
        let mut velocity = vec![0.0; n];
        let mut acceleration = vec![0.0; n];

        for i in 1..n {
            let dt_days = (times[i] - times[i - 1]) / 86400.0;
            if dt_days.abs() > 1e-12 {
                velocity[i] = (displacements[i] - displacements[i - 1]) / dt_days;
            }
        }
        if n > 0 {
            velocity[0] = if n > 1 { velocity[1] } else { 0.0 };
        }

        for i in 1..n {
            let dt_days = (times[i] - times[i - 1]) / 86400.0;
            if dt_days.abs() > 1e-12 {
                acceleration[i] = (velocity[i] - velocity[i - 1]) / dt_days;
            }
        }
        if n > 0 {
            acceleration[0] = if n > 1 { acceleration[1] } else { 0.0 };
        }

        // Linear regression for trend
        let (slope, intercept) = linear_regression(&times, &displacements);
        // Convert slope from mm/s to mm/day
        let slope_per_day = slope * 86400.0;

        TimeSeries {
            time_s: times,
            displacement_mm: displacements,
            velocity_mm_per_day: velocity,
            acceleration_mm_per_day2: acceleration,
            trend_slope_mm_per_day: slope_per_day,
            trend_intercept_mm: intercept,
        }
    }

    /// Fit a sinusoidal seasonal correction to displacement data.
    /// Assumes annual periodicity (~365.25 days).
    pub fn fit_seasonal_correction(
        &self,
        times_s: &[f64],
        displacements_mm: &[f64],
    ) -> SeasonalCorrection {
        let n = times_s.len();
        if n < 3 {
            return SeasonalCorrection {
                amplitude_mm: 0.0,
                phase_rad: 0.0,
                period_s: 365.25 * 86400.0,
                mean_mm: if n > 0 {
                    displacements_mm.iter().sum::<f64>() / n as f64
                } else {
                    0.0
                },
            };
        }

        let period = 365.25 * 86400.0;
        let omega = 2.0 * PI / period;

        // Least-squares fit: d(t) = mean + A*cos(omega*t) + B*sin(omega*t)
        // Minimize sum of (d_i - mean - A*cos - B*sin)^2
        let mean = displacements_mm.iter().sum::<f64>() / n as f64;

        let mut sum_c = 0.0;
        let mut sum_s = 0.0;
        let mut sum_cc = 0.0;
        let mut sum_ss = 0.0;
        let mut sum_cs = 0.0;
        let mut sum_dc = 0.0;
        let mut sum_ds = 0.0;

        for i in 0..n {
            let c = (omega * times_s[i]).cos();
            let s = (omega * times_s[i]).sin();
            let d = displacements_mm[i] - mean;
            sum_c += c;
            sum_s += s;
            sum_cc += c * c;
            sum_ss += s * s;
            sum_cs += c * s;
            sum_dc += d * c;
            sum_ds += d * s;
        }

        // Solve 2x2 system: [[sum_cc, sum_cs], [sum_cs, sum_ss]] * [A, B] = [sum_dc, sum_ds]
        let det = sum_cc * sum_ss - sum_cs * sum_cs;
        let (a_coeff, b_coeff) = if det.abs() > 1e-20 {
            (
                (sum_dc * sum_ss - sum_ds * sum_cs) / det,
                (sum_ds * sum_cc - sum_dc * sum_cs) / det,
            )
        } else {
            (0.0, 0.0)
        };

        let amplitude = (a_coeff * a_coeff + b_coeff * b_coeff).sqrt();
        let phase = b_coeff.atan2(a_coeff);

        SeasonalCorrection {
            amplitude_mm: amplitude,
            phase_rad: phase,
            period_s: period,
            mean_mm: mean,
        }
    }

    /// Remove seasonal component from displacement data.
    pub fn remove_seasonal(
        &self,
        times_s: &[f64],
        displacements_mm: &[f64],
        correction: &SeasonalCorrection,
    ) -> Vec<f64> {
        let omega = 2.0 * PI / correction.period_s;
        times_s
            .iter()
            .zip(displacements_mm.iter())
            .map(|(&t, &d)| {
                let seasonal = correction.amplitude_mm * (omega * t + correction.phase_rad).cos();
                d - seasonal
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Inverse velocity method (Fukuzono)
    // -----------------------------------------------------------------------

    /// Perform inverse velocity analysis for failure time prediction.
    /// Velocities in mm/day, times in seconds.
    pub fn inverse_velocity_analysis(
        &self,
        times_s: &[f64],
        velocities_mm_per_day: &[f64],
    ) -> InverseVelocityResult {
        let n = times_s.len().min(velocities_mm_per_day.len());
        let mut valid_t = Vec::new();
        let mut inv_v = Vec::new();

        for i in 0..n {
            let v = velocities_mm_per_day[i].abs();
            if v > 1e-12 {
                valid_t.push(times_s[i]);
                inv_v.push(1.0 / v);
            }
        }

        // Linear regression on 1/v vs t
        let (slope, intercept) = linear_regression(&valid_t, &inv_v);

        // Failure when 1/v = 0 → t_failure = -intercept/slope
        let failure_time = if slope < -1e-20 {
            let tf = -intercept / slope;
            // Only valid if failure time is in the future relative to last data point
            if !valid_t.is_empty() && tf > *valid_t.last().unwrap() {
                Some(tf)
            } else {
                None
            }
        } else {
            None
        };

        InverseVelocityResult {
            time_s: valid_t,
            inverse_velocity: inv_v,
            failure_time_s: failure_time,
            regression_slope: slope,
            regression_intercept: intercept,
        }
    }

    // -----------------------------------------------------------------------
    // Factor of safety estimation (simplified displacement-rate based)
    // -----------------------------------------------------------------------

    /// Estimate a simplified factor of safety indicator from displacement rate trend.
    /// Returns a value where < 1.0 suggests approaching failure.
    /// Based on the ratio of critical velocity to current velocity.
    pub fn displacement_rate_fos(
        &self,
        current_velocity_mm_per_day: f64,
        critical_velocity_mm_per_day: f64,
    ) -> f64 {
        let v = current_velocity_mm_per_day.abs();
        if v < 1e-12 {
            return f64::INFINITY;
        }
        critical_velocity_mm_per_day.abs() / v
    }

    // -----------------------------------------------------------------------
    // Resultant displacement and vector data
    // -----------------------------------------------------------------------

    /// Compute resultant displacement and direction from A and B cumulative profiles.
    pub fn resultant_displacement(cum_a_mm: &[f64], cum_b_mm: &[f64]) -> Vec<(f64, f64)> {
        let n = cum_a_mm.len().min(cum_b_mm.len());
        (0..n)
            .map(|i| {
                let r = (cum_a_mm[i] * cum_a_mm[i] + cum_b_mm[i] * cum_b_mm[i]).sqrt();
                let theta = cum_b_mm[i].atan2(cum_a_mm[i]);
                (r, theta)
            })
            .collect()
    }

    /// Generate vector plot data: (depth, magnitude_mm, direction_rad).
    pub fn vector_plot_data(&self, profile: &DisplacementProfile) -> Vec<(f64, f64, f64)> {
        let n = profile.depth_m.len();
        (0..n)
            .map(|i| {
                (
                    profile.depth_m[i],
                    profile.resultant_mm[i],
                    profile.direction_rad[i],
                )
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Simple linear regression: y = slope * x + intercept.
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len().min(y.len());
    if n < 2 {
        return (0.0, if n == 1 { y[0] } else { 0.0 });
    }

    let nf = n as f64;
    let sum_x: f64 = x[..n].iter().sum();
    let sum_y: f64 = y[..n].iter().sum();
    let sum_xx: f64 = x[..n].iter().map(|&xi| xi * xi).sum();
    let sum_xy: f64 = x[..n].iter().zip(y[..n].iter()).map(|(&xi, &yi)| xi * yi).sum();

    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / nf);
    }

    let slope = (nf * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / nf;
    (slope, intercept)
}

/// Detect systematic drift: checks if the tilt values have a strong linear trend with depth.
fn detect_systematic_drift(tilts: &[f64]) -> bool {
    let n = tilts.len();
    if n < 5 {
        return false;
    }

    // Linear regression on index vs tilt
    let indices: Vec<f64> = (0..n).map(|i| i as f64).collect();
    let (slope, intercept) = linear_regression(&indices, tilts);

    // Check R² to see if the trend is strong
    let mean_y = tilts.iter().sum::<f64>() / n as f64;
    let ss_tot: f64 = tilts.iter().map(|&t| (t - mean_y).powi(2)).sum();
    let ss_res: f64 = tilts
        .iter()
        .enumerate()
        .map(|(i, &t)| {
            let predicted = slope * i as f64 + intercept;
            (t - predicted).powi(2)
        })
        .sum();

    if ss_tot < 1e-20 {
        return false;
    }
    let r_squared = 1.0 - ss_res / ss_tot;

    // Strong linear trend with non-trivial slope indicates drift
    r_squared > 0.95 && slope.abs() > 1e-6
}

/// Compute spiral correction from A and B axis data.
/// Returns estimated rotation at each depth due to casing spiral.
fn compute_spiral_correction(tilt_a: &[f64], tilt_b: &[f64]) -> Vec<f64> {
    let n = tilt_a.len().min(tilt_b.len());
    let mut corrections = vec![0.0; n];

    if n < 2 {
        return corrections;
    }

    // Estimate local rotation angle from the relationship between A and B axes.
    // If the casing spirals, the axes rotate with depth. The spiral angle at depth i
    // is estimated from the arctangent of (B/A) change relative to expected.
    let mut cumulative_rotation = 0.0;
    for i in 1..n {
        let angle_prev = tilt_b[i - 1].atan2(tilt_a[i - 1]);
        let angle_curr = tilt_b[i].atan2(tilt_a[i]);
        let mut delta = angle_curr - angle_prev;
        // Wrap to [-pi, pi]
        while delta > PI {
            delta -= 2.0 * PI;
        }
        while delta < -PI {
            delta += 2.0 * PI;
        }
        cumulative_rotation += delta;
        corrections[i] = cumulative_rotation;
    }

    corrections
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> InclinometerConfig {
        InclinometerConfig {
            gauge_length_m: 0.5,
            casing_depth_m: 20.0,
            reading_interval_m: 0.5,
            num_readings: 40,
        }
    }

    fn make_reading(n: usize, tilt_a: f64, tilt_b: f64, timestamp: f64) -> InclinometerReading {
        InclinometerReading {
            depth_m: (0..n).map(|i| i as f64 * 0.5).collect(),
            tilt_a_rad: vec![tilt_a; n],
            tilt_b_rad: vec![tilt_b; n],
            timestamp_s: timestamp,
        }
    }

    #[test]
    fn test_config_creation() {
        let config = default_config();
        assert_eq!(config.gauge_length_m, 0.5);
        assert_eq!(config.casing_depth_m, 20.0);
        assert_eq!(config.num_readings, 40);
    }

    #[test]
    fn test_incremental_displacement_zero_tilt() {
        let proc = InclinometerProcessor::new(default_config());
        let tilts = vec![0.0; 10];
        let inc = proc.incremental_displacement(&tilts);
        for &v in &inc {
            assert!((v).abs() < 1e-12);
        }
    }

    #[test]
    fn test_incremental_displacement_small_angle() {
        let proc = InclinometerProcessor::new(default_config());
        // For small angles, sin(theta) ≈ theta
        let theta = 0.001; // 1 mrad
        let tilts = vec![theta; 5];
        let inc = proc.incremental_displacement(&tilts);
        // Expected: 0.5 * sin(0.001) * 1000 ≈ 0.5 mm
        for &v in &inc {
            assert!((v - 0.5 * theta.sin() * 1000.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_cumulative_from_bottom_uniform() {
        let proc = InclinometerProcessor::new(default_config());
        let inc = vec![1.0; 5]; // 1 mm each
        let cum = proc.cumulative_from_bottom(&inc);
        // Bottom element should have value 1.0, top should have 5.0
        assert!((cum[4] - 1.0).abs() < 1e-10);
        assert!((cum[0] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_cumulative_from_bottom_single() {
        let proc = InclinometerProcessor::new(default_config());
        let inc = vec![3.5];
        let cum = proc.cumulative_from_bottom(&inc);
        assert_eq!(cum.len(), 1);
        assert!((cum[0] - 3.5).abs() < 1e-10);
    }

    #[test]
    fn test_displacement_profile_basic() {
        let proc = InclinometerProcessor::new(default_config());
        let reading = make_reading(10, 0.002, 0.0, 0.0);
        let profile = proc.displacement_profile(&reading);
        assert_eq!(profile.depth_m.len(), 10);
        assert_eq!(profile.cumulative_a_mm.len(), 10);
        // Top should have largest cumulative displacement
        assert!(profile.cumulative_a_mm[0].abs() > profile.cumulative_a_mm[9].abs());
    }

    #[test]
    fn test_displacement_profile_resultant() {
        let proc = InclinometerProcessor::new(default_config());
        let reading = make_reading(5, 0.001, 0.001, 0.0);
        let profile = proc.displacement_profile(&reading);
        // Resultant = sqrt(A^2 + B^2) should be > either component
        for i in 0..5 {
            let expected = (profile.cumulative_a_mm[i].powi(2)
                + profile.cumulative_b_mm[i].powi(2))
            .sqrt();
            assert!((profile.resultant_mm[i] - expected).abs() < 1e-10);
        }
    }

    #[test]
    fn test_displacement_profile_direction() {
        let proc = InclinometerProcessor::new(default_config());
        // A only → direction should be ~0
        let r1 = make_reading(5, 0.001, 0.0, 0.0);
        let p1 = proc.displacement_profile(&r1);
        for &d in &p1.direction_rad {
            assert!(d.abs() < 0.01 || (d - PI).abs() < 0.01 || (d + PI).abs() < 0.01
                || p1.resultant_mm[0] < 1e-10);
        }

        // B only → direction should be ~pi/2
        let r2 = make_reading(5, 0.0, 0.001, 0.0);
        let p2 = proc.displacement_profile(&r2);
        assert!((p2.direction_rad[0] - PI / 2.0).abs() < 0.1);
    }

    #[test]
    fn test_change_profile() {
        let proc = InclinometerProcessor::new(default_config());
        let initial = make_reading(10, 0.001, 0.0, 0.0);
        let current = make_reading(10, 0.002, 0.0, 86400.0); // 1 day later
        let change = proc.change_profile(&initial, &current);
        assert_eq!(change.depth_m.len(), 10);
        // Delta should be positive (more displacement)
        assert!(change.delta_a_mm[0] > 0.0);
        // Rate should be non-zero
        assert!(change.rate_a_mm_per_day[0].abs() > 0.0);
    }

    #[test]
    fn test_change_profile_no_time_difference() {
        let proc = InclinometerProcessor::new(default_config());
        let initial = make_reading(5, 0.001, 0.0, 100.0);
        let current = make_reading(5, 0.002, 0.0, 100.0); // Same timestamp
        let change = proc.change_profile(&initial, &current);
        // Should not panic; rate uses safe division
        assert_eq!(change.depth_m.len(), 5);
    }

    #[test]
    fn test_max_displacement() {
        let proc = InclinometerProcessor::new(default_config());
        let reading = make_reading(10, 0.002, 0.0, 0.0);
        let profile = proc.displacement_profile(&reading);
        let (depth, mag) = proc.max_displacement(&profile);
        // Maximum should be at the top (depth=0)
        assert!((depth - 0.0).abs() < 1e-10);
        assert!(mag > 0.0);
    }

    #[test]
    fn test_detect_shear_zones_none() {
        let proc = InclinometerProcessor::new(default_config());
        let reading = make_reading(10, 0.0001, 0.0, 0.0);
        let profile = proc.displacement_profile(&reading);
        let zones = proc.detect_shear_zones(&profile, 1.0); // 1 mm threshold
        // With very small tilt, no zones
        assert!(zones.is_empty());
    }

    #[test]
    fn test_detect_shear_zones_present() {
        let proc = InclinometerProcessor::new(default_config());
        let mut reading = make_reading(20, 0.0001, 0.0, 0.0);
        // Insert a large tilt at depth index 10 (shear zone)
        reading.tilt_a_rad[10] = 0.05; // ~50 mrad → large incremental
        let profile = proc.displacement_profile(&reading);
        let zones = proc.detect_shear_zones(&profile, 0.1);
        assert!(!zones.is_empty());
        assert!((zones[0].depth_m - 5.0).abs() < 0.01); // index 10 * 0.5m
    }

    #[test]
    fn test_find_inflection_points() {
        let proc = InclinometerProcessor::new(default_config());
        // Create a cubic-like profile: concave up then concave down
        // f(x) = x^3 - 3*x^2 has inflection at x=1
        let cum: Vec<f64> = (0..20)
            .map(|i| {
                let x = i as f64 / 10.0; // 0..2
                x * x * x - 3.0 * x * x
            })
            .collect();
        let inflections = proc.find_inflection_points(&cum);
        // Cubic has an inflection point where second derivative changes sign
        assert!(!inflections.is_empty(), "Should detect inflection in cubic profile");
    }

    #[test]
    fn test_find_inflection_points_short() {
        let proc = InclinometerProcessor::new(default_config());
        let cum = vec![1.0, 2.0];
        let inflections = proc.find_inflection_points(&cum);
        assert!(inflections.is_empty());
    }

    #[test]
    fn test_verify_checksum_perfect() {
        let proc = InclinometerProcessor::new(default_config());
        let depths = vec![0.0, 0.5, 1.0];
        let a_plus = vec![0.01, 0.02, 0.015];
        let a_minus = vec![-0.01, -0.02, -0.015]; // Perfect: sum = 0
        let result = proc.verify_checksum(&depths, &a_plus, &a_minus, 0.001);
        assert!(result.pass.iter().all(|&p| p));
        for &cs in &result.checksum {
            assert!(cs.abs() < 1e-10);
        }
    }

    #[test]
    fn test_verify_checksum_with_error() {
        let proc = InclinometerProcessor::new(default_config());
        let depths = vec![0.0, 0.5, 1.0];
        let a_plus = vec![0.01, 0.02, 0.015];
        let a_minus = vec![-0.009, -0.02, -0.013]; // Error at indices 0 and 2
        let result = proc.verify_checksum(&depths, &a_plus, &a_minus, 0.0005);
        // Index 0: checksum = 0.001 > 0.0005 → fail
        assert!(!result.pass[0]);
        // Index 1: checksum = 0 → pass
        assert!(result.pass[1]);
    }

    #[test]
    fn test_checksum_bias_and_sensitivity() {
        let proc = InclinometerProcessor::new(default_config());
        let depths = vec![0.0];
        let a_plus = vec![0.03];
        let a_minus = vec![-0.01];
        let result = proc.verify_checksum(&depths, &a_plus, &a_minus, 1.0);
        // Bias = (0.03 + (-0.01))/2 = 0.01
        assert!((result.bias[0] - 0.01).abs() < 1e-10);
        // Sensitivity = (0.03 - (-0.01))/2 = 0.02
        assert!((result.sensitivity[0] - 0.02).abs() < 1e-10);
    }

    #[test]
    fn test_data_quality_no_a_minus() {
        let proc = InclinometerProcessor::new(default_config());
        let reading = make_reading(10, 0.001, 0.0, 0.0);
        let report = proc.data_quality_check(&reading, 0.001, None);
        assert!((report.checksum_pass_rate - 1.0).abs() < 1e-10);
        assert_eq!(report.missing_count, 0);
    }

    #[test]
    fn test_data_quality_with_nan() {
        let proc = InclinometerProcessor::new(default_config());
        let mut reading = make_reading(10, 0.001, 0.0, 0.0);
        reading.tilt_a_rad[3] = f64::NAN;
        reading.tilt_a_rad[7] = f64::INFINITY;
        let report = proc.data_quality_check(&reading, 0.001, None);
        assert_eq!(report.missing_count, 2);
    }

    #[test]
    fn test_interpolate_missing() {
        let mut values = vec![1.0, f64::NAN, f64::NAN, 4.0, 5.0];
        InclinometerProcessor::interpolate_missing(&mut values);
        // Interpolated values between 1.0 and 4.0
        assert!(!values[1].is_nan());
        assert!(!values[2].is_nan());
        assert!(values[1] > 1.0 && values[1] < 4.0);
        assert!(values[2] > values[1] && values[2] < 4.0);
    }

    #[test]
    fn test_interpolate_missing_no_nans() {
        let mut values = vec![1.0, 2.0, 3.0];
        InclinometerProcessor::interpolate_missing(&mut values);
        assert!((values[0] - 1.0).abs() < 1e-10);
        assert!((values[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_velocity_class_extremely_slow() {
        // 1 mm/year in mm/s
        let v = 1.0 / (365.25 * 86400.0);
        assert_eq!(VelocityClass::from_mm_per_s(v), VelocityClass::ExtremelySlow);
    }

    #[test]
    fn test_velocity_class_very_slow() {
        // 100 mm/year in mm/s
        let v = 100.0 / (365.25 * 86400.0);
        assert_eq!(VelocityClass::from_mm_per_s(v), VelocityClass::VerySlow);
    }

    #[test]
    fn test_velocity_class_slow() {
        // 5 m/month = 5000 mm/month in mm/s
        let v = 5000.0 / (30.44 * 86400.0);
        assert_eq!(VelocityClass::from_mm_per_s(v), VelocityClass::Slow);
    }

    #[test]
    fn test_velocity_class_moderate() {
        // 1 m/hour in mm/s = 1000/3600 ≈ 0.278
        let v = 1000.0 / 3600.0;
        assert_eq!(VelocityClass::from_mm_per_s(v), VelocityClass::Moderate);
    }

    #[test]
    fn test_velocity_class_extremely_rapid() {
        let v = 10_000.0; // 10 m/s
        assert_eq!(
            VelocityClass::from_mm_per_s(v),
            VelocityClass::ExtremelyRapid
        );
    }

    #[test]
    fn test_velocity_class_label() {
        assert_eq!(VelocityClass::Moderate.label(), "Moderate");
        assert_eq!(VelocityClass::ExtremelyRapid.label(), "Extremely rapid");
        assert_eq!(VelocityClass::ExtremelySlow.label(), "Extremely slow");
    }

    #[test]
    fn test_alarm_normal() {
        let proc = InclinometerProcessor::new(default_config());
        let thresholds = AlarmThresholds {
            displacement_mm: [10.0, 25.0, 50.0, 100.0],
            velocity_mm_per_day: [0.5, 1.0, 2.0, 5.0],
            acceleration_mm_per_day2: [0.1, 0.2, 0.5, 1.0],
        };
        let level = proc.evaluate_alarm(&thresholds, 5.0, 0.2, 0.05);
        assert_eq!(level, AlarmLevel::Normal);
    }

    #[test]
    fn test_alarm_attention() {
        let proc = InclinometerProcessor::new(default_config());
        let thresholds = AlarmThresholds {
            displacement_mm: [10.0, 25.0, 50.0, 100.0],
            velocity_mm_per_day: [0.5, 1.0, 2.0, 5.0],
            acceleration_mm_per_day2: [0.1, 0.2, 0.5, 1.0],
        };
        let level = proc.evaluate_alarm(&thresholds, 15.0, 0.2, 0.05);
        assert_eq!(level, AlarmLevel::Attention);
    }

    #[test]
    fn test_alarm_evacuation() {
        let proc = InclinometerProcessor::new(default_config());
        let thresholds = AlarmThresholds {
            displacement_mm: [10.0, 25.0, 50.0, 100.0],
            velocity_mm_per_day: [0.5, 1.0, 2.0, 5.0],
            acceleration_mm_per_day2: [0.1, 0.2, 0.5, 1.0],
        };
        let level = proc.evaluate_alarm(&thresholds, 150.0, 0.2, 0.05);
        assert_eq!(level, AlarmLevel::Evacuation);
    }

    #[test]
    fn test_alarm_velocity_trigger() {
        let proc = InclinometerProcessor::new(default_config());
        let thresholds = AlarmThresholds {
            displacement_mm: [10.0, 25.0, 50.0, 100.0],
            velocity_mm_per_day: [0.5, 1.0, 2.0, 5.0],
            acceleration_mm_per_day2: [0.1, 0.2, 0.5, 1.0],
        };
        // Displacement normal but velocity triggers Warning
        let level = proc.evaluate_alarm(&thresholds, 5.0, 1.5, 0.05);
        assert_eq!(level, AlarmLevel::Warning);
    }

    #[test]
    fn test_alarm_level_ordering() {
        assert!(AlarmLevel::Normal < AlarmLevel::Attention);
        assert!(AlarmLevel::Warning < AlarmLevel::Alarm);
        assert!(AlarmLevel::Alarm < AlarmLevel::Evacuation);
    }

    #[test]
    fn test_time_series_at_depth() {
        let proc = InclinometerProcessor::new(default_config());
        let readings: Vec<InclinometerReading> = (0..5)
            .map(|i| {
                let tilt = 0.001 * (i + 1) as f64;
                make_reading(10, tilt, 0.0, i as f64 * 86400.0)
            })
            .collect();
        let ts = proc.time_series_at_depth(&readings, 0);
        assert_eq!(ts.time_s.len(), 5);
        assert_eq!(ts.displacement_mm.len(), 5);
        // Displacement should increase
        for i in 1..5 {
            assert!(ts.displacement_mm[i] > ts.displacement_mm[i - 1]);
        }
        // Trend slope should be positive
        assert!(ts.trend_slope_mm_per_day > 0.0);
    }

    #[test]
    fn test_time_series_velocity() {
        let proc = InclinometerProcessor::new(default_config());
        let readings: Vec<InclinometerReading> = (0..4)
            .map(|i| {
                let tilt = 0.001 * (i + 1) as f64;
                make_reading(10, tilt, 0.0, i as f64 * 86400.0)
            })
            .collect();
        let ts = proc.time_series_at_depth(&readings, 0);
        // Velocity should be positive for increasing displacement
        for i in 1..ts.velocity_mm_per_day.len() {
            assert!(ts.velocity_mm_per_day[i] > 0.0);
        }
    }

    #[test]
    fn test_seasonal_correction_fit() {
        let proc = InclinometerProcessor::new(default_config());
        let period = 365.25 * 86400.0;
        let n = 100;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * period / n as f64).collect();
        let amp = 5.0;
        let phase = 0.3;
        let mean = 20.0;
        let displacements: Vec<f64> = times
            .iter()
            .map(|&t| mean + amp * (2.0 * PI / period * t + phase).cos())
            .collect();

        let correction = proc.fit_seasonal_correction(&times, &displacements);
        assert!((correction.amplitude_mm - amp).abs() < 0.5);
        assert!((correction.mean_mm - mean).abs() < 0.5);
    }

    #[test]
    fn test_remove_seasonal() {
        let proc = InclinometerProcessor::new(default_config());
        let period = 365.25 * 86400.0;
        let correction = SeasonalCorrection {
            amplitude_mm: 3.0,
            phase_rad: 0.0,
            period_s: period,
            mean_mm: 10.0,
        };
        let times = vec![0.0, period / 4.0, period / 2.0];
        let displacements = vec![13.0, 10.0, 7.0]; // cos: 3, 0, -3 + mean 10
        let corrected = proc.remove_seasonal(&times, &displacements, &correction);
        // After removing seasonal, should be close to the mean
        for &v in &corrected {
            assert!((v - 10.0).abs() < 0.5);
        }
    }

    #[test]
    fn test_inverse_velocity_analysis() {
        let proc = InclinometerProcessor::new(default_config());
        // Accelerating: velocity increases linearly → 1/v decreases linearly
        let times: Vec<f64> = (0..10).map(|i| i as f64 * 86400.0).collect();
        let velocities: Vec<f64> = (0..10).map(|i| 0.5 + 0.1 * i as f64).collect();
        let result = proc.inverse_velocity_analysis(&times, &velocities);
        assert!(!result.inverse_velocity.is_empty());
        // 1/v should decrease as velocity increases
        assert!(result.inverse_velocity.last().unwrap() < &result.inverse_velocity[0]);
    }

    #[test]
    fn test_inverse_velocity_empty() {
        let proc = InclinometerProcessor::new(default_config());
        let result = proc.inverse_velocity_analysis(&[], &[]);
        assert!(result.inverse_velocity.is_empty());
        assert!(result.failure_time_s.is_none());
    }

    #[test]
    fn test_displacement_rate_fos() {
        let proc = InclinometerProcessor::new(default_config());
        // Current velocity half of critical → FOS = 2.0
        let fos = proc.displacement_rate_fos(1.0, 2.0);
        assert!((fos - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_displacement_rate_fos_zero_velocity() {
        let proc = InclinometerProcessor::new(default_config());
        let fos = proc.displacement_rate_fos(0.0, 2.0);
        assert!(fos.is_infinite());
    }

    #[test]
    fn test_resultant_displacement() {
        let cum_a = vec![3.0, 0.0, -3.0];
        let cum_b = vec![4.0, 5.0, 4.0];
        let results = InclinometerProcessor::resultant_displacement(&cum_a, &cum_b);
        assert!((results[0].0 - 5.0).abs() < 1e-10); // sqrt(9+16) = 5
        assert!((results[1].0 - 5.0).abs() < 1e-10); // sqrt(0+25) = 5
        assert!((results[2].0 - 5.0).abs() < 1e-10); // sqrt(9+16) = 5
    }

    #[test]
    fn test_resultant_direction() {
        let cum_a = vec![1.0];
        let cum_b = vec![1.0];
        let results = InclinometerProcessor::resultant_displacement(&cum_a, &cum_b);
        // atan2(1, 1) = pi/4
        assert!((results[0].1 - PI / 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_vector_plot_data() {
        let proc = InclinometerProcessor::new(default_config());
        let reading = make_reading(5, 0.002, 0.001, 0.0);
        let profile = proc.displacement_profile(&reading);
        let plot_data = proc.vector_plot_data(&profile);
        assert_eq!(plot_data.len(), 5);
        // Each entry: (depth, magnitude, direction)
        for (i, &(d, m, _dir)) in plot_data.iter().enumerate() {
            assert!((d - i as f64 * 0.5).abs() < 1e-10);
            assert!(m >= 0.0);
        }
    }

    #[test]
    fn test_linear_regression_perfect_line() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (slope, intercept) = linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < 1e-10);
        assert!((intercept - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_regression_flat() {
        let x = vec![1.0, 2.0, 3.0];
        let y = vec![5.0, 5.0, 5.0];
        let (slope, intercept) = linear_regression(&x, &y);
        assert!(slope.abs() < 1e-10);
        assert!((intercept - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_spiral_correction_no_rotation() {
        let a = vec![0.01, 0.01, 0.01, 0.01];
        let b = vec![0.0, 0.0, 0.0, 0.0];
        let corr = compute_spiral_correction(&a, &b);
        assert_eq!(corr.len(), 4);
        // No rotation expected
        for &c in &corr {
            assert!(c.abs() < 0.1);
        }
    }

    #[test]
    fn test_detect_drift_no_drift() {
        // Random-ish tilts with no trend
        let tilts = vec![0.001, -0.001, 0.002, -0.002, 0.001, -0.001, 0.0005, -0.0015];
        assert!(!detect_systematic_drift(&tilts));
    }

    #[test]
    fn test_detect_drift_present() {
        // Monotonically increasing tilt → strong drift
        let tilts: Vec<f64> = (0..20).map(|i| 0.001 * i as f64).collect();
        assert!(detect_systematic_drift(&tilts));
    }

    #[test]
    fn test_shear_zone_dominant_axis() {
        let proc = InclinometerProcessor::new(default_config());
        let mut reading = make_reading(10, 0.0001, 0.0001, 0.0);
        // Large B-axis tilt at index 5
        reading.tilt_b_rad[5] = 0.1;
        let profile = proc.displacement_profile(&reading);
        let zones = proc.detect_shear_zones(&profile, 0.1);
        assert!(!zones.is_empty());
        assert_eq!(zones[0].dominant_axis, 'B');
    }

    #[test]
    fn test_alarm_level_labels() {
        assert_eq!(AlarmLevel::Normal.label(), "Normal");
        assert_eq!(AlarmLevel::Attention.label(), "Attention");
        assert_eq!(AlarmLevel::Warning.label(), "Warning");
        assert_eq!(AlarmLevel::Alarm.label(), "Alarm");
        assert_eq!(AlarmLevel::Evacuation.label(), "Evacuation");
    }

    #[test]
    fn test_velocity_class_negative() {
        // Negative velocity should also classify correctly
        let v = -100.0 / (365.25 * 86400.0);
        assert_eq!(VelocityClass::from_mm_per_s(v), VelocityClass::VerySlow);
    }
}

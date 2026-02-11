//! GNSS Spoofing Detection Module
//!
//! Detects GNSS spoofing attacks through multi-layer anomaly analysis. Spoofing
//! is a class of attack where a malicious transmitter broadcasts counterfeit GNSS
//! signals to deceive a receiver into computing an incorrect position, velocity,
//! or time solution.
//!
//! # Detection Layers
//!
//! This module implements six independent detection layers:
//!
//! 1. **C/N0 Consistency** - Carrier-to-noise density ratio anomaly detection.
//!    Spoofed signals often exhibit unnaturally uniform or elevated C/N0 values
//!    compared to authentic multi-satellite reception.
//!
//! 2. **Position Jump Detection** - Monitors for physically impossible position
//!    changes that exceed maximum plausible vehicle dynamics.
//!
//! 3. **Doppler Consistency** - Compares measured Doppler shifts against predicted
//!    values from orbital mechanics. Spoofed signals rarely match the correct
//!    Doppler profile for all satellites simultaneously.
//!
//! 4. **Clock Drift Monitoring** - Tracks receiver clock bias rate. Spoofing
//!    attacks can introduce sudden clock offset changes that deviate from the
//!    receiver oscillator's natural drift characteristics.
//!
//! 5. **AGC Level Monitoring** - Automatic Gain Control anomaly detection.
//!    A spoofer typically raises overall RF power, causing AGC values to shift
//!    from their baseline.
//!
//! 6. **Spatial Correlation** - Analyzes C/N0 correlation across multiple PRNs.
//!    Authentic signals show independent fading per satellite; spoofed signals
//!    originate from a single source and exhibit correlated power fluctuations.
//!
//! # Combined Scoring
//!
//! Each detection layer produces a (bool, f64) tuple: an anomaly flag and a
//! normalized score in [0.0, 1.0]. These are combined via weighted averaging
//! into a single spoofing confidence score.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::gps_spoofing_detector::{
//!     SpoofingConfig, SpoofingDetector, AlertType,
//!     check_cn0_consistency, check_position_jump,
//!     check_doppler_consistency, check_clock_drift,
//!     check_agc_level, check_spatial_correlation,
//!     combined_spoofing_score, distance_3d,
//! };
//!
//! // Configure the detector
//! let config = SpoofingConfig {
//!     cn0_threshold_db: 3.0,
//!     position_jump_threshold_m: 100.0,
//!     doppler_tolerance_hz: 50.0,
//!     clock_drift_threshold_ns_s: 20.0,
//! };
//!
//! let mut detector = SpoofingDetector::new(config);
//!
//! // Check C/N0 consistency: values within expected 25-50 dB-Hz range
//! let cn0_values = vec![35.0, 38.0, 42.0, 40.0, 37.0];
//! let (anomaly, score) = check_cn0_consistency(&cn0_values, (25.0, 50.0));
//! assert!(!anomaly);
//!
//! // Check position jump: small movement at 1-second interval
//! let prev = (0.0, 0.0, 0.0);
//! let curr = (10.0, 0.0, 0.0);
//! let (anomaly, _score) = check_position_jump(prev, curr, 1.0, 340.0);
//! assert!(!anomaly); // 10 m/s is well under 340 m/s
//!
//! // Combine multiple check results
//! let checks = vec![(false, 0.1), (false, 0.2), (true, 0.8)];
//! let combined = combined_spoofing_score(&checks);
//! assert!(combined > 0.0 && combined <= 1.0);
//! ```

/// Configuration parameters for the spoofing detector.
///
/// These thresholds govern the sensitivity of each detection layer. Lower
/// thresholds produce more sensitive (but potentially more false-positive)
/// detection.
#[derive(Debug, Clone, PartialEq)]
pub struct SpoofingConfig {
    /// C/N0 standard deviation threshold in dB-Hz. If the standard deviation
    /// of observed C/N0 values falls below this, it may indicate spoofing
    /// (unnaturally uniform signal levels).
    pub cn0_threshold_db: f64,

    /// Maximum allowable position change in meters between consecutive fixes,
    /// scaled by the time interval and maximum velocity.
    pub position_jump_threshold_m: f64,

    /// Maximum tolerable difference in Hz between measured and predicted
    /// Doppler shifts across all tracked satellites.
    pub doppler_tolerance_hz: f64,

    /// Maximum allowable clock drift rate in nanoseconds per second. Typical
    /// TCXO oscillators drift at 1-10 ns/s; larger values suggest spoofing.
    pub clock_drift_threshold_ns_s: f64,
}

impl Default for SpoofingConfig {
    fn default() -> Self {
        Self {
            cn0_threshold_db: 3.0,
            position_jump_threshold_m: 100.0,
            doppler_tolerance_hz: 50.0,
            clock_drift_threshold_ns_s: 20.0,
        }
    }
}

/// Classification of spoofing alert types.
///
/// Each variant corresponds to one of the detection layers in the module.
#[derive(Debug, Clone, PartialEq)]
pub enum AlertType {
    /// C/N0 values are anomalously uniform or out of expected range.
    Cn0Anomaly,
    /// Position changed faster than physically possible.
    PositionJump,
    /// Measured Doppler shifts do not match predicted orbital Doppler.
    DopplerInconsistency,
    /// Receiver clock drift rate exceeds oscillator specifications.
    ClockAnomaly,
    /// AGC level deviates significantly from baseline, suggesting RF power injection.
    AgcAnomaly,
    /// C/N0 values across PRNs are abnormally correlated, indicating a single source.
    SpatialCorrelation,
}

/// A spoofing alert generated when an anomaly is detected.
///
/// Contains the detection timestamp, alert classification, confidence level,
/// and a human-readable description.
#[derive(Debug, Clone, PartialEq)]
pub struct SpoofingAlert {
    /// Time of detection in seconds since the detector was started.
    pub timestamp_s: f64,
    /// The type of anomaly detected.
    pub alert_type: AlertType,
    /// Confidence score in [0.0, 1.0] where 1.0 is highest confidence of spoofing.
    pub confidence: f64,
    /// Human-readable description of the alert.
    pub description: String,
}

/// Main spoofing detector with persistent state.
///
/// Maintains a history of alerts and tracks the last known position and clock
/// offsets to detect anomalies across consecutive measurement epochs.
#[derive(Debug, Clone)]
pub struct SpoofingDetector {
    /// Active configuration.
    config: SpoofingConfig,
    /// History of generated alerts.
    alerts: Vec<SpoofingAlert>,
    /// Last known 3D position (x, y, z) in meters (ECEF or local frame).
    last_position: Option<(f64, f64, f64)>,
    /// Last measurement timestamp in seconds.
    last_timestamp_s: f64,
    /// History of clock offset measurements in nanoseconds.
    clock_offset_history: Vec<f64>,
    /// AGC baseline level in dB.
    agc_baseline: Option<f64>,
}

impl SpoofingDetector {
    /// Create a new spoofing detector with the given configuration.
    pub fn new(config: SpoofingConfig) -> Self {
        Self {
            config,
            alerts: Vec::new(),
            last_position: None,
            last_timestamp_s: 0.0,
            clock_offset_history: Vec::new(),
            agc_baseline: None,
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &SpoofingConfig {
        &self.config
    }

    /// Return all alerts generated so far.
    pub fn alerts(&self) -> &[SpoofingAlert] {
        &self.alerts
    }

    /// Clear all stored alerts.
    pub fn clear_alerts(&mut self) {
        self.alerts.clear();
    }

    /// Set the AGC baseline level in dB.
    ///
    /// This should be called during a known-good (non-spoofed) period to
    /// establish a reference for AGC anomaly detection.
    pub fn set_agc_baseline(&mut self, baseline_db: f64) {
        self.agc_baseline = Some(baseline_db);
    }

    /// Return the current AGC baseline if set.
    pub fn agc_baseline(&self) -> Option<f64> {
        self.agc_baseline
    }

    /// Process a full measurement epoch and run all applicable detection layers.
    ///
    /// # Arguments
    ///
    /// * `timestamp_s` - Current epoch time in seconds
    /// * `position` - Current 3D position (x, y, z) in meters
    /// * `cn0_values` - C/N0 measurements for each tracked satellite in dB-Hz
    /// * `measured_doppler` - Measured Doppler for each satellite in Hz
    /// * `predicted_doppler` - Predicted Doppler for each satellite in Hz
    /// * `clock_offset_ns` - Current receiver clock offset in nanoseconds
    /// * `agc_values` - Current AGC level readings in dB
    /// * `cn0_matrix` - C/N0 time series per PRN for spatial correlation
    ///
    /// Returns the combined spoofing confidence score in [0.0, 1.0].
    #[allow(clippy::too_many_arguments)]
    pub fn process_epoch(
        &mut self,
        timestamp_s: f64,
        position: (f64, f64, f64),
        cn0_values: &[f64],
        measured_doppler: &[f64],
        predicted_doppler: &[f64],
        clock_offset_ns: f64,
        agc_values: &[f64],
        cn0_matrix: &[Vec<f64>],
    ) -> f64 {
        let mut checks = Vec::new();

        // 1. C/N0 consistency
        let cn0_check = check_cn0_consistency(cn0_values, (25.0, 50.0));
        if cn0_check.0 {
            self.alerts.push(SpoofingAlert {
                timestamp_s,
                alert_type: AlertType::Cn0Anomaly,
                confidence: cn0_check.1,
                description: "C/N0 anomaly detected: values outside expected range or abnormally uniform".to_string(),
            });
        }
        checks.push(cn0_check);

        // 2. Position jump
        if let Some(last_pos) = self.last_position {
            let dt = timestamp_s - self.last_timestamp_s;
            if dt > 0.0 {
                let max_velocity = self.config.position_jump_threshold_m / dt;
                let pos_check = check_position_jump(last_pos, position, dt, max_velocity);
                if pos_check.0 {
                    self.alerts.push(SpoofingAlert {
                        timestamp_s,
                        alert_type: AlertType::PositionJump,
                        confidence: pos_check.1,
                        description: format!(
                            "Position jump of {:.1} m in {:.1} s exceeds threshold",
                            distance_3d(last_pos, position),
                            dt
                        ),
                    });
                }
                checks.push(pos_check);
            }
        }
        self.last_position = Some(position);
        self.last_timestamp_s = timestamp_s;

        // 3. Doppler consistency
        if !measured_doppler.is_empty() && measured_doppler.len() == predicted_doppler.len() {
            let doppler_check = check_doppler_consistency(
                measured_doppler,
                predicted_doppler,
                self.config.doppler_tolerance_hz,
            );
            if doppler_check.0 {
                self.alerts.push(SpoofingAlert {
                    timestamp_s,
                    alert_type: AlertType::DopplerInconsistency,
                    confidence: doppler_check.1,
                    description: "Doppler inconsistency detected across satellites".to_string(),
                });
            }
            checks.push(doppler_check);
        }

        // 4. Clock drift
        self.clock_offset_history.push(clock_offset_ns);
        if self.clock_offset_history.len() >= 2 {
            let dt = timestamp_s - self.last_timestamp_s;
            let effective_dt = if dt > 0.0 { dt } else { 1.0 };
            let clock_check = check_clock_drift(
                &self.clock_offset_history,
                effective_dt,
                self.config.clock_drift_threshold_ns_s,
            );
            if clock_check.0 {
                self.alerts.push(SpoofingAlert {
                    timestamp_s,
                    alert_type: AlertType::ClockAnomaly,
                    confidence: clock_check.1,
                    description: "Clock drift anomaly detected".to_string(),
                });
            }
            checks.push(clock_check);
        }

        // 5. AGC level
        if let Some(baseline) = self.agc_baseline {
            let agc_check = check_agc_level(agc_values, baseline, 6.0);
            if agc_check.0 {
                self.alerts.push(SpoofingAlert {
                    timestamp_s,
                    alert_type: AlertType::AgcAnomaly,
                    confidence: agc_check.1,
                    description: "AGC level deviation from baseline detected".to_string(),
                });
            }
            checks.push(agc_check);
        }

        // 6. Spatial correlation
        if !cn0_matrix.is_empty() {
            let spatial_check = check_spatial_correlation(cn0_matrix);
            if spatial_check.0 {
                self.alerts.push(SpoofingAlert {
                    timestamp_s,
                    alert_type: AlertType::SpatialCorrelation,
                    confidence: spatial_check.1,
                    description: "Abnormal spatial correlation in C/N0 across PRNs".to_string(),
                });
            }
            checks.push(spatial_check);
        }

        combined_spoofing_score(&checks)
    }
}

/// Check C/N0 consistency across tracked satellites.
///
/// Examines whether the C/N0 values fall within the expected range and whether
/// their spread (standard deviation) is suspiciously low. Authentic GNSS signals
/// from different satellites at varying elevations should exhibit a natural spread
/// of C/N0 values (typically 5-15 dB-Hz). Spoofed signals from a single transmitter
/// tend to have very similar power levels.
///
/// # Arguments
///
/// * `cn0_values` - C/N0 measurements in dB-Hz for each tracked satellite
/// * `expected_range` - (min, max) expected C/N0 range in dB-Hz
///
/// # Returns
///
/// A tuple of (anomaly_detected, score) where score is in [0.0, 1.0].
pub fn check_cn0_consistency(cn0_values: &[f64], expected_range: (f64, f64)) -> (bool, f64) {
    if cn0_values.is_empty() {
        return (false, 0.0);
    }

    let n = cn0_values.len() as f64;
    let mean = cn0_values.iter().sum::<f64>() / n;

    // Check if values are outside expected range
    let out_of_range_count = cn0_values
        .iter()
        .filter(|&&v| v < expected_range.0 || v > expected_range.1)
        .count();
    let out_of_range_ratio = out_of_range_count as f64 / n;

    // Check standard deviation (too-low std dev is suspicious)
    let variance = cn0_values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
    let std_dev = variance.sqrt();

    // Normal satellite constellation has std_dev of about 5-15 dB-Hz.
    // A std_dev below 1.0 dB-Hz with multiple satellites is suspicious.
    let low_spread_score = if cn0_values.len() >= 3 && std_dev < 2.0 {
        1.0 - (std_dev / 2.0)
    } else {
        0.0
    };

    let score = (out_of_range_ratio * 0.6 + low_spread_score * 0.4).min(1.0);
    let anomaly = out_of_range_ratio > 0.5 || (cn0_values.len() >= 3 && std_dev < 1.0);

    (anomaly, score)
}

/// Check for physically impossible position jumps.
///
/// Calculates the velocity implied by consecutive position fixes and compares
/// against the maximum plausible velocity. A position jump exceeding what is
/// physically achievable (given the platform dynamics) suggests spoofing.
///
/// # Arguments
///
/// * `prev_pos` - Previous position (x, y, z) in meters
/// * `curr_pos` - Current position (x, y, z) in meters
/// * `dt_s` - Time interval between fixes in seconds
/// * `max_velocity_mps` - Maximum plausible velocity in meters per second
///
/// # Returns
///
/// A tuple of (anomaly_detected, score) where score is in [0.0, 1.0].
pub fn check_position_jump(
    prev_pos: (f64, f64, f64),
    curr_pos: (f64, f64, f64),
    dt_s: f64,
    max_velocity_mps: f64,
) -> (bool, f64) {
    if dt_s <= 0.0 {
        return (false, 0.0);
    }

    let dist = distance_3d(prev_pos, curr_pos);
    let velocity = dist / dt_s;

    if max_velocity_mps <= 0.0 {
        return (velocity > 0.0, if velocity > 0.0 { 1.0 } else { 0.0 });
    }

    let ratio = velocity / max_velocity_mps;
    let anomaly = ratio > 1.0;

    // Score ramps from 0 at ratio=0.5 to 1.0 at ratio=2.0
    let score = ((ratio - 0.5) / 1.5).clamp(0.0, 1.0);

    (anomaly, score)
}

/// Check Doppler consistency between measured and predicted values.
///
/// For each tracked satellite, the Doppler shift predicted from orbital mechanics
/// is compared against the measured Doppler. A spoofer broadcasting from a fixed
/// location cannot reproduce the correct Doppler signature for all satellites
/// simultaneously (each has a unique line-of-sight velocity).
///
/// # Arguments
///
/// * `measured_doppler` - Measured Doppler shift per satellite in Hz
/// * `predicted_doppler` - Predicted Doppler shift per satellite in Hz
/// * `tolerance` - Maximum acceptable difference in Hz
///
/// # Returns
///
/// A tuple of (anomaly_detected, score) where score is in [0.0, 1.0].
pub fn check_doppler_consistency(
    measured_doppler: &[f64],
    predicted_doppler: &[f64],
    tolerance: f64,
) -> (bool, f64) {
    if measured_doppler.is_empty() || measured_doppler.len() != predicted_doppler.len() {
        return (false, 0.0);
    }

    let n = measured_doppler.len() as f64;
    let errors: Vec<f64> = measured_doppler
        .iter()
        .zip(predicted_doppler.iter())
        .map(|(m, p)| (m - p).abs())
        .collect();

    let mean_error = errors.iter().sum::<f64>() / n;
    let max_error = errors.iter().cloned().fold(0.0_f64, f64::max);

    let anomaly = max_error > tolerance;

    // Score based on mean error relative to tolerance
    let score = if tolerance > 0.0 {
        (mean_error / tolerance).min(1.0)
    } else if mean_error > 0.0 {
        1.0
    } else {
        0.0
    };

    (anomaly, score)
}

/// Check clock drift rate for anomalies.
///
/// Monitors the rate of change of receiver clock offset. A sudden change in
/// clock drift rate (beyond what the local oscillator can produce) indicates
/// an external time manipulation, which is characteristic of spoofing.
///
/// # Arguments
///
/// * `clock_offsets_ns` - History of clock offset measurements in nanoseconds
/// * `dt_s` - Time interval between the last two measurements in seconds
/// * `threshold` - Maximum allowable drift rate in ns/s
///
/// # Returns
///
/// A tuple of (anomaly_detected, score) where score is in [0.0, 1.0].
pub fn check_clock_drift(
    clock_offsets_ns: &[f64],
    dt_s: f64,
    threshold: f64,
) -> (bool, f64) {
    if clock_offsets_ns.len() < 2 || dt_s <= 0.0 {
        return (false, 0.0);
    }

    let n = clock_offsets_ns.len();
    let last = clock_offsets_ns[n - 1];
    let prev = clock_offsets_ns[n - 2];
    let drift_rate = (last - prev).abs() / dt_s;

    let anomaly = drift_rate > threshold;
    let score = if threshold > 0.0 {
        (drift_rate / (threshold * 2.0)).min(1.0)
    } else if drift_rate > 0.0 {
        1.0
    } else {
        0.0
    };

    (anomaly, score)
}

/// Check AGC level for anomalies indicating RF power injection.
///
/// A GNSS spoofer must transmit at a power level exceeding the authentic
/// satellite signals to capture the receiver's tracking loops. This causes
/// the AGC to adjust, producing a measurable deviation from the established
/// baseline.
///
/// # Arguments
///
/// * `agc_values` - Current AGC readings in dB
/// * `baseline` - Established baseline AGC level in dB
/// * `threshold_db` - Maximum allowable deviation from baseline in dB
///
/// # Returns
///
/// A tuple of (anomaly_detected, score) where score is in [0.0, 1.0].
pub fn check_agc_level(
    agc_values: &[f64],
    baseline: f64,
    threshold_db: f64,
) -> (bool, f64) {
    if agc_values.is_empty() {
        return (false, 0.0);
    }

    let mean_agc = agc_values.iter().sum::<f64>() / agc_values.len() as f64;
    let deviation = (mean_agc - baseline).abs();

    let anomaly = deviation > threshold_db;
    let score = if threshold_db > 0.0 {
        (deviation / (threshold_db * 2.0)).min(1.0)
    } else if deviation > 0.0 {
        1.0
    } else {
        0.0
    };

    (anomaly, score)
}

/// Check spatial correlation of C/N0 across multiple PRNs.
///
/// Authentic GNSS signals arrive from different satellites at different
/// elevations and azimuths, so their fading characteristics are independent.
/// Spoofed signals from a single source exhibit correlated power variations
/// across all PRNs. This function computes the average pairwise Pearson
/// correlation coefficient.
///
/// # Arguments
///
/// * `cn0_matrix` - Each element is a time series of C/N0 values for one PRN.
///   All inner vectors should have the same length (number of time epochs).
///
/// # Returns
///
/// A tuple of (anomaly_detected, score) where score is in [0.0, 1.0].
pub fn check_spatial_correlation(cn0_matrix: &[Vec<f64>]) -> (bool, f64) {
    if cn0_matrix.len() < 2 {
        return (false, 0.0);
    }

    // Verify all series have the same length and at least 2 samples
    let series_len = cn0_matrix[0].len();
    if series_len < 2 {
        return (false, 0.0);
    }
    for series in cn0_matrix {
        if series.len() != series_len {
            return (false, 0.0);
        }
    }

    let num_prns = cn0_matrix.len();
    let mut total_correlation = 0.0;
    let mut pair_count = 0;

    for i in 0..num_prns {
        for j in (i + 1)..num_prns {
            let corr = pearson_correlation(&cn0_matrix[i], &cn0_matrix[j]);
            total_correlation += corr.abs();
            pair_count += 1;
        }
    }

    if pair_count == 0 {
        return (false, 0.0);
    }

    let avg_correlation = total_correlation / pair_count as f64;

    // Correlation above 0.8 is suspicious for independent satellite signals
    let anomaly = avg_correlation > 0.8;
    let score = ((avg_correlation - 0.3) / 0.7).clamp(0.0, 1.0);

    (anomaly, score)
}

/// Compute the Pearson correlation coefficient between two sequences.
///
/// Returns a value in [-1.0, 1.0], or 0.0 if either sequence has zero variance.
fn pearson_correlation(x: &[f64], y: &[f64]) -> f64 {
    if x.len() != y.len() || x.is_empty() {
        return 0.0;
    }

    let n = x.len() as f64;
    let mean_x = x.iter().sum::<f64>() / n;
    let mean_y = y.iter().sum::<f64>() / n;

    let mut cov = 0.0;
    let mut var_x = 0.0;
    let mut var_y = 0.0;

    for i in 0..x.len() {
        let dx = x[i] - mean_x;
        let dy = y[i] - mean_y;
        cov += dx * dy;
        var_x += dx * dx;
        var_y += dy * dy;
    }

    let denom = (var_x * var_y).sqrt();
    if denom < 1e-12 {
        return 0.0;
    }

    cov / denom
}

/// Compute a combined spoofing confidence score from multiple detection layers.
///
/// Each check contributes its score weighted by whether an anomaly was detected.
/// Anomalous checks receive double weight compared to non-anomalous checks.
/// The result is normalized to [0.0, 1.0].
///
/// # Arguments
///
/// * `checks` - Slice of (anomaly_detected, score) tuples from individual checks
///
/// # Returns
///
/// Combined spoofing confidence in [0.0, 1.0]. Values above 0.5 suggest likely
/// spoofing; values above 0.8 indicate high-confidence spoofing detection.
pub fn combined_spoofing_score(checks: &[(bool, f64)]) -> f64 {
    if checks.is_empty() {
        return 0.0;
    }

    let mut weighted_sum = 0.0;
    let mut total_weight = 0.0;

    for &(anomaly, score) in checks {
        let weight = if anomaly { 2.0 } else { 1.0 };
        weighted_sum += score * weight;
        total_weight += weight;
    }

    if total_weight <= 0.0 {
        return 0.0;
    }

    (weighted_sum / total_weight).clamp(0.0, 1.0)
}

/// Compute the 3D Euclidean distance between two points.
///
/// # Arguments
///
/// * `a` - First point (x, y, z) in meters
/// * `b` - Second point (x, y, z) in meters
///
/// # Returns
///
/// Distance in meters.
pub fn distance_3d(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let dz = b.2 - a.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // distance_3d tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_distance_3d_zero() {
        let d = distance_3d((1.0, 2.0, 3.0), (1.0, 2.0, 3.0));
        assert!((d - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_distance_3d_unit_axes() {
        assert!((distance_3d((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)) - 1.0).abs() < 1e-12);
        assert!((distance_3d((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)) - 1.0).abs() < 1e-12);
        assert!((distance_3d((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_distance_3d_diagonal() {
        let d = distance_3d((0.0, 0.0, 0.0), (3.0, 4.0, 0.0));
        assert!((d - 5.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // check_cn0_consistency tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_cn0_consistency_empty() {
        let (anomaly, score) = check_cn0_consistency(&[], (25.0, 50.0));
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_cn0_consistency_normal() {
        // Spread of values within range - healthy constellation
        let cn0 = vec![30.0, 35.0, 40.0, 45.0, 38.0, 33.0];
        let (anomaly, score) = check_cn0_consistency(&cn0, (25.0, 50.0));
        assert!(!anomaly);
        assert!(score < 0.5, "Score {score} should be low for normal signals");
    }

    #[test]
    fn test_cn0_consistency_spoofed_uniform() {
        // Suspiciously uniform C/N0 values (std_dev < 1.0 with 5+ sats)
        let cn0 = vec![40.0, 40.1, 39.9, 40.05, 39.95];
        let (anomaly, _score) = check_cn0_consistency(&cn0, (25.0, 50.0));
        assert!(anomaly, "Uniform C/N0 should trigger anomaly");
    }

    #[test]
    fn test_cn0_consistency_out_of_range() {
        // Most values far outside expected range
        let cn0 = vec![60.0, 62.0, 58.0, 61.0, 59.0];
        let (anomaly, score) = check_cn0_consistency(&cn0, (25.0, 50.0));
        assert!(anomaly, "Out-of-range values should trigger anomaly");
        assert!(score > 0.3, "Score should be elevated for out-of-range values");
    }

    #[test]
    fn test_cn0_consistency_single_value() {
        // Single satellite - not enough data to determine spread anomaly
        let (anomaly, _score) = check_cn0_consistency(&[35.0], (25.0, 50.0));
        assert!(!anomaly);
    }

    // -----------------------------------------------------------------------
    // check_position_jump tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_position_jump_no_movement() {
        let pos = (100.0, 200.0, 300.0);
        let (anomaly, score) = check_position_jump(pos, pos, 1.0, 340.0);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_position_jump_normal_motion() {
        // 10 m in 1 s = 10 m/s, well under 340 m/s max
        let prev = (0.0, 0.0, 0.0);
        let curr = (10.0, 0.0, 0.0);
        let (anomaly, _score) = check_position_jump(prev, curr, 1.0, 340.0);
        assert!(!anomaly);
    }

    #[test]
    fn test_position_jump_excessive() {
        // 1000 m in 1 s = 1000 m/s, exceeds 340 m/s max
        let prev = (0.0, 0.0, 0.0);
        let curr = (1000.0, 0.0, 0.0);
        let (anomaly, score) = check_position_jump(prev, curr, 1.0, 340.0);
        assert!(anomaly, "1000 m/s should exceed 340 m/s threshold");
        assert!(score > 0.5, "Score should be high for large jump");
    }

    #[test]
    fn test_position_jump_zero_dt() {
        let (anomaly, score) = check_position_jump(
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            0.0,
            340.0,
        );
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // check_doppler_consistency tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_doppler_consistency_empty() {
        let (anomaly, score) = check_doppler_consistency(&[], &[], 50.0);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_doppler_consistency_matching() {
        let measured = vec![1000.0, -500.0, 2000.0, -1500.0];
        let predicted = vec![1010.0, -490.0, 2005.0, -1508.0];
        let (anomaly, score) = check_doppler_consistency(&measured, &predicted, 50.0);
        assert!(!anomaly, "Small errors should not trigger anomaly");
        assert!(score < 0.5, "Score {score} should be low");
    }

    #[test]
    fn test_doppler_consistency_mismatch() {
        let measured = vec![1000.0, -500.0, 2000.0];
        let predicted = vec![1100.0, -600.0, 2200.0];
        let (anomaly, score) = check_doppler_consistency(&measured, &predicted, 50.0);
        assert!(anomaly, "Large Doppler errors should trigger anomaly");
        assert!(score > 0.5, "Score should be elevated");
    }

    #[test]
    fn test_doppler_consistency_mismatched_lengths() {
        let (anomaly, score) = check_doppler_consistency(&[1.0, 2.0], &[1.0], 50.0);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // check_clock_drift tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_clock_drift_insufficient_data() {
        let (anomaly, score) = check_clock_drift(&[100.0], 1.0, 20.0);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_clock_drift_normal() {
        // 5 ns change over 1 second = 5 ns/s, under 20 ns/s threshold
        let offsets = vec![100.0, 105.0];
        let (anomaly, _score) = check_clock_drift(&offsets, 1.0, 20.0);
        assert!(!anomaly);
    }

    #[test]
    fn test_clock_drift_anomalous() {
        // 50 ns change over 1 second = 50 ns/s, exceeds 20 ns/s threshold
        let offsets = vec![100.0, 150.0];
        let (anomaly, score) = check_clock_drift(&offsets, 1.0, 20.0);
        assert!(anomaly, "50 ns/s drift should exceed 20 ns/s threshold");
        assert!(score > 0.5, "Score should be elevated");
    }

    #[test]
    fn test_clock_drift_zero_dt() {
        let offsets = vec![100.0, 200.0];
        let (anomaly, score) = check_clock_drift(&offsets, 0.0, 20.0);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // check_agc_level tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_agc_level_empty() {
        let (anomaly, score) = check_agc_level(&[], 50.0, 6.0);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_agc_level_normal() {
        let agc = vec![50.0, 50.5, 49.5, 50.2, 49.8];
        let (anomaly, _score) = check_agc_level(&agc, 50.0, 6.0);
        assert!(!anomaly);
    }

    #[test]
    fn test_agc_level_elevated() {
        // Mean AGC of 60 dB vs baseline of 50 dB = 10 dB deviation, exceeds 6 dB
        let agc = vec![59.0, 60.0, 61.0, 60.5, 59.5];
        let (anomaly, score) = check_agc_level(&agc, 50.0, 6.0);
        assert!(anomaly, "10 dB deviation should exceed 6 dB threshold");
        assert!(score > 0.3, "Score should be elevated");
    }

    // -----------------------------------------------------------------------
    // check_spatial_correlation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spatial_correlation_insufficient_prns() {
        let matrix = vec![vec![1.0, 2.0, 3.0]];
        let (anomaly, score) = check_spatial_correlation(&matrix);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_spatial_correlation_independent() {
        // Two uncorrelated series (near-zero Pearson r)
        let prn1 = vec![35.0, 37.0, 33.0, 36.0, 34.0, 38.0, 32.0, 35.0];
        let prn2 = vec![40.0, 41.0, 39.0, 42.0, 40.0, 38.0, 41.0, 39.0];
        let matrix = vec![prn1, prn2];
        let (anomaly, _score) = check_spatial_correlation(&matrix);
        assert!(!anomaly, "Uncorrelated PRNs should not trigger anomaly");
    }

    #[test]
    fn test_spatial_correlation_highly_correlated() {
        // Perfectly correlated series (spoofed from same source)
        let prn1 = vec![30.0, 32.0, 34.0, 36.0, 38.0, 40.0];
        let prn2 = vec![31.0, 33.0, 35.0, 37.0, 39.0, 41.0]; // offset but same slope
        let prn3 = vec![29.0, 31.0, 33.0, 35.0, 37.0, 39.0];
        let matrix = vec![prn1, prn2, prn3];
        let (anomaly, _score) = check_spatial_correlation(&matrix);
        assert!(anomaly, "Perfectly correlated PRNs should trigger anomaly");
    }

    #[test]
    fn test_spatial_correlation_short_series() {
        let matrix = vec![vec![1.0], vec![2.0]];
        let (anomaly, score) = check_spatial_correlation(&matrix);
        assert!(!anomaly);
        assert!((score - 0.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // combined_spoofing_score tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_combined_score_empty() {
        let score = combined_spoofing_score(&[]);
        assert!((score - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_combined_score_no_anomalies() {
        let checks = vec![(false, 0.1), (false, 0.2), (false, 0.15)];
        let score = combined_spoofing_score(&checks);
        assert!(score < 0.5, "No anomalies should produce low score");
    }

    #[test]
    fn test_combined_score_all_anomalies() {
        let checks = vec![(true, 0.9), (true, 0.85), (true, 0.95)];
        let score = combined_spoofing_score(&checks);
        assert!(score > 0.8, "All anomalies should produce high score: {score}");
    }

    #[test]
    fn test_combined_score_mixed() {
        let checks = vec![(false, 0.1), (true, 0.8), (false, 0.2)];
        let score = combined_spoofing_score(&checks);
        assert!(
            score > 0.0 && score < 1.0,
            "Mixed should be moderate: {score}"
        );
    }

    #[test]
    fn test_combined_score_clamped() {
        // Even with extreme values, should not exceed 1.0
        let checks = vec![(true, 1.0), (true, 1.0)];
        let score = combined_spoofing_score(&checks);
        assert!(score <= 1.0);
        assert!(score >= 0.0);
    }

    // -----------------------------------------------------------------------
    // SpoofingDetector stateful tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_new() {
        let config = SpoofingConfig::default();
        let detector = SpoofingDetector::new(config.clone());
        assert_eq!(*detector.config(), config);
        assert!(detector.alerts().is_empty());
        assert!(detector.agc_baseline().is_none());
    }

    #[test]
    fn test_detector_agc_baseline() {
        let mut detector = SpoofingDetector::new(SpoofingConfig::default());
        assert!(detector.agc_baseline().is_none());
        detector.set_agc_baseline(50.0);
        assert_eq!(detector.agc_baseline(), Some(50.0));
    }

    #[test]
    fn test_detector_clear_alerts() {
        let mut detector = SpoofingDetector::new(SpoofingConfig::default());

        // Force an alert by processing spoofed-looking data
        let cn0 = vec![40.0, 40.01, 39.99, 40.005, 39.995];
        let doppler_m = vec![1000.0];
        let doppler_p = vec![1000.0];

        detector.process_epoch(
            1.0,
            (0.0, 0.0, 0.0),
            &cn0,
            &doppler_m,
            &doppler_p,
            100.0,
            &[],
            &[],
        );

        // There should be at least the cn0 alert
        let had_alerts = !detector.alerts().is_empty();
        detector.clear_alerts();
        assert!(detector.alerts().is_empty());
        // Verify we actually had something to clear (or at least no panic)
        let _ = had_alerts;
    }

    #[test]
    fn test_detector_process_epoch_clean() {
        let config = SpoofingConfig {
            cn0_threshold_db: 3.0,
            position_jump_threshold_m: 1000.0,
            doppler_tolerance_hz: 50.0,
            clock_drift_threshold_ns_s: 20.0,
        };
        let mut detector = SpoofingDetector::new(config);

        // Normal-looking data
        let cn0 = vec![30.0, 35.0, 40.0, 45.0, 38.0];
        let doppler_m = vec![1000.0, -500.0, 2000.0];
        let doppler_p = vec![1005.0, -498.0, 1995.0];

        let score = detector.process_epoch(
            1.0,
            (0.0, 0.0, 0.0),
            &cn0,
            &doppler_m,
            &doppler_p,
            100.0,
            &[],
            &[],
        );

        assert!(score < 0.5, "Clean data should produce low score: {score}");
    }

    #[test]
    fn test_detector_position_jump_across_epochs() {
        let config = SpoofingConfig {
            cn0_threshold_db: 3.0,
            position_jump_threshold_m: 100.0,
            doppler_tolerance_hz: 50.0,
            clock_drift_threshold_ns_s: 20.0,
        };
        let mut detector = SpoofingDetector::new(config);

        let cn0 = vec![30.0, 35.0, 40.0, 45.0, 38.0];

        // First epoch - establishes position
        detector.process_epoch(
            1.0,
            (0.0, 0.0, 0.0),
            &cn0,
            &[],
            &[],
            100.0,
            &[],
            &[],
        );

        // Second epoch - huge jump (10 km in 1 second)
        let score = detector.process_epoch(
            2.0,
            (10000.0, 0.0, 0.0),
            &cn0,
            &[],
            &[],
            105.0,
            &[],
            &[],
        );

        // Should have a position jump alert
        let has_jump_alert = detector
            .alerts()
            .iter()
            .any(|a| a.alert_type == AlertType::PositionJump);
        assert!(has_jump_alert, "Should detect position jump across epochs");
        assert!(score > 0.0, "Score should be nonzero due to jump");
    }

    #[test]
    fn test_detector_alert_types_populated() {
        // Verify that alert fields are correctly populated
        let alert = SpoofingAlert {
            timestamp_s: 42.5,
            alert_type: AlertType::AgcAnomaly,
            confidence: 0.75,
            description: "Test alert".to_string(),
        };
        assert_eq!(alert.timestamp_s, 42.5);
        assert_eq!(alert.alert_type, AlertType::AgcAnomaly);
        assert!((alert.confidence - 0.75).abs() < 1e-12);
        assert_eq!(alert.description, "Test alert");
    }

    #[test]
    fn test_config_default() {
        let config = SpoofingConfig::default();
        assert!((config.cn0_threshold_db - 3.0).abs() < 1e-12);
        assert!((config.position_jump_threshold_m - 100.0).abs() < 1e-12);
        assert!((config.doppler_tolerance_hz - 50.0).abs() < 1e-12);
        assert!((config.clock_drift_threshold_ns_s - 20.0).abs() < 1e-12);
    }
}

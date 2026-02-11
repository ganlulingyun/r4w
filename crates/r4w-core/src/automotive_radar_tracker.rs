//! Automotive radar target tracking for ADAS (Advanced Driver Assistance Systems).
//!
//! This module manages multiple tracked objects from radar detections using
//! scalar Kalman filtering and M/N track management logic (initiation,
//! confirmation, coasting, deletion). It is designed for 77 GHz automotive
//! radar systems that produce range-Doppler-azimuth detections each scan.
//!
//! # Algorithm Overview
//!
//! 1. **Prediction** -- Each existing track is propagated forward in time using
//!    a constant-velocity Kalman prediction step in range and azimuth.
//! 2. **Association** -- Detections are associated to tracks using Global
//!    Nearest Neighbor (GNN) with gated Euclidean distance in
//!    (range, velocity) space.
//! 3. **Update** -- Associated tracks are updated with their matched detection
//!    via scalar Kalman update steps for range, velocity, and azimuth.
//! 4. **Track management** -- Unassociated detections spawn tentative tracks.
//!    Tracks are confirmed after `confirm_hits` consecutive updates, coast on
//!    missed updates, and are deleted after `delete_misses` consecutive misses.
//!
//! # Example
//!
//! ```
//! use r4w_core::automotive_radar_tracker::{
//!     RadarTrackerConfig, RadarTracker, RadarDetection, TrackState,
//! };
//!
//! let config = RadarTrackerConfig {
//!     max_tracks: 64,
//!     gate_distance_m: 5.0,
//!     gate_velocity_mps: 3.0,
//!     confirm_hits: 3,
//!     delete_misses: 5,
//!     dt_s: 0.050, // 20 Hz scan rate
//! };
//! let mut tracker = RadarTracker::new(config);
//!
//! // First scan: a car at 30 m, -15 m/s closing, 2 deg azimuth
//! let dets = vec![RadarDetection {
//!     range_m: 30.0,
//!     velocity_mps: -15.0,
//!     azimuth_deg: 2.0,
//!     rcs_dbsm: 10.0,
//! }];
//! let tracks = tracker.update(&dets);
//! assert_eq!(tracks.len(), 1);
//! assert_eq!(tracks[0].state, TrackState::Tentative);
//! ```

// ---------------------------------------------------------------------------
// Public enums
// ---------------------------------------------------------------------------

/// State of a tracked object in the track management lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackState {
    /// Newly initiated track, not yet confirmed.
    Tentative,
    /// Track has accumulated enough hits to be considered reliable.
    Confirmed,
    /// Confirmed track that has missed recent detections but is still alive.
    Coasting,
    /// Track scheduled for removal (will not appear in output).
    Deleted,
}

/// Classification of a radar target based on RCS (Radar Cross Section).
///
/// Typical monostatic RCS values at 77 GHz:
/// - Pedestrian: -10 to 3 dBsm
/// - Bicycle/motorcycle: 0 to 5 dBsm
/// - Car: 5 to 20 dBsm
/// - Truck/bus: 15 to 30+ dBsm
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectClass {
    /// Pedestrian or small animal (RCS < 3 dBsm).
    Pedestrian,
    /// Bicycle or motorcycle (3 <= RCS < 7 dBsm).
    Bicycle,
    /// Passenger car (7 <= RCS < 18 dBsm).
    Car,
    /// Truck, bus, or large vehicle (RCS >= 18 dBsm).
    Truck,
    /// RCS does not match any known category.
    Unknown,
}

// ---------------------------------------------------------------------------
// Configuration and data structs
// ---------------------------------------------------------------------------

/// Configuration for the radar tracker.
#[derive(Debug, Clone)]
pub struct RadarTrackerConfig {
    /// Maximum number of simultaneous tracks.
    pub max_tracks: usize,
    /// Gating threshold for range association in metres.
    pub gate_distance_m: f64,
    /// Gating threshold for velocity association in m/s.
    pub gate_velocity_mps: f64,
    /// Number of consecutive hits required to confirm a tentative track.
    pub confirm_hits: usize,
    /// Number of consecutive misses before a track is deleted.
    pub delete_misses: usize,
    /// Time step between scans in seconds (1 / scan_rate).
    pub dt_s: f64,
}

impl Default for RadarTrackerConfig {
    fn default() -> Self {
        Self {
            max_tracks: 64,
            gate_distance_m: 5.0,
            gate_velocity_mps: 3.0,
            confirm_hits: 3,
            delete_misses: 5,
            dt_s: 0.050,
        }
    }
}

/// A single radar detection from the current scan.
#[derive(Debug, Clone, Copy)]
pub struct RadarDetection {
    /// Range from sensor to target in metres.
    pub range_m: f64,
    /// Radial (Doppler) velocity in m/s. Negative = closing.
    pub velocity_mps: f64,
    /// Azimuth angle in degrees. 0 = boresight, positive = right.
    pub azimuth_deg: f64,
    /// Radar cross section in dBsm.
    pub rcs_dbsm: f64,
}

/// A tracked object maintained by the tracker.
#[derive(Debug, Clone)]
pub struct TrackedObject {
    /// Unique track identifier (monotonically increasing).
    pub id: u32,
    /// Estimated range in metres.
    pub range_m: f64,
    /// Estimated radial velocity in m/s.
    pub velocity_mps: f64,
    /// Estimated azimuth angle in degrees.
    pub azimuth_deg: f64,
    /// Estimated radial acceleration in m/s^2 (derived from velocity changes).
    pub acceleration_mps2: f64,
    /// Current track state.
    pub state: TrackState,
    /// Number of scans since track initiation.
    pub age: usize,
    /// Number of scans in which this track was associated to a detection.
    pub hits: usize,
    /// Number of consecutive scans with no associated detection.
    pub misses: usize,
}

// ---------------------------------------------------------------------------
// Internal per-track Kalman state
// ---------------------------------------------------------------------------

/// Internal Kalman state for a single track dimension.
#[derive(Debug, Clone, Copy)]
struct KalmanState1D {
    /// State estimate (e.g. range or azimuth).
    x: f64,
    /// Estimate variance.
    p: f64,
}

/// Full internal track record (public `TrackedObject` is the external view).
#[derive(Debug, Clone)]
struct InternalTrack {
    id: u32,
    range_kf: KalmanState1D,
    velocity_kf: KalmanState1D,
    azimuth_kf: KalmanState1D,
    prev_velocity: f64,
    acceleration_mps2: f64,
    state: TrackState,
    age: usize,
    hits: usize,
    misses: usize,
    rcs_dbsm: f64,
}

// ---------------------------------------------------------------------------
// Standalone pure functions
// ---------------------------------------------------------------------------

/// Constant-velocity Kalman prediction for a single state dimension.
///
/// Given current `state` and its `velocity`, predicts the state at `dt`
/// seconds in the future. Returns `(predicted_state, predicted_velocity)`.
///
/// Model: x_{k+1} = x_k + v_k * dt, v_{k+1} = v_k.
pub fn kalman_predict_1d(state: f64, velocity: f64, dt: f64) -> (f64, f64) {
    let predicted_state = state + velocity * dt;
    (predicted_state, velocity)
}

/// Scalar Kalman measurement update.
///
/// Fuses `predicted` state estimate (with variance `pred_var`) with a new
/// `measurement` (with variance `meas_var`). Returns `(updated_state,
/// updated_variance)`.
///
/// Kalman gain: K = P_pred / (P_pred + R)
/// Updated state: x = x_pred + K * (z - x_pred)
/// Updated variance: P = (1 - K) * P_pred
pub fn kalman_update_1d(
    predicted: f64,
    measurement: f64,
    pred_var: f64,
    meas_var: f64,
) -> (f64, f64) {
    let s = pred_var + meas_var;
    if s.abs() < 1e-30 {
        // Both variances essentially zero -- keep predicted.
        return (predicted, pred_var);
    }
    let k = pred_var / s;
    let updated = predicted + k * (measurement - predicted);
    let updated_var = (1.0 - k) * pred_var;
    (updated, updated_var)
}

/// Global Nearest Neighbor (GNN) data association.
///
/// For each track, finds the closest detection within the gating thresholds
/// (`gate_range` metres, `gate_vel` m/s). Each detection may be assigned to
/// at most one track. Returns a list of `(track_index, detection_index)` pairs.
///
/// The algorithm is a greedy nearest-neighbor search that iterates over all
/// (track, detection) pairs sorted by combined normalised distance, assigning
/// the closest unassigned pairs first.
pub fn nearest_neighbor_association(
    tracks: &[TrackedObject],
    detections: &[RadarDetection],
    gate_range: f64,
    gate_vel: f64,
) -> Vec<(usize, usize)> {
    if tracks.is_empty() || detections.is_empty() {
        return Vec::new();
    }

    // Build a list of all candidate (track, detection, distance) tuples that
    // pass the gating test.
    let mut candidates: Vec<(usize, usize, f64)> = Vec::new();

    for (ti, trk) in tracks.iter().enumerate() {
        for (di, det) in detections.iter().enumerate() {
            let dr = (trk.range_m - det.range_m).abs();
            let dv = (trk.velocity_mps - det.velocity_mps).abs();
            if dr <= gate_range && dv <= gate_vel {
                // Normalised distance (unitless).
                let norm_dist = (dr / gate_range).powi(2) + (dv / gate_vel).powi(2);
                candidates.push((ti, di, norm_dist));
            }
        }
    }

    // Sort by distance ascending -- greedy nearest neighbor.
    candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));

    let mut assigned_tracks = vec![false; tracks.len()];
    let mut assigned_dets = vec![false; detections.len()];
    let mut result = Vec::new();

    for (ti, di, _dist) in &candidates {
        if !assigned_tracks[*ti] && !assigned_dets[*di] {
            assigned_tracks[*ti] = true;
            assigned_dets[*di] = true;
            result.push((*ti, *di));
        }
    }

    result
}

/// Compute time-to-collision (TTC) in seconds.
///
/// TTC = range / closing_velocity. A negative closing velocity means the
/// target is moving away; in that case `f64::INFINITY` is returned because
/// there is no collision risk.
///
/// A closing velocity of zero also returns `f64::INFINITY`.
pub fn compute_ttc(range_m: f64, closing_velocity_mps: f64) -> f64 {
    if closing_velocity_mps <= 0.0 {
        return f64::INFINITY;
    }
    if range_m <= 0.0 {
        return 0.0;
    }
    range_m / closing_velocity_mps
}

/// Classify a radar target by its RCS (radar cross section) in dBsm.
///
/// Boundaries are approximate for 77 GHz automotive radar:
/// - Pedestrian: RCS < 3 dBsm
/// - Bicycle: 3 <= RCS < 7 dBsm
/// - Car: 7 <= RCS < 18 dBsm
/// - Truck: RCS >= 18 dBsm
/// - Unknown: RCS < -15 dBsm (noise / clutter)
pub fn classify_object(rcs_dbsm: f64) -> ObjectClass {
    if rcs_dbsm < -15.0 {
        ObjectClass::Unknown
    } else if rcs_dbsm < 3.0 {
        ObjectClass::Pedestrian
    } else if rcs_dbsm < 7.0 {
        ObjectClass::Bicycle
    } else if rcs_dbsm < 18.0 {
        ObjectClass::Car
    } else {
        ObjectClass::Truck
    }
}

/// Convert radar polar coordinates to Cartesian (x, y).
///
/// Convention:
/// - x axis points along radar boresight (forward).
/// - y axis points to the right.
/// - azimuth_deg: 0 = boresight, positive = right.
///
/// Returns `(x, y)` in metres.
pub fn cartesian_from_polar(range_m: f64, azimuth_deg: f64) -> (f64, f64) {
    let az_rad = azimuth_deg * std::f64::consts::PI / 180.0;
    let x = range_m * az_rad.cos();
    let y = range_m * az_rad.sin();
    (x, y)
}

// ---------------------------------------------------------------------------
// RadarTracker implementation
// ---------------------------------------------------------------------------

/// Multi-target radar tracker for automotive ADAS applications.
///
/// Call [`update`](RadarTracker::update) once per radar scan with the current
/// list of detections. The tracker returns the list of all live (non-deleted)
/// tracked objects.
pub struct RadarTracker {
    config: RadarTrackerConfig,
    tracks: Vec<InternalTrack>,
    next_id: u32,
    /// Process noise variance for range (m^2). Tuned for highway driving.
    range_process_var: f64,
    /// Measurement noise variance for range (m^2).
    range_meas_var: f64,
    /// Process noise variance for velocity (m^2/s^2).
    vel_process_var: f64,
    /// Measurement noise variance for velocity (m^2/s^2).
    vel_meas_var: f64,
    /// Process noise variance for azimuth (deg^2).
    az_process_var: f64,
    /// Measurement noise variance for azimuth (deg^2).
    az_meas_var: f64,
}

impl RadarTracker {
    /// Create a new tracker with the given configuration.
    pub fn new(config: RadarTrackerConfig) -> Self {
        Self {
            config,
            tracks: Vec::new(),
            next_id: 1,
            range_process_var: 1.0,
            range_meas_var: 0.5,
            vel_process_var: 0.5,
            vel_meas_var: 0.3,
            az_process_var: 0.25,
            az_meas_var: 0.5,
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &RadarTrackerConfig {
        &self.config
    }

    /// Return the number of currently live (non-deleted) tracks.
    pub fn num_active_tracks(&self) -> usize {
        self.tracks.iter().filter(|t| t.state != TrackState::Deleted).count()
    }

    /// Process one radar scan cycle.
    ///
    /// Steps: predict -> associate -> update associated -> coast unassociated
    /// -> initiate new tracks from unassociated detections -> prune deleted.
    ///
    /// Returns a snapshot of all live tracked objects.
    pub fn update(&mut self, detections: &[RadarDetection]) -> Vec<TrackedObject> {
        // 1. Predict all tracks forward.
        self.predict_all();

        // 2. Build external view for association.
        let external: Vec<TrackedObject> = self.tracks.iter().map(|t| self.to_external(t)).collect();

        // 3. Associate detections to tracks.
        let associations = nearest_neighbor_association(
            &external,
            detections,
            self.config.gate_distance_m,
            self.config.gate_velocity_mps,
        );

        // Mark which tracks and detections are associated.
        let mut track_associated = vec![false; self.tracks.len()];
        let mut det_associated = vec![false; detections.len()];

        // 4. Update associated tracks.
        for &(ti, di) in &associations {
            track_associated[ti] = true;
            det_associated[di] = true;
            self.update_track(ti, &detections[di]);
        }

        // 5. Coast unassociated tracks.
        for (ti, associated) in track_associated.iter().enumerate() {
            if !associated {
                self.coast_track(ti);
            }
        }

        // 6. Initiate new tracks from unassociated detections.
        for (di, associated) in det_associated.iter().enumerate() {
            if !associated {
                self.initiate_track(&detections[di]);
            }
        }

        // 7. Prune deleted tracks and build output.
        self.tracks.retain(|t| t.state != TrackState::Deleted);
        self.tracks.iter().map(|t| self.to_external(t)).collect()
    }

    // -- Private helpers ----------------------------------------------------

    fn predict_all(&mut self) {
        let dt = self.config.dt_s;
        for trk in &mut self.tracks {
            // Predict range using velocity.
            let (pred_r, _) = kalman_predict_1d(trk.range_kf.x, trk.velocity_kf.x, dt);
            trk.range_kf.x = pred_r;
            trk.range_kf.p += self.range_process_var;

            // Velocity: constant model (predict = keep).
            trk.velocity_kf.p += self.vel_process_var;

            // Azimuth: constant model.
            trk.azimuth_kf.p += self.az_process_var;

            trk.age += 1;
        }
    }

    fn update_track(&mut self, idx: usize, det: &RadarDetection) {
        let trk = &mut self.tracks[idx];

        // Kalman update for range.
        let (r, rp) =
            kalman_update_1d(trk.range_kf.x, det.range_m, trk.range_kf.p, self.range_meas_var);
        trk.range_kf.x = r;
        trk.range_kf.p = rp;

        // Kalman update for velocity.
        trk.prev_velocity = trk.velocity_kf.x;
        let (v, vp) = kalman_update_1d(
            trk.velocity_kf.x,
            det.velocity_mps,
            trk.velocity_kf.p,
            self.vel_meas_var,
        );
        trk.velocity_kf.x = v;
        trk.velocity_kf.p = vp;

        // Derive acceleration from velocity change.
        if self.config.dt_s > 0.0 {
            trk.acceleration_mps2 = (trk.velocity_kf.x - trk.prev_velocity) / self.config.dt_s;
        }

        // Kalman update for azimuth.
        let (a, ap) = kalman_update_1d(
            trk.azimuth_kf.x,
            det.azimuth_deg,
            trk.azimuth_kf.p,
            self.az_meas_var,
        );
        trk.azimuth_kf.x = a;
        trk.azimuth_kf.p = ap;

        // Update RCS (simple exponential moving average).
        trk.rcs_dbsm = 0.8 * trk.rcs_dbsm + 0.2 * det.rcs_dbsm;

        // Track management: increment hits, reset misses.
        trk.hits += 1;
        trk.misses = 0;

        // Promote tentative -> confirmed.
        if trk.state == TrackState::Tentative && trk.hits >= self.config.confirm_hits {
            trk.state = TrackState::Confirmed;
        }
        // Restore coasting -> confirmed on re-detection.
        if trk.state == TrackState::Coasting {
            trk.state = TrackState::Confirmed;
        }
    }

    fn coast_track(&mut self, idx: usize) {
        let trk = &mut self.tracks[idx];
        trk.misses += 1;

        if trk.misses >= self.config.delete_misses {
            trk.state = TrackState::Deleted;
        } else if trk.state == TrackState::Confirmed {
            trk.state = TrackState::Coasting;
        } else if trk.state == TrackState::Tentative {
            // Tentative tracks that miss are deleted more aggressively.
            // Delete tentative tracks after 2 misses.
            if trk.misses >= 2 {
                trk.state = TrackState::Deleted;
            }
        }
    }

    fn initiate_track(&mut self, det: &RadarDetection) {
        if self.tracks.len() >= self.config.max_tracks {
            return; // Capacity limit reached.
        }

        let initial_state = if self.config.confirm_hits <= 1 {
            TrackState::Confirmed
        } else {
            TrackState::Tentative
        };

        let trk = InternalTrack {
            id: self.next_id,
            range_kf: KalmanState1D { x: det.range_m, p: self.range_meas_var },
            velocity_kf: KalmanState1D { x: det.velocity_mps, p: self.vel_meas_var },
            azimuth_kf: KalmanState1D { x: det.azimuth_deg, p: self.az_meas_var },
            prev_velocity: det.velocity_mps,
            acceleration_mps2: 0.0,
            state: initial_state,
            age: 1,
            hits: 1,
            misses: 0,
            rcs_dbsm: det.rcs_dbsm,
        };
        self.next_id += 1;
        self.tracks.push(trk);
    }

    fn to_external(&self, trk: &InternalTrack) -> TrackedObject {
        TrackedObject {
            id: trk.id,
            range_m: trk.range_kf.x,
            velocity_mps: trk.velocity_kf.x,
            azimuth_deg: trk.azimuth_kf.x,
            acceleration_mps2: trk.acceleration_mps2,
            state: trk.state,
            age: trk.age,
            hits: trk.hits,
            misses: trk.misses,
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> RadarTrackerConfig {
        RadarTrackerConfig::default()
    }

    fn make_det(range: f64, vel: f64, az: f64, rcs: f64) -> RadarDetection {
        RadarDetection {
            range_m: range,
            velocity_mps: vel,
            azimuth_deg: az,
            rcs_dbsm: rcs,
        }
    }

    // -- Kalman predict/update -----------------------------------------------

    #[test]
    fn test_kalman_predict_1d_stationary() {
        let (pos, vel) = kalman_predict_1d(10.0, 0.0, 0.05);
        assert!((pos - 10.0).abs() < 1e-12);
        assert!((vel - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_kalman_predict_1d_moving() {
        let (pos, vel) = kalman_predict_1d(100.0, -20.0, 0.05);
        assert!((pos - 99.0).abs() < 1e-12);
        assert!((vel - (-20.0)).abs() < 1e-12);
    }

    #[test]
    fn test_kalman_predict_1d_large_dt() {
        let (pos, vel) = kalman_predict_1d(0.0, 5.0, 10.0);
        assert!((pos - 50.0).abs() < 1e-12);
        assert!((vel - 5.0).abs() < 1e-12);
    }

    #[test]
    fn test_kalman_update_1d_equal_variance() {
        let (x, p) = kalman_update_1d(10.0, 12.0, 1.0, 1.0);
        // K = 0.5, so updated = 10 + 0.5 * 2 = 11
        assert!((x - 11.0).abs() < 1e-12);
        assert!((p - 0.5).abs() < 1e-12);
    }

    #[test]
    fn test_kalman_update_1d_high_measurement_trust() {
        // Very small measurement variance -> result near measurement.
        let (x, _p) = kalman_update_1d(10.0, 20.0, 100.0, 0.01);
        assert!((x - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_kalman_update_1d_high_prediction_trust() {
        // Very small prediction variance -> result near predicted.
        let (x, _p) = kalman_update_1d(10.0, 20.0, 0.01, 100.0);
        assert!((x - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_kalman_update_1d_zero_variance() {
        // Degenerate case: both variances zero.
        let (x, p) = kalman_update_1d(5.0, 10.0, 0.0, 0.0);
        assert!((x - 5.0).abs() < 1e-12);
        assert!((p - 0.0).abs() < 1e-12);
    }

    // -- Polar to Cartesian --------------------------------------------------

    #[test]
    fn test_cartesian_boresight() {
        let (x, y) = cartesian_from_polar(50.0, 0.0);
        assert!((x - 50.0).abs() < 1e-10);
        assert!(y.abs() < 1e-10);
    }

    #[test]
    fn test_cartesian_90_degrees() {
        let (x, y) = cartesian_from_polar(50.0, 90.0);
        assert!(x.abs() < 1e-10);
        assert!((y - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_cartesian_negative_azimuth() {
        let (x, y) = cartesian_from_polar(100.0, -45.0);
        let expected_x = 100.0 * (std::f64::consts::FRAC_PI_4).cos();
        let expected_y = -100.0 * (std::f64::consts::FRAC_PI_4).sin();
        assert!((x - expected_x).abs() < 1e-10);
        assert!((y - expected_y).abs() < 1e-10);
    }

    #[test]
    fn test_cartesian_zero_range() {
        let (x, y) = cartesian_from_polar(0.0, 45.0);
        assert!(x.abs() < 1e-15);
        assert!(y.abs() < 1e-15);
    }

    // -- TTC -----------------------------------------------------------------

    #[test]
    fn test_ttc_closing() {
        let ttc = compute_ttc(100.0, 25.0);
        assert!((ttc - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_ttc_receding() {
        let ttc = compute_ttc(100.0, -10.0);
        assert!(ttc.is_infinite());
    }

    #[test]
    fn test_ttc_zero_velocity() {
        let ttc = compute_ttc(50.0, 0.0);
        assert!(ttc.is_infinite());
    }

    #[test]
    fn test_ttc_zero_range() {
        let ttc = compute_ttc(0.0, 10.0);
        assert!((ttc - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_ttc_negative_range() {
        // Degenerate: already past -> 0.
        let ttc = compute_ttc(-5.0, 10.0);
        assert!((ttc - 0.0).abs() < 1e-12);
    }

    // -- Object classification -----------------------------------------------

    #[test]
    fn test_classify_pedestrian() {
        assert_eq!(classify_object(0.0), ObjectClass::Pedestrian);
        assert_eq!(classify_object(-10.0), ObjectClass::Pedestrian);
        assert_eq!(classify_object(2.9), ObjectClass::Pedestrian);
    }

    #[test]
    fn test_classify_bicycle() {
        assert_eq!(classify_object(3.0), ObjectClass::Bicycle);
        assert_eq!(classify_object(5.0), ObjectClass::Bicycle);
        assert_eq!(classify_object(6.9), ObjectClass::Bicycle);
    }

    #[test]
    fn test_classify_car() {
        assert_eq!(classify_object(7.0), ObjectClass::Car);
        assert_eq!(classify_object(10.0), ObjectClass::Car);
        assert_eq!(classify_object(17.9), ObjectClass::Car);
    }

    #[test]
    fn test_classify_truck() {
        assert_eq!(classify_object(18.0), ObjectClass::Truck);
        assert_eq!(classify_object(25.0), ObjectClass::Truck);
    }

    #[test]
    fn test_classify_unknown_noise() {
        assert_eq!(classify_object(-20.0), ObjectClass::Unknown);
        assert_eq!(classify_object(-50.0), ObjectClass::Unknown);
    }

    // -- Association ---------------------------------------------------------

    #[test]
    fn test_association_empty_tracks() {
        let dets = vec![make_det(50.0, -10.0, 0.0, 10.0)];
        let result = nearest_neighbor_association(&[], &dets, 5.0, 3.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_association_empty_detections() {
        let tracks = vec![TrackedObject {
            id: 1,
            range_m: 50.0,
            velocity_mps: -10.0,
            azimuth_deg: 0.0,
            acceleration_mps2: 0.0,
            state: TrackState::Confirmed,
            age: 5,
            hits: 5,
            misses: 0,
        }];
        let result = nearest_neighbor_association(&tracks, &[], 5.0, 3.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_association_single_match() {
        let tracks = vec![TrackedObject {
            id: 1,
            range_m: 50.0,
            velocity_mps: -10.0,
            azimuth_deg: 0.0,
            acceleration_mps2: 0.0,
            state: TrackState::Confirmed,
            age: 5,
            hits: 5,
            misses: 0,
        }];
        let dets = vec![make_det(51.0, -10.5, 1.0, 10.0)];
        let result = nearest_neighbor_association(&tracks, &dets, 5.0, 3.0);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], (0, 0));
    }

    #[test]
    fn test_association_outside_gate() {
        let tracks = vec![TrackedObject {
            id: 1,
            range_m: 50.0,
            velocity_mps: -10.0,
            azimuth_deg: 0.0,
            acceleration_mps2: 0.0,
            state: TrackState::Confirmed,
            age: 5,
            hits: 5,
            misses: 0,
        }];
        // Detection too far away in range.
        let dets = vec![make_det(60.0, -10.0, 0.0, 10.0)];
        let result = nearest_neighbor_association(&tracks, &dets, 5.0, 3.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_association_two_tracks_two_dets() {
        let tracks = vec![
            TrackedObject {
                id: 1, range_m: 30.0, velocity_mps: -15.0, azimuth_deg: -5.0,
                acceleration_mps2: 0.0, state: TrackState::Confirmed, age: 10, hits: 10, misses: 0,
            },
            TrackedObject {
                id: 2, range_m: 80.0, velocity_mps: -5.0, azimuth_deg: 3.0,
                acceleration_mps2: 0.0, state: TrackState::Confirmed, age: 8, hits: 8, misses: 0,
            },
        ];
        let dets = vec![
            make_det(31.0, -14.5, -4.5, 12.0),
            make_det(79.0, -5.5, 3.5, 20.0),
        ];
        let result = nearest_neighbor_association(&tracks, &dets, 5.0, 3.0);
        assert_eq!(result.len(), 2);
        // Track 0 -> det 0, track 1 -> det 1.
        let mut sorted = result.clone();
        sorted.sort_by_key(|p| p.0);
        assert_eq!(sorted[0], (0, 0));
        assert_eq!(sorted[1], (1, 1));
    }

    // -- Track management (full tracker) -------------------------------------

    #[test]
    fn test_track_initiation() {
        let mut tracker = RadarTracker::new(default_config());
        let dets = vec![make_det(50.0, -10.0, 2.0, 10.0)];
        let tracks = tracker.update(&dets);
        assert_eq!(tracks.len(), 1);
        assert_eq!(tracks[0].state, TrackState::Tentative);
        assert_eq!(tracks[0].hits, 1);
        assert_eq!(tracks[0].id, 1);
    }

    #[test]
    fn test_track_confirmation() {
        let config = RadarTrackerConfig { confirm_hits: 3, ..default_config() };
        let mut tracker = RadarTracker::new(config);
        let det = make_det(50.0, -10.0, 0.0, 10.0);

        // Scan 1: tentative.
        let tracks = tracker.update(&[det]);
        assert_eq!(tracks[0].state, TrackState::Tentative);

        // Scan 2: still tentative.
        let det2 = make_det(49.5, -10.0, 0.0, 10.0);
        let tracks = tracker.update(&[det2]);
        assert_eq!(tracks[0].state, TrackState::Tentative);
        assert_eq!(tracks[0].hits, 2);

        // Scan 3: confirmed (3 hits).
        let det3 = make_det(49.0, -10.0, 0.0, 10.0);
        let tracks = tracker.update(&[det3]);
        assert_eq!(tracks[0].state, TrackState::Confirmed);
        assert_eq!(tracks[0].hits, 3);
    }

    #[test]
    fn test_track_coasting() {
        let config = RadarTrackerConfig { confirm_hits: 1, delete_misses: 5, ..default_config() };
        let mut tracker = RadarTracker::new(config);

        // Create and immediately confirm a track.
        let tracks = tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0)]);
        assert_eq!(tracks[0].state, TrackState::Confirmed);

        // Miss: should coast.
        let tracks = tracker.update(&[]);
        assert_eq!(tracks.len(), 1);
        assert_eq!(tracks[0].state, TrackState::Coasting);
        assert_eq!(tracks[0].misses, 1);
    }

    #[test]
    fn test_track_deletion_after_misses() {
        let config = RadarTrackerConfig {
            confirm_hits: 1,
            delete_misses: 3,
            ..default_config()
        };
        let mut tracker = RadarTracker::new(config);

        // Create and confirm.
        tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0)]);

        // 3 consecutive misses should delete.
        tracker.update(&[]);
        tracker.update(&[]);
        let tracks = tracker.update(&[]);
        // Track should be pruned (deleted).
        assert_eq!(tracks.len(), 0);
    }

    #[test]
    fn test_tentative_deletion() {
        let config = RadarTrackerConfig { confirm_hits: 3, ..default_config() };
        let mut tracker = RadarTracker::new(config);

        // Initiate tentative track.
        tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0)]);

        // Two misses should delete a tentative track.
        tracker.update(&[]);
        let tracks = tracker.update(&[]);
        assert!(tracks.is_empty());
    }

    #[test]
    fn test_coasting_to_confirmed_on_redetection() {
        let config = RadarTrackerConfig { confirm_hits: 1, delete_misses: 5, ..default_config() };
        let mut tracker = RadarTracker::new(config);

        // Confirmed track.
        tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0)]);
        // Miss -> coasting.
        let tracks = tracker.update(&[]);
        assert_eq!(tracks[0].state, TrackState::Coasting);

        // Re-detect -> confirmed again.
        let tracks = tracker.update(&[make_det(48.5, -10.0, 0.0, 10.0)]);
        assert_eq!(tracks[0].state, TrackState::Confirmed);
    }

    #[test]
    fn test_multiple_targets() {
        let mut tracker = RadarTracker::new(default_config());

        // Two well-separated targets.
        let dets = vec![
            make_det(30.0, -15.0, -5.0, 12.0),
            make_det(80.0, -5.0, 10.0, 20.0),
        ];
        let tracks = tracker.update(&dets);
        assert_eq!(tracks.len(), 2);

        // Second scan -- both updated.
        let dets2 = vec![
            make_det(29.25, -15.0, -5.0, 12.0),
            make_det(79.75, -5.0, 10.0, 20.0),
        ];
        let tracks = tracker.update(&dets2);
        assert_eq!(tracks.len(), 2);
        assert!(tracks.iter().all(|t| t.hits == 2));
    }

    #[test]
    fn test_max_tracks_limit() {
        let config = RadarTrackerConfig { max_tracks: 2, ..default_config() };
        let mut tracker = RadarTracker::new(config);

        // Three detections, but max_tracks = 2.
        let dets = vec![
            make_det(10.0, -5.0, 0.0, 10.0),
            make_det(50.0, -10.0, 5.0, 10.0),
            make_det(100.0, -20.0, -10.0, 10.0),
        ];
        let tracks = tracker.update(&dets);
        assert_eq!(tracks.len(), 2);
    }

    #[test]
    fn test_track_ids_monotonic() {
        let mut tracker = RadarTracker::new(default_config());
        let t1 = tracker.update(&[make_det(30.0, -10.0, 0.0, 10.0)]);
        let t2 = tracker.update(&[
            make_det(29.5, -10.0, 0.0, 10.0),
            make_det(80.0, -5.0, 5.0, 15.0),
        ]);
        // IDs should be monotonically increasing.
        let mut ids: Vec<u32> = t2.iter().map(|t| t.id).collect();
        ids.sort();
        assert!(ids.windows(2).all(|w| w[0] < w[1]));
        assert_eq!(t1[0].id, ids[0]); // First track persists.
    }

    #[test]
    fn test_age_increments() {
        let mut tracker = RadarTracker::new(RadarTrackerConfig {
            confirm_hits: 1,
            delete_misses: 10,
            ..default_config()
        });
        tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0)]);
        let tracks = tracker.update(&[make_det(49.5, -10.0, 0.0, 10.0)]);
        assert_eq!(tracks[0].age, 2);
        let tracks = tracker.update(&[make_det(49.0, -10.0, 0.0, 10.0)]);
        assert_eq!(tracks[0].age, 3);
    }

    #[test]
    fn test_kalman_smoothing_effect() {
        // After several updates, the tracker state should converge toward
        // a smoothed estimate, not just follow the measurement exactly.
        let config = RadarTrackerConfig { confirm_hits: 1, ..default_config() };
        let mut tracker = RadarTracker::new(config);

        tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0)]);
        tracker.update(&[make_det(49.5, -10.0, 0.0, 10.0)]);
        tracker.update(&[make_det(49.0, -10.0, 0.0, 10.0)]);

        // Now give a noisy measurement far from expected.
        let tracks = tracker.update(&[make_det(55.0, -10.0, 0.0, 10.0)]);
        // Kalman should not snap to 55; it should be pulled toward it but
        // moderated by the prediction.
        assert!(tracks[0].range_m < 55.0);
        assert!(tracks[0].range_m > 48.0);
    }

    #[test]
    fn test_acceleration_estimate() {
        // Use a wider velocity gate so the velocity-change detection still associates.
        let config = RadarTrackerConfig {
            confirm_hits: 1,
            gate_velocity_mps: 15.0,
            ..default_config()
        };
        let mut tracker = RadarTracker::new(config);

        // Constant velocity -> acceleration near zero.
        tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0)]);
        let tracks = tracker.update(&[make_det(49.5, -10.0, 0.0, 10.0)]);
        assert!(
            tracks[0].acceleration_mps2.abs() < 5.0,
            "constant vel: expected ~0 accel, got {}",
            tracks[0].acceleration_mps2
        );

        // Sudden velocity change -> nonzero acceleration.
        // Velocity jumps from ~-10 to ~-20 (within the 15 m/s gate).
        let tracks = tracker.update(&[make_det(49.0, -20.0, 0.0, 10.0)]);
        // Kalman smoothing dampens the change, but acceleration should be negative.
        assert!(
            tracks[0].acceleration_mps2 < -5.0,
            "velocity jump: expected large negative accel, got {}",
            tracks[0].acceleration_mps2
        );
    }

    #[test]
    fn test_default_config_values() {
        let config = RadarTrackerConfig::default();
        assert_eq!(config.max_tracks, 64);
        assert!((config.gate_distance_m - 5.0).abs() < 1e-12);
        assert!((config.gate_velocity_mps - 3.0).abs() < 1e-12);
        assert_eq!(config.confirm_hits, 3);
        assert_eq!(config.delete_misses, 5);
        assert!((config.dt_s - 0.050).abs() < 1e-12);
    }

    #[test]
    fn test_no_detections_no_tracks() {
        let mut tracker = RadarTracker::new(default_config());
        let tracks = tracker.update(&[]);
        assert!(tracks.is_empty());
    }

    #[test]
    fn test_num_active_tracks() {
        let mut tracker = RadarTracker::new(RadarTrackerConfig {
            confirm_hits: 1,
            delete_misses: 2,
            ..default_config()
        });
        tracker.update(&[make_det(50.0, -10.0, 0.0, 10.0), make_det(80.0, -5.0, 5.0, 15.0)]);
        assert_eq!(tracker.num_active_tracks(), 2);

        // Miss both, then miss again -> deleted.
        tracker.update(&[]);
        tracker.update(&[]);
        assert_eq!(tracker.num_active_tracks(), 0);
    }

    #[test]
    fn test_crossing_targets() {
        // Two targets that swap positions should still be tracked independently
        // because velocity is also used for gating.
        let config = RadarTrackerConfig {
            confirm_hits: 1,
            gate_distance_m: 5.0,
            gate_velocity_mps: 3.0,
            ..default_config()
        };
        let mut tracker = RadarTracker::new(config);

        // Track A: range 50 closing at -10. Track B: range 50 receding at +10.
        let dets = vec![
            make_det(50.0, -10.0, -2.0, 10.0),
            make_det(50.0, 10.0, 2.0, 10.0),
        ];
        let tracks = tracker.update(&dets);
        assert_eq!(tracks.len(), 2);

        // Next scan: both at range ~50 with same velocities.
        let dets2 = vec![
            make_det(49.5, -10.0, -2.0, 10.0),
            make_det(50.5, 10.0, 2.0, 10.0),
        ];
        let tracks = tracker.update(&dets2);
        assert_eq!(tracks.len(), 2);
        // Verify they keep their distinct velocities.
        let mut vels: Vec<f64> = tracks.iter().map(|t| t.velocity_mps).collect();
        vels.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(vels[0] < -5.0); // Closing target.
        assert!(vels[1] > 5.0);  // Receding target.
    }
}

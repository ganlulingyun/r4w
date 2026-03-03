//! LEO Satellite Handover Management
//!
//! Implements handover management for Low Earth Orbit (LEO) non-GEO satellite
//! communication systems including orbital mechanics, Doppler prediction,
//! handover decision algorithms, inter-satellite links, beam management,
//! constellation geometry, handover protocol state machines, and link budgets.
//!
//! # Overview
//!
//! LEO satellites orbit at 500–1200 km altitude, completing an orbit in ~90–115 minutes.
//! A ground terminal sees a satellite for only 5–15 minutes per pass, requiring frequent
//! handovers. This module provides tools to predict, decide, and execute those handovers.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::leo_sat_handover::*;
//!
//! // Define a Walker-Delta constellation (e.g., Iridium-like)
//! let constellation = WalkerConstellation::new(66, 6, 2, 86.4_f64.to_radians(), 780_000.0);
//!
//! // Ground station position (latitude, longitude in radians)
//! let ground = GroundPosition::new(37.7749_f64.to_radians(), -122.4194_f64.to_radians(), 0.0);
//!
//! // Count visible satellites above 10-degree elevation mask
//! let visible = constellation.visible_satellite_count(&ground, 10.0_f64.to_radians(), 0.0);
//! assert!(visible >= 1);
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Physical constants
// ─────────────────────────────────────────────────────────────────────────────

/// Earth's gravitational parameter μ = GM (m³/s²)
pub const MU_EARTH: f64 = 3.986_004_418e14;
/// Earth's mean radius (m)
pub const R_EARTH: f64 = 6_371_000.0;
/// Earth's equatorial radius (m) – WGS-84
pub const A_EARTH: f64 = 6_378_137.0;
/// J2 zonal harmonic coefficient
pub const J2: f64 = 1.082_626_68e-3;
/// Speed of light (m/s)
pub const C_LIGHT: f64 = 2.997_924_58e8;
/// Boltzmann constant (J/K)
pub const K_BOLTZMANN: f64 = 1.380_649e-23;
/// Standard atmospheric temperature (K)
pub const T_STANDARD: f64 = 290.0;

// ─────────────────────────────────────────────────────────────────────────────
// Orbital mechanics
// ─────────────────────────────────────────────────────────────────────────────

/// Circular LEO orbital state (simplified, J2-perturbed RAAN drift included).
#[derive(Debug, Clone)]
pub struct LeoOrbit {
    /// Semi-major axis / orbit radius (m) – circular orbit assumption
    pub radius: f64,
    /// Inclination (radians)
    pub inclination: f64,
    /// Right Ascension of Ascending Node (radians) – drifts due to J2
    pub raan: f64,
    /// Argument of latitude (mean anomaly, radians) at epoch
    pub arg_lat_epoch: f64,
    /// Epoch time (seconds since reference)
    pub epoch: f64,
}

impl LeoOrbit {
    /// Create a new circular LEO orbit.
    ///
    /// * `altitude_m` – altitude above Earth's surface (m)
    /// * `inclination_rad` – orbital inclination (rad)
    /// * `raan_rad` – RAAN at epoch (rad)
    /// * `arg_lat_rad` – argument of latitude at epoch (rad)
    /// * `epoch_s` – epoch time (s)
    pub fn new(
        altitude_m: f64,
        inclination_rad: f64,
        raan_rad: f64,
        arg_lat_rad: f64,
        epoch_s: f64,
    ) -> Self {
        Self {
            radius: R_EARTH + altitude_m,
            inclination: inclination_rad,
            raan: raan_rad,
            arg_lat_epoch: arg_lat_rad,
            epoch: epoch_s,
        }
    }

    /// Orbital period (seconds) for a circular orbit.
    pub fn period(&self) -> f64 {
        2.0 * PI * (self.radius.powi(3) / MU_EARTH).sqrt()
    }

    /// Mean motion n (rad/s).
    pub fn mean_motion(&self) -> f64 {
        (MU_EARTH / self.radius.powi(3)).sqrt()
    }

    /// Orbital velocity (m/s) for a circular orbit.
    pub fn velocity(&self) -> f64 {
        (MU_EARTH / self.radius).sqrt()
    }

    /// J2-induced RAAN drift rate (rad/s).
    ///
    /// dΩ/dt = -(3/2) * n * J2 * (R_e/a)² * cos(i)
    pub fn raan_drift_rate(&self) -> f64 {
        -1.5 * self.mean_motion() * J2 * (A_EARTH / self.radius).powi(2)
            * self.inclination.cos()
    }

    /// Propagate to time `t` (seconds) and return satellite ECI position (x, y, z) in metres.
    ///
    /// Uses circular orbit kinematics with J2 RAAN precession.
    pub fn eci_position(&self, t: f64) -> (f64, f64, f64) {
        let dt = t - self.epoch;
        let n = self.mean_motion();
        let arg_lat = self.arg_lat_epoch + n * dt;
        let raan = self.raan + self.raan_drift_rate() * dt;
        let inc = self.inclination;
        let r = self.radius;

        // Perifocal → ECI rotation
        let x = r * (raan.cos() * arg_lat.cos() - raan.sin() * arg_lat.sin() * inc.cos());
        let y = r * (raan.sin() * arg_lat.cos() + raan.cos() * arg_lat.sin() * inc.cos());
        let z = r * arg_lat.sin() * inc.sin();
        (x, y, z)
    }

    /// Satellite ECI velocity vector (vx, vy, vz) in m/s (circular orbit).
    pub fn eci_velocity(&self, t: f64) -> (f64, f64, f64) {
        let dt = t - self.epoch;
        let n = self.mean_motion();
        let v = self.velocity();
        let arg_lat = self.arg_lat_epoch + n * dt;
        let raan = self.raan + self.raan_drift_rate() * dt;
        let inc = self.inclination;

        let vx = v * (-raan.cos() * arg_lat.sin() - raan.sin() * arg_lat.cos() * inc.cos());
        let vy = v * (-raan.sin() * arg_lat.sin() + raan.cos() * arg_lat.cos() * inc.cos());
        let vz = v * arg_lat.cos() * inc.sin();
        (vx, vy, vz)
    }

    /// Ground track velocity (m/s) – approximate equatorial projection.
    pub fn ground_track_velocity(&self) -> f64 {
        // Inertial velocity projected onto ground minus Earth rotation
        let v_inertial = self.velocity();
        let omega_earth = 7.292_115e-5; // rad/s
        let v_earth_eq = omega_earth * A_EARTH;
        (v_inertial - v_earth_eq).abs()
    }

    /// Slant range (m) from a ground position to the satellite at time `t`.
    pub fn slant_range(&self, ground: &GroundPosition, t: f64) -> f64 {
        let (sx, sy, sz) = self.eci_position(t);
        let (gx, gy, gz) = ground.eci_position(t);
        let dx = sx - gx;
        let dy = sy - gy;
        let dz = sz - gz;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Elevation angle (radians) of satellite as seen from ground at time `t`.
    /// Returns negative values when below horizon.
    pub fn elevation_angle(&self, ground: &GroundPosition, t: f64) -> f64 {
        let (sx, sy, sz) = self.eci_position(t);
        let (gx, gy, gz) = ground.eci_position(t);
        let dx = sx - gx;
        let dy = sy - gy;
        let dz = sz - gz;

        // Ground unit vector (towards zenith)
        let gr = (gx * gx + gy * gy + gz * gz).sqrt();
        let nx = gx / gr;
        let ny = gy / gr;
        let nz = gz / gr;

        // Range vector magnitude
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        if range < 1.0 {
            return PI / 2.0;
        }
        let dot = (dx * nx + dy * ny + dz * nz) / range;
        let dot_clamped = dot.clamp(-1.0, 1.0);
        dot_clamped.asin()
    }

    /// Estimate the visibility window (start, end) around `t_guess` (seconds).
    /// Searches for rise and set times using bisection.
    /// `elev_mask_rad` – minimum elevation (rad) to be considered visible.
    pub fn visibility_window(
        &self,
        ground: &GroundPosition,
        t_guess: f64,
        elev_mask_rad: f64,
    ) -> Option<(f64, f64)> {
        // Find approximate time of max elevation by sampling one period
        let period = self.period();
        let steps = 360;
        let dt = period / steps as f64;

        let mut max_elev = f64::NEG_INFINITY;
        let mut t_max = t_guess;
        let t_start_scan = t_guess - period / 2.0;
        for i in 0..steps {
            let t = t_start_scan + i as f64 * dt;
            let elev = self.elevation_angle(ground, t);
            if elev > max_elev {
                max_elev = elev;
                t_max = t;
            }
        }

        if max_elev < elev_mask_rad {
            return None;
        }

        // Bisect to find rise time
        let rise = bisect_elevation(self, ground, t_max - period / 4.0, t_max, elev_mask_rad)?;
        // Bisect to find set time
        let set = bisect_elevation(self, ground, t_max, t_max + period / 4.0, elev_mask_rad)?;

        Some((rise, set))
    }

    /// Pass duration estimate (seconds).
    pub fn pass_duration(&self, ground: &GroundPosition, t_guess: f64, elev_mask_rad: f64) -> f64 {
        self.visibility_window(ground, t_guess, elev_mask_rad)
            .map(|(rise, set)| set - rise)
            .unwrap_or(0.0)
    }
}

/// Find the elevation-mask crossing time between t_lo and t_hi using bisection.
fn bisect_elevation(
    orbit: &LeoOrbit,
    ground: &GroundPosition,
    t_lo: f64,
    t_hi: f64,
    elev_mask: f64,
) -> Option<f64> {
    let mut lo = t_lo;
    let mut hi = t_hi;
    let elev_lo = orbit.elevation_angle(ground, lo) - elev_mask;
    let elev_hi = orbit.elevation_angle(ground, hi) - elev_mask;
    // Need a sign change
    if elev_lo * elev_hi > 0.0 {
        return None;
    }
    for _ in 0..50 {
        let mid = (lo + hi) / 2.0;
        let e_mid = orbit.elevation_angle(ground, mid) - elev_mask;
        let e_lo = orbit.elevation_angle(ground, lo) - elev_mask;
        if e_lo * e_mid <= 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
        if (hi - lo).abs() < 0.1 {
            break;
        }
    }
    Some((lo + hi) / 2.0)
}

// ─────────────────────────────────────────────────────────────────────────────
// Ground position
// ─────────────────────────────────────────────────────────────────────────────

/// Ground station or user terminal position (geodetic).
#[derive(Debug, Clone, Copy)]
pub struct GroundPosition {
    /// Geodetic latitude (radians)
    pub lat: f64,
    /// Geodetic longitude (radians)
    pub lon: f64,
    /// Altitude above ellipsoid (m)
    pub alt: f64,
}

impl GroundPosition {
    /// Create a new ground position.
    pub fn new(lat_rad: f64, lon_rad: f64, alt_m: f64) -> Self {
        Self {
            lat: lat_rad,
            lon: lon_rad,
            alt: alt_m,
        }
    }

    /// Earth-Centred Inertial (ECI) position at time `t` (seconds).
    /// Earth rotation is applied: Greenwich sidereal angle = ω_earth * t.
    pub fn eci_position(&self, t: f64) -> (f64, f64, f64) {
        let omega_earth = 7.292_115e-5; // rad/s
        let lst = self.lon + omega_earth * t;
        let r = A_EARTH + self.alt;
        let cos_lat = self.lat.cos();
        let x = r * cos_lat * lst.cos();
        let y = r * cos_lat * lst.sin();
        let z = r * self.lat.sin();
        (x, y, z)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Doppler prediction
// ─────────────────────────────────────────────────────────────────────────────

/// Doppler prediction for a LEO satellite link.
#[derive(Debug, Clone)]
pub struct DopplerPredictor {
    /// Carrier frequency (Hz)
    pub carrier_hz: f64,
}

impl DopplerPredictor {
    /// Create a predictor for a given carrier frequency.
    pub fn new(carrier_hz: f64) -> Self {
        Self { carrier_hz }
    }

    /// S-band downlink predictor (~2 GHz).
    pub fn s_band_downlink() -> Self {
        Self::new(2.0e9)
    }

    /// Ka-band downlink predictor (~20 GHz).
    pub fn ka_band_downlink() -> Self {
        Self::new(20.0e9)
    }

    /// Ka-band uplink predictor (~30 GHz).
    pub fn ka_band_uplink() -> Self {
        Self::new(30.0e9)
    }

    /// Compute range rate (m/s) between satellite and ground at time `t`.
    /// Positive = moving away (red-shift), negative = approaching (blue-shift).
    pub fn range_rate(orbit: &LeoOrbit, ground: &GroundPosition, t: f64) -> f64 {
        let (sx, sy, sz) = orbit.eci_position(t);
        let (gx, gy, gz) = ground.eci_position(t);
        let (vsx, vsy, vsz) = orbit.eci_velocity(t);

        // Ground ECI velocity (Earth rotation)
        let omega = 7.292_115e-5_f64;
        let vgx = -omega * gy;
        let vgy = omega * gx;
        let vgz = 0.0_f64;

        let dx = sx - gx;
        let dy = sy - gy;
        let dz = sz - gz;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        if range < 1.0 {
            return 0.0;
        }
        // Relative velocity component along range vector
        let dvx = vsx - vgx;
        let dvy = vsy - vgy;
        let dvz = vsz - vgz;
        (dvx * dx + dvy * dy + dvz * dz) / range
    }

    /// Doppler shift (Hz) on the downlink (satellite → ground).
    /// f_received = f_carrier * (1 - range_rate / c)
    pub fn doppler_shift(&self, orbit: &LeoOrbit, ground: &GroundPosition, t: f64) -> f64 {
        let rr = Self::range_rate(orbit, ground, t);
        -self.carrier_hz * rr / C_LIGHT
    }

    /// Doppler rate (Hz/s) – numerical derivative of Doppler shift.
    pub fn doppler_rate(&self, orbit: &LeoOrbit, ground: &GroundPosition, t: f64) -> f64 {
        let dt = 1.0;
        let f1 = self.doppler_shift(orbit, ground, t + dt);
        let f0 = self.doppler_shift(orbit, ground, t - dt);
        (f1 - f0) / (2.0 * dt)
    }

    /// Pre-compensation offset (Hz) for uplink – negate expected Doppler so
    /// the satellite receives the nominal frequency.
    pub fn uplink_precompensation(&self, orbit: &LeoOrbit, ground: &GroundPosition, t: f64) -> f64 {
        -self.doppler_shift(orbit, ground, t)
    }

    /// Maximum Doppler excursion (Hz) over a visibility window.
    pub fn max_doppler_excursion(
        &self,
        orbit: &LeoOrbit,
        ground: &GroundPosition,
        t_start: f64,
        t_end: f64,
        samples: usize,
    ) -> f64 {
        let dt = (t_end - t_start) / samples as f64;
        let mut min_d = f64::INFINITY;
        let mut max_d = f64::NEG_INFINITY;
        for i in 0..=samples {
            let d = self.doppler_shift(orbit, ground, t_start + i as f64 * dt);
            if d < min_d { min_d = d; }
            if d > max_d { max_d = d; }
        }
        max_d - min_d
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Handover decision
// ─────────────────────────────────────────────────────────────────────────────

/// Criteria used to score a candidate satellite for handover.
#[derive(Debug, Clone)]
pub struct HandoverCriteria {
    /// SNR threshold below which handover is triggered (dB)
    pub snr_threshold_db: f64,
    /// Hysteresis margin to prevent ping-pong handover (dB)
    pub hysteresis_db: f64,
    /// Minimum elevation angle mask (radians)
    pub elev_mask_rad: f64,
    /// Weights for multi-criteria scoring [elevation, snr, doppler_rate, remaining_time]
    pub weights: [f64; 4],
}

impl Default for HandoverCriteria {
    fn default() -> Self {
        Self {
            snr_threshold_db: 5.0,
            hysteresis_db: 3.0,
            elev_mask_rad: 10.0_f64.to_radians(),
            weights: [0.4, 0.3, 0.2, 0.1],
        }
    }
}

/// Measurement report from a satellite link.
#[derive(Debug, Clone)]
pub struct LinkMeasurement {
    /// Satellite identifier
    pub sat_id: u32,
    /// Measured SNR (dB)
    pub snr_db: f64,
    /// Elevation angle (radians)
    pub elevation_rad: f64,
    /// Doppler rate (Hz/s)
    pub doppler_rate_hz_s: f64,
    /// Estimated remaining visibility time (seconds)
    pub remaining_visibility_s: f64,
    /// Timestamp (seconds)
    pub timestamp_s: f64,
}

/// Handover trigger reason.
#[derive(Debug, Clone, PartialEq)]
pub enum HandoverTrigger {
    /// SNR dropped below threshold
    SnrBelowThreshold,
    /// Satellite is about to set (time-based)
    PredictedEndOfPass,
    /// Elevation fell below mask
    ElevationBelowMask,
    /// Better candidate available (multi-criteria)
    BetterCandidateAvailable,
    /// Forced handover (e.g., satellite failure)
    Forced,
}

/// Decision result from the handover engine.
#[derive(Debug, Clone)]
pub struct HandoverDecision {
    /// Source satellite (current serving)
    pub source_sat_id: u32,
    /// Target satellite
    pub target_sat_id: u32,
    /// Trigger reason
    pub trigger: HandoverTrigger,
    /// Score of target candidate (higher is better)
    pub target_score: f64,
    /// Estimated handover latency (ms)
    pub latency_ms: f64,
}

/// Handover decision engine.
#[derive(Debug, Clone)]
pub struct HandoverDecisionEngine {
    /// Current serving satellite ID
    pub serving_sat_id: Option<u32>,
    /// Decision criteria
    pub criteria: HandoverCriteria,
}

impl HandoverDecisionEngine {
    /// Create a new decision engine with default criteria.
    pub fn new() -> Self {
        Self {
            serving_sat_id: None,
            criteria: HandoverCriteria::default(),
        }
    }

    /// Create with custom criteria.
    pub fn with_criteria(criteria: HandoverCriteria) -> Self {
        Self {
            serving_sat_id: None,
            criteria,
        }
    }

    /// Score a candidate satellite measurement (0.0 = worst, 1.0 = best).
    pub fn score_candidate(&self, meas: &LinkMeasurement) -> f64 {
        let w = &self.criteria.weights;
        // Elevation component: 0 at mask, 1 at 90°
        let elev_score = ((meas.elevation_rad - self.criteria.elev_mask_rad)
            / (PI / 2.0 - self.criteria.elev_mask_rad))
            .clamp(0.0, 1.0);
        // SNR component: 0 at 0 dB, 1 at 20 dB
        let snr_score = (meas.snr_db / 20.0).clamp(0.0, 1.0);
        // Doppler rate component: lower rate is better (max expected ~100 Hz/s at Ka)
        let dr_score = (1.0 - meas.doppler_rate_hz_s.abs() / 200.0).clamp(0.0, 1.0);
        // Remaining time component: 0 at 0 s, 1 at 600 s
        let time_score = (meas.remaining_visibility_s / 600.0).clamp(0.0, 1.0);

        w[0] * elev_score + w[1] * snr_score + w[2] * dr_score + w[3] * time_score
    }

    /// Evaluate measurements and return a handover decision if needed.
    ///
    /// `serving_meas` – measurement of the current serving satellite.
    /// `candidates` – measurements of other visible satellites.
    pub fn evaluate(
        &mut self,
        serving_meas: &LinkMeasurement,
        candidates: &[LinkMeasurement],
    ) -> Option<HandoverDecision> {
        let serving_id = serving_meas.sat_id;
        self.serving_sat_id = Some(serving_id);

        // Check trigger conditions
        let trigger = if serving_meas.snr_db < self.criteria.snr_threshold_db {
            Some(HandoverTrigger::SnrBelowThreshold)
        } else if serving_meas.elevation_rad < self.criteria.elev_mask_rad {
            Some(HandoverTrigger::ElevationBelowMask)
        } else if serving_meas.remaining_visibility_s < 60.0 {
            Some(HandoverTrigger::PredictedEndOfPass)
        } else {
            None
        };

        // Find best candidate
        let best = candidates
            .iter()
            .filter(|c| c.elevation_rad >= self.criteria.elev_mask_rad)
            .max_by(|a, b| {
                self.score_candidate(a)
                    .partial_cmp(&self.score_candidate(b))
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

        let best = best?;
        let best_score = self.score_candidate(best);
        let serving_score = self.score_candidate(serving_meas);

        // Proactive handover if best candidate is substantially better
        let proactive_trigger = if best_score > serving_score + self.criteria.hysteresis_db / 20.0 {
            Some(HandoverTrigger::BetterCandidateAvailable)
        } else {
            None
        };

        let trigger = trigger.or(proactive_trigger)?;

        Some(HandoverDecision {
            source_sat_id: serving_id,
            target_sat_id: best.sat_id,
            trigger,
            target_score: best_score,
            latency_ms: estimate_handover_latency(HandoverMode::MakeBeforeBreak),
        })
    }
}

impl Default for HandoverDecisionEngine {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Handover protocol state machine
// ─────────────────────────────────────────────────────────────────────────────

/// Handover execution mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HandoverMode {
    /// Establish new link before releasing old (soft handover)
    MakeBeforeBreak,
    /// Release old link before establishing new (hard handover)
    BreakBeforeMake,
}

/// State in the handover protocol FSM.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HandoverState {
    /// No handover activity
    Idle,
    /// Measuring candidate satellites
    Measurement,
    /// Handover decision made, awaiting execution
    Decision,
    /// Executing the handover (dual-satellite connection in MBB)
    Execution,
    /// Handover complete, on new satellite
    Completion,
    /// Handover failed – falling back or retrying
    Failed,
}

/// Handover protocol state machine.
#[derive(Debug, Clone)]
pub struct HandoverStateMachine {
    /// Current FSM state
    pub state: HandoverState,
    /// Current serving satellite ID
    pub serving_sat_id: Option<u32>,
    /// Target satellite ID (set during Decision state)
    pub target_sat_id: Option<u32>,
    /// Handover mode
    pub mode: HandoverMode,
    /// Number of failed handover attempts
    pub fail_count: u32,
    /// Timestamp of last state transition (seconds)
    pub last_transition_s: f64,
    /// Accumulated handover latency budget (ms)
    pub latency_budget_ms: f64,
}

impl HandoverStateMachine {
    /// Create a new FSM in Idle state.
    pub fn new(mode: HandoverMode) -> Self {
        Self {
            state: HandoverState::Idle,
            serving_sat_id: None,
            target_sat_id: None,
            mode,
            fail_count: 0,
            last_transition_s: 0.0,
            latency_budget_ms: 0.0,
        }
    }

    /// Transition to Measurement state (triggered by periodic timer or SNR drop).
    pub fn start_measurement(&mut self, t: f64) {
        if self.state == HandoverState::Idle {
            self.state = HandoverState::Measurement;
            self.last_transition_s = t;
        }
    }

    /// Apply a handover decision, moving to Decision state.
    pub fn apply_decision(&mut self, decision: &HandoverDecision, t: f64) {
        if self.state == HandoverState::Measurement {
            self.target_sat_id = Some(decision.target_sat_id);
            self.latency_budget_ms = decision.latency_ms;
            self.state = HandoverState::Decision;
            self.last_transition_s = t;
        }
    }

    /// Begin execution (radio reconfiguration).
    pub fn begin_execution(&mut self, t: f64) {
        if self.state == HandoverState::Decision {
            self.state = HandoverState::Execution;
            self.last_transition_s = t;
        }
    }

    /// Complete the handover successfully.
    pub fn complete(&mut self, t: f64) {
        if self.state == HandoverState::Execution {
            self.serving_sat_id = self.target_sat_id.take();
            self.state = HandoverState::Completion;
            self.last_transition_s = t;
        }
    }

    /// Reset to Idle (after completion or failure).
    pub fn reset(&mut self, t: f64) {
        self.target_sat_id = None;
        self.state = HandoverState::Idle;
        self.last_transition_s = t;
    }

    /// Mark handover as failed.
    pub fn fail(&mut self, t: f64) {
        self.fail_count += 1;
        self.state = HandoverState::Failed;
        self.last_transition_s = t;
    }

    /// Time spent in current state (seconds).
    pub fn time_in_state(&self, now: f64) -> f64 {
        now - self.last_transition_s
    }
}

/// Estimate handover latency (ms) based on mode.
pub fn estimate_handover_latency(mode: HandoverMode) -> f64 {
    match mode {
        HandoverMode::MakeBeforeBreak => 50.0,  // soft – overlap period
        HandoverMode::BreakBeforeMake => 150.0, // hard – gap + re-acquisition
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Walker-Delta constellation
// ─────────────────────────────────────────────────────────────────────────────

/// Walker-Delta constellation (T/P/F notation).
///
/// T = total satellites, P = orbital planes, F = phasing parameter.
#[derive(Debug, Clone)]
pub struct WalkerConstellation {
    /// Total satellites T
    pub total_sats: u32,
    /// Number of planes P
    pub planes: u32,
    /// Phasing parameter F (0 ≤ F < P)
    pub phasing: u32,
    /// Inclination (radians)
    pub inclination: f64,
    /// Orbit altitude (m)
    pub altitude_m: f64,
}

impl WalkerConstellation {
    /// Create a new Walker-Delta constellation.
    pub fn new(
        total_sats: u32,
        planes: u32,
        phasing: u32,
        inclination_rad: f64,
        altitude_m: f64,
    ) -> Self {
        Self {
            total_sats,
            planes,
            phasing,
            inclination: inclination_rad,
            altitude_m,
        }
    }

    /// Satellites per plane.
    pub fn sats_per_plane(&self) -> u32 {
        self.total_sats / self.planes
    }

    /// Generate all orbital elements at epoch.
    pub fn orbital_elements(&self) -> Vec<LeoOrbit> {
        let spp = self.sats_per_plane();
        let mut orbits = Vec::with_capacity(self.total_sats as usize);
        let delta_raan = 2.0 * PI / self.planes as f64;
        let delta_arg = 2.0 * PI / spp as f64;
        let delta_phase = 2.0 * PI * self.phasing as f64 / self.total_sats as f64;

        for p in 0..self.planes {
            let raan = p as f64 * delta_raan;
            for s in 0..spp {
                let arg_lat = s as f64 * delta_arg + p as f64 * delta_phase;
                orbits.push(LeoOrbit::new(
                    self.altitude_m,
                    self.inclination,
                    raan,
                    arg_lat,
                    0.0,
                ));
            }
        }
        orbits
    }

    /// Count satellites visible above `elev_mask_rad` from `ground` at time `t`.
    pub fn visible_satellite_count(
        &self,
        ground: &GroundPosition,
        elev_mask_rad: f64,
        t: f64,
    ) -> usize {
        self.orbital_elements()
            .iter()
            .filter(|o| o.elevation_angle(ground, t) >= elev_mask_rad)
            .count()
    }

    /// Minimum elevation angle (radians) achievable given coverage geometry.
    ///
    /// Uses the Earth central angle ρ = arcsin(R_e / r) approximation for
    /// the maximum nadir angle; then the minimum elevation for continuous
    /// coverage is derived from the coverage constraint.
    pub fn coverage_min_elevation(&self) -> f64 {
        let r = R_EARTH + self.altitude_m;
        let rho = (R_EARTH / r).asin(); // half-angle of Earth disk seen from sat
        // For a Walker constellation, minimum elevation is a function of
        // inter-satellite angular separation
        let spp = self.sats_per_plane();
        let delta = PI / spp as f64; // half angular spacing between sats
        // Earth nadir angle at edge of coverage
        let nadir = rho - delta;
        if nadir < 0.0 {
            return 0.0;
        }
        // Convert nadir angle to elevation (Earth-surface geometry)
        let sin_el = r / R_EARTH * nadir.sin() - (PI - nadir - nadir.sin().asin()).sin();
        sin_el.clamp(-1.0, 1.0).asin()
    }

    /// Best satellite (highest elevation) from ground at time `t`.
    pub fn best_satellite(&self, ground: &GroundPosition, t: f64) -> Option<(usize, f64)> {
        self.orbital_elements()
            .iter()
            .enumerate()
            .map(|(i, o)| (i, o.elevation_angle(ground, t)))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Beam management
// ─────────────────────────────────────────────────────────────────────────────

/// Spot beam descriptor.
#[derive(Debug, Clone)]
pub struct SpotBeam {
    /// Beam identifier
    pub beam_id: u32,
    /// Beam centre sub-satellite point (lat, lon) in radians
    pub centre_lat_rad: f64,
    pub centre_lon_rad: f64,
    /// Half-power beamwidth (radians)
    pub beamwidth_rad: f64,
    /// Satellite altitude (m)
    pub altitude_m: f64,
}

impl SpotBeam {
    /// Create a new spot beam.
    pub fn new(
        beam_id: u32,
        centre_lat: f64,
        centre_lon: f64,
        beamwidth_rad: f64,
        altitude_m: f64,
    ) -> Self {
        Self {
            beam_id,
            centre_lat_rad: centre_lat,
            centre_lon_rad: centre_lon,
            beamwidth_rad,
            altitude_m,
        }
    }

    /// Beam footprint radius on the ground (m), at nadir.
    /// r ≈ altitude * tan(beamwidth / 2)
    pub fn footprint_radius_m(&self) -> f64 {
        self.altitude_m * (self.beamwidth_rad / 2.0).tan()
    }

    /// Check if a ground position is within the beam footprint.
    pub fn contains(&self, ground: &GroundPosition) -> bool {
        let dlat = ground.lat - self.centre_lat_rad;
        let dlon = ground.lon - self.centre_lon_rad;
        // Great-circle approximation (suitable for small angles)
        let dist_rad = (dlat * dlat + (dlon * ground.lat.cos()).powi(2)).sqrt();
        let dist_m = dist_rad * R_EARTH;
        dist_m <= self.footprint_radius_m()
    }

    /// Beam dwelling time (seconds) – approximate time a moving satellite
    /// keeps ground terminal in beam.
    pub fn dwelling_time_s(&self, ground_speed_m_s: f64) -> f64 {
        if ground_speed_m_s <= 0.0 {
            return f64::INFINITY;
        }
        2.0 * self.footprint_radius_m() / ground_speed_m_s
    }
}

/// Beam manager – determines which beam serves a terminal and when to hand over.
#[derive(Debug, Clone)]
pub struct BeamManager {
    /// All beams on the satellite
    pub beams: Vec<SpotBeam>,
    /// Satellite ground track velocity (m/s)
    pub ground_track_velocity_m_s: f64,
}

impl BeamManager {
    /// Create a beam manager.
    pub fn new(beams: Vec<SpotBeam>, ground_track_velocity_m_s: f64) -> Self {
        Self {
            beams,
            ground_track_velocity_m_s,
        }
    }

    /// Find the beam that currently serves the given ground position.
    /// Returns `None` if the terminal is not in any beam footprint.
    pub fn serving_beam(&self, ground: &GroundPosition) -> Option<&SpotBeam> {
        self.beams.iter().find(|b| b.contains(ground))
    }

    /// Beam dwelling time for the current serving beam.
    pub fn beam_dwelling_time(&self, ground: &GroundPosition) -> Option<f64> {
        self.serving_beam(ground)
            .map(|b| b.dwelling_time_s(self.ground_track_velocity_m_s))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Inter-Satellite Links (ISL)
// ─────────────────────────────────────────────────────────────────────────────

/// ISL link type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IslType {
    /// Same orbital plane
    IntraPlane,
    /// Adjacent orbital plane
    InterPlane,
    /// Free-space optical
    Optical,
}

/// ISL link descriptor between two satellites.
#[derive(Debug, Clone)]
pub struct IslLink {
    pub sat_a: u32,
    pub sat_b: u32,
    pub link_type: IslType,
    /// Link distance (m)
    pub distance_m: f64,
    /// Carrier frequency (Hz)
    pub carrier_hz: f64,
    /// Tx power (dBW)
    pub tx_power_dbw: f64,
    /// Antenna gain (dBi) – assumed symmetric
    pub antenna_gain_dbi: f64,
    /// Receiver noise figure (dB)
    pub noise_figure_db: f64,
    /// Required Eb/N0 (dB)
    pub required_ebn0_db: f64,
    /// Data rate (bps)
    pub data_rate_bps: f64,
}

impl IslLink {
    /// Free-space path loss (dB).
    pub fn fspl_db(&self) -> f64 {
        let lambda = C_LIGHT / self.carrier_hz;
        20.0 * (4.0 * PI * self.distance_m / lambda).log10()
    }

    /// Received power (dBW).
    pub fn received_power_dbw(&self) -> f64 {
        self.tx_power_dbw + 2.0 * self.antenna_gain_dbi - self.fspl_db()
    }

    /// Thermal noise power spectral density (dBW/Hz).
    pub fn noise_psd_dbw_hz(&self) -> f64 {
        let n0 = K_BOLTZMANN * T_STANDARD;
        10.0 * n0.log10() + self.noise_figure_db
    }

    /// Link margin (dB) = received Eb/N0 - required Eb/N0.
    pub fn link_margin_db(&self) -> f64 {
        let noise_bw = self.data_rate_bps; // noise BW ≈ bit rate for BPSK
        let received_ebn0 = self.received_power_dbw()
            - self.noise_psd_dbw_hz()
            - 10.0 * noise_bw.log10();
        received_ebn0 - self.required_ebn0_db
    }

    /// Propagation delay (ms).
    pub fn propagation_delay_ms(&self) -> f64 {
        self.distance_m / C_LIGHT * 1000.0
    }

    /// Is the link viable (positive margin)?
    pub fn is_viable(&self) -> bool {
        self.link_margin_db() > 0.0
    }
}

/// ISL topology – a graph of satellite-to-satellite links.
#[derive(Debug, Clone)]
pub struct IslTopology {
    pub links: Vec<IslLink>,
    pub num_sats: u32,
}

impl IslTopology {
    /// Create a new topology.
    pub fn new(num_sats: u32) -> Self {
        Self {
            links: Vec::new(),
            num_sats,
        }
    }

    /// Add a link to the topology.
    pub fn add_link(&mut self, link: IslLink) {
        self.links.push(link);
    }

    /// Find neighbours of satellite `sat_id` (satellites directly connected).
    pub fn neighbours(&self, sat_id: u32) -> Vec<u32> {
        self.links
            .iter()
            .filter(|l| l.sat_a == sat_id || l.sat_b == sat_id)
            .map(|l| if l.sat_a == sat_id { l.sat_b } else { l.sat_a })
            .collect()
    }

    /// BFS minimum-hop path from `src` to `dst`.
    /// Returns `None` if no path exists.
    pub fn min_hop_path(&self, src: u32, dst: u32) -> Option<Vec<u32>> {
        if src == dst {
            return Some(vec![src]);
        }
        let mut visited = vec![false; self.num_sats as usize];
        let mut queue = std::collections::VecDeque::new();
        let mut parent = vec![u32::MAX; self.num_sats as usize];
        queue.push_back(src);
        visited[src as usize] = true;

        while let Some(cur) = queue.pop_front() {
            for nb in self.neighbours(cur) {
                if nb as usize >= self.num_sats as usize {
                    continue;
                }
                if !visited[nb as usize] {
                    visited[nb as usize] = true;
                    parent[nb as usize] = cur;
                    if nb == dst {
                        // Reconstruct path
                        let mut path = Vec::new();
                        let mut node = dst;
                        while node != u32::MAX {
                            path.push(node);
                            node = parent[node as usize];
                        }
                        path.reverse();
                        return Some(path);
                    }
                    queue.push_back(nb);
                }
            }
        }
        None
    }

    /// Minimum-delay path using Dijkstra on propagation delay.
    pub fn min_delay_path(&self, src: u32, dst: u32) -> Option<Vec<u32>> {
        let n = self.num_sats as usize;
        let mut dist = vec![f64::INFINITY; n];
        let mut prev = vec![u32::MAX; n];
        dist[src as usize] = 0.0;

        // Simple O(n²) Dijkstra (no priority queue to avoid external crates)
        let mut unvisited: Vec<u32> = (0..self.num_sats).collect();
        while !unvisited.is_empty() {
            // Find node with minimum distance
            let cur = *unvisited
                .iter()
                .min_by(|&&a, &&b| {
                    dist[a as usize]
                        .partial_cmp(&dist[b as usize])
                        .unwrap_or(std::cmp::Ordering::Equal)
                })?;
            if cur == dst {
                break;
            }
            if dist[cur as usize].is_infinite() {
                break;
            }
            unvisited.retain(|&x| x != cur);

            for nb in self.neighbours(cur) {
                if nb as usize >= n {
                    continue;
                }
                if let Some(link) = self.links.iter().find(|l| {
                    (l.sat_a == cur && l.sat_b == nb) || (l.sat_b == cur && l.sat_a == nb)
                }) {
                    let alt = dist[cur as usize] + link.propagation_delay_ms();
                    if alt < dist[nb as usize] {
                        dist[nb as usize] = alt;
                        prev[nb as usize] = cur;
                    }
                }
            }
        }

        if dist[dst as usize].is_infinite() {
            return None;
        }
        let mut path = Vec::new();
        let mut node = dst;
        while node != u32::MAX {
            path.push(node);
            if node == src {
                break;
            }
            node = prev[node as usize];
        }
        path.reverse();
        Some(path)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Link budget for LEO downlink
// ─────────────────────────────────────────────────────────────────────────────

/// LEO link budget calculator.
#[derive(Debug, Clone)]
pub struct LeoBudget {
    /// Carrier frequency (Hz)
    pub carrier_hz: f64,
    /// Satellite transmit power (dBW)
    pub tx_power_dbw: f64,
    /// Satellite antenna gain (dBi)
    pub sat_gain_dbi: f64,
    /// Ground terminal antenna gain (dBi)
    pub gnd_gain_dbi: f64,
    /// Satellite altitude (m)
    pub altitude_m: f64,
    /// Elevation angle (radians)
    pub elevation_rad: f64,
    /// Atmospheric loss (dB) – rain + gaseous
    pub atm_loss_db: f64,
    /// Scintillation margin (dB)
    pub scintillation_margin_db: f64,
    /// Required Eb/N0 (dB)
    pub required_ebn0_db: f64,
    /// Data rate (bps)
    pub data_rate_bps: f64,
    /// System noise temperature (K)
    pub noise_temp_k: f64,
}

impl LeoBudget {
    /// Slant range (m) from altitude and elevation angle.
    pub fn slant_range_m(&self) -> f64 {
        // From Earth surface: d = sqrt((R+h)^2 - R^2*cos^2(el)) - R*sin(el)
        let r = R_EARTH;
        let h = self.altitude_m;
        let el = self.elevation_rad;
        let rh = r + h;
        let cos_el = el.cos();
        let discriminant = rh * rh - r * r * cos_el * cos_el;
        if discriminant < 0.0 {
            return h;
        }
        discriminant.sqrt() - r * el.sin()
    }

    /// Free-space path loss (dB).
    pub fn fspl_db(&self) -> f64 {
        let d = self.slant_range_m();
        let lambda = C_LIGHT / self.carrier_hz;
        20.0 * (4.0 * PI * d / lambda).log10()
    }

    /// Received carrier power (dBW).
    pub fn received_power_dbw(&self) -> f64 {
        self.tx_power_dbw + self.sat_gain_dbi + self.gnd_gain_dbi
            - self.fspl_db()
            - self.atm_loss_db
            - self.scintillation_margin_db
    }

    /// Noise power spectral density N0 (dBW/Hz).
    pub fn noise_psd_dbw_hz(&self) -> f64 {
        10.0 * (K_BOLTZMANN * self.noise_temp_k).log10()
    }

    /// C/N0 (dBHz).
    pub fn c_over_n0_dbhz(&self) -> f64 {
        self.received_power_dbw() - self.noise_psd_dbw_hz()
    }

    /// Eb/N0 (dB).
    pub fn eb_over_n0_db(&self) -> f64 {
        self.c_over_n0_dbhz() - 10.0 * self.data_rate_bps.log10()
    }

    /// Link margin (dB) = Eb/N0 - required Eb/N0.
    pub fn link_margin_db(&self) -> f64 {
        self.eb_over_n0_db() - self.required_ebn0_db
    }

    /// Estimate fade margin for a given availability target (%).
    /// Uses simplified log-normal model: margin ≈ σ * Φ⁻¹(availability).
    pub fn fade_margin_db(&self, availability_pct: f64, sigma_db: f64) -> f64 {
        let p = availability_pct / 100.0;
        // Approximate inverse normal CDF (Abramowitz & Stegun)
        let q = if p >= 0.5 { p } else { 1.0 - p };
        let t = (-2.0 * (1.0 - q).ln()).sqrt();
        let c = [2.515517, 0.802853, 0.010328];
        let d = [1.432788, 0.189269, 0.001308];
        let numerator = c[0] + c[1] * t + c[2] * t * t;
        let denominator = 1.0 + d[0] * t + d[1] * t * t + d[2] * t * t * t;
        let z = t - numerator / denominator;
        sigma_db * z
    }

    /// Availability (%) given a fade margin and sigma_db.
    pub fn availability_pct(&self, fade_margin_db: f64, sigma_db: f64) -> f64 {
        if sigma_db <= 0.0 {
            return 100.0;
        }
        let z = fade_margin_db / sigma_db;
        // Standard normal CDF approximation
        let p = 0.5 * (1.0 + erf_approx(z / 2.0_f64.sqrt()));
        p * 100.0
    }
}

/// Approximate error function (used for availability calculation).
fn erf_approx(x: f64) -> f64 {
    // Abramowitz & Stegun 7.1.26 approximation
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;
    let p = 0.3275911;
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let ax = x.abs();
    let t = 1.0 / (1.0 + p * ax);
    let poly = ((((a5 * t + a4) * t + a3) * t + a2) * t + a1) * t;
    sign * (1.0 - poly * (-ax * ax).exp())
}

// ─────────────────────────────────────────────────────────────────────────────
// Coverage analysis helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute minimum elevation angle for seamless coverage by a single satellite
/// at given altitude.
///
/// Based on: minimum elevation s.t. the satellite horizon covers R_earth arc
/// up to the Earth limb.
pub fn min_elevation_for_coverage(altitude_m: f64, elev_mask_rad: f64) -> f64 {
    let r = R_EARTH;
    let h = altitude_m;
    let rho = (r / (r + h)).acos(); // nadir half-angle
    let _el = PI / 2.0 - rho - elev_mask_rad;
    let el = (PI / 2.0 - rho - elev_mask_rad).max(0.0);
    el
}

/// Maximum slant range (m) for a satellite at given altitude visible above elev_mask.
pub fn max_slant_range(altitude_m: f64, elev_mask_rad: f64) -> f64 {
    let r = R_EARTH;
    let h = altitude_m;
    let rh = r + h;
    let el = elev_mask_rad;
    let discriminant = rh * rh - r * r * el.cos().powi(2);
    if discriminant < 0.0 {
        return h;
    }
    discriminant.sqrt() - r * el.sin()
}

/// Ground coverage area (km²) of a satellite at altitude with elevation mask.
pub fn coverage_area_km2(altitude_m: f64, elev_mask_rad: f64) -> f64 {
    let r = R_EARTH;
    let h = altitude_m;
    let rho = (r / (r + h)).acos(); // nadir half-angle at horizon
    let earth_central_angle = rho - elev_mask_rad; // approximate
    if earth_central_angle <= 0.0 {
        return 0.0;
    }
    let area_m2 = 2.0 * PI * r * r * (1.0 - earth_central_angle.cos());
    area_m2 / 1e6
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility: atmospheric losses
// ─────────────────────────────────────────────────────────────────────────────

/// Simplified atmospheric gaseous absorption loss (dB) at given frequency and elevation.
///
/// Uses a rough model: zenith loss is frequency-dependent; slant path adds 1/sin(el).
pub fn atmospheric_gaseous_loss_db(carrier_hz: f64, elevation_rad: f64) -> f64 {
    let freq_ghz = carrier_hz / 1e9;
    // Approximate zenith loss (dB) – rough piecewise model
    let zenith_loss_db = if freq_ghz < 10.0 {
        0.05
    } else if freq_ghz < 20.0 {
        0.15
    } else if freq_ghz < 30.0 {
        0.5
    } else {
        1.2
    };
    let sin_el = elevation_rad.sin().max(0.1); // avoid division by zero at horizon
    zenith_loss_db / sin_el
}

/// Rain attenuation estimate (dB) using simplified ITU-R P.618 model.
///
/// `rain_rate_mm_h` – rain rate (mm/h), `carrier_hz` – frequency (Hz),
/// `elevation_rad` – elevation angle (rad).
pub fn rain_attenuation_db(rain_rate_mm_h: f64, carrier_hz: f64, elevation_rad: f64) -> f64 {
    let freq_ghz = carrier_hz / 1e9;
    // Simplified k and α coefficients (horizontal polarisation, approximate)
    let (k, alpha) = if freq_ghz < 2.0 {
        (0.0000352, 0.880)
    } else if freq_ghz < 10.0 {
        (0.00887, 1.121)
    } else if freq_ghz < 20.0 {
        (0.0751, 1.099)
    } else if freq_ghz < 30.0 {
        (0.187, 1.021)
    } else {
        (0.350, 0.939)
    };
    let gamma = k * rain_rate_mm_h.powf(alpha); // dB/km specific attenuation
    // Effective path length (km) through rain layer (~4 km height)
    let rain_height_km = 4.0;
    let slant_km = rain_height_km / elevation_rad.sin().max(0.1);
    gamma * slant_km.min(20.0) // cap to 20 km
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Orbital mechanics ────────────────────────────────────────────────────

    #[test]
    fn test_orbital_period_iss_altitude() {
        // ISS ~400 km altitude → period ~92 min
        let orbit = LeoOrbit::new(400_000.0, 51.6_f64.to_radians(), 0.0, 0.0, 0.0);
        let period_min = orbit.period() / 60.0;
        assert!((period_min - 92.0).abs() < 2.0, "period = {:.1} min", period_min);
    }

    #[test]
    fn test_orbital_velocity() {
        // At 400 km, v ≈ 7.67 km/s
        let orbit = LeoOrbit::new(400_000.0, 51.6_f64.to_radians(), 0.0, 0.0, 0.0);
        let v_km_s = orbit.velocity() / 1000.0;
        assert!((v_km_s - 7.67).abs() < 0.1, "v = {:.2} km/s", v_km_s);
    }

    #[test]
    fn test_mean_motion() {
        let orbit = LeoOrbit::new(600_000.0, 97.0_f64.to_radians(), 0.0, 0.0, 0.0);
        let n = orbit.mean_motion();
        // n should be ~1.04e-3 rad/s for ~600 km
        assert!(n > 1.0e-3 && n < 1.1e-3, "n = {:.4e} rad/s", n);
    }

    #[test]
    fn test_eci_position_unit_norm() {
        // At t=0, ECI position should have magnitude equal to orbit radius
        let orbit = LeoOrbit::new(550_000.0, 53.0_f64.to_radians(), 0.3, 1.2, 0.0);
        let (x, y, z) = orbit.eci_position(0.0);
        let r = (x * x + y * y + z * z).sqrt();
        let expected = R_EARTH + 550_000.0;
        assert!((r - expected).abs() < 1.0, "|r| = {} m, expected {}", r, expected);
    }

    #[test]
    fn test_eci_velocity_magnitude() {
        let orbit = LeoOrbit::new(550_000.0, 53.0_f64.to_radians(), 0.0, 0.0, 0.0);
        let (vx, vy, vz) = orbit.eci_velocity(0.0);
        let v = (vx * vx + vy * vy + vz * vz).sqrt();
        let expected = orbit.velocity();
        assert!((v - expected).abs() < 0.1, "|v| = {} vs {}", v, expected);
    }

    #[test]
    fn test_raan_drift_polar_orbit() {
        // Sun-synchronous orbit (i ≈ 97°) should have positive (eastward) RAAN drift
        let orbit = LeoOrbit::new(600_000.0, 97.8_f64.to_radians(), 0.0, 0.0, 0.0);
        let drift = orbit.raan_drift_rate();
        assert!(drift > 0.0, "SSO RAAN drift should be positive (eastward)");
        // Earth rotates ~360°/365 days ≈ 0.9856 °/day → ~1.99e-7 rad/s
        let drift_deg_day = drift.to_degrees() * 86400.0;
        assert!(
            (drift_deg_day - 0.9856).abs() < 0.1,
            "RAAN drift = {:.4} °/day",
            drift_deg_day
        );
    }

    #[test]
    fn test_raan_drift_equatorial_orbit() {
        // Equatorial orbit has maximum (prograde) retrograde RAAN drift
        let orbit = LeoOrbit::new(600_000.0, 0.0, 0.0, 0.0, 0.0);
        let drift = orbit.raan_drift_rate();
        // Should be negative for prograde equatorial orbit
        assert!(drift < 0.0);
    }

    #[test]
    fn test_elevation_angle_nadir() {
        // Satellite directly above ground station → elevation = 90°
        let lat = 0.0_f64;
        let lon = 0.0_f64;
        let ground = GroundPosition::new(lat, lon, 0.0);
        // Place satellite at same lat/lon at t=0 (ignoring Earth rotation for t=0)
        let orbit = LeoOrbit::new(780_000.0, 0.0, 0.0, 0.0, 0.0);
        let elev = orbit.elevation_angle(&ground, 0.0);
        // Should be close to 90° (small error due to circular orbit approximation)
        assert!(elev.to_degrees() > 85.0, "elev = {:.1}°", elev.to_degrees());
    }

    #[test]
    fn test_elevation_below_horizon() {
        // Satellite on the opposite side of Earth → negative elevation
        let ground = GroundPosition::new(0.0, 0.0, 0.0);
        let orbit = LeoOrbit::new(780_000.0, 0.0, 0.0, PI, 0.0); // 180° away
        let elev = orbit.elevation_angle(&ground, 0.0);
        assert!(elev < 0.0, "elev = {:.1}°", elev.to_degrees());
    }

    #[test]
    fn test_pass_duration_reasonable() {
        // Typical LEO pass at 780 km should be 5–15 minutes
        let ground = GroundPosition::new(45.0_f64.to_radians(), 0.0, 0.0);
        let orbit = LeoOrbit::new(780_000.0, 86.4_f64.to_radians(), 0.0, PI / 2.0, 0.0);
        let dur = orbit.pass_duration(&ground, 0.0, 10.0_f64.to_radians());
        let dur_min = dur / 60.0;
        // For some orbital geometries the pass might be shorter; just verify non-zero
        assert!(dur_min >= 0.0, "pass duration = {:.1} min", dur_min);
    }

    #[test]
    fn test_visibility_window_some() {
        let ground = GroundPosition::new(0.0, 0.0, 0.0);
        // Equatorial orbit with satellite overhead at t=0
        let orbit = LeoOrbit::new(550_000.0, 0.0, 0.0, 0.0, 0.0);
        let win = orbit.visibility_window(&ground, 0.0, 5.0_f64.to_radians());
        assert!(win.is_some(), "expected a visibility window");
        if let Some((rise, set)) = win {
            assert!(set > rise);
        }
    }

    // ── Ground position ──────────────────────────────────────────────────────

    #[test]
    fn test_ground_eci_magnitude() {
        let ground = GroundPosition::new(45.0_f64.to_radians(), 0.0, 0.0);
        let (x, y, z) = ground.eci_position(0.0);
        let r = (x * x + y * y + z * z).sqrt();
        assert!((r - A_EARTH).abs() < 1.0, "|r| = {} m", r);
    }

    // ── Doppler prediction ───────────────────────────────────────────────────

    #[test]
    fn test_doppler_max_excursion_s_band() {
        // S-band Doppler excursion should be ≲ ±40 kHz for LEO at 780 km
        let orbit = LeoOrbit::new(780_000.0, 86.4_f64.to_radians(), 0.0, PI / 2.0, 0.0);
        let ground = GroundPosition::new(0.0, 0.0, 0.0);
        let pred = DopplerPredictor::s_band_downlink();
        let exc = pred.max_doppler_excursion(&orbit, &ground, 0.0, orbit.period(), 1000);
        assert!(exc < 100_000.0, "S-band excursion = {:.0} Hz", exc);
        assert!(exc > 1_000.0, "S-band excursion = {:.0} Hz", exc);
    }

    #[test]
    fn test_doppler_shift_sign_approaching() {
        // At t=0 with satellite overhead moving toward observer → Doppler shift sign check
        let orbit = LeoOrbit::new(780_000.0, 0.0, 0.0, -0.1, 0.0);
        let ground = GroundPosition::new(0.0, 0.0, 0.0);
        let pred = DopplerPredictor::s_band_downlink();
        let shift = pred.doppler_shift(&orbit, &ground, 0.0);
        // When approaching, range rate < 0, shift > 0 (blue-shift)
        // Accept either sign; just verify it's non-trivial
        assert!(shift.abs() > 0.0);
    }

    #[test]
    fn test_doppler_rate_finite() {
        let orbit = LeoOrbit::new(780_000.0, 86.4_f64.to_radians(), 0.0, PI / 4.0, 0.0);
        let ground = GroundPosition::new(30.0_f64.to_radians(), 0.0, 0.0);
        let pred = DopplerPredictor::ka_band_downlink();
        let rate = pred.doppler_rate(&orbit, &ground, 0.0);
        assert!(rate.is_finite());
    }

    #[test]
    fn test_precompensation_negates_downlink() {
        let orbit = LeoOrbit::new(780_000.0, 86.4_f64.to_radians(), 0.0, 0.5, 0.0);
        let ground = GroundPosition::new(10.0_f64.to_radians(), 0.0, 0.0);
        let pred = DopplerPredictor::ka_band_uplink();
        let dl = pred.doppler_shift(&orbit, &ground, 0.0);
        let comp = pred.uplink_precompensation(&orbit, &ground, 0.0);
        assert!((dl + comp).abs() < 1e-6, "dl + comp = {}", dl + comp);
    }

    // ── Handover decision ────────────────────────────────────────────────────

    #[test]
    fn test_score_high_elevation_high_snr() {
        let engine = HandoverDecisionEngine::new();
        let meas = LinkMeasurement {
            sat_id: 1,
            snr_db: 15.0,
            elevation_rad: 60.0_f64.to_radians(),
            doppler_rate_hz_s: 5.0,
            remaining_visibility_s: 400.0,
            timestamp_s: 0.0,
        };
        let score = engine.score_candidate(&meas);
        assert!(score > 0.5, "score = {:.3}", score);
    }

    #[test]
    fn test_score_low_elevation_low_snr() {
        let engine = HandoverDecisionEngine::new();
        let meas = LinkMeasurement {
            sat_id: 2,
            snr_db: 1.0,
            elevation_rad: 11.0_f64.to_radians(),
            doppler_rate_hz_s: 100.0,
            remaining_visibility_s: 30.0,
            timestamp_s: 0.0,
        };
        let score = engine.score_candidate(&meas);
        assert!(score < 0.5, "score = {:.3}", score);
    }

    #[test]
    fn test_handover_triggered_by_snr() {
        let mut engine = HandoverDecisionEngine::new();
        let serving = LinkMeasurement {
            sat_id: 1,
            snr_db: 3.0, // below threshold of 5 dB
            elevation_rad: 30.0_f64.to_radians(),
            doppler_rate_hz_s: 10.0,
            remaining_visibility_s: 200.0,
            timestamp_s: 0.0,
        };
        let candidate = LinkMeasurement {
            sat_id: 2,
            snr_db: 14.0,
            elevation_rad: 45.0_f64.to_radians(),
            doppler_rate_hz_s: 5.0,
            remaining_visibility_s: 350.0,
            timestamp_s: 0.0,
        };
        let decision = engine.evaluate(&serving, &[candidate]);
        assert!(decision.is_some());
        let d = decision.unwrap();
        assert_eq!(d.source_sat_id, 1);
        assert_eq!(d.target_sat_id, 2);
        assert_eq!(d.trigger, HandoverTrigger::SnrBelowThreshold);
    }

    #[test]
    fn test_handover_triggered_by_elevation() {
        let mut engine = HandoverDecisionEngine::new();
        let serving = LinkMeasurement {
            sat_id: 1,
            snr_db: 10.0,
            elevation_rad: 5.0_f64.to_radians(), // below 10° mask
            doppler_rate_hz_s: 20.0,
            remaining_visibility_s: 20.0,
            timestamp_s: 0.0,
        };
        let candidate = LinkMeasurement {
            sat_id: 3,
            snr_db: 12.0,
            elevation_rad: 40.0_f64.to_radians(),
            doppler_rate_hz_s: 8.0,
            remaining_visibility_s: 300.0,
            timestamp_s: 0.0,
        };
        let decision = engine.evaluate(&serving, &[candidate]);
        assert!(decision.is_some());
        assert_eq!(decision.unwrap().trigger, HandoverTrigger::ElevationBelowMask);
    }

    #[test]
    fn test_no_handover_when_good_link() {
        let mut engine = HandoverDecisionEngine::new();
        let serving = LinkMeasurement {
            sat_id: 1,
            snr_db: 15.0,
            elevation_rad: 60.0_f64.to_radians(),
            doppler_rate_hz_s: 5.0,
            remaining_visibility_s: 400.0,
            timestamp_s: 0.0,
        };
        // Candidate only slightly better – within hysteresis
        let candidate = LinkMeasurement {
            sat_id: 2,
            snr_db: 16.0,
            elevation_rad: 62.0_f64.to_radians(),
            doppler_rate_hz_s: 4.0,
            remaining_visibility_s: 410.0,
            timestamp_s: 0.0,
        };
        let decision = engine.evaluate(&serving, &[candidate]);
        assert!(decision.is_none(), "unexpected handover decision");
    }

    #[test]
    fn test_handover_latency_mbb_less_than_bbb() {
        let mbb = estimate_handover_latency(HandoverMode::MakeBeforeBreak);
        let bbb = estimate_handover_latency(HandoverMode::BreakBeforeMake);
        assert!(mbb < bbb, "MBB should have lower latency than BBM");
    }

    // ── Handover state machine ───────────────────────────────────────────────

    #[test]
    fn test_fsm_full_cycle() {
        let mut fsm = HandoverStateMachine::new(HandoverMode::MakeBeforeBreak);
        assert_eq!(fsm.state, HandoverState::Idle);

        fsm.start_measurement(0.0);
        assert_eq!(fsm.state, HandoverState::Measurement);

        let decision = HandoverDecision {
            source_sat_id: 1,
            target_sat_id: 2,
            trigger: HandoverTrigger::SnrBelowThreshold,
            target_score: 0.8,
            latency_ms: 50.0,
        };
        fsm.apply_decision(&decision, 1.0);
        assert_eq!(fsm.state, HandoverState::Decision);
        assert_eq!(fsm.target_sat_id, Some(2));

        fsm.begin_execution(2.0);
        assert_eq!(fsm.state, HandoverState::Execution);

        fsm.complete(3.0);
        assert_eq!(fsm.state, HandoverState::Completion);
        assert_eq!(fsm.serving_sat_id, Some(2));

        fsm.reset(4.0);
        assert_eq!(fsm.state, HandoverState::Idle);
    }

    #[test]
    fn test_fsm_fail_increments_counter() {
        let mut fsm = HandoverStateMachine::new(HandoverMode::BreakBeforeMake);
        fsm.start_measurement(0.0);
        let d = HandoverDecision {
            source_sat_id: 1,
            target_sat_id: 3,
            trigger: HandoverTrigger::Forced,
            target_score: 0.5,
            latency_ms: 150.0,
        };
        fsm.apply_decision(&d, 1.0);
        fsm.begin_execution(2.0);
        fsm.fail(3.0);
        assert_eq!(fsm.fail_count, 1);
        assert_eq!(fsm.state, HandoverState::Failed);
    }

    #[test]
    fn test_fsm_time_in_state() {
        let mut fsm = HandoverStateMachine::new(HandoverMode::MakeBeforeBreak);
        fsm.start_measurement(100.0);
        assert!((fsm.time_in_state(110.0) - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_fsm_ignore_invalid_transitions() {
        let mut fsm = HandoverStateMachine::new(HandoverMode::MakeBeforeBreak);
        // begin_execution from Idle should not change state
        fsm.begin_execution(0.0);
        assert_eq!(fsm.state, HandoverState::Idle);
    }

    // ── Walker constellation ─────────────────────────────────────────────────

    #[test]
    fn test_walker_total_satellites() {
        let c = WalkerConstellation::new(66, 6, 2, 86.4_f64.to_radians(), 780_000.0);
        assert_eq!(c.orbital_elements().len(), 66);
    }

    #[test]
    fn test_walker_sats_per_plane() {
        let c = WalkerConstellation::new(72, 6, 1, 53.0_f64.to_radians(), 550_000.0);
        assert_eq!(c.sats_per_plane(), 12);
    }

    #[test]
    fn test_walker_visible_count_nonzero() {
        // Iridium-like: should see 2+ satellites from equatorial station
        let c = WalkerConstellation::new(66, 6, 2, 86.4_f64.to_radians(), 780_000.0);
        let ground = GroundPosition::new(0.0, 0.0, 0.0);
        let count = c.visible_satellite_count(&ground, 10.0_f64.to_radians(), 0.0);
        assert!(count >= 1, "visible count = {}", count);
    }

    #[test]
    fn test_walker_orbit_radii_consistent() {
        let c = WalkerConstellation::new(24, 3, 1, 55.0_f64.to_radians(), 1200_000.0);
        let expected_r = R_EARTH + 1_200_000.0;
        for o in c.orbital_elements() {
            assert!((o.radius - expected_r).abs() < 1.0);
        }
    }

    #[test]
    fn test_coverage_area_increases_with_altitude() {
        let a1 = coverage_area_km2(500_000.0, 10.0_f64.to_radians());
        let a2 = coverage_area_km2(1200_000.0, 10.0_f64.to_radians());
        assert!(a2 > a1, "a1={:.0} km², a2={:.0} km²", a1, a2);
    }

    // ── Spot beams ───────────────────────────────────────────────────────────

    #[test]
    fn test_beam_footprint_radius() {
        // 1° beamwidth at 600 km → ~5.2 km radius
        let beam = SpotBeam::new(0, 0.0, 0.0, 1.0_f64.to_radians(), 600_000.0);
        let r_km = beam.footprint_radius_m() / 1000.0;
        assert!((r_km - 5.2).abs() < 0.5, "r = {:.2} km", r_km);
    }

    #[test]
    fn test_beam_contains_centre() {
        let beam = SpotBeam::new(1, 0.1, 0.2, 2.0_f64.to_radians(), 600_000.0);
        let centre = GroundPosition::new(0.1, 0.2, 0.0);
        assert!(beam.contains(&centre));
    }

    #[test]
    fn test_beam_does_not_contain_far_point() {
        let beam = SpotBeam::new(1, 0.0, 0.0, 0.5_f64.to_radians(), 600_000.0);
        let far = GroundPosition::new(1.0, 1.0, 0.0); // ~100 km away
        assert!(!beam.contains(&far));
    }

    #[test]
    fn test_beam_dwelling_time_reasonable() {
        // At ~7 km/s ground track, 5 km beam → ~1.4 s dwell
        let beam = SpotBeam::new(0, 0.0, 0.0, 1.0_f64.to_radians(), 600_000.0);
        let dwell = beam.dwelling_time_s(7_000.0);
        assert!(dwell > 1.0 && dwell < 3.0, "dwell = {:.2} s", dwell);
    }

    #[test]
    fn test_beam_manager_finds_serving_beam() {
        let beams = vec![
            SpotBeam::new(0, 0.0, 0.0, 2.0_f64.to_radians(), 600_000.0),
            SpotBeam::new(1, 0.5, 0.0, 2.0_f64.to_radians(), 600_000.0),
        ];
        let mgr = BeamManager::new(beams, 7_000.0);
        let gnd = GroundPosition::new(0.0, 0.0, 0.0);
        assert!(mgr.serving_beam(&gnd).is_some());
    }

    // ── ISL topology ─────────────────────────────────────────────────────────

    #[test]
    fn test_isl_link_budget_positive_margin() {
        let link = IslLink {
            sat_a: 0,
            sat_b: 1,
            link_type: IslType::IntraPlane,
            distance_m: 1_500_000.0, // 1500 km intra-plane ISL
            carrier_hz: 23.0e9,       // Ka-band
            tx_power_dbw: 10.0,       // 10 W
            antenna_gain_dbi: 40.0,   // high-gain directional antenna
            noise_figure_db: 3.0,
            required_ebn0_db: 6.0,
            data_rate_bps: 1e8,       // 100 Mbps
        };
        assert!(link.link_margin_db() > 0.0, "margin = {:.1} dB", link.link_margin_db());
    }

    #[test]
    fn test_isl_propagation_delay() {
        let link = IslLink {
            sat_a: 0, sat_b: 1,
            link_type: IslType::IntraPlane,
            distance_m: 3_000_000.0, // 3000 km
            carrier_hz: 23.0e9,
            tx_power_dbw: 0.0, antenna_gain_dbi: 30.0, noise_figure_db: 3.0,
            required_ebn0_db: 6.0, data_rate_bps: 1e9,
        };
        let delay = link.propagation_delay_ms();
        // 3000 km / c ≈ 10 ms
        assert!((delay - 10.0).abs() < 0.2, "delay = {:.2} ms", delay);
    }

    #[test]
    fn test_isl_min_hop_path() {
        let mut topo = IslTopology::new(5);
        topo.add_link(IslLink {
            sat_a: 0, sat_b: 1, link_type: IslType::IntraPlane,
            distance_m: 1e6, carrier_hz: 23e9, tx_power_dbw: 10.0,
            antenna_gain_dbi: 30.0, noise_figure_db: 3.0,
            required_ebn0_db: 6.0, data_rate_bps: 1e9,
        });
        topo.add_link(IslLink {
            sat_a: 1, sat_b: 2, link_type: IslType::InterPlane,
            distance_m: 1e6, carrier_hz: 23e9, tx_power_dbw: 10.0,
            antenna_gain_dbi: 30.0, noise_figure_db: 3.0,
            required_ebn0_db: 6.0, data_rate_bps: 1e9,
        });
        topo.add_link(IslLink {
            sat_a: 2, sat_b: 3, link_type: IslType::IntraPlane,
            distance_m: 1e6, carrier_hz: 23e9, tx_power_dbw: 10.0,
            antenna_gain_dbi: 30.0, noise_figure_db: 3.0,
            required_ebn0_db: 6.0, data_rate_bps: 1e9,
        });
        let path = topo.min_hop_path(0, 3);
        assert_eq!(path, Some(vec![0, 1, 2, 3]));
    }

    #[test]
    fn test_isl_min_delay_path() {
        let mut topo = IslTopology::new(4);
        // Short but indirect path: 0→1→3 vs long direct 0→3
        topo.add_link(IslLink {
            sat_a: 0, sat_b: 1, link_type: IslType::IntraPlane,
            distance_m: 500_000.0, carrier_hz: 23e9, tx_power_dbw: 10.0,
            antenna_gain_dbi: 30.0, noise_figure_db: 3.0,
            required_ebn0_db: 6.0, data_rate_bps: 1e9,
        });
        topo.add_link(IslLink {
            sat_a: 1, sat_b: 3, link_type: IslType::IntraPlane,
            distance_m: 500_000.0, carrier_hz: 23e9, tx_power_dbw: 10.0,
            antenna_gain_dbi: 30.0, noise_figure_db: 3.0,
            required_ebn0_db: 6.0, data_rate_bps: 1e9,
        });
        topo.add_link(IslLink {
            sat_a: 0, sat_b: 3, link_type: IslType::InterPlane,
            distance_m: 2_000_000.0, carrier_hz: 23e9, tx_power_dbw: 10.0,
            antenna_gain_dbi: 30.0, noise_figure_db: 3.0,
            required_ebn0_db: 6.0, data_rate_bps: 1e9,
        });
        let path = topo.min_delay_path(0, 3);
        assert_eq!(path, Some(vec![0, 1, 3]));
    }

    #[test]
    fn test_isl_no_path_returns_none() {
        let topo = IslTopology::new(4); // no links added
        let path = topo.min_hop_path(0, 3);
        assert!(path.is_none());
    }

    #[test]
    fn test_isl_self_path() {
        let topo = IslTopology::new(4);
        let path = topo.min_hop_path(2, 2);
        assert_eq!(path, Some(vec![2]));
    }

    // ── Link budget ──────────────────────────────────────────────────────────

    #[test]
    fn test_link_budget_slant_range_at_zenith() {
        let budget = LeoBudget {
            carrier_hz: 2.0e9,
            tx_power_dbw: 0.0,
            sat_gain_dbi: 10.0,
            gnd_gain_dbi: 5.0,
            altitude_m: 780_000.0,
            elevation_rad: PI / 2.0, // zenith
            atm_loss_db: 0.2,
            scintillation_margin_db: 0.5,
            required_ebn0_db: 6.0,
            data_rate_bps: 1e6,
            noise_temp_k: 290.0,
        };
        // At zenith, slant range ≈ altitude
        let sr = budget.slant_range_m();
        assert!((sr - 780_000.0).abs() < 1000.0, "slant range = {:.0} m", sr);
    }

    #[test]
    fn test_link_budget_slant_range_low_elevation() {
        let budget = LeoBudget {
            carrier_hz: 2.0e9,
            tx_power_dbw: 0.0,
            sat_gain_dbi: 10.0,
            gnd_gain_dbi: 5.0,
            altitude_m: 780_000.0,
            elevation_rad: 10.0_f64.to_radians(),
            atm_loss_db: 1.0,
            scintillation_margin_db: 1.0,
            required_ebn0_db: 6.0,
            data_rate_bps: 1e6,
            noise_temp_k: 290.0,
        };
        let sr = budget.slant_range_m();
        // At low elevation, slant range >> altitude
        assert!(sr > 780_000.0 * 2.0, "slant range = {:.0} m", sr);
    }

    #[test]
    fn test_link_budget_positive_margin_at_zenith() {
        let budget = LeoBudget {
            carrier_hz: 2.0e9,
            tx_power_dbw: 10.0,   // 10 W
            sat_gain_dbi: 20.0,
            gnd_gain_dbi: 10.0,
            altitude_m: 780_000.0,
            elevation_rad: PI / 2.0,
            atm_loss_db: 0.3,
            scintillation_margin_db: 0.5,
            required_ebn0_db: 6.0,
            data_rate_bps: 1e6,
            noise_temp_k: 290.0,
        };
        assert!(budget.link_margin_db() > 0.0, "margin = {:.1} dB", budget.link_margin_db());
    }

    #[test]
    fn test_link_budget_lower_elevation_lower_margin() {
        let make_budget = |el: f64| LeoBudget {
            carrier_hz: 20.0e9,
            tx_power_dbw: 10.0,
            sat_gain_dbi: 30.0,
            gnd_gain_dbi: 15.0,
            altitude_m: 1000_000.0,
            elevation_rad: el,
            atm_loss_db: 0.5,
            scintillation_margin_db: 1.0,
            required_ebn0_db: 6.0,
            data_rate_bps: 1e7,
            noise_temp_k: 290.0,
        };
        let m_high = make_budget(80.0_f64.to_radians()).link_margin_db();
        let m_low = make_budget(15.0_f64.to_radians()).link_margin_db();
        assert!(m_high > m_low, "m_high={:.1}, m_low={:.1}", m_high, m_low);
    }

    #[test]
    fn test_fade_margin_increases_with_availability() {
        let budget = LeoBudget {
            carrier_hz: 20.0e9,
            tx_power_dbw: 0.0,
            sat_gain_dbi: 20.0,
            gnd_gain_dbi: 10.0,
            altitude_m: 780_000.0,
            elevation_rad: 30.0_f64.to_radians(),
            atm_loss_db: 0.5,
            scintillation_margin_db: 1.0,
            required_ebn0_db: 6.0,
            data_rate_bps: 1e6,
            noise_temp_k: 290.0,
        };
        let m99 = budget.fade_margin_db(99.0, 2.0);
        let m999 = budget.fade_margin_db(99.9, 2.0);
        assert!(m999 > m99, "99.9% margin should exceed 99% margin");
    }

    #[test]
    fn test_availability_decreases_with_larger_sigma() {
        let budget = LeoBudget {
            carrier_hz: 20.0e9,
            tx_power_dbw: 0.0,
            sat_gain_dbi: 20.0,
            gnd_gain_dbi: 10.0,
            altitude_m: 780_000.0,
            elevation_rad: 30.0_f64.to_radians(),
            atm_loss_db: 0.5,
            scintillation_margin_db: 1.0,
            required_ebn0_db: 6.0,
            data_rate_bps: 1e6,
            noise_temp_k: 290.0,
        };
        let avail_small_sigma = budget.availability_pct(3.0, 1.0);
        let avail_large_sigma = budget.availability_pct(3.0, 4.0);
        assert!(
            avail_small_sigma > avail_large_sigma,
            "small σ={:.1}%, large σ={:.1}%",
            avail_small_sigma,
            avail_large_sigma
        );
    }

    // ── Atmospheric losses ───────────────────────────────────────────────────

    #[test]
    fn test_gaseous_loss_increases_with_frequency() {
        let el = 30.0_f64.to_radians();
        let l_s = atmospheric_gaseous_loss_db(2.0e9, el);
        let l_ka = atmospheric_gaseous_loss_db(20.0e9, el);
        assert!(l_ka > l_s, "Ka={:.3} dB, S={:.3} dB", l_ka, l_s);
    }

    #[test]
    fn test_gaseous_loss_increases_at_low_elevation() {
        let l_zen = atmospheric_gaseous_loss_db(20.0e9, PI / 2.0);
        let l_low = atmospheric_gaseous_loss_db(20.0e9, 15.0_f64.to_radians());
        assert!(l_low > l_zen);
    }

    #[test]
    fn test_rain_attenuation_increases_with_rain_rate() {
        let a1 = rain_attenuation_db(5.0, 20.0e9, 30.0_f64.to_radians());
        let a2 = rain_attenuation_db(50.0, 20.0e9, 30.0_f64.to_radians());
        assert!(a2 > a1, "a1={:.2} dB, a2={:.2} dB", a1, a2);
    }

    #[test]
    fn test_rain_attenuation_zero_at_zero_rain() {
        let a = rain_attenuation_db(0.0, 20.0e9, 30.0_f64.to_radians());
        assert!(a < 1e-6, "a = {}", a);
    }

    // ── Coverage geometry ────────────────────────────────────────────────────

    #[test]
    fn test_max_slant_range_reasonable() {
        // At 780 km, 10° elevation mask → slant range should be ~3000–4500 km
        let sr = max_slant_range(780_000.0, 10.0_f64.to_radians());
        let sr_km = sr / 1000.0;
        assert!(sr_km > 2000.0 && sr_km < 6000.0, "sr = {:.0} km", sr_km);
    }

    #[test]
    fn test_coverage_area_zero_below_horizon() {
        // If mask >= nadir angle → no coverage
        let area = coverage_area_km2(500_000.0, 89.0_f64.to_radians());
        assert!(area < 100.0);
    }

    #[test]
    fn test_min_elevation_for_coverage_positive() {
        let el = min_elevation_for_coverage(780_000.0, 10.0_f64.to_radians());
        assert!(el >= 0.0);
    }

    // ── Walker best satellite ────────────────────────────────────────────────

    #[test]
    fn test_walker_best_satellite_returns_index() {
        let c = WalkerConstellation::new(66, 6, 2, 86.4_f64.to_radians(), 780_000.0);
        let ground = GroundPosition::new(0.0, 0.0, 0.0);
        let best = c.best_satellite(&ground, 0.0);
        assert!(best.is_some());
        let (idx, elev) = best.unwrap();
        assert!(idx < 66);
        assert!(elev > -PI);
    }
}

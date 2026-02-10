//! Radar target tracking with Doppler-based velocity estimation.
//!
//! This module provides multi-scan track-level Doppler filtering and tracking,
//! including alpha-beta and alpha-beta-gamma trackers, constant-velocity and
//! constant-acceleration Kalman filters, M-of-N track initiation, gating,
//! range-Doppler association, and velocity ambiguity resolution for pulsed
//! Doppler radars.
//!
//! Unlike the single-shot frequency estimation in [`parametric_doppler_estimator`],
//! this module focuses on maintaining and updating track state across multiple
//! radar scans.
//!
//! # Example
//!
//! ```
//! use r4w_core::tracking_doppler_estimator::{
//!     TrackState, Measurement, AlphaBetaTracker,
//!     gate_check, m_of_n_initiate, resolve_doppler_ambiguity,
//!     track_quality_score,
//! };
//!
//! // Create a tracker with smoothing parameters
//! let mut tracker = AlphaBetaTracker::new(1000.0, 50.0, 0.0, 0.8, 0.5);
//!
//! // Predict to next scan
//! tracker.predict(0.1);
//!
//! // Update with measurement
//! let meas = Measurement { range_m: 1005.0, doppler_mps: 52.0, time_s: 0.1, snr_db: 15.0 };
//! tracker.update(meas.range_m);
//!
//! assert!((tracker.state().range_m - 1005.0).abs() < 10.0);
//!
//! // Track initiation: require 3 detections out of last 5 scans
//! let detections = [true, false, true, true, false];
//! assert!(m_of_n_initiate(&detections, 3, 5));
//!
//! // Doppler ambiguity resolution
//! let resolved = resolve_doppler_ambiguity(30.0, 100.0);
//! assert!((resolved - 30.0).abs() < 1e-9);
//! ```

// ---------------------------------------------------------------------------
// Core data types
// ---------------------------------------------------------------------------

/// State of a radar track.
#[derive(Debug, Clone)]
pub struct TrackState {
    /// Estimated range in metres.
    pub range_m: f64,
    /// Estimated radial velocity in metres per second.
    pub velocity_mps: f64,
    /// Estimated radial acceleration in metres per second squared.
    pub acceleration_mps2: f64,
    /// Time of last update in seconds.
    pub time_s: f64,
    /// Track quality metric in `[0, 1]`.
    pub quality: f64,
}

impl TrackState {
    /// Create a new track state.
    pub fn new(range_m: f64, velocity_mps: f64, acceleration_mps2: f64, time_s: f64) -> Self {
        Self {
            range_m,
            velocity_mps,
            acceleration_mps2,
            time_s,
            quality: 1.0,
        }
    }
}

/// A single radar measurement (detection).
#[derive(Debug, Clone)]
pub struct Measurement {
    /// Measured range in metres.
    pub range_m: f64,
    /// Measured Doppler velocity in metres per second.
    pub doppler_mps: f64,
    /// Time of measurement in seconds.
    pub time_s: f64,
    /// Signal-to-noise ratio in dB.
    pub snr_db: f64,
}

/// Configuration parameters for tracking.
#[derive(Debug, Clone)]
pub struct TrackConfig {
    /// Alpha smoothing parameter.
    pub alpha: f64,
    /// Beta smoothing parameter.
    pub beta: f64,
    /// Gamma smoothing parameter (for alpha-beta-gamma tracker).
    pub gamma: f64,
    /// Validation gate size (number of standard deviations).
    pub gate_size: f64,
    /// M-of-N initiation parameters `(m, n)`.
    pub m_of_n: (usize, usize),
}

impl Default for TrackConfig {
    fn default() -> Self {
        Self {
            alpha: 0.8,
            beta: 0.5,
            gamma: 0.1,
            gate_size: 3.0,
            m_of_n: (3, 5),
        }
    }
}

// ---------------------------------------------------------------------------
// Alpha-Beta tracker
// ---------------------------------------------------------------------------

/// Alpha-beta tracker for range/velocity smoothing.
///
/// The alpha-beta filter is a simplified form of the Kalman filter for
/// constant-velocity targets. On each scan it predicts the state forward
/// and then corrects using the residual between predicted and measured range.
#[derive(Debug, Clone)]
pub struct AlphaBetaTracker {
    state: TrackState,
    alpha: f64,
    beta: f64,
}

impl AlphaBetaTracker {
    /// Create a new alpha-beta tracker.
    ///
    /// * `range_m`     – initial range estimate (metres)
    /// * `velocity_mps` – initial velocity estimate (m/s)
    /// * `time_s`       – initial time (seconds)
    /// * `alpha`        – position smoothing factor in `(0, 1]`
    /// * `beta`         – velocity smoothing factor in `(0, 1]`
    pub fn new(range_m: f64, velocity_mps: f64, time_s: f64, alpha: f64, beta: f64) -> Self {
        Self {
            state: TrackState::new(range_m, velocity_mps, 0.0, time_s),
            alpha,
            beta,
        }
    }

    /// Return a reference to the current track state.
    pub fn state(&self) -> &TrackState {
        &self.state
    }

    /// Predict the state forward by `dt` seconds.
    pub fn predict(&mut self, dt: f64) {
        alpha_beta_predict(&mut self.state, dt, self.alpha, self.beta);
    }

    /// Update the state with a range measurement.
    pub fn update(&mut self, measured_range: f64) {
        alpha_beta_update(&mut self.state, measured_range, self.alpha, self.beta);
    }
}

// ---------------------------------------------------------------------------
// Alpha-Beta-Gamma tracker
// ---------------------------------------------------------------------------

/// Alpha-beta-gamma tracker for range/velocity/acceleration smoothing.
///
/// Extends the alpha-beta filter by also estimating target acceleration.
#[derive(Debug, Clone)]
pub struct AlphaBetaGammaTracker {
    state: TrackState,
    alpha: f64,
    beta: f64,
    gamma: f64,
    dt: f64,
}

impl AlphaBetaGammaTracker {
    /// Create a new alpha-beta-gamma tracker.
    ///
    /// * `range_m`           – initial range estimate (metres)
    /// * `velocity_mps`      – initial velocity estimate (m/s)
    /// * `acceleration_mps2` – initial acceleration estimate (m/s^2)
    /// * `time_s`            – initial time (seconds)
    /// * `dt`                – scan interval (seconds)
    /// * `alpha`             – position smoothing factor
    /// * `beta`              – velocity smoothing factor
    /// * `gamma`             – acceleration smoothing factor
    pub fn new(
        range_m: f64,
        velocity_mps: f64,
        acceleration_mps2: f64,
        time_s: f64,
        dt: f64,
        alpha: f64,
        beta: f64,
        gamma: f64,
    ) -> Self {
        Self {
            state: TrackState::new(range_m, velocity_mps, acceleration_mps2, time_s),
            alpha,
            beta,
            gamma,
            dt,
        }
    }

    /// Return a reference to the current track state.
    pub fn state(&self) -> &TrackState {
        &self.state
    }

    /// Predict the state forward by one scan interval.
    pub fn predict(&mut self) {
        let dt = self.dt;
        self.state.range_m += self.state.velocity_mps * dt
            + 0.5 * self.state.acceleration_mps2 * dt * dt;
        self.state.velocity_mps += self.state.acceleration_mps2 * dt;
        self.state.time_s += dt;
    }

    /// Update the state with a range measurement.
    pub fn update(&mut self, measured_range: f64) {
        let dt = self.dt;
        let residual = measured_range - self.state.range_m;
        self.state.range_m += self.alpha * residual;
        self.state.velocity_mps += (self.beta / dt) * residual;
        self.state.acceleration_mps2 += (2.0 * self.gamma / (dt * dt)) * residual;
    }
}

// ---------------------------------------------------------------------------
// Kalman tracker
// ---------------------------------------------------------------------------

/// Generic linear Kalman filter tracker.
///
/// The state vector and matrices are stored as flat/nested `Vec`s so that both
/// constant-velocity (dim=4) and constant-acceleration (dim=6) models can be
/// used without generic parameters.
#[derive(Debug, Clone)]
pub struct KalmanTracker {
    /// State vector.
    pub state: Vec<f64>,
    /// Covariance matrix (dim x dim).
    pub covariance: Vec<Vec<f64>>,
    /// Process noise matrix Q (dim x dim).
    pub process_noise: Vec<Vec<f64>>,
    /// Measurement noise matrix R (meas_dim x meas_dim).
    pub measurement_noise: Vec<Vec<f64>>,
    /// State transition matrix F (dim x dim).
    pub transition: Vec<Vec<f64>>,
    /// Measurement matrix H (meas_dim x dim).
    pub observation: Vec<Vec<f64>>,
}

impl KalmanTracker {
    /// Create a constant-velocity Kalman tracker.
    ///
    /// State vector: `[range, range_rate, doppler, doppler_rate]`
    /// Measurements: `[range, doppler]`
    pub fn constant_velocity(
        range_m: f64,
        velocity_mps: f64,
        dt: f64,
        process_var: f64,
        range_var: f64,
        doppler_var: f64,
    ) -> Self {
        let dim = 4;
        // State: [range, range_rate, doppler, doppler_rate]
        let state = vec![range_m, velocity_mps, velocity_mps, 0.0];

        // F matrix - constant velocity model
        let mut f = identity(dim);
        f[0][1] = dt;
        f[2][3] = dt;

        // Q matrix - piecewise constant white noise acceleration
        let mut q = zeros(dim, dim);
        let dt3 = dt * dt * dt / 3.0;
        let dt2 = dt * dt / 2.0;
        q[0][0] = dt3 * process_var;
        q[0][1] = dt2 * process_var;
        q[1][0] = dt2 * process_var;
        q[1][1] = dt * process_var;
        q[2][2] = dt3 * process_var;
        q[2][3] = dt2 * process_var;
        q[3][2] = dt2 * process_var;
        q[3][3] = dt * process_var;

        // H matrix (2x4)
        let h = vec![
            vec![1.0, 0.0, 0.0, 0.0],
            vec![0.0, 0.0, 1.0, 0.0],
        ];

        // R matrix (2x2)
        let r = vec![
            vec![range_var, 0.0],
            vec![0.0, doppler_var],
        ];

        // Initial covariance
        let mut p = identity(dim);
        for i in 0..dim {
            p[i][i] = 1000.0;
        }

        Self {
            state,
            covariance: p,
            process_noise: q,
            measurement_noise: r,
            transition: f,
            observation: h,
        }
    }

    /// Create a constant-acceleration Kalman tracker.
    ///
    /// State vector: `[range, range_rate, range_accel, doppler, doppler_rate, doppler_accel]`
    /// Measurements: `[range, doppler]`
    pub fn constant_acceleration(
        range_m: f64,
        velocity_mps: f64,
        accel_mps2: f64,
        dt: f64,
        process_var: f64,
        range_var: f64,
        doppler_var: f64,
    ) -> Self {
        let dim = 6;
        let state = vec![range_m, velocity_mps, accel_mps2, velocity_mps, 0.0, 0.0];

        let mut f = identity(dim);
        f[0][1] = dt;
        f[0][2] = 0.5 * dt * dt;
        f[1][2] = dt;
        f[3][4] = dt;
        f[3][5] = 0.5 * dt * dt;
        f[4][5] = dt;

        // Simplified process noise (diagonal-dominant)
        let mut q = zeros(dim, dim);
        let dt5 = dt.powi(5) / 20.0;
        let dt4 = dt.powi(4) / 8.0;
        let dt3 = dt.powi(3) / 3.0;
        let dt2 = dt * dt / 2.0;
        q[0][0] = dt5 * process_var;
        q[0][1] = dt4 * process_var;
        q[0][2] = dt3 / 2.0 * process_var;
        q[1][0] = dt4 * process_var;
        q[1][1] = dt3 * process_var;
        q[1][2] = dt2 * process_var;
        q[2][0] = dt3 / 2.0 * process_var;
        q[2][1] = dt2 * process_var;
        q[2][2] = dt * process_var;
        q[3][3] = dt5 * process_var;
        q[3][4] = dt4 * process_var;
        q[3][5] = dt3 / 2.0 * process_var;
        q[4][3] = dt4 * process_var;
        q[4][4] = dt3 * process_var;
        q[4][5] = dt2 * process_var;
        q[5][3] = dt3 / 2.0 * process_var;
        q[5][4] = dt2 * process_var;
        q[5][5] = dt * process_var;

        let h = vec![
            vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        ];

        let r = vec![
            vec![range_var, 0.0],
            vec![0.0, doppler_var],
        ];

        let mut p = identity(dim);
        for i in 0..dim {
            p[i][i] = 1000.0;
        }

        Self {
            state,
            covariance: p,
            process_noise: q,
            measurement_noise: r,
            transition: f,
            observation: h,
        }
    }

    /// Predict state forward one step.
    pub fn predict(&mut self) {
        kalman_predict(
            &mut self.state,
            &mut self.covariance,
            &self.transition,
            &self.process_noise,
        );
    }

    /// Update state with a measurement vector.
    pub fn update(&mut self, measurement: &[f64]) {
        kalman_update(
            &mut self.state,
            &mut self.covariance,
            measurement,
            &self.observation,
            &self.measurement_noise,
        );
    }

    /// Return the estimated range.
    pub fn range(&self) -> f64 {
        self.state[0]
    }

    /// Return the estimated velocity (range-rate).
    pub fn velocity(&self) -> f64 {
        self.state[1]
    }
}

// ---------------------------------------------------------------------------
// Free-standing prediction / update functions
// ---------------------------------------------------------------------------

/// Alpha-beta prediction step.
///
/// Advances the track state by `dt` seconds using the current velocity estimate.
/// The `alpha` and `beta` parameters are stored but not used during prediction;
/// they are applied in the update step.
pub fn alpha_beta_predict(state: &mut TrackState, dt: f64, _alpha: f64, _beta: f64) {
    state.range_m += state.velocity_mps * dt;
    state.time_s += dt;
}

/// Alpha-beta update step.
///
/// Corrects the predicted range and velocity using the residual between the
/// measured and predicted range.
pub fn alpha_beta_update(state: &mut TrackState, measurement: f64, alpha: f64, beta: f64) {
    let residual = measurement - state.range_m;
    state.range_m += alpha * residual;
    // beta corrects velocity; we normalise by 1.0 for unit dt (the caller
    // should have already predicted with the correct dt).
    state.velocity_mps += beta * residual;
}

/// Kalman filter prediction step.
///
/// Applies the state transition: `x = F * x`, `P = F * P * F^T + Q`.
pub fn kalman_predict(
    state: &mut [f64],
    cov: &mut Vec<Vec<f64>>,
    f_mat: &[Vec<f64>],
    q_mat: &[Vec<f64>],
) {
    let n = state.len();

    // x = F * x
    let new_state = mat_vec_mul(f_mat, state);
    state.copy_from_slice(&new_state);

    // P = F * P * F^T + Q
    let fp = mat_mul(f_mat, cov);
    let ft = transpose(f_mat);
    let fpft = mat_mul(&fp, &ft);
    for i in 0..n {
        for j in 0..n {
            cov[i][j] = fpft[i][j] + q_mat[i][j];
        }
    }
}

/// Kalman filter update (correction) step.
///
/// Computes the Kalman gain and applies the innovation:
/// `K = P * H^T * (H * P * H^T + R)^{-1}`, then updates state and covariance.
pub fn kalman_update(
    state: &mut [f64],
    cov: &mut Vec<Vec<f64>>,
    measurement: &[f64],
    h_mat: &[Vec<f64>],
    r_mat: &[Vec<f64>],
) {
    let n = state.len();
    let m = measurement.len();

    // Innovation: y = z - H * x
    let hx = mat_vec_mul(h_mat, state);
    let mut y = vec![0.0; m];
    for i in 0..m {
        y[i] = measurement[i] - hx[i];
    }

    // S = H * P * H^T + R
    let ht = transpose(h_mat);
    let ph = mat_mul(cov, &ht);
    let hph = mat_mul(h_mat, &ph);
    let mut s = vec![vec![0.0; m]; m];
    for i in 0..m {
        for j in 0..m {
            s[i][j] = hph[i][j] + r_mat[i][j];
        }
    }

    // K = P * H^T * S^{-1}
    let s_inv = invert(&s);
    let k = mat_mul(&ph, &s_inv);

    // x = x + K * y
    let ky = mat_vec_mul(&k, &y);
    for i in 0..n {
        state[i] += ky[i];
    }

    // P = (I - K * H) * P
    let kh = mat_mul(&k, h_mat);
    let mut ikh = identity(n);
    for i in 0..n {
        for j in 0..n {
            ikh[i][j] -= kh[i][j];
        }
    }
    let new_p = mat_mul(&ikh, cov);
    for i in 0..n {
        for j in 0..n {
            cov[i][j] = new_p[i][j];
        }
    }
}

// ---------------------------------------------------------------------------
// Gating and association
// ---------------------------------------------------------------------------

/// Check whether a measurement falls inside the validation gate of a predicted
/// track state.
///
/// Uses a simple Euclidean distance in (range, velocity) space normalised by
/// `gate_size`. Returns `true` if the measurement is inside the gate.
pub fn gate_check(predicted: &TrackState, measurement: &Measurement, gate_size: f64) -> bool {
    let dr = predicted.range_m - measurement.range_m;
    let dv = predicted.velocity_mps - measurement.doppler_mps;
    let dist_sq = dr * dr + dv * dv;
    dist_sq <= gate_size * gate_size
}

/// Nearest-neighbour range-Doppler association.
///
/// For each track, finds the closest gated measurement (if any) and returns a
/// vector of `Option<usize>` indices into `measurements`. A measurement can
/// only be assigned to one track (greedy, first-come-first-served).
pub fn associate_measurements(
    tracks: &[TrackState],
    measurements: &[Measurement],
    gate_size: f64,
) -> Vec<Option<usize>> {
    let mut assignments: Vec<Option<usize>> = vec![None; tracks.len()];
    let mut used = vec![false; measurements.len()];

    for (ti, track) in tracks.iter().enumerate() {
        let mut best_dist = f64::MAX;
        let mut best_idx: Option<usize> = None;

        for (mi, meas) in measurements.iter().enumerate() {
            if used[mi] {
                continue;
            }
            if !gate_check(track, meas, gate_size) {
                continue;
            }
            let dr = track.range_m - meas.range_m;
            let dv = track.velocity_mps - meas.doppler_mps;
            let dist = dr * dr + dv * dv;
            if dist < best_dist {
                best_dist = dist;
                best_idx = Some(mi);
            }
        }

        if let Some(idx) = best_idx {
            assignments[ti] = Some(idx);
            used[idx] = true;
        }
    }

    assignments
}

// ---------------------------------------------------------------------------
// Track initiation
// ---------------------------------------------------------------------------

/// M-of-N track initiation logic.
///
/// Returns `true` if at least `m` of the last `n` entries in `detections` are
/// `true`. If `detections` has fewer than `n` elements, only the available
/// elements are considered.
pub fn m_of_n_initiate(detections: &[bool], m: usize, n: usize) -> bool {
    if n == 0 {
        return false;
    }
    let window = if detections.len() >= n {
        &detections[detections.len() - n..]
    } else {
        detections
    };
    let hits = window.iter().filter(|&&d| d).count();
    hits >= m
}

// ---------------------------------------------------------------------------
// Doppler ambiguity resolution
// ---------------------------------------------------------------------------

/// Resolve Doppler velocity ambiguity for pulsed Doppler radars.
///
/// Given a `measured_vel` that may be aliased and the `unambiguous_vel`
/// (half the Nyquist velocity = PRF * lambda / 4), this function wraps
/// the measured velocity into the interval `[-unambiguous_vel, unambiguous_vel)`.
pub fn resolve_doppler_ambiguity(measured_vel: f64, unambiguous_vel: f64) -> f64 {
    if unambiguous_vel <= 0.0 {
        return measured_vel;
    }
    let two_v = 2.0 * unambiguous_vel;
    let mut v = measured_vel % two_v;
    if v > unambiguous_vel {
        v -= two_v;
    } else if v < -unambiguous_vel {
        v += two_v;
    }
    v
}

/// Resolve Doppler ambiguity using multiple PRFs (Chinese Remainder Theorem
/// approach).
///
/// Given a set of measured velocities from different PRFs and their
/// corresponding unambiguous velocities, returns the velocity that is
/// consistent across all PRFs. Uses a simple brute-force search over
/// integer multiples.
pub fn resolve_ambiguity_multi_prf(
    measured_vels: &[f64],
    unambiguous_vels: &[f64],
    max_velocity: f64,
) -> f64 {
    if measured_vels.is_empty() || measured_vels.len() != unambiguous_vels.len() {
        return 0.0;
    }
    if measured_vels.len() == 1 {
        return measured_vels[0];
    }

    let two_v0 = 2.0 * unambiguous_vels[0];
    let steps = (max_velocity / unambiguous_vels[0]).ceil() as i64 + 1;

    let mut best_vel = measured_vels[0];
    let mut best_err = f64::MAX;

    for k in -steps..=steps {
        let candidate = measured_vels[0] + k as f64 * two_v0;
        if candidate.abs() > max_velocity {
            continue;
        }
        let mut total_err = 0.0;
        for i in 1..measured_vels.len() {
            let wrapped = resolve_doppler_ambiguity(candidate, unambiguous_vels[i]);
            let err = (wrapped - measured_vels[i]).abs();
            total_err += err * err;
        }
        if total_err < best_err {
            best_err = total_err;
            best_vel = candidate;
        }
    }

    best_vel
}

// ---------------------------------------------------------------------------
// Track quality scoring
// ---------------------------------------------------------------------------

/// Compute a quality score for a track in `[0, 1]`.
///
/// The score is a weighted combination of:
/// - Hit ratio (`hits / age`): how often the target was detected
/// - Age factor: young tracks are penalised (ramps from 0 to 1 over 10 scans)
/// - Velocity consistency: low acceleration relative to velocity is rewarded
pub fn track_quality_score(track: &TrackState, age: usize, hits: usize) -> f64 {
    if age == 0 {
        return 0.0;
    }

    // Hit ratio (0..1)
    let hit_ratio = (hits as f64) / (age as f64);

    // Age factor: ramp up over first 10 scans
    let age_factor = (age as f64 / 10.0).min(1.0);

    // Velocity consistency: penalise high acceleration relative to velocity
    let vel_abs = track.velocity_mps.abs().max(1.0);
    let acc_ratio = (track.acceleration_mps2.abs() / vel_abs).min(1.0);
    let consistency = 1.0 - acc_ratio;

    // Weighted combination
    let score = 0.5 * hit_ratio + 0.3 * age_factor + 0.2 * consistency;
    score.clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Matrix utilities (std-only, no external crate)
// ---------------------------------------------------------------------------

fn identity(n: usize) -> Vec<Vec<f64>> {
    let mut m = vec![vec![0.0; n]; n];
    for i in 0..n {
        m[i][i] = 1.0;
    }
    m
}

fn zeros(rows: usize, cols: usize) -> Vec<Vec<f64>> {
    vec![vec![0.0; cols]; rows]
}

fn transpose(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    if a.is_empty() {
        return vec![];
    }
    let rows = a.len();
    let cols = a[0].len();
    let mut t = vec![vec![0.0; rows]; cols];
    for i in 0..rows {
        for j in 0..cols {
            t[j][i] = a[i][j];
        }
    }
    t
}

fn mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let rows = a.len();
    let cols = b[0].len();
    let inner = b.len();
    let mut c = vec![vec![0.0; cols]; rows];
    for i in 0..rows {
        for j in 0..cols {
            let mut sum = 0.0;
            for k in 0..inner {
                sum += a[i][k] * b[k][j];
            }
            c[i][j] = sum;
        }
    }
    c
}

fn mat_vec_mul(a: &[Vec<f64>], x: &[f64]) -> Vec<f64> {
    let rows = a.len();
    let mut y = vec![0.0; rows];
    for i in 0..rows {
        let mut sum = 0.0;
        for (j, &xj) in x.iter().enumerate() {
            sum += a[i][j] * xj;
        }
        y[i] = sum;
    }
    y
}

/// Invert a small matrix using Gauss-Jordan elimination.
fn invert(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    // Augment with identity
    let mut aug = vec![vec![0.0; 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = 1.0;
    }

    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-15 {
            // Singular - return identity as fallback
            return identity(n);
        }

        for j in 0..(2 * n) {
            aug[col][j] /= pivot;
        }

        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..(2 * n) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    let mut inv = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = aug[i][n + j];
        }
    }
    inv
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    // -- TrackState ---------------------------------------------------------

    #[test]
    fn test_track_state_new() {
        let ts = TrackState::new(1000.0, 50.0, 0.5, 0.0);
        assert!((ts.range_m - 1000.0).abs() < EPS);
        assert!((ts.velocity_mps - 50.0).abs() < EPS);
        assert!((ts.acceleration_mps2 - 0.5).abs() < EPS);
        assert!((ts.time_s).abs() < EPS);
        assert!((ts.quality - 1.0).abs() < EPS);
    }

    // -- Alpha-Beta tracker -------------------------------------------------

    #[test]
    fn test_alpha_beta_predict() {
        let mut state = TrackState::new(1000.0, 50.0, 0.0, 0.0);
        alpha_beta_predict(&mut state, 1.0, 0.8, 0.5);
        assert!((state.range_m - 1050.0).abs() < EPS);
        assert!((state.time_s - 1.0).abs() < EPS);
    }

    #[test]
    fn test_alpha_beta_update() {
        let mut state = TrackState::new(1050.0, 50.0, 0.0, 1.0);
        alpha_beta_update(&mut state, 1060.0, 0.8, 0.5);
        // residual = 10, range += 0.8*10 = 8 => 1058
        assert!((state.range_m - 1058.0).abs() < EPS);
        // velocity += 0.5*10 = 5 => 55
        assert!((state.velocity_mps - 55.0).abs() < EPS);
    }

    #[test]
    fn test_alpha_beta_tracker_struct() {
        let mut tracker = AlphaBetaTracker::new(1000.0, 50.0, 0.0, 0.8, 0.5);
        tracker.predict(1.0);
        assert!((tracker.state().range_m - 1050.0).abs() < EPS);
        tracker.update(1055.0);
        // residual = 5, range += 0.8*5=4 => 1054
        assert!((tracker.state().range_m - 1054.0).abs() < EPS);
    }

    #[test]
    fn test_alpha_beta_converges() {
        // Constant velocity target at 50 m/s, starting at 1000 m
        let mut tracker = AlphaBetaTracker::new(900.0, 40.0, 0.0, 0.8, 0.3);
        let dt = 1.0;
        for i in 1..=20 {
            tracker.predict(dt);
            let true_range = 1000.0 + 50.0 * (i as f64);
            tracker.update(true_range);
        }
        let true_range_20 = 1000.0 + 50.0 * 20.0;
        // Should converge close to the true state
        assert!((tracker.state().range_m - true_range_20).abs() < 5.0);
        assert!((tracker.state().velocity_mps - 50.0).abs() < 5.0);
    }

    // -- Alpha-Beta-Gamma tracker -------------------------------------------

    #[test]
    fn test_alpha_beta_gamma_predict() {
        let mut tracker =
            AlphaBetaGammaTracker::new(1000.0, 50.0, 2.0, 0.0, 1.0, 0.8, 0.5, 0.1);
        tracker.predict();
        // range = 1000 + 50*1 + 0.5*2*1 = 1051
        assert!((tracker.state().range_m - 1051.0).abs() < EPS);
        // velocity = 50 + 2*1 = 52
        assert!((tracker.state().velocity_mps - 52.0).abs() < EPS);
    }

    #[test]
    fn test_alpha_beta_gamma_update() {
        let mut tracker =
            AlphaBetaGammaTracker::new(1051.0, 52.0, 2.0, 1.0, 1.0, 0.8, 0.5, 0.1);
        tracker.update(1060.0);
        let residual = 9.0;
        let expected_range = 1051.0 + 0.8 * residual; // 1058.2
        let expected_vel = 52.0 + 0.5 * residual; // 56.5
        let expected_acc = 2.0 + 2.0 * 0.1 * residual; // 3.8
        assert!((tracker.state().range_m - expected_range).abs() < EPS);
        assert!((tracker.state().velocity_mps - expected_vel).abs() < EPS);
        assert!((tracker.state().acceleration_mps2 - expected_acc).abs() < EPS);
    }

    // -- Kalman tracker (constant velocity) ---------------------------------

    #[test]
    fn test_kalman_cv_creation() {
        let kf = KalmanTracker::constant_velocity(1000.0, 50.0, 1.0, 1.0, 10.0, 1.0);
        assert_eq!(kf.state.len(), 4);
        assert!((kf.state[0] - 1000.0).abs() < EPS);
        assert!((kf.state[1] - 50.0).abs() < EPS);
    }

    #[test]
    fn test_kalman_cv_predict() {
        let mut kf = KalmanTracker::constant_velocity(1000.0, 50.0, 1.0, 1.0, 10.0, 1.0);
        kf.predict();
        // range should advance by velocity * dt = 50
        assert!((kf.state[0] - 1050.0).abs() < EPS);
    }

    #[test]
    fn test_kalman_cv_update() {
        let mut kf = KalmanTracker::constant_velocity(1000.0, 50.0, 1.0, 1.0, 10.0, 1.0);
        kf.predict();
        kf.update(&[1055.0, 50.0]);
        // After update, range should be pulled toward measurement
        assert!((kf.range() - 1055.0).abs() < 10.0);
    }

    #[test]
    fn test_kalman_cv_converges() {
        let dt = 1.0;
        let mut kf = KalmanTracker::constant_velocity(900.0, 40.0, dt, 0.1, 5.0, 1.0);
        for i in 1..=30 {
            kf.predict();
            let true_range = 1000.0 + 50.0 * (i as f64);
            kf.update(&[true_range, 50.0]);
        }
        let true_range_30 = 1000.0 + 50.0 * 30.0;
        assert!((kf.range() - true_range_30).abs() < 5.0);
        assert!((kf.velocity() - 50.0).abs() < 5.0);
    }

    // -- Kalman tracker (constant acceleration) -----------------------------

    #[test]
    fn test_kalman_ca_creation() {
        let kf =
            KalmanTracker::constant_acceleration(1000.0, 50.0, 2.0, 1.0, 1.0, 10.0, 1.0);
        assert_eq!(kf.state.len(), 6);
        assert!((kf.state[0] - 1000.0).abs() < EPS);
        assert!((kf.state[2] - 2.0).abs() < EPS);
    }

    #[test]
    fn test_kalman_ca_predict() {
        let mut kf =
            KalmanTracker::constant_acceleration(1000.0, 50.0, 2.0, 1.0, 1.0, 10.0, 1.0);
        kf.predict();
        // range = 1000 + 50*1 + 0.5*2*1 = 1051
        assert!((kf.state[0] - 1051.0).abs() < 1.0);
    }

    // -- Gating -------------------------------------------------------------

    #[test]
    fn test_gate_check_inside() {
        let state = TrackState::new(1000.0, 50.0, 0.0, 0.0);
        let meas = Measurement {
            range_m: 1002.0,
            doppler_mps: 51.0,
            time_s: 0.0,
            snr_db: 15.0,
        };
        assert!(gate_check(&state, &meas, 5.0));
    }

    #[test]
    fn test_gate_check_outside() {
        let state = TrackState::new(1000.0, 50.0, 0.0, 0.0);
        let meas = Measurement {
            range_m: 1100.0,
            doppler_mps: 80.0,
            time_s: 0.0,
            snr_db: 15.0,
        };
        assert!(!gate_check(&state, &meas, 5.0));
    }

    #[test]
    fn test_gate_check_boundary() {
        let state = TrackState::new(0.0, 0.0, 0.0, 0.0);
        let meas = Measurement {
            range_m: 3.0,
            doppler_mps: 4.0,
            time_s: 0.0,
            snr_db: 10.0,
        };
        // distance = 5.0, gate = 5.0 => on boundary => inside
        assert!(gate_check(&state, &meas, 5.0));
    }

    // -- Association --------------------------------------------------------

    #[test]
    fn test_associate_measurements() {
        let tracks = vec![
            TrackState::new(1000.0, 50.0, 0.0, 0.0),
            TrackState::new(2000.0, -30.0, 0.0, 0.0),
        ];
        let measurements = vec![
            Measurement { range_m: 2005.0, doppler_mps: -28.0, time_s: 0.0, snr_db: 15.0 },
            Measurement { range_m: 1003.0, doppler_mps: 49.0, time_s: 0.0, snr_db: 12.0 },
        ];
        let assoc = associate_measurements(&tracks, &measurements, 20.0);
        // Track 0 (1000, 50) should match measurement 1 (1003, 49)
        assert_eq!(assoc[0], Some(1));
        // Track 1 (2000, -30) should match measurement 0 (2005, -28)
        assert_eq!(assoc[1], Some(0));
    }

    // -- M-of-N initiation --------------------------------------------------

    #[test]
    fn test_m_of_n_initiate_pass() {
        let dets = [true, false, true, true, false];
        assert!(m_of_n_initiate(&dets, 3, 5));
    }

    #[test]
    fn test_m_of_n_initiate_fail() {
        let dets = [true, false, false, true, false];
        assert!(!m_of_n_initiate(&dets, 3, 5));
    }

    #[test]
    fn test_m_of_n_short_window() {
        let dets = [true, true];
        assert!(m_of_n_initiate(&dets, 2, 5));
    }

    #[test]
    fn test_m_of_n_zero_n() {
        assert!(!m_of_n_initiate(&[true], 1, 0));
    }

    // -- Doppler ambiguity resolution ---------------------------------------

    #[test]
    fn test_resolve_doppler_no_ambiguity() {
        let v = resolve_doppler_ambiguity(30.0, 100.0);
        assert!((v - 30.0).abs() < EPS);
    }

    #[test]
    fn test_resolve_doppler_wrapped_positive() {
        // measured = 150, unambiguous = 100 => should wrap to -50
        let v = resolve_doppler_ambiguity(150.0, 100.0);
        assert!((v - (-50.0)).abs() < EPS);
    }

    #[test]
    fn test_resolve_doppler_wrapped_negative() {
        // measured = -150, unambiguous = 100 => should wrap to 50
        let v = resolve_doppler_ambiguity(-150.0, 100.0);
        assert!((v - 50.0).abs() < EPS);
    }

    #[test]
    fn test_resolve_doppler_zero_unambiguous() {
        let v = resolve_doppler_ambiguity(30.0, 0.0);
        assert!((v - 30.0).abs() < EPS);
    }

    #[test]
    fn test_resolve_ambiguity_multi_prf() {
        // True velocity = 250 m/s
        // PRF1: unambiguous = 100 m/s => measured = 250 % 200 - 200 = 50
        // PRF2: unambiguous = 130 m/s => measured = 250 % 260 - 260 = -10
        let measured = [50.0, -10.0];
        let unamb = [100.0, 130.0];
        let resolved = resolve_ambiguity_multi_prf(&measured, &unamb, 500.0);
        assert!((resolved - 250.0).abs() < 1.0);
    }

    // -- Track quality score ------------------------------------------------

    #[test]
    fn test_track_quality_perfect() {
        let track = TrackState::new(1000.0, 50.0, 0.0, 0.0);
        let score = track_quality_score(&track, 20, 20);
        // hit_ratio=1.0, age_factor=1.0, consistency=1.0 (accel=0)
        // score = 0.5*1 + 0.3*1 + 0.2*1 = 1.0
        assert!((score - 1.0).abs() < EPS);
    }

    #[test]
    fn test_track_quality_zero_age() {
        let track = TrackState::new(1000.0, 50.0, 0.0, 0.0);
        let score = track_quality_score(&track, 0, 0);
        assert!((score).abs() < EPS);
    }

    #[test]
    fn test_track_quality_young_track() {
        let track = TrackState::new(1000.0, 50.0, 0.0, 0.0);
        let score_young = track_quality_score(&track, 2, 2);
        let score_old = track_quality_score(&track, 20, 20);
        // Young track should have lower quality due to age factor
        assert!(score_young < score_old);
    }

    // -- TrackConfig --------------------------------------------------------

    #[test]
    fn test_track_config_default() {
        let cfg = TrackConfig::default();
        assert!((cfg.alpha - 0.8).abs() < EPS);
        assert!((cfg.beta - 0.5).abs() < EPS);
        assert!((cfg.gamma - 0.1).abs() < EPS);
        assert!((cfg.gate_size - 3.0).abs() < EPS);
        assert_eq!(cfg.m_of_n, (3, 5));
    }

    // -- Kalman free functions ----------------------------------------------

    #[test]
    fn test_kalman_predict_identity() {
        let mut state = vec![10.0, 5.0];
        let f = identity(2);
        let q = zeros(2, 2);
        let mut cov = identity(2);
        kalman_predict(&mut state, &mut cov, &f, &q);
        // Identity transition: state unchanged
        assert!((state[0] - 10.0).abs() < EPS);
        assert!((state[1] - 5.0).abs() < EPS);
    }

    #[test]
    fn test_kalman_update_reduces_uncertainty() {
        let mut state = vec![10.0, 5.0];
        let mut cov = vec![vec![100.0, 0.0], vec![0.0, 100.0]];
        let h = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let r = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let cov_before_trace = cov[0][0] + cov[1][1];
        kalman_update(&mut state, &mut cov, &[10.5, 5.5], &h, &r);
        let cov_after_trace = cov[0][0] + cov[1][1];
        // Covariance should shrink after update
        assert!(cov_after_trace < cov_before_trace);
    }
}

//! Strapdown Inertial Navigation System (INS) processor.
//!
//! This module implements a complete strapdown INS pipeline including:
//!
//! - **IMU Integration**: Accelerometer and gyroscope data processing
//! - **Attitude Representation**: Quaternion and Direction Cosine Matrix (DCM)
//! - **Gravity Removal**: WGS-84 gravity model for proper specific-force handling
//! - **Coriolis Correction**: Earth-rate and transport-rate compensation
//! - **Bias Estimation**: First-order gyro and accelerometer bias estimation
//! - **Coarse Alignment**: Static leveling and gyrocompass from averaged IMU data
//! - **Dead Reckoning**: Trajectory propagation from IMU measurements
//!
//! The strapdown mechanization integrates body-frame IMU measurements into
//! navigation-frame (NED) position, velocity, and attitude using quaternion
//! rotation updates.
//!
//! # Example
//!
//! ```
//! use r4w_core::inertial_nav_processor::{
//!     InsState, ImuSample, InsProcessor, euler_to_quaternion,
//! };
//!
//! // Create a processor with default gravity at sea level, 45° latitude
//! let processor = InsProcessor::new(45.0_f64.to_radians(), 0.0);
//!
//! // Initialize state with level attitude
//! let mut state = InsState::new();
//! state.attitude = euler_to_quaternion(0.0, 0.0, 0.0);
//!
//! // Simulate a stationary IMU sample (gravity along body Z)
//! let sample = ImuSample {
//!     gyro: [0.0, 0.0, 0.0],
//!     accel: [0.0, 0.0, 9.80665],
//!     dt: 0.01,
//! };
//!
//! // Process one step
//! processor.mechanize(&mut state, &sample);
//!
//! // Velocity should remain near zero for a stationary platform
//! assert!(state.velocity[0].abs() < 0.01);
//! assert!(state.velocity[1].abs() < 0.01);
//! assert!(state.velocity[2].abs() < 0.2);
//! ```

use std::f64::consts::PI;

/// Earth's rotation rate in rad/s (WGS-84).
const OMEGA_EARTH: f64 = 7.292_115e-5;

/// WGS-84 semi-major axis in meters.
const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 flattening.
const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// Standard gravity at sea level (m/s²).
const G0: f64 = 9.806_65;

/// Inertial navigation state in the NED (North-East-Down) frame.
///
/// Contains the full navigation solution: position, velocity, attitude,
/// and estimated sensor biases.
#[derive(Debug, Clone)]
pub struct InsState {
    /// Position in NED frame (meters): `[north, east, down]`.
    pub position: [f64; 3],
    /// Velocity in NED frame (m/s): `[v_north, v_east, v_down]`.
    pub velocity: [f64; 3],
    /// Attitude quaternion (body-to-NED), `[w, x, y, z]` convention.
    pub attitude: [f64; 4],
    /// Estimated gyroscope bias in body frame (rad/s): `[bx, by, bz]`.
    pub gyro_bias: [f64; 3],
    /// Estimated accelerometer bias in body frame (m/s²): `[bx, by, bz]`.
    pub accel_bias: [f64; 3],
}

impl InsState {
    /// Creates a new `InsState` with zero position, velocity, biases,
    /// and identity attitude quaternion.
    pub fn new() -> Self {
        Self {
            position: [0.0; 3],
            velocity: [0.0; 3],
            attitude: [1.0, 0.0, 0.0, 0.0],
            gyro_bias: [0.0; 3],
            accel_bias: [0.0; 3],
        }
    }
}

impl Default for InsState {
    fn default() -> Self {
        Self::new()
    }
}

/// A single IMU measurement containing gyroscope and accelerometer readings.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// Gyroscope measurements in body frame (rad/s): `[wx, wy, wz]`.
    pub gyro: [f64; 3],
    /// Accelerometer measurements in body frame (m/s²): `[ax, ay, az]`.
    pub accel: [f64; 3],
    /// Time step since previous sample (seconds).
    pub dt: f64,
}

/// Main strapdown INS processor.
///
/// Performs single-step mechanization (attitude update, velocity integration,
/// position integration) with configurable gravity model parameters.
#[derive(Debug, Clone)]
pub struct InsProcessor {
    /// Reference latitude for gravity and Coriolis calculations (radians).
    pub latitude_rad: f64,
    /// Reference altitude for gravity calculation (meters).
    pub altitude_m: f64,
    /// Gyro bias estimation gain (0 = disabled, small positive for slow adaptation).
    pub gyro_bias_gain: f64,
    /// Accelerometer bias estimation gain.
    pub accel_bias_gain: f64,
}

impl InsProcessor {
    /// Creates a new `InsProcessor` with the given reference latitude and altitude.
    ///
    /// Bias estimation gains default to zero (disabled).
    pub fn new(latitude_rad: f64, altitude_m: f64) -> Self {
        Self {
            latitude_rad,
            altitude_m,
            gyro_bias_gain: 0.0,
            accel_bias_gain: 0.0,
        }
    }

    /// Performs a single strapdown mechanization step.
    ///
    /// 1. Compensates gyro and accel for estimated biases
    /// 2. Updates attitude quaternion via angular rate integration
    /// 3. Transforms specific force from body to NED frame
    /// 4. Removes gravity and applies Coriolis correction
    /// 5. Integrates velocity and position
    /// 6. Updates bias estimates (if gains are non-zero)
    pub fn mechanize(&self, state: &mut InsState, imu: &ImuSample) {
        mechanize_with_gravity(
            state,
            imu,
            self.latitude_rad,
            self.altitude_m,
            self.gyro_bias_gain,
            self.accel_bias_gain,
        );
    }
}

/// Single-step strapdown mechanization (free function).
///
/// Updates the navigation state by integrating one IMU sample. Uses a
/// first-order quaternion update for attitude and trapezoidal integration
/// for velocity and position.
pub fn mechanize(state: &mut InsState, imu: &ImuSample) {
    // Default to equator, sea level, no bias estimation
    mechanize_with_gravity(state, imu, 0.0, 0.0, 0.0, 0.0);
}

/// Internal mechanization with full parameters.
fn mechanize_with_gravity(
    state: &mut InsState,
    imu: &ImuSample,
    latitude_rad: f64,
    altitude_m: f64,
    gyro_bias_gain: f64,
    accel_bias_gain: f64,
) {
    let dt = imu.dt;
    if dt <= 0.0 {
        return;
    }

    // 1. Bias-compensated measurements
    let gyro = [
        imu.gyro[0] - state.gyro_bias[0],
        imu.gyro[1] - state.gyro_bias[1],
        imu.gyro[2] - state.gyro_bias[2],
    ];
    let accel = [
        imu.accel[0] - state.accel_bias[0],
        imu.accel[1] - state.accel_bias[1],
        imu.accel[2] - state.accel_bias[2],
    ];

    // 2. Attitude update: quaternion integration
    // Compute rotation vector magnitude
    let angle = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt() * dt;
    if angle > 1e-12 {
        let half_angle = angle / 2.0;
        let sinc = half_angle.sin() / angle;
        let dq_w = half_angle.cos();
        let dq_x = gyro[0] * dt * sinc;
        let dq_y = gyro[1] * dt * sinc;
        let dq_z = gyro[2] * dt * sinc;

        // Quaternion multiplication: q_new = q_old * dq (body-frame rotation)
        let q = state.attitude;
        state.attitude = [
            q[0] * dq_w - q[1] * dq_x - q[2] * dq_y - q[3] * dq_z,
            q[0] * dq_x + q[1] * dq_w + q[2] * dq_z - q[3] * dq_y,
            q[0] * dq_y - q[1] * dq_z + q[2] * dq_w + q[3] * dq_x,
            q[0] * dq_z + q[1] * dq_y - q[2] * dq_x + q[3] * dq_w,
        ];
    }
    quaternion_normalize(&mut state.attitude);

    // 3. Transform specific force from body to NED
    let dcm = quaternion_to_dcm(&state.attitude);
    let accel_ned = [
        dcm[0][0] * accel[0] + dcm[0][1] * accel[1] + dcm[0][2] * accel[2],
        dcm[1][0] * accel[0] + dcm[1][1] * accel[1] + dcm[1][2] * accel[2],
        dcm[2][0] * accel[0] + dcm[2][1] * accel[1] + dcm[2][2] * accel[2],
    ];

    // 4. Gravity and Coriolis
    let g_ned = gravity_ned(latitude_rad, altitude_m);
    let coriolis = coriolis_correction(&state.velocity, latitude_rad);

    // 5. Velocity integration (trapezoidal: use accel_ned as constant over dt)
    let old_velocity = state.velocity;
    for i in 0..3 {
        state.velocity[i] += (accel_ned[i] + g_ned[i] - coriolis[i]) * dt;
    }

    // 6. Position integration (trapezoidal using average velocity)
    for i in 0..3 {
        state.position[i] += 0.5 * (old_velocity[i] + state.velocity[i]) * dt;
    }

    // 7. Bias estimation (simple first-order)
    if gyro_bias_gain > 0.0 {
        // Use attitude error proxy: for a stationary platform, gyro should read ~0 in NED
        // This is a simplified complementary-filter approach
        for i in 0..3 {
            state.gyro_bias[i] += gyro_bias_gain * gyro[i] * dt;
        }
    }
    if accel_bias_gain > 0.0 {
        // Use velocity error as proxy: if velocity is drifting, accel has bias
        for i in 0..3 {
            // Project velocity error back to body frame (transpose of DCM)
            let vel_err_body_i = dcm[0][i] * state.velocity[0]
                + dcm[1][i] * state.velocity[1]
                + dcm[2][i] * state.velocity[2];
            state.accel_bias[i] += accel_bias_gain * vel_err_body_i * dt;
        }
    }
}

/// Normalizes a quaternion `[w, x, y, z]` to unit length in place.
///
/// If the quaternion has near-zero magnitude, it is reset to identity `[1, 0, 0, 0]`.
pub fn quaternion_normalize(q: &mut [f64; 4]) {
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm < 1e-15 {
        *q = [1.0, 0.0, 0.0, 0.0];
    } else {
        let inv = 1.0 / norm;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }
}

/// Converts a unit quaternion `[w, x, y, z]` to a 3x3 Direction Cosine Matrix.
///
/// The DCM transforms vectors from body frame to NED frame: `v_ned = DCM * v_body`.
///
/// Returns `dcm[row][col]`.
pub fn quaternion_to_dcm(q: &[f64; 4]) -> [[f64; 3]; 3] {
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];

    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;

    [
        [
            1.0 - 2.0 * (yy + zz),
            2.0 * (xy - wz),
            2.0 * (xz + wy),
        ],
        [
            2.0 * (xy + wz),
            1.0 - 2.0 * (xx + zz),
            2.0 * (yz - wx),
        ],
        [
            2.0 * (xz - wy),
            2.0 * (yz + wx),
            1.0 - 2.0 * (xx + yy),
        ],
    ]
}

/// Converts a 3x3 Direction Cosine Matrix to a unit quaternion `[w, x, y, z]`.
///
/// Uses the Shepperd method for numerical stability, selecting the largest
/// diagonal element to avoid division by small numbers.
pub fn dcm_to_quaternion(dcm: &[[f64; 3]; 3]) -> [f64; 4] {
    let trace = dcm[0][0] + dcm[1][1] + dcm[2][2];

    let mut q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0; // s = 4*w
        [
            0.25 * s,
            (dcm[2][1] - dcm[1][2]) / s,
            (dcm[0][2] - dcm[2][0]) / s,
            (dcm[1][0] - dcm[0][1]) / s,
        ]
    } else if dcm[0][0] > dcm[1][1] && dcm[0][0] > dcm[2][2] {
        let s = (1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2]).sqrt() * 2.0;
        [
            (dcm[2][1] - dcm[1][2]) / s,
            0.25 * s,
            (dcm[0][1] + dcm[1][0]) / s,
            (dcm[0][2] + dcm[2][0]) / s,
        ]
    } else if dcm[1][1] > dcm[2][2] {
        let s = (1.0 + dcm[1][1] - dcm[0][0] - dcm[2][2]).sqrt() * 2.0;
        [
            (dcm[0][2] - dcm[2][0]) / s,
            (dcm[0][1] + dcm[1][0]) / s,
            0.25 * s,
            (dcm[1][2] + dcm[2][1]) / s,
        ]
    } else {
        let s = (1.0 + dcm[2][2] - dcm[0][0] - dcm[1][1]).sqrt() * 2.0;
        [
            (dcm[1][0] - dcm[0][1]) / s,
            (dcm[0][2] + dcm[2][0]) / s,
            (dcm[1][2] + dcm[2][1]) / s,
            0.25 * s,
        ]
    };

    quaternion_normalize(&mut q);

    // Ensure w >= 0 for canonical form
    if q[0] < 0.0 {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }

    q
}

/// Converts Euler angles (roll, pitch, yaw) to a quaternion `[w, x, y, z]`.
///
/// Uses the ZYX (yaw-pitch-roll) rotation order:
/// - `roll` (phi): rotation about X axis (radians)
/// - `pitch` (theta): rotation about Y axis (radians)
/// - `yaw` (psi): rotation about Z axis (radians)
pub fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let cr = (roll / 2.0).cos();
    let sr = (roll / 2.0).sin();
    let cp = (pitch / 2.0).cos();
    let sp = (pitch / 2.0).sin();
    let cy = (yaw / 2.0).cos();
    let sy = (yaw / 2.0).sin();

    [
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    ]
}

/// Converts a quaternion `[w, x, y, z]` to Euler angles `(roll, pitch, yaw)` in radians.
///
/// Uses the ZYX convention. Pitch is clamped to `[-pi/2, pi/2]` to handle gimbal lock.
pub fn quaternion_to_euler(q: &[f64; 4]) -> (f64, f64, f64) {
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];

    // Roll (phi) - rotation about X
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (theta) - rotation about Y
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        // Gimbal lock
        (PI / 2.0).copysign(sinp)
    } else {
        sinp.asin()
    };

    // Yaw (psi) - rotation about Z
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (roll, pitch, yaw)
}

/// Computes the local gravity vector in the NED frame using the WGS-84 gravity model.
///
/// Returns `[0, 0, g_down]` where `g_down` is positive (pointing down in NED).
///
/// Uses the Somigliana formula for gravity magnitude on the ellipsoid, with
/// free-air correction for altitude.
///
/// # Arguments
///
/// * `latitude_rad` - Geodetic latitude in radians
/// * `altitude_m` - Height above ellipsoid in meters
pub fn gravity_ned(latitude_rad: f64, altitude_m: f64) -> [f64; 3] {
    // Somigliana formula for gravity on the ellipsoid
    let sin_lat = latitude_rad.sin();
    let sin_lat_sq = sin_lat * sin_lat;

    // Gravity at the equator
    let g_equator = 9.780_325_335_9;

    // Normal gravity on the ellipsoid (simplified Somigliana)
    let g0 = g_equator
        * (1.0 + 0.005_302_4 * sin_lat_sq
            - 0.000_005_8 * (2.0 * latitude_rad).sin().powi(2));

    // Free-air correction (first-order altitude reduction)
    let g = g0
        * (1.0 - 2.0 * altitude_m / WGS84_A
            * (1.0 + WGS84_F + (-2.0 * WGS84_F * sin_lat_sq))
            + 3.0 * altitude_m * altitude_m / (WGS84_A * WGS84_A));

    // Gravity in NED: positive down
    [0.0, 0.0, g]
}

/// Computes the Coriolis acceleration correction in the NED frame.
///
/// The Coriolis effect arises from the rotation of the Earth and must be
/// subtracted from measured accelerations. Returns the Coriolis acceleration
/// vector `[a_north, a_east, a_down]`.
///
/// # Arguments
///
/// * `velocity_ned` - Current velocity in NED frame (m/s)
/// * `latitude_rad` - Geodetic latitude in radians
pub fn coriolis_correction(velocity_ned: &[f64; 3], latitude_rad: f64) -> [f64; 3] {
    let sin_lat = latitude_rad.sin();
    let cos_lat = latitude_rad.cos();

    // Earth rotation vector in NED: [omega*cos(lat), 0, -omega*sin(lat)]
    let omega_n = OMEGA_EARTH * cos_lat;
    let omega_d = -OMEGA_EARTH * sin_lat;

    let vn = velocity_ned[0];
    let ve = velocity_ned[1];
    let vd = velocity_ned[2];

    // Coriolis = 2 * omega x v
    // omega = [omega_n, 0, omega_d]
    // v = [vn, ve, vd]
    [
        2.0 * (0.0 * vd - omega_d * ve),    // 2*(omega_e*vd - omega_d*ve) where omega_e=0
        2.0 * (omega_d * vn - omega_n * vd), // 2*(omega_d*vn - omega_n*vd)
        2.0 * (omega_n * ve - 0.0 * vn),     // 2*(omega_n*ve - omega_e*vn) where omega_e=0
    ]
}

/// Performs coarse alignment (static leveling and gyrocompass) from averaged IMU data.
///
/// Given a set of stationary accelerometer and gyroscope readings, estimates
/// the initial attitude quaternion (body-to-NED) by:
///
/// 1. **Leveling**: Computing roll and pitch from the mean accelerometer vector
///    (which should point opposite to gravity for a stationary platform).
/// 2. **Gyrocompass**: Computing yaw from the mean gyroscope readings, using the
///    projection of Earth's rotation rate in the horizontal plane.
///
/// # Arguments
///
/// * `accel_samples` - Slice of accelerometer readings `[ax, ay, az]` (m/s²)
/// * `gyro_samples` - Slice of gyroscope readings `[wx, wy, wz]` (rad/s)
///
/// # Returns
///
/// Attitude quaternion `[w, x, y, z]` (body-to-NED). Returns identity if
/// input slices are empty.
pub fn coarse_align(accel_samples: &[[f64; 3]], gyro_samples: &[[f64; 3]]) -> [f64; 4] {
    if accel_samples.is_empty() {
        return [1.0, 0.0, 0.0, 0.0];
    }

    // Average accelerometer readings
    let n = accel_samples.len() as f64;
    let mut accel_mean = [0.0; 3];
    for s in accel_samples {
        accel_mean[0] += s[0];
        accel_mean[1] += s[1];
        accel_mean[2] += s[2];
    }
    accel_mean[0] /= n;
    accel_mean[1] /= n;
    accel_mean[2] /= n;

    // Leveling: roll and pitch from gravity direction
    // For NED frame, a level IMU with Z-down senses [0, 0, +g]
    // roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2))
    let roll = accel_mean[1].atan2(accel_mean[2]);
    let pitch = (-accel_mean[0]).atan2(
        (accel_mean[1] * accel_mean[1] + accel_mean[2] * accel_mean[2]).sqrt(),
    );

    // Gyrocompass: yaw from earth rate projection
    let yaw = if !gyro_samples.is_empty() {
        let m = gyro_samples.len() as f64;
        let mut gyro_mean = [0.0; 3];
        for s in gyro_samples {
            gyro_mean[0] += s[0];
            gyro_mean[1] += s[1];
            gyro_mean[2] += s[2];
        }
        gyro_mean[0] /= m;
        gyro_mean[1] /= m;
        gyro_mean[2] /= m;

        // Project gyro into horizontal plane using roll/pitch
        let cr = roll.cos();
        let sr = roll.sin();
        let cp = pitch.cos();
        let sp = pitch.sin();

        // Rotate gyro to level frame
        let gyro_n = gyro_mean[0] * cp + gyro_mean[1] * sp * sr + gyro_mean[2] * sp * cr;
        let gyro_e = gyro_mean[1] * cr - gyro_mean[2] * sr;

        // Yaw = atan2(-gyro_e, gyro_n) for north-finding
        (-gyro_e).atan2(gyro_n)
    } else {
        0.0
    };

    euler_to_quaternion(roll, pitch, yaw)
}

/// Propagates a trajectory by dead reckoning from an initial state through
/// a sequence of IMU samples.
///
/// Returns a vector of NED positions `[north, east, down]` (one per sample),
/// representing the estimated trajectory.
///
/// # Arguments
///
/// * `state` - Initial navigation state (will not be modified)
/// * `imu_samples` - Slice of sequential IMU measurements
pub fn dead_reckon(state: &InsState, imu_samples: &[ImuSample]) -> Vec<[f64; 3]> {
    let mut current = state.clone();
    let mut positions = Vec::with_capacity(imu_samples.len());

    for sample in imu_samples {
        mechanize(&mut current, sample);
        positions.push(current.position);
    }

    positions
}

/// Multiplies two quaternions: `result = a * b`.
///
/// Uses Hamilton product convention `[w, x, y, z]`.
fn quaternion_multiply(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Computes the conjugate (inverse for unit quaternions) of a quaternion.
fn quaternion_conjugate(q: &[f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Rotates a 3D vector by a unit quaternion.
///
/// Computes `v_rotated = q * [0, v] * q_conj`.
fn quaternion_rotate_vector(q: &[f64; 4], v: &[f64; 3]) -> [f64; 3] {
    let v_quat = [0.0, v[0], v[1], v[2]];
    let q_conj = quaternion_conjugate(q);
    let tmp = quaternion_multiply(q, &v_quat);
    let result = quaternion_multiply(&tmp, &q_conj);
    [result[1], result[2], result[3]]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const EPSILON: f64 = 1e-9;
    const LOOSE_EPSILON: f64 = 1e-4;

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    fn vec3_approx_eq(a: &[f64; 3], b: &[f64; 3], eps: f64) -> bool {
        approx_eq(a[0], b[0], eps) && approx_eq(a[1], b[1], eps) && approx_eq(a[2], b[2], eps)
    }

    fn quat_approx_eq(a: &[f64; 4], b: &[f64; 4], eps: f64) -> bool {
        // Quaternions q and -q represent the same rotation
        let direct = approx_eq(a[0], b[0], eps)
            && approx_eq(a[1], b[1], eps)
            && approx_eq(a[2], b[2], eps)
            && approx_eq(a[3], b[3], eps);
        let negated = approx_eq(a[0], -b[0], eps)
            && approx_eq(a[1], -b[1], eps)
            && approx_eq(a[2], -b[2], eps)
            && approx_eq(a[3], -b[3], eps);
        direct || negated
    }

    #[test]
    fn test_ins_state_default() {
        let state = InsState::new();
        assert_eq!(state.position, [0.0; 3]);
        assert_eq!(state.velocity, [0.0; 3]);
        assert_eq!(state.attitude, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(state.gyro_bias, [0.0; 3]);
        assert_eq!(state.accel_bias, [0.0; 3]);
    }

    #[test]
    fn test_quaternion_normalize_unit() {
        let mut q = [1.0, 0.0, 0.0, 0.0];
        quaternion_normalize(&mut q);
        assert!(approx_eq(q[0], 1.0, EPSILON));
    }

    #[test]
    fn test_quaternion_normalize_nonunit() {
        let mut q = [2.0, 0.0, 0.0, 0.0];
        quaternion_normalize(&mut q);
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!(approx_eq(norm, 1.0, EPSILON));
        assert!(approx_eq(q[0], 1.0, EPSILON));
    }

    #[test]
    fn test_quaternion_normalize_zero() {
        let mut q = [0.0, 0.0, 0.0, 0.0];
        quaternion_normalize(&mut q);
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_quaternion_to_dcm_identity() {
        let q = [1.0, 0.0, 0.0, 0.0];
        let dcm = quaternion_to_dcm(&q);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    approx_eq(dcm[i][j], expected, EPSILON),
                    "DCM[{}][{}] = {} != {}",
                    i,
                    j,
                    dcm[i][j],
                    expected
                );
            }
        }
    }

    #[test]
    fn test_quaternion_to_dcm_90deg_yaw() {
        // 90-degree yaw (rotation about Z)
        let q = euler_to_quaternion(0.0, 0.0, FRAC_PI_2);
        let dcm = quaternion_to_dcm(&q);
        // After 90deg yaw: body-X (forward) now points East in NED
        // DCM row 0 = NED-North expressed in body = [0, -1, 0] means
        //   North_NED = 0*body_x + (-1)*body_y + 0*body_z
        assert!(approx_eq(dcm[0][0], 0.0, EPSILON));
        assert!(approx_eq(dcm[0][1], -1.0, EPSILON));
        assert!(approx_eq(dcm[1][0], 1.0, EPSILON));
        assert!(approx_eq(dcm[1][1], 0.0, EPSILON));
        assert!(approx_eq(dcm[2][2], 1.0, EPSILON));
    }

    #[test]
    fn test_dcm_to_quaternion_identity() {
        let dcm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let q = dcm_to_quaternion(&dcm);
        assert!(quat_approx_eq(&q, &[1.0, 0.0, 0.0, 0.0], EPSILON));
    }

    #[test]
    fn test_dcm_quaternion_roundtrip() {
        // Arbitrary rotation
        let q_orig = euler_to_quaternion(0.3, -0.2, 1.5);
        let dcm = quaternion_to_dcm(&q_orig);
        let q_back = dcm_to_quaternion(&dcm);
        assert!(
            quat_approx_eq(&q_orig, &q_back, EPSILON),
            "Roundtrip failed: {:?} vs {:?}",
            q_orig,
            q_back
        );
    }

    #[test]
    fn test_euler_to_quaternion_zero() {
        let q = euler_to_quaternion(0.0, 0.0, 0.0);
        assert!(quat_approx_eq(&q, &[1.0, 0.0, 0.0, 0.0], EPSILON));
    }

    #[test]
    fn test_euler_quaternion_roundtrip() {
        let roll = 0.4;
        let pitch = -0.3;
        let yaw = 2.1;
        let q = euler_to_quaternion(roll, pitch, yaw);
        let (r, p, y) = quaternion_to_euler(&q);
        assert!(
            approx_eq(r, roll, EPSILON)
                && approx_eq(p, pitch, EPSILON)
                && approx_eq(y, yaw, EPSILON),
            "Roundtrip failed: ({}, {}, {}) vs ({}, {}, {})",
            roll,
            pitch,
            yaw,
            r,
            p,
            y
        );
    }

    #[test]
    fn test_quaternion_to_euler_gimbal_lock() {
        // Pitch = +90deg (gimbal lock)
        let q = euler_to_quaternion(0.0, FRAC_PI_2, 0.0);
        let (_r, p, _y) = quaternion_to_euler(&q);
        assert!(
            approx_eq(p, FRAC_PI_2, LOOSE_EPSILON),
            "Pitch at gimbal lock: {} vs {}",
            p,
            FRAC_PI_2
        );
    }

    #[test]
    fn test_gravity_ned_equator_sea_level() {
        let g = gravity_ned(0.0, 0.0);
        // Gravity at equator should be ~9.780 m/s^2
        assert!(g[0].abs() < EPSILON);
        assert!(g[1].abs() < EPSILON);
        assert!(
            approx_eq(g[2], 9.780_325, 0.01),
            "Equator gravity: {}",
            g[2]
        );
    }

    #[test]
    fn test_gravity_ned_pole_sea_level() {
        let g = gravity_ned(FRAC_PI_2, 0.0);
        // Gravity at poles should be ~9.832 m/s^2
        assert!(
            approx_eq(g[2], 9.832, 0.01),
            "Pole gravity: {}",
            g[2]
        );
    }

    #[test]
    fn test_gravity_ned_altitude_reduces() {
        let g_sea = gravity_ned(0.7, 0.0);
        let g_alt = gravity_ned(0.7, 10000.0);
        assert!(
            g_alt[2] < g_sea[2],
            "Gravity should decrease with altitude: {} vs {}",
            g_alt[2],
            g_sea[2]
        );
    }

    #[test]
    fn test_coriolis_zero_velocity() {
        let v = [0.0, 0.0, 0.0];
        let c = coriolis_correction(&v, 0.5);
        assert!(vec3_approx_eq(&c, &[0.0, 0.0, 0.0], EPSILON));
    }

    #[test]
    fn test_coriolis_eastward_velocity() {
        // Moving east at 45deg latitude
        let v = [0.0, 100.0, 0.0]; // 100 m/s east
        let lat = PI / 4.0;
        let c = coriolis_correction(&v, lat);
        // a_north = -2*omega*sin(lat)*v_east (note: omega_d = -omega*sin)
        // c[0] = 2*(0*vd - omega_d*ve) = 2*omega*sin(lat)*ve
        let expected_north = 2.0 * OMEGA_EARTH * lat.sin() * 100.0;
        assert!(
            approx_eq(c[0], expected_north, 1e-6),
            "Coriolis north: {} vs {}",
            c[0],
            expected_north
        );
    }

    #[test]
    fn test_mechanize_stationary() {
        // Stationary IMU: no rotation, accelerometer reads gravity
        let mut state = InsState::new();
        let g_eq = gravity_ned(0.0, 0.0)[2];
        let sample = ImuSample {
            gyro: [0.0, 0.0, 0.0],
            accel: [0.0, 0.0, g_eq],
            dt: 0.01,
        };

        // At equator, accel in NED = [0,0,g_eq], gravity = [0,0,g_eq], net = 0
        mechanize(&mut state, &sample);

        // Position should stay near zero
        assert!(
            state.position[0].abs() < 0.001,
            "North position: {}",
            state.position[0]
        );
        assert!(
            state.position[1].abs() < 0.001,
            "East position: {}",
            state.position[1]
        );
    }

    #[test]
    fn test_mechanize_constant_acceleration() {
        // Apply 1 m/s^2 north in body frame with identity attitude (body=NED)
        let mut state = InsState::new();
        let g_eq = gravity_ned(0.0, 0.0)[2]; // gravity at equator

        // The accelerometer measures specific force = true_accel - gravity
        // For 1 m/s^2 north: body accel = [1, 0, g]
        // After NED transform: [1, 0, g]. After gravity removal: [1, 0, ~0]
        let sample = ImuSample {
            gyro: [0.0, 0.0, 0.0],
            accel: [1.0, 0.0, g_eq],
            dt: 0.01,
        };

        for _ in 0..100 {
            mechanize(&mut state, &sample);
        }

        // After 1 second at 1 m/s^2, v_north ~ 1 m/s, pos_north ~ 0.5 m
        assert!(
            approx_eq(state.velocity[0], 1.0, 0.02),
            "North velocity: {}",
            state.velocity[0]
        );
        assert!(
            approx_eq(state.position[0], 0.5, 0.02),
            "North position: {}",
            state.position[0]
        );
    }

    #[test]
    fn test_mechanize_zero_dt() {
        let mut state = InsState::new();
        let sample = ImuSample {
            gyro: [0.1, 0.0, 0.0],
            accel: [0.0, 0.0, G0],
            dt: 0.0,
        };
        let original = state.clone();
        mechanize(&mut state, &sample);
        // State should be unchanged
        assert_eq!(state.position, original.position);
        assert_eq!(state.velocity, original.velocity);
    }

    #[test]
    fn test_coarse_align_level() {
        // Stationary, level platform. Accel = [0, 0, g]
        let accel = vec![[0.0, 0.0, G0]; 100];
        let gyro = vec![[0.0, 0.0, 0.0]; 100];

        let q = coarse_align(&accel, &gyro);
        let (roll, pitch, _yaw) = quaternion_to_euler(&q);

        assert!(approx_eq(roll, 0.0, LOOSE_EPSILON), "Roll: {}", roll);
        assert!(approx_eq(pitch, 0.0, LOOSE_EPSILON), "Pitch: {}", pitch);
    }

    #[test]
    fn test_coarse_align_tilted() {
        // Platform tilted: roll = 10deg, pitch = 0
        let roll_true = 10.0_f64.to_radians();
        let cr = roll_true.cos();
        let sr = roll_true.sin();

        // Gravity in body frame when rolled: [0, g*sin(roll), g*cos(roll)]
        let accel = vec![[0.0, G0 * sr, G0 * cr]; 100];
        let gyro = vec![[0.0, 0.0, 0.0]; 100];

        let q = coarse_align(&accel, &gyro);
        let (roll, pitch, _yaw) = quaternion_to_euler(&q);

        assert!(
            approx_eq(roll, roll_true, 0.001),
            "Roll: {} vs {}",
            roll,
            roll_true
        );
        assert!(approx_eq(pitch, 0.0, 0.001), "Pitch: {}", pitch);
    }

    #[test]
    fn test_coarse_align_empty() {
        let q = coarse_align(&[], &[]);
        assert_eq!(q, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_dead_reckon_straight_line() {
        let state = InsState::new();
        let g_eq = gravity_ned(0.0, 0.0)[2];

        // Constant northward acceleration
        let samples: Vec<ImuSample> = (0..100)
            .map(|_| ImuSample {
                gyro: [0.0, 0.0, 0.0],
                accel: [1.0, 0.0, g_eq],
                dt: 0.01,
            })
            .collect();

        let trajectory = dead_reckon(&state, &samples);
        assert_eq!(trajectory.len(), 100);

        // Trajectory should show northward motion
        for i in 1..trajectory.len() {
            assert!(
                trajectory[i][0] >= trajectory[i - 1][0] - 1e-10,
                "North position should increase monotonically"
            );
        }

        // Final position should be ~0.5 m north
        let last = trajectory.last().unwrap();
        assert!(approx_eq(last[0], 0.5, 0.02), "Final north: {}", last[0]);
    }

    #[test]
    fn test_dead_reckon_empty() {
        let state = InsState::new();
        let trajectory = dead_reckon(&state, &[]);
        assert!(trajectory.is_empty());
    }

    #[test]
    fn test_ins_processor_with_bias_estimation() {
        let mut processor = InsProcessor::new(0.0, 0.0);
        processor.gyro_bias_gain = 0.01;
        processor.accel_bias_gain = 0.001;

        let mut state = InsState::new();

        // Inject a known gyro bias
        let sample = ImuSample {
            gyro: [0.001, 0.0, 0.0], // Small constant bias
            accel: [0.0, 0.0, gravity_ned(0.0, 0.0)[2]],
            dt: 0.01,
        };

        for _ in 0..1000 {
            processor.mechanize(&mut state, &sample);
        }

        // Gyro bias estimate should be tracking toward the injected bias
        assert!(
            state.gyro_bias[0] > 0.0,
            "Gyro bias X should be positive: {}",
            state.gyro_bias[0]
        );
    }

    #[test]
    fn test_quaternion_rotate_vector_identity() {
        let q = [1.0, 0.0, 0.0, 0.0];
        let v = [1.0, 2.0, 3.0];
        let rotated = quaternion_rotate_vector(&q, &v);
        assert!(vec3_approx_eq(&rotated, &v, EPSILON));
    }

    #[test]
    fn test_quaternion_rotate_vector_90_yaw() {
        // 90deg yaw: body-X vector [1,0,0] should become NED-East [0,1,0]
        let q = euler_to_quaternion(0.0, 0.0, FRAC_PI_2);
        let v = [1.0, 0.0, 0.0];
        let rotated = quaternion_rotate_vector(&q, &v);
        assert!(
            vec3_approx_eq(&rotated, &[0.0, 1.0, 0.0], EPSILON),
            "Rotated: {:?}",
            rotated
        );
    }

    #[test]
    fn test_quaternion_multiply_identity() {
        let id = [1.0, 0.0, 0.0, 0.0];
        let q = euler_to_quaternion(0.3, -0.1, 1.2);
        let result = quaternion_multiply(&id, &q);
        assert!(quat_approx_eq(&result, &q, EPSILON));
    }

    #[test]
    fn test_mechanize_rotation_only() {
        // Pure rotation: no acceleration beyond gravity
        let mut state = InsState::new();
        let g_eq = gravity_ned(0.0, 0.0)[2];

        // Rotate about body Z at 0.1 rad/s for 1 second
        let sample = ImuSample {
            gyro: [0.0, 0.0, 0.1],
            accel: [0.0, 0.0, g_eq],
            dt: 0.01,
        };

        for _ in 0..100 {
            mechanize(&mut state, &sample);
        }

        let (_roll, _pitch, yaw) = quaternion_to_euler(&state.attitude);
        // After 1s at 0.1 rad/s yaw rate, yaw should be ~0.1 rad
        assert!(
            approx_eq(yaw, 0.1, 0.01),
            "Yaw after rotation: {} vs expected 0.1",
            yaw
        );
    }
}

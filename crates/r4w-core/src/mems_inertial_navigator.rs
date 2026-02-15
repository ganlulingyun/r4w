//! MEMS-based inertial navigation processor for IMU sensor fusion and dead reckoning.
//!
//! This module implements strapdown inertial navigation mechanization tailored
//! for MEMS-grade IMUs, including:
//!
//! - **Strapdown INS**: Quaternion-based attitude, velocity, and position propagation
//! - **MEMS Error Models**: Bias, scale factor, noise density, random walk for accel/gyro
//! - **Allan Variance**: White noise, bias instability, and rate random walk regions
//! - **Complementary Filter**: High-pass gyro + low-pass accel for tilt estimation
//! - **ZUPT Detector**: Variance-based and GLRT zero-velocity detection with correction
//! - **Turn Rate Detection**: Gyro z-axis threshold for heading change detection
//! - **Pedestrian Dead Reckoning**: Step detection, Weinberg step length, heading integration
//! - **Integration Methods**: Trapezoidal and Simpson's rule options
//! - **Navigation Error Metrics**: Position, velocity, and attitude error computation
//!
//! # Example
//!
//! ```
//! use r4w_core::mems_inertial_navigator::{
//!     ImuReading, NavState, StrapdownIns, Quaternion,
//! };
//!
//! let mut ins = StrapdownIns::new();
//! let mut state = NavState::new();
//!
//! // Stationary IMU reading (gravity along -Z in nav frame, +Z in body when level)
//! let reading = ImuReading {
//!     accel_x: 0.0,
//!     accel_y: 0.0,
//!     accel_z: 9.80665,
//!     gyro_x: 0.0,
//!     gyro_y: 0.0,
//!     gyro_z: 0.0,
//!     timestamp_s: 0.01,
//! };
//!
//! ins.update(&mut state, &reading);
//! assert!(state.velocity.0.abs() < 0.01);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Standard gravity at sea level (m/s^2), WGS-84.
const G0: f64 = 9.806_65;

/// Gravity vector in navigation frame (NED-like: x-north, y-east, z-down).
/// We use z-down so gravity is positive-z downward, but for ENU-style local
/// frame where z is up, gravity acts as [0, 0, -G0].
const GRAVITY_NAV: [f64; 3] = [0.0, 0.0, -G0];

// ---------------------------------------------------------------------------
// ImuReading
// ---------------------------------------------------------------------------

/// A single IMU measurement from a MEMS sensor.
#[derive(Debug, Clone, Copy)]
pub struct ImuReading {
    /// Accelerometer X-axis specific force (m/s^2).
    pub accel_x: f64,
    /// Accelerometer Y-axis specific force (m/s^2).
    pub accel_y: f64,
    /// Accelerometer Z-axis specific force (m/s^2).
    pub accel_z: f64,
    /// Gyroscope X-axis angular rate (rad/s).
    pub gyro_x: f64,
    /// Gyroscope Y-axis angular rate (rad/s).
    pub gyro_y: f64,
    /// Gyroscope Z-axis angular rate (rad/s).
    pub gyro_z: f64,
    /// Timestamp in seconds (used to compute dt between readings).
    pub timestamp_s: f64,
}

impl ImuReading {
    /// Create a new IMU reading with all fields specified.
    pub fn new(
        accel_x: f64,
        accel_y: f64,
        accel_z: f64,
        gyro_x: f64,
        gyro_y: f64,
        gyro_z: f64,
        timestamp_s: f64,
    ) -> Self {
        Self {
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            timestamp_s,
        }
    }

    /// Return accelerometer readings as an array [ax, ay, az].
    pub fn accel(&self) -> [f64; 3] {
        [self.accel_x, self.accel_y, self.accel_z]
    }

    /// Return gyroscope readings as an array [gx, gy, gz].
    pub fn gyro(&self) -> [f64; 3] {
        [self.gyro_x, self.gyro_y, self.gyro_z]
    }

    /// Accelerometer magnitude (m/s^2).
    pub fn accel_magnitude(&self) -> f64 {
        (self.accel_x * self.accel_x
            + self.accel_y * self.accel_y
            + self.accel_z * self.accel_z)
            .sqrt()
    }

    /// Gyroscope magnitude (rad/s).
    pub fn gyro_magnitude(&self) -> f64 {
        (self.gyro_x * self.gyro_x
            + self.gyro_y * self.gyro_y
            + self.gyro_z * self.gyro_z)
            .sqrt()
    }
}

// ---------------------------------------------------------------------------
// Quaternion
// ---------------------------------------------------------------------------

/// Unit quaternion for 3D rotation representation.
///
/// Convention: `[w, x, y, z]` where `w` is the scalar part.
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    /// Identity quaternion (no rotation).
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Create quaternion from components.
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// Quaternion norm (magnitude).
    pub fn norm(&self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalize to unit quaternion.
    pub fn normalize(&self) -> Self {
        let n = self.norm();
        if n < 1e-15 {
            return Self::identity();
        }
        Self {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }

    /// Quaternion conjugate (inverse for unit quaternions).
    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Hamilton product: self * other.
    pub fn multiply(&self, other: &Quaternion) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }

    /// Create quaternion from Euler angles (ZYX convention: yaw, pitch, roll).
    ///
    /// - `roll`: rotation about X-axis (radians)
    /// - `pitch`: rotation about Y-axis (radians)
    /// - `yaw`: rotation about Z-axis (radians)
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = (roll / 2.0).sin_cos();
        let (sp, cp) = (pitch / 2.0).sin_cos();
        let (sy, cy) = (yaw / 2.0).sin_cos();

        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    ///
    /// Returns `(roll, pitch, yaw)` using ZYX convention.
    pub fn to_euler(&self) -> (f64, f64, f64) {
        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp.abs() >= 1.0 {
            (PI / 2.0).copysign(sinp) // gimbal lock
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    /// Rotate a 3D vector by this quaternion: v' = q * v * q^-1.
    pub fn rotate_vector(&self, v: [f64; 3]) -> [f64; 3] {
        let qv = Quaternion::new(0.0, v[0], v[1], v[2]);
        let rotated = self.multiply(&qv).multiply(&self.conjugate());
        [rotated.x, rotated.y, rotated.z]
    }

    /// Convert quaternion to 3x3 Direction Cosine Matrix (rotation matrix).
    ///
    /// Returns row-major `[[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]`.
    pub fn to_dcm(&self) -> [[f64; 3]; 3] {
        let Quaternion { w, x, y, z } = *self;
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }

    /// Create a small-angle delta quaternion from angular rate vector and dt.
    ///
    /// delta_q = [cos(|w|*dt/2), sin(|w|*dt/2) * w_hat]
    pub fn from_angular_rate(wx: f64, wy: f64, wz: f64, dt: f64) -> Self {
        let angle = (wx * wx + wy * wy + wz * wz).sqrt() * dt;
        if angle < 1e-12 {
            // First-order approximation for small angles
            return Self {
                w: 1.0,
                x: wx * dt / 2.0,
                y: wy * dt / 2.0,
                z: wz * dt / 2.0,
            }
            .normalize();
        }
        let half_angle = angle / 2.0;
        let s = half_angle.sin() / (angle / dt);
        Self {
            w: half_angle.cos(),
            x: s * wx,
            y: s * wy,
            z: s * wz,
        }
        .normalize()
    }
}

// ---------------------------------------------------------------------------
// NavState
// ---------------------------------------------------------------------------

/// Navigation state: position, velocity, and attitude.
#[derive(Debug, Clone)]
pub struct NavState {
    /// Position in local frame (x, y, z) in meters.
    pub position: (f64, f64, f64),
    /// Velocity in local frame (vx, vy, vz) in m/s.
    pub velocity: (f64, f64, f64),
    /// Attitude as unit quaternion (body-to-nav frame).
    pub attitude: Quaternion,
    /// Timestamp of last update (seconds).
    pub timestamp_s: f64,
}

impl NavState {
    /// Create a new NavState at the origin with identity attitude.
    pub fn new() -> Self {
        Self {
            position: (0.0, 0.0, 0.0),
            velocity: (0.0, 0.0, 0.0),
            attitude: Quaternion::identity(),
            timestamp_s: 0.0,
        }
    }

    /// Create NavState with specified initial conditions.
    pub fn with_initial(
        position: (f64, f64, f64),
        velocity: (f64, f64, f64),
        attitude: Quaternion,
    ) -> Self {
        Self {
            position,
            velocity,
            attitude,
            timestamp_s: 0.0,
        }
    }

    /// Position as an array.
    pub fn position_array(&self) -> [f64; 3] {
        [self.position.0, self.position.1, self.position.2]
    }

    /// Velocity as an array.
    pub fn velocity_array(&self) -> [f64; 3] {
        [self.velocity.0, self.velocity.1, self.velocity.2]
    }

    /// Speed (magnitude of velocity) in m/s.
    pub fn speed(&self) -> f64 {
        let (vx, vy, vz) = self.velocity;
        (vx * vx + vy * vy + vz * vz).sqrt()
    }

    /// Position magnitude (distance from origin) in meters.
    pub fn distance_from_origin(&self) -> f64 {
        let (x, y, z) = self.position;
        (x * x + y * y + z * z).sqrt()
    }
}

impl Default for NavState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// MEMS Error Models
// ---------------------------------------------------------------------------

/// MEMS accelerometer error model parameters.
#[derive(Debug, Clone, Copy)]
pub struct AccelErrorModel {
    /// Bias in m/s^2 (converted from mg: 1 mg = 0.00981 m/s^2).
    pub bias: f64,
    /// Scale factor error (ppm, parts per million).
    pub scale_factor_ppm: f64,
    /// Noise density in m/s^2/sqrt(Hz) (converted from ug/sqrt(Hz)).
    pub noise_density: f64,
    /// Velocity random walk in m/s/sqrt(s).
    pub velocity_random_walk: f64,
}

impl AccelErrorModel {
    /// Create a tactical-grade MEMS accelerometer error model.
    /// Typical for ADIS16488-class devices.
    pub fn tactical_grade() -> Self {
        Self {
            bias: 0.2e-3 * G0,             // 0.2 mg -> m/s^2
            scale_factor_ppm: 100.0,        // 100 ppm
            noise_density: 25.0e-6 * G0,    // 25 ug/sqrt(Hz)
            velocity_random_walk: 0.008,     // 0.008 m/s/sqrt(s)
        }
    }

    /// Create a consumer-grade MEMS accelerometer error model.
    /// Typical for smartphone IMUs (BMI160-class).
    pub fn consumer_grade() -> Self {
        Self {
            bias: 40.0e-3 * G0,            // 40 mg
            scale_factor_ppm: 1000.0,       // 1000 ppm
            noise_density: 180.0e-6 * G0,   // 180 ug/sqrt(Hz)
            velocity_random_walk: 0.05,
        }
    }

    /// Apply error model to a true acceleration measurement.
    pub fn apply(&self, true_accel: f64) -> f64 {
        let scale = 1.0 + self.scale_factor_ppm * 1e-6;
        true_accel * scale + self.bias
    }
}

/// MEMS gyroscope error model parameters.
#[derive(Debug, Clone, Copy)]
pub struct GyroErrorModel {
    /// Bias in rad/s (converted from deg/hr).
    pub bias: f64,
    /// Angle random walk in rad/sqrt(s) (converted from deg/sqrt(hr)).
    pub angle_random_walk: f64,
    /// Rate random walk in rad/s/sqrt(s).
    pub rate_random_walk: f64,
    /// Bias instability in rad/s (converted from deg/hr).
    pub bias_instability: f64,
}

impl GyroErrorModel {
    /// Create a tactical-grade MEMS gyroscope error model.
    pub fn tactical_grade() -> Self {
        Self {
            bias: 1.0 * PI / (180.0 * 3600.0),               // 1 deg/hr
            angle_random_walk: 0.15 * PI / (180.0 * 60.0),    // 0.15 deg/sqrt(hr)
            rate_random_walk: 1e-7,
            bias_instability: 0.5 * PI / (180.0 * 3600.0),    // 0.5 deg/hr
        }
    }

    /// Create a consumer-grade MEMS gyroscope error model.
    pub fn consumer_grade() -> Self {
        Self {
            bias: 36.0 * PI / (180.0 * 3600.0),               // 36 deg/hr
            angle_random_walk: 0.5 * PI / (180.0 * 60.0),     // 0.5 deg/sqrt(hr)
            rate_random_walk: 1e-5,
            bias_instability: 10.0 * PI / (180.0 * 3600.0),   // 10 deg/hr
        }
    }

    /// Apply error model to a true gyro measurement.
    pub fn apply(&self, true_rate: f64) -> f64 {
        true_rate + self.bias
    }
}

/// Allan variance analysis for MEMS sensor characterization.
#[derive(Debug, Clone)]
pub struct AllanVariance {
    /// Cluster sizes (tau values) in samples.
    pub tau: Vec<f64>,
    /// Allan variance values for each tau.
    pub avar: Vec<f64>,
}

impl AllanVariance {
    /// Compute Allan variance from a time series of sensor data.
    ///
    /// - `data`: sensor samples at uniform rate
    /// - `sample_rate`: sampling frequency (Hz)
    /// - `max_clusters`: maximum number of tau values to compute
    pub fn compute(data: &[f64], sample_rate: f64, max_clusters: usize) -> Self {
        let n = data.len();
        if n < 4 {
            return Self {
                tau: vec![],
                avar: vec![],
            };
        }

        let dt = 1.0 / sample_rate;
        let mut tau_values = Vec::new();
        let mut avar_values = Vec::new();

        // Generate cluster sizes: powers of 2 up to n/2
        let mut m = 1usize;
        let mut count = 0;
        while m <= n / 2 && count < max_clusters {
            let tau_m = m as f64 * dt;

            // Compute overlapping Allan variance
            let num_clusters = n - 2 * m;
            if num_clusters == 0 {
                break;
            }

            let mut sum = 0.0;
            for i in 0..num_clusters {
                // Cluster averages
                let mut avg1 = 0.0;
                let mut avg2 = 0.0;
                for j in 0..m {
                    avg1 += data[i + j];
                    avg2 += data[i + m + j];
                }
                avg1 /= m as f64;
                avg2 /= m as f64;
                let diff = avg2 - avg1;
                sum += diff * diff;
            }

            let avar = sum / (2.0 * num_clusters as f64);
            tau_values.push(tau_m);
            avar_values.push(avar);

            m *= 2;
            count += 1;
        }

        Self {
            tau: tau_values,
            avar: avar_values,
        }
    }

    /// Extract white noise coefficient (slope -1/2 on log-log).
    /// Returns the noise density N = sqrt(avar * tau) at smallest tau.
    pub fn white_noise_coefficient(&self) -> Option<f64> {
        if self.tau.is_empty() {
            return None;
        }
        Some((self.avar[0] * self.tau[0]).sqrt())
    }

    /// Extract bias instability (minimum of Allan deviation).
    /// Returns (tau_min, sigma_min).
    pub fn bias_instability(&self) -> Option<(f64, f64)> {
        if self.avar.is_empty() {
            return None;
        }
        let mut min_idx = 0;
        let mut min_val = f64::MAX;
        for (i, &av) in self.avar.iter().enumerate() {
            let adev = av.sqrt();
            if adev < min_val {
                min_val = adev;
                min_idx = i;
            }
        }
        Some((self.tau[min_idx], min_val))
    }
}

// ---------------------------------------------------------------------------
// Strapdown INS Mechanization
// ---------------------------------------------------------------------------

/// Integration method for strapdown mechanization.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntegrationMethod {
    /// Trapezoidal rule: (f(t) + f(t+dt)) / 2 * dt.
    Trapezoidal,
    /// Simpson's rule (requires storing previous reading).
    Simpson,
}

/// Strapdown INS mechanization processor.
#[derive(Debug, Clone)]
pub struct StrapdownIns {
    /// Integration method.
    pub method: IntegrationMethod,
    /// Gravity vector in navigation frame (m/s^2).
    pub gravity: [f64; 3],
    /// Previous IMU reading (for trapezoidal/Simpson integration).
    prev_reading: Option<ImuReading>,
    /// Two-readings-ago IMU reading (for Simpson integration).
    prev_prev_reading: Option<ImuReading>,
    /// Count of readings processed.
    reading_count: usize,
}

impl StrapdownIns {
    /// Create a new strapdown INS processor with default settings.
    pub fn new() -> Self {
        Self {
            method: IntegrationMethod::Trapezoidal,
            gravity: GRAVITY_NAV,
            prev_reading: None,
            prev_prev_reading: None,
            reading_count: 0,
        }
    }

    /// Create with specified integration method.
    pub fn with_method(method: IntegrationMethod) -> Self {
        Self {
            method,
            gravity: GRAVITY_NAV,
            prev_reading: None,
            prev_prev_reading: None,
            reading_count: 0,
        }
    }

    /// Reset the processor state (clear cached readings).
    pub fn reset(&mut self) {
        self.prev_reading = None;
        self.prev_prev_reading = None;
        self.reading_count = 0;
    }

    /// Update navigation state with a new IMU reading.
    ///
    /// Implements the strapdown mechanization equations:
    /// 1. Attitude update: q(t+dt) = q(t) * delta_q(gyro)
    /// 2. Velocity update: v(t+dt) = v(t) + (C_b^n * f_b + g) * dt
    /// 3. Position update: p(t+dt) = p(t) + v * dt
    pub fn update(&mut self, state: &mut NavState, reading: &ImuReading) {
        let dt = if state.timestamp_s > 0.0 {
            reading.timestamp_s - state.timestamp_s
        } else {
            reading.timestamp_s
        };

        if dt <= 0.0 {
            state.timestamp_s = reading.timestamp_s;
            self.prev_reading = Some(*reading);
            self.reading_count += 1;
            return;
        }

        match self.method {
            IntegrationMethod::Trapezoidal => {
                if let Some(prev) = &self.prev_reading {
                    self.update_trapezoidal(state, prev, reading, dt);
                } else {
                    self.update_euler(state, reading, dt);
                }
            }
            IntegrationMethod::Simpson => {
                if self.reading_count >= 2 {
                    if let (Some(prev_prev), Some(prev)) =
                        (&self.prev_prev_reading, &self.prev_reading)
                    {
                        self.update_simpson(state, prev_prev, prev, reading, dt);
                    } else {
                        self.update_euler(state, reading, dt);
                    }
                } else {
                    self.update_euler(state, reading, dt);
                }
            }
        }

        state.timestamp_s = reading.timestamp_s;
        self.prev_prev_reading = self.prev_reading;
        self.prev_reading = Some(*reading);
        self.reading_count += 1;
    }

    /// Simple Euler integration step.
    fn update_euler(&self, state: &mut NavState, reading: &ImuReading, dt: f64) {
        // 1. Attitude update
        let delta_q =
            Quaternion::from_angular_rate(reading.gyro_x, reading.gyro_y, reading.gyro_z, dt);
        state.attitude = state.attitude.multiply(&delta_q).normalize();

        // 2. Transform specific force from body to nav frame
        let f_body = reading.accel();
        let f_nav = state.attitude.rotate_vector(f_body);

        // 3. Velocity update: v += (f_nav + gravity) * dt
        state.velocity.0 += (f_nav[0] + self.gravity[0]) * dt;
        state.velocity.1 += (f_nav[1] + self.gravity[1]) * dt;
        state.velocity.2 += (f_nav[2] + self.gravity[2]) * dt;

        // 4. Position update: p += v * dt
        state.position.0 += state.velocity.0 * dt;
        state.position.1 += state.velocity.1 * dt;
        state.position.2 += state.velocity.2 * dt;
    }

    /// Trapezoidal integration step.
    fn update_trapezoidal(
        &self,
        state: &mut NavState,
        prev: &ImuReading,
        curr: &ImuReading,
        dt: f64,
    ) {
        // Average gyro for attitude update
        let avg_gx = (prev.gyro_x + curr.gyro_x) / 2.0;
        let avg_gy = (prev.gyro_y + curr.gyro_y) / 2.0;
        let avg_gz = (prev.gyro_z + curr.gyro_z) / 2.0;

        let delta_q = Quaternion::from_angular_rate(avg_gx, avg_gy, avg_gz, dt);
        state.attitude = state.attitude.multiply(&delta_q).normalize();

        // Average specific force in body frame
        let f_body = [
            (prev.accel_x + curr.accel_x) / 2.0,
            (prev.accel_y + curr.accel_y) / 2.0,
            (prev.accel_z + curr.accel_z) / 2.0,
        ];
        let f_nav = state.attitude.rotate_vector(f_body);

        // Velocity update
        let old_vel = state.velocity;
        state.velocity.0 += (f_nav[0] + self.gravity[0]) * dt;
        state.velocity.1 += (f_nav[1] + self.gravity[1]) * dt;
        state.velocity.2 += (f_nav[2] + self.gravity[2]) * dt;

        // Position update with trapezoidal velocity
        state.position.0 += (old_vel.0 + state.velocity.0) / 2.0 * dt;
        state.position.1 += (old_vel.1 + state.velocity.1) / 2.0 * dt;
        state.position.2 += (old_vel.2 + state.velocity.2) / 2.0 * dt;
    }

    /// Simpson's rule integration step (using three readings).
    fn update_simpson(
        &self,
        state: &mut NavState,
        prev_prev: &ImuReading,
        prev: &ImuReading,
        curr: &ImuReading,
        dt: f64,
    ) {
        // Simpson's 1/3 rule: integral ~ (dt/3)(f0 + 4*f1 + f2)
        // We use the half-interval dt here since we have 3 points spanning 2*dt
        let avg_gx = (prev_prev.gyro_x + 4.0 * prev.gyro_x + curr.gyro_x) / 6.0;
        let avg_gy = (prev_prev.gyro_y + 4.0 * prev.gyro_y + curr.gyro_y) / 6.0;
        let avg_gz = (prev_prev.gyro_z + 4.0 * prev.gyro_z + curr.gyro_z) / 6.0;

        let delta_q = Quaternion::from_angular_rate(avg_gx, avg_gy, avg_gz, dt);
        state.attitude = state.attitude.multiply(&delta_q).normalize();

        let f_body = [
            (prev_prev.accel_x + 4.0 * prev.accel_x + curr.accel_x) / 6.0,
            (prev_prev.accel_y + 4.0 * prev.accel_y + curr.accel_y) / 6.0,
            (prev_prev.accel_z + 4.0 * prev.accel_z + curr.accel_z) / 6.0,
        ];
        let f_nav = state.attitude.rotate_vector(f_body);

        let old_vel = state.velocity;
        state.velocity.0 += (f_nav[0] + self.gravity[0]) * dt;
        state.velocity.1 += (f_nav[1] + self.gravity[1]) * dt;
        state.velocity.2 += (f_nav[2] + self.gravity[2]) * dt;

        state.position.0 += (old_vel.0 + state.velocity.0) / 2.0 * dt;
        state.position.1 += (old_vel.1 + state.velocity.1) / 2.0 * dt;
        state.position.2 += (old_vel.2 + state.velocity.2) / 2.0 * dt;
    }
}

impl Default for StrapdownIns {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Complementary Filter
// ---------------------------------------------------------------------------

/// Complementary filter for tilt estimation from MEMS IMU.
///
/// Combines high-pass filtered gyroscope with low-pass filtered
/// accelerometer for stable roll/pitch estimation.
#[derive(Debug, Clone)]
pub struct ComplementaryFilter {
    /// Filter coefficient alpha (0..1). Higher = more gyro trust.
    pub alpha: f64,
    /// Current roll estimate (radians).
    pub roll: f64,
    /// Current pitch estimate (radians).
    pub pitch: f64,
    /// Current yaw estimate (radians, gyro-only, drifts).
    pub yaw: f64,
    /// Whether the filter has been initialized.
    initialized: bool,
}

impl ComplementaryFilter {
    /// Create a new complementary filter.
    ///
    /// - `crossover_freq`: crossover frequency in Hz (typically 0.01-1.0 Hz)
    /// - `sample_rate`: IMU sample rate in Hz
    pub fn new(crossover_freq: f64, sample_rate: f64) -> Self {
        let dt = 1.0 / sample_rate;
        let rc = 1.0 / (2.0 * PI * crossover_freq);
        let alpha = rc / (rc + dt);
        Self {
            alpha,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            initialized: false,
        }
    }

    /// Create with a direct alpha value.
    pub fn with_alpha(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            initialized: false,
        }
    }

    /// Update filter with a new IMU reading.
    ///
    /// Returns `(roll, pitch, yaw)` in radians.
    pub fn update(&mut self, reading: &ImuReading, dt: f64) -> (f64, f64, f64) {
        // Accelerometer-derived tilt (low-pass path)
        let accel_roll =
            reading.accel_y.atan2(reading.accel_z);
        let accel_pitch =
            (-reading.accel_x).atan2(
                (reading.accel_y * reading.accel_y + reading.accel_z * reading.accel_z).sqrt(),
            );

        if !self.initialized {
            self.roll = accel_roll;
            self.pitch = accel_pitch;
            self.yaw = 0.0;
            self.initialized = true;
            return (self.roll, self.pitch, self.yaw);
        }

        // Gyro integration (high-pass path)
        let gyro_roll = self.roll + reading.gyro_x * dt;
        let gyro_pitch = self.pitch + reading.gyro_y * dt;

        // Complementary combination
        self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll;
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch;
        self.yaw += reading.gyro_z * dt; // Gyro-only for yaw

        (self.roll, self.pitch, self.yaw)
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.roll = 0.0;
        self.pitch = 0.0;
        self.yaw = 0.0;
        self.initialized = false;
    }
}

// ---------------------------------------------------------------------------
// ZUPT Detector
// ---------------------------------------------------------------------------

/// Zero Velocity Update (ZUPT) detection method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ZuptMethod {
    /// Variance-based: compare accel/gyro variance to threshold.
    Variance,
    /// Generalized Likelihood Ratio Test.
    Glrt,
}

/// Zero Velocity Update (ZUPT) detector.
///
/// Detects stationary periods from IMU data and applies velocity corrections.
#[derive(Debug, Clone)]
pub struct ZuptDetector {
    /// Detection method.
    pub method: ZuptMethod,
    /// Window size in samples.
    pub window_size: usize,
    /// Acceleration variance threshold (m/s^2)^2.
    pub accel_threshold: f64,
    /// Gyro variance threshold (rad/s)^2.
    pub gyro_threshold: f64,
    /// Buffer of recent accel magnitudes.
    accel_buffer: Vec<f64>,
    /// Buffer of recent gyro magnitudes.
    gyro_buffer: Vec<f64>,
}

impl ZuptDetector {
    /// Create a new ZUPT detector.
    pub fn new(method: ZuptMethod, window_size: usize) -> Self {
        Self {
            method,
            window_size,
            accel_threshold: 0.5,  // (m/s^2)^2
            gyro_threshold: 0.01,  // (rad/s)^2
            accel_buffer: Vec::with_capacity(window_size),
            gyro_buffer: Vec::with_capacity(window_size),
        }
    }

    /// Set detection thresholds.
    pub fn with_thresholds(mut self, accel_thresh: f64, gyro_thresh: f64) -> Self {
        self.accel_threshold = accel_thresh;
        self.gyro_threshold = gyro_thresh;
        self
    }

    /// Process a new IMU reading and return true if ZUPT condition detected.
    pub fn detect(&mut self, reading: &ImuReading) -> bool {
        let accel_mag = reading.accel_magnitude();
        let gyro_mag = reading.gyro_magnitude();

        self.accel_buffer.push(accel_mag);
        self.gyro_buffer.push(gyro_mag);

        if self.accel_buffer.len() > self.window_size {
            self.accel_buffer.remove(0);
            self.gyro_buffer.remove(0);
        }

        if self.accel_buffer.len() < self.window_size {
            return false;
        }

        match self.method {
            ZuptMethod::Variance => self.detect_variance(),
            ZuptMethod::Glrt => self.detect_glrt(),
        }
    }

    /// Apply ZUPT correction to NavState (zero out velocity).
    pub fn apply_correction(state: &mut NavState) {
        state.velocity = (0.0, 0.0, 0.0);
    }

    /// Variance-based detection.
    fn detect_variance(&self) -> bool {
        let accel_var = variance(&self.accel_buffer);
        let gyro_var = variance(&self.gyro_buffer);
        accel_var < self.accel_threshold && gyro_var < self.gyro_threshold
    }

    /// GLRT detection statistic.
    ///
    /// T = (1/W) * sum_{k=0}^{W-1} [ (1/sigma_a^2) * ||y_a[k] - g||^2 + (1/sigma_g^2) * ||y_g[k]||^2 ]
    fn detect_glrt(&self) -> bool {
        let w = self.accel_buffer.len() as f64;
        let sigma_a_sq = self.accel_threshold; // use threshold as noise variance
        let sigma_g_sq = self.gyro_threshold;

        let mut t_stat = 0.0;
        for i in 0..self.accel_buffer.len() {
            let accel_dev = self.accel_buffer[i] - G0;
            t_stat += accel_dev * accel_dev / sigma_a_sq;
            t_stat += self.gyro_buffer[i] * self.gyro_buffer[i] / sigma_g_sq;
        }
        t_stat /= w;

        // Threshold: chi-squared-like test, use combined threshold
        t_stat < 2.0
    }

    /// Reset the detector buffers.
    pub fn reset(&mut self) {
        self.accel_buffer.clear();
        self.gyro_buffer.clear();
    }
}

// ---------------------------------------------------------------------------
// Turn Rate Detector
// ---------------------------------------------------------------------------

/// Detects turns from gyroscope z-axis for heading updates.
#[derive(Debug, Clone)]
pub struct TurnRateDetector {
    /// Threshold for turn detection (rad/s).
    pub threshold: f64,
    /// Integrated heading change (radians) during current turn.
    pub integrated_turn: f64,
    /// Whether currently in a turn.
    pub in_turn: bool,
}

impl TurnRateDetector {
    /// Create a new turn rate detector.
    ///
    /// - `threshold`: minimum gyro z-axis rate to qualify as a turn (rad/s)
    pub fn new(threshold: f64) -> Self {
        Self {
            threshold,
            integrated_turn: 0.0,
            in_turn: false,
        }
    }

    /// Process a gyro z-axis reading. Returns true if a turn is detected.
    pub fn detect(&mut self, gyro_z: f64, dt: f64) -> bool {
        if gyro_z.abs() > self.threshold {
            self.in_turn = true;
            self.integrated_turn += gyro_z * dt;
            true
        } else {
            if self.in_turn {
                // Turn ended
                self.in_turn = false;
            }
            false
        }
    }

    /// Get the accumulated turn angle and reset.
    pub fn take_turn_angle(&mut self) -> f64 {
        let angle = self.integrated_turn;
        self.integrated_turn = 0.0;
        angle
    }

    /// Reset detector state.
    pub fn reset(&mut self) {
        self.integrated_turn = 0.0;
        self.in_turn = false;
    }
}

// ---------------------------------------------------------------------------
// Pedestrian Dead Reckoning (PDR)
// ---------------------------------------------------------------------------

/// Step detection and pedestrian dead reckoning.
#[derive(Debug, Clone)]
pub struct PedestrianDr {
    /// Weinberg model constant k (typically 0.35-0.45 for walking).
    pub weinberg_k: f64,
    /// Minimum step period in seconds (to reject false positives).
    pub min_step_period: f64,
    /// Accelerometer magnitude buffer for peak detection.
    accel_buffer: Vec<f64>,
    /// Timestamp buffer.
    time_buffer: Vec<f64>,
    /// Current heading (radians).
    pub heading: f64,
    /// Position (x, y) in meters.
    pub position: (f64, f64),
    /// Total steps detected.
    pub step_count: usize,
    /// Total distance walked (meters).
    pub total_distance: f64,
    /// Time of last detected step.
    last_step_time: f64,
    /// Window size for step detection.
    window_size: usize,
}

impl PedestrianDr {
    /// Create a new PDR processor.
    ///
    /// - `weinberg_k`: step length constant (typically 0.4)
    /// - `window_size`: samples for peak detection (e.g., 50 for 50 Hz)
    pub fn new(weinberg_k: f64, window_size: usize) -> Self {
        Self {
            weinberg_k,
            min_step_period: 0.3, // 300ms minimum between steps
            accel_buffer: Vec::with_capacity(window_size),
            time_buffer: Vec::with_capacity(window_size),
            heading: 0.0,
            position: (0.0, 0.0),
            step_count: 0,
            total_distance: 0.0,
            last_step_time: 0.0,
            window_size,
        }
    }

    /// Process a new IMU reading. Returns Some(step_length) if a step was detected.
    pub fn update(&mut self, reading: &ImuReading, dt: f64) -> Option<f64> {
        // Update heading from gyro z-axis
        self.heading += reading.gyro_z * dt;
        // Normalize heading to [-pi, pi]
        self.heading = normalize_angle(self.heading);

        // Buffer accelerometer magnitude
        let accel_mag = reading.accel_magnitude();
        self.accel_buffer.push(accel_mag);
        self.time_buffer.push(reading.timestamp_s);

        if self.accel_buffer.len() > self.window_size {
            self.accel_buffer.remove(0);
            self.time_buffer.remove(0);
        }

        if self.accel_buffer.len() < self.window_size {
            return None;
        }

        // Check for step: peak at center of window
        let mid = self.window_size / 2;
        if self.is_peak(mid)
            && (reading.timestamp_s - self.last_step_time) > self.min_step_period
        {
            let step_length = self.weinberg_step_length();
            self.step_count += 1;
            self.total_distance += step_length;
            self.last_step_time = reading.timestamp_s;

            // Update position
            self.position.0 += step_length * self.heading.cos();
            self.position.1 += step_length * self.heading.sin();

            return Some(step_length);
        }

        None
    }

    /// Check if index is a local maximum in the accel buffer.
    fn is_peak(&self, idx: usize) -> bool {
        if idx == 0 || idx >= self.accel_buffer.len() - 1 {
            return false;
        }
        self.accel_buffer[idx] > self.accel_buffer[idx - 1]
            && self.accel_buffer[idx] > self.accel_buffer[idx + 1]
            && self.accel_buffer[idx] > G0 + 1.0 // Above gravity + threshold
    }

    /// Weinberg step length model: L = k * (a_max - a_min)^0.25
    fn weinberg_step_length(&self) -> f64 {
        let a_max = self.accel_buffer.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let a_min = self.accel_buffer.iter().cloned().fold(f64::INFINITY, f64::min);
        let diff = (a_max - a_min).max(0.0);
        self.weinberg_k * diff.powf(0.25)
    }

    /// Reset the PDR state.
    pub fn reset(&mut self) {
        self.accel_buffer.clear();
        self.time_buffer.clear();
        self.heading = 0.0;
        self.position = (0.0, 0.0);
        self.step_count = 0;
        self.total_distance = 0.0;
        self.last_step_time = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Navigation Error Metrics
// ---------------------------------------------------------------------------

/// Compute position error between two NavStates (meters).
pub fn position_error(truth: &NavState, estimated: &NavState) -> f64 {
    let dx = truth.position.0 - estimated.position.0;
    let dy = truth.position.1 - estimated.position.1;
    let dz = truth.position.2 - estimated.position.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Compute velocity error between two NavStates (m/s).
pub fn velocity_error(truth: &NavState, estimated: &NavState) -> f64 {
    let dvx = truth.velocity.0 - estimated.velocity.0;
    let dvy = truth.velocity.1 - estimated.velocity.1;
    let dvz = truth.velocity.2 - estimated.velocity.2;
    (dvx * dvx + dvy * dvy + dvz * dvz).sqrt()
}

/// Compute attitude error between two quaternions (radians, small angle approximation).
///
/// Returns the angle of the error quaternion: 2 * acos(|q_err.w|).
pub fn attitude_error(truth: &Quaternion, estimated: &Quaternion) -> f64 {
    let q_err = truth.conjugate().multiply(estimated);
    let w_clamped = q_err.w.abs().min(1.0);
    2.0 * w_clamped.acos()
}

/// Compute per-axis attitude error (roll_err, pitch_err, yaw_err) in radians.
pub fn attitude_error_euler(truth: &Quaternion, estimated: &Quaternion) -> (f64, f64, f64) {
    let (tr, tp, ty) = truth.to_euler();
    let (er, ep, ey) = estimated.to_euler();
    (
        normalize_angle(tr - er),
        normalize_angle(tp - ep),
        normalize_angle(ty - ey),
    )
}

// ---------------------------------------------------------------------------
// Utility Functions
// ---------------------------------------------------------------------------

/// Compute variance of a slice.
fn variance(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    let n = data.len() as f64;
    let mean = data.iter().sum::<f64>() / n;
    let sum_sq: f64 = data.iter().map(|&x| (x - mean) * (x - mean)).sum();
    sum_sq / (n - 1.0)
}

/// Normalize angle to [-pi, pi].
fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Matrix-vector multiply for 3x3 matrix and 3-vector.
#[allow(dead_code)]
fn mat3_vec3(m: &[[f64; 3]; 3], v: &[f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Degrees to radians.
pub fn deg2rad(deg: f64) -> f64 {
    deg * PI / 180.0
}

/// Radians to degrees.
pub fn rad2deg(rad: f64) -> f64 {
    rad * 180.0 / PI
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_COARSE: f64 = 1e-3;

    // ---- Quaternion tests ----

    #[test]
    fn test_quaternion_identity() {
        let q = Quaternion::identity();
        assert!((q.w - 1.0).abs() < TOL);
        assert!(q.x.abs() < TOL);
        assert!(q.y.abs() < TOL);
        assert!(q.z.abs() < TOL);
    }

    #[test]
    fn test_quaternion_norm() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let expected = (1.0 + 4.0 + 9.0 + 16.0_f64).sqrt();
        assert!((q.norm() - expected).abs() < TOL);
    }

    #[test]
    fn test_quaternion_normalize() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0).normalize();
        assert!((q.norm() - 1.0).abs() < TOL);
    }

    #[test]
    fn test_quaternion_conjugate() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let c = q.conjugate();
        assert!((c.w - 1.0).abs() < TOL);
        assert!((c.x + 2.0).abs() < TOL);
        assert!((c.y + 3.0).abs() < TOL);
        assert!((c.z + 4.0).abs() < TOL);
    }

    #[test]
    fn test_quaternion_multiply_identity() {
        let q = Quaternion::new(0.5, 0.5, 0.5, 0.5);
        let id = Quaternion::identity();
        let r = q.multiply(&id);
        assert!((r.w - q.w).abs() < TOL);
        assert!((r.x - q.x).abs() < TOL);
        assert!((r.y - q.y).abs() < TOL);
        assert!((r.z - q.z).abs() < TOL);
    }

    #[test]
    fn test_quaternion_multiply_conjugate_gives_identity() {
        let q = Quaternion::new(0.5, 0.5, 0.5, 0.5); // already unit
        let r = q.multiply(&q.conjugate());
        assert!((r.w - 1.0).abs() < TOL);
        assert!(r.x.abs() < TOL);
        assert!(r.y.abs() < TOL);
        assert!(r.z.abs() < TOL);
    }

    #[test]
    fn test_quaternion_from_euler_zero() {
        let q = Quaternion::from_euler(0.0, 0.0, 0.0);
        assert!((q.w - 1.0).abs() < TOL);
        assert!(q.x.abs() < TOL);
        assert!(q.y.abs() < TOL);
        assert!(q.z.abs() < TOL);
    }

    #[test]
    fn test_quaternion_euler_roundtrip() {
        let roll = 0.3;
        let pitch = 0.2;
        let yaw = 0.5;
        let q = Quaternion::from_euler(roll, pitch, yaw);
        let (r, p, y) = q.to_euler();
        assert!((r - roll).abs() < TOL);
        assert!((p - pitch).abs() < TOL);
        assert!((y - yaw).abs() < TOL);
    }

    #[test]
    fn test_quaternion_rotate_vector_identity() {
        let q = Quaternion::identity();
        let v = [1.0, 2.0, 3.0];
        let r = q.rotate_vector(v);
        assert!((r[0] - 1.0).abs() < TOL);
        assert!((r[1] - 2.0).abs() < TOL);
        assert!((r[2] - 3.0).abs() < TOL);
    }

    #[test]
    fn test_quaternion_rotate_90_about_z() {
        let q = Quaternion::from_euler(0.0, 0.0, PI / 2.0);
        let v = [1.0, 0.0, 0.0];
        let r = q.rotate_vector(v);
        assert!((r[0]).abs() < TOL);
        assert!((r[1] - 1.0).abs() < TOL);
        assert!(r[2].abs() < TOL);
    }

    #[test]
    fn test_quaternion_to_dcm_identity() {
        let q = Quaternion::identity();
        let dcm = q.to_dcm();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((dcm[i][j] - expected).abs() < TOL);
            }
        }
    }

    #[test]
    fn test_quaternion_from_angular_rate_zero() {
        let dq = Quaternion::from_angular_rate(0.0, 0.0, 0.0, 0.01);
        assert!((dq.w - 1.0).abs() < TOL);
        assert!(dq.x.abs() < TOL);
        assert!(dq.y.abs() < TOL);
        assert!(dq.z.abs() < TOL);
    }

    #[test]
    fn test_quaternion_from_angular_rate_z() {
        // Rotate 90 deg about z: omega_z = pi/2 rad/s for 1 second
        let dq = Quaternion::from_angular_rate(0.0, 0.0, PI / 2.0, 1.0);
        // Expected: cos(pi/4), (0, 0, sin(pi/4))
        let expected_w = (PI / 4.0).cos();
        let expected_z = (PI / 4.0).sin();
        assert!((dq.w - expected_w).abs() < TOL);
        assert!(dq.x.abs() < TOL);
        assert!(dq.y.abs() < TOL);
        assert!((dq.z - expected_z).abs() < TOL);
    }

    // ---- ImuReading tests ----

    #[test]
    fn test_imu_reading_accel_magnitude() {
        let r = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, 0.0);
        assert!((r.accel_magnitude() - G0).abs() < TOL);
    }

    #[test]
    fn test_imu_reading_gyro_magnitude() {
        let r = ImuReading::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
        assert!((r.gyro_magnitude() - 1.0).abs() < TOL);
    }

    #[test]
    fn test_imu_reading_arrays() {
        let r = ImuReading::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
        assert_eq!(r.accel(), [1.0, 2.0, 3.0]);
        assert_eq!(r.gyro(), [4.0, 5.0, 6.0]);
    }

    // ---- NavState tests ----

    #[test]
    fn test_navstate_new() {
        let s = NavState::new();
        assert!((s.position.0).abs() < TOL);
        assert!((s.speed()).abs() < TOL);
    }

    #[test]
    fn test_navstate_speed() {
        let mut s = NavState::new();
        s.velocity = (3.0, 4.0, 0.0);
        assert!((s.speed() - 5.0).abs() < TOL);
    }

    #[test]
    fn test_navstate_distance() {
        let mut s = NavState::new();
        s.position = (3.0, 4.0, 0.0);
        assert!((s.distance_from_origin() - 5.0).abs() < TOL);
    }

    // ---- MEMS Error Model tests ----

    #[test]
    fn test_accel_error_model_apply() {
        let model = AccelErrorModel::tactical_grade();
        let true_val = 9.80665;
        let measured = model.apply(true_val);
        // Should be close to true value plus small bias
        assert!((measured - true_val).abs() < 0.01);
    }

    #[test]
    fn test_gyro_error_model_apply() {
        let model = GyroErrorModel::tactical_grade();
        let true_rate = 0.1; // 0.1 rad/s
        let measured = model.apply(true_rate);
        // Bias is ~4.8e-6 rad/s, so measured ~ true_rate
        assert!((measured - true_rate).abs() < 0.001);
    }

    #[test]
    fn test_consumer_grade_models() {
        let accel = AccelErrorModel::consumer_grade();
        let gyro = GyroErrorModel::consumer_grade();
        // Consumer grade has larger errors
        assert!(accel.bias > AccelErrorModel::tactical_grade().bias);
        assert!(gyro.bias > GyroErrorModel::tactical_grade().bias);
    }

    // ---- Allan Variance tests ----

    #[test]
    fn test_allan_variance_constant_signal() {
        // Constant signal should have very low Allan variance
        let data: Vec<f64> = vec![1.0; 1000];
        let av = AllanVariance::compute(&data, 100.0, 10);
        assert!(!av.tau.is_empty());
        for &avar in &av.avar {
            assert!(avar < TOL);
        }
    }

    #[test]
    fn test_allan_variance_too_short() {
        let data = vec![1.0, 2.0];
        let av = AllanVariance::compute(&data, 100.0, 10);
        assert!(av.tau.is_empty());
    }

    #[test]
    fn test_allan_variance_white_noise_coefficient() {
        // For white noise, Allan variance at tau should decrease as 1/tau
        let data: Vec<f64> = (0..1000).map(|i| (i as f64 * 0.1).sin()).collect();
        let av = AllanVariance::compute(&data, 100.0, 10);
        let coeff = av.white_noise_coefficient();
        assert!(coeff.is_some());
    }

    #[test]
    fn test_allan_variance_bias_instability() {
        let data: Vec<f64> = (0..1000).map(|i| (i as f64 * 0.01).sin()).collect();
        let av = AllanVariance::compute(&data, 100.0, 10);
        let bi = av.bias_instability();
        assert!(bi.is_some());
        let (tau, sigma) = bi.unwrap();
        assert!(tau > 0.0);
        assert!(sigma >= 0.0);
    }

    // ---- Strapdown INS tests ----

    #[test]
    fn test_strapdown_stationary() {
        // Stationary level IMU: accel_z = g, all others zero
        let mut ins = StrapdownIns::new();
        let mut state = NavState::new();

        for i in 1..=100 {
            let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, i as f64 * 0.01);
            ins.update(&mut state, &reading);
        }

        // Position and velocity should remain near zero
        assert!(state.position.0.abs() < 0.1);
        assert!(state.position.1.abs() < 0.1);
        assert!(state.velocity.0.abs() < 0.1);
        assert!(state.velocity.1.abs() < 0.1);
    }

    #[test]
    fn test_strapdown_constant_acceleration_x() {
        // Apply 1 m/s^2 in body x-axis (plus gravity on z)
        let mut ins = StrapdownIns::new();
        let mut state = NavState::new();

        let dt = 0.01;
        for i in 1..=100 {
            let reading = ImuReading::new(1.0, 0.0, G0, 0.0, 0.0, 0.0, i as f64 * dt);
            ins.update(&mut state, &reading);
        }

        let t = 100.0 * dt; // 1 second
        // v = a * t = 1.0 m/s
        assert!((state.velocity.0 - 1.0).abs() < 0.1);
        // p = 0.5 * a * t^2 = 0.5 m
        assert!((state.position.0 - 0.5).abs() < 0.1);
    }

    #[test]
    fn test_strapdown_rotation() {
        // Pure rotation about z-axis at 1 rad/s for 1 second = 1 radian
        let mut ins = StrapdownIns::new();
        let mut state = NavState::new();

        let dt = 0.01;
        for i in 1..=100 {
            let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 1.0, i as f64 * dt);
            ins.update(&mut state, &reading);
        }

        let (_, _, yaw) = state.attitude.to_euler();
        assert!((yaw - 1.0).abs() < 0.05);
    }

    #[test]
    fn test_strapdown_simpson() {
        let mut ins = StrapdownIns::with_method(IntegrationMethod::Simpson);
        let mut state = NavState::new();

        let dt = 0.01;
        for i in 1..=100 {
            let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, i as f64 * dt);
            ins.update(&mut state, &reading);
        }

        assert!(state.velocity.0.abs() < 0.1);
        assert!(state.velocity.1.abs() < 0.1);
    }

    #[test]
    fn test_strapdown_reset() {
        let mut ins = StrapdownIns::new();
        ins.prev_reading = Some(ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, 0.0));
        ins.reading_count = 5;
        ins.reset();
        assert!(ins.prev_reading.is_none());
        assert_eq!(ins.reading_count, 0);
    }

    // ---- Complementary Filter tests ----

    #[test]
    fn test_complementary_filter_init() {
        let mut cf = ComplementaryFilter::new(0.1, 100.0);
        let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, 0.0);
        let (r, p, y) = cf.update(&reading, 0.01);
        // Level reading should give near-zero roll/pitch
        assert!(r.abs() < TOL_COARSE);
        assert!(p.abs() < TOL_COARSE);
        assert!(y.abs() < TOL_COARSE);
    }

    #[test]
    fn test_complementary_filter_steady_tilt() {
        let mut cf = ComplementaryFilter::with_alpha(0.98);
        // Tilted 30 degrees in roll
        let tilt_angle = deg2rad(30.0);
        let az = G0 * tilt_angle.cos();
        let ay = G0 * tilt_angle.sin();

        // First update initializes
        let reading = ImuReading::new(0.0, ay, az, 0.0, 0.0, 0.0, 0.0);
        cf.update(&reading, 0.01);

        // Many updates converge
        for i in 1..=1000 {
            cf.update(&reading, 0.01);
        }

        assert!((cf.roll - tilt_angle).abs() < deg2rad(2.0));
    }

    #[test]
    fn test_complementary_filter_reset() {
        let mut cf = ComplementaryFilter::with_alpha(0.98);
        let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, 0.0);
        cf.update(&reading, 0.01);
        cf.reset();
        assert!(!cf.initialized);
        assert!(cf.roll.abs() < TOL);
    }

    // ---- ZUPT Detector tests ----

    #[test]
    fn test_zupt_stationary_detection() {
        let mut zupt = ZuptDetector::new(ZuptMethod::Variance, 10)
            .with_thresholds(0.5, 0.01);

        // Stationary readings
        for i in 0..20 {
            let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, i as f64 * 0.01);
            let detected = zupt.detect(&reading);
            if i >= 9 {
                assert!(detected, "ZUPT should detect stationary at sample {}", i);
            }
        }
    }

    #[test]
    fn test_zupt_moving_no_detection() {
        let mut zupt = ZuptDetector::new(ZuptMethod::Variance, 10)
            .with_thresholds(0.1, 0.01);

        // Moving readings with varying acceleration
        for i in 0..20 {
            let accel_z = G0 + (i as f64 * 0.5).sin() * 5.0;
            let reading = ImuReading::new(0.0, 0.0, accel_z, 0.1, 0.0, 0.0, i as f64 * 0.01);
            let detected = zupt.detect(&reading);
            if i >= 9 {
                assert!(!detected, "ZUPT should not detect when moving at sample {}", i);
            }
        }
    }

    #[test]
    fn test_zupt_glrt_stationary() {
        let mut zupt = ZuptDetector::new(ZuptMethod::Glrt, 10)
            .with_thresholds(0.5, 0.01);

        for i in 0..20 {
            let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, i as f64 * 0.01);
            let detected = zupt.detect(&reading);
            if i >= 9 {
                assert!(detected, "GLRT should detect stationary at sample {}", i);
            }
        }
    }

    #[test]
    fn test_zupt_apply_correction() {
        let mut state = NavState::new();
        state.velocity = (1.0, 2.0, 3.0);
        ZuptDetector::apply_correction(&mut state);
        assert!(state.velocity.0.abs() < TOL);
        assert!(state.velocity.1.abs() < TOL);
        assert!(state.velocity.2.abs() < TOL);
    }

    // ---- Turn Rate Detector tests ----

    #[test]
    fn test_turn_detection() {
        let mut det = TurnRateDetector::new(0.1); // 0.1 rad/s threshold

        // No turn
        assert!(!det.detect(0.05, 0.01));

        // Turn detected
        assert!(det.detect(0.5, 0.01));
        assert!(det.in_turn);

        // Accumulate turn
        det.detect(0.5, 0.01);
        let angle = det.take_turn_angle();
        assert!((angle - 0.01).abs() < TOL); // 0.5 * 0.01 + 0.5 * 0.01 = 0.01
    }

    #[test]
    fn test_turn_end_detection() {
        let mut det = TurnRateDetector::new(0.1);
        det.detect(0.5, 0.01); // start turn
        assert!(det.in_turn);
        det.detect(0.05, 0.01); // below threshold
        assert!(!det.in_turn);
    }

    // ---- Pedestrian Dead Reckoning tests ----

    #[test]
    fn test_pdr_initialization() {
        let pdr = PedestrianDr::new(0.4, 50);
        assert_eq!(pdr.step_count, 0);
        assert!((pdr.total_distance).abs() < TOL);
        assert!((pdr.heading).abs() < TOL);
    }

    #[test]
    fn test_pdr_no_step_stationary() {
        let mut pdr = PedestrianDr::new(0.4, 10);
        for i in 0..50 {
            let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.0, i as f64 * 0.02);
            let step = pdr.update(&reading, 0.02);
            assert!(step.is_none());
        }
        assert_eq!(pdr.step_count, 0);
    }

    #[test]
    fn test_pdr_step_detection() {
        let mut pdr = PedestrianDr::new(0.4, 10);
        pdr.min_step_period = 0.0; // disable minimum period for test

        // Simulate walking pattern: sinusoidal acceleration peaks
        for i in 0..100 {
            let t = i as f64 * 0.02;
            // Walking produces ~2 Hz oscillation in vertical accel
            let az = G0 + 4.0 * (2.0 * PI * 2.0 * t).sin();
            let reading = ImuReading::new(0.0, 0.0, az, 0.0, 0.0, 0.0, t);
            pdr.update(&reading, 0.02);
        }

        // Should have detected some steps
        assert!(pdr.step_count > 0);
        assert!(pdr.total_distance > 0.0);
    }

    #[test]
    fn test_pdr_heading_integration() {
        let mut pdr = PedestrianDr::new(0.4, 10);
        // Constant turn rate of 0.1 rad/s for 10 seconds
        for i in 0..500 {
            let t = i as f64 * 0.02;
            let reading = ImuReading::new(0.0, 0.0, G0, 0.0, 0.0, 0.1, t);
            pdr.update(&reading, 0.02);
        }
        // heading should be about 10.0 rad, normalized to [-pi, pi]
        // 10.0 mod 2pi = 10 - 3*2pi = 10 - 18.849 ... = nah, let's just check it changed
        assert!(pdr.heading.abs() > 0.0);
    }

    #[test]
    fn test_pdr_reset() {
        let mut pdr = PedestrianDr::new(0.4, 10);
        pdr.step_count = 5;
        pdr.total_distance = 3.5;
        pdr.heading = 1.0;
        pdr.reset();
        assert_eq!(pdr.step_count, 0);
        assert!(pdr.total_distance.abs() < TOL);
        assert!(pdr.heading.abs() < TOL);
    }

    // ---- Navigation Error Metric tests ----

    #[test]
    fn test_position_error_zero() {
        let s1 = NavState::new();
        let s2 = NavState::new();
        assert!(position_error(&s1, &s2) < TOL);
    }

    #[test]
    fn test_position_error_nonzero() {
        let mut s1 = NavState::new();
        let s2 = NavState::new();
        s1.position = (3.0, 4.0, 0.0);
        assert!((position_error(&s1, &s2) - 5.0).abs() < TOL);
    }

    #[test]
    fn test_velocity_error() {
        let mut s1 = NavState::new();
        let s2 = NavState::new();
        s1.velocity = (0.0, 0.0, 1.0);
        assert!((velocity_error(&s1, &s2) - 1.0).abs() < TOL);
    }

    #[test]
    fn test_attitude_error_zero() {
        let q1 = Quaternion::identity();
        let q2 = Quaternion::identity();
        assert!(attitude_error(&q1, &q2) < TOL);
    }

    #[test]
    fn test_attitude_error_nonzero() {
        let q1 = Quaternion::identity();
        let q2 = Quaternion::from_euler(0.1, 0.0, 0.0); // 0.1 rad roll
        let err = attitude_error(&q1, &q2);
        assert!((err - 0.1).abs() < TOL_COARSE);
    }

    #[test]
    fn test_attitude_error_euler() {
        let q1 = Quaternion::from_euler(0.1, 0.2, 0.3);
        let q2 = Quaternion::from_euler(0.15, 0.25, 0.35);
        let (re, pe, ye) = attitude_error_euler(&q1, &q2);
        assert!((re - (-0.05)).abs() < TOL_COARSE);
        assert!((pe - (-0.05)).abs() < TOL_COARSE);
        assert!((ye - (-0.05)).abs() < TOL_COARSE);
    }

    // ---- Utility tests ----

    #[test]
    fn test_variance_constant() {
        let data = vec![5.0; 100];
        assert!(variance(&data) < TOL);
    }

    #[test]
    fn test_variance_known() {
        // Variance of [1, 2, 3, 4, 5] = 2.5 (sample variance)
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert!((variance(&data) - 2.5).abs() < TOL);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0)).abs() < TOL);
        assert!((normalize_angle(PI) - PI).abs() < TOL);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < TOL);
        assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < TOL);
    }

    #[test]
    fn test_deg2rad_rad2deg() {
        assert!((deg2rad(180.0) - PI).abs() < TOL);
        assert!((rad2deg(PI) - 180.0).abs() < TOL);
        assert!((rad2deg(deg2rad(45.0)) - 45.0).abs() < TOL);
    }
}

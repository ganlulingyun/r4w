//! MEMS Inertial Measurement Unit (IMU) signal processing for navigation,
//! stabilization, and motion tracking.
//!
//! This module provides comprehensive signal processing for MEMS-based IMUs
//! (accelerometers and gyroscopes), including calibration, sensor fusion,
//! orientation estimation, and motion analysis. Key capabilities:
//!
//! - **Sensor Configuration**: Accelerometer (±2/4/8/16g) and gyroscope
//!   (±250/500/1000/2000 dps) range selection, noise characterization
//! - **Calibration**: 6-position accelerometer calibration (scale, bias, cross-axis),
//!   static gyroscope bias estimation, temperature compensation
//! - **Orientation Filters**: Complementary filter (alpha-tunable accel+gyro fusion),
//!   Madgwick gradient descent AHRS (quaternion output, configurable beta)
//! - **Quaternion Math**: Hamilton product, quaternion ↔ Euler conversion,
//!   rotation matrix extraction, normalization
//! - **Navigation**: Dead reckoning (double integration with drift warning),
//!   gyroscope angular integration (trapezoidal), gravity removal
//! - **Noise Analysis**: Allan variance/deviation for bias instability,
//!   spectral noise density to RMS conversion, vibration rectification error
//! - **Motion Detection**: Step detection pedometer (peak detection with
//!   refractory period), heading from magnetometer with tilt compensation
//! - **Utility**: Low-pass IIR filter, sensor fusion weight computation
//!
//! # Example
//!
//! ```
//! use r4w_core::mems_inertial_measurement_unit::{
//!     ImuConfig, AccelRange, GyroRange, ImuProcessor, ImuSample,
//! };
//!
//! // Configure a 6-DOF IMU at 100 Hz
//! let config = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
//! let mut proc = ImuProcessor::new(config);
//!
//! // A stationary sample (gravity along +Z)
//! let sample = ImuSample {
//!     accel: [0.0, 0.0, 9.80665],
//!     gyro: [0.0, 0.0, 0.0],
//!     mag: None,
//!     timestamp_s: 0.0,
//! };
//!
//! let (pitch, roll) = ImuProcessor::tilt_from_accel(&sample.accel);
//! assert!(pitch.abs() < 0.01);
//! assert!(roll.abs() < 0.01);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Standard gravity at sea level (m/s^2), WGS-84.
const G0: f64 = 9.806_65;

/// Degrees-to-radians conversion factor.
const DEG_TO_RAD: f64 = PI / 180.0;

/// Radians-to-degrees conversion factor.
const RAD_TO_DEG: f64 = 180.0 / PI;

// ---------------------------------------------------------------------------
// Accelerometer range enumeration
// ---------------------------------------------------------------------------

/// Accelerometer full-scale range selection.
///
/// Typical MEMS accelerometer ranges from consumer (±2g) to industrial (±16g).
/// Higher range reduces sensitivity but allows measuring larger accelerations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AccelRange {
    /// ±2g full scale (highest sensitivity, ~16384 LSB/g for 16-bit ADC)
    G2,
    /// ±4g full scale (~8192 LSB/g)
    G4,
    /// ±8g full scale (~4096 LSB/g)
    G8,
    /// ±16g full scale (lowest sensitivity, ~2048 LSB/g)
    G16,
}

impl AccelRange {
    /// Return the full-scale range in m/s^2.
    pub fn full_scale_ms2(&self) -> f64 {
        match self {
            AccelRange::G2 => 2.0 * G0,
            AccelRange::G4 => 4.0 * G0,
            AccelRange::G8 => 8.0 * G0,
            AccelRange::G16 => 16.0 * G0,
        }
    }

    /// Return the full-scale range in g.
    pub fn full_scale_g(&self) -> f64 {
        match self {
            AccelRange::G2 => 2.0,
            AccelRange::G4 => 4.0,
            AccelRange::G8 => 8.0,
            AccelRange::G16 => 16.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Gyroscope range enumeration
// ---------------------------------------------------------------------------

/// Gyroscope full-scale range selection.
///
/// Typical MEMS gyroscope ranges. Higher range reduces angular rate
/// sensitivity but can measure faster rotations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GyroRange {
    /// ±250 degrees per second
    Dps250,
    /// ±500 degrees per second
    Dps500,
    /// ±1000 degrees per second
    Dps1000,
    /// ±2000 degrees per second
    Dps2000,
}

impl GyroRange {
    /// Return the full-scale range in degrees per second.
    pub fn full_scale_dps(&self) -> f64 {
        match self {
            GyroRange::Dps250 => 250.0,
            GyroRange::Dps500 => 500.0,
            GyroRange::Dps1000 => 1000.0,
            GyroRange::Dps2000 => 2000.0,
        }
    }

    /// Return the full-scale range in radians per second.
    pub fn full_scale_rps(&self) -> f64 {
        self.full_scale_dps() * DEG_TO_RAD
    }
}

// ---------------------------------------------------------------------------
// Axis count / DOF
// ---------------------------------------------------------------------------

/// Degrees of freedom for the IMU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AxisCount {
    /// 6-DOF: 3-axis accelerometer + 3-axis gyroscope
    Dof6,
    /// 9-DOF: 6-DOF + 3-axis magnetometer
    Dof9,
}

// ---------------------------------------------------------------------------
// ImuConfig
// ---------------------------------------------------------------------------

/// Configuration for a MEMS inertial measurement unit.
///
/// Describes sensor ranges, sample rate, noise characteristics, and
/// degree-of-freedom count.
#[derive(Debug, Clone)]
pub struct ImuConfig {
    /// Accelerometer full-scale range.
    pub accel_range: AccelRange,
    /// Gyroscope full-scale range.
    pub gyro_range: GyroRange,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Accelerometer noise density in µg/√Hz (e.g., 100 for MPU-6050).
    pub accel_noise_density_ug: f64,
    /// Gyroscope noise density in °/s/√Hz (e.g., 0.005 for MPU-6050).
    pub gyro_noise_density_dps: f64,
    /// Accelerometer bias stability in µg (e.g., 500 for consumer MEMS).
    pub accel_bias_stability_ug: f64,
    /// Gyroscope bias stability in °/hr (e.g., 10 for consumer MEMS).
    pub gyro_bias_stability_dph: f64,
    /// Degrees of freedom.
    pub axis_count: AxisCount,
}

impl ImuConfig {
    /// Create a new IMU configuration with the given ranges and sample rate.
    ///
    /// Noise and bias parameters default to typical consumer MEMS values
    /// (similar to MPU-6050 / ICM-20948).
    pub fn new(accel_range: AccelRange, gyro_range: GyroRange, sample_rate: f64) -> Self {
        Self {
            accel_range,
            gyro_range,
            sample_rate,
            accel_noise_density_ug: 100.0,
            gyro_noise_density_dps: 0.005,
            accel_bias_stability_ug: 500.0,
            gyro_bias_stability_dph: 10.0,
            axis_count: AxisCount::Dof6,
        }
    }

    /// Create a 9-DOF configuration (with magnetometer).
    pub fn new_9dof(accel_range: AccelRange, gyro_range: GyroRange, sample_rate: f64) -> Self {
        let mut cfg = Self::new(accel_range, gyro_range, sample_rate);
        cfg.axis_count = AxisCount::Dof9;
        cfg
    }

    /// Return the Nyquist bandwidth for this sample rate.
    pub fn bandwidth(&self) -> f64 {
        self.sample_rate / 2.0
    }
}

// ---------------------------------------------------------------------------
// ImuSample
// ---------------------------------------------------------------------------

/// A single IMU measurement sample.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// Accelerometer readings [ax, ay, az] in m/s^2.
    pub accel: [f64; 3],
    /// Gyroscope readings [gx, gy, gz] in rad/s.
    pub gyro: [f64; 3],
    /// Optional magnetometer readings [mx, my, mz] in µT (for 9-DOF).
    pub mag: Option<[f64; 3]>,
    /// Timestamp in seconds.
    pub timestamp_s: f64,
}

// ---------------------------------------------------------------------------
// CalibrationResult
// ---------------------------------------------------------------------------

/// Result of accelerometer 6-position calibration.
///
/// Models the sensor error as:
///   a_true = M * S * (a_raw - b)
///
/// where S is the scale factor matrix, M is the cross-axis (misalignment)
/// matrix, and b is the bias vector.
#[derive(Debug, Clone)]
pub struct AccelCalibration {
    /// Scale factors [sx, sy, sz] (ideally all 1.0).
    pub scale: [f64; 3],
    /// Bias offsets [bx, by, bz] in m/s^2.
    pub bias: [f64; 3],
    /// Cross-axis sensitivity matrix (3x3 row-major).
    /// Diagonal elements are 1.0; off-diagonal represent misalignment.
    pub cross_axis: [[f64; 3]; 3],
}

impl AccelCalibration {
    /// Apply calibration to a raw accelerometer reading.
    pub fn apply(&self, raw: &[f64; 3]) -> [f64; 3] {
        // Step 1: Remove bias
        let debiased = [
            raw[0] - self.bias[0],
            raw[1] - self.bias[1],
            raw[2] - self.bias[2],
        ];
        // Step 2: Apply scale
        let scaled = [
            debiased[0] * self.scale[0],
            debiased[1] * self.scale[1],
            debiased[2] * self.scale[2],
        ];
        // Step 3: Apply cross-axis correction
        let mut corrected = [0.0f64; 3];
        for i in 0..3 {
            for j in 0..3 {
                corrected[i] += self.cross_axis[i][j] * scaled[j];
            }
        }
        corrected
    }
}

/// Result of gyroscope calibration.
#[derive(Debug, Clone)]
pub struct GyroCalibration {
    /// Static bias offsets [bx, by, bz] in rad/s.
    pub bias: [f64; 3],
    /// Temperature coefficient for bias drift [cx, cy, cz] in (rad/s)/°C.
    pub temp_coeff: [f64; 3],
    /// Reference temperature in °C.
    pub ref_temp: f64,
}

impl GyroCalibration {
    /// Apply calibration at a given temperature.
    pub fn apply(&self, raw: &[f64; 3], temperature_c: f64) -> [f64; 3] {
        let dt = temperature_c - self.ref_temp;
        [
            raw[0] - self.bias[0] - self.temp_coeff[0] * dt,
            raw[1] - self.bias[1] - self.temp_coeff[1] * dt,
            raw[2] - self.bias[2] - self.temp_coeff[2] * dt,
        ]
    }
}

// ---------------------------------------------------------------------------
// Quaternion helpers (standalone functions)
// ---------------------------------------------------------------------------

/// Multiply two quaternions using the Hamilton product.
///
/// Quaternion convention: [w, x, y, z] where w is the scalar part.
///
/// q1 * q2 = [w1*w2 - x1*x2 - y1*y2 - z1*z2,
///            w1*x2 + x1*w2 + y1*z2 - z1*y2,
///            w1*y2 - x1*z2 + y1*w2 + z1*x2,
///            w1*z2 + x1*y2 - y1*x2 + z1*w2]
pub fn quaternion_multiply(q1: &[f64; 4], q2: &[f64; 4]) -> [f64; 4] {
    let (w1, x1, y1, z1) = (q1[0], q1[1], q1[2], q1[3]);
    let (w2, x2, y2, z2) = (q2[0], q2[1], q2[2], q2[3]);
    [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]
}

/// Normalize a quaternion to unit length.
pub fn quaternion_normalize(q: &[f64; 4]) -> [f64; 4] {
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm < 1e-15 {
        return [1.0, 0.0, 0.0, 0.0]; // identity fallback
    }
    let inv = 1.0 / norm;
    [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv]
}

/// Compute the conjugate (inverse for unit quaternions) of a quaternion.
pub fn quaternion_conjugate(q: &[f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Convert a quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw] in radians.
///
/// Uses the ZYX (yaw-pitch-roll) convention. Gimbal lock occurs when pitch ≈ ±90°.
pub fn quaternion_to_euler(q: &[f64; 4]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);

    // Roll (rotation about X)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (rotation about Y)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        (PI / 2.0).copysign(sinp) // gimbal lock
    } else {
        sinp.asin()
    };

    // Yaw (rotation about Z)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    [roll, pitch, yaw]
}

/// Convert Euler angles [roll, pitch, yaw] in radians to a quaternion [w, x, y, z].
///
/// Uses the ZYX (yaw-pitch-roll) convention.
pub fn euler_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sy, cy) = (yaw / 2.0).sin_cos();

    [
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    ]
}

/// Extract a 3x3 rotation matrix (DCM) from a unit quaternion [w, x, y, z].
///
/// Returns the matrix in row-major order as [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]].
pub fn rotation_matrix_from_quaternion(q: &[f64; 4]) -> [[f64; 3]; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let (x2, y2, z2) = (x * x, y * y, z * z);
    [
        [
            1.0 - 2.0 * (y2 + z2),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x2 + z2),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x2 + y2),
        ],
    ]
}

// ---------------------------------------------------------------------------
// ImuProcessor
// ---------------------------------------------------------------------------

/// Signal processor for MEMS IMU data.
///
/// Provides calibration, orientation estimation, navigation, noise analysis,
/// and motion detection for accelerometer + gyroscope (+ optional magnetometer)
/// data streams.
pub struct ImuProcessor {
    /// Sensor configuration.
    pub config: ImuConfig,
    /// Current orientation quaternion [w, x, y, z].
    pub orientation: [f64; 4],
    /// Current velocity estimate [vx, vy, vz] in m/s (for dead reckoning).
    pub velocity: [f64; 3],
    /// Current position estimate [x, y, z] in meters (for dead reckoning).
    pub position: [f64; 3],
    /// Complementary filter state: pitch and roll in radians.
    pub comp_pitch: f64,
    pub comp_roll: f64,
    /// Madgwick filter beta parameter (algorithm gain).
    pub madgwick_beta: f64,
    /// Previous timestamp for dt computation.
    prev_timestamp: f64,
    /// Previous gyro readings for trapezoidal integration.
    prev_gyro: [f64; 3],
    /// Integrated angle from gyroscope [rx, ry, rz] in radians.
    pub integrated_angle: [f64; 3],
    /// Step counter for pedometer.
    pub step_count: u32,
    /// Timestamp of last detected step (for refractory period).
    last_step_time: f64,
    /// Low-pass filter state (one per axis, 3 axes).
    lp_state: [f64; 3],
    /// Whether the processor has been initialized with a first sample.
    initialized: bool,
}

impl ImuProcessor {
    /// Create a new IMU processor with the given configuration.
    pub fn new(config: ImuConfig) -> Self {
        Self {
            config,
            orientation: [1.0, 0.0, 0.0, 0.0], // identity quaternion
            velocity: [0.0; 3],
            position: [0.0; 3],
            comp_pitch: 0.0,
            comp_roll: 0.0,
            madgwick_beta: 0.1, // default Madgwick gain
            prev_timestamp: 0.0,
            prev_gyro: [0.0; 3],
            integrated_angle: [0.0; 3],
            step_count: 0,
            last_step_time: -1.0,
            lp_state: [0.0; 3],
            initialized: false,
        }
    }

    /// Reset processor state to initial conditions.
    pub fn reset(&mut self) {
        self.orientation = [1.0, 0.0, 0.0, 0.0];
        self.velocity = [0.0; 3];
        self.position = [0.0; 3];
        self.comp_pitch = 0.0;
        self.comp_roll = 0.0;
        self.prev_timestamp = 0.0;
        self.prev_gyro = [0.0; 3];
        self.integrated_angle = [0.0; 3];
        self.step_count = 0;
        self.last_step_time = -1.0;
        self.lp_state = [0.0; 3];
        self.initialized = false;
    }

    // -----------------------------------------------------------------------
    // Calibration
    // -----------------------------------------------------------------------

    /// Perform 6-position accelerometer calibration.
    ///
    /// Takes 6 sets of readings, one for each orientation where a single axis
    /// points up (+g) and down (-g). The order expected is:
    ///   [+X up, -X up, +Y up, -Y up, +Z up, -Z up]
    ///
    /// Each entry is the average reading [ax, ay, az] in m/s^2 for that position.
    ///
    /// Returns scale factors, biases, and a cross-axis sensitivity matrix.
    pub fn calibrate_accelerometer(
        readings: &[[f64; 3]; 6],
    ) -> AccelCalibration {
        // For each axis, the +/- positions give us:
        //   reading_plus  = scale * (+g) + bias + cross_axis_contributions
        //   reading_minus = scale * (-g) + bias + cross_axis_contributions
        //
        // Simplified approach (ignoring cross-axis for scale/bias estimation):
        //   scale_i = (reading_plus_i - reading_minus_i) / (2 * g)
        //   bias_i  = (reading_plus_i + reading_minus_i) / 2

        let g = G0;
        let mut scale = [0.0f64; 3];
        let mut bias = [0.0f64; 3];

        // +X up is readings[0], -X up is readings[1]
        // For X-axis: the X component sees ±g
        scale[0] = (readings[0][0] - readings[1][0]) / (2.0 * g);
        bias[0] = (readings[0][0] + readings[1][0]) / 2.0;

        // +Y up is readings[2], -Y up is readings[3]
        scale[1] = (readings[2][1] - readings[3][1]) / (2.0 * g);
        bias[1] = (readings[2][1] + readings[3][1]) / 2.0;

        // +Z up is readings[4], -Z up is readings[5]
        scale[2] = (readings[4][2] - readings[5][2]) / (2.0 * g);
        bias[2] = (readings[4][2] + readings[5][2]) / 2.0;

        // Estimate cross-axis sensitivity from the off-axis readings.
        // When X is pointed up, the Y and Z readings should ideally be zero.
        // cross_axis[i][j] = how much axis j leaks into axis i.
        let mut cross_axis = [[0.0f64; 3]; 3];
        // Start with identity
        for i in 0..3 {
            cross_axis[i][i] = 1.0;
        }

        // Cross-axis: when +X up, avg of Y reading from +X and -X positions
        // gives leakage of X into Y
        let leak_x_to_y = (readings[0][1] + readings[1][1]) / 2.0 - bias[1];
        let leak_x_to_z = (readings[0][2] + readings[1][2]) / 2.0 - bias[2];
        if scale[0].abs() > 1e-10 {
            cross_axis[1][0] = -leak_x_to_y / (scale[0] * g);
            cross_axis[2][0] = -leak_x_to_z / (scale[0] * g);
        }

        let leak_y_to_x = (readings[2][0] + readings[3][0]) / 2.0 - bias[0];
        let leak_y_to_z = (readings[2][2] + readings[3][2]) / 2.0 - bias[2];
        if scale[1].abs() > 1e-10 {
            cross_axis[0][1] = -leak_y_to_x / (scale[1] * g);
            cross_axis[2][1] = -leak_y_to_z / (scale[1] * g);
        }

        let leak_z_to_x = (readings[4][0] + readings[5][0]) / 2.0 - bias[0];
        let leak_z_to_y = (readings[4][1] + readings[5][1]) / 2.0 - bias[1];
        if scale[2].abs() > 1e-10 {
            cross_axis[0][2] = -leak_z_to_x / (scale[2] * g);
            cross_axis[1][2] = -leak_z_to_y / (scale[2] * g);
        }

        // Invert scale to convert from raw to true
        // (we want to divide by scale factor, not multiply)
        let inv_scale = [
            if scale[0].abs() > 1e-10 { 1.0 / scale[0] } else { 1.0 },
            if scale[1].abs() > 1e-10 { 1.0 / scale[1] } else { 1.0 },
            if scale[2].abs() > 1e-10 { 1.0 / scale[2] } else { 1.0 },
        ];

        AccelCalibration {
            scale: inv_scale,
            bias,
            cross_axis,
        }
    }

    /// Perform static gyroscope bias calibration.
    ///
    /// Averages gyroscope readings collected while the sensor is stationary.
    /// Optionally estimates temperature-dependent bias drift.
    ///
    /// `readings`: array of gyro measurements [gx, gy, gz] in rad/s.
    /// `temperatures`: optional per-reading temperatures in °C.
    pub fn calibrate_gyroscope(
        readings: &[[f64; 3]],
        temperatures: Option<&[f64]>,
    ) -> GyroCalibration {
        assert!(!readings.is_empty(), "Need at least one reading for calibration");

        let n = readings.len() as f64;
        let mut bias = [0.0f64; 3];
        for r in readings {
            bias[0] += r[0];
            bias[1] += r[1];
            bias[2] += r[2];
        }
        bias[0] /= n;
        bias[1] /= n;
        bias[2] /= n;

        let mut ref_temp = 25.0;
        let mut temp_coeff = [0.0f64; 3];

        if let Some(temps) = temperatures {
            assert!(temps.len() == readings.len(), "Temperature count must match readings");
            let mean_temp: f64 = temps.iter().sum::<f64>() / n;
            ref_temp = mean_temp;

            // Linear regression: coeff = sum((t_i - mean_t)*(g_i - mean_g)) / sum((t_i - mean_t)^2)
            let mut sum_dt2 = 0.0f64;
            let mut sum_dt_dg = [0.0f64; 3];
            for (i, r) in readings.iter().enumerate() {
                let dt = temps[i] - mean_temp;
                sum_dt2 += dt * dt;
                for j in 0..3 {
                    sum_dt_dg[j] += dt * (r[j] - bias[j]);
                }
            }
            if sum_dt2 > 1e-15 {
                for j in 0..3 {
                    temp_coeff[j] = sum_dt_dg[j] / sum_dt2;
                }
            }
        }

        GyroCalibration {
            bias,
            temp_coeff,
            ref_temp,
        }
    }

    // -----------------------------------------------------------------------
    // Orientation estimation
    // -----------------------------------------------------------------------

    /// Compute pitch and roll from the accelerometer gravity vector.
    ///
    /// Assumes the sensor is in a quasi-static state (no significant linear
    /// acceleration). Uses atan2 for full quadrant resolution.
    ///
    /// Returns (pitch, roll) in radians.
    /// - Pitch: rotation about Y axis (nose up positive)
    /// - Roll: rotation about X axis (right wing down positive)
    pub fn tilt_from_accel(accel: &[f64; 3]) -> (f64, f64) {
        let (ax, ay, az) = (accel[0], accel[1], accel[2]);
        let roll = ay.atan2(az);
        let pitch = (-ax).atan2((ay * ay + az * az).sqrt());
        (pitch, roll)
    }

    /// Compute magnetic heading (yaw) from magnetometer readings with
    /// tilt compensation.
    ///
    /// `mag`: magnetometer [mx, my, mz] in any consistent unit (µT, Gauss).
    /// `pitch`, `roll`: current tilt angles in radians.
    ///
    /// Returns heading in radians [0, 2π), where 0 = magnetic field aligned
    /// with body X axis (body X pointing toward magnetic north).
    pub fn heading_from_mag(mag: &[f64; 3], pitch: f64, roll: f64) -> f64 {
        let (mx, my, mz) = (mag[0], mag[1], mag[2]);
        let (sp, cp) = pitch.sin_cos();
        let (sr, cr) = roll.sin_cos();

        // Tilt-compensated magnetic field components in the horizontal plane
        let mx_h = mx * cp + my * sp * sr + mz * sp * cr;
        let my_h = my * cr - mz * sr;

        let mut heading = my_h.atan2(mx_h);
        if heading < 0.0 {
            heading += 2.0 * PI;
        }
        heading
    }

    /// Update orientation using a complementary filter.
    ///
    /// Fuses high-passed gyroscope data with low-passed accelerometer tilt.
    /// `alpha` controls the blend: higher alpha trusts the gyroscope more.
    /// Typical values: 0.96 - 0.99.
    ///
    /// Updates `self.comp_pitch` and `self.comp_roll`.
    ///
    /// Returns (pitch, roll) in radians.
    pub fn complementary_filter(
        &mut self,
        accel: &[f64; 3],
        gyro: &[f64; 3],
        dt: f64,
        alpha: f64,
    ) -> (f64, f64) {
        // Tilt from accelerometer
        let (accel_pitch, accel_roll) = Self::tilt_from_accel(accel);

        // Integrate gyroscope (gyro[0] = roll rate, gyro[1] = pitch rate)
        let gyro_pitch = self.comp_pitch + gyro[1] * dt;
        let gyro_roll = self.comp_roll + gyro[0] * dt;

        // Complementary blend
        self.comp_pitch = alpha * gyro_pitch + (1.0 - alpha) * accel_pitch;
        self.comp_roll = alpha * gyro_roll + (1.0 - alpha) * accel_roll;

        (self.comp_pitch, self.comp_roll)
    }

    /// Update orientation using the Madgwick AHRS gradient descent filter.
    ///
    /// This is a computationally efficient orientation filter that uses a
    /// gradient descent algorithm to compute the direction of the gyroscope
    /// measurement error as a quaternion derivative.
    ///
    /// `accel`: accelerometer [ax, ay, az] in m/s^2 (will be normalized).
    /// `gyro`: gyroscope [gx, gy, gz] in rad/s.
    /// `dt`: time step in seconds.
    ///
    /// Updates `self.orientation` and returns the quaternion [w, x, y, z].
    pub fn madgwick_filter(
        &mut self,
        accel: &[f64; 3],
        gyro: &[f64; 3],
        dt: f64,
    ) -> [f64; 4] {
        let q = self.orientation;
        let (q0, q1, q2, q3) = (q[0], q[1], q[2], q[3]);
        let beta = self.madgwick_beta;

        // Normalize accelerometer measurement
        let a_norm = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();
        if a_norm < 1e-10 {
            // Cannot correct without valid accelerometer data; gyro-only update
            let qd = quaternion_multiply(
                &q,
                &[0.0, gyro[0] * 0.5, gyro[1] * 0.5, gyro[2] * 0.5],
            );
            let mut qn = [
                q[0] + qd[0] * dt,
                q[1] + qd[1] * dt,
                q[2] + qd[2] * dt,
                q[3] + qd[3] * dt,
            ];
            qn = quaternion_normalize(&qn);
            self.orientation = qn;
            return qn;
        }

        let ax = accel[0] / a_norm;
        let ay = accel[1] / a_norm;
        let az = accel[2] / a_norm;

        // Gradient of the objective function (Madgwick Eq. 25)
        // f = [2(q1*q3 - q0*q2) - ax,
        //      2(q0*q1 + q2*q3) - ay,
        //      2(0.5 - q1^2 - q2^2) - az]
        let f1 = 2.0 * (q1 * q3 - q0 * q2) - ax;
        let f2 = 2.0 * (q0 * q1 + q2 * q3) - ay;
        let f3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az;

        // Jacobian (Madgwick Eq. 26)
        let j_t_f = [
            -2.0 * q2 * f1 + 2.0 * q1 * f2,
            2.0 * q3 * f1 + 2.0 * q0 * f2 - 4.0 * q1 * f3,
            -2.0 * q0 * f1 + 2.0 * q3 * f2 - 4.0 * q2 * f3,
            2.0 * q1 * f1 + 2.0 * q2 * f2,
        ];

        // Normalize gradient
        let grad_norm = (j_t_f[0] * j_t_f[0]
            + j_t_f[1] * j_t_f[1]
            + j_t_f[2] * j_t_f[2]
            + j_t_f[3] * j_t_f[3])
            .sqrt();

        let (s0, s1, s2, s3) = if grad_norm > 1e-15 {
            let inv = 1.0 / grad_norm;
            (
                j_t_f[0] * inv,
                j_t_f[1] * inv,
                j_t_f[2] * inv,
                j_t_f[3] * inv,
            )
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        // Quaternion rate of change from gyroscope
        let qd = quaternion_multiply(
            &q,
            &[0.0, gyro[0] * 0.5, gyro[1] * 0.5, gyro[2] * 0.5],
        );

        // Apply feedback
        let mut qn = [
            q0 + (qd[0] - beta * s0) * dt,
            q1 + (qd[1] - beta * s1) * dt,
            q2 + (qd[2] - beta * s2) * dt,
            q3 + (qd[3] - beta * s3) * dt,
        ];
        qn = quaternion_normalize(&qn);
        self.orientation = qn;
        qn
    }

    // -----------------------------------------------------------------------
    // Navigation
    // -----------------------------------------------------------------------

    /// Integrate gyroscope angular rates to compute angle change.
    ///
    /// Uses trapezoidal integration for improved accuracy.
    /// `gyro`: current gyroscope readings [gx, gy, gz] in rad/s.
    /// `dt`: time step in seconds.
    ///
    /// Returns integrated angle increment [drx, dry, drz] in radians.
    /// Also updates `self.integrated_angle`.
    pub fn gyro_integration(&mut self, gyro: &[f64; 3], dt: f64) -> [f64; 3] {
        let d_angle = if self.initialized {
            // Trapezoidal rule: (prev + current) / 2 * dt
            [
                (self.prev_gyro[0] + gyro[0]) * 0.5 * dt,
                (self.prev_gyro[1] + gyro[1]) * 0.5 * dt,
                (self.prev_gyro[2] + gyro[2]) * 0.5 * dt,
            ]
        } else {
            // First sample: rectangular rule
            [gyro[0] * dt, gyro[1] * dt, gyro[2] * dt]
        };

        self.integrated_angle[0] += d_angle[0];
        self.integrated_angle[1] += d_angle[1];
        self.integrated_angle[2] += d_angle[2];
        self.prev_gyro = *gyro;
        self.initialized = true;

        d_angle
    }

    /// Remove gravity from accelerometer readings in the body frame.
    ///
    /// Uses the current orientation quaternion to rotate the gravity vector
    /// into the body frame and subtract it from the accelerometer reading.
    ///
    /// Returns linear acceleration [ax, ay, az] in m/s^2.
    pub fn gravity_removal(&self, accel: &[f64; 3]) -> [f64; 3] {
        // Gravity in world frame (assuming NED: gravity is [0, 0, +g])
        // For ENU (z-up): gravity is [0, 0, -g]
        // We use z-up convention: gravity = [0, 0, -g]
        let g_world = [0.0, 0.0, -G0];

        // Rotate gravity vector from world to body frame:
        // g_body = q^(-1) * [0, gx, gy, gz] * q
        let qc = quaternion_conjugate(&self.orientation);
        let g_quat = [0.0, g_world[0], g_world[1], g_world[2]];
        let tmp = quaternion_multiply(&qc, &g_quat);
        let g_body_quat = quaternion_multiply(&tmp, &self.orientation);

        [
            accel[0] - g_body_quat[1],
            accel[1] - g_body_quat[2],
            accel[2] - g_body_quat[3],
        ]
    }

    /// Perform dead reckoning by double-integrating linear acceleration.
    ///
    /// **Warning**: Dead reckoning from MEMS accelerometers drifts rapidly
    /// due to bias and noise being integrated twice. Position error grows
    /// as O(t^2) for bias errors and O(t^(5/2)) for noise. Use only for
    /// short-duration motion (seconds, not minutes).
    ///
    /// `linear_accel`: gravity-free acceleration [ax, ay, az] in m/s^2
    ///   (output from `gravity_removal`).
    /// `dt`: time step in seconds.
    ///
    /// Updates `self.velocity` and `self.position`.
    pub fn dead_reckoning(&mut self, linear_accel: &[f64; 3], dt: f64) {
        // Trapezoidal velocity integration: v = v + a * dt
        for i in 0..3 {
            self.velocity[i] += linear_accel[i] * dt;
        }
        // Trapezoidal position integration: p = p + v * dt
        for i in 0..3 {
            self.position[i] += self.velocity[i] * dt;
        }
    }

    // -----------------------------------------------------------------------
    // Noise analysis
    // -----------------------------------------------------------------------

    /// Convert spectral noise density to RMS noise at a given bandwidth.
    ///
    /// `noise_density`: spectral noise density in units/√Hz
    ///   (e.g., µg/√Hz for accelerometer, °/s/√Hz for gyro).
    /// `bandwidth`: measurement bandwidth in Hz.
    ///
    /// RMS noise = noise_density * √bandwidth
    pub fn noise_density_to_rms(noise_density: f64, bandwidth: f64) -> f64 {
        noise_density * bandwidth.sqrt()
    }

    /// Compute Allan variance and Allan deviation from a gyroscope time series.
    ///
    /// The Allan variance characterizes sensor noise as a function of
    /// averaging time (cluster size). It reveals:
    /// - White noise (slope -1/2 on log-log ADEV plot)
    /// - Bias instability (minimum of ADEV curve)
    /// - Rate random walk (slope +1/2)
    ///
    /// `data`: time series of gyroscope readings (rad/s) at uniform sample rate.
    /// `sample_rate`: sample rate in Hz.
    /// `max_clusters`: maximum number of cluster sizes to compute (log-spaced).
    ///
    /// Returns Vec of (tau, allan_variance, allan_deviation) tuples.
    pub fn allan_variance(
        data: &[f64],
        sample_rate: f64,
        max_clusters: usize,
    ) -> Vec<(f64, f64, f64)> {
        let n = data.len();
        if n < 3 {
            return vec![];
        }

        let dt = 1.0 / sample_rate;

        // Generate log-spaced cluster sizes from 1 to n/2
        let max_m = n / 2;
        let mut cluster_sizes = Vec::new();
        let log_min = 0.0f64; // log10(1)
        let log_max = (max_m as f64).log10();
        let num_points = max_clusters.min(max_m);

        for i in 0..num_points {
            let log_m = log_min + (log_max - log_min) * (i as f64) / (num_points as f64 - 1.0).max(1.0);
            let m = 10.0f64.powf(log_m).round() as usize;
            if m >= 1 && m <= max_m && !cluster_sizes.contains(&m) {
                cluster_sizes.push(m);
            }
        }
        cluster_sizes.sort();

        let mut results = Vec::with_capacity(cluster_sizes.len());

        for &m in &cluster_sizes {
            let tau = m as f64 * dt;
            let num_clusters = n / m;
            if num_clusters < 2 {
                continue;
            }

            // Compute cluster averages
            let mut averages = Vec::with_capacity(num_clusters);
            for k in 0..num_clusters {
                let start = k * m;
                let end = start + m;
                let sum: f64 = data[start..end].iter().sum();
                averages.push(sum / m as f64);
            }

            // Allan variance = (1 / 2(N-1)) * sum((avg[i+1] - avg[i])^2)
            let mut avar = 0.0;
            for i in 0..averages.len() - 1 {
                let diff = averages[i + 1] - averages[i];
                avar += diff * diff;
            }
            avar /= 2.0 * (averages.len() - 1) as f64;

            let adev = avar.sqrt();
            results.push((tau, avar, adev));
        }

        results
    }

    /// Estimate vibration rectification error from accelerometer nonlinearity.
    ///
    /// When a MEMS accelerometer with quadratic nonlinearity is subjected to
    /// vibration, the squared vibration signal produces a DC offset (rectification
    /// error). This is proportional to the vibration RMS squared and the
    /// second-order nonlinearity coefficient.
    ///
    /// `k2`: second-order nonlinearity coefficient (units of g/g^2, e.g., 0.001).
    /// `vib_rms_g`: RMS vibration level in g.
    ///
    /// Returns the rectification error in g.
    pub fn vibration_rectification(k2: f64, vib_rms_g: f64) -> f64 {
        // Rectification error ≈ k2 * vrms^2
        k2 * vib_rms_g * vib_rms_g
    }

    // -----------------------------------------------------------------------
    // Motion detection
    // -----------------------------------------------------------------------

    /// Detect steps from accelerometer magnitude for pedometer applications.
    ///
    /// Uses peak detection on the magnitude of the accelerometer vector
    /// with a configurable threshold above nominal gravity and a refractory
    /// period to prevent double-counting.
    ///
    /// `accel_data`: sequence of accelerometer readings [ax, ay, az] in m/s^2.
    /// `timestamps`: corresponding timestamps in seconds.
    /// `threshold_g`: peak detection threshold above 1g (e.g., 0.3 for 0.3g).
    /// `refractory_s`: minimum time between steps in seconds (e.g., 0.3).
    ///
    /// Returns vector of step timestamps.
    pub fn step_detection(
        accel_data: &[[f64; 3]],
        timestamps: &[f64],
        threshold_g: f64,
        refractory_s: f64,
    ) -> Vec<f64> {
        assert_eq!(accel_data.len(), timestamps.len());
        if accel_data.len() < 3 {
            return vec![];
        }

        let threshold = G0 * (1.0 + threshold_g);
        let mut steps = Vec::new();
        let mut last_step_time = -refractory_s * 2.0; // allow first step

        // Compute magnitudes
        let magnitudes: Vec<f64> = accel_data
            .iter()
            .map(|a| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt())
            .collect();

        // Simple peak detection: a peak is a sample greater than both neighbors
        // and above the threshold, respecting the refractory period.
        for i in 1..magnitudes.len() - 1 {
            if magnitudes[i] > magnitudes[i - 1]
                && magnitudes[i] > magnitudes[i + 1]
                && magnitudes[i] > threshold
                && (timestamps[i] - last_step_time) >= refractory_s
            {
                steps.push(timestamps[i]);
                last_step_time = timestamps[i];
            }
        }

        steps
    }

    // -----------------------------------------------------------------------
    // Sensor fusion weights
    // -----------------------------------------------------------------------

    /// Compute optimal sensor fusion weights from noise characteristics.
    ///
    /// Given the variance of two sensor estimates of the same quantity,
    /// computes the optimal Kalman-like weights that minimize the fused
    /// estimate variance.
    ///
    /// `var_a`: variance of sensor A.
    /// `var_b`: variance of sensor B.
    ///
    /// Returns (weight_a, weight_b) where fused = wa * a + wb * b.
    pub fn sensor_fusion_weights(var_a: f64, var_b: f64) -> (f64, f64) {
        let total = var_a + var_b;
        if total < 1e-30 {
            return (0.5, 0.5);
        }
        // Optimal weight is inversely proportional to variance
        let wa = var_b / total;
        let wb = var_a / total;
        (wa, wb)
    }

    // -----------------------------------------------------------------------
    // Low-pass filter
    // -----------------------------------------------------------------------

    /// Apply a simple first-order IIR low-pass filter to a 3-axis signal.
    ///
    /// y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    ///
    /// `input`: current sample [x, y, z].
    /// `alpha`: filter coefficient (0 < alpha <= 1). Smaller alpha = more smoothing.
    ///   Typical: alpha = dt / (rc + dt) where rc = 1/(2*pi*fc).
    ///
    /// Returns filtered sample. Updates internal filter state.
    pub fn low_pass_filter(&mut self, input: &[f64; 3], alpha: f64) -> [f64; 3] {
        let a = alpha.clamp(0.0, 1.0);
        for i in 0..3 {
            self.lp_state[i] = a * input[i] + (1.0 - a) * self.lp_state[i];
        }
        self.lp_state
    }

    /// Compute the low-pass filter alpha coefficient from cutoff frequency and sample rate.
    ///
    /// `cutoff_hz`: desired cutoff frequency in Hz.
    /// `sample_rate`: sample rate in Hz.
    ///
    /// Returns alpha coefficient for use with `low_pass_filter()`.
    pub fn lp_alpha(cutoff_hz: f64, sample_rate: f64) -> f64 {
        let rc = 1.0 / (2.0 * PI * cutoff_hz);
        let dt = 1.0 / sample_rate;
        dt / (rc + dt)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;
    const EPSILON_LOOSE: f64 = 1e-3;

    // -----------------------------------------------------------------------
    // AccelRange / GyroRange
    // -----------------------------------------------------------------------

    #[test]
    fn test_accel_range_full_scale() {
        assert!((AccelRange::G2.full_scale_g() - 2.0).abs() < EPSILON);
        assert!((AccelRange::G4.full_scale_g() - 4.0).abs() < EPSILON);
        assert!((AccelRange::G8.full_scale_g() - 8.0).abs() < EPSILON);
        assert!((AccelRange::G16.full_scale_g() - 16.0).abs() < EPSILON);

        assert!((AccelRange::G2.full_scale_ms2() - 2.0 * G0).abs() < EPSILON);
    }

    #[test]
    fn test_gyro_range_full_scale() {
        assert!((GyroRange::Dps250.full_scale_dps() - 250.0).abs() < EPSILON);
        assert!((GyroRange::Dps500.full_scale_dps() - 500.0).abs() < EPSILON);
        assert!((GyroRange::Dps1000.full_scale_dps() - 1000.0).abs() < EPSILON);
        assert!((GyroRange::Dps2000.full_scale_dps() - 2000.0).abs() < EPSILON);

        let rps = GyroRange::Dps250.full_scale_rps();
        assert!((rps - 250.0 * DEG_TO_RAD).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // ImuConfig
    // -----------------------------------------------------------------------

    #[test]
    fn test_imu_config_new() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 200.0);
        assert_eq!(cfg.accel_range, AccelRange::G4);
        assert_eq!(cfg.gyro_range, GyroRange::Dps500);
        assert!((cfg.sample_rate - 200.0).abs() < EPSILON);
        assert_eq!(cfg.axis_count, AxisCount::Dof6);
        assert!((cfg.bandwidth() - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_imu_config_9dof() {
        let cfg = ImuConfig::new_9dof(AccelRange::G2, GyroRange::Dps250, 100.0);
        assert_eq!(cfg.axis_count, AxisCount::Dof9);
    }

    // -----------------------------------------------------------------------
    // Quaternion operations
    // -----------------------------------------------------------------------

    #[test]
    fn test_quaternion_multiply_identity() {
        let id = [1.0, 0.0, 0.0, 0.0];
        let q = [0.707, 0.707, 0.0, 0.0];
        let result = quaternion_multiply(&id, &q);
        for i in 0..4 {
            assert!((result[i] - q[i]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_quaternion_multiply_associativity() {
        let q1 = quaternion_normalize(&[1.0, 1.0, 0.0, 0.0]);
        let q2 = quaternion_normalize(&[1.0, 0.0, 1.0, 0.0]);
        let q3 = quaternion_normalize(&[1.0, 0.0, 0.0, 1.0]);

        let r1 = quaternion_multiply(&quaternion_multiply(&q1, &q2), &q3);
        let r2 = quaternion_multiply(&q1, &quaternion_multiply(&q2, &q3));
        for i in 0..4 {
            assert!((r1[i] - r2[i]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_quaternion_normalize() {
        let q = [2.0, 1.0, 1.0, 1.0];
        let n = quaternion_normalize(&q);
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2] + n[3] * n[3]).sqrt();
        assert!((len - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_quaternion_normalize_zero() {
        let q = [0.0, 0.0, 0.0, 0.0];
        let n = quaternion_normalize(&q);
        assert!((n[0] - 1.0).abs() < EPSILON); // identity fallback
    }

    #[test]
    fn test_quaternion_conjugate() {
        let q = [0.5, 0.5, 0.5, 0.5];
        let qc = quaternion_conjugate(&q);
        assert!((qc[0] - 0.5).abs() < EPSILON);
        assert!((qc[1] + 0.5).abs() < EPSILON);
        assert!((qc[2] + 0.5).abs() < EPSILON);
        assert!((qc[3] + 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_quaternion_inverse_property() {
        // q * q^(-1) should be identity for unit quaternions
        let q = quaternion_normalize(&[1.0, 2.0, 3.0, 4.0]);
        let qc = quaternion_conjugate(&q);
        let product = quaternion_multiply(&q, &qc);
        assert!((product[0] - 1.0).abs() < EPSILON);
        assert!(product[1].abs() < EPSILON);
        assert!(product[2].abs() < EPSILON);
        assert!(product[3].abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Euler <-> Quaternion conversion
    // -----------------------------------------------------------------------

    #[test]
    fn test_euler_to_quaternion_identity() {
        let q = euler_to_quaternion(0.0, 0.0, 0.0);
        assert!((q[0] - 1.0).abs() < EPSILON);
        assert!(q[1].abs() < EPSILON);
        assert!(q[2].abs() < EPSILON);
        assert!(q[3].abs() < EPSILON);
    }

    #[test]
    fn test_quaternion_to_euler_roundtrip() {
        let roll = 0.3;
        let pitch = 0.2;
        let yaw = 0.5;
        let q = euler_to_quaternion(roll, pitch, yaw);
        let euler = quaternion_to_euler(&q);
        assert!((euler[0] - roll).abs() < EPSILON_LOOSE);
        assert!((euler[1] - pitch).abs() < EPSILON_LOOSE);
        assert!((euler[2] - yaw).abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_euler_quaternion_90_deg_rotations() {
        // 90° roll
        let q = euler_to_quaternion(PI / 2.0, 0.0, 0.0);
        let euler = quaternion_to_euler(&q);
        assert!((euler[0] - PI / 2.0).abs() < EPSILON_LOOSE);
        assert!(euler[1].abs() < EPSILON_LOOSE);
    }

    // -----------------------------------------------------------------------
    // Rotation matrix
    // -----------------------------------------------------------------------

    #[test]
    fn test_rotation_matrix_identity() {
        let q = [1.0, 0.0, 0.0, 0.0];
        let r = rotation_matrix_from_quaternion(&q);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((r[i][j] - expected).abs() < EPSILON);
            }
        }
    }

    #[test]
    fn test_rotation_matrix_90_deg_z() {
        // 90° yaw rotation
        let q = euler_to_quaternion(0.0, 0.0, PI / 2.0);
        let r = rotation_matrix_from_quaternion(&q);
        // Rotating [1,0,0] by 90° about Z should give [0,1,0]
        let x_rot = [
            r[0][0] * 1.0 + r[0][1] * 0.0 + r[0][2] * 0.0,
            r[1][0] * 1.0 + r[1][1] * 0.0 + r[1][2] * 0.0,
            r[2][0] * 1.0 + r[2][1] * 0.0 + r[2][2] * 0.0,
        ];
        assert!(x_rot[0].abs() < EPSILON_LOOSE);
        assert!((x_rot[1] - 1.0).abs() < EPSILON_LOOSE);
        assert!(x_rot[2].abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_rotation_matrix_orthogonality() {
        let q = quaternion_normalize(&[1.0, 0.5, 0.3, 0.2]);
        let r = rotation_matrix_from_quaternion(&q);
        // R * R^T should be identity
        for i in 0..3 {
            for j in 0..3 {
                let mut dot = 0.0;
                for k in 0..3 {
                    dot += r[i][k] * r[j][k];
                }
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((dot - expected).abs() < EPSILON_LOOSE);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Tilt from accelerometer
    // -----------------------------------------------------------------------

    #[test]
    fn test_tilt_from_accel_level() {
        // Sensor level: gravity along +Z
        let (pitch, roll) = ImuProcessor::tilt_from_accel(&[0.0, 0.0, G0]);
        assert!(pitch.abs() < EPSILON);
        assert!(roll.abs() < EPSILON);
    }

    #[test]
    fn test_tilt_from_accel_pitched() {
        // Sensor pitched 45° nose up: gravity has -X and +Z components
        let angle = PI / 4.0;
        let ax = -G0 * angle.sin();
        let az = G0 * angle.cos();
        let (pitch, _roll) = ImuProcessor::tilt_from_accel(&[ax, 0.0, az]);
        assert!((pitch - angle).abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_tilt_from_accel_rolled() {
        // Sensor rolled 30° right: gravity has +Y and +Z components
        let angle = PI / 6.0;
        let ay = G0 * angle.sin();
        let az = G0 * angle.cos();
        let (_pitch, roll) = ImuProcessor::tilt_from_accel(&[0.0, ay, az]);
        assert!((roll - angle).abs() < EPSILON_LOOSE);
    }

    // -----------------------------------------------------------------------
    // Heading from magnetometer
    // -----------------------------------------------------------------------

    #[test]
    fn test_heading_from_mag_north() {
        // Magnetometer pointing north (positive X), level sensor
        let heading = ImuProcessor::heading_from_mag(&[1.0, 0.0, 0.0], 0.0, 0.0);
        assert!(heading.abs() < EPSILON_LOOSE || (heading - 2.0 * PI).abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_heading_from_mag_east() {
        // Magnetometer pointing east (positive Y)
        let heading = ImuProcessor::heading_from_mag(&[0.0, 1.0, 0.0], 0.0, 0.0);
        assert!((heading - PI / 2.0).abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_heading_from_mag_south() {
        // Magnetometer pointing south (negative X)
        let heading = ImuProcessor::heading_from_mag(&[-1.0, 0.0, 0.0], 0.0, 0.0);
        assert!((heading - PI).abs() < EPSILON_LOOSE);
    }

    // -----------------------------------------------------------------------
    // Complementary filter
    // -----------------------------------------------------------------------

    #[test]
    fn test_complementary_filter_stationary() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let dt = 0.01;
        let alpha = 0.98;

        // Level, stationary
        for _ in 0..100 {
            proc.complementary_filter(&[0.0, 0.0, G0], &[0.0, 0.0, 0.0], dt, alpha);
        }
        assert!(proc.comp_pitch.abs() < EPSILON_LOOSE);
        assert!(proc.comp_roll.abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_complementary_filter_converges() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let dt = 0.01;
        let alpha = 0.98;

        // Tilted 10° pitch, let filter converge
        let tilt = 10.0 * DEG_TO_RAD;
        let ax = -G0 * tilt.sin();
        let az = G0 * tilt.cos();
        for _ in 0..1000 {
            proc.complementary_filter(&[ax, 0.0, az], &[0.0, 0.0, 0.0], dt, alpha);
        }
        assert!((proc.comp_pitch - tilt).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // Madgwick filter
    // -----------------------------------------------------------------------

    #[test]
    fn test_madgwick_filter_stationary() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        proc.madgwick_beta = 0.1;
        let dt = 0.01;

        // Apply stationary readings for convergence
        for _ in 0..500 {
            proc.madgwick_filter(&[0.0, 0.0, G0], &[0.0, 0.0, 0.0], dt);
        }

        let euler = quaternion_to_euler(&proc.orientation);
        assert!(euler[0].abs() < 0.05); // roll near 0
        assert!(euler[1].abs() < 0.05); // pitch near 0
    }

    #[test]
    fn test_madgwick_filter_with_rotation() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        proc.madgwick_beta = 0.05;
        let dt = 0.01;

        // Start with convergence to level
        for _ in 0..200 {
            proc.madgwick_filter(&[0.0, 0.0, G0], &[0.0, 0.0, 0.0], dt);
        }

        // Apply constant yaw rate for 1 second (90 deg/s = pi/2 rad/s)
        let yaw_rate = PI / 2.0;
        for _ in 0..100 {
            proc.madgwick_filter(&[0.0, 0.0, G0], &[0.0, 0.0, yaw_rate], dt);
        }

        let euler = quaternion_to_euler(&proc.orientation);
        // Should have rotated approximately 90° in yaw
        assert!((euler[2].abs() - PI / 2.0).abs() < 0.3);
    }

    #[test]
    fn test_madgwick_filter_zero_accel_fallback() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let dt = 0.01;

        // Zero accelerometer should not crash, gyro-only update
        let q = proc.madgwick_filter(&[0.0, 0.0, 0.0], &[0.0, 0.0, 0.1], dt);
        let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((len - 1.0).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Accelerometer calibration
    // -----------------------------------------------------------------------

    #[test]
    fn test_accel_calibration_ideal() {
        // Ideal sensor: perfect readings
        let readings = [
            [G0, 0.0, 0.0],   // +X up
            [-G0, 0.0, 0.0],  // -X up
            [0.0, G0, 0.0],   // +Y up
            [0.0, -G0, 0.0],  // -Y up
            [0.0, 0.0, G0],   // +Z up
            [0.0, 0.0, -G0],  // -Z up
        ];
        let cal = ImuProcessor::calibrate_accelerometer(&readings);

        // Scale should be 1/1 = 1.0
        for i in 0..3 {
            assert!((cal.scale[i] - 1.0).abs() < EPSILON);
            assert!(cal.bias[i].abs() < EPSILON);
        }
    }

    #[test]
    fn test_accel_calibration_with_bias() {
        // Sensor with 0.1 m/s^2 bias on each axis
        let bias = 0.1;
        let readings = [
            [G0 + bias, bias, bias],
            [-G0 + bias, bias, bias],
            [bias, G0 + bias, bias],
            [bias, -G0 + bias, bias],
            [bias, bias, G0 + bias],
            [bias, bias, -G0 + bias],
        ];
        let cal = ImuProcessor::calibrate_accelerometer(&readings);

        for i in 0..3 {
            assert!((cal.bias[i] - bias).abs() < EPSILON);
        }
    }

    #[test]
    fn test_accel_calibration_apply() {
        let cal = AccelCalibration {
            scale: [1.0, 1.0, 1.0],
            bias: [0.1, -0.2, 0.3],
            cross_axis: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        let raw = [G0 + 0.1, -0.2, 0.3];
        let corrected = cal.apply(&raw);
        assert!((corrected[0] - G0).abs() < EPSILON);
        assert!(corrected[1].abs() < EPSILON);
        assert!(corrected[2].abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Gyroscope calibration
    // -----------------------------------------------------------------------

    #[test]
    fn test_gyro_calibration_static() {
        // Stationary gyro with known bias
        let bias_x = 0.01;
        let bias_y = -0.02;
        let bias_z = 0.005;
        let readings: Vec<[f64; 3]> = (0..100)
            .map(|_| [bias_x, bias_y, bias_z])
            .collect();

        let cal = ImuProcessor::calibrate_gyroscope(&readings, None);
        assert!((cal.bias[0] - bias_x).abs() < EPSILON);
        assert!((cal.bias[1] - bias_y).abs() < EPSILON);
        assert!((cal.bias[2] - bias_z).abs() < EPSILON);
    }

    #[test]
    fn test_gyro_calibration_with_temperature() {
        // Bias that drifts linearly with temperature
        let base_bias = 0.01;
        let temp_coeff = 0.001; // rad/s per °C
        let ref_temp = 25.0;

        let mut readings = Vec::new();
        let mut temps = Vec::new();
        for i in 0..100 {
            let t = 20.0 + 0.1 * i as f64; // 20°C to 30°C
            let bias_at_t = base_bias + temp_coeff * (t - ref_temp);
            readings.push([bias_at_t, 0.0, 0.0]);
            temps.push(t);
        }

        let cal = ImuProcessor::calibrate_gyroscope(&readings, Some(&temps));
        assert!((cal.temp_coeff[0] - temp_coeff).abs() < 0.01);
    }

    #[test]
    fn test_gyro_calibration_apply() {
        let cal = GyroCalibration {
            bias: [0.01, -0.02, 0.005],
            temp_coeff: [0.001, 0.0, 0.0],
            ref_temp: 25.0,
        };
        let raw = [0.01 + 0.001 * 5.0, -0.02, 0.005];
        let corrected = cal.apply(&raw, 30.0); // 5°C above ref
        assert!(corrected[0].abs() < EPSILON);
        assert!(corrected[1].abs() < EPSILON);
        assert!(corrected[2].abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Gyro integration
    // -----------------------------------------------------------------------

    #[test]
    fn test_gyro_integration_constant_rate() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let dt = 0.01;
        let rate = 0.1; // rad/s

        // Integrate for 1 second
        for _ in 0..100 {
            proc.gyro_integration(&[rate, 0.0, 0.0], dt);
        }

        // Should accumulate rate * 1.0 = 0.1 radians
        assert!((proc.integrated_angle[0] - 0.1 * 1.0).abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_gyro_integration_trapezoidal() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let dt = 0.01;

        // Linearly increasing rate: should integrate more accurately with trapezoidal
        for i in 0..100 {
            let rate = 0.001 * i as f64;
            proc.gyro_integration(&[rate, 0.0, 0.0], dt);
        }
        // Analytical integral of 0.001*t from 0 to 1s = 0.001 * 0.5 * 1.0 = 0.0005 * 100 steps...
        // rate goes 0.0, 0.001, 0.002, ..., 0.099
        // integral ≈ sum of trapezoids ≈ 0.0495 radians
        assert!(proc.integrated_angle[0] > 0.04);
        assert!(proc.integrated_angle[0] < 0.06);
    }

    // -----------------------------------------------------------------------
    // Gravity removal
    // -----------------------------------------------------------------------

    #[test]
    fn test_gravity_removal_level() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let proc = ImuProcessor::new(cfg);

        // Level orientation (identity quaternion), gravity along -Z in world
        // In body frame with identity rotation, accel reads [0, 0, -G0] for gravity
        // But we'll use [0, 0, G0] convention (z-up, accel reads +G0 when level)
        // gravity_removal subtracts the expected gravity component
        let linear = proc.gravity_removal(&[0.0, 0.0, -G0]);
        // With identity quaternion, g_body = [0, 0, -G0], so linear = [0,0,-G0] - [0,0,-G0] = [0,0,0]
        assert!(linear[0].abs() < EPSILON_LOOSE);
        assert!(linear[1].abs() < EPSILON_LOOSE);
        assert!(linear[2].abs() < EPSILON_LOOSE);
    }

    // -----------------------------------------------------------------------
    // Dead reckoning
    // -----------------------------------------------------------------------

    #[test]
    fn test_dead_reckoning_constant_accel() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let dt = 0.01;
        let accel = 1.0; // 1 m/s^2

        // Apply constant acceleration for 1 second
        for _ in 0..100 {
            proc.dead_reckoning(&[accel, 0.0, 0.0], dt);
        }

        // v = a * t = 1.0 m/s
        assert!((proc.velocity[0] - 1.0).abs() < EPSILON_LOOSE);
        // p = 0.5 * a * t^2 = 0.5 m (approximately, with discrete integration)
        assert!((proc.position[0] - 0.5).abs() < 0.02);
    }

    #[test]
    fn test_dead_reckoning_zero_accel() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let dt = 0.01;

        for _ in 0..100 {
            proc.dead_reckoning(&[0.0, 0.0, 0.0], dt);
        }
        assert!(proc.velocity[0].abs() < EPSILON);
        assert!(proc.position[0].abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Noise analysis
    // -----------------------------------------------------------------------

    #[test]
    fn test_noise_density_to_rms() {
        // 100 µg/√Hz at 50 Hz bandwidth
        let rms = ImuProcessor::noise_density_to_rms(100.0, 50.0);
        // Expected: 100 * sqrt(50) ≈ 707 µg
        assert!((rms - 100.0 * 50.0f64.sqrt()).abs() < 0.01);
    }

    #[test]
    fn test_noise_density_to_rms_zero_bw() {
        let rms = ImuProcessor::noise_density_to_rms(100.0, 0.0);
        assert!(rms.abs() < EPSILON);
    }

    #[test]
    fn test_allan_variance_constant() {
        // Constant signal should have very low Allan variance
        let data: Vec<f64> = vec![1.0; 1000];
        let result = ImuProcessor::allan_variance(&data, 100.0, 10);
        for (_, avar, _) in &result {
            assert!(*avar < EPSILON);
        }
    }

    #[test]
    fn test_allan_variance_white_noise() {
        // White noise: ADEV ∝ 1/√τ, so AVAR ∝ 1/τ
        // Use a deterministic pseudo-random sequence
        let n = 10000;
        let mut data = vec![0.0f64; n];
        let mut rng_state: u64 = 12345;
        for i in 0..n {
            // Simple LCG PRNG
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u = (rng_state >> 33) as f64 / (1u64 << 31) as f64;
            data[i] = u - 0.5; // center around zero
        }

        let result = ImuProcessor::allan_variance(&data, 1000.0, 20);
        assert!(!result.is_empty());
        // At short tau, AVAR should be larger than at long tau (for white noise)
        if result.len() >= 2 {
            assert!(result[0].1 > result[result.len() - 1].1);
        }
    }

    #[test]
    fn test_allan_variance_too_short() {
        let data = vec![1.0, 2.0]; // only 2 samples
        let result = ImuProcessor::allan_variance(&data, 100.0, 5);
        // Should be empty or have very few entries
        assert!(result.len() <= 1);
    }

    // -----------------------------------------------------------------------
    // Vibration rectification
    // -----------------------------------------------------------------------

    #[test]
    fn test_vibration_rectification() {
        let k2 = 0.001; // 0.1% nonlinearity
        let vib_rms = 1.0; // 1g RMS
        let error = ImuProcessor::vibration_rectification(k2, vib_rms);
        assert!((error - 0.001).abs() < EPSILON);
    }

    #[test]
    fn test_vibration_rectification_zero() {
        let error = ImuProcessor::vibration_rectification(0.001, 0.0);
        assert!(error.abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Step detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_step_detection_walking() {
        // Simulate walking: place explicit sharp peaks at known positions
        let sample_rate = 100.0;
        let n = 300; // 3 seconds
        let mut accel_data = Vec::new();
        let mut timestamps = Vec::new();

        // Place step impulses at samples 50, 100, 150, 200, 250
        // (i.e., at t = 0.5, 1.0, 1.5, 2.0, 2.5 seconds, every 0.5s)
        let step_samples = [50usize, 100, 150, 200, 250];

        for i in 0..n {
            let t = i as f64 / sample_rate;
            timestamps.push(t);

            let az = if step_samples.contains(&i) {
                // Peak sample: 1.5g above gravity
                G0 + G0 * 0.5
            } else if step_samples.iter().any(|&s| i == s + 1 || (i > 0 && i == s - 1)) {
                // Neighbors of peak: slightly elevated but below peak
                G0 + G0 * 0.15
            } else {
                G0 // normal gravity
            };
            accel_data.push([0.0, 0.0, az]);
        }

        let steps = ImuProcessor::step_detection(&accel_data, &timestamps, 0.2, 0.3);
        // Expect 5 steps (refractory 0.3s, step spacing 0.5s)
        assert_eq!(steps.len(), 5, "Expected 5 steps, got {}", steps.len());
    }

    #[test]
    fn test_step_detection_stationary() {
        // Stationary: no steps
        let n = 100;
        let mut accel_data = Vec::new();
        let mut timestamps = Vec::new();
        for i in 0..n {
            timestamps.push(i as f64 / 100.0);
            accel_data.push([0.0, 0.0, G0]);
        }
        let steps = ImuProcessor::step_detection(&accel_data, &timestamps, 0.3, 0.3);
        assert!(steps.is_empty());
    }

    #[test]
    fn test_step_detection_refractory() {
        // Two peaks close together should be merged
        let mut accel_data = Vec::new();
        let mut timestamps = Vec::new();
        for i in 0..100 {
            let t = i as f64 / 100.0;
            timestamps.push(t);
            let az = if i == 10 || i == 12 {
                G0 + 5.0 // Two peaks 0.02s apart
            } else {
                G0
            };
            accel_data.push([0.0, 0.0, az]);
        }
        let steps = ImuProcessor::step_detection(&accel_data, &timestamps, 0.3, 0.3);
        assert_eq!(steps.len(), 1); // refractory prevents double count
    }

    // -----------------------------------------------------------------------
    // Sensor fusion weights
    // -----------------------------------------------------------------------

    #[test]
    fn test_sensor_fusion_weights_equal() {
        let (wa, wb) = ImuProcessor::sensor_fusion_weights(1.0, 1.0);
        assert!((wa - 0.5).abs() < EPSILON);
        assert!((wb - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_sensor_fusion_weights_unequal() {
        let (wa, wb) = ImuProcessor::sensor_fusion_weights(1.0, 3.0);
        // Lower variance sensor gets higher weight
        assert!((wa - 0.75).abs() < EPSILON);
        assert!((wb - 0.25).abs() < EPSILON);
    }

    #[test]
    fn test_sensor_fusion_weights_zero() {
        let (wa, wb) = ImuProcessor::sensor_fusion_weights(0.0, 0.0);
        assert!((wa - 0.5).abs() < EPSILON);
        assert!((wb - 0.5).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Low-pass filter
    // -----------------------------------------------------------------------

    #[test]
    fn test_low_pass_filter_convergence() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let alpha = 0.1;

        // Step input: should converge to the step value
        for _ in 0..200 {
            proc.low_pass_filter(&[1.0, 2.0, 3.0], alpha);
        }

        assert!((proc.lp_state[0] - 1.0).abs() < 0.01);
        assert!((proc.lp_state[1] - 2.0).abs() < 0.01);
        assert!((proc.lp_state[2] - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_low_pass_filter_alpha_1() {
        // Alpha=1 means no filtering (output = input)
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);
        let result = proc.low_pass_filter(&[5.0, 6.0, 7.0], 1.0);
        assert!((result[0] - 5.0).abs() < EPSILON);
        assert!((result[1] - 6.0).abs() < EPSILON);
        assert!((result[2] - 7.0).abs() < EPSILON);
    }

    #[test]
    fn test_lp_alpha_computation() {
        let alpha = ImuProcessor::lp_alpha(10.0, 100.0);
        // fc=10Hz, fs=100Hz: rc=1/(2*pi*10)≈0.0159, dt=0.01
        // alpha = 0.01 / (0.0159 + 0.01) ≈ 0.386
        assert!(alpha > 0.3 && alpha < 0.5);
    }

    // -----------------------------------------------------------------------
    // Processor lifecycle
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_reset() {
        let cfg = ImuConfig::new(AccelRange::G4, GyroRange::Dps500, 100.0);
        let mut proc = ImuProcessor::new(cfg);

        // Modify state
        proc.velocity = [1.0, 2.0, 3.0];
        proc.position = [4.0, 5.0, 6.0];
        proc.step_count = 10;

        // Reset
        proc.reset();
        assert!(proc.velocity[0].abs() < EPSILON);
        assert!(proc.position[0].abs() < EPSILON);
        assert_eq!(proc.step_count, 0);
        assert!((proc.orientation[0] - 1.0).abs() < EPSILON);
    }
}

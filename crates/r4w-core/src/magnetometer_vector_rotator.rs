//! 3-axis magnetometer calibration and rotation matrix operations.
//!
//! This module provides hard-iron and soft-iron calibration for 3-axis
//! magnetometers, heading/azimuth estimation, tilt compensation using
//! accelerometer data, magnetic declination correction, and rotation
//! matrix utilities based on Euler angles.
//!
//! Complex numbers are represented as `(f64, f64)` tuples `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::magnetometer_vector_rotator::{MagnetometerCalibrator, Vec3};
//!
//! // Raw magnetometer samples (would normally come from sensor)
//! let samples = vec![
//!     Vec3 { x:  250.0, y:   10.0, z:  -5.0 },
//!     Vec3 { x: -230.0, y:   15.0, z:   0.0 },
//!     Vec3 { x:   10.0, y:  260.0, z:   5.0 },
//!     Vec3 { x:    5.0, y: -240.0, z:  -3.0 },
//!     Vec3 { x:   12.0, y:    8.0, z: 270.0 },
//!     Vec3 { x:    8.0, y:   12.0, z:-250.0 },
//! ];
//!
//! let mut cal = MagnetometerCalibrator::new();
//! cal.add_samples(&samples);
//! cal.compute_calibration();
//!
//! // Apply calibration to a reading
//! let raw = Vec3 { x: 250.0, y: 10.0, z: -5.0 };
//! let calibrated = cal.apply(&raw);
//!
//! // Compute heading from calibrated 2D data
//! let heading_deg = calibrated.heading_2d_deg();
//! assert!(heading_deg >= 0.0 && heading_deg < 360.0);
//! ```

use std::f64::consts::PI;

/// A 3-component vector (x, y, z).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    /// Create a new vector.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Zero vector.
    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Vector magnitude (L2 norm).
    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalize to unit length. Returns zero vector if magnitude is zero.
    pub fn normalized(&self) -> Self {
        let m = self.magnitude();
        if m < 1e-15 {
            Self::zero()
        } else {
            Self {
                x: self.x / m,
                y: self.y / m,
                z: self.z / m,
            }
        }
    }

    /// Dot product.
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product.
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Compute heading from 2D magnetometer (x=North, y=East) in degrees [0, 360).
    ///
    /// Uses `atan2(y, x)` and converts to compass bearing.
    pub fn heading_2d_deg(&self) -> f64 {
        let angle_rad = self.y.atan2(self.x);
        let mut deg = angle_rad.to_degrees();
        if deg < 0.0 {
            deg += 360.0;
        }
        deg
    }

    /// Magnetic inclination (dip angle) in degrees.
    ///
    /// The angle between the horizontal plane and the field vector.
    /// Positive means the field dips below the horizontal (typical in
    /// the northern hemisphere).
    pub fn inclination_deg(&self) -> f64 {
        let horiz = (self.x * self.x + self.y * self.y).sqrt();
        // inclination is measured as positive downward (into the earth)
        (-self.z).atan2(horiz).to_degrees()
    }

    /// Element-wise subtraction.
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    /// Element-wise addition.
    pub fn add(&self, other: &Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    /// Scale by a scalar.
    pub fn scale(&self, s: f64) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        }
    }

    /// Convert to complex representation on the XY plane: `(x, y)`.
    pub fn to_complex_xy(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}

/// A 3x3 matrix stored in row-major order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3 {
    /// Row-major: `m[row][col]`
    pub m: [[f64; 3]; 3],
}

impl Mat3 {
    /// Identity matrix.
    pub fn identity() -> Self {
        Self {
            m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    /// Diagonal matrix from three values.
    pub fn diagonal(d0: f64, d1: f64, d2: f64) -> Self {
        Self {
            m: [[d0, 0.0, 0.0], [0.0, d1, 0.0], [0.0, 0.0, d2]],
        }
    }

    /// Multiply matrix by a vector.
    pub fn mul_vec(&self, v: &Vec3) -> Vec3 {
        Vec3 {
            x: self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            y: self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            z: self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        }
    }

    /// Matrix multiplication: self * other.
    pub fn mul_mat(&self, other: &Self) -> Self {
        let mut result = [[0.0f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    result[i][j] += self.m[i][k] * other.m[k][j];
                }
            }
        }
        Self { m: result }
    }

    /// Transpose.
    pub fn transpose(&self) -> Self {
        Self {
            m: [
                [self.m[0][0], self.m[1][0], self.m[2][0]],
                [self.m[0][1], self.m[1][1], self.m[2][1]],
                [self.m[0][2], self.m[1][2], self.m[2][2]],
            ],
        }
    }

    /// Determinant.
    pub fn determinant(&self) -> f64 {
        let m = &self.m;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Build rotation matrix from Euler angles (ZYX intrinsic / aerospace convention).
    ///
    /// - `yaw` (psi): rotation about Z axis (radians)
    /// - `pitch` (theta): rotation about Y axis (radians)
    /// - `roll` (phi): rotation about X axis (radians)
    ///
    /// Returns R = Rz(yaw) * Ry(pitch) * Rx(roll).
    pub fn from_euler_zyx(yaw: f64, pitch: f64, roll: f64) -> Self {
        let (sy, cy) = yaw.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sr, cr) = roll.sin_cos();

        Self {
            m: [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
        }
    }

    /// Build a rotation matrix about the Z axis.
    pub fn rotation_z(angle_rad: f64) -> Self {
        let (s, c) = angle_rad.sin_cos();
        Self {
            m: [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    /// Build a rotation matrix about the Y axis.
    pub fn rotation_y(angle_rad: f64) -> Self {
        let (s, c) = angle_rad.sin_cos();
        Self {
            m: [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
        }
    }

    /// Build a rotation matrix about the X axis.
    pub fn rotation_x(angle_rad: f64) -> Self {
        let (s, c) = angle_rad.sin_cos();
        Self {
            m: [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
        }
    }

    /// Scale each element by a scalar.
    pub fn scale(&self, s: f64) -> Self {
        let mut result = self.m;
        for row in &mut result {
            for val in row.iter_mut() {
                *val *= s;
            }
        }
        Self { m: result }
    }
}

/// Magnetometer calibrator for hard-iron and soft-iron correction.
///
/// Collects raw 3-axis magnetometer samples, estimates calibration
/// parameters, and applies corrections to produce calibrated readings.
#[derive(Debug, Clone)]
pub struct MagnetometerCalibrator {
    samples: Vec<Vec3>,
    /// Hard-iron offset (bias on each axis).
    hard_iron_offset: Vec3,
    /// Soft-iron correction matrix.
    soft_iron_matrix: Mat3,
    /// Whether `compute_calibration` has been called.
    calibrated: bool,
}

impl MagnetometerCalibrator {
    /// Create a new (uncalibrated) calibrator.
    pub fn new() -> Self {
        Self {
            samples: Vec::new(),
            hard_iron_offset: Vec3::zero(),
            soft_iron_matrix: Mat3::identity(),
            calibrated: false,
        }
    }

    /// Add a single raw magnetometer sample.
    pub fn add_sample(&mut self, sample: Vec3) {
        self.samples.push(sample);
        self.calibrated = false;
    }

    /// Add multiple raw magnetometer samples.
    pub fn add_samples(&mut self, samples: &[Vec3]) {
        self.samples.extend_from_slice(samples);
        self.calibrated = false;
    }

    /// Number of collected samples.
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }

    /// Compute calibration parameters from collected samples.
    ///
    /// Hard-iron offset is estimated as the midpoint of the min/max on
    /// each axis. Soft-iron correction scales each axis so that the
    /// calibrated data forms a sphere of radius equal to the average
    /// semi-axis length.
    pub fn compute_calibration(&mut self) {
        if self.samples.is_empty() {
            return;
        }

        // --- Hard-iron: midpoint of min/max on each axis ---
        let mut min_v = Vec3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max_v = Vec3::new(f64::MIN, f64::MIN, f64::MIN);

        for s in &self.samples {
            if s.x < min_v.x {
                min_v.x = s.x;
            }
            if s.y < min_v.y {
                min_v.y = s.y;
            }
            if s.z < min_v.z {
                min_v.z = s.z;
            }
            if s.x > max_v.x {
                max_v.x = s.x;
            }
            if s.y > max_v.y {
                max_v.y = s.y;
            }
            if s.z > max_v.z {
                max_v.z = s.z;
            }
        }

        self.hard_iron_offset = Vec3::new(
            (min_v.x + max_v.x) / 2.0,
            (min_v.y + max_v.y) / 2.0,
            (min_v.z + max_v.z) / 2.0,
        );

        // --- Soft-iron: scale each axis to average radius ---
        let range_x = (max_v.x - min_v.x) / 2.0;
        let range_y = (max_v.y - min_v.y) / 2.0;
        let range_z = (max_v.z - min_v.z) / 2.0;

        let avg_radius = (range_x + range_y + range_z) / 3.0;

        if avg_radius < 1e-15 || range_x < 1e-15 || range_y < 1e-15 || range_z < 1e-15 {
            self.soft_iron_matrix = Mat3::identity();
        } else {
            self.soft_iron_matrix = Mat3::diagonal(
                avg_radius / range_x,
                avg_radius / range_y,
                avg_radius / range_z,
            );
        }

        self.calibrated = true;
    }

    /// Apply calibration (hard-iron removal + soft-iron correction) to a raw reading.
    pub fn apply(&self, raw: &Vec3) -> Vec3 {
        let centered = raw.sub(&self.hard_iron_offset);
        self.soft_iron_matrix.mul_vec(&centered)
    }

    /// Return the estimated hard-iron offset.
    pub fn hard_iron_offset(&self) -> Vec3 {
        self.hard_iron_offset
    }

    /// Return the soft-iron correction matrix.
    pub fn soft_iron_matrix(&self) -> Mat3 {
        self.soft_iron_matrix
    }

    /// Whether calibration has been computed.
    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Calibration quality metric: sphericity.
    ///
    /// After applying calibration to all stored samples, computes the
    /// ratio of minimum to maximum distance from the origin. A perfect
    /// sphere yields 1.0; highly ellipsoidal data yields a value closer
    /// to 0.0.
    pub fn sphericity(&self) -> f64 {
        if self.samples.len() < 2 {
            return 0.0;
        }

        let mut min_r = f64::MAX;
        let mut max_r = f64::MIN;

        for s in &self.samples {
            let cal = self.apply(s);
            let r = cal.magnitude();
            if r < min_r {
                min_r = r;
            }
            if r > max_r {
                max_r = r;
            }
        }

        if max_r < 1e-15 {
            return 0.0;
        }

        min_r / max_r
    }
}

impl Default for MagnetometerCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute heading from calibrated magnetometer data with tilt compensation.
///
/// Given calibrated magnetometer vector `mag` and accelerometer vector `accel`
/// (both in the sensor body frame), computes the tilt-compensated magnetic
/// heading in degrees [0, 360).
///
/// # Coordinate convention
///
/// - X: forward (North in level, aligned orientation)
/// - Y: right (East)
/// - Z: down
///
/// Accelerometer measures gravity (positive Z when level).
pub fn tilt_compensated_heading(mag: &Vec3, accel: &Vec3) -> f64 {
    // Pitch (rotation about Y) and roll (rotation about X) from accelerometer
    let pitch = (-accel.x).atan2(accel.z);
    let roll = accel.y.atan2((accel.x * accel.x + accel.z * accel.z).sqrt());

    let (sp, cp) = pitch.sin_cos();
    let (sr, cr) = roll.sin_cos();

    // Tilt-compensated magnetic components in the horizontal plane
    let mx_h = mag.x * cp + mag.y * sp * sr + mag.z * sp * cr;
    let my_h = mag.y * cr - mag.z * sr;

    let heading_rad = my_h.atan2(mx_h);
    let mut heading_deg = heading_rad.to_degrees();
    if heading_deg < 0.0 {
        heading_deg += 360.0;
    }
    heading_deg
}

/// Apply magnetic declination correction to a heading.
///
/// `heading_deg`: magnetic heading in degrees [0, 360)
/// `declination_deg`: magnetic declination (positive east, negative west)
///
/// Returns true heading in degrees [0, 360).
pub fn apply_declination(heading_deg: f64, declination_deg: f64) -> f64 {
    let true_heading = heading_deg + declination_deg;
    // Wrap to [0, 360)
    true_heading.rem_euclid(360.0)
}

/// Rotate a vector using Euler angles (ZYX convention).
pub fn rotate_euler_zyx(v: &Vec3, yaw: f64, pitch: f64, roll: f64) -> Vec3 {
    let r = Mat3::from_euler_zyx(yaw, pitch, roll);
    r.mul_vec(v)
}

/// Multiply two complex numbers represented as `(re, im)` tuples.
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Magnitude of a complex number `(re, im)`.
pub fn complex_mag(c: (f64, f64)) -> f64 {
    (c.0 * c.0 + c.1 * c.1).sqrt()
}

/// Phase angle of a complex number `(re, im)` in radians.
pub fn complex_phase(c: (f64, f64)) -> f64 {
    c.1.atan2(c.0)
}

/// Convert a 2D rotation angle to a complex number `(cos theta, sin theta)`.
pub fn rotation_to_complex(angle_rad: f64) -> (f64, f64) {
    (angle_rad.cos(), angle_rad.sin())
}

/// Rotate a 2D point (as complex number) by an angle.
pub fn rotate_complex_2d(point: (f64, f64), angle_rad: f64) -> (f64, f64) {
    complex_mul(point, rotation_to_complex(angle_rad))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const TOL: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn vec3_approx_eq(a: &Vec3, b: &Vec3, tol: f64) -> bool {
        approx_eq(a.x, b.x, tol) && approx_eq(a.y, b.y, tol) && approx_eq(a.z, b.z, tol)
    }

    // ------------------------------------------------------------------
    // Test 1: Vec3 magnitude
    // ------------------------------------------------------------------
    #[test]
    fn test_vec3_magnitude() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert!(approx_eq(v.magnitude(), 5.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 2: Vec3 normalized
    // ------------------------------------------------------------------
    #[test]
    fn test_vec3_normalized() {
        let v = Vec3::new(0.0, 0.0, 5.0);
        let n = v.normalized();
        assert!(approx_eq(n.z, 1.0, TOL));
        assert!(approx_eq(n.magnitude(), 1.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 3: Vec3 dot product
    // ------------------------------------------------------------------
    #[test]
    fn test_vec3_dot() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert!(approx_eq(a.dot(&b), 32.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 4: Vec3 cross product
    // ------------------------------------------------------------------
    #[test]
    fn test_vec3_cross() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = x.cross(&y);
        assert!(vec3_approx_eq(&z, &Vec3::new(0.0, 0.0, 1.0), TOL));
    }

    // ------------------------------------------------------------------
    // Test 5: heading_2d_deg north
    // ------------------------------------------------------------------
    #[test]
    fn test_heading_2d_north() {
        // Pointing along +X => 0 degrees
        let v = Vec3::new(1.0, 0.0, 0.0);
        assert!(approx_eq(v.heading_2d_deg(), 0.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 6: heading_2d_deg east
    // ------------------------------------------------------------------
    #[test]
    fn test_heading_2d_east() {
        // Pointing along +Y => 90 degrees
        let v = Vec3::new(0.0, 1.0, 0.0);
        assert!(approx_eq(v.heading_2d_deg(), 90.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 7: heading_2d_deg south
    // ------------------------------------------------------------------
    #[test]
    fn test_heading_2d_south() {
        let v = Vec3::new(-1.0, 0.0, 0.0);
        assert!(approx_eq(v.heading_2d_deg(), 180.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 8: heading_2d_deg west
    // ------------------------------------------------------------------
    #[test]
    fn test_heading_2d_west() {
        // Pointing along -Y => 270 degrees
        let v = Vec3::new(0.0, -1.0, 0.0);
        assert!(approx_eq(v.heading_2d_deg(), 270.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 9: Mat3 identity mul_vec
    // ------------------------------------------------------------------
    #[test]
    fn test_mat3_identity_mul_vec() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let result = Mat3::identity().mul_vec(&v);
        assert!(vec3_approx_eq(&result, &v, TOL));
    }

    // ------------------------------------------------------------------
    // Test 10: Mat3 rotation_z 90 degrees
    // ------------------------------------------------------------------
    #[test]
    fn test_mat3_rotation_z_90() {
        let r = Mat3::rotation_z(FRAC_PI_2);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let result = r.mul_vec(&v);
        // Rotating x-hat 90 degrees about z => y-hat
        assert!(vec3_approx_eq(&result, &Vec3::new(0.0, 1.0, 0.0), TOL));
    }

    // ------------------------------------------------------------------
    // Test 11: Mat3 determinant of identity
    // ------------------------------------------------------------------
    #[test]
    fn test_mat3_determinant_identity() {
        assert!(approx_eq(Mat3::identity().determinant(), 1.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 12: Mat3 mul_mat identity
    // ------------------------------------------------------------------
    #[test]
    fn test_mat3_mul_mat_identity() {
        let a = Mat3::diagonal(2.0, 3.0, 4.0);
        let result = a.mul_mat(&Mat3::identity());
        assert_eq!(result, a);
    }

    // ------------------------------------------------------------------
    // Test 13: Euler ZYX identity (all zeros)
    // ------------------------------------------------------------------
    #[test]
    fn test_euler_zyx_identity() {
        let r = Mat3::from_euler_zyx(0.0, 0.0, 0.0);
        let v = Vec3::new(1.0, 2.0, 3.0);
        let result = r.mul_vec(&v);
        assert!(vec3_approx_eq(&result, &v, TOL));
    }

    // ------------------------------------------------------------------
    // Test 14: Hard-iron calibration
    // ------------------------------------------------------------------
    #[test]
    fn test_hard_iron_calibration() {
        let mut cal = MagnetometerCalibrator::new();
        // Samples with a +10 offset on X
        cal.add_samples(&[
            Vec3::new(110.0, 100.0, 100.0),
            Vec3::new(-90.0, -100.0, -100.0),
            Vec3::new(10.0, 100.0, -100.0),
            Vec3::new(10.0, -100.0, 100.0),
        ]);
        cal.compute_calibration();
        let offset = cal.hard_iron_offset();
        // Hard-iron offset should be (10, 0, 0)
        assert!(approx_eq(offset.x, 10.0, TOL));
        assert!(approx_eq(offset.y, 0.0, TOL));
        assert!(approx_eq(offset.z, 0.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 15: Soft-iron correction makes axes equal
    // ------------------------------------------------------------------
    #[test]
    fn test_soft_iron_correction() {
        let mut cal = MagnetometerCalibrator::new();
        // X axis range 200 (+-100), Y range 100 (+-50), Z range 200 (+-100)
        cal.add_samples(&[
            Vec3::new(100.0, 50.0, 100.0),
            Vec3::new(-100.0, -50.0, -100.0),
        ]);
        cal.compute_calibration();

        // Apply to a sample at +max on each axis
        let cal_pos = cal.apply(&Vec3::new(100.0, 50.0, 100.0));
        // After soft-iron, all axes should have the same magnitude
        // Average radius = (100 + 50 + 100)/3 = 83.333...
        let avg_r = (100.0 + 50.0 + 100.0) / 3.0;
        assert!(approx_eq(cal_pos.x, avg_r, TOL));
        assert!(approx_eq(cal_pos.y, avg_r, TOL));
        assert!(approx_eq(cal_pos.z, avg_r, TOL));
    }

    // ------------------------------------------------------------------
    // Test 16: Sphericity of perfect sphere data
    // ------------------------------------------------------------------
    #[test]
    fn test_sphericity_perfect() {
        let mut cal = MagnetometerCalibrator::new();
        // Generate points on a perfect sphere (radius 100)
        let n = 50;
        for i in 0..n {
            let theta = PI * (i as f64) / (n as f64);
            for j in 0..n {
                let phi = 2.0 * PI * (j as f64) / (n as f64);
                cal.add_sample(Vec3::new(
                    100.0 * theta.sin() * phi.cos(),
                    100.0 * theta.sin() * phi.sin(),
                    100.0 * theta.cos(),
                ));
            }
        }
        cal.compute_calibration();
        let s = cal.sphericity();
        // Should be very close to 1.0 for a perfect sphere
        assert!(s > 0.95, "sphericity was {}", s);
    }

    // ------------------------------------------------------------------
    // Test 17: Tilt-compensated heading (level sensor)
    // ------------------------------------------------------------------
    #[test]
    fn test_tilt_compensated_heading_level() {
        // Sensor is level, gravity along +Z
        let accel = Vec3::new(0.0, 0.0, 1.0);
        // Magnetic field pointing North (+X)
        let mag = Vec3::new(1.0, 0.0, 0.0);
        let heading = tilt_compensated_heading(&mag, &accel);
        assert!(
            approx_eq(heading, 0.0, 0.1),
            "heading was {}",
            heading
        );
    }

    // ------------------------------------------------------------------
    // Test 18: Declination correction
    // ------------------------------------------------------------------
    #[test]
    fn test_declination_correction() {
        // Magnetic heading 350, declination +15 => true heading 5
        let true_h = apply_declination(350.0, 15.0);
        assert!(approx_eq(true_h, 5.0, TOL));

        // Magnetic heading 10, declination -20 => true heading 350
        let true_h2 = apply_declination(10.0, -20.0);
        assert!(approx_eq(true_h2, 350.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 19: rotate_euler_zyx round-trip
    // ------------------------------------------------------------------
    #[test]
    fn test_rotate_euler_round_trip() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let yaw = 0.3;
        let pitch = 0.2;
        let roll = 0.1;
        let rotated = rotate_euler_zyx(&v, yaw, pitch, roll);
        // Inverse rotation: transpose (since rotation matrices are orthogonal)
        let r_inv = Mat3::from_euler_zyx(yaw, pitch, roll).transpose();
        let recovered = r_inv.mul_vec(&rotated);
        assert!(
            vec3_approx_eq(&recovered, &v, 1e-9),
            "recovered: {:?}",
            recovered
        );
    }

    // ------------------------------------------------------------------
    // Test 20: Complex multiplication
    // ------------------------------------------------------------------
    #[test]
    fn test_complex_mul() {
        // (1 + 2i) * (3 + 4i) = (3-8) + (4+6)i = (-5, 10)
        let result = complex_mul((1.0, 2.0), (3.0, 4.0));
        assert!(approx_eq(result.0, -5.0, TOL));
        assert!(approx_eq(result.1, 10.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 21: Complex magnitude and phase
    // ------------------------------------------------------------------
    #[test]
    fn test_complex_mag_phase() {
        let c = (3.0, 4.0);
        assert!(approx_eq(complex_mag(c), 5.0, TOL));
        assert!(approx_eq(complex_phase(c), (4.0f64).atan2(3.0), TOL));
    }

    // ------------------------------------------------------------------
    // Test 22: Rotation via complex 2D
    // ------------------------------------------------------------------
    #[test]
    fn test_rotate_complex_2d() {
        // Rotate (1, 0) by 90 degrees => (0, 1)
        let result = rotate_complex_2d((1.0, 0.0), FRAC_PI_2);
        assert!(approx_eq(result.0, 0.0, TOL));
        assert!(approx_eq(result.1, 1.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 23: Inclination
    // ------------------------------------------------------------------
    #[test]
    fn test_inclination() {
        // Pure horizontal field => 0 inclination
        let v = Vec3::new(1.0, 0.0, 0.0);
        assert!(approx_eq(v.inclination_deg(), 0.0, TOL));

        // Field pointing straight down (-Z in body = into earth)
        let v_down = Vec3::new(0.0, 0.0, -1.0);
        assert!(approx_eq(v_down.inclination_deg(), 90.0, TOL));
    }

    // ------------------------------------------------------------------
    // Test 24: Calibrator default and is_calibrated
    // ------------------------------------------------------------------
    #[test]
    fn test_calibrator_default_state() {
        let cal = MagnetometerCalibrator::default();
        assert!(!cal.is_calibrated());
        assert_eq!(cal.sample_count(), 0);
    }

    // ------------------------------------------------------------------
    // Test 25: Mat3 transpose is involution
    // ------------------------------------------------------------------
    #[test]
    fn test_mat3_transpose_involution() {
        let m = Mat3::from_euler_zyx(0.5, 0.3, 0.1);
        let m_tt = m.transpose().transpose();
        for i in 0..3 {
            for j in 0..3 {
                assert!(approx_eq(m.m[i][j], m_tt.m[i][j], TOL));
            }
        }
    }
}

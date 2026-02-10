//! Quaternion-based attitude estimation for spacecraft/vehicle orientation.
//!
//! This module provides quaternion algebra and attitude estimation algorithms
//! including gyroscope integration, TRIAD determination, complementary filtering,
//! and spherical linear interpolation (SLERP).
//!
//! # Example
//!
//! ```
//! use r4w_core::quaternion_attitude_tracker::{Quaternion, AttitudeTracker};
//!
//! // Create an identity quaternion (no rotation)
//! let q = Quaternion::identity();
//! assert!((q.norm() - 1.0).abs() < 1e-12);
//!
//! // Create a quaternion from Euler angles (roll=0, pitch=0, yaw=90 deg)
//! let q_yaw = Quaternion::from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2);
//! let (roll, pitch, yaw) = q_yaw.to_euler();
//! assert!((yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
//!
//! // Integrate a gyroscope reading over a time step
//! let mut tracker = AttitudeTracker::new();
//! tracker.gyro_update([0.0, 0.0, 0.1], 0.01); // 0.1 rad/s yaw rate, 10ms step
//! let attitude = tracker.quaternion();
//! assert!(attitude.norm() > 0.999);
//! ```

use std::f64::consts::PI;

// ─── Quaternion ──────────────────────────────────────────────────────────────

/// Unit quaternion representing a 3-D rotation.
///
/// Stored as `(w, x, y, z)` where `w` is the scalar part and `(x, y, z)` is
/// the vector part.  The convention is Hamilton multiplication with
/// right-hand rotations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    /// Create a new quaternion from components.
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// Identity quaternion (no rotation).
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Create a quaternion from an axis-angle representation.
    ///
    /// `axis` must be a unit vector; `angle` is in radians.
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Self {
        let half = angle / 2.0;
        let s = half.sin();
        Self {
            w: half.cos(),
            x: axis[0] * s,
            y: axis[1] * s,
            z: axis[2] * s,
        }
    }

    /// Convert to axis-angle representation.
    ///
    /// Returns `(axis, angle)`.  If the rotation angle is near zero the axis
    /// is returned as `[0, 0, 1]`.
    pub fn to_axis_angle(&self) -> ([f64; 3], f64) {
        let q = self.normalized();
        let angle = 2.0 * q.w.clamp(-1.0, 1.0).acos();
        let s = (1.0 - q.w * q.w).sqrt();
        if s < 1e-12 {
            ([0.0, 0.0, 1.0], angle)
        } else {
            ([q.x / s, q.y / s, q.z / s], angle)
        }
    }

    /// Create a quaternion from ZYX (yaw-pitch-roll) Euler angles.
    ///
    /// Rotation order: first yaw about Z, then pitch about Y, then roll about X.
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

    /// Extract ZYX Euler angles `(roll, pitch, yaw)` in radians.
    pub fn to_euler(&self) -> (f64, f64, f64) {
        let q = self.normalized();
        // roll (x-axis rotation)
        let sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        let cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // pitch (y-axis rotation)
        let sinp = 2.0 * (q.w * q.y - q.z * q.x);
        let pitch = if sinp.abs() >= 1.0 {
            (PI / 2.0).copysign(sinp) // gimbal lock
        } else {
            sinp.asin()
        };

        // yaw (z-axis rotation)
        let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }

    /// Build a quaternion from a 3x3 rotation matrix (row-major).
    ///
    /// Uses Shepperd's method for numerical robustness.
    pub fn from_rotation_matrix(m: &[[f64; 3]; 3]) -> Self {
        let trace = m[0][0] + m[1][1] + m[2][2];
        if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            Self {
                w: 0.25 / s,
                x: (m[2][1] - m[1][2]) * s,
                y: (m[0][2] - m[2][0]) * s,
                z: (m[1][0] - m[0][1]) * s,
            }
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = 2.0 * (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt();
            Self {
                w: (m[2][1] - m[1][2]) / s,
                x: 0.25 * s,
                y: (m[0][1] + m[1][0]) / s,
                z: (m[0][2] + m[2][0]) / s,
            }
        } else if m[1][1] > m[2][2] {
            let s = 2.0 * (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt();
            Self {
                w: (m[0][2] - m[2][0]) / s,
                x: (m[0][1] + m[1][0]) / s,
                y: 0.25 * s,
                z: (m[1][2] + m[2][1]) / s,
            }
        } else {
            let s = 2.0 * (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt();
            Self {
                w: (m[1][0] - m[0][1]) / s,
                x: (m[0][2] + m[2][0]) / s,
                y: (m[1][2] + m[2][1]) / s,
                z: 0.25 * s,
            }
        }
    }

    /// Convert to a 3x3 rotation matrix (row-major).
    pub fn to_rotation_matrix(&self) -> [[f64; 3]; 3] {
        let q = self.normalized();
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
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

    /// Squared norm (sum of squares of components).
    pub fn norm_squared(&self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Euclidean norm.
    pub fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }

    /// Return a normalised copy.
    pub fn normalized(&self) -> Self {
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

    /// Normalize in place.
    pub fn normalize(&mut self) {
        *self = self.normalized();
    }

    /// Conjugate: negate the vector part.
    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Multiplicative inverse.  For unit quaternions this equals the conjugate.
    pub fn inverse(&self) -> Self {
        let n2 = self.norm_squared();
        if n2 < 1e-30 {
            return Self::identity();
        }
        let c = self.conjugate();
        Self {
            w: c.w / n2,
            x: c.x / n2,
            y: c.y / n2,
            z: c.z / n2,
        }
    }

    /// Hamilton product `self * rhs`.
    pub fn mul(&self, rhs: &Quaternion) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }

    /// Scalar multiplication.
    pub fn scale(&self, s: f64) -> Self {
        Self {
            w: self.w * s,
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        }
    }

    /// Component-wise addition.
    pub fn add(&self, rhs: &Quaternion) -> Self {
        Self {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }

    /// Dot product (4-D inner product).
    pub fn dot(&self, rhs: &Quaternion) -> f64 {
        self.w * rhs.w + self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Rotate a 3-D vector by this unit quaternion: `v' = q v q*`.
    pub fn rotate_vector(&self, v: [f64; 3]) -> [f64; 3] {
        let qv = Quaternion::new(0.0, v[0], v[1], v[2]);
        let rotated = self.mul(&qv).mul(&self.conjugate());
        [rotated.x, rotated.y, rotated.z]
    }

    /// Spherical linear interpolation between `self` and `other`.
    ///
    /// `t` ranges from 0.0 (self) to 1.0 (other).
    pub fn slerp(&self, other: &Quaternion, t: f64) -> Self {
        let mut dot = self.dot(other);

        // Ensure shortest path
        let other = if dot < 0.0 {
            dot = -dot;
            other.scale(-1.0)
        } else {
            *other
        };

        // Fall back to linear interpolation for nearly identical quaternions
        if dot > 0.9995 {
            return self
                .scale(1.0 - t)
                .add(&other.scale(t))
                .normalized();
        }

        let theta_0 = dot.clamp(-1.0, 1.0).acos();
        let sin_theta_0 = theta_0.sin();

        let s0 = ((1.0 - t) * theta_0).sin() / sin_theta_0;
        let s1 = (t * theta_0).sin() / sin_theta_0;

        self.scale(s0).add(&other.scale(s1)).normalized()
    }

    /// Angle (in radians) between two unit quaternion orientations.
    ///
    /// Returns a value in `[0, pi]`.
    pub fn angle_to(&self, other: &Quaternion) -> f64 {
        let d = self.dot(other).abs().clamp(0.0, 1.0);
        2.0 * d.acos()
    }

    /// Build a *pure* quaternion from an angular-velocity vector `[wx, wy, wz]`.
    pub fn from_angular_velocity(omega: [f64; 3]) -> Self {
        Self {
            w: 0.0,
            x: omega[0],
            y: omega[1],
            z: omega[2],
        }
    }

    /// Quaternion time-derivative for angular velocity `omega` (body frame).
    ///
    /// `q_dot = 0.5 * q * omega_q` where `omega_q = (0, wx, wy, wz)`.
    pub fn kinematics(&self, omega: [f64; 3]) -> Self {
        let omega_q = Quaternion::from_angular_velocity(omega);
        self.mul(&omega_q).scale(0.5)
    }

    /// Estimate angular velocity from two consecutive quaternions separated by `dt`.
    ///
    /// Returns body-frame angular velocity `[wx, wy, wz]` in rad/s.
    pub fn angular_velocity_to(&self, next: &Quaternion, dt: f64) -> [f64; 3] {
        let dq = self.inverse().mul(next);
        let (axis, angle) = dq.to_axis_angle();
        let rate = angle / dt;
        [axis[0] * rate, axis[1] * rate, axis[2] * rate]
    }
}

// ─── Complex helper (f64,f64) ────────────────────────────────────────────────

/// Multiply two complex numbers represented as `(re, im)` tuples.
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Magnitude of a complex number.
pub fn complex_abs(c: (f64, f64)) -> f64 {
    (c.0 * c.0 + c.1 * c.1).sqrt()
}

/// Conjugate of a complex number.
pub fn complex_conj(c: (f64, f64)) -> (f64, f64) {
    (c.0, -c.1)
}

// ─── 3-D vector helpers ──────────────────────────────────────────────────────

fn vec3_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn vec3_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn vec3_norm(v: [f64; 3]) -> f64 {
    vec3_dot(v, v).sqrt()
}

fn vec3_normalize(v: [f64; 3]) -> [f64; 3] {
    let n = vec3_norm(v);
    if n < 1e-15 {
        return [0.0, 0.0, 0.0];
    }
    [v[0] / n, v[1] / n, v[2] / n]
}

#[allow(dead_code)]
fn vec3_scale(v: [f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

#[allow(dead_code)]
fn vec3_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

// ─── 3x3 matrix helpers ─────────────────────────────────────────────────────

fn mat3_transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

fn mat3_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    r
}

/// Build a 3x3 matrix from three column vectors.
fn mat3_from_cols(c0: [f64; 3], c1: [f64; 3], c2: [f64; 3]) -> [[f64; 3]; 3] {
    [
        [c0[0], c1[0], c2[0]],
        [c0[1], c1[1], c2[1]],
        [c0[2], c1[2], c2[2]],
    ]
}

// ─── TRIAD algorithm ─────────────────────────────────────────────────────────

/// Compute attitude (rotation matrix) from two pairs of reference/body vectors
/// using the TRIAD algorithm.
///
/// * `ref1, ref2` - two non-parallel unit vectors in the reference frame
/// * `body1, body2` - corresponding unit vectors in the body frame
///
/// Returns a 3x3 rotation matrix `R` such that `body = R * ref`.
pub fn triad(
    ref1: [f64; 3],
    ref2: [f64; 3],
    body1: [f64; 3],
    body2: [f64; 3],
) -> [[f64; 3]; 3] {
    // Reference triad
    let r1 = vec3_normalize(ref1);
    let r2 = vec3_normalize(vec3_cross(ref1, ref2));
    let r3 = vec3_cross(r1, r2);

    // Body triad
    let b1 = vec3_normalize(body1);
    let b2 = vec3_normalize(vec3_cross(body1, body2));
    let b3 = vec3_cross(b1, b2);

    let m_body = mat3_from_cols(b1, b2, b3);
    let m_ref = mat3_from_cols(r1, r2, r3);
    let m_ref_t = mat3_transpose(&m_ref);

    mat3_mul(&m_body, &m_ref_t)
}

/// Convenience: TRIAD returning a quaternion.
pub fn triad_quaternion(
    ref1: [f64; 3],
    ref2: [f64; 3],
    body1: [f64; 3],
    body2: [f64; 3],
) -> Quaternion {
    let r = triad(ref1, ref2, body1, body2);
    Quaternion::from_rotation_matrix(&r).normalized()
}

// ─── AttitudeTracker (complementary filter) ──────────────────────────────────

/// Quaternion-based attitude tracker with complementary filter fusion.
///
/// Integrates gyroscope readings for fast updates and corrects long-term drift
/// using accelerometer (gravity) and magnetometer (north) reference vectors.
pub struct AttitudeTracker {
    q: Quaternion,
    /// Complementary-filter gain for reference-vector correction (0..1).
    /// 0 = pure gyro, 1 = pure reference.
    pub alpha: f64,
}

impl AttitudeTracker {
    /// Create a new tracker starting at the identity orientation.
    pub fn new() -> Self {
        Self {
            q: Quaternion::identity(),
            alpha: 0.02,
        }
    }

    /// Create a tracker with a custom initial quaternion and filter gain.
    pub fn with_initial(q: Quaternion, alpha: f64) -> Self {
        Self {
            q: q.normalized(),
            alpha: alpha.clamp(0.0, 1.0),
        }
    }

    /// Current attitude quaternion.
    pub fn quaternion(&self) -> Quaternion {
        self.q
    }

    /// Current attitude as Euler angles `(roll, pitch, yaw)`.
    pub fn euler(&self) -> (f64, f64, f64) {
        self.q.to_euler()
    }

    /// Pure gyroscope integration step (first-order).
    ///
    /// `gyro` is `[wx, wy, wz]` in rad/s (body frame), `dt` in seconds.
    pub fn gyro_update(&mut self, gyro: [f64; 3], dt: f64) {
        let q_dot = self.q.kinematics(gyro);
        self.q = self.q.add(&q_dot.scale(dt)).normalized();
    }

    /// Complementary filter update: gyro integration + gravity correction.
    ///
    /// `accel` should be the measured accelerometer vector (body frame,
    /// approximately `[0, 0, -g]` when stationary, but need not be normalised).
    pub fn update_with_accel(&mut self, gyro: [f64; 3], accel: [f64; 3], dt: f64) {
        // 1. Gyro propagation
        self.gyro_update(gyro, dt);

        // 2. Gravity reference correction
        let accel_n = vec3_normalize(accel);
        if vec3_norm(accel_n) < 0.5 {
            return; // skip if accel data is degenerate
        }

        // Expected gravity direction in body frame from current estimate
        let gravity_ref = [0.0, 0.0, -1.0];
        let gravity_est = self.q.conjugate().rotate_vector(gravity_ref);

        // Rotation error between measured and estimated gravity
        let cross = vec3_cross(accel_n, gravity_est);
        let correction = Quaternion::new(
            1.0,
            cross[0] * self.alpha,
            cross[1] * self.alpha,
            cross[2] * self.alpha,
        );

        self.q = self.q.mul(&correction).normalized();
    }

    /// Full complementary filter update with accelerometer and magnetometer.
    ///
    /// `mag` is the measured magnetic field vector in the body frame.
    pub fn update_with_accel_mag(
        &mut self,
        gyro: [f64; 3],
        accel: [f64; 3],
        mag: [f64; 3],
        dt: f64,
    ) {
        // 1. Gyro + accel update
        self.update_with_accel(gyro, accel, dt);

        // 2. Magnetometer heading correction (yaw only)
        let mag_n = vec3_normalize(mag);
        if vec3_norm(mag_n) < 0.5 {
            return;
        }

        // Project magnetometer into horizontal plane using current tilt estimate
        let mag_world = self.q.rotate_vector(mag_n);
        let heading_meas = mag_world[1].atan2(mag_world[0]);

        // Expected heading = 0 (north along +X in world frame)
        let heading_err = heading_meas;

        // Small-angle yaw correction
        let half_corr = heading_err * self.alpha * 0.5;
        let yaw_correction = Quaternion::new(half_corr.cos(), 0.0, 0.0, half_corr.sin());
        self.q = yaw_correction.mul(&self.q).normalized();
    }

    /// Reset to identity orientation.
    pub fn reset(&mut self) {
        self.q = Quaternion::identity();
    }

    /// Set the attitude directly.
    pub fn set_quaternion(&mut self, q: Quaternion) {
        self.q = q.normalized();
    }
}

impl Default for AttitudeTracker {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    const EPS: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- Quaternion basics ---

    #[test]
    fn test_identity_norm() {
        let q = Quaternion::identity();
        assert!(approx_eq(q.norm(), 1.0, EPS));
    }

    #[test]
    fn test_quaternion_conjugate() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let c = q.conjugate();
        assert_eq!(c.w, 1.0);
        assert_eq!(c.x, -2.0);
        assert_eq!(c.y, -3.0);
        assert_eq!(c.z, -4.0);
    }

    #[test]
    fn test_quaternion_inverse() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let inv = q.inverse();
        let product = q.mul(&inv);
        assert!(approx_eq(product.w, 1.0, 1e-9));
        assert!(approx_eq(product.x, 0.0, 1e-9));
        assert!(approx_eq(product.y, 0.0, 1e-9));
        assert!(approx_eq(product.z, 0.0, 1e-9));
    }

    #[test]
    fn test_quaternion_multiplication_non_commutative() {
        let a = Quaternion::from_axis_angle([1.0, 0.0, 0.0], FRAC_PI_4);
        let b = Quaternion::from_axis_angle([0.0, 1.0, 0.0], FRAC_PI_4);
        let ab = a.mul(&b);
        let ba = b.mul(&a);
        // Hamilton product is not commutative for non-parallel axes
        let diff = (ab.w - ba.w).abs() + (ab.x - ba.x).abs() + (ab.y - ba.y).abs() + (ab.z - ba.z).abs();
        assert!(diff > 1e-6, "Quaternion multiplication should be non-commutative");
    }

    #[test]
    fn test_quaternion_normalization() {
        let q = Quaternion::new(1.0, 1.0, 1.0, 1.0);
        let n = q.normalized();
        assert!(approx_eq(n.norm(), 1.0, EPS));
        assert!(approx_eq(n.w, 0.5, EPS));
    }

    #[test]
    fn test_mul_by_conjugate_gives_norm_squared() {
        let q = Quaternion::new(2.0, 3.0, 4.0, 5.0);
        let product = q.mul(&q.conjugate());
        let expected_n2 = q.norm_squared();
        assert!(approx_eq(product.w, expected_n2, 1e-9));
        assert!(approx_eq(product.x, 0.0, 1e-9));
        assert!(approx_eq(product.y, 0.0, 1e-9));
        assert!(approx_eq(product.z, 0.0, 1e-9));
    }

    // --- Euler angle conversions ---

    #[test]
    fn test_euler_roundtrip_yaw() {
        let yaw = FRAC_PI_2;
        let q = Quaternion::from_euler(0.0, 0.0, yaw);
        let (r, p, y) = q.to_euler();
        assert!(approx_eq(r, 0.0, EPS));
        assert!(approx_eq(p, 0.0, EPS));
        assert!(approx_eq(y, yaw, EPS));
    }

    #[test]
    fn test_euler_roundtrip_roll_pitch_yaw() {
        let (roll, pitch, yaw) = (0.3, -0.2, 1.1);
        let q = Quaternion::from_euler(roll, pitch, yaw);
        let (r, p, y) = q.to_euler();
        assert!(approx_eq(r, roll, 1e-9));
        assert!(approx_eq(p, pitch, 1e-9));
        assert!(approx_eq(y, yaw, 1e-9));
    }

    // --- Rotation matrix conversions ---

    #[test]
    fn test_rotation_matrix_identity() {
        let q = Quaternion::identity();
        let m = q.to_rotation_matrix();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(approx_eq(m[i][j], expected, EPS));
            }
        }
    }

    #[test]
    fn test_rotation_matrix_roundtrip() {
        let q_orig = Quaternion::from_euler(0.5, -0.3, 1.2).normalized();
        let m = q_orig.to_rotation_matrix();
        let q_back = Quaternion::from_rotation_matrix(&m).normalized();
        // q and -q represent the same rotation
        let dot = q_orig.dot(&q_back).abs();
        assert!(approx_eq(dot, 1.0, 1e-9));
    }

    // --- Axis-angle ---

    #[test]
    fn test_axis_angle_roundtrip() {
        let axis = [0.0, 1.0, 0.0];
        let angle = 1.0;
        let q = Quaternion::from_axis_angle(axis, angle);
        let (a, ang) = q.to_axis_angle();
        assert!(approx_eq(ang, angle, 1e-9));
        assert!(approx_eq(a[1], 1.0, 1e-9));
    }

    // --- Rotate vector ---

    #[test]
    fn test_rotate_vector_90_about_z() {
        let q = Quaternion::from_axis_angle([0.0, 0.0, 1.0], FRAC_PI_2);
        let v = q.rotate_vector([1.0, 0.0, 0.0]);
        assert!(approx_eq(v[0], 0.0, 1e-9));
        assert!(approx_eq(v[1], 1.0, 1e-9));
        assert!(approx_eq(v[2], 0.0, 1e-9));
    }

    // --- Slerp ---

    #[test]
    fn test_slerp_endpoints() {
        let a = Quaternion::identity();
        let b = Quaternion::from_euler(0.0, 0.0, PI / 3.0);
        let s0 = a.slerp(&b, 0.0);
        let s1 = a.slerp(&b, 1.0);
        assert!(approx_eq(s0.dot(&a).abs(), 1.0, 1e-9));
        assert!(approx_eq(s1.dot(&b).abs(), 1.0, 1e-9));
    }

    #[test]
    fn test_slerp_midpoint() {
        let a = Quaternion::identity();
        let b = Quaternion::from_axis_angle([0.0, 0.0, 1.0], PI / 2.0);
        let mid = a.slerp(&b, 0.5);
        // Midpoint should be a 45-degree rotation about Z
        let expected = Quaternion::from_axis_angle([0.0, 0.0, 1.0], PI / 4.0);
        assert!(approx_eq(mid.dot(&expected).abs(), 1.0, 1e-9));
    }

    // --- Angle between quaternions ---

    #[test]
    fn test_angle_to_same() {
        let q = Quaternion::from_euler(0.5, 0.3, 1.0);
        assert!(approx_eq(q.angle_to(&q), 0.0, 1e-6));
    }

    #[test]
    fn test_angle_to_known() {
        let a = Quaternion::identity();
        let b = Quaternion::from_axis_angle([0.0, 0.0, 1.0], 0.5);
        let angle = a.angle_to(&b);
        assert!(approx_eq(angle, 0.5, 1e-9));
    }

    // --- Gyroscope kinematics ---

    #[test]
    fn test_gyro_integration_constant_rate() {
        // Constant yaw rate of 1 rad/s for 1 second in 1000 steps
        let mut tracker = AttitudeTracker::new();
        let dt = 0.001;
        for _ in 0..1000 {
            tracker.gyro_update([0.0, 0.0, 1.0], dt);
        }
        let (_, _, yaw) = tracker.euler();
        assert!(approx_eq(yaw, 1.0, 0.01)); // ~1% accuracy from first-order integration
    }

    // --- Angular velocity estimation ---

    #[test]
    fn test_angular_velocity_estimation() {
        let q1 = Quaternion::identity();
        let omega = [0.0, 0.0, 2.0]; // 2 rad/s about Z
        let dt = 0.01;
        let q_dot = q1.kinematics(omega);
        let q2 = q1.add(&q_dot.scale(dt)).normalized();
        let est = q1.angular_velocity_to(&q2, dt);
        assert!(approx_eq(est[2], 2.0, 0.1)); // rough due to first-order
    }

    // --- TRIAD ---

    #[test]
    fn test_triad_identity() {
        // Same vectors in both frames -> identity rotation
        let v1 = [1.0, 0.0, 0.0];
        let v2 = [0.0, 1.0, 0.0];
        let r = triad(v1, v2, v1, v2);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    approx_eq(r[i][j], expected, 1e-9),
                    "TRIAD identity failed at [{i}][{j}]: got {}, expected {expected}",
                    r[i][j]
                );
            }
        }
    }

    #[test]
    fn test_triad_known_rotation() {
        // 90 degrees about Z: X->Y, Y->-X
        let ref1 = [1.0, 0.0, 0.0];
        let ref2 = [0.0, 1.0, 0.0];
        let body1 = [0.0, 1.0, 0.0];
        let body2 = [-1.0, 0.0, 0.0];
        let q = triad_quaternion(ref1, ref2, body1, body2);
        let (_, _, yaw) = q.to_euler();
        assert!(approx_eq(yaw, FRAC_PI_2, 1e-9));
    }

    // --- Complementary filter ---

    #[test]
    fn test_complementary_filter_static() {
        // With no rotation and gravity pointing down, the filter should
        // stay near identity.
        let mut tracker = AttitudeTracker::new();
        for _ in 0..100 {
            tracker.update_with_accel([0.0, 0.0, 0.0], [0.0, 0.0, -9.81], 0.01);
        }
        let (r, p, _) = tracker.euler();
        assert!(approx_eq(r, 0.0, 0.05));
        assert!(approx_eq(p, 0.0, 0.05));
    }

    // --- Complex helpers ---

    #[test]
    fn test_complex_mul() {
        let a = (3.0, 4.0);
        let b = (1.0, -2.0);
        let c = complex_mul(a, b);
        // (3+4i)(1-2i) = 3-6i+4i-8i^2 = 3-2i+8 = 11-2i
        assert!(approx_eq(c.0, 11.0, EPS));
        assert!(approx_eq(c.1, -2.0, EPS));
    }

    #[test]
    fn test_complex_abs() {
        assert!(approx_eq(complex_abs((3.0, 4.0)), 5.0, EPS));
    }

    #[test]
    fn test_complex_conj() {
        let c = complex_conj((3.0, 4.0));
        assert_eq!(c, (3.0, -4.0));
    }

    // --- Scale and add ---

    #[test]
    fn test_quaternion_scale_and_add() {
        let a = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let b = Quaternion::new(0.0, 1.0, 0.0, 0.0);
        let sum = a.scale(2.0).add(&b.scale(3.0));
        assert!(approx_eq(sum.w, 2.0, EPS));
        assert!(approx_eq(sum.x, 3.0, EPS));
    }

    // --- Dot product ---

    #[test]
    fn test_dot_product_orthogonal() {
        // Two quaternions 180 degrees apart have dot = 0
        let a = Quaternion::from_axis_angle([0.0, 0.0, 1.0], 0.0);
        let b = Quaternion::from_axis_angle([0.0, 0.0, 1.0], PI);
        assert!(approx_eq(a.dot(&b), 0.0, 1e-9));
    }

    // --- Tracker API ---

    #[test]
    fn test_tracker_reset() {
        let mut tracker = AttitudeTracker::new();
        tracker.gyro_update([1.0, 0.0, 0.0], 0.1);
        tracker.reset();
        let q = tracker.quaternion();
        assert!(approx_eq(q.w, 1.0, EPS));
        assert!(approx_eq(q.x, 0.0, EPS));
    }

    #[test]
    fn test_tracker_set_quaternion() {
        let mut tracker = AttitudeTracker::new();
        let q = Quaternion::from_euler(0.5, 0.0, 0.0);
        tracker.set_quaternion(q);
        let (r, _, _) = tracker.euler();
        assert!(approx_eq(r, 0.5, 1e-9));
    }

    #[test]
    fn test_tracker_default() {
        let t = AttitudeTracker::default();
        assert!(approx_eq(t.quaternion().w, 1.0, EPS));
    }
}

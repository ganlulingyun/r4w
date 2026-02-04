//! Keplerian orbit propagation for GNSS satellites
//!
//! Implements two-body orbital mechanics with Keplerian elements.
//! Provides nominal orbit constructors for GPS, Galileo, and GLONASS constellations.

use crate::coordinates::{EcefPosition, EcefVelocity};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Gravitational parameter for Earth (m^3/s^2)
const GM_EARTH: f64 = 3.986_004_418e14;
/// Earth rotation rate (rad/s)
const OMEGA_E: f64 = 7.292_115_0e-5;

/// Keplerian orbital elements
// trace:FR-040 | ai:claude
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeplerianOrbit {
    /// Semi-major axis in meters
    pub a: f64,
    /// Eccentricity (0 = circular)
    pub e: f64,
    /// Inclination in radians
    pub i: f64,
    /// Right ascension of ascending node (RAAN) in radians
    pub omega_0: f64,
    /// Argument of perigee in radians
    pub omega: f64,
    /// Mean anomaly at epoch in radians
    pub m0: f64,
    /// Epoch time in seconds (GPS time or scenario time)
    pub t_epoch: f64,
    /// Rate of change of RAAN in rad/s (optional, for J2 perturbation)
    pub omega_dot: f64,
}

impl KeplerianOrbit {
    /// Mean motion (rad/s)
    pub fn mean_motion(&self) -> f64 {
        (GM_EARTH / (self.a * self.a * self.a)).sqrt()
    }

    /// Orbital period in seconds
    pub fn period(&self) -> f64 {
        2.0 * PI / self.mean_motion()
    }

    /// Compute satellite ECEF position and velocity at time t
    pub fn position_velocity_at(&self, t: f64) -> (EcefPosition, EcefVelocity) {
        let dt = t - self.t_epoch;
        let n = self.mean_motion();

        // Mean anomaly
        let m = (self.m0 + n * dt) % (2.0 * PI);

        // Eccentric anomaly via Newton-Raphson
        let ecc_anom = solve_kepler(m, self.e);

        // True anomaly
        let sin_e = ecc_anom.sin();
        let cos_e = ecc_anom.cos();
        let sqrt_1_e2 = (1.0 - self.e * self.e).sqrt();
        let true_anom = (sqrt_1_e2 * sin_e).atan2(cos_e - self.e);

        // Radius
        let r = self.a * (1.0 - self.e * cos_e);

        // Position in orbital plane
        let x_orb = r * true_anom.cos();
        let y_orb = r * true_anom.sin();

        // Velocity in orbital plane
        let h = (GM_EARTH * self.a * (1.0 - self.e * self.e)).sqrt(); // specific angular momentum
        let vx_orb = -GM_EARTH / h * true_anom.sin();
        let vy_orb = GM_EARTH / h * (self.e + true_anom.cos());

        // RAAN at time t (with optional precession)
        let omega_t = self.omega_0 + self.omega_dot * dt;

        // Rotation from orbital plane to ECI
        let cos_omega = self.omega.cos();
        let sin_omega = self.omega.sin();
        let cos_raan = omega_t.cos();
        let sin_raan = omega_t.sin();
        let cos_i = self.i.cos();
        let sin_i = self.i.sin();

        // ECI position
        let x_eci = (cos_raan * cos_omega - sin_raan * sin_omega * cos_i) * x_orb
            + (-cos_raan * sin_omega - sin_raan * cos_omega * cos_i) * y_orb;
        let y_eci = (sin_raan * cos_omega + cos_raan * sin_omega * cos_i) * x_orb
            + (-sin_raan * sin_omega + cos_raan * cos_omega * cos_i) * y_orb;
        let z_eci = (sin_omega * sin_i) * x_orb + (cos_omega * sin_i) * y_orb;

        // ECI velocity
        let vx_eci = (cos_raan * cos_omega - sin_raan * sin_omega * cos_i) * vx_orb
            + (-cos_raan * sin_omega - sin_raan * cos_omega * cos_i) * vy_orb;
        let vy_eci = (sin_raan * cos_omega + cos_raan * sin_omega * cos_i) * vx_orb
            + (-sin_raan * sin_omega + cos_raan * cos_omega * cos_i) * vy_orb;
        let vz_eci = (sin_omega * sin_i) * vx_orb + (cos_omega * sin_i) * vy_orb;

        // ECI to ECEF rotation (Earth rotation)
        let theta = OMEGA_E * t;
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        let x_ecef = cos_t * x_eci + sin_t * y_eci;
        let y_ecef = -sin_t * x_eci + cos_t * y_eci;
        let z_ecef = z_eci;

        let vx_ecef = cos_t * vx_eci + sin_t * vy_eci + OMEGA_E * y_ecef;
        let vy_ecef = -sin_t * vx_eci + cos_t * vy_eci - OMEGA_E * x_ecef;
        let vz_ecef = vz_eci;

        (
            EcefPosition::new(x_ecef, y_ecef, z_ecef),
            EcefVelocity::new(vx_ecef, vy_ecef, vz_ecef),
        )
    }

    /// Create a nominal GPS orbit for a given orbital plane and slot
    ///
    /// GPS constellation: 6 planes (A-F), 4+ slots per plane
    /// Semi-major axis: ~26,559.7 km, inclination: 55 deg
    pub fn gps_nominal(plane: u8, slot: u8) -> Self {
        assert!(plane < 6, "GPS has 6 orbital planes (0-5)");
        assert!(slot < 6, "Use slot 0-5");

        let a = 26_559_700.0; // meters
        let raan_spacing = 60.0_f64.to_radians(); // 60 deg between planes
        let slot_spacing = 60.0_f64.to_radians(); // ~60 deg between slots

        Self {
            a,
            e: 0.0,
            i: 55.0_f64.to_radians(),
            omega_0: plane as f64 * raan_spacing,
            omega: 0.0,
            m0: slot as f64 * slot_spacing,
            t_epoch: 0.0,
            omega_dot: 0.0,
        }
    }

    /// Create a nominal Galileo orbit for a given orbital plane and slot
    ///
    /// Galileo constellation: 3 planes, 8-10 satellites per plane
    /// Semi-major axis: ~29,600 km, inclination: 56 deg
    pub fn galileo_nominal(plane: u8, slot: u8) -> Self {
        assert!(plane < 3, "Galileo has 3 orbital planes (0-2)");
        assert!(slot < 10, "Use slot 0-9");

        let a = 29_600_318.0;
        let raan_spacing = 120.0_f64.to_radians();
        let slot_spacing = 36.0_f64.to_radians();

        Self {
            a,
            e: 0.0,
            i: 56.0_f64.to_radians(),
            omega_0: plane as f64 * raan_spacing,
            omega: 0.0,
            m0: slot as f64 * slot_spacing,
            t_epoch: 0.0,
            omega_dot: 0.0,
        }
    }

    /// Create a nominal GLONASS orbit
    ///
    /// GLONASS: 3 planes, 8 satellites per plane
    /// Semi-major axis: ~25,508 km, inclination: 64.8 deg
    pub fn glonass_nominal(plane: u8, slot: u8) -> Self {
        assert!(plane < 3, "GLONASS has 3 orbital planes (0-2)");
        assert!(slot < 8, "Use slot 0-7");

        let a = 25_508_000.0;
        let raan_spacing = 120.0_f64.to_radians();
        let slot_spacing = 45.0_f64.to_radians();

        Self {
            a,
            e: 0.0,
            i: 64.8_f64.to_radians(),
            omega_0: plane as f64 * raan_spacing,
            omega: 0.0,
            m0: slot as f64 * slot_spacing,
            t_epoch: 0.0,
            omega_dot: 0.0,
        }
    }
}

/// Solve Kepler's equation M = E - e*sin(E) using Newton-Raphson
fn solve_kepler(m: f64, e: f64) -> f64 {
    let mut ecc = m;
    for _ in 0..20 {
        let de = (ecc - e * ecc.sin() - m) / (1.0 - e * ecc.cos());
        ecc -= de;
        if de.abs() < 1e-14 {
            break;
        }
    }
    ecc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_orbital_period() {
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let period = orbit.period();
        // GPS orbital period is approximately 11h 58m = 43080 seconds (half sidereal day)
        let expected = 43080.0;
        assert!(
            (period - expected).abs() < 120.0, // within 2 minutes
            "GPS period: {} s, expected ~{} s",
            period,
            expected
        );
    }

    #[test]
    fn test_galileo_orbital_period() {
        let orbit = KeplerianOrbit::galileo_nominal(0, 0);
        let period = orbit.period();
        // Galileo orbital period ~14.08 hours = ~50688 s
        assert!(
            period > 49000.0 && period < 52000.0,
            "Galileo period: {} s",
            period
        );
    }

    #[test]
    fn test_gps_altitude() {
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let (pos, _) = orbit.position_velocity_at(0.0);
        let r = (pos.x * pos.x + pos.y * pos.y + pos.z * pos.z).sqrt();
        // GPS orbit radius ~26,560 km
        assert!(
            (r - 26_559_700.0).abs() < 100_000.0,
            "GPS radius: {} m",
            r
        );
    }

    #[test]
    fn test_velocity_magnitude() {
        let orbit = KeplerianOrbit::gps_nominal(0, 0);
        let (_, vel) = orbit.position_velocity_at(0.0);
        let speed = vel.speed();
        // GPS orbital velocity ~3.9 km/s
        assert!(speed > 3000.0 && speed < 5000.0, "GPS speed: {} m/s", speed);
    }

    #[test]
    fn test_kepler_solver_circular() {
        let e = solve_kepler(1.0, 0.0);
        assert!((e - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_kepler_solver_eccentric() {
        let m = 1.0;
        let ecc = 0.1;
        let e = solve_kepler(m, ecc);
        // Verify: M = E - e*sin(E)
        let m_check = e - ecc * e.sin();
        assert!((m_check - m).abs() < 1e-12);
    }
}

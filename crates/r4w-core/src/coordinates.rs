//! Coordinate types and geodetic conversions
//!
//! Provides ECEF (Earth-Centered, Earth-Fixed) and LLA (Latitude, Longitude, Altitude)
//! coordinate types with bidirectional conversions, plus look angle computations.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// WGS-84 semi-major axis in meters
const WGS84_A: f64 = 6_378_137.0;
/// WGS-84 flattening
const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS-84 first eccentricity squared
const WGS84_E2: f64 = 2.0 * WGS84_F - WGS84_F * WGS84_F;
/// Speed of light in m/s
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Earth-Centered, Earth-Fixed position in meters
// trace:FR-039 | ai:claude
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EcefPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl EcefPosition {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Distance to another ECEF position in meters
    pub fn distance_to(&self, other: &EcefPosition) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Unit vector from self to other
    pub fn direction_to(&self, other: &EcefPosition) -> [f64; 3] {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        let r = (dx * dx + dy * dy + dz * dz).sqrt();
        if r < 1e-10 {
            return [0.0, 0.0, 0.0];
        }
        [dx / r, dy / r, dz / r]
    }
}

impl std::ops::Sub for EcefPosition {
    type Output = EcefVelocity;
    fn sub(self, rhs: Self) -> EcefVelocity {
        EcefVelocity {
            vx: self.x - rhs.x,
            vy: self.y - rhs.y,
            vz: self.z - rhs.z,
        }
    }
}

/// ECEF velocity in meters per second
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EcefVelocity {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}

impl EcefVelocity {
    pub fn new(vx: f64, vy: f64, vz: f64) -> Self {
        Self { vx, vy, vz }
    }

    pub fn zero() -> Self {
        Self { vx: 0.0, vy: 0.0, vz: 0.0 }
    }

    /// Speed magnitude in m/s
    pub fn speed(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt()
    }

    /// Dot product with a unit direction vector
    pub fn dot_direction(&self, dir: &[f64; 3]) -> f64 {
        self.vx * dir[0] + self.vy * dir[1] + self.vz * dir[2]
    }
}

/// Latitude, Longitude, Altitude (WGS-84 geodetic)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LlaPosition {
    /// Latitude in degrees (-90 to +90)
    pub lat_deg: f64,
    /// Longitude in degrees (-180 to +180)
    pub lon_deg: f64,
    /// Altitude above WGS-84 ellipsoid in meters
    pub alt_m: f64,
}

impl LlaPosition {
    pub fn new(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Self {
        Self { lat_deg, lon_deg, alt_m }
    }

    pub fn lat_rad(&self) -> f64 {
        self.lat_deg.to_radians()
    }

    pub fn lon_rad(&self) -> f64 {
        self.lon_deg.to_radians()
    }
}

/// Look angle from observer to target
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LookAngle {
    /// Elevation angle in degrees (0=horizon, 90=zenith)
    pub elevation_deg: f64,
    /// Azimuth angle in degrees (0=North, 90=East)
    pub azimuth_deg: f64,
    /// Slant range in meters
    pub range_m: f64,
}

/// Convert LLA to ECEF using WGS-84 ellipsoid
pub fn lla_to_ecef(lla: &LlaPosition) -> EcefPosition {
    let lat = lla.lat_rad();
    let lon = lla.lon_rad();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();

    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();

    EcefPosition {
        x: (n + lla.alt_m) * cos_lat * cos_lon,
        y: (n + lla.alt_m) * cos_lat * sin_lon,
        z: (n * (1.0 - WGS84_E2) + lla.alt_m) * sin_lat,
    }
}

/// Convert ECEF to LLA using iterative method (Bowring)
pub fn ecef_to_lla(ecef: &EcefPosition) -> LlaPosition {
    let x = ecef.x;
    let y = ecef.y;
    let z = ecef.z;
    let p = (x * x + y * y).sqrt();
    let lon = y.atan2(x);

    // Iterative Bowring method
    let b = WGS84_A * (1.0 - WGS84_F);
    let ep2 = (WGS84_A * WGS84_A - b * b) / (b * b);
    let mut beta = (z / (p * (1.0 - WGS84_F))).atan();

    for _ in 0..10 {
        let sin_beta = beta.sin();
        let cos_beta = beta.cos();
        let lat = (z + ep2 * b * sin_beta * sin_beta * sin_beta)
            .atan2(p - WGS84_E2 * WGS84_A * cos_beta * cos_beta * cos_beta);
        let new_beta = lat.atan2((1.0 - WGS84_F) * lat.tan().recip()).atan();
        // Use the parametric latitude update
        let _ = new_beta;
        beta = ((1.0 - WGS84_F) * lat.tan()).atan();
    }

    let sin_beta = beta.sin();
    let cos_beta = beta.cos();
    let lat = (z + ep2 * b * sin_beta * sin_beta * sin_beta)
        .atan2(p - WGS84_E2 * WGS84_A * cos_beta * cos_beta * cos_beta);

    let sin_lat = lat.sin();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let alt = if lat.cos().abs() > 1e-10 {
        p / lat.cos() - n
    } else {
        z.abs() - b
    };

    LlaPosition {
        lat_deg: lat.to_degrees(),
        lon_deg: lon.to_degrees(),
        alt_m: alt,
    }
}

/// Compute look angle (elevation, azimuth, range) from observer to target
pub fn look_angle(observer: &EcefPosition, observer_lla: &LlaPosition, target: &EcefPosition) -> LookAngle {
    let dx = target.x - observer.x;
    let dy = target.y - observer.y;
    let dz = target.z - observer.z;
    let range_m = (dx * dx + dy * dy + dz * dz).sqrt();

    let lat = observer_lla.lat_rad();
    let lon = observer_lla.lon_rad();

    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();

    // Rotate to ENU (East-North-Up) frame
    let east = -sin_lon * dx + cos_lon * dy;
    let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    let up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

    let elevation_deg = up.atan2((east * east + north * north).sqrt()).to_degrees();
    let mut azimuth_deg = east.atan2(north).to_degrees();
    if azimuth_deg < 0.0 {
        azimuth_deg += 360.0;
    }

    LookAngle {
        elevation_deg,
        azimuth_deg,
        range_m,
    }
}

/// Compute range rate (radial velocity) between observer and target
/// Returns positive when moving apart, negative when approaching
pub fn range_rate(
    obs_pos: &EcefPosition,
    obs_vel: &EcefVelocity,
    tgt_pos: &EcefPosition,
    tgt_vel: &EcefVelocity,
) -> f64 {
    let dir = obs_pos.direction_to(tgt_pos);
    let rel_vel = EcefVelocity {
        vx: tgt_vel.vx - obs_vel.vx,
        vy: tgt_vel.vy - obs_vel.vy,
        vz: tgt_vel.vz - obs_vel.vz,
    };
    rel_vel.dot_direction(&dir)
}

/// Free-space path loss in dB given distance and frequency
pub fn fspl_db(distance_m: f64, frequency_hz: f64) -> f64 {
    if distance_m <= 0.0 || frequency_hz <= 0.0 {
        return 0.0;
    }
    20.0 * (4.0 * PI * distance_m * frequency_hz / SPEED_OF_LIGHT).log10()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lla_ecef_roundtrip() {
        let lla = LlaPosition::new(40.0, -75.0, 100.0);
        let ecef = lla_to_ecef(&lla);
        let lla2 = ecef_to_lla(&ecef);
        assert!((lla.lat_deg - lla2.lat_deg).abs() < 1e-6, "lat: {} vs {}", lla.lat_deg, lla2.lat_deg);
        assert!((lla.lon_deg - lla2.lon_deg).abs() < 1e-6, "lon: {} vs {}", lla.lon_deg, lla2.lon_deg);
        assert!((lla.alt_m - lla2.alt_m).abs() < 1.0, "alt: {} vs {}", lla.alt_m, lla2.alt_m);
    }

    #[test]
    fn test_lla_ecef_equator() {
        let lla = LlaPosition::new(0.0, 0.0, 0.0);
        let ecef = lla_to_ecef(&lla);
        assert!((ecef.x - WGS84_A).abs() < 1.0);
        assert!(ecef.y.abs() < 1e-6);
        assert!(ecef.z.abs() < 1e-6);
    }

    #[test]
    fn test_distance() {
        let a = EcefPosition::new(0.0, 0.0, 0.0);
        let b = EcefPosition::new(3.0, 4.0, 0.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_look_angle_zenith() {
        let obs_lla = LlaPosition::new(0.0, 0.0, 0.0);
        let obs_ecef = lla_to_ecef(&obs_lla);
        // Point directly above on equator at prime meridian
        let target = EcefPosition::new(obs_ecef.x + 1000.0, obs_ecef.y, obs_ecef.z);
        let la = look_angle(&obs_ecef, &obs_lla, &target);
        assert!((la.elevation_deg - 90.0).abs() < 1.0);
        assert!((la.range_m - 1000.0).abs() < 1.0);
    }

    #[test]
    fn test_range_rate() {
        let obs = EcefPosition::new(0.0, 0.0, 0.0);
        let obs_vel = EcefVelocity::zero();
        let tgt = EcefPosition::new(1000.0, 0.0, 0.0);
        let tgt_vel = EcefVelocity::new(100.0, 0.0, 0.0); // moving away
        let rr = range_rate(&obs, &obs_vel, &tgt, &tgt_vel);
        assert!((rr - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_fspl() {
        // FSPL = 20*log10(4*pi*d*f/c) at 1km, 1575.42 MHz
        let loss = fspl_db(1000.0, 1_575_420_000.0);
        assert!(loss > 90.0 && loss < 100.0, "FSPL = {} dB", loss);
        // At GPS orbit distance (~20,200 km), should be ~182 dB
        let loss_orbit = fspl_db(20_200_000.0, 1_575_420_000.0);
        assert!(loss_orbit > 178.0 && loss_orbit < 186.0, "FSPL orbit = {} dB", loss_orbit);
    }
}

//! Receiver trajectory models
//!
//! Defines motion profiles for simulated receivers: static, linear, waypoints, circular.

use r4w_core::coordinates::{EcefPosition, EcefVelocity, lla_to_ecef, LlaPosition};
use serde::{Deserialize, Serialize};

/// State of a moving entity at a given time
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryState {
    pub position: EcefPosition,
    pub velocity: EcefVelocity,
    pub time_s: f64,
}

/// Receiver trajectory definition
// trace:FR-039 | ai:claude
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Trajectory {
    /// Fixed position
    Static {
        position: LlaPosition,
    },
    /// Constant velocity from start position
    Linear {
        start: LlaPosition,
        /// Velocity in ENU frame (east, north, up) m/s
        velocity_enu: [f64; 3],
    },
    /// Sequence of waypoints with timestamps
    Waypoints {
        /// (time_s, position)
        points: Vec<(f64, LlaPosition)>,
    },
    /// Circular motion around a center point
    Circular {
        center: LlaPosition,
        /// Radius in meters
        radius_m: f64,
        /// Angular velocity in rad/s (positive = counterclockwise)
        omega_rad_s: f64,
        /// Initial bearing in degrees from North
        initial_bearing_deg: f64,
    },
}

impl Trajectory {
    /// Get position and velocity at time t
    pub fn state_at(&self, t: f64) -> TrajectoryState {
        match self {
            Trajectory::Static { position } => {
                TrajectoryState {
                    position: lla_to_ecef(position),
                    velocity: EcefVelocity::zero(),
                    time_s: t,
                }
            }
            Trajectory::Linear { start, velocity_enu } => {
                let ecef_start = lla_to_ecef(start);
                let lat = start.lat_rad();
                let lon = start.lon_rad();
                let (sin_lat, cos_lat) = (lat.sin(), lat.cos());
                let (sin_lon, cos_lon) = (lon.sin(), lon.cos());

                // ENU to ECEF rotation
                let ve = velocity_enu[0];
                let vn = velocity_enu[1];
                let vu = velocity_enu[2];

                let vx = -sin_lon * ve - sin_lat * cos_lon * vn + cos_lat * cos_lon * vu;
                let vy = cos_lon * ve - sin_lat * sin_lon * vn + cos_lat * sin_lon * vu;
                let vz = cos_lat * vn + sin_lat * vu;

                TrajectoryState {
                    position: EcefPosition::new(
                        ecef_start.x + vx * t,
                        ecef_start.y + vy * t,
                        ecef_start.z + vz * t,
                    ),
                    velocity: EcefVelocity::new(vx, vy, vz),
                    time_s: t,
                }
            }
            Trajectory::Waypoints { points } => {
                if points.is_empty() {
                    return TrajectoryState {
                        position: EcefPosition::new(0.0, 0.0, 0.0),
                        velocity: EcefVelocity::zero(),
                        time_s: t,
                    };
                }
                if points.len() == 1 || t <= points[0].0 {
                    return TrajectoryState {
                        position: lla_to_ecef(&points[0].1),
                        velocity: EcefVelocity::zero(),
                        time_s: t,
                    };
                }
                if t >= points.last().unwrap().0 {
                    return TrajectoryState {
                        position: lla_to_ecef(&points.last().unwrap().1),
                        velocity: EcefVelocity::zero(),
                        time_s: t,
                    };
                }
                // Find segment
                let idx = points.iter().position(|(pt, _)| *pt > t).unwrap_or(points.len()) - 1;
                let (t0, p0) = &points[idx];
                let (t1, p1) = &points[idx + 1];
                let dt = t1 - t0;
                let frac = (t - t0) / dt;

                let e0 = lla_to_ecef(p0);
                let e1 = lla_to_ecef(p1);

                let pos = EcefPosition::new(
                    e0.x + (e1.x - e0.x) * frac,
                    e0.y + (e1.y - e0.y) * frac,
                    e0.z + (e1.z - e0.z) * frac,
                );
                let vel = EcefVelocity::new(
                    (e1.x - e0.x) / dt,
                    (e1.y - e0.y) / dt,
                    (e1.z - e0.z) / dt,
                );
                TrajectoryState { position: pos, velocity: vel, time_s: t }
            }
            Trajectory::Circular { center, radius_m, omega_rad_s, initial_bearing_deg } => {
                let ecef_center = lla_to_ecef(center);
                let lat = center.lat_rad();
                let lon = center.lon_rad();
                let (sin_lat, cos_lat) = (lat.sin(), lat.cos());
                let (sin_lon, cos_lon) = (lon.sin(), lon.cos());

                let bearing = initial_bearing_deg.to_radians() + omega_rad_s * t;
                let east = radius_m * bearing.sin();
                let north = radius_m * bearing.cos();

                // ENU offset to ECEF
                let dx = -sin_lon * east - sin_lat * cos_lon * north;
                let dy = cos_lon * east - sin_lat * sin_lon * north;
                let dz = cos_lat * north;

                let pos = EcefPosition::new(
                    ecef_center.x + dx,
                    ecef_center.y + dy,
                    ecef_center.z + dz,
                );

                // Velocity: derivative of position w.r.t. time
                let d_east = radius_m * omega_rad_s * bearing.cos();
                let d_north = -radius_m * omega_rad_s * bearing.sin();

                let vx = -sin_lon * d_east - sin_lat * cos_lon * d_north;
                let vy = cos_lon * d_east - sin_lat * sin_lon * d_north;
                let vz = cos_lat * d_north;

                TrajectoryState {
                    position: pos,
                    velocity: EcefVelocity::new(vx, vy, vz),
                    time_s: t,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_static_trajectory() {
        let traj = Trajectory::Static {
            position: LlaPosition::new(40.0, -75.0, 100.0),
        };
        let s0 = traj.state_at(0.0);
        let s1 = traj.state_at(10.0);
        assert!((s0.position.x - s1.position.x).abs() < 1e-6);
        assert!(s0.velocity.speed() < 1e-10);
    }

    #[test]
    fn test_linear_trajectory() {
        let traj = Trajectory::Linear {
            start: LlaPosition::new(0.0, 0.0, 0.0),
            velocity_enu: [10.0, 0.0, 0.0], // 10 m/s east
        };
        let s0 = traj.state_at(0.0);
        let s1 = traj.state_at(1.0);
        let dist = s0.position.distance_to(&s1.position);
        assert!((dist - 10.0).abs() < 0.1, "distance = {}", dist);
    }

    #[test]
    fn test_circular_trajectory() {
        let traj = Trajectory::Circular {
            center: LlaPosition::new(0.0, 0.0, 0.0),
            radius_m: 100.0,
            omega_rad_s: 0.1,
            initial_bearing_deg: 0.0,
        };
        let s = traj.state_at(0.0);
        let speed = s.velocity.speed();
        let expected = 100.0 * 0.1; // r * omega
        assert!((speed - expected).abs() < 0.5, "speed = {} vs {}", speed, expected);
    }
}

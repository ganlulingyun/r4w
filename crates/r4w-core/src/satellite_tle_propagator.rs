//! Simplified SGP4 orbital propagation from Two-Line Element (TLE) sets.
//!
//! This module provides satellite tracking and Doppler prediction using
//! standard NORAD TLE data. It implements:
//!
//! - TLE parsing (standard 2-line, 69-character format)
//! - Simplified SGP4 propagation with J2 secular perturbations
//! - Kepler equation solver (Newton-Raphson)
//! - ECI position/velocity computation
//! - Observer-relative range, elevation, azimuth
//! - Doppler shift prediction for arbitrary carrier frequencies
//! - Pass prediction (rise/set above minimum elevation)
//!
//! # Example
//!
//! ```
//! use r4w_core::satellite_tle_propagator::{TleParser, Sgp4Propagator, ObserverPosition};
//!
//! // ISS TLE (example)
//! let line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9997";
//! let line2 = "2 25544  51.6400 200.0000 0007417  50.0000 310.1200 15.49560000100002";
//!
//! let elements = TleParser::parse(line1, line2).unwrap();
//! assert_eq!(elements.norad_id, 25544);
//!
//! let propagator = Sgp4Propagator::new(&elements);
//! let state = propagator.propagate_minutes(0.0);
//! // Position in ECI frame (km)
//! let r = (state.position[0].powi(2) + state.position[1].powi(2)
//!     + state.position[2].powi(2)).sqrt();
//! // ISS orbits ~420 km altitude, so radius should be ~6790 km
//! assert!(r > 6500.0 && r < 7200.0);
//!
//! // Doppler shift at 437 MHz (UHF amateur satellite)
//! let observer = ObserverPosition { lat_deg: 40.0, lon_deg: -74.0, alt_m: 0.0 };
//! let doppler = propagator.doppler_hz(0.0, &observer, 437.0e6);
//! // Doppler can be positive or negative depending on geometry
//! assert!(doppler.abs() < 15_000.0); // Max ~12 kHz for LEO at UHF
//! ```

use std::f64::consts::PI;

// ─── Constants ────────────────────────────────────────────────────────────────

/// Earth gravitational parameter (km³/s²)
const MU_EARTH: f64 = 398600.4418;

/// Earth equatorial radius (km) — WGS-84
const RE_EARTH: f64 = 6378.137;

/// Earth flattening factor — WGS-84
const FLATTENING: f64 = 1.0 / 298.257223563;

/// Earth J2 zonal harmonic
const J2: f64 = 1.08263e-3;

/// Earth rotation rate (rad/s)
const OMEGA_EARTH: f64 = 7.2921159e-5;

/// Minutes per day
const MIN_PER_DAY: f64 = 1440.0;

/// Speed of light (m/s)
const C_LIGHT: f64 = 299_792_458.0;

/// Two-pi
const TWO_PI: f64 = 2.0 * PI;

/// Degrees to radians
const DEG2RAD: f64 = PI / 180.0;

/// Radians to degrees
const RAD2DEG: f64 = 180.0 / PI;

// ─── Orbital Elements ─────────────────────────────────────────────────────────

/// Orbital elements extracted from a TLE set.
#[derive(Debug, Clone)]
pub struct OrbitalElements {
    /// NORAD catalog number
    pub norad_id: u32,
    /// International designator (e.g., "98067A")
    pub intl_designator: String,
    /// Epoch year (full, e.g., 2024)
    pub epoch_year: u32,
    /// Epoch day of year (fractional)
    pub epoch_day: f64,
    /// First derivative of mean motion (rev/day²) ÷ 2
    pub ndot_over_2: f64,
    /// Drag term (B* in 1/earth-radii)
    pub bstar: f64,
    /// Inclination (radians)
    pub inclination: f64,
    /// Right ascension of ascending node (radians)
    pub raan: f64,
    /// Eccentricity (dimensionless)
    pub eccentricity: f64,
    /// Argument of perigee (radians)
    pub arg_perigee: f64,
    /// Mean anomaly at epoch (radians)
    pub mean_anomaly: f64,
    /// Mean motion (radians/minute)
    pub mean_motion: f64,
    /// Revolution number at epoch
    pub rev_number: u32,
}

impl OrbitalElements {
    /// Orbital period in minutes.
    pub fn period_minutes(&self) -> f64 {
        TWO_PI / self.mean_motion
    }

    /// Semi-major axis in km, derived from mean motion.
    pub fn semi_major_axis_km(&self) -> f64 {
        // n = sqrt(mu / a^3), n in rad/s => a = (mu / n^2)^(1/3)
        let n_rad_s = self.mean_motion / 60.0; // rad/min -> rad/s
        (MU_EARTH / (n_rad_s * n_rad_s)).powf(1.0 / 3.0)
    }

    /// Apogee altitude in km above the equatorial radius.
    pub fn apogee_altitude_km(&self) -> f64 {
        let a = self.semi_major_axis_km();
        a * (1.0 + self.eccentricity) - RE_EARTH
    }

    /// Perigee altitude in km above the equatorial radius.
    pub fn perigee_altitude_km(&self) -> f64 {
        let a = self.semi_major_axis_km();
        a * (1.0 - self.eccentricity) - RE_EARTH
    }
}

// ─── TLE Parser ───────────────────────────────────────────────────────────────

/// Parser for NORAD Two-Line Element sets.
pub struct TleParser;

impl TleParser {
    /// Parse two TLE lines into [`OrbitalElements`].
    ///
    /// Each line must be exactly 69 characters. Line 1 starts with '1',
    /// line 2 starts with '2'.
    pub fn parse(line1: &str, line2: &str) -> Result<OrbitalElements, TleParseError> {
        if line1.len() != 69 {
            return Err(TleParseError::InvalidLength { line: 1, len: line1.len() });
        }
        if line2.len() != 69 {
            return Err(TleParseError::InvalidLength { line: 2, len: line2.len() });
        }
        if !line1.starts_with('1') {
            return Err(TleParseError::InvalidLineNumber { line: 1 });
        }
        if !line2.starts_with('2') {
            return Err(TleParseError::InvalidLineNumber { line: 2 });
        }

        // Verify checksums
        Self::verify_checksum(line1, 1)?;
        Self::verify_checksum(line2, 2)?;

        // ── Line 1 fields ──
        let norad_id: u32 = line1[2..7].trim().parse()
            .map_err(|_| TleParseError::ParseField("norad_id"))?;

        let intl_designator = line1[9..17].trim().to_string();

        let epoch_year_2digit: u32 = line1[18..20].trim().parse()
            .map_err(|_| TleParseError::ParseField("epoch_year"))?;
        let epoch_year = if epoch_year_2digit >= 57 {
            1900 + epoch_year_2digit
        } else {
            2000 + epoch_year_2digit
        };

        let epoch_day: f64 = line1[20..32].trim().parse()
            .map_err(|_| TleParseError::ParseField("epoch_day"))?;

        let ndot_over_2: f64 = line1[33..43].trim().parse()
            .map_err(|_| TleParseError::ParseField("ndot_over_2"))?;

        let bstar = Self::parse_tle_float(&line1[53..61])?;

        // ── Line 2 fields ──
        let inclination_deg: f64 = line2[8..16].trim().parse()
            .map_err(|_| TleParseError::ParseField("inclination"))?;

        let raan_deg: f64 = line2[17..25].trim().parse()
            .map_err(|_| TleParseError::ParseField("raan"))?;

        // Eccentricity: has implied leading "0."
        let ecc_str = format!("0.{}", line2[26..33].trim());
        let eccentricity: f64 = ecc_str.parse()
            .map_err(|_| TleParseError::ParseField("eccentricity"))?;

        let arg_perigee_deg: f64 = line2[34..42].trim().parse()
            .map_err(|_| TleParseError::ParseField("arg_perigee"))?;

        let mean_anomaly_deg: f64 = line2[43..51].trim().parse()
            .map_err(|_| TleParseError::ParseField("mean_anomaly"))?;

        let mean_motion_rev_day: f64 = line2[52..63].trim().parse()
            .map_err(|_| TleParseError::ParseField("mean_motion"))?;

        let rev_number: u32 = line2[63..68].trim().parse()
            .map_err(|_| TleParseError::ParseField("rev_number"))?;

        // Convert to internal units
        let mean_motion_rad_min = mean_motion_rev_day * TWO_PI / MIN_PER_DAY;

        Ok(OrbitalElements {
            norad_id,
            intl_designator,
            epoch_year,
            epoch_day,
            ndot_over_2,
            bstar,
            inclination: inclination_deg * DEG2RAD,
            raan: raan_deg * DEG2RAD,
            eccentricity,
            arg_perigee: arg_perigee_deg * DEG2RAD,
            mean_anomaly: mean_anomaly_deg * DEG2RAD,
            mean_motion: mean_motion_rad_min,
            rev_number,
        })
    }

    /// Parse TLE-format scientific notation: " 12345-6" => 0.12345e-6
    fn parse_tle_float(s: &str) -> Result<f64, TleParseError> {
        let s = s.trim();
        if s.is_empty() || s == "00000-0" || s == "00000+0" {
            return Ok(0.0);
        }

        // Format: [+-]NNNNN[+-]N  e.g., " 10270-3" means 0.10270e-3
        // or with leading sign: "-11606-4" means -0.11606e-4
        let (sign, rest) = if s.starts_with('-') {
            (-1.0, &s[1..])
        } else if s.starts_with('+') {
            (1.0, &s[1..])
        } else {
            (1.0, s)
        };

        // Find the exponent sign (last + or - that is not at position 0)
        let exp_pos = rest.rfind(|c: char| c == '+' || c == '-');
        match exp_pos {
            Some(pos) if pos > 0 => {
                let mantissa_str = &rest[..pos];
                let exp_str = &rest[pos..];
                let mantissa: f64 = format!("0.{}", mantissa_str).parse()
                    .map_err(|_| TleParseError::ParseField("bstar_mantissa"))?;
                let exp: i32 = exp_str.parse()
                    .map_err(|_| TleParseError::ParseField("bstar_exponent"))?;
                Ok(sign * mantissa * 10.0f64.powi(exp))
            }
            _ => {
                // No exponent found, try direct parse
                let val: f64 = format!("0.{}", rest).parse()
                    .map_err(|_| TleParseError::ParseField("bstar_direct"))?;
                Ok(sign * val)
            }
        }
    }

    /// Verify TLE line checksum (modulo-10 sum of digits, minus signs count as 1).
    fn verify_checksum(line: &str, line_num: u8) -> Result<(), TleParseError> {
        let expected: u32 = line.as_bytes()[68]
            .wrapping_sub(b'0') as u32;
        let mut sum: u32 = 0;
        for &b in &line.as_bytes()[..68] {
            if b >= b'0' && b <= b'9' {
                sum += (b - b'0') as u32;
            } else if b == b'-' {
                sum += 1;
            }
        }
        if sum % 10 != expected {
            return Err(TleParseError::ChecksumMismatch {
                line: line_num,
                expected,
                computed: sum % 10,
            });
        }
        Ok(())
    }
}

/// Errors that can occur during TLE parsing.
#[derive(Debug, Clone)]
pub enum TleParseError {
    /// A TLE line has the wrong length.
    InvalidLength { line: u8, len: usize },
    /// A TLE line does not start with the expected line number.
    InvalidLineNumber { line: u8 },
    /// A specific field could not be parsed.
    ParseField(&'static str),
    /// Line checksum verification failed.
    ChecksumMismatch { line: u8, expected: u32, computed: u32 },
}

impl std::fmt::Display for TleParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TleParseError::InvalidLength { line, len } => {
                write!(f, "TLE line {} has invalid length {} (expected 69)", line, len)
            }
            TleParseError::InvalidLineNumber { line } => {
                write!(f, "TLE line {} does not start with correct line number", line)
            }
            TleParseError::ParseField(field) => {
                write!(f, "Failed to parse TLE field: {}", field)
            }
            TleParseError::ChecksumMismatch { line, expected, computed } => {
                write!(
                    f,
                    "TLE line {} checksum mismatch: expected {}, computed {}",
                    line, expected, computed
                )
            }
        }
    }
}

// ─── ECI State Vector ─────────────────────────────────────────────────────────

/// Satellite state vector in Earth-Centered Inertial (ECI) frame.
#[derive(Debug, Clone)]
pub struct EciState {
    /// Position [x, y, z] in km (TEME frame)
    pub position: [f64; 3],
    /// Velocity [vx, vy, vz] in km/s (TEME frame)
    pub velocity: [f64; 3],
    /// Time offset from epoch in minutes
    pub time_minutes: f64,
}

impl EciState {
    /// Magnitude of position vector in km.
    pub fn radius_km(&self) -> f64 {
        (self.position[0].powi(2) + self.position[1].powi(2) + self.position[2].powi(2)).sqrt()
    }

    /// Magnitude of velocity vector in km/s.
    pub fn speed_km_s(&self) -> f64 {
        (self.velocity[0].powi(2) + self.velocity[1].powi(2) + self.velocity[2].powi(2)).sqrt()
    }

    /// Altitude above the equatorial Earth radius in km.
    pub fn altitude_km(&self) -> f64 {
        self.radius_km() - RE_EARTH
    }
}

// ─── Observer Position ────────────────────────────────────────────────────────

/// Ground observer position in geodetic coordinates.
#[derive(Debug, Clone)]
pub struct ObserverPosition {
    /// Geodetic latitude in degrees (north positive)
    pub lat_deg: f64,
    /// Geodetic longitude in degrees (east positive)
    pub lon_deg: f64,
    /// Altitude above WGS-84 ellipsoid in meters
    pub alt_m: f64,
}

impl ObserverPosition {
    /// Convert geodetic coordinates to ECEF [x, y, z] in km.
    pub fn to_ecef_km(&self) -> [f64; 3] {
        let lat = self.lat_deg * DEG2RAD;
        let lon = self.lon_deg * DEG2RAD;
        let alt_km = self.alt_m / 1000.0;

        let e2 = FLATTENING * (2.0 - FLATTENING);
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let n = RE_EARTH / (1.0 - e2 * sin_lat * sin_lat).sqrt();

        [
            (n + alt_km) * cos_lat * lon.cos(),
            (n + alt_km) * cos_lat * lon.sin(),
            (n * (1.0 - e2) + alt_km) * sin_lat,
        ]
    }
}

// ─── Observation Result ───────────────────────────────────────────────────────

/// Result of satellite observation from a ground observer.
#[derive(Debug, Clone)]
pub struct ObservationResult {
    /// Slant range from observer to satellite in km
    pub range_km: f64,
    /// Elevation angle in degrees (above horizon)
    pub elevation_deg: f64,
    /// Azimuth angle in degrees (clockwise from north)
    pub azimuth_deg: f64,
    /// Range rate in km/s (positive = moving away)
    pub range_rate_km_s: f64,
    /// Doppler shift in Hz for a given carrier frequency
    pub doppler_hz: f64,
}

// ─── Pass Event ───────────────────────────────────────────────────────────────

/// A satellite pass over an observer's location.
#[derive(Debug, Clone)]
pub struct PassEvent {
    /// Time of acquisition of signal (rise above min elevation), minutes from epoch
    pub aos_minutes: f64,
    /// Time of loss of signal (set below min elevation), minutes from epoch
    pub los_minutes: f64,
    /// Time of maximum elevation, minutes from epoch
    pub max_el_minutes: f64,
    /// Maximum elevation angle in degrees
    pub max_elevation_deg: f64,
    /// Duration of the pass in minutes
    pub duration_minutes: f64,
}

// ─── Simplified SGP4 Propagator ───────────────────────────────────────────────

/// Simplified SGP4 orbital propagator.
///
/// Implements secular perturbations due to the J2 zonal harmonic and
/// atmospheric drag (B* term). This is a simplified version of the full
/// SGP4/SDP4 propagator, suitable for LEO/MEO short-term predictions.
pub struct Sgp4Propagator {
    /// Original orbital elements at epoch
    elements: OrbitalElements,
    /// Semi-major axis at epoch (km)
    a0: f64,
    /// Recovered mean motion (rad/min)
    n0: f64,
    /// Intermediate values cached for propagation
    cos_i0: f64,
    sin_i0: f64,
    /// Semi-latus rectum parameter
    #[allow(dead_code)]
    p0: f64,
}

impl Sgp4Propagator {
    /// Create a new SGP4 propagator from orbital elements.
    pub fn new(elements: &OrbitalElements) -> Self {
        let e0 = elements.eccentricity;
        let i0 = elements.inclination;
        let n0 = elements.mean_motion; // rad/min

        // Semi-major axis from mean motion (km)
        let n0_rad_s = n0 / 60.0;
        let a0 = (MU_EARTH / (n0_rad_s * n0_rad_s)).powf(1.0 / 3.0);

        let cos_i0 = i0.cos();
        let sin_i0 = i0.sin();

        let p0 = a0 * (1.0 - e0 * e0);

        Sgp4Propagator {
            elements: elements.clone(),
            a0,
            n0,
            cos_i0,
            sin_i0,
            p0,
        }
    }

    /// Propagate satellite state to `dt_minutes` after epoch.
    ///
    /// Returns the ECI position and velocity.
    pub fn propagate_minutes(&self, dt_minutes: f64) -> EciState {
        let e0 = self.elements.eccentricity;
        let n0 = self.n0;
        let a0 = self.a0;

        // ── J2 secular perturbations ──
        let p0 = a0 * (1.0 - e0 * e0);
        let j2_factor = 1.5 * J2 * (RE_EARTH / p0).powi(2);

        // Rate of change of RAAN (rad/min)
        let raan_dot = -j2_factor * n0 * self.cos_i0;

        // Rate of change of argument of perigee (rad/min)
        let omega_dot = j2_factor * n0 * (2.0 - 2.5 * self.sin_i0 * self.sin_i0);

        // Drag effect on mean motion: simple B* model
        // dn/dt ~ ndot_over_2 * 2 (in rev/day^2, converted)
        let ndot_rad_min2 = self.elements.ndot_over_2 * 2.0 * TWO_PI / (MIN_PER_DAY * MIN_PER_DAY);

        // Updated mean motion
        let n_t = n0 + ndot_rad_min2 * dt_minutes;

        // Updated semi-major axis from mean motion
        let n_t_rad_s = n_t / 60.0;
        let a_t = (MU_EARTH / (n_t_rad_s * n_t_rad_s)).powf(1.0 / 3.0);

        // Updated eccentricity (simplified drag decay)
        let e_t = (e0 - 2.0 * self.elements.bstar * RE_EARTH * dt_minutes * 60.0 / a0)
            .max(1e-6)
            .min(1.0 - 1e-6);

        // Updated angular elements
        let m_t = self.elements.mean_anomaly + n0 * dt_minutes
            + 0.5 * ndot_rad_min2 * dt_minutes * dt_minutes;
        let m_t = normalize_angle(m_t);

        let raan_t = normalize_angle(self.elements.raan + raan_dot * dt_minutes);
        let omega_t = normalize_angle(self.elements.arg_perigee + omega_dot * dt_minutes);

        // ── Solve Kepler equation ──
        let ecc_anomaly = solve_kepler(m_t, e_t, 1e-12, 50);

        // True anomaly
        let sin_e = ecc_anomaly.sin();
        let cos_e = ecc_anomaly.cos();
        let sqrt_1me2 = (1.0 - e_t * e_t).sqrt();
        let true_anomaly = f64::atan2(sqrt_1me2 * sin_e, cos_e - e_t);

        // Distance
        let r = a_t * (1.0 - e_t * cos_e);

        // Position in orbital plane
        let u = omega_t + true_anomaly;
        let cos_u = u.cos();
        let sin_u = u.sin();

        let cos_raan = raan_t.cos();
        let sin_raan = raan_t.sin();
        let cos_i = self.elements.inclination.cos();
        let sin_i = self.elements.inclination.sin();

        // ECI position (km)
        let x = r * (cos_raan * cos_u - sin_raan * sin_u * cos_i);
        let y = r * (sin_raan * cos_u + cos_raan * sin_u * cos_i);
        let z = r * sin_u * sin_i;

        // Velocity computation
        let p_t = a_t * (1.0 - e_t * e_t);
        let r_dot = (MU_EARTH / p_t).sqrt() * e_t * true_anomaly.sin();
        let r_f_dot = (MU_EARTH / p_t).sqrt() * (1.0 + e_t * true_anomaly.cos());

        let vx = r_dot * (cos_raan * cos_u - sin_raan * sin_u * cos_i)
            - r_f_dot * (cos_raan * sin_u + sin_raan * cos_u * cos_i);
        let vy = r_dot * (sin_raan * cos_u + cos_raan * sin_u * cos_i)
            - r_f_dot * (sin_raan * sin_u - cos_raan * cos_u * cos_i);
        let vz = r_dot * sin_u * sin_i + r_f_dot * cos_u * sin_i;

        EciState {
            position: [x, y, z],
            velocity: [vx, vy, vz],
            time_minutes: dt_minutes,
        }
    }

    /// Compute the observation result from a ground observer at a given time.
    ///
    /// The `dt_minutes` is time since the TLE epoch.
    pub fn observe(
        &self,
        dt_minutes: f64,
        observer: &ObserverPosition,
        carrier_freq_hz: f64,
    ) -> ObservationResult {
        let state = self.propagate_minutes(dt_minutes);

        // GMST at observation time (approximate: epoch GMST = 0 + Earth rotation)
        let gmst = OMEGA_EARTH * dt_minutes * 60.0;

        // Convert ECI to ECEF
        let cos_g = gmst.cos();
        let sin_g = gmst.sin();
        let sat_ecef = [
            state.position[0] * cos_g + state.position[1] * sin_g,
            -state.position[0] * sin_g + state.position[1] * cos_g,
            state.position[2],
        ];
        let sat_vel_ecef = [
            state.velocity[0] * cos_g + state.velocity[1] * sin_g
                + OMEGA_EARTH * (-state.position[0] * sin_g + state.position[1] * cos_g),
            -state.velocity[0] * sin_g + state.velocity[1] * cos_g
                + OMEGA_EARTH * (-state.position[0] * cos_g - state.position[1] * sin_g),
            state.velocity[2],
        ];

        let obs_ecef = observer.to_ecef_km();

        // Range vector (ECEF)
        let dx = sat_ecef[0] - obs_ecef[0];
        let dy = sat_ecef[1] - obs_ecef[1];
        let dz = sat_ecef[2] - obs_ecef[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();

        // Range rate
        let dvx = sat_vel_ecef[0];
        let dvy = sat_vel_ecef[1];
        let dvz = sat_vel_ecef[2];
        let range_rate = (dx * dvx + dy * dvy + dz * dvz) / range;

        // Convert range vector to topocentric (ENU) for elevation/azimuth
        let lat = observer.lat_deg * DEG2RAD;
        let lon = observer.lon_deg * DEG2RAD;
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let sin_lon = lon.sin();
        let cos_lon = lon.cos();

        // Rotation from ECEF to ENU
        let east = -sin_lon * dx + cos_lon * dy;
        let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
        let up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

        let elevation = (up / range).asin() * RAD2DEG;
        let azimuth = east.atan2(north) * RAD2DEG;
        let azimuth = if azimuth < 0.0 { azimuth + 360.0 } else { azimuth };

        // Doppler shift: f_d = -f * v_r / c
        let doppler = -carrier_freq_hz * (range_rate * 1000.0) / C_LIGHT;

        ObservationResult {
            range_km: range,
            elevation_deg: elevation,
            azimuth_deg: azimuth,
            range_rate_km_s: range_rate,
            doppler_hz: doppler,
        }
    }

    /// Compute Doppler shift in Hz at a given time for an observer and carrier frequency.
    pub fn doppler_hz(
        &self,
        dt_minutes: f64,
        observer: &ObserverPosition,
        carrier_freq_hz: f64,
    ) -> f64 {
        self.observe(dt_minutes, observer, carrier_freq_hz).doppler_hz
    }

    /// Compute the elevation angle in degrees at a given time for an observer.
    pub fn elevation_deg(&self, dt_minutes: f64, observer: &ObserverPosition) -> f64 {
        self.observe(dt_minutes, observer, 0.0).elevation_deg
    }

    /// Compute the slant range in km at a given time for an observer.
    pub fn range_km(&self, dt_minutes: f64, observer: &ObserverPosition) -> f64 {
        self.observe(dt_minutes, observer, 0.0).range_km
    }

    /// Check if the satellite is visible (above a minimum elevation) at a given time.
    pub fn is_visible(
        &self,
        dt_minutes: f64,
        observer: &ObserverPosition,
        min_elevation_deg: f64,
    ) -> bool {
        self.elevation_deg(dt_minutes, observer) >= min_elevation_deg
    }

    /// Find pass events (rise/set) within a time window.
    ///
    /// Searches from `start_minutes` to `start_minutes + duration_minutes`
    /// with the given step size. Returns passes where the satellite rises
    /// above `min_elevation_deg`.
    pub fn find_passes(
        &self,
        observer: &ObserverPosition,
        start_minutes: f64,
        duration_minutes: f64,
        step_minutes: f64,
        min_elevation_deg: f64,
    ) -> Vec<PassEvent> {
        let mut passes = Vec::new();
        let mut t = start_minutes;
        let end = start_minutes + duration_minutes;

        let mut in_pass = false;
        let mut aos = 0.0;
        let mut max_el = f64::NEG_INFINITY;
        let mut max_el_time = 0.0;

        while t <= end {
            let el = self.elevation_deg(t, observer);

            if el >= min_elevation_deg {
                if !in_pass {
                    // AOS: refine with bisection
                    aos = self.bisect_elevation(
                        observer,
                        (t - step_minutes).max(start_minutes),
                        t,
                        min_elevation_deg,
                        20,
                    );
                    in_pass = true;
                    max_el = el;
                    max_el_time = t;
                }
                if el > max_el {
                    max_el = el;
                    max_el_time = t;
                }
            } else if in_pass {
                // LOS: refine with bisection
                let los = self.bisect_elevation(
                    observer,
                    t - step_minutes,
                    t,
                    min_elevation_deg,
                    20,
                );
                passes.push(PassEvent {
                    aos_minutes: aos,
                    los_minutes: los,
                    max_el_minutes: max_el_time,
                    max_elevation_deg: max_el,
                    duration_minutes: los - aos,
                });
                in_pass = false;
                max_el = f64::NEG_INFINITY;
            }

            t += step_minutes;
        }

        // Handle pass that extends beyond the window
        if in_pass {
            passes.push(PassEvent {
                aos_minutes: aos,
                los_minutes: end,
                max_el_minutes: max_el_time,
                max_elevation_deg: max_el,
                duration_minutes: end - aos,
            });
        }

        passes
    }

    /// Bisect to find the time when elevation crosses a threshold.
    fn bisect_elevation(
        &self,
        observer: &ObserverPosition,
        t_low: f64,
        t_high: f64,
        threshold_deg: f64,
        iterations: u32,
    ) -> f64 {
        let mut lo = t_low;
        let mut hi = t_high;
        for _ in 0..iterations {
            let mid = (lo + hi) / 2.0;
            let el = self.elevation_deg(mid, observer);
            if el >= threshold_deg {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        (lo + hi) / 2.0
    }
}

// ─── Kepler Equation Solver ───────────────────────────────────────────────────

/// Solve Kepler's equation `M = E - e*sin(E)` for eccentric anomaly `E`.
///
/// Uses Newton-Raphson iteration starting from `M` as initial guess.
pub fn solve_kepler(mean_anomaly: f64, eccentricity: f64, tolerance: f64, max_iter: u32) -> f64 {
    let m = normalize_angle(mean_anomaly);
    let mut e_anom = if eccentricity > 0.8 { PI } else { m };

    for _ in 0..max_iter {
        let f = e_anom - eccentricity * e_anom.sin() - m;
        let f_prime = 1.0 - eccentricity * e_anom.cos();
        let delta = f / f_prime;
        e_anom -= delta;
        if delta.abs() < tolerance {
            break;
        }
    }
    e_anom
}

/// Solve Kepler's equation in the complex plane using `(f64, f64)` tuples.
///
/// This is an educational variant that demonstrates the Newton-Raphson
/// method using complex arithmetic, useful for understanding convergence
/// basins. For real orbits, use [`solve_kepler`] instead.
pub fn solve_kepler_complex(
    mean_anomaly: (f64, f64),
    eccentricity: f64,
    tolerance: f64,
    max_iter: u32,
) -> (f64, f64) {
    // z = E (complex eccentric anomaly)
    let mut z = mean_anomaly;

    for _ in 0..max_iter {
        // f(z) = z - e * sin(z) - M
        let sin_z = complex_sin(z);
        let f = complex_sub(
            complex_sub(z, complex_scale(sin_z, eccentricity)),
            mean_anomaly,
        );

        // f'(z) = 1 - e * cos(z)
        let cos_z = complex_cos(z);
        let fp = complex_sub((1.0, 0.0), complex_scale(cos_z, eccentricity));

        // Newton step: z = z - f(z)/f'(z)
        let delta = complex_div(f, fp);
        z = complex_sub(z, delta);

        if complex_abs(delta) < tolerance {
            break;
        }
    }
    z
}

// ─── Complex Number Helpers (f64, f64) ────────────────────────────────────────

/// Add two complex numbers.
#[inline]
pub fn complex_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers.
#[inline]
pub fn complex_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Multiply two complex numbers.
#[inline]
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Divide two complex numbers.
#[inline]
pub fn complex_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Scale a complex number by a real factor.
#[inline]
pub fn complex_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// Absolute value (modulus) of a complex number.
#[inline]
pub fn complex_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex sine: sin(a + bi) = sin(a)cosh(b) + i*cos(a)sinh(b)
#[inline]
pub fn complex_sin(z: (f64, f64)) -> (f64, f64) {
    (z.0.sin() * z.1.cosh(), z.0.cos() * z.1.sinh())
}

/// Complex cosine: cos(a + bi) = cos(a)cosh(b) - i*sin(a)sinh(b)
#[inline]
pub fn complex_cos(z: (f64, f64)) -> (f64, f64) {
    (z.0.cos() * z.1.cosh(), -z.0.sin() * z.1.sinh())
}

/// Complex exponential: exp(a + bi) = exp(a) * (cos(b) + i*sin(b))
#[inline]
pub fn complex_exp(z: (f64, f64)) -> (f64, f64) {
    let r = z.0.exp();
    (r * z.1.cos(), r * z.1.sin())
}

// ─── Utility ──────────────────────────────────────────────────────────────────

/// Normalize an angle to [0, 2pi).
fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % TWO_PI;
    if a < 0.0 {
        a += TWO_PI;
    }
    a
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ISS TLE for testing
    const ISS_LINE1: &str =
        "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9997";
    const ISS_LINE2: &str =
        "2 25544  51.6400 200.0000 0007417  50.0000 310.1200 15.49560000100002";

    #[test]
    fn test_parse_tle_basic() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        assert_eq!(el.norad_id, 25544);
        assert_eq!(el.epoch_year, 2024);
        assert!((el.epoch_day - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_parse_tle_inclination() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let inc_deg = el.inclination * RAD2DEG;
        assert!((inc_deg - 51.64).abs() < 0.01);
    }

    #[test]
    fn test_parse_tle_eccentricity() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        assert!((el.eccentricity - 0.0007417).abs() < 1e-8);
    }

    #[test]
    fn test_parse_tle_mean_motion() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        // 15.4956 rev/day => 15.4956 * 2pi / 1440 rad/min
        let expected = 15.4956 * TWO_PI / MIN_PER_DAY;
        assert!((el.mean_motion - expected).abs() < 1e-8);
    }

    #[test]
    fn test_parse_tle_invalid_length() {
        let short = "1 25544U 98067A   24001.50000000";
        let result = TleParser::parse(short, ISS_LINE2);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_tle_wrong_line_number() {
        let bad_line1 = format!("2{}", &ISS_LINE1[1..]);
        let result = TleParser::parse(&bad_line1, ISS_LINE2);
        assert!(result.is_err());
    }

    #[test]
    fn test_orbital_period() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let period = el.period_minutes();
        // ISS: ~92 minutes
        assert!(period > 88.0 && period < 96.0, "Period {} out of range", period);
    }

    #[test]
    fn test_apogee_perigee() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let apogee = el.apogee_altitude_km();
        let perigee = el.perigee_altitude_km();
        // ISS orbits at ~420 km altitude
        assert!(apogee > 350.0 && apogee < 500.0, "Apogee {} out of range", apogee);
        assert!(perigee > 350.0 && perigee < 500.0, "Perigee {} out of range", perigee);
        assert!(apogee >= perigee);
    }

    #[test]
    fn test_kepler_circular() {
        // For e=0, E should equal M
        let e = solve_kepler(1.0, 0.0, 1e-12, 50);
        assert!((e - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_kepler_eccentric() {
        // Verify M = E - e*sin(E)
        let m = 1.5;
        let ecc = 0.5;
        let e = solve_kepler(m, ecc, 1e-12, 50);
        let m_check = e - ecc * e.sin();
        assert!((m_check - m).abs() < 1e-10, "Kepler residual: {}", (m_check - m).abs());
    }

    #[test]
    fn test_kepler_high_eccentricity() {
        let m = 0.1;
        let ecc = 0.95;
        let e = solve_kepler(m, ecc, 1e-12, 50);
        let m_check = e - ecc * e.sin();
        assert!((m_check - m).abs() < 1e-10);
    }

    #[test]
    fn test_kepler_complex_real_axis() {
        // Complex Kepler solver on the real axis should match real solver
        let m = 1.5;
        let ecc = 0.3;
        let real_result = solve_kepler(m, ecc, 1e-12, 50);
        let complex_result = solve_kepler_complex((m, 0.0), ecc, 1e-10, 50);
        assert!((complex_result.0 - real_result).abs() < 1e-8);
        assert!(complex_result.1.abs() < 1e-8);
    }

    #[test]
    fn test_propagate_epoch() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let prop = Sgp4Propagator::new(&el);
        let state = prop.propagate_minutes(0.0);

        // At epoch, should have reasonable position
        let r = state.radius_km();
        assert!(r > 6500.0 && r < 7200.0, "Radius {} out of range", r);

        // Velocity should be ~7.5-7.8 km/s for LEO
        let v = state.speed_km_s();
        assert!(v > 7.0 && v < 8.5, "Velocity {} out of range", v);
    }

    #[test]
    fn test_propagate_half_orbit() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let prop = Sgp4Propagator::new(&el);
        let period = el.period_minutes();

        let state0 = prop.propagate_minutes(0.0);
        let state_half = prop.propagate_minutes(period / 2.0);

        // After half orbit, radius should still be reasonable
        let r = state_half.radius_km();
        assert!(r > 6500.0 && r < 7200.0);

        // Position should be roughly opposite
        let dot = state0.position[0] * state_half.position[0]
            + state0.position[1] * state_half.position[1]
            + state0.position[2] * state_half.position[2];
        // Dot product should be negative (opposite side)
        assert!(dot < 0.0, "Half-orbit dot product should be negative: {}", dot);
    }

    #[test]
    fn test_observer_ecef() {
        let obs = ObserverPosition { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 };
        let ecef = obs.to_ecef_km();
        // On equator at prime meridian, x ~ RE, y ~ 0, z ~ 0
        assert!((ecef[0] - RE_EARTH).abs() < 0.1);
        assert!(ecef[1].abs() < 0.001);
        assert!(ecef[2].abs() < 0.001);
    }

    #[test]
    fn test_observer_pole() {
        let obs = ObserverPosition { lat_deg: 90.0, lon_deg: 0.0, alt_m: 0.0 };
        let ecef = obs.to_ecef_km();
        // At north pole: x ~ 0, y ~ 0, z ~ RE * (1 - f)
        let polar_radius = RE_EARTH * (1.0 - FLATTENING);
        assert!(ecef[0].abs() < 0.001);
        assert!(ecef[1].abs() < 0.001);
        assert!((ecef[2] - polar_radius).abs() < 0.1);
    }

    #[test]
    fn test_range_computation() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let prop = Sgp4Propagator::new(&el);
        let obs = ObserverPosition { lat_deg: 40.0, lon_deg: -74.0, alt_m: 0.0 };

        let range = prop.range_km(0.0, &obs);
        // Range should be between ~400 km (overhead) and ~2500 km (at horizon)
        assert!(range > 200.0 && range < 15_000.0, "Range {} out of range", range);
    }

    #[test]
    fn test_doppler_sign() {
        // Doppler should be bounded for LEO at UHF
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let prop = Sgp4Propagator::new(&el);
        let obs = ObserverPosition { lat_deg: 40.0, lon_deg: -74.0, alt_m: 0.0 };

        let doppler = prop.doppler_hz(0.0, &obs, 437.0e6);
        // Max LEO Doppler at 437 MHz is about +/-12 kHz
        assert!(
            doppler.abs() < 15_000.0,
            "Doppler {} Hz seems too large for LEO",
            doppler
        );
    }

    #[test]
    fn test_pass_prediction() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let prop = Sgp4Propagator::new(&el);
        let obs = ObserverPosition { lat_deg: 40.0, lon_deg: -74.0, alt_m: 0.0 };

        // Search over 24 hours (1440 minutes) with 1-minute steps
        let passes = prop.find_passes(&obs, 0.0, 1440.0, 1.0, 5.0);

        // ISS makes ~15.5 orbits per day, observer should see some passes
        // Just verify the function doesn't panic and returns valid data
        for pass in &passes {
            assert!(pass.los_minutes > pass.aos_minutes);
            assert!(pass.max_elevation_deg >= 5.0);
            assert!(pass.duration_minutes > 0.0);
        }
    }

    #[test]
    fn test_complex_arithmetic() {
        // (1+2i) * (3+4i) = (1*3-2*4) + (1*4+2*3)i = -5 + 10i
        let result = complex_mul((1.0, 2.0), (3.0, 4.0));
        assert!((result.0 - (-5.0)).abs() < 1e-12);
        assert!((result.1 - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_div() {
        // (1+2i) / (1+2i) = 1 + 0i
        let result = complex_div((1.0, 2.0), (1.0, 2.0));
        assert!((result.0 - 1.0).abs() < 1e-12);
        assert!(result.1.abs() < 1e-12);
    }

    #[test]
    fn test_complex_exp_euler() {
        // exp(i*pi) = -1 + 0i (Euler's formula)
        let result = complex_exp((0.0, PI));
        assert!((result.0 - (-1.0)).abs() < 1e-12);
        assert!(result.1.abs() < 1e-12);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0)).abs() < 1e-12);
        assert!((normalize_angle(TWO_PI) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-12);
    }

    #[test]
    fn test_tle_float_parsing() {
        // "10270-3" means 0.10270e-3
        let val = TleParser::parse_tle_float(" 10270-3").unwrap();
        assert!((val - 0.10270e-3).abs() < 1e-12);
    }

    #[test]
    fn test_bstar_parsing() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        // B* = 0.10270e-3
        assert!((el.bstar - 0.10270e-3).abs() < 1e-10);
    }

    #[test]
    fn test_eci_altitude() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let prop = Sgp4Propagator::new(&el);
        let state = prop.propagate_minutes(0.0);
        let alt = state.altitude_km();
        // ISS ~420 km
        assert!(alt > 350.0 && alt < 500.0, "Altitude {} out of range", alt);
    }

    #[test]
    fn test_semi_major_axis() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        let sma = el.semi_major_axis_km();
        // ISS SMA ~ 6798 km (RE + 420 km)
        assert!(sma > 6700.0 && sma < 6900.0, "SMA {} out of range", sma);
    }

    #[test]
    fn test_intl_designator() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        assert_eq!(el.intl_designator, "98067A");
    }

    #[test]
    fn test_rev_number() {
        let el = TleParser::parse(ISS_LINE1, ISS_LINE2).unwrap();
        assert_eq!(el.rev_number, 10000);
    }
}

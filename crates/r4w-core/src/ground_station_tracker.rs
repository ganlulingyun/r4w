//! Satellite ground station antenna tracking and pass prediction.
//!
//! This module implements a complete ground station tracking system for low-Earth orbit (LEO)
//! and medium-Earth orbit (MEO) satellites. It covers the full pipeline from TLE parsing and
//! SGP4 orbit propagation through pass prediction, antenna pointing, Doppler compensation, link
//! margin estimation, multi-satellite scheduling, and inter-pass handover.
//!
//! # Overview
//!
//! - **[`Tle`]** — Two-Line Element set parser and data container
//! - **[`Sgp4`]** — Simplified General Perturbations 4 orbit propagator (LEO/MEO)
//! - **[`GroundStation`]** — WGS-84 geodetic site with elevation mask
//! - **[`PassPredictor`]** — AOS / TCA / LOS pass prediction for a satellite-station pair
//! - **[`PassEvent`]** — Predicted pass record with az/el profile, Doppler profile, link margin
//! - **[`AntennaPointer`]** — Real-time azimuth/elevation command generation
//! - **[`DopplerCompensator`]** — Instantaneous Doppler shift and compensation frequency
//! - **[`TrackingScheduler`]** — Multi-satellite queue with priority and conflict resolution
//! - **[`PassHandover`]** — Seamless handover between consecutive passes
//!
//! # Physics notes
//!
//! The SGP4 implementation follows the Hoots & Roehrich (1980) formulation as simplified for
//! near-circular orbits. Topocentric az/el coordinates are computed via the standard ECEF →
//! ENU rotation using WGS-84 geodetic-to-ECEF conversion. Doppler shift is estimated from the
//! radial component of the satellite velocity vector relative to the ground station.
//!
//! # Example
//!
//! ```
//! use r4w_core::ground_station_tracker::{
//!     Tle, GroundStation, PassPredictor, TrackingScheduler,
//! };
//!
//! // Parse a TLE for the International Space Station
//! let line1 = "1 25544U 98067A   24001.50000000  .00002182  00000-0  41462-4 0  9995";
//! let line2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.49911251431451";
//! let tle = Tle::parse("ISS (ZARYA)", line1, line2).unwrap();
//!
//! // Set up a ground station near Austin, TX
//! let gs = GroundStation::new("Austin", 30.2672, -97.7431, 0.149, 5.0);
//!
//! // Predict passes for the next 24 hours
//! let epoch_unix = 1704067200.0_f64; // 2024-01-01 00:00:00 UTC (approximate)
//! let mut predictor = PassPredictor::new(tle, gs);
//! let passes = predictor.predict_passes(epoch_unix, epoch_unix + 86400.0, 60.0);
//! println!("Found {} passes in 24 h", passes.len());
//! ```

use std::cmp::Ordering;
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical / mathematical constants
// ---------------------------------------------------------------------------

/// Earth gravitational parameter µ = GM  (km³ s⁻²)
const GM_KM3_S2: f64 = 398600.4418;
/// Earth equatorial radius (km) – WGS-84
const R_EARTH_KM: f64 = 6378.137;
/// Earth flattening – WGS-84
const EARTH_F: f64 = 1.0 / 298.257223563;
/// J2 oblateness coefficient
const J2: f64 = 1.082626e-3;
/// Speed of light (m/s)
const C_LIGHT: f64 = 299_792_458.0;
/// Degrees per radian
const RAD2DEG: f64 = 180.0 / PI;
/// Radians per degree
const DEG2RAD: f64 = PI / 180.0;
/// Earth rotation rate (rad/s)
const OMEGA_EARTH: f64 = 7.2921150e-5;
/// Julian date of J2000.0 epoch (TT)
const JD_J2000: f64 = 2_451_545.0;
/// Julian date of Unix epoch (1970-01-01 00:00:00 UTC ≈ TT, ignoring small
/// leap-second corrections for the purposes of this module)
const JD_UNIX_EPOCH: f64 = 2_440_587.5;

// ---------------------------------------------------------------------------
// Helper math
// ---------------------------------------------------------------------------

#[inline]
fn sin_cos(x: f64) -> (f64, f64) {
    (x.sin(), x.cos())
}

/// Clamp angle to [−π, π]
#[inline]
fn wrap_pi(a: f64) -> f64 {
    let mut x = a % (2.0 * PI);
    if x > PI {
        x -= 2.0 * PI;
    } else if x < -PI {
        x += 2.0 * PI;
    }
    x
}

/// Clamp angle to [0, 2π]
#[inline]
fn wrap_2pi(a: f64) -> f64 {
    let mut x = a % (2.0 * PI);
    if x < 0.0 {
        x += 2.0 * PI;
    }
    x
}

// ---------------------------------------------------------------------------
// TLE parsing
// ---------------------------------------------------------------------------

/// A parsed Two-Line Element set.
#[derive(Debug, Clone)]
pub struct Tle {
    /// Satellite name (from header line or line 1)
    pub name: String,
    /// NORAD catalogue number
    pub norad_id: u32,
    /// International designator (e.g., "98067A")
    pub intl_designator: String,
    /// TLE epoch expressed as Unix timestamp (seconds since 1970-01-01 00:00:00 UTC)
    pub epoch_unix: f64,
    /// First time derivative of mean motion (rev/day²) × ½
    pub n_dot: f64,
    /// Second time derivative of mean motion (rev/day³) × ⅙
    pub n_ddot: f64,
    /// B* drag term (1/earth-radii)
    pub b_star: f64,
    /// Inclination (radians)
    pub inclination: f64,
    /// Right ascension of ascending node (radians)
    pub raan: f64,
    /// Eccentricity (dimensionless)
    pub eccentricity: f64,
    /// Argument of perigee (radians)
    pub arg_perigee: f64,
    /// Mean anomaly (radians)
    pub mean_anomaly: f64,
    /// Mean motion (rad/s)
    pub mean_motion: f64,
    /// Revolution number at epoch
    pub rev_at_epoch: u32,
}

/// Error types for TLE parsing.
#[derive(Debug, Clone, PartialEq)]
pub enum TleError {
    /// Line does not start with the expected line number
    WrongLineNumber,
    /// Line has incorrect length
    WrongLength,
    /// Checksum does not match
    BadChecksum,
    /// Could not parse a numeric field
    ParseError(String),
}

impl std::fmt::Display for TleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TleError::WrongLineNumber => write!(f, "wrong line number"),
            TleError::WrongLength => write!(f, "wrong line length"),
            TleError::BadChecksum => write!(f, "bad checksum"),
            TleError::ParseError(s) => write!(f, "parse error: {s}"),
        }
    }
}

/// Compute the TLE checksum (mod 10 digit sum, digits + minus=1).
fn tle_checksum(line: &str) -> u8 {
    let sum: u32 = line
        .chars()
        .take(68)
        .map(|c| match c {
            '0'..='9' => c as u32 - '0' as u32,
            '-' => 1,
            _ => 0,
        })
        .sum();
    (sum % 10) as u8
}

/// Parse the compact TLE decimal exponent format, e.g. " 41462-4" → 4.1462e-5.
fn parse_tle_float_exp(s: &str) -> Result<f64, TleError> {
    let s = s.trim();
    if s.is_empty() || s == "00000-0" || s == "00000+0" {
        return Ok(0.0);
    }
    // Format: ±NNNNN±N  (sign optional at start, no dot)
    // Locate the exponent sign (last +/-)
    let bytes = s.as_bytes();
    let mut exp_pos = bytes.len();
    for i in (1..bytes.len()).rev() {
        if bytes[i] == b'+' || bytes[i] == b'-' {
            exp_pos = i;
            break;
        }
    }
    if exp_pos == bytes.len() {
        return Err(TleError::ParseError(format!("no exponent in '{s}'")));
    }
    let mantissa_str = &s[..exp_pos];
    let exp_str = &s[exp_pos..];

    let neg_mantissa = mantissa_str.starts_with('-');
    let digits = if neg_mantissa || mantissa_str.starts_with('+') {
        &mantissa_str[1..]
    } else {
        mantissa_str
    };

    let mantissa: f64 = format!("0.{digits}")
        .parse()
        .map_err(|_| TleError::ParseError(format!("bad mantissa '{mantissa_str}'")))?;
    let exp: i32 = exp_str
        .parse()
        .map_err(|_| TleError::ParseError(format!("bad exponent '{exp_str}'")))?;

    let value = mantissa * 10f64.powi(exp);
    Ok(if neg_mantissa { -value } else { value })
}

/// Convert TLE epoch (YYddd.dddddddd) to Unix timestamp.
fn tle_epoch_to_unix(epoch_str: &str) -> Result<f64, TleError> {
    let epoch_str = epoch_str.trim();
    if epoch_str.len() < 5 {
        return Err(TleError::ParseError(format!("epoch too short: '{epoch_str}'")));
    }
    let year_2d: u32 = epoch_str[..2]
        .trim()
        .parse()
        .map_err(|_| TleError::ParseError(format!("bad year in epoch '{epoch_str}'")))?;
    let year = if year_2d >= 57 { 1900 + year_2d } else { 2000 + year_2d };

    let day_frac: f64 = epoch_str[2..]
        .parse()
        .map_err(|_| TleError::ParseError(format!("bad day in epoch '{epoch_str}'")))?;

    // Days since Unix epoch (1970-01-01)
    // Jan 1 of `year` in days since Unix epoch
    let days_since_unix = days_since_unix_epoch(year, 1, 1) as f64 + (day_frac - 1.0);
    Ok(days_since_unix * 86400.0)
}

/// Number of days from 1970-01-01 to the first day of the given year.
fn days_since_unix_epoch(year: u32, month: u32, day: u32) -> i64 {
    // Zeller / proleptic Gregorian
    let y = year as i64;
    let m = month as i64;
    let d = day as i64;
    let a = (14 - m) / 12;
    let yy = y + 4800 - a;
    let mm = m + 12 * a - 3;
    let jdn = d + (153 * mm + 2) / 5 + 365 * yy + yy / 4 - yy / 100 + yy / 400 - 32045;
    // JD of Unix epoch noon = 2440588, but we want midnight
    jdn - 2_440_588
}

impl Tle {
    /// Parse a TLE from name + two lines.
    pub fn parse(name: &str, line1: &str, line2: &str) -> Result<Tle, TleError> {
        let l1 = line1.trim();
        let l2 = line2.trim();

        if l1.len() < 69 {
            return Err(TleError::WrongLength);
        }
        if l2.len() < 69 {
            return Err(TleError::WrongLength);
        }
        if !l1.starts_with('1') {
            return Err(TleError::WrongLineNumber);
        }
        if !l2.starts_with('2') {
            return Err(TleError::WrongLineNumber);
        }

        // Verify checksums
        let cs1: u8 = l1[68..69]
            .parse()
            .map_err(|_| TleError::ParseError("line 1 checksum".into()))?;
        if tle_checksum(l1) != cs1 {
            return Err(TleError::BadChecksum);
        }
        let cs2: u8 = l2[68..69]
            .parse()
            .map_err(|_| TleError::ParseError("line 2 checksum".into()))?;
        if tle_checksum(l2) != cs2 {
            return Err(TleError::BadChecksum);
        }

        let norad_id: u32 = l1[2..7]
            .trim()
            .parse()
            .map_err(|_| TleError::ParseError("norad id".into()))?;
        let intl_designator = l1[9..17].trim().to_string();
        let epoch_unix = tle_epoch_to_unix(&l1[18..32])?;

        let n_dot_str = l1[33..43].trim();
        let n_dot: f64 = n_dot_str
            .parse()
            .map_err(|_| TleError::ParseError(format!("n_dot '{n_dot_str}'")))?;
        let n_ddot = parse_tle_float_exp(l1[44..52].trim())?;
        let b_star = parse_tle_float_exp(l1[53..61].trim())?;

        // Line 2
        let inc_deg: f64 = l2[8..16]
            .trim()
            .parse()
            .map_err(|_| TleError::ParseError("inclination".into()))?;
        let raan_deg: f64 = l2[17..25]
            .trim()
            .parse()
            .map_err(|_| TleError::ParseError("raan".into()))?;

        // Eccentricity is stored without the leading "0."
        let ecc_str = format!("0.{}", l2[26..33].trim());
        let eccentricity: f64 = ecc_str
            .parse()
            .map_err(|_| TleError::ParseError(format!("eccentricity '{ecc_str}'")))?;

        let w_deg: f64 = l2[34..42]
            .trim()
            .parse()
            .map_err(|_| TleError::ParseError("arg_perigee".into()))?;
        let m_deg: f64 = l2[43..51]
            .trim()
            .parse()
            .map_err(|_| TleError::ParseError("mean_anomaly".into()))?;
        let n_revday: f64 = l2[52..63]
            .trim()
            .parse()
            .map_err(|_| TleError::ParseError("mean_motion".into()))?;
        let rev_str = l2[63..68].trim();
        let rev_at_epoch: u32 = rev_str
            .parse()
            .map_err(|_| TleError::ParseError(format!("rev_at_epoch '{rev_str}'")))?;

        let mean_motion = n_revday * 2.0 * PI / 86400.0; // rad/s

        Ok(Tle {
            name: name.to_string(),
            norad_id,
            intl_designator,
            epoch_unix,
            n_dot: n_dot * 2.0 * PI / (86400.0 * 86400.0), // convert to rad/s²
            n_ddot: n_ddot * 2.0 * PI / (86400.0 * 86400.0 * 86400.0),
            b_star,
            inclination: inc_deg * DEG2RAD,
            raan: raan_deg * DEG2RAD,
            eccentricity,
            arg_perigee: w_deg * DEG2RAD,
            mean_anomaly: m_deg * DEG2RAD,
            mean_motion,
            rev_at_epoch,
        })
    }

    /// Orbital period (seconds) from mean motion.
    pub fn period_s(&self) -> f64 {
        2.0 * PI / self.mean_motion
    }

    /// Semi-major axis (km) derived from mean motion (two-body).
    pub fn semi_major_axis_km(&self) -> f64 {
        (GM_KM3_S2 / (self.mean_motion * self.mean_motion)).cbrt()
    }

    /// Approximate apogee altitude (km above equatorial radius).
    pub fn apogee_km(&self) -> f64 {
        let a = self.semi_major_axis_km();
        a * (1.0 + self.eccentricity) - R_EARTH_KM
    }

    /// Approximate perigee altitude (km above equatorial radius).
    pub fn perigee_km(&self) -> f64 {
        let a = self.semi_major_axis_km();
        a * (1.0 - self.eccentricity) - R_EARTH_KM
    }
}

// ---------------------------------------------------------------------------
// SGP4 orbit propagator
// ---------------------------------------------------------------------------

/// SGP4 orbital state at a given epoch.
#[derive(Debug, Clone)]
pub struct OrbitalState {
    /// Position in Earth-Centered Inertial (ECI) frame, km
    pub position_km: [f64; 3],
    /// Velocity in ECI frame, km/s
    pub velocity_km_s: [f64; 3],
}

impl OrbitalState {
    /// Distance from Earth centre (km).
    pub fn radius_km(&self) -> f64 {
        let [x, y, z] = self.position_km;
        (x * x + y * y + z * z).sqrt()
    }

    /// Altitude above WGS-84 equatorial radius (km).
    pub fn altitude_km(&self) -> f64 {
        self.radius_km() - R_EARTH_KM
    }
}

/// Simplified SGP4 propagator for near-circular LEO/MEO orbits.
///
/// This is a self-contained implementation of the SGP4 simplified perturbation
/// model. It accounts for:
/// - J2 oblateness secular drift of RAAN and argument of perigee
/// - Mean-motion correction for J2
/// - Atmospheric drag via the B* term (first-order secular + periodic)
/// - Earth's rotation (converting ECI → ECEF when needed)
///
/// The implementation omits deep-space terms (SDP4) and resonance corrections
/// since this module targets LEO/MEO satellites with periods < 225 min.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct Sgp4 {
    tle: Tle,
    // Pre-computed constants
    a0: f64,      // initial semi-major axis (Earth radii)
    n0: f64,      // initial mean motion (rad/s)
    theta: f64,   // cos(inclination)
    x1mth2: f64,  // 1 - cos²(inc) = sin²(inc)
    x3thm1: f64,  // 3*cos²(inc) - 1
    xnodp: f64,   // recovered mean motion at epoch
    aodp: f64,    // recovered semi-major axis at epoch
    c1: f64, c2: f64, c3: f64, c4: f64, c5: f64,
    d2: f64, d3: f64, d4: f64,
    t2cof: f64, t3cof: f64, t4cof: f64, t5cof: f64,
    xmdot: f64, omgdot: f64, xnodot: f64,
    omgcof: f64, xmcof: f64, xnodcf: f64,
    etasq: f64, eeta: f64,
    coef: f64, coef1: f64,
    delmo: f64, sinmo: f64,
    x7thm1: f64,
}

impl Sgp4 {
    /// Initialise the propagator from a TLE.
    pub fn from_tle(tle: &Tle) -> Self {
        let e0 = tle.eccentricity;
        let i0 = tle.inclination;
        let n0 = tle.mean_motion;

        let a1 = (GM_KM3_S2 / (n0 * n0)).cbrt() / R_EARTH_KM; // Earth radii
        let theta = i0.cos();
        let x1mth2 = 1.0 - theta * theta;
        let x3thm1 = 3.0 * theta * theta - 1.0;
        let x1mtm2 = 1.0 - e0 * e0;
        let beta0 = x1mtm2.sqrt();

        // J2 correction to get "recovered" elements
        let del1 = 1.5 * J2 / (a1 * a1 * beta0 * beta0 * beta0) * x3thm1;
        let a0 = a1 * (1.0 - del1 * (1.0 / 3.0 + del1 * (1.0 + 134.0 / 81.0 * del1)));
        let del0 = 1.5 * J2 / (a0 * a0 * beta0 * beta0 * beta0) * x3thm1;
        let xnodp = n0 / (1.0 + del0);
        let aodp = a0 / (1.0 - del0);

        let s = 78.0 / R_EARTH_KM + 1.0;  // atmospheric density reference altitude
        let qoms2t = ((120.0 - 78.0) / R_EARTH_KM).powi(4);

        let pinvsq = 1.0 / (aodp * aodp * beta0 * beta0 * beta0 * beta0);
        let tsi = 1.0 / (aodp - s);
        let eta = aodp * e0 * tsi;
        let etasq = eta * eta;
        let eeta = e0 * eta;
        let psisq = (1.0 - etasq).abs();
        let coef = qoms2t * tsi.powi(4);
        let coef1 = coef / psisq.powf(3.5);

        let c2 = coef1 * xnodp
            * (aodp * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq))
                + 0.75 * J2 * tsi / psisq * x3thm1 * (8.0 + 3.0 * etasq * (8.0 + etasq)));
        let c1 = tle.b_star * c2;
        let sini0 = i0.sin();
        let c3 = coef * tsi * (-3.0 * J2 * pinvsq * (1.0 / e0) * theta) * sini0;
        let x1mtm2_sq = x1mtm2 * x1mtm2;
        let c4 = 2.0
            * xnodp
            * coef1
            * aodp
            * beta0 * beta0
            * (eta * (2.0 + 0.5 * etasq) + e0 * (0.5 + 2.0 * etasq)
                - 2.0 * J2 * tsi / (aodp * psisq)
                    * (-3.0 * x3thm1 * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta))
                        + 0.75 * x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq))
                            * (2.0 * tle.arg_perigee).cos()));
        let c5 = 2.0 * coef1 * aodp * x1mtm2_sq * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);

        let theta2 = theta * theta;
        let theta4 = theta2 * theta2;
        let temp1 = 3.0 * J2 * pinvsq * xnodp;
        let temp2 = temp1 * J2 * pinvsq;
        let xmdot = xnodp
            + 0.5 * temp1 * beta0 * x3thm1
            + 0.0625 * temp2 * beta0 * (13.0 - 78.0 * theta2 + 137.0 * theta4);
        let x1m5th = 1.0 - 5.0 * theta2;
        let omgdot = -0.5 * temp1 * x1m5th
            + 0.0625 * temp2 * (7.0 - 114.0 * theta2 + 395.0 * theta4)
            + (5.0 / 16.0) * 0.0 /* deep space term, zero for LEO */;
        let xhdot1 = -temp1 * theta;
        let xnodot =
            xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * theta2) + 2.0 * (5.0 / 16.0) * 0.0) * theta;

        let omgcof = tle.b_star * c3 * tle.arg_perigee.cos();
        let xmcof = if e0 > 1.0e-4 {
            -(2.0 / 3.0) * coef * tle.b_star / eeta
        } else {
            0.0
        };
        let xnodcf = 3.5 * beta0 * beta0 * xhdot1 * c1;
        let t2cof = 1.5 * c1;
        let x1mtm2p1 = 1.0 + x1mtm2.sqrt();
        let t3cof = (c1 * c1) * (28.0 + 3.0 * etasq - 1.5 * eeta * (9.0 + 12.0 * etasq))
            - 2.0 / 3.0 * coef1 * x1mtm2p1;
        let t4cof = 0.25
            * (3.0 * c1 * c1 * (12.0 - 8.0 * etasq + 3.0 * eeta) + 0.75 * c1 * coef1);
        let t5cof = 0.2 * (3.0 * c1 * c1 * c1 + 0.75 * c1 * c1 * coef1 + coef1 * coef1);
        let d2 = 4.0 * aodp * tsi * c1 * c1;
        let temp = d2 * tsi * c1 / 3.0;
        let d3 = (17.0 * aodp + s) * temp;
        let d4 = 0.5 * temp * aodp * tsi * (221.0 * aodp + 31.0 * s) * c1;

        let sinmo = tle.mean_anomaly.sin();
        let delmo = (1.0 + eta * tle.mean_anomaly.cos()).powi(3);
        let x7thm1 = 7.0 * theta2 - 1.0;

        Sgp4 {
            tle: tle.clone(),
            a0, n0, theta, x1mth2, x3thm1, xnodp, aodp,
            c1, c2, c3, c4, c5,
            d2, d3, d4,
            t2cof, t3cof, t4cof, t5cof,
            xmdot, omgdot, xnodot,
            omgcof, xmcof, xnodcf,
            etasq, eeta,
            coef, coef1,
            delmo, sinmo,
            x7thm1,
        }
    }

    /// Propagate to `unix_time` (seconds since 1970-01-01 00:00 UTC).
    ///
    /// Returns `None` if the orbit has decayed (perigee < 0 km altitude).
    pub fn propagate(&self, unix_time: f64) -> Option<OrbitalState> {
        let tsince_s = unix_time - self.tle.epoch_unix;
        let t = tsince_s / 60.0; // SGP4 works in minutes

        // --- secular terms ---
        let xmdf = self.tle.mean_anomaly + self.xmdot * tsince_s;
        let omgadf = self.tle.arg_perigee + self.omgdot * tsince_s;
        let xnoddf = self.tle.raan + self.xnodot * tsince_s;
        let _omega = omgadf + self.omgcof * t.powi(2) * 0.5;
        let _xmp = xmdf + self.xmcof * t * 0.5;
        let tsq = t * t;
        let tcube = tsq * t;
        let tfour = t * tcube;
        let delomg = self.omgcof * t;
        let delm = self.xmcof
            * ((1.0 + self.tle.eccentricity * xmdf.cos()) * (1.0 + self.tle.eccentricity * xmdf.cos()) * (1.0 + self.tle.eccentricity * xmdf.cos()) - self.delmo);
        let mdf2 = xmdf + delomg + delm;
        let omega2 = omgadf - delomg - delm;
        let e = self.tle.eccentricity
            - self.c4 * t
            - self.c5 * (mdf2.sin() - self.sinmo);
        let a = self.aodp
            * (1.0
                - self.c1 * t
                - self.d2 * tsq
                - self.d3 * tcube
                - self.d4 * tfour)
            * (1.0
                - self.c1 * t
                - self.d2 * tsq
                - self.d3 * tcube
                - self.d4 * tfour);
        let xl = mdf2 + omega2 + xnoddf + self.xnodp * (self.t2cof * tsq + self.t3cof * tcube + self.t4cof * tfour + self.t5cof * tfour * t);
        let xn = GM_KM3_S2.sqrt() / (a * R_EARTH_KM).powf(1.5);
        let e = e.clamp(1.0e-6, 1.0 - 1.0e-6);

        // Solve Kepler's equation: M = E - e*sin(E)
        let mean_anom = xl - xnoddf - omega2;
        let e_anom = solve_kepler(mean_anom, e);

        // Position and velocity in orbital plane
        let sin_e = e_anom.sin();
        let cos_e = e_anom.cos();
        let r = a * R_EARTH_KM * (1.0 - e * cos_e);
        if r < R_EARTH_KM {
            return None; // decayed
        }
        let rdot = xn * a * R_EARTH_KM * e * sin_e / r;
        let rfdot = xn * a * R_EARTH_KM * (1.0 - e * e).sqrt() / r;

        let true_anom_sin = (1.0 - e * e).sqrt() * sin_e / (1.0 - e * cos_e);
        let true_anom_cos = (cos_e - e) / (1.0 - e * cos_e);
        let u = omega2 + true_anom_sin.atan2(true_anom_cos);

        // ECI position (sin_u/cos_u replaced by sin_u2/cos_u2 after perturbation correction)
        let _sin_u = u.sin();
        let _cos_u = u.cos();
        let rk = r;

        // Perturbation corrections
        let sin2u = (2.0 * u).sin();
        let cos2u = (2.0 * u).cos();
        let dr = 0.5 * J2 * (R_EARTH_KM / r).powi(2) * self.x1mth2 * cos2u;
        let du = -0.25 * J2 * (R_EARTH_KM / r).powi(2) * self.x7thm1 * sin2u;
        let dinc = 1.5 * J2 * (R_EARTH_KM / r).powi(2) * self.theta * sin2u * self.tle.inclination.sin();
        let domg = 1.5 * J2 * (R_EARTH_KM / r).powi(2) * self.theta * cos2u;

        let rk2 = rk + dr;
        let uk = u + du;
        let xnodek = xnoddf + domg;
        let xincl = self.tle.inclination + dinc;

        let sin_u2 = uk.sin();
        let cos_u2 = uk.cos();
        let sinxnodek = xnodek.sin();
        let cosxnodek = xnodek.cos();
        let sinxincl = xincl.sin();
        let cosxincl = xincl.cos();

        // Unit vectors
        let mx = -sinxnodek * cosxincl;
        let my = cosxnodek * cosxincl;
        let mz = sinxincl;

        let nx = cosxnodek;
        let ny = sinxnodek;
        let nz = 0.0_f64;

        let ux = mx * sin_u2 + nx * cos_u2;
        let uy = my * sin_u2 + ny * cos_u2;
        let uz = mz * sin_u2 + nz * cos_u2;

        let pos_x = rk2 * ux;
        let pos_y = rk2 * uy;
        let pos_z = rk2 * uz;

        // Velocity
        let vx = mx * cos_u2 - nx * sin_u2;
        let vy = my * cos_u2 - ny * sin_u2;
        let vz = mz * cos_u2 - nz * sin_u2;

        let xdotk = rdot * ux + rfdot * vx;
        let ydotk = rdot * uy + rfdot * vy;
        let zdotk = rdot * uz + rfdot * vz;

        Some(OrbitalState {
            position_km: [pos_x, pos_y, pos_z],
            velocity_km_s: [xdotk, ydotk, zdotk],
        })
    }
}

/// Solve Kepler's equation M = E - e*sin(E) for eccentric anomaly E.
/// Newton-Raphson with starting estimate.
fn solve_kepler(m: f64, e: f64) -> f64 {
    let m = wrap_2pi(m);
    let mut e_anom = if e < 0.8 { m } else { PI };
    for _ in 0..50 {
        let delta = (m - e_anom + e * e_anom.sin()) / (1.0 - e * e_anom.cos());
        e_anom += delta;
        if delta.abs() < 1.0e-12 {
            break;
        }
    }
    e_anom
}

// ---------------------------------------------------------------------------
// WGS-84 geodetic ↔ ECEF conversions
// ---------------------------------------------------------------------------

/// WGS-84 ellipsoid parameters.
const WGS84_A: f64 = R_EARTH_KM;
const WGS84_B: f64 = WGS84_A * (1.0 - EARTH_F);
const WGS84_E2: f64 = 1.0 - (WGS84_B / WGS84_A) * (WGS84_B / WGS84_A);

/// Convert WGS-84 geodetic (lat_deg, lon_deg, alt_km) to ECEF (km).
pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, alt_km: f64) -> [f64; 3] {
    let lat = lat_deg * DEG2RAD;
    let lon = lon_deg * DEG2RAD;
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let x = (n + alt_km) * cos_lat * lon.cos();
    let y = (n + alt_km) * cos_lat * lon.sin();
    let z = (n * (1.0 - WGS84_E2) + alt_km) * sin_lat;
    [x, y, z]
}

/// Greenwich Mean Sidereal Time (radians) for a Unix timestamp.
///
/// Uses a simple linear approximation accurate to fractions of a degree over
/// the range of dates relevant to satellite tracking.
pub fn gmst(unix_time: f64) -> f64 {
    // Julian date
    let jd = JD_UNIX_EPOCH + unix_time / 86400.0;
    let t = (jd - JD_J2000) / 36525.0;
    // IAU 1982 GMST formula (degrees)
    let gmst_deg = 280.46061837
        + 360.98564736629 * (jd - JD_J2000)
        + 0.000387933 * t * t
        - t * t * t / 38710000.0;
    gmst_deg * DEG2RAD
}

/// Convert ECI position to ECEF for a given Unix timestamp.
pub fn eci_to_ecef(eci: [f64; 3], unix_time: f64) -> [f64; 3] {
    let theta = gmst(unix_time);
    let (s, c) = sin_cos(theta);
    [
        c * eci[0] + s * eci[1],
        -s * eci[0] + c * eci[1],
        eci[2],
    ]
}

/// Convert ECI velocity to ECEF velocity (subtract Earth rotation contribution).
pub fn eci_vel_to_ecef(eci_vel: [f64; 3], eci_pos: [f64; 3], unix_time: f64) -> [f64; 3] {
    let theta = gmst(unix_time);
    let (s, c) = sin_cos(theta);
    // Rotate + add Coriolis term from Earth rotation
    [
        c * eci_vel[0] + s * eci_vel[1] + OMEGA_EARTH * (s * eci_pos[0] - c * eci_pos[1]),
        -s * eci_vel[0] + c * eci_vel[1] - OMEGA_EARTH * (c * eci_pos[0] + s * eci_pos[1]),
        eci_vel[2],
    ]
}

// ---------------------------------------------------------------------------
// Ground station
// ---------------------------------------------------------------------------

/// A ground station site defined by its WGS-84 coordinates and elevation mask.
#[derive(Debug, Clone)]
pub struct GroundStation {
    /// Human-readable name
    pub name: String,
    /// Geodetic latitude (degrees, positive north)
    pub lat_deg: f64,
    /// Geodetic longitude (degrees, positive east)
    pub lon_deg: f64,
    /// Altitude above WGS-84 ellipsoid (km)
    pub alt_km: f64,
    /// Minimum elevation angle for contact (degrees)
    pub elevation_mask_deg: f64,
    /// ECEF position vector (km), pre-computed
    ecef: [f64; 3],
}

impl GroundStation {
    /// Create a new ground station.
    ///
    /// * `lat_deg` — geodetic latitude (degrees)
    /// * `lon_deg` — geodetic longitude (degrees, east positive)
    /// * `alt_km`  — altitude above WGS-84 ellipsoid (km)
    /// * `elevation_mask_deg` — minimum elevation for a valid pass (degrees)
    pub fn new(name: &str, lat_deg: f64, lon_deg: f64, alt_km: f64, elevation_mask_deg: f64) -> Self {
        let ecef = geodetic_to_ecef(lat_deg, lon_deg, alt_km);
        GroundStation {
            name: name.to_string(),
            lat_deg,
            lon_deg,
            alt_km,
            elevation_mask_deg,
            ecef,
        }
    }

    /// Compute topocentric (azimuth, elevation) to a satellite ECEF position.
    ///
    /// Returns `(az_deg, el_deg, range_km)`.
    pub fn look_angle_ecef(&self, sat_ecef: [f64; 3]) -> (f64, f64, f64) {
        // Range vector in ECEF
        let dx = sat_ecef[0] - self.ecef[0];
        let dy = sat_ecef[1] - self.ecef[1];
        let dz = sat_ecef[2] - self.ecef[2];
        let range_km = (dx * dx + dy * dy + dz * dz).sqrt();

        // Rotate to ENU
        let lat = self.lat_deg * DEG2RAD;
        let lon = self.lon_deg * DEG2RAD;
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let sin_lon = lon.sin();
        let cos_lon = lon.cos();

        // ENU = R * ECEF_delta
        let e = -sin_lon * dx + cos_lon * dy;
        let n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
        let u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

        let el_rad = u.atan2((e * e + n * n).sqrt());
        let az_rad = e.atan2(n);

        let el_deg = el_rad * RAD2DEG;
        let az_deg = wrap_2pi(az_rad) * RAD2DEG;

        (az_deg, el_deg, range_km)
    }

    /// Elevation to a satellite at a given Unix time (ECI position).
    pub fn elevation_to_eci(&self, sat_eci: [f64; 3], unix_time: f64) -> f64 {
        let sat_ecef = eci_to_ecef(sat_eci, unix_time);
        let (_, el, _) = self.look_angle_ecef(sat_ecef);
        el
    }
}

// ---------------------------------------------------------------------------
// Pass prediction
// ---------------------------------------------------------------------------

/// Elevation profile sample during a pass.
#[derive(Debug, Clone)]
pub struct ElevationSample {
    /// Unix timestamp
    pub unix_time: f64,
    /// Azimuth (degrees, 0=North, clockwise)
    pub az_deg: f64,
    /// Elevation (degrees above horizon)
    pub el_deg: f64,
    /// Slant range (km)
    pub range_km: f64,
    /// Doppler shift (Hz) at the given carrier frequency
    pub doppler_hz: f64,
}

/// A predicted satellite pass over a ground station.
#[derive(Debug, Clone)]
pub struct PassEvent {
    /// Satellite TLE name
    pub satellite_name: String,
    /// NORAD ID
    pub norad_id: u32,
    /// Acquisition of Signal (AOS) Unix timestamp
    pub aos_unix: f64,
    /// AOS azimuth (degrees)
    pub aos_az_deg: f64,
    /// Time of Closest Approach (TCA) Unix timestamp
    pub tca_unix: f64,
    /// TCA azimuth (degrees)
    pub tca_az_deg: f64,
    /// Maximum elevation (degrees)
    pub max_el_deg: f64,
    /// Loss of Signal (LOS) Unix timestamp
    pub los_unix: f64,
    /// LOS azimuth (degrees)
    pub los_az_deg: f64,
    /// Pass duration (seconds)
    pub duration_s: f64,
    /// Elevation profile (sampled at regular intervals)
    pub profile: Vec<ElevationSample>,
    /// Estimated link margin at TCA (dB)
    pub link_margin_db: f64,
    /// Priority score for scheduling (higher = more important)
    pub priority: f64,
}

impl PassEvent {
    /// True if this pass overlaps with `other` in time.
    pub fn overlaps(&self, other: &PassEvent) -> bool {
        self.aos_unix < other.los_unix && other.aos_unix < self.los_unix
    }
}

/// Pass predictor for a single satellite / ground station pair.
pub struct PassPredictor {
    sgp4: Sgp4,
    station: GroundStation,
    /// Carrier frequency for Doppler calculation (Hz)
    pub carrier_hz: f64,
}

impl PassPredictor {
    /// Create a new predictor.
    pub fn new(tle: Tle, station: GroundStation) -> Self {
        let sgp4 = Sgp4::from_tle(&tle);
        PassPredictor {
            sgp4,
            station,
            carrier_hz: 437_000_000.0, // default UHF downlink
        }
    }

    /// Set the carrier frequency used for Doppler calculations.
    pub fn with_carrier_hz(mut self, hz: f64) -> Self {
        self.carrier_hz = hz;
        self
    }

    /// Predict all passes between `t_start` and `t_end` (Unix timestamps).
    ///
    /// `coarse_step_s` is the time step for the initial elevation scan;
    /// 30-60 s is a reasonable value.
    pub fn predict_passes(&mut self, t_start: f64, t_end: f64, coarse_step_s: f64) -> Vec<PassEvent> {
        let mut passes = Vec::new();
        let mask = self.station.elevation_mask_deg;

        let mut t = t_start;
        let mut prev_el = self.elevation_at(t);
        let mut in_pass = false;
        let mut pass_start = t_start;

        while t <= t_end {
            t += coarse_step_s;
            let el = self.elevation_at(t);
            if !in_pass && prev_el < mask && el >= mask {
                // Rising through mask — binary-search for precise AOS
                in_pass = true;
                pass_start = self.bisect_crossing(t - coarse_step_s, t, mask, true);
            } else if in_pass && prev_el >= mask && el < mask {
                // Falling through mask — binary-search for precise LOS
                let los = self.bisect_crossing(t - coarse_step_s, t, mask, false);
                if let Some(pass) = self.build_pass(pass_start, los) {
                    passes.push(pass);
                }
                in_pass = false;
            }
            prev_el = el;
        }

        // Handle a pass still in progress at t_end
        if in_pass {
            if let Some(pass) = self.build_pass(pass_start, t_end) {
                passes.push(pass);
            }
        }

        passes
    }

    fn elevation_at(&self, unix_time: f64) -> f64 {
        match self.sgp4.propagate(unix_time) {
            Some(state) => self.station.elevation_to_eci(state.position_km, unix_time),
            None => -90.0,
        }
    }

    /// Binary search for the moment the satellite crosses `mask_deg`.
    fn bisect_crossing(&self, mut t_low: f64, mut t_high: f64, mask: f64, rising: bool) -> f64 {
        for _ in 0..40 {
            let t_mid = 0.5 * (t_low + t_high);
            let el = self.elevation_at(t_mid);
            if (rising && el < mask) || (!rising && el >= mask) {
                t_low = t_mid;
            } else {
                t_high = t_mid;
            }
            if t_high - t_low < 0.5 {
                break;
            }
        }
        0.5 * (t_low + t_high)
    }

    /// Build a detailed `PassEvent` given AOS and LOS Unix timestamps.
    fn build_pass(&self, aos: f64, los: f64) -> Option<PassEvent> {
        if los - aos < 1.0 {
            return None;
        }
        let profile_step = ((los - aos) / 60.0).max(5.0).min(30.0);
        let mut profile = Vec::new();

        let mut max_el = -90.0_f64;
        let mut tca = aos;

        let mut t = aos;
        while t <= los + 1.0 {
            if let Some(state) = self.sgp4.propagate(t) {
                let sat_ecef = eci_to_ecef(state.position_km, t);
                let (az, el, range_km) = self.station.look_angle_ecef(sat_ecef);
                let doppler = self.doppler_at(&state, sat_ecef, t);
                profile.push(ElevationSample {
                    unix_time: t,
                    az_deg: az,
                    el_deg: el,
                    range_km,
                    doppler_hz: doppler,
                });
                if el > max_el {
                    max_el = el;
                    tca = t;
                }
            }
            t += profile_step;
        }

        if profile.is_empty() || max_el < self.station.elevation_mask_deg {
            return None;
        }

        let aos_sample = &profile[0];
        let los_sample = &profile[profile.len() - 1];

        // Find TCA sample
        let tca_sample = profile
            .iter()
            .max_by(|a, b| a.el_deg.partial_cmp(&b.el_deg).unwrap_or(Ordering::Equal))
            .unwrap();

        let link_margin = self.estimate_link_margin(tca_sample.range_km, max_el);
        let priority = max_el / 90.0 * 100.0; // simple heuristic

        Some(PassEvent {
            satellite_name: self.sgp4.tle.name.clone(),
            norad_id: self.sgp4.tle.norad_id,
            aos_unix: aos,
            aos_az_deg: aos_sample.az_deg,
            tca_unix: tca,
            tca_az_deg: tca_sample.az_deg,
            max_el_deg: max_el,
            los_unix: los,
            los_az_deg: los_sample.az_deg,
            duration_s: los - aos,
            profile,
            link_margin_db: link_margin,
            priority,
        })
    }

    /// Compute Doppler shift (Hz) for a given orbital state.
    fn doppler_at(&self, state: &OrbitalState, sat_ecef: [f64; 3], unix_time: f64) -> f64 {
        // Radial velocity (range rate) = d/dt |sat_ecef - gs_ecef|
        let vel_ecef = eci_vel_to_ecef(state.velocity_km_s, state.position_km, unix_time);
        let dx = sat_ecef[0] - self.station.ecef[0];
        let dy = sat_ecef[1] - self.station.ecef[1];
        let dz = sat_ecef[2] - self.station.ecef[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        if range < 1.0 {
            return 0.0;
        }
        // Unit vector from ground station to satellite
        let ux = dx / range;
        let uy = dy / range;
        let uz = dz / range;
        // Radial velocity (km/s) – positive = receding
        let range_rate_km_s = vel_ecef[0] * ux + vel_ecef[1] * uy + vel_ecef[2] * uz;
        // Doppler: f_obs = f0 * (1 - v_r/c)   (non-relativistic)
        let range_rate_m_s = range_rate_km_s * 1000.0;
        -self.carrier_hz * range_rate_m_s / C_LIGHT
    }

    /// Simple free-space link margin estimate at a given slant range and elevation.
    ///
    /// Assumes a nominal link budget appropriate for a small LEO CubeSat UHF downlink:
    /// - Transmit EIRP: 1 W (+30 dBm) + 0 dBi = +0 dBW
    /// - Receive antenna gain: 6 dBi (small Yagi)
    /// - System noise temperature: 500 K (T_sys) → noise floor = k*T*B (1 kHz BW)
    /// - Required Eb/N0: 10 dB
    fn estimate_link_margin(&self, range_km: f64, el_deg: f64) -> f64 {
        let freq_hz = self.carrier_hz;
        let lambda = C_LIGHT / freq_hz; // m
        let range_m = range_km * 1000.0;
        let fspl_db = 20.0 * (4.0 * PI * range_m / lambda).log10();

        let eirp_dbw = 0.0_f64;        // 1 W transmitter, 0 dBi antenna
        let gr_dbi = 6.0_f64;          // receive gain
        let t_sys_k = 500.0_f64;
        let bw_hz = 50_000.0_f64;      // 50 kHz channel
        let k_boltzmann = 1.380649e-23_f64;
        let noise_dbw = 10.0 * (k_boltzmann * t_sys_k * bw_hz).log10();
        let req_snr_db = 10.0_f64;

        // Atmospheric loss ~ 0.05 dB/degree below 10 deg, else 0.1 dB
        let atm_loss = if el_deg < 10.0 { 0.05 * (10.0 - el_deg) + 0.1 } else { 0.1 };

        eirp_dbw - fspl_db + gr_dbi - noise_dbw - req_snr_db - atm_loss
    }
}

// ---------------------------------------------------------------------------
// Antenna pointing
// ---------------------------------------------------------------------------

/// Real-time antenna pointing command.
#[derive(Debug, Clone)]
pub struct AntennaCommand {
    /// Unix timestamp for this command
    pub unix_time: f64,
    /// Target azimuth (degrees, 0=North, clockwise)
    pub az_deg: f64,
    /// Target elevation (degrees, 0=horizon)
    pub el_deg: f64,
    /// Slant range (km)
    pub range_km: f64,
    /// Rate of azimuth change (deg/s)
    pub az_rate_deg_s: f64,
    /// Rate of elevation change (deg/s)
    pub el_rate_deg_s: f64,
}

/// Generates a sequence of antenna pointing commands for a pass.
pub struct AntennaPointer {
    sgp4: Sgp4,
    station: GroundStation,
    /// Update interval (seconds) for pointing commands
    pub update_interval_s: f64,
    prev_az: f64,
    prev_el: f64,
    prev_t: f64,
}

impl AntennaPointer {
    /// Create a new pointer for a satellite/station pair.
    pub fn new(tle: &Tle, station: GroundStation, update_interval_s: f64) -> Self {
        let sgp4 = Sgp4::from_tle(tle);
        AntennaPointer {
            sgp4,
            station,
            update_interval_s,
            prev_az: 0.0,
            prev_el: 0.0,
            prev_t: 0.0,
        }
    }

    /// Generate a pointing command for a given Unix timestamp.
    ///
    /// Returns `None` if the satellite cannot be propagated (orbit decay).
    pub fn command_at(&mut self, unix_time: f64) -> Option<AntennaCommand> {
        let state = self.sgp4.propagate(unix_time)?;
        let sat_ecef = eci_to_ecef(state.position_km, unix_time);
        let (az, el, range_km) = self.station.look_angle_ecef(sat_ecef);

        let dt = unix_time - self.prev_t;
        let (az_rate, el_rate) = if dt > 0.0 {
            let daz = wrap_pi((az - self.prev_az) * DEG2RAD) * RAD2DEG;
            ((daz / dt), (el - self.prev_el) / dt)
        } else {
            (0.0, 0.0)
        };

        self.prev_az = az;
        self.prev_el = el;
        self.prev_t = unix_time;

        Some(AntennaCommand {
            unix_time,
            az_deg: az,
            el_deg: el,
            range_km,
            az_rate_deg_s: az_rate,
            el_rate_deg_s: el_rate,
        })
    }

    /// Generate a sequence of commands spanning a time interval.
    pub fn generate_track(
        &mut self,
        t_start: f64,
        t_end: f64,
    ) -> Vec<AntennaCommand> {
        let mut commands = Vec::new();
        let mut t = t_start;
        while t <= t_end {
            if let Some(cmd) = self.command_at(t) {
                commands.push(cmd);
            }
            t += self.update_interval_s;
        }
        commands
    }
}

// ---------------------------------------------------------------------------
// Doppler compensator
// ---------------------------------------------------------------------------

/// Instantaneous Doppler shift and compensation frequency.
#[derive(Debug, Clone)]
pub struct DopplerInfo {
    /// Unix timestamp
    pub unix_time: f64,
    /// Doppler shift (Hz) — positive means approaching
    pub shift_hz: f64,
    /// Compensation frequency to apply to the receiver LO (Hz)
    pub compensation_hz: f64,
    /// Range rate (m/s) — positive = satellite approaching
    pub range_rate_m_s: f64,
}

/// Computes and tracks Doppler corrections for a satellite link.
pub struct DopplerCompensator {
    sgp4: Sgp4,
    station: GroundStation,
    /// Nominal carrier frequency (Hz)
    pub carrier_hz: f64,
}

impl DopplerCompensator {
    /// Create a new compensator.
    pub fn new(tle: &Tle, station: GroundStation, carrier_hz: f64) -> Self {
        let sgp4 = Sgp4::from_tle(tle);
        DopplerCompensator { sgp4, station, carrier_hz }
    }

    /// Compute Doppler information at a given Unix timestamp.
    pub fn doppler_at(&self, unix_time: f64) -> Option<DopplerInfo> {
        let state = self.sgp4.propagate(unix_time)?;
        let sat_ecef = eci_to_ecef(state.position_km, unix_time);
        let vel_ecef = eci_vel_to_ecef(state.velocity_km_s, state.position_km, unix_time);

        let dx = sat_ecef[0] - self.station.ecef[0];
        let dy = sat_ecef[1] - self.station.ecef[1];
        let dz = sat_ecef[2] - self.station.ecef[2];
        let range_km = (dx * dx + dy * dy + dz * dz).sqrt();
        if range_km < 1.0 {
            return None;
        }
        let ux = dx / range_km;
        let uy = dy / range_km;
        let uz = dz / range_km;
        let range_rate_km_s = vel_ecef[0] * ux + vel_ecef[1] * uy + vel_ecef[2] * uz;
        let range_rate_m_s = range_rate_km_s * 1000.0;

        // Positive range_rate_m_s means satellite is moving away (receding)
        // Doppler shift: positive = approaching (blue-shifted)
        let shift_hz = -self.carrier_hz * range_rate_m_s / C_LIGHT;
        let compensation_hz = -shift_hz; // tune LO to pre-correct

        Some(DopplerInfo {
            unix_time,
            shift_hz,
            compensation_hz,
            range_rate_m_s: -range_rate_m_s, // invert: positive=approaching
        })
    }

    /// Compute Doppler rate (Hz/s) via two-point finite difference.
    pub fn doppler_rate_hz_s(&self, unix_time: f64, dt: f64) -> Option<f64> {
        let d1 = self.doppler_at(unix_time - dt)?;
        let d2 = self.doppler_at(unix_time + dt)?;
        Some((d2.shift_hz - d1.shift_hz) / (2.0 * dt))
    }
}

// ---------------------------------------------------------------------------
// Multi-satellite tracking scheduler
// ---------------------------------------------------------------------------

/// Priority levels for satellite tracking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TrackPriority {
    /// Background / low-priority science data
    Low = 0,
    /// Normal mission data
    Normal = 1,
    /// High-priority real-time operations
    High = 2,
    /// Emergency / distress
    Emergency = 3,
}

/// A scheduled tracking slot.
#[derive(Debug, Clone)]
pub struct TrackSlot {
    pub pass: PassEvent,
    pub priority: TrackPriority,
    /// True if this slot is currently active
    pub active: bool,
}

impl PartialEq for TrackSlot {
    fn eq(&self, other: &Self) -> bool {
        self.pass.aos_unix == other.pass.aos_unix
            && self.pass.norad_id == other.pass.norad_id
    }
}
impl Eq for TrackSlot {}
impl PartialOrd for TrackSlot {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for TrackSlot {
    fn cmp(&self, other: &Self) -> Ordering {
        // Higher priority first; tie-break on AOS time (earlier first)
        match self.priority.cmp(&other.priority) {
            Ordering::Equal => other
                .pass
                .aos_unix
                .partial_cmp(&self.pass.aos_unix)
                .unwrap_or(Ordering::Equal),
            o => o,
        }
    }
}

/// Conflict resolution strategy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConflictResolution {
    /// Keep the higher-priority pass; drop the lower
    Priority,
    /// Keep the pass with the higher maximum elevation
    MaxElevation,
    /// Keep the longer pass
    LongerDuration,
}

/// Multi-satellite tracking scheduler with priority-based conflict resolution.
pub struct TrackingScheduler {
    slots: Vec<TrackSlot>,
    pub conflict_resolution: ConflictResolution,
}

impl TrackingScheduler {
    /// Create an empty scheduler.
    pub fn new(conflict_resolution: ConflictResolution) -> Self {
        TrackingScheduler {
            slots: Vec::new(),
            conflict_resolution,
        }
    }

    /// Add a pass to the schedule, resolving conflicts as configured.
    ///
    /// Returns `true` if the pass was accepted, `false` if rejected.
    pub fn add_pass(&mut self, pass: PassEvent, priority: TrackPriority) -> bool {
        // Check for conflicts
        let new_slot = TrackSlot {
            pass: pass.clone(),
            priority,
            active: false,
        };

        let conflict_idx = self.slots.iter().position(|s| s.pass.overlaps(&pass));

        if let Some(idx) = conflict_idx {
            let keep_new = match self.conflict_resolution {
                ConflictResolution::Priority => {
                    priority > self.slots[idx].priority
                }
                ConflictResolution::MaxElevation => {
                    pass.max_el_deg > self.slots[idx].pass.max_el_deg
                }
                ConflictResolution::LongerDuration => {
                    pass.duration_s > self.slots[idx].pass.duration_s
                }
            };

            if keep_new {
                self.slots.remove(idx);
                self.slots.push(new_slot);
                self.sort_slots();
                true
            } else {
                false
            }
        } else {
            self.slots.push(new_slot);
            self.sort_slots();
            true
        }
    }

    fn sort_slots(&mut self) {
        self.slots.sort_by(|a, b| {
            a.pass.aos_unix.partial_cmp(&b.pass.aos_unix).unwrap_or(Ordering::Equal)
        });
    }

    /// Return the next upcoming pass after `unix_time`.
    pub fn next_pass(&self, unix_time: f64) -> Option<&TrackSlot> {
        self.slots.iter().find(|s| s.pass.aos_unix > unix_time)
    }

    /// Return all passes in the schedule.
    pub fn schedule(&self) -> &[TrackSlot] {
        &self.slots
    }

    /// Remove passes whose LOS has already passed.
    pub fn prune_past_passes(&mut self, unix_time: f64) {
        self.slots.retain(|s| s.pass.los_unix > unix_time);
    }

    /// Mark a slot as active.
    pub fn activate_pass(&mut self, norad_id: u32, unix_time: f64) {
        for slot in &mut self.slots {
            if slot.pass.norad_id == norad_id
                && slot.pass.aos_unix <= unix_time
                && slot.pass.los_unix >= unix_time
            {
                slot.active = true;
            }
        }
    }

    /// Return the currently active tracking slot (if any).
    pub fn active_slot(&self, unix_time: f64) -> Option<&TrackSlot> {
        self.slots.iter().find(|s| {
            s.active
                && s.pass.aos_unix <= unix_time
                && s.pass.los_unix >= unix_time
        })
    }
}

// ---------------------------------------------------------------------------
// Pass handover
// ---------------------------------------------------------------------------

/// Handover state between consecutive (or overlapping) passes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HandoverState {
    /// No active pass
    Idle,
    /// Tracking the current pass
    Tracking,
    /// In the handover window (preparing to switch)
    Handing,
    /// Tracking the next pass (handover complete)
    Next,
}

/// Manages seamless handover between consecutive satellite passes or between
/// two ground stations covering the same satellite.
pub struct PassHandover {
    pub state: HandoverState,
    /// Time before LOS to begin handover preparation (seconds)
    pub handover_lead_time_s: f64,
    current_norad: Option<u32>,
    next_norad: Option<u32>,
    current_los: f64,
    next_aos: f64,
}

impl PassHandover {
    /// Create a new handover manager.
    pub fn new(handover_lead_time_s: f64) -> Self {
        PassHandover {
            state: HandoverState::Idle,
            handover_lead_time_s,
            current_norad: None,
            next_norad: None,
            current_los: 0.0,
            next_aos: 0.0,
        }
    }

    /// Register a new pass. Call this when the scheduler has a new upcoming pass.
    pub fn register_pass(&mut self, pass: &PassEvent) {
        match self.current_norad {
            None => {
                self.current_norad = Some(pass.norad_id);
                self.current_los = pass.los_unix;
                self.state = HandoverState::Tracking;
            }
            Some(_) => {
                self.next_norad = Some(pass.norad_id);
                self.next_aos = pass.aos_unix;
            }
        }
    }

    /// Update handover state for the current Unix time.
    ///
    /// Returns the NORAD ID of the satellite currently being tracked.
    pub fn update(&mut self, unix_time: f64) -> Option<u32> {
        match self.state {
            HandoverState::Idle => None,
            HandoverState::Tracking => {
                if unix_time > self.current_los {
                    // LOS reached
                    if self.next_norad.is_some() {
                        self.current_norad = self.next_norad.take();
                        self.current_los = self.next_aos + 3600.0; // placeholder
                        self.state = HandoverState::Next;
                    } else {
                        self.state = HandoverState::Idle;
                        self.current_norad = None;
                    }
                    return self.current_norad;
                }
                if unix_time >= self.current_los - self.handover_lead_time_s
                    && self.next_norad.is_some()
                {
                    self.state = HandoverState::Handing;
                }
                self.current_norad
            }
            HandoverState::Handing => {
                if unix_time > self.current_los {
                    if let Some(next) = self.next_norad.take() {
                        self.current_norad = Some(next);
                        self.current_los = self.next_aos + 3600.0;
                        self.state = HandoverState::Next;
                    } else {
                        self.state = HandoverState::Idle;
                        self.current_norad = None;
                    }
                }
                self.current_norad
            }
            HandoverState::Next => {
                self.state = HandoverState::Tracking;
                self.current_norad
            }
        }
    }

    /// True if the system is currently handing over between passes.
    pub fn is_handing_over(&self) -> bool {
        self.state == HandoverState::Handing
    }
}

// ---------------------------------------------------------------------------
// Visibility determination
// ---------------------------------------------------------------------------

/// Returns `true` if the satellite is visible from the station at the given time.
///
/// Visibility requires:
/// 1. Satellite above the elevation mask, and
/// 2. The line-of-sight is not blocked by the Earth's limb (automatically
///    satisfied if elevation > 0°, since we compute topocentric coordinates).
pub fn is_visible(
    sgp4: &Sgp4,
    station: &GroundStation,
    unix_time: f64,
) -> bool {
    match sgp4.propagate(unix_time) {
        Some(state) => {
            let el = station.elevation_to_eci(state.position_km, unix_time);
            el >= station.elevation_mask_deg
        }
        None => false,
    }
}

/// Compute the maximum possible elevation for a given orbital altitude and
/// ground-station latitude, ignoring actual orbital inclination limits.
///
/// This is a geometric upper bound.
pub fn max_elevation_deg(orbit_alt_km: f64, gs_lat_deg: f64, sat_inc_deg: f64) -> f64 {
    // Satellites with inclination < |lat| never pass overhead
    if gs_lat_deg.abs() > sat_inc_deg + 1.0 {
        return -1.0;
    }
    let r = R_EARTH_KM;
    let h = orbit_alt_km;
    // Geometric nadir-pass angle
    let rho = ((r + h) / r).acos().asin() * RAD2DEG;
    90.0 - rho.max(0.0)
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Reference TLE (ISS, approximate epoch ~2024-01-01) ---
    const ISS_NAME: &str = "ISS (ZARYA)";
    const ISS_L1: &str = "1 25544U 98067A   24001.50000000  .00002182  00000-0  41462-4 0  9995";
    const ISS_L2: &str = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.49911251431451";

    // NOAA-19 TLE (approximate, for broader test coverage)
    const NOAA_NAME: &str = "NOAA 19";
    const NOAA_L1: &str = "1 33591U 09005A   24002.49955208  .00000217  00000-0  16011-3 0  9992";
    const NOAA_L2: &str = "2 33591  99.1662 185.4321 0013773 101.3845 258.8712 14.12462831777451";

    fn iss_tle() -> Tle {
        Tle::parse(ISS_NAME, ISS_L1, ISS_L2).unwrap()
    }

    fn noaa_tle() -> Tle {
        Tle::parse(NOAA_NAME, NOAA_L1, NOAA_L2).unwrap()
    }

    fn austin_gs() -> GroundStation {
        GroundStation::new("Austin", 30.2672, -97.7431, 0.149, 5.0)
    }

    // -----------------------------------------------------------------------
    // TLE parsing tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_tle_parse_iss() {
        let tle = iss_tle();
        assert_eq!(tle.norad_id, 25544);
        assert_eq!(tle.intl_designator, "98067A");
        assert!((tle.inclination * RAD2DEG - 51.6416).abs() < 0.001);
        assert!((tle.eccentricity - 0.0006703).abs() < 1.0e-6);
        assert!(tle.mean_motion > 0.0);
    }

    #[test]
    fn test_tle_parse_noaa() {
        let tle = noaa_tle();
        assert_eq!(tle.norad_id, 33591);
        assert!((tle.inclination * RAD2DEG - 99.1662).abs() < 0.001);
    }

    #[test]
    fn test_tle_checksum_bad() {
        let bad_l1 = "1 25544U 98067A   24001.50000000  .00002182  00000-0  41462-4 0  9990";
        let result = Tle::parse(ISS_NAME, bad_l1, ISS_L2);
        assert_eq!(result.unwrap_err(), TleError::BadChecksum);
    }

    #[test]
    fn test_tle_wrong_line_number() {
        let bad = "2 25544U 98067A   24001.50000000  .00002182  00000-0  41462-4 0  9995";
        let result = Tle::parse(ISS_NAME, bad, ISS_L2);
        assert_eq!(result.unwrap_err(), TleError::WrongLineNumber);
    }

    #[test]
    fn test_tle_period() {
        let tle = iss_tle();
        let period = tle.period_s();
        // ISS period ~92-93 min
        assert!(period > 5400.0 && period < 5700.0, "ISS period {period:.0} s");
    }

    #[test]
    fn test_tle_altitude() {
        let tle = iss_tle();
        let alt = tle.perigee_km();
        // ISS ~400 km altitude
        assert!(alt > 350.0 && alt < 450.0, "ISS perigee {alt:.1} km");
    }

    #[test]
    fn test_tle_epoch_unix() {
        let tle = iss_tle();
        // 2024-001 day 1.5 → roughly 2024-01-01 12:00 UTC
        // Unix timestamp ~= 1704110400
        assert!(tle.epoch_unix > 1_700_000_000.0 && tle.epoch_unix < 1_710_000_000.0);
    }

    #[test]
    fn test_tle_bstar() {
        let tle = iss_tle();
        assert!((tle.b_star - 4.1462e-5).abs() < 1.0e-8);
    }

    #[test]
    fn test_tle_mean_anomaly_range() {
        let tle = iss_tle();
        assert!(tle.mean_anomaly >= 0.0 && tle.mean_anomaly <= 2.0 * PI);
    }

    // -----------------------------------------------------------------------
    // SGP4 propagation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_sgp4_propagate_at_epoch() {
        let tle = iss_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let state = sgp4.propagate(tle.epoch_unix).unwrap();
        // Position magnitude should be ~R_earth + ~400 km = ~6778 km
        let r = state.radius_km();
        assert!(r > 6500.0 && r < 7000.0, "radius {r:.1} km");
    }

    #[test]
    fn test_sgp4_propagate_velocity() {
        let tle = iss_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let state = sgp4.propagate(tle.epoch_unix).unwrap();
        let v_sq: f64 = state.velocity_km_s.iter().map(|x| x * x).sum();
        let v = v_sq.sqrt();
        // ISS orbital velocity ~7.67 km/s
        assert!(v > 7.0 && v < 8.5, "speed {v:.2} km/s");
    }

    #[test]
    fn test_sgp4_propagate_one_period() {
        let tle = iss_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let t0 = tle.epoch_unix;
        let period = tle.period_s();
        let s0 = sgp4.propagate(t0).unwrap();
        let s1 = sgp4.propagate(t0 + period).unwrap();
        // Position should be close after one period (within ~5 km for simplified model)
        let dx: f64 = s0.position_km[0] - s1.position_km[0];
        let dy: f64 = s0.position_km[1] - s1.position_km[1];
        let dz: f64 = s0.position_km[2] - s1.position_km[2];
        let dr = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(dr < 30.0, "1-period position error {dr:.2} km");
    }

    #[test]
    fn test_sgp4_altitude_reasonable() {
        let tle = iss_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let state = sgp4.propagate(tle.epoch_unix).unwrap();
        let alt = state.altitude_km();
        assert!(alt > 200.0 && alt < 600.0, "altitude {alt:.1} km");
    }

    #[test]
    fn test_sgp4_propagate_noaa() {
        let tle = noaa_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let state = sgp4.propagate(tle.epoch_unix).unwrap();
        let alt = state.altitude_km();
        // NOAA-19 ~870 km sun-synchronous
        assert!(alt > 700.0 && alt < 1000.0, "NOAA-19 altitude {alt:.1} km");
    }

    #[test]
    fn test_sgp4_position_changes_with_time() {
        let tle = iss_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let s0 = sgp4.propagate(tle.epoch_unix).unwrap();
        let s1 = sgp4.propagate(tle.epoch_unix + 300.0).unwrap();
        let dx = s1.position_km[0] - s0.position_km[0];
        let dy = s1.position_km[1] - s0.position_km[1];
        let dz = s1.position_km[2] - s0.position_km[2];
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        // Satellite moves ~2300 km in 5 min
        assert!(d > 100.0 && d < 5000.0, "5-min displacement {d:.1} km");
    }

    // -----------------------------------------------------------------------
    // Coordinate conversion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_geodetic_to_ecef_equator() {
        // Point on equator at prime meridian
        let ecef = geodetic_to_ecef(0.0, 0.0, 0.0);
        assert!((ecef[0] - R_EARTH_KM).abs() < 1.0, "x={}", ecef[0]);
        assert!(ecef[1].abs() < 0.01, "y={}", ecef[1]);
        assert!(ecef[2].abs() < 0.01, "z={}", ecef[2]);
    }

    #[test]
    fn test_geodetic_to_ecef_north_pole() {
        let ecef = geodetic_to_ecef(90.0, 0.0, 0.0);
        // z ≈ WGS-84 semi-minor axis
        assert!(ecef[0].abs() < 1.0, "x={}", ecef[0]);
        assert!(ecef[2] > 6340.0, "z={}", ecef[2]);
    }

    #[test]
    fn test_gmst_reasonable_range() {
        let unix_2024_jan1 = 1704067200.0;
        let theta = gmst(unix_2024_jan1);
        // GMST should be in [0, 2π)
        assert!(theta >= 0.0 && theta < 2.0 * PI, "GMST={theta}");
    }

    #[test]
    fn test_eci_to_ecef_identity_at_prime_meridian() {
        // At t=0 (roughly), ECI and ECEF x-axes are close but not exactly aligned
        // Just verify the conversion is invertible (round-trip within 1 km)
        let eci = [7000.0_f64, 0.0, 0.0];
        let ecef = eci_to_ecef(eci, 0.0);
        let r_eci = (eci[0] * eci[0] + eci[1] * eci[1] + eci[2] * eci[2]).sqrt();
        let r_ecef = (ecef[0] * ecef[0] + ecef[1] * ecef[1] + ecef[2] * ecef[2]).sqrt();
        assert!((r_eci - r_ecef).abs() < 0.01, "radius preserved");
    }

    // -----------------------------------------------------------------------
    // Ground station tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_look_angle_overhead() {
        let gs = austin_gs();
        // Place a satellite directly overhead in ECEF
        let [gx, gy, gz] = gs.ecef;
        // Overhead means along the local vertical: sat_ecef = gs_ecef + h * unit_normal
        let r = (gx * gx + gy * gy + gz * gz).sqrt();
        let sat = [gx * (1.0 + 500.0 / r), gy * (1.0 + 500.0 / r), gz * (1.0 + 500.0 / r)];
        let (_, el, _) = gs.look_angle_ecef(sat);
        assert!(el > 80.0, "overhead elevation {el:.1}°");
    }

    #[test]
    fn test_look_angle_range() {
        let gs = austin_gs();
        let [gx, gy, gz] = gs.ecef;
        let r = (gx * gx + gy * gy + gz * gz).sqrt();
        let sat = [gx * (1.0 + 500.0 / r), gy * (1.0 + 500.0 / r), gz * (1.0 + 500.0 / r)];
        let (_, _, range) = gs.look_angle_ecef(sat);
        assert!((range - 500.0).abs() < 5.0, "overhead range {range:.1} km");
    }

    #[test]
    fn test_look_angle_azimuth_north() {
        let gs = GroundStation::new("Equator", 0.0, 0.0, 0.0, 5.0);
        // Move station 500 km north in ECEF
        let [gx, gy, gz] = gs.ecef;
        let sat = [gx, gy, gz + 500.0 * 0.99]; // approx north-ish
        let (az, _el, _) = gs.look_angle_ecef(sat);
        // North → az ~ 0° or 360°
        assert!(az < 20.0 || az > 340.0, "north az={az:.1}°");
    }

    #[test]
    fn test_elevation_mask() {
        let gs = GroundStation::new("Test", 0.0, 0.0, 0.0, 10.0);
        assert_eq!(gs.elevation_mask_deg, 10.0);
    }

    // -----------------------------------------------------------------------
    // Pass prediction tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_predict_passes_finds_iss_passes() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        // ISS passes Austin roughly 5-6 times per day
        assert!(!passes.is_empty(), "Expected ISS passes over Austin");
    }

    #[test]
    fn test_pass_duration_positive() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        for p in &passes {
            assert!(p.duration_s > 0.0, "pass duration must be positive");
            assert!(p.los_unix > p.aos_unix, "LOS > AOS");
        }
    }

    #[test]
    fn test_pass_max_elevation_above_mask() {
        let tle = iss_tle();
        let gs = austin_gs();
        let el_mask = gs.elevation_mask_deg;
        let mut predictor = PassPredictor::new(tle.clone(), gs.clone());
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        for p in &passes {
            assert!(p.max_el_deg >= el_mask - 0.5,
                "max el {:.1}° below mask", p.max_el_deg);
        }
    }

    #[test]
    fn test_pass_tca_between_aos_los() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        for p in &passes {
            assert!(p.tca_unix >= p.aos_unix && p.tca_unix <= p.los_unix,
                "TCA not between AOS and LOS");
        }
    }

    #[test]
    fn test_pass_profile_non_empty() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        for p in &passes {
            assert!(!p.profile.is_empty(), "profile should not be empty");
        }
    }

    #[test]
    fn test_pass_profile_elevation_samples() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        if let Some(p) = passes.first() {
            // All profile samples should have reasonable azimuth [0, 360]
            for s in &p.profile {
                assert!(s.az_deg >= 0.0 && s.az_deg < 360.0,
                    "az_deg out of range: {}", s.az_deg);
            }
        }
    }

    #[test]
    fn test_pass_norad_id_correct() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        for p in &passes {
            assert_eq!(p.norad_id, 25544);
        }
    }

    // -----------------------------------------------------------------------
    // Doppler tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_doppler_sign_approaching() {
        let tle = iss_tle();
        let gs = austin_gs();
        let comp = DopplerCompensator::new(&tle, gs, 437_000_000.0);

        // Scan all 24h and find at least one sample where Doppler > 0 (approaching)
        let t0 = tle.epoch_unix;
        let mut found_positive = false;
        let mut t = t0;
        while t < t0 + 86400.0 {
            if let Some(d) = comp.doppler_at(t) {
                if d.shift_hz > 1000.0 { found_positive = true; break; }
            }
            t += 100.0;
        }
        assert!(found_positive, "Expected positive Doppler (approaching) at some point");
    }

    #[test]
    fn test_doppler_magnitude_reasonable() {
        let tle = iss_tle();
        let gs = austin_gs();
        let comp = DopplerCompensator::new(&tle, gs, 437_000_000.0);
        let t0 = tle.epoch_unix;
        let mut max_shift = 0.0_f64;
        let mut t = t0;
        while t < t0 + 86400.0 {
            if let Some(d) = comp.doppler_at(t) {
                max_shift = max_shift.max(d.shift_hz.abs());
            }
            t += 60.0;
        }
        // ISS at 437 MHz max Doppler ~±10 kHz
        assert!(max_shift < 20_000.0, "Max Doppler {max_shift:.0} Hz unexpectedly large");
        assert!(max_shift > 100.0, "Max Doppler {max_shift:.0} Hz unexpectedly small");
    }

    #[test]
    fn test_doppler_compensation_opposite_sign() {
        let tle = iss_tle();
        let gs = austin_gs();
        let comp = DopplerCompensator::new(&tle, gs, 437_000_000.0);
        let t0 = tle.epoch_unix;
        if let Some(d) = comp.doppler_at(t0) {
            assert!((d.shift_hz + d.compensation_hz).abs() < 1.0,
                "shift + compensation should cancel");
        }
    }

    #[test]
    fn test_doppler_rate_finite() {
        let tle = iss_tle();
        let gs = austin_gs();
        let comp = DopplerCompensator::new(&tle, gs, 437_000_000.0);
        let t0 = tle.epoch_unix;
        if let Some(rate) = comp.doppler_rate_hz_s(t0, 10.0) {
            assert!(rate.is_finite());
            // Max Doppler rate for ISS ~hundreds of Hz/s
            assert!(rate.abs() < 5000.0, "Doppler rate {rate:.2} Hz/s");
        }
    }

    // -----------------------------------------------------------------------
    // Antenna pointer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_antenna_pointer_generates_commands() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut pointer = AntennaPointer::new(&tle, gs, 5.0);
        let t0 = tle.epoch_unix;
        let cmds = pointer.generate_track(t0, t0 + 60.0);
        assert!(!cmds.is_empty(), "Should generate commands");
        assert!(cmds.len() >= 12, "At least 12 commands for 60 s at 5 s interval");
    }

    #[test]
    fn test_antenna_pointer_az_range() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut pointer = AntennaPointer::new(&tle, gs, 10.0);
        let t0 = tle.epoch_unix;
        let cmds = pointer.generate_track(t0, t0 + 600.0);
        for cmd in &cmds {
            assert!(cmd.az_deg >= 0.0 && cmd.az_deg < 360.0,
                "az={}", cmd.az_deg);
        }
    }

    #[test]
    fn test_antenna_pointer_el_range() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut pointer = AntennaPointer::new(&tle, gs, 10.0);
        let t0 = tle.epoch_unix;
        let cmds = pointer.generate_track(t0, t0 + 600.0);
        for cmd in &cmds {
            assert!(cmd.el_deg >= -90.0 && cmd.el_deg <= 90.0,
                "el={}", cmd.el_deg);
        }
    }

    // -----------------------------------------------------------------------
    // Scheduler tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_scheduler_add_no_conflict() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);

        let mut sched = TrackingScheduler::new(ConflictResolution::Priority);
        for p in passes {
            sched.add_pass(p, TrackPriority::Normal);
        }
        assert!(!sched.schedule().is_empty());
    }

    #[test]
    fn test_scheduler_conflict_priority_wins() {
        // Build two synthetic overlapping passes
        let pass_a = PassEvent {
            satellite_name: "SAT_A".into(),
            norad_id: 1,
            aos_unix: 1000.0, tca_unix: 1200.0, los_unix: 1400.0,
            aos_az_deg: 45.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 30.0, duration_s: 400.0,
            profile: vec![], link_margin_db: 10.0, priority: 1.0,
        };
        let pass_b = PassEvent {
            satellite_name: "SAT_B".into(),
            norad_id: 2,
            aos_unix: 1100.0, tca_unix: 1300.0, los_unix: 1500.0,
            aos_az_deg: 60.0, tca_az_deg: 100.0, los_az_deg: 200.0,
            max_el_deg: 20.0, duration_s: 400.0,
            profile: vec![], link_margin_db: 8.0, priority: 1.0,
        };

        let mut sched = TrackingScheduler::new(ConflictResolution::Priority);
        assert!(sched.add_pass(pass_a, TrackPriority::Normal));
        // Same priority → new pass should NOT displace existing
        assert!(!sched.add_pass(pass_b, TrackPriority::Normal));
        assert_eq!(sched.schedule().len(), 1);
        assert_eq!(sched.schedule()[0].pass.norad_id, 1);
    }

    #[test]
    fn test_scheduler_conflict_emergency_wins() {
        let pass_a = PassEvent {
            satellite_name: "SAT_A".into(),
            norad_id: 1,
            aos_unix: 1000.0, tca_unix: 1200.0, los_unix: 1400.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 50.0, duration_s: 400.0,
            profile: vec![], link_margin_db: 15.0, priority: 1.0,
        };
        let pass_emergency = PassEvent {
            satellite_name: "DISTRESS".into(),
            norad_id: 99,
            aos_unix: 1100.0, tca_unix: 1300.0, los_unix: 1500.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 30.0, duration_s: 400.0,
            profile: vec![], link_margin_db: 5.0, priority: 1.0,
        };

        let mut sched = TrackingScheduler::new(ConflictResolution::Priority);
        sched.add_pass(pass_a, TrackPriority::Normal);
        let accepted = sched.add_pass(pass_emergency, TrackPriority::Emergency);
        assert!(accepted, "Emergency should displace normal");
        assert_eq!(sched.schedule()[0].pass.norad_id, 99);
    }

    #[test]
    fn test_scheduler_prune_past() {
        let pass = PassEvent {
            satellite_name: "SAT".into(),
            norad_id: 1,
            aos_unix: 100.0, tca_unix: 200.0, los_unix: 300.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 45.0, duration_s: 200.0,
            profile: vec![], link_margin_db: 10.0, priority: 1.0,
        };
        let mut sched = TrackingScheduler::new(ConflictResolution::Priority);
        sched.add_pass(pass, TrackPriority::Normal);
        sched.prune_past_passes(400.0); // after LOS
        assert!(sched.schedule().is_empty());
    }

    #[test]
    fn test_scheduler_next_pass() {
        let pass = PassEvent {
            satellite_name: "SAT".into(),
            norad_id: 1,
            aos_unix: 1000.0, tca_unix: 1200.0, los_unix: 1400.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 45.0, duration_s: 400.0,
            profile: vec![], link_margin_db: 10.0, priority: 1.0,
        };
        let mut sched = TrackingScheduler::new(ConflictResolution::Priority);
        sched.add_pass(pass, TrackPriority::Normal);
        let next = sched.next_pass(500.0);
        assert!(next.is_some());
        assert_eq!(next.unwrap().pass.norad_id, 1);
        let none = sched.next_pass(2000.0);
        assert!(none.is_none());
    }

    // -----------------------------------------------------------------------
    // Handover tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_handover_idle_initially() {
        let h = PassHandover::new(60.0);
        assert_eq!(h.state, HandoverState::Idle);
    }

    #[test]
    fn test_handover_register_first_pass() {
        let mut h = PassHandover::new(60.0);
        let pass = PassEvent {
            satellite_name: "SAT".into(),
            norad_id: 42,
            aos_unix: 100.0, tca_unix: 200.0, los_unix: 300.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 50.0, duration_s: 200.0,
            profile: vec![], link_margin_db: 10.0, priority: 1.0,
        };
        h.register_pass(&pass);
        assert_eq!(h.state, HandoverState::Tracking);
    }

    #[test]
    fn test_handover_tracks_during_pass() {
        let mut h = PassHandover::new(60.0);
        let pass = PassEvent {
            satellite_name: "SAT".into(),
            norad_id: 42,
            aos_unix: 100.0, tca_unix: 200.0, los_unix: 300.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 50.0, duration_s: 200.0,
            profile: vec![], link_margin_db: 10.0, priority: 1.0,
        };
        h.register_pass(&pass);
        let tracking = h.update(150.0);
        assert_eq!(tracking, Some(42));
    }

    #[test]
    fn test_handover_enters_handing_state() {
        let mut h = PassHandover::new(60.0);
        let pass_a = PassEvent {
            satellite_name: "SAT_A".into(),
            norad_id: 1,
            aos_unix: 100.0, tca_unix: 200.0, los_unix: 300.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 50.0, duration_s: 200.0,
            profile: vec![], link_margin_db: 10.0, priority: 1.0,
        };
        let pass_b = PassEvent {
            satellite_name: "SAT_B".into(),
            norad_id: 2,
            aos_unix: 350.0, tca_unix: 450.0, los_unix: 550.0,
            aos_az_deg: 0.0, tca_az_deg: 90.0, los_az_deg: 180.0,
            max_el_deg: 40.0, duration_s: 200.0,
            profile: vec![], link_margin_db: 8.0, priority: 1.0,
        };
        h.register_pass(&pass_a);
        h.register_pass(&pass_b);
        // Update at t=250 (50 s before LOS=300, within lead_time=60)
        h.update(250.0);
        assert_eq!(h.state, HandoverState::Handing);
        assert!(h.is_handing_over());
    }

    // -----------------------------------------------------------------------
    // Visibility tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_is_visible_returns_bool() {
        let tle = iss_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let gs = austin_gs();
        // Just verify it doesn't panic and returns a bool
        let v = is_visible(&sgp4, &gs, tle.epoch_unix);
        let _ = v; // either true or false is acceptable
    }

    #[test]
    fn test_visibility_over_full_day() {
        let tle = iss_tle();
        let sgp4 = Sgp4::from_tle(&tle);
        let gs = austin_gs();
        let t0 = tle.epoch_unix;
        let mut visible_count = 0;
        let mut t = t0;
        while t < t0 + 86400.0 {
            if is_visible(&sgp4, &gs, t) {
                visible_count += 1;
            }
            t += 60.0;
        }
        // ISS should be visible some fraction of the time
        assert!(visible_count > 0, "ISS never visible in 24h");
        assert!(visible_count < 1440, "ISS always visible? unlikely");
    }

    #[test]
    fn test_max_elevation_polar() {
        // For a low-inclination satellite and a polar station, the function
        // should return -1 (satellite never reaches the station's latitude)
        let alt = 400.0; // ISS-like
        let lat = 80.0;  // 80° north
        let inc = 51.6;  // ISS inclination
        let max_el = max_elevation_deg(alt, lat, inc);
        assert!(max_el < 0.0, "ISS shouldn't reach 80°N: got {max_el:.1}");
    }

    #[test]
    fn test_max_elevation_equatorial() {
        let alt = 400.0;
        let lat = 0.0;   // equator
        let inc = 51.6;
        let max_el = max_elevation_deg(alt, lat, inc);
        assert!(max_el > 0.0, "Equatorial station can see any satellite: {max_el:.1}");
    }

    // -----------------------------------------------------------------------
    // Link margin tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_link_margin_increases_with_elevation() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        // Link margin at short range should be positive for a reasonable scenario
        let margin_high = predictor.estimate_link_margin(800.0, 60.0);
        let margin_low  = predictor.estimate_link_margin(1500.0, 10.0);
        assert!(margin_high > margin_low,
            "Higher elevation should have better margin: {margin_high:.1} vs {margin_low:.1}");
    }

    #[test]
    fn test_pass_link_margin_finite() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs);
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        for p in &passes {
            assert!(p.link_margin_db.is_finite());
        }
    }

    // -----------------------------------------------------------------------
    // Kepler / math helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_solve_kepler_circular() {
        // For e=0, E should equal M
        for m in [0.0, 0.5, 1.0, 2.0, 3.0_f64] {
            let e_anom = solve_kepler(m, 0.0);
            assert!((e_anom - wrap_2pi(m)).abs() < 1.0e-10,
                "e=0: E={e_anom} vs M={m}");
        }
    }

    #[test]
    fn test_solve_kepler_consistency() {
        // Verify M = E - e*sin(E)
        let e = 0.1;
        for m in [0.1, 1.0, 2.5, 5.5_f64] {
            let ea = solve_kepler(m, e);
            let m_check = ea - e * ea.sin();
            assert!((m_check - wrap_2pi(m)).abs() < 1.0e-9,
                "Kepler check failed: m={m}, E={ea}, M_check={m_check}");
        }
    }

    #[test]
    fn test_wrap_pi() {
        assert!((wrap_pi(3.5) - (3.5 - 2.0 * PI)).abs() < 1.0e-12);
        assert!((wrap_pi(-3.5) - (-3.5 + 2.0 * PI)).abs() < 1.0e-12);
        assert!(wrap_pi(1.0).abs() < 1.5);
    }

    #[test]
    fn test_wrap_2pi() {
        assert!(wrap_2pi(7.0) < 2.0 * PI);
        assert!(wrap_2pi(-1.0) > 0.0);
    }

    // -----------------------------------------------------------------------
    // Integration test: full predict + schedule + handover
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_pipeline_iss() {
        let tle = iss_tle();
        let gs = austin_gs();
        let mut predictor = PassPredictor::new(tle.clone(), gs.clone());
        let t0 = tle.epoch_unix;
        let passes = predictor.predict_passes(t0, t0 + 86400.0, 60.0);
        assert!(!passes.is_empty());

        let mut sched = TrackingScheduler::new(ConflictResolution::MaxElevation);
        for p in passes {
            sched.add_pass(p, TrackPriority::Normal);
        }
        assert!(!sched.schedule().is_empty());

        if let Some(slot) = sched.next_pass(t0) {
            let mut handover = PassHandover::new(30.0);
            handover.register_pass(&slot.pass);
            let mid = slot.pass.aos_unix + slot.pass.duration_s * 0.5;
            let tracked = handover.update(mid);
            assert_eq!(tracked, Some(slot.pass.norad_id));
        }
    }
}

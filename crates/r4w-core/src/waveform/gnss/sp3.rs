//! SP3 Precise Ephemeris Parser
//!
//! Parses SP3 (Standard Product 3) format files containing precise satellite
//! positions and clock offsets. SP3 files are produced by analysis centers
//! like CODE, IGS, GFZ with centimeter-level accuracy.
//!
//! ## File Format
//!
//! SP3 is a text format with:
//! - Header lines (version, time system, satellite list, accuracy)
//! - Epoch records (marked with '*')
//! - Position records (marked with 'P' + satellite ID)
//!
//! ## Interpolation
//!
//! SP3 files typically have 15-minute intervals. For sub-interval positions,
//! we use 9-point Lagrange polynomial interpolation.
//!
//! ## Usage
//!
//! ```ignore
//! use r4w_core::waveform::gnss::sp3::Sp3Ephemeris;
//!
//! let sp3 = Sp3Ephemeris::from_file("COD0OPSFIN_20250150000_01D_05M_ORB.SP3")?;
//! let (pos, vel, clk) = sp3.interpolate("G01", gps_time_s)?;
//! ```

use crate::coordinates::{EcefPosition, EcefVelocity};
use crate::waveform::gnss::types::GnssConstellation;
use std::collections::HashMap;
use std::path::Path;

/// SP3 file header information
#[derive(Debug, Clone)]
pub struct Sp3Header {
    /// SP3 version (c, d)
    pub version: char,
    /// Position/Velocity flag (P = position only, V = position + velocity)
    pub pos_vel_flag: char,
    /// Start epoch (GPS seconds)
    pub start_time_gps_s: f64,
    /// Number of epochs in file
    pub num_epochs: u32,
    /// Data used descriptor
    pub data_used: String,
    /// Coordinate system (e.g., "IGS14", "ITRF")
    pub coord_system: String,
    /// Orbit type (FIT, EXT, etc.)
    pub orbit_type: String,
    /// Agency (CODE, IGS, GFZ, etc.)
    pub agency: String,
    /// GPS week
    pub gps_week: u32,
    /// Seconds of week
    pub gps_sow: f64,
    /// Epoch interval in seconds
    pub interval_s: f64,
    /// Satellite list
    pub satellites: Vec<String>,
    /// Satellite accuracy (2^n mm)
    pub accuracy: HashMap<String, f64>,
}

/// Single epoch position/clock record
#[derive(Debug, Clone)]
pub struct Sp3Record {
    /// GPS time in seconds
    pub gps_time_s: f64,
    /// ECEF X position in meters
    pub x_m: f64,
    /// ECEF Y position in meters
    pub y_m: f64,
    /// ECEF Z position in meters
    pub z_m: f64,
    /// Clock offset in seconds (can be None if bad/missing)
    pub clock_s: Option<f64>,
    /// Position predicted flag
    pub predicted: bool,
    /// Bad orbit flag
    pub bad_orbit: bool,
    /// Bad clock flag
    pub bad_clock: bool,
}

/// Parsed SP3 ephemeris data
#[derive(Debug)]
pub struct Sp3Ephemeris {
    /// File header
    pub header: Sp3Header,
    /// Position records by satellite ID (e.g., "G01", "E05", "R24")
    pub records: HashMap<String, Vec<Sp3Record>>,
}

impl Sp3Ephemeris {
    /// Load SP3 ephemeris from a file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let path = path.as_ref();
        if !path.exists() {
            return Err(format!("SP3 file not found: {}", path.display()));
        }

        let content = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read SP3 file: {}", e))?;

        Self::parse(&content)
    }

    /// Parse SP3 content from string
    pub fn parse(content: &str) -> Result<Self, String> {
        let lines: Vec<&str> = content.lines().collect();
        if lines.len() < 23 {
            return Err("SP3 file too short".to_string());
        }

        // Parse header (line 1)
        let line1 = lines[0];
        if !line1.starts_with('#') {
            return Err("Invalid SP3 file: missing header".to_string());
        }

        let version = line1.chars().nth(1).unwrap_or('c');
        let pos_vel_flag = line1.chars().nth(2).unwrap_or('P');

        // Parse start time from line 1
        let year: i32 = line1[3..7].trim().parse().unwrap_or(2025);
        let month: u32 = line1[8..10].trim().parse().unwrap_or(1);
        let day: u32 = line1[11..13].trim().parse().unwrap_or(1);
        let hour: u32 = line1[14..16].trim().parse().unwrap_or(0);
        let minute: u32 = line1[17..19].trim().parse().unwrap_or(0);
        let second: f64 = line1[20..31].trim().parse().unwrap_or(0.0);
        let num_epochs: u32 = line1[32..39].trim().parse().unwrap_or(0);
        let data_used = line1.get(40..45).unwrap_or("").trim().to_string();
        let coord_system = line1.get(46..51).unwrap_or("").trim().to_string();
        let orbit_type = line1.get(52..55).unwrap_or("").trim().to_string();
        let agency = line1.get(56..60).unwrap_or("").trim().to_string();

        let start_time_gps_s = gps_time_from_calendar(year, month, day, hour, minute, second);

        // Parse line 2 (GPS week, SOW, interval)
        let line2 = lines[1];
        let gps_week: u32 = line2.get(3..7).unwrap_or("0").trim().parse().unwrap_or(0);
        let gps_sow: f64 = line2.get(8..23).unwrap_or("0").trim().parse().unwrap_or(0.0);
        let interval_s: f64 = line2.get(24..38).unwrap_or("900").trim().parse().unwrap_or(900.0);

        // Parse satellite list (lines 3-7, each can have 17 satellites)
        let mut satellites = Vec::new();
        for i in 2..7 {
            if i >= lines.len() {
                break;
            }
            let line = lines[i];
            if !line.starts_with('+') {
                continue;
            }
            // Satellites start at column 9, 3 chars each
            for j in 0..17 {
                let start = 9 + j * 3;
                let end = start + 3;
                if end <= line.len() {
                    let sv = line[start..end].trim();
                    if !sv.is_empty() && sv != "0" && sv != "00" {
                        // Handle both "G01" and " G1" formats
                        let sv_clean = sv.trim();
                        if !sv_clean.is_empty() && sv_clean.chars().next().unwrap().is_alphabetic() {
                            satellites.push(normalize_sv_id(sv_clean));
                        }
                    }
                }
            }
        }

        // Parse accuracy exponents (lines 8-12)
        let mut accuracy = HashMap::new();
        let mut sv_idx = 0;
        for i in 7..12 {
            if i >= lines.len() {
                break;
            }
            let line = lines[i];
            if !line.starts_with('+') {
                continue;
            }
            for j in 0..17 {
                let start = 9 + j * 3;
                let end = start + 3;
                if end <= line.len() && sv_idx < satellites.len() {
                    let acc_str = line[start..end].trim();
                    if let Ok(exp) = acc_str.parse::<i32>() {
                        // Accuracy in meters: 2^exp mm = 2^exp / 1000 m
                        let acc_m = (2.0_f64).powi(exp) / 1000.0;
                        accuracy.insert(satellites[sv_idx].clone(), acc_m);
                    }
                    sv_idx += 1;
                }
            }
        }

        let header = Sp3Header {
            version,
            pos_vel_flag,
            start_time_gps_s,
            num_epochs,
            data_used,
            coord_system,
            orbit_type,
            agency,
            gps_week,
            gps_sow,
            interval_s,
            satellites,
            accuracy,
        };

        // Parse epoch and position records
        let mut records: HashMap<String, Vec<Sp3Record>> = HashMap::new();
        let mut current_epoch_gps_s = 0.0;

        for line in &lines[22..] {
            if line.starts_with('*') {
                // Epoch line: *  YYYY MM DD HH MM SS.SSSSSSSS
                let year: i32 = line.get(3..7).unwrap_or("2025").trim().parse().unwrap_or(2025);
                let month: u32 = line.get(8..10).unwrap_or("1").trim().parse().unwrap_or(1);
                let day: u32 = line.get(11..13).unwrap_or("1").trim().parse().unwrap_or(1);
                let hour: u32 = line.get(14..16).unwrap_or("0").trim().parse().unwrap_or(0);
                let minute: u32 = line.get(17..19).unwrap_or("0").trim().parse().unwrap_or(0);
                let second: f64 = line.get(20..31).unwrap_or("0").trim().parse().unwrap_or(0.0);
                current_epoch_gps_s = gps_time_from_calendar(year, month, day, hour, minute, second);
            } else if line.starts_with('P') {
                // Position line: PG01 X.XXXXXX Y.YYYYYY Z.ZZZZZZ C.CCCCCC
                let sv_id = normalize_sv_id(line.get(1..4).unwrap_or("G01").trim());

                // Positions in km, convert to meters
                let x_km: f64 = line.get(4..18).unwrap_or("0").trim().parse().unwrap_or(0.0);
                let y_km: f64 = line.get(18..32).unwrap_or("0").trim().parse().unwrap_or(0.0);
                let z_km: f64 = line.get(32..46).unwrap_or("0").trim().parse().unwrap_or(0.0);

                // Clock in microseconds, convert to seconds
                let clk_us: f64 = line.get(46..60).unwrap_or("999999").trim().parse().unwrap_or(999999.999999);

                // Check for bad values (999999)
                let bad_orbit = x_km.abs() > 900000.0 || y_km.abs() > 900000.0 || z_km.abs() > 900000.0;
                let bad_clock = clk_us.abs() > 900000.0;

                // Check for predicted flag (column 80)
                let predicted = line.len() >= 80 && line.chars().nth(79) == Some('P');

                let record = Sp3Record {
                    gps_time_s: current_epoch_gps_s,
                    x_m: x_km * 1000.0,
                    y_m: y_km * 1000.0,
                    z_m: z_km * 1000.0,
                    clock_s: if bad_clock { None } else { Some(clk_us * 1e-6) },
                    predicted,
                    bad_orbit,
                    bad_clock,
                };

                records.entry(sv_id).or_default().push(record);
            } else if line.starts_with("EOF") {
                break;
            }
        }

        // Sort records by time for each satellite
        for (_, recs) in records.iter_mut() {
            recs.sort_by(|a, b| a.gps_time_s.partial_cmp(&b.gps_time_s).unwrap());
        }

        Ok(Self { header, records })
    }

    /// Get interpolated position and clock at a specific GPS time
    ///
    /// Uses 9-point Lagrange interpolation for positions (recommended for SP3).
    /// Returns (position, velocity, clock_offset) or None if data unavailable.
    pub fn interpolate(
        &self,
        sv_id: &str,
        gps_time_s: f64,
    ) -> Option<(EcefPosition, EcefVelocity, f64)> {
        let sv_id = normalize_sv_id(sv_id);
        let records = self.records.get(&sv_id)?;

        if records.len() < 9 {
            return None;
        }

        // Find the closest epoch index
        let mut closest_idx = 0;
        let mut min_diff = f64::MAX;
        for (i, rec) in records.iter().enumerate() {
            let diff = (rec.gps_time_s - gps_time_s).abs();
            if diff < min_diff {
                min_diff = diff;
                closest_idx = i;
            }
        }

        // Select 9 points centered on closest (or as close as possible)
        let half_window = 4;
        let start = if closest_idx < half_window {
            0
        } else if closest_idx + half_window >= records.len() {
            records.len().saturating_sub(9)
        } else {
            closest_idx - half_window
        };
        let end = (start + 9).min(records.len());

        if end - start < 9 {
            return None;
        }

        let window = &records[start..end];

        // Check for bad orbits in window
        if window.iter().any(|r| r.bad_orbit) {
            return None;
        }

        // Lagrange interpolation for X, Y, Z
        let times: Vec<f64> = window.iter().map(|r| r.gps_time_s).collect();
        let xs: Vec<f64> = window.iter().map(|r| r.x_m).collect();
        let ys: Vec<f64> = window.iter().map(|r| r.y_m).collect();
        let zs: Vec<f64> = window.iter().map(|r| r.z_m).collect();

        let x = lagrange_interpolate(&times, &xs, gps_time_s);
        let y = lagrange_interpolate(&times, &ys, gps_time_s);
        let z = lagrange_interpolate(&times, &zs, gps_time_s);

        // Velocity via numerical differentiation (central difference, dt=1s)
        let dt = 0.5;
        let x_plus = lagrange_interpolate(&times, &xs, gps_time_s + dt);
        let x_minus = lagrange_interpolate(&times, &xs, gps_time_s - dt);
        let y_plus = lagrange_interpolate(&times, &ys, gps_time_s + dt);
        let y_minus = lagrange_interpolate(&times, &ys, gps_time_s - dt);
        let z_plus = lagrange_interpolate(&times, &zs, gps_time_s + dt);
        let z_minus = lagrange_interpolate(&times, &zs, gps_time_s - dt);

        let vx = (x_plus - x_minus) / (2.0 * dt);
        let vy = (y_plus - y_minus) / (2.0 * dt);
        let vz = (z_plus - z_minus) / (2.0 * dt);

        // Clock interpolation (linear is usually sufficient)
        let clocks: Vec<f64> = window
            .iter()
            .filter_map(|r| r.clock_s)
            .collect();
        let clock_times: Vec<f64> = window
            .iter()
            .filter(|r| r.clock_s.is_some())
            .map(|r| r.gps_time_s)
            .collect();

        let clock = if clocks.len() >= 2 {
            lagrange_interpolate(&clock_times, &clocks, gps_time_s)
        } else {
            0.0
        };

        Some((
            EcefPosition::new(x, y, z),
            EcefVelocity::new(vx, vy, vz),
            clock,
        ))
    }

    /// Get list of available satellites
    pub fn satellites(&self) -> Vec<String> {
        self.records.keys().cloned().collect()
    }

    /// Get satellites for a specific constellation
    pub fn satellites_for_constellation(&self, constellation: GnssConstellation) -> Vec<String> {
        let prefix = match constellation {
            GnssConstellation::Gps => 'G',
            GnssConstellation::Galileo => 'E',
            GnssConstellation::Glonass => 'R',
            GnssConstellation::BeiDou => 'C',
        };

        self.records
            .keys()
            .filter(|s| s.starts_with(prefix))
            .cloned()
            .collect()
    }

    /// Get time span covered by this SP3 file
    pub fn time_span(&self) -> Option<(f64, f64)> {
        let mut min_time = f64::MAX;
        let mut max_time = f64::MIN;

        for records in self.records.values() {
            for rec in records {
                if rec.gps_time_s < min_time {
                    min_time = rec.gps_time_s;
                }
                if rec.gps_time_s > max_time {
                    max_time = rec.gps_time_s;
                }
            }
        }

        if min_time < max_time {
            Some((min_time, max_time))
        } else {
            None
        }
    }

    /// Get summary of satellites by constellation
    pub fn summary(&self) -> HashMap<GnssConstellation, usize> {
        let mut counts = HashMap::new();

        for sv in self.records.keys() {
            let constellation = match sv.chars().next() {
                Some('G') => Some(GnssConstellation::Gps),
                Some('E') => Some(GnssConstellation::Galileo),
                Some('R') => Some(GnssConstellation::Glonass),
                Some('C') => Some(GnssConstellation::BeiDou),
                _ => None,
            };

            if let Some(c) = constellation {
                *counts.entry(c).or_insert(0) += 1;
            }
        }

        counts
    }
}

/// Normalize satellite ID to standard format (e.g., "G1" -> "G01", " E5" -> "E05")
fn normalize_sv_id(sv: &str) -> String {
    let sv = sv.trim();
    if sv.len() == 2 {
        // "G1" -> "G01"
        let prefix = sv.chars().next().unwrap();
        let num = sv.chars().nth(1).unwrap();
        format!("{}{:02}", prefix, num.to_digit(10).unwrap_or(0))
    } else if sv.len() == 3 {
        // Already normalized or needs fixing
        let prefix = sv.chars().next().unwrap();
        let num_str: String = sv.chars().skip(1).collect();
        if let Ok(num) = num_str.trim().parse::<u32>() {
            format!("{}{:02}", prefix, num)
        } else {
            sv.to_string()
        }
    } else {
        sv.to_string()
    }
}

/// Convert calendar date to GPS time (seconds since Jan 6, 1980)
///
/// SP3 files use GPS time (not UTC), so no leap second correction is needed.
fn gps_time_from_calendar(year: i32, month: u32, day: u32, hour: u32, min: u32, sec: f64) -> f64 {
    // Days from GPS epoch (Jan 6, 1980) to target date
    let target_jd = julian_day(year, month, day);
    let epoch_jd = julian_day(1980, 1, 6);
    let days = (target_jd - epoch_jd) as f64;
    // SP3 timestamps are already in GPS time, no leap second adjustment needed
    days * 86400.0 + hour as f64 * 3600.0 + min as f64 * 60.0 + sec
}

/// Julian day number for a calendar date (Gregorian)
fn julian_day(year: i32, month: u32, day: u32) -> i64 {
    let y = year as i64;
    let m = month as i64;
    let d = day as i64;
    let a = (14 - m) / 12;
    let y2 = y + 4800 - a;
    let m2 = m + 12 * a - 3;
    d + (153 * m2 + 2) / 5 + 365 * y2 + y2 / 4 - y2 / 100 + y2 / 400 - 32045
}

/// Lagrange polynomial interpolation
fn lagrange_interpolate(x_points: &[f64], y_points: &[f64], x: f64) -> f64 {
    let n = x_points.len();
    let mut result = 0.0;

    for i in 0..n {
        let mut term = y_points[i];
        for j in 0..n {
            if i != j {
                term *= (x - x_points[j]) / (x_points[i] - x_points[j]);
            }
        }
        result += term;
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_sv_id() {
        assert_eq!(normalize_sv_id("G01"), "G01");
        assert_eq!(normalize_sv_id("G1"), "G01");
        assert_eq!(normalize_sv_id(" E5"), "E05");
        assert_eq!(normalize_sv_id("R24"), "R24");
    }

    #[test]
    fn test_gps_time_from_calendar() {
        // 2025-01-15 00:00:00 UTC
        let gps_time = gps_time_from_calendar(2025, 1, 15, 0, 0, 0.0);
        // Should be approximately 1420934418 (GPS time includes leap seconds)
        assert!(gps_time > 1420000000.0);
        assert!(gps_time < 1430000000.0);
    }

    #[test]
    fn test_lagrange_interpolate() {
        // Simple linear case
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![0.0, 1.0, 2.0];
        let result = lagrange_interpolate(&x, &y, 0.5);
        assert!((result - 0.5).abs() < 1e-10);

        // Quadratic case
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![0.0, 1.0, 4.0]; // y = x^2
        let result = lagrange_interpolate(&x, &y, 1.5);
        assert!((result - 2.25).abs() < 1e-10);
    }
}

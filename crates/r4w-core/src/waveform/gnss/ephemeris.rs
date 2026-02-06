//! GNSS Broadcast Ephemeris from RINEX files
//!
//! Provides real satellite positions from broadcast ephemerides downloaded from
//! NASA CDDIS (Crustal Dynamics Data Information System). This gives much more
//! accurate satellite positions than nominal Keplerian orbits.
//!
//! ## Data Sources
//!
//! - **CDDIS**: <https://cddis.nasa.gov/archive/gnss/data/daily/>
//! - **File format**: RINEX v3/v4 navigation files (multi-GNSS)
//! - **Update frequency**: Daily files, updated throughout the day
//!
//! ## Usage
//!
//! ```ignore
//! use r4w_core::waveform::gnss::ephemeris::RinexEphemeris;
//!
//! // Load from a RINEX file
//! let eph = RinexEphemeris::from_file("BRDC00IGS_R_20252580000_01D_MN.rnx")?;
//!
//! // Get satellite position at a specific GPS time
//! let gps_time = 1442003372.0; // GPS seconds
//! if let Some((pos, vel)) = eph.sv_position(3, GnssConstellation::Galileo, gps_time) {
//!     println!("E03 position: {:?}", pos);
//! }
//! ```

use crate::coordinates::{EcefPosition, EcefVelocity};
use crate::waveform::gnss::types::GnssConstellation;
use std::collections::HashMap;
use std::path::Path;

#[cfg(feature = "ephemeris")]
use rinex::prelude::*;

/// Wrapper around parsed RINEX ephemeris data
#[derive(Debug)]
pub struct RinexEphemeris {
    #[cfg(feature = "ephemeris")]
    rinex: Rinex,
    #[cfg(not(feature = "ephemeris"))]
    _phantom: std::marker::PhantomData<()>,
    /// Cache of computed positions (prn, constellation, time) -> (pos, vel)
    cache: std::cell::RefCell<HashMap<(u8, GnssConstellation, u64), (EcefPosition, EcefVelocity)>>,
}

/// Source of ephemeris data for scenario generation
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type", content = "value")]
pub enum EphemerisSource {
    /// Use nominal Keplerian orbits (default, no external data needed)
    Nominal,
    /// Load ephemeris from a RINEX file (broadcast, meter-level accuracy)
    RinexFile(std::path::PathBuf),
    /// Use cached ephemeris for a specific date (YYYY-MM-DD)
    Cached(String),
    /// Automatically fetch ephemeris from CDDIS for the scenario time
    AutoFetch,
    /// Load precise ephemeris from SP3 file (cm-level accuracy)
    Sp3File(std::path::PathBuf),
}

/// Source of ionospheric delay model
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type", content = "value")]
pub enum IonosphereSource {
    /// Use Klobuchar broadcast model (~50% accuracy)
    Klobuchar,
    /// Load TEC maps from IONEX file (higher accuracy)
    IonexFile(std::path::PathBuf),
    /// Disable ionospheric delay
    Disabled,
}

impl Default for IonosphereSource {
    fn default() -> Self {
        Self::Klobuchar
    }
}

impl Default for EphemerisSource {
    fn default() -> Self {
        Self::Nominal
    }
}

/// Result of ephemeris position query
#[derive(Debug, Clone)]
pub struct SvState {
    pub prn: u8,
    pub constellation: GnssConstellation,
    pub position: EcefPosition,
    pub velocity: EcefVelocity,
    pub clock_bias_s: f64,
    pub clock_drift_s_per_s: f64,
}

impl RinexEphemeris {
    /// Load ephemeris from a RINEX navigation file
    #[cfg(feature = "ephemeris")]
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let path = path.as_ref();

        // Check file exists first (rinex crate panics on file not found)
        if !path.exists() {
            return Err(format!("RINEX file not found: {}", path.display()));
        }

        let rinex = Rinex::from_file(path)
            .map_err(|e| format!("Failed to parse RINEX file: {}", e))?;

        if !rinex.is_navigation_rinex() {
            return Err("File is not a RINEX navigation file".to_string());
        }

        Ok(Self {
            rinex,
            cache: std::cell::RefCell::new(HashMap::new()),
        })
    }

    #[cfg(not(feature = "ephemeris"))]
    pub fn from_file<P: AsRef<Path>>(_path: P) -> Result<Self, String> {
        Err("RINEX ephemeris support requires the 'ephemeris' feature".to_string())
    }

    /// Get satellite ECEF position and velocity at a GPS time
    ///
    /// Returns None if no valid ephemeris is available for the satellite at the given time.
    #[cfg(feature = "ephemeris")]
    pub fn sv_position(
        &self,
        prn: u8,
        constellation: GnssConstellation,
        gps_time_s: f64,
    ) -> Option<(EcefPosition, EcefVelocity)> {
        // Check cache first (quantize time to nearest second for caching)
        let cache_key = (prn, constellation, gps_time_s as u64);
        if let Some(cached) = self.cache.borrow().get(&cache_key) {
            return Some(cached.clone());
        }

        // Convert our constellation to rinex SV
        let rinex_constellation = match constellation {
            GnssConstellation::Gps => rinex::prelude::Constellation::GPS,
            GnssConstellation::Galileo => rinex::prelude::Constellation::Galileo,
            GnssConstellation::Glonass => rinex::prelude::Constellation::Glonass,
            GnssConstellation::BeiDou => rinex::prelude::Constellation::BeiDou,
        };

        let sv = SV {
            constellation: rinex_constellation,
            prn,
        };

        // Convert GPS time to rinex Epoch
        // GPS epoch: Jan 6, 1980. GPS time doesn't have leap seconds.
        let gps_epoch = Epoch::from_gpst_seconds(gps_time_s);

        // Try to get orbital state from RINEX
        let orbit = self.rinex.sv_orbit(sv, gps_epoch)?;

        // Extract ECEF position (convert from km to m)
        let pos = EcefPosition::new(
            orbit.radius_km.x * 1000.0,
            orbit.radius_km.y * 1000.0,
            orbit.radius_km.z * 1000.0,
        );

        // Extract velocity (convert from km/s to m/s)
        let vel = EcefVelocity::new(
            orbit.velocity_km_s.x * 1000.0,
            orbit.velocity_km_s.y * 1000.0,
            orbit.velocity_km_s.z * 1000.0,
        );

        // Cache the result
        self.cache.borrow_mut().insert(cache_key, (pos.clone(), vel.clone()));

        Some((pos, vel))
    }

    #[cfg(not(feature = "ephemeris"))]
    pub fn sv_position(
        &self,
        _prn: u8,
        _constellation: GnssConstellation,
        _gps_time_s: f64,
    ) -> Option<(EcefPosition, EcefVelocity)> {
        None
    }

    /// Get satellite clock correction in seconds
    ///
    /// Returns (clock_bias, clock_drift) or None if unavailable.
    #[cfg(feature = "ephemeris")]
    pub fn sv_clock(
        &self,
        prn: u8,
        constellation: GnssConstellation,
        gps_time_s: f64,
    ) -> Option<(f64, f64)> {
        let rinex_constellation = match constellation {
            GnssConstellation::Gps => rinex::prelude::Constellation::GPS,
            GnssConstellation::Galileo => rinex::prelude::Constellation::Galileo,
            GnssConstellation::Glonass => rinex::prelude::Constellation::Glonass,
            GnssConstellation::BeiDou => rinex::prelude::Constellation::BeiDou,
        };

        let sv = SV {
            constellation: rinex_constellation,
            prn,
        };

        let gps_epoch = Epoch::from_gpst_seconds(gps_time_s);

        // Get ephemeris and extract clock parameters
        let (_, _, eph) = self.rinex.nav_ephemeris_selection(sv, gps_epoch)?;

        let clock_bias = eph.clock_bias;
        let clock_drift = eph.clock_drift;

        Some((clock_bias, clock_drift))
    }

    #[cfg(not(feature = "ephemeris"))]
    pub fn sv_clock(
        &self,
        _prn: u8,
        _constellation: GnssConstellation,
        _gps_time_s: f64,
    ) -> Option<(f64, f64)> {
        None
    }

    /// Get list of satellites with valid ephemeris at a given time
    #[cfg(feature = "ephemeris")]
    pub fn available_satellites(
        &self,
        constellation: GnssConstellation,
        gps_time_s: f64,
    ) -> Vec<u8> {
        let rinex_constellation = match constellation {
            GnssConstellation::Gps => rinex::prelude::Constellation::GPS,
            GnssConstellation::Galileo => rinex::prelude::Constellation::Galileo,
            GnssConstellation::Glonass => rinex::prelude::Constellation::Glonass,
            GnssConstellation::BeiDou => rinex::prelude::Constellation::BeiDou,
        };

        let gps_epoch = Epoch::from_gpst_seconds(gps_time_s);

        let mut prns = Vec::new();

        // Check PRNs 1-50 (covers all current constellations)
        for prn in 1..=50 {
            let sv = SV {
                constellation: rinex_constellation,
                prn,
            };
            if self.rinex.nav_ephemeris_selection(sv, gps_epoch).is_some() {
                prns.push(prn);
            }
        }

        prns
    }

    #[cfg(not(feature = "ephemeris"))]
    pub fn available_satellites(
        &self,
        _constellation: GnssConstellation,
        _gps_time_s: f64,
    ) -> Vec<u8> {
        Vec::new()
    }

    /// Get the time span covered by this ephemeris file
    #[cfg(feature = "ephemeris")]
    pub fn time_span(&self) -> Option<(f64, f64)> {
        let mut min_time: Option<f64> = None;
        let mut max_time: Option<f64> = None;

        for (key, _) in self.rinex.nav_ephemeris_frames_iter() {
            let gps_secs = key.epoch.to_gpst_seconds();
            min_time = Some(min_time.map_or(gps_secs, |m| m.min(gps_secs)));
            max_time = Some(max_time.map_or(gps_secs, |m| m.max(gps_secs)));
        }

        match (min_time, max_time) {
            (Some(min), Some(max)) => Some((min, max)),
            _ => None,
        }
    }

    #[cfg(not(feature = "ephemeris"))]
    pub fn time_span(&self) -> Option<(f64, f64)> {
        None
    }

    /// Get summary of constellations and satellite count in the file
    #[cfg(feature = "ephemeris")]
    pub fn summary(&self) -> HashMap<GnssConstellation, usize> {
        let mut counts: HashMap<GnssConstellation, std::collections::HashSet<u8>> = HashMap::new();

        for (key, _) in self.rinex.nav_ephemeris_frames_iter() {
            let constellation = match key.sv.constellation {
                rinex::prelude::Constellation::GPS => Some(GnssConstellation::Gps),
                rinex::prelude::Constellation::Galileo => Some(GnssConstellation::Galileo),
                rinex::prelude::Constellation::Glonass => Some(GnssConstellation::Glonass),
                rinex::prelude::Constellation::BeiDou => Some(GnssConstellation::BeiDou),
                _ => None,
            };
            if let Some(c) = constellation {
                counts.entry(c).or_default().insert(key.sv.prn);
            }
        }

        counts.into_iter().map(|(k, v)| (k, v.len())).collect()
    }

    #[cfg(not(feature = "ephemeris"))]
    pub fn summary(&self) -> HashMap<GnssConstellation, usize> {
        HashMap::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ephemeris_source_default() {
        let source = EphemerisSource::default();
        assert!(matches!(source, EphemerisSource::Nominal));
    }

    #[test]
    fn test_ephemeris_source_serde() {
        let sources = vec![
            EphemerisSource::Nominal,
            EphemerisSource::RinexFile("/path/to/file.rnx".into()),
            EphemerisSource::Cached("2025-09-15".to_string()),
            EphemerisSource::AutoFetch,
        ];

        for source in sources {
            let json = serde_json::to_string(&source).unwrap();
            let decoded: EphemerisSource = serde_json::from_str(&json).unwrap();
            // Just verify it round-trips without error
            let _ = serde_json::to_string(&decoded).unwrap();
        }
    }

    #[cfg(feature = "ephemeris")]
    #[test]
    fn test_from_file_not_found() {
        let result = RinexEphemeris::from_file("/nonexistent/file.rnx");
        assert!(result.is_err());
    }

    #[cfg(not(feature = "ephemeris"))]
    #[test]
    fn test_from_file_feature_disabled() {
        let result = RinexEphemeris::from_file("/any/file.rnx");
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("feature"));
    }
}

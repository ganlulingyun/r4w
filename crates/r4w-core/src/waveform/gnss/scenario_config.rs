//! GNSS scenario configuration and presets
//!
//! Defines the configuration for multi-satellite GNSS IQ generation scenarios,
//! including satellite selection, receiver parameters, environment models, and output format.

use super::environment::{AntennaPattern, GnssMultipathPreset, KeplerianOrbit, KlobucharModel, SaastamoinenModel};
use super::types::{GnssConstellation, GnssSignal};
use crate::coordinates::{lla_to_ecef, look_angle, LlaPosition};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Real-world constellation PRN-to-orbital-slot lookup tables
// ---------------------------------------------------------------------------

/// GPS constellation: real PRN assignments to orbital planes/slots.
/// Each entry is (PRN, plane 0-5 = A-F, slot 0-5 = 1-6).
/// Source: US Coast Guard Navigation Center, GPS Constellation Status (Jan 2026).
const GPS_CONSTELLATION: &[(u8, u8, u8)] = &[
    // Plane A
    (24, 0, 0), // SVN 65, slot A-1
    (31, 0, 1), // SVN 52, slot A-2
    (30, 0, 2), // SVN 64, slot A-3
    ( 7, 0, 3), // SVN 48, slot A-4
    (28, 0, 5), // SVN 79, slot A-6
    // Plane B
    (16, 1, 0), // SVN 56, slot B-1
    (25, 1, 1), // SVN 62, slot B-2
    (22, 1, 2), // SVN 44, slot B-3
    (12, 1, 3), // SVN 58, slot B-4
    (26, 1, 4), // SVN 71, slot B-5
    (14, 1, 5), // SVN 77, slot B-6
    // Plane C
    (29, 2, 0), // SVN 57, slot C-1
    (27, 2, 1), // SVN 66, slot C-2
    ( 8, 2, 2), // SVN 72, slot C-3
    (17, 2, 3), // SVN 53, slot C-4
    (19, 2, 4), // SVN 59, slot C-5
    // Plane D
    ( 2, 3, 0), // SVN 61, slot D-1
    ( 1, 3, 1), // SVN 80, slot D-2
    ( 6, 3, 3), // SVN 67, slot D-4
    (11, 3, 4), // SVN 78, slot D-5
    (18, 3, 5), // SVN 75, slot D-6
    // Plane E
    ( 3, 4, 0), // SVN 69, slot E-1
    (10, 4, 1), // SVN 73, slot E-2
    ( 5, 4, 2), // SVN 50, slot E-3
    (23, 4, 4), // SVN 76, slot E-5
    (21, 4, 5), // SVN 81, slot E-6
    // Plane F
    (32, 5, 0), // SVN 70, slot F-1
    (15, 5, 1), // SVN 55, slot F-2
    ( 9, 5, 2), // SVN 68, slot F-3
    ( 4, 5, 3), // SVN 74, slot F-4
    (13, 5, 5), // SVN 43, slot F-6
];

/// Galileo constellation: real SVID (PRN) assignments to orbital planes/slots.
/// Each entry is (SVID, plane 0-2 = A-C, slot 0-7 = 1-8).
/// Walker 24/3/1 configuration: 8 nominal slots per plane, 45° spacing, 15° phasing.
/// Source: European GNSS Service Centre, Orbital and Technical Parameters (Jan 2026).
const GALILEO_CONSTELLATION: &[(u8, u8, u8)] = &[
    // Plane A
    (31, 0, 0), // GSAT0218, slot A01
    (23, 0, 1), // GSAT0226, slot A02
    (21, 0, 2), // GSAT0215, slot A03
    (27, 0, 3), // GSAT0217, slot A04
    (30, 0, 4), // GSAT0206, slot A05
    ( 2, 0, 5), // GSAT0211, slot A06
    (25, 0, 6), // GSAT0216, slot A07
    (16, 0, 7), // GSAT0232, slot A08
    // Plane B
    (13, 1, 0), // GSAT0220, slot B01
    (15, 1, 1), // GSAT0221, slot B02
    (34, 1, 2), // GSAT0223, slot B03
    (36, 1, 3), // GSAT0219, slot B04
    (11, 1, 4), // GSAT0101, slot B05
    (12, 1, 5), // GSAT0102, slot B06
    (33, 1, 6), // GSAT0222, slot B07
    (26, 1, 7), // GSAT0203, slot B08
    // Plane C
    ( 5, 2, 0), // GSAT0214, slot C01
    ( 9, 2, 1), // GSAT0209, slot C02
    ( 4, 2, 2), // GSAT0213, slot C03
    (19, 2, 3), // GSAT0103, slot C04
    (29, 2, 4), // GSAT0225, slot C05
    ( 7, 2, 5), // GSAT0207, slot C06
    ( 8, 2, 6), // GSAT0208, slot C07
    ( 3, 2, 7), // GSAT0212, slot C08
];

/// GLONASS constellation: approximate PRN-to-slot mapping.
/// Each entry is (slot_number used as PRN, plane 0-2, slot 0-7).
/// GLONASS uses frequency channel numbers, not PRN codes. The slot
/// number (1-24) serves as a satellite identifier.
/// Source: Information-Analytical Centre, GLONASS constellation status.
const GLONASS_CONSTELLATION: &[(u8, u8, u8)] = &[
    // Plane 1
    ( 1, 0, 0), ( 2, 0, 1), ( 3, 0, 2), ( 4, 0, 3),
    ( 5, 0, 4), ( 6, 0, 5), ( 7, 0, 6), ( 8, 0, 7),
    // Plane 2
    ( 9, 1, 0), (10, 1, 1), (11, 1, 2), (12, 1, 3),
    (13, 1, 4), (14, 1, 5), (15, 1, 6), (16, 1, 7),
    // Plane 3
    (17, 2, 0), (18, 2, 1), (19, 2, 2), (20, 2, 3),
    (21, 2, 4), (22, 2, 5), (23, 2, 6), (24, 2, 7),
];

/// Configuration for a single satellite in the scenario
// trace:FR-041 | ai:claude
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatelliteConfig {
    /// GNSS signal type
    pub signal: GnssSignal,
    /// PRN number
    pub prn: u8,
    /// Orbital plane index (0-based)
    pub plane: u8,
    /// Slot within plane (0-based)
    pub slot: u8,
    /// Transmit power in dBW (typically 13-16 dBW for GPS)
    pub tx_power_dbw: f64,
    /// Whether to include navigation data modulation
    pub nav_data: bool,
}

impl SatelliteConfig {
    /// Look up the real-world PRN for a given constellation, plane, and slot.
    ///
    /// Uses the constellation lookup tables (GPS_CONSTELLATION, GALILEO_CONSTELLATION,
    /// GLONASS_CONSTELLATION) to find the actual PRN assigned to each orbital position.
    /// Returns 0 if the plane/slot combination has no assigned satellite.
    pub fn lookup_prn(constellation: GnssConstellation, plane: u8, slot: u8) -> u8 {
        let table: &[(u8, u8, u8)] = match constellation {
            GnssConstellation::Gps | GnssConstellation::BeiDou => GPS_CONSTELLATION,
            GnssConstellation::Galileo => GALILEO_CONSTELLATION,
            GnssConstellation::Glonass => GLONASS_CONSTELLATION,
        };
        for &(prn, p, s) in table {
            if p == plane && s == slot {
                return prn;
            }
        }
        0 // No satellite assigned to this slot
    }

    /// Create a GPS L1 C/A satellite config with PRN looked up from constellation table
    pub fn gps_l1ca(plane: u8, slot: u8) -> Self {
        let prn = Self::lookup_prn(GnssConstellation::Gps, plane, slot);
        assert!(prn > 0, "No GPS satellite at plane={} slot={}", plane, slot);
        Self {
            signal: GnssSignal::GpsL1Ca,
            prn,
            plane,
            slot,
            tx_power_dbw: 14.3, // Typical GPS L1 EIRP
            nav_data: true,
        }
    }

    /// Create a Galileo E1 satellite config with PRN looked up from constellation table
    pub fn galileo_e1(plane: u8, slot: u8) -> Self {
        let prn = Self::lookup_prn(GnssConstellation::Galileo, plane, slot);
        assert!(prn > 0, "No Galileo satellite at plane={} slot={}", plane, slot);
        Self {
            signal: GnssSignal::GalileoE1,
            prn,
            plane,
            slot,
            tx_power_dbw: 15.0,
            nav_data: true,
        }
    }

    /// Create a GPS L5 satellite config with PRN looked up from constellation table
    pub fn gps_l5(plane: u8, slot: u8) -> Self {
        let prn = Self::lookup_prn(GnssConstellation::Gps, plane, slot);
        assert!(prn > 0, "No GPS satellite at plane={} slot={}", plane, slot);
        Self {
            signal: GnssSignal::GpsL5,
            prn,
            plane,
            slot,
            tx_power_dbw: 15.5,
            nav_data: true,
        }
    }

    /// Create a GLONASS L1OF satellite config with PRN looked up from constellation table
    pub fn glonass_l1of(plane: u8, slot: u8) -> Self {
        let prn = Self::lookup_prn(GnssConstellation::Glonass, plane, slot);
        assert!(prn > 0, "No GLONASS satellite at plane={} slot={}", plane, slot);
        Self {
            signal: GnssSignal::GlonassL1of,
            prn,
            plane,
            slot,
            tx_power_dbw: 14.7,
            nav_data: true,
        }
    }
}

/// Receiver configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReceiverConfig {
    /// Receiver position
    pub position: LlaPosition,
    /// Receiver antenna pattern
    pub antenna: AntennaPattern,
    /// Minimum elevation mask in degrees
    pub elevation_mask_deg: f64,
    /// Receiver noise figure in dB
    pub noise_figure_db: f64,
    /// Front-end bandwidth in Hz
    pub bandwidth_hz: f64,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            position: LlaPosition::new(41.08, -85.14, 240.0), // Fort Wayne, IN
            antenna: AntennaPattern::default_patch(),
            elevation_mask_deg: 5.0,
            noise_figure_db: 2.0,
            bandwidth_hz: 2_046_000.0,
        }
    }
}

/// Environment model configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentConfig {
    /// Enable ionospheric delay
    pub ionosphere_enabled: bool,
    /// Klobuchar model (if custom coefficients needed)
    pub ionosphere_model: Option<KlobucharModel>,
    /// Enable tropospheric delay
    pub troposphere_enabled: bool,
    /// Tropospheric model
    pub troposphere_model: Option<SaastamoinenModel>,
    /// Multipath environment preset
    pub multipath_preset: GnssMultipathPreset,
    /// Enable multipath
    pub multipath_enabled: bool,
}

impl Default for EnvironmentConfig {
    fn default() -> Self {
        Self {
            ionosphere_enabled: true,
            ionosphere_model: None,
            troposphere_enabled: true,
            troposphere_model: None,
            multipath_preset: GnssMultipathPreset::OpenSky,
            multipath_enabled: false,
        }
    }
}

/// Output configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputConfig {
    /// Output sample rate in Hz
    pub sample_rate: f64,
    /// Scenario duration in seconds
    pub duration_s: f64,
    /// Processing block size in samples (0 = auto)
    pub block_size: usize,
    /// Random seed for reproducibility
    pub seed: u64,
    /// Scenario start time as GPS seconds (seconds since Jan 6, 1980 00:00:00 UTC).
    /// Orbital positions are computed at this epoch + elapsed sample time.
    pub start_time_gps_s: f64,
}

/// Compute GPS time in seconds for a given UTC date and time of day.
///
/// GPS epoch is January 6, 1980 00:00:00 UTC. GPS time does not include
/// leap seconds (as of 2025 there are 18 leap seconds between UTC and GPS).
pub fn gps_time_from_utc(year: i32, month: u32, day: u32, hour: u32, min: u32, sec: u32) -> f64 {
    // Days from GPS epoch (Jan 6, 1980) to target date
    let target_jd = julian_day(year, month, day);
    let epoch_jd = julian_day(1980, 1, 6);
    let days = (target_jd - epoch_jd) as f64;
    let utc_seconds = days * 86400.0 + hour as f64 * 3600.0 + min as f64 * 60.0 + sec as f64;
    // GPS time = UTC + leap seconds (18 as of 2017, no new ones through 2025)
    utc_seconds + 18.0
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

impl Default for OutputConfig {
    fn default() -> Self {
        // Default: 2026-02-04 15:00 EST (20:00 UTC), Fort Wayne IN
        let start_time = gps_time_from_utc(2026, 2, 4, 20, 0, 0);
        Self {
            sample_rate: 2_046_000.0,
            duration_s: 0.001,
            block_size: 0, // auto
            seed: 42,
            start_time_gps_s: start_time,
        }
    }
}

/// Full GNSS scenario configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GnssScenarioConfig {
    /// Satellites to include
    pub satellites: Vec<SatelliteConfig>,
    /// Receiver settings
    pub receiver: ReceiverConfig,
    /// Environment model settings
    pub environment: EnvironmentConfig,
    /// Output settings
    pub output: OutputConfig,
}

/// Pre-configured scenario presets
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GnssScenarioPreset {
    /// Open sky with 4 GPS SVs, static receiver
    OpenSky,
    /// Urban canyon with multipath, 6 GPS SVs
    UrbanCanyon,
    /// Driving scenario with receiver motion
    Driving,
    /// Walking scenario with slow motion
    Walking,
    /// High-dynamics (aircraft/missile)
    HighDynamics,
    /// Multi-constellation (GPS + Galileo)
    MultiConstellation,
}

impl std::fmt::Display for GnssScenarioPreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OpenSky => write!(f, "Open Sky"),
            Self::UrbanCanyon => write!(f, "Urban Canyon"),
            Self::Driving => write!(f, "Driving"),
            Self::Walking => write!(f, "Walking"),
            Self::HighDynamics => write!(f, "High Dynamics"),
            Self::MultiConstellation => write!(f, "Multi-Constellation"),
        }
    }
}

impl GnssScenarioPreset {
    /// Convert preset to full configuration
    pub fn to_config(&self) -> GnssScenarioConfig {
        match self {
            // All presets use satellites visible from Fort Wayne IN (41.08N, 85.14W)
            // at 2026-02-04 15:00 EST (GPS time 1454270418 s) with nominal Keplerian orbits.
            Self::OpenSky => GnssScenarioConfig {
                satellites: vec![
                    SatelliteConfig::gps_l1ca(4, 2),   // PRN 5
                    SatelliteConfig::gps_l1ca(2, 3),   // PRN 17
                    SatelliteConfig::gps_l1ca(1, 4),   // PRN 26
                    SatelliteConfig::gps_l1ca(3, 3),   // PRN 6
                    SatelliteConfig::gps_l1ca(2, 4),   // PRN 19
                    SatelliteConfig::gps_l1ca(1, 3),   // PRN 12
                    SatelliteConfig::gps_l1ca(4, 1),   // PRN 10
                    SatelliteConfig::gps_l1ca(5, 2),   // PRN 9
                ],
                receiver: ReceiverConfig::default(),
                environment: EnvironmentConfig {
                    multipath_preset: GnssMultipathPreset::OpenSky,
                    ..Default::default()
                },
                output: OutputConfig::default(),
            },
            Self::UrbanCanyon => GnssScenarioConfig {
                satellites: vec![
                    SatelliteConfig::gps_l1ca(4, 2),   // PRN 5
                    SatelliteConfig::gps_l1ca(2, 3),   // PRN 17
                    SatelliteConfig::gps_l1ca(1, 4),   // PRN 26
                    SatelliteConfig::gps_l1ca(3, 3),   // PRN 6
                    SatelliteConfig::gps_l1ca(2, 4),   // PRN 19
                    SatelliteConfig::gps_l1ca(1, 3),   // PRN 12
                    SatelliteConfig::gps_l1ca(4, 1),   // PRN 10
                    SatelliteConfig::gps_l1ca(5, 2),   // PRN 9
                ],
                receiver: ReceiverConfig {
                    elevation_mask_deg: 15.0,
                    ..Default::default()
                },
                environment: EnvironmentConfig {
                    multipath_preset: GnssMultipathPreset::UrbanCanyon,
                    multipath_enabled: true,
                    ..Default::default()
                },
                output: OutputConfig::default(),
            },
            Self::Driving => GnssScenarioConfig {
                satellites: vec![
                    SatelliteConfig::gps_l1ca(4, 2),   // PRN 5
                    SatelliteConfig::gps_l1ca(2, 3),   // PRN 17
                    SatelliteConfig::gps_l1ca(1, 4),   // PRN 26
                    SatelliteConfig::gps_l1ca(3, 3),   // PRN 6
                    SatelliteConfig::gps_l1ca(2, 4),   // PRN 19
                    SatelliteConfig::gps_l1ca(1, 3),   // PRN 12
                    SatelliteConfig::gps_l1ca(4, 1),   // PRN 10
                ],
                receiver: ReceiverConfig::default(),
                environment: EnvironmentConfig {
                    multipath_preset: GnssMultipathPreset::Suburban,
                    multipath_enabled: true,
                    ..Default::default()
                },
                output: OutputConfig {
                    duration_s: 0.01,
                    ..Default::default()
                },
            },
            Self::Walking => GnssScenarioConfig {
                satellites: vec![
                    SatelliteConfig::gps_l1ca(4, 2),   // PRN 5
                    SatelliteConfig::gps_l1ca(2, 3),   // PRN 17
                    SatelliteConfig::gps_l1ca(1, 4),   // PRN 26
                    SatelliteConfig::gps_l1ca(3, 3),   // PRN 6
                    SatelliteConfig::gps_l1ca(2, 4),   // PRN 19
                    SatelliteConfig::gps_l1ca(1, 3),   // PRN 12
                ],
                receiver: ReceiverConfig::default(),
                environment: EnvironmentConfig::default(),
                output: OutputConfig {
                    duration_s: 0.01,
                    ..Default::default()
                },
            },
            Self::HighDynamics => GnssScenarioConfig {
                satellites: vec![
                    SatelliteConfig::gps_l1ca(4, 2),   // PRN 5
                    SatelliteConfig::gps_l1ca(2, 3),   // PRN 17
                    SatelliteConfig::gps_l1ca(1, 4),   // PRN 26
                    SatelliteConfig::gps_l1ca(3, 3),   // PRN 6
                    SatelliteConfig::gps_l1ca(2, 4),   // PRN 19
                    SatelliteConfig::gps_l1ca(1, 3),   // PRN 12
                    SatelliteConfig::gps_l1ca(4, 1),   // PRN 10
                    SatelliteConfig::gps_l1ca(5, 2),   // PRN 9
                ],
                receiver: ReceiverConfig::default(),
                environment: EnvironmentConfig::default(),
                output: OutputConfig {
                    duration_s: 0.001,
                    ..Default::default()
                },
            },
            Self::MultiConstellation => GnssScenarioConfig {
                satellites: vec![
                    // GPS
                    SatelliteConfig::gps_l1ca(4, 2),   // PRN 5
                    SatelliteConfig::gps_l1ca(2, 3),   // PRN 17
                    SatelliteConfig::gps_l1ca(1, 4),   // PRN 26
                    SatelliteConfig::gps_l1ca(3, 3),   // PRN 6
                    SatelliteConfig::gps_l1ca(2, 4),   // PRN 19
                    // Galileo
                    SatelliteConfig::galileo_e1(0, 7),  // PRN 16 (E16)
                    SatelliteConfig::galileo_e1(1, 5),  // PRN 12 (E12)
                    SatelliteConfig::galileo_e1(0, 0),  // PRN 31 (E31)
                    SatelliteConfig::galileo_e1(1, 6),  // PRN 33 (E33)
                    SatelliteConfig::galileo_e1(1, 4),  // PRN 11 (E11)
                    SatelliteConfig::galileo_e1(0, 6),  // PRN 25 (E25)
                ],
                receiver: ReceiverConfig::default(),
                environment: EnvironmentConfig::default(),
                output: OutputConfig::default(),
            },
        }
    }

    /// All preset variants
    pub fn all() -> &'static [GnssScenarioPreset] {
        &[
            Self::OpenSky,
            Self::UrbanCanyon,
            Self::Driving,
            Self::Walking,
            Self::HighDynamics,
            Self::MultiConstellation,
        ]
    }
}

/// Discover visible satellites for a given signal type, location, and time.
///
/// Uses real-world constellation lookup tables (GPS NAVCEN, Galileo GSC) to
/// iterate over actual satellite PRN/plane/slot assignments, computes orbital
/// position at `gps_time_s` using nominal Keplerian orbits, and returns
/// `SatelliteConfig` entries for satellites above `elevation_mask_deg`,
/// sorted by descending elevation.
pub fn discover_visible_satellites(
    signal: GnssSignal,
    position: &LlaPosition,
    gps_time_s: f64,
    elevation_mask_deg: f64,
) -> Vec<(SatelliteConfig, f64)> {
    let rx_ecef = lla_to_ecef(position);

    // Select the real-world constellation lookup table
    let constellation_table: &[(u8, u8, u8)] = match signal.constellation() {
        GnssConstellation::Gps => GPS_CONSTELLATION,
        GnssConstellation::Galileo => GALILEO_CONSTELLATION,
        GnssConstellation::Glonass => GLONASS_CONSTELLATION,
        GnssConstellation::BeiDou => GPS_CONSTELLATION, // fallback
    };

    let mut visible: Vec<(SatelliteConfig, f64)> = Vec::new();

    for &(prn, plane, slot) in constellation_table {
        let orbit = match signal.constellation() {
            GnssConstellation::Gps => KeplerianOrbit::gps_nominal(plane, slot),
            GnssConstellation::Galileo => KeplerianOrbit::galileo_nominal(plane, slot),
            GnssConstellation::Glonass => KeplerianOrbit::glonass_nominal(plane, slot),
            GnssConstellation::BeiDou => KeplerianOrbit::gps_nominal(plane, slot),
        };

        let (pos, _vel) = orbit.position_velocity_at(gps_time_s);
        let la = look_angle(&rx_ecef, position, &pos);

        if la.elevation_deg > elevation_mask_deg {
            let sat = SatelliteConfig {
                signal,
                prn,
                plane,
                slot,
                tx_power_dbw: match signal {
                    GnssSignal::GpsL1Ca => 14.3,
                    GnssSignal::GpsL5 => 15.5,
                    GnssSignal::GlonassL1of => 14.7,
                    GnssSignal::GalileoE1 => 15.0,
                },
                nav_data: true,
            };
            visible.push((sat, la.elevation_deg));
        }
    }

    // Sort by descending elevation
    visible.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    visible
}

/// Discover visible satellites for all signal types used in a config.
///
/// Replaces the satellite list in the config with auto-discovered visible
/// satellites at the config's receiver position and start time.
pub fn discover_satellites_for_config(config: &mut GnssScenarioConfig) {
    // Collect unique signal types from the existing config (preserving order)
    let mut signal_types: Vec<GnssSignal> = Vec::new();
    for sat in &config.satellites {
        if !signal_types.contains(&sat.signal) {
            signal_types.push(sat.signal);
        }
    }

    let mut all_sats = Vec::new();

    for signal in &signal_types {
        let visible = discover_visible_satellites(
            *signal,
            &config.receiver.position,
            config.output.start_time_gps_s,
            config.receiver.elevation_mask_deg,
        );

        for (sat, _el) in visible {
            all_sats.push(sat);
        }
    }

    config.satellites = all_sats;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_all_presets_produce_valid_config() {
        for preset in GnssScenarioPreset::all() {
            let config = preset.to_config();
            assert!(!config.satellites.is_empty(), "{} has no satellites", preset);
            assert!(config.output.sample_rate > 0.0);
            assert!(config.output.duration_s > 0.0);
        }
    }

    #[test]
    fn test_satellite_config_constructors() {
        // GPS plane A (0), slot 0 -> PRN 24 (from real constellation table)
        let gps = SatelliteConfig::gps_l1ca(0, 0);
        assert_eq!(gps.signal, GnssSignal::GpsL1Ca);
        assert_eq!(gps.prn, 24);
        assert_eq!(gps.plane, 0);
        assert_eq!(gps.slot, 0);

        // GPS plane C (2), slot 3 -> PRN 17
        let gps2 = SatelliteConfig::gps_l1ca(2, 3);
        assert_eq!(gps2.prn, 17);

        // Galileo plane A (0), slot 0 -> PRN 31 (E31)
        let gal = SatelliteConfig::galileo_e1(0, 0);
        assert_eq!(gal.signal, GnssSignal::GalileoE1);
        assert_eq!(gal.prn, 31);

        // Galileo plane B (1), slot 4 -> PRN 11 (E11)
        let gal2 = SatelliteConfig::galileo_e1(1, 4);
        assert_eq!(gal2.prn, 11);

        // GLONASS plane 2, slot 3 -> PRN 20
        let glo = SatelliteConfig::glonass_l1of(2, 3);
        assert_eq!(glo.prn, 20);
    }

    #[test]
    #[ignore] // Calibration tool — run manually with --ignored
    fn calibrate_galileo_raan_offset() {
        use crate::coordinates::{lla_to_ecef, look_angle};
        let pos = LlaPosition::new(41.08, -85.14, 240.0);
        let rx_ecef = lla_to_ecef(&pos);
        let t = gps_time_from_utc(2026, 2, 4, 20, 1, 10);
        let el_mask = 5.0;

        // MATLAB shows these Galileo PRNs visible at this time/location
        let expected: &[u8] = &[2, 3, 5, 11, 12, 16, 23, 25, 31, 33];
        // Note: E10 excluded (auxiliary slot, not in our 24-sat nominal table)

        let mut best_raan_deg = 0.0f64;
        let mut best_m0_deg = 0.0f64;
        let mut best_score = 0usize;
        let mut best_extra = 100usize;
        let mut best_prns: Vec<u8> = Vec::new();

        // Sweep RAAN offset and M0 offset in 2D (2° steps)
        for raan_i in 0..180 {
            let raan_offset = (raan_i as f64 * 2.0).to_radians();
            for m0_i in 0..180 {
                let m0_offset = (m0_i as f64 * 2.0).to_radians();
                let mut visible_prns: Vec<u8> = Vec::new();

                for &(prn, plane, slot) in GALILEO_CONSTELLATION {
                    let mut orbit = KeplerianOrbit::galileo_nominal(plane, slot);
                    orbit.omega_0 += raan_offset;
                    orbit.m0 += m0_offset;
                    let (sat_pos, _) = orbit.position_velocity_at(t);
                    let la = look_angle(&rx_ecef, &pos, &sat_pos);
                    if la.elevation_deg > el_mask {
                        visible_prns.push(prn);
                    }
                }

                let matched = visible_prns.iter()
                    .filter(|p| expected.contains(p))
                    .count();
                let extra = visible_prns.len() - matched;

                if matched > best_score || (matched == best_score && extra < best_extra) {
                    best_score = matched;
                    best_extra = extra;
                    best_raan_deg = raan_i as f64 * 2.0;
                    best_m0_deg = m0_i as f64 * 2.0;
                    best_prns = visible_prns.clone();
                }
            }
        }

        eprintln!("\nBest: RAAN={:.0}° M0={:.0}° score={}/{} extra={} visible={:?}",
            best_raan_deg, best_m0_deg, best_score, expected.len(), best_extra, best_prns);
        eprintln!("Expected: {:?}", expected);

        // Fine-tune around best in 0.5° steps
        let raan_center = best_raan_deg;
        let m0_center = best_m0_deg;
        for dr in -20..=20 {
            let raan_offset = (raan_center + dr as f64 * 0.5).to_radians();
            for dm in -20..=20 {
                let m0_offset = (m0_center + dm as f64 * 0.5).to_radians();
                let mut visible_prns: Vec<u8> = Vec::new();

                for &(prn, plane, slot) in GALILEO_CONSTELLATION {
                    let mut orbit = KeplerianOrbit::galileo_nominal(plane, slot);
                    orbit.omega_0 += raan_offset;
                    orbit.m0 += m0_offset;
                    let (sat_pos, _) = orbit.position_velocity_at(t);
                    let la = look_angle(&rx_ecef, &pos, &sat_pos);
                    if la.elevation_deg > el_mask {
                        visible_prns.push(prn);
                    }
                }

                let matched = visible_prns.iter()
                    .filter(|p| expected.contains(p))
                    .count();
                let extra = visible_prns.len() - matched;

                if matched > best_score || (matched == best_score && extra < best_extra) {
                    best_score = matched;
                    best_extra = extra;
                    best_raan_deg = raan_center + dr as f64 * 0.5;
                    best_m0_deg = m0_center + dm as f64 * 0.5;
                    best_prns = visible_prns.clone();
                }
            }
        }

        eprintln!("Refined: RAAN={:.1}° M0={:.1}° score={}/{} extra={} visible={:?}",
            best_raan_deg, best_m0_deg, best_score, expected.len(), best_extra, best_prns);

        // Also check elevations at best offset
        let raan_offset = best_raan_deg.to_radians();
        let m0_offset = best_m0_deg.to_radians();
        eprintln!("\nAll satellites at best offset:");
        for &(prn, plane, slot) in GALILEO_CONSTELLATION {
            let mut orbit = KeplerianOrbit::galileo_nominal(plane, slot);
            orbit.omega_0 += raan_offset;
            orbit.m0 += m0_offset;
            let (sat_pos, _) = orbit.position_velocity_at(t);
            let la = look_angle(&rx_ecef, &pos, &sat_pos);
            let marker = if expected.contains(&prn) { "*" } else { " " };
            if la.elevation_deg > -5.0 {
                eprintln!("  {}E{:02} plane={} slot={} el={:.1}° az={:.1}°",
                    marker, prn, plane, slot, la.elevation_deg, la.azimuth_deg);
            }
        }
    }

    #[test]
    #[ignore] // Calibration tool — run manually with --ignored
    fn calibrate_gps_raan_offset() {
        use crate::coordinates::{lla_to_ecef, look_angle};
        let pos = LlaPosition::new(41.08, -85.14, 240.0);
        let rx_ecef = lla_to_ecef(&pos);
        // MATLAB GPS plot is at 22:00:10 UTC (2 hours later than Galileo)
        let t = gps_time_from_utc(2026, 2, 4, 22, 0, 10);
        let el_mask = 5.0;

        // MATLAB shows these GPS PRNs visible
        let expected: &[u8] = &[2, 10, 11, 12, 16, 25, 31, 33, 36];
        // Note: PRN 33 and 36 don't exist in our 31-satellite table

        let mut best_raan_deg = 0.0f64;
        let mut best_m0_deg = 0.0f64;
        let mut best_score = 0usize;
        let mut best_extra = 100usize;
        let mut best_prns: Vec<u8> = Vec::new();

        for raan_i in 0..180 {
            let raan_offset = (raan_i as f64 * 2.0).to_radians();
            for m0_i in 0..180 {
                let m0_offset = (m0_i as f64 * 2.0).to_radians();
                let mut visible_prns: Vec<u8> = Vec::new();

                for &(prn, plane, slot) in GPS_CONSTELLATION {
                    let mut orbit = KeplerianOrbit::gps_nominal(plane, slot);
                    orbit.omega_0 += raan_offset;
                    orbit.m0 += m0_offset;
                    let (sat_pos, _) = orbit.position_velocity_at(t);
                    let la = look_angle(&rx_ecef, &pos, &sat_pos);
                    if la.elevation_deg > el_mask {
                        visible_prns.push(prn);
                    }
                }

                let matched = visible_prns.iter()
                    .filter(|p| expected.contains(p))
                    .count();
                let extra = visible_prns.len() - matched;

                if matched > best_score || (matched == best_score && extra < best_extra) {
                    best_score = matched;
                    best_extra = extra;
                    best_raan_deg = raan_i as f64 * 2.0;
                    best_m0_deg = m0_i as f64 * 2.0;
                    best_prns = visible_prns.clone();
                }
            }
        }

        eprintln!("GPS Best: RAAN={:.0}° M0={:.0}° score={}/{} extra={} visible={:?}",
            best_raan_deg, best_m0_deg, best_score, expected.len(), best_extra, best_prns);
        eprintln!("GPS Expected: {:?}", expected);

        // Show all GPS at best offset
        let raan_offset = best_raan_deg.to_radians();
        let m0_offset = best_m0_deg.to_radians();
        eprintln!("\nAll GPS at best offset:");
        for &(prn, plane, slot) in GPS_CONSTELLATION {
            let mut orbit = KeplerianOrbit::gps_nominal(plane, slot);
            orbit.omega_0 += raan_offset;
            orbit.m0 += m0_offset;
            let (sat_pos, _) = orbit.position_velocity_at(t);
            let la = look_angle(&rx_ecef, &pos, &sat_pos);
            let marker = if expected.contains(&prn) { "*" } else { " " };
            if la.elevation_deg > 0.0 {
                eprintln!("  {}G{:02} plane={} slot={} el={:.1}° az={:.1}°",
                    marker, prn, plane, slot, la.elevation_deg, la.azimuth_deg);
            }
        }
    }

    #[test]
    #[ignore] // Calibration tool — run manually with --ignored
    fn verify_galileo_offset_at_2200() {
        use crate::coordinates::{lla_to_ecef, look_angle};
        let pos = LlaPosition::new(41.08, -85.14, 240.0);
        let rx_ecef = lla_to_ecef(&pos);
        // First MATLAB plot time: 22:00:10 UTC
        let t = gps_time_from_utc(2026, 2, 4, 22, 0, 10);
        let el_mask = 5.0;
        let raan_offset = 118.0f64.to_radians();
        let m0_offset = 176.0f64.to_radians();

        // MATLAB plot 1 PRNs (if Galileo): 2, 10, 11, 12, 16, 25, 31, 33, 36
        let expected: &[u8] = &[2, 10, 11, 12, 16, 25, 31, 33, 36];

        eprintln!("\nGalileo visibility at 22:00:10 UTC with RAAN=118° M0=176°:");
        let mut visible: Vec<u8> = Vec::new();
        for &(prn, plane, slot) in GALILEO_CONSTELLATION {
            let mut orbit = KeplerianOrbit::galileo_nominal(plane, slot);
            orbit.omega_0 += raan_offset;
            orbit.m0 += m0_offset;
            let (sat_pos, _) = orbit.position_velocity_at(t);
            let la = look_angle(&rx_ecef, &pos, &sat_pos);
            if la.elevation_deg > el_mask {
                visible.push(prn);
                let marker = if expected.contains(&prn) { "*" } else { " " };
                eprintln!("  {}E{:02} el={:.1}° az={:.1}°", marker, prn, la.elevation_deg, la.azimuth_deg);
            }
        }
        let matched = visible.iter().filter(|p| expected.contains(p)).count();
        eprintln!("Score: {}/{}, visible: {:?}", matched, expected.len(), visible);
        eprintln!("Expected: {:?}", expected);
    }
}

//! GNSS scenario configuration and presets
//!
//! Defines the configuration for multi-satellite GNSS IQ generation scenarios,
//! including satellite selection, receiver parameters, environment models, and output format.

use super::environment::{AntennaPattern, GnssMultipathPreset, KlobucharModel, SaastamoinenModel};
use super::types::GnssSignal;
use crate::coordinates::LlaPosition;
use serde::{Deserialize, Serialize};

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
    /// Create a GPS L1 C/A satellite config
    pub fn gps_l1ca(prn: u8, plane: u8, slot: u8) -> Self {
        Self {
            signal: GnssSignal::GpsL1Ca,
            prn,
            plane,
            slot,
            tx_power_dbw: 14.3, // Typical GPS L1 EIRP
            nav_data: true,
        }
    }

    /// Create a Galileo E1 satellite config
    pub fn galileo_e1(prn: u8, plane: u8, slot: u8) -> Self {
        Self {
            signal: GnssSignal::GalileoE1,
            prn,
            plane,
            slot,
            tx_power_dbw: 15.0,
            nav_data: true,
        }
    }

    /// Create a GPS L5 satellite config
    pub fn gps_l5(prn: u8, plane: u8, slot: u8) -> Self {
        Self {
            signal: GnssSignal::GpsL5,
            prn,
            plane,
            slot,
            tx_power_dbw: 15.5,
            nav_data: true,
        }
    }

    /// Create a GLONASS L1OF satellite config
    pub fn glonass_l1of(prn: u8, plane: u8, slot: u8) -> Self {
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
            position: LlaPosition::new(40.0, -75.0, 50.0), // Philadelphia
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
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            sample_rate: 2_046_000.0,
            duration_s: 0.001,
            block_size: 0, // auto
            seed: 42,
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
            Self::OpenSky => GnssScenarioConfig {
                satellites: vec![
                    SatelliteConfig::gps_l1ca(1, 0, 0),
                    SatelliteConfig::gps_l1ca(7, 1, 0),
                    SatelliteConfig::gps_l1ca(14, 2, 0),
                    SatelliteConfig::gps_l1ca(21, 3, 0),
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
                    SatelliteConfig::gps_l1ca(1, 0, 0),
                    SatelliteConfig::gps_l1ca(3, 0, 2),
                    SatelliteConfig::gps_l1ca(7, 1, 0),
                    SatelliteConfig::gps_l1ca(14, 2, 0),
                    SatelliteConfig::gps_l1ca(21, 3, 0),
                    SatelliteConfig::gps_l1ca(28, 4, 0),
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
                    SatelliteConfig::gps_l1ca(1, 0, 0),
                    SatelliteConfig::gps_l1ca(7, 1, 0),
                    SatelliteConfig::gps_l1ca(14, 2, 0),
                    SatelliteConfig::gps_l1ca(21, 3, 0),
                    SatelliteConfig::gps_l1ca(28, 4, 0),
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
                    SatelliteConfig::gps_l1ca(1, 0, 0),
                    SatelliteConfig::gps_l1ca(7, 1, 0),
                    SatelliteConfig::gps_l1ca(14, 2, 0),
                    SatelliteConfig::gps_l1ca(21, 3, 0),
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
                    SatelliteConfig::gps_l1ca(1, 0, 0),
                    SatelliteConfig::gps_l1ca(7, 1, 0),
                    SatelliteConfig::gps_l1ca(14, 2, 0),
                    SatelliteConfig::gps_l1ca(21, 3, 0),
                    SatelliteConfig::gps_l1ca(28, 4, 0),
                    SatelliteConfig::gps_l1ca(32, 5, 0),
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
                    SatelliteConfig::gps_l1ca(1, 0, 0),
                    SatelliteConfig::gps_l1ca(7, 1, 0),
                    SatelliteConfig::gps_l1ca(14, 2, 0),
                    SatelliteConfig::galileo_e1(1, 0, 0),
                    SatelliteConfig::galileo_e1(5, 0, 4),
                    SatelliteConfig::galileo_e1(9, 1, 0),
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
        let gps = SatelliteConfig::gps_l1ca(1, 0, 0);
        assert_eq!(gps.signal, GnssSignal::GpsL1Ca);
        assert_eq!(gps.prn, 1);

        let gal = SatelliteConfig::galileo_e1(1, 0, 0);
        assert_eq!(gal.signal, GnssSignal::GalileoE1);
    }
}

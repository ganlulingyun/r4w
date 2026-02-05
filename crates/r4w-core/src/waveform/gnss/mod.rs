//! GNSS (Global Navigation Satellite System) Waveforms
//!
//! Implements baseband signal generation, acquisition, tracking, and navigation
//! data extraction for multiple GNSS constellations:
//!
//! - **GPS L1 C/A**: BPSK(1), 1023-chip Gold codes, 1.023 Mchip/s
//! - **GLONASS L1OF**: BPSK(0.5), 511-chip m-sequence, FDMA
//! - **Galileo E1**: CBOC(6,1,1/11), 4092-chip memory codes
//! - **GPS L5**: QPSK(10), 10230-chip codes, 10.23 Mchip/s
//!
//! ## Signal Processing Pipeline
//!
//! ```text
//! ┌──────────┐   ┌───────────┐   ┌──────────┐   ┌──────────┐
//! │ Nav Data │ → │ PRN Code  │ → │ Spreading│ → │   BPSK   │
//! │ (50 bps) │   │ Generator │   │  (XOR)   │   │ Modulate │
//! └──────────┘   └───────────┘   └──────────┘   └──────────┘
//!
//! ┌──────────┐   ┌───────────┐   ┌──────────┐   ┌──────────┐
//! │  2D FFT  │ → │ DLL/PLL   │ → │ Bit Sync │ → │ Nav Data │
//! │ Acquire  │   │ Tracking  │   │          │   │ Decode   │
//! └──────────┘   └───────────┘   └──────────┘   └──────────┘
//! ```

pub mod types;
pub mod prn;
pub mod acquisition;
pub mod tracking;
pub mod nav_message;
pub mod gps_l1ca;
pub mod glonass_l1of;
pub mod boc;
pub mod galileo_e1;
pub mod gps_l5;
pub mod environment;
pub mod satellite_emitter;
pub mod scenario;
pub mod scenario_config;

pub use types::*;
pub use prn::{GpsCaCodeGenerator, GlonassCodeGenerator, GalileoE1CodeGenerator, GpsL5CodeGenerator};
pub use acquisition::PcpsAcquisition;
pub use tracking::TrackingChannel;
pub use nav_message::LnavMessage;
pub use gps_l1ca::GpsL1Ca;
pub use glonass_l1of::GlonassL1of;
pub use galileo_e1::GalileoE1;
pub use gps_l5::GpsL5;
pub use environment::{AntennaPattern, BodyAttitude, GnssMultipathPreset, KeplerianOrbit, KlobucharModel, SaastamoinenModel};
pub use satellite_emitter::SatelliteEmitter;
pub use scenario::GnssScenario;
pub use scenario_config::{GnssScenarioConfig, GnssScenarioPreset, SatelliteConfig, ReceiverConfig, EnvironmentConfig, OutputConfig, gps_time_from_utc, discover_visible_satellites, discover_satellites_for_config};

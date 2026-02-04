//! GNSS environment models
//!
//! Physical models for GNSS signal propagation including orbital mechanics,
//! ionospheric/tropospheric delays, multipath, and antenna patterns.

pub mod antenna;
pub mod ionosphere;
pub mod multipath;
pub mod orbit;
pub mod troposphere;

pub use antenna::{AntennaPattern, BodyAttitude};
pub use ionosphere::KlobucharModel;
pub use multipath::{GnssMultipathPreset, MultipathTap};
pub use orbit::KeplerianOrbit;
pub use troposphere::SaastamoinenModel;

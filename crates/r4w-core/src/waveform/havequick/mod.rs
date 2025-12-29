//! HAVEQUICK Waveform Implementation
//!
//! This module provides a framework for HAVEQUICK (Have Quick) UHF frequency
//! hopping radio implementation. The classified components are abstracted
//! behind traits that must be implemented by authorized parties.
//!
//! # Overview
//!
//! HAVEQUICK is a military UHF (225-400 MHz) frequency hopping system used
//! primarily for air-to-air and air-to-ground communications. Key features:
//!
//! - **Frequency Range**: UHF 225-400 MHz (~7000 channels, 25 kHz spacing)
//! - **Modulation**: AM for voice, ASK for data
//! - **Security**: Word of Day (WOD), Time of Day (TOD), Net ID
//! - **Anti-Jam**: Frequency hopping across the UHF band
//!
//! # Architecture
//!
//! The implementation separates unclassified framework code from classified algorithms:
//!
//! - **Framework (this module)**: AM/ASK modulation, channel structure, timing
//! - **Classified (external)**: Hopping algorithm, TRANSEC integration
//!
//! # Usage with Simulator
//!
//! ```rust
//! use r4w_core::waveform::havequick::HavequickBuilder;
//!
//! // Create a test instance with simulator (non-secure) implementations
//! let havequick = HavequickBuilder::simulator().build().unwrap();
//! ```
//!
//! # Usage with Classified Implementations
//!
//! ```rust,ignore
//! use r4w_core::waveform::havequick::HavequickBuilder;
//! use my_classified_crate::{ClassifiedHopper, ClassifiedTransec, ...};
//!
//! let havequick = HavequickBuilder::new()
//!     .with_hopper(Box::new(ClassifiedHopper::new()))
//!     .with_transec(Box::new(ClassifiedTransec::new()))
//!     // ... other components
//!     .build()
//!     .unwrap();
//! ```
//!
//! # Differences from SINCGARS
//!
//! | Feature | HAVEQUICK | SINCGARS |
//! |---------|-----------|----------|
//! | Band | UHF (225-400 MHz) | VHF (30-88 MHz) |
//! | Modulation | AM/ASK | FM |
//! | Primary Use | Air-to-Air/Ground | Ground tactical |
//! | Channels | ~7000 | ~2320 |
//!
//! # References
//!
//! - [Have Quick - Wikipedia](https://en.wikipedia.org/wiki/Have_Quick)
//! - MIL-STD-188-141B (related ALE standard)

mod types;
mod traits;
mod framework;
mod simulator;
mod builder;

pub use types::*;
pub use traits::*;
pub use framework::Havequick;
pub use builder::HavequickBuilder;
pub use simulator::{
    SimulatorHopper, SimulatorTimeSync, SimulatorNetController,
    SimulatorTransec, AmVoiceCodec, AskDataModem,
};

//! SINCGARS Waveform Implementation
//!
//! This module provides a framework for SINCGARS (Single Channel Ground and Airborne
//! Radio System) frequency hopping implementation. The classified components are
//! abstracted behind traits that must be implemented by authorized parties.
//!
//! # Architecture
//!
//! The implementation separates unclassified framework code from classified algorithms:
//!
//! - **Framework (this module)**: FHSS engine, timing, audio codec, data framing
//! - **Classified (external)**: Hopping algorithm, TRANSEC, crypto
//!
//! # Usage with Simulator
//!
//! ```rust
//! use r4w_core::waveform::sincgars::SincgarsBuilder;
//!
//! // Create a test instance with simulator (non-secure) implementations
//! let sincgars = SincgarsBuilder::simulator().build().unwrap();
//! ```
//!
//! # Usage with Classified Implementations
//!
//! ```rust,ignore
//! use r4w_core::waveform::sincgars::SincgarsBuilder;
//! use my_classified_crate::{ClassifiedHopper, ClassifiedTransec, ...};
//!
//! let sincgars = SincgarsBuilder::new()
//!     .with_hopper(Box::new(ClassifiedHopper::new()))
//!     .with_transec(Box::new(ClassifiedTransec::new()))
//!     // ... other classified components
//!     .build()
//!     .unwrap();
//! ```

mod types;
mod traits;
mod framework;
mod simulator;
mod builder;
mod audio;
mod data;

pub use types::*;
pub use traits::*;
pub use framework::Sincgars;
pub use builder::SincgarsBuilder;
pub use simulator::{
    SimulatorHopper, SimulatorTransec, SimulatorNetMapper,
    SimulatorTimeSync, SimulatorCrypto,
};
pub use audio::CvsdCodec;
pub use data::{SincgarsDataMode, DataFramer};

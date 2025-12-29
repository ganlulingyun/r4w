//! Link-16 Tactical Data Link
//!
//! This module implements the Link-16 (TADIL-J) tactical data link used by NATO
//! forces for sharing situational awareness. Also known as JTIDS (Joint Tactical
//! Information Distribution System) or MIDS (Multifunctional Information
//! Distribution System).
//!
//! # Overview
//!
//! Link-16 provides jam-resistant, secure, high-capacity communications through:
//! - **TDMA**: Time Division Multiple Access with 1536 slots per 12.8 minute frame
//! - **Frequency Hopping**: 51 frequencies in L-band (960-1215 MHz)
//! - **MSK Modulation**: Minimum Shift Keying for spectral efficiency
//! - **Reed-Solomon FEC**: Error correction for reliability
//! - **TRANSEC**: Transmission security encryption
//!
//! # Architecture
//!
//! The implementation follows a trait-based architecture similar to SINCGARS
//! and HAVEQUICK, allowing classified components to be swapped:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                     Link-16 Framework                       │
//! ├─────────────────────────────────────────────────────────────┤
//! │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
//! │  │   Hopping   │  │   TimeSync  │  │    MessageCodec     │  │
//! │  │   Pattern   │  │             │  │   (J-Series Msgs)   │  │
//! │  └─────────────┘  └─────────────┘  └─────────────────────┘  │
//! │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
//! │  │     FEC     │  │ Interleaver │  │      TRANSEC        │  │
//! │  │(Reed-Solomon)│ │             │  │    (Encryption)     │  │
//! │  └─────────────┘  └─────────────┘  └─────────────────────┘  │
//! │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
//! │  │  MSK Modem  │  │   Pulse     │  │  Network Controller │  │
//! │  │             │  │  Formatter  │  │                     │  │
//! │  └─────────────┘  └─────────────┘  └─────────────────────┘  │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # J-Series Messages
//!
//! Link-16 uses standardized J-series messages for different purposes:
//!
//! | Series | Purpose |
//! |--------|---------|
//! | J0 | Network Management |
//! | J2 | PPLI (Precise Participant Location & ID) |
//! | J3 | Tracks (Air, Surface, Land) |
//! | J7 | Track Management |
//! | J12 | Mission Management |
//! | J28 | Free Text |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::waveform::link16::{Link16Builder, Npg, JSeriesMessage};
//!
//! // Create a Link-16 simulator instance
//! let mut link16 = Link16Builder::simulator()
//!     .with_sample_rate(5_000_000.0)
//!     .build()
//!     .expect("Failed to create Link-16");
//!
//! // Join network
//! link16.join_network(1234, 5, b"crypto_variable").unwrap();
//!
//! // Subscribe to surveillance NPG
//! link16.subscribe_npg(Npg::SURVEILLANCE);
//!
//! // Check current slot and frequency
//! let slot = link16.current_slot();
//! let freq = link16.current_frequency();
//! println!("Slot: {}.{}, Freq: {}", slot.epoch, slot.slot, freq);
//! ```
//!
//! # Security Note
//!
//! The actual hopping algorithm and TRANSEC encryption are classified.
//! This implementation provides unclassified simulator components for
//! training and development purposes only.

pub mod builder;
pub mod framework;
pub mod simulator;
pub mod traits;
pub mod types;

pub use builder::Link16Builder;
pub use framework::Link16;
pub use traits::*;
pub use types::*;

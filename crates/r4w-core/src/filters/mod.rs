//! Digital Filters for Signal Processing
//!
//! This module provides filters commonly used in digital communications:
//!
//! - **Pulse Shaping Filters**: Control spectral characteristics
//!   - Raised Cosine (RC)
//!   - Root Raised Cosine (RRC)
//!   - Gaussian
//!
//! ## Why Pulse Shaping?
//!
//! Raw digital pulses (rectangular) have infinite bandwidth due to sharp
//! transitions. Pulse shaping:
//! - Limits bandwidth (spectral efficiency)
//! - Reduces intersymbol interference (ISI)
//! - Improves adjacent channel performance
//!
//! ## Root Raised Cosine (RRC)
//!
//! The most common pulse shaping filter. When RRC is used at both TX and RX,
//! the combined response is Raised Cosine (Nyquist filter = zero ISI).
//!
//! ```text
//! TX: Data → Upsample → RRC Filter → DAC → RF
//! RX: RF → ADC → RRC Filter → Downsample → Data
//!                   ↓
//!           Combined: RC (zero ISI at sample points)
//! ```
//!
//! ## Gaussian Filter
//!
//! Used in GMSK (GSM, Bluetooth). Provides excellent spectral containment
//! but introduces controlled ISI.

pub mod fir;
pub mod pulse_shaping;

pub use fir::FirFilter;
pub use pulse_shaping::{GaussianFilter, PulseShapingFilter, RaisedCosineFilter, RootRaisedCosineFilter};

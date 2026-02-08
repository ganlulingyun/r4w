//! Digital Filters for Signal Processing
//!
//! This module provides a comprehensive filter framework for digital communications:
//!
//! ## Filter Traits
//!
//! - [`Filter`] - Core trait for all digital filters
//! - [`RealFilter`] - Efficient real-valued sample processing
//! - [`FirFilterOps`] - FIR-specific operations (coefficients, linear phase check)
//! - [`FrequencyResponse`] - Magnitude/phase response analysis
//!
//! ## Filter Types
//!
//! - [`FirFilter`] - General-purpose FIR filter with lowpass/highpass/bandpass/bandstop
//! - **Pulse Shaping Filters**: Control spectral characteristics
//!   - [`RaisedCosineFilter`] - Nyquist filter for zero ISI
//!   - [`RootRaisedCosineFilter`] - TX/RX matched filter (most common)
//!   - [`GaussianFilter`] - GMSK (GSM, Bluetooth)
//!
//! ## Architecture
//!
//! ```text
//! Filter (core trait)
//!    ├── FirFilterOps (FIR-specific)
//!    │      └── FrequencyResponse (auto-implemented)
//!    ├── RealFilter (efficient real processing)
//!    └── [future: IirFilterOps]
//!
//! Implementations:
//!    FirFilter ──────┬── Filter
//!                    ├── RealFilter
//!                    └── FirFilterOps
//!
//!    PulseShapingFilter (separate trait for pulse shapers)
//! ```
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
//! ## Example Usage
//!
//! ```rust,ignore
//! use r4w_core::filters::{FirFilter, Filter, FrequencyResponse};
//! use num_complex::Complex64;
//!
//! // Create a lowpass filter
//! let mut lpf = FirFilter::lowpass(1000.0, 8000.0, 63);
//!
//! // Check frequency response
//! let dc_gain = lpf.magnitude_response_db(0.0, 8000.0);
//! let stopband = lpf.magnitude_response_db(3000.0, 8000.0);
//! println!("DC: {:.1} dB, Stopband: {:.1} dB", dc_gain, stopband);
//!
//! // Process samples
//! let input = vec![Complex64::new(1.0, 0.0); 100];
//! let output = lpf.process_block(&input);
//! ```

pub mod fir;
pub mod pulse_shaping;
pub mod traits;

// Re-export core traits
pub use traits::{Filter, FilterResponse, FilterType, FirFilterOps, FrequencyResponse, RealFilter};

// Re-export filter implementations
pub use fir::FirFilter;
pub use pulse_shaping::{GaussianFilter, PulseShapingFilter, RaisedCosineFilter, RootRaisedCosineFilter};

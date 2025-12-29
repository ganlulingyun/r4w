//! # R4W Workshop
//!
//! Educational exercises for learning SDR and waveform development with R4W.
//!
//! ## USRP Track (exercises 01-09)
//!
//! Hands-on exercises for working with USRP hardware:
//! - Device discovery and configuration
//! - Basic RX/TX operations
//! - Loopback testing with attenuators
//! - LoRa transmission and reception
//! - Over-the-air link testing
//! - Timing synchronization (PPS, GPSDO)
//! - Automated sensitivity testing
//!
//! ## Advanced DSP Track (exercises 10-83)
//!
//! Deep dive into DSP and waveform development:
//! - DSP fundamentals (complex numbers, FFT, filtering)
//! - Modulation techniques (PSK, QAM, OFDM, CSS)
//! - Synchronization (symbol timing, carrier recovery)
//! - Channel effects (AWGN, multipath, Doppler)
//! - Error control (CRC, Hamming, convolutional codes)
//! - Performance analysis (BER testing, sensitivity)
//! - Custom waveform implementation
//!
//! ## Running Exercises
//!
//! ```bash
//! # Run with simulator (no hardware needed)
//! cargo run -p r4w-workshop --example 01_device_discovery -- --simulator
//!
//! # Run with USRP hardware
//! cargo run -p r4w-workshop --example 01_device_discovery -- --device "uhd://type=b200"
//! ```

pub use r4w_core;
pub use r4w_sim;

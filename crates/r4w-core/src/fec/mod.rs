//! Forward Error Correction (FEC) Blocks
//!
//! Standalone, reusable FEC implementations for digital communications.
//!
//! ## Available Codecs
//!
//! - [`ConvolutionalEncoder`] + [`ViterbiDecoder`] - Convolutional codes with Viterbi decoding
//!
//! ## Usage
//!
//! ```rust
//! use r4w_core::fec::{ConvolutionalEncoder, ViterbiDecoder, ConvCodeConfig};
//!
//! // NASA standard rate 1/2, K=7 code
//! let config = ConvCodeConfig::nasa_k7_rate_half();
//! let mut encoder = ConvolutionalEncoder::new(config.clone());
//! let mut decoder = ViterbiDecoder::new(config);
//!
//! // Encode
//! let data = vec![true, false, true, true, false, false, true, false];
//! let encoded = encoder.encode(&data);
//!
//! // Decode (hard decision)
//! let decoded = decoder.decode_hard(&encoded, data.len());
//! assert_eq!(decoded, data);
//! ```

pub mod convolutional;

pub use convolutional::{ConvCodeConfig, ConvolutionalEncoder, ViterbiDecoder};

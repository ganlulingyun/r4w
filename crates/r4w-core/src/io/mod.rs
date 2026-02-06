//! I/O utilities for IQ sample data.
//!
//! This module provides unified handling for IQ sample formats across the r4w
//! codebase, including file I/O, UDP streaming, and format conversion.
//!
//! # Overview
//!
//! The primary type is [`IqFormat`], which represents the various binary formats
//! used to store and transmit I/Q samples. It replaces scattered format definitions
//! in sigmf, benchmark, GUI streaming, and CLI code with a single source of truth.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::io::IqFormat;
//! use r4w_core::types::IQSample;
//!
//! // Parse format from CLI argument
//! let format = IqFormat::from_str("sc16").unwrap();
//! assert_eq!(format.bytes_per_sample(), 4);
//!
//! // Write samples to a file
//! let samples = vec![IQSample::new(0.5, -0.5)];
//! let mut file = std::fs::File::create("/tmp/test.iq").unwrap();
//! format.write_samples(&mut file, &samples).unwrap();
//! ```

mod format;

pub use format::IqFormat;

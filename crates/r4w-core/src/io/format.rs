//! Unified IQ sample format handling.
//!
//! This module provides a single source of truth for IQ sample formats across
//! r4w, replacing scattered implementations in sigmf, benchmark, GUI streaming,
//! and CLI code.
//!
//! # Supported Formats
//!
//! | Format | Bytes/Sample | SigMF Type | Description |
//! |--------|--------------|------------|-------------|
//! | Cf64   | 16           | cf64_le    | Complex float64, maximum precision |
//! | Cf32   | 8            | cf32_le    | Complex float32, USRP/GNU Radio compatible |
//! | Ci16   | 4            | ci16_le    | Complex int16, compact storage |
//! | Ci8    | 2            | ci8        | Complex int8, very compact |
//! | Cu8    | 2            | cu8        | Complex uint8, RTL-SDR native format |
//!
//! # Scaling Conventions
//!
//! Integer formats use normalized [-1.0, 1.0] scaling:
//! - Ci16: multiply by 32767, clamp to [-32768, 32767], divide by 32768 on read
//! - Ci8: multiply by 127, clamp to [-128, 127], divide by 128 on read
//! - Cu8: shift by 127.5 for unsigned representation (RTL-SDR convention)
//!
//! # Example
//!
//! ```rust
//! use r4w_core::io::IqFormat;
//! use r4w_core::types::IQSample;
//!
//! // Parse format from CLI string
//! let format = IqFormat::from_str("ettus").unwrap();
//! assert_eq!(format, IqFormat::Cf32);
//! assert_eq!(format.bytes_per_sample(), 8);
//!
//! // Write samples
//! let samples = vec![IQSample::new(0.5, -0.5)];
//! let mut buffer = Vec::new();
//! format.write_samples(&mut buffer, &samples).unwrap();
//! ```

use crate::types::IQSample;
use std::io::{self, Read, Write};

/// Unified IQ sample format enum.
///
/// This replaces scattered format definitions across the codebase and provides
/// consistent handling for file I/O, UDP streaming, and format conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum IqFormat {
    /// Complex float64 (16 bytes/sample, SigMF "cf64_le")
    /// Maximum precision, default for internal processing
    Cf64,

    /// Complex float32 (8 bytes/sample, SigMF "cf32_le")
    /// USRP/Ettus and GNU Radio compatible format
    #[default]
    Cf32,

    /// Complex signed int16 (4 bytes/sample, SigMF "ci16_le")
    /// Compact storage, common for SDR hardware
    Ci16,

    /// Complex signed int8 (2 bytes/sample, SigMF "ci8")
    /// Very compact storage
    Ci8,

    /// Complex unsigned int8 (2 bytes/sample, SigMF "cu8")
    /// RTL-SDR native format, DC offset at 127.5
    Cu8,
}

impl IqFormat {
    /// Returns the size of one I/Q sample in bytes.
    ///
    /// This includes both the I and Q components.
    #[inline]
    pub const fn bytes_per_sample(&self) -> usize {
        match self {
            IqFormat::Cf64 => 16, // 8 bytes I + 8 bytes Q
            IqFormat::Cf32 => 8,  // 4 bytes I + 4 bytes Q
            IqFormat::Ci16 => 4,  // 2 bytes I + 2 bytes Q
            IqFormat::Ci8 => 2,   // 1 byte I + 1 byte Q
            IqFormat::Cu8 => 2,   // 1 byte I + 1 byte Q
        }
    }

    /// Returns the SigMF datatype string for this format.
    ///
    /// This is used in SigMF metadata files and follows the SigMF specification.
    pub const fn sigmf_datatype(&self) -> &'static str {
        match self {
            IqFormat::Cf64 => "cf64_le",
            IqFormat::Cf32 => "cf32_le",
            IqFormat::Ci16 => "ci16_le",
            IqFormat::Ci8 => "ci8",
            IqFormat::Cu8 => "cu8",
        }
    }

    /// Returns a human-readable display name for the format.
    pub const fn display_name(&self) -> &'static str {
        match self {
            IqFormat::Cf64 => "cf64 (Complex Float64)",
            IqFormat::Cf32 => "cf32 (Complex Float32)",
            IqFormat::Ci16 => "ci16 (Complex Signed Int16)",
            IqFormat::Ci8 => "ci8 (Complex Signed Int8)",
            IqFormat::Cu8 => "cu8 (Complex Unsigned Int8)",
        }
    }

    /// Returns a short name for the format (for CLI output, etc.).
    pub const fn short_name(&self) -> &'static str {
        match self {
            IqFormat::Cf64 => "cf64",
            IqFormat::Cf32 => "cf32",
            IqFormat::Ci16 => "ci16",
            IqFormat::Ci8 => "ci8",
            IqFormat::Cu8 => "cu8",
        }
    }

    /// Parse a format from a string, supporting many common aliases.
    ///
    /// # Supported Aliases
    ///
    /// | Format | Aliases |
    /// |--------|---------|
    /// | Cf64   | f64, cf64, cf64_le, complex64 |
    /// | Cf32   | f32, cf32, cf32_le, ettus, float32, float |
    /// | Ci16   | i16, ci16, ci16_le, sc16, int16, short |
    /// | Ci8    | i8, ci8, int8 |
    /// | Cu8    | u8, cu8, uint8, rtlsdr |
    ///
    /// # Returns
    ///
    /// `Some(IqFormat)` if the string matches a known format, `None` otherwise.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            // Cf64 aliases
            "f64" | "cf64" | "cf64_le" | "complex64" => Some(IqFormat::Cf64),

            // Cf32 aliases (most common)
            "f32" | "cf32" | "cf32_le" | "ettus" | "float32" | "float" => Some(IqFormat::Cf32),

            // Ci16 aliases
            "i16" | "ci16" | "ci16_le" | "sc16" | "int16" | "short" => Some(IqFormat::Ci16),

            // Ci8 aliases
            "i8" | "ci8" | "int8" => Some(IqFormat::Ci8),

            // Cu8 aliases
            "u8" | "cu8" | "uint8" | "rtlsdr" => Some(IqFormat::Cu8),

            _ => None,
        }
    }

    /// Parse a format from a SigMF datatype string.
    ///
    /// This is the inverse of `sigmf_datatype()`.
    pub fn from_sigmf_datatype(s: &str) -> Option<Self> {
        match s {
            "cf64_le" | "cf64" => Some(IqFormat::Cf64),
            "cf32_le" | "cf32" => Some(IqFormat::Cf32),
            "ci16_le" | "ci16" => Some(IqFormat::Ci16),
            "ci8" => Some(IqFormat::Ci8),
            "cu8" => Some(IqFormat::Cu8),
            _ => None,
        }
    }

    /// Returns all supported format variants.
    pub const fn all() -> &'static [IqFormat] {
        &[
            IqFormat::Cf64,
            IqFormat::Cf32,
            IqFormat::Ci16,
            IqFormat::Ci8,
            IqFormat::Cu8,
        ]
    }

    /// Write a single I/Q sample to a writer in this format.
    ///
    /// # Arguments
    /// * `writer` - The output writer
    /// * `sample` - The I/Q sample to write
    ///
    /// # Errors
    /// Returns an IO error if writing fails.
    pub fn write_sample<W: Write>(&self, writer: &mut W, sample: &IQSample) -> io::Result<()> {
        match self {
            IqFormat::Cf64 => {
                writer.write_all(&sample.re.to_le_bytes())?;
                writer.write_all(&sample.im.to_le_bytes())?;
            }
            IqFormat::Cf32 => {
                writer.write_all(&(sample.re as f32).to_le_bytes())?;
                writer.write_all(&(sample.im as f32).to_le_bytes())?;
            }
            IqFormat::Ci16 => {
                // Scale: [-1.0, 1.0] -> [-32767, 32767], clamp to i16 range
                let re = (sample.re * 32767.0).clamp(-32768.0, 32767.0) as i16;
                let im = (sample.im * 32767.0).clamp(-32768.0, 32767.0) as i16;
                writer.write_all(&re.to_le_bytes())?;
                writer.write_all(&im.to_le_bytes())?;
            }
            IqFormat::Ci8 => {
                // Scale: [-1.0, 1.0] -> [-127, 127], clamp to i8 range
                let re = (sample.re * 127.0).clamp(-128.0, 127.0) as i8;
                let im = (sample.im * 127.0).clamp(-128.0, 127.0) as i8;
                writer.write_all(&re.to_le_bytes())?;
                writer.write_all(&im.to_le_bytes())?;
            }
            IqFormat::Cu8 => {
                // RTL-SDR convention: shift [-1.0, 1.0] to [0, 255] with DC at 127.5
                let re = ((sample.re + 1.0) * 127.5).clamp(0.0, 255.0) as u8;
                let im = ((sample.im + 1.0) * 127.5).clamp(0.0, 255.0) as u8;
                writer.write_all(&[re])?;
                writer.write_all(&[im])?;
            }
        }
        Ok(())
    }

    /// Read a single I/Q sample from a reader in this format.
    ///
    /// # Arguments
    /// * `reader` - The input reader
    ///
    /// # Returns
    /// The decoded I/Q sample, or an IO error if reading fails.
    pub fn read_sample<R: Read>(&self, reader: &mut R) -> io::Result<IQSample> {
        match self {
            IqFormat::Cf64 => {
                let mut buf = [0u8; 16];
                reader.read_exact(&mut buf)?;
                let re = f64::from_le_bytes(buf[0..8].try_into().unwrap());
                let im = f64::from_le_bytes(buf[8..16].try_into().unwrap());
                Ok(IQSample::new(re, im))
            }
            IqFormat::Cf32 => {
                let mut buf = [0u8; 8];
                reader.read_exact(&mut buf)?;
                let re = f32::from_le_bytes(buf[0..4].try_into().unwrap()) as f64;
                let im = f32::from_le_bytes(buf[4..8].try_into().unwrap()) as f64;
                Ok(IQSample::new(re, im))
            }
            IqFormat::Ci16 => {
                let mut buf = [0u8; 4];
                reader.read_exact(&mut buf)?;
                let re = i16::from_le_bytes(buf[0..2].try_into().unwrap());
                let im = i16::from_le_bytes(buf[2..4].try_into().unwrap());
                // Scale: [-32768, 32767] -> [-1.0, 1.0]
                Ok(IQSample::new(re as f64 / 32768.0, im as f64 / 32768.0))
            }
            IqFormat::Ci8 => {
                let mut buf = [0u8; 2];
                reader.read_exact(&mut buf)?;
                let re = buf[0] as i8;
                let im = buf[1] as i8;
                // Scale: [-128, 127] -> [-1.0, 1.0]
                Ok(IQSample::new(re as f64 / 128.0, im as f64 / 128.0))
            }
            IqFormat::Cu8 => {
                let mut buf = [0u8; 2];
                reader.read_exact(&mut buf)?;
                // RTL-SDR convention: [0, 255] with DC at 127.5 -> [-1.0, 1.0]
                let re = (buf[0] as f64 - 127.5) / 127.5;
                let im = (buf[1] as f64 - 127.5) / 127.5;
                Ok(IQSample::new(re, im))
            }
        }
    }

    /// Write multiple I/Q samples to a writer in this format.
    ///
    /// This is more efficient than calling `write_sample` in a loop as it
    /// reduces the number of individual write calls.
    ///
    /// # Arguments
    /// * `writer` - The output writer
    /// * `samples` - The I/Q samples to write
    ///
    /// # Returns
    /// The number of bytes written, or an IO error if writing fails.
    pub fn write_samples<W: Write>(&self, writer: &mut W, samples: &[IQSample]) -> io::Result<usize> {
        let bytes_per_sample = self.bytes_per_sample();

        for sample in samples {
            self.write_sample(writer, sample)?;
        }

        Ok(samples.len() * bytes_per_sample)
    }

    /// Read multiple I/Q samples from a reader in this format.
    ///
    /// # Arguments
    /// * `reader` - The input reader
    /// * `count` - The number of samples to read
    ///
    /// # Returns
    /// A vector of decoded I/Q samples, or an IO error if reading fails.
    pub fn read_samples<R: Read>(&self, reader: &mut R, count: usize) -> io::Result<Vec<IQSample>> {
        let mut samples = Vec::with_capacity(count);
        for _ in 0..count {
            samples.push(self.read_sample(reader)?);
        }
        Ok(samples)
    }

    /// Convert raw bytes to I/Q samples.
    ///
    /// This is useful for parsing UDP packets or memory-mapped files.
    ///
    /// # Arguments
    /// * `data` - The raw byte buffer
    ///
    /// # Returns
    /// A vector of decoded I/Q samples.
    pub fn parse_bytes(&self, data: &[u8]) -> Vec<IQSample> {
        let bytes_per_sample = self.bytes_per_sample();
        let num_samples = data.len() / bytes_per_sample;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * bytes_per_sample;
            let sample = match self {
                IqFormat::Cf64 => {
                    let re = f64::from_le_bytes(data[offset..offset + 8].try_into().unwrap());
                    let im = f64::from_le_bytes(data[offset + 8..offset + 16].try_into().unwrap());
                    IQSample::new(re, im)
                }
                IqFormat::Cf32 => {
                    let re = f32::from_le_bytes(data[offset..offset + 4].try_into().unwrap()) as f64;
                    let im = f32::from_le_bytes(data[offset + 4..offset + 8].try_into().unwrap()) as f64;
                    IQSample::new(re, im)
                }
                IqFormat::Ci16 => {
                    let re = i16::from_le_bytes(data[offset..offset + 2].try_into().unwrap());
                    let im = i16::from_le_bytes(data[offset + 2..offset + 4].try_into().unwrap());
                    IQSample::new(re as f64 / 32768.0, im as f64 / 32768.0)
                }
                IqFormat::Ci8 => {
                    let re = data[offset] as i8;
                    let im = data[offset + 1] as i8;
                    IQSample::new(re as f64 / 128.0, im as f64 / 128.0)
                }
                IqFormat::Cu8 => {
                    let re = (data[offset] as f64 - 127.5) / 127.5;
                    let im = (data[offset + 1] as f64 - 127.5) / 127.5;
                    IQSample::new(re, im)
                }
            };
            samples.push(sample);
        }

        samples
    }

    /// Convert I/Q samples to raw bytes.
    ///
    /// This is useful for creating UDP packets or writing to memory-mapped files.
    ///
    /// # Arguments
    /// * `samples` - The I/Q samples to convert
    ///
    /// # Returns
    /// A vector of raw bytes.
    pub fn to_bytes(&self, samples: &[IQSample]) -> Vec<u8> {
        let bytes_per_sample = self.bytes_per_sample();
        let mut data = Vec::with_capacity(samples.len() * bytes_per_sample);

        for sample in samples {
            match self {
                IqFormat::Cf64 => {
                    data.extend_from_slice(&sample.re.to_le_bytes());
                    data.extend_from_slice(&sample.im.to_le_bytes());
                }
                IqFormat::Cf32 => {
                    data.extend_from_slice(&(sample.re as f32).to_le_bytes());
                    data.extend_from_slice(&(sample.im as f32).to_le_bytes());
                }
                IqFormat::Ci16 => {
                    let re = (sample.re * 32767.0).clamp(-32768.0, 32767.0) as i16;
                    let im = (sample.im * 32767.0).clamp(-32768.0, 32767.0) as i16;
                    data.extend_from_slice(&re.to_le_bytes());
                    data.extend_from_slice(&im.to_le_bytes());
                }
                IqFormat::Ci8 => {
                    let re = (sample.re * 127.0).clamp(-128.0, 127.0) as i8;
                    let im = (sample.im * 127.0).clamp(-128.0, 127.0) as i8;
                    data.push(re as u8);
                    data.push(im as u8);
                }
                IqFormat::Cu8 => {
                    let re = ((sample.re + 1.0) * 127.5).clamp(0.0, 255.0) as u8;
                    let im = ((sample.im + 1.0) * 127.5).clamp(0.0, 255.0) as u8;
                    data.push(re);
                    data.push(im);
                }
            }
        }

        data
    }
}

impl std::fmt::Display for IqFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.short_name())
    }
}

impl std::str::FromStr for IqFormat {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        IqFormat::from_str(s).ok_or_else(|| {
            format!(
                "Unknown IQ format '{}'. Valid formats: f64, f32/ettus, sc16/ci16, ci8, cu8/rtlsdr",
                s
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_bytes_per_sample() {
        assert_eq!(IqFormat::Cf64.bytes_per_sample(), 16);
        assert_eq!(IqFormat::Cf32.bytes_per_sample(), 8);
        assert_eq!(IqFormat::Ci16.bytes_per_sample(), 4);
        assert_eq!(IqFormat::Ci8.bytes_per_sample(), 2);
        assert_eq!(IqFormat::Cu8.bytes_per_sample(), 2);
    }

    #[test]
    fn test_sigmf_datatype() {
        assert_eq!(IqFormat::Cf64.sigmf_datatype(), "cf64_le");
        assert_eq!(IqFormat::Cf32.sigmf_datatype(), "cf32_le");
        assert_eq!(IqFormat::Ci16.sigmf_datatype(), "ci16_le");
        assert_eq!(IqFormat::Ci8.sigmf_datatype(), "ci8");
        assert_eq!(IqFormat::Cu8.sigmf_datatype(), "cu8");
    }

    #[test]
    fn test_from_str_aliases() {
        // Cf64 aliases
        assert_eq!(IqFormat::from_str("f64"), Some(IqFormat::Cf64));
        assert_eq!(IqFormat::from_str("cf64"), Some(IqFormat::Cf64));
        assert_eq!(IqFormat::from_str("CF64"), Some(IqFormat::Cf64));
        assert_eq!(IqFormat::from_str("complex64"), Some(IqFormat::Cf64));

        // Cf32 aliases
        assert_eq!(IqFormat::from_str("f32"), Some(IqFormat::Cf32));
        assert_eq!(IqFormat::from_str("cf32"), Some(IqFormat::Cf32));
        assert_eq!(IqFormat::from_str("ettus"), Some(IqFormat::Cf32));
        assert_eq!(IqFormat::from_str("ETTUS"), Some(IqFormat::Cf32));
        assert_eq!(IqFormat::from_str("float32"), Some(IqFormat::Cf32));
        assert_eq!(IqFormat::from_str("float"), Some(IqFormat::Cf32));

        // Ci16 aliases
        assert_eq!(IqFormat::from_str("i16"), Some(IqFormat::Ci16));
        assert_eq!(IqFormat::from_str("ci16"), Some(IqFormat::Ci16));
        assert_eq!(IqFormat::from_str("sc16"), Some(IqFormat::Ci16));
        assert_eq!(IqFormat::from_str("SC16"), Some(IqFormat::Ci16));
        assert_eq!(IqFormat::from_str("int16"), Some(IqFormat::Ci16));
        assert_eq!(IqFormat::from_str("short"), Some(IqFormat::Ci16));

        // Ci8 aliases
        assert_eq!(IqFormat::from_str("i8"), Some(IqFormat::Ci8));
        assert_eq!(IqFormat::from_str("ci8"), Some(IqFormat::Ci8));
        assert_eq!(IqFormat::from_str("int8"), Some(IqFormat::Ci8));

        // Cu8 aliases
        assert_eq!(IqFormat::from_str("u8"), Some(IqFormat::Cu8));
        assert_eq!(IqFormat::from_str("cu8"), Some(IqFormat::Cu8));
        assert_eq!(IqFormat::from_str("uint8"), Some(IqFormat::Cu8));
        assert_eq!(IqFormat::from_str("rtlsdr"), Some(IqFormat::Cu8));
        assert_eq!(IqFormat::from_str("RTLSDR"), Some(IqFormat::Cu8));

        // Invalid
        assert_eq!(IqFormat::from_str("invalid"), None);
        assert_eq!(IqFormat::from_str(""), None);
    }

    #[test]
    fn test_from_sigmf_datatype() {
        assert_eq!(IqFormat::from_sigmf_datatype("cf64_le"), Some(IqFormat::Cf64));
        assert_eq!(IqFormat::from_sigmf_datatype("cf32_le"), Some(IqFormat::Cf32));
        assert_eq!(IqFormat::from_sigmf_datatype("ci16_le"), Some(IqFormat::Ci16));
        assert_eq!(IqFormat::from_sigmf_datatype("ci8"), Some(IqFormat::Ci8));
        assert_eq!(IqFormat::from_sigmf_datatype("cu8"), Some(IqFormat::Cu8));
        assert_eq!(IqFormat::from_sigmf_datatype("unknown"), None);
    }

    #[test]
    fn test_roundtrip_cf64() {
        let original = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(-1.0, 1.0),
            IQSample::new(0.0, 0.0),
        ];

        let mut buffer = Vec::new();
        IqFormat::Cf64.write_samples(&mut buffer, &original).unwrap();

        let mut cursor = Cursor::new(buffer);
        let decoded = IqFormat::Cf64.read_samples(&mut cursor, original.len()).unwrap();

        for (orig, dec) in original.iter().zip(decoded.iter()) {
            assert!((orig.re - dec.re).abs() < 1e-10);
            assert!((orig.im - dec.im).abs() < 1e-10);
        }
    }

    #[test]
    fn test_roundtrip_cf32() {
        let original = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(-1.0, 1.0),
            IQSample::new(0.0, 0.0),
        ];

        let mut buffer = Vec::new();
        IqFormat::Cf32.write_samples(&mut buffer, &original).unwrap();

        let mut cursor = Cursor::new(buffer);
        let decoded = IqFormat::Cf32.read_samples(&mut cursor, original.len()).unwrap();

        for (orig, dec) in original.iter().zip(decoded.iter()) {
            // f32 precision is ~7 decimal digits
            assert!((orig.re - dec.re).abs() < 1e-6);
            assert!((orig.im - dec.im).abs() < 1e-6);
        }
    }

    #[test]
    fn test_roundtrip_ci16() {
        let original = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(-1.0, 1.0),
            IQSample::new(0.0, 0.0),
        ];

        let mut buffer = Vec::new();
        IqFormat::Ci16.write_samples(&mut buffer, &original).unwrap();

        let mut cursor = Cursor::new(buffer);
        let decoded = IqFormat::Ci16.read_samples(&mut cursor, original.len()).unwrap();

        for (orig, dec) in original.iter().zip(decoded.iter()) {
            // i16 precision is ~1/32768 ≈ 3e-5
            assert!((orig.re - dec.re).abs() < 1e-4, "re: {} vs {}", orig.re, dec.re);
            assert!((orig.im - dec.im).abs() < 1e-4, "im: {} vs {}", orig.im, dec.im);
        }
    }

    #[test]
    fn test_roundtrip_ci8() {
        let original = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(-1.0, 1.0),
            IQSample::new(0.0, 0.0),
        ];

        let mut buffer = Vec::new();
        IqFormat::Ci8.write_samples(&mut buffer, &original).unwrap();

        let mut cursor = Cursor::new(buffer);
        let decoded = IqFormat::Ci8.read_samples(&mut cursor, original.len()).unwrap();

        for (orig, dec) in original.iter().zip(decoded.iter()) {
            // i8 precision is ~1/128 ≈ 0.008
            assert!((orig.re - dec.re).abs() < 0.02, "re: {} vs {}", orig.re, dec.re);
            assert!((orig.im - dec.im).abs() < 0.02, "im: {} vs {}", orig.im, dec.im);
        }
    }

    #[test]
    fn test_roundtrip_cu8() {
        let original = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(-1.0, 1.0),
            IQSample::new(0.0, 0.0),
        ];

        let mut buffer = Vec::new();
        IqFormat::Cu8.write_samples(&mut buffer, &original).unwrap();

        let mut cursor = Cursor::new(buffer);
        let decoded = IqFormat::Cu8.read_samples(&mut cursor, original.len()).unwrap();

        for (orig, dec) in original.iter().zip(decoded.iter()) {
            // u8 precision is ~1/127.5 ≈ 0.008
            assert!((orig.re - dec.re).abs() < 0.02, "re: {} vs {}", orig.re, dec.re);
            assert!((orig.im - dec.im).abs() < 0.02, "im: {} vs {}", orig.im, dec.im);
        }
    }

    #[test]
    fn test_parse_bytes() {
        let samples = vec![
            IQSample::new(0.25, -0.25),
            IQSample::new(0.75, -0.75),
        ];

        for format in IqFormat::all() {
            let bytes = format.to_bytes(&samples);
            let parsed = format.parse_bytes(&bytes);

            assert_eq!(parsed.len(), samples.len(), "format: {:?}", format);

            let tolerance = match format {
                IqFormat::Cf64 => 1e-10,
                IqFormat::Cf32 => 1e-6,
                IqFormat::Ci16 => 1e-4,
                IqFormat::Ci8 | IqFormat::Cu8 => 0.02,
            };

            for (orig, dec) in samples.iter().zip(parsed.iter()) {
                assert!(
                    (orig.re - dec.re).abs() < tolerance,
                    "format {:?}: re {} vs {}",
                    format,
                    orig.re,
                    dec.re
                );
                assert!(
                    (orig.im - dec.im).abs() < tolerance,
                    "format {:?}: im {} vs {}",
                    format,
                    orig.im,
                    dec.im
                );
            }
        }
    }

    #[test]
    fn test_clamping() {
        // Test that values outside [-1.0, 1.0] are clamped properly
        let samples = vec![IQSample::new(2.0, -3.0)];

        // Ci16: should clamp to [-32768, 32767]
        let bytes = IqFormat::Ci16.to_bytes(&samples);
        let parsed = IqFormat::Ci16.parse_bytes(&bytes);
        assert!(parsed[0].re > 0.99, "Ci16 re clamped high: {}", parsed[0].re);
        assert!(parsed[0].im < -0.99, "Ci16 im clamped low: {}", parsed[0].im);

        // Cu8: should clamp to [0, 255]
        let bytes = IqFormat::Cu8.to_bytes(&samples);
        let parsed = IqFormat::Cu8.parse_bytes(&bytes);
        assert!(parsed[0].re > 0.99, "Cu8 re clamped high: {}", parsed[0].re);
        assert!(parsed[0].im < -0.99, "Cu8 im clamped low: {}", parsed[0].im);
    }

    #[test]
    fn test_display() {
        assert_eq!(format!("{}", IqFormat::Cf64), "cf64");
        assert_eq!(format!("{}", IqFormat::Cf32), "cf32");
        assert_eq!(format!("{}", IqFormat::Ci16), "ci16");
        assert_eq!(format!("{}", IqFormat::Ci8), "ci8");
        assert_eq!(format!("{}", IqFormat::Cu8), "cu8");
    }

    #[test]
    fn test_from_str_trait() {
        let fmt: IqFormat = "ettus".parse().unwrap();
        assert_eq!(fmt, IqFormat::Cf32);

        let result: Result<IqFormat, _> = "invalid".parse();
        assert!(result.is_err());
    }

    #[test]
    fn test_default() {
        assert_eq!(IqFormat::default(), IqFormat::Cf32);
    }
}

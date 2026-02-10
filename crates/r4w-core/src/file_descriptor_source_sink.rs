//! # File Descriptor Source and Sink
//!
//! Read and write sample data through file descriptors, enabling
//! pipe-based I/O for integration with external tools (e.g., piping
//! from `rtl_sdr`, `hackrf_transfer`, or to `sox`/`ffmpeg`).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::file_descriptor_source_sink::{FdSink, SampleFormat};
//!
//! let mut sink = FdSink::new(SampleFormat::F64);
//! let samples = vec![(1.0, 0.0), (0.0, 1.0)];
//! let bytes = sink.encode(&samples);
//! assert_eq!(bytes.len(), 32); // 2 samples * 2 components * 8 bytes
//! ```

/// Sample format for I/O.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SampleFormat {
    /// 64-bit float (f64), interleaved I/Q.
    F64,
    /// 32-bit float (f32), interleaved I/Q (Ettus/USRP format).
    F32,
    /// 16-bit signed integer, interleaved I/Q (sc16).
    I16,
    /// 8-bit unsigned integer, interleaved I/Q (RTL-SDR cu8).
    U8,
    /// 8-bit signed integer, interleaved I/Q.
    I8,
}

impl SampleFormat {
    /// Bytes per I/Q sample pair.
    pub fn bytes_per_sample(&self) -> usize {
        match self {
            SampleFormat::F64 => 16,
            SampleFormat::F32 => 8,
            SampleFormat::I16 => 4,
            SampleFormat::U8 => 2,
            SampleFormat::I8 => 2,
        }
    }

    /// Name string.
    pub fn name(&self) -> &str {
        match self {
            SampleFormat::F64 => "cf64",
            SampleFormat::F32 => "cf32",
            SampleFormat::I16 => "ci16",
            SampleFormat::U8 => "cu8",
            SampleFormat::I8 => "ci8",
        }
    }
}

/// File descriptor sink (encoder).
#[derive(Debug, Clone)]
pub struct FdSink {
    format: SampleFormat,
    total_samples: u64,
}

impl FdSink {
    /// Create a new FD sink with the given format.
    pub fn new(format: SampleFormat) -> Self {
        Self {
            format,
            total_samples: 0,
        }
    }

    /// Encode complex IQ samples to bytes.
    pub fn encode(&mut self, samples: &[(f64, f64)]) -> Vec<u8> {
        self.total_samples += samples.len() as u64;

        match self.format {
            SampleFormat::F64 => {
                let mut bytes = Vec::with_capacity(samples.len() * 16);
                for &(i, q) in samples {
                    bytes.extend_from_slice(&i.to_le_bytes());
                    bytes.extend_from_slice(&q.to_le_bytes());
                }
                bytes
            }
            SampleFormat::F32 => {
                let mut bytes = Vec::with_capacity(samples.len() * 8);
                for &(i, q) in samples {
                    bytes.extend_from_slice(&(i as f32).to_le_bytes());
                    bytes.extend_from_slice(&(q as f32).to_le_bytes());
                }
                bytes
            }
            SampleFormat::I16 => {
                let mut bytes = Vec::with_capacity(samples.len() * 4);
                for &(i, q) in samples {
                    let si = (i * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
                    let sq = (q * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
                    bytes.extend_from_slice(&si.to_le_bytes());
                    bytes.extend_from_slice(&sq.to_le_bytes());
                }
                bytes
            }
            SampleFormat::U8 => {
                let mut bytes = Vec::with_capacity(samples.len() * 2);
                for &(i, q) in samples {
                    let ui = (i * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8;
                    let uq = (q * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8;
                    bytes.push(ui);
                    bytes.push(uq);
                }
                bytes
            }
            SampleFormat::I8 => {
                let mut bytes = Vec::with_capacity(samples.len() * 2);
                for &(i, q) in samples {
                    let si = (i * 127.0).round().clamp(-128.0, 127.0) as i8;
                    let sq = (q * 127.0).round().clamp(-128.0, 127.0) as i8;
                    bytes.push(si as u8);
                    bytes.push(sq as u8);
                }
                bytes
            }
        }
    }

    /// Get total samples encoded.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get the format.
    pub fn format(&self) -> SampleFormat {
        self.format
    }
}

/// File descriptor source (decoder).
#[derive(Debug, Clone)]
pub struct FdSource {
    format: SampleFormat,
    total_samples: u64,
}

impl FdSource {
    /// Create a new FD source with the given format.
    pub fn new(format: SampleFormat) -> Self {
        Self {
            format,
            total_samples: 0,
        }
    }

    /// Decode bytes to complex IQ samples.
    pub fn decode(&mut self, bytes: &[u8]) -> Vec<(f64, f64)> {
        let bps = self.format.bytes_per_sample();
        let num_samples = bytes.len() / bps;
        let mut samples = Vec::with_capacity(num_samples);

        for idx in 0..num_samples {
            let offset = idx * bps;
            let sample = match self.format {
                SampleFormat::F64 => {
                    let i = f64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap());
                    let q = f64::from_le_bytes(bytes[offset + 8..offset + 16].try_into().unwrap());
                    (i, q)
                }
                SampleFormat::F32 => {
                    let i = f32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
                    let q = f32::from_le_bytes(bytes[offset + 4..offset + 8].try_into().unwrap());
                    (i as f64, q as f64)
                }
                SampleFormat::I16 => {
                    let i = i16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap());
                    let q = i16::from_le_bytes(bytes[offset + 2..offset + 4].try_into().unwrap());
                    (i as f64 / 32768.0, q as f64 / 32768.0)
                }
                SampleFormat::U8 => {
                    let i = bytes[offset];
                    let q = bytes[offset + 1];
                    ((i as f64 - 127.5) / 127.5, (q as f64 - 127.5) / 127.5)
                }
                SampleFormat::I8 => {
                    let i = bytes[offset] as i8;
                    let q = bytes[offset + 1] as i8;
                    (i as f64 / 128.0, q as f64 / 128.0)
                }
            };
            samples.push(sample);
        }

        self.total_samples += num_samples as u64;
        samples
    }

    /// Get total samples decoded.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get the format.
    pub fn format(&self) -> SampleFormat {
        self.format
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_f64_roundtrip() {
        let mut sink = FdSink::new(SampleFormat::F64);
        let mut source = FdSource::new(SampleFormat::F64);
        let samples = vec![(0.5, -0.3), (0.0, 1.0)];
        let bytes = sink.encode(&samples);
        let decoded = source.decode(&bytes);
        assert_eq!(decoded.len(), 2);
        assert!((decoded[0].0 - 0.5).abs() < 1e-10);
        assert!((decoded[0].1 + 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_f32_roundtrip() {
        let mut sink = FdSink::new(SampleFormat::F32);
        let mut source = FdSource::new(SampleFormat::F32);
        let samples = vec![(0.5, -0.5)];
        let bytes = sink.encode(&samples);
        assert_eq!(bytes.len(), 8);
        let decoded = source.decode(&bytes);
        assert!((decoded[0].0 - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_i16_roundtrip() {
        let mut sink = FdSink::new(SampleFormat::I16);
        let mut source = FdSource::new(SampleFormat::I16);
        let samples = vec![(0.5, -0.5)];
        let bytes = sink.encode(&samples);
        assert_eq!(bytes.len(), 4);
        let decoded = source.decode(&bytes);
        assert!((decoded[0].0 - 0.5).abs() < 0.001);
        assert!((decoded[0].1 + 0.5).abs() < 0.001);
    }

    #[test]
    fn test_u8_roundtrip() {
        let mut sink = FdSink::new(SampleFormat::U8);
        let mut source = FdSource::new(SampleFormat::U8);
        let samples = vec![(0.0, 0.0)];
        let bytes = sink.encode(&samples);
        assert_eq!(bytes.len(), 2);
        let decoded = source.decode(&bytes);
        assert!(decoded[0].0.abs() < 0.02);
        assert!(decoded[0].1.abs() < 0.02);
    }

    #[test]
    fn test_i8_roundtrip() {
        let mut sink = FdSink::new(SampleFormat::I8);
        let mut source = FdSource::new(SampleFormat::I8);
        let samples = vec![(0.5, -0.25)];
        let bytes = sink.encode(&samples);
        assert_eq!(bytes.len(), 2);
        let decoded = source.decode(&bytes);
        assert!((decoded[0].0 - 0.5).abs() < 0.02);
    }

    #[test]
    fn test_bytes_per_sample() {
        assert_eq!(SampleFormat::F64.bytes_per_sample(), 16);
        assert_eq!(SampleFormat::F32.bytes_per_sample(), 8);
        assert_eq!(SampleFormat::I16.bytes_per_sample(), 4);
        assert_eq!(SampleFormat::U8.bytes_per_sample(), 2);
        assert_eq!(SampleFormat::I8.bytes_per_sample(), 2);
    }

    #[test]
    fn test_format_name() {
        assert_eq!(SampleFormat::F64.name(), "cf64");
        assert_eq!(SampleFormat::U8.name(), "cu8");
    }

    #[test]
    fn test_total_samples() {
        let mut sink = FdSink::new(SampleFormat::F64);
        sink.encode(&[(1.0, 0.0), (0.0, 1.0)]);
        assert_eq!(sink.total_samples(), 2);
        sink.encode(&[(1.0, 0.0)]);
        assert_eq!(sink.total_samples(), 3);
    }

    #[test]
    fn test_empty() {
        let mut sink = FdSink::new(SampleFormat::F64);
        let bytes = sink.encode(&[]);
        assert!(bytes.is_empty());
        let mut source = FdSource::new(SampleFormat::F64);
        let decoded = source.decode(&[]);
        assert!(decoded.is_empty());
    }

    #[test]
    fn test_multiple_samples_f32() {
        let mut sink = FdSink::new(SampleFormat::F32);
        let mut source = FdSource::new(SampleFormat::F32);
        let samples: Vec<(f64, f64)> = (0..10).map(|i| (i as f64 * 0.1, -i as f64 * 0.1)).collect();
        let bytes = sink.encode(&samples);
        assert_eq!(bytes.len(), 80);
        let decoded = source.decode(&bytes);
        assert_eq!(decoded.len(), 10);
    }
}

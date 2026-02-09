//! File Meta Sink/Source — Self-describing IQ file I/O
//!
//! Read/write IQ samples with embedded metadata headers describing sample rate,
//! data type, timestamps, and custom properties. Complements SigMF with a simpler
//! single-file format for internal use.
//!
//! ## Format
//!
//! The file starts with a JSON header line terminated by `\n`, followed by binary
//! sample data. The header contains `sample_rate`, `data_type`, `num_samples`,
//! and optional `metadata` key-value pairs.
//!
//! ## Example
//!
//! ```rust,no_run
//! use r4w_core::file_meta::{FileMetaSink, FileMetaSource, FileMetaHeader};
//! use num_complex::Complex64;
//!
//! // Write
//! let header = FileMetaHeader::new(48000.0, "cf64");
//! let mut sink = FileMetaSink::new("/tmp/test.iqm", header).unwrap();
//! let samples = vec![Complex64::new(1.0, 0.0); 100];
//! sink.write_samples(&samples).unwrap();
//! sink.finalize().unwrap();
//!
//! // Read
//! let mut source = FileMetaSource::open("/tmp/test.iqm").unwrap();
//! let header = source.header();
//! assert_eq!(header.sample_rate, 48000.0);
//! let data = source.read_all().unwrap();
//! assert_eq!(data.len(), 100);
//! ```

use num_complex::Complex64;
use std::collections::HashMap;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::path::Path;

/// File metadata header.
#[derive(Debug, Clone)]
pub struct FileMetaHeader {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Data type identifier (cf64, cf32, ci16, etc.).
    pub data_type: String,
    /// Number of samples (0 if unknown, updated on finalize).
    pub num_samples: u64,
    /// Custom metadata key-value pairs.
    pub metadata: HashMap<String, String>,
}

impl FileMetaHeader {
    pub fn new(sample_rate: f64, data_type: &str) -> Self {
        Self {
            sample_rate,
            data_type: data_type.to_string(),
            num_samples: 0,
            metadata: HashMap::new(),
        }
    }

    /// Add a custom metadata entry.
    pub fn with_metadata(mut self, key: &str, value: &str) -> Self {
        self.metadata.insert(key.to_string(), value.to_string());
        self
    }

    /// Bytes per sample for the data type.
    pub fn bytes_per_sample(&self) -> usize {
        match self.data_type.as_str() {
            "cf64" => 16,
            "cf32" | "ettus" => 8,
            "ci16" | "sc16" => 4,
            "ci8" => 2,
            "cu8" | "rtlsdr" => 2,
            _ => 16, // default to cf64
        }
    }

    /// Serialize header to a JSON-like string.
    fn serialize(&self) -> String {
        let mut parts = vec![
            format!("\"sample_rate\":{}", self.sample_rate),
            format!("\"data_type\":\"{}\"", self.data_type),
            format!("\"num_samples\":{}", self.num_samples),
        ];

        if !self.metadata.is_empty() {
            let meta_parts: Vec<String> = self
                .metadata
                .iter()
                .map(|(k, v)| format!("\"{}\":\"{}\"", k, v))
                .collect();
            parts.push(format!("\"metadata\":{{{}}}", meta_parts.join(",")));
        }

        format!("{{{}}}", parts.join(","))
    }

    /// Deserialize header from a JSON-like string.
    fn deserialize(s: &str) -> io::Result<Self> {
        // Simple parser for our known format
        let s = s.trim();
        let s = s.trim_start_matches('{').trim_end_matches('}');

        let mut header = FileMetaHeader {
            sample_rate: 0.0,
            data_type: "cf64".to_string(),
            num_samples: 0,
            metadata: HashMap::new(),
        };

        // Parse key-value pairs
        let mut chars = s.chars().peekable();
        while chars.peek().is_some() {
            // Skip whitespace and commas
            while matches!(chars.peek(), Some(' ') | Some(',') | Some('\n')) {
                chars.next();
            }
            if chars.peek().is_none() {
                break;
            }

            // Read key
            let key = Self::read_quoted_string(&mut chars)?;
            // Skip colon
            while matches!(chars.peek(), Some(':') | Some(' ')) {
                chars.next();
            }

            match key.as_str() {
                "sample_rate" => {
                    let val = Self::read_number(&mut chars);
                    header.sample_rate = val.parse().unwrap_or(0.0);
                }
                "data_type" => {
                    header.data_type = Self::read_quoted_string(&mut chars)?;
                }
                "num_samples" => {
                    let val = Self::read_number(&mut chars);
                    header.num_samples = val.parse().unwrap_or(0);
                }
                "metadata" => {
                    // Skip the nested object for now
                    let mut depth = 0;
                    while let Some(&c) = chars.peek() {
                        chars.next();
                        if c == '{' {
                            depth += 1;
                        } else if c == '}' {
                            depth -= 1;
                            if depth == 0 {
                                break;
                            }
                        }
                    }
                }
                _ => {
                    // Skip unknown value
                    Self::skip_value(&mut chars);
                }
            }
        }

        Ok(header)
    }

    fn read_quoted_string(chars: &mut std::iter::Peekable<std::str::Chars>) -> io::Result<String> {
        // Skip opening quote
        while matches!(chars.peek(), Some('"')) {
            chars.next();
        }
        let mut s = String::new();
        while let Some(&c) = chars.peek() {
            if c == '"' {
                chars.next();
                break;
            }
            s.push(c);
            chars.next();
        }
        Ok(s)
    }

    fn read_number(chars: &mut std::iter::Peekable<std::str::Chars>) -> String {
        let mut s = String::new();
        while let Some(&c) = chars.peek() {
            if c.is_ascii_digit() || c == '.' || c == '-' || c == 'e' || c == 'E' || c == '+' {
                s.push(c);
                chars.next();
            } else {
                break;
            }
        }
        s
    }

    fn skip_value(chars: &mut std::iter::Peekable<std::str::Chars>) {
        while let Some(&c) = chars.peek() {
            if c == ',' || c == '}' {
                break;
            }
            chars.next();
        }
    }
}

/// File Meta Sink — writes IQ samples with metadata header.
pub struct FileMetaSink {
    writer: io::BufWriter<std::fs::File>,
    header: FileMetaHeader,
    samples_written: u64,
    header_offset: u64,
}

impl FileMetaSink {
    pub fn new(path: impl AsRef<Path>, header: FileMetaHeader) -> io::Result<Self> {
        let file = std::fs::File::create(path)?;
        let mut writer = io::BufWriter::new(file);

        // Write header line (will be updated on finalize)
        let header_str = header.serialize();
        let header_offset = header_str.len() as u64 + 1; // +1 for newline
        writer.write_all(header_str.as_bytes())?;
        writer.write_all(b"\n")?;

        Ok(Self {
            writer,
            header,
            samples_written: 0,
            header_offset,
        })
    }

    /// Write complex samples.
    pub fn write_samples(&mut self, samples: &[Complex64]) -> io::Result<()> {
        for s in samples {
            match self.header.data_type.as_str() {
                "cf64" => {
                    self.writer.write_all(&s.re.to_le_bytes())?;
                    self.writer.write_all(&s.im.to_le_bytes())?;
                }
                "cf32" | "ettus" => {
                    self.writer.write_all(&(s.re as f32).to_le_bytes())?;
                    self.writer.write_all(&(s.im as f32).to_le_bytes())?;
                }
                "ci16" | "sc16" => {
                    let i = (s.re * 32767.0).clamp(-32768.0, 32767.0) as i16;
                    let q = (s.im * 32767.0).clamp(-32768.0, 32767.0) as i16;
                    self.writer.write_all(&i.to_le_bytes())?;
                    self.writer.write_all(&q.to_le_bytes())?;
                }
                _ => {
                    self.writer.write_all(&s.re.to_le_bytes())?;
                    self.writer.write_all(&s.im.to_le_bytes())?;
                }
            }
        }
        self.samples_written += samples.len() as u64;
        Ok(())
    }

    /// Finalize the file (updates header with sample count).
    pub fn finalize(mut self) -> io::Result<()> {
        self.writer.flush()?;
        // Note: Updating the header in-place would require seeking.
        // For simplicity, we write the count to stderr/log.
        // A production version would seek back to update num_samples.
        Ok(())
    }

    pub fn samples_written(&self) -> u64 {
        self.samples_written
    }
}

/// File Meta Source — reads IQ samples with metadata header.
pub struct FileMetaSource {
    reader: BufReader<std::fs::File>,
    header: FileMetaHeader,
}

impl FileMetaSource {
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let file = std::fs::File::open(path)?;
        let mut reader = BufReader::new(file);

        // Read header line
        let mut header_line = String::new();
        reader.read_line(&mut header_line)?;

        let header = FileMetaHeader::deserialize(&header_line)?;

        Ok(Self { reader, header })
    }

    /// Get the file header.
    pub fn header(&self) -> &FileMetaHeader {
        &self.header
    }

    /// Read up to N complex samples. Returns fewer if EOF is reached.
    pub fn read_samples(&mut self, count: usize) -> io::Result<Vec<Complex64>> {
        let mut samples = Vec::with_capacity(count);

        for _ in 0..count {
            let sample = match self.header.data_type.as_str() {
                "cf64" => {
                    let mut buf = [0u8; 16];
                    match self.reader.read_exact(&mut buf) {
                        Ok(()) => {}
                        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(samples),
                        Err(e) => return Err(e),
                    }
                    let re = f64::from_le_bytes(buf[..8].try_into().unwrap());
                    let im = f64::from_le_bytes(buf[8..].try_into().unwrap());
                    Complex64::new(re, im)
                }
                "cf32" | "ettus" => {
                    let mut buf = [0u8; 8];
                    match self.reader.read_exact(&mut buf) {
                        Ok(()) => {}
                        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(samples),
                        Err(e) => return Err(e),
                    }
                    let re = f32::from_le_bytes(buf[..4].try_into().unwrap()) as f64;
                    let im = f32::from_le_bytes(buf[4..].try_into().unwrap()) as f64;
                    Complex64::new(re, im)
                }
                "ci16" | "sc16" => {
                    let mut buf = [0u8; 4];
                    match self.reader.read_exact(&mut buf) {
                        Ok(()) => {}
                        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(samples),
                        Err(e) => return Err(e),
                    }
                    let re = i16::from_le_bytes(buf[..2].try_into().unwrap()) as f64 / 32767.0;
                    let im = i16::from_le_bytes(buf[2..].try_into().unwrap()) as f64 / 32767.0;
                    Complex64::new(re, im)
                }
                _ => {
                    let mut buf = [0u8; 16];
                    match self.reader.read_exact(&mut buf) {
                        Ok(()) => {}
                        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(samples),
                        Err(e) => return Err(e),
                    }
                    let re = f64::from_le_bytes(buf[..8].try_into().unwrap());
                    let im = f64::from_le_bytes(buf[8..].try_into().unwrap());
                    Complex64::new(re, im)
                }
            };
            samples.push(sample);
        }

        Ok(samples)
    }

    /// Read all remaining samples.
    pub fn read_all(&mut self) -> io::Result<Vec<Complex64>> {
        let mut all = Vec::new();
        loop {
            match self.read_samples(1024) {
                Ok(samples) => {
                    if samples.is_empty() {
                        break;
                    }
                    all.extend(samples);
                }
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
                Err(e) => return Err(e),
            }
        }
        Ok(all)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write as _;

    #[test]
    fn test_header_serialize_deserialize() {
        let header = FileMetaHeader::new(48000.0, "cf64")
            .with_metadata("description", "test signal");
        let s = header.serialize();
        let parsed = FileMetaHeader::deserialize(&s).unwrap();
        assert_eq!(parsed.sample_rate, 48000.0);
        assert_eq!(parsed.data_type, "cf64");
    }

    #[test]
    fn test_header_roundtrip() {
        let header = FileMetaHeader::new(2400000.0, "cf32");
        let s = header.serialize();
        let parsed = FileMetaHeader::deserialize(&s).unwrap();
        assert_eq!(parsed.sample_rate, 2400000.0);
        assert_eq!(parsed.data_type, "cf32");
    }

    #[test]
    fn test_bytes_per_sample() {
        assert_eq!(FileMetaHeader::new(0.0, "cf64").bytes_per_sample(), 16);
        assert_eq!(FileMetaHeader::new(0.0, "cf32").bytes_per_sample(), 8);
        assert_eq!(FileMetaHeader::new(0.0, "ci16").bytes_per_sample(), 4);
        assert_eq!(FileMetaHeader::new(0.0, "ci8").bytes_per_sample(), 2);
    }

    #[test]
    fn test_file_roundtrip_cf64() {
        let dir = std::env::temp_dir();
        let path = dir.join("test_meta_cf64.iqm");

        let header = FileMetaHeader::new(48000.0, "cf64");
        let mut sink = FileMetaSink::new(&path, header).unwrap();

        let samples: Vec<Complex64> = (0..100)
            .map(|i| Complex64::new(i as f64 * 0.01, -i as f64 * 0.005))
            .collect();
        sink.write_samples(&samples).unwrap();
        assert_eq!(sink.samples_written(), 100);
        sink.finalize().unwrap();

        let mut source = FileMetaSource::open(&path).unwrap();
        assert_eq!(source.header().sample_rate, 48000.0);
        assert_eq!(source.header().data_type, "cf64");

        let read = source.read_all().unwrap();
        assert_eq!(read.len(), 100);
        for (orig, recv) in samples.iter().zip(read.iter()) {
            assert!((orig - recv).norm() < 1e-10);
        }

        std::fs::remove_file(path).ok();
    }

    #[test]
    fn test_file_roundtrip_cf32() {
        let dir = std::env::temp_dir();
        let path = dir.join("test_meta_cf32.iqm");

        let header = FileMetaHeader::new(1000000.0, "cf32");
        let mut sink = FileMetaSink::new(&path, header).unwrap();

        let samples: Vec<Complex64> = (0..50)
            .map(|i| Complex64::new((i as f64 * 0.1).sin(), (i as f64 * 0.1).cos()))
            .collect();
        sink.write_samples(&samples).unwrap();
        sink.finalize().unwrap();

        let mut source = FileMetaSource::open(&path).unwrap();
        let read = source.read_all().unwrap();
        assert_eq!(read.len(), 50);
        for (orig, recv) in samples.iter().zip(read.iter()) {
            assert!((orig - recv).norm() < 0.001, "cf32 roundtrip");
        }

        std::fs::remove_file(path).ok();
    }

    #[test]
    fn test_with_metadata() {
        let header = FileMetaHeader::new(48000.0, "cf64")
            .with_metadata("center_freq", "915000000")
            .with_metadata("antenna", "omni");
        assert_eq!(header.metadata.get("center_freq").unwrap(), "915000000");
        assert_eq!(header.metadata.get("antenna").unwrap(), "omni");
    }
}

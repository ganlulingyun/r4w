//! File Source/Sink — Streaming raw IQ file I/O
//!
//! Reads and writes raw binary IQ sample files in various formats
//! (cf64, cf32, ci16, cu8, etc.). FileSource reads samples from disk
//! as a streaming source, FileSink writes sample streams to disk.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::file_source_sink::{IqFileWriter, IqFileReader, IqFileFormat};
//! use num_complex::Complex64;
//!
//! // Write samples
//! let samples = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
//! let tmp = std::env::temp_dir().join("r4w_test_file_source.cf64");
//! let mut writer = IqFileWriter::new(&tmp, IqFileFormat::Cf64).unwrap();
//! writer.write(&samples).unwrap();
//! writer.close().unwrap();
//!
//! // Read them back
//! let mut reader = IqFileReader::new(&tmp, IqFileFormat::Cf64).unwrap();
//! let read_back = reader.read(2).unwrap();
//! assert_eq!(read_back.len(), 2);
//! assert!((read_back[0].re - 1.0).abs() < 1e-10);
//! std::fs::remove_file(&tmp).ok();
//! ```

use num_complex::Complex64;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

/// IQ sample format for raw files.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IqFileFormat {
    /// Complex float64 (interleaved f64, 16 bytes/sample).
    Cf64,
    /// Complex float32 (interleaved f32, 8 bytes/sample, USRP/Ettus).
    Cf32,
    /// Complex int16 (interleaved i16, 4 bytes/sample).
    Ci16,
    /// Complex int8 (interleaved i8, 2 bytes/sample).
    Ci8,
    /// Complex uint8 (interleaved u8, 2 bytes/sample, RTL-SDR, DC=128).
    Cu8,
}

impl IqFileFormat {
    /// Bytes per complex sample.
    pub fn bytes_per_sample(&self) -> usize {
        match self {
            Self::Cf64 => 16,
            Self::Cf32 => 8,
            Self::Ci16 => 4,
            Self::Ci8 => 2,
            Self::Cu8 => 2,
        }
    }

    /// Detect format from file extension.
    pub fn from_extension(path: &Path) -> Option<Self> {
        let ext = path.extension()?.to_str()?.to_lowercase();
        match ext.as_str() {
            "cf64" | "fc64" => Some(Self::Cf64),
            "cf32" | "fc32" | "iq" => Some(Self::Cf32),
            "ci16" | "sc16" | "cs16" => Some(Self::Ci16),
            "ci8" | "cs8" => Some(Self::Ci8),
            "cu8" | "raw" => Some(Self::Cu8),
            _ => None,
        }
    }

    /// Get format name.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Cf64 => "cf64",
            Self::Cf32 => "cf32",
            Self::Ci16 => "ci16",
            Self::Ci8 => "ci8",
            Self::Cu8 => "cu8",
        }
    }
}

/// Streaming IQ file reader.
pub struct IqFileReader {
    reader: BufReader<File>,
    format: IqFileFormat,
    samples_read: u64,
}

impl IqFileReader {
    /// Open a file for reading.
    pub fn new(path: &Path, format: IqFileFormat) -> io::Result<Self> {
        let file = File::open(path)?;
        Ok(Self {
            reader: BufReader::new(file),
            format,
            samples_read: 0,
        })
    }

    /// Open with auto-detected format from extension.
    pub fn auto(path: &Path) -> io::Result<Self> {
        let format = IqFileFormat::from_extension(path)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "Unknown file extension"))?;
        Self::new(path, format)
    }

    /// Read up to `n` samples. Returns fewer at EOF.
    pub fn read(&mut self, n: usize) -> io::Result<Vec<Complex64>> {
        let bytes_needed = n * self.format.bytes_per_sample();
        let mut buf = vec![0u8; bytes_needed];
        let bytes_read = self.reader.read(&mut buf)?;
        let samples_available = bytes_read / self.format.bytes_per_sample();

        let mut samples = Vec::with_capacity(samples_available);
        for i in 0..samples_available {
            let offset = i * self.format.bytes_per_sample();
            let sample = self.decode_sample(&buf[offset..offset + self.format.bytes_per_sample()]);
            samples.push(sample);
        }

        self.samples_read += samples.len() as u64;
        Ok(samples)
    }

    fn decode_sample(&self, bytes: &[u8]) -> Complex64 {
        match self.format {
            IqFileFormat::Cf64 => {
                let re = f64::from_le_bytes(bytes[0..8].try_into().unwrap());
                let im = f64::from_le_bytes(bytes[8..16].try_into().unwrap());
                Complex64::new(re, im)
            }
            IqFileFormat::Cf32 => {
                let re = f32::from_le_bytes(bytes[0..4].try_into().unwrap()) as f64;
                let im = f32::from_le_bytes(bytes[4..8].try_into().unwrap()) as f64;
                Complex64::new(re, im)
            }
            IqFileFormat::Ci16 => {
                let re = i16::from_le_bytes(bytes[0..2].try_into().unwrap()) as f64 / 32768.0;
                let im = i16::from_le_bytes(bytes[2..4].try_into().unwrap()) as f64 / 32768.0;
                Complex64::new(re, im)
            }
            IqFileFormat::Ci8 => {
                let re = bytes[0] as i8 as f64 / 128.0;
                let im = bytes[1] as i8 as f64 / 128.0;
                Complex64::new(re, im)
            }
            IqFileFormat::Cu8 => {
                let re = (bytes[0] as f64 - 128.0) / 128.0;
                let im = (bytes[1] as f64 - 128.0) / 128.0;
                Complex64::new(re, im)
            }
        }
    }

    /// Get total samples read so far.
    pub fn samples_read(&self) -> u64 {
        self.samples_read
    }

    /// Get the format.
    pub fn format(&self) -> IqFileFormat {
        self.format
    }
}

/// Streaming IQ file writer.
pub struct IqFileWriter {
    writer: BufWriter<File>,
    format: IqFileFormat,
    samples_written: u64,
}

impl IqFileWriter {
    /// Create a file for writing (truncates existing).
    pub fn new(path: &Path, format: IqFileFormat) -> io::Result<Self> {
        let file = File::create(path)?;
        Ok(Self {
            writer: BufWriter::new(file),
            format,
            samples_written: 0,
        })
    }

    /// Create with auto-detected format from extension.
    pub fn auto(path: &Path) -> io::Result<Self> {
        let format = IqFileFormat::from_extension(path)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "Unknown file extension"))?;
        Self::new(path, format)
    }

    /// Append to existing file.
    pub fn append(path: &Path, format: IqFileFormat) -> io::Result<Self> {
        let file = std::fs::OpenOptions::new().create(true).append(true).open(path)?;
        Ok(Self {
            writer: BufWriter::new(file),
            format,
            samples_written: 0,
        })
    }

    /// Write a block of samples.
    pub fn write(&mut self, samples: &[Complex64]) -> io::Result<()> {
        for sample in samples {
            self.encode_sample(*sample)?;
        }
        self.samples_written += samples.len() as u64;
        Ok(())
    }

    fn encode_sample(&mut self, sample: Complex64) -> io::Result<()> {
        match self.format {
            IqFileFormat::Cf64 => {
                self.writer.write_all(&sample.re.to_le_bytes())?;
                self.writer.write_all(&sample.im.to_le_bytes())?;
            }
            IqFileFormat::Cf32 => {
                self.writer.write_all(&(sample.re as f32).to_le_bytes())?;
                self.writer.write_all(&(sample.im as f32).to_le_bytes())?;
            }
            IqFileFormat::Ci16 => {
                let re = (sample.re * 32767.0).clamp(-32768.0, 32767.0) as i16;
                let im = (sample.im * 32767.0).clamp(-32768.0, 32767.0) as i16;
                self.writer.write_all(&re.to_le_bytes())?;
                self.writer.write_all(&im.to_le_bytes())?;
            }
            IqFileFormat::Ci8 => {
                let re = (sample.re * 127.0).clamp(-128.0, 127.0) as i8;
                let im = (sample.im * 127.0).clamp(-128.0, 127.0) as i8;
                self.writer.write_all(&[re as u8])?;
                self.writer.write_all(&[im as u8])?;
            }
            IqFileFormat::Cu8 => {
                let re = ((sample.re * 128.0) + 128.0).clamp(0.0, 255.0) as u8;
                let im = ((sample.im * 128.0) + 128.0).clamp(0.0, 255.0) as u8;
                self.writer.write_all(&[re])?;
                self.writer.write_all(&[im])?;
            }
        }
        Ok(())
    }

    /// Flush the writer.
    pub fn flush(&mut self) -> io::Result<()> {
        self.writer.flush()
    }

    /// Close the writer (flush + drop).
    pub fn close(mut self) -> io::Result<()> {
        self.writer.flush()
    }

    /// Get total samples written.
    pub fn samples_written(&self) -> u64 {
        self.samples_written
    }

    /// Get the format.
    pub fn format(&self) -> IqFileFormat {
        self.format
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::env;

    fn temp_path(name: &str) -> std::path::PathBuf {
        env::temp_dir().join(format!("r4w_fss_test_{}", name))
    }

    fn test_samples() -> Vec<Complex64> {
        vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(0.0, -1.0),
            Complex64::new(0.5, 0.5),
        ]
    }

    #[test]
    fn test_cf64_roundtrip() {
        let path = temp_path("cf64.cf64");
        let samples = test_samples();
        let mut writer = IqFileWriter::new(&path, IqFileFormat::Cf64).unwrap();
        writer.write(&samples).unwrap();
        writer.close().unwrap();

        let mut reader = IqFileReader::new(&path, IqFileFormat::Cf64).unwrap();
        let read_back = reader.read(10).unwrap();
        assert_eq!(read_back.len(), 5);
        for (a, b) in samples.iter().zip(read_back.iter()) {
            assert!((a.re - b.re).abs() < 1e-10);
            assert!((a.im - b.im).abs() < 1e-10);
        }
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_cf32_roundtrip() {
        let path = temp_path("cf32.cf32");
        let samples = test_samples();
        let mut writer = IqFileWriter::new(&path, IqFileFormat::Cf32).unwrap();
        writer.write(&samples).unwrap();
        writer.close().unwrap();

        let mut reader = IqFileReader::new(&path, IqFileFormat::Cf32).unwrap();
        let read_back = reader.read(10).unwrap();
        assert_eq!(read_back.len(), 5);
        for (a, b) in samples.iter().zip(read_back.iter()) {
            assert!((a.re - b.re).abs() < 1e-5); // f32 precision
            assert!((a.im - b.im).abs() < 1e-5);
        }
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_ci16_roundtrip() {
        let path = temp_path("ci16.ci16");
        let samples = test_samples();
        let mut writer = IqFileWriter::new(&path, IqFileFormat::Ci16).unwrap();
        writer.write(&samples).unwrap();
        writer.close().unwrap();

        let mut reader = IqFileReader::new(&path, IqFileFormat::Ci16).unwrap();
        let read_back = reader.read(10).unwrap();
        assert_eq!(read_back.len(), 5);
        // i16 quantization: ~1/32768 precision
        for (a, b) in samples.iter().zip(read_back.iter()) {
            assert!((a.re - b.re).abs() < 0.001);
            assert!((a.im - b.im).abs() < 0.001);
        }
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_cu8_roundtrip() {
        let path = temp_path("cu8.cu8");
        let samples = vec![
            Complex64::new(0.0, 0.0),
            Complex64::new(0.5, -0.5),
        ];
        let mut writer = IqFileWriter::new(&path, IqFileFormat::Cu8).unwrap();
        writer.write(&samples).unwrap();
        writer.close().unwrap();

        let mut reader = IqFileReader::new(&path, IqFileFormat::Cu8).unwrap();
        let read_back = reader.read(10).unwrap();
        assert_eq!(read_back.len(), 2);
        // cu8 has ~1/128 precision
        for (a, b) in samples.iter().zip(read_back.iter()) {
            assert!((a.re - b.re).abs() < 0.02);
            assert!((a.im - b.im).abs() < 0.02);
        }
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_format_from_extension() {
        assert_eq!(IqFileFormat::from_extension(Path::new("file.cf64")), Some(IqFileFormat::Cf64));
        assert_eq!(IqFileFormat::from_extension(Path::new("file.iq")), Some(IqFileFormat::Cf32));
        assert_eq!(IqFileFormat::from_extension(Path::new("file.sc16")), Some(IqFileFormat::Ci16));
        assert_eq!(IqFileFormat::from_extension(Path::new("file.raw")), Some(IqFileFormat::Cu8));
        assert_eq!(IqFileFormat::from_extension(Path::new("file.txt")), None);
    }

    #[test]
    fn test_bytes_per_sample() {
        assert_eq!(IqFileFormat::Cf64.bytes_per_sample(), 16);
        assert_eq!(IqFileFormat::Cf32.bytes_per_sample(), 8);
        assert_eq!(IqFileFormat::Ci16.bytes_per_sample(), 4);
        assert_eq!(IqFileFormat::Ci8.bytes_per_sample(), 2);
        assert_eq!(IqFileFormat::Cu8.bytes_per_sample(), 2);
    }

    #[test]
    fn test_format_names() {
        assert_eq!(IqFileFormat::Cf64.name(), "cf64");
        assert_eq!(IqFileFormat::Cf32.name(), "cf32");
    }

    #[test]
    fn test_auto_open() {
        let path = temp_path("auto.cf64");
        let samples = test_samples();
        let mut writer = IqFileWriter::auto(&path).unwrap();
        writer.write(&samples).unwrap();
        writer.close().unwrap();

        let mut reader = IqFileReader::auto(&path).unwrap();
        let read_back = reader.read(10).unwrap();
        assert_eq!(read_back.len(), 5);
        assert_eq!(reader.format(), IqFileFormat::Cf64);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_samples_count() {
        let path = temp_path("count.cf64");
        let samples = test_samples();
        let mut writer = IqFileWriter::new(&path, IqFileFormat::Cf64).unwrap();
        writer.write(&samples).unwrap();
        assert_eq!(writer.samples_written(), 5);
        writer.close().unwrap();

        let mut reader = IqFileReader::new(&path, IqFileFormat::Cf64).unwrap();
        reader.read(3).unwrap();
        assert_eq!(reader.samples_read(), 3);
        reader.read(10).unwrap();
        assert_eq!(reader.samples_read(), 5);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_partial_read() {
        let path = temp_path("partial.cf64");
        let samples = test_samples();
        let mut writer = IqFileWriter::new(&path, IqFileFormat::Cf64).unwrap();
        writer.write(&samples).unwrap();
        writer.close().unwrap();

        let mut reader = IqFileReader::new(&path, IqFileFormat::Cf64).unwrap();
        let chunk1 = reader.read(2).unwrap();
        assert_eq!(chunk1.len(), 2);
        let chunk2 = reader.read(2).unwrap();
        assert_eq!(chunk2.len(), 2);
        let chunk3 = reader.read(10).unwrap(); // Only 1 left
        assert_eq!(chunk3.len(), 1);
        std::fs::remove_file(&path).ok();
    }
}

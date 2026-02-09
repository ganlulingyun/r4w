//! WAV File Source/Sink — Read and write WAV audio files
//!
//! Pure Rust WAV file I/O for audio-frequency signals. Supports mono
//! (real), stereo (I/Q as L/R), 8/16/24/32-bit PCM, and 32-bit float.
//! GNU Radio equivalent: `wavfile_source`, `wavfile_sink`.
//!
//! ## Example
//!
//! ```rust,no_run
//! use r4w_core::wav_source_sink::{WavFileSink, WavFileSource};
//!
//! // Write a 1 kHz tone to WAV
//! let mut sink = WavFileSink::create_mono("/tmp/test.wav", 48000).unwrap();
//! let tone: Vec<f64> = (0..48000).map(|i| {
//!     (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin()
//! }).collect();
//! sink.write_samples(&tone).unwrap();
//! sink.close().unwrap();
//!
//! // Read it back
//! let mut source = WavFileSource::open("/tmp/test.wav").unwrap();
//! assert_eq!(source.sample_rate(), 48000);
//! let data = source.read_all().unwrap();
//! assert_eq!(data.len(), 48000);
//! ```

use std::io::{self, Read, Seek, SeekFrom, Write};
use std::fs::File;
use std::path::Path;
use num_complex::Complex64;

/// WAV file reader.
#[derive(Debug)]
pub struct WavFileSource {
    reader: File,
    sample_rate: u32,
    channels: u16,
    bits_per_sample: u16,
    num_samples: u64,
    data_start: u64,
    samples_read: u64,
    format_tag: u16,
}

impl WavFileSource {
    /// Open a WAV file for reading.
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let mut file = File::open(path)?;
        let header = Self::read_header(&mut file)?;
        Ok(header)
    }

    fn read_header(file: &mut File) -> io::Result<WavFileSource> {
        let mut buf4 = [0u8; 4];
        let mut buf2 = [0u8; 2];

        // RIFF header
        file.read_exact(&mut buf4)?;
        if &buf4 != b"RIFF" {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "not a RIFF file"));
        }
        file.read_exact(&mut buf4)?; // file size - 8
        file.read_exact(&mut buf4)?;
        if &buf4 != b"WAVE" {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "not a WAVE file"));
        }

        let mut sample_rate = 0u32;
        let mut channels = 0u16;
        let mut bits_per_sample = 0u16;
        let mut data_start = 0u64;
        let mut data_size = 0u32;
        let mut format_tag = 1u16; // PCM

        // Read chunks
        loop {
            if file.read_exact(&mut buf4).is_err() {
                break;
            }
            let chunk_id = buf4;
            file.read_exact(&mut buf4)?;
            let chunk_size = u32::from_le_bytes(buf4);

            if &chunk_id == b"fmt " {
                file.read_exact(&mut buf2)?;
                format_tag = u16::from_le_bytes(buf2);
                file.read_exact(&mut buf2)?;
                channels = u16::from_le_bytes(buf2);
                file.read_exact(&mut buf4)?;
                sample_rate = u32::from_le_bytes(buf4);
                file.read_exact(&mut buf4)?; // byte rate
                file.read_exact(&mut buf2)?; // block align
                file.read_exact(&mut buf2)?;
                bits_per_sample = u16::from_le_bytes(buf2);
                // Skip remaining fmt chunk data
                let read_so_far = 16;
                if chunk_size > read_so_far {
                    file.seek(SeekFrom::Current((chunk_size - read_so_far) as i64))?;
                }
            } else if &chunk_id == b"data" {
                data_start = file.stream_position()?;
                data_size = chunk_size;
                break;
            } else {
                // Skip unknown chunk
                file.seek(SeekFrom::Current(chunk_size as i64))?;
            }
        }

        if data_start == 0 {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "no data chunk found"));
        }

        let bytes_per_sample = (bits_per_sample as u64 / 8) * channels as u64;
        let num_samples = if bytes_per_sample > 0 {
            data_size as u64 / bytes_per_sample
        } else {
            0
        };

        Ok(WavFileSource {
            reader: file.try_clone()?,
            sample_rate,
            channels,
            bits_per_sample,
            num_samples,
            data_start,
            samples_read: 0,
            format_tag,
        })
    }

    /// Get sample rate in Hz.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Get number of channels.
    pub fn channels(&self) -> u16 {
        self.channels
    }

    /// Get bits per sample.
    pub fn bits_per_sample(&self) -> u16 {
        self.bits_per_sample
    }

    /// Get total number of sample frames.
    pub fn num_samples(&self) -> u64 {
        self.num_samples
    }

    /// Get duration in seconds.
    pub fn duration_secs(&self) -> f64 {
        if self.sample_rate > 0 {
            self.num_samples as f64 / self.sample_rate as f64
        } else {
            0.0
        }
    }

    /// Check if at end of file.
    pub fn is_eof(&self) -> bool {
        self.samples_read >= self.num_samples
    }

    /// Read mono samples as f64 (range -1.0 to 1.0).
    /// For stereo files, channels are interleaved.
    pub fn read_samples(&mut self, count: usize) -> io::Result<Vec<f64>> {
        let remaining = (self.num_samples - self.samples_read) as usize;
        let to_read = count.min(remaining);
        if to_read == 0 {
            return Ok(Vec::new());
        }

        let total_values = to_read * self.channels as usize;
        let mut samples = Vec::with_capacity(total_values);

        match (self.format_tag, self.bits_per_sample) {
            (1, 8) => {
                let mut buf = vec![0u8; total_values];
                self.reader.read_exact(&mut buf)?;
                for &b in &buf {
                    samples.push((b as f64 - 128.0) / 128.0);
                }
            }
            (1, 16) => {
                let mut buf = vec![0u8; total_values * 2];
                self.reader.read_exact(&mut buf)?;
                for i in 0..total_values {
                    let v = i16::from_le_bytes([buf[i * 2], buf[i * 2 + 1]]);
                    samples.push(v as f64 / 32768.0);
                }
            }
            (1, 24) => {
                let mut buf = vec![0u8; total_values * 3];
                self.reader.read_exact(&mut buf)?;
                for i in 0..total_values {
                    let b0 = buf[i * 3] as i32;
                    let b1 = buf[i * 3 + 1] as i32;
                    let b2 = buf[i * 3 + 2] as i32;
                    let v = (b2 << 24 | b1 << 16 | b0 << 8) >> 8; // sign extend
                    samples.push(v as f64 / 8388608.0);
                }
            }
            (1, 32) => {
                let mut buf = vec![0u8; total_values * 4];
                self.reader.read_exact(&mut buf)?;
                for i in 0..total_values {
                    let v = i32::from_le_bytes([buf[i * 4], buf[i * 4 + 1], buf[i * 4 + 2], buf[i * 4 + 3]]);
                    samples.push(v as f64 / 2147483648.0);
                }
            }
            (3, 32) => {
                // IEEE float
                let mut buf = vec![0u8; total_values * 4];
                self.reader.read_exact(&mut buf)?;
                for i in 0..total_values {
                    let v = f32::from_le_bytes([buf[i * 4], buf[i * 4 + 1], buf[i * 4 + 2], buf[i * 4 + 3]]);
                    samples.push(v as f64);
                }
            }
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    format!("unsupported WAV format: tag={}, bits={}", self.format_tag, self.bits_per_sample),
                ));
            }
        }

        self.samples_read += to_read as u64;

        // For mono, return as-is. For stereo, average channels to mono.
        if self.channels == 1 {
            Ok(samples)
        } else {
            let mono: Vec<f64> = samples.chunks(self.channels as usize)
                .map(|ch| ch.iter().sum::<f64>() / ch.len() as f64)
                .collect();
            Ok(mono)
        }
    }

    /// Read all remaining samples.
    pub fn read_all(&mut self) -> io::Result<Vec<f64>> {
        let remaining = (self.num_samples - self.samples_read) as usize;
        self.read_samples(remaining)
    }

    /// Read stereo as complex (L=real, R=imag).
    pub fn read_stereo_iq(&mut self, count: usize) -> io::Result<Vec<Complex64>> {
        if self.channels < 2 {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "file is not stereo"));
        }

        let remaining = (self.num_samples - self.samples_read) as usize;
        let to_read = count.min(remaining);
        if to_read == 0 {
            return Ok(Vec::new());
        }

        // Read raw interleaved samples
        let total_values = to_read * self.channels as usize;
        let mut raw = Vec::with_capacity(total_values);

        match (self.format_tag, self.bits_per_sample) {
            (1, 16) => {
                let mut buf = vec![0u8; total_values * 2];
                self.reader.read_exact(&mut buf)?;
                for i in 0..total_values {
                    let v = i16::from_le_bytes([buf[i * 2], buf[i * 2 + 1]]);
                    raw.push(v as f64 / 32768.0);
                }
            }
            (3, 32) => {
                let mut buf = vec![0u8; total_values * 4];
                self.reader.read_exact(&mut buf)?;
                for i in 0..total_values {
                    let v = f32::from_le_bytes([buf[i * 4], buf[i * 4 + 1], buf[i * 4 + 2], buf[i * 4 + 3]]);
                    raw.push(v as f64);
                }
            }
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    format!("stereo IQ only supports 16-bit PCM or 32-bit float"),
                ));
            }
        }

        self.samples_read += to_read as u64;

        let iq: Vec<Complex64> = raw.chunks(self.channels as usize)
            .map(|ch| Complex64::new(ch[0], if ch.len() > 1 { ch[1] } else { 0.0 }))
            .collect();
        Ok(iq)
    }

    /// Seek to a specific sample frame.
    pub fn seek(&mut self, sample: u64) -> io::Result<()> {
        let bytes_per_frame = (self.bits_per_sample as u64 / 8) * self.channels as u64;
        let offset = self.data_start + sample * bytes_per_frame;
        self.reader.seek(SeekFrom::Start(offset))?;
        self.samples_read = sample;
        Ok(())
    }
}

/// WAV file writer.
#[derive(Debug)]
pub struct WavFileSink {
    writer: File,
    sample_rate: u32,
    channels: u16,
    bits_per_sample: u16,
    format_tag: u16,
    data_start: u64,
    samples_written: u64,
}

impl WavFileSink {
    /// Create a WAV file with specified parameters.
    pub fn create(path: impl AsRef<Path>, sample_rate: u32, channels: u16, bits_per_sample: u16) -> io::Result<Self> {
        let format_tag = if bits_per_sample == 32 && channels > 0 { 3 } else { 1 }; // float32 or PCM
        Self::create_with_format(path, sample_rate, channels, bits_per_sample, format_tag)
    }

    /// Create a 16-bit mono WAV file.
    pub fn create_mono(path: impl AsRef<Path>, sample_rate: u32) -> io::Result<Self> {
        Self::create_with_format(path, sample_rate, 1, 16, 1)
    }

    /// Create a 16-bit stereo WAV file for IQ (L=I, R=Q).
    pub fn create_stereo_iq(path: impl AsRef<Path>, sample_rate: u32) -> io::Result<Self> {
        Self::create_with_format(path, sample_rate, 2, 16, 1)
    }

    fn create_with_format(path: impl AsRef<Path>, sample_rate: u32, channels: u16, bits_per_sample: u16, format_tag: u16) -> io::Result<Self> {
        let mut file = File::create(path)?;

        let block_align = channels * (bits_per_sample / 8);
        let byte_rate = sample_rate * block_align as u32;

        // Write RIFF header (will be updated on close)
        file.write_all(b"RIFF")?;
        file.write_all(&0u32.to_le_bytes())?; // placeholder
        file.write_all(b"WAVE")?;

        // fmt chunk
        file.write_all(b"fmt ")?;
        file.write_all(&16u32.to_le_bytes())?; // chunk size
        file.write_all(&format_tag.to_le_bytes())?;
        file.write_all(&channels.to_le_bytes())?;
        file.write_all(&sample_rate.to_le_bytes())?;
        file.write_all(&byte_rate.to_le_bytes())?;
        file.write_all(&block_align.to_le_bytes())?;
        file.write_all(&bits_per_sample.to_le_bytes())?;

        // data chunk header
        file.write_all(b"data")?;
        file.write_all(&0u32.to_le_bytes())?; // placeholder
        let data_start = file.stream_position()?;

        Ok(WavFileSink {
            writer: file,
            sample_rate,
            channels,
            bits_per_sample,
            format_tag,
            data_start,
            samples_written: 0,
        })
    }

    /// Write mono f64 samples (range -1.0 to 1.0).
    pub fn write_samples(&mut self, samples: &[f64]) -> io::Result<()> {
        match (self.format_tag, self.bits_per_sample) {
            (1, 16) => {
                let mut buf = Vec::with_capacity(samples.len() * 2);
                for &s in samples {
                    let v = (s.clamp(-1.0, 1.0) * 32767.0) as i16;
                    buf.extend_from_slice(&v.to_le_bytes());
                }
                self.writer.write_all(&buf)?;
            }
            (1, 8) => {
                let mut buf = Vec::with_capacity(samples.len());
                for &s in samples {
                    let v = ((s.clamp(-1.0, 1.0) + 1.0) * 128.0).min(255.0) as u8;
                    buf.push(v);
                }
                self.writer.write_all(&buf)?;
            }
            (3, 32) => {
                let mut buf = Vec::with_capacity(samples.len() * 4);
                for &s in samples {
                    buf.extend_from_slice(&(s as f32).to_le_bytes());
                }
                self.writer.write_all(&buf)?;
            }
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    "unsupported format for writing",
                ));
            }
        }
        self.samples_written += samples.len() as u64 / self.channels as u64;
        Ok(())
    }

    /// Write complex samples as stereo (L=real, R=imag).
    pub fn write_complex_as_stereo(&mut self, samples: &[Complex64]) -> io::Result<()> {
        if self.channels != 2 {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "file is not stereo"));
        }
        match (self.format_tag, self.bits_per_sample) {
            (1, 16) => {
                let mut buf = Vec::with_capacity(samples.len() * 4);
                for s in samples {
                    let l = (s.re.clamp(-1.0, 1.0) * 32767.0) as i16;
                    let r = (s.im.clamp(-1.0, 1.0) * 32767.0) as i16;
                    buf.extend_from_slice(&l.to_le_bytes());
                    buf.extend_from_slice(&r.to_le_bytes());
                }
                self.writer.write_all(&buf)?;
            }
            (3, 32) => {
                let mut buf = Vec::with_capacity(samples.len() * 8);
                for s in samples {
                    buf.extend_from_slice(&(s.re as f32).to_le_bytes());
                    buf.extend_from_slice(&(s.im as f32).to_le_bytes());
                }
                self.writer.write_all(&buf)?;
            }
            _ => {
                return Err(io::Error::new(io::ErrorKind::Unsupported, "unsupported format"));
            }
        }
        self.samples_written += samples.len() as u64;
        Ok(())
    }

    /// Get number of sample frames written.
    pub fn samples_written(&self) -> u64 {
        self.samples_written
    }

    /// Get sample rate.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Close the file and update headers.
    pub fn close(mut self) -> io::Result<()> {
        self.finalize()
    }

    fn finalize(&mut self) -> io::Result<()> {
        let bytes_per_frame = (self.bits_per_sample as u64 / 8) * self.channels as u64;
        let data_size = self.samples_written * bytes_per_frame;

        // Update data chunk size
        self.writer.seek(SeekFrom::Start(self.data_start - 4))?;
        self.writer.write_all(&(data_size as u32).to_le_bytes())?;

        // Update RIFF chunk size
        let riff_size = self.data_start + data_size - 8;
        self.writer.seek(SeekFrom::Start(4))?;
        self.writer.write_all(&(riff_size as u32).to_le_bytes())?;

        self.writer.flush()
    }
}

impl Drop for WavFileSink {
    fn drop(&mut self) {
        let _ = self.finalize();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn temp_path(name: &str) -> String {
        format!("/tmp/r4w_test_{}.wav", name)
    }

    #[test]
    fn test_mono_16bit_roundtrip() {
        let path = temp_path("mono16");
        let samples: Vec<f64> = (0..1000).map(|i| {
            (2.0 * PI * 440.0 * i as f64 / 48000.0).sin()
        }).collect();

        // Write
        let mut sink = WavFileSink::create_mono(&path, 48000).unwrap();
        sink.write_samples(&samples).unwrap();
        sink.close().unwrap();

        // Read
        let mut source = WavFileSource::open(&path).unwrap();
        assert_eq!(source.sample_rate(), 48000);
        assert_eq!(source.channels(), 1);
        assert_eq!(source.bits_per_sample(), 16);
        assert_eq!(source.num_samples(), 1000);
        assert!((source.duration_secs() - 1000.0 / 48000.0).abs() < 1e-6);

        let read_back = source.read_all().unwrap();
        assert_eq!(read_back.len(), 1000);

        // 16-bit quantization error should be small
        for (a, b) in samples.iter().zip(read_back.iter()) {
            assert!((a - b).abs() < 0.001, "mismatch: {} vs {}", a, b);
        }

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_stereo_iq_roundtrip() {
        let path = temp_path("stereo_iq");
        let iq: Vec<Complex64> = (0..500).map(|i| {
            let t = i as f64 / 48000.0;
            Complex64::new(
                (2.0 * PI * 1000.0 * t).cos(),
                (2.0 * PI * 1000.0 * t).sin(),
            )
        }).collect();

        let mut sink = WavFileSink::create_stereo_iq(&path, 48000).unwrap();
        sink.write_complex_as_stereo(&iq).unwrap();
        sink.close().unwrap();

        let mut source = WavFileSource::open(&path).unwrap();
        assert_eq!(source.channels(), 2);
        assert_eq!(source.num_samples(), 500);

        let read_back = source.read_stereo_iq(500).unwrap();
        assert_eq!(read_back.len(), 500);

        for (a, b) in iq.iter().zip(read_back.iter()) {
            assert!((a.re - b.re).abs() < 0.001);
            assert!((a.im - b.im).abs() < 0.001);
        }

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_seek() {
        let path = temp_path("seek");
        let samples: Vec<f64> = (0..1000).map(|i| i as f64 / 1000.0).collect();

        let mut sink = WavFileSink::create_mono(&path, 44100).unwrap();
        sink.write_samples(&samples).unwrap();
        sink.close().unwrap();

        let mut source = WavFileSource::open(&path).unwrap();
        source.seek(500).unwrap();
        let data = source.read_samples(10).unwrap();
        assert_eq!(data.len(), 10);
        // First sample should be around 0.5 (index 500 / 1000)
        assert!((data[0] - 0.5).abs() < 0.01);

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_eof() {
        let path = temp_path("eof");
        let mut sink = WavFileSink::create_mono(&path, 44100).unwrap();
        sink.write_samples(&[0.0; 10]).unwrap();
        sink.close().unwrap();

        let mut source = WavFileSource::open(&path).unwrap();
        assert!(!source.is_eof());
        source.read_all().unwrap();
        assert!(source.is_eof());

        // Reading past EOF returns empty
        let data = source.read_samples(100).unwrap();
        assert!(data.is_empty());

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_samples_written_count() {
        let path = temp_path("count");
        let mut sink = WavFileSink::create_mono(&path, 44100).unwrap();
        assert_eq!(sink.samples_written(), 0);
        sink.write_samples(&[0.0; 100]).unwrap();
        assert_eq!(sink.samples_written(), 100);
        sink.write_samples(&[0.0; 50]).unwrap();
        assert_eq!(sink.samples_written(), 150);
        sink.close().unwrap();

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_8bit_pcm() {
        let path = temp_path("8bit");
        let mut sink = WavFileSink::create_with_format(&path, 8000, 1, 8, 1).unwrap();
        sink.write_samples(&[0.0, 0.5, -0.5, 1.0, -1.0]).unwrap();
        sink.close().unwrap();

        let mut source = WavFileSource::open(&path).unwrap();
        assert_eq!(source.bits_per_sample(), 8);
        let data = source.read_all().unwrap();
        assert_eq!(data.len(), 5);
        // 8-bit has coarse quantization
        assert!(data[0].abs() < 0.05); // ~0
        assert!(data[1] > 0.3); // ~0.5

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_float32() {
        let path = temp_path("float32");
        let mut sink = WavFileSink::create_with_format(&path, 48000, 1, 32, 3).unwrap();
        let samples = vec![0.1, 0.2, 0.3, -0.5, 0.999];
        sink.write_samples(&samples).unwrap();
        sink.close().unwrap();

        let mut source = WavFileSource::open(&path).unwrap();
        assert_eq!(source.bits_per_sample(), 32);
        let data = source.read_all().unwrap();
        assert_eq!(data.len(), 5);
        for (a, b) in samples.iter().zip(data.iter()) {
            assert!((a - b).abs() < 1e-6);
        }

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_empty_file() {
        let path = temp_path("empty");
        let sink = WavFileSink::create_mono(&path, 44100).unwrap();
        sink.close().unwrap();

        let mut source = WavFileSource::open(&path).unwrap();
        assert_eq!(source.num_samples(), 0);
        assert!(source.is_eof());

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_invalid_file() {
        let path = temp_path("invalid");
        std::fs::write(&path, b"not a wav file").unwrap();
        assert!(WavFileSource::open(&path).is_err());
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn test_sample_rate_accessor() {
        let path = temp_path("sr_access");
        let sink = WavFileSink::create_mono(&path, 22050).unwrap();
        assert_eq!(sink.sample_rate(), 22050);
        sink.close().unwrap();
        std::fs::remove_file(&path).ok();
    }
}

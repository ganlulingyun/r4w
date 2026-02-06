//! # SigMF File I/O Driver
//!
//! Provides file-based SDR device for reading and writing SigMF recordings.
//!
//! ## SigMF Format
//!
//! SigMF (Signal Metadata Format) is an open standard for describing
//! signal recordings. It consists of:
//!
//! - **Data file** (`.sigmf-data`): Raw I/Q samples
//! - **Metadata file** (`.sigmf-meta`): JSON description of the recording
//!
//! ## Supported Sample Formats
//!
//! - `cf32_le`: Complex float32, little-endian (native R4W format)
//! - `ci16_le`: Complex int16, little-endian
//! - `ci8`: Complex int8
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_sim::hal::sigmf::{SigMfReader, SigMfWriter};
//!
//! // Read a SigMF recording
//! let mut reader = SigMfReader::open("recording.sigmf-meta")?;
//! println!("Sample rate: {} Hz", reader.sample_rate());
//! println!("Frequency: {} Hz", reader.frequency());
//!
//! let samples = reader.read_samples(1024)?;
//!
//! // Write a SigMF recording
//! let mut writer = SigMfWriter::create("output", 1e6, 915e6)?;
//! writer.write_samples(&samples)?;
//! writer.close()?;
//! ```

use r4w_core::io::IqFormat;
use r4w_core::timing::Timestamp;
use r4w_core::types::IQSample;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use crate::device::{SdrError, SdrResult};

/// SigMF global metadata.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SigMfGlobal {
    /// SigMF core namespace
    #[serde(rename = "core:datatype")]
    pub datatype: String,

    /// Sample rate in Hz
    #[serde(rename = "core:sample_rate")]
    pub sample_rate: f64,

    /// Version of SigMF spec
    #[serde(rename = "core:version")]
    pub version: String,

    /// Number of channels (optional)
    #[serde(rename = "core:num_channels", skip_serializing_if = "Option::is_none")]
    pub num_channels: Option<u32>,

    /// SHA512 hash of data file (optional)
    #[serde(rename = "core:sha512", skip_serializing_if = "Option::is_none")]
    pub sha512: Option<String>,

    /// Offset in data file (optional)
    #[serde(rename = "core:offset", skip_serializing_if = "Option::is_none")]
    pub offset: Option<u64>,

    /// Description (optional)
    #[serde(rename = "core:description", skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    /// Author (optional)
    #[serde(rename = "core:author", skip_serializing_if = "Option::is_none")]
    pub author: Option<String>,

    /// Recording date/time (optional)
    #[serde(rename = "core:datetime", skip_serializing_if = "Option::is_none")]
    pub datetime: Option<String>,

    /// License (optional)
    #[serde(rename = "core:license", skip_serializing_if = "Option::is_none")]
    pub license: Option<String>,

    /// Hardware used (optional)
    #[serde(rename = "core:hw", skip_serializing_if = "Option::is_none")]
    pub hw: Option<String>,

    /// R4W-specific: waveform name (optional)
    #[serde(rename = "r4w:waveform", skip_serializing_if = "Option::is_none")]
    pub r4w_waveform: Option<String>,

    /// Additional extensions
    #[serde(flatten)]
    pub extensions: HashMap<String, serde_json::Value>,
}

impl Default for SigMfGlobal {
    fn default() -> Self {
        Self {
            datatype: "cf32_le".to_string(),
            sample_rate: 1_000_000.0,
            version: "1.0.0".to_string(),
            num_channels: Some(1),
            sha512: None,
            offset: None,
            description: None,
            author: None,
            datetime: None,
            license: None,
            hw: None,
            r4w_waveform: None,
            extensions: HashMap::new(),
        }
    }
}

/// SigMF capture segment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SigMfCapture {
    /// Sample index where this capture starts
    #[serde(rename = "core:sample_start")]
    pub sample_start: u64,

    /// Center frequency in Hz
    #[serde(rename = "core:frequency", skip_serializing_if = "Option::is_none")]
    pub frequency: Option<f64>,

    /// Date/time of capture start
    #[serde(rename = "core:datetime", skip_serializing_if = "Option::is_none")]
    pub datetime: Option<String>,

    /// Additional fields
    #[serde(flatten)]
    pub extensions: HashMap<String, serde_json::Value>,
}

impl Default for SigMfCapture {
    fn default() -> Self {
        Self {
            sample_start: 0,
            frequency: None,
            datetime: None,
            extensions: HashMap::new(),
        }
    }
}

/// SigMF annotation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SigMfAnnotation {
    /// Sample index where annotation starts
    #[serde(rename = "core:sample_start")]
    pub sample_start: u64,

    /// Number of samples in annotation
    #[serde(rename = "core:sample_count")]
    pub sample_count: u64,

    /// Frequency lower bound (optional)
    #[serde(rename = "core:freq_lower_edge", skip_serializing_if = "Option::is_none")]
    pub freq_lower_edge: Option<f64>,

    /// Frequency upper bound (optional)
    #[serde(rename = "core:freq_upper_edge", skip_serializing_if = "Option::is_none")]
    pub freq_upper_edge: Option<f64>,

    /// Label/description (optional)
    #[serde(rename = "core:label", skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,

    /// Comment (optional)
    #[serde(rename = "core:comment", skip_serializing_if = "Option::is_none")]
    pub comment: Option<String>,

    /// Additional fields
    #[serde(flatten)]
    pub extensions: HashMap<String, serde_json::Value>,
}

/// Complete SigMF metadata structure.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SigMfMeta {
    /// Global metadata
    pub global: SigMfGlobal,

    /// Capture segments
    pub captures: Vec<SigMfCapture>,

    /// Annotations
    pub annotations: Vec<SigMfAnnotation>,
}

impl Default for SigMfMeta {
    fn default() -> Self {
        Self {
            global: SigMfGlobal::default(),
            captures: vec![SigMfCapture::default()],
            annotations: Vec::new(),
        }
    }
}

impl SigMfMeta {
    /// Create new metadata with basic parameters.
    pub fn new(sample_rate: f64, frequency: f64, datatype: &str) -> Self {
        let mut meta = Self::default();
        meta.global.sample_rate = sample_rate;
        meta.global.datatype = datatype.to_string();
        meta.captures[0].frequency = Some(frequency);
        meta
    }

    /// Set description.
    pub fn with_description(mut self, desc: &str) -> Self {
        self.global.description = Some(desc.to_string());
        self
    }

    /// Set author.
    pub fn with_author(mut self, author: &str) -> Self {
        self.global.author = Some(author.to_string());
        self
    }

    /// Set waveform name (R4W extension).
    pub fn with_waveform(mut self, waveform: &str) -> Self {
        self.global.r4w_waveform = Some(waveform.to_string());
        self
    }

    /// Add an annotation.
    pub fn add_annotation(&mut self, start: u64, count: u64, label: &str) {
        self.annotations.push(SigMfAnnotation {
            sample_start: start,
            sample_count: count,
            label: Some(label.to_string()),
            freq_lower_edge: None,
            freq_upper_edge: None,
            comment: None,
            extensions: HashMap::new(),
        });
    }
}

/// Sample format information.
///
/// This struct provides metadata about SigMF sample formats.
/// For I/O operations, convert to `IqFormat` using `to_iq_format()`.
#[derive(Debug, Clone, Copy)]
pub struct SampleFormat {
    /// Bytes per sample (both I and Q)
    pub bytes_per_sample: usize,
    /// Is complex (I/Q)?
    pub is_complex: bool,
    /// Is floating point?
    pub is_float: bool,
    /// Bits per component
    pub bits: usize,
    /// Corresponding IqFormat (None for real-only formats)
    iq_format: Option<IqFormat>,
}

impl SampleFormat {
    /// Parse SigMF datatype string.
    pub fn from_datatype(datatype: &str) -> Option<Self> {
        match datatype {
            "cf32_le" | "cf32" => Some(Self {
                bytes_per_sample: 8,
                is_complex: true,
                is_float: true,
                bits: 32,
                iq_format: Some(IqFormat::Cf32),
            }),
            "cf64_le" | "cf64" => Some(Self {
                bytes_per_sample: 16,
                is_complex: true,
                is_float: true,
                bits: 64,
                iq_format: Some(IqFormat::Cf64),
            }),
            "ci16_le" | "ci16" => Some(Self {
                bytes_per_sample: 4,
                is_complex: true,
                is_float: false,
                bits: 16,
                iq_format: Some(IqFormat::Ci16),
            }),
            "ci8" => Some(Self {
                bytes_per_sample: 2,
                is_complex: true,
                is_float: false,
                bits: 8,
                iq_format: Some(IqFormat::Ci8),
            }),
            // Unsigned 8-bit complex (RTL-SDR native format)
            "cu8" => Some(Self {
                bytes_per_sample: 2,
                is_complex: true,
                is_float: false,
                bits: 8,
                iq_format: Some(IqFormat::Cu8),
            }),
            "ri32_le" | "ri32" => Some(Self {
                bytes_per_sample: 4,
                is_complex: false,
                is_float: false,
                bits: 32,
                iq_format: None, // Real-only format
            }),
            "rf32_le" | "rf32" => Some(Self {
                bytes_per_sample: 4,
                is_complex: false,
                is_float: true,
                bits: 32,
                iq_format: None, // Real-only format
            }),
            _ => None,
        }
    }

    /// Get the unified IqFormat for complex formats.
    ///
    /// Returns `None` for real-only formats (ri32, rf32).
    pub fn to_iq_format(&self) -> Option<IqFormat> {
        self.iq_format
    }

    /// Create from IqFormat
    pub fn from_iq_format(fmt: IqFormat) -> Self {
        Self {
            bytes_per_sample: fmt.bytes_per_sample(),
            is_complex: true,
            is_float: matches!(fmt, IqFormat::Cf32 | IqFormat::Cf64),
            bits: match fmt {
                IqFormat::Cf64 => 64,
                IqFormat::Cf32 => 32,
                IqFormat::Ci16 => 16,
                IqFormat::Ci8 | IqFormat::Cu8 => 8,
            },
            iq_format: Some(fmt),
        }
    }
}

/// SigMF file reader.
pub struct SigMfReader {
    /// Metadata
    meta: SigMfMeta,
    /// Sample format
    format: SampleFormat,
    /// Data file reader
    data_file: BufReader<File>,
    /// Base path (without extension) - kept for future extensions
    #[allow(dead_code)]
    base_path: PathBuf,
    /// Current sample position
    position: u64,
    /// Total samples in file
    total_samples: u64,
}

impl SigMfReader {
    /// Open a SigMF recording.
    ///
    /// Pass either the `.sigmf-meta` file or the base name.
    pub fn open<P: AsRef<Path>>(path: P) -> SdrResult<Self> {
        let path = path.as_ref();

        // Determine base path
        let base_path = if path.extension().map_or(false, |e| e == "sigmf-meta") {
            path.with_extension("")
        } else if path.extension().map_or(false, |e| e == "sigmf-data") {
            path.with_extension("")
        } else {
            path.to_path_buf()
        };

        let meta_path = base_path.with_extension("sigmf-meta");
        let data_path = base_path.with_extension("sigmf-data");

        // Read metadata
        let meta_file = File::open(&meta_path).map_err(|e| {
            SdrError::ConfigError(format!("Failed to open {}: {}", meta_path.display(), e))
        })?;
        let meta: SigMfMeta = serde_json::from_reader(BufReader::new(meta_file)).map_err(|e| {
            SdrError::ConfigError(format!("Failed to parse metadata: {}", e))
        })?;

        // Parse sample format
        let format = SampleFormat::from_datatype(&meta.global.datatype).ok_or_else(|| {
            SdrError::ConfigError(format!("Unsupported datatype: {}", meta.global.datatype))
        })?;

        // Open data file
        let data_file = File::open(&data_path).map_err(|e| {
            SdrError::ConfigError(format!("Failed to open {}: {}", data_path.display(), e))
        })?;

        // Calculate total samples
        let file_size = data_file.metadata().map_err(|e| {
            SdrError::HardwareError(format!("Failed to get file size: {}", e))
        })?.len();
        let total_samples = file_size / format.bytes_per_sample as u64;

        Ok(Self {
            meta,
            format,
            data_file: BufReader::new(data_file),
            base_path,
            position: 0,
            total_samples,
        })
    }

    /// Get metadata.
    pub fn metadata(&self) -> &SigMfMeta {
        &self.meta
    }

    /// Get sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.meta.global.sample_rate
    }

    /// Get center frequency.
    pub fn frequency(&self) -> f64 {
        self.meta.captures.first()
            .and_then(|c| c.frequency)
            .unwrap_or(0.0)
    }

    /// Get total number of samples.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get current position.
    pub fn position(&self) -> u64 {
        self.position
    }

    /// Get remaining samples.
    pub fn remaining(&self) -> u64 {
        self.total_samples.saturating_sub(self.position)
    }

    /// Seek to sample position.
    pub fn seek(&mut self, sample: u64) -> SdrResult<()> {
        let byte_offset = sample * self.format.bytes_per_sample as u64;
        self.data_file.seek(SeekFrom::Start(byte_offset)).map_err(|e| {
            SdrError::HardwareError(format!("Seek failed: {}", e))
        })?;
        self.position = sample;
        Ok(())
    }

    /// Read samples into buffer.
    ///
    /// Returns the number of samples actually read.
    /// Uses the unified `IqFormat` for parsing.
    pub fn read_samples(&mut self, buffer: &mut [IQSample]) -> SdrResult<usize> {
        let to_read = buffer.len().min(self.remaining() as usize);
        if to_read == 0 {
            return Ok(0);
        }

        // Get IqFormat for this datatype
        let iq_format = self.format.to_iq_format().ok_or_else(|| {
            SdrError::Unsupported(format!(
                "Datatype {} is not a complex format",
                self.meta.global.datatype
            ))
        })?;

        // Read raw bytes
        let bytes_needed = to_read * iq_format.bytes_per_sample();
        let mut byte_buf = vec![0u8; bytes_needed];
        self.data_file.read_exact(&mut byte_buf).map_err(|e| {
            SdrError::HardwareError(format!("Read failed: {}", e))
        })?;

        // Parse using unified IqFormat
        let samples = iq_format.parse_bytes(&byte_buf);
        buffer[..to_read].copy_from_slice(&samples);

        self.position += to_read as u64;
        Ok(to_read)
    }

    /// Read all remaining samples.
    pub fn read_all(&mut self) -> SdrResult<Vec<IQSample>> {
        let remaining = self.remaining() as usize;
        let mut buffer = vec![IQSample::new(0.0, 0.0); remaining];
        self.read_samples(&mut buffer)?;
        Ok(buffer)
    }

    /// Create timestamp for current position.
    pub fn timestamp(&self) -> Timestamp {
        Timestamp::at_sample(self.position, self.sample_rate())
    }
}

/// SigMF file writer.
pub struct SigMfWriter {
    /// Metadata
    meta: SigMfMeta,
    /// Data file writer
    data_file: BufWriter<File>,
    /// Base path (without extension)
    base_path: PathBuf,
    /// Samples written
    samples_written: u64,
}

impl SigMfWriter {
    /// Create a new SigMF recording.
    ///
    /// Pass the base name (without extension).
    pub fn create<P: AsRef<Path>>(
        path: P,
        sample_rate: f64,
        frequency: f64,
    ) -> SdrResult<Self> {
        Self::create_with_format(path, sample_rate, frequency, "cf32_le")
    }

    /// Create with specific sample format.
    pub fn create_with_format<P: AsRef<Path>>(
        path: P,
        sample_rate: f64,
        frequency: f64,
        datatype: &str,
    ) -> SdrResult<Self> {
        let base_path = path.as_ref().to_path_buf();
        let data_path = base_path.with_extension("sigmf-data");

        // Validate datatype
        if SampleFormat::from_datatype(datatype).is_none() {
            return Err(SdrError::ConfigError(format!(
                "Unsupported datatype: {}",
                datatype
            )));
        }

        // Create data file
        let data_file = File::create(&data_path).map_err(|e| {
            SdrError::ConfigError(format!("Failed to create {}: {}", data_path.display(), e))
        })?;

        // Create metadata
        let mut meta = SigMfMeta::new(sample_rate, frequency, datatype);
        meta.global.datetime = Some(chrono::Utc::now().to_rfc3339());
        meta.global.author = Some("R4W".to_string());
        meta.captures[0].datetime = meta.global.datetime.clone();

        Ok(Self {
            meta,
            data_file: BufWriter::new(data_file),
            base_path,
            samples_written: 0,
        })
    }

    /// Set description.
    pub fn set_description(&mut self, desc: &str) {
        self.meta.global.description = Some(desc.to_string());
    }

    /// Set waveform name.
    pub fn set_waveform(&mut self, waveform: &str) {
        self.meta.global.r4w_waveform = Some(waveform.to_string());
    }

    /// Set hardware description.
    pub fn set_hardware(&mut self, hw: &str) {
        self.meta.global.hw = Some(hw.to_string());
    }

    /// Add an annotation.
    pub fn add_annotation(&mut self, start: u64, count: u64, label: &str) {
        self.meta.add_annotation(start, count, label);
    }

    /// Get samples written.
    pub fn samples_written(&self) -> u64 {
        self.samples_written
    }

    /// Write samples.
    ///
    /// Uses the unified `IqFormat` for serialization.
    pub fn write_samples(&mut self, samples: &[IQSample]) -> SdrResult<usize> {
        // Get IqFormat for this datatype
        let format = SampleFormat::from_datatype(&self.meta.global.datatype)
            .ok_or_else(|| {
                SdrError::Unsupported(format!(
                    "Datatype {} not supported",
                    self.meta.global.datatype
                ))
            })?;

        let iq_format = format.to_iq_format().ok_or_else(|| {
            SdrError::Unsupported(format!(
                "Datatype {} is not a complex format",
                self.meta.global.datatype
            ))
        })?;

        // Serialize using unified IqFormat
        let bytes = iq_format.to_bytes(samples);
        self.data_file.write_all(&bytes).map_err(|e| {
            SdrError::HardwareError(format!("Write failed: {}", e))
        })?;

        self.samples_written += samples.len() as u64;
        Ok(samples.len())
    }

    /// Flush buffers.
    pub fn flush(&mut self) -> SdrResult<()> {
        self.data_file.flush().map_err(|e| {
            SdrError::HardwareError(format!("Flush failed: {}", e))
        })
    }

    /// Close and finalize the recording.
    pub fn close(mut self) -> SdrResult<SigMfMeta> {
        // Flush data
        self.flush()?;

        // Write metadata
        let meta_path = self.base_path.with_extension("sigmf-meta");
        let meta_file = File::create(&meta_path).map_err(|e| {
            SdrError::ConfigError(format!("Failed to create {}: {}", meta_path.display(), e))
        })?;

        serde_json::to_writer_pretty(BufWriter::new(meta_file), &self.meta).map_err(|e| {
            SdrError::ConfigError(format!("Failed to write metadata: {}", e))
        })?;

        Ok(self.meta)
    }
}

/// Helper to annotate samples with waveform information.
pub fn annotate_waveform(
    meta: &mut SigMfMeta,
    waveform: &str,
    start: u64,
    count: u64,
    params: &str,
) {
    let mut ann = SigMfAnnotation {
        sample_start: start,
        sample_count: count,
        label: Some(waveform.to_string()),
        comment: Some(params.to_string()),
        freq_lower_edge: None,
        freq_upper_edge: None,
        extensions: HashMap::new(),
    };
    ann.extensions.insert(
        "r4w:waveform".to_string(),
        serde_json::Value::String(waveform.to_string()),
    );
    meta.annotations.push(ann);
}

// =============================================================================
// SigMF Device Driver - File-based SDR Device
// =============================================================================

use super::{
    ClockControl, DeviceDriver, DeviceInfo,
    SdrDeviceExt, StreamConfig, StreamDirection, StreamHandle, StreamStatus, TunerControl,
};
use crate::device::{DeviceCapabilities, SdrConfig};

/// SigMF-based SDR device for file I/O.
///
/// This device allows using SigMF recordings as if they were live SDR devices.
/// Supports both reading (RX) and writing (TX) operations.
///
/// # Example
///
/// ```rust,ignore
/// use r4w_sim::hal::{create_default_registry, SdrDeviceExt};
///
/// let registry = create_default_registry();
/// let device = registry.create("file://recording.sigmf-meta")?;
/// ```
pub struct SigMfDevice {
    name: String,
    config: SdrConfig,
    reader: Option<SigMfReader>,
    writer: Option<SigMfWriter>,
    #[allow(dead_code)]
    base_path: PathBuf,
    mode: SigMfMode,
}

/// File device mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SigMfMode {
    /// Read from existing file
    Read,
    /// Write to new file
    Write,
    /// Not yet determined (will be set on first operation)
    Unset,
}

impl SigMfDevice {
    /// Open a SigMF file for reading.
    pub fn open_read<P: AsRef<Path>>(path: P) -> SdrResult<Self> {
        let reader = SigMfReader::open(&path)?;
        let meta = reader.metadata();

        let config = SdrConfig {
            frequency: meta.captures.first()
                .and_then(|c| c.frequency)
                .unwrap_or(0.0),
            sample_rate: meta.global.sample_rate,
            bandwidth: meta.global.sample_rate,
            rx_gain: 0.0,
            tx_gain: 0.0,
            antenna: "FILE".to_string(),
            buffer_size: 8192,
        };

        Ok(Self {
            name: format!("SigMF: {}", path.as_ref().display()),
            config,
            reader: Some(reader),
            writer: None,
            base_path: path.as_ref().to_path_buf(),
            mode: SigMfMode::Read,
        })
    }

    /// Create a new SigMF file for writing.
    pub fn create_write<P: AsRef<Path>>(
        path: P,
        sample_rate: f64,
        frequency: f64,
    ) -> SdrResult<Self> {
        let writer = SigMfWriter::create(&path, sample_rate, frequency)?;

        let config = SdrConfig {
            frequency,
            sample_rate,
            bandwidth: sample_rate,
            rx_gain: 0.0,
            tx_gain: 0.0,
            antenna: "FILE".to_string(),
            buffer_size: 8192,
        };

        Ok(Self {
            name: format!("SigMF: {}", path.as_ref().display()),
            config,
            reader: None,
            writer: Some(writer),
            base_path: path.as_ref().to_path_buf(),
            mode: SigMfMode::Write,
        })
    }

    /// Get metadata (if reading).
    pub fn metadata(&self) -> Option<&SigMfMeta> {
        self.reader.as_ref().map(|r| r.metadata())
    }

    /// Get total samples (if reading).
    pub fn total_samples(&self) -> u64 {
        self.reader.as_ref().map(|r| r.total_samples()).unwrap_or(0)
    }

    /// Get samples written (if writing).
    pub fn samples_written(&self) -> u64 {
        self.writer.as_ref().map(|w| w.samples_written()).unwrap_or(0)
    }
}

/// Stream handle for SigMF file operations
struct SigMfStream {
    direction: StreamDirection,
    reader: Option<SigMfReader>,
    writer: Option<SigMfWriter>,
    running: bool,
    samples_processed: u64,
    sample_rate: f64,
}

impl StreamHandle for SigMfStream {
    fn direction(&self) -> StreamDirection {
        self.direction
    }

    fn start(&mut self) -> SdrResult<()> {
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> SdrResult<()> {
        self.running = false;
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.running
    }

    fn read(
        &mut self,
        buffer: &mut [IQSample],
        _timeout: std::time::Duration,
    ) -> SdrResult<(usize, Timestamp)> {
        if !self.running {
            return Err(SdrError::NotStarted);
        }

        let reader = self.reader.as_mut().ok_or_else(|| {
            SdrError::ConfigError("No reader available".to_string())
        })?;

        let timestamp = reader.timestamp();
        let count = reader.read_samples(buffer)?;
        self.samples_processed += count as u64;

        Ok((count, timestamp))
    }

    fn write(
        &mut self,
        buffer: &[IQSample],
        _timestamp: Option<Timestamp>,
        _timeout: std::time::Duration,
    ) -> SdrResult<usize> {
        if !self.running {
            return Err(SdrError::NotStarted);
        }

        let writer = self.writer.as_mut().ok_or_else(|| {
            SdrError::ConfigError("No writer available".to_string())
        })?;

        let count = writer.write_samples(buffer)?;
        self.samples_processed += count as u64;

        Ok(count)
    }

    fn status(&self) -> StreamStatus {
        StreamStatus {
            overflow_count: 0,
            underflow_count: 0,
            late_count: 0,
            samples_processed: self.samples_processed,
            buffer_level: 0,
        }
    }

    fn timestamp(&self) -> Timestamp {
        Timestamp::at_sample(self.samples_processed, self.sample_rate)
    }

    fn available(&self) -> usize {
        self.reader.as_ref().map(|r| r.remaining() as usize).unwrap_or(0)
    }

    fn free_space(&self) -> usize {
        // Files have unlimited space
        usize::MAX
    }
}

/// Tuner control for file device (mostly no-ops)
#[allow(dead_code)]
struct SigMfTuner {
    config: SdrConfig,
}

impl TunerControl for SigMfTuner {
    fn set_frequency(&mut self, freq_hz: u64) -> SdrResult<u64> {
        self.config.frequency = freq_hz as f64;
        Ok(freq_hz)
    }

    fn frequency(&self) -> u64 {
        self.config.frequency as u64
    }

    fn set_sample_rate(&mut self, _rate: f64) -> SdrResult<f64> {
        // Cannot change sample rate of existing file
        Ok(self.config.sample_rate)
    }

    fn sample_rate(&self) -> f64 {
        self.config.sample_rate
    }

    fn set_bandwidth(&mut self, bw_hz: f64) -> SdrResult<f64> {
        self.config.bandwidth = bw_hz;
        Ok(bw_hz)
    }

    fn bandwidth(&self) -> f64 {
        self.config.bandwidth
    }

    fn set_rx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        self.config.rx_gain = gain_db;
        Ok(gain_db)
    }

    fn rx_gain(&self) -> f64 {
        self.config.rx_gain
    }

    fn set_tx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        self.config.tx_gain = gain_db;
        Ok(gain_db)
    }

    fn tx_gain(&self) -> f64 {
        self.config.tx_gain
    }

    fn set_antenna(&mut self, antenna: &str) -> SdrResult<()> {
        self.config.antenna = antenna.to_string();
        Ok(())
    }

    fn antenna(&self) -> &str {
        &self.config.antenna
    }

    fn available_antennas(&self) -> Vec<String> {
        vec!["FILE".to_string()]
    }

    fn frequency_range(&self) -> (u64, u64) {
        (0, u64::MAX)
    }

    fn sample_rate_range(&self) -> (f64, f64) {
        (1.0, 1e12)
    }

    fn gain_range(&self) -> (f64, f64) {
        (-100.0, 100.0)
    }
}

impl SdrDeviceExt for SigMfDevice {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities {
            can_tx: self.mode == SigMfMode::Write || self.mode == SigMfMode::Unset,
            can_rx: self.mode == SigMfMode::Read,
            full_duplex: false,
            min_frequency: 0.0,
            max_frequency: 1e12,
            max_sample_rate: 1e12,
            tx_channels: 1,
            rx_channels: 1,
        }
    }

    fn config(&self) -> &SdrConfig {
        &self.config
    }

    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()> {
        // Only update what we can (most is fixed by file)
        self.config.rx_gain = config.rx_gain;
        self.config.tx_gain = config.tx_gain;
        Ok(())
    }

    fn create_rx_stream(&mut self, _config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        if self.mode != SigMfMode::Read {
            return Err(SdrError::ConfigError("Device not open for reading".to_string()));
        }

        // Take ownership of reader
        let reader = self.reader.take().ok_or_else(|| {
            SdrError::ConfigError("Reader already in use".to_string())
        })?;

        Ok(Box::new(SigMfStream {
            direction: StreamDirection::Rx,
            reader: Some(reader),
            writer: None,
            running: false,
            samples_processed: 0,
            sample_rate: self.config.sample_rate,
        }))
    }

    fn create_tx_stream(&mut self, _config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        if self.mode != SigMfMode::Write {
            return Err(SdrError::ConfigError("Device not open for writing".to_string()));
        }

        // Take ownership of writer
        let writer = self.writer.take().ok_or_else(|| {
            SdrError::ConfigError("Writer already in use".to_string())
        })?;

        Ok(Box::new(SigMfStream {
            direction: StreamDirection::Tx,
            reader: None,
            writer: Some(writer),
            running: false,
            samples_processed: 0,
            sample_rate: self.config.sample_rate,
        }))
    }

    fn tuner(&mut self) -> &mut dyn TunerControl {
        // This is a bit awkward - we need to store a tuner
        // For now, create a static one. In real implementation,
        // the tuner would be a field.
        unimplemented!("File device tuner access requires refactoring")
    }

    fn clock(&mut self) -> Option<&mut dyn ClockControl> {
        None // Files don't have clocks
    }
}

/// File device driver for the registry.
pub struct FileDriver;

impl FileDriver {
    pub fn new() -> Self {
        Self
    }
}

impl Default for FileDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceDriver for FileDriver {
    fn name(&self) -> &str {
        "file"
    }

    fn discover(&self) -> Vec<DeviceInfo> {
        // Files aren't discoverable
        Vec::new()
    }

    fn create(&self, info: &DeviceInfo) -> SdrResult<Box<dyn SdrDeviceExt>> {
        SigMfDevice::open_read(&info.address)
            .map(|d| Box::new(d) as Box<dyn SdrDeviceExt>)
    }

    fn create_from_string(&self, args: &str) -> SdrResult<Box<dyn SdrDeviceExt>> {
        // Parse args: could be just a path, or "path,mode=write,rate=1e6,freq=915e6"
        if args.contains(',') {
            // Parse key=value pairs
            let parts: Vec<&str> = args.split(',').collect();
            let path = parts[0];

            let mut mode = "read";
            let mut rate = 1e6;
            let mut freq = 915e6;

            for part in &parts[1..] {
                if let Some((key, value)) = part.split_once('=') {
                    match key.trim() {
                        "mode" => mode = value.trim(),
                        "rate" => rate = value.trim().parse().unwrap_or(1e6),
                        "freq" => freq = value.trim().parse().unwrap_or(915e6),
                        _ => {}
                    }
                }
            }

            if mode == "write" {
                SigMfDevice::create_write(path, rate, freq)
                    .map(|d| Box::new(d) as Box<dyn SdrDeviceExt>)
            } else {
                SigMfDevice::open_read(path)
                    .map(|d| Box::new(d) as Box<dyn SdrDeviceExt>)
            }
        } else {
            // Just a path - open for reading
            SigMfDevice::open_read(args)
                .map(|d| Box::new(d) as Box<dyn SdrDeviceExt>)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_sample_format_parsing() {
        let cf32 = SampleFormat::from_datatype("cf32_le").unwrap();
        assert_eq!(cf32.bytes_per_sample, 8);
        assert!(cf32.is_complex);
        assert!(cf32.is_float);

        let ci16 = SampleFormat::from_datatype("ci16_le").unwrap();
        assert_eq!(ci16.bytes_per_sample, 4);
        assert!(ci16.is_complex);
        assert!(!ci16.is_float);

        let ci8 = SampleFormat::from_datatype("ci8").unwrap();
        assert_eq!(ci8.bytes_per_sample, 2);
    }

    #[test]
    fn test_sigmf_metadata_creation() {
        let meta = SigMfMeta::new(1e6, 915e6, "cf32_le")
            .with_description("Test recording")
            .with_author("R4W Tests");

        assert_eq!(meta.global.sample_rate, 1e6);
        assert_eq!(meta.captures[0].frequency, Some(915e6));
        assert_eq!(meta.global.description, Some("Test recording".to_string()));
    }

    #[test]
    fn test_write_read_roundtrip() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path().join("test");

        // Generate test samples
        let samples: Vec<IQSample> = (0..1000)
            .map(|i| {
                let phase = i as f64 * 0.1;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect();

        // Write
        {
            let mut writer = SigMfWriter::create(&base_path, 1e6, 915e6).unwrap();
            writer.set_description("Test recording");
            writer.write_samples(&samples).unwrap();
            writer.close().unwrap();
        }

        // Read back
        {
            let mut reader = SigMfReader::open(base_path.with_extension("sigmf-meta")).unwrap();
            assert_eq!(reader.sample_rate(), 1e6);
            assert_eq!(reader.frequency(), 915e6);
            assert_eq!(reader.total_samples(), 1000);

            let mut read_samples = vec![IQSample::new(0.0, 0.0); 1000];
            let count = reader.read_samples(&mut read_samples).unwrap();
            assert_eq!(count, 1000);

            // Compare
            for (orig, read) in samples.iter().zip(read_samples.iter()) {
                assert!((orig.re - read.re).abs() < 1e-5);
                assert!((orig.im - read.im).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn test_ci16_format() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path().join("test_ci16");

        let samples: Vec<IQSample> = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(1.0, 0.0),
            IQSample::new(0.0, 1.0),
        ];

        // Write as ci16
        {
            let mut writer =
                SigMfWriter::create_with_format(&base_path, 1e6, 915e6, "ci16_le").unwrap();
            writer.write_samples(&samples).unwrap();
            writer.close().unwrap();
        }

        // Read back
        {
            let mut reader = SigMfReader::open(base_path.with_extension("sigmf-meta")).unwrap();
            let mut read_samples = vec![IQSample::new(0.0, 0.0); 3];
            reader.read_samples(&mut read_samples).unwrap();

            // ci16 has less precision
            for (orig, read) in samples.iter().zip(read_samples.iter()) {
                assert!((orig.re - read.re).abs() < 0.001);
                assert!((orig.im - read.im).abs() < 0.001);
            }
        }
    }

    #[test]
    fn test_seek() {
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path().join("test_seek");

        let samples: Vec<IQSample> = (0..100)
            .map(|i| IQSample::new(i as f64, 0.0))
            .collect();

        {
            let mut writer = SigMfWriter::create(&base_path, 1e6, 915e6).unwrap();
            writer.write_samples(&samples).unwrap();
            writer.close().unwrap();
        }

        {
            let mut reader = SigMfReader::open(base_path.with_extension("sigmf-meta")).unwrap();

            // Seek to middle
            reader.seek(50).unwrap();
            assert_eq!(reader.position(), 50);

            let mut buf = vec![IQSample::new(0.0, 0.0); 1];
            reader.read_samples(&mut buf).unwrap();
            assert!((buf[0].re - 50.0).abs() < 1e-5);
        }
    }

    // =========================================================================
    // GNU Radio Compatibility Tests (MF-023)
    // =========================================================================

    #[test]
    fn test_cu8_format_rtlsdr_compat() {
        // cu8 is the native format for RTL-SDR and commonly used with GNU Radio
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path().join("test_cu8");

        // Test full range of values
        let samples: Vec<IQSample> = vec![
            IQSample::new(-1.0, -1.0),  // Should map to (0, 0)
            IQSample::new(0.0, 0.0),     // Should map to (127, 127)
            IQSample::new(1.0, 1.0),     // Should map to (255, 255)
            IQSample::new(0.5, -0.5),    // Should map to (191, 63) approximately
        ];

        // Write as cu8
        {
            let mut writer =
                SigMfWriter::create_with_format(&base_path, 2.4e6, 100e6, "cu8").unwrap();
            writer.set_description("RTL-SDR compatible test recording");
            writer.set_hardware("R4W Test");
            writer.write_samples(&samples).unwrap();
            writer.close().unwrap();
        }

        // Read back
        {
            let mut reader = SigMfReader::open(base_path.with_extension("sigmf-meta")).unwrap();
            assert_eq!(reader.metadata().global.datatype, "cu8");

            let mut read_samples = vec![IQSample::new(0.0, 0.0); 4];
            reader.read_samples(&mut read_samples).unwrap();

            // cu8 has limited precision (8-bit)
            for (orig, read) in samples.iter().zip(read_samples.iter()) {
                assert!((orig.re - read.re).abs() < 0.02, "I mismatch: {} vs {}", orig.re, read.re);
                assert!((orig.im - read.im).abs() < 0.02, "Q mismatch: {} vs {}", orig.im, read.im);
            }
        }
    }

    #[test]
    fn test_gnuradio_metadata_structure() {
        // Verify metadata structure matches what GNU Radio's gr-sigmf expects
        let temp_dir = TempDir::new().unwrap();
        let base_path = temp_dir.path().join("gnuradio_compat");

        let samples: Vec<IQSample> = (0..1000)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 100.0;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect();

        // Create recording with full metadata
        {
            let mut writer = SigMfWriter::create(&base_path, 2.4e6, 915e6).unwrap();
            writer.set_description("GNU Radio compatibility test");
            writer.set_hardware("R4W Simulator");
            writer.set_waveform("LoRa SF7");
            writer.add_annotation(0, 500, "preamble");
            writer.add_annotation(500, 500, "payload");
            writer.write_samples(&samples).unwrap();
            let meta = writer.close().unwrap();

            // Verify structure matches GNU Radio expectations
            assert_eq!(meta.global.version, "1.0.0");
            assert_eq!(meta.global.datatype, "cf32_le");
            assert!(meta.global.num_channels.is_some());
            assert!(meta.global.datetime.is_some());

            // Verify captures array exists with frequency
            assert!(!meta.captures.is_empty());
            assert_eq!(meta.captures[0].sample_start, 0);
            assert!(meta.captures[0].frequency.is_some());

            // Verify annotations
            assert_eq!(meta.annotations.len(), 2);
            assert_eq!(meta.annotations[0].sample_count, 500);
            assert_eq!(meta.annotations[1].sample_start, 500);
        }

        // Verify the JSON structure by re-reading metadata file
        {
            let meta_path = base_path.with_extension("sigmf-meta");
            let meta_content = std::fs::read_to_string(&meta_path).unwrap();
            let parsed: serde_json::Value = serde_json::from_str(&meta_content).unwrap();

            // GNU Radio expects these exact field names
            assert!(parsed["global"]["core:datatype"].is_string());
            assert!(parsed["global"]["core:sample_rate"].is_number());
            assert!(parsed["global"]["core:version"].is_string());
            assert!(parsed["captures"].is_array());
            assert!(parsed["annotations"].is_array());

            // Captures must have core:sample_start
            assert!(parsed["captures"][0]["core:sample_start"].is_number());
        }
    }

    #[test]
    fn test_all_sigmf_formats_roundtrip() {
        // Test all formats that GNU Radio commonly uses
        let formats = ["cf32_le", "ci16_le", "ci8", "cu8"];
        let temp_dir = TempDir::new().unwrap();

        // Test signal: 10 Hz sine wave sampled at 100 Hz
        let samples: Vec<IQSample> = (0..100)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 10.0;
                IQSample::new(phase.cos() * 0.9, phase.sin() * 0.9) // 0.9 to avoid clipping
            })
            .collect();

        for format in formats {
            let base_path = temp_dir.path().join(format!("test_{}", format));

            // Write
            {
                let mut writer =
                    SigMfWriter::create_with_format(&base_path, 100.0, 1e6, format).unwrap();
                writer.write_samples(&samples).unwrap();
                writer.close().unwrap();
            }

            // Read and verify
            {
                let mut reader = SigMfReader::open(base_path.with_extension("sigmf-meta")).unwrap();
                let mut read_samples = vec![IQSample::new(0.0, 0.0); 100];
                reader.read_samples(&mut read_samples).unwrap();

                // Calculate expected precision based on format
                let tolerance = match format {
                    "cf32_le" => 1e-5,  // 32-bit float, very precise
                    "ci16_le" => 0.001, // 16-bit int
                    "ci8" | "cu8" => 0.02, // 8-bit int
                    _ => 0.1,
                };

                for (i, (orig, read)) in samples.iter().zip(read_samples.iter()).enumerate() {
                    assert!(
                        (orig.re - read.re).abs() < tolerance,
                        "{} I mismatch at {}: {} vs {}", format, i, orig.re, read.re
                    );
                    assert!(
                        (orig.im - read.im).abs() < tolerance,
                        "{} Q mismatch at {}: {} vs {}", format, i, orig.im, read.im
                    );
                }
            }
        }
    }
}

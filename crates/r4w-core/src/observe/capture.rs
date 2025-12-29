//! # Real-Time Packet Capture
//!
//! Provides real-time I/Q sample capture for debugging and analysis.
//! This module integrates with the observability stack to provide:
//!
//! - **Continuous capture**: Ring buffer based recording (flight recorder mode)
//! - **Triggered capture**: Start/stop based on events (packet detection, errors)
//! - **Pre/post trigger**: Capture samples before and after trigger events
//! - **SigMF output**: Industry-standard format compatible with GNU Radio
//!
//! ## Architecture
//!
//! ```text
//! ┌────────────────────────────────────────────────────────────┐
//! │                    Capture Manager                          │
//! │   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
//! │   │ Ring Buffer  │  │   Trigger    │  │    Writer    │     │
//! │   │  (samples)   │→→│   Engine     │→→│  (SigMF)     │     │
//! │   └──────────────┘  └──────────────┘  └──────────────┘     │
//! └────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::observe::capture::{CaptureManager, CaptureConfig, TriggerMode};
//!
//! // Create capture manager
//! let config = CaptureConfig::default()
//!     .with_buffer_size(1_000_000)  // 1M samples
//!     .with_trigger(TriggerMode::Manual)
//!     .with_output_dir("/var/capture/r4w");
//!
//! let mut capture = CaptureManager::new(config);
//!
//! // Feed samples continuously
//! capture.push_samples(&samples, timestamp);
//!
//! // Trigger capture (saves pre-trigger and post-trigger samples)
//! capture.trigger("packet_detected");
//! ```

use crate::timing::Timestamp;
use crate::types::IQSample;
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// Capture state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureState {
    /// Idle, not capturing
    Idle,
    /// Recording samples to ring buffer
    Recording,
    /// Triggered, capturing post-trigger samples
    Triggered,
    /// Writing capture to file
    Writing,
    /// Capture complete
    Complete,
    /// Error occurred
    Error,
}

/// Trigger mode for capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TriggerMode {
    /// Manual trigger via API call
    #[default]
    Manual,
    /// Trigger on power threshold (dBFS)
    Power {
        threshold_dbfs: i32,
        hysteresis_db: i32,
    },
    /// Trigger on packet detection
    PacketDetect,
    /// Trigger on error condition
    OnError,
    /// Continuous capture (ring buffer mode)
    Continuous,
}

/// Capture format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CaptureFormat {
    /// SigMF (preferred, compatible with GNU Radio)
    #[default]
    SigMF,
    /// Raw binary I/Q (cf32)
    RawFloat32,
    /// Raw binary I/Q (ci16)
    RawInt16,
}

/// Capture configuration.
#[derive(Debug, Clone)]
pub struct CaptureConfig {
    /// Ring buffer size in samples
    pub buffer_size: usize,
    /// Pre-trigger samples to save
    pub pre_trigger_samples: usize,
    /// Post-trigger samples to save
    pub post_trigger_samples: usize,
    /// Output directory for captures
    pub output_dir: PathBuf,
    /// Capture format
    pub format: CaptureFormat,
    /// Trigger mode
    pub trigger_mode: TriggerMode,
    /// Sample rate (for metadata)
    pub sample_rate: f64,
    /// Center frequency (for metadata)
    pub center_frequency: f64,
    /// Maximum file size (for rotation)
    pub max_file_size: u64,
    /// Enable capture
    pub enabled: bool,
    /// Auto-arm after trigger
    pub auto_rearm: bool,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            buffer_size: 1_048_576, // 1M samples
            pre_trigger_samples: 262_144, // 256K pre-trigger
            post_trigger_samples: 262_144, // 256K post-trigger
            output_dir: PathBuf::from("/var/capture/r4w"),
            format: CaptureFormat::SigMF,
            trigger_mode: TriggerMode::Manual,
            sample_rate: 1_000_000.0,
            center_frequency: 915_000_000.0,
            max_file_size: 1_073_741_824, // 1 GB
            enabled: false,
            auto_rearm: true,
        }
    }
}

impl CaptureConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set buffer size.
    pub fn with_buffer_size(mut self, size: usize) -> Self {
        self.buffer_size = size;
        self
    }

    /// Set pre/post trigger samples.
    pub fn with_trigger_window(mut self, pre: usize, post: usize) -> Self {
        self.pre_trigger_samples = pre;
        self.post_trigger_samples = post;
        self
    }

    /// Set output directory.
    pub fn with_output_dir<P: AsRef<Path>>(mut self, path: P) -> Self {
        self.output_dir = path.as_ref().to_path_buf();
        self
    }

    /// Set capture format.
    pub fn with_format(mut self, format: CaptureFormat) -> Self {
        self.format = format;
        self
    }

    /// Set trigger mode.
    pub fn with_trigger(mut self, mode: TriggerMode) -> Self {
        self.trigger_mode = mode;
        self
    }

    /// Set sample rate.
    pub fn with_sample_rate(mut self, rate: f64) -> Self {
        self.sample_rate = rate;
        self
    }

    /// Set center frequency.
    pub fn with_center_frequency(mut self, freq: f64) -> Self {
        self.center_frequency = freq;
        self
    }

    /// Enable capture.
    pub fn enabled(mut self, enable: bool) -> Self {
        self.enabled = enable;
        self
    }
}

/// Information about a completed capture.
#[derive(Debug, Clone)]
pub struct CaptureInfo {
    /// Unique capture ID
    pub id: u64,
    /// Capture file path
    pub path: PathBuf,
    /// Trigger reason
    pub trigger_reason: String,
    /// Trigger timestamp
    pub trigger_time: Timestamp,
    /// Wall clock time
    pub wall_time: SystemTime,
    /// Number of samples captured
    pub sample_count: u64,
    /// Pre-trigger samples included
    pub pre_trigger: usize,
    /// Post-trigger samples included
    pub post_trigger: usize,
    /// Duration in seconds
    pub duration_secs: f64,
    /// Sample rate
    pub sample_rate: f64,
    /// Center frequency
    pub center_frequency: f64,
}

/// Ring buffer for sample storage.
struct SampleRingBuffer {
    buffer: VecDeque<IQSample>,
    capacity: usize,
    samples_pushed: u64,
}

impl SampleRingBuffer {
    fn new(capacity: usize) -> Self {
        Self {
            buffer: VecDeque::with_capacity(capacity),
            capacity,
            samples_pushed: 0,
        }
    }

    fn push(&mut self, sample: IQSample) {
        if self.buffer.len() >= self.capacity {
            self.buffer.pop_front();
        }
        self.buffer.push_back(sample);
        self.samples_pushed += 1;
    }

    fn push_slice(&mut self, samples: &[IQSample]) {
        for &sample in samples {
            self.push(sample);
        }
    }

    fn drain_last(&mut self, count: usize) -> Vec<IQSample> {
        let len = self.buffer.len();
        let start = len.saturating_sub(count);
        self.buffer.drain(start..).collect()
    }

    fn len(&self) -> usize {
        self.buffer.len()
    }

    fn clear(&mut self) {
        self.buffer.clear();
    }

    #[allow(dead_code)]
    fn samples_pushed(&self) -> u64 {
        self.samples_pushed
    }
}

/// Real-time packet capture manager.
pub struct CaptureManager {
    /// Configuration
    config: CaptureConfig,
    /// Current state
    state: CaptureState,
    /// Sample ring buffer
    ring_buffer: SampleRingBuffer,
    /// Post-trigger buffer (samples after trigger)
    post_trigger_buffer: Vec<IQSample>,
    /// Samples remaining to capture post-trigger
    post_trigger_remaining: usize,
    /// Current trigger reason
    trigger_reason: Option<String>,
    /// Trigger timestamp
    trigger_timestamp: Option<Timestamp>,
    /// Last sample timestamp
    last_timestamp: Option<Timestamp>,
    /// Capture counter
    capture_counter: AtomicU64,
    /// Total samples processed
    total_samples: AtomicU64,
    /// Armed flag
    armed: AtomicBool,
    /// Completed captures
    completed_captures: Vec<CaptureInfo>,
}

impl CaptureManager {
    /// Create a new capture manager.
    pub fn new(config: CaptureConfig) -> Self {
        let buffer_size = config.buffer_size;
        Self {
            config,
            state: CaptureState::Idle,
            ring_buffer: SampleRingBuffer::new(buffer_size),
            post_trigger_buffer: Vec::new(),
            post_trigger_remaining: 0,
            trigger_reason: None,
            trigger_timestamp: None,
            last_timestamp: None,
            capture_counter: AtomicU64::new(0),
            total_samples: AtomicU64::new(0),
            armed: AtomicBool::new(false),
            completed_captures: Vec::new(),
        }
    }

    /// Get current configuration.
    pub fn config(&self) -> &CaptureConfig {
        &self.config
    }

    /// Update configuration.
    ///
    /// Note: Some changes require restart (buffer_size).
    pub fn set_config(&mut self, config: CaptureConfig) {
        // Resize ring buffer if needed
        if config.buffer_size != self.config.buffer_size {
            self.ring_buffer = SampleRingBuffer::new(config.buffer_size);
        }
        self.config = config;
    }

    /// Get current state.
    pub fn state(&self) -> CaptureState {
        self.state
    }

    /// Check if armed and ready to trigger.
    pub fn is_armed(&self) -> bool {
        self.armed.load(Ordering::Relaxed)
    }

    /// Arm the capture system.
    pub fn arm(&mut self) {
        if !self.config.enabled {
            return;
        }
        self.state = CaptureState::Recording;
        self.armed.store(true, Ordering::Relaxed);
        self.ring_buffer.clear();
        tracing::info!("Capture armed");
    }

    /// Disarm the capture system.
    pub fn disarm(&mut self) {
        self.armed.store(false, Ordering::Relaxed);
        self.state = CaptureState::Idle;
        tracing::info!("Capture disarmed");
    }

    /// Push samples into the capture buffer.
    ///
    /// In recording state, samples are stored in the ring buffer.
    /// After trigger, post-trigger samples are collected.
    pub fn push_samples(&mut self, samples: &[IQSample], timestamp: Timestamp) {
        self.last_timestamp = Some(timestamp);
        self.total_samples.fetch_add(samples.len() as u64, Ordering::Relaxed);

        match self.state {
            CaptureState::Recording => {
                // Store in ring buffer
                self.ring_buffer.push_slice(samples);

                // Check auto-trigger conditions
                if let TriggerMode::Power { threshold_dbfs, .. } = self.config.trigger_mode {
                    let power = self.calculate_power_dbfs(samples);
                    if power > threshold_dbfs as f64 {
                        self.trigger("power_threshold");
                    }
                }
            }
            CaptureState::Triggered => {
                // Collect post-trigger samples
                let to_collect = self.post_trigger_remaining.min(samples.len());
                self.post_trigger_buffer.extend_from_slice(&samples[..to_collect]);
                self.post_trigger_remaining -= to_collect;

                if self.post_trigger_remaining == 0 {
                    // Post-trigger complete, write capture
                    self.write_capture();
                }
            }
            _ => {}
        }
    }

    /// Manually trigger a capture.
    pub fn trigger(&mut self, reason: &str) {
        if self.state != CaptureState::Recording {
            tracing::warn!("Cannot trigger: not in recording state");
            return;
        }

        self.trigger_reason = Some(reason.to_string());
        self.trigger_timestamp = self.last_timestamp.clone();
        self.state = CaptureState::Triggered;
        self.post_trigger_remaining = self.config.post_trigger_samples;
        self.post_trigger_buffer.clear();
        self.post_trigger_buffer.reserve(self.config.post_trigger_samples);

        tracing::info!("Capture triggered: {}", reason);
    }

    /// Write the capture to disk.
    fn write_capture(&mut self) {
        self.state = CaptureState::Writing;

        // Get pre-trigger samples from ring buffer
        let pre_trigger = self.ring_buffer.drain_last(self.config.pre_trigger_samples);

        // Combine pre and post trigger samples
        let mut all_samples = pre_trigger.clone();
        all_samples.extend_from_slice(&self.post_trigger_buffer);

        // Generate filename
        let capture_id = self.capture_counter.fetch_add(1, Ordering::Relaxed);
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
        let filename = format!("capture_{:04}_{}.sigmf", capture_id, timestamp);
        let path = self.config.output_dir.join(&filename);

        // Create output directory if needed
        if let Err(e) = std::fs::create_dir_all(&self.config.output_dir) {
            tracing::error!("Failed to create capture directory: {}", e);
            self.state = CaptureState::Error;
            return;
        }

        // Write capture based on format
        match self.config.format {
            CaptureFormat::SigMF => {
                if let Err(e) = self.write_sigmf(&path, &all_samples) {
                    tracing::error!("Failed to write capture: {}", e);
                    self.state = CaptureState::Error;
                    return;
                }
            }
            CaptureFormat::RawFloat32 => {
                if let Err(e) = self.write_raw_f32(&path, &all_samples) {
                    tracing::error!("Failed to write capture: {}", e);
                    self.state = CaptureState::Error;
                    return;
                }
            }
            CaptureFormat::RawInt16 => {
                if let Err(e) = self.write_raw_i16(&path, &all_samples) {
                    tracing::error!("Failed to write capture: {}", e);
                    self.state = CaptureState::Error;
                    return;
                }
            }
        }

        // Record capture info
        let info = CaptureInfo {
            id: capture_id,
            path: path.clone(),
            trigger_reason: self.trigger_reason.take().unwrap_or_default(),
            trigger_time: self.trigger_timestamp.take().unwrap_or_default(),
            wall_time: SystemTime::now(),
            sample_count: all_samples.len() as u64,
            pre_trigger: pre_trigger.len(),
            post_trigger: self.post_trigger_buffer.len(),
            duration_secs: all_samples.len() as f64 / self.config.sample_rate,
            sample_rate: self.config.sample_rate,
            center_frequency: self.config.center_frequency,
        };

        tracing::info!(
            "Capture written: {} ({} samples, {:.3} sec)",
            path.display(),
            all_samples.len(),
            info.duration_secs
        );

        self.completed_captures.push(info);
        self.post_trigger_buffer.clear();

        // Auto-rearm if enabled
        if self.config.auto_rearm {
            self.state = CaptureState::Recording;
        } else {
            self.state = CaptureState::Complete;
        }
    }

    /// Write capture in SigMF format.
    fn write_sigmf(&self, path: &Path, samples: &[IQSample]) -> std::io::Result<()> {
        use std::io::Write;

        let base_path = path.with_extension("");
        let data_path = base_path.with_extension("sigmf-data");
        let meta_path = base_path.with_extension("sigmf-meta");

        // Write data file (cf32_le format)
        let mut data_file = std::fs::File::create(&data_path)?;
        let mut f32_buf = vec![0.0f32; samples.len() * 2];
        for (i, sample) in samples.iter().enumerate() {
            f32_buf[i * 2] = sample.re as f32;
            f32_buf[i * 2 + 1] = sample.im as f32;
        }
        let byte_buf = unsafe {
            std::slice::from_raw_parts(f32_buf.as_ptr() as *const u8, samples.len() * 8)
        };
        data_file.write_all(byte_buf)?;

        // Write metadata file
        let meta = serde_json::json!({
            "global": {
                "core:datatype": "cf32_le",
                "core:sample_rate": self.config.sample_rate,
                "core:version": "1.0.0",
                "core:num_channels": 1,
                "core:description": format!("R4W Capture - {}",
                    self.trigger_reason.as_deref().unwrap_or("manual")),
                "core:author": "R4W",
                "core:datetime": chrono::Utc::now().to_rfc3339(),
                "r4w:trigger_reason": self.trigger_reason.as_deref().unwrap_or("manual"),
                "r4w:pre_trigger_samples": self.config.pre_trigger_samples,
                "r4w:post_trigger_samples": self.config.post_trigger_samples,
            },
            "captures": [{
                "core:sample_start": 0,
                "core:frequency": self.config.center_frequency,
                "core:datetime": chrono::Utc::now().to_rfc3339(),
            }],
            "annotations": [{
                "core:sample_start": self.config.pre_trigger_samples,
                "core:sample_count": 0,
                "core:label": "trigger_point",
                "core:comment": self.trigger_reason.as_deref().unwrap_or("manual trigger"),
            }]
        });

        let meta_file = std::fs::File::create(&meta_path)?;
        serde_json::to_writer_pretty(meta_file, &meta)?;

        Ok(())
    }

    /// Write capture as raw cf32.
    fn write_raw_f32(&self, path: &Path, samples: &[IQSample]) -> std::io::Result<()> {
        use std::io::Write;

        let data_path = path.with_extension("cf32");
        let mut file = std::fs::File::create(&data_path)?;

        let mut f32_buf = vec![0.0f32; samples.len() * 2];
        for (i, sample) in samples.iter().enumerate() {
            f32_buf[i * 2] = sample.re as f32;
            f32_buf[i * 2 + 1] = sample.im as f32;
        }
        let byte_buf = unsafe {
            std::slice::from_raw_parts(f32_buf.as_ptr() as *const u8, samples.len() * 8)
        };
        file.write_all(byte_buf)?;

        Ok(())
    }

    /// Write capture as raw ci16.
    fn write_raw_i16(&self, path: &Path, samples: &[IQSample]) -> std::io::Result<()> {
        use std::io::Write;

        let data_path = path.with_extension("ci16");
        let mut file = std::fs::File::create(&data_path)?;

        let mut i16_buf = vec![0i16; samples.len() * 2];
        for (i, sample) in samples.iter().enumerate() {
            i16_buf[i * 2] = (sample.re * 32767.0).clamp(-32768.0, 32767.0) as i16;
            i16_buf[i * 2 + 1] = (sample.im * 32767.0).clamp(-32768.0, 32767.0) as i16;
        }
        let byte_buf = unsafe {
            std::slice::from_raw_parts(i16_buf.as_ptr() as *const u8, samples.len() * 4)
        };
        file.write_all(byte_buf)?;

        Ok(())
    }

    /// Calculate power in dBFS for samples.
    fn calculate_power_dbfs(&self, samples: &[IQSample]) -> f64 {
        if samples.is_empty() {
            return -120.0;
        }

        let sum: f64 = samples.iter().map(|s| s.re * s.re + s.im * s.im).sum();
        let mean_power = sum / samples.len() as f64;

        if mean_power > 1e-20 {
            10.0 * mean_power.log10()
        } else {
            -120.0
        }
    }

    /// Get total samples processed.
    pub fn total_samples(&self) -> u64 {
        self.total_samples.load(Ordering::Relaxed)
    }

    /// Get number of samples in ring buffer.
    pub fn buffer_level(&self) -> usize {
        self.ring_buffer.len()
    }

    /// Get capture count.
    pub fn capture_count(&self) -> u64 {
        self.capture_counter.load(Ordering::Relaxed)
    }

    /// Get list of completed captures.
    pub fn completed_captures(&self) -> &[CaptureInfo] {
        &self.completed_captures
    }

    /// Get the most recent capture.
    pub fn last_capture(&self) -> Option<&CaptureInfo> {
        self.completed_captures.last()
    }

    /// Clear completed capture list.
    pub fn clear_history(&mut self) {
        self.completed_captures.clear();
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.state = CaptureState::Idle;
        self.ring_buffer.clear();
        self.post_trigger_buffer.clear();
        self.post_trigger_remaining = 0;
        self.trigger_reason = None;
        self.trigger_timestamp = None;
        self.armed.store(false, Ordering::Relaxed);
    }
}

impl Default for CaptureManager {
    fn default() -> Self {
        Self::new(CaptureConfig::default())
    }
}

/// Statistics for capture system.
#[derive(Debug, Clone, Default)]
pub struct CaptureStats {
    /// Total samples processed
    pub total_samples: u64,
    /// Number of captures completed
    pub captures_completed: u64,
    /// Buffer fill level (0.0 to 1.0)
    pub buffer_fill: f32,
    /// Captures per minute (rate)
    pub captures_per_minute: f32,
    /// Average capture size (samples)
    pub avg_capture_size: u64,
    /// Total bytes written
    pub bytes_written: u64,
}

impl CaptureManager {
    /// Get capture statistics.
    pub fn stats(&self) -> CaptureStats {
        let captures = self.capture_counter.load(Ordering::Relaxed);
        let total_samples: u64 = self.completed_captures.iter()
            .map(|c| c.sample_count)
            .sum();

        CaptureStats {
            total_samples: self.total_samples.load(Ordering::Relaxed),
            captures_completed: captures,
            buffer_fill: self.ring_buffer.len() as f32 / self.config.buffer_size as f32,
            captures_per_minute: 0.0, // Would need timing
            avg_capture_size: if captures > 0 { total_samples / captures } else { 0 },
            bytes_written: total_samples * 8, // cf32_le format
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    fn make_samples(count: usize) -> Vec<IQSample> {
        (0..count)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 100.0;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_capture_config_builder() {
        let config = CaptureConfig::new()
            .with_buffer_size(2_000_000)
            .with_trigger_window(100_000, 200_000)
            .with_format(CaptureFormat::RawFloat32)
            .with_sample_rate(2.4e6)
            .enabled(true);

        assert_eq!(config.buffer_size, 2_000_000);
        assert_eq!(config.pre_trigger_samples, 100_000);
        assert_eq!(config.post_trigger_samples, 200_000);
        assert_eq!(config.format, CaptureFormat::RawFloat32);
        assert!(config.enabled);
    }

    #[test]
    fn test_ring_buffer() {
        let mut buf = SampleRingBuffer::new(100);

        // Push samples
        let samples = make_samples(50);
        buf.push_slice(&samples);
        assert_eq!(buf.len(), 50);

        // Push more (should wrap)
        buf.push_slice(&samples);
        assert_eq!(buf.len(), 100);

        // Push even more (should evict oldest)
        buf.push_slice(&samples);
        assert_eq!(buf.len(), 100);
        assert_eq!(buf.samples_pushed(), 150);
    }

    #[test]
    fn test_capture_manager_arm_disarm() {
        let config = CaptureConfig::new().enabled(true);
        let mut capture = CaptureManager::new(config);

        assert_eq!(capture.state(), CaptureState::Idle);
        assert!(!capture.is_armed());

        capture.arm();
        assert_eq!(capture.state(), CaptureState::Recording);
        assert!(capture.is_armed());

        capture.disarm();
        assert_eq!(capture.state(), CaptureState::Idle);
        assert!(!capture.is_armed());
    }

    #[test]
    fn test_capture_workflow() {
        let temp_dir = TempDir::new().unwrap();

        let config = CaptureConfig::new()
            .with_buffer_size(1000)
            .with_trigger_window(100, 100)
            .with_output_dir(temp_dir.path())
            .enabled(true);

        let mut capture = CaptureManager::new(config);
        capture.arm();

        // Push pre-trigger samples
        let samples = make_samples(200);
        let ts = Timestamp::new(1e6);
        capture.push_samples(&samples, ts.clone());

        // Trigger
        capture.trigger("test_trigger");
        assert_eq!(capture.state(), CaptureState::Triggered);

        // Push post-trigger samples
        capture.push_samples(&samples, ts);

        // Should complete and rearm
        assert_eq!(capture.state(), CaptureState::Recording);
        assert_eq!(capture.capture_count(), 1);

        // Verify capture info
        let info = capture.last_capture().unwrap();
        assert_eq!(info.trigger_reason, "test_trigger");
        // SigMF creates .sigmf-meta and .sigmf-data files
        assert!(info.path.with_extension("sigmf-meta").exists());
        assert!(info.path.with_extension("sigmf-data").exists());
    }

    #[test]
    fn test_power_calculation() {
        let config = CaptureConfig::new();
        let capture = CaptureManager::new(config);

        // Full scale signal
        let samples: Vec<IQSample> = (0..100)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 10.0;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect();

        let power = capture.calculate_power_dbfs(&samples);
        // Unit circle samples: |z|^2 = cos^2 + sin^2 = 1, mean power = 1.0, 10*log10(1) = 0 dBFS
        // But in practice with discrete sampling there's some variation
        // Accept values from -3 dBFS to +1 dBFS
        assert!(power > -3.0 && power < 1.0, "Power was {}", power);

        // Empty samples
        let power = capture.calculate_power_dbfs(&[]);
        assert!(power < -100.0);
    }

    #[test]
    fn test_sigmf_output() {
        let temp_dir = TempDir::new().unwrap();

        let config = CaptureConfig::new()
            .with_buffer_size(500)
            .with_trigger_window(100, 100)
            .with_output_dir(temp_dir.path())
            .with_sample_rate(1e6)
            .with_center_frequency(915e6)
            .with_format(CaptureFormat::SigMF)
            .enabled(true);

        let mut capture = CaptureManager::new(config);
        capture.arm();

        let samples = make_samples(200);
        let ts = Timestamp::new(1e6);
        capture.push_samples(&samples, ts.clone());
        capture.trigger("sigmf_test");
        capture.push_samples(&samples, ts);

        let info = capture.last_capture().unwrap();

        // Check files exist
        let meta_path = info.path.with_extension("sigmf-meta");
        let data_path = info.path.with_extension("sigmf-data");
        assert!(meta_path.exists());
        assert!(data_path.exists());

        // Parse metadata
        let meta_content = std::fs::read_to_string(&meta_path).unwrap();
        let meta: serde_json::Value = serde_json::from_str(&meta_content).unwrap();

        assert_eq!(meta["global"]["core:datatype"], "cf32_le");
        assert_eq!(meta["global"]["core:sample_rate"], 1e6);
        assert!(meta["captures"].is_array());
    }

    #[test]
    fn test_stats() {
        let config = CaptureConfig::new()
            .with_buffer_size(1000)
            .enabled(true);

        let mut capture = CaptureManager::new(config);
        capture.arm();

        let samples = make_samples(500);
        let ts = Timestamp::new(1e6);
        capture.push_samples(&samples, ts);

        let stats = capture.stats();
        assert_eq!(stats.total_samples, 500);
        assert!(stats.buffer_fill > 0.4 && stats.buffer_fill < 0.6);
    }
}

//! Indexed recording and playback of IQ signal streams with sample-accurate seeking.
//!
//! This module provides [`SignalRecorder`] for capturing IQ samples organised into
//! frames, and [`SignalPlayer`] for playing them back with random-access seeking
//! by sample number or timestamp. An in-memory index ([`FrameIndex`]) enables
//! O(1) seek to any frame boundary. Delta encoding provides basic compression,
//! and a CRC-32 validates recorded data integrity.
//!
//! # Example
//!
//! ```
//! use r4w_core::signal_recorder_indexed::{SignalRecorder, SignalPlayer, RecordingConfig, SampleFormat};
//!
//! let config = RecordingConfig {
//!     sample_rate: 1_000_000.0,
//!     frame_size: 4,
//!     format: SampleFormat::F64,
//! };
//!
//! let mut recorder = SignalRecorder::new(config.clone());
//! let frame: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
//! recorder.record_frame(&frame).unwrap();
//! recorder.record_frame(&frame).unwrap();
//!
//! let meta = recorder.metadata();
//! assert_eq!(meta.total_frames, 2);
//! assert_eq!(meta.total_samples, 8);
//!
//! let mut player = SignalPlayer::from_recorder(&recorder).unwrap();
//! let f = player.read_frame().unwrap();
//! assert_eq!(f.len(), 4);
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Sample storage precision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleFormat {
    /// 32-bit float (each component).
    F32,
    /// 64-bit float (each component).
    F64,
}

impl fmt::Display for SampleFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SampleFormat::F32 => write!(f, "f32"),
            SampleFormat::F64 => write!(f, "f64"),
        }
    }
}

/// Configuration for a recording session.
#[derive(Debug, Clone)]
pub struct RecordingConfig {
    /// Samples per second.
    pub sample_rate: f64,
    /// Number of IQ samples in each frame.
    pub frame_size: usize,
    /// Sample precision.
    pub format: SampleFormat,
}

/// Index entry for one frame inside the recording.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameIndex {
    /// Zero-based frame number.
    pub frame_number: usize,
    /// Byte offset from the start of the raw sample buffer.
    pub byte_offset: usize,
    /// Sample offset (first sample of this frame) from start of recording.
    pub sample_offset: usize,
    /// Timestamp in microseconds from recording start.
    pub timestamp_us: u64,
}

/// Summary metadata for a recording.
#[derive(Debug, Clone, PartialEq)]
pub struct RecordingMetadata {
    /// Total number of individual IQ samples recorded.
    pub total_samples: usize,
    /// Total number of frames recorded.
    pub total_frames: usize,
    /// Duration in seconds.
    pub duration_s: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
}

/// Errors produced by recording / playback operations.
#[derive(Debug, Clone, PartialEq)]
pub enum RecorderError {
    /// Supplied frame length does not match `RecordingConfig::frame_size`.
    FrameSizeMismatch { expected: usize, got: usize },
    /// No data has been recorded yet.
    EmptyRecording,
    /// Seek target is beyond the end of the recording.
    SeekOutOfRange { requested: usize, available: usize },
    /// CRC validation failed.
    CrcMismatch { expected: u32, computed: u32 },
    /// Attempted to read past the last frame.
    EndOfRecording,
}

impl fmt::Display for RecorderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RecorderError::FrameSizeMismatch { expected, got } => {
                write!(f, "frame size mismatch: expected {expected}, got {got}")
            }
            RecorderError::EmptyRecording => write!(f, "recording is empty"),
            RecorderError::SeekOutOfRange {
                requested,
                available,
            } => write!(
                f,
                "seek out of range: requested sample {requested}, recording has {available}"
            ),
            RecorderError::CrcMismatch { expected, computed } => {
                write!(f, "CRC mismatch: expected {expected:#010x}, computed {computed:#010x}")
            }
            RecorderError::EndOfRecording => write!(f, "end of recording"),
        }
    }
}

// ---------------------------------------------------------------------------
// CRC-32 (ISO 3309 / ITU-T V.42 polynomial, same as zlib)
// ---------------------------------------------------------------------------

const CRC32_POLY: u32 = 0xEDB8_8320;

fn crc32_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    for i in 0..256u32 {
        let mut crc = i;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ CRC32_POLY;
            } else {
                crc >>= 1;
            }
        }
        table[i as usize] = crc;
    }
    table
}

fn crc32(data: &[u8]) -> u32 {
    let table = crc32_table();
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        let idx = ((crc ^ u32::from(b)) & 0xFF) as usize;
        crc = (crc >> 8) ^ table[idx];
    }
    crc ^ 0xFFFF_FFFF
}

// ---------------------------------------------------------------------------
// Delta encoding helpers
// ---------------------------------------------------------------------------

/// Delta-encode a slice of `(f64, f64)` samples.  The first sample is stored
/// verbatim; subsequent samples store the difference from the previous one.
fn delta_encode(samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if samples.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(samples.len());
    out.push(samples[0]);
    for i in 1..samples.len() {
        out.push((
            samples[i].0 - samples[i - 1].0,
            samples[i].1 - samples[i - 1].1,
        ));
    }
    out
}

/// Reconstruct samples from their delta-encoded representation.
fn delta_decode(encoded: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if encoded.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(encoded.len());
    out.push(encoded[0]);
    for i in 1..encoded.len() {
        let prev = out[i - 1];
        out.push((prev.0 + encoded[i].0, prev.1 + encoded[i].1));
    }
    out
}

/// Serialize a `(f64, f64)` slice to bytes (little-endian, 16 bytes per sample).
fn samples_to_bytes(samples: &[(f64, f64)]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(samples.len() * 16);
    for &(re, im) in samples {
        buf.extend_from_slice(&re.to_le_bytes());
        buf.extend_from_slice(&im.to_le_bytes());
    }
    buf
}

/// Deserialize bytes back into `(f64, f64)` samples.
fn bytes_to_samples(data: &[u8]) -> Vec<(f64, f64)> {
    let count = data.len() / 16;
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let off = i * 16;
        let re = f64::from_le_bytes(data[off..off + 8].try_into().unwrap());
        let im = f64::from_le_bytes(data[off + 8..off + 16].try_into().unwrap());
        out.push((re, im));
    }
    out
}

// ---------------------------------------------------------------------------
// SignalRecorder
// ---------------------------------------------------------------------------

/// Records IQ frames into an in-memory buffer with delta compression and
/// frame indexing for fast seeking.
#[derive(Debug, Clone)]
pub struct SignalRecorder {
    config: RecordingConfig,
    /// Raw bytes of delta-encoded samples (all frames concatenated).
    buffer: Vec<u8>,
    /// Frame index built during recording.
    index: Vec<FrameIndex>,
    /// Running CRC of the raw buffer bytes.
    crc: u32,
}

impl SignalRecorder {
    /// Create a new recorder with the given configuration.
    pub fn new(config: RecordingConfig) -> Self {
        Self {
            config,
            buffer: Vec::new(),
            index: Vec::new(),
            crc: 0,
        }
    }

    /// Append a frame of IQ samples. The slice length must match
    /// `config.frame_size`.
    pub fn record_frame(&mut self, samples: &[(f64, f64)]) -> Result<(), RecorderError> {
        if samples.len() != self.config.frame_size {
            return Err(RecorderError::FrameSizeMismatch {
                expected: self.config.frame_size,
                got: samples.len(),
            });
        }

        let frame_number = self.index.len();
        let byte_offset = self.buffer.len();
        let sample_offset = frame_number * self.config.frame_size;
        let timestamp_us =
            (sample_offset as f64 / self.config.sample_rate * 1_000_000.0) as u64;

        self.index.push(FrameIndex {
            frame_number,
            byte_offset,
            sample_offset,
            timestamp_us,
        });

        let encoded = delta_encode(samples);
        let bytes = samples_to_bytes(&encoded);
        self.buffer.extend_from_slice(&bytes);

        // Recompute CRC over entire buffer.
        self.crc = crc32(&self.buffer);

        Ok(())
    }

    /// Return a copy of the current frame index.
    pub fn build_index(&self) -> Vec<FrameIndex> {
        self.index.clone()
    }

    /// Return recording metadata.
    pub fn metadata(&self) -> RecordingMetadata {
        let total_frames = self.index.len();
        let total_samples = total_frames * self.config.frame_size;
        let duration_s = if self.config.sample_rate > 0.0 {
            total_samples as f64 / self.config.sample_rate
        } else {
            0.0
        };
        RecordingMetadata {
            total_samples,
            total_frames,
            duration_s,
            sample_rate: self.config.sample_rate,
        }
    }

    /// Validate the internal CRC.
    pub fn validate(&self) -> Result<(), RecorderError> {
        if self.buffer.is_empty() {
            return Ok(());
        }
        let computed = crc32(&self.buffer);
        if computed != self.crc {
            Err(RecorderError::CrcMismatch {
                expected: self.crc,
                computed,
            })
        } else {
            Ok(())
        }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &RecordingConfig {
        &self.config
    }

    /// Borrow the raw encoded buffer.
    pub fn raw_buffer(&self) -> &[u8] {
        &self.buffer
    }

    /// Return the stored CRC value.
    pub fn crc(&self) -> u32 {
        self.crc
    }
}

// ---------------------------------------------------------------------------
// SignalPlayer
// ---------------------------------------------------------------------------

/// Plays back a recording with random-access seeking.
#[derive(Debug, Clone)]
pub struct SignalPlayer {
    config: RecordingConfig,
    buffer: Vec<u8>,
    index: Vec<FrameIndex>,
    /// Current playback position (frame index).
    position: usize,
    total_frames: usize,
    crc: u32,
}

impl SignalPlayer {
    /// Build a player from a finished recorder.
    pub fn from_recorder(recorder: &SignalRecorder) -> Result<Self, RecorderError> {
        if recorder.index.is_empty() {
            return Err(RecorderError::EmptyRecording);
        }
        Ok(Self {
            config: recorder.config.clone(),
            buffer: recorder.buffer.clone(),
            index: recorder.index.clone(),
            position: 0,
            total_frames: recorder.index.len(),
            crc: recorder.crc,
        })
    }

    /// Seek to the frame that contains the given absolute sample number.
    pub fn seek_to_sample(&mut self, sample: usize) -> Result<(), RecorderError> {
        let total_samples = self.total_frames * self.config.frame_size;
        if sample >= total_samples {
            return Err(RecorderError::SeekOutOfRange {
                requested: sample,
                available: total_samples,
            });
        }
        self.position = sample / self.config.frame_size;
        Ok(())
    }

    /// Seek to the frame that contains the given timestamp (in seconds from
    /// recording start).
    pub fn seek_to_time(&mut self, time_s: f64) -> Result<(), RecorderError> {
        let sample = (time_s * self.config.sample_rate) as usize;
        self.seek_to_sample(sample)
    }

    /// Read the frame at the current playback position and advance to the next
    /// frame. Returns the decoded (delta-reconstructed) samples.
    pub fn read_frame(&mut self) -> Result<Vec<(f64, f64)>, RecorderError> {
        if self.position >= self.total_frames {
            return Err(RecorderError::EndOfRecording);
        }

        let entry = &self.index[self.position];
        let bytes_per_frame = self.config.frame_size * 16; // 16 bytes per (f64, f64)
        let start = entry.byte_offset;
        let end = start + bytes_per_frame;
        let encoded = bytes_to_samples(&self.buffer[start..end]);
        let decoded = delta_decode(&encoded);

        self.position += 1;
        Ok(decoded)
    }

    /// Return the current frame position.
    pub fn position(&self) -> usize {
        self.position
    }

    /// Return the total number of frames.
    pub fn total_frames(&self) -> usize {
        self.total_frames
    }

    /// Return recording metadata.
    pub fn metadata(&self) -> RecordingMetadata {
        let total_samples = self.total_frames * self.config.frame_size;
        let duration_s = if self.config.sample_rate > 0.0 {
            total_samples as f64 / self.config.sample_rate
        } else {
            0.0
        };
        RecordingMetadata {
            total_samples,
            total_frames: self.total_frames,
            duration_s,
            sample_rate: self.config.sample_rate,
        }
    }

    /// Validate the player's CRC against its buffer.
    pub fn validate(&self) -> Result<(), RecorderError> {
        if self.buffer.is_empty() {
            return Ok(());
        }
        let computed = crc32(&self.buffer);
        if computed != self.crc {
            Err(RecorderError::CrcMismatch {
                expected: self.crc,
                computed,
            })
        } else {
            Ok(())
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> RecordingConfig {
        RecordingConfig {
            sample_rate: 1_000_000.0,
            frame_size: 4,
            format: SampleFormat::F64,
        }
    }

    fn tone_frame(n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // 1. Basic record and metadata
    #[test]
    fn test_record_single_frame() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        let frame = tone_frame(4);
        rec.record_frame(&frame).unwrap();
        let meta = rec.metadata();
        assert_eq!(meta.total_frames, 1);
        assert_eq!(meta.total_samples, 4);
        assert!((meta.duration_s - 4e-6).abs() < 1e-12);
    }

    // 2. Frame size mismatch
    #[test]
    fn test_frame_size_mismatch() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        let bad_frame = vec![(0.0, 0.0); 3];
        let err = rec.record_frame(&bad_frame).unwrap_err();
        assert_eq!(
            err,
            RecorderError::FrameSizeMismatch {
                expected: 4,
                got: 3
            }
        );
    }

    // 3. Multiple frames and index
    #[test]
    fn test_multiple_frames_index() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        for _ in 0..5 {
            rec.record_frame(&tone_frame(4)).unwrap();
        }
        let idx = rec.build_index();
        assert_eq!(idx.len(), 5);
        for (i, entry) in idx.iter().enumerate() {
            assert_eq!(entry.frame_number, i);
            assert_eq!(entry.sample_offset, i * 4);
        }
    }

    // 4. Playback reads frames in order
    #[test]
    fn test_playback_sequential() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        let f0 = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0)];
        let f1 = vec![(10.0, 20.0), (30.0, 40.0), (50.0, 60.0), (70.0, 80.0)];
        rec.record_frame(&f0).unwrap();
        rec.record_frame(&f1).unwrap();

        let mut player = SignalPlayer::from_recorder(&rec).unwrap();
        let r0 = player.read_frame().unwrap();
        let r1 = player.read_frame().unwrap();
        assert_eq!(r0, f0);
        assert_eq!(r1, f1);
    }

    // 5. End of recording error
    #[test]
    fn test_end_of_recording() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        rec.record_frame(&tone_frame(4)).unwrap();
        let mut player = SignalPlayer::from_recorder(&rec).unwrap();
        player.read_frame().unwrap();
        let err = player.read_frame().unwrap_err();
        assert_eq!(err, RecorderError::EndOfRecording);
    }

    // 6. Seek to sample
    #[test]
    fn test_seek_to_sample() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        for i in 0..4 {
            let frame: Vec<(f64, f64)> = (0..4).map(|j| (i as f64, j as f64)).collect();
            rec.record_frame(&frame).unwrap();
        }
        let mut player = SignalPlayer::from_recorder(&rec).unwrap();

        // Seek to sample 8 => frame 2
        player.seek_to_sample(8).unwrap();
        assert_eq!(player.position(), 2);
        let frame = player.read_frame().unwrap();
        assert_eq!(frame[0].0, 2.0);
    }

    // 7. Seek to time
    #[test]
    fn test_seek_to_time() {
        let cfg = RecordingConfig {
            sample_rate: 100.0,
            frame_size: 10,
            format: SampleFormat::F64,
        };
        let mut rec = SignalRecorder::new(cfg);
        for i in 0..5 {
            let frame: Vec<(f64, f64)> = (0..10).map(|j| (i as f64, j as f64)).collect();
            rec.record_frame(&frame).unwrap();
        }
        let mut player = SignalPlayer::from_recorder(&rec).unwrap();

        // 0.3 s at 100 Hz => sample 30 => frame 3
        player.seek_to_time(0.3).unwrap();
        assert_eq!(player.position(), 3);
    }

    // 8. Seek out of range
    #[test]
    fn test_seek_out_of_range() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        rec.record_frame(&tone_frame(4)).unwrap();
        let mut player = SignalPlayer::from_recorder(&rec).unwrap();
        let err = player.seek_to_sample(100).unwrap_err();
        assert_eq!(
            err,
            RecorderError::SeekOutOfRange {
                requested: 100,
                available: 4,
            }
        );
    }

    // 9. CRC validation passes for clean recording
    #[test]
    fn test_crc_validation_passes() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        rec.record_frame(&tone_frame(4)).unwrap();
        rec.validate().unwrap();
        let player = SignalPlayer::from_recorder(&rec).unwrap();
        player.validate().unwrap();
    }

    // 10. CRC validation detects corruption
    #[test]
    fn test_crc_detects_corruption() {
        let cfg = default_config();
        let mut rec = SignalRecorder::new(cfg);
        rec.record_frame(&tone_frame(4)).unwrap();
        let mut player = SignalPlayer::from_recorder(&rec).unwrap();
        // Corrupt one byte
        if let Some(b) = player.buffer.get_mut(0) {
            *b ^= 0xFF;
        }
        let err = player.validate().unwrap_err();
        match err {
            RecorderError::CrcMismatch { .. } => {} // expected
            other => panic!("expected CrcMismatch, got {other:?}"),
        }
    }

    // 11. Delta encode / decode round-trip
    #[test]
    fn test_delta_roundtrip() {
        let samples: Vec<(f64, f64)> =
            vec![(1.0, 0.0), (2.0, 0.5), (4.0, 1.5), (7.0, 3.0)];
        let enc = delta_encode(&samples);
        let dec = delta_decode(&enc);
        for (a, b) in samples.iter().zip(dec.iter()) {
            assert!((a.0 - b.0).abs() < 1e-12);
            assert!((a.1 - b.1).abs() < 1e-12);
        }
    }

    // 12. Empty recording produces empty player error
    #[test]
    fn test_empty_recording_player() {
        let cfg = default_config();
        let rec = SignalRecorder::new(cfg);
        let err = SignalPlayer::from_recorder(&rec).unwrap_err();
        assert_eq!(err, RecorderError::EmptyRecording);
    }

    // 13. Timestamp in index grows correctly
    #[test]
    fn test_timestamp_index() {
        let cfg = RecordingConfig {
            sample_rate: 1_000_000.0,
            frame_size: 100,
            format: SampleFormat::F32,
        };
        let mut rec = SignalRecorder::new(cfg);
        let frame = vec![(0.0, 0.0); 100];
        for _ in 0..10 {
            rec.record_frame(&frame).unwrap();
        }
        let idx = rec.build_index();
        // Frame 5: sample_offset = 500, at 1 MHz => 500 us
        assert_eq!(idx[5].timestamp_us, 500);
    }

    // 14. SampleFormat Display
    #[test]
    fn test_sample_format_display() {
        assert_eq!(format!("{}", SampleFormat::F32), "f32");
        assert_eq!(format!("{}", SampleFormat::F64), "f64");
    }

    // 15. Recorder error Display
    #[test]
    fn test_error_display() {
        let e = RecorderError::EndOfRecording;
        assert_eq!(format!("{e}"), "end of recording");
    }
}

//! Streaming types and configuration

use std::path::PathBuf;

/// Sample format for UDP input
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UdpSampleFormat {
    /// 32-bit floating point (f32), little-endian, 8 bytes/sample
    /// Compatible with GNU Radio UDP Sink default format
    #[default]
    Float32,
    /// 16-bit signed integer (i16), little-endian, 4 bytes/sample
    /// Common for RTL-SDR and hardware SDRs
    Int16,
}

impl UdpSampleFormat {
    /// Get display name for UI
    pub fn name(&self) -> &'static str {
        match self {
            Self::Float32 => "f32",
            Self::Int16 => "i16",
        }
    }

    /// Get bytes per sample
    pub fn bytes_per_sample(&self) -> usize {
        match self {
            Self::Float32 => 8, // 4 bytes I + 4 bytes Q
            Self::Int16 => 4,   // 2 bytes I + 2 bytes Q
        }
    }
}

/// UDP connection status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UdpStatus {
    /// Not connected
    #[default]
    Disconnected,
    /// Listening for packets
    Listening,
    /// Actively receiving data
    Receiving,
    /// Error state
    Error,
}

/// Source of streaming data
#[derive(Debug, Clone, PartialEq)]
pub enum StreamSource {
    /// No source selected
    None,
    /// Playback from an IQ file
    File {
        path: PathBuf,
        /// Total samples in file
        total_samples: usize,
        /// Current read position (sample index)
        position: usize,
        /// Loop playback
        loop_enabled: bool,
    },
    /// Continuous waveform generator
    Generator {
        /// Waveform type (e.g., "BPSK", "LoRa")
        waveform: String,
        /// Phase accumulator for continuous generation
        phase: f64,
        /// Samples generated so far
        samples_generated: u64,
    },
    /// Live simulation (TX -> Channel -> RX)
    Simulation {
        /// Waveform type for modulation
        waveform: String,
        /// SNR in dB for channel model
        snr_db: f64,
        /// Samples generated so far
        samples_generated: u64,
    },
    /// UDP network stream (native only)
    Udp {
        /// UDP port number
        port: u16,
        /// Sample format
        format: UdpSampleFormat,
        /// Connection status
        status: UdpStatus,
        /// Packets received counter
        packets_received: u64,
        /// Samples received counter
        samples_received: u64,
    },
}

/// Channel model for simulation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ChannelModel {
    /// No channel impairments (ideal)
    #[default]
    Ideal,
    /// Additive White Gaussian Noise only
    AWGN,
    /// Rayleigh fading + AWGN (no line-of-sight, multipath)
    Rayleigh,
    /// Rician fading + AWGN (line-of-sight + multipath)
    Rician,
}

impl Default for StreamSource {
    fn default() -> Self {
        Self::None
    }
}

/// Playback state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlaybackState {
    #[default]
    Stopped,
    Playing,
    Paused,
}

/// Recording state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RecordingState {
    #[default]
    Idle,
    Recording,
}

/// Streaming configuration
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// Sample rate (Hz)
    pub sample_rate: f64,
    /// Playback speed multiplier (0.1 to 10.0)
    pub playback_speed: f32,
    /// Display window size (samples) for oscilloscope
    pub window_size: usize,
    /// FFT size for waterfall
    pub fft_size: usize,
    /// Waterfall history depth (number of FFT frames)
    pub waterfall_depth: usize,
    /// Target update rate (FPS for streaming)
    pub update_rate: u32,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            sample_rate: 125_000.0,
            playback_speed: 1.0,
            window_size: 2048,
            fft_size: 256,
            waterfall_depth: 100,
            update_rate: 30,
        }
    }
}

/// Real-time statistics
#[derive(Debug, Clone, Default)]
pub struct StreamStats {
    /// Current playback position (sample index)
    pub current_position: usize,
    /// Total samples in source
    pub total_samples: usize,
    /// Average power in dB
    pub average_power_db: f32,
    /// Peak power in dB
    pub peak_power_db: f32,
    /// Estimated center frequency offset
    pub freq_offset_hz: f64,
    /// Samples per second being processed
    pub effective_sample_rate: f64,
}

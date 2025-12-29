//! Real-time streaming module for SDR visualization
//!
//! This module provides streaming playback and visualization capabilities:
//! - File playback of IQ recordings
//! - Continuous waveform generation
//! - UDP network streaming (native only)
//! - Ring buffer for efficient sliding window display
//! - Waterfall spectrogram state management

mod ring_buffer;
mod types;
pub mod waterfall;
mod manager;

// UDP receiver (native only - not available in WASM)
#[cfg(not(target_arch = "wasm32"))]
pub mod udp;

pub use ring_buffer::RingBuffer;
pub use types::{ChannelModel, PlaybackState, RecordingState, StreamConfig, StreamSource, StreamStats};
pub use types::{UdpSampleFormat, UdpStatus};
pub use waterfall::WaterfallState;
pub use manager::StreamManager;

#[cfg(not(target_arch = "wasm32"))]
pub use udp::{UdpReceiver, UdpStats};

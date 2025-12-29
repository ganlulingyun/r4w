//! UDP I/Q sample receiver for real-time streaming
//!
//! Receives raw I/Q samples over UDP, compatible with:
//! - GNU Radio UDP Sink block (f32 format)
//! - RTL-SDR tools (i16 format)
//! - Custom SDR applications
//!
//! # Example
//!
//! ```ignore
//! // Start receiver on port 5000 with f32 format
//! let receiver = UdpReceiver::start(5000, UdpSampleFormat::Float32)?;
//!
//! // Poll for samples (non-blocking)
//! if let Some(samples) = receiver.poll() {
//!     // Process samples...
//! }
//!
//! // Receiver stops when dropped
//! ```

use r4w_core::types::IQSample;
use std::io;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use super::types::UdpSampleFormat;

/// Maximum UDP packet size (standard Ethernet MTU minus headers)
#[allow(dead_code)]
const MAX_PACKET_SIZE: usize = 65507;

/// Receive buffer size (4KB typical for SDR streams)
const RECV_BUFFER_SIZE: usize = 4096;

/// Socket read timeout for checking stop flag
const SOCKET_TIMEOUT_MS: u64 = 100;

/// Message types from UDP thread to main thread
pub enum UdpMessage {
    /// Batch of received samples
    Samples(Vec<IQSample>),
    /// Status update
    Status(UdpThreadStatus),
    /// Error occurred
    Error(String),
}

/// Status from UDP thread
#[derive(Debug, Clone)]
pub enum UdpThreadStatus {
    /// Successfully bound and listening
    Listening,
    /// Receiving data
    Receiving,
    /// Thread stopped
    Stopped,
}

/// Statistics from UDP receiver
#[derive(Debug, Clone, Default)]
pub struct UdpStats {
    /// Number of UDP packets received
    pub packets_received: u64,
    /// Number of I/Q samples received
    pub samples_received: u64,
    /// Number of bytes received
    pub bytes_received: u64,
    /// Number of errors encountered
    pub errors: u64,
}

/// UDP receiver handle - manages background thread
pub struct UdpReceiver {
    /// Channel to receive samples and status
    rx: Receiver<UdpMessage>,
    /// Thread handle
    handle: Option<JoinHandle<()>>,
    /// Flag to signal thread to stop
    stop_flag: Arc<AtomicBool>,
    /// Accumulated statistics
    stats: UdpStats,
    /// Last error message
    last_error: Option<String>,
}

impl UdpReceiver {
    /// Start UDP receiver on specified port
    ///
    /// # Arguments
    /// * `port` - UDP port to listen on (1024-65535 recommended)
    /// * `format` - Sample format (Float32 or Int16)
    ///
    /// # Returns
    /// * `Ok(UdpReceiver)` - Receiver handle
    /// * `Err(String)` - Error message if binding fails
    pub fn start(port: u16, format: UdpSampleFormat) -> Result<Self, String> {
        let (tx, rx) = mpsc::channel();
        let stop_flag = Arc::new(AtomicBool::new(false));
        let stop_flag_clone = stop_flag.clone();

        let handle = thread::Builder::new()
            .name(format!("udp-receiver-{}", port))
            .spawn(move || {
                Self::receiver_thread(port, format, tx, stop_flag_clone);
            })
            .map_err(|e| format!("Failed to spawn thread: {}", e))?;

        Ok(Self {
            rx,
            handle: Some(handle),
            stop_flag,
            stats: UdpStats::default(),
            last_error: None,
        })
    }

    /// Receiver thread main loop
    fn receiver_thread(
        port: u16,
        format: UdpSampleFormat,
        tx: Sender<UdpMessage>,
        stop_flag: Arc<AtomicBool>,
    ) {
        // Bind UDP socket
        let socket = match UdpSocket::bind(format!("0.0.0.0:{}", port)) {
            Ok(s) => s,
            Err(e) => {
                let _ = tx.send(UdpMessage::Error(format!("Bind failed on port {}: {}", port, e)));
                return;
            }
        };

        // Set non-blocking with timeout for stop checking
        if let Err(e) = socket.set_read_timeout(Some(Duration::from_millis(SOCKET_TIMEOUT_MS))) {
            let _ = tx.send(UdpMessage::Error(format!("Set timeout failed: {}", e)));
            return;
        }

        // Notify listening status
        let _ = tx.send(UdpMessage::Status(UdpThreadStatus::Listening));

        tracing::info!("UDP receiver listening on port {} ({:?} format)", port, format);

        // Receive buffer
        let mut buf = [0u8; RECV_BUFFER_SIZE];
        let mut has_received = false;

        while !stop_flag.load(Ordering::Relaxed) {
            match socket.recv(&mut buf) {
                Ok(len) => {
                    if !has_received {
                        has_received = true;
                        let _ = tx.send(UdpMessage::Status(UdpThreadStatus::Receiving));
                    }

                    let samples = Self::parse_samples(&buf[..len], format);
                    if !samples.is_empty() {
                        if tx.send(UdpMessage::Samples(samples)).is_err() {
                            // Channel closed, exit thread
                            break;
                        }
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    // Timeout - check stop flag and continue
                    continue;
                }
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                    // Windows uses TimedOut instead of WouldBlock
                    continue;
                }
                Err(e) => {
                    let _ = tx.send(UdpMessage::Error(format!("Recv error: {}", e)));
                }
            }
        }

        let _ = tx.send(UdpMessage::Status(UdpThreadStatus::Stopped));
        tracing::info!("UDP receiver stopped");
    }

    /// Parse raw bytes into IQ samples
    fn parse_samples(data: &[u8], format: UdpSampleFormat) -> Vec<IQSample> {
        match format {
            UdpSampleFormat::Float32 => Self::parse_f32(data),
            UdpSampleFormat::Int16 => Self::parse_i16(data),
        }
    }

    /// Parse f32 little-endian interleaved I/Q samples
    fn parse_f32(data: &[u8]) -> Vec<IQSample> {
        // 8 bytes per sample (4 bytes I + 4 bytes Q)
        let num_samples = data.len() / 8;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * 8;
            if offset + 8 <= data.len() {
                let i_val = f32::from_le_bytes([
                    data[offset],
                    data[offset + 1],
                    data[offset + 2],
                    data[offset + 3],
                ]) as f64;
                let q_val = f32::from_le_bytes([
                    data[offset + 4],
                    data[offset + 5],
                    data[offset + 6],
                    data[offset + 7],
                ]) as f64;
                samples.push(IQSample::new(i_val, q_val));
            }
        }
        samples
    }

    /// Parse i16 little-endian interleaved I/Q samples
    fn parse_i16(data: &[u8]) -> Vec<IQSample> {
        // 4 bytes per sample (2 bytes I + 2 bytes Q)
        let num_samples = data.len() / 4;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * 4;
            if offset + 4 <= data.len() {
                let i_raw = i16::from_le_bytes([data[offset], data[offset + 1]]);
                let q_raw = i16::from_le_bytes([data[offset + 2], data[offset + 3]]);
                // Scale to [-1.0, 1.0] range
                let i_val = i_raw as f64 / 32768.0;
                let q_val = q_raw as f64 / 32768.0;
                samples.push(IQSample::new(i_val, q_val));
            }
        }
        samples
    }

    /// Poll for received samples (non-blocking)
    ///
    /// Returns all samples received since last poll, or None if no samples available.
    pub fn poll(&mut self) -> Option<Vec<IQSample>> {
        let mut all_samples = Vec::new();

        loop {
            match self.rx.try_recv() {
                Ok(UdpMessage::Samples(samples)) => {
                    self.stats.packets_received += 1;
                    self.stats.samples_received += samples.len() as u64;
                    self.stats.bytes_received += (samples.len() * 16) as u64; // Complex64 = 16 bytes
                    all_samples.extend(samples);
                }
                Ok(UdpMessage::Status(status)) => {
                    tracing::debug!("UDP status: {:?}", status);
                }
                Ok(UdpMessage::Error(err)) => {
                    self.stats.errors += 1;
                    self.last_error = Some(err.clone());
                    tracing::warn!("UDP error: {}", err);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    tracing::debug!("UDP channel disconnected");
                    break;
                }
            }
        }

        if all_samples.is_empty() {
            None
        } else {
            Some(all_samples)
        }
    }

    /// Stop the receiver thread
    pub fn stop(&mut self) {
        self.stop_flag.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }

    /// Get current statistics
    pub fn stats(&self) -> &UdpStats {
        &self.stats
    }

    /// Get last error message, if any
    pub fn last_error(&self) -> Option<&str> {
        self.last_error.as_deref()
    }

    /// Check if receiver is still running
    pub fn is_running(&self) -> bool {
        self.handle.is_some() && !self.stop_flag.load(Ordering::Relaxed)
    }
}

impl Drop for UdpReceiver {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_f32() {
        // Create test data: one sample with I=1.0, Q=0.5
        let i_bytes = 1.0f32.to_le_bytes();
        let q_bytes = 0.5f32.to_le_bytes();
        let mut data = Vec::new();
        data.extend_from_slice(&i_bytes);
        data.extend_from_slice(&q_bytes);

        let samples = UdpReceiver::parse_f32(&data);
        assert_eq!(samples.len(), 1);
        assert!((samples[0].re - 1.0).abs() < 1e-6);
        assert!((samples[0].im - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_parse_i16() {
        // Create test data: one sample with I=16384 (0.5), Q=-16384 (-0.5)
        let mut data = Vec::new();
        data.extend_from_slice(&16384i16.to_le_bytes());
        data.extend_from_slice(&(-16384i16).to_le_bytes());

        let samples = UdpReceiver::parse_i16(&data);
        assert_eq!(samples.len(), 1);
        assert!((samples[0].re - 0.5).abs() < 0.001);
        assert!((samples[0].im - (-0.5)).abs() < 0.001);
    }

    #[test]
    fn test_parse_multiple_samples() {
        // Two f32 samples
        let mut data = Vec::new();
        for _ in 0..2 {
            data.extend_from_slice(&1.0f32.to_le_bytes());
            data.extend_from_slice(&0.0f32.to_le_bytes());
        }

        let samples = UdpReceiver::parse_f32(&data);
        assert_eq!(samples.len(), 2);
    }

    #[test]
    fn test_parse_partial_sample() {
        // Only 6 bytes (not enough for a full f32 sample)
        let data = [0u8; 6];
        let samples = UdpReceiver::parse_f32(&data);
        assert_eq!(samples.len(), 0);
    }
}

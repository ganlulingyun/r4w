//! UDP Source/Sink — Network IQ streaming over UDP
//!
//! Stream IQ samples over UDP for network-based SDR applications.
//! Widely used for USRP remote streaming, inter-process communication,
//! and integration with external tools (GQRX, MATLAB, etc.).
//! GNU Radio equivalent: `udp_source`, `udp_sink`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::udp_source_sink::{UdpSinkConfig, UdpSourceConfig, UdpPacket};
//!
//! // Configure a sink
//! let config = UdpSinkConfig {
//!     host: "127.0.0.1".to_string(),
//!     port: 50000,
//!     payload_size: 1472,
//!     add_header: true,
//! };
//! assert_eq!(config.payload_size, 1472);
//!
//! // Create a packet with sequence number
//! let pkt = UdpPacket::new(42, &[1.0, 2.0, 3.0, 4.0]);
//! let bytes = pkt.to_bytes();
//! let parsed = UdpPacket::from_bytes(&bytes).unwrap();
//! assert_eq!(parsed.seq, 42);
//! assert_eq!(parsed.samples.len(), 4);
//! ```

/// Configuration for a UDP sink (transmitter).
#[derive(Debug, Clone)]
pub struct UdpSinkConfig {
    /// Destination host address.
    pub host: String,
    /// Destination port.
    pub port: u16,
    /// Maximum payload size in bytes (default: 1472 for standard MTU).
    pub payload_size: usize,
    /// Whether to prepend a sequence number header.
    pub add_header: bool,
}

impl Default for UdpSinkConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".to_string(),
            port: 50000,
            payload_size: 1472,
            add_header: true,
        }
    }
}

/// Configuration for a UDP source (receiver).
#[derive(Debug, Clone)]
pub struct UdpSourceConfig {
    /// Local bind address.
    pub host: String,
    /// Local bind port.
    pub port: u16,
    /// Maximum receive buffer size in bytes.
    pub recv_buf_size: usize,
    /// Whether packets include a sequence number header.
    pub has_header: bool,
    /// Timeout for recv in milliseconds (0 = blocking).
    pub timeout_ms: u64,
}

impl Default for UdpSourceConfig {
    fn default() -> Self {
        Self {
            host: "0.0.0.0".to_string(),
            port: 50000,
            recv_buf_size: 65536,
            has_header: true,
            timeout_ms: 1000,
        }
    }
}

/// A UDP packet containing IQ samples as interleaved f32 values.
///
/// Wire format (when header enabled):
/// ```text
/// [seq_num: u32 LE (4 bytes)] [sample_0_f32 LE] [sample_1_f32 LE] ...
/// ```
///
/// Samples are interleaved I/Q as f32 (4 bytes each).
#[derive(Debug, Clone)]
pub struct UdpPacket {
    /// Sequence number (monotonically increasing per stream).
    pub seq: u32,
    /// Interleaved IQ samples as f32 values.
    pub samples: Vec<f32>,
}

impl UdpPacket {
    /// Create a new packet.
    pub fn new(seq: u32, samples: &[f32]) -> Self {
        Self {
            seq,
            samples: samples.to_vec(),
        }
    }

    /// Serialize to bytes (with header).
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(4 + self.samples.len() * 4);
        buf.extend_from_slice(&self.seq.to_le_bytes());
        for &s in &self.samples {
            buf.extend_from_slice(&s.to_le_bytes());
        }
        buf
    }

    /// Serialize to bytes without header (raw samples only).
    pub fn to_bytes_no_header(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.samples.len() * 4);
        for &s in &self.samples {
            buf.extend_from_slice(&s.to_le_bytes());
        }
        buf
    }

    /// Parse from bytes (with header).
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 4 {
            return None;
        }
        let seq = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let sample_bytes = &data[4..];
        if sample_bytes.len() % 4 != 0 {
            return None;
        }
        let samples: Vec<f32> = sample_bytes
            .chunks_exact(4)
            .map(|chunk| f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
            .collect();
        Some(Self { seq, samples })
    }

    /// Parse from bytes without header (raw samples).
    pub fn from_bytes_no_header(data: &[u8]) -> Option<Self> {
        if data.len() % 4 != 0 {
            return None;
        }
        let samples: Vec<f32> = data
            .chunks_exact(4)
            .map(|chunk| f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
            .collect();
        Some(Self { seq: 0, samples })
    }

    /// Number of complex IQ samples (pairs of f32).
    pub fn num_iq_samples(&self) -> usize {
        self.samples.len() / 2
    }

    /// Maximum f32 samples that fit in `payload_size` bytes (with header).
    pub fn max_samples_for_payload(payload_size: usize) -> usize {
        (payload_size.saturating_sub(4)) / 4
    }

    /// Maximum f32 samples that fit in `payload_size` bytes (without header).
    pub fn max_samples_for_payload_no_header(payload_size: usize) -> usize {
        payload_size / 4
    }
}

/// UDP Sink — sends IQ samples over UDP.
///
/// Non-blocking sender that packetizes IQ data and transmits via UDP.
#[derive(Debug)]
pub struct UdpSink {
    /// Configuration.
    config: UdpSinkConfig,
    /// Socket (created on first send).
    socket: Option<std::net::UdpSocket>,
    /// Sequence counter.
    seq: u32,
    /// Total samples sent.
    total_samples: u64,
    /// Total packets sent.
    total_packets: u64,
}

impl UdpSink {
    /// Create a new UDP sink.
    pub fn new(config: UdpSinkConfig) -> Self {
        Self {
            config,
            socket: None,
            seq: 0,
            total_samples: 0,
            total_packets: 0,
        }
    }

    /// Initialize the socket (binds to any available port).
    pub fn connect(&mut self) -> std::io::Result<()> {
        let sock = std::net::UdpSocket::bind("0.0.0.0:0")?;
        sock.set_nonblocking(true)?;
        self.socket = Some(sock);
        Ok(())
    }

    /// Send IQ samples. Automatically packetizes to fit payload_size.
    ///
    /// Returns number of packets sent.
    pub fn send(&mut self, samples: &[f32]) -> std::io::Result<usize> {
        if self.socket.is_none() {
            self.connect()?;
        }
        let sock = self.socket.as_ref().unwrap();
        let dest = format!("{}:{}", self.config.host, self.config.port);

        let max_samples = if self.config.add_header {
            UdpPacket::max_samples_for_payload(self.config.payload_size)
        } else {
            UdpPacket::max_samples_for_payload_no_header(self.config.payload_size)
        };
        let max_samples = max_samples.max(1);

        let mut packets_sent = 0;
        for chunk in samples.chunks(max_samples) {
            let pkt = UdpPacket::new(self.seq, chunk);
            let bytes = if self.config.add_header {
                pkt.to_bytes()
            } else {
                pkt.to_bytes_no_header()
            };
            sock.send_to(&bytes, &dest)?;
            self.seq = self.seq.wrapping_add(1);
            self.total_samples += chunk.len() as u64;
            self.total_packets += 1;
            packets_sent += 1;
        }
        Ok(packets_sent)
    }

    /// Get total samples sent.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get total packets sent.
    pub fn total_packets(&self) -> u64 {
        self.total_packets
    }

    /// Get current sequence number.
    pub fn seq(&self) -> u32 {
        self.seq
    }
}

/// UDP Source — receives IQ samples over UDP.
///
/// Binds to a local port and receives IQ sample packets.
#[derive(Debug)]
pub struct UdpSource {
    /// Configuration.
    config: UdpSourceConfig,
    /// Socket.
    socket: Option<std::net::UdpSocket>,
    /// Last received sequence number.
    last_seq: Option<u32>,
    /// Total samples received.
    total_samples: u64,
    /// Total packets received.
    total_packets: u64,
    /// Dropped packets (detected by sequence gaps).
    dropped_packets: u64,
}

impl UdpSource {
    /// Create a new UDP source.
    pub fn new(config: UdpSourceConfig) -> Self {
        Self {
            config,
            socket: None,
            last_seq: None,
            total_samples: 0,
            total_packets: 0,
            dropped_packets: 0,
        }
    }

    /// Bind to the configured address and port.
    pub fn bind(&mut self) -> std::io::Result<()> {
        let addr = format!("{}:{}", self.config.host, self.config.port);
        let sock = std::net::UdpSocket::bind(&addr)?;
        if self.config.timeout_ms > 0 {
            sock.set_read_timeout(Some(std::time::Duration::from_millis(
                self.config.timeout_ms,
            )))?;
        }
        self.socket = Some(sock);
        Ok(())
    }

    /// Receive one packet. Returns None on timeout.
    pub fn recv(&mut self) -> std::io::Result<Option<UdpPacket>> {
        if self.socket.is_none() {
            self.bind()?;
        }
        let sock = self.socket.as_ref().unwrap();
        let mut buf = vec![0u8; self.config.recv_buf_size];
        match sock.recv_from(&mut buf) {
            Ok((n, _addr)) => {
                let pkt = if self.config.has_header {
                    UdpPacket::from_bytes(&buf[..n])
                } else {
                    UdpPacket::from_bytes_no_header(&buf[..n])
                };
                if let Some(ref pkt) = pkt {
                    // Track packet loss
                    if self.config.has_header {
                        if let Some(last) = self.last_seq {
                            let expected = last.wrapping_add(1);
                            if pkt.seq != expected {
                                let gap = pkt.seq.wrapping_sub(expected) as u64;
                                self.dropped_packets += gap;
                            }
                        }
                        self.last_seq = Some(pkt.seq);
                    }
                    self.total_samples += pkt.samples.len() as u64;
                    self.total_packets += 1;
                }
                Ok(pkt)
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                Ok(None)
            }
            Err(e) => Err(e),
        }
    }

    /// Get total samples received.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get total packets received.
    pub fn total_packets(&self) -> u64 {
        self.total_packets
    }

    /// Get dropped packet count.
    pub fn dropped_packets(&self) -> u64 {
        self.dropped_packets
    }

    /// Get packet loss ratio (0.0 = no loss).
    pub fn loss_ratio(&self) -> f64 {
        let total = self.total_packets + self.dropped_packets;
        if total == 0 {
            0.0
        } else {
            self.dropped_packets as f64 / total as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packet_roundtrip_with_header() {
        let samples = vec![1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0];
        let pkt = UdpPacket::new(42, &samples);
        let bytes = pkt.to_bytes();
        let parsed = UdpPacket::from_bytes(&bytes).unwrap();
        assert_eq!(parsed.seq, 42);
        assert_eq!(parsed.samples, samples);
    }

    #[test]
    fn test_packet_roundtrip_no_header() {
        let samples = vec![1.0f32, -1.0, 0.5, -0.5];
        let pkt = UdpPacket::new(0, &samples);
        let bytes = pkt.to_bytes_no_header();
        let parsed = UdpPacket::from_bytes_no_header(&bytes).unwrap();
        assert_eq!(parsed.samples, samples);
    }

    #[test]
    fn test_packet_too_short() {
        assert!(UdpPacket::from_bytes(&[1, 2]).is_none());
    }

    #[test]
    fn test_packet_misaligned() {
        // 4 byte header + 3 bytes (not a multiple of 4)
        assert!(UdpPacket::from_bytes(&[0, 0, 0, 0, 1, 2, 3]).is_none());
    }

    #[test]
    fn test_packet_empty() {
        let pkt = UdpPacket::new(0, &[]);
        let bytes = pkt.to_bytes();
        assert_eq!(bytes.len(), 4); // header only
        let parsed = UdpPacket::from_bytes(&bytes).unwrap();
        assert!(parsed.samples.is_empty());
    }

    #[test]
    fn test_num_iq_samples() {
        let pkt = UdpPacket::new(0, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert_eq!(pkt.num_iq_samples(), 3);
    }

    #[test]
    fn test_max_samples_for_payload() {
        // 1472 bytes payload - 4 header = 1468 / 4 = 367 f32 samples
        assert_eq!(UdpPacket::max_samples_for_payload(1472), 367);
        // Without header: 1472 / 4 = 368
        assert_eq!(UdpPacket::max_samples_for_payload_no_header(1472), 368);
    }

    #[test]
    fn test_sink_config_default() {
        let config = UdpSinkConfig::default();
        assert_eq!(config.host, "127.0.0.1");
        assert_eq!(config.port, 50000);
        assert_eq!(config.payload_size, 1472);
        assert!(config.add_header);
    }

    #[test]
    fn test_source_config_default() {
        let config = UdpSourceConfig::default();
        assert_eq!(config.host, "0.0.0.0");
        assert_eq!(config.port, 50000);
        assert!(config.has_header);
    }

    #[test]
    fn test_udp_loopback() {
        // Create source and sink on localhost
        let src_config = UdpSourceConfig {
            host: "127.0.0.1".to_string(),
            port: 0, // OS-assigned
            has_header: true,
            timeout_ms: 500,
            ..Default::default()
        };
        let mut source = UdpSource::new(src_config);
        source.bind().unwrap();

        // Get the actual bound port
        let local_addr = source.socket.as_ref().unwrap().local_addr().unwrap();
        let port = local_addr.port();

        let sink_config = UdpSinkConfig {
            host: "127.0.0.1".to_string(),
            port,
            payload_size: 1472,
            add_header: true,
        };
        let mut sink = UdpSink::new(sink_config);

        // Send samples
        let samples = vec![1.0f32, 2.0, 3.0, 4.0];
        let packets = sink.send(&samples).unwrap();
        assert_eq!(packets, 1);
        assert_eq!(sink.total_samples(), 4);

        // Receive
        let pkt = source.recv().unwrap().unwrap();
        assert_eq!(pkt.seq, 0);
        assert_eq!(pkt.samples, samples);
        assert_eq!(source.total_samples(), 4);
    }

    #[test]
    fn test_sink_packetization() {
        // Small payload to force multiple packets
        let config = UdpSinkConfig {
            host: "127.0.0.1".to_string(),
            port: 0, // Will fail send but that's OK for testing packet count logic
            payload_size: 20, // 4 header + 16 = 4 f32 samples max
            add_header: true,
        };
        let sink = UdpSink::new(config);
        let max = UdpPacket::max_samples_for_payload(20);
        assert_eq!(max, 4);
    }

    #[test]
    fn test_sequence_wrapping() {
        let pkt1 = UdpPacket::new(u32::MAX, &[1.0]);
        let bytes1 = pkt1.to_bytes();
        let parsed1 = UdpPacket::from_bytes(&bytes1).unwrap();
        assert_eq!(parsed1.seq, u32::MAX);
    }

    #[test]
    fn test_udp_source_timeout() {
        let config = UdpSourceConfig {
            host: "127.0.0.1".to_string(),
            port: 0,
            timeout_ms: 10, // very short
            ..Default::default()
        };
        let mut source = UdpSource::new(config);
        source.bind().unwrap();
        // Should return None on timeout, not error
        let result = source.recv().unwrap();
        assert!(result.is_none());
    }
}

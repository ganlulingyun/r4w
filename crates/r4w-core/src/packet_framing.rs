//! Packet Formatter / Parser
//!
//! Generic packet framing for radio protocols. Handles sync word insertion,
//! header formatting, payload encapsulation, and CRC protection.
//!
//! ## Frame Format
//!
//! ```text
//! ┌──────────┬────────┬─────────┬─────────┬──────┐
//! │ Preamble │ Sync   │ Header  │ Payload │ CRC  │
//! │ (opt.)   │ Word   │ (len,   │         │      │
//! │          │        │  seq,   │         │      │
//! │          │        │  flags) │         │      │
//! └──────────┴────────┴─────────┴─────────┴──────┘
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::packet_framing::{PacketFormatter, PacketParser, FrameConfig};
//!
//! let config = FrameConfig::default();
//! let mut formatter = PacketFormatter::new(config.clone());
//! let parser = PacketParser::new(config);
//!
//! let payload = b"Hello Radio!";
//! let frame = formatter.format(payload);
//!
//! let parsed = parser.parse(&frame);
//! assert!(parsed.is_some());
//! let pkt = parsed.unwrap();
//! assert_eq!(&pkt.payload, payload);
//! ```

use crate::crc::{CrcComputer, Crc16, Crc32};

/// CRC type for frame integrity check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrcType {
    /// No CRC.
    None,
    /// CRC-16 (2 bytes).
    Crc16,
    /// CRC-32 (4 bytes).
    Crc32,
}

/// Header format options.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HeaderFormat {
    /// Minimal: 1-byte length only.
    Minimal,
    /// Standard: 2-byte length + 1-byte sequence + 1-byte flags.
    Standard,
    /// Extended: 2-byte length + 1-byte sequence + 1-byte flags + 2-byte source + 2-byte dest.
    Extended,
}

/// Frame configuration.
#[derive(Debug, Clone)]
pub struct FrameConfig {
    /// Preamble pattern (repeated bytes before sync word).
    pub preamble: Vec<u8>,
    /// Number of preamble repetitions.
    pub preamble_count: usize,
    /// Sync word (access code) for frame detection.
    pub sync_word: Vec<u8>,
    /// Header format.
    pub header_format: HeaderFormat,
    /// CRC type.
    pub crc_type: CrcType,
    /// Maximum payload length in bytes.
    pub max_payload: usize,
}

impl Default for FrameConfig {
    fn default() -> Self {
        Self {
            preamble: vec![0xAA], // Alternating bits for clock recovery
            preamble_count: 4,
            sync_word: vec![0x7E, 0x7E], // HDLC-like sync
            header_format: HeaderFormat::Standard,
            crc_type: CrcType::Crc16,
            max_payload: 255,
        }
    }
}

impl FrameConfig {
    /// AX.25-inspired frame config with distinct sync word.
    pub fn ax25() -> Self {
        Self {
            preamble: vec![0xAA],
            preamble_count: 8,
            sync_word: vec![0x7E, 0x7E],
            header_format: HeaderFormat::Standard,
            crc_type: CrcType::Crc16,
            max_payload: 256,
        }
    }

    /// Simple frame with minimal overhead.
    pub fn minimal() -> Self {
        Self {
            preamble: vec![0xAA],
            preamble_count: 2,
            sync_word: vec![0xD3, 0x91], // Unique 2-byte sync
            header_format: HeaderFormat::Minimal,
            crc_type: CrcType::None,
            max_payload: 255,
        }
    }

    /// ISM band packet radio config.
    pub fn ism_packet() -> Self {
        Self {
            preamble: vec![0xAA],
            preamble_count: 4,
            sync_word: vec![0x2D, 0xD4], // Common ISM sync word
            header_format: HeaderFormat::Extended,
            crc_type: CrcType::Crc16,
            max_payload: 255,
        }
    }

    /// Header size in bytes for this configuration.
    pub fn header_size(&self) -> usize {
        match self.header_format {
            HeaderFormat::Minimal => 1,
            HeaderFormat::Standard => 4,
            HeaderFormat::Extended => 8,
        }
    }

    /// CRC size in bytes.
    pub fn crc_size(&self) -> usize {
        match self.crc_type {
            CrcType::None => 0,
            CrcType::Crc16 => 2,
            CrcType::Crc32 => 4,
        }
    }

    /// Total overhead (preamble + sync + header + CRC).
    pub fn overhead(&self) -> usize {
        self.preamble.len() * self.preamble_count
            + self.sync_word.len()
            + self.header_size()
            + self.crc_size()
    }
}

/// Parsed packet header.
#[derive(Debug, Clone)]
pub struct FrameHeader {
    /// Payload length.
    pub length: u16,
    /// Sequence number (0 for Minimal format).
    pub sequence: u8,
    /// Flags byte (0 for Minimal format).
    pub flags: u8,
    /// Source address (0 for Minimal/Standard format).
    pub source: u16,
    /// Destination address (0 for Minimal/Standard format).
    pub destination: u16,
}

/// Parsed frame result.
#[derive(Debug, Clone)]
pub struct ParsedFrame {
    /// Header fields.
    pub header: FrameHeader,
    /// Payload data.
    pub payload: Vec<u8>,
    /// Whether CRC check passed.
    pub crc_ok: bool,
}

/// Packet formatter (TX side).
///
/// Builds frames from payloads: preamble + sync + header + payload + CRC.
#[derive(Debug, Clone)]
pub struct PacketFormatter {
    config: FrameConfig,
    sequence: u8,
}

impl PacketFormatter {
    /// Create a new packet formatter.
    pub fn new(config: FrameConfig) -> Self {
        Self {
            config,
            sequence: 0,
        }
    }

    /// Format a payload into a complete frame.
    pub fn format(&mut self, payload: &[u8]) -> Vec<u8> {
        self.format_with_header(payload, 0, 0, 0)
    }

    /// Format with explicit addressing (for Extended header).
    pub fn format_addressed(&mut self, payload: &[u8], source: u16, destination: u16) -> Vec<u8> {
        self.format_with_header(payload, 0, source, destination)
    }

    /// Format with full header control.
    pub fn format_with_header(
        &mut self,
        payload: &[u8],
        flags: u8,
        source: u16,
        destination: u16,
    ) -> Vec<u8> {
        let len = payload.len().min(self.config.max_payload);
        let mut frame = Vec::with_capacity(self.config.overhead() + len);

        // Preamble
        for _ in 0..self.config.preamble_count {
            frame.extend_from_slice(&self.config.preamble);
        }

        // Sync word
        frame.extend_from_slice(&self.config.sync_word);

        // Header
        let header_start = frame.len();
        match self.config.header_format {
            HeaderFormat::Minimal => {
                frame.push(len as u8);
            }
            HeaderFormat::Standard => {
                frame.extend_from_slice(&(len as u16).to_le_bytes());
                frame.push(self.sequence);
                frame.push(flags);
            }
            HeaderFormat::Extended => {
                frame.extend_from_slice(&(len as u16).to_le_bytes());
                frame.push(self.sequence);
                frame.push(flags);
                frame.extend_from_slice(&source.to_le_bytes());
                frame.extend_from_slice(&destination.to_le_bytes());
            }
        }

        // Payload
        frame.extend_from_slice(&payload[..len]);

        // CRC (over header + payload)
        let crc_data = &frame[header_start..];
        match self.config.crc_type {
            CrcType::None => {}
            CrcType::Crc16 => {
                let crc = Crc16::compute(crc_data);
                frame.extend_from_slice(&crc.to_le_bytes());
            }
            CrcType::Crc32 => {
                let crc = Crc32::compute(crc_data);
                frame.extend_from_slice(&crc.to_le_bytes());
            }
        }

        self.sequence = self.sequence.wrapping_add(1);
        frame
    }

    /// Reset the sequence counter.
    pub fn reset(&mut self) {
        self.sequence = 0;
    }

    /// Get the current sequence number.
    pub fn sequence(&self) -> u8 {
        self.sequence
    }
}

/// Packet parser (RX side).
///
/// Detects sync word, extracts header and payload, verifies CRC.
#[derive(Debug, Clone)]
pub struct PacketParser {
    config: FrameConfig,
}

impl PacketParser {
    /// Create a new packet parser.
    pub fn new(config: FrameConfig) -> Self {
        Self { config }
    }

    /// Find the sync word in a byte stream and return the offset after it.
    pub fn find_sync(&self, data: &[u8]) -> Option<usize> {
        let sync = &self.config.sync_word;
        if sync.is_empty() || data.len() < sync.len() {
            return None;
        }
        for i in 0..=(data.len() - sync.len()) {
            if &data[i..i + sync.len()] == sync.as_slice() {
                return Some(i + sync.len());
            }
        }
        None
    }

    /// Parse a complete frame (starting from the sync word or after preamble).
    ///
    /// Automatically searches for the sync word in the input.
    pub fn parse(&self, data: &[u8]) -> Option<ParsedFrame> {
        let start = self.find_sync(data)?;
        self.parse_from_header(&data[start..])
    }

    /// Parse from the header start (after sync word already stripped).
    pub fn parse_from_header(&self, data: &[u8]) -> Option<ParsedFrame> {
        let header_size = self.config.header_size();
        if data.len() < header_size {
            return None;
        }

        // Parse header
        let header = match self.config.header_format {
            HeaderFormat::Minimal => {
                let length = data[0] as u16;
                FrameHeader {
                    length,
                    sequence: 0,
                    flags: 0,
                    source: 0,
                    destination: 0,
                }
            }
            HeaderFormat::Standard => {
                let length = u16::from_le_bytes([data[0], data[1]]);
                FrameHeader {
                    length,
                    sequence: data[2],
                    flags: data[3],
                    source: 0,
                    destination: 0,
                }
            }
            HeaderFormat::Extended => {
                let length = u16::from_le_bytes([data[0], data[1]]);
                FrameHeader {
                    length,
                    sequence: data[2],
                    flags: data[3],
                    source: u16::from_le_bytes([data[4], data[5]]),
                    destination: u16::from_le_bytes([data[6], data[7]]),
                }
            }
        };

        let payload_len = header.length as usize;
        let total_needed = header_size + payload_len + self.config.crc_size();
        if data.len() < total_needed {
            return None;
        }

        let payload = data[header_size..header_size + payload_len].to_vec();

        // Verify CRC
        let crc_ok = match self.config.crc_type {
            CrcType::None => true,
            CrcType::Crc16 => {
                let crc_data = &data[..header_size + payload_len];
                let expected = Crc16::compute(crc_data);
                let crc_offset = header_size + payload_len;
                let received =
                    u16::from_le_bytes([data[crc_offset], data[crc_offset + 1]]);
                expected == received
            }
            CrcType::Crc32 => {
                let crc_data = &data[..header_size + payload_len];
                let expected = Crc32::compute(crc_data);
                let crc_offset = header_size + payload_len;
                let received = u32::from_le_bytes([
                    data[crc_offset],
                    data[crc_offset + 1],
                    data[crc_offset + 2],
                    data[crc_offset + 3],
                ]);
                expected == received
            }
        };

        Some(ParsedFrame {
            header,
            payload,
            crc_ok,
        })
    }

    /// Parse all frames in a byte stream (returns all found frames).
    pub fn parse_stream(&self, data: &[u8]) -> Vec<ParsedFrame> {
        let mut frames = Vec::new();
        let mut offset = 0;

        while offset < data.len() {
            if let Some(sync_end) = self.find_sync(&data[offset..]) {
                let abs_start = offset + sync_end;
                if let Some(frame) = self.parse_from_header(&data[abs_start..]) {
                    let frame_len = self.config.header_size()
                        + frame.header.length as usize
                        + self.config.crc_size();
                    frames.push(frame);
                    offset = abs_start + frame_len;
                } else {
                    offset += sync_end;
                }
            } else {
                break;
            }
        }

        frames
    }
}

/// Compute frame efficiency (payload / total).
pub fn frame_efficiency(config: &FrameConfig, payload_size: usize) -> f64 {
    let total = config.overhead() + payload_size;
    if total > 0 {
        payload_size as f64 / total as f64
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_parse_roundtrip() {
        let config = FrameConfig::default();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let payload = b"Hello Radio!";
        let frame = formatter.format(payload);
        let parsed = parser.parse(&frame);

        assert!(parsed.is_some(), "Should parse frame");
        let pkt = parsed.unwrap();
        assert_eq!(&pkt.payload, payload);
        assert!(pkt.crc_ok, "CRC should pass");
        assert_eq!(pkt.header.sequence, 0);
    }

    #[test]
    fn test_sequence_increments() {
        let config = FrameConfig::default();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        for i in 0..5u8 {
            let frame = formatter.format(b"test");
            let pkt = parser.parse(&frame).unwrap();
            assert_eq!(pkt.header.sequence, i);
        }
    }

    #[test]
    fn test_minimal_format() {
        let config = FrameConfig::minimal();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let payload = b"Minimal overhead";
        let frame = formatter.format(payload);
        let pkt = parser.parse(&frame).unwrap();

        assert_eq!(&pkt.payload, payload);
    }

    #[test]
    fn test_extended_addressing() {
        let config = FrameConfig::ism_packet();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let frame = formatter.format_addressed(b"data", 0x1234, 0x5678);
        let pkt = parser.parse(&frame).unwrap();

        assert_eq!(&pkt.payload, b"data");
        assert_eq!(pkt.header.source, 0x1234);
        assert_eq!(pkt.header.destination, 0x5678);
        assert!(pkt.crc_ok);
    }

    #[test]
    fn test_crc_detects_corruption() {
        let config = FrameConfig::default();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let mut frame = formatter.format(b"Integrity check");
        // Corrupt one payload byte
        let corrupt_idx = frame.len() / 2;
        frame[corrupt_idx] ^= 0xFF;

        let pkt = parser.parse(&frame);
        if let Some(pkt) = pkt {
            assert!(!pkt.crc_ok, "CRC should detect corruption");
        }
    }

    #[test]
    fn test_sync_word_detection() {
        let config = FrameConfig::default();
        let parser = PacketParser::new(config.clone());

        // Sync word embedded in garbage
        let mut data = vec![0x00, 0xFF, 0x12];
        data.extend_from_slice(&config.sync_word);
        data.extend(vec![0x00; 20]);

        let offset = parser.find_sync(&data);
        assert!(offset.is_some());
        assert_eq!(offset.unwrap(), 3 + config.sync_word.len());
    }

    #[test]
    fn test_parse_stream_multiple() {
        let config = FrameConfig::default();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let mut stream = Vec::new();
        for i in 0..3 {
            let msg = format!("Msg{}", i);
            stream.extend(formatter.format(msg.as_bytes()));
        }

        let frames = parser.parse_stream(&stream);
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].payload, b"Msg0");
        assert_eq!(frames[1].payload, b"Msg1");
        assert_eq!(frames[2].payload, b"Msg2");
    }

    #[test]
    fn test_crc32_mode() {
        let config = FrameConfig {
            crc_type: CrcType::Crc32,
            ..Default::default()
        };
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let frame = formatter.format(b"CRC32 test");
        let pkt = parser.parse(&frame).unwrap();
        assert_eq!(&pkt.payload, b"CRC32 test");
        assert!(pkt.crc_ok);
    }

    #[test]
    fn test_no_crc_mode() {
        let config = FrameConfig {
            crc_type: CrcType::None,
            ..Default::default()
        };
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let frame = formatter.format(b"No CRC");
        let pkt = parser.parse(&frame).unwrap();
        assert_eq!(&pkt.payload, b"No CRC");
        assert!(pkt.crc_ok); // Always true when CRC is disabled
    }

    #[test]
    fn test_frame_efficiency() {
        let config = FrameConfig::default();
        let eff = frame_efficiency(&config, 100);
        // Overhead: 4*1 preamble + 2 sync + 4 header + 2 CRC = 12
        // Efficiency: 100/112 ≈ 0.89
        assert!(eff > 0.85 && eff < 0.95, "Efficiency should be ~89%: got {eff:.2}");
    }

    #[test]
    fn test_empty_payload() {
        let config = FrameConfig::default();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let frame = formatter.format(b"");
        let pkt = parser.parse(&frame).unwrap();
        assert!(pkt.payload.is_empty());
        assert!(pkt.crc_ok);
    }

    #[test]
    fn test_max_payload() {
        let config = FrameConfig {
            max_payload: 10,
            ..Default::default()
        };
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        // Payload longer than max should be truncated
        let frame = formatter.format(b"This is longer than 10 bytes");
        let pkt = parser.parse(&frame).unwrap();
        assert_eq!(pkt.payload.len(), 10);
    }

    #[test]
    fn test_ax25_config() {
        let config = FrameConfig::ax25();
        let mut formatter = PacketFormatter::new(config.clone());
        let parser = PacketParser::new(config);

        let frame = formatter.format(b"AX.25 packet");
        let pkt = parser.parse(&frame).unwrap();
        assert_eq!(&pkt.payload, b"AX.25 packet");
        assert!(pkt.crc_ok);
    }
}

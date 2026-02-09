//! Header/Payload Demultiplexer
//!
//! Demultiplexes variable-length packets where the header encodes the payload
//! length. This is a key block for packet-based receivers:
//!
//! 1. Read a fixed-length header from the input stream
//! 2. Parse the header to determine payload length
//! 3. Extract the payload of the decoded length
//! 4. Output (header, payload) pairs
//!
//! Supports configurable header formats (length field position, endianness,
//! CRC validation).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::header_payload_demux::{HeaderPayloadDemux, HeaderConfig};
//!
//! let config = HeaderConfig::simple(4); // 4-byte header, first 2 bytes = length
//! let mut demux = HeaderPayloadDemux::new(config);
//!
//! // Build a packet: [length_lo, length_hi, 0, 0, payload...]
//! let mut data = vec![5, 0, 0, 0]; // header: length=5
//! data.extend_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE]); // 5-byte payload
//! let packets = demux.process(&data);
//! assert_eq!(packets.len(), 1);
//! assert_eq!(packets[0].payload, vec![0xAA, 0xBB, 0xCC, 0xDD, 0xEE]);
//! ```

/// Configuration for the header format.
#[derive(Debug, Clone)]
pub struct HeaderConfig {
    /// Header length in bytes
    pub header_len: usize,
    /// Byte offset of the length field within the header
    pub length_field_offset: usize,
    /// Size of the length field in bytes (1, 2, or 4)
    pub length_field_size: usize,
    /// Whether the length field is big-endian
    pub big_endian: bool,
    /// Maximum allowed payload length (for safety)
    pub max_payload_len: usize,
    /// Whether the length field includes the header length
    pub length_includes_header: bool,
}

impl HeaderConfig {
    /// Simple header format: N-byte header, length in first 2 bytes (LE).
    pub fn simple(header_len: usize) -> Self {
        Self {
            header_len,
            length_field_offset: 0,
            length_field_size: 2,
            big_endian: false,
            max_payload_len: 65535,
            length_includes_header: false,
        }
    }

    /// Custom header format with all parameters.
    pub fn custom(
        header_len: usize,
        length_field_offset: usize,
        length_field_size: usize,
        big_endian: bool,
    ) -> Self {
        assert!(length_field_size <= 4);
        assert!(length_field_offset + length_field_size <= header_len);
        Self {
            header_len,
            length_field_offset,
            length_field_size,
            big_endian,
            max_payload_len: 65535,
            length_includes_header: false,
        }
    }

    /// Parse the length field from a header.
    fn parse_length(&self, header: &[u8]) -> Option<usize> {
        if header.len() < self.header_len {
            return None;
        }

        let offset = self.length_field_offset;
        let mut value: u32 = 0;

        for i in 0..self.length_field_size {
            let byte = header[offset + i] as u32;
            if self.big_endian {
                value = (value << 8) | byte;
            } else {
                value |= byte << (8 * i);
            }
        }

        let mut payload_len = value as usize;
        if self.length_includes_header {
            payload_len = payload_len.saturating_sub(self.header_len);
        }

        if payload_len > self.max_payload_len {
            return None;
        }

        Some(payload_len)
    }
}

/// A demuxed packet with separate header and payload.
#[derive(Debug, Clone, PartialEq)]
pub struct DemuxedPacket {
    /// The raw header bytes
    pub header: Vec<u8>,
    /// The payload bytes
    pub payload: Vec<u8>,
    /// Decoded payload length from header
    pub declared_length: usize,
}

/// State of the demultiplexer.
#[derive(Debug, Clone, PartialEq)]
enum DemuxState {
    /// Waiting for header bytes
    ReadingHeader,
    /// Reading payload of known length
    ReadingPayload { length: usize },
}

/// Header/Payload demultiplexer.
#[derive(Debug, Clone)]
pub struct HeaderPayloadDemux {
    /// Header configuration
    config: HeaderConfig,
    /// Current state
    state: DemuxState,
    /// Internal buffer
    buffer: Vec<u8>,
    /// Current header (when reading payload)
    current_header: Vec<u8>,
    /// Total packets decoded
    packets_decoded: usize,
    /// Total packets with invalid headers
    invalid_headers: usize,
}

impl HeaderPayloadDemux {
    /// Create a new header/payload demultiplexer.
    pub fn new(config: HeaderConfig) -> Self {
        Self {
            config,
            state: DemuxState::ReadingHeader,
            buffer: Vec::new(),
            current_header: Vec::new(),
            packets_decoded: 0,
            invalid_headers: 0,
        }
    }

    /// Process input bytes and extract complete packets.
    pub fn process(&mut self, input: &[u8]) -> Vec<DemuxedPacket> {
        self.buffer.extend_from_slice(input);
        let mut packets = Vec::new();

        loop {
            match self.state {
                DemuxState::ReadingHeader => {
                    if self.buffer.len() < self.config.header_len {
                        break; // Need more data
                    }

                    let header: Vec<u8> =
                        self.buffer.drain(..self.config.header_len).collect();

                    if let Some(payload_len) = self.config.parse_length(&header) {
                        self.current_header = header;
                        self.state = DemuxState::ReadingPayload {
                            length: payload_len,
                        };
                    } else {
                        // Invalid header — skip one byte and try again
                        self.invalid_headers += 1;
                        // Put back all but first byte
                        let mut rest = header;
                        rest.remove(0);
                        let mut new_buf = rest;
                        new_buf.extend(self.buffer.drain(..));
                        self.buffer = new_buf;
                    }
                }
                DemuxState::ReadingPayload { length } => {
                    if self.buffer.len() < length {
                        break; // Need more data
                    }

                    let payload: Vec<u8> = self.buffer.drain(..length).collect();
                    packets.push(DemuxedPacket {
                        header: self.current_header.clone(),
                        payload,
                        declared_length: length,
                    });
                    self.packets_decoded += 1;
                    self.current_header.clear();
                    self.state = DemuxState::ReadingHeader;
                }
            }
        }

        packets
    }

    /// Get number of successfully decoded packets.
    pub fn packets_decoded(&self) -> usize {
        self.packets_decoded
    }

    /// Get number of invalid headers encountered.
    pub fn invalid_headers(&self) -> usize {
        self.invalid_headers
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.state = DemuxState::ReadingHeader;
        self.buffer.clear();
        self.current_header.clear();
        self.packets_decoded = 0;
        self.invalid_headers = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_packet() {
        let config = HeaderConfig::simple(4);
        let mut demux = HeaderPayloadDemux::new(config);

        // 4-byte header with length=3 (LE: 03 00), then 3-byte payload
        let data = vec![3, 0, 0, 0, 0xAA, 0xBB, 0xCC];
        let packets = demux.process(&data);

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].header, vec![3, 0, 0, 0]);
        assert_eq!(packets[0].payload, vec![0xAA, 0xBB, 0xCC]);
        assert_eq!(packets[0].declared_length, 3);
    }

    #[test]
    fn test_multiple_packets() {
        let config = HeaderConfig::simple(2);
        let mut demux = HeaderPayloadDemux::new(config);

        let mut data = Vec::new();
        // Packet 1: length=2, payload=[0x11, 0x22]
        data.extend_from_slice(&[2, 0, 0x11, 0x22]);
        // Packet 2: length=1, payload=[0x33]
        data.extend_from_slice(&[1, 0, 0x33]);

        let packets = demux.process(&data);
        assert_eq!(packets.len(), 2);
        assert_eq!(packets[0].payload, vec![0x11, 0x22]);
        assert_eq!(packets[1].payload, vec![0x33]);
    }

    #[test]
    fn test_incremental_feeding() {
        let config = HeaderConfig::simple(4);
        let mut demux = HeaderPayloadDemux::new(config);

        // Feed header in two pieces
        assert!(demux.process(&[5, 0]).is_empty());
        assert!(demux.process(&[0, 0]).is_empty());
        // Feed payload in two pieces
        assert!(demux.process(&[1, 2, 3]).is_empty());
        let packets = demux.process(&[4, 5]);

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].payload, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_big_endian() {
        let config = HeaderConfig::custom(2, 0, 2, true);
        let mut demux = HeaderPayloadDemux::new(config);

        // Length = 0x0003 (big-endian)
        let data = vec![0x00, 0x03, 0xAA, 0xBB, 0xCC];
        let packets = demux.process(&data);

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].payload, vec![0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn test_length_includes_header() {
        let mut config = HeaderConfig::simple(4);
        config.length_includes_header = true;

        let mut demux = HeaderPayloadDemux::new(config);

        // Total length = 7, header = 4 → payload = 3
        let data = vec![7, 0, 0, 0, 0x11, 0x22, 0x33];
        let packets = demux.process(&data);

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].payload, vec![0x11, 0x22, 0x33]);
    }

    #[test]
    fn test_zero_length_payload() {
        let config = HeaderConfig::simple(4);
        let mut demux = HeaderPayloadDemux::new(config);

        // Length = 0
        let data = vec![0, 0, 0, 0];
        let packets = demux.process(&data);

        assert_eq!(packets.len(), 1);
        assert!(packets[0].payload.is_empty());
    }

    #[test]
    fn test_max_payload_exceeded() {
        let mut config = HeaderConfig::simple(4);
        config.max_payload_len = 10;

        let mut demux = HeaderPayloadDemux::new(config);

        // Length = 1000 (exceeds max of 10)
        let data = vec![0xE8, 0x03, 0xFF, 0xFF]; // 1000 in LE, no trailing data
        let packets = demux.process(&data);

        // Header is invalid (length > max), skips byte-by-byte but not enough data for another packet
        assert!(packets.is_empty());
        assert!(demux.invalid_headers() > 0);
    }

    #[test]
    fn test_one_byte_length_field() {
        let config = HeaderConfig::custom(1, 0, 1, false);
        let mut demux = HeaderPayloadDemux::new(config);

        let data = vec![3, 0xAA, 0xBB, 0xCC];
        let packets = demux.process(&data);

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].payload, vec![0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn test_reset() {
        let config = HeaderConfig::simple(4);
        let mut demux = HeaderPayloadDemux::new(config);

        demux.process(&[3, 0, 0, 0, 1, 2, 3]);
        assert_eq!(demux.packets_decoded(), 1);

        demux.reset();
        assert_eq!(demux.packets_decoded(), 0);
    }

    #[test]
    fn test_offset_length_field() {
        // Header: [type, flags, len_lo, len_hi] — length at offset 2
        let config = HeaderConfig::custom(4, 2, 2, false);
        let mut demux = HeaderPayloadDemux::new(config);

        let data = vec![0x01, 0x00, 0x02, 0x00, 0xAA, 0xBB];
        let packets = demux.process(&data);

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].payload, vec![0xAA, 0xBB]);
    }
}

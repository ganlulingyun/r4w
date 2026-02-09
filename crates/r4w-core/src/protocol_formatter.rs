//! Protocol Formatter — Pluggable Packet Header Generation and Parsing
//!
//! Configurable packet header generator and parser using trait-based
//! format objects. Supports access code, length fields, CRC headers,
//! and sequence counters. The `HeaderFormat` trait enables custom
//! framing strategies for any packet radio system.
//! GNU Radio equivalent: `gr::digital::protocol_formatter_bb`,
//! `gr::digital::protocol_parser_b`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::protocol_formatter::{DefaultHeaderFormat, HeaderFormat, HeaderContext};
//!
//! let fmt = DefaultHeaderFormat::new(0xACDD_A4E2u64);
//! let ctx = HeaderContext::new();
//! let header = fmt.format(100, &ctx);
//! assert!(!header.is_empty());
//!
//! let parsed = fmt.parse(&header);
//! assert!(parsed.is_some());
//! assert_eq!(parsed.unwrap().payload_len, 100);
//! ```

/// Header format trait — implement for custom packet framing.
pub trait HeaderFormat: Send + Sync {
    /// Generate header bytes for a given payload length.
    fn format(&self, payload_len: usize, context: &HeaderContext) -> Vec<u8>;

    /// Parse header bytes and extract info.
    fn parse(&self, header: &[u8]) -> Option<HeaderInfo>;

    /// Get header length in bytes.
    fn header_len(&self) -> usize;
}

/// Context provided to formatter (counters, metadata).
#[derive(Debug, Clone, Default)]
pub struct HeaderContext {
    /// Packet sequence number.
    pub seq_num: u32,
    /// Bits per symbol for current modulation.
    pub bps: u8,
    /// Extra metadata.
    pub metadata: Vec<(String, String)>,
}

impl HeaderContext {
    /// Create default context.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with sequence number.
    pub fn with_seq(seq_num: u32) -> Self {
        Self {
            seq_num,
            ..Default::default()
        }
    }
}

/// Parsed header information.
#[derive(Debug, Clone)]
pub struct HeaderInfo {
    /// Payload length in bytes.
    pub payload_len: usize,
    /// CRC valid flag.
    pub crc_valid: bool,
    /// Sequence number (if present).
    pub seq_num: Option<u32>,
    /// Bits per symbol (if present).
    pub bps: Option<u8>,
}

/// Default header format: access code + 12-bit length (repeated) + 16-bit CRC.
///
/// Header layout (bytes):
/// [access_code (8)] [len_hi (1)] [len_lo_a | len_hi_b (1)] [len_lo_b (1)] [crc16 (2)]
/// Total: 13 bytes
#[derive(Debug, Clone)]
pub struct DefaultHeaderFormat {
    access_code: u64,
}

impl DefaultHeaderFormat {
    /// Create with a given access code.
    pub fn new(access_code: u64) -> Self {
        Self { access_code }
    }

    /// Get the access code.
    pub fn access_code(&self) -> u64 {
        self.access_code
    }
}

impl HeaderFormat for DefaultHeaderFormat {
    fn format(&self, payload_len: usize, _context: &HeaderContext) -> Vec<u8> {
        let len12 = (payload_len & 0xFFF) as u16;
        let mut header = Vec::with_capacity(13);

        // Access code (8 bytes, big-endian)
        header.extend_from_slice(&self.access_code.to_be_bytes());

        // Length field: 12 bits repeated twice = 3 bytes
        let len_bytes = [
            ((len12 >> 4) & 0xFF) as u8,
            (((len12 & 0x0F) << 4) | ((len12 >> 8) & 0x0F)) as u8,
            (len12 & 0xFF) as u8,
        ];
        header.extend_from_slice(&len_bytes);

        // CRC-16 over length bytes
        let crc = crc16(&len_bytes);
        header.extend_from_slice(&crc.to_be_bytes());

        header
    }

    fn parse(&self, header: &[u8]) -> Option<HeaderInfo> {
        if header.len() < 13 {
            return None;
        }

        // Verify access code
        let ac = u64::from_be_bytes(header[0..8].try_into().ok()?);
        if ac != self.access_code {
            return None;
        }

        // Extract length fields
        let len_a = ((header[8] as u16) << 4) | ((header[9] >> 4) as u16);
        let len_b = (((header[9] & 0x0F) as u16) << 8) | (header[10] as u16);

        // Both copies should match
        if len_a != len_b {
            return None;
        }

        // Verify CRC
        let crc_received = u16::from_be_bytes([header[11], header[12]]);
        let crc_computed = crc16(&header[8..11]);
        let crc_valid = crc_received == crc_computed;

        Some(HeaderInfo {
            payload_len: len_a as usize,
            crc_valid,
            seq_num: None,
            bps: None,
        })
    }

    fn header_len(&self) -> usize {
        13
    }
}

/// Counter header format: adds sequence number and BPS.
///
/// Header layout:
/// [access_code (8)] [seq_num (2)] [bps (1)] [payload_len (2)] [crc16 (2)]
/// Total: 15 bytes
#[derive(Debug, Clone)]
pub struct CounterHeaderFormat {
    access_code: u64,
}

impl CounterHeaderFormat {
    /// Create with access code.
    pub fn new(access_code: u64) -> Self {
        Self { access_code }
    }
}

impl HeaderFormat for CounterHeaderFormat {
    fn format(&self, payload_len: usize, context: &HeaderContext) -> Vec<u8> {
        let mut header = Vec::with_capacity(15);

        // Access code
        header.extend_from_slice(&self.access_code.to_be_bytes());

        // Sequence number (16-bit)
        header.extend_from_slice(&(context.seq_num as u16).to_be_bytes());

        // BPS
        header.push(context.bps);

        // Payload length (16-bit)
        header.extend_from_slice(&(payload_len as u16).to_be_bytes());

        // CRC-16 over seq + bps + len
        let crc = crc16(&header[8..]);
        header.extend_from_slice(&crc.to_be_bytes());

        header
    }

    fn parse(&self, header: &[u8]) -> Option<HeaderInfo> {
        if header.len() < 15 {
            return None;
        }

        let ac = u64::from_be_bytes(header[0..8].try_into().ok()?);
        if ac != self.access_code {
            return None;
        }

        let seq = u16::from_be_bytes([header[8], header[9]]);
        let bps = header[10];
        let payload_len = u16::from_be_bytes([header[11], header[12]]);

        let crc_received = u16::from_be_bytes([header[13], header[14]]);
        let crc_computed = crc16(&header[8..13]);

        Some(HeaderInfo {
            payload_len: payload_len as usize,
            crc_valid: crc_received == crc_computed,
            seq_num: Some(seq as u32),
            bps: Some(bps),
        })
    }

    fn header_len(&self) -> usize {
        15
    }
}

/// Protocol formatter block — prepends header to payload.
pub struct ProtocolFormatter {
    format: Box<dyn HeaderFormat>,
    seq_num: u32,
}

impl ProtocolFormatter {
    /// Create with a header format.
    pub fn new(format: Box<dyn HeaderFormat>) -> Self {
        Self {
            format,
            seq_num: 0,
        }
    }

    /// Format a payload: returns header + payload.
    pub fn format_payload(&mut self, payload: &[u8]) -> Vec<u8> {
        let ctx = HeaderContext {
            seq_num: self.seq_num,
            bps: 1,
            metadata: Vec::new(),
        };
        self.seq_num = self.seq_num.wrapping_add(1);

        let mut packet = self.format.format(payload.len(), &ctx);
        packet.extend_from_slice(payload);
        packet
    }

    /// Get current sequence number.
    pub fn seq_num(&self) -> u32 {
        self.seq_num
    }

    /// Reset sequence counter.
    pub fn reset(&mut self) {
        self.seq_num = 0;
    }
}

/// Protocol parser block — extracts header info from received data.
pub struct ProtocolParser {
    format: Box<dyn HeaderFormat>,
}

impl ProtocolParser {
    /// Create with a header format.
    pub fn new(format: Box<dyn HeaderFormat>) -> Self {
        Self { format }
    }

    /// Parse header from received data.
    pub fn parse<'a>(&self, data: &'a [u8]) -> Option<(HeaderInfo, &'a [u8])> {
        let info = self.format.parse(data)?;
        let header_len = self.format.header_len();
        let payload_end = header_len + info.payload_len;
        if data.len() >= payload_end {
            Some((info, &data[header_len..payload_end]))
        } else {
            Some((info, &data[header_len..]))
        }
    }

    /// Get expected header length.
    pub fn header_len(&self) -> usize {
        self.format.header_len()
    }
}

/// Simple CRC-16 (CCITT).
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    const ACCESS_CODE: u64 = 0xACDD_A4E2_0000_0000;

    #[test]
    fn test_default_format_roundtrip() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let ctx = HeaderContext::new();
        let header = fmt.format(100, &ctx);
        assert_eq!(header.len(), 13);

        let parsed = fmt.parse(&header).unwrap();
        assert_eq!(parsed.payload_len, 100);
        assert!(parsed.crc_valid);
    }

    #[test]
    fn test_counter_format_roundtrip() {
        let fmt = CounterHeaderFormat::new(ACCESS_CODE);
        let ctx = HeaderContext {
            seq_num: 42,
            bps: 2,
            metadata: Vec::new(),
        };
        let header = fmt.format(256, &ctx);
        assert_eq!(header.len(), 15);

        let parsed = fmt.parse(&header).unwrap();
        assert_eq!(parsed.payload_len, 256);
        assert_eq!(parsed.seq_num, Some(42));
        assert_eq!(parsed.bps, Some(2));
        assert!(parsed.crc_valid);
    }

    #[test]
    fn test_crc_detects_errors() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let ctx = HeaderContext::new();
        let mut header = fmt.format(50, &ctx);

        // Corrupt a length byte
        header[9] ^= 0x01;
        let parsed = fmt.parse(&header);
        // Should either fail to parse or have invalid CRC
        if let Some(info) = parsed {
            assert!(!info.crc_valid || info.payload_len != 50);
        }
    }

    #[test]
    fn test_length_field_repeated() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let ctx = HeaderContext::new();
        let header = fmt.format(0xABC, &ctx);
        let parsed = fmt.parse(&header).unwrap();
        assert_eq!(parsed.payload_len, 0xABC);
    }

    #[test]
    fn test_formatter_block() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let mut formatter = ProtocolFormatter::new(Box::new(fmt));

        let payload = b"Hello, World!";
        let packet = formatter.format_payload(payload);
        assert_eq!(packet.len(), 13 + payload.len());
        assert_eq!(formatter.seq_num(), 1);
    }

    #[test]
    fn test_parser_block() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let mut formatter = ProtocolFormatter::new(Box::new(fmt));

        let payload = b"Test data";
        let packet = formatter.format_payload(payload);

        let parser_fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let parser = ProtocolParser::new(Box::new(parser_fmt));
        let (info, extracted) = parser.parse(&packet).unwrap();
        assert_eq!(info.payload_len, payload.len());
        assert_eq!(extracted, payload);
    }

    #[test]
    fn test_wrong_access_code() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let ctx = HeaderContext::new();
        let header = fmt.format(10, &ctx);

        let wrong_fmt = DefaultHeaderFormat::new(0x1234_5678_9ABC_DEF0);
        assert!(wrong_fmt.parse(&header).is_none());
    }

    #[test]
    fn test_truncated_header() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let short = vec![0u8; 5];
        assert!(fmt.parse(&short).is_none());
    }

    #[test]
    fn test_zero_length_payload() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let ctx = HeaderContext::new();
        let header = fmt.format(0, &ctx);
        let parsed = fmt.parse(&header).unwrap();
        assert_eq!(parsed.payload_len, 0);
        assert!(parsed.crc_valid);
    }

    #[test]
    fn test_sequence_incrementing() {
        let fmt = CounterHeaderFormat::new(ACCESS_CODE);
        let mut formatter = ProtocolFormatter::new(Box::new(fmt));

        for i in 0..5 {
            let _ = formatter.format_payload(&[0u8; 10]);
            assert_eq!(formatter.seq_num(), i + 1);
        }
    }

    #[test]
    fn test_formatter_reset() {
        let fmt = DefaultHeaderFormat::new(ACCESS_CODE);
        let mut formatter = ProtocolFormatter::new(Box::new(fmt));
        let _ = formatter.format_payload(&[0u8; 10]);
        formatter.reset();
        assert_eq!(formatter.seq_num(), 0);
    }
}

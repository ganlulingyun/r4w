//! Protocol-Level Header Extraction and Validation
//!
//! Parses structured packet headers from bitstreams. Supports arbitrary field
//! layouts with unsigned, signed, boolean, enum, and reserved field types.
//! Optional CRC validation protects header integrity.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::packet_header_parser::{HeaderFormat, HeaderParser, FieldType};
//!
//! let mut fmt = HeaderFormat::new("SimpleHeader");
//! fmt.add_field("version", 4, FieldType::Unsigned)
//!    .add_field("type", 4, FieldType::Unsigned)
//!    .add_field("length", 8, FieldType::Unsigned);
//!
//! let parser = HeaderParser::new(fmt);
//! // 16 bits: version=0b0011, type=0b0101, length=0b11111111
//! let bits: Vec<bool> = [
//!     false, false, true, true,   // version = 3
//!     false, true, false, true,   // type = 5
//!     true, true, true, true, true, true, true, true, // length = 255
//! ].to_vec();
//!
//! let header = parser.parse(&bits).unwrap();
//! assert_eq!(header.get_unsigned("version"), Some(3));
//! assert_eq!(header.get_unsigned("type"), Some(5));
//! assert_eq!(header.get_unsigned("length"), Some(255));
//! assert!(header.is_valid());
//! ```

use std::collections::HashMap;

// ============================================================================
// Field Types
// ============================================================================

/// The type of a header field, controlling how bits are interpreted.
#[derive(Debug, Clone, PartialEq)]
pub enum FieldType {
    /// Unsigned integer (MSB first).
    Unsigned,
    /// Signed integer (two's complement, MSB first).
    Signed,
    /// Single-bit boolean (true if any bit is set for multi-bit fields).
    Boolean,
    /// Enumerated value: maps integer values to string labels.
    Enum(Vec<(u64, String)>),
    /// Reserved field (skipped during parsing, not stored).
    Reserved,
}

// ============================================================================
// Header Field
// ============================================================================

/// Describes a single field within a header layout.
#[derive(Debug, Clone)]
pub struct HeaderField {
    /// Human-readable field name.
    pub name: String,
    /// Bit offset from the start of the header.
    pub bit_offset: usize,
    /// Width of this field in bits.
    pub bit_length: usize,
    /// How to interpret the bits.
    pub field_type: FieldType,
}

// ============================================================================
// CRC Field Descriptor
// ============================================================================

/// Describes a CRC field appended to the header for integrity checking.
#[derive(Debug, Clone)]
struct CrcField {
    /// Field name (used in ParsedHeader).
    name: String,
    /// Width of the CRC in bits (8, 16, or 32).
    bit_length: usize,
    /// Generator polynomial.
    polynomial: u32,
    /// Bit offset where the CRC field begins.
    bit_offset: usize,
}

// ============================================================================
// Header Format
// ============================================================================

/// Defines the complete layout of a packet header as a sequence of typed fields.
#[derive(Debug, Clone)]
pub struct HeaderFormat {
    /// Name of this header format (e.g., "IEEE 802.15.4").
    pub name: String,
    /// Ordered list of fields.
    pub fields: Vec<HeaderField>,
    /// Optional CRC field for validation.
    crc: Option<CrcField>,
    /// Running bit offset for the next field to be added.
    next_offset: usize,
}

impl HeaderFormat {
    /// Create a new, empty header format with the given name.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            fields: Vec::new(),
            crc: None,
            next_offset: 0,
        }
    }

    /// Add a field to the header layout. Fields are appended in order; the bit
    /// offset is computed automatically from previously added fields.
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn add_field(&mut self, name: &str, bit_length: usize, field_type: FieldType) -> &mut Self {
        self.fields.push(HeaderField {
            name: name.to_string(),
            bit_offset: self.next_offset,
            bit_length,
            field_type,
        });
        self.next_offset += bit_length;
        self
    }

    /// Add a CRC field to the header. The CRC is computed over all preceding
    /// bits (before the CRC field itself). `polynomial` is used with a simple
    /// bit-level CRC algorithm.
    pub fn add_crc_field(&mut self, name: &str, bit_length: usize, polynomial: u32) {
        self.crc = Some(CrcField {
            name: name.to_string(),
            bit_length,
            polynomial,
            bit_offset: self.next_offset,
        });
        self.next_offset += bit_length;
    }

    /// Total number of bits in the header (including CRC if present).
    pub fn total_bits(&self) -> usize {
        self.next_offset
    }

    /// Number of non-CRC fields in the header.
    pub fn field_count(&self) -> usize {
        self.fields.len()
    }
}

// ============================================================================
// Parsed Value
// ============================================================================

/// A value extracted from a parsed header field.
#[derive(Debug, Clone, PartialEq)]
pub enum ParsedValue {
    /// Unsigned integer value.
    Unsigned(u64),
    /// Signed integer value (two's complement).
    Signed(i64),
    /// Boolean value.
    Bool(bool),
    /// Enumerated value with both the raw integer and its label.
    Enum(u64, String),
}

// ============================================================================
// Parsed Header
// ============================================================================

/// The result of parsing a bitstream against a [`HeaderFormat`].
#[derive(Debug, Clone)]
pub struct ParsedHeader {
    /// Map of field name to parsed value.
    pub values: HashMap<String, ParsedValue>,
    /// Whether the CRC (if present) matched.
    pub crc_valid: bool,
}

impl ParsedHeader {
    /// Retrieve an unsigned integer field by name.
    pub fn get_unsigned(&self, field: &str) -> Option<u64> {
        match self.values.get(field)? {
            ParsedValue::Unsigned(v) => Some(*v),
            ParsedValue::Enum(v, _) => Some(*v),
            _ => None,
        }
    }

    /// Retrieve a signed integer field by name.
    pub fn get_signed(&self, field: &str) -> Option<i64> {
        match self.values.get(field)? {
            ParsedValue::Signed(v) => Some(*v),
            _ => None,
        }
    }

    /// Retrieve a boolean field by name.
    pub fn get_bool(&self, field: &str) -> Option<bool> {
        match self.values.get(field)? {
            ParsedValue::Bool(v) => Some(*v),
            _ => None,
        }
    }

    /// Returns true if the header is valid (CRC matched or no CRC was defined).
    pub fn is_valid(&self) -> bool {
        self.crc_valid
    }
}

// ============================================================================
// Parse Error
// ============================================================================

/// Errors that can occur during header parsing.
#[derive(Debug, Clone, PartialEq)]
pub enum ParseError {
    /// The input bitstream is shorter than the header format requires.
    TooShort {
        /// Number of bits required.
        expected: usize,
        /// Number of bits provided.
        actual: usize,
    },
    /// A field could not be parsed (e.g., enum value not in the lookup table).
    InvalidField(String),
    /// The CRC computed over the header data does not match the CRC field.
    CrcMismatch {
        /// Expected CRC value (from the header).
        expected: u64,
        /// Computed CRC value.
        computed: u64,
    },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::TooShort { expected, actual } => {
                write!(f, "input too short: need {} bits, got {}", expected, actual)
            }
            ParseError::InvalidField(msg) => write!(f, "invalid field: {}", msg),
            ParseError::CrcMismatch { expected, computed } => {
                write!(
                    f,
                    "CRC mismatch: expected 0x{:X}, computed 0x{:X}",
                    expected, computed
                )
            }
        }
    }
}

impl std::error::Error for ParseError {}

// ============================================================================
// Header Parser
// ============================================================================

/// Parses bitstreams according to a [`HeaderFormat`] definition.
pub struct HeaderParser {
    format: HeaderFormat,
}

impl HeaderParser {
    /// Create a new parser for the given header format.
    pub fn new(format: HeaderFormat) -> Self {
        Self { format }
    }

    /// Parse a header from a slice of boolean bits (MSB first per field).
    pub fn parse(&self, bits: &[bool]) -> Result<ParsedHeader, ParseError> {
        let required = self.format.total_bits();
        if bits.len() < required {
            return Err(ParseError::TooShort {
                expected: required,
                actual: bits.len(),
            });
        }

        let mut values = HashMap::new();

        for field in &self.format.fields {
            let field_bits = &bits[field.bit_offset..field.bit_offset + field.bit_length];
            let value = Self::parse_field(field, field_bits)?;
            if let Some(v) = value {
                values.insert(field.name.clone(), v);
            }
        }

        // CRC validation
        let crc_valid = if let Some(ref crc_field) = self.format.crc {
            let data_bits = &bits[..crc_field.bit_offset];
            let crc_bits = &bits[crc_field.bit_offset..crc_field.bit_offset + crc_field.bit_length];

            let expected_crc = bits_to_u64(crc_bits);
            let computed_crc = compute_bit_crc(data_bits, crc_field.bit_length, crc_field.polynomial);

            if expected_crc != computed_crc {
                return Err(ParseError::CrcMismatch {
                    expected: expected_crc,
                    computed: computed_crc,
                });
            }
            true
        } else {
            true
        };

        Ok(ParsedHeader { values, crc_valid })
    }

    /// Parse a header from a byte slice. Bytes are expanded to bits (MSB first).
    pub fn parse_bytes(&self, bytes: &[u8]) -> Result<ParsedHeader, ParseError> {
        let bits = bytes_to_bits(bytes);
        self.parse(&bits)
    }

    /// Parse a single field from its bit slice.
    fn parse_field(field: &HeaderField, bits: &[bool]) -> Result<Option<ParsedValue>, ParseError> {
        match &field.field_type {
            FieldType::Reserved => Ok(None),
            FieldType::Unsigned => {
                let val = bits_to_u64(bits);
                Ok(Some(ParsedValue::Unsigned(val)))
            }
            FieldType::Signed => {
                let val = bits_to_i64(bits);
                Ok(Some(ParsedValue::Signed(val)))
            }
            FieldType::Boolean => {
                let val = bits.iter().any(|&b| b);
                Ok(Some(ParsedValue::Bool(val)))
            }
            FieldType::Enum(mapping) => {
                let raw = bits_to_u64(bits);
                if let Some((_, label)) = mapping.iter().find(|(k, _)| *k == raw) {
                    Ok(Some(ParsedValue::Enum(raw, label.clone())))
                } else {
                    Err(ParseError::InvalidField(format!(
                        "field '{}': unknown enum value {}",
                        field.name, raw
                    )))
                }
            }
        }
    }
}

// ============================================================================
// Bit Utilities
// ============================================================================

/// Convert a bool slice (MSB first) to an unsigned 64-bit integer.
fn bits_to_u64(bits: &[bool]) -> u64 {
    let mut val: u64 = 0;
    for &b in bits {
        val = (val << 1) | (b as u64);
    }
    val
}

/// Convert a bool slice (MSB first) to a signed 64-bit integer (two's complement).
fn bits_to_i64(bits: &[bool]) -> i64 {
    if bits.is_empty() {
        return 0;
    }
    let raw = bits_to_u64(bits);
    let width = bits.len();
    if width >= 64 {
        return raw as i64;
    }
    // Sign-extend: if the MSB is set, fill upper bits with 1s
    if bits[0] {
        let mask = !((1u64 << width) - 1);
        (raw | mask) as i64
    } else {
        raw as i64
    }
}

/// Expand a byte slice to a vector of bools (MSB first per byte).
fn bytes_to_bits(bytes: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(bytes.len() * 8);
    for &byte in bytes {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1 != 0);
        }
    }
    bits
}

/// Simple bit-level CRC computation. Processes each bit of `data` through a
/// shift-register with the given `polynomial`. Returns the CRC truncated to
/// `crc_bits` width.
fn compute_bit_crc(data: &[bool], crc_bits: usize, polynomial: u32) -> u64 {
    let poly = polynomial as u64;
    let top_bit = 1u64 << (crc_bits - 1);
    let mask = (1u64 << crc_bits) - 1;

    let mut crc: u64 = 0;
    for &bit in data {
        let xor_flag = (crc & top_bit) != 0;
        crc = (crc << 1) & mask;
        if bit {
            crc |= 1;
        }
        if xor_flag {
            crc ^= poly & mask;
        }
    }

    // Process crc_bits zero bits to flush the remainder
    for _ in 0..crc_bits {
        let xor_flag = (crc & top_bit) != 0;
        crc = (crc << 1) & mask;
        if xor_flag {
            crc ^= poly & mask;
        }
    }

    crc
}

/// Convert a u64 value to a vector of bools (MSB first) with the given bit width.
fn u64_to_bits(value: u64, width: usize) -> Vec<bool> {
    let mut bits = Vec::with_capacity(width);
    for i in (0..width).rev() {
        bits.push((value >> i) & 1 != 0);
    }
    bits
}

// ============================================================================
// Factory Functions
// ============================================================================

/// Create an IEEE 802.15.4 MAC header format.
///
/// Layout (simplified):
/// - Frame Control (16 bits): frame type (3), security enabled (1), frame pending (1),
///   ack request (1), PAN ID compression (1), reserved (3), dest addr mode (2),
///   frame version (2), src addr mode (2)
/// - Sequence Number (8 bits)
/// - Destination PAN ID (16 bits)
/// - Destination Address (16 bits, short addressing)
pub fn ieee_802_15_4_header() -> HeaderFormat {
    let mut fmt = HeaderFormat::new("IEEE 802.15.4 MAC");

    // Frame Control subfields (16 bits total)
    fmt.add_field(
        "frame_type",
        3,
        FieldType::Enum(vec![
            (0, "Beacon".to_string()),
            (1, "Data".to_string()),
            (2, "Ack".to_string()),
            (3, "MAC Command".to_string()),
        ]),
    );
    fmt.add_field("security_enabled", 1, FieldType::Boolean);
    fmt.add_field("frame_pending", 1, FieldType::Boolean);
    fmt.add_field("ack_request", 1, FieldType::Boolean);
    fmt.add_field("pan_id_compression", 1, FieldType::Boolean);
    fmt.add_field("reserved_fc", 3, FieldType::Reserved);
    fmt.add_field("dest_addr_mode", 2, FieldType::Unsigned);
    fmt.add_field("frame_version", 2, FieldType::Unsigned);
    fmt.add_field("src_addr_mode", 2, FieldType::Unsigned);

    // Sequence Number
    fmt.add_field("sequence_number", 8, FieldType::Unsigned);

    // Destination PAN ID (16 bits)
    fmt.add_field("dest_pan_id", 16, FieldType::Unsigned);

    // Destination Address (16-bit short)
    fmt.add_field("dest_address", 16, FieldType::Unsigned);

    fmt
}

/// Create a simple packet header format suitable for basic radio protocols.
///
/// Layout:
/// - Version (4 bits)
/// - Type (4 bits)
/// - Length (16 bits)
/// - CRC-16 (16 bits, polynomial 0x8005)
pub fn simple_packet_header() -> HeaderFormat {
    let mut fmt = HeaderFormat::new("Simple Packet");
    fmt.add_field("version", 4, FieldType::Unsigned);
    fmt.add_field("type", 4, FieldType::Unsigned);
    fmt.add_field("length", 16, FieldType::Unsigned);
    fmt.add_crc_field("crc", 16, 0x8005);
    fmt
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_header_format_construction_and_fields() {
        let mut fmt = HeaderFormat::new("Test");
        fmt.add_field("a", 4, FieldType::Unsigned)
            .add_field("b", 8, FieldType::Unsigned)
            .add_field("c", 1, FieldType::Boolean);

        assert_eq!(fmt.field_count(), 3);
        assert_eq!(fmt.total_bits(), 13);
        assert_eq!(fmt.fields[0].name, "a");
        assert_eq!(fmt.fields[0].bit_offset, 0);
        assert_eq!(fmt.fields[0].bit_length, 4);
        assert_eq!(fmt.fields[1].bit_offset, 4);
        assert_eq!(fmt.fields[1].bit_length, 8);
        assert_eq!(fmt.fields[2].bit_offset, 12);
        assert_eq!(fmt.fields[2].bit_length, 1);
    }

    #[test]
    fn test_parse_unsigned_field() {
        let mut fmt = HeaderFormat::new("U8");
        fmt.add_field("value", 8, FieldType::Unsigned);
        let parser = HeaderParser::new(fmt);

        // 0b10110011 = 179
        let bits = vec![true, false, true, true, false, false, true, true];
        let hdr = parser.parse(&bits).unwrap();
        assert_eq!(hdr.get_unsigned("value"), Some(179));
        assert!(hdr.is_valid());
    }

    #[test]
    fn test_parse_signed_field() {
        let mut fmt = HeaderFormat::new("Signed");
        fmt.add_field("temperature", 8, FieldType::Signed);
        let parser = HeaderParser::new(fmt);

        // -5 in 8-bit two's complement = 0b11111011
        let bits = vec![true, true, true, true, true, false, true, true];
        let hdr = parser.parse(&bits).unwrap();
        assert_eq!(hdr.get_signed("temperature"), Some(-5));

        // +42 = 0b00101010
        let bits2 = vec![false, false, true, false, true, false, true, false];
        let hdr2 = parser.parse(&bits2).unwrap();
        assert_eq!(hdr2.get_signed("temperature"), Some(42));
    }

    #[test]
    fn test_parse_boolean_field() {
        let mut fmt = HeaderFormat::new("Bool");
        fmt.add_field("flag_a", 1, FieldType::Boolean)
            .add_field("flag_b", 1, FieldType::Boolean);
        let parser = HeaderParser::new(fmt);

        let bits = vec![true, false];
        let hdr = parser.parse(&bits).unwrap();
        assert_eq!(hdr.get_bool("flag_a"), Some(true));
        assert_eq!(hdr.get_bool("flag_b"), Some(false));
    }

    #[test]
    fn test_parse_from_bytes() {
        let mut fmt = HeaderFormat::new("ByteTest");
        fmt.add_field("byte0", 8, FieldType::Unsigned)
            .add_field("byte1", 8, FieldType::Unsigned);
        let parser = HeaderParser::new(fmt);

        let hdr = parser.parse_bytes(&[0xAB, 0xCD]).unwrap();
        assert_eq!(hdr.get_unsigned("byte0"), Some(0xAB));
        assert_eq!(hdr.get_unsigned("byte1"), Some(0xCD));
    }

    #[test]
    fn test_too_short_input_error() {
        let mut fmt = HeaderFormat::new("Long");
        fmt.add_field("data", 16, FieldType::Unsigned);
        let parser = HeaderParser::new(fmt);

        let bits = vec![true, false, true]; // only 3 bits, need 16
        let err = parser.parse(&bits).unwrap_err();
        assert_eq!(
            err,
            ParseError::TooShort {
                expected: 16,
                actual: 3,
            }
        );
    }

    #[test]
    fn test_crc_validation() {
        let mut fmt = HeaderFormat::new("CrcTest");
        fmt.add_field("payload", 8, FieldType::Unsigned);
        fmt.add_crc_field("crc", 8, 0x07); // CRC-8 polynomial

        // Compute what the CRC should be for payload = 0xA5 (0b10100101)
        let payload_bits: Vec<bool> = vec![true, false, true, false, false, true, false, true];
        let crc_val = compute_bit_crc(&payload_bits, 8, 0x07);
        let crc_bits = u64_to_bits(crc_val, 8);

        let mut all_bits = payload_bits.clone();
        all_bits.extend_from_slice(&crc_bits);

        let parser = HeaderParser::new(fmt.clone());
        let hdr = parser.parse(&all_bits).unwrap();
        assert_eq!(hdr.get_unsigned("payload"), Some(0xA5));
        assert!(hdr.is_valid());

        // Corrupt one CRC bit and verify mismatch
        let mut corrupted = all_bits.clone();
        corrupted[8] = !corrupted[8]; // flip first CRC bit
        let err = parser.parse(&corrupted).unwrap_err();
        match err {
            ParseError::CrcMismatch { .. } => {} // expected
            other => panic!("expected CrcMismatch, got {:?}", other),
        }
    }

    #[test]
    fn test_ieee_802_15_4_format() {
        let fmt = ieee_802_15_4_header();
        assert_eq!(fmt.name, "IEEE 802.15.4 MAC");
        // 16 (frame control) + 8 (seq) + 16 (dest PAN) + 16 (dest addr) = 56 bits
        assert_eq!(fmt.total_bits(), 56);
        // 3 reserved bits do count as a field; total named fields = 10
        // frame_type, security_enabled, frame_pending, ack_request, pan_id_compression,
        // reserved_fc, dest_addr_mode, frame_version, src_addr_mode,
        // sequence_number, dest_pan_id, dest_address
        assert_eq!(fmt.field_count(), 12);

        // Parse a Data frame with ack request, seq=1, dest PAN=0x1234, dest=0xFFFF (broadcast)
        // Frame type = 1 (Data) = 0b001
        // security_enabled = 0, frame_pending = 0, ack_request = 1
        // pan_id_compression = 0, reserved = 000
        // dest_addr_mode = 10 (short), frame_version = 00, src_addr_mode = 00
        let mut bits = Vec::new();
        bits.extend_from_slice(&[false, false, true]);        // frame_type = 1 (Data)
        bits.push(false);                                      // security_enabled = 0
        bits.push(false);                                      // frame_pending = 0
        bits.push(true);                                       // ack_request = 1
        bits.push(false);                                      // pan_id_compression = 0
        bits.extend_from_slice(&[false, false, false]);        // reserved
        bits.extend_from_slice(&[true, false]);                // dest_addr_mode = 2
        bits.extend_from_slice(&[false, false]);               // frame_version = 0
        bits.extend_from_slice(&[false, false]);               // src_addr_mode = 0
        // Sequence number = 1 = 0b00000001
        bits.extend_from_slice(&[false, false, false, false, false, false, false, true]);
        // Dest PAN ID = 0x1234
        bits.extend(u64_to_bits(0x1234, 16));
        // Dest Address = 0xFFFF (broadcast)
        bits.extend(u64_to_bits(0xFFFF, 16));

        let parser = HeaderParser::new(fmt);
        let hdr = parser.parse(&bits).unwrap();
        assert_eq!(hdr.get_unsigned("sequence_number"), Some(1));
        assert_eq!(hdr.get_bool("ack_request"), Some(true));
        assert_eq!(hdr.get_bool("security_enabled"), Some(false));
        assert_eq!(hdr.get_unsigned("dest_pan_id"), Some(0x1234));
        assert_eq!(hdr.get_unsigned("dest_address"), Some(0xFFFF));
        assert_eq!(hdr.get_unsigned("dest_addr_mode"), Some(2));
    }

    #[test]
    fn test_simple_packet_header_format() {
        let fmt = simple_packet_header();
        assert_eq!(fmt.name, "Simple Packet");
        // version(4) + type(4) + length(16) + crc(16) = 40 bits
        assert_eq!(fmt.total_bits(), 40);
        assert_eq!(fmt.field_count(), 3); // version, type, length (CRC is separate)

        // Build a valid header: version=1, type=2, length=100
        let mut data_bits = Vec::new();
        data_bits.extend(u64_to_bits(1, 4));    // version
        data_bits.extend(u64_to_bits(2, 4));    // type
        data_bits.extend(u64_to_bits(100, 16)); // length

        let crc_val = compute_bit_crc(&data_bits, 16, 0x8005);
        let crc_bits = u64_to_bits(crc_val, 16);

        let mut all_bits = data_bits;
        all_bits.extend_from_slice(&crc_bits);

        let parser = HeaderParser::new(fmt);
        let hdr = parser.parse(&all_bits).unwrap();
        assert_eq!(hdr.get_unsigned("version"), Some(1));
        assert_eq!(hdr.get_unsigned("type"), Some(2));
        assert_eq!(hdr.get_unsigned("length"), Some(100));
        assert!(hdr.is_valid());
    }

    #[test]
    fn test_multiple_fields_round_trip() {
        let mut fmt = HeaderFormat::new("MultiField");
        fmt.add_field("unsigned_4", 4, FieldType::Unsigned)
            .add_field("signed_8", 8, FieldType::Signed)
            .add_field("flag", 1, FieldType::Boolean)
            .add_field("reserved", 3, FieldType::Reserved)
            .add_field(
                "mode",
                2,
                FieldType::Enum(vec![
                    (0, "Off".to_string()),
                    (1, "Low".to_string()),
                    (2, "High".to_string()),
                    (3, "Auto".to_string()),
                ]),
            )
            .add_field("counter", 16, FieldType::Unsigned);

        assert_eq!(fmt.total_bits(), 34);
        assert_eq!(fmt.field_count(), 6);

        // Encode: unsigned_4=0xF, signed_8=-128 (0x80), flag=true, reserved=000,
        //         mode=2 (High), counter=0x1234
        let mut bits = Vec::new();
        bits.extend(u64_to_bits(0xF, 4));           // unsigned_4 = 15
        bits.extend(u64_to_bits(0x80, 8));           // signed_8 = -128
        bits.push(true);                              // flag = true
        bits.extend_from_slice(&[false, false, false]); // reserved
        bits.extend(u64_to_bits(2, 2));              // mode = 2 (High)
        bits.extend(u64_to_bits(0x1234, 16));        // counter

        let parser = HeaderParser::new(fmt);
        let hdr = parser.parse(&bits).unwrap();

        assert_eq!(hdr.get_unsigned("unsigned_4"), Some(15));
        assert_eq!(hdr.get_signed("signed_8"), Some(-128));
        assert_eq!(hdr.get_bool("flag"), Some(true));
        assert!(hdr.values.get("reserved").is_none()); // reserved not stored
        match hdr.values.get("mode") {
            Some(ParsedValue::Enum(2, label)) => assert_eq!(label, "High"),
            other => panic!("expected Enum(2, High), got {:?}", other),
        }
        assert_eq!(hdr.get_unsigned("counter"), Some(0x1234));
        assert!(hdr.is_valid());
    }
}

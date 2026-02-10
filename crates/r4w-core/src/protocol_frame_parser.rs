//! # Protocol Frame Parser
//!
//! A configurable, template-driven frame parser for custom and standard
//! communication protocols. Define your frame structure once using
//! [`FrameDefinition`], then parse, build, and validate frames with
//! [`ProtocolFrameParser`].
//!
//! # Example
//!
//! ```
//! use r4w_core::protocol_frame_parser::*;
//!
//! // Define a simple protocol: [0xAA, 0x55] sync, 1-byte ID, 1-byte length,
//! // payload, XOR-8 checksum.
//! let def = FrameDefinition {
//!     sync_pattern: vec![0xAA, 0x55],
//!     header_fields: vec![
//!         FieldDefinition {
//!             name: "id".into(),
//!             bit_offset: 0,
//!             bit_length: 8,
//!             field_type: FieldType::U8,
//!         },
//!     ],
//!     length_field: Some(LengthField {
//!         bit_offset: 8,
//!         bit_length: 8,
//!         includes_header: false,
//!     }),
//!     checksum_type: ChecksumType::Xor8,
//! };
//!
//! let parser = ProtocolFrameParser::new(def);
//!
//! // Build a frame
//! let mut fields = std::collections::HashMap::new();
//! fields.insert("id".into(), FieldValue::Unsigned(0x07));
//! let frame_bytes = parser.build_frame(&fields, &[0xDE, 0xAD]).unwrap();
//!
//! // Parse it back
//! let parsed = parser.parse(&frame_bytes).unwrap();
//! assert_eq!(parsed.fields["id"], FieldValue::Unsigned(0x07));
//! assert_eq!(parsed.payload, vec![0xDE, 0xAD]);
//! assert!(parsed.valid_checksum);
//! ```

use std::collections::HashMap;
use std::fmt;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Supported field types for header fields.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldType {
    U8,
    U16,
    U32,
    Bool,
    /// Fixed-length byte array (length inferred from `bit_length / 8`).
    Bytes,
}

/// A parsed field value.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldValue {
    Unsigned(u64),
    Boolean(bool),
    Bytes(Vec<u8>),
}

/// Checksum algorithms supported by the parser.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChecksumType {
    None,
    Crc8,
    Crc16,
    Crc32,
    Sum8,
    Xor8,
}

/// Describes a single header field inside a frame.
#[derive(Debug, Clone)]
pub struct FieldDefinition {
    pub name: String,
    pub bit_offset: usize,
    pub bit_length: usize,
    pub field_type: FieldType,
}

/// Describes the length field that encodes the payload size.
#[derive(Debug, Clone)]
pub struct LengthField {
    /// Bit offset relative to the start of the header (after sync).
    pub bit_offset: usize,
    /// Bit width of the length field (typically 8 or 16).
    pub bit_length: usize,
    /// If `true`, the encoded length includes the header bytes; otherwise it
    /// represents only the payload length.
    pub includes_header: bool,
}

/// Complete definition of a protocol frame layout.
#[derive(Debug, Clone)]
pub struct FrameDefinition {
    /// Byte pattern that marks the start of a frame.
    pub sync_pattern: Vec<u8>,
    /// Ordered list of header fields (after the sync pattern).
    pub header_fields: Vec<FieldDefinition>,
    /// Optional length field inside the header.
    pub length_field: Option<LengthField>,
    /// Checksum type appended after the payload.
    pub checksum_type: ChecksumType,
}

/// Result of successfully parsing a single frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParsedFrame {
    /// Extracted header field values keyed by field name.
    pub fields: HashMap<String, FieldValue>,
    /// Payload bytes (may be empty).
    pub payload: Vec<u8>,
    /// Whether the checksum (if any) validated correctly.
    pub valid_checksum: bool,
}

/// Errors that can occur during parsing or frame construction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameError {
    /// The sync pattern was not found in the input.
    SyncNotFound,
    /// Not enough data to parse the header or payload.
    InsufficientData,
    /// A required field was missing when building a frame.
    MissingField(String),
    /// A field value does not fit in the declared bit width.
    FieldOverflow(String),
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FrameError::SyncNotFound => write!(f, "sync pattern not found"),
            FrameError::InsufficientData => write!(f, "insufficient data"),
            FrameError::MissingField(name) => write!(f, "missing field: {}", name),
            FrameError::FieldOverflow(name) => write!(f, "field overflow: {}", name),
        }
    }
}

impl std::error::Error for FrameError {}

// ---------------------------------------------------------------------------
// ProtocolFrameParser
// ---------------------------------------------------------------------------

/// A configurable, template-driven frame parser.
///
/// Given a [`FrameDefinition`] the parser can locate sync patterns, extract
/// header fields, read payloads, compute/verify checksums, build frames from
/// field values, and parse multi-frame byte streams with error recovery.
pub struct ProtocolFrameParser {
    def: FrameDefinition,
}

impl ProtocolFrameParser {
    /// Create a new parser from the given frame definition.
    pub fn new(def: FrameDefinition) -> Self {
        Self { def }
    }

    /// Return a reference to the underlying frame definition.
    pub fn definition(&self) -> &FrameDefinition {
        &self.def
    }

    // -- Sync detection -----------------------------------------------------

    /// Search `data` for the sync pattern starting at the beginning.
    /// Returns the byte offset of the first occurrence, or `None`.
    pub fn find_sync(&self, data: &[u8]) -> Option<usize> {
        let pat = &self.def.sync_pattern;
        if pat.is_empty() || data.len() < pat.len() {
            return None;
        }
        data.windows(pat.len()).position(|w| w == pat.as_slice())
    }

    // -- Header helpers -----------------------------------------------------

    /// Total header size in bytes (rounded up from max bit extent).
    fn header_byte_len(&self) -> usize {
        let max_bit = self.header_max_bit();
        (max_bit + 7) / 8
    }

    /// Maximum bit extent of header fields (including the length field).
    fn header_max_bit(&self) -> usize {
        let mut max = 0usize;
        for f in &self.def.header_fields {
            let end = f.bit_offset + f.bit_length;
            if end > max {
                max = end;
            }
        }
        if let Some(ref lf) = self.def.length_field {
            let end = lf.bit_offset + lf.bit_length;
            if end > max {
                max = end;
            }
        }
        max
    }

    /// Checksum length in bytes.
    fn checksum_len(&self) -> usize {
        match self.def.checksum_type {
            ChecksumType::None => 0,
            ChecksumType::Crc8 | ChecksumType::Sum8 | ChecksumType::Xor8 => 1,
            ChecksumType::Crc16 => 2,
            ChecksumType::Crc32 => 4,
        }
    }

    // -- Bit extraction / insertion -----------------------------------------

    /// Extract an unsigned value of `bit_length` bits starting at
    /// `bit_offset` from `bytes`. Big-endian bit order.
    fn extract_bits(bytes: &[u8], bit_offset: usize, bit_length: usize) -> u64 {
        let mut value: u64 = 0;
        for i in 0..bit_length {
            let byte_idx = (bit_offset + i) / 8;
            let bit_idx = 7 - ((bit_offset + i) % 8); // MSB first
            if byte_idx < bytes.len() {
                let bit = ((bytes[byte_idx] >> bit_idx) & 1) as u64;
                value = (value << 1) | bit;
            }
        }
        value
    }

    /// Write an unsigned value into `bytes` at `bit_offset` for `bit_length`
    /// bits. Big-endian bit order.
    fn insert_bits(bytes: &mut [u8], bit_offset: usize, bit_length: usize, value: u64) {
        for i in 0..bit_length {
            let byte_idx = (bit_offset + i) / 8;
            let bit_idx = 7 - ((bit_offset + i) % 8);
            let src_bit = (value >> (bit_length - 1 - i)) & 1;
            if byte_idx < bytes.len() {
                if src_bit == 1 {
                    bytes[byte_idx] |= 1 << bit_idx;
                } else {
                    bytes[byte_idx] &= !(1 << bit_idx);
                }
            }
        }
    }

    /// Extract bytes (for [`FieldType::Bytes`]) -- must be byte-aligned.
    fn extract_field_bytes(bytes: &[u8], bit_offset: usize, bit_length: usize) -> Vec<u8> {
        let start = bit_offset / 8;
        let len = bit_length / 8;
        if start + len <= bytes.len() {
            bytes[start..start + len].to_vec()
        } else {
            Vec::new()
        }
    }

    // -- Field extraction / insertion ---------------------------------------

    fn extract_field(header: &[u8], fd: &FieldDefinition) -> FieldValue {
        match fd.field_type {
            FieldType::U8 | FieldType::U16 | FieldType::U32 => {
                FieldValue::Unsigned(Self::extract_bits(header, fd.bit_offset, fd.bit_length))
            }
            FieldType::Bool => {
                FieldValue::Boolean(Self::extract_bits(header, fd.bit_offset, fd.bit_length) != 0)
            }
            FieldType::Bytes => {
                FieldValue::Bytes(Self::extract_field_bytes(header, fd.bit_offset, fd.bit_length))
            }
        }
    }

    fn insert_field(
        header: &mut [u8],
        fd: &FieldDefinition,
        val: &FieldValue,
    ) -> Result<(), FrameError> {
        match (val, &fd.field_type) {
            (FieldValue::Unsigned(v), FieldType::U8 | FieldType::U16 | FieldType::U32) => {
                let max = if fd.bit_length >= 64 {
                    u64::MAX
                } else {
                    (1u64 << fd.bit_length) - 1
                };
                if *v > max {
                    return Err(FrameError::FieldOverflow(fd.name.clone()));
                }
                Self::insert_bits(header, fd.bit_offset, fd.bit_length, *v);
            }
            (FieldValue::Boolean(b), FieldType::Bool) => {
                Self::insert_bits(header, fd.bit_offset, fd.bit_length, *b as u64);
            }
            (FieldValue::Bytes(b), FieldType::Bytes) => {
                let start = fd.bit_offset / 8;
                let len = fd.bit_length / 8;
                let copy_len = b.len().min(len);
                header[start..start + copy_len].copy_from_slice(&b[..copy_len]);
            }
            _ => return Err(FrameError::FieldOverflow(fd.name.clone())),
        }
        Ok(())
    }

    // -- Checksum computation -----------------------------------------------

    /// Compute checksum over `data` and return it as a byte vector.
    pub fn compute_checksum(&self, data: &[u8]) -> Vec<u8> {
        match self.def.checksum_type {
            ChecksumType::None => Vec::new(),
            ChecksumType::Sum8 => {
                let s: u8 = data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
                vec![s]
            }
            ChecksumType::Xor8 => {
                let x: u8 = data.iter().fold(0u8, |acc, &b| acc ^ b);
                vec![x]
            }
            ChecksumType::Crc8 => {
                // CRC-8/SMBUS polynomial 0x07
                let mut crc: u8 = 0x00;
                for &byte in data {
                    crc ^= byte;
                    for _ in 0..8 {
                        if crc & 0x80 != 0 {
                            crc = (crc << 1) ^ 0x07;
                        } else {
                            crc <<= 1;
                        }
                    }
                }
                vec![crc]
            }
            ChecksumType::Crc16 => {
                // CRC-16/CCITT-FALSE polynomial 0x1021, init 0xFFFF
                let mut crc: u16 = 0xFFFF;
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
                vec![(crc >> 8) as u8, crc as u8]
            }
            ChecksumType::Crc32 => {
                // CRC-32 (ISO 3309 / ITU-T V.42)
                let mut crc: u32 = 0xFFFFFFFF;
                for &byte in data {
                    crc ^= byte as u32;
                    for _ in 0..8 {
                        if crc & 1 != 0 {
                            crc = (crc >> 1) ^ 0xEDB88320;
                        } else {
                            crc >>= 1;
                        }
                    }
                }
                crc ^= 0xFFFFFFFF;
                vec![
                    (crc >> 24) as u8,
                    (crc >> 16) as u8,
                    (crc >> 8) as u8,
                    crc as u8,
                ]
            }
        }
    }

    /// Validate the checksum of a complete frame (sync + header + payload +
    /// checksum). Returns `true` when the checksum matches or when
    /// [`ChecksumType::None`].
    pub fn validate_checksum(&self, frame: &[u8]) -> bool {
        if self.def.checksum_type == ChecksumType::None {
            return true;
        }
        let ck_len = self.checksum_len();
        if frame.len() < ck_len {
            return false;
        }
        let (body, stored) = frame.split_at(frame.len() - ck_len);
        let computed = self.compute_checksum(body);
        computed.as_slice() == stored
    }

    // -- Parsing ------------------------------------------------------------

    /// Parse a single frame starting at the beginning of `data`.
    ///
    /// `data` must start with the sync pattern. Use [`find_sync`] first if
    /// the stream may contain leading garbage.
    ///
    /// [`find_sync`]: ProtocolFrameParser::find_sync
    pub fn parse(&self, data: &[u8]) -> Result<ParsedFrame, FrameError> {
        let sync_len = self.def.sync_pattern.len();
        let hdr_len = self.header_byte_len();
        let ck_len = self.checksum_len();

        // Verify sync
        if data.len() < sync_len {
            return Err(FrameError::InsufficientData);
        }
        if &data[..sync_len] != self.def.sync_pattern.as_slice() {
            return Err(FrameError::SyncNotFound);
        }

        // Need at least header
        if data.len() < sync_len + hdr_len {
            return Err(FrameError::InsufficientData);
        }

        let header = &data[sync_len..sync_len + hdr_len];

        // Extract fields
        let mut fields = HashMap::new();
        for fd in &self.def.header_fields {
            fields.insert(fd.name.clone(), Self::extract_field(header, fd));
        }

        // Determine payload length
        let payload_len = if let Some(ref lf) = self.def.length_field {
            let raw = Self::extract_bits(header, lf.bit_offset, lf.bit_length) as usize;
            if lf.includes_header {
                raw.saturating_sub(hdr_len)
            } else {
                raw
            }
        } else {
            // No length field -- payload is everything after header minus checksum
            let remaining = data.len().saturating_sub(sync_len + hdr_len + ck_len);
            remaining
        };

        let payload_start = sync_len + hdr_len;
        let frame_end = payload_start + payload_len + ck_len;
        if data.len() < frame_end {
            return Err(FrameError::InsufficientData);
        }

        let payload = data[payload_start..payload_start + payload_len].to_vec();
        let valid_checksum = self.validate_checksum(&data[..frame_end]);

        Ok(ParsedFrame {
            fields,
            payload,
            valid_checksum,
        })
    }

    /// Parse a byte stream that may contain multiple frames.
    ///
    /// The parser searches for sync patterns, attempts to parse each frame,
    /// and on failure skips ahead to the next sync word (error recovery).
    pub fn parse_stream(&self, data: &[u8]) -> Vec<ParsedFrame> {
        let mut results = Vec::new();
        let mut offset = 0;
        let sync_len = self.def.sync_pattern.len();

        while offset < data.len() {
            // Find next sync
            match self.find_sync(&data[offset..]) {
                Some(rel) => {
                    let abs = offset + rel;
                    match self.parse(&data[abs..]) {
                        Ok(frame) => {
                            // Advance past this frame
                            let hdr_len = self.header_byte_len();
                            let ck_len = self.checksum_len();
                            let frame_len =
                                sync_len + hdr_len + frame.payload.len() + ck_len;
                            results.push(frame);
                            offset = abs + frame_len;
                        }
                        Err(_) => {
                            // Skip past this sync and try the next
                            offset = abs + sync_len;
                        }
                    }
                }
                None => break,
            }
        }

        results
    }

    // -- Frame construction -------------------------------------------------

    /// Build a complete frame (sync + header + payload + checksum) from the
    /// given field values and payload bytes.
    pub fn build_frame(
        &self,
        fields: &HashMap<String, FieldValue>,
        payload: &[u8],
    ) -> Result<Vec<u8>, FrameError> {
        let sync_len = self.def.sync_pattern.len();
        let hdr_len = self.header_byte_len();
        let ck_len = self.checksum_len();

        let mut frame = Vec::with_capacity(sync_len + hdr_len + payload.len() + ck_len);

        // Sync
        frame.extend_from_slice(&self.def.sync_pattern);

        // Header (zero-initialized)
        let header_start = frame.len();
        frame.resize(header_start + hdr_len, 0u8);

        // Insert user fields
        for fd in &self.def.header_fields {
            let val = fields
                .get(&fd.name)
                .ok_or_else(|| FrameError::MissingField(fd.name.clone()))?;
            Self::insert_field(&mut frame[header_start..header_start + hdr_len], fd, val)?;
        }

        // Insert length field
        if let Some(ref lf) = self.def.length_field {
            let encoded_len = if lf.includes_header {
                hdr_len + payload.len()
            } else {
                payload.len()
            };
            let max = if lf.bit_length >= 64 {
                u64::MAX
            } else {
                (1u64 << lf.bit_length) - 1
            };
            if encoded_len as u64 > max {
                return Err(FrameError::FieldOverflow("length".into()));
            }
            Self::insert_bits(
                &mut frame[header_start..header_start + hdr_len],
                lf.bit_offset,
                lf.bit_length,
                encoded_len as u64,
            );
        }

        // Payload
        frame.extend_from_slice(payload);

        // Checksum (computed over sync + header + payload)
        let cksum = self.compute_checksum(&frame);
        frame.extend_from_slice(&cksum);

        Ok(frame)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: simple protocol with 2-byte sync, 1-byte ID, 1-byte length,
    /// payload, XOR-8 checksum.
    fn simple_def() -> FrameDefinition {
        FrameDefinition {
            sync_pattern: vec![0xAA, 0x55],
            header_fields: vec![FieldDefinition {
                name: "id".into(),
                bit_offset: 0,
                bit_length: 8,
                field_type: FieldType::U8,
            }],
            length_field: Some(LengthField {
                bit_offset: 8,
                bit_length: 8,
                includes_header: false,
            }),
            checksum_type: ChecksumType::Xor8,
        }
    }

    #[test]
    fn test_build_and_parse_roundtrip() {
        let parser = ProtocolFrameParser::new(simple_def());
        let mut fields = HashMap::new();
        fields.insert("id".into(), FieldValue::Unsigned(0x42));
        let frame = parser.build_frame(&fields, &[1, 2, 3]).unwrap();
        let parsed = parser.parse(&frame).unwrap();
        assert_eq!(parsed.fields["id"], FieldValue::Unsigned(0x42));
        assert_eq!(parsed.payload, vec![1, 2, 3]);
        assert!(parsed.valid_checksum);
    }

    #[test]
    fn test_find_sync() {
        let parser = ProtocolFrameParser::new(simple_def());
        let data = [0x00, 0x00, 0xAA, 0x55, 0x01, 0x00];
        assert_eq!(parser.find_sync(&data), Some(2));
    }

    #[test]
    fn test_find_sync_not_present() {
        let parser = ProtocolFrameParser::new(simple_def());
        let data = [0x00, 0x01, 0x02];
        assert_eq!(parser.find_sync(&data), None);
    }

    #[test]
    fn test_parse_insufficient_data() {
        let parser = ProtocolFrameParser::new(simple_def());
        // Only the sync, no header
        let data = [0xAA, 0x55];
        let result = parser.parse(&data);
        assert_eq!(result.unwrap_err(), FrameError::InsufficientData);
    }

    #[test]
    fn test_parse_wrong_sync() {
        let parser = ProtocolFrameParser::new(simple_def());
        let data = [0xBB, 0x55, 0x00, 0x00, 0x00];
        let result = parser.parse(&data);
        assert_eq!(result.unwrap_err(), FrameError::SyncNotFound);
    }

    #[test]
    fn test_missing_field_in_build() {
        let parser = ProtocolFrameParser::new(simple_def());
        let fields = HashMap::new(); // missing "id"
        let result = parser.build_frame(&fields, &[]);
        assert!(matches!(result, Err(FrameError::MissingField(_))));
    }

    #[test]
    fn test_checksum_xor8() {
        let def = FrameDefinition {
            sync_pattern: vec![0xAA],
            header_fields: vec![],
            length_field: None,
            checksum_type: ChecksumType::Xor8,
        };
        let parser = ProtocolFrameParser::new(def);
        assert_eq!(parser.compute_checksum(&[0x01, 0x02, 0x03]), vec![0x00]);
        assert_eq!(parser.compute_checksum(&[0xFF, 0x00]), vec![0xFF]);
    }

    #[test]
    fn test_checksum_sum8() {
        let def = FrameDefinition {
            sync_pattern: vec![],
            header_fields: vec![],
            length_field: None,
            checksum_type: ChecksumType::Sum8,
        };
        let parser = ProtocolFrameParser::new(def);
        assert_eq!(parser.compute_checksum(&[1, 2, 3]), vec![6]);
        // Wrapping
        assert_eq!(parser.compute_checksum(&[0xFF, 0x02]), vec![0x01]);
    }

    #[test]
    fn test_checksum_crc8() {
        let def = FrameDefinition {
            sync_pattern: vec![],
            header_fields: vec![],
            length_field: None,
            checksum_type: ChecksumType::Crc8,
        };
        let parser = ProtocolFrameParser::new(def);
        let ck = parser.compute_checksum(&[0x00]);
        assert_eq!(ck.len(), 1);
        // CRC-8/SMBUS of 0x00 is 0x00
        assert_eq!(ck, vec![0x00]);
    }

    #[test]
    fn test_checksum_crc16() {
        let def = FrameDefinition {
            sync_pattern: vec![],
            header_fields: vec![],
            length_field: None,
            checksum_type: ChecksumType::Crc16,
        };
        let parser = ProtocolFrameParser::new(def);
        let ck = parser.compute_checksum(b"123456789");
        assert_eq!(ck.len(), 2);
        // CRC-16/CCITT-FALSE of "123456789" is 0x29B1
        assert_eq!(ck, vec![0x29, 0xB1]);
    }

    #[test]
    fn test_checksum_crc32() {
        let def = FrameDefinition {
            sync_pattern: vec![],
            header_fields: vec![],
            length_field: None,
            checksum_type: ChecksumType::Crc32,
        };
        let parser = ProtocolFrameParser::new(def);
        let ck = parser.compute_checksum(b"123456789");
        assert_eq!(ck.len(), 4);
        // CRC-32 of "123456789" is 0xCBF43926
        assert_eq!(ck, vec![0xCB, 0xF4, 0x39, 0x26]);
    }

    #[test]
    fn test_validate_checksum_valid() {
        let parser = ProtocolFrameParser::new(simple_def());
        let mut fields = HashMap::new();
        fields.insert("id".into(), FieldValue::Unsigned(0x01));
        let frame = parser.build_frame(&fields, &[0xAB]).unwrap();
        assert!(parser.validate_checksum(&frame));
    }

    #[test]
    fn test_validate_checksum_invalid() {
        let parser = ProtocolFrameParser::new(simple_def());
        let mut fields = HashMap::new();
        fields.insert("id".into(), FieldValue::Unsigned(0x01));
        let mut frame = parser.build_frame(&fields, &[0xAB]).unwrap();
        // Corrupt the checksum
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let parsed = parser.parse(&frame).unwrap();
        assert!(!parsed.valid_checksum);
    }

    #[test]
    fn test_parse_stream_multiple_frames() {
        let parser = ProtocolFrameParser::new(simple_def());
        let mut fields1 = HashMap::new();
        fields1.insert("id".into(), FieldValue::Unsigned(1));
        let mut fields2 = HashMap::new();
        fields2.insert("id".into(), FieldValue::Unsigned(2));

        let f1 = parser.build_frame(&fields1, &[0x10]).unwrap();
        let f2 = parser.build_frame(&fields2, &[0x20, 0x21]).unwrap();

        let mut stream = Vec::new();
        stream.extend_from_slice(&f1);
        stream.extend_from_slice(&f2);

        let frames = parser.parse_stream(&stream);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].fields["id"], FieldValue::Unsigned(1));
        assert_eq!(frames[0].payload, vec![0x10]);
        assert_eq!(frames[1].fields["id"], FieldValue::Unsigned(2));
        assert_eq!(frames[1].payload, vec![0x20, 0x21]);
    }

    #[test]
    fn test_parse_stream_with_garbage() {
        let parser = ProtocolFrameParser::new(simple_def());
        let mut fields = HashMap::new();
        fields.insert("id".into(), FieldValue::Unsigned(7));
        let frame = parser.build_frame(&fields, &[0xFF]).unwrap();

        let mut stream = vec![0x00, 0x01, 0x02]; // garbage
        stream.extend_from_slice(&frame);
        stream.extend_from_slice(&[0x99, 0x98]); // trailing garbage

        let frames = parser.parse_stream(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].fields["id"], FieldValue::Unsigned(7));
        assert!(frames[0].valid_checksum);
    }

    #[test]
    fn test_bool_field() {
        let def = FrameDefinition {
            sync_pattern: vec![0xFE],
            header_fields: vec![
                FieldDefinition {
                    name: "flag".into(),
                    bit_offset: 0,
                    bit_length: 1,
                    field_type: FieldType::Bool,
                },
                FieldDefinition {
                    name: "seq".into(),
                    bit_offset: 1,
                    bit_length: 7,
                    field_type: FieldType::U8,
                },
            ],
            length_field: Some(LengthField {
                bit_offset: 8,
                bit_length: 8,
                includes_header: false,
            }),
            checksum_type: ChecksumType::None,
        };
        let parser = ProtocolFrameParser::new(def);
        let mut fields = HashMap::new();
        fields.insert("flag".into(), FieldValue::Boolean(true));
        fields.insert("seq".into(), FieldValue::Unsigned(42));
        let frame = parser.build_frame(&fields, &[0xBB]).unwrap();
        let parsed = parser.parse(&frame).unwrap();
        assert_eq!(parsed.fields["flag"], FieldValue::Boolean(true));
        assert_eq!(parsed.fields["seq"], FieldValue::Unsigned(42));
        assert_eq!(parsed.payload, vec![0xBB]);
    }

    #[test]
    fn test_u16_field() {
        let def = FrameDefinition {
            sync_pattern: vec![0xDE, 0xAD],
            header_fields: vec![FieldDefinition {
                name: "addr".into(),
                bit_offset: 0,
                bit_length: 16,
                field_type: FieldType::U16,
            }],
            length_field: None,
            checksum_type: ChecksumType::None,
        };
        let parser = ProtocolFrameParser::new(def);
        let mut fields = HashMap::new();
        fields.insert("addr".into(), FieldValue::Unsigned(0x1234));
        let frame = parser.build_frame(&fields, &[]).unwrap();
        let parsed = parser.parse(&frame).unwrap();
        assert_eq!(parsed.fields["addr"], FieldValue::Unsigned(0x1234));
    }

    #[test]
    fn test_bytes_field() {
        let def = FrameDefinition {
            sync_pattern: vec![0x7E],
            header_fields: vec![FieldDefinition {
                name: "mac".into(),
                bit_offset: 0,
                bit_length: 48, // 6 bytes
                field_type: FieldType::Bytes,
            }],
            length_field: Some(LengthField {
                bit_offset: 48,
                bit_length: 8,
                includes_header: false,
            }),
            checksum_type: ChecksumType::None,
        };
        let parser = ProtocolFrameParser::new(def);
        let mac = vec![0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        let mut fields = HashMap::new();
        fields.insert("mac".into(), FieldValue::Bytes(mac.clone()));
        let frame = parser.build_frame(&fields, &[0x01]).unwrap();
        let parsed = parser.parse(&frame).unwrap();
        assert_eq!(parsed.fields["mac"], FieldValue::Bytes(mac));
        assert_eq!(parsed.payload, vec![0x01]);
    }

    #[test]
    fn test_no_checksum_roundtrip() {
        let def = FrameDefinition {
            sync_pattern: vec![0xCC],
            header_fields: vec![FieldDefinition {
                name: "type".into(),
                bit_offset: 0,
                bit_length: 8,
                field_type: FieldType::U8,
            }],
            length_field: Some(LengthField {
                bit_offset: 8,
                bit_length: 8,
                includes_header: false,
            }),
            checksum_type: ChecksumType::None,
        };
        let parser = ProtocolFrameParser::new(def);
        let mut fields = HashMap::new();
        fields.insert("type".into(), FieldValue::Unsigned(3));
        let frame = parser.build_frame(&fields, &[10, 20]).unwrap();
        let parsed = parser.parse(&frame).unwrap();
        assert_eq!(parsed.fields["type"], FieldValue::Unsigned(3));
        assert_eq!(parsed.payload, vec![10, 20]);
        assert!(parsed.valid_checksum); // None always valid
    }

    #[test]
    fn test_field_overflow_rejected() {
        let parser = ProtocolFrameParser::new(simple_def());
        let mut fields = HashMap::new();
        fields.insert("id".into(), FieldValue::Unsigned(256)); // 8-bit field, max 255
        let result = parser.build_frame(&fields, &[]);
        assert!(matches!(result, Err(FrameError::FieldOverflow(_))));
    }

    #[test]
    fn test_length_includes_header() {
        let def = FrameDefinition {
            sync_pattern: vec![0xAB],
            header_fields: vec![],
            length_field: Some(LengthField {
                bit_offset: 0,
                bit_length: 8,
                includes_header: true,
            }),
            checksum_type: ChecksumType::None,
        };
        let parser = ProtocolFrameParser::new(def);
        let fields = HashMap::new();
        let frame = parser.build_frame(&fields, &[0x01, 0x02]).unwrap();
        // sync(1) + header(1: length field) + payload(2)
        // length field should encode header_len + payload_len = 1 + 2 = 3
        assert_eq!(frame.len(), 4);
        // The length byte (frame[1]) should be 3
        assert_eq!(frame[1], 3);
        let parsed = parser.parse(&frame).unwrap();
        assert_eq!(parsed.payload, vec![0x01, 0x02]);
    }

    #[test]
    fn test_error_display() {
        assert_eq!(format!("{}", FrameError::SyncNotFound), "sync pattern not found");
        assert_eq!(
            format!("{}", FrameError::MissingField("x".into())),
            "missing field: x"
        );
    }
}

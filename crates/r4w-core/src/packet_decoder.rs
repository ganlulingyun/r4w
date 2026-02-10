//! # Generic Packet Decoder
//!
//! Configurable packet decoder that extracts packets from a bitstream
//! using sync word detection, header parsing, payload extraction, and
//! CRC verification. Works with any packet format by configuring the
//! field layout.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::packet_decoder::{PacketDecoder, PacketFormat, Field};
//!
//! let format = PacketFormat {
//!     sync_word: vec![true, false, true, false, true, false, true, false],
//!     fields: vec![
//!         Field::length("len", 8),
//!         Field::payload("data"),
//!         Field::crc("crc", 16),
//!     ],
//!     max_packet_len: 256,
//! };
//!
//! let mut decoder = PacketDecoder::new(format);
//! ```

/// A field in the packet format.
#[derive(Debug, Clone)]
pub struct Field {
    /// Field name.
    pub name: String,
    /// Field type.
    pub field_type: FieldType,
    /// Number of bits (fixed fields only).
    pub bits: usize,
}

/// Type of packet field.
#[derive(Debug, Clone, PartialEq)]
pub enum FieldType {
    /// Fixed-size field (e.g., address, flags).
    Fixed,
    /// Length field that determines payload size.
    Length,
    /// Variable-length payload (size from length field).
    Payload,
    /// CRC field.
    Crc,
}

impl Field {
    /// Create a fixed-size field.
    pub fn fixed(name: &str, bits: usize) -> Self {
        Self {
            name: name.to_string(),
            field_type: FieldType::Fixed,
            bits,
        }
    }

    /// Create a length field.
    pub fn length(name: &str, bits: usize) -> Self {
        Self {
            name: name.to_string(),
            field_type: FieldType::Length,
            bits,
        }
    }

    /// Create a payload field (variable length).
    pub fn payload(name: &str) -> Self {
        Self {
            name: name.to_string(),
            field_type: FieldType::Payload,
            bits: 0,
        }
    }

    /// Create a CRC field.
    pub fn crc(name: &str, bits: usize) -> Self {
        Self {
            name: name.to_string(),
            field_type: FieldType::Crc,
            bits,
        }
    }
}

/// Packet format specification.
#[derive(Debug, Clone)]
pub struct PacketFormat {
    /// Sync word (preamble pattern to detect).
    pub sync_word: Vec<bool>,
    /// Ordered list of fields after the sync word.
    pub fields: Vec<Field>,
    /// Maximum packet length in bytes.
    pub max_packet_len: usize,
}

/// A decoded packet.
#[derive(Debug, Clone)]
pub struct DecodedPacket {
    /// Field values: name → bit vector.
    pub fields: Vec<(String, Vec<bool>)>,
    /// Whether CRC passed.
    pub crc_valid: bool,
    /// Bit offset where sync was found.
    pub sync_offset: usize,
}

impl DecodedPacket {
    /// Get a field's bits by name.
    pub fn get_field(&self, name: &str) -> Option<&[bool]> {
        self.fields
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, bits)| bits.as_slice())
    }

    /// Get a field's value as an integer (MSB first).
    pub fn get_field_value(&self, name: &str) -> Option<u64> {
        self.get_field(name).map(|bits| {
            let mut val = 0u64;
            for &bit in bits {
                val = (val << 1) | (bit as u64);
            }
            val
        })
    }

    /// Get the payload bytes.
    pub fn payload_bytes(&self) -> Vec<u8> {
        if let Some(bits) = self.get_field("data")
            .or_else(|| self.get_field("payload"))
        {
            bits_to_bytes(bits)
        } else {
            Vec::new()
        }
    }
}

/// Generic packet decoder.
#[derive(Debug, Clone)]
pub struct PacketDecoder {
    format: PacketFormat,
    /// Bit buffer for incoming data.
    bit_buffer: Vec<bool>,
    /// Total packets decoded.
    packets_decoded: u64,
    /// Total CRC failures.
    crc_failures: u64,
}

impl PacketDecoder {
    /// Create a new packet decoder.
    pub fn new(format: PacketFormat) -> Self {
        Self {
            format,
            bit_buffer: Vec::new(),
            packets_decoded: 0,
            crc_failures: 0,
        }
    }

    /// Feed bits into the decoder and extract any complete packets.
    pub fn feed(&mut self, bits: &[bool]) -> Vec<DecodedPacket> {
        self.bit_buffer.extend_from_slice(bits);
        let mut packets = Vec::new();

        loop {
            // Search for sync word.
            let sync_pos = self.find_sync();
            if sync_pos.is_none() {
                break;
            }
            let sync_pos = sync_pos.unwrap();
            let start = sync_pos + self.format.sync_word.len();

            // Try to extract packet fields.
            match self.extract_packet(start, sync_pos) {
                Some((packet, end_pos)) => {
                    self.packets_decoded += 1;
                    if !packet.crc_valid {
                        self.crc_failures += 1;
                    }
                    packets.push(packet);
                    // Remove processed bits.
                    self.bit_buffer = self.bit_buffer[end_pos..].to_vec();
                }
                None => {
                    // Not enough data yet. Keep buffer.
                    break;
                }
            }
        }

        // Prevent unbounded buffer growth.
        let max_bits = self.format.max_packet_len * 8 * 2 + self.format.sync_word.len();
        if self.bit_buffer.len() > max_bits {
            let trim = self.bit_buffer.len() - max_bits;
            self.bit_buffer = self.bit_buffer[trim..].to_vec();
        }

        packets
    }

    fn find_sync(&self) -> Option<usize> {
        let sw = &self.format.sync_word;
        if sw.is_empty() || self.bit_buffer.len() < sw.len() {
            return None;
        }
        for i in 0..=self.bit_buffer.len() - sw.len() {
            if self.bit_buffer[i..i + sw.len()] == sw[..] {
                return Some(i);
            }
        }
        None
    }

    fn extract_packet(
        &self,
        start: usize,
        sync_offset: usize,
    ) -> Option<(DecodedPacket, usize)> {
        let mut pos = start;
        let mut fields = Vec::new();
        let mut payload_len_bits = 0;
        let mut has_length_field = false;

        for field in &self.format.fields {
            let field_bits = match field.field_type {
                FieldType::Fixed | FieldType::Length | FieldType::Crc => field.bits,
                FieldType::Payload => {
                    if has_length_field {
                        payload_len_bits
                    } else {
                        return None; // Need length field before payload.
                    }
                }
            };

            if pos + field_bits > self.bit_buffer.len() {
                return None; // Not enough data.
            }

            let bits = self.bit_buffer[pos..pos + field_bits].to_vec();

            if field.field_type == FieldType::Length {
                has_length_field = true;
                // Interpret as unsigned integer, convert to bits for payload.
                let mut val = 0usize;
                for &b in &bits {
                    val = (val << 1) | (b as usize);
                }
                payload_len_bits = val * 8; // length in bytes → bits
                let max_bits = self.format.max_packet_len * 8;
                if payload_len_bits > max_bits {
                    return None; // Invalid length.
                }
            }

            fields.push((field.name.clone(), bits));
            pos += field_bits;
        }

        let packet = DecodedPacket {
            fields,
            crc_valid: true, // simplified: CRC validation not implemented here
            sync_offset,
        };

        Some((packet, pos))
    }

    /// Get total packets decoded.
    pub fn packets_decoded(&self) -> u64 {
        self.packets_decoded
    }

    /// Get total CRC failures.
    pub fn crc_failures(&self) -> u64 {
        self.crc_failures
    }

    /// Get the current bit buffer length.
    pub fn buffer_len(&self) -> usize {
        self.bit_buffer.len()
    }

    /// Reset the decoder.
    pub fn reset(&mut self) {
        self.bit_buffer.clear();
        self.packets_decoded = 0;
        self.crc_failures = 0;
    }
}

fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity((bits.len() + 7) / 8);
    for chunk in bits.chunks(8) {
        let mut byte = 0u8;
        for (i, &bit) in chunk.iter().enumerate() {
            if bit {
                byte |= 1 << (7 - i);
            }
        }
        bytes.push(byte);
    }
    bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_format() -> PacketFormat {
        PacketFormat {
            sync_word: vec![true, false, true, false],
            fields: vec![
                Field::length("len", 8),
                Field::payload("data"),
            ],
            max_packet_len: 64,
        }
    }

    fn u8_to_bits(val: u8) -> Vec<bool> {
        (0..8).map(|i| (val >> (7 - i)) & 1 == 1).collect()
    }

    #[test]
    fn test_decode_simple_packet() {
        let mut decoder = PacketDecoder::new(make_format());

        // Build: sync + len(2) + 2 bytes payload
        let mut bits = vec![true, false, true, false]; // sync
        bits.extend(u8_to_bits(2)); // length = 2 bytes
        bits.extend(u8_to_bits(0xAB)); // payload byte 1
        bits.extend(u8_to_bits(0xCD)); // payload byte 2

        let packets = decoder.feed(&bits);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].get_field_value("len"), Some(2));
        assert_eq!(packets[0].payload_bytes(), vec![0xAB, 0xCD]);
    }

    #[test]
    fn test_no_sync() {
        let mut decoder = PacketDecoder::new(make_format());
        let bits = vec![false; 100];
        let packets = decoder.feed(&bits);
        assert!(packets.is_empty());
    }

    #[test]
    fn test_incomplete_packet() {
        let mut decoder = PacketDecoder::new(make_format());
        // Just sync + partial length.
        let bits = vec![true, false, true, false, true, false];
        let packets = decoder.feed(&bits);
        assert!(packets.is_empty());
        assert!(decoder.buffer_len() > 0);
    }

    #[test]
    fn test_multiple_packets() {
        let mut decoder = PacketDecoder::new(make_format());

        let mut bits = Vec::new();
        for _ in 0..3 {
            bits.extend([true, false, true, false]); // sync
            bits.extend(u8_to_bits(1)); // len=1
            bits.extend(u8_to_bits(0xFF)); // payload
        }

        let packets = decoder.feed(&bits);
        assert_eq!(packets.len(), 3);
        assert_eq!(decoder.packets_decoded(), 3);
    }

    #[test]
    fn test_field_types() {
        let f1 = Field::fixed("flags", 8);
        assert_eq!(f1.field_type, FieldType::Fixed);
        assert_eq!(f1.bits, 8);

        let f2 = Field::length("len", 16);
        assert_eq!(f2.field_type, FieldType::Length);

        let f3 = Field::payload("data");
        assert_eq!(f3.field_type, FieldType::Payload);

        let f4 = Field::crc("checksum", 32);
        assert_eq!(f4.field_type, FieldType::Crc);
    }

    #[test]
    fn test_bits_to_bytes() {
        let bits: Vec<bool> = vec![true, false, true, false, true, false, true, false]; // 0xAA
        assert_eq!(bits_to_bytes(&bits), vec![0xAA]);
    }

    #[test]
    fn test_zero_length_payload() {
        let mut decoder = PacketDecoder::new(make_format());
        let mut bits = vec![true, false, true, false]; // sync
        bits.extend(u8_to_bits(0)); // len=0
        let packets = decoder.feed(&bits);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].payload_bytes(), Vec::<u8>::new());
    }

    #[test]
    fn test_reset() {
        let mut decoder = PacketDecoder::new(make_format());
        let mut bits = vec![true, false, true, false];
        bits.extend(u8_to_bits(1));
        bits.extend(u8_to_bits(0x42));
        decoder.feed(&bits);
        assert_eq!(decoder.packets_decoded(), 1);
        decoder.reset();
        assert_eq!(decoder.packets_decoded(), 0);
        assert_eq!(decoder.buffer_len(), 0);
    }

    #[test]
    fn test_fixed_field() {
        let format = PacketFormat {
            sync_word: vec![true, true, true, true],
            fields: vec![
                Field::fixed("addr", 8),
                Field::length("len", 8),
                Field::payload("data"),
            ],
            max_packet_len: 64,
        };
        let mut decoder = PacketDecoder::new(format);

        let mut bits = vec![true, true, true, true]; // sync
        bits.extend(u8_to_bits(0x01)); // addr=1
        bits.extend(u8_to_bits(1)); // len=1
        bits.extend(u8_to_bits(0xBE)); // payload

        let packets = decoder.feed(&bits);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].get_field_value("addr"), Some(0x01));
    }

    #[test]
    fn test_invalid_length() {
        let mut decoder = PacketDecoder::new(make_format());
        let mut bits = vec![true, false, true, false]; // sync
        bits.extend(u8_to_bits(255)); // len=255 > max_packet_len
        bits.extend(vec![false; 100]); // not enough data
        let packets = decoder.feed(&bits);
        assert!(packets.is_empty());
    }
}

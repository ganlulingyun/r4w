//! Packet Encoder — Assemble data into framed packets
//!
//! Frames raw data bytes into packets with sync word, length field, payload,
//! and CRC. Supports common framing formats used in packet radio and IoT
//! protocols. Complementary to the existing packet_framing module which handles
//! parsing; this module handles assembly.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::packet_encoder::{PacketEncoder, PacketFormat};
//!
//! let encoder = PacketEncoder::new(PacketFormat::default());
//! let payload = b"Hello";
//! let packet = encoder.encode(payload);
//! // Packet contains: sync_word + length + payload + CRC
//! assert!(packet.len() > payload.len());
//! ```

/// Packet framing format.
#[derive(Debug, Clone)]
pub struct PacketFormat {
    /// Sync word prepended to each packet.
    pub sync_word: Vec<u8>,
    /// Whether to include a length field.
    pub include_length: bool,
    /// Length field size in bytes (1 or 2).
    pub length_bytes: usize,
    /// Whether to include a CRC.
    pub include_crc: bool,
    /// CRC type.
    pub crc_type: CrcType,
    /// Whether to whitening/scramble the payload.
    pub whitening: bool,
    /// Preamble bytes (repeated 0xAA pattern typically).
    pub preamble_bytes: usize,
}

impl Default for PacketFormat {
    fn default() -> Self {
        Self {
            sync_word: vec![0x7E, 0x7E],
            include_length: true,
            length_bytes: 1,
            include_crc: true,
            crc_type: CrcType::Crc16Ccitt,
            whitening: false,
            preamble_bytes: 4,
        }
    }
}

/// CRC type for packet integrity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrcType {
    /// No CRC.
    None,
    /// CRC-8 (x^8 + x^2 + x + 1).
    Crc8,
    /// CRC-16 CCITT (x^16 + x^12 + x^5 + 1).
    Crc16Ccitt,
    /// CRC-32 (Ethernet).
    Crc32,
}

/// Packet encoder.
#[derive(Debug, Clone)]
pub struct PacketEncoder {
    format: PacketFormat,
}

impl PacketEncoder {
    /// Create a packet encoder with given format.
    pub fn new(format: PacketFormat) -> Self {
        Self { format }
    }

    /// Create with LoRa-like format.
    pub fn lora_like() -> Self {
        Self::new(PacketFormat {
            sync_word: vec![0x34, 0x12],
            include_length: true,
            length_bytes: 1,
            include_crc: true,
            crc_type: CrcType::Crc16Ccitt,
            whitening: false,
            preamble_bytes: 8,
        })
    }

    /// Create with simple format (minimal overhead).
    pub fn simple() -> Self {
        Self::new(PacketFormat {
            sync_word: vec![0xAA, 0x55],
            include_length: true,
            length_bytes: 1,
            include_crc: true,
            crc_type: CrcType::Crc8,
            whitening: false,
            preamble_bytes: 2,
        })
    }

    /// Encode a payload into a framed packet.
    pub fn encode(&self, payload: &[u8]) -> Vec<u8> {
        let mut packet = Vec::new();

        // Preamble
        for _ in 0..self.format.preamble_bytes {
            packet.push(0xAA);
        }

        // Sync word
        packet.extend_from_slice(&self.format.sync_word);

        // Length field
        if self.format.include_length {
            match self.format.length_bytes {
                1 => packet.push(payload.len() as u8),
                2 => {
                    packet.push((payload.len() >> 8) as u8);
                    packet.push(payload.len() as u8);
                }
                _ => packet.push(payload.len() as u8),
            }
        }

        // Payload
        if self.format.whitening {
            let whitened = whiten(payload);
            packet.extend_from_slice(&whitened);
        } else {
            packet.extend_from_slice(payload);
        }

        // CRC (over payload only)
        if self.format.include_crc {
            match self.format.crc_type {
                CrcType::None => {}
                CrcType::Crc8 => {
                    let crc = crc8(payload);
                    packet.push(crc);
                }
                CrcType::Crc16Ccitt => {
                    let crc = crc16_ccitt(payload);
                    packet.push((crc >> 8) as u8);
                    packet.push(crc as u8);
                }
                CrcType::Crc32 => {
                    let crc = crc32(payload);
                    packet.push((crc >> 24) as u8);
                    packet.push((crc >> 16) as u8);
                    packet.push((crc >> 8) as u8);
                    packet.push(crc as u8);
                }
            }
        }

        packet
    }

    /// Encode payload to bits (MSB first per byte).
    pub fn encode_bits(&self, payload: &[u8]) -> Vec<bool> {
        let packet = self.encode(payload);
        let mut bits = Vec::with_capacity(packet.len() * 8);
        for byte in &packet {
            for i in (0..8).rev() {
                bits.push((byte >> i) & 1 == 1);
            }
        }
        bits
    }

    /// Get the overhead in bytes (preamble + sync + length + CRC).
    pub fn overhead(&self) -> usize {
        let mut oh = self.format.preamble_bytes + self.format.sync_word.len();
        if self.format.include_length {
            oh += self.format.length_bytes;
        }
        if self.format.include_crc {
            oh += match self.format.crc_type {
                CrcType::None => 0,
                CrcType::Crc8 => 1,
                CrcType::Crc16Ccitt => 2,
                CrcType::Crc32 => 4,
            };
        }
        oh
    }

    /// Get the format.
    pub fn format(&self) -> &PacketFormat {
        &self.format
    }
}

/// Simple LFSR whitening (XOR with pseudo-random sequence).
fn whiten(data: &[u8]) -> Vec<u8> {
    let mut lfsr: u16 = 0x1FF;
    data.iter().map(|&byte| {
        let mut out = 0u8;
        for i in 0..8 {
            let bit = (lfsr & 1) as u8;
            let feedback = ((lfsr >> 0) ^ (lfsr >> 5)) & 1;
            lfsr = (lfsr >> 1) | (feedback << 8);
            out |= (((byte >> i) & 1) ^ bit) << i;
        }
        out
    }).collect()
}

/// CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07).
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
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
    crc
}

/// CRC-16 CCITT with polynomial x^16 + x^12 + x^5 + 1.
fn crc16_ccitt(data: &[u8]) -> u16 {
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
    crc
}

/// CRC-32 (Ethernet polynomial).
fn crc32(data: &[u8]) -> u32 {
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
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_format() {
        let enc = PacketEncoder::new(PacketFormat::default());
        let packet = enc.encode(b"Hi");
        // 4 preamble + 2 sync + 1 length + 2 payload + 2 CRC = 11
        assert_eq!(packet.len(), 11);
    }

    #[test]
    fn test_preamble() {
        let enc = PacketEncoder::new(PacketFormat::default());
        let packet = enc.encode(b"X");
        assert_eq!(packet[0], 0xAA);
        assert_eq!(packet[1], 0xAA);
        assert_eq!(packet[2], 0xAA);
        assert_eq!(packet[3], 0xAA);
    }

    #[test]
    fn test_sync_word() {
        let enc = PacketEncoder::new(PacketFormat::default());
        let packet = enc.encode(b"X");
        assert_eq!(packet[4], 0x7E); // First sync byte
        assert_eq!(packet[5], 0x7E);
    }

    #[test]
    fn test_length_field() {
        let enc = PacketEncoder::new(PacketFormat::default());
        let packet = enc.encode(b"Hello");
        assert_eq!(packet[6], 5); // Length = 5 bytes
    }

    #[test]
    fn test_payload_present() {
        let enc = PacketEncoder::new(PacketFormat::default());
        let packet = enc.encode(b"AB");
        assert_eq!(packet[7], b'A');
        assert_eq!(packet[8], b'B');
    }

    #[test]
    fn test_crc_appended() {
        let enc = PacketEncoder::new(PacketFormat::default());
        let packet1 = enc.encode(b"Hello");
        let packet2 = enc.encode(b"World");
        // CRC should differ for different payloads
        let crc1 = &packet1[packet1.len()-2..];
        let crc2 = &packet2[packet2.len()-2..];
        assert_ne!(crc1, crc2);
    }

    #[test]
    fn test_no_crc() {
        let format = PacketFormat {
            include_crc: false,
            crc_type: CrcType::None,
            ..PacketFormat::default()
        };
        let enc = PacketEncoder::new(format);
        let packet = enc.encode(b"Hi");
        // 4 preamble + 2 sync + 1 length + 2 payload = 9 (no CRC)
        assert_eq!(packet.len(), 9);
    }

    #[test]
    fn test_simple_format() {
        let enc = PacketEncoder::simple();
        let packet = enc.encode(b"T");
        let expected = 2 + 2 + 1 + 1 + 1; // preamble + sync + len + payload + crc8
        assert_eq!(packet.len(), expected);
    }

    #[test]
    fn test_encode_bits() {
        let enc = PacketEncoder::new(PacketFormat {
            preamble_bytes: 0,
            sync_word: vec![],
            include_length: false,
            include_crc: false,
            ..PacketFormat::default()
        });
        let bits = enc.encode_bits(&[0xA5]); // 10100101
        assert_eq!(bits.len(), 8);
        assert_eq!(bits, vec![true, false, true, false, false, true, false, true]);
    }

    #[test]
    fn test_overhead() {
        let enc = PacketEncoder::new(PacketFormat::default());
        assert_eq!(enc.overhead(), 4 + 2 + 1 + 2); // preamble + sync + len + CRC16
    }

    #[test]
    fn test_whitening() {
        let format = PacketFormat {
            whitening: true,
            ..PacketFormat::default()
        };
        let enc = PacketEncoder::new(format);
        let packet = enc.encode(b"Hello");
        // Payload should be different from original
        assert_ne!(&packet[7..12], b"Hello");
    }

    #[test]
    fn test_crc8() {
        let c1 = crc8(b"Hello");
        let c2 = crc8(b"World");
        assert_ne!(c1, c2);
        // Same data = same CRC
        assert_eq!(crc8(b"Hello"), crc8(b"Hello"));
    }

    #[test]
    fn test_crc32_format() {
        let format = PacketFormat {
            crc_type: CrcType::Crc32,
            ..PacketFormat::default()
        };
        let enc = PacketEncoder::new(format);
        let packet = enc.encode(b"X");
        // 4 preamble + 2 sync + 1 length + 1 payload + 4 CRC32 = 12
        assert_eq!(packet.len(), 12);
    }

    #[test]
    fn test_two_byte_length() {
        let format = PacketFormat {
            length_bytes: 2,
            ..PacketFormat::default()
        };
        let enc = PacketEncoder::new(format);
        let packet = enc.encode(b"X");
        // Length field is 2 bytes: [0x00, 0x01]
        assert_eq!(packet[6], 0x00);
        assert_eq!(packet[7], 0x01);
    }
}

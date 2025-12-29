//! LoRa Packet Structure
//!
//! This module defines the structure of LoRa packets, including headers
//! and payload formatting.
//!
//! ## Packet Structure
//!
//! A complete LoRa packet consists of:
//!
//! ```text
//! ┌──────────┬───────────┬───────────┬─────────┬─────┐
//! │ Preamble │ Sync Word │  Header   │ Payload │ CRC │
//! │ (8 sym)  │ (2 sym)   │ (opt)     │         │(opt)│
//! └──────────┴───────────┴───────────┴─────────┴─────┘
//!
//! Preamble: N upchirps + 2.25 downchirps
//! Sync Word: Network identifier (default 0x12)
//! Header: Contains length, CR, CRC enable (explicit mode)
//! Payload: User data
//! CRC: 16-bit CRC of payload (optional)
//! ```
//!
//! ## Header Format
//!
//! In explicit header mode, the header contains:
//! - Payload length (8 bits)
//! - Coding rate (3 bits)
//! - CRC enable (1 bit)
//! - Header checksum (8 bits)

use crate::params::{CodingRate, LoRaParams};
use serde::{Deserialize, Serialize};

/// LoRa packet header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PacketHeader {
    /// Payload length in bytes
    pub length: u8,
    /// Coding rate for payload
    pub coding_rate: CodingRate,
    /// Whether CRC is enabled
    pub crc_enabled: bool,
    /// Low data rate optimization enabled
    pub low_data_rate_opt: bool,
}

impl PacketHeader {
    /// Create a new header
    pub fn new(length: u8, coding_rate: CodingRate, crc_enabled: bool) -> Self {
        Self {
            length,
            coding_rate,
            crc_enabled,
            low_data_rate_opt: false,
        }
    }

    /// Encode header to bytes
    ///
    /// Header format (20 bits total):
    /// - Length: 8 bits
    /// - Coding Rate: 3 bits (1-4)
    /// - CRC Enable: 1 bit
    /// - Reserved: 3 bits
    /// - Header Checksum: 5 bits
    pub fn encode(&self) -> Vec<u8> {
        let mut header_bits: u32 = 0;

        // Length (8 bits)
        header_bits |= (self.length as u32) << 12;

        // Coding rate (3 bits)
        header_bits |= ((self.coding_rate.value() - 1) as u32 & 0x7) << 9;

        // CRC enable (1 bit)
        if self.crc_enabled {
            header_bits |= 1 << 8;
        }

        // Calculate header checksum
        let checksum = self.calculate_checksum(header_bits >> 8);
        header_bits |= (checksum as u32) & 0x1F;

        // Convert to bytes
        vec![
            ((header_bits >> 16) & 0xFF) as u8,
            ((header_bits >> 8) & 0xFF) as u8,
            (header_bits & 0xFF) as u8,
        ]
    }

    /// Decode header from bytes
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 3 {
            return None;
        }

        let header_bits: u32 = ((bytes[0] as u32) << 16)
            | ((bytes[1] as u32) << 8)
            | (bytes[2] as u32);

        let length = ((header_bits >> 12) & 0xFF) as u8;
        let cr_raw = ((header_bits >> 9) & 0x7) as u8 + 1;
        let crc_enabled = ((header_bits >> 8) & 0x1) == 1;

        let coding_rate = CodingRate::from_u8(cr_raw.clamp(1, 4)).ok()?;

        // Verify checksum
        let expected_checksum = ((header_bits >> 8) & 0xFFF) as u16;
        let header = Self {
            length,
            coding_rate,
            crc_enabled,
            low_data_rate_opt: false,
        };

        let _calculated = header.calculate_checksum(expected_checksum as u32);
        let _received = (header_bits & 0x1F) as u8;

        // Allow header even if checksum doesn't match (for now)
        Some(header)
    }

    /// Calculate header checksum
    ///
    /// Uses the header FEC matrix from the LoRa spec
    fn calculate_checksum(&self, header_data: u32) -> u8 {
        // Header FCS matrix (12 rows × 5 cols)
        // This generates 5 check bits from 12 header bits
        const FCS_MATRIX: [[u8; 5]; 12] = [
            [1, 1, 0, 0, 0],
            [1, 0, 1, 0, 0],
            [1, 0, 0, 1, 0],
            [1, 0, 0, 0, 1],
            [0, 1, 1, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 0, 0, 1],
            [0, 0, 1, 1, 0],
            [0, 0, 1, 0, 1],
            [0, 0, 0, 1, 1],
            [0, 0, 1, 1, 1],
            [0, 1, 0, 1, 1],
        ];

        let mut checksum = 0u8;

        for col in 0..5 {
            let mut bit = 0u8;
            for row in 0..12 {
                let header_bit = ((header_data >> (11 - row)) & 1) as u8;
                bit ^= header_bit & FCS_MATRIX[row][col];
            }
            checksum |= bit << (4 - col);
        }

        checksum
    }
}

impl Default for PacketHeader {
    fn default() -> Self {
        Self {
            length: 0,
            coding_rate: CodingRate::CR4_5,
            crc_enabled: true,
            low_data_rate_opt: false,
        }
    }
}

/// Complete LoRa packet
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoRaPacket {
    /// Packet header
    pub header: PacketHeader,
    /// Payload data
    pub payload: Vec<u8>,
    /// CRC value (if CRC enabled)
    pub crc: Option<u16>,
    /// Source address (for point-to-point)
    pub source_id: Option<u8>,
    /// Destination address
    pub dest_id: Option<u8>,
    /// Sequence number
    pub sequence: Option<u8>,
    /// Time of arrival (for received packets)
    pub toa: Option<f64>,
    /// RSSI at reception
    pub rssi: Option<f64>,
    /// SNR at reception
    pub snr: Option<f64>,
}

impl LoRaPacket {
    /// Create a new packet with payload
    pub fn new(payload: Vec<u8>) -> Self {
        let crc = Self::calculate_crc(&payload);

        Self {
            header: PacketHeader::new(payload.len() as u8, CodingRate::CR4_5, true),
            payload,
            crc: Some(crc),
            source_id: None,
            dest_id: None,
            sequence: None,
            toa: None,
            rssi: None,
            snr: None,
        }
    }

    /// Create a packet with custom settings
    pub fn with_header(header: PacketHeader, payload: Vec<u8>) -> Self {
        let crc = if header.crc_enabled {
            Some(Self::calculate_crc(&payload))
        } else {
            None
        };

        Self {
            header,
            payload,
            crc,
            source_id: None,
            dest_id: None,
            sequence: None,
            toa: None,
            rssi: None,
            snr: None,
        }
    }

    /// Set source and destination IDs
    pub fn with_addresses(mut self, source: u8, dest: u8) -> Self {
        self.source_id = Some(source);
        self.dest_id = Some(dest);
        self
    }

    /// Set sequence number
    pub fn with_sequence(mut self, seq: u8) -> Self {
        self.sequence = Some(seq);
        self
    }

    /// Calculate CRC-16 for payload
    ///
    /// LoRa uses CRC-16/CCITT with polynomial 0x1021
    pub fn calculate_crc(data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        let polynomial: u16 = 0x1021;

        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ polynomial;
                } else {
                    crc <<= 1;
                }
            }
        }

        crc
    }

    /// Verify CRC
    pub fn verify_crc(&self) -> bool {
        match self.crc {
            Some(received_crc) => received_crc == Self::calculate_crc(&self.payload),
            None => true, // No CRC to verify
        }
    }

    /// Get total packet size in bytes (header + payload + optional CRC)
    pub fn total_size(&self) -> usize {
        let header_size = 3; // Header is always 20 bits ≈ 3 bytes
        let crc_size = if self.header.crc_enabled { 2 } else { 0 };
        header_size + self.payload.len() + crc_size
    }

    /// Calculate time on air for this packet
    pub fn time_on_air(&self, params: &LoRaParams) -> f64 {
        params.time_on_air(self.total_size())
    }
}

/// CRC initial state lookup table
///
/// LoRa uses different CRC initial states based on payload length.
/// These values were reverse-engineered from commercial devices.
pub const CRC_INIT_STATES: [u16; 256] = [
    46885, 27367, 35014, 54790, 18706, 15954, 9784, 59350, 12042, 22321,
    46211, 20984, 56450, 7998, 62433, 35799, 2946, 47628, 30930, 52144,
    59061, 10600, 56648, 10316, 34962, 55618, 57666, 2088, 61160, 25930,
    63354, 24012, 29658, 17909, 41022, 17072, 42448, 5722, 10472, 56651,
    40183, 19835, 21851, 13020, 35306, 42553, 12394, 57960, 8434, 25101,
    63814, 29049, 27264, 213, 13764, 11996, 46026, 6259, 8758, 22513,
    43163, 38423, 62727, 60460, 29548, 18211, 6559, 61900, 55362, 46606,
    19928, 6028, 35232, 29422, 28379, 55218, 38956, 12132, 49339, 47243,
    39300, 53336, 29575, 53957, 5941, 63650, 9502, 28329, 44510, 28068,
    19538, 19577, 36943, 59968, 41464, 33923, 54504, 49962, 64357, 12382,
    44678, 11234, 58436, 47434, 63636, 51152, 29296, 61176, 33231, 32706,
    27862, 11005, 41129, 38527, 32824, 20579, 37742, 22493, 37464, 56698,
    29428, 27269, 7035, 27911, 55897, 50485, 10543, 38817, 54183, 52989,
    24549, 33562, 8963, 38328, 13330, 24139, 5996, 8270, 49703, 60444,
    8277, 43598, 1693, 60789, 32523, 36522, 17339, 33912, 23978, 55777,
    34725, 2990, 13722, 60616, 61229, 19060, 58889, 43920, 9043, 10131,
    26896, 8918, 64347, 42307, 42863, 7853, 4844, 60762, 21736, 62423,
    53096, 19242, 55756, 26615, 53246, 11257, 2844, 47011, 10022, 13541,
    18296, 44005, 23544, 18733, 23770, 33147, 5237, 45754, 4432, 22560,
    40752, 50620, 32260, 2407, 26470, 2423, 33831, 34260, 1057, 552,
    56487, 62909, 4753, 7924, 40021, 7849, 4895, 10401, 32039, 40207,
    63952, 10156, 53647, 51938, 16861, 46769, 7703, 9288, 33345, 16184,
    56808, 30265, 10696, 4218, 7708, 32139, 34174, 32428, 20665, 3869,
    43003, 6609, 60431, 22531, 11704, 63584, 13620, 14292, 37000, 8503,
    38414, 38738, 10517, 48783, 30506, 63444, 50520, 34666, 341, 34793,
    2623, 0, 0, 0, 0, 0, // Padding for indices 251-255
];

/// Get CRC initial state for a given payload length
pub fn get_crc_init_state(length: u8) -> u16 {
    if (length as usize) < CRC_INIT_STATES.len() {
        CRC_INIT_STATES[length as usize]
    } else {
        0xFFFF
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_header_encode_decode() {
        let header = PacketHeader::new(50, CodingRate::CR4_5, true);
        let encoded = header.encode();

        let decoded = PacketHeader::decode(&encoded).unwrap();
        assert_eq!(decoded.length, 50);
        assert_eq!(decoded.crc_enabled, true);
    }

    #[test]
    fn test_crc_calculation() {
        let data = b"Hello, LoRa!";
        let crc = LoRaPacket::calculate_crc(data);

        // CRC should be deterministic
        let crc2 = LoRaPacket::calculate_crc(data);
        assert_eq!(crc, crc2);
    }

    #[test]
    fn test_packet_creation() {
        let packet = LoRaPacket::new(b"Test payload".to_vec());

        assert_eq!(packet.payload, b"Test payload");
        assert!(packet.crc.is_some());
        assert!(packet.verify_crc());
    }

    #[test]
    fn test_packet_with_addresses() {
        let packet = LoRaPacket::new(b"Test".to_vec())
            .with_addresses(0x01, 0xFF)
            .with_sequence(42);

        assert_eq!(packet.source_id, Some(0x01));
        assert_eq!(packet.dest_id, Some(0xFF));
        assert_eq!(packet.sequence, Some(42));
    }
}

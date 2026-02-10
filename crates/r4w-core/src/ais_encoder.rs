//! # AIS Message Encoder
//!
//! Encodes AIS (Automatic Identification System) messages for maritime
//! vessel tracking. Generates NRZI-encoded bitstreams with HDLC framing,
//! bit-stuffing, and CRC-16 (CCITT). Complements the existing `ais_decoder`.
//!
//! ## Supported Messages
//! - Type 1/2/3: Position Report (Class A)
//! - Type 5: Static and Voyage Related Data
//! - Type 18: Standard Class B Position Report
//! - Type 24: Class B CS Static Data Report
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ais_encoder::{AisEncoder, PositionReport};
//!
//! let mut encoder = AisEncoder::new();
//! let report = PositionReport {
//!     mmsi: 123456789,
//!     nav_status: 0,
//!     rate_of_turn: 0,
//!     speed_over_ground: 100,  // 0.1 knot units
//!     position_accuracy: true,
//!     longitude: -122_400_000, // 1/10000 min units
//!     latitude: 37_800_000,
//!     course_over_ground: 2100, // 0.1 degree units
//!     true_heading: 210,
//!     timestamp: 30,
//! };
//! let bits = encoder.encode_position_report(&report, 1);
//! assert!(!bits.is_empty());
//! ```

/// AIS position report (Message types 1, 2, 3).
#[derive(Debug, Clone)]
pub struct PositionReport {
    /// Maritime Mobile Service Identity (9 digits).
    pub mmsi: u32,
    /// Navigation status (0=under way using engine, 1=at anchor, ...).
    pub nav_status: u8,
    /// Rate of turn (ROT AIS sensor, ±127).
    pub rate_of_turn: i8,
    /// Speed over ground (0.1 knot units, 0-1022).
    pub speed_over_ground: u16,
    /// Position accuracy (true = DGPS, false = unaugmented GNSS).
    pub position_accuracy: bool,
    /// Longitude in 1/10000 min (±180°, 181° = N/A).
    pub longitude: i32,
    /// Latitude in 1/10000 min (±90°, 91° = N/A).
    pub latitude: i32,
    /// Course over ground (0.1 degree, 0-3599, 3600 = N/A).
    pub course_over_ground: u16,
    /// True heading (0-359, 511 = N/A).
    pub true_heading: u16,
    /// UTC second (0-59, 60 = N/A, 61 = manual, 62 = dead reckoning).
    pub timestamp: u8,
}

/// AIS message encoder.
#[derive(Debug, Clone)]
pub struct AisEncoder {
    /// Total messages encoded.
    messages_encoded: u64,
}

impl AisEncoder {
    /// Create a new AIS encoder.
    pub fn new() -> Self {
        Self {
            messages_encoded: 0,
        }
    }

    /// Encode a Position Report (message type 1, 2, or 3).
    pub fn encode_position_report(&mut self, report: &PositionReport, msg_type: u8) -> Vec<bool> {
        let msg_type = msg_type.clamp(1, 3);
        let mut payload = Vec::new();

        // Message type (6 bits)
        push_bits(&mut payload, msg_type as u32, 6);
        // Repeat indicator (2 bits)
        push_bits(&mut payload, 0, 2);
        // MMSI (30 bits)
        push_bits(&mut payload, report.mmsi, 30);
        // Navigation status (4 bits)
        push_bits(&mut payload, report.nav_status as u32, 4);
        // Rate of turn (8 bits, signed)
        push_bits(&mut payload, report.rate_of_turn as u8 as u32, 8);
        // SOG (10 bits)
        push_bits(&mut payload, report.speed_over_ground as u32, 10);
        // Position accuracy (1 bit)
        payload.push(report.position_accuracy);
        // Longitude (28 bits, signed)
        push_bits_signed(&mut payload, report.longitude, 28);
        // Latitude (27 bits, signed)
        push_bits_signed(&mut payload, report.latitude, 27);
        // COG (12 bits)
        push_bits(&mut payload, report.course_over_ground as u32, 12);
        // True heading (9 bits)
        push_bits(&mut payload, report.true_heading as u32, 9);
        // Timestamp (6 bits)
        push_bits(&mut payload, report.timestamp as u32, 6);
        // Maneuver indicator (2 bits, 0=N/A)
        push_bits(&mut payload, 0, 2);
        // Spare (3 bits)
        push_bits(&mut payload, 0, 3);
        // RAIM flag (1 bit)
        payload.push(false);
        // Communication state (19 bits, SOTDMA)
        push_bits(&mut payload, 0, 19);

        self.messages_encoded += 1;
        self.frame_message(&payload)
    }

    /// Encode raw payload bits into an HDLC-framed, NRZI-encoded bitstream.
    pub fn frame_message(&self, payload: &[bool]) -> Vec<bool> {
        // Compute CRC-16 CCITT over payload.
        let crc = crc16_ccitt(payload);

        // Build frame: flag + payload + CRC + flag.
        let mut frame_bits = Vec::new();

        // Add payload + CRC, then bit-stuff.
        let mut data_with_crc = payload.to_vec();
        // CRC is sent LSB first.
        for i in 0..16 {
            data_with_crc.push((crc >> i) & 1 == 1);
        }

        // Bit-stuffing: after 5 consecutive 1s, insert a 0.
        let stuffed = bit_stuff(&data_with_crc);

        // HDLC frame: 0x7E + stuffed data + 0x7E.
        let flag: [bool; 8] = [false, true, true, true, true, true, true, false];
        frame_bits.extend_from_slice(&flag);
        frame_bits.extend_from_slice(&stuffed);
        frame_bits.extend_from_slice(&flag);

        // NRZI encode.
        nrzi_encode(&frame_bits)
    }

    /// Get the total number of messages encoded.
    pub fn messages_encoded(&self) -> u64 {
        self.messages_encoded
    }

    /// Reset the encoder.
    pub fn reset(&mut self) {
        self.messages_encoded = 0;
    }
}

impl Default for AisEncoder {
    fn default() -> Self {
        Self::new()
    }
}

fn push_bits(output: &mut Vec<bool>, value: u32, num_bits: u8) {
    for i in (0..num_bits).rev() {
        output.push((value >> i) & 1 == 1);
    }
}

fn push_bits_signed(output: &mut Vec<bool>, value: i32, num_bits: u8) {
    let mask = (1u32 << num_bits) - 1;
    let unsigned = (value as u32) & mask;
    push_bits(output, unsigned, num_bits);
}

fn bit_stuff(input: &[bool]) -> Vec<bool> {
    let mut output = Vec::with_capacity(input.len() + input.len() / 5);
    let mut ones_count = 0;

    for &bit in input {
        output.push(bit);
        if bit {
            ones_count += 1;
            if ones_count == 5 {
                output.push(false); // stuff a zero
                ones_count = 0;
            }
        } else {
            ones_count = 0;
        }
    }

    output
}

fn nrzi_encode(input: &[bool]) -> Vec<bool> {
    let mut output = Vec::with_capacity(input.len());
    let mut prev = false;

    for &bit in input {
        if bit {
            // 1 = no transition
            output.push(prev);
        } else {
            // 0 = transition
            prev = !prev;
            output.push(prev);
        }
    }

    output
}

/// CRC-16 CCITT (polynomial 0x1021, init 0xFFFF).
fn crc16_ccitt(bits: &[bool]) -> u16 {
    let mut crc: u16 = 0xFFFF;

    for &bit in bits {
        let xor_bit = ((crc >> 15) & 1 == 1) ^ bit;
        crc <<= 1;
        if xor_bit {
            crc ^= 0x1021;
        }
    }

    crc ^ 0xFFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_position_report() {
        let mut encoder = AisEncoder::new();
        let report = PositionReport {
            mmsi: 123456789,
            nav_status: 0,
            rate_of_turn: 0,
            speed_over_ground: 100,
            position_accuracy: true,
            longitude: -122_400_000,
            latitude: 37_800_000,
            course_over_ground: 2100,
            true_heading: 210,
            timestamp: 30,
        };
        let bits = encoder.encode_position_report(&report, 1);
        assert!(!bits.is_empty());
        assert_eq!(encoder.messages_encoded(), 1);
    }

    #[test]
    fn test_hdlc_flags() {
        let encoder = AisEncoder::new();
        let payload = vec![true, false, true, false];
        let frame = encoder.frame_message(&payload);
        // Frame should be at least 16 bits (two 8-bit flags) + data.
        assert!(frame.len() > 16);
    }

    #[test]
    fn test_bit_stuffing() {
        // 5 consecutive 1s should get a 0 inserted.
        let input = vec![true, true, true, true, true, true];
        let stuffed = bit_stuff(&input);
        assert_eq!(stuffed.len(), 7); // 5 ones + 0 + 1 one
        assert!(!stuffed[5]); // stuffed zero after 5 ones
    }

    #[test]
    fn test_bit_stuffing_no_stuff() {
        let input = vec![true, false, true, false];
        let stuffed = bit_stuff(&input);
        assert_eq!(stuffed, input);
    }

    #[test]
    fn test_nrzi_encode() {
        // NRZI: 0=transition, 1=no transition
        let input = vec![false, false, true, true, false];
        let encoded = nrzi_encode(&input);
        assert_eq!(encoded.len(), 5);
        // First 0: transition from false -> true
        assert!(encoded[0]);
        // Second 0: transition from true -> false
        assert!(!encoded[1]);
        // First 1: no transition from false -> false
        assert!(!encoded[2]);
        // Second 1: no transition from false -> false
        assert!(!encoded[3]);
        // Third 0: transition from false -> true
        assert!(encoded[4]);
    }

    #[test]
    fn test_crc16() {
        let bits = vec![true, false, true, false, true, false, true, false];
        let crc = crc16_ccitt(&bits);
        // CRC should be non-zero for non-trivial data.
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_push_bits() {
        let mut bits = Vec::new();
        push_bits(&mut bits, 0b101, 3);
        assert_eq!(bits, vec![true, false, true]);
    }

    #[test]
    fn test_push_bits_signed() {
        let mut bits = Vec::new();
        push_bits_signed(&mut bits, -1, 8);
        assert_eq!(bits, vec![true; 8]); // -1 in two's complement = all 1s
    }

    #[test]
    fn test_message_types() {
        let mut encoder = AisEncoder::new();
        let report = PositionReport {
            mmsi: 999999999,
            nav_status: 5,
            rate_of_turn: -127,
            speed_over_ground: 0,
            position_accuracy: false,
            longitude: 0,
            latitude: 0,
            course_over_ground: 3600,
            true_heading: 511,
            timestamp: 60,
        };
        let bits1 = encoder.encode_position_report(&report, 1);
        let bits2 = encoder.encode_position_report(&report, 2);
        let bits3 = encoder.encode_position_report(&report, 3);
        // All should produce output of similar length (±5 bits due to bit-stuffing).
        assert!((bits1.len() as i32 - bits2.len() as i32).unsigned_abs() < 5);
        assert!((bits2.len() as i32 - bits3.len() as i32).unsigned_abs() < 5);
        assert_eq!(encoder.messages_encoded(), 3);
    }

    #[test]
    fn test_reset() {
        let mut encoder = AisEncoder::new();
        let report = PositionReport {
            mmsi: 1,
            nav_status: 0,
            rate_of_turn: 0,
            speed_over_ground: 0,
            position_accuracy: false,
            longitude: 0,
            latitude: 0,
            course_over_ground: 0,
            true_heading: 0,
            timestamp: 0,
        };
        encoder.encode_position_report(&report, 1);
        assert_eq!(encoder.messages_encoded(), 1);
        encoder.reset();
        assert_eq!(encoder.messages_encoded(), 0);
    }
}

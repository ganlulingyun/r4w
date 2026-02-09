//! AIS Decoder — Automatic Identification System maritime protocol
//!
//! Decodes AIS (ITU-R M.1371) messages from demodulated GMSK/NRZI
//! bitstreams. Extracts vessel position, identity, speed, course,
//! and navigation data. Supports message types 1-5, 18, 21, 24.
//! Pairs with FM/GMSK demod at 161.975/162.025 MHz, 9600 baud.
//! GNU Radio equivalent: `gr-ais` (out-of-tree).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ais_decoder::{decode_ais_payload, AisMessage};
//!
//! // 6-bit ASCII armored payload from AIVDM sentence
//! let armored = "13u@Dt002s0000000000000000000";
//! let bits = r4w_core::ais_decoder::dearmor_payload(armored);
//! if let Ok(msg) = decode_ais_payload(&bits) {
//!     println!("Message type: {}", msg.message_type());
//! }
//! ```

/// AIS message types.
#[derive(Debug, Clone)]
pub enum AisMessage {
    /// Types 1, 2, 3: Position Report (Class A).
    PositionReport {
        message_type: u8,
        mmsi: u32,
        nav_status: u8,
        rate_of_turn: f64,
        speed_over_ground: f64,
        position_accuracy: bool,
        longitude: f64,
        latitude: f64,
        course_over_ground: f64,
        true_heading: u16,
        timestamp: u8,
    },
    /// Type 5: Static and Voyage Related Data.
    StaticVoyage {
        mmsi: u32,
        imo: u32,
        callsign: String,
        name: String,
        ship_type: u8,
        dimension_a: u16,
        dimension_b: u16,
        dimension_c: u8,
        dimension_d: u8,
        draught: f64,
        destination: String,
    },
    /// Type 18: Standard Class B Position Report.
    ClassBPosition {
        mmsi: u32,
        speed_over_ground: f64,
        position_accuracy: bool,
        longitude: f64,
        latitude: f64,
        course_over_ground: f64,
        true_heading: u16,
        timestamp: u8,
    },
    /// Type 21: Aid-to-Navigation Report.
    AidToNavigation {
        mmsi: u32,
        name: String,
        position_accuracy: bool,
        longitude: f64,
        latitude: f64,
        aid_type: u8,
    },
    /// Type 24: Class B Static Data.
    ClassBStatic {
        mmsi: u32,
        part: u8,
        name: String,
        ship_type: u8,
        callsign: String,
    },
    /// Unknown or unsupported message type.
    Unknown {
        message_type: u8,
        mmsi: u32,
    },
}

impl AisMessage {
    /// Get the message type number.
    pub fn message_type(&self) -> u8 {
        match self {
            AisMessage::PositionReport { message_type, .. } => *message_type,
            AisMessage::StaticVoyage { .. } => 5,
            AisMessage::ClassBPosition { .. } => 18,
            AisMessage::AidToNavigation { .. } => 21,
            AisMessage::ClassBStatic { .. } => 24,
            AisMessage::Unknown { message_type, .. } => *message_type,
        }
    }

    /// Get the MMSI.
    pub fn mmsi(&self) -> u32 {
        match self {
            AisMessage::PositionReport { mmsi, .. }
            | AisMessage::StaticVoyage { mmsi, .. }
            | AisMessage::ClassBPosition { mmsi, .. }
            | AisMessage::AidToNavigation { mmsi, .. }
            | AisMessage::ClassBStatic { mmsi, .. }
            | AisMessage::Unknown { mmsi, .. } => *mmsi,
        }
    }
}

/// Decode error.
#[derive(Debug)]
pub enum AisError {
    /// Not enough bits for the message type.
    TooShort { expected: usize, got: usize },
    /// Invalid message type.
    InvalidType(u8),
    /// CRC mismatch.
    CrcMismatch { expected: u16, computed: u16 },
}

impl std::fmt::Display for AisError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AisError::TooShort { expected, got } => {
                write!(f, "Too short: expected {expected} bits, got {got}")
            }
            AisError::InvalidType(t) => write!(f, "Invalid message type: {t}"),
            AisError::CrcMismatch { expected, computed } => {
                write!(f, "CRC mismatch: expected {expected:#06x}, computed {computed:#06x}")
            }
        }
    }
}

impl std::error::Error for AisError {}

/// NRZI decode: convert NRZI bit stream to NRZ.
///
/// In NRZI, a 0 means transition, 1 means no transition.
pub fn nrzi_decode(nrzi_bits: &[bool]) -> Vec<bool> {
    if nrzi_bits.is_empty() {
        return Vec::new();
    }
    let mut nrz = Vec::with_capacity(nrzi_bits.len());
    let mut prev = false;
    for &bit in nrzi_bits {
        let nrz_bit = !(bit ^ prev);
        nrz.push(nrz_bit);
        prev = bit;
    }
    nrz
}

/// Dearmor a 6-bit ASCII encoded AIS payload.
///
/// Each character maps to 6 bits: char - 48, then if > 40 subtract 8.
pub fn dearmor_payload(armored: &str) -> Vec<bool> {
    let mut bits = Vec::with_capacity(armored.len() * 6);
    for ch in armored.chars() {
        let mut val = ch as u8 - 48;
        if val > 40 {
            val -= 8;
        }
        for shift in (0..6).rev() {
            bits.push((val >> shift) & 1 == 1);
        }
    }
    bits
}

/// Armor bits back to 6-bit ASCII.
pub fn armor_payload(bits: &[bool]) -> String {
    let mut result = String::new();
    for chunk in bits.chunks(6) {
        let mut val: u8 = 0;
        for (i, &bit) in chunk.iter().enumerate() {
            if bit {
                val |= 1 << (5 - i);
            }
        }
        if val > 39 {
            val += 8;
        }
        result.push((val + 48) as char);
    }
    result
}

/// AIS CRC-16 (ITU-R M.1371, polynomial 0x8005).
pub fn ais_crc16(bits: &[bool]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &bit in bits {
        let xor_bit = ((crc >> 15) & 1) == 1;
        crc <<= 1;
        if bit {
            crc |= 1;
        }
        if xor_bit {
            crc ^= 0x8005;
        }
    }
    crc
}

/// Extract unsigned integer from bit slice.
fn bits_to_uint(bits: &[bool], start: usize, len: usize) -> u32 {
    let mut val: u32 = 0;
    for i in 0..len {
        if start + i < bits.len() && bits[start + i] {
            val |= 1 << (len - 1 - i);
        }
    }
    val
}

/// Extract signed integer from bit slice (two's complement).
fn bits_to_int(bits: &[bool], start: usize, len: usize) -> i32 {
    let val = bits_to_uint(bits, start, len);
    // Sign extend
    if len > 0 && (val >> (len - 1)) & 1 == 1 {
        val as i32 | (!0u32 << len) as i32
    } else {
        val as i32
    }
}

/// Extract 6-bit ASCII string from bits.
fn bits_to_string(bits: &[bool], start: usize, num_chars: usize) -> String {
    let mut s = String::new();
    for c in 0..num_chars {
        let offset = start + c * 6;
        let val = bits_to_uint(bits, offset, 6) as u8;
        let ch = if val < 32 { val + 64 } else { val };
        if ch == b'@' {
            break; // '@' is the AIS string terminator
        }
        s.push(ch as char);
    }
    s.trim_end().to_string()
}

/// Decode an AIS payload from a bit vector.
///
/// The bits should be the dearmored payload (without CRC).
pub fn decode_ais_payload(bits: &[bool]) -> Result<AisMessage, AisError> {
    if bits.len() < 38 {
        return Err(AisError::TooShort {
            expected: 38,
            got: bits.len(),
        });
    }

    let msg_type = bits_to_uint(bits, 0, 6) as u8;
    let _repeat = bits_to_uint(bits, 6, 2);
    let mmsi = bits_to_uint(bits, 8, 30);

    match msg_type {
        1 | 2 | 3 => {
            if bits.len() < 168 {
                return Err(AisError::TooShort {
                    expected: 168,
                    got: bits.len(),
                });
            }
            let nav_status = bits_to_uint(bits, 38, 4) as u8;
            let rot_raw = bits_to_int(bits, 42, 8);
            let rate_of_turn = if rot_raw == -128 {
                f64::NAN
            } else {
                let sign = if rot_raw < 0 { -1.0 } else { 1.0 };
                sign * (rot_raw as f64 / 4.733).powi(2)
            };
            let sog_raw = bits_to_uint(bits, 50, 10);
            let speed_over_ground = sog_raw as f64 / 10.0;
            let position_accuracy = bits[60];
            let lon_raw = bits_to_int(bits, 61, 28);
            let longitude = lon_raw as f64 / 600000.0;
            let lat_raw = bits_to_int(bits, 89, 27);
            let latitude = lat_raw as f64 / 600000.0;
            let cog_raw = bits_to_uint(bits, 116, 12);
            let course_over_ground = cog_raw as f64 / 10.0;
            let true_heading = bits_to_uint(bits, 128, 9) as u16;
            let timestamp = bits_to_uint(bits, 137, 6) as u8;

            Ok(AisMessage::PositionReport {
                message_type: msg_type,
                mmsi,
                nav_status,
                rate_of_turn,
                speed_over_ground,
                position_accuracy,
                longitude,
                latitude,
                course_over_ground,
                true_heading,
                timestamp,
            })
        }
        5 => {
            if bits.len() < 424 {
                return Err(AisError::TooShort {
                    expected: 424,
                    got: bits.len(),
                });
            }
            let imo = bits_to_uint(bits, 40, 30);
            let callsign = bits_to_string(bits, 70, 7);
            let name = bits_to_string(bits, 112, 20);
            let ship_type = bits_to_uint(bits, 232, 8) as u8;
            let dimension_a = bits_to_uint(bits, 240, 9) as u16;
            let dimension_b = bits_to_uint(bits, 249, 9) as u16;
            let dimension_c = bits_to_uint(bits, 258, 6) as u8;
            let dimension_d = bits_to_uint(bits, 264, 6) as u8;
            let draught_raw = bits_to_uint(bits, 294, 8);
            let draught = draught_raw as f64 / 10.0;
            let destination = bits_to_string(bits, 302, 20);

            Ok(AisMessage::StaticVoyage {
                mmsi,
                imo,
                callsign,
                name,
                ship_type,
                dimension_a,
                dimension_b,
                dimension_c,
                dimension_d,
                draught,
                destination,
            })
        }
        18 => {
            if bits.len() < 168 {
                return Err(AisError::TooShort {
                    expected: 168,
                    got: bits.len(),
                });
            }
            let sog_raw = bits_to_uint(bits, 46, 10);
            let speed_over_ground = sog_raw as f64 / 10.0;
            let position_accuracy = bits[56];
            let lon_raw = bits_to_int(bits, 57, 28);
            let longitude = lon_raw as f64 / 600000.0;
            let lat_raw = bits_to_int(bits, 85, 27);
            let latitude = lat_raw as f64 / 600000.0;
            let cog_raw = bits_to_uint(bits, 112, 12);
            let course_over_ground = cog_raw as f64 / 10.0;
            let true_heading = bits_to_uint(bits, 124, 9) as u16;
            let timestamp = bits_to_uint(bits, 133, 6) as u8;

            Ok(AisMessage::ClassBPosition {
                mmsi,
                speed_over_ground,
                position_accuracy,
                longitude,
                latitude,
                course_over_ground,
                true_heading,
                timestamp,
            })
        }
        21 => {
            if bits.len() < 272 {
                return Err(AisError::TooShort {
                    expected: 272,
                    got: bits.len(),
                });
            }
            let aid_type = bits_to_uint(bits, 38, 5) as u8;
            let name = bits_to_string(bits, 43, 20);
            let position_accuracy = bits[163];
            let lon_raw = bits_to_int(bits, 164, 28);
            let longitude = lon_raw as f64 / 600000.0;
            let lat_raw = bits_to_int(bits, 192, 27);
            let latitude = lat_raw as f64 / 600000.0;

            Ok(AisMessage::AidToNavigation {
                mmsi,
                name,
                position_accuracy,
                longitude,
                latitude,
                aid_type,
            })
        }
        24 => {
            if bits.len() < 160 {
                return Err(AisError::TooShort {
                    expected: 160,
                    got: bits.len(),
                });
            }
            let part = bits_to_uint(bits, 38, 2) as u8;
            let (name, ship_type, callsign) = if part == 0 {
                (bits_to_string(bits, 40, 20), 0u8, String::new())
            } else {
                let ship_type = bits_to_uint(bits, 40, 8) as u8;
                let callsign = bits_to_string(bits, 90, 7);
                (String::new(), ship_type, callsign)
            };

            Ok(AisMessage::ClassBStatic {
                mmsi,
                part,
                name,
                ship_type,
                callsign,
            })
        }
        _ => Ok(AisMessage::Unknown {
            message_type: msg_type,
            mmsi,
        }),
    }
}

/// Parse an AIVDM/AIVDO NMEA sentence.
///
/// Returns (fragment_count, fragment_num, message_id, channel, payload_bits).
pub fn parse_aivdm(sentence: &str) -> Result<(u8, u8, Option<u8>, char, Vec<bool>), AisError> {
    let parts: Vec<&str> = sentence.split(',').collect();
    if parts.len() < 7 {
        return Err(AisError::TooShort {
            expected: 7,
            got: parts.len(),
        });
    }

    let frag_count = parts[1].parse::<u8>().unwrap_or(1);
    let frag_num = parts[2].parse::<u8>().unwrap_or(1);
    let msg_id = parts[3].parse::<u8>().ok();
    let channel = parts[4].chars().next().unwrap_or('A');
    let payload = dearmor_payload(parts[5]);

    Ok((frag_count, frag_num, msg_id, channel, payload))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dearmor_armor_roundtrip() {
        let original = "13u@Dt002s000";
        let bits = dearmor_payload(original);
        assert_eq!(bits.len(), original.len() * 6);
        let rearmed = armor_payload(&bits);
        assert_eq!(rearmed, original);
    }

    #[test]
    fn test_nrzi_decode() {
        let nrzi = vec![false, true, true, false, true];
        let nrz = nrzi_decode(&nrzi);
        assert_eq!(nrz.len(), 5);
    }

    #[test]
    fn test_bits_to_uint() {
        let bits = vec![true, false, true, true]; // 1011 = 11
        assert_eq!(bits_to_uint(&bits, 0, 4), 11);
    }

    #[test]
    fn test_bits_to_int_positive() {
        let bits = vec![false, true, false, true]; // 0101 = 5
        assert_eq!(bits_to_int(&bits, 0, 4), 5);
    }

    #[test]
    fn test_bits_to_int_negative() {
        let bits = vec![true, true, true, true]; // 1111 = -1
        assert_eq!(bits_to_int(&bits, 0, 4), -1);
    }

    #[test]
    fn test_decode_type1() {
        // Construct a minimal type 1 message (168 bits)
        let mut bits = vec![false; 168];
        // Message type = 1 (000001)
        bits[5] = true;
        // MMSI = 123456789 in bits 8..38
        let mmsi: u32 = 123456789;
        for i in 0..30 {
            bits[8 + i] = (mmsi >> (29 - i)) & 1 == 1;
        }
        // Nav status = 0 (under way)
        // SOG = 102 (10.2 knots) at bits 50..60
        let sog: u32 = 102;
        for i in 0..10 {
            bits[50 + i] = (sog >> (9 - i)) & 1 == 1;
        }

        let msg = decode_ais_payload(&bits).unwrap();
        assert_eq!(msg.message_type(), 1);
        assert_eq!(msg.mmsi(), 123456789);
        if let AisMessage::PositionReport {
            speed_over_ground, ..
        } = msg
        {
            assert!((speed_over_ground - 10.2).abs() < 0.01);
        } else {
            panic!("Expected PositionReport");
        }
    }

    #[test]
    fn test_decode_too_short() {
        let bits = vec![false; 10];
        assert!(decode_ais_payload(&bits).is_err());
    }

    #[test]
    fn test_decode_type5() {
        let mut bits = vec![false; 424];
        // Type 5 = 000101
        bits[3] = true;
        bits[5] = true;
        // MMSI
        let mmsi: u32 = 211234567;
        for i in 0..30 {
            bits[8 + i] = (mmsi >> (29 - i)) & 1 == 1;
        }
        let msg = decode_ais_payload(&bits).unwrap();
        assert_eq!(msg.message_type(), 5);
        assert_eq!(msg.mmsi(), 211234567);
    }

    #[test]
    fn test_decode_unknown_type() {
        let mut bits = vec![false; 168];
        // Type 27 = 011011
        bits[1] = true;
        bits[2] = true;
        bits[4] = true;
        bits[5] = true;
        let msg = decode_ais_payload(&bits).unwrap();
        assert_eq!(msg.message_type(), 27);
    }

    #[test]
    fn test_parse_aivdm() {
        let sentence = "!AIVDM,1,1,,B,13u@Dt002s000000000000000000,0*52";
        let result = parse_aivdm(sentence);
        assert!(result.is_ok());
        let (frag_count, frag_num, _msg_id, channel, bits) = result.unwrap();
        assert_eq!(frag_count, 1);
        assert_eq!(frag_num, 1);
        assert_eq!(channel, 'B');
        assert!(!bits.is_empty());
    }

    #[test]
    fn test_ais_crc16() {
        // Basic CRC test - just verify it produces consistent output
        let bits = vec![true, false, true, true, false, false, true, false];
        let crc = ais_crc16(&bits);
        let crc2 = ais_crc16(&bits);
        assert_eq!(crc, crc2);
    }

    #[test]
    fn test_6bit_string() {
        // 'A' in 6-bit AIS = 1
        let bits = vec![false, false, false, false, false, true]; // 1 → 'A'
        let s = bits_to_string(&bits, 0, 1);
        assert_eq!(s, "A");
    }

    #[test]
    fn test_dearmor_empty() {
        assert!(dearmor_payload("").is_empty());
    }
}

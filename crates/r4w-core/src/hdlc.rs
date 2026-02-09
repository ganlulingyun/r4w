//! HDLC Framing and Deframing
//!
//! High-Level Data Link Control (HDLC) frame encoding and decoding with:
//! - Flag bytes (0x7E) for frame delimiting
//! - Bit-stuffing (insert 0 after five consecutive 1s)
//! - CRC-16/CCITT Frame Check Sequence (FCS)
//!
//! Used by AX.25 (amateur packet radio), APRS, and many satellite protocols.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::hdlc::{HdlcFramer, HdlcDeframer};
//!
//! let framer = HdlcFramer::new();
//! let data = b"Hello AX.25!";
//! let frame_bits = framer.frame(data);
//!
//! let mut deframer = HdlcDeframer::new();
//! let frames = deframer.process_bits(&frame_bits);
//! assert_eq!(frames.len(), 1);
//! assert_eq!(frames[0], data.to_vec());
//! ```

/// HDLC flag byte (0x7E = 01111110)
const HDLC_FLAG: u8 = 0x7E;

/// CRC-16/CCITT polynomial (reflected)
const CRC16_CCITT_POLY: u16 = 0x8408;

/// Compute CRC-16/CCITT (used as FCS in HDLC/AX.25).
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ CRC16_CCITT_POLY;
            } else {
                crc >>= 1;
            }
        }
    }
    crc ^ 0xFFFF
}

/// HDLC frame encoder.
///
/// Encodes data into HDLC frames with flag delimiters, bit-stuffing, and FCS.
#[derive(Debug, Clone)]
pub struct HdlcFramer {
    /// Number of preamble flags to send
    preamble_flags: usize,
    /// Number of postamble flags to send
    postamble_flags: usize,
}

impl HdlcFramer {
    /// Create a new HDLC framer with default 3 preamble + 2 postamble flags.
    pub fn new() -> Self {
        Self {
            preamble_flags: 3,
            postamble_flags: 2,
        }
    }

    /// Create with custom flag counts.
    pub fn with_flags(preamble: usize, postamble: usize) -> Self {
        Self {
            preamble_flags: preamble,
            postamble_flags: postamble,
        }
    }

    /// Frame data into HDLC bit stream.
    /// Returns a vector of bool representing the HDLC frame bits.
    pub fn frame(&self, data: &[u8]) -> Vec<bool> {
        let mut bits = Vec::new();

        // Preamble flags
        for _ in 0..self.preamble_flags {
            Self::push_byte(&mut bits, HDLC_FLAG);
        }

        // Compute FCS
        let fcs = crc16_ccitt(data);

        // Bit-stuff and send data
        let mut ones_count = 0;
        for &byte in data {
            for bit_idx in 0..8 {
                let bit = (byte >> bit_idx) & 1 != 0; // LSB first
                bits.push(bit);
                if bit {
                    ones_count += 1;
                    if ones_count == 5 {
                        bits.push(false); // Bit stuff
                        ones_count = 0;
                    }
                } else {
                    ones_count = 0;
                }
            }
        }

        // FCS (LSB first, bit-stuffed)
        for byte_idx in 0..2 {
            let fcs_byte = if byte_idx == 0 { fcs as u8 } else { (fcs >> 8) as u8 };
            for bit_idx in 0..8 {
                let bit = (fcs_byte >> bit_idx) & 1 != 0;
                bits.push(bit);
                if bit {
                    ones_count += 1;
                    if ones_count == 5 {
                        bits.push(false);
                        ones_count = 0;
                    }
                } else {
                    ones_count = 0;
                }
            }
        }

        // Postamble flags
        for _ in 0..self.postamble_flags {
            Self::push_byte(&mut bits, HDLC_FLAG);
        }

        bits
    }

    /// Push a byte as 8 bits (LSB first, no bit-stuffing).
    fn push_byte(bits: &mut Vec<bool>, byte: u8) {
        for i in 0..8 {
            bits.push((byte >> i) & 1 != 0);
        }
    }
}

impl Default for HdlcFramer {
    fn default() -> Self {
        Self::new()
    }
}

/// HDLC frame decoder.
///
/// Processes a bit stream and extracts HDLC frames with bit-unstuffing
/// and FCS verification. Uses a shift register for flag detection and
/// buffers raw bits between flags for post-processing.
#[derive(Debug, Clone)]
pub struct HdlcDeframer {
    /// Shift register for flag detection (last 8 raw bits)
    shift_reg: u8,
    /// Raw bits collected between flags
    raw_bits: Vec<bool>,
    /// Whether we've seen at least one flag
    saw_flag: bool,
    /// Number of raw bits since last flag
    bits_since_flag: usize,
    /// Total bits processed
    bit_count: usize,
}

impl HdlcDeframer {
    /// Create a new HDLC deframer.
    pub fn new() -> Self {
        Self {
            shift_reg: 0,
            raw_bits: Vec::new(),
            saw_flag: false,
            bits_since_flag: 0,
            bit_count: 0,
        }
    }

    /// Process a stream of bits and return decoded frames.
    pub fn process_bits(&mut self, bits: &[bool]) -> Vec<Vec<u8>> {
        let mut frames = Vec::new();

        for &bit in bits {
            self.bit_count += 1;

            // Shift the bit into the shift register
            self.shift_reg = (self.shift_reg >> 1) | if bit { 0x80 } else { 0 };

            // Check for flag (0x7E = 01111110)
            if self.shift_reg == HDLC_FLAG {
                if self.saw_flag && self.bits_since_flag > 0 {
                    // We have raw bits between two flags — try to decode
                    // Remove the last 7 bits from raw_bits (they're part of this flag)
                    let keep = self.raw_bits.len().saturating_sub(7);
                    self.raw_bits.truncate(keep);
                    if let Some(data) = Self::decode_raw_bits(&self.raw_bits) {
                        frames.push(data);
                    }
                }
                self.raw_bits.clear();
                self.saw_flag = true;
                self.bits_since_flag = 0;
                continue;
            }

            if self.saw_flag {
                self.raw_bits.push(bit);
                self.bits_since_flag += 1;
            }
        }

        frames
    }

    /// Decode raw bits (between flags) by unstuffing and verifying FCS.
    fn decode_raw_bits(raw: &[bool]) -> Option<Vec<u8>> {
        if raw.is_empty() {
            // Empty frame with valid FCS is possible (0-byte payload)
        }

        // Bit-unstuff: remove 0 after five consecutive 1s
        let mut unstuffed = Vec::with_capacity(raw.len());
        let mut ones_count = 0u32;
        for &bit in raw {
            if bit {
                ones_count += 1;
                unstuffed.push(true);
                if ones_count > 5 {
                    // Too many consecutive 1s without stuffing — abort
                    return None;
                }
            } else {
                if ones_count == 5 {
                    // This is a stuff bit — discard
                    ones_count = 0;
                    continue;
                }
                unstuffed.push(false);
                ones_count = 0;
            }
        }

        // Must be a multiple of 8 bits
        if unstuffed.len() % 8 != 0 || unstuffed.len() < 16 {
            return None;
        }

        // Convert bits to bytes (LSB first)
        let mut bytes = Vec::new();
        for chunk in unstuffed.chunks(8) {
            let mut byte = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                if bit {
                    byte |= 1 << i;
                }
            }
            bytes.push(byte);
        }

        if bytes.len() < 2 {
            return None;
        }

        // Last 2 bytes are FCS
        let data_len = bytes.len() - 2;
        let data = &bytes[..data_len];
        let received_fcs = bytes[data_len] as u16 | ((bytes[data_len + 1] as u16) << 8);
        let computed_fcs = crc16_ccitt(data);

        if received_fcs == computed_fcs {
            Some(data.to_vec())
        } else {
            None // FCS mismatch
        }
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.shift_reg = 0;
        self.raw_bits.clear();
        self.saw_flag = false;
        self.bits_since_flag = 0;
        self.bit_count = 0;
    }
}

impl Default for HdlcDeframer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_ccitt() {
        // Known test vector: "123456789" → 0x906E
        let data = b"123456789";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, 0x906E);
    }

    #[test]
    fn test_frame_deframe_roundtrip() {
        let framer = HdlcFramer::new();
        let mut deframer = HdlcDeframer::new();

        let data = b"Hello HDLC!";
        let bits = framer.frame(data);
        let frames = deframer.process_bits(&bits);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], data.to_vec());
    }

    #[test]
    fn test_empty_payload() {
        let framer = HdlcFramer::new();
        let mut deframer = HdlcDeframer::new();

        let data: &[u8] = &[];
        let bits = framer.frame(data);
        let frames = deframer.process_bits(&bits);

        assert_eq!(frames.len(), 1);
        assert!(frames[0].is_empty());
    }

    #[test]
    fn test_binary_payload() {
        let framer = HdlcFramer::new();
        let mut deframer = HdlcDeframer::new();

        let data = vec![0x00, 0xFF, 0x55, 0xAA, 0x7E, 0x7D]; // Include flag-like bytes
        let bits = framer.frame(&data);
        let frames = deframer.process_bits(&bits);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], data);
    }

    #[test]
    fn test_all_ones_payload() {
        let framer = HdlcFramer::new();
        let mut deframer = HdlcDeframer::new();

        let data = vec![0xFF; 10]; // Forces maximum bit-stuffing
        let bits = framer.frame(&data);
        let frames = deframer.process_bits(&bits);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], data);
    }

    #[test]
    fn test_multiple_frames() {
        let framer = HdlcFramer::new();
        let mut deframer = HdlcDeframer::new();

        let data1 = b"Frame 1";
        let data2 = b"Frame 2";

        let mut bits = framer.frame(data1);
        bits.extend(framer.frame(data2));

        let frames = deframer.process_bits(&bits);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], data1.to_vec());
        assert_eq!(frames[1], data2.to_vec());
    }

    #[test]
    fn test_flag_count() {
        let framer = HdlcFramer::with_flags(1, 1);
        let data = b"Test";
        let bits = framer.frame(data);
        // Should have 2 flags (1 pre + 1 post) = 16 bits of flags
        // Plus data + FCS + possible bit stuffing
        assert!(bits.len() >= 16 + (4 + 2) * 8);
    }

    #[test]
    fn test_corrupted_fcs() {
        let framer = HdlcFramer::new();
        let mut deframer = HdlcDeframer::new();

        let data = b"Hello";
        let mut bits = framer.frame(data);
        // Corrupt a data bit (not in the flags)
        let mid = bits.len() / 2;
        bits[mid] = !bits[mid];

        let frames = deframer.process_bits(&bits);
        // Should not decode (FCS mismatch)
        assert!(frames.is_empty());
    }

    #[test]
    fn test_reset() {
        let mut deframer = HdlcDeframer::new();
        let framer = HdlcFramer::new();
        let bits = framer.frame(b"test");
        deframer.process_bits(&bits);
        deframer.reset();
        assert!(!deframer.saw_flag);
        assert!(deframer.raw_bits.is_empty());
    }

    #[test]
    fn test_ax25_like_payload() {
        // Simulate an AX.25-like payload
        let framer = HdlcFramer::new();
        let mut deframer = HdlcDeframer::new();

        // AX.25 address field (14 bytes) + control + PID + info
        let mut payload = Vec::new();
        // Destination: "APRS  " (shifted left 1 bit)
        payload.extend_from_slice(&[0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0x60]);
        // Source: "N0CALL" (shifted left 1 bit)
        payload.extend_from_slice(&[0x9C, 0x60, 0x86, 0x82, 0x98, 0x98, 0xE1]);
        // Control: UI frame
        payload.push(0x03);
        // PID: No layer 3
        payload.push(0xF0);
        // Info
        payload.extend_from_slice(b"!4903.50N/07201.75W-Test");

        let bits = framer.frame(&payload);
        let frames = deframer.process_bits(&bits);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], payload);
    }
}

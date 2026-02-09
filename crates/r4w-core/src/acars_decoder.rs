//! ACARS Decoder — Aircraft Communications Addressing and Reporting System
//!
//! Decodes ACARS messages from AM-demodulated audio on 131.550 MHz.
//! Extracts aircraft registration, flight ID, message type, and text.
//! Uses 2400 baud AM-MSK modulation with CRC-CCITT validation.
//! GNU Radio equivalent: `gr-acars2` (out-of-tree) / `acarsdec`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::acars_decoder::{AcarsDecoder, AcarsMessage};
//!
//! let mut decoder = AcarsDecoder::new();
//! // In practice, feed demodulated bits from 131.550 MHz
//! assert_eq!(decoder.messages().len(), 0);
//! ```

/// ACARS message structure.
#[derive(Debug, Clone)]
pub struct AcarsMessage {
    /// Mode character (typically '2' for VHF ACARS).
    pub mode: char,
    /// Aircraft registration (7 characters, e.g., ".N12345").
    pub aircraft_reg: String,
    /// Acknowledgment character.
    pub ack: char,
    /// 2-character message label (e.g., "H1", "5Z").
    pub label: String,
    /// Block identifier.
    pub block_id: char,
    /// Flight ID / message sequence number.
    pub flight_id: String,
    /// Message text body.
    pub text: String,
    /// Whether CRC was valid.
    pub crc_valid: bool,
}

/// ACARS decoder error.
#[derive(Debug)]
pub enum AcarsError {
    /// Frame too short.
    TooShort,
    /// Invalid character in frame.
    InvalidChar,
    /// CRC mismatch.
    CrcMismatch,
    /// No SYN/SOH found.
    NoSync,
}

impl std::fmt::Display for AcarsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AcarsError::TooShort => write!(f, "Frame too short"),
            AcarsError::InvalidChar => write!(f, "Invalid character"),
            AcarsError::CrcMismatch => write!(f, "CRC mismatch"),
            AcarsError::NoSync => write!(f, "No sync found"),
        }
    }
}

impl std::error::Error for AcarsError {}

/// ACARS special characters.
const SYN: u8 = 0x16;
const SOH: u8 = 0x01;
const STX: u8 = 0x02;
const ETX: u8 = 0x03;
const ETB: u8 = 0x17;
const DEL: u8 = 0x7F;

/// ACARS decoder with bit-level processing.
#[derive(Debug, Clone)]
pub struct AcarsDecoder {
    /// Bit buffer.
    bit_buffer: Vec<bool>,
    /// Decoded messages.
    messages: Vec<AcarsMessage>,
    /// Frames processed.
    frame_count: usize,
    /// CRC errors.
    crc_errors: usize,
}

impl AcarsDecoder {
    /// Create a new ACARS decoder.
    pub fn new() -> Self {
        Self {
            bit_buffer: Vec::new(),
            messages: Vec::new(),
            frame_count: 0,
            crc_errors: 0,
        }
    }

    /// Feed raw bits from demodulator.
    pub fn feed_bits(&mut self, bits: &[bool]) {
        self.bit_buffer.extend_from_slice(bits);
        self.process_buffer();
    }

    /// Decode a frame from raw bytes.
    pub fn decode_frame(&mut self, frame: &[u8]) -> Result<AcarsMessage, AcarsError> {
        // Minimum frame: SOH + mode + reg(7) + ack + label(2) + block_id + ETX/ETB + BCS(2)
        if frame.len() < 16 {
            return Err(AcarsError::TooShort);
        }

        // Find SOH
        let soh_pos = frame.iter().position(|&b| b == SOH);
        let start = match soh_pos {
            Some(pos) => pos + 1,
            None => return Err(AcarsError::NoSync),
        };

        if frame.len() < start + 13 {
            return Err(AcarsError::TooShort);
        }

        let mode = frame[start] as char;
        let aircraft_reg: String = frame[start + 1..start + 8]
            .iter()
            .map(|&b| (b & 0x7F) as char)
            .collect();
        let ack = frame[start + 8] as char;
        let label: String = frame[start + 9..start + 11]
            .iter()
            .map(|&b| (b & 0x7F) as char)
            .collect();
        let block_id = frame[start + 11] as char;

        // Find text between STX and ETX/ETB
        let text_start = frame[start + 12..]
            .iter()
            .position(|&b| b == STX)
            .map(|p| start + 12 + p + 1);

        let end_marker = frame
            .iter()
            .rposition(|&b| b == ETX || b == ETB)
            .unwrap_or(frame.len());

        let text = if let Some(ts) = text_start {
            if ts < end_marker {
                frame[ts..end_marker]
                    .iter()
                    .map(|&b| (b & 0x7F) as char)
                    .collect()
            } else {
                String::new()
            }
        } else {
            String::new()
        };

        // Flight ID (in text header or label-dependent)
        let flight_id = if text.len() >= 6 {
            text[..6].trim().to_string()
        } else {
            String::new()
        };

        // CRC validation
        let crc_valid = if frame.len() >= end_marker + 3 {
            let computed = crc_ccitt(&frame[..end_marker + 1]);
            let received = ((frame[end_marker + 1] as u16) << 8) | frame[end_marker + 2] as u16;
            computed == received
        } else {
            false
        };

        if !crc_valid {
            self.crc_errors += 1;
        }

        self.frame_count += 1;
        let msg = AcarsMessage {
            mode,
            aircraft_reg: aircraft_reg.trim().to_string(),
            ack,
            label,
            block_id,
            flight_id,
            text,
            crc_valid,
        };
        self.messages.push(msg.clone());
        Ok(msg)
    }

    /// Process the bit buffer looking for frames.
    fn process_buffer(&mut self) {
        // Convert bits to bytes and look for SYN patterns
        while self.bit_buffer.len() >= 8 * 20 {
            // Look for preamble (SYN SYN SYN SOH)
            let sync_pattern = [SYN, SYN, SOH];
            if let Some(pos) = self.find_byte_pattern(&sync_pattern) {
                // Extract frame bytes starting from SOH
                let byte_start = pos + 2; // Skip two SYN bytes
                let remaining_bits = self.bit_buffer.len() - byte_start * 8;
                if remaining_bits < 13 * 8 {
                    break; // Need more data
                }

                // Find ETX or ETB
                let max_frame_len = (remaining_bits / 8).min(256);
                let mut frame_bytes = Vec::new();
                for i in 0..max_frame_len {
                    let byte_offset = (byte_start + i) * 8;
                    if byte_offset + 8 > self.bit_buffer.len() {
                        break;
                    }
                    let byte = bits_to_byte(&self.bit_buffer[byte_offset..byte_offset + 8]);
                    frame_bytes.push(byte);
                    if byte == ETX || byte == ETB {
                        // Include 2 more bytes for BCS
                        for j in 1..=2 {
                            let bo = (byte_start + i + j) * 8;
                            if bo + 8 <= self.bit_buffer.len() {
                                frame_bytes.push(bits_to_byte(&self.bit_buffer[bo..bo + 8]));
                            }
                        }
                        break;
                    }
                }

                let _ = self.decode_frame(&frame_bytes);
                // Remove processed bits
                let drain_to = ((byte_start + frame_bytes.len()) * 8).min(self.bit_buffer.len());
                self.bit_buffer.drain(..drain_to);
            } else {
                // No sync found, discard oldest byte
                if self.bit_buffer.len() >= 8 {
                    self.bit_buffer.drain(..8);
                } else {
                    break;
                }
            }
        }
    }

    /// Find a byte pattern in the bit buffer.
    fn find_byte_pattern(&self, pattern: &[u8]) -> Option<usize> {
        let pat_bits = pattern.len() * 8;
        if self.bit_buffer.len() < pat_bits {
            return None;
        }
        'outer: for byte_offset in 0..=(self.bit_buffer.len() / 8 - pattern.len()) {
            for (i, &expected) in pattern.iter().enumerate() {
                let bit_offset = (byte_offset + i) * 8;
                let byte = bits_to_byte(&self.bit_buffer[bit_offset..bit_offset + 8]);
                if byte != expected {
                    continue 'outer;
                }
            }
            return Some(byte_offset);
        }
        None
    }

    /// Get all decoded messages.
    pub fn messages(&self) -> &[AcarsMessage] {
        &self.messages
    }

    /// Frames processed.
    pub fn frame_count(&self) -> usize {
        self.frame_count
    }

    /// CRC errors encountered.
    pub fn crc_errors(&self) -> usize {
        self.crc_errors
    }

    /// Reset the decoder.
    pub fn reset(&mut self) {
        self.bit_buffer.clear();
        self.messages.clear();
        self.frame_count = 0;
        self.crc_errors = 0;
    }
}

impl Default for AcarsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC-CCITT (0xFFFF initial, polynomial 0x1021).
pub fn crc_ccitt(data: &[u8]) -> u16 {
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

/// Convert 8 bits to a byte (MSB first).
fn bits_to_byte(bits: &[bool]) -> u8 {
    let mut byte: u8 = 0;
    for (i, &bit) in bits.iter().enumerate().take(8) {
        if bit {
            byte |= 1 << (7 - i);
        }
    }
    byte
}

/// Parse ACARS pre-assembled text (as seen from `acarsdec` output).
pub fn parse_acars_text(text: &str) -> Option<AcarsMessage> {
    // Simple parser for "REG LABEL TEXT" format
    let parts: Vec<&str> = text.splitn(3, ' ').collect();
    if parts.len() < 2 {
        return None;
    }
    Some(AcarsMessage {
        mode: '2',
        aircraft_reg: parts[0].to_string(),
        ack: ' ',
        label: if parts.len() > 1 {
            parts[1].to_string()
        } else {
            String::new()
        },
        block_id: ' ',
        flight_id: String::new(),
        text: if parts.len() > 2 {
            parts[2].to_string()
        } else {
            String::new()
        },
        crc_valid: true,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc_ccitt() {
        // Known CRC-CCITT test vector: "123456789" → 0x29B1
        let data = b"123456789";
        let crc = crc_ccitt(data);
        assert_eq!(crc, 0x29B1);
    }

    #[test]
    fn test_bits_to_byte() {
        let bits = [true, false, true, false, true, false, true, false]; // 0xAA
        assert_eq!(bits_to_byte(&bits), 0xAA);

        let bits = [false; 8]; // 0x00
        assert_eq!(bits_to_byte(&bits), 0x00);

        let bits = [true; 8]; // 0xFF
        assert_eq!(bits_to_byte(&bits), 0xFF);
    }

    #[test]
    fn test_decode_frame() {
        let mut decoder = AcarsDecoder::new();
        // Construct a minimal frame: SOH + mode + reg(7) + ack + label(2) + block_id + STX + text + ETX + BCS(2)
        let mut frame = vec![SOH];
        frame.push(b'2'); // mode
        frame.extend_from_slice(b".N12345"); // reg
        frame.push(b' '); // ack
        frame.extend_from_slice(b"H1"); // label
        frame.push(b'1'); // block_id
        frame.push(STX);
        frame.extend_from_slice(b"TEST MESSAGE");
        frame.push(ETX);
        // Compute CRC
        let crc = crc_ccitt(&frame);
        frame.push((crc >> 8) as u8);
        frame.push((crc & 0xFF) as u8);

        let result = decoder.decode_frame(&frame);
        assert!(result.is_ok());
        let msg = result.unwrap();
        assert_eq!(msg.mode, '2');
        assert_eq!(msg.aircraft_reg, ".N12345");
        assert_eq!(msg.label, "H1");
        assert!(msg.crc_valid);
    }

    #[test]
    fn test_decode_frame_too_short() {
        let mut decoder = AcarsDecoder::new();
        let frame = vec![SOH, b'2'];
        assert!(decoder.decode_frame(&frame).is_err());
    }

    #[test]
    fn test_decode_frame_no_soh() {
        let mut decoder = AcarsDecoder::new();
        let frame = vec![0u8; 20];
        assert!(decoder.decode_frame(&frame).is_err());
    }

    #[test]
    fn test_parse_text() {
        let msg = parse_acars_text("N12345 H1 HELLO WORLD");
        assert!(msg.is_some());
        let msg = msg.unwrap();
        assert_eq!(msg.aircraft_reg, "N12345");
        assert_eq!(msg.label, "H1");
        assert_eq!(msg.text, "HELLO WORLD");
    }

    #[test]
    fn test_decoder_creation() {
        let decoder = AcarsDecoder::new();
        assert_eq!(decoder.frame_count(), 0);
        assert_eq!(decoder.crc_errors(), 0);
        assert!(decoder.messages().is_empty());
    }

    #[test]
    fn test_reset() {
        let mut decoder = AcarsDecoder::new();
        decoder.frame_count = 5;
        decoder.crc_errors = 2;
        decoder.reset();
        assert_eq!(decoder.frame_count(), 0);
        assert_eq!(decoder.crc_errors(), 0);
    }

    #[test]
    fn test_default() {
        let decoder = AcarsDecoder::default();
        assert_eq!(decoder.frame_count(), 0);
    }

    #[test]
    fn test_crc_empty() {
        let crc = crc_ccitt(&[]);
        assert_eq!(crc, 0xFFFF); // Initial value with no data
    }
}

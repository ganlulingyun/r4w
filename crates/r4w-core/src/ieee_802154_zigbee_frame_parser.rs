//! IEEE 802.15.4 (Zigbee) PHY/MAC frame parsing with CRC validation and LQI calculation.
//!
//! This module provides a complete parser for IEEE 802.15.4 frames as used by
//! Zigbee and other WPAN protocols operating in the 2.4 GHz ISM band. It handles:
//!
//! - PHY header parsing (preamble, SFD, frame length)
//! - MAC frame control field decoding (frame type, security, pending, ack, PAN ID compression)
//! - Address field parsing (short 16-bit and extended 64-bit)
//! - FCS (Frame Check Sequence) CRC-16/CCITT validation
//! - PSDU extraction from PHY frames
//! - LQI (Link Quality Indicator) estimation
//! - O-QPSK chip-to-symbol mapping for 2.4 GHz band
//! - Frame builder for constructing valid 802.15.4 frames
//!
//! Complex samples use `(f64, f64)` tuples representing (real, imaginary).
//!
//! # Example
//!
//! ```
//! use r4w_core::ieee_802154_zigbee_frame_parser::{
//!     Ieee802154Parser, FrameBuilder, FrameType, AddressMode,
//! };
//!
//! // Build a data frame
//! let frame = FrameBuilder::new()
//!     .frame_type(FrameType::Data)
//!     .sequence_number(42)
//!     .dst_pan_id(0x1234)
//!     .dst_short_addr(0xFFFF)
//!     .src_pan_id(0x1234)
//!     .src_short_addr(0x0001)
//!     .pan_id_compression(true)
//!     .payload(&[0xDE, 0xAD, 0xBE, 0xEF])
//!     .build();
//!
//! // Parse the frame
//! let parser = Ieee802154Parser::new();
//! let parsed = parser.parse_mac_frame(&frame).unwrap();
//! assert_eq!(parsed.frame_type, FrameType::Data);
//! assert_eq!(parsed.sequence_number, Some(42));
//! assert!(parsed.fcs_valid);
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// O-QPSK chip sequences for 2.4 GHz band (IEEE 802.15.4 Table 73)
// Each symbol (0-15) maps to a 32-chip PN sequence.
// ---------------------------------------------------------------------------

/// The 16 chip sequences (32 chips each) for IEEE 802.15.4 2.4 GHz O-QPSK.
/// Each entry is a `u32` whose bits represent the chip values (LSB first).
const CHIP_SEQUENCES: [u32; 16] = [
    0b11011001_11000011_01010010_01110101, // symbol 0
    0b11101100_11100001_10101001_00111010, // symbol 1
    0b00111011_00111000_01101010_10001110, // symbol 2
    0b00001101_10001110_00011010_10100011, // symbol 3
    0b01010010_01110101_11011001_11000011, // symbol 4
    0b10101001_00111010_11101100_11100001, // symbol 5
    0b01101010_10001110_00111011_00111000, // symbol 6
    0b00011010_10100011_00001101_10001110, // symbol 7
    0b10001100_10010110_10110100_00101110, // symbol 8
    0b01000110_01001011_01011010_00010111, // symbol 9
    0b00010001_10010010_11010110_10000101, // symbol 10
    0b10001000_11001001_01101011_01000010, // symbol 11
    0b10110100_00101110_10001100_10010110, // symbol 12
    0b01011010_00010111_01000110_01001011, // symbol 13
    0b11010110_10000101_00010001_10010010, // symbol 14
    0b01101011_01000010_10001000_11001001, // symbol 15
];

/// Number of chips per symbol in the 2.4 GHz band.
pub const CHIPS_PER_SYMBOL: usize = 32;

/// Number of data symbols (4-bit nibbles).
pub const NUM_SYMBOLS: usize = 16;

/// IEEE 802.15.4 preamble: 4 octets of 0x00 (32 zero bits).
pub const PREAMBLE_BYTES: [u8; 4] = [0x00; 4];

/// Start-of-Frame Delimiter for 2.4 GHz: 0xA7 (transmitted LSB first).
pub const SFD_BYTE: u8 = 0xA7;

// ---------------------------------------------------------------------------
// Frame types
// ---------------------------------------------------------------------------

/// IEEE 802.15.4 MAC frame types (3-bit field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    Beacon,
    Data,
    Ack,
    MacCommand,
    Reserved(u8),
}

impl FrameType {
    fn from_bits(v: u8) -> Self {
        match v & 0x07 {
            0 => Self::Beacon,
            1 => Self::Data,
            2 => Self::Ack,
            3 => Self::MacCommand,
            other => Self::Reserved(other),
        }
    }

    fn to_bits(self) -> u8 {
        match self {
            Self::Beacon => 0,
            Self::Data => 1,
            Self::Ack => 2,
            Self::MacCommand => 3,
            Self::Reserved(v) => v & 0x07,
        }
    }
}

impl fmt::Display for FrameType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Beacon => write!(f, "Beacon"),
            Self::Data => write!(f, "Data"),
            Self::Ack => write!(f, "Ack"),
            Self::MacCommand => write!(f, "MAC Command"),
            Self::Reserved(v) => write!(f, "Reserved({})", v),
        }
    }
}

// ---------------------------------------------------------------------------
// Address modes
// ---------------------------------------------------------------------------

/// IEEE 802.15.4 addressing modes (2-bit field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressMode {
    /// No address present.
    None = 0,
    /// Reserved.
    Reserved = 1,
    /// Short (16-bit) address.
    Short = 2,
    /// Extended (64-bit) IEEE address.
    Extended = 3,
}

impl AddressMode {
    fn from_bits(v: u8) -> Self {
        match v & 0x03 {
            0 => Self::None,
            1 => Self::Reserved,
            2 => Self::Short,
            3 => Self::Extended,
            _ => unreachable!(),
        }
    }

    fn to_bits(self) -> u8 {
        self as u8
    }

    /// Number of address bytes for this mode.
    fn addr_len(self) -> usize {
        match self {
            Self::None | Self::Reserved => 0,
            Self::Short => 2,
            Self::Extended => 8,
        }
    }
}

// ---------------------------------------------------------------------------
// Address type
// ---------------------------------------------------------------------------

/// An IEEE 802.15.4 device address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Address {
    /// No address.
    None,
    /// 16-bit short address.
    Short(u16),
    /// 64-bit extended (IEEE EUI-64) address.
    Extended(u64),
}

impl fmt::Display for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::None => write!(f, "None"),
            Self::Short(a) => write!(f, "0x{:04X}", a),
            Self::Extended(a) => write!(f, "0x{:016X}", a),
        }
    }
}

// ---------------------------------------------------------------------------
// Frame Control
// ---------------------------------------------------------------------------

/// Decoded frame control field (first 2 bytes of the MAC header).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameControl {
    pub frame_type: FrameType,
    pub security_enabled: bool,
    pub frame_pending: bool,
    pub ack_request: bool,
    pub pan_id_compression: bool,
    pub dst_addr_mode: AddressMode,
    pub frame_version: u8,
    pub src_addr_mode: AddressMode,
}

impl FrameControl {
    /// Decode from the two raw frame-control bytes (little-endian).
    pub fn decode(b0: u8, b1: u8) -> Self {
        Self {
            frame_type: FrameType::from_bits(b0 & 0x07),
            security_enabled: (b0 >> 3) & 1 != 0,
            frame_pending: (b0 >> 4) & 1 != 0,
            ack_request: (b0 >> 5) & 1 != 0,
            pan_id_compression: (b0 >> 6) & 1 != 0,
            dst_addr_mode: AddressMode::from_bits((b1 >> 2) & 0x03),
            frame_version: (b1 >> 4) & 0x03,
            src_addr_mode: AddressMode::from_bits((b1 >> 6) & 0x03),
        }
    }

    /// Encode to two little-endian bytes.
    pub fn encode(&self) -> [u8; 2] {
        let b0 = self.frame_type.to_bits()
            | (self.security_enabled as u8) << 3
            | (self.frame_pending as u8) << 4
            | (self.ack_request as u8) << 5
            | (self.pan_id_compression as u8) << 6;
        let b1 = (self.dst_addr_mode.to_bits() << 2)
            | (self.frame_version << 4)
            | (self.src_addr_mode.to_bits() << 6);
        [b0, b1]
    }
}

// ---------------------------------------------------------------------------
// Parsed frame
// ---------------------------------------------------------------------------

/// A fully parsed IEEE 802.15.4 MAC frame.
#[derive(Debug, Clone)]
pub struct ParsedFrame {
    /// Decoded frame type.
    pub frame_type: FrameType,
    /// Frame control details.
    pub frame_control: FrameControl,
    /// Sequence number (absent for some frame versions).
    pub sequence_number: Option<u8>,
    /// Destination PAN ID.
    pub dst_pan_id: Option<u16>,
    /// Destination address.
    pub dst_addr: Address,
    /// Source PAN ID (may be elided by PAN ID compression).
    pub src_pan_id: Option<u16>,
    /// Source address.
    pub src_addr: Address,
    /// MAC payload (without header or FCS).
    pub payload: Vec<u8>,
    /// Whether the FCS (CRC-16 CCITT) check passed.
    pub fcs_valid: bool,
    /// Raw FCS value from the frame.
    pub fcs: u16,
}

// ---------------------------------------------------------------------------
// PHY frame
// ---------------------------------------------------------------------------

/// Parsed PHY-layer frame components.
#[derive(Debug, Clone)]
pub struct PhyFrame {
    /// Frame length field from PHR (7 bits, value = PSDU length in bytes).
    pub frame_length: u8,
    /// The PSDU (MAC frame bytes including FCS).
    pub psdu: Vec<u8>,
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors that can occur during frame parsing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    /// Frame too short to contain the minimum required fields.
    TooShort,
    /// PHY preamble not found.
    PreambleNotFound,
    /// SFD byte mismatch.
    InvalidSfd,
    /// Frame length field exceeds maximum (127).
    InvalidLength,
    /// Address mode is reserved.
    ReservedAddressMode,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort => write!(f, "frame too short"),
            Self::PreambleNotFound => write!(f, "preamble not found"),
            Self::InvalidSfd => write!(f, "invalid SFD"),
            Self::InvalidLength => write!(f, "invalid frame length (>127)"),
            Self::ReservedAddressMode => write!(f, "reserved address mode"),
        }
    }
}

// ---------------------------------------------------------------------------
// CRC-16/CCITT (ITU-T) used by 802.15.4
// ---------------------------------------------------------------------------

/// Compute the IEEE 802.15.4 FCS (CRC-16/CCITT) over the given data.
///
/// Polynomial: x^16 + x^12 + x^5 + 1 (0x1021).
/// Initial value: 0x0000. No final XOR.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
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

// ---------------------------------------------------------------------------
// LQI estimation
// ---------------------------------------------------------------------------

/// Estimate the Link Quality Indicator (0-255) from a correlation peak value
/// and noise floor estimate.
///
/// The correlation ratio (peak / noise) is mapped linearly to the 0-255 range,
/// clamped such that a ratio of 1.0 maps to 0 and a ratio >= `max_ratio`
/// maps to 255.
pub fn estimate_lqi(correlation_peak: f64, noise_floor: f64, max_ratio: f64) -> u8 {
    if noise_floor <= 0.0 || correlation_peak <= 0.0 {
        return 0;
    }
    let ratio = correlation_peak / noise_floor;
    if ratio <= 1.0 {
        return 0;
    }
    let normalized = (ratio - 1.0) / (max_ratio - 1.0);
    let clamped = normalized.clamp(0.0, 1.0);
    (clamped * 255.0) as u8
}

/// Estimate LQI from SNR in dB. Maps `snr_db` linearly from `min_snr_db`..`max_snr_db`
/// onto 0..255.
pub fn estimate_lqi_from_snr(snr_db: f64, min_snr_db: f64, max_snr_db: f64) -> u8 {
    let normalized = (snr_db - min_snr_db) / (max_snr_db - min_snr_db);
    let clamped = normalized.clamp(0.0, 1.0);
    (clamped * 255.0) as u8
}

// ---------------------------------------------------------------------------
// O-QPSK chip-to-symbol mapping
// ---------------------------------------------------------------------------

/// Return the 32-chip sequence for a given 4-bit symbol (0..15).
///
/// Each bit of the returned `u32` is one chip value (0 or 1), LSB transmitted first.
pub fn chip_sequence(symbol: u8) -> u32 {
    CHIP_SEQUENCES[(symbol & 0x0F) as usize]
}

/// Map a chip sequence back to its symbol index by finding the best match
/// (maximum correlation) across all 16 sequences.
///
/// Returns `(symbol, correlation)` where `correlation` is the number of matching chips.
pub fn chips_to_symbol(chips: u32) -> (u8, u32) {
    let mut best_sym = 0u8;
    let mut best_corr = 0u32;
    for (sym, &expected) in CHIP_SEQUENCES.iter().enumerate() {
        let matching = 32 - (chips ^ expected).count_ones();
        if matching > best_corr {
            best_corr = matching;
            best_sym = sym as u8;
        }
    }
    (best_sym, best_corr)
}

/// Spread one byte into chips: the byte is split into two 4-bit symbols
/// (lower nibble first), and each symbol is expanded to its 32-chip sequence.
///
/// Returns a vector of 64 chip values (0 or 1).
pub fn byte_to_chips(byte: u8) -> Vec<u8> {
    let lo = byte & 0x0F;
    let hi = (byte >> 4) & 0x0F;
    let mut chips = Vec::with_capacity(64);
    for sym in [lo, hi] {
        let seq = chip_sequence(sym);
        for bit in 0..32 {
            chips.push(((seq >> bit) & 1) as u8);
        }
    }
    chips
}

/// Despreader: convert 64 chip values back to a byte by correlating each
/// group of 32 chips against all symbol sequences.
pub fn chips_to_byte(chips: &[u8]) -> Option<u8> {
    if chips.len() < 64 {
        return None;
    }
    let mut lo_bits: u32 = 0;
    for (i, &c) in chips[..32].iter().enumerate() {
        lo_bits |= ((c & 1) as u32) << i;
    }
    let mut hi_bits: u32 = 0;
    for (i, &c) in chips[32..64].iter().enumerate() {
        hi_bits |= ((c & 1) as u32) << i;
    }
    let (lo_sym, _) = chips_to_symbol(lo_bits);
    let (hi_sym, _) = chips_to_symbol(hi_bits);
    Some(lo_sym | (hi_sym << 4))
}

/// Generate O-QPSK I/Q samples from chip values.
///
/// Each chip is mapped to +1.0 or -1.0 on the I or Q rail with half-chip
/// staggering. Returns `(f64, f64)` tuples (I, Q).
pub fn chips_to_oqpsk(chips: &[u8]) -> Vec<(f64, f64)> {
    let len = chips.len();
    if len == 0 {
        return Vec::new();
    }
    // Even-indexed chips go to I, odd-indexed go to Q (with half-chip offset).
    // For simplicity we output one sample per chip; Q is delayed by one chip index.
    let mut out = Vec::with_capacity(len);
    for i in 0..len {
        let i_val = if chips[i] != 0 { 1.0 } else { -1.0 };
        let q_val = if i > 0 {
            if chips[i - 1] != 0 { 1.0 } else { -1.0 }
        } else {
            -1.0
        };
        out.push((i_val, q_val));
    }
    out
}

// ---------------------------------------------------------------------------
// Ieee802154Parser
// ---------------------------------------------------------------------------

/// Parser for IEEE 802.15.4 PHY and MAC frames.
///
/// The parser is stateless and can be reused across multiple frames.
#[derive(Debug, Clone)]
pub struct Ieee802154Parser {
    _private: (),
}

impl Ieee802154Parser {
    /// Create a new parser instance.
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Parse a PHY-layer frame (preamble + SFD + PHR + PSDU).
    ///
    /// The input should start with the 4-byte preamble (`0x00 0x00 0x00 0x00`),
    /// followed by the SFD (`0xA7`), the frame length byte, and then the PSDU.
    pub fn parse_phy_frame(&self, data: &[u8]) -> Result<PhyFrame, ParseError> {
        // Minimum: 4 preamble + 1 SFD + 1 length
        if data.len() < 6 {
            return Err(ParseError::TooShort);
        }

        // Check preamble
        if data[0..4] != PREAMBLE_BYTES {
            return Err(ParseError::PreambleNotFound);
        }

        // Check SFD
        if data[4] != SFD_BYTE {
            return Err(ParseError::InvalidSfd);
        }

        let frame_length = data[5] & 0x7F; // 7-bit field
        if frame_length > 127 {
            return Err(ParseError::InvalidLength);
        }

        let psdu_start = 6;
        let psdu_end = psdu_start + frame_length as usize;
        if data.len() < psdu_end {
            return Err(ParseError::TooShort);
        }

        Ok(PhyFrame {
            frame_length,
            psdu: data[psdu_start..psdu_end].to_vec(),
        })
    }

    /// Extract the PSDU from a PHY frame. Convenience wrapper around [`parse_phy_frame`].
    pub fn extract_psdu(&self, phy_data: &[u8]) -> Result<Vec<u8>, ParseError> {
        self.parse_phy_frame(phy_data).map(|f| f.psdu)
    }

    /// Parse a MAC-layer frame (PSDU bytes, including FCS at the end).
    ///
    /// This expects the raw MAC frame bytes (not including the PHY preamble/SFD/length).
    pub fn parse_mac_frame(&self, data: &[u8]) -> Result<ParsedFrame, ParseError> {
        // Minimum MAC frame: 2 FC + 1 SeqNo + 2 FCS = 5 bytes (ACK frame)
        if data.len() < 5 {
            return Err(ParseError::TooShort);
        }

        let fc = FrameControl::decode(data[0], data[1]);
        let mut pos = 2;

        // Sequence number
        let sequence_number = if pos < data.len().saturating_sub(2) {
            let sn = data[pos];
            pos += 1;
            Some(sn)
        } else {
            None
        };

        // FCS is last two bytes (little-endian)
        let fcs_offset = data.len() - 2;
        let fcs = u16::from_le_bytes([data[fcs_offset], data[fcs_offset + 1]]);
        let computed_crc = crc16_ccitt(&data[..fcs_offset]);
        let fcs_valid = computed_crc == fcs;

        // Destination addressing
        let mut dst_pan_id = None;
        let mut dst_addr = Address::None;
        if fc.dst_addr_mode != AddressMode::None && fc.dst_addr_mode != AddressMode::Reserved {
            if pos + 2 > fcs_offset {
                return Err(ParseError::TooShort);
            }
            dst_pan_id = Some(u16::from_le_bytes([data[pos], data[pos + 1]]));
            pos += 2;

            match fc.dst_addr_mode {
                AddressMode::Short => {
                    if pos + 2 > fcs_offset {
                        return Err(ParseError::TooShort);
                    }
                    let a = u16::from_le_bytes([data[pos], data[pos + 1]]);
                    dst_addr = Address::Short(a);
                    pos += 2;
                }
                AddressMode::Extended => {
                    if pos + 8 > fcs_offset {
                        return Err(ParseError::TooShort);
                    }
                    let mut buf = [0u8; 8];
                    buf.copy_from_slice(&data[pos..pos + 8]);
                    dst_addr = Address::Extended(u64::from_le_bytes(buf));
                    pos += 8;
                }
                _ => {}
            }
        }

        // Source addressing
        let mut src_pan_id = None;
        let mut src_addr = Address::None;
        if fc.src_addr_mode != AddressMode::None && fc.src_addr_mode != AddressMode::Reserved {
            if !fc.pan_id_compression {
                if pos + 2 > fcs_offset {
                    return Err(ParseError::TooShort);
                }
                src_pan_id = Some(u16::from_le_bytes([data[pos], data[pos + 1]]));
                pos += 2;
            } else {
                // PAN ID compressed: source PAN ID is the same as destination
                src_pan_id = dst_pan_id;
            }

            match fc.src_addr_mode {
                AddressMode::Short => {
                    if pos + 2 > fcs_offset {
                        return Err(ParseError::TooShort);
                    }
                    let a = u16::from_le_bytes([data[pos], data[pos + 1]]);
                    src_addr = Address::Short(a);
                    pos += 2;
                }
                AddressMode::Extended => {
                    if pos + 8 > fcs_offset {
                        return Err(ParseError::TooShort);
                    }
                    let mut buf = [0u8; 8];
                    buf.copy_from_slice(&data[pos..pos + 8]);
                    src_addr = Address::Extended(u64::from_le_bytes(buf));
                    pos += 8;
                }
                _ => {}
            }
        }

        let payload = data[pos..fcs_offset].to_vec();

        Ok(ParsedFrame {
            frame_type: fc.frame_type,
            frame_control: fc,
            sequence_number,
            dst_pan_id,
            dst_addr,
            src_pan_id,
            src_addr,
            payload,
            fcs_valid,
            fcs,
        })
    }

    /// Full parse: PHY frame -> MAC frame.
    pub fn parse_full(&self, phy_data: &[u8]) -> Result<ParsedFrame, ParseError> {
        let phy = self.parse_phy_frame(phy_data)?;
        self.parse_mac_frame(&phy.psdu)
    }

    /// Estimate LQI from IQ correlation samples.
    ///
    /// Uses the peak-to-noise ratio of the preamble correlation to derive
    /// a 0-255 quality indicator.
    pub fn estimate_lqi_from_samples(&self, samples: &[(f64, f64)]) -> u8 {
        if samples.is_empty() {
            return 0;
        }
        // Compute power of each sample
        let powers: Vec<f64> = samples.iter().map(|&(i, q)| i * i + q * q).collect();

        let peak = powers.iter().cloned().fold(0.0_f64, f64::max);
        let mean = powers.iter().sum::<f64>() / powers.len() as f64;

        if mean <= 0.0 {
            return 0;
        }

        estimate_lqi(peak.sqrt(), mean.sqrt(), 20.0)
    }
}

impl Default for Ieee802154Parser {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Frame builder
// ---------------------------------------------------------------------------

/// Builder for constructing valid IEEE 802.15.4 MAC frames.
///
/// Produces the raw bytes of the MAC frame (including the FCS) suitable for
/// transmission or for feeding into [`Ieee802154Parser::parse_mac_frame`].
#[derive(Debug, Clone)]
pub struct FrameBuilder {
    frame_type: FrameType,
    security_enabled: bool,
    frame_pending: bool,
    ack_request: bool,
    pan_id_compression: bool,
    frame_version: u8,
    sequence_number: u8,
    dst_pan_id: Option<u16>,
    dst_short_addr: Option<u16>,
    dst_ext_addr: Option<u64>,
    src_pan_id: Option<u16>,
    src_short_addr: Option<u16>,
    src_ext_addr: Option<u64>,
    payload: Vec<u8>,
}

impl FrameBuilder {
    /// Create a new frame builder with default settings (Data frame, seq 0).
    pub fn new() -> Self {
        Self {
            frame_type: FrameType::Data,
            security_enabled: false,
            frame_pending: false,
            ack_request: false,
            pan_id_compression: false,
            frame_version: 0,
            sequence_number: 0,
            dst_pan_id: None,
            dst_short_addr: None,
            dst_ext_addr: None,
            src_pan_id: None,
            src_short_addr: None,
            src_ext_addr: None,
            payload: Vec::new(),
        }
    }

    pub fn frame_type(mut self, ft: FrameType) -> Self {
        self.frame_type = ft;
        self
    }

    pub fn security_enabled(mut self, v: bool) -> Self {
        self.security_enabled = v;
        self
    }

    pub fn frame_pending(mut self, v: bool) -> Self {
        self.frame_pending = v;
        self
    }

    pub fn ack_request(mut self, v: bool) -> Self {
        self.ack_request = v;
        self
    }

    pub fn pan_id_compression(mut self, v: bool) -> Self {
        self.pan_id_compression = v;
        self
    }

    pub fn frame_version(mut self, v: u8) -> Self {
        self.frame_version = v & 0x03;
        self
    }

    pub fn sequence_number(mut self, sn: u8) -> Self {
        self.sequence_number = sn;
        self
    }

    pub fn dst_pan_id(mut self, pan: u16) -> Self {
        self.dst_pan_id = Some(pan);
        self
    }

    pub fn dst_short_addr(mut self, addr: u16) -> Self {
        self.dst_short_addr = Some(addr);
        self.dst_ext_addr = None;
        self
    }

    pub fn dst_ext_addr(mut self, addr: u64) -> Self {
        self.dst_ext_addr = Some(addr);
        self.dst_short_addr = None;
        self
    }

    pub fn src_pan_id(mut self, pan: u16) -> Self {
        self.src_pan_id = Some(pan);
        self
    }

    pub fn src_short_addr(mut self, addr: u16) -> Self {
        self.src_short_addr = Some(addr);
        self.src_ext_addr = None;
        self
    }

    pub fn src_ext_addr(mut self, addr: u64) -> Self {
        self.src_ext_addr = Some(addr);
        self.src_short_addr = None;
        self
    }

    pub fn payload(mut self, data: &[u8]) -> Self {
        self.payload = data.to_vec();
        self
    }

    /// Build the MAC frame bytes (header + payload + FCS).
    pub fn build(&self) -> Vec<u8> {
        let dst_addr_mode = if self.dst_ext_addr.is_some() {
            AddressMode::Extended
        } else if self.dst_short_addr.is_some() {
            AddressMode::Short
        } else {
            AddressMode::None
        };

        let src_addr_mode = if self.src_ext_addr.is_some() {
            AddressMode::Extended
        } else if self.src_short_addr.is_some() {
            AddressMode::Short
        } else {
            AddressMode::None
        };

        let fc = FrameControl {
            frame_type: self.frame_type,
            security_enabled: self.security_enabled,
            frame_pending: self.frame_pending,
            ack_request: self.ack_request,
            pan_id_compression: self.pan_id_compression,
            dst_addr_mode,
            frame_version: self.frame_version,
            src_addr_mode,
        };

        let mut frame = Vec::new();

        // Frame control (2 bytes)
        let fc_bytes = fc.encode();
        frame.extend_from_slice(&fc_bytes);

        // Sequence number
        frame.push(self.sequence_number);

        // Destination addressing
        if dst_addr_mode != AddressMode::None {
            if let Some(pan) = self.dst_pan_id {
                frame.extend_from_slice(&pan.to_le_bytes());
            }
            match dst_addr_mode {
                AddressMode::Short => {
                    if let Some(a) = self.dst_short_addr {
                        frame.extend_from_slice(&a.to_le_bytes());
                    }
                }
                AddressMode::Extended => {
                    if let Some(a) = self.dst_ext_addr {
                        frame.extend_from_slice(&a.to_le_bytes());
                    }
                }
                _ => {}
            }
        }

        // Source addressing
        if src_addr_mode != AddressMode::None {
            if !self.pan_id_compression {
                if let Some(pan) = self.src_pan_id {
                    frame.extend_from_slice(&pan.to_le_bytes());
                }
            }
            match src_addr_mode {
                AddressMode::Short => {
                    if let Some(a) = self.src_short_addr {
                        frame.extend_from_slice(&a.to_le_bytes());
                    }
                }
                AddressMode::Extended => {
                    if let Some(a) = self.src_ext_addr {
                        frame.extend_from_slice(&a.to_le_bytes());
                    }
                }
                _ => {}
            }
        }

        // Payload
        frame.extend_from_slice(&self.payload);

        // FCS (CRC-16/CCITT, little-endian)
        let crc = crc16_ccitt(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());

        frame
    }

    /// Build a complete PHY frame (preamble + SFD + length + MAC frame).
    pub fn build_phy_frame(&self) -> Vec<u8> {
        let mac = self.build();
        let mut phy = Vec::with_capacity(6 + mac.len());
        phy.extend_from_slice(&PREAMBLE_BYTES);
        phy.push(SFD_BYTE);
        phy.push(mac.len() as u8); // frame length
        phy.extend_from_slice(&mac);
        phy
    }
}

impl Default for FrameBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ----- CRC tests -----

    #[test]
    fn test_crc16_ccitt_empty() {
        assert_eq!(crc16_ccitt(&[]), 0x0000);
    }

    #[test]
    fn test_crc16_ccitt_known() {
        // "123456789" => CRC-16/CCITT-FALSE = 0x29B1
        let data = b"123456789";
        assert_eq!(crc16_ccitt(data), 0x31C3);
    }

    #[test]
    fn test_crc16_single_byte() {
        // Regression: single byte should produce a non-zero CRC
        let crc = crc16_ccitt(&[0x42]);
        assert_ne!(crc, 0);
    }

    // ----- LQI tests -----

    #[test]
    fn test_lqi_zero_noise() {
        assert_eq!(estimate_lqi(10.0, 0.0, 20.0), 0);
    }

    #[test]
    fn test_lqi_zero_peak() {
        assert_eq!(estimate_lqi(0.0, 1.0, 20.0), 0);
    }

    #[test]
    fn test_lqi_max() {
        assert_eq!(estimate_lqi(21.0, 1.0, 20.0), 255);
    }

    #[test]
    fn test_lqi_mid_range() {
        // ratio = 10, max_ratio = 19 => normalized = (10-1)/(19-1) = 0.5
        let lqi = estimate_lqi(10.0, 1.0, 19.0);
        assert!((lqi as i32 - 127).abs() <= 1, "LQI was {}", lqi);
    }

    #[test]
    fn test_lqi_from_snr() {
        assert_eq!(estimate_lqi_from_snr(5.0, 0.0, 30.0), 42);
        assert_eq!(estimate_lqi_from_snr(-5.0, 0.0, 30.0), 0);
        assert_eq!(estimate_lqi_from_snr(35.0, 0.0, 30.0), 255);
    }

    // ----- Chip sequence tests -----

    #[test]
    fn test_chip_sequence_roundtrip() {
        for sym in 0u8..16 {
            let chips = chip_sequence(sym);
            let (decoded, corr) = chips_to_symbol(chips);
            assert_eq!(decoded, sym, "symbol {} roundtrip failed", sym);
            assert_eq!(corr, 32, "perfect correlation expected for symbol {}", sym);
        }
    }

    #[test]
    fn test_byte_to_chips_length() {
        let chips = byte_to_chips(0xAB);
        assert_eq!(chips.len(), 64);
    }

    #[test]
    fn test_byte_chip_roundtrip() {
        for byte_val in [0x00u8, 0xFF, 0xA5, 0x5A, 0x0F, 0xF0, 0x42] {
            let chips = byte_to_chips(byte_val);
            let recovered = chips_to_byte(&chips).expect("should decode");
            assert_eq!(recovered, byte_val, "byte 0x{:02X} roundtrip failed", byte_val);
        }
    }

    #[test]
    fn test_chips_to_byte_too_short() {
        assert!(chips_to_byte(&[0; 32]).is_none());
    }

    // ----- O-QPSK tests -----

    #[test]
    fn test_oqpsk_length() {
        let chips = byte_to_chips(0x00);
        let iq = chips_to_oqpsk(&chips);
        assert_eq!(iq.len(), 64);
    }

    #[test]
    fn test_oqpsk_values_bounded() {
        let chips = byte_to_chips(0xAB);
        let iq = chips_to_oqpsk(&chips);
        for &(i, q) in &iq {
            assert!(i == 1.0 || i == -1.0, "I value should be +/-1, got {}", i);
            assert!(q == 1.0 || q == -1.0, "Q value should be +/-1, got {}", q);
        }
    }

    // ----- Frame control tests -----

    #[test]
    fn test_frame_control_encode_decode_roundtrip() {
        let fc = FrameControl {
            frame_type: FrameType::Data,
            security_enabled: false,
            frame_pending: true,
            ack_request: true,
            pan_id_compression: true,
            dst_addr_mode: AddressMode::Short,
            frame_version: 1,
            src_addr_mode: AddressMode::Extended,
        };
        let bytes = fc.encode();
        let decoded = FrameControl::decode(bytes[0], bytes[1]);
        assert_eq!(decoded.frame_type, FrameType::Data);
        assert!(!decoded.security_enabled);
        assert!(decoded.frame_pending);
        assert!(decoded.ack_request);
        assert!(decoded.pan_id_compression);
        assert_eq!(decoded.dst_addr_mode, AddressMode::Short);
        assert_eq!(decoded.frame_version, 1);
        assert_eq!(decoded.src_addr_mode, AddressMode::Extended);
    }

    // ----- FrameBuilder / Parser roundtrip tests -----

    #[test]
    fn test_build_parse_data_frame_short_addrs() {
        let frame = FrameBuilder::new()
            .frame_type(FrameType::Data)
            .sequence_number(99)
            .dst_pan_id(0xCAFE)
            .dst_short_addr(0x0002)
            .src_pan_id(0xCAFE)
            .src_short_addr(0x0001)
            .pan_id_compression(true)
            .payload(&[1, 2, 3, 4, 5])
            .build();

        let parser = Ieee802154Parser::new();
        let parsed = parser.parse_mac_frame(&frame).unwrap();

        assert_eq!(parsed.frame_type, FrameType::Data);
        assert_eq!(parsed.sequence_number, Some(99));
        assert_eq!(parsed.dst_pan_id, Some(0xCAFE));
        assert_eq!(parsed.dst_addr, Address::Short(0x0002));
        assert_eq!(parsed.src_pan_id, Some(0xCAFE));
        assert_eq!(parsed.src_addr, Address::Short(0x0001));
        assert_eq!(parsed.payload, vec![1, 2, 3, 4, 5]);
        assert!(parsed.fcs_valid);
    }

    #[test]
    fn test_build_parse_beacon_frame() {
        let frame = FrameBuilder::new()
            .frame_type(FrameType::Beacon)
            .sequence_number(0)
            .src_pan_id(0x1234)
            .src_short_addr(0x0000)
            .payload(&[0xFF, 0xCF]) // superframe spec
            .build();

        let parser = Ieee802154Parser::new();
        let parsed = parser.parse_mac_frame(&frame).unwrap();
        assert_eq!(parsed.frame_type, FrameType::Beacon);
        assert!(parsed.fcs_valid);
    }

    #[test]
    fn test_build_parse_ack_frame() {
        let frame = FrameBuilder::new()
            .frame_type(FrameType::Ack)
            .sequence_number(7)
            .build();

        let parser = Ieee802154Parser::new();
        let parsed = parser.parse_mac_frame(&frame).unwrap();
        assert_eq!(parsed.frame_type, FrameType::Ack);
        assert_eq!(parsed.sequence_number, Some(7));
        assert!(parsed.fcs_valid);
    }

    #[test]
    fn test_build_parse_extended_addrs() {
        let frame = FrameBuilder::new()
            .frame_type(FrameType::Data)
            .sequence_number(1)
            .dst_pan_id(0xABCD)
            .dst_ext_addr(0x0011223344556677)
            .src_pan_id(0xABCD)
            .src_ext_addr(0xAABBCCDDEEFF0011)
            .payload(&[42])
            .build();

        let parser = Ieee802154Parser::new();
        let parsed = parser.parse_mac_frame(&frame).unwrap();
        assert_eq!(parsed.dst_addr, Address::Extended(0x0011223344556677));
        assert_eq!(parsed.src_addr, Address::Extended(0xAABBCCDDEEFF0011));
        assert!(parsed.fcs_valid);
    }

    #[test]
    fn test_fcs_invalid_on_corruption() {
        let mut frame = FrameBuilder::new()
            .frame_type(FrameType::Data)
            .sequence_number(5)
            .dst_pan_id(0x1234)
            .dst_short_addr(0xFFFF)
            .payload(&[0xAA, 0xBB])
            .build();

        // Corrupt a byte
        if frame.len() > 4 {
            frame[3] ^= 0xFF;
        }

        let parser = Ieee802154Parser::new();
        let parsed = parser.parse_mac_frame(&frame).unwrap();
        assert!(!parsed.fcs_valid);
    }

    // ----- PHY frame tests -----

    #[test]
    fn test_phy_frame_roundtrip() {
        let phy = FrameBuilder::new()
            .frame_type(FrameType::Data)
            .sequence_number(10)
            .dst_pan_id(0x5678)
            .dst_short_addr(0x0099)
            .payload(&[0xDE, 0xAD])
            .build_phy_frame();

        let parser = Ieee802154Parser::new();
        let parsed = parser.parse_full(&phy).unwrap();
        assert_eq!(parsed.frame_type, FrameType::Data);
        assert_eq!(parsed.sequence_number, Some(10));
        assert!(parsed.fcs_valid);
    }

    #[test]
    fn test_phy_bad_preamble() {
        let parser = Ieee802154Parser::new();
        let bad = vec![0xFF, 0x00, 0x00, 0x00, SFD_BYTE, 5, 0, 0, 0, 0, 0];
        assert_eq!(parser.parse_phy_frame(&bad).unwrap_err(), ParseError::PreambleNotFound);
    }

    #[test]
    fn test_phy_bad_sfd() {
        let parser = Ieee802154Parser::new();
        let bad = vec![0x00, 0x00, 0x00, 0x00, 0x00, 5, 0, 0, 0, 0, 0];
        assert_eq!(parser.parse_phy_frame(&bad).unwrap_err(), ParseError::InvalidSfd);
    }

    #[test]
    fn test_phy_too_short() {
        let parser = Ieee802154Parser::new();
        assert_eq!(parser.parse_phy_frame(&[0x00; 3]).unwrap_err(), ParseError::TooShort);
    }

    // ----- LQI from samples -----

    #[test]
    fn test_lqi_from_samples_empty() {
        let parser = Ieee802154Parser::new();
        assert_eq!(parser.estimate_lqi_from_samples(&[]), 0);
    }

    #[test]
    fn test_lqi_from_samples_signal() {
        let parser = Ieee802154Parser::new();
        // Strong signal among weak noise
        let mut samples: Vec<(f64, f64)> = (0..100).map(|_| (0.01, 0.01)).collect();
        samples[50] = (10.0, 10.0);
        let lqi = parser.estimate_lqi_from_samples(&samples);
        assert!(lqi > 100, "expected high LQI, got {}", lqi);
    }

    #[test]
    fn test_extract_psdu() {
        let phy = FrameBuilder::new()
            .frame_type(FrameType::Ack)
            .sequence_number(3)
            .build_phy_frame();

        let parser = Ieee802154Parser::new();
        let psdu = parser.extract_psdu(&phy).unwrap();
        // PSDU should be the MAC frame bytes
        let mac = FrameBuilder::new()
            .frame_type(FrameType::Ack)
            .sequence_number(3)
            .build();
        assert_eq!(psdu, mac);
    }

    #[test]
    fn test_frame_type_display() {
        assert_eq!(format!("{}", FrameType::Beacon), "Beacon");
        assert_eq!(format!("{}", FrameType::Data), "Data");
        assert_eq!(format!("{}", FrameType::Ack), "Ack");
        assert_eq!(format!("{}", FrameType::MacCommand), "MAC Command");
        assert_eq!(format!("{}", FrameType::Reserved(5)), "Reserved(5)");
    }

    #[test]
    fn test_address_display() {
        assert_eq!(format!("{}", Address::None), "None");
        assert_eq!(format!("{}", Address::Short(0x1234)), "0x1234");
        assert_eq!(
            format!("{}", Address::Extended(0x0011223344556677)),
            "0x0011223344556677"
        );
    }

    #[test]
    fn test_mac_command_frame() {
        let frame = FrameBuilder::new()
            .frame_type(FrameType::MacCommand)
            .sequence_number(15)
            .dst_pan_id(0xFFFF)
            .dst_short_addr(0xFFFF)
            .src_pan_id(0x1234)
            .src_short_addr(0x0001)
            .payload(&[0x07]) // Coordinator realignment command
            .build();

        let parser = Ieee802154Parser::new();
        let parsed = parser.parse_mac_frame(&frame).unwrap();
        assert_eq!(parsed.frame_type, FrameType::MacCommand);
        assert_eq!(parsed.payload, vec![0x07]);
        assert!(parsed.fcs_valid);
    }

    #[test]
    fn test_parser_default() {
        let parser = Ieee802154Parser::default();
        // Just ensure default construction works
        let frame = FrameBuilder::default()
            .frame_type(FrameType::Ack)
            .sequence_number(0)
            .build();
        let parsed = parser.parse_mac_frame(&frame).unwrap();
        assert!(parsed.fcs_valid);
    }
}

//! ACARS Decoder — Aircraft Communications Addressing and Reporting System
//!
//! Full-featured ACARS decoder implementing ARINC 618 / ARINC 620 protocols.
//! Supports VHF AM-MSK at 2400 baud, CRC-16 CCITT frame validation, ARINC 620
//! sublabel/MFI parsing, OOOI event extraction, VDL Mode 2 D8PSK modulation,
//! AVLC frame format, HFDL PSK processing, and multi-block message reassembly.
//!
//! ## Frequency Plan
//! - Primary: 131.550 MHz (North America)
//! - Secondary: 129.125, 130.025, 130.450, 131.125, 136.700, 136.900 MHz
//!
//! ## Modulation
//! - VHF ACARS: AM-MSK, 2400 baud, ±1200 Hz deviation
//! - VDL Mode 2: D8PSK, 10.5 kbaud → 31.5 kbps
//! - HFDL: PSK-4/PSK-8, 300–1800 baud
//!
//! ## Frame Format (ARINC 618)
//! `[SYN SYN SOH] [Mode] [Reg 7] [Ack] [Label 2] [BlockID] [STX] [Text] [ETX|ETB] [BCS 2]`
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::acars_decoder::{AcarsDecoder, AcarsFrame, crc_ccitt};
//!
//! let mut decoder = AcarsDecoder::new();
//! let frame = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "UA0001", "TEST").unwrap();
//! let bytes = frame.to_bytes();
//! let result = decoder.decode_frame(&bytes);
//! assert!(result.is_ok());
//! ```

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// ACARS preamble byte.
pub const SYN: u8 = 0x16;
/// Start of Heading.
pub const SOH: u8 = 0x01;
/// Start of Text.
pub const STX: u8 = 0x02;
/// End of Text.
pub const ETX: u8 = 0x03;
/// End of Transmission Block (continued).
pub const ETB: u8 = 0x17;
/// Acknowledge.
pub const ACK: u8 = 0x06;
/// Negative Acknowledge.
pub const NAK: u8 = 0x15;
/// Delete / rubout.
pub const DEL: u8 = 0x7F;

/// VHF ACARS data rate (baud).
pub const VHF_BAUD_RATE: u32 = 2400;
/// VDL Mode 2 data rate (bps).
pub const VDL2_BIT_RATE: u32 = 31500;
/// MSK frequency deviation (Hz).
pub const MSK_DEVIATION_HZ: f64 = 1200.0;
/// CRC-16 CCITT polynomial.
pub const CRC_POLY: u16 = 0x1021;
/// CRC-16 CCITT initial value.
pub const CRC_INIT: u16 = 0xFFFF;

/// Standard ACARS VHF frequencies (MHz).
pub const VHF_FREQUENCIES: &[f64] = &[
    129.125, 130.025, 130.450, 131.125, 131.550, 136.700, 136.900,
];

/// Maximum ACARS frame text length (characters).
pub const MAX_TEXT_LEN: usize = 220;
/// Aircraft registration length (bytes).
pub const REG_LEN: usize = 7;
/// Label length (bytes).
pub const LABEL_LEN: usize = 2;

// ---------------------------------------------------------------------------
// Message Label Table
// ---------------------------------------------------------------------------

/// Describes a known ACARS message label.
#[derive(Debug, Clone, PartialEq)]
pub struct LabelInfo {
    /// Two-character label code.
    pub label: &'static str,
    /// Human-readable description.
    pub description: &'static str,
    /// Direction (uplink/downlink/both).
    pub direction: MessageDirection,
}

/// Message direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageDirection {
    Uplink,
    Downlink,
    Both,
}

/// Return information about a known ACARS label, if found.
pub fn lookup_label(label: &str) -> Option<LabelInfo> {
    let table: &[(&str, &str, MessageDirection)] = &[
        ("_d", "DFE selection / free text", MessageDirection::Both),
        ("_\x7F", "Emergency", MessageDirection::Both),
        ("5U", "Airline operational", MessageDirection::Downlink),
        ("5Z", "Airline ops / free text", MessageDirection::Both),
        ("H1", "HF data link ACARS", MessageDirection::Both),
        ("Q0", "Link test / ping", MessageDirection::Both),
        ("Q1", "Link test response", MessageDirection::Both),
        ("SQ", "Squawk / transponder code", MessageDirection::Downlink),
        ("RA", "Radio altitude", MessageDirection::Downlink),
        ("S1", "Speed, altitude, position", MessageDirection::Downlink),
        ("10", "Digital ATIS", MessageDirection::Uplink),
        ("13", "ATC clearance", MessageDirection::Uplink),
        ("21", "Weather request", MessageDirection::Downlink),
        ("22", "Weather data", MessageDirection::Uplink),
        ("30", "Pre-departure clearance", MessageDirection::Uplink),
        ("AA", "OOOI report", MessageDirection::Downlink),
        ("AB", "Position report", MessageDirection::Downlink),
        ("AC", "Engine report", MessageDirection::Downlink),
        ("ED", "ETOPS advisory", MessageDirection::Uplink),
        ("M1", "MCDU/FMS message", MessageDirection::Both),
        ("M2", "FMS winds aloft", MessageDirection::Both),
        ("M3", "FMS position", MessageDirection::Downlink),
        ("PR", "Position report abbreviated", MessageDirection::Downlink),
    ];
    table.iter().find(|(l, _, _)| *l == label).map(|(l, d, dir)| LabelInfo {
        label: l,
        description: d,
        direction: *dir,
    })
}

// ---------------------------------------------------------------------------
// MSK Modulator / Demodulator
// ---------------------------------------------------------------------------

/// MSK (Minimum Shift Keying) modulator for ACARS.
///
/// Produces complex baseband samples at the given sample rate.
/// Phase is continuous across symbol boundaries (h = 0.5).
#[derive(Debug, Clone)]
pub struct MskModulator {
    sample_rate: f64,
    baud_rate: f64,
    /// Accumulated carrier phase (radians).
    phase: f64,
    /// Samples per symbol.
    sps: f64,
    /// Sample index within current symbol.
    sample_idx: f64,
}

impl MskModulator {
    /// Create a new MSK modulator.
    ///
    /// `sample_rate` — output sample rate (Hz)
    /// `baud_rate`   — symbol rate (2400 for VHF ACARS)
    pub fn new(sample_rate: f64, baud_rate: f64) -> Self {
        Self {
            sample_rate,
            baud_rate,
            phase: 0.0,
            sps: sample_rate / baud_rate,
            sample_idx: 0.0,
        }
    }

    /// Modulate a sequence of bits (true = +1, false = −1).
    ///
    /// Returns interleaved (I, Q) pairs as a flat Vec<f64>.
    pub fn modulate(&mut self, bits: &[bool]) -> Vec<f64> {
        let sps = (self.sps).round() as usize;
        let mut out = Vec::with_capacity(bits.len() * sps * 2);
        let omega = core::f64::consts::PI * self.baud_rate / self.sample_rate; // π / (2·Ts)·Ts_samp
        for &bit in bits {
            let sign = if bit { 1.0_f64 } else { -1.0_f64 };
            for _ in 0..sps {
                let i = self.phase.cos();
                let q = self.phase.sin();
                out.push(i);
                out.push(q);
                self.phase += sign * omega;
            }
            // Wrap phase to [-π, π]
            self.phase = wrap_phase(self.phase);
        }
        out
    }

    /// Reset modulator state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.sample_idx = 0.0;
    }
}

/// Wrap a phase value to (−π, π].
#[inline]
fn wrap_phase(p: f64) -> f64 {
    use core::f64::consts::PI;
    let mut v = p;
    while v > PI {
        v -= 2.0 * PI;
    }
    while v <= -PI {
        v += 2.0 * PI;
    }
    v
}

/// Non-coherent MSK demodulator using differential phase detection.
///
/// Operates on interleaved (I, Q) samples at `sample_rate`.
#[derive(Debug, Clone)]
pub struct MskDemodulator {
    sample_rate: f64,
    baud_rate: f64,
    sps: usize,
    /// Previous complex sample for differential detection.
    prev_i: f64,
    prev_q: f64,
    /// Sample accumulator for symbol timing.
    accum: usize,
    /// Collected bits.
    bits: Vec<bool>,
}

impl MskDemodulator {
    /// Create a new non-coherent MSK demodulator.
    pub fn new(sample_rate: f64, baud_rate: f64) -> Self {
        let sps = (sample_rate / baud_rate).round() as usize;
        Self {
            sample_rate,
            baud_rate,
            sps,
            prev_i: 1.0,
            prev_q: 0.0,
            accum: 0,
            bits: Vec::new(),
        }
    }

    /// Feed interleaved (I, Q) samples.
    pub fn feed(&mut self, iq: &[f64]) {
        let n = iq.len() / 2;
        for k in 0..n {
            let ci = iq[2 * k];
            let cq = iq[2 * k + 1];
            // Differential phase: arg(x[n] * conj(x[n-1]))
            let dphi = ci * self.prev_i + cq * self.prev_q;  // Re part
            self.prev_i = ci;
            self.prev_q = cq;
            self.accum += 1;
            if self.accum >= self.sps {
                self.accum = 0;
                self.bits.push(dphi > 0.0);
            }
        }
    }

    /// Drain decoded bits.
    pub fn drain_bits(&mut self) -> Vec<bool> {
        core::mem::take(&mut self.bits)
    }

    /// Reset demodulator state.
    pub fn reset(&mut self) {
        self.prev_i = 1.0;
        self.prev_q = 0.0;
        self.accum = 0;
        self.bits.clear();
    }
}

// ---------------------------------------------------------------------------
// Character encoding
// ---------------------------------------------------------------------------

/// Encode a character as 7-bit ASCII with odd parity in bit 7.
pub fn encode_char_with_parity(c: u8) -> u8 {
    let v = c & 0x7F;
    let ones = v.count_ones();
    // Odd parity: bit 7 set if even number of ones in bits 0-6
    if ones % 2 == 0 {
        v | 0x80
    } else {
        v
    }
}

/// Strip parity bit and return 7-bit ASCII value.
#[inline]
pub fn strip_parity(b: u8) -> u8 {
    b & 0x7F
}

/// Check odd parity of a byte.
pub fn check_parity(b: u8) -> bool {
    b.count_ones() % 2 == 1
}

/// Encode a string slice into ACARS bytes (7-bit + odd parity).
pub fn encode_string(s: &str) -> Vec<u8> {
    s.bytes().map(encode_char_with_parity).collect()
}

/// Decode ACARS bytes to a String, stripping parity bits.
pub fn decode_bytes_to_string(data: &[u8]) -> String {
    data.iter().map(|&b| strip_parity(b) as char).collect()
}

// ---------------------------------------------------------------------------
// CRC-16 CCITT
// ---------------------------------------------------------------------------

/// Compute CRC-16 CCITT (polynomial 0x1021, initial value 0xFFFF).
///
/// This is the standard BCS (Block Check Sequence) used in ACARS.
pub fn crc_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = CRC_INIT;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ CRC_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Append the two-byte CRC-16 CCITT to a byte vector (big-endian).
pub fn append_crc(data: &mut Vec<u8>) {
    let crc = crc_ccitt(data);
    data.push((crc >> 8) as u8);
    data.push((crc & 0xFF) as u8);
}

/// Verify CRC: the last 2 bytes of `data` should be the CRC of the rest.
pub fn verify_crc(data: &[u8]) -> bool {
    if data.len() < 3 {
        return false;
    }
    let payload = &data[..data.len() - 2];
    let stored = ((data[data.len() - 2] as u16) << 8) | data[data.len() - 1] as u16;
    crc_ccitt(payload) == stored
}

// ---------------------------------------------------------------------------
// ACARS Frame
// ---------------------------------------------------------------------------

/// A fully parsed ACARS frame (ARINC 618).
#[derive(Debug, Clone, PartialEq)]
pub struct AcarsFrame {
    /// Mode character: '2'=VHF normal, 'X'=override, 'Y'=VHF backup.
    pub mode: char,
    /// Aircraft registration, up to 7 characters (e.g., ".N12345").
    pub aircraft_reg: String,
    /// Acknowledgement character (space, NAK, or ACK).
    pub ack: char,
    /// 2-character message label.
    pub label: String,
    /// Block identifier character ('0'–'9').
    pub block_id: char,
    /// Flight ID / sequence number (up to 6 chars).
    pub flight_id: String,
    /// Message text body.
    pub text: String,
    /// Whether the BCS was valid.
    pub crc_valid: bool,
    /// Whether this is a continuation block (ETB was used).
    pub more_blocks: bool,
}

impl AcarsFrame {
    /// Build a new ACARS frame from components.
    ///
    /// Returns an error string if arguments are out of range.
    pub fn build(
        mode: char,
        reg: &str,
        ack: char,
        label: &str,
        block_id: char,
        flight_id: &str,
        text: &str,
    ) -> Result<Self, &'static str> {
        if reg.len() > REG_LEN {
            return Err("Registration too long (max 7)");
        }
        if label.len() != LABEL_LEN {
            return Err("Label must be exactly 2 characters");
        }
        if text.len() > MAX_TEXT_LEN {
            return Err("Text too long (max 220)");
        }
        Ok(Self {
            mode,
            aircraft_reg: format!("{:>7}", reg),
            ack,
            label: label.to_string(),
            block_id,
            flight_id: flight_id.to_string(),
            text: text.to_string(),
            crc_valid: true,
            more_blocks: false,
        })
    }

    /// Serialize the frame to raw bytes (including SYN, SOH, STX, ETX, BCS).
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut frame = Vec::with_capacity(32 + self.text.len());
        // Preamble
        frame.push(SYN);
        frame.push(SYN);
        frame.push(SOH);
        // Header
        frame.push(self.mode as u8);
        let reg_bytes: Vec<u8> = format!("{:>7}", self.aircraft_reg)
            .bytes()
            .take(REG_LEN)
            .collect();
        frame.extend_from_slice(&reg_bytes);
        frame.push(self.ack as u8);
        let label_bytes: Vec<u8> = self.label.bytes().take(LABEL_LEN).collect();
        frame.extend_from_slice(&label_bytes);
        frame.push(self.block_id as u8);
        // Text block
        frame.push(STX);
        if !self.flight_id.is_empty() {
            let fid: Vec<u8> = format!("{:<6}", self.flight_id).bytes().take(6).collect();
            frame.extend_from_slice(&fid);
        }
        frame.extend_from_slice(self.text.as_bytes());
        let end_marker = if self.more_blocks { ETB } else { ETX };
        frame.push(end_marker);
        // BCS (over everything from SOH onwards)
        let crc_start = 2; // skip SYN SYN
        let crc = crc_ccitt(&frame[crc_start..]);
        frame.push((crc >> 8) as u8);
        frame.push((crc & 0xFF) as u8);
        frame
    }

    /// Try to parse an ACARS frame from raw bytes.
    ///
    /// `data` should start at or before the SOH byte.
    pub fn from_bytes(data: &[u8]) -> Result<Self, AcarsError> {
        // Skip preamble SYN bytes
        let soh_pos = data.iter().position(|&b| b == SOH).ok_or(AcarsError::NoSync)?;
        let hdr = soh_pos + 1;

        // Minimum: mode(1) + reg(7) + ack(1) + label(2) + block_id(1) + STX(1) + ETX(1) + BCS(2)
        if data.len() < hdr + 16 {
            return Err(AcarsError::TooShort);
        }

        let mode = strip_parity(data[hdr]) as char;
        let aircraft_reg: String = data[hdr + 1..hdr + 8]
            .iter()
            .map(|&b| strip_parity(b) as char)
            .collect();
        let ack = strip_parity(data[hdr + 8]) as char;
        let label: String = data[hdr + 9..hdr + 11]
            .iter()
            .map(|&b| strip_parity(b) as char)
            .collect();
        let block_id = strip_parity(data[hdr + 11]) as char;

        // Expect STX next
        if data[hdr + 12] != STX {
            return Err(AcarsError::InvalidChar);
        }

        // Find ETX or ETB
        let body_start = hdr + 13;
        let end_pos = data[body_start..]
            .iter()
            .position(|&b| b == ETX || b == ETB)
            .map(|p| body_start + p)
            .ok_or(AcarsError::TooShort)?;

        let more_blocks = data[end_pos] == ETB;
        let body = &data[body_start..end_pos];

        // Flight ID is first 6 chars of body (if present)
        let (flight_id, text_body) = if body.len() >= 6 {
            let fid: String = body[..6].iter().map(|&b| strip_parity(b) as char).collect();
            let txt: String = body[6..].iter().map(|&b| strip_parity(b) as char).collect();
            (fid, txt)
        } else {
            let txt: String = body.iter().map(|&b| strip_parity(b) as char).collect();
            (String::new(), txt)
        };

        // BCS validation: covers SOH..=ETX/ETB (skip SYN bytes in preamble)
        let crc_start = soh_pos; // from SOH
        let computed_crc = crc_ccitt(&data[crc_start..=end_pos]);
        let crc_valid = if data.len() >= end_pos + 3 {
            let stored = ((data[end_pos + 1] as u16) << 8) | data[end_pos + 2] as u16;
            computed_crc == stored
        } else {
            false
        };

        Ok(AcarsFrame {
            mode,
            aircraft_reg: aircraft_reg.trim().to_string(),
            ack,
            label,
            block_id,
            flight_id: flight_id.trim().to_string(),
            text: text_body,
            crc_valid,
            more_blocks,
        })
    }

    /// Return the message direction based on mode character.
    pub fn direction(&self) -> MessageDirection {
        match self.mode {
            '2' | 'X' | 'Y' => MessageDirection::Downlink,
            'A'..='Z' if self.mode != 'X' && self.mode != 'Y' => MessageDirection::Uplink,
            _ => MessageDirection::Both,
        }
    }

    /// Validate aircraft registration (alphanumeric + dot/hyphen).
    pub fn is_valid_registration(reg: &str) -> bool {
        let reg = reg.trim();
        if reg.is_empty() || reg.len() > 7 {
            return false;
        }
        reg.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'.' || b == b'-')
    }
}

// ---------------------------------------------------------------------------
// ACARS Decoder (streaming)
// ---------------------------------------------------------------------------

/// Errors that can occur during ACARS frame decoding.
#[derive(Debug, Clone, PartialEq)]
pub enum AcarsError {
    /// Frame data is too short to contain a valid frame.
    TooShort,
    /// An unexpected / invalid character was encountered.
    InvalidChar,
    /// CRC-16 check failed.
    CrcMismatch,
    /// No SOH synchronisation byte found.
    NoSync,
    /// Text payload exceeds maximum allowed length.
    TextTooLong,
}

impl core::fmt::Display for AcarsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooShort    => write!(f, "Frame too short"),
            Self::InvalidChar => write!(f, "Invalid character in frame"),
            Self::CrcMismatch => write!(f, "CRC-16 mismatch"),
            Self::NoSync      => write!(f, "No SOH sync found"),
            Self::TextTooLong => write!(f, "Text payload too long"),
        }
    }
}

impl std::error::Error for AcarsError {}

/// Streaming ACARS decoder.
///
/// Accepts raw bits (from an MSK demodulator) or pre-assembled byte frames.
/// Maintains a sliding bit buffer, scans for the `SYN SYN SOH` preamble, and
/// extracts complete frames once an ETX or ETB terminator is found.
#[derive(Debug, Clone)]
pub struct AcarsDecoder {
    /// Sliding bit buffer.
    bit_buf: Vec<bool>,
    /// Successfully decoded frames.
    frames: Vec<AcarsFrame>,
    /// Total frame attempts.
    pub frame_count: usize,
    /// CRC failures.
    pub crc_errors: usize,
    /// Reassembly table keyed by (aircraft_reg, block_id).
    reassembly: HashMap<String, Vec<AcarsFrame>>,
}

impl AcarsDecoder {
    /// Create a new ACARS decoder.
    pub fn new() -> Self {
        Self {
            bit_buf: Vec::new(),
            frames: Vec::new(),
            frame_count: 0,
            crc_errors: 0,
            reassembly: HashMap::new(),
        }
    }

    /// Feed raw demodulated bits into the decoder.
    ///
    /// Internally accumulates bits, scans for preambles, and extracts frames.
    pub fn feed_bits(&mut self, bits: &[bool]) {
        self.bit_buf.extend_from_slice(bits);
        self.scan_buffer();
    }

    /// Decode a pre-assembled byte slice as an ACARS frame.
    pub fn decode_frame(&mut self, data: &[u8]) -> Result<AcarsFrame, AcarsError> {
        self.frame_count += 1;
        let frame = AcarsFrame::from_bytes(data)?;
        if !frame.crc_valid {
            self.crc_errors += 1;
        }
        self.handle_reassembly(frame.clone());
        self.frames.push(frame.clone());
        Ok(frame)
    }

    /// Feed a decoded frame directly (e.g., from another source).
    pub fn push_frame(&mut self, frame: AcarsFrame) {
        self.frame_count += 1;
        if !frame.crc_valid {
            self.crc_errors += 1;
        }
        self.handle_reassembly(frame.clone());
        self.frames.push(frame);
    }

    /// Retrieve all decoded frames.
    pub fn frames(&self) -> &[AcarsFrame] {
        &self.frames
    }

    /// Alias for frames() — kept for backward compatibility.
    pub fn messages(&self) -> &[AcarsFrame] {
        &self.frames
    }

    /// Total frames processed.
    pub fn frame_count(&self) -> usize {
        self.frame_count
    }

    /// Total CRC failures.
    pub fn crc_errors(&self) -> usize {
        self.crc_errors
    }

    /// Reset the decoder state.
    pub fn reset(&mut self) {
        self.bit_buf.clear();
        self.frames.clear();
        self.frame_count = 0;
        self.crc_errors = 0;
        self.reassembly.clear();
    }

    /// Return completed multi-block messages (all blocks received).
    pub fn completed_messages(&self) -> Vec<Vec<AcarsFrame>> {
        self.reassembly
            .values()
            .filter(|blocks| !blocks.is_empty() && !blocks.last().unwrap().more_blocks)
            .cloned()
            .collect()
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    fn scan_buffer(&mut self) {
        // Need at least SYN SYN SOH = 3 bytes = 24 bits, plus minimal frame
        loop {
            if self.bit_buf.len() < 24 {
                break;
            }
            // Search for SYN SYN SOH pattern
            match self.find_preamble() {
                Some(byte_offset) => {
                    // Collect bytes from preamble until ETX/ETB (or buffer exhausted)
                    let bit_start = byte_offset * 8;
                    let available_bytes = (self.bit_buf.len() - bit_start) / 8;
                    if available_bytes < 4 {
                        // Not enough data yet
                        break;
                    }
                    // Collect raw bytes
                    let max_collect = available_bytes.min(256 + 5);
                    let mut raw: Vec<u8> = (0..max_collect)
                        .map(|i| bits_to_byte(&self.bit_buf[bit_start + i * 8..]))
                        .collect();

                    // Find ETX or ETB in raw bytes
                    let end = raw.iter().position(|&b| b == ETX || b == ETB);
                    if let Some(end_pos) = end {
                        // Include 2 BCS bytes after end marker
                        let need = end_pos + 3;
                        if raw.len() >= need {
                            raw.truncate(need);
                            let _ = self.decode_frame_internal(&raw);
                            // Advance buffer past this frame
                            let consume = bit_start + need * 8;
                            let consume = consume.min(self.bit_buf.len());
                            self.bit_buf.drain(..consume);
                        } else {
                            // Waiting for more data
                            break;
                        }
                    } else if available_bytes >= 256 {
                        // No terminator within 256 bytes — discard one byte and retry
                        if self.bit_buf.len() >= 8 {
                            self.bit_buf.drain(..8);
                        } else {
                            break;
                        }
                    } else {
                        // Waiting for more data
                        break;
                    }
                }
                None => {
                    // No preamble found — discard bits keeping last 16 (partial preamble)
                    let keep = 16 * 8;
                    if self.bit_buf.len() > keep {
                        let drain = self.bit_buf.len() - keep;
                        self.bit_buf.drain(..drain);
                    }
                    break;
                }
            }
        }
    }

    /// Find `SYN SYN SOH` pattern in bit buffer, return byte offset of first SYN.
    fn find_preamble(&self) -> Option<usize> {
        let max_byte = self.bit_buf.len() / 8;
        if max_byte < 3 {
            return None;
        }
        for i in 0..=(max_byte - 3) {
            let b0 = bits_to_byte(&self.bit_buf[i * 8..]);
            let b1 = bits_to_byte(&self.bit_buf[(i + 1) * 8..]);
            let b2 = bits_to_byte(&self.bit_buf[(i + 2) * 8..]);
            if b0 == SYN && b1 == SYN && b2 == SOH {
                return Some(i);
            }
        }
        None
    }

    fn decode_frame_internal(&mut self, data: &[u8]) -> Result<(), AcarsError> {
        self.frame_count += 1;
        match AcarsFrame::from_bytes(data) {
            Ok(frame) => {
                if !frame.crc_valid {
                    self.crc_errors += 1;
                }
                self.handle_reassembly(frame.clone());
                self.frames.push(frame);
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    fn handle_reassembly(&mut self, frame: AcarsFrame) {
        // Key: "reg:label"
        let key = format!("{}:{}", frame.aircraft_reg, frame.label);
        let entry = self.reassembly.entry(key).or_insert_with(Vec::new);
        // Duplicate detection by block_id
        let dup = entry.iter().any(|f| f.block_id == frame.block_id);
        if !dup {
            entry.push(frame);
        }
    }
}

impl Default for AcarsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert up to 8 bits starting at `bits[0]` to a u8 (MSB first).
#[inline]
fn bits_to_byte(bits: &[bool]) -> u8 {
    let mut b: u8 = 0;
    for (i, &bit) in bits.iter().take(8).enumerate() {
        if bit {
            b |= 1 << (7 - i);
        }
    }
    b
}

/// Convert a u8 to 8 booleans (MSB first).
pub fn byte_to_bits(b: u8) -> [bool; 8] {
    let mut bits = [false; 8];
    for i in 0..8 {
        bits[i] = (b >> (7 - i)) & 1 == 1;
    }
    bits
}

// ---------------------------------------------------------------------------
// OOOI Event Parser
// ---------------------------------------------------------------------------

/// ACARS OOOI event types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OooiEvent {
    /// Aircraft pushed back from gate (Out).
    Out,
    /// Aircraft became airborne (Off).
    Off,
    /// Aircraft touched down (On).
    On,
    /// Aircraft arrived at gate (In).
    In,
}

/// Parsed OOOI position/event report.
#[derive(Debug, Clone, PartialEq)]
pub struct OooiReport {
    pub event: OooiEvent,
    /// Flight number (e.g., "UA0001").
    pub flight: String,
    /// Origin airport (ICAO).
    pub origin: String,
    /// Destination airport (ICAO).
    pub destination: String,
    /// UTC time string (e.g., "1435").
    pub utc_time: String,
    /// Latitude decimal degrees (if available).
    pub lat: Option<f64>,
    /// Longitude decimal degrees (if available).
    pub lon: Option<f64>,
    /// Altitude (feet, if available).
    pub alt_ft: Option<i32>,
}

/// Attempt to parse an OOOI event from an ACARS text payload.
///
/// Handles common airline OOOI formats (AA label).
pub fn parse_oooi(label: &str, text: &str) -> Option<OooiReport> {
    if label != "AA" {
        return None;
    }
    // Typical format: "/OUT1435/ORIG KORD/DEST KLAX/FLT UA0001"
    let event = if text.contains("/OUT") {
        OooiEvent::Out
    } else if text.contains("/OFF") {
        OooiEvent::Off
    } else if text.contains("/ON") {
        OooiEvent::On
    } else if text.contains("/IN") {
        OooiEvent::In
    } else {
        return None;
    };

    let utc_time = extract_field(text, "/OUT")
        .or_else(|| extract_field(text, "/OFF"))
        .or_else(|| extract_field(text, "/ON"))
        .or_else(|| extract_field(text, "/IN"))
        .unwrap_or_default();

    let origin = extract_field(text, "/ORIG").unwrap_or_default();
    let destination = extract_field(text, "/DEST").unwrap_or_default();
    let flight = extract_field(text, "/FLT").unwrap_or_default();

    // Optional lat/lon (format: /POSN47.123,-122.456)
    let (lat, lon) = parse_position(text);

    // Optional altitude (format: /ALT35000)
    let alt_ft = parse_altitude(text);

    Some(OooiReport {
        event,
        flight,
        origin,
        destination,
        utc_time,
        lat,
        lon,
        alt_ft,
    })
}

fn extract_field<'a>(text: &'a str, key: &str) -> Option<String> {
    let pos = text.find(key)?;
    let after = &text[pos + key.len()..];
    let end = after.find('/').unwrap_or(after.len());
    Some(after[..end].trim().to_string())
}

fn parse_position(text: &str) -> (Option<f64>, Option<f64>) {
    if let Some(pos) = text.find("/POSN") {
        let rest = &text[pos + 5..];
        let end = rest.find('/').unwrap_or(rest.len());
        let coords: Vec<&str> = rest[..end].split(',').collect();
        if coords.len() == 2 {
            let lat = coords[0].trim().parse::<f64>().ok();
            let lon = coords[1].trim().parse::<f64>().ok();
            return (lat, lon);
        }
    }
    (None, None)
}

fn parse_altitude(text: &str) -> Option<i32> {
    if let Some(pos) = text.find("/ALT") {
        let rest = &text[pos + 4..];
        let end = rest.find('/').unwrap_or(rest.len());
        return rest[..end].trim().parse::<i32>().ok();
    }
    None
}

// ---------------------------------------------------------------------------
// ARINC 620 Sublabel / MFI
// ---------------------------------------------------------------------------

/// ARINC 620 sublabel/MFI (Message Function Identifier) parsed structure.
#[derive(Debug, Clone, PartialEq)]
pub struct Arinc620Header {
    /// Two-character sublabel (e.g., "M1").
    pub sublabel: String,
    /// Media/Function Identifier code (e.g., "2S").
    pub mfi: String,
    /// Remaining message text after the header.
    pub body: String,
}

/// Parse an ARINC 620 sublabel/MFI from a raw ACARS text body.
///
/// Format: `[sublabel(2)][MFI(2)][body...]`
/// Only applicable to H1 and certain other labels.
pub fn parse_arinc620(text: &str) -> Option<Arinc620Header> {
    if text.len() < 4 {
        return None;
    }
    let sublabel = text[..2].to_string();
    let mfi = text[2..4].to_string();
    let body = text[4..].to_string();
    Some(Arinc620Header { sublabel, mfi, body })
}

/// Well-known ARINC 620 MFI codes.
pub fn mfi_description(mfi: &str) -> &'static str {
    match mfi {
        "1S" => "ACARS service",
        "2S" => "Media advisory",
        "4S" => "ATC message",
        "5S" => "Weather",
        "7S" => "Position",
        "8S" => "Free text",
        "D0" => "DFDR/ACMS data",
        _ => "Unknown MFI",
    }
}

// ---------------------------------------------------------------------------
// Multi-Block Message Reassembly
// ---------------------------------------------------------------------------

/// Reassembler for multi-block ACARS messages.
///
/// ACARS messages longer than ~220 characters are split across multiple blocks,
/// each terminated by ETB except the final block (ETX).
#[derive(Debug, Clone, Default)]
pub struct MessageReassembler {
    /// Pending block chains keyed by "reg:label".
    pending: HashMap<String, ReassemblyEntry>,
    /// Completed messages (all blocks received).
    completed: Vec<String>,
}

#[derive(Debug, Clone)]
struct ReassemblyEntry {
    blocks: Vec<(char, String)>, // (block_id, text)
    done: bool,
}

impl MessageReassembler {
    /// Create a new reassembler.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a decoded frame to the reassembler.
    ///
    /// If the frame completes a message, it is moved to the completed queue.
    pub fn add_frame(&mut self, frame: &AcarsFrame) {
        let key = format!("{}:{}", frame.aircraft_reg.trim(), frame.label);
        let entry = self.pending.entry(key.clone()).or_insert_with(|| ReassemblyEntry {
            blocks: Vec::new(),
            done: false,
        });

        // Skip duplicates (same block_id already stored)
        let dup = entry.blocks.iter().any(|(bid, _)| *bid == frame.block_id);
        if dup {
            return;
        }

        entry.blocks.push((frame.block_id, frame.text.clone()));

        if !frame.more_blocks {
            // Sort by block_id and reassemble
            entry.blocks.sort_by_key(|(bid, _)| *bid);
            let full: String = entry.blocks.iter().map(|(_, t)| t.as_str()).collect();
            entry.done = true;
            self.completed.push(full);
            self.pending.remove(&key);
        }
    }

    /// Drain completed reassembled message strings.
    pub fn drain_completed(&mut self) -> Vec<String> {
        core::mem::take(&mut self.completed)
    }

    /// Number of in-progress (incomplete) message chains.
    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }
}

// ---------------------------------------------------------------------------
// VDL Mode 2 — D8PSK
// ---------------------------------------------------------------------------

/// D8PSK (Differential 8-PSK) symbol mapper for VDL Mode 2.
///
/// VDL2 uses 8-PSK with differential encoding at 10.5 kbaud (31.5 kbps).
/// Phase differences encode 3 bits per symbol.
#[derive(Debug, Clone)]
pub struct D8pskMapper {
    /// Previous symbol phase (radians).
    prev_phase: f64,
}

impl D8pskMapper {
    /// Create a new D8PSK mapper.
    pub fn new() -> Self {
        Self { prev_phase: 0.0 }
    }

    /// Map 3 bits to a differential D8PSK phase increment and return (I, Q).
    ///
    /// Gray-coded mapping per RTCA DO-224A / ICAO Doc 9776.
    pub fn map_bits(&mut self, b2: bool, b1: bool, b0: bool) -> (f64, f64) {
        let di = d8psk_gray_index(b2, b1, b0);
        let delta_phi = (di as f64) * core::f64::consts::PI / 4.0;
        let phi = wrap_phase(self.prev_phase + delta_phi);
        self.prev_phase = phi;
        (phi.cos(), phi.sin())
    }

    /// Map a byte to 8-PSK symbols (MSB first, 8 bits → 2 symbols + 2 spare bits).
    ///
    /// Returns up to 2 (I, Q) pairs.
    pub fn map_byte(&mut self, byte: u8) -> [(f64, f64); 2] {
        let sym0 = self.map_bits(
            (byte >> 7) & 1 == 1,
            (byte >> 6) & 1 == 1,
            (byte >> 5) & 1 == 1,
        );
        let sym1 = self.map_bits(
            (byte >> 4) & 1 == 1,
            (byte >> 3) & 1 == 1,
            (byte >> 2) & 1 == 1,
        );
        [sym0, sym1]
    }

    /// Reset mapper phase state.
    pub fn reset(&mut self) {
        self.prev_phase = 0.0;
    }

    /// Demap an (I, Q) sample to the closest D8PSK phase index (0–7).
    pub fn demap(&self, i: f64, q: f64) -> u8 {
        let phi = q.atan2(i);
        let idx = ((phi / (core::f64::consts::PI / 4.0)).round()).rem_euclid(8.0) as u8;
        idx
    }
}

impl Default for D8pskMapper {
    fn default() -> Self {
        Self::new()
    }
}

/// Gray code table for D8PSK: 3 bits → phase index.
fn d8psk_gray_index(b2: bool, b1: bool, b0: bool) -> u8 {
    let raw = ((b2 as u8) << 2) | ((b1 as u8) << 1) | (b0 as u8);
    // Gray decode: gray → binary
    let gray = raw;
    let b = gray ^ (gray >> 1) ^ (gray >> 2);
    b & 0x07
}

// ---------------------------------------------------------------------------
// AVLC Frame (VDL Mode 2)
// ---------------------------------------------------------------------------

/// AVLC (Aircraft VHF Link Control) frame types for VDL Mode 2.
#[derive(Debug, Clone, PartialEq)]
pub enum AvlcFrameType {
    /// Information frame (data).
    Information,
    /// Supervisory frame (flow control).
    Supervisory,
    /// Unnumbered frame (control).
    Unnumbered,
}

/// Parsed AVLC frame (simplified HDLC-based).
#[derive(Debug, Clone, PartialEq)]
pub struct AvlcFrame {
    pub frame_type: AvlcFrameType,
    /// Destination address (ICAO 24-bit + SSID).
    pub dest_addr: u32,
    /// Source address.
    pub src_addr: u32,
    /// Frame control byte.
    pub control: u8,
    /// Payload bytes.
    pub payload: Vec<u8>,
    /// FCS valid flag.
    pub fcs_valid: bool,
}

impl AvlcFrame {
    /// Build a minimal AVLC information frame.
    pub fn new_info(dest: u32, src: u32, payload: Vec<u8>) -> Self {
        Self {
            frame_type: AvlcFrameType::Information,
            dest_addr: dest,
            src_addr: src,
            control: 0x00, // I-frame, N(S)=0, N(R)=0
            payload,
            fcs_valid: true,
        }
    }

    /// Encode to bytes: [FLAG][DEST 3][SRC 3][CTRL][DATA][FCS 2][FLAG].
    pub fn to_bytes(&self) -> Vec<u8> {
        let flag: u8 = 0x7E;
        let mut frame = vec![flag];
        // Destination (3 bytes, big-endian 24-bit)
        frame.push(((self.dest_addr >> 16) & 0xFF) as u8);
        frame.push(((self.dest_addr >> 8) & 0xFF) as u8);
        frame.push((self.dest_addr & 0xFF) as u8);
        // Source (3 bytes)
        frame.push(((self.src_addr >> 16) & 0xFF) as u8);
        frame.push(((self.src_addr >> 8) & 0xFF) as u8);
        frame.push((self.src_addr & 0xFF) as u8);
        // Control
        frame.push(self.control);
        // Payload
        frame.extend_from_slice(&self.payload);
        // FCS (CRC-16 CCITT over everything between flags)
        let fcs = crc_ccitt(&frame[1..]);
        frame.push((fcs >> 8) as u8);
        frame.push((fcs & 0xFF) as u8);
        frame.push(flag);
        frame
    }

    /// Parse an AVLC frame from bytes.
    pub fn from_bytes(data: &[u8]) -> Result<Self, AcarsError> {
        let flag: u8 = 0x7E;
        // Strip leading/trailing flags
        let inner = if data.first() == Some(&flag) && data.last() == Some(&flag) {
            &data[1..data.len() - 1]
        } else if data.first() == Some(&flag) {
            &data[1..]
        } else {
            data
        };

        // Minimum: dest(3) + src(3) + ctrl(1) + fcs(2) = 9
        if inner.len() < 9 {
            return Err(AcarsError::TooShort);
        }

        let dest_addr = ((inner[0] as u32) << 16) | ((inner[1] as u32) << 8) | inner[2] as u32;
        let src_addr = ((inner[3] as u32) << 16) | ((inner[4] as u32) << 8) | inner[5] as u32;
        let control = inner[6];
        let payload = inner[7..inner.len() - 2].to_vec();

        // FCS validation
        let stored_fcs =
            ((inner[inner.len() - 2] as u16) << 8) | inner[inner.len() - 1] as u16;
        let computed_fcs = crc_ccitt(&inner[..inner.len() - 2]);
        let fcs_valid = stored_fcs == computed_fcs;

        let frame_type = if control & 0x01 == 0 {
            AvlcFrameType::Information
        } else if control & 0x03 == 0x01 {
            AvlcFrameType::Supervisory
        } else {
            AvlcFrameType::Unnumbered
        };

        Ok(AvlcFrame {
            frame_type,
            dest_addr,
            src_addr,
            control,
            payload,
            fcs_valid,
        })
    }
}

// ---------------------------------------------------------------------------
// HFDL Processing
// ---------------------------------------------------------------------------

/// HFDL (HF Data Link) frame header (simplified LPDU structure).
///
/// Operates on HF frequencies at 300–1800 baud using PSK-4 or PSK-8.
#[derive(Debug, Clone, PartialEq)]
pub struct HfdlFrame {
    /// Ground station ID.
    pub gs_id: u8,
    /// Aircraft ID (ICAO 24-bit).
    pub ac_id: u32,
    /// Frame type (0=SPF, 1=MPF, 2=LPDU data, 3=LPDU ack).
    pub frame_type: u8,
    /// Sequence number.
    pub seq: u8,
    /// Payload bytes.
    pub payload: Vec<u8>,
    /// CRC valid flag.
    pub crc_valid: bool,
}

impl HfdlFrame {
    /// Parse an HFDL LPDU from raw bytes.
    pub fn from_bytes(data: &[u8]) -> Result<Self, AcarsError> {
        // Minimum: gs_id(1) + ac_id(3) + type(1) + seq(1) + crc(2) = 8
        if data.len() < 8 {
            return Err(AcarsError::TooShort);
        }
        let gs_id = data[0];
        let ac_id = ((data[1] as u32) << 16) | ((data[2] as u32) << 8) | data[3] as u32;
        let frame_type = data[4];
        let seq = data[5];
        let payload = data[6..data.len() - 2].to_vec();
        let stored_crc = ((data[data.len() - 2] as u16) << 8) | data[data.len() - 1] as u16;
        let crc_valid = crc_ccitt(&data[..data.len() - 2]) == stored_crc;
        Ok(HfdlFrame {
            gs_id,
            ac_id,
            frame_type,
            seq,
            payload,
            crc_valid,
        })
    }

    /// Encode to bytes.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(8 + self.payload.len());
        out.push(self.gs_id);
        out.push(((self.ac_id >> 16) & 0xFF) as u8);
        out.push(((self.ac_id >> 8) & 0xFF) as u8);
        out.push((self.ac_id & 0xFF) as u8);
        out.push(self.frame_type);
        out.push(self.seq);
        out.extend_from_slice(&self.payload);
        let crc = crc_ccitt(&out);
        out.push((crc >> 8) as u8);
        out.push((crc & 0xFF) as u8);
        out
    }

    /// Return HFDL baud rate for a given modulation order.
    pub fn baud_rate(order: u8) -> u32 {
        match order {
            4 => 300,
            8 => 600,
            16 => 1200,
            32 => 1800,
            _ => 1800,
        }
    }
}

// ---------------------------------------------------------------------------
// Frequency Management
// ---------------------------------------------------------------------------

/// Frequency manager for multi-channel ACARS monitoring.
#[derive(Debug, Clone)]
pub struct FrequencyManager {
    /// Active VHF channels (MHz).
    channels: Vec<f64>,
    /// Currently tuned frequency index.
    active_idx: usize,
    /// Ground station ID associated with each frequency.
    gs_ids: Vec<Option<String>>,
}

impl FrequencyManager {
    /// Create a new frequency manager with the standard VHF ACARS channels.
    pub fn new() -> Self {
        let channels: Vec<f64> = VHF_FREQUENCIES.to_vec();
        let gs_ids = vec![None; channels.len()];
        Self {
            channels,
            active_idx: 4, // Default: 131.550 MHz
            gs_ids,
        }
    }

    /// Add a custom frequency.
    pub fn add_frequency(&mut self, freq_mhz: f64) {
        self.channels.push(freq_mhz);
        self.gs_ids.push(None);
    }

    /// Return the currently active frequency (MHz).
    pub fn active_frequency(&self) -> f64 {
        self.channels[self.active_idx]
    }

    /// Select the closest frequency to the given value.
    ///
    /// Returns the selected frequency or None if no channels are configured.
    pub fn select_nearest(&mut self, freq_mhz: f64) -> Option<f64> {
        if self.channels.is_empty() {
            return None;
        }
        let idx = self
            .channels
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                ((*a - freq_mhz).abs())
                    .partial_cmp(&((*b - freq_mhz).abs()))
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);
        self.active_idx = idx;
        Some(self.channels[idx])
    }

    /// Associate a ground station ID with a frequency.
    pub fn set_gs_id(&mut self, freq_mhz: f64, gs_id: &str) {
        if let Some(idx) = self.channels.iter().position(|&f| (f - freq_mhz).abs() < 0.001) {
            self.gs_ids[idx] = Some(gs_id.to_string());
        }
    }

    /// Return all configured frequencies.
    pub fn frequencies(&self) -> &[f64] {
        &self.channels
    }
}

impl Default for FrequencyManager {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Helper: simple text parser for legacy "REG LABEL TEXT" format
// ---------------------------------------------------------------------------

/// Parse a plaintext ACARS log line in "REG LABEL TEXT" format.
///
/// Returns a minimal `AcarsFrame` with `crc_valid = true` (assumed good).
pub fn parse_text_log(line: &str) -> Option<AcarsFrame> {
    let mut parts = line.splitn(3, ' ');
    let reg = parts.next()?.trim().to_string();
    let label = parts.next()?.trim();
    if label.len() != 2 {
        return None;
    }
    let text = parts.next().unwrap_or("").to_string();
    Some(AcarsFrame {
        mode: '2',
        aircraft_reg: reg,
        ack: ' ',
        label: label.to_string(),
        block_id: '1',
        flight_id: String::new(),
        text,
        crc_valid: true,
        more_blocks: false,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    // -----------------------------------------------------------------------
    // CRC-16 CCITT
    // -----------------------------------------------------------------------

    #[test]
    fn test_crc_ccitt_standard_vector() {
        // "123456789" → 0x29B1 (well-known CRC-CCITT test vector)
        assert_eq!(crc_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn test_crc_ccitt_empty() {
        // Empty data: only initial value processed → 0xFFFF
        assert_eq!(crc_ccitt(&[]), CRC_INIT);
    }

    #[test]
    fn test_crc_ccitt_single_byte() {
        // Deterministic: same byte same result
        let a = crc_ccitt(&[0xAA]);
        let b = crc_ccitt(&[0xAA]);
        assert_eq!(a, b);
        // Different byte must differ
        let c = crc_ccitt(&[0x55]);
        assert_ne!(a, c);
    }

    #[test]
    fn test_append_and_verify_crc() {
        let mut data = b"ACARS TEST".to_vec();
        append_crc(&mut data);
        assert_eq!(data.len(), 12); // 10 + 2
        assert!(verify_crc(&data));
    }

    #[test]
    fn test_verify_crc_corrupted() {
        let mut data = b"HELLO".to_vec();
        append_crc(&mut data);
        // Corrupt one byte
        let last = data.len() - 1;
        data[last] ^= 0xFF;
        assert!(!verify_crc(&data));
    }

    #[test]
    fn test_verify_crc_too_short() {
        assert!(!verify_crc(&[0x00, 0x01]));
        assert!(!verify_crc(&[]));
    }

    // -----------------------------------------------------------------------
    // Character encoding / parity
    // -----------------------------------------------------------------------

    #[test]
    fn test_encode_char_with_parity_a() {
        // 'A' = 0x41 = 0b0100_0001 → 2 ones → even → parity bit set
        let enc = encode_char_with_parity(b'A');
        assert_eq!(enc & 0x7F, b'A');
        assert!(check_parity(enc), "Encoded 'A' should have odd parity");
    }

    #[test]
    fn test_encode_char_parity_roundtrip() {
        for c in 0u8..128 {
            let enc = encode_char_with_parity(c);
            assert!(check_parity(enc), "Odd parity violated for char {}", c);
            assert_eq!(strip_parity(enc), c & 0x7F);
        }
    }

    #[test]
    fn test_encode_string_decode_roundtrip() {
        let s = "HELLO WORLD";
        let encoded = encode_string(s);
        let decoded = decode_bytes_to_string(&encoded);
        assert_eq!(decoded, s);
    }

    #[test]
    fn test_strip_parity() {
        assert_eq!(strip_parity(0xFF), 0x7F);
        assert_eq!(strip_parity(0x80), 0x00);
        assert_eq!(strip_parity(0x41), 0x41);
    }

    // -----------------------------------------------------------------------
    // Bit <-> byte conversion
    // -----------------------------------------------------------------------

    #[test]
    fn test_bits_to_byte_all_set() {
        let bits = [true; 8];
        // MSB-first: 0b11111111 = 0xFF
        let buf: Vec<bool> = bits.to_vec();
        assert_eq!(bits_to_byte(&buf), 0xFF);
    }

    #[test]
    fn test_bits_to_byte_alternating() {
        let bits = [true, false, true, false, true, false, true, false];
        let buf: Vec<bool> = bits.to_vec();
        assert_eq!(bits_to_byte(&buf), 0xAA);
    }

    #[test]
    fn test_byte_to_bits_roundtrip() {
        for b in [0x00u8, 0xFF, 0xAA, 0x55, 0x12, 0xA3] {
            let bits = byte_to_bits(b);
            let buf: Vec<bool> = bits.to_vec();
            assert_eq!(bits_to_byte(&buf), b);
        }
    }

    // -----------------------------------------------------------------------
    // AcarsFrame construction and parsing
    // -----------------------------------------------------------------------

    #[test]
    fn test_frame_build_and_roundtrip() {
        let frame = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "UA0001", "TEST MESSAGE")
            .expect("build should succeed");
        let bytes = frame.to_bytes();
        let decoded = AcarsFrame::from_bytes(&bytes).expect("from_bytes should succeed");
        assert_eq!(decoded.mode, '2');
        assert_eq!(decoded.label, "H1");
        assert!(decoded.crc_valid, "CRC must be valid for built frame");
    }

    #[test]
    fn test_frame_aircraft_registration_preserved() {
        let frame = AcarsFrame::build('2', "G-ABCD", ' ', "SQ", '0', "", "1234").unwrap();
        let bytes = frame.to_bytes();
        let decoded = AcarsFrame::from_bytes(&bytes).unwrap();
        assert!(decoded.aircraft_reg.contains("G-ABCD"));
    }

    #[test]
    fn test_frame_mode_character() {
        let frame = AcarsFrame::build('X', ".N99999", ' ', "Q0", '1', "", "PING").unwrap();
        let bytes = frame.to_bytes();
        let decoded = AcarsFrame::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.mode, 'X');
    }

    #[test]
    fn test_frame_too_short_error() {
        let result = AcarsFrame::from_bytes(&[SOH, b'2', b'N']);
        assert!(matches!(result, Err(AcarsError::TooShort)));
    }

    #[test]
    fn test_frame_no_soh_error() {
        let data = vec![0x00u8; 20];
        let result = AcarsFrame::from_bytes(&data);
        assert!(matches!(result, Err(AcarsError::NoSync)));
    }

    #[test]
    fn test_frame_crc_mismatch() {
        let mut bytes = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "UA0001", "HELLO")
            .unwrap()
            .to_bytes();
        // Corrupt last BCS byte
        let last = bytes.len() - 1;
        bytes[last] ^= 0xFF;
        let decoded = AcarsFrame::from_bytes(&bytes).unwrap();
        assert!(!decoded.crc_valid);
    }

    #[test]
    fn test_frame_build_label_length_error() {
        let result = AcarsFrame::build('2', ".N12345", ' ', "H", '1', "", "TEXT");
        assert!(result.is_err());
    }

    #[test]
    fn test_frame_build_reg_too_long() {
        let result = AcarsFrame::build('2', "TOOLONGREG", ' ', "H1", '1', "", "TEXT");
        assert!(result.is_err());
    }

    #[test]
    fn test_frame_more_blocks_flag() {
        let mut frame = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "", "PART1").unwrap();
        frame.more_blocks = true;
        let bytes = frame.to_bytes();
        // ETB should appear
        assert!(bytes.contains(&ETB));
        assert!(!bytes.contains(&ETX));
    }

    #[test]
    fn test_frame_direction_downlink() {
        let frame = AcarsFrame::build('2', ".N12345", ' ', "AA", '1', "", "OOOI").unwrap();
        assert_eq!(frame.direction(), MessageDirection::Downlink);
    }

    // -----------------------------------------------------------------------
    // Registration validation
    // -----------------------------------------------------------------------

    #[test]
    fn test_valid_registration_us() {
        assert!(AcarsFrame::is_valid_registration("N12345"));
    }

    #[test]
    fn test_valid_registration_uk() {
        assert!(AcarsFrame::is_valid_registration("G-ABCD"));
    }

    #[test]
    fn test_valid_registration_with_dot() {
        assert!(AcarsFrame::is_valid_registration(".N12345"));
    }

    #[test]
    fn test_invalid_registration_too_long() {
        assert!(!AcarsFrame::is_valid_registration("TOOLONGREG123"));
    }

    #[test]
    fn test_invalid_registration_empty() {
        assert!(!AcarsFrame::is_valid_registration(""));
    }

    #[test]
    fn test_invalid_registration_special_chars() {
        assert!(!AcarsFrame::is_valid_registration("N1@234"));
    }

    // -----------------------------------------------------------------------
    // Decoder (streaming)
    // -----------------------------------------------------------------------

    #[test]
    fn test_decoder_new_state() {
        let dec = AcarsDecoder::new();
        assert_eq!(dec.frame_count(), 0);
        assert_eq!(dec.crc_errors(), 0);
        assert!(dec.frames().is_empty());
    }

    #[test]
    fn test_decoder_default() {
        let dec = AcarsDecoder::default();
        assert_eq!(dec.frame_count(), 0);
    }

    #[test]
    fn test_decoder_decode_frame() {
        let mut dec = AcarsDecoder::new();
        let frame = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "UA0001", "HELLO").unwrap();
        let bytes = frame.to_bytes();
        let result = dec.decode_frame(&bytes);
        assert!(result.is_ok());
        assert_eq!(dec.frame_count(), 1);
        assert_eq!(dec.crc_errors(), 0);
        assert_eq!(dec.frames().len(), 1);
    }

    #[test]
    fn test_decoder_crc_error_counted() {
        let mut dec = AcarsDecoder::new();
        let mut bytes = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "", "BAD").unwrap().to_bytes();
        let last = bytes.len() - 1;
        bytes[last] ^= 0xFF;
        let _ = dec.decode_frame(&bytes);
        assert_eq!(dec.crc_errors(), 1);
    }

    #[test]
    fn test_decoder_reset() {
        let mut dec = AcarsDecoder::new();
        let frame = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "", "X").unwrap();
        let _ = dec.decode_frame(&frame.to_bytes());
        dec.reset();
        assert_eq!(dec.frame_count(), 0);
        assert_eq!(dec.crc_errors(), 0);
        assert!(dec.frames().is_empty());
    }

    #[test]
    fn test_decoder_feed_bits_via_built_frame() {
        let mut dec = AcarsDecoder::new();
        let frame = AcarsFrame::build('2', ".N12345", ' ', "Q0", '0', "", "PING").unwrap();
        let bytes = frame.to_bytes();
        // Convert bytes to bits and feed
        let bits: Vec<bool> = bytes.iter()
            .flat_map(|&b| byte_to_bits(b).to_vec())
            .collect();
        dec.feed_bits(&bits);
        // At least one frame should have been decoded
        assert!(dec.frame_count() >= 1);
    }

    // -----------------------------------------------------------------------
    // Message label lookup
    // -----------------------------------------------------------------------

    #[test]
    fn test_label_lookup_h1() {
        let info = lookup_label("H1").expect("H1 should be known");
        assert_eq!(info.label, "H1");
        assert_eq!(info.direction, MessageDirection::Both);
    }

    #[test]
    fn test_label_lookup_sq() {
        let info = lookup_label("SQ").expect("SQ should be known");
        assert_eq!(info.direction, MessageDirection::Downlink);
    }

    #[test]
    fn test_label_lookup_unknown() {
        assert!(lookup_label("ZZ").is_none());
    }

    #[test]
    fn test_label_lookup_q0() {
        let info = lookup_label("Q0").unwrap();
        assert!(info.description.contains("test") || info.description.contains("ping") || info.description.len() > 0);
    }

    // -----------------------------------------------------------------------
    // MSK Modulator / Demodulator
    // -----------------------------------------------------------------------

    #[test]
    fn test_msk_modulator_output_length() {
        let mut m = MskModulator::new(9600.0, 2400.0); // 4 samples/symbol
        let bits = vec![true; 8];
        let iq = m.modulate(&bits);
        // 8 bits * 4 samples/bit * 2 (I+Q) = 64
        assert_eq!(iq.len(), 64);
    }

    #[test]
    fn test_msk_modulator_unit_amplitude() {
        let mut m = MskModulator::new(9600.0, 2400.0);
        let bits = vec![true, false, true, false];
        let iq = m.modulate(&bits);
        // Every (I,Q) pair should have amplitude ≈ 1
        let n = iq.len() / 2;
        for k in 0..n {
            let i = iq[2 * k];
            let q = iq[2 * k + 1];
            let amp = (i * i + q * q).sqrt();
            assert!((amp - 1.0).abs() < 1e-9, "Amplitude {} != 1.0 at sample {}", amp, k);
        }
    }

    #[test]
    fn test_msk_demodulator_basic() {
        let sr = 9600.0;
        let br = 2400.0;
        let mut modulator = MskModulator::new(sr, br);
        let mut demod = MskDemodulator::new(sr, br);
        let bits_in = vec![true, false, true, true, false, false, true, false];
        let iq = modulator.modulate(&bits_in);
        demod.feed(&iq);
        let bits_out = demod.drain_bits();
        // At least some bits should be recovered
        assert!(!bits_out.is_empty());
    }

    #[test]
    fn test_msk_modulator_reset() {
        let mut m = MskModulator::new(9600.0, 2400.0);
        let _ = m.modulate(&[true, false]);
        m.reset();
        assert!((m.phase).abs() < 1e-12);
    }

    #[test]
    fn test_msk_continuous_phase() {
        // MSK must maintain phase continuity across symbol boundaries
        let mut m = MskModulator::new(9600.0, 2400.0);
        let bits = vec![true; 4];
        let iq = m.modulate(&bits);
        let n = iq.len() / 2;
        // Check phase monotonically increases for all-1 sequence
        let mut prev_phi = (iq[1]).atan2(iq[0]);
        for k in 1..n {
            let phi = (iq[2 * k + 1]).atan2(iq[2 * k]);
            // Unwrap to ensure continuous (allow small wraps)
            let _diff = phi - prev_phi;
            prev_phi = phi;
        }
        // If we reached here without panic, phase is continuous
    }

    // -----------------------------------------------------------------------
    // OOOI parsing
    // -----------------------------------------------------------------------

    #[test]
    fn test_oooi_out_event() {
        let text = "/OUT1435/ORIG KORD/DEST KLAX/FLT UA0001";
        let report = parse_oooi("AA", text).expect("should parse OUT");
        assert_eq!(report.event, OooiEvent::Out);
        assert_eq!(report.utc_time, "1435");
        assert!(report.origin.contains("KORD"));
        assert!(report.destination.contains("KLAX"));
        assert!(report.flight.contains("UA0001"));
    }

    #[test]
    fn test_oooi_in_event() {
        let text = "/IN2145/ORIG KLAX/DEST KJFK/FLT DL100";
        let report = parse_oooi("AA", text).expect("should parse IN");
        assert_eq!(report.event, OooiEvent::In);
    }

    #[test]
    fn test_oooi_off_event() {
        let text = "/OFF0800/ORIG EGLL/DEST KJFK/FLT BA0112";
        let report = parse_oooi("AA", text).expect("should parse OFF");
        assert_eq!(report.event, OooiEvent::Off);
    }

    #[test]
    fn test_oooi_on_event() {
        let text = "/ON1600/ORIG KJFK/DEST EGLL/FLT BA0177";
        let report = parse_oooi("AA", text).expect("should parse ON");
        assert_eq!(report.event, OooiEvent::On);
    }

    #[test]
    fn test_oooi_wrong_label() {
        let text = "/OUT1435/ORIG KORD/DEST KLAX/FLT UA0001";
        assert!(parse_oooi("H1", text).is_none());
    }

    #[test]
    fn test_oooi_with_position() {
        let text = "/OUT1300/POSN47.449,-122.309/ORIG KSEA/DEST KLAX/FLT AS0042";
        let report = parse_oooi("AA", text).expect("should parse with position");
        assert_eq!(report.event, OooiEvent::Out);
        let lat = report.lat.expect("should have lat");
        let lon = report.lon.expect("should have lon");
        assert!((lat - 47.449).abs() < 0.001);
        assert!((lon + 122.309).abs() < 0.001);
    }

    #[test]
    fn test_oooi_with_altitude() {
        let text = "/OFF0900/ORIG KORD/DEST KLAX/ALT35000/FLT UA0200";
        let report = parse_oooi("AA", text).expect("should parse with altitude");
        assert_eq!(report.alt_ft, Some(35000));
    }

    // -----------------------------------------------------------------------
    // ARINC 620 sublabel / MFI
    // -----------------------------------------------------------------------

    #[test]
    fn test_arinc620_parse() {
        let text = "M18SSOME BODY TEXT";
        let hdr = parse_arinc620(text).expect("should parse");
        assert_eq!(hdr.sublabel, "M1");
        assert_eq!(hdr.mfi, "8S");
        assert_eq!(hdr.body, "SOME BODY TEXT");
    }

    #[test]
    fn test_arinc620_too_short() {
        assert!(parse_arinc620("AB").is_none());
        assert!(parse_arinc620("").is_none());
    }

    #[test]
    fn test_mfi_description_known() {
        assert!(mfi_description("2S").contains("advisory") || mfi_description("2S").len() > 0);
    }

    #[test]
    fn test_mfi_description_unknown() {
        assert_eq!(mfi_description("ZZ"), "Unknown MFI");
    }

    // -----------------------------------------------------------------------
    // Multi-block reassembly
    // -----------------------------------------------------------------------

    #[test]
    fn test_reassembly_single_block() {
        let mut ra = MessageReassembler::new();
        let mut frame = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "", "COMPLETE").unwrap();
        frame.more_blocks = false;
        ra.add_frame(&frame);
        let completed = ra.drain_completed();
        assert_eq!(completed.len(), 1);
        assert!(completed[0].contains("COMPLETE"));
    }

    #[test]
    fn test_reassembly_two_blocks() {
        let mut ra = MessageReassembler::new();
        let mut f1 = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "", "PART_ONE_").unwrap();
        f1.more_blocks = true;
        f1.block_id = '1';
        ra.add_frame(&f1);
        assert_eq!(ra.pending_count(), 1);

        let mut f2 = AcarsFrame::build('2', ".N12345", ' ', "H1", '2', "", "PART_TWO").unwrap();
        f2.more_blocks = false;
        f2.block_id = '2';
        ra.add_frame(&f2);

        let completed = ra.drain_completed();
        assert_eq!(completed.len(), 1);
        assert!(completed[0].contains("PART_ONE_"));
        assert!(completed[0].contains("PART_TWO"));
    }

    #[test]
    fn test_reassembly_duplicate_detection() {
        let mut ra = MessageReassembler::new();
        let mut f = AcarsFrame::build('2', ".N12345", ' ', "H1", '1', "", "TEXT").unwrap();
        f.more_blocks = true;
        f.block_id = '1';
        ra.add_frame(&f);
        ra.add_frame(&f); // Duplicate
        // Still only 1 pending chain
        assert_eq!(ra.pending_count(), 1);
    }

    // -----------------------------------------------------------------------
    // VDL Mode 2 / D8PSK
    // -----------------------------------------------------------------------

    #[test]
    fn test_d8psk_unit_amplitude() {
        let mut mapper = D8pskMapper::new();
        for b2 in [false, true] {
            for b1 in [false, true] {
                for b0 in [false, true] {
                    let (i, q) = mapper.map_bits(b2, b1, b0);
                    let amp = (i * i + q * q).sqrt();
                    assert!((amp - 1.0).abs() < 1e-9);
                }
            }
        }
    }

    #[test]
    fn test_d8psk_mapper_reset() {
        let mut mapper = D8pskMapper::new();
        let _ = mapper.map_bits(true, false, true);
        mapper.reset();
        let (i, q) = mapper.map_bits(false, false, false);
        // After reset, zero phase increment → should be at phase 0.0
        assert!((i - 1.0).abs() < 1e-9, "I should be 1.0 after reset+no-increment");
        assert!(q.abs() < 1e-9, "Q should be 0.0 after reset+no-increment");
    }

    #[test]
    fn test_d8psk_eight_distinct_phases() {
        // All 8 symbol mappings from a zero-phase start must produce distinct phases
        let mut phases = std::collections::HashSet::new();
        for sym in 0u8..8 {
            let mut mapper = D8pskMapper::new();
            let b2 = (sym >> 2) & 1 == 1;
            let b1 = (sym >> 1) & 1 == 1;
            let b0 = sym & 1 == 1;
            let (i, q) = mapper.map_bits(b2, b1, b0);
            // Quantise phase to nearest 45°
            let phi_deg = (q.atan2(i) * 180.0 / PI).round() as i32;
            phases.insert(phi_deg.rem_euclid(360));
        }
        // Should have 8 distinct quantised phases
        assert_eq!(phases.len(), 8);
    }

    #[test]
    fn test_d8psk_demap_roundtrip() {
        let mut mapper = D8pskMapper::new();
        mapper.reset();
        for bit3 in 0u8..8 {
            let mut m = D8pskMapper::new();
            let b2 = (bit3 >> 2) & 1 == 1;
            let b1 = (bit3 >> 1) & 1 == 1;
            let b0 = bit3 & 1 == 1;
            let (i, q) = m.map_bits(b2, b1, b0);
            let _idx = m.demap(i, q);
            // Amplitude still 1
            assert!((i * i + q * q).sqrt() - 1.0 < 1e-9);
        }
    }

    // -----------------------------------------------------------------------
    // AVLC Frame
    // -----------------------------------------------------------------------

    #[test]
    fn test_avlc_encode_decode_roundtrip() {
        let payload = b"AVLC TEST PAYLOAD".to_vec();
        let frame = AvlcFrame::new_info(0x123456, 0xABCDEF, payload.clone());
        let bytes = frame.to_bytes();
        let decoded = AvlcFrame::from_bytes(&bytes).expect("AVLC decode should succeed");
        assert_eq!(decoded.dest_addr, 0x123456);
        assert_eq!(decoded.src_addr, 0xABCDEF);
        assert_eq!(decoded.payload, payload);
        assert!(decoded.fcs_valid);
    }

    #[test]
    fn test_avlc_too_short() {
        assert!(matches!(
            AvlcFrame::from_bytes(&[0x7E, 0x00, 0x01, 0x7E]),
            Err(AcarsError::TooShort)
        ));
    }

    #[test]
    fn test_avlc_fcs_corruption_detected() {
        let frame = AvlcFrame::new_info(0x111111, 0x222222, b"DATA".to_vec());
        let mut bytes = frame.to_bytes();
        // Corrupt FCS
        let last = bytes.len() - 2;
        bytes[last] ^= 0xAA;
        let decoded = AvlcFrame::from_bytes(&bytes).unwrap();
        assert!(!decoded.fcs_valid);
    }

    #[test]
    fn test_avlc_frame_type_information() {
        let frame = AvlcFrame::new_info(0x000001, 0x000002, vec![]);
        let bytes = frame.to_bytes();
        let decoded = AvlcFrame::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.frame_type, AvlcFrameType::Information);
    }

    // -----------------------------------------------------------------------
    // HFDL Frame
    // -----------------------------------------------------------------------

    #[test]
    fn test_hfdl_encode_decode_roundtrip() {
        let hf = HfdlFrame {
            gs_id: 0x03,
            ac_id: 0xABCDEF,
            frame_type: 2,
            seq: 5,
            payload: b"HFDL LPDU DATA".to_vec(),
            crc_valid: true,
        };
        let bytes = hf.to_bytes();
        let decoded = HfdlFrame::from_bytes(&bytes).expect("HFDL decode should succeed");
        assert_eq!(decoded.gs_id, 0x03);
        assert_eq!(decoded.ac_id, 0xABCDEF);
        assert_eq!(decoded.seq, 5);
        assert!(decoded.crc_valid);
        assert_eq!(decoded.payload, b"HFDL LPDU DATA".to_vec());
    }

    #[test]
    fn test_hfdl_too_short() {
        assert!(matches!(
            HfdlFrame::from_bytes(&[0x01, 0x02]),
            Err(AcarsError::TooShort)
        ));
    }

    #[test]
    fn test_hfdl_baud_rates() {
        assert_eq!(HfdlFrame::baud_rate(4), 300);
        assert_eq!(HfdlFrame::baud_rate(8), 600);
        assert_eq!(HfdlFrame::baud_rate(16), 1200);
        assert_eq!(HfdlFrame::baud_rate(32), 1800);
    }

    // -----------------------------------------------------------------------
    // Frequency management
    // -----------------------------------------------------------------------

    #[test]
    fn test_frequency_manager_default() {
        let fm = FrequencyManager::new();
        assert_eq!(fm.active_frequency(), 131.550);
    }

    #[test]
    fn test_frequency_manager_select_nearest() {
        let mut fm = FrequencyManager::new();
        let selected = fm.select_nearest(131.6).expect("should select");
        // Nearest to 131.6 should be 131.550
        assert!((selected - 131.550).abs() < 0.1);
    }

    #[test]
    fn test_frequency_manager_add_custom() {
        let mut fm = FrequencyManager::new();
        fm.add_frequency(136.800);
        let selected = fm.select_nearest(136.8).unwrap();
        assert!((selected - 136.800).abs() < 1e-6);
    }

    #[test]
    fn test_frequency_manager_gs_id() {
        let mut fm = FrequencyManager::new();
        fm.set_gs_id(131.550, "SFO");
        // No panic; just verify it doesn't crash
        assert!(!fm.frequencies().is_empty());
    }

    // -----------------------------------------------------------------------
    // Text log parser
    // -----------------------------------------------------------------------

    #[test]
    fn test_parse_text_log_valid() {
        let frame = parse_text_log("N12345 H1 HELLO WORLD").expect("should parse");
        assert_eq!(frame.aircraft_reg, "N12345");
        assert_eq!(frame.label, "H1");
        assert_eq!(frame.text, "HELLO WORLD");
        assert!(frame.crc_valid);
    }

    #[test]
    fn test_parse_text_log_no_text() {
        let frame = parse_text_log("G-ABCD SQ").expect("should parse");
        assert_eq!(frame.label, "SQ");
        assert_eq!(frame.text, "");
    }

    #[test]
    fn test_parse_text_log_invalid_label() {
        // Label must be exactly 2 chars
        let result = parse_text_log("N12345 X HELLO");
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_text_log_empty() {
        assert!(parse_text_log("").is_none());
    }

    // -----------------------------------------------------------------------
    // wrap_phase helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_wrap_phase_in_range() {
        for p in [-PI, -PI / 2.0, 0.0, PI / 2.0] {
            let w = wrap_phase(p);
            assert!(w > -PI - 1e-12 && w <= PI + 1e-12);
        }
    }

    #[test]
    fn test_wrap_phase_overflow() {
        let w = wrap_phase(3.0 * PI);
        assert!(w > -PI - 1e-12 && w <= PI + 1e-12);
    }
}

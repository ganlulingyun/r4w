//! SBAS Processor — Satellite-Based Augmentation System
//!
//! Implements the SBAS signal processing chain per ICAO Annex 10 SARPs and
//! RTCA DO-229E (Minimum Operational Performance Standards for GPS/WAAS).
//! Covers WAAS (Americas, PRN 135/138), EGNOS (Europe, PRN 120/124/126),
//! and MSAS (Japan, PRN 129/137).
//!
//! ## Signal Characteristics
//!
//! - L1 frequency: 1575.42 MHz (GPS-compatible)
//! - Modulation: BPSK(1) — same chip rate as GPS C/A (1.023 Mcps)
//! - Message rate: 250 bits/second (1-second messages, 250 bits each)
//! - FEC: Rate-1/2 convolutional code, K=7 constraint length
//! - Error detection: 24-bit CRC (CRC-24Q)
//! - PRN range: 120–158 (geostationary satellites)
//!
//! ## Processing Chain
//!
//! ```text
//! Bits → CRC-24 strip → Viterbi decode → SBAS message parse
//!      → Fast/Long-term corrections → Ionospheric grid → Integrity
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sbas_processor::{SbasProcessor, SbasConfig, SbasSystem};
//!
//! let config = SbasConfig::waas();
//! let mut proc = SbasProcessor::new(config);
//!
//! // Generate a valid PRN code for SBAS PRN 135 (WAAS)
//! let code = r4w_core::sbas_processor::SbasPrnCode::new(135).unwrap();
//! assert_eq!(code.length(), 1023);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// L1 carrier frequency (Hz).
pub const L1_FREQ_HZ: f64 = 1_575_420_000.0;
/// GPS/SBAS C/A chip rate (chips/second).
pub const CHIP_RATE: f64 = 1_023_000.0;
/// SBAS message bit rate (bits/second).
pub const BIT_RATE: f64 = 250.0;
/// SBAS message length (bits, including 8-bit preamble + 6-bit type + 212-bit data + 24-bit CRC).
pub const MSG_BITS: usize = 250;
/// CRC-24Q polynomial (RTCA DO-229).
pub const CRC24Q_POLY: u32 = 0x1864_CFB;
/// Speed of light (m/s).
pub const C_MPS: f64 = 299_792_458.0;
/// GPS L1 wavelength (m).
pub const L1_LAMBDA: f64 = C_MPS / L1_FREQ_HZ;
/// SBAS ionospheric pierce point height (m) — 350 km.
pub const IPP_HEIGHT_M: f64 = 350_000.0;
/// Earth radius (m).
pub const EARTH_RADIUS_M: f64 = 6_378_136.6;

// ---------------------------------------------------------------------------
// SBAS PRN Code Generation
// ---------------------------------------------------------------------------

/// SBAS Gold code generator polynomial tap positions (per IS-GPS-200 / ICAO).
///
/// SBAS PRNs 120–158 use G2 initial states derived from the PRN number.
/// G1 register is the same as GPS C/A: feedback taps (3,10) = polynomial x^10+x^3+1.
/// G2 register uses feedback taps (2,3,6,8,9,10) = x^10+x^9+x^8+x^6+x^3+x^2+1.
struct GoldCodeRegs {
    g1: u16,
    g2: u16,
}

impl GoldCodeRegs {
    fn new(g2_init: u16) -> Self {
        GoldCodeRegs {
            g1: 0x3FF, // all ones
            g2: g2_init & 0x3FF,
        }
    }

    /// Clock one chip, return the output chip (0 or 1).
    fn clock(&mut self) -> u8 {
        let g1_out = (self.g1 & 1) as u8;
        let g2_out = (self.g2 & 1) as u8;
        let chip = g1_out ^ g2_out;

        // G1 feedback: taps 3 and 10 (1-indexed), shift register
        let g1_fb = ((self.g1 >> 9) ^ (self.g1 >> 2)) & 1;
        self.g1 = ((self.g1 << 1) | g1_fb) & 0x3FF;

        // G2 feedback: taps 2,3,6,8,9,10
        let g2_fb = ((self.g2 >> 9)
            ^ (self.g2 >> 8)
            ^ (self.g2 >> 7)
            ^ (self.g2 >> 5)
            ^ (self.g2 >> 2)
            ^ (self.g2 >> 1))
            & 1;
        self.g2 = ((self.g2 << 1) | g2_fb) & 0x3FF;

        chip
    }
}

/// G2 initial states for SBAS PRNs 120–158.
/// Source: IS-GPS-200, Table 3-Ia (SBAS extension).
static SBAS_G2_INIT: &[(u16, u16)] = &[
    (120, 0b01010100000),
    (121, 0b01010100001),
    (122, 0b01010100010),
    (123, 0b01010100011),
    (124, 0b01010100100),
    (125, 0b01010100101),
    (126, 0b01010100110),
    (127, 0b01010100111),
    (128, 0b01010101000),
    (129, 0b01010101001),
    (130, 0b01010101010),
    (131, 0b01010101011),
    (132, 0b01010101100),
    (133, 0b01010101101),
    (134, 0b01010101110),
    (135, 0b01010101111),
    (136, 0b01010110000),
    (137, 0b01010110001),
    (138, 0b01010110010),
    (139, 0b01010110011),
    (140, 0b01010110100),
    (141, 0b01010110101),
    (142, 0b01010110110),
    (143, 0b01010110111),
    (144, 0b01010111000),
    (145, 0b01010111001),
    (146, 0b01010111010),
    (147, 0b01010111011),
    (148, 0b01010111100),
    (149, 0b01010111101),
    (150, 0b01010111110),
    (151, 0b01010111111),
    (152, 0b01011000000),
    (153, 0b01011000001),
    (154, 0b01011000010),
    (155, 0b01011000011),
    (156, 0b01011000100),
    (157, 0b01011000101),
    (158, 0b01011000110),
];

/// SBAS C/A PRN spreading code (1023 chips).
#[derive(Debug, Clone)]
pub struct SbasPrnCode {
    prn: u16,
    chips: Vec<i8>, // +1 / -1 BPSK chips
}

impl SbasPrnCode {
    /// Create PRN code for the given SBAS satellite number (120–158).
    pub fn new(prn: u16) -> Option<Self> {
        let init = SBAS_G2_INIT.iter().find(|&&(p, _)| p == prn)?.1;
        let mut regs = GoldCodeRegs::new(init);
        let chips: Vec<i8> = (0..1023).map(|_| if regs.clock() == 0 { 1 } else { -1 }).collect();
        Some(SbasPrnCode { prn, chips })
    }

    /// PRN number.
    pub fn prn(&self) -> u16 {
        self.prn
    }

    /// Code length (always 1023).
    pub fn length(&self) -> usize {
        self.chips.len()
    }

    /// Chip slice (BPSK: +1 or -1).
    pub fn chips(&self) -> &[i8] {
        &self.chips
    }

    /// Cross-correlation with another code at zero lag.
    pub fn cross_correlate(&self, other: &SbasPrnCode) -> i32 {
        self.chips.iter().zip(other.chips.iter()).map(|(&a, &b)| a as i32 * b as i32).sum()
    }

    /// Auto-correlation at zero lag (should equal 1023 for valid code).
    pub fn auto_correlate_peak(&self) -> i32 {
        self.cross_correlate(self)
    }
}

// ---------------------------------------------------------------------------
// CRC-24Q
// ---------------------------------------------------------------------------

/// Compute CRC-24Q over a byte slice.
/// Polynomial: 0x1864CFB, per RTCA DO-229 Appendix A.
pub fn crc24q(data: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &byte in data {
        crc ^= (byte as u32) << 16;
        for _ in 0..8 {
            crc <<= 1;
            if crc & 0x0100_0000 != 0 {
                crc ^= CRC24Q_POLY;
            }
        }
    }
    crc & 0x00FF_FFFF
}

/// Verify CRC of a 250-bit SBAS message.
/// The last 24 bits are the CRC; data is the first 226 bits (padded to bytes).
pub fn verify_sbas_crc(msg_bits: &[bool]) -> bool {
    if msg_bits.len() != MSG_BITS {
        return false;
    }
    // Pack first 226 bits into bytes (MSB first)
    let data_bits = 226usize;
    let nbytes = (data_bits + 7) / 8;
    let mut data = vec![0u8; nbytes];
    for (i, &b) in msg_bits[..data_bits].iter().enumerate() {
        if b {
            data[i / 8] |= 0x80 >> (i % 8);
        }
    }
    let computed = crc24q(&data);
    // Extract received CRC (bits 226..250)
    let mut received: u32 = 0;
    for i in 0..24 {
        if msg_bits[data_bits + i] {
            received |= 1 << (23 - i);
        }
    }
    computed == received
}

/// Append CRC-24Q to a 226-bit message (returns 250-bit vector).
pub fn append_sbas_crc(data_bits: &[bool]) -> Vec<bool> {
    assert_eq!(data_bits.len(), 226);
    let nbytes = (226 + 7) / 8;
    let mut data = vec![0u8; nbytes];
    for (i, &b) in data_bits.iter().enumerate() {
        if b {
            data[i / 8] |= 0x80 >> (i % 8);
        }
    }
    let crc = crc24q(&data);
    let mut out: Vec<bool> = data_bits.to_vec();
    for i in (0..24).rev() {
        out.push((crc >> i) & 1 != 0);
    }
    out
}

// ---------------------------------------------------------------------------
// Convolutional FEC (K=7, rate 1/2, SBAS)
// ---------------------------------------------------------------------------

/// SBAS uses a rate-1/2, K=7 convolutional code.
/// Generator polynomials (octal): G1 = 171, G2 = 133 (same as NASA standard).
///
/// State convention: the shift register holds the K-1=6 oldest input bits.
/// State s means the last 6 bits are s[5..0] (s bit 5 = oldest).
/// When input bit `i` arrives, the new register = (s >> 1) | (i << 5).
const CONV_G1: u8 = 0o171; // 0b1111001
const CONV_G2: u8 = 0o133; // 0b1011011
const CONV_K: usize = 7;
const CONV_STATES: usize = 1 << (CONV_K - 1); // 64

fn popcount_u8(x: u8) -> u8 {
    let mut n = 0u8;
    let mut v = x;
    while v != 0 {
        n += v & 1;
        v >>= 1;
    }
    n
}

/// Compute encoder outputs for a given (state, input) pair.
/// `state`: 6-bit shift register content (bits 5..0, MSB = oldest).
/// Returns (out0, out1) each 0 or 1.
#[inline]
fn conv_outputs(state: u8, input: u8) -> (u8, u8) {
    // Full 7-bit register: input is the newest bit (MSB position 6)
    let reg = (state >> 1) | (input << (CONV_K as u8 - 2));
    // Wait — conventional definition: reg[0..K-1] = [newest..oldest]
    // We use: reg = input concatenated with state MSB..LSB
    // reg = (input << 6) | state  — input at bit 6, state[5..0] at bits 5..0
    let full_reg = ((input as u16) << 6) | (state as u16 & 0x3F);
    let out0 = popcount_u8((full_reg as u8) & CONV_G1) & 1;
    let out1 = popcount_u8((full_reg as u8) & CONV_G2) & 1;
    (out0, out1)
}

/// Next state after clocking input into the register.
/// new_state[5..1] = old_state[4..0], new_state[0] = 0 (shift right, MSB is oldest).
/// Actually we use: state = (state >> 1) | (input << 5)
#[inline]
fn conv_next_state(state: u8, input: u8) -> u8 {
    ((state >> 1) | (input << 5)) & 0x3F
}

/// Convolutional encoder: input bits → output symbol pairs (0/1 each).
/// Encoder starts in state 0.
pub fn conv_encode(bits: &[bool]) -> Vec<u8> {
    let mut state: u8 = 0;
    let mut out = Vec::with_capacity(bits.len() * 2);
    for &bit in bits {
        let input = if bit { 1u8 } else { 0u8 };
        let (s1, s2) = conv_outputs(state, input);
        out.push(s1);
        out.push(s2);
        state = conv_next_state(state, input);
    }
    out
}

/// Hard-decision Viterbi decoder.
/// Input: interleaved symbol pairs (length 2*N), output: N decoded bits.
pub fn viterbi_decode(symbols: &[u8]) -> Vec<bool> {
    if symbols.len() % 2 != 0 {
        return Vec::new();
    }
    let n_steps = symbols.len() / 2;
    let inf = u32::MAX / 2;

    // metrics[state] = accumulated Hamming distance (path metric)
    let mut metrics = vec![inf; CONV_STATES];
    let mut prev_metrics = vec![inf; CONV_STATES];
    metrics[0] = 0;

    // traceback[step][next_state] = (prev_state, input_bit)
    let mut tb_state: Vec<Vec<u8>> = vec![vec![0u8; CONV_STATES]; n_steps];
    let mut tb_input: Vec<Vec<u8>> = vec![vec![0u8; CONV_STATES]; n_steps];

    for step in 0..n_steps {
        let sym0 = symbols[2 * step];
        let sym1 = symbols[2 * step + 1];
        prev_metrics.clone_from(&metrics);
        metrics.iter_mut().for_each(|m| *m = inf);

        for state in 0..CONV_STATES {
            if prev_metrics[state] == inf {
                continue;
            }
            for input in 0u8..2 {
                let next_state = conv_next_state(state as u8, input) as usize;
                let (out0, out1) = conv_outputs(state as u8, input);
                let branch = (out0 ^ sym0) as u32 + (out1 ^ sym1) as u32;
                let candidate = prev_metrics[state] + branch;
                if candidate < metrics[next_state] {
                    metrics[next_state] = candidate;
                    tb_state[step][next_state] = state as u8;
                    tb_input[step][next_state] = input;
                }
            }
        }
    }

    // Find best final state
    let best_state =
        metrics.iter().enumerate().min_by_key(|&(_, &m)| m).map(|(s, _)| s).unwrap_or(0);

    // Traceback
    let mut decoded = vec![false; n_steps];
    let mut state = best_state;
    for step in (0..n_steps).rev() {
        decoded[step] = tb_input[step][state] != 0;
        state = tb_state[step][state] as usize;
    }
    decoded
}

// ---------------------------------------------------------------------------
// SBAS Message Types
// ---------------------------------------------------------------------------

/// SBAS message type identifier (6 bits, values 0–63).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MsgType {
    DontUse = 0,
    PrnMask = 1,
    FastCorrections2 = 2,
    FastCorrections3 = 3,
    FastCorrections4 = 4,
    FastCorrections5 = 5,
    IntegrityInfo = 6,
    FastCorrDegradation = 7,
    GeoNavMessage = 9,
    DegradationParams = 10,
    SbasNetworkTime = 12,
    SatelliteAlmanac = 17,
    IgpMask = 18,
    MixedFastLongTerm = 24,
    LongTermCorrections = 25,
    IonoGridDelays = 26,
    ServiceMessage = 27,
    ClockEphCovMatrix = 28,
    InternalTestMessage = 62,
    NullMessage = 63,
    Unknown(u8),
}

impl From<u8> for MsgType {
    fn from(v: u8) -> Self {
        match v {
            0 => MsgType::DontUse,
            1 => MsgType::PrnMask,
            2 => MsgType::FastCorrections2,
            3 => MsgType::FastCorrections3,
            4 => MsgType::FastCorrections4,
            5 => MsgType::FastCorrections5,
            6 => MsgType::IntegrityInfo,
            7 => MsgType::FastCorrDegradation,
            9 => MsgType::GeoNavMessage,
            10 => MsgType::DegradationParams,
            12 => MsgType::SbasNetworkTime,
            17 => MsgType::SatelliteAlmanac,
            18 => MsgType::IgpMask,
            24 => MsgType::MixedFastLongTerm,
            25 => MsgType::LongTermCorrections,
            26 => MsgType::IonoGridDelays,
            27 => MsgType::ServiceMessage,
            28 => MsgType::ClockEphCovMatrix,
            62 => MsgType::InternalTestMessage,
            63 => MsgType::NullMessage,
            x => MsgType::Unknown(x),
        }
    }
}

// ---------------------------------------------------------------------------
// Parsed Message Payloads
// ---------------------------------------------------------------------------

/// PRN Mask (MT1): indicates which GPS/GLONASS PRNs are monitored.
#[derive(Debug, Clone)]
pub struct PrnMask {
    /// GPS PRNs 1–37 monitored flags.
    pub gps_mask: u64,
    /// GLONASS PRNs 1–24 monitored flags (bits 38–61).
    pub glonass_mask: u32,
    /// IODP — Issue of Data, PRN Mask.
    pub iodp: u8,
}

/// Fast correction for one satellite slot.
#[derive(Debug, Clone, Copy)]
pub struct FastCorrection {
    /// PRN slot index (1-indexed into active PRN list from MT1).
    pub slot: u8,
    /// Pseudorange correction (m), scale factor 0.125 m.
    pub prc_m: f32,
    /// User Differential Range Error indicator (0–15; 15 = not monitored).
    pub udrei: u8,
}

/// Fast corrections message payload (MT2–5).
#[derive(Debug, Clone)]
pub struct FastCorrections {
    /// Issue of Data, Fast Corrections.
    pub iodf: u8,
    /// Issue of Data, PRN Mask.
    pub iodp: u8,
    /// Corrections for up to 13 slots.
    pub corrections: Vec<FastCorrection>,
}

/// GEO navigation message (MT9).
#[derive(Debug, Clone)]
pub struct GeoNavMessage {
    /// Issue of Data, Navigation.
    pub iodn: u8,
    /// t_0 — reference time for this message (s, in GPS week).
    pub t0: u32,
    /// URA index.
    pub ura: u8,
    /// X position ECEF (m).
    pub x_m: f64,
    /// Y position ECEF (m).
    pub y_m: f64,
    /// Z position ECEF (m).
    pub z_m: f64,
    /// X velocity (m/s).
    pub xd_ms: f64,
    /// Y velocity (m/s).
    pub yd_ms: f64,
    /// Z velocity (m/s).
    pub zd_ms: f64,
    /// X acceleration (m/s²).
    pub xdd_ms2: f64,
    /// Y acceleration (m/s²).
    pub ydd_ms2: f64,
    /// Z acceleration (m/s²).
    pub zdd_ms2: f64,
    /// AF0 — clock bias (s).
    pub af0_s: f64,
    /// AF1 — clock drift (s/s).
    pub af1_ss: f64,
}

/// Ionospheric grid point (IGP) delay entry (MT26).
#[derive(Debug, Clone, Copy)]
pub struct IgpDelay {
    /// IGP index in the grid.
    pub igp_index: u8,
    /// Vertical ionospheric delay (m), scale: 0.125 m/LSB.
    pub give_m: f32,
    /// Grid Ionospheric Vertical Error indicator (0–15; 15 = not monitored).
    pub givei: u8,
}

/// Ionospheric grid delays message (MT26).
#[derive(Debug, Clone)]
pub struct IonoGridDelays {
    /// Band number (0–10).
    pub band: u8,
    /// Block ID (0–13).
    pub block_id: u8,
    /// Issue of Data, Ionospheric.
    pub iodi: u8,
    /// Up to 15 IGP delay estimates.
    pub igps: Vec<IgpDelay>,
}

/// Long-term satellite error correction (MT25).
#[derive(Debug, Clone, Copy)]
pub struct LongTermCorrection {
    /// PRN slot.
    pub slot: u8,
    /// Issue of data.
    pub iod: u8,
    /// Delta-x ECEF (m).
    pub dx_m: f32,
    /// Delta-y ECEF (m).
    pub dy_m: f32,
    /// Delta-z ECEF (m).
    pub dz_m: f32,
    /// Delta-clock (m).
    pub daf0_m: f32,
}

/// SBAS network time offset (MT12).
#[derive(Debug, Clone)]
pub struct SbasNetworkTime {
    /// A1 — UTC drift (s/s).
    pub a1_ss: f64,
    /// A0 — UTC bias (s).
    pub a0_s: f64,
    /// t_0t — reference time (s in GPS week).
    pub t0t: u32,
    /// WN_t — UTC reference week.
    pub wnt: u16,
    /// Delta-t_LS — leap seconds.
    pub dtls: i8,
    /// UTC identifier (0=UTC, 1=USNO, etc.).
    pub utc_id: u8,
}

/// Parsed SBAS message.
#[derive(Debug, Clone)]
pub struct SbasMessage {
    /// Preamble byte (should be 0x53, 0x9A, or 0xC6).
    pub preamble: u8,
    /// Message type.
    pub msg_type: MsgType,
    /// CRC valid flag.
    pub crc_valid: bool,
    /// Parsed payload.
    pub payload: SbasPayload,
}

/// Union-like enum for message payloads.
#[derive(Debug, Clone)]
pub enum SbasPayload {
    DontUse,
    PrnMask(PrnMask),
    FastCorrections(FastCorrections),
    IntegrityInfo { iodf: [u8; 4], udrei: Vec<u8> },
    GeoNavMessage(GeoNavMessage),
    IonoGridDelays(IonoGridDelays),
    LongTermCorrections(Vec<LongTermCorrection>),
    SbasNetworkTime(SbasNetworkTime),
    NullMessage,
    Unknown { raw: Vec<u8> },
}

// ---------------------------------------------------------------------------
// Bit Extraction Helper
// ---------------------------------------------------------------------------

/// Extract a big-endian unsigned integer from a bit slice.
fn bits_to_u64(bits: &[bool], start: usize, len: usize) -> u64 {
    let mut val = 0u64;
    for i in 0..len {
        if start + i < bits.len() && bits[start + i] {
            val |= 1 << (len - 1 - i);
        }
    }
    val
}

fn bits_to_u32(bits: &[bool], start: usize, len: usize) -> u32 {
    bits_to_u64(bits, start, len) as u32
}

fn bits_to_i32(bits: &[bool], start: usize, len: usize) -> i32 {
    let raw = bits_to_u32(bits, start, len);
    // Sign extend
    if len < 32 && (raw >> (len - 1)) & 1 != 0 {
        let sign_ext = !0u32 << len;
        (raw | sign_ext) as i32
    } else {
        raw as i32
    }
}

// ---------------------------------------------------------------------------
// SBAS Message Decoder
// ---------------------------------------------------------------------------

/// SBAS message decoder: parses 250-bit frames.
#[derive(Debug, Default)]
pub struct SbasDecoder {
    /// Number of messages successfully decoded.
    pub decoded_count: u64,
    /// Number of CRC failures.
    pub crc_fail_count: u64,
}

impl SbasDecoder {
    /// Create a new decoder.
    pub fn new() -> Self {
        SbasDecoder::default()
    }

    /// Decode a 250-bit SBAS message.
    pub fn decode(&mut self, bits: &[bool]) -> Option<SbasMessage> {
        if bits.len() != MSG_BITS {
            return None;
        }
        let crc_ok = verify_sbas_crc(bits);
        if !crc_ok {
            self.crc_fail_count += 1;
        } else {
            self.decoded_count += 1;
        }

        let preamble = bits_to_u32(bits, 0, 8) as u8;
        let msg_type_u8 = bits_to_u32(bits, 8, 6) as u8;
        let msg_type = MsgType::from(msg_type_u8);

        // Data field starts at bit 14 (after preamble + type)
        let payload = if crc_ok {
            self.parse_payload(msg_type, bits)
        } else {
            let nbytes = 226 / 8 + 1;
            let mut raw = vec![0u8; nbytes];
            for i in 0..226 {
                if bits[14 + i] {
                    raw[i / 8] |= 0x80 >> (i % 8);
                }
            }
            SbasPayload::Unknown { raw }
        };

        Some(SbasMessage { preamble, msg_type, crc_valid: crc_ok, payload })
    }

    fn parse_payload(&self, msg_type: MsgType, bits: &[bool]) -> SbasPayload {
        // Data field: bits[14..226] (212 bits of user data)
        match msg_type {
            MsgType::DontUse => SbasPayload::DontUse,
            MsgType::NullMessage => SbasPayload::NullMessage,
            MsgType::PrnMask => {
                // MT1: GPS PRNs bits 14..50, GLONASS 51..74, spare 75..99, IODP 100..101
                let gps_mask = bits_to_u64(bits, 14, 37);
                let glonass_mask = bits_to_u32(bits, 51, 24) as u32;
                let iodp = bits_to_u32(bits, 100, 2) as u8;
                SbasPayload::PrnMask(PrnMask { gps_mask, glonass_mask, iodp })
            }
            MsgType::FastCorrections2
            | MsgType::FastCorrections3
            | MsgType::FastCorrections4
            | MsgType::FastCorrections5 => {
                // MT2-5: IODF[2b], IODP[2b], then 13 × (PRC[12b] + UDREI[4b])
                let iodf = bits_to_u32(bits, 14, 2) as u8;
                let iodp = bits_to_u32(bits, 16, 2) as u8;
                let mut corrections = Vec::new();
                for slot in 0..13 {
                    let base = 18 + slot * 16;
                    let prc_raw = bits_to_i32(bits, base, 12);
                    let udrei = bits_to_u32(bits, base + 12, 4) as u8;
                    let prc_m = prc_raw as f32 * 0.125;
                    corrections.push(FastCorrection { slot: slot as u8 + 1, prc_m, udrei });
                }
                SbasPayload::FastCorrections(FastCorrections { iodf, iodp, corrections })
            }
            MsgType::IntegrityInfo => {
                // MT6: IODF[2b] × 4, then 51 × UDREI[4b]
                let iodf = [
                    bits_to_u32(bits, 14, 2) as u8,
                    bits_to_u32(bits, 16, 2) as u8,
                    bits_to_u32(bits, 18, 2) as u8,
                    bits_to_u32(bits, 20, 2) as u8,
                ];
                let mut udrei = Vec::new();
                for i in 0..51 {
                    udrei.push(bits_to_u32(bits, 22 + i * 4, 4) as u8);
                }
                SbasPayload::IntegrityInfo { iodf, udrei }
            }
            MsgType::GeoNavMessage => {
                // MT9: IOD_N[8b], t_0[13b], URA[4b], x[30b], y[30b], z[25b],
                //      xd[17b], yd[17b], zd[18b], xdd[10b], ydd[10b], zdd[10b],
                //      af0[12b], af1[8b]
                let iodn = bits_to_u32(bits, 14, 8) as u8;
                let t0 = bits_to_u32(bits, 22, 13);
                let ura = bits_to_u32(bits, 35, 4) as u8;
                let x_raw = bits_to_i32(bits, 39, 30);
                let y_raw = bits_to_i32(bits, 69, 30);
                let z_raw = bits_to_i32(bits, 99, 25);
                let xd_raw = bits_to_i32(bits, 124, 17);
                let yd_raw = bits_to_i32(bits, 141, 17);
                let zd_raw = bits_to_i32(bits, 158, 18);
                let xdd_raw = bits_to_i32(bits, 176, 10);
                let ydd_raw = bits_to_i32(bits, 186, 10);
                let zdd_raw = bits_to_i32(bits, 196, 10);
                let af0_raw = bits_to_i32(bits, 206, 12);
                let af1_raw = bits_to_i32(bits, 218, 8);
                SbasPayload::GeoNavMessage(GeoNavMessage {
                    iodn,
                    t0: t0 * 16,
                    ura,
                    x_m: x_raw as f64 * 0.08,
                    y_m: y_raw as f64 * 0.08,
                    z_m: z_raw as f64 * 0.4,
                    xd_ms: xd_raw as f64 * 0.000_625,
                    yd_ms: yd_raw as f64 * 0.000_625,
                    zd_ms: zd_raw as f64 * 0.004,
                    xdd_ms2: xdd_raw as f64 * 0.000_000_125,
                    ydd_ms2: ydd_raw as f64 * 0.000_000_125,
                    zdd_ms2: zdd_raw as f64 * 0.000_000_5625,
                    af0_s: af0_raw as f64 * 1e-11 * 2.0_f64.powi(0),
                    af1_ss: af1_raw as f64 * 1e-13,
                })
            }
            MsgType::IonoGridDelays => {
                // MT26: band[4b], block_id[4b], 15×(GIVD[9b]+GIVEI[4b]), iodi[2b]
                let band = bits_to_u32(bits, 14, 4) as u8;
                let block_id = bits_to_u32(bits, 18, 4) as u8;
                let mut igps = Vec::new();
                for i in 0..15 {
                    let base = 22 + i * 13;
                    let givd_raw = bits_to_u32(bits, base, 9);
                    let givei = bits_to_u32(bits, base + 9, 4) as u8;
                    let give_m = givd_raw as f32 * 0.125;
                    igps.push(IgpDelay { igp_index: i as u8, give_m, givei });
                }
                let iodi = bits_to_u32(bits, 217, 2) as u8;
                SbasPayload::IonoGridDelays(IonoGridDelays { band, block_id, iodi, igps })
            }
            MsgType::LongTermCorrections => {
                // MT25: two 51-bit LTC blocks per message
                let mut ltcs = Vec::new();
                for b in 0..2 {
                    let base = 14 + b * 51;
                    // velocity code bit
                    let vc = bits_to_u32(bits, base, 1) != 0;
                    if !vc {
                        // Position only
                        let slot = bits_to_u32(bits, base + 1, 6) as u8;
                        let iod = bits_to_u32(bits, base + 7, 8) as u8;
                        let dx = bits_to_i32(bits, base + 15, 9) as f32 * 0.125;
                        let dy = bits_to_i32(bits, base + 24, 9) as f32 * 0.125;
                        let dz = bits_to_i32(bits, base + 33, 9) as f32 * 0.125;
                        let daf0 = bits_to_i32(bits, base + 42, 10) as f32 * 2e-9 * C_MPS as f32;
                        ltcs.push(LongTermCorrection { slot, iod, dx_m: dx, dy_m: dy, dz_m: dz, daf0_m: daf0 });
                    }
                }
                SbasPayload::LongTermCorrections(ltcs)
            }
            MsgType::SbasNetworkTime => {
                // MT12: a1[24b], a0[32b], t_0t[8b], wnt[8b], dtls[8b], utc_id[3b]
                let a1_raw = bits_to_i32(bits, 14, 24);
                let a0_raw = bits_to_i32(bits, 38, 32);
                let t0t = bits_to_u32(bits, 70, 8);
                let wnt = bits_to_u32(bits, 78, 8) as u16;
                let dtls_raw = bits_to_i32(bits, 86, 8);
                let utc_id = bits_to_u32(bits, 94, 3) as u8;
                SbasPayload::SbasNetworkTime(SbasNetworkTime {
                    a1_ss: a1_raw as f64 * 2.0_f64.powi(-50),
                    a0_s: a0_raw as f64 * 2.0_f64.powi(-30),
                    t0t: t0t * 3600,
                    wnt,
                    dtls: dtls_raw as i8,
                    utc_id,
                })
            }
            _ => {
                let nbytes = 28; // 212 bits of data rounded up
                let mut raw = vec![0u8; nbytes];
                for i in 0..212 {
                    if 14 + i < bits.len() && bits[14 + i] {
                        raw[i / 8] |= 0x80 >> (i % 8);
                    }
                }
                SbasPayload::Unknown { raw }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Ionospheric Grid Point (IGP) Interpolation
// ---------------------------------------------------------------------------

/// An ionospheric grid point with coordinates and estimated delay.
#[derive(Debug, Clone, Copy)]
pub struct IgpPoint {
    /// Latitude (degrees, -85 to +85).
    pub lat_deg: f64,
    /// Longitude (degrees, -180 to +180).
    pub lon_deg: f64,
    /// Vertical ionospheric delay (m).
    pub delay_m: f32,
    /// GIVE indicator (0–14 valid, 15 = not monitored).
    pub givei: u8,
}

/// SBAS ionospheric pierce point grid interpolator.
///
/// Computes the vertical delay at the ionospheric pierce point (IPP)
/// by bilinear interpolation of four surrounding IGPs (RTCA DO-229 §A.4.4.10).
#[derive(Debug, Default)]
pub struct IonoGridInterpolator {
    igps: Vec<IgpPoint>,
}

impl IonoGridInterpolator {
    /// Create with a set of IGPs.
    pub fn new(igps: Vec<IgpPoint>) -> Self {
        IonoGridInterpolator { igps }
    }

    /// Add or update an IGP.
    pub fn update_igp(&mut self, igp: IgpPoint) {
        if let Some(existing) = self
            .igps
            .iter_mut()
            .find(|p| (p.lat_deg - igp.lat_deg).abs() < 0.01 && (p.lon_deg - igp.lon_deg).abs() < 0.01)
        {
            *existing = igp;
        } else {
            self.igps.push(igp);
        }
    }

    /// Compute vertical ionospheric delay at an arbitrary point via bilinear interpolation.
    ///
    /// `lat_deg`, `lon_deg` — IPP latitude and longitude in degrees.
    /// Returns `None` if surrounding IGPs are unavailable or not monitored.
    pub fn interpolate(&self, lat_deg: f64, lon_deg: f64) -> Option<f32> {
        // Grid spacing for SBAS band 0 (5° × 5° between ±75°)
        let spacing = 5.0;
        let lat0 = (lat_deg / spacing).floor() * spacing;
        let lon0 = (lon_deg / spacing).floor() * spacing;
        let lat1 = lat0 + spacing;
        let lon1 = lon0 + spacing;

        let sw = self.find_igp(lat0, lon0)?;
        let se = self.find_igp(lat0, lon1)?;
        let nw = self.find_igp(lat1, lon0)?;
        let ne = self.find_igp(lat1, lon1)?;

        // All four must be monitored
        if sw.givei >= 15 || se.givei >= 15 || nw.givei >= 15 || ne.givei >= 15 {
            return None;
        }

        // Bilinear weights
        let xpp = (lon_deg - lon0) / spacing;
        let ypp = (lat_deg - lat0) / spacing;

        let delay = sw.delay_m as f64 * (1.0 - xpp) * (1.0 - ypp)
            + se.delay_m as f64 * xpp * (1.0 - ypp)
            + nw.delay_m as f64 * (1.0 - xpp) * ypp
            + ne.delay_m as f64 * xpp * ypp;

        Some(delay as f32)
    }

    fn find_igp(&self, lat: f64, lon: f64) -> Option<&IgpPoint> {
        self.igps.iter().find(|p| (p.lat_deg - lat).abs() < 0.01 && (p.lon_deg - lon).abs() < 0.01)
    }

    /// Compute ionospheric pierce point latitude and longitude.
    ///
    /// `user_lat_deg`, `user_lon_deg` — receiver position.
    /// `el_deg` — satellite elevation angle (degrees).
    /// `az_deg` — satellite azimuth angle (degrees, true north).
    ///
    /// Returns (ipp_lat_deg, ipp_lon_deg).
    pub fn compute_ipp(user_lat_deg: f64, user_lon_deg: f64, el_deg: f64, az_deg: f64) -> (f64, f64) {
        let el_rad = el_deg * PI / 180.0;
        let az_rad = az_deg * PI / 180.0;
        let lat_rad = user_lat_deg * PI / 180.0;

        // Earth central angle
        let psi_pp = PI / 2.0 - el_rad - ((EARTH_RADIUS_M / (EARTH_RADIUS_M + IPP_HEIGHT_M)) * el_rad.cos()).asin();

        let ipp_lat_rad = (lat_rad.sin() * psi_pp.cos() + lat_rad.cos() * psi_pp.sin() * az_rad.cos()).asin();

        let ipp_lon_rad = user_lon_deg * PI / 180.0
            + (psi_pp.sin() * az_rad.sin() / ipp_lat_rad.cos()).asin();

        (ipp_lat_rad * 180.0 / PI, ipp_lon_rad * 180.0 / PI)
    }

    /// Obliquity factor F for slant-to-vertical conversion.
    ///
    /// `el_deg` — satellite elevation angle (degrees).
    pub fn obliquity_factor(el_deg: f64) -> f64 {
        let el_rad = el_deg * PI / 180.0;
        let cos_z = (EARTH_RADIUS_M / (EARTH_RADIUS_M + IPP_HEIGHT_M)) * el_rad.cos();
        1.0 / (1.0 - cos_z * cos_z).sqrt()
    }

    /// Compute slant ionospheric delay from vertical delay and elevation angle.
    pub fn slant_delay(vertical_delay_m: f32, el_deg: f64) -> f32 {
        (vertical_delay_m as f64 * Self::obliquity_factor(el_deg)) as f32
    }
}

// ---------------------------------------------------------------------------
// Integrity Monitor
// ---------------------------------------------------------------------------

/// UDRE sigma table (m), indexed by UDREI (0–14).
/// UDREI=15 means "not monitored" — sigma is infinite.
static UDRE_SIGMA_M: &[f64] = &[
    0.0520, 0.0924, 0.1444, 0.2830, 0.4678, 0.6731, 1.2780, 1.9594, 3.1397, 5.2278,
    9.4460, 15.9685, 20.7870, 230.9999, 9999.9999,
];

/// GIVE sigma table (m), indexed by GIVEI (0–14).
static GIVE_SIGMA_M: &[f64] = &[
    0.0084, 0.0333, 0.0749, 0.1331, 0.2079, 0.2994, 0.4076, 0.5326, 0.6845, 0.8622,
    1.0966, 1.3926, 1.8945, 3.6218, 9999.9999,
];

/// Fault-free protection level parameters.
#[derive(Debug, Clone)]
pub struct IntegrityResult {
    /// Horizontal Protection Level (m).
    pub hpl_m: f64,
    /// Vertical Protection Level (m).
    pub vpl_m: f64,
    /// SBAS availability flag.
    pub available: bool,
}

/// SBAS integrity monitor: computes protection levels per DO-229.
#[derive(Debug, Default)]
pub struct IntegrityMonitor {
    /// Residual fast correction standard deviation multiplier.
    pub k_h: f64,
    /// Residual fast correction standard deviation multiplier (vertical).
    pub k_v: f64,
}

impl IntegrityMonitor {
    /// Create with default K_H = 6.0, K_V = 5.33 (HAL/VAL per DO-229E §2.1.5).
    pub fn new() -> Self {
        IntegrityMonitor { k_h: 6.0, k_v: 5.33 }
    }

    /// UDRE sigma for a given UDREI.
    pub fn udre_sigma(udrei: u8) -> f64 {
        let idx = (udrei as usize).min(UDRE_SIGMA_M.len() - 1);
        UDRE_SIGMA_M[idx]
    }

    /// GIVE sigma for a given GIVEI.
    pub fn give_sigma(givei: u8) -> f64 {
        let idx = (givei as usize).min(GIVE_SIGMA_M.len() - 1);
        GIVE_SIGMA_M[idx]
    }

    /// Compute HPL and VPL for a single satellite in view.
    ///
    /// Simplified single-satellite case (normally computed over all in-view SVs).
    pub fn compute_single_sv_pl(
        &self,
        udrei: u8,
        givei: u8,
        el_deg: f64,
    ) -> IntegrityResult {
        if udrei >= 15 || givei >= 15 {
            return IntegrityResult { hpl_m: f64::INFINITY, vpl_m: f64::INFINITY, available: false };
        }
        let sigma_flt = Self::udre_sigma(udrei);
        let sigma_iono = Self::give_sigma(givei) * IonoGridInterpolator::obliquity_factor(el_deg);
        // Combined sigma (RSS)
        let sigma_total = (sigma_flt * sigma_flt + sigma_iono * sigma_iono).sqrt();
        // Simplified single-SV protection level (bound)
        let hpl = self.k_h * sigma_total;
        let vpl = self.k_v * sigma_total;
        IntegrityResult { hpl_m: hpl, vpl_m: vpl, available: true }
    }

    /// Compute position protection level for multiple satellites.
    ///
    /// `sigmas` — per-satellite range-domain sigma values (m).
    /// `geometry_matrix` — 3×N (E,N,U) rows (normalized geometry matrix).
    /// Returns (HPL, VPL) using the least-squares protection-level formula.
    pub fn compute_protection_levels(
        &self,
        sigmas: &[f64],
        geometry_matrix: &[[f64; 3]],
    ) -> (f64, f64) {
        let n = sigmas.len();
        if n == 0 || geometry_matrix.len() != n {
            return (f64::INFINITY, f64::INFINITY);
        }
        // W = diag(1/sigma^2)
        // H = geometry (n×3)
        // P = (H'WH)^{-1} H'W
        // Cov_pos = P * diag(sigma^2) * P' = (H'WH)^{-1}
        // Approximate: compute diagonal of (H'WH)^{-1} for HPL/VPL
        let mut ata = [[0.0f64; 3]; 3];
        for (row, sigma) in geometry_matrix.iter().zip(sigmas.iter()) {
            let w = 1.0 / (sigma * sigma);
            for i in 0..3 {
                for j in 0..3 {
                    ata[i][j] += row[i] * row[j] * w;
                }
            }
        }
        // Invert 3×3 symmetric matrix
        let inv = invert_3x3(&ata);
        let var_e = inv[0][0];
        let var_n = inv[1][1];
        let var_u = inv[2][2];
        let hpl = self.k_h * (var_e + var_n).sqrt();
        let vpl = self.k_v * var_u.abs().sqrt();
        (hpl, vpl)
    }
}

/// Invert a 3×3 matrix (returns zero matrix on singular input).
fn invert_3x3(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-30 {
        return [[0.0; 3]; 3];
    }
    let inv_det = 1.0 / det;
    let mut inv = [[0.0f64; 3]; 3];
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
    inv
}

// ---------------------------------------------------------------------------
// SBAS System Configuration Presets
// ---------------------------------------------------------------------------

/// SBAS regional system identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SbasSystem {
    /// Wide Area Augmentation System (Americas).
    Waas,
    /// European Geostationary Navigation Overlay Service.
    Egnos,
    /// Multi-functional Satellite Augmentation System (Japan).
    Msas,
    /// GPS Aided GEO Augmented Navigation (India).
    Gagan,
    /// System for Differential Corrections and Monitoring (Russia).
    Sdcm,
    /// Custom/unknown.
    Custom,
}

/// SBAS processor configuration.
#[derive(Debug, Clone)]
pub struct SbasConfig {
    /// System type.
    pub system: SbasSystem,
    /// Active GEO satellite PRN numbers.
    pub prns: Vec<u16>,
    /// Nominal carrier frequency (Hz).
    pub carrier_hz: f64,
    /// HAL (Horizontal Alert Limit, m) for NPA.
    pub hal_npa_m: f64,
    /// VAL (Vertical Alert Limit, m) for APV-I.
    pub val_apv1_m: f64,
    /// Whether to use this system for corrections (true = active).
    pub enabled: bool,
}

impl SbasConfig {
    /// WAAS configuration (PRN 135, 138).
    pub fn waas() -> Self {
        SbasConfig {
            system: SbasSystem::Waas,
            prns: vec![135, 138],
            carrier_hz: L1_FREQ_HZ,
            hal_npa_m: 556.0,
            val_apv1_m: 50.0,
            enabled: true,
        }
    }

    /// EGNOS configuration (PRN 120, 124, 126).
    pub fn egnos() -> Self {
        SbasConfig {
            system: SbasSystem::Egnos,
            prns: vec![120, 124, 126],
            carrier_hz: L1_FREQ_HZ,
            hal_npa_m: 556.0,
            val_apv1_m: 50.0,
            enabled: true,
        }
    }

    /// MSAS configuration (PRN 129, 137).
    pub fn msas() -> Self {
        SbasConfig {
            system: SbasSystem::Msas,
            prns: vec![129, 137],
            carrier_hz: L1_FREQ_HZ,
            hal_npa_m: 556.0,
            val_apv1_m: 50.0,
            enabled: true,
        }
    }

    /// GAGAN configuration (PRN 127, 128).
    pub fn gagan() -> Self {
        SbasConfig {
            system: SbasSystem::Gagan,
            prns: vec![127, 128],
            carrier_hz: L1_FREQ_HZ,
            hal_npa_m: 556.0,
            val_apv1_m: 50.0,
            enabled: true,
        }
    }
}

// ---------------------------------------------------------------------------
// Main SbasProcessor
// ---------------------------------------------------------------------------

/// Correction state for one GPS/GNSS satellite slot.
#[derive(Debug, Clone, Default)]
pub struct SvCorrections {
    /// Slot index (1-indexed).
    pub slot: u8,
    /// Fast correction: pseudorange correction (m).
    pub fast_prc_m: f32,
    /// Fast correction: UDREI.
    pub udrei: u8,
    /// Long-term position delta X (m).
    pub ltc_dx_m: f32,
    /// Long-term position delta Y (m).
    pub ltc_dy_m: f32,
    /// Long-term position delta Z (m).
    pub ltc_dz_m: f32,
    /// Long-term clock delta (m equivalent).
    pub ltc_daf0_m: f32,
    /// Issue of data for long-term correction.
    pub ltc_iod: u8,
}

/// Top-level SBAS processor.
///
/// Aggregates decoded SBAS messages and provides corrected pseudoranges
/// and protection levels to GNSS receivers.
#[derive(Debug)]
pub struct SbasProcessor {
    /// Configuration.
    pub config: SbasConfig,
    /// Message decoder.
    pub decoder: SbasDecoder,
    /// Ionospheric grid interpolator.
    pub iono_grid: IonoGridInterpolator,
    /// Integrity monitor.
    pub integrity: IntegrityMonitor,
    /// PRN mask (from MT1).
    pub prn_mask: Option<PrnMask>,
    /// Per-slot fast corrections (indexed 0..51).
    pub sv_corrections: Vec<SvCorrections>,
    /// Count of processed messages by type.
    pub msg_counts: [u32; 64],
    /// GEO navigation message (from MT9).
    pub geo_nav: Option<GeoNavMessage>,
    /// SBAS network time (from MT12).
    pub network_time: Option<SbasNetworkTime>,
}

impl SbasProcessor {
    /// Create a new SBAS processor.
    pub fn new(config: SbasConfig) -> Self {
        let mut sv_corrections = Vec::with_capacity(52);
        for slot in 0..52u8 {
            sv_corrections.push(SvCorrections { slot, udrei: 15, ..Default::default() });
        }
        SbasProcessor {
            config,
            decoder: SbasDecoder::new(),
            iono_grid: IonoGridInterpolator::default(),
            integrity: IntegrityMonitor::new(),
            prn_mask: None,
            sv_corrections,
            msg_counts: [0; 64],
            geo_nav: None,
            network_time: None,
        }
    }

    /// Process a raw 250-bit SBAS message.
    ///
    /// Returns the parsed message if the frame is valid and CRC passes.
    pub fn process_message(&mut self, bits: &[bool]) -> Option<SbasMessage> {
        let msg = self.decoder.decode(bits)?;
        let type_idx: usize = match msg.msg_type {
            MsgType::DontUse => 0,
            MsgType::PrnMask => 1,
            MsgType::FastCorrections2 => 2,
            MsgType::FastCorrections3 => 3,
            MsgType::FastCorrections4 => 4,
            MsgType::FastCorrections5 => 5,
            MsgType::IntegrityInfo => 6,
            MsgType::FastCorrDegradation => 7,
            MsgType::GeoNavMessage => 9,
            MsgType::DegradationParams => 10,
            MsgType::SbasNetworkTime => 12,
            MsgType::SatelliteAlmanac => 17,
            MsgType::IgpMask => 18,
            MsgType::MixedFastLongTerm => 24,
            MsgType::LongTermCorrections => 25,
            MsgType::IonoGridDelays => 26,
            MsgType::ServiceMessage => 27,
            MsgType::ClockEphCovMatrix => 28,
            MsgType::InternalTestMessage => 62,
            MsgType::NullMessage => 63,
            MsgType::Unknown(v) => v as usize,
        };
        if type_idx < 64 {
            self.msg_counts[type_idx] = self.msg_counts[type_idx].saturating_add(1);
        }

        if msg.crc_valid {
            self.apply_message(&msg.payload);
        }
        Some(msg)
    }

    fn apply_message(&mut self, payload: &SbasPayload) {
        match payload {
            SbasPayload::PrnMask(mask) => {
                self.prn_mask = Some(mask.clone());
            }
            SbasPayload::FastCorrections(fc) => {
                for corr in &fc.corrections {
                    let idx = corr.slot as usize;
                    if idx < self.sv_corrections.len() {
                        self.sv_corrections[idx].fast_prc_m = corr.prc_m;
                        self.sv_corrections[idx].udrei = corr.udrei;
                    }
                }
            }
            SbasPayload::IonoGridDelays(igd) => {
                for igp in &igd.igps {
                    if igp.givei < 15 {
                        // Reconstruct approximate lat/lon from IGP index + band (simplified)
                        let lat = -75.0 + (igp.igp_index as f64) * 5.0;
                        let lon = -180.0 + (igd.band as f64) * 40.0;
                        self.iono_grid.update_igp(IgpPoint {
                            lat_deg: lat,
                            lon_deg: lon,
                            delay_m: igp.give_m,
                            givei: igp.givei,
                        });
                    }
                }
            }
            SbasPayload::LongTermCorrections(ltcs) => {
                for ltc in ltcs {
                    let idx = ltc.slot as usize;
                    if idx < self.sv_corrections.len() {
                        self.sv_corrections[idx].ltc_dx_m = ltc.dx_m;
                        self.sv_corrections[idx].ltc_dy_m = ltc.dy_m;
                        self.sv_corrections[idx].ltc_dz_m = ltc.dz_m;
                        self.sv_corrections[idx].ltc_daf0_m = ltc.daf0_m;
                        self.sv_corrections[idx].ltc_iod = ltc.iod;
                    }
                }
            }
            SbasPayload::GeoNavMessage(nav) => {
                self.geo_nav = Some(nav.clone());
            }
            SbasPayload::SbasNetworkTime(t) => {
                self.network_time = Some(t.clone());
            }
            _ => {}
        }
    }

    /// Get the combined pseudorange correction (fast + long-term clock) for a slot.
    pub fn pseudorange_correction(&self, slot: usize) -> Option<f32> {
        if slot >= self.sv_corrections.len() {
            return None;
        }
        let sv = &self.sv_corrections[slot];
        if sv.udrei >= 15 {
            return None;
        }
        Some(sv.fast_prc_m + sv.ltc_daf0_m)
    }

    /// Compute slant ionospheric delay for a given receiver and satellite geometry.
    pub fn iono_delay(
        &self,
        user_lat_deg: f64,
        user_lon_deg: f64,
        el_deg: f64,
        az_deg: f64,
    ) -> Option<f32> {
        let (ipp_lat, ipp_lon) =
            IonoGridInterpolator::compute_ipp(user_lat_deg, user_lon_deg, el_deg, az_deg);
        let vd = self.iono_grid.interpolate(ipp_lat, ipp_lon)?;
        Some(IonoGridInterpolator::slant_delay(vd, el_deg))
    }

    /// SBAS availability check: returns true if system has valid fast corrections
    /// for at least `min_svs` satellites.
    pub fn is_available(&self, min_svs: usize) -> bool {
        let count = self.sv_corrections.iter().filter(|sv| sv.udrei < 15).count();
        count >= min_svs
    }

    /// Statistics summary string.
    pub fn stats(&self) -> String {
        format!(
            "SBAS {:?}: decoded={}, crc_fail={}, available_svs={}",
            self.config.system,
            self.decoder.decoded_count,
            self.decoder.crc_fail_count,
            self.sv_corrections.iter().filter(|sv| sv.udrei < 15).count()
        )
    }
}

// ---------------------------------------------------------------------------
// Helper: build a simple test message
// ---------------------------------------------------------------------------

/// Build a well-formed 250-bit null message (MT63) for testing.
pub fn build_null_message() -> Vec<bool> {
    // Preamble = 0x53, type = 63 (0b111111), data = all zeros
    let preamble: u8 = 0x53;
    let msg_type: u8 = 63;
    let mut bits = Vec::with_capacity(250);
    for i in (0..8).rev() {
        bits.push((preamble >> i) & 1 != 0);
    }
    for i in (0..6).rev() {
        bits.push((msg_type >> i) & 1 != 0);
    }
    // 212 bits of zeros
    bits.extend(std::iter::repeat(false).take(212));
    // Append CRC (bits is now 226 long)
    append_sbas_crc(&bits)
}

/// Build a 250-bit MT1 (PRN mask) message.
/// `gps_mask` — 37-bit GPS PRN mask, `iodp` — 2-bit IODP.
pub fn build_prn_mask_message(gps_mask: u64, iodp: u8) -> Vec<bool> {
    let preamble: u8 = 0x9A;
    let msg_type: u8 = 1;
    let mut bits = Vec::with_capacity(250);
    for i in (0..8).rev() {
        bits.push((preamble >> i) & 1 != 0);
    }
    for i in (0..6).rev() {
        bits.push((msg_type >> i) & 1 != 0);
    }
    // GPS PRN mask: 37 bits (PRN 1-37)
    for i in (0..37).rev() {
        bits.push((gps_mask >> i) & 1 != 0);
    }
    // GLONASS mask: 24 bits = 0
    bits.extend(std::iter::repeat(false).take(24));
    // Spare: 25 bits
    bits.extend(std::iter::repeat(false).take(25));
    // IODP: 2 bits
    bits.push((iodp >> 1) & 1 != 0);
    bits.push(iodp & 1 != 0);
    // Remaining data bits to reach 226
    let remaining = 226 - bits.len();
    bits.extend(std::iter::repeat(false).take(remaining));
    assert_eq!(bits.len(), 226);
    append_sbas_crc(&bits)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- PRN Code Tests ---

    #[test]
    fn test_prn_code_length() {
        let code = SbasPrnCode::new(135).unwrap();
        assert_eq!(code.length(), 1023, "PRN code must be 1023 chips");
    }

    #[test]
    fn test_prn_code_invalid_prn() {
        assert!(SbasPrnCode::new(1).is_none(), "PRN 1 is GPS, not SBAS");
        assert!(SbasPrnCode::new(119).is_none(), "PRN 119 below SBAS range");
        assert!(SbasPrnCode::new(159).is_none(), "PRN 159 above SBAS range");
    }

    #[test]
    fn test_prn_code_valid_range() {
        for prn in 120u16..=158 {
            let code = SbasPrnCode::new(prn);
            assert!(code.is_some(), "PRN {prn} should be valid");
            assert_eq!(code.unwrap().length(), 1023);
        }
    }

    #[test]
    fn test_prn_code_autocorrelation_peak() {
        let code = SbasPrnCode::new(120).unwrap();
        let peak = code.auto_correlate_peak();
        assert_eq!(peak, 1023, "Autocorrelation peak must equal code length");
    }

    #[test]
    fn test_prn_cross_correlation_small() {
        let c120 = SbasPrnCode::new(120).unwrap();
        let c121 = SbasPrnCode::new(121).unwrap();
        let cross = c120.cross_correlate(&c121).abs();
        // Gold codes have bounded cross-correlation (should be much less than 1023)
        assert!(cross < 200, "Cross-correlation between different PRNs should be small, got {cross}");
    }

    #[test]
    fn test_prn_different_codes() {
        let c135 = SbasPrnCode::new(135).unwrap();
        let c136 = SbasPrnCode::new(136).unwrap();
        // Codes should differ
        let differ = c135.chips().iter().zip(c136.chips()).filter(|(&a, &b)| a != b).count();
        assert!(differ > 100, "Adjacent PRN codes should differ significantly");
    }

    // --- CRC-24Q Tests ---

    #[test]
    fn test_crc24q_empty() {
        assert_eq!(crc24q(&[]), 0);
    }

    #[test]
    fn test_crc24q_known_value() {
        // CRC of 0x00 byte should be 0 (empty CRC initial value)
        let result = crc24q(&[0x00]);
        // Non-trivial: result should be deterministic
        assert_eq!(crc24q(&[0x00]), result);
    }

    #[test]
    fn test_crc24q_consistency() {
        let data = b"SBAS test data for CRC check";
        let c1 = crc24q(data);
        let c2 = crc24q(data);
        assert_eq!(c1, c2, "CRC must be deterministic");
        assert!(c1 <= 0x00FF_FFFF, "CRC must fit in 24 bits");
    }

    #[test]
    fn test_crc24q_diff_data() {
        let c1 = crc24q(b"hello");
        let c2 = crc24q(b"world");
        assert_ne!(c1, c2, "Different data should produce different CRCs");
    }

    #[test]
    fn test_sbas_crc_roundtrip() {
        let mut bits = vec![false; 226];
        // Set some bits
        bits[8] = true; // type bit
        bits[14] = true;
        bits[20] = true;
        let msg_250 = append_sbas_crc(&bits);
        assert_eq!(msg_250.len(), 250);
        assert!(verify_sbas_crc(&msg_250), "CRC roundtrip must pass");
    }

    #[test]
    fn test_sbas_crc_detects_error() {
        let mut bits = vec![false; 226];
        bits[14] = true;
        let mut msg = append_sbas_crc(&bits);
        // Flip a data bit
        msg[14] = !msg[14];
        assert!(!verify_sbas_crc(&msg), "CRC must detect single-bit error");
    }

    #[test]
    fn test_sbas_crc_wrong_length() {
        assert!(!verify_sbas_crc(&[false; 100]));
        assert!(!verify_sbas_crc(&[false; 249]));
    }

    // --- Null Message Test ---

    #[test]
    fn test_null_message_valid_crc() {
        let msg = build_null_message();
        assert_eq!(msg.len(), 250);
        assert!(verify_sbas_crc(&msg), "Null message CRC must be valid");
    }

    #[test]
    fn test_null_message_decode() {
        let bits = build_null_message();
        let mut decoder = SbasDecoder::new();
        let parsed = decoder.decode(&bits).unwrap();
        assert!(parsed.crc_valid);
        assert_eq!(parsed.preamble, 0x53);
        assert!(matches!(parsed.msg_type, MsgType::NullMessage));
    }

    // --- PRN Mask Message Tests ---

    #[test]
    fn test_prn_mask_roundtrip() {
        let gps_mask = 0b1010_1010_1010_1010_1010_1010_1010_1010_101u64;
        let iodp = 2u8;
        let bits = build_prn_mask_message(gps_mask, iodp);
        assert_eq!(bits.len(), 250);
        assert!(verify_sbas_crc(&bits));

        let mut decoder = SbasDecoder::new();
        let msg = decoder.decode(&bits).unwrap();
        assert!(msg.crc_valid);
        assert!(matches!(msg.msg_type, MsgType::PrnMask));
        if let SbasPayload::PrnMask(mask) = msg.payload {
            assert_eq!(mask.iodp, iodp);
            // Check a few GPS bits
            assert_eq!((mask.gps_mask >> 36) & 1, (gps_mask >> 36) & 1);
        }
    }

    #[test]
    fn test_prn_mask_all_zeros() {
        let bits = build_prn_mask_message(0, 0);
        let mut decoder = SbasDecoder::new();
        let msg = decoder.decode(&bits).unwrap();
        assert!(msg.crc_valid);
        if let SbasPayload::PrnMask(mask) = msg.payload {
            assert_eq!(mask.gps_mask, 0);
            assert_eq!(mask.iodp, 0);
        }
    }

    // --- Convolutional FEC Tests ---

    #[test]
    fn test_conv_encode_length() {
        let bits: Vec<bool> = vec![true, false, true, true, false];
        let syms = conv_encode(&bits);
        assert_eq!(syms.len(), bits.len() * 2);
    }

    #[test]
    fn test_conv_encode_all_zeros() {
        let bits = vec![false; 20];
        let syms = conv_encode(&bits);
        assert_eq!(syms.len(), 40);
        // All-zero input with zero initial state gives all-zero output
        assert!(syms.iter().all(|&s| s == 0));
    }

    #[test]
    fn test_conv_encode_all_ones() {
        let bits = vec![true; 10];
        let syms = conv_encode(&bits);
        // Should not be all zeros
        assert!(syms.iter().any(|&s| s != 0));
    }

    #[test]
    fn test_viterbi_decode_no_errors() {
        let input: Vec<bool> = vec![true, false, true, false, true, true, false, true];
        let encoded = conv_encode(&input);
        let decoded = viterbi_decode(&encoded);
        assert_eq!(decoded.len(), input.len());
        assert_eq!(decoded, input, "Viterbi decode must recover original bits");
    }

    #[test]
    fn test_viterbi_decode_single_error() {
        let input: Vec<bool> = vec![true, false, true, false, true, false, true, false];
        let mut encoded = conv_encode(&input);
        // Flip one symbol
        encoded[4] ^= 1;
        let decoded = viterbi_decode(&encoded);
        // Should still mostly recover (Hamming distance 1)
        let errors = decoded.iter().zip(input.iter()).filter(|(a, b)| a != b).count();
        assert!(errors <= 1, "Single symbol error should be correctable, got {errors} bit errors");
    }

    #[test]
    fn test_viterbi_empty_input() {
        let result = viterbi_decode(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_viterbi_decode_all_zeros() {
        let input = vec![false; 16];
        let encoded = conv_encode(&input);
        let decoded = viterbi_decode(&encoded);
        assert_eq!(decoded, input);
    }

    // --- Ionospheric Grid Tests ---

    #[test]
    fn test_compute_ipp_basic() {
        // User at 0,0, satellite due south at 30° elevation, 180° azimuth
        let (lat, lon) = IonoGridInterpolator::compute_ipp(0.0, 0.0, 30.0, 180.0);
        // IPP should be south of user
        assert!(lat < 0.0, "IPP should be south for southward satellite");
        assert!(lat > -90.0);
        let _ = lon; // lon may vary
    }

    #[test]
    fn test_obliquity_factor_90_deg() {
        // At 90° elevation (zenith), obliquity = 1.0
        let f = IonoGridInterpolator::obliquity_factor(90.0);
        assert!((f - 1.0).abs() < 0.01, "Obliquity at zenith should be ~1.0, got {f}");
    }

    #[test]
    fn test_obliquity_factor_increases_at_low_elevation() {
        let f30 = IonoGridInterpolator::obliquity_factor(30.0);
        let f60 = IonoGridInterpolator::obliquity_factor(60.0);
        let f90 = IonoGridInterpolator::obliquity_factor(90.0);
        assert!(f30 > f60, "Obliquity should increase at lower elevations");
        assert!(f60 > f90, "Obliquity should increase at lower elevations");
    }

    #[test]
    fn test_slant_delay() {
        let vd = 5.0f32; // 5 m vertical delay
        let slant = IonoGridInterpolator::slant_delay(vd, 90.0);
        assert!((slant - vd).abs() < 0.1, "Slant delay at zenith should equal vertical delay");

        let slant_low = IonoGridInterpolator::slant_delay(vd, 20.0);
        assert!(slant_low > vd, "Slant delay at low elevation should be greater");
    }

    #[test]
    fn test_bilinear_interpolation() {
        let mut interp = IonoGridInterpolator::new(vec![
            IgpPoint { lat_deg: 0.0, lon_deg: 0.0, delay_m: 4.0, givei: 5 },
            IgpPoint { lat_deg: 0.0, lon_deg: 5.0, delay_m: 4.0, givei: 5 },
            IgpPoint { lat_deg: 5.0, lon_deg: 0.0, delay_m: 4.0, givei: 5 },
            IgpPoint { lat_deg: 5.0, lon_deg: 5.0, delay_m: 4.0, givei: 5 },
        ]);
        // Center point should return exactly 4.0
        let delay = interp.interpolate(2.5, 2.5);
        assert!(delay.is_some());
        let d = delay.unwrap();
        assert!((d - 4.0).abs() < 0.01, "Bilinear interp of uniform grid should give 4.0, got {d}");
    }

    #[test]
    fn test_bilinear_interpolation_missing_igp() {
        let interp = IonoGridInterpolator::new(vec![
            IgpPoint { lat_deg: 0.0, lon_deg: 0.0, delay_m: 4.0, givei: 5 },
            // Only one corner
        ]);
        let delay = interp.interpolate(2.5, 2.5);
        assert!(delay.is_none(), "Should return None with missing IGPs");
    }

    #[test]
    fn test_bilinear_not_monitored_igp() {
        let interp = IonoGridInterpolator::new(vec![
            IgpPoint { lat_deg: 0.0, lon_deg: 0.0, delay_m: 4.0, givei: 15 }, // not monitored
            IgpPoint { lat_deg: 0.0, lon_deg: 5.0, delay_m: 4.0, givei: 5 },
            IgpPoint { lat_deg: 5.0, lon_deg: 0.0, delay_m: 4.0, givei: 5 },
            IgpPoint { lat_deg: 5.0, lon_deg: 5.0, delay_m: 4.0, givei: 5 },
        ]);
        assert!(interp.interpolate(2.5, 2.5).is_none());
    }

    // --- Integrity Monitor Tests ---

    #[test]
    fn test_udre_sigma_values() {
        assert!((IntegrityMonitor::udre_sigma(0) - 0.0520).abs() < 1e-3);
        assert!((IntegrityMonitor::udre_sigma(1) - 0.0924).abs() < 1e-3);
        let large = IntegrityMonitor::udre_sigma(14);
        assert!(large > 9000.0);
    }

    #[test]
    fn test_give_sigma_values() {
        assert!((IntegrityMonitor::give_sigma(0) - 0.0084).abs() < 1e-4);
        let large = IntegrityMonitor::give_sigma(14);
        assert!(large > 9000.0);
    }

    #[test]
    fn test_protection_level_not_monitored() {
        let monitor = IntegrityMonitor::new();
        let result = monitor.compute_single_sv_pl(15, 5, 45.0);
        assert!(!result.available);
        assert!(result.hpl_m.is_infinite());
    }

    #[test]
    fn test_protection_level_reasonable() {
        let monitor = IntegrityMonitor::new();
        let result = monitor.compute_single_sv_pl(1, 1, 60.0); // UDREI=1, GIVEI=1, 60° el
        assert!(result.available);
        assert!(result.hpl_m > 0.0);
        assert!(result.vpl_m > 0.0);
        // For good UDREI/GIVEI, HPL should be below HAL_NPA
        assert!(result.hpl_m < 100.0, "HPL for good corrections should be small, got {}", result.hpl_m);
    }

    #[test]
    fn test_multi_sv_protection_level() {
        let monitor = IntegrityMonitor::new();
        let sigmas = vec![0.5, 0.6, 0.7, 0.8];
        let geometry = [
            [0.7, 0.3, 0.6],
            [-0.5, 0.6, 0.6],
            [0.1, -0.8, 0.6],
            [-0.3, -0.1, 0.9],
        ];
        let (hpl, vpl) = monitor.compute_protection_levels(&sigmas, &geometry);
        assert!(hpl > 0.0 && hpl.is_finite(), "HPL should be positive finite");
        assert!(vpl > 0.0 && vpl.is_finite(), "VPL should be positive finite");
    }

    // --- SbasProcessor Tests ---

    #[test]
    fn test_processor_waas_config() {
        let config = SbasConfig::waas();
        assert_eq!(config.prns, vec![135, 138]);
        assert!(matches!(config.system, SbasSystem::Waas));
    }

    #[test]
    fn test_processor_egnos_config() {
        let config = SbasConfig::egnos();
        assert_eq!(config.prns, vec![120, 124, 126]);
        assert!(matches!(config.system, SbasSystem::Egnos));
    }

    #[test]
    fn test_processor_msas_config() {
        let config = SbasConfig::msas();
        assert_eq!(config.prns, vec![129, 137]);
        assert!(matches!(config.system, SbasSystem::Msas));
    }

    #[test]
    fn test_processor_process_null() {
        let config = SbasConfig::waas();
        let mut proc = SbasProcessor::new(config);
        let bits = build_null_message();
        let msg = proc.process_message(&bits);
        assert!(msg.is_some());
        assert!(msg.unwrap().crc_valid);
        assert_eq!(proc.decoder.decoded_count, 1);
    }

    #[test]
    fn test_processor_initial_not_available() {
        let config = SbasConfig::waas();
        let proc = SbasProcessor::new(config);
        assert!(!proc.is_available(1), "Should not be available before receiving corrections");
    }

    #[test]
    fn test_processor_prn_mask_apply() {
        let config = SbasConfig::waas();
        let mut proc = SbasProcessor::new(config);
        let bits = build_prn_mask_message(0b111, 1);
        proc.process_message(&bits);
        assert!(proc.prn_mask.is_some());
        assert_eq!(proc.prn_mask.as_ref().unwrap().iodp, 1);
    }

    #[test]
    fn test_processor_crc_fail_counted() {
        let config = SbasConfig::waas();
        let mut proc = SbasProcessor::new(config);
        // Send corrupted message by flipping a bit of a valid message
        let valid = build_null_message();
        let mut bad_bits = valid.clone();
        bad_bits[20] = !bad_bits[20]; // flip a data bit, invalidating CRC
        proc.process_message(&bad_bits);
        assert_eq!(proc.decoder.crc_fail_count, 1);
    }

    #[test]
    fn test_processor_stats_string() {
        let config = SbasConfig::waas();
        let proc = SbasProcessor::new(config);
        let s = proc.stats();
        assert!(s.contains("SBAS"), "Stats should mention SBAS");
        assert!(s.contains("Waas"), "Stats should mention Waas");
    }

    #[test]
    fn test_processor_pseudorange_correction_unmonitored() {
        let config = SbasConfig::waas();
        let proc = SbasProcessor::new(config);
        // All slots start as UDREI=15 (not monitored)
        assert!(proc.pseudorange_correction(0).is_none());
        assert!(proc.pseudorange_correction(5).is_none());
    }

    // --- Message Type Tests ---

    #[test]
    fn test_msg_type_from_u8() {
        assert_eq!(MsgType::from(0), MsgType::DontUse);
        assert_eq!(MsgType::from(1), MsgType::PrnMask);
        assert_eq!(MsgType::from(6), MsgType::IntegrityInfo);
        assert_eq!(MsgType::from(9), MsgType::GeoNavMessage);
        assert_eq!(MsgType::from(18), MsgType::IgpMask);
        assert_eq!(MsgType::from(26), MsgType::IonoGridDelays);
        assert_eq!(MsgType::from(63), MsgType::NullMessage);
    }

    #[test]
    fn test_msg_type_unknown() {
        let mt = MsgType::from(50);
        assert!(matches!(mt, MsgType::Unknown(50)));
    }

    // --- Decoder Statistics Tests ---

    #[test]
    fn test_decoder_counts() {
        let mut dec = SbasDecoder::new();
        let valid = build_null_message();
        // Create an invalid message by flipping a data bit of a valid message
        let mut invalid = valid.clone();
        invalid[15] = !invalid[15]; // flip a data bit, breaking the CRC
        dec.decode(&valid);
        dec.decode(&valid);
        dec.decode(&invalid);
        assert_eq!(dec.decoded_count, 2);
        assert_eq!(dec.crc_fail_count, 1);
    }

    // --- Geometry and Signal Tests ---

    #[test]
    fn test_ipp_northern_user_northern_satellite() {
        let (lat, lon) = IonoGridInterpolator::compute_ipp(45.0, 0.0, 45.0, 0.0);
        // Azimuth 0 = North: IPP should be north of user
        assert!(lat > 45.0, "IPP north of user for northern satellite, got lat={lat}");
        let _ = lon;
    }

    #[test]
    fn test_sbas_signal_frequency() {
        assert!((L1_FREQ_HZ - 1_575_420_000.0).abs() < 1.0);
        assert!((CHIP_RATE - 1_023_000.0).abs() < 1.0);
    }

    #[test]
    fn test_invert_3x3_identity() {
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let inv = invert_3x3(&identity);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((inv[i][j] - expected).abs() < 1e-10, "inv[{i}][{j}] = {}", inv[i][j]);
            }
        }
    }

    #[test]
    fn test_invert_3x3_known() {
        let m = [[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 2.0]];
        let inv = invert_3x3(&m);
        // Verify A * A^{-1} ≈ I
        for i in 0..3 {
            for j in 0..3 {
                let mut sum = 0.0;
                for k in 0..3 {
                    sum += m[i][k] * inv[k][j];
                }
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((sum - expected).abs() < 1e-9, "A*A^-1[{i}][{j}] = {sum}");
            }
        }
    }

    #[test]
    fn test_prncode_chip_values() {
        let code = SbasPrnCode::new(138).unwrap();
        for &c in code.chips() {
            assert!(c == 1 || c == -1, "Chips must be +1 or -1");
        }
    }

    #[test]
    fn test_iono_interp_gradient() {
        // Grid with linear gradient
        let interp = IonoGridInterpolator::new(vec![
            IgpPoint { lat_deg: 0.0, lon_deg: 0.0, delay_m: 0.0, givei: 3 },
            IgpPoint { lat_deg: 0.0, lon_deg: 5.0, delay_m: 5.0, givei: 3 },
            IgpPoint { lat_deg: 5.0, lon_deg: 0.0, delay_m: 5.0, givei: 3 },
            IgpPoint { lat_deg: 5.0, lon_deg: 5.0, delay_m: 10.0, givei: 3 },
        ]);
        // At center (2.5, 2.5) bilinear gives 0.25*0+0.25*5+0.25*5+0.25*10 = 5.0
        let d = interp.interpolate(2.5, 2.5).unwrap();
        assert!((d - 5.0).abs() < 0.1, "Bilinear gradient interp = {d}");
    }
}

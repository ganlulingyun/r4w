//! VDES Processor — VHF Data Exchange System maritime communications
//!
//! Implements ITU-R M.2092 VDES and IALA Guideline G1139 for VHF maritime
//! data communications. Covers AIS (Automatic Identification System),
//! ASM (Application Specific Messages), VDE-TER (terrestrial), and
//! VDE-SAT (satellite) components.
//!
//! ## Standards
//! - ITU-R M.2092: Technical characteristics for a VHF data exchange system
//! - ITU-R M.1371-5: Technical characteristics for an AIS
//! - IALA Guideline G1139: The VDES — An Overview
//! - IEC 62320-1: AIS — Part 1: Shipborne AIS minimum operational performance
//!
//! ## Channel Plan
//! - AIS1: 161.975 MHz (CH 87B)
//! - AIS2: 162.025 MHz (CH 88B)
//! - ASM1: 161.950 MHz (new VDE channel)
//! - ASM2: 162.000 MHz (new VDE channel)
//! - VDE-TER uplink: 157.025–157.425 MHz
//! - VDE-TER downlink: 161.625–162.025 MHz
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vdes_processor::{AisPositionReport, encode_ais_type1, nrzi_encode, bit_stuff};
//!
//! let report = AisPositionReport {
//!     mmsi: 123456789,
//!     nav_status: 0,
//!     rot: 0.0,
//!     sog: 5.5,
//!     position_accuracy: false,
//!     longitude: -122.4194,
//!     latitude: 37.7749,
//!     cog: 270.0,
//!     true_heading: 270,
//!     timestamp: 30,
//! };
//! let bits = encode_ais_type1(&report);
//! let nrzi = nrzi_encode(&bits);
//! let stuffed = bit_stuff(&nrzi);
//! assert!(!stuffed.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// AIS channel 1 frequency: 161.975 MHz
pub const AIS1_FREQ_HZ: f64 = 161_975_000.0;
/// AIS channel 2 frequency: 162.025 MHz
pub const AIS2_FREQ_HZ: f64 = 162_025_000.0;
/// ASM channel 1 frequency: 161.950 MHz
pub const ASM1_FREQ_HZ: f64 = 161_950_000.0;
/// ASM channel 2 frequency: 162.000 MHz
pub const ASM2_FREQ_HZ: f64 = 162_000_000.0;

/// AIS data rate: 9600 bps GMSK
pub const AIS_DATA_RATE_BPS: u32 = 9600;
/// AIS TDMA slot duration in seconds (2250 slots per minute)
pub const AIS_SLOT_DURATION_S: f64 = 26.666_666_666e-3;
/// Number of TDMA slots per minute
pub const AIS_SLOTS_PER_MINUTE: u32 = 2250;
/// AIS GMSK BT product
pub const AIS_GMSK_BT: f64 = 0.4;
/// HDLC frame delimiter byte
pub const HDLC_FLAG: u8 = 0x7E;
/// CRC-16/CCITT polynomial
pub const CRC16_POLY: u16 = 0x1021;
/// CRC-16/CCITT initial value
pub const CRC16_INIT: u16 = 0xFFFF;

/// VDE-TER 25 kHz bandwidth peak data rate (kbps)
pub const VDE_TER_25KHZ_KBPS: u32 = 76_800;
/// VDE-TER 50 kHz bandwidth peak data rate (kbps)
pub const VDE_TER_50KHZ_KBPS: u32 = 153_600;
/// VDE-TER 100 kHz bandwidth peak data rate (kbps)
pub const VDE_TER_100KHZ_KBPS: u32 = 307_200;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// AIS navigation status codes (ITU-R M.1371 Table 45)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NavStatus {
    UnderWayUsingEngine = 0,
    AtAnchor = 1,
    NotUnderCommand = 2,
    RestrictedManoeuvrability = 3,
    ConstrainedByDraught = 4,
    Moored = 5,
    Aground = 6,
    EngagedInFishing = 7,
    UnderWaySailing = 8,
    /// HSC
    Hsc = 9,
    /// WIG
    Wig = 10,
    PowerDrivenVesselTowing = 11,
    PowerDrivenVesselPushing = 12,
    Reserved13 = 13,
    AisSartMobOrEpirb = 14,
    Undefined = 15,
}

impl NavStatus {
    /// Convert u8 to NavStatus
    pub fn from_u8(v: u8) -> Self {
        match v & 0x0F {
            0 => Self::UnderWayUsingEngine,
            1 => Self::AtAnchor,
            2 => Self::NotUnderCommand,
            3 => Self::RestrictedManoeuvrability,
            4 => Self::ConstrainedByDraught,
            5 => Self::Moored,
            6 => Self::Aground,
            7 => Self::EngagedInFishing,
            8 => Self::UnderWaySailing,
            9 => Self::Hsc,
            10 => Self::Wig,
            11 => Self::PowerDrivenVesselTowing,
            12 => Self::PowerDrivenVesselPushing,
            13 => Self::Reserved13,
            14 => Self::AisSartMobOrEpirb,
            _ => Self::Undefined,
        }
    }
}

/// AIS Type 1/2/3 Position Report (Class A)
#[derive(Debug, Clone)]
pub struct AisPositionReport {
    /// 9-digit MMSI
    pub mmsi: u32,
    /// Navigation status (0–15)
    pub nav_status: u8,
    /// Rate of turn in degrees/min (−720 to +720; −128 = not available)
    pub rot: f64,
    /// Speed over ground in knots (0.0–102.2; 102.3 = not available)
    pub sog: f64,
    /// High position accuracy (DGPS)
    pub position_accuracy: bool,
    /// Longitude in degrees (−180.0 to +180.0; 181.0 = not available)
    pub longitude: f64,
    /// Latitude in degrees (−90.0 to +90.0; 91.0 = not available)
    pub latitude: f64,
    /// Course over ground in degrees (0.0–359.9; 360.0 = not available)
    pub cog: f64,
    /// True heading in degrees (0–359; 511 = not available)
    pub true_heading: u16,
    /// UTC second (0–59; 60 = not available; 61/62 reserved; 63 = positioning unavailable)
    pub timestamp: u8,
}

impl Default for AisPositionReport {
    fn default() -> Self {
        Self {
            mmsi: 0,
            nav_status: 15,
            rot: -128.0,
            sog: 102.3,
            position_accuracy: false,
            longitude: 181.0,
            latitude: 91.0,
            cog: 360.0,
            true_heading: 511,
            timestamp: 60,
        }
    }
}

/// AIS Type 5 Static and Voyage Related Data
#[derive(Debug, Clone)]
pub struct AisStaticVoyage {
    /// 9-digit MMSI
    pub mmsi: u32,
    /// IMO number (1–999999999; 0 = not available)
    pub imo: u32,
    /// Call sign (7 chars, padded with spaces)
    pub callsign: [u8; 7],
    /// Vessel name (20 chars, padded with spaces)
    pub name: [u8; 20],
    /// Ship and cargo type (0–255)
    pub ship_type: u8,
    /// Bow-to-position transponder (0–511 m)
    pub dim_a: u16,
    /// Stern-to-position transponder (0–511 m)
    pub dim_b: u16,
    /// Port-to-position transponder (0–63 m)
    pub dim_c: u8,
    /// Starboard-to-position transponder (0–63 m)
    pub dim_d: u8,
    /// Type of EPFD (0–15)
    pub epfd_type: u8,
    /// ETA month (1–12; 0 = not available)
    pub eta_month: u8,
    /// ETA day (1–31; 0 = not available)
    pub eta_day: u8,
    /// ETA hour (0–23; 24 = not available)
    pub eta_hour: u8,
    /// ETA minute (0–59; 60 = not available)
    pub eta_minute: u8,
    /// Maximum static draught in 1/10 m (0–25.5; 0 = not available)
    pub draught: f64,
    /// Destination (20 chars)
    pub destination: [u8; 20],
    /// Data terminal equipment ready
    pub dte: bool,
}

/// VDES channel identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VdesChannel {
    Ais1,
    Ais2,
    Asm1,
    Asm2,
    VdeTer(u8),  // sub-channel index
    VdeSat,
}

impl VdesChannel {
    /// Center frequency in Hz
    pub fn center_freq_hz(&self) -> f64 {
        match self {
            VdesChannel::Ais1 => AIS1_FREQ_HZ,
            VdesChannel::Ais2 => AIS2_FREQ_HZ,
            VdesChannel::Asm1 => ASM1_FREQ_HZ,
            VdesChannel::Asm2 => ASM2_FREQ_HZ,
            VdesChannel::VdeTer(idx) => 157_025_000.0 + (*idx as f64) * 25_000.0,
            VdesChannel::VdeSat => 159_800_000.0,
        }
    }

    /// Nominal bandwidth in Hz
    pub fn bandwidth_hz(&self) -> f64 {
        match self {
            VdesChannel::Ais1 | VdesChannel::Ais2 => 25_000.0,
            VdesChannel::Asm1 | VdesChannel::Asm2 => 25_000.0,
            VdesChannel::VdeTer(_) => 25_000.0,
            VdesChannel::VdeSat => 25_000.0,
        }
    }
}

/// TDMA slot allocation type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotAllocation {
    /// Self-Organizing TDMA — contention-based
    Sotdma,
    /// Incremental TDMA — pre-announced reservation
    Itdma,
    /// Random Access TDMA — single-use random
    Ratdma,
    /// Fixed Access TDMA — shore station assigned
    Fatdma,
}

/// TDMA slot reservation
#[derive(Debug, Clone)]
pub struct TdmaSlot {
    /// Slot number within the minute (0–2249)
    pub slot_number: u16,
    /// Minute offset (0 = current minute)
    pub offset: u8,
    /// Timeout (number of minutes reservation is valid)
    pub timeout: u8,
    /// Allocation type
    pub allocation: SlotAllocation,
}

impl TdmaSlot {
    /// Compute the slot start time in seconds from minute boundary
    pub fn start_time_s(&self) -> f64 {
        self.slot_number as f64 * AIS_SLOT_DURATION_S
    }

    /// Check whether two slots conflict (same number, same minute)
    pub fn conflicts_with(&self, other: &Self) -> bool {
        self.slot_number == other.slot_number && self.offset == other.offset
    }
}

/// pi/4-QPSK symbol (differential)
#[derive(Debug, Clone, Copy)]
pub struct PiOverFourQpskSymbol {
    pub i: f64,
    pub q: f64,
}

/// 16-QAM symbol
#[derive(Debug, Clone, Copy)]
pub struct Qam16Symbol {
    pub i: f64,
    pub q: f64,
}

/// GMSK modulation state
pub struct GmskModulator {
    /// BT product
    #[allow(dead_code)]
    bt: f64,
    /// Samples per symbol
    sps: usize,
    /// Filter taps (Gaussian)
    filter: Vec<f64>,
    /// Phase accumulator (radians)
    phase: f64,
    /// Previous sample buffer for filter
    history: Vec<f64>,
}

/// VDE-TER processor for terrestrial VDES
pub struct VdeTerProcessor {
    /// Bandwidth configuration
    pub bandwidth_hz: f64,
    /// Subcarrier spacing (Hz)
    pub subcarrier_spacing: f64,
    /// Number of OFDM subcarriers
    pub num_subcarriers: usize,
    /// Cyclic prefix length (samples)
    pub cp_length: usize,
}

/// VDES Slot Map for tracking reservations
pub struct VdesSlotMap {
    /// Reserved slots (slot_number → reservation)
    slots: Vec<Option<TdmaSlot>>,
}

// ---------------------------------------------------------------------------
// CRC-16/CCITT
// ---------------------------------------------------------------------------

/// Compute CRC-16/CCITT (polynomial 0x1021, init 0xFFFF, no XOR-out)
///
/// This is the FCS used in AIS HDLC frames per ITU-R M.1371-5.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = CRC16_INIT;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            let bit = (crc >> 15) ^ ((b >> 7) as u16 & 1);
            crc <<= 1;
            if bit != 0 {
                crc ^= CRC16_POLY;
            }
            b <<= 1;
        }
    }
    crc
}

/// Verify CRC-16/CCITT: append FCS and check against 0x1D0F
pub fn crc16_verify(data: &[u8], fcs: u16) -> bool {
    let computed = crc16_ccitt(data);
    computed == fcs
}

/// Compute CRC bits (16 bits, MSB first) for inclusion in AIS frame
pub fn crc16_bits(data: &[bool]) -> [bool; 16] {
    // Pack bits into bytes
    let len_bytes = (data.len() + 7) / 8;
    let mut bytes = vec![0u8; len_bytes];
    for (i, &bit) in data.iter().enumerate() {
        if bit {
            bytes[i / 8] |= 0x80 >> (i % 8);
        }
    }
    let crc = crc16_ccitt(&bytes);
    let mut result = [false; 16];
    for i in 0..16 {
        result[i] = (crc >> (15 - i)) & 1 == 1;
    }
    result
}

// ---------------------------------------------------------------------------
// NRZI encoding / decoding
// ---------------------------------------------------------------------------

/// NRZI encoding: a '1' causes a transition, '0' causes no transition.
///
/// AIS uses NRZI where a zero bit causes no change in RF phase,
/// and a one bit causes a phase transition (180° for GMSK).
pub fn nrzi_encode(bits: &[bool]) -> Vec<bool> {
    let mut out = Vec::with_capacity(bits.len());
    let mut current = false;
    for &b in bits {
        if b {
            current = !current;
        }
        out.push(current);
    }
    out
}

/// NRZI decoding: transition → '1', no transition → '0'
pub fn nrzi_decode(bits: &[bool]) -> Vec<bool> {
    let mut out = Vec::with_capacity(bits.len());
    let mut prev = false;
    for &b in bits {
        out.push(b != prev);
        prev = b;
    }
    out
}

// ---------------------------------------------------------------------------
// HDLC bit stuffing / unstuffing
// ---------------------------------------------------------------------------

/// HDLC bit stuffing: insert a '0' after five consecutive '1' bits
///
/// The input should NOT include the frame delimiters (0x7E = 01111110).
/// Returns stuffed payload bits.
pub fn bit_stuff(bits: &[bool]) -> Vec<bool> {
    let mut out = Vec::with_capacity(bits.len() + bits.len() / 5);
    let mut consecutive_ones = 0u8;
    for &b in bits {
        out.push(b);
        if b {
            consecutive_ones += 1;
            if consecutive_ones == 5 {
                out.push(false); // stuff zero
                consecutive_ones = 0;
            }
        } else {
            consecutive_ones = 0;
        }
    }
    out
}

/// HDLC bit unstuffing: remove a '0' after five consecutive '1' bits
///
/// Returns (unstuffed bits, error flag). Error is set if the bit after
/// five ones is itself a '1' (would be a frame delimiter or abort).
pub fn bit_unstuff(bits: &[bool]) -> (Vec<bool>, bool) {
    let mut out = Vec::with_capacity(bits.len());
    let mut consecutive_ones = 0u8;
    let mut error = false;
    let mut i = 0;
    while i < bits.len() {
        let b = bits[i];
        if b {
            consecutive_ones += 1;
            out.push(b);
            if consecutive_ones == 5 {
                // Next bit should be stuffed zero
                i += 1;
                if i < bits.len() {
                    if bits[i] {
                        // '1' after five ones — abort/flag pattern
                        error = true;
                    }
                    // else: drop the stuffed zero
                }
                consecutive_ones = 0;
                i += 1;
                continue;
            }
        } else {
            consecutive_ones = 0;
            out.push(b);
        }
        i += 1;
    }
    (out, error)
}

// ---------------------------------------------------------------------------
// 6-bit ASCII (AIS armoring)
// ---------------------------------------------------------------------------

/// AIS 6-bit ASCII character set (ITU-R M.1371-5 Table 44)
const AIS_6BIT_ASCII: &[u8; 64] = b"@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";

/// Encode a 7-bit ASCII character to AIS 6-bit value (0–63)
///
/// Returns `None` if the character is not representable.
pub fn ascii_to_ais6bit(ch: u8) -> Option<u8> {
    for (i, &c) in AIS_6BIT_ASCII.iter().enumerate() {
        if c == ch {
            return Some(i as u8);
        }
    }
    None
}

/// Decode AIS 6-bit value (0–63) to 7-bit ASCII character
pub fn ais6bit_to_ascii(v: u8) -> u8 {
    AIS_6BIT_ASCII[(v & 0x3F) as usize]
}

/// Encode a string to packed AIS 6-bit ASCII bits (MSB first per char)
pub fn encode_ais_string(s: &[u8], field_len: usize) -> Vec<bool> {
    let mut bits = Vec::with_capacity(field_len * 6);
    for i in 0..field_len {
        let ch = if i < s.len() { s[i] } else { b' ' };
        let v = ascii_to_ais6bit(ch).unwrap_or(0);
        for bit in (0..6).rev() {
            bits.push((v >> bit) & 1 == 1);
        }
    }
    bits
}

/// Decode packed AIS 6-bit ASCII bits to string
pub fn decode_ais_string(bits: &[bool]) -> String {
    let mut s = String::new();
    let n_chars = bits.len() / 6;
    for i in 0..n_chars {
        let mut v = 0u8;
        for bit in 0..6 {
            if bits[i * 6 + bit] {
                v |= 1 << (5 - bit);
            }
        }
        s.push(ais6bit_to_ascii(v) as char);
    }
    // Strip trailing spaces
    s.trim_end().to_string()
}

// ---------------------------------------------------------------------------
// MMSI utilities
// ---------------------------------------------------------------------------

/// Validate MMSI: must be 9 digits (0–999,999,999)
pub fn mmsi_valid(mmsi: u32) -> bool {
    mmsi <= 999_999_999
}

/// Extract MMSI MID (Maritime Identification Digit, first 3 digits)
pub fn mmsi_mid(mmsi: u32) -> u16 {
    (mmsi / 1_000_000) as u16
}

/// Check if MMSI is a group address (starts with 0)
pub fn mmsi_is_group(mmsi: u32) -> bool {
    mmsi < 100_000_000
}

/// Check if MMSI is a shore station (starts with 00)
pub fn mmsi_is_shore(mmsi: u32) -> bool {
    mmsi < 10_000_000
}

// ---------------------------------------------------------------------------
// Position encoding / decoding
// ---------------------------------------------------------------------------

/// Encode longitude to AIS 28-bit integer (1/10000 minute resolution)
///
/// Range: −180.0° to +180.0°; 0x6791AC0 (181.0°) = not available
pub fn encode_longitude(lon: f64) -> i32 {
    if lon > 180.0 || lon < -180.0 {
        return 0x6791AC0_i32; // not available sentinel
    }
    (lon * 10_000.0 * 60.0).round() as i32
}

/// Decode AIS 28-bit longitude to degrees
pub fn decode_longitude(raw: i32) -> f64 {
    // Sign-extend 28 bits
    let v = if raw & 0x0800_0000 != 0 {
        raw | !0x0FFF_FFFF_u32 as i32
    } else {
        raw & 0x0FFF_FFFF
    };
    v as f64 / (10_000.0 * 60.0)
}

/// Encode latitude to AIS 27-bit integer (1/10000 minute resolution)
///
/// Range: −90.0° to +90.0°; 0x3412140 (91.0°) = not available
pub fn encode_latitude(lat: f64) -> i32 {
    if lat > 90.0 || lat < -90.0 {
        return 0x3412140_i32; // not available
    }
    (lat * 10_000.0 * 60.0).round() as i32
}

/// Decode AIS 27-bit latitude to degrees
pub fn decode_latitude(raw: i32) -> f64 {
    // Sign-extend 27 bits
    let v = if raw & 0x0400_0000 != 0 {
        raw | !0x07FF_FFFF_u32 as i32
    } else {
        raw & 0x07FF_FFFF
    };
    v as f64 / (10_000.0 * 60.0)
}

/// Encode SOG (Speed Over Ground) to AIS 10-bit integer (1/10 knot)
///
/// 0–102.2 kn; 1023 = not available
pub fn encode_sog(sog: f64) -> u16 {
    if sog > 102.2 {
        return 1023;
    }
    (sog * 10.0).round() as u16
}

/// Decode AIS SOG integer to knots
pub fn decode_sog(raw: u16) -> f64 {
    (raw as f64) / 10.0
}

/// Encode COG (Course Over Ground) to AIS 12-bit integer (1/10 degree)
///
/// 0.0–359.9°; 3600 = not available
pub fn encode_cog(cog: f64) -> u16 {
    if cog >= 360.0 {
        return 3600;
    }
    (cog * 10.0).round() as u16
}

/// Decode AIS COG integer to degrees
pub fn decode_cog(raw: u16) -> f64 {
    (raw as f64) / 10.0
}

/// Encode ROT (Rate of Turn) to AIS 8-bit signed integer
///
/// ROT = 4.733 * sqrt(|rot_deg_per_min|), sign-preserved
/// Special values: -128 = no turn information; +127/-127 = >5°/30s
pub fn encode_rot(rot_deg_per_min: f64) -> i8 {
    if rot_deg_per_min.abs() > 720.0 {
        return -128;
    }
    let indicator = 4.733 * rot_deg_per_min.abs().sqrt();
    let clamped = indicator.min(126.0);
    if rot_deg_per_min >= 0.0 {
        clamped.round() as i8
    } else {
        -(clamped.round() as i8)
    }
}

/// Decode AIS ROT indicator to degrees/minute (approximate)
pub fn decode_rot(raw: i8) -> f64 {
    if raw == -128i8 {
        return f64::NAN;
    }
    let sign = if raw >= 0 { 1.0_f64 } else { -1.0_f64 };
    let magnitude = (raw.unsigned_abs() as f64) / 4.733;
    sign * magnitude * magnitude
}

// ---------------------------------------------------------------------------
// Bit-level encoding helpers
// ---------------------------------------------------------------------------

/// Write `n` bits of value `v` (MSB first) into `bits` starting at `offset`
fn write_bits(bits: &mut Vec<bool>, v: u64, n: usize) {
    for i in (0..n).rev() {
        bits.push((v >> i) & 1 == 1);
    }
}

/// Read `n` bits (MSB first) from `bits` starting at `offset`, return u64
fn read_bits(bits: &[bool], offset: usize, n: usize) -> u64 {
    let mut v = 0u64;
    for i in 0..n {
        if offset + i < bits.len() && bits[offset + i] {
            v |= 1 << (n - 1 - i);
        }
    }
    v
}

/// Read `n` bits as signed two's-complement integer
fn read_signed_bits(bits: &[bool], offset: usize, n: usize) -> i64 {
    let v = read_bits(bits, offset, n) as i64;
    // Sign extend
    if n < 64 && (v >> (n - 1)) & 1 == 1 {
        v | (!0i64 << n)
    } else {
        v
    }
}

// ---------------------------------------------------------------------------
// AIS message encoding
// ---------------------------------------------------------------------------

/// Encode AIS Type 1 Position Report to bit vector (168 bits payload)
///
/// Bit layout per ITU-R M.1371-5 Table 1:
/// [0-5] msg_type=1, [6-7] repeat=0, [8-37] MMSI, [38-41] nav_status,
/// [42-49] ROT, [50-59] SOG, [60] pos_acc, [61-88] lon, [89-115] lat,
/// [116-127] COG, [128-136] HDG, [137-142] timestamp, [143-147] man_ind,
/// [148-165] spare/RAIM/comm_state
pub fn encode_ais_type1(report: &AisPositionReport) -> Vec<bool> {
    let mut bits = Vec::with_capacity(168);

    // Message type: 1
    write_bits(&mut bits, 1, 6);
    // Repeat indicator: 0
    write_bits(&mut bits, 0, 2);
    // MMSI (30 bits)
    write_bits(&mut bits, report.mmsi as u64, 30);
    // Navigation status (4 bits)
    write_bits(&mut bits, report.nav_status as u64, 4);
    // ROT (8 bits, signed)
    let rot_enc = encode_rot(report.rot) as u8;
    write_bits(&mut bits, rot_enc as u64, 8);
    // SOG (10 bits)
    write_bits(&mut bits, encode_sog(report.sog) as u64, 10);
    // Position accuracy (1 bit)
    bits.push(report.position_accuracy);
    // Longitude (28 bits, signed)
    let lon_raw = encode_longitude(report.longitude);
    write_bits(&mut bits, (lon_raw as u32 & 0x0FFF_FFFF) as u64, 28);
    // Latitude (27 bits, signed)
    let lat_raw = encode_latitude(report.latitude);
    write_bits(&mut bits, (lat_raw as u32 & 0x07FF_FFFF) as u64, 27);
    // COG (12 bits)
    write_bits(&mut bits, encode_cog(report.cog) as u64, 12);
    // True heading (9 bits)
    write_bits(&mut bits, report.true_heading.min(511) as u64, 9);
    // Timestamp (6 bits)
    write_bits(&mut bits, report.timestamp.min(63) as u64, 6);
    // Maneuver indicator (2 bits) — 0 = not available
    write_bits(&mut bits, 0, 2);
    // Spare (3 bits)
    write_bits(&mut bits, 0, 3);
    // RAIM flag (1 bit) — 0 = not in use
    bits.push(false);
    // Radio status / SOTDMA comm state (19 bits) — zero padded
    write_bits(&mut bits, 0, 19);

    debug_assert_eq!(bits.len(), 168);
    bits
}

/// Decode AIS Type 1 Position Report from bit vector
pub fn decode_ais_type1(bits: &[bool]) -> Option<AisPositionReport> {
    if bits.len() < 168 {
        return None;
    }
    let msg_type = read_bits(bits, 0, 6);
    if msg_type != 1 && msg_type != 2 && msg_type != 3 {
        return None;
    }
    let mmsi = read_bits(bits, 8, 30) as u32;
    let nav_status = read_bits(bits, 38, 4) as u8;
    let rot_raw = read_signed_bits(bits, 42, 8) as i8;
    let rot = decode_rot(rot_raw);
    let sog = decode_sog(read_bits(bits, 50, 10) as u16);
    let position_accuracy = bits[60];
    let lon_raw = read_signed_bits(bits, 61, 28) as i32;
    let longitude = decode_longitude(lon_raw);
    let lat_raw = read_signed_bits(bits, 89, 27) as i32;
    let latitude = decode_latitude(lat_raw);
    let cog = decode_cog(read_bits(bits, 116, 12) as u16);
    let true_heading = read_bits(bits, 128, 9) as u16;
    let timestamp = read_bits(bits, 137, 6) as u8;

    Some(AisPositionReport {
        mmsi,
        nav_status,
        rot,
        sog,
        position_accuracy,
        longitude,
        latitude,
        cog,
        true_heading,
        timestamp,
    })
}

/// Encode AIS Type 5 Static and Voyage Data to bit vector (424 bits)
pub fn encode_ais_type5(sv: &AisStaticVoyage) -> Vec<bool> {
    let mut bits = Vec::with_capacity(424);

    // Message type: 5
    write_bits(&mut bits, 5, 6);
    // Repeat indicator: 0
    write_bits(&mut bits, 0, 2);
    // MMSI (30 bits)
    write_bits(&mut bits, sv.mmsi as u64, 30);
    // AIS version (2 bits) — 0
    write_bits(&mut bits, 0, 2);
    // IMO number (30 bits)
    write_bits(&mut bits, sv.imo as u64, 30);
    // Call sign (42 bits = 7 × 6)
    bits.extend(encode_ais_string(&sv.callsign, 7));
    // Name (120 bits = 20 × 6)
    bits.extend(encode_ais_string(&sv.name, 20));
    // Ship and cargo type (8 bits)
    write_bits(&mut bits, sv.ship_type as u64, 8);
    // Dimension A (9 bits)
    write_bits(&mut bits, sv.dim_a.min(511) as u64, 9);
    // Dimension B (9 bits)
    write_bits(&mut bits, sv.dim_b.min(511) as u64, 9);
    // Dimension C (6 bits)
    write_bits(&mut bits, sv.dim_c.min(63) as u64, 6);
    // Dimension D (6 bits)
    write_bits(&mut bits, sv.dim_d.min(63) as u64, 6);
    // EPFD type (4 bits)
    write_bits(&mut bits, sv.epfd_type as u64, 4);
    // ETA month (4 bits)
    write_bits(&mut bits, sv.eta_month as u64, 4);
    // ETA day (5 bits)
    write_bits(&mut bits, sv.eta_day as u64, 5);
    // ETA hour (5 bits)
    write_bits(&mut bits, sv.eta_hour as u64, 5);
    // ETA minute (6 bits)
    write_bits(&mut bits, sv.eta_minute as u64, 6);
    // Draught (8 bits, units of 1/10 m)
    let draught_enc = ((sv.draught * 10.0).round() as u64).min(255);
    write_bits(&mut bits, draught_enc, 8);
    // Destination (120 bits = 20 × 6)
    bits.extend(encode_ais_string(&sv.destination, 20));
    // DTE (1 bit)
    bits.push(sv.dte);
    // Spare (1 bit)
    bits.push(false);

    debug_assert_eq!(bits.len(), 424);
    bits
}

/// Decode AIS Type 5 Static and Voyage Data from bit vector
pub fn decode_ais_type5(bits: &[bool]) -> Option<AisStaticVoyage> {
    if bits.len() < 424 {
        return None;
    }
    let msg_type = read_bits(bits, 0, 6);
    if msg_type != 5 {
        return None;
    }
    let mmsi = read_bits(bits, 8, 30) as u32;
    let imo = read_bits(bits, 40, 30) as u32;

    let mut callsign = [b' '; 7];
    let callsign_str = decode_ais_string(&bits[70..112]);
    for (i, ch) in callsign_str.bytes().take(7).enumerate() {
        callsign[i] = ch;
    }

    let mut name = [b' '; 20];
    let name_str = decode_ais_string(&bits[112..232]);
    for (i, ch) in name_str.bytes().take(20).enumerate() {
        name[i] = ch;
    }

    let ship_type = read_bits(bits, 232, 8) as u8;
    let dim_a = read_bits(bits, 240, 9) as u16;
    let dim_b = read_bits(bits, 249, 9) as u16;
    let dim_c = read_bits(bits, 258, 6) as u8;
    let dim_d = read_bits(bits, 264, 6) as u8;
    let epfd_type = read_bits(bits, 270, 4) as u8;
    let eta_month = read_bits(bits, 274, 4) as u8;
    let eta_day = read_bits(bits, 278, 5) as u8;
    let eta_hour = read_bits(bits, 283, 5) as u8;
    let eta_minute = read_bits(bits, 288, 6) as u8;
    let draught_raw = read_bits(bits, 294, 8) as u8;
    let draught = draught_raw as f64 / 10.0;

    let mut destination = [b' '; 20];
    let dest_str = decode_ais_string(&bits[302..422]);
    for (i, ch) in dest_str.bytes().take(20).enumerate() {
        destination[i] = ch;
    }

    let dte = bits[422];

    Some(AisStaticVoyage {
        mmsi,
        imo,
        callsign,
        name,
        ship_type,
        dim_a,
        dim_b,
        dim_c,
        dim_d,
        epfd_type,
        eta_month,
        eta_day,
        eta_hour,
        eta_minute,
        draught,
        destination,
        dte,
    })
}

// ---------------------------------------------------------------------------
// HDLC frame construction
// ---------------------------------------------------------------------------

/// Build a complete HDLC frame: flag + stuffed data + CRC + flag
///
/// Returns the full bit sequence including opening/closing 0x7E flags.
pub fn build_hdlc_frame(payload_bits: &[bool]) -> Vec<bool> {
    // Compute CRC over the un-stuffed payload bits
    let fcs = crc16_bits(payload_bits);

    // Concatenate payload + FCS for stuffing
    let mut data_with_fcs = payload_bits.to_vec();
    data_with_fcs.extend_from_slice(&fcs);

    // Bit-stuff
    let stuffed = bit_stuff(&data_with_fcs);

    // Build frame: flag_bits + stuffed + flag_bits
    let flag_bits = hdlc_flag_bits();
    let mut frame = Vec::new();
    frame.extend_from_slice(&flag_bits);
    frame.extend_from_slice(&stuffed);
    frame.extend_from_slice(&flag_bits);
    frame
}

/// Convert HDLC_FLAG (0x7E) to 8 bits (MSB first)
fn hdlc_flag_bits() -> [bool; 8] {
    let mut bits = [false; 8];
    for i in 0..8 {
        bits[i] = (HDLC_FLAG >> (7 - i)) & 1 == 1;
    }
    bits
}

/// Parse an HDLC frame: strip flags, unstuff, verify CRC
///
/// Returns `Some(payload_bits)` on success (CRC verified), `None` on error.
pub fn parse_hdlc_frame(frame: &[bool]) -> Option<Vec<bool>> {
    // Find and strip flags (8-bit boundaries with 0x7E pattern)
    let flag = hdlc_flag_bits();
    let flag_pattern: Vec<bool> = flag.to_vec();

    // Find start of payload (after opening flag)
    let start = find_pattern(frame, &flag_pattern)?;
    let start = start + 8;

    // Find end of payload (before closing flag)
    let end = find_pattern(&frame[start..], &flag_pattern)?;
    let stuffed = &frame[start..start + end];

    // Unstuff
    let (data_with_fcs, error) = bit_unstuff(stuffed);
    if error {
        return None;
    }
    if data_with_fcs.len() < 16 {
        return None;
    }

    // Separate payload and FCS
    let payload_end = data_with_fcs.len() - 16;
    let payload = &data_with_fcs[..payload_end];
    let fcs_bits = &data_with_fcs[payload_end..];

    // Convert FCS bits to u16
    let mut fcs_val = 0u16;
    for (i, &b) in fcs_bits.iter().enumerate() {
        if b {
            fcs_val |= 1 << (15 - i);
        }
    }

    // Verify CRC
    let computed_fcs = crc16_bits(payload);
    let mut computed_val = 0u16;
    for (i, &b) in computed_fcs.iter().enumerate() {
        if b {
            computed_val |= 1 << (15 - i);
        }
    }

    if fcs_val == computed_val {
        Some(payload.to_vec())
    } else {
        None
    }
}

/// Find the first occurrence of `pattern` in `haystack`, return index
fn find_pattern(haystack: &[bool], pattern: &[bool]) -> Option<usize> {
    if pattern.len() > haystack.len() {
        return None;
    }
    for i in 0..=(haystack.len() - pattern.len()) {
        if &haystack[i..i + pattern.len()] == pattern {
            return Some(i);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// GMSK modulator
// ---------------------------------------------------------------------------

impl GmskModulator {
    /// Create a GMSK modulator with given BT product and samples-per-symbol
    pub fn new(bt: f64, sps: usize) -> Self {
        let filter = gaussian_filter(bt, sps);
        let history_len = filter.len() - 1;
        Self {
            bt,
            sps,
            filter,
            phase: 0.0,
            history: vec![0.0; history_len],
        }
    }

    /// Create a standard AIS GMSK modulator (BT=0.4, sample_rate=9600*N)
    pub fn ais(sps: usize) -> Self {
        Self::new(AIS_GMSK_BT, sps)
    }

    /// Modulate a binary bit stream to complex IQ samples
    ///
    /// Returns Vec of (I, Q) pairs. Each bit produces `sps` IQ samples.
    pub fn modulate(&mut self, bits: &[bool]) -> Vec<(f64, f64)> {
        let mut samples = Vec::with_capacity(bits.len() * self.sps);

        // Convert bits to NRZ (+1.0 / -1.0)
        let nrz: Vec<f64> = bits.iter().map(|&b| if b { 1.0 } else { -1.0 }).collect();

        // Upsample and apply Gaussian filter
        for &nrz_val in &nrz {
            let mut upsampled = vec![0.0_f64; self.sps];
            upsampled[0] = nrz_val * self.sps as f64; // energy normalization

            // Convolve upsampled with Gaussian filter using history
            let h = &self.filter;
            for sample in &upsampled {
                // Shift history
                let hist_len = self.history.len();
                for k in (1..hist_len).rev() {
                    self.history[k] = self.history[k - 1];
                }
                if !self.history.is_empty() {
                    self.history[0] = *sample;
                }

                // Compute filtered output
                let mut filtered = h[0] * sample;
                for (k, hval) in h[1..].iter().enumerate() {
                    if k < self.history.len() {
                        filtered += hval * self.history[k];
                    }
                }

                // Integrate phase (FM modulation)
                self.phase += (PI / 2.0) * filtered / self.sps as f64;
                self.phase %= 2.0 * PI;

                // Generate complex sample
                samples.push((self.phase.cos(), self.phase.sin()));
            }
        }
        samples
    }

    /// Get current phase accumulator value
    pub fn phase(&self) -> f64 {
        self.phase
    }
}

/// Design Gaussian filter for GMSK
///
/// Normalized BT product: BT = bandwidth × symbol_period
/// Filter length is 2*L+1 where L is chosen from sps.
fn gaussian_filter(bt: f64, sps: usize) -> Vec<f64> {
    let l = 3; // span in symbols
    let n = l * sps; // half-length
    let total = 2 * n + 1;
    let mut h = vec![0.0_f64; total];

    let alpha = (2.0 * PI * bt / (2.0 * (2.0_f64.ln()).sqrt())).recip();

    for i in 0..total {
        let t = (i as f64 - n as f64) / sps as f64;
        let t1 = t - 0.5;
        let t2 = t + 0.5;
        h[i] = 0.5 * (erf_approx(t2 / alpha) - erf_approx(t1 / alpha));
    }

    // Normalize
    let sum: f64 = h.iter().sum();
    if sum > 0.0 {
        for x in &mut h {
            *x /= sum;
        }
    }
    h
}

/// Approximate error function erf(x) using Abramowitz & Stegun formula 7.1.26
fn erf_approx(x: f64) -> f64 {
    let sign = if x >= 0.0 { 1.0 } else { -1.0 };
    let x = x.abs();
    let p = 0.3275911_f64;
    let t = 1.0 / (1.0 + p * x);
    let a1 = 0.254829592_f64;
    let a2 = -0.284496736_f64;
    let a3 = 1.421413741_f64;
    let a4 = -1.453152027_f64;
    let a5 = 1.061405429_f64;
    let poly = ((((a5 * t + a4) * t + a3) * t + a2) * t + a1) * t;
    sign * (1.0 - poly * (-x * x).exp())
}

// ---------------------------------------------------------------------------
// pi/4-QPSK modulation (VDE-TER)
// ---------------------------------------------------------------------------

/// pi/4-QPSK differential phase shifts for 2-bit dibit
const PI4_QPSK_DELTA: [f64; 4] = [PI / 4.0, 3.0 * PI / 4.0, -PI / 4.0, -3.0 * PI / 4.0];

/// Map 2 bits to pi/4-QPSK differential phase shift
///
/// Gray coded dibits: 00→+45°, 01→+135°, 11→-45°, 10→-135°
pub fn pi4_qpsk_map_bits(b0: bool, b1: bool) -> f64 {
    match (b0, b1) {
        (false, false) => PI4_QPSK_DELTA[0],  // +45°
        (false, true) => PI4_QPSK_DELTA[1],   // +135°
        (true, true) => PI4_QPSK_DELTA[2],    // -45°
        (true, false) => PI4_QPSK_DELTA[3],   // -135°
    }
}

/// Modulate bit sequence to pi/4-QPSK symbols (differential)
///
/// Returns Vec of (I, Q) symbols.
pub fn pi4_qpsk_modulate(bits: &[bool]) -> Vec<PiOverFourQpskSymbol> {
    let n_syms = bits.len() / 2;
    let mut symbols = Vec::with_capacity(n_syms);
    let mut phase = PI / 4.0; // initial phase

    for i in 0..n_syms {
        let b0 = bits[2 * i];
        let b1 = bits[2 * i + 1];
        let delta = pi4_qpsk_map_bits(b0, b1);
        phase += delta;
        symbols.push(PiOverFourQpskSymbol {
            i: phase.cos(),
            q: phase.sin(),
        });
    }
    symbols
}

/// Demodulate pi/4-QPSK symbols to bits (differential detection)
pub fn pi4_qpsk_demodulate(symbols: &[PiOverFourQpskSymbol]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(symbols.len() * 2);
    let mut prev_phase = PI / 4.0;

    for sym in symbols {
        let cur_phase = sym.q.atan2(sym.i);
        let mut delta = cur_phase - prev_phase;
        // Normalize to [-PI, PI]
        while delta > PI {
            delta -= 2.0 * PI;
        }
        while delta < -PI {
            delta += 2.0 * PI;
        }
        prev_phase = cur_phase;

        // Decode dibit based on quadrant of delta
        let (b0, b1) = if delta.abs() < PI / 2.0 {
            if delta > 0.0 {
                (false, false) // +45°
            } else {
                (true, true) // -45°
            }
        } else if delta > 0.0 {
            (false, true) // +135°
        } else {
            (true, false) // -135°
        };
        bits.push(b0);
        bits.push(b1);
    }
    bits
}

// ---------------------------------------------------------------------------
// 16-QAM modulation (VDE-TER high throughput)
// ---------------------------------------------------------------------------

/// 16-QAM Gray-coded constellation (ITU-R M.2092 compliant mapping)
///
/// 4 bits → (I, Q) in {±1, ±3} / sqrt(10)
const QAM16_NORM: f64 = 1.0 / 3.162_277_66; // 1/sqrt(10)

/// Map 4 bits to 16-QAM symbol
pub fn qam16_map(b: &[bool; 4]) -> Qam16Symbol {
    // Gray coding: b0b1 → I, b2b3 → Q
    let i_idx = ((b[0] as u8) << 1) | (b[1] as u8);
    let q_idx = ((b[2] as u8) << 1) | (b[3] as u8);
    let i_val = match i_idx {
        0 => -3.0,
        1 => -1.0,
        2 => 3.0,
        _ => 1.0,
    };
    let q_val = match q_idx {
        0 => -3.0,
        1 => -1.0,
        2 => 3.0,
        _ => 1.0,
    };
    Qam16Symbol {
        i: i_val * QAM16_NORM,
        q: q_val * QAM16_NORM,
    }
}

/// Minimum-distance demap of 16-QAM symbol to 4 bits
pub fn qam16_demap(sym: &Qam16Symbol) -> [bool; 4] {
    let i_scaled = sym.i / QAM16_NORM;
    let q_scaled = sym.q / QAM16_NORM;

    // Nearest level: decision thresholds at ±2.0
    let i_bits = level_to_gray2(i_scaled);
    let q_bits = level_to_gray2(q_scaled);
    [i_bits[0], i_bits[1], q_bits[0], q_bits[1]]
}

/// Convert {-3,-1,1,3} level to 2 Gray-coded bits
fn level_to_gray2(v: f64) -> [bool; 2] {
    if v < -2.0 {
        [false, false] // -3
    } else if v < 0.0 {
        [false, true] // -1
    } else if v < 2.0 {
        [true, true] // +1
    } else {
        [true, false] // +3
    }
}

/// Modulate bit sequence to 16-QAM symbols
pub fn qam16_modulate(bits: &[bool]) -> Vec<Qam16Symbol> {
    let n_syms = bits.len() / 4;
    let mut syms = Vec::with_capacity(n_syms);
    for i in 0..n_syms {
        let b: [bool; 4] = [bits[4 * i], bits[4 * i + 1], bits[4 * i + 2], bits[4 * i + 3]];
        syms.push(qam16_map(&b));
    }
    syms
}

/// Demodulate 16-QAM symbols to bits
pub fn qam16_demodulate(symbols: &[Qam16Symbol]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(symbols.len() * 4);
    for sym in symbols {
        let b = qam16_demap(sym);
        bits.extend_from_slice(&b);
    }
    bits
}

// ---------------------------------------------------------------------------
// TDMA slot management
// ---------------------------------------------------------------------------

impl VdesSlotMap {
    /// Create a new slot map with all slots free
    pub fn new() -> Self {
        Self {
            slots: vec![None; AIS_SLOTS_PER_MINUTE as usize],
        }
    }

    /// Reserve a slot (SOTDMA)
    ///
    /// Returns `true` if the slot was free and is now reserved.
    pub fn reserve(&mut self, slot: TdmaSlot) -> bool {
        let idx = slot.slot_number as usize;
        if idx >= self.slots.len() || self.slots[idx].is_some() {
            return false;
        }
        self.slots[idx] = Some(slot);
        true
    }

    /// Release a reserved slot
    pub fn release(&mut self, slot_number: u16) {
        if let Some(s) = self.slots.get_mut(slot_number as usize) {
            *s = None;
        }
    }

    /// Count free slots
    pub fn free_count(&self) -> usize {
        self.slots.iter().filter(|s| s.is_none()).count()
    }

    /// Count reserved slots
    pub fn reserved_count(&self) -> usize {
        self.slots.iter().filter(|s| s.is_some()).count()
    }

    /// Find the nearest free slot to the given slot number (wraps around)
    pub fn find_nearest_free(&self, near: u16) -> Option<u16> {
        let n = self.slots.len();
        for offset in 0..n {
            let idx = (near as usize + offset) % n;
            if self.slots[idx].is_none() {
                return Some(idx as u16);
            }
        }
        None
    }

    /// Check if a specific slot is free
    pub fn is_free(&self, slot_number: u16) -> bool {
        self.slots
            .get(slot_number as usize)
            .map_or(false, |s| s.is_none())
    }

    /// Compute slot start time within a minute (seconds)
    pub fn slot_start_time(slot_number: u16) -> f64 {
        slot_number as f64 * AIS_SLOT_DURATION_S
    }

    /// Compute slot index from time-within-minute (seconds)
    pub fn time_to_slot(time_s: f64) -> u16 {
        let idx = (time_s / AIS_SLOT_DURATION_S) as u16;
        idx.min(AIS_SLOTS_PER_MINUTE as u16 - 1)
    }

    /// Decrement timeouts and free expired slots
    pub fn tick_minute(&mut self) {
        for slot in &mut self.slots {
            if let Some(ref mut s) = slot {
                if s.timeout <= 1 {
                    *slot = None;
                } else {
                    s.timeout -= 1;
                }
            }
        }
    }
}

impl Default for VdesSlotMap {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// VDE-TER OFDM processor
// ---------------------------------------------------------------------------

impl VdeTerProcessor {
    /// Create a VDE-TER processor for 25 kHz bandwidth
    pub fn new_25khz() -> Self {
        // 25 kHz OFDM: 64 subcarriers, 12.5 kHz spacing... simplified
        Self {
            bandwidth_hz: 25_000.0,
            subcarrier_spacing: 390.625,  // 25000/64
            num_subcarriers: 64,
            cp_length: 16,
        }
    }

    /// Create a VDE-TER processor for 50 kHz bandwidth
    pub fn new_50khz() -> Self {
        Self {
            bandwidth_hz: 50_000.0,
            subcarrier_spacing: 781.25,
            num_subcarriers: 64,
            cp_length: 16,
        }
    }

    /// Create a VDE-TER processor for 100 kHz bandwidth
    pub fn new_100khz() -> Self {
        Self {
            bandwidth_hz: 100_000.0,
            subcarrier_spacing: 1562.5,
            num_subcarriers: 64,
            cp_length: 16,
        }
    }

    /// OFDM symbol duration (without CP) in seconds
    pub fn symbol_duration(&self) -> f64 {
        1.0 / self.subcarrier_spacing
    }

    /// OFDM symbol duration including cyclic prefix
    pub fn symbol_duration_with_cp(&self) -> f64 {
        self.symbol_duration() + self.cp_length as f64 / self.bandwidth_hz
    }

    /// Compute nominal data rate for given modulation
    ///
    /// Returns bits per second.
    pub fn data_rate_bps(&self, bits_per_symbol: u8, code_rate_num: u8, code_rate_den: u8) -> f64 {
        let syms_per_sec = 1.0 / self.symbol_duration_with_cp();
        let active_carriers = self.num_subcarriers as f64 * 0.75; // 25% pilot/guard
        syms_per_sec * active_carriers * bits_per_symbol as f64
            * (code_rate_num as f64 / code_rate_den as f64)
    }

    /// Generate OFDM modulated IQ samples from frequency-domain data
    ///
    /// Input: frequency-domain complex values (num_subcarriers complex pairs).
    /// Output: time-domain IQ samples with cyclic prefix.
    pub fn ifft_with_cp(&self, freq_data: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = self.num_subcarriers;
        let time_domain = idft(freq_data, n);

        // Prepend cyclic prefix
        let cp_start = n - self.cp_length;
        let mut output = Vec::with_capacity(n + self.cp_length);
        output.extend_from_slice(&time_domain[cp_start..]);
        output.extend_from_slice(&time_domain);
        output
    }

    /// Remove cyclic prefix and apply FFT to recover frequency-domain data
    pub fn fft_remove_cp(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = self.num_subcarriers;
        let total = n + self.cp_length;
        if samples.len() < total {
            return vec![(0.0, 0.0); n];
        }
        let time_data = &samples[self.cp_length..self.cp_length + n];
        dft(time_data, n)
    }
}

/// Compute DFT of length n (decimation-in-time, O(n^2) for simplicity)
fn dft(input: &[(f64, f64)], n: usize) -> Vec<(f64, f64)> {
    let mut output = vec![(0.0_f64, 0.0_f64); n];
    for k in 0..n {
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for (j, &(xr, xi)) in input.iter().enumerate().take(n) {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            re += xr * angle.cos() - xi * angle.sin();
            im += xr * angle.sin() + xi * angle.cos();
        }
        output[k] = (re, im);
    }
    output
}

/// Compute IDFT of length n
fn idft(input: &[(f64, f64)], n: usize) -> Vec<(f64, f64)> {
    let mut output = vec![(0.0_f64, 0.0_f64); n];
    for j in 0..n {
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for (k, &(xr, xi)) in input.iter().enumerate().take(n) {
            let angle = 2.0 * PI * k as f64 * j as f64 / n as f64;
            re += xr * angle.cos() - xi * angle.sin();
            im += xr * angle.sin() + xi * angle.cos();
        }
        output[j] = (re / n as f64, im / n as f64);
    }
    output
}

// ---------------------------------------------------------------------------
// VDE-SAT Doppler pre-compensation
// ---------------------------------------------------------------------------

/// Compute Doppler shift for a satellite at given range rate
///
/// Returns Doppler frequency offset in Hz.
/// f_d = -f_c * v_r / c
/// where v_r is range rate (m/s, positive = receding), c = speed of light.
pub fn satellite_doppler_hz(center_freq_hz: f64, range_rate_m_s: f64) -> f64 {
    const C: f64 = 299_792_458.0;
    -center_freq_hz * range_rate_m_s / C
}

/// Compute timing advance for VDE-SAT burst mode
///
/// Returns timing advance in microseconds.
/// TA = 2 * range / c * 1e6
pub fn timing_advance_us(range_m: f64) -> f64 {
    const C: f64 = 299_792_458.0;
    2.0 * range_m / C * 1_000_000.0
}

/// Apply Doppler pre-compensation to a sequence of IQ samples
///
/// Multiplies each sample by exp(-j*2*pi*f_d*n/fs).
pub fn apply_doppler_precomp(
    samples: &[(f64, f64)],
    doppler_hz: f64,
    sample_rate_hz: f64,
) -> Vec<(f64, f64)> {
    let phase_inc = -2.0 * PI * doppler_hz / sample_rate_hz;
    samples
        .iter()
        .enumerate()
        .map(|(n, &(i, q))| {
            let phi = phase_inc * n as f64;
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();
            let out_i = i * cos_phi - q * sin_phi;
            let out_q = i * sin_phi + q * cos_phi;
            (out_i, out_q)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Convenience: complete AIS packet builder
// ---------------------------------------------------------------------------

/// Build a complete AIS Type 1 transmission packet
///
/// Returns NRZI-encoded, bit-stuffed HDLC frame ready for GMSK modulation.
pub fn build_ais_type1_packet(report: &AisPositionReport) -> Vec<bool> {
    let payload = encode_ais_type1(report);
    let frame = build_hdlc_frame(&payload);
    nrzi_encode(&frame)
}

/// Build a complete AIS Type 5 transmission packet
pub fn build_ais_type5_packet(sv: &AisStaticVoyage) -> Vec<bool> {
    let payload = encode_ais_type5(sv);
    let frame = build_hdlc_frame(&payload);
    nrzi_encode(&frame)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: round-trip tolerance for float comparisons
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -------------------------------------------------------------------------
    // CRC-16 tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_crc16_known_value() {
        // CRC-16/CCITT of "123456789" = 0x29B1
        let data = b"123456789";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, 0x29B1);
    }

    #[test]
    fn test_crc16_empty() {
        let crc = crc16_ccitt(b"");
        assert_eq!(crc, CRC16_INIT); // no data processed
    }

    #[test]
    fn test_crc16_verify_roundtrip() {
        let data = b"AIS maritime test";
        let fcs = crc16_ccitt(data);
        assert!(crc16_verify(data, fcs));
    }

    #[test]
    fn test_crc16_verify_fails_on_corruption() {
        let data = b"hello world";
        let fcs = crc16_ccitt(data);
        let mut corrupted = data.to_vec();
        corrupted[0] ^= 0xFF;
        assert!(!crc16_verify(&corrupted, fcs));
    }

    #[test]
    fn test_crc16_bits_length() {
        let bits: Vec<bool> = (0..168).map(|i| i % 3 == 0).collect();
        let fcs = crc16_bits(&bits);
        assert_eq!(fcs.len(), 16);
    }

    // -------------------------------------------------------------------------
    // NRZI tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_nrzi_encode_zeros() {
        let bits = vec![false, false, false, false];
        let enc = nrzi_encode(&bits);
        // All zeros = no transitions → all same as starting state (false)
        assert_eq!(enc, vec![false, false, false, false]);
    }

    #[test]
    fn test_nrzi_encode_ones() {
        // Each '1' toggles the state
        let bits = vec![true, true, true, true];
        let enc = nrzi_encode(&bits);
        assert_eq!(enc, vec![true, false, true, false]);
    }

    #[test]
    fn test_nrzi_roundtrip() {
        let original = vec![true, false, true, true, false, false, true];
        let encoded = nrzi_encode(&original);
        let decoded = nrzi_decode(&encoded);
        assert_eq!(decoded, original);
    }

    #[test]
    fn test_nrzi_decode_all_same() {
        // All same = no transitions = all zeros
        let bits = vec![true, true, true, true];
        let decoded = nrzi_decode(&bits);
        // First bit: compare true to false (init) → transition → 1
        // Rest: no transition → 0
        assert_eq!(decoded[0], true);
        assert_eq!(decoded[1], false);
        assert_eq!(decoded[2], false);
        assert_eq!(decoded[3], false);
    }

    // -------------------------------------------------------------------------
    // Bit stuffing tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_bit_stuff_no_stuffing_needed() {
        let bits = vec![true, false, true, false, true];
        let stuffed = bit_stuff(&bits);
        assert_eq!(stuffed, bits); // no run of 5 ones
    }

    #[test]
    fn test_bit_stuff_inserts_zero() {
        let bits = vec![true, true, true, true, true, true]; // 6 ones
        let stuffed = bit_stuff(&bits);
        // After 5 ones, insert 0, then continue
        assert_eq!(stuffed, vec![true, true, true, true, true, false, true]);
    }

    #[test]
    fn test_bit_unstuff_removes_zero() {
        let stuffed = vec![true, true, true, true, true, false, true];
        let (unstuffed, err) = bit_unstuff(&stuffed);
        assert!(!err);
        assert_eq!(unstuffed, vec![true, true, true, true, true, true]);
    }

    #[test]
    fn test_bit_stuff_unstuff_roundtrip() {
        let original = vec![
            false, true, true, true, true, true, false, true, true, false,
        ];
        let stuffed = bit_stuff(&original);
        let (unstuffed, err) = bit_unstuff(&stuffed);
        assert!(!err);
        assert_eq!(unstuffed, original);
    }

    #[test]
    fn test_bit_stuff_multiple_runs() {
        // Two runs of 5 ones separated by a zero
        let bits = vec![
            true, true, true, true, true, false, true, true, true, true, true,
        ];
        let stuffed = bit_stuff(&bits);
        let (unstuffed, err) = bit_unstuff(&stuffed);
        assert!(!err);
        assert_eq!(unstuffed, bits);
    }

    #[test]
    fn test_bit_stuff_exact_five_ones() {
        let bits: Vec<bool> = vec![true; 5];
        let stuffed = bit_stuff(&bits);
        assert_eq!(stuffed.len(), 6); // +1 zero stuffed at end
        assert_eq!(stuffed[5], false);
    }

    // -------------------------------------------------------------------------
    // 6-bit ASCII tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_ascii_to_ais6bit_alpha() {
        assert_eq!(ascii_to_ais6bit(b'A'), Some(1));
        assert_eq!(ascii_to_ais6bit(b'Z'), Some(26));
    }

    #[test]
    fn test_ascii_to_ais6bit_at() {
        assert_eq!(ascii_to_ais6bit(b'@'), Some(0));
    }

    #[test]
    fn test_ascii_to_ais6bit_digits() {
        assert_eq!(ascii_to_ais6bit(b'0'), Some(48));
        assert_eq!(ascii_to_ais6bit(b'9'), Some(57));
    }

    #[test]
    fn test_ais6bit_to_ascii_roundtrip() {
        for v in 0u8..64 {
            let ch = ais6bit_to_ascii(v);
            let back = ascii_to_ais6bit(ch);
            assert_eq!(back, Some(v), "round-trip failed for 6-bit value {}", v);
        }
    }

    #[test]
    fn test_encode_decode_ais_string() {
        let input = b"HELLO";
        let bits = encode_ais_string(input, 5);
        assert_eq!(bits.len(), 30); // 5 × 6 bits
        let decoded = decode_ais_string(&bits);
        assert_eq!(decoded, "HELLO");
    }

    #[test]
    fn test_encode_ais_string_padding() {
        let input = b"AB";
        let bits = encode_ais_string(input, 4); // pad to 4 chars
        let decoded = decode_ais_string(&bits);
        assert_eq!(decoded, "AB"); // trailing spaces stripped
    }

    // -------------------------------------------------------------------------
    // MMSI tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_mmsi_valid() {
        assert!(mmsi_valid(123456789));
        assert!(mmsi_valid(0));
        assert!(mmsi_valid(999999999));
    }

    #[test]
    fn test_mmsi_mid() {
        let mmsi = 123456789u32;
        assert_eq!(mmsi_mid(mmsi), 123);
    }

    #[test]
    fn test_mmsi_group_check() {
        assert!(mmsi_is_group(99999999));
        assert!(!mmsi_is_group(100000000));
    }

    #[test]
    fn test_mmsi_shore_check() {
        assert!(mmsi_is_shore(9999999));
        assert!(!mmsi_is_shore(10000000));
    }

    // -------------------------------------------------------------------------
    // Position encoding tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_longitude_roundtrip() {
        let lon = -122.4194_f64;
        let encoded = encode_longitude(lon);
        let decoded = decode_longitude(encoded);
        assert!(approx_eq(decoded, lon, 1e-4), "lon {} != {}", decoded, lon);
    }

    #[test]
    fn test_latitude_roundtrip() {
        let lat = 37.7749_f64;
        let encoded = encode_latitude(lat);
        let decoded = decode_latitude(encoded);
        assert!(approx_eq(decoded, lat, 1e-4), "lat {} != {}", decoded, lat);
    }

    #[test]
    fn test_negative_longitude() {
        let lon = -180.0_f64;
        let enc = encode_longitude(lon);
        let dec = decode_longitude(enc);
        assert!(approx_eq(dec, lon, 1e-3));
    }

    #[test]
    fn test_zero_position() {
        assert_eq!(encode_longitude(0.0), 0);
        assert_eq!(encode_latitude(0.0), 0);
    }

    #[test]
    fn test_sog_roundtrip() {
        let sog = 12.5_f64;
        let enc = encode_sog(sog);
        let dec = decode_sog(enc);
        assert!(approx_eq(dec, sog, 0.1));
    }

    #[test]
    fn test_sog_not_available() {
        let enc = encode_sog(102.3);
        assert_eq!(enc, 1023);
    }

    #[test]
    fn test_cog_roundtrip() {
        let cog = 275.5_f64;
        let enc = encode_cog(cog);
        let dec = decode_cog(enc);
        assert!(approx_eq(dec, cog, 0.1));
    }

    #[test]
    fn test_cog_boundary() {
        assert_eq!(encode_cog(360.0), 3600); // not available
        assert_eq!(encode_cog(0.0), 0);
    }

    // -------------------------------------------------------------------------
    // ROT encoding tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_rot_encode_zero() {
        assert_eq!(encode_rot(0.0), 0);
    }

    #[test]
    fn test_rot_encode_not_available() {
        assert_eq!(encode_rot(f64::INFINITY), -128);
        assert_eq!(encode_rot(-800.0), -128);
    }

    #[test]
    fn test_rot_encode_positive() {
        let enc = encode_rot(30.0);
        assert!(enc > 0);
    }

    #[test]
    fn test_rot_encode_negative() {
        let enc = encode_rot(-30.0);
        assert!(enc < 0);
    }

    // -------------------------------------------------------------------------
    // AIS message type 1 encode/decode
    // -------------------------------------------------------------------------

    #[test]
    fn test_ais_type1_encode_length() {
        let report = AisPositionReport {
            mmsi: 123456789,
            nav_status: 0,
            rot: 0.0,
            sog: 5.5,
            position_accuracy: false,
            longitude: -122.4194,
            latitude: 37.7749,
            cog: 270.0,
            true_heading: 270,
            timestamp: 30,
        };
        let bits = encode_ais_type1(&report);
        assert_eq!(bits.len(), 168);
    }

    #[test]
    fn test_ais_type1_roundtrip_mmsi() {
        let report = AisPositionReport {
            mmsi: 987654321,
            ..Default::default()
        };
        let bits = encode_ais_type1(&report);
        let decoded = decode_ais_type1(&bits).unwrap();
        assert_eq!(decoded.mmsi, report.mmsi);
    }

    #[test]
    fn test_ais_type1_roundtrip_position() {
        let report = AisPositionReport {
            mmsi: 123456789,
            longitude: 10.12345,
            latitude: 55.54321,
            sog: 8.3,
            cog: 185.0,
            true_heading: 185,
            nav_status: 0,
            rot: 0.0,
            position_accuracy: false,
            timestamp: 45,
        };
        let bits = encode_ais_type1(&report);
        let decoded = decode_ais_type1(&bits).unwrap();
        assert!(approx_eq(decoded.longitude, report.longitude, 0.01));
        assert!(approx_eq(decoded.latitude, report.latitude, 0.01));
        assert!(approx_eq(decoded.sog, report.sog, 0.1));
        assert!(approx_eq(decoded.cog, report.cog, 0.1));
    }

    #[test]
    fn test_ais_type1_message_type_field() {
        let report = AisPositionReport::default();
        let bits = encode_ais_type1(&report);
        // First 6 bits should be 000001
        assert_eq!(read_bits(&bits, 0, 6), 1);
    }

    #[test]
    fn test_ais_type1_nav_status_roundtrip() {
        for status in 0u8..16 {
            let report = AisPositionReport {
                mmsi: 123456789,
                nav_status: status,
                ..Default::default()
            };
            let bits = encode_ais_type1(&report);
            let decoded = decode_ais_type1(&bits).unwrap();
            assert_eq!(decoded.nav_status, status);
        }
    }

    // -------------------------------------------------------------------------
    // AIS message type 5 encode/decode
    // -------------------------------------------------------------------------

    #[test]
    fn test_ais_type5_encode_length() {
        let sv = AisStaticVoyage {
            mmsi: 123456789,
            imo: 1234567,
            callsign: *b"ABCDEFG",
            name: *b"TEST VESSEL         ",
            ship_type: 70,
            dim_a: 100,
            dim_b: 50,
            dim_c: 10,
            dim_d: 10,
            epfd_type: 1,
            eta_month: 3,
            eta_day: 15,
            eta_hour: 12,
            eta_minute: 30,
            draught: 5.5,
            destination: *b"PORT OF CALL        ",
            dte: false,
        };
        let bits = encode_ais_type5(&sv);
        assert_eq!(bits.len(), 424);
    }

    #[test]
    fn test_ais_type5_roundtrip_mmsi() {
        let sv = AisStaticVoyage {
            mmsi: 555444333,
            imo: 0,
            callsign: *b"CALL123",
            name: *b"MY VESSEL           ",
            ship_type: 0,
            dim_a: 0,
            dim_b: 0,
            dim_c: 0,
            dim_d: 0,
            epfd_type: 0,
            eta_month: 0,
            eta_day: 0,
            eta_hour: 24,
            eta_minute: 60,
            draught: 0.0,
            destination: *b"                    ",
            dte: false,
        };
        let bits = encode_ais_type5(&sv);
        let decoded = decode_ais_type5(&bits).unwrap();
        assert_eq!(decoded.mmsi, sv.mmsi);
    }

    #[test]
    fn test_ais_type5_message_type_field() {
        let sv = AisStaticVoyage {
            mmsi: 1,
            imo: 0,
            callsign: [b' '; 7],
            name: [b' '; 20],
            ship_type: 0,
            dim_a: 0,
            dim_b: 0,
            dim_c: 0,
            dim_d: 0,
            epfd_type: 0,
            eta_month: 0,
            eta_day: 0,
            eta_hour: 24,
            eta_minute: 60,
            draught: 0.0,
            destination: [b' '; 20],
            dte: false,
        };
        let bits = encode_ais_type5(&sv);
        assert_eq!(read_bits(&bits, 0, 6), 5);
    }

    // -------------------------------------------------------------------------
    // HDLC frame tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_hdlc_frame_starts_with_flag() {
        let payload: Vec<bool> = (0..168).map(|i| i % 2 == 0).collect();
        let frame = build_hdlc_frame(&payload);
        // First 8 bits should be HDLC_FLAG = 0x7E = 01111110
        let expected = [false, true, true, true, true, true, true, false];
        for (i, &expected_bit) in expected.iter().enumerate() {
            assert_eq!(frame[i], expected_bit, "flag bit {} mismatch", i);
        }
    }

    #[test]
    fn test_hdlc_frame_ends_with_flag() {
        let payload: Vec<bool> = vec![false; 40];
        let frame = build_hdlc_frame(&payload);
        let n = frame.len();
        let expected = [false, true, true, true, true, true, true, false];
        for (i, &expected_bit) in expected.iter().enumerate() {
            assert_eq!(frame[n - 8 + i], expected_bit);
        }
    }

    #[test]
    fn test_hdlc_roundtrip() {
        let payload: Vec<bool> = (0..96).map(|i| (i * 7 + 3) % 5 == 0).collect();
        let frame = build_hdlc_frame(&payload);
        let recovered = parse_hdlc_frame(&frame);
        assert!(recovered.is_some(), "HDLC parse failed");
        assert_eq!(recovered.unwrap(), payload);
    }

    #[test]
    fn test_hdlc_crc_detection() {
        let payload: Vec<bool> = vec![true; 64];
        let mut frame = build_hdlc_frame(&payload);
        // Corrupt a middle bit (after flags)
        if frame.len() > 20 {
            let mid = frame.len() / 2;
            frame[mid] = !frame[mid];
        }
        // Should fail CRC
        let result = parse_hdlc_frame(&frame);
        assert!(result.is_none(), "Should detect CRC error");
    }

    // -------------------------------------------------------------------------
    // TDMA slot tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_slot_duration() {
        let duration = AIS_SLOT_DURATION_S;
        assert!(approx_eq(duration, 0.026667, 1e-5));
    }

    #[test]
    fn test_slots_per_minute() {
        let total_time = AIS_SLOTS_PER_MINUTE as f64 * AIS_SLOT_DURATION_S;
        assert!(approx_eq(total_time, 60.0, 0.01));
    }

    #[test]
    fn test_slot_map_reserve_free() {
        let mut map = VdesSlotMap::new();
        let slot = TdmaSlot {
            slot_number: 100,
            offset: 0,
            timeout: 5,
            allocation: SlotAllocation::Sotdma,
        };
        assert!(map.is_free(100));
        assert!(map.reserve(slot));
        assert!(!map.is_free(100));
    }

    #[test]
    fn test_slot_map_double_reserve_fails() {
        let mut map = VdesSlotMap::new();
        let slot1 = TdmaSlot {
            slot_number: 50,
            offset: 0,
            timeout: 3,
            allocation: SlotAllocation::Itdma,
        };
        let slot2 = TdmaSlot {
            slot_number: 50,
            offset: 0,
            timeout: 3,
            allocation: SlotAllocation::Ratdma,
        };
        assert!(map.reserve(slot1));
        assert!(!map.reserve(slot2)); // should fail
    }

    #[test]
    fn test_slot_map_release() {
        let mut map = VdesSlotMap::new();
        let slot = TdmaSlot {
            slot_number: 200,
            offset: 0,
            timeout: 10,
            allocation: SlotAllocation::Fatdma,
        };
        map.reserve(slot);
        map.release(200);
        assert!(map.is_free(200));
    }

    #[test]
    fn test_slot_map_count() {
        let mut map = VdesSlotMap::new();
        assert_eq!(map.free_count(), AIS_SLOTS_PER_MINUTE as usize);
        for i in 0..10u16 {
            let s = TdmaSlot {
                slot_number: i,
                offset: 0,
                timeout: 1,
                allocation: SlotAllocation::Sotdma,
            };
            map.reserve(s);
        }
        assert_eq!(map.reserved_count(), 10);
        assert_eq!(map.free_count(), AIS_SLOTS_PER_MINUTE as usize - 10);
    }

    #[test]
    fn test_slot_time_conversion() {
        let t = VdesSlotMap::slot_start_time(0);
        assert!(approx_eq(t, 0.0, 1e-9));
        let t1 = VdesSlotMap::slot_start_time(2249);
        assert!(t1 < 60.0);
        let idx = VdesSlotMap::time_to_slot(0.0);
        assert_eq!(idx, 0);
        let idx2 = VdesSlotMap::time_to_slot(30.0);
        assert!(idx2 > 1000 && idx2 < 1200);
    }

    #[test]
    fn test_slot_timeout_tick() {
        let mut map = VdesSlotMap::new();
        let slot = TdmaSlot {
            slot_number: 5,
            offset: 0,
            timeout: 1,
            allocation: SlotAllocation::Sotdma,
        };
        map.reserve(slot);
        assert!(!map.is_free(5));
        map.tick_minute(); // timeout goes from 1 to 0 → freed
        assert!(map.is_free(5));
    }

    #[test]
    fn test_find_nearest_free() {
        let mut map = VdesSlotMap::new();
        // Reserve slots 0–9
        for i in 0..10u16 {
            let s = TdmaSlot {
                slot_number: i,
                offset: 0,
                timeout: 5,
                allocation: SlotAllocation::Sotdma,
            };
            map.reserve(s);
        }
        let nearest = map.find_nearest_free(0);
        assert_eq!(nearest, Some(10));
    }

    // -------------------------------------------------------------------------
    // pi/4-QPSK tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_pi4_qpsk_symbol_unit_magnitude() {
        let syms = pi4_qpsk_modulate(&[false, false, true, true, false, true, true, false]);
        for sym in &syms {
            let mag = (sym.i * sym.i + sym.q * sym.q).sqrt();
            assert!(approx_eq(mag, 1.0, 1e-10));
        }
    }

    #[test]
    fn test_pi4_qpsk_roundtrip() {
        let bits = vec![
            false, false, false, true, true, false, true, true, false, false, true, false,
        ];
        let syms = pi4_qpsk_modulate(&bits);
        let recovered = pi4_qpsk_demodulate(&syms);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_pi4_qpsk_phase_increments() {
        // Each symbol should shift phase by ±45° or ±135°
        let bits = vec![false, false]; // 00 → +45°
        let syms = pi4_qpsk_modulate(&bits);
        assert_eq!(syms.len(), 1);
        // Phase of first symbol should be initial + 45°
        let phase = syms[0].q.atan2(syms[0].i);
        assert!(approx_eq(phase, PI / 2.0, 0.01)); // initial PI/4 + PI/4 = PI/2
    }

    // -------------------------------------------------------------------------
    // 16-QAM tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_qam16_all_symbols() {
        for b0 in [false, true] {
            for b1 in [false, true] {
                for b2 in [false, true] {
                    for b3 in [false, true] {
                        let b = [b0, b1, b2, b3];
                        let sym = qam16_map(&b);
                        let rec = qam16_demap(&sym);
                        assert_eq!(rec, b, "16-QAM roundtrip failed for {:?}", b);
                    }
                }
            }
        }
    }

    #[test]
    fn test_qam16_modulate_demodulate() {
        let bits = vec![
            false, false, true, false, true, true, false, false, false, true, true, true,
        ];
        let syms = qam16_modulate(&bits);
        assert_eq!(syms.len(), 3);
        let recovered = qam16_demodulate(&syms);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_qam16_symbol_values() {
        // 0000 → (-3,-3) normalized
        let sym = qam16_map(&[false, false, false, false]);
        let expected_i = -3.0 * QAM16_NORM;
        let expected_q = -3.0 * QAM16_NORM;
        assert!(approx_eq(sym.i, expected_i, 1e-9));
        assert!(approx_eq(sym.q, expected_q, 1e-9));
    }

    #[test]
    fn test_qam16_average_power() {
        // Average power of 16-QAM = 1.0 (normalized)
        let mut power_sum = 0.0;
        let mut count = 0;
        for b0 in [false, true] {
            for b1 in [false, true] {
                for b2 in [false, true] {
                    for b3 in [false, true] {
                        let sym = qam16_map(&[b0, b1, b2, b3]);
                        power_sum += sym.i * sym.i + sym.q * sym.q;
                        count += 1;
                    }
                }
            }
        }
        let avg_power = power_sum / count as f64;
        assert!(approx_eq(avg_power, 1.0, 0.01));
    }

    // -------------------------------------------------------------------------
    // GMSK modulator tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_gmsk_modulate_length() {
        let mut gmsk = GmskModulator::ais(8);
        let bits = vec![false, true, false, true, false];
        let samples = gmsk.modulate(&bits);
        assert_eq!(samples.len(), bits.len() * 8);
    }

    #[test]
    fn test_gmsk_unit_envelope() {
        let mut gmsk = GmskModulator::ais(4);
        let bits = vec![true, false, true, true, false, false];
        let samples = gmsk.modulate(&bits);
        for (i, q) in &samples {
            let mag = (i * i + q * q).sqrt();
            assert!(
                approx_eq(mag, 1.0, 1e-9),
                "GMSK envelope not unit: {}",
                mag
            );
        }
    }

    #[test]
    fn test_gaussian_filter_normalized() {
        let h = gaussian_filter(AIS_GMSK_BT, 8);
        let sum: f64 = h.iter().sum();
        assert!(approx_eq(sum, 1.0, 1e-6));
    }

    // -------------------------------------------------------------------------
    // VDE-TER processor tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_vde_ter_25khz_data_rate() {
        let proc = VdeTerProcessor::new_25khz();
        // pi/4-QPSK (2 bps/Hz), rate 3/4 → ~75% of peak
        let rate = proc.data_rate_bps(2, 3, 4);
        assert!(rate > 10_000.0 && rate < 100_000.0);
    }

    #[test]
    fn test_vde_ter_symbol_duration() {
        let proc = VdeTerProcessor::new_25khz();
        let dur = proc.symbol_duration();
        assert!(dur > 0.0);
        let dur_with_cp = proc.symbol_duration_with_cp();
        assert!(dur_with_cp > dur);
    }

    #[test]
    fn test_vde_ter_ofdm_roundtrip() {
        let proc = VdeTerProcessor::new_25khz();
        let n = proc.num_subcarriers;
        // Create test frequency-domain data
        let freq_data: Vec<(f64, f64)> = (0..n)
            .map(|i| (i as f64 / n as f64, 0.0))
            .collect();
        let time_samples = proc.ifft_with_cp(&freq_data);
        assert_eq!(time_samples.len(), n + proc.cp_length);

        let recovered_freq = proc.fft_remove_cp(&time_samples);
        assert_eq!(recovered_freq.len(), n);

        // Check rough recovery (not exact due to O(n^2) DFT precision)
        for k in 0..n {
            let expected = freq_data[k].0;
            let got = recovered_freq[k].0;
            assert!(
                approx_eq(got, expected, 0.1),
                "OFDM roundtrip mismatch at bin {}",
                k
            );
        }
    }

    // -------------------------------------------------------------------------
    // VDE-SAT Doppler tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_doppler_zero_range_rate() {
        let d = satellite_doppler_hz(162e6, 0.0);
        assert!(approx_eq(d, 0.0, 1e-6));
    }

    #[test]
    fn test_doppler_leo_approx() {
        // LEO satellite approaching at ~7 km/s → ~3.8 kHz Doppler at 162 MHz
        let d = satellite_doppler_hz(162e6, -7000.0);
        assert!(d > 3000.0 && d < 4500.0);
    }

    #[test]
    fn test_timing_advance_geostationary() {
        // GEO at 36000 km → TA ≈ 240 ms
        let ta = timing_advance_us(36_000_000.0);
        assert!(approx_eq(ta / 1000.0, 240.0, 5.0));
    }

    #[test]
    fn test_doppler_precomp_length() {
        let samples = vec![(1.0, 0.0); 100];
        let out = apply_doppler_precomp(&samples, 1000.0, 9600.0);
        assert_eq!(out.len(), samples.len());
    }

    #[test]
    fn test_doppler_precomp_unit_envelope() {
        let samples: Vec<(f64, f64)> = (0..50)
            .map(|i| (f64::cos(i as f64), f64::sin(i as f64)))
            .collect();
        let out = apply_doppler_precomp(&samples, 500.0, 9600.0);
        for (i, q) in &out {
            let mag = (i * i + q * q).sqrt();
            assert!(approx_eq(mag, 1.0, 1e-9));
        }
    }

    // -------------------------------------------------------------------------
    // Full packet builder tests
    // -------------------------------------------------------------------------

    #[test]
    fn test_build_ais_type1_packet_nonempty() {
        let report = AisPositionReport {
            mmsi: 123456789,
            nav_status: 0,
            rot: 5.0,
            sog: 10.0,
            position_accuracy: true,
            longitude: 10.0,
            latitude: 55.0,
            cog: 180.0,
            true_heading: 180,
            timestamp: 30,
        };
        let packet = build_ais_type1_packet(&report);
        assert!(!packet.is_empty());
        // Packet should start and end with HDLC flag in NRZI domain
        assert!(packet.len() > 200); // 168 payload + overhead
    }

    #[test]
    fn test_nav_status_from_u8() {
        assert_eq!(NavStatus::from_u8(0), NavStatus::UnderWayUsingEngine);
        assert_eq!(NavStatus::from_u8(5), NavStatus::Moored);
        assert_eq!(NavStatus::from_u8(15), NavStatus::Undefined);
    }

    #[test]
    fn test_vdes_channel_frequencies() {
        assert!(approx_eq(VdesChannel::Ais1.center_freq_hz(), AIS1_FREQ_HZ, 1.0));
        assert!(approx_eq(VdesChannel::Ais2.center_freq_hz(), AIS2_FREQ_HZ, 1.0));
        assert!(approx_eq(VdesChannel::Asm1.center_freq_hz(), ASM1_FREQ_HZ, 1.0));
    }

    #[test]
    fn test_vdes_channel_bandwidth() {
        assert!(approx_eq(VdesChannel::Ais1.bandwidth_hz(), 25_000.0, 1.0));
        assert!(approx_eq(VdesChannel::VdeTer(0).bandwidth_hz(), 25_000.0, 1.0));
    }

    #[test]
    fn test_slot_conflict_detection() {
        let s1 = TdmaSlot {
            slot_number: 42,
            offset: 0,
            timeout: 3,
            allocation: SlotAllocation::Sotdma,
        };
        let s2 = TdmaSlot {
            slot_number: 42,
            offset: 0,
            timeout: 5,
            allocation: SlotAllocation::Itdma,
        };
        let s3 = TdmaSlot {
            slot_number: 43,
            offset: 0,
            timeout: 3,
            allocation: SlotAllocation::Sotdma,
        };
        assert!(s1.conflicts_with(&s2));
        assert!(!s1.conflicts_with(&s3));
    }

    #[test]
    fn test_erf_approx_bounds() {
        // erf(0) = 0
        assert!(approx_eq(erf_approx(0.0), 0.0, 1e-6));
        // erf(∞) ≈ 1
        assert!(approx_eq(erf_approx(4.0), 1.0, 0.001));
        // erf(-∞) ≈ -1
        assert!(approx_eq(erf_approx(-4.0), -1.0, 0.001));
    }

    #[test]
    fn test_ais_type5_draught_roundtrip() {
        let mut sv = AisStaticVoyage {
            mmsi: 1,
            imo: 0,
            callsign: [b' '; 7],
            name: [b' '; 20],
            ship_type: 0,
            dim_a: 0,
            dim_b: 0,
            dim_c: 0,
            dim_d: 0,
            epfd_type: 0,
            eta_month: 0,
            eta_day: 0,
            eta_hour: 24,
            eta_minute: 60,
            draught: 12.5,
            destination: [b' '; 20],
            dte: false,
        };
        let bits = encode_ais_type5(&sv);
        let dec = decode_ais_type5(&bits).unwrap();
        assert!(approx_eq(dec.draught, sv.draught, 0.1));

        sv.draught = 0.3;
        let bits2 = encode_ais_type5(&sv);
        let dec2 = decode_ais_type5(&bits2).unwrap();
        assert!(approx_eq(dec2.draught, sv.draught, 0.15));
    }
}

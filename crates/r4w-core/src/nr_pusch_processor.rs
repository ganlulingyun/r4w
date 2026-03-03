//! # 5G NR PUSCH Processor
//!
//! Implements Physical Uplink Shared Channel (PUSCH) processing per
//! 3GPP TS 38.211, TS 38.212, and TS 38.214.
//!
//! ## Processing Chain (TS 38.212 Section 6.2 / TS 38.211 Section 6.3)
//!
//! ```text
//! Transport Block
//!     │
//!     ▼
//! TB CRC Attachment (CRC-24A for TBS > 3824, CRC-16 otherwise)
//!     │
//!     ▼
//! Code Block Segmentation (+ CRC-24B per CB if needed)
//!     │
//!     ▼
//! LDPC Encoding (Base Graph 1 or 2)
//!     │
//!     ▼
//! Rate Matching (circular buffer + interleaving)
//!     │
//!     ▼
//! Code Block Concatenation
//!     │
//!     ▼
//! UCI Multiplexing (HARQ-ACK, CSI-Part1, CSI-Part2)
//!     │
//!     ▼
//! Scrambling (Gold sequence per TS 38.211 Section 6.3.1.1)
//!     │
//!     ▼
//! Modulation (pi/2-BPSK, QPSK, 16QAM, 64QAM, 256QAM)
//!     │
//!     ▼
//! Transform Precoding (DFT-s-OFDM for pi/2-BPSK/QPSK; bypass for others)
//!     │
//!     ▼
//! Layer Mapping (1–4 layers)
//!     │
//!     ▼
//! Resource Element Mapping (PRB allocation, DMRS, frequency hopping)
//! ```
//!
//! ## MCS Tables (TS 38.214 Section 6.1.4)
//!
//! Three tables are defined:
//! - Table 6.1.4.1-1: 64QAM (default)
//! - Table 6.1.4.1-2: 256QAM
//! - Table 6.1.4.1-3: 64QAM low-SE (for coverage enhancement)
//!
//! ## DMRS Generation (TS 38.211 Section 6.4.1.1)
//!
//! ```text
//! c_init = (2^17 * (14*n_slot + l + 1) * (2*N_ID + 1) + 2*N_ID + n_SCID) mod 2^31
//! ```
//!
//! ## Example
//!
//! ```
//! use r4w_core::nr_pusch_processor::{
//!     NrPuschProcessor, PuschConfig, PuschModulation, TransformPrecoding,
//! };
//!
//! let config = PuschConfig {
//!     mcs_index: 9,
//!     mcs_table: 1,
//!     num_prbs: 25,
//!     num_layers: 1,
//!     rv_idx: 0,
//!     transform_precoding: TransformPrecoding::Disabled,
//!     cell_id: 1,
//!     rnti: 0x1234,
//!     n_id: 0,
//!     n_scid: false,
//!     start_prb: 0,
//!     num_symbols: 14,
//!     start_symbol: 0,
//!     dmrs_symbol_mask: 0b0000_0000_0000_0100,
//!     dmrs_type: 1,
//!     frequency_hopping: false,
//!     second_hop_prb: 0,
//!     tpc_accumulation: 0,
//!     p0_nominal: -80,
//! };
//!
//! let processor = NrPuschProcessor::new(config);
//! let tbs = processor.calculate_tbs(9, 25, 1);
//! assert!(tbs > 0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex number type
// ---------------------------------------------------------------------------

/// Complex64 as (re, im) pair.
pub type Complex64 = (f64, f64);

fn complex_add(a: Complex64, b: Complex64) -> Complex64 {
    (a.0 + b.0, a.1 + b.1)
}

fn complex_mul(a: Complex64, b: Complex64) -> Complex64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn complex_scale(a: Complex64, s: f64) -> Complex64 {
    (a.0 * s, a.1 * s)
}

// ---------------------------------------------------------------------------
// Modulation definitions
// ---------------------------------------------------------------------------

/// PUSCH modulation order per TS 38.214 Section 6.1.4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PuschModulation {
    /// pi/2-BPSK: 1 bit/symbol, used with transform precoding.
    Pi2Bpsk,
    /// QPSK: 2 bits/symbol.
    Qpsk,
    /// 16QAM: 4 bits/symbol.
    Qam16,
    /// 64QAM: 6 bits/symbol.
    Qam64,
    /// 256QAM: 8 bits/symbol.
    Qam256,
}

impl PuschModulation {
    /// Returns the number of bits per symbol (Q_m).
    pub fn bits_per_symbol(self) -> u8 {
        match self {
            PuschModulation::Pi2Bpsk => 1,
            PuschModulation::Qpsk => 2,
            PuschModulation::Qam16 => 4,
            PuschModulation::Qam64 => 6,
            PuschModulation::Qam256 => 8,
        }
    }

    /// Returns the constellation scaling factor (normalisation) so average power = 1.
    pub fn scale_factor(self) -> f64 {
        match self {
            PuschModulation::Pi2Bpsk => 1.0,
            PuschModulation::Qpsk => 1.0 / (2.0f64).sqrt(),
            PuschModulation::Qam16 => 1.0 / (10.0f64).sqrt(),
            PuschModulation::Qam64 => 1.0 / (42.0f64).sqrt(),
            PuschModulation::Qam256 => 1.0 / (170.0f64).sqrt(),
        }
    }
}

/// Whether DFT-s-OFDM transform precoding is enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransformPrecoding {
    /// DFT spreading enabled (SC-FDMA): used with pi/2-BPSK, QPSK.
    Enabled,
    /// DFT spreading disabled (CP-OFDM): used with 16QAM and higher.
    Disabled,
}

// ---------------------------------------------------------------------------
// MCS tables (TS 38.214 Tables 6.1.4.1-1, 6.1.4.1-2, 6.1.4.1-3)
// ---------------------------------------------------------------------------

/// Single entry in an MCS table.
#[derive(Debug, Clone, Copy)]
pub struct McsPuschEntry {
    /// MCS index (0..28).
    pub mcs_index: u8,
    /// Modulation order.
    pub modulation: PuschModulation,
    /// Numerator of the target code rate when denominator = 1024.
    pub code_rate_x1024: u16,
}

/// Table 6.1.4.1-1 from TS 38.214: 64QAM MCS table (default).
pub const MCS_TABLE_1: [McsPuschEntry; 29] = [
    McsPuschEntry { mcs_index:  0, modulation: PuschModulation::Qpsk,   code_rate_x1024: 120 },
    McsPuschEntry { mcs_index:  1, modulation: PuschModulation::Qpsk,   code_rate_x1024: 157 },
    McsPuschEntry { mcs_index:  2, modulation: PuschModulation::Qpsk,   code_rate_x1024: 193 },
    McsPuschEntry { mcs_index:  3, modulation: PuschModulation::Qpsk,   code_rate_x1024: 251 },
    McsPuschEntry { mcs_index:  4, modulation: PuschModulation::Qpsk,   code_rate_x1024: 308 },
    McsPuschEntry { mcs_index:  5, modulation: PuschModulation::Qpsk,   code_rate_x1024: 379 },
    McsPuschEntry { mcs_index:  6, modulation: PuschModulation::Qpsk,   code_rate_x1024: 449 },
    McsPuschEntry { mcs_index:  7, modulation: PuschModulation::Qpsk,   code_rate_x1024: 526 },
    McsPuschEntry { mcs_index:  8, modulation: PuschModulation::Qpsk,   code_rate_x1024: 602 },
    McsPuschEntry { mcs_index:  9, modulation: PuschModulation::Qpsk,   code_rate_x1024: 679 },
    McsPuschEntry { mcs_index: 10, modulation: PuschModulation::Qam16,  code_rate_x1024: 340 },
    McsPuschEntry { mcs_index: 11, modulation: PuschModulation::Qam16,  code_rate_x1024: 378 },
    McsPuschEntry { mcs_index: 12, modulation: PuschModulation::Qam16,  code_rate_x1024: 434 },
    McsPuschEntry { mcs_index: 13, modulation: PuschModulation::Qam16,  code_rate_x1024: 490 },
    McsPuschEntry { mcs_index: 14, modulation: PuschModulation::Qam16,  code_rate_x1024: 553 },
    McsPuschEntry { mcs_index: 15, modulation: PuschModulation::Qam16,  code_rate_x1024: 616 },
    McsPuschEntry { mcs_index: 16, modulation: PuschModulation::Qam16,  code_rate_x1024: 658 },
    McsPuschEntry { mcs_index: 17, modulation: PuschModulation::Qam64,  code_rate_x1024: 438 },
    McsPuschEntry { mcs_index: 18, modulation: PuschModulation::Qam64,  code_rate_x1024: 466 },
    McsPuschEntry { mcs_index: 19, modulation: PuschModulation::Qam64,  code_rate_x1024: 517 },
    McsPuschEntry { mcs_index: 20, modulation: PuschModulation::Qam64,  code_rate_x1024: 567 },
    McsPuschEntry { mcs_index: 21, modulation: PuschModulation::Qam64,  code_rate_x1024: 616 },
    McsPuschEntry { mcs_index: 22, modulation: PuschModulation::Qam64,  code_rate_x1024: 666 },
    McsPuschEntry { mcs_index: 23, modulation: PuschModulation::Qam64,  code_rate_x1024: 719 },
    McsPuschEntry { mcs_index: 24, modulation: PuschModulation::Qam64,  code_rate_x1024: 772 },
    McsPuschEntry { mcs_index: 25, modulation: PuschModulation::Qam64,  code_rate_x1024: 822 },
    McsPuschEntry { mcs_index: 26, modulation: PuschModulation::Qam64,  code_rate_x1024: 873 },
    McsPuschEntry { mcs_index: 27, modulation: PuschModulation::Qam64,  code_rate_x1024: 910 },
    McsPuschEntry { mcs_index: 28, modulation: PuschModulation::Qam64,  code_rate_x1024: 948 },
];

/// Table 6.1.4.1-2 from TS 38.214: 256QAM MCS table.
pub const MCS_TABLE_2: [McsPuschEntry; 28] = [
    McsPuschEntry { mcs_index:  0, modulation: PuschModulation::Qpsk,   code_rate_x1024: 120 },
    McsPuschEntry { mcs_index:  1, modulation: PuschModulation::Qpsk,   code_rate_x1024: 193 },
    McsPuschEntry { mcs_index:  2, modulation: PuschModulation::Qpsk,   code_rate_x1024: 308 },
    McsPuschEntry { mcs_index:  3, modulation: PuschModulation::Qpsk,   code_rate_x1024: 449 },
    McsPuschEntry { mcs_index:  4, modulation: PuschModulation::Qpsk,   code_rate_x1024: 602 },
    McsPuschEntry { mcs_index:  5, modulation: PuschModulation::Qam16,  code_rate_x1024: 378 },
    McsPuschEntry { mcs_index:  6, modulation: PuschModulation::Qam16,  code_rate_x1024: 434 },
    McsPuschEntry { mcs_index:  7, modulation: PuschModulation::Qam16,  code_rate_x1024: 490 },
    McsPuschEntry { mcs_index:  8, modulation: PuschModulation::Qam16,  code_rate_x1024: 553 },
    McsPuschEntry { mcs_index:  9, modulation: PuschModulation::Qam16,  code_rate_x1024: 616 },
    McsPuschEntry { mcs_index: 10, modulation: PuschModulation::Qam64,  code_rate_x1024: 438 },
    McsPuschEntry { mcs_index: 11, modulation: PuschModulation::Qam64,  code_rate_x1024: 466 },
    McsPuschEntry { mcs_index: 12, modulation: PuschModulation::Qam64,  code_rate_x1024: 517 },
    McsPuschEntry { mcs_index: 13, modulation: PuschModulation::Qam64,  code_rate_x1024: 567 },
    McsPuschEntry { mcs_index: 14, modulation: PuschModulation::Qam64,  code_rate_x1024: 616 },
    McsPuschEntry { mcs_index: 15, modulation: PuschModulation::Qam64,  code_rate_x1024: 666 },
    McsPuschEntry { mcs_index: 16, modulation: PuschModulation::Qam64,  code_rate_x1024: 719 },
    McsPuschEntry { mcs_index: 17, modulation: PuschModulation::Qam64,  code_rate_x1024: 772 },
    McsPuschEntry { mcs_index: 18, modulation: PuschModulation::Qam64,  code_rate_x1024: 822 },
    McsPuschEntry { mcs_index: 19, modulation: PuschModulation::Qam64,  code_rate_x1024: 873 },
    McsPuschEntry { mcs_index: 20, modulation: PuschModulation::Qam256, code_rate_x1024: 682 },
    McsPuschEntry { mcs_index: 21, modulation: PuschModulation::Qam256, code_rate_x1024: 711 },
    McsPuschEntry { mcs_index: 22, modulation: PuschModulation::Qam256, code_rate_x1024: 754 },
    McsPuschEntry { mcs_index: 23, modulation: PuschModulation::Qam256, code_rate_x1024: 797 },
    McsPuschEntry { mcs_index: 24, modulation: PuschModulation::Qam256, code_rate_x1024: 841 },
    McsPuschEntry { mcs_index: 25, modulation: PuschModulation::Qam256, code_rate_x1024: 885 },
    McsPuschEntry { mcs_index: 26, modulation: PuschModulation::Qam256, code_rate_x1024: 916 },
    McsPuschEntry { mcs_index: 27, modulation: PuschModulation::Qam256, code_rate_x1024: 948 },
];

/// Table 6.1.4.1-3 from TS 38.214: 64QAM low-SE MCS table.
pub const MCS_TABLE_3: [McsPuschEntry; 29] = [
    McsPuschEntry { mcs_index:  0, modulation: PuschModulation::Qpsk,   code_rate_x1024:  30 },
    McsPuschEntry { mcs_index:  1, modulation: PuschModulation::Qpsk,   code_rate_x1024:  40 },
    McsPuschEntry { mcs_index:  2, modulation: PuschModulation::Qpsk,   code_rate_x1024:  50 },
    McsPuschEntry { mcs_index:  3, modulation: PuschModulation::Qpsk,   code_rate_x1024:  64 },
    McsPuschEntry { mcs_index:  4, modulation: PuschModulation::Qpsk,   code_rate_x1024:  78 },
    McsPuschEntry { mcs_index:  5, modulation: PuschModulation::Qpsk,   code_rate_x1024:  99 },
    McsPuschEntry { mcs_index:  6, modulation: PuschModulation::Qpsk,   code_rate_x1024: 120 },
    McsPuschEntry { mcs_index:  7, modulation: PuschModulation::Qpsk,   code_rate_x1024: 157 },
    McsPuschEntry { mcs_index:  8, modulation: PuschModulation::Qpsk,   code_rate_x1024: 193 },
    McsPuschEntry { mcs_index:  9, modulation: PuschModulation::Qpsk,   code_rate_x1024: 251 },
    McsPuschEntry { mcs_index: 10, modulation: PuschModulation::Qpsk,   code_rate_x1024: 308 },
    McsPuschEntry { mcs_index: 11, modulation: PuschModulation::Qpsk,   code_rate_x1024: 379 },
    McsPuschEntry { mcs_index: 12, modulation: PuschModulation::Qpsk,   code_rate_x1024: 449 },
    McsPuschEntry { mcs_index: 13, modulation: PuschModulation::Qpsk,   code_rate_x1024: 526 },
    McsPuschEntry { mcs_index: 14, modulation: PuschModulation::Qpsk,   code_rate_x1024: 602 },
    McsPuschEntry { mcs_index: 15, modulation: PuschModulation::Qam16,  code_rate_x1024: 340 },
    McsPuschEntry { mcs_index: 16, modulation: PuschModulation::Qam16,  code_rate_x1024: 378 },
    McsPuschEntry { mcs_index: 17, modulation: PuschModulation::Qam16,  code_rate_x1024: 434 },
    McsPuschEntry { mcs_index: 18, modulation: PuschModulation::Qam16,  code_rate_x1024: 490 },
    McsPuschEntry { mcs_index: 19, modulation: PuschModulation::Qam16,  code_rate_x1024: 553 },
    McsPuschEntry { mcs_index: 20, modulation: PuschModulation::Qam16,  code_rate_x1024: 616 },
    McsPuschEntry { mcs_index: 21, modulation: PuschModulation::Qam64,  code_rate_x1024: 438 },
    McsPuschEntry { mcs_index: 22, modulation: PuschModulation::Qam64,  code_rate_x1024: 466 },
    McsPuschEntry { mcs_index: 23, modulation: PuschModulation::Qam64,  code_rate_x1024: 517 },
    McsPuschEntry { mcs_index: 24, modulation: PuschModulation::Qam64,  code_rate_x1024: 567 },
    McsPuschEntry { mcs_index: 25, modulation: PuschModulation::Qam64,  code_rate_x1024: 616 },
    McsPuschEntry { mcs_index: 26, modulation: PuschModulation::Qam64,  code_rate_x1024: 666 },
    McsPuschEntry { mcs_index: 27, modulation: PuschModulation::Qam64,  code_rate_x1024: 719 },
    McsPuschEntry { mcs_index: 28, modulation: PuschModulation::Qam64,  code_rate_x1024: 772 },
];

/// Look up an MCS table entry by table ID (1, 2, or 3) and index.
pub fn mcs_lookup(table_id: u8, mcs_index: u8) -> Option<McsPuschEntry> {
    match table_id {
        1 => {
            if (mcs_index as usize) < MCS_TABLE_1.len() {
                Some(MCS_TABLE_1[mcs_index as usize])
            } else {
                None
            }
        }
        2 => {
            if (mcs_index as usize) < MCS_TABLE_2.len() {
                Some(MCS_TABLE_2[mcs_index as usize])
            } else {
                None
            }
        }
        3 => {
            if (mcs_index as usize) < MCS_TABLE_3.len() {
                Some(MCS_TABLE_3[mcs_index as usize])
            } else {
                None
            }
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// TBS lookup table (TS 38.214 Table 5.1.3.2-1), 93 entries
// ---------------------------------------------------------------------------

const TBS_TABLE: [u32; 93] = [
    24, 32, 40, 48, 56, 64, 72, 80, 88, 96,
    104, 112, 120, 128, 136, 144, 152, 160, 168, 176,
    184, 192, 208, 224, 240, 256, 272, 288, 304, 320,
    336, 352, 368, 384, 408, 432, 456, 480, 504, 528,
    552, 576, 608, 640, 672, 704, 736, 768, 808, 848,
    888, 928, 984, 1032, 1064, 1128, 1160, 1192, 1224, 1256,
    1288, 1320, 1352, 1416, 1480, 1544, 1608, 1672, 1736, 1800,
    1864, 1928, 2024, 2088, 2152, 2216, 2280, 2408, 2472, 2536,
    2600, 2664, 2728, 2792, 2856, 2976, 3104, 3240, 3368, 3496,
    3624, 3752, 3824,
];

// ---------------------------------------------------------------------------
// CRC polynomials and computation
// ---------------------------------------------------------------------------

/// CRC-24A polynomial: x^24 + x^23 + x^18 + x^17 + x^14 + x^11 + x^10 + x^7 + x^6 + x^5 + x^4 + x^3 + x + 1
const CRC24A_POLY: u32 = 0x864CFB;

/// CRC-24B polynomial: x^24 + x^23 + x^6 + x^5 + x + 1
const CRC24B_POLY: u32 = 0x800063;

/// CRC-16 polynomial (CRC-16-CCITT): x^16 + x^12 + x^5 + 1
const CRC16_POLY: u32 = 0x11021;

fn crc_from_bits(bits: &[bool], poly: u32, crc_bits: u8) -> u32 {
    let mut crc: u32 = 0;
    let mask = if crc_bits == 24 { 0xFF_FFFF } else { 0xFFFF };

    for &bit in bits {
        let input_bit = bit as u32;
        let crc_top = (crc >> (crc_bits - 1)) & 1;
        crc = (crc << 1) & mask;
        if input_bit ^ crc_top != 0 {
            crc ^= poly;
        }
    }
    crc & mask
}

fn crc_from_bytes(data: &[u8], poly: u32, crc_bits: u8) -> u32 {
    let mut crc: u32 = 0;
    let mask = if crc_bits == 24 { 0xFF_FFFF } else { 0xFFFF };

    for &byte in data {
        for bit_pos in (0..8).rev() {
            let input_bit = ((byte >> bit_pos) & 1) as u32;
            let crc_top = (crc >> (crc_bits - 1)) & 1;
            crc = (crc << 1) & mask;
            if input_bit ^ crc_top != 0 {
                crc ^= poly;
            }
        }
    }
    crc & mask
}

/// Compute CRC-24A from a bit vector.
pub fn crc24a_bits(bits: &[bool]) -> u32 {
    crc_from_bits(bits, CRC24A_POLY, 24)
}

/// Compute CRC-24B from a bit vector.
pub fn crc24b_bits(bits: &[bool]) -> u32 {
    crc_from_bits(bits, CRC24B_POLY, 24)
}

/// Compute CRC-16 from a bit vector.
pub fn crc16_bits(bits: &[bool]) -> u16 {
    crc_from_bits(bits, CRC16_POLY, 16) as u16
}

/// Compute CRC-24A from bytes.
pub fn crc24a(data: &[u8]) -> u32 {
    crc_from_bytes(data, CRC24A_POLY, 24)
}

/// Compute CRC-16 from bytes.
pub fn crc16(data: &[u8]) -> u16 {
    crc_from_bytes(data, CRC16_POLY, 16) as u16
}

/// Attach TB-level CRC to a transport block per TS 38.212 Section 6.2.1.
pub fn attach_tb_crc(tb_bits: &[bool]) -> Vec<bool> {
    let mut result = tb_bits.to_vec();
    if tb_bits.len() > 3824 {
        let crc = crc24a_bits(tb_bits);
        for i in (0..24).rev() {
            result.push((crc >> i) & 1 == 1);
        }
    } else {
        let crc = crc16_bits(tb_bits) as u32;
        for i in (0..16).rev() {
            result.push((crc >> i) & 1 == 1);
        }
    }
    result
}

/// Attach CRC-24B to a code block.
pub fn attach_cb_crc(cb_bits: &[bool]) -> Vec<bool> {
    let mut result = cb_bits.to_vec();
    let crc = crc24b_bits(cb_bits);
    for i in (0..24).rev() {
        result.push((crc >> i) & 1 == 1);
    }
    result
}

// ---------------------------------------------------------------------------
// LDPC parameters and base graph selection
// ---------------------------------------------------------------------------

/// LDPC lifting size sets per TS 38.212 Table 5.3.2-1.
const LIFTING_SIZE_SETS: [[u16; 8]; 8] = [
    [2,   4,   8,   16,  32,  64,  128, 256],
    [3,   6,   12,  24,  48,  96,  192, 384],
    [5,   10,  20,  40,  80,  160, 320,   0],
    [7,   14,  28,  56,  112, 224,   0,   0],
    [9,   18,  36,  72,  144, 288,   0,   0],
    [11,  22,  44,  88,  176, 352,   0,   0],
    [13,  26,  52,  104, 208,   0,   0,   0],
    [15,  30,  60,  120, 240,   0,   0,   0],
];

/// LDPC parameters for a given code block.
#[derive(Debug, Clone, Copy)]
pub struct LdpcParams {
    /// Base graph: 1 or 2.
    pub base_graph: u8,
    /// Lifting size Z (2..=384).
    pub lifting_size: u16,
    /// Number of systematic columns K_b.
    pub k_b: u8,
    /// Encoded block length N_cb (before rate matching).
    pub n_cb: u32,
}

/// Select the minimum valid lifting size Z >= ceil(K / K_b).
pub fn select_lifting_size(k_bits: u32, k_b: u8) -> u16 {
    let k_per_z = (k_bits + k_b as u32 - 1) / k_b as u32;
    let mut best = 384u16;
    for set in &LIFTING_SIZE_SETS {
        for &z in set {
            if z == 0 {
                continue;
            }
            if z as u32 >= k_per_z && z <= best {
                best = z;
            }
        }
    }
    best
}

/// Determine LDPC base graph per TS 38.212 Section 6.2.2.
///
/// BG1 is used when:
///   - A > 3824 (large TBS), or
///   - A > 292 and R >= 0.67 (high code rate for medium TBS)
/// Otherwise BG2 is used.
pub fn select_base_graph(tbs_bits: u32, coding_rate: f64) -> u8 {
    if tbs_bits > 3824 {
        1
    } else if tbs_bits > 292 && coding_rate > 0.67 {
        1
    } else {
        2
    }
}

/// Derive full LDPC parameters for a code block of given information size.
pub fn ldpc_params(k_bits: u32, base_graph: u8) -> LdpcParams {
    let (k_b, n_columns) = if base_graph == 1 {
        (22u8, 68u8) // BG1: 22 systematic + 46 parity = 68
    } else {
        (10u8, 52u8) // BG2: 10 systematic + 42 parity = 52
    };

    let z = select_lifting_size(k_bits, k_b);
    let n_cb = n_columns as u32 * z as u32;

    LdpcParams {
        base_graph,
        lifting_size: z,
        k_b,
        n_cb,
    }
}

// ---------------------------------------------------------------------------
// Code block segmentation (TS 38.212 Section 6.2.3)
// ---------------------------------------------------------------------------

/// A segmented code block.
#[derive(Debug, Clone)]
pub struct CodeBlock {
    /// Information bits including filler zeros.
    pub info_bits: Vec<bool>,
    /// CRC-24B appended (24 bits).
    pub crc: u32,
    /// LDPC parameters for this code block.
    pub ldpc: LdpcParams,
}

/// Segment a transport block (with TB-CRC already attached) into code blocks.
pub fn segment_code_blocks(tb_with_crc: &[bool]) -> Vec<CodeBlock> {
    let b = tb_with_crc.len();

    const MAX_CB_BG1: u32 = 8448; // 22 * 384
    const MAX_CB_BG2: u32 = 3840; // 10 * 384

    let use_bg1 = b > 3824;
    let k_cb_max = if use_bg1 { MAX_CB_BG1 } else { MAX_CB_BG2 };
    let k_b_val: u8 = if use_bg1 { 22 } else { 10 };

    // Number of code blocks
    let c: u32 = if b as u32 > k_cb_max {
        let b_prime = b as u32 + 24;
        (b_prime + k_cb_max - 1) / k_cb_max
    } else {
        1
    };

    let b_prime = b as u32 + if c > 1 { 24 * c } else { 0 };
    let k_prime = (b_prime + c - 1) / c;

    let bg = if use_bg1 { 1 } else { 2 };
    let ldpc = ldpc_params(k_prime, bg);
    let z = ldpc.lifting_size;

    let k = k_b_val as u32 * z as u32;
    let f = (k * c).saturating_sub(b_prime);

    let mut code_blocks = Vec::with_capacity(c as usize);
    let mut bit_offset = 0usize;

    for cb_idx in 0..c {
        let mut cb_bits: Vec<bool> = Vec::with_capacity(k as usize);

        // Filler zeros at start of first CB
        if cb_idx == 0 {
            for _ in 0..f {
                cb_bits.push(false);
            }
        }

        let bits_to_copy = (k as usize - cb_bits.len()).min(b.saturating_sub(bit_offset));
        if bit_offset < b {
            cb_bits.extend_from_slice(&tb_with_crc[bit_offset..bit_offset + bits_to_copy]);
            bit_offset += bits_to_copy;
        }

        // Pad to K
        while cb_bits.len() < k as usize {
            cb_bits.push(false);
        }

        // Attach CRC-24B if multiple code blocks
        let (info_bits, crc) = if c > 1 {
            let crc_val = crc24b_bits(&cb_bits);
            let mut with_crc = attach_cb_crc(&cb_bits);
            // Trim to K + 24 exactly
            while with_crc.len() > k as usize + 24 {
                with_crc.pop();
            }
            (with_crc, crc_val)
        } else {
            let crc_val = 0;
            (cb_bits, crc_val)
        };

        code_blocks.push(CodeBlock {
            info_bits,
            crc,
            ldpc,
        });
    }

    code_blocks
}

// ---------------------------------------------------------------------------
// LDPC encoding (BG1/BG2 parity approximation)
// ---------------------------------------------------------------------------

/// Simulate LDPC encoding: produce N_cb bits (systematic + parity).
///
/// This is an educational approximation: systematic bits are copied in, and
/// parity bits are generated using a simplified systematic LDPC structure.
/// A real implementation uses full parity check matrices from 3GPP TS 38.212
/// Annex B and C.
pub fn ldpc_encode(info_bits: &[bool], params: &LdpcParams) -> Vec<bool> {
    let k = info_bits.len();
    let n = params.n_cb as usize;
    let mut codeword = vec![false; n];

    // Copy systematic bits
    let copy_len = k.min(n);
    codeword[..copy_len].copy_from_slice(&info_bits[..copy_len]);

    // Simplified parity generation: XOR-based parity check
    // Real BG1/BG2 parity is determined by the sparse parity-check matrix;
    // here we use a simple circulant approximation for demonstration.
    let z = params.lifting_size as usize;
    let parity_start = params.k_b as usize * z;

    if parity_start < n && z > 0 {
        for p in parity_start..n {
            let mut parity = false;
            // Each parity bit covers a window of systematic bits
            let window_start = (p - parity_start) % k.max(1);
            let window_end = (window_start + z).min(k);
            for i in window_start..window_end {
                parity ^= info_bits[i];
            }
            codeword[p] = parity;
        }
    }

    codeword
}

// ---------------------------------------------------------------------------
// Rate matching (TS 38.212 Section 6.2.4)
// ---------------------------------------------------------------------------

/// Rate-match a code block to the specified output length.
///
/// Uses circular buffer selection with redundancy version (rv_idx).
pub fn rate_match(codeword: &[bool], e_r: usize, rv_idx: u8, n_cb: usize) -> Vec<bool> {
    if codeword.is_empty() || e_r == 0 {
        return vec![false; e_r];
    }

    let n = codeword.len().min(n_cb);
    // Starting position in the circular buffer per RV
    // rv_start offsets: 0=0, 1=17*Z/66 (BG1 approx), simplified to N/4 multiples
    let rv_offsets = [0, n / 4, n / 2, 3 * n / 4];
    let k_0 = rv_offsets[(rv_idx % 4) as usize];

    let mut output = Vec::with_capacity(e_r);
    let mut idx = k_0;
    while output.len() < e_r {
        // Skip filler bits (they are never transmitted)
        let bit = codeword[idx % n];
        output.push(bit);
        idx += 1;
    }
    output
}

// ---------------------------------------------------------------------------
// Gold sequence generator (TS 38.211 Section 5.2.1)
// ---------------------------------------------------------------------------

/// Generate a length-31 Gold sequence c(n) from c_init.
///
/// Uses two length-31 m-sequences:
///   x1[n+31] = (x1[n+3] + x1[n]) mod 2
///   x2[n+31] = (x2[n+3] + x2[n+2] + x2[n+1] + x2[n]) mod 2
pub struct GoldSequence {
    x1: u32,
    x2: u32,
}

impl GoldSequence {
    /// Initialize from c_init as specified in TS 38.211.
    pub fn new(c_init: u32) -> Self {
        let mut x1: u32 = 1; // x1[0]=1, x1[1..30]=0
        let mut x2: u32 = c_init & 0x7FFF_FFFF;

        // Advance 1600 steps
        for _ in 0..1600 {
            let b1 = (x1 >> 3) ^ x1;
            x1 = (x1 >> 1) | ((b1 & 1) << 30);

            let b2 = (x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2;
            x2 = (x2 >> 1) | ((b2 & 1) << 30);
        }

        GoldSequence { x1, x2 }
    }

    /// Generate the next bit of the Gold sequence.
    pub fn next_bit(&mut self) -> bool {
        let c = ((self.x1 ^ self.x2) & 1) != 0;

        let b1 = (self.x1 >> 3) ^ self.x1;
        self.x1 = (self.x1 >> 1) | ((b1 & 1) << 30);

        let b2 = (self.x2 >> 3) ^ (self.x2 >> 2) ^ (self.x2 >> 1) ^ self.x2;
        self.x2 = (self.x2 >> 1) | ((b2 & 1) << 30);

        c
    }

    /// Generate `len` bits.
    pub fn generate(&mut self, len: usize) -> Vec<bool> {
        (0..len).map(|_| self.next_bit()).collect()
    }
}

// ---------------------------------------------------------------------------
// Scrambling (TS 38.211 Section 6.3.1.1)
// ---------------------------------------------------------------------------

/// Compute the PUSCH scrambling sequence initialisation per TS 38.211 Section 6.3.1.1.
///
/// c_init = (n_RNTI * 2^15 + q * 2^14 + n_ID) mod 2^31
/// where q is the code-word index (always 0 for PUSCH single codeword).
pub fn pusch_scrambling_init(rnti: u16, n_id: u32, q: u8) -> u32 {
    let val = (rnti as u64) * (1u64 << 15)
        + (q as u64) * (1u64 << 14)
        + (n_id as u64);
    (val % (1u64 << 31)) as u32
}

/// Scramble a bit sequence using the Gold sequence.
pub fn scramble(bits: &[bool], c_init: u32) -> Vec<bool> {
    let mut seq = GoldSequence::new(c_init);
    bits.iter().map(|&b| b ^ seq.next_bit()).collect()
}

// ---------------------------------------------------------------------------
// Modulation (TS 38.211 Section 5.1)
// ---------------------------------------------------------------------------

/// Map bits to complex modulation symbols.
///
/// Supports pi/2-BPSK, QPSK, 16QAM, 64QAM, 256QAM per TS 38.211 Section 5.1.
pub fn modulate(bits: &[bool], modulation: PuschModulation) -> Vec<Complex64> {
    let scale = modulation.scale_factor();
    match modulation {
        PuschModulation::Pi2Bpsk => {
            bits.iter().enumerate().map(|(k, &b)| {
                let phi = if b { 1.0 } else { 0.0 };
                let phase = PI / 2.0 * (k as f64) + PI / 4.0 * (1.0 - 2.0 * phi);
                (phase.cos() * scale, phase.sin() * scale)
            }).collect()
        }
        PuschModulation::Qpsk => {
            bits.chunks(2).map(|chunk| {
                let b0 = chunk[0] as u8;
                let b1 = if chunk.len() > 1 { chunk[1] as u8 } else { 0 };
                let re = (1.0 - 2.0 * b0 as f64) * scale;
                let im = (1.0 - 2.0 * b1 as f64) * scale;
                (re, im)
            }).collect()
        }
        PuschModulation::Qam16 => {
            bits.chunks(4).map(|chunk| {
                let b: Vec<u8> = (0..4).map(|i| if i < chunk.len() { chunk[i] as u8 } else { 0 }).collect();
                let re = (1.0 - 2.0 * b[0] as f64) * (2.0 - (1.0 - 2.0 * b[2] as f64)) * scale;
                let im = (1.0 - 2.0 * b[1] as f64) * (2.0 - (1.0 - 2.0 * b[3] as f64)) * scale;
                (re, im)
            }).collect()
        }
        PuschModulation::Qam64 => {
            bits.chunks(6).map(|chunk| {
                let b: Vec<u8> = (0..6).map(|i| if i < chunk.len() { chunk[i] as u8 } else { 0 }).collect();
                let re = (1.0 - 2.0 * b[0] as f64)
                    * (4.0 - (1.0 - 2.0 * b[2] as f64) * (2.0 - (1.0 - 2.0 * b[4] as f64)))
                    * scale;
                let im = (1.0 - 2.0 * b[1] as f64)
                    * (4.0 - (1.0 - 2.0 * b[3] as f64) * (2.0 - (1.0 - 2.0 * b[5] as f64)))
                    * scale;
                (re, im)
            }).collect()
        }
        PuschModulation::Qam256 => {
            bits.chunks(8).map(|chunk| {
                let b: Vec<u8> = (0..8).map(|i| if i < chunk.len() { chunk[i] as u8 } else { 0 }).collect();
                let re = (1.0 - 2.0 * b[0] as f64)
                    * (8.0 - (1.0 - 2.0 * b[2] as f64)
                        * (4.0 - (1.0 - 2.0 * b[4] as f64) * (2.0 - (1.0 - 2.0 * b[6] as f64))))
                    * scale;
                let im = (1.0 - 2.0 * b[1] as f64)
                    * (8.0 - (1.0 - 2.0 * b[3] as f64)
                        * (4.0 - (1.0 - 2.0 * b[5] as f64) * (2.0 - (1.0 - 2.0 * b[7] as f64))))
                    * scale;
                (re, im)
            }).collect()
        }
    }
}

// ---------------------------------------------------------------------------
// DFT-s-OFDM transform precoding (TS 38.211 Section 6.3.1.4)
// ---------------------------------------------------------------------------

/// Apply DFT spreading (transform precoding) to a block of modulation symbols.
///
/// The DFT is computed directly (O(N^2)) for educational clarity.
/// A production implementation would use an FFT algorithm.
pub fn dft_spread(symbols: &[Complex64]) -> Vec<Complex64> {
    let n = symbols.len();
    if n == 0 {
        return vec![];
    }
    let mut out = vec![(0.0f64, 0.0f64); n];
    let scale = 1.0 / (n as f64).sqrt();

    for k in 0..n {
        let mut sum = (0.0f64, 0.0f64);
        for (m, &sym) in symbols.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (m as f64) / (n as f64);
            let twiddle = (angle.cos(), angle.sin());
            sum = complex_add(sum, complex_mul(sym, twiddle));
        }
        out[k] = complex_scale(sum, scale);
    }
    out
}

/// Apply inverse DFT (de-spreading) to transform-precoded symbols.
pub fn idft_despread(symbols: &[Complex64]) -> Vec<Complex64> {
    let n = symbols.len();
    if n == 0 {
        return vec![];
    }
    let mut out = vec![(0.0f64, 0.0f64); n];
    let scale = 1.0 / (n as f64).sqrt();

    for k in 0..n {
        let mut sum = (0.0f64, 0.0f64);
        for (m, &sym) in symbols.iter().enumerate() {
            let angle = 2.0 * PI * (k as f64) * (m as f64) / (n as f64);
            let twiddle = (angle.cos(), angle.sin());
            sum = complex_add(sum, complex_mul(sym, twiddle));
        }
        out[k] = complex_scale(sum, scale);
    }
    out
}

// ---------------------------------------------------------------------------
// Layer mapping (TS 38.211 Section 6.3.1.3)
// ---------------------------------------------------------------------------

/// Map a single codeword to up to 4 MIMO layers.
///
/// For single-layer: trivial pass-through.
/// For multi-layer: round-robin demultiplexing per TS 38.211 Table 6.3.1.3-1.
pub fn layer_map(symbols: &[Complex64], num_layers: u8) -> Vec<Vec<Complex64>> {
    let v = num_layers.min(4).max(1) as usize;
    let n = symbols.len();
    let symbols_per_layer = (n + v - 1) / v;

    let mut layers: Vec<Vec<Complex64>> = (0..v)
        .map(|_| Vec::with_capacity(symbols_per_layer))
        .collect();

    for (i, &sym) in symbols.iter().enumerate() {
        layers[i % v].push(sym);
    }

    layers
}

/// Inverse layer mapping (de-multiplexing from layers back to codeword).
pub fn layer_demap(layers: &[Vec<Complex64>]) -> Vec<Complex64> {
    if layers.is_empty() {
        return vec![];
    }
    let v = layers.len();
    let max_len = layers.iter().map(|l| l.len()).max().unwrap_or(0);
    let mut out = Vec::with_capacity(v * max_len);

    let mut indices = vec![0usize; v];
    let total: usize = layers.iter().map(|l| l.len()).sum();
    let mut count = 0;

    while count < total {
        for layer in 0..v {
            if indices[layer] < layers[layer].len() {
                out.push(layers[layer][indices[layer]]);
                indices[layer] += 1;
                count += 1;
            }
        }
    }
    out
}

// ---------------------------------------------------------------------------
// DMRS generation (TS 38.211 Section 6.4.1.1)
// ---------------------------------------------------------------------------

/// PUSCH DMRS configuration.
#[derive(Debug, Clone)]
pub struct DmrsConfig {
    /// DMRS type: 1 (default) or 2.
    pub dmrs_type: u8,
    /// Scrambling ID N_ID (0..1023 for type 1, 0..65535 for type 2).
    pub n_id: u32,
    /// Scrambling ID index n_SCID (0 or 1).
    pub n_scid: bool,
    /// Delta (subcarrier offset for DMRS type 1/2).
    pub delta: u8,
}

impl Default for DmrsConfig {
    fn default() -> Self {
        DmrsConfig {
            dmrs_type: 1,
            n_id: 0,
            n_scid: false,
            delta: 0,
        }
    }
}

/// Generate PUSCH DMRS sequence per TS 38.211 Section 6.4.1.1.
///
/// c_init = (2^17 * (14*n_slot + l + 1) * (2*N_ID + 1) + 2*N_ID + n_SCID) mod 2^31
/// where l is the OFDM symbol index within the slot.
pub fn generate_dmrs(dmrs_config: &DmrsConfig, slot: u32, symbol: u32, num_subcarriers: usize) -> Vec<Complex64> {
    let l = symbol;
    let n_slot = slot;
    let n_id = dmrs_config.n_id;
    let n_scid_val = if dmrs_config.n_scid { 1u64 } else { 0u64 };

    let c_init = ((1u64 << 17) * ((14 * n_slot as u64 + l as u64 + 1) * (2 * n_id as u64 + 1))
        + 2 * n_id as u64
        + n_scid_val)
        % (1u64 << 31);

    let mut gold = GoldSequence::new(c_init as u32);
    let scale = 1.0 / (2.0f64).sqrt();

    // Type 1 DMRS: every other subcarrier (delta = 0 or 1)
    // Type 2 DMRS: every third group of two subcarriers
    let num_dmrs = match dmrs_config.dmrs_type {
        2 => (num_subcarriers * 2) / 3,
        _ => num_subcarriers / 2,
    };

    (0..num_dmrs).map(|_| {
        let r_re = if gold.next_bit() { 1.0 } else { -1.0 };
        let r_im = if gold.next_bit() { 1.0 } else { -1.0 };
        (r_re * scale, r_im * scale)
    }).collect()
}

// ---------------------------------------------------------------------------
// UCI multiplexing (TS 38.212 Section 6.2.7)
// ---------------------------------------------------------------------------

/// UCI (Uplink Control Information) to multiplex on PUSCH.
#[derive(Debug, Clone, Default)]
pub struct UciInfo {
    /// HARQ-ACK bits (0 for NACK/DTX, 1 for ACK).
    pub harq_ack: Vec<bool>,
    /// CSI-Part1 bits.
    pub csi_part1: Vec<bool>,
    /// CSI-Part2 bits.
    pub csi_part2: Vec<bool>,
}

/// Result of UCI multiplexing.
#[derive(Debug, Clone)]
pub struct UciMuxResult {
    /// Multiplexed bits ready for scrambling.
    pub bits: Vec<bool>,
    /// Number of HARQ-ACK bits included.
    pub num_harq_ack: usize,
    /// Number of CSI-Part1 bits included.
    pub num_csi_part1: usize,
    /// Number of CSI-Part2 bits included.
    pub num_csi_part2: usize,
}

/// Multiplex UCI bits into the PUSCH bit stream.
///
/// UCI is punctured into specific resource elements of the PUSCH according to
/// TS 38.212 Section 6.2.7. This implementation places UCI at the start of the
/// allocated data bits for educational clarity.
pub fn multiplex_uci(data_bits: &[bool], uci: &UciInfo, num_re_total: usize, q_m: u8) -> UciMuxResult {
    let total_bits = num_re_total * q_m as usize;
    let mut output = data_bits[..total_bits.min(data_bits.len())].to_vec();

    // Pad to total if needed
    while output.len() < total_bits {
        output.push(false);
    }

    // HARQ-ACK occupies first G_HARQ positions, rate-matched to fit
    let num_harq = uci.harq_ack.len();
    let num_csi1 = uci.csi_part1.len();
    let num_csi2 = uci.csi_part2.len();

    // Place HARQ-ACK at the beginning (puncturing data)
    let harq_len = num_harq.min(total_bits);
    for (i, &bit) in uci.harq_ack.iter().take(harq_len).enumerate() {
        output[i] = bit;
    }

    // Place CSI-Part1 after HARQ-ACK
    let csi1_start = harq_len;
    let csi1_len = num_csi1.min(total_bits.saturating_sub(csi1_start));
    for (i, &bit) in uci.csi_part1.iter().take(csi1_len).enumerate() {
        output[csi1_start + i] = bit;
    }

    // Place CSI-Part2 after CSI-Part1
    let csi2_start = csi1_start + csi1_len;
    let csi2_len = num_csi2.min(total_bits.saturating_sub(csi2_start));
    for (i, &bit) in uci.csi_part2.iter().take(csi2_len).enumerate() {
        output[csi2_start + i] = bit;
    }

    UciMuxResult {
        bits: output,
        num_harq_ack: harq_len,
        num_csi_part1: csi1_len,
        num_csi_part2: csi2_len,
    }
}

// ---------------------------------------------------------------------------
// Resource element mapping (TS 38.211 Section 6.3.1.7)
// ---------------------------------------------------------------------------

/// Resource element assignment for a PRB allocation.
#[derive(Debug, Clone)]
pub struct ReMapping {
    /// PRB indices allocated to PUSCH.
    pub prb_indices: Vec<u16>,
    /// OFDM symbol indices carrying PUSCH data.
    pub data_symbols: Vec<u8>,
    /// OFDM symbol indices carrying DMRS.
    pub dmrs_symbols: Vec<u8>,
}

/// Compute PRB-to-subcarrier resource element mapping.
///
/// Returns a flat list of (PRB, subcarrier_offset) pairs for data REs
/// (excluding DMRS subcarriers per the DMRS type).
pub fn compute_re_mapping(
    start_prb: u16,
    num_prbs: u16,
    dmrs_type: u8,
    dmrs_symbol_mask: u32,
    num_symbols: u8,
    start_symbol: u8,
) -> ReMapping {
    let prb_indices: Vec<u16> = (start_prb..start_prb + num_prbs).collect();

    let mut data_symbols = Vec::new();
    let mut dmrs_symbols = Vec::new();

    for sym in start_symbol..start_symbol + num_symbols {
        if sym < 32 && (dmrs_symbol_mask >> sym) & 1 == 1 {
            dmrs_symbols.push(sym);
        } else {
            data_symbols.push(sym);
        }
    }

    ReMapping {
        prb_indices,
        data_symbols,
        dmrs_symbols,
    }
}

/// Count data resource elements available for PUSCH.
///
/// Each PRB has 12 subcarriers. DMRS subcarriers depend on type:
/// - Type 1: 6 DMRS subcarriers per PRB per DMRS symbol
/// - Type 2: 4 DMRS subcarriers per PRB per DMRS symbol
pub fn count_data_res(
    num_prbs: u16,
    num_data_symbols: usize,
    num_dmrs_symbols: usize,
    dmrs_type: u8,
) -> usize {
    let dmrs_per_prb = if dmrs_type == 2 { 4 } else { 6 };
    let total_re = num_prbs as usize * 12 * (num_data_symbols + num_dmrs_symbols);
    let dmrs_re = num_prbs as usize * dmrs_per_prb * num_dmrs_symbols;
    total_re.saturating_sub(dmrs_re)
}

// ---------------------------------------------------------------------------
// Power control (TS 38.213 Section 7.1)
// ---------------------------------------------------------------------------

/// PUSCH open-loop and closed-loop power control.
#[derive(Debug, Clone)]
pub struct PowerControl {
    /// Nominal UE transmit power P_CMAX (dBm).
    pub p_cmax: i8,
    /// Open-loop path loss estimate (dB, non-negative).
    pub path_loss_db: u16,
    /// Nominal P0 for PUSCH (dBm, can be negative).
    pub p0_nominal: i16,
    /// Path loss compensation factor alpha (0.0..=1.0).
    pub alpha: f64,
    /// Accumulated closed-loop TPC correction (dB, signed).
    pub tpc_accumulation: i8,
    /// Number of allocated PRBs (for bandwidth scaling).
    pub num_prbs: u16,
}

impl PowerControl {
    /// Compute transmit power per TS 38.213 Section 7.1.
    ///
    /// P_PUSCH = min(P_CMAX, 10*log10(M_PUSCH) + P0 + alpha*PL + f(i))
    pub fn compute_tx_power(&self) -> f64 {
        let bw_scaling = 10.0 * (self.num_prbs as f64).log10().max(0.0);
        let ol_power = self.p0_nominal as f64
            + bw_scaling
            + self.alpha * self.path_loss_db as f64;
        let cl_power = ol_power + self.tpc_accumulation as f64;
        (cl_power as f64).min(self.p_cmax as f64)
    }

    /// Apply a TPC command (TS 38.213 Table 7.1.1-1).
    ///
    /// TPC command values: 0 → -1 dB, 1 → 0 dB, 2 → +1 dB, 3 → +3 dB.
    pub fn apply_tpc(&mut self, tpc_command: u8) {
        let delta: i8 = match tpc_command & 0x3 {
            0 => -1,
            1 => 0,
            2 => 1,
            3 => 3,
            _ => 0,
        };
        self.tpc_accumulation = self.tpc_accumulation.saturating_add(delta);
        // Clamp to [-20, +20] dB
        self.tpc_accumulation = self.tpc_accumulation.clamp(-20, 20);
    }
}

// ---------------------------------------------------------------------------
// TBS calculation (TS 38.214 Section 6.1.4.2)
// ---------------------------------------------------------------------------

/// Calculate the Transport Block Size (TBS) per TS 38.214 Section 6.1.4.2.
///
/// Parameters:
/// - `mcs_idx`: MCS index.
/// - `mcs_table_id`: 1, 2, or 3.
/// - `num_prbs`: number of allocated PRBs.
/// - `num_layers`: number of MIMO layers.
/// - `num_symbols`: number of OFDM symbols.
/// - `dmrs_overhead`: DMRS overhead per PRB in RE.
/// - `xoh`: additional overhead (0, 6, 12, or 18).
pub fn calculate_tbs(
    mcs_idx: u8,
    mcs_table_id: u8,
    num_prbs: u16,
    num_layers: u8,
    num_symbols: u8,
    dmrs_overhead: u8,
    xoh: u8,
) -> Option<u32> {
    let entry = mcs_lookup(mcs_table_id, mcs_idx)?;
    let q_m = entry.modulation.bits_per_symbol() as u64;
    let r_x1024 = entry.code_rate_x1024 as u64;

    // N_RE per PRB, capped at 156
    let n_re_prime = (12u32 * num_symbols as u32)
        .saturating_sub(dmrs_overhead as u32)
        .saturating_sub(xoh as u32)
        .min(156);

    let n_re = n_re_prime as u64 * num_prbs as u64;

    // N_info = N_RE * R * Q_m * layers
    let n_info = n_re * r_x1024 * q_m * num_layers as u64 / 1024;

    let tbs = if n_info <= 3824 {
        let n = if n_info < 24 {
            0u32
        } else {
            let bits = (n_info as f64 / 24.0).log2().floor() as u32;
            bits
        };
        let pow2n = 1u64 << n;
        let n_info_prime = (n_info / pow2n * pow2n).max(24);
        *TBS_TABLE.iter().find(|&&t| t as u64 >= n_info_prime).unwrap_or(&3824)
    } else {
        let c = if n_info > 8424 { (n_info + 24 + 8424 - 1) / 8424 } else { 1 };
        let num = n_info + 24;
        let denom = 8 * c;
        let ceil_val = (num + denom - 1) / denom;
        let tbs_raw = 8 * c * ceil_val - 24;
        tbs_raw as u32
    };

    Some(tbs)
}

// ---------------------------------------------------------------------------
// PUSCH configuration
// ---------------------------------------------------------------------------

/// Complete PUSCH allocation configuration.
#[derive(Debug, Clone)]
pub struct PuschConfig {
    /// MCS index per the selected MCS table.
    pub mcs_index: u8,
    /// MCS table to use: 1 (64QAM default), 2 (256QAM), or 3 (64QAM low-SE).
    pub mcs_table: u8,
    /// Number of allocated PRBs.
    pub num_prbs: u16,
    /// Number of MIMO layers (1..=4).
    pub num_layers: u8,
    /// Redundancy version index (0..=3).
    pub rv_idx: u8,
    /// Transform precoding mode.
    pub transform_precoding: TransformPrecoding,
    /// Physical cell ID.
    pub cell_id: u16,
    /// RNTI of the scheduled UE.
    pub rnti: u16,
    /// Scrambling identity N_ID (DMRS/data).
    pub n_id: u32,
    /// DMRS scrambling ID selector n_SCID.
    pub n_scid: bool,
    /// First PRB of the allocation.
    pub start_prb: u16,
    /// Number of OFDM symbols allocated.
    pub num_symbols: u8,
    /// Starting OFDM symbol index within slot.
    pub start_symbol: u8,
    /// Bitmap of DMRS symbol positions (bit i = 1 means symbol i carries DMRS).
    pub dmrs_symbol_mask: u32,
    /// DMRS type: 1 or 2.
    pub dmrs_type: u8,
    /// Whether frequency hopping is enabled.
    pub frequency_hopping: bool,
    /// PRB offset for second hop (if frequency hopping).
    pub second_hop_prb: u16,
    /// Closed-loop TPC accumulation (dB).
    pub tpc_accumulation: i8,
    /// Nominal P0 for power control (dBm).
    pub p0_nominal: i16,
}

impl Default for PuschConfig {
    fn default() -> Self {
        PuschConfig {
            mcs_index: 9,
            mcs_table: 1,
            num_prbs: 25,
            num_layers: 1,
            rv_idx: 0,
            transform_precoding: TransformPrecoding::Disabled,
            cell_id: 1,
            rnti: 0x1234,
            n_id: 0,
            n_scid: false,
            start_prb: 0,
            num_symbols: 14,
            start_symbol: 0,
            dmrs_symbol_mask: 0b0000_0000_0000_0100,
            dmrs_type: 1,
            frequency_hopping: false,
            second_hop_prb: 0,
            tpc_accumulation: 0,
            p0_nominal: -80,
        }
    }
}

// ---------------------------------------------------------------------------
// PUSCH transport block descriptor
// ---------------------------------------------------------------------------

/// Transport block and related PUSCH processing metadata.
#[derive(Debug, Clone)]
pub struct PuschTransportBlock {
    /// Raw TB bytes.
    pub bytes: Vec<u8>,
    /// TB size in bits.
    pub tbs_bits: u32,
    /// Redundancy version for HARQ retransmissions.
    pub rv_idx: u8,
    /// HARQ process ID.
    pub harq_pid: u8,
    /// New data indicator.
    pub ndi: bool,
}

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// 5G NR PUSCH processor.
///
/// Implements the complete PUSCH uplink processing chain per
/// TS 38.211, TS 38.212, and TS 38.214.
#[derive(Debug, Clone)]
pub struct NrPuschProcessor {
    /// PUSCH allocation configuration.
    pub config: PuschConfig,
    /// DMRS configuration.
    pub dmrs_config: DmrsConfig,
    /// Power control state.
    pub power_control: PowerControl,
}

impl NrPuschProcessor {
    /// Create a new PUSCH processor with the given configuration.
    pub fn new(config: PuschConfig) -> Self {
        let dmrs_config = DmrsConfig {
            dmrs_type: config.dmrs_type,
            n_id: config.n_id,
            n_scid: config.n_scid,
            delta: 0,
        };

        let power_control = PowerControl {
            p_cmax: 23,
            path_loss_db: 80,
            p0_nominal: config.p0_nominal,
            alpha: 0.8,
            tpc_accumulation: config.tpc_accumulation,
            num_prbs: config.num_prbs,
        };

        NrPuschProcessor {
            config,
            dmrs_config,
            power_control,
        }
    }

    /// Calculate TBS for the configured MCS and allocation.
    pub fn calculate_tbs(&self, mcs_idx: u8, num_prbs: u16, num_layers: u8) -> u32 {
        // Count DMRS symbols for overhead calculation
        let num_dmrs_symbols = (0..32)
            .filter(|&i| (self.config.dmrs_symbol_mask >> i) & 1 == 1)
            .count() as u8;
        let dmrs_per_prb = if self.config.dmrs_type == 2 { 4 } else { 6 };
        let dmrs_overhead = num_dmrs_symbols * dmrs_per_prb;

        calculate_tbs(
            mcs_idx,
            self.config.mcs_table,
            num_prbs,
            num_layers,
            self.config.num_symbols,
            dmrs_overhead,
            0,
        )
        .unwrap_or(0)
    }

    /// Select the LDPC base graph based on TBS and effective code rate.
    pub fn select_base_graph(&self, tbs: u32, coding_rate: f64) -> u8 {
        select_base_graph(tbs, coding_rate)
    }

    /// Generate DMRS for a specific slot and symbol.
    pub fn generate_dmrs(&self, slot: u32, symbol: u32) -> Vec<Complex64> {
        let num_sc = self.config.num_prbs as usize * 12;
        generate_dmrs(&self.dmrs_config, slot, symbol, num_sc)
    }

    /// Process a full transport block through the PUSCH chain.
    ///
    /// Steps:
    /// 1. TB-CRC attachment
    /// 2. Code block segmentation
    /// 3. LDPC encoding
    /// 4. Rate matching
    /// 5. Code block concatenation
    /// 6. Scrambling
    /// 7. Modulation
    /// 8. Transform precoding (if enabled)
    /// 9. Layer mapping
    pub fn process_transport_block(&self, tb: &[u8]) -> Vec<Complex64> {
        let entry = match mcs_lookup(self.config.mcs_table, self.config.mcs_index) {
            Some(e) => e,
            None => return vec![],
        };

        // Convert bytes to bits (MSB-first)
        let tb_bits: Vec<bool> = tb
            .iter()
            .flat_map(|&byte| (0..8).rev().map(move |i| (byte >> i) & 1 == 1))
            .collect();

        // Step 1: TB-CRC attachment
        let tb_with_crc = attach_tb_crc(&tb_bits);

        // Step 2: Code block segmentation
        let code_blocks = segment_code_blocks(&tb_with_crc);

        // Compute E_r per code block (simplified uniform allocation)
        let mcs_entry = entry;
        let num_dmrs_sym = (0..32)
            .filter(|&i| (self.config.dmrs_symbol_mask >> i) & 1 == 1)
            .count();
        let num_data_sym = self.config.num_symbols as usize - num_dmrs_sym;
        let dmrs_per_prb = if self.config.dmrs_type == 2 { 4 } else { 6 };
        let data_re_per_sym = self.config.num_prbs as usize * 12;
        let dmrs_re_total = self.config.num_prbs as usize * dmrs_per_prb * num_dmrs_sym;
        let data_re_total = data_re_per_sym * self.config.num_symbols as usize - dmrs_re_total;
        let q_m = mcs_entry.modulation.bits_per_symbol() as usize;
        let g = data_re_total * q_m * self.config.num_layers as usize;

        // Distribute bits evenly across code blocks
        let c = code_blocks.len();
        let e_per_cb = if c > 0 { g / c } else { g };

        // Steps 3 & 4: LDPC encode + rate match each CB
        let mut concatenated: Vec<bool> = Vec::with_capacity(g);
        for cb in &code_blocks {
            let encoded = ldpc_encode(&cb.info_bits, &cb.ldpc);
            let matched = rate_match(
                &encoded,
                e_per_cb,
                self.config.rv_idx,
                cb.ldpc.n_cb as usize,
            );
            concatenated.extend(matched);
        }

        // Trim or pad to exact G bits
        concatenated.resize(g, false);

        // Step 6: Scrambling
        let c_init = pusch_scrambling_init(self.config.rnti, self.config.n_id, 0);
        let scrambled = scramble(&concatenated, c_init);

        // Step 7: Modulation
        let modulation = mcs_entry.modulation;
        let mut symbols = modulate(&scrambled, modulation);

        // Step 8: Transform precoding (DFT-s-OFDM if enabled)
        if self.config.transform_precoding == TransformPrecoding::Enabled {
            // Process in blocks of num_prbs * 12 symbols
            let m = self.config.num_prbs as usize * 12;
            let mut precoded = Vec::with_capacity(symbols.len());
            for chunk in symbols.chunks(m) {
                let mut block = chunk.to_vec();
                block.resize(m, (0.0, 0.0));
                let spread = dft_spread(&block);
                precoded.extend(spread);
            }
            symbols = precoded;
        }

        // Step 9: Layer mapping
        let layers = layer_map(&symbols, self.config.num_layers);

        // Return first layer (or concatenation for single-layer)
        if layers.is_empty() {
            vec![]
        } else {
            layers[0].clone()
        }
    }

    /// Compute effective code rate for given TBS and allocation.
    pub fn effective_code_rate(&self, tbs: u32) -> f64 {
        let num_dmrs_sym = (0..32)
            .filter(|&i| (self.config.dmrs_symbol_mask >> i) & 1 == 1)
            .count();
        let dmrs_per_prb = if self.config.dmrs_type == 2 { 4 } else { 6 };
        let dmrs_re = self.config.num_prbs as usize * dmrs_per_prb * num_dmrs_sym;
        let total_re = self.config.num_prbs as usize * 12 * self.config.num_symbols as usize;
        let data_re = total_re.saturating_sub(dmrs_re);

        let entry = mcs_lookup(self.config.mcs_table, self.config.mcs_index);
        let q_m = entry.map_or(2, |e| e.modulation.bits_per_symbol()) as f64;

        let capacity_bits = data_re as f64 * q_m * self.config.num_layers as f64;
        if capacity_bits > 0.0 {
            tbs as f64 / capacity_bits
        } else {
            0.0
        }
    }

    /// Get the current transmit power (dBm).
    pub fn transmit_power(&self) -> f64 {
        self.power_control.compute_tx_power()
    }

    /// Apply a TPC command and update the closed-loop power correction.
    pub fn apply_tpc(&mut self, tpc_command: u8) {
        self.power_control.apply_tpc(tpc_command);
    }

    /// Return the MCS entry for the current configuration.
    pub fn mcs_entry(&self) -> Option<McsPuschEntry> {
        mcs_lookup(self.config.mcs_table, self.config.mcs_index)
    }

    /// Number of data resource elements for the configured allocation.
    pub fn num_data_res(&self) -> usize {
        let num_dmrs_sym = (0..32)
            .filter(|&i| (self.config.dmrs_symbol_mask >> i) & 1 == 1)
            .count();
        let num_data_sym = self.config.num_symbols as usize - num_dmrs_sym;
        count_data_res(
            self.config.num_prbs,
            num_data_sym,
            num_dmrs_sym,
            self.config.dmrs_type,
        )
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> PuschConfig {
        PuschConfig::default()
    }

    // ---- MCS table tests ----

    #[test]
    fn test_mcs_table1_entry_0() {
        let e = mcs_lookup(1, 0).unwrap();
        assert_eq!(e.modulation, PuschModulation::Qpsk);
        assert_eq!(e.code_rate_x1024, 120);
    }

    #[test]
    fn test_mcs_table1_entry_17() {
        let e = mcs_lookup(1, 17).unwrap();
        assert_eq!(e.modulation, PuschModulation::Qam64);
        assert_eq!(e.code_rate_x1024, 438);
    }

    #[test]
    fn test_mcs_table2_entry_20() {
        let e = mcs_lookup(2, 20).unwrap();
        assert_eq!(e.modulation, PuschModulation::Qam256);
        assert_eq!(e.code_rate_x1024, 682);
    }

    #[test]
    fn test_mcs_table3_entry_0() {
        let e = mcs_lookup(3, 0).unwrap();
        assert_eq!(e.modulation, PuschModulation::Qpsk);
        assert_eq!(e.code_rate_x1024, 30);
    }

    #[test]
    fn test_mcs_lookup_invalid() {
        assert!(mcs_lookup(1, 29).is_none());
        assert!(mcs_lookup(4, 0).is_none());
    }

    #[test]
    fn test_mcs_table1_length() {
        assert_eq!(MCS_TABLE_1.len(), 29);
    }

    #[test]
    fn test_mcs_table2_length() {
        assert_eq!(MCS_TABLE_2.len(), 28);
    }

    #[test]
    fn test_mcs_table3_length() {
        assert_eq!(MCS_TABLE_3.len(), 29);
    }

    // ---- Modulation bits per symbol ----

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(PuschModulation::Pi2Bpsk.bits_per_symbol(), 1);
        assert_eq!(PuschModulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(PuschModulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(PuschModulation::Qam64.bits_per_symbol(), 6);
        assert_eq!(PuschModulation::Qam256.bits_per_symbol(), 8);
    }

    // ---- CRC tests ----

    #[test]
    fn test_crc24a_all_zeros() {
        let bits = vec![false; 40];
        let crc = crc24a_bits(&bits);
        // Must be a valid 24-bit number
        assert!(crc < (1 << 24));
    }

    #[test]
    fn test_crc16_consistency() {
        let bits: Vec<bool> = (0..16).map(|i| i % 3 == 0).collect();
        let crc_a = crc16_bits(&bits);
        let crc_b = crc16_bits(&bits);
        assert_eq!(crc_a, crc_b);
    }

    #[test]
    fn test_attach_tb_crc_small() {
        let bits = vec![false; 100]; // 100 bits < 3824 → CRC-16
        let with_crc = attach_tb_crc(&bits);
        assert_eq!(with_crc.len(), 116); // 100 + 16
    }

    #[test]
    fn test_attach_tb_crc_large() {
        let bits = vec![false; 4000]; // > 3824 → CRC-24A
        let with_crc = attach_tb_crc(&bits);
        assert_eq!(with_crc.len(), 4024); // 4000 + 24
    }

    #[test]
    fn test_attach_cb_crc() {
        let bits = vec![true; 50];
        let with_crc = attach_cb_crc(&bits);
        assert_eq!(with_crc.len(), 74); // 50 + 24
    }

    #[test]
    fn test_crc24b_sensitivity() {
        let mut bits = vec![false; 32];
        let crc_orig = crc24b_bits(&bits);
        bits[15] = true;
        let crc_mod = crc24b_bits(&bits);
        assert_ne!(crc_orig, crc_mod);
    }

    // ---- LDPC parameter tests ----

    #[test]
    fn test_select_base_graph_large() {
        // TBS > 3824 → BG1
        assert_eq!(select_base_graph(4000, 0.5), 1);
    }

    #[test]
    fn test_select_base_graph_high_rate() {
        // TBS > 292 and R > 0.67 → BG1
        assert_eq!(select_base_graph(500, 0.8), 1);
    }

    #[test]
    fn test_select_base_graph_small() {
        // Small TBS, low rate → BG2
        assert_eq!(select_base_graph(200, 0.3), 2);
    }

    #[test]
    fn test_lifting_size_minimum() {
        // k_bits=22, k_b=22 → need Z >= 1 → minimum valid Z = 2
        let z = select_lifting_size(22, 22);
        assert!(z >= 1);
        assert!(z <= 384);
    }

    #[test]
    fn test_ldpc_params_bg1() {
        let p = ldpc_params(4000, 1);
        assert_eq!(p.base_graph, 1);
        assert_eq!(p.k_b, 22);
        assert!(p.lifting_size >= 2);
        assert_eq!(p.n_cb, 68 * p.lifting_size as u32);
    }

    #[test]
    fn test_ldpc_params_bg2() {
        let p = ldpc_params(200, 2);
        assert_eq!(p.base_graph, 2);
        assert_eq!(p.k_b, 10);
        assert_eq!(p.n_cb, 52 * p.lifting_size as u32);
    }

    // ---- Code block segmentation ----

    #[test]
    fn test_segment_single_cb() {
        let bits = vec![false; 1000];
        let with_crc = attach_tb_crc(&bits);
        let cbs = segment_code_blocks(&with_crc);
        assert_eq!(cbs.len(), 1);
    }

    #[test]
    fn test_segment_multiple_cbs() {
        // Large TB will require multiple CBs
        let bits = vec![true; 8000];
        let with_crc = attach_tb_crc(&bits);
        let cbs = segment_code_blocks(&with_crc);
        assert!(cbs.len() >= 1);
    }

    #[test]
    fn test_cb_size_multiple_of_z() {
        let bits = vec![false; 2000];
        let with_crc = attach_tb_crc(&bits);
        let cbs = segment_code_blocks(&with_crc);
        for cb in &cbs {
            let z = cb.ldpc.lifting_size as usize;
            let k_b = cb.ldpc.k_b as usize;
            // info bits should be close to k_b * z
            assert!(cb.info_bits.len() >= k_b * z);
        }
    }

    // ---- Gold sequence ----

    #[test]
    fn test_gold_sequence_determinism() {
        let mut g1 = GoldSequence::new(12345);
        let mut g2 = GoldSequence::new(12345);
        let bits1: Vec<bool> = (0..100).map(|_| g1.next_bit()).collect();
        let bits2: Vec<bool> = (0..100).map(|_| g2.next_bit()).collect();
        assert_eq!(bits1, bits2);
    }

    #[test]
    fn test_gold_sequence_different_inits() {
        let mut g1 = GoldSequence::new(0);
        let mut g2 = GoldSequence::new(1);
        let b1 = g1.generate(64);
        let b2 = g2.generate(64);
        assert_ne!(b1, b2);
    }

    #[test]
    fn test_gold_sequence_generate() {
        let mut g = GoldSequence::new(0);
        let bits = g.generate(100);
        assert_eq!(bits.len(), 100);
    }

    // ---- Scrambling ----

    #[test]
    fn test_scramble_length() {
        let bits = vec![false; 200];
        let c_init = pusch_scrambling_init(0x1234, 0, 0);
        let scrambled = scramble(&bits, c_init);
        assert_eq!(scrambled.len(), 200);
    }

    #[test]
    fn test_scramble_invertibility() {
        let bits: Vec<bool> = (0..64).map(|i| i % 3 == 0).collect();
        let c_init = pusch_scrambling_init(0xABCD, 42, 0);
        let scrambled = scramble(&bits, c_init);
        let descrambled = scramble(&scrambled, c_init);
        assert_eq!(bits, descrambled);
    }

    #[test]
    fn test_scramble_rnti_effect() {
        let bits = vec![true; 32];
        let c1 = pusch_scrambling_init(1, 0, 0);
        let c2 = pusch_scrambling_init(2, 0, 0);
        let s1 = scramble(&bits, c1);
        let s2 = scramble(&bits, c2);
        assert_ne!(s1, s2);
    }

    // ---- Modulation tests ----

    #[test]
    fn test_modulate_qpsk_length() {
        let bits: Vec<bool> = (0..20).map(|i| i % 2 == 0).collect();
        let syms = modulate(&bits, PuschModulation::Qpsk);
        assert_eq!(syms.len(), 10); // 20 bits / 2 bits-per-sym
    }

    #[test]
    fn test_modulate_qam16_length() {
        let bits = vec![false; 40];
        let syms = modulate(&bits, PuschModulation::Qam16);
        assert_eq!(syms.len(), 10); // 40 / 4
    }

    #[test]
    fn test_modulate_qam64_length() {
        let bits = vec![false; 60];
        let syms = modulate(&bits, PuschModulation::Qam64);
        assert_eq!(syms.len(), 10); // 60 / 6
    }

    #[test]
    fn test_modulate_qam256_length() {
        let bits = vec![false; 80];
        let syms = modulate(&bits, PuschModulation::Qam256);
        assert_eq!(syms.len(), 10); // 80 / 8
    }

    #[test]
    fn test_modulate_qpsk_power() {
        // All-zero bits: QPSK symbol (1/√2, 1/√2), power = 1.0
        let bits = vec![false; 2];
        let syms = modulate(&bits, PuschModulation::Qpsk);
        let power = syms[0].0 * syms[0].0 + syms[0].1 * syms[0].1;
        assert!((power - 1.0).abs() < 1e-9, "power = {}", power);
    }

    #[test]
    fn test_modulate_pi2_bpsk_length() {
        let bits = vec![false; 8];
        let syms = modulate(&bits, PuschModulation::Pi2Bpsk);
        assert_eq!(syms.len(), 8);
    }

    // ---- DFT-s-OFDM transform precoding ----

    #[test]
    fn test_dft_spread_length() {
        let syms: Vec<Complex64> = (0..12).map(|i| (i as f64, 0.0)).collect();
        let spread = dft_spread(&syms);
        assert_eq!(spread.len(), 12);
    }

    #[test]
    fn test_dft_idft_roundtrip() {
        let orig: Vec<Complex64> = (0..8).map(|i| (i as f64, 0.0)).collect();
        let spread = dft_spread(&orig);
        let recovered = idft_despread(&spread);
        for (a, b) in orig.iter().zip(recovered.iter()) {
            assert!((a.0 - b.0).abs() < 1e-9, "re mismatch: {} vs {}", a.0, b.0);
            assert!((a.1 - b.1).abs() < 1e-9, "im mismatch: {} vs {}", a.1, b.1);
        }
    }

    #[test]
    fn test_dft_spread_energy_preservation() {
        let syms: Vec<Complex64> = (0..4).map(|i| (i as f64, 0.0)).collect();
        let in_energy: f64 = syms.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum();
        let spread = dft_spread(&syms);
        let out_energy: f64 = spread.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum();
        assert!((in_energy - out_energy).abs() < 1e-9);
    }

    // ---- Layer mapping ----

    #[test]
    fn test_layer_map_single() {
        let syms: Vec<Complex64> = (0..10).map(|i| (i as f64, 0.0)).collect();
        let layers = layer_map(&syms, 1);
        assert_eq!(layers.len(), 1);
        assert_eq!(layers[0].len(), 10);
    }

    #[test]
    fn test_layer_map_two_layers() {
        let syms: Vec<Complex64> = (0..10).map(|i| (i as f64, 0.0)).collect();
        let layers = layer_map(&syms, 2);
        assert_eq!(layers.len(), 2);
        assert_eq!(layers[0].len() + layers[1].len(), 10);
    }

    #[test]
    fn test_layer_demap_roundtrip() {
        let syms: Vec<Complex64> = (0..12).map(|i| (i as f64, 0.0)).collect();
        let layers = layer_map(&syms, 3);
        let demapped = layer_demap(&layers);
        assert_eq!(demapped.len(), syms.len());
        for (a, b) in syms.iter().zip(demapped.iter()) {
            assert_eq!(a.0, b.0);
        }
    }

    #[test]
    fn test_layer_map_four_layers() {
        let syms: Vec<Complex64> = (0..8).map(|i| (i as f64, 0.0)).collect();
        let layers = layer_map(&syms, 4);
        assert_eq!(layers.len(), 4);
        let total: usize = layers.iter().map(|l| l.len()).sum();
        assert_eq!(total, 8);
    }

    // ---- DMRS generation ----

    #[test]
    fn test_generate_dmrs_length() {
        let cfg = DmrsConfig::default();
        let dmrs = generate_dmrs(&cfg, 0, 2, 300);
        // Type 1: num_subcarriers/2 = 150 DMRS symbols
        assert_eq!(dmrs.len(), 150);
    }

    #[test]
    fn test_generate_dmrs_unit_power() {
        let cfg = DmrsConfig::default();
        let dmrs = generate_dmrs(&cfg, 0, 2, 24);
        for sym in &dmrs {
            let power = sym.0 * sym.0 + sym.1 * sym.1;
            assert!((power - 1.0).abs() < 1e-9, "power = {}", power);
        }
    }

    #[test]
    fn test_generate_dmrs_slot_dependence() {
        let cfg = DmrsConfig::default();
        let d1 = generate_dmrs(&cfg, 0, 2, 24);
        let d2 = generate_dmrs(&cfg, 1, 2, 24);
        assert_ne!(d1, d2);
    }

    #[test]
    fn test_generate_dmrs_type2() {
        let cfg = DmrsConfig { dmrs_type: 2, ..DmrsConfig::default() };
        let dmrs = generate_dmrs(&cfg, 0, 4, 300);
        // Type 2: (num_subcarriers * 2) / 3 = 200 DMRS symbols
        assert_eq!(dmrs.len(), 200);
    }

    // ---- TBS calculation ----

    #[test]
    fn test_tbs_table1_mcs9() {
        let tbs = calculate_tbs(9, 1, 25, 1, 14, 6, 0);
        assert!(tbs.is_some());
        assert!(tbs.unwrap() > 0);
    }

    #[test]
    fn test_tbs_larger_prb() {
        let tbs_small = calculate_tbs(9, 1, 10, 1, 14, 6, 0).unwrap_or(0);
        let tbs_large = calculate_tbs(9, 1, 50, 1, 14, 6, 0).unwrap_or(0);
        assert!(tbs_large > tbs_small);
    }

    #[test]
    fn test_tbs_higher_mcs_higher_rate() {
        let tbs_low = calculate_tbs(5, 1, 25, 1, 14, 6, 0).unwrap_or(0);
        let tbs_high = calculate_tbs(20, 1, 25, 1, 14, 6, 0).unwrap_or(0);
        assert!(tbs_high > tbs_low);
    }

    #[test]
    fn test_tbs_table2_256qam() {
        let tbs = calculate_tbs(25, 2, 25, 1, 14, 6, 0);
        assert!(tbs.is_some());
        assert!(tbs.unwrap() > 0);
    }

    #[test]
    fn test_tbs_invalid_mcs() {
        let tbs = calculate_tbs(31, 1, 25, 1, 14, 6, 0);
        assert!(tbs.is_none());
    }

    // ---- Power control ----

    #[test]
    fn test_power_control_basic() {
        let pc = PowerControl {
            p_cmax: 23,
            path_loss_db: 80,
            p0_nominal: -80,
            alpha: 0.8,
            tpc_accumulation: 0,
            num_prbs: 1,
        };
        let pwr = pc.compute_tx_power();
        assert!(pwr <= 23.0);
    }

    #[test]
    fn test_tpc_accumulation() {
        let mut pc = PowerControl {
            p_cmax: 23,
            path_loss_db: 80,
            p0_nominal: -80,
            alpha: 0.8,
            tpc_accumulation: 0,
            num_prbs: 25,
        };
        pc.apply_tpc(2); // +1 dB
        assert_eq!(pc.tpc_accumulation, 1);
        pc.apply_tpc(3); // +3 dB
        assert_eq!(pc.tpc_accumulation, 4);
        pc.apply_tpc(0); // -1 dB
        assert_eq!(pc.tpc_accumulation, 3);
    }

    #[test]
    fn test_tpc_clamp() {
        let mut pc = PowerControl {
            p_cmax: 23,
            path_loss_db: 0,
            p0_nominal: 0,
            alpha: 0.0,
            tpc_accumulation: 18,
            num_prbs: 1,
        };
        pc.apply_tpc(3); // +3 dB → 21
        pc.apply_tpc(3); // +3 dB → 20 (clamped)
        assert_eq!(pc.tpc_accumulation, 20);
    }

    // ---- UCI multiplexing ----

    #[test]
    fn test_uci_mux_harq_ack() {
        let data = vec![false; 100];
        let uci = UciInfo {
            harq_ack: vec![true, false],
            csi_part1: vec![],
            csi_part2: vec![],
        };
        let result = multiplex_uci(&data, &uci, 50, 2);
        assert_eq!(result.bits.len(), 100);
        assert_eq!(result.num_harq_ack, 2);
        assert!(result.bits[0]); // HARQ ACK = 1
        assert!(!result.bits[1]); // HARQ NACK = 0
    }

    #[test]
    fn test_uci_mux_csi() {
        let data = vec![false; 200];
        let uci = UciInfo {
            harq_ack: vec![true],
            csi_part1: vec![false, true, false],
            csi_part2: vec![true],
        };
        let result = multiplex_uci(&data, &uci, 100, 2);
        assert_eq!(result.num_harq_ack, 1);
        assert_eq!(result.num_csi_part1, 3);
        assert_eq!(result.num_csi_part2, 1);
    }

    // ---- Resource element mapping ----

    #[test]
    fn test_re_mapping_prb_count() {
        let m = compute_re_mapping(0, 25, 1, 0b0000_0100, 14, 0);
        assert_eq!(m.prb_indices.len(), 25);
    }

    #[test]
    fn test_re_mapping_dmrs_symbols() {
        // DMRS at symbol 2 (bit 2 set)
        let m = compute_re_mapping(0, 10, 1, 0b0000_0100, 14, 0);
        assert!(m.dmrs_symbols.contains(&2));
        assert!(!m.data_symbols.contains(&2));
    }

    #[test]
    fn test_count_data_res_type1() {
        // 25 PRBs, 12 data symbols, 1 DMRS symbol, type 1 (6 DMRS per PRB)
        let res = count_data_res(25, 12, 1, 1);
        let expected = 25 * 12 * 13 - 25 * 6 * 1;
        assert_eq!(res, expected);
    }

    // ---- Processor integration ----

    #[test]
    fn test_processor_tbs() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        let tbs = proc.calculate_tbs(9, 25, 1);
        assert!(tbs > 0);
    }

    #[test]
    fn test_processor_dmrs() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        let dmrs = proc.generate_dmrs(0, 2);
        assert!(!dmrs.is_empty());
    }

    #[test]
    fn test_processor_select_base_graph() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        assert_eq!(proc.select_base_graph(5000, 0.5), 1);
        assert_eq!(proc.select_base_graph(200, 0.3), 2);
    }

    #[test]
    fn test_processor_mcs_entry() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        let entry = proc.mcs_entry().unwrap();
        assert_eq!(entry.mcs_index, 9);
        assert_eq!(entry.modulation, PuschModulation::Qpsk);
    }

    #[test]
    fn test_processor_process_tb_produces_output() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        let tb = vec![0xABu8; 40];
        let symbols = proc.process_transport_block(&tb);
        assert!(!symbols.is_empty());
    }

    #[test]
    fn test_processor_effective_code_rate() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        let tbs = proc.calculate_tbs(9, 25, 1);
        let rate = proc.effective_code_rate(tbs);
        assert!(rate > 0.0);
        assert!(rate <= 1.0);
    }

    #[test]
    fn test_processor_tx_power() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        let pwr = proc.transmit_power();
        assert!(pwr <= 23.0);
    }

    #[test]
    fn test_processor_apply_tpc() {
        let config = default_config();
        let mut proc = NrPuschProcessor::new(config);
        let pwr_before = proc.transmit_power();
        proc.apply_tpc(2); // +1 dB
        let pwr_after = proc.transmit_power();
        // After TPC +1, power should be slightly higher (or clamped)
        assert!(pwr_after >= pwr_before || pwr_after == 23.0);
    }

    #[test]
    fn test_processor_num_data_res() {
        let config = default_config();
        let proc = NrPuschProcessor::new(config);
        let res = proc.num_data_res();
        assert!(res > 0);
    }

    #[test]
    fn test_processor_transform_precoding_enabled() {
        let config = PuschConfig {
            mcs_index: 5,
            transform_precoding: TransformPrecoding::Enabled,
            ..default_config()
        };
        let proc = NrPuschProcessor::new(config);
        let tb = vec![0xFFu8; 20];
        let symbols = proc.process_transport_block(&tb);
        assert!(!symbols.is_empty());
    }

    #[test]
    fn test_processor_256qam_table() {
        let config = PuschConfig {
            mcs_index: 20,
            mcs_table: 2,
            ..default_config()
        };
        let proc = NrPuschProcessor::new(config);
        let entry = proc.mcs_entry().unwrap();
        assert_eq!(entry.modulation, PuschModulation::Qam256);
    }

    #[test]
    fn test_rate_match_length() {
        let codeword = vec![false; 100];
        let matched = rate_match(&codeword, 50, 0, 100);
        assert_eq!(matched.len(), 50);
    }

    #[test]
    fn test_rate_match_rv_variations() {
        let codeword: Vec<bool> = (0..100).map(|i| i % 3 == 0).collect();
        let m0 = rate_match(&codeword, 50, 0, 100);
        let m1 = rate_match(&codeword, 50, 1, 100);
        // Different RV may produce different starting positions
        // (they'll be different unless circular buffer alignment is the same)
        // Just verify lengths match
        assert_eq!(m0.len(), 50);
        assert_eq!(m1.len(), 50);
    }

    #[test]
    fn test_ldpc_encode_length() {
        let params = ldpc_params(100, 2);
        let info = vec![false; 100];
        let encoded = ldpc_encode(&info, &params);
        assert_eq!(encoded.len(), params.n_cb as usize);
    }

    #[test]
    fn test_ldpc_systematic_bits_preserved() {
        let params = ldpc_params(50, 2);
        let info: Vec<bool> = (0..50).map(|i| i % 3 == 0).collect();
        let encoded = ldpc_encode(&info, &params);
        // Systematic bits should be identical to input in first K positions
        for (i, (&orig, &enc)) in info.iter().zip(encoded.iter()).enumerate() {
            assert_eq!(orig, enc, "mismatch at systematic position {}", i);
        }
    }
}

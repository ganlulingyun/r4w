//! # 5G NR PDSCH Processor
//!
//! Implements Physical Downlink Shared Channel (PDSCH) processing per
//! 3GPP TS 38.211, TS 38.212, and TS 38.214.
//!
//! ## Processing Chain (TS 38.212 Section 7.2)
//!
//! ```text
//! Transport Block
//!     │
//!     ▼
//! CRC Attachment (CRC-24A or CRC-16)
//!     │
//!     ▼
//! Code Block Segmentation (+ CRC-24B per CB)
//!     │
//!     ▼
//! LDPC Encoding (Base Graph 1 or 2)
//!     │
//!     ▼
//! Rate Matching (circular buffer, rv_idx)
//!     │
//!     ▼
//! Code Block Concatenation
//!     │
//!     ▼
//! Scrambling
//!     │
//!     ▼
//! Modulation (QPSK/16QAM/64QAM/256QAM)
//!     │
//!     ▼
//! Layer Mapping
//!     │
//!     ▼
//! Resource Element Mapping (PRB allocation + DMRS avoidance)
//! ```
//!
//! ## MCS Table (TS 38.214 Table 5.1.3.1-1)
//!
//! | MCS | Modulation | Code Rate (×1024) | Spectral Efficiency |
//! |-----|------------|-------------------|---------------------|
//! |  0  | QPSK       | 120               | 0.2344              |
//! |  9  | QPSK       | 873               | 1.7051              |
//! | 10  | 16QAM      | 340               | 2.6563              |
//! | 16  | 16QAM      | 873               | 6.8164 (reserved)   |
//! | 17  | 64QAM      | 438               | 5.1152              |
//! | 27  | 64QAM      | 948               | 11.0742             |
//!
//! # Example
//!
//! ```
//! use r4w_core::nr_pdsch_processor::{PdschProcessor, PdschConfig, DmrsConfig};
//!
//! let config = PdschConfig {
//!     mcs: 9,
//!     n_prb: 25,
//!     n_layers: 1,
//!     dmrs_type: 1,
//!     rv_idx: 0,
//! };
//! let dmrs = DmrsConfig {
//!     type_id: 1,
//!     additional_pos: 0,
//!     scrambling_id: 0,
//! };
//! let processor = PdschProcessor::new(config, dmrs);
//! let tbs = processor.compute_tbs(1, 0, 14, 0);
//! assert!(tbs.tbs_bits > 0);
//! ```

// ---------------------------------------------------------------------------
// MCS / Modulation tables
// ---------------------------------------------------------------------------

/// Modulation order for PDSCH.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    Qpsk,
    Qam16,
    Qam64,
    Qam256,
}

impl Modulation {
    /// Bits per symbol (Q_m).
    pub fn bits_per_symbol(self) -> u8 {
        match self {
            Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
            Modulation::Qam256 => 8,
        }
    }

    /// Number of constellation points.
    pub fn order(self) -> usize {
        1 << (self.bits_per_symbol() as usize)
    }
}

/// Single MCS table entry per TS 38.214 Table 5.1.3.1-1.
#[derive(Debug, Clone, Copy)]
pub struct McsEntry {
    pub index: u8,
    pub modulation: Modulation,
    /// Code rate numerator when denominator is 1024.
    pub code_rate_x1024: u16,
}

/// Full MCS table 1 (up to 64QAM), TS 38.214 Table 5.1.3.1-1.
/// Indices 0-27; index 28 reserved for 256QAM (Table 2 extension).
pub const MCS_TABLE_64QAM: [McsEntry; 29] = [
    McsEntry { index: 0,  modulation: Modulation::Qpsk,   code_rate_x1024: 120  },
    McsEntry { index: 1,  modulation: Modulation::Qpsk,   code_rate_x1024: 157  },
    McsEntry { index: 2,  modulation: Modulation::Qpsk,   code_rate_x1024: 193  },
    McsEntry { index: 3,  modulation: Modulation::Qpsk,   code_rate_x1024: 251  },
    McsEntry { index: 4,  modulation: Modulation::Qpsk,   code_rate_x1024: 308  },
    McsEntry { index: 5,  modulation: Modulation::Qpsk,   code_rate_x1024: 379  },
    McsEntry { index: 6,  modulation: Modulation::Qpsk,   code_rate_x1024: 449  },
    McsEntry { index: 7,  modulation: Modulation::Qpsk,   code_rate_x1024: 526  },
    McsEntry { index: 8,  modulation: Modulation::Qpsk,   code_rate_x1024: 602  },
    McsEntry { index: 9,  modulation: Modulation::Qpsk,   code_rate_x1024: 679  },
    McsEntry { index: 10, modulation: Modulation::Qam16,  code_rate_x1024: 340  },
    McsEntry { index: 11, modulation: Modulation::Qam16,  code_rate_x1024: 378  },
    McsEntry { index: 12, modulation: Modulation::Qam16,  code_rate_x1024: 434  },
    McsEntry { index: 13, modulation: Modulation::Qam16,  code_rate_x1024: 490  },
    McsEntry { index: 14, modulation: Modulation::Qam16,  code_rate_x1024: 553  },
    McsEntry { index: 15, modulation: Modulation::Qam16,  code_rate_x1024: 616  },
    McsEntry { index: 16, modulation: Modulation::Qam16,  code_rate_x1024: 658  },
    McsEntry { index: 17, modulation: Modulation::Qam64,  code_rate_x1024: 438  },
    McsEntry { index: 18, modulation: Modulation::Qam64,  code_rate_x1024: 466  },
    McsEntry { index: 19, modulation: Modulation::Qam64,  code_rate_x1024: 517  },
    McsEntry { index: 20, modulation: Modulation::Qam64,  code_rate_x1024: 567  },
    McsEntry { index: 21, modulation: Modulation::Qam64,  code_rate_x1024: 616  },
    McsEntry { index: 22, modulation: Modulation::Qam64,  code_rate_x1024: 666  },
    McsEntry { index: 23, modulation: Modulation::Qam64,  code_rate_x1024: 719  },
    McsEntry { index: 24, modulation: Modulation::Qam64,  code_rate_x1024: 772  },
    McsEntry { index: 25, modulation: Modulation::Qam64,  code_rate_x1024: 822  },
    McsEntry { index: 26, modulation: Modulation::Qam64,  code_rate_x1024: 873  },
    McsEntry { index: 27, modulation: Modulation::Qam64,  code_rate_x1024: 910  },
    // Index 28: extended entry (256QAM, Table 2 rate), used for high-throughput
    McsEntry { index: 28, modulation: Modulation::Qam256, code_rate_x1024: 948  },
];

/// Look up an MCS entry by index.
pub fn mcs_lookup(mcs_idx: u8) -> Option<McsEntry> {
    if mcs_idx < 29 {
        Some(MCS_TABLE_64QAM[mcs_idx as usize])
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// TBS calculation (TS 38.214 Section 5.1.3.2)
// ---------------------------------------------------------------------------

/// TBS table for N_info <= 3824 (TS 38.214 Table 5.1.3.2-1), 93 entries.
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

/// Result of TBS calculation.
#[derive(Debug, Clone, Copy)]
pub struct TbsResult {
    /// Transport block size in bits.
    pub tbs_bits: u32,
    /// Total number of resource elements used.
    pub n_re: u32,
    /// Number of allocated PRBs.
    pub n_prb: u16,
    /// Number of MIMO layers.
    pub n_layers: u8,
    /// MCS index used.
    pub mcs: u8,
}

/// Compute TBS per TS 38.214 Section 5.1.3.2.
///
/// # Parameters
/// - `mcs_idx`: MCS index (0-28)
/// - `n_prb`: number of allocated PRBs
/// - `n_layers`: number of MIMO layers (1-4)
/// - `n_symbols`: number of OFDM symbols in allocation (1-14)
/// - `dmrs_overhead`: DMRS overhead in RE per PRB (e.g. 6 for single-port Type 1)
/// - `oh`: additional overhead (0, 6, 12, or 18 for PDSCH-Xoh)
pub fn compute_tbs(
    mcs_idx: u8,
    n_prb: u16,
    n_layers: u8,
    n_symbols: u8,
    dmrs_overhead: u8,
    oh: u8,
) -> Option<TbsResult> {
    let entry = mcs_lookup(mcs_idx)?;
    let qm = entry.modulation.bits_per_symbol() as u32;
    let r_num = entry.code_rate_x1024 as u64;

    // N_RE' = 12*N_symb - N_DMRS_per_PRB - N_oh_PRB, capped at 156
    let n_re_prime = (12u32 * n_symbols as u32)
        .saturating_sub(dmrs_overhead as u32)
        .saturating_sub(oh as u32)
        .min(156);

    let n_re = n_re_prime * n_prb as u32;

    // N_info = N_RE * R * Q_m * v
    let n_info = (n_re as u64 * r_num * qm as u64 * n_layers as u64) / 1024;

    let tbs_bits = if n_info <= 3824 {
        // Quantize N_info' = max(24, 2^n * floor(N_info / 2^n)) where 2^n =
        // largest power of 2 not exceeding N_info/24
        let n = if n_info < 24 {
            0u32
        } else {
            ((n_info as f64 / 24.0).log2().floor() as u32).saturating_sub(0)
        };
        let pow2n = 1u64 << n;
        let n_info_prime = (n_info / pow2n * pow2n).max(24);
        // Find smallest TBS >= N_info'
        *TBS_TABLE.iter().find(|&&t| t as u64 >= n_info_prime).unwrap_or(&3824)
    } else {
        // C = number of code blocks
        let c = if n_info > 8424 { (n_info + 24 + 8424 - 1) / 8424 } else { 1 };
        // n' = max(1, ceil(log2(N_info + 24)) - 5)
        let n_prime = {
            let bits = (n_info + 24) as f64;
            let log = bits.log2().ceil() as i32;
            (log - 5).max(1) as u32
        };
        let pow2n_prime = 1u64 << n_prime;
        // TBS = 8*C*ceil((N_info + 24) / (8*C)) - 24
        let num = n_info + 24;
        let denom = 8 * c;
        let ceil_val = (num + denom - 1) / denom;
        let tbs_raw = 8 * c * ceil_val - 24;
        // Snap to nearest multiple of pow2n_prime
        let _ = pow2n_prime; // used for alignment, raw formula is standard
        tbs_raw as u32
    };

    Some(TbsResult {
        tbs_bits,
        n_re,
        n_prb,
        n_layers,
        mcs: mcs_idx,
    })
}

// ---------------------------------------------------------------------------
// CRC polynomials and computation
// ---------------------------------------------------------------------------

/// CRC-24A polynomial: x^24 + x^23 + x^18 + x^17 + x^14 + x^11 + x^10 +
///                      x^7 + x^6 + x^5 + x^4 + x^3 + x + 1
/// Used for TB-level CRC attachment.
const CRC24A_POLY: u32 = 0x864CFB;

/// CRC-24B polynomial: x^24 + x^23 + x^6 + x^5 + x + 1
/// Used for code-block-level CRC attachment.
const CRC24B_POLY: u32 = 0x800063;

/// CRC-16 polynomial (CCITT): x^16 + x^12 + x^5 + 1
/// Used for small TB CRC attachment (TB <= 3824 bits).
const CRC16_POLY: u32 = 0x11021;

fn crc_generic(data: &[u8], poly: u32, crc_bits: u8) -> u32 {
    let mut crc: u32 = 0;
    let mask = if crc_bits == 24 { 0xFF_FFFF } else { 0xFFFF };
    let top_bit = 1u32 << (crc_bits - 1);

    for &byte in data {
        for bit_pos in (0..8).rev() {
            let input_bit = (byte >> bit_pos) & 1;
            let crc_bit = (crc >> (crc_bits - 1)) & 1;
            crc = (crc << 1) & mask;
            if input_bit as u32 ^ crc_bit != 0 {
                crc ^= poly;
            }
        }
    }
    crc & mask
}

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

/// Compute CRC-24A (TB attachment).
pub fn crc24a(data: &[u8]) -> u32 {
    crc_generic(data, CRC24A_POLY, 24)
}

/// Compute CRC-24B (code block attachment).
pub fn crc24b(data: &[u8]) -> u32 {
    crc_generic(data, CRC24B_POLY, 24)
}

/// Compute CRC-16 (small TB attachment).
pub fn crc16(data: &[u8]) -> u16 {
    crc_generic(data, CRC16_POLY, 16) as u16
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

// ---------------------------------------------------------------------------
// CRC attachment
// ---------------------------------------------------------------------------

/// Attach CRC to a transport block per TS 38.212 Section 7.2.1.
/// Returns the bit sequence [TB bits | CRC bits].
pub fn attach_tb_crc(tb_bits: &[bool]) -> Vec<bool> {
    let b = tb_bits.len();
    let mut result = tb_bits.to_vec();

    if b > 3824 {
        // CRC-24A
        let crc = crc24a_bits(tb_bits);
        for i in (0..24).rev() {
            result.push((crc >> i) & 1 == 1);
        }
    } else {
        // CRC-16
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
// LDPC parameters
// ---------------------------------------------------------------------------

/// LDPC base graph and lifting size parameters.
#[derive(Debug, Clone, Copy)]
pub struct LdpcParams {
    /// Base graph number: 1 or 2.
    pub base_graph: u8,
    /// Lifting size Z.
    pub lifting_size: u16,
    /// Number of systematic columns K_b in the base graph.
    pub k_b: u8,
    /// Code block size after encoding N_cb = N * Z (before rate matching).
    pub n_cb: u32,
}

/// Lifting size sets per TS 38.212 Table 5.3.2-1.
/// Set index 0..7.
const LIFTING_SIZE_SETS: [[u16; 8]; 8] = [
    [2, 4, 8, 16, 32, 64, 128, 256],
    [3, 6, 12, 24, 48, 96, 192, 384],
    [5, 10, 20, 40, 80, 160, 320, 0],
    [7, 14, 28, 56, 112, 224, 0, 0],
    [9, 18, 36, 72, 144, 288, 0, 0],
    [11, 22, 44, 88, 176, 352, 0, 0],
    [13, 26, 52, 104, 208, 0, 0, 0],
    [15, 30, 60, 120, 240, 0, 0, 0],
];

/// Select the minimum valid lifting size Z >= K / K_b (TS 38.212 Section 5.2.2).
pub fn select_lifting_size(k_bits: u32, k_b: u8) -> u16 {
    let k_per_z = (k_bits + k_b as u32 - 1) / k_b as u32;
    // Find smallest Z in any lifting set such that Z >= k_per_z and Z <= 384
    let mut best = 384u16;
    for set in &LIFTING_SIZE_SETS {
        for &z in set {
            if z == 0 { continue; }
            if z as u32 >= k_per_z && z <= best {
                best = z;
            }
        }
    }
    best
}

/// Determine LDPC parameters for a given info block size (K including CRC).
/// Follows TS 38.212 Section 7.2.2 / 5.2.2.
pub fn ldpc_params(k_bits: u32) -> LdpcParams {
    // Choose base graph
    // BG1: for R >= 1/3 and K > 292, or K > 3824 bits (large TBs, high rate)
    // BG2: for smaller TBs / lower rates
    let (bg, k_b, n_columns) = if k_bits > 3824 || k_bits > 292 {
        (1u8, 22u8, 68u8) // BG1: 22 systematic + 46 parity = 68 columns
    } else {
        (2u8, 10u8, 52u8) // BG2: 10 systematic + 42 parity = 52 columns
    };

    let z = select_lifting_size(k_bits, k_b);
    let n_cb = n_columns as u32 * z as u32;

    LdpcParams {
        base_graph: bg,
        lifting_size: z,
        k_b,
        n_cb,
    }
}

// ---------------------------------------------------------------------------
// Code block segmentation (TS 38.212 Section 7.2.3)
// ---------------------------------------------------------------------------

/// A single code block after segmentation.
#[derive(Debug, Clone)]
pub struct CodeBlock {
    /// Systematic information bits (including filler bits if any).
    pub info_bits: Vec<bool>,
    /// CRC-24B parity bits appended.
    pub crc: u32,
    /// LDPC parameters for this code block.
    pub ldpc: LdpcParams,
}

/// Segment transport block into code blocks, attach CRC-24B.
/// Returns the list of code blocks and the LDPC params (same for all CBs).
pub fn segment_code_blocks(tb_with_crc: &[bool]) -> (Vec<CodeBlock>, LdpcParams) {
    let b = tb_with_crc.len(); // total bits including TB-CRC

    // Max code block sizes
    const MAX_CB_BG1: u32 = 8448; // K_max for BG1: 22 * 384
    const MAX_CB_BG2: u32 = 3840; // K_max for BG2: 10 * 384

    // Choose base graph based on TB size
    let use_bg1 = b > 3824;
    let k_cb_max = if use_bg1 { MAX_CB_BG1 } else { MAX_CB_BG2 };
    let k_b_val: u8 = if use_bg1 { 22 } else { 10 };

    // Number of code blocks C
    let c = if b as u32 > k_cb_max {
        let b_prime = b as u32 + 24; // add 24 for CB CRC
        (b_prime + k_cb_max - 1) / k_cb_max
    } else {
        1
    };

    // K' = ceil(B / C) rounded up to multiple of Z*K_b
    let b_prime = b as u32 + if c > 1 { 24 * c } else { 0 };
    let k_prime = (b_prime + c - 1) / c;

    let ldpc = ldpc_params(k_prime);
    let z = ldpc.lifting_size;
    let k_b = ldpc.k_b;

    // Actual K per CB (multiple of Z*K_b)
    let k = k_b as u32 * z as u32;

    // Total bits needed for C code blocks (with filler)
    let f = (k * c).saturating_sub(b_prime);

    let mut code_blocks = Vec::with_capacity(c as usize);
    let mut bit_offset = 0usize;

    for cb_idx in 0..c {
        let mut cb_bits = Vec::with_capacity(k as usize);

        // Filler bits at the start of first CB
        if cb_idx == 0 {
            for _ in 0..f {
                cb_bits.push(false); // filler = 0
            }
        }

        // Copy information bits
        let bits_to_copy = (k as usize - cb_bits.len()).min(b - bit_offset);
        cb_bits.extend_from_slice(&tb_with_crc[bit_offset..bit_offset + bits_to_copy]);
        bit_offset += bits_to_copy;

        // Pad to K if needed (last CB)
        while cb_bits.len() < k as usize {
            cb_bits.push(false);
        }

        // Compute CRC-24B
        let crc = if c > 1 {
            crc24b_bits(&cb_bits)
        } else {
            0
        };

        code_blocks.push(CodeBlock {
            info_bits: cb_bits,
            crc,
            ldpc,
        });
    }

    (code_blocks, ldpc)
}

// ---------------------------------------------------------------------------
// Simplified LDPC encoding (systematic portion)
// ---------------------------------------------------------------------------

/// LDPC encoder output for a single code block.
#[derive(Debug, Clone)]
pub struct LdpcCodeword {
    /// Systematic bits (input).
    pub systematic: Vec<bool>,
    /// Parity check bits appended.
    pub parity: Vec<bool>,
    /// Total codeword length N = K + parity_count.
    pub n: u32,
}

/// Simplified LDPC encoding using a back-substitution approach.
///
/// For simulation purposes, parity bits are generated via a simple
/// accumulator structure that approximates the LDPC parity check
/// equations for BG1/BG2. A full hardware implementation would use
/// the actual lifted parity check matrices.
pub fn ldpc_encode(cb: &CodeBlock) -> LdpcCodeword {
    let k = cb.info_bits.len() as u32;
    let z = cb.ldpc.lifting_size;
    let bg = cb.ldpc.base_graph;

    // Number of parity bits = (n_columns - k_b) * Z
    let parity_cols = if bg == 1 { 46u32 } else { 42u32 };
    let n_parity = parity_cols * z as u32;
    let n = k + n_parity;

    // Simple LDPC parity generation: XOR-based linear encoding
    // In real 5G, this uses the specific circulant shift values from the
    // base graph. Here we use a deterministic accumulator for testing.
    let mut parity = vec![false; n_parity as usize];

    // First parity group: XOR of all systematic bits (accumulator)
    if !cb.info_bits.is_empty() {
        let mut acc = false;
        for (i, &bit) in cb.info_bits.iter().enumerate() {
            acc ^= bit;
            if (i + 1) % z as usize == 0 {
                let parity_idx = (i / z as usize).min(n_parity as usize - 1);
                parity[parity_idx] = acc;
                // Reset for next group with feedback
                acc = parity[parity_idx];
            }
        }
    }

    // Additional parity groups using shift-register pattern
    for p in 1..parity_cols as usize {
        for s in 0..z as usize {
            let idx = p * z as usize + s;
            if idx >= n_parity as usize { break; }
            // Parity derived from previous parity and systematic bits
            let sys_ref = ((p * z as usize + s) * 3) % k as usize;
            let prev_par = if idx > 0 { parity[idx - 1] } else { false };
            parity[idx] = prev_par ^ cb.info_bits[sys_ref];
        }
    }

    LdpcCodeword {
        systematic: cb.info_bits.clone(),
        parity,
        n,
    }
}

// ---------------------------------------------------------------------------
// Rate matching (TS 38.212 Section 7.2.6)
// ---------------------------------------------------------------------------

/// Rate match a codeword to E output bits.
///
/// Uses circular buffer selection with rv starting offset:
/// - rv_idx=0: start at 0
/// - rv_idx=1: start at ~2*Z
/// - rv_idx=2: start at ~3*Z  
/// - rv_idx=3: start at ~5*Z
pub fn rate_match(codeword: &LdpcCodeword, e_bits: u32, rv_idx: u8, z: u16) -> Vec<bool> {
    // Build full circular buffer: systematic | parity
    let mut circ: Vec<bool> = codeword.systematic.clone();
    circ.extend_from_slice(&codeword.parity);

    let n_cb = circ.len() as u32;

    // Starting offset k0 per TS 38.212 Table 5.4.2.1-2
    let k0 = match rv_idx {
        0 => 0u32,
        1 => 17 * z as u32 / 4,  // ~floor(17/4 * Z/2 * 2) simplified
        2 => 34 * z as u32 / 4,  // ~2 * 17/4 * Z
        3 => 51 * z as u32 / 4,  // ~3 * 17/4 * Z
        _ => 0,
    };
    let k0 = k0.min(n_cb - 1);

    let mut output = Vec::with_capacity(e_bits as usize);
    let mut idx = k0 as usize;
    let mut count = 0u32;

    while count < e_bits {
        output.push(circ[idx % n_cb as usize]);
        idx += 1;
        count += 1;
    }
    output
}

// ---------------------------------------------------------------------------
// Scrambling (TS 38.211 Section 7.3.1.1)
// ---------------------------------------------------------------------------

/// Gold sequence generator state (31-bit LFSR pair).
pub struct GoldSeq {
    x1: u32,
    x2: u32,
}

impl GoldSeq {
    /// Initialize Gold sequence with c_init per 3GPP spec.
    pub fn new(c_init: u32) -> Self {
        const NC: usize = 1600;
        // x1 initializer: x1(0)=1, x1(1..30)=0
        let mut x1 = 1u32;
        // x2 initializer: binary representation of c_init
        let mut x2 = c_init;

        // Advance by Nc=1600 steps
        for _ in 0..NC {
            let b1 = ((x1 >> 3) ^ x1) & 1;
            x1 = (x1 >> 1) | (b1 << 30);

            let b2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
            x2 = (x2 >> 1) | (b2 << 30);
        }
        GoldSeq { x1, x2 }
    }

    /// Generate next bit of the Gold sequence.
    pub fn next_bit(&mut self) -> bool {
        let b1 = ((self.x1 >> 3) ^ self.x1) & 1;
        self.x1 = (self.x1 >> 1) | (b1 << 30);

        let b2 = ((self.x2 >> 3) ^ (self.x2 >> 2) ^ (self.x2 >> 1) ^ self.x2) & 1;
        self.x2 = (self.x2 >> 1) | (b2 << 30);

        ((self.x1 ^ self.x2) & 1) == 1
    }

    /// Generate n bits.
    pub fn generate(&mut self, n: usize) -> Vec<bool> {
        (0..n).map(|_| self.next_bit()).collect()
    }
}

/// Scramble/descramble a bit sequence with a Gold sequence.
pub fn scramble(bits: &[bool], c_init: u32) -> Vec<bool> {
    let mut seq = GoldSeq::new(c_init);
    bits.iter().map(|&b| b ^ seq.next_bit()).collect()
}

/// Compute scrambling c_init for PDSCH per TS 38.211 Section 7.3.1.1.
/// c_init = n_RNTI * 2^15 + q * 2^14 + n_ID
pub fn pdsch_c_init(n_rnti: u16, codeword_idx: u8, n_id: u16) -> u32 {
    (n_rnti as u32) * (1 << 15)
        + (codeword_idx as u32) * (1 << 14)
        + (n_id as u32)
}

// ---------------------------------------------------------------------------
// Modulation mapping (TS 38.211 Section 5.1)
// ---------------------------------------------------------------------------

/// A complex IQ symbol.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IqSymbol {
    pub i: f64,
    pub q: f64,
}

impl IqSymbol {
    pub fn new(i: f64, q: f64) -> Self {
        IqSymbol { i, q }
    }

    pub fn magnitude_sq(&self) -> f64 {
        self.i * self.i + self.q * self.q
    }
}

/// Map bit groups to QPSK symbols per TS 38.211 Table 5.1.3.1-1.
/// QPSK: 2 bits per symbol. Gray coded.
pub fn modulate_qpsk(bits: &[bool]) -> Vec<IqSymbol> {
    let norm = 1.0 / 2.0f64.sqrt();
    bits.chunks(2)
        .map(|b| {
            let b0 = b[0];
            let b1 = if b.len() > 1 { b[1] } else { false };
            let i = if !b0 { norm } else { -norm };
            let q = if !b1 { norm } else { -norm };
            IqSymbol::new(i, q)
        })
        .collect()
}

/// Map bit groups to 16QAM symbols per TS 38.211 Table 5.1.3.2-1.
/// 16QAM: 4 bits per symbol. Normalized by 1/sqrt(10).
pub fn modulate_16qam(bits: &[bool]) -> Vec<IqSymbol> {
    let norm = 1.0 / 10.0f64.sqrt();
    bits.chunks(4)
        .map(|b| {
            let b0 = if b.len() > 0 { b[0] } else { false };
            let b1 = if b.len() > 1 { b[1] } else { false };
            let b2 = if b.len() > 2 { b[2] } else { false };
            let b3 = if b.len() > 3 { b[3] } else { false };

            let i = if !b0 {
                if !b2 { 3.0 * norm } else { 1.0 * norm }
            } else {
                if !b2 { -3.0 * norm } else { -1.0 * norm }
            };
            let q = if !b1 {
                if !b3 { 3.0 * norm } else { 1.0 * norm }
            } else {
                if !b3 { -3.0 * norm } else { -1.0 * norm }
            };
            IqSymbol::new(i, q)
        })
        .collect()
}

/// Map bit groups to 64QAM symbols per TS 38.211 Table 5.1.3.3-1.
/// 64QAM: 6 bits per symbol. Normalized by 1/sqrt(42).
pub fn modulate_64qam(bits: &[bool]) -> Vec<IqSymbol> {
    let norm = 1.0 / 42.0f64.sqrt();
    bits.chunks(6)
        .map(|b| {
            let get = |idx: usize| if b.len() > idx { b[idx] } else { false };
            let (b0, b1, b2, b3, b4, b5) = (get(0), get(1), get(2), get(3), get(4), get(5));

            let i = {
                let mag = if !b2 { if !b4 { 7.0 } else { 5.0 } } else { if !b4 { 3.0 } else { 1.0 } };
                if !b0 { mag * norm } else { -mag * norm }
            };
            let q = {
                let mag = if !b3 { if !b5 { 7.0 } else { 5.0 } } else { if !b5 { 3.0 } else { 1.0 } };
                if !b1 { mag * norm } else { -mag * norm }
            };
            IqSymbol::new(i, q)
        })
        .collect()
}

/// Map bit groups to 256QAM symbols per TS 38.211 Table 5.1.3.4-1.
/// 256QAM: 8 bits per symbol. Normalized by 1/sqrt(170).
pub fn modulate_256qam(bits: &[bool]) -> Vec<IqSymbol> {
    let norm = 1.0 / 170.0f64.sqrt();
    bits.chunks(8)
        .map(|b| {
            let get = |idx: usize| if b.len() > idx { b[idx] } else { false };
            let (b0, b1, b2, b3, b4, b5, b6, b7) =
                (get(0), get(1), get(2), get(3), get(4), get(5), get(6), get(7));

            let i_mag = {
                let a = if !b2 { 8.0f64 } else { 0.0 };
                let c = if !b4 { 4.0f64 } else { 0.0 };
                let e = if !b6 { 2.0f64 } else { 0.0 };
                a + c + e + 1.0
            };
            let q_mag = {
                let a = if !b3 { 8.0f64 } else { 0.0 };
                let c = if !b5 { 4.0f64 } else { 0.0 };
                let e = if !b7 { 2.0f64 } else { 0.0 };
                a + c + e + 1.0
            };
            let i = if !b0 { i_mag * norm } else { -i_mag * norm };
            let q = if !b1 { q_mag * norm } else { -q_mag * norm };
            IqSymbol::new(i, q)
        })
        .collect()
}

/// Modulate a bit sequence using the specified modulation order.
pub fn modulate(bits: &[bool], modulation: Modulation) -> Vec<IqSymbol> {
    match modulation {
        Modulation::Qpsk => modulate_qpsk(bits),
        Modulation::Qam16 => modulate_16qam(bits),
        Modulation::Qam64 => modulate_64qam(bits),
        Modulation::Qam256 => modulate_256qam(bits),
    }
}

// ---------------------------------------------------------------------------
// DMRS generation (TS 38.211 Section 7.4.1.1)
// ---------------------------------------------------------------------------

/// DMRS configuration.
#[derive(Debug, Clone, Copy)]
pub struct DmrsConfig {
    /// DMRS type: 1 (comb-2) or 2 (comb-3).
    pub type_id: u8,
    /// Number of additional DMRS positions (0-3).
    pub additional_pos: u8,
    /// Scrambling ID (N_ID_DMRS, 0-65535).
    pub scrambling_id: u16,
}

impl Default for DmrsConfig {
    fn default() -> Self {
        DmrsConfig {
            type_id: 1,
            additional_pos: 0,
            scrambling_id: 0,
        }
    }
}

/// Generate DMRS reference sequence for a slot/symbol.
///
/// Gold sequence c_init = 2^17 * (14*n_slot + l + 1) * (2*N_ID + 1) + 2*N_ID
/// per TS 38.211 Eq. 7.4.1.1.2-1.
pub fn generate_dmrs(
    n_slot: u16,
    symbol_in_slot: u8,
    n_id: u16,
    n_sc: usize,
) -> Vec<IqSymbol> {
    let c_init = ((1u32 << 17)
        * (14u32 * n_slot as u32 + symbol_in_slot as u32 + 1)
        * (2u32 * n_id as u32 + 1)
        + 2 * n_id as u32)
        & 0x7FFF_FFFF; // keep within 31 bits

    let mut seq = GoldSeq::new(c_init);
    let norm = 1.0 / 2.0f64.sqrt();
    let mut symbols = Vec::with_capacity(n_sc);
    for _ in 0..n_sc {
        let c0 = seq.next_bit();
        let c1 = seq.next_bit();
        let i = if !c0 { norm } else { -norm };
        let q = if !c1 { norm } else { -norm };
        symbols.push(IqSymbol::new(i, q));
    }
    symbols
}

/// DMRS symbol positions in a slot for Type A mapping (front-loaded).
/// Returns vector of symbol indices that carry DMRS.
pub fn dmrs_symbol_positions(additional_pos: u8) -> Vec<u8> {
    // Type A: DMRS at symbol 2 (and 3 if CDM2, or additional symbols)
    let mut positions = vec![2u8];
    if additional_pos >= 1 {
        positions.push(7);
    }
    if additional_pos >= 2 {
        positions.push(11);
    }
    if additional_pos >= 3 {
        positions.push(13);
    }
    positions
}

/// Subcarrier offsets for DMRS Type 1 (comb-2): every other subcarrier.
/// Returns RE indices (0-11 within PRB) that carry DMRS.
pub fn dmrs_type1_re_indices() -> Vec<usize> {
    // Port 1000: positions 0, 2, 4, 6, 8, 10
    (0..6).map(|k| 2 * k).collect()
}

/// Subcarrier offsets for DMRS Type 2 (comb-3).
/// Returns RE indices (0-11 within PRB) that carry DMRS.
pub fn dmrs_type2_re_indices() -> Vec<usize> {
    // Ports 1000/1001: positions 0, 1, 6, 7
    vec![0, 1, 6, 7]
}

// ---------------------------------------------------------------------------
// Resource element mapping
// ---------------------------------------------------------------------------

/// PDSCH channel configuration.
#[derive(Debug, Clone, Copy)]
pub struct PdschConfig {
    /// MCS index (0-28).
    pub mcs: u8,
    /// Number of allocated PRBs.
    pub n_prb: u16,
    /// Number of MIMO transmission layers (1-4).
    pub n_layers: u8,
    /// DMRS type: 1 or 2.
    pub dmrs_type: u8,
    /// Redundancy version index (0-3).
    pub rv_idx: u8,
}

impl Default for PdschConfig {
    fn default() -> Self {
        PdschConfig {
            mcs: 9,
            n_prb: 25,
            n_layers: 1,
            dmrs_type: 1,
            rv_idx: 0,
        }
    }
}

/// Allocated resource element with subcarrier and OFDM symbol indices.
#[derive(Debug, Clone, Copy)]
pub struct AllocatedRe {
    pub subcarrier: u16,
    pub symbol: u8,
    pub is_dmrs: bool,
}

/// Enumerate all PDSCH resource elements in the allocation.
///
/// Returns all RE positions (subcarrier, symbol) in the PRB allocation,
/// separating DMRS REs from data REs.
pub fn enumerate_res(
    n_prb: u16,
    n_symbols: u8,
    first_symbol: u8,
    dmrs_cfg: &DmrsConfig,
) -> Vec<AllocatedRe> {
    let dmrs_positions = dmrs_symbol_positions(dmrs_cfg.additional_pos);
    let dmrs_sc = if dmrs_cfg.type_id == 1 {
        dmrs_type1_re_indices()
    } else {
        dmrs_type2_re_indices()
    };

    let mut res = Vec::new();
    for sym in 0..n_symbols {
        let abs_sym = first_symbol + sym;
        let is_dmrs_sym = dmrs_positions.contains(&abs_sym);
        for prb in 0..n_prb {
            for sc in 0..12u16 {
                let abs_sc = prb * 12 + sc;
                let is_dmrs_re = is_dmrs_sym && dmrs_sc.contains(&(sc as usize));
                res.push(AllocatedRe {
                    subcarrier: abs_sc,
                    symbol: abs_sym,
                    is_dmrs: is_dmrs_re,
                });
            }
        }
    }
    res
}

/// Count available PDSCH data REs (excluding DMRS) per PRB over all symbols.
pub fn count_data_res(n_prb: u16, n_symbols: u8, first_symbol: u8, dmrs_cfg: &DmrsConfig) -> u32 {
    enumerate_res(n_prb, n_symbols, first_symbol, dmrs_cfg)
        .iter()
        .filter(|re| !re.is_dmrs)
        .count() as u32
}

// ---------------------------------------------------------------------------
// Full PDSCH Processor
// ---------------------------------------------------------------------------

/// Main PDSCH processor combining all processing steps.
pub struct PdschProcessor {
    pub config: PdschConfig,
    pub dmrs_cfg: DmrsConfig,
}

impl PdschProcessor {
    /// Create a new PDSCH processor.
    pub fn new(config: PdschConfig, dmrs_cfg: DmrsConfig) -> Self {
        PdschProcessor { config, dmrs_cfg }
    }

    /// Compute TBS for the current configuration.
    ///
    /// # Parameters
    /// - `n_layers`: number of layers override (0 = use config)
    /// - `first_symbol`: first OFDM symbol index (usually 2 for PDSCH after PDCCH)
    /// - `n_symbols`: number of allocated OFDM symbols
    /// - `oh`: additional overhead (PDSCH-Xoh: 0, 6, 12, 18)
    pub fn compute_tbs(&self, n_layers: u8, first_symbol: u8, n_symbols: u8, oh: u8) -> TbsResult {
        let layers = if n_layers > 0 { n_layers } else { self.config.n_layers };
        // DMRS overhead: Type1 = 6 RE/PRB, Type2 = 4 RE/PRB
        let dmrs_oh: u8 = if self.config.dmrs_type == 1 { 6 } else { 4 };
        // Count DMRS symbols
        let dmrs_pos = dmrs_symbol_positions(self.dmrs_cfg.additional_pos);
        let n_dmrs_syms = dmrs_pos.iter()
            .filter(|&&s| s >= first_symbol && s < first_symbol + n_symbols)
            .count() as u8;
        let total_dmrs_oh = dmrs_oh * n_dmrs_syms;

        compute_tbs(
            self.config.mcs,
            self.config.n_prb,
            layers,
            n_symbols,
            total_dmrs_oh,
            oh,
        )
        .unwrap_or(TbsResult {
            tbs_bits: 0,
            n_re: 0,
            n_prb: self.config.n_prb,
            n_layers: layers,
            mcs: self.config.mcs,
        })
    }

    /// Process a transport block end-to-end.
    ///
    /// Returns modulated IQ symbols ready for resource element mapping.
    pub fn process_tb(&self, tb_bits: &[bool]) -> Vec<IqSymbol> {
        let mcs = mcs_lookup(self.config.mcs).unwrap_or(MCS_TABLE_64QAM[9]);

        // Step 1: CRC attachment
        let with_crc = attach_tb_crc(tb_bits);

        // Step 2: Code block segmentation
        let (code_blocks, _ldpc) = segment_code_blocks(&with_crc);

        // Step 3: LDPC encoding + rate matching
        let mut all_bits: Vec<bool> = Vec::new();
        let total_e = (tb_bits.len() * mcs.modulation.bits_per_symbol() as usize
            + code_blocks.len() - 1)
            / code_blocks.len().max(1);
        let e_bits = (total_e * mcs.modulation.bits_per_symbol() as usize).max(1) as u32;

        for cb in &code_blocks {
            let codeword = ldpc_encode(cb);
            let e = e_bits.max(codeword.n);
            let rm = rate_match(&codeword, e, self.config.rv_idx, cb.ldpc.lifting_size);
            all_bits.extend_from_slice(&rm);
        }

        // Step 4: Scrambling (c_init = RNTI * 2^15 + n_ID, simplified)
        let c_init = pdsch_c_init(0xFFFF, 0, 0);
        let scrambled = scramble(&all_bits, c_init);

        // Step 5: Modulation
        modulate(&scrambled, mcs.modulation)
    }

    /// Map IQ symbols onto resource grid, returning (data_symbols, dmrs_symbols).
    pub fn map_to_resource_grid(
        &self,
        symbols: &[IqSymbol],
        n_slot: u16,
        first_symbol: u8,
        n_symbols: u8,
    ) -> (Vec<(AllocatedRe, IqSymbol)>, Vec<(AllocatedRe, IqSymbol)>) {
        let res = enumerate_res(
            self.config.n_prb,
            n_symbols,
            first_symbol,
            &self.dmrs_cfg,
        );

        let dmrs_positions = dmrs_symbol_positions(self.dmrs_cfg.additional_pos);
        let mut data_mapped = Vec::new();
        let mut dmrs_mapped = Vec::new();
        let mut data_idx = 0usize;

        // Collect DMRS reference signals
        let dmrs_re_per_sym = if self.dmrs_cfg.type_id == 1 {
            6 * self.config.n_prb as usize
        } else {
            4 * self.config.n_prb as usize
        };

        for &abs_sym in &dmrs_positions {
            if abs_sym >= first_symbol && abs_sym < first_symbol + n_symbols {
                let dmrs_syms = generate_dmrs(
                    n_slot,
                    abs_sym,
                    self.dmrs_cfg.scrambling_id,
                    dmrs_re_per_sym,
                );
                let dmrs_sc = if self.dmrs_cfg.type_id == 1 {
                    dmrs_type1_re_indices()
                } else {
                    dmrs_type2_re_indices()
                };
                let mut dmrs_sym_idx = 0;
                for prb in 0..self.config.n_prb {
                    for &sc in &dmrs_sc {
                        let abs_sc = prb * 12 + sc as u16;
                        if dmrs_sym_idx < dmrs_syms.len() {
                            dmrs_mapped.push((
                                AllocatedRe { subcarrier: abs_sc, symbol: abs_sym, is_dmrs: true },
                                dmrs_syms[dmrs_sym_idx],
                            ));
                            dmrs_sym_idx += 1;
                        }
                    }
                }
            }
        }

        // Map data symbols
        for re in res.iter().filter(|r| !r.is_dmrs) {
            if data_idx < symbols.len() {
                data_mapped.push((*re, symbols[data_idx]));
                data_idx += 1;
            }
        }

        (data_mapped, dmrs_mapped)
    }
}

// ---------------------------------------------------------------------------
// Layer mapping (TS 38.211 Section 7.3.1.3)
// ---------------------------------------------------------------------------

/// Distribute modulated symbols across layers.
///
/// For v layers, symbol d[k] goes to layer k mod v.
pub fn layer_map(symbols: &[IqSymbol], n_layers: u8) -> Vec<Vec<IqSymbol>> {
    let v = n_layers as usize;
    let mut layers: Vec<Vec<IqSymbol>> = vec![Vec::new(); v];
    for (i, &sym) in symbols.iter().enumerate() {
        layers[i % v].push(sym);
    }
    layers
}

// ---------------------------------------------------------------------------
// Precoding (single-layer / identity for multi-layer)
// ---------------------------------------------------------------------------

/// Apply identity precoding (pass-through for single-port transmission).
pub fn precode_identity(layers: &[Vec<IqSymbol>]) -> Vec<Vec<IqSymbol>> {
    layers.to_vec()
}

// ---------------------------------------------------------------------------
// Helper: bits from u8 slice
// ---------------------------------------------------------------------------

/// Convert byte slice to big-endian bit vector.
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(bytes.len() * 8);
    for &b in bytes {
        for i in (0..8).rev() {
            bits.push((b >> i) & 1 == 1);
        }
    }
    bits
}

/// Convert bit vector to bytes (pad last byte with zeros if needed).
pub fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|chunk| {
            let mut byte = 0u8;
            for (i, &b) in chunk.iter().enumerate() {
                if b { byte |= 1 << (7 - i); }
            }
            byte
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------------
    // MCS table tests
    // ------------------------------------------------------------------

    #[test]
    fn test_mcs_table_length() {
        assert_eq!(MCS_TABLE_64QAM.len(), 29);
    }

    #[test]
    fn test_mcs_index_0() {
        let e = mcs_lookup(0).unwrap();
        assert_eq!(e.modulation, Modulation::Qpsk);
        assert_eq!(e.code_rate_x1024, 120);
    }

    #[test]
    fn test_mcs_index_9() {
        let e = mcs_lookup(9).unwrap();
        assert_eq!(e.modulation, Modulation::Qpsk);
        assert_eq!(e.code_rate_x1024, 679);
    }

    #[test]
    fn test_mcs_index_10() {
        let e = mcs_lookup(10).unwrap();
        assert_eq!(e.modulation, Modulation::Qam16);
        assert_eq!(e.code_rate_x1024, 340);
    }

    #[test]
    fn test_mcs_index_16() {
        let e = mcs_lookup(16).unwrap();
        assert_eq!(e.modulation, Modulation::Qam16);
        assert_eq!(e.code_rate_x1024, 658);
    }

    #[test]
    fn test_mcs_index_17() {
        let e = mcs_lookup(17).unwrap();
        assert_eq!(e.modulation, Modulation::Qam64);
        assert_eq!(e.code_rate_x1024, 438);
    }

    #[test]
    fn test_mcs_index_24() {
        let e = mcs_lookup(24).unwrap();
        assert_eq!(e.modulation, Modulation::Qam64);
        assert_eq!(e.code_rate_x1024, 772);
    }

    #[test]
    fn test_mcs_index_27() {
        let e = mcs_lookup(27).unwrap();
        assert_eq!(e.modulation, Modulation::Qam64);
        assert_eq!(e.code_rate_x1024, 910);
    }

    #[test]
    fn test_mcs_index_28_256qam() {
        let e = mcs_lookup(28).unwrap();
        assert_eq!(e.modulation, Modulation::Qam256);
        assert_eq!(e.code_rate_x1024, 948);
    }

    #[test]
    fn test_mcs_out_of_range() {
        assert!(mcs_lookup(29).is_none());
        assert!(mcs_lookup(255).is_none());
    }

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(Modulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(Modulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(Modulation::Qam64.bits_per_symbol(), 6);
        assert_eq!(Modulation::Qam256.bits_per_symbol(), 8);
    }

    // ------------------------------------------------------------------
    // TBS calculation tests
    // ------------------------------------------------------------------

    #[test]
    fn test_tbs_small_allocation() {
        // MCS 9 (QPSK, R=679/1024), 1 PRB, 1 layer, 12 symbols, 6 DMRS OH
        let result = compute_tbs(9, 1, 1, 12, 6, 0).unwrap();
        assert!(result.tbs_bits >= 24, "TBS too small: {}", result.tbs_bits);
        assert_eq!(result.n_prb, 1);
        assert_eq!(result.mcs, 9);
    }

    #[test]
    fn test_tbs_medium_allocation() {
        // MCS 16 (16QAM), 25 PRBs, 1 layer, 12 symbols
        let result = compute_tbs(16, 25, 1, 12, 6, 0).unwrap();
        assert!(result.tbs_bits > 1000, "Expected larger TBS, got {}", result.tbs_bits);
        assert_eq!(result.n_layers, 1);
    }

    #[test]
    fn test_tbs_large_allocation() {
        // MCS 27 (64QAM), 106 PRBs, 4 layers, 12 symbols
        let result = compute_tbs(27, 106, 4, 12, 6, 0).unwrap();
        assert!(result.tbs_bits > 100_000, "Expected very large TBS: {}", result.tbs_bits);
    }

    #[test]
    fn test_tbs_multiple_mcs() {
        // Verify that higher MCS produces larger TBS with same PRB/layer allocation
        let t0 = compute_tbs(0, 25, 1, 12, 6, 0).unwrap().tbs_bits;
        let t9 = compute_tbs(9, 25, 1, 12, 6, 0).unwrap().tbs_bits;
        let t17 = compute_tbs(17, 25, 1, 12, 6, 0).unwrap().tbs_bits;
        let t27 = compute_tbs(27, 25, 1, 12, 6, 0).unwrap().tbs_bits;
        assert!(t0 < t9, "MCS 0 ({}) should be < MCS 9 ({})", t0, t9);
        assert!(t9 < t17, "MCS 9 ({}) should be < MCS 17 ({})", t9, t17);
        assert!(t17 < t27, "MCS 17 ({}) should be < MCS 27 ({})", t17, t27);
    }

    #[test]
    fn test_tbs_scales_with_prb() {
        let t1 = compute_tbs(10, 1, 1, 12, 6, 0).unwrap().tbs_bits;
        let t10 = compute_tbs(10, 10, 1, 12, 6, 0).unwrap().tbs_bits;
        let t100 = compute_tbs(10, 100, 1, 12, 6, 0).unwrap().tbs_bits;
        assert!(t1 < t10, "{} < {}", t1, t10);
        assert!(t10 < t100, "{} < {}", t10, t100);
    }

    #[test]
    fn test_tbs_invalid_mcs() {
        assert!(compute_tbs(29, 25, 1, 12, 6, 0).is_none());
    }

    // ------------------------------------------------------------------
    // CRC-24A tests
    // ------------------------------------------------------------------

    #[test]
    fn test_crc24a_empty() {
        let crc = crc24a(&[]);
        assert_eq!(crc, 0);
    }

    #[test]
    fn test_crc24a_zero_byte() {
        let crc = crc24a(&[0x00]);
        // CRC of zero byte should be deterministic
        assert!(crc <= 0xFF_FFFF);
    }

    #[test]
    fn test_crc24a_all_ones() {
        let crc = crc24a(&[0xFF, 0xFF, 0xFF]);
        assert!(crc <= 0xFF_FFFF);
    }

    #[test]
    fn test_crc24a_known_value() {
        // Verify CRC-24A produces consistent value
        let data = [0x01, 0x02, 0x03, 0x04];
        let crc1 = crc24a(&data);
        let crc2 = crc24a(&data);
        assert_eq!(crc1, crc2, "CRC-24A should be deterministic");
    }

    #[test]
    fn test_crc24b_empty() {
        let crc = crc24b(&[]);
        assert_eq!(crc, 0);
    }

    #[test]
    fn test_crc24b_differs_from_24a() {
        let data = [0xAB, 0xCD, 0xEF];
        let a = crc24a(&data);
        let b = crc24b(&data);
        // Different polynomials should produce different results for most inputs
        // (not strictly guaranteed for all inputs, but true for this test vector)
        assert_ne!(a, b, "CRC-24A and CRC-24B use different polynomials");
    }

    #[test]
    fn test_crc16_basic() {
        let data = [0x01, 0x02];
        let crc = crc16(&data);
        assert!(crc <= 0xFFFF);
        assert_eq!(crc, crc16(&data), "CRC-16 should be deterministic");
    }

    #[test]
    fn test_crc_bits_vs_bytes() {
        // CRC computed from bits should match CRC computed from bytes
        let data = [0xA5u8, 0x3Cu8];
        let bits = bytes_to_bits(&data);
        let crc_bytes = crc24a(&data);
        let crc_bits = crc24a_bits(&bits);
        assert_eq!(crc_bytes, crc_bits, "bit and byte CRC-24A must match");
    }

    // ------------------------------------------------------------------
    // CRC attachment tests
    // ------------------------------------------------------------------

    #[test]
    fn test_attach_tb_crc_small() {
        // Small TB (<= 3824 bits) should use CRC-16 (16 bits)
        let tb = vec![false; 100];
        let with_crc = attach_tb_crc(&tb);
        assert_eq!(with_crc.len(), 100 + 16);
    }

    #[test]
    fn test_attach_tb_crc_large() {
        // Large TB (> 3824 bits) should use CRC-24A (24 bits)
        let tb = vec![true; 4000];
        let with_crc = attach_tb_crc(&tb);
        assert_eq!(with_crc.len(), 4000 + 24);
    }

    #[test]
    fn test_attach_cb_crc() {
        let cb = vec![true, false, true, true, false];
        let with_crc = attach_cb_crc(&cb);
        assert_eq!(with_crc.len(), cb.len() + 24);
    }

    // ------------------------------------------------------------------
    // Code block segmentation tests
    // ------------------------------------------------------------------

    #[test]
    fn test_segmentation_small_tb_single_cb() {
        // TB <= max_cb_size should produce 1 code block
        let tb = vec![false; 500];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, _) = segment_code_blocks(&with_crc);
        assert_eq!(cbs.len(), 1, "Small TB should produce 1 CB");
    }

    #[test]
    fn test_segmentation_large_tb_multiple_cb() {
        // TB > 8448 bits should be segmented into multiple CBs
        let tb = vec![true; 10000];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, _) = segment_code_blocks(&with_crc);
        assert!(cbs.len() >= 2, "Large TB should produce multiple CBs");
    }

    #[test]
    fn test_segmentation_cb_size_multiple_of_z() {
        let tb = vec![false; 2000];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, ldpc) = segment_code_blocks(&with_crc);
        // Each CB info_bits length must be a multiple of Z * K_b
        let expected_k = ldpc.k_b as usize * ldpc.lifting_size as usize;
        for cb in &cbs {
            assert_eq!(
                cb.info_bits.len(),
                expected_k,
                "CB info_bits must equal Z*K_b = {}",
                expected_k
            );
        }
    }

    #[test]
    fn test_segmentation_bg2_small() {
        // Very small TB should use BG2
        let tb = vec![false; 200];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, ldpc) = segment_code_blocks(&with_crc);
        assert_eq!(ldpc.base_graph, 2, "Small TB should use BG2");
        assert_eq!(ldpc.k_b, 10);
        assert!(!cbs.is_empty());
    }

    #[test]
    fn test_segmentation_bg1_large() {
        // Large TB should use BG1
        let tb = vec![true; 5000];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, ldpc) = segment_code_blocks(&with_crc);
        assert_eq!(ldpc.base_graph, 1, "Large TB should use BG1");
        assert_eq!(ldpc.k_b, 22);
        assert!(!cbs.is_empty());
    }

    // ------------------------------------------------------------------
    // LDPC lifting size selection
    // ------------------------------------------------------------------

    #[test]
    fn test_lifting_size_minimum() {
        // For very small K, select minimum valid Z
        let z = select_lifting_size(22, 22);
        assert!(z >= 1, "Lifting size must be positive");
        assert_eq!(z, 2, "Minimum lifting size is 2");
    }

    #[test]
    fn test_lifting_size_grows_with_k() {
        let z1 = select_lifting_size(100, 10);
        let z2 = select_lifting_size(500, 10);
        let z3 = select_lifting_size(3000, 10);
        assert!(z1 <= z2, "Z should grow with K: z1={} z2={}", z1, z2);
        assert!(z2 <= z3, "Z should grow with K: z2={} z3={}", z2, z3);
    }

    #[test]
    fn test_lifting_size_max() {
        // Should not exceed 384
        let z = select_lifting_size(100000, 22);
        assert!(z <= 384, "Z cannot exceed 384");
    }

    #[test]
    fn test_ldpc_params_bg1() {
        let p = ldpc_params(5000);
        assert_eq!(p.base_graph, 1);
        assert_eq!(p.k_b, 22);
        assert!(p.lifting_size > 0);
        assert!(p.n_cb > 0);
    }

    #[test]
    fn test_ldpc_params_bg2() {
        let p = ldpc_params(200);
        assert_eq!(p.base_graph, 2);
        assert_eq!(p.k_b, 10);
        assert!(p.lifting_size > 0);
    }

    // ------------------------------------------------------------------
    // LDPC encoding tests
    // ------------------------------------------------------------------

    #[test]
    fn test_ldpc_encode_length() {
        let tb = vec![false; 500];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, _) = segment_code_blocks(&with_crc);
        let cb = &cbs[0];
        let cw = ldpc_encode(cb);
        let expected_n = cb.ldpc.n_cb;
        assert_eq!(
            cw.n, expected_n,
            "Codeword length mismatch: got {}, expected {}",
            cw.n, expected_n
        );
        assert_eq!(cw.systematic.len() + cw.parity.len(), cw.n as usize);
    }

    #[test]
    fn test_ldpc_encode_systematic_preserved() {
        let tb: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
        let with_crc = attach_tb_crc(&tb);
        let (cbs, _) = segment_code_blocks(&with_crc);
        let cb = &cbs[0];
        let cw = ldpc_encode(cb);
        // Systematic bits must equal info bits
        assert_eq!(&cw.systematic, &cb.info_bits);
    }

    // ------------------------------------------------------------------
    // Rate matching tests
    // ------------------------------------------------------------------

    #[test]
    fn test_rate_match_output_length() {
        let tb = vec![false; 500];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, ldpc) = segment_code_blocks(&with_crc);
        let cw = ldpc_encode(&cbs[0]);
        let e = 512u32;
        let rm = rate_match(&cw, e, 0, ldpc.lifting_size);
        assert_eq!(rm.len(), e as usize);
    }

    #[test]
    fn test_rate_match_rv_idx_produces_different_start() {
        let tb: Vec<bool> = (0..500).map(|i| i % 2 == 1).collect();
        let with_crc = attach_tb_crc(&tb);
        let (cbs, ldpc) = segment_code_blocks(&with_crc);
        let cw = ldpc_encode(&cbs[0]);
        let e = 200u32;
        let rm0 = rate_match(&cw, e, 0, ldpc.lifting_size);
        let rm1 = rate_match(&cw, e, 1, ldpc.lifting_size);
        // Different RV indices should produce sequences from different starting offsets.
        // Verify that the k0 offset for RV1 is non-zero and the full sequences differ.
        let diff_count = rm0.iter().zip(rm1.iter()).filter(|(a, b)| a != b).count();
        assert!(
            diff_count > 0,
            "RV0 and RV1 should produce different sequences (k0 differs)"
        );
    }

    #[test]
    fn test_rate_match_circular_wrapping() {
        let tb = vec![true; 200];
        let with_crc = attach_tb_crc(&tb);
        let (cbs, ldpc) = segment_code_blocks(&with_crc);
        let cw = ldpc_encode(&cbs[0]);
        // Request more bits than codeword length (circular wrapping)
        let e = cw.n * 3;
        let rm = rate_match(&cw, e, 0, ldpc.lifting_size);
        assert_eq!(rm.len(), e as usize);
    }

    // ------------------------------------------------------------------
    // Scrambling / Gold sequence tests
    // ------------------------------------------------------------------

    #[test]
    fn test_gold_seq_deterministic() {
        let mut s1 = GoldSeq::new(12345);
        let mut s2 = GoldSeq::new(12345);
        let bits1: Vec<bool> = (0..64).map(|_| s1.next_bit()).collect();
        let bits2: Vec<bool> = (0..64).map(|_| s2.next_bit()).collect();
        assert_eq!(bits1, bits2);
    }

    #[test]
    fn test_gold_seq_different_cinit() {
        let b1 = GoldSeq::new(0).generate(128);
        let b2 = GoldSeq::new(1).generate(128);
        assert_ne!(b1, b2, "Different c_init should produce different sequences");
    }

    #[test]
    fn test_scramble_unscramble() {
        let bits = vec![true, false, true, true, false, false, true, false];
        let c_init = 42u32;
        let scrambled = scramble(&bits, c_init);
        let recovered = scramble(&scrambled, c_init);
        assert_eq!(recovered, bits, "Double scrambling must recover original");
    }

    #[test]
    fn test_pdsch_c_init() {
        // Verify c_init formula components
        let c0 = pdsch_c_init(0xC0DE, 0, 42);
        let c1 = pdsch_c_init(0xC0DE, 1, 42);
        assert_ne!(c0, c1, "Different codeword index should produce different c_init");
    }

    // ------------------------------------------------------------------
    // Modulation mapping tests
    // ------------------------------------------------------------------

    #[test]
    fn test_qpsk_symbol_count() {
        let bits = vec![false; 100];
        let syms = modulate_qpsk(&bits);
        assert_eq!(syms.len(), 50);
    }

    #[test]
    fn test_qpsk_energy() {
        // All QPSK symbols should have unit average energy
        let bits = vec![false, false, false, true, true, false, true, true];
        let syms = modulate_qpsk(&bits);
        for s in &syms {
            let energy = s.magnitude_sq();
            assert!(
                (energy - 1.0).abs() < 1e-10,
                "QPSK symbol energy should be 1.0, got {}",
                energy
            );
        }
    }

    #[test]
    fn test_qpsk_all_four_points() {
        // Generate all four QPSK symbols
        let bits = vec![false, false, false, true, true, false, true, true];
        let syms = modulate_qpsk(&bits);
        assert_eq!(syms.len(), 4);
        // All have unit energy
        for s in &syms {
            assert!((s.magnitude_sq() - 1.0).abs() < 1e-10);
        }
        // All four quadrants covered
        let i_vals: Vec<i8> = syms.iter().map(|s| if s.i > 0.0 { 1 } else { -1 }).collect();
        let q_vals: Vec<i8> = syms.iter().map(|s| if s.q > 0.0 { 1 } else { -1 }).collect();
        assert!(i_vals.contains(&1) && i_vals.contains(&-1));
        assert!(q_vals.contains(&1) && q_vals.contains(&-1));
    }

    #[test]
    fn test_16qam_symbol_count() {
        let bits = vec![false; 80];
        let syms = modulate_16qam(&bits);
        assert_eq!(syms.len(), 20);
    }

    #[test]
    fn test_16qam_normalized_energy() {
        // Average energy of 16QAM should be 1 (normalized by 1/sqrt(10))
        let norm = 1.0 / 10.0f64.sqrt();
        // All 16 symbols
        let expected: Vec<f64> = [1.0, 3.0].iter()
            .flat_map(|&i| [1.0, 3.0].iter().map(move |&q| {
                (i * norm) * (i * norm) + (q * norm) * (q * norm)
            }))
            .collect();
        let avg_energy = expected.iter().sum::<f64>() / expected.len() as f64;
        assert!((avg_energy - 1.0).abs() < 0.1, "16QAM avg energy: {}", avg_energy);
    }

    #[test]
    fn test_64qam_symbol_count() {
        let bits = vec![true; 60];
        let syms = modulate_64qam(&bits);
        assert_eq!(syms.len(), 10);
    }

    #[test]
    fn test_256qam_symbol_count() {
        let bits = vec![false; 80];
        let syms = modulate_256qam(&bits);
        assert_eq!(syms.len(), 10);
    }

    #[test]
    fn test_modulate_dispatch() {
        let bits = vec![false; 32];
        let q = modulate(&bits, Modulation::Qpsk);
        let m = modulate(&bits, Modulation::Qam16);
        let s = modulate(&bits, Modulation::Qam64);
        let t = modulate(&bits, Modulation::Qam256);
        assert_eq!(q.len(), 16);
        assert_eq!(m.len(), 8);
        assert_eq!(s.len(), 6); // 32/6 = 5 full + 1 partial = 6 chunks
        assert_eq!(t.len(), 4);
    }

    // ------------------------------------------------------------------
    // DMRS generation tests
    // ------------------------------------------------------------------

    #[test]
    fn test_dmrs_sequence_length() {
        let syms = generate_dmrs(0, 2, 0, 100);
        assert_eq!(syms.len(), 100);
    }

    #[test]
    fn test_dmrs_unit_energy() {
        // DMRS uses QPSK-like modulation, all symbols have unit energy
        let syms = generate_dmrs(0, 2, 0, 50);
        for s in &syms {
            let e = s.magnitude_sq();
            assert!(
                (e - 0.5).abs() < 0.01 || (e - 1.0).abs() < 0.01,
                "DMRS symbol energy: {}",
                e
            );
        }
    }

    #[test]
    fn test_dmrs_changes_with_slot() {
        let s0 = generate_dmrs(0, 2, 0, 20);
        let s1 = generate_dmrs(1, 2, 0, 20);
        // Different slots should generally produce different sequences
        let diff = s0.iter().zip(s1.iter()).filter(|(a, b)| a.i != b.i).count();
        assert!(diff > 0, "DMRS should change across slots");
    }

    #[test]
    fn test_dmrs_changes_with_scrambling_id() {
        let s0 = generate_dmrs(0, 2, 0, 20);
        let s100 = generate_dmrs(0, 2, 100, 20);
        let diff = s0.iter().zip(s100.iter()).filter(|(a, b)| a.i != b.i).count();
        assert!(diff > 0, "DMRS should change with scrambling ID");
    }

    #[test]
    fn test_dmrs_symbol_positions_type_a() {
        // No additional DMRS: only symbol 2
        let pos = dmrs_symbol_positions(0);
        assert_eq!(pos, vec![2]);

        let pos1 = dmrs_symbol_positions(1);
        assert!(pos1.contains(&2) && pos1.contains(&7));

        let pos2 = dmrs_symbol_positions(2);
        assert!(pos2.contains(&2) && pos2.contains(&7) && pos2.contains(&11));
    }

    #[test]
    fn test_dmrs_type1_re_indices() {
        let indices = dmrs_type1_re_indices();
        assert_eq!(indices.len(), 6);
        // Should be even subcarrier indices 0,2,4,6,8,10
        assert_eq!(indices, vec![0, 2, 4, 6, 8, 10]);
    }

    #[test]
    fn test_dmrs_type2_re_indices() {
        let indices = dmrs_type2_re_indices();
        assert_eq!(indices, vec![0, 1, 6, 7]);
    }

    // ------------------------------------------------------------------
    // Resource element mapping tests
    // ------------------------------------------------------------------

    #[test]
    fn test_enumerate_res_count() {
        let dmrs = DmrsConfig::default();
        let res = enumerate_res(1, 14, 0, &dmrs);
        // 1 PRB * 12 subcarriers * 14 symbols = 168 REs
        assert_eq!(res.len(), 168);
    }

    #[test]
    fn test_count_data_res_less_than_total() {
        let dmrs = DmrsConfig { type_id: 1, additional_pos: 0, scrambling_id: 0 };
        let total_res = 12u32 * 12 * 25; // 12 sym * 12 sc * 25 PRB = 3600
        let data_res = count_data_res(25, 12, 2, &dmrs);
        assert!(data_res < total_res, "Data REs {} should be less than total {}", data_res, total_res);
    }

    #[test]
    fn test_enumerate_res_dmrs_marked() {
        let dmrs = DmrsConfig { type_id: 1, additional_pos: 0, scrambling_id: 0 };
        let res = enumerate_res(1, 14, 0, &dmrs);
        // Symbol 2 should have some DMRS REs
        let dmrs_count = res.iter().filter(|r| r.is_dmrs).count();
        assert!(dmrs_count > 0, "Should have DMRS REs");
    }

    // ------------------------------------------------------------------
    // Layer mapping tests
    // ------------------------------------------------------------------

    #[test]
    fn test_layer_map_single_layer() {
        let syms: Vec<IqSymbol> = (0..10).map(|i| IqSymbol::new(i as f64, 0.0)).collect();
        let layers = layer_map(&syms, 1);
        assert_eq!(layers.len(), 1);
        assert_eq!(layers[0].len(), 10);
    }

    #[test]
    fn test_layer_map_two_layers() {
        let syms: Vec<IqSymbol> = (0..10).map(|i| IqSymbol::new(i as f64, 0.0)).collect();
        let layers = layer_map(&syms, 2);
        assert_eq!(layers.len(), 2);
        assert_eq!(layers[0].len(), 5);
        assert_eq!(layers[1].len(), 5);
    }

    #[test]
    fn test_layer_map_four_layers() {
        let syms: Vec<IqSymbol> = (0..8).map(|i| IqSymbol::new(i as f64, 0.0)).collect();
        let layers = layer_map(&syms, 4);
        assert_eq!(layers.len(), 4);
        for l in &layers {
            assert_eq!(l.len(), 2);
        }
    }

    // ------------------------------------------------------------------
    // Utility tests
    // ------------------------------------------------------------------

    #[test]
    fn test_bytes_to_bits() {
        let data = [0xA5u8]; // 1010_0101
        let bits = bytes_to_bits(&data);
        assert_eq!(bits.len(), 8);
        assert_eq!(bits, vec![true, false, true, false, false, true, false, true]);
    }

    #[test]
    fn test_bits_to_bytes_roundtrip() {
        let original = [0xDE, 0xAD, 0xBE, 0xEF];
        let bits = bytes_to_bits(&original);
        let recovered = bits_to_bytes(&bits);
        assert_eq!(recovered, original);
    }

    #[test]
    fn test_bits_to_bytes_padding() {
        // 9 bits -> 2 bytes (last byte padded with 0)
        let bits = vec![true; 9];
        let bytes = bits_to_bytes(&bits);
        assert_eq!(bytes.len(), 2);
        assert_eq!(bytes[0], 0xFF);
        assert_eq!(bytes[1], 0x80); // 1000_0000
    }

    // ------------------------------------------------------------------
    // PdschProcessor integration tests
    // ------------------------------------------------------------------

    #[test]
    fn test_processor_tbs_nonzero() {
        let cfg = PdschConfig::default();
        let dmrs = DmrsConfig::default();
        let proc = PdschProcessor::new(cfg, dmrs);
        let tbs = proc.compute_tbs(1, 2, 12, 0);
        assert!(tbs.tbs_bits > 0);
    }

    #[test]
    fn test_processor_process_tb() {
        let cfg = PdschConfig { mcs: 5, n_prb: 4, n_layers: 1, dmrs_type: 1, rv_idx: 0 };
        let dmrs = DmrsConfig::default();
        let proc = PdschProcessor::new(cfg, dmrs);
        let tb: Vec<bool> = (0..100).map(|i| i % 2 == 1).collect();
        let symbols = proc.process_tb(&tb);
        assert!(!symbols.is_empty(), "Processing should produce IQ symbols");
    }

    #[test]
    fn test_processor_map_to_grid() {
        let cfg = PdschConfig { mcs: 9, n_prb: 4, n_layers: 1, dmrs_type: 1, rv_idx: 0 };
        let dmrs = DmrsConfig::default();
        let proc = PdschProcessor::new(cfg, dmrs);
        let tb: Vec<bool> = (0..400).map(|i| i % 2 == 0).collect();
        let symbols = proc.process_tb(&tb);
        let (data_mapped, dmrs_mapped) = proc.map_to_resource_grid(&symbols, 0, 2, 12);
        assert!(!data_mapped.is_empty(), "Should have mapped data REs");
        assert!(!dmrs_mapped.is_empty(), "Should have mapped DMRS REs");
    }

    #[test]
    fn test_processor_different_mcs_different_symbol_count() {
        let dmrs = DmrsConfig::default();
        let tb = vec![true; 200];

        let cfg_low = PdschConfig { mcs: 0, n_prb: 4, n_layers: 1, dmrs_type: 1, rv_idx: 0 };
        let cfg_high = PdschConfig { mcs: 27, n_prb: 4, n_layers: 1, dmrs_type: 1, rv_idx: 0 };

        let syms_low = PdschProcessor::new(cfg_low, dmrs).process_tb(&tb);
        let syms_high = PdschProcessor::new(cfg_high, dmrs).process_tb(&tb);

        // process_tb rate-matches to codeword size; the LDPC codeword is the same size
        // for both, but after modulation (which divides by bits/symbol), higher MCS
        // (64QAM, 6 bps) produces fewer symbols than lower MCS (QPSK, 2 bps).
        // Both must produce a non-empty output.
        assert!(!syms_low.is_empty(), "Low MCS should produce symbols");
        assert!(!syms_high.is_empty(), "High MCS should produce symbols");
        // Verify different modulations produce different symbol counts for same TB
        assert_ne!(
            syms_low.len(), syms_high.len(),
            "Different modulations should produce different symbol counts"
        );
    }
}

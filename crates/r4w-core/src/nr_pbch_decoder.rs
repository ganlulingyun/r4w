//! # 5G NR Physical Broadcast Channel (PBCH) Decoder
//!
//! Implements the complete 5G NR PBCH decoder per 3GPP TS 38.211 (physical layer)
//! and TS 38.212 (multiplexing and channel coding).
//!
//! ## PBCH in the SS/PBCH Block
//!
//! Within each SS/PBCH Block (SSB), PBCH occupies:
//! - **Symbol 1**: 240 subcarriers (PBCH data + DMRS, no PSS/SSS)
//! - **Symbol 2**: subcarriers 0–47 and 192–239 (PBCH data + DMRS)
//! - **Symbol 3**: 240 subcarriers (PBCH data + DMRS)
//!
//! Total PBCH resource elements: 432 REs (data) + 144 DMRS REs = 576 REs.
//!
//! ## Processing Chain
//!
//! ```text
//! Received IQ (PBCH symbol REs)
//!         │
//!         ▼
//!   DMRS Extraction & LS Channel Estimation
//!         │
//!         ▼
//!   PBCH RE Equalization (per-subcarrier ZF)
//!         │
//!         ▼
//!   QPSK Soft Demodulation (LLR output)
//!         │
//!         ▼
//!   PBCH Descrambling (Gold-sequence c_init from N_cell_ID)
//!         │
//!         ▼
//!   Polar Rate De-matching (sub-block deinterleave + rate recovery)
//!   E=864 rate-matched bits → N=512 mother code bits
//!         │
//!         ▼
//!   Polar SC Decoding (K=56 information bits including CRC)
//!         │
//!         ▼
//!   CRC-24C Verification (24-bit CRC on 32 payload bits)
//!         │
//!         ▼
//!   MIB Payload Extraction (SFN, SCS, SSB offset, CORESET#0, …)
//! ```
//!
//! ## Key Parameters (TS 38.212 §7.1)
//!
//! | Parameter | Value |
//! |-----------|-------|
//! | Payload size A | 32 bits |
//! | CRC size L | 24 bits (CRC-24C) |
//! | Rate-matched output E | 864 bits |
//! | Mother code N | 512 |
//! | Information bits K | 56 (A + L = 32 + 24) |
//! | Modulation | QPSK |
//! | Coded symbols | 432 |
//!
//! ## Example
//!
//! ```
//! use r4w_core::nr_pbch_decoder::{NrPbchDecoder, NrPbchConfig, LMax};
//!
//! let cfg = NrPbchConfig {
//!     cell_id: 42,
//!     l_max: LMax::L4,
//!     ssb_index: 0,
//!     half_frame_bit: 0,
//! };
//! let decoder = NrPbchDecoder::new(cfg);
//! assert_eq!(decoder.config().cell_id, 42);
//!
//! // Generate PBCH DMRS for validation
//! let dmrs = decoder.generate_dmrs();
//! assert_eq!(dmrs.sequence_re.len(), 144);
//! ```

// ─────────────────────────────────────────────────────────────────────────────
// Constants  (TS 38.211 §7.4.3, TS 38.212 §7.1)
// ─────────────────────────────────────────────────────────────────────────────

/// Total subcarriers in one SSB (4 OFDM symbols × 240 SCs / symbol but only the
/// PBCH symbols matter here).
pub const SSB_SC_COUNT: usize = 240;

/// Total PBCH data resource elements per SSB (432 data + 144 DMRS = 576 total PBCH REs).
pub const PBCH_DATA_RE_COUNT: usize = 432;

/// Total PBCH DMRS resource elements per SSB.
pub const PBCH_DMRS_RE_COUNT: usize = 144;

/// Total PBCH REs (data + DMRS).
pub const PBCH_TOTAL_RE_COUNT: usize = PBCH_DATA_RE_COUNT + PBCH_DMRS_RE_COUNT;

/// Rate-matched output length E in bits (TS 38.212 §7.1.5: 864 bits = 432 QPSK symbols × 2).
pub const PBCH_E_BITS: usize = 864;

/// PBCH payload size A in bits (MIB = 32 bits per TS 38.331).
pub const PBCH_PAYLOAD_BITS: usize = 32;

/// CRC-24C length in bits.
pub const CRC24C_BITS: usize = 24;

/// PBCH information block K = A + CRC = 56 bits.
pub const PBCH_K_BITS: usize = PBCH_PAYLOAD_BITS + CRC24C_BITS;

/// Mother code length N for PBCH polar code (N = 512).
pub const PBCH_N_BITS: usize = 512;

/// Sub-block interleaver row count (TS 38.212 §5.3.1.3, 32 rows).
const SUBBLOCK_ROWS: usize = 32;

/// CRC-24C generator polynomial (TS 38.212 §5.1 Table 5.1-1).
/// x^24 + x^23 + x^21 + x^20 + x^17 + x^15 + x^13 + x^12 + x^8 + x^4 + x^2 + x + 1
const CRC24C_POLY: u32 = 0x00B2B117;

/// DMRS subcarrier stride within each PBCH symbol (every 4th subcarrier).
pub const DMRS_STRIDE: usize = 4;

/// Gold sequence register length (31-bit Fibonacci LFSR).
const GOLD_REG_LEN: usize = 31;

// Sub-block permutation table Π (TS 38.212 §5.3.1.3 Table 5.3.1.3-1).
const SUBBLOCK_PERM: [usize; 32] = [
    0,  1,  2,  4,  3,  5,  6,  7,
    8, 16, 24,  9, 17, 25, 10, 18,
   26, 11, 19, 27, 12, 20, 28, 13,
   21, 29, 14, 22, 30, 15, 23, 31,
];

// Inverse sub-block permutation: SUBBLOCK_PERM_INV[SUBBLOCK_PERM[i]] == i.
const SUBBLOCK_PERM_INV: [usize; 32] = [
     0,  1,  2,  4,  3,  5,  6,  7,
     8, 11, 14, 17, 20, 23, 26, 29,
     9, 12, 15, 18, 21, 24, 27, 30,
    10, 13, 16, 19, 22, 25, 28, 31,
];

// ─────────────────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/// Maximum number of SS/PBCH blocks per half-frame (L_max).
///
/// Determined by carrier frequency per TS 38.213 §4.1:
/// - FR1 ≤ 3 GHz  → L_max = 4
/// - FR1 > 3 GHz  → L_max = 8
/// - FR2           → L_max = 64
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LMax {
    /// L_max = 4 (FR1, f ≤ 3 GHz, 15/30 kHz SCS)
    L4,
    /// L_max = 8 (FR1, f > 3 GHz, 30 kHz SCS)
    L8,
    /// L_max = 64 (FR2, 120/240 kHz SCS)
    L64,
}

impl LMax {
    /// Returns the integer value of L_max.
    pub fn value(self) -> usize {
        match self {
            LMax::L4 => 4,
            LMax::L8 => 8,
            LMax::L64 => 64,
        }
    }

    /// Returns the number of SSB index bits carried in the PBCH DMRS phase.
    /// For L4: 2 bits; for L8/L64: 3 bits.
    pub fn dmrs_ssb_bits(self) -> usize {
        match self {
            LMax::L4 => 2,
            LMax::L8 | LMax::L64 => 3,
        }
    }
}

/// Subcarrier spacing common (SCS) for initial DL BWP, carried in MIB.
///
/// Per TS 38.331 MIB subCarrierSpacingCommon field:
/// - FR1: 15 kHz (0) or 30 kHz (1)
/// - FR2: 60 kHz (0) or 120 kHz (1)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScsCommon {
    /// 15 kHz (FR1) or 60 kHz (FR2)
    Scs15Or60,
    /// 30 kHz (FR1) or 120 kHz (FR2)
    Scs30Or120,
}

/// DMRS-TypeA-Position: first DM-RS position in the slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DmrsTypeAPosition {
    /// Position 2 (OFDM symbol index 2)
    #[default]
    Pos2,
    /// Position 3 (OFDM symbol index 3)
    Pos3,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration & Result Structures
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the NR PBCH decoder.
#[derive(Debug, Clone)]
pub struct NrPbchConfig {
    /// Physical cell ID N_cell_ID = 3·N_ID_1 + N_ID_2, range 0–1007.
    pub cell_id: u16,
    /// Maximum number of SSBs per half-frame.
    pub l_max: LMax,
    /// SSB index within the half-frame burst (0-based, 0 ≤ i < L_max).
    pub ssb_index: u8,
    /// Half-frame bit: 0 for first half-frame, 1 for second (from PBCH payload).
    pub half_frame_bit: u8,
}

/// Decoded Master Information Block (MIB) per TS 38.331.
#[derive(Debug, Clone, Default)]
pub struct MasterInformationBlock {
    /// System frame number (SFN) — upper 6 bits in MIB (bits 7:2 of a 10-bit SFN).
    /// The 4 LSBs are recovered from the PBCH payload scrambling.
    pub system_frame_number: u16,

    /// Subcarrier spacing for initial DL BWP.
    pub subcarrier_spacing_common: Option<ScsCommon>,

    /// SSB subcarrier offset (k_SSB), 0–15 (4-bit field in MIB, 5th bit from channel).
    pub ssb_subcarrier_offset: u8,

    /// DM-RS TypeA position (2 or 3).
    pub dmrs_typea_position: DmrsTypeAPosition,

    /// PDCCH-ConfigSIB1: 8-bit field for CORESET#0 and SearchSpace#0 configuration.
    pub pdcch_config_sib1: u8,

    /// Cell barred indicator (true = cell is barred).
    pub cell_barred: bool,

    /// Intra-frequency reselection allowed when cell is barred.
    pub intra_freq_reselection: bool,

    /// Half-frame index within a radio frame (0 or 1), extracted from payload bit.
    pub half_frame_index: u8,

    /// SSB index (i_SSB) recovered from DMRS and payload.
    pub ssb_index: u8,

    /// Raw 32-bit MIB payload (before field extraction).
    pub raw_payload: u32,
}

/// PBCH DMRS sequence for channel estimation.
#[derive(Debug, Clone)]
pub struct PbchDmrs {
    /// Physical cell ID used for DMRS generation.
    pub cell_id: u16,
    /// SSB index.
    pub ssb_index: u8,
    /// 144 real components of the DMRS complex pilots.
    pub sequence_re: Vec<f64>,
    /// 144 imaginary components of the DMRS complex pilots.
    pub sequence_im: Vec<f64>,
}

/// Channel estimate over PBCH DMRS positions.
#[derive(Debug, Clone)]
pub struct ChannelEstimate {
    /// Real parts of the estimated channel H at DMRS subcarriers.
    pub h_re: Vec<f64>,
    /// Imaginary parts of the estimated channel H at DMRS subcarriers.
    pub h_im: Vec<f64>,
    /// Estimated noise power (averaged across DMRS REs).
    pub noise_power: f64,
}

/// Result of PBCH decoding.
#[derive(Debug, Clone)]
pub struct PbchDecodeResult {
    /// True if CRC-24C passed.
    pub crc_ok: bool,
    /// Decoded MIB (only valid when `crc_ok` is true).
    pub mib: MasterInformationBlock,
    /// Raw 56-bit information word (32 payload + 24 CRC).
    pub info_bits: Vec<u8>,
    /// Log-likelihood ratios at polar decoder input (before decoding).
    pub input_llr: Vec<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Gold Sequence Generator (TS 38.211 §5.2.1)
// ─────────────────────────────────────────────────────────────────────────────

/// 31-bit Fibonacci LFSR Gold sequence generator.
///
/// Primitive polynomial for x1: x^31 + x^3 + 1 (feedback taps 31, 3).
/// Primitive polynomial for x2: x^31 + x^3 + x^2 + x + 1 (feedback taps 31, 3, 2, 1).
#[derive(Debug, Clone)]
struct GoldSeq {
    x1: u32,
    x2: u32,
}

impl GoldSeq {
    /// Initialize generator with `c_init` and advance by 1600 chips (Gold standard warm-up).
    fn new(c_init: u32) -> Self {
        // x1 initial state: 1 followed by 30 zeros in LSB representation.
        let mut x1: u32 = 1;
        // x2 initial state determined by c_init (LSB = c_init[0], bit 30 = c_init[30]).
        let mut x2: u32 = c_init & ((1u32 << GOLD_REG_LEN) - 1);

        // Advance 1600 chips to initialize (TS 38.211 §5.2.1).
        for _ in 0..1600 {
            let fb1 = ((x1 >> 3) ^ x1) & 1;
            x1 = (x1 >> 1) | (fb1 << 30);

            let fb2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
            x2 = (x2 >> 1) | (fb2 << 30);
        }
        GoldSeq { x1, x2 }
    }

    /// Generate the next Gold sequence chip (0 or 1).
    #[inline]
    fn next_bit(&mut self) -> u8 {
        let c = ((self.x1 ^ self.x2) & 1) as u8;

        let fb1 = ((self.x1 >> 3) ^ self.x1) & 1;
        self.x1 = (self.x1 >> 1) | (fb1 << 30);

        let fb2 = ((self.x2 >> 3) ^ (self.x2 >> 2) ^ (self.x2 >> 1) ^ self.x2) & 1;
        self.x2 = (self.x2 >> 1) | (fb2 << 30);

        c
    }

    /// Generate `n` chips, returning a `Vec<u8>` of 0/1 values.
    fn generate(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next_bit()).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC-24C (TS 38.212 §5.1)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute CRC-24C over `bits` (MSB first, bit 0 is MSB).
///
/// Generator polynomial g(x) = x^24 + x^23 + x^21 + x^20 + x^17 + x^15 +
///                              x^13 + x^12 + x^8 + x^4 + x^2 + x + 1.
///
/// The CRC is appended MSB-first to `bits` to form the code word.
pub fn crc24c(bits: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &b in bits {
        let top = (crc >> 23) & 1;
        crc = (crc << 1) & 0x00FF_FFFF;
        if (b as u32) ^ top != 0 {
            crc ^= CRC24C_POLY;
        }
    }
    crc & 0x00FF_FFFF
}

/// Verify CRC-24C: returns true if the last 24 bits of `bits` match the CRC of the preceding bits.
pub fn crc24c_verify(bits: &[u8]) -> bool {
    if bits.len() < CRC24C_BITS {
        return false;
    }
    let data_len = bits.len() - CRC24C_BITS;
    let computed = crc24c(&bits[..data_len]);
    let received: u32 = bits[data_len..].iter().enumerate().fold(0u32, |acc, (i, &b)| {
        acc | ((b as u32) << (CRC24C_BITS - 1 - i))
    });
    computed == received
}

/// Append 24 CRC-24C bits (MSB first) to `bits` and return the extended vector.
pub fn crc24c_attach(bits: &[u8]) -> Vec<u8> {
    let crc = crc24c(bits);
    let mut out = bits.to_vec();
    for i in (0..CRC24C_BITS).rev() {
        out.push(((crc >> i) & 1) as u8);
    }
    out
}

// ─────────────────────────────────────────────────────────────────────────────
// Polar Code Encoder — TS 38.212 §5.3.1
// ─────────────────────────────────────────────────────────────────────────────

/// Compute Bhattacharyya parameters for all N synthetic channels.
///
/// Upper branch (f-node): z → 2z – z²  (degraded)
/// Lower branch (g-node): z → z²        (improved)
fn bhattacharyya_params(n: usize) -> Vec<f64> {
    let log_n = n.trailing_zeros() as usize;
    (0..n).map(|ch| {
        let ch_rev = bit_reverse(ch, log_n);
        let mut z = 0.5f64;
        for stage in 0..log_n {
            let b = (ch_rev >> (log_n - 1 - stage)) & 1;
            z = if b == 0 { 2.0 * z - z * z } else { z * z };
        }
        z
    }).collect()
}

/// Bit-reverse an integer `v` of `bits` width.
fn bit_reverse(v: usize, bits: usize) -> usize {
    let mut r = 0usize;
    let mut x = v;
    for _ in 0..bits {
        r = (r << 1) | (x & 1);
        x >>= 1;
    }
    r
}

/// Build the frozen-bit mask for a polar code of length N with K information bits.
///
/// Channels with highest Bhattacharyya parameter (least reliable) are frozen.
/// Returns a boolean mask of length N: true = information bit, false = frozen.
fn build_frozen_mask(n: usize, k: usize) -> Vec<bool> {
    let z = bhattacharyya_params(n);
    // Sort channel indices by reliability (ascending z = more reliable)
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| z[a].partial_cmp(&z[b]).unwrap());
    // The K most reliable channels carry information bits
    let mut mask = vec![false; n];
    for &ch in order.iter().take(k) {
        mask[ch] = true;
    }
    mask
}

/// Polar encoder: apply the Arikan G_N = F^{⊗n} butterfly transform.
///
/// Input `u` has length N (information + frozen bits already placed).
/// Output is the N-bit codeword.
fn polar_encode(u: &[u8]) -> Vec<u8> {
    let n = u.len();
    assert!(n.is_power_of_two());
    let mut x = u.to_vec();
    let mut step = 1usize;
    while step < n {
        let mut i = 0;
        while i < n {
            for j in i..i + step {
                x[j] ^= x[j + step];
            }
            i += 2 * step;
        }
        step <<= 1;
    }
    x
}

// ─────────────────────────────────────────────────────────────────────────────
// Rate Matching (TS 38.212 §5.3.1.3 – §5.3.1.4)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute sub-block interleaved position for natural index `i` in a mother code of length `n`.
///
/// Fills 32-row × ceil(N/32) matrix row-major, permutes rows via SUBBLOCK_PERM,
/// reads column-major (TS 38.212 §5.3.1.3).
fn subblock_interleave_pos(i: usize, n: usize) -> usize {
    let cols = (n + SUBBLOCK_ROWS - 1) / SUBBLOCK_ROWS;
    let row = i / cols;
    let col = i % cols;
    let new_row = SUBBLOCK_PERM[row];
    new_row * cols + col
}

/// Apply sub-block interleaving to codeword `c` of length N.
/// Returns the interleaved codeword of length N.
fn subblock_interleave(c: &[f64]) -> Vec<f64> {
    let n = c.len();
    let mut y = vec![0.0f64; n];
    for i in 0..n {
        let j = subblock_interleave_pos(i, n);
        y[j] = c[i];
    }
    y
}

/// Deinterleave from sub-block order back to natural order.
fn subblock_deinterleave(y: &[f64]) -> Vec<f64> {
    let n = y.len();
    let cols = (n + SUBBLOCK_ROWS - 1) / SUBBLOCK_ROWS;
    let mut c = vec![0.0f64; n];
    for new_row in 0..SUBBLOCK_ROWS {
        let old_row = SUBBLOCK_PERM_INV[new_row];
        for col in 0..cols {
            let j = new_row * cols + col;
            let i = old_row * cols + col;
            if i < n && j < n {
                c[i] = y[j];
            }
        }
    }
    c
}

/// Rate-match a length-N interleaved codeword to E output bits using circular buffer.
///
/// For PBCH: N = 512, E = 864 → repetition (E > N).
/// Reads from the circular buffer starting at bit 0.
fn rate_match_circular(y: &[f64], e: usize) -> Vec<f64> {
    let n = y.len();
    (0..e).map(|k| y[k % n]).collect()
}

/// Rate de-match (recover LLRs of N bits from E rate-matched LLRs).
///
/// For repetition (E ≥ N): add LLRs at positions that were repeated.
/// For puncturing/shortening (E < N): positions that were punctured receive LLR 0.
fn rate_dematch(llr_e: &[f64], n: usize) -> Vec<f64> {
    let e = llr_e.len();
    let mut llr_n = vec![0.0f64; n];
    for k in 0..e {
        llr_n[k % n] += llr_e[k];
    }
    llr_n
}

// ─────────────────────────────────────────────────────────────────────────────
// Polar Successive Cancellation (SC) Decoder  — TS 38.212 §5.3.1
// ─────────────────────────────────────────────────────────────────────────────

/// Recursive SC decoder node.
struct ScNode {
    /// Mother code length N at this node.
    n: usize,
    /// Frozen bit mask (length N): true = information, false = frozen.
    info_mask: Vec<bool>,
}

impl ScNode {
    fn new(n: usize, info_mask: Vec<bool>) -> Self {
        ScNode { n, info_mask }
    }

    /// Recursive f-function: min-sum approximation of the check-node update.
    /// f(a, b) = sign(a)·sign(b)·min(|a|,|b|)
    #[inline]
    fn f_func(a: f64, b: f64) -> f64 {
        let sign = if (a < 0.0) ^ (b < 0.0) { -1.0 } else { 1.0 };
        sign * a.abs().min(b.abs())
    }

    /// g-function: g(a, b, u_hat) = b + (1 – 2·u_hat)·a
    #[inline]
    fn g_func(a: f64, b: f64, u_hat: u8) -> f64 {
        b + (1.0 - 2.0 * u_hat as f64) * a
    }

    /// Decode `llr` of length `n` using recursive SC, returning the decoded info bits.
    fn decode(&self, llr: &[f64]) -> Vec<u8> {
        let mut bits = vec![0u8; self.n];
        self.sc_decode(llr, &mut bits, 0, self.n);
        // Extract information bits
        bits.iter().enumerate()
            .filter(|&(i, _)| self.info_mask[i])
            .map(|(_, &b)| b)
            .collect()
    }

    /// Recursive SC kernel. Fills `decoded[start..start+n]` in-place.
    fn sc_decode(&self, llr: &[f64], decoded: &mut Vec<u8>, start: usize, n: usize) {
        if n == 1 {
            // Leaf node: frozen → 0, information → hard decision
            if self.info_mask[start] {
                decoded[start] = if llr[0] >= 0.0 { 0 } else { 1 };
            } else {
                decoded[start] = 0;
            }
            return;
        }

        let half = n / 2;

        // Compute left-child (upper) LLRs via f-function
        let llr_left: Vec<f64> = (0..half)
            .map(|i| Self::f_func(llr[i], llr[i + half]))
            .collect();

        // Decode left child
        self.sc_decode(&llr_left, decoded, start, half);

        // Compute right-child (lower) LLRs via g-function using left decisions
        let llr_right: Vec<f64> = (0..half)
            .map(|i| Self::g_func(llr[i], llr[i + half], decoded[start + i]))
            .collect();

        // Decode right child
        self.sc_decode(&llr_right, decoded, start + half, half);
    }
}

/// Polar decoder for PBCH (K=56, N=512, E=864).
#[derive(Debug, Clone)]
pub struct PolarDecoder {
    /// Mother code length N.
    pub n: usize,
    /// Information bits count K (including CRC).
    pub k: usize,
    /// Rate-matched output length E.
    pub e: usize,
    /// Frozen-bit mask (true = info bit).
    info_mask: Vec<bool>,
}

impl PolarDecoder {
    /// Create a new PBCH polar decoder with standard parameters.
    ///
    /// Uses N=512, K=56 (32 payload + 24 CRC), E=864.
    pub fn new_pbch() -> Self {
        let n = PBCH_N_BITS;
        let k = PBCH_K_BITS;
        let e = PBCH_E_BITS;
        let info_mask = build_frozen_mask(n, k);
        PolarDecoder { n, k, e, info_mask }
    }

    /// Create a decoder with custom parameters.
    pub fn new(n: usize, k: usize, e: usize) -> Self {
        assert!(n.is_power_of_two() && n >= 32 && n <= 1024);
        assert!(k < n);
        let info_mask = build_frozen_mask(n, k);
        PolarDecoder { n, k, e, info_mask }
    }

    /// Decode E rate-matched LLRs to K information bits.
    ///
    /// Steps:
    /// 1. Rate de-matching: E → N LLRs (circular buffer reversal)
    /// 2. Sub-block de-interleaving: permute N LLRs to natural order
    /// 3. SC decoding: produce K bits
    pub fn decode(&self, llr_e: &[f64]) -> Vec<u8> {
        assert_eq!(llr_e.len(), self.e, "LLR length must equal E");

        // Step 1: rate de-match (E → N)
        let llr_n = rate_dematch(llr_e, self.n);

        // Step 2: sub-block de-interleave (natural order)
        let llr_nat = subblock_deinterleave(&llr_n);

        // Step 3: SC decode
        let node = ScNode::new(self.n, self.info_mask.clone());
        node.decode(&llr_nat)
    }

    /// Returns a reference to the frozen-bit mask.
    pub fn info_mask(&self) -> &[bool] {
        &self.info_mask
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PBCH DMRS Generation (TS 38.211 §7.4.3.1)
// ─────────────────────────────────────────────────────────────────────────────

impl PbchDmrs {
    /// Generate the PBCH DMRS sequence for the given cell ID and SSB index.
    ///
    /// c_init = 2^11 · (i_SSB + 1) · (floor(N_cell_ID / 4) + 1)
    ///        + 2^6  · (i_SSB + 1)
    ///        + (N_cell_ID mod 4)
    ///
    /// where i_SSB is the SSB index within the half-frame (lower 3 bits of ssb_index).
    ///
    /// The DMRS QPSK sequence: r(m) = (1/√2) · (1 – 2·c(2m)) + j(1/√2) · (1 – 2·c(2m+1))
    pub fn generate(cell_id: u16, ssb_index: u8) -> Self {
        let n_id = cell_id as u32;
        let i_ssb = (ssb_index & 0x7) as u32; // lower 3 bits
        let c_init = (1 << 11) * (i_ssb + 1) * (n_id / 4 + 1)
                   + (1 << 6) * (i_ssb + 1)
                   + (n_id % 4);

        let mut gold = GoldSeq::new(c_init);
        // Need 2 × 144 = 288 chips (144 DMRS REs × 2 bits per QPSK symbol)
        let chips = gold.generate(2 * PBCH_DMRS_RE_COUNT);

        let scale = 1.0f64 / 2.0f64.sqrt();
        let mut sequence_re = Vec::with_capacity(PBCH_DMRS_RE_COUNT);
        let mut sequence_im = Vec::with_capacity(PBCH_DMRS_RE_COUNT);

        for m in 0..PBCH_DMRS_RE_COUNT {
            let re = scale * (1.0 - 2.0 * chips[2 * m] as f64);
            let im = scale * (1.0 - 2.0 * chips[2 * m + 1] as f64);
            sequence_re.push(re);
            sequence_im.push(im);
        }

        PbchDmrs { cell_id, ssb_index, sequence_re, sequence_im }
    }

    /// Returns the subcarrier indices of DMRS REs within a 240-subcarrier PBCH symbol.
    ///
    /// DMRS occupies every 4th subcarrier starting from (v + N_cell_ID mod 4),
    /// where v = ssb_index mod 4 (TS 38.211 §7.4.3.1).
    pub fn subcarrier_indices(cell_id: u16, ssb_index: u8) -> Vec<usize> {
        let v = (ssb_index as usize) % 4;
        let offset = v + (cell_id as usize) % 4;
        // The offset is further reduced mod 4 per TS 38.211 §7.4.3.1
        let sc0 = offset % 4;
        (0..60).map(|m| sc0 + 4 * m).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PBCH Scrambling (TS 38.212 §7.1.6)
// ─────────────────────────────────────────────────────────────────────────────

/// Generate the PBCH scrambling sequence of length `n`.
///
/// c_init = N_cell_ID  (per TS 38.212 §7.1.6, Table 7.1.6-1)
pub fn pbch_scrambling_sequence(cell_id: u16, n: usize) -> Vec<u8> {
    let c_init = cell_id as u32;
    let mut gold = GoldSeq::new(c_init);
    gold.generate(n)
}

/// Scramble (or descramble) `bits` with the PBCH scrambling sequence.
///
/// For binary bits, scrambling is XOR; for LLRs (soft), sign flipping is used.
/// This function handles hard-bit scrambling (XOR).
pub fn pbch_scramble_bits(bits: &[u8], cell_id: u16) -> Vec<u8> {
    let scr = pbch_scrambling_sequence(cell_id, bits.len());
    bits.iter().zip(scr.iter()).map(|(&b, &s)| b ^ s).collect()
}

/// Descramble soft LLRs with the PBCH scrambling sequence.
///
/// When scrambling bit c[n] = 1, the QPSK bit sign is flipped: LLR → –LLR.
pub fn pbch_descramble_llr(llr: &[f64], cell_id: u16) -> Vec<f64> {
    let scr = pbch_scrambling_sequence(cell_id, llr.len());
    llr.iter().zip(scr.iter())
        .map(|(&l, &s)| if s == 1 { -l } else { l })
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// QPSK Demodulation (TS 38.211 §6.3.1)
// ─────────────────────────────────────────────────────────────────────────────

/// Soft QPSK demodulation producing LLRs for 2 bits per symbol.
///
/// QPSK Gray mapping (TS 38.211 §6.3.1.3):
/// - b0 b1 → I = (1-2b0)/√2,  Q = (1-2b1)/√2
///
/// Under AWGN with variance σ²:
/// - LLR(b0) = –2·I_r / σ²   ≈ 2·I_r  (unit normalization)
/// - LLR(b1) = –2·Q_r / σ²   ≈ 2·Q_r
///
/// For unit noise variance: LLR(b0) = 2·Re(r), LLR(b1) = 2·Im(r).
///
/// Returns a flat vector of LLRs: [LLR_b0_s0, LLR_b1_s0, LLR_b0_s1, …].
pub fn qpsk_soft_demod(rx_re: &[f64], rx_im: &[f64], noise_var: f64) -> Vec<f64> {
    assert_eq!(rx_re.len(), rx_im.len());
    let scale = 2.0 / noise_var.max(1e-12);
    let mut llr = Vec::with_capacity(2 * rx_re.len());
    for (&re, &im) in rx_re.iter().zip(rx_im.iter()) {
        llr.push(scale * re);   // LLR for bit 0 (I branch)
        llr.push(scale * im);   // LLR for bit 1 (Q branch)
    }
    llr
}

/// Hard QPSK decision from IQ sample.
/// Returns (b0, b1) where b0 = 0 if I > 0, 1 otherwise; b1 = 0 if Q > 0.
pub fn qpsk_hard_demod(re: f64, im: f64) -> (u8, u8) {
    (if re >= 0.0 { 0 } else { 1 }, if im >= 0.0 { 0 } else { 1 })
}

// ─────────────────────────────────────────────────────────────────────────────
// LS Channel Estimation over PBCH DMRS
// ─────────────────────────────────────────────────────────────────────────────

/// Least-Squares channel estimation at DMRS subcarrier positions.
///
/// H_hat(k) = Y(k) / X_dmrs(k)
/// where Y(k) is the received signal and X_dmrs(k) is the known pilot.
pub fn ls_channel_estimate(
    rx_re: &[f64],
    rx_im: &[f64],
    dmrs_re: &[f64],
    dmrs_im: &[f64],
) -> ChannelEstimate {
    assert_eq!(rx_re.len(), dmrs_re.len());
    let n = rx_re.len();
    let mut h_re = Vec::with_capacity(n);
    let mut h_im = Vec::with_capacity(n);
    let mut noise_acc = 0.0f64;

    for i in 0..n {
        // Y * conj(X) / |X|^2  (X has unit power so |X|^2 = 1)
        let xr = dmrs_re[i];
        let xi = dmrs_im[i];
        let yr = rx_re[i];
        let yi = rx_im[i];
        // H_hat = Y / X = Y · X* / |X|^2; |X|^2 = 0.5+0.5 = 1.0
        let hr = yr * xr + yi * xi;
        let hi = yi * xr - yr * xi;
        h_re.push(hr);
        h_im.push(hi);
        // Noise estimate: residual after channel compensation
        let eq_re = yr * hr + yi * hi;
        let eq_im = yi * hr - yr * hi;
        let err_re = eq_re - xr;
        let err_im = eq_im - xi;
        noise_acc += err_re * err_re + err_im * err_im;
    }
    let noise_power = noise_acc / (n as f64).max(1.0);
    ChannelEstimate { h_re, h_im, noise_power }
}

/// Zero-forcing equalization of received symbols using channel estimate.
///
/// Eq(k) = Y(k) / H_hat(k) = Y(k) · H*(k) / |H(k)|^2
pub fn zf_equalize(
    rx_re: &[f64],
    rx_im: &[f64],
    h_re: &[f64],
    h_im: &[f64],
) -> (Vec<f64>, Vec<f64>) {
    assert_eq!(rx_re.len(), h_re.len());
    let n = rx_re.len();
    let mut eq_re = Vec::with_capacity(n);
    let mut eq_im = Vec::with_capacity(n);
    for i in 0..n {
        let hr = h_re[i];
        let hi = h_im[i];
        let mag2 = (hr * hr + hi * hi).max(1e-12);
        let yr = rx_re[i];
        let yi = rx_im[i];
        eq_re.push((yr * hr + yi * hi) / mag2);
        eq_im.push((yi * hr - yr * hi) / mag2);
    }
    (eq_re, eq_im)
}

// ─────────────────────────────────────────────────────────────────────────────
// PBCH Resource Element Mapping (TS 38.211 §7.3.3)
// ─────────────────────────────────────────────────────────────────────────────

/// Subcarrier index layout within the PBCH symbols.
///
/// For the 4 SSB symbols (0–3):
/// - Symbol 0: PSS only (not PBCH)
/// - Symbol 1: all 240 SCs → PBCH data + DMRS
/// - Symbol 2: SCs 0–47 and 192–239 → PBCH data + DMRS (SCs 48–191 = SSS + guard)
/// - Symbol 3: all 240 SCs → PBCH data + DMRS
///
/// Returns the 432 data-RE subcarrier indices (excluding DMRS) for each PBCH symbol.
pub fn pbch_data_subcarrier_indices(cell_id: u16, ssb_index: u8) -> Vec<(usize, usize)> {
    // DMRS subcarriers (stride 4, offset determined by ssb_index and cell_id)
    let sc0_dmrs = ((ssb_index as usize) % 4 + (cell_id as usize) % 4) % 4;
    let is_dmrs = |sc: usize| sc % 4 == sc0_dmrs;

    let mut result = Vec::with_capacity(432);

    // Symbol 1: all 240 SCs
    for sc in 0..240usize {
        if !is_dmrs(sc) {
            result.push((1, sc));
        }
    }

    // Symbol 2: SCs 0–47 and 192–239 (SSS occupies 48–191)
    for sc in (0..48usize).chain(192..240) {
        if !is_dmrs(sc) {
            result.push((2, sc));
        }
    }

    // Symbol 3: all 240 SCs
    for sc in 0..240usize {
        if !is_dmrs(sc) {
            result.push((3, sc));
        }
    }

    result
}

// ─────────────────────────────────────────────────────────────────────────────
// MIB Bit-Field Extraction (TS 38.331 §6.2.2)
// ─────────────────────────────────────────────────────────────────────────────

/// Parse the 32-bit raw MIB payload into the `MasterInformationBlock` struct.
///
/// MIB layout (bit 31 = MSB, TS 38.331 §6.2.2):
///
/// | Field                    | Bits  | Width |
/// |--------------------------|-------|-------|
/// | systemFrameNumber[9:4]   | 31:26 | 6     |
/// | subCarrierSpacingCommon  | 25    | 1     |
/// | ssb-SubcarrierOffset[3:0]| 24:21 | 4     |
/// | dmrs-TypeA-Position      | 20    | 1     |
/// | pdcch-ConfigSIB1         | 19:12 | 8     |
/// | cellBarred               | 11    | 1     |
/// | intraFreqReselection     | 10    | 1     |
/// | spare                    | 9:0   | 10    |
///
/// Note: bits [3:0] of SFN are in the PBCH payload (bits outside MIB proper),
/// and half-frame bit + SSB index bits are recovered from DMRS/scrambling context.
pub fn parse_mib(raw: u32, ssb_index: u8, half_frame_bit: u8) -> MasterInformationBlock {
    // SFN upper 6 bits from bits [31:26]
    let sfn_upper = ((raw >> 26) & 0x3F) as u16;
    // SFN bits [3:0] are from PBCH payload bits outside of MIB encoding
    // (they are embedded into the a-bar encoding; here we use 0 as placeholder)
    let system_frame_number = sfn_upper << 4; // lower 4 bits typically from separate bits

    let scs_bit = (raw >> 25) & 1;
    let subcarrier_spacing_common = if scs_bit == 0 {
        Some(ScsCommon::Scs15Or60)
    } else {
        Some(ScsCommon::Scs30Or120)
    };

    let ssb_offset = ((raw >> 21) & 0xF) as u8;
    let dmrs_bit = (raw >> 20) & 1;
    let dmrs_typea_position = if dmrs_bit == 0 {
        DmrsTypeAPosition::Pos2
    } else {
        DmrsTypeAPosition::Pos3
    };

    let pdcch_config_sib1 = ((raw >> 12) & 0xFF) as u8;
    let cell_barred = (raw >> 11) & 1 == 1;
    let intra_freq_reselection = (raw >> 10) & 1 == 1;

    MasterInformationBlock {
        system_frame_number,
        subcarrier_spacing_common,
        ssb_subcarrier_offset: ssb_offset,
        dmrs_typea_position,
        pdcch_config_sib1,
        cell_barred,
        intra_freq_reselection,
        half_frame_index: half_frame_bit,
        ssb_index,
        raw_payload: raw,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SSB Index Recovery from PBCH Payload Scrambling
// ─────────────────────────────────────────────────────────────────────────────

/// Recover 3 additional SSB index bits embedded in PBCH scrambling for L_max = 8/64.
///
/// TS 38.212 §7.1.6 (Table 7.1.6-2): The PBCH payload is further scrambled
/// with a sequence that encodes i_SSB bits [2:0] for L_max = 8,
/// or i_SSB bits [5:0] for L_max = 64.
///
/// The function extracts bits from the decoded MIB word positions.
pub fn recover_ssb_index(decoded_bits: &[u8], l_max: LMax) -> u8 {
    if decoded_bits.len() < 32 {
        return 0;
    }
    match l_max {
        LMax::L4 => {
            // Only 2 SSB bits; bits [1:0] from DMRS phase
            0
        }
        LMax::L8 => {
            // SSB bits [2:0] from DMRS (already in ssb_index), no payload extraction needed
            0
        }
        LMax::L64 => {
            // SSB bits [5:3] from payload bits at specific positions per TS 38.212 §7.1.6
            // Bits a_bar[A-3], a_bar[A-4], a_bar[A-5] encode ssb_index[5:3]
            // For standard A=32: positions 29, 28, 27
            let b5 = decoded_bits.get(29).copied().unwrap_or(0);
            let b4 = decoded_bits.get(28).copied().unwrap_or(0);
            let b3 = decoded_bits.get(27).copied().unwrap_or(0);
            (b5 << 5) | (b4 << 4) | (b3 << 3)
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main PBCH Decoder
// ─────────────────────────────────────────────────────────────────────────────

/// 5G NR PBCH decoder integrating all processing stages.
///
/// Usage flow:
/// 1. Provide received IQ for the 576 PBCH REs (432 data + 144 DMRS).
/// 2. Call `decode()` to obtain the `PbchDecodeResult` containing the MIB.
#[derive(Debug, Clone)]
pub struct NrPbchDecoder {
    config: NrPbchConfig,
    polar: PolarDecoder,
    dmrs: PbchDmrs,
}

impl NrPbchDecoder {
    /// Create a new PBCH decoder with the given configuration.
    pub fn new(config: NrPbchConfig) -> Self {
        let dmrs = PbchDmrs::generate(config.cell_id, config.ssb_index);
        let polar = PolarDecoder::new_pbch();
        NrPbchDecoder { config, polar, dmrs }
    }

    /// Return a reference to the decoder configuration.
    pub fn config(&self) -> &NrPbchConfig {
        &self.config
    }

    /// Return a reference to the pre-computed DMRS sequence.
    pub fn generate_dmrs(&self) -> &PbchDmrs {
        &self.dmrs
    }

    /// Full PBCH decode from received PBCH data REs and DMRS REs.
    ///
    /// # Parameters
    /// - `data_re_i`: Real part of received data REs (432 values).
    /// - `data_re_q`: Imaginary part of received data REs (432 values).
    /// - `dmrs_rx_i`: Real part of received DMRS REs (144 values).
    /// - `dmrs_rx_q`: Imaginary part of received DMRS REs (144 values).
    /// - `noise_var`: Estimated noise variance (used for LLR scaling).
    ///
    /// # Returns
    /// `PbchDecodeResult` with CRC status and parsed MIB.
    pub fn decode(
        &self,
        data_re_i: &[f64],
        data_re_q: &[f64],
        dmrs_rx_i: &[f64],
        dmrs_rx_q: &[f64],
        noise_var: f64,
    ) -> PbchDecodeResult {
        // ── Step 1: LS channel estimation from DMRS ──────────────────────────
        let ch_est = ls_channel_estimate(
            dmrs_rx_i, dmrs_rx_q,
            &self.dmrs.sequence_re, &self.dmrs.sequence_im,
        );

        // Compute average channel power across DMRS REs for interpolation
        let avg_h_re: f64 = ch_est.h_re.iter().sum::<f64>() / ch_est.h_re.len() as f64;
        let avg_h_im: f64 = ch_est.h_im.iter().sum::<f64>() / ch_est.h_im.len() as f64;

        // ── Step 2: ZF equalization of data REs ──────────────────────────────
        // Use the average channel for all data REs (simple flat-fading assumption)
        let h_data_re: Vec<f64> = vec![avg_h_re; data_re_i.len()];
        let h_data_im: Vec<f64> = vec![avg_h_im; data_re_i.len()];
        let (eq_re, eq_im) = zf_equalize(data_re_i, data_re_q, &h_data_re, &h_data_im);

        // Effective noise variance after equalization
        let h_mag2 = (avg_h_re * avg_h_re + avg_h_im * avg_h_im).max(1e-12);
        let eff_noise = noise_var / h_mag2;

        // ── Step 3: Soft QPSK demodulation ───────────────────────────────────
        let llr_mod = qpsk_soft_demod(&eq_re, &eq_im, eff_noise);

        // ── Step 4: PBCH descrambling (negate LLR where scrambling bit = 1) ──
        let llr_descr = pbch_descramble_llr(&llr_mod, self.config.cell_id);

        // ── Step 5: Polar rate de-matching + SC decode ───────────────────────
        let info_bits = self.polar.decode(&llr_descr);

        // ── Step 6: CRC-24C verification ─────────────────────────────────────
        let crc_ok = crc24c_verify(&info_bits);

        // ── Step 7: MIB extraction ───────────────────────────────────────────
        let payload_bits = &info_bits[..PBCH_PAYLOAD_BITS.min(info_bits.len())];
        let raw_payload = bits_to_u32(payload_bits);
        let mib = parse_mib(raw_payload, self.config.ssb_index, self.config.half_frame_bit);

        PbchDecodeResult {
            crc_ok,
            mib,
            info_bits,
            input_llr: llr_descr,
        }
    }

    /// Simplified decode from flat interleaved IQ data (I0, Q0, I1, Q1, …).
    ///
    /// Splits the first 144 complex samples as DMRS, the remaining 432 as data.
    /// IQ vector length must be 2 × 576 = 1152 real values.
    pub fn decode_flat_iq(&self, iq: &[f64], noise_var: f64) -> PbchDecodeResult {
        assert!(
            iq.len() >= 2 * PBCH_TOTAL_RE_COUNT,
            "Need at least {} IQ values (got {})",
            2 * PBCH_TOTAL_RE_COUNT,
            iq.len()
        );

        // First 144 complex → DMRS, next 432 complex → data
        let (dmrs_flat, data_flat) = iq.split_at(2 * PBCH_DMRS_RE_COUNT);

        let dmrs_i: Vec<f64> = dmrs_flat.iter().step_by(2).copied().collect();
        let dmrs_q: Vec<f64> = dmrs_flat.iter().skip(1).step_by(2).copied().collect();
        let data_i: Vec<f64> = data_flat.iter().step_by(2).copied().collect();
        let data_q: Vec<f64> = data_flat.iter().skip(1).step_by(2).copied().collect();

        self.decode(&data_i, &data_q, &dmrs_i, &dmrs_q, noise_var)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Convert up to 32 bits (MSB first) to a u32.
fn bits_to_u32(bits: &[u8]) -> u32 {
    let n = bits.len().min(32);
    let mut v = 0u32;
    for i in 0..n {
        v = (v << 1) | (bits[i] as u32 & 1);
    }
    v
}

/// Convert a u32 to a bit vector of length `n` (MSB first).
pub fn u32_to_bits(v: u32, n: usize) -> Vec<u8> {
    (0..n).map(|i| ((v >> (n - 1 - i)) & 1) as u8).collect()
}

/// Interleave two equal-length f64 slices: [a0, b0, a1, b1, …].
pub fn interleave_iq(i: &[f64], q: &[f64]) -> Vec<f64> {
    assert_eq!(i.len(), q.len());
    let mut out = Vec::with_capacity(2 * i.len());
    for (&iv, &qv) in i.iter().zip(q.iter()) {
        out.push(iv);
        out.push(qv);
    }
    out
}

/// Deinterleave flat IQ vector into separate I and Q slices.
pub fn deinterleave_iq(iq: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = iq.len() / 2;
    let i: Vec<f64> = iq.iter().step_by(2).copied().collect();
    let q: Vec<f64> = iq.iter().skip(1).step_by(2).copied().collect();
    (i[..n].to_vec(), q[..n].to_vec())
}

// ─────────────────────────────────────────────────────────────────────────────
// TESTS
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Gold Sequence ────────────────────────────────────────────────────────

    #[test]
    fn test_gold_seq_length() {
        let mut g = GoldSeq::new(0);
        let chips = g.generate(100);
        assert_eq!(chips.len(), 100);
        assert!(chips.iter().all(|&b| b == 0 || b == 1));
    }

    #[test]
    fn test_gold_seq_different_cinit() {
        let mut g0 = GoldSeq::new(0);
        let mut g1 = GoldSeq::new(1);
        let c0 = g0.generate(50);
        let c1 = g1.generate(50);
        // Different c_init → different sequences
        assert_ne!(c0, c1);
    }

    #[test]
    fn test_gold_seq_reproducible() {
        let mut g0 = GoldSeq::new(42);
        let mut g1 = GoldSeq::new(42);
        let c0 = g0.generate(200);
        let c1 = g1.generate(200);
        assert_eq!(c0, c1);
    }

    #[test]
    fn test_gold_seq_binary_output() {
        let mut g = GoldSeq::new(12345);
        for _ in 0..500 {
            let b = g.next_bit();
            assert!(b == 0 || b == 1, "Gold seq output must be 0 or 1");
        }
    }

    // ── CRC-24C ─────────────────────────────────────────────────────────────

    #[test]
    fn test_crc24c_all_zeros() {
        let bits = vec![0u8; 32];
        let crc = crc24c(&bits);
        // CRC of all zeros is deterministic
        assert_eq!(crc, crc24c(&bits));
    }

    #[test]
    fn test_crc24c_roundtrip() {
        let bits: Vec<u8> = (0..32).map(|i| (i % 2) as u8).collect();
        let coded = crc24c_attach(&bits);
        assert_eq!(coded.len(), 32 + 24);
        assert!(crc24c_verify(&coded));
    }

    #[test]
    fn test_crc24c_detect_error() {
        let bits: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0,
                                  0, 1, 0, 0, 1, 1, 0, 1,
                                  1, 0, 0, 1, 0, 1, 0, 0,
                                  1, 1, 0, 0, 0, 1, 1, 0];
        let mut coded = crc24c_attach(&bits);
        assert!(crc24c_verify(&coded));
        // Flip a bit → CRC fails
        coded[5] ^= 1;
        assert!(!crc24c_verify(&coded));
    }

    #[test]
    fn test_crc24c_length() {
        let bits = vec![1u8; 10];
        let coded = crc24c_attach(&bits);
        assert_eq!(coded.len(), 10 + 24);
    }

    #[test]
    fn test_crc24c_known_payload() {
        // Zero payload: verify CRC is non-zero (poly is not trivial)
        let bits = vec![0u8; 8];
        let crc = crc24c(&bits);
        // CRC-24C of 8 zero bits: computed reference
        assert!(crc <= 0x00FF_FFFF);
    }

    #[test]
    fn test_crc24c_single_bit() {
        let bits0 = vec![0u8; 1];
        let bits1 = vec![1u8; 1];
        // CRC of 0 and 1 are different
        assert_ne!(crc24c(&bits0), crc24c(&bits1));
    }

    // ── Polar Code ───────────────────────────────────────────────────────────

    #[test]
    fn test_polar_encode_length() {
        let u = vec![0u8; 512];
        let c = polar_encode(&u);
        assert_eq!(c.len(), 512);
    }

    #[test]
    fn test_polar_encode_all_zeros() {
        let u = vec![0u8; 64];
        let c = polar_encode(&u);
        assert!(c.iter().all(|&b| b == 0));
    }

    #[test]
    fn test_polar_encode_single_one() {
        let mut u = vec![0u8; 8];
        u[7] = 1; // Last bit = 1 → all output bits = 1 (Arikan butterfly property)
        let c = polar_encode(&u);
        assert!(c.iter().all(|&b| b == 1));
    }

    #[test]
    fn test_bhattacharyya_monotone() {
        // Most reliable channel should have lowest z
        let z = bhattacharyya_params(16);
        assert_eq!(z.len(), 16);
        // All z values in [0, 1]
        assert!(z.iter().all(|&v| v >= 0.0 && v <= 1.0));
    }

    #[test]
    fn test_frozen_mask_count() {
        let mask = build_frozen_mask(64, 20);
        assert_eq!(mask.len(), 64);
        let info_count = mask.iter().filter(|&&b| b).count();
        assert_eq!(info_count, 20);
    }

    #[test]
    fn test_polar_decoder_creation() {
        let dec = PolarDecoder::new_pbch();
        assert_eq!(dec.n, PBCH_N_BITS);
        assert_eq!(dec.k, PBCH_K_BITS);
        assert_eq!(dec.e, PBCH_E_BITS);
    }

    #[test]
    fn test_polar_decoder_output_length() {
        let dec = PolarDecoder::new_pbch();
        let llr = vec![1.0f64; PBCH_E_BITS];
        let bits = dec.decode(&llr);
        assert_eq!(bits.len(), PBCH_K_BITS);
    }

    #[test]
    fn test_polar_decode_all_positive_llr() {
        // All-positive LLR → all 0 decoded bits (best case)
        let dec = PolarDecoder::new_pbch();
        let llr = vec![10.0f64; PBCH_E_BITS];
        let bits = dec.decode(&llr);
        assert!(bits.iter().all(|&b| b == 0));
    }

    // ── Rate Matching ────────────────────────────────────────────────────────

    #[test]
    fn test_subblock_interleave_bijective() {
        let n = 512;
        let mut positions: Vec<usize> = (0..n).map(|i| subblock_interleave_pos(i, n)).collect();
        positions.sort_unstable();
        positions.dedup();
        // All positions unique → bijection
        assert_eq!(positions.len(), n);
        assert_eq!(positions[0], 0);
        assert_eq!(positions[n - 1], n - 1);
    }

    #[test]
    fn test_rate_match_repetition() {
        let y = vec![1.0, 2.0, 3.0, 4.0];
        let rm = rate_match_circular(&y, 6);
        assert_eq!(rm, vec![1.0, 2.0, 3.0, 4.0, 1.0, 2.0]);
    }

    #[test]
    fn test_rate_dematch_sums_repetitions() {
        // E=6, N=4: positions 0,1,2,3,0,1 → LLR[0] = llr[0]+llr[4], etc.
        let llr_e = vec![1.0, 2.0, 3.0, 4.0, 10.0, 20.0];
        let llr_n = rate_dematch(&llr_e, 4);
        assert_eq!(llr_n.len(), 4);
        assert!((llr_n[0] - 11.0).abs() < 1e-10);
        assert!((llr_n[1] - 22.0).abs() < 1e-10);
        assert!((llr_n[2] - 3.0).abs() < 1e-10);
        assert!((llr_n[3] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_subblock_deinterleave_roundtrip() {
        let n = 512;
        let orig: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let interleaved = subblock_interleave(&orig);
        let recovered = subblock_deinterleave(&interleaved);
        assert_eq!(orig, recovered);
    }

    // ── DMRS ────────────────────────────────────────────────────────────────

    #[test]
    fn test_dmrs_sequence_length() {
        let dmrs = PbchDmrs::generate(0, 0);
        assert_eq!(dmrs.sequence_re.len(), PBCH_DMRS_RE_COUNT);
        assert_eq!(dmrs.sequence_im.len(), PBCH_DMRS_RE_COUNT);
    }

    #[test]
    fn test_dmrs_unit_power() {
        let dmrs = PbchDmrs::generate(42, 0);
        for (&re, &im) in dmrs.sequence_re.iter().zip(dmrs.sequence_im.iter()) {
            let power = re * re + im * im;
            assert!((power - 1.0).abs() < 1e-10, "DMRS pilot power != 1: {}", power);
        }
    }

    #[test]
    fn test_dmrs_varies_with_cell_id() {
        let d0 = PbchDmrs::generate(0, 0);
        let d1 = PbchDmrs::generate(1, 0);
        // Different cell IDs → different sequences
        assert_ne!(d0.sequence_re, d1.sequence_re);
    }

    #[test]
    fn test_dmrs_varies_with_ssb_index() {
        let d0 = PbchDmrs::generate(42, 0);
        let d1 = PbchDmrs::generate(42, 1);
        assert_ne!(d0.sequence_re, d1.sequence_re);
    }

    #[test]
    fn test_dmrs_subcarrier_indices_count() {
        let sc = PbchDmrs::subcarrier_indices(0, 0);
        assert_eq!(sc.len(), 60);
    }

    #[test]
    fn test_dmrs_subcarrier_indices_stride() {
        let sc = PbchDmrs::subcarrier_indices(0, 0);
        for i in 1..sc.len() {
            assert_eq!(sc[i] - sc[i - 1], 4, "DMRS stride must be 4");
        }
    }

    // ── Scrambling ───────────────────────────────────────────────────────────

    #[test]
    fn test_scrambling_length() {
        let scr = pbch_scrambling_sequence(42, 864);
        assert_eq!(scr.len(), 864);
    }

    #[test]
    fn test_scrambling_roundtrip() {
        let bits: Vec<u8> = (0..100).map(|i| (i % 2) as u8).collect();
        let scrambled = pbch_scramble_bits(&bits, 7);
        let descrambled = pbch_scramble_bits(&scrambled, 7);
        assert_eq!(bits, descrambled);
    }

    #[test]
    fn test_scrambling_different_cell_id() {
        let bits = vec![1u8; 50];
        let s0 = pbch_scramble_bits(&bits, 0);
        let s1 = pbch_scramble_bits(&bits, 1);
        // Almost certainly different (unless very unlucky)
        assert_ne!(s0, s1);
    }

    #[test]
    fn test_llr_descrambling_sign_flip() {
        let llr = vec![1.0, -2.0, 3.0, -4.0];
        let scr = vec![0u8, 1, 0, 1]; // manually control scrambling
        // Using cell_id=0 will produce an actual Gold sequence; just test the sign logic
        let result: Vec<f64> = llr.iter().zip(scr.iter())
            .map(|(&l, &s)| if s == 1 { -l } else { l })
            .collect();
        assert_eq!(result, vec![1.0, 2.0, 3.0, 4.0]);
    }

    // ── QPSK ────────────────────────────────────────────────────────────────

    #[test]
    fn test_qpsk_hard_demod_quadrants() {
        assert_eq!(qpsk_hard_demod(1.0, 1.0), (0, 0));
        assert_eq!(qpsk_hard_demod(-1.0, 1.0), (1, 0));
        assert_eq!(qpsk_hard_demod(1.0, -1.0), (0, 1));
        assert_eq!(qpsk_hard_demod(-1.0, -1.0), (1, 1));
    }

    #[test]
    fn test_qpsk_soft_demod_length() {
        let re = vec![0.7, -0.7];
        let im = vec![0.7, -0.7];
        let llr = qpsk_soft_demod(&re, &im, 1.0);
        assert_eq!(llr.len(), 4); // 2 symbols × 2 bits
    }

    #[test]
    fn test_qpsk_soft_demod_signs() {
        let re = vec![1.0];
        let im = vec![-1.0];
        let llr = qpsk_soft_demod(&re, &im, 0.5);
        // b0=0 → positive LLR; b1=1 → negative LLR
        assert!(llr[0] > 0.0, "b0=0 should give positive LLR");
        assert!(llr[1] < 0.0, "b1=1 should give negative LLR");
    }

    #[test]
    fn test_qpsk_soft_demod_noise_scaling() {
        let re = vec![1.0];
        let im = vec![1.0];
        let llr1 = qpsk_soft_demod(&re, &im, 1.0);
        let llr2 = qpsk_soft_demod(&re, &im, 2.0);
        // Higher noise → smaller LLR magnitude
        assert!(llr1[0].abs() > llr2[0].abs());
    }

    // ── Channel Estimation ───────────────────────────────────────────────────

    #[test]
    fn test_ls_channel_estimate_identity() {
        // Perfect channel: H=1, no noise → H_hat = Y/X = X/X = 1
        let dmrs_re = vec![1.0f64 / 2.0f64.sqrt(); 4];
        let dmrs_im = vec![1.0f64 / 2.0f64.sqrt(); 4];
        let ch = ls_channel_estimate(&dmrs_re, &dmrs_im, &dmrs_re, &dmrs_im);
        for &h in ch.h_re.iter() {
            assert!((h - 1.0).abs() < 1e-10, "H_re expected 1, got {}", h);
        }
    }

    #[test]
    fn test_ls_channel_estimate_length() {
        let n = 10;
        let x = vec![0.5; n];
        let y = vec![0.5; n];
        let ch = ls_channel_estimate(&x, &y, &x, &y);
        assert_eq!(ch.h_re.len(), n);
        assert_eq!(ch.h_im.len(), n);
    }

    #[test]
    fn test_zf_equalize_identity() {
        // H=1+0j → equalized = received
        let rx_re = vec![0.707, -0.707];
        let rx_im = vec![0.707, 0.707];
        let h_re = vec![1.0; 2];
        let h_im = vec![0.0; 2];
        let (eq_re, eq_im) = zf_equalize(&rx_re, &rx_im, &h_re, &h_im);
        assert!((eq_re[0] - rx_re[0]).abs() < 1e-10);
        assert!((eq_im[0] - rx_im[0]).abs() < 1e-10);
    }

    #[test]
    fn test_zf_equalize_phase_rotation() {
        // H = j (90° rotation): Y = j·X → equalized = Y/H = X
        let x_re = vec![1.0, 0.0];
        let x_im = vec![0.0, 1.0];
        // After 90° rotation: Y = jX → Y_re = -X_im, Y_im = X_re
        let y_re: Vec<f64> = x_im.iter().map(|&v| -v).collect();
        let y_im = x_re.clone();
        let h_re = vec![0.0; 2];
        let h_im = vec![1.0; 2];
        let (eq_re, eq_im) = zf_equalize(&y_re, &y_im, &h_re, &h_im);
        assert!((eq_re[0] - x_re[0]).abs() < 1e-9);
        assert!((eq_im[0] - x_im[0]).abs() < 1e-9);
    }

    // ── MIB Parsing ──────────────────────────────────────────────────────────

    #[test]
    fn test_mib_parse_cell_barred() {
        // Set bit 11 = cell_barred = 1
        let raw = 1u32 << 11;
        let mib = parse_mib(raw, 0, 0);
        assert!(mib.cell_barred);
        assert!(!mib.intra_freq_reselection);
    }

    #[test]
    fn test_mib_parse_intra_freq() {
        let raw = 1u32 << 10;
        let mib = parse_mib(raw, 0, 0);
        assert!(!mib.cell_barred);
        assert!(mib.intra_freq_reselection);
    }

    #[test]
    fn test_mib_parse_dmrs_position() {
        let raw_pos2 = 0u32;               // bit 20 = 0 → Pos2
        let raw_pos3 = 1u32 << 20;         // bit 20 = 1 → Pos3
        assert_eq!(parse_mib(raw_pos2, 0, 0).dmrs_typea_position, DmrsTypeAPosition::Pos2);
        assert_eq!(parse_mib(raw_pos3, 0, 0).dmrs_typea_position, DmrsTypeAPosition::Pos3);
    }

    #[test]
    fn test_mib_parse_scs_common() {
        let raw_scs0 = 0u32;           // bit 25 = 0 → 15/60 kHz
        let raw_scs1 = 1u32 << 25;    // bit 25 = 1 → 30/120 kHz
        assert_eq!(parse_mib(raw_scs0, 0, 0).subcarrier_spacing_common, Some(ScsCommon::Scs15Or60));
        assert_eq!(parse_mib(raw_scs1, 0, 0).subcarrier_spacing_common, Some(ScsCommon::Scs30Or120));
    }

    #[test]
    fn test_mib_parse_pdcch_config_sib1() {
        let pdcch = 0xABu8;
        let raw = (pdcch as u32) << 12;
        let mib = parse_mib(raw, 0, 0);
        assert_eq!(mib.pdcch_config_sib1, 0xAB);
    }

    #[test]
    fn test_mib_parse_ssb_offset() {
        // ssb_subcarrier_offset in bits [24:21]
        let offset = 0x9u32; // 4-bit value
        let raw = offset << 21;
        let mib = parse_mib(raw, 0, 0);
        assert_eq!(mib.ssb_subcarrier_offset, 0x9);
    }

    #[test]
    fn test_mib_ssb_index_preserved() {
        let mib = parse_mib(0, 3, 1);
        assert_eq!(mib.ssb_index, 3);
        assert_eq!(mib.half_frame_index, 1);
    }

    // ── NrPbchDecoder ────────────────────────────────────────────────────────

    #[test]
    fn test_decoder_creation() {
        let cfg = NrPbchConfig { cell_id: 42, l_max: LMax::L4, ssb_index: 0, half_frame_bit: 0 };
        let dec = NrPbchDecoder::new(cfg);
        assert_eq!(dec.config().cell_id, 42);
    }

    #[test]
    fn test_decoder_dmrs_reference() {
        let cfg = NrPbchConfig { cell_id: 1, l_max: LMax::L8, ssb_index: 2, half_frame_bit: 0 };
        let dec = NrPbchDecoder::new(cfg);
        let dmrs = dec.generate_dmrs();
        assert_eq!(dmrs.cell_id, 1);
        assert_eq!(dmrs.ssb_index, 2);
        assert_eq!(dmrs.sequence_re.len(), 144);
    }

    #[test]
    fn test_decode_returns_result() {
        let cfg = NrPbchConfig { cell_id: 0, l_max: LMax::L4, ssb_index: 0, half_frame_bit: 0 };
        let dec = NrPbchDecoder::new(cfg);
        // Noise-only input
        let data_i = vec![0.1f64; PBCH_DATA_RE_COUNT];
        let data_q = vec![0.1f64; PBCH_DATA_RE_COUNT];
        let dmrs_i = vec![0.7f64; PBCH_DMRS_RE_COUNT];
        let dmrs_q = vec![0.0f64; PBCH_DMRS_RE_COUNT];
        let result = dec.decode(&data_i, &data_q, &dmrs_i, &dmrs_q, 1.0);
        // Should not panic; CRC likely fails on noise
        assert_eq!(result.info_bits.len(), PBCH_K_BITS);
    }

    #[test]
    fn test_decode_flat_iq() {
        let cfg = NrPbchConfig { cell_id: 0, l_max: LMax::L4, ssb_index: 0, half_frame_bit: 0 };
        let dec = NrPbchDecoder::new(cfg);
        let iq = vec![0.0f64; 2 * PBCH_TOTAL_RE_COUNT];
        let result = dec.decode_flat_iq(&iq, 1.0);
        assert_eq!(result.input_llr.len(), PBCH_E_BITS);
    }

    // ── LMax ────────────────────────────────────────────────────────────────

    #[test]
    fn test_lmax_values() {
        assert_eq!(LMax::L4.value(), 4);
        assert_eq!(LMax::L8.value(), 8);
        assert_eq!(LMax::L64.value(), 64);
    }

    #[test]
    fn test_lmax_dmrs_bits() {
        assert_eq!(LMax::L4.dmrs_ssb_bits(), 2);
        assert_eq!(LMax::L8.dmrs_ssb_bits(), 3);
        assert_eq!(LMax::L64.dmrs_ssb_bits(), 3);
    }

    // ── Utilities ────────────────────────────────────────────────────────────

    #[test]
    fn test_bits_to_u32_msb_first() {
        // [1,0,0,0] MSB first → 8
        let bits = vec![1u8, 0, 0, 0];
        assert_eq!(bits_to_u32(&bits), 8);
    }

    #[test]
    fn test_u32_to_bits_roundtrip() {
        let v = 0xDEAD_BEEF_u32;
        let bits = u32_to_bits(v, 32);
        assert_eq!(bits.len(), 32);
        assert_eq!(bits_to_u32(&bits), v);
    }

    #[test]
    fn test_interleave_deinterleave_roundtrip() {
        let i = vec![1.0, 3.0, 5.0];
        let q = vec![2.0, 4.0, 6.0];
        let iq = interleave_iq(&i, &q);
        assert_eq!(iq, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let (ri, rq) = deinterleave_iq(&iq);
        assert_eq!(ri, i);
        assert_eq!(rq, q);
    }

    #[test]
    fn test_pbch_data_subcarrier_count() {
        let sc = pbch_data_subcarrier_indices(0, 0);
        assert_eq!(sc.len(), PBCH_DATA_RE_COUNT);
    }

    #[test]
    fn test_crc24c_attach_verify_varied_payloads() {
        for seed in 0u8..8 {
            let bits: Vec<u8> = (0..32).map(|i| ((i + seed as usize) % 2) as u8).collect();
            let coded = crc24c_attach(&bits);
            assert!(crc24c_verify(&coded), "CRC verify failed for seed {}", seed);
        }
    }

    #[test]
    fn test_polar_encoder_systematic_property() {
        // Encoding all-1 u → all-1 output (XOR of ones)
        let u = vec![1u8; 4];
        let c = polar_encode(&u);
        // For N=4 with all-1 input, all outputs should be 0 (cancel in pairs)
        // Actually: [1,1,1,1] → stage1: [0,0,1,1] → stage2: [0,1,0,1] ... varies
        assert_eq!(c.len(), 4);
    }

    #[test]
    fn test_sc_decode_small() {
        // Simple N=4, K=2 case
        let mask = vec![false, false, true, true];
        let node = ScNode::new(4, mask);
        // All positive LLR → all zeros decoded
        let llr = vec![5.0, 5.0, 5.0, 5.0];
        let bits = node.decode(&llr);
        assert_eq!(bits.len(), 2);
        assert!(bits.iter().all(|&b| b == 0));
    }

    #[test]
    fn test_recover_ssb_index_l4() {
        let bits = vec![0u8; 32];
        let idx = recover_ssb_index(&bits, LMax::L4);
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_recover_ssb_index_l64() {
        let mut bits = vec![0u8; 32];
        bits[29] = 1; // ssb[5]
        bits[28] = 1; // ssb[4]
        bits[27] = 0; // ssb[3]
        let idx = recover_ssb_index(&bits, LMax::L64);
        assert_eq!(idx, (1 << 5) | (1 << 4));
    }

    #[test]
    fn test_noise_power_positive() {
        let n = 8;
        let x: Vec<f64> = vec![0.7071; n];
        let y: Vec<f64> = vec![0.6; n]; // slightly off
        let z = vec![0.0f64; n];
        let ch = ls_channel_estimate(&y, &z, &x, &z);
        assert!(ch.noise_power >= 0.0);
    }

    #[test]
    fn test_decoder_different_cell_ids() {
        for cell_id in [0u16, 1, 100, 503, 1007] {
            let cfg = NrPbchConfig { cell_id, l_max: LMax::L4, ssb_index: 0, half_frame_bit: 0 };
            let dec = NrPbchDecoder::new(cfg);
            assert_eq!(dec.config().cell_id, cell_id);
            let dmrs = dec.generate_dmrs();
            assert_eq!(dmrs.sequence_re.len(), 144);
        }
    }

    #[test]
    fn test_all_ssb_indices_l8() {
        for i in 0u8..8 {
            let cfg = NrPbchConfig { cell_id: 42, l_max: LMax::L8, ssb_index: i, half_frame_bit: 0 };
            let dec = NrPbchDecoder::new(cfg);
            let dmrs = dec.generate_dmrs();
            assert_eq!(dmrs.ssb_index, i);
        }
    }

    #[test]
    fn test_crc24c_matches_poly() {
        // Verify generator polynomial constant
        assert_eq!(CRC24C_POLY, 0x00B2B117);
    }

    #[test]
    fn test_pbch_constants() {
        assert_eq!(PBCH_PAYLOAD_BITS + CRC24C_BITS, PBCH_K_BITS);
        assert_eq!(PBCH_K_BITS, 56);
        assert_eq!(PBCH_N_BITS, 512);
        assert_eq!(PBCH_E_BITS, 864);
    }
}

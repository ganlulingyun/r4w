//! # 5G NR DMRS Processor
//!
//! Implements Demodulation Reference Signal (DMRS) generation and channel
//! estimation per 3GPP TS 38.211 Sections 6.4.1 and 7.4.1.
//!
//! ## Overview
//!
//! DMRS are known reference signals embedded in the 5G NR resource grid.
//! The UE and gNB use them for coherent demodulation by estimating the
//! radio channel. This module covers:
//!
//! - **Gold sequence generator** (TS 38.211 Section 5.2.1)
//! - **PDSCH DMRS** Type 1 (comb-2) and Type 2 (comb-3) – Section 7.4.1.1
//! - **PUSCH DMRS** – Section 6.4.1.1
//! - **PDCCH DMRS** – Section 7.4.1.3
//! - **PUCCH DMRS** (Formats 1–4) – Section 6.4.1.3
//! - **CDM groups** with Orthogonal Cover Codes (OCC)
//! - **DMRS symbol positions** (pos0/pos1/pos2/pos3, single/double-symbol)
//! - **Low-PAPR base sequences** for transform-precoded PUSCH
//! - **LS channel estimation** with linear interpolation
//!
//! ## Sequence Formula
//!
//! ```text
//! r(n) = (1/√2)(1 − 2·c(2n)) + j(1/√2)(1 − 2·c(2n+1))
//! ```
//!
//! where c(n) is the length-31 Gold sequence:
//!
//! ```text
//! c(n) = (x1(n + Nc) + x2(n + Nc)) mod 2,   Nc = 1600
//! x1(n+31) = (x1(n+3) + x1(n))            mod 2
//! x2(n+31) = (x2(n+3) + x2(n+2) + x2(n+1) + x2(n)) mod 2
//! ```
//!
//! ## Example
//!
//! ```
//! use r4w_core::nr_dmrs_processor::{
//!     NrDmrsProcessor, DmrsConfig, DmrsType, DmrsChannel, DmrsLength,
//! };
//!
//! let config = DmrsConfig {
//!     dmrs_type: DmrsType::Type1,
//!     channel: DmrsChannel::Pdsch,
//!     length: DmrsLength::SingleSymbol,
//!     additional_pos: 0,
//!     n_id: 1001,
//!     n_scid: 0,
//!     n_id_cell: 1,
//!     n_prb: 25,
//!     n_layers: 1,
//!     cdm_group: 0,
//!     occ_index: 0,
//!     power_boost_db: 0.0,
//!     transform_precoding: false,
//! };
//!
//! let proc = NrDmrsProcessor::new(config);
//! let seq = proc.generate_sequence(0, 2);    // slot 0, symbol 2
//! assert!(!seq.is_empty());
//! let positions = proc.get_dmrs_positions();
//! assert!(!positions.is_empty());
//! ```

// ---------------------------------------------------------------------------
// Complex number – minimal inline implementation (no external crates)
// ---------------------------------------------------------------------------

/// 64-bit complex number (real + imaginary f64).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex64 {
    pub re: f64,
    pub im: f64,
}

impl Complex64 {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    /// Complex conjugate.
    #[inline]
    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }

    /// Squared magnitude |z|^2.
    #[inline]
    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    /// Magnitude |z|.
    #[inline]
    pub fn norm(self) -> f64 {
        self.norm_sqr().sqrt()
    }
}

impl std::ops::Add for Complex64 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl std::ops::Sub for Complex64 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl std::ops::Mul for Complex64 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }
}

impl std::ops::Div for Complex64 {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        let d = rhs.norm_sqr();
        Self::new(
            (self.re * rhs.re + self.im * rhs.im) / d,
            (self.im * rhs.re - self.re * rhs.im) / d,
        )
    }
}

impl std::ops::Mul<f64> for Complex64 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.re * s, self.im * s)
    }
}

impl std::ops::Div<f64> for Complex64 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.re / s, self.im / s)
    }
}

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// DMRS configuration type per TS 38.211 Section 7.4.1.1.
///
/// * `Type1` – Comb-2: 6 DMRS subcarriers per PRB (CDM groups: 0, 1).
/// * `Type2` – Comb-3: 4 DMRS subcarriers per PRB (CDM groups: 0, 1, 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmrsType {
    Type1,
    Type2,
}

/// Physical channel carrying the DMRS.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmrsChannel {
    /// Physical Downlink Shared Channel (Section 7.4.1.1).
    Pdsch,
    /// Physical Uplink Shared Channel (Section 6.4.1.1).
    Pusch,
    /// Physical Downlink Control Channel (Section 7.4.1.3).
    Pdcch,
    /// Physical Uplink Control Channel (Sections 6.4.1.3.x).
    Pucch(PucchFormat),
}

/// PUCCH format selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PucchFormat {
    Format1,
    Format2,
    Format3,
    Format4,
}

/// Number of DMRS symbols per allocation (single vs double symbol).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmrsLength {
    SingleSymbol,
    DoubleSymbol,
}

/// Additional DMRS positions for PDSCH/PUSCH (pos0–pos3).
///
/// The numeric value directly maps to `additionalPosition` in the 3GPP spec.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdditionalPos {
    Pos0 = 0,
    Pos1 = 1,
    Pos2 = 2,
    Pos3 = 3,
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Full DMRS configuration.
#[derive(Debug, Clone)]
pub struct DmrsConfig {
    /// DMRS mapping type (Type 1 or Type 2).
    pub dmrs_type: DmrsType,
    /// Target channel.
    pub channel: DmrsChannel,
    /// Single-symbol or double-symbol DMRS.
    pub length: DmrsLength,
    /// Additional DMRS positions (0–3); use `AdditionalPos` values.
    pub additional_pos: u8,
    /// Scrambling ID n_ID (cell-ID range 0–1007 or UE-specific 0–65535).
    pub n_id: u32,
    /// n_SCID: 0 or 1 (selects which scrambling init to use for PDSCH).
    pub n_scid: u8,
    /// Physical cell ID (used in some c_init computations).
    pub n_id_cell: u32,
    /// Number of allocated PRBs.
    pub n_prb: u16,
    /// Number of MIMO layers.
    pub n_layers: u8,
    /// CDM group index (0 for Type1; 0-2 for Type2).
    pub cdm_group: u8,
    /// Orthogonal Cover Code (OCC) index within the CDM group.
    pub occ_index: u8,
    /// DMRS power boosting relative to data RE (dB, typically 0.0).
    pub power_boost_db: f64,
    /// Enable transform-precoded PUSCH DMRS (low-PAPR base sequences).
    pub transform_precoding: bool,
}

impl DmrsConfig {
    /// Convenience constructor with sensible defaults for PDSCH Type 1.
    pub fn pdsch_type1(n_id: u32, n_prb: u16) -> Self {
        DmrsConfig {
            dmrs_type: DmrsType::Type1,
            channel: DmrsChannel::Pdsch,
            length: DmrsLength::SingleSymbol,
            additional_pos: 0,
            n_id,
            n_scid: 0,
            n_id_cell: n_id % 1008,
            n_prb,
            n_layers: 1,
            cdm_group: 0,
            occ_index: 0,
            power_boost_db: 0.0,
            transform_precoding: false,
        }
    }

    /// Convenience constructor for PDSCH Type 2.
    pub fn pdsch_type2(n_id: u32, n_prb: u16, cdm_group: u8) -> Self {
        DmrsConfig {
            dmrs_type: DmrsType::Type2,
            channel: DmrsChannel::Pdsch,
            length: DmrsLength::SingleSymbol,
            additional_pos: 0,
            n_id,
            n_scid: 0,
            n_id_cell: n_id % 1008,
            n_prb,
            n_layers: 1,
            cdm_group: cdm_group.min(2),
            occ_index: 0,
            power_boost_db: 0.0,
            transform_precoding: false,
        }
    }

    /// Convenience constructor for PDCCH DMRS.
    pub fn pdcch(n_id_cell: u32, n_prb: u16) -> Self {
        DmrsConfig {
            dmrs_type: DmrsType::Type1,
            channel: DmrsChannel::Pdcch,
            length: DmrsLength::SingleSymbol,
            additional_pos: 0,
            n_id: n_id_cell,
            n_scid: 0,
            n_id_cell,
            n_prb,
            n_layers: 1,
            cdm_group: 0,
            occ_index: 0,
            power_boost_db: 0.0,
            transform_precoding: false,
        }
    }

    /// Convenience constructor for PUSCH.
    pub fn pusch(n_id: u32, n_prb: u16, transform_precoding: bool) -> Self {
        DmrsConfig {
            dmrs_type: DmrsType::Type1,
            channel: DmrsChannel::Pusch,
            length: DmrsLength::SingleSymbol,
            additional_pos: 0,
            n_id,
            n_scid: 0,
            n_id_cell: n_id % 1008,
            n_prb,
            n_layers: 1,
            cdm_group: 0,
            occ_index: 0,
            power_boost_db: 0.0,
            transform_precoding,
        }
    }
}

// ---------------------------------------------------------------------------
// Gold sequence generator (TS 38.211 Section 5.2.1)
// ---------------------------------------------------------------------------

/// Length-31 Gold sequence generator.
///
/// Implements TS 38.211 Section 5.2.1:
///
/// * x1(n+31) = (x1(n+3) + x1(n)) mod 2
/// * x2(n+31) = (x2(n+3) + x2(n+2) + x2(n+1) + x2(n)) mod 2
/// * c(n) = (x1(n + Nc) + x2(n + Nc)) mod 2,  Nc = 1600
///
/// The sequence is used to form DMRS complex symbols via:
/// r(n) = (1/√2)(1 − 2c(2n)) + j(1/√2)(1 − 2c(2n+1))
pub struct GoldSequence {
    x1: u32,
    x2: u32,
}

/// Nc offset in the Gold sequence initialisation.
const NC: usize = 1600;
/// Degree of the m-sequence polynomials.
const GOLD_DEGREE: usize = 31;

impl GoldSequence {
    /// Initialise the generator from `c_init` per the 3GPP definition.
    ///
    /// The x1 initial state is always `1` (i.e. x1(0)=1, rest 0).
    /// The x2 initial state is the 31-bit representation of `c_init`.
    pub fn new(c_init: u32) -> Self {
        // Initialise x1: x1(0)=1
        let mut x1: u32 = 1u32;
        // Initialise x2 from c_init (lower 31 bits)
        let mut x2: u32 = c_init & 0x7FFF_FFFF;

        // Advance Nc steps
        for _ in 0..NC {
            // x1: tap at positions 3 and 0 (polynomial: 1 + x^3 + x^31)
            let x1_new_bit = ((x1 >> 3) ^ x1) & 1;
            x1 = (x1 >> 1) | (x1_new_bit << (GOLD_DEGREE - 1));

            // x2: tap at positions 3,2,1,0 (polynomial: 1 + x + x^2 + x^3 + x^31)
            let x2_new_bit = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
            x2 = (x2 >> 1) | (x2_new_bit << (GOLD_DEGREE - 1));
        }

        Self { x1, x2 }
    }

    /// Generate `length` bits of the Gold sequence.
    pub fn generate(&mut self, length: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(length);
        for _ in 0..length {
            let c = ((self.x1 ^ self.x2) & 1) as u8;
            out.push(c);

            let x1_new_bit = ((self.x1 >> 3) ^ self.x1) & 1;
            self.x1 = (self.x1 >> 1) | (x1_new_bit << (GOLD_DEGREE - 1));

            let x2_new_bit = ((self.x2 >> 3) ^ (self.x2 >> 2) ^ (self.x2 >> 1) ^ self.x2) & 1;
            self.x2 = (self.x2 >> 1) | (x2_new_bit << (GOLD_DEGREE - 1));
        }
        out
    }
}

/// Stateless Gold sequence generator: compute `length` bits from `c_init`.
pub fn gold_sequence(c_init: u32, length: usize) -> Vec<u8> {
    GoldSequence::new(c_init).generate(length)
}

/// Convert a Gold bit sequence to complex DMRS symbols.
///
/// r(n) = (1/√2)(1 − 2·c(2n)) + j·(1/√2)(1 − 2·c(2n+1))
pub fn gold_to_dmrs(bits: &[u8]) -> Vec<Complex64> {
    let inv_sqrt2 = 1.0_f64 / 2.0_f64.sqrt();
    let n = bits.len() / 2;
    (0..n)
        .map(|i| {
            let re = inv_sqrt2 * (1.0 - 2.0 * bits[2 * i] as f64);
            let im = inv_sqrt2 * (1.0 - 2.0 * bits[2 * i + 1] as f64);
            Complex64::new(re, im)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Orthogonal Cover Codes (OCC) per TS 38.211
// ---------------------------------------------------------------------------

/// OCC for time-domain CDM (length 2) – TS 38.211 Table 7.4.1.1.2-1.
///
/// For Type 1 DMRS, the CDM group uses a 2-element OCC:
/// * Port 0 (OCC index 0): [+1, +1]
/// * Port 1 (OCC index 1): [+1, −1]
pub const OCC_LENGTH2: [[f64; 2]; 2] = [[1.0, 1.0], [1.0, -1.0]];

/// OCC for time-domain CDM (length 4) – TS 38.211 Table 7.4.1.1.2-3.
///
/// Used for double-symbol DMRS:
/// * OCC 0: [+1, +1, +1, +1]
/// * OCC 1: [+1, −1, +1, −1]
/// * OCC 2: [+1, +1, −1, −1]
/// * OCC 3: [+1, −1, −1, +1]
pub const OCC_LENGTH4: [[f64; 4]; 4] = [
    [1.0, 1.0, 1.0, 1.0],
    [1.0, -1.0, 1.0, -1.0],
    [1.0, 1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0, 1.0],
];

/// OCC for frequency-domain CDM – Type 2, two subcarriers per CDM group.
///
/// * OCC 0: [+1, +1]
/// * OCC 1: [+1, −1]
pub const OCC_FREQ_LENGTH2: [[f64; 2]; 2] = [[1.0, 1.0], [1.0, -1.0]];

// ---------------------------------------------------------------------------
// DMRS subcarrier mapping helpers
// ---------------------------------------------------------------------------

/// Return the subcarrier offsets for DMRS Type 1 within one PRB (12 subcarriers).
///
/// CDM group 0: k = {0, 2, 4, 6, 8, 10}
/// CDM group 1: k = {1, 3, 5, 7, 9, 11}
pub fn type1_subcarrier_offsets(cdm_group: u8) -> Vec<u32> {
    (0..6u32)
        .map(|i| 2 * i + (cdm_group & 1) as u32)
        .collect()
}

/// Return the subcarrier offsets for DMRS Type 2 within one PRB (12 subcarriers).
///
/// CDM group 0: k = {0, 1, 6, 7}
/// CDM group 1: k = {2, 3, 8, 9}
/// CDM group 2: k = {4, 5, 10, 11}
pub fn type2_subcarrier_offsets(cdm_group: u8) -> Vec<u32> {
    let base = (cdm_group.min(2) as u32) * 2;
    vec![base, base + 1, base + 6, base + 7]
}

/// PDCCH DMRS uses every 4th subcarrier within a CORESET (Section 7.4.1.3).
///
/// Returns offsets 0, 4, 8 within a REG (Resource-Element Group of 12 subcarriers).
pub fn pdcch_subcarrier_offsets() -> Vec<u32> {
    vec![1, 5, 9]
}

// ---------------------------------------------------------------------------
// c_init computation per channel
// ---------------------------------------------------------------------------

/// Compute c_init for PDSCH DMRS per TS 38.211 Section 7.4.1.1.2.
///
/// c_init = (2^17 * (14*n_s_f_mu + l + 1) * (2*n_ID + 1) + 2*n_ID + n_SCID) mod 2^31
///
/// where:
/// * n_s_f_mu = slot number within a frame (0-based, subcarrier spacing dependent)
/// * l = OFDM symbol number within the slot
/// * n_ID = scrambling identity (n_SCID selects between n_ID^0 and n_ID^1)
/// * n_SCID = 0 or 1
pub fn pdsch_dmrs_c_init(n_s_f_mu: u32, l: u32, n_id: u32, n_scid: u8) -> u32 {
    let a = (14 * n_s_f_mu + l + 1) as u64;
    let b = (2 * n_id as u64 + 1) as u64;
    let c_init = ((1u64 << 17) * a * b + 2 * n_id as u64 + n_scid as u64) & 0x7FFF_FFFF;
    c_init as u32
}

/// Compute c_init for PUSCH DMRS (CP-OFDM) per TS 38.211 Section 6.4.1.1.1.2.
///
/// c_init = (2^17 * (14*n_s_f_mu + l + 1) * (2*n_ID + 1) + 2*n_ID + n_SCID) mod 2^31
///
/// The formula matches PDSCH; n_ID for PUSCH is the PUSCH scrambling ID.
pub fn pusch_dmrs_c_init(n_s_f_mu: u32, l: u32, n_id: u32, n_scid: u8) -> u32 {
    pdsch_dmrs_c_init(n_s_f_mu, l, n_id, n_scid)
}

/// Compute c_init for PDCCH DMRS per TS 38.211 Section 7.4.1.3.1.
///
/// c_init = (2^17 * (14*n_s_f_mu + l + 1) * (2*n_ID_cell + 1) + n_RNTI*2^15 + n_ID_cell) mod 2^31
///
/// For the simplified case (n_RNTI = 0):
/// c_init = (2^17 * (14*n_s_f_mu + l + 1) * (2*n_ID + 1) + n_ID) mod 2^31
pub fn pdcch_dmrs_c_init(n_s_f_mu: u32, l: u32, n_id: u32) -> u32 {
    let a = (14 * n_s_f_mu + l + 1) as u64;
    let b = (2 * n_id as u64 + 1) as u64;
    let c_init = ((1u64 << 17) * a * b + n_id as u64) & 0x7FFF_FFFF;
    c_init as u32
}

/// Compute c_init for PUCCH Format 2 DMRS per TS 38.211 Section 6.4.1.3.2.2.
///
/// c_init = (2^17 * (14*n_s_f_mu + l + 1) * (2*n_ID + 1) + 2*n_ID) mod 2^31
pub fn pucch_f2_dmrs_c_init(n_s_f_mu: u32, l: u32, n_id: u32) -> u32 {
    let a = (14 * n_s_f_mu + l + 1) as u64;
    let b = (2 * n_id as u64 + 1) as u64;
    let c_init = ((1u64 << 17) * a * b + 2 * n_id as u64) & 0x7FFF_FFFF;
    c_init as u32
}

// ---------------------------------------------------------------------------
// DMRS symbol positions (TS 38.211 Section 7.4.1.1.2 Tables)
// ---------------------------------------------------------------------------

/// Return DMRS OFDM symbol positions within a slot for PDSCH/PUSCH.
///
/// Per TS 38.211 Tables 7.4.1.1.2-3 and 7.4.1.1.2-4.
///
/// # Parameters
/// * `length` – Single or double symbol DMRS
/// * `additional_pos` – 0 to 3 (additional DMRS positions)
/// * `n_symbols` – number of allocated OFDM symbols (1-14)
/// * `start_symbol` – first OFDM symbol of the allocation (0-based within slot)
///
/// Returns the list of DMRS OFDM symbol indices (absolute within the slot).
pub fn dmrs_symbol_positions(
    length: DmrsLength,
    additional_pos: u8,
    n_symbols: u8,
    start_symbol: u8,
) -> Vec<u8> {
    let l0 = start_symbol; // first DMRS always at start_symbol

    match length {
        DmrsLength::SingleSymbol => {
            // Table 7.4.1.1.2-3 – simplified representative mapping
            let additional = additional_pos.min(3);
            match (n_symbols, additional) {
                (2..=7, _) => vec![l0],
                (8..=9, 0) => vec![l0],
                (8..=9, 1) => vec![l0, l0 + 7],
                (10..=11, 0) => vec![l0],
                (10..=11, 1) => vec![l0, l0 + 9],
                (10..=11, 2) => vec![l0, l0 + 6, l0 + 9],
                (12..=14, 0) => vec![l0],
                (12..=14, 1) => vec![l0, l0 + 11],
                (12..=14, 2) => vec![l0, l0 + 7, l0 + 11],
                (12..=14, 3) => vec![l0, l0 + 5, l0 + 8, l0 + 11],
                _ => vec![l0],
            }
        }
        DmrsLength::DoubleSymbol => {
            // Table 7.4.1.1.2-4 – simplified representative mapping
            let additional = additional_pos.min(1);
            match (n_symbols, additional) {
                (4..=9, _) => vec![l0, l0 + 1],
                (10..=12, 0) => vec![l0, l0 + 1],
                (10..=12, 1) => vec![l0, l0 + 1, l0 + 8, l0 + 9],
                (13..=14, 0) => vec![l0, l0 + 1],
                (13..=14, 1) => vec![l0, l0 + 1, l0 + 11, l0 + 12],
                _ => vec![l0, l0 + 1],
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Low-PAPR base sequences (transform-precoded PUSCH DMRS)
// ---------------------------------------------------------------------------

/// Low-PAPR base sequence table excerpt (TS 38.211 Table 5.2.2.1.2-1).
///
/// These are the φ(n) values for sequences of length Msc = 12 (one PRB),
/// group u = 0, base sequence v = 0.
///
/// r^(u,v)(n) = exp(j * φ(n) * π / 4)
const LOW_PAPR_PHI_U0_MZC12: [i8; 12] = [-1, 1, 3, -3, 3, 3, 1, 1, 3, 1, -3, 3];

/// Low-PAPR base sequence group table (u) for Zadoff-Chu style sequences.
///
/// For Msc >= 36 the sequence is derived from Zadoff-Chu roots.
/// For Msc = 12 (one PRB), the Phi table from TS 38.211 Section 5.2.2 is used.
///
/// This function returns the length-12 sequence for group u, sequence v.
pub fn low_papr_sequence_prb(group_u: u8, seq_v: u8) -> Vec<Complex64> {
    // For group 0, v 0: use the φ table directly
    let phi = match (group_u % 30, seq_v % 2) {
        // Simplified: only group 0 exact table; others approximated via ZC-style
        (0, 0) => LOW_PAPR_PHI_U0_MZC12.as_ref(),
        _ => &LOW_PAPR_PHI_U0_MZC12,
    };

    use std::f64::consts::PI;
    phi.iter()
        .map(|&p| {
            let angle = p as f64 * PI / 4.0;
            Complex64::new(angle.cos(), angle.sin())
        })
        .collect()
}

/// Zadoff-Chu root sequence for PUSCH DMRS with transform precoding,
/// when Msc >= 36 per TS 38.211 Section 5.2.2.1.
///
/// r_u_v(n) = x_q(n mod N_ZC) for n = 0..Msc-1
/// x_q(m) = exp(-j * π * q * m * (m+1) / N_ZC)
///
/// where N_ZC is the largest prime <= Msc.
pub fn zadoff_chu_sequence(root_q: u32, length_mzc: usize) -> Vec<Complex64> {
    use std::f64::consts::PI;
    let q = root_q as f64;
    let nzc = length_mzc as f64;
    (0..length_mzc)
        .map(|m| {
            let m = m as f64;
            let angle = -PI * q * m * (m + 1.0) / nzc;
            Complex64::new(angle.cos(), angle.sin())
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// Result of channel estimation at one DMRS resource element.
#[derive(Debug, Clone, Copy)]
pub struct ChEstSample {
    /// Subcarrier index within the allocated bandwidth.
    pub subcarrier: usize,
    /// Estimated complex channel coefficient H = Y / R.
    pub h: Complex64,
}

/// 5G NR DMRS Processor.
///
/// Generates DMRS reference sequences, maps them onto resource elements, and
/// performs Least-Squares channel estimation with linear interpolation.
#[derive(Debug, Clone)]
pub struct NrDmrsProcessor {
    config: DmrsConfig,
}

impl NrDmrsProcessor {
    /// Create a new processor from a `DmrsConfig`.
    pub fn new(config: DmrsConfig) -> Self {
        Self { config }
    }

    /// Access the stored configuration.
    pub fn config(&self) -> &DmrsConfig {
        &self.config
    }

    // -----------------------------------------------------------------------
    // c_init selection
    // -----------------------------------------------------------------------

    /// Compute `c_init` for the given slot and symbol.
    pub fn c_init(&self, slot: u32, symbol: u32) -> u32 {
        match &self.config.channel {
            DmrsChannel::Pdsch => {
                pdsch_dmrs_c_init(slot, symbol, self.config.n_id, self.config.n_scid)
            }
            DmrsChannel::Pusch => {
                pusch_dmrs_c_init(slot, symbol, self.config.n_id, self.config.n_scid)
            }
            DmrsChannel::Pdcch => pdcch_dmrs_c_init(slot, symbol, self.config.n_id),
            DmrsChannel::Pucch(fmt) => match fmt {
                PucchFormat::Format2 => {
                    pucch_f2_dmrs_c_init(slot, symbol, self.config.n_id)
                }
                // Formats 1, 3, 4 use cyclic-shift based OCC – simplified c_init
                _ => pdsch_dmrs_c_init(slot, symbol, self.config.n_id, 0),
            },
        }
    }

    // -----------------------------------------------------------------------
    // Sequence generation
    // -----------------------------------------------------------------------

    /// Generate the DMRS base sequence `r(m)` for the given slot/symbol.
    ///
    /// Returns one complex sample per DMRS subcarrier across all allocated PRBs.
    /// The number of samples depends on the DMRS type and number of PRBs.
    pub fn generate_sequence(&self, slot: u32, symbol: u32) -> Vec<Complex64> {
        if self.config.transform_precoding {
            return self.generate_low_papr_sequence();
        }

        let c_init = self.c_init(slot, symbol);
        let n_dmrs_re = self.num_dmrs_re_per_prb() as usize * self.config.n_prb as usize;
        let bits = gold_sequence(c_init, 2 * n_dmrs_re);
        let mut seq = gold_to_dmrs(&bits);

        // Apply power boosting
        if self.config.power_boost_db != 0.0 {
            let gain = 10.0_f64.powf(self.config.power_boost_db / 20.0);
            for s in seq.iter_mut() {
                *s = *s * gain;
            }
        }

        // Apply OCC weighting for CDM groups
        seq = self.apply_occ(seq, symbol);
        seq
    }

    /// Number of DMRS resource elements per PRB.
    pub fn num_dmrs_re_per_prb(&self) -> u32 {
        match (&self.config.channel, self.config.dmrs_type) {
            (DmrsChannel::Pdcch, _) => 3, // every 4th subcarrier → 3 per 12 RE REG
            (_, DmrsType::Type1) => 6,    // comb-2: 6 per PRB per CDM group
            (_, DmrsType::Type2) => 4,    // comb-3: 4 per PRB per CDM group
        }
    }

    /// Apply Orthogonal Cover Code to the sequence.
    ///
    /// The OCC spreads one DMRS port across two (or four) consecutive symbols,
    /// allowing multiple ports to share the same subcarrier resources.
    fn apply_occ(&self, seq: Vec<Complex64>, symbol: u32) -> Vec<Complex64> {
        match self.config.length {
            DmrsLength::SingleSymbol => {
                // OCC length 2 over time (two consecutive DMRS symbols treated together)
                // For single-symbol DMRS, the OCC is applied in frequency
                let occ_w = OCC_FREQ_LENGTH2[self.config.occ_index.min(1) as usize];
                seq.iter()
                    .enumerate()
                    .map(|(i, &s)| {
                        let w = occ_w[i % 2];
                        s * w
                    })
                    .collect()
            }
            DmrsLength::DoubleSymbol => {
                // OCC length 4 across two consecutive DMRS symbols
                // symbol_offset: 0 for first DMRS symbol, 1 for second
                let sym_off = (symbol % 2) as usize;
                let occ_idx = self.config.occ_index.min(3) as usize;
                let w = OCC_LENGTH4[occ_idx];
                seq.iter()
                    .enumerate()
                    .map(|(i, &s)| {
                        let w_val = w[(sym_off * 2 + i % 2) % 4];
                        s * w_val
                    })
                    .collect()
            }
        }
    }

    // -----------------------------------------------------------------------
    // Low-PAPR sequence for transform-precoded PUSCH
    // -----------------------------------------------------------------------

    /// Generate low-PAPR sequence for transform-precoded PUSCH DMRS.
    ///
    /// For Msc = 12 (one PRB) the φ-table is used; for larger allocations
    /// a Zadoff-Chu root sequence is used.
    fn generate_low_papr_sequence(&self) -> Vec<Complex64> {
        let msc = 12 * self.config.n_prb as usize;
        if msc == 12 {
            low_papr_sequence_prb(0, 0)
        } else {
            // Select ZC root: q = floor(N_ZC/31) * (u+1) simplified
            let nzc = largest_prime_le(msc);
            let root_q = (nzc as u32 / 31 + 1).max(1);
            zadoff_chu_sequence(root_q, msc)
        }
    }

    // -----------------------------------------------------------------------
    // DMRS positions
    // -----------------------------------------------------------------------

    /// Return OFDM symbol indices that carry DMRS within the slot.
    ///
    /// For PDSCH/PUSCH uses `dmrs_symbol_positions`; for PDCCH returns symbol 0.
    pub fn get_dmrs_positions(&self) -> Vec<u32> {
        match self.config.channel {
            DmrsChannel::Pdcch => vec![0],
            DmrsChannel::Pucch(PucchFormat::Format1) => {
                // PUCCH Format 1: DMRS on alternating symbols (0, 2, 4, …)
                (0..14u32).step_by(2).collect()
            }
            DmrsChannel::Pucch(PucchFormat::Format2) => vec![0, 1],
            DmrsChannel::Pucch(PucchFormat::Format3) => {
                // PUCCH Format 3: DMRS on symbol 1 and additional if > 9 symbols
                vec![0, 1, 8, 9]
            }
            DmrsChannel::Pucch(PucchFormat::Format4) => vec![0, 1],
            _ => {
                // PDSCH / PUSCH
                dmrs_symbol_positions(
                    self.config.length,
                    self.config.additional_pos,
                    14, // full slot
                    2,  // l0 at symbol 2 (typical for PDSCH)
                )
                .iter()
                .map(|&x| x as u32)
                .collect()
            }
        }
    }

    /// Return (PRB, subcarrier_within_PRB) pairs of all DMRS resource elements.
    pub fn get_dmrs_resource_elements(&self) -> Vec<(u32, u32)> {
        let offsets: Vec<u32> = match (&self.config.channel, self.config.dmrs_type) {
            (DmrsChannel::Pdcch, _) => pdcch_subcarrier_offsets(),
            (_, DmrsType::Type1) => type1_subcarrier_offsets(self.config.cdm_group),
            (_, DmrsType::Type2) => type2_subcarrier_offsets(self.config.cdm_group),
        };
        let mut res = Vec::new();
        for prb in 0..self.config.n_prb as u32 {
            for &k in &offsets {
                res.push((prb, k));
            }
        }
        res
    }

    /// Absolute subcarrier indices (0-based across the total bandwidth) for DMRS.
    ///
    /// Assumes that PRB 0 starts at subcarrier 0 (relative to the allocation).
    pub fn get_absolute_subcarriers(&self) -> Vec<u32> {
        self.get_dmrs_resource_elements()
            .iter()
            .map(|(prb, k)| prb * 12 + k)
            .collect()
    }

    // -----------------------------------------------------------------------
    // Channel estimation
    // -----------------------------------------------------------------------

    /// Least-Squares channel estimation at DMRS positions.
    ///
    /// H_LS(k) = Y(k) / R(k)
    ///
    /// where Y is the received signal and R is the known DMRS reference.
    /// Returns one `ChEstSample` per DMRS RE with the estimated channel.
    pub fn estimate_channel_ls(
        &self,
        rx_dmrs: &[Complex64],
        tx_dmrs: &[Complex64],
    ) -> Vec<ChEstSample> {
        let n = rx_dmrs.len().min(tx_dmrs.len());
        let subcarriers = self.get_absolute_subcarriers();
        (0..n)
            .map(|i| {
                let h = if tx_dmrs[i].norm_sqr() > 1e-12 {
                    rx_dmrs[i] / tx_dmrs[i]
                } else {
                    Complex64::new(0.0, 0.0)
                };
                ChEstSample {
                    subcarrier: *subcarriers.get(i).unwrap_or(&(i as u32)) as usize,
                    h,
                }
            })
            .collect()
    }

    /// Linear interpolation of channel estimates to all data subcarriers.
    ///
    /// Given LS estimates at `dmrs_samples` (sorted by subcarrier index),
    /// returns channel coefficients at every subcarrier in `0..total_subcarriers`.
    pub fn interpolate_channel(
        &self,
        dmrs_samples: &[ChEstSample],
        total_subcarriers: usize,
    ) -> Vec<Complex64> {
        if dmrs_samples.is_empty() {
            return vec![Complex64::new(1.0, 0.0); total_subcarriers];
        }

        let mut result = vec![Complex64::new(0.0, 0.0); total_subcarriers];

        // Extrapolate for subcarriers below first DMRS position
        let first = &dmrs_samples[0];
        for k in 0..first.subcarrier.min(total_subcarriers) {
            result[k] = first.h;
        }

        // Interpolate between adjacent DMRS positions
        for w in dmrs_samples.windows(2) {
            let l = &w[0];
            let r = &w[1];
            let span = (r.subcarrier - l.subcarrier) as f64;
            if span <= 0.0 {
                continue;
            }
            let k_start = l.subcarrier.min(total_subcarriers);
            let k_end = r.subcarrier.min(total_subcarriers);
            for k in k_start..k_end {
                let alpha = (k - l.subcarrier) as f64 / span;
                let re = l.h.re + alpha * (r.h.re - l.h.re);
                let im = l.h.im + alpha * (r.h.im - l.h.im);
                result[k] = Complex64::new(re, im);
            }
        }

        // Extrapolate beyond the last DMRS position
        let last = dmrs_samples.last().unwrap();
        for k in last.subcarrier.min(total_subcarriers)..total_subcarriers {
            result[k] = last.h;
        }

        result
    }

    /// Full pipeline: generate DMRS, apply to received signal, return channel estimates.
    ///
    /// * `slot` – slot number in the frame
    /// * `symbol` – OFDM symbol index within the slot
    /// * `rx_signal` – received complex samples at DMRS subcarrier positions
    ///
    /// Returns LS channel estimates.
    pub fn process_dmrs_symbol(
        &self,
        slot: u32,
        symbol: u32,
        rx_signal: &[Complex64],
    ) -> Vec<ChEstSample> {
        let tx_dmrs = self.generate_sequence(slot, symbol);
        self.estimate_channel_ls(rx_signal, &tx_dmrs)
    }
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

/// Find the largest prime <= n (for Zadoff-Chu length selection).
fn largest_prime_le(n: usize) -> usize {
    if n < 2 {
        return 2;
    }
    let mut p = n;
    while p >= 2 {
        if is_prime(p) {
            return p;
        }
        p -= 1;
    }
    2
}

fn is_prime(n: usize) -> bool {
    if n < 2 {
        return false;
    }
    if n == 2 {
        return true;
    }
    if n % 2 == 0 {
        return false;
    }
    let mut i = 3;
    while i * i <= n {
        if n % i == 0 {
            return false;
        }
        i += 2;
    }
    true
}

/// Normalise a complex vector to unit average power.
pub fn normalise_sequence(seq: &mut Vec<Complex64>) {
    if seq.is_empty() {
        return;
    }
    let power: f64 = seq.iter().map(|s| s.norm_sqr()).sum::<f64>() / seq.len() as f64;
    if power > 1e-12 {
        let gain = 1.0 / power.sqrt();
        for s in seq.iter_mut() {
            *s = *s * gain;
        }
    }
}

/// Compute cross-correlation between two equal-length sequences.
pub fn cross_correlation(a: &[Complex64], b: &[Complex64]) -> Complex64 {
    let n = a.len().min(b.len());
    let mut acc = Complex64::new(0.0, 0.0);
    for i in 0..n {
        acc = acc + a[i] * b[i].conj();
    }
    acc / n as f64
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Complex64 arithmetic -----------------------------------------------

    #[test]
    fn test_complex64_add() {
        let a = Complex64::new(1.0, 2.0);
        let b = Complex64::new(3.0, 4.0);
        let c = a + b;
        assert!((c.re - 4.0).abs() < 1e-12);
        assert!((c.im - 6.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex64_mul() {
        let a = Complex64::new(1.0, 2.0);
        let b = Complex64::new(3.0, 4.0);
        let c = a * b; // (1+2j)(3+4j) = 3+4j+6j-8 = -5+10j
        assert!((c.re - (-5.0)).abs() < 1e-12);
        assert!((c.im - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex64_div() {
        let a = Complex64::new(1.0, 0.0);
        let b = Complex64::new(2.0, 0.0);
        let c = a / b;
        assert!((c.re - 0.5).abs() < 1e-12);
        assert!(c.im.abs() < 1e-12);
    }

    #[test]
    fn test_complex64_conj() {
        let a = Complex64::new(3.0, -4.0);
        let b = a.conj();
        assert!((b.re - 3.0).abs() < 1e-12);
        assert!((b.im - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex64_norm() {
        let a = Complex64::new(3.0, 4.0);
        assert!((a.norm() - 5.0).abs() < 1e-10);
    }

    // --- Gold sequence -------------------------------------------------------

    #[test]
    fn test_gold_sequence_length() {
        let bits = gold_sequence(0, 100);
        assert_eq!(bits.len(), 100);
    }

    #[test]
    fn test_gold_sequence_binary() {
        let bits = gold_sequence(42, 200);
        for &b in &bits {
            assert!(b == 0 || b == 1, "Bit must be 0 or 1, got {}", b);
        }
    }

    #[test]
    fn test_gold_sequence_deterministic() {
        let a = gold_sequence(12345, 50);
        let b = gold_sequence(12345, 50);
        assert_eq!(a, b);
    }

    #[test]
    fn test_gold_sequence_different_c_init() {
        let a = gold_sequence(0, 64);
        let b = gold_sequence(1, 64);
        // Different c_init should produce different sequences
        assert_ne!(a, b);
    }

    #[test]
    fn test_gold_sequence_incremental() {
        // Two separate generators with the same c_init must agree
        let full = gold_sequence(777, 32);
        let mut gen = GoldSequence::new(777);
        let part1 = gen.generate(16);
        let part2 = gen.generate(16);
        let combined: Vec<u8> = part1.into_iter().chain(part2).collect();
        assert_eq!(full, combined);
    }

    #[test]
    fn test_gold_sequence_balance() {
        // A long sequence should be roughly 50% ones
        let bits = gold_sequence(0xDEAD_BEEF, 2000);
        let ones: usize = bits.iter().filter(|&&b| b == 1).count();
        let ratio = ones as f64 / 2000.0;
        assert!((ratio - 0.5).abs() < 0.05, "Balance ratio={:.3}", ratio);
    }

    // --- Gold-to-DMRS conversion --------------------------------------------

    #[test]
    fn test_gold_to_dmrs_length() {
        let bits = gold_sequence(0, 24);
        let dmrs = gold_to_dmrs(&bits);
        assert_eq!(dmrs.len(), 12);
    }

    #[test]
    fn test_gold_to_dmrs_unit_power() {
        // Each symbol should have |r(n)|^2 = 1 (since 1/sqrt(2)^2 + 1/sqrt(2)^2 = 1)
        let bits = gold_sequence(0, 200);
        let dmrs = gold_to_dmrs(&bits);
        for s in &dmrs {
            let p = s.norm_sqr();
            assert!((p - 1.0).abs() < 1e-10, "Power={:.6}", p);
        }
    }

    #[test]
    fn test_gold_to_dmrs_values() {
        // bits [0, 0] → re=(1-0)/√2 = 1/√2, im=(1-0)/√2 = 1/√2
        let bits = vec![0u8, 0, 1, 1, 0, 1, 1, 0];
        let dmrs = gold_to_dmrs(&bits);
        let inv_sqrt2 = 1.0_f64 / 2.0_f64.sqrt();
        assert!((dmrs[0].re - inv_sqrt2).abs() < 1e-12);
        assert!((dmrs[0].im - inv_sqrt2).abs() < 1e-12);
        assert!((dmrs[1].re - (-inv_sqrt2)).abs() < 1e-12);
        assert!((dmrs[1].im - (-inv_sqrt2)).abs() < 1e-12);
    }

    // --- c_init formulas ----------------------------------------------------

    #[test]
    fn test_pdsch_c_init_zero() {
        // slot=0, symbol=0, n_id=0, n_scid=0
        let ci = pdsch_dmrs_c_init(0, 0, 0, 0);
        // (2^17 * 1 * 1 + 0 + 0) = 131072
        assert_eq!(ci, 131072);
    }

    #[test]
    fn test_pdsch_c_init_slot1() {
        let ci = pdsch_dmrs_c_init(1, 0, 1, 0);
        let a = 14u64 + 1;
        let b = 3u64;
        let expected = ((1u64 << 17) * a * b + 2 + 0) & 0x7FFF_FFFF;
        assert_eq!(ci as u64, expected);
    }

    #[test]
    fn test_pusch_c_init_equals_pdsch() {
        // PUSCH c_init formula is the same as PDSCH
        let a = pusch_dmrs_c_init(3, 7, 500, 1);
        let b = pdsch_dmrs_c_init(3, 7, 500, 1);
        assert_eq!(a, b);
    }

    #[test]
    fn test_pdcch_c_init() {
        // slot=0, symbol=0, n_id=42
        let ci = pdcch_dmrs_c_init(0, 0, 42);
        // (2^17 * 1 * 85 + 42) & mask
        let expected = ((1u64 << 17) * 1 * 85 + 42) & 0x7FFF_FFFF;
        assert_eq!(ci as u64, expected);
    }

    #[test]
    fn test_pucch_f2_c_init() {
        let ci = pucch_f2_dmrs_c_init(0, 0, 10);
        // (2^17 * 1 * 21 + 20) & mask
        let expected = ((1u64 << 17) * 1 * 21 + 20) & 0x7FFF_FFFF;
        assert_eq!(ci as u64, expected);
    }

    // --- Subcarrier offsets --------------------------------------------------

    #[test]
    fn test_type1_cdm0_offsets() {
        let offsets = type1_subcarrier_offsets(0);
        assert_eq!(offsets, vec![0, 2, 4, 6, 8, 10]);
    }

    #[test]
    fn test_type1_cdm1_offsets() {
        let offsets = type1_subcarrier_offsets(1);
        assert_eq!(offsets, vec![1, 3, 5, 7, 9, 11]);
    }

    #[test]
    fn test_type2_cdm0_offsets() {
        let offsets = type2_subcarrier_offsets(0);
        assert_eq!(offsets, vec![0, 1, 6, 7]);
    }

    #[test]
    fn test_type2_cdm1_offsets() {
        let offsets = type2_subcarrier_offsets(1);
        assert_eq!(offsets, vec![2, 3, 8, 9]);
    }

    #[test]
    fn test_type2_cdm2_offsets() {
        let offsets = type2_subcarrier_offsets(2);
        assert_eq!(offsets, vec![4, 5, 10, 11]);
    }

    #[test]
    fn test_pdcch_offsets() {
        let offsets = pdcch_subcarrier_offsets();
        assert_eq!(offsets, vec![1, 5, 9]);
    }

    // --- DMRS symbol positions ----------------------------------------------

    #[test]
    fn test_single_symbol_pos0_short() {
        let pos = dmrs_symbol_positions(DmrsLength::SingleSymbol, 0, 6, 2);
        assert_eq!(pos, vec![2]);
    }

    #[test]
    fn test_single_symbol_pos1_medium() {
        let pos = dmrs_symbol_positions(DmrsLength::SingleSymbol, 1, 9, 2);
        assert_eq!(pos, vec![2, 9]);
    }

    #[test]
    fn test_single_symbol_pos2_long() {
        let pos = dmrs_symbol_positions(DmrsLength::SingleSymbol, 2, 12, 2);
        assert_eq!(pos, vec![2, 9, 13]);
    }

    #[test]
    fn test_single_symbol_pos3_full_slot() {
        let pos = dmrs_symbol_positions(DmrsLength::SingleSymbol, 3, 14, 2);
        assert_eq!(pos, vec![2, 7, 10, 13]);
    }

    #[test]
    fn test_double_symbol_pos0() {
        let pos = dmrs_symbol_positions(DmrsLength::DoubleSymbol, 0, 14, 2);
        assert_eq!(pos, vec![2, 3]);
    }

    #[test]
    fn test_double_symbol_pos1_long() {
        let pos = dmrs_symbol_positions(DmrsLength::DoubleSymbol, 1, 14, 2);
        assert_eq!(pos, vec![2, 3, 13, 14]);
    }

    #[test]
    fn test_double_symbol_pos0_medium() {
        let pos = dmrs_symbol_positions(DmrsLength::DoubleSymbol, 0, 10, 2);
        assert_eq!(pos, vec![2, 3]);
    }

    // --- Processor: sequence generation -------------------------------------

    #[test]
    fn test_processor_generate_sequence_type1() {
        let cfg = DmrsConfig::pdsch_type1(1001, 25);
        let proc = NrDmrsProcessor::new(cfg);
        let seq = proc.generate_sequence(0, 2);
        // 25 PRBs * 6 RE/PRB = 150 DMRS symbols
        assert_eq!(seq.len(), 150);
    }

    #[test]
    fn test_processor_generate_sequence_type2() {
        let cfg = DmrsConfig::pdsch_type2(0, 10, 0);
        let proc = NrDmrsProcessor::new(cfg);
        let seq = proc.generate_sequence(0, 2);
        // 10 PRBs * 4 RE/PRB = 40 DMRS symbols
        assert_eq!(seq.len(), 40);
    }

    #[test]
    fn test_processor_sequence_unit_power_type1() {
        let cfg = DmrsConfig::pdsch_type1(42, 5);
        let proc = NrDmrsProcessor::new(cfg);
        let seq = proc.generate_sequence(1, 4);
        for s in &seq {
            let p = s.norm_sqr();
            // After OCC (±1 coefficients applied to unit-power symbols the power remains 1)
            assert!((p - 1.0).abs() < 1e-8, "power={:.8}", p);
        }
    }

    #[test]
    fn test_processor_sequence_different_slots() {
        let cfg = DmrsConfig::pdsch_type1(0, 5);
        let proc = NrDmrsProcessor::new(cfg);
        let s0 = proc.generate_sequence(0, 2);
        let s1 = proc.generate_sequence(1, 2);
        // Different slots → different c_init → different sequences
        assert_ne!(s0, s1);
    }

    #[test]
    fn test_processor_power_boost() {
        let mut cfg = DmrsConfig::pdsch_type1(0, 1);
        cfg.power_boost_db = 6.0; // ~2x linear
        let proc = NrDmrsProcessor::new(cfg);
        let seq = proc.generate_sequence(0, 2);
        for s in &seq {
            let p = s.norm_sqr();
            // 6 dB boost → power ≈ 4x
            assert!((p - 4.0).abs() < 0.1, "power={:.4}", p);
        }
    }

    // --- Processor: num_dmrs_re_per_prb -------------------------------------

    #[test]
    fn test_num_dmrs_re_type1() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        assert_eq!(proc.num_dmrs_re_per_prb(), 6);
    }

    #[test]
    fn test_num_dmrs_re_type2() {
        let cfg = DmrsConfig::pdsch_type2(0, 1, 0);
        let proc = NrDmrsProcessor::new(cfg);
        assert_eq!(proc.num_dmrs_re_per_prb(), 4);
    }

    #[test]
    fn test_num_dmrs_re_pdcch() {
        let cfg = DmrsConfig::pdcch(42, 1);
        let proc = NrDmrsProcessor::new(cfg);
        assert_eq!(proc.num_dmrs_re_per_prb(), 3);
    }

    // --- Processor: positions & resource elements ---------------------------

    #[test]
    fn test_get_dmrs_positions_pdsch() {
        let cfg = DmrsConfig::pdsch_type1(0, 10);
        let proc = NrDmrsProcessor::new(cfg);
        let pos = proc.get_dmrs_positions();
        assert!(!pos.is_empty());
        assert!(pos.contains(&2)); // l0 = 2
    }

    #[test]
    fn test_get_dmrs_positions_pdcch() {
        let cfg = DmrsConfig::pdcch(0, 4);
        let proc = NrDmrsProcessor::new(cfg);
        let pos = proc.get_dmrs_positions();
        assert_eq!(pos, vec![0]);
    }

    #[test]
    fn test_get_dmrs_resource_elements_type1() {
        let cfg = DmrsConfig::pdsch_type1(0, 2);
        let proc = NrDmrsProcessor::new(cfg);
        let re = proc.get_dmrs_resource_elements();
        // 2 PRBs * 6 RE = 12 entries
        assert_eq!(re.len(), 12);
        // First PRB, CDM group 0: offsets 0,2,4,6,8,10
        assert!(re.contains(&(0, 0)));
        assert!(re.contains(&(0, 2)));
        assert!(re.contains(&(1, 0)));
    }

    #[test]
    fn test_get_absolute_subcarriers_type1_cdm0() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        let sc = proc.get_absolute_subcarriers();
        assert_eq!(sc, vec![0, 2, 4, 6, 8, 10]);
    }

    #[test]
    fn test_get_absolute_subcarriers_type2_cdm0() {
        let mut cfg = DmrsConfig::pdsch_type2(0, 1, 0);
        cfg.dmrs_type = DmrsType::Type2;
        let proc = NrDmrsProcessor::new(cfg);
        let sc = proc.get_absolute_subcarriers();
        assert_eq!(sc, vec![0, 1, 6, 7]);
    }

    // --- Channel estimation -------------------------------------------------

    #[test]
    fn test_ls_estimation_identity_channel() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        let tx = proc.generate_sequence(0, 2);
        // Identity channel: rx = tx
        let estimates = proc.estimate_channel_ls(&tx, &tx);
        for est in &estimates {
            // H_LS = rx/tx = 1+0j
            assert!((est.h.re - 1.0).abs() < 1e-8, "H.re={:.8}", est.h.re);
            assert!(est.h.im.abs() < 1e-8, "H.im={:.8}", est.h.im);
        }
    }

    #[test]
    fn test_ls_estimation_scaled_channel() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        let tx = proc.generate_sequence(0, 2);
        // Channel gain = 0.5
        let rx: Vec<Complex64> = tx.iter().map(|&s| s * 0.5).collect();
        let estimates = proc.estimate_channel_ls(&rx, &tx);
        for est in &estimates {
            assert!((est.h.re - 0.5).abs() < 1e-8, "H.re={:.8}", est.h.re);
            assert!(est.h.im.abs() < 1e-8, "H.im={:.8}", est.h.im);
        }
    }

    #[test]
    fn test_ls_estimation_phase_shift() {
        use std::f64::consts::PI;
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        let tx = proc.generate_sequence(0, 2);
        // Channel: pure phase rotation of π/4
        let phase = PI / 4.0;
        let h_true = Complex64::new(phase.cos(), phase.sin());
        let rx: Vec<Complex64> = tx.iter().map(|&s| s * h_true).collect();
        let estimates = proc.estimate_channel_ls(&rx, &tx);
        for est in &estimates {
            assert!((est.h.re - h_true.re).abs() < 1e-8);
            assert!((est.h.im - h_true.im).abs() < 1e-8);
        }
    }

    // --- Interpolation ------------------------------------------------------

    #[test]
    fn test_interpolation_flat_channel() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        // Estimate H=1 at subcarriers 0, 2, 4, 6, 8, 10
        let samples: Vec<ChEstSample> = (0..6)
            .map(|i| ChEstSample {
                subcarrier: i * 2,
                h: Complex64::new(1.0, 0.0),
            })
            .collect();
        let h_all = proc.interpolate_channel(&samples, 12);
        for h in &h_all {
            assert!((h.re - 1.0).abs() < 1e-6, "h.re={:.6}", h.re);
        }
    }

    #[test]
    fn test_interpolation_linear_channel() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        // Two DMRS points: H=0 at k=0, H=1 at k=11
        let samples = vec![
            ChEstSample { subcarrier: 0, h: Complex64::new(0.0, 0.0) },
            ChEstSample { subcarrier: 11, h: Complex64::new(1.0, 0.0) },
        ];
        let h_all = proc.interpolate_channel(&samples, 12);
        // At k=0: H≈0, at k=11: H≈1
        assert!(h_all[0].re.abs() < 1e-6);
        assert!((h_all[11].re - 1.0).abs() < 1e-6);
        // k=5 should be ≈ 5/11
        let expected = 5.0 / 11.0;
        assert!((h_all[5].re - expected).abs() < 1e-6);
    }

    #[test]
    fn test_interpolation_empty_samples() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg);
        let h_all = proc.interpolate_channel(&[], 12);
        // Should return default H=1 everywhere
        assert_eq!(h_all.len(), 12);
        for h in &h_all {
            assert!((h.re - 1.0).abs() < 1e-10);
        }
    }

    // --- Full pipeline -------------------------------------------------------

    #[test]
    fn test_process_dmrs_symbol() {
        let cfg = DmrsConfig::pdsch_type1(0, 1);
        let proc = NrDmrsProcessor::new(cfg.clone());
        let tx = proc.generate_sequence(0, 2);
        // Perfect channel
        let estimates = proc.process_dmrs_symbol(0, 2, &tx);
        assert_eq!(estimates.len(), 6); // 1 PRB * 6 DMRS RE
        for est in &estimates {
            assert!((est.h.re - 1.0).abs() < 1e-8);
        }
    }

    // --- Transform precoding (low-PAPR) -------------------------------------

    #[test]
    fn test_low_papr_sequence_length() {
        let seq = low_papr_sequence_prb(0, 0);
        assert_eq!(seq.len(), 12);
    }

    #[test]
    fn test_low_papr_sequence_unit_power() {
        let seq = low_papr_sequence_prb(0, 0);
        for s in &seq {
            assert!((s.norm_sqr() - 1.0).abs() < 1e-10, "power={:.10}", s.norm_sqr());
        }
    }

    #[test]
    fn test_pusch_transform_precoding_sequence() {
        let mut cfg = DmrsConfig::pusch(500, 1, true);
        cfg.transform_precoding = true;
        let proc = NrDmrsProcessor::new(cfg);
        let seq = proc.generate_sequence(0, 2);
        assert_eq!(seq.len(), 12);
    }

    #[test]
    fn test_zadoff_chu_length() {
        let seq = zadoff_chu_sequence(1, 36);
        assert_eq!(seq.len(), 36);
    }

    #[test]
    fn test_zadoff_chu_unit_magnitude() {
        let seq = zadoff_chu_sequence(7, 25);
        for s in &seq {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    // --- CDM and OCC --------------------------------------------------------

    #[test]
    fn test_occ_length2_orthogonal() {
        // OCC[0] · OCC[1] = +1*+1 + +1*−1 = 0
        let dot: f64 = OCC_LENGTH2[0]
            .iter()
            .zip(OCC_LENGTH2[1].iter())
            .map(|(a, b)| a * b)
            .sum();
        assert!(dot.abs() < 1e-12);
    }

    #[test]
    fn test_occ_length4_orthogonal() {
        // All pairs in OCC_LENGTH4 should be orthogonal
        for i in 0..4 {
            for j in (i + 1)..4 {
                let dot: f64 = OCC_LENGTH4[i]
                    .iter()
                    .zip(OCC_LENGTH4[j].iter())
                    .map(|(a, b)| a * b)
                    .sum();
                assert!(dot.abs() < 1e-12, "OCC[{}]·OCC[{}]={}", i, j, dot);
            }
        }
    }

    #[test]
    fn test_different_cdm_groups_orthogonal_type2() {
        // Subcarrier sets for CDM groups 0 and 1 should not overlap
        let g0 = type2_subcarrier_offsets(0);
        let g1 = type2_subcarrier_offsets(1);
        for &k in &g0 {
            assert!(!g1.contains(&k), "CDM group 0 and 1 share subcarrier {}", k);
        }
    }

    #[test]
    fn test_type1_cdm_groups_cover_all_subcarriers() {
        let g0 = type1_subcarrier_offsets(0);
        let g1 = type1_subcarrier_offsets(1);
        let mut all: Vec<u32> = g0.into_iter().chain(g1).collect();
        all.sort_unstable();
        assert_eq!(all, vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
    }

    // --- Utility ------------------------------------------------------------

    #[test]
    fn test_normalise_sequence() {
        let mut seq = vec![
            Complex64::new(2.0, 0.0),
            Complex64::new(0.0, 2.0),
        ];
        normalise_sequence(&mut seq);
        for s in &seq {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_normalise_empty() {
        let mut seq: Vec<Complex64> = Vec::new();
        normalise_sequence(&mut seq); // must not panic
    }

    #[test]
    fn test_cross_correlation_identical() {
        let seq = gold_to_dmrs(&gold_sequence(0, 24));
        let cc = cross_correlation(&seq, &seq);
        // Should be real-valued and ≈ 1 for unit-power sequences
        assert!((cc.re - 1.0).abs() < 1e-6, "cc.re={:.8}", cc.re);
        assert!(cc.im.abs() < 1e-6);
    }

    #[test]
    fn test_cross_correlation_orthogonal() {
        // Two DMRS sequences from different c_init are low-cross-correlation
        let a = gold_to_dmrs(&gold_sequence(0, 200));
        let b = gold_to_dmrs(&gold_sequence(1, 200));
        let cc = cross_correlation(&a, &b);
        // For Gold sequences the cross-correlation magnitude is bounded
        assert!(cc.norm() < 0.5, "|cc|={:.4}", cc.norm());
    }

    #[test]
    fn test_largest_prime_le() {
        assert_eq!(largest_prime_le(12), 11);
        assert_eq!(largest_prime_le(36), 31);
        assert_eq!(largest_prime_le(37), 37);
        assert_eq!(largest_prime_le(48), 47);
        assert_eq!(largest_prime_le(1), 2);
    }

    #[test]
    fn test_is_prime() {
        assert!(!is_prime(0));
        assert!(!is_prime(1));
        assert!(is_prime(2));
        assert!(is_prime(3));
        assert!(!is_prime(4));
        assert!(is_prime(31));
        assert!(!is_prime(32));
    }

    // --- PUCCH DMRS ---------------------------------------------------------

    #[test]
    fn test_pucch_format1_positions() {
        let cfg = DmrsConfig {
            dmrs_type: DmrsType::Type1,
            channel: DmrsChannel::Pucch(PucchFormat::Format1),
            length: DmrsLength::SingleSymbol,
            additional_pos: 0,
            n_id: 0,
            n_scid: 0,
            n_id_cell: 0,
            n_prb: 1,
            n_layers: 1,
            cdm_group: 0,
            occ_index: 0,
            power_boost_db: 0.0,
            transform_precoding: false,
        };
        let proc = NrDmrsProcessor::new(cfg);
        let pos = proc.get_dmrs_positions();
        // PUCCH Format 1: alternating symbols 0,2,4,6,8,10,12
        assert!(pos.contains(&0));
        assert!(pos.contains(&2));
        assert!(!pos.contains(&1));
    }

    #[test]
    fn test_pucch_format2_positions() {
        let cfg = DmrsConfig {
            dmrs_type: DmrsType::Type1,
            channel: DmrsChannel::Pucch(PucchFormat::Format2),
            length: DmrsLength::SingleSymbol,
            additional_pos: 0,
            n_id: 0,
            n_scid: 0,
            n_id_cell: 0,
            n_prb: 1,
            n_layers: 1,
            cdm_group: 0,
            occ_index: 0,
            power_boost_db: 0.0,
            transform_precoding: false,
        };
        let proc = NrDmrsProcessor::new(cfg);
        let pos = proc.get_dmrs_positions();
        assert_eq!(pos, vec![0, 1]);
    }

    // --- Config convenience constructors ------------------------------------

    #[test]
    fn test_config_pdsch_type1_defaults() {
        let cfg = DmrsConfig::pdsch_type1(1001, 25);
        assert_eq!(cfg.dmrs_type, DmrsType::Type1);
        assert_eq!(cfg.n_prb, 25);
        assert_eq!(cfg.n_id, 1001);
        assert!(!cfg.transform_precoding);
    }

    #[test]
    fn test_config_pdsch_type2_cdm_group() {
        let cfg = DmrsConfig::pdsch_type2(0, 10, 2);
        assert_eq!(cfg.dmrs_type, DmrsType::Type2);
        assert_eq!(cfg.cdm_group, 2);
    }

    #[test]
    fn test_config_pusch_transform_precoding() {
        let cfg = DmrsConfig::pusch(100, 5, true);
        assert!(cfg.transform_precoding);
        assert_eq!(cfg.n_prb, 5);
    }

    #[test]
    fn test_config_pdcch_channel() {
        let cfg = DmrsConfig::pdcch(42, 4);
        assert_eq!(cfg.n_id, 42);
        assert_eq!(cfg.n_prb, 4);
        match cfg.channel {
            DmrsChannel::Pdcch => {}
            _ => panic!("Expected Pdcch"),
        }
    }
}

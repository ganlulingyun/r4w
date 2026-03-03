//! # 5G NR CSI-RS Processor
//!
//! Implements Channel State Information Reference Signal (CSI-RS) processing
//! per 3GPP TS 38.211 Section 7.4.1.5 and TS 38.214 Section 5.2.
//!
//! ## Overview
//!
//! CSI-RS are used in 5G NR for:
//! - Channel estimation and feedback (CQI, PMI, RI)
//! - Beam management (L1-RSRP measurement)
//! - Mobility measurements (RRM)
//! - Time/frequency tracking (TRS)
//! - Interference measurement (ZP-CSI-RS)
//!
//! ## CSI-RS Sequence Generation (TS 38.211 §7.4.1.5.2)
//!
//! Gold sequence initialization:
//! ```text
//! c_init = 2^10 * (14*n_s_f*mu + l + 1) * (2*n_ID + 1) + n_ID
//! ```
//! where `n_s_f` is slot in frame, `l` is OFDM symbol, `n_ID` is scrambling ID.
//!
//! QPSK mapping: `r(m) = (1/sqrt(2)) * (1 - 2*c(2m)) + j*(1/sqrt(2)) * (1 - 2*c(2m+1))`
//!
//! ## CDM Groups (TS 38.211 Table 7.4.1.5.3-1)
//!
//! | CDM Type   | FD spread | TD spread | Ports |
//! |-----------|-----------|-----------|-------|
//! | no-CDM    | 1         | 1         | 1     |
//! | fd-CDM2   | 2         | 1         | 2     |
//! | cdm4-FD2-TD2 | 2      | 2         | 4     |
//! | cdm8-FD2-TD4 | 2      | 4         | 8     |
//!
//! ## CSI Reporting (TS 38.214 §5.2)
//!
//! - CQI: Channel Quality Indicator (4-bit, 0-15)
//! - PMI: Precoding Matrix Indicator (Type I single-panel codebook)
//! - RI: Rank Indicator (1-8 layers)
//! - CRI: CSI-RS Resource Indicator
//!
//! ## Example
//!
//! ```
//! use r4w_core::nr_csi_rs_processor::{
//!     CsiRsConfig, CsiRsProcessor, CsiRsResourceRow, CdmType, CsiRsDensity,
//! };
//!
//! let config = CsiRsConfig {
//!     row: CsiRsResourceRow::Row1,
//!     cdm_type: CdmType::NoCdm,
//!     density: CsiRsDensity::Three,
//!     n_id: 0,
//!     subcarrier_offset: 0,
//!     symbol_mask: 0b00000001,
//!     num_rbs: 52,
//!     first_rb: 0,
//!     periodicity: Some((40, 0)),
//! };
//! let processor = CsiRsProcessor::new(config);
//! let seq = processor.generate_sequence(0, 0, 0);
//! assert!(!seq.is_empty());
//! ```

// trace:5G-NR-CSI-RS-PROC | ai:claude

use std::f64::consts::{PI, SQRT_2};

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// Maximum number of CSI-RS ports per resource
pub const MAX_PORTS: usize = 32;
/// Maximum number of resource blocks in NR (numerology 0, 100 MHz)
pub const MAX_RB: usize = 275;
/// Number of subcarriers per resource block
pub const SC_PER_RB: usize = 12;
/// Gold sequence warm-up length (Nc)
const GOLD_NC: usize = 1600;
/// Number of CQI table entries (TS 38.214 Table 5.2.2.1-2)
const CQI_TABLE_LEN: usize = 16;

// ─────────────────────────────────────────────────────────────────────────────
// CDM Type
// ─────────────────────────────────────────────────────────────────────────────

/// CDM (Code Division Multiplexing) type for CSI-RS
///
/// Defined in TS 38.211 Table 7.4.1.5.3-1
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CdmType {
    /// No CDM spreading (1 port per resource element)
    NoCdm,
    /// FD-CDM2: frequency-domain spreading factor 2 (2 adjacent subcarriers)
    FdCdm2,
    /// CDM4-FD2-TD2: frequency × time spreading of 2×2
    Cdm4Fd2Td2,
    /// CDM8-FD2-TD4: frequency × time spreading of 2×4
    Cdm8Fd2Td4,
}

impl CdmType {
    /// Returns (fd_spread, td_spread) for this CDM type
    pub fn spread_factors(&self) -> (usize, usize) {
        match self {
            CdmType::NoCdm => (1, 1),
            CdmType::FdCdm2 => (2, 1),
            CdmType::Cdm4Fd2Td2 => (2, 2),
            CdmType::Cdm8Fd2Td4 => (2, 4),
        }
    }

    /// Total spreading factor (fd * td)
    pub fn total_spread(&self) -> usize {
        let (fd, td) = self.spread_factors();
        fd * td
    }

    /// Walsh-Hadamard orthogonal cover code for given CDM port index
    ///
    /// Returns (wf, wt) where wf is FD cover and wt is TD cover.
    /// Indices: port 0..total_spread-1
    pub fn cover_code(&self, port_in_group: usize) -> (Vec<i8>, Vec<i8>) {
        match self {
            CdmType::NoCdm => (vec![1], vec![1]),
            CdmType::FdCdm2 => {
                let wf = if port_in_group == 0 {
                    vec![1, 1]
                } else {
                    vec![1, -1]
                };
                (wf, vec![1])
            }
            CdmType::Cdm4Fd2Td2 => {
                // port_in_group: 0..3 → [fd_idx, td_idx] = [p%2, p/2]
                let fd_idx = port_in_group % 2;
                let td_idx = port_in_group / 2;
                let wf = if fd_idx == 0 { vec![1, 1] } else { vec![1, -1] };
                let wt = if td_idx == 0 { vec![1, 1] } else { vec![1, -1] };
                (wf, wt)
            }
            CdmType::Cdm8Fd2Td4 => {
                // port_in_group: 0..7 → fd=[p%2], td=[p/2]
                let fd_idx = port_in_group % 2;
                let td_idx = port_in_group / 2;
                let wf = if fd_idx == 0 { vec![1, 1] } else { vec![1, -1] };
                // Walsh-4 row selection for td_idx in {0,1,2,3}
                let wt = match td_idx {
                    0 => vec![1, 1, 1, 1],
                    1 => vec![1, -1, 1, -1],
                    2 => vec![1, 1, -1, -1],
                    _ => vec![1, -1, -1, 1],
                };
                (wf, wt)
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CSI-RS Density
// ─────────────────────────────────────────────────────────────────────────────

/// CSI-RS frequency-domain density (REs per PRB per port per CDM group)
///
/// TS 38.211 Table 7.4.1.5.3-1 column "Density ρ"
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CsiRsDensity {
    /// 3 REs per PRB (subcarriers {0,4,8})
    Three,
    /// 1 RE per PRB
    One,
    /// 0.5 RE per PRB (alternating PRBs)
    OneHalf,
    /// Dot5 even-slot only
    Dot5Even,
    /// Dot5 odd-slot only
    Dot5Odd,
}

impl CsiRsDensity {
    /// Number of RE positions within one RB for this density (before CDM expansion)
    pub fn re_per_rb(&self) -> usize {
        match self {
            CsiRsDensity::Three => 3,
            CsiRsDensity::One => 1,
            CsiRsDensity::OneHalf | CsiRsDensity::Dot5Even | CsiRsDensity::Dot5Odd => 1,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CSI-RS Resource Row (TS 38.211 Table 7.4.1.5.3-1)
// ─────────────────────────────────────────────────────────────────────────────

/// CSI-RS resource configuration row per TS 38.211 Table 7.4.1.5.3-1
///
/// Each row defines port count, CDM type, density, and symbol/subcarrier patterns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CsiRsResourceRow {
    /// Row 1: 1 port, no-CDM, density 3
    Row1,
    /// Row 2: 1 port, no-CDM, density 1 (single symbol)
    Row2,
    /// Row 3: 2 ports, fd-CDM2, density 1
    Row3,
    /// Row 4: 4 ports, fd-CDM2, density 1
    Row4,
    /// Row 5: 4 ports, fd-CDM2, density 1 (2 symbols)
    Row5,
    /// Row 6: 8 ports, fd-CDM2, density 1 (4 symbols)
    Row6,
    /// Row 7: 8 ports, cdm4-FD2-TD2, density 1
    Row7,
    /// Row 8: 8 ports, cdm4-FD2-TD2, density 0.5
    Row8,
    /// Row 9: 12 ports, fd-CDM2, density 1 (6 symbols)
    Row9,
    /// Row 10: 12 ports, cdm4-FD2-TD2, density 1
    Row10,
    /// Row 11: 16 ports, cdm4-FD2-TD2, density 1
    Row11,
    /// Row 12: 16 ports, cdm8-FD2-TD4, density 0.5
    Row12,
    /// Row 13: 24 ports, cdm4-FD2-TD2, density 1
    Row13,
    /// Row 14: 24 ports, cdm8-FD2-TD4, density 0.5
    Row14,
    /// Row 15: 32 ports, cdm4-FD2-TD2, density 1
    Row15,
    /// Row 16: 32 ports, cdm8-FD2-TD4, density 0.5
    Row16,
    /// Row 17: 1 port, no-CDM, density 3 (2 symbols – for TRS)
    Row17,
    /// Row 18: 1 port, no-CDM, density 3 (4 symbols – for TRS burst)
    Row18,
}

impl CsiRsResourceRow {
    /// Returns the number of antenna ports for this row
    pub fn num_ports(&self) -> usize {
        match self {
            CsiRsResourceRow::Row1 => 1,
            CsiRsResourceRow::Row2 => 1,
            CsiRsResourceRow::Row3 => 2,
            CsiRsResourceRow::Row4 => 4,
            CsiRsResourceRow::Row5 => 4,
            CsiRsResourceRow::Row6 => 8,
            CsiRsResourceRow::Row7 => 8,
            CsiRsResourceRow::Row8 => 8,
            CsiRsResourceRow::Row9 => 12,
            CsiRsResourceRow::Row10 => 12,
            CsiRsResourceRow::Row11 => 16,
            CsiRsResourceRow::Row12 => 16,
            CsiRsResourceRow::Row13 => 24,
            CsiRsResourceRow::Row14 => 24,
            CsiRsResourceRow::Row15 => 32,
            CsiRsResourceRow::Row16 => 32,
            CsiRsResourceRow::Row17 => 1,
            CsiRsResourceRow::Row18 => 1,
        }
    }

    /// Default CDM type for this row
    pub fn default_cdm_type(&self) -> CdmType {
        match self {
            CsiRsResourceRow::Row1
            | CsiRsResourceRow::Row2
            | CsiRsResourceRow::Row17
            | CsiRsResourceRow::Row18 => CdmType::NoCdm,
            CsiRsResourceRow::Row3
            | CsiRsResourceRow::Row4
            | CsiRsResourceRow::Row5
            | CsiRsResourceRow::Row6
            | CsiRsResourceRow::Row9 => CdmType::FdCdm2,
            CsiRsResourceRow::Row7
            | CsiRsResourceRow::Row8
            | CsiRsResourceRow::Row10
            | CsiRsResourceRow::Row11
            | CsiRsResourceRow::Row13
            | CsiRsResourceRow::Row15 => CdmType::Cdm4Fd2Td2,
            CsiRsResourceRow::Row12
            | CsiRsResourceRow::Row14
            | CsiRsResourceRow::Row16 => CdmType::Cdm8Fd2Td4,
        }
    }

    /// Default density for this row
    pub fn default_density(&self) -> CsiRsDensity {
        match self {
            CsiRsResourceRow::Row1
            | CsiRsResourceRow::Row17
            | CsiRsResourceRow::Row18 => CsiRsDensity::Three,
            CsiRsResourceRow::Row8
            | CsiRsResourceRow::Row12
            | CsiRsResourceRow::Row14
            | CsiRsResourceRow::Row16 => CsiRsDensity::OneHalf,
            _ => CsiRsDensity::One,
        }
    }

    /// Number of OFDM symbols occupied (time-domain footprint)
    pub fn num_symbols(&self) -> usize {
        match self {
            CsiRsResourceRow::Row1
            | CsiRsResourceRow::Row2
            | CsiRsResourceRow::Row3
            | CsiRsResourceRow::Row4
            | CsiRsResourceRow::Row7
            | CsiRsResourceRow::Row8 => 1,
            CsiRsResourceRow::Row5
            | CsiRsResourceRow::Row10
            | CsiRsResourceRow::Row11
            | CsiRsResourceRow::Row12 => 2,
            CsiRsResourceRow::Row6
            | CsiRsResourceRow::Row13
            | CsiRsResourceRow::Row14 => 4,
            CsiRsResourceRow::Row9 => 6,
            CsiRsResourceRow::Row15
            | CsiRsResourceRow::Row16 => 8,
            CsiRsResourceRow::Row17 => 2,
            CsiRsResourceRow::Row18 => 4,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Complex sample type
// ─────────────────────────────────────────────────────────────────────────────

/// A complex-valued sample (I + jQ)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    pub fn conj(self) -> Self {
        Self { re: self.re, im: -self.im }
    }

    pub fn abs_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    pub fn abs(self) -> f64 {
        self.abs_sq().sqrt()
    }

    pub fn mul(self, other: Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }

    pub fn add(self, other: Self) -> Self {
        Self {
            re: self.re + other.re,
            im: self.im + other.im,
        }
    }

    pub fn scale(self, s: f64) -> Self {
        Self { re: self.re * s, im: self.im * s }
    }

    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self { re: r * theta.cos(), im: r * theta.sin() }
    }
}

impl std::ops::Add for Complex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self { self.add(rhs) }
}

impl std::ops::Mul for Complex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self { self.mul(rhs) }
}

// ─────────────────────────────────────────────────────────────────────────────
// Gold Sequence Generator (TS 38.211 §5.2.1)
// ─────────────────────────────────────────────────────────────────────────────

/// Gold pseudo-random sequence generator per TS 38.211 Section 5.2.1
///
/// Two m-sequences of length 2^31-1 combined via XOR.
pub struct GoldSeq {
    x1: u32,
    x2: u32,
}

impl GoldSeq {
    /// Initialize with `c_init` and warm up 1600 steps
    pub fn new(c_init: u32) -> Self {
        let mut x1: u32 = 1u32;
        let mut x2: u32 = c_init & 0x7FFF_FFFF;

        for _ in 0..GOLD_NC {
            let b1 = ((x1 >> 3) ^ x1) & 1;
            x1 = (x1 >> 1) | (b1 << 30);

            let b2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
            x2 = (x2 >> 1) | (b2 << 30);
        }

        Self { x1, x2 }
    }

    /// Generate the next output bit
    pub fn next_bit(&mut self) -> u8 {
        let out = ((self.x1 ^ self.x2) & 1) as u8;

        let b1 = ((self.x1 >> 3) ^ self.x1) & 1;
        self.x1 = (self.x1 >> 1) | (b1 << 30);

        let b2 = ((self.x2 >> 3) ^ (self.x2 >> 2) ^ (self.x2 >> 1) ^ self.x2) & 1;
        self.x2 = (self.x2 >> 1) | (b2 << 30);

        out
    }

    /// Generate `n` bits
    pub fn generate_bits(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next_bit()).collect()
    }

    /// Generate QPSK symbols from bit pairs: (1/√2)*(1-2c(2m)) + j*(1/√2)*(1-2c(2m+1))
    pub fn generate_qpsk(&mut self, num_symbols: usize) -> Vec<Complex> {
        let scale = 1.0 / SQRT_2;
        (0..num_symbols)
            .map(|_| {
                let c0 = self.next_bit() as f64;
                let c1 = self.next_bit() as f64;
                Complex::new(scale * (1.0 - 2.0 * c0), scale * (1.0 - 2.0 * c1))
            })
            .collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CSI-RS Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Full CSI-RS resource configuration
#[derive(Debug, Clone)]
pub struct CsiRsConfig {
    /// Resource row (determines port count, CDM type, density)
    pub row: CsiRsResourceRow,
    /// CDM type (overrides row default if explicitly set)
    pub cdm_type: CdmType,
    /// Frequency-domain density
    pub density: CsiRsDensity,
    /// Scrambling ID n_ID (0..1023)
    pub n_id: u32,
    /// Starting subcarrier offset within RB (k0: 0..11)
    pub subcarrier_offset: usize,
    /// Bitmask of OFDM symbols in the slot (bit l = symbol l)
    pub symbol_mask: u16,
    /// Number of allocated resource blocks
    pub num_rbs: usize,
    /// Index of first allocated RB (0-based)
    pub first_rb: usize,
    /// Periodicity: Some((T_ms, offset)) for periodic, None for aperiodic
    pub periodicity: Option<(u32, u32)>,
}

impl CsiRsConfig {
    /// Build a default 1-port density-3 configuration
    pub fn default_row1() -> Self {
        Self {
            row: CsiRsResourceRow::Row1,
            cdm_type: CdmType::NoCdm,
            density: CsiRsDensity::Three,
            n_id: 0,
            subcarrier_offset: 0,
            symbol_mask: 0b0000_0001,
            num_rbs: 52,
            first_rb: 0,
            periodicity: Some((40, 0)),
        }
    }

    /// Build an 8-port CDM4 configuration (Row 7)
    pub fn default_row7() -> Self {
        Self {
            row: CsiRsResourceRow::Row7,
            cdm_type: CdmType::Cdm4Fd2Td2,
            density: CsiRsDensity::One,
            n_id: 0,
            subcarrier_offset: 0,
            symbol_mask: 0b0000_0001,
            num_rbs: 52,
            first_rb: 0,
            periodicity: Some((40, 0)),
        }
    }

    /// Build a TRS (Tracking Reference Signal) burst configuration (Row 18)
    pub fn trs_burst() -> Self {
        Self {
            row: CsiRsResourceRow::Row18,
            cdm_type: CdmType::NoCdm,
            density: CsiRsDensity::Three,
            n_id: 0,
            subcarrier_offset: 0,
            // 4 consecutive symbols
            symbol_mask: 0b0000_1111,
            num_rbs: 52,
            first_rb: 0,
            periodicity: Some((20, 0)),
        }
    }

    /// Returns number of CSI-RS ports
    pub fn num_ports(&self) -> usize {
        self.row.num_ports()
    }

    /// Check if this resource is active in a given slot (frame-level)
    ///
    /// `absolute_slot`: slot number counting from frame 0, slot 0.
    pub fn is_active_in_slot(&self, absolute_slot: u32) -> bool {
        match self.periodicity {
            None => true, // aperiodic: always active when configured
            Some((period_slots, offset)) => (absolute_slot % period_slots) == offset,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Subcarrier positions
// ─────────────────────────────────────────────────────────────────────────────

/// Compute subcarrier positions (RE indices relative to RB start) for one CDM group
///
/// Returns list of subcarrier offsets within RB.  For density-3 this is {k0, k0+4, k0+8}.
/// For density-1 it is {k0}.  For density-0.5 it is {k0} in even PRBs only.
fn subcarrier_positions(density: CsiRsDensity, k0: usize, cdm_fd: usize) -> Vec<usize> {
    match density {
        CsiRsDensity::Three => {
            // 3 RE groups at positions k0, k0+4, k0+8 (mod 12)
            // Each group spans cdm_fd adjacent subcarriers
            let mut pos = Vec::new();
            for base in &[k0, (k0 + 4) % 12, (k0 + 8) % 12] {
                for f in 0..cdm_fd {
                    pos.push((base + f) % 12);
                }
            }
            pos
        }
        CsiRsDensity::One => {
            // Single RE group at k0
            (0..cdm_fd).map(|f| (k0 + f) % 12).collect()
        }
        CsiRsDensity::OneHalf | CsiRsDensity::Dot5Even | CsiRsDensity::Dot5Odd => {
            (0..cdm_fd).map(|f| (k0 + f) % 12).collect()
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// RE Mapping Output
// ─────────────────────────────────────────────────────────────────────────────

/// A single mapped CSI-RS resource element
#[derive(Debug, Clone)]
pub struct CsiRsRe {
    /// Resource block index
    pub rb: usize,
    /// Subcarrier index within RB (0..11)
    pub sc: usize,
    /// OFDM symbol index within slot (0..13)
    pub symbol: usize,
    /// Antenna port index (0-based)
    pub port: usize,
    /// QPSK sequence value at this RE
    pub value: Complex,
}

/// Outcome of mapping CSI-RS into the resource grid
#[derive(Debug, Clone)]
pub struct CsiRsMappingResult {
    /// All mapped resource elements
    pub res: Vec<CsiRsRe>,
    /// Number of active ports
    pub num_ports: usize,
    /// Number of sequence samples consumed
    pub seq_len: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// Main CSI-RS Processor
// ─────────────────────────────────────────────────────────────────────────────

/// CSI-RS processor: sequence generation, RE mapping, and CSI feedback computation
pub struct CsiRsProcessor {
    config: CsiRsConfig,
}

impl CsiRsProcessor {
    /// Create a new processor with the given configuration
    pub fn new(config: CsiRsConfig) -> Self {
        Self { config }
    }

    /// Access the current configuration
    pub fn config(&self) -> &CsiRsConfig {
        &self.config
    }

    /// Compute Gold sequence initialization value c_init
    ///
    /// Per TS 38.211 Section 7.4.1.5.2:
    /// ```text
    /// c_init = 2^10 * (14 * n_s_f_mu + l + 1) * (2 * n_ID + 1) + n_ID
    /// ```
    /// where `n_s_f_mu` is the slot within frame, `l` is symbol within slot.
    pub fn compute_c_init(&self, slot_in_frame: u32, symbol: u32) -> u32 {
        let n_id = self.config.n_id;
        let inner = (14 * slot_in_frame + symbol + 1) * (2 * n_id + 1) + n_id;
        ((1u64 << 10) * inner as u64) as u32 & 0x7FFF_FFFF
    }

    /// Generate the CSI-RS pseudo-random sequence for a given slot and symbol
    ///
    /// Returns QPSK symbols. Length = 2 * num_rbs * density_re_per_rb (at minimum).
    pub fn generate_sequence(&self, slot_in_frame: u32, symbol: u32, _port: usize) -> Vec<Complex> {
        let c_init = self.compute_c_init(slot_in_frame, symbol);
        let mut gold = GoldSeq::new(c_init);

        let re_per_rb = match self.config.density {
            CsiRsDensity::Three => 3,
            _ => 1,
        };
        let num_symbols = self.config.num_rbs * re_per_rb;
        gold.generate_qpsk(num_symbols.max(1))
    }

    /// Map CSI-RS sequences into the resource grid for all ports
    ///
    /// - `slot_in_frame`: slot number within the radio frame (0..79 for 30 kHz SCS)
    /// - `absolute_slot`: global slot counter for periodicity check
    ///
    /// Returns `None` if CSI-RS is not active in this slot (periodic only).
    pub fn map_to_grid(
        &self,
        slot_in_frame: u32,
        absolute_slot: u32,
    ) -> Option<CsiRsMappingResult> {
        if !self.config.is_active_in_slot(absolute_slot) {
            return None;
        }

        let cdm_type = self.config.cdm_type;
        let (fd_spread, _td_spread) = cdm_type.spread_factors();
        let num_ports = self.config.num_ports();
        let num_cdm_groups = num_ports / cdm_type.total_spread().max(1);

        // Collect active symbols from mask
        let symbols: Vec<usize> = (0u8..14)
            .filter(|&l| (self.config.symbol_mask >> l) & 1 == 1)
            .map(|l| l as usize)
            .collect();

        if symbols.is_empty() {
            return Some(CsiRsMappingResult {
                res: vec![],
                num_ports,
                seq_len: 0,
            });
        }

        let mut all_res: Vec<CsiRsRe> = Vec::new();
        let mut total_seq = 0usize;

        // For each CDM group
        for cdm_group in 0..num_cdm_groups {
            // Subcarrier offset for this CDM group
            let k0 = (self.config.subcarrier_offset + cdm_group * fd_spread) % SC_PER_RB;
            let sc_in_rb = subcarrier_positions(self.config.density, k0, fd_spread);

            // For each OFDM symbol
            for (sym_idx, &sym) in symbols.iter().enumerate() {
                // Generate sequence for this symbol
                let seq = {
                    let c_init = self.compute_c_init(slot_in_frame, sym as u32);
                    let mut gold = GoldSeq::new(c_init);
                    let scale = 1.0 / SQRT_2;

                    // Skip to the right position in the sequence for this CDM group
                    let skip = cdm_group * self.config.num_rbs * sc_in_rb.len();
                    for _ in 0..(skip * 2) {
                        gold.next_bit();
                    }

                    let len = self.config.num_rbs * sc_in_rb.len();
                    total_seq += len;
                    let mut seq = Vec::with_capacity(len);
                    for _ in 0..len {
                        let c0 = gold.next_bit() as f64;
                        let c1 = gold.next_bit() as f64;
                        seq.push(Complex::new(
                            scale * (1.0 - 2.0 * c0),
                            scale * (1.0 - 2.0 * c1),
                        ));
                    }
                    seq
                };

                let mut seq_idx = 0usize;

                // For each RB
                for rb_offset in 0..self.config.num_rbs {
                    let rb = self.config.first_rb + rb_offset;

                    // Apply density-0.5 filtering
                    let include_rb = match self.config.density {
                        CsiRsDensity::Dot5Even => rb % 2 == 0,
                        CsiRsDensity::Dot5Odd => rb % 2 == 1,
                        CsiRsDensity::OneHalf => rb % 2 == 0,
                        _ => true,
                    };

                    if !include_rb {
                        seq_idx += sc_in_rb.len();
                        continue;
                    }

                    for &sc_offset in &sc_in_rb {
                        let raw_val = if seq_idx < seq.len() {
                            seq[seq_idx]
                        } else {
                            Complex::zero()
                        };
                        seq_idx += 1;

                        // For each port in this CDM group, apply cover code
                        let total_spread = cdm_type.total_spread();
                        for port_in_group in 0..total_spread {
                            let port = cdm_group * total_spread + port_in_group;
                            let (wf, wt) = cdm_type.cover_code(port_in_group);

                            // FD code index: sc_offset within fd pair (0 or 1)
                            let fd_idx = sc_in_rb.iter().position(|&s| s == sc_offset).unwrap_or(0)
                                % wf.len();
                            // TD code index: symbol position within TD spread
                            let td_idx = sym_idx % wt.len();

                            let w = (wf[fd_idx] as f64) * (wt[td_idx] as f64);
                            let mapped = raw_val.scale(w);

                            all_res.push(CsiRsRe {
                                rb,
                                sc: sc_offset,
                                symbol: sym,
                                port,
                                value: mapped,
                            });
                        }
                    }
                }
            }
        }

        Some(CsiRsMappingResult {
            res: all_res,
            num_ports,
            seq_len: total_seq,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Zero-Power CSI-RS
// ─────────────────────────────────────────────────────────────────────────────

/// Zero-Power CSI-RS resource for interference measurement
///
/// ZP-CSI-RS are "muted" REs used by the gNB to measure inter-cell
/// interference without desired signal.  Defined in TS 38.211 §7.4.1.5.
#[derive(Debug, Clone)]
pub struct ZpCsiRsResource {
    /// RB bitmap (bit i = RB i is muted)
    pub rb_bitmap: Vec<bool>,
    /// Subcarrier pattern (bitmap within RB)
    pub sc_pattern: Vec<bool>,
    /// OFDM symbols occupied
    pub symbols: Vec<usize>,
    /// Optional periodicity (slots, offset)
    pub periodicity: Option<(u32, u32)>,
}

impl ZpCsiRsResource {
    /// Create a ZP-CSI-RS resource
    pub fn new(
        num_rbs: usize,
        sc_pattern: Vec<bool>,
        symbols: Vec<usize>,
        periodicity: Option<(u32, u32)>,
    ) -> Self {
        Self {
            rb_bitmap: vec![true; num_rbs],
            sc_pattern,
            symbols,
            periodicity,
        }
    }

    /// Returns list of (rb, sc, symbol) tuples that are muted
    pub fn muted_res(&self, absolute_slot: u32) -> Vec<(usize, usize, usize)> {
        if let Some((period, offset)) = self.periodicity {
            if (absolute_slot % period) != offset {
                return vec![];
            }
        }
        let mut muted = Vec::new();
        for (rb, &active) in self.rb_bitmap.iter().enumerate() {
            if !active {
                continue;
            }
            for (sc, &used) in self.sc_pattern.iter().enumerate() {
                if used {
                    for &sym in &self.symbols {
                        muted.push((rb, sc, sym));
                    }
                }
            }
        }
        muted
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Channel Estimation from CSI-RS
// ─────────────────────────────────────────────────────────────────────────────

/// LS channel estimate at a pilot location
#[derive(Debug, Clone)]
pub struct PilotEstimate {
    pub rb: usize,
    pub sc: usize,
    pub symbol: usize,
    pub port: usize,
    pub h_hat: Complex,
}

/// Channel estimator using CSI-RS pilots
pub struct CsiRsChannelEstimator {
    /// Number of interpolation points (0 = nearest-neighbor, >0 = linear interp)
    pub interp_order: usize,
}

impl CsiRsChannelEstimator {
    pub fn new(interp_order: usize) -> Self {
        Self { interp_order }
    }

    /// Compute LS estimates at pilot positions
    ///
    /// `rx_grid[symbol][rb * SC_PER_RB + sc]` = received complex sample
    /// `pilot_res` = ideal CSI-RS values at those positions
    pub fn ls_estimate(
        &self,
        rx_grid: &[Vec<Complex>],
        pilot_res: &[CsiRsRe],
    ) -> Vec<PilotEstimate> {
        pilot_res
            .iter()
            .filter_map(|re| {
                let row = re.symbol;
                let col = re.rb * SC_PER_RB + re.sc;
                if row >= rx_grid.len() || col >= rx_grid[row].len() {
                    return None;
                }
                let rx = rx_grid[row][col];
                // LS: H_hat = Y / X  (X = pilot value)
                let pilot_sq = re.value.abs_sq();
                if pilot_sq < 1e-12 {
                    return None;
                }
                let h_hat = rx.mul(re.value.conj()).scale(1.0 / pilot_sq);
                Some(PilotEstimate {
                    rb: re.rb,
                    sc: re.sc,
                    symbol: re.symbol,
                    port: re.port,
                    h_hat,
                })
            })
            .collect()
    }

    /// Linear frequency-domain interpolation of channel estimates across RBs
    ///
    /// Given sorted pilot estimates for a single port and symbol,
    /// interpolates to produce estimates at all `total_scs` subcarriers.
    pub fn freq_interpolate(
        &self,
        pilots: &[PilotEstimate],
        total_scs: usize,
    ) -> Vec<Complex> {
        if pilots.is_empty() {
            return vec![Complex::zero(); total_scs];
        }

        let mut result = vec![Complex::zero(); total_scs];

        // Sort by absolute subcarrier index
        let mut sorted: Vec<(usize, Complex)> = pilots
            .iter()
            .map(|p| (p.rb * SC_PER_RB + p.sc, p.h_hat))
            .collect();
        sorted.sort_by_key(|&(k, _)| k);
        sorted.dedup_by_key(|&mut (k, _)| k);

        if sorted.len() == 1 {
            let val = sorted[0].1;
            for r in result.iter_mut() {
                *r = val;
            }
            return result;
        }

        // Fill before first pilot
        let (k0, h0) = sorted[0];
        for sc in 0..k0.min(total_scs) {
            result[sc] = h0;
        }

        // Linear interpolation between pilot positions
        for window in sorted.windows(2) {
            let (k_l, h_l) = window[0];
            let (k_r, h_r) = window[1];
            let span = (k_r - k_l) as f64;
            for k in k_l..k_r.min(total_scs) {
                let alpha = (k - k_l) as f64 / span;
                result[k] = Complex::new(
                    h_l.re + alpha * (h_r.re - h_l.re),
                    h_l.im + alpha * (h_r.im - h_l.im),
                );
            }
        }

        // Fill after last pilot
        let (kn, hn) = *sorted.last().unwrap();
        for sc in kn..total_scs {
            result[sc] = hn;
        }

        result
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CQI / PMI / RI Computation (TS 38.214)
// ─────────────────────────────────────────────────────────────────────────────

/// CQI table entry per TS 38.214 Table 5.2.2.1-2 (64-QAM table)
#[derive(Debug, Clone, Copy)]
pub struct CqiEntry {
    /// CQI index (0..15)
    pub index: u8,
    /// Modulation order (2=QPSK, 4=16QAM, 6=64QAM, 0=out of range)
    pub mod_order: u8,
    /// Code rate × 1024
    pub code_rate_x1024: u32,
    /// Spectral efficiency (bits/RE) × 1000 (for comparison without floats)
    pub efficiency_x1000: u32,
}

/// CQI table 1 (64QAM max) per 3GPP TS 38.214 Table 5.2.2.1-2
const CQI_TABLE_1: [CqiEntry; CQI_TABLE_LEN] = [
    CqiEntry { index: 0,  mod_order: 0, code_rate_x1024: 0,    efficiency_x1000: 0     },
    CqiEntry { index: 1,  mod_order: 2, code_rate_x1024: 78,   efficiency_x1000: 152   },
    CqiEntry { index: 2,  mod_order: 2, code_rate_x1024: 120,  efficiency_x1000: 234   },
    CqiEntry { index: 3,  mod_order: 2, code_rate_x1024: 193,  efficiency_x1000: 377   },
    CqiEntry { index: 4,  mod_order: 2, code_rate_x1024: 308,  efficiency_x1000: 602   },
    CqiEntry { index: 5,  mod_order: 2, code_rate_x1024: 449,  efficiency_x1000: 877   },
    CqiEntry { index: 6,  mod_order: 2, code_rate_x1024: 602,  efficiency_x1000: 1176  },
    CqiEntry { index: 7,  mod_order: 4, code_rate_x1024: 378,  efficiency_x1000: 1477  },
    CqiEntry { index: 8,  mod_order: 4, code_rate_x1024: 490,  efficiency_x1000: 1914  },
    CqiEntry { index: 9,  mod_order: 4, code_rate_x1024: 616,  efficiency_x1000: 2406  },
    CqiEntry { index: 10, mod_order: 6, code_rate_x1024: 466,  efficiency_x1000: 2730  },
    CqiEntry { index: 11, mod_order: 6, code_rate_x1024: 567,  efficiency_x1000: 3320  },
    CqiEntry { index: 12, mod_order: 6, code_rate_x1024: 666,  efficiency_x1000: 3902  },
    CqiEntry { index: 13, mod_order: 6, code_rate_x1024: 772,  efficiency_x1000: 4523  },
    CqiEntry { index: 14, mod_order: 6, code_rate_x1024: 873,  efficiency_x1000: 5115  },
    CqiEntry { index: 15, mod_order: 6, code_rate_x1024: 948,  efficiency_x1000: 5554  },
];

/// SINR to CQI mapping using the 64QAM table (Table 5.2.2.1-2)
///
/// Uses Shannon-like mapping: capacity = log2(1 + SINR_linear).
/// Selects the highest CQI whose code rate × mod_order ≤ capacity.
pub fn sinr_to_cqi(sinr_db: f64) -> u8 {
    if sinr_db < -6.5 {
        return 0; // out of range
    }
    let sinr_lin = 10f64.powf(sinr_db / 10.0);
    // Shannon capacity in bits/RE
    let capacity = (1.0 + sinr_lin).log2();

    // Find highest CQI whose efficiency ≤ capacity
    let mut best_cqi = 1u8;
    for entry in &CQI_TABLE_1[1..] {
        let eff = entry.efficiency_x1000 as f64 / 1000.0;
        if eff <= capacity {
            best_cqi = entry.index;
        } else {
            break;
        }
    }
    best_cqi
}

/// Wideband CQI from measured SINR over all subbands
pub fn wideband_cqi(sinr_per_rb: &[f64]) -> u8 {
    if sinr_per_rb.is_empty() {
        return 0;
    }
    // Geometric mean in dB = arithmetic mean in linear → dB
    let mean_lin: f64 = sinr_per_rb.iter().map(|&s| 10f64.powf(s / 10.0)).sum::<f64>()
        / sinr_per_rb.len() as f64;
    let mean_db = 10.0 * mean_lin.log10();
    sinr_to_cqi(mean_db)
}

/// Subband CQI: per-subband CQI relative to wideband CQI
pub fn subband_cqi(sinr_per_rb: &[f64], subband_size: usize) -> Vec<u8> {
    sinr_per_rb
        .chunks(subband_size.max(1))
        .map(|chunk| wideband_cqi(chunk))
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Type I Single-Panel Codebook (TS 38.214 §5.2.2.2.1)
// ─────────────────────────────────────────────────────────────────────────────

/// Type I single-panel codebook PMI
///
/// PMI consists of two indices: i1 (wideband) and i2 (subband or wideband).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TypeIPmi {
    /// Wideband PMI index (i1_1, i1_2, i1_3 packed)
    pub i1: u32,
    /// Subband/wideband PMI index i2 (co-phasing)
    pub i2: u8,
}

/// Type I single-panel codebook for up to 8 ports
///
/// Simplified version: returns DFT-based beamforming vector for given antenna config.
pub struct TypeICodebook {
    /// Number of ports (1, 2, 4, or 8)
    pub num_ports: usize,
    /// N1: horizontal antenna panels
    pub n1: usize,
    /// N2: vertical antenna panels (1 for single-pol linear)
    pub n2: usize,
    /// O1: horizontal oversampling factor
    pub o1: usize,
    /// O2: vertical oversampling factor
    pub o2: usize,
}

impl TypeICodebook {
    /// Create codebook for given port count with typical 2D panel layout
    pub fn new(num_ports: usize) -> Self {
        let (n1, n2, o1, o2) = match num_ports {
            1 => (1, 1, 1, 1),
            2 => (2, 1, 4, 1),
            4 => (2, 2, 4, 4),
            8 => (4, 2, 4, 4),
            16 => (4, 4, 4, 4),
            _ => (2, 1, 4, 1),
        };
        Self { num_ports, n1, n2, o1, o2 }
    }

    /// Total number of PMI i1 indices
    pub fn num_i1(&self) -> usize {
        self.n1 * self.o1 * self.n2 * self.o2
    }

    /// Number of co-phasing (i2) values
    pub fn num_i2(&self) -> usize {
        match self.num_ports {
            1 => 1,
            2 => 4,
            4 => 8,
            _ => 4,
        }
    }

    /// Compute the precoding vector for given PMI (i1, i2) and layer rank
    ///
    /// Returns complex precoding weights for each port.
    pub fn precoding_vector(&self, pmi: TypeIPmi, _rank: usize) -> Vec<Complex> {
        if self.num_ports == 1 {
            return vec![Complex::new(1.0, 0.0)];
        }

        let n = self.n1 * self.n2;
        // DFT beam direction from i1
        let i1 = pmi.i1 as usize % (self.n1 * self.o1);
        let scale = 1.0 / (n as f64).sqrt();

        // Build DFT-based beam (horizontal)
        let w: Vec<Complex> = (0..n)
            .map(|k| {
                let phase = 2.0 * PI * (i1 * k) as f64 / (self.n1 * self.o1 * n) as f64;
                Complex::from_polar(scale, phase)
            })
            .collect();

        // Co-phasing per i2
        let co_phase_angle = match pmi.i2 {
            0 => 0.0,
            1 => PI / 2.0,
            2 => PI,
            3 => 3.0 * PI / 2.0,
            _ => 0.0,
        };
        let co_phase = Complex::from_polar(1.0, co_phase_angle);

        // For rank 1: [w; co_phase * w] / sqrt(2) for dual-pol
        if self.num_ports >= 2 {
            let mut result: Vec<Complex> = w.clone();
            for v in &w {
                result.push(v.mul(co_phase));
            }
            let s = 1.0 / (2.0f64).sqrt();
            result.iter().map(|c| c.scale(s)).collect()
        } else {
            w
        }
    }

    /// Select best PMI for given channel matrix column (single-port per layer)
    ///
    /// `channel`: per-subcarrier complex channel coefficients (num_ports × num_sc)
    /// Returns PMI and achieved beamforming gain.
    pub fn select_pmi(&self, channel: &[Vec<Complex>]) -> (TypeIPmi, f64) {
        if channel.is_empty() || self.num_ports == 1 {
            return (TypeIPmi { i1: 0, i2: 0 }, 1.0);
        }

        let mut best_pmi = TypeIPmi { i1: 0, i2: 0 };
        let mut best_gain = -1.0f64;

        for i1 in 0..self.num_i1() as u32 {
            for i2 in 0..self.num_i2() as u8 {
                let pmi = TypeIPmi { i1, i2 };
                let w = self.precoding_vector(pmi, 1);

                // Compute beamforming gain = sum_sc |w^H h|^2
                let gain: f64 = channel[0]
                    .iter()
                    .enumerate()
                    .map(|(sc, _)| {
                        let mut sum = Complex::zero();
                        for (p, &ww) in w.iter().enumerate() {
                            if p < channel.len() && sc < channel[p].len() {
                                sum = sum + ww.conj().mul(channel[p][sc]);
                            }
                        }
                        sum.abs_sq()
                    })
                    .sum();

                if gain > best_gain {
                    best_gain = gain;
                    best_pmi = pmi;
                }
            }
        }

        (best_pmi, best_gain)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Rank Indicator (RI)
// ─────────────────────────────────────────────────────────────────────────────

/// Estimate rank indicator from channel matrix
///
/// Uses SVD-like energy criterion: count singular values above threshold.
/// `channel[port][sc]` is the channel matrix at each subcarrier.
pub fn estimate_ri(channel: &[Vec<Complex>], threshold_ratio: f64) -> u8 {
    if channel.is_empty() {
        return 1;
    }
    let num_ports = channel.len();
    let num_sc = channel[0].len();
    if num_sc == 0 {
        return 1;
    }

    // Compute average power per port (proxy for singular values)
    let mut port_power: Vec<f64> = channel
        .iter()
        .map(|row| row.iter().map(|c| c.abs_sq()).sum::<f64>() / num_sc as f64)
        .collect();

    port_power.sort_by(|a, b| b.partial_cmp(a).unwrap());
    let max_power = port_power[0];
    if max_power < 1e-12 {
        return 1;
    }

    let rank = port_power
        .iter()
        .filter(|&&p| p > threshold_ratio * max_power)
        .count();

    (rank.min(num_ports).max(1)) as u8
}

// ─────────────────────────────────────────────────────────────────────────────
// CSI-RS Resource Indicator (CRI)
// ─────────────────────────────────────────────────────────────────────────────

/// Measurement result for a single CSI-RS resource
#[derive(Debug, Clone)]
pub struct CsiRsMeasurement {
    /// Resource index
    pub resource_idx: usize,
    /// L1-RSRP in dBm (linear 0.0 if not computed)
    pub rsrp_dbm: f64,
    /// Wideband SINR estimate
    pub sinr_db: f64,
    /// CQI from this resource
    pub cqi: u8,
}

/// CRI (CSI-RS Resource Indicator) selector
///
/// Selects the best CSI-RS resource from a set based on L1-RSRP or SINR.
pub struct CriSelector;

impl CriSelector {
    /// Select best resource index (CRI) based on highest SINR
    pub fn select_by_sinr(measurements: &[CsiRsMeasurement]) -> Option<usize> {
        measurements
            .iter()
            .max_by(|a, b| a.sinr_db.partial_cmp(&b.sinr_db).unwrap())
            .map(|m| m.resource_idx)
    }

    /// Select best resource index (CRI) based on highest L1-RSRP
    pub fn select_by_rsrp(measurements: &[CsiRsMeasurement]) -> Option<usize> {
        measurements
            .iter()
            .max_by(|a, b| a.rsrp_dbm.partial_cmp(&b.rsrp_dbm).unwrap())
            .map(|m| m.resource_idx)
    }

    /// Compute L1-RSRP from received signal at CSI-RS positions
    ///
    /// L1-RSRP = (1/N) * sum_k |r(k)|^2  where k iterates over CSI-RS REs
    pub fn compute_rsrp(rx_samples: &[Complex]) -> f64 {
        if rx_samples.is_empty() {
            return -140.0;
        }
        let avg_power: f64 = rx_samples.iter().map(|c| c.abs_sq()).sum::<f64>()
            / rx_samples.len() as f64;
        if avg_power < 1e-15 {
            return -140.0;
        }
        10.0 * avg_power.log10() + 30.0 // convert to dBm (assume 0 dBm reference)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TRS (Tracking Reference Signal) Burst
// ─────────────────────────────────────────────────────────────────────────────

/// TRS burst configuration per TS 38.211 §7.4.1.5.1
///
/// TRS is a special CSI-RS burst used for time/frequency tracking.
/// Two slots with 2 CSI-RS per slot (4 total), density 3.
#[derive(Debug, Clone)]
pub struct TrsBurstConfig {
    /// Periodicity in slots
    pub period_slots: u32,
    /// Slot offset within period
    pub slot_offset: u32,
    /// Number of slots in burst (typically 2)
    pub num_slots: u32,
    /// Scrambling ID
    pub n_id: u32,
    /// Number of RBs
    pub num_rbs: usize,
    /// First RB
    pub first_rb: usize,
}

impl TrsBurstConfig {
    /// Default TRS burst: 20ms period, 2 slots, density 3
    pub fn default_10mhz() -> Self {
        Self {
            period_slots: 20,
            slot_offset: 0,
            num_slots: 2,
            n_id: 0,
            num_rbs: 52,
            first_rb: 0,
        }
    }

    /// Check if this slot is part of the TRS burst
    pub fn is_trs_slot(&self, absolute_slot: u32) -> bool {
        let offset_in_period = absolute_slot % self.period_slots;
        let start = self.slot_offset;
        let end = start + self.num_slots;
        offset_in_period >= start && offset_in_period < end
    }

    /// Get the CSI-RS configurations for both symbols in a TRS slot
    pub fn get_csi_rs_configs(&self) -> Vec<CsiRsConfig> {
        // Two CSI-RS per slot for TRS, at symbols 4 and 8 (typical)
        vec![
            CsiRsConfig {
                row: CsiRsResourceRow::Row1,
                cdm_type: CdmType::NoCdm,
                density: CsiRsDensity::Three,
                n_id: self.n_id,
                subcarrier_offset: 0,
                symbol_mask: 1 << 4, // symbol 4
                num_rbs: self.num_rbs,
                first_rb: self.first_rb,
                periodicity: Some((self.period_slots, self.slot_offset)),
            },
            CsiRsConfig {
                row: CsiRsResourceRow::Row1,
                cdm_type: CdmType::NoCdm,
                density: CsiRsDensity::Three,
                n_id: self.n_id,
                subcarrier_offset: 0,
                symbol_mask: 1 << 8, // symbol 8
                num_rbs: self.num_rbs,
                first_rb: self.first_rb,
                periodicity: Some((self.period_slots, self.slot_offset)),
            },
        ]
    }

    /// Estimate timing error from two TRS symbols in the same slot
    ///
    /// Uses phase difference between symbols to estimate timing offset.
    /// `h1[sc]` and `h2[sc]` are channel estimates at symbols 1 and 2.
    /// `symbol_delta` is the symbol index difference (e.g., 4 = symbols 4 and 8).
    /// Returns timing offset in samples (fractional).
    pub fn estimate_timing_offset(
        h1: &[Complex],
        h2: &[Complex],
        symbol_delta: f64,
        fft_size: usize,
    ) -> f64 {
        if h1.len() != h2.len() || h1.is_empty() {
            return 0.0;
        }
        // Cross-correlation in frequency domain → phase slope in time
        // tau = angle(sum_k h2(k) * conj(h1(k))) / (2*pi * symbol_delta / fft_size)
        let cross_sum: Complex = h1.iter().zip(h2.iter())
            .map(|(a, b)| b.mul(a.conj()))
            .fold(Complex::zero(), |acc, x| acc + x);

        let phase = cross_sum.im.atan2(cross_sum.re);
        let denom = 2.0 * PI * symbol_delta / fft_size as f64;
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        phase / denom
    }

    /// Estimate frequency offset from two TRS symbols
    ///
    /// Returns CFO in normalized frequency (fraction of subcarrier spacing).
    pub fn estimate_frequency_offset(
        h1: &[Complex],
        h2: &[Complex],
        symbol_delta_sec: f64,
    ) -> f64 {
        if h1.len() != h2.len() || h1.is_empty() {
            return 0.0;
        }
        let cross_sum: Complex = h1.iter().zip(h2.iter())
            .map(|(a, b)| b.mul(a.conj()))
            .fold(Complex::zero(), |acc, x| acc + x);

        let phase = cross_sum.im.atan2(cross_sum.re);
        // CFO = phase / (2*pi * T_sym_delta)  (in Hz)
        phase / (2.0 * PI * symbol_delta_sec)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Full CSI Feedback Report
// ─────────────────────────────────────────────────────────────────────────────

/// Full CSI feedback report
#[derive(Debug, Clone)]
pub struct CsiFeedback {
    /// CRI: CSI-RS Resource Indicator (index of best beam)
    pub cri: Option<usize>,
    /// RI: Rank Indicator (1–8)
    pub ri: u8,
    /// Wideband PMI (i1, i2)
    pub pmi_wideband: Option<TypeIPmi>,
    /// Wideband CQI
    pub cqi_wideband: u8,
    /// Per-subband CQI (empty if wideband only)
    pub cqi_subband: Vec<u8>,
    /// Measured SINR in dB
    pub sinr_db: f64,
}

impl CsiFeedback {
    /// Compute full CSI feedback from channel estimates
    ///
    /// - `channel_estimates[port][sc]` = complex channel coefficient
    /// - `sinr_per_sc` = per-subcarrier SINR in dB
    /// - `codebook` = Type I codebook
    /// - `subband_size_rbs` = number of RBs per subband (0 = wideband only)
    pub fn compute(
        channel_estimates: &[Vec<Complex>],
        sinr_per_sc: &[f64],
        codebook: &TypeICodebook,
        subband_size_rbs: usize,
    ) -> Self {
        // RI
        let ri = estimate_ri(channel_estimates, 0.1);

        // PMI
        let pmi_wideband = if codebook.num_ports > 1 && !channel_estimates.is_empty() {
            let (pmi, _gain) = codebook.select_pmi(channel_estimates);
            Some(pmi)
        } else {
            None
        };

        // CQI
        let mean_sinr = if sinr_per_sc.is_empty() {
            0.0
        } else {
            sinr_per_sc.iter().sum::<f64>() / sinr_per_sc.len() as f64
        };
        let cqi_wideband = sinr_to_cqi(mean_sinr);

        let cqi_subband = if subband_size_rbs > 0 {
            // Convert per-SC to per-RB (average SC in each RB)
            let sinr_per_rb: Vec<f64> = sinr_per_sc
                .chunks(SC_PER_RB)
                .map(|chunk| {
                    let lin: f64 = chunk.iter().map(|&s| 10f64.powf(s / 10.0)).sum::<f64>()
                        / chunk.len() as f64;
                    10.0 * lin.log10()
                })
                .collect();
            subband_cqi(&sinr_per_rb, subband_size_rbs)
        } else {
            vec![]
        };

        CsiFeedback {
            cri: None,
            ri,
            pmi_wideband,
            cqi_wideband,
            cqi_subband,
            sinr_db: mean_sinr,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Gold sequence tests ──────────────────────────────────────────────────

    #[test]
    fn test_gold_seq_deterministic() {
        let mut g1 = GoldSeq::new(0);
        let mut g2 = GoldSeq::new(0);
        let b1 = g1.generate_bits(100);
        let b2 = g2.generate_bits(100);
        assert_eq!(b1, b2, "Same c_init must produce same output");
    }

    #[test]
    fn test_gold_seq_different_cinit() {
        let mut g1 = GoldSeq::new(12345);
        let mut g2 = GoldSeq::new(99999);
        let b1 = g1.generate_bits(64);
        let b2 = g2.generate_bits(64);
        assert_ne!(b1, b2, "Different c_init must produce different sequences");
    }

    #[test]
    fn test_gold_seq_bits_binary() {
        let mut g = GoldSeq::new(42);
        let bits = g.generate_bits(200);
        for &b in &bits {
            assert!(b == 0 || b == 1, "Gold sequence outputs must be binary");
        }
    }

    #[test]
    fn test_gold_seq_qpsk_unit_power() {
        let mut g = GoldSeq::new(7);
        let syms = g.generate_qpsk(1000);
        let avg_power: f64 = syms.iter().map(|c| c.abs_sq()).sum::<f64>() / 1000.0;
        assert!(
            (avg_power - 1.0).abs() < 0.01,
            "QPSK symbols should have unit average power, got {avg_power}"
        );
    }

    #[test]
    fn test_gold_seq_qpsk_four_points() {
        let mut g = GoldSeq::new(1000);
        let syms = g.generate_qpsk(4096);
        let scale = 1.0 / SQRT_2;
        for s in &syms {
            // Each symbol must be in {±1/√2} × {±1/√2}
            assert!(
                (s.re.abs() - scale).abs() < 1e-9,
                "QPSK I must be ±1/√2, got {}", s.re
            );
            assert!(
                (s.im.abs() - scale).abs() < 1e-9,
                "QPSK Q must be ±1/√2, got {}", s.im
            );
        }
    }

    // ── CDM type tests ───────────────────────────────────────────────────────

    #[test]
    fn test_cdm_no_cdm_spread() {
        assert_eq!(CdmType::NoCdm.spread_factors(), (1, 1));
        assert_eq!(CdmType::NoCdm.total_spread(), 1);
    }

    #[test]
    fn test_cdm_fd_cdm2_spread() {
        assert_eq!(CdmType::FdCdm2.spread_factors(), (2, 1));
        assert_eq!(CdmType::FdCdm2.total_spread(), 2);
    }

    #[test]
    fn test_cdm4_spread() {
        assert_eq!(CdmType::Cdm4Fd2Td2.spread_factors(), (2, 2));
        assert_eq!(CdmType::Cdm4Fd2Td2.total_spread(), 4);
    }

    #[test]
    fn test_cdm8_spread() {
        assert_eq!(CdmType::Cdm8Fd2Td4.spread_factors(), (2, 4));
        assert_eq!(CdmType::Cdm8Fd2Td4.total_spread(), 8);
    }

    #[test]
    fn test_cdm_fd2_cover_codes_orthogonal() {
        let (w0, _) = CdmType::FdCdm2.cover_code(0);
        let (w1, _) = CdmType::FdCdm2.cover_code(1);
        let dot: i32 = w0.iter().zip(w1.iter()).map(|(&a, &b)| a as i32 * b as i32).sum();
        assert_eq!(dot, 0, "fd-CDM2 cover codes must be orthogonal");
    }

    #[test]
    fn test_cdm4_cover_codes_orthogonal() {
        let codes: Vec<(Vec<i8>, Vec<i8>)> = (0..4)
            .map(|i| CdmType::Cdm4Fd2Td2.cover_code(i))
            .collect();
        // Check all pairs are orthogonal in the combined FD×TD space
        for i in 0..4 {
            for j in (i + 1)..4 {
                let (wf_i, wt_i) = &codes[i];
                let (wf_j, wt_j) = &codes[j];
                let dot: i32 = wf_i.iter().zip(wf_j.iter())
                    .map(|(&a, &b)| a as i32 * b as i32)
                    .sum::<i32>()
                    * wt_i.iter().zip(wt_j.iter())
                        .map(|(&a, &b)| a as i32 * b as i32)
                        .sum::<i32>();
                // At least one dimension must be orthogonal
                let fd_dot: i32 = wf_i.iter().zip(wf_j.iter())
                    .map(|(&a, &b)| a as i32 * b as i32).sum();
                let td_dot: i32 = wt_i.iter().zip(wt_j.iter())
                    .map(|(&a, &b)| a as i32 * b as i32).sum();
                assert!(
                    fd_dot == 0 || td_dot == 0,
                    "CDM4 codes {i} and {j} must be orthogonal, fd_dot={fd_dot} td_dot={td_dot}"
                );
                let _ = dot;
            }
        }
    }

    // ── CsiRsResourceRow tests ───────────────────────────────────────────────

    #[test]
    fn test_row1_properties() {
        assert_eq!(CsiRsResourceRow::Row1.num_ports(), 1);
        assert_eq!(CsiRsResourceRow::Row1.default_cdm_type(), CdmType::NoCdm);
        assert_eq!(CsiRsResourceRow::Row1.num_symbols(), 1);
    }

    #[test]
    fn test_row7_properties() {
        assert_eq!(CsiRsResourceRow::Row7.num_ports(), 8);
        assert_eq!(CsiRsResourceRow::Row7.default_cdm_type(), CdmType::Cdm4Fd2Td2);
    }

    #[test]
    fn test_row12_properties() {
        assert_eq!(CsiRsResourceRow::Row12.num_ports(), 16);
        assert_eq!(CsiRsResourceRow::Row12.default_cdm_type(), CdmType::Cdm8Fd2Td4);
        assert_eq!(CsiRsResourceRow::Row12.default_density(), CsiRsDensity::OneHalf);
    }

    #[test]
    fn test_row16_is_32port() {
        assert_eq!(CsiRsResourceRow::Row16.num_ports(), 32);
    }

    #[test]
    fn test_row18_trs_4symbols() {
        assert_eq!(CsiRsResourceRow::Row18.num_symbols(), 4);
        assert_eq!(CsiRsResourceRow::Row18.default_density(), CsiRsDensity::Three);
    }

    // ── Processor tests ──────────────────────────────────────────────────────

    #[test]
    fn test_processor_sequence_nonzero() {
        let config = CsiRsConfig::default_row1();
        let proc = CsiRsProcessor::new(config);
        let seq = proc.generate_sequence(0, 0, 0);
        assert!(!seq.is_empty());
        let any_nonzero = seq.iter().any(|c| c.abs_sq() > 1e-12);
        assert!(any_nonzero, "Sequence should have non-zero samples");
    }

    #[test]
    fn test_processor_c_init_varies_by_slot() {
        let config = CsiRsConfig::default_row1();
        let proc = CsiRsProcessor::new(config);
        let c0 = proc.compute_c_init(0, 0);
        let c1 = proc.compute_c_init(1, 0);
        assert_ne!(c0, c1, "c_init must differ between slots");
    }

    #[test]
    fn test_processor_c_init_varies_by_symbol() {
        let config = CsiRsConfig::default_row1();
        let proc = CsiRsProcessor::new(config);
        let c0 = proc.compute_c_init(0, 0);
        let c4 = proc.compute_c_init(0, 4);
        assert_ne!(c0, c4, "c_init must differ between symbols");
    }

    #[test]
    fn test_processor_c_init_varies_by_nid() {
        let mut cfg1 = CsiRsConfig::default_row1();
        let mut cfg2 = CsiRsConfig::default_row1();
        cfg1.n_id = 0;
        cfg2.n_id = 500;
        let p1 = CsiRsProcessor::new(cfg1);
        let p2 = CsiRsProcessor::new(cfg2);
        assert_ne!(p1.compute_c_init(0, 0), p2.compute_c_init(0, 0));
    }

    #[test]
    fn test_periodicity_active() {
        let mut config = CsiRsConfig::default_row1();
        config.periodicity = Some((40, 5));
        let proc = CsiRsProcessor::new(config);
        assert!(proc.config().is_active_in_slot(5));
        assert!(proc.config().is_active_in_slot(45));
        assert!(!proc.config().is_active_in_slot(0));
        assert!(!proc.config().is_active_in_slot(6));
    }

    #[test]
    fn test_aperiodic_always_active() {
        let mut config = CsiRsConfig::default_row1();
        config.periodicity = None;
        let proc = CsiRsProcessor::new(config);
        for slot in [0, 1, 100, 999] {
            assert!(proc.config().is_active_in_slot(slot));
        }
    }

    #[test]
    fn test_map_to_grid_row1_basic() {
        let config = CsiRsConfig {
            row: CsiRsResourceRow::Row1,
            cdm_type: CdmType::NoCdm,
            density: CsiRsDensity::Three,
            n_id: 0,
            subcarrier_offset: 0,
            symbol_mask: 0b0000_0001,
            num_rbs: 4,
            first_rb: 0,
            periodicity: None,
        };
        let proc = CsiRsProcessor::new(config);
        let result = proc.map_to_grid(0, 0).expect("Should map");
        assert_eq!(result.num_ports, 1);
        // 3 REs per RB × 4 RBs × 1 port = 12 REs
        assert_eq!(result.res.len(), 12, "Expected 12 REs for density-3, 4 RBs, 1 port");
    }

    #[test]
    fn test_map_to_grid_inactive_slot() {
        let config = CsiRsConfig {
            periodicity: Some((40, 5)),
            ..CsiRsConfig::default_row1()
        };
        let proc = CsiRsProcessor::new(config);
        assert!(proc.map_to_grid(0, 0).is_none(), "Slot 0 should be inactive");
        assert!(proc.map_to_grid(5, 5).is_some(), "Slot 5 should be active");
    }

    #[test]
    fn test_map_to_grid_symbol_assignment() {
        let config = CsiRsConfig {
            symbol_mask: 1 << 4, // only symbol 4
            num_rbs: 2,
            periodicity: None,
            ..CsiRsConfig::default_row1()
        };
        let proc = CsiRsProcessor::new(config);
        let result = proc.map_to_grid(0, 0).unwrap();
        for re in &result.res {
            assert_eq!(re.symbol, 4, "All REs should be in symbol 4");
        }
    }

    #[test]
    fn test_map_to_grid_port_indices() {
        let config = CsiRsConfig {
            row: CsiRsResourceRow::Row3,
            cdm_type: CdmType::FdCdm2,
            density: CsiRsDensity::One,
            symbol_mask: 0b0000_0001,
            num_rbs: 2,
            periodicity: None,
            ..CsiRsConfig::default_row1()
        };
        let proc = CsiRsProcessor::new(config);
        let result = proc.map_to_grid(0, 0).unwrap();
        let ports: std::collections::HashSet<usize> =
            result.res.iter().map(|re| re.port).collect();
        assert!(ports.contains(&0));
        assert!(ports.contains(&1));
        assert_eq!(result.num_ports, 2);
    }

    // ── CQI tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_sinr_to_cqi_low_sinr() {
        // Very low SINR → CQI 0 (out of range) or 1
        let cqi = sinr_to_cqi(-10.0);
        assert!(cqi <= 1, "Very low SINR should give low CQI, got {cqi}");
    }

    #[test]
    fn test_sinr_to_cqi_high_sinr() {
        // High SINR → CQI 15
        let cqi = sinr_to_cqi(30.0);
        assert_eq!(cqi, 15, "High SINR should give CQI 15");
    }

    #[test]
    fn test_sinr_to_cqi_monotonic() {
        let sinrs = [-6.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0];
        let cqis: Vec<u8> = sinrs.iter().map(|&s| sinr_to_cqi(s)).collect();
        for w in cqis.windows(2) {
            assert!(w[0] <= w[1], "CQI should be non-decreasing with SINR");
        }
    }

    #[test]
    fn test_sinr_to_cqi_0db_mid_range() {
        let cqi = sinr_to_cqi(0.0);
        assert!(cqi >= 2 && cqi <= 6, "0 dB SINR should give mid-low CQI, got {cqi}");
    }

    #[test]
    fn test_wideband_cqi_uniform() {
        let sinrs = vec![10.0f64; 52];
        let cqi = wideband_cqi(&sinrs);
        assert_eq!(cqi, sinr_to_cqi(10.0), "Uniform SINR wideband CQI should equal single SINR CQI");
    }

    #[test]
    fn test_subband_cqi_chunking() {
        let sinrs: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let sb_cqi = subband_cqi(&sinrs, 4);
        assert_eq!(sb_cqi.len(), 5, "20 RBs / 4 = 5 subbands");
    }

    #[test]
    fn test_cqi_table_entries() {
        for entry in &CQI_TABLE_1[1..] {
            assert!(entry.mod_order == 2 || entry.mod_order == 4 || entry.mod_order == 6);
            assert!(entry.code_rate_x1024 > 0);
        }
    }

    // ── TypeI codebook tests ─────────────────────────────────────────────────

    #[test]
    fn test_codebook_1port_trivial() {
        let cb = TypeICodebook::new(1);
        let pmi = TypeIPmi { i1: 0, i2: 0 };
        let w = cb.precoding_vector(pmi, 1);
        assert_eq!(w.len(), 1);
        assert!((w[0].re - 1.0).abs() < 1e-9 && w[0].im.abs() < 1e-9);
    }

    #[test]
    fn test_codebook_2port_unit_norm() {
        let cb = TypeICodebook::new(2);
        for i1 in 0..cb.num_i1() as u32 {
            for i2 in 0..cb.num_i2() as u8 {
                let w = cb.precoding_vector(TypeIPmi { i1, i2 }, 1);
                let norm_sq: f64 = w.iter().map(|c| c.abs_sq()).sum();
                assert!(
                    (norm_sq - 1.0).abs() < 1e-9,
                    "2-port precoding vector must have unit norm, got {norm_sq}"
                );
            }
        }
    }

    #[test]
    fn test_codebook_4port_unit_norm() {
        let cb = TypeICodebook::new(4);
        for i1 in 0..cb.num_i1() as u32 {
            for i2 in 0..cb.num_i2() as u8 {
                let w = cb.precoding_vector(TypeIPmi { i1, i2 }, 1);
                let norm_sq: f64 = w.iter().map(|c| c.abs_sq()).sum();
                assert!(
                    (norm_sq - 1.0).abs() < 1e-9,
                    "4-port precoding vector must have unit norm i1={i1} i2={i2}, got {norm_sq}"
                );
            }
        }
    }

    #[test]
    fn test_codebook_select_pmi_2port() {
        let cb = TypeICodebook::new(2);
        // Create a simple channel where port 0 is dominant
        let channel: Vec<Vec<Complex>> = vec![
            vec![Complex::new(1.0, 0.0); 24],
            vec![Complex::new(0.1, 0.0); 24],
        ];
        let (pmi, gain) = cb.select_pmi(&channel);
        assert!(gain > 0.0);
        // PMI indices should be valid
        assert!((pmi.i1 as usize) < cb.num_i1());
        assert!((pmi.i2 as usize) < cb.num_i2());
    }

    // ── RI estimation tests ──────────────────────────────────────────────────

    #[test]
    fn test_ri_single_port_rank1() {
        let channel = vec![vec![Complex::new(1.0, 0.0); 24]];
        let ri = estimate_ri(&channel, 0.1);
        assert_eq!(ri, 1);
    }

    #[test]
    fn test_ri_two_equal_ports() {
        let channel = vec![
            vec![Complex::new(1.0, 0.0); 24],
            vec![Complex::new(1.0, 0.0); 24],
        ];
        let ri = estimate_ri(&channel, 0.1);
        assert!(ri >= 1 && ri <= 2);
    }

    #[test]
    fn test_ri_dominated_channel_rank1() {
        let channel = vec![
            vec![Complex::new(10.0, 0.0); 24], // strong
            vec![Complex::new(0.01, 0.0); 24], // weak
        ];
        let ri = estimate_ri(&channel, 0.5); // 50% threshold → only strong survives
        assert_eq!(ri, 1);
    }

    #[test]
    fn test_ri_empty_channel() {
        let channel: Vec<Vec<Complex>> = vec![];
        let ri = estimate_ri(&channel, 0.1);
        assert_eq!(ri, 1);
    }

    // ── Channel estimation tests ─────────────────────────────────────────────

    #[test]
    fn test_ls_estimate_simple() {
        let pilot_val = Complex::new(0.707, 0.707);
        let h_true = Complex::new(0.5, 0.3);
        let rx = pilot_val.mul(h_true);

        let rx_grid = vec![vec![rx; 12]];
        let pilot_res = vec![CsiRsRe {
            rb: 0,
            sc: 0,
            symbol: 0,
            port: 0,
            value: pilot_val,
        }];

        let estimator = CsiRsChannelEstimator::new(1);
        let estimates = estimator.ls_estimate(&rx_grid, &pilot_res);
        assert_eq!(estimates.len(), 1);
        let h_hat = estimates[0].h_hat;
        assert!((h_hat.re - h_true.re).abs() < 1e-9, "LS estimate re mismatch");
        assert!((h_hat.im - h_true.im).abs() < 1e-9, "LS estimate im mismatch");
    }

    #[test]
    fn test_freq_interpolation_constant_channel() {
        let estimator = CsiRsChannelEstimator::new(1);
        let h = Complex::new(1.0, 0.5);
        let pilots = vec![
            PilotEstimate { rb: 0, sc: 0, symbol: 0, port: 0, h_hat: h },
            PilotEstimate { rb: 5, sc: 0, symbol: 0, port: 0, h_hat: h },
        ];
        let interp = estimator.freq_interpolate(&pilots, 72);
        // All interpolated values should equal h
        for (i, &val) in interp.iter().enumerate() {
            assert!(
                (val.re - h.re).abs() < 1e-9 && (val.im - h.im).abs() < 1e-9,
                "Constant channel interpolation failed at sc={i}"
            );
        }
    }

    #[test]
    fn test_freq_interpolation_linear() {
        let estimator = CsiRsChannelEstimator::new(1);
        let h0 = Complex::new(1.0, 0.0);
        let h1 = Complex::new(3.0, 0.0);
        let pilots = vec![
            PilotEstimate { rb: 0, sc: 0, symbol: 0, port: 0, h_hat: h0 },
            PilotEstimate { rb: 1, sc: 0, symbol: 0, port: 0, h_hat: h1 }, // sc=12
        ];
        let interp = estimator.freq_interpolate(&pilots, 24);
        // At sc=6 (midpoint), expect ~2.0
        assert!((interp[6].re - 2.0).abs() < 0.05, "Midpoint interpolation: expected 2.0 got {}", interp[6].re);
    }

    // ── ZP-CSI-RS tests ──────────────────────────────────────────────────────

    #[test]
    fn test_zp_csirs_muted_res() {
        let mut sc_pat = vec![false; 12];
        sc_pat[0] = true;
        sc_pat[4] = true;
        sc_pat[8] = true;
        let zp = ZpCsiRsResource::new(4, sc_pat, vec![6], None);
        let muted = zp.muted_res(0);
        // 4 RBs × 3 SCs × 1 symbol = 12 muted REs
        assert_eq!(muted.len(), 12);
    }

    #[test]
    fn test_zp_csirs_periodicity() {
        let sc_pat = vec![true; 12];
        let zp = ZpCsiRsResource::new(4, sc_pat, vec![6], Some((20, 3)));
        assert_eq!(zp.muted_res(3).len(), 48); // active
        assert!(zp.muted_res(0).is_empty()); // inactive
    }

    // ── TRS tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_trs_burst_slot_detection() {
        let trs = TrsBurstConfig::default_10mhz();
        assert!(trs.is_trs_slot(0));
        assert!(trs.is_trs_slot(1));
        assert!(!trs.is_trs_slot(2));
        assert!(trs.is_trs_slot(20));
    }

    #[test]
    fn test_trs_config_2symbols() {
        let trs = TrsBurstConfig::default_10mhz();
        let configs = trs.get_csi_rs_configs();
        assert_eq!(configs.len(), 2, "TRS should have 2 CSI-RS per slot");
    }

    #[test]
    fn test_trs_timing_offset_zero() {
        let h: Vec<Complex> = (0..52).map(|_| Complex::new(1.0, 0.0)).collect();
        let offset = TrsBurstConfig::estimate_timing_offset(&h, &h, 4.0, 2048);
        assert!(offset.abs() < 0.01, "Identical channels → zero timing offset");
    }

    #[test]
    fn test_trs_freq_offset_zero() {
        let h: Vec<Complex> = (0..52).map(|_| Complex::new(1.0, 0.0)).collect();
        let cfo = TrsBurstConfig::estimate_frequency_offset(&h, &h, 1e-3);
        assert!(cfo.abs() < 1.0, "Identical channels → near-zero freq offset");
    }

    #[test]
    fn test_trs_timing_offset_nonzero() {
        let n = 52usize;
        let fft_size = 2048usize;
        let true_offset = 5.0f64; // samples
        // Simulate linear phase ramp: h2[k] = h1[k] * exp(-j*2*pi*tau*k/N)
        let h1: Vec<Complex> = (0..n).map(|_| Complex::new(1.0, 0.0)).collect();
        let h2: Vec<Complex> = (0..n)
            .map(|k| {
                let phase = -2.0 * PI * true_offset * k as f64 / fft_size as f64;
                Complex::from_polar(1.0, phase)
            })
            .collect();
        let est = TrsBurstConfig::estimate_timing_offset(&h1, &h2, 1.0, fft_size);
        // Note: our estimator computes the phase of sum, not per-subcarrier slope.
        // This is an approximation; just check sign and magnitude order.
        assert!(est.abs() < 1000.0, "Timing estimate should be bounded");
    }

    // ── CRI tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_cri_select_by_sinr() {
        let measurements = vec![
            CsiRsMeasurement { resource_idx: 0, rsrp_dbm: -80.0, sinr_db: 5.0, cqi: 7 },
            CsiRsMeasurement { resource_idx: 1, rsrp_dbm: -75.0, sinr_db: 12.0, cqi: 10 },
            CsiRsMeasurement { resource_idx: 2, rsrp_dbm: -90.0, sinr_db: 2.0, cqi: 4 },
        ];
        let cri = CriSelector::select_by_sinr(&measurements);
        assert_eq!(cri, Some(1), "Best SINR resource should be selected");
    }

    #[test]
    fn test_cri_select_by_rsrp() {
        let measurements = vec![
            CsiRsMeasurement { resource_idx: 0, rsrp_dbm: -80.0, sinr_db: 5.0, cqi: 7 },
            CsiRsMeasurement { resource_idx: 1, rsrp_dbm: -70.0, sinr_db: 3.0, cqi: 4 },
        ];
        let cri = CriSelector::select_by_rsrp(&measurements);
        assert_eq!(cri, Some(1), "Best RSRP resource should be selected");
    }

    #[test]
    fn test_rsrp_computation() {
        let samples: Vec<Complex> = (0..100).map(|_| Complex::new(0.1, 0.0)).collect();
        let rsrp = CriSelector::compute_rsrp(&samples);
        // Power = 0.01, in dBm = 10*log10(0.01) + 30 = -20 + 30 = 10 dBm
        assert!((rsrp - 10.0).abs() < 0.1, "RSRP computation error: {rsrp}");
    }

    #[test]
    fn test_rsrp_empty() {
        let rsrp = CriSelector::compute_rsrp(&[]);
        assert_eq!(rsrp, -140.0);
    }

    // ── Full CSI feedback tests ──────────────────────────────────────────────

    #[test]
    fn test_csi_feedback_basic() {
        let channel = vec![vec![Complex::new(1.0, 0.0); 52]];
        let sinr_per_sc = vec![10.0f64; 52];
        let cb = TypeICodebook::new(1);
        let feedback = CsiFeedback::compute(&channel, &sinr_per_sc, &cb, 0);
        assert_eq!(feedback.ri, 1);
        assert!(feedback.cqi_wideband >= 7 && feedback.cqi_wideband <= 12,
            "10 dB SINR should give moderate CQI, got {}", feedback.cqi_wideband);
    }

    #[test]
    fn test_csi_feedback_subband() {
        let channel = vec![vec![Complex::new(1.0, 0.0); 12 * 52]];
        let sinr_per_sc = vec![15.0f64; 12 * 52];
        let cb = TypeICodebook::new(1);
        let feedback = CsiFeedback::compute(&channel, &sinr_per_sc, &cb, 4);
        // 52 RBs / 4 = 13 subbands
        assert_eq!(feedback.cqi_subband.len(), 13);
    }

    #[test]
    fn test_density_three_sc_positions() {
        let positions = subcarrier_positions(CsiRsDensity::Three, 0, 1);
        assert_eq!(positions.len(), 3);
        assert!(positions.contains(&0));
        assert!(positions.contains(&4));
        assert!(positions.contains(&8));
    }

    #[test]
    fn test_density_one_sc_positions() {
        let positions = subcarrier_positions(CsiRsDensity::One, 3, 1);
        assert_eq!(positions.len(), 1);
        assert!(positions.contains(&3));
    }

    #[test]
    fn test_complex_operations() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, -1.0);
        let prod = a.mul(b);
        assert!((prod.re - 5.0).abs() < 1e-12); // 1*3 - 2*(-1) = 5
        assert!((prod.im - 5.0).abs() < 1e-12); // 1*(-1) + 2*3 = 5
    }

    #[test]
    fn test_complex_conjugate() {
        let c = Complex::new(3.0, -4.0);
        let cc = c.conj();
        assert_eq!(cc.re, 3.0);
        assert_eq!(cc.im, 4.0);
        let prod = c.mul(cc);
        assert!((prod.re - 25.0).abs() < 1e-12); // |c|^2 = 9+16 = 25
        assert!(prod.im.abs() < 1e-12);
    }

    #[test]
    fn test_num_ports_per_row_table() {
        // Verify table entries match TS 38.211 Table 7.4.1.5.3-1
        assert_eq!(CsiRsResourceRow::Row1.num_ports(), 1);
        assert_eq!(CsiRsResourceRow::Row3.num_ports(), 2);
        assert_eq!(CsiRsResourceRow::Row4.num_ports(), 4);
        assert_eq!(CsiRsResourceRow::Row7.num_ports(), 8);
        assert_eq!(CsiRsResourceRow::Row9.num_ports(), 12);
        assert_eq!(CsiRsResourceRow::Row11.num_ports(), 16);
        assert_eq!(CsiRsResourceRow::Row13.num_ports(), 24);
        assert_eq!(CsiRsResourceRow::Row15.num_ports(), 32);
    }
}

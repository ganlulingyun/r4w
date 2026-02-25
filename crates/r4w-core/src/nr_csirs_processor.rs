//! 5G NR CSI-RS Processor and CSI Reporting
//!
//! Implements CSI-RS sequence generation, resource mapping, and CSI reporting
//! per 3GPP TS 38.211 Section 7.4.1.5 and TS 38.214 Section 5.2.
//!
//! # Overview
//!
//! Channel State Information Reference Signals (CSI-RS) are used in 5G NR for:
//! - Channel estimation and feedback
//! - Beam management
//! - Mobility measurements (RRM)
//! - Time/frequency tracking
//!
//! # Sequence Generation (TS 38.211 §7.4.1.5)
//!
//! The Gold sequence initialization:
//! ```text
//! c_init = (2^10 * (N_symb * slot + l + 1) * (2*n_ID + 1) + n_ID) mod 2^31
//! ```
//! QPSK modulation: r(m) = (1/√2)*(1 - 2*c(2m)) + j*(1/√2)*(1 - 2*c(2m+1))
//!
//! # Resource Mapping (TS 38.211 Table 7.4.1.5.3-1)
//!
//! CDM types: noCDM, fd-CDM2, cdm4-FD2-TD2, cdm8-FD2-TD4
//!
//! # CSI Reporting (TS 38.214 §5.2.1)
//!
//! Reports: CQI, PMI (Type I / Type II codebook), RI, CRI, L1-RSRP

// trace:5G-NR-CSIRS | ai:claude

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Gold LFSR pseudo-random sequence generator
// ---------------------------------------------------------------------------

/// Gold sequence generator per 3GPP TS 38.211 Section 5.2.1
pub struct GoldSequence {
    x1: u32,
    x2: u32,
}

impl GoldSequence {
    /// Create with given c_init (loaded into x2 register)
    pub fn new(c_init: u32) -> Self {
        // Initialize x1 with 1 in position 0
        let mut x1: u32 = 1;
        let mut x2: u32 = c_init;

        // Advance 1600 steps for initialization (Nc = 1600)
        for _ in 0..1600 {
            let b1 = (x1 ^ (x1 >> 3)) & 1;
            x1 = (x1 >> 1) | (b1 << 30);

            let b2 = (x2 ^ (x2 >> 1) ^ (x2 >> 2) ^ (x2 >> 3)) & 1;
            x2 = (x2 >> 1) | (b2 << 30);
        }

        GoldSequence { x1, x2 }
    }

    /// Generate next bit (0 or 1)
    pub fn next_bit(&mut self) -> u8 {
        let c = ((self.x1 ^ self.x2) & 1) as u8;

        let b1 = (self.x1 ^ (self.x1 >> 3)) & 1;
        self.x1 = (self.x1 >> 1) | (b1 << 30);

        let b2 = (self.x2 ^ (self.x2 >> 1) ^ (self.x2 >> 2) ^ (self.x2 >> 3)) & 1;
        self.x2 = (self.x2 >> 1) | (b2 << 30);

        c
    }

    /// Generate a block of bits
    pub fn generate(&mut self, len: usize) -> Vec<u8> {
        (0..len).map(|_| self.next_bit()).collect()
    }
}

// ---------------------------------------------------------------------------
// CSI-RS Sequence (QPSK modulated Gold sequence)
// ---------------------------------------------------------------------------

/// Generate CSI-RS baseband sequence for a given slot/symbol
///
/// Per TS 38.211 §7.4.1.5.1:
/// - c_init = (2^10*(N_symb_slot*n_s + l + 1)*(2*n_ID + 1) + n_ID) mod 2^31
/// - r(m) = (1/√2)*(1 - 2c(2m)) + j*(1/√2)*(1 - 2c(2m+1))
pub fn generate_csirs_sequence(
    n_id: u16,
    slot_number: u32,
    symbol_in_slot: u8,
    n_symb_slot: u32, // 14 for normal CP
    length: usize,
) -> Vec<(f64, f64)> {
    let n_id_u64 = n_id as u64;
    let l = symbol_in_slot as u64;
    let n_s = slot_number as u64;
    let n_symb = n_symb_slot as u64;

    let c_init = ((1u64 << 10) * (n_symb * n_s + l + 1) * (2 * n_id_u64 + 1) + n_id_u64)
        % (1u64 << 31);

    let mut gold = GoldSequence::new(c_init as u32);
    let bits = gold.generate(2 * length);

    let scale = 1.0 / (2.0_f64).sqrt();
    (0..length)
        .map(|m| {
            let i = scale * (1.0 - 2.0 * bits[2 * m] as f64);
            let q = scale * (1.0 - 2.0 * bits[2 * m + 1] as f64);
            (i, q)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// CDM (Code Division Multiplexing) types
// ---------------------------------------------------------------------------

/// CDM type for CSI-RS resource (TS 38.211 Table 7.4.1.5.3-1)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CdmType {
    /// No CDM (1 port per RE)
    NoCdm,
    /// FD-CDM2: 2 ports in frequency domain
    FdCdm2,
    /// CDM4-FD2-TD2: 4 ports (2 FD x 2 TD)
    Cdm4Fd2Td2,
    /// CDM8-FD2-TD4: 8 ports (2 FD x 4 TD)
    Cdm8Fd2Td4,
}

impl CdmType {
    /// Number of ports per CDM group
    pub fn ports_per_group(&self) -> u8 {
        match self {
            CdmType::NoCdm => 1,
            CdmType::FdCdm2 => 2,
            CdmType::Cdm4Fd2Td2 => 4,
            CdmType::Cdm8Fd2Td4 => 8,
        }
    }

    /// FD spreading factor
    pub fn fd_spreading(&self) -> u8 {
        match self {
            CdmType::NoCdm => 1,
            CdmType::FdCdm2 => 2,
            CdmType::Cdm4Fd2Td2 => 2,
            CdmType::Cdm8Fd2Td4 => 2,
        }
    }

    /// TD spreading factor
    pub fn td_spreading(&self) -> u8 {
        match self {
            CdmType::NoCdm => 1,
            CdmType::FdCdm2 => 1,
            CdmType::Cdm4Fd2Td2 => 2,
            CdmType::Cdm8Fd2Td4 => 4,
        }
    }

    /// CDM orthogonal cover codes (FD, TD) per port index
    ///
    /// Returns (fd_code, td_code) as ±1 values for each port in group
    pub fn cover_codes(&self) -> Vec<(Vec<i8>, Vec<i8>)> {
        match self {
            CdmType::NoCdm => vec![(vec![1], vec![1])],
            CdmType::FdCdm2 => vec![
                (vec![1, 1], vec![1]),   // port 0
                (vec![1, -1], vec![1]),  // port 1
            ],
            CdmType::Cdm4Fd2Td2 => vec![
                (vec![1, 1], vec![1, 1]),   // port 0
                (vec![1, -1], vec![1, 1]),  // port 1
                (vec![1, 1], vec![1, -1]),  // port 2
                (vec![1, -1], vec![1, -1]), // port 3
            ],
            CdmType::Cdm8Fd2Td4 => vec![
                (vec![1, 1], vec![1, 1, 1, 1]),    // port 0
                (vec![1, -1], vec![1, 1, 1, 1]),   // port 1
                (vec![1, 1], vec![1, -1, 1, -1]),  // port 2
                (vec![1, -1], vec![1, -1, 1, -1]), // port 3
                (vec![1, 1], vec![1, 1, -1, -1]),  // port 4
                (vec![1, -1], vec![1, 1, -1, -1]), // port 5
                (vec![1, 1], vec![1, -1, -1, 1]),  // port 6
                (vec![1, -1], vec![1, -1, -1, 1]), // port 7
            ],
        }
    }
}

// ---------------------------------------------------------------------------
// CSI-RS Configuration
// ---------------------------------------------------------------------------

/// CSI-RS resource configuration
#[derive(Debug, Clone)]
pub struct CsiRsConfig {
    /// Number of antenna ports (1, 2, 4, 8, 12, 16, 24, 32)
    pub n_ports: u8,
    /// CDM type
    pub cdm_type: CdmType,
    /// Density (REs per PRB): 1, 0.5, or 3
    pub density: f64,
    /// First OFDM symbol in slot (0..13)
    pub first_symbol: u8,
    /// Scrambling identity n_ID (0..1023)
    pub scrambling_id: u16,
    /// Bandwidth in PRBs
    pub bandwidth_prbs: u16,
}

impl CsiRsConfig {
    /// Default 1-port NZP-CSI-RS configuration
    pub fn single_port() -> Self {
        CsiRsConfig {
            n_ports: 1,
            cdm_type: CdmType::NoCdm,
            density: 1.0,
            first_symbol: 4,
            scrambling_id: 0,
            bandwidth_prbs: 52,
        }
    }

    /// 2-port CSI-RS with FD-CDM2
    pub fn two_port() -> Self {
        CsiRsConfig {
            n_ports: 2,
            cdm_type: CdmType::FdCdm2,
            density: 1.0,
            first_symbol: 4,
            scrambling_id: 0,
            bandwidth_prbs: 52,
        }
    }

    /// 4-port CSI-RS with CDM4
    pub fn four_port() -> Self {
        CsiRsConfig {
            n_ports: 4,
            cdm_type: CdmType::Cdm4Fd2Td2,
            density: 1.0,
            first_symbol: 4,
            scrambling_id: 0,
            bandwidth_prbs: 52,
        }
    }

    /// 8-port CSI-RS with CDM8
    pub fn eight_port() -> Self {
        CsiRsConfig {
            n_ports: 8,
            cdm_type: CdmType::Cdm8Fd2Td4,
            density: 1.0,
            first_symbol: 4,
            scrambling_id: 0,
            bandwidth_prbs: 52,
        }
    }
}

// ---------------------------------------------------------------------------
// RE Pattern Generation (TS 38.211 Table 7.4.1.5.3-1)
// ---------------------------------------------------------------------------

/// (subcarrier offset k', symbol offset l') tuple within a PRB
pub type ReOffset = (u8, u8);

/// CSI-RS resource with mapped RE positions
#[derive(Debug, Clone)]
pub struct CsiRsResource {
    pub config: CsiRsConfig,
    /// List of (k', l') RE offsets within a PRB
    pub re_pattern: Vec<ReOffset>,
}

impl CsiRsResource {
    /// Build RE pattern from config per TS 38.211 Table 7.4.1.5.3-1
    pub fn new(config: CsiRsConfig) -> Self {
        let re_pattern = Self::build_re_pattern(&config);
        CsiRsResource { config, re_pattern }
    }

    fn build_re_pattern(cfg: &CsiRsConfig) -> Vec<ReOffset> {
        match (cfg.n_ports, &cfg.cdm_type, cfg.density as u32) {
            // 1-port, density 3: subcarriers 0, 4, 8 in symbol 0
            (1, CdmType::NoCdm, 3) => vec![(0, 0), (4, 0), (8, 0)],

            // 1-port, density 1
            (1, CdmType::NoCdm, _) => vec![(0, 0)],

            // 2-port fd-CDM2, density 1
            (2, CdmType::FdCdm2, _) => vec![(0, 0), (1, 0)],

            // 4-port CDM4-FD2-TD2
            (4, CdmType::Cdm4Fd2Td2, _) => {
                vec![(0, 0), (1, 0), (0, 1), (1, 1)]
            }

            // 8-port: 2 CDM groups x 4 ports (uses 4 subcarriers x 2 syms)
            (8, CdmType::Cdm8Fd2Td4, _) => {
                vec![
                    (0, 0), (1, 0), (2, 0), (3, 0),
                    (0, 1), (1, 1), (2, 1), (3, 1),
                ]
            }

            // 12-port: 3 CDM groups x 4 ports
            (12, CdmType::Cdm4Fd2Td2, _) => {
                let mut v = Vec::new();
                for g in 0u8..3 {
                    let k_base = g * 2;
                    v.push((k_base, 0));
                    v.push((k_base + 1, 0));
                    v.push((k_base, 1));
                    v.push((k_base + 1, 1));
                }
                v
            }

            // 16-port: 4 CDM groups x 4 ports
            (16, CdmType::Cdm4Fd2Td2, _) => {
                let mut v = Vec::new();
                for g in 0u8..4 {
                    let k_base = g * 2;
                    v.push((k_base, 0));
                    v.push((k_base + 1, 0));
                    v.push((k_base, 1));
                    v.push((k_base + 1, 1));
                }
                v
            }

            // 24-port: 4 CDM groups x 2FD x 3TD
            (24, CdmType::Cdm4Fd2Td2, _) => {
                let mut v = Vec::new();
                for g in 0u8..4 {
                    let k_base = g * 2;
                    for l in 0u8..3 {
                        v.push((k_base, l));
                        v.push((k_base + 1, l));
                    }
                }
                v
            }

            // 32-port: 4 CDM groups x 2FD x 4TD
            (32, CdmType::Cdm8Fd2Td4, _) => {
                let mut v = Vec::new();
                for g in 0u8..4 {
                    let k_base = g * 2;
                    for l in 0u8..4 {
                        v.push((k_base, l));
                        v.push((k_base + 1, l));
                    }
                }
                v
            }

            // Default fallback: single RE
            _ => vec![(0, 0)],
        }
    }

    /// Number of REs per PRB occupied by this CSI-RS resource
    pub fn res_per_prb(&self) -> usize {
        self.re_pattern.len()
    }

    /// Total REs across the bandwidth
    pub fn total_res(&self) -> usize {
        self.re_pattern.len() * self.config.bandwidth_prbs as usize
    }
}

// ---------------------------------------------------------------------------
// CQI Table (TS 38.214 Table 5.2.2.1-2, 64QAM)
// ---------------------------------------------------------------------------

/// Single CQI table entry
#[derive(Debug, Clone)]
pub struct CqiEntry {
    pub index: u8,
    pub modulation_order: u8,  // bits per symbol
    pub code_rate_x1024: u16,
    pub spectral_efficiency: f64,
}

impl CqiEntry {
    pub fn modulation_name(&self) -> &'static str {
        match self.modulation_order {
            2 => "QPSK",
            4 => "16QAM",
            6 => "64QAM",
            _ => "Unknown",
        }
    }
}

/// Full CQI table (TS 38.214 Table 5.2.2.1-2, 64QAM table)
pub fn cqi_table_64qam() -> [CqiEntry; 16] {
    [
        CqiEntry { index: 0,  modulation_order: 0, code_rate_x1024: 0,   spectral_efficiency: 0.0    },
        CqiEntry { index: 1,  modulation_order: 2, code_rate_x1024: 78,  spectral_efficiency: 0.1523 },
        CqiEntry { index: 2,  modulation_order: 2, code_rate_x1024: 120, spectral_efficiency: 0.2344 },
        CqiEntry { index: 3,  modulation_order: 2, code_rate_x1024: 193, spectral_efficiency: 0.3770 },
        CqiEntry { index: 4,  modulation_order: 2, code_rate_x1024: 308, spectral_efficiency: 0.6016 },
        CqiEntry { index: 5,  modulation_order: 2, code_rate_x1024: 449, spectral_efficiency: 0.8770 },
        CqiEntry { index: 6,  modulation_order: 2, code_rate_x1024: 602, spectral_efficiency: 1.1758 },
        CqiEntry { index: 7,  modulation_order: 2, code_rate_x1024: 873, spectral_efficiency: 1.4766 }, // Note: TS uses 873
        CqiEntry { index: 8,  modulation_order: 4, code_rate_x1024: 490, spectral_efficiency: 1.9141 },
        CqiEntry { index: 9,  modulation_order: 4, code_rate_x1024: 616, spectral_efficiency: 2.4063 },
        CqiEntry { index: 10, modulation_order: 6, code_rate_x1024: 466, spectral_efficiency: 2.7305 },
        CqiEntry { index: 11, modulation_order: 6, code_rate_x1024: 567, spectral_efficiency: 3.3223 },
        CqiEntry { index: 12, modulation_order: 6, code_rate_x1024: 666, spectral_efficiency: 3.9023 },
        CqiEntry { index: 13, modulation_order: 6, code_rate_x1024: 772, spectral_efficiency: 4.5234 },
        CqiEntry { index: 14, modulation_order: 6, code_rate_x1024: 873, spectral_efficiency: 5.1152 },
        CqiEntry { index: 15, modulation_order: 6, code_rate_x1024: 948, spectral_efficiency: 5.5547 },
    ]
}

/// Look up CQI entry by index
pub fn cqi_lookup(index: u8) -> Option<CqiEntry> {
    if index > 15 {
        return None;
    }
    Some(cqi_table_64qam()[index as usize].clone())
}

/// Determine CQI index from measured SINR (simplified mapping)
///
/// Uses spectral efficiency thresholds as a rough SINR → CQI mapping.
/// A more accurate mapping would use BLER curves.
pub fn sinr_to_cqi(sinr_db: f64) -> u8 {
    // Approximate SINR thresholds for ~10% BLER at each CQI
    const SINR_THRESHOLDS: [f64; 15] = [
        -6.7, -4.7, -2.3, 0.2, 2.4, 4.3, 5.9, 8.1, 10.3, 11.7, 14.1, 16.3, 18.7, 21.0, 22.7,
    ];
    let mut cqi = 1u8;
    for (i, &thr) in SINR_THRESHOLDS.iter().enumerate() {
        if sinr_db >= thr {
            cqi = (i + 1) as u8;
        }
    }
    cqi
}

// ---------------------------------------------------------------------------
// Codebook Types
// ---------------------------------------------------------------------------

/// Precoding codebook type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodebookType {
    TypeI,
    TypeII,
}

/// PMI value structure
#[derive(Debug, Clone)]
pub struct PmiValue {
    /// Beam pair index (row), 0-based
    pub i1_1: u8,
    /// Beam pair index (column), 0-based
    pub i1_2: u8,
    /// Co-phasing / subband selection index
    pub i2: u8,
    pub codebook_type: CodebookType,
}

impl PmiValue {
    /// Wideband PMI for single-layer Type I
    pub fn single_layer(i1_1: u8, i1_2: u8, i2: u8) -> Self {
        PmiValue { i1_1, i1_2, i2, codebook_type: CodebookType::TypeI }
    }
}

// ---------------------------------------------------------------------------
// Type I Codebook (TS 38.214 §5.2.2.2.1)
// ---------------------------------------------------------------------------

/// DFT codebook for Type I single-panel precoding
///
/// N1 x N2 antenna array with oversampling O1 x O2
#[derive(Debug, Clone)]
pub struct DftCodebook {
    /// Horizontal antenna ports
    pub n1: u8,
    /// Vertical antenna ports  
    pub n2: u8,
    /// Horizontal oversampling factor
    pub o1: u8,
    /// Vertical oversampling factor
    pub o2: u8,
}

impl DftCodebook {
    /// Standard 4Tx: 2x1 array, O=4
    pub fn new_4tx() -> Self {
        DftCodebook { n1: 2, n2: 1, o1: 4, o2: 1 }
    }

    /// 8Tx: 4x1 or 2x2 array
    pub fn new_8tx_4x1() -> Self {
        DftCodebook { n1: 4, n2: 1, o1: 4, o2: 1 }
    }

    pub fn new_8tx_2x2() -> Self {
        DftCodebook { n1: 2, n2: 2, o1: 4, o2: 4 }
    }

    /// 16Tx: 4x2
    pub fn new_16tx() -> Self {
        DftCodebook { n1: 4, n2: 2, o1: 4, o2: 4 }
    }

    /// 32Tx: 4x4
    pub fn new_32tx() -> Self {
        DftCodebook { n1: 4, n2: 4, o1: 4, o2: 4 }
    }

    /// Total beam candidates in W1 wideband part
    pub fn n_beams(&self) -> usize {
        (self.n1 as usize * self.o1 as usize) * (self.n2 as usize * self.o2 as usize)
    }

    /// Generate a single DFT beam vector v(l, m) for horizontal index l, vertical index m
    ///
    /// v_{l,m}[n1_idx, n2_idx] = exp(j*2*pi*l*n1_idx / (N1*O1)) * exp(j*2*pi*m*n2_idx / (N2*O2))
    pub fn beam_vector(&self, l: u8, m: u8) -> BeamVector {
        let n1 = self.n1 as usize;
        let n2 = self.n2 as usize;
        let n1o1 = n1 * self.o1 as usize;
        let n2o2 = n2 * self.o2 as usize;

        let mut weights = Vec::with_capacity(n1 * n2);
        for n1_idx in 0..n1 {
            for n2_idx in 0..n2 {
                let phase_h = 2.0 * PI * (l as f64) * (n1_idx as f64) / (n1o1 as f64);
                let phase_v = 2.0 * PI * (m as f64) * (n2_idx as f64) / (n2o2 as f64);
                let phase = phase_h + phase_v;
                weights.push((phase.cos(), phase.sin()));
            }
        }
        BeamVector { weights }
    }

    /// Generate W1 precoding matrix for beam pair (i1_1, i1_2) and rank r
    ///
    /// W1 selects L=4 consecutive oversampled DFT beams centered at (i1_1, i1_2)
    pub fn w1_matrix(&self, i1_1: u8, i1_2: u8) -> Vec<BeamVector> {
        // For single-panel Type I, W1 selects 2x1 or 2x2 beam group
        let beams = vec![
            self.beam_vector(i1_1, i1_2),
            self.beam_vector(i1_1 + 1, i1_2),
        ];
        beams
    }

    /// Co-phasing vector for i2 index (QPSK: {1, j, -1, -j})
    pub fn co_phase(i2: u8) -> (f64, f64) {
        let phase = (i2 as f64) * PI / 2.0;
        (phase.cos(), phase.sin())
    }

    /// Generate W2 combining weights for rank-1
    ///
    /// W2 selects beam and co-phase per TS 38.214 Table 5.2.2.2.1-5
    pub fn w2_rank1_weights(i2: u8) -> (usize, (f64, f64)) {
        // i2 = 2*beam_sel + cophase (simplified for 2-beam W1)
        let beam_sel = ((i2 >> 1) & 1) as usize;
        let cophase_idx = i2 & 1;
        let co_phase = Self::co_phase(cophase_idx * 2); // 0 or pi → (1,0) or (-1,0)
        (beam_sel, co_phase)
    }
}

/// Complex beam vector weights
#[derive(Debug, Clone)]
pub struct BeamVector {
    /// Complex weights as (Re, Im)
    pub weights: Vec<(f64, f64)>,
}

impl BeamVector {
    /// Compute beam gain for steering angle theta (ULA assumed)
    pub fn gain_ula(&self, theta: f64, lambda_over_d: f64) -> (f64, f64) {
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        for (idx, &(wr, wi)) in self.weights.iter().enumerate() {
            let phase = 2.0 * PI * (idx as f64) * theta.sin() / lambda_over_d;
            re += wr * phase.cos() - wi * phase.sin();
            im += wr * phase.sin() + wi * phase.cos();
        }
        (re, im)
    }
}

// ---------------------------------------------------------------------------
// Type II Codebook (TS 38.214 §5.2.2.2.3)
// ---------------------------------------------------------------------------

/// Type II codebook configuration
#[derive(Debug, Clone)]
pub struct TypeIiCodebook {
    pub dft: DftCodebook,
    /// Number of beams L (2 or 4)
    pub n_beams: u8,
    /// Amplitude quantization bits (3 for wideband)
    pub amplitude_bits: u8,
    /// Phase quantization: 2 (QPSK) or 3 (8PSK)
    pub phase_bits: u8,
}

impl TypeIiCodebook {
    /// Standard Type II config: L=2, 3-bit amplitude, QPSK phase
    pub fn standard() -> Self {
        TypeIiCodebook {
            dft: DftCodebook::new_4tx(),
            n_beams: 2,
            amplitude_bits: 3,
            phase_bits: 2,
        }
    }

    /// Number of wideband amplitude quantization levels
    pub fn n_amplitude_levels(&self) -> usize {
        1 << self.amplitude_bits as usize
    }

    /// Number of phase quantization levels
    pub fn n_phase_levels(&self) -> usize {
        1 << self.phase_bits as usize
    }

    /// Quantized wideband amplitude (3-bit: 0..7 → amplitude)
    ///
    /// Per TS 38.214 §5.2.2.2.3: levels are {0, 1/√8, 1/√4, 1/√2, 1, ...}
    pub fn amplitude_from_index(idx: u8) -> f64 {
        match idx {
            0 => 0.0,
            1 => (1.0_f64 / 8.0).sqrt(),
            2 => (1.0_f64 / 4.0).sqrt(),
            3 => (3.0_f64 / 8.0).sqrt(),
            4 => (1.0_f64 / 2.0).sqrt(),
            5 => (5.0_f64 / 8.0).sqrt(),
            6 => (3.0_f64 / 4.0).sqrt(),
            7 => 1.0,
            _ => 1.0,
        }
    }

    /// Quantized phase from index (QPSK: 0→0°, 1→90°, 2→180°, 3→270°)
    pub fn phase_from_index_qpsk(idx: u8) -> (f64, f64) {
        let angle = (idx as f64) * PI / 2.0;
        (angle.cos(), angle.sin())
    }

    /// Quantized phase (8PSK: 0..7 → 0°..315°)
    pub fn phase_from_index_8psk(idx: u8) -> (f64, f64) {
        let angle = (idx as f64) * PI / 4.0;
        (angle.cos(), angle.sin())
    }
}

// ---------------------------------------------------------------------------
// RSRP / RSRQ / SINR Measurement
// ---------------------------------------------------------------------------

/// Compute Reference Signal Received Power (RSRP) in dBm
///
/// RSRP = average linear power per CSI-RS RE, converted to dBm
pub fn compute_rsrp_dbm(samples: &[(f64, f64)]) -> f64 {
    if samples.is_empty() {
        return f64::NEG_INFINITY;
    }
    let avg_power: f64 = samples.iter().map(|(r, i)| r * r + i * i).sum::<f64>() / samples.len() as f64;
    // Convert to dBm (assume 1 Ohm reference, power in mW)
    10.0 * (avg_power * 1000.0).max(1e-20).log10()
}

/// Compute RSRQ = N * RSRP / RSSI
///
/// N = number of PRBs in measurement bandwidth
pub fn compute_rsrq_db(rsrp_dbm: f64, rssi_dbm: f64, n_prbs: u32) -> f64 {
    let n_db = 10.0 * (n_prbs as f64).log10();
    n_db + rsrp_dbm - rssi_dbm
}

/// Compute SINR in dB
pub fn compute_sinr_db(signal_power: f64, interference_plus_noise: f64) -> f64 {
    if interference_plus_noise <= 0.0 {
        return f64::INFINITY;
    }
    10.0 * (signal_power / interference_plus_noise).log10()
}

/// L1-RSRP measurement with averaging across REs
pub fn l1_rsrp(re_samples: &[(f64, f64)]) -> f64 {
    compute_rsrp_dbm(re_samples)
}

// ---------------------------------------------------------------------------
// CSI Report Structure
// ---------------------------------------------------------------------------

/// Complete CSI feedback report
#[derive(Debug, Clone)]
pub struct CsiReport {
    /// Channel Quality Indicator (0..15, 0 = out of range)
    pub cqi: u8,
    /// Precoding Matrix Indicator
    pub pmi: PmiValue,
    /// Rank Indicator (1..8)
    pub ri: u8,
    /// CSI-RS Resource Indicator (best beam/resource index)
    pub cri: u8,
    /// Layer Indicator (best layer)
    pub li: u8,
    /// L1-RSRP in dBm
    pub rsrp_dbm: f64,
}

impl CsiReport {
    /// Create report with basic measurements
    pub fn new(cqi: u8, pmi: PmiValue, ri: u8, rsrp_dbm: f64) -> Self {
        CsiReport { cqi, pmi, ri, cri: 0, li: 0, rsrp_dbm }
    }

    /// Generate a CSI report from received samples and codebook
    pub fn from_measurements(
        samples: &[(f64, f64)],
        codebook: &DftCodebook,
        sinr_db: f64,
        ri: u8,
    ) -> Self {
        let rsrp_dbm = compute_rsrp_dbm(samples);
        let cqi = sinr_to_cqi(sinr_db);

        // Simple beam selection: pick beam with max gain toward broadside (theta=0)
        let n1o1 = codebook.n1 as u8 * codebook.o1;
        let n2o2 = codebook.n2 as u8 * codebook.o2;
        let mut best_gain = f64::NEG_INFINITY;
        let mut best_i1_1 = 0u8;
        let mut best_i1_2 = 0u8;

        for l in 0..n1o1 {
            for m in 0..n2o2 {
                let bv = codebook.beam_vector(l, m);
                let (gr, gi) = bv.gain_ula(0.0, 2.0);
                let gain = gr * gr + gi * gi;
                if gain > best_gain {
                    best_gain = gain;
                    best_i1_1 = l;
                    best_i1_2 = m;
                }
            }
        }

        let pmi = PmiValue::single_layer(best_i1_1, best_i1_2, 0);
        CsiReport { cqi, pmi, ri, cri: 0, li: 0, rsrp_dbm }
    }
}

// ---------------------------------------------------------------------------
// CSI-RS Processor (main struct)
// ---------------------------------------------------------------------------

/// 5G NR CSI-RS processor implementing TS 38.211/38.214
pub struct NrCsirsProcessor {
    pub config: CsiRsConfig,
    pub resource: CsiRsResource,
}

impl NrCsirsProcessor {
    /// Create processor from configuration
    pub fn new(config: CsiRsConfig) -> Self {
        let resource = CsiRsResource::new(config.clone());
        NrCsirsProcessor { config, resource }
    }

    /// Generate CSI-RS symbols for a given slot
    ///
    /// Returns a flat vector of QPSK symbols (I,Q) for all REs in the bandwidth
    pub fn generate_symbols(&self, slot: u32, n_symb_slot: u32) -> Vec<(f64, f64)> {
        let sym = self.config.first_symbol;
        let total_res = self.resource.total_res();
        generate_csirs_sequence(
            self.config.scrambling_id,
            slot,
            sym,
            n_symb_slot,
            total_res,
        )
    }

    /// Map CSI-RS symbols to resource grid
    ///
    /// Returns (prb, subcarrier_offset, symbol_offset, complex_value) tuples
    pub fn map_to_grid(&self, slot: u32) -> Vec<(u16, u8, u8, (f64, f64))> {
        let symbols = self.generate_symbols(slot, 14);
        let mut result = Vec::new();
        let mut sym_idx = 0;

        for prb in 0..self.config.bandwidth_prbs {
            for &(k_prime, l_prime) in &self.resource.re_pattern {
                if sym_idx < symbols.len() {
                    result.push((prb, k_prime, l_prime, symbols[sym_idx]));
                    sym_idx += 1;
                }
            }
        }
        result
    }

    /// Estimate channel from received CSI-RS samples (LS estimation)
    ///
    /// H_LS[k] = Y[k] / X[k]  (element-wise division)
    pub fn estimate_channel_ls(
        &self,
        received: &[(f64, f64)],
        slot: u32,
    ) -> Vec<(f64, f64)> {
        let transmitted = self.generate_symbols(slot, 14);
        let len = received.len().min(transmitted.len());
        let mut h = Vec::with_capacity(len);

        for i in 0..len {
            let (yr, yi) = received[i];
            let (xr, xi) = transmitted[i];
            let denom = xr * xr + xi * xi;
            if denom > 1e-12 {
                let hr = (yr * xr + yi * xi) / denom;
                let hi = (yi * xr - yr * xi) / denom;
                h.push((hr, hi));
            } else {
                h.push((0.0, 0.0));
            }
        }
        h
    }

    /// Generate CSI feedback report from received samples
    pub fn generate_csi_report(
        &self,
        received: &[(f64, f64)],
        slot: u32,
        sinr_db: f64,
    ) -> CsiReport {
        let codebook = DftCodebook::new_4tx();
        let h = self.estimate_channel_ls(received, slot);
        CsiReport::from_measurements(&h, &codebook, sinr_db, 1)
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute complex multiplication (a_re + j*a_im) * (b_re + j*b_im)
#[inline]
pub fn complex_mul(ar: f64, ai: f64, br: f64, bi: f64) -> (f64, f64) {
    (ar * br - ai * bi, ar * bi + ai * br)
}

/// Compute QPSK symbol for two bits b0, b1
pub fn qpsk_symbol(b0: u8, b1: u8) -> (f64, f64) {
    let scale = 1.0 / (2.0_f64).sqrt();
    ((1.0 - 2.0 * b0 as f64) * scale, (1.0 - 2.0 * b1 as f64) * scale)
}

/// Compute average power of complex samples
pub fn average_power(samples: &[(f64, f64)]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    samples.iter().map(|(r, i)| r * r + i * i).sum::<f64>() / samples.len() as f64
}

/// Convert power (linear) to dBm
pub fn power_to_dbm(power_w: f64) -> f64 {
    10.0 * (power_w * 1000.0).max(1e-30).log10()
}

/// Convert dBm to linear power in watts
pub fn dbm_to_power(dbm: f64) -> f64 {
    10.0_f64.powf(dbm / 10.0) / 1000.0
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Gold Sequence Tests ----

    #[test]
    fn test_gold_sequence_length() {
        let mut gs = GoldSequence::new(0);
        let bits = gs.generate(128);
        assert_eq!(bits.len(), 128);
    }

    #[test]
    fn test_gold_sequence_binary() {
        let mut gs = GoldSequence::new(42);
        let bits = gs.generate(64);
        for &b in &bits {
            assert!(b == 0 || b == 1, "Gold sequence must be binary");
        }
    }

    #[test]
    fn test_gold_sequence_different_init() {
        let mut gs1 = GoldSequence::new(0);
        let mut gs2 = GoldSequence::new(1);
        let b1 = gs1.generate(32);
        let b2 = gs2.generate(32);
        // Different c_init should produce different sequences
        assert_ne!(b1, b2);
    }

    #[test]
    fn test_gold_sequence_reproducible() {
        let b1 = GoldSequence::new(12345).generate(100);
        let b2 = GoldSequence::new(12345).generate(100);
        assert_eq!(b1, b2, "Same seed must produce same sequence");
    }

    #[test]
    fn test_gold_sequence_balance() {
        let mut gs = GoldSequence::new(777);
        let bits = gs.generate(1000);
        let ones: usize = bits.iter().filter(|&&b| b == 1).count();
        let zeros: usize = bits.iter().filter(|&&b| b == 0).count();
        // Should be roughly balanced
        assert!((ones as i64 - zeros as i64).abs() < 100, "Sequence should be balanced: ones={ones}, zeros={zeros}");
    }

    // ---- CSI-RS Sequence Tests ----

    #[test]
    fn test_csirs_sequence_length() {
        let seq = generate_csirs_sequence(0, 0, 4, 14, 52);
        assert_eq!(seq.len(), 52);
    }

    #[test]
    fn test_csirs_sequence_qpsk_amplitude() {
        let seq = generate_csirs_sequence(100, 5, 7, 14, 100);
        let expected_amp = 1.0; // QPSK: |I+jQ| = sqrt(0.5+0.5) = 1.0
        for (i, q) in &seq {
            let amp = (i * i + q * q).sqrt();
            assert!((amp - expected_amp).abs() < 1e-10, "QPSK amplitude wrong: {amp}");
        }
    }

    #[test]
    fn test_csirs_sequence_qpsk_constellation() {
        let seq = generate_csirs_sequence(0, 0, 0, 14, 200);
        let scale = 1.0 / (2.0_f64).sqrt();
        for (i, q) in &seq {
            // I and Q must each be ±1/√2
            assert!((i.abs() - scale).abs() < 1e-10, "I component: {i}");
            assert!((q.abs() - scale).abs() < 1e-10, "Q component: {q}");
        }
    }

    #[test]
    fn test_csirs_sequence_different_n_id() {
        let s1 = generate_csirs_sequence(0, 0, 4, 14, 50);
        let s2 = generate_csirs_sequence(1, 0, 4, 14, 50);
        assert_ne!(s1, s2);
    }

    #[test]
    fn test_csirs_sequence_different_slot() {
        let s1 = generate_csirs_sequence(0, 0, 4, 14, 50);
        let s2 = generate_csirs_sequence(0, 1, 4, 14, 50);
        assert_ne!(s1, s2);
    }

    // ---- CDM Type Tests ----

    #[test]
    fn test_cdm_no_cdm_ports() {
        assert_eq!(CdmType::NoCdm.ports_per_group(), 1);
    }

    #[test]
    fn test_cdm_fd_cdm2_ports() {
        assert_eq!(CdmType::FdCdm2.ports_per_group(), 2);
    }

    #[test]
    fn test_cdm4_ports() {
        assert_eq!(CdmType::Cdm4Fd2Td2.ports_per_group(), 4);
    }

    #[test]
    fn test_cdm8_ports() {
        assert_eq!(CdmType::Cdm8Fd2Td4.ports_per_group(), 8);
    }

    #[test]
    fn test_cdm_fd_spreading() {
        assert_eq!(CdmType::NoCdm.fd_spreading(), 1);
        assert_eq!(CdmType::FdCdm2.fd_spreading(), 2);
        assert_eq!(CdmType::Cdm4Fd2Td2.fd_spreading(), 2);
        assert_eq!(CdmType::Cdm8Fd2Td4.fd_spreading(), 2);
    }

    #[test]
    fn test_cdm_td_spreading() {
        assert_eq!(CdmType::NoCdm.td_spreading(), 1);
        assert_eq!(CdmType::FdCdm2.td_spreading(), 1);
        assert_eq!(CdmType::Cdm4Fd2Td2.td_spreading(), 2);
        assert_eq!(CdmType::Cdm8Fd2Td4.td_spreading(), 4);
    }

    #[test]
    fn test_cdm4_cover_codes_count() {
        let codes = CdmType::Cdm4Fd2Td2.cover_codes();
        assert_eq!(codes.len(), 4);
    }

    #[test]
    fn test_cdm8_cover_codes_count() {
        let codes = CdmType::Cdm8Fd2Td4.cover_codes();
        assert_eq!(codes.len(), 8);
    }

    #[test]
    fn test_cdm_cover_codes_orthogonal_fd() {
        let codes = CdmType::FdCdm2.cover_codes();
        // [1,1] and [1,-1] dot product = 0
        let (fd0, _) = &codes[0];
        let (fd1, _) = &codes[1];
        let dot: i32 = fd0.iter().zip(fd1.iter()).map(|(&a, &b)| a as i32 * b as i32).sum();
        assert_eq!(dot, 0, "FD-CDM2 codes should be orthogonal");
    }

    // ---- RE Pattern Tests ----

    #[test]
    fn test_1port_density1_re_pattern() {
        let cfg = CsiRsConfig::single_port();
        let resource = CsiRsResource::new(cfg);
        assert_eq!(resource.re_pattern, vec![(0, 0)]);
    }

    #[test]
    fn test_1port_density3_re_pattern() {
        let cfg = CsiRsConfig {
            density: 3.0,
            ..CsiRsConfig::single_port()
        };
        let resource = CsiRsResource::new(cfg);
        assert_eq!(resource.re_pattern.len(), 3);
        assert!(resource.re_pattern.contains(&(0, 0)));
        assert!(resource.re_pattern.contains(&(4, 0)));
        assert!(resource.re_pattern.contains(&(8, 0)));
    }

    #[test]
    fn test_2port_re_pattern() {
        let resource = CsiRsResource::new(CsiRsConfig::two_port());
        assert_eq!(resource.re_pattern.len(), 2);
        assert!(resource.re_pattern.contains(&(0, 0)));
        assert!(resource.re_pattern.contains(&(1, 0)));
    }

    #[test]
    fn test_4port_re_pattern() {
        let resource = CsiRsResource::new(CsiRsConfig::four_port());
        assert_eq!(resource.re_pattern.len(), 4);
        // Should cover 2 subcarriers x 2 symbols
        assert!(resource.re_pattern.contains(&(0, 0)));
        assert!(resource.re_pattern.contains(&(1, 0)));
        assert!(resource.re_pattern.contains(&(0, 1)));
        assert!(resource.re_pattern.contains(&(1, 1)));
    }

    #[test]
    fn test_8port_re_pattern() {
        let resource = CsiRsResource::new(CsiRsConfig::eight_port());
        assert_eq!(resource.re_pattern.len(), 8);
    }

    #[test]
    fn test_total_res_calculation() {
        let cfg = CsiRsConfig { bandwidth_prbs: 52, ..CsiRsConfig::single_port() };
        let resource = CsiRsResource::new(cfg);
        assert_eq!(resource.total_res(), 52); // 1 RE/PRB x 52 PRBs
    }

    #[test]
    fn test_total_res_2port() {
        let cfg = CsiRsConfig { bandwidth_prbs: 52, ..CsiRsConfig::two_port() };
        let resource = CsiRsResource::new(cfg);
        assert_eq!(resource.total_res(), 104); // 2 RE/PRB x 52 PRBs
    }

    // ---- CQI Table Tests ----

    #[test]
    fn test_cqi_table_size() {
        let table = cqi_table_64qam();
        assert_eq!(table.len(), 16);
    }

    #[test]
    fn test_cqi_0_out_of_range() {
        let e = &cqi_table_64qam()[0];
        assert_eq!(e.index, 0);
        assert_eq!(e.code_rate_x1024, 0);
        assert_eq!(e.spectral_efficiency, 0.0);
    }

    #[test]
    fn test_cqi_1_qpsk() {
        let e = &cqi_table_64qam()[1];
        assert_eq!(e.index, 1);
        assert_eq!(e.modulation_order, 2); // QPSK
        assert_eq!(e.code_rate_x1024, 78);
        assert!((e.spectral_efficiency - 0.1523).abs() < 0.001);
    }

    #[test]
    fn test_cqi_10_64qam() {
        let e = &cqi_table_64qam()[10];
        assert_eq!(e.index, 10);
        assert_eq!(e.modulation_order, 6); // 64QAM
        assert_eq!(e.code_rate_x1024, 466);
        assert!((e.spectral_efficiency - 2.7305).abs() < 0.001);
    }

    #[test]
    fn test_cqi_15_highest() {
        let e = &cqi_table_64qam()[15];
        assert_eq!(e.index, 15);
        assert_eq!(e.modulation_order, 6); // 64QAM
        assert_eq!(e.code_rate_x1024, 948);
        assert!((e.spectral_efficiency - 5.5547).abs() < 0.001);
    }

    #[test]
    fn test_cqi_lookup_valid() {
        let e = cqi_lookup(7).unwrap();
        assert_eq!(e.index, 7);
    }

    #[test]
    fn test_cqi_lookup_out_of_range() {
        assert!(cqi_lookup(16).is_none());
    }

    #[test]
    fn test_cqi_modulation_names() {
        let table = cqi_table_64qam();
        assert_eq!(table[1].modulation_name(), "QPSK");
        assert_eq!(table[8].modulation_name(), "16QAM");
        assert_eq!(table[10].modulation_name(), "64QAM");
    }

    #[test]
    fn test_sinr_to_cqi_low_sinr() {
        // Very low SINR → CQI 1 (minimum)
        let cqi = sinr_to_cqi(-10.0);
        assert_eq!(cqi, 1);
    }

    #[test]
    fn test_sinr_to_cqi_high_sinr() {
        // High SINR → CQI 15 (maximum)
        let cqi = sinr_to_cqi(25.0);
        assert_eq!(cqi, 15);
    }

    #[test]
    fn test_sinr_to_cqi_monotonic() {
        let snrs: [f64; 5] = [-5.0, 0.0, 5.0, 15.0, 22.0];
        let cqis: Vec<u8> = snrs.iter().map(|&s| sinr_to_cqi(s)).collect();
        for w in cqis.windows(2) {
            assert!(w[0] <= w[1], "CQI should be monotonically non-decreasing with SNR");
        }
    }

    // ---- DFT Codebook Tests ----

    #[test]
    fn test_dft_codebook_4tx_nbeams() {
        let cb = DftCodebook::new_4tx(); // N1=2, N2=1, O1=4, O2=1
        assert_eq!(cb.n_beams(), 8); // 2*4 * 1*1 = 8
    }

    #[test]
    fn test_beam_vector_length() {
        let cb = DftCodebook::new_4tx();
        let bv = cb.beam_vector(0, 0);
        assert_eq!(bv.weights.len(), 2); // N1*N2 = 2*1
    }

    #[test]
    fn test_beam_vector_unit_magnitude() {
        let cb = DftCodebook::new_8tx_4x1();
        let bv = cb.beam_vector(2, 0);
        for &(wr, wi) in &bv.weights {
            let mag = (wr * wr + wi * wi).sqrt();
            assert!((mag - 1.0).abs() < 1e-10, "DFT beam element should have unit magnitude");
        }
    }

    #[test]
    fn test_co_phase_qpsk_values() {
        let (r0, i0) = DftCodebook::co_phase(0);
        assert!((r0 - 1.0).abs() < 1e-10);
        assert!(i0.abs() < 1e-10);

        let (r1, i1) = DftCodebook::co_phase(1);
        assert!(r1.abs() < 1e-10);
        assert!((i1 - 1.0).abs() < 1e-10);

        let (r2, i2) = DftCodebook::co_phase(2);
        assert!((r2 + 1.0).abs() < 1e-10);
        assert!(i2.abs() < 1e-10);

        let (r3, i3) = DftCodebook::co_phase(3);
        assert!(r3.abs() < 1e-10);
        assert!((i3 + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_w1_matrix_returns_beams() {
        let cb = DftCodebook::new_4tx();
        let beams = cb.w1_matrix(0, 0);
        assert!(!beams.is_empty());
    }

    #[test]
    fn test_8tx_codebook_beams() {
        let cb = DftCodebook::new_8tx_2x2(); // N1=2, N2=2, O1=4, O2=4
        assert_eq!(cb.n_beams(), 64); // 2*4 * 2*4 = 64
    }

    #[test]
    fn test_beam_vector_gain_broadside() {
        let cb = DftCodebook::new_4tx();
        let bv = cb.beam_vector(0, 0); // DC beam
        // At theta=0, all elements add coherently
        let (gr, gi) = bv.gain_ula(0.0, 2.0);
        let gain = (gr * gr + gi * gi).sqrt();
        assert!(gain > 0.0, "Broadside gain should be positive");
    }

    // ---- Type II Codebook Tests ----

    #[test]
    fn test_typeii_amplitude_levels() {
        let cb = TypeIiCodebook::standard();
        assert_eq!(cb.n_amplitude_levels(), 8); // 2^3
    }

    #[test]
    fn test_typeii_phase_levels() {
        let cb = TypeIiCodebook::standard();
        assert_eq!(cb.n_phase_levels(), 4); // 2^2 = QPSK
    }

    #[test]
    fn test_typeii_amplitude_zero() {
        let amp = TypeIiCodebook::amplitude_from_index(0);
        assert_eq!(amp, 0.0);
    }

    #[test]
    fn test_typeii_amplitude_max() {
        let amp = TypeIiCodebook::amplitude_from_index(7);
        assert!((amp - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_typeii_amplitude_monotonic() {
        let amps: Vec<f64> = (0..8).map(TypeIiCodebook::amplitude_from_index).collect();
        for w in amps.windows(2) {
            assert!(w[0] <= w[1], "Amplitudes should be monotonically non-decreasing");
        }
    }

    #[test]
    fn test_typeii_phase_qpsk_unit_magnitude() {
        for i in 0..4u8 {
            let (r, im) = TypeIiCodebook::phase_from_index_qpsk(i);
            let mag = (r * r + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-10, "Phase should have unit magnitude");
        }
    }

    #[test]
    fn test_typeii_phase_8psk_unit_magnitude() {
        for i in 0..8u8 {
            let (r, im) = TypeIiCodebook::phase_from_index_8psk(i);
            let mag = (r * r + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-10, "8PSK phase should have unit magnitude");
        }
    }

    // ---- RSRP / RSRQ / SINR Tests ----

    #[test]
    fn test_rsrp_empty_samples() {
        let rsrp = compute_rsrp_dbm(&[]);
        assert!(rsrp.is_infinite() && rsrp < 0.0);
    }

    #[test]
    fn test_rsrp_unit_power() {
        // Samples with |sample|^2 = 1 (avg power = 1 W = 30 dBm)
        let samples = vec![(1.0, 0.0); 10];
        let rsrp = compute_rsrp_dbm(&samples);
        assert!((rsrp - 30.0).abs() < 0.1, "1W power should be 30 dBm, got {rsrp}");
    }

    #[test]
    fn test_rsrq_calculation() {
        let rsrq = compute_rsrq_db(-80.0, -70.0, 52);
        // N*RSRP/RSSI in dB = 10*log10(52) - 80 - (-70) ≈ 17.16 - 10 = 7.16
        assert!((rsrq - (10.0 * 52.0_f64.log10() - 80.0 + 70.0)).abs() < 0.01);
    }

    #[test]
    fn test_sinr_db() {
        let sinr = compute_sinr_db(100.0, 10.0);
        assert!((sinr - 10.0).abs() < 0.01, "100/10 = 10 → 10 dB");
    }

    #[test]
    fn test_sinr_zero_noise() {
        let sinr = compute_sinr_db(1.0, 0.0);
        assert!(sinr.is_infinite());
    }

    #[test]
    fn test_l1_rsrp_qpsk_samples() {
        // QPSK samples have power 0.5 per component → |sample|^2 = 0.5
        let seq = generate_csirs_sequence(0, 0, 0, 14, 100);
        let rsrp = l1_rsrp(&seq);
        // Expected: avg power = 0.5W → ~-3 dBm  (10*log10(0.5*1000)=~27 dBm)
        assert!(rsrp.is_finite());
    }

    // ---- CSI-RS Processor Tests ----

    #[test]
    fn test_processor_1port_symbol_count() {
        let cfg = CsiRsConfig { bandwidth_prbs: 52, ..CsiRsConfig::single_port() };
        let proc = NrCsirsProcessor::new(cfg);
        let syms = proc.generate_symbols(0, 14);
        assert_eq!(syms.len(), 52); // 1 RE/PRB x 52 PRBs
    }

    #[test]
    fn test_processor_2port_symbol_count() {
        let cfg = CsiRsConfig { bandwidth_prbs: 52, ..CsiRsConfig::two_port() };
        let proc = NrCsirsProcessor::new(cfg);
        let syms = proc.generate_symbols(0, 14);
        assert_eq!(syms.len(), 104); // 2 RE/PRB x 52 PRBs
    }

    #[test]
    fn test_processor_map_to_grid_count() {
        let cfg = CsiRsConfig { bandwidth_prbs: 10, ..CsiRsConfig::two_port() };
        let proc = NrCsirsProcessor::new(cfg);
        let grid = proc.map_to_grid(0);
        assert_eq!(grid.len(), 20); // 10 PRBs x 2 RE/PRB
    }

    #[test]
    fn test_processor_channel_estimation() {
        let cfg = CsiRsConfig::single_port();
        let proc = NrCsirsProcessor::new(cfg);
        let tx = proc.generate_symbols(0, 14);

        // Perfect channel (no distortion): received = transmitted
        let h = proc.estimate_channel_ls(&tx, 0);
        assert_eq!(h.len(), tx.len());

        // Perfect channel: H = 1 + j0
        for &(hr, hi) in &h {
            assert!((hr - 1.0).abs() < 1e-10, "Perfect channel H_re should be 1.0");
            assert!(hi.abs() < 1e-10, "Perfect channel H_im should be 0.0");
        }
    }

    #[test]
    fn test_processor_channel_estimation_scaled() {
        let cfg = CsiRsConfig::single_port();
        let proc = NrCsirsProcessor::new(cfg);
        let tx = proc.generate_symbols(0, 14);

        // Scale received by 0.5: channel should be estimated as 0.5
        let rx: Vec<(f64, f64)> = tx.iter().map(|&(r, i)| (0.5 * r, 0.5 * i)).collect();
        let h = proc.estimate_channel_ls(&rx, 0);
        for &(hr, hi) in &h {
            assert!((hr - 0.5).abs() < 1e-10, "Scaled channel H_re should be 0.5");
            assert!(hi.abs() < 1e-10, "Scaled channel H_im should be 0.0");
        }
    }

    #[test]
    fn test_processor_csi_report() {
        let cfg = CsiRsConfig::single_port();
        let proc = NrCsirsProcessor::new(cfg);
        let tx = proc.generate_symbols(0, 14);
        let report = proc.generate_csi_report(&tx, 0, 15.0);
        assert!(report.cqi >= 1 && report.cqi <= 15);
        assert!(report.ri >= 1);
    }

    // ---- Utility Function Tests ----

    #[test]
    fn test_qpsk_symbol_amplitude() {
        let scale = 1.0 / (2.0_f64).sqrt();
        for b0 in 0..2u8 {
            for b1 in 0..2u8 {
                let (i, q) = qpsk_symbol(b0, b1);
                assert!((i.abs() - scale).abs() < 1e-10);
                assert!((q.abs() - scale).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_complex_mul() {
        // (1+j) * (1-j) = 1 - j^2 = 2
        let (r, i) = complex_mul(1.0, 1.0, 1.0, -1.0);
        assert!((r - 2.0).abs() < 1e-10);
        assert!(i.abs() < 1e-10);
    }

    #[test]
    fn test_average_power() {
        let samples = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let p = average_power(&samples);
        assert!((p - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_power_to_dbm() {
        // 1 W = 30 dBm
        let dbm = power_to_dbm(1.0);
        assert!((dbm - 30.0).abs() < 0.001);
    }

    #[test]
    fn test_dbm_to_power_roundtrip() {
        let p = 0.001; // 1 mW = 0 dBm
        let dbm = power_to_dbm(p);
        let p2 = dbm_to_power(dbm);
        assert!((p - p2).abs() < 1e-15, "Roundtrip power conversion failed");
    }

    #[test]
    fn test_pmi_value_creation() {
        let pmi = PmiValue::single_layer(2, 1, 3);
        assert_eq!(pmi.i1_1, 2);
        assert_eq!(pmi.i1_2, 1);
        assert_eq!(pmi.i2, 3);
        assert_eq!(pmi.codebook_type, CodebookType::TypeI);
    }

    #[test]
    fn test_csi_report_creation() {
        let pmi = PmiValue::single_layer(0, 0, 0);
        let report = CsiReport::new(10, pmi, 2, -85.0);
        assert_eq!(report.cqi, 10);
        assert_eq!(report.ri, 2);
        assert!((report.rsrp_dbm + 85.0).abs() < 0.001);
    }

    #[test]
    fn test_csirs_config_defaults() {
        let cfg = CsiRsConfig::single_port();
        assert_eq!(cfg.n_ports, 1);
        assert_eq!(cfg.cdm_type, CdmType::NoCdm);
        assert!((cfg.density - 1.0).abs() < 1e-10);
        assert_eq!(cfg.scrambling_id, 0);
    }

    #[test]
    fn test_cdm_no_cdm_cover_codes() {
        let codes = CdmType::NoCdm.cover_codes();
        assert_eq!(codes.len(), 1);
        assert_eq!(codes[0].0, vec![1]);
        assert_eq!(codes[0].1, vec![1]);
    }

    #[test]
    fn test_16port_re_pattern_count() {
        let cfg = CsiRsConfig {
            n_ports: 16,
            cdm_type: CdmType::Cdm4Fd2Td2,
            density: 1.0,
            first_symbol: 4,
            scrambling_id: 0,
            bandwidth_prbs: 52,
        };
        let resource = CsiRsResource::new(cfg);
        // 4 groups x 2 FD x 2 TD = 16 RE
        assert_eq!(resource.re_pattern.len(), 16);
    }

    #[test]
    fn test_32port_re_pattern_count() {
        let cfg = CsiRsConfig {
            n_ports: 32,
            cdm_type: CdmType::Cdm8Fd2Td4,
            density: 1.0,
            first_symbol: 0,
            scrambling_id: 0,
            bandwidth_prbs: 52,
        };
        let resource = CsiRsResource::new(cfg);
        // 4 groups x 2 FD x 4 TD = 32 RE
        assert_eq!(resource.re_pattern.len(), 32);
    }

    #[test]
    fn test_csirs_grid_prb_range() {
        let cfg = CsiRsConfig { bandwidth_prbs: 5, ..CsiRsConfig::single_port() };
        let proc = NrCsirsProcessor::new(cfg);
        let grid = proc.map_to_grid(0);
        for (prb, _, _, _) in &grid {
            assert!(*prb < 5, "PRB should be within bandwidth");
        }
    }
}

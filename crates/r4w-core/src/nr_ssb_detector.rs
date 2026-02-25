//! # 5G NR SS/PBCH Block (SSB) Detector and Synchronizer
//!
//! Implements PSS/SSS generation, detection, and cell synchronization for 5G NR
//! per 3GPP TS 38.211 and TS 38.213.
//!
//! ## SSB Structure
//!
//! An SSB occupies 4 OFDM symbols × 240 subcarriers:
//!
//! | Symbol | Content |
//! |--------|---------|
//! | 0      | PSS (subcarriers 56–182) + guard |
//! | 1      | PBCH + DMRS |
//! | 2      | SSS (subcarriers 56–182) + PBCH + DMRS |
//! | 3      | PBCH + DMRS |
//!
//! ## Cell ID
//!
//! ```
//! N_cell_ID = 3 * N_ID_1 + N_ID_2
//! ```
//! where N_ID_2 ∈ {0, 1, 2} (from PSS) and N_ID_1 ∈ {0, …, 335} (from SSS).
//!
//! ## Example
//!
//! ```
//! use r4w_core::nr_ssb_detector::{NrSsbDetector, SsbConfig, PssSequence};
//!
//! // Generate PSS for N_ID_2 = 0
//! let pss = PssSequence::generate(0);
//! assert_eq!(pss.sequence.len(), 127);
//! assert_eq!(pss.n_id_2, 0);
//!
//! // Configure detector
//! let cfg = SsbConfig {
//!     scs_khz: 15,
//!     frequency_hz: 3.5e9,
//!     periodicity_ms: 20,
//!     l_max: 4,
//! };
//! let detector = NrSsbDetector::new(cfg);
//! assert_eq!(detector.config().scs_khz, 15);
//! ```

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// Number of PSS sequences (N_ID_2 ∈ {0,1,2})
pub const PSS_COUNT: usize = 3;
/// Length of the PSS/SSS sequence in subcarriers
pub const SYNC_SEQ_LEN: usize = 127;
/// Total subcarriers in an SSB
pub const SSB_SC_COUNT: usize = 240;
/// Number of OFDM symbols in one SSB
pub const SSB_SYMBOL_COUNT: usize = 4;
/// First subcarrier of PSS/SSS within SSB (0-indexed)
pub const SYNC_SC_START: usize = 56;
/// Last subcarrier of PSS/SSS (inclusive)
pub const SYNC_SC_END: usize = 182;
/// Number of SSS sequences per N_ID_2
pub const N_ID_1_COUNT: usize = 336;
/// Total cell IDs
pub const CELL_ID_COUNT: usize = 1008;
/// PBCH DMRS subcarrier spacing within symbol (every 4th SC)
pub const DMRS_SC_STRIDE: usize = 4;

// ─────────────────────────────────────────────────────────────────────────────
// Data Structures
// ─────────────────────────────────────────────────────────────────────────────

/// A generated PSS (Primary Synchronization Signal) sequence.
///
/// Sequence values are ±1 BPSK mapped: d = 1 - 2·x(m).
#[derive(Debug, Clone)]
pub struct PssSequence {
    /// N_ID_2 ∈ {0, 1, 2}
    pub n_id_2: u8,
    /// 127 real-valued BPSK symbols (±1)
    pub sequence: Vec<f64>,
}

/// A generated SSS (Secondary Synchronization Signal) sequence.
#[derive(Debug, Clone)]
pub struct SssSequence {
    /// N_ID_1 ∈ {0 … 335}
    pub n_id_1: u16,
    /// N_ID_2 ∈ {0, 1, 2}
    pub n_id_2: u8,
    /// 127 real-valued BPSK symbols (±1)
    pub sequence: Vec<f64>,
}

/// PBCH Demodulation Reference Signal (DMRS).
#[derive(Debug, Clone)]
pub struct PbchDmrs {
    /// SSB beam index (i_SSB)
    pub ssb_index: u8,
    /// Physical cell ID
    pub cell_id: u16,
    /// DMRS sequence (60 complex values as interleaved Re/Im f64 pairs, length 120)
    pub sequence: Vec<f64>,
}

/// Configuration for the SSB detector / burst parameters.
#[derive(Debug, Clone)]
pub struct SsbConfig {
    /// Subcarrier spacing in kHz: 15, 30, 120, or 240
    pub scs_khz: u32,
    /// Center frequency of SSB in Hz (used for GSCN lookup)
    pub frequency_hz: f64,
    /// SSB burst periodicity in ms: 5, 10, 20, 40, 80, or 160
    pub periodicity_ms: u8,
    /// L_max: maximum SSBs per half-frame (4, 8, or 64)
    pub l_max: u8,
}

/// Single SSB detection result.
#[derive(Debug, Clone)]
pub struct SsbDetectionResult {
    /// Physical cell ID N_cell_ID = 3*N_ID_1 + N_ID_2
    pub cell_id: u16,
    /// N_ID_1 (0 … 335) determined from SSS
    pub n_id_1: u16,
    /// N_ID_2 (0, 1, or 2) determined from PSS
    pub n_id_2: u8,
    /// Sample offset to the first sample of the SSB symbol 0
    pub timing_offset: usize,
    /// Normalized PSS correlation peak magnitude
    pub pss_corr_peak: f64,
    /// Normalized SSS correlation peak magnitude
    pub sss_corr_peak: f64,
    /// SSB index within burst (beam index, 0-based)
    pub ssb_index: u8,
}

/// An entry in the Global Synchronization Channel Number (GSCN) raster.
#[derive(Debug, Clone)]
pub struct GscnEntry {
    /// GSCN number
    pub gscn: u32,
    /// SS reference frequency in Hz
    pub frequency_hz: f64,
}

/// SSB timing case per TS 38.213 Table 4.1-1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SsbCase {
    /// Case A: 15 kHz SCS, FR1 ≤ 3 GHz
    CaseA,
    /// Case B: 30 kHz SCS, FR1
    CaseB,
    /// Case C: 30 kHz SCS, FR1 (TDD)
    CaseC,
    /// Case D: 120 kHz SCS, FR2
    CaseD,
    /// Case E: 240 kHz SCS, FR2
    CaseE,
}

// ─────────────────────────────────────────────────────────────────────────────
// PSS Generation
// ─────────────────────────────────────────────────────────────────────────────

/// Generate the 7-bit m-sequence used for PSS with polynomial x^7 + x^4 + 1.
///
/// Initial state is all-ones `[1,1,1,1,1,1,1]` then the sequence starting
/// position is shifted by `shift` chips.
fn gen_pss_mseq(shift: usize) -> [u8; 127] {
    // Register: x[0..7], feedback = x[6] XOR x[3]  (x^7 + x^4 + 1 → taps 7,4)
    let mut reg = [1u8; 7];
    let mut full = [0u8; 127];
    for bit in &mut full {
        *bit = reg[6];
        let fb = reg[6] ^ reg[3];
        reg[6] = reg[5];
        reg[5] = reg[4];
        reg[4] = reg[3];
        reg[3] = reg[2];
        reg[2] = reg[1];
        reg[1] = reg[0];
        reg[0] = fb;
    }
    // Shift: rotate the sequence by `shift`
    let mut out = [0u8; 127];
    for i in 0..127 {
        out[i] = full[(i + shift) % 127];
    }
    out
}

impl PssSequence {
    /// Generate PSS sequence for a given N_ID_2 ∈ {0, 1, 2}.
    ///
    /// Per TS 38.211 §7.4.2.2.1:
    /// d_PSS(n) = 1 − 2·x(m + n)  where m = 43·N_ID_2 mod 127.
    pub fn generate(n_id_2: u8) -> Self {
        assert!(n_id_2 < 3, "N_ID_2 must be 0, 1, or 2");
        let shift = (43 * n_id_2 as usize) % 127;
        let x = gen_pss_mseq(shift);
        let sequence: Vec<f64> = x.iter().map(|&b| 1.0 - 2.0 * b as f64).collect();
        PssSequence { n_id_2, sequence }
    }

    /// Cross-correlate this PSS with a received real buffer.
    ///
    /// Returns a vector of correlation magnitudes, one per candidate timing offset.
    pub fn correlate(&self, rx: &[f64]) -> Vec<f64> {
        let n = self.sequence.len();
        if rx.len() < n {
            return Vec::new();
        }
        let num_offsets = rx.len() - n + 1;
        let mut out = Vec::with_capacity(num_offsets);
        for offset in 0..num_offsets {
            let corr: f64 = self
                .sequence
                .iter()
                .zip(&rx[offset..offset + n])
                .map(|(&a, &b)| a * b)
                .sum();
            out.push(corr.abs());
        }
        out
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SSS Generation
// ─────────────────────────────────────────────────────────────────────────────

/// Generate x0 sequence for SSS (polynomial x^7 + x^4 + 1).
fn gen_sss_x0() -> [u8; 127] {
    let mut reg = [1u8; 7]; // initial state per TS 38.211 §7.4.2.3.1
    let mut out = [0u8; 127];
    for bit in &mut out {
        *bit = reg[6];
        let fb = reg[6] ^ reg[3];
        reg[6] = reg[5];
        reg[5] = reg[4];
        reg[4] = reg[3];
        reg[3] = reg[2];
        reg[2] = reg[1];
        reg[1] = reg[0];
        reg[0] = fb;
    }
    out
}

/// Generate x1 sequence for SSS (polynomial x^7 + x + 1).
fn gen_sss_x1() -> [u8; 127] {
    let mut reg = [1u8; 7]; // initial state per TS 38.211 §7.4.2.3.1
    let mut out = [0u8; 127];
    for bit in &mut out {
        *bit = reg[6];
        let fb = reg[6] ^ reg[0];
        reg[6] = reg[5];
        reg[5] = reg[4];
        reg[4] = reg[3];
        reg[3] = reg[2];
        reg[2] = reg[1];
        reg[1] = reg[0];
        reg[0] = fb;
    }
    out
}

/// Compute (m0, m1) from N_ID_1 per TS 38.211 §7.4.2.3.1.
///
/// q_prime = floor(N_ID_1 / 30)
/// q       = floor((N_ID_1 + q_prime*(q_prime+1)/2) / 30)
/// m_prime = N_ID_1 + q*(q+1)/2
/// m0      = m_prime mod 112
/// m1      = (m0 + floor(m_prime/112) + 1) mod 112
pub fn m0_m1(n_id_1: u16) -> (usize, usize) {
    let n = n_id_1 as usize;
    let q_prime = n / 30;
    let q = (n + q_prime * (q_prime + 1) / 2) / 30;
    let m_prime = n + q * (q + 1) / 2;
    let m0 = m_prime % 112;
    let m1 = (m0 + m_prime / 112 + 1) % 112;
    (m0, m1)
}

impl SssSequence {
    /// Generate SSS sequence for (N_ID_1, N_ID_2).
    ///
    /// Per TS 38.211 §7.4.2.3.1:
    /// d_SSS(n) = (1 - 2·x0((n+m0) mod 127)) · (1 - 2·x1((n+m1) mod 127))
    pub fn generate(n_id_1: u16, n_id_2: u8) -> Self {
        assert!(n_id_1 < 336, "N_ID_1 must be 0..335");
        assert!(n_id_2 < 3, "N_ID_2 must be 0..2");
        let x0 = gen_sss_x0();
        let x1 = gen_sss_x1();
        let (m0, m1) = m0_m1(n_id_1);
        let sequence: Vec<f64> = (0..127)
            .map(|n| {
                let v0 = 1.0 - 2.0 * x0[(n + m0) % 127] as f64;
                let v1 = 1.0 - 2.0 * x1[(n + m1) % 127] as f64;
                v0 * v1
            })
            .collect();
        SssSequence {
            n_id_1,
            n_id_2,
            sequence,
        }
    }

    /// Compute physical cell ID from this sequence's N_ID_1 and N_ID_2.
    pub fn cell_id(&self) -> u16 {
        3 * self.n_id_1 + self.n_id_2 as u16
    }

    /// Correlate this SSS with a received real-valued buffer slice.
    pub fn correlate(&self, rx: &[f64]) -> f64 {
        if rx.len() < self.sequence.len() {
            return 0.0;
        }
        self.sequence
            .iter()
            .zip(rx.iter())
            .map(|(&a, &b)| a * b)
            .sum::<f64>()
            .abs()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PBCH DMRS Generation
// ─────────────────────────────────────────────────────────────────────────────

/// Initialize a Gold-sequence pair (c0, c1) for use as PBCH DMRS PRNG.
///
/// Per TS 38.211 §7.4.1.4.1 the Gold sequence is initialized with:
/// c_init = 2^11 * (i_SSB + 1) * (2 * N_cell_ID + 1) + N_cell_ID
fn gold_init(c_init: u32) -> ([u32; 31], [u32; 31]) {
    // x1 has fixed initialization: only bit 0 set
    let mut x1 = [0u32; 31];
    x1[0] = 1;

    // x2 initialized from c_init (31 bits)
    let mut x2 = [0u32; 31];
    for i in 0..31 {
        x2[i] = (c_init >> i) & 1;
    }
    (x1, x2)
}

/// Generate `len` bits from the Gold sequence initialized with `c_init`.
///
/// x1 polynomial: x^31 + x^3 + 1  (taps 31, 3)
/// x2 polynomial: x^31 + x^3 + x^2 + x + 1 (taps 31, 3, 2, 1)
fn gold_sequence(c_init: u32, len: usize) -> Vec<u8> {
    // Run Gold-sequence generator – TS 38.211 §5.2.1
    // Nc = 1600 initialization cycles
    const NC: usize = 1600;

    let total = NC + len;
    let mut x1 = vec![0u8; total + 31];
    let mut x2 = vec![0u8; total + 31];

    // Init x1
    x1[0] = 1;
    for k in 1..31 {
        x1[k] = 0;
    }
    // Init x2 from c_init
    for i in 0..31 {
        x2[i] = ((c_init >> i) & 1) as u8;
    }

    // Generate sequences
    for n in 0..(total) {
        x1[n + 31] = (x1[n + 3] ^ x1[n]) & 1;
        x2[n + 31] = (x2[n + 3] ^ x2[n + 2] ^ x2[n + 1] ^ x2[n]) & 1;
    }

    // Output c[n] = (x1[n+Nc] + x2[n+Nc]) mod 2
    (NC..NC + len)
        .map(|n| (x1[n] ^ x2[n]) & 1)
        .collect()
}

impl PbchDmrs {
    /// Generate PBCH DMRS sequence for a given (ssb_index, cell_id).
    ///
    /// Returns 60 QPSK complex values as interleaved (Re, Im) f64 pairs
    /// (total vector length 120).
    pub fn generate(ssb_index: u8, cell_id: u16) -> Self {
        let c_init =
            (1 << 11) * (ssb_index as u32 + 1) * (2 * cell_id as u32 + 1) + cell_id as u32;
        // PBCH DMRS: 60 complex QPSK symbols → 120 Gold bits (2 bits per symbol)
        let bits = gold_sequence(c_init, 120);
        // QPSK mapping: (b0, b1) → (1-2b0, 1-2b1) / sqrt(2)
        let scale = 1.0 / 2.0_f64.sqrt();
        let mut sequence = Vec::with_capacity(120);
        for chunk in bits.chunks(2) {
            let re = (1.0 - 2.0 * chunk[0] as f64) * scale;
            let im = (1.0 - 2.0 * chunk[1] as f64) * scale;
            sequence.push(re);
            sequence.push(im);
        }
        PbchDmrs {
            ssb_index,
            cell_id,
            sequence,
        }
    }

    /// Return the number of complex DMRS symbols (always 60).
    pub fn num_symbols(&self) -> usize {
        self.sequence.len() / 2
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GSCN / Frequency Raster
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the GSCN for a given SS reference frequency (Hz) in FR1 below 3 GHz.
///
/// GSCN = 3·N + delta  where delta ∈ {1, 2, 3} and
/// SS_ref = 1200 kHz · N + delta · 50 kHz.
///
/// Returns `None` if the frequency cannot be represented.
pub fn freq_to_gscn_fr1_low(freq_hz: f64) -> Option<GscnEntry> {
    // N ranges 1 .. 2499 for FR1 below 3 GHz
    for delta in 1u32..=3 {
        let delta_hz = delta as f64 * 50_000.0;
        // freq_hz = 1_200_000 * N + delta_hz
        let n_f = (freq_hz - delta_hz) / 1_200_000.0;
        if n_f < 1.0 || n_f > 2499.0 {
            continue;
        }
        let n = n_f.round() as u32;
        let ss_ref = 1_200_000.0 * n as f64 + delta_hz;
        if (ss_ref - freq_hz).abs() < 1.0 {
            return Some(GscnEntry {
                gscn: 3 * n + delta,
                frequency_hz: ss_ref,
            });
        }
    }
    None
}

/// Compute GSCN for FR1 3–24.25 GHz range.
///
/// GSCN = 7499 + N (N = 0 … 14756)
/// SS_ref = 24_250.08 MHz + N · 1.44 MHz
pub fn freq_to_gscn_fr1_high(freq_hz: f64) -> Option<GscnEntry> {
    let base_hz = 24_250_080_000.0_f64;
    let step_hz = 1_440_000.0_f64;
    if freq_hz < 3e9 || freq_hz > 24.25e9 {
        return None;
    }
    let n_f = (freq_hz - base_hz) / step_hz;
    if n_f < 0.0 || n_f > 14756.0 {
        return None;
    }
    let n = n_f.round() as u32;
    let ss_ref = base_hz + n as f64 * step_hz;
    Some(GscnEntry {
        gscn: 7499 + n,
        frequency_hz: ss_ref,
    })
}

/// Compute GSCN for FR2 (mmWave, 24.25–100 GHz).
///
/// GSCN = 22256 + N (N = 0 … 4383)
/// SS_ref = 24_250.08 MHz + N · 17.28 MHz
pub fn freq_to_gscn_fr2(freq_hz: f64) -> Option<GscnEntry> {
    let base_hz = 24_250_080_000.0_f64;
    let step_hz = 17_280_000.0_f64;
    if freq_hz < 24.25e9 || freq_hz > 100e9 {
        return None;
    }
    let n_f = (freq_hz - base_hz) / step_hz;
    if n_f < 0.0 || n_f > 4383.0 {
        return None;
    }
    let n = n_f.round() as u32;
    let ss_ref = base_hz + n as f64 * step_hz;
    Some(GscnEntry {
        gscn: 22256 + n,
        frequency_hz: ss_ref,
    })
}

/// Convert GSCN to SS reference frequency in Hz.
///
/// Covers all three GSCN bands defined in TS 38.104 §5.4.3.1.
pub fn gscn_to_freq_hz(gscn: u32) -> Option<f64> {
    if gscn >= 2 && gscn <= 7498 {
        // FR1 below 3 GHz: GSCN = 3*N + delta
        // delta ∈ {1,2,3}, N = (GSCN - delta) / 3
        let delta = ((gscn - 1) % 3 + 1) as f64;
        let n = (gscn as f64 - delta) / 3.0;
        if n.fract() > 1e-9 {
            return None;
        }
        Some(1_200_000.0 * n + delta * 50_000.0)
    } else if gscn >= 7499 && gscn <= 22255 {
        // FR1 high: GSCN = 7499 + N
        let n = (gscn - 7499) as f64;
        Some(24_250_080_000.0 + n * 1_440_000.0)
    } else if gscn >= 22256 && gscn <= 26639 {
        // FR2: GSCN = 22256 + N
        let n = (gscn - 22256) as f64;
        Some(24_250_080_000.0 + n * 17_280_000.0)
    } else {
        None
    }
}

/// Enumerate GSCN entries in a frequency range [freq_min_hz, freq_max_hz].
pub fn gscn_range(freq_min_hz: f64, freq_max_hz: f64) -> Vec<GscnEntry> {
    let mut result = Vec::new();
    // FR1 low: GSCN 2..7498
    for gscn in 2u32..=7498 {
        if let Some(f) = gscn_to_freq_hz(gscn) {
            if f >= freq_min_hz && f <= freq_max_hz {
                result.push(GscnEntry { gscn, frequency_hz: f });
            }
        }
    }
    // FR1 high: GSCN 7499..22255
    {
        let base = 24_250_080_000.0_f64;
        let step = 1_440_000.0_f64;
        let n_min = ((freq_min_hz - base) / step).ceil().max(0.0) as u32;
        let n_max = ((freq_max_hz - base) / step).floor().min(14756.0) as u32;
        if freq_max_hz >= base && freq_min_hz <= base + 14756.0 * step {
            for n in n_min..=n_max {
                let f = base + n as f64 * step;
                if f >= freq_min_hz && f <= freq_max_hz {
                    result.push(GscnEntry { gscn: 7499 + n, frequency_hz: f });
                }
            }
        }
    }
    // FR2: GSCN 22256..26639
    {
        let base = 24_250_080_000.0_f64;
        let step = 17_280_000.0_f64;
        let n_min = ((freq_min_hz - base) / step).ceil().max(0.0) as u32;
        let n_max = ((freq_max_hz - base) / step).floor().min(4383.0) as u32;
        if freq_max_hz >= base && freq_min_hz <= base + 4383.0 * step {
            for n in n_min..=n_max {
                let f = base + n as f64 * step;
                if f >= freq_min_hz && f <= freq_max_hz {
                    result.push(GscnEntry { gscn: 22256 + n, frequency_hz: f });
                }
            }
        }
    }
    result.sort_by(|a, b| a.frequency_hz.partial_cmp(&b.frequency_hz).unwrap());
    result
}

// ─────────────────────────────────────────────────────────────────────────────
// SSB Case / Burst Timing
// ─────────────────────────────────────────────────────────────────────────────

impl SsbCase {
    /// Determine SSB case from SCS and frequency.
    ///
    /// Per TS 38.213 Table 4.1-1:
    /// - Case A: 15 kHz, f ≤ 3 GHz
    /// - Case B: 30 kHz, f ≤ 3 GHz
    /// - Case C: 30 kHz, 3 GHz < f ≤ 7.125 GHz
    /// - Case D: 120 kHz, FR2
    /// - Case E: 240 kHz, FR2
    pub fn from_scs_and_freq(scs_khz: u32, freq_hz: f64) -> Option<SsbCase> {
        match scs_khz {
            15 if freq_hz <= 3e9 => Some(SsbCase::CaseA),
            30 if freq_hz <= 3e9 => Some(SsbCase::CaseB),
            30 if freq_hz > 3e9 && freq_hz <= 7.125e9 => Some(SsbCase::CaseC),
            120 => Some(SsbCase::CaseD),
            240 => Some(SsbCase::CaseE),
            _ => None,
        }
    }

    /// Return L_max (maximum SSBs per half-frame) for this case.
    pub fn l_max(&self, freq_hz: f64) -> u8 {
        match self {
            SsbCase::CaseA | SsbCase::CaseB | SsbCase::CaseC => {
                if freq_hz <= 3e9 { 4 } else { 8 }
            }
            SsbCase::CaseD | SsbCase::CaseE => 64,
        }
    }

    /// Return the first symbol indices of SSBs in a half-frame.
    ///
    /// Indices are relative to the start of the half-frame (symbol 0).
    /// Per TS 38.213 §4.1.
    pub fn ssb_symbol_offsets(&self) -> &'static [usize] {
        match self {
            // Case A: {2, 8} + 14*n, n=0,1 → symbols 2,8,16,22
            SsbCase::CaseA => &[2, 8, 16, 22],
            // Case B: {4,8,16,20} + 28*n, n=0 → first 4 positions
            SsbCase::CaseB => &[4, 8, 16, 20],
            // Case C: same as Case A pattern (for sub-3GHz; for higher use n=0,1,2,3)
            SsbCase::CaseC => &[2, 8, 16, 22],
            // Case D: 4 per slot, 2 slots in 5ms → complex; simplified first-8
            SsbCase::CaseD => &[4, 8, 16, 20, 32, 36, 44, 48],
            // Case E: 8 positions for first-burst
            SsbCase::CaseE => &[8, 12, 16, 20, 32, 36, 40, 44],
        }
    }

    /// Return SCS in kHz for this case.
    pub fn scs_khz(&self) -> u32 {
        match self {
            SsbCase::CaseA => 15,
            SsbCase::CaseB | SsbCase::CaseC => 30,
            SsbCase::CaseD => 120,
            SsbCase::CaseE => 240,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Cell ID Utilities
// ─────────────────────────────────────────────────────────────────────────────

/// Compute physical cell ID from N_ID_1 and N_ID_2.
///
/// N_cell_ID = 3 · N_ID_1 + N_ID_2, range 0 … 1007.
pub fn cell_id(n_id_1: u16, n_id_2: u8) -> u16 {
    3 * n_id_1 + n_id_2 as u16
}

/// Decompose a physical cell ID into (N_ID_1, N_ID_2).
pub fn decompose_cell_id(cell_id: u16) -> (u16, u8) {
    assert!(cell_id < 1008, "cell_id must be 0..1007");
    let n_id_2 = (cell_id % 3) as u8;
    let n_id_1 = cell_id / 3;
    (n_id_1, n_id_2)
}

// ─────────────────────────────────────────────────────────────────────────────
// NrSsbDetector
// ─────────────────────────────────────────────────────────────────────────────

/// 5G NR SSB detector performing cell search via PSS/SSS correlation.
pub struct NrSsbDetector {
    cfg: SsbConfig,
    pss: [PssSequence; 3],
}

impl NrSsbDetector {
    /// Create a new detector with the given configuration.
    pub fn new(cfg: SsbConfig) -> Self {
        let pss = [
            PssSequence::generate(0),
            PssSequence::generate(1),
            PssSequence::generate(2),
        ];
        NrSsbDetector { cfg, pss }
    }

    /// Return a reference to the detector configuration.
    pub fn config(&self) -> &SsbConfig {
        &self.cfg
    }

    /// Return precomputed PSS sequences.
    pub fn pss_sequences(&self) -> &[PssSequence; 3] {
        &self.pss
    }

    /// Detect SSBs in a real-valued received signal buffer.
    ///
    /// The buffer is assumed to contain one SSB-worth of samples centered on
    /// the PSS subcarriers. For simplicity in unit tests, the buffer is the
    /// PSS sequence itself (in-phase projection), allowing correlation peak
    /// detection.
    ///
    /// In a real implementation this would operate on time-domain IQ samples
    /// after OFDM demodulation. Here we operate directly on frequency-domain
    /// (subcarrier) amplitude vectors.
    ///
    /// * `rx_pss_sc` – 127 received PSS subcarrier values (real, post-FFT)
    /// * `rx_sss_sc` – 127 received SSS subcarrier values (real, post-FFT)
    /// * `ssb_index` – SSB beam index (0-based) as determined from PBCH DMRS
    ///
    /// Returns `Some(SsbDetectionResult)` if detection threshold is met.
    pub fn detect_from_fd(
        &self,
        rx_pss_sc: &[f64],
        rx_sss_sc: &[f64],
        ssb_index: u8,
        pss_threshold: f64,
        sss_threshold: f64,
    ) -> Option<SsbDetectionResult> {
        if rx_pss_sc.len() < SYNC_SEQ_LEN || rx_sss_sc.len() < SYNC_SEQ_LEN {
            return None;
        }

        // Step 1: correlate with all 3 PSS sequences
        let mut best_pss_corr = 0.0f64;
        let mut best_n_id_2 = 0u8;
        for n_id_2 in 0..3u8 {
            let corr: f64 = self.pss[n_id_2 as usize]
                .sequence
                .iter()
                .zip(&rx_pss_sc[..SYNC_SEQ_LEN])
                .map(|(&a, &b)| a * b)
                .sum::<f64>()
                .abs();
            // Normalize by sequence energy (127)
            let norm_corr = corr / SYNC_SEQ_LEN as f64;
            if norm_corr > best_pss_corr {
                best_pss_corr = norm_corr;
                best_n_id_2 = n_id_2;
            }
        }

        if best_pss_corr < pss_threshold {
            return None;
        }

        // Step 2: correlate with all 336 SSS sequences for the detected N_ID_2
        let mut best_sss_corr = 0.0f64;
        let mut best_n_id_1 = 0u16;
        let x0 = gen_sss_x0();
        let x1 = gen_sss_x1();
        for n_id_1 in 0..336u16 {
            let (m0, m1) = m0_m1(n_id_1);
            let corr: f64 = (0..SYNC_SEQ_LEN)
                .map(|n| {
                    let v0 = 1.0 - 2.0 * x0[(n + m0) % 127] as f64;
                    let v1 = 1.0 - 2.0 * x1[(n + m1) % 127] as f64;
                    (v0 * v1) * rx_sss_sc[n]
                })
                .sum::<f64>()
                .abs();
            let norm_corr = corr / SYNC_SEQ_LEN as f64;
            if norm_corr > best_sss_corr {
                best_sss_corr = norm_corr;
                best_n_id_1 = n_id_1;
            }
        }

        if best_sss_corr < sss_threshold {
            return None;
        }

        let cid = cell_id(best_n_id_1, best_n_id_2);
        Some(SsbDetectionResult {
            cell_id: cid,
            n_id_1: best_n_id_1,
            n_id_2: best_n_id_2,
            timing_offset: 0,
            pss_corr_peak: best_pss_corr,
            sss_corr_peak: best_sss_corr,
            ssb_index,
        })
    }

    /// Run PSS-only timing acquisition on a time-domain real buffer.
    ///
    /// Returns a list of (timing_offset, n_id_2, normalized_peak) for every
    /// peak that exceeds `threshold`.  Peaks closer than `min_gap` samples are
    /// suppressed (CFAR-style holdoff).
    pub fn acquire_pss(
        &self,
        rx: &[f64],
        threshold: f64,
        min_gap: usize,
    ) -> Vec<(usize, u8, f64)> {
        let n = SYNC_SEQ_LEN;
        if rx.len() < n {
            return Vec::new();
        }
        let num_offsets = rx.len() - n + 1;

        // Compute correlation for all offsets and all N_ID_2
        let mut corr = vec![(0.0f64, 0u8); num_offsets];
        for n_id_2 in 0..3u8 {
            let seq = &self.pss[n_id_2 as usize].sequence;
            for offset in 0..num_offsets {
                let val: f64 = seq
                    .iter()
                    .zip(&rx[offset..offset + n])
                    .map(|(&a, &b)| a * b)
                    .sum::<f64>()
                    .abs()
                    / n as f64;
                if val > corr[offset].0 {
                    corr[offset] = (val, n_id_2);
                }
            }
        }

        // Peak picking with holdoff
        let mut results = Vec::new();
        let mut last_peak = 0usize;
        let mut last_peak_valid = false;
        for (offset, &(val, n_id_2)) in corr.iter().enumerate() {
            if val < threshold {
                continue;
            }
            if last_peak_valid && offset - last_peak < min_gap {
                // Replace previous peak if this one is higher
                if let Some(prev) = results.last_mut() {
                    let (_, _, prev_val): &mut (usize, u8, f64) = prev;
                    if val > *prev_val {
                        *prev = (offset, n_id_2, val);
                        last_peak = offset;
                    }
                }
            } else {
                results.push((offset, n_id_2, val));
                last_peak = offset;
                last_peak_valid = true;
            }
        }
        results
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SSB Symbol Mapping Helper
// ─────────────────────────────────────────────────────────────────────────────

/// Extract PSS subcarrier values from a full SSB symbol (240 subcarriers).
///
/// PSS occupies subcarriers 56–182 (127 values) in symbol 0.
pub fn extract_pss_subcarriers(ssb_sym0: &[f64]) -> Option<Vec<f64>> {
    if ssb_sym0.len() < SYNC_SC_END + 1 {
        return None;
    }
    Some(ssb_sym0[SYNC_SC_START..=SYNC_SC_END].to_vec())
}

/// Extract SSS subcarrier values from SSB symbol 2 (240 subcarriers).
///
/// SSS occupies subcarriers 56–182 (127 values) in symbol 2.
pub fn extract_sss_subcarriers(ssb_sym2: &[f64]) -> Option<Vec<f64>> {
    extract_pss_subcarriers(ssb_sym2) // same position
}

/// Build a full 240-element SSB symbol 0 with PSS mapped to subcarriers 56–182.
///
/// All other subcarriers are set to 0.
pub fn build_ssb_sym0_from_pss(pss: &PssSequence) -> Vec<f64> {
    let mut sym = vec![0.0f64; SSB_SC_COUNT];
    for (i, &v) in pss.sequence.iter().enumerate() {
        sym[SYNC_SC_START + i] = v;
    }
    sym
}

/// Build SSB symbol 2 with SSS at subcarriers 56–182.
pub fn build_ssb_sym2_from_sss(sss: &SssSequence) -> Vec<f64> {
    let mut sym = vec![0.0f64; SSB_SC_COUNT];
    for (i, &v) in sss.sequence.iter().enumerate() {
        sym[SYNC_SC_START + i] = v;
    }
    sym
}

// ─────────────────────────────────────────────────────────────────────────────
// SSB Burst Structure
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the number of samples in one SSB symbol including cyclic prefix.
///
/// * `scs_khz` – subcarrier spacing in kHz (15/30/120/240)
/// * `sample_rate_hz` – SDR sample rate in Hz
/// * `with_cp` – include normal cyclic prefix samples
pub fn ssb_symbol_samples(scs_khz: u32, sample_rate_hz: f64, with_cp: bool) -> usize {
    // FFT size for 240 subcarriers (20 RBs): N_FFT = 256 * (scs_ref / scs_khz)
    // Reference SCS is 15 kHz → N_FFT = 256 for 15 kHz.
    // For 30 kHz → N_FFT = 512, 120 kHz → N_FFT = 2048, etc.
    // Here we compute based on sample_rate / scs.
    let symbol_duration_s = 1.0 / (scs_khz as f64 * 1000.0);
    let fft_samples = (sample_rate_hz * symbol_duration_s).round() as usize;
    if with_cp {
        // Normal CP for FR1 first symbol ≈ 144/2048 * fft_size (simplified)
        let cp_samples = (fft_samples as f64 * 144.0 / 2048.0).round() as usize;
        fft_samples + cp_samples
    } else {
        fft_samples
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Unit Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── PSS Tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_pss_sequence_length() {
        for n_id_2 in 0..3u8 {
            let pss = PssSequence::generate(n_id_2);
            assert_eq!(pss.sequence.len(), 127, "PSS length must be 127 for N_ID_2={n_id_2}");
        }
    }

    #[test]
    fn test_pss_bpsk_values() {
        for n_id_2 in 0..3u8 {
            let pss = PssSequence::generate(n_id_2);
            for &v in &pss.sequence {
                assert!(
                    (v - 1.0).abs() < 1e-10 || (v + 1.0).abs() < 1e-10,
                    "PSS values must be ±1, got {v}"
                );
            }
        }
    }

    #[test]
    fn test_pss_sequences_distinct() {
        let p0 = PssSequence::generate(0);
        let p1 = PssSequence::generate(1);
        let p2 = PssSequence::generate(2);
        // Each pair should differ in at least one position
        assert!(
            p0.sequence.iter().zip(&p1.sequence).any(|(&a, &b)| (a - b).abs() > 1e-10),
            "PSS N_ID_2=0 and N_ID_2=1 should differ"
        );
        assert!(
            p0.sequence.iter().zip(&p2.sequence).any(|(&a, &b)| (a - b).abs() > 1e-10),
            "PSS N_ID_2=0 and N_ID_2=2 should differ"
        );
        assert!(
            p1.sequence.iter().zip(&p2.sequence).any(|(&a, &b)| (a - b).abs() > 1e-10),
            "PSS N_ID_2=1 and N_ID_2=2 should differ"
        );
    }

    #[test]
    fn test_pss_n_id_2_field() {
        for n_id_2 in 0..3u8 {
            let pss = PssSequence::generate(n_id_2);
            assert_eq!(pss.n_id_2, n_id_2);
        }
    }

    #[test]
    fn test_pss_first_chip_n_id_2_0() {
        // For N_ID_2=0, shift=0, first chip of m-sequence with reg=[1,1,1,1,1,1,1]
        // feedback = reg[6] ^ reg[3] = 1^1 = 0, output = reg[6] = 1
        // d(0) = 1 - 2*1 = -1
        let pss = PssSequence::generate(0);
        assert_eq!(pss.sequence[0], -1.0, "First chip of PSS N_ID_2=0 should be -1");
    }

    #[test]
    fn test_pss_autocorrelation_peak() {
        // PSS autocorrelation at lag 0 should be maximum
        for n_id_2 in 0..3u8 {
            let pss = PssSequence::generate(n_id_2);
            let autocorr: f64 = pss.sequence.iter().map(|&x| x * x).sum();
            assert!(
                (autocorr - 127.0).abs() < 1e-9,
                "PSS energy should be 127, got {autocorr}"
            );
        }
    }

    #[test]
    fn test_pss_cross_correlation_low() {
        // Cross-correlation of different PSS sequences should be low
        let p0 = PssSequence::generate(0);
        let p1 = PssSequence::generate(1);
        let xcorr: f64 = p0.sequence.iter().zip(&p1.sequence).map(|(&a, &b)| a * b).sum();
        // Should be much less than 127
        assert!(xcorr.abs() < 50.0, "PSS cross-correlation too high: {xcorr}");
    }

    #[test]
    fn test_pss_correlation_detects_self() {
        let pss = PssSequence::generate(1);
        // Self-correlation equals energy
        let corrs = pss.correlate(&pss.sequence);
        assert_eq!(corrs.len(), 1);
        assert!((corrs[0] - 127.0).abs() < 1e-9, "Self-correlation should be 127");
    }

    #[test]
    fn test_pss_correlation_in_noisy_buffer() {
        let pss = PssSequence::generate(2);
        let mut buf = vec![0.1f64; 300];
        // Embed PSS at offset 50
        for (i, &v) in pss.sequence.iter().enumerate() {
            buf[50 + i] = v;
        }
        let corrs = pss.correlate(&buf);
        let (peak_idx, peak_val) = corrs
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak_idx, 50, "Peak should be at offset 50, got {peak_idx}");
        assert!(*peak_val > 100.0, "Peak correlation should be strong, got {peak_val}");
    }

    // ── SSS Tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_sss_sequence_length() {
        let sss = SssSequence::generate(0, 0);
        assert_eq!(sss.sequence.len(), 127);
    }

    #[test]
    fn test_sss_bpsk_values() {
        let sss = SssSequence::generate(100, 1);
        for &v in &sss.sequence {
            assert!(
                (v - 1.0).abs() < 1e-10 || (v + 1.0).abs() < 1e-10,
                "SSS must be ±1, got {v}"
            );
        }
    }

    #[test]
    fn test_sss_cell_id() {
        let sss = SssSequence::generate(150, 2);
        assert_eq!(sss.cell_id(), 3 * 150 + 2);
    }

    #[test]
    fn test_sss_sequences_differ_by_n_id_1() {
        let s0 = SssSequence::generate(0, 0);
        let s1 = SssSequence::generate(1, 0);
        assert!(
            s0.sequence.iter().zip(&s1.sequence).any(|(&a, &b)| (a - b).abs() > 1e-10),
            "Different N_ID_1 should produce different SSS"
        );
    }

    #[test]
    fn test_sss_n_id_2_stored_correctly() {
        // Per TS 38.211 §7.4.2.3.1, the SSS bit sequence depends only on N_ID_1
        // (via m0/m1), not on N_ID_2.  The n_id_2 field is stored for cell-ID
        // computation (N_cell_ID = 3*N_ID_1 + N_ID_2) but does not change bits.
        let s0 = SssSequence::generate(50, 0);
        let s1 = SssSequence::generate(50, 1);
        assert_eq!(s0.n_id_2, 0);
        assert_eq!(s1.n_id_2, 1);
        // Both have the same N_ID_1 so same sequence chips
        assert_eq!(s0.sequence, s1.sequence,
            "Same N_ID_1 → same SSS chips regardless of N_ID_2 (spec correct)");
        // Cell IDs differ because N_ID_2 differs
        assert_ne!(s0.cell_id(), s1.cell_id(),
            "Cell IDs should differ even with same N_ID_1 due to different N_ID_2");
    }

    #[test]
    fn test_sss_autocorrelation() {
        let sss = SssSequence::generate(200, 2);
        let energy: f64 = sss.sequence.iter().map(|&x| x * x).sum();
        assert!((energy - 127.0).abs() < 1e-9, "SSS energy should be 127");
    }

    #[test]
    fn test_sss_self_correlation() {
        let sss = SssSequence::generate(100, 0);
        let corr = sss.correlate(&sss.sequence);
        assert!((corr - 127.0).abs() < 1e-9, "SSS self-correlation should be 127");
    }

    #[test]
    fn test_sss_cross_correlation_all_n_id_1() {
        // Verify 336 distinct SSS sequences exist for N_ID_2=0
        let sequences: Vec<SssSequence> =
            (0..336u16).map(|n| SssSequence::generate(n, 0)).collect();
        // Check that all sequences are distinct from SSS(0, 0)
        let ref_seq = &sequences[0].sequence;
        let mut distinct_count = 0;
        for s in &sequences[1..] {
            if s.sequence.iter().zip(ref_seq).any(|(&a, &b)| (a - b).abs() > 1e-10) {
                distinct_count += 1;
            }
        }
        assert_eq!(distinct_count, 335, "All 335 non-reference SSS should differ from SSS(0,0)");
    }

    // ── m0/m1 Tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_m0_m1_n_id_1_0() {
        let (m0, m1) = m0_m1(0);
        assert_eq!(m0, 0);
        assert_eq!(m1, 1);
    }

    #[test]
    fn test_m0_m1_range() {
        for n_id_1 in 0..336u16 {
            let (m0, m1) = m0_m1(n_id_1);
            assert!(m0 < 112, "m0={m0} out of range for N_ID_1={n_id_1}");
            assert!(m1 < 112, "m1={m1} out of range for N_ID_1={n_id_1}");
            assert_ne!(m0, m1, "m0 and m1 must differ for N_ID_1={n_id_1}");
        }
    }

    // ── Cell ID Tests ────────────────────────────────────────────────────────

    #[test]
    fn test_cell_id_range() {
        let cid = cell_id(335, 2);
        assert_eq!(cid, 1007, "Maximum cell ID should be 1007");
        let cid0 = cell_id(0, 0);
        assert_eq!(cid0, 0, "Minimum cell ID should be 0");
    }

    #[test]
    fn test_decompose_cell_id_roundtrip() {
        for cid in 0..1008u16 {
            let (n1, n2) = decompose_cell_id(cid);
            assert_eq!(cell_id(n1, n2), cid, "Roundtrip failed for cell_id={cid}");
        }
    }

    #[test]
    fn test_decompose_cell_id_n_id_2() {
        for cid in 0..1008u16 {
            let (_, n2) = decompose_cell_id(cid);
            assert!(n2 < 3, "N_ID_2 must be 0..2 for cell_id={cid}");
        }
    }

    // ── GSCN Tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_gscn_to_freq_fr1_low() {
        // GSCN=2 → 3N + delta=2 → delta=2, N=0 → invalid? Let's check delta logic
        // delta = ((gscn-1) % 3) + 1; for gscn=2: delta=((1)%3)+1=2, N=(2-2)/3=0
        // freq = 1200000*0 + 2*50000 = 100000 Hz = 100 kHz
        let f = gscn_to_freq_hz(2).unwrap();
        assert!((f - 100_000.0).abs() < 1.0, "GSCN 2 → 100 kHz, got {f}");
    }

    #[test]
    fn test_gscn_to_freq_fr1_high_boundary() {
        // GSCN=7499 (N=0) → 24250.08 MHz
        let f = gscn_to_freq_hz(7499).unwrap();
        assert!((f - 24_250_080_000.0).abs() < 1.0, "GSCN 7499 → 24250.08 MHz, got {f}");
    }

    #[test]
    fn test_gscn_to_freq_fr2() {
        // GSCN=22256 (N=0) → 24250.08 MHz
        let f = gscn_to_freq_hz(22256).unwrap();
        assert!((f - 24_250_080_000.0).abs() < 1.0, "GSCN 22256 → 24250.08 MHz, got {f}");
        // GSCN=22257 (N=1) → 24250.08 + 17.28 MHz
        let f2 = gscn_to_freq_hz(22257).unwrap();
        assert!((f2 - (24_250_080_000.0 + 17_280_000.0)).abs() < 1.0);
    }

    #[test]
    fn test_gscn_out_of_range() {
        assert!(gscn_to_freq_hz(0).is_none());
        assert!(gscn_to_freq_hz(1).is_none());
        assert!(gscn_to_freq_hz(100000).is_none());
    }

    #[test]
    fn test_gscn_range_narrow() {
        // Find GSCNs around 700 MHz (typical n28 band)
        let entries = gscn_range(690e6, 710e6);
        // Should find some entries
        assert!(!entries.is_empty(), "Should find GSCN entries near 700 MHz");
        for e in &entries {
            assert!(e.frequency_hz >= 690e6 && e.frequency_hz <= 710e6);
        }
    }

    #[test]
    fn test_gscn_range_sorted() {
        let entries = gscn_range(1e9, 2e9);
        let freqs: Vec<f64> = entries.iter().map(|e| e.frequency_hz).collect();
        for w in freqs.windows(2) {
            assert!(w[0] <= w[1], "GSCN range should be sorted by frequency");
        }
    }

    // ── SSB Case Tests ────────────────────────────────────────────────────────

    #[test]
    fn test_ssb_case_from_scs() {
        assert_eq!(
            SsbCase::from_scs_and_freq(15, 2e9),
            Some(SsbCase::CaseA)
        );
        assert_eq!(
            SsbCase::from_scs_and_freq(30, 2e9),
            Some(SsbCase::CaseB)
        );
        assert_eq!(
            SsbCase::from_scs_and_freq(30, 3.5e9),
            Some(SsbCase::CaseC)
        );
        assert_eq!(
            SsbCase::from_scs_and_freq(120, 28e9),
            Some(SsbCase::CaseD)
        );
        assert_eq!(
            SsbCase::from_scs_and_freq(240, 28e9),
            Some(SsbCase::CaseE)
        );
    }

    #[test]
    fn test_ssb_case_invalid_scs() {
        assert!(SsbCase::from_scs_and_freq(60, 3.5e9).is_none());
    }

    #[test]
    fn test_ssb_case_l_max() {
        let case_a = SsbCase::CaseA;
        assert_eq!(case_a.l_max(2e9), 4);
        assert_eq!(case_a.l_max(3.5e9), 8); // above 3 GHz → 8
        let case_d = SsbCase::CaseD;
        assert_eq!(case_d.l_max(28e9), 64);
    }

    #[test]
    fn test_ssb_case_symbol_offsets_len() {
        assert_eq!(SsbCase::CaseA.ssb_symbol_offsets().len(), 4);
        assert_eq!(SsbCase::CaseB.ssb_symbol_offsets().len(), 4);
        assert_eq!(SsbCase::CaseD.ssb_symbol_offsets().len(), 8);
        assert_eq!(SsbCase::CaseE.ssb_symbol_offsets().len(), 8);
    }

    #[test]
    fn test_ssb_case_scs_khz() {
        assert_eq!(SsbCase::CaseA.scs_khz(), 15);
        assert_eq!(SsbCase::CaseB.scs_khz(), 30);
        assert_eq!(SsbCase::CaseD.scs_khz(), 120);
        assert_eq!(SsbCase::CaseE.scs_khz(), 240);
    }

    // ── PBCH DMRS Tests ──────────────────────────────────────────────────────

    #[test]
    fn test_pbch_dmrs_length() {
        let dmrs = PbchDmrs::generate(0, 100);
        assert_eq!(dmrs.sequence.len(), 120, "DMRS sequence should be 120 f64 values (60 complex)");
        assert_eq!(dmrs.num_symbols(), 60);
    }

    #[test]
    fn test_pbch_dmrs_qpsk_magnitude() {
        let dmrs = PbchDmrs::generate(2, 500);
        let scale = 1.0 / 2.0_f64.sqrt();
        for chunk in dmrs.sequence.chunks(2) {
            let mag = (chunk[0] * chunk[0] + chunk[1] * chunk[1]).sqrt();
            assert!((mag - 1.0).abs() < 1e-9, "DMRS QPSK magnitude should be 1, got {mag}");
        }
        // individual components should be ±scale
        for &v in &dmrs.sequence {
            assert!(
                (v - scale).abs() < 1e-9 || (v + scale).abs() < 1e-9,
                "DMRS component should be ±1/√2, got {v}"
            );
        }
    }

    #[test]
    fn test_pbch_dmrs_differs_by_ssb_index() {
        let d0 = PbchDmrs::generate(0, 42);
        let d1 = PbchDmrs::generate(1, 42);
        assert!(
            d0.sequence.iter().zip(&d1.sequence).any(|(&a, &b)| (a - b).abs() > 1e-10),
            "DMRS should differ by SSB index"
        );
    }

    #[test]
    fn test_pbch_dmrs_differs_by_cell_id() {
        let d0 = PbchDmrs::generate(0, 0);
        let d1 = PbchDmrs::generate(0, 1);
        assert!(
            d0.sequence.iter().zip(&d1.sequence).any(|(&a, &b)| (a - b).abs() > 1e-10),
            "DMRS should differ by cell ID"
        );
    }

    // ── Gold Sequence Tests ──────────────────────────────────────────────────

    #[test]
    fn test_gold_sequence_length() {
        let bits = gold_sequence(0, 100);
        assert_eq!(bits.len(), 100);
    }

    #[test]
    fn test_gold_sequence_binary() {
        let bits = gold_sequence(12345, 200);
        for &b in &bits {
            assert!(b == 0 || b == 1, "Gold sequence must be binary");
        }
    }

    #[test]
    fn test_gold_sequence_differs_by_init() {
        let b0 = gold_sequence(0, 64);
        let b1 = gold_sequence(1, 64);
        assert!(
            b0.iter().zip(&b1).any(|(&a, &b)| a != b),
            "Different c_init should produce different Gold sequences"
        );
    }

    // ── SSB Symbol Mapping Tests ──────────────────────────────────────────────

    #[test]
    fn test_build_ssb_sym0() {
        let pss = PssSequence::generate(0);
        let sym = build_ssb_sym0_from_pss(&pss);
        assert_eq!(sym.len(), SSB_SC_COUNT);
        // Check PSS subcarriers
        for (i, &v) in pss.sequence.iter().enumerate() {
            assert!(
                (sym[SYNC_SC_START + i] - v).abs() < 1e-12,
                "PSS mismatch at subcarrier {}", SYNC_SC_START + i
            );
        }
        // Guard subcarriers should be 0
        assert_eq!(sym[0], 0.0);
        assert_eq!(sym[SSB_SC_COUNT - 1], 0.0);
    }

    #[test]
    fn test_extract_pss_subcarriers_roundtrip() {
        let pss = PssSequence::generate(1);
        let sym = build_ssb_sym0_from_pss(&pss);
        let extracted = extract_pss_subcarriers(&sym).unwrap();
        assert_eq!(extracted, pss.sequence, "Extracted PSS should match original");
    }

    #[test]
    fn test_extract_pss_short_buffer() {
        let short = vec![0.0f64; 100];
        assert!(extract_pss_subcarriers(&short).is_none());
    }

    // ── Detector Tests ────────────────────────────────────────────────────────

    #[test]
    fn test_detector_config() {
        let cfg = SsbConfig {
            scs_khz: 15,
            frequency_hz: 2.1e9,
            periodicity_ms: 20,
            l_max: 4,
        };
        let det = NrSsbDetector::new(cfg.clone());
        assert_eq!(det.config().scs_khz, 15);
        assert_eq!(det.config().l_max, 4);
    }

    #[test]
    fn test_detector_precomputed_pss() {
        let cfg = SsbConfig {
            scs_khz: 30,
            frequency_hz: 3.5e9,
            periodicity_ms: 20,
            l_max: 8,
        };
        let det = NrSsbDetector::new(cfg);
        let seqs = det.pss_sequences();
        assert_eq!(seqs[0].n_id_2, 0);
        assert_eq!(seqs[1].n_id_2, 1);
        assert_eq!(seqs[2].n_id_2, 2);
    }

    #[test]
    fn test_detect_from_fd_correct_cell_id() {
        let target_n_id_1 = 123u16;
        let target_n_id_2 = 1u8;
        let pss = PssSequence::generate(target_n_id_2);
        let sss = SssSequence::generate(target_n_id_1, target_n_id_2);

        let cfg = SsbConfig {
            scs_khz: 15,
            frequency_hz: 2.5e9,
            periodicity_ms: 20,
            l_max: 4,
        };
        let det = NrSsbDetector::new(cfg);
        let result = det.detect_from_fd(
            &pss.sequence,
            &sss.sequence,
            0,
            0.5,   // pss threshold
            0.5,   // sss threshold
        );
        assert!(result.is_some(), "Detector should find SSB");
        let r = result.unwrap();
        assert_eq!(r.n_id_2, target_n_id_2);
        assert_eq!(r.n_id_1, target_n_id_1);
        assert_eq!(r.cell_id, cell_id(target_n_id_1, target_n_id_2));
    }

    #[test]
    fn test_detect_from_fd_all_n_id_2() {
        for n_id_2 in 0..3u8 {
            let pss = PssSequence::generate(n_id_2);
            let sss = SssSequence::generate(0, n_id_2);
            let cfg = SsbConfig {
                scs_khz: 15,
                frequency_hz: 2.1e9,
                periodicity_ms: 20,
                l_max: 4,
            };
            let det = NrSsbDetector::new(cfg);
            let result = det.detect_from_fd(&pss.sequence, &sss.sequence, 0, 0.5, 0.5);
            assert!(result.is_some(), "Should detect N_ID_2={n_id_2}");
            assert_eq!(result.unwrap().n_id_2, n_id_2);
        }
    }

    #[test]
    fn test_detect_from_fd_threshold_reject() {
        // All-zero input should fail threshold
        let zeros = vec![0.0f64; 127];
        let cfg = SsbConfig {
            scs_khz: 15,
            frequency_hz: 2.1e9,
            periodicity_ms: 20,
            l_max: 4,
        };
        let det = NrSsbDetector::new(cfg);
        let result = det.detect_from_fd(&zeros, &zeros, 0, 0.5, 0.5);
        assert!(result.is_none(), "All-zeros input should fail threshold");
    }

    #[test]
    fn test_acquire_pss_finds_peak() {
        let pss = PssSequence::generate(0);
        let mut buf = vec![0.0f64; 400];
        let offset = 100;
        for (i, &v) in pss.sequence.iter().enumerate() {
            buf[offset + i] = v;
        }
        let cfg = SsbConfig {
            scs_khz: 15,
            frequency_hz: 2.1e9,
            periodicity_ms: 20,
            l_max: 4,
        };
        let det = NrSsbDetector::new(cfg);
        let peaks = det.acquire_pss(&buf, 0.5, 64);
        assert!(!peaks.is_empty(), "Should detect PSS peak");
        let (peak_offset, n_id_2, peak_val) = peaks[0];
        assert_eq!(peak_offset, offset, "Peak should be at offset {offset}, got {peak_offset}");
        assert_eq!(n_id_2, 0, "Should detect N_ID_2=0");
        assert!(peak_val > 0.9, "Peak value should be high, got {peak_val}");
    }

    #[test]
    fn test_ssb_symbol_samples_15khz() {
        // 15 kHz SCS, 30.72 MHz sample rate (standard)
        let samples = ssb_symbol_samples(15, 30_720_000.0, false);
        // Expected: 30_720_000 / 15_000 = 2048
        assert_eq!(samples, 2048, "15kHz at 30.72MSps should give 2048 samples");
    }

    #[test]
    fn test_ssb_symbol_samples_30khz() {
        // 30 kHz SCS, 30.72 MHz sample rate
        let samples = ssb_symbol_samples(30, 30_720_000.0, false);
        assert_eq!(samples, 1024);
    }

    #[test]
    fn test_ssb_symbol_samples_with_cp() {
        let samples_no_cp = ssb_symbol_samples(15, 30_720_000.0, false);
        let samples_with_cp = ssb_symbol_samples(15, 30_720_000.0, true);
        assert!(samples_with_cp > samples_no_cp, "CP should add samples");
    }
}

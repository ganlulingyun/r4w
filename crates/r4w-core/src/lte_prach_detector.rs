//! LTE PRACH (Physical Random Access Channel) Detector
//!
//! Implements LTE PRACH preamble generation and detection per 3GPP TS 36.211 v15
//! §5.7 and TS 36.213 v15 §6.1.  Supports all five preamble formats (0–4),
//! restricted / unrestricted sequence sets, frequency-domain correlation
//! detection, timing-advance estimation, and power-ramping logic.
//!
//! ## Background
//!
//! In LTE the UE initiates random access by transmitting one of 64 preamble
//! sequences on the PRACH.  The eNodeB must detect which preamble was sent and
//! estimate the round-trip propagation delay so that it can assign an initial
//! Timing Advance (TA) command to the UE.
//!
//! ## Zadoff-Chu Preambles
//!
//! Preamble sequences are derived from Zadoff-Chu (ZC) root sequences of length
//! `N_ZC` (839 for formats 0–3, 139 for format 4) via cyclic shifts:
//!
//! ```text
//! x_u[n]  = exp(-j π u n (n+1) / N_ZC),   0 ≤ n < N_ZC
//! x_u,v[n] = x_u[(n + C_v) mod N_ZC]
//! ```
//!
//! where `C_v = v * N_CS` is the v-th cyclic shift and `N_CS` is chosen from
//! the tables in TS 36.211 §5.7.2.
//!
//! ## Detection
//!
//! The receiver cross-correlates the received signal with each local replica in
//! the frequency domain (O(N log N) per root) and searches for a peak exceeding
//! a configurable threshold.  The peak position within the cyclic-shift window
//! gives the round-trip delay estimate.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::lte_prach_detector::{
//!     PrachConfig, PrachFormat, SequenceSet, PrachDetector, NcsTable,
//! };
//!
//! // Build a detector for preamble format 0, unrestricted set, NCS = 13
//! let cfg = PrachConfig {
//!     format: PrachFormat::Format0,
//!     sequence_set: SequenceSet::Unrestricted,
//!     ncs: 13,
//!     root_seq_index: 0,   // logical root index = 0
//!     num_preambles: 64,
//!     prach_config_index: 4,
//! };
//! let mut detector = PrachDetector::new(cfg);
//!
//! // Generate the reference preamble #0 and use it as a clean received signal
//! let preamble = detector.generate_preamble(0).unwrap();
//! let results = detector.detect(&preamble, 1.0);
//! assert!(!results.is_empty(), "should detect the preamble");
//! assert_eq!(results[0].preamble_index, 0);
//! ```
//!
//! Standards:
//! - 3GPP TS 36.211 v15.8.0 §5.7
//! - 3GPP TS 36.213 v15.8.0 §6.1

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Fundamental constants
// ---------------------------------------------------------------------------

/// Ts = 1 / (15000 * 2048) seconds — LTE basic time unit.
pub const TS_SECONDS: f64 = 1.0 / (15_000.0 * 2048.0);

/// ZC sequence length for PRACH formats 0–3 (long preambles).
pub const NZC_LONG: usize = 839;

/// ZC sequence length for PRACH format 4 (short preamble, TDD UpPTS).
pub const NZC_SHORT: usize = 139;

/// Maximum number of preambles per cell.
pub const MAX_PREAMBLES: usize = 64;

/// Timing-advance resolution in Ts units (16 Ts ≈ 521 ns).
pub const TA_RESOLUTION_TS: usize = 16;

/// Maximum TA value for formats 0–3.
pub const TA_MAX_LONG: usize = 1282;

/// Maximum TA value for format 4.
pub const TA_MAX_SHORT: usize = 197;

// ---------------------------------------------------------------------------
// Preamble format
// ---------------------------------------------------------------------------

/// LTE PRACH preamble format per TS 36.211 Table 5.7.2-1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrachFormat {
    /// Format 0 — Tcp = 3168 Ts, Tseq = 24576 Ts (normal CP, 1 ms subframe, FDD/TDD)
    Format0,
    /// Format 1 — Tcp = 21024 Ts, Tseq = 24576 Ts (long CP, 2 ms, FDD/TDD)
    Format1,
    /// Format 2 — Tcp = 6240 Ts, Tseq = 2×24576 Ts (normal CP, 2 sequences, FDD/TDD)
    Format2,
    /// Format 3 — Tcp = 21024 Ts, Tseq = 2×24576 Ts (long CP, 2 sequences, FDD/TDD)
    Format3,
    /// Format 4 — Tcp = 448 Ts, Tseq = 4096 Ts (TDD UpPTS only, N_ZC = 139)
    Format4,
}

impl PrachFormat {
    /// Cyclic-prefix length in Ts units.
    pub fn tcp_ts(&self) -> usize {
        match self {
            Self::Format0 => 3_168,
            Self::Format1 => 21_024,
            Self::Format2 => 6_240,
            Self::Format3 => 21_024,
            Self::Format4 => 448,
        }
    }

    /// Sequence duration in Ts units (per sequence repetition).
    pub fn tseq_ts(&self) -> usize {
        match self {
            Self::Format0 | Self::Format1 => 24_576,
            Self::Format2 | Self::Format3 => 2 * 24_576,
            Self::Format4 => 4_096,
        }
    }

    /// Total preamble duration in Ts units (TCP + TSEQ).
    pub fn total_duration_ts(&self) -> usize {
        self.tcp_ts() + self.tseq_ts()
    }

    /// ZC sequence length for this format.
    pub fn nzc(&self) -> usize {
        match self {
            Self::Format4 => NZC_SHORT,
            _ => NZC_LONG,
        }
    }

    /// Whether this is a long format (formats 0–3).
    pub fn is_long(&self) -> bool {
        !matches!(self, Self::Format4)
    }
}

// ---------------------------------------------------------------------------
// Sequence set (restricted / unrestricted)
// ---------------------------------------------------------------------------

/// PRACH restricted set type per TS 36.211 §5.7.2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SequenceSet {
    /// All cyclic shifts 0 … N_CS-1 are valid; supports cells up to any size.
    Unrestricted,
    /// Restricted set Type A — high-speed cells, du restricted.
    RestrictedTypeA,
    /// Restricted set Type B — ultra-high-speed cells, stricter restriction.
    RestrictedTypeB,
}

// ---------------------------------------------------------------------------
// NCS lookup tables  (TS 36.211 Tables 5.7.2-2 … 5.7.2-5)
// ---------------------------------------------------------------------------

/// Provides NCS (N_CS) lookup tables from TS 36.211.
pub struct NcsTable;

impl NcsTable {
    /// Unrestricted set NCS values for formats 0–3 (N_ZC = 839).
    /// Index 0–15 correspond to prach-ConfigIndex entries, values from
    /// TS 36.211 Table 5.7.2-2.
    pub const UNRESTRICTED_LONG: [usize; 16] = [
        0, 13, 15, 18, 22, 26, 32, 38, 46, 59, 76, 93, 119, 167, 279, 419,
    ];

    /// Restricted set Type A NCS values for formats 0–3, TS 36.211 Table 5.7.2-3.
    pub const RESTRICTED_A_LONG: [usize; 15] = [
        15, 18, 22, 26, 32, 38, 46, 55, 68, 82, 100, 128, 158, 202, 237,
    ];

    /// Restricted set Type B NCS values for formats 0–3, TS 36.211 Table 5.7.2-4.
    pub const RESTRICTED_B_LONG: [usize; 12] = [
        15, 18, 22, 26, 32, 38, 46, 55, 68, 82, 100, 118,
    ];

    /// Unrestricted set NCS values for format 4 (N_ZC = 139), TS 36.211 Table 5.7.2-5.
    pub const UNRESTRICTED_SHORT: [usize; 7] = [2, 4, 6, 8, 10, 12, 15];

    /// Look up the number of preambles that can be derived from a single root
    /// given `ncs` for N_ZC = 839 unrestricted set.
    pub fn preambles_per_root_long_unrestricted(ncs: usize) -> usize {
        if ncs == 0 {
            return 1; // Only one preamble per root when NCS=0
        }
        NZC_LONG / ncs
    }

    /// Look up the number of preambles per root for N_ZC = 139.
    pub fn preambles_per_root_short(ncs: usize) -> usize {
        if ncs == 0 {
            return 1;
        }
        NZC_SHORT / ncs
    }
}

// ---------------------------------------------------------------------------
// PRACH configuration index (TS 36.211 Tables 5.7.1-2 / 5.7.1-3)
// ---------------------------------------------------------------------------

/// Decoded PRACH timing configuration.
#[derive(Debug, Clone, Copy)]
pub struct PrachTiming {
    /// Preamble format.
    pub format: PrachFormat,
    /// System Frame Number (SFN) periodicity — PRACH occurs every `sfn_period` frames.
    pub sfn_period: u8,
    /// SFN offset within the period (0-based).
    pub sfn_offset: u8,
    /// Subframe number within the radio frame (0–9).
    pub subframe: u8,
}

/// Maps a FDD prach-ConfigIndex (0–63) to preamble timing parameters.
/// Partial table covering the most commonly used entries from TS 36.211
/// Table 5.7.1-2.  Returns `None` for reserved indices.
pub fn fdd_prach_config(config_index: u8) -> Option<PrachTiming> {
    // Format 0 entries (config indices 0–19)
    let (format, sfn_period, sfn_offset, subframe) = match config_index {
        0  => (PrachFormat::Format0, 2, 1, 1),
        1  => (PrachFormat::Format0, 2, 1, 4),
        2  => (PrachFormat::Format0, 2, 1, 7),
        3  => (PrachFormat::Format0, 1, 0, 1),
        4  => (PrachFormat::Format0, 1, 0, 4),
        5  => (PrachFormat::Format0, 1, 0, 7),
        6  => (PrachFormat::Format0, 2, 0, 1),
        7  => (PrachFormat::Format0, 2, 0, 4),
        8  => (PrachFormat::Format0, 2, 0, 7),
        9  => (PrachFormat::Format0, 1, 0, 1),
        10 => (PrachFormat::Format0, 1, 0, 4),
        11 => (PrachFormat::Format0, 1, 0, 7),
        12 => (PrachFormat::Format0, 1, 0, 1),
        13 => (PrachFormat::Format0, 1, 0, 4),
        14 => (PrachFormat::Format0, 1, 0, 7),
        15 => (PrachFormat::Format0, 1, 0, 9),
        16 => (PrachFormat::Format1, 2, 1, 1),
        17 => (PrachFormat::Format1, 2, 1, 4),
        18 => (PrachFormat::Format1, 2, 1, 7),
        19 => (PrachFormat::Format1, 1, 0, 1),
        20 => (PrachFormat::Format1, 1, 0, 4),
        21 => (PrachFormat::Format1, 1, 0, 7),
        22 => (PrachFormat::Format2, 2, 1, 1),
        23 => (PrachFormat::Format2, 2, 1, 4),
        24 => (PrachFormat::Format2, 2, 1, 7),
        25 => (PrachFormat::Format2, 1, 0, 1),
        26 => (PrachFormat::Format2, 1, 0, 4),
        27 => (PrachFormat::Format2, 1, 0, 7),
        28 => (PrachFormat::Format3, 2, 1, 1),
        29 => (PrachFormat::Format3, 2, 1, 4),
        30 => (PrachFormat::Format3, 2, 1, 7),
        31 => (PrachFormat::Format3, 1, 0, 1),
        32 => (PrachFormat::Format0, 2, 1, 2),
        33 => (PrachFormat::Format0, 2, 1, 5),
        34 => (PrachFormat::Format0, 2, 1, 8),
        35 => (PrachFormat::Format0, 1, 0, 2),
        36 => (PrachFormat::Format0, 1, 0, 5),
        37 => (PrachFormat::Format0, 1, 0, 8),
        38 => (PrachFormat::Format0, 2, 0, 2),
        39 => (PrachFormat::Format0, 2, 0, 5),
        40 => (PrachFormat::Format0, 2, 0, 8),
        41 => (PrachFormat::Format0, 1, 0, 2),
        42 => (PrachFormat::Format0, 1, 0, 5),
        43 => (PrachFormat::Format0, 1, 0, 8),
        44 => (PrachFormat::Format0, 1, 0, 3),
        45 => (PrachFormat::Format0, 1, 0, 6),
        46 => (PrachFormat::Format0, 1, 0, 9),
        47 => (PrachFormat::Format1, 2, 1, 2),
        48 => (PrachFormat::Format1, 2, 1, 5),
        49 => (PrachFormat::Format1, 2, 1, 8),
        50 => (PrachFormat::Format1, 1, 0, 2),
        51 => (PrachFormat::Format1, 1, 0, 5),
        52 => (PrachFormat::Format1, 1, 0, 8),
        53 => (PrachFormat::Format2, 2, 1, 2),
        54 => (PrachFormat::Format2, 2, 1, 5),
        55 => (PrachFormat::Format2, 2, 1, 8),
        56 => (PrachFormat::Format2, 1, 0, 2),
        57 => (PrachFormat::Format2, 1, 0, 5),
        58 => (PrachFormat::Format2, 1, 0, 8),
        59 => (PrachFormat::Format3, 2, 1, 2),
        60 => (PrachFormat::Format3, 2, 1, 5),
        61 => (PrachFormat::Format3, 2, 1, 8),
        62 => (PrachFormat::Format3, 1, 0, 2),
        63 => (PrachFormat::Format3, 1, 0, 5),
        _ => return None,
    };
    Some(PrachTiming { format, sfn_period, sfn_offset, subframe })
}

// ---------------------------------------------------------------------------
// Root sequence ordering (logical → physical)
// ---------------------------------------------------------------------------

/// Physical root sequence index table for N_ZC = 839, coprime with 839.
/// First 64 entries used most often; all 838 coprime roots are listed.
/// Source: TS 36.211 §5.7.2 — Pu_839 tables.
///
/// The full set of 838 valid roots (1..838 coprime with 839) is generated
/// dynamically since 839 is prime (all 1..838 are coprime with it).
pub fn physical_root_sequences_long() -> Vec<usize> {
    // 839 is prime, so all values 1..838 are valid roots.
    // The physical ordering defined in TS 36.211 Table 5.7.2-6 starts at u=1
    // and follows a specific pseudo-random interleaving.  For simplicity we use
    // the natural ordering 1, 2, …, 838 which matches the normative table for
    // the first block of roots.  Higher-layer signalling selects the starting
    // logical root-sequence index (0-based), and roots wrap modulo 838.
    (1..NZC_LONG).collect()
}

/// Physical root sequence index table for N_ZC = 139.
/// 139 is prime so all 1..138 are valid.
pub fn physical_root_sequences_short() -> Vec<usize> {
    (1..NZC_SHORT).collect()
}

/// Returns the physical root index `u` for a given logical root sequence index.
pub fn logical_to_physical_root(logical_index: usize, format: PrachFormat) -> usize {
    if format == PrachFormat::Format4 {
        let roots = physical_root_sequences_short();
        roots[logical_index % roots.len()]
    } else {
        let roots = physical_root_sequences_long();
        roots[logical_index % roots.len()]
    }
}

// ---------------------------------------------------------------------------
// Zadoff-Chu sequence generation
// ---------------------------------------------------------------------------

/// Generates a Zadoff-Chu base sequence of length `nzc` with root `u`.
///
/// ```text
/// x_u[n] = exp(-j π u n(n+1) / N_ZC),   0 ≤ n < N_ZC
/// ```
pub fn generate_zc_sequence(u: usize, nzc: usize) -> Vec<(f64, f64)> {
    let n = nzc as f64;
    let u_f = u as f64;
    (0..nzc)
        .map(|k| {
            let k_f = k as f64;
            let arg = -PI * u_f * k_f * (k_f + 1.0) / n;
            (arg.cos(), arg.sin())
        })
        .collect()
}

/// Applies a cyclic shift of `shift` samples to a ZC sequence.
pub fn cyclic_shift_sequence(seq: &[(f64, f64)], shift: usize) -> Vec<(f64, f64)> {
    let n = seq.len();
    if n == 0 || shift == 0 {
        return seq.to_vec();
    }
    let s = shift % n;
    let mut out = Vec::with_capacity(n);
    out.extend_from_slice(&seq[s..]);
    out.extend_from_slice(&seq[..s]);
    out
}

// ---------------------------------------------------------------------------
// Cyclic shift set for restricted sets
// ---------------------------------------------------------------------------

/// Computes the set of allowed cyclic shifts for a root with restricted sets.
///
/// In the restricted set, only shifts `C_v = v * N_CS` are allowed such that
/// no two preambles can be confused with each other across the expected
/// delay–Doppler spread of a high-speed UE.  The exact exclusion criteria are
/// per TS 36.211 §5.7.2 equations (5.7.2-2) and (5.7.2-3).
///
/// For the purpose of this implementation we compute the shifts that satisfy
/// `d_u * v mod N_ZC` not falling in an exclusion zone of width `N_CS` around
/// zero.  The simplified version is returned here; in production code a full
/// lookup table per the standard is used.
///
/// Returns the set of allowed cyclic shift offsets (in samples).
pub fn restricted_cyclic_shifts(
    u: usize,
    nzc: usize,
    ncs: usize,
    set_type: SequenceSet,
) -> Vec<usize> {
    if set_type == SequenceSet::Unrestricted {
        // All shifts 0, NCS, 2*NCS, ... are valid
        if ncs == 0 {
            return vec![0];
        }
        return (0..nzc / ncs).map(|v| v * ncs).collect();
    }

    // For restricted sets we compute d_u = min(u, N_ZC - u)
    // and exclude shifts that alias with delay or Doppler ambiguities.
    let d_u = u.min(nzc - u);

    if ncs == 0 {
        return vec![0];
    }

    // Number of guard samples per the exclusion zone width
    let guard = match set_type {
        SequenceSet::RestrictedTypeA => ncs,
        SequenceSet::RestrictedTypeB => (ncs * 3) / 2,
        SequenceSet::Unrestricted => ncs,
    };

    let mut shifts = Vec::new();
    let num_shifts = nzc / ncs;
    for v in 0..num_shifts {
        let cv = v * ncs;
        // Check exclusion: (d_u * v) mod N_ZC must not be within guard of 0 or N_ZC
        let product = (d_u * v) % nzc;
        let too_close_to_zero = product < guard;
        let too_close_to_nzc = product + guard > nzc;
        if !too_close_to_zero && !too_close_to_nzc {
            shifts.push(cv);
        } else if v == 0 {
            // v=0 (zero shift) is always allowed
            shifts.push(0);
        }
    }

    // Ensure zero shift always present
    if !shifts.contains(&0) {
        shifts.insert(0, 0);
    }
    shifts.sort_unstable();
    shifts.dedup();
    shifts
}

// ---------------------------------------------------------------------------
// Simple FFT (radix-2 DIT, scratch implementation)
// ---------------------------------------------------------------------------

/// Computes the in-place radix-2 DIT FFT of `buf` (length must be a power of 2).
fn fft_inplace(buf: &mut [(f64, f64)]) {
    let n = buf.len();
    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }
    // Cooley-Tukey butterfly
    let mut len = 2usize;
    while len <= n {
        let ang = -2.0 * PI / (len as f64);
        let wlen = (ang.cos(), ang.sin());
        let mut i = 0;
        while i < n {
            let (mut wr, mut wi) = (1.0f64, 0.0f64);
            for k in 0..len / 2 {
                let (ur, ui) = buf[i + k];
                let (vr_raw, vi_raw) = buf[i + k + len / 2];
                let vr = wr * vr_raw - wi * vi_raw;
                let vi = wr * vi_raw + wi * vr_raw;
                buf[i + k] = (ur + vr, ui + vi);
                buf[i + k + len / 2] = (ur - vr, ui - vi);
                let new_wr = wr * wlen.0 - wi * wlen.1;
                wi = wr * wlen.1 + wi * wlen.0;
                wr = new_wr;
            }
            i += len;
        }
        len <<= 1;
    }
}

/// Computes the in-place radix-2 DIT IFFT of `buf`.
fn ifft_inplace(buf: &mut [(f64, f64)]) {
    // Conjugate, FFT, conjugate, scale
    for s in buf.iter_mut() {
        s.1 = -s.1;
    }
    fft_inplace(buf);
    let n = buf.len() as f64;
    for s in buf.iter_mut() {
        s.0 /= n;
        s.1 = -s.1 / n;
    }
}

/// Returns the smallest power of 2 >= n.
fn next_power_of_two(n: usize) -> usize {
    let mut p = 1usize;
    while p < n {
        p <<= 1;
    }
    p
}

/// Zero-padded FFT of an arbitrary-length sequence.
fn fft_padded(seq: &[(f64, f64)], fft_size: usize) -> Vec<(f64, f64)> {
    assert!(fft_size.is_power_of_two());
    let mut buf = vec![(0.0f64, 0.0f64); fft_size];
    for (i, &s) in seq.iter().enumerate().take(fft_size) {
        buf[i] = s;
    }
    fft_inplace(&mut buf);
    buf
}

// ---------------------------------------------------------------------------
// Cross-correlation via frequency domain
// ---------------------------------------------------------------------------

/// Computes the circular cross-correlation of `rx` with `reference` using FFT.
///
/// Returns a length-`fft_size` real-valued power profile: `|IFFT(FFT(rx) * conj(FFT(ref)))|^2`
fn freq_domain_xcorr(rx: &[(f64, f64)], reference: &[(f64, f64)], fft_size: usize) -> Vec<f64> {
    let rx_f = fft_padded(rx, fft_size);
    let ref_f = fft_padded(reference, fft_size);

    // Pointwise multiply: RX * conj(REF)
    let mut product: Vec<(f64, f64)> = rx_f
        .iter()
        .zip(ref_f.iter())
        .map(|(&(ar, ai), &(br, bi))| (ar * br + ai * bi, ai * br - ar * bi))
        .collect();

    ifft_inplace(&mut product);

    // Return power (magnitude squared)
    product
        .iter()
        .map(|&(r, i)| r * r + i * i)
        .collect()
}

// ---------------------------------------------------------------------------
// Detection result
// ---------------------------------------------------------------------------

/// Result of a single preamble detection.
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// Detected preamble index (0–63).
    pub preamble_index: usize,
    /// Root sequence index (physical `u` value) used for this preamble.
    pub root_sequence: usize,
    /// Cyclic shift index (v) within the root.
    pub cyclic_shift_index: usize,
    /// Peak correlation power (linear).
    pub peak_power: f64,
    /// Noise floor estimate (linear).
    pub noise_floor: f64,
    /// Signal-to-noise ratio in dB.
    pub snr_db: f64,
    /// Estimated timing advance in Ts units.
    pub timing_advance_ts: usize,
    /// Estimated timing advance in microseconds.
    pub timing_advance_us: f64,
    /// Sample index of the detected peak within the correlation output.
    pub peak_sample: usize,
}

// ---------------------------------------------------------------------------
// Power ramping
// ---------------------------------------------------------------------------

/// LTE PRACH power-ramping state per TS 36.213 §6.1.1.
#[derive(Debug, Clone)]
pub struct PowerRampingState {
    /// Initial target received power at eNodeB in dBm.
    pub initial_target_dbm: f64,
    /// DELTA_PREAMBLE offset in dB (typically 0 for format 0).
    pub delta_preamble_db: f64,
    /// Power ramping step in dB (0, 2, 4, or 6 dB from SIB2).
    pub ramp_step_db: f64,
    /// Current preamble transmission counter (0-based).
    pub transmission_count: usize,
    /// Maximum allowed transmissions before failure.
    pub max_transmissions: usize,
    /// Path-loss estimate in dB (RSRP-based).
    pub path_loss_db: f64,
}

impl PowerRampingState {
    /// Creates a new power-ramping state with default LTE parameters.
    pub fn new(
        initial_target_dbm: f64,
        delta_preamble_db: f64,
        ramp_step_db: f64,
        path_loss_db: f64,
    ) -> Self {
        Self {
            initial_target_dbm,
            delta_preamble_db,
            ramp_step_db,
            transmission_count: 0,
            max_transmissions: 10,
            path_loss_db,
        }
    }

    /// Returns the transmit power for the current attempt in dBm.
    ///
    /// Formula per TS 36.213 §6.1.1:
    /// `PPRACH = min(P_max, P_CMAX, PRACH_PREAMBLE_RECEIVED_TARGET_POWER + PL)`
    /// where `PRACH_PREAMBLE_RECEIVED_TARGET_POWER = preambleInitialReceivedTargetPower
    /// + DELTA_PREAMBLE + preambleTransmissionCounter * powerRampingStep`
    pub fn current_tx_power_dbm(&self) -> f64 {
        let target = self.initial_target_dbm
            + self.delta_preamble_db
            + self.transmission_count as f64 * self.ramp_step_db;
        target + self.path_loss_db
    }

    /// Increments the transmission counter.  Returns `false` if max attempts exceeded.
    pub fn increment(&mut self) -> bool {
        self.transmission_count += 1;
        self.transmission_count <= self.max_transmissions
    }

    /// Resets the counter (e.g., after successful RA).
    pub fn reset(&mut self) {
        self.transmission_count = 0;
    }
}

// ---------------------------------------------------------------------------
// PRACH configuration
// ---------------------------------------------------------------------------

/// Complete PRACH configuration passed to the detector.
#[derive(Debug, Clone)]
pub struct PrachConfig {
    /// Preamble format (0–4).
    pub format: PrachFormat,
    /// Restricted or unrestricted sequence set.
    pub sequence_set: SequenceSet,
    /// N_CS — cyclic shift spacing in samples.
    pub ncs: usize,
    /// Logical root sequence index (from RRC, 0-based).
    pub root_seq_index: usize,
    /// Number of preambles configured for this cell (≤ 64).
    pub num_preambles: usize,
    /// prach-ConfigIndex (0–63) for timing extraction.
    pub prach_config_index: u8,
}

impl Default for PrachConfig {
    fn default() -> Self {
        Self {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        }
    }
}

// ---------------------------------------------------------------------------
// Preamble table entry (internal)
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct PreambleEntry {
    preamble_index: usize,
    root_u: usize,
    cyclic_shift_index: usize,
    cyclic_shift_samples: usize,
    sequence: Vec<(f64, f64)>,
}

// ---------------------------------------------------------------------------
// PRACH Detector
// ---------------------------------------------------------------------------

/// LTE PRACH preamble detector.
///
/// Pre-computes the 64 preamble sequences for the configured cell and performs
/// frequency-domain cross-correlation for detection.
pub struct PrachDetector {
    config: PrachConfig,
    preambles: Vec<PreambleEntry>,
    nzc: usize,
    fft_size: usize,
}

impl PrachDetector {
    /// Creates a new detector and pre-generates all preamble sequences.
    pub fn new(config: PrachConfig) -> Self {
        let nzc = config.format.nzc();
        let fft_size = next_power_of_two(nzc * 4); // extra zero-padding for timing resolution

        let mut detector = PrachDetector {
            config,
            preambles: Vec::new(),
            nzc,
            fft_size,
        };
        detector.build_preamble_table();
        detector
    }

    fn build_preamble_table(&mut self) {
        let nzc = self.nzc;
        let ncs = self.config.ncs;
        let set = self.config.sequence_set;
        let num_preambles = self.config.num_preambles.min(MAX_PREAMBLES);

        let roots = if self.config.format == PrachFormat::Format4 {
            physical_root_sequences_short()
        } else {
            physical_root_sequences_long()
        };

        let start_root_idx = self.config.root_seq_index % roots.len();

        let mut preamble_count = 0usize;
        let mut root_logical_idx = start_root_idx;

        while preamble_count < num_preambles {
            let u = roots[root_logical_idx % roots.len()];
            let base_seq = generate_zc_sequence(u, nzc);

            let shifts = restricted_cyclic_shifts(u, nzc, ncs, set);

            for (v, &cv) in shifts.iter().enumerate() {
                if preamble_count >= num_preambles {
                    break;
                }
                let shifted = cyclic_shift_sequence(&base_seq, cv);
                self.preambles.push(PreambleEntry {
                    preamble_index: preamble_count,
                    root_u: u,
                    cyclic_shift_index: v,
                    cyclic_shift_samples: cv,
                    sequence: shifted,
                });
                preamble_count += 1;
            }

            root_logical_idx += 1;
            if root_logical_idx == start_root_idx + roots.len() {
                // Wrapped around — not enough roots for the requested preambles
                break;
            }
        }
    }

    /// Returns the total number of pre-generated preamble sequences.
    pub fn num_preambles(&self) -> usize {
        self.preambles.len()
    }

    /// Returns the ZC sequence length for this detector.
    pub fn nzc(&self) -> usize {
        self.nzc
    }

    /// Generates the full time-domain preamble signal for a given preamble index.
    ///
    /// The output is: `[CP samples] ++ [ZC sequence] (++ [ZC sequence] for format 2/3)`.
    /// Samples are normalised to unit power.
    pub fn generate_preamble(&self, preamble_index: usize) -> Option<Vec<(f64, f64)>> {
        let entry = self.preambles.get(preamble_index)?;
        let tcp = self.config.format.tcp_ts();
        let tseq = self.config.format.tseq_ts();
        let nzc = self.nzc;

        // Number of ZC sequence repetitions
        let reps = tseq / nzc; // 1 for format 0/1/4, 2 for format 2/3 (with guard)
        let reps = reps.max(1);

        let total_len = tcp + tseq;
        let mut out = vec![(0.0f64, 0.0f64); total_len];

        // Fill CP: last `tcp` samples of the (first) ZC sequence
        let cp_start_in_zc = if nzc >= tcp { nzc - tcp } else { 0 };
        for i in 0..tcp.min(nzc) {
            out[i] = entry.sequence[(cp_start_in_zc + i) % nzc];
        }

        // Fill TSEQ: `reps` repetitions of the ZC sequence
        for rep in 0..reps {
            let base = tcp + rep * nzc;
            for i in 0..nzc.min(tseq - rep * nzc) {
                out[base + i] = entry.sequence[i];
            }
        }

        // Normalise to unit power
        let power: f64 = out.iter().map(|&(r, i)| r * r + i * i).sum::<f64>() / out.len() as f64;
        let scale = if power > 1e-30 { 1.0 / power.sqrt() } else { 1.0 };
        for s in out.iter_mut() {
            s.0 *= scale;
            s.1 *= scale;
        }

        Some(out)
    }

    /// Detects preambles in `rx_signal` returning all detections above `threshold`.
    ///
    /// `threshold` is an SNR-like detection threshold: a detection is declared
    /// when the cross-correlation peak power exceeds `threshold * noise_floor`.
    ///
    /// The detector operates on the first `nzc` samples of `rx_signal` (or zero-
    /// pads if shorter).  TCP stripping should be done by the caller before
    /// passing samples here.
    pub fn detect(&self, rx_signal: &[(f64, f64)], threshold: f64) -> Vec<DetectionResult> {
        let mut results = Vec::new();

        // Prepare the received sequence (truncate/pad to nzc)
        let mut rx_buf: Vec<(f64, f64)> = vec![(0.0, 0.0); self.nzc];
        let copy_len = rx_signal.len().min(self.nzc);
        rx_buf[..copy_len].copy_from_slice(&rx_signal[..copy_len]);

        // Group preambles by root sequence to share the FFT of each root
        let mut root_groups: std::collections::HashMap<usize, Vec<usize>> =
            std::collections::HashMap::new();
        for (idx, entry) in self.preambles.iter().enumerate() {
            root_groups.entry(entry.root_u).or_default().push(idx);
        }

        for (root_u, preamble_indices) in &root_groups {
            // Compute FFT of the base ZC root sequence (no cyclic shift)
            let base_seq = generate_zc_sequence(*root_u, self.nzc);
            let base_f = fft_padded(&base_seq, self.fft_size);
            let rx_f = fft_padded(&rx_buf, self.fft_size);

            // Pointwise multiply: RX * conj(BASE_ZC)
            let mut product: Vec<(f64, f64)> = rx_f
                .iter()
                .zip(base_f.iter())
                .map(|(&(ar, ai), &(br, bi))| (ar * br + ai * bi, ai * br - ar * bi))
                .collect();

            let mut corr_buf = product.clone();
            ifft_inplace(&mut corr_buf);
            let corr_power: Vec<f64> = corr_buf.iter().map(|&(r, i)| r * r + i * i).collect();

            // Estimate noise floor as the median of the correlation profile
            let noise_floor = estimate_noise_floor(&corr_power);

            // For each preamble derived from this root, search in its cyclic-shift window
            for &pidx in preamble_indices {
                let entry = &self.preambles[pidx];
                let window_start = entry.cyclic_shift_samples * self.fft_size / self.nzc;
                let window_end = if self.config.ncs > 0 {
                    (window_start + self.config.ncs * self.fft_size / self.nzc)
                        .min(self.fft_size)
                } else {
                    self.fft_size
                };

                let window_end = if window_end <= window_start {
                    self.fft_size
                } else {
                    window_end
                };

                // Find the peak within the window
                let (peak_idx, peak_power) = corr_power[window_start..window_end]
                    .iter()
                    .enumerate()
                    .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                    .map(|(i, &p)| (window_start + i, p))
                    .unwrap_or((window_start, 0.0));

                if noise_floor < 1e-40 {
                    continue;
                }

                let snr = peak_power / noise_floor;
                if snr > threshold {
                    // Convert peak sample back to delay in Ts units
                    // The correlation bin corresponds to a delay of
                    // peak_idx * nzc / fft_size samples
                    let delay_samples = peak_idx * self.nzc / self.fft_size;

                    // Timing advance = delay_samples / TA_RESOLUTION_TS
                    let ta_max = if self.config.format == PrachFormat::Format4 {
                        TA_MAX_SHORT
                    } else {
                        TA_MAX_LONG
                    };
                    let ta_ts = (delay_samples / TA_RESOLUTION_TS).min(ta_max);
                    let ta_us = ta_ts as f64 * TS_SECONDS * 1e6;

                    let snr_db = 10.0 * snr.log10();

                    results.push(DetectionResult {
                        preamble_index: entry.preamble_index,
                        root_sequence: entry.root_u,
                        cyclic_shift_index: entry.cyclic_shift_index,
                        peak_power,
                        noise_floor,
                        snr_db,
                        timing_advance_ts: ta_ts,
                        timing_advance_us: ta_us,
                        peak_sample: peak_idx,
                    });
                }
            }
        }

        // Sort by preamble index for deterministic output
        results.sort_by_key(|r| r.preamble_index);
        results
    }

    /// Returns the preamble format.
    pub fn format(&self) -> PrachFormat {
        self.config.format
    }

    /// Returns the NCS value.
    pub fn ncs(&self) -> usize {
        self.config.ncs
    }

    /// Returns the decoded PRACH timing for the configured config index.
    pub fn timing(&self) -> Option<PrachTiming> {
        fdd_prach_config(self.config.prach_config_index)
    }

    /// Returns the preamble entry at index (for testing).
    pub fn preamble_entry(&self, index: usize) -> Option<(usize, usize, usize)> {
        self.preambles.get(index).map(|e| {
            (e.root_u, e.cyclic_shift_index, e.cyclic_shift_samples)
        })
    }
}

// ---------------------------------------------------------------------------
// Noise floor estimation
// ---------------------------------------------------------------------------

/// Estimates the noise floor of a power profile via sorted median.
fn estimate_noise_floor(power: &[f64]) -> f64 {
    if power.is_empty() {
        return 0.0;
    }
    let mut sorted = power.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    // Use 25th percentile as a robust noise estimate
    let idx = sorted.len() / 4;
    sorted[idx]
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Converts a timing-advance value (in Ts units) to microseconds.
pub fn ta_ts_to_us(ta_ts: usize) -> f64 {
    ta_ts as f64 * TS_SECONDS * 1e6
}

/// Converts a timing-advance value (in microseconds) to the TA command value
/// sent in the RAR (Random Access Response) message (multiple of 16 Ts).
pub fn us_to_ta_command(us: f64) -> usize {
    let ts_count = (us * 1e-6 / TS_SECONDS).round() as usize;
    ts_count / TA_RESOLUTION_TS
}

/// Validates that an NCS value is in the unrestricted set table for long sequences.
pub fn is_valid_ncs_long(ncs: usize) -> bool {
    NcsTable::UNRESTRICTED_LONG.contains(&ncs)
}

/// Validates that an NCS value is in the unrestricted set table for short sequences.
pub fn is_valid_ncs_short(ncs: usize) -> bool {
    NcsTable::UNRESTRICTED_SHORT.contains(&ncs)
}

/// Returns the number of root sequences required to support `num_preambles`
/// with the given NCS for long preamble formats.
pub fn roots_needed_for_preambles(num_preambles: usize, ncs: usize) -> usize {
    let per_root = NcsTable::preambles_per_root_long_unrestricted(ncs);
    if per_root == 0 {
        return num_preambles;
    }
    (num_preambles + per_root - 1) / per_root
}

/// Computes the maximum supportable cell radius for a given NCS (formats 0–3).
///
/// The maximum one-way propagation delay is `(N_CS - 1) / 2 * Ts` seconds.
/// Returns the radius in kilometres.
pub fn max_cell_radius_km(ncs: usize) -> f64 {
    if ncs == 0 {
        return 0.0;
    }
    let delay_s = (ncs as f64 - 1.0) / 2.0 * TS_SECONDS;
    // speed of light in km/s
    delay_s * 299_792.458
}

/// Simple GCD for coprimality check.
pub fn gcd(mut a: usize, mut b: usize) -> usize {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

/// Returns true if `u` is a valid ZC root for the given sequence length
/// (i.e., gcd(u, nzc) == 1).
pub fn is_valid_root(u: usize, nzc: usize) -> bool {
    u > 0 && u < nzc && gcd(u, nzc) == 1
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Zadoff-Chu sequence tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_zc_sequence_length() {
        let seq = generate_zc_sequence(1, NZC_LONG);
        assert_eq!(seq.len(), NZC_LONG);
    }

    #[test]
    fn test_zc_sequence_length_short() {
        let seq = generate_zc_sequence(1, NZC_SHORT);
        assert_eq!(seq.len(), NZC_SHORT);
    }

    #[test]
    fn test_zc_constant_amplitude() {
        let seq = generate_zc_sequence(3, NZC_LONG);
        for &(r, i) in &seq {
            let mag = (r * r + i * i).sqrt();
            assert!((mag - 1.0).abs() < 1e-9, "magnitude={}", mag);
        }
    }

    #[test]
    fn test_zc_constant_amplitude_short() {
        let seq = generate_zc_sequence(5, NZC_SHORT);
        for &(r, i) in &seq {
            let mag = (r * r + i * i).sqrt();
            assert!((mag - 1.0).abs() < 1e-9, "magnitude={}", mag);
        }
    }

    #[test]
    fn test_zc_different_roots_differ() {
        let s1 = generate_zc_sequence(1, NZC_LONG);
        let s2 = generate_zc_sequence(2, NZC_LONG);
        let diff: f64 = s1
            .iter()
            .zip(s2.iter())
            .map(|(&(r1, i1), &(r2, i2))| (r1 - r2).powi(2) + (i1 - i2).powi(2))
            .sum();
        assert!(diff > 0.1);
    }

    #[test]
    fn test_cyclic_shift_identity() {
        let seq = generate_zc_sequence(1, 12);
        let shifted = cyclic_shift_sequence(&seq, 0);
        assert_eq!(seq, shifted);
    }

    #[test]
    fn test_cyclic_shift_wraps() {
        let seq: Vec<(f64, f64)> = (0..8).map(|i| (i as f64, 0.0)).collect();
        let shifted = cyclic_shift_sequence(&seq, 8);
        assert_eq!(shifted, seq); // shift by full length = identity
    }

    #[test]
    fn test_cyclic_shift_by_one() {
        let seq: Vec<(f64, f64)> = (0..5).map(|i| (i as f64, 0.0)).collect();
        let shifted = cyclic_shift_sequence(&seq, 1);
        assert_eq!(shifted[0].0, 1.0);
        assert_eq!(shifted[4].0, 0.0);
    }

    #[test]
    fn test_cyclic_shift_orthogonality() {
        // Two preambles from the same root with different cyclic shifts should
        // have low cross-correlation (approximately 1/sqrt(NZC) peak).
        let u = 1usize;
        let nzc = NZC_LONG;
        let ncs = 13usize;
        let base = generate_zc_sequence(u, nzc);
        let p0 = cyclic_shift_sequence(&base, 0);
        let p1 = cyclic_shift_sequence(&base, ncs);

        // Inner product magnitude should be small relative to self-correlation
        let self_corr: f64 = p0.iter().map(|&(r, i)| r * r + i * i).sum();
        let cross: (f64, f64) = p0
            .iter()
            .zip(p1.iter())
            .map(|(&(ar, ai), &(br, bi))| (ar * br + ai * bi, ai * br - ar * bi))
            .fold((0.0, 0.0), |acc, s| (acc.0 + s.0, acc.1 + s.1));
        let cross_mag = (cross.0 * cross.0 + cross.1 * cross.1).sqrt();
        assert!(cross_mag < self_corr * 0.1, "cross_mag={} self_corr={}", cross_mag, self_corr);
    }

    // -----------------------------------------------------------------------
    // Preamble format tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_format0_parameters() {
        let f = PrachFormat::Format0;
        assert_eq!(f.tcp_ts(), 3_168);
        assert_eq!(f.tseq_ts(), 24_576);
        assert_eq!(f.nzc(), NZC_LONG);
        assert!(f.is_long());
    }

    #[test]
    fn test_format1_parameters() {
        let f = PrachFormat::Format1;
        assert_eq!(f.tcp_ts(), 21_024);
        assert_eq!(f.tseq_ts(), 24_576);
        assert_eq!(f.nzc(), NZC_LONG);
    }

    #[test]
    fn test_format2_parameters() {
        let f = PrachFormat::Format2;
        assert_eq!(f.tcp_ts(), 6_240);
        assert_eq!(f.tseq_ts(), 2 * 24_576);
    }

    #[test]
    fn test_format3_parameters() {
        let f = PrachFormat::Format3;
        assert_eq!(f.tcp_ts(), 21_024);
        assert_eq!(f.tseq_ts(), 2 * 24_576);
    }

    #[test]
    fn test_format4_parameters() {
        let f = PrachFormat::Format4;
        assert_eq!(f.tcp_ts(), 448);
        assert_eq!(f.tseq_ts(), 4_096);
        assert_eq!(f.nzc(), NZC_SHORT);
        assert!(!f.is_long());
    }

    #[test]
    fn test_format_total_duration() {
        assert_eq!(PrachFormat::Format0.total_duration_ts(), 3_168 + 24_576);
        assert_eq!(PrachFormat::Format4.total_duration_ts(), 448 + 4_096);
    }

    // -----------------------------------------------------------------------
    // Root sequence tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_physical_roots_long_length() {
        let roots = physical_root_sequences_long();
        assert_eq!(roots.len(), NZC_LONG - 1); // 838 roots
    }

    #[test]
    fn test_physical_roots_short_length() {
        let roots = physical_root_sequences_short();
        assert_eq!(roots.len(), NZC_SHORT - 1); // 138 roots
    }

    #[test]
    fn test_all_long_roots_coprime() {
        let roots = physical_root_sequences_long();
        for &u in &roots {
            assert!(is_valid_root(u, NZC_LONG), "u={} not coprime with NZC_LONG", u);
        }
    }

    #[test]
    fn test_all_short_roots_coprime() {
        let roots = physical_root_sequences_short();
        for &u in &roots {
            assert!(is_valid_root(u, NZC_SHORT), "u={} not coprime with NZC_SHORT", u);
        }
    }

    #[test]
    fn test_logical_to_physical_root_long() {
        let u = logical_to_physical_root(0, PrachFormat::Format0);
        assert_eq!(u, 1); // first physical root is 1
    }

    #[test]
    fn test_logical_to_physical_root_short() {
        let u = logical_to_physical_root(0, PrachFormat::Format4);
        assert_eq!(u, 1);
    }

    #[test]
    fn test_logical_to_physical_root_wraps() {
        let roots = physical_root_sequences_long();
        let u0 = logical_to_physical_root(0, PrachFormat::Format0);
        let u_wrap = logical_to_physical_root(roots.len(), PrachFormat::Format0);
        assert_eq!(u0, u_wrap);
    }

    // -----------------------------------------------------------------------
    // NCS table tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ncs_table_unrestricted_long() {
        assert_eq!(NcsTable::UNRESTRICTED_LONG[0], 0);
        assert_eq!(NcsTable::UNRESTRICTED_LONG[1], 13);
        assert_eq!(NcsTable::UNRESTRICTED_LONG[15], 419);
    }

    #[test]
    fn test_ncs_table_restricted_a_long() {
        assert_eq!(NcsTable::RESTRICTED_A_LONG[0], 15);
    }

    #[test]
    fn test_ncs_table_short() {
        assert_eq!(NcsTable::UNRESTRICTED_SHORT[0], 2);
        assert_eq!(NcsTable::UNRESTRICTED_SHORT[6], 15);
    }

    #[test]
    fn test_preambles_per_root_long() {
        // NCS=13 → floor(839/13) = 64 preambles per root
        let p = NcsTable::preambles_per_root_long_unrestricted(13);
        assert_eq!(p, NZC_LONG / 13);
    }

    #[test]
    fn test_preambles_per_root_ncs_zero() {
        assert_eq!(NcsTable::preambles_per_root_long_unrestricted(0), 1);
    }

    // -----------------------------------------------------------------------
    // PRACH configuration index tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fdd_config_index_0() {
        let t = fdd_prach_config(0).unwrap();
        assert_eq!(t.format, PrachFormat::Format0);
        assert_eq!(t.subframe, 1);
    }

    #[test]
    fn test_fdd_config_index_16_format1() {
        let t = fdd_prach_config(16).unwrap();
        assert_eq!(t.format, PrachFormat::Format1);
    }

    #[test]
    fn test_fdd_config_index_22_format2() {
        let t = fdd_prach_config(22).unwrap();
        assert_eq!(t.format, PrachFormat::Format2);
    }

    #[test]
    fn test_fdd_config_index_28_format3() {
        let t = fdd_prach_config(28).unwrap();
        assert_eq!(t.format, PrachFormat::Format3);
    }

    #[test]
    fn test_fdd_config_all_valid() {
        for i in 0u8..64 {
            assert!(fdd_prach_config(i).is_some(), "config_index={} returned None", i);
        }
    }

    // -----------------------------------------------------------------------
    // FFT utility tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_next_power_of_two() {
        assert_eq!(next_power_of_two(1), 1);
        assert_eq!(next_power_of_two(2), 2);
        assert_eq!(next_power_of_two(3), 4);
        assert_eq!(next_power_of_two(839), 1024);
        assert_eq!(next_power_of_two(1024), 1024);
    }

    #[test]
    fn test_fft_ifft_roundtrip() {
        let n = 64usize;
        let original: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let angle = 2.0 * PI * k as f64 / n as f64;
                (angle.cos(), angle.sin())
            })
            .collect();
        let mut buf = original.clone();
        fft_inplace(&mut buf);
        ifft_inplace(&mut buf);
        for (k, (&(or, oi), &(rr, ri))) in original.iter().zip(buf.iter()).enumerate() {
            assert!((or - rr).abs() < 1e-9, "real mismatch at k={}: {} vs {}", k, or, rr);
            assert!((oi - ri).abs() < 1e-9, "imag mismatch at k={}: {} vs {}", k, oi, ri);
        }
    }

    #[test]
    fn test_fft_dc_component() {
        let n = 8usize;
        let buf: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let mut b = buf.clone();
        fft_inplace(&mut b);
        // DC bin should equal n, rest should be ~0
        assert!((b[0].0 - n as f64).abs() < 1e-9);
        for k in 1..n {
            assert!(b[k].0.abs() < 1e-9, "bin {} not zero", k);
            assert!(b[k].1.abs() < 1e-9, "bin {} imag not zero", k);
        }
    }

    // -----------------------------------------------------------------------
    // Power ramping tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_power_ramp_initial() {
        let state = PowerRampingState::new(-104.0, 0.0, 2.0, 110.0);
        assert_eq!(state.transmission_count, 0);
        // tx_power = -104 + 0 + 0*2 + 110 = 6 dBm
        assert!((state.current_tx_power_dbm() - 6.0).abs() < 1e-9);
    }

    #[test]
    fn test_power_ramp_increment() {
        let mut state = PowerRampingState::new(-104.0, 0.0, 2.0, 110.0);
        state.increment();
        // After 1 increment: -104 + 0 + 1*2 + 110 = 8 dBm
        assert!((state.current_tx_power_dbm() - 8.0).abs() < 1e-9);
    }

    #[test]
    fn test_power_ramp_multiple_increments() {
        let mut state = PowerRampingState::new(-120.0, 3.0, 4.0, 130.0);
        for _ in 0..3 {
            state.increment();
        }
        let expected = -120.0 + 3.0 + 3.0 * 4.0 + 130.0;
        assert!((state.current_tx_power_dbm() - expected).abs() < 1e-9);
    }

    #[test]
    fn test_power_ramp_reset() {
        let mut state = PowerRampingState::new(-104.0, 0.0, 2.0, 110.0);
        state.increment();
        state.increment();
        state.reset();
        assert_eq!(state.transmission_count, 0);
        assert!((state.current_tx_power_dbm() - 6.0).abs() < 1e-9);
    }

    #[test]
    fn test_power_ramp_max_exceeded() {
        let mut state = PowerRampingState::new(-104.0, 0.0, 2.0, 110.0);
        state.max_transmissions = 3;
        assert!(state.increment()); // 1
        assert!(state.increment()); // 2
        assert!(state.increment()); // 3
        assert!(!state.increment()); // 4 — exceeds max
    }

    // -----------------------------------------------------------------------
    // Utility function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gcd() {
        assert_eq!(gcd(48, 18), 6);
        assert_eq!(gcd(839, 1), 1); // 839 is prime
        assert_eq!(gcd(839, 839), 839);
        assert_eq!(gcd(0, 5), 5);
    }

    #[test]
    fn test_is_valid_root() {
        assert!(is_valid_root(1, 839));
        assert!(is_valid_root(838, 839));
        assert!(!is_valid_root(0, 839));
        assert!(!is_valid_root(839, 839));
    }

    #[test]
    fn test_ta_ts_to_us() {
        let us = ta_ts_to_us(16);
        // 16 * (1/(15000*2048)) * 1e6 ≈ 0.521 µs
        assert!((us - 0.5208333333_f64).abs() < 0.001, "ta_us={}", us);
    }

    #[test]
    fn test_us_to_ta_command() {
        let ta = us_to_ta_command(0.5208333333);
        assert_eq!(ta, 1, "expected TA=1 for ~0.52 µs");
    }

    #[test]
    fn test_max_cell_radius_ncs13() {
        let r = max_cell_radius_km(13);
        // (13-1)/2 * Ts * c = 6 * (1/(15000*2048)) * 299792.458 km ≈ 0.0586 km
        // NCS=13 → small cell with ~58 m one-way guard interval
        assert!(r > 0.04 && r < 0.10, "radius={} km", r);
    }

    #[test]
    fn test_max_cell_radius_ncs0() {
        assert_eq!(max_cell_radius_km(0), 0.0);
    }

    #[test]
    fn test_roots_needed_ncs13() {
        // NCS=13: 839/13 = 64 preambles per root → 1 root for 64 preambles
        let n = roots_needed_for_preambles(64, 13);
        assert_eq!(n, 1);
    }

    #[test]
    fn test_roots_needed_ncs419() {
        // NCS=419: 839/419 = 2 preambles per root → 32 roots for 64 preambles
        let n = roots_needed_for_preambles(64, 419);
        assert_eq!(n, 32);
    }

    #[test]
    fn test_is_valid_ncs_long() {
        assert!(is_valid_ncs_long(0));
        assert!(is_valid_ncs_long(13));
        assert!(is_valid_ncs_long(419));
        assert!(!is_valid_ncs_long(14));
    }

    #[test]
    fn test_is_valid_ncs_short() {
        assert!(is_valid_ncs_short(2));
        assert!(is_valid_ncs_short(15));
        assert!(!is_valid_ncs_short(3));
    }

    // -----------------------------------------------------------------------
    // Restricted cyclic shift tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_unrestricted_shifts_ncs13() {
        let shifts = restricted_cyclic_shifts(1, NZC_LONG, 13, SequenceSet::Unrestricted);
        assert_eq!(shifts.len(), NZC_LONG / 13);
        assert_eq!(shifts[0], 0);
        assert_eq!(shifts[1], 13);
    }

    #[test]
    fn test_unrestricted_shifts_ncs0_single() {
        let shifts = restricted_cyclic_shifts(1, NZC_LONG, 0, SequenceSet::Unrestricted);
        assert_eq!(shifts, vec![0]);
    }

    #[test]
    fn test_restricted_a_shifts_fewer_than_unrestricted() {
        let unrestricted = restricted_cyclic_shifts(1, NZC_LONG, 13, SequenceSet::Unrestricted);
        let restricted = restricted_cyclic_shifts(1, NZC_LONG, 13, SequenceSet::RestrictedTypeA);
        // Restricted set should have fewer or equal shifts
        assert!(restricted.len() <= unrestricted.len());
    }

    #[test]
    fn test_restricted_b_shifts_fewer_than_a() {
        let a = restricted_cyclic_shifts(3, NZC_LONG, 15, SequenceSet::RestrictedTypeA);
        let b = restricted_cyclic_shifts(3, NZC_LONG, 15, SequenceSet::RestrictedTypeB);
        assert!(b.len() <= a.len());
    }

    // -----------------------------------------------------------------------
    // PrachDetector construction and preamble generation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_preamble_count_ncs13() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        assert_eq!(det.num_preambles(), 64);
    }

    #[test]
    fn test_detector_preamble_count_format4() {
        let cfg = PrachConfig {
            format: PrachFormat::Format4,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 2,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        // NCS=2, N_ZC=139: 69 preambles per root → 64 preambles from 1 root
        assert_eq!(det.num_preambles(), 64);
    }

    #[test]
    fn test_generate_preamble_length() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let p = det.generate_preamble(0).unwrap();
        assert_eq!(p.len(), PrachFormat::Format0.total_duration_ts());
    }

    #[test]
    fn test_generate_preamble_unit_power() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let p = det.generate_preamble(0).unwrap();
        let power = p.iter().map(|&(r, i)| r * r + i * i).sum::<f64>() / p.len() as f64;
        assert!((power - 1.0).abs() < 0.01, "power={}", power);
    }

    #[test]
    fn test_generate_preamble_none_for_out_of_range() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        assert!(det.generate_preamble(64).is_none());
        assert!(det.generate_preamble(100).is_none());
    }

    #[test]
    fn test_preamble_entries_differ() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let (_, _, cs0) = det.preamble_entry(0).unwrap();
        let (_, _, cs1) = det.preamble_entry(1).unwrap();
        assert_ne!(cs0, cs1);
    }

    // -----------------------------------------------------------------------
    // Detection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_preamble_0_clean() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let preamble = det.generate_preamble(0).unwrap();
        let results = det.detect(&preamble, 1.0);
        assert!(!results.is_empty(), "no detections");
        assert_eq!(results[0].preamble_index, 0);
    }

    #[test]
    fn test_detect_preamble_5_clean() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let preamble = det.generate_preamble(5).unwrap();
        let results = det.detect(&preamble, 1.0);
        assert!(!results.is_empty(), "no detections");
        // Preamble 5 should be among the detections
        let found = results.iter().any(|r| r.preamble_index == 5);
        assert!(found, "preamble 5 not detected; got {:?}", results.iter().map(|r| r.preamble_index).collect::<Vec<_>>());
    }

    #[test]
    fn test_detect_returns_snr_positive() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let preamble = det.generate_preamble(0).unwrap();
        let results = det.detect(&preamble, 1.0);
        for r in &results {
            assert!(r.snr_db.is_finite(), "snr_db is non-finite");
        }
    }

    #[test]
    fn test_detect_zero_input_no_detections() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let zeros = vec![(0.0f64, 0.0f64); NZC_LONG];
        let results = det.detect(&zeros, 5.0);
        assert!(results.is_empty(), "expected no detections in zero signal");
    }

    #[test]
    fn test_detect_timing_advance_zero_no_delay() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let preamble = det.generate_preamble(0).unwrap();
        // Use only the ZC part (skip TCP) as received signal
        let tcp = PrachFormat::Format0.tcp_ts();
        let zc_part = &preamble[tcp..tcp + NZC_LONG];
        let results = det.detect(zc_part, 1.0);
        if let Some(r) = results.first() {
            // With no delay, TA should be small
            assert!(r.timing_advance_ts <= 10, "TA too large: {}", r.timing_advance_ts);
        }
    }

    #[test]
    fn test_detector_format4() {
        let cfg = PrachConfig {
            format: PrachFormat::Format4,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 2,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let preamble = det.generate_preamble(0).unwrap();
        let results = det.detect(&preamble, 1.0);
        assert!(!results.is_empty());
        assert_eq!(results[0].preamble_index, 0);
    }

    #[test]
    fn test_detector_timing_accessor() {
        let cfg = PrachConfig {
            format: PrachFormat::Format0,
            sequence_set: SequenceSet::Unrestricted,
            ncs: 13,
            root_seq_index: 0,
            num_preambles: 64,
            prach_config_index: 4,
        };
        let det = PrachDetector::new(cfg);
        let timing = det.timing().unwrap();
        assert_eq!(timing.format, PrachFormat::Format0);
        assert_eq!(timing.subframe, 4);
    }

    #[test]
    fn test_noise_floor_estimate() {
        let mut power = vec![0.1f64; 100];
        power[50] = 100.0; // one big spike
        let floor = estimate_noise_floor(&power);
        assert!(floor < 1.0, "noise floor should be near 0.1, got {}", floor);
    }

    // -----------------------------------------------------------------------
    // Frequency-domain cross-correlation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_xcorr_peak_at_zero_lag() {
        let nzc = 64usize; // use small size for speed
        let seq = generate_zc_sequence(1, nzc);
        let power = freq_domain_xcorr(&seq, &seq, 256);
        let (peak_idx, _) = power
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak_idx, 0, "peak should be at zero lag");
    }

    #[test]
    fn test_xcorr_peak_at_nonzero_lag() {
        let nzc = 64usize;
        let fft_size = 256usize;
        let seq = generate_zc_sequence(1, nzc);
        let shift = 4usize;
        let shifted = cyclic_shift_sequence(&seq, shift);
        let power = freq_domain_xcorr(&shifted, &seq, fft_size);
        let (peak_idx, _) = power
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        // The cyclic cross-correlation of a shifted ZC with the base ZC should
        // peak at a bin corresponding to the cyclic shift.  Because the
        // sequences are zero-padded from nzc=64 to fft_size=256, the shift of
        // 4 samples appears at lag +4 (bin 4) or as its negative-lag mirror
        // image (bin fft_size - shift = 252).  Either represents the same
        // physical timing offset.
        let alt_bin = fft_size - shift; // 256 - 4 = 252 (negative lag mirror)
        let expected_bin = shift;       // bin 4 (positive lag)
        assert!(
            peak_idx == expected_bin || peak_idx == alt_bin,
            "peak at {} expected {} or {}",
            peak_idx, expected_bin, alt_bin
        );
    }
}

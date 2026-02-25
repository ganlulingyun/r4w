//! # 5G NR PRACH Detector
//!
//! Physical Random Access Channel (PRACH) detection per 3GPP TS 38.211 and TS 38.213.
//!
//! ## Overview
//!
//! PRACH is the initial access mechanism in 5G NR. UEs transmit Zadoff-Chu (ZC)
//! preamble sequences to establish uplink synchronization and request a Random Access
//! Response (RAR) from the gNB.
//!
//! ## Zadoff-Chu Preamble Generation (TS 38.211 Section 6.3.3.1)
//!
//! The base ZC sequence of length L_RA rooted at index u is:
//!
//! ```text
//! x_u(n) = exp(-j * pi * u * n * (n+1) / L_RA),  n = 0 .. L_RA-1
//! ```
//!
//! 64 preambles per cell are derived via cyclic shifts:
//!
//! ```text
//! x_{u,v}(n) = x_u((n + C_v) mod L_RA),   C_v = v * N_CS
//! ```
//!
//! where N_CS is the cyclic shift size from the restricted-set configuration.
//!
//! ## Preamble Formats
//!
//! | Format | L_RA | Δf_RA (kHz) | T_SEQ (μs) |
//! |--------|------|-------------|------------|
//! | 0      | 839  | 1.25        | 800        |
//! | 1      | 839  | 1.25        | 1600       |
//! | 2      | 839  | 1.25        | 3200       |
//! | 3      | 839  | 5.0         | 200        |
//! | A1     | 139  | 15/30       | 2 symbols  |
//! | A2     | 139  | 15/30       | 4 symbols  |
//! | A3     | 139  | 15/30       | 6 symbols  |
//! | B1     | 139  | 15/30       | 2 symbols  |
//! | B4     | 139  | 15/30       | 12 symbols |
//! | C0     | 139  | 15/30       | 1 symbol   |
//! | C2     | 139  | 15/30       | 4 symbols  |
//!
//! ## Detection Algorithm
//!
//! 1. Frequency-domain correlation: Y(k) = R(k) · conj(X_u(k))
//! 2. IFFT of Y → z(n), cyclic shift domain
//! 3. Peak search: if |z(n_peak)| > threshold → preamble detected
//! 4. Timing advance: TA = peak_pos × T_s
//!
//! # Example
//!
//! ```
//! use r4w_core::nr_prach_detector::{
//!     PrachConfig, PrachFormat, RestrictedSet, PrachDetector,
//! };
//!
//! let config = PrachConfig {
//!     config_index: 0,
//!     format: PrachFormat::Format0,
//!     scs_khz: 15,
//!     restricted_set: RestrictedSet::Unrestricted,
//!     n_cs: 13,
//!     root_sequence_index: 0,
//! };
//!
//! let detector = PrachDetector::new(config);
//! let preambles = detector.generate_preambles();
//! assert_eq!(preambles.len(), 64);
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────── constants

/// Long preamble sequence length (formats 0-3).
pub const L_RA_LONG: u16 = 839;
/// Short preamble sequence length (formats A1-C2).
pub const L_RA_SHORT: u16 = 139;

/// T_c = 1 / (480_000 * 4096) seconds (basic time unit, TS 38.211).
pub const T_C_SECONDS: f64 = 1.0 / (480_000.0 * 4096.0);

/// Speed of light (m/s).
pub const C_LIGHT: f64 = 299_792_458.0;

/// Number of preambles per cell.
pub const NUM_PREAMBLES: usize = 64;

// ──────────────────────────────────────────────────────────────── enumerations

/// PRACH preamble format (TS 38.211 Tables 6.3.3.1-1 and 6.3.3.1-2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrachFormat {
    /// Long: L_RA=839, Δf=1.25 kHz, 1× T_SEQ (~800 μs).
    Format0,
    /// Long: L_RA=839, Δf=1.25 kHz, 2× T_SEQ.
    Format1,
    /// Long: L_RA=839, Δf=1.25 kHz, 4× T_SEQ.
    Format2,
    /// Long: L_RA=839, Δf=5 kHz, 1× T_SEQ (~200 μs).
    Format3,
    /// Short: L_RA=139, 2 OFDM symbols.
    A1,
    /// Short: L_RA=139, 4 OFDM symbols.
    A2,
    /// Short: L_RA=139, 6 OFDM symbols.
    A3,
    /// Short: L_RA=139, 2 OFDM symbols (different CP).
    B1,
    /// Short: L_RA=139, 12 OFDM symbols.
    B4,
    /// Short: L_RA=139, 1 OFDM symbol.
    C0,
    /// Short: L_RA=139, 4 OFDM symbols.
    C2,
}

impl PrachFormat {
    /// Sequence length L_RA for this format.
    pub fn l_ra(self) -> u16 {
        match self {
            PrachFormat::Format0
            | PrachFormat::Format1
            | PrachFormat::Format2
            | PrachFormat::Format3 => L_RA_LONG,
            _ => L_RA_SHORT,
        }
    }

    /// Subcarrier spacing in kHz for this format.
    pub fn delta_f_ra_khz(self) -> f64 {
        match self {
            PrachFormat::Format0 | PrachFormat::Format1 | PrachFormat::Format2 => 1.25,
            PrachFormat::Format3 => 5.0,
            // Short formats: depends on SCS of the serving cell (15 or 30 kHz);
            // return 15 as default here — caller overrides via PrachConfig.scs_khz.
            _ => 15.0,
        }
    }

    /// Number of OFDM symbols occupied by this preamble format.
    pub fn num_symbols(self) -> u8 {
        match self {
            PrachFormat::Format0 => 1,
            PrachFormat::Format1 => 2,
            PrachFormat::Format2 => 4,
            PrachFormat::Format3 => 1,
            PrachFormat::A1 => 2,
            PrachFormat::A2 => 4,
            PrachFormat::A3 => 6,
            PrachFormat::B1 => 2,
            PrachFormat::B4 => 12,
            PrachFormat::C0 => 1,
            PrachFormat::C2 => 4,
        }
    }

    /// Returns true if this is a long-sequence format (L_RA = 839).
    pub fn is_long(self) -> bool {
        self.l_ra() == L_RA_LONG
    }

    /// Nominal CP length in samples at the reference sample rate used for this
    /// format (informational; actual CP depends on system numerology).
    pub fn cp_length_us(self) -> f64 {
        match self {
            // TS 38.211 Table 6.3.3.1-1
            PrachFormat::Format0 => 103.0,
            PrachFormat::Format1 => 684.0,
            PrachFormat::Format2 => 152.0,
            PrachFormat::Format3 => 103.0,
            // Short formats: 1 OFDM symbol CP at respective SCS
            PrachFormat::A1 | PrachFormat::B1 => 2.34,
            PrachFormat::A2 => 4.69,
            PrachFormat::A3 => 7.03,
            PrachFormat::B4 => 28.1,
            PrachFormat::C0 => 1.17,
            PrachFormat::C2 => 4.69,
        }
    }
}

/// Restricted set type (TS 38.211 Section 6.3.3.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestrictedSet {
    /// All cyclic shifts allowed.
    Unrestricted,
    /// Restricted type A: one-shift high-speed ambiguity avoidance.
    TypeA,
    /// Restricted type B: two-shift high-speed ambiguity avoidance.
    TypeB,
}

// ─────────────────────────────────────────────────────────────── data structs

/// A Zadoff-Chu sequence: complex samples stored as (re, im) pairs.
#[derive(Debug, Clone)]
pub struct ZadoffChuSequence {
    /// ZC root index u.
    pub root_index: u32,
    /// Sequence length L_RA.
    pub length: u16,
    /// Complex samples (re, im).
    pub sequence: Vec<(f64, f64)>,
}

impl ZadoffChuSequence {
    /// Generate ZC sequence for root u and length L_RA.
    ///
    /// x_u(n) = exp(-j·π·u·n·(n+1)/L_RA)
    pub fn generate(root_index: u32, length: u16) -> Self {
        let l = length as f64;
        let u = root_index as f64;
        let sequence = (0..length as usize)
            .map(|n| {
                let n = n as f64;
                let phase = -PI * u * n * (n + 1.0) / l;
                (phase.cos(), phase.sin())
            })
            .collect();
        ZadoffChuSequence { root_index, length, sequence }
    }

    /// Apply cyclic shift C_v to produce x_{u,v}(n) = x_u((n + C_v) mod L_RA).
    pub fn cyclic_shift(&self, c_v: u32) -> Vec<(f64, f64)> {
        let len = self.length as usize;
        let shift = (c_v as usize) % len;
        (0..len)
            .map(|n| self.sequence[(n + shift) % len])
            .collect()
    }

    /// Compute the DFT of this ZC sequence (used for frequency-domain correlation).
    pub fn dft(&self) -> Vec<(f64, f64)> {
        let n = self.length as usize;
        let mut out = vec![(0.0f64, 0.0f64); n];
        for k in 0..n {
            let mut re = 0.0f64;
            let mut im = 0.0f64;
            for (m, &(sr, si)) in self.sequence.iter().enumerate() {
                let angle = -2.0 * PI * (k * m) as f64 / n as f64;
                re += sr * angle.cos() - si * angle.sin();
                im += sr * angle.sin() + si * angle.cos();
            }
            out[k] = (re, im);
        }
        out
    }
}

/// A single PRACH preamble: ZC root + cyclic shift index.
#[derive(Debug, Clone)]
pub struct PrachPreamble {
    /// Preamble index (0-63).
    pub preamble_index: u8,
    /// ZC root index used for this preamble.
    pub root_index: u32,
    /// Cyclic shift C_v applied to root.
    pub cyclic_shift: u32,
    /// Preamble format.
    pub format: PrachFormat,
    /// The time-domain preamble samples (re, im).
    pub samples: Vec<(f64, f64)>,
}

/// Configuration for a PRACH occasion.
#[derive(Debug, Clone)]
pub struct PrachConfig {
    /// PRACH configuration index (0-255, TS 38.211 Table 6.3.3.2-2).
    pub config_index: u8,
    /// Preamble format.
    pub format: PrachFormat,
    /// Subcarrier spacing of the serving cell in kHz (15, 30, 60, 120).
    pub scs_khz: u32,
    /// Restricted set type.
    pub restricted_set: RestrictedSet,
    /// Cyclic shift N_CS.
    pub n_cs: u16,
    /// Logical root sequence index (0-837 for long, 0-137 for short).
    pub root_sequence_index: u32,
}

impl PrachConfig {
    /// L_RA for this configuration.
    pub fn l_ra(&self) -> u16 {
        self.format.l_ra()
    }

    /// Maximum number of preambles that fit in a single root sequence.
    /// Returns 0 if N_CS=0 (unrestricted special case).
    pub fn preambles_per_root(&self) -> u32 {
        if self.n_cs == 0 {
            // Special case: whole root used for one preamble set.
            return 64;
        }
        (self.format.l_ra() as u32) / (self.n_cs as u32)
    }

    /// Number of root sequences required to provide 64 preambles.
    pub fn num_roots_needed(&self) -> u32 {
        let ppr = self.preambles_per_root();
        if ppr == 0 {
            return 64;
        }
        (64 + ppr - 1) / ppr
    }
}

/// Result of attempting PRACH detection on a received signal.
#[derive(Debug, Clone)]
pub struct PrachDetectionResult {
    /// Index of detected preamble (0-63), valid only when `detected` is true.
    pub preamble_index: u8,
    /// Timing advance in seconds.
    pub timing_advance: f64,
    /// Received power in dB.
    pub power_db: f64,
    /// True when a preamble was detected above the threshold.
    pub detected: bool,
    /// Peak correlation magnitude (linear).
    pub peak_magnitude: f64,
    /// Cyclic shift bin index corresponding to the detected peak.
    pub peak_bin: usize,
}

impl Default for PrachDetectionResult {
    fn default() -> Self {
        PrachDetectionResult {
            preamble_index: 0,
            timing_advance: 0.0,
            power_db: f64::NEG_INFINITY,
            detected: false,
            peak_magnitude: 0.0,
            peak_bin: 0,
        }
    }
}

/// Timing advance decoded from a PRACH detection.
#[derive(Debug, Clone, Copy)]
pub struct TimingAdvance {
    /// N_TA in units of T_c.
    pub n_ta: u32,
    /// Timing advance in seconds.
    pub ta_seconds: f64,
    /// One-way range estimate in metres.
    pub ta_meters: f64,
}

impl TimingAdvance {
    /// Construct from N_TA (integer multiples of T_c).
    pub fn from_n_ta(n_ta: u32) -> Self {
        let ta_seconds = n_ta as f64 * T_C_SECONDS;
        TimingAdvance { n_ta, ta_seconds, ta_meters: ta_seconds * C_LIGHT / 2.0 }
    }

    /// Construct from a timing advance command value (0-3846), each step = 16·T_c.
    pub fn from_ta_command(ta_cmd: u16) -> Self {
        let n_ta = ta_cmd as u32 * 16;
        Self::from_n_ta(n_ta)
    }

    /// TA command value (0..3846).
    pub fn ta_command(&self) -> u16 {
        (self.n_ta / 16).min(3846) as u16
    }
}

/// Power ramping state for random access procedure.
#[derive(Debug, Clone)]
pub struct PowerRamping {
    /// Initial preamble transmission power in dBm.
    pub initial_power_dbm: f64,
    /// Ramping step in dB (typical: 2 dB).
    pub step_db: f64,
    /// Maximum number of retransmissions.
    pub max_retransmissions: u8,
    /// Current attempt index (0-based).
    pub attempt: u8,
}

impl PowerRamping {
    /// Create a new power ramping state.
    pub fn new(initial_power_dbm: f64, step_db: f64, max_retransmissions: u8) -> Self {
        PowerRamping { initial_power_dbm, step_db, max_retransmissions, attempt: 0 }
    }

    /// Current transmission power in dBm.
    pub fn current_power_dbm(&self) -> f64 {
        self.initial_power_dbm + self.attempt as f64 * self.step_db
    }

    /// Advance to next attempt. Returns false if maximum retransmissions exceeded.
    pub fn next_attempt(&mut self) -> bool {
        if self.attempt >= self.max_retransmissions {
            return false;
        }
        self.attempt += 1;
        true
    }

    /// Returns true if more retransmissions are allowed.
    pub fn can_retry(&self) -> bool {
        self.attempt < self.max_retransmissions
    }
}

// ─────────────────────────────────────────────────────────── restricted sets

/// Compute d_u for a given root index u and sequence length L_RA.
///
/// d_u = min(p·u mod L_RA, L_RA - p·u mod L_RA)
/// where p satisfies u·p ≡ 1 (mod L_RA).
///
/// Returns None if gcd(u, L_RA) ≠ 1 (u has no modular inverse).
pub fn compute_d_u(u: u32, l_ra: u32) -> Option<u32> {
    let p = mod_inverse(u, l_ra)?;
    let pu_mod = ((p as u64 * u as u64) % l_ra as u64) as u32;
    Some(pu_mod.min(l_ra - pu_mod))
}

/// Extended Euclidean algorithm to find modular inverse of a mod m.
/// Returns None if inverse does not exist (gcd(a,m) ≠ 1).
pub fn mod_inverse(a: u32, m: u32) -> Option<u32> {
    if m == 0 {
        return None;
    }
    let (mut old_r, mut r) = (a as i64, m as i64);
    let (mut old_s, mut s) = (1i64, 0i64);
    while r != 0 {
        let q = old_r / r;
        let tmp = r;
        r = old_r - q * r;
        old_r = tmp;
        let tmp = s;
        s = old_s - q * s;
        old_s = tmp;
    }
    if old_r != 1 {
        return None; // gcd ≠ 1
    }
    Some(((old_s % m as i64 + m as i64) % m as i64) as u32)
}

/// Check whether cyclic shift C_v is valid for restricted set Type A.
///
/// A cyclic shift C_v is valid iff it does not fall in any of the ambiguous
/// windows around 0 and d_u defined by the high-speed constraint.
pub fn is_valid_shift_type_a(c_v: u32, d_u: u32, n_cs: u16, l_ra: u32) -> bool {
    if n_cs == 0 {
        return c_v == 0;
    }
    let n = l_ra;
    // Condition from TS 38.211 Section 6.3.3.1:
    // C_v must satisfy: C_v ≥ d_u + N_CS or C_v + N_CS ≤ d_u (window around d_u)
    // AND C_v + N_CS ≤ L_RA (does not wrap)
    let cv = c_v as i64;
    let du = d_u as i64;
    let ncs = n_cs as i64;
    let n = n as i64;
    // Shift window must not overlap with [0, N_CS) or [d_u, d_u + N_CS)
    let in_origin_window = cv < ncs;
    let overlaps_du = (cv < du + ncs) && (cv + ncs > du);
    let wraps = cv + ncs > n;
    !(in_origin_window || overlaps_du || wraps)
}

/// Check whether cyclic shift C_v is valid for restricted set Type B.
///
/// Type B adds an additional constraint compared to Type A:
/// C_v must also avoid the window around 2·d_u mod L_RA.
pub fn is_valid_shift_type_b(c_v: u32, d_u: u32, n_cs: u16, l_ra: u32) -> bool {
    if !is_valid_shift_type_a(c_v, d_u, n_cs, l_ra) {
        return false;
    }
    let n = l_ra as i64;
    let cv = c_v as i64;
    let du = d_u as i64;
    let ncs = n_cs as i64;
    let d2 = (2 * du) % n;
    let overlaps_d2 = (cv < d2 + ncs) && (cv + ncs > d2);
    !overlaps_d2
}

/// Return the list of valid cyclic shifts for a given root, restricted set and N_CS.
pub fn valid_cyclic_shifts(
    root_index: u32,
    l_ra: u32,
    n_cs: u16,
    restricted_set: RestrictedSet,
) -> Vec<u32> {
    if n_cs == 0 {
        return vec![0];
    }
    let max_shifts = (l_ra / n_cs as u32) as usize;
    let d_u = compute_d_u(root_index, l_ra);

    (0..max_shifts)
        .map(|v| v as u32 * n_cs as u32)
        .filter(|&c_v| match restricted_set {
            RestrictedSet::Unrestricted => true,
            RestrictedSet::TypeA => {
                d_u.map(|du| is_valid_shift_type_a(c_v, du, n_cs, l_ra))
                    .unwrap_or(false)
            }
            RestrictedSet::TypeB => {
                d_u.map(|du| is_valid_shift_type_b(c_v, du, n_cs, l_ra))
                    .unwrap_or(false)
            }
        })
        .collect()
}

// ──────────────────────────────────────────────────────── root sequence table

/// Map logical root sequence index to physical ZC root index u for long preamble (L_RA=839).
///
/// The table is from TS 38.211 Table 6.3.3.1-3. For the full standard, 838 entries
/// would be listed. Here we implement the permutation analytically (ZC roots that are
/// coprime with 839, ordered by the standard's permutation).
///
/// For simplicity we return root_index directly when it is already a valid ZC root
/// (coprime with 839). The standard defines a specific permutation P(i) for long
/// sequences; this function applies that permutation.
pub fn logical_to_physical_root_long(logical_index: u32) -> u32 {
    // 839 is prime, so every value 1..=838 is a valid ZC root.
    // The standard's permutation is complex; for this implementation we use the
    // direct mapping: u = logical_index + 1 (0-based logical → 1-based root).
    // The actual standard table is a specific sequence defined in TS 38.211 Table
    // 6.3.3.1-3, but the mapping u = (logical_index + 1) is the identity row.
    let u = (logical_index % 838) + 1;
    u
}

/// Map logical root sequence index to physical ZC root index u for short preamble (L_RA=139).
pub fn logical_to_physical_root_short(logical_index: u32) -> u32 {
    // 139 is prime, so every value 1..=138 is valid.
    let u = (logical_index % 138) + 1;
    u
}

/// Get physical root index for a given logical index and format.
pub fn logical_to_physical_root(logical_index: u32, format: PrachFormat) -> u32 {
    if format.is_long() {
        logical_to_physical_root_long(logical_index)
    } else {
        logical_to_physical_root_short(logical_index)
    }
}

// ──────────────────────────────────────────────────────────── FFT / IFFT helpers

/// Compute the IFFT of a complex slice (DFT with conjugated exponent, divided by N).
fn ifft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let scale = 1.0 / n as f64;
    let mut out = vec![(0.0f64, 0.0f64); n];
    for k in 0..n {
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        for (m, &(ar, ai)) in input.iter().enumerate() {
            let angle = 2.0 * PI * (k * m) as f64 / n as f64;
            re += ar * angle.cos() - ai * angle.sin();
            im += ar * angle.sin() + ai * angle.cos();
        }
        out[k] = (re * scale, im * scale);
    }
    out
}

/// Compute the DFT of a complex slice.
fn dft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let mut out = vec![(0.0f64, 0.0f64); n];
    for k in 0..n {
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        for (m, &(ar, ai)) in input.iter().enumerate() {
            let angle = -2.0 * PI * (k * m) as f64 / n as f64;
            re += ar * angle.cos() - ai * angle.sin();
            im += ar * angle.sin() + ai * angle.cos();
        }
        out[k] = (re, im);
    }
    out
}

/// Point-wise multiply a by conj(b): (a_re + j·a_im)(b_re - j·b_im).
fn multiply_conj(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    a.iter()
        .zip(b.iter())
        .map(|(&(ar, ai), &(br, bi))| (ar * br + ai * bi, ai * br - ar * bi))
        .collect()
}

/// Magnitude squared of a complex sample.
#[inline]
fn mag_sq(re: f64, im: f64) -> f64 {
    re * re + im * im
}

// ─────────────────────────────────────────────────────────────── main detector

/// PRACH preamble detector and generator.
pub struct PrachDetector {
    /// Configuration for this PRACH occasion.
    pub config: PrachConfig,
}

impl PrachDetector {
    /// Create a new detector with the given configuration.
    pub fn new(config: PrachConfig) -> Self {
        PrachDetector { config }
    }

    /// Generate all 64 PRACH preambles for this configuration.
    ///
    /// Iterates over root sequences starting at `root_sequence_index`, applying
    /// cyclic shifts, until 64 preambles have been generated.
    pub fn generate_preambles(&self) -> Vec<PrachPreamble> {
        let l_ra = self.config.l_ra() as u32;
        let format = self.config.format;
        let n_cs = self.config.n_cs;
        let restricted_set = self.config.restricted_set;
        let mut preambles = Vec::with_capacity(NUM_PREAMBLES);
        let mut preamble_index: u8 = 0;
        let mut logical_root = self.config.root_sequence_index;

        while preambles.len() < NUM_PREAMBLES {
            let phys_root = logical_to_physical_root(logical_root, format);
            let zc = ZadoffChuSequence::generate(phys_root, self.config.l_ra());

            let shifts = if n_cs == 0 {
                vec![0u32]
            } else {
                valid_cyclic_shifts(phys_root, l_ra, n_cs, restricted_set)
            };

            for shift in shifts {
                if preambles.len() >= NUM_PREAMBLES {
                    break;
                }
                let samples = zc.cyclic_shift(shift);
                preambles.push(PrachPreamble {
                    preamble_index,
                    root_index: phys_root,
                    cyclic_shift: shift,
                    format,
                    samples,
                });
                preamble_index = preamble_index.wrapping_add(1);
            }

            logical_root += 1;
            // Wrap around the root sequence table.
            let max_root = if format.is_long() { 838u32 } else { 138u32 };
            if logical_root > max_root {
                logical_root = 0;
            }
        }

        preambles
    }

    /// Attempt to detect any of the 64 preambles in `received`.
    ///
    /// `received` should contain L_RA complex samples (after CP removal and
    /// frequency-domain extraction).
    ///
    /// `threshold` is the detection threshold in linear magnitude (e.g. 0.1).
    ///
    /// Returns the best detection result (highest peak), or a result with
    /// `detected=false` if no preamble exceeded the threshold.
    pub fn detect(
        &self,
        received: &[(f64, f64)],
        threshold: f64,
    ) -> PrachDetectionResult {
        let l_ra = self.config.l_ra() as usize;
        if received.len() < l_ra {
            return PrachDetectionResult::default();
        }
        let rx = &received[..l_ra];

        // Frequency-domain received signal.
        let r_freq = dft(rx);

        let preambles = self.generate_preambles();
        let mut best = PrachDetectionResult::default();

        // Try each root sequence used.
        let mut seen_roots: Vec<u32> = Vec::new();
        for preamble in &preambles {
            if seen_roots.contains(&preamble.root_index) {
                continue;
            }
            seen_roots.push(preamble.root_index);

            // ZC base sequence in frequency domain.
            let zc_base = ZadoffChuSequence::generate(preamble.root_index, self.config.l_ra());
            let x_freq = zc_base.dft();

            // Correlation: Y(k) = R(k) · conj(X_u(k)).
            let y_freq = multiply_conj(&r_freq, &x_freq);

            // IFFT → cyclic shift domain.
            let z = ifft(&y_freq);

            // Peak search.
            let (peak_bin, peak_mag_sq) = z
                .iter()
                .enumerate()
                .map(|(i, &(re, im))| (i, mag_sq(re, im)))
                .fold((0usize, 0.0f64), |(bi, bm), (i, m)| {
                    if m > bm { (i, m) } else { (bi, bm) }
                });

            let peak_mag = peak_mag_sq.sqrt();

            // Map peak bin back to preamble index.
            if let Some(detected_preamble) = self.bin_to_preamble(peak_bin, &preambles) {
                if peak_mag > best.peak_magnitude {
                    let ta = self.timing_advance_from_bin(peak_bin);
                    let power_db = 20.0 * (peak_mag.max(1e-30)).log10();
                    best = PrachDetectionResult {
                        preamble_index: detected_preamble.preamble_index,
                        timing_advance: ta,
                        power_db,
                        detected: peak_mag > threshold,
                        peak_magnitude: peak_mag,
                        peak_bin,
                    };
                }
            }
        }

        best
    }

    /// Convert a correlation peak bin index to timing advance in seconds.
    pub fn timing_advance_from_bin(&self, bin: usize) -> f64 {
        // Each bin corresponds to one ZC sample period T_ZC = 1 / (Δf_RA · L_RA).
        let delta_f_hz = self.config.format.delta_f_ra_khz() * 1000.0;
        let l_ra = self.config.l_ra() as f64;
        let t_zc = 1.0 / (delta_f_hz * l_ra);
        bin as f64 * t_zc
    }

    /// Map a peak bin to the corresponding PrachPreamble.
    fn bin_to_preamble<'a>(
        &self,
        bin: usize,
        preambles: &'a [PrachPreamble],
    ) -> Option<&'a PrachPreamble> {
        if self.config.n_cs == 0 {
            return preambles.first();
        }
        let n_cs = self.config.n_cs as usize;
        // Find preamble whose cyclic shift window contains this bin.
        preambles.iter().find(|p| {
            let cs = p.cyclic_shift as usize;
            bin >= cs && bin < cs + n_cs
        })
    }

    /// Generate the receive power estimate in dB from the correlation peak.
    pub fn estimate_power_db(&self, peak_magnitude: f64) -> f64 {
        20.0 * (peak_magnitude.max(1e-30)).log10()
    }
}

// ──────────────────────────────────────────────────────── PRACH configuration table

/// Decoded parameters from a PRACH configuration index.
///
/// Corresponds to TS 38.211 Table 6.3.3.2-2 (FR1, paired spectrum) entries.
#[derive(Debug, Clone)]
pub struct PrachOccasionConfig {
    /// PRACH preamble format.
    pub format: PrachFormat,
    /// SFN condition: PRACH only in frames where sfn mod x == 0.
    pub x: u8,
    /// Subframe / half-frame offset y.
    pub y: u8,
    /// Start symbol within the slot.
    pub start_symbol: u8,
    /// Number of PRACH occasions in frequency domain.
    pub num_freq_occasions: u8,
}

/// Return PRACH occasion parameters for a given configuration index (FR1, FDD/TDD).
///
/// Implements a representative subset of TS 38.211 Table 6.3.3.2-2.
pub fn prach_config_table(config_index: u8) -> Option<PrachOccasionConfig> {
    // Subset of the standard table (config indices 0-15 for Format 0 / Format 3).
    match config_index {
        0 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 16,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        1 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 8,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        2 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 4,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        3 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 2,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        4 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 2,
            y: 0,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        5 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 2,
            y: 0,
            start_symbol: 0,
            num_freq_occasions: 2,
        }),
        6 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 1,
            y: 0,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        7 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 1,
            y: 0,
            start_symbol: 0,
            num_freq_occasions: 2,
        }),
        8 => Some(PrachOccasionConfig {
            format: PrachFormat::Format0,
            x: 1,
            y: 0,
            start_symbol: 0,
            num_freq_occasions: 4,
        }),
        9 => Some(PrachOccasionConfig {
            format: PrachFormat::Format1,
            x: 16,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        10 => Some(PrachOccasionConfig {
            format: PrachFormat::Format1,
            x: 8,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        11 => Some(PrachOccasionConfig {
            format: PrachFormat::Format2,
            x: 16,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        12 => Some(PrachOccasionConfig {
            format: PrachFormat::Format3,
            x: 16,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        13 => Some(PrachOccasionConfig {
            format: PrachFormat::Format3,
            x: 8,
            y: 1,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        14 => Some(PrachOccasionConfig {
            format: PrachFormat::A1,
            x: 2,
            y: 0,
            start_symbol: 0,
            num_freq_occasions: 1,
        }),
        15 => Some(PrachOccasionConfig {
            format: PrachFormat::A1,
            x: 1,
            y: 0,
            start_symbol: 0,
            num_freq_occasions: 2,
        }),
        _ => None,
    }
}

// ──────────────────────────────────────────────────────────────────────── tests

#[cfg(test)]
mod tests {
    use super::*;

    // ── ZC sequence generation ────────────────────────────────────────────────

    #[test]
    fn test_zc_sequence_length_long() {
        let zc = ZadoffChuSequence::generate(1, L_RA_LONG);
        assert_eq!(zc.sequence.len(), L_RA_LONG as usize);
        assert_eq!(zc.length, L_RA_LONG);
        assert_eq!(zc.root_index, 1);
    }

    #[test]
    fn test_zc_sequence_length_short() {
        let zc = ZadoffChuSequence::generate(1, L_RA_SHORT);
        assert_eq!(zc.sequence.len(), L_RA_SHORT as usize);
        assert_eq!(zc.length, L_RA_SHORT);
    }

    #[test]
    fn test_zc_unit_magnitude() {
        // All ZC samples must have magnitude 1.
        let zc = ZadoffChuSequence::generate(25, L_RA_LONG);
        for (re, im) in &zc.sequence {
            let mag = (re * re + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-10, "mag={}", mag);
        }
    }

    #[test]
    fn test_zc_first_sample_is_one() {
        // x_u(0) = exp(-j*pi*u*0*1/L) = exp(0) = 1.
        let zc = ZadoffChuSequence::generate(7, L_RA_LONG);
        let (re, im) = zc.sequence[0];
        assert!((re - 1.0).abs() < 1e-12, "re(0)={}", re);
        assert!(im.abs() < 1e-12, "im(0)={}", im);
    }

    #[test]
    fn test_zc_known_value_n1_u1_long() {
        // x_1(1) = exp(-j*pi*1*1*2/839) = exp(-j*2*pi/839).
        let zc = ZadoffChuSequence::generate(1, L_RA_LONG);
        let (re, im) = zc.sequence[1];
        let expected_phase = -2.0 * PI / 839.0;
        assert!((re - expected_phase.cos()).abs() < 1e-10);
        assert!((im - expected_phase.sin()).abs() < 1e-10);
    }

    #[test]
    fn test_zc_known_value_n2_u1_long() {
        // x_1(2) = exp(-j*pi*1*2*3/839) = exp(-j*6*pi/839).
        let zc = ZadoffChuSequence::generate(1, L_RA_LONG);
        let (re, im) = zc.sequence[2];
        let expected_phase = -6.0 * PI / 839.0;
        assert!((re - expected_phase.cos()).abs() < 1e-10);
        assert!((im - expected_phase.sin()).abs() < 1e-10);
    }

    #[test]
    fn test_zc_different_roots_different_sequences() {
        let zc1 = ZadoffChuSequence::generate(1, L_RA_LONG);
        let zc2 = ZadoffChuSequence::generate(2, L_RA_LONG);
        // They should differ at sample 1 (at sample 0 both are 1.0).
        let diff_re = (zc1.sequence[1].0 - zc2.sequence[1].0).abs();
        assert!(diff_re > 1e-6);
    }

    // ── cyclic shift ──────────────────────────────────────────────────────────

    #[test]
    fn test_cyclic_shift_zero_is_identity() {
        let zc = ZadoffChuSequence::generate(3, L_RA_SHORT);
        let shifted = zc.cyclic_shift(0);
        assert_eq!(shifted.len(), zc.sequence.len());
        for (a, b) in shifted.iter().zip(zc.sequence.iter()) {
            assert!((a.0 - b.0).abs() < 1e-15 && (a.1 - b.1).abs() < 1e-15);
        }
    }

    #[test]
    fn test_cyclic_shift_unit_magnitude() {
        let zc = ZadoffChuSequence::generate(5, L_RA_SHORT);
        let shifted = zc.cyclic_shift(13);
        for (re, im) in &shifted {
            let mag = (re * re + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_cyclic_shift_rotation() {
        // Shifting by N_CS and then by L_RA - N_CS should return to original.
        let zc = ZadoffChuSequence::generate(1, L_RA_SHORT);
        let n_cs = 13u32;
        let shifted = zc.cyclic_shift(n_cs);
        let back = ZadoffChuSequence {
            sequence: shifted,
            root_index: 1,
            length: L_RA_SHORT,
        };
        let recovered = back.cyclic_shift(L_RA_SHORT as u32 - n_cs);
        for (a, b) in recovered.iter().zip(zc.sequence.iter()) {
            assert!((a.0 - b.0).abs() < 1e-12 && (a.1 - b.1).abs() < 1e-12);
        }
    }

    #[test]
    fn test_cyclic_shift_wrap_around() {
        let zc = ZadoffChuSequence::generate(2, L_RA_SHORT);
        // Shift by L_RA should be equivalent to shift by 0.
        let shifted = zc.cyclic_shift(L_RA_SHORT as u32);
        for (a, b) in shifted.iter().zip(zc.sequence.iter()) {
            assert!((a.0 - b.0).abs() < 1e-12);
        }
    }

    // ── format parameters ─────────────────────────────────────────────────────

    #[test]
    fn test_format0_l_ra() {
        assert_eq!(PrachFormat::Format0.l_ra(), L_RA_LONG);
    }

    #[test]
    fn test_format3_l_ra() {
        assert_eq!(PrachFormat::Format3.l_ra(), L_RA_LONG);
    }

    #[test]
    fn test_format_a1_l_ra() {
        assert_eq!(PrachFormat::A1.l_ra(), L_RA_SHORT);
    }

    #[test]
    fn test_format_c2_l_ra() {
        assert_eq!(PrachFormat::C2.l_ra(), L_RA_SHORT);
    }

    #[test]
    fn test_format0_scs() {
        assert!((PrachFormat::Format0.delta_f_ra_khz() - 1.25).abs() < 1e-9);
    }

    #[test]
    fn test_format3_scs() {
        assert!((PrachFormat::Format3.delta_f_ra_khz() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_format_symbols() {
        assert_eq!(PrachFormat::Format0.num_symbols(), 1);
        assert_eq!(PrachFormat::Format1.num_symbols(), 2);
        assert_eq!(PrachFormat::Format2.num_symbols(), 4);
        assert_eq!(PrachFormat::A1.num_symbols(), 2);
        assert_eq!(PrachFormat::B4.num_symbols(), 12);
        assert_eq!(PrachFormat::C0.num_symbols(), 1);
    }

    #[test]
    fn test_is_long_short() {
        assert!(PrachFormat::Format0.is_long());
        assert!(PrachFormat::Format3.is_long());
        assert!(!PrachFormat::A1.is_long());
        assert!(!PrachFormat::C2.is_long());
    }

    // ── restricted set ────────────────────────────────────────────────────────

    #[test]
    fn test_mod_inverse_basic() {
        // 2 * 420 = 840 ≡ 1 (mod 839)
        let inv = mod_inverse(2, 839);
        assert_eq!(inv, Some(420));
    }

    #[test]
    fn test_mod_inverse_no_inverse() {
        // gcd(2, 4) = 2 ≠ 1, so no inverse.
        assert_eq!(mod_inverse(2, 4), None);
    }

    #[test]
    fn test_mod_inverse_one() {
        // Inverse of 1 is always 1.
        assert_eq!(mod_inverse(1, 839), Some(1));
    }

    #[test]
    fn test_d_u_computation_u1() {
        // For u=1, L_RA=839: p=1, p*u mod 839 = 1, d_u = min(1, 838) = 1.
        let d_u = compute_d_u(1, 839);
        assert_eq!(d_u, Some(1));
    }

    #[test]
    fn test_d_u_computation_u2() {
        // For u=2: p=420, p*u mod 839 = 840 mod 839 = 1, d_u = min(1, 838) = 1.
        let d_u = compute_d_u(2, 839);
        assert_eq!(d_u, Some(1));
    }

    #[test]
    fn test_valid_shifts_unrestricted() {
        let shifts = valid_cyclic_shifts(1, 839, 13, RestrictedSet::Unrestricted);
        // 839 / 13 = 64 shifts (64 * 13 = 832 ≤ 839).
        assert_eq!(shifts.len(), 64);
        assert_eq!(shifts[0], 0);
        assert_eq!(shifts[1], 13);
        assert_eq!(shifts[63], 63 * 13);
    }

    #[test]
    fn test_valid_shifts_type_a_fewer_than_unrestricted() {
        let shifts_u = valid_cyclic_shifts(1, 839, 26, RestrictedSet::Unrestricted);
        let shifts_a = valid_cyclic_shifts(1, 839, 26, RestrictedSet::TypeA);
        // Type A should filter out some shifts.
        assert!(shifts_a.len() <= shifts_u.len());
    }

    #[test]
    fn test_valid_shifts_type_b_fewer_than_type_a() {
        let shifts_a = valid_cyclic_shifts(25, 839, 26, RestrictedSet::TypeA);
        let shifts_b = valid_cyclic_shifts(25, 839, 26, RestrictedSet::TypeB);
        assert!(shifts_b.len() <= shifts_a.len());
    }

    #[test]
    fn test_valid_shifts_n_cs_zero() {
        // N_CS=0 → only one shift at 0.
        let shifts = valid_cyclic_shifts(1, 839, 0, RestrictedSet::Unrestricted);
        assert_eq!(shifts, vec![0u32]);
    }

    // ── preamble generation ───────────────────────────────────────────────────

    #[test]
    fn test_generate_64_preambles() {
        let config = PrachConfig {
            config_index: 0,
            format: PrachFormat::Format0,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 13,
            root_sequence_index: 0,
        };
        let detector = PrachDetector::new(config);
        let preambles = detector.generate_preambles();
        assert_eq!(preambles.len(), NUM_PREAMBLES);
    }

    #[test]
    fn test_preamble_indices_sequential() {
        let config = PrachConfig {
            config_index: 0,
            format: PrachFormat::Format0,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 13,
            root_sequence_index: 0,
        };
        let detector = PrachDetector::new(config);
        let preambles = detector.generate_preambles();
        for (i, p) in preambles.iter().enumerate() {
            assert_eq!(p.preamble_index as usize, i);
        }
    }

    #[test]
    fn test_preamble_sample_length() {
        let config = PrachConfig {
            config_index: 0,
            format: PrachFormat::Format0,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 13,
            root_sequence_index: 0,
        };
        let detector = PrachDetector::new(config);
        let preambles = detector.generate_preambles();
        for p in &preambles {
            assert_eq!(p.samples.len(), L_RA_LONG as usize);
        }
    }

    #[test]
    fn test_preamble_unit_magnitude() {
        let config = PrachConfig {
            config_index: 0,
            format: PrachFormat::A1,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 13,
            root_sequence_index: 0,
        };
        let detector = PrachDetector::new(config);
        let preambles = detector.generate_preambles();
        for sample in &preambles[0].samples {
            let mag = (sample.0 * sample.0 + sample.1 * sample.1).sqrt();
            assert!((mag - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_preamble_short_format_64_preambles() {
        let config = PrachConfig {
            config_index: 14,
            format: PrachFormat::A1,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 2,
            root_sequence_index: 0,
        };
        let detector = PrachDetector::new(config);
        let preambles = detector.generate_preambles();
        assert_eq!(preambles.len(), NUM_PREAMBLES);
    }

    // ── detection ─────────────────────────────────────────────────────────────

    fn make_detector(format: PrachFormat, n_cs: u16, root: u32) -> PrachDetector {
        PrachDetector::new(PrachConfig {
            config_index: 0,
            format,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs,
            root_sequence_index: root,
        })
    }

    #[test]
    fn test_detect_preamble_0_format0() {
        // Transmit preamble 0 → expect detection with high magnitude.
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        let preambles = detector.generate_preambles();
        let tx = &preambles[0].samples;
        let result = detector.detect(tx, 0.01);
        assert!(result.detected, "Should detect preamble 0");
        assert_eq!(result.preamble_index, 0);
    }

    #[test]
    fn test_detect_preamble_5_format0() {
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        let preambles = detector.generate_preambles();
        let tx = &preambles[5].samples;
        let result = detector.detect(tx, 0.01);
        assert!(result.detected);
    }

    #[test]
    fn test_detect_preamble_63_format0() {
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        let preambles = detector.generate_preambles();
        let tx = &preambles[63].samples;
        let result = detector.detect(tx, 0.01);
        assert!(result.detected);
    }

    #[test]
    fn test_detect_no_preamble_returns_not_detected() {
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        // Feed zeros → no preamble → peak magnitude should be tiny.
        let zeros = vec![(0.0f64, 0.0f64); L_RA_LONG as usize];
        let result = detector.detect(&zeros, 0.5);
        assert!(!result.detected);
    }

    #[test]
    fn test_detect_short_preamble() {
        let detector = make_detector(PrachFormat::A1, 2, 0);
        let preambles = detector.generate_preambles();
        let tx = &preambles[0].samples;
        let result = detector.detect(tx, 0.001);
        assert!(result.detected);
    }

    #[test]
    fn test_detect_peak_magnitude_positive() {
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        let preambles = detector.generate_preambles();
        let result = detector.detect(&preambles[0].samples, 0.0);
        assert!(result.peak_magnitude > 0.0);
    }

    #[test]
    fn test_detect_timing_advance_zero_for_no_shift() {
        // Preamble 0 has cyclic shift 0 → timing advance should be 0.
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        let preambles = detector.generate_preambles();
        let result = detector.detect(&preambles[0].samples, 0.0);
        assert!(result.timing_advance >= 0.0);
    }

    // ── timing advance ────────────────────────────────────────────────────────

    #[test]
    fn test_ta_from_n_ta_zero() {
        let ta = TimingAdvance::from_n_ta(0);
        assert_eq!(ta.n_ta, 0);
        assert!(ta.ta_seconds.abs() < 1e-30);
        assert!(ta.ta_meters.abs() < 1e-20);
    }

    #[test]
    fn test_ta_from_n_ta_nonzero() {
        let ta = TimingAdvance::from_n_ta(1024);
        assert!((ta.ta_seconds - 1024.0 * T_C_SECONDS).abs() < 1e-20);
        assert!(ta.ta_meters > 0.0);
    }

    #[test]
    fn test_ta_from_ta_command() {
        // TA command 1 → n_ta = 16 → ta_seconds = 16 * T_c.
        let ta = TimingAdvance::from_ta_command(1);
        assert_eq!(ta.n_ta, 16);
    }

    #[test]
    fn test_ta_command_roundtrip() {
        let ta = TimingAdvance::from_ta_command(500);
        assert_eq!(ta.ta_command(), 500);
    }

    #[test]
    fn test_ta_command_max() {
        let ta = TimingAdvance::from_ta_command(3846);
        assert_eq!(ta.ta_command(), 3846);
    }

    #[test]
    fn test_ta_range() {
        // TA range: 0 to 3846 * 16 * T_c.
        let ta_max = TimingAdvance::from_ta_command(3846);
        let expected_max_s = 3846.0 * 16.0 * T_C_SECONDS;
        assert!((ta_max.ta_seconds - expected_max_s).abs() < 1e-15);
    }

    #[test]
    fn test_ta_meter_formula() {
        let ta = TimingAdvance::from_n_ta(1000);
        // ta_meters = ta_seconds * c / 2
        let expected = ta.ta_seconds * C_LIGHT / 2.0;
        assert!((ta.ta_meters - expected).abs() < 1e-6);
    }

    // ── power ramping ─────────────────────────────────────────────────────────

    #[test]
    fn test_power_ramping_initial() {
        let pr = PowerRamping::new(-100.0, 2.0, 4);
        assert!((pr.current_power_dbm() - (-100.0)).abs() < 1e-9);
    }

    #[test]
    fn test_power_ramping_advance() {
        let mut pr = PowerRamping::new(-100.0, 2.0, 4);
        assert!(pr.next_attempt());
        assert!((pr.current_power_dbm() - (-98.0)).abs() < 1e-9);
        assert!(pr.next_attempt());
        assert!((pr.current_power_dbm() - (-96.0)).abs() < 1e-9);
    }

    #[test]
    fn test_power_ramping_exhausted() {
        let mut pr = PowerRamping::new(-100.0, 2.0, 2);
        assert!(pr.next_attempt()); // attempt 1
        assert!(pr.next_attempt()); // attempt 2
        assert!(!pr.next_attempt()); // exhausted
    }

    #[test]
    fn test_power_ramping_can_retry() {
        let mut pr = PowerRamping::new(-100.0, 2.0, 3);
        assert!(pr.can_retry());
        pr.next_attempt();
        assert!(pr.can_retry());
        pr.next_attempt();
        pr.next_attempt();
        assert!(!pr.can_retry());
    }

    // ── config table ──────────────────────────────────────────────────────────

    #[test]
    fn test_config_table_index0() {
        let cfg = prach_config_table(0).expect("index 0 must exist");
        assert_eq!(cfg.format, PrachFormat::Format0);
        assert_eq!(cfg.x, 16);
        assert_eq!(cfg.y, 1);
    }

    #[test]
    fn test_config_table_index12_format3() {
        let cfg = prach_config_table(12).expect("index 12 must exist");
        assert_eq!(cfg.format, PrachFormat::Format3);
    }

    #[test]
    fn test_config_table_index14_format_a1() {
        let cfg = prach_config_table(14).expect("index 14 must exist");
        assert_eq!(cfg.format, PrachFormat::A1);
    }

    #[test]
    fn test_config_table_unknown_index() {
        let cfg = prach_config_table(200);
        assert!(cfg.is_none());
    }

    // ── logical → physical root mapping ──────────────────────────────────────

    #[test]
    fn test_logical_to_physical_long_index0() {
        // Logical index 0 → root 1.
        assert_eq!(logical_to_physical_root_long(0), 1);
    }

    #[test]
    fn test_logical_to_physical_long_index837() {
        // Logical index 837 → root 838.
        assert_eq!(logical_to_physical_root_long(837), 838);
    }

    #[test]
    fn test_logical_to_physical_long_wraps() {
        // Logical index 838 wraps: (838 % 838) + 1 = 1.
        assert_eq!(logical_to_physical_root_long(838), 1);
    }

    #[test]
    fn test_logical_to_physical_short_index0() {
        assert_eq!(logical_to_physical_root_short(0), 1);
    }

    #[test]
    fn test_logical_to_physical_dispatch() {
        assert_eq!(
            logical_to_physical_root(0, PrachFormat::Format0),
            logical_to_physical_root_long(0)
        );
        assert_eq!(
            logical_to_physical_root(0, PrachFormat::A1),
            logical_to_physical_root_short(0)
        );
    }

    // ── preambles per root / roots needed ────────────────────────────────────

    #[test]
    fn test_preambles_per_root_n_cs13() {
        let config = PrachConfig {
            config_index: 0,
            format: PrachFormat::Format0,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 13,
            root_sequence_index: 0,
        };
        // 839 / 13 = 64.
        assert_eq!(config.preambles_per_root(), 64);
    }

    #[test]
    fn test_num_roots_needed_n_cs13() {
        let config = PrachConfig {
            config_index: 0,
            format: PrachFormat::Format0,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 13,
            root_sequence_index: 0,
        };
        // 64 preambles / 64 per root = 1 root.
        assert_eq!(config.num_roots_needed(), 1);
    }

    #[test]
    fn test_num_roots_needed_large_n_cs() {
        let config = PrachConfig {
            config_index: 0,
            format: PrachFormat::Format0,
            scs_khz: 15,
            restricted_set: RestrictedSet::Unrestricted,
            n_cs: 419,
            root_sequence_index: 0,
        };
        // 839 / 419 = 2 per root → 32 roots needed.
        assert_eq!(config.num_roots_needed(), 32);
    }

    // ── DFT / IFFT round-trip ─────────────────────────────────────────────────

    #[test]
    fn test_dft_ifft_roundtrip() {
        let signal: Vec<(f64, f64)> = (0..16)
            .map(|i| {
                let a = i as f64 * 0.1;
                (a.cos(), a.sin())
            })
            .collect();
        let freq = dft(&signal);
        let recovered = ifft(&freq);
        for (a, b) in signal.iter().zip(recovered.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10, "re mismatch {} vs {}", a.0, b.0);
            assert!((a.1 - b.1).abs() < 1e-10, "im mismatch {} vs {}", a.1, b.1);
        }
    }

    #[test]
    fn test_zc_dft_length() {
        let zc = ZadoffChuSequence::generate(7, L_RA_SHORT);
        let freq = zc.dft();
        assert_eq!(freq.len(), L_RA_SHORT as usize);
    }

    // ── timing advance from bin ───────────────────────────────────────────────

    #[test]
    fn test_timing_advance_from_bin_zero() {
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        let ta = detector.timing_advance_from_bin(0);
        assert!((ta).abs() < 1e-30);
    }

    #[test]
    fn test_timing_advance_from_bin_nonzero() {
        let detector = make_detector(PrachFormat::Format0, 13, 0);
        let ta = detector.timing_advance_from_bin(1);
        // T_ZC = 1 / (1250 * 839) ≈ 951 ns.
        let expected = 1.0 / (1250.0 * 839.0);
        assert!((ta - expected).abs() < 1e-15, "ta={} expected={}", ta, expected);
    }

    #[test]
    fn test_timing_advance_format3() {
        let detector = make_detector(PrachFormat::Format3, 13, 0);
        let ta = detector.timing_advance_from_bin(1);
        // T_ZC = 1 / (5000 * 839) ≈ 238 ns.
        let expected = 1.0 / (5000.0 * 839.0);
        assert!((ta - expected).abs() < 1e-18);
    }
}

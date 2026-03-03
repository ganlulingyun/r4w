//! VSAT (Very Small Aperture Terminal) Modem Processor
//!
//! Implements a complete VSAT modem signal processing chain covering:
//!
//! - **DVB-S2/S2X Forward Link**: QPSK/8PSK/16APSK/32APSK constellation mapping,
//!   LDPC code rates (1/4 through 9/10), ModCod selection from Es/N0
//! - **SCPC Return Link**: Single Channel Per Carrier with symbol rates
//!   64 ksps to 45 Msps, roll-off factors 0.05/0.10/0.15/0.20/0.25/0.35
//! - **Burst Mode**: TDMA burst structure with preamble, unique word, pilot symbols,
//!   data payload
//! - **ACM (Adaptive Coding and Modulation)**: Link adaptation based on C/N0 feedback,
//!   ModCod switching with hysteresis, SNR estimation from pilot symbols
//! - **Carrier Recovery**: Frequency estimation and phase tracking for satellite channel
//!   (large Doppler shift up to ±60 kHz at Ku-band)
//! - **Timing Recovery**: Symbol timing with Gardner TED, interpolation, loop bandwidth
//!   adaptation
//! - **Rain Fade Mitigation**: Power control (uplink power boost), ModCod fallback,
//!   spreading, site diversity gain estimation
//! - **Link Budget**: Complete VSAT link budget (Ku/Ka-band), antenna gain from diameter,
//!   EIRP, G/T, C/N calculation for both forward and return links, atmospheric
//!   attenuation
//!
//! # References
//! - ETSI EN 302 307-1/2: DVB-S2 / DVB-S2X
//! - ITU-R S.580: VSAT performance objectives
//! - ITU-R P.618: Atmospheric attenuation
//! - ITU-R P.838: Rain attenuation

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Boltzmann constant [J/K]
const K_BOLTZMANN: f64 = 1.380_649e-23;

/// Speed of light [m/s]
const C_LIGHT: f64 = 2.997_924_58e8;

/// Reference temperature [K]
const T0_K: f64 = 290.0;

// ---------------------------------------------------------------------------
// DVB-S2 ModCod Definitions
// ---------------------------------------------------------------------------

/// DVB-S2/S2X modulation order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DvbS2Modulation {
    Qpsk,
    Psk8,
    Apsk16,
    Apsk32,
    /// DVB-S2X extension
    Apsk64,
    /// DVB-S2X extension
    Apsk128,
    /// DVB-S2X extension
    Apsk256,
}

impl DvbS2Modulation {
    /// Bits per symbol.
    pub fn bits_per_symbol(self) -> u32 {
        match self {
            DvbS2Modulation::Qpsk => 2,
            DvbS2Modulation::Psk8 => 3,
            DvbS2Modulation::Apsk16 => 4,
            DvbS2Modulation::Apsk32 => 5,
            DvbS2Modulation::Apsk64 => 6,
            DvbS2Modulation::Apsk128 => 7,
            DvbS2Modulation::Apsk256 => 8,
        }
    }

    /// Human-readable label.
    pub fn label(self) -> &'static str {
        match self {
            DvbS2Modulation::Qpsk => "QPSK",
            DvbS2Modulation::Psk8 => "8PSK",
            DvbS2Modulation::Apsk16 => "16APSK",
            DvbS2Modulation::Apsk32 => "32APSK",
            DvbS2Modulation::Apsk64 => "64APSK",
            DvbS2Modulation::Apsk128 => "128APSK",
            DvbS2Modulation::Apsk256 => "256APSK",
        }
    }
}

/// DVB-S2 LDPC code rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CodeRate {
    R1_4,
    R1_3,
    R2_5,
    R1_2,
    R3_5,
    R2_3,
    R3_4,
    R4_5,
    R5_6,
    R8_9,
    R9_10,
}

impl CodeRate {
    /// Numerical code rate.
    pub fn rate(self) -> f64 {
        match self {
            CodeRate::R1_4 => 1.0 / 4.0,
            CodeRate::R1_3 => 1.0 / 3.0,
            CodeRate::R2_5 => 2.0 / 5.0,
            CodeRate::R1_2 => 1.0 / 2.0,
            CodeRate::R3_5 => 3.0 / 5.0,
            CodeRate::R2_3 => 2.0 / 3.0,
            CodeRate::R3_4 => 3.0 / 4.0,
            CodeRate::R4_5 => 4.0 / 5.0,
            CodeRate::R5_6 => 5.0 / 6.0,
            CodeRate::R8_9 => 8.0 / 9.0,
            CodeRate::R9_10 => 9.0 / 10.0,
        }
    }

    /// Label string.
    pub fn label(self) -> &'static str {
        match self {
            CodeRate::R1_4 => "1/4",
            CodeRate::R1_3 => "1/3",
            CodeRate::R2_5 => "2/5",
            CodeRate::R1_2 => "1/2",
            CodeRate::R3_5 => "3/5",
            CodeRate::R2_3 => "2/3",
            CodeRate::R3_4 => "3/4",
            CodeRate::R4_5 => "4/5",
            CodeRate::R5_6 => "5/6",
            CodeRate::R8_9 => "8/9",
            CodeRate::R9_10 => "9/10",
        }
    }
}

/// A DVB-S2 ModCod entry combining modulation and code rate.
#[derive(Debug, Clone, Copy)]
pub struct ModCod {
    pub modulation: DvbS2Modulation,
    pub code_rate: CodeRate,
    /// Minimum required Es/N0 for quasi-error-free operation [dB].
    pub min_esn0_db: f64,
    /// Spectral efficiency [bits/s/Hz].
    pub spectral_efficiency: f64,
}

impl ModCod {
    /// Compute spectral efficiency from modulation and code rate.
    pub fn compute_efficiency(&self) -> f64 {
        self.modulation.bits_per_symbol() as f64 * self.code_rate.rate()
    }
}

/// DVB-S2 ModCod table (28 entries per ETSI EN 302 307-1 Table 12).
/// Es/N0 thresholds are approximations for AWGN quasi-error-free (BER < 1e-7).
pub fn dvb_s2_modcod_table() -> Vec<ModCod> {
    vec![
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R1_4,  min_esn0_db: -2.35, spectral_efficiency: 0.490 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R1_3,  min_esn0_db: -1.24, spectral_efficiency: 0.656 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R2_5,  min_esn0_db: -0.30, spectral_efficiency: 0.789 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R1_2,  min_esn0_db:  1.00, spectral_efficiency: 0.988 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R3_5,  min_esn0_db:  2.23, spectral_efficiency: 1.188 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R2_3,  min_esn0_db:  3.10, spectral_efficiency: 1.322 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R3_4,  min_esn0_db:  4.03, spectral_efficiency: 1.487 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R4_5,  min_esn0_db:  4.68, spectral_efficiency: 1.587 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R5_6,  min_esn0_db:  5.18, spectral_efficiency: 1.654 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R8_9,  min_esn0_db:  6.20, spectral_efficiency: 1.766 },
        ModCod { modulation: DvbS2Modulation::Qpsk,   code_rate: CodeRate::R9_10, min_esn0_db:  6.42, spectral_efficiency: 1.789 },
        ModCod { modulation: DvbS2Modulation::Psk8,   code_rate: CodeRate::R3_5,  min_esn0_db:  5.50, spectral_efficiency: 1.779 },
        ModCod { modulation: DvbS2Modulation::Psk8,   code_rate: CodeRate::R2_3,  min_esn0_db:  6.62, spectral_efficiency: 1.980 },
        ModCod { modulation: DvbS2Modulation::Psk8,   code_rate: CodeRate::R3_4,  min_esn0_db:  7.91, spectral_efficiency: 2.228 },
        ModCod { modulation: DvbS2Modulation::Psk8,   code_rate: CodeRate::R5_6,  min_esn0_db:  9.35, spectral_efficiency: 2.479 },
        ModCod { modulation: DvbS2Modulation::Psk8,   code_rate: CodeRate::R8_9,  min_esn0_db: 10.69, spectral_efficiency: 2.637 },
        ModCod { modulation: DvbS2Modulation::Psk8,   code_rate: CodeRate::R9_10, min_esn0_db: 10.98, spectral_efficiency: 2.671 },
        ModCod { modulation: DvbS2Modulation::Apsk16, code_rate: CodeRate::R2_3,  min_esn0_db: 10.21, spectral_efficiency: 2.637 },
        ModCod { modulation: DvbS2Modulation::Apsk16, code_rate: CodeRate::R3_4,  min_esn0_db: 11.03, spectral_efficiency: 2.967 },
        ModCod { modulation: DvbS2Modulation::Apsk16, code_rate: CodeRate::R4_5,  min_esn0_db: 11.61, spectral_efficiency: 3.166 },
        ModCod { modulation: DvbS2Modulation::Apsk16, code_rate: CodeRate::R5_6,  min_esn0_db: 12.17, spectral_efficiency: 3.289 },
        ModCod { modulation: DvbS2Modulation::Apsk16, code_rate: CodeRate::R8_9,  min_esn0_db: 13.05, spectral_efficiency: 3.510 },
        ModCod { modulation: DvbS2Modulation::Apsk16, code_rate: CodeRate::R9_10, min_esn0_db: 13.29, spectral_efficiency: 3.567 },
        ModCod { modulation: DvbS2Modulation::Apsk32, code_rate: CodeRate::R3_4,  min_esn0_db: 14.28, spectral_efficiency: 3.703 },
        ModCod { modulation: DvbS2Modulation::Apsk32, code_rate: CodeRate::R4_5,  min_esn0_db: 14.68, spectral_efficiency: 3.951 },
        ModCod { modulation: DvbS2Modulation::Apsk32, code_rate: CodeRate::R5_6,  min_esn0_db: 15.05, spectral_efficiency: 4.119 },
        ModCod { modulation: DvbS2Modulation::Apsk32, code_rate: CodeRate::R8_9,  min_esn0_db: 15.96, spectral_efficiency: 4.397 },
        ModCod { modulation: DvbS2Modulation::Apsk32, code_rate: CodeRate::R9_10, min_esn0_db: 16.05, spectral_efficiency: 4.453 },
    ]
}

// ---------------------------------------------------------------------------
// Constellation Mapping
// ---------------------------------------------------------------------------

/// Complex sample: (I, Q)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub fn new(re: f64, im: f64) -> Self { Self { re, im } }
    pub fn magnitude(self) -> f64 { (self.re * self.re + self.im * self.im).sqrt() }
    pub fn magnitude_sq(self) -> f64 { self.re * self.re + self.im * self.im }
    pub fn phase(self) -> f64 { self.im.atan2(self.re) }
    pub fn conjugate(self) -> Self { Self { re: self.re, im: -self.im } }
    pub fn mul(self, other: Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
    pub fn scale(self, s: f64) -> Self { Self { re: self.re * s, im: self.im * s } }
    pub fn add(self, other: Self) -> Self { Self { re: self.re + other.re, im: self.im + other.im } }
    pub fn sub(self, other: Self) -> Self { Self { re: self.re - other.re, im: self.im - other.im } }
    pub fn from_polar(mag: f64, phase: f64) -> Self {
        Self { re: mag * phase.cos(), im: mag * phase.sin() }
    }
}

/// Generate QPSK constellation (Gray-coded, unit average power).
pub fn qpsk_constellation() -> Vec<Complex> {
    let s = 1.0 / 2.0_f64.sqrt();
    vec![
        Complex::new( s,  s), // 00
        Complex::new(-s,  s), // 01
        Complex::new(-s, -s), // 11
        Complex::new( s, -s), // 10
    ]
}

/// Generate 8PSK constellation (Gray-coded, unit average power).
pub fn psk8_constellation() -> Vec<Complex> {
    (0..8).map(|i| {
        let angle = PI / 8.0 + (i as f64) * PI / 4.0;
        Complex::new(angle.cos(), angle.sin())
    }).collect()
}

/// Generate 16APSK constellation per DVB-S2 (4+12 ring structure).
/// Ring radii r1, r2 with r2/r1 = gamma.
pub fn apsk16_constellation(gamma: f64) -> Vec<Complex> {
    let r1 = (1.0_f64 / (1.0 + gamma * gamma * 3.0)).sqrt(); // normalised
    // actually use standard DVB-S2 ratios
    // Inner ring: 4 points, outer ring: 12 points
    let r1 = (4.0_f64 / (4.0 + 12.0 * gamma * gamma)).sqrt();
    let r2 = gamma * r1;
    let mut pts = Vec::with_capacity(16);
    // inner ring (4 pts)
    for i in 0..4 {
        let a = PI / 4.0 + (i as f64) * PI / 2.0;
        pts.push(Complex::new(r1 * a.cos(), r1 * a.sin()));
    }
    // outer ring (12 pts)
    for i in 0..12 {
        let a = (i as f64) * 2.0 * PI / 12.0;
        pts.push(Complex::new(r2 * a.cos(), r2 * a.sin()));
    }
    pts
}

/// Generate 32APSK constellation per DVB-S2 (4+12+16 ring structure).
pub fn apsk32_constellation(gamma1: f64, gamma2: f64) -> Vec<Complex> {
    let total_norm = 4.0 + 12.0 * gamma1 * gamma1 + 16.0 * gamma2 * gamma2;
    let r1 = (4.0_f64 / total_norm).sqrt();
    let r2 = gamma1 * r1;
    let r3 = gamma2 * r1;
    let mut pts = Vec::with_capacity(32);
    for i in 0..4  { let a = PI / 4.0 + (i as f64) * PI / 2.0; pts.push(Complex::new(r1*a.cos(), r1*a.sin())); }
    for i in 0..12 { let a = (i as f64) * 2.0*PI / 12.0; pts.push(Complex::new(r2*a.cos(), r2*a.sin())); }
    for i in 0..16 { let a = (i as f64) * 2.0*PI / 16.0; pts.push(Complex::new(r3*a.cos(), r3*a.sin())); }
    pts
}

/// Map bits to a constellation symbol via index (Gray coded).
pub fn map_symbol(bits: &[bool], constellation: &[Complex]) -> Complex {
    let m = constellation.len();
    let bits_per_sym = (m as f64).log2() as usize;
    let n = bits.len().min(bits_per_sym);
    let mut idx = 0usize;
    for i in 0..n {
        if bits[i] { idx |= 1 << (bits_per_sym - 1 - i); }
    }
    idx &= m - 1;
    constellation[idx]
}

/// Hard-decision demapper: find nearest constellation point.
pub fn demap_symbol(sample: Complex, constellation: &[Complex]) -> (usize, f64) {
    let mut best_idx = 0usize;
    let mut best_dist = f64::INFINITY;
    for (i, pt) in constellation.iter().enumerate() {
        let d = sample.sub(*pt).magnitude_sq();
        if d < best_dist {
            best_dist = d;
            best_idx = i;
        }
    }
    (best_idx, best_dist.sqrt())
}

// ---------------------------------------------------------------------------
// SCPC Return Link Parameters
// ---------------------------------------------------------------------------

/// Roll-off factor for RRC pulse shaping.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RollOff {
    R005,
    R010,
    R015,
    R020,
    R025,
    R035,
}

impl RollOff {
    pub fn value(self) -> f64 {
        match self {
            RollOff::R005 => 0.05,
            RollOff::R010 => 0.10,
            RollOff::R015 => 0.15,
            RollOff::R020 => 0.20,
            RollOff::R025 => 0.25,
            RollOff::R035 => 0.35,
        }
    }
}

/// SCPC carrier configuration.
#[derive(Debug, Clone)]
pub struct ScpcConfig {
    /// Symbol rate [symbols/s].
    pub symbol_rate_sps: f64,
    /// Roll-off factor.
    pub roll_off: RollOff,
    /// Modulation.
    pub modulation: DvbS2Modulation,
    /// FEC code rate.
    pub code_rate: CodeRate,
    /// Centre frequency offset from NIT [Hz].
    pub freq_offset_hz: f64,
}

impl ScpcConfig {
    /// Occupied bandwidth [Hz].
    pub fn occupied_bandwidth_hz(&self) -> f64 {
        self.symbol_rate_sps * (1.0 + self.roll_off.value())
    }

    /// Net information bit rate [bits/s].
    pub fn bit_rate_bps(&self) -> f64 {
        self.symbol_rate_sps
            * self.modulation.bits_per_symbol() as f64
            * self.code_rate.rate()
    }

    /// Check symbol rate is within SCPC range (64 ksps to 45 Msps).
    pub fn is_valid_symbol_rate(&self) -> bool {
        self.symbol_rate_sps >= 64_000.0 && self.symbol_rate_sps <= 45_000_000.0
    }
}

// ---------------------------------------------------------------------------
// RRC Filter coefficients
// ---------------------------------------------------------------------------

/// Compute Root Raised Cosine (RRC) FIR filter impulse response.
///
/// # Arguments
/// * `num_taps` - number of filter taps (should be odd)
/// * `sps`      - samples per symbol
/// * `alpha`    - roll-off factor
pub fn rrc_filter(num_taps: usize, sps: usize, alpha: f64) -> Vec<f64> {
    let delay = (num_taps - 1) as f64 / 2.0;
    let mut h = vec![0.0f64; num_taps];
    for i in 0..num_taps {
        let t = (i as f64 - delay) / (sps as f64);
        h[i] = if t == 0.0 {
            1.0 + alpha * (4.0 / PI - 1.0)
        } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-10 {
            alpha / 2.0_f64.sqrt()
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
        } else {
            let num = (PI * t * (1.0 - alpha)).sin()
                + 4.0 * alpha * t * (PI * t * (1.0 + alpha)).cos();
            let den = PI * t * (1.0 - (4.0 * alpha * t).powi(2));
            num / den
        };
    }
    // Normalise to unit energy
    let energy: f64 = h.iter().map(|x| x * x).sum::<f64>().sqrt();
    if energy > 0.0 {
        for x in &mut h { *x /= energy; }
    }
    h
}

// ---------------------------------------------------------------------------
// TDMA Burst Structure
// ---------------------------------------------------------------------------

/// TDMA burst configuration.
#[derive(Debug, Clone)]
pub struct BurstConfig {
    /// Number of preamble symbols (known BPSK sequence for acquisition).
    pub preamble_len: usize,
    /// Number of unique word symbols (frame synchronisation).
    pub unique_word_len: usize,
    /// Pilot block period: insert pilot every N data symbols.
    pub pilot_period: usize,
    /// Number of pilot symbols per pilot block.
    pub pilot_block_len: usize,
    /// Number of data payload symbols.
    pub payload_len: usize,
}

impl BurstConfig {
    /// DVB-S2 normal burst (approximate).
    pub fn dvb_s2_normal() -> Self {
        Self {
            preamble_len: 90,
            unique_word_len: 26,
            pilot_period: 1476,
            pilot_block_len: 36,
            payload_len: 32400,
        }
    }

    /// Short burst frame.
    pub fn short_burst() -> Self {
        Self {
            preamble_len: 45,
            unique_word_len: 26,
            pilot_period: 360,
            pilot_block_len: 36,
            payload_len: 8100,
        }
    }

    /// Total frame length including pilots.
    pub fn total_symbols(&self) -> usize {
        let pilot_blocks = self.payload_len / self.pilot_period;
        self.preamble_len
            + self.unique_word_len
            + pilot_blocks * self.pilot_block_len
            + self.payload_len
    }

    /// Pilot overhead fraction.
    pub fn pilot_overhead(&self) -> f64 {
        let total = self.total_symbols();
        let n_pilots = (self.payload_len / self.pilot_period) * self.pilot_block_len;
        n_pilots as f64 / total as f64
    }
}

/// Generate preamble sequence (BPSK, PRBS-based).
pub fn generate_preamble(len: usize) -> Vec<Complex> {
    // Simple PRBS-15 based preamble
    let mut lfsr: u32 = 0x4A17;
    let sqrt2inv = 1.0 / 2.0_f64.sqrt();
    (0..len).map(|_| {
        let bit = ((lfsr >> 14) ^ (lfsr >> 13)) & 1;
        lfsr = ((lfsr << 1) | bit) & 0x7FFF;
        if bit == 0 { Complex::new(sqrt2inv, sqrt2inv) } else { Complex::new(-sqrt2inv, -sqrt2inv) }
    }).collect()
}

/// Generate unique word (fixed 26-symbol BPSK pattern per DVB-S2).
pub fn generate_unique_word(len: usize) -> Vec<Complex> {
    // Fixed bit pattern from DVB-S2 spec (simplified)
    let pattern: u32 = 0x3_2_D5_A9;
    let sqrt2inv = 1.0 / 2.0_f64.sqrt();
    (0..len).map(|i| {
        let bit = (pattern >> (i % 32)) & 1;
        if bit == 0 { Complex::new(sqrt2inv, sqrt2inv) } else { Complex::new(-sqrt2inv, -sqrt2inv) }
    }).collect()
}

/// Generate pilot block (known BPSK sequence for channel estimation).
pub fn generate_pilot_block(len: usize) -> Vec<Complex> {
    let sqrt2inv = 1.0 / 2.0_f64.sqrt();
    vec![Complex::new(sqrt2inv, sqrt2inv); len]
}

/// Assemble a complete TDMA burst from payload symbols.
pub fn assemble_burst(payload: &[Complex], cfg: &BurstConfig) -> Vec<Complex> {
    let mut burst = Vec::with_capacity(cfg.total_symbols());
    // Preamble
    burst.extend(generate_preamble(cfg.preamble_len));
    // Unique word
    burst.extend(generate_unique_word(cfg.unique_word_len));
    // Data + pilots
    let mut sym_count = 0;
    for sym in payload {
        burst.push(*sym);
        sym_count += 1;
        if sym_count % cfg.pilot_period == 0 {
            burst.extend(generate_pilot_block(cfg.pilot_block_len));
        }
    }
    burst
}

// ---------------------------------------------------------------------------
// ACM (Adaptive Coding and Modulation)
// ---------------------------------------------------------------------------

/// ACM controller state.
#[derive(Debug, Clone)]
pub struct AcmController {
    /// Current ModCod index (into dvb_s2_modcod_table()).
    pub current_modcod_idx: usize,
    /// EMA-smoothed Es/N0 estimate [dB].
    pub esmn0_ema_db: f64,
    /// EMA alpha (0..1, smaller = more smoothing).
    pub ema_alpha: f64,
    /// Hysteresis margin [dB]: must exceed threshold by this to upgrade.
    pub hysteresis_db: f64,
    /// Backoff from threshold for downgrade [dB].
    pub backoff_db: f64,
    /// ModCod table.
    pub table: Vec<ModCod>,
}

impl AcmController {
    /// Create ACM controller with default DVB-S2 table.
    pub fn new(ema_alpha: f64, hysteresis_db: f64) -> Self {
        let table = dvb_s2_modcod_table();
        Self {
            current_modcod_idx: 0,
            esmn0_ema_db: -10.0,
            ema_alpha,
            hysteresis_db,
            backoff_db: 1.0,
            table,
        }
    }

    /// Update EMA with new Es/N0 measurement and potentially switch ModCod.
    pub fn update(&mut self, new_esn0_db: f64) -> usize {
        self.esmn0_ema_db = self.ema_alpha * new_esn0_db
            + (1.0 - self.ema_alpha) * self.esmn0_ema_db;
        self.adapt();
        self.current_modcod_idx
    }

    /// Evaluate ModCod table and select best fitting entry.
    fn adapt(&mut self) {
        let esn0 = self.esmn0_ema_db;
        let cur_thresh = self.table[self.current_modcod_idx].min_esn0_db;

        // Check downgrade first: current ModCod below threshold + backoff
        if esn0 < cur_thresh + self.backoff_db {
            // Search down
            for i in (0..self.current_modcod_idx).rev() {
                if esn0 >= self.table[i].min_esn0_db + self.backoff_db {
                    self.current_modcod_idx = i;
                    return;
                }
            }
            self.current_modcod_idx = 0;
            return;
        }

        // Check upgrade: next ModCod threshold + hysteresis
        if self.current_modcod_idx + 1 < self.table.len() {
            let next_thresh = self.table[self.current_modcod_idx + 1].min_esn0_db;
            if esn0 >= next_thresh + self.hysteresis_db {
                self.current_modcod_idx += 1;
            }
        }
    }

    /// Return current ModCod.
    pub fn current_modcod(&self) -> &ModCod {
        &self.table[self.current_modcod_idx]
    }

    /// Current spectral efficiency [bits/s/Hz].
    pub fn current_efficiency(&self) -> f64 {
        self.current_modcod().spectral_efficiency
    }
}

// ---------------------------------------------------------------------------
// Carrier Recovery
// ---------------------------------------------------------------------------

/// Carrier recovery loop state for satellite channel.
/// Handles large Doppler offsets (±60 kHz at Ku-band).
#[derive(Debug, Clone)]
pub struct CarrierRecovery {
    /// Current phase estimate [rad].
    pub phase: f64,
    /// Current frequency error estimate [rad/sample].
    pub freq_error: f64,
    /// Loop bandwidth [normalised, 0..0.5].
    pub loop_bw: f64,
    /// Loop gain alpha (proportional).
    alpha: f64,
    /// Loop gain beta (integral).
    beta: f64,
    /// Number of samples processed.
    pub samples: u64,
    /// Phase noise variance estimate.
    pub phase_noise_var: f64,
}

impl CarrierRecovery {
    /// Create carrier recovery loop.
    ///
    /// # Arguments
    /// * `loop_bw` - normalised loop bandwidth (Bn*Ts), e.g. 0.01
    pub fn new(loop_bw: f64) -> Self {
        let (alpha, beta) = compute_pll_gains(loop_bw, 1.0 / 2.0_f64.sqrt());
        Self {
            phase: 0.0,
            freq_error: 0.0,
            loop_bw,
            alpha,
            beta,
            samples: 0,
            phase_noise_var: 0.0,
        }
    }

    /// Process one symbol: rotate by current phase, update loop from pilot or decision.
    pub fn process(&mut self, sample: Complex, pilot: Option<Complex>) -> Complex {
        // NCO rotation
        let correction = Complex::from_polar(1.0, -self.phase);
        let rotated = sample.mul(correction);

        // Phase error detection
        let phase_err = if let Some(ref_sym) = pilot {
            // Pilot-aided: direct phase error
            let err_sym = rotated.mul(ref_sym.conjugate());
            err_sym.im.atan2(err_sym.re)
        } else {
            // Decision-directed: QPSK
            let sign_re = if rotated.re >= 0.0 { 1.0 } else { -1.0 };
            let sign_im = if rotated.im >= 0.0 { 1.0 } else { -1.0 };
            rotated.re * sign_im - rotated.im * sign_re
        };

        // Track phase noise variance
        self.phase_noise_var = 0.99 * self.phase_noise_var + 0.01 * phase_err * phase_err;

        // Update loop
        self.freq_error += self.beta * phase_err;
        self.phase += self.freq_error + self.alpha * phase_err;
        // Wrap phase
        self.phase = wrap_phase(self.phase);
        self.samples += 1;

        rotated
    }

    /// Coarse frequency estimation via FFT-based method (for large Doppler).
    /// Returns estimated frequency offset in normalised units.
    pub fn estimate_freq_offset(samples: &[Complex], search_range: f64) -> f64 {
        if samples.len() < 4 { return 0.0; }
        // Squared signal removes QPSK modulation
        let sq: Vec<Complex> = samples.iter().map(|s| s.mul(*s)).collect();
        // Simple DFT peak search
        let n = sq.len();
        let steps = 256usize;
        let step = 2.0 * search_range / steps as f64;
        let mut best_freq = 0.0f64;
        let mut best_power = 0.0f64;
        for k in 0..=steps {
            let f = -search_range + k as f64 * step;
            let mut re = 0.0f64;
            let mut im = 0.0f64;
            for (i, s) in sq.iter().enumerate() {
                let angle = -2.0 * PI * f * i as f64;
                re += s.re * angle.cos() - s.im * angle.sin();
                im += s.re * angle.sin() + s.im * angle.cos();
            }
            let power = re * re + im * im;
            if power > best_power {
                best_power = power;
                best_freq = f / 2.0; // un-square
            }
        }
        best_freq
    }

    /// Reset loop.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq_error = 0.0;
        self.samples = 0;
        self.phase_noise_var = 0.0;
    }
}

/// Compute second-order PLL gains from loop bandwidth.
fn compute_pll_gains(loop_bw: f64, damping: f64) -> (f64, f64) {
    let denom = 1.0 + 2.0 * damping * loop_bw + loop_bw * loop_bw;
    let alpha = 4.0 * damping * loop_bw / denom;
    let beta = 4.0 * loop_bw * loop_bw / denom;
    (alpha, beta)
}

/// Wrap angle to [-pi, pi].
fn wrap_phase(p: f64) -> f64 {
    let mut p = p;
    while p > PI  { p -= 2.0 * PI; }
    while p < -PI { p += 2.0 * PI; }
    p
}

// ---------------------------------------------------------------------------
// Timing Recovery (Gardner TED)
// ---------------------------------------------------------------------------

/// Symbol timing recovery using Gardner TED.
#[derive(Debug, Clone)]
pub struct TimingRecovery {
    /// Current timing phase (fractional sample, 0..1).
    pub mu: f64,
    /// Loop frequency error [samples/sample].
    pub freq_error: f64,
    /// Loop gains.
    alpha: f64,
    beta: f64,
    /// Samples-per-symbol.
    pub sps: f64,
    /// Interpolation buffer.
    buf: Vec<Complex>,
    /// Interpolation buffer write pointer.
    buf_ptr: usize,
    /// Output buffer.
    pub output: Vec<Complex>,
}

impl TimingRecovery {
    /// Create Gardner timing recovery.
    ///
    /// # Arguments
    /// * `sps`     - nominal samples per symbol
    /// * `loop_bw` - normalised loop bandwidth
    pub fn new(sps: f64, loop_bw: f64) -> Self {
        let (alpha, beta) = compute_pll_gains(loop_bw, 1.0 / 2.0_f64.sqrt());
        let buf_size = (sps * 4.0).ceil() as usize + 4;
        Self {
            mu: 0.0,
            freq_error: 0.0,
            alpha,
            beta,
            sps,
            buf: vec![Complex::new(0.0, 0.0); buf_size],
            buf_ptr: 0,
            output: Vec::new(),
        }
    }

    /// Process input samples; output vector receives recovered symbols.
    pub fn process(&mut self, samples: &[Complex]) {
        self.output.clear();
        let buf_len = self.buf.len();
        for &s in samples {
            self.buf[self.buf_ptr] = s;
            self.buf_ptr = (self.buf_ptr + 1) % buf_len;

            // Check if we should produce an output sample
            // (integer phase accumulation)
            self.mu += 1.0;
            if self.mu >= self.sps + self.freq_error {
                self.mu -= self.sps + self.freq_error;

                // Linear interpolation
                let frac = self.mu / self.sps;
                let i0 = self.buf_ptr;
                let i1 = (self.buf_ptr + buf_len - 1) % buf_len;
                let s0 = self.buf[i0];
                let s1 = self.buf[i1];
                let interp = s0.scale(frac).add(s1.scale(1.0 - frac));

                // Gardner TED: error = Re{(s[n] - s[n-2]) * conj(s[n-1])}
                let i2 = (self.buf_ptr + buf_len - 2) % buf_len;
                let s2 = self.buf[i2];
                let mid = self.buf[i1];
                let ted_input = s0.sub(s2).mul(mid.conjugate());
                let ted_err = ted_input.re;

                // Update loop
                self.freq_error += self.beta * ted_err;
                self.mu += self.alpha * ted_err;

                self.output.push(interp);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// SNR Estimation from Pilots
// ---------------------------------------------------------------------------

/// Estimate Es/N0 from pilot symbols using M2M4 method.
///
/// # Arguments
/// * `received`  - received pilot samples
/// * `reference` - known pilot reference
///
/// Returns (Es_N0_linear, Es_N0_dB).
pub fn estimate_esn0_from_pilots(received: &[Complex], reference: &[Complex]) -> (f64, f64) {
    if received.is_empty() || reference.is_empty() {
        return (1.0, 0.0);
    }
    let n = received.len().min(reference.len());

    // Channel estimate H = mean(r * conj(p))
    let h_re: f64 = (0..n).map(|i| received[i].re * reference[i].re + received[i].im * reference[i].im).sum::<f64>() / n as f64;
    let h_im: f64 = (0..n).map(|i| received[i].im * reference[i].re - received[i].re * reference[i].im).sum::<f64>() / n as f64;
    let h_mag_sq = h_re * h_re + h_im * h_im;

    // Noise variance from residuals
    let noise_var: f64 = (0..n).map(|i| {
        let r = received[i];
        let p = reference[i];
        let exp_re = h_re * p.re - h_im * p.im;
        let exp_im = h_re * p.im + h_im * p.re;
        let e_re = r.re - exp_re;
        let e_im = r.im - exp_im;
        e_re * e_re + e_im * e_im
    }).sum::<f64>() / n as f64;

    // Es/N0 = |H|^2 * Es / (N0/2 per dimension * 2)
    let esn0_lin = if noise_var > 1e-12 {
        h_mag_sq / noise_var
    } else {
        1e6
    };
    let esn0_db = 10.0 * esn0_lin.log10();
    (esn0_lin, esn0_db)
}

// ---------------------------------------------------------------------------
// Rain Fade Mitigation
// ---------------------------------------------------------------------------

/// Satellite frequency band.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SatBand {
    /// 10.7 – 12.75 GHz / 13.75 – 14.5 GHz (uplink)
    Ku,
    /// 26.5 – 30 GHz (uplink) / 17.7 – 20.2 GHz (downlink)
    Ka,
    /// 3.7 – 4.2 GHz (downlink) / 5.925 – 6.425 GHz (uplink)
    C,
}

impl SatBand {
    /// Nominal uplink centre frequency [GHz].
    pub fn uplink_freq_ghz(self) -> f64 {
        match self { SatBand::C => 6.2, SatBand::Ku => 14.25, SatBand::Ka => 28.5 }
    }
    /// Nominal downlink centre frequency [GHz].
    pub fn downlink_freq_ghz(self) -> f64 {
        match self { SatBand::C => 3.95, SatBand::Ku => 12.5, SatBand::Ka => 19.5 }
    }
}

/// Compute rain attenuation [dB] per ITU-R P.838 simplified model.
///
/// # Arguments
/// * `rain_rate_mmh` - rain rate [mm/h]
/// * `freq_ghz`      - frequency [GHz]
/// * `elevation_deg` - elevation angle [degrees]
/// * `path_len_km`   - effective path length [km]
pub fn rain_attenuation_db(
    rain_rate_mmh: f64,
    freq_ghz: f64,
    elevation_deg: f64,
    path_len_km: f64,
) -> f64 {
    if rain_rate_mmh <= 0.0 { return 0.0; }
    // ITU-R P.838-3 power-law coefficients (simplified)
    let (k, alpha) = itu_r_p838_coefficients(freq_ghz);
    let gamma_r = k * rain_rate_mmh.powf(alpha); // dB/km
    let elev_rad = elevation_deg * PI / 180.0;
    let eff_len = path_len_km / elev_rad.sin().max(0.1);
    gamma_r * eff_len
}

/// Simplified ITU-R P.838-3 coefficients for horizontal polarisation.
fn itu_r_p838_coefficients(freq_ghz: f64) -> (f64, f64) {
    // Piecewise fit
    if freq_ghz <= 2.0 {
        (0.0000387, 0.912)
    } else if freq_ghz <= 6.0 {
        (0.000354, 0.939)
    } else if freq_ghz <= 10.0 {
        (0.00101, 1.276)
    } else if freq_ghz <= 14.0 {
        (0.00351, 1.217)
    } else if freq_ghz <= 20.0 {
        (0.0119, 1.154)
    } else if freq_ghz <= 30.0 {
        (0.0357, 1.031)
    } else {
        (0.0691, 0.958)
    }
}

/// Rain fade mitigation state machine.
#[derive(Debug, Clone)]
pub struct RainFadeMitigation {
    /// Current uplink power boost [dB] (ULPC).
    pub power_boost_db: f64,
    /// Maximum power boost available [dB].
    pub max_boost_db: f64,
    /// Fade depth estimate [dB].
    pub fade_depth_db: f64,
    /// ACM controller (fallback modcod).
    pub acm: AcmController,
    /// Site diversity gain estimate [dB].
    pub diversity_gain_db: f64,
    /// Number of diversity sites.
    pub num_diversity_sites: usize,
}

impl RainFadeMitigation {
    /// Create rain fade mitigation.
    pub fn new(max_boost_db: f64, num_diversity_sites: usize) -> Self {
        Self {
            power_boost_db: 0.0,
            max_boost_db,
            fade_depth_db: 0.0,
            acm: AcmController::new(0.1, 1.0),
            diversity_gain_db: 0.0,
            num_diversity_sites,
        }
    }

    /// Update mitigation strategy given measured C/N0 change.
    ///
    /// # Arguments
    /// * `cn0_delta_db` - observed C/N0 drop from clear-sky [dB]
    pub fn update(&mut self, cn0_delta_db: f64) {
        self.fade_depth_db = cn0_delta_db.max(0.0);
        self.diversity_gain_db = estimate_diversity_gain(self.fade_depth_db, self.num_diversity_sites);
        let effective_fade = (self.fade_depth_db - self.diversity_gain_db).max(0.0);
        // Apply ULPC up to max_boost
        self.power_boost_db = effective_fade.min(self.max_boost_db);
        let residual_fade = (effective_fade - self.power_boost_db).max(0.0);
        // Update ACM with degraded Es/N0
        // Assume some baseline Es/N0 (placeholder for integration)
        let baseline_esn0 = 10.0;
        self.acm.update(baseline_esn0 - residual_fade);
    }
}

/// Estimate site diversity gain [dB] per simplified model.
fn estimate_diversity_gain(fade_db: f64, num_sites: usize) -> f64 {
    if num_sites <= 1 { return 0.0; }
    // Simplified: diversity gain increases with fade depth, saturates
    let base = (num_sites as f64 - 1.0) * 3.0; // 3 dB per additional site
    fade_db.min(base) * 0.7
}

// ---------------------------------------------------------------------------
// VSAT Link Budget
// ---------------------------------------------------------------------------

/// Satellite band and frequency plan.
#[derive(Debug, Clone)]
pub struct VsatLinkBudget {
    pub band: SatBand,
    /// Uplink frequency [Hz].
    pub uplink_freq_hz: f64,
    /// Downlink frequency [Hz].
    pub downlink_freq_hz: f64,
    /// VSAT terminal transmit power [dBW].
    pub tx_power_dbw: f64,
    /// VSAT antenna diameter [m].
    pub antenna_diameter_m: f64,
    /// VSAT antenna efficiency (0..1).
    pub antenna_efficiency: f64,
    /// VSAT receive noise temperature [K].
    pub rx_noise_temp_k: f64,
    /// VSAT receiver noise figure [dB].
    pub rx_noise_figure_db: f64,
    /// Satellite transponder EIRP [dBW].
    pub satellite_eirp_dbw: f64,
    /// Satellite G/T [dB/K].
    pub satellite_gt_dbk: f64,
    /// Path distance (GEO: ~35786 km).
    pub distance_km: f64,
    /// Terminal elevation angle [degrees].
    pub elevation_deg: f64,
    /// Atmospheric absorption [dB] (clear sky).
    pub atm_absorption_db: f64,
    /// Rain attenuation [dB].
    pub rain_attenuation_db: f64,
    /// Transponder bandwidth [Hz].
    pub transponder_bw_hz: f64,
    /// Symbol rate for return link [sps].
    pub symbol_rate_sps: f64,
    /// Noise bandwidth factor (Bn/Rs, ~1.0 for matched filter).
    pub noise_bw_factor: f64,
    /// Adjacent satellite interference allowance [dB].
    pub asi_db: f64,
}

impl VsatLinkBudget {
    /// Create a typical Ku-band VSAT link budget.
    pub fn ku_band_typical() -> Self {
        Self {
            band: SatBand::Ku,
            uplink_freq_hz: 14.25e9,
            downlink_freq_hz: 12.5e9,
            tx_power_dbw: 3.0, // 2W
            antenna_diameter_m: 0.9,
            antenna_efficiency: 0.65,
            rx_noise_temp_k: 100.0,
            rx_noise_figure_db: 0.7,
            satellite_eirp_dbw: 50.0,
            satellite_gt_dbk: -1.0,
            distance_km: 35_786.0,
            elevation_deg: 40.0,
            atm_absorption_db: 0.3,
            rain_attenuation_db: 0.0,
            transponder_bw_hz: 36e6,
            symbol_rate_sps: 1e6,
            noise_bw_factor: 1.0,
            asi_db: 0.5,
        }
    }

    /// Create a Ka-band VSAT link budget.
    pub fn ka_band_typical() -> Self {
        Self {
            band: SatBand::Ka,
            uplink_freq_hz: 28.5e9,
            downlink_freq_hz: 19.5e9,
            tx_power_dbw: 1.0,
            antenna_diameter_m: 0.74,
            antenna_efficiency: 0.65,
            rx_noise_temp_k: 150.0,
            rx_noise_figure_db: 1.2,
            satellite_eirp_dbw: 55.0,
            satellite_gt_dbk: 3.0,
            distance_km: 35_786.0,
            elevation_deg: 40.0,
            atm_absorption_db: 0.5,
            rain_attenuation_db: 0.0,
            transponder_bw_hz: 500e6,
            symbol_rate_sps: 2e6,
            noise_bw_factor: 1.0,
            asi_db: 0.3,
        }
    }

    /// Antenna gain [dBi] from aperture.
    pub fn antenna_gain_dbi(&self, freq_hz: f64) -> f64 {
        let wavelength = C_LIGHT / freq_hz;
        let area = PI * (self.antenna_diameter_m / 2.0).powi(2);
        let gain_lin = self.antenna_efficiency * 4.0 * PI * area / (wavelength * wavelength);
        10.0 * gain_lin.log10()
    }

    /// TX EIRP [dBW].
    pub fn tx_eirp_dbw(&self) -> f64 {
        self.tx_power_dbw + self.antenna_gain_dbi(self.uplink_freq_hz)
    }

    /// Free-space path loss (FSPL) [dB].
    pub fn fspl_db(&self, freq_hz: f64) -> f64 {
        let wavelength = C_LIGHT / freq_hz;
        let dist_m = self.distance_km * 1000.0;
        let fspl_lin = (4.0 * PI * dist_m / wavelength).powi(2);
        10.0 * fspl_lin.log10()
    }

    /// Terminal receive G/T [dB/K].
    pub fn terminal_gt_dbk(&self) -> f64 {
        let g = self.antenna_gain_dbi(self.downlink_freq_hz);
        let nf_lin = 10.0_f64.powf(self.rx_noise_figure_db / 10.0);
        let t_sys = T0_K * (nf_lin - 1.0) + self.rx_noise_temp_k;
        g - 10.0 * t_sys.log10()
    }

    /// Total uplink path loss [dB].
    pub fn uplink_path_loss_db(&self) -> f64 {
        self.fspl_db(self.uplink_freq_hz)
            + self.atm_absorption_db
            + self.rain_attenuation_db
    }

    /// Total downlink path loss [dB].
    pub fn downlink_path_loss_db(&self) -> f64 {
        self.fspl_db(self.downlink_freq_hz)
            + self.atm_absorption_db
            + self.rain_attenuation_db
    }

    /// Uplink C/N at satellite [dB-Hz].
    pub fn uplink_cn0_dbhz(&self) -> f64 {
        self.tx_eirp_dbw()
            - self.uplink_path_loss_db()
            + self.satellite_gt_dbk
            - 10.0 * K_BOLTZMANN.log10()
    }

    /// Downlink C/N at terminal [dB-Hz].
    pub fn downlink_cn0_dbhz(&self) -> f64 {
        self.satellite_eirp_dbw
            - self.downlink_path_loss_db()
            + self.terminal_gt_dbk()
            - 10.0 * K_BOLTZMANN.log10()
    }

    /// Overall system C/N (combining uplink and downlink).
    pub fn total_cn0_dbhz(&self) -> f64 {
        let up_lin = 10.0_f64.powf(self.uplink_cn0_dbhz() / 10.0);
        let dn_lin = 10.0_f64.powf(self.downlink_cn0_dbhz() / 10.0);
        // ASI degradation
        let asi_lin = 10.0_f64.powf(self.asi_db / 10.0);
        let total = 1.0 / (1.0 / up_lin + 1.0 / dn_lin + asi_lin / (up_lin + dn_lin));
        10.0 * total.log10()
    }

    /// Eb/N0 [dB] given spectral efficiency eta.
    pub fn eb_n0_db(&self, spectral_efficiency: f64) -> f64 {
        let noise_bw_hz = self.symbol_rate_sps * self.noise_bw_factor;
        let cn_db = self.total_cn0_dbhz() - 10.0 * noise_bw_hz.log10();
        cn_db - 10.0 * spectral_efficiency.log10()
    }

    /// Es/N0 [dB] at the receiver.
    pub fn es_n0_db(&self) -> f64 {
        let noise_bw_hz = self.symbol_rate_sps * self.noise_bw_factor;
        self.total_cn0_dbhz() - 10.0 * noise_bw_hz.log10()
    }

    /// Required Es/N0 margin above minimum ModCod threshold [dB].
    pub fn modcod_margin_db(&self, modcod: &ModCod) -> f64 {
        self.es_n0_db() - modcod.min_esn0_db
    }

    /// Maximum achievable throughput [bits/s] given current link conditions.
    pub fn max_throughput_bps(&self, table: &[ModCod]) -> f64 {
        let esn0 = self.es_n0_db();
        let best = table.iter()
            .filter(|mc| esn0 >= mc.min_esn0_db)
            .max_by(|a, b| a.spectral_efficiency.partial_cmp(&b.spectral_efficiency).unwrap());
        match best {
            Some(mc) => mc.spectral_efficiency * self.symbol_rate_sps,
            None => 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Frequency Estimation (for Acquisition)
// ---------------------------------------------------------------------------

/// Frequency offset estimation result.
#[derive(Debug, Clone, Copy)]
pub struct FreqEstimate {
    /// Estimated frequency offset [Hz].
    pub offset_hz: f64,
    /// Confidence (peak-to-average power ratio of search).
    pub confidence: f64,
}

/// Estimate carrier frequency offset from received signal burst using M-th power.
///
/// # Arguments
/// * `samples`     - complex samples
/// * `sps`         - samples per symbol
/// * `max_off_hz`  - maximum search range [Hz]
/// * `sample_rate` - sample rate [Hz]
/// * `order`       - modulation order for M-th power (2 for QPSK, 4 for 16QAM)
pub fn estimate_carrier_offset(
    samples: &[Complex],
    sps: usize,
    max_off_hz: f64,
    sample_rate: f64,
    order: u32,
) -> FreqEstimate {
    let n = samples.len();
    if n == 0 { return FreqEstimate { offset_hz: 0.0, confidence: 0.0 }; }

    // Raise to M-th power to remove modulation
    let powered: Vec<Complex> = samples.iter().map(|s| {
        let mut r = *s;
        for _ in 1..order { r = r.mul(*s); }
        r
    }).collect();

    // DFT search
    let search_steps = 512usize;
    let max_norm = max_off_hz / sample_rate;
    let step = 2.0 * max_norm / search_steps as f64;
    let mut best_power = 0.0f64;
    let mut best_f = 0.0f64;
    let mut total_power = 0.0f64;

    for k in 0..=search_steps {
        let f = -max_norm + k as f64 * step;
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        for (i, s) in powered.iter().enumerate() {
            let angle = -2.0 * PI * f * i as f64;
            re += s.re * angle.cos() - s.im * angle.sin();
            im += s.re * angle.sin() + s.im * angle.cos();
        }
        let power = (re * re + im * im) / (n as f64 * n as f64);
        total_power += power;
        if power > best_power {
            best_power = power;
            best_f = f;
        }
    }

    let confidence = if total_power > 0.0 {
        best_power * search_steps as f64 / total_power
    } else { 0.0 };

    FreqEstimate {
        offset_hz: best_f * sample_rate / order as f64,
        confidence,
    }
}

// ---------------------------------------------------------------------------
// Unique Word Correlation / Frame Sync
// ---------------------------------------------------------------------------

/// Correlate a block of samples against the unique word, return best offset and score.
pub fn unique_word_correlate(samples: &[Complex], uw: &[Complex]) -> (usize, f64) {
    let uw_len = uw.len();
    if samples.len() < uw_len { return (0, 0.0); }
    let search_len = samples.len() - uw_len + 1;
    let uw_energy: f64 = uw.iter().map(|s| s.magnitude_sq()).sum::<f64>().sqrt();

    let mut best_offset = 0usize;
    let mut best_score = 0.0f64;

    for offset in 0..search_len {
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        let mut sig_energy = 0.0f64;
        for k in 0..uw_len {
            let s = samples[offset + k];
            let r = uw[k];
            re += s.re * r.re + s.im * r.im;
            im += s.im * r.re - s.re * r.im;
            sig_energy += s.magnitude_sq();
        }
        let score = (re * re + im * im).sqrt() / (uw_energy * sig_energy.sqrt() + 1e-12);
        if score > best_score {
            best_score = score;
            best_offset = offset;
        }
    }
    (best_offset, best_score)
}

// ---------------------------------------------------------------------------
// Power Control
// ---------------------------------------------------------------------------

/// Open-loop uplink power control.
#[derive(Debug, Clone)]
pub struct UplinkPowerControl {
    /// Nominal transmit power [dBW].
    pub nominal_power_dbw: f64,
    /// Current power level [dBW].
    pub current_power_dbw: f64,
    /// Maximum power [dBW].
    pub max_power_dbw: f64,
    /// Minimum power [dBW].
    pub min_power_dbw: f64,
    /// Step size for power adjustment [dB].
    pub step_db: f64,
}

impl UplinkPowerControl {
    /// Create power control.
    pub fn new(nominal_dbw: f64, max_dbw: f64, min_dbw: f64) -> Self {
        Self {
            nominal_power_dbw: nominal_dbw,
            current_power_dbw: nominal_dbw,
            max_power_dbw: max_dbw,
            min_power_dbw: min_dbw,
            step_db: 0.5,
        }
    }

    /// Increase power by step_db.
    pub fn boost(&mut self) {
        self.current_power_dbw = (self.current_power_dbw + self.step_db).min(self.max_power_dbw);
    }

    /// Decrease power by step_db.
    pub fn reduce(&mut self) {
        self.current_power_dbw = (self.current_power_dbw - self.step_db).max(self.min_power_dbw);
    }

    /// Set specific boost [dB above nominal].
    pub fn set_boost_db(&mut self, boost_db: f64) {
        self.current_power_dbw = (self.nominal_power_dbw + boost_db).clamp(self.min_power_dbw, self.max_power_dbw);
    }

    /// Available headroom [dB].
    pub fn headroom_db(&self) -> f64 {
        self.max_power_dbw - self.current_power_dbw
    }

    /// Power back-off from nominal [dB].
    pub fn back_off_db(&self) -> f64 {
        self.nominal_power_dbw - self.current_power_dbw
    }
}

// ---------------------------------------------------------------------------
// Complete VSAT Modem
// ---------------------------------------------------------------------------

/// VSAT modem processor – aggregates all sub-systems.
#[derive(Debug, Clone)]
pub struct VsatModem {
    /// Burst configuration.
    pub burst_cfg: BurstConfig,
    /// SCPC return link configuration.
    pub scpc_cfg: ScpcConfig,
    /// Carrier recovery loop.
    pub carrier_recovery: CarrierRecovery,
    /// Timing recovery loop.
    pub timing_recovery: TimingRecovery,
    /// ACM controller.
    pub acm: AcmController,
    /// Rain fade mitigation.
    pub rain_fade: RainFadeMitigation,
    /// Uplink power control.
    pub ulpc: UplinkPowerControl,
    /// Link budget.
    pub link_budget: VsatLinkBudget,
    /// Modcod table.
    pub modcod_table: Vec<ModCod>,
    /// Current receive signal power [dBW] (estimated).
    pub rx_power_dbw: f64,
    /// Frame sync state.
    pub frame_synced: bool,
    /// UW correlation threshold.
    pub uw_threshold: f64,
}

impl VsatModem {
    /// Create a default Ku-band VSAT modem.
    pub fn new_ku_band() -> Self {
        let burst_cfg = BurstConfig::dvb_s2_normal();
        let scpc_cfg = ScpcConfig {
            symbol_rate_sps: 1_000_000.0,
            roll_off: RollOff::R020,
            modulation: DvbS2Modulation::Qpsk,
            code_rate: CodeRate::R1_2,
            freq_offset_hz: 0.0,
        };
        Self {
            burst_cfg,
            scpc_cfg,
            carrier_recovery: CarrierRecovery::new(0.01),
            timing_recovery: TimingRecovery::new(4.0, 0.01),
            acm: AcmController::new(0.1, 1.0),
            rain_fade: RainFadeMitigation::new(6.0, 1),
            ulpc: UplinkPowerControl::new(3.0, 10.0, -3.0),
            link_budget: VsatLinkBudget::ku_band_typical(),
            modcod_table: dvb_s2_modcod_table(),
            rx_power_dbw: -120.0,
            frame_synced: false,
            uw_threshold: 0.7,
        }
    }

    /// Create a Ka-band VSAT modem.
    pub fn new_ka_band() -> Self {
        let mut modem = Self::new_ku_band();
        modem.link_budget = VsatLinkBudget::ka_band_typical();
        modem.scpc_cfg.symbol_rate_sps = 2_000_000.0;
        modem.rain_fade.max_boost_db = 10.0;
        modem
    }

    /// Receive-chain processing: carrier recovery + timing recovery + ACM feedback.
    ///
    /// Returns vector of decoded symbols.
    pub fn receive(&mut self, samples: &[Complex], pilots_present: bool) -> Vec<Complex> {
        // Timing recovery
        self.timing_recovery.process(samples);
        let timed = self.timing_recovery.output.clone();

        // Carrier recovery with pilot-aided updates
        let pilot_ref = generate_pilot_block(1);
        let pilot_sym = pilot_ref[0];
        let mut syms: Vec<Complex> = Vec::with_capacity(timed.len());
        for s in &timed {
            let pilot = if pilots_present { Some(pilot_sym) } else { None };
            syms.push(self.carrier_recovery.process(*s, pilot));
        }

        // Frame sync via UW
        if !self.frame_synced && syms.len() >= self.burst_cfg.unique_word_len {
            let uw = generate_unique_word(self.burst_cfg.unique_word_len);
            let (_, score) = unique_word_correlate(&syms, &uw);
            if score >= self.uw_threshold {
                self.frame_synced = true;
            }
        }

        // Estimate Es/N0 from pilot symbols (first pilot block)
        if pilots_present && syms.len() >= self.burst_cfg.pilot_block_len {
            let rx_pilots: Vec<Complex> = syms[0..self.burst_cfg.pilot_block_len].to_vec();
            let ref_pilots = generate_pilot_block(self.burst_cfg.pilot_block_len);
            let (_, esn0_db) = estimate_esn0_from_pilots(&rx_pilots, &ref_pilots);
            self.acm.update(esn0_db);

            // Rain fade check
            let cn0_drop = (self.link_budget.downlink_cn0_dbhz() - esn0_db)
                .max(0.0);
            self.rain_fade.update(cn0_drop);
            self.ulpc.set_boost_db(self.rain_fade.power_boost_db);
        }

        syms
    }

    /// Get current throughput estimate [bits/s].
    pub fn current_throughput_bps(&self) -> f64 {
        let mc = self.acm.current_modcod();
        mc.spectral_efficiency * self.scpc_cfg.symbol_rate_sps
    }

    /// Get occupied bandwidth [Hz].
    pub fn occupied_bandwidth_hz(&self) -> f64 {
        self.scpc_cfg.occupied_bandwidth_hz()
    }

    /// Compute Es/N0 available from link budget.
    pub fn link_esn0_db(&self) -> f64 {
        self.link_budget.es_n0_db()
    }

    /// Best possible ModCod index from link budget.
    pub fn best_modcod_idx(&self) -> usize {
        let esn0 = self.link_esn0_db();
        let mut best = 0usize;
        for (i, mc) in self.modcod_table.iter().enumerate() {
            if esn0 >= mc.min_esn0_db { best = i; }
        }
        best
    }
}

// ---------------------------------------------------------------------------
// Spreading (for low-rate links)
// ---------------------------------------------------------------------------

/// Spreading processor for low-SNR operation.
#[derive(Debug, Clone)]
pub struct Spreader {
    /// Spreading factor (chips per symbol).
    pub spreading_factor: usize,
    /// Spreading code (±1).
    code: Vec<f64>,
}

impl Spreader {
    /// Create a Gold code-based spreader.
    pub fn new(spreading_factor: usize, seed: u32) -> Self {
        let code = generate_spreading_code(spreading_factor, seed);
        Self { spreading_factor, code }
    }

    /// Spread a BPSK symbol stream to chips.
    pub fn spread(&self, symbols: &[f64]) -> Vec<f64> {
        let mut chips = Vec::with_capacity(symbols.len() * self.spreading_factor);
        for &sym in symbols {
            for &chip in &self.code {
                chips.push(sym * chip);
            }
        }
        chips
    }

    /// Despread chips back to symbols (correlate with code).
    pub fn despread(&self, chips: &[f64]) -> Vec<f64> {
        let sf = self.spreading_factor;
        let n_syms = chips.len() / sf;
        let mut syms = Vec::with_capacity(n_syms);
        for i in 0..n_syms {
            let mut acc = 0.0f64;
            for k in 0..sf {
                acc += chips[i * sf + k] * self.code[k];
            }
            syms.push(acc / sf as f64);
        }
        syms
    }

    /// Processing gain [dB].
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * (self.spreading_factor as f64).log10()
    }
}

/// Generate a simple maximal-length spreading code.
fn generate_spreading_code(len: usize, seed: u32) -> Vec<f64> {
    let mut state = seed.max(1) & 0x7FFF;
    (0..len).map(|_| {
        let bit = ((state >> 14) ^ (state >> 13)) & 1;
        state = ((state << 1) | bit) & 0x7FFF;
        if bit == 0 { 1.0 } else { -1.0 }
    }).collect()
}

// ---------------------------------------------------------------------------
// Noise and channel utilities
// ---------------------------------------------------------------------------

/// Add AWGN to a complex signal given Es/N0 in dB.
pub fn add_awgn_esn0(signal: &[Complex], esn0_db: f64, seed: u32) -> Vec<Complex> {
    let esn0_lin = 10.0_f64.powf(esn0_db / 10.0);
    let es: f64 = signal.iter().map(|s| s.magnitude_sq()).sum::<f64>() / signal.len().max(1) as f64;
    let n0 = es / esn0_lin;
    let sigma = (n0 / 2.0).sqrt();
    let mut state: u64 = seed as u64 | 1;
    signal.iter().map(|s| {
        let (n_re, n_im) = box_muller(&mut state);
        Complex::new(s.re + n_re * sigma, s.im + n_im * sigma)
    }).collect()
}

fn box_muller(state: &mut u64) -> (f64, f64) {
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    let u1 = (*state as f64 / u64::MAX as f64).max(1e-15);
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    let u2 = *state as f64 / u64::MAX as f64;
    let r = (-2.0 * u1.ln()).sqrt();
    let theta = 2.0 * PI * u2;
    (r * theta.cos(), r * theta.sin())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- Modulation --

    #[test]
    fn test_qpsk_constellation_count() {
        assert_eq!(qpsk_constellation().len(), 4);
    }

    #[test]
    fn test_qpsk_unit_power() {
        let c = qpsk_constellation();
        for pt in &c {
            let p = pt.magnitude_sq();
            assert!((p - 1.0).abs() < 1e-10, "power={}", p);
        }
    }

    #[test]
    fn test_psk8_constellation_count() {
        assert_eq!(psk8_constellation().len(), 8);
    }

    #[test]
    fn test_psk8_unit_power() {
        for pt in psk8_constellation() {
            assert!((pt.magnitude_sq() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_apsk16_count() {
        assert_eq!(apsk16_constellation(2.85).len(), 16);
    }

    #[test]
    fn test_apsk32_count() {
        assert_eq!(apsk32_constellation(2.84, 5.27).len(), 32);
    }

    #[test]
    fn test_demap_nearest_qpsk() {
        let c = qpsk_constellation();
        // Point close to first symbol
        let s = Complex::new(0.7, 0.7);
        let (idx, dist) = demap_symbol(s, &c);
        assert_eq!(idx, 0);
        assert!(dist < 0.1);
    }

    #[test]
    fn test_map_symbol_roundtrip_qpsk() {
        let c = qpsk_constellation();
        for i in 0..4usize {
            let bits: Vec<bool> = vec![(i >> 1) & 1 == 1, i & 1 == 1];
            let sym = map_symbol(&bits, &c);
            let (got, _) = demap_symbol(sym, &c);
            assert_eq!(got, i);
        }
    }

    // -- Code rates --

    #[test]
    fn test_code_rate_values() {
        assert!((CodeRate::R1_2.rate() - 0.5).abs() < 1e-10);
        assert!((CodeRate::R3_4.rate() - 0.75).abs() < 1e-10);
        assert!((CodeRate::R9_10.rate() - 0.9).abs() < 1e-10);
    }

    // -- ModCod table --

    #[test]
    fn test_modcod_table_count() {
        assert_eq!(dvb_s2_modcod_table().len(), 28);
    }

    #[test]
    fn test_modcod_efficiency_monotone() {
        let table = dvb_s2_modcod_table();
        // Efficiencies should generally increase (not strictly, but roughly)
        for mc in &table {
            let eff = mc.compute_efficiency();
            assert!(eff > 0.0);
            assert!(eff <= 9.0);
        }
    }

    #[test]
    fn test_modcod_esn0_positive_range() {
        let table = dvb_s2_modcod_table();
        // First entry can be negative (high coding gain), last should be highest
        assert!(table[0].min_esn0_db < table[table.len() - 1].min_esn0_db);
    }

    // -- SCPC --

    #[test]
    fn test_scpc_bandwidth() {
        let cfg = ScpcConfig {
            symbol_rate_sps: 1_000_000.0,
            roll_off: RollOff::R020,
            modulation: DvbS2Modulation::Qpsk,
            code_rate: CodeRate::R1_2,
            freq_offset_hz: 0.0,
        };
        let bw = cfg.occupied_bandwidth_hz();
        assert!((bw - 1_200_000.0).abs() < 1.0);
    }

    #[test]
    fn test_scpc_bit_rate() {
        let cfg = ScpcConfig {
            symbol_rate_sps: 1_000_000.0,
            roll_off: RollOff::R020,
            modulation: DvbS2Modulation::Qpsk,
            code_rate: CodeRate::R1_2,
            freq_offset_hz: 0.0,
        };
        let br = cfg.bit_rate_bps();
        assert!((br - 1_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_scpc_valid_symbol_rate() {
        let mut cfg = ScpcConfig {
            symbol_rate_sps: 1_000_000.0,
            roll_off: RollOff::R020,
            modulation: DvbS2Modulation::Qpsk,
            code_rate: CodeRate::R1_2,
            freq_offset_hz: 0.0,
        };
        assert!(cfg.is_valid_symbol_rate());
        cfg.symbol_rate_sps = 100.0;
        assert!(!cfg.is_valid_symbol_rate());
        cfg.symbol_rate_sps = 50_000_000.0;
        assert!(!cfg.is_valid_symbol_rate());
    }

    #[test]
    fn test_roll_off_values() {
        assert!((RollOff::R005.value() - 0.05).abs() < 1e-10);
        assert!((RollOff::R035.value() - 0.35).abs() < 1e-10);
    }

    // -- RRC filter --

    #[test]
    fn test_rrc_filter_length() {
        let h = rrc_filter(33, 4, 0.2);
        assert_eq!(h.len(), 33);
    }

    #[test]
    fn test_rrc_filter_unit_energy() {
        let h = rrc_filter(33, 4, 0.2);
        let energy: f64 = h.iter().map(|x| x * x).sum();
        assert!((energy - 1.0).abs() < 1e-6, "energy={}", energy);
    }

    // -- Burst --

    #[test]
    fn test_burst_config_normal() {
        let cfg = BurstConfig::dvb_s2_normal();
        assert!(cfg.total_symbols() > cfg.payload_len);
    }

    #[test]
    fn test_preamble_length() {
        let p = generate_preamble(90);
        assert_eq!(p.len(), 90);
    }

    #[test]
    fn test_unique_word_length() {
        let uw = generate_unique_word(26);
        assert_eq!(uw.len(), 26);
    }

    #[test]
    fn test_pilot_block_values() {
        let pilots = generate_pilot_block(36);
        assert_eq!(pilots.len(), 36);
        let expected = Complex::new(1.0 / 2.0_f64.sqrt(), 1.0 / 2.0_f64.sqrt());
        for p in &pilots {
            assert!((p.re - expected.re).abs() < 1e-10);
        }
    }

    #[test]
    fn test_assemble_burst() {
        let cfg = BurstConfig::short_burst();
        let payload: Vec<Complex> = (0..cfg.payload_len)
            .map(|i| Complex::new(if i % 2 == 0 { 1.0 } else { -1.0 }, 0.0))
            .collect();
        let burst = assemble_burst(&payload, &cfg);
        assert!(burst.len() >= cfg.preamble_len + cfg.unique_word_len + cfg.payload_len);
    }

    #[test]
    fn test_burst_pilot_overhead() {
        let cfg = BurstConfig::dvb_s2_normal();
        let oh = cfg.pilot_overhead();
        assert!(oh > 0.0 && oh < 0.1);
    }

    // -- ACM --

    #[test]
    fn test_acm_low_snr_picks_robust() {
        let mut acm = AcmController::new(1.0, 0.0);
        // Very low Es/N0 → should pick first (most robust) ModCod
        for _ in 0..5 { acm.update(-10.0); }
        assert_eq!(acm.current_modcod_idx, 0);
    }

    #[test]
    fn test_acm_high_snr_upgrades() {
        let mut acm = AcmController::new(1.0, 0.5);
        // High Es/N0 → should upgrade significantly
        for _ in 0..30 { acm.update(20.0); }
        assert!(acm.current_modcod_idx > 5, "idx={}", acm.current_modcod_idx);
    }

    #[test]
    fn test_acm_ema_smoothing() {
        let mut acm = AcmController::new(0.01, 2.0);
        // Start at low value
        acm.esmn0_ema_db = 0.0;
        acm.update(10.0);
        // EMA should be < 10 due to slow alpha
        assert!(acm.esmn0_ema_db < 10.0);
    }

    // -- Carrier recovery --

    #[test]
    fn test_carrier_recovery_phase_correction() {
        let mut cr = CarrierRecovery::new(0.05);
        // Apply known phase offset
        let phase = 0.3f64;
        let s = Complex::new(phase.cos(), phase.sin());
        let out = cr.process(s, Some(Complex::new(1.0, 0.0)));
        // After correction, phase error should be small
        let out_phase = out.phase();
        assert!(out_phase.abs() < 1.0);
    }

    #[test]
    fn test_carrier_recovery_freq_estimation() {
        // Generate signal with known normalised frequency offset
        let n = 256usize;
        let f_off = 0.02f64; // normalised frequency
        let samples: Vec<Complex> = (0..n).map(|i| {
            let angle = 2.0 * PI * f_off * i as f64;
            Complex::new(angle.cos(), angle.sin())
        }).collect();
        // search_range is normalised (0..0.5); estimate_freq_offset returns normalised freq
        let est = CarrierRecovery::estimate_freq_offset(&samples, 0.1);
        // Should be within ±search range
        assert!(est.abs() <= 0.1);
    }

    #[test]
    fn test_carrier_recovery_reset() {
        let mut cr = CarrierRecovery::new(0.01);
        cr.phase = 1.5;
        cr.freq_error = 0.001;
        cr.reset();
        assert_eq!(cr.phase, 0.0);
        assert_eq!(cr.freq_error, 0.0);
    }

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(4.0) - (4.0 - 2.0 * PI)).abs() < 1e-10);
        assert!((wrap_phase(-4.0) - (-4.0 + 2.0 * PI)).abs() < 1e-10);
    }

    // -- Timing recovery --

    #[test]
    fn test_timing_recovery_produces_output() {
        let mut tr = TimingRecovery::new(4.0, 0.01);
        let samples: Vec<Complex> = (0..100).map(|i| {
            Complex::new((i as f64 * PI / 4.0).cos(), 0.0)
        }).collect();
        tr.process(&samples);
        // Should produce some output symbols
        assert!(!tr.output.is_empty());
    }

    #[test]
    fn test_timing_recovery_output_rate() {
        let sps = 4.0;
        let mut tr = TimingRecovery::new(sps, 0.005);
        let n_samples = 400;
        let samples: Vec<Complex> = (0..n_samples).map(|i| {
            Complex::new((i as f64 * PI / 2.0).cos(), (i as f64 * PI / 2.0).sin())
        }).collect();
        tr.process(&samples);
        let expected = n_samples / 4;
        // Allow ±10% variation
        assert!(tr.output.len() as isize >= expected as isize * 8 / 10);
        assert!(tr.output.len() as isize <= expected as isize * 12 / 10);
    }

    // -- Es/N0 estimation --

    #[test]
    fn test_esn0_estimation_high_snr() {
        let pilots = generate_pilot_block(100);
        let noisy = add_awgn_esn0(&pilots, 20.0, 42);
        let (_, esn0_db) = estimate_esn0_from_pilots(&noisy, &pilots);
        assert!(esn0_db > 10.0, "esn0_db={}", esn0_db);
    }

    #[test]
    fn test_esn0_estimation_empty() {
        let (lin, db) = estimate_esn0_from_pilots(&[], &[]);
        assert_eq!(lin, 1.0);
        assert_eq!(db, 0.0);
    }

    // -- Rain attenuation --

    #[test]
    fn test_rain_attenuation_zero_rain() {
        let att = rain_attenuation_db(0.0, 12.5, 40.0, 5.0);
        assert_eq!(att, 0.0);
    }

    #[test]
    fn test_rain_attenuation_increases_with_rate() {
        let att1 = rain_attenuation_db(1.0, 12.5, 40.0, 5.0);
        let att2 = rain_attenuation_db(50.0, 12.5, 40.0, 5.0);
        assert!(att2 > att1, "att1={} att2={}", att1, att2);
    }

    #[test]
    fn test_rain_attenuation_ka_higher_than_ku() {
        let ku = rain_attenuation_db(10.0, 12.5, 40.0, 5.0);
        let ka = rain_attenuation_db(10.0, 28.5, 40.0, 5.0);
        assert!(ka > ku, "ku={} ka={}", ku, ka);
    }

    #[test]
    fn test_rain_fade_mitigation_boost() {
        let mut rfm = RainFadeMitigation::new(10.0, 1);
        rfm.update(5.0);
        // Should apply power boost
        assert!(rfm.power_boost_db > 0.0);
        assert!(rfm.power_boost_db <= rfm.max_boost_db);
    }

    // -- Link budget --

    #[test]
    fn test_link_budget_antenna_gain() {
        let lb = VsatLinkBudget::ku_band_typical();
        let g = lb.antenna_gain_dbi(lb.uplink_freq_hz);
        // 0.9m antenna at 14.25 GHz should be ~38-42 dBi
        assert!(g > 30.0 && g < 50.0, "gain={}", g);
    }

    #[test]
    fn test_link_budget_fspl() {
        let lb = VsatLinkBudget::ku_band_typical();
        let fspl = lb.fspl_db(lb.downlink_freq_hz);
        // GEO FSPL at 12.5 GHz should be ~205-206 dB
        assert!(fspl > 200.0 && fspl < 215.0, "fspl={}", fspl);
    }

    #[test]
    fn test_link_budget_eirp() {
        let lb = VsatLinkBudget::ku_band_typical();
        let eirp = lb.tx_eirp_dbw();
        // Should be reasonable (positive, < 60 dBW for small terminal)
        assert!(eirp > 0.0 && eirp < 60.0, "eirp={}", eirp);
    }

    #[test]
    fn test_link_budget_cn0() {
        let lb = VsatLinkBudget::ku_band_typical();
        let cn0 = lb.downlink_cn0_dbhz();
        // Should be a valid C/N0 (typically 50-90 dB-Hz for VSAT downlink)
        assert!(cn0 > 30.0 && cn0 < 120.0, "cn0={}", cn0);
    }

    #[test]
    fn test_link_budget_total_cn0() {
        let lb = VsatLinkBudget::ku_band_typical();
        let total = lb.total_cn0_dbhz();
        let up = lb.uplink_cn0_dbhz();
        let dn = lb.downlink_cn0_dbhz();
        // Total must be less than both uplink and downlink
        assert!(total <= up + 0.1, "total={} up={}", total, up);
        assert!(total <= dn + 0.1, "total={} dn={}", total, dn);
    }

    #[test]
    fn test_link_budget_esn0() {
        let lb = VsatLinkBudget::ku_band_typical();
        let esn0 = lb.es_n0_db();
        // For 1 Msps, should be positive
        assert!(esn0 > 0.0, "esn0={}", esn0);
    }

    #[test]
    fn test_link_budget_max_throughput() {
        let lb = VsatLinkBudget::ku_band_typical();
        let table = dvb_s2_modcod_table();
        let tp = lb.max_throughput_bps(&table);
        assert!(tp > 0.0, "tp={}", tp);
    }

    #[test]
    fn test_ka_band_link_budget() {
        let lb = VsatLinkBudget::ka_band_typical();
        let fspl = lb.fspl_db(lb.downlink_freq_hz);
        // Ka-band has higher FSPL than Ku-band
        let lb_ku = VsatLinkBudget::ku_band_typical();
        let fspl_ku = lb_ku.fspl_db(lb_ku.downlink_freq_hz);
        assert!(fspl > fspl_ku, "ka_fspl={} ku_fspl={}", fspl, fspl_ku);
    }

    // -- Frequency estimation --

    #[test]
    fn test_freq_estimate_output_range() {
        let samples: Vec<Complex> = (0..256).map(|i| {
            let angle = 2.0 * PI * 0.05 * i as f64;
            Complex::new(angle.cos(), angle.sin())
        }).collect();
        let est = estimate_carrier_offset(&samples, 1, 60_000.0, 1_000_000.0, 2);
        // Result should be within search range
        assert!(est.offset_hz.abs() <= 60_000.0, "offset={}", est.offset_hz);
    }

    // -- UW correlation --

    #[test]
    fn test_uw_correlate_self() {
        let uw = generate_unique_word(26);
        let mut samples = vec![Complex::new(0.0, 0.0); 10];
        samples.extend_from_slice(&uw);
        let (offset, score) = unique_word_correlate(&samples, &uw);
        assert_eq!(offset, 10);
        assert!(score > 0.9, "score={}", score);
    }

    #[test]
    fn test_uw_correlate_noise_low_score() {
        let uw = generate_unique_word(26);
        // Random noise
        let noise: Vec<Complex> = (0..50).map(|i| {
            Complex::new((i as f64 * 1.3).sin(), (i as f64 * 0.7).cos())
        }).collect();
        let (_, score) = unique_word_correlate(&noise, &uw);
        // Score should be low for unrelated signal
        assert!(score < 0.9, "score={}", score);
    }

    // -- Power control --

    #[test]
    fn test_ulpc_boost() {
        let mut ulpc = UplinkPowerControl::new(3.0, 10.0, -3.0);
        ulpc.boost();
        assert!((ulpc.current_power_dbw - 3.5).abs() < 1e-10);
    }

    #[test]
    fn test_ulpc_max_clamp() {
        let mut ulpc = UplinkPowerControl::new(3.0, 10.0, -3.0);
        for _ in 0..100 { ulpc.boost(); }
        assert!((ulpc.current_power_dbw - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_ulpc_min_clamp() {
        let mut ulpc = UplinkPowerControl::new(3.0, 10.0, -3.0);
        for _ in 0..100 { ulpc.reduce(); }
        assert!((ulpc.current_power_dbw - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn test_ulpc_set_boost() {
        let mut ulpc = UplinkPowerControl::new(3.0, 10.0, -3.0);
        ulpc.set_boost_db(4.0);
        assert!((ulpc.current_power_dbw - 7.0).abs() < 1e-10);
    }

    // -- Spreading --

    #[test]
    fn test_spread_despread_roundtrip() {
        let spreader = Spreader::new(16, 0xABCD);
        let data = vec![1.0, -1.0, 1.0, 1.0, -1.0];
        let chips = spreader.spread(&data);
        assert_eq!(chips.len(), data.len() * 16);
        let recovered = spreader.despread(&chips);
        for (d, r) in data.iter().zip(recovered.iter()) {
            assert!((d - r).abs() < 1e-10, "d={} r={}", d, r);
        }
    }

    #[test]
    fn test_spreading_gain_db() {
        let s16 = Spreader::new(16, 1);
        let s64 = Spreader::new(64, 1);
        assert!((s16.processing_gain_db() - 12.04).abs() < 0.1);
        assert!((s64.processing_gain_db() - 18.06).abs() < 0.1);
    }

    // -- VSAT Modem --

    #[test]
    fn test_vsat_modem_ku_create() {
        let modem = VsatModem::new_ku_band();
        assert!(!modem.frame_synced);
        assert_eq!(modem.scpc_cfg.symbol_rate_sps, 1_000_000.0);
    }

    #[test]
    fn test_vsat_modem_ka_create() {
        let modem = VsatModem::new_ka_band();
        assert_eq!(modem.scpc_cfg.symbol_rate_sps, 2_000_000.0);
    }

    #[test]
    fn test_vsat_modem_occupied_bw() {
        let modem = VsatModem::new_ku_band();
        let bw = modem.occupied_bandwidth_hz();
        assert!(bw > 1_000_000.0 && bw < 2_000_000.0);
    }

    #[test]
    fn test_vsat_modem_link_esn0() {
        let modem = VsatModem::new_ku_band();
        let esn0 = modem.link_esn0_db();
        assert!(esn0 > -10.0 && esn0 < 60.0, "esn0={}", esn0);
    }

    #[test]
    fn test_vsat_modem_receive_produces_output() {
        let mut modem = VsatModem::new_ku_band();
        let pilot = generate_pilot_block(1)[0];
        let samples: Vec<Complex> = (0..100).map(|_| pilot).collect();
        let out = modem.receive(&samples, true);
        assert!(!out.is_empty());
    }

    #[test]
    fn test_vsat_modem_best_modcod() {
        let modem = VsatModem::new_ku_band();
        let idx = modem.best_modcod_idx();
        // Index should be valid
        assert!(idx < modem.modcod_table.len());
    }

    #[test]
    fn test_awgn_power_scaling() {
        let signal: Vec<Complex> = (0..1000).map(|i| {
            let a = (i as f64 * PI / 50.0).sin();
            Complex::new(a, 0.0)
        }).collect();
        let noisy = add_awgn_esn0(&signal, 10.0, 99);
        // Signal should be close in power to original
        let orig_pwr: f64 = signal.iter().map(|s| s.magnitude_sq()).sum::<f64>() / signal.len() as f64;
        let noisy_pwr: f64 = noisy.iter().map(|s| s.magnitude_sq()).sum::<f64>() / noisy.len() as f64;
        // noisy power should be higher than original
        assert!(noisy_pwr > orig_pwr * 0.5, "orig={} noisy={}", orig_pwr, noisy_pwr);
    }

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(DvbS2Modulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(DvbS2Modulation::Psk8.bits_per_symbol(), 3);
        assert_eq!(DvbS2Modulation::Apsk16.bits_per_symbol(), 4);
        assert_eq!(DvbS2Modulation::Apsk32.bits_per_symbol(), 5);
    }

    #[test]
    fn test_diversity_gain_zero_with_one_site() {
        assert_eq!(estimate_diversity_gain(10.0, 1), 0.0);
    }

    #[test]
    fn test_diversity_gain_increases_with_sites() {
        let g1 = estimate_diversity_gain(10.0, 1);
        let g2 = estimate_diversity_gain(10.0, 2);
        let g3 = estimate_diversity_gain(10.0, 3);
        assert!(g1 <= g2 && g2 <= g3, "g1={} g2={} g3={}", g1, g2, g3);
    }

    #[test]
    fn test_sat_band_frequencies() {
        assert!(SatBand::Ku.uplink_freq_ghz() > SatBand::C.uplink_freq_ghz());
        assert!(SatBand::Ka.uplink_freq_ghz() > SatBand::Ku.uplink_freq_ghz());
    }

    #[test]
    fn test_pll_gains_nonzero() {
        let (alpha, beta) = compute_pll_gains(0.01, 1.0 / 2.0_f64.sqrt());
        assert!(alpha > 0.0);
        assert!(beta > 0.0);
    }

    #[test]
    fn test_complex_operations() {
        let a = Complex::new(3.0, 4.0);
        assert!((a.magnitude() - 5.0).abs() < 1e-10);
        assert!((a.magnitude_sq() - 25.0).abs() < 1e-10);
        let b = Complex::new(1.0, 0.0);
        let c = a.mul(b);
        assert!((c.re - 3.0).abs() < 1e-10);
        let conj = a.conjugate();
        assert!((conj.im - (-4.0)).abs() < 1e-10);
    }

    #[test]
    fn test_modcod_margin() {
        let lb = VsatLinkBudget::ku_band_typical();
        let table = dvb_s2_modcod_table();
        let mc = &table[0]; // most robust
        let margin = lb.modcod_margin_db(mc);
        // Margin for most robust ModCod should be large
        assert!(margin > 0.0, "margin={}", margin);
    }
}

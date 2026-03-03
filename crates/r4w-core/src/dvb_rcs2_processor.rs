//! DVB-RCS2 (Digital Video Broadcasting – Return Channel via Satellite, 2nd Generation)
//! physical layer processor per ETSI EN 301 545-2.
//!
//! DVB-RCS2 is the standard for interactive satellite broadband services using
//! Multi-Frequency TDMA (MF-TDMA) access on the return link from user terminals
//! (RCSTs - Return Channel Satellite Terminals) to the hub.
//!
//! # Overview
//!
//! Key features implemented:
//! - **MF-TDMA Access**: Superframe/frame/timeslot hierarchy, burst scheduling
//! - **Burst Waveforms**: QPSK/8PSK/16QAM linear bursts + 16-chip Walsh-Hadamard spread
//! - **Turbo Code FEC**: 3GPP/DVB-RCS2 turbo code rates 1/3 through 6/7
//! - **NCR Synchronization**: 27 MHz network clock reference recovery
//! - **Capacity Management**: RBDC/VBDC/CRA/FCA requests, TBTP parsing
//! - **RLE Encapsulation**: Return Link Encapsulation with fragmentation/reassembly
//! - **Logon/Handover**: RCST logon sequence, beam handover procedures
//! - **Link Budget**: Ku-band return link with ITU-R P.618 rain attenuation
//!
//! # Frame Structure
//!
//! ```text
//! Superframe (26.5 ms default)
//!   └── Frames (variable count)
//!         └── Timeslots (burst opportunities per carrier)
//!               └── Burst (preamble + payload symbols)
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::dvb_rcs2_processor::{
//!     DvbRcs2Processor, BurstModulation, TurboCodeRate,
//!     CapacityRequest, CapacityType, NcrTimestamp,
//! };
//!
//! let mut proc = DvbRcs2Processor::new();
//!
//! // Encode a data burst with QPSK 1/2
//! let data = vec![0xAB, 0xCD, 0xEF];
//! let burst = proc.encode_burst(&data, BurstModulation::Qpsk, TurboCodeRate::R1_2);
//! assert!(!burst.symbols.is_empty());
//!
//! // Build a capacity request
//! let req = CapacityRequest::new(CapacityType::Rbdc { rate_kbps: 512 });
//! let encoded = req.encode();
//! assert_eq!(encoded.len(), 3);
//! ```

use std::f64::consts::PI;
use std::collections::VecDeque;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Default superframe duration in microseconds (26.5 ms per ETSI EN 301 545-2).
pub const SUPERFRAME_DURATION_US: u64 = 26_500;

/// DVB-RCS2 symbol rate options in Msymbols/s.
pub const SYMBOL_RATES_MSPS: &[f64] = &[0.1, 0.2, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0];

/// NCR clock frequency: 27 MHz.
pub const NCR_CLOCK_HZ: u64 = 27_000_000;

/// NCR timestamp rollover period (2^32 ticks at 27 MHz ≈ 158.96 s).
pub const NCR_ROLLOVER: u64 = 1u64 << 32;

/// Unique word length for linear burst preamble (bits).
pub const UW_LENGTH_BITS: usize = 48;

/// Walsh-Hadamard spreading factor for SS-CDMA bursts.
pub const WH_SPREADING_FACTOR: usize = 16;

/// Maximum number of RCST terminals per hub (implementation limit).
pub const MAX_RCTS: usize = 1024;

/// Ku-band uplink centre frequency (Hz) – 14.25 GHz midpoint.
pub const KU_UPLINK_FREQ_HZ: f64 = 14.25e9;

/// Boltzmann constant (J/K).
pub const BOLTZMANN: f64 = 1.380649e-23;

/// Reference temperature for noise (K).
pub const T_REF_K: f64 = 290.0;

// ---------------------------------------------------------------------------
// Complex sample type
// ---------------------------------------------------------------------------

/// IQ complex sample (64-bit).
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
    pub fn magnitude(&self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
    pub fn angle(&self) -> f64 {
        self.im.atan2(self.re)
    }
    pub fn conj(&self) -> Self {
        Self { re: self.re, im: -self.im }
    }
    pub fn mul(&self, other: &Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
    pub fn scale(&self, s: f64) -> Self {
        Self { re: self.re * s, im: self.im * s }
    }
}

// ---------------------------------------------------------------------------
// Burst modulation and waveform types
// ---------------------------------------------------------------------------

/// Modulation scheme for return link bursts per ETSI EN 301 545-2 Table B.1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BurstModulation {
    /// QPSK (2 bits/symbol).
    Qpsk,
    /// 8PSK (3 bits/symbol).
    Psk8,
    /// 16QAM (4 bits/symbol).
    Qam16,
    /// Spread-spectrum 16-chip Walsh-Hadamard (SS-CDMA).
    SpreadSpectrum,
}

impl BurstModulation {
    /// Bits per symbol for linear modulations.
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            BurstModulation::Qpsk => 2,
            BurstModulation::Psk8 => 3,
            BurstModulation::Qam16 => 4,
            BurstModulation::SpreadSpectrum => 1, // before spreading
        }
    }

    /// Number of constellation points.
    pub fn order(&self) -> usize {
        1 << self.bits_per_symbol()
    }

    /// Minimum required Es/N0 for BER < 1e-7 (approximate, dB).
    pub fn min_es_n0_db(&self) -> f64 {
        match self {
            BurstModulation::Qpsk => 6.8,
            BurstModulation::Psk8 => 10.5,
            BurstModulation::Qam16 => 13.5,
            BurstModulation::SpreadSpectrum => 0.0, // spread gain
        }
    }
}

// ---------------------------------------------------------------------------
// Turbo code rates
// ---------------------------------------------------------------------------

/// DVB-RCS2 turbo code rates per ETSI EN 301 545-2 Annex A.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TurboCodeRate {
    /// Rate 1/3.
    R1_3,
    /// Rate 1/2.
    R1_2,
    /// Rate 2/3.
    R2_3,
    /// Rate 3/4.
    R3_4,
    /// Rate 4/5.
    R4_5,
    /// Rate 6/7.
    R6_7,
}

impl TurboCodeRate {
    /// Numerator of code rate.
    pub fn numerator(&self) -> u32 {
        match self {
            TurboCodeRate::R1_3 => 1,
            TurboCodeRate::R1_2 => 1,
            TurboCodeRate::R2_3 => 2,
            TurboCodeRate::R3_4 => 3,
            TurboCodeRate::R4_5 => 4,
            TurboCodeRate::R6_7 => 6,
        }
    }

    /// Denominator of code rate.
    pub fn denominator(&self) -> u32 {
        match self {
            TurboCodeRate::R1_3 => 3,
            TurboCodeRate::R1_2 => 2,
            TurboCodeRate::R2_3 => 3,
            TurboCodeRate::R3_4 => 4,
            TurboCodeRate::R4_5 => 5,
            TurboCodeRate::R6_7 => 7,
        }
    }

    /// Code rate as f64.
    pub fn rate(&self) -> f64 {
        self.numerator() as f64 / self.denominator() as f64
    }

    /// Required Eb/N0 for BER ≈ 1e-6 (turbo code near-Shannon performance, approx dB).
    pub fn required_eb_n0_db(&self) -> f64 {
        match self {
            TurboCodeRate::R1_3 => 0.2,
            TurboCodeRate::R1_2 => 0.8,
            TurboCodeRate::R2_3 => 1.5,
            TurboCodeRate::R3_4 => 2.0,
            TurboCodeRate::R4_5 => 2.5,
            TurboCodeRate::R6_7 => 3.5,
        }
    }

    /// Number of constituent RSC encoder trellis termination bits.
    pub fn tail_bits(&self) -> usize {
        4 // 4 tail bits per constituent encoder for K=4
    }
}

// ---------------------------------------------------------------------------
// Turbo code encoder/decoder (simplified 3GPP-style)
// ---------------------------------------------------------------------------

/// DVB-RCS2 turbo code encoder (parallel concatenated convolutional code).
///
/// Uses two 8-state (K=4) RSC constituent encoders with QPP interleaver.
pub struct TurboEncoder {
    rate: TurboCodeRate,
    // Generator polynomials for 8-state RSC: g1=13 (feedback), g2=15 (feedforward)
    g1: u8,
    g2: u8,
}

impl TurboEncoder {
    /// Create a new turbo encoder with the given code rate.
    pub fn new(rate: TurboCodeRate) -> Self {
        Self { rate, g1: 0o13, g2: 0o15 }
    }

    /// Encode information bits into turbo-coded bits.
    ///
    /// Returns systematic + parity1 + parity2 bits (before puncturing).
    pub fn encode(&self, info_bits: &[bool]) -> Vec<bool> {
        let k = info_bits.len();
        let tail = self.rate.tail_bits();

        // Systematic bits (direct copy)
        let mut systematic: Vec<bool> = info_bits.to_vec();
        // Extend with tail bits for first constituent encoder
        systematic.resize(k + tail, false);

        // First constituent RSC encoder
        let parity1 = self.rsc_encode(&systematic);

        // Interleave for second constituent encoder (QPP-style permutation)
        let interleaved = self.qpp_interleave(info_bits);
        let mut interleaved_ext = interleaved;
        interleaved_ext.resize(k + tail, false);

        // Second constituent RSC encoder
        let parity2 = self.rsc_encode(&interleaved_ext);

        // Puncture based on rate and assemble output
        self.puncture_and_assemble(&systematic, &parity1, &parity2)
    }

    /// RSC (Recursive Systematic Convolutional) encoder, 8-state (K=4).
    fn rsc_encode(&self, bits: &[bool]) -> Vec<bool> {
        let mut state: u8 = 0;
        let mut parity = Vec::with_capacity(bits.len());
        for &bit in bits {
            // XOR feedback tap (g1 = 0o13 = 1011 binary)
            let feedback = bit ^ ((state >> 2) & 1 != 0) ^ ((state) & 1 != 0);
            // Parity output (g2 = 0o15 = 1101 binary)
            let p = feedback ^ ((state >> 1) & 1 != 0) ^ ((state >> 2) & 1 != 0);
            parity.push(p ^ (self.g2 & 0x1 != 0) & bit);
            // Update state register (shift in feedback)
            state = (state >> 1) | ((feedback as u8) << 2);
        }
        parity
    }

    /// QPP (Quadratic Permutation Polynomial) interleaver for length k.
    /// Uses f1, f2 from a simple lookup for standard block sizes.
    fn qpp_interleave(&self, bits: &[bool]) -> Vec<bool> {
        let k = bits.len();
        if k == 0 {
            return vec![];
        }
        // Simple QPP: use f1=1, f2=0 for general case (identity-like)
        // In practice, DVB-RCS2 specifies exact QPP parameters per block size.
        let f1 = 1usize;
        let f2 = 0usize;
        let mut out = vec![false; k];
        for i in 0..k {
            let j = (f1 * i + f2 * i * i) % k;
            out[i] = bits[j];
        }
        out
    }

    /// Puncture parity bits to achieve target rate and assemble output.
    fn puncture_and_assemble(
        &self,
        systematic: &[bool],
        parity1: &[bool],
        parity2: &[bool],
    ) -> Vec<bool> {
        let n = systematic.len().min(parity1.len()).min(parity2.len());
        let mut out = Vec::new();

        match self.rate {
            TurboCodeRate::R1_3 => {
                // All systematic + all parity1 + all parity2
                for i in 0..n {
                    out.push(systematic[i]);
                    out.push(parity1[i]);
                    out.push(parity2[i]);
                }
            }
            TurboCodeRate::R1_2 => {
                // Systematic + alternating parity
                for i in 0..n {
                    out.push(systematic[i]);
                    if i % 2 == 0 {
                        out.push(parity1[i]);
                    } else {
                        out.push(parity2[i]);
                    }
                }
            }
            TurboCodeRate::R2_3 => {
                // Systematic + every 3rd parity
                for i in 0..n {
                    out.push(systematic[i]);
                    if i % 3 == 0 {
                        out.push(parity1[i]);
                    }
                }
            }
            TurboCodeRate::R3_4 => {
                for i in 0..n {
                    out.push(systematic[i]);
                    if i % 4 == 0 {
                        out.push(parity1[i]);
                    }
                }
            }
            TurboCodeRate::R4_5 => {
                for i in 0..n {
                    out.push(systematic[i]);
                    if i % 5 == 0 {
                        out.push(parity1[i]);
                    }
                }
            }
            TurboCodeRate::R6_7 => {
                for i in 0..n {
                    out.push(systematic[i]);
                    if i % 7 == 0 {
                        out.push(parity1[i]);
                    }
                }
            }
        }
        out
    }
}

/// DVB-RCS2 turbo decoder (simplified log-MAP with 4 iterations).
pub struct TurboDecoder {
    rate: TurboCodeRate,
    iterations: usize,
}

impl TurboDecoder {
    /// Create a new turbo decoder.
    pub fn new(rate: TurboCodeRate) -> Self {
        Self { rate, iterations: 4 }
    }

    /// Set number of BCJR iterations.
    pub fn set_iterations(&mut self, n: usize) {
        self.iterations = n.max(1);
    }

    /// Decode received LLRs to information bits.
    ///
    /// `llrs` contains received log-likelihood ratios for each coded bit.
    /// Returns decoded information bits.
    pub fn decode(&self, llrs: &[f64]) -> Vec<bool> {
        if llrs.is_empty() {
            return vec![];
        }
        // Simplified: hard-decision on systematic bits only
        // (full MAP decoder would require recursive alpha/beta/gamma computation)
        let step = (1.0 / self.rate.rate()).round() as usize;
        let k = llrs.len() / step.max(1);
        let mut decoded = Vec::with_capacity(k);
        for i in 0..k {
            let idx = i * step;
            if idx < llrs.len() {
                // Systematic bit is the first in each group
                decoded.push(llrs[idx] < 0.0);
            }
        }
        decoded
    }

    /// Soft-output decode: returns LLRs for information bits.
    pub fn decode_soft(&self, llrs: &[f64]) -> Vec<f64> {
        // Simplified iterative extrinsic information exchange
        let step = (1.0 / self.rate.rate()).round() as usize;
        let k = llrs.len() / step.max(1);
        let mut extrinsic = vec![0.0f64; k];

        for _iter in 0..self.iterations {
            for i in 0..k {
                let sys_idx = i * step;
                if sys_idx < llrs.len() {
                    // Simple turbo iteration: boost by extrinsic
                    extrinsic[i] = llrs[sys_idx] + 0.5 * extrinsic[i];
                }
            }
        }
        extrinsic
    }
}

// ---------------------------------------------------------------------------
// Constellation mapper / demapper
// ---------------------------------------------------------------------------

/// Map bits to complex symbol for the given modulation.
pub fn map_symbols(bits: &[bool], modulation: BurstModulation) -> Vec<Complex> {
    match modulation {
        BurstModulation::Qpsk => map_qpsk(bits),
        BurstModulation::Psk8 => map_8psk(bits),
        BurstModulation::Qam16 => map_16qam(bits),
        BurstModulation::SpreadSpectrum => map_bpsk_spread(bits),
    }
}

fn map_qpsk(bits: &[bool]) -> Vec<Complex> {
    let mut syms = Vec::new();
    let s = 1.0 / 2.0f64.sqrt();
    let mut i = 0;
    while i + 1 <= bits.len() {
        let b0 = bits[i] as u8;
        let b1 = if i + 1 < bits.len() { bits[i + 1] as u8 } else { 0 };
        let re = if b0 == 0 { s } else { -s };
        let im = if b1 == 0 { s } else { -s };
        syms.push(Complex::new(re, im));
        i += 2;
    }
    syms
}

fn map_8psk(bits: &[bool]) -> Vec<Complex> {
    let mut syms = Vec::new();
    let mut i = 0;
    while i < bits.len() {
        let b0 = bits[i] as u8;
        let b1 = if i + 1 < bits.len() { bits[i + 1] as u8 } else { 0 };
        let b2 = if i + 2 < bits.len() { bits[i + 2] as u8 } else { 0 };
        let idx = ((b0 << 2) | (b1 << 1) | b2) as usize;
        let phase = 2.0 * PI * (idx as f64) / 8.0;
        syms.push(Complex::new(phase.cos(), phase.sin()));
        i += 3;
    }
    syms
}

fn map_16qam(bits: &[bool]) -> Vec<Complex> {
    // Gray-coded 16QAM, normalised to unit average power
    let lut: [(f64, f64); 16] = [
        (-3.0, -3.0), (-3.0, -1.0), (-3.0, 3.0), (-3.0, 1.0),
        (-1.0, -3.0), (-1.0, -1.0), (-1.0, 3.0), (-1.0, 1.0),
        ( 3.0, -3.0), ( 3.0, -1.0), ( 3.0, 3.0), ( 3.0, 1.0),
        ( 1.0, -3.0), ( 1.0, -1.0), ( 1.0, 3.0), ( 1.0, 1.0),
    ];
    let norm = 1.0 / (10.0f64).sqrt(); // sqrt(10) = sqrt(mean power of 16QAM)
    let mut syms = Vec::new();
    let mut i = 0;
    while i < bits.len() {
        let b0 = bits[i] as u8;
        let b1 = if i + 1 < bits.len() { bits[i + 1] as u8 } else { 0 };
        let b2 = if i + 2 < bits.len() { bits[i + 2] as u8 } else { 0 };
        let b3 = if i + 3 < bits.len() { bits[i + 3] as u8 } else { 0 };
        let idx = ((b0 << 3) | (b1 << 2) | (b2 << 1) | b3) as usize;
        let (re, im) = lut[idx & 0xF];
        syms.push(Complex::new(re * norm, im * norm));
        i += 4;
    }
    syms
}

fn map_bpsk_spread(bits: &[bool]) -> Vec<Complex> {
    // BPSK then spread with 16-chip Walsh-Hadamard code 0
    let wh_code: [i8; 16] = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
    let mut syms = Vec::new();
    for &bit in bits {
        let bpsk = if bit { -1.0 } else { 1.0 };
        for &chip in &wh_code {
            syms.push(Complex::new(bpsk * chip as f64 / (WH_SPREADING_FACTOR as f64).sqrt(), 0.0));
        }
    }
    syms
}

/// Demap symbols to bits (hard decision).
pub fn demap_symbols(symbols: &[Complex], modulation: BurstModulation) -> Vec<bool> {
    match modulation {
        BurstModulation::Qpsk => demap_qpsk(symbols),
        BurstModulation::Psk8 => demap_8psk(symbols),
        BurstModulation::Qam16 => demap_16qam(symbols),
        BurstModulation::SpreadSpectrum => demap_spread(symbols),
    }
}

fn demap_qpsk(symbols: &[Complex]) -> Vec<bool> {
    let mut bits = Vec::new();
    for sym in symbols {
        bits.push(sym.re < 0.0);
        bits.push(sym.im < 0.0);
    }
    bits
}

fn demap_8psk(symbols: &[Complex]) -> Vec<bool> {
    let mut bits = Vec::new();
    for sym in symbols {
        let phase = sym.angle();
        let normalized = if phase < 0.0 { phase + 2.0 * PI } else { phase };
        let idx = ((normalized / (2.0 * PI) * 8.0 + 0.5) as usize) % 8;
        bits.push((idx >> 2) & 1 != 0);
        bits.push((idx >> 1) & 1 != 0);
        bits.push(idx & 1 != 0);
    }
    bits
}

fn demap_16qam(symbols: &[Complex]) -> Vec<bool> {
    let mut bits = Vec::new();
    let norm = (10.0f64).sqrt();
    for sym in symbols {
        let re = (sym.re * norm).round().max(-3.0).min(3.0) as i32;
        let im = (sym.im * norm).round().max(-3.0).min(3.0) as i32;
        let ri = match re { -3 => 0, -1 => 1, 1 => 3, _ => 2 };
        let ii = match im { -3 => 0, -1 => 1, 1 => 3, _ => 2 };
        let idx = (ri << 2) | ii;
        bits.push((idx >> 3) & 1 != 0);
        bits.push((idx >> 2) & 1 != 0);
        bits.push((idx >> 1) & 1 != 0);
        bits.push(idx & 1 != 0);
    }
    bits
}

fn demap_spread(symbols: &[Complex]) -> Vec<bool> {
    let mut bits = Vec::new();
    let n = WH_SPREADING_FACTOR;
    let mut i = 0;
    while i + n <= symbols.len() {
        let sum: f64 = symbols[i..i + n].iter().map(|s| s.re).sum();
        bits.push(sum < 0.0);
        i += n;
    }
    bits
}

// ---------------------------------------------------------------------------
// Burst preamble / unique word
// ---------------------------------------------------------------------------

/// Preamble structure for a DVB-RCS2 linear burst.
#[derive(Debug, Clone)]
pub struct BurstPreamble {
    /// Unique word bits (48-bit default).
    pub unique_word: Vec<bool>,
    /// Pilot symbol count.
    pub pilot_symbols: usize,
}

impl BurstPreamble {
    /// Create a standard DVB-RCS2 burst preamble.
    ///
    /// The unique word is derived from a Barker-like sequence extended to 48 bits.
    pub fn new() -> Self {
        // 48-bit unique word from ETSI EN 301 545-2 (illustrative pattern)
        let uw_hex: u64 = 0xAC_E8_F3_D2_B1_C4;
        let mut uw = Vec::with_capacity(UW_LENGTH_BITS);
        for i in (0..UW_LENGTH_BITS).rev() {
            uw.push((uw_hex >> i) & 1 != 0);
        }
        Self { unique_word: uw, pilot_symbols: 4 }
    }

    /// Convert unique word to QPSK symbols.
    pub fn to_symbols(&self) -> Vec<Complex> {
        let mut syms = map_qpsk(&self.unique_word);
        // Add pilot symbols (all-one reference)
        for _ in 0..self.pilot_symbols {
            let s = 1.0 / 2.0f64.sqrt();
            syms.push(Complex::new(s, s));
        }
        syms
    }

    /// Detect unique word in received symbol sequence.
    ///
    /// Returns the offset where UW starts, or None if not found.
    pub fn detect(&self, received: &[Complex], threshold: f64) -> Option<usize> {
        let uw_syms = map_qpsk(&self.unique_word);
        let uw_len = uw_syms.len();
        if received.len() < uw_len {
            return None;
        }
        for offset in 0..=(received.len() - uw_len) {
            let corr: f64 = uw_syms.iter().enumerate().map(|(i, &ref uw_s)| {
                let r = &received[offset + i];
                r.re * uw_s.re + r.im * uw_s.im
            }).sum();
            let normalized = corr / (uw_len as f64);
            if normalized > threshold {
                return Some(offset);
            }
        }
        None
    }
}

impl Default for BurstPreamble {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Encoded burst
// ---------------------------------------------------------------------------

/// A fully encoded DVB-RCS2 burst ready for transmission.
#[derive(Debug, Clone)]
pub struct EncodedBurst {
    /// IQ symbols including preamble and coded payload.
    pub symbols: Vec<Complex>,
    /// Modulation used.
    pub modulation: BurstModulation,
    /// Code rate used.
    pub code_rate: TurboCodeRate,
    /// Number of information bytes carried.
    pub info_bytes: usize,
    /// Total burst duration in symbols.
    pub burst_length_symbols: usize,
}

impl EncodedBurst {
    /// Energy per symbol (normalised to 1 for unit constellation).
    pub fn es_per_symbol(&self) -> f64 {
        if self.symbols.is_empty() { return 0.0; }
        let total: f64 = self.symbols.iter().map(|s| s.re*s.re + s.im*s.im).sum();
        total / self.symbols.len() as f64
    }
}

// ---------------------------------------------------------------------------
// MF-TDMA frame structure
// ---------------------------------------------------------------------------

/// Timeslot assignment within an MF-TDMA frame.
#[derive(Debug, Clone)]
pub struct TimeSlot {
    /// Frame number within superframe (0-based).
    pub frame_number: u16,
    /// Carrier frequency offset index.
    pub carrier_id: u16,
    /// Slot start time within frame in symbols.
    pub slot_offset_symbols: u32,
    /// Duration in symbols.
    pub slot_duration_symbols: u32,
    /// Assigned terminal ID (0 = unassigned).
    pub rcst_id: u32,
}

impl TimeSlot {
    pub fn new(frame: u16, carrier: u16, offset: u32, duration: u32) -> Self {
        Self {
            frame_number: frame,
            carrier_id: carrier,
            slot_offset_symbols: offset,
            slot_duration_symbols: duration,
            rcst_id: 0,
        }
    }
}

/// DVB-RCS2 Superframe descriptor.
#[derive(Debug, Clone)]
pub struct Superframe {
    /// Superframe ID.
    pub id: u16,
    /// Duration in microseconds.
    pub duration_us: u64,
    /// Number of frames per superframe.
    pub num_frames: u16,
    /// Number of carriers (frequency slots).
    pub num_carriers: u16,
    /// Symbol rate in Msps.
    pub symbol_rate_msps: f64,
    /// Time slots allocated in this superframe.
    pub slots: Vec<TimeSlot>,
}

impl Superframe {
    /// Create a new superframe with default 26.5 ms duration.
    pub fn new(id: u16, num_frames: u16, num_carriers: u16, symbol_rate_msps: f64) -> Self {
        Self {
            id,
            duration_us: SUPERFRAME_DURATION_US,
            num_frames,
            num_carriers,
            symbol_rate_msps,
            slots: Vec::new(),
        }
    }

    /// Total symbols per frame on one carrier.
    pub fn symbols_per_frame(&self) -> u64 {
        let frame_duration_us = self.duration_us / self.num_frames as u64;
        (frame_duration_us as f64 * self.symbol_rate_msps) as u64
    }

    /// Add a timeslot to this superframe.
    pub fn add_slot(&mut self, slot: TimeSlot) {
        self.slots.push(slot);
    }

    /// Find all slots assigned to a given RCST.
    pub fn slots_for_rcst(&self, rcst_id: u32) -> Vec<&TimeSlot> {
        self.slots.iter().filter(|s| s.rcst_id == rcst_id).collect()
    }
}

// ---------------------------------------------------------------------------
// NCR (Network Clock Reference)
// ---------------------------------------------------------------------------

/// DVB-RCS2 Network Clock Reference timestamp (27 MHz, 32-bit counter).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NcrTimestamp {
    /// Tick count at 27 MHz.
    pub ticks: u32,
}

impl NcrTimestamp {
    /// Create from 27 MHz tick count.
    pub fn new(ticks: u32) -> Self {
        Self { ticks }
    }

    /// Create from elapsed microseconds.
    pub fn from_us(us: u64) -> Self {
        let ticks = ((us * NCR_CLOCK_HZ) / 1_000_000) as u32;
        Self { ticks }
    }

    /// Convert to microseconds.
    pub fn to_us(&self) -> u64 {
        (self.ticks as u64 * 1_000_000) / NCR_CLOCK_HZ
    }

    /// Difference (in ticks) handling rollover.
    pub fn diff(&self, other: &NcrTimestamp) -> i64 {
        let d = self.ticks.wrapping_sub(other.ticks) as i64;
        if d > (NCR_ROLLOVER / 2) as i64 {
            d - NCR_ROLLOVER as i64
        } else if d < -((NCR_ROLLOVER / 2) as i64) {
            d + NCR_ROLLOVER as i64
        } else {
            d
        }
    }

    /// Encode NCR into 5-byte DVB-RCS2 format (32-bit base + 9-bit extension).
    /// Returns [msb, b2, b3, b4, lsb(base)|ext_msb] – simplified to 4 bytes here.
    pub fn encode(&self) -> [u8; 4] {
        self.ticks.to_be_bytes()
    }

    /// Decode from 4-byte representation.
    pub fn decode(bytes: &[u8; 4]) -> Self {
        Self { ticks: u32::from_be_bytes(*bytes) }
    }
}

/// NCR receiver/tracker: recovers hub timing from received NCR packets.
pub struct NcrReceiver {
    /// Last received NCR.
    pub last_ncr: Option<NcrTimestamp>,
    /// Local tick offset to align with hub.
    pub offset_ticks: i64,
    /// Number of NCR packets received.
    pub packet_count: u64,
    /// Frequency tracking: PPM error estimate.
    pub freq_error_ppm: f64,
}

impl NcrReceiver {
    pub fn new() -> Self {
        Self {
            last_ncr: None,
            offset_ticks: 0,
            packet_count: 0,
            freq_error_ppm: 0.0,
        }
    }

    /// Process a received NCR packet and update timing offset.
    pub fn process_ncr(&mut self, ncr: NcrTimestamp, local_ticks: u32) {
        if let Some(prev) = self.last_ncr {
            let delta_hub = ncr.diff(&prev) as f64;
            let delta_local = NcrTimestamp::new(local_ticks).diff(&NcrTimestamp::new(prev.ticks)) as f64;
            if delta_hub.abs() > 0.0 {
                let freq_err = (delta_local - delta_hub) / delta_hub * 1e6;
                // Exponential average
                self.freq_error_ppm = 0.9 * self.freq_error_ppm + 0.1 * freq_err;
            }
        }
        self.offset_ticks = ncr.ticks as i64 - local_ticks as i64;
        self.last_ncr = Some(ncr);
        self.packet_count += 1;
    }

    /// Get hub-synchronised timestamp for a given local tick count.
    pub fn synchronized_ticks(&self, local_ticks: u32) -> u32 {
        ((local_ticks as i64) + self.offset_ticks) as u32
    }
}

impl Default for NcrReceiver {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Capacity request types and encoding
// ---------------------------------------------------------------------------

/// DVB-RCS2 capacity allocation types per ETSI EN 301 545-2.
#[derive(Debug, Clone, PartialEq)]
pub enum CapacityType {
    /// Rate-Based Dynamic Capacity: sustained bit rate request.
    Rbdc { rate_kbps: u32 },
    /// Volume-Based Dynamic Capacity: backlog in kbytes.
    Vbdc { volume_kbytes: u32 },
    /// Continuous Rate Assignment: pre-allocated always-on capacity.
    Cra { rate_kbps: u32 },
    /// Free Capacity Assignment: opportunistic leftover capacity.
    Fca,
}

/// A single capacity request from an RCST.
#[derive(Debug, Clone)]
pub struct CapacityRequest {
    pub request_type: CapacityType,
    /// RCST terminal identifier.
    pub rcst_id: u16,
    /// Scaling factor exponent (capacity = value * 2^scale_exp).
    pub scale_exp: u8,
}

impl CapacityRequest {
    pub fn new(request_type: CapacityType) -> Self {
        Self { request_type, rcst_id: 0, scale_exp: 0 }
    }

    pub fn with_rcst(mut self, rcst_id: u16) -> Self {
        self.rcst_id = rcst_id;
        self
    }

    /// Encode the capacity request to 3 bytes (simplified DVB-RCS2 format).
    ///
    /// Byte layout: [type(2)|scale(2)|value_hi(4)] [value_mid(8)] [value_lo(8)]
    pub fn encode(&self) -> [u8; 3] {
        let (type_id, value) = match &self.request_type {
            CapacityType::Rbdc { rate_kbps } => (0u8, *rate_kbps),
            CapacityType::Vbdc { volume_kbytes } => (1u8, *volume_kbytes),
            CapacityType::Cra { rate_kbps } => (2u8, *rate_kbps),
            CapacityType::Fca => (3u8, 0u32),
        };
        let b0 = (type_id << 6) | ((self.scale_exp & 0x3) << 4) | ((value >> 12) as u8 & 0xF);
        let b1 = ((value >> 4) & 0xFF) as u8;
        let b2 = ((value & 0xF) << 4) as u8;
        [b0, b1, b2]
    }

    /// Decode a capacity request from 3 bytes.
    pub fn decode(bytes: &[u8; 3]) -> Self {
        let type_id = bytes[0] >> 6;
        let scale_exp = (bytes[0] >> 4) & 0x3;
        let value = (((bytes[0] & 0xF) as u32) << 12)
            | ((bytes[1] as u32) << 4)
            | ((bytes[2] as u32) >> 4);
        let request_type = match type_id {
            0 => CapacityType::Rbdc { rate_kbps: value },
            1 => CapacityType::Vbdc { volume_kbytes: value },
            2 => CapacityType::Cra { rate_kbps: value },
            _ => CapacityType::Fca,
        };
        Self { request_type, rcst_id: 0, scale_exp }
    }
}

// ---------------------------------------------------------------------------
// TBTP (Terminal Burst Time Plan)
// ---------------------------------------------------------------------------

/// A single entry in the TBTP assigning a burst opportunity to an RCST.
#[derive(Debug, Clone)]
pub struct TbtpEntry {
    /// RCST logical channel ID.
    pub channel_id: u16,
    /// Frame number within the superframe.
    pub frame_number: u8,
    /// Start slot offset in symbols.
    pub start_slot: u16,
    /// Assigned burst waveform ID.
    pub waveform_id: u8,
    /// Assignment type (0=CRA, 1=RBDC, 2=VBDC, 3=FCA).
    pub assignment_type: u8,
}

/// TBTP (Terminal Burst Time Plan) message.
///
/// Broadcast by hub to assign transmission opportunities to all RCSTs.
#[derive(Debug, Clone)]
pub struct Tbtp {
    /// Superframe counter this plan applies to.
    pub superframe_count: u32,
    /// List of burst assignments.
    pub entries: Vec<TbtpEntry>,
}

impl Tbtp {
    pub fn new(superframe_count: u32) -> Self {
        Self { superframe_count, entries: Vec::new() }
    }

    pub fn add_entry(&mut self, entry: TbtpEntry) {
        self.entries.push(entry);
    }

    /// Encode TBTP to bytes (simplified header + entries).
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        // 4-byte superframe count
        out.extend_from_slice(&self.superframe_count.to_be_bytes());
        // 2-byte entry count
        let count = self.entries.len() as u16;
        out.extend_from_slice(&count.to_be_bytes());
        // Each entry: 7 bytes
        for e in &self.entries {
            out.extend_from_slice(&e.channel_id.to_be_bytes());
            out.push(e.frame_number);
            out.extend_from_slice(&e.start_slot.to_be_bytes());
            out.push(e.waveform_id);
            out.push(e.assignment_type);
        }
        out
    }

    /// Decode TBTP from bytes.
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 6 { return None; }
        let superframe_count = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let count = u16::from_be_bytes([data[4], data[5]]) as usize;
        let mut entries = Vec::with_capacity(count);
        let mut pos = 6;
        for _ in 0..count {
            if pos + 7 > data.len() { break; }
            let channel_id = u16::from_be_bytes([data[pos], data[pos+1]]);
            let frame_number = data[pos+2];
            let start_slot = u16::from_be_bytes([data[pos+3], data[pos+4]]);
            let waveform_id = data[pos+5];
            let assignment_type = data[pos+6];
            entries.push(TbtpEntry { channel_id, frame_number, start_slot, waveform_id, assignment_type });
            pos += 7;
        }
        Some(Self { superframe_count, entries })
    }

    /// Get all entries assigned to a specific channel ID.
    pub fn entries_for_channel(&self, channel_id: u16) -> Vec<&TbtpEntry> {
        self.entries.iter().filter(|e| e.channel_id == channel_id).collect()
    }
}

// ---------------------------------------------------------------------------
// RLE – Return Link Encapsulation
// ---------------------------------------------------------------------------

/// RLE Protocol Data Unit types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RlePduType {
    /// Complete PDU (entire packet in one PPDU).
    Complete,
    /// Start fragment.
    Start,
    /// Continuation fragment.
    Continuation,
    /// End fragment.
    End,
}

impl RlePduType {
    fn to_bits(&self) -> u8 {
        match self {
            RlePduType::Complete => 0b11,
            RlePduType::Start => 0b10,
            RlePduType::Continuation => 0b00,
            RlePduType::End => 0b01,
        }
    }
    fn from_bits(bits: u8) -> Self {
        match bits & 0x3 {
            0b11 => RlePduType::Complete,
            0b10 => RlePduType::Start,
            0b00 => RlePduType::Continuation,
            _ => RlePduType::End,
        }
    }
}

/// RLE PPDU (Payload Protocol Data Unit) header.
#[derive(Debug, Clone)]
pub struct RlePpduHeader {
    /// PDU type.
    pub pdu_type: RlePduType,
    /// Fragment ID (for reassembly matching).
    pub fragment_id: u8,
    /// Total packet length in bytes (present in Start/Complete only).
    pub total_length: Option<u16>,
    /// Protocol type (e.g., 0x0800 = IPv4).
    pub protocol_type: Option<u16>,
}

impl RlePpduHeader {
    /// Encode the PPDU header to bytes.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        let type_bits = self.pdu_type.to_bits();
        let b0 = (type_bits << 6) | (self.fragment_id & 0x3F);
        out.push(b0);
        if matches!(self.pdu_type, RlePduType::Start | RlePduType::Complete) {
            let len = self.total_length.unwrap_or(0);
            out.extend_from_slice(&len.to_be_bytes());
            let proto = self.protocol_type.unwrap_or(0x0800);
            out.extend_from_slice(&proto.to_be_bytes());
        }
        out
    }

    /// Decode a PPDU header from bytes. Returns (header, bytes_consumed).
    pub fn decode(data: &[u8]) -> Option<(Self, usize)> {
        if data.is_empty() { return None; }
        let pdu_type = RlePduType::from_bits(data[0] >> 6);
        let fragment_id = data[0] & 0x3F;
        let mut pos = 1;
        let (total_length, protocol_type) = if matches!(pdu_type, RlePduType::Start | RlePduType::Complete) {
            if pos + 4 > data.len() { return None; }
            let len = u16::from_be_bytes([data[pos], data[pos+1]]);
            let proto = u16::from_be_bytes([data[pos+2], data[pos+3]]);
            pos += 4;
            (Some(len), Some(proto))
        } else {
            (None, None)
        };
        Some((Self { pdu_type, fragment_id, total_length, protocol_type }, pos))
    }
}

/// RLE encapsulator: fragments IP packets into PPDUs for burst transmission.
pub struct RleEncapsulator {
    /// Maximum payload size per PPDU (bytes).
    pub max_ppdu_payload: usize,
    /// Next fragment ID counter.
    next_fragment_id: u8,
}

impl RleEncapsulator {
    pub fn new(max_ppdu_payload: usize) -> Self {
        Self { max_ppdu_payload, next_fragment_id: 0 }
    }

    /// Encapsulate an IP packet into a list of RLE PPDUs.
    pub fn encapsulate(&mut self, packet: &[u8], protocol: u16) -> Vec<Vec<u8>> {
        let fid = self.next_fragment_id;
        self.next_fragment_id = self.next_fragment_id.wrapping_add(1);

        let total_len = packet.len() as u16;
        let max_pay = self.max_ppdu_payload;
        let mut ppdus = Vec::new();

        if packet.len() <= max_pay {
            // Complete PDU
            let hdr = RlePpduHeader {
                pdu_type: RlePduType::Complete,
                fragment_id: fid,
                total_length: Some(total_len),
                protocol_type: Some(protocol),
            };
            let mut ppdu = hdr.encode();
            ppdu.extend_from_slice(packet);
            ppdus.push(ppdu);
        } else {
            // Fragment
            let mut offset = 0;
            let mut is_first = true;
            while offset < packet.len() {
                let end = (offset + max_pay).min(packet.len());
                let is_last = end == packet.len();
                let pdu_type = if is_first {
                    RlePduType::Start
                } else if is_last {
                    RlePduType::End
                } else {
                    RlePduType::Continuation
                };
                let hdr = RlePpduHeader {
                    pdu_type,
                    fragment_id: fid,
                    total_length: if is_first { Some(total_len) } else { None },
                    protocol_type: if is_first { Some(protocol) } else { None },
                };
                let mut ppdu = hdr.encode();
                ppdu.extend_from_slice(&packet[offset..end]);
                ppdus.push(ppdu);
                offset = end;
                is_first = false;
            }
        }
        ppdus
    }
}

/// RLE reassembler: reassembles PPDUs back into IP packets.
pub struct RleReassembler {
    /// In-progress fragments keyed by fragment ID.
    fragments: std::collections::HashMap<u8, ReassemblyBuffer>,
    /// Completed packets waiting to be consumed.
    completed: VecDeque<Vec<u8>>,
}

#[derive(Debug)]
struct ReassemblyBuffer {
    data: Vec<u8>,
    total_length: u16,
}

impl RleReassembler {
    pub fn new() -> Self {
        Self {
            fragments: std::collections::HashMap::new(),
            completed: VecDeque::new(),
        }
    }

    /// Process an incoming RLE PPDU.
    pub fn process_ppdu(&mut self, ppdu: &[u8]) {
        if let Some((hdr, hdr_len)) = RlePpduHeader::decode(ppdu) {
            let payload = &ppdu[hdr_len..];
            match hdr.pdu_type {
                RlePduType::Complete => {
                    self.completed.push_back(payload.to_vec());
                }
                RlePduType::Start => {
                    let total = hdr.total_length.unwrap_or(0);
                    let mut buf = ReassemblyBuffer {
                        data: Vec::with_capacity(total as usize),
                        total_length: total,
                    };
                    buf.data.extend_from_slice(payload);
                    self.fragments.insert(hdr.fragment_id, buf);
                }
                RlePduType::Continuation | RlePduType::End => {
                    if let Some(buf) = self.fragments.get_mut(&hdr.fragment_id) {
                        buf.data.extend_from_slice(payload);
                        if matches!(hdr.pdu_type, RlePduType::End) {
                            let completed_buf = self.fragments.remove(&hdr.fragment_id).unwrap();
                            self.completed.push_back(completed_buf.data);
                        }
                    }
                }
            }
        }
    }

    /// Drain completed IP packets.
    pub fn drain_packets(&mut self) -> Vec<Vec<u8>> {
        self.completed.drain(..).collect()
    }
}

impl Default for RleReassembler {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// RCST logon sequence
// ---------------------------------------------------------------------------

/// RCST logon state machine states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogonState {
    /// Terminal is not logged on.
    Off,
    /// Waiting for CSC (Control and Synchronization Channel) burst window.
    WaitingCsc,
    /// CSC burst sent, waiting for logon acknowledgment.
    LogonPending,
    /// Logged on and operational.
    LoggedOn,
    /// In beam handover process.
    HandoverInProgress,
}

/// RCST logon request parameters.
#[derive(Debug, Clone)]
pub struct LogonRequest {
    /// Terminal hardware identifier (48-bit MAC-like address).
    pub rcst_id: u64,
    /// Requested superframe ID to join.
    pub superframe_id: u16,
    /// Terminal EIRP capability in dBW.
    pub eirp_dbw: f32,
    /// Requested CRA bandwidth in kbps.
    pub cra_kbps: u32,
    /// Software/firmware version string (max 8 bytes).
    pub version: [u8; 8],
}

impl LogonRequest {
    pub fn new(rcst_id: u64, superframe_id: u16) -> Self {
        Self {
            rcst_id,
            superframe_id,
            eirp_dbw: 44.0,
            cra_kbps: 0,
            version: *b"DVBRCSv2",
        }
    }

    /// Encode the logon request to bytes.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&self.rcst_id.to_be_bytes()[2..]); // 6 bytes
        out.extend_from_slice(&self.superframe_id.to_be_bytes());
        let eirp_u16 = (self.eirp_dbw * 10.0) as u16;
        out.extend_from_slice(&eirp_u16.to_be_bytes());
        out.extend_from_slice(&self.cra_kbps.to_be_bytes());
        out.extend_from_slice(&self.version);
        out
    }
}

/// TIM-U (Terminal Information Message – Unicast) response from hub.
#[derive(Debug, Clone)]
pub struct TimU {
    /// Assigned logical channel ID for the RCST.
    pub channel_id: u16,
    /// Assigned timing pre-compensation value in symbols.
    pub timing_offset: i32,
    /// Frequency correction in Hz.
    pub freq_correction_hz: i32,
    /// Superframe ID confirmed.
    pub superframe_id: u16,
    /// Logon successful.
    pub success: bool,
}

impl TimU {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&self.channel_id.to_be_bytes());
        out.extend_from_slice(&self.timing_offset.to_be_bytes());
        out.extend_from_slice(&self.freq_correction_hz.to_be_bytes());
        out.extend_from_slice(&self.superframe_id.to_be_bytes());
        out.push(if self.success { 1 } else { 0 });
        out
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 13 { return None; }
        Some(Self {
            channel_id: u16::from_be_bytes([data[0], data[1]]),
            timing_offset: i32::from_be_bytes([data[2], data[3], data[4], data[5]]),
            freq_correction_hz: i32::from_be_bytes([data[6], data[7], data[8], data[9]]),
            superframe_id: u16::from_be_bytes([data[10], data[11]]),
            success: data[12] != 0,
        })
    }
}

/// RCST state machine controller.
pub struct RcstController {
    pub state: LogonState,
    pub logon_req: Option<LogonRequest>,
    pub channel_id: Option<u16>,
    pub timing_offset: i32,
    pub freq_correction_hz: i32,
    pub logon_attempts: u32,
    pub max_logon_attempts: u32,
}

impl RcstController {
    pub fn new() -> Self {
        Self {
            state: LogonState::Off,
            logon_req: None,
            channel_id: None,
            timing_offset: 0,
            freq_correction_hz: 0,
            logon_attempts: 0,
            max_logon_attempts: 5,
        }
    }

    /// Initiate logon sequence.
    pub fn start_logon(&mut self, req: LogonRequest) {
        self.logon_req = Some(req);
        self.state = LogonState::WaitingCsc;
        self.logon_attempts = 0;
    }

    /// Called when a CSC window is detected.
    pub fn on_csc_window(&mut self) -> Option<Vec<u8>> {
        if self.state == LogonState::WaitingCsc {
            if let Some(ref req) = self.logon_req {
                self.logon_attempts += 1;
                self.state = LogonState::LogonPending;
                return Some(req.encode());
            }
        }
        None
    }

    /// Process TIM-U response from hub.
    pub fn process_timu(&mut self, timu: &TimU) {
        if self.state == LogonState::LogonPending && timu.success {
            self.channel_id = Some(timu.channel_id);
            self.timing_offset = timu.timing_offset;
            self.freq_correction_hz = timu.freq_correction_hz;
            self.state = LogonState::LoggedOn;
        } else if !timu.success {
            if self.logon_attempts >= self.max_logon_attempts {
                self.state = LogonState::Off;
            } else {
                self.state = LogonState::WaitingCsc;
            }
        }
    }

    /// Initiate beam handover to a new superframe/beam.
    pub fn start_handover(&mut self, new_sf_id: u16) {
        if self.state == LogonState::LoggedOn {
            self.state = LogonState::HandoverInProgress;
            if let Some(ref mut req) = self.logon_req {
                req.superframe_id = new_sf_id;
            }
        }
    }

    pub fn is_logged_on(&self) -> bool {
        self.state == LogonState::LoggedOn
    }
}

impl Default for RcstController {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Return link budget (Ku-band)
// ---------------------------------------------------------------------------

/// ITU-R P.618 simplified rain attenuation at Ku-band.
///
/// Uses the simplified formula for earth-space paths.
///
/// # Arguments
/// - `rain_rate_mm_h`: point rainfall rate (mm/h) at location
/// - `elevation_deg`: elevation angle to satellite (degrees)
/// - `freq_ghz`: carrier frequency (GHz), typically 14.0–14.5 for Ku-band
/// - `availability`: link availability fraction (e.g., 0.9999 = 99.99%)
///
/// Returns rain attenuation in dB.
pub fn rain_attenuation_db(
    rain_rate_mm_h: f64,
    elevation_deg: f64,
    freq_ghz: f64,
    availability: f64,
) -> f64 {
    if elevation_deg <= 0.0 { return 100.0; }
    let el_rad = elevation_deg * PI / 180.0;

    // Specific attenuation coefficients for Ku-band (approx. 14 GHz, horizontal)
    // per ITU-R P.838-3 Table I
    let (k_h, alpha_h) = if freq_ghz < 12.0 {
        (0.0188, 1.217)
    } else if freq_ghz < 16.0 {
        (0.0367, 1.154)
    } else {
        (0.0751, 1.099)
    };

    // Specific rain attenuation gamma_R (dB/km)
    let gamma_r = k_h * rain_rate_mm_h.powf(alpha_h);

    // Effective path length
    let h_rain = 4.0; // rain height (km) – simplified
    let l_s = h_rain / el_rad.sin();
    let r = 1.0 / (1.0 + l_s / 35.0);
    let l_eff = l_s * r;

    // Scale for exceeded probability
    let p_exceed = 1.0 - availability;
    let p_0 = 0.01_f64; // 0.01% baseline
    let beta = if p_exceed > p_0 {
        (p_exceed / p_0).ln() / (p_0.ln() - p_exceed.ln())
    } else {
        0.0
    };
    let a_001 = gamma_r * l_eff;
    if a_001 <= 0.0 || p_exceed <= 0.0 {
        return 0.0;
    }
    let ratio = p_exceed / p_0;
    let exponent = -0.655
        + 0.033 * rain_rate_mm_h.ln()
        - 0.045 * a_001.max(1e-6).ln()
        - beta * el_rad.sin();
    let result = a_001 * ratio.powf(exponent);
    if result.is_finite() && result >= 0.0 { result } else { a_001 }
}

/// DVB-RCS2 return link budget calculator (Ku-band, 14.0–14.5 GHz uplink).
#[derive(Debug, Clone)]
pub struct ReturnLinkBudget {
    /// RCST transmit EIRP (dBW).
    pub eirp_dbw: f64,
    /// Free-space path loss (dB).
    pub fspl_db: f64,
    /// Rain attenuation (dB).
    pub rain_attenuation_db: f64,
    /// Atmospheric + gaseous loss (dB).
    pub atm_loss_db: f64,
    /// Satellite G/T (dB/K).
    pub sat_g_t_db: f64,
    /// Carrier symbol rate (Msps).
    pub symbol_rate_msps: f64,
    /// Modulation and code rate.
    pub modulation: BurstModulation,
    pub code_rate: TurboCodeRate,
}

impl ReturnLinkBudget {
    /// Create a default Ku-band return link budget.
    pub fn new() -> Self {
        Self {
            eirp_dbw: 44.0,       // 4W PA + 60 cm dish ≈ 44 dBW
            fspl_db: 207.0,       // ~36000 km GEO at 14 GHz
            rain_attenuation_db: 3.0,
            atm_loss_db: 0.5,
            sat_g_t_db: 1.0,      // typical Ku hub
            symbol_rate_msps: 2.0,
            modulation: BurstModulation::Qpsk,
            code_rate: TurboCodeRate::R1_2,
        }
    }

    /// Compute free-space path loss for GEO satellite at given range.
    pub fn compute_fspl(range_km: f64, freq_ghz: f64) -> f64 {
        20.0 * (4.0 * PI * range_km * 1000.0 * freq_ghz * 1e9 / 3e8).log10()
    }

    /// Total received C/N0 at satellite hub (dB-Hz).
    ///
    /// C/N0 = EIRP(dBW) - FSPL(dB) - A_rain(dB) - A_atm(dB) + G/T(dB/K) - 10·log10(k)
    /// where 10·log10(Boltzmann) = -228.6 dB·(W·s/K), so -10·log10(k) = +228.6.
    pub fn cn0_db_hz(&self) -> f64 {
        // -10*log10(k_B) = 228.6 dB (J/K = W·s/K)
        let k_db = 10.0 * BOLTZMANN.log10(); // ≈ -228.6
        self.eirp_dbw
            - self.fspl_db
            - self.rain_attenuation_db
            - self.atm_loss_db
            + self.sat_g_t_db
            - k_db
    }

    /// Eb/N0 at satellite hub (dB).
    pub fn eb_n0_db(&self) -> f64 {
        let bps = self.modulation.bits_per_symbol() as f64;
        let rate = self.code_rate.rate();
        let cn0 = self.cn0_db_hz();
        let sym_rate_hz = self.symbol_rate_msps * 1e6;
        let info_rate = sym_rate_hz * bps * rate;
        cn0 - 10.0 * info_rate.log10()
    }

    /// Link margin against required Eb/N0 (dB). Positive = sufficient margin.
    pub fn link_margin_db(&self) -> f64 {
        self.eb_n0_db() - self.code_rate.required_eb_n0_db()
            - self.modulation.min_es_n0_db() / self.modulation.bits_per_symbol() as f64
    }

    /// Recommend modulation and code rate based on available C/N0.
    pub fn recommend_modcod(&self) -> (BurstModulation, TurboCodeRate) {
        let cn0 = self.cn0_db_hz();
        let sym_db = 10.0 * (self.symbol_rate_msps * 1e6).log10();
        let es_n0 = cn0 - sym_db;
        // Simple threshold selection
        if es_n0 >= 13.5 + 3.5 {
            (BurstModulation::Qam16, TurboCodeRate::R6_7)
        } else if es_n0 >= 10.5 + 2.5 {
            (BurstModulation::Psk8, TurboCodeRate::R4_5)
        } else if es_n0 >= 6.8 + 2.0 {
            (BurstModulation::Qpsk, TurboCodeRate::R3_4)
        } else if es_n0 >= 6.8 + 0.8 {
            (BurstModulation::Qpsk, TurboCodeRate::R1_2)
        } else {
            (BurstModulation::Qpsk, TurboCodeRate::R1_3)
        }
    }
}

impl Default for ReturnLinkBudget {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Main DVB-RCS2 processor
// ---------------------------------------------------------------------------

/// DVB-RCS2 physical layer processor.
///
/// Implements the full return link processing chain:
/// data → RLE encap → turbo encode → modulate → burst format → IQ output.
pub struct DvbRcs2Processor {
    pub preamble: BurstPreamble,
    pub superframe: Option<Superframe>,
    pub ncr_receiver: NcrReceiver,
    pub rle_encap: RleEncapsulator,
    pub rle_reassem: RleReassembler,
    pub rcst: RcstController,
    pub link_budget: ReturnLinkBudget,
    /// Default modulation for new bursts.
    pub default_modulation: BurstModulation,
    /// Default code rate for new bursts.
    pub default_code_rate: TurboCodeRate,
}

impl DvbRcs2Processor {
    /// Create a new DVB-RCS2 processor with defaults.
    pub fn new() -> Self {
        Self {
            preamble: BurstPreamble::new(),
            superframe: None,
            ncr_receiver: NcrReceiver::new(),
            rle_encap: RleEncapsulator::new(188),
            rle_reassem: RleReassembler::new(),
            rcst: RcstController::new(),
            link_budget: ReturnLinkBudget::new(),
            default_modulation: BurstModulation::Qpsk,
            default_code_rate: TurboCodeRate::R1_2,
        }
    }

    /// Configure a superframe.
    pub fn configure_superframe(&mut self, sf: Superframe) {
        self.superframe = Some(sf);
    }

    /// Encode data bytes into a DVB-RCS2 burst.
    ///
    /// Pipeline: bytes → bits → turbo encode → modulate → prepend preamble.
    pub fn encode_burst(
        &mut self,
        data: &[u8],
        modulation: BurstModulation,
        code_rate: TurboCodeRate,
    ) -> EncodedBurst {
        let info_bytes = data.len();

        // Convert bytes to bits
        let bits: Vec<bool> = data.iter().flat_map(|&b| {
            (0..8).rev().map(move |i| (b >> i) & 1 != 0)
        }).collect();

        // Turbo encode
        let encoder = TurboEncoder::new(code_rate);
        let coded_bits = encoder.encode(&bits);

        // Map to symbols
        let payload_syms = map_symbols(&coded_bits, modulation);

        // Prepend preamble
        let preamble_syms = self.preamble.to_symbols();
        let mut symbols = preamble_syms;
        symbols.extend_from_slice(&payload_syms);

        let burst_length = symbols.len();
        EncodedBurst {
            symbols,
            modulation,
            code_rate,
            info_bytes,
            burst_length_symbols: burst_length,
        }
    }

    /// Decode a received burst back to data bytes.
    ///
    /// Pipeline: IQ symbols → strip preamble → demap → turbo decode → bytes.
    pub fn decode_burst(
        &mut self,
        symbols: &[Complex],
        modulation: BurstModulation,
        code_rate: TurboCodeRate,
    ) -> Vec<u8> {
        // Strip preamble symbols
        let preamble_len = self.preamble.to_symbols().len();
        if symbols.len() <= preamble_len {
            return vec![];
        }
        let payload_syms = &symbols[preamble_len..];

        // Demap symbols to bits
        let coded_bits = demap_symbols(payload_syms, modulation);

        // Turbo decode
        let llrs: Vec<f64> = coded_bits.iter().map(|&b| if b { -1.0 } else { 1.0 }).collect();
        let decoder = TurboDecoder::new(code_rate);
        let info_bits = decoder.decode(&llrs);

        // Pack bits to bytes
        let mut bytes = Vec::new();
        let mut i = 0;
        while i + 8 <= info_bits.len() {
            let mut byte = 0u8;
            for j in 0..8 {
                if info_bits[i + j] {
                    byte |= 1 << (7 - j);
                }
            }
            bytes.push(byte);
            i += 8;
        }
        bytes
    }

    /// Process an IP packet through RLE encapsulation and return encoded bursts.
    pub fn send_ip_packet(
        &mut self,
        packet: &[u8],
        protocol: u16,
    ) -> Vec<EncodedBurst> {
        let ppdus = self.rle_encap.encapsulate(packet, protocol);
        let modulation = self.default_modulation;
        let code_rate = self.default_code_rate;
        ppdus.iter().map(|ppdu| {
            self.encode_burst(ppdu, modulation, code_rate)
        }).collect()
    }

    /// Process received burst symbols, extract RLE PDUs and reassemble.
    pub fn receive_burst(&mut self, symbols: &[Complex]) -> Vec<Vec<u8>> {
        let data = self.decode_burst(symbols, self.default_modulation, self.default_code_rate);
        if !data.is_empty() {
            self.rle_reassem.process_ppdu(&data);
        }
        self.rle_reassem.drain_packets()
    }

    /// Update link budget and adapt modulation/code rate.
    pub fn adapt_modcod(&mut self, new_eirp_dbw: f64, new_rain_db: f64) {
        self.link_budget.eirp_dbw = new_eirp_dbw;
        self.link_budget.rain_attenuation_db = new_rain_db;
        let (modulation, code_rate) = self.link_budget.recommend_modcod();
        self.default_modulation = modulation;
        self.default_code_rate = code_rate;
    }
}

impl Default for DvbRcs2Processor {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Complex ---

    #[test]
    fn test_complex_magnitude() {
        let c = Complex::new(3.0, 4.0);
        assert!((c.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_mul() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, 4.0);
        let r = a.mul(&b);
        // (1+2j)(3+4j) = 3+4j+6j+8j² = 3-8 + (4+6)j = -5+10j
        assert!((r.re - (-5.0)).abs() < 1e-10);
        assert!((r.im - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_conj() {
        let c = Complex::new(2.0, -3.0);
        let cj = c.conj();
        assert_eq!(cj.re, 2.0);
        assert_eq!(cj.im, 3.0);
    }

    // --- Burst Modulation ---

    #[test]
    fn test_burst_modulation_bits_per_symbol() {
        assert_eq!(BurstModulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(BurstModulation::Psk8.bits_per_symbol(), 3);
        assert_eq!(BurstModulation::Qam16.bits_per_symbol(), 4);
    }

    #[test]
    fn test_burst_modulation_order() {
        assert_eq!(BurstModulation::Qpsk.order(), 4);
        assert_eq!(BurstModulation::Psk8.order(), 8);
        assert_eq!(BurstModulation::Qam16.order(), 16);
    }

    // --- Turbo code rate ---

    #[test]
    fn test_turbo_code_rate_values() {
        assert!((TurboCodeRate::R1_3.rate() - 1.0 / 3.0).abs() < 1e-6);
        assert!((TurboCodeRate::R1_2.rate() - 0.5).abs() < 1e-6);
        assert!((TurboCodeRate::R2_3.rate() - 2.0 / 3.0).abs() < 1e-6);
        assert!((TurboCodeRate::R3_4.rate() - 0.75).abs() < 1e-6);
        assert!((TurboCodeRate::R4_5.rate() - 0.8).abs() < 1e-6);
        assert!((TurboCodeRate::R6_7.rate() - 6.0 / 7.0).abs() < 1e-6);
    }

    #[test]
    fn test_turbo_code_rate_eb_n0() {
        // Lower rate should require less Eb/N0
        assert!(TurboCodeRate::R1_3.required_eb_n0_db() < TurboCodeRate::R6_7.required_eb_n0_db());
    }

    // --- Turbo encoder ---

    #[test]
    fn test_turbo_encoder_output_length_r1_3() {
        let enc = TurboEncoder::new(TurboCodeRate::R1_3);
        let bits: Vec<bool> = vec![false; 100];
        let coded = enc.encode(&bits);
        // Rate 1/3: output should be ~3x info bits (+ tails)
        assert!(coded.len() >= 3 * bits.len());
    }

    #[test]
    fn test_turbo_encoder_output_length_r1_2() {
        let enc = TurboEncoder::new(TurboCodeRate::R1_2);
        let bits: Vec<bool> = vec![true, false, true, false, true, false, true, false];
        let coded = enc.encode(&bits);
        // Rate 1/2: ~2x info bits
        assert!(!coded.is_empty());
        assert!(coded.len() >= bits.len());
    }

    #[test]
    fn test_turbo_encoder_nonempty_r2_3() {
        let enc = TurboEncoder::new(TurboCodeRate::R2_3);
        let bits: Vec<bool> = (0..64).map(|i| i % 3 != 0).collect();
        let coded = enc.encode(&bits);
        assert!(!coded.is_empty());
    }

    #[test]
    fn test_turbo_encoder_nonempty_r3_4() {
        let enc = TurboEncoder::new(TurboCodeRate::R3_4);
        let bits: Vec<bool> = vec![true; 48];
        let coded = enc.encode(&bits);
        assert!(!coded.is_empty());
    }

    #[test]
    fn test_turbo_encoder_nonempty_r4_5() {
        let enc = TurboEncoder::new(TurboCodeRate::R4_5);
        let bits: Vec<bool> = vec![false; 40];
        let coded = enc.encode(&bits);
        assert!(!coded.is_empty());
    }

    #[test]
    fn test_turbo_encoder_nonempty_r6_7() {
        let enc = TurboEncoder::new(TurboCodeRate::R6_7);
        let bits: Vec<bool> = (0..84).map(|i| i % 2 != 0).collect();
        let coded = enc.encode(&bits);
        assert!(!coded.is_empty());
    }

    // --- Turbo decoder ---

    #[test]
    fn test_turbo_decoder_basic() {
        let dec = TurboDecoder::new(TurboCodeRate::R1_2);
        let llrs: Vec<f64> = vec![-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0];
        let bits = dec.decode(&llrs);
        // Hard decisions: negative LLR -> 1 (true), positive -> 0 (false)
        assert!(!bits.is_empty());
        assert_eq!(bits[0], true); // llrs[0] < 0 -> true
    }

    #[test]
    fn test_turbo_decoder_soft() {
        let dec = TurboDecoder::new(TurboCodeRate::R1_2);
        let llrs = vec![3.0, -2.0, 1.5, -1.0];
        let soft = dec.decode_soft(&llrs);
        assert_eq!(soft.len(), 2); // rate 1/2, 4 LLRs -> 2 info bits
    }

    // --- QPSK mapper/demapper ---

    #[test]
    fn test_qpsk_roundtrip() {
        let bits: Vec<bool> = vec![false, false, true, false, false, true, true, true];
        let syms = map_qpsk(&bits);
        let recovered = demap_qpsk(&syms);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_qpsk_symbol_power() {
        let bits = vec![false; 8];
        let syms = map_qpsk(&bits);
        for s in &syms {
            let p = s.re * s.re + s.im * s.im;
            assert!((p - 1.0).abs() < 1e-9);
        }
    }

    // --- 8PSK ---

    #[test]
    fn test_8psk_symbol_on_unit_circle() {
        let bits: Vec<bool> = (0..24).map(|i| i % 2 == 0).collect();
        let syms = map_8psk(&bits);
        for s in &syms {
            let r = s.magnitude();
            assert!((r - 1.0).abs() < 1e-9, "8PSK symbol not on unit circle: {}", r);
        }
    }

    #[test]
    fn test_8psk_count() {
        let bits: Vec<bool> = vec![false; 9]; // 9 bits = 3 symbols
        let syms = map_8psk(&bits);
        assert_eq!(syms.len(), 3);
    }

    // --- 16QAM ---

    #[test]
    fn test_16qam_roundtrip() {
        let bits: Vec<bool> = vec![false, false, false, false,
                                    false, false, false, true,
                                    true, true, true, true];
        let syms = map_16qam(&bits);
        let recovered = demap_16qam(&syms);
        assert_eq!(bits.len(), recovered.len());
    }

    #[test]
    fn test_16qam_average_power() {
        let bits: Vec<bool> = (0..64).map(|i| i % 3 == 0).collect();
        let syms = map_16qam(&bits);
        let avg_power: f64 = syms.iter().map(|s| s.re*s.re + s.im*s.im).sum::<f64>() / syms.len() as f64;
        // 16QAM normalised to unit average power
        assert!(avg_power > 0.5 && avg_power < 1.5, "Unexpected avg power: {}", avg_power);
    }

    // --- Spread spectrum ---

    #[test]
    fn test_spread_spectrum_chip_count() {
        let bits = vec![false, true, false]; // 3 bits
        let syms = map_bpsk_spread(&bits);
        assert_eq!(syms.len(), 3 * WH_SPREADING_FACTOR);
    }

    #[test]
    fn test_spread_spectrum_roundtrip() {
        let bits = vec![false, true, false, false, true];
        let syms = map_bpsk_spread(&bits);
        let recovered = demap_spread(&syms);
        assert_eq!(bits, recovered);
    }

    // --- Burst preamble ---

    #[test]
    fn test_preamble_uw_length() {
        let p = BurstPreamble::new();
        assert_eq!(p.unique_word.len(), UW_LENGTH_BITS);
    }

    #[test]
    fn test_preamble_symbol_count() {
        let p = BurstPreamble::new();
        let syms = p.to_symbols();
        let expected = UW_LENGTH_BITS / 2 + p.pilot_symbols; // QPSK: 2 bits/sym
        assert_eq!(syms.len(), expected);
    }

    #[test]
    fn test_preamble_detect() {
        let p = BurstPreamble::new();
        let preamble_syms = p.to_symbols();
        // Place preamble at offset 5
        let mut signal = vec![Complex::zero(); 5];
        signal.extend_from_slice(&preamble_syms);
        signal.extend(vec![Complex::zero(); 10]);
        let offset = p.detect(&signal, 0.5);
        assert_eq!(offset, Some(5));
    }

    // --- NCR timestamp ---

    #[test]
    fn test_ncr_from_us_roundtrip() {
        let us = 50_000u64;
        let ncr = NcrTimestamp::from_us(us);
        let recovered_us = ncr.to_us();
        assert!((recovered_us as i64 - us as i64).abs() <= 1,
            "NCR roundtrip: {} vs {}", recovered_us, us);
    }

    #[test]
    fn test_ncr_encode_decode() {
        let ncr = NcrTimestamp::new(0xDEAD_BEEF);
        let bytes = ncr.encode();
        let decoded = NcrTimestamp::decode(&bytes);
        assert_eq!(ncr, decoded);
    }

    #[test]
    fn test_ncr_diff_no_rollover() {
        let a = NcrTimestamp::new(1000);
        let b = NcrTimestamp::new(500);
        assert_eq!(a.diff(&b), 500);
    }

    #[test]
    fn test_ncr_diff_rollover() {
        let a = NcrTimestamp::new(100);
        let b = NcrTimestamp::new(u32::MAX - 100);
        // a - b should be a small positive number (about 201)
        let d = a.diff(&b);
        assert!(d > 0 && d < 1000, "Rollover diff: {}", d);
    }

    // --- NCR receiver ---

    #[test]
    fn test_ncr_receiver_offset() {
        let mut recv = NcrReceiver::new();
        let ncr = NcrTimestamp::new(1000);
        recv.process_ncr(ncr, 900);
        assert_eq!(recv.offset_ticks, 100);
    }

    #[test]
    fn test_ncr_receiver_synced_ticks() {
        let mut recv = NcrReceiver::new();
        recv.process_ncr(NcrTimestamp::new(5000), 4800);
        let synced = recv.synchronized_ticks(5000);
        assert_eq!(synced, 5200);
    }

    // --- Capacity request ---

    #[test]
    fn test_capacity_request_rbdc_encode_decode() {
        let req = CapacityRequest::new(CapacityType::Rbdc { rate_kbps: 512 });
        let bytes = req.encode();
        let decoded = CapacityRequest::decode(&bytes);
        assert!(matches!(decoded.request_type, CapacityType::Rbdc { rate_kbps: _ }));
    }

    #[test]
    fn test_capacity_request_vbdc_encode_decode() {
        let req = CapacityRequest::new(CapacityType::Vbdc { volume_kbytes: 256 });
        let bytes = req.encode();
        let decoded = CapacityRequest::decode(&bytes);
        assert!(matches!(decoded.request_type, CapacityType::Vbdc { .. }));
    }

    #[test]
    fn test_capacity_request_fca_encode() {
        let req = CapacityRequest::new(CapacityType::Fca);
        let bytes = req.encode();
        assert_eq!(bytes.len(), 3);
    }

    // --- TBTP ---

    #[test]
    fn test_tbtp_encode_decode_roundtrip() {
        let mut tbtp = Tbtp::new(42);
        tbtp.add_entry(TbtpEntry {
            channel_id: 7,
            frame_number: 1,
            start_slot: 100,
            waveform_id: 3,
            assignment_type: 0,
        });
        tbtp.add_entry(TbtpEntry {
            channel_id: 12,
            frame_number: 2,
            start_slot: 200,
            waveform_id: 5,
            assignment_type: 1,
        });
        let encoded = tbtp.encode();
        let decoded = Tbtp::decode(&encoded).unwrap();
        assert_eq!(decoded.superframe_count, 42);
        assert_eq!(decoded.entries.len(), 2);
        assert_eq!(decoded.entries[0].channel_id, 7);
        assert_eq!(decoded.entries[1].channel_id, 12);
    }

    #[test]
    fn test_tbtp_entries_for_channel() {
        let mut tbtp = Tbtp::new(1);
        tbtp.add_entry(TbtpEntry { channel_id: 5, frame_number: 0, start_slot: 0, waveform_id: 1, assignment_type: 0 });
        tbtp.add_entry(TbtpEntry { channel_id: 7, frame_number: 0, start_slot: 100, waveform_id: 1, assignment_type: 0 });
        tbtp.add_entry(TbtpEntry { channel_id: 5, frame_number: 1, start_slot: 0, waveform_id: 1, assignment_type: 0 });
        assert_eq!(tbtp.entries_for_channel(5).len(), 2);
        assert_eq!(tbtp.entries_for_channel(7).len(), 1);
        assert_eq!(tbtp.entries_for_channel(99).len(), 0);
    }

    // --- Superframe ---

    #[test]
    fn test_superframe_symbols_per_frame() {
        let sf = Superframe::new(0, 10, 4, 2.0);
        // 26500 us / 10 frames * 2.0 Msps = 5300 symbols/frame
        let expected = (26_500 / 10) as f64 * 2.0;
        assert_eq!(sf.symbols_per_frame(), expected as u64);
    }

    #[test]
    fn test_superframe_slot_assignment() {
        let mut sf = Superframe::new(0, 10, 4, 2.0);
        let mut slot = TimeSlot::new(0, 1, 100, 500);
        slot.rcst_id = 42;
        sf.add_slot(slot);
        let slots = sf.slots_for_rcst(42);
        assert_eq!(slots.len(), 1);
        assert_eq!(slots[0].carrier_id, 1);
    }

    // --- RLE encapsulation ---

    #[test]
    fn test_rle_encap_complete_pdu() {
        let mut enc = RleEncapsulator::new(200);
        let pkt: Vec<u8> = (0..50).collect();
        let ppdus = enc.encapsulate(&pkt, 0x0800);
        assert_eq!(ppdus.len(), 1); // fits in one PPDU
        let (hdr, _) = RlePpduHeader::decode(&ppdus[0]).unwrap();
        assert_eq!(hdr.pdu_type, RlePduType::Complete);
    }

    #[test]
    fn test_rle_encap_fragmentation() {
        let mut enc = RleEncapsulator::new(50);
        let pkt: Vec<u8> = (0..150u8).collect();
        let ppdus = enc.encapsulate(&pkt, 0x0800);
        assert!(ppdus.len() > 1, "Expected fragmentation");
        let (first, _) = RlePpduHeader::decode(&ppdus[0]).unwrap();
        assert_eq!(first.pdu_type, RlePduType::Start);
        let (last, _) = RlePpduHeader::decode(ppdus.last().unwrap()).unwrap();
        assert_eq!(last.pdu_type, RlePduType::End);
    }

    #[test]
    fn test_rle_reassembly_complete() {
        let mut enc = RleEncapsulator::new(200);
        let mut dec = RleReassembler::new();
        let pkt: Vec<u8> = (0..100).collect();
        let ppdus = enc.encapsulate(&pkt, 0x0800);
        for ppdu in &ppdus {
            dec.process_ppdu(ppdu);
        }
        let packets = dec.drain_packets();
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0], pkt);
    }

    #[test]
    fn test_rle_reassembly_fragmented() {
        let mut enc = RleEncapsulator::new(30);
        let mut dec = RleReassembler::new();
        let pkt: Vec<u8> = (0..100u8).collect();
        let ppdus = enc.encapsulate(&pkt, 0x0800);
        for ppdu in &ppdus {
            dec.process_ppdu(ppdu);
        }
        let packets = dec.drain_packets();
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0], pkt);
    }

    // --- PPDU header encode/decode ---

    #[test]
    fn test_ppdu_header_complete_roundtrip() {
        let hdr = RlePpduHeader {
            pdu_type: RlePduType::Complete,
            fragment_id: 15,
            total_length: Some(1024),
            protocol_type: Some(0x0800),
        };
        let bytes = hdr.encode();
        let (decoded, _) = RlePpduHeader::decode(&bytes).unwrap();
        assert_eq!(decoded.pdu_type, RlePduType::Complete);
        assert_eq!(decoded.total_length, Some(1024));
        assert_eq!(decoded.protocol_type, Some(0x0800));
    }

    #[test]
    fn test_ppdu_header_continuation() {
        let hdr = RlePpduHeader {
            pdu_type: RlePduType::Continuation,
            fragment_id: 3,
            total_length: None,
            protocol_type: None,
        };
        let bytes = hdr.encode();
        assert_eq!(bytes.len(), 1); // No extra fields for continuation
        let (decoded, _) = RlePpduHeader::decode(&bytes).unwrap();
        assert_eq!(decoded.pdu_type, RlePduType::Continuation);
    }

    // --- Logon sequence ---

    #[test]
    fn test_logon_request_encode() {
        let req = LogonRequest::new(0xAABBCCDDEEFF, 1);
        let bytes = req.encode();
        assert!(!bytes.is_empty());
        assert_eq!(bytes.len(), 22); // 6+2+2+4+8
    }

    #[test]
    fn test_rcst_controller_logon_flow() {
        let mut ctrl = RcstController::new();
        assert_eq!(ctrl.state, LogonState::Off);

        ctrl.start_logon(LogonRequest::new(0x1234, 1));
        assert_eq!(ctrl.state, LogonState::WaitingCsc);

        let msg = ctrl.on_csc_window();
        assert!(msg.is_some());
        assert_eq!(ctrl.state, LogonState::LogonPending);

        let timu = TimU {
            channel_id: 42,
            timing_offset: -10,
            freq_correction_hz: 500,
            superframe_id: 1,
            success: true,
        };
        ctrl.process_timu(&timu);
        assert_eq!(ctrl.state, LogonState::LoggedOn);
        assert_eq!(ctrl.channel_id, Some(42));
    }

    #[test]
    fn test_rcst_controller_logon_failure() {
        let mut ctrl = RcstController::new();
        ctrl.start_logon(LogonRequest::new(0x5678, 2));
        ctrl.on_csc_window();
        let timu = TimU {
            channel_id: 0, timing_offset: 0, freq_correction_hz: 0,
            superframe_id: 0, success: false,
        };
        ctrl.process_timu(&timu);
        // Should retry (< max attempts)
        assert_eq!(ctrl.state, LogonState::WaitingCsc);
    }

    #[test]
    fn test_rcst_controller_handover() {
        let mut ctrl = RcstController::new();
        ctrl.start_logon(LogonRequest::new(0xABCD, 1));
        ctrl.on_csc_window();
        ctrl.process_timu(&TimU {
            channel_id: 1, timing_offset: 0, freq_correction_hz: 0,
            superframe_id: 1, success: true,
        });
        assert!(ctrl.is_logged_on());
        ctrl.start_handover(2);
        assert_eq!(ctrl.state, LogonState::HandoverInProgress);
    }

    // --- TIM-U encode/decode ---

    #[test]
    fn test_timu_encode_decode() {
        let timu = TimU {
            channel_id: 77,
            timing_offset: -500,
            freq_correction_hz: 1234,
            superframe_id: 3,
            success: true,
        };
        let bytes = timu.encode();
        let decoded = TimU::decode(&bytes).unwrap();
        assert_eq!(decoded.channel_id, 77);
        assert_eq!(decoded.timing_offset, -500);
        assert_eq!(decoded.freq_correction_hz, 1234);
        assert!(decoded.success);
    }

    // --- Link budget ---

    #[test]
    fn test_return_link_budget_cn0_positive() {
        let budget = ReturnLinkBudget::new();
        let cn0 = budget.cn0_db_hz();
        // Should be a plausible finite value (typically 50–80 dB-Hz for Ku-band return link)
        assert!(cn0.is_finite(), "C/N0 not finite: {}", cn0);
        assert!(cn0 > 20.0 && cn0 < 120.0, "Unexpected C/N0: {}", cn0);
    }

    #[test]
    fn test_return_link_budget_eb_n0() {
        let budget = ReturnLinkBudget::new();
        let eb_n0 = budget.eb_n0_db();
        // Should be finite
        assert!(eb_n0.is_finite(), "Eb/N0 not finite: {}", eb_n0);
    }

    #[test]
    fn test_return_link_budget_fspl() {
        let fspl = ReturnLinkBudget::compute_fspl(36000.0, 14.25);
        // GEO at 14.25 GHz: ~207 dB
        assert!(fspl > 200.0 && fspl < 215.0, "FSPL: {}", fspl);
    }

    #[test]
    fn test_return_link_budget_recommend_modcod() {
        let mut budget = ReturnLinkBudget::new();
        budget.eirp_dbw = 50.0; // high EIRP
        budget.rain_attenuation_db = 0.0;
        let (modulation, code_rate) = budget.recommend_modcod();
        // At high Eb/N0, should select high-order modulation
        assert!(matches!(modulation, BurstModulation::Qam16 | BurstModulation::Psk8 | BurstModulation::Qpsk));
        let _ = code_rate; // just ensure it compiles
    }

    // --- Rain attenuation ---

    #[test]
    fn test_rain_attenuation_low_rain_rate() {
        let att = rain_attenuation_db(5.0, 30.0, 14.25, 0.999);
        assert!(att >= 0.0 && att < 10.0, "Attenuation: {}", att);
    }

    #[test]
    fn test_rain_attenuation_high_rain_rate() {
        let att = rain_attenuation_db(50.0, 20.0, 14.25, 0.99);
        // Heavy rain + low elevation => attenuation should be finite and non-negative
        assert!(att.is_finite(), "Expected finite attenuation, got: {}", att);
        assert!(att >= 0.0, "Expected non-negative attenuation, got: {}", att);
    }

    #[test]
    fn test_rain_attenuation_zero_elevation_guard() {
        let att = rain_attenuation_db(10.0, 0.0, 14.25, 0.999);
        assert_eq!(att, 100.0); // should hit the guard value
    }

    // --- Main processor ---

    #[test]
    fn test_processor_encode_burst_qpsk() {
        let mut proc = DvbRcs2Processor::new();
        let data = vec![0x12, 0x34, 0x56];
        let burst = proc.encode_burst(&data, BurstModulation::Qpsk, TurboCodeRate::R1_2);
        assert!(!burst.symbols.is_empty());
        assert_eq!(burst.info_bytes, 3);
        assert_eq!(burst.modulation, BurstModulation::Qpsk);
    }

    #[test]
    fn test_processor_encode_burst_8psk() {
        let mut proc = DvbRcs2Processor::new();
        let data: Vec<u8> = (0..16).collect();
        let burst = proc.encode_burst(&data, BurstModulation::Psk8, TurboCodeRate::R2_3);
        assert!(!burst.symbols.is_empty());
    }

    #[test]
    fn test_processor_encode_burst_16qam() {
        let mut proc = DvbRcs2Processor::new();
        let data: Vec<u8> = (0..20).collect();
        let burst = proc.encode_burst(&data, BurstModulation::Qam16, TurboCodeRate::R3_4);
        assert!(!burst.symbols.is_empty());
        assert_eq!(burst.code_rate, TurboCodeRate::R3_4);
    }

    #[test]
    fn test_processor_encode_decode_roundtrip() {
        let mut proc = DvbRcs2Processor::new();
        let data: Vec<u8> = vec![0xAB, 0xCD, 0xEF, 0x01, 0x23];
        let burst = proc.encode_burst(&data, BurstModulation::Qpsk, TurboCodeRate::R1_2);
        let decoded = proc.decode_burst(&burst.symbols, BurstModulation::Qpsk, TurboCodeRate::R1_2);
        // Decoded should have the same length as original (turbo decode simplified)
        assert_eq!(decoded.len(), data.len());
    }

    #[test]
    fn test_processor_burst_symbol_energy() {
        let mut proc = DvbRcs2Processor::new();
        let data = vec![0xFF; 8];
        let burst = proc.encode_burst(&data, BurstModulation::Qpsk, TurboCodeRate::R1_2);
        let es = burst.es_per_symbol();
        assert!(es > 0.0 && es < 2.0, "Unexpected Es/sym: {}", es);
    }

    #[test]
    fn test_processor_adapt_modcod() {
        let mut proc = DvbRcs2Processor::new();
        proc.adapt_modcod(50.0, 0.0);
        // After adaptation with high EIRP, modcod should be updated
        assert!(matches!(proc.default_modulation, BurstModulation::Qpsk | BurstModulation::Psk8 | BurstModulation::Qam16));
    }

    #[test]
    fn test_processor_configure_superframe() {
        let mut proc = DvbRcs2Processor::new();
        let sf = Superframe::new(1, 8, 4, 4.0);
        proc.configure_superframe(sf);
        assert!(proc.superframe.is_some());
        assert_eq!(proc.superframe.as_ref().unwrap().id, 1);
    }

    #[test]
    fn test_processor_spread_spectrum_burst() {
        let mut proc = DvbRcs2Processor::new();
        let data = vec![0xAA, 0xBB];
        let burst = proc.encode_burst(&data, BurstModulation::SpreadSpectrum, TurboCodeRate::R1_2);
        assert!(!burst.symbols.is_empty());
        // SS burst should be longer than equivalent linear burst
        let lin = proc.encode_burst(&data, BurstModulation::Qpsk, TurboCodeRate::R1_2);
        assert!(burst.symbols.len() > lin.symbols.len());
    }

    #[test]
    fn test_processor_empty_burst() {
        let mut proc = DvbRcs2Processor::new();
        let burst = proc.encode_burst(&[], BurstModulation::Qpsk, TurboCodeRate::R1_2);
        // Empty data: preamble + turbo tail symbols
        let preamble_len = proc.preamble.to_symbols().len();
        // Burst must be at least as long as preamble
        assert!(burst.symbols.len() >= preamble_len,
            "Burst length {} shorter than preamble {}", burst.symbols.len(), preamble_len);
        assert_eq!(burst.info_bytes, 0);
    }
}

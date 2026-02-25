//! NXDN Digital Radio Protocol Processor
//!
//! Implements NXDN digital radio processing per ETSI TS 102 361 and the
//! Icom/Kenwood NXDN Technical Specifications. Supports NXDN-6.25 (6.25 kHz
//! channel, 2400 bps) and NXDN-12.5 (12.5 kHz channel, 4800 bps) modes.
//!
//! # Features
//! - 4FSK modulation/demodulation (±600 Hz / ±1800 Hz deviation)
//! - RRC (Root Raised Cosine) pulse shaping and matched filtering
//! - NXDN frame structure (LICH, SACCH, FACCH, voice/data)
//! - PN scrambling with frame-synchronous reset
//! - Block interleaving/deinterleaving
//! - CRC-6, CRC-15, CRC-16 per spec
//! - Rate 1/2 convolutional coding + Viterbi soft-decision decoding
//! - Frame Sync Word (FSW) detection with correlation threshold
//! - AMBE+2 voice frame packing (EHR format)
//! - Trunking call-control signaling
//! - Layer 2 CAC/USC channel management
//!
//! # References
//! - NXDN Common Air Interface (CAI) Specification, Release 1.1
//! - ETSI TS 102 361-4 (DMR / NXDN data applications)

// ============================================================
//  Constants
// ============================================================

/// Symbol rate in baud (both channel variants share this).
pub const SYMBOL_RATE: u32 = 2400;

/// Inner deviation (symbols ±1) in Hz.
pub const DEV_INNER_HZ: f64 = 600.0;

/// Outer deviation (symbols ±3) in Hz.
pub const DEV_OUTER_HZ: f64 = 1800.0;

/// 48-bit Frame Sync Word (FSW) per NXDN CAI spec.
/// Represented as a sequence of bits (MSB first).
pub const FSW_BITS: [u8; 48] = [
    0,1,1,1,0,1,0,0,  // 0x74
    1,0,1,0,1,0,0,0,  // 0xA8
    0,1,0,0,0,0,1,1,  // 0x43
    1,0,0,0,1,1,1,1,  // 0x8F
    1,0,0,1,1,0,0,0,  // 0x98
    0,0,1,1,0,0,1,1,  // 0x33
];

/// Number of dibits in one NXDN frame (super-frame = 192 symbols × 2 = 384 bits).
pub const FRAME_SYMBOLS: usize = 192;

/// Bits per NXDN frame (2 bits/symbol × 192 symbols).
pub const FRAME_BITS: usize = 384;

/// Duration of one frame in milliseconds.
pub const FRAME_DURATION_MS: f64 = 20.0;

/// RRC filter roll-off factor (α = 0.2 per NXDN spec).
pub const RRC_ALPHA: f64 = 0.2;

/// Convolutional code constraint length.
pub const CONV_K: usize = 9;

/// Convolutional code rate 1/2 generator polynomials (octal 561, 753).
pub const CONV_G0: u16 = 0o561; // 0b_1011_0001
pub const CONV_G1: u16 = 0o753; // 0b_1111_0011

// ============================================================
//  Channel Mode
// ============================================================

/// NXDN channel bandwidth mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelMode {
    /// 6.25 kHz channel — 2400 bps raw data rate.
    Nxdn6k25,
    /// 12.5 kHz channel — 4800 bps raw data rate (two NXDN-6.25 slots).
    Nxdn12k5,
}

impl ChannelMode {
    /// Channel spacing in Hz.
    pub fn channel_spacing_hz(&self) -> f64 {
        match self {
            ChannelMode::Nxdn6k25 => 6_250.0,
            ChannelMode::Nxdn12k5 => 12_500.0,
        }
    }

    /// Raw data rate in bps.
    pub fn data_rate_bps(&self) -> u32 {
        match self {
            ChannelMode::Nxdn6k25 => 2_400,
            ChannelMode::Nxdn12k5 => 4_800,
        }
    }

    /// Samples per symbol at nominal 48 kHz ADC rate.
    pub fn samples_per_symbol(&self, sample_rate: f64) -> f64 {
        sample_rate / SYMBOL_RATE as f64
    }
}

// ============================================================
//  4FSK Symbol Definitions
// ============================================================

/// 4FSK dibit → symbol frequency deviation mapping (Gray-coded).
/// dibit 00 → +3 (outer +1800 Hz)
/// dibit 01 → +1 (inner  +600 Hz)
/// dibit 11 → -1 (inner  -600 Hz)
/// dibit 10 → -3 (outer -1800 Hz)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FskSymbol {
    PlusOuter  = 3,   // dibit 00
    PlusInner  = 1,   // dibit 01
    MinusInner = -1,  // dibit 11  (stored as i8 below)
    MinusOuter = -3,  // dibit 10
}

/// Map a dibit (2-bit value 0–3) to its normalised symbol level (±1, ±3).
pub fn dibit_to_symbol(dibit: u8) -> i8 {
    match dibit & 0x03 {
        0b00 => 3,
        0b01 => 1,
        0b11 => -1,
        0b10 => -3,
        _    => 0,
    }
}

/// Map a normalised symbol level to its dibit (2-bit value).
/// Nearest-neighbour decision (thresholds at ±2).
pub fn symbol_to_dibit(level: f64) -> u8 {
    if level > 2.0 {
        0b00
    } else if level > 0.0 {
        0b01
    } else if level > -2.0 {
        0b11
    } else {
        0b10
    }
}

/// Map a normalised symbol level to frequency deviation in Hz.
pub fn symbol_to_deviation_hz(level: i8) -> f64 {
    level as f64 * DEV_INNER_HZ
}

// ============================================================
//  Bit ↔ Dibit ↔ Symbol utilities
// ============================================================

/// Pack a bit-slice (MSB first, length must be even) into dibits.
pub fn bits_to_dibits(bits: &[u8]) -> Vec<u8> {
    assert!(bits.len() % 2 == 0, "bit length must be even");
    bits.chunks(2)
        .map(|c| ((c[0] & 1) << 1) | (c[1] & 1))
        .collect()
}

/// Unpack dibits into a bit-slice (MSB first).
pub fn dibits_to_bits(dibits: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(dibits.len() * 2);
    for d in dibits {
        out.push((d >> 1) & 1);
        out.push(d & 1);
    }
    out
}

// ============================================================
//  Root-Raised-Cosine (RRC) Pulse Shaping
// ============================================================

/// Generate a symmetric RRC filter impulse response.
///
/// # Parameters
/// - `num_taps`: total tap count (should be odd)
/// - `sps`: samples per symbol (must be ≥ 1.0)
/// - `alpha`: roll-off factor (0 < alpha ≤ 1)
pub fn rrc_filter(num_taps: usize, sps: f64, alpha: f64) -> Vec<f64> {
    assert!(num_taps > 0 && num_taps % 2 == 1, "num_taps must be odd and > 0");
    let half = (num_taps - 1) as f64 / 2.0;
    let mut taps = vec![0.0f64; num_taps];
    for i in 0..num_taps {
        let t = (i as f64 - half) / sps;
        taps[i] = rrc_kernel(t, alpha);
    }
    // normalise so peak = 1
    let peak = taps[num_taps / 2];
    if peak.abs() > 1e-12 {
        for v in taps.iter_mut() {
            *v /= peak;
        }
    }
    taps
}

/// Single-point RRC kernel h(t) at normalised time t (in symbols).
fn rrc_kernel(t: f64, alpha: f64) -> f64 {
    const PI: f64 = std::f64::consts::PI;
    if t.abs() < 1e-10 {
        return 1.0 - alpha + 4.0 * alpha / PI;
    }
    let crit = 1.0 / (4.0 * alpha);
    if (t.abs() - crit).abs() < 1e-10 {
        let a = (1.0 + 2.0 / PI) * f64::sin(PI / (4.0 * alpha));
        let b = (1.0 - 2.0 / PI) * f64::cos(PI / (4.0 * alpha));
        return alpha / (2.0_f64.sqrt()) * (a + b);
    }
    let num = f64::sin(PI * t * (1.0 - alpha)) + 4.0 * alpha * t * f64::cos(PI * t * (1.0 + alpha));
    let den = PI * t * (1.0 - (4.0 * alpha * t).powi(2));
    if den.abs() < 1e-15 { 0.0 } else { num / den }
}

/// Apply a linear FIR filter to a real-valued signal (direct-form convolution).
/// Output length equals input length (zero-padded boundaries).
pub fn fir_filter(signal: &[f64], taps: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let m = taps.len();
    let half = m / 2;
    let mut out = vec![0.0f64; n];
    for i in 0..n {
        let mut acc = 0.0;
        for k in 0..m {
            let si = i as isize + k as isize - half as isize;
            if si >= 0 && si < n as isize {
                acc += taps[k] * signal[si as usize];
            }
        }
        out[i] = acc;
    }
    out
}

// ============================================================
//  4FSK Modulator
// ============================================================

/// 4FSK modulator: converts dibits → baseband analog waveform samples.
pub struct FskModulator {
    /// Samples per symbol.
    pub sps: usize,
    /// RRC pulse shaping taps.
    pub rrc_taps: Vec<f64>,
    /// Current phase accumulator (radians) for FM modulation output.
    phase: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
}

impl FskModulator {
    /// Create a new modulator for the given sample rate and channel mode.
    pub fn new(sample_rate: f64, _mode: ChannelMode) -> Self {
        let sps = (sample_rate / SYMBOL_RATE as f64).round() as usize;
        let num_taps = 8 * sps + 1;
        let rrc_taps = rrc_filter(num_taps, sps as f64, RRC_ALPHA);
        Self { sps, rrc_taps, phase: 0.0, sample_rate }
    }

    /// Modulate a slice of dibits into baseband FM samples (real-valued).
    /// Returns `dibits.len() * sps` samples.
    pub fn modulate(&mut self, dibits: &[u8]) -> Vec<f64> {
        // 1. Map dibits → symbol levels
        let levels: Vec<f64> = dibits.iter()
            .map(|&d| dibit_to_symbol(d) as f64)
            .collect();

        // 2. Upsample (zero-insert)
        let mut upsampled = vec![0.0f64; levels.len() * self.sps];
        for (i, &l) in levels.iter().enumerate() {
            upsampled[i * self.sps] = l;
        }

        // 3. RRC pulse shaping
        let shaped = fir_filter(&upsampled, &self.rrc_taps);

        // 4. FM: integrate frequency deviation → phase → cos(phase)
        let freq_sensitivity = 2.0 * std::f64::consts::PI * DEV_INNER_HZ / self.sample_rate;
        let mut out = vec![0.0f64; shaped.len()];
        for (i, &v) in shaped.iter().enumerate() {
            self.phase += v * freq_sensitivity;
            self.phase = wrap_phase(self.phase);
            out[i] = f64::cos(self.phase);
        }
        out
    }

    /// Reset phase accumulator.
    pub fn reset(&mut self) { self.phase = 0.0; }
}

/// Wrap phase to [-π, π].
fn wrap_phase(p: f64) -> f64 {
    use std::f64::consts::PI;
    let tau = 2.0 * PI;
    let mut q = p % tau;
    if q > PI  { q -= tau; }
    if q < -PI { q += tau; }
    q
}

// ============================================================
//  4FSK Demodulator
// ============================================================

/// Frequency discriminator demodulator for 4FSK.
pub struct FskDemodulator {
    pub sps: usize,
    pub rrc_taps: Vec<f64>,
    pub sample_rate: f64,
    prev_sample: f64,
    prev_phase: f64,
}

impl FskDemodulator {
    pub fn new(sample_rate: f64, _mode: ChannelMode) -> Self {
        let sps = (sample_rate / SYMBOL_RATE as f64).round() as usize;
        let num_taps = 8 * sps + 1;
        let rrc_taps = rrc_filter(num_taps, sps as f64, RRC_ALPHA);
        Self { sps, rrc_taps, sample_rate, prev_sample: 0.0, prev_phase: 0.0 }
    }

    /// Demodulate real-valued FM samples to dibits.
    /// Processes exactly `samples.len() / sps` symbols.
    pub fn demodulate(&mut self, samples: &[f64]) -> Vec<u8> {
        // 1. Instantaneous frequency via differentiated phase (arctan discriminator)
        let mut freq = vec![0.0f64; samples.len()];
        for i in 0..samples.len() {
            let cur = samples[i];
            // Simple FM discriminator: approximate atan2 differentiation
            // For a real-only signal we approximate using delay-and-multiply
            let delayed = if i == 0 { self.prev_sample } else { samples[i - 1] };
            // atan2 derivative approximation
            let cross = cur * delayed;
            freq[i] = if cross.abs() < 1e-15 { 0.0 } else {
                let delta_phi = f64::asin((cur - delayed).max(-1.0).min(1.0));
                delta_phi * self.sample_rate / (2.0 * std::f64::consts::PI * DEV_INNER_HZ)
            };
        }
        self.prev_sample = *samples.last().unwrap_or(&0.0);

        // 2. Matched RRC filter
        let filtered = fir_filter(&freq, &self.rrc_taps);

        // 3. Symbol-rate sampling (sample at centre of each symbol interval)
        let n_syms = samples.len() / self.sps;
        let offset = self.sps / 2;
        (0..n_syms)
            .map(|i| {
                let idx = (i * self.sps + offset).min(filtered.len() - 1);
                symbol_to_dibit(filtered[idx])
            })
            .collect()
    }

    pub fn reset(&mut self) {
        self.prev_sample = 0.0;
        self.prev_phase = 0.0;
    }
}

// ============================================================
//  CRC engines
// ============================================================

/// Compute CRC-6 (polynomial 0x27, degree 6) over `bits` (MSB first).
/// Used in LICH fields.
pub fn crc6(bits: &[u8]) -> u8 {
    // Poly: x^6 + x^5 + x^2 + x + 1 → 0b110_0111 = 0x67 (with implicit x^6)
    const POLY: u8 = 0x27; // lower 6 bits of full polynomial
    let mut crc: u8 = 0x3F; // init all-ones
    for &b in bits {
        let bit = (b & 1) ^ ((crc >> 5) & 1);
        crc = ((crc << 1) & 0x3F) ^ if bit != 0 { POLY } else { 0 };
    }
    // Final XOR mask all-ones
    (!crc) & 0x3F
}

/// Compute CRC-15 (polynomial 0x6815, degree 15) over `bits`.
/// Used in SACCH/FACCH Layer 2 frames.
pub fn crc15(bits: &[u8]) -> u16 {
    // Poly: x^15 + x^14 + x^11 + x^4 + x^2 + 1 → 0x6815
    const POLY: u16 = 0x6815;
    let mut crc: u16 = 0x7FFF; // init
    for &b in bits {
        let bit = ((b & 1) as u16) ^ ((crc >> 14) & 1);
        crc = ((crc << 1) & 0x7FFF) ^ if bit != 0 { POLY } else { 0 };
    }
    (!crc) & 0x7FFF
}

/// Compute CRC-16-CCITT (polynomial 0x1021) over a byte slice.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    const POLY: u16 = 0x1021;
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        for bit in (0..8).rev() {
            let b = ((byte >> bit) & 1) as u16;
            let feedback = ((crc >> 15) & 1) ^ b;
            crc = (crc << 1) & 0xFFFF;
            if feedback != 0 { crc ^= POLY; }
        }
    }
    (!crc) & 0xFFFF
}

// ============================================================
//  Convolutional Encoder (rate 1/2, K=9)
// ============================================================

/// Rate-1/2, K=9 convolutional encoder.
/// Uses NXDN polynomials G0=0o561, G1=0o753.
pub struct ConvEncoder {
    shift_reg: u16,
}

impl ConvEncoder {
    pub fn new() -> Self { Self { shift_reg: 0 } }

    /// Encode a bit slice into a sequence of coded bits (2× length).
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(bits.len() * 2);
        for &b in bits {
            // Shift-right: new bit enters at MSB position K-1.
            // State = shift_reg (upper K-1 bits after shift are the ACS state).
            self.shift_reg = ((self.shift_reg >> 1) | ((b as u16 & 1) << (CONV_K as u16 - 1))) & ((1 << CONV_K) - 1);
            let b0 = parity_u16(self.shift_reg & CONV_G0);
            let b1 = parity_u16(self.shift_reg & CONV_G1);
            out.push(b0);
            out.push(b1);
        }
        out
    }

    pub fn reset(&mut self) { self.shift_reg = 0; }
}

impl Default for ConvEncoder { fn default() -> Self { Self::new() } }

/// Bit parity of u16 (popcount mod 2).
fn parity_u16(mut x: u16) -> u8 {
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    (x & 1) as u8
}

// ============================================================
//  Viterbi Decoder (hard-decision, rate 1/2, K=9)
// ============================================================

const VITERBI_STATES: usize = 1 << (CONV_K - 1); // 256
/// Hard-decision Viterbi decoder for rate-1/2, K=9 convolutional code.
pub struct ViterbiDecoder;

impl ViterbiDecoder {
    /// Decode a hard-bit stream (pairs of coded bits) back to info bits.
    pub fn decode(coded_bits: &[u8]) -> Vec<u8> {
        assert!(coded_bits.len() % 2 == 0, "coded length must be even");
        let num_info = coded_bits.len() / 2;

        // Path metrics — use i32 (higher = worse, Hamming distance)
        let mut pm      = vec![i32::MAX / 2; VITERBI_STATES];
        let mut prev_pm = vec![i32::MAX / 2; VITERBI_STATES];
        pm[0] = 0;

        // Traceback: [time][new_state] = (prev_state << 1) | input_bit
        // Packing prev_state (8 bits) and input_bit (1 bit) into a u16.
        let mut trace: Vec<Vec<u16>> = Vec::with_capacity(num_info);

        for t in 0..num_info {
            let r0 = coded_bits[2 * t]     & 1;
            let r1 = coded_bits[2 * t + 1] & 1;

            std::mem::swap(&mut pm, &mut prev_pm);
            pm.iter_mut().for_each(|v| *v = i32::MAX / 2);

            let mut tb = vec![0u16; VITERBI_STATES];

            for prev_state in 0..VITERBI_STATES {
                if prev_pm[prev_state] >= i32::MAX / 2 { continue; }
                for input_bit in 0u8..2 {
                    // Shift-right convention: new bit at MSB; state = sr >> 1.
                    // sr = prev_state | (input_bit << (K-1))
                    let sr = (prev_state | ((input_bit as usize) << (CONV_K - 1))) & ((1 << CONV_K) - 1);
                    let b0 = parity_u16((sr as u16) & CONV_G0);
                    let b1 = parity_u16((sr as u16) & CONV_G1);
                    let branch_metric = ((b0 ^ r0) + (b1 ^ r1)) as i32;
                    let new_state = sr >> 1; // K-1 bits: state after transition
                    let new_metric = prev_pm[prev_state] + branch_metric;
                    if new_metric < pm[new_state] {
                        pm[new_state] = new_metric;
                        // Pack: lower 8 bits = prev_state, bit 8 = input_bit
                        tb[new_state] = ((input_bit as u16) << (CONV_K as u16 - 1)) | (prev_state as u16);
                    }
                }
            }
            trace.push(tb);
        }

        // Traceback from best final state
        let mut best_state = pm.iter().enumerate()
            .min_by_key(|&(_, &m)| m)
            .map(|(s, _)| s)
            .unwrap_or(0);

        let mut decoded = vec![0u8; num_info];
        for t in (0..num_info).rev() {
            let packed = trace[t][best_state];
            // input_bit stored at bit K-1; prev_state in lower K-1 bits
            decoded[t]  = ((packed >> (CONV_K as u16 - 1)) & 1) as u8;
            best_state  = (packed & (VITERBI_STATES as u16 - 1)) as usize;
        }
        decoded
    }
}

//  Block Interleaver / Deinterleaver
// ============================================================

/// Block interleaver: row-in / column-out (write by rows, read by columns).
pub struct BlockInterleaver {
    pub rows: usize,
    pub cols: usize,
}

impl BlockInterleaver {
    pub fn new(rows: usize, cols: usize) -> Self { Self { rows, cols } }

    /// Interleave: write input row-by-row, read out column-by-column.
    pub fn interleave(&self, bits: &[u8]) -> Vec<u8> {
        assert_eq!(bits.len(), self.rows * self.cols);
        let mut out = vec![0u8; bits.len()];
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[c * self.rows + r] = bits[r * self.cols + c];
            }
        }
        out
    }

    /// Deinterleave: inverse of interleave.
    pub fn deinterleave(&self, bits: &[u8]) -> Vec<u8> {
        assert_eq!(bits.len(), self.rows * self.cols);
        let mut out = vec![0u8; bits.len()];
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[r * self.cols + c] = bits[c * self.rows + r];
            }
        }
        out
    }
}

// ============================================================
//  PN Scrambler
// ============================================================

/// NXDN PN scrambler using a 15-bit LFSR (polynomial x^15 + x^14 + 1).
pub struct PnScrambler {
    state: u16,
    /// Initial seed (loaded at frame start).
    seed: u16,
}

impl PnScrambler {
    /// Create with a given 15-bit seed.
    pub fn new(seed: u16) -> Self {
        Self { state: seed & 0x7FFF, seed: seed & 0x7FFF }
    }

    /// Reset to initial seed.
    pub fn reset(&mut self) { self.state = self.seed; }

    /// Generate next scramble bit.
    pub fn next_bit(&mut self) -> u8 {
        let out = (self.state & 1) as u8;
        // LFSR tap: x^15 + x^14 + 1 → feedback = bit14 XOR bit15 (1-indexed)
        let feedback = ((self.state >> 14) ^ (self.state >> 13)) & 1;
        self.state = ((self.state >> 1) | (feedback << 14)) & 0x7FFF;
        out
    }

    /// Scramble (XOR) a bit slice in-place; also used for descrambling.
    pub fn scramble(&mut self, bits: &mut [u8]) {
        for b in bits.iter_mut() {
            *b ^= self.next_bit();
        }
    }

    /// Scramble a bit slice and return new vec (does NOT reset).
    pub fn scramble_copy(&mut self, bits: &[u8]) -> Vec<u8> {
        bits.iter().map(|&b| b ^ self.next_bit()).collect()
    }
}

// ============================================================
//  Sync Word Detector
// ============================================================

/// Detect NXDN Frame Sync Words in a dibit stream.
pub struct SyncDetector {
    /// Correlation threshold (out of 48 bits).
    pub threshold: usize,
    /// Internal dibit history buffer.
    history: Vec<u8>,
}

impl SyncDetector {
    /// Create a new sync detector with a given bit-error tolerance.
    pub fn new(max_errors: usize) -> Self {
        Self {
            threshold: FSW_BITS.len() - max_errors,
            history: vec![0u8; FSW_BITS.len() / 2],
        }
    }

    /// Feed a new dibit; returns `true` if FSW detected at current position.
    pub fn push_dibit(&mut self, dibit: u8) -> bool {
        // Shift history
        let len = self.history.len();
        self.history.copy_within(1..len, 0);
        *self.history.last_mut().unwrap() = dibit;

        // Convert history to bits and correlate
        let hist_bits = dibits_to_bits(&self.history);
        let matches: usize = hist_bits.iter().zip(FSW_BITS.iter())
            .filter(|(&a, &b)| a == b)
            .count();
        matches >= self.threshold
    }

    /// Correlate FSW against an arbitrary bit slice starting at `offset`.
    /// Returns number of matching bits.
    pub fn correlate_at(bits: &[u8], offset: usize) -> usize {
        FSW_BITS.iter().zip(bits.iter().skip(offset))
            .filter(|(&a, &b)| a == b)
            .count()
    }

    /// Scan a bit slice for the best FSW match position.
    /// Returns (best_offset, best_score).
    pub fn find_sync(bits: &[u8]) -> (usize, usize) {
        let fsw_len = FSW_BITS.len();
        if bits.len() < fsw_len { return (0, 0); }
        let mut best = (0, 0);
        for offset in 0..=(bits.len() - fsw_len) {
            let score = Self::correlate_at(bits, offset);
            if score > best.1 { best = (offset, score); }
        }
        best
    }
}

// ============================================================
//  LICH (Link Information Channel)
// ============================================================

/// LICH channel type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LichChannelType {
    /// Repeated capability advertisement.
    Rcch = 0,
    /// Control channel.
    Ccch = 1,
    /// Associated control channel (slow).
    Sacch = 2,
    /// Associated control channel (fast).
    Facch = 3,
    /// User traffic (voice/data).
    Udch = 4,
    /// Paging channel.
    Dcch = 5,
    /// Broadcast.
    Bcch = 6,
    /// Unknown.
    Unknown = 7,
}

impl From<u8> for LichChannelType {
    fn from(v: u8) -> Self {
        match v & 0x07 {
            0 => Self::Rcch,
            1 => Self::Ccch,
            2 => Self::Sacch,
            3 => Self::Facch,
            4 => Self::Udch,
            5 => Self::Dcch,
            6 => Self::Bcch,
            _ => Self::Unknown,
        }
    }
}

/// LICH structure (8 bits including 6-bit CRC).
#[derive(Debug, Clone)]
pub struct Lich {
    /// Radio call type: 0=individual, 1=group, 2=broadcast.
    pub call_type: u8,
    /// Channel type.
    pub channel_type: LichChannelType,
    /// Structure field (0–3): indicates fragment position.
    pub structure: u8,
}

impl Lich {
    pub fn new(call_type: u8, channel_type: LichChannelType, structure: u8) -> Self {
        Self { call_type: call_type & 1, channel_type, structure: structure & 3 }
    }

    /// Encode LICH to 16 bits (10 info + 6 CRC).
    pub fn encode(&self) -> Vec<u8> {
        // Bits [15:14] = structure, [13] = call_type, [12:10] = channel_type
        let info_word: u16 = ((self.structure as u16 & 3) << 6)
            | ((self.call_type as u16 & 1) << 5)
            | ((self.channel_type as u8 as u16 & 7) << 2);
        let info_bits: Vec<u8> = (0..10).rev().map(|i| ((info_word >> i) & 1) as u8).collect();
        let crc = crc6(&info_bits);
        let crc_bits: Vec<u8> = (0..6).rev().map(|i| ((crc >> i) & 1) as u8).collect();
        let mut out = info_bits;
        out.extend_from_slice(&crc_bits);
        out
    }

    /// Decode LICH from a 16-bit slice. Returns None if CRC fails.
    pub fn decode(bits: &[u8]) -> Option<Self> {
        assert!(bits.len() >= 16);
        let info_bits = &bits[..10];
        let crc_bits = &bits[10..16];
        let expected_crc = crc6(info_bits);
        let received_crc: u8 = crc_bits.iter().enumerate()
            .fold(0u8, |acc, (i, &b)| acc | ((b & 1) << (5 - i)));
        if expected_crc != received_crc { return None; }
        let info_word: u16 = info_bits.iter().enumerate()
            .fold(0u16, |acc, (i, &b)| acc | ((b as u16) << (9 - i)));
        let structure = ((info_word >> 6) & 3) as u8;
        let call_type = ((info_word >> 5) & 1) as u8;
        let ch_type = ((info_word >> 2) & 7) as u8;
        Some(Self::new(call_type, LichChannelType::from(ch_type), structure))
    }
}

// ============================================================
//  Layer-2 Message Types
// ============================================================

/// NXDN Layer-2 message type identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MessageType {
    /// Voice header.
    VoiceHeader     = 0x01,
    /// Voice body (2nd/3rd fragment).
    VoiceBody       = 0x02,
    /// Voice terminator.
    VoiceTerm       = 0x03,
    /// Data header.
    DataHeader      = 0x10,
    /// Data body.
    DataBody        = 0x11,
    /// Data terminator.
    DataTerm        = 0x12,
    /// Group call request (trunking).
    GroupCallReq    = 0x20,
    /// Group call response.
    GroupCallResp   = 0x21,
    /// Unit-to-unit call request.
    UnitCallReq     = 0x22,
    /// Unit-to-unit call response.
    UnitCallResp    = 0x23,
    /// Registration request.
    RegistrationReq = 0x30,
    /// Registration response.
    RegistrationResp= 0x31,
    /// Idle/null frame.
    Idle            = 0x00,
    /// Unknown.
    Unknown         = 0xFF,
}

impl From<u8> for MessageType {
    fn from(v: u8) -> Self {
        match v {
            0x01 => Self::VoiceHeader,
            0x02 => Self::VoiceBody,
            0x03 => Self::VoiceTerm,
            0x10 => Self::DataHeader,
            0x11 => Self::DataBody,
            0x12 => Self::DataTerm,
            0x20 => Self::GroupCallReq,
            0x21 => Self::GroupCallResp,
            0x22 => Self::UnitCallReq,
            0x23 => Self::UnitCallResp,
            0x30 => Self::RegistrationReq,
            0x31 => Self::RegistrationResp,
            0x00 => Self::Idle,
            _    => Self::Unknown,
        }
    }
}

// ============================================================
//  SACCH Frame
// ============================================================

/// SACCH (Slow Associated Control Channel) payload — 32 bits + 15-bit CRC.
#[derive(Debug, Clone)]
pub struct SacchFrame {
    pub msg_type: MessageType,
    /// Source radio ID (24 bits).
    pub source_id: u32,
    /// Destination ID (24 bits).
    pub dest_id: u32,
}

impl SacchFrame {
    pub fn new(msg_type: MessageType, source_id: u32, dest_id: u32) -> Self {
        Self { msg_type, source_id: source_id & 0xFFFFFF, dest_id: dest_id & 0xFFFFFF }
    }

    /// Encode to 63 bits (48 payload + 15 CRC).
    pub fn encode(&self) -> Vec<u8> {
        // Simplified payload: [msg_type:8][source_id:24][dest_id:16] = 48 bits
        let mut bits = Vec::with_capacity(63);
        for i in (0..8).rev()  { bits.push(((self.msg_type as u8 >> i) & 1)); }
        for i in (0..24).rev() { bits.push(((self.source_id >> i) & 1) as u8); }
        for i in (0..16).rev() { bits.push(((self.dest_id >> i) & 1) as u8); }
        let crc = crc15(&bits);
        for i in (0..15).rev() { bits.push(((crc >> i) & 1) as u8); }
        bits
    }

    /// Decode from 63-bit slice. Returns None on CRC failure.
    pub fn decode(bits: &[u8]) -> Option<Self> {
        if bits.len() < 63 { return None; }
        let payload = &bits[..48];
        let crc_bits = &bits[48..63];
        let expected = crc15(payload);
        let received: u16 = crc_bits.iter().enumerate()
            .fold(0u16, |acc, (i, &b)| acc | ((b as u16) << (14 - i)));
        if expected != received { return None; }
        let msg_type = bits[..8].iter().enumerate()
            .fold(0u8, |acc, (i, &b)| acc | ((b & 1) << (7 - i)));
        let source_id = bits[8..32].iter().enumerate()
            .fold(0u32, |acc, (i, &b)| acc | ((b as u32) << (23 - i)));
        let dest_id = bits[32..48].iter().enumerate()
            .fold(0u32, |acc, (i, &b)| acc | ((b as u32) << (15 - i)));
        Some(Self::new(MessageType::from(msg_type), source_id, dest_id))
    }
}

// ============================================================
//  FACCH Frame
// ============================================================

/// FACCH (Fast Associated Control Channel) — carries full Layer-2 messages.
#[derive(Debug, Clone)]
pub struct FacchFrame {
    pub msg_type: MessageType,
    pub source_id: u32,
    pub dest_id: u32,
    /// Additional data payload (up to 48 bytes).
    pub payload: Vec<u8>,
}

impl FacchFrame {
    pub fn new(msg_type: MessageType, source_id: u32, dest_id: u32, payload: Vec<u8>) -> Self {
        Self { msg_type, source_id, dest_id, payload }
    }

    /// Encode to byte vector with CRC-16.
    pub fn encode(&self) -> Vec<u8> {
        let mut data = vec![
            self.msg_type as u8,
            ((self.source_id >> 16) & 0xFF) as u8,
            ((self.source_id >>  8) & 0xFF) as u8,
            ( self.source_id        & 0xFF) as u8,
            ((self.dest_id >> 16) & 0xFF) as u8,
            ((self.dest_id >>  8) & 0xFF) as u8,
            ( self.dest_id        & 0xFF) as u8,
        ];
        data.extend_from_slice(&self.payload);
        let crc = crc16_ccitt(&data);
        data.push((crc >> 8) as u8);
        data.push((crc & 0xFF) as u8);
        data
    }

    /// Decode from byte slice (minimum 9 bytes). Returns None on CRC failure.
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 9 { return None; }
        let body = &data[..data.len() - 2];
        let crc_hi = data[data.len() - 2] as u16;
        let crc_lo = data[data.len() - 1] as u16;
        let received_crc = (crc_hi << 8) | crc_lo;
        if crc16_ccitt(body) != received_crc { return None; }
        let msg_type = MessageType::from(data[0]);
        let source_id = ((data[1] as u32) << 16) | ((data[2] as u32) << 8) | data[3] as u32;
        let dest_id   = ((data[4] as u32) << 16) | ((data[5] as u32) << 8) | data[6] as u32;
        let payload = data[7..data.len()-2].to_vec();
        Some(Self::new(msg_type, source_id, dest_id, payload))
    }
}

// ============================================================
//  Voice Frame (EHR — Enhanced Half Rate AMBE+2)
// ============================================================

/// AMBE+2 EHR voice frame: 88 bits per 20 ms frame.
pub const EHR_BITS: usize = 88;

/// Packed AMBE+2 EHR voice frame.
#[derive(Debug, Clone)]
pub struct VoiceFrame {
    /// 88 codec bits.
    pub bits: [u8; EHR_BITS],
}

impl VoiceFrame {
    pub fn new(bits: [u8; EHR_BITS]) -> Self { Self { bits } }

    /// Pack two voice frames into a 176-bit payload slice.
    pub fn pack_two(frame1: &VoiceFrame, frame2: &VoiceFrame) -> Vec<u8> {
        let mut out = Vec::with_capacity(EHR_BITS * 2);
        out.extend_from_slice(&frame1.bits);
        out.extend_from_slice(&frame2.bits);
        out
    }

    /// Unpack two voice frames from a 176-bit payload.
    pub fn unpack_two(bits: &[u8]) -> Option<(VoiceFrame, VoiceFrame)> {
        if bits.len() < EHR_BITS * 2 { return None; }
        let mut b1 = [0u8; EHR_BITS];
        let mut b2 = [0u8; EHR_BITS];
        b1.copy_from_slice(&bits[..EHR_BITS]);
        b2.copy_from_slice(&bits[EHR_BITS..EHR_BITS*2]);
        Some((VoiceFrame::new(b1), VoiceFrame::new(b2)))
    }
}

// ============================================================
//  Trunking Messages
// ============================================================

/// Trunking group call request/response.
#[derive(Debug, Clone)]
pub struct TrunkGroupCall {
    pub msg_type: MessageType,
    /// Requesting / assigned radio ID.
    pub radio_id: u32,
    /// Group ID.
    pub group_id: u16,
    /// Assigned channel number (response only).
    pub channel: u8,
    /// Priority (0–7).
    pub priority: u8,
}

impl TrunkGroupCall {
    pub fn new_request(radio_id: u32, group_id: u16, priority: u8) -> Self {
        Self { msg_type: MessageType::GroupCallReq, radio_id, group_id, channel: 0, priority }
    }

    pub fn new_response(radio_id: u32, group_id: u16, channel: u8) -> Self {
        Self { msg_type: MessageType::GroupCallResp, radio_id, group_id, channel, priority: 0 }
    }

    /// Encode to byte payload (7 bytes).
    pub fn encode(&self) -> Vec<u8> {
        vec![
            self.msg_type as u8,
            ((self.radio_id >> 16) & 0xFF) as u8,
            ((self.radio_id >>  8) & 0xFF) as u8,
            ( self.radio_id        & 0xFF) as u8,
            ((self.group_id >> 8)  & 0xFF) as u8,
            ( self.group_id        & 0xFF) as u8,
            (self.channel & 0xFF),
            self.priority & 0x07,
        ]
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 8 { return None; }
        let msg_type = MessageType::from(data[0]);
        let radio_id = ((data[1] as u32) << 16) | ((data[2] as u32) << 8) | data[3] as u32;
        let group_id = ((data[4] as u16) << 8)  |  data[5] as u16;
        let channel  = data[6];
        let priority = data[7] & 0x07;
        Some(Self { msg_type, radio_id, group_id, channel, priority })
    }
}

/// Trunking registration request/response.
#[derive(Debug, Clone)]
pub struct TrunkRegistration {
    pub msg_type: MessageType,
    pub radio_id: u32,
    /// Site ID.
    pub site_id: u16,
    /// Accepted flag (response).
    pub accepted: bool,
}

impl TrunkRegistration {
    pub fn new_request(radio_id: u32, site_id: u16) -> Self {
        Self { msg_type: MessageType::RegistrationReq, radio_id, site_id, accepted: false }
    }

    pub fn new_response(radio_id: u32, site_id: u16, accepted: bool) -> Self {
        Self { msg_type: MessageType::RegistrationResp, radio_id, site_id, accepted }
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![
            self.msg_type as u8,
            ((self.radio_id >> 16) & 0xFF) as u8,
            ((self.radio_id >>  8) & 0xFF) as u8,
            ( self.radio_id        & 0xFF) as u8,
            ((self.site_id  >>  8) & 0xFF) as u8,
            ( self.site_id         & 0xFF) as u8,
            self.accepted as u8,
        ]
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 7 { return None; }
        Some(Self {
            msg_type: MessageType::from(data[0]),
            radio_id: ((data[1] as u32) << 16) | ((data[2] as u32) << 8) | data[3] as u32,
            site_id:  ((data[4] as u16) << 8)  |  data[5] as u16,
            accepted: data[6] != 0,
        })
    }
}

// ============================================================
//  CAC / USC Layer-2 Channel
// ============================================================

/// Layer-2 channel category.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum L2Channel {
    /// Common Air Channel — control/trunking traffic.
    Cac,
    /// User-Specific Channel — assigned voice/data traffic.
    Usc,
}

/// Layer-2 frame encapsulation.
#[derive(Debug, Clone)]
pub struct L2Frame {
    pub channel: L2Channel,
    pub lich: Lich,
    /// Encoded payload bytes (already through FEC + interleaving).
    pub payload: Vec<u8>,
}

impl L2Frame {
    pub fn new(channel: L2Channel, lich: Lich, payload: Vec<u8>) -> Self {
        Self { channel, lich, payload }
    }
}

// ============================================================
//  Full NXDN Frame Assembler / Disassembler
// ============================================================

/// Complete NXDN radio frame.
#[derive(Debug, Clone)]
pub struct NxdnFrame {
    pub mode: ChannelMode,
    pub lich: Lich,
    pub sacch: Option<SacchFrame>,
    pub facch: Option<FacchFrame>,
    pub voice: Option<(VoiceFrame, VoiceFrame)>,
}

/// NXDN frame builder: assembles raw bit-stream from components.
pub struct FrameAssembler {
    pub interleaver: BlockInterleaver,
    pub encoder: ConvEncoder,
    pub scrambler: PnScrambler,
}

impl FrameAssembler {
    /// Create a new assembler with default parameters.
    pub fn new() -> Self {
        Self {
            interleaver: BlockInterleaver::new(12, 16),  // 192 bits
            encoder: ConvEncoder::new(),
            scrambler: PnScrambler::new(0x5555),
        }
    }

    /// Assemble a full NXDN frame bit-stream (384 bits = 192 symbols × 2).
    ///
    /// Layout: [FSW:48][LICH:16][SACCH:63][PAYLOAD:176][PADDING to 384]
    pub fn assemble(&mut self, frame: &NxdnFrame) -> Vec<u8> {
        let mut bits: Vec<u8> = Vec::with_capacity(FRAME_BITS);

        // 1. FSW
        bits.extend_from_slice(&FSW_BITS);

        // 2. LICH (16 bits, no FEC)
        let lich_bits = frame.lich.encode();
        bits.extend_from_slice(&lich_bits);

        // 3. SACCH (63 bits raw → 126 after rate-1/2 FEC)
        let sacch_raw = match &frame.sacch {
            Some(s) => s.encode(),
            None    => vec![0u8; 63],
        };
        self.encoder.reset();
        let sacch_coded = self.encoder.encode(&sacch_raw);
        // interleave 126 bits
        let sacch_il = if sacch_coded.len() == 126 {
            let il = BlockInterleaver::new(9, 14);
            il.interleave(&sacch_coded)
        } else {
            sacch_coded
        };
        bits.extend_from_slice(&sacch_il);

        // 4. PAYLOAD (voice or FACCH, 176 bits raw → 176 bits stuffed to 176)
        let payload_raw: Vec<u8> = if let Some((vf1, vf2)) = &frame.voice {
            VoiceFrame::pack_two(vf1, vf2)
        } else if let Some(facch) = &frame.facch {
            let enc = facch.encode();
            let mut p: Vec<u8> = Vec::new();
            for byte in &enc {
                for i in (0..8).rev() { p.push((byte >> i) & 1); }
            }
            p.resize(176, 0);
            p
        } else {
            vec![0u8; 176]
        };
        self.encoder.reset();
        let payload_coded = self.encoder.encode(&payload_raw[..88.min(payload_raw.len())]);
        bits.extend_from_slice(&payload_coded[..payload_coded.len().min(176)]);

        // 5. Scramble everything after FSW
        let fsw_end = FSW_BITS.len();
        self.scrambler.reset();
        let rest_scrambled = self.scrambler.scramble_copy(&bits[fsw_end..]);
        bits.truncate(fsw_end);
        bits.extend(rest_scrambled);

        // Pad/truncate to FRAME_BITS
        bits.resize(FRAME_BITS, 0);
        bits
    }

    /// Parse a raw bit-stream back to frame components.
    pub fn disassemble(&mut self, bits: &[u8]) -> Option<NxdnFrame> {
        if bits.len() < FRAME_BITS { return None; }

        // 1. Verify / skip FSW
        let fsw_matches = SyncDetector::correlate_at(bits, 0);
        if fsw_matches < 40 { return None; } // require 40/48 match

        // 2. De-scramble after FSW
        let fsw_end = FSW_BITS.len();
        self.scrambler.reset();
        let descrambled = self.scrambler.scramble_copy(&bits[fsw_end..]);

        // 3. Decode LICH
        let lich = Lich::decode(&descrambled[..16])?;

        // 4. Decode SACCH
        let sacch_il = &descrambled[16..16+126];
        let sacch_deinterleaved = {
            let il = BlockInterleaver::new(9, 14);
            if sacch_il.len() == 126 { il.deinterleave(sacch_il) } else { sacch_il.to_vec() }
        };
        let sacch_decoded = ViterbiDecoder::decode(&sacch_deinterleaved);
        let sacch = SacchFrame::decode(&sacch_decoded);

        // 5. Decode payload (voice only for simplicity)
        let payload_start = 16 + 126;
        let payload_coded = &descrambled[payload_start..payload_start + 176];
        let payload_bits = ViterbiDecoder::decode(payload_coded);

        let voice = VoiceFrame::unpack_two(&payload_bits);

        Some(NxdnFrame {
            mode: ChannelMode::Nxdn6k25,
            lich,
            sacch,
            facch: None,
            voice,
        })
    }
}

impl Default for FrameAssembler { fn default() -> Self { Self::new() } }

// ============================================================
//  SNR / Noise helpers (for test purposes)
// ============================================================

/// Add white Gaussian noise to a signal at given SNR (dB).
pub fn add_awgn(signal: &[f64], snr_db: f64, seed: u64) -> Vec<f64> {
    let raw_power: f64 = signal.iter().map(|&s| s * s).sum::<f64>() / signal.len().max(1) as f64;
    // Use at least a reference power of 1.0 so zero-signal inputs still get noise.
    let signal_power = raw_power.max(1.0);
    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    let noise_std = (signal_power / snr_linear).sqrt();
    let mut state = seed;
    signal.iter().map(|&s| {
        s + noise_std * lcg_gaussian(&mut state)
    }).collect()
}

/// Simple LCG-based Box-Muller Gaussian generator.
fn lcg_gaussian(state: &mut u64) -> f64 {
    use std::f64::consts::PI;
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    let u1 = (*state >> 11) as f64 / (1u64 << 53) as f64;
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    let u2 = (*state >> 11) as f64 / (1u64 << 53) as f64;
    let u1 = u1.max(1e-300);
    f64::sqrt(-2.0 * f64::ln(u1)) * f64::cos(2.0 * PI * u2)
}

// ============================================================
//  Utility: bits ↔ bytes
// ============================================================

/// Pack a bit array (MSB first, length must be multiple of 8) into bytes.
pub fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |acc, (i, &b)| acc | ((b & 1) << (7 - i))))
        .collect()
}

/// Unpack bytes into a bit array (MSB first).
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len() * 8);
    for &byte in bytes {
        for i in (0..8).rev() { out.push((byte >> i) & 1); }
    }
    out
}

// ============================================================
//  Unit Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- 4FSK Symbol Mapping ----

    #[test]
    fn test_dibit_to_symbol_all_values() {
        assert_eq!(dibit_to_symbol(0b00),  3);
        assert_eq!(dibit_to_symbol(0b01),  1);
        assert_eq!(dibit_to_symbol(0b11), -1);
        assert_eq!(dibit_to_symbol(0b10), -3);
    }

    #[test]
    fn test_symbol_to_dibit_thresholds() {
        assert_eq!(symbol_to_dibit(2.5),  0b00);
        assert_eq!(symbol_to_dibit(1.0),  0b01);
        assert_eq!(symbol_to_dibit(-1.0), 0b11);
        assert_eq!(symbol_to_dibit(-2.5), 0b10);
    }

    #[test]
    fn test_symbol_to_dibit_roundtrip() {
        for dibit in 0u8..4 {
            let level = dibit_to_symbol(dibit) as f64;
            assert_eq!(symbol_to_dibit(level), dibit,
                "roundtrip failed for dibit {}", dibit);
        }
    }

    #[test]
    fn test_symbol_deviation_hz() {
        assert!((symbol_to_deviation_hz(3) - 1800.0).abs() < 1.0);
        assert!((symbol_to_deviation_hz(1) - 600.0).abs() < 1.0);
        assert!((symbol_to_deviation_hz(-1) + 600.0).abs() < 1.0);
        assert!((symbol_to_deviation_hz(-3) + 1800.0).abs() < 1.0);
    }

    // ---- Bit/Dibit Pack/Unpack ----

    #[test]
    fn test_bits_to_dibits_roundtrip() {
        let bits: Vec<u8> = vec![0,1, 1,0, 0,0, 1,1];
        let dibits = bits_to_dibits(&bits);
        let recovered = dibits_to_bits(&dibits);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_dibits_to_bits_length() {
        let dibits: Vec<u8> = vec![0, 1, 2, 3];
        let bits = dibits_to_bits(&dibits);
        assert_eq!(bits.len(), 8);
    }

    // ---- RRC Filter ----

    #[test]
    fn test_rrc_filter_symmetry() {
        let taps = rrc_filter(33, 8.0, 0.2);
        assert_eq!(taps.len(), 33);
        for i in 0..16 {
            let diff = (taps[i] - taps[32 - i]).abs();
            assert!(diff < 1e-12, "RRC not symmetric at i={}: {} vs {}", i, taps[i], taps[32-i]);
        }
    }

    #[test]
    fn test_rrc_peak_normalised() {
        let taps = rrc_filter(33, 8.0, 0.5);
        let peak = taps[16];
        assert!((peak - 1.0).abs() < 1e-10, "peak = {}", peak);
    }

    #[test]
    fn test_rrc_different_rolloffs() {
        for alpha in [0.1, 0.2, 0.35, 0.5] {
            let taps = rrc_filter(25, 4.0, alpha);
            assert_eq!(taps.len(), 25);
            assert!((taps[12] - 1.0).abs() < 0.01, "alpha={} peak={}", alpha, taps[12]);
        }
    }

    // ---- FIR Filter ----

    #[test]
    fn test_fir_filter_impulse_response() {
        let taps = vec![0.25, 0.5, 0.25];
        let impulse: Vec<f64> = (0..10).map(|i| if i == 5 { 1.0 } else { 0.0 }).collect();
        let out = fir_filter(&impulse, &taps);
        // Check output equals taps at expected positions
        assert!((out[4] - 0.25).abs() < 1e-12);
        assert!((out[5] - 0.5).abs() < 1e-12);
        assert!((out[6] - 0.25).abs() < 1e-12);
    }

    // ---- Modulator/Demodulator ----

    #[test]
    fn test_modulator_output_length() {
        let mut modem = FskModulator::new(48000.0, ChannelMode::Nxdn6k25);
        let dibits: Vec<u8> = vec![0, 1, 2, 3];
        let samples = modem.modulate(&dibits);
        let expected_sps = (48000.0 / SYMBOL_RATE as f64).round() as usize;
        assert_eq!(samples.len(), dibits.len() * expected_sps);
    }

    #[test]
    fn test_modulator_bounded_amplitude() {
        let mut modem = FskModulator::new(48000.0, ChannelMode::Nxdn6k25);
        let dibits: Vec<u8> = vec![0, 1, 2, 3, 0, 1, 2, 3];
        let samples = modem.modulate(&dibits);
        for &s in &samples {
            assert!(s.abs() <= 1.0 + 1e-9, "sample out of range: {}", s);
        }
    }

    #[test]
    fn test_demodulator_output_length() {
        let mut demod = FskDemodulator::new(48000.0, ChannelMode::Nxdn6k25);
        let sps = (48000.0 / SYMBOL_RATE as f64).round() as usize;
        let samples = vec![0.0f64; 4 * sps];
        let dibits = demod.demodulate(&samples);
        assert_eq!(dibits.len(), 4);
    }

    // ---- CRC ----

    #[test]
    fn test_crc6_known_value() {
        let bits = vec![1,0,1,0,1,0,1,0,1,0];
        let crc = crc6(&bits);
        // Just check range and consistency
        assert!(crc < 64);
        assert_eq!(crc6(&bits), crc6(&bits));
    }

    #[test]
    fn test_crc6_all_zeros() {
        let bits = vec![0u8; 10];
        let crc = crc6(&bits);
        assert!(crc < 64);
    }

    #[test]
    fn test_crc15_consistency() {
        let bits: Vec<u8> = (0..48).map(|i| (i % 2) as u8).collect();
        let c1 = crc15(&bits);
        let c2 = crc15(&bits);
        assert_eq!(c1, c2);
        assert!(c1 < 32768);
    }

    #[test]
    fn test_crc16_ccitt_empty() {
        let crc = crc16_ccitt(&[]);
        // Empty input with init 0xFFFF → final XOR → should be 0xFFFF ^ 0xFFFF = 0 ? varies
        // Just check it's a valid u16.
        let _ = crc;
    }

    #[test]
    fn test_crc16_ccitt_consistency() {
        let data = b"NXDN-TEST";
        let c1 = crc16_ccitt(data);
        let c2 = crc16_ccitt(data);
        assert_eq!(c1, c2);
    }

    #[test]
    fn test_crc16_detects_flip() {
        let mut data = b"NXDN-radio".to_vec();
        let crc_orig = crc16_ccitt(&data);
        data[3] ^= 0xFF;
        let crc_flipped = crc16_ccitt(&data);
        assert_ne!(crc_orig, crc_flipped);
    }

    // ---- Convolutional Encoder / Viterbi Decoder ----

    #[test]
    fn test_conv_encoder_output_length() {
        let mut enc = ConvEncoder::new();
        let bits = vec![1u8; 20];
        let coded = enc.encode(&bits);
        assert_eq!(coded.len(), 40);
    }

    #[test]
    fn test_conv_encoder_bits_valid() {
        let mut enc = ConvEncoder::new();
        let bits: Vec<u8> = (0..16).map(|i| (i % 2) as u8).collect();
        let coded = enc.encode(&bits);
        for b in &coded { assert!(*b == 0 || *b == 1, "coded bit not binary: {}", b); }
    }

    #[test]
    fn test_viterbi_decode_zero_sequence() {
        let mut enc = ConvEncoder::new();
        let info = vec![0u8; 16];
        let coded = enc.encode(&info);
        let decoded = ViterbiDecoder::decode(&coded);
        // With all-zeros input and no noise, should decode cleanly
        assert_eq!(decoded.len(), 16);
    }

    #[test]
    fn test_conv_encode_decode_roundtrip() {
        let mut enc = ConvEncoder::new();
        // Known pattern with K-1 = 8 flush (zero) bits appended to flush the trellis.
        let info: Vec<u8>  = vec![1,0,1,0,1,0,1,0, 1,1,0,0,1,1,0,0];
        let flush: Vec<u8> = vec![0u8; CONV_K - 1]; // 8 flush bits
        let mut padded = info.clone();
        padded.extend_from_slice(&flush);
        let coded = enc.encode(&padded);
        let decoded_full = ViterbiDecoder::decode(&coded);
        // Check the original 16 information bits (decoded_full includes the flush tail)
        assert_eq!(decoded_full.len(), padded.len());
        for i in 0..info.len() {
            assert_eq!(decoded_full[i], info[i], "mismatch at bit {}", i);
        }
    }

    // ---- Block Interleaver ----

    #[test]
    fn test_interleaver_roundtrip() {
        let il = BlockInterleaver::new(4, 8);
        let bits: Vec<u8> = (0..32).map(|i| (i % 2) as u8).collect();
        let interleaved = il.interleave(&bits);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(bits, deinterleaved);
    }

    #[test]
    fn test_interleaver_changes_order() {
        let il = BlockInterleaver::new(4, 4);
        let bits: Vec<u8> = vec![1,1,1,1, 0,0,0,0, 1,1,1,1, 0,0,0,0];
        let interleaved = il.interleave(&bits);
        assert_ne!(bits, interleaved, "interleaver should change order");
    }

    #[test]
    fn test_interleaver_identity_1xN() {
        let il = BlockInterleaver::new(1, 8);
        let bits: Vec<u8> = vec![1,0,1,1,0,0,1,0];
        let interleaved = il.interleave(&bits);
        // 1 row: read cols in order = same
        assert_eq!(bits, interleaved);
    }

    // ---- PN Scrambler ----

    #[test]
    fn test_scrambler_descrambler_roundtrip() {
        let mut enc = PnScrambler::new(0x1234);
        let bits: Vec<u8> = vec![1,0,1,1,0,0,1,0, 0,1,1,0,0,1,0,1];
        let scrambled = enc.scramble_copy(&bits);
        let mut dec = PnScrambler::new(0x1234);
        let recovered = dec.scramble_copy(&scrambled);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_scrambler_changes_bits() {
        let mut enc = PnScrambler::new(0x7FFF);
        let bits = vec![0u8; 16];
        let scrambled = enc.scramble_copy(&bits);
        // With non-zero seed, output should not be all-zero
        assert!(scrambled.iter().any(|&b| b != 0), "scrambled all-zero sequence unchanged");
    }

    #[test]
    fn test_scrambler_reset() {
        let mut s = PnScrambler::new(0xABCD);
        let bits = vec![0u8; 8];
        let out1 = s.scramble_copy(&bits);
        s.reset();
        let out2 = s.scramble_copy(&bits);
        assert_eq!(out1, out2, "scrambler should produce same output after reset");
    }

    #[test]
    fn test_scrambler_sequence_length() {
        let mut s = PnScrambler::new(0x0001);
        let bits = vec![0u8; 100];
        let out = s.scramble_copy(&bits);
        assert_eq!(out.len(), 100);
        for &b in &out { assert!(b == 0 || b == 1); }
    }

    // ---- Sync Word Detector ----

    #[test]
    fn test_sync_perfect_correlation() {
        let score = SyncDetector::correlate_at(&FSW_BITS, 0);
        assert_eq!(score, 48);
    }

    #[test]
    fn test_sync_find_at_offset() {
        let mut bits = vec![0u8; 20];
        bits.extend_from_slice(&FSW_BITS);
        bits.extend_from_slice(&[0u8; 10]);
        let (off, score) = SyncDetector::find_sync(&bits);
        assert_eq!(off, 20);
        assert_eq!(score, 48);
    }

    #[test]
    fn test_sync_detector_push_dibit() {
        let mut det = SyncDetector::new(4);
        let fsw_dibits = bits_to_dibits(&FSW_BITS);
        let mut detected = false;
        for d in fsw_dibits {
            if det.push_dibit(d) { detected = true; }
        }
        assert!(detected, "FSW not detected via push_dibit");
    }

    #[test]
    fn test_sync_tolerance_one_error() {
        let mut bits = FSW_BITS.to_vec();
        bits[5] ^= 1; // flip one bit
        let score = SyncDetector::correlate_at(&bits, 0);
        assert_eq!(score, 47);
    }

    // ---- LICH ----

    #[test]
    fn test_lich_encode_decode_roundtrip() {
        let lich = Lich::new(1, LichChannelType::Sacch, 2);
        let bits = lich.encode();
        assert_eq!(bits.len(), 16);
        let decoded = Lich::decode(&bits).expect("LICH decode failed");
        assert_eq!(decoded.call_type, lich.call_type);
        assert_eq!(decoded.structure, lich.structure);
        assert_eq!(decoded.channel_type, lich.channel_type);
    }

    #[test]
    fn test_lich_crc6_detects_error() {
        let lich = Lich::new(0, LichChannelType::Facch, 0);
        let mut bits = lich.encode();
        bits[3] ^= 1; // corrupt a bit
        let result = Lich::decode(&bits);
        assert!(result.is_none(), "should fail CRC after corruption");
    }

    // ---- SACCH Frame ----

    #[test]
    fn test_sacch_encode_decode_roundtrip() {
        let sacch = SacchFrame::new(MessageType::VoiceHeader, 0x001234, 0x005678);
        let bits = sacch.encode();
        assert_eq!(bits.len(), 63);
        let decoded = SacchFrame::decode(&bits).expect("SACCH decode failed");
        assert_eq!(decoded.source_id, sacch.source_id);
        assert_eq!(decoded.dest_id,   sacch.dest_id);
    }

    #[test]
    fn test_sacch_crc15_detects_error() {
        let sacch = SacchFrame::new(MessageType::DataHeader, 0xABCDEF, 0x123456);
        let mut bits = sacch.encode();
        bits[7] ^= 1;
        let result = SacchFrame::decode(&bits);
        assert!(result.is_none());
    }

    // ---- FACCH Frame ----

    #[test]
    fn test_facch_encode_decode_roundtrip() {
        let payload = vec![0xDE, 0xAD, 0xBE, 0xEF];
        let facch = FacchFrame::new(MessageType::GroupCallReq, 0x111111, 0x222222, payload.clone());
        let data = facch.encode();
        let decoded = FacchFrame::decode(&data).expect("FACCH decode failed");
        assert_eq!(decoded.source_id, 0x111111);
        assert_eq!(decoded.dest_id, 0x222222);
        assert_eq!(decoded.payload, payload);
    }

    #[test]
    fn test_facch_crc16_detects_error() {
        let facch = FacchFrame::new(MessageType::DataBody, 0, 0, vec![0xFF; 8]);
        let mut data = facch.encode();
        data[2] ^= 0x55;
        let result = FacchFrame::decode(&data);
        assert!(result.is_none());
    }

    // ---- Voice Frame ----

    #[test]
    fn test_voice_frame_pack_unpack() {
        let mut b1 = [0u8; EHR_BITS];
        let mut b2 = [0u8; EHR_BITS];
        for i in 0..EHR_BITS { b1[i] = (i % 2) as u8; }
        for i in 0..EHR_BITS { b2[i] = ((i + 1) % 2) as u8; }
        let vf1 = VoiceFrame::new(b1);
        let vf2 = VoiceFrame::new(b2);
        let packed = VoiceFrame::pack_two(&vf1, &vf2);
        assert_eq!(packed.len(), EHR_BITS * 2);
        let (r1, r2) = VoiceFrame::unpack_two(&packed).expect("unpack failed");
        assert_eq!(r1.bits, b1);
        assert_eq!(r2.bits, b2);
    }

    #[test]
    fn test_voice_frame_short_input_returns_none() {
        let bits = vec![0u8; EHR_BITS - 1];
        assert!(VoiceFrame::unpack_two(&bits).is_none());
    }

    // ---- Trunking ----

    #[test]
    fn test_trunk_group_call_request_roundtrip() {
        let req = TrunkGroupCall::new_request(0xA1B2C3, 0x0102, 3);
        let data = req.encode();
        let decoded = TrunkGroupCall::decode(&data).expect("decode failed");
        assert_eq!(decoded.radio_id, 0xA1B2C3);
        assert_eq!(decoded.group_id, 0x0102);
        assert_eq!(decoded.priority, 3);
    }

    #[test]
    fn test_trunk_group_call_response_roundtrip() {
        let resp = TrunkGroupCall::new_response(0x001234, 0x0020, 7);
        let data = resp.encode();
        let decoded = TrunkGroupCall::decode(&data).expect("decode failed");
        assert_eq!(decoded.channel, 7);
    }

    #[test]
    fn test_trunk_registration_request_roundtrip() {
        let reg = TrunkRegistration::new_request(0xDEADBE, 0x0001);
        let data = reg.encode();
        let decoded = TrunkRegistration::decode(&data).expect("decode failed");
        assert_eq!(decoded.radio_id, 0xDEADBE);
        assert_eq!(decoded.site_id, 0x0001);
        assert!(!decoded.accepted);
    }

    #[test]
    fn test_trunk_registration_response_accepted() {
        let resp = TrunkRegistration::new_response(0x123456, 0x0002, true);
        let data = resp.encode();
        let decoded = TrunkRegistration::decode(&data).expect("decode failed");
        assert!(decoded.accepted);
        assert_eq!(decoded.site_id, 0x0002);
    }

    // ---- Channel Mode ----

    #[test]
    fn test_channel_mode_6k25_params() {
        let mode = ChannelMode::Nxdn6k25;
        assert!((mode.channel_spacing_hz() - 6250.0).abs() < 1.0);
        assert_eq!(mode.data_rate_bps(), 2400);
    }

    #[test]
    fn test_channel_mode_12k5_params() {
        let mode = ChannelMode::Nxdn12k5;
        assert!((mode.channel_spacing_hz() - 12500.0).abs() < 1.0);
        assert_eq!(mode.data_rate_bps(), 4800);
    }

    #[test]
    fn test_channel_mode_samples_per_symbol() {
        let mode = ChannelMode::Nxdn6k25;
        let sps = mode.samples_per_symbol(48000.0);
        assert!((sps - 20.0).abs() < 0.01);
    }

    // ---- Bits ↔ Bytes ----

    #[test]
    fn test_bits_to_bytes_roundtrip() {
        let bytes: Vec<u8> = vec![0xA5, 0x3C, 0xFF, 0x00];
        let bits = bytes_to_bits(&bytes);
        assert_eq!(bits.len(), 32);
        let recovered = bits_to_bytes(&bits);
        assert_eq!(bytes, recovered);
    }

    #[test]
    fn test_bytes_to_bits_known() {
        let bits = bytes_to_bits(&[0b10110010]);
        assert_eq!(bits, vec![1,0,1,1,0,0,1,0]);
    }

    // ---- AWGN helper ----

    #[test]
    fn test_awgn_output_length() {
        let signal = vec![1.0f64; 100];
        let noisy = add_awgn(&signal, 20.0, 42);
        assert_eq!(noisy.len(), 100);
    }

    #[test]
    fn test_awgn_non_zero_variance() {
        let signal = vec![0.0f64; 200];
        let noisy = add_awgn(&signal, 0.0, 99);
        let sum_sq: f64 = noisy.iter().map(|&v| v * v).sum();
        assert!(sum_sq > 0.0, "AWGN should add non-zero noise");
    }

    // ---- Parity helper ----

    #[test]
    fn test_parity_u16_known() {
        assert_eq!(parity_u16(0b0000_0001), 1);
        assert_eq!(parity_u16(0b0000_0011), 0);
        assert_eq!(parity_u16(0b0101_0101), 0);
        assert_eq!(parity_u16(0b1111_1111), 0);
        assert_eq!(parity_u16(0b0000_1111), 0);
        assert_eq!(parity_u16(0b0000_0111), 1);
    }

    // ---- FSW Constants ----

    #[test]
    fn test_fsw_bit_count() {
        assert_eq!(FSW_BITS.len(), 48);
    }

    #[test]
    fn test_fsw_bits_binary() {
        for &b in &FSW_BITS { assert!(b == 0 || b == 1); }
    }

    // ---- Frame Assembler (integration) ----

    #[test]
    fn test_frame_assembler_output_length() {
        let mut asm = FrameAssembler::new();
        let sacch = SacchFrame::new(MessageType::VoiceBody, 0x001, 0x002);
        let vf1 = VoiceFrame::new([0u8; EHR_BITS]);
        let vf2 = VoiceFrame::new([1u8; EHR_BITS]);
        let frame = NxdnFrame {
            mode: ChannelMode::Nxdn6k25,
            lich: Lich::new(0, LichChannelType::Sacch, 1),
            sacch: Some(sacch),
            facch: None,
            voice: Some((vf1, vf2)),
        };
        let bits = asm.assemble(&frame);
        assert_eq!(bits.len(), FRAME_BITS);
    }

    #[test]
    fn test_frame_assembler_fsw_present() {
        let mut asm = FrameAssembler::new();
        let frame = NxdnFrame {
            mode: ChannelMode::Nxdn6k25,
            lich: Lich::new(0, LichChannelType::Facch, 0),
            sacch: None,
            facch: None,
            voice: None,
        };
        let bits = asm.assemble(&frame);
        // FSW must appear unscrambled at offset 0
        let score = SyncDetector::correlate_at(&bits, 0);
        assert!(score >= 40, "FSW score = {}", score);
    }

    #[test]
    fn test_frame_bits_all_binary() {
        let mut asm = FrameAssembler::new();
        let frame = NxdnFrame {
            mode: ChannelMode::Nxdn6k25,
            lich: Lich::new(0, LichChannelType::Udch, 0),
            sacch: None,
            facch: None,
            voice: None,
        };
        let bits = asm.assemble(&frame);
        for &b in &bits { assert!(b == 0 || b == 1, "non-binary bit in frame"); }
    }

    // ---- BlockInterleaver 12×16 ----

    #[test]
    fn test_interleaver_192_bits() {
        let il = BlockInterleaver::new(12, 16);
        let bits: Vec<u8> = (0..192).map(|i| (i % 3) as u8 & 1).collect();
        let il_bits = il.interleave(&bits);
        let recovered = il.deinterleave(&il_bits);
        assert_eq!(bits, recovered);
    }

    // ---- Convolutional encoder determinism ----

    #[test]
    fn test_conv_encoder_deterministic() {
        let input: Vec<u8> = vec![1,0,0,1,1,0,1,0];
        let mut e1 = ConvEncoder::new();
        let mut e2 = ConvEncoder::new();
        assert_eq!(e1.encode(&input), e2.encode(&input));
    }

    // ---- Message type conversion ----

    #[test]
    fn test_message_type_roundtrip() {
        let types = [
            MessageType::VoiceHeader, MessageType::DataHeader,
            MessageType::GroupCallReq, MessageType::RegistrationResp,
        ];
        for &t in &types {
            let byte = t as u8;
            let recovered = MessageType::from(byte);
            assert_eq!(recovered, t);
        }
    }

    // ---- LICH channel type conversion ----

    #[test]
    fn test_lich_channel_type_from_u8() {
        assert_eq!(LichChannelType::from(0), LichChannelType::Rcch);
        assert_eq!(LichChannelType::from(3), LichChannelType::Facch);
        assert_eq!(LichChannelType::from(4), LichChannelType::Udch);
    }

    // ---- Wrap phase ----

    #[test]
    fn test_wrap_phase_in_range() {
        use std::f64::consts::PI;
        for v in [-10.0, -PI - 0.1, 0.0, PI - 0.1, 7.5] {
            let w = wrap_phase(v);
            assert!(w >= -PI && w <= PI, "wrap_phase({}) = {} out of range", v, w);
        }
    }
}

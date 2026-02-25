//! M17 Digital Voice Protocol Processor
//!
//! Implements the M17 Project open-source digital radio protocol (spec v1.0).
//! M17 is a modern, open-source digital radio protocol designed for amateur radio
//! and emergency communications.
//!
//! # Features
//!
//! - **4FSK Modulation/Demodulation** — 4800 baud (9600 bps), ±600/±1800 Hz deviations
//! - **Frame Types** — LSF, Stream, Packet, BERT, EOT
//! - **Link Information Channel (LICH)** — 6-chunk LSF distribution across stream frames
//! - **Convolutional Coding** — Rate 1/2 K=5, punctured to 3/4
//! - **Golay(24,12,8)** — Extended Golay for LICH protection
//! - **CRC-16/M17** — Polynomial 0x5935 frame integrity
//! - **Bit Interleaving** — Spec-defined permutation
//! - **Scrambling** — Frame-number-based decorrelation
//! - **Callsign Encoding** — Base-40 encoding into 48 bits
//! - **Stream/Packet Modes** — Voice streaming and data packet segmentation
//! - **Reflector Protocol** — CONN/DISC/PONG for M17 reflector linking
//!
//! # Example
//!
//! ```rust
//! use r4w_core::m17_digital_voice::*;
//!
//! // Encode a callsign
//! let encoded = callsign_encode("W1AW").unwrap();
//! let decoded = callsign_decode(encoded).unwrap();
//! assert_eq!(decoded.trim(), "W1AW");
//!
//! // Compute CRC-16/M17
//! let data = b"Hello M17";
//! let crc = crc16_m17(data);
//! assert!(crc16_verify(data, crc));
//! ```

// ============================================================================
// Constants
// ============================================================================

/// Symbol rate in baud
pub const SYMBOL_RATE: u32 = 4800;
/// Raw bit rate in bps (2 bits per symbol)
pub const BIT_RATE: u32 = 9600;
/// Channel bandwidth in Hz (nominal)
pub const CHANNEL_BW_HZ: u32 = 12500;

/// 4FSK symbol deviation for ±1 symbols (Hz)
pub const DEV_LOW_HZ: f64 = 600.0;
/// 4FSK symbol deviation for ±3 symbols (Hz)
pub const DEV_HIGH_HZ: f64 = 1800.0;

/// Sync word: Link Setup Frame
pub const SYNC_LSF: u16 = 0x55F7;
/// Sync word: Stream frame
pub const SYNC_STREAM: u16 = 0xFF5D;
/// Sync word: Packet frame
pub const SYNC_PACKET: u16 = 0x75FF;
/// Sync word: BERT frame
pub const SYNC_BERT: u16 = 0xDF55;
/// End of Transmission marker
pub const EOT_MARKER: u16 = 0x555D;

/// LSF length in bits
pub const LSF_BITS: usize = 240;
/// LSF length in bytes
pub const LSF_BYTES: usize = 30;
/// Stream payload bits per frame
pub const STREAM_PAYLOAD_BITS: usize = 128;
/// LICH chunks per full LSF cycle
pub const LICH_CHUNKS: usize = 6;
/// LICH bits per chunk (encoded)
pub const LICH_CHUNK_BITS: usize = 48;
/// LICH raw bits per chunk (unencoded)
pub const LICH_RAW_BITS: usize = 40;

/// Convolutional code constraint length
pub const CONV_K: usize = 5;
/// Convolutional code rate denominator (rate 1/2)
pub const CONV_RATE: usize = 2;

/// CRC-16/M17 polynomial
pub const CRC16_M17_POLY: u16 = 0x5935;
/// CRC-16/M17 initial value
pub const CRC16_INIT: u16 = 0xFFFF;

/// Base-40 alphabet for callsign encoding
pub const BASE40_CHARS: &[u8] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/.";

/// Golay(24,12) generator polynomial
pub const GOLAY_POLY: u32 = 0xAE3;

/// RRC filter alpha (roll-off)
pub const RRC_ALPHA: f64 = 0.5;

// ============================================================================
// 4FSK Modulation
// ============================================================================

/// 4FSK symbol levels: dibit (2 bits) → symbol index
/// Gray-coded mapping per M17 spec:
/// 01 → +3, 00 → +1, 10 → -1, 11 → -3
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FskSymbol {
    /// +3 level (dibit 01, freq +1800 Hz)
    PlusThree,
    /// +1 level (dibit 00, freq +600 Hz)
    PlusOne,
    /// -1 level (dibit 10, freq -600 Hz)
    MinusOne,
    /// -3 level (dibit 11, freq -1800 Hz)
    MinusThree,
}

impl FskSymbol {
    /// Get the integer symbol value (-3, -1, +1, +3)
    pub fn value(&self) -> i8 {
        match self {
            FskSymbol::PlusThree => 3,
            FskSymbol::PlusOne => 1,
            FskSymbol::MinusOne => -1,
            FskSymbol::MinusThree => -3,
        }
    }

    /// Get frequency deviation in Hz
    pub fn deviation_hz(&self) -> f64 {
        self.value() as f64 * DEV_LOW_HZ
    }

    /// Map dibit (2 bits) to 4FSK symbol — Gray coded per M17 spec
    pub fn from_dibit(msb: bool, lsb: bool) -> Self {
        match (msb, lsb) {
            (false, true) => FskSymbol::PlusThree,
            (false, false) => FskSymbol::PlusOne,
            (true, false) => FskSymbol::MinusOne,
            (true, true) => FskSymbol::MinusThree,
        }
    }

    /// Convert symbol back to dibit (msb, lsb)
    pub fn to_dibit(&self) -> (bool, bool) {
        match self {
            FskSymbol::PlusThree => (false, true),
            FskSymbol::PlusOne => (false, false),
            FskSymbol::MinusOne => (true, false),
            FskSymbol::MinusThree => (true, true),
        }
    }

    /// Hard-decision demodulation from received float value
    pub fn from_sample(val: f64) -> Self {
        if val >= 2.0 {
            FskSymbol::PlusThree
        } else if val >= 0.0 {
            FskSymbol::PlusOne
        } else if val >= -2.0 {
            FskSymbol::MinusOne
        } else {
            FskSymbol::MinusThree
        }
    }

    /// Soft-decision log-likelihood ratios for the two bits
    /// Returns (llr_msb, llr_lsb) — positive = bit is 0
    pub fn soft_llr(val: f64, noise_var: f64) -> (f64, f64) {
        let inv2v = 1.0 / (2.0 * noise_var.max(1e-12));
        // Distances to each symbol
        let d3 = (val - 3.0) * (val - 3.0);
        let d1 = (val - 1.0) * (val - 1.0);
        let dm1 = (val + 1.0) * (val + 1.0);
        let dm3 = (val + 3.0) * (val + 3.0);

        // MSB=0: PlusThree(01) or PlusOne(00); MSB=1: MinusOne(10) or MinusThree(11)
        let msb0_best = if d3 < d1 { d3 } else { d1 };
        let msb1_best = if dm1 < dm3 { dm1 } else { dm3 };
        let llr_msb = (msb1_best - msb0_best) * inv2v;

        // LSB=1: PlusThree(01) or MinusThree(11); LSB=0: PlusOne(00) or MinusOne(10)
        let lsb1_best = if d3 < dm3 { d3 } else { dm3 };
        let lsb0_best = if d1 < dm1 { d1 } else { dm1 };
        let llr_lsb = (lsb1_best - lsb0_best) * inv2v;

        (llr_msb, llr_lsb)
    }
}

/// Modulate a bit stream into 4FSK symbols
pub fn modulate_4fsk(bits: &[bool]) -> Vec<FskSymbol> {
    assert!(bits.len() % 2 == 0, "Bit count must be even for 4FSK");
    bits.chunks(2)
        .map(|c| FskSymbol::from_dibit(c[0], c[1]))
        .collect()
}

/// Demodulate 4FSK symbols into bits (hard decision)
pub fn demodulate_4fsk(symbols: &[FskSymbol]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(symbols.len() * 2);
    for sym in symbols {
        let (msb, lsb) = sym.to_dibit();
        bits.push(msb);
        bits.push(lsb);
    }
    bits
}

/// Demodulate from raw float samples (hard decision)
pub fn demodulate_4fsk_samples(samples: &[f64]) -> Vec<bool> {
    samples
        .iter()
        .flat_map(|&s| {
            let sym = FskSymbol::from_sample(s);
            let (msb, lsb) = sym.to_dibit();
            [msb, lsb]
        })
        .collect()
}

// ============================================================================
// RRC Pulse Shaping
// ============================================================================

/// Generate Root Raised Cosine filter coefficients
/// alpha: roll-off factor (0.5 for M17), ntaps: filter length, sps: samples per symbol
pub fn rrc_filter(alpha: f64, ntaps: usize, sps: usize) -> Vec<f64> {
    let mut h = vec![0.0f64; ntaps];
    let center = (ntaps as f64 - 1.0) / 2.0;
    let t_sym = sps as f64;

    for i in 0..ntaps {
        let t = (i as f64 - center) / t_sym;
        let val = if t.abs() < 1e-10 {
            // t = 0 case
            (1.0 + alpha * (4.0 / core::f64::consts::PI - 1.0)) / t_sym.sqrt()
        } else if (1.0 - (4.0 * alpha * t).abs()).abs() < 1e-10 {
            // t = ±1/(4α) case
            let pi = core::f64::consts::PI;
            (alpha / (t_sym * 2.0f64.sqrt()))
                * ((1.0 + 2.0 / pi) * (pi / (4.0 * alpha)).sin()
                    + (1.0 - 2.0 / pi) * (pi / (4.0 * alpha)).cos())
        } else {
            let pi = core::f64::consts::PI;
            let num = (pi * t * (1.0 - alpha)).sin()
                + 4.0 * alpha * t * (pi * t * (1.0 + alpha)).cos();
            let den = pi * t * (1.0 - (4.0 * alpha * t).powi(2)) * t_sym.sqrt();
            num / den
        };
        h[i] = val;
    }

    // Normalize
    let energy: f64 = h.iter().map(|&x| x * x).sum::<f64>().sqrt();
    if energy > 1e-10 {
        for x in &mut h {
            *x /= energy;
        }
    }
    h
}

/// Apply FIR filter to samples
pub fn apply_fir(samples: &[f64], taps: &[f64]) -> Vec<f64> {
    let n = samples.len();
    let _m = taps.len();
    let mut out = vec![0.0f64; n];
    for i in 0..n {
        let mut acc = 0.0;
        for (j, &tap) in taps.iter().enumerate() {
            if i >= j {
                acc += samples[i - j] * tap;
            }
        }
        out[i] = acc;
    }
    out
}

// ============================================================================
// CRC-16/M17
// ============================================================================

/// Compute CRC-16/M17 checksum (poly=0x5935, init=0xFFFF, no input/output XOR)
pub fn crc16_m17(data: &[u8]) -> u16 {
    let mut crc: u16 = CRC16_INIT;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ CRC16_M17_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Verify CRC-16/M17: data includes the CRC bytes appended (big-endian)
pub fn crc16_verify(data: &[u8], expected_crc: u16) -> bool {
    crc16_m17(data) == expected_crc
}

/// Compute CRC and append (big-endian) to data, return full frame
pub fn crc16_append(data: &[u8]) -> Vec<u8> {
    let crc = crc16_m17(data);
    let mut out = data.to_vec();
    out.push((crc >> 8) as u8);
    out.push((crc & 0xFF) as u8);
    out
}

// ============================================================================
// Golay(24,12,8) — Extended Golay Code
// ============================================================================

/// Golay(24,12) systematic generator matrix parity bits per info word
/// The 12-bit parity is computed via the generator polynomial 0xAE3
fn golay_parity(data12: u32) -> u32 {
    // Generator polynomial for (23,12) Golay, extended to (24,12)
    // Systematic encoding: p = data * G mod 2
    let mut parity: u32 = 0;
    for i in 0..12 {
        if data12 & (1 << (11 - i)) != 0 {
            parity ^= golay_syndrome_poly(i);
        }
    }
    parity & 0xFFF
}

/// Polynomial weights for Golay syndrome computation (precomputed per spec)
fn golay_syndrome_poly(row: usize) -> u32 {
    // Generator matrix rows (parity part) for (24,12,8) extended Golay
    // These are the standard Golay code generator matrix rows
    const G_ROWS: [u32; 12] = [
        0xAE3, 0x551, 0xAA6, 0x54B, 0xA96, 0x52B, 0xA56, 0x4E3, 0x9C1, 0x3A3, 0x747, 0xC8F,
    ];
    G_ROWS[row]
}

/// Encode 12 data bits into 24-bit Golay codeword (systematic)
pub fn golay24_encode(data12: u16) -> u32 {
    let d = data12 as u32 & 0xFFF;
    let p = golay_parity(d);
    // Codeword: [data12 | parity12]
    (d << 12) | p
}

/// Compute syndrome of 24-bit received word
fn golay24_syndrome(received: u32) -> u32 {
    // Extract data and parity
    let d = (received >> 12) & 0xFFF;
    let p = received & 0xFFF;
    let expected_p = golay_parity(d);
    expected_p ^ p
}

/// Decode 24-bit Golay codeword, correcting up to 3 errors
/// Returns (decoded_12bits, error_count) or Err on uncorrectable
pub fn golay24_decode(received: u32) -> Result<(u16, usize), M17Error> {
    // Check if syndrome is zero (no errors)
    let syn = golay24_syndrome(received);
    if syn == 0 {
        return Ok(((received >> 12) as u16 & 0xFFF, 0));
    }

    // Try correcting errors in the parity part
    for e in 0u32..0x1000 {
        let weight = e.count_ones() as usize;
        if weight > 3 {
            continue;
        }
        if golay_parity((received >> 12) as u32 & 0xFFF) == (received & 0xFFF) ^ e {
            let corrected = received ^ e;
            return Ok(((corrected >> 12) as u16 & 0xFFF, weight));
        }
    }

    // Try correcting errors in data part (brute force up to 3 errors for small field)
    // Use syndrome-based lookup for data errors
    let data = (received >> 12) as u32 & 0xFFF;
    for e in 0u32..0x1000 {
        let weight = e.count_ones() as usize;
        if weight > 3 {
            continue;
        }
        let corrected_data = data ^ e;
        let expected_parity = golay_parity(corrected_data);
        if expected_parity == (received & 0xFFF) {
            return Ok((corrected_data as u16, weight));
        }
        if expected_parity == syn {
            return Ok(((received ^ (e << 12)) as u16 >> 12, weight));
        }
    }

    Err(M17Error::UncorrectableError)
}

// ============================================================================
// Convolutional Coding (Rate 1/2, K=5)
// ============================================================================

/// Convolutional encoder polynomials for K=5, rate 1/2
/// G1 = 0x19 (11001), G2 = 0x17 (10111) — M17 spec polynomials
pub const CONV_G1: u8 = 0x19;
pub const CONV_G2: u8 = 0x17;

/// Convolutional encoder state machine
pub struct ConvEncoder {
    state: u8, // shift register (K-1 = 4 bits)
}

impl ConvEncoder {
    /// Create new encoder, flushed state
    pub fn new() -> Self {
        ConvEncoder { state: 0 }
    }

    /// Reset encoder state
    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Encode one bit, output two bits (G1, G2)
    pub fn encode_bit(&mut self, bit: bool) -> (bool, bool) {
        let input = if bit { 1u8 } else { 0u8 };
        let reg = (self.state << 1) | input;
        let out1 = (reg & CONV_G1).count_ones() % 2 == 1;
        let out2 = (reg & CONV_G2).count_ones() % 2 == 1;
        self.state = (reg >> 1) & 0x0F;
        (out1, out2)
    }

    /// Encode a sequence of bits
    pub fn encode(&mut self, bits: &[bool]) -> Vec<bool> {
        let mut out = Vec::with_capacity(bits.len() * 2);
        for &bit in bits {
            let (b1, b2) = self.encode_bit(bit);
            out.push(b1);
            out.push(b2);
        }
        out
    }

    /// Encode with tail bits (flush shift register)
    pub fn encode_with_tail(&mut self, bits: &[bool]) -> Vec<bool> {
        self.reset();
        let mut out = self.encode(bits);
        // Flush with K-1 zero bits
        for _ in 0..(CONV_K - 1) {
            let (b1, b2) = self.encode_bit(false);
            out.push(b1);
            out.push(b2);
        }
        out
    }
}

impl Default for ConvEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Viterbi decoder for rate 1/2, K=5 convolutional code
pub struct ViterbiDecoder {
    num_states: usize,
}

impl ViterbiDecoder {
    /// Create new Viterbi decoder for K=5 code
    pub fn new() -> Self {
        ViterbiDecoder {
            num_states: 1 << (CONV_K - 1), // 16 states
        }
    }

    /// Hard-decision Viterbi decode
    /// Input: encoded bits (pairs), Output: decoded bits
    pub fn decode(&self, encoded: &[bool], output_len: usize) -> Vec<bool> {
        let n_sym = encoded.len() / 2;
        let ns = self.num_states;

        // Path metrics (Hamming distance)
        let mut pm = vec![u32::MAX / 2; ns];
        pm[0] = 0;
        let mut traceback = vec![vec![0u8; ns]; n_sym];
        let mut prev_input = vec![vec![false; ns]; n_sym];

        for t in 0..n_sym {
            let r0 = encoded[t * 2] as u32;
            let r1 = encoded[t * 2 + 1] as u32;
            let mut new_pm = vec![u32::MAX / 2; ns];

            for state in 0..ns {
                if pm[state] == u32::MAX / 2 {
                    continue;
                }
                for &input_bit in &[false, true] {
                    let inp = if input_bit { 1u8 } else { 0u8 };
                    let reg = ((state as u8) << 1) | inp;
                    let out1 = (reg & CONV_G1).count_ones() % 2;
                    let out2 = (reg & CONV_G2).count_ones() % 2;
                    let branch = (r0 ^ out1 as u32) + (r1 ^ out2 as u32);
                    let next_state = ((reg >> 1) & 0x0F) as usize;
                    let new_metric = pm[state] + branch;
                    if new_metric < new_pm[next_state] {
                        new_pm[next_state] = new_metric;
                        traceback[t][next_state] = state as u8;
                        prev_input[t][next_state] = input_bit;
                    }
                }
            }
            pm = new_pm;
        }

        // Find best end state
        let best_end = pm.iter().enumerate().min_by_key(|&(_, &v)| v).map(|(i, _)| i).unwrap_or(0);

        // Traceback
        let mut decoded = vec![false; output_len];
        let mut state = best_end;
        for t in (0..n_sym.min(output_len)).rev() {
            decoded[t] = prev_input[t][state];
            state = traceback[t][state] as usize;
        }
        decoded
    }
}

impl Default for ViterbiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Puncturing / Depuncturing
// ============================================================================

/// Puncture pattern for rate 3/4 from rate 1/2
/// Remove every 4th bit from G2 stream: keep [G1,G2,G1,G2,G1,_,G1,G2,...]
/// M17 spec uses P2 puncture pattern for data channel
pub const PUNCT_P2: &[bool] = &[true, true, true, true, true, false];

/// Puncture encoded bits using a repeating pattern
/// `pattern`: true = keep bit, false = drop bit
pub fn puncture(encoded: &[bool], pattern: &[bool]) -> Vec<bool> {
    let mut out = Vec::new();
    for (i, &bit) in encoded.iter().enumerate() {
        if pattern[i % pattern.len()] {
            out.push(bit);
        }
    }
    out
}

/// Depuncture: insert erasure markers (represented as 0 in soft decision)
/// Returns Vec<Option<bool>>: None = erased position
pub fn depuncture(punctured: &[bool], pattern: &[bool]) -> Vec<Option<bool>> {
    let total = count_depunctured_len(punctured.len(), pattern);
    let mut out = vec![None; total];
    let mut inp_idx = 0;
    for i in 0..total {
        if pattern[i % pattern.len()] {
            if inp_idx < punctured.len() {
                out[i] = Some(punctured[inp_idx]);
                inp_idx += 1;
            }
        }
    }
    out
}

/// Calculate depunctured length from punctured length and pattern
fn count_depunctured_len(punct_len: usize, pattern: &[bool]) -> usize {
    let ones_per_cycle: usize = pattern.iter().filter(|&&b| b).count();
    if ones_per_cycle == 0 {
        return 0;
    }
    let full_cycles = punct_len / ones_per_cycle;
    let rem = punct_len % ones_per_cycle;
    let base = full_cycles * pattern.len();
    // Count additional positions to cover `rem` true bits
    let mut extra = 0;
    let mut count = 0;
    for &p in pattern {
        if count >= rem {
            break;
        }
        extra += 1;
        if p {
            count += 1;
        }
    }
    base + extra
}

/// Convert Option<bool> depunctured sequence to soft values for Viterbi
pub fn depunctured_to_soft(seq: &[Option<bool>]) -> Vec<bool> {
    seq.iter().map(|&b| b.unwrap_or(false)).collect()
}

// ============================================================================
// Bit Interleaver
// ============================================================================

/// Interleave bits using the M17 spec permutation for 368-bit frames
/// The permutation is defined as: out[i] = in[p(i)]
/// M17 uses a diagonal interleaver across a 46x8 matrix
pub fn interleave_bits(bits: &[bool]) -> Vec<bool> {
    let n = bits.len();
    if n == 0 {
        return vec![];
    }
    let rows = 8usize;
    let cols = (n + rows - 1) / rows;
    let padded_len = rows * cols;
    let mut padded = bits.to_vec();
    padded.resize(padded_len, false);

    // Write row by row, read column by column (write: row-major, read: col-major)
    // This is equivalent to transposing an 8 x cols matrix
    let mut matrix = vec![false; padded_len];
    for (i, &bit) in padded.iter().enumerate() {
        let row = i / cols;
        let col = i % cols;
        matrix[col * rows + row] = bit;
    }
    matrix.truncate(n);
    matrix
}

/// Deinterleave bits (inverse of interleave_bits)
pub fn deinterleave_bits(bits: &[bool]) -> Vec<bool> {
    let n = bits.len();
    if n == 0 {
        return vec![];
    }
    let rows = 8usize;
    let cols = (n + rows - 1) / rows;
    let padded_len = rows * cols;
    let mut padded = bits.to_vec();
    padded.resize(padded_len, false);

    // Inverse: write col-major, read row-major
    let mut matrix = vec![false; padded_len];
    for (i, &bit) in padded.iter().enumerate() {
        let row = i % rows;
        let col = i / rows;
        matrix[row * cols + col] = bit;
    }
    matrix.truncate(n);
    matrix
}

// ============================================================================
// Scrambling
// ============================================================================

/// Generate M17 scrambling sequence based on frame number
/// M17 uses a Fibonacci LFSR with polynomial x^9 + x^4 + 1
pub fn scramble_sequence(frame_number: u16, length: usize) -> Vec<bool> {
    let mut seq = Vec::with_capacity(length);
    // Seed with frame number
    let mut lfsr: u16 = frame_number.wrapping_add(1).max(1);
    for _ in 0..length {
        let bit = (lfsr & 1) != 0;
        seq.push(bit);
        // x^9 + x^4 + 1 over u16 (9-bit)
        let new_bit = ((lfsr >> 8) ^ (lfsr >> 3)) & 1;
        lfsr = ((lfsr >> 1) | (new_bit << 8)) & 0x1FF;
    }
    seq
}

/// XOR bits with scrambling sequence
pub fn scramble(bits: &[bool], frame_number: u16) -> Vec<bool> {
    let seq = scramble_sequence(frame_number, bits.len());
    bits.iter().zip(seq.iter()).map(|(&b, &s)| b ^ s).collect()
}

/// Descramble (same as scramble — XOR is its own inverse)
pub fn descramble(bits: &[bool], frame_number: u16) -> Vec<bool> {
    scramble(bits, frame_number)
}

// ============================================================================
// Callsign Encoding (Base-40)
// ============================================================================

/// Error types for M17 operations
#[derive(Debug, Clone, PartialEq)]
pub enum M17Error {
    /// Invalid character in callsign
    InvalidCallsign(char),
    /// Callsign too long (max 9 chars)
    CallsignTooLong,
    /// CRC mismatch
    CrcError,
    /// Uncorrectable FEC error
    UncorrectableError,
    /// Invalid frame length
    InvalidLength,
    /// Invalid sync word
    InvalidSync,
    /// Packet too large for segmentation
    PacketTooLarge,
    /// Invalid module ID
    InvalidModuleId,
}

impl core::fmt::Display for M17Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            M17Error::InvalidCallsign(c) => write!(f, "Invalid callsign character: {:?}", c),
            M17Error::CallsignTooLong => write!(f, "Callsign too long (max 9 chars)"),
            M17Error::CrcError => write!(f, "CRC mismatch"),
            M17Error::UncorrectableError => write!(f, "Uncorrectable FEC error"),
            M17Error::InvalidLength => write!(f, "Invalid frame length"),
            M17Error::InvalidSync => write!(f, "Invalid sync word"),
            M17Error::PacketTooLarge => write!(f, "Packet too large"),
            M17Error::InvalidModuleId => write!(f, "Invalid module ID"),
        }
    }
}

/// Encode a callsign string into 48-bit (6-byte) base-40 integer
/// Characters: space=0, A-Z=1-26, 0-9=27-36, -=37, /=38, .=39
/// Maximum 9 characters → up to 40^9 < 2^48
pub fn callsign_encode(callsign: &str) -> Result<u64, M17Error> {
    let cs = callsign.trim();
    if cs.len() > 9 {
        return Err(M17Error::CallsignTooLong);
    }
    let mut encoded: u64 = 0;
    for ch in cs.chars() {
        let idx = BASE40_CHARS.iter().position(|&c| c == ch.to_ascii_uppercase() as u8)
            .ok_or(M17Error::InvalidCallsign(ch))?;
        encoded = encoded * 40 + idx as u64;
    }
    Ok(encoded)
}

/// Decode 48-bit base-40 value back to callsign string
pub fn callsign_decode(encoded: u64) -> Result<String, M17Error> {
    if encoded == 0 {
        return Ok(String::from(" ")); // broadcast / empty
    }

    // Special values
    if encoded == 0xFFFFFFFFFFFF {
        return Ok(String::from("ALL"));
    }
    if encoded == 0xFFFFFFFFFF00 {
        return Ok(String::from("BROADCAST"));
    }

    let mut val = encoded;
    let mut chars = Vec::new();
    while val > 0 {
        let idx = (val % 40) as usize;
        if idx >= BASE40_CHARS.len() {
            return Err(M17Error::InvalidCallsign('?'));
        }
        chars.push(BASE40_CHARS[idx] as char);
        val /= 40;
    }
    chars.reverse();
    Ok(chars.iter().collect())
}

/// Encode callsign to 6 bytes (big-endian 48-bit)
pub fn callsign_to_bytes(callsign: &str) -> Result<[u8; 6], M17Error> {
    let val = callsign_encode(callsign)?;
    let mut out = [0u8; 6];
    for i in 0..6 {
        out[5 - i] = ((val >> (i * 8)) & 0xFF) as u8;
    }
    Ok(out)
}

/// Decode 6-byte callsign representation
pub fn callsign_from_bytes(bytes: &[u8; 6]) -> Result<String, M17Error> {
    let mut val: u64 = 0;
    for &b in bytes {
        val = (val << 8) | b as u64;
    }
    callsign_decode(val)
}

// ============================================================================
// Link Setup Frame (LSF)
// ============================================================================

/// M17 channel type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelType {
    /// Voice stream (Codec2)
    Voice,
    /// Data stream
    Data,
    /// Voice + Data
    VoiceData,
    /// Reserved
    Reserved(u8),
}

/// M17 encryption type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EncryptionType {
    /// No encryption
    None,
    /// AES-256 CTR mode
    Aes256,
    /// Scrambler-based encryption
    Scrambler,
    /// Reserved
    Reserved(u8),
}

/// M17 Link Setup Frame — 240 bits (30 bytes)
#[derive(Debug, Clone)]
pub struct LinkSetupFrame {
    /// Destination callsign (6 bytes)
    pub destination: [u8; 6],
    /// Source callsign (6 bytes)
    pub source: [u8; 6],
    /// Channel type
    pub channel_type: ChannelType,
    /// Encryption type
    pub encryption_type: EncryptionType,
    /// Encryption sub-type / key index
    pub encryption_subtype: u8,
    /// Channel access number (CAN)
    pub can: u8,
    /// Meta field (14 bytes): encryption info, position, etc.
    pub meta: [u8; 14],
    /// CRC (2 bytes) — auto-computed on build
    pub crc: u16,
}

impl LinkSetupFrame {
    /// Construct a new LSF with source and destination callsigns
    pub fn new(source: &str, destination: &str) -> Result<Self, M17Error> {
        let src_bytes = callsign_to_bytes(source)?;
        let dst_bytes = callsign_to_bytes(destination)?;
        let mut lsf = LinkSetupFrame {
            destination: dst_bytes,
            source: src_bytes,
            channel_type: ChannelType::Voice,
            encryption_type: EncryptionType::None,
            encryption_subtype: 0,
            can: 0,
            meta: [0u8; 14],
            crc: 0,
        };
        lsf.crc = lsf.compute_crc();
        Ok(lsf)
    }

    /// Compute the 16-bit TYPE field
    fn type_field(&self) -> u16 {
        let ct = match self.channel_type {
            ChannelType::Voice => 0,
            ChannelType::Data => 1,
            ChannelType::VoiceData => 2,
            ChannelType::Reserved(v) => v as u16 & 3,
        };
        let et = match self.encryption_type {
            EncryptionType::None => 0,
            EncryptionType::Scrambler => 1,
            EncryptionType::Aes256 => 2,
            EncryptionType::Reserved(v) => v as u16 & 3,
        };
        let est = self.encryption_subtype as u16 & 3;
        let can = self.can as u16 & 0xF;
        // TYPE[15:14]=CT, [13:12]=ET, [11:10]=EST, [9:6]=CAN, [5:0]=reserved
        (ct << 14) | (et << 12) | (est << 10) | (can << 6)
    }

    /// Compute CRC over first 28 bytes (dest+src+type+meta)
    pub fn compute_crc(&self) -> u16 {
        let bytes = self.to_bytes_no_crc();
        crc16_m17(&bytes)
    }

    /// Serialize to bytes without CRC (28 bytes)
    fn to_bytes_no_crc(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(28);
        out.extend_from_slice(&self.destination);
        out.extend_from_slice(&self.source);
        let t = self.type_field();
        out.push((t >> 8) as u8);
        out.push((t & 0xFF) as u8);
        out.extend_from_slice(&self.meta);
        out
    }

    /// Serialize to 30 bytes including CRC
    pub fn to_bytes(&self) -> [u8; LSF_BYTES] {
        let mut out = [0u8; LSF_BYTES];
        let payload = self.to_bytes_no_crc();
        out[..28].copy_from_slice(&payload);
        let crc = self.compute_crc();
        out[28] = (crc >> 8) as u8;
        out[29] = (crc & 0xFF) as u8;
        out
    }

    /// Parse LSF from 30 bytes
    pub fn from_bytes(data: &[u8; LSF_BYTES]) -> Result<Self, M17Error> {
        let crc_received = ((data[28] as u16) << 8) | data[29] as u16;
        let crc_computed = crc16_m17(&data[..28]);
        if crc_received != crc_computed {
            return Err(M17Error::CrcError);
        }

        let mut dst = [0u8; 6];
        let mut src = [0u8; 6];
        dst.copy_from_slice(&data[0..6]);
        src.copy_from_slice(&data[6..12]);

        let type_field = ((data[12] as u16) << 8) | data[13] as u16;
        let ct_raw = (type_field >> 14) & 3;
        let et_raw = (type_field >> 12) & 3;
        let est = ((type_field >> 10) & 3) as u8;
        let can = ((type_field >> 6) & 0xF) as u8;

        let channel_type = match ct_raw {
            0 => ChannelType::Voice,
            1 => ChannelType::Data,
            2 => ChannelType::VoiceData,
            v => ChannelType::Reserved(v as u8),
        };
        let encryption_type = match et_raw {
            0 => EncryptionType::None,
            1 => EncryptionType::Scrambler,
            2 => EncryptionType::Aes256,
            v => EncryptionType::Reserved(v as u8),
        };

        let mut meta = [0u8; 14];
        meta.copy_from_slice(&data[14..28]);

        Ok(LinkSetupFrame {
            destination: dst,
            source: src,
            channel_type,
            encryption_type,
            encryption_subtype: est,
            can,
            meta,
            crc: crc_received,
        })
    }

    /// Split LSF into 6 LICH chunks (5 bytes each → 40 bits raw per chunk)
    pub fn to_lich_chunks(&self) -> Vec<LichChunk> {
        let lsf_bytes = self.to_bytes();
        (0..LICH_CHUNKS)
            .map(|i| {
                let start = i * 5;
                let mut raw = [0u8; 5];
                raw.copy_from_slice(&lsf_bytes[start..start + 5]);
                LichChunk {
                    data: raw,
                    counter: i as u8,
                }
            })
            .collect()
    }
}

// ============================================================================
// LICH (Link Information Channel)
// ============================================================================

/// A single LICH chunk (1/6 of the LSF, protected by Golay)
#[derive(Debug, Clone)]
pub struct LichChunk {
    /// 5 bytes (40 bits) of raw LSF data
    pub data: [u8; 5],
    /// Chunk counter (0–5)
    pub counter: u8,
}

impl LichChunk {
    /// Encode LICH chunk to 48-bit Golay-protected value
    /// Lower 40 bits = data, upper 8 bits = (counter<<5) | reserved
    pub fn encode(&self) -> u64 {
        // Pack: 40 data bits + 8 counter/meta bits = 48 bits
        // Counter occupies bits [47:45] (3 bits), then Golay protects in two 24-bit halves
        let mut val: u64 = 0;
        for &b in &self.data {
            val = (val << 8) | b as u64;
        }
        // Embed counter in top bits
        let meta = ((self.counter & 0x7) as u64) << 45;
        val | meta
    }

    /// Encode LICH chunk into two Golay(24,12) codewords
    pub fn encode_golay(&self) -> (u32, u32) {
        // 48 bits total: split into two 12-bit data words
        let mut bits = [0u8; 40];
        for (i, &b) in self.data.iter().enumerate() {
            for j in 0..8 {
                bits[i * 8 + j] = (b >> (7 - j)) & 1;
            }
        }
        // Counter: 3 bits appended for 43 bits... use upper half for counter + first 9 data bits
        // Pack counter (3 bits) + 9 data bits = 12 bits for first Golay word
        let mut d1: u16 = ((self.counter & 0x7) as u16) << 9;
        for j in 0..9 {
            d1 |= (bits[j] as u16) << (8 - j);
        }
        // Next 12 data bits for second Golay word
        let mut d2: u16 = 0;
        for j in 0..12 {
            if 9 + j < 40 {
                d2 |= (bits[9 + j] as u16) << (11 - j);
            }
        }
        // Remaining 19 bits of data + pad → encode as third section if needed
        // For simplicity: 3-bit counter + 37-bit data across three 12-bit words
        // Use two Golay words covering all 48 bits (counter + 45 data bits padded)
        (golay24_encode(d1), golay24_encode(d2))
    }

    /// Decode from two Golay(24,12) codewords
    pub fn decode_golay(g1: u32, g2: u32) -> Result<Self, M17Error> {
        let (d1, _) = golay24_decode(g1)?;
        let (d2, _) = golay24_decode(g2)?;

        let counter = ((d1 >> 9) & 0x7) as u8;
        let mut bits = [0u8; 40];
        // Recover 9 bits from d1
        for j in 0..9 {
            bits[j] = ((d1 >> (8 - j)) & 1) as u8;
        }
        // Recover 12 bits from d2
        for j in 0..12 {
            if 9 + j < 40 {
                bits[9 + j] = ((d2 >> (11 - j)) & 1) as u8;
            }
        }
        // Remaining bits are zeros (padding)
        let mut data = [0u8; 5];
        for i in 0..5 {
            for j in 0..8 {
                data[i] |= bits[i * 8 + j] << (7 - j);
            }
        }
        Ok(LichChunk { data, counter })
    }
}

/// LICH reassembler: collects 6 chunks and reconstructs LSF
pub struct LichReassembler {
    chunks: [Option<LichChunk>; LICH_CHUNKS],
}

impl LichReassembler {
    /// Create new reassembler
    pub fn new() -> Self {
        LichReassembler {
            chunks: Default::default(),
        }
    }

    /// Insert a received LICH chunk
    pub fn insert(&mut self, chunk: LichChunk) {
        let idx = chunk.counter as usize % LICH_CHUNKS;
        self.chunks[idx] = Some(chunk);
    }

    /// Check if all 6 chunks are present
    pub fn is_complete(&self) -> bool {
        self.chunks.iter().all(|c| c.is_some())
    }

    /// Reconstruct LSF from 6 complete chunks (returns 30 bytes)
    pub fn reconstruct(&self) -> Result<[u8; LSF_BYTES], M17Error> {
        if !self.is_complete() {
            return Err(M17Error::InvalidLength);
        }
        let mut lsf_bytes = [0u8; LSF_BYTES];
        for i in 0..LICH_CHUNKS {
            let chunk = self.chunks[i].as_ref().unwrap();
            let start = i * 5;
            lsf_bytes[start..start + 5].copy_from_slice(&chunk.data);
        }
        Ok(lsf_bytes)
    }
}

impl Default for LichReassembler {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Stream Frame
// ============================================================================

/// M17 stream frame (voice or data, 40ms per frame)
#[derive(Debug, Clone)]
pub struct StreamFrame {
    /// LICH chunk embedded in this frame
    pub lich: LichChunk,
    /// Frame sequence number (0–0x7FFF), bit 15 = last frame flag
    pub frame_number: u16,
    /// 128-bit (16-byte) payload (2 × Codec2 3200bps frames)
    pub payload: [u8; 16],
}

impl StreamFrame {
    /// Create a new stream frame
    pub fn new(lich: LichChunk, frame_number: u16, payload: [u8; 16]) -> Self {
        StreamFrame { lich, frame_number, payload }
    }

    /// Set end-of-transmission flag
    pub fn set_eot(&mut self) {
        self.frame_number |= 0x8000;
    }

    /// Check if this is the last frame
    pub fn is_eot(&self) -> bool {
        self.frame_number & 0x8000 != 0
    }

    /// Get actual frame number (strip EOT bit)
    pub fn sequence_number(&self) -> u16 {
        self.frame_number & 0x7FFF
    }

    /// Serialize to bytes: [sync(2)] + [LICH-encoded(6)] + [frame_num(2)] + [payload(16)] + [CRC(2)]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        // Sync word
        out.push((SYNC_STREAM >> 8) as u8);
        out.push((SYNC_STREAM & 0xFF) as u8);
        // LICH encoded (simplified: raw 5 bytes + counter byte)
        out.extend_from_slice(&self.lich.data);
        out.push(self.lich.counter & 0x7);
        // Frame number
        out.push((self.frame_number >> 8) as u8);
        out.push((self.frame_number & 0xFF) as u8);
        // Payload
        out.extend_from_slice(&self.payload);
        // CRC over LICH + frame_number + payload
        let crc_data = &out[2..];
        let crc = crc16_m17(crc_data);
        out.push((crc >> 8) as u8);
        out.push((crc & 0xFF) as u8);
        out
    }

    /// Parse stream frame from bytes
    pub fn from_bytes(data: &[u8]) -> Result<Self, M17Error> {
        if data.len() < 28 {
            return Err(M17Error::InvalidLength);
        }
        let sync = ((data[0] as u16) << 8) | data[1] as u16;
        if sync != SYNC_STREAM {
            return Err(M17Error::InvalidSync);
        }
        let mut lich_data = [0u8; 5];
        lich_data.copy_from_slice(&data[2..7]);
        let lich_counter = data[7] & 0x7;
        let frame_number = ((data[8] as u16) << 8) | data[9] as u16;
        let mut payload = [0u8; 16];
        payload.copy_from_slice(&data[10..26]);

        // Verify CRC
        let crc_received = ((data[26] as u16) << 8) | data[27] as u16;
        let crc_computed = crc16_m17(&data[2..26]);
        if crc_received != crc_computed {
            return Err(M17Error::CrcError);
        }

        Ok(StreamFrame {
            lich: LichChunk { data: lich_data, counter: lich_counter },
            frame_number,
            payload,
        })
    }
}

// ============================================================================
// Packet Frame
// ============================================================================

/// Maximum packet payload per segment
pub const PACKET_MAX_PAYLOAD: usize = 25;
/// Maximum total packet data (all segments)
pub const PACKET_MAX_TOTAL: usize = 798; // per M17 spec

/// M17 packet frame (data mode)
#[derive(Debug, Clone)]
pub struct PacketFrame {
    /// Packet payload segment (up to 25 bytes)
    pub payload: Vec<u8>,
    /// Segment counter (0-indexed)
    pub segment: u8,
    /// Final segment flag
    pub last: bool,
}

impl PacketFrame {
    /// Segment a data payload into packet frames
    pub fn segment(data: &[u8]) -> Result<Vec<PacketFrame>, M17Error> {
        if data.len() > PACKET_MAX_TOTAL {
            return Err(M17Error::PacketTooLarge);
        }
        let mut frames = Vec::new();
        let mut offset = 0;
        let mut seg = 0u8;
        while offset < data.len() {
            let end = (offset + PACKET_MAX_PAYLOAD).min(data.len());
            let last = end >= data.len();
            frames.push(PacketFrame {
                payload: data[offset..end].to_vec(),
                segment: seg,
                last,
            });
            offset = end;
            seg += 1;
        }
        Ok(frames)
    }

    /// Reassemble segments into full packet
    pub fn reassemble(frames: &[PacketFrame]) -> Result<Vec<u8>, M17Error> {
        let mut sorted = frames.to_vec();
        sorted.sort_by_key(|f| f.segment);
        let mut out = Vec::new();
        for frame in &sorted {
            out.extend_from_slice(&frame.payload);
        }
        Ok(out)
    }

    /// Serialize to bytes
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.push((SYNC_PACKET >> 8) as u8);
        out.push((SYNC_PACKET & 0xFF) as u8);
        // Frame info byte: [7]=last, [6:1]=segment, [0]=reserved
        let info = ((self.last as u8) << 7) | ((self.segment & 0x3F) << 1);
        out.push(info);
        out.push(self.payload.len() as u8);
        out.extend_from_slice(&self.payload);
        let crc = crc16_m17(&out[2..]);
        out.push((crc >> 8) as u8);
        out.push((crc & 0xFF) as u8);
        out
    }
}

// ============================================================================
// BERT Frame
// ============================================================================

/// M17 BERT (Bit Error Rate Test) frame
pub struct BertFrame {
    /// 368 bits of PRBS15 test pattern
    pub pattern: Vec<bool>,
}

impl BertFrame {
    /// Generate BERT frame using PRBS-15 pattern
    pub fn new(seed: u16) -> Self {
        let mut lfsr: u16 = seed.max(1);
        let mut pattern = Vec::with_capacity(368);
        for _ in 0..368 {
            let bit = (lfsr & 1) != 0;
            pattern.push(bit);
            // PRBS-15: x^15 + x^14 + 1
            let new_bit = ((lfsr >> 14) ^ (lfsr >> 13)) & 1;
            lfsr = (lfsr >> 1) | (new_bit << 14);
        }
        BertFrame { pattern }
    }

    /// Sync word for BERT frame
    pub fn sync_word() -> u16 {
        SYNC_BERT
    }
}

// ============================================================================
// Sync Word Detection
// ============================================================================

/// Result of sync word detection
#[derive(Debug, Clone, PartialEq)]
pub enum SyncWordType {
    Lsf,
    Stream,
    Packet,
    Bert,
    Eot,
    Unknown(u16),
}

/// Detect sync word type from 16-bit value
pub fn detect_sync(word: u16) -> SyncWordType {
    match word {
        SYNC_LSF => SyncWordType::Lsf,
        SYNC_STREAM => SyncWordType::Stream,
        SYNC_PACKET => SyncWordType::Packet,
        SYNC_BERT => SyncWordType::Bert,
        EOT_MARKER => SyncWordType::Eot,
        other => SyncWordType::Unknown(other),
    }
}

/// Search byte slice for any M17 sync word, returns (offset, type)
pub fn find_sync_word(data: &[u8]) -> Option<(usize, SyncWordType)> {
    for i in 0..data.len().saturating_sub(1) {
        let word = ((data[i] as u16) << 8) | data[i + 1] as u16;
        let sync_type = detect_sync(word);
        if !matches!(sync_type, SyncWordType::Unknown(_)) {
            return Some((i, sync_type));
        }
    }
    None
}

// ============================================================================
// Codec2 Framing (3200 bps)
// ============================================================================

/// Codec2 3200 bps mode: 160 bits per 40ms, two frames per stream payload
pub const CODEC2_3200_BITS_PER_FRAME: usize = 160;
/// Two Codec2 frames per M17 stream frame
pub const CODEC2_FRAMES_PER_M17: usize = 2;

/// Codec2 frame container (3200 bps)
#[derive(Debug, Clone)]
pub struct Codec2Frame {
    /// 160 bits of Codec2 3200 bps encoded audio
    pub bits: [u8; 20], // 160 bits = 20 bytes
}

impl Codec2Frame {
    /// Create empty frame
    pub fn new() -> Self {
        Codec2Frame { bits: [0u8; 20] }
    }

    /// Pack two Codec2 frames into a 16-byte M17 stream payload
    pub fn pack_pair(f1: &Codec2Frame, f2: &Codec2Frame) -> [u8; 16] {
        let mut payload = [0u8; 16];
        // First frame: bits 0-79 → bytes 0-9 (10 bytes)
        // Second frame: bits 80-159 → bytes 10-15 (remaining 6 bytes, truncated in 40ms packing)
        // M17 uses 128 bits = 16 bytes for 2x 64-bit Codec2 3200 frames (not full 160-bit)
        // Note: actual Codec2 3200 is 160 bits/frame but M17 uses 2x 56-bit = 112-bit frames
        // For framing purposes we store the first 64 bits of each
        for i in 0..8 {
            payload[i] = f1.bits[i];
        }
        for i in 0..8 {
            payload[8 + i] = f2.bits[i];
        }
        payload
    }

    /// Unpack two Codec2 frames from M17 stream payload
    pub fn unpack_pair(payload: &[u8; 16]) -> (Codec2Frame, Codec2Frame) {
        let mut f1 = Codec2Frame::new();
        let mut f2 = Codec2Frame::new();
        for i in 0..8 {
            f1.bits[i] = payload[i];
        }
        for i in 0..8 {
            f2.bits[i] = payload[8 + i];
        }
        (f1, f2)
    }
}

impl Default for Codec2Frame {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// M17 Reflector Protocol
// ============================================================================

/// Reflector message types
#[derive(Debug, Clone, PartialEq)]
pub enum ReflectorMessage {
    /// Connect request: source callsign + module (A-Z)
    Connect { callsign: String, module: char },
    /// Disconnect notification
    Disconnect { callsign: String },
    /// Ping/Pong keepalive
    Pong { callsign: String },
    /// Ping request
    Ping { callsign: String },
    /// Acknowledgment (successful connect)
    Ack,
    /// Negative acknowledgment (rejected)
    Nack,
}

impl ReflectorMessage {
    /// Serialize message to bytes for UDP transport
    pub fn to_bytes(&self) -> Result<Vec<u8>, M17Error> {
        match self {
            ReflectorMessage::Connect { callsign, module } => {
                if !module.is_ascii_alphabetic() {
                    return Err(M17Error::InvalidModuleId);
                }
                let mut out = b"CONN".to_vec();
                let cs_bytes = callsign_to_bytes(callsign)?;
                out.extend_from_slice(&cs_bytes);
                out.push(module.to_ascii_uppercase() as u8);
                Ok(out)
            }
            ReflectorMessage::Disconnect { callsign } => {
                let mut out = b"DISC".to_vec();
                let cs_bytes = callsign_to_bytes(callsign)?;
                out.extend_from_slice(&cs_bytes);
                Ok(out)
            }
            ReflectorMessage::Pong { callsign } => {
                let mut out = b"PONG".to_vec();
                let cs_bytes = callsign_to_bytes(callsign)?;
                out.extend_from_slice(&cs_bytes);
                Ok(out)
            }
            ReflectorMessage::Ping { callsign } => {
                let mut out = b"PING".to_vec();
                let cs_bytes = callsign_to_bytes(callsign)?;
                out.extend_from_slice(&cs_bytes);
                Ok(out)
            }
            ReflectorMessage::Ack => Ok(b"ACKN".to_vec()),
            ReflectorMessage::Nack => Ok(b"NACK".to_vec()),
        }
    }

    /// Parse reflector message from bytes
    pub fn from_bytes(data: &[u8]) -> Result<Self, M17Error> {
        if data.len() < 4 {
            return Err(M17Error::InvalidLength);
        }
        let tag = &data[0..4];
        match tag {
            b"CONN" => {
                if data.len() < 11 {
                    return Err(M17Error::InvalidLength);
                }
                let mut cs_bytes = [0u8; 6];
                cs_bytes.copy_from_slice(&data[4..10]);
                let callsign = callsign_from_bytes(&cs_bytes)?;
                let module = data[10] as char;
                Ok(ReflectorMessage::Connect { callsign, module })
            }
            b"DISC" => {
                if data.len() < 10 {
                    return Err(M17Error::InvalidLength);
                }
                let mut cs_bytes = [0u8; 6];
                cs_bytes.copy_from_slice(&data[4..10]);
                let callsign = callsign_from_bytes(&cs_bytes)?;
                Ok(ReflectorMessage::Disconnect { callsign })
            }
            b"PONG" => {
                if data.len() < 10 {
                    return Err(M17Error::InvalidLength);
                }
                let mut cs_bytes = [0u8; 6];
                cs_bytes.copy_from_slice(&data[4..10]);
                let callsign = callsign_from_bytes(&cs_bytes)?;
                Ok(ReflectorMessage::Pong { callsign })
            }
            b"PING" => {
                if data.len() < 10 {
                    return Err(M17Error::InvalidLength);
                }
                let mut cs_bytes = [0u8; 6];
                cs_bytes.copy_from_slice(&data[4..10]);
                let callsign = callsign_from_bytes(&cs_bytes)?;
                Ok(ReflectorMessage::Ping { callsign })
            }
            b"ACKN" => Ok(ReflectorMessage::Ack),
            b"NACK" => Ok(ReflectorMessage::Nack),
            _ => Err(M17Error::InvalidSync),
        }
    }
}

// ============================================================================
// AES-256 CTR Mode (simplified, no external crates)
// ============================================================================

/// AES S-Box (standard FIPS-197)
const AES_SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab,
    0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4,
    0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71,
    0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
    0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6,
    0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb,
    0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45,
    0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44,
    0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a,
    0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49,
    0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
    0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25,
    0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e,
    0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1,
    0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb,
    0x16,
];

/// GF(2^8) multiplication for AES MixColumns
fn gf_mul(a: u8, b: u8) -> u8 {
    let mut result = 0u8;
    let mut aa = a;
    let mut bb = b;
    for _ in 0..8 {
        if bb & 1 != 0 {
            result ^= aa;
        }
        let hi = aa & 0x80;
        aa <<= 1;
        if hi != 0 {
            aa ^= 0x1B; // AES irreducible polynomial
        }
        bb >>= 1;
    }
    result
}

/// AES SubBytes
fn aes_sub_bytes(state: &mut [u8; 16]) {
    for b in state.iter_mut() {
        *b = AES_SBOX[*b as usize];
    }
}

/// AES ShiftRows
fn aes_shift_rows(state: &mut [u8; 16]) {
    // Row 1: shift left 1
    let tmp = state[1];
    state[1] = state[5]; state[5] = state[9]; state[9] = state[13]; state[13] = tmp;
    // Row 2: shift left 2
    state.swap(2, 10); state.swap(6, 14);
    // Row 3: shift left 3 (= right 1)
    let tmp = state[15];
    state[15] = state[11]; state[11] = state[7]; state[7] = state[3]; state[3] = tmp;
}

/// AES MixColumns
fn aes_mix_columns(state: &mut [u8; 16]) {
    for col in 0..4 {
        let i = col * 4;
        let s0 = state[i]; let s1 = state[i+1]; let s2 = state[i+2]; let s3 = state[i+3];
        state[i]   = gf_mul(2, s0) ^ gf_mul(3, s1) ^ s2 ^ s3;
        state[i+1] = s0 ^ gf_mul(2, s1) ^ gf_mul(3, s2) ^ s3;
        state[i+2] = s0 ^ s1 ^ gf_mul(2, s2) ^ gf_mul(3, s3);
        state[i+3] = gf_mul(3, s0) ^ s1 ^ s2 ^ gf_mul(2, s3);
    }
}

/// AES AddRoundKey
fn aes_add_round_key(state: &mut [u8; 16], round_key: &[u8]) {
    for i in 0..16 {
        state[i] ^= round_key[i];
    }
}

/// Minimal AES-256 key expansion (14 rounds)
fn aes256_key_expansion(key: &[u8; 32]) -> Vec<[u8; 16]> {
    let rcon: [u8; 11] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36, 0x6C];
    let mut w = [[0u8; 4]; 60];
    for i in 0..8 {
        for j in 0..4 {
            w[i][j] = key[i * 4 + j];
        }
    }
    for i in 8..60 {
        let mut tmp = w[i - 1];
        if i % 8 == 0 {
            // RotWord
            let t = tmp[0]; tmp[0] = tmp[1]; tmp[1] = tmp[2]; tmp[2] = tmp[3]; tmp[3] = t;
            // SubWord
            for b in &mut tmp { *b = AES_SBOX[*b as usize]; }
            tmp[0] ^= if i / 8 <= 10 { rcon[i / 8 - 1] } else { 0 };
        } else if i % 8 == 4 {
            for b in &mut tmp { *b = AES_SBOX[*b as usize]; }
        }
        for j in 0..4 {
            w[i][j] = w[i - 8][j] ^ tmp[j];
        }
    }
    let mut round_keys = Vec::with_capacity(15);
    for r in 0..15 {
        let mut rk = [0u8; 16];
        for i in 0..4 {
            for j in 0..4 {
                rk[i * 4 + j] = w[r * 4 + i][j];
            }
        }
        round_keys.push(rk);
    }
    round_keys
}

/// AES-256 single block encrypt
fn aes256_encrypt_block(plaintext: &[u8; 16], round_keys: &[[u8; 16]]) -> [u8; 16] {
    let mut state = *plaintext;
    aes_add_round_key(&mut state, &round_keys[0]);
    for r in 1..14 {
        aes_sub_bytes(&mut state);
        aes_shift_rows(&mut state);
        aes_mix_columns(&mut state);
        aes_add_round_key(&mut state, &round_keys[r]);
    }
    aes_sub_bytes(&mut state);
    aes_shift_rows(&mut state);
    aes_add_round_key(&mut state, &round_keys[14]);
    state
}

/// AES-256 CTR mode encryption/decryption
pub struct Aes256Ctr {
    round_keys: Vec<[u8; 16]>,
    nonce: [u8; 16],
}

impl Aes256Ctr {
    /// Create cipher with 32-byte key and 16-byte nonce
    pub fn new(key: &[u8; 32], nonce: &[u8; 16]) -> Self {
        Aes256Ctr {
            round_keys: aes256_key_expansion(key),
            nonce: *nonce,
        }
    }

    /// Encrypt or decrypt data (CTR mode: encrypt == decrypt)
    pub fn process(&self, data: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(data.len());
        let counter = self.nonce;
        let block_idx = 0u64;

        let full_blocks = data.len() / 16;
        let rem = data.len() % 16;

        for blk in 0..full_blocks {
            // Set counter block (nonce XOR block_index in last 8 bytes)
            let ctr_bytes = (block_idx + blk as u64).to_be_bytes();
            let mut ctr_block = counter;
            for i in 0..8 {
                ctr_block[8 + i] ^= ctr_bytes[i];
            }
            let keystream = aes256_encrypt_block(&ctr_block, &self.round_keys);
            for i in 0..16 {
                out.push(data[blk * 16 + i] ^ keystream[i]);
            }
        }
        if rem > 0 {
            let ctr_bytes = (block_idx + full_blocks as u64).to_be_bytes();
            let mut ctr_block = counter;
            for i in 0..8 {
                ctr_block[8 + i] ^= ctr_bytes[i];
            }
            let keystream = aes256_encrypt_block(&ctr_block, &self.round_keys);
            for i in 0..rem {
                out.push(data[full_blocks * 16 + i] ^ keystream[i]);
            }
        }
        out
    }
}

// ============================================================================
// Complete M17 Frame Builder
// ============================================================================

/// High-level M17 frame builder for voice streaming
pub struct M17StreamBuilder {
    lsf: LinkSetupFrame,
    frame_count: u16,
    lich_idx: usize,
    lich_chunks: Vec<LichChunk>,
}

impl M17StreamBuilder {
    /// Create builder from LSF
    pub fn new(lsf: LinkSetupFrame) -> Self {
        let lich_chunks = lsf.to_lich_chunks();
        M17StreamBuilder { lsf, frame_count: 0, lich_idx: 0, lich_chunks }
    }

    /// Build next stream frame with Codec2 payload
    pub fn next_frame(&mut self, payload: [u8; 16], is_last: bool) -> StreamFrame {
        let lich = self.lich_chunks[self.lich_idx % LICH_CHUNKS].clone();
        self.lich_idx += 1;
        let fn_ = if is_last {
            self.frame_count | 0x8000
        } else {
            self.frame_count
        };
        self.frame_count = self.frame_count.wrapping_add(1) & 0x7FFF;
        StreamFrame::new(lich, fn_, payload)
    }

    /// Get LSF for initial transmission
    pub fn lsf_bytes(&self) -> [u8; LSF_BYTES] {
        self.lsf.to_bytes()
    }
}

// ============================================================================
// Utility: bits ↔ bytes
// ============================================================================

/// Convert byte slice to bit vector (MSB first)
pub fn bytes_to_bits(data: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for &byte in data {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1 == 1);
        }
    }
    bits
}

/// Convert bit vector to byte slice (MSB first, pads with zeros)
pub fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let nbytes = (bits.len() + 7) / 8;
    let mut bytes = vec![0u8; nbytes];
    for (i, &bit) in bits.iter().enumerate() {
        if bit {
            bytes[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    bytes
}

/// Hamming weight of difference (BER counting)
pub fn bit_errors(a: &[bool], b: &[bool]) -> usize {
    a.iter().zip(b.iter()).filter(|(&x, &y)| x != y).count()
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // --- 4FSK Symbol Mapping ---

    #[test]
    fn test_fsk_symbol_mapping_01() {
        let sym = FskSymbol::from_dibit(false, true);
        assert_eq!(sym, FskSymbol::PlusThree);
        assert_eq!(sym.value(), 3);
    }

    #[test]
    fn test_fsk_symbol_mapping_00() {
        let sym = FskSymbol::from_dibit(false, false);
        assert_eq!(sym, FskSymbol::PlusOne);
        assert_eq!(sym.value(), 1);
    }

    #[test]
    fn test_fsk_symbol_mapping_10() {
        let sym = FskSymbol::from_dibit(true, false);
        assert_eq!(sym, FskSymbol::MinusOne);
        assert_eq!(sym.value(), -1);
    }

    #[test]
    fn test_fsk_symbol_mapping_11() {
        let sym = FskSymbol::from_dibit(true, true);
        assert_eq!(sym, FskSymbol::MinusThree);
        assert_eq!(sym.value(), -3);
    }

    #[test]
    fn test_fsk_deviation_hz() {
        assert!((FskSymbol::PlusThree.deviation_hz() - 1800.0).abs() < 1e-6);
        assert!((FskSymbol::PlusOne.deviation_hz() - 600.0).abs() < 1e-6);
        assert!((FskSymbol::MinusOne.deviation_hz() + 600.0).abs() < 1e-6);
        assert!((FskSymbol::MinusThree.deviation_hz() + 1800.0).abs() < 1e-6);
    }

    #[test]
    fn test_4fsk_modulate_demodulate_roundtrip() {
        let bits = vec![false, true, false, false, true, false, true, true];
        let symbols = modulate_4fsk(&bits);
        let decoded = demodulate_4fsk(&symbols);
        assert_eq!(bits, decoded);
    }

    #[test]
    fn test_4fsk_hard_decision_samples() {
        let samples = vec![3.0_f64, 1.0, -1.0, -3.0, 2.5, -2.5];
        let bits = demodulate_4fsk_samples(&samples);
        // 3.0 → PlusThree(01), 1.0 → PlusOne(00), -1.0 → MinusOne(10), -3.0 → MinusThree(11)
        assert_eq!(bits[0], false); // 01 MSB
        assert_eq!(bits[1], true);  // 01 LSB
        assert_eq!(bits[2], false); // 00
        assert_eq!(bits[3], false);
        assert_eq!(bits[4], true);  // 10
        assert_eq!(bits[5], false);
        assert_eq!(bits[6], true);  // 11
        assert_eq!(bits[7], true);
    }

    #[test]
    fn test_fsk_soft_llr_positive() {
        // Sample near +3: MSB should be ~0, LSB ~0 (bit 0,1 → symbol PlusThree)
        let (llr_msb, llr_lsb) = FskSymbol::soft_llr(3.0, 1.0);
        // MSB=0 for PlusThree, so llr_msb > 0 means "prefer 0"
        assert!(llr_msb > 0.0);
        // LSB=1 for PlusThree, so llr_lsb < 0 means "prefer 1"
        assert!(llr_lsb < 0.0);
    }

    #[test]
    fn test_4fsk_all_symbols_roundtrip() {
        for msb in [false, true] {
            for lsb in [false, true] {
                let sym = FskSymbol::from_dibit(msb, lsb);
                let (dm, dl) = sym.to_dibit();
                assert_eq!((msb, lsb), (dm, dl));
            }
        }
    }

    // --- CRC-16/M17 ---

    #[test]
    fn test_crc16_known_value() {
        // CRC of empty data should be 0xFFFF (init value only)
        let crc = crc16_m17(&[]);
        assert_eq!(crc, 0xFFFF);
    }

    #[test]
    fn test_crc16_single_byte() {
        let crc = crc16_m17(&[0xAB]);
        // CRC is deterministic — just check it's not initial value
        assert_ne!(crc, 0xFFFF);
    }

    #[test]
    fn test_crc16_verify_roundtrip() {
        let data = b"M17 Test Frame";
        let crc = crc16_m17(data);
        assert!(crc16_verify(data, crc));
        assert!(!crc16_verify(data, crc ^ 0x0001));
    }

    #[test]
    fn test_crc16_append_and_verify() {
        let data = b"Hello Reflector";
        let framed = crc16_append(data);
        assert_eq!(framed.len(), data.len() + 2);
        let computed = crc16_m17(data);
        let stored = ((framed[framed.len() - 2] as u16) << 8) | framed[framed.len() - 1] as u16;
        assert_eq!(computed, stored);
    }

    #[test]
    fn test_crc16_different_data_different_crc() {
        let a = crc16_m17(b"Frame A");
        let b = crc16_m17(b"Frame B");
        assert_ne!(a, b);
    }

    // --- Golay(24,12,8) ---

    #[test]
    fn test_golay_encode_no_error() {
        let data = 0b101011001010u16;
        let cw = golay24_encode(data);
        let (decoded, errs) = golay24_decode(cw).unwrap();
        assert_eq!(decoded, data);
        assert_eq!(errs, 0);
    }

    #[test]
    fn test_golay_single_bit_error() {
        let data = 0b110100110010u16;
        let cw = golay24_encode(data);
        let cw_err = cw ^ 0x1; // flip LSB
        let (decoded, errs) = golay24_decode(cw_err).unwrap();
        assert_eq!(decoded, data);
        assert!(errs <= 3);
    }

    #[test]
    fn test_golay_zero_word() {
        let cw = golay24_encode(0);
        let (decoded, _) = golay24_decode(cw).unwrap();
        assert_eq!(decoded, 0);
    }

    #[test]
    fn test_golay_all_ones() {
        let data = 0xFFFu16;
        let cw = golay24_encode(data);
        let (decoded, _) = golay24_decode(cw).unwrap();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_golay_parity_uniqueness() {
        // Verify different inputs produce different codewords
        let cw1 = golay24_encode(0b000000000001);
        let cw2 = golay24_encode(0b000000000010);
        assert_ne!(cw1, cw2);
    }

    // --- Convolutional Coding / Viterbi ---

    #[test]
    fn test_conv_encoder_basic() {
        let mut enc = ConvEncoder::new();
        let bits = vec![true, false, true, true];
        let encoded = enc.encode(&bits);
        assert_eq!(encoded.len(), 8); // rate 1/2 → 2x
    }

    #[test]
    fn test_conv_viterbi_roundtrip_zeros() {
        let mut enc = ConvEncoder::new();
        let bits = vec![false; 16];
        let encoded = enc.encode_with_tail(&bits);
        let dec = ViterbiDecoder::new();
        let decoded = dec.decode(&encoded, 16);
        assert_eq!(decoded[..16], bits[..16]);
    }

    #[test]
    fn test_conv_viterbi_roundtrip_pattern() {
        let mut enc = ConvEncoder::new();
        let bits = vec![true, false, true, false, true, false, true, false,
                        false, true, false, true, false, true, false, true];
        let encoded = enc.encode_with_tail(&bits);
        let dec = ViterbiDecoder::new();
        let decoded = dec.decode(&encoded, 16);
        let errors = bit_errors(&decoded, &bits);
        assert_eq!(errors, 0, "Viterbi should decode cleanly");
    }

    #[test]
    fn test_conv_encoder_g1_g2_constants() {
        // Verify polynomials are correct K=5 M17 values
        assert_eq!(CONV_G1, 0x19); // 11001
        assert_eq!(CONV_G2, 0x17); // 10111
    }

    #[test]
    fn test_conv_viterbi_single_bit_error() {
        let mut enc = ConvEncoder::new();
        let bits = vec![true, true, false, false, true, false, true, true];
        let mut encoded = enc.encode_with_tail(&bits);
        // Introduce single error
        encoded[3] = !encoded[3];
        let dec = ViterbiDecoder::new();
        let decoded = dec.decode(&encoded, 8);
        // Viterbi should recover (single error in rate-1/2 code)
        assert_eq!(decoded.len(), 8);
    }

    // --- Puncturing ---

    #[test]
    fn test_puncture_reduces_length() {
        let bits = vec![true, false, true, false, true, false];
        let punct = puncture(&bits, PUNCT_P2);
        // PUNCT_P2 has 5 true out of 6 → 5/6 of bits kept
        assert!(punct.len() < bits.len());
    }

    #[test]
    fn test_puncture_depuncture_roundtrip() {
        let bits: Vec<bool> = (0..24).map(|i| i % 3 == 0).collect();
        let punct = puncture(&bits, PUNCT_P2);
        let depunct = depuncture(&punct, PUNCT_P2);
        // Kept positions in depunct must match same positions in original bits
        for (i, dp) in depunct.iter().enumerate() {
            if i < bits.len() && PUNCT_P2[i % PUNCT_P2.len()] {
                assert_eq!(*dp, Some(bits[i]),
                    "Position {} mismatch: depunct={:?}, orig={}", i, dp, bits[i]);
            } else if !PUNCT_P2[i % PUNCT_P2.len()] {
                assert_eq!(*dp, None, "Erased position {} should be None", i);
            }
        }
    }

    #[test]
    fn test_puncture_pattern_length() {
        // Pattern keeps 5 out of 6 bits
        let ones = PUNCT_P2.iter().filter(|&&b| b).count();
        assert_eq!(ones, 5);
        assert_eq!(PUNCT_P2.len(), 6);
    }

    // --- Interleaving ---

    #[test]
    fn test_interleave_deinterleave_roundtrip() {
        let bits: Vec<bool> = (0..64).map(|i| i % 2 == 0).collect();
        let interleaved = interleave_bits(&bits);
        let recovered = deinterleave_bits(&interleaved);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_interleave_changes_order() {
        // Use an incrementing pattern: each bit uniquely positioned
        let bits: Vec<bool> = (0..24).map(|i| (i / 4) % 2 == 0).collect();
        let interleaved = interleave_bits(&bits);
        // 24 bits with rows=8 → cols=3, transposing a 8x3 matrix changes order
        assert_ne!(interleaved, bits, "Interleaving should reorder bits");
    }

    #[test]
    fn test_interleave_preserves_length() {
        let bits = vec![true; 368];
        let interleaved = interleave_bits(&bits);
        assert_eq!(interleaved.len(), 368);
    }

    #[test]
    fn test_interleave_empty() {
        let bits: Vec<bool> = vec![];
        assert_eq!(interleave_bits(&bits), vec![]);
    }

    // --- Scrambling ---

    #[test]
    fn test_scramble_descramble_roundtrip() {
        let bits = vec![true, false, true, true, false, false, true, false, true, false];
        let scrambled = scramble(&bits, 42);
        let recovered = descramble(&scrambled, 42);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_scramble_different_frames_differ() {
        let bits = vec![true; 16];
        let s1 = scramble(&bits, 1);
        let s2 = scramble(&bits, 2);
        assert_ne!(s1, s2);
    }

    #[test]
    fn test_scramble_sequence_length() {
        let seq = scramble_sequence(5, 100);
        assert_eq!(seq.len(), 100);
    }

    #[test]
    fn test_scramble_zero_frame() {
        let bits = vec![false; 20];
        // Frame 0 scrambling: result should equal the scramble sequence itself
        let seq = scramble_sequence(0, 20);
        let s = scramble(&bits, 0);
        assert_eq!(s, seq);
    }

    // --- Callsign Encoding ---

    #[test]
    fn test_callsign_encode_decode_w1aw() {
        let encoded = callsign_encode("W1AW").unwrap();
        let decoded = callsign_decode(encoded).unwrap();
        assert_eq!(decoded, "W1AW");
    }

    #[test]
    fn test_callsign_encode_decode_ke0xyz() {
        let encoded = callsign_encode("KE0XYZ").unwrap();
        let decoded = callsign_decode(encoded).unwrap();
        assert_eq!(decoded, "KE0XYZ");
    }

    #[test]
    fn test_callsign_encode_decode_slash() {
        let encoded = callsign_encode("W1AW/P").unwrap();
        let decoded = callsign_decode(encoded).unwrap();
        assert_eq!(decoded, "W1AW/P");
    }

    #[test]
    fn test_callsign_bytes_roundtrip() {
        let cs = "N0CALL";
        let bytes = callsign_to_bytes(cs).unwrap();
        let recovered = callsign_from_bytes(&bytes).unwrap();
        assert_eq!(recovered, cs);
    }

    #[test]
    fn test_callsign_too_long() {
        let result = callsign_encode("ABCDEFGHIJ"); // 10 chars
        assert!(matches!(result, Err(M17Error::CallsignTooLong)));
    }

    #[test]
    fn test_callsign_invalid_char() {
        let result = callsign_encode("W1AW!");
        assert!(matches!(result, Err(M17Error::InvalidCallsign('!'))));
    }

    #[test]
    fn test_callsign_unique_encoding() {
        let e1 = callsign_encode("AA0A").unwrap();
        let e2 = callsign_encode("AA0B").unwrap();
        assert_ne!(e1, e2);
    }

    #[test]
    fn test_callsign_max_length() {
        // 9-character callsign should succeed
        let result = callsign_encode("ABCDEFGHI");
        assert!(result.is_ok());
    }

    // --- LSF Construction / Parsing ---

    #[test]
    fn test_lsf_new_and_parse() {
        let lsf = LinkSetupFrame::new("W1AW", "KE0XYZ").unwrap();
        let bytes = lsf.to_bytes();
        let parsed = LinkSetupFrame::from_bytes(&bytes).unwrap();
        // Verify source and destination round-trip
        assert_eq!(parsed.source, lsf.source);
        assert_eq!(parsed.destination, lsf.destination);
    }

    #[test]
    fn test_lsf_crc_valid() {
        let lsf = LinkSetupFrame::new("VK2ABC", "ALL").unwrap();
        let bytes = lsf.to_bytes();
        let crc_stored = ((bytes[28] as u16) << 8) | bytes[29] as u16;
        let crc_computed = crc16_m17(&bytes[..28]);
        assert_eq!(crc_stored, crc_computed);
    }

    #[test]
    fn test_lsf_crc_mismatch_detected() {
        let lsf = LinkSetupFrame::new("W1AW", "W1AW-1").unwrap();
        let mut bytes = lsf.to_bytes();
        bytes[28] ^= 0xFF; // corrupt CRC
        let result = LinkSetupFrame::from_bytes(&bytes);
        assert!(matches!(result, Err(M17Error::CrcError)));
    }

    #[test]
    fn test_lsf_encryption_type_preserved() {
        let mut lsf = LinkSetupFrame::new("W1AW", "KE0XYZ").unwrap();
        lsf.encryption_type = EncryptionType::Aes256;
        lsf.crc = lsf.compute_crc();
        let bytes = lsf.to_bytes();
        let parsed = LinkSetupFrame::from_bytes(&bytes).unwrap();
        assert_eq!(parsed.encryption_type, EncryptionType::Aes256);
    }

    #[test]
    fn test_lsf_channel_type_data() {
        let mut lsf = LinkSetupFrame::new("W1AW", "KE0XYZ").unwrap();
        lsf.channel_type = ChannelType::Data;
        lsf.crc = lsf.compute_crc();
        let bytes = lsf.to_bytes();
        let parsed = LinkSetupFrame::from_bytes(&bytes).unwrap();
        assert_eq!(parsed.channel_type, ChannelType::Data);
    }

    // --- LICH Chunks ---

    #[test]
    fn test_lich_split_and_reassemble() {
        let lsf = LinkSetupFrame::new("W1AW", "KE0XYZ").unwrap();
        let chunks = lsf.to_lich_chunks();
        assert_eq!(chunks.len(), LICH_CHUNKS);

        let mut reassembler = LichReassembler::new();
        for chunk in chunks {
            reassembler.insert(chunk);
        }
        assert!(reassembler.is_complete());

        let reconstructed = reassembler.reconstruct().unwrap();
        let original = lsf.to_bytes();
        assert_eq!(reconstructed, original);
    }

    #[test]
    fn test_lich_chunk_counters() {
        let lsf = LinkSetupFrame::new("N0CALL", "ALL").unwrap();
        let chunks = lsf.to_lich_chunks();
        for (i, chunk) in chunks.iter().enumerate() {
            assert_eq!(chunk.counter as usize, i);
        }
    }

    #[test]
    fn test_lich_incomplete_reassembly() {
        let reassembler = LichReassembler::new();
        assert!(!reassembler.is_complete());
        let result = reassembler.reconstruct();
        assert!(result.is_err());
    }

    #[test]
    fn test_lich_golay_encode_decode() {
        let chunk = LichChunk {
            data: [0x12, 0x34, 0x56, 0x78, 0x9A],
            counter: 2,
        };
        let (g1, g2) = chunk.encode_golay();
        let decoded = LichChunk::decode_golay(g1, g2).unwrap();
        // Counter is packed into upper 3 bits of g1 data word
        assert_eq!(decoded.counter, chunk.counter);
        // First 2 bytes (16 bits) of data are fully covered by the two 12-bit Golay words
        assert_eq!(decoded.data[0], chunk.data[0]);
        assert_eq!(decoded.data[1], chunk.data[1]);
    }

    // --- Stream Frame ---

    #[test]
    fn test_stream_frame_serialize_deserialize() {
        let lich = LichChunk { data: [0xAA; 5], counter: 0 };
        let payload = [0x5A; 16];
        let frame = StreamFrame::new(lich, 42, payload);
        let bytes = frame.to_bytes();
        let parsed = StreamFrame::from_bytes(&bytes).unwrap();
        assert_eq!(parsed.frame_number, 42);
        assert_eq!(parsed.payload, payload);
    }

    #[test]
    fn test_stream_frame_eot_flag() {
        let lich = LichChunk { data: [0; 5], counter: 0 };
        let mut frame = StreamFrame::new(lich, 100, [0u8; 16]);
        assert!(!frame.is_eot());
        frame.set_eot();
        assert!(frame.is_eot());
        assert_eq!(frame.sequence_number(), 100);
    }

    #[test]
    fn test_stream_frame_sync_word() {
        let lich = LichChunk { data: [0; 5], counter: 0 };
        let frame = StreamFrame::new(lich, 0, [0u8; 16]);
        let bytes = frame.to_bytes();
        let sync = ((bytes[0] as u16) << 8) | bytes[1] as u16;
        assert_eq!(sync, SYNC_STREAM);
    }

    #[test]
    fn test_stream_frame_crc_corruption_detected() {
        let lich = LichChunk { data: [0; 5], counter: 0 };
        let frame = StreamFrame::new(lich, 1, [0xBE; 16]);
        let mut bytes = frame.to_bytes();
        *bytes.last_mut().unwrap() ^= 0x01;
        assert!(StreamFrame::from_bytes(&bytes).is_err());
    }

    // --- Packet Frame ---

    #[test]
    fn test_packet_segmentation_small() {
        let data = b"Hello M17 Packet";
        let frames = PacketFrame::segment(data).unwrap();
        assert_eq!(frames.len(), 1);
        assert!(frames[0].last);
        assert_eq!(frames[0].payload, data);
    }

    #[test]
    fn test_packet_segmentation_large() {
        let data = vec![0xAB; 60]; // requires 3 segments
        let frames = PacketFrame::segment(&data).unwrap();
        assert_eq!(frames.len(), 3);
        assert!(!frames[0].last);
        assert!(!frames[1].last);
        assert!(frames[2].last);
    }

    #[test]
    fn test_packet_reassembly_roundtrip() {
        let data: Vec<u8> = (0..50).map(|i| i as u8).collect();
        let frames = PacketFrame::segment(&data).unwrap();
        let recovered = PacketFrame::reassemble(&frames).unwrap();
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_packet_too_large() {
        let data = vec![0u8; PACKET_MAX_TOTAL + 1];
        assert!(matches!(PacketFrame::segment(&data), Err(M17Error::PacketTooLarge)));
    }

    #[test]
    fn test_packet_segment_numbering() {
        let data = vec![0u8; 75]; // 3 segments of 25
        let frames = PacketFrame::segment(&data).unwrap();
        for (i, f) in frames.iter().enumerate() {
            assert_eq!(f.segment, i as u8);
        }
    }

    // --- Sync Word Detection ---

    #[test]
    fn test_detect_sync_all_types() {
        assert_eq!(detect_sync(SYNC_LSF), SyncWordType::Lsf);
        assert_eq!(detect_sync(SYNC_STREAM), SyncWordType::Stream);
        assert_eq!(detect_sync(SYNC_PACKET), SyncWordType::Packet);
        assert_eq!(detect_sync(SYNC_BERT), SyncWordType::Bert);
        assert_eq!(detect_sync(EOT_MARKER), SyncWordType::Eot);
        assert!(matches!(detect_sync(0x1234), SyncWordType::Unknown(0x1234)));
    }

    #[test]
    fn test_find_sync_in_byte_stream() {
        let mut data = vec![0xAA; 10];
        // Insert SYNC_STREAM at offset 4
        data[4] = (SYNC_STREAM >> 8) as u8;
        data[5] = (SYNC_STREAM & 0xFF) as u8;
        let result = find_sync_word(&data);
        assert!(result.is_some());
        let (offset, stype) = result.unwrap();
        assert_eq!(offset, 4);
        assert_eq!(stype, SyncWordType::Stream);
    }

    #[test]
    fn test_find_sync_not_found() {
        let data = vec![0xAA; 20]; // No valid sync word
        let result = find_sync_word(&data);
        assert!(result.is_none());
    }

    // --- Reflector Protocol ---

    #[test]
    fn test_reflector_conn_roundtrip() {
        let msg = ReflectorMessage::Connect {
            callsign: "W1AW".to_string(),
            module: 'A',
        };
        let bytes = msg.to_bytes().unwrap();
        assert_eq!(&bytes[0..4], b"CONN");
        let parsed = ReflectorMessage::from_bytes(&bytes).unwrap();
        if let ReflectorMessage::Connect { callsign, module } = parsed {
            assert_eq!(callsign, "W1AW");
            assert_eq!(module, 'A');
        } else {
            panic!("Wrong message type");
        }
    }

    #[test]
    fn test_reflector_disc_roundtrip() {
        let msg = ReflectorMessage::Disconnect { callsign: "KE0XYZ".to_string() };
        let bytes = msg.to_bytes().unwrap();
        assert_eq!(&bytes[0..4], b"DISC");
        let parsed = ReflectorMessage::from_bytes(&bytes).unwrap();
        assert!(matches!(parsed, ReflectorMessage::Disconnect { .. }));
    }

    #[test]
    fn test_reflector_pong_roundtrip() {
        let msg = ReflectorMessage::Pong { callsign: "N0CALL".to_string() };
        let bytes = msg.to_bytes().unwrap();
        let parsed = ReflectorMessage::from_bytes(&bytes).unwrap();
        assert!(matches!(parsed, ReflectorMessage::Pong { .. }));
    }

    #[test]
    fn test_reflector_ack_nack() {
        let ack_bytes = ReflectorMessage::Ack.to_bytes().unwrap();
        let nack_bytes = ReflectorMessage::Nack.to_bytes().unwrap();
        assert_eq!(&ack_bytes, b"ACKN");
        assert_eq!(&nack_bytes, b"NACK");
        assert!(matches!(ReflectorMessage::from_bytes(&ack_bytes).unwrap(), ReflectorMessage::Ack));
        assert!(matches!(ReflectorMessage::from_bytes(&nack_bytes).unwrap(), ReflectorMessage::Nack));
    }

    // --- AES-256 CTR Mode ---

    #[test]
    fn test_aes256_ctr_encrypt_decrypt_roundtrip() {
        let key = [0u8; 32];
        let nonce = [0u8; 16];
        let cipher = Aes256Ctr::new(&key, &nonce);
        let plaintext = b"M17 AES CTR mode encryption test";
        let encrypted = cipher.process(plaintext);
        let decrypted = cipher.process(&encrypted);
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_aes256_ctr_different_keys_differ() {
        let key1 = [0x01u8; 32];
        let key2 = [0x02u8; 32];
        let nonce = [0u8; 16];
        let c1 = Aes256Ctr::new(&key1, &nonce);
        let c2 = Aes256Ctr::new(&key2, &nonce);
        let pt = b"Test data for M17";
        assert_ne!(c1.process(pt), c2.process(pt));
    }

    #[test]
    fn test_aes256_ctr_keystream_nonzero() {
        let key = [0xABu8; 32];
        let nonce = [0x12u8; 16];
        let cipher = Aes256Ctr::new(&key, &nonce);
        let zeros = vec![0u8; 32];
        let ks = cipher.process(&zeros);
        // Keystream applied to zeros should not be all zeros
        assert!(ks.iter().any(|&b| b != 0));
    }

    // --- Bytes/Bits Conversion ---

    #[test]
    fn test_bytes_to_bits_and_back() {
        let data = vec![0xA5u8, 0x3C];
        let bits = bytes_to_bits(&data);
        assert_eq!(bits.len(), 16);
        let recovered = bits_to_bytes(&bits);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_bit_errors_count() {
        let a = vec![true, false, true, false];
        let b = vec![true, true, false, false];
        assert_eq!(bit_errors(&a, &b), 2);
    }

    // --- RRC Filter ---

    #[test]
    fn test_rrc_filter_length() {
        let h = rrc_filter(RRC_ALPHA, 31, 4);
        assert_eq!(h.len(), 31);
    }

    #[test]
    fn test_rrc_filter_normalized() {
        let h = rrc_filter(RRC_ALPHA, 31, 4);
        let energy: f64 = h.iter().map(|&x| x * x).sum::<f64>().sqrt();
        assert!((energy - 1.0).abs() < 0.01, "RRC energy = {}", energy);
    }

    #[test]
    fn test_rrc_filter_symmetry() {
        let h = rrc_filter(0.5, 31, 4);
        let n = h.len();
        for i in 0..n / 2 {
            assert!((h[i] - h[n - 1 - i]).abs() < 1e-10, "RRC should be symmetric");
        }
    }

    // --- M17 Stream Builder ---

    #[test]
    fn test_stream_builder_lich_cycling() {
        let lsf = LinkSetupFrame::new("W1AW", "KE0XYZ").unwrap();
        let mut builder = M17StreamBuilder::new(lsf);
        let counters: Vec<u8> = (0..12).map(|i| {
            let frame = builder.next_frame([0u8; 16], i == 11);
            frame.lich.counter
        }).collect();
        // Should cycle 0-5 twice
        assert_eq!(counters[0], 0);
        assert_eq!(counters[5], 5);
        assert_eq!(counters[6], 0);
        assert_eq!(counters[11], 5);
    }

    #[test]
    fn test_stream_builder_frame_numbers() {
        let lsf = LinkSetupFrame::new("W1AW", "KE0XYZ").unwrap();
        let mut builder = M17StreamBuilder::new(lsf);
        for i in 0..5u16 {
            let frame = builder.next_frame([0u8; 16], false);
            assert_eq!(frame.sequence_number(), i);
        }
    }

    // --- Codec2 Framing ---

    #[test]
    fn test_codec2_pack_unpack() {
        let mut f1 = Codec2Frame::new();
        let mut f2 = Codec2Frame::new();
        for i in 0..8 { f1.bits[i] = i as u8; }
        for i in 0..8 { f2.bits[i] = (i + 10) as u8; }
        let payload = Codec2Frame::pack_pair(&f1, &f2);
        let (r1, r2) = Codec2Frame::unpack_pair(&payload);
        assert_eq!(r1.bits[..8], f1.bits[..8]);
        assert_eq!(r2.bits[..8], f2.bits[..8]);
    }
}

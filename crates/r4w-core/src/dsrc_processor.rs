//! DSRC (Dedicated Short-Range Communications) Processor
//!
//! Implements IEEE 802.11p / WAVE (Wireless Access in Vehicular Environments)
//! physical and protocol layer processing for Intelligent Transportation Systems (ITS).
//!
//! # Standards
//! - IEEE 802.11p-2010: Amendment for WAVE physical and MAC layers
//! - IEEE 1609.3: WAVE networking services (WSMP)
//! - IEEE 1609.4: Multi-channel operation
//! - SAE J2735: Dedicated Short-Range Communications Message Set Dictionary (BSM)
//!
//! # Key Parameters (802.11p vs 802.11a)
//! - Carrier: 5.850–5.925 GHz ITS band (US), 5.855–5.925 GHz (EU)
//! - Channel bandwidth: 10 MHz (half of 802.11a's 20 MHz)
//! - FFT size: 64 points
//! - Subcarriers: 52 used (48 data + 4 pilots)
//! - Symbol duration: 8.0 µs (6.4 µs FFT + 1.6 µs GI)
//! - Data rates: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbps
//! - Convolutional code: rate 1/2, k=7, G1=0133, G2=0171 (octal)

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// FFT size for 802.11p OFDM
pub const FFT_SIZE: usize = 64;

/// Number of used subcarriers (48 data + 4 pilot)
pub const NUM_USED_SUBCARRIERS: usize = 52;

/// Number of data subcarriers per OFDM symbol
pub const NUM_DATA_SUBCARRIERS: usize = 48;

/// Number of pilot subcarriers per OFDM symbol
pub const NUM_PILOT_SUBCARRIERS: usize = 4;

/// Guard interval length in samples (1.6 µs at 10 MHz = 16 samples)
pub const GI_LENGTH: usize = 16;

/// OFDM symbol duration = FFT_SIZE + GI_LENGTH
pub const SYMBOL_SAMPLES: usize = FFT_SIZE + GI_LENGTH;

/// Short Training Sequence repetitions
pub const STS_REPETITIONS: usize = 10;

/// Long Training Sequence symbols
pub const LTS_SYMBOLS: usize = 2;

/// Convolutional code constraint length
pub const CONV_K: usize = 7;

/// Convolutional code generator polynomial G1 (133 octal = 0b1011011)
pub const CONV_G1: u8 = 0b1011011;

/// Convolutional code generator polynomial G2 (171 octal = 0b1111001)
pub const CONV_G2: u8 = 0b1111001;

/// CCH channel number (Control Channel per IEEE 1609.4)
pub const CCH_CHANNEL: u8 = 178;

/// SCH channel numbers (Service Channels per IEEE 1609.4)
pub const SCH_CHANNELS: [u8; 6] = [172, 174, 176, 180, 182, 184];

/// Sync interval duration in milliseconds per IEEE 1609.4
pub const SYNC_INTERVAL_MS: u64 = 100;

/// CCH interval fraction (50 ms of 100 ms sync)
pub const CCH_INTERVAL_MS: u64 = 50;

/// SCH interval fraction (50 ms of 100 ms sync)
pub const SCH_INTERVAL_MS: u64 = 50;

/// WSMP version (IEEE 1609.3)
pub const WSMP_VERSION: u8 = 3;

/// ITS-G5 center frequency in Hz for channel 178 (5.890 GHz)
pub const CHANNEL_178_FREQ_HZ: f64 = 5_890_000_000.0;

/// Channel spacing in Hz (10 MHz for DSRC)
pub const CHANNEL_SPACING_HZ: f64 = 10_000_000.0;

/// Sample rate for 802.11p at 10 MHz channel bandwidth
pub const SAMPLE_RATE_HZ: f64 = 10_000_000.0;

// ---------------------------------------------------------------------------
// MCS / Rate Table
// ---------------------------------------------------------------------------

/// Modulation and Coding Scheme entry for 802.11p
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mcs {
    /// MCS index (0–7)
    pub index: u8,
    /// Data rate in Mbps
    pub data_rate_mbps: f32,
    /// Modulation order (1=BPSK, 2=QPSK, 4=16QAM, 6=64QAM)
    pub bits_per_symbol: u8,
    /// Code rate numerator
    pub code_rate_num: u8,
    /// Code rate denominator
    pub code_rate_den: u8,
    /// Minimum SNR for this MCS (dB)
    pub min_snr_db: f32,
    /// Coded bits per OFDM symbol (= bits_per_symbol * 48)
    pub coded_bits_per_symbol: u16,
    /// Data bits per OFDM symbol
    pub data_bits_per_symbol: u16,
}

impl Mcs {
    /// Returns the puncturing pattern for this rate.
    /// Rate 1/2: no puncturing. Rate 2/3: [1,1,0,1]. Rate 3/4: [1,1,0,1,1,0].
    pub fn puncture_pattern(&self) -> &'static [u8] {
        match (self.code_rate_num, self.code_rate_den) {
            (1, 2) => &[1, 1],
            (2, 3) => &[1, 1, 0, 1],
            (3, 4) => &[1, 1, 0, 1, 1, 0],
            _ => &[1, 1],
        }
    }
}

/// 802.11p MCS table (8 data rates per IEEE 802.11p-2010 Table 17-15)
pub const MCS_TABLE: [Mcs; 8] = [
    Mcs { index: 0, data_rate_mbps: 3.0,  bits_per_symbol: 1, code_rate_num: 1, code_rate_den: 2, min_snr_db: 4.0,  coded_bits_per_symbol: 48,  data_bits_per_symbol: 24  },
    Mcs { index: 1, data_rate_mbps: 4.5,  bits_per_symbol: 1, code_rate_num: 3, code_rate_den: 4, min_snr_db: 5.0,  coded_bits_per_symbol: 48,  data_bits_per_symbol: 36  },
    Mcs { index: 2, data_rate_mbps: 6.0,  bits_per_symbol: 2, code_rate_num: 1, code_rate_den: 2, min_snr_db: 6.5,  coded_bits_per_symbol: 96,  data_bits_per_symbol: 48  },
    Mcs { index: 3, data_rate_mbps: 9.0,  bits_per_symbol: 2, code_rate_num: 3, code_rate_den: 4, min_snr_db: 8.5,  coded_bits_per_symbol: 96,  data_bits_per_symbol: 72  },
    Mcs { index: 4, data_rate_mbps: 12.0, bits_per_symbol: 4, code_rate_num: 1, code_rate_den: 2, min_snr_db: 11.5, coded_bits_per_symbol: 192, data_bits_per_symbol: 96  },
    Mcs { index: 5, data_rate_mbps: 18.0, bits_per_symbol: 4, code_rate_num: 3, code_rate_den: 4, min_snr_db: 14.5, coded_bits_per_symbol: 192, data_bits_per_symbol: 144 },
    Mcs { index: 6, data_rate_mbps: 24.0, bits_per_symbol: 6, code_rate_num: 2, code_rate_den: 3, min_snr_db: 17.5, coded_bits_per_symbol: 288, data_bits_per_symbol: 192 },
    Mcs { index: 7, data_rate_mbps: 27.0, bits_per_symbol: 6, code_rate_num: 3, code_rate_den: 4, min_snr_db: 19.0, coded_bits_per_symbol: 288, data_bits_per_symbol: 216 },
];

/// Select MCS for a given SNR (returns highest supportable rate)
pub fn select_mcs(snr_db: f32) -> &'static Mcs {
    let mut best = &MCS_TABLE[0];
    for mcs in &MCS_TABLE {
        if snr_db >= mcs.min_snr_db {
            best = mcs;
        }
    }
    best
}

/// Get MCS by index
pub fn mcs_by_index(index: u8) -> Option<&'static Mcs> {
    MCS_TABLE.iter().find(|m| m.index == index)
}

// ---------------------------------------------------------------------------
// Pilot subcarrier indices (802.11p, same as 802.11a)
// ---------------------------------------------------------------------------

/// Pilot subcarrier indices in the 64-point FFT (relative to DC, -32..+31)
pub const PILOT_INDICES: [i32; 4] = [-21, -7, 7, 21];

/// Data subcarrier indices (all used subcarriers minus pilots, minus DC, minus guards)
/// 802.11p uses subcarriers -26..-1, +1..+26 excluding pilots at ±7, ±21
pub fn data_subcarrier_indices() -> Vec<i32> {
    let used: Vec<i32> = (-26_i32..=26_i32).filter(|&k| k != 0).collect();
    used.into_iter()
        .filter(|k| !PILOT_INDICES.contains(k))
        .collect()
}

// ---------------------------------------------------------------------------
// Complex number (simple f64 pair, no external crate)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub fn new(re: f64, im: f64) -> Self { Self { re, im } }
    pub fn zero() -> Self { Self { re: 0.0, im: 0.0 } }
    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self { re: r * theta.cos(), im: r * theta.sin() }
    }
    pub fn abs(&self) -> f64 { (self.re * self.re + self.im * self.im).sqrt() }
    pub fn abs_sq(&self) -> f64 { self.re * self.re + self.im * self.im }
    pub fn conj(&self) -> Self { Self { re: self.re, im: -self.im } }
    pub fn mul(&self, other: &Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
    pub fn add(&self, other: &Self) -> Self {
        Self { re: self.re + other.re, im: self.im + other.im }
    }
    pub fn scale(&self, s: f64) -> Self { Self { re: self.re * s, im: self.im * s } }
    pub fn arg(&self) -> f64 { self.im.atan2(self.re) }
}

// ---------------------------------------------------------------------------
// FFT (radix-2 Cooley-Tukey, in-place, pure Rust)
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT.  `buf` must have power-of-2 length.
/// `inverse = false` computes FFT; `true` computes IFFT (unnormalized).
pub fn fft_inplace(buf: &mut [Complex], inverse: bool) {
    let n = buf.len();
    // Bit-reverse permutation
    let bits = n.trailing_zeros() as usize;
    for i in 0..n {
        let j = bit_reverse(i, bits);
        if i < j {
            buf.swap(i, j);
        }
    }
    // Cooley-Tukey butterfly
    let mut len = 2;
    while len <= n {
        let ang = if inverse { 2.0 * PI / len as f64 } else { -2.0 * PI / len as f64 };
        let wlen = Complex::from_polar(1.0, ang);
        for i in (0..n).step_by(len) {
            let mut w = Complex::new(1.0, 0.0);
            for j in 0..len / 2 {
                let u = buf[i + j];
                let v = buf[i + j + len / 2].mul(&w);
                buf[i + j] = u.add(&v);
                buf[i + j + len / 2] = Complex::new(u.re - v.re, u.im - v.im);
                w = w.mul(&wlen);
            }
        }
        len <<= 1;
    }
    if inverse {
        let n_f = n as f64;
        for s in buf.iter_mut() {
            s.re /= n_f;
            s.im /= n_f;
        }
    }
}

fn bit_reverse(mut x: usize, bits: usize) -> usize {
    let mut result = 0;
    for _ in 0..bits {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    result
}

/// Forward FFT (returns new Vec)
pub fn fft(input: &[Complex]) -> Vec<Complex> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, false);
    buf
}

/// Inverse FFT (normalized)
pub fn ifft(input: &[Complex]) -> Vec<Complex> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, true);
    buf
}

// ---------------------------------------------------------------------------
// Short Training Sequence (STS) per 802.11p / 802.11a
// ---------------------------------------------------------------------------

/// Generate 802.11p Short Training Sequence (STS).
/// 12 subcarriers modulated with BPSK using S[k] sequence (same as 802.11a).
/// Returns 10 repetitions of a 16-sample STS symbol (160 samples total).
pub fn generate_sts() -> Vec<Complex> {
    // 802.11a STS frequency-domain sequence (12 non-zero subcarriers out of 64)
    // These are placed at indices -24,-20,-16,-12,-8,-4,+4,+8,+12,+16,+20,+24
    let s_seq: [i8; 12] = [1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1]; // simplified ±1 BPSK
    let sts_subcarriers: [usize; 12] = [40, 44, 48, 52, 56, 60, 4, 8, 12, 16, 20, 24];

    let mut freq_domain = vec![Complex::zero(); 64];
    let scale = (13.0_f64 / 6.0_f64).sqrt(); // normalization factor
    for (i, &k) in sts_subcarriers.iter().enumerate() {
        freq_domain[k] = Complex::new(s_seq[i] as f64 * scale, 0.0);
    }
    let time_domain = ifft(&freq_domain);

    // STS period is 16 samples (64/4), repeat 10 times + guard interval
    let sts_period = 16;
    let mut out = Vec::with_capacity(STS_REPETITIONS * sts_period);
    for rep in 0..STS_REPETITIONS {
        let start = (rep * sts_period) % 64;
        for i in 0..sts_period {
            out.push(time_domain[(start + i) % 64]);
        }
    }
    out
}

/// Generate 802.11p Long Training Sequence (LTS).
/// Two 64-sample OFDM symbols with standard LTS sequence, preceded by 32-sample GI.
/// Returns 32 (GI) + 64 + 64 = 160 samples.
pub fn generate_lts() -> Vec<Complex> {
    // LTS frequency domain: 52 used subcarriers with known BPSK values
    let lts_freq: [i8; 53] = [
        0,  1, -1, -1,  1,  1, -1,  1, -1,  1, -1, -1, -1, -1, -1,  1,
        1, -1, -1,  1, -1,  1, -1,  1,  1,  1,  1,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  1,  1, -1, -1,  1,  1, -1,  1, -1,  1,  1,
        1,  1,  1,  1, -1,
    ];
    // Map to FFT bins: subcarriers -26..+26 around DC=0
    let mut freq_domain = vec![Complex::zero(); 64];
    for i in 0..26 {
        let bin = 64 - 26 + i; // bins 38..63 for subcarriers -26..-1
        let lts_idx = i;
        freq_domain[bin] = Complex::new(lts_freq[lts_idx] as f64, 0.0);
    }
    for i in 1..=26 {
        let bin = i;
        let lts_idx = 26 + i;
        freq_domain[bin] = Complex::new(lts_freq[lts_idx] as f64, 0.0);
    }

    let time_symbol = ifft(&freq_domain);

    // GI = last 32 samples of LTS symbol (cyclic prefix)
    let gi_len = 32;
    let mut out = Vec::with_capacity(gi_len + 64 * LTS_SYMBOLS);
    for i in (64 - gi_len)..64 {
        out.push(time_symbol[i]);
    }
    // Two LTS symbols
    for _ in 0..LTS_SYMBOLS {
        out.extend_from_slice(&time_symbol);
    }
    out
}

// ---------------------------------------------------------------------------
// SIGNAL field encoding / decoding (802.11p preamble)
// ---------------------------------------------------------------------------

/// SIGNAL field content (24 bits: rate 4b, reserved 1b, length 12b, parity 1b, tail 6b)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SignalField {
    /// Rate code (4 bits, see SIGNAL_RATE_CODES)
    pub rate_code: u8,
    /// PSDU length in octets (12 bits, 0–4095)
    pub length: u16,
}

/// Map MCS index to 4-bit rate code in SIGNAL field (per 802.11a Table 17-6)
pub const SIGNAL_RATE_CODES: [u8; 8] = [
    0b1101, // 3 Mbps BPSK 1/2
    0b1111, // 4.5 Mbps BPSK 3/4
    0b0101, // 6 Mbps QPSK 1/2
    0b0111, // 9 Mbps QPSK 3/4
    0b1001, // 12 Mbps 16-QAM 1/2
    0b1011, // 18 Mbps 16-QAM 3/4
    0b0001, // 24 Mbps 64-QAM 2/3
    0b0011, // 27 Mbps 64-QAM 3/4
];

impl SignalField {
    /// Encode 24-bit SIGNAL field into bits (LSB first per 802.11p)
    pub fn encode(&self) -> [u8; 24] {
        let mut bits = [0u8; 24];
        // Rate (4 bits, LSB first)
        for i in 0..4 {
            bits[i] = (self.rate_code >> i) & 1;
        }
        // Reserved bit
        bits[4] = 0;
        // Length (12 bits, LSB first)
        for i in 0..12 {
            bits[5 + i] = ((self.length >> i) & 1) as u8;
        }
        // Parity (even parity over bits 0..16)
        let parity: u8 = bits[0..17].iter().sum::<u8>() & 1;
        bits[17] = parity;
        // Tail (6 zero bits)
        for i in 18..24 {
            bits[i] = 0;
        }
        bits
    }

    /// Decode SIGNAL field from 24 bits
    pub fn decode(bits: &[u8; 24]) -> Option<Self> {
        let rate_code = bits[0] | (bits[1] << 1) | (bits[2] << 2) | (bits[3] << 3);
        let length = bits[5..17].iter().enumerate()
            .fold(0u16, |acc, (i, &b)| acc | ((b as u16) << i));
        // Verify parity
        let parity: u8 = bits[0..17].iter().sum::<u8>() & 1;
        if parity != bits[17] {
            return None;
        }
        Some(Self { rate_code, length })
    }

    /// Get MCS index from rate code
    pub fn mcs_index(&self) -> Option<u8> {
        SIGNAL_RATE_CODES.iter().position(|&r| r == self.rate_code).map(|i| i as u8)
    }
}

// ---------------------------------------------------------------------------
// Convolutional Encoder (rate 1/2, k=7, G1=0133, G2=0171 octal)
// ---------------------------------------------------------------------------

/// Convolutional encoder state
#[derive(Debug, Clone)]
pub struct ConvEncoder {
    state: u8, // 6-bit shift register
}

impl ConvEncoder {
    pub fn new() -> Self { Self { state: 0 } }

    /// Encode one bit, returns (c1, c2) coded bits
    pub fn encode_bit(&mut self, bit: u8) -> (u8, u8) {
        let input = (bit & 1) << 6;
        let reg = (input | self.state) as u8;
        let c1 = (reg & CONV_G1).count_ones() as u8 & 1;
        let c2 = (reg & CONV_G2).count_ones() as u8 & 1;
        self.state = (reg >> 1) & 0x3F;
        (c1, c2)
    }

    /// Encode a byte slice, returns coded bits
    pub fn encode_bits(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(bits.len() * 2);
        for &b in bits {
            let (c1, c2) = self.encode_bit(b);
            out.push(c1);
            out.push(c2);
        }
        out
    }

    /// Reset encoder state
    pub fn reset(&mut self) { self.state = 0; }

    /// Flush tail bits (6 zero bits to reset shift register)
    pub fn flush(&mut self) -> Vec<u8> {
        self.encode_bits(&[0, 0, 0, 0, 0, 0])
    }
}

impl Default for ConvEncoder {
    fn default() -> Self { Self::new() }
}

// ---------------------------------------------------------------------------
// Puncturing / Depuncturing
// ---------------------------------------------------------------------------

/// Puncture coded bits according to pattern.
/// Pattern `[1,1,0,1]` means: keep bit 0, keep bit 1, drop bit 2, keep bit 3, repeat.
pub fn puncture(coded: &[u8], pattern: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    for (i, &bit) in coded.iter().enumerate() {
        if pattern[i % pattern.len()] == 1 {
            out.push(bit);
        }
    }
    out
}

/// Depuncture: insert erasure (127) where punctured bits were
pub fn depuncture(received: &[u8], pattern: &[u8]) -> Vec<u8> {
    let total_in = received.len();
    let ones_per_period: usize = pattern.iter().filter(|&&p| p == 1).count();
    let periods = (total_in + ones_per_period - 1) / ones_per_period;
    let total_out = periods * pattern.len();
    let mut out = Vec::with_capacity(total_out);
    let mut rx_idx = 0;
    for i in 0..total_out {
        if pattern[i % pattern.len()] == 1 {
            if rx_idx < received.len() {
                out.push(received[rx_idx]);
                rx_idx += 1;
            } else {
                out.push(127); // erasure
            }
        } else {
            out.push(127); // erasure
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Viterbi Decoder (hard-decision, rate 1/2, k=7)
// ---------------------------------------------------------------------------

const VITERBI_STATES: usize = 64; // 2^(k-1)

/// Hard-decision Viterbi decoder for rate-1/2 k=7 convolutional code.
pub struct ViterbiDecoder;

impl ViterbiDecoder {
    /// Decode received bits (0/1 hard decisions, 127 = erasure).
    /// Each traceback entry stores `(prev_state << 1) | input_bit` for the winning transition
    /// into each state at each time step.
    /// Returns decoded bits.
    pub fn decode(received: &[u8], num_bits: usize) -> Vec<u8> {
        let num_symbols = received.len() / 2;
        let mut path_metric = vec![u32::MAX / 2; VITERBI_STATES];
        // traceback[t][next_state] = (prev_state << 1) | input_bit
        let mut traceback: Vec<Vec<u8>> = vec![vec![0u8; VITERBI_STATES]; num_symbols];
        path_metric[0] = 0;

        for t in 0..num_symbols {
            let r0 = received[2 * t];
            let r1 = received[2 * t + 1];
            let mut new_pm = vec![u32::MAX / 2; VITERBI_STATES];

            for state in 0..VITERBI_STATES {
                if path_metric[state] == u32::MAX / 2 { continue; }
                for input_bit in 0u8..2 {
                    let reg = ((input_bit as usize) << 6) | state;
                    let c1 = (reg as u8 & CONV_G1).count_ones() as u8 & 1;
                    let c2 = (reg as u8 & CONV_G2).count_ones() as u8 & 1;
                    let next_state = (reg >> 1) & 0x3F;
                    let bm = hamming_dist_hard(r0, c1) + hamming_dist_hard(r1, c2);
                    let candidate = path_metric[state] + bm as u32;
                    if candidate < new_pm[next_state] {
                        new_pm[next_state] = candidate;
                        // Pack (prev_state << 1 | input_bit) into traceback
                        traceback[t][next_state] = ((state as u8) << 1) | input_bit;
                    }
                }
            }
            path_metric = new_pm;
        }

        // Find best final state
        let best_state = path_metric.iter().enumerate()
            .min_by_key(|&(_, &v)| v)
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Traceback
        let mut decoded = vec![0u8; num_symbols];
        let mut state = best_state;
        for t in (0..num_symbols).rev() {
            let tb = traceback[t][state];
            decoded[t] = tb & 1;
            state = (tb >> 1) as usize;
        }
        decoded.truncate(num_bits);
        decoded
    }
}

fn hamming_dist_hard(rx: u8, exp: u8) -> u8 {
    if rx == 127 { 0 } else { (rx ^ exp) & 1 }
}

// ---------------------------------------------------------------------------
// Interleaver (802.11p same as 802.11a)
// ---------------------------------------------------------------------------

/// 802.11p block interleaver.
/// Two-step permutation per 802.11a clause 17.3.5.7.
/// `n_cbps` = coded bits per OFDM symbol.
/// `n_bpsc` = bits per subcarrier.
pub struct Interleaver {
    pub n_cbps: usize,
    pub n_bpsc: usize,
}

impl Interleaver {
    pub fn new(n_cbps: usize, n_bpsc: usize) -> Self {
        Self { n_cbps, n_bpsc }
    }

    fn s(&self) -> usize {
        std::cmp::max(self.n_bpsc / 2, 1)
    }

    /// First permutation index
    fn p1(&self, k: usize) -> usize {
        (self.n_cbps / 16) * (k % 16) + k / 16
    }

    /// Second permutation index
    fn p2(&self, j: usize) -> usize {
        let s = self.s();
        s * (j / s) + (j + self.n_cbps - 16 * j / self.n_cbps) % s
    }

    /// Interleave a block of `n_cbps` bits
    pub fn interleave(&self, bits: &[u8]) -> Vec<u8> {
        assert_eq!(bits.len(), self.n_cbps);
        let mut out = vec![0u8; self.n_cbps];
        for k in 0..self.n_cbps {
            let j = self.p1(k);
            let i = self.p2(j);
            out[i] = bits[k];
        }
        out
    }

    /// Deinterleave a block of `n_cbps` bits
    pub fn deinterleave(&self, bits: &[u8]) -> Vec<u8> {
        assert_eq!(bits.len(), self.n_cbps);
        let mut out = vec![0u8; self.n_cbps];
        // Reverse: for each output position, find where it came from
        for k in 0..self.n_cbps {
            let j = self.p1(k);
            let i = self.p2(j);
            out[k] = bits[i];
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Constellation Mapping / Demapping
// ---------------------------------------------------------------------------

/// Modulation type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Modulation {
    Bpsk,
    Qpsk,
    Qam16,
    Qam64,
}

impl Modulation {
    pub fn bits_per_symbol(self) -> usize {
        match self {
            Modulation::Bpsk  => 1,
            Modulation::Qpsk  => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        }
    }
}

/// Gray-coded BPSK: 0→-1, 1→+1
pub fn bpsk_map(bit: u8) -> Complex {
    if bit == 0 { Complex::new(-1.0, 0.0) } else { Complex::new(1.0, 0.0) }
}

/// BPSK demap (hard)
pub fn bpsk_demap(s: &Complex) -> u8 {
    if s.re >= 0.0 { 1 } else { 0 }
}

/// Gray-coded QPSK: 2 bits → point on unit circle
pub fn qpsk_map(bits: &[u8]) -> Complex {
    let i = if bits[0] == 0 { -1.0 } else { 1.0 };
    let q = if bits[1] == 0 { -1.0 } else { 1.0 };
    Complex::new(i / 2_f64.sqrt(), q / 2_f64.sqrt())
}

/// QPSK demap (hard)
pub fn qpsk_demap(s: &Complex) -> [u8; 2] {
    [if s.re >= 0.0 { 1 } else { 0 }, if s.im >= 0.0 { 1 } else { 0 }]
}

/// 16-QAM Gray coded mapping (4 bits). Normalization factor = 1/sqrt(10).
pub fn qam16_map(bits: &[u8]) -> Complex {
    let norm = 1.0 / 10.0_f64.sqrt();
    let i = gray_to_pam4((bits[0] << 1) | bits[1]) as f64 * norm;
    let q = gray_to_pam4((bits[2] << 1) | bits[3]) as f64 * norm;
    Complex::new(i, q)
}

/// 64-QAM Gray coded mapping (6 bits). Normalization factor = 1/sqrt(42).
pub fn qam64_map(bits: &[u8]) -> Complex {
    let norm = 1.0 / 42.0_f64.sqrt();
    let i_bits = (bits[0] << 2) | (bits[1] << 1) | bits[2];
    let q_bits = (bits[3] << 2) | (bits[4] << 1) | bits[5];
    let i = gray_to_pam8(i_bits) as f64 * norm;
    let q = gray_to_pam8(q_bits) as f64 * norm;
    Complex::new(i, q)
}

fn gray_to_pam4(gray: u8) -> i32 {
    match gray & 0x3 {
        0b00 => -3, 0b01 => -1, 0b11 => 1, 0b10 => 3, _ => 0,
    }
}

fn gray_to_pam8(gray: u8) -> i32 {
    match gray & 0x7 {
        0b000 => -7, 0b001 => -5, 0b011 => -3, 0b010 => -1,
        0b110 => 1,  0b111 => 3,  0b101 => 5,  0b100 => 7,
        _ => 0,
    }
}

/// Map a slice of bits to constellation symbols for the given modulation
pub fn map_bits(bits: &[u8], modulation: Modulation) -> Vec<Complex> {
    let bps = modulation.bits_per_symbol();
    assert_eq!(bits.len() % bps, 0);
    let mut out = Vec::with_capacity(bits.len() / bps);
    let mut i = 0;
    while i + bps <= bits.len() {
        let sym = match modulation {
            Modulation::Bpsk  => bpsk_map(bits[i]),
            Modulation::Qpsk  => qpsk_map(&bits[i..i+2]),
            Modulation::Qam16 => qam16_map(&bits[i..i+4]),
            Modulation::Qam64 => qam64_map(&bits[i..i+6]),
        };
        out.push(sym);
        i += bps;
    }
    out
}

/// Demap symbols to bits (hard decision) for the given modulation
pub fn demap_symbols(symbols: &[Complex], modulation: Modulation) -> Vec<u8> {
    let mut bits = Vec::with_capacity(symbols.len() * modulation.bits_per_symbol());
    for s in symbols {
        match modulation {
            Modulation::Bpsk => bits.push(bpsk_demap(s)),
            Modulation::Qpsk => { let b = qpsk_demap(s); bits.push(b[0]); bits.push(b[1]); }
            Modulation::Qam16 => {
                let norm = 10.0_f64.sqrt();
                let i_q = [s.re * norm, s.im * norm];
                for v in &i_q {
                    let (b0, b1) = pam4_demap(*v);
                    bits.push(b0); bits.push(b1);
                }
            }
            Modulation::Qam64 => {
                let norm = 42.0_f64.sqrt();
                let i_q = [s.re * norm, s.im * norm];
                for v in &i_q {
                    let (b0, b1, b2) = pam8_demap(*v);
                    bits.push(b0); bits.push(b1); bits.push(b2);
                }
            }
        }
    }
    bits
}

fn pam4_demap(v: f64) -> (u8, u8) {
    if v >= 2.0      { (1, 0) }
    else if v >= 0.0 { (1, 1) }
    else if v >= -2.0{ (0, 1) }
    else             { (0, 0) }
}

fn pam8_demap(v: f64) -> (u8, u8, u8) {
    if      v >= 6.0  { (1, 0, 0) }
    else if v >= 4.0  { (1, 0, 1) }
    else if v >= 2.0  { (1, 1, 1) }
    else if v >= 0.0  { (1, 1, 0) }
    else if v >= -2.0 { (0, 1, 0) }
    else if v >= -4.0 { (0, 1, 1) }
    else if v >= -6.0 { (0, 0, 1) }
    else              { (0, 0, 0) }
}

// ---------------------------------------------------------------------------
// OFDM Modulator / Demodulator
// ---------------------------------------------------------------------------

/// Map a modulation type from MCS index
pub fn modulation_for_mcs(mcs_index: u8) -> Modulation {
    match mcs_index {
        0 | 1 => Modulation::Bpsk,
        2 | 3 => Modulation::Qpsk,
        4 | 5 => Modulation::Qam16,
        6 | 7 => Modulation::Qam64,
        _     => Modulation::Bpsk,
    }
}

/// OFDM symbol modulator: maps data + pilot bits to time-domain samples.
/// `data_symbols`: 48 complex data subcarriers
/// Returns `SYMBOL_SAMPLES` (80) time-domain samples.
pub fn ofdm_modulate_symbol(data_symbols: &[Complex]) -> Vec<Complex> {
    assert_eq!(data_symbols.len(), NUM_DATA_SUBCARRIERS);
    let mut freq_domain = vec![Complex::zero(); FFT_SIZE];

    // Place data subcarriers
    let data_indices = data_subcarrier_indices();
    for (i, &sc_idx) in data_indices.iter().enumerate() {
        let bin = if sc_idx < 0 { (FFT_SIZE as i32 + sc_idx) as usize } else { sc_idx as usize };
        freq_domain[bin] = data_symbols[i];
    }

    // Place pilot subcarriers (BPSK +1/-1 per 802.11a pilot sequence)
    let pilot_values = [1.0_f64, 1.0, 1.0, -1.0]; // simplified pilot polarity
    for (i, &sc_idx) in PILOT_INDICES.iter().enumerate() {
        let bin = if sc_idx < 0 { (FFT_SIZE as i32 + sc_idx) as usize } else { sc_idx as usize };
        freq_domain[bin] = Complex::new(pilot_values[i], 0.0);
    }

    // IFFT
    let time_domain = ifft(&freq_domain);

    // Add cyclic prefix
    let mut out = Vec::with_capacity(SYMBOL_SAMPLES);
    for i in (FFT_SIZE - GI_LENGTH)..FFT_SIZE {
        out.push(time_domain[i]);
    }
    out.extend_from_slice(&time_domain);
    out
}

/// OFDM symbol demodulator: strips CP, FFT, returns 48 data subcarrier symbols.
/// `channel_estimates`: per-subcarrier complex gains (None = no equalization)
pub fn ofdm_demodulate_symbol(
    samples: &[Complex],
    channel_estimates: Option<&[Complex]>,
) -> Vec<Complex> {
    assert!(samples.len() >= SYMBOL_SAMPLES);
    // Strip GI
    let fft_input = &samples[GI_LENGTH..GI_LENGTH + FFT_SIZE];
    let freq_domain = fft(fft_input);

    let data_indices = data_subcarrier_indices();
    let mut out = Vec::with_capacity(NUM_DATA_SUBCARRIERS);

    for (i, &sc_idx) in data_indices.iter().enumerate() {
        let bin = if sc_idx < 0 { (FFT_SIZE as i32 + sc_idx) as usize } else { sc_idx as usize };
        let sym = freq_domain[bin];
        let equalized = if let Some(h) = channel_estimates {
            if h[i].abs_sq() > 1e-10 {
                sym.mul(&h[i].conj()).scale(1.0 / h[i].abs_sq())
            } else {
                sym
            }
        } else {
            sym
        };
        out.push(equalized);
    }
    out
}

/// Simple pilot-based channel estimator.
/// Returns per-data-subcarrier estimates by linear interpolation from pilot estimates.
pub fn estimate_channel_from_pilots(samples: &[Complex]) -> Vec<Complex> {
    assert!(samples.len() >= SYMBOL_SAMPLES);
    let fft_input = &samples[GI_LENGTH..GI_LENGTH + FFT_SIZE];
    let freq_domain = fft(fft_input);

    // Known pilot values
    let pilot_values = [1.0_f64, 1.0, 1.0, -1.0];
    let mut pilot_estimates = [(0_i32, Complex::new(1.0, 0.0)); 4];

    for (i, &sc_idx) in PILOT_INDICES.iter().enumerate() {
        let bin = if sc_idx < 0 { (FFT_SIZE as i32 + sc_idx) as usize } else { sc_idx as usize };
        let rx = freq_domain[bin];
        let _known = Complex::new(pilot_values[i], 0.0);
        // H_est = rx / known (scalar known)
        let h_est = rx.scale(1.0 / pilot_values[i]);
        pilot_estimates[i] = (sc_idx, h_est);
    }

    // Linear interpolation to data subcarriers
    let data_indices = data_subcarrier_indices();
    let mut channel = Vec::with_capacity(NUM_DATA_SUBCARRIERS);

    for &sc_idx in &data_indices {
        // Find surrounding pilots
        let h = interpolate_channel(sc_idx, &pilot_estimates);
        channel.push(h);
    }
    channel
}

fn interpolate_channel(sc_idx: i32, pilots: &[(i32, Complex)]) -> Complex {
    // Find the two surrounding pilots
    let mut lo = pilots[0];
    let mut hi = pilots[pilots.len() - 1];
    for &p in pilots {
        if p.0 <= sc_idx && p.0 >= lo.0 { lo = p; }
        if p.0 >= sc_idx && p.0 <= hi.0 { hi = p; }
    }
    if lo.0 == hi.0 {
        return lo.1;
    }
    let t = (sc_idx - lo.0) as f64 / (hi.0 - lo.0) as f64;
    Complex::new(
        lo.1.re + t * (hi.1.re - lo.1.re),
        lo.1.im + t * (hi.1.im - lo.1.im),
    )
}

// ---------------------------------------------------------------------------
// WSMP (Wave Short Message Protocol) – IEEE 1609.3
// ---------------------------------------------------------------------------

/// WSMP header (IEEE 1609.3 WSMP-N-Header)
#[derive(Debug, Clone, PartialEq)]
pub struct WsmpHeader {
    /// Protocol stack type (0x00 for WSMP)
    pub subtype: u8,
    /// WSMP version (3)
    pub version: u8,
    /// Provider Service Identifier (32-bit PSID)
    pub psid: u32,
    /// Channel number (e.g., 178 for CCH)
    pub channel_number: u8,
    /// Data rate (encoded per 802.11p MCS, in units of 500 kbps)
    pub data_rate: u8,
    /// Transmit power in dBm + 128 (0xFF = use default)
    pub tx_power: u8,
    /// Channel load 0..100
    pub channel_load: u8,
    /// Information element length
    pub wsm_length: u16,
}

impl WsmpHeader {
    /// Create a new WSMP header
    pub fn new(psid: u32, channel: u8, data_rate_mbps: f32, tx_power_dbm: i8) -> Self {
        let data_rate = (data_rate_mbps * 2.0) as u8; // units of 500 kbps
        let tx_power = (tx_power_dbm as i16 + 128) as u8;
        Self {
            subtype: 0,
            version: WSMP_VERSION,
            psid,
            channel_number: channel,
            data_rate,
            tx_power,
            channel_load: 0,
            wsm_length: 0,
        }
    }

    /// Encode WSMP header to bytes (simplified WSMP-N encoding)
    pub fn encode(&self, wsm_data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        // WSMP-N header fields
        out.push((self.subtype << 4) | (self.version & 0x0F));
        // PSID variable-length encoding (simplified 4-byte form)
        out.push(((self.psid >> 24) & 0xFF) as u8);
        out.push(((self.psid >> 16) & 0xFF) as u8);
        out.push(((self.psid >> 8) & 0xFF) as u8);
        out.push((self.psid & 0xFF) as u8);
        // Extension header for channel access (type=0x0F, len=3)
        out.push(0x0F);
        out.push(3);
        out.push(self.channel_number);
        out.push(self.data_rate);
        out.push(self.tx_power);
        // WSM length
        let wsm_len = wsm_data.len() as u16;
        out.push((wsm_len >> 8) as u8);
        out.push((wsm_len & 0xFF) as u8);
        // WSM data
        out.extend_from_slice(wsm_data);
        out
    }

    /// Decode WSMP header from bytes
    pub fn decode(data: &[u8]) -> Option<(Self, usize)> {
        if data.len() < 12 { return None; }
        let mut pos = 0;
        let subtype = data[pos] >> 4;
        let version = data[pos] & 0x0F;
        pos += 1;
        if version != WSMP_VERSION { return None; }
        let psid = u32::from_be_bytes([data[pos], data[pos+1], data[pos+2], data[pos+3]]);
        pos += 4;
        // Extension header
        let _ext_type = data[pos]; pos += 1;
        let _ext_len  = data[pos]; pos += 1;
        let channel_number = data[pos]; pos += 1;
        let data_rate = data[pos]; pos += 1;
        let tx_power = data[pos]; pos += 1;
        // WSM length
        if pos + 2 > data.len() { return None; }
        let wsm_length = u16::from_be_bytes([data[pos], data[pos+1]]);
        pos += 2;
        Some((Self { subtype, version, psid, channel_number, data_rate, tx_power,
                     channel_load: 0, wsm_length }, pos))
    }
}

// ---------------------------------------------------------------------------
// IEEE 1609.4 Multi-Channel Operation
// ---------------------------------------------------------------------------

/// Channel type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelType {
    /// Control Channel (CCH) – channel 178
    Cch,
    /// Service Channel (SCH) – channels 172,174,176,180,182,184
    Sch(u8),
}

impl ChannelType {
    pub fn channel_number(&self) -> u8 {
        match self {
            ChannelType::Cch => CCH_CHANNEL,
            ChannelType::Sch(n) => *n,
        }
    }

    pub fn center_frequency_hz(&self) -> f64 {
        let ch = self.channel_number() as f64;
        // f = 5000 MHz + ch * 5 MHz (ITU-R channel plan)
        (5000.0 + ch * 5.0) * 1_000_000.0
    }

    pub fn is_valid_sch(channel: u8) -> bool {
        SCH_CHANNELS.contains(&channel)
    }
}

/// Multi-channel operation state machine per IEEE 1609.4
#[derive(Debug, Clone)]
pub struct ChannelCoordinator {
    /// Current time in ms (modulo sync interval)
    pub time_ms: u64,
    /// Active channel
    pub active_channel: ChannelType,
    /// Alternate SCH for this device
    pub sch: u8,
}

impl ChannelCoordinator {
    pub fn new(sch: u8) -> Self {
        assert!(ChannelType::is_valid_sch(sch) || sch == CCH_CHANNEL,
                "Invalid SCH channel number");
        Self { time_ms: 0, active_channel: ChannelType::Cch, sch }
    }

    /// Advance time by `delta_ms` ms, returns true if channel switched
    pub fn advance(&mut self, delta_ms: u64) -> bool {
        let old = self.active_channel;
        self.time_ms = (self.time_ms + delta_ms) % SYNC_INTERVAL_MS;
        self.update_channel();
        old != self.active_channel
    }

    fn update_channel(&mut self) {
        self.active_channel = if self.time_ms < CCH_INTERVAL_MS {
            ChannelType::Cch
        } else {
            ChannelType::Sch(self.sch)
        };
    }

    /// True if currently in CCH interval
    pub fn is_cch_interval(&self) -> bool {
        self.time_ms < CCH_INTERVAL_MS
    }

    /// Time remaining in current interval (ms)
    pub fn remaining_ms(&self) -> u64 {
        if self.is_cch_interval() {
            CCH_INTERVAL_MS - self.time_ms
        } else {
            SYNC_INTERVAL_MS - self.time_ms
        }
    }
}

// ---------------------------------------------------------------------------
// SAE J2735 Basic Safety Message (BSM) Part I
// ---------------------------------------------------------------------------

/// BSM Part I data fields (SAE J2735)
#[derive(Debug, Clone, PartialEq)]
pub struct BsmPartI {
    /// Message count (0–127)
    pub msg_count: u8,
    /// Temporary ID (32-bit)
    pub temp_id: u32,
    /// DSEC of minute (0–65535)
    pub dsec: u16,
    /// Latitude in 1/10 µdeg (−900000000..900000000)
    pub lat: i32,
    /// Longitude in 1/10 µdeg (−1800000000..1800000000)
    pub lon: i32,
    /// Elevation in 0.1m (−4096..61439)
    pub elevation: i16,
    /// Speed in 0.02 m/s (0..8191)
    pub speed: u16,
    /// Heading in 0.0125 deg (0..28800)
    pub heading: u16,
    /// Steering wheel angle in 1.5 deg (−126..127)
    pub steer_angle: i8,
    /// Acceleration set (longitudinal m/s^2 * 100, -2000..2001)
    pub accel_long: i16,
    /// Lateral acceleration m/s^2 * 100
    pub accel_lat: i16,
    /// Vertical acceleration g * 100 (-127..127)
    pub accel_vert: i8,
    /// Yaw rate 0.01 deg/s (-32767..32767)
    pub yaw_rate: i16,
    /// Brakes applied bitmask (4 bits: wheel brakes, ABS, traction ctrl, stability ctrl)
    pub brake_status: u8,
    /// Vehicle width in 0.01m (0..511)
    pub vehicle_width: u16,
    /// Vehicle length in 0.01m (0..4095)
    pub vehicle_length: u16,
}

impl BsmPartI {
    /// Encode BSM Part I to bytes (simplified UPER-like packed encoding)
    /// Real J2735 uses ASN.1 UPER; this is a simplified fixed-width encoding.
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(30);
        // Message count (7 bits), temp_id (32 bits)
        buf.push(self.msg_count & 0x7F);
        buf.extend_from_slice(&self.temp_id.to_be_bytes());
        // DSEC (16 bits)
        buf.extend_from_slice(&self.dsec.to_be_bytes());
        // Lat (32 bits), Lon (32 bits)
        buf.extend_from_slice(&self.lat.to_be_bytes());
        buf.extend_from_slice(&self.lon.to_be_bytes());
        // Elevation (16 bits)
        buf.extend_from_slice(&(self.elevation as u16).to_be_bytes());
        // Speed (13 bits packed in 2 bytes)
        buf.extend_from_slice(&(self.speed & 0x1FFF).to_be_bytes());
        // Heading (15 bits packed in 2 bytes)
        buf.extend_from_slice(&(self.heading & 0x7FFF).to_be_bytes());
        // Steer angle (8 bits signed)
        buf.push(self.steer_angle as u8);
        // Accel long/lat/vert (16+16+8 bits)
        buf.extend_from_slice(&self.accel_long.to_be_bytes());
        buf.extend_from_slice(&self.accel_lat.to_be_bytes());
        buf.push(self.accel_vert as u8);
        // Yaw rate (16 bits)
        buf.extend_from_slice(&self.yaw_rate.to_be_bytes());
        // Brake status (8 bits)
        buf.push(self.brake_status);
        // Vehicle size (16+12 bits packed as 4 bytes)
        buf.extend_from_slice(&self.vehicle_width.to_be_bytes());
        let vlen = self.vehicle_length & 0x0FFF;
        buf.extend_from_slice(&vlen.to_be_bytes());
        buf
    }

    /// Decode BSM Part I from bytes
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 28 { return None; }
        let mut pos = 0;
        let msg_count = data[pos] & 0x7F; pos += 1;
        let temp_id = u32::from_be_bytes([data[pos], data[pos+1], data[pos+2], data[pos+3]]); pos += 4;
        let dsec = u16::from_be_bytes([data[pos], data[pos+1]]); pos += 2;
        let lat = i32::from_be_bytes([data[pos], data[pos+1], data[pos+2], data[pos+3]]); pos += 4;
        let lon = i32::from_be_bytes([data[pos], data[pos+1], data[pos+2], data[pos+3]]); pos += 4;
        let elevation = i16::from_be_bytes([data[pos], data[pos+1]]) as i16; pos += 2;
        let speed = u16::from_be_bytes([data[pos], data[pos+1]]) & 0x1FFF; pos += 2;
        let heading = u16::from_be_bytes([data[pos], data[pos+1]]) & 0x7FFF; pos += 2;
        let steer_angle = data[pos] as i8; pos += 1;
        let accel_long = i16::from_be_bytes([data[pos], data[pos+1]]); pos += 2;
        let accel_lat = i16::from_be_bytes([data[pos], data[pos+1]]); pos += 2;
        let accel_vert = data[pos] as i8; pos += 1;
        let yaw_rate = i16::from_be_bytes([data[pos], data[pos+1]]); pos += 2;
        let brake_status = data[pos]; pos += 1;
        let vehicle_width = u16::from_be_bytes([data[pos], data[pos+1]]); pos += 2;
        let vehicle_length = u16::from_be_bytes([data[pos], data[pos+1]]) & 0x0FFF;
        let _ = pos;
        Some(BsmPartI {
            msg_count, temp_id, dsec, lat, lon, elevation, speed, heading,
            steer_angle, accel_long, accel_lat, accel_vert, yaw_rate, brake_status,
            vehicle_width, vehicle_length,
        })
    }

    /// Latitude in decimal degrees
    pub fn lat_deg(&self) -> f64 { self.lat as f64 / 10_000_000.0 }
    /// Longitude in decimal degrees
    pub fn lon_deg(&self) -> f64 { self.lon as f64 / 10_000_000.0 }
    /// Speed in m/s
    pub fn speed_mps(&self) -> f64 { self.speed as f64 * 0.02 }
    /// Heading in degrees (0=North, 90=East)
    pub fn heading_deg(&self) -> f64 { self.heading as f64 * 0.0125 }
    /// Elevation in meters
    pub fn elevation_m(&self) -> f64 { self.elevation as f64 * 0.1 }
}

// ---------------------------------------------------------------------------
// Path Loss Models
// ---------------------------------------------------------------------------

/// 802.11p channel scenario
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelScenario {
    /// Highway Line-of-Sight
    HighwayLos,
    /// Urban NLOS
    UrbanNlos,
    /// Intersection scenario
    Intersection,
}

/// Dual-slope log-distance path loss model for 802.11p.
/// Returns path loss in dB.
///
/// L(d) = L(d0) + 10*n1*log10(d/d0)               for d <= d_break
/// L(d) = L(d_break) + 10*n2*log10(d/d_break)      for d > d_break
pub fn path_loss_dual_slope(
    distance_m: f64,
    freq_hz: f64,
    scenario: ChannelScenario,
) -> f64 {
    let c = 3e8_f64;
    let d0 = 1.0_f64;
    let lambda = c / freq_hz;
    // Free-space loss at d0 = 1m
    let l0 = 20.0 * (4.0 * PI * d0 / lambda).log10();

    let (n1, n2, d_break) = match scenario {
        ChannelScenario::HighwayLos  => (1.9, 3.8, 100.0),
        ChannelScenario::UrbanNlos   => (2.3, 3.5, 50.0),
        ChannelScenario::Intersection=> (2.0, 4.0, 60.0),
    };

    let d = distance_m.max(d0);
    if d <= d_break {
        l0 + 10.0 * n1 * (d / d0).log10()
    } else {
        let l_break = l0 + 10.0 * n1 * (d_break / d0).log10();
        l_break + 10.0 * n2 * (d / d_break).log10()
    }
}

/// Nakagami-m fading model: returns average received power in linear scale.
/// `path_loss_db`: path loss in dB.
/// `nakagami_m`: fading parameter (m=1 is Rayleigh, m→∞ is AWGN).
/// Returns received power in linear scale (normalized).
pub fn nakagami_fading_power(path_loss_db: f64, nakagami_m: f64) -> f64 {
    // Average power after path loss
    let p_avg = 10.0_f64.powf(-path_loss_db / 10.0);
    // Nakagami-m: mean power = p_avg, variance = p_avg^2 / m
    // Simplified: just return average (fading realization requires PRNG)
    p_avg * (1.0 + 1.0 / nakagami_m.max(0.5))
}

/// Received SNR estimate for 802.11p link.
/// `tx_power_dbm`: transmit power.
/// `path_loss_db`: path loss.
/// `noise_figure_db`: receiver noise figure.
/// `bandwidth_hz`: channel bandwidth.
/// Returns SNR in dB.
pub fn compute_snr_db(
    tx_power_dbm: f64,
    path_loss_db: f64,
    noise_figure_db: f64,
    bandwidth_hz: f64,
) -> f64 {
    let k_b = 1.38e-23_f64;
    let t0 = 290.0_f64;
    let noise_power_dbm = 10.0 * (k_b * t0 * bandwidth_hz).log10() + 30.0 + noise_figure_db;
    let rx_power_dbm = tx_power_dbm - path_loss_db;
    rx_power_dbm - noise_power_dbm
}

// ---------------------------------------------------------------------------
// DSRC Frame Builder (PPDU construction)
// ---------------------------------------------------------------------------

/// 802.11p PPDU (Physical Layer Convergence Procedure Data Unit)
#[derive(Debug, Clone)]
pub struct Ppdu {
    /// MCS index
    pub mcs_index: u8,
    /// PSDU payload bytes
    pub psdu: Vec<u8>,
}

impl Ppdu {
    pub fn new(mcs_index: u8, psdu: Vec<u8>) -> Self {
        assert!(mcs_index < 8, "Invalid MCS index");
        Self { mcs_index, psdu }
    }

    /// Build complete 802.11p time-domain waveform.
    /// Returns IQ samples: STS + LTS + SIGNAL + DATA symbols.
    pub fn build_waveform(&self) -> Vec<Complex> {
        let mcs = &MCS_TABLE[self.mcs_index as usize];
        let modulation = modulation_for_mcs(self.mcs_index);

        // Preamble: STS (160 samples) + LTS (160 samples)
        let mut waveform = generate_sts();
        waveform.extend(generate_lts());

        // SIGNAL field: encode, interleave, modulate (always BPSK r=1/2)
        let signal_field = SignalField {
            rate_code: SIGNAL_RATE_CODES[self.mcs_index as usize],
            length: self.psdu.len() as u16,
        };
        let signal_bits = signal_field.encode();
        let mut enc = ConvEncoder::new();
        let signal_coded = enc.encode_bits(&signal_bits);
        let interleaver = Interleaver::new(48, 1); // BPSK: n_cbps=48, n_bpsc=1
        let signal_interleaved = interleaver.interleave(&signal_coded[..48]);
        let signal_syms = map_bits(&signal_interleaved, Modulation::Bpsk);
        waveform.extend(ofdm_modulate_symbol(&signal_syms));

        // DATA field: scramble, encode, puncture, interleave, modulate
        let n_cbps = mcs.coded_bits_per_symbol as usize;
        let n_bpsc = mcs.bits_per_symbol as usize;
        let data_bits_per_sym = mcs.data_bits_per_symbol as usize;

        // Convert PSDU to bits (MSB first)
        let mut psdu_bits: Vec<u8> = self.psdu.iter().flat_map(|&b| {
            (0..8).rev().map(move |i| (b >> i) & 1)
        }).collect();

        // Scramble (IEEE 802.11 scrambler, polynomial x^7+x^4+1)
        scramble_bits(&mut psdu_bits, 0x7F);

        // Pad to multiple of data_bits_per_sym
        while psdu_bits.len() % data_bits_per_sym != 0 {
            psdu_bits.push(0);
        }

        // Encode + puncture
        let mut encoder = ConvEncoder::new();
        let puncture_pat = mcs.puncture_pattern();

        for chunk in psdu_bits.chunks(data_bits_per_sym) {
            let coded = encoder.encode_bits(chunk);
            let punctured = puncture(&coded, puncture_pat);

            // Pad/truncate to n_cbps
            let mut block = punctured;
            block.resize(n_cbps, 0);

            // Interleave
            let interleaver = Interleaver::new(n_cbps, n_bpsc);
            let interleaved = interleaver.interleave(&block);

            // Modulate
            let symbols = map_bits(&interleaved, modulation);
            // Pad to 48 symbols if needed
            let mut data_syms = symbols;
            data_syms.resize(NUM_DATA_SUBCARRIERS, Complex::zero());

            waveform.extend(ofdm_modulate_symbol(&data_syms));
        }

        waveform
    }

    /// Compute nominal PPDU duration in microseconds
    pub fn duration_us(&self) -> f64 {
        let mcs = &MCS_TABLE[self.mcs_index as usize];
        let data_bits_per_sym = mcs.data_bits_per_symbol as usize;
        let n_data_syms = (self.psdu.len() * 8 + data_bits_per_sym - 1) / data_bits_per_sym;
        let preamble_us = 40.0; // 16 µs STS + 8 µs LTS GI + 8+8 µs LTS = ~32 µs + SIGNAL 8 µs
        let signal_us = 8.0;
        let data_us = n_data_syms as f64 * 8.0; // 8 µs per OFDM symbol
        preamble_us + signal_us + data_us
    }
}

/// IEEE 802.11 scrambler / descrambler (polynomial x^7+x^4+1)
pub fn scramble_bits(bits: &mut [u8], init_state: u8) {
    let mut state = init_state & 0x7F;
    for bit in bits.iter_mut() {
        let feedback = ((state >> 6) ^ (state >> 3)) & 1;
        *bit ^= feedback as u8;
        state = ((state << 1) | feedback) & 0x7F;
    }
}

// ---------------------------------------------------------------------------
// DSRC Processor (top-level)
// ---------------------------------------------------------------------------

/// Top-level DSRC / 802.11p processor
#[derive(Debug, Clone)]
pub struct DsrcProcessor {
    /// Current channel coordinator
    pub channel_coord: ChannelCoordinator,
    /// Default MCS index
    pub default_mcs: u8,
    /// Default TX power in dBm
    pub tx_power_dbm: f64,
    /// Receiver noise figure in dB
    pub noise_figure_db: f64,
    /// Target SNR for rate selection
    pub target_snr_db: f32,
}

impl DsrcProcessor {
    /// Create with default settings (CCH 178, MCS=0, +23 dBm)
    pub fn new() -> Self {
        Self {
            channel_coord: ChannelCoordinator::new(SCH_CHANNELS[0]),
            default_mcs: 0,
            tx_power_dbm: 23.0,
            noise_figure_db: 6.0,
            target_snr_db: 15.0,
        }
    }

    /// Transmit BSM over WSMP on CCH
    pub fn transmit_bsm(&self, bsm: &BsmPartI) -> Vec<u8> {
        let bsm_bytes = bsm.encode();
        let wsmp = WsmpHeader::new(0x20, CCH_CHANNEL, 6.0, self.tx_power_dbm as i8);
        wsmp.encode(&bsm_bytes)
    }

    /// Select MCS based on link distance and scenario
    pub fn select_rate_for_distance(
        &self,
        distance_m: f64,
        scenario: ChannelScenario,
    ) -> &'static Mcs {
        let pl = path_loss_dual_slope(distance_m, CHANNEL_178_FREQ_HZ, scenario);
        let snr = compute_snr_db(self.tx_power_dbm, pl, self.noise_figure_db, CHANNEL_SPACING_HZ);
        select_mcs(snr as f32)
    }

    /// Build complete 802.11p PPDU waveform for a WSMP payload
    pub fn build_ppdu_waveform(&self, wsmp_payload: &[u8]) -> Vec<Complex> {
        let ppdu = Ppdu::new(self.default_mcs, wsmp_payload.to_vec());
        ppdu.build_waveform()
    }

    /// Estimate received SNR from IQ samples power (simple estimator)
    pub fn estimate_snr_from_samples(signal: &[Complex], noise: &[Complex]) -> f64 {
        let sig_power: f64 = signal.iter().map(|s| s.abs_sq()).sum::<f64>() / signal.len() as f64;
        let noise_power: f64 = if noise.is_empty() { 1e-10 }
            else { noise.iter().map(|s| s.abs_sq()).sum::<f64>() / noise.len() as f64 };
        10.0 * (sig_power / noise_power.max(1e-30)).log10()
    }
}

impl Default for DsrcProcessor {
    fn default() -> Self { Self::new() }
}

// ---------------------------------------------------------------------------
// CRC-32 (IEEE 802.11 FCS)
// ---------------------------------------------------------------------------

/// Compute CRC-32 per IEEE 802.11 (polynomial 0x04C11DB7, reflected)
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        let mut b = byte as u32;
        for _ in 0..8 {
            let mixed = (crc ^ b) & 1;
            crc >>= 1;
            if mixed != 0 {
                crc ^= 0xEDB8_8320; // reflected polynomial
            }
            b >>= 1;
        }
    }
    !crc
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Complex arithmetic ---

    #[test]
    fn test_complex_mul() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, 4.0);
        let c = a.mul(&b);
        assert!((c.re - (-5.0)).abs() < 1e-10);
        assert!((c.im - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_from_polar() {
        let c = Complex::from_polar(1.0, 0.0);
        assert!((c.re - 1.0).abs() < 1e-10);
        assert!(c.im.abs() < 1e-10);
    }

    #[test]
    fn test_complex_conj() {
        let c = Complex::new(3.0, -4.0);
        let conj = c.conj();
        assert_eq!(conj.re, 3.0);
        assert_eq!(conj.im, 4.0);
    }

    // --- FFT ---

    #[test]
    fn test_fft_ifft_roundtrip() {
        let mut input = vec![Complex::zero(); 64];
        input[1] = Complex::new(1.0, 0.0);
        let spectrum = fft(&input);
        let recovered = ifft(&spectrum);
        for (a, b) in input.iter().zip(recovered.iter()) {
            assert!((a.re - b.re).abs() < 1e-10, "re mismatch: {} vs {}", a.re, b.re);
            assert!((a.im - b.im).abs() < 1e-10, "im mismatch: {} vs {}", a.im, b.im);
        }
    }

    #[test]
    fn test_fft_dc() {
        // DC input: FFT should give N at bin 0, 0 elsewhere
        let n = 64;
        let input = vec![Complex::new(1.0, 0.0); n];
        let out = fft(&input);
        assert!((out[0].re - n as f64).abs() < 1e-8);
        for i in 1..n {
            assert!(out[i].abs() < 1e-8, "bin {} should be zero", i);
        }
    }

    #[test]
    fn test_fft_size_is_64() {
        assert_eq!(FFT_SIZE, 64);
    }

    #[test]
    fn test_bit_reverse() {
        assert_eq!(bit_reverse(0b1010, 4), 0b0101);
        assert_eq!(bit_reverse(0b0000, 4), 0b0000);
        assert_eq!(bit_reverse(0b1111, 4), 0b1111);
    }

    // --- MCS table ---

    #[test]
    fn test_mcs_table_count() {
        assert_eq!(MCS_TABLE.len(), 8);
    }

    #[test]
    fn test_mcs_data_rates() {
        let rates = [3.0_f32, 4.5, 6.0, 9.0, 12.0, 18.0, 24.0, 27.0];
        for (i, &r) in rates.iter().enumerate() {
            assert_eq!(MCS_TABLE[i].data_rate_mbps, r, "MCS {} rate mismatch", i);
        }
    }

    #[test]
    fn test_select_mcs_low_snr() {
        let mcs = select_mcs(2.0);
        assert_eq!(mcs.index, 0); // BPSK 1/2 at 3 Mbps
    }

    #[test]
    fn test_select_mcs_high_snr() {
        let mcs = select_mcs(25.0);
        assert_eq!(mcs.index, 7); // 64-QAM 3/4 at 27 Mbps
    }

    #[test]
    fn test_mcs_by_index() {
        for i in 0..8 {
            let m = mcs_by_index(i).unwrap();
            assert_eq!(m.index, i);
        }
        assert!(mcs_by_index(8).is_none());
    }

    #[test]
    fn test_mcs_puncture_patterns() {
        assert_eq!(MCS_TABLE[0].puncture_pattern(), &[1, 1]);        // 1/2
        assert_eq!(MCS_TABLE[1].puncture_pattern(), &[1, 1, 0, 1, 1, 0]); // 3/4
        assert_eq!(MCS_TABLE[6].puncture_pattern(), &[1, 1, 0, 1]); // 2/3
    }

    // --- Pilot subcarrier indices ---

    #[test]
    fn test_data_subcarrier_count() {
        assert_eq!(data_subcarrier_indices().len(), NUM_DATA_SUBCARRIERS);
    }

    #[test]
    fn test_pilot_indices_not_in_data() {
        let data = data_subcarrier_indices();
        for &p in &PILOT_INDICES {
            assert!(!data.contains(&p), "Pilot {} in data subcarriers", p);
        }
    }

    #[test]
    fn test_no_dc_subcarrier() {
        let data = data_subcarrier_indices();
        assert!(!data.contains(&0));
    }

    // --- SIGNAL field ---

    #[test]
    fn test_signal_field_encode_decode_roundtrip() {
        let sig = SignalField { rate_code: SIGNAL_RATE_CODES[2], length: 100 };
        let bits = sig.encode();
        let recovered = SignalField::decode(&bits).unwrap();
        assert_eq!(recovered.rate_code, sig.rate_code);
        assert_eq!(recovered.length, sig.length);
    }

    #[test]
    fn test_signal_field_parity() {
        let sig = SignalField { rate_code: SIGNAL_RATE_CODES[0], length: 50 };
        let bits = sig.encode();
        // Tail bits should be zero
        for &b in &bits[18..24] {
            assert_eq!(b, 0, "Tail bits must be zero");
        }
    }

    #[test]
    fn test_signal_field_mcs_index() {
        for i in 0..8 {
            let sig = SignalField { rate_code: SIGNAL_RATE_CODES[i], length: 0 };
            assert_eq!(sig.mcs_index().unwrap(), i as u8);
        }
    }

    #[test]
    fn test_signal_field_bad_parity() {
        let sig = SignalField { rate_code: SIGNAL_RATE_CODES[0], length: 100 };
        let mut bits = sig.encode();
        bits[17] ^= 1; // flip parity
        assert!(SignalField::decode(&bits).is_none());
    }

    // --- Convolutional encoder ---

    #[test]
    fn test_conv_encoder_all_zeros() {
        let mut enc = ConvEncoder::new();
        let coded = enc.encode_bits(&[0, 0, 0, 0]);
        assert_eq!(coded, vec![0, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn test_conv_encoder_single_one() {
        let mut enc = ConvEncoder::new();
        let (c1, c2) = enc.encode_bit(1);
        // Input=1, state=0: reg=1<<6=64, G1=0x5B, G2=0x79
        let expected_c1 = (0b1000000u8 & CONV_G1).count_ones() as u8 & 1;
        let expected_c2 = (0b1000000u8 & CONV_G2).count_ones() as u8 & 1;
        assert_eq!(c1, expected_c1);
        assert_eq!(c2, expected_c2);
    }

    #[test]
    fn test_conv_encoder_reset() {
        let mut enc = ConvEncoder::new();
        enc.encode_bit(1);
        enc.reset();
        assert_eq!(enc.state, 0);
    }

    #[test]
    fn test_conv_encoder_rate() {
        let mut enc = ConvEncoder::new();
        let input = vec![1u8; 8];
        let coded = enc.encode_bits(&input);
        assert_eq!(coded.len(), 16); // rate 1/2
    }

    // --- Puncturing ---

    #[test]
    fn test_puncture_rate_half() {
        let coded = vec![1, 0, 1, 0, 1, 0];
        let p = puncture(&coded, &[1, 1]);
        assert_eq!(p, coded); // no puncturing for rate 1/2
    }

    #[test]
    fn test_puncture_rate_two_thirds() {
        let coded = vec![1, 0, 1, 0, 1, 0, 1, 0];
        let p = puncture(&coded, &[1, 1, 0, 1]);
        // Keep indices 0,1,3 per period: bits at 0,1,3,4,5,7
        assert_eq!(p.len(), 6);
    }

    #[test]
    fn test_depuncture_rate_two_thirds() {
        let tx = vec![1u8, 0, 1, 0, 1, 0];
        let rx = depuncture(&tx, &[1, 1, 0, 1]);
        // Every 3rd element in the pattern is 0 → erasure (127)
        assert!(rx.contains(&127));
    }

    // --- Interleaver ---

    #[test]
    fn test_interleaver_bpsk_roundtrip() {
        let il = Interleaver::new(48, 1);
        let bits: Vec<u8> = (0..48u8).map(|i| i & 1).collect();
        let interleaved = il.interleave(&bits);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(bits, deinterleaved);
    }

    #[test]
    fn test_interleaver_qam16_roundtrip() {
        let il = Interleaver::new(192, 4);
        let bits: Vec<u8> = (0..192u8).map(|i| i & 1).collect();
        let interleaved = il.interleave(&bits);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(bits, deinterleaved);
    }

    #[test]
    fn test_interleaver_permutes() {
        let il = Interleaver::new(48, 1);
        let bits: Vec<u8> = (0..48u8).map(|i| (i % 2) as u8).collect();
        let interleaved = il.interleave(&bits);
        // Just check it's a permutation (same elements, different order)
        let mut s1 = bits.clone(); s1.sort();
        let mut s2 = interleaved.clone(); s2.sort();
        assert_eq!(s1, s2);
    }

    // --- Constellation mapping ---

    #[test]
    fn test_bpsk_roundtrip() {
        for bit in 0..2 {
            let sym = bpsk_map(bit);
            assert_eq!(bpsk_demap(&sym), bit);
        }
    }

    #[test]
    fn test_qpsk_roundtrip() {
        for b0 in 0..2u8 {
            for b1 in 0..2u8 {
                let sym = qpsk_map(&[b0, b1]);
                let [rb0, rb1] = qpsk_demap(&sym);
                assert_eq!(rb0, b0);
                assert_eq!(rb1, b1);
            }
        }
    }

    #[test]
    fn test_qam16_power() {
        // Average power of 16-QAM should be ~1.0
        let bits_patterns: Vec<Vec<u8>> = (0..16)
            .map(|i| vec![(i>>3)&1, (i>>2)&1, (i>>1)&1, i&1])
            .collect();
        let power: f64 = bits_patterns.iter()
            .map(|b| qam16_map(b).abs_sq())
            .sum::<f64>() / 16.0;
        assert!((power - 1.0).abs() < 0.01, "16-QAM avg power = {}", power);
    }

    #[test]
    fn test_map_bits_bpsk() {
        let bits = vec![0u8, 1, 0, 1];
        let syms = map_bits(&bits, Modulation::Bpsk);
        assert_eq!(syms.len(), 4);
        assert!(syms[0].re < 0.0);
        assert!(syms[1].re > 0.0);
    }

    #[test]
    fn test_demap_symbols_bpsk_roundtrip() {
        let bits: Vec<u8> = vec![1, 0, 1, 1, 0, 0];
        let syms = map_bits(&bits, Modulation::Bpsk);
        let recovered = demap_symbols(&syms, Modulation::Bpsk);
        assert_eq!(bits, recovered);
    }

    // --- OFDM modulation ---

    #[test]
    fn test_ofdm_symbol_length() {
        let data = vec![Complex::new(1.0, 0.0); NUM_DATA_SUBCARRIERS];
        let samples = ofdm_modulate_symbol(&data);
        assert_eq!(samples.len(), SYMBOL_SAMPLES);
    }

    #[test]
    fn test_ofdm_symbol_length_constant() {
        assert_eq!(SYMBOL_SAMPLES, 80); // 64 + 16
    }

    #[test]
    fn test_ofdm_demodulate_flat_channel() {
        let data_syms: Vec<Complex> = (0..48)
            .map(|i| if i % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect();
        let samples = ofdm_modulate_symbol(&data_syms);
        let recovered = ofdm_demodulate_symbol(&samples, None);
        assert_eq!(recovered.len(), NUM_DATA_SUBCARRIERS);
        // Check first data subcarrier sign is preserved
        assert!((recovered[0].re - data_syms[0].re).abs() < 0.1);
    }

    // --- STS / LTS ---

    #[test]
    fn test_sts_length() {
        let sts = generate_sts();
        assert_eq!(sts.len(), STS_REPETITIONS * 16);
    }

    #[test]
    fn test_lts_length() {
        let lts = generate_lts();
        // 32 (GI) + 64 * 2 (symbols) = 160
        assert_eq!(lts.len(), 160);
    }

    // --- WSMP ---

    #[test]
    fn test_wsmp_encode_decode() {
        let hdr = WsmpHeader::new(0x20, CCH_CHANNEL, 6.0, 23);
        let payload = b"Hello V2X";
        let encoded = hdr.encode(payload);
        let (decoded_hdr, offset) = WsmpHeader::decode(&encoded).unwrap();
        assert_eq!(decoded_hdr.psid, 0x20);
        assert_eq!(decoded_hdr.channel_number, CCH_CHANNEL);
        let wsm_data = &encoded[offset..offset + decoded_hdr.wsm_length as usize];
        assert_eq!(wsm_data, payload);
    }

    #[test]
    fn test_wsmp_version() {
        let hdr = WsmpHeader::new(0x100, CCH_CHANNEL, 3.0, 20);
        let encoded = hdr.encode(b"test");
        let first_byte = encoded[0];
        assert_eq!(first_byte & 0x0F, WSMP_VERSION);
    }

    #[test]
    fn test_wsmp_data_rate_encoding() {
        let hdr = WsmpHeader::new(0x20, 178, 6.0, 23);
        assert_eq!(hdr.data_rate, 12); // 6.0 * 2 = 12
    }

    // --- Channel coordination ---

    #[test]
    fn test_channel_coord_initial_cch() {
        let coord = ChannelCoordinator::new(SCH_CHANNELS[0]);
        assert_eq!(coord.active_channel, ChannelType::Cch);
        assert!(coord.is_cch_interval());
    }

    #[test]
    fn test_channel_coord_switch_to_sch() {
        let mut coord = ChannelCoordinator::new(SCH_CHANNELS[0]);
        let switched = coord.advance(CCH_INTERVAL_MS);
        assert!(switched);
        assert_eq!(coord.active_channel, ChannelType::Sch(SCH_CHANNELS[0]));
    }

    #[test]
    fn test_channel_coord_full_cycle() {
        let mut coord = ChannelCoordinator::new(SCH_CHANNELS[0]);
        coord.advance(SYNC_INTERVAL_MS);
        // After full cycle, should be back at CCH
        assert_eq!(coord.active_channel, ChannelType::Cch);
    }

    #[test]
    fn test_channel_coord_remaining() {
        let coord = ChannelCoordinator::new(SCH_CHANNELS[0]);
        assert_eq!(coord.remaining_ms(), CCH_INTERVAL_MS);
    }

    #[test]
    fn test_channel_type_frequency() {
        let cch = ChannelType::Cch;
        let freq = cch.center_frequency_hz();
        // Channel 178 → 5000 + 178*5 = 5890 MHz
        assert_eq!(freq, 5_890_000_000.0);
    }

    #[test]
    fn test_sch_channels_valid() {
        for &ch in &SCH_CHANNELS {
            assert!(ChannelType::is_valid_sch(ch));
        }
        assert!(!ChannelType::is_valid_sch(178)); // CCH not a valid SCH
    }

    // --- BSM ---

    #[test]
    fn test_bsm_encode_decode_roundtrip() {
        let bsm = BsmPartI {
            msg_count: 42,
            temp_id: 0xDEADBEEF,
            dsec: 1000,
            lat: 377_500_000,   // 37.75 deg (San Francisco)
            lon: -1_224_000_000, // -122.4 deg
            elevation: 100,
            speed: 500,         // 10 m/s
            heading: 3600,      // 45 deg NE
            steer_angle: 0,
            accel_long: 0,
            accel_lat: 0,
            accel_vert: 0,
            yaw_rate: 0,
            brake_status: 0,
            vehicle_width: 200, // 2.0 m
            vehicle_length: 450, // 4.5 m
        };
        let encoded = bsm.encode();
        let decoded = BsmPartI::decode(&encoded).unwrap();
        assert_eq!(decoded.msg_count, bsm.msg_count);
        assert_eq!(decoded.temp_id, bsm.temp_id);
        assert_eq!(decoded.lat, bsm.lat);
        assert_eq!(decoded.lon, bsm.lon);
        assert_eq!(decoded.speed, bsm.speed);
        assert_eq!(decoded.heading, bsm.heading);
        assert_eq!(decoded.vehicle_length, bsm.vehicle_length);
    }

    #[test]
    fn test_bsm_unit_conversions() {
        let bsm = BsmPartI {
            msg_count: 0, temp_id: 0, dsec: 0,
            lat: 377_500_000, lon: -1_224_000_000,
            elevation: 100, speed: 500, heading: 7200,
            steer_angle: 0, accel_long: 0, accel_lat: 0, accel_vert: 0,
            yaw_rate: 0, brake_status: 0,
            vehicle_width: 200, vehicle_length: 450,
        };
        assert!((bsm.lat_deg() - 37.75).abs() < 0.001);
        assert!((bsm.lon_deg() - (-122.4)).abs() < 0.001);
        assert!((bsm.speed_mps() - 10.0).abs() < 0.01);
        assert!((bsm.heading_deg() - 90.0).abs() < 0.01);
        assert!((bsm.elevation_m() - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_bsm_decode_short_data() {
        assert!(BsmPartI::decode(&[0u8; 10]).is_none());
    }

    // --- Path loss ---

    #[test]
    fn test_path_loss_dual_slope_near() {
        // At 1 m: free-space loss
        let pl = path_loss_dual_slope(1.0, CHANNEL_178_FREQ_HZ, ChannelScenario::HighwayLos);
        // Free-space at 5.89 GHz, 1m ≈ 47.8 dB
        assert!(pl > 40.0 && pl < 60.0, "PL at 1m = {} dB", pl);
    }

    #[test]
    fn test_path_loss_increases_with_distance() {
        let pl1 = path_loss_dual_slope(50.0, CHANNEL_178_FREQ_HZ, ChannelScenario::HighwayLos);
        let pl2 = path_loss_dual_slope(500.0, CHANNEL_178_FREQ_HZ, ChannelScenario::HighwayLos);
        assert!(pl2 > pl1, "PL should increase with distance");
    }

    #[test]
    fn test_path_loss_urban_nlos_worse() {
        let d = 200.0;
        let pl_los  = path_loss_dual_slope(d, CHANNEL_178_FREQ_HZ, ChannelScenario::HighwayLos);
        let pl_nlos = path_loss_dual_slope(d, CHANNEL_178_FREQ_HZ, ChannelScenario::UrbanNlos);
        assert!(pl_nlos > pl_los, "Urban NLOS should have higher PL than Highway LOS");
    }

    #[test]
    fn test_compute_snr_db() {
        let snr = compute_snr_db(23.0, 70.0, 6.0, 10e6);
        assert!(snr > 0.0 && snr < 60.0, "SNR = {} dB", snr);
    }

    #[test]
    fn test_nakagami_fading() {
        let p = nakagami_fading_power(60.0, 1.0);
        assert!(p > 0.0);
        // Higher m → closer to 1x average
        let p_m10 = nakagami_fading_power(60.0, 10.0);
        let p_m1  = nakagami_fading_power(60.0, 1.0);
        // With m=10, variance is lower → power closer to mean than m=1
        // Just verify both positive
        assert!(p_m1 > 0.0 && p_m10 > 0.0);
    }

    // --- PPDU ---

    #[test]
    fn test_ppdu_duration_mcs0() {
        let ppdu = Ppdu::new(0, vec![0u8; 100]);
        let dur = ppdu.duration_us();
        // preamble (40) + signal (8) + data symbols * 8
        // 100*8 = 800 bits, 24 bits/sym → 34 syms → 272 µs
        assert!(dur > 200.0 && dur < 500.0, "Duration = {} µs", dur);
    }

    #[test]
    fn test_ppdu_duration_increases_with_payload() {
        let ppdu_short = Ppdu::new(2, vec![0u8; 10]);
        let ppdu_long  = Ppdu::new(2, vec![0u8; 100]);
        assert!(ppdu_long.duration_us() > ppdu_short.duration_us());
    }

    #[test]
    fn test_ppdu_waveform_not_empty() {
        let ppdu = Ppdu::new(0, vec![0xABu8; 10]);
        let waveform = ppdu.build_waveform();
        assert!(!waveform.is_empty());
    }

    #[test]
    fn test_ppdu_higher_mcs_shorter() {
        // Higher MCS → more bits per symbol → fewer symbols → shorter
        let ppdu_low  = Ppdu::new(0, vec![0u8; 100]);
        let ppdu_high = Ppdu::new(7, vec![0u8; 100]);
        assert!(ppdu_high.duration_us() < ppdu_low.duration_us(),
            "High MCS ({} µs) should be shorter than low MCS ({} µs)",
            ppdu_high.duration_us(), ppdu_low.duration_us());
    }

    // --- Scrambler ---

    #[test]
    fn test_scrambler_self_inverse() {
        let original = vec![1u8, 0, 1, 1, 0, 0, 1, 0];
        let mut bits = original.clone();
        scramble_bits(&mut bits, 0x7F);
        scramble_bits(&mut bits, 0x7F);
        assert_eq!(bits, original);
    }

    #[test]
    fn test_scrambler_changes_bits() {
        let mut bits = vec![0u8; 16];
        scramble_bits(&mut bits, 0x7F);
        // Not all zeros after scrambling
        assert!(bits.iter().any(|&b| b != 0));
    }

    // --- DSRC Processor ---

    #[test]
    fn test_dsrc_processor_default() {
        let proc = DsrcProcessor::new();
        assert_eq!(proc.default_mcs, 0);
    }

    #[test]
    fn test_dsrc_transmit_bsm() {
        let proc = DsrcProcessor::new();
        let bsm = BsmPartI {
            msg_count: 1, temp_id: 0xAABBCCDD, dsec: 500,
            lat: 400_000_000, lon: -750_000_000,
            elevation: 0, speed: 200, heading: 0,
            steer_angle: 0, accel_long: 0, accel_lat: 0, accel_vert: 0,
            yaw_rate: 0, brake_status: 0,
            vehicle_width: 180, vehicle_length: 480,
        };
        let wsmp_bytes = proc.transmit_bsm(&bsm);
        assert!(!wsmp_bytes.is_empty());
        // Channel in header should be CCH
        let (hdr, _) = WsmpHeader::decode(&wsmp_bytes).unwrap();
        assert_eq!(hdr.channel_number, CCH_CHANNEL);
    }

    #[test]
    fn test_dsrc_select_rate_close() {
        let proc = DsrcProcessor::new();
        let mcs = proc.select_rate_for_distance(10.0, ChannelScenario::HighwayLos);
        // At close range, SNR is high → should select high MCS
        assert!(mcs.index >= 4);
    }

    #[test]
    fn test_dsrc_select_rate_far() {
        let proc = DsrcProcessor::new();
        let mcs = proc.select_rate_for_distance(800.0, ChannelScenario::UrbanNlos);
        // At long range/NLOS, SNR is low → should select low MCS
        assert!(mcs.index <= 4);
    }

    #[test]
    fn test_snr_estimator() {
        let signal = vec![Complex::new(1.0, 0.0); 100];
        let noise = vec![Complex::new(0.1, 0.0); 100];
        let snr = DsrcProcessor::estimate_snr_from_samples(&signal, &noise);
        assert!((snr - 20.0).abs() < 0.1, "Expected SNR ≈ 20 dB, got {} dB", snr);
    }

    // --- CRC-32 ---

    #[test]
    fn test_crc32_known_value() {
        // CRC32 of b"123456789" = 0xCBF43926
        let crc = crc32(b"123456789");
        assert_eq!(crc, 0xCBF43926, "CRC32 mismatch: {:#010X}", crc);
    }

    #[test]
    fn test_crc32_empty() {
        let crc = crc32(b"");
        assert_eq!(crc, 0x00000000);
    }

    #[test]
    fn test_crc32_consistency() {
        let data = b"Hello DSRC!";
        assert_eq!(crc32(data), crc32(data));
    }

    // --- Modulation for MCS ---

    #[test]
    fn test_modulation_for_mcs() {
        assert_eq!(modulation_for_mcs(0), Modulation::Bpsk);
        assert_eq!(modulation_for_mcs(1), Modulation::Bpsk);
        assert_eq!(modulation_for_mcs(2), Modulation::Qpsk);
        assert_eq!(modulation_for_mcs(3), Modulation::Qpsk);
        assert_eq!(modulation_for_mcs(4), Modulation::Qam16);
        assert_eq!(modulation_for_mcs(5), Modulation::Qam16);
        assert_eq!(modulation_for_mcs(6), Modulation::Qam64);
        assert_eq!(modulation_for_mcs(7), Modulation::Qam64);
    }

    // --- Channel estimation ---

    #[test]
    fn test_channel_estimation_returns_48() {
        let data = vec![Complex::new(1.0, 0.0); NUM_DATA_SUBCARRIERS];
        let samples = ofdm_modulate_symbol(&data);
        let h = estimate_channel_from_pilots(&samples);
        assert_eq!(h.len(), NUM_DATA_SUBCARRIERS);
    }

    // --- Integration: encode and decode WSMP + BSM ---

    #[test]
    fn test_wsmp_bsm_roundtrip() {
        let proc = DsrcProcessor::new();
        let bsm = BsmPartI {
            msg_count: 7, temp_id: 0x12345678, dsec: 999,
            lat: 510_000_000, lon: 0,
            elevation: 50, speed: 100, heading: 14400,
            steer_angle: -10, accel_long: 10, accel_lat: -5, accel_vert: 0,
            yaw_rate: 100, brake_status: 0x08,
            vehicle_width: 220, vehicle_length: 500,
        };
        let wsmp_bytes = proc.transmit_bsm(&bsm);
        let (hdr, offset) = WsmpHeader::decode(&wsmp_bytes).unwrap();
        let wsm_data = &wsmp_bytes[offset..offset + hdr.wsm_length as usize];
        let recovered_bsm = BsmPartI::decode(wsm_data).unwrap();
        assert_eq!(recovered_bsm.temp_id, bsm.temp_id);
        assert_eq!(recovered_bsm.lat, bsm.lat);
        assert_eq!(recovered_bsm.speed, bsm.speed);
        assert_eq!(recovered_bsm.brake_status, bsm.brake_status);
    }
}

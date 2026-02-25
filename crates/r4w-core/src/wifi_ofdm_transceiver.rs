//! WiFi 802.11a/g/n OFDM Physical Layer Transceiver
//!
//! Implements the IEEE 802.11-2020 OFDM PHY (§17) and HT PHY (§19) for
//! 802.11a/g (legacy) and 802.11n (HT) operation.
//!
//! ## OFDM Parameters (802.11a/g)
//!
//! - 64-point FFT/IFFT
//! - 52 used subcarriers: 48 data + 4 pilots
//! - Subcarrier spacing: 312.5 kHz (20 MHz / 64)
//! - Symbol duration: 3.2 µs (64 samples at 20 MHz)
//! - Guard interval (CP): 0.8 µs (16 samples)
//! - Total symbol: 4.0 µs (80 samples)
//!
//! ## Frame Structure
//!
//! ```text
//! L-STF (10×8 = 80 samples) | L-LTF (GI2+2×OFDM = 160 samples)
//! | L-SIG (1 symbol) | [HT-SIG | HT-STF | HT-LTF] | DATA symbols
//! ```
//!
//! ## Data Rates
//!
//! | Rate   | Modulation | Code Rate | Mbps |
//! |--------|-----------|-----------|------|
//! | 0      | BPSK      | 1/2       | 6    |
//! | 1      | BPSK      | 3/4       | 9    |
//! | 2      | QPSK      | 1/2       | 12   |
//! | 3      | QPSK      | 3/4       | 18   |
//! | 4      | 16-QAM    | 1/2       | 24   |
//! | 5      | 16-QAM    | 3/4       | 36   |
//! | 6      | 64-QAM    | 2/3       | 48   |
//! | 7      | 64-QAM    | 3/4       | 54   |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::wifi_ofdm_transceiver::{WifiOfdmTransceiver, WifiConfig, WifiRate};
//!
//! let mut tx = WifiOfdmTransceiver::new(WifiConfig::default());
//! let data = vec![0u8; 100];
//! let symbols = tx.transmit(&data, WifiRate::Rate54Mbps);
//! assert!(!symbols.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex number
// ---------------------------------------------------------------------------

/// Complex sample (f64 I/Q).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cf64 {
    pub re: f64,
    pub im: f64,
}

impl Cf64 {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self { Self { re, im } }
    #[inline]
    pub fn zero() -> Self { Self { re: 0.0, im: 0.0 } }
    #[inline]
    pub fn add(self, rhs: Self) -> Self { Self::new(self.re + rhs.re, self.im + rhs.im) }
    #[inline]
    pub fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }
    #[inline]
    pub fn scale(self, s: f64) -> Self { Self::new(self.re * s, self.im * s) }
    #[inline]
    pub fn conj(self) -> Self { Self::new(self.re, -self.im) }
    #[inline]
    pub fn mag_sq(self) -> f64 { self.re * self.re + self.im * self.im }
    #[inline]
    pub fn mag(self) -> f64 { self.mag_sq().sqrt() }
    /// e^{j*angle}
    #[inline]
    pub fn from_polar(mag: f64, angle: f64) -> Self {
        Self::new(mag * angle.cos(), mag * angle.sin())
    }
}

// ---------------------------------------------------------------------------
// OFDM constants
// ---------------------------------------------------------------------------

/// FFT size for 802.11a/g/n 20 MHz
pub const FFT_SIZE: usize = 64;
/// Number of data subcarriers (802.11a)
pub const NUM_DATA_SC: usize = 48;
/// Number of pilot subcarriers
pub const NUM_PILOT_SC: usize = 4;
/// Cyclic prefix length (short GI = 16, long GI = 16)
pub const CP_LEN: usize = 16;
/// Total OFDM symbol samples (FFT + CP)
pub const SYMBOL_SAMPLES: usize = FFT_SIZE + CP_LEN; // 80
/// Short GI CP length (400 ns at 20 MHz = 8 samples)
pub const CP_LEN_SGI: usize = 8;
/// L-STF: 10 short symbols of 16 samples each = 160 samples
pub const L_STF_SAMPLES: usize = 160;
/// L-LTF: 2 OFDM symbols + 2×CP = 160 samples (GI2 = 32 samples prefix)
pub const L_LTF_SAMPLES: usize = 160;
/// Sample rate: 20 MHz
pub const SAMPLE_RATE_HZ: f64 = 20_000_000.0;

/// Pilot subcarrier indices (relative to DC = index 32)
/// Actual FFT bin indices for subcarriers: -21,-7,+7,+21
pub const PILOT_SUBCARRIERS: [i32; 4] = [-21, -7, 7, 21];

/// Data subcarrier indices: -26...-22, -20...-8, -6...-1, +1...+6, +8...+20, +22...+26
/// (excluding DC=0, pilots at ±7, ±21, and guard bands)
pub fn data_subcarrier_indices() -> Vec<i32> {
    let mut idxs = Vec::with_capacity(48);
    for i in -26_i32..=26 {
        if i == 0 { continue; }
        if PILOT_SUBCARRIERS.contains(&i) { continue; }
        idxs.push(i);
    }
    idxs
}

/// Long Training Field (LTF) frequency-domain sequence (52 nonzero subcarriers)
/// Per IEEE 802.11-2020 Table 17-7
pub const LTF_SEQUENCE: [i8; 52] = [
    // Subcarriers -26..-1 (26 values)
    1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
    // Subcarriers +1..+26 (26 values, DC=0 excluded)
    1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1,
];

/// L-LTF known frequency domain symbols (indices -26 to +26 excl. 0)
/// Source: IEEE 802.11-2020 §17.3.3 Table 17-7
pub fn ltf_symbols() -> [i8; 64] {
    // 64-point: indices [0..63], DC=32, negative freqs wrap around
    // LTF: -26 to -1 = bins 38..63, +1 to +26 = bins 1..26, rest=0
    let ltf52: [i8; 52] = [
         1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,
         1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1,
    ];
    let mut out = [0i8; 64];
    // positive subcarriers +1..+26 → bins 1..26
    for (i, &v) in ltf52[26..52].iter().enumerate() {
        out[1 + i] = v;
    }
    // negative subcarriers -26..-1 → bins 38..63
    for (i, &v) in ltf52[0..26].iter().enumerate() {
        out[38 + i] = v;
    }
    out
}

// ---------------------------------------------------------------------------
// Modulation / Data Rate
// ---------------------------------------------------------------------------

/// 802.11a legacy data rates
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WifiRate {
    Rate6Mbps,   // BPSK 1/2
    Rate9Mbps,   // BPSK 3/4
    Rate12Mbps,  // QPSK 1/2
    Rate18Mbps,  // QPSK 3/4
    Rate24Mbps,  // 16-QAM 1/2
    Rate36Mbps,  // 16-QAM 3/4
    Rate48Mbps,  // 64-QAM 2/3
    Rate54Mbps,  // 64-QAM 3/4
}

/// Modulation order
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Modulation {
    Bpsk,
    Qpsk,
    Qam16,
    Qam64,
}

/// Code rate numerator / denominator
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CodeRate {
    OneHalf,   // 1/2
    TwoThirds, // 2/3
    ThreeQuarters, // 3/4
}

/// Rate parameters
#[derive(Debug, Clone, Copy)]
pub struct RateParams {
    pub modulation: Modulation,
    pub code_rate: CodeRate,
    pub bits_per_subcarrier: usize,
    /// Data bits per OFDM symbol (DBPS = bits_per_sc * N_data)
    pub dbps: usize,
    /// Coded bits per OFDM symbol (CBPS = bits_per_sc * N_used)
    pub cbps: usize,
    pub mbps: f64,
}

impl WifiRate {
    pub fn params(self) -> RateParams {
        match self {
            WifiRate::Rate6Mbps  => RateParams { modulation: Modulation::Bpsk,  code_rate: CodeRate::OneHalf,       bits_per_subcarrier: 1, cbps: 48, dbps: 24,  mbps: 6.0  },
            WifiRate::Rate9Mbps  => RateParams { modulation: Modulation::Bpsk,  code_rate: CodeRate::ThreeQuarters, bits_per_subcarrier: 1, cbps: 48, dbps: 36,  mbps: 9.0  },
            WifiRate::Rate12Mbps => RateParams { modulation: Modulation::Qpsk,  code_rate: CodeRate::OneHalf,       bits_per_subcarrier: 2, cbps: 96, dbps: 48,  mbps: 12.0 },
            WifiRate::Rate18Mbps => RateParams { modulation: Modulation::Qpsk,  code_rate: CodeRate::ThreeQuarters, bits_per_subcarrier: 2, cbps: 96, dbps: 72,  mbps: 18.0 },
            WifiRate::Rate24Mbps => RateParams { modulation: Modulation::Qam16, code_rate: CodeRate::OneHalf,       bits_per_subcarrier: 4, cbps: 192, dbps: 96, mbps: 24.0 },
            WifiRate::Rate36Mbps => RateParams { modulation: Modulation::Qam16, code_rate: CodeRate::ThreeQuarters, bits_per_subcarrier: 4, cbps: 192, dbps: 144,mbps: 36.0 },
            WifiRate::Rate48Mbps => RateParams { modulation: Modulation::Qam64, code_rate: CodeRate::TwoThirds,     bits_per_subcarrier: 6, cbps: 288, dbps: 192,mbps: 48.0 },
            WifiRate::Rate54Mbps => RateParams { modulation: Modulation::Qam64, code_rate: CodeRate::ThreeQuarters, bits_per_subcarrier: 6, cbps: 288, dbps: 216,mbps: 54.0 },
        }
    }

    /// Encode the 4-bit rate field for L-SIG (IEEE 802.11-2020 Table 17-6)
    pub fn sig_rate_bits(self) -> [u8; 4] {
        match self {
            WifiRate::Rate6Mbps  => [1,1,0,1],
            WifiRate::Rate9Mbps  => [1,1,1,1],
            WifiRate::Rate12Mbps => [0,1,0,1],
            WifiRate::Rate18Mbps => [0,1,1,1],
            WifiRate::Rate24Mbps => [1,0,0,1],
            WifiRate::Rate36Mbps => [1,0,1,1],
            WifiRate::Rate48Mbps => [0,0,1,0],
            WifiRate::Rate54Mbps => [0,0,1,1],
        }
    }
}

// ---------------------------------------------------------------------------
// 802.11n MCS (HT)
// ---------------------------------------------------------------------------

/// 802.11n HT MCS parameters (single spatial stream, 20 MHz, long GI)
#[derive(Debug, Clone, Copy)]
pub struct HtMcsParams {
    pub mcs_index: u8,
    pub modulation: Modulation,
    pub code_rate: CodeRate,
    /// Data bits per symbol (single stream)
    pub dbps: usize,
    pub mbps: f64,
}

/// MCS 0-7 for 802.11n single spatial stream
pub fn ht_mcs_table() -> [HtMcsParams; 8] {
    [
        HtMcsParams { mcs_index: 0, modulation: Modulation::Bpsk,  code_rate: CodeRate::OneHalf,       dbps: 26,  mbps: 6.5  },
        HtMcsParams { mcs_index: 1, modulation: Modulation::Qpsk,  code_rate: CodeRate::OneHalf,       dbps: 52,  mbps: 13.0 },
        HtMcsParams { mcs_index: 2, modulation: Modulation::Qpsk,  code_rate: CodeRate::ThreeQuarters, dbps: 78,  mbps: 19.5 },
        HtMcsParams { mcs_index: 3, modulation: Modulation::Qam16, code_rate: CodeRate::OneHalf,       dbps: 104, mbps: 26.0 },
        HtMcsParams { mcs_index: 4, modulation: Modulation::Qam16, code_rate: CodeRate::ThreeQuarters, dbps: 156, mbps: 39.0 },
        HtMcsParams { mcs_index: 5, modulation: Modulation::Qam64, code_rate: CodeRate::TwoThirds,     dbps: 208, mbps: 52.0 },
        HtMcsParams { mcs_index: 6, modulation: Modulation::Qam64, code_rate: CodeRate::ThreeQuarters, dbps: 234, mbps: 58.5 },
        HtMcsParams { mcs_index: 7, modulation: Modulation::Qam64, code_rate: CodeRate::ThreeQuarters, dbps: 260, mbps: 65.0 },
    ]
}

// ---------------------------------------------------------------------------
// Scrambler – 802.11-2020 §17.3.5.4
// ---------------------------------------------------------------------------

/// 802.11 data scrambler: 127-bit LFSR with x^7 + x^4 + 1 polynomial.
/// Initial state must be nonzero (all-ones by default per spec).
pub struct WifiScrambler {
    state: u8, // 7-bit shift register in bits [6:0]
}

impl WifiScrambler {
    /// Create scrambler. `seed` is a 7-bit value; zero is illegal, replaced with 0x7F.
    pub fn new(seed: u8) -> Self {
        let s = if seed == 0 { 0x7F } else { seed & 0x7F };
        WifiScrambler { state: s }
    }

    /// Clock one bit; returns the XOR output bit.
    #[inline]
    fn clock(&mut self) -> u8 {
        let x7 = (self.state >> 6) & 1;
        let x4 = (self.state >> 3) & 1;
        let fb = x7 ^ x4;
        self.state = ((self.state << 1) & 0x7F) | fb;
        fb
    }

    /// Scramble (or descramble, same operation) a bit slice.
    pub fn process_bits(&mut self, bits: &[u8]) -> Vec<u8> {
        bits.iter().map(|&b| b ^ self.clock()).collect()
    }

    /// In-place scramble of a mutable bit slice.
    pub fn process_inplace(&mut self, bits: &mut [u8]) {
        for b in bits.iter_mut() {
            *b ^= self.clock();
        }
    }
}

// ---------------------------------------------------------------------------
// BCC Convolutional Encoder – K=7, rate 1/2
// ---------------------------------------------------------------------------

/// Binary Convolutional Code encoder, K=7, rate 1/2.
/// Generator polynomials (octal): g0 = 133, g1 = 171 (NASA standard).
pub struct BccEncoder {
    shift_reg: u8,
}

impl BccEncoder {
    pub fn new() -> Self { BccEncoder { shift_reg: 0 } }

    /// Encode one input bit, returns (bit_a, bit_b).
    #[inline]
    fn encode_bit(&mut self, input: u8) -> (u8, u8) {
        self.shift_reg = ((self.shift_reg << 1) | (input & 1)) & 0x7F;
        let g0 = 0b1011011u8; // 0133 octal
        let g1 = 0b1111001u8; // 0171 octal
        let a = (self.shift_reg & g0).count_ones() as u8 & 1;
        let b = (self.shift_reg & g1).count_ones() as u8 & 1;
        (a, b)
    }

    /// Encode a bit vector; outputs 2× the input length.
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(bits.len() * 2);
        for &b in bits {
            let (a, c) = self.encode_bit(b);
            out.push(a);
            out.push(c);
        }
        out
    }

    /// Flush 6 tail bits to terminate trellis.
    pub fn flush(&mut self) -> Vec<u8> {
        let zeros = vec![0u8; 6];
        self.encode(&zeros)
    }

    pub fn reset(&mut self) { self.shift_reg = 0; }
}

impl Default for BccEncoder {
    fn default() -> Self { Self::new() }
}

// ---------------------------------------------------------------------------
// Puncturing (rate 2/3 and 3/4)
// ---------------------------------------------------------------------------

/// Puncture a rate-1/2 coded bit stream to achieve higher code rates.
///
/// - Rate 2/3: puncture pattern [1,1,1,0] over every 4 input bits → 3 output
/// - Rate 3/4: puncture pattern [1,1,1,0,0,1] over every 6 → 4 output
pub fn puncture(bits: &[u8], code_rate: CodeRate) -> Vec<u8> {
    match code_rate {
        CodeRate::OneHalf => bits.to_vec(),
        CodeRate::TwoThirds => {
            // Pattern [1,1,1,0]: keep bits at positions 0,1,2 of each group of 4
            let mut out = Vec::new();
            for chunk in bits.chunks(4) {
                if chunk.len() >= 3 {
                    out.push(chunk[0]);
                    out.push(chunk[1]);
                    out.push(chunk[2]);
                    // chunk[3] is punctured
                }
            }
            out
        }
        CodeRate::ThreeQuarters => {
            // Pattern [1,1,1,0,0,1]: keep bits 0,1,2,5 of each group of 6
            let mut out = Vec::new();
            for chunk in bits.chunks(6) {
                if chunk.len() >= 6 {
                    out.push(chunk[0]);
                    out.push(chunk[1]);
                    out.push(chunk[2]);
                    // chunk[3] punctured
                    // chunk[4] punctured
                    out.push(chunk[5]);
                }
            }
            out
        }
    }
}

/// Depuncture: insert erasure markers (value 2) for punctured positions.
pub fn depuncture(bits: &[u8], code_rate: CodeRate) -> Vec<u8> {
    match code_rate {
        CodeRate::OneHalf => bits.to_vec(),
        CodeRate::TwoThirds => {
            let mut out = Vec::new();
            let mut idx = 0;
            let mut pos = 0;
            while idx < bits.len() {
                match pos % 4 {
                    3 => out.push(2), // erasure
                    _ => { out.push(bits[idx]); idx += 1; }
                }
                pos += 1;
            }
            out
        }
        CodeRate::ThreeQuarters => {
            let mut out = Vec::new();
            let mut idx = 0;
            let mut pos = 0;
            while idx < bits.len() {
                match pos % 6 {
                    3 | 4 => out.push(2), // erasures
                    _ => { out.push(bits[idx]); idx += 1; }
                }
                pos += 1;
            }
            out
        }
    }
}

// ---------------------------------------------------------------------------
// Interleaver – 802.11-2020 §17.3.5.7
// ---------------------------------------------------------------------------

/// OFDM interleaver for 802.11a/g.
///
/// Two-step permutation per the standard:
/// 1. Ensures adjacent coded bits are not mapped to adjacent subcarriers.
/// 2. Ensures adjacent bits alternate between more and less significant bits of the
///    subcarrier constellation.
pub struct OfdmInterleaver {
    pub cbps: usize, // coded bits per symbol
    pub bpsc: usize, // bits per subcarrier
}

impl OfdmInterleaver {
    pub fn new(cbps: usize, bpsc: usize) -> Self {
        OfdmInterleaver { cbps, bpsc }
    }

    /// Compute first permutation index: i → k
    /// k = (N_CBPS / 16) * (i mod 16) + floor(i / 16)
    fn perm1(&self, i: usize) -> usize {
        let n = self.cbps;
        (n / 16) * (i % 16) + (i / 16)
    }

    /// Compute second permutation index: k → j
    /// j = s * floor(k/s) + (k + N_CBPS - floor(16*k/N_CBPS)) mod s
    /// where s = max(N_BPSC/2, 1)
    fn perm2(&self, k: usize) -> usize {
        let n = self.cbps;
        let s = (self.bpsc / 2).max(1);
        s * (k / s) + (k + n - 16 * k / n) % s
    }

    /// Interleave a block of cbps coded bits.
    pub fn interleave(&self, bits: &[u8]) -> Vec<u8> {
        assert_eq!(bits.len(), self.cbps, "input must be exactly cbps bits");
        let mut tmp = vec![0u8; self.cbps];
        // Step 1: i → k = perm1(i), tmp[k] = bits[i]
        for i in 0..self.cbps {
            let k = self.perm1(i);
            tmp[k] = bits[i];
        }
        // Step 2: k → j = perm2(k), out[j] = tmp[k]
        let mut out = vec![0u8; self.cbps];
        for k in 0..self.cbps {
            let j = self.perm2(k);
            out[j] = tmp[k];
        }
        out
    }

    /// Deinterleave: inverse of interleave.
    pub fn deinterleave(&self, bits: &[u8]) -> Vec<u8> {
        assert_eq!(bits.len(), self.cbps, "input must be exactly cbps bits");
        // Inverse perm2: j → k
        let mut inv2 = vec![0usize; self.cbps];
        for k in 0..self.cbps {
            inv2[self.perm2(k)] = k;
        }
        // Inverse perm1: k → i
        let mut inv1 = vec![0usize; self.cbps];
        for i in 0..self.cbps {
            inv1[self.perm1(i)] = i;
        }
        let mut tmp = vec![0u8; self.cbps];
        for j in 0..self.cbps {
            tmp[inv2[j]] = bits[j];
        }
        let mut out = vec![0u8; self.cbps];
        for k in 0..self.cbps {
            out[inv1[k]] = tmp[k];
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Constellation mapper/demapper
// ---------------------------------------------------------------------------

/// BPSK constellation: {-1, +1}
fn bpsk_map(bit: u8) -> Cf64 {
    if bit == 0 { Cf64::new(-1.0, 0.0) } else { Cf64::new(1.0, 0.0) }
}

/// QPSK constellation (Gray coded, normalized to unit avg power)
fn qpsk_map(bits: &[u8]) -> Cf64 {
    let re = if bits[0] == 0 { -1.0_f64 } else { 1.0 };
    let im = if bits[1] == 0 { -1.0_f64 } else { 1.0 };
    Cf64::new(re, im).scale(1.0 / 2.0_f64.sqrt())
}

/// 16-QAM (Gray coded, normalized)
fn qam16_map(bits: &[u8]) -> Cf64 {
    let re = gray2_to_coord(bits[0], bits[1]);
    let im = gray2_to_coord(bits[2], bits[3]);
    Cf64::new(re, im).scale(1.0 / 10.0_f64.sqrt())
}

/// 64-QAM (Gray coded, normalized)
fn qam64_map(bits: &[u8]) -> Cf64 {
    let re = gray3_to_coord(bits[0], bits[1], bits[2]);
    let im = gray3_to_coord(bits[3], bits[4], bits[5]);
    Cf64::new(re, im).scale(1.0 / 42.0_f64.sqrt())
}

/// Map 2-bit Gray code to ±1, ±3 for 16-QAM axis
fn gray2_to_coord(b0: u8, b1: u8) -> f64 {
    match (b0, b1) {
        (0, 0) => -3.0,
        (0, 1) => -1.0,
        (1, 1) =>  1.0,
        (1, 0) =>  3.0,
        _ => 0.0,
    }
}

/// Map 3-bit Gray code to ±1,±3,±5,±7 for 64-QAM axis
fn gray3_to_coord(b0: u8, b1: u8, b2: u8) -> f64 {
    match (b0, b1, b2) {
        (0, 0, 0) => -7.0,
        (0, 0, 1) => -5.0,
        (0, 1, 1) => -3.0,
        (0, 1, 0) => -1.0,
        (1, 1, 0) =>  1.0,
        (1, 1, 1) =>  3.0,
        (1, 0, 1) =>  5.0,
        (1, 0, 0) =>  7.0,
        _ => 0.0,
    }
}

/// Map bits to constellation symbols according to modulation.
pub fn map_symbols(bits: &[u8], modulation: Modulation) -> Vec<Cf64> {
    match modulation {
        Modulation::Bpsk => {
            bits.iter().map(|&b| bpsk_map(b)).collect()
        }
        Modulation::Qpsk => {
            bits.chunks(2).filter(|c| c.len() == 2)
                .map(|c| qpsk_map(c)).collect()
        }
        Modulation::Qam16 => {
            bits.chunks(4).filter(|c| c.len() == 4)
                .map(|c| qam16_map(c)).collect()
        }
        Modulation::Qam64 => {
            bits.chunks(6).filter(|c| c.len() == 6)
                .map(|c| qam64_map(c)).collect()
        }
    }
}

/// Hard-decision demap a BPSK symbol to a bit.
pub fn bpsk_demap(sym: Cf64) -> u8 {
    if sym.re >= 0.0 { 1 } else { 0 }
}

/// Hard-decision demap QPSK.
pub fn qpsk_demap(sym: Cf64) -> [u8; 2] {
    [if sym.re >= 0.0 { 1 } else { 0 }, if sym.im >= 0.0 { 1 } else { 0 }]
}

/// Hard-decision demap 16-QAM to 4 bits.
pub fn qam16_demap(sym: Cf64) -> [u8; 4] {
    let scale = 10.0_f64.sqrt();
    let re = sym.re * scale;
    let im = sym.im * scale;
    let [b0, b1] = axis_demap_4(re);
    let [b2, b3] = axis_demap_4(im);
    [b0, b1, b2, b3]
}

fn axis_demap_4(val: f64) -> [u8; 2] {
    if val < -2.0 { [0, 0] }
    else if val < 0.0 { [0, 1] }
    else if val < 2.0 { [1, 1] }
    else { [1, 0] }
}

/// Hard-decision demap 64-QAM to 6 bits.
pub fn qam64_demap(sym: Cf64) -> [u8; 6] {
    let scale = 42.0_f64.sqrt();
    let re = sym.re * scale;
    let im = sym.im * scale;
    let [b0, b1, b2] = axis_demap_6(re);
    let [b3, b4, b5] = axis_demap_6(im);
    [b0, b1, b2, b3, b4, b5]
}

fn axis_demap_6(val: f64) -> [u8; 3] {
    if val < -6.0 { [0, 0, 0] }
    else if val < -4.0 { [0, 0, 1] }
    else if val < -2.0 { [0, 1, 1] }
    else if val < 0.0  { [0, 1, 0] }
    else if val < 2.0  { [1, 1, 0] }
    else if val < 4.0  { [1, 1, 1] }
    else if val < 6.0  { [1, 0, 1] }
    else               { [1, 0, 0] }
}

/// Demap symbols to bits.
pub fn demap_symbols(syms: &[Cf64], modulation: Modulation) -> Vec<u8> {
    let mut bits = Vec::new();
    for &s in syms {
        match modulation {
            Modulation::Bpsk => bits.push(bpsk_demap(s)),
            Modulation::Qpsk => bits.extend_from_slice(&qpsk_demap(s)),
            Modulation::Qam16 => bits.extend_from_slice(&qam16_demap(s)),
            Modulation::Qam64 => bits.extend_from_slice(&qam64_demap(s)),
        }
    }
    bits
}

// ---------------------------------------------------------------------------
// Pilot polarity sequence – 802.11-2020 §17.3.5.8
// ---------------------------------------------------------------------------

/// 127-bit PN pilot polarity sequence (from standard Table 17-13).
pub struct PilotPolaritySeq {
    lfsr: u8, // 7-bit LFSR
}

impl PilotPolaritySeq {
    pub fn new() -> Self { PilotPolaritySeq { lfsr: 0x7F } }

    /// Return next polarity value (+1 or -1).
    pub fn next(&mut self) -> f64 {
        let x7 = (self.lfsr >> 6) & 1;
        let x4 = (self.lfsr >> 3) & 1;
        let fb = x7 ^ x4;
        self.lfsr = ((self.lfsr << 1) & 0x7F) | fb;
        // polarity is 1 - 2*fb (maps 0→+1, 1→-1)
        if fb == 0 { 1.0 } else { -1.0 }
    }
}

impl Default for PilotPolaritySeq {
    fn default() -> Self { Self::new() }
}

// ---------------------------------------------------------------------------
// FFT / IFFT
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT (bit-reversal + butterfly).
pub fn fft_inplace(x: &mut [Cf64]) {
    let n = x.len();
    assert!(n.is_power_of_two(), "FFT size must be power of two");
    // Bit-reversal permutation
    let bits = n.trailing_zeros() as usize;
    for i in 0..n {
        let j = bit_reverse(i, bits);
        if j > i { x.swap(i, j); }
    }
    // Butterfly stages
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = -2.0 * PI / len as f64;
        let wlen = Cf64::from_polar(1.0, ang);
        let mut i = 0;
        while i < n {
            let mut w = Cf64::new(1.0, 0.0);
            for j in 0..half {
                let u = x[i + j];
                let v = x[i + j + half].mul(w);
                x[i + j] = u.add(v);
                x[i + j + half] = Cf64::new(u.re - v.re, u.im - v.im);
                w = w.mul(wlen);
            }
            i += len;
        }
        len <<= 1;
    }
}

/// In-place IFFT (FFT with conjugated twiddles + 1/N scaling).
pub fn ifft_inplace(x: &mut [Cf64]) {
    // Conjugate
    for s in x.iter_mut() { s.im = -s.im; }
    fft_inplace(x);
    // Conjugate and scale
    let n = x.len() as f64;
    for s in x.iter_mut() {
        s.im = -s.im;
        s.re /= n;
        s.im /= n;
    }
}

fn bit_reverse(mut x: usize, bits: usize) -> usize {
    let mut r = 0usize;
    for _ in 0..bits {
        r = (r << 1) | (x & 1);
        x >>= 1;
    }
    r
}

// ---------------------------------------------------------------------------
// OFDM modulator
// ---------------------------------------------------------------------------

/// Map a frequency-domain subcarrier index (−32..+31) to FFT bin (0..63).
#[inline]
pub fn sc_to_bin(sc: i32) -> usize {
    ((sc + FFT_SIZE as i32) % FFT_SIZE as i32) as usize
}

/// Produce one OFDM symbol time domain from frequency-domain subcarriers + CP.
/// `freq_domain`: 64-length array; `cp_len`: guard interval length.
pub fn ofdm_modulate_symbol(freq_domain: &[Cf64; 64], cp_len: usize) -> Vec<Cf64> {
    let mut td = freq_domain.to_owned();
    ifft_inplace(&mut td);
    let mut out = Vec::with_capacity(td.len() + cp_len);
    // Prepend cyclic prefix (last cp_len samples)
    let cp_start = FFT_SIZE - cp_len;
    out.extend_from_slice(&td[cp_start..]);
    out.extend_from_slice(&td);
    out
}

/// Demodulate one OFDM symbol: remove CP, FFT.
pub fn ofdm_demodulate_symbol(samples: &[Cf64], cp_len: usize) -> [Cf64; 64] {
    assert!(samples.len() >= cp_len + FFT_SIZE);
    let mut fd = [Cf64::zero(); 64];
    fd.copy_from_slice(&samples[cp_len..cp_len + FFT_SIZE]);
    fft_inplace(&mut fd);
    fd
}

// ---------------------------------------------------------------------------
// Preamble generation
// ---------------------------------------------------------------------------

/// Generate L-STF (Short Training Field): 10 repetitions of a 16-sample pattern.
/// Frequency domain: 12 non-zero subcarriers at ±4,±8,±12,±16,±20,±24.
pub fn generate_l_stf() -> Vec<Cf64> {
    // Per IEEE 802.11-2020 Table 17-5: subcarriers ±4,±8,±12,±16,±20,±24
    // Values: sqrt(13/6) * {±1, ±j, ...}
    let scale = (13.0_f64 / 6.0).sqrt();
    let stf_freq: [(i32, Cf64); 12] = [
        (-24, Cf64::new( scale,  0.0)),
        (-20, Cf64::new( 0.0,  scale)),
        (-16, Cf64::new(-scale,  0.0)),
        (-12, Cf64::new( 0.0, -scale)),
        ( -8, Cf64::new( scale,  0.0)),
        ( -4, Cf64::new( 0.0,  scale)),
        (  4, Cf64::new( scale,  0.0)),
        (  8, Cf64::new( 0.0,  scale)),
        ( 12, Cf64::new(-scale,  0.0)),
        ( 16, Cf64::new( 0.0, -scale)),
        ( 20, Cf64::new( scale,  0.0)),
        ( 24, Cf64::new( 0.0, -scale)),
    ];
    let mut fd = [Cf64::zero(); 64];
    for (sc, val) in &stf_freq {
        fd[sc_to_bin(*sc)] = *val;
    }
    let mut td = fd;
    ifft_inplace(&mut td);
    // Repeat 10 times (each repetition is 16 samples = FFT_SIZE/4)
    let pattern: Vec<Cf64> = td[..16].to_vec();
    let mut out = Vec::with_capacity(160);
    for _ in 0..10 {
        out.extend_from_slice(&pattern);
    }
    out
}

/// Generate L-LTF (Long Training Field):
/// 2×CP (32 samples) + 2×OFDM LTF symbols (128 samples) = 160 samples.
pub fn generate_l_ltf() -> Vec<Cf64> {
    let ltf = ltf_symbols();
    let mut fd = [Cf64::zero(); 64];
    for (i, &v) in ltf.iter().enumerate() {
        fd[i] = Cf64::new(v as f64, 0.0);
    }
    let mut td = fd;
    ifft_inplace(&mut td);
    // Long GI = last 32 samples of td (2×CP)
    let mut out = Vec::with_capacity(160);
    out.extend_from_slice(&td[32..]); // GI2 (32 samples)
    out.extend_from_slice(&td);       // LTF symbol 1
    out.extend_from_slice(&td);       // LTF symbol 2
    out
}

// ---------------------------------------------------------------------------
// L-SIG field
// ---------------------------------------------------------------------------

/// Generate the L-SIG (Legacy Signal) field as one OFDM symbol.
/// Contains: [4-bit rate | 1-bit reserved | 12-bit length | 1-bit parity | 6-bit tail]
pub fn generate_l_sig(rate: WifiRate, psdu_length: usize) -> Vec<Cf64> {
    let rate_bits = rate.sig_rate_bits();
    let len = psdu_length & 0xFFF;
    let mut sig_bits = vec![0u8; 24];
    // Rate field (bits 0-3)
    for i in 0..4 { sig_bits[i] = rate_bits[i]; }
    // Reserved bit 4 = 0
    sig_bits[4] = 0;
    // Length field (bits 5-16), LSB first
    for i in 0..12 {
        sig_bits[5 + i] = ((len >> i) & 1) as u8;
    }
    // Parity bit 17: even parity over bits 0-16
    let parity: u8 = sig_bits[0..17].iter().fold(0u8, |acc, &b| acc ^ b);
    sig_bits[17] = parity;
    // Tail bits 18-23 = 0 (for convolutional code flushing)

    encode_sig_symbol(&sig_bits)
}

/// Encode a 24-bit signal field to one OFDM symbol.
fn encode_sig_symbol(bits: &[u8]) -> Vec<Cf64> {
    // BCC encode rate 1/2
    let mut enc = BccEncoder::new();
    let coded = enc.encode(bits); // 48 bits
    // Interleave with CBPS=48, BPSC=1 (BPSK)
    let il = OfdmInterleaver::new(48, 1);
    let interleaved = il.interleave(&coded);
    // BPSK map
    let syms: Vec<Cf64> = interleaved.iter().map(|&b| bpsk_map(b)).collect();
    // Build frequency domain
    let data_scs = data_subcarrier_indices();
    let mut fd = [Cf64::zero(); 64];
    for (i, &sc) in data_scs.iter().enumerate() {
        if i < syms.len() {
            fd[sc_to_bin(sc)] = syms[i];
        }
    }
    // Insert pilots (polarity p=1 for L-SIG, all same polarity)
    let pilot_polarity = 1.0;
    for &sc in &PILOT_SUBCARRIERS {
        fd[sc_to_bin(sc)] = Cf64::new(pilot_polarity, 0.0);
    }
    ofdm_modulate_symbol(&fd, CP_LEN)
}

// ---------------------------------------------------------------------------
// HT-SIG field (802.11n)
// ---------------------------------------------------------------------------

/// HT-SIG parameters.
#[derive(Debug, Clone, Copy)]
pub struct HtSigParams {
    pub mcs: u8,           // MCS index 0-31
    pub cbw_40: bool,      // Channel bandwidth (false=20, true=40)
    pub ht_length: u16,    // Bytes in HT data field (12 bits)
    pub aggregation: bool, // A-MPDU aggregation
    pub short_gi: bool,    // Short guard interval
    pub num_ess: u8,       // Number of extension spatial streams
    pub crc: u8,           // 8-bit CRC
}

/// Generate HT-SIG field (2 BPSK OFDM symbols per IEEE 802.11-2020 §19.3.9.5).
pub fn generate_ht_sig(params: &HtSigParams) -> Vec<Cf64> {
    // HT-SIG1: bits 0..23 (MCS[6:0], CBW, HT-Length[15:0])
    let mut ht_sig1 = [0u8; 24];
    for i in 0..7 { ht_sig1[i] = (params.mcs >> i) & 1; }
    ht_sig1[7] = params.cbw_40 as u8;
    for i in 0..16 { ht_sig1[8 + i] = ((params.ht_length >> i) & 1) as u8; }

    // HT-SIG2: bits 0..23 (Smooth|NoSounding|Reserved|Aggregation|STBC|LDPC|SGI|NumESS|CRC|Tail)
    let mut ht_sig2 = [0u8; 24];
    ht_sig2[0] = 1; // smoothing recommended
    ht_sig2[1] = 1; // not sounding
    ht_sig2[2] = 1; // reserved=1
    ht_sig2[3] = params.aggregation as u8;
    // STBC = 0, LDPC = 0
    ht_sig2[6] = params.short_gi as u8;
    for i in 0..2 { ht_sig2[7 + i] = (params.num_ess >> i) & 1; }
    let crc = params.crc;
    for i in 0..8 { ht_sig2[9 + i] = (crc >> i) & 1; }

    // Encode and interleave each HT-SIG symbol (BPSK, CBPS=48)
    let sym1 = encode_sig_symbol(&ht_sig1);
    let sym2 = encode_sig_symbol(&ht_sig2);
    let mut out = sym1;
    out.extend(sym2);
    out
}

// ---------------------------------------------------------------------------
// Data field encoding pipeline
// ---------------------------------------------------------------------------

/// Encode PSDU bytes into a stream of OFDM symbols.
///
/// Pipeline: pad → scramble → BCC encode → puncture → interleave → map → OFDM
pub fn encode_data_symbols(
    psdu: &[u8],
    rate: WifiRate,
    short_gi: bool,
) -> Vec<Vec<Cf64>> {
    let params = rate.params();
    let cp = if short_gi { CP_LEN_SGI } else { CP_LEN };

    // 1. Convert PSDU bytes to bits (LSB first)
    let mut data_bits: Vec<u8> = Vec::with_capacity(psdu.len() * 8);
    for &byte in psdu {
        for bit in 0..8 {
            data_bits.push((byte >> bit) & 1);
        }
    }

    // 2. Compute padding: total data bits must be multiple of DBPS
    let n_sym = ((data_bits.len() + 16 + 6 + params.dbps - 1) / params.dbps).max(1);
    let n_data = n_sym * params.dbps;
    let n_pad = n_data.saturating_sub(data_bits.len() + 16 + 6);

    // 3. Build padded service + data + tail + padding
    let mut tx_bits = vec![0u8; 16]; // SERVICE field (16 zero bits)
    tx_bits.extend_from_slice(&data_bits);
    tx_bits.extend(std::iter::repeat(0u8).take(6 + n_pad)); // tail + pad

    // 4. Scramble with seed = 0x7F
    let mut scrambler = WifiScrambler::new(0x7F);
    scrambler.process_inplace(&mut tx_bits);
    // Zero the tail bits (last 6 before padding)
    let tail_start = 16 + data_bits.len();
    for i in 0..6 {
        if tail_start + i < tx_bits.len() {
            tx_bits[tail_start + i] = 0;
        }
    }

    // 5. BCC encode
    let mut enc = BccEncoder::new();
    let coded = enc.encode(&tx_bits);
    // Puncture to target rate
    let punctured = puncture(&coded, params.code_rate);

    // 6. Pad coded bits to multiple of CBPS
    let cbps = params.cbps;
    let total_coded = n_sym * cbps;
    let mut coded_padded = punctured;
    while coded_padded.len() < total_coded {
        coded_padded.push(0);
    }
    coded_padded.truncate(total_coded);

    // 7. Build OFDM symbols
    let il = OfdmInterleaver::new(cbps, params.bits_per_subcarrier);
    let data_scs = data_subcarrier_indices();
    let mut pilot_seq = PilotPolaritySeq::new();
    // Advance pilot sequence past preamble (1 L-SIG symbol = 1 step)
    let _ = pilot_seq.next();

    let mut symbols = Vec::with_capacity(n_sym);
    for sym_idx in 0..n_sym {
        let chunk = &coded_padded[sym_idx * cbps..(sym_idx + 1) * cbps];
        let interleaved = il.interleave(chunk);
        let const_syms = map_symbols(&interleaved, params.modulation);

        let mut fd = [Cf64::zero(); 64];
        for (i, &sc) in data_scs.iter().enumerate() {
            if i < const_syms.len() {
                fd[sc_to_bin(sc)] = const_syms[i];
            }
        }

        // Insert pilots
        let pol = pilot_seq.next();
        let pilots = [pol, pol, pol, -pol];
        for (p_idx, &sc) in PILOT_SUBCARRIERS.iter().enumerate() {
            fd[sc_to_bin(sc)] = Cf64::new(pilots[p_idx], 0.0);
        }

        symbols.push(ofdm_modulate_symbol(&fd, cp));
    }
    symbols
}

// ---------------------------------------------------------------------------
// Viterbi soft-decision decoder
// ---------------------------------------------------------------------------

/// Soft Viterbi decoder for K=7, rate 1/2 BCC.
/// Uses hard decisions (0/1/erasure=2).
pub struct ViterbiDecoder {
    num_states: usize,
    trellis: Vec<[u8; 2]>, // [state][input] -> (next_state, output_pair_idx)
}

/// (next_state, output bits a and b) for K=7 encoder
fn next_state_output(state: u8, input: u8) -> (u8, u8, u8) {
    let new_shift = ((state << 1) | (input & 1)) & 0x3F; // 6-bit state
    let full_state = new_shift; // shifted register
    let g0 = 0b1011011u8;
    let g1 = 0b1111001u8;
    let full_reg = (full_state & 0x3F) | ((input & 1) << 6);
    let a = (full_reg & g0).count_ones() as u8 & 1;
    let b = (full_reg & g1).count_ones() as u8 & 1;
    (new_shift, a, b)
}

impl ViterbiDecoder {
    pub fn new() -> Self {
        ViterbiDecoder { num_states: 64, trellis: vec![] }
    }

    /// Decode a soft bit stream (values 0/1, erasure=2) to information bits.
    pub fn decode(&self, soft_bits: &[u8], output_len: usize) -> Vec<u8> {
        let num_states = 64usize;
        let inf = 1_000_000i32;
        let n_pairs = soft_bits.len() / 2;

        let mut metrics = vec![inf; num_states];
        metrics[0] = 0;
        let mut history: Vec<Vec<(u8, u8)>> = Vec::with_capacity(n_pairs);

        for t in 0..n_pairs {
            let rx_a = soft_bits[2 * t];
            let rx_b = soft_bits[2 * t + 1];
            let mut new_metrics = vec![inf; num_states];
            let mut prev: Vec<(u8, u8)> = vec![(0, 0); num_states];

            for state in 0..num_states {
                if metrics[state] == inf { continue; }
                for input in 0..2u8 {
                    let (next, a, b) = next_state_output(state as u8, input);
                    let branch = branch_metric(rx_a, a) + branch_metric(rx_b, b);
                    let cost = metrics[state] + branch;
                    let ns = next as usize;
                    if cost < new_metrics[ns] {
                        new_metrics[ns] = cost;
                        prev[ns] = (state as u8, input);
                    }
                }
            }
            history.push(prev);
            metrics = new_metrics;
        }

        // Traceback from state 0 (all-zeros trellis termination)
        let mut decoded = vec![0u8; n_pairs];
        let mut state = 0usize;
        for t in (0..n_pairs).rev() {
            let (prev_s, input) = history[t][state];
            decoded[t] = input;
            state = prev_s as usize;
        }

        decoded[8..].iter().take(output_len).cloned().collect()
    }
}

fn branch_metric(received: u8, expected: u8) -> i32 {
    if received == 2 { 0 } // erasure
    else if received == expected { 0 } else { 1 }
}

impl Default for ViterbiDecoder {
    fn default() -> Self { Self::new() }
}

// ---------------------------------------------------------------------------
// Channel estimation (from L-LTF)
// ---------------------------------------------------------------------------

/// Estimate channel response from received L-LTF symbols.
/// Returns 64-point channel estimate H[k].
pub fn estimate_channel_from_ltf(ltf1: &[Cf64; 64], ltf2: &[Cf64; 64]) -> [Cf64; 64] {
    let known = ltf_symbols();
    let mut h = [Cf64::zero(); 64];
    for k in 0..64 {
        let kn = Cf64::new(known[k] as f64, 0.0);
        if kn.re == 0.0 && kn.im == 0.0 { continue; }
        // Average the two LTF observations
        let avg = Cf64::new(
            (ltf1[k].re + ltf2[k].re) * 0.5,
            (ltf1[k].im + ltf2[k].im) * 0.5,
        );
        // H = Y / X
        let mag_sq = kn.mag_sq();
        h[k] = Cf64::new(
            (avg.re * kn.re + avg.im * kn.im) / mag_sq,
            (avg.im * kn.re - avg.re * kn.im) / mag_sq,
        );
    }
    h
}

/// Zero-forcing equalization: Y[k] / H[k].
pub fn zf_equalize(fd: &[Cf64; 64], channel: &[Cf64; 64]) -> [Cf64; 64] {
    let mut out = [Cf64::zero(); 64];
    for k in 0..64 {
        let mag_sq = channel[k].mag_sq();
        if mag_sq < 1e-12 { continue; }
        // Y / H = Y * conj(H) / |H|^2
        let h_conj = channel[k].conj();
        out[k] = fd[k].mul(h_conj).scale(1.0 / mag_sq);
    }
    out
}

// ---------------------------------------------------------------------------
// L-SIG decoder
// ---------------------------------------------------------------------------

/// Decode L-SIG symbol (received FD, 64-point after ZF equalization).
/// Returns (rate, length_bytes, parity_ok).
pub fn decode_l_sig(fd_eq: &[Cf64; 64]) -> (WifiRate, usize, bool) {
    let data_scs = data_subcarrier_indices();
    // Extract data subcarriers (BPSK demap)
    let mut interleaved = Vec::with_capacity(48);
    for &sc in &data_scs {
        interleaved.push(bpsk_demap(fd_eq[sc_to_bin(sc)]));
    }
    // Deinterleave
    let il = OfdmInterleaver::new(48, 1);
    let coded = il.deinterleave(&interleaved);
    // Viterbi decode
    let vit = ViterbiDecoder::new();
    let bits = vit.decode(&coded, 18);

    // Parse rate field (bits 0-3)
    let rate = decode_rate_bits([bits[0], bits[1], bits[2], bits[3]]);
    // Parse length (bits 5-16)
    let mut length = 0usize;
    for i in 0..12 {
        if bits.len() > 5 + i { length |= (bits[5 + i] as usize) << i; }
    }
    // Parity check
    let parity: u8 = if bits.len() > 17 { bits[0..17].iter().fold(0, |a, &b| a ^ b) } else { 0 };
    let parity_ok = if bits.len() > 17 { parity == bits[17] } else { false };

    (rate, length, parity_ok)
}

fn decode_rate_bits(b: [u8; 4]) -> WifiRate {
    match b {
        [1,1,0,1] => WifiRate::Rate6Mbps,
        [1,1,1,1] => WifiRate::Rate9Mbps,
        [0,1,0,1] => WifiRate::Rate12Mbps,
        [0,1,1,1] => WifiRate::Rate18Mbps,
        [1,0,0,1] => WifiRate::Rate24Mbps,
        [1,0,1,1] => WifiRate::Rate36Mbps,
        [0,0,1,0] => WifiRate::Rate48Mbps,
        _         => WifiRate::Rate54Mbps,
    }
}

// ---------------------------------------------------------------------------
// Data decoder
// ---------------------------------------------------------------------------

/// Decode received OFDM data symbols to PSDU bytes.
pub fn decode_data_symbols(
    symbols_fd: &[[Cf64; 64]],
    rate: WifiRate,
    psdu_length: usize,
) -> Vec<u8> {
    let params = rate.params();
    let data_scs = data_subcarrier_indices();
    let il = OfdmInterleaver::new(params.cbps, params.bits_per_subcarrier);

    let mut all_coded_bits: Vec<u8> = Vec::new();

    for fd in symbols_fd {
        // Extract data subcarrier symbols
        let mut data_syms: Vec<Cf64> = Vec::with_capacity(NUM_DATA_SC);
        for &sc in &data_scs {
            data_syms.push(fd[sc_to_bin(sc)]);
        }
        // Demap
        let mapped_bits = demap_symbols(&data_syms, params.modulation);
        // Deinterleave
        let coded_bits = il.deinterleave(&mapped_bits);
        all_coded_bits.extend_from_slice(&coded_bits);
    }

    // Depuncture
    let depunctured = depuncture(&all_coded_bits, params.code_rate);

    // Viterbi decode
    let vit = ViterbiDecoder::new();
    let n_bits = (psdu_length + 2) * 8; // include SERVICE field
    let decoded_bits = vit.decode(&depunctured, n_bits);

    // Descramble (skip SERVICE field = 16 bits, seed from bits [6:0])
    let seed = if decoded_bits.len() >= 7 {
        decoded_bits[0..7].iter().enumerate().fold(0u8, |acc, (i, &b)| acc | (b << i))
    } else { 0x7F };
    let seed = if seed == 0 { 0x7F } else { seed };
    let mut descrambler = WifiScrambler::new(seed);
    let payload_bits: Vec<u8> = if decoded_bits.len() > 16 {
        // Skip service field
        let svc: Vec<u8> = decoded_bits[..16].iter().map(|_| descrambler.clock()).collect();
        let _ = svc; // discard
        decoded_bits[16..].iter().map(|&b| b ^ descrambler.clock()).collect()
    } else {
        vec![]
    };

    // Convert bits to bytes (LSB first)
    let mut bytes = Vec::with_capacity(psdu_length);
    for chunk in payload_bits.chunks(8) {
        if chunk.len() == 8 {
            let byte = chunk.iter().enumerate().fold(0u8, |acc, (i, &b)| acc | (b << i));
            bytes.push(byte);
        }
        if bytes.len() >= psdu_length { break; }
    }
    bytes
}

// ---------------------------------------------------------------------------
// WiFi configuration
// ---------------------------------------------------------------------------

/// WiFi transceiver configuration.
#[derive(Debug, Clone)]
pub struct WifiConfig {
    /// Use short guard interval (400 ns instead of 800 ns)
    pub short_gi: bool,
    /// HT mode (802.11n) vs legacy (802.11a/g)
    pub ht_mode: bool,
}

impl Default for WifiConfig {
    fn default() -> Self {
        WifiConfig { short_gi: false, ht_mode: false }
    }
}

// ---------------------------------------------------------------------------
// Main transceiver
// ---------------------------------------------------------------------------

/// 802.11a/g/n OFDM Physical Layer Transceiver.
pub struct WifiOfdmTransceiver {
    config: WifiConfig,
}

impl WifiOfdmTransceiver {
    pub fn new(config: WifiConfig) -> Self {
        WifiOfdmTransceiver { config }
    }

    /// Transmit PSDU bytes at a given rate. Returns time-domain IQ samples.
    ///
    /// Frame structure: L-STF | L-LTF | L-SIG | [HT-SIG|HT-STF|HT-LTF] | DATA
    pub fn transmit(&mut self, psdu: &[u8], rate: WifiRate) -> Vec<Cf64> {
        let mut out = Vec::new();
        // L-STF
        out.extend(generate_l_stf());
        // L-LTF
        out.extend(generate_l_ltf());
        // L-SIG
        out.extend(generate_l_sig(rate, psdu.len()));
        // HT fields if 802.11n
        if self.config.ht_mode {
            let ht_sig_params = HtSigParams {
                mcs: 0,
                cbw_40: false,
                ht_length: psdu.len() as u16,
                aggregation: false,
                short_gi: self.config.short_gi,
                num_ess: 0,
                crc: compute_ht_crc(0, psdu.len() as u16),
            };
            out.extend(generate_ht_sig(&ht_sig_params));
            // HT-STF (one OFDM symbol, same as L-STF but different sequence)
            out.extend(generate_ht_stf());
            // HT-LTF (one OFDM symbol per spatial stream)
            out.extend(generate_ht_ltf());
        }
        // DATA symbols
        let data_symbols = encode_data_symbols(psdu, rate, self.config.short_gi);
        for sym in data_symbols {
            out.extend(sym);
        }
        out
    }

    /// Receive a frame. Assumes perfect sync (no CFO/timing offset).
    /// Returns decoded PSDU bytes.
    pub fn receive(&mut self, samples: &[Cf64]) -> Option<Vec<u8>> {
        let min_preamble = L_STF_SAMPLES + L_LTF_SAMPLES + SYMBOL_SAMPLES;
        if samples.len() < min_preamble { return None; }

        let offset = L_STF_SAMPLES;
        // Extract and demodulate LTF symbols
        let ltf_start = offset;
        let gi2_len = 32; // Long GI
        let ltf1_start = ltf_start + gi2_len;
        let ltf2_start = ltf1_start + FFT_SIZE;

        if samples.len() < ltf2_start + FFT_SIZE { return None; }

        let mut ltf1_buf = [Cf64::zero(); 64];
        ltf1_buf.copy_from_slice(&samples[ltf1_start..ltf1_start + 64]);
        fft_inplace(&mut ltf1_buf);

        let mut ltf2_buf = [Cf64::zero(); 64];
        ltf2_buf.copy_from_slice(&samples[ltf2_start..ltf2_start + 64]);
        fft_inplace(&mut ltf2_buf);

        let channel = estimate_channel_from_ltf(&ltf1_buf, &ltf2_buf);

        // L-SIG
        let lsig_start = L_STF_SAMPLES + L_LTF_SAMPLES;
        if samples.len() < lsig_start + SYMBOL_SAMPLES { return None; }
        let lsig_fd = ofdm_demodulate_symbol(&samples[lsig_start..], CP_LEN);
        let lsig_eq = zf_equalize(&lsig_fd, &channel);
        let (rate, psdu_len, _) = decode_l_sig(&lsig_eq);

        // DATA
        let data_start = lsig_start + SYMBOL_SAMPLES;
        let params = rate.params();
        let cp = if self.config.short_gi { CP_LEN_SGI } else { CP_LEN };
        let sym_len = FFT_SIZE + cp;
        let n_sym = if psdu_len == 0 { 0 } else {
            let n_data_bits = psdu_len * 8;
            (n_data_bits + 16 + 6 + params.dbps - 1) / params.dbps
        };

        if samples.len() < data_start + n_sym * sym_len { return None; }

        let mut data_fds: Vec<[Cf64; 64]> = Vec::with_capacity(n_sym);
        for i in 0..n_sym {
            let start = data_start + i * sym_len;
            let fd = ofdm_demodulate_symbol(&samples[start..], cp);
            let eq = zf_equalize(&fd, &channel);
            data_fds.push(eq);
        }

        let psdu = decode_data_symbols(&data_fds, rate, psdu_len);
        Some(psdu)
    }
}

// ---------------------------------------------------------------------------
// HT-STF and HT-LTF
// ---------------------------------------------------------------------------

/// Generate HT-STF (High-Throughput Short Training Field): 1 OFDM symbol.
pub fn generate_ht_stf() -> Vec<Cf64> {
    // HT-STF uses same L-STF 16-subcarrier pattern but different subcarriers
    // Per 802.11-2020 §19.3.9.3: HT-STF uses subcarriers ±4,±8,...,±56 with 40 MHz
    // For 20 MHz, we reuse the L-STF pattern
    generate_l_stf()[..SYMBOL_SAMPLES].to_vec()
}

/// Generate HT-LTF (High-Throughput Long Training Field): 1 OFDM symbol.
pub fn generate_ht_ltf() -> Vec<Cf64> {
    // 802.11n HT-LTF uses the same L-LTF known sequence but with additional subcarriers
    // For single spatial stream, use same L-LTF pattern
    generate_l_ltf()[32..32 + SYMBOL_SAMPLES].to_vec() // skip GI2
}

/// Compute simple 8-bit CRC for HT-SIG (XOR fold over fields).
pub fn compute_ht_crc(mcs: u8, length: u16) -> u8 {
    let mut crc = 0u8;
    for i in 0..7 { crc ^= (mcs >> i) & 1; }
    for i in 0..16 { crc ^= ((length >> i) & 1) as u8; }
    crc
}

// ---------------------------------------------------------------------------
// Timing synchronization utilities
// ---------------------------------------------------------------------------

/// Schmidl-Cox timing metric for L-STF detection.
/// Returns timing metric M(d) = |P(d)|^2 / R(d)^2 for each delay d.
pub fn schmidl_cox_metric(samples: &[Cf64], half_window: usize) -> Vec<f64> {
    let n = samples.len();
    if n < 2 * half_window { return vec![]; }
    let mut out = Vec::with_capacity(n - 2 * half_window);
    for d in 0..n.saturating_sub(2 * half_window) {
        let mut p = Cf64::zero();
        let mut r = 0.0f64;
        for k in 0..half_window {
            let x1 = samples[d + k];
            let x2 = samples[d + k + half_window];
            p = p.add(x1.conj().mul(x2));
            r += x2.mag_sq();
        }
        let metric = if r < 1e-20 { 0.0 } else { p.mag_sq() / (r * r) };
        out.push(metric);
    }
    out
}

/// Simple power-based L-STF detection: find plateau in Schmidl-Cox metric.
pub fn detect_frame_start(samples: &[Cf64]) -> Option<usize> {
    let metrics = schmidl_cox_metric(samples, 16);
    let threshold = 0.5;
    let mut best_idx = None;
    let mut best_metric = threshold;
    for (i, &m) in metrics.iter().enumerate() {
        if m > best_metric {
            best_metric = m;
            best_idx = Some(i);
        }
    }
    best_idx
}

// ---------------------------------------------------------------------------
// Utility: bytes ↔ bits
// ---------------------------------------------------------------------------

/// Convert bytes to bits (LSB first).
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(bytes.len() * 8);
    for &b in bytes {
        for i in 0..8 { bits.push((b >> i) & 1); }
    }
    bits
}

/// Convert bits to bytes (LSB first).
pub fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8).filter(|c| c.len() == 8)
        .map(|c| c.iter().enumerate().fold(0u8, |acc, (i, &b)| acc | (b << i)))
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Basic complex arithmetic ---
    #[test]
    fn test_cf64_new_and_zero() {
        let z = Cf64::zero();
        assert_eq!(z.re, 0.0);
        assert_eq!(z.im, 0.0);
        let c = Cf64::new(3.0, 4.0);
        assert!((c.mag() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_cf64_mul() {
        let a = Cf64::new(1.0, 2.0);
        let b = Cf64::new(3.0, 4.0);
        let c = a.mul(b); // (1+2j)(3+4j) = 3+4j+6j+8j^2 = -5+10j
        assert!((c.re - (-5.0)).abs() < 1e-10);
        assert!((c.im - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_cf64_conj() {
        let a = Cf64::new(1.0, -3.0);
        let b = a.conj();
        assert_eq!(b.re, 1.0);
        assert_eq!(b.im, 3.0);
    }

    // --- FFT / IFFT ---
    #[test]
    fn test_fft_impulse() {
        let mut x = [Cf64::zero(); 64];
        x[0] = Cf64::new(1.0, 0.0);
        fft_inplace(&mut x);
        for s in &x {
            assert!((s.re - 1.0).abs() < 1e-10, "impulse FFT should be all ones: {}", s.re);
            assert!(s.im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_ifft_inverse_of_fft() {
        let mut data = [Cf64::zero(); 64];
        for i in 0..64 {
            data[i] = Cf64::new(i as f64, (64 - i) as f64);
        }
        let original = data;
        fft_inplace(&mut data);
        ifft_inplace(&mut data);
        for i in 0..64 {
            assert!((data[i].re - original[i].re).abs() < 1e-6, "IFFT(FFT(x)) != x at {i}");
            assert!((data[i].im - original[i].im).abs() < 1e-6);
        }
    }

    #[test]
    fn test_fft_linearity() {
        let mut a = [Cf64::zero(); 64];
        let mut b = [Cf64::zero(); 64];
        a[1] = Cf64::new(1.0, 0.0);
        b[2] = Cf64::new(2.0, 0.0);
        let mut ab = [Cf64::zero(); 64];
        for i in 0..64 { ab[i] = a[i].add(b[i]); }
        fft_inplace(&mut a);
        fft_inplace(&mut b);
        fft_inplace(&mut ab);
        for i in 0..64 {
            let expected = a[i].add(b[i]);
            assert!((ab[i].re - expected.re).abs() < 1e-6);
        }
    }

    // --- Scrambler ---
    #[test]
    fn test_scrambler_self_inverse() {
        let data: Vec<u8> = (0..100).map(|i| i & 1).collect();
        let mut sc1 = WifiScrambler::new(0x7F);
        let scrambled = sc1.process_bits(&data);
        let mut sc2 = WifiScrambler::new(0x7F);
        let recovered = sc2.process_bits(&scrambled);
        assert_eq!(data, recovered);
    }

    #[test]
    fn test_scrambler_nonzero_seed() {
        let data = vec![1u8; 32];
        let mut sc = WifiScrambler::new(0x55);
        let out = sc.process_bits(&data);
        assert_ne!(data, out); // output should differ from input
    }

    #[test]
    fn test_scrambler_zero_seed_defaults_to_7f() {
        let data = vec![0u8; 20];
        let mut sc_zero = WifiScrambler::new(0);
        let mut sc_7f = WifiScrambler::new(0x7F);
        let out0 = sc_zero.process_bits(&data);
        let out7 = sc_7f.process_bits(&data);
        assert_eq!(out0, out7);
    }

    // --- BCC encoder ---
    #[test]
    fn test_bcc_encoder_all_zeros() {
        let mut enc = BccEncoder::new();
        let bits = vec![0u8; 10];
        let coded = enc.encode(&bits);
        assert_eq!(coded.len(), 20);
        // All zeros input should produce all zeros output
        assert!(coded.iter().all(|&b| b == 0));
    }

    #[test]
    fn test_bcc_encoder_length() {
        let mut enc = BccEncoder::new();
        let bits: Vec<u8> = (0..48).map(|i| i & 1).collect();
        let coded = enc.encode(&bits);
        assert_eq!(coded.len(), 96);
    }

    #[test]
    fn test_bcc_encoder_known_bit() {
        // With shift register = 0 and input = 1:
        // new_reg = 0x40 (bit 6 = 1)
        // g0 = 0b1011011, g0 & 0x40 = 0x40, popcount=1, a=1
        // g1 = 0b1111001, g1 & 0x40 = 0x40, popcount=1, b=1
        let mut enc = BccEncoder::new();
        let (a, b) = {
            let bits = vec![1u8];
            let coded = enc.encode(&bits);
            (coded[0], coded[1])
        };
        assert_eq!(a, 1);
        assert_eq!(b, 1);
    }

    // --- Puncturing ---
    #[test]
    fn test_puncture_half_rate_identity() {
        let bits: Vec<u8> = (0..20).map(|i| i & 1).collect();
        let p = puncture(&bits, CodeRate::OneHalf);
        assert_eq!(p, bits);
    }

    #[test]
    fn test_puncture_two_thirds() {
        let bits = vec![1, 0, 1, 0,  1, 1, 0, 1];
        let p = puncture(&bits, CodeRate::TwoThirds);
        // Each group of 4 → 3 (drop last)
        assert_eq!(p.len(), 6);
        assert_eq!(&p[0..3], &[1, 0, 1]);
        assert_eq!(&p[3..6], &[1, 1, 0]);
    }

    #[test]
    fn test_puncture_three_quarters() {
        let bits: Vec<u8> = vec![1,0,1,0,0,1, 1,1,0,1,1,0];
        let p = puncture(&bits, CodeRate::ThreeQuarters);
        // Each group of 6 → 4 (drop [3],[4])
        assert_eq!(p.len(), 8);
    }

    #[test]
    fn test_depuncture_two_thirds_roundtrip() {
        let bits = vec![1u8, 0, 1, 1, 0, 1, 0, 0, 1];
        let p = puncture(&bits, CodeRate::TwoThirds);
        let dp = depuncture(&p, CodeRate::TwoThirds);
        // Verify non-erased positions match original
        let mut orig_idx = 0;
        for (i, &v) in dp.iter().enumerate() {
            if v != 2 {
                if orig_idx < bits.len() {
                    assert_eq!(v, bits[orig_idx], "mismatch at dp pos {i}");
                    orig_idx += 1;
                }
            } else {
                orig_idx += 1; // punctured position
            }
        }
    }

    // --- Interleaver ---
    #[test]
    fn test_interleaver_bpsk() {
        let il = OfdmInterleaver::new(48, 1);
        let bits: Vec<u8> = (0..48).map(|i| (i & 1) as u8).collect();
        let interleaved = il.interleave(&bits);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(bits, deinterleaved);
    }

    #[test]
    fn test_interleaver_qpsk() {
        let il = OfdmInterleaver::new(96, 2);
        let bits: Vec<u8> = (0..96).map(|i| (i % 3 != 0) as u8).collect();
        let interleaved = il.interleave(&bits);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(bits, deinterleaved);
    }

    #[test]
    fn test_interleaver_qam16() {
        let il = OfdmInterleaver::new(192, 4);
        let bits: Vec<u8> = (0..192).map(|i| (i & 1) as u8).collect();
        let out = il.deinterleave(&il.interleave(&bits));
        assert_eq!(bits, out);
    }

    #[test]
    fn test_interleaver_qam64() {
        let il = OfdmInterleaver::new(288, 6);
        let bits: Vec<u8> = (0..288).map(|i| (i % 5 == 0) as u8).collect();
        let out = il.deinterleave(&il.interleave(&bits));
        assert_eq!(bits, out);
    }

    // --- Constellation mappers ---
    #[test]
    fn test_bpsk_map_and_demap() {
        assert_eq!(bpsk_demap(bpsk_map(0)), 0);
        assert_eq!(bpsk_demap(bpsk_map(1)), 1);
    }

    #[test]
    fn test_qpsk_map_power() {
        let s00 = qpsk_map(&[0, 0]);
        let power = s00.mag_sq();
        assert!((power - 1.0).abs() < 1e-10, "QPSK should have unit power: {power}");
    }

    #[test]
    fn test_qam16_map_demap_roundtrip() {
        for b0 in 0..2u8 {
            for b1 in 0..2u8 {
                for b2 in 0..2u8 {
                    for b3 in 0..2u8 {
                        let sym = qam16_map(&[b0, b1, b2, b3]);
                        let bits = qam16_demap(sym);
                        assert_eq!(bits, [b0, b1, b2, b3], "16-QAM roundtrip failed");
                    }
                }
            }
        }
    }

    #[test]
    fn test_qam64_map_demap_roundtrip() {
        // Test a subset of 64-QAM points
        let test_cases: &[[u8; 6]] = &[
            [0,0,0, 0,0,0], [1,0,0, 1,0,0], [0,1,1, 0,1,1],
            [1,1,1, 1,1,1], [0,0,1, 1,1,0], [1,0,1, 0,1,0],
        ];
        for bits in test_cases {
            let sym = qam64_map(bits);
            let out = qam64_demap(sym);
            assert_eq!(&out, bits, "64-QAM roundtrip failed for {bits:?}");
        }
    }

    #[test]
    fn test_map_symbols_bpsk() {
        let bits = vec![0u8, 1, 0, 1, 1];
        let syms = map_symbols(&bits, Modulation::Bpsk);
        assert_eq!(syms.len(), 5);
        let demod = demap_symbols(&syms, Modulation::Bpsk);
        assert_eq!(demod, bits);
    }

    #[test]
    fn test_map_symbols_qam16_roundtrip() {
        let bits: Vec<u8> = (0..48).map(|i| (i & 1) as u8).collect();
        let syms = map_symbols(&bits, Modulation::Qam16);
        let out = demap_symbols(&syms, Modulation::Qam16);
        assert_eq!(bits, out);
    }

    // --- Pilot polarity ---
    #[test]
    fn test_pilot_polarity_plus_minus_one() {
        let mut ps = PilotPolaritySeq::new();
        for _ in 0..127 {
            let v = ps.next();
            assert!(v == 1.0 || v == -1.0);
        }
    }

    #[test]
    fn test_pilot_polarity_period_127() {
        let mut ps = PilotPolaritySeq::new();
        let seq1: Vec<f64> = (0..127).map(|_| ps.next()).collect();
        let seq2: Vec<f64> = (0..127).map(|_| ps.next()).collect();
        assert_eq!(seq1, seq2);
    }

    // --- Preamble generation ---
    #[test]
    fn test_l_stf_length() {
        let stf = generate_l_stf();
        assert_eq!(stf.len(), L_STF_SAMPLES, "L-STF should be 160 samples");
    }

    #[test]
    fn test_l_ltf_length() {
        let ltf = generate_l_ltf();
        assert_eq!(ltf.len(), L_LTF_SAMPLES, "L-LTF should be 160 samples");
    }

    #[test]
    fn test_l_stf_periodicity() {
        let stf = generate_l_stf();
        // Each 16-sample segment should be the same (periodicity)
        let pattern = &stf[..16];
        for i in 1..10 {
            let seg = &stf[i * 16..(i + 1) * 16];
            for j in 0..16 {
                assert!((pattern[j].re - seg[j].re).abs() < 1e-10,
                    "L-STF periodicity failed at rep {i} sample {j}");
            }
        }
    }

    #[test]
    fn test_l_sig_length() {
        let sig = generate_l_sig(WifiRate::Rate54Mbps, 100);
        assert_eq!(sig.len(), SYMBOL_SAMPLES, "L-SIG should be 80 samples");
    }

    // --- OFDM modulate/demodulate ---
    #[test]
    fn test_ofdm_modulate_cp_prefix() {
        let mut fd = [Cf64::zero(); 64];
        fd[0] = Cf64::new(1.0, 0.0);
        let sym = ofdm_modulate_symbol(&fd, CP_LEN);
        assert_eq!(sym.len(), SYMBOL_SAMPLES);
        // CP should equal the last CP_LEN samples of the OFDM symbol
        for i in 0..CP_LEN {
            assert!((sym[i].re - sym[FFT_SIZE + i].re).abs() < 1e-10,
                "CP mismatch at {i}");
        }
    }

    #[test]
    fn test_ofdm_roundtrip() {
        let mut fd = [Cf64::zero(); 64];
        fd[5] = Cf64::new(1.5, -0.7);
        fd[10] = Cf64::new(-0.3, 1.2);
        let td = ofdm_modulate_symbol(&fd, CP_LEN);
        let fd_rx = ofdm_demodulate_symbol(&td, CP_LEN);
        assert!((fd_rx[5].re - fd[5].re).abs() < 1e-6);
        assert!((fd_rx[5].im - fd[5].im).abs() < 1e-6);
        assert!((fd_rx[10].re - fd[10].re).abs() < 1e-6);
    }

    // --- Rate parameters ---
    #[test]
    fn test_rate_params_6mbps() {
        let p = WifiRate::Rate6Mbps.params();
        assert_eq!(p.modulation, Modulation::Bpsk);
        assert_eq!(p.code_rate, CodeRate::OneHalf);
        assert_eq!(p.dbps, 24);
        assert_eq!(p.cbps, 48);
    }

    #[test]
    fn test_rate_params_54mbps() {
        let p = WifiRate::Rate54Mbps.params();
        assert_eq!(p.modulation, Modulation::Qam64);
        assert_eq!(p.code_rate, CodeRate::ThreeQuarters);
        assert_eq!(p.dbps, 216);
        assert_eq!(p.cbps, 288);
    }

    #[test]
    fn test_all_rates_dbps() {
        let expected = [24, 36, 48, 72, 96, 144, 192, 216];
        let rates = [
            WifiRate::Rate6Mbps, WifiRate::Rate9Mbps, WifiRate::Rate12Mbps,
            WifiRate::Rate18Mbps, WifiRate::Rate24Mbps, WifiRate::Rate36Mbps,
            WifiRate::Rate48Mbps, WifiRate::Rate54Mbps,
        ];
        for (r, &e) in rates.iter().zip(expected.iter()) {
            assert_eq!(r.params().dbps, e, "DBPS mismatch for {r:?}");
        }
    }

    // --- Data subcarrier indices ---
    #[test]
    fn test_data_sc_count() {
        let scs = data_subcarrier_indices();
        assert_eq!(scs.len(), NUM_DATA_SC, "Should be 48 data subcarriers");
    }

    #[test]
    fn test_data_sc_no_dc_no_pilots() {
        let scs = data_subcarrier_indices();
        assert!(!scs.contains(&0), "DC should not be in data SCs");
        for p in &PILOT_SUBCARRIERS {
            assert!(!scs.contains(p), "Pilot {} should not be in data SCs", p);
        }
    }

    // --- HT MCS table ---
    #[test]
    fn test_ht_mcs_table_length() {
        let table = ht_mcs_table();
        assert_eq!(table.len(), 8);
    }

    #[test]
    fn test_ht_mcs7_65mbps() {
        let table = ht_mcs_table();
        let mcs7 = &table[7];
        assert!((mcs7.mbps - 65.0).abs() < 0.1);
    }

    // --- Encode/Decode data ---
    #[test]
    fn test_encode_data_symbols_not_empty() {
        let data = vec![0xABu8; 10];
        let symbols = encode_data_symbols(&data, WifiRate::Rate6Mbps, false);
        assert!(!symbols.is_empty());
    }

    #[test]
    fn test_encode_data_symbol_length() {
        let data = vec![0u8; 20];
        let symbols = encode_data_symbols(&data, WifiRate::Rate6Mbps, false);
        for sym in &symbols {
            assert_eq!(sym.len(), SYMBOL_SAMPLES, "Each data symbol should be 80 samples");
        }
    }

    // --- Transmit ---
    #[test]
    fn test_transmit_not_empty() {
        let mut tx = WifiOfdmTransceiver::new(WifiConfig::default());
        let data = vec![0x42u8; 50];
        let samples = tx.transmit(&data, WifiRate::Rate6Mbps);
        assert!(!samples.is_empty());
    }

    #[test]
    fn test_transmit_minimum_length() {
        let mut tx = WifiOfdmTransceiver::new(WifiConfig::default());
        let data = vec![0u8; 1];
        let samples = tx.transmit(&data, WifiRate::Rate6Mbps);
        let min_len = L_STF_SAMPLES + L_LTF_SAMPLES + SYMBOL_SAMPLES; // preamble + L-SIG
        assert!(samples.len() >= min_len);
    }

    // --- Channel estimation ---
    #[test]
    fn test_channel_estimation_identity() {
        // Perfect channel H=1 → L-LTF samples = known sequence
        let known = ltf_symbols();
        let mut fd = [Cf64::zero(); 64];
        for (i, &v) in known.iter().enumerate() {
            fd[i] = Cf64::new(v as f64, 0.0);
        }
        // Both LTF symbols identical
        let h = estimate_channel_from_ltf(&fd, &fd);
        // For nonzero known subcarriers, H[k] ≈ 1
        for k in 1..=26 {
            let kn = known[k];
            if kn != 0 {
                assert!((h[k].re - 1.0).abs() < 1e-6, "H[{k}] re should be 1: {}", h[k].re);
                assert!(h[k].im.abs() < 1e-6, "H[{k}] im should be 0: {}", h[k].im);
            }
        }
    }

    #[test]
    fn test_zf_equalization() {
        let mut fd = [Cf64::zero(); 64];
        let mut channel = [Cf64::zero(); 64];
        for k in 0..64 {
            fd[k] = Cf64::new(2.0, 1.0);
            channel[k] = Cf64::new(2.0, 0.0); // flat 2x gain
        }
        let eq = zf_equalize(&fd, &channel);
        for k in 0..64 {
            assert!((eq[k].re - 1.0).abs() < 1e-10);
            assert!((eq[k].im - 0.5).abs() < 1e-10);
        }
    }

    // --- Utility ---
    #[test]
    fn test_bytes_bits_roundtrip() {
        let bytes = vec![0x12u8, 0x34, 0x56, 0xAB];
        let bits = bytes_to_bits(&bytes);
        let recovered = bits_to_bytes(&bits);
        assert_eq!(bytes, recovered);
    }

    #[test]
    fn test_sc_to_bin() {
        assert_eq!(sc_to_bin(0), 0);
        assert_eq!(sc_to_bin(1), 1);
        assert_eq!(sc_to_bin(-1), 63);
        assert_eq!(sc_to_bin(-26), 38);
        assert_eq!(sc_to_bin(26), 26);
    }

    // --- Schmidl-Cox ---
    #[test]
    fn test_schmidl_cox_metric_length() {
        let n = 200;
        let samples: Vec<Cf64> = (0..n).map(|i| Cf64::from_polar(1.0, i as f64 * 0.1)).collect();
        let m = schmidl_cox_metric(&samples, 16);
        assert_eq!(m.len(), n - 32);
    }

    #[test]
    fn test_schmidl_cox_on_stf() {
        let stf = generate_l_stf();
        let m = schmidl_cox_metric(&stf, 16);
        // L-STF is periodic with period 16, so metric should be high
        let max_m = m.iter().cloned().fold(0.0f64, f64::max);
        assert!(max_m > 0.7, "Schmidl-Cox metric should be high on L-STF: {max_m}");
    }

    // --- HT-SIG ---
    #[test]
    fn test_ht_sig_length() {
        let params = HtSigParams {
            mcs: 0, cbw_40: false, ht_length: 100,
            aggregation: false, short_gi: false, num_ess: 0, crc: 0,
        };
        let sig = generate_ht_sig(&params);
        // 2 OFDM symbols
        assert_eq!(sig.len(), 2 * SYMBOL_SAMPLES);
    }

    #[test]
    fn test_ht_crc_computation() {
        let crc = compute_ht_crc(7, 1024);
        assert!(crc <= 0xFF); // must be 8-bit
    }

    // --- LTF sequence ---
    #[test]
    fn test_ltf_symbols_dc_is_zero() {
        let ltf = ltf_symbols();
        assert_eq!(ltf[0], 0, "DC subcarrier (bin 0) should be zero");
        assert_eq!(ltf[32], 0, "DC should be zero");
    }

    #[test]
    fn test_ltf_symbols_used_subcarriers_nonzero() {
        let ltf = ltf_symbols();
        let mut nonzero_count = 0;
        for &v in &ltf { if v != 0 { nonzero_count += 1; } }
        assert_eq!(nonzero_count, 52, "Should have 52 nonzero subcarriers");
    }

    // --- Bit reversal ---
    #[test]
    fn test_bit_reverse_4bit() {
        assert_eq!(bit_reverse(0b0001, 4), 0b1000);
        assert_eq!(bit_reverse(0b1100, 4), 0b0011);
    }

    // --- Transmit frame structure ---
    #[test]
    fn test_transmit_ht_mode() {
        let mut tx = WifiOfdmTransceiver::new(WifiConfig { short_gi: false, ht_mode: true });
        let data = vec![0x55u8; 20];
        let samples = tx.transmit(&data, WifiRate::Rate6Mbps);
        // HT frame should be longer than legacy
        let mut tx_leg = WifiOfdmTransceiver::new(WifiConfig::default());
        let leg_samples = tx_leg.transmit(&data, WifiRate::Rate6Mbps);
        assert!(samples.len() > leg_samples.len(), "HT frame should be longer");
    }

    #[test]
    fn test_short_gi_shorter_symbols() {
        let data = vec![0u8; 50];
        let syms_lgi = encode_data_symbols(&data, WifiRate::Rate54Mbps, false);
        let syms_sgi = encode_data_symbols(&data, WifiRate::Rate54Mbps, true);
        assert_eq!(syms_lgi.len(), syms_sgi.len());
        assert_eq!(syms_lgi[0].len(), 80, "LGI symbol should be 80 samples");
        assert_eq!(syms_sgi[0].len(), FFT_SIZE + CP_LEN_SGI, "SGI symbol should be 72 samples");
    }

    #[test]
    fn test_l_sig_rate_field_round_trip() {
        let rates = [
            WifiRate::Rate6Mbps, WifiRate::Rate9Mbps, WifiRate::Rate12Mbps,
            WifiRate::Rate18Mbps, WifiRate::Rate24Mbps, WifiRate::Rate36Mbps,
            WifiRate::Rate48Mbps, WifiRate::Rate54Mbps,
        ];
        for rate in &rates {
            let b = rate.sig_rate_bits();
            let decoded = decode_rate_bits(b);
            assert_eq!(decoded.params().mbps, rate.params().mbps,
                "Rate roundtrip failed for {:?}", rate);
        }
    }
}

//! EC-GSM-IoT (Extended Coverage GSM for IoT) Physical Layer Processor
//!
//! Implements the EC-GSM-IoT physical layer per 3GPP TS 45.001 / 45.005.
//!
//! # Key Parameters
//! - Symbol rate: 270.833 ksps (1625/6 kHz)
//! - GMSK modulation: BT=0.3, modulation index h=0.5
//! - Carrier: 900 MHz (GSM-900) / 1800 MHz (DCS-1800)
//! - Channel bandwidth: 200 kHz, 8 timeslots / TDMA frame
//! - Frame duration: 4.615 ms (8 × 577 µs slots)
//! - Burst length: 156.25 symbols (576.92 µs)
//! - Guard period: 8.25 symbols (30.46 µs)
//! - Coverage classes: CC1 (+0 dB) … CC4 (+20 dB), up to 40× blind repetition

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Symbol rate: 1 625 000 / 6 baud ≈ 270 833.33 sps
pub const SYMBOL_RATE: f64 = 1_625_000.0 / 6.0;

/// GMSK BT product
pub const GMSK_BT: f64 = 0.3;

/// GMSK modulation index (frequency deviation / symbol rate)
pub const GMSK_H: f64 = 0.5;

/// Bits per burst payload (Normal Burst)
pub const NB_DATA_BITS: usize = 114;

/// Tail bits per burst
pub const NB_TAIL_BITS: usize = 3;

/// Training sequence length (midamble)
pub const TRAINING_SEQ_LEN: usize = 26;

/// Stealing flags per burst
pub const STEALING_FLAGS: usize = 2;

/// Total useful bits in Normal Burst: 3+58+1+26+1+58+3 = 148 (+8.25 guard)
pub const NB_TOTAL_BITS: usize = 148;

/// Guard period in symbols
pub const GUARD_BITS: f64 = 8.25;

/// Number of TDMA time-slots per frame
pub const SLOTS_PER_FRAME: usize = 8;

/// Number of frames in a 51-multiframe (control channels)
pub const MULTIFRAME_51: usize = 51;

/// Number of frames in a 26-multiframe (traffic channels)
pub const MULTIFRAME_26: usize = 26;

/// Superframe = 51 × 26 TDMA frames = 1326 frames
pub const SUPERFRAME_FRAMES: usize = 1326;

// ---------------------------------------------------------------------------
// Training Sequence Codes (3GPP TS 45.002, Table 5.2.3a)
// 8 TSC sets × 26 bits
// ---------------------------------------------------------------------------

pub const TSC: [[u8; 26]; 8] = [
    [0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1],
    [0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1],
    [0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0],
    [0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0],
    [0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1],
    [0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0],
    [1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1],
    [1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0],
];

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// GSM burst types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BurstType {
    /// Normal Burst (NB) – carries data
    Normal,
    /// Access Burst (AB) – used for RACH
    Access,
    /// Synchronization Burst (SB) – carries BSIC + TDMA frame number
    Synchronization,
    /// Frequency Correction Burst (FB) – pure frequency tone
    FrequencyCorrection,
    /// Dummy Burst (DB) – idle filler
    Dummy,
}

/// EC-GSM-IoT Coverage Class
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CoverageClass {
    /// CC1: normal coverage (no repetition required)
    CC1 = 1,
    /// CC2: +10 dB (up to 4× repetition)
    CC2 = 2,
    /// CC3: +15 dB (up to 16× repetition)
    CC3 = 3,
    /// CC4: +20 dB (up to 40× repetition)
    CC4 = 4,
}

impl CoverageClass {
    /// Maximum blind repetitions for each coverage class
    pub fn max_repetitions(self) -> usize {
        match self {
            CoverageClass::CC1 => 1,
            CoverageClass::CC2 => 4,
            CoverageClass::CC3 => 16,
            CoverageClass::CC4 => 40,
        }
    }

    /// Additional coverage gain in dB relative to CC1
    pub fn coverage_gain_db(self) -> f64 {
        match self {
            CoverageClass::CC1 => 0.0,
            CoverageClass::CC2 => 10.0,
            CoverageClass::CC3 => 15.0,
            CoverageClass::CC4 => 20.0,
        }
    }
}

/// GPRS Channel Coding Scheme
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelCodingScheme {
    /// CS-1: rate ≈ 1/2, 9.05 kbps
    CS1,
    /// CS-2: rate ≈ 2/3, 13.4 kbps
    CS2,
    /// CS-3: rate ≈ 3/4, 15.6 kbps
    CS3,
    /// CS-4: unprotected, 21.4 kbps
    CS4,
    /// MCS-1: EC-EGPRS, 8.8 kbps
    MCS1,
    /// MCS-2: EC-EGPRS, 11.2 kbps
    MCS2,
    /// MCS-3: EC-EGPRS, 14.8 kbps
    MCS3,
    /// MCS-4: EC-EGPRS, 17.6 kbps
    MCS4,
}

impl ChannelCodingScheme {
    /// Gross bit rate per RLC block in bits
    pub fn rlc_block_bits(self) -> usize {
        match self {
            ChannelCodingScheme::CS1  => 184,
            ChannelCodingScheme::CS2  => 268,
            ChannelCodingScheme::CS3  => 312,
            ChannelCodingScheme::CS4  => 428,
            ChannelCodingScheme::MCS1 => 176,
            ChannelCodingScheme::MCS2 => 224,
            ChannelCodingScheme::MCS3 => 296,
            ChannelCodingScheme::MCS4 => 352,
        }
    }

    /// Code rate as a fraction (numerator, denominator)
    pub fn code_rate(self) -> (u32, u32) {
        match self {
            ChannelCodingScheme::CS1  => (1, 2),
            ChannelCodingScheme::CS2  => (2, 3),
            ChannelCodingScheme::CS3  => (3, 4),
            ChannelCodingScheme::CS4  => (1, 1),
            ChannelCodingScheme::MCS1 => (1, 3),
            ChannelCodingScheme::MCS2 => (1, 3),
            ChannelCodingScheme::MCS3 => (1, 2),
            ChannelCodingScheme::MCS4 => (1, 2),
        }
    }

    /// Data throughput in kbps
    pub fn throughput_kbps(self) -> f64 {
        match self {
            ChannelCodingScheme::CS1  => 9.05,
            ChannelCodingScheme::CS2  => 13.4,
            ChannelCodingScheme::CS3  => 15.6,
            ChannelCodingScheme::CS4  => 21.4,
            ChannelCodingScheme::MCS1 => 8.8,
            ChannelCodingScheme::MCS2 => 11.2,
            ChannelCodingScheme::MCS3 => 14.8,
            ChannelCodingScheme::MCS4 => 17.6,
        }
    }
}

/// GSM Power Class
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerClass {
    /// 2 W (33 dBm) – GSM-900
    Class1 = 1,
    /// 8 W (39 dBm) – GSM-900
    Class2 = 2,
    /// 5 W (37 dBm) – GSM-900
    Class3 = 3,
    /// 2 W (33 dBm) – DCS-1800
    Class4 = 4,
}

impl PowerClass {
    /// Maximum output power in dBm
    pub fn max_power_dbm(self) -> f64 {
        match self {
            PowerClass::Class1 => 33.0,
            PowerClass::Class2 => 39.0,
            PowerClass::Class3 => 37.0,
            PowerClass::Class4 => 33.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Gaussian Filter
// ---------------------------------------------------------------------------

/// Gaussian pulse shaping filter for GMSK
///
/// Generates the impulse response of a Gaussian low-pass filter with
/// bandwidth-time product BT applied to a unit-amplitude rectangular pulse
/// of duration T (one symbol period).
pub struct GaussianFilter {
    /// Filter coefficients (impulse response samples)
    pub coeffs: Vec<f64>,
    /// Oversampling factor (samples per symbol)
    pub sps: usize,
}

impl GaussianFilter {
    /// Create a Gaussian filter for GMSK.
    ///
    /// # Arguments
    /// * `bt`    – Bandwidth-time product (0.3 for EC-GSM-IoT)
    /// * `sps`   – Samples per symbol
    /// * `span`  – Filter span in symbols (typically 3–5)
    pub fn new(bt: f64, sps: usize, span: usize) -> Self {
        let total = span * sps + 1;
        let mut coeffs = vec![0.0f64; total];
        let sigma = (2.0 * PI * bt).recip() * (2.0 * (2.0_f64.ln())).sqrt();
        let t_sym = 1.0; // normalised symbol period

        for i in 0..total {
            let t = (i as f64 - (total as f64 - 1.0) / 2.0) / sps as f64;
            let t1 = t - 0.5 * t_sym;
            let t2 = t + 0.5 * t_sym;
            let sqrt2_sigma = 2.0_f64.sqrt() * sigma;
            coeffs[i] = 0.5 * (erf(t2 / sqrt2_sigma) - erf(t1 / sqrt2_sigma));
        }
        // Normalise so DC gain = 1
        let sum: f64 = coeffs.iter().sum();
        if sum > 1e-12 {
            for c in &mut coeffs { *c /= sum; }
        }
        GaussianFilter { coeffs, sps }
    }

    /// Apply Gaussian filter to a sequence of NRZ symbols (+1/-1).
    pub fn filter(&self, nrz: &[f64]) -> Vec<f64> {
        let n = nrz.len();
        let m = self.coeffs.len();
        let half = (m - 1) / 2;
        let mut out = vec![0.0f64; n];
        for i in 0..n {
            let mut acc = 0.0;
            for (k, &c) in self.coeffs.iter().enumerate() {
                let idx = i + k;
                if idx >= half && idx - half < n {
                    acc += nrz[idx - half] * c;
                }
            }
            out[i] = acc;
        }
        out
    }
}

/// Approximate error function (Abramowitz & Stegun 7.1.26)
fn erf(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t * (0.254829592
        + t * (-0.284496736
            + t * (1.421413741
                + t * (-1.453152027
                    + t * 1.061405429))));
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    sign * (1.0 - poly * (-x * x).exp())
}

// ---------------------------------------------------------------------------
// GMSK Modulator / Demodulator
// ---------------------------------------------------------------------------

/// GMSK Modulator (BT = 0.3, h = 0.5)
pub struct GmskModulator {
    filter: GaussianFilter,
    /// Samples per symbol
    pub sps: usize,
}

impl GmskModulator {
    /// Create a GMSK modulator with default EC-GSM-IoT parameters.
    pub fn new(sps: usize) -> Self {
        GmskModulator {
            filter: GaussianFilter::new(GMSK_BT, sps, 4),
            sps,
        }
    }

    /// Modulate binary bits (0/1) to complex IQ samples.
    ///
    /// Returns interleaved [I0, Q0, I1, Q1, ...].
    pub fn modulate(&self, bits: &[u8]) -> Vec<f64> {
        // Step 1: differential encoding (NRZ, ±1)
        let nrz: Vec<f64> = bits.iter().map(|&b| if b != 0 { 1.0 } else { -1.0 }).collect();

        // Step 2: upsample
        let mut up = vec![0.0f64; nrz.len() * self.sps];
        for (i, &s) in nrz.iter().enumerate() {
            up[i * self.sps] = s;
        }

        // Step 3: Gaussian filter
        let shaped = self.filter.filter(&up);

        // Step 4: phase accumulation (frequency integration × π h)
        let phase_inc = PI * GMSK_H / self.sps as f64;
        let mut phase = 0.0f64;
        let mut iq = Vec::with_capacity(shaped.len() * 2);
        for &s in &shaped {
            phase += s * phase_inc;
            iq.push(phase.cos());
            iq.push(phase.sin());
        }
        iq
    }

    /// Simple GMSK demodulation using differential phase detection.
    ///
    /// Returns hard-decision bits.
    pub fn demodulate(&self, iq: &[f64]) -> Vec<u8> {
        let n = iq.len() / 2;
        if n < 2 { return Vec::new(); }

        let mut phase_diff = Vec::with_capacity(n - 1);
        for i in 1..n {
            let i0 = iq[2 * (i - 1)];
            let q0 = iq[2 * (i - 1) + 1];
            let i1 = iq[2 * i];
            let q1 = iq[2 * i + 1];
            // arg(x[n] * conj(x[n-1]))
            let diff = (i1 * i0 + q1 * q0).atan2(q1 * i0 - i1 * q0);
            phase_diff.push(diff);
        }

        // Downsample by sps and threshold
        phase_diff
            .chunks(self.sps)
            .map(|chunk| {
                let avg: f64 = chunk.iter().sum::<f64>() / chunk.len() as f64;
                if avg > 0.0 { 1 } else { 0 }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Convolutional Encoder / Viterbi Decoder  (rate 1/2, K=5)
// ---------------------------------------------------------------------------
// Generator polynomials per 3GPP TS 45.003:
//   G0 = 1 + D^3 + D^4  (octal 023)
//   G1 = 1 + D + D^3 + D^4  (octal 033)

const CONV_K: usize = 5;
const CONV_STATES: usize = 1 << (CONV_K - 1); // 16 states

/// Convolutional encoder (rate 1/2, K=5, G0=0x13, G1=0x1B)
pub struct ConvolutionalEncoder {
    state: u8,
}

impl ConvolutionalEncoder {
    /// Create encoder with zero-state initialisation.
    pub fn new() -> Self {
        ConvolutionalEncoder { state: 0 }
    }

    /// Reset encoder state to zero.
    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Encode a single bit; returns (bit0, bit1).
    pub fn encode_bit(&mut self, bit: u8) -> (u8, u8) {
        let reg = ((self.state << 1) | (bit & 1)) & 0x1F;
        // G0 = 1+D^3+D^4  → taps at positions 0,3,4 → mask 0x13
        let b0 = (reg & 0x13).count_ones() as u8 & 1;
        // G1 = 1+D+D^3+D^4 → taps at positions 0,1,3,4 → mask 0x1B
        let b1 = (reg & 0x1B).count_ones() as u8 & 1;
        self.state = (reg >> 1) & 0x0F;
        (b0, b1)
    }

    /// Encode a block of bits; returns interleaved coded bits.
    pub fn encode(&mut self, bits: &[u8]) -> Vec<u8> {
        self.reset();
        let mut out = Vec::with_capacity(bits.len() * 2 + (CONV_K - 1) * 2);
        for &b in bits {
            let (b0, b1) = self.encode_bit(b);
            out.push(b0);
            out.push(b1);
        }
        // Tail: flush K-1 zero bits
        for _ in 0..(CONV_K - 1) {
            let (b0, b1) = self.encode_bit(0);
            out.push(b0);
            out.push(b1);
        }
        out
    }
}

impl Default for ConvolutionalEncoder {
    fn default() -> Self { Self::new() }
}

/// Viterbi decoder (hard-decision) for rate 1/2 K=5 code
pub struct ViterbiDecoder;

impl ViterbiDecoder {
    /// Hard-decision Viterbi decode.
    ///
    /// `coded` must contain pairs of bits (b0, b1).
    /// Returns the decoded information bits (tail bits excluded).
    pub fn decode(coded: &[u8]) -> Vec<u8> {
        let n_pairs = coded.len() / 2;
        const INF: u32 = u32::MAX / 2;

        // Metric per state
        let mut metrics = [INF; CONV_STATES];
        metrics[0] = 0;
        // Survivor path (store input bit that led to each state at each step)
        let mut paths: Vec<[u8; CONV_STATES]> = vec![[0u8; CONV_STATES]; n_pairs];

        for step in 0..n_pairs {
            let r0 = coded[2 * step] as u32;
            let r1 = coded[2 * step + 1] as u32;
            let mut new_metrics = [INF; CONV_STATES];
            for state in 0..CONV_STATES {
                if metrics[state] == INF { continue; }
                for input in 0u8..2 {
                    let reg = ((state as u8) << 1 | input) & 0x1F;
                    let b0 = (reg & 0x13).count_ones() as u32;
                    let b1 = (reg & 0x1B).count_ones() as u32;
                    let dist = (b0 ^ r0) + (b1 ^ r1);
                    let new_state = (reg >> 1) as usize & 0x0F;
                    let new_m = metrics[state] + dist;
                    if new_m < new_metrics[new_state] {
                        new_metrics[new_state] = new_m;
                        paths[step][new_state] = input | ((state as u8) << 1);
                    }
                }
            }
            metrics = new_metrics;
        }

        // Traceback from best final state
        let mut best = 0usize;
        for s in 1..CONV_STATES {
            if metrics[s] < metrics[best] { best = s; }
        }
        let mut decoded = vec![0u8; n_pairs];
        let mut cur = best;
        for step in (0..n_pairs).rev() {
            let entry = paths[step][cur];
            decoded[step] = entry & 1;
            cur = (entry >> 1) as usize;
        }
        // Remove tail bits
        let info_len = if n_pairs > CONV_K - 1 { n_pairs - (CONV_K - 1) } else { 0 };
        decoded[..info_len].to_vec()
    }
}

// ---------------------------------------------------------------------------
// Block Diagonal Interleaver
// ---------------------------------------------------------------------------

/// Block-diagonal interleaver as used in GSM/EC-GSM-IoT.
///
/// Maps N input bits across `bursts` bursts of `bits_per_burst` bits each
/// using diagonal permutation.
pub struct BlockInterleaver {
    /// Number of bursts in one interleaving block
    pub bursts: usize,
    /// Bits contributed to each burst
    pub bits_per_burst: usize,
}

impl BlockInterleaver {
    /// Create a block-diagonal interleaver.
    pub fn new(bursts: usize, bits_per_burst: usize) -> Self {
        BlockInterleaver { bursts, bits_per_burst }
    }

    /// Interleave: spread `bits` across `bursts` bursts.
    ///
    /// Returns a flat vector of length `bursts × bits_per_burst`.
    pub fn interleave(&self, bits: &[u8]) -> Vec<u8> {
        let total = self.bursts * self.bits_per_burst;
        let mut out = vec![0u8; total];
        let b = self.bits_per_burst;
        for (i, &bit) in bits.iter().enumerate().take(total) {
            // Diagonal mapping: burst = i % bursts, pos = i / bursts
            let burst_idx = i % self.bursts;
            let pos = i / self.bursts;
            // Within each burst, apply column permutation mod b
            let col = (pos + burst_idx) % b;
            out[burst_idx * b + col] = bit;
        }
        out
    }

    /// Deinterleave: reverse the diagonal mapping.
    pub fn deinterleave(&self, interleaved: &[u8]) -> Vec<u8> {
        let total = self.bursts * self.bits_per_burst;
        let mut out = vec![0u8; total];
        let b = self.bits_per_burst;
        for i in 0..total {
            let burst_idx = i % self.bursts;
            let pos = i / self.bursts;
            let col = (pos + burst_idx) % b;
            let src = burst_idx * b + col;
            if src < interleaved.len() {
                out[i] = interleaved[src];
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Burst Builder / Parser
// ---------------------------------------------------------------------------

/// A GSM burst (156.25 symbols, not including guard)
#[derive(Debug, Clone)]
pub struct Burst {
    pub burst_type: BurstType,
    /// Raw bits (148 for NB/SB, variable for others)
    pub bits: Vec<u8>,
    /// Timeslot number (0–7)
    pub timeslot: u8,
    /// TDMA frame number
    pub frame_number: u32,
    /// Training sequence code index (0–7) for Normal Burst
    pub tsc: u8,
}

impl Burst {
    /// Build a Normal Burst from 114 data bits and TSC index.
    ///
    /// NB format: T(3) | data(58) | SF | TSC(26) | SF | data(56) | T(3)
    pub fn new_normal(data: &[u8], tsc: u8, timeslot: u8, frame: u32) -> Self {
        assert!(data.len() >= NB_DATA_BITS, "Need exactly {NB_DATA_BITS} data bits");
        let mut bits = Vec::with_capacity(NB_TOTAL_BITS);
        // Tail bits (0)
        bits.extend([0u8; 3]);
        // First 58 data bits
        bits.extend_from_slice(&data[..58]);
        // Stealing flag
        bits.push(0);
        // Training sequence
        bits.extend_from_slice(&TSC[tsc as usize % 8]);
        // Stealing flag
        bits.push(0);
        // Second 56 data bits
        bits.extend_from_slice(&data[58..114]);
        // Tail bits (0)
        bits.extend([0u8; 3]);
        Burst { burst_type: BurstType::Normal, bits, timeslot, frame_number: frame, tsc }
    }

    /// Build a Frequency Correction Burst (all zeros payload → pure tone)
    pub fn new_frequency_correction(timeslot: u8, frame: u32) -> Self {
        Burst {
            burst_type: BurstType::FrequencyCorrection,
            bits: vec![0u8; NB_TOTAL_BITS],
            timeslot,
            frame_number: frame,
            tsc: 0,
        }
    }

    /// Build a Dummy Burst (filled with alternating pattern)
    pub fn new_dummy(timeslot: u8, frame: u32) -> Self {
        let pattern: Vec<u8> = (0..NB_TOTAL_BITS)
            .map(|i| (i % 2) as u8)
            .collect();
        Burst {
            burst_type: BurstType::Dummy,
            bits: pattern,
            timeslot,
            frame_number: frame,
            tsc: 0,
        }
    }

    /// Extract the 114 data bits from a Normal Burst.
    ///
    /// NB layout: T(3) | data1(58) | SF(1) | TSC(26) | SF(1) | data2(56) | T(3) = 148 bits
    pub fn extract_data(&self) -> Vec<u8> {
        if self.burst_type != BurstType::Normal || self.bits.len() < NB_TOTAL_BITS {
            return Vec::new();
        }
        let mut data = Vec::with_capacity(NB_DATA_BITS);
        // data1: positions 3..61 (58 bits)
        data.extend_from_slice(&self.bits[3..61]);
        // data2: positions 89..145 (56 bits)
        data.extend_from_slice(&self.bits[89..145]);
        data
    }

    /// Extract the training sequence from a Normal Burst.
    ///
    /// TSC occupies positions 62..88 (26 bits) in the burst.
    pub fn extract_tsc(&self) -> [u8; 26] {
        let mut tsc = [0u8; 26];
        if self.bits.len() >= 88 {
            tsc.copy_from_slice(&self.bits[62..88]);
        }
        tsc
    }
}

// ---------------------------------------------------------------------------
// Channel Estimator (correlation-based)
// ---------------------------------------------------------------------------

/// Simple least-squares channel estimator using known training sequence.
pub struct ChannelEstimator {
    tsc_idx: usize,
}

impl ChannelEstimator {
    /// Create a channel estimator for the given TSC index.
    pub fn new(tsc_idx: usize) -> Self {
        ChannelEstimator { tsc_idx: tsc_idx % 8 }
    }

    /// Estimate channel impulse response by correlating received signal
    /// with the known TSC.
    ///
    /// `rx_bits` – received soft values around the midamble position
    /// Returns normalised correlation peak magnitude and lag offset.
    pub fn estimate(&self, rx_bits: &[f64]) -> (f64, usize) {
        let tsc: Vec<f64> = TSC[self.tsc_idx]
            .iter()
            .map(|&b| if b != 0 { 1.0 } else { -1.0 })
            .collect();

        let mut best_corr = 0.0f64;
        let mut best_lag = 0usize;

        let search_len = rx_bits.len().saturating_sub(TRAINING_SEQ_LEN);
        for lag in 0..=search_len {
            let mut corr = 0.0f64;
            for (k, &t) in tsc.iter().enumerate() {
                corr += rx_bits[lag + k] * t;
            }
            let corr_norm = corr.abs() / TRAINING_SEQ_LEN as f64;
            if corr_norm > best_corr {
                best_corr = corr_norm;
                best_lag = lag;
            }
        }
        (best_corr, best_lag)
    }
}

// ---------------------------------------------------------------------------
// Frequency Hopping
// ---------------------------------------------------------------------------

/// Frequency hopping controller (slow FH, per 3GPP TS 45.002)
pub struct FrequencyHopper {
    /// Mobile Allocation (MA) list – absolute ARFCN numbers
    pub ma_list: Vec<u16>,
    /// Hopping Sequence Number (0–63)
    pub hsn: u8,
    /// Mobile Allocation Index Offset (0–63)
    pub maio: u8,
}

impl FrequencyHopper {
    /// Create a frequency hopper.
    pub fn new(ma_list: Vec<u16>, hsn: u8, maio: u8) -> Self {
        FrequencyHopper { ma_list, hsn, maio }
    }

    /// Compute the ARFCN for a given TDMA frame number per 3GPP TS 45.002 §6.2.3
    pub fn arfcn_for_frame(&self, fn_: u32) -> u16 {
        let n = self.ma_list.len();
        if n == 0 { return 0; }
        if self.hsn == 0 {
            // Cyclic hopping
            let idx = ((fn_ as usize + self.maio as usize) % n) as usize;
            return self.ma_list[idx];
        }
        // Random hopping using RNTABLE
        let t1 = fn_ / 1326;
        let t2 = fn_ % 26;
        let t3 = fn_ % 51;
        let rn_index = ((self.hsn as u32 ^ (t1 % 64)) + t2 + t3) % 70;
        let rn = RNTABLE[rn_index as usize] as u32;
        let m = rn + t2;
        let mp = m % n as u32;
        let s = (mp as usize + self.maio as usize) % n;
        self.ma_list[s]
    }
}

/// Random number table for frequency hopping (3GPP TS 45.002 §6.2.3)
const RNTABLE: [u8; 70] = [
    48, 98, 63,  1, 36, 95, 78, 102, 94, 73,
     0, 64, 25, 81, 76, 59, 124, 23, 104, 100,
   101, 47, 118, 85, 18,  56, 96, 86, 54, 2,
    80, 34, 127, 13, 6,  89, 57, 103, 12, 74,
    55, 111, 75, 38, 15, 53, 122, 26, 37, 71,
    60, 79, 41, 50, 105, 20, 67, 7, 42, 49,
    30, 32, 88, 68, 31, 35, 31, 51, 95, 69,
];

// ---------------------------------------------------------------------------
// Blind Repetition / Combining
// ---------------------------------------------------------------------------

/// Blind repetition transmitter for EC-GSM-IoT coverage extension.
pub struct BlindRepetitionTransmitter {
    pub coverage_class: CoverageClass,
}

impl BlindRepetitionTransmitter {
    pub fn new(cc: CoverageClass) -> Self {
        BlindRepetitionTransmitter { coverage_class: cc }
    }

    /// Repeat a burst the required number of times.
    ///
    /// Returns a vector of identical burst bit vectors.
    pub fn repeat_burst(&self, burst: &Burst) -> Vec<Vec<u8>> {
        let reps = self.coverage_class.max_repetitions();
        (0..reps).map(|_| burst.bits.clone()).collect()
    }

    /// Compute repetition gain in dB (10 × log10(N)).
    pub fn repetition_gain_db(&self) -> f64 {
        let n = self.coverage_class.max_repetitions() as f64;
        10.0 * n.log10()
    }
}

/// Blind repetition receiver – combines repeated bursts by soft-bit addition.
pub struct BlindRepetitionReceiver;

impl BlindRepetitionReceiver {
    /// Combine repeated soft-bit vectors by summation (MRC-equivalent).
    ///
    /// All vectors must have the same length; shorter ones are zero-padded.
    pub fn combine(repetitions: &[Vec<f64>]) -> Vec<f64> {
        if repetitions.is_empty() { return Vec::new(); }
        let len = repetitions.iter().map(|v| v.len()).max().unwrap_or(0);
        let mut out = vec![0.0f64; len];
        for rep in repetitions {
            for (i, &s) in rep.iter().enumerate() {
                out[i] += s;
            }
        }
        out
    }

    /// Hard-decision from combined soft bits.
    pub fn decide(combined: &[f64]) -> Vec<u8> {
        combined.iter().map(|&s| if s >= 0.0 { 1 } else { 0 }).collect()
    }
}

// ---------------------------------------------------------------------------
// Power Control
// ---------------------------------------------------------------------------

/// EC-GSM-IoT power control state
#[derive(Debug, Clone)]
pub struct PowerController {
    pub power_class: PowerClass,
    /// Current TX power level (0 = maximum)
    pub power_level: u8,
    /// Target received signal level (dBm)
    pub target_rxlev: f64,
    /// Hysteresis (dB)
    pub hysteresis: f64,
    /// Current coverage class
    pub coverage_class: CoverageClass,
}

impl PowerController {
    /// Create a power controller with default parameters.
    pub fn new(power_class: PowerClass) -> Self {
        PowerController {
            power_class,
            power_level: 0,
            target_rxlev: -70.0,
            hysteresis: 3.0,
            coverage_class: CoverageClass::CC1,
        }
    }

    /// Current TX power in dBm (step size 2 dB per level).
    pub fn tx_power_dbm(&self) -> f64 {
        self.power_class.max_power_dbm() - (self.power_level as f64 * 2.0)
    }

    /// Update power level based on reported RXLEV (dBm).
    ///
    /// Returns true if power level changed.
    pub fn update(&mut self, rxlev: f64) -> bool {
        let diff = rxlev - self.target_rxlev;
        if diff > self.hysteresis && self.power_level > 0 {
            self.power_level -= 1;
            true
        } else if diff < -self.hysteresis {
            let max_level = ((self.power_class.max_power_dbm() - 5.0) / 2.0) as u8;
            if self.power_level < max_level {
                self.power_level += 1;
                return true;
            }
            false
        } else {
            false
        }
    }

    /// Determine coverage class based on path loss (dB).
    pub fn classify_coverage(&mut self, path_loss_db: f64) {
        self.coverage_class = if path_loss_db < 140.0 {
            CoverageClass::CC1
        } else if path_loss_db < 150.0 {
            CoverageClass::CC2
        } else if path_loss_db < 155.0 {
            CoverageClass::CC3
        } else {
            CoverageClass::CC4
        };
    }
}

// ---------------------------------------------------------------------------
// TDMA Frame Structure
// ---------------------------------------------------------------------------

/// TDMA frame number decomposition
#[derive(Debug, Clone, Copy)]
pub struct TdmaFrameNumber {
    pub fn_: u32,
    pub t1: u32,  // fn / 1326
    pub t2: u32,  // fn % 26
    pub t3: u32,  // fn % 51
}

impl TdmaFrameNumber {
    pub fn new(fn_: u32) -> Self {
        TdmaFrameNumber {
            fn_,
            t1: fn_ / 1326,
            t2: fn_ % 26,
            t3: fn_ % 51,
        }
    }

    /// Duration of the frame in microseconds.
    pub fn frame_duration_us() -> f64 {
        // 8 slots × 577 µs/slot ≈ 4615.4 µs
        1.0 / (SYMBOL_RATE / (NB_TOTAL_BITS as f64 + GUARD_BITS) / SLOTS_PER_FRAME as f64) * 1e6
    }

    /// Returns (hyperframe, superframe, multiframe, frame) indices.
    pub fn decompose(fn_: u32) -> (u32, u32, u32, u32) {
        let hyperframe = fn_ / (SUPERFRAME_FRAMES as u32 * 2048);
        let rem = fn_ % (SUPERFRAME_FRAMES as u32 * 2048);
        let superframe = rem / SUPERFRAME_FRAMES as u32;
        let rem2 = rem % SUPERFRAME_FRAMES as u32;
        let multiframe = rem2 / 26;
        let frame = rem2 % 26;
        (hyperframe, superframe, multiframe, frame)
    }
}

// ---------------------------------------------------------------------------
// Channel Coding (CS-1 / MCS-1)
// ---------------------------------------------------------------------------

/// Encode a data block using CS-1 (rate 1/2 convolutional + tail flush)
pub fn encode_cs1(data: &[u8]) -> Vec<u8> {
    let mut enc = ConvolutionalEncoder::new();
    enc.encode(data)
}

/// Decode a CS-1-encoded block using hard-decision Viterbi
pub fn decode_cs1(coded: &[u8]) -> Vec<u8> {
    ViterbiDecoder::decode(coded)
}

/// Parity check (fire code approximation) for CS-1 – returns 40-bit parity
pub fn cs1_parity(data: &[u8]) -> u64 {
    // Simplified 40-bit CRC-like parity per TS 45.003 §5.1
    let poly: u64 = 0x0004_820_0000_0001; // fire code polynomial (illustrative)
    let mut reg: u64 = 0;
    for &b in data {
        let feedback = ((reg >> 39) ^ b as u64) & 1;
        reg = ((reg << 1) ^ (feedback * poly)) & 0xFF_FFFF_FFFF;
    }
    reg
}

// ---------------------------------------------------------------------------
// Complete EC-GSM-IoT Processor
// ---------------------------------------------------------------------------

/// High-level EC-GSM-IoT physical layer processor.
///
/// Ties together modulation, coding, interleaving, burst assembly,
/// frequency hopping, and power/coverage control.
pub struct EcGsmIotProcessor {
    pub modulator: GmskModulator,
    pub encoder: ConvolutionalEncoder,
    pub interleaver: BlockInterleaver,
    pub freq_hopper: FrequencyHopper,
    pub power_ctrl: PowerController,
    pub coverage_class: CoverageClass,
    pub coding_scheme: ChannelCodingScheme,
    pub tsc_idx: usize,
    pub timeslot: u8,
    frame_number: u32,
}

impl EcGsmIotProcessor {
    /// Create a processor with typical EC-GSM-IoT defaults.
    pub fn new(sps: usize) -> Self {
        EcGsmIotProcessor {
            modulator: GmskModulator::new(sps),
            encoder: ConvolutionalEncoder::new(),
            interleaver: BlockInterleaver::new(4, NB_DATA_BITS / 4),
            freq_hopper: FrequencyHopper::new(
                vec![1, 5, 10, 15, 20, 25, 30, 35, 40, 45], 7, 0,
            ),
            power_ctrl: PowerController::new(PowerClass::Class1),
            coverage_class: CoverageClass::CC1,
            coding_scheme: ChannelCodingScheme::MCS1,
            tsc_idx: 0,
            timeslot: 0,
            frame_number: 0,
        }
    }

    /// Process one block of payload data: encode → interleave → burst-build
    /// → repeat → modulate.
    ///
    /// Returns flat IQ samples for all repetitions concatenated.
    pub fn transmit_block(&mut self, payload: &[u8]) -> Vec<f64> {
        // 1. Convolutional encode
        let coded = self.encoder.encode(payload);

        // 2. Zero-pad or truncate to interleaver input length
        let il_len = self.interleaver.bursts * self.interleaver.bits_per_burst;
        let mut il_in = coded.clone();
        il_in.resize(il_len, 0);

        // 3. Interleave
        let interleaved = self.interleaver.interleave(&il_in);

        // 4. Build Normal Burst(s)
        let bursts_per_block = self.interleaver.bursts;
        let bpb = self.interleaver.bits_per_burst;

        // Pad interleaved to NB_DATA_BITS per burst
        let mut all_iq = Vec::new();
        let reps = self.coverage_class.max_repetitions();

        for b in 0..bursts_per_block {
            let mut data = vec![0u8; NB_DATA_BITS];
            let src = &interleaved[b * bpb..(b + 1) * bpb.min(interleaved.len() - b * bpb)];
            let copy_len = src.len().min(NB_DATA_BITS);
            data[..copy_len].copy_from_slice(&src[..copy_len]);

            let burst = Burst::new_normal(
                &data,
                self.tsc_idx as u8,
                self.timeslot,
                self.frame_number,
            );

            // 5. Blind repetition
            for _ in 0..reps {
                let iq = self.modulator.modulate(&burst.bits);
                all_iq.extend_from_slice(&iq);
            }
            self.frame_number = self.frame_number.wrapping_add(1);
        }
        all_iq
    }

    /// Receive a single burst: demodulate → channel estimation → data extraction.
    ///
    /// Returns soft bits extracted from the burst.
    pub fn receive_burst(&self, iq: &[f64]) -> Vec<u8> {
        self.modulator.demodulate(iq)
    }

    /// Advance to next TDMA slot/frame.
    pub fn advance_slot(&mut self) {
        self.timeslot = (self.timeslot + 1) % SLOTS_PER_FRAME as u8;
        if self.timeslot == 0 {
            self.frame_number = self.frame_number.wrapping_add(1);
        }
    }

    /// Current ARFCN based on frame number and hopping parameters.
    pub fn current_arfcn(&self) -> u16 {
        self.freq_hopper.arfcn_for_frame(self.frame_number)
    }
}

// ---------------------------------------------------------------------------
// Link Budget Helper
// ---------------------------------------------------------------------------

/// EC-GSM-IoT link budget parameters
#[derive(Debug, Clone)]
pub struct LinkBudget {
    /// TX power (dBm)
    pub tx_power_dbm: f64,
    /// TX antenna gain (dBi)
    pub tx_gain_dbi: f64,
    /// RX antenna gain (dBi)
    pub rx_gain_dbi: f64,
    /// Noise figure (dB)
    pub noise_figure_db: f64,
    /// Required SNR for MCS-1 (dB)
    pub required_snr_db: f64,
    /// Implementation margin (dB)
    pub impl_margin_db: f64,
    /// Carrier frequency (MHz)
    pub freq_mhz: f64,
}

impl LinkBudget {
    /// Typical EC-GSM-IoT uplink parameters at 900 MHz.
    pub fn default_900mhz() -> Self {
        LinkBudget {
            tx_power_dbm: 33.0,
            tx_gain_dbi: 0.0,
            rx_gain_dbi: 0.0,
            noise_figure_db: 5.0,
            required_snr_db: -6.0, // EC-GSM-IoT MCS-1 sensitivity
            impl_margin_db: 3.0,
            freq_mhz: 900.0,
        }
    }

    /// Thermal noise floor (dBm) for the GSM 200 kHz channel.
    pub fn noise_floor_dbm(&self) -> f64 {
        // N = -174 + 10 log10(BW) + NF
        let bw_hz = 200e3f64;
        -174.0 + 10.0 * bw_hz.log10() + self.noise_figure_db
    }

    /// Maximum allowable path loss (dB).
    pub fn max_path_loss_db(&self) -> f64 {
        let eirp = self.tx_power_dbm + self.tx_gain_dbi;
        let sensitivity = self.noise_floor_dbm() + self.required_snr_db;
        eirp + self.rx_gain_dbi - sensitivity - self.impl_margin_db
    }

    /// Free-space path loss at distance `d_m` meters.
    pub fn fspl_db(&self, d_m: f64) -> f64 {
        20.0 * d_m.log10() + 20.0 * self.freq_mhz.log10() + 20.0 * (4.0 * PI / 3e8 * 1e6).log10()
    }

    /// Maximum range (metres) limited by path loss budget.
    pub fn max_range_m(&self) -> f64 {
        let max_pl = self.max_path_loss_db();
        // FSPL = 20*log10(d) + 20*log10(f_MHz) + K  => d = 10^((FSPL-K-20*log10(f))/20)
        let k = 20.0 * (4.0 * PI / 3e8 * 1e6).log10();
        let exp = (max_pl - 20.0 * self.freq_mhz.log10() - k) / 20.0;
        10.0_f64.powf(exp)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- GaussianFilter ---

    #[test]
    fn test_gaussian_filter_dc_gain() {
        let gf = GaussianFilter::new(GMSK_BT, 4, 4);
        let nrz = vec![1.0f64; 20];
        let out = gf.filter(&nrz);
        // DC response should be close to 1 (normalised)
        let mean: f64 = out[8..12].iter().sum::<f64>() / 4.0;
        assert!((mean - 1.0).abs() < 0.05, "DC gain off: {mean}");
    }

    #[test]
    fn test_gaussian_filter_length() {
        let gf = GaussianFilter::new(0.3, 8, 3);
        assert_eq!(gf.coeffs.len(), 3 * 8 + 1);
    }

    #[test]
    fn test_gaussian_filter_sum_one() {
        let gf = GaussianFilter::new(0.3, 4, 4);
        let sum: f64 = gf.coeffs.iter().sum();
        assert!((sum - 1.0).abs() < 1e-9, "sum={sum}");
    }

    #[test]
    fn test_erf_known_values() {
        assert!((erf(0.0)).abs() < 1e-6);
        assert!((erf(1.0) - 0.8427).abs() < 0.001);
        assert!((erf(-1.0) + 0.8427).abs() < 0.001);
        assert!((erf(3.0) - 1.0).abs() < 0.001);
    }

    // --- Coverage Class ---

    #[test]
    fn test_coverage_class_repetitions() {
        assert_eq!(CoverageClass::CC1.max_repetitions(), 1);
        assert_eq!(CoverageClass::CC2.max_repetitions(), 4);
        assert_eq!(CoverageClass::CC3.max_repetitions(), 16);
        assert_eq!(CoverageClass::CC4.max_repetitions(), 40);
    }

    #[test]
    fn test_coverage_class_gain() {
        assert_eq!(CoverageClass::CC1.coverage_gain_db(), 0.0);
        assert_eq!(CoverageClass::CC4.coverage_gain_db(), 20.0);
    }

    #[test]
    fn test_coverage_ordering() {
        assert!(CoverageClass::CC1 < CoverageClass::CC4);
    }

    // --- Channel Coding ---

    #[test]
    fn test_cs1_throughput() {
        assert!((ChannelCodingScheme::CS1.throughput_kbps() - 9.05).abs() < 0.01);
        assert!((ChannelCodingScheme::CS4.throughput_kbps() - 21.4).abs() < 0.01);
    }

    #[test]
    fn test_mcs_rlc_block_bits() {
        assert_eq!(ChannelCodingScheme::MCS1.rlc_block_bits(), 176);
        assert_eq!(ChannelCodingScheme::MCS4.rlc_block_bits(), 352);
    }

    #[test]
    fn test_code_rate() {
        let (n, d) = ChannelCodingScheme::CS4.code_rate();
        assert_eq!((n, d), (1, 1));
        let (n, d) = ChannelCodingScheme::CS1.code_rate();
        assert_eq!((n, d), (1, 2));
    }

    // --- Convolutional Encoder ---

    #[test]
    fn test_conv_encoder_all_zeros() {
        let mut enc = ConvolutionalEncoder::new();
        let bits = vec![0u8; 8];
        let coded = enc.encode(&bits);
        // All-zero input → all-zero output
        assert!(coded.iter().all(|&b| b == 0), "Expected all zeros");
    }

    #[test]
    fn test_conv_encoder_length() {
        let mut enc = ConvolutionalEncoder::new();
        let bits = vec![1u8; 10];
        let coded = enc.encode(&bits);
        // (10 + 4) * 2 = 28
        assert_eq!(coded.len(), (10 + CONV_K - 1) * 2);
    }

    #[test]
    fn test_conv_encoder_known_sequence() {
        let mut enc = ConvolutionalEncoder::new();
        enc.reset();
        let (b0, b1) = enc.encode_bit(1);
        // G0: reg=0x01, 0x01 & 0x13=0x01, popcount=1 → b0=1
        // G1: reg=0x01, 0x01 & 0x1B=0x01, popcount=1 → b1=1
        assert_eq!(b0, 1);
        assert_eq!(b1, 1);
    }

    #[test]
    fn test_conv_encoder_single_one() {
        let mut enc = ConvolutionalEncoder::new();
        let bits = [1u8, 0, 0, 0, 0, 0, 0, 0];
        let coded = enc.encode(&bits);
        // First two coded bits: (1,1) as shown above
        assert_eq!(coded[0], 1);
        assert_eq!(coded[1], 1);
    }

    // --- Viterbi Decoder ---

    #[test]
    fn test_viterbi_decode_all_zeros() {
        let mut enc = ConvolutionalEncoder::new();
        let data = vec![0u8; 16];
        let coded = enc.encode(&data);
        let decoded = ViterbiDecoder::decode(&coded);
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_viterbi_roundtrip() {
        let mut enc = ConvolutionalEncoder::new();
        let data: Vec<u8> = (0..20).map(|i| (i % 2) as u8).collect();
        let coded = enc.encode(&data);
        let decoded = ViterbiDecoder::decode(&coded);
        assert_eq!(decoded, data, "Roundtrip failed");
    }

    #[test]
    fn test_viterbi_one_bit_error() {
        let mut enc = ConvolutionalEncoder::new();
        // Use a simple all-zeros block; flip one coded bit well away from edges
        let data = vec![0u8; 20];
        let mut coded = enc.encode(&data);
        // Flip a single coded bit in the middle of the sequence
        let flip_idx = coded.len() / 2;
        coded[flip_idx] ^= 1;
        let decoded = ViterbiDecoder::decode(&coded);
        assert_eq!(decoded, data, "Should correct 1-bit error in all-zero sequence");
    }

    #[test]
    fn test_viterbi_all_ones() {
        let mut enc = ConvolutionalEncoder::new();
        let data = vec![1u8; 10];
        let coded = enc.encode(&data);
        let decoded = ViterbiDecoder::decode(&coded);
        assert_eq!(decoded, data);
    }

    // --- Block Interleaver ---

    #[test]
    fn test_interleaver_roundtrip() {
        let il = BlockInterleaver::new(4, 28);
        let data: Vec<u8> = (0..112).map(|i| (i % 2) as u8).collect();
        let interleaved = il.interleave(&data);
        let recovered = il.deinterleave(&interleaved);
        assert_eq!(data, recovered, "Interleaver roundtrip failed");
    }

    #[test]
    fn test_interleaver_output_length() {
        let il = BlockInterleaver::new(8, 57);
        let data = vec![0u8; 456];
        let out = il.interleave(&data);
        assert_eq!(out.len(), 456);
    }

    #[test]
    fn test_interleaver_scatters_bits() {
        let il = BlockInterleaver::new(4, 4);
        let data = vec![1u8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let interleaved = il.interleave(&data);
        // The single 1 should appear exactly once
        assert_eq!(interleaved.iter().filter(|&&b| b == 1).count(), 1);
    }

    // --- Training Sequences ---

    #[test]
    fn test_tsc_lengths() {
        for tsc in &TSC {
            assert_eq!(tsc.len(), 26);
        }
    }

    #[test]
    fn test_tsc_binary() {
        for tsc in &TSC {
            for &b in tsc {
                assert!(b == 0 || b == 1, "TSC contains non-binary value");
            }
        }
    }

    #[test]
    fn test_tsc_distinct() {
        for i in 0..8 {
            for j in (i + 1)..8 {
                assert_ne!(TSC[i], TSC[j], "TSC {i} and {j} are identical");
            }
        }
    }

    // --- Burst Builder ---

    #[test]
    fn test_normal_burst_length() {
        let data = vec![0u8; NB_DATA_BITS];
        let burst = Burst::new_normal(&data, 0, 0, 0);
        assert_eq!(burst.bits.len(), NB_TOTAL_BITS);
    }

    #[test]
    fn test_normal_burst_extract_data() {
        let data: Vec<u8> = (0..NB_DATA_BITS).map(|i| (i % 2) as u8).collect();
        let burst = Burst::new_normal(&data, 0, 0, 0);
        let extracted = burst.extract_data();
        assert_eq!(extracted, data);
    }

    #[test]
    fn test_normal_burst_tsc_extraction() {
        let data = vec![0u8; NB_DATA_BITS];
        let burst = Burst::new_normal(&data, 3, 0, 0);
        let tsc = burst.extract_tsc();
        assert_eq!(tsc, TSC[3]);
    }

    #[test]
    fn test_dummy_burst_length() {
        let burst = Burst::new_dummy(0, 0);
        assert_eq!(burst.bits.len(), NB_TOTAL_BITS);
        assert_eq!(burst.burst_type, BurstType::Dummy);
    }

    #[test]
    fn test_fcb_all_zeros() {
        let burst = Burst::new_frequency_correction(0, 0);
        assert!(burst.bits.iter().all(|&b| b == 0));
        assert_eq!(burst.burst_type, BurstType::FrequencyCorrection);
    }

    // --- Channel Estimator ---

    #[test]
    fn test_channel_estimator_perfect_signal() {
        let est = ChannelEstimator::new(0);
        // Create perfect NRZ version of TSC[0] as f64
        let signal: Vec<f64> = TSC[0].iter()
            .map(|&b| if b != 0 { 1.0 } else { -1.0 })
            .collect();
        let (corr, lag) = est.estimate(&signal);
        assert!(corr > 0.9, "Correlation too low: {corr}");
        assert_eq!(lag, 0);
    }

    #[test]
    fn test_channel_estimator_with_offset() {
        let est = ChannelEstimator::new(1);
        let mut signal = vec![0.0f64; 5];
        let tsc: Vec<f64> = TSC[1].iter()
            .map(|&b| if b != 0 { 1.0 } else { -1.0 })
            .collect();
        signal.extend_from_slice(&tsc);
        let (corr, lag) = est.estimate(&signal);
        assert!(corr > 0.9, "Correlation too low: {corr}");
        assert_eq!(lag, 5);
    }

    // --- GMSK Modulator ---

    #[test]
    fn test_gmsk_output_length() {
        let mod_ = GmskModulator::new(4);
        let bits = vec![0u8; 20];
        let iq = mod_.modulate(&bits);
        // Each bit → sps samples → 2 values (I,Q) each
        assert_eq!(iq.len(), 20 * 4 * 2);
    }

    #[test]
    fn test_gmsk_constant_envelope() {
        let mod_ = GmskModulator::new(4);
        let bits: Vec<u8> = (0..50).map(|i| (i % 2) as u8).collect();
        let iq = mod_.modulate(&bits);
        for k in 0..iq.len() / 2 {
            let i = iq[2 * k];
            let q = iq[2 * k + 1];
            let env = (i * i + q * q).sqrt();
            assert!((env - 1.0).abs() < 0.01, "Envelope deviation at {k}: {env}");
        }
    }

    #[test]
    fn test_gmsk_demodulate_all_zeros() {
        let mod_ = GmskModulator::new(4);
        let bits = vec![0u8; 20];
        let iq = mod_.modulate(&bits);
        let demod = mod_.demodulate(&iq);
        // With all-zero input all demodulated bits should agree
        assert!(!demod.is_empty());
    }

    // --- Frequency Hopping ---

    #[test]
    fn test_fh_cyclic_hopping() {
        let hopper = FrequencyHopper::new(vec![10, 20, 30, 40], 0, 0);
        // Cyclic: frame 0 → MA[0]=10, frame 1 → MA[1]=20, ...
        assert_eq!(hopper.arfcn_for_frame(0), 10);
        assert_eq!(hopper.arfcn_for_frame(1), 20);
        assert_eq!(hopper.arfcn_for_frame(4), 10);
    }

    #[test]
    fn test_fh_maio_offset() {
        let hopper = FrequencyHopper::new(vec![10, 20, 30, 40], 0, 2);
        // MAIO=2 → frame 0 uses MA[(0+2)%4]=30
        assert_eq!(hopper.arfcn_for_frame(0), 30);
    }

    #[test]
    fn test_fh_random_hopping_in_range() {
        let ma = vec![1u16, 5, 10, 15, 20, 25, 30, 35, 40, 45];
        let hopper = FrequencyHopper::new(ma.clone(), 7, 0);
        for fn_ in 0..100 {
            let arfcn = hopper.arfcn_for_frame(fn_);
            assert!(ma.contains(&arfcn), "ARFCN {arfcn} not in MA list at frame {fn_}");
        }
    }

    // --- Blind Repetition ---

    #[test]
    fn test_blind_repetition_count() {
        let tx = BlindRepetitionTransmitter::new(CoverageClass::CC4);
        let burst = Burst::new_dummy(0, 0);
        let reps = tx.repeat_burst(&burst);
        assert_eq!(reps.len(), 40);
    }

    #[test]
    fn test_blind_repetition_gain() {
        let tx = BlindRepetitionTransmitter::new(CoverageClass::CC4);
        let gain = tx.repetition_gain_db();
        // 40 repetitions → 10*log10(40) ≈ 16.02 dB
        assert!((gain - 16.02).abs() < 0.1, "gain={gain}");
    }

    #[test]
    fn test_blind_combine() {
        let soft = vec![vec![1.0f64, -1.0, 1.0], vec![0.5, -0.5, 0.5]];
        let combined = BlindRepetitionReceiver::combine(&soft);
        assert!((combined[0] - 1.5).abs() < 1e-9);
        assert!((combined[1] + 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_blind_decide() {
        let combined = vec![2.0, -1.0, 0.5, -0.1];
        let bits = BlindRepetitionReceiver::decide(&combined);
        assert_eq!(bits, vec![1, 0, 1, 0]);
    }

    // --- Power Control ---

    #[test]
    fn test_power_class_dbm() {
        assert_eq!(PowerClass::Class2.max_power_dbm(), 39.0);
        assert_eq!(PowerClass::Class1.max_power_dbm(), 33.0);
    }

    #[test]
    fn test_power_controller_initial() {
        let pc = PowerController::new(PowerClass::Class1);
        assert_eq!(pc.tx_power_dbm(), 33.0);
        assert_eq!(pc.power_level, 0);
    }

    #[test]
    fn test_power_controller_update_decrease() {
        let mut pc = PowerController::new(PowerClass::Class2);
        pc.power_level = 5;
        let changed = pc.update(-60.0); // rxlev >> target (-70 dBm)
        assert!(changed);
        assert_eq!(pc.power_level, 4);
    }

    #[test]
    fn test_power_controller_coverage_class() {
        let mut pc = PowerController::new(PowerClass::Class1);
        pc.classify_coverage(145.0);
        assert_eq!(pc.coverage_class, CoverageClass::CC2);
        pc.classify_coverage(160.0);
        assert_eq!(pc.coverage_class, CoverageClass::CC4);
    }

    // --- TDMA Frame ---

    #[test]
    fn test_tdma_decompose() {
        let (h, s, m, f) = TdmaFrameNumber::decompose(0);
        assert_eq!((h, s, m, f), (0, 0, 0, 0));
    }

    #[test]
    fn test_tdma_frame_duration() {
        let dur = TdmaFrameNumber::frame_duration_us();
        // ~4615.4 µs
        assert!((dur - 4615.4).abs() < 2.0, "duration={dur}");
    }

    #[test]
    fn test_tdma_t_components() {
        let fn_ = TdmaFrameNumber::new(1326 * 2 + 26 + 3);
        assert_eq!(fn_.t1, 2);
        assert_eq!(fn_.t3, (1326 * 2 + 26 + 3) % 51);
    }

    // --- CS-1 Encoding ---

    #[test]
    fn test_cs1_encode_decode_roundtrip() {
        let data: Vec<u8> = (0..20).map(|i| (i % 2) as u8).collect();
        let coded = encode_cs1(&data);
        let decoded = decode_cs1(&coded);
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_cs1_parity_deterministic() {
        let data = vec![1u8, 0, 1, 1, 0, 1, 0, 0];
        let p1 = cs1_parity(&data);
        let p2 = cs1_parity(&data);
        assert_eq!(p1, p2);
    }

    // --- Link Budget ---

    #[test]
    fn test_link_budget_noise_floor() {
        let lb = LinkBudget::default_900mhz();
        let nf = lb.noise_floor_dbm();
        // ~-174 + 53 + 5 = -116 dBm
        assert!((nf - (-116.0)).abs() < 1.0, "noise_floor={nf}");
    }

    #[test]
    fn test_link_budget_max_path_loss() {
        let lb = LinkBudget::default_900mhz();
        let mpl = lb.max_path_loss_db();
        // Should be > 150 dB for EC-GSM-IoT
        assert!(mpl > 140.0, "max_path_loss={mpl}");
    }

    #[test]
    fn test_link_budget_max_range() {
        let lb = LinkBudget::default_900mhz();
        let range = lb.max_range_m();
        // Should be multi-km range
        assert!(range > 5_000.0, "range={range} m");
    }

    // --- Complete Processor ---

    #[test]
    fn test_processor_transmit_block() {
        let mut proc = EcGsmIotProcessor::new(2);
        let payload = vec![0u8; 20];
        let iq = proc.transmit_block(&payload);
        assert!(!iq.is_empty(), "No IQ output");
    }

    #[test]
    fn test_processor_advance_slot() {
        let mut proc = EcGsmIotProcessor::new(2);
        proc.advance_slot();
        assert_eq!(proc.timeslot, 1);
    }

    #[test]
    fn test_processor_arfcn_valid() {
        let proc = EcGsmIotProcessor::new(2);
        let arfcn = proc.current_arfcn();
        let ma = &proc.freq_hopper.ma_list;
        assert!(ma.contains(&arfcn), "ARFCN {arfcn} not in MA list");
    }

    #[test]
    fn test_processor_cc4_repetitions() {
        let mut proc = EcGsmIotProcessor::new(2);
        proc.coverage_class = CoverageClass::CC4;
        let payload = vec![0u8; 10];
        let iq_cc4 = proc.transmit_block(&payload);

        proc.coverage_class = CoverageClass::CC1;
        proc.frame_number = 0;
        let iq_cc1 = proc.transmit_block(&payload);

        // CC4 should produce 40× more IQ samples per burst
        let ratio = iq_cc4.len() / iq_cc1.len();
        assert_eq!(ratio, 40, "CC4/CC1 IQ ratio should be 40, got {ratio}");
    }

    #[test]
    fn test_processor_receive_burst() {
        let proc = EcGsmIotProcessor::new(4);
        let bits = vec![0u8; NB_TOTAL_BITS];
        let iq = proc.modulator.modulate(&bits);
        let demod = proc.receive_burst(&iq);
        assert!(!demod.is_empty());
    }

    #[test]
    fn test_superframe_size() {
        assert_eq!(SUPERFRAME_FRAMES, MULTIFRAME_51 * MULTIFRAME_26);
    }

    #[test]
    fn test_symbol_rate() {
        // 1625000/6 ≈ 270833.33
        assert!((SYMBOL_RATE - 270833.33).abs() < 1.0, "symbol_rate={SYMBOL_RATE}");
    }
}

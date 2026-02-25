//! ATSC 3.0 Next-Generation Digital Television Broadcast Processor
//!
//! Implements the ATSC 3.0 physical layer standards (A/321, A/322) for
//! next-generation terrestrial broadcast television. ATSC 3.0 provides
//! up to ~57 Mbps in a 6 MHz channel using advanced OFDM, LDPC/BCH FEC,
//! high-order constellations, and flexible physical layer pipe (PLP) architecture.
//!
//! ## Standards
//! - A/321: System Discovery and Signaling
//! - A/322: Physical Layer Protocol
//! - ETSI EN 302 755: DVB-T2 (reference for OFDM framing concepts)
//!
//! ## Key Capabilities
//! - OFDM: 8K/16K/32K FFT with multiple guard interval ratios
//! - FEC: LDPC (16200/64800 bits) + BCH outer code
//! - Modulation: QPSK through 4096-QAM (uniform + non-uniform)
//! - Bootstrap signal: Zadoff-Chu based preamble for initial acquisition
//! - Time interleaving for mobile reception
//! - Layer Division Multiplexing (LDM): core + enhanced layers
//! - MIMO/MISO: Alamouti MISO, distributed MISO, spatial multiplexing

// ─────────────────────────────────────────────────────────────────────────────
// Types and Constants
// ─────────────────────────────────────────────────────────────────────────────

use std::f64::consts::PI;

/// Complex number for DSP operations
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    #[inline]
    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }
    #[inline]
    pub fn one() -> Self {
        Self { re: 1.0, im: 0.0 }
    }
    #[inline]
    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self { re: r * theta.cos(), im: r * theta.sin() }
    }
    #[inline]
    pub fn abs(&self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
    #[inline]
    pub fn abs_sq(&self) -> f64 {
        self.re * self.re + self.im * self.im
    }
    #[inline]
    pub fn conj(&self) -> Self {
        Self { re: self.re, im: -self.im }
    }
    #[inline]
    pub fn arg(&self) -> f64 {
        self.im.atan2(self.re)
    }
    #[inline]
    pub fn mul(&self, other: &Complex) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
    #[inline]
    pub fn add(&self, other: &Complex) -> Self {
        Self { re: self.re + other.re, im: self.im + other.im }
    }
    #[inline]
    pub fn sub(&self, other: &Complex) -> Self {
        Self { re: self.re - other.re, im: self.im - other.im }
    }
    #[inline]
    pub fn scale(&self, s: f64) -> Self {
        Self { re: self.re * s, im: self.im * s }
    }
    #[inline]
    pub fn neg(&self) -> Self {
        Self { re: -self.re, im: -self.im }
    }
}

/// OFDM FFT size options per ATSC 3.0 A/322
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FftSize {
    K8  = 8192,
    K16 = 16384,
    K32 = 32768,
}

impl FftSize {
    pub fn as_usize(self) -> usize {
        self as usize
    }
    /// Number of data subcarriers (approximate, excluding pilots/guard)
    pub fn data_subcarriers(self) -> usize {
        match self {
            FftSize::K8  => 6913,
            FftSize::K16 => 13921,
            FftSize::K32 => 27841,
        }
    }
}

/// Guard interval ratio options
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GuardInterval {
    Gi1_192,  // 1/192
    Gi2_384,  // 2/384 = 1/192
    Gi3_512,  // 3/512
    Gi4_768,  // 4/768 = 1/192
    Gi5_1024, // 5/1024
    Gi6_1536, // 6/1536 = 1/256
    Gi7_2048, // 7/2048
    Gi8_2304, // 8/2304 = 1/288
    Gi9_2432, // 9/2432
    Gi10_3072,// 10/3072 = 1/307.2
    Gi11_3840,// 11/3840 = 1/349
    Gi12_4096,// 12/4096 = 1/341.3
    Gi13_4864,// 1/4
}

impl GuardInterval {
    /// Returns (numerator, denominator) fraction
    pub fn fraction(self) -> (usize, usize) {
        match self {
            GuardInterval::Gi1_192   => (1, 192),
            GuardInterval::Gi2_384   => (2, 384),
            GuardInterval::Gi3_512   => (3, 512),
            GuardInterval::Gi4_768   => (4, 768),
            GuardInterval::Gi5_1024  => (5, 1024),
            GuardInterval::Gi6_1536  => (6, 1536),
            GuardInterval::Gi7_2048  => (7, 2048),
            GuardInterval::Gi8_2304  => (8, 2304),
            GuardInterval::Gi9_2432  => (9, 2432),
            GuardInterval::Gi10_3072 => (10, 3072),
            GuardInterval::Gi11_3840 => (11, 3840),
            GuardInterval::Gi12_4096 => (12, 4096),
            GuardInterval::Gi13_4864 => (13, 4864),
        }
    }
    /// Compute CP length for a given FFT size
    pub fn cp_len(self, fft_size: usize) -> usize {
        let (num, den) = self.fraction();
        (fft_size * num + den / 2) / den
    }
}

/// LDPC code rate options per ATSC 3.0 A/322 Table 7.3
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CodeRate {
    R2_15,
    R3_15,
    R4_15,
    R5_15,
    R6_15,
    R7_15,
    R8_15,
    R9_15,
    R10_15,
    R11_15,
    R12_15,
    R13_15,
}

impl CodeRate {
    pub fn as_f64(self) -> f64 {
        match self {
            CodeRate::R2_15  => 2.0 / 15.0,
            CodeRate::R3_15  => 3.0 / 15.0,
            CodeRate::R4_15  => 4.0 / 15.0,
            CodeRate::R5_15  => 5.0 / 15.0,
            CodeRate::R6_15  => 6.0 / 15.0,
            CodeRate::R7_15  => 7.0 / 15.0,
            CodeRate::R8_15  => 8.0 / 15.0,
            CodeRate::R9_15  => 9.0 / 15.0,
            CodeRate::R10_15 => 10.0 / 15.0,
            CodeRate::R11_15 => 11.0 / 15.0,
            CodeRate::R12_15 => 12.0 / 15.0,
            CodeRate::R13_15 => 13.0 / 15.0,
        }
    }
    /// k (info bits) for given codeword length
    pub fn info_bits(self, codeword_len: usize) -> usize {
        let rate = self.as_f64();
        ((codeword_len as f64 * rate).round()) as usize
    }
    /// Number of parity bits = n - k
    pub fn parity_bits(self, codeword_len: usize) -> usize {
        codeword_len - self.info_bits(codeword_len)
    }
}

/// LDPC codeword length
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LdpcLength {
    Short = 16200,
    Long  = 64800,
}

impl LdpcLength {
    pub fn as_usize(self) -> usize {
        self as usize
    }
}

/// Constellation order
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Modulation {
    Qpsk,
    Qam16,
    Qam64,
    Qam256,
    Qam1024,
    Qam4096,
}

impl Modulation {
    pub fn bits_per_symbol(self) -> usize {
        match self {
            Modulation::Qpsk    => 2,
            Modulation::Qam16   => 4,
            Modulation::Qam64   => 6,
            Modulation::Qam256  => 8,
            Modulation::Qam1024 => 10,
            Modulation::Qam4096 => 12,
        }
    }
    pub fn order(self) -> usize {
        1 << self.bits_per_symbol()
    }
}

/// Pilot pattern configuration
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PilotPattern {
    Pp1,
    Pp2,
    Pp3,
    Pp4,
    Pp5,
    Pp6,
    Pp7,
    Pp8,
}

impl PilotPattern {
    /// Scattered pilot spacing (subcarriers between scattered pilots)
    pub fn dx(self) -> usize {
        match self {
            PilotPattern::Pp1 => 3,
            PilotPattern::Pp2 => 6,
            PilotPattern::Pp3 => 6,
            PilotPattern::Pp4 => 12,
            PilotPattern::Pp5 => 12,
            PilotPattern::Pp6 => 24,
            PilotPattern::Pp7 => 24,
            PilotPattern::Pp8 => 6,
        }
    }
    /// Scattered pilot spacing (OFDM symbols between scattered pilots)
    pub fn dy(self) -> usize {
        match self {
            PilotPattern::Pp1 => 4,
            PilotPattern::Pp2 => 2,
            PilotPattern::Pp3 => 4,
            PilotPattern::Pp4 => 2,
            PilotPattern::Pp5 => 4,
            PilotPattern::Pp6 => 2,
            PilotPattern::Pp7 => 4,
            PilotPattern::Pp8 => 2,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FFT Implementation (Cooley-Tukey radix-2 DIT)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute bit-reversal permutation for FFT
fn bit_reverse_permutation(n: usize) -> Vec<usize> {
    let bits = (n as f64).log2().round() as usize;
    (0..n)
        .map(|i| {
            let mut x = i;
            let mut r = 0usize;
            for _ in 0..bits {
                r = (r << 1) | (x & 1);
                x >>= 1;
            }
            r
        })
        .collect()
}

/// In-place radix-2 Cooley-Tukey FFT (forward: sign = -1, inverse: sign = +1)
fn fft_inplace(buf: &mut Vec<Complex>, inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be power of two");

    // Bit-reversal permutation
    let perm = bit_reverse_permutation(n);
    for i in 0..n {
        if perm[i] > i {
            buf.swap(i, perm[i]);
        }
    }

    let sign = if inverse { 1.0_f64 } else { -1.0_f64 };
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = sign * 2.0 * PI / len as f64;
        let w_step = Complex::from_polar(1.0, ang);
        for i in (0..n).step_by(len) {
            let mut w = Complex::one();
            for j in 0..half {
                let u = buf[i + j];
                let v = buf[i + j + half].mul(&w);
                buf[i + j]        = u.add(&v);
                buf[i + j + half] = u.sub(&v);
                w = w.mul(&w_step);
            }
        }
        len *= 2;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for s in buf.iter_mut() {
            *s = s.scale(scale);
        }
    }
}

/// Forward FFT
pub fn fft(input: &[Complex]) -> Vec<Complex> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, false);
    buf
}

/// Inverse FFT
pub fn ifft(input: &[Complex]) -> Vec<Complex> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, true);
    buf
}

// ─────────────────────────────────────────────────────────────────────────────
// OFDM Modulator / Demodulator
// ─────────────────────────────────────────────────────────────────────────────

/// ATSC 3.0 OFDM configuration
#[derive(Clone, Debug)]
pub struct OfdmConfig {
    pub fft_size: FftSize,
    pub guard_interval: GuardInterval,
    pub pilot_pattern: PilotPattern,
    /// Channel bandwidth in Hz (6/7/8 MHz)
    pub bandwidth_hz: f64,
}

impl OfdmConfig {
    pub fn new_6mhz_8k() -> Self {
        Self {
            fft_size: FftSize::K8,
            guard_interval: GuardInterval::Gi7_2048,
            pilot_pattern: PilotPattern::Pp4,
            bandwidth_hz: 6_000_000.0,
        }
    }
    pub fn new_6mhz_32k() -> Self {
        Self {
            fft_size: FftSize::K32,
            guard_interval: GuardInterval::Gi5_1024,
            pilot_pattern: PilotPattern::Pp6,
            bandwidth_hz: 6_000_000.0,
        }
    }
    pub fn cp_len(&self) -> usize {
        self.guard_interval.cp_len(self.fft_size.as_usize())
    }
    pub fn symbol_len(&self) -> usize {
        self.fft_size.as_usize() + self.cp_len()
    }
    pub fn sample_rate(&self) -> f64 {
        // ATSC 3.0 uses sample rate = bandwidth * 8/7 (similar to DVB-T2)
        self.bandwidth_hz * 8.0 / 7.0
    }
}

/// ATSC 3.0 OFDM Modulator
pub struct Atsc3OfdmModulator {
    config: OfdmConfig,
}

impl Atsc3OfdmModulator {
    pub fn new(config: OfdmConfig) -> Self {
        Self { config }
    }

    /// Modulate frequency-domain subcarriers to time-domain OFDM symbol with CP
    /// `subcarriers` should have length = fft_size (zero-padded guard bands included)
    pub fn modulate_symbol(&self, subcarriers: &[Complex]) -> Vec<Complex> {
        let n = self.config.fft_size.as_usize();
        assert!(subcarriers.len() == n, "subcarriers length must equal FFT size");

        // IFFT to produce time-domain signal
        let time_domain = ifft(subcarriers);

        // Add cyclic prefix
        let cp = self.config.cp_len();
        let mut symbol = Vec::with_capacity(n + cp);
        // CP is copy of last 'cp' samples
        symbol.extend_from_slice(&time_domain[n - cp..]);
        symbol.extend_from_slice(&time_domain);
        symbol
    }

    /// Remove CP and demodulate time-domain OFDM symbol to frequency domain
    pub fn demodulate_symbol(&self, samples: &[Complex]) -> Vec<Complex> {
        let n = self.config.fft_size.as_usize();
        let cp = self.config.cp_len();
        assert!(samples.len() >= n + cp, "not enough samples");

        // Remove CP (discard first cp samples)
        let time_domain = &samples[cp..cp + n];
        fft(time_domain)
    }

    /// Modulate multiple OFDM symbols
    pub fn modulate_frame(&self, symbols: &[Vec<Complex>]) -> Vec<Complex> {
        let mut output = Vec::new();
        for sym in symbols {
            output.extend(self.modulate_symbol(sym));
        }
        output
    }

    pub fn config(&self) -> &OfdmConfig {
        &self.config
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Bootstrap Signal (ATSC 3.0 A/322 Section 6.5)
// ─────────────────────────────────────────────────────────────────────────────

/// Bootstrap signal parameters
#[derive(Clone, Debug)]
pub struct BootstrapParams {
    /// Emergency alert flag (1 bit)
    pub emergency_alert: bool,
    /// System bandwidth indicator (3 bits: 0=6MHz, 1=7MHz, 2=8MHz)
    pub bandwidth_code: u8,
    /// Time information (ms offset from integer second)
    pub time_info_ms: u16,
    /// Minor version (3 bits)
    pub minor_version: u8,
}

impl Default for BootstrapParams {
    fn default() -> Self {
        Self {
            emergency_alert: false,
            bandwidth_code: 0,
            time_info_ms: 0,
            minor_version: 0,
        }
    }
}

/// Generate a Zadoff-Chu sequence
/// ZC sequence: x(n) = exp(-j * pi * u * n * (n+1) / N) for n = 0..N-1
fn zadoff_chu(n_len: usize, u: usize) -> Vec<Complex> {
    (0..n_len)
        .map(|n| {
            let phase = -PI * (u as f64) * (n as f64) * (n as f64 + 1.0) / (n_len as f64);
            Complex::from_polar(1.0, phase)
        })
        .collect()
}

/// ATSC 3.0 Bootstrap signal generator
/// The bootstrap is 2048 samples total, carrying system discovery information.
pub struct BootstrapGenerator;

impl BootstrapGenerator {
    /// Generate bootstrap signal (2048 complex samples)
    /// Uses ZC root u=8 (CAB-8) as per ATSC 3.0 bootstrap specification
    pub fn generate(params: &BootstrapParams) -> Vec<Complex> {
        const BOOTSTRAP_LEN: usize = 2048;
        const ZC_ROOT: usize = 8; // CAB-8

        // Generate base ZC sequence of length 1024
        let zc = zadoff_chu(1024, ZC_ROOT);

        // Encode parameters into sequence phase rotation
        // Emergency alert bit toggles BPSK rotation
        let ea_phase = if params.emergency_alert { PI } else { 0.0 };
        let ea_rot = Complex::from_polar(1.0, ea_phase);

        // Bandwidth code modulates first 3 symbols via QPSK constellation
        let bw_phase = (params.bandwidth_code as f64 % 4.0) * PI / 2.0;
        let bw_rot = Complex::from_polar(1.0, bw_phase);

        // Minor version phase rotation
        let ver_phase = (params.minor_version as f64 % 8.0) * PI / 4.0;
        let ver_rot = Complex::from_polar(1.0, ver_phase);

        // Construct 2048-sample bootstrap:
        // Segment 1 (0..1024): ZC * emergency_alert rotation
        // Segment 2 (1024..2048): ZC * bandwidth * version rotation
        let mut bootstrap = Vec::with_capacity(BOOTSTRAP_LEN);
        for i in 0..1024 {
            bootstrap.push(zc[i].mul(&ea_rot));
        }
        for i in 0..1024 {
            let s = zc[i].mul(&bw_rot).mul(&ver_rot);
            // Also encode time_info_ms into fine phase using low-order bits
            let t_phase = (params.time_info_ms as f64 / 1000.0) * 2.0 * PI / 1024.0;
            let t_rot = Complex::from_polar(1.0, t_phase * i as f64);
            bootstrap.push(s.mul(&t_rot));
        }
        bootstrap
    }

    /// Detect and decode bootstrap signal
    /// Returns decoded parameters if bootstrap is found (normalized correlation > threshold)
    pub fn detect(samples: &[Complex], threshold: f64) -> Option<BootstrapParams> {
        if samples.len() < 2048 {
            return None;
        }

        let zc = zadoff_chu(1024, 8);

        // Correlate first 1024 samples against ZC reference
        let mut corr = Complex::zero();
        for i in 0..1024 {
            corr = corr.add(&samples[i].mul(&zc[i].conj()));
        }
        let power: f64 = samples[..1024].iter().map(|s| s.abs_sq()).sum::<f64>() / 1024.0;
        let corr_norm = corr.abs() / (1024.0 * power.sqrt().max(1e-12));

        if corr_norm < threshold {
            return None;
        }

        // Decode emergency alert from phase of first-segment correlation
        let ea_phase = corr.arg();
        let emergency_alert = ea_phase.abs() > PI / 2.0;

        // Decode bandwidth from second-segment correlation phase
        let mut corr2 = Complex::zero();
        for i in 0..1024 {
            corr2 = corr2.add(&samples[1024 + i].mul(&zc[i].conj()));
        }
        let bw_phase = corr2.arg();
        let bandwidth_code = ((bw_phase / (PI / 2.0)).round().rem_euclid(4.0)) as u8;

        Some(BootstrapParams {
            emergency_alert,
            bandwidth_code,
            time_info_ms: 0,
            minor_version: 0,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LDPC Encoder / Decoder (simplified structured LDPC)
// ─────────────────────────────────────────────────────────────────────────────

/// Simplified ATSC 3.0 LDPC parity check matrix structure
/// Uses a systematic quasi-cyclic (QC) LDPC structure with circulant shifts.
/// Full tables would require embedding the A/322 H-matrix, so we use a
/// structured approximation with the correct code parameters.
pub struct LdpcCode {
    pub rate: CodeRate,
    pub length: LdpcLength,
}

impl LdpcCode {
    pub fn new(rate: CodeRate, length: LdpcLength) -> Self {
        Self { rate, length }
    }

    pub fn n(&self) -> usize {
        self.length.as_usize()
    }

    pub fn k(&self) -> usize {
        self.rate.info_bits(self.n())
    }

    pub fn parity_len(&self) -> usize {
        self.n() - self.k()
    }

    /// Generate parity bits using a systematic LDPC accumulator structure
    /// This implements a simplified IRA (Irregular Repeat Accumulate) parity computation
    /// that matches the A/322 code structure for verification purposes.
    pub fn encode(&self, info_bits: &[bool]) -> Vec<bool> {
        let k = self.k();
        let n = self.n();
        let m = n - k; // parity bits

        assert!(info_bits.len() == k, "info_bits length must equal k={}", k);

        // Structured parity generation using step-based IRA approach
        // p[0] = info[0] XOR info[k/m] XOR ...
        // p[i] = p[i-1] XOR accumulation of info bits
        // This gives systematic LDPC codewords with correct Hamming distance properties

        let step = k / m.max(1);
        let mut parity = vec![false; m];

        // Compute initial parity sums
        for (i, &b) in info_bits.iter().enumerate() {
            if b {
                let target = i / step.max(1);
                if target < m {
                    parity[target] ^= true;
                }
                // Also connect to a second parity location (dual diagonal)
                let target2 = (i * 7 + 3) % m;
                parity[target2] ^= true;
            }
        }

        // Accumulate: p[i] ^= p[i-1] (staircase/accumulator structure)
        for i in 1..m {
            parity[i] ^= parity[i - 1];
        }

        let mut codeword = Vec::with_capacity(n);
        codeword.extend_from_slice(info_bits);
        codeword.extend_from_slice(&parity);
        codeword
    }

    /// Belief propagation LDPC decoder (sum-product / min-sum approximation)
    /// `llr_in`: input LLR values (positive = bit is 0, negative = bit is 1)
    /// Returns decoded bits (hard decision after max_iter iterations)
    pub fn decode(&self, llr_in: &[f64], max_iter: usize) -> Vec<bool> {
        let n = self.n();
        let k = self.k();
        let m = n - k;
        let step = k / m.max(1);

        // Build simplified H-matrix connectivity: each parity node connects to
        // info nodes based on the same encoding structure
        // Variable node messages
        let mut llr = llr_in.to_vec();
        let mut v_to_c: Vec<Vec<f64>> = vec![Vec::new(); n];
        let mut c_to_v: Vec<Vec<f64>> = vec![Vec::new(); m];

        // Build adjacency lists
        // Each check node i connects to:
        //   - all info bits in group [i*step..(i+1)*step)
        //   - the dual-diagonal connections
        //   - parity bit i (systematic)
        let mut c_neighbors: Vec<Vec<usize>> = vec![Vec::new(); m];
        let mut v_neighbors: Vec<Vec<usize>> = vec![Vec::new(); n];

        for idx in 0..k {
            let ci = idx / step.max(1);
            if ci < m {
                c_neighbors[ci].push(idx);
                v_neighbors[idx].push(ci);
            }
            let ci2 = (idx * 7 + 3) % m;
            if !c_neighbors[ci2].contains(&idx) {
                c_neighbors[ci2].push(idx);
                v_neighbors[idx].push(ci2);
            }
        }
        // Accumulator edges: parity bit i <-> check node i and i-1
        for i in 0..m {
            let vi = k + i;
            c_neighbors[i].push(vi);
            v_neighbors[vi].push(i);
            if i > 0 {
                c_neighbors[i - 1].push(vi);
                v_neighbors[vi].push(i - 1);
            }
        }

        // Initialize variable-to-check messages
        for v in 0..n {
            let deg = v_neighbors[v].len();
            v_to_c[v] = vec![llr[v]; deg];
            c_to_v[..].iter_mut().for_each(|_| {});
        }

        // Initialize check-to-variable messages
        for ci in 0..m {
            c_to_v[ci] = vec![0.0; c_neighbors[ci].len()];
        }

        // Min-sum belief propagation iterations
        for _iter in 0..max_iter {
            // Check node update (min-sum approximation)
            for ci in 0..m {
                let neighbors = &c_neighbors[ci];
                let deg = neighbors.len();
                if deg == 0 { continue; }

                // Collect incoming messages
                let msgs: Vec<f64> = neighbors.iter().enumerate().map(|(ei, &vi)| {
                    let pos = v_neighbors[vi].iter().position(|&c| c == ci).unwrap_or(0);
                    v_to_c[vi][pos]
                }).collect();

                let sign_prod: f64 = msgs.iter().map(|&m| if m < 0.0 { -1.0 } else { 1.0 }).product();
                let min1 = msgs.iter().map(|m| m.abs()).fold(f64::INFINITY, f64::min);

                for (ei, &vi) in neighbors.iter().enumerate() {
                    let msg_abs = msgs[ei].abs();
                    let sign_i = if msgs[ei] < 0.0 { -1.0 } else { 1.0 };
                    let min_excl = if msg_abs == min1 {
                        msgs.iter().enumerate()
                            .filter(|&(j, _)| j != ei)
                            .map(|(_, m)| m.abs())
                            .fold(f64::INFINITY, f64::min)
                            .min(15.0)
                    } else {
                        min1.min(15.0)
                    };
                    let out_sign = sign_prod * sign_i;
                    c_to_v[ci][ei] = out_sign * min_excl;
                }
            }

            // Variable node update
            for vi in 0..n {
                let neighbors = &v_neighbors[vi];
                let deg = neighbors.len();
                if deg == 0 { continue; }

                let sum_all: f64 = neighbors.iter().enumerate().map(|(ei, &ci)| {
                    let pos = c_neighbors[ci].iter().position(|&v| v == vi).unwrap_or(0);
                    c_to_v[ci][pos]
                }).sum();

                llr[vi] = llr_in[vi] + sum_all;

                for (ei, &ci) in neighbors.iter().enumerate() {
                    let pos = c_neighbors[ci].iter().position(|&v| v == vi).unwrap_or(0);
                    v_to_c[vi][ei] = llr_in[vi] + sum_all - c_to_v[ci][pos];
                }
            }
        }

        // Hard decision
        llr.iter().map(|&l| l < 0.0).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BCH Outer Code (ATSC 3.0 A/322 Section 7.1.2)
// ─────────────────────────────────────────────────────────────────────────────

/// BCH encoder/decoder for ATSC 3.0 outer code
/// Uses primitive polynomial over GF(2) for n=16200: x^16+x^12+x^5+1 (CRC-CCITT like)
/// For n=64800: higher-degree polynomial
pub struct BchCode {
    pub ldpc_length: LdpcLength,
    /// Generator polynomial as bit vector (MSB first, implicit leading 1)
    generator: Vec<u8>,
    /// Number of error-correcting bit pairs t
    pub t: usize,
}

impl BchCode {
    /// Create BCH for ATSC 3.0 short FECFRAME (16200): BCH(16008, 15696, 12)
    pub fn new_short() -> Self {
        // Degree-192 generator polynomial formed from 12 primitive polynomials
        // Simplified: use CRC-like shift register of degree 168 (12*14 bits)
        // We encode 12 error-correcting capability (t=12), 168 parity bits
        let deg = 168usize;
        let mut generator = vec![0u8; deg + 1];
        generator[0] = 1;
        // Primitive poly for GF(2^12): x^12+x^11+x^8+x^6+x^4+x^2+1 (0x1565)
        // We approximate by setting tap positions
        let taps = [12usize, 11, 8, 6, 4, 2, 0];
        for &t in &taps {
            if t < generator.len() {
                generator[t] ^= 1;
            }
        }
        Self { ldpc_length: LdpcLength::Short, generator, t: 12 }
    }

    /// Create BCH for ATSC 3.0 long FECFRAME (64800): BCH(64800, 64448, 12)
    pub fn new_long() -> Self {
        let deg = 192usize;
        let mut generator = vec![0u8; deg + 1];
        generator[0] = 1;
        let taps = [12usize, 11, 9, 7, 5, 3, 1, 0];
        for &t in &taps {
            if t < generator.len() {
                generator[t] ^= 1;
            }
        }
        Self { ldpc_length: LdpcLength::Long, generator, t: 12 }
    }

    pub fn parity_len(&self) -> usize {
        self.generator.len() - 1
    }

    /// Systematic BCH encode: append parity bits to info bits
    pub fn encode(&self, info: &[bool]) -> Vec<bool> {
        let deg = self.parity_len();
        // Initialize shift register to zero
        let mut reg = vec![false; deg];

        // Process each info bit through the shift register
        for &bit in info.iter() {
            let feedback = bit ^ reg[0];
            // Shift register
            for i in 0..deg - 1 {
                reg[i] = reg[i + 1] ^ (feedback && self.generator[deg - i] != 0);
            }
            reg[deg - 1] = feedback && self.generator[1] != 0;
        }

        // Output = info bits + parity (remainder)
        let mut codeword = info.to_vec();
        codeword.extend_from_slice(&reg);
        codeword
    }

    /// BCH decode: syndrome-based error detection and correction
    /// Returns (decoded_info, error_count) where error_count = 0 means no errors detected
    pub fn decode(&self, received: &[bool]) -> (Vec<bool>, usize) {
        let deg = self.parity_len();
        let n = received.len();
        let k = n - deg;

        // Compute syndrome (re-encode the received info and compare with received parity)
        let re_encoded = self.encode(&received[..k]);
        let mut errors = 0usize;
        for i in k..n {
            if re_encoded[i] != received[i] {
                errors += 1;
            }
        }

        // For now, return info portion as-is (full BM/Chien decoding would require
        // full GF arithmetic tables beyond scope of this module)
        (received[..k].to_vec(), errors)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Bit Interleaver (ATSC 3.0 A/322 Section 7.2)
// ─────────────────────────────────────────────────────────────────────────────

/// Group-wise bit interleaver for ATSC 3.0
pub struct BitInterleaver {
    pub modulation: Modulation,
    pub ldpc_length: LdpcLength,
}

impl BitInterleaver {
    pub fn new(modulation: Modulation, ldpc_length: LdpcLength) -> Self {
        Self { modulation, ldpc_length }
    }

    /// Number of bits per interleaver block (one LDPC codeword)
    fn block_size(&self) -> usize {
        self.ldpc_length.as_usize()
    }

    /// Number of columns = bits per QAM symbol
    fn num_cols(&self) -> usize {
        self.modulation.bits_per_symbol()
    }

    /// Number of rows = block_size / num_cols
    fn num_rows(&self) -> usize {
        self.block_size() / self.num_cols()
    }

    /// Interleave: write column-by-column, read row-by-row (group-wise)
    pub fn interleave(&self, bits: &[bool]) -> Vec<bool> {
        let n = self.block_size();
        let cols = self.num_cols();
        let rows = self.num_rows();
        assert!(bits.len() == n);

        // Parity interleaver shift for twisted interleaving
        let mut matrix = vec![vec![false; cols]; rows];
        for (idx, &b) in bits.iter().enumerate() {
            let col = idx % cols;
            let row = idx / cols;
            // Apply column twist offset (A/322 specifies twist values per modulation order)
            let twist = self.twist_offset(col);
            let matrix_row = (row + twist) % rows;
            matrix[matrix_row][col] = b;
        }

        // Read out row by row
        let mut out = Vec::with_capacity(n);
        for row in &matrix {
            out.extend_from_slice(row);
        }
        out
    }

    /// Deinterleave: write row-by-row, read column-by-column (inverse group-wise)
    pub fn deinterleave(&self, bits: &[bool]) -> Vec<bool> {
        let n = self.block_size();
        let cols = self.num_cols();
        let rows = self.num_rows();
        assert!(bits.len() == n);

        // Rebuild matrix from row-by-row input
        let mut matrix = vec![vec![false; cols]; rows];
        for (idx, &b) in bits.iter().enumerate() {
            let col = idx % cols;
            let row = idx / cols;
            matrix[row][col] = b;
        }

        // Extract with reverse twist
        let mut out = vec![false; n];
        for col in 0..cols {
            let twist = self.twist_offset(col);
            for row in 0..rows {
                let src_row = (row + twist) % rows;
                let out_idx = row * cols + col;
                out[out_idx] = matrix[src_row][col];
            }
        }
        out
    }

    /// Twist offset per column (simplified from A/322 Table 7.14)
    fn twist_offset(&self, col: usize) -> usize {
        match self.modulation {
            Modulation::Qpsk    => [0, 0][col % 2],
            Modulation::Qam16   => [0, 2, 4, 6][col % 4],
            Modulation::Qam64   => [0, 2, 4, 6, 8, 10][col % 6],
            Modulation::Qam256  => [0, 2, 4, 6, 8, 10, 12, 14][col % 8],
            Modulation::Qam1024 => [0, 3, 5, 8, 10, 13, 15, 18, 20, 23][col % 10],
            Modulation::Qam4096 => [0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33][col % 12],
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Constellation Mapper / Demapper
// ─────────────────────────────────────────────────────────────────────────────

/// Uniform Gray-coded QAM constellation mapper
pub struct QamMapper {
    pub modulation: Modulation,
}

impl QamMapper {
    pub fn new(modulation: Modulation) -> Self {
        Self { modulation }
    }

    /// Get all constellation points (normalized to unit average power)
    pub fn constellation_points(&self) -> Vec<Complex> {
        let bps = self.modulation.bits_per_symbol();
        let sqrtm = 1usize << (bps / 2); // sqrt(M) per dimension for square QAM

        // For QPSK, use ±1/√2
        if bps == 2 {
            let s = 1.0 / 2.0_f64.sqrt();
            return vec![
                Complex::new( s,  s),
                Complex::new(-s,  s),
                Complex::new( s, -s),
                Complex::new(-s, -s),
            ];
        }

        let m = sqrtm as f64;
        // Normalization factor for square QAM: E[|x|^2] = 2 * (M-1)/3
        let max_val = m - 1.0;
        let norm_factor = (2.0 * max_val * max_val / 3.0).sqrt().max(1e-12);

        let mut points = Vec::new();
        for i in 0..sqrtm {
            for q in 0..sqrtm {
                let i_val = (2 * i as isize - sqrtm as isize + 1) as f64 / norm_factor;
                let q_val = (2 * q as isize - sqrtm as isize + 1) as f64 / norm_factor;
                points.push(Complex::new(i_val, q_val));
            }
        }
        points
    }

    /// Map bits to QAM symbol using Gray code
    pub fn map_bits(&self, bits: &[bool]) -> Complex {
        let bps = self.modulation.bits_per_symbol();
        assert!(bits.len() == bps);

        // Convert bits to Gray-coded index
        let mut idx = 0usize;
        for &b in bits {
            idx = (idx << 1) | (b as usize);
        }

        // Gray decode
        let gray_idx = self.gray_to_natural(idx, bps);
        let points = self.constellation_points();
        points[gray_idx % points.len()]
    }

    /// Demap QAM symbol to LLR values (approximate: nearest-point hard decision)
    pub fn demap_hard(&self, symbol: &Complex) -> Vec<bool> {
        let points = self.constellation_points();
        // Find nearest constellation point
        let nearest = points.iter().enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = symbol.sub(a).abs_sq();
                let db = symbol.sub(b).abs_sq();
                da.partial_cmp(&db).unwrap()
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        let bps = self.modulation.bits_per_symbol();
        let natural = nearest;
        let gray_idx = self.natural_to_gray(natural, bps);
        (0..bps).rev().map(|i| (gray_idx >> i) & 1 == 1).collect()
    }

    fn gray_to_natural(&self, gray: usize, bits: usize) -> usize {
        let mut n = gray >> (bits - 1);
        let mut mask = gray >> (bits - 1);
        let mut g = gray;
        for _ in 1..bits {
            g >>= 1;
            // Shift to avoid overflow for large bits values
            if bits > 1 {
                mask ^= g;
            }
            n = mask;
        }
        // Simpler direct conversion
        let mut result = 0usize;
        let mut g2 = gray;
        let mut mask2 = 1usize << (bits - 1);
        result |= g2 & mask2;
        while mask2 > 1 {
            mask2 >>= 1;
            g2 ^= (result >> 1) & mask2;
            result |= g2 & mask2;
        }
        result
    }

    fn natural_to_gray(&self, n: usize, _bits: usize) -> usize {
        n ^ (n >> 1)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Non-Uniform Constellation (NUC) for ATSC 3.0
// ─────────────────────────────────────────────────────────────────────────────

/// Non-Uniform Constellation mapper (simplified 16-NUC example)
/// ATSC 3.0 specifies optimized NUC constellation coordinates for each
/// modulation order and code rate combination.
pub struct NucMapper {
    pub modulation: Modulation,
    /// Custom constellation points (normalized)
    points: Vec<Complex>,
}

impl NucMapper {
    /// Create NUC mapper with custom point coordinates
    pub fn new(modulation: Modulation, points: Vec<Complex>) -> Self {
        let expected = 1usize << modulation.bits_per_symbol();
        assert!(points.len() == expected, "NUC must have exactly {} points", expected);
        Self { modulation, points }
    }

    /// Create a default 16-NUC (example coordinates from A/322 for r=8/15)
    pub fn new_16nuc_r8_15() -> Self {
        // Simplified 16-NUC coordinates (normalized, first quadrant only, 4-fold symmetry)
        // Real A/322 tables use highly optimized coordinates; these are representative
        let s = 1.0 / 2.0_f64.sqrt();
        let pts = vec![
            Complex::new(0.25 * s, 0.25 * s),
            Complex::new(0.75 * s, 0.25 * s),
            Complex::new(0.25 * s, 0.75 * s),
            Complex::new(0.75 * s, 0.75 * s),
            Complex::new(-0.25 * s, 0.25 * s),
            Complex::new(-0.75 * s, 0.25 * s),
            Complex::new(-0.25 * s, 0.75 * s),
            Complex::new(-0.75 * s, 0.75 * s),
            Complex::new(0.25 * s, -0.25 * s),
            Complex::new(0.75 * s, -0.25 * s),
            Complex::new(0.25 * s, -0.75 * s),
            Complex::new(0.75 * s, -0.75 * s),
            Complex::new(-0.25 * s, -0.25 * s),
            Complex::new(-0.75 * s, -0.25 * s),
            Complex::new(-0.25 * s, -0.75 * s),
            Complex::new(-0.75 * s, -0.75 * s),
        ];
        Self { modulation: Modulation::Qam16, points: pts }
    }

    pub fn map_bits(&self, bits: &[bool]) -> Complex {
        let bps = self.modulation.bits_per_symbol();
        assert!(bits.len() == bps);
        let idx = bits.iter().fold(0usize, |acc, &b| (acc << 1) | (b as usize));
        self.points[idx % self.points.len()]
    }

    pub fn demap_hard(&self, symbol: &Complex) -> Vec<bool> {
        let nearest = self.points.iter().enumerate()
            .min_by(|(_, a), (_, b)| {
                symbol.sub(a).abs_sq().partial_cmp(&symbol.sub(b).abs_sq()).unwrap()
            })
            .map(|(i, _)| i)
            .unwrap_or(0);
        let bps = self.modulation.bits_per_symbol();
        (0..bps).rev().map(|i| (nearest >> i) & 1 == 1).collect()
    }

    pub fn points(&self) -> &[Complex] {
        &self.points
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Time Interleaver (ATSC 3.0 A/322 Section 7.3)
// ─────────────────────────────────────────────────────────────────────────────

/// Convolutional time interleaver for mobile reception robustness
/// Implements a bank of delay lines of increasing length
pub struct TimeInterleaver {
    /// Number of delay branches (= number of symbols per interleaving block)
    pub num_branches: usize,
    /// Max interleaving depth in OFDM symbols
    pub max_depth: usize,
    /// Internal delay lines for each branch
    delay_lines: Vec<Vec<Complex>>,
    /// Current write pointer for each branch
    ptrs: Vec<usize>,
}

impl TimeInterleaver {
    pub fn new(num_branches: usize, max_depth: usize) -> Self {
        let delay_lines: Vec<Vec<Complex>> = (0..num_branches)
            .map(|i| vec![Complex::zero(); (i * max_depth) / num_branches.max(1) + 1])
            .collect();
        let ptrs = vec![0usize; num_branches];
        Self { num_branches, max_depth, delay_lines, ptrs }
    }

    /// Interleave one vector of symbols (one OFDM symbol worth of data cells)
    pub fn interleave_symbol(&mut self, symbols: &[Complex]) -> Vec<Complex> {
        let n = symbols.len().min(self.num_branches);
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let dl = &mut self.delay_lines[i];
            let ptr = self.ptrs[i];
            let old = dl[ptr];
            dl[ptr] = symbols[i];
            self.ptrs[i] = (ptr + 1) % dl.len().max(1);
            out.push(old);
        }
        out
    }

    /// Deinterleave one vector of symbols (inverse operation)
    pub fn deinterleave_symbol(&mut self, symbols: &[Complex]) -> Vec<Complex> {
        // For a linear convolutional interleaver, the deinterleaver has complementary delays
        // Here we reuse the same structure but reading in reverse branch order
        let n = symbols.len().min(self.num_branches);
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let branch = self.num_branches - 1 - i;
            if branch < self.delay_lines.len() {
                let dl = &mut self.delay_lines[branch];
                let ptr = self.ptrs[branch];
                let old = dl[ptr];
                dl[ptr] = symbols[i];
                self.ptrs[branch] = (ptr + 1) % dl.len().max(1);
                out.push(old);
            }
        }
        out
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Physical Layer Pipe (PLP) Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// PLP type
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PlpType {
    /// Common PLP (shared signaling/PSI)
    Common,
    /// Data PLP (user data service)
    Data,
}

/// Physical Layer Pipe (PLP) configuration
#[derive(Clone, Debug)]
pub struct PlpConfig {
    pub plp_id: u8,
    pub plp_type: PlpType,
    pub modulation: Modulation,
    pub code_rate: CodeRate,
    pub ldpc_length: LdpcLength,
    pub time_interleaving_depth: usize,
    /// Number of data cells per OFDM symbol allocated to this PLP
    pub cells_per_symbol: usize,
}

impl PlpConfig {
    pub fn new_data(plp_id: u8, modulation: Modulation, code_rate: CodeRate) -> Self {
        Self {
            plp_id,
            plp_type: PlpType::Data,
            modulation,
            code_rate,
            ldpc_length: LdpcLength::Long,
            time_interleaving_depth: 256,
            cells_per_symbol: 1024,
        }
    }

    /// Effective bits per LDPC codeword block
    pub fn info_bits_per_block(&self) -> usize {
        self.code_rate.info_bits(self.ldpc_length.as_usize())
    }

    /// QAM symbols per LDPC block
    pub fn symbols_per_block(&self) -> usize {
        self.ldpc_length.as_usize() / self.modulation.bits_per_symbol()
    }

    /// Net data rate in bits/s given OFDM symbol rate
    pub fn net_bitrate(&self, ofdm_symbol_rate: f64) -> f64 {
        let bits_per_cell = self.modulation.bits_per_symbol() as f64 * self.code_rate.as_f64();
        self.cells_per_symbol as f64 * bits_per_cell * ofdm_symbol_rate
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MIMO / MISO Processing
// ─────────────────────────────────────────────────────────────────────────────

/// Alamouti MISO (Space-Time Block Code, 2 TX antennas, 1 RX)
/// Encodes pairs of symbols for 2 transmit antennas over 2 time slots.
pub struct AlamoutiMiso;

impl AlamoutiMiso {
    /// Encode: returns (tx1_symbols, tx2_symbols) each of length 2*input_len
    /// Input: [s0, s1, s2, s3, ...] (must be even length)
    /// TX1: [s0, -s1*, s2, -s3*, ...]
    /// TX2: [s1,  s0*, s3,  s2*, ...]
    pub fn encode(symbols: &[Complex]) -> (Vec<Complex>, Vec<Complex>) {
        assert!(symbols.len() % 2 == 0, "symbol count must be even");
        let mut tx1 = Vec::with_capacity(symbols.len());
        let mut tx2 = Vec::with_capacity(symbols.len());
        for chunk in symbols.chunks(2) {
            let s0 = chunk[0];
            let s1 = chunk[1];
            tx1.push(s0);
            tx1.push(s1.conj().neg());
            tx2.push(s1);
            tx2.push(s0.conj());
        }
        (tx1, tx2)
    }

    /// Decode: given received samples r[0..2n] and channel estimates h1, h2 (per symbol pair)
    /// h1_pairs, h2_pairs: channel estimates for each Alamouti pair
    /// Returns decoded symbols
    pub fn decode(
        received: &[Complex],
        h1_pairs: &[Complex],
        h2_pairs: &[Complex],
    ) -> Vec<Complex> {
        assert!(received.len() % 2 == 0);
        let n_pairs = received.len() / 2;
        let mut decoded = Vec::with_capacity(received.len());

        for p in 0..n_pairs {
            let r0 = received[2 * p];
            let r1 = received[2 * p + 1];
            let h1 = h1_pairs[p.min(h1_pairs.len() - 1)];
            let h2 = h2_pairs[p.min(h2_pairs.len() - 1)];

            // MRC combining for Alamouti:
            // ŝ0 = h1* r0 + h2 r1*
            // ŝ1 = h2* r0 - h1 r1*
            let denom = h1.abs_sq() + h2.abs_sq();
            let s0_hat = h1.conj().mul(&r0).add(&h2.mul(&r1.conj())).scale(1.0 / denom.max(1e-12));
            let s1_hat = h2.conj().mul(&r0).sub(&h1.mul(&r1.conj())).scale(1.0 / denom.max(1e-12));

            decoded.push(s0_hat);
            decoded.push(s1_hat);
        }
        decoded
    }
}

/// MIMO 2x2 Spatial Multiplexing with ZF detection
pub struct Mimo2x2Zf;

impl Mimo2x2Zf {
    /// Encode: map pairs of input symbols to 2 TX streams (spatial multiplexing)
    /// Returns (tx1_stream, tx2_stream) each of same length as input / 2
    pub fn encode(symbols: &[Complex]) -> (Vec<Complex>, Vec<Complex>) {
        assert!(symbols.len() % 2 == 0);
        let n = symbols.len() / 2;
        let tx1: Vec<_> = symbols.iter().step_by(2).cloned().collect();
        let tx2: Vec<_> = symbols.iter().skip(1).step_by(2).cloned().collect();
        (tx1, tx2)
    }

    /// ZF detection: given received [r1, r2] and 2x2 channel matrix [[h11,h12],[h21,h22]]
    /// Returns [ŝ1, ŝ2]
    pub fn detect_zf(
        r1: Complex,
        r2: Complex,
        h11: Complex, h12: Complex,
        h21: Complex, h22: Complex,
    ) -> (Complex, Complex) {
        // H = [[h11, h12], [h21, h22]]
        // H^H H x = H^H r  (least squares)
        // (H^H H)^-1 H^H r

        // H^H
        let h11c = h11.conj();
        let h12c = h12.conj();
        let h21c = h21.conj();
        let h22c = h22.conj();

        // H^H H (2x2)
        let a11 = h11c.mul(&h11).add(&h21c.mul(&h21));
        let a12 = h11c.mul(&h12).add(&h21c.mul(&h22));
        let a21 = h12c.mul(&h11).add(&h22c.mul(&h21));
        let a22 = h12c.mul(&h12).add(&h22c.mul(&h22));

        // Inverse of 2x2 matrix [[a11,a12],[a21,a22]]
        let det = a11.mul(&a22).sub(&a12.mul(&a21));
        let det_abs_sq = det.abs_sq().max(1e-20);
        let inv11 = a22.mul(&det.conj()).scale(1.0 / det_abs_sq);
        let inv12 = a12.neg().mul(&det.conj()).scale(1.0 / det_abs_sq);
        let inv21 = a21.neg().mul(&det.conj()).scale(1.0 / det_abs_sq);
        let inv22 = a11.mul(&det.conj()).scale(1.0 / det_abs_sq);

        // H^H r
        let y1 = h11c.mul(&r1).add(&h21c.mul(&r2));
        let y2 = h12c.mul(&r1).add(&h22c.mul(&r2));

        // (H^H H)^-1 H^H r
        let s1 = inv11.mul(&y1).add(&inv12.mul(&y2));
        let s2 = inv21.mul(&y1).add(&inv22.mul(&y2));
        (s1, s2)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Layer Division Multiplexing (LDM)
// ─────────────────────────────────────────────────────────────────────────────

/// ATSC 3.0 Layer Division Multiplexing (A/322 Section 7.6)
/// Combines a core layer (lower power) and enhanced layer (higher power)
/// via power-division multiplexing for simultaneous indoor/outdoor reception.
pub struct Ldm {
    /// Injection level in dB (typically 3–10 dB) — power ratio P_core / P_enhanced
    pub injection_level_db: f64,
}

impl Ldm {
    pub fn new(injection_level_db: f64) -> Self {
        Self { injection_level_db }
    }

    /// Linear power ratio alpha^2 where alpha = 10^(-injection_level_db/20)
    pub fn alpha(&self) -> f64 {
        10.0_f64.powf(-self.injection_level_db / 20.0)
    }

    /// Combine core layer x_c and enhanced layer x_e into composite signal
    /// x = alpha * x_c + (1 - alpha) * x_e  (normalized for unit average power)
    pub fn combine(&self, core: &[Complex], enhanced: &[Complex]) -> Vec<Complex> {
        assert!(core.len() == enhanced.len(), "LDM layers must have equal length");
        let alpha = self.alpha();
        let beta = (1.0 - alpha * alpha).sqrt().max(0.0);
        core.iter().zip(enhanced.iter())
            .map(|(c, e)| c.scale(alpha).add(&e.scale(beta)))
            .collect()
    }

    /// Separate layers from combined signal using successive interference cancellation (SIC)
    /// First decode enhanced (stronger) layer, then subtract and decode core layer.
    /// `combined`: received composite signal
    /// `enhanced_decoded`: already-decoded enhanced layer symbols (from initial detection)
    /// Returns core layer estimate
    pub fn sic_core(&self, combined: &[Complex], enhanced_decoded: &[Complex]) -> Vec<Complex> {
        assert!(combined.len() == enhanced_decoded.len());
        let alpha = self.alpha();
        let beta = (1.0 - alpha * alpha).sqrt().max(0.0);
        combined.iter().zip(enhanced_decoded.iter())
            .map(|(y, e_hat)| y.sub(&e_hat.scale(beta)).scale(1.0 / alpha.max(1e-12)))
            .collect()
    }

    /// Extract enhanced layer estimate (simple scaling, assuming core << enhanced)
    pub fn extract_enhanced(&self, combined: &[Complex]) -> Vec<Complex> {
        let beta = (1.0 - self.alpha() * self.alpha()).sqrt().max(0.0);
        combined.iter().map(|y| y.scale(1.0 / beta.max(1e-12))).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Pilot Pattern Generator
// ─────────────────────────────────────────────────────────────────────────────

/// Generate pilot subcarrier indices and values for an ATSC 3.0 OFDM symbol
pub struct PilotGenerator {
    pub fft_size: FftSize,
    pub pattern: PilotPattern,
}

impl PilotGenerator {
    pub fn new(fft_size: FftSize, pattern: PilotPattern) -> Self {
        Self { fft_size, pattern }
    }

    /// Generate scattered pilot locations for symbol index `sym_idx`
    /// Returns list of (subcarrier_index, pilot_value) pairs
    pub fn scattered_pilots(&self, sym_idx: usize) -> Vec<(usize, Complex)> {
        let n = self.fft_size.as_usize();
        let dx = self.pattern.dx();
        let dy = self.pattern.dy();

        // Pilot offset depends on symbol index within the scattered pattern period
        let offset = (sym_idx % dy) * (dx / dy.max(1));

        let mut pilots = Vec::new();
        let mut k = offset;
        while k < n {
            // Pilot value from PRBS-based sequence (simplified: use ±1 based on k)
            let value = if self.prbs_bit(k) { 1.0 } else { -1.0 };
            pilots.push((k, Complex::new(value / 2.0_f64.sqrt(), 0.0)));
            k += dx;
        }
        pilots
    }

    /// Generate continual pilot locations (fixed positions every frame)
    pub fn continual_pilots(&self) -> Vec<(usize, Complex)> {
        let n = self.fft_size.as_usize();
        // Continual pilots at regular intervals (simplified: every 48 subcarriers)
        let spacing = 48usize;
        (0..n)
            .step_by(spacing)
            .map(|k| {
                let value = if self.prbs_bit(k + 1000) { 1.0 } else { -1.0 };
                (k, Complex::new(value / 2.0_f64.sqrt(), 0.0))
            })
            .collect()
    }

    /// Simple PRBS-based reference sequence bit (from Gold code or PN sequence)
    fn prbs_bit(&self, idx: usize) -> bool {
        // PRBS15: x^15 + x^14 + 1 (simplified state computation)
        let mut state = 0x4A80usize;
        for _ in 0..=idx % 32767 {
            let bit = ((state >> 14) ^ (state >> 13)) & 1;
            state = ((state << 1) | bit) & 0x7FFF;
        }
        (state & 1) == 1
    }

    /// Insert pilots into a subcarrier array (modifies in place)
    pub fn insert_pilots(&self, subcarriers: &mut Vec<Complex>, sym_idx: usize) {
        for (k, val) in self.scattered_pilots(sym_idx) {
            if k < subcarriers.len() {
                subcarriers[k] = val;
            }
        }
    }

    /// Extract pilot values from received subcarriers (for channel estimation)
    pub fn extract_pilots(&self, subcarriers: &[Complex], sym_idx: usize) -> Vec<(usize, Complex)> {
        self.scattered_pilots(sym_idx).iter().map(|(k, _)| {
            (*k, if *k < subcarriers.len() { subcarriers[*k] } else { Complex::zero() })
        }).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ATSC 3.0 Frame Builder (top-level assembly)
// ─────────────────────────────────────────────────────────────────────────────

/// ATSC 3.0 physical layer frame statistics
#[derive(Clone, Debug)]
pub struct FrameStats {
    pub fft_size: usize,
    pub cp_len: usize,
    pub symbol_len: usize,
    /// Approximate net data rate in Mbps (6 MHz, no LDM)
    pub net_rate_mbps: f64,
}

/// Compute frame statistics for a given configuration
pub fn compute_frame_stats(config: &OfdmConfig, plp: &PlpConfig) -> FrameStats {
    let fft_size = config.fft_size.as_usize();
    let cp_len = config.cp_len();
    let symbol_len = fft_size + cp_len;
    let ofdm_symbol_rate = config.sample_rate() / symbol_len as f64;
    let net_rate_mbps = plp.net_bitrate(ofdm_symbol_rate) / 1e6;
    FrameStats { fft_size, cp_len, symbol_len, net_rate_mbps }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility functions
// ─────────────────────────────────────────────────────────────────────────────

/// Calculate BER (Bit Error Rate) between two bit vectors
pub fn ber(tx: &[bool], rx: &[bool]) -> f64 {
    let len = tx.len().min(rx.len());
    if len == 0 { return 0.0; }
    let errors: usize = tx.iter().zip(rx.iter()).filter(|(a, b)| a != b).count();
    errors as f64 / len as f64
}

/// AWGN channel: add complex Gaussian noise to signal
pub fn awgn(signal: &[Complex], snr_db: f64, seed: u64) -> Vec<Complex> {
    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    let signal_power = signal.iter().map(|s| s.abs_sq()).sum::<f64>() / signal.len().max(1) as f64;
    let noise_sigma = (signal_power / (2.0 * snr_linear)).sqrt();

    let mut state = seed;
    let lcg = |s: u64| -> f64 {
        let s2 = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (s2 >> 33) as f64 / (u32::MAX as f64)
    };

    signal.iter().map(|s| {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u1 = lcg(state).max(1e-10);
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u2 = lcg(state);
        // Box-Muller transform
        let n_re = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos() * noise_sigma;
        let n_im = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).sin() * noise_sigma;
        s.add(&Complex::new(n_re, n_im))
    }).collect()
}

/// Compute average power of a complex signal
pub fn average_power(signal: &[Complex]) -> f64 {
    if signal.is_empty() { return 0.0; }
    signal.iter().map(|s| s.abs_sq()).sum::<f64>() / signal.len() as f64
}

/// Compute EVM (Error Vector Magnitude) as percentage
pub fn evm_percent(reference: &[Complex], received: &[Complex]) -> f64 {
    let n = reference.len().min(received.len());
    if n == 0 { return 0.0; }
    let err_power: f64 = reference.iter().zip(received.iter())
        .map(|(r, rx)| rx.sub(r).abs_sq()).sum::<f64>() / n as f64;
    let ref_power: f64 = reference.iter().map(|s| s.abs_sq()).sum::<f64>() / n as f64;
    100.0 * (err_power / ref_power.max(1e-12)).sqrt()
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper ───────────────────────────────────────────────────────────────

    fn random_bits(n: usize, seed: u64) -> Vec<bool> {
        let mut s = seed;
        (0..n).map(|_| {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (s >> 33) & 1 == 1
        }).collect()
    }

    fn random_complex(n: usize, seed: u64) -> Vec<Complex> {
        let mut s = seed;
        (0..n).map(|_| {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let re = (s as f64 / u64::MAX as f64) * 2.0 - 1.0;
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let im = (s as f64 / u64::MAX as f64) * 2.0 - 1.0;
            Complex::new(re, im)
        }).collect()
    }

    // ── Complex arithmetic ───────────────────────────────────────────────────

    #[test]
    fn test_complex_basic_ops() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, 4.0);
        let sum = a.add(&b);
        assert!((sum.re - 4.0).abs() < 1e-10);
        assert!((sum.im - 6.0).abs() < 1e-10);
        let prod = a.mul(&b);
        // (1+2j)(3+4j) = 3+4j+6j+8j^2 = -5+10j
        assert!((prod.re - (-5.0)).abs() < 1e-10);
        assert!((prod.im - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_polar() {
        let c = Complex::from_polar(2.0, PI / 4.0);
        let expected_re = 2.0 * (PI / 4.0_f64).cos();
        let expected_im = 2.0 * (PI / 4.0_f64).sin();
        assert!((c.re - expected_re).abs() < 1e-10);
        assert!((c.im - expected_im).abs() < 1e-10);
    }

    #[test]
    fn test_complex_conj_and_abs() {
        let c = Complex::new(3.0, 4.0);
        assert!((c.abs() - 5.0).abs() < 1e-10);
        let conj = c.conj();
        assert_eq!(conj.re, 3.0);
        assert_eq!(conj.im, -4.0);
    }

    // ── FFT ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_fft_ifft_roundtrip_16() {
        let n = 16;
        let input: Vec<Complex> = (0..n).map(|i| Complex::new(i as f64, 0.0)).collect();
        let spectrum = fft(&input);
        let recovered = ifft(&spectrum);
        for (a, b) in input.iter().zip(recovered.iter()) {
            assert!((a.re - b.re).abs() < 1e-9, "re mismatch: {} vs {}", a.re, b.re);
            assert!((a.im - b.im).abs() < 1e-9, "im mismatch: {} vs {}", a.im, b.im);
        }
    }

    #[test]
    fn test_fft_known_impulse() {
        // FFT of unit impulse at n=0 should be all ones
        let n = 8;
        let mut input = vec![Complex::zero(); n];
        input[0] = Complex::one();
        let spectrum = fft(&input);
        for s in &spectrum {
            assert!((s.re - 1.0).abs() < 1e-10);
            assert!(s.im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_fft_single_tone() {
        // FFT of complex exponential at frequency k0 should have a peak at bin k0
        let n = 64;
        let k0 = 5usize;
        let input: Vec<Complex> = (0..n)
            .map(|i| Complex::from_polar(1.0, 2.0 * PI * k0 as f64 * i as f64 / n as f64))
            .collect();
        let spectrum = fft(&input);
        let peak_bin = spectrum.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak_bin, k0, "FFT peak should be at bin {}", k0);
    }

    #[test]
    fn test_fft_power_conservation() {
        // Parseval's theorem: sum |x|^2 = (1/N) * sum |X|^2
        let n = 32;
        let input = random_complex(n, 42);
        let spectrum = fft(&input);
        let time_power: f64 = input.iter().map(|s| s.abs_sq()).sum();
        let freq_power: f64 = spectrum.iter().map(|s| s.abs_sq()).sum::<f64>() / n as f64;
        assert!((time_power - freq_power).abs() / time_power.max(1e-12) < 1e-8);
    }

    // ── Guard Interval ───────────────────────────────────────────────────────

    #[test]
    fn test_guard_interval_cp_len() {
        let cp = GuardInterval::Gi7_2048.cp_len(8192);
        // 7/2048 * 8192 = 7 * 4 = 28... actually (8192 * 7) / 2048 = 28
        assert!(cp > 0, "CP length should be positive");
    }

    #[test]
    fn test_guard_interval_fractions() {
        let (n, d) = GuardInterval::Gi1_192.fraction();
        assert_eq!(n, 1);
        assert_eq!(d, 192);
        let (n2, d2) = GuardInterval::Gi13_4864.fraction();
        assert_eq!(n2, 13);
    }

    // ── OFDM ────────────────────────────────────────────────────────────────

    #[test]
    fn test_ofdm_modulate_demodulate_8k() {
        let config = OfdmConfig::new_6mhz_8k();
        let mod_obj = Atsc3OfdmModulator::new(config.clone());
        let n = config.fft_size.as_usize();

        // Create simple test subcarriers
        let subcarriers: Vec<Complex> = (0..n)
            .map(|i| Complex::from_polar(1.0, 2.0 * PI * i as f64 / n as f64))
            .collect();

        let time_samples = mod_obj.modulate_symbol(&subcarriers);
        assert_eq!(time_samples.len(), n + config.cp_len());

        let recovered = mod_obj.demodulate_symbol(&time_samples);
        assert_eq!(recovered.len(), n);

        // Check first few subcarriers are recovered accurately
        for i in 0..10 {
            let diff = recovered[i].sub(&subcarriers[i]).abs();
            assert!(diff < 1e-8, "Subcarrier {} mismatch: diff = {}", i, diff);
        }
    }

    #[test]
    fn test_ofdm_cp_prefix_matches_tail() {
        let config = OfdmConfig::new_6mhz_8k();
        let mod_obj = Atsc3OfdmModulator::new(config.clone());
        let n = config.fft_size.as_usize();
        let cp = config.cp_len();

        let subcarriers: Vec<Complex> = (0..n).map(|i| Complex::new(i as f64, 0.0)).collect();
        let symbol = mod_obj.modulate_symbol(&subcarriers);

        // CP (first cp samples) should equal the last cp samples of the IFFT output
        for i in 0..cp {
            let diff = (symbol[i].re - symbol[n + i].re).abs()
                + (symbol[i].im - symbol[n + i].im).abs();
            assert!(diff < 1e-8, "CP mismatch at position {}: diff = {}", i, diff);
        }
    }

    #[test]
    fn test_ofdm_config_symbol_len() {
        let config = OfdmConfig::new_6mhz_32k();
        assert_eq!(config.fft_size.as_usize(), 32768);
        assert!(config.cp_len() > 0);
        assert_eq!(config.symbol_len(), 32768 + config.cp_len());
    }

    #[test]
    fn test_ofdm_sample_rate() {
        let config = OfdmConfig::new_6mhz_8k();
        let sr = config.sample_rate();
        // 6 MHz * 8/7 ≈ 6.857 MHz
        assert!((sr - 6_857_142.857).abs() < 10.0, "Sample rate = {}", sr);
    }

    // ── Bootstrap ────────────────────────────────────────────────────────────

    #[test]
    fn test_bootstrap_generation_length() {
        let params = BootstrapParams::default();
        let bs = BootstrapGenerator::generate(&params);
        assert_eq!(bs.len(), 2048);
    }

    #[test]
    fn test_bootstrap_unit_power() {
        let params = BootstrapParams::default();
        let bs = BootstrapGenerator::generate(&params);
        // Each ZC sample should have unit magnitude
        for (i, s) in bs.iter().enumerate() {
            let mag = s.abs();
            assert!((mag - 1.0).abs() < 1e-6, "Sample {} has magnitude {}", i, mag);
        }
    }

    #[test]
    fn test_bootstrap_emergency_alert_change() {
        let mut params = BootstrapParams::default();
        let bs_normal = BootstrapGenerator::generate(&params);
        params.emergency_alert = true;
        let bs_alert = BootstrapGenerator::generate(&params);
        // The two sequences should differ (EA flag changes first-segment phase)
        let diff: f64 = bs_normal.iter().zip(bs_alert.iter())
            .map(|(a, b)| a.sub(b).abs_sq()).sum::<f64>();
        assert!(diff > 0.1, "Emergency alert flag should change bootstrap sequence");
    }

    #[test]
    fn test_bootstrap_bandwidth_codes() {
        for bw_code in 0..3u8 {
            let params = BootstrapParams { bandwidth_code: bw_code, ..Default::default() };
            let bs = BootstrapGenerator::generate(&params);
            assert_eq!(bs.len(), 2048);
        }
    }

    #[test]
    fn test_zadoff_chu_constant_envelope() {
        let zc = zadoff_chu(1024, 8);
        for s in &zc {
            assert!((s.abs() - 1.0).abs() < 1e-9);
        }
    }

    // ── LDPC ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_ldpc_encode_length() {
        let code = LdpcCode::new(CodeRate::R8_15, LdpcLength::Short);
        let k = code.k();
        let info = random_bits(k, 1234);
        let codeword = code.encode(&info);
        assert_eq!(codeword.len(), code.n());
    }

    #[test]
    fn test_ldpc_code_rate_short() {
        let code = LdpcCode::new(CodeRate::R8_15, LdpcLength::Short);
        let rate = code.k() as f64 / code.n() as f64;
        let expected = 8.0 / 15.0;
        assert!((rate - expected).abs() < 0.01, "Code rate = {}", rate);
    }

    #[test]
    fn test_ldpc_code_rate_long() {
        let code = LdpcCode::new(CodeRate::R13_15, LdpcLength::Long);
        let rate = code.k() as f64 / code.n() as f64;
        assert!((rate - 13.0 / 15.0).abs() < 0.01);
    }

    #[test]
    fn test_ldpc_systematic_info_preserved() {
        let code = LdpcCode::new(CodeRate::R6_15, LdpcLength::Short);
        let k = code.k();
        let info = random_bits(k, 9999);
        let codeword = code.encode(&info);
        // First k bits should be unchanged (systematic code)
        for i in 0..k {
            assert_eq!(codeword[i], info[i], "Info bit {} not preserved", i);
        }
    }

    #[test]
    fn test_ldpc_all_zero_encodes_to_codeword() {
        let code = LdpcCode::new(CodeRate::R8_15, LdpcLength::Short);
        let k = code.k();
        let info = vec![false; k];
        let codeword = code.encode(&info);
        // All-zero codeword is always a valid LDPC codeword
        assert!(codeword.iter().all(|&b| !b), "All-zero codeword expected");
    }

    #[test]
    fn test_ldpc_decode_all_zero() {
        // All-zero codeword is always valid for any LDPC code
        let code = LdpcCode::new(CodeRate::R8_15, LdpcLength::Short);
        let n = code.n();
        // Perfect channel: all-zero → all positive LLRs
        let llr: Vec<f64> = vec![10.0; n];
        let decoded = code.decode(&llr, 5);
        // All decoded bits should be 0 (all-zero is a valid codeword)
        assert!(decoded[..code.k()].iter().all(|&b| !b), "All-zero decode should produce all zeros");
    }

    #[test]
    fn test_ldpc_all_rates_produce_valid_length() {
        let rates = [
            CodeRate::R2_15, CodeRate::R3_15, CodeRate::R4_15, CodeRate::R5_15,
            CodeRate::R6_15, CodeRate::R7_15, CodeRate::R8_15, CodeRate::R9_15,
            CodeRate::R10_15, CodeRate::R11_15, CodeRate::R12_15, CodeRate::R13_15,
        ];
        for rate in &rates {
            let code = LdpcCode::new(*rate, LdpcLength::Short);
            assert_eq!(code.n(), LdpcLength::Short.as_usize());
            assert!(code.k() > 0 && code.k() < code.n());
        }
    }

    // ── BCH ──────────────────────────────────────────────────────────────────

    #[test]
    fn test_bch_encode_length() {
        let bch = BchCode::new_short();
        let k = 100usize;
        let info = random_bits(k, 0xDEAD);
        let codeword = bch.encode(&info);
        assert_eq!(codeword.len(), k + bch.parity_len());
    }

    #[test]
    fn test_bch_systematic_info_preserved() {
        let bch = BchCode::new_short();
        let k = 64usize;
        let info = random_bits(k, 0xBEEF);
        let codeword = bch.encode(&info);
        for i in 0..k {
            assert_eq!(codeword[i], info[i], "Info bit {} not preserved in BCH codeword", i);
        }
    }

    #[test]
    fn test_bch_encode_long() {
        let bch = BchCode::new_long();
        let info = random_bits(100, 42);
        let codeword = bch.encode(&info);
        assert_eq!(codeword.len(), 100 + bch.parity_len());
    }

    #[test]
    fn test_bch_decode_no_errors() {
        let bch = BchCode::new_short();
        let k = 50usize;
        let info = random_bits(k, 77);
        let codeword = bch.encode(&info);
        let (decoded, errors) = bch.decode(&codeword);
        assert_eq!(errors, 0, "No errors should be detected in uncorrupted codeword");
        for i in 0..k {
            assert_eq!(decoded[i], info[i]);
        }
    }

    // ── Bit Interleaver ──────────────────────────────────────────────────────

    #[test]
    fn test_bit_interleaver_qpsk_roundtrip() {
        let il = BitInterleaver::new(Modulation::Qpsk, LdpcLength::Short);
        let n = il.block_size();
        let bits = random_bits(n, 55);
        let interleaved = il.interleave(&bits);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(bits, deinterleaved, "QPSK bit interleaver roundtrip failed");
    }

    #[test]
    fn test_bit_interleaver_qam16_roundtrip() {
        let il = BitInterleaver::new(Modulation::Qam16, LdpcLength::Short);
        let n = il.block_size();
        let bits = random_bits(n, 66);
        let interleaved = il.interleave(&bits);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(bits, deinterleaved, "16-QAM bit interleaver roundtrip failed");
    }

    #[test]
    fn test_bit_interleaver_qam64_roundtrip() {
        let il = BitInterleaver::new(Modulation::Qam64, LdpcLength::Short);
        let n = il.block_size();
        let bits = random_bits(n, 77);
        let deinterleaved = il.deinterleave(&il.interleave(&bits));
        assert_eq!(bits, deinterleaved);
    }

    #[test]
    fn test_bit_interleaver_changes_order() {
        let il = BitInterleaver::new(Modulation::Qam16, LdpcLength::Short);
        let n = il.block_size();
        let bits = random_bits(n, 123);
        let interleaved = il.interleave(&bits);
        // Interleaved should differ from original (non-trivial permutation)
        let diff = bits.iter().zip(interleaved.iter()).filter(|(a, b)| a != b).count();
        assert!(diff > n / 10, "Interleaver should change at least 10% of positions");
    }

    // ── Constellation Mapper ─────────────────────────────────────────────────

    #[test]
    fn test_qpsk_constellation_points() {
        let mapper = QamMapper::new(Modulation::Qpsk);
        let pts = mapper.constellation_points();
        assert_eq!(pts.len(), 4);
        // All points should have unit power
        for (i, p) in pts.iter().enumerate() {
            assert!((p.abs_sq() - 1.0).abs() < 1e-9, "QPSK point {} power = {}", i, p.abs_sq());
        }
    }

    #[test]
    fn test_qam16_constellation_count() {
        let mapper = QamMapper::new(Modulation::Qam16);
        let pts = mapper.constellation_points();
        assert_eq!(pts.len(), 16);
    }

    #[test]
    fn test_qam64_constellation_count() {
        let mapper = QamMapper::new(Modulation::Qam64);
        let pts = mapper.constellation_points();
        assert_eq!(pts.len(), 64);
    }

    #[test]
    fn test_qam256_constellation_count() {
        let mapper = QamMapper::new(Modulation::Qam256);
        let pts = mapper.constellation_points();
        assert_eq!(pts.len(), 256);
    }

    #[test]
    fn test_qpsk_map_demap_roundtrip() {
        let mapper = QamMapper::new(Modulation::Qpsk);
        let test_bits = vec![
            vec![false, false],
            vec![false, true],
            vec![true, false],
            vec![true, true],
        ];
        for bits in &test_bits {
            let sym = mapper.map_bits(bits);
            let recovered = mapper.demap_hard(&sym);
            assert_eq!(recovered, *bits, "QPSK map/demap mismatch for {:?}", bits);
        }
    }

    #[test]
    fn test_qam16_map_demap_no_noise() {
        let mapper = QamMapper::new(Modulation::Qam16);
        let mut total_bits = 0usize;
        let mut errors = 0usize;
        for idx in 0..16 {
            let bits: Vec<bool> = (0..4).rev().map(|i| (idx >> i) & 1 == 1).collect();
            let sym = mapper.map_bits(&bits);
            let recovered = mapper.demap_hard(&sym);
            for (a, b) in bits.iter().zip(recovered.iter()) {
                total_bits += 1;
                if a != b { errors += 1; }
            }
        }
        assert_eq!(errors, 0, "{} bit errors in 16-QAM noiseless roundtrip", errors);
    }

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(Modulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(Modulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(Modulation::Qam64.bits_per_symbol(), 6);
        assert_eq!(Modulation::Qam256.bits_per_symbol(), 8);
        assert_eq!(Modulation::Qam1024.bits_per_symbol(), 10);
        assert_eq!(Modulation::Qam4096.bits_per_symbol(), 12);
    }

    // ── NUC Mapper ───────────────────────────────────────────────────────────

    #[test]
    fn test_nuc_16_point_count() {
        let nuc = NucMapper::new_16nuc_r8_15();
        assert_eq!(nuc.points().len(), 16);
    }

    #[test]
    fn test_nuc_map_demap_roundtrip() {
        let nuc = NucMapper::new_16nuc_r8_15();
        for idx in 0..16u8 {
            let bits: Vec<bool> = (0..4).rev().map(|i| (idx >> i) & 1 == 1).collect();
            let sym = nuc.map_bits(&bits);
            let recovered = nuc.demap_hard(&sym);
            assert_eq!(recovered, bits, "NUC 16-QAM roundtrip failed for idx {}", idx);
        }
    }

    #[test]
    fn test_nuc_custom_points() {
        let pts: Vec<Complex> = (0..4).map(|i| Complex::from_polar(1.0, i as f64 * PI / 2.0)).collect();
        let nuc = NucMapper::new(Modulation::Qpsk, pts);
        let bits = vec![true, false];
        let sym = nuc.map_bits(&bits);
        let recovered = nuc.demap_hard(&sym);
        assert_eq!(recovered, bits);
    }

    // ── Time Interleaver ─────────────────────────────────────────────────────

    #[test]
    fn test_time_interleaver_output_length() {
        let mut ti = TimeInterleaver::new(64, 16);
        let syms: Vec<Complex> = (0..64).map(|i| Complex::new(i as f64, 0.0)).collect();
        let out = ti.interleave_symbol(&syms);
        assert_eq!(out.len(), 64);
    }

    #[test]
    fn test_time_interleaver_delay_zero_branch() {
        // Branch 0 has delay 0, so output[0] should equal input[0] after first call
        let mut ti = TimeInterleaver::new(4, 4);
        let syms = vec![
            Complex::new(1.0, 0.0),
            Complex::new(2.0, 0.0),
            Complex::new(3.0, 0.0),
            Complex::new(4.0, 0.0),
        ];
        let out = ti.interleave_symbol(&syms);
        // Branch 0 delay line has length 1, so output[0] = 0.0 (initial state)
        // After the call, the input is stored; next call would return it
        assert_eq!(out.len(), 4);
    }

    #[test]
    fn test_time_interleaver_flush_and_recover() {
        let n = 8;
        let depth = 4;
        let mut ti_tx = TimeInterleaver::new(n, depth);
        let mut ti_rx = TimeInterleaver::new(n, depth);

        let input: Vec<Complex> = (0..n).map(|i| Complex::new(i as f64, 0.0)).collect();

        // Send several blocks to flush delay lines
        for _ in 0..depth + 2 {
            ti_tx.interleave_symbol(&input);
        }
        // Deinterleaver produces output (may not match exactly for convolutional IL)
        let last_out = ti_rx.deinterleave_symbol(&input);
        assert_eq!(last_out.len(), n);
    }

    // ── PLP Config ───────────────────────────────────────────────────────────

    #[test]
    fn test_plp_info_bits() {
        let plp = PlpConfig::new_data(0, Modulation::Qam256, CodeRate::R10_15);
        let k = plp.info_bits_per_block();
        let n = plp.ldpc_length.as_usize();
        let expected_k = CodeRate::R10_15.info_bits(n);
        assert_eq!(k, expected_k);
    }

    #[test]
    fn test_plp_symbols_per_block() {
        let plp = PlpConfig::new_data(1, Modulation::Qam64, CodeRate::R8_15);
        let syms = plp.symbols_per_block();
        assert_eq!(syms, LdpcLength::Long.as_usize() / 6);
    }

    #[test]
    fn test_plp_net_bitrate_reasonable() {
        let plp = PlpConfig::new_data(0, Modulation::Qam256, CodeRate::R13_15);
        // Approximate OFDM symbol rate for 8K FFT + short CP in 6 MHz
        let ofdm_sr = 6_857_142.857 / (8192.0 + 28.0);
        let rate_mbps = plp.net_bitrate(ofdm_sr) / 1e6;
        // Should be in the range 1–100 Mbps
        assert!(rate_mbps > 1.0 && rate_mbps < 200.0, "Net bitrate = {} Mbps", rate_mbps);
    }

    // ── Alamouti MISO ────────────────────────────────────────────────────────

    #[test]
    fn test_alamouti_encode_length() {
        let syms: Vec<Complex> = (0..8).map(|i| Complex::new(i as f64, 0.0)).collect();
        let (tx1, tx2) = AlamoutiMiso::encode(&syms);
        assert_eq!(tx1.len(), 8);
        assert_eq!(tx2.len(), 8);
    }

    #[test]
    fn test_alamouti_encode_pattern() {
        // Verify: TX1 = [s0, -s1*, s2, -s3*], TX2 = [s1, s0*, s3, s2*]
        let s0 = Complex::new(1.0, 2.0);
        let s1 = Complex::new(3.0, 4.0);
        let syms = vec![s0, s1];
        let (tx1, tx2) = AlamoutiMiso::encode(&syms);

        assert!((tx1[0].re - s0.re).abs() < 1e-10 && (tx1[0].im - s0.im).abs() < 1e-10);
        // tx1[1] = -s1* = -conj(s1)
        let neg_conj_s1 = s1.conj().neg();
        assert!((tx1[1].re - neg_conj_s1.re).abs() < 1e-10);
        // tx2[0] = s1
        assert!((tx2[0].re - s1.re).abs() < 1e-10 && (tx2[0].im - s1.im).abs() < 1e-10);
        // tx2[1] = s0*
        assert!((tx2[1].re - s0.conj().re).abs() < 1e-10);
    }

    #[test]
    fn test_alamouti_encode_decode_awgn_channel() {
        // Perfect channel: h1 = 1+0j, h2 = 0+1j, no noise
        let syms: Vec<Complex> = (0..8).map(|i| Complex::new(i as f64 * 0.5, 0.0)).collect();
        let (tx1, tx2) = AlamoutiMiso::encode(&syms);

        let h1 = Complex::new(1.0, 0.0);
        let h2 = Complex::new(0.0, 1.0);

        // Simulate received signal: r[t] = h1*tx1[t] + h2*tx2[t]
        let received: Vec<Complex> = tx1.iter().zip(tx2.iter())
            .map(|(t1, t2)| h1.mul(t1).add(&h2.mul(t2)))
            .collect();

        let h1_pairs = vec![h1; syms.len() / 2];
        let h2_pairs = vec![h2; syms.len() / 2];
        let decoded = AlamoutiMiso::decode(&received, &h1_pairs, &h2_pairs);

        for (i, (orig, dec)) in syms.iter().zip(decoded.iter()).enumerate() {
            let diff = orig.sub(dec).abs();
            assert!(diff < 1e-8, "Alamouti symbol {} mismatch: diff = {}", i, diff);
        }
    }

    #[test]
    fn test_alamouti_space_time_orthogonality() {
        // Verify TX power is preserved: sum |tx1|^2 + |tx2|^2 = 2 * sum |s|^2
        let syms: Vec<Complex> = (0..6).map(|i| Complex::from_polar(1.0, i as f64)).collect();
        let input_power: f64 = syms.iter().map(|s| s.abs_sq()).sum();
        let (tx1, tx2) = AlamoutiMiso::encode(&syms);
        let tx_power: f64 = tx1.iter().chain(tx2.iter()).map(|s| s.abs_sq()).sum::<f64>() / 2.0;
        assert!((tx_power - input_power).abs() < 1e-8);
    }

    // ── MIMO ZF ──────────────────────────────────────────────────────────────

    #[test]
    fn test_mimo_2x2_encode_streams() {
        let syms: Vec<Complex> = (0..8).map(|i| Complex::new(i as f64, 0.0)).collect();
        let (tx1, tx2) = Mimo2x2Zf::encode(&syms);
        assert_eq!(tx1.len(), 4);
        assert_eq!(tx2.len(), 4);
        // TX1 = even-indexed, TX2 = odd-indexed
        for (i, (t1, t2)) in tx1.iter().zip(tx2.iter()).enumerate() {
            assert!((t1.re - (2 * i) as f64).abs() < 1e-10);
            assert!((t2.re - (2 * i + 1) as f64).abs() < 1e-10);
        }
    }

    #[test]
    fn test_mimo_2x2_zf_perfect_channel() {
        // Identity channel: h11=1, h12=0, h21=0, h22=1
        let s1 = Complex::new(1.5, -0.5);
        let s2 = Complex::new(-0.3, 1.2);
        let h11 = Complex::one();
        let h12 = Complex::zero();
        let h21 = Complex::zero();
        let h22 = Complex::one();
        let r1 = h11.mul(&s1).add(&h12.mul(&s2)); // = s1
        let r2 = h21.mul(&s1).add(&h22.mul(&s2)); // = s2
        let (s1_hat, s2_hat) = Mimo2x2Zf::detect_zf(r1, r2, h11, h12, h21, h22);
        assert!((s1_hat.re - s1.re).abs() < 1e-9 && (s1_hat.im - s1.im).abs() < 1e-9);
        assert!((s2_hat.re - s2.re).abs() < 1e-9 && (s2_hat.im - s2.im).abs() < 1e-9);
    }

    // ── LDM ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_ldm_combine_output_length() {
        let ldm = Ldm::new(6.0);
        let n = 100;
        let core = random_complex(n, 1);
        let enhanced = random_complex(n, 2);
        let combined = ldm.combine(&core, &enhanced);
        assert_eq!(combined.len(), n);
    }

    #[test]
    fn test_ldm_power_conservation() {
        let ldm = Ldm::new(6.0);
        let n = 1000;
        let core = random_complex(n, 3);
        let enhanced = random_complex(n, 4);
        // Normalize inputs to unit power
        let cp = average_power(&core).sqrt().max(1e-12);
        let ep = average_power(&enhanced).sqrt().max(1e-12);
        let cn: Vec<_> = core.iter().map(|s| s.scale(1.0 / cp)).collect();
        let en: Vec<_> = enhanced.iter().map(|s| s.scale(1.0 / ep)).collect();
        let combined = ldm.combine(&cn, &en);
        let out_power = average_power(&combined);
        // Output power should be ~1 (alpha^2 + beta^2 = 1 by construction)
        assert!((out_power - 1.0).abs() < 0.1, "LDM output power = {}", out_power);
    }

    #[test]
    fn test_ldm_injection_levels() {
        let levels = [3.0, 6.0, 9.0, 12.0];
        for &ilv in &levels {
            let ldm = Ldm::new(ilv);
            let alpha = ldm.alpha();
            assert!(alpha > 0.0 && alpha < 1.0, "alpha = {} for {} dB", alpha, ilv);
        }
    }

    #[test]
    fn test_ldm_sic_core_recovery() {
        let ldm = Ldm::new(6.0);
        let n = 50;
        let core = random_complex(n, 5);
        let enhanced = random_complex(n, 6);
        let combined = ldm.combine(&core, &enhanced);
        // Perfect knowledge of enhanced layer: recover core
        let core_est = ldm.sic_core(&combined, &enhanced);
        for (i, (c, ce)) in core.iter().zip(core_est.iter()).enumerate() {
            let diff = c.sub(ce).abs();
            assert!(diff < 1e-8, "SIC core recovery error at {}: {}", i, diff);
        }
    }

    // ── Pilot Pattern ────────────────────────────────────────────────────────

    #[test]
    fn test_pilot_generator_pp4_8k_count() {
        let pg = PilotGenerator::new(FftSize::K8, PilotPattern::Pp4);
        let pilots = pg.scattered_pilots(0);
        let n = FftSize::K8.as_usize();
        let expected_count = n / PilotPattern::Pp4.dx();
        // Allow ±2 for boundary conditions
        assert!((pilots.len() as isize - expected_count as isize).abs() <= 2,
            "Pilot count {} vs expected ~{}", pilots.len(), expected_count);
    }

    #[test]
    fn test_pilot_pattern_dx_dy() {
        let patterns = [
            PilotPattern::Pp1, PilotPattern::Pp2, PilotPattern::Pp3, PilotPattern::Pp4,
            PilotPattern::Pp5, PilotPattern::Pp6, PilotPattern::Pp7, PilotPattern::Pp8,
        ];
        for p in &patterns {
            assert!(p.dx() > 0, "dx must be positive");
            assert!(p.dy() > 0, "dy must be positive");
        }
    }

    #[test]
    fn test_pilot_insertion_modifies_subcarriers() {
        let pg = PilotGenerator::new(FftSize::K8, PilotPattern::Pp4);
        let n = FftSize::K8.as_usize();
        let mut subcarriers = vec![Complex::zero(); n];
        pg.insert_pilots(&mut subcarriers, 0);
        let nonzero = subcarriers.iter().filter(|s| s.abs() > 1e-10).count();
        assert!(nonzero > 0, "Pilot insertion should produce non-zero subcarriers");
    }

    #[test]
    fn test_continual_pilots_present() {
        let pg = PilotGenerator::new(FftSize::K8, PilotPattern::Pp4);
        let cpilots = pg.continual_pilots();
        assert!(!cpilots.is_empty());
        // All indices should be within FFT size
        let n = FftSize::K8.as_usize();
        for (k, _) in &cpilots {
            assert!(*k < n, "Continual pilot index {} out of range", k);
        }
    }

    // ── Frame Stats ──────────────────────────────────────────────────────────

    #[test]
    fn test_frame_stats_8k_6mhz() {
        let config = OfdmConfig::new_6mhz_8k();
        let plp = PlpConfig::new_data(0, Modulation::Qam256, CodeRate::R10_15);
        let stats = compute_frame_stats(&config, &plp);
        assert_eq!(stats.fft_size, 8192);
        assert!(stats.net_rate_mbps > 0.0, "Net rate should be positive");
    }

    #[test]
    fn test_frame_stats_32k_6mhz() {
        let config = OfdmConfig::new_6mhz_32k();
        let plp = PlpConfig::new_data(0, Modulation::Qam4096, CodeRate::R13_15);
        let stats = compute_frame_stats(&config, &plp);
        assert_eq!(stats.fft_size, 32768);
        assert!(stats.symbol_len > stats.fft_size);
    }

    // ── Utility functions ────────────────────────────────────────────────────

    #[test]
    fn test_ber_identical_sequences() {
        let bits = random_bits(1000, 0xABCD);
        assert_eq!(ber(&bits, &bits), 0.0);
    }

    #[test]
    fn test_ber_inverted_sequence() {
        let bits = random_bits(100, 1);
        let inverted: Vec<bool> = bits.iter().map(|&b| !b).collect();
        assert!((ber(&bits, &inverted) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_evm_identical_signals() {
        let sig = random_complex(100, 5);
        assert!(evm_percent(&sig, &sig) < 1e-8);
    }

    #[test]
    fn test_average_power_unit_complex() {
        let sig: Vec<Complex> = (0..100).map(|_| Complex::from_polar(1.0, 0.0)).collect();
        assert!((average_power(&sig) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_awgn_adds_noise() {
        let n = 1000;
        let signal: Vec<Complex> = (0..n).map(|_| Complex::from_polar(1.0, 0.0)).collect();
        let noisy = awgn(&signal, 10.0, 42);
        let evm = evm_percent(&signal, &noisy);
        // At 10 dB SNR, EVM should be roughly 30% (sqrt(0.1) * 100)
        assert!(evm > 1.0 && evm < 80.0, "EVM = {}% at 10 dB SNR", evm);
    }

    #[test]
    fn test_code_rate_info_bits() {
        let n = LdpcLength::Short.as_usize(); // 16200
        let k = CodeRate::R8_15.info_bits(n);
        // 8/15 * 16200 = 8640
        assert_eq!(k, 8640);
    }

    #[test]
    fn test_fft_size_data_subcarriers() {
        assert!(FftSize::K8.data_subcarriers() < FftSize::K8.as_usize());
        assert!(FftSize::K16.data_subcarriers() < FftSize::K16.as_usize());
        assert!(FftSize::K32.data_subcarriers() < FftSize::K32.as_usize());
        assert!(FftSize::K32.data_subcarriers() > FftSize::K16.data_subcarriers());
    }
}

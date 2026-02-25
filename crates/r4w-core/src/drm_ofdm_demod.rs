//! Digital Radio Mondiale (DRM / DRM+) OFDM Demodulator
//!
//! Implements the physical layer for DRM per ETSI ES 201 980 covering:
//! - Robustness modes A/B/C/D (HF shortwave) and Mode E (DRM+, VHF)
//! - OFDM modulation with guard interval insertion/removal (arbitrary-size DFT)
//! - Pilot structure: frequency/time reference pilots and gain/scattered pilots
//! - QAM constellations: 4-QAM (QPSK), 16-QAM, 64-QAM
//! - Hierarchical modulation with MSP/LSP streams
//! - Convolutional encoding K=7 rate-1/4 mother code with puncturing
//! - Tailbiting convolutional codes for MSC
//! - Cell interleaver (frequency-domain symbol permutation)
//! - Time interleaver (configurable depth: short/long)
//! - Energy dispersal PRBS (x^9 + x^5 + 1)
//! - CRC-8 (FAC) and CRC-16 (SDC) integrity checks
//! - FAC (Fast Access Channel): channel parameters, 64 QPSK cells/frame
//! - SDC (Service Description Channel): multiplex configuration
//! - AAC audio super-frame structure with AU extraction
//!
//! ## Robustness Mode Parameters (ETSI ES 201 980 Table 59)
//!
//! | Mode | FFT | Carriers | Tu (ms) | Tg (ms) | BW (kHz) |
//! |------|-----|----------|---------|---------|----------|
//! |  A   | 576 |   288    |  24.00  |   2.66  |   10     |
//! |  B   | 256 |   206    |  21.33  |   5.33  |   10     |
//! |  C   | 176 |   136    |  14.67  |   5.33  |   10     |
//! |  D   | 128 |    88    |   9.33  |   7.33  |   10     |
//! |  E   |2048 |   213    |  26.66  |   3.33  |  100     |
//!
//! Note: Modes A-D use non-power-of-2 FFT sizes. The DFT is implemented
//! using Bluestein's chirp-Z algorithm to support arbitrary sizes.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::drm_ofdm_demod::{DrmMode, DrmParams, DrmOfdmDemodulator};
//!
//! // Configure for Mode B (most common HF robustness mode)
//! let params = DrmParams::for_mode(DrmMode::B);
//! let demod = DrmOfdmDemodulator::new(params);
//! assert_eq!(demod.params().fft_size, 256);
//! assert_eq!(demod.params().num_carriers, 206);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Core complex number
// ---------------------------------------------------------------------------

/// A 64-bit complex number (real + imaginary).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cx {
    pub re: f64,
    pub im: f64,
}

impl Cx {
    #[inline] pub const fn new(re: f64, im: f64) -> Self { Self { re, im } }
    #[inline] pub fn zero() -> Self { Self { re: 0.0, im: 0.0 } }
    #[inline] pub fn from_polar(r: f64, theta: f64) -> Self {
        Self { re: r * theta.cos(), im: r * theta.sin() }
    }
    #[inline] pub fn mag_sq(&self) -> f64 { self.re * self.re + self.im * self.im }
    #[inline] pub fn mag(&self)    -> f64 { self.mag_sq().sqrt() }
    #[inline] pub fn arg(&self)    -> f64 { self.im.atan2(self.re) }
    #[inline] pub fn conj(&self)   -> Self { Self { re: self.re, im: -self.im } }
    #[inline] pub fn scale(&self, s: f64) -> Self { Self { re: self.re * s, im: self.im * s } }
    #[inline] pub fn add(&self, o: &Self) -> Self { Self { re: self.re + o.re, im: self.im + o.im } }
    #[inline] pub fn sub(&self, o: &Self) -> Self { Self { re: self.re - o.re, im: self.im - o.im } }
    #[inline] pub fn mul(&self, o: &Self) -> Self {
        Self {
            re: self.re * o.re - self.im * o.im,
            im: self.re * o.im + self.im * o.re,
        }
    }
    /// Divide self by other (returns zero on near-zero denominator).
    #[inline] pub fn div(&self, o: &Self) -> Self {
        let d = o.mag_sq();
        if d < 1e-300 { return Self::zero(); }
        self.mul(&o.conj()).scale(1.0 / d)
    }
}

// ---------------------------------------------------------------------------
// Power-of-2 FFT (radix-2 Cooley-Tukey, used internally by Bluestein)
// ---------------------------------------------------------------------------

fn bit_reverse_copy(buf: &mut [Cx]) {
    let n = buf.len();
    let bits = n.trailing_zeros() as usize;
    for i in 0..n {
        let j = reverse_bits(i, bits);
        if i < j { buf.swap(i, j); }
    }
}

fn reverse_bits(mut x: usize, bits: usize) -> usize {
    let mut r = 0usize;
    for _ in 0..bits {
        r = (r << 1) | (x & 1);
        x >>= 1;
    }
    r
}

/// In-place radix-2 DIT FFT. `n` must be a power of two.
pub fn fft_pow2_inplace(buf: &mut [Cx], inverse: bool) {
    let n = buf.len();
    debug_assert!(n.is_power_of_two(), "FFT size must be power of two");
    bit_reverse_copy(buf);
    let sign = if inverse { 1.0_f64 } else { -1.0_f64 };
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = sign * PI / half as f64;
        let (wim, wre) = ang.sin_cos();
        let w_init = Cx::new(wre, wim);
        for i in (0..n).step_by(len) {
            let mut w = Cx::new(1.0, 0.0);
            for j in 0..half {
                let u = buf[i + j];
                let t = buf[i + j + half].mul(&w);
                buf[i + j]        = u.add(&t);
                buf[i + j + half] = u.sub(&t);
                w = w.mul(&w_init);
            }
        }
        len <<= 1;
    }
    if inverse {
        let scale = 1.0 / n as f64;
        for x in buf.iter_mut() { *x = x.scale(scale); }
    }
}

fn pow2_fft(input: &[Cx], inverse: bool) -> Vec<Cx> {
    let mut buf = input.to_vec();
    fft_pow2_inplace(&mut buf, inverse);
    buf
}

// ---------------------------------------------------------------------------
// Bluestein chirp-Z DFT for arbitrary-size sequences
// ---------------------------------------------------------------------------

/// Find smallest power of two >= n.
fn next_pow2(n: usize) -> usize {
    let mut p = 1usize;
    while p < n { p <<= 1; }
    p
}

/// Compute chirp-Z DFT of arbitrary length `n` via Bluestein's algorithm.
///
/// Returns the N-point DFT of `input` for any `n >= 1`.
/// Forward transform uses twiddle factor W = exp(-j*2*pi/N).
/// Inverse transform uses W = exp(+j*2*pi/N) and normalises by 1/N.
pub fn dft_arbitrary(input: &[Cx], inverse: bool) -> Vec<Cx> {
    let n = input.len();
    if n == 0 { return vec![]; }
    if n == 1 { return input.to_vec(); }

    // Special case: power of 2 — use fast path
    if n.is_power_of_two() {
        return pow2_fft(input, inverse);
    }

    let sign = if inverse { 1.0_f64 } else { -1.0_f64 };

    // Chirp sequence: w[k] = exp(+j * pi * k^2 / n)  (Bluestein)
    let chirp: Vec<Cx> = (0..n).map(|k| {
        let phase = sign * PI * (k as f64) * (k as f64) / n as f64;
        Cx::from_polar(1.0, phase)
    }).collect();

    // Bluestein length must be at least 2n-1
    let m = next_pow2(2 * n - 1);

    // a[k] = input[k] * conj(chirp[k])  (multiply by chirp conjugate)
    let mut a = vec![Cx::zero(); m];
    for k in 0..n {
        a[k] = input[k].mul(&chirp[k].conj());
    }

    // b[k] = chirp[k] for k in [0..n] and chirp[m-k] for k in [m-n+1..m]
    let mut b = vec![Cx::zero(); m];
    for k in 0..n {
        b[k] = chirp[k];
        if k > 0 { b[m - k] = chirp[k]; }
    }

    // Convolution via FFT
    let fa = pow2_fft(&a, false);
    let fb = pow2_fft(&b, false);
    let mut fc: Vec<Cx> = fa.iter().zip(fb.iter()).map(|(x, y)| x.mul(y)).collect();
    pow2_fft_inplace_inverse(&mut fc);

    // Extract output: y[k] = fc[k] * conj(chirp[k])
    let mut out: Vec<Cx> = (0..n).map(|k| fc[k].mul(&chirp[k].conj())).collect();

    if inverse {
        let scale = 1.0 / n as f64;
        for x in out.iter_mut() { *x = x.scale(scale); }
    }
    out
}

fn pow2_fft_inplace_inverse(buf: &mut [Cx]) {
    pow2_fft_inplace(buf, true);
}

fn pow2_fft_inplace(buf: &mut [Cx], inverse: bool) {
    fft_pow2_inplace(buf, inverse);
}

/// Forward DFT (any size).
pub fn fft(input: &[Cx]) -> Vec<Cx> { dft_arbitrary(input, false) }

/// Inverse DFT (any size).
pub fn ifft(input: &[Cx]) -> Vec<Cx> { dft_arbitrary(input, true) }

// ---------------------------------------------------------------------------
// DRM Robustness Mode
// ---------------------------------------------------------------------------

/// DRM/DRM+ robustness mode per ETSI ES 201 980 §7.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrmMode {
    /// Mode A: HF, good propagation, FFT=576, carriers=288
    A,
    /// Mode B: HF, typical propagation, FFT=256, carriers=206 (most common)
    B,
    /// Mode C: HF, poor propagation, FFT=176, carriers=136
    C,
    /// Mode D: HF, very poor propagation, FFT=128, carriers=88
    D,
    /// Mode E: VHF (DRM+), FFT=2048, carriers=213, BW=100 kHz
    E,
}

/// Spectrum occupancy (bandwidth) per ETSI ES 201 980 §6.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpecOccupancy {
    /// 4.5 kHz single sideband
    Occ0,
    /// 5 kHz single sideband
    Occ1,
    /// 9 kHz double sideband
    Occ2,
    /// 10 kHz double sideband
    Occ3,
    /// 18 kHz double sideband
    Occ4,
    /// 20 kHz double sideband
    Occ5,
}

/// Interleaver depth selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterleaverDepth {
    /// Short interleaving (~400 ms)
    Short,
    /// Long interleaving (~2 s)
    Long,
}

/// DRM physical-layer parameters derived from robustness mode.
#[derive(Debug, Clone)]
pub struct DrmParams {
    pub mode: DrmMode,
    /// IFFT/FFT size (may be non-power-of-2 for modes A-D)
    pub fft_size: usize,
    /// Number of active OFDM carriers (including pilots)
    pub num_carriers: usize,
    /// Useful symbol duration Tu in seconds
    pub tu_sec: f64,
    /// Guard interval duration Tg in seconds
    pub tg_sec: f64,
    /// Total OFDM symbol duration (Tu + Tg) in seconds
    pub ts_sec: f64,
    /// Number of OFDM symbols per transmission frame
    pub symbols_per_frame: usize,
    /// Number of transmission frames per super-frame
    pub frames_per_superframe: usize,
    /// Sample rate in Hz
    pub sample_rate_hz: f64,
    /// Guard interval length in samples
    pub cp_len: usize,
    /// Number of FAC cells per frame
    pub fac_cells: usize,
    /// Number of SDC cells per super-frame
    pub sdc_cells: usize,
}

impl DrmParams {
    /// Construct parameters for the given robustness mode.
    pub fn for_mode(mode: DrmMode) -> Self {
        match mode {
            DrmMode::A => Self {
                mode,
                fft_size: 576,
                num_carriers: 288,
                tu_sec: 576.0 / 24000.0,
                tg_sec: 64.0 / 24000.0,
                ts_sec: 640.0 / 24000.0,
                symbols_per_frame: 15,
                frames_per_superframe: 3,
                sample_rate_hz: 24000.0,
                cp_len: 64,
                fac_cells: 64,
                sdc_cells: 48,
            },
            DrmMode::B => Self {
                mode,
                fft_size: 256,
                num_carriers: 206,
                tu_sec: 256.0 / 12000.0,
                tg_sec: 64.0 / 12000.0,
                ts_sec: 320.0 / 12000.0,
                symbols_per_frame: 15,
                frames_per_superframe: 3,
                sample_rate_hz: 12000.0,
                cp_len: 64,
                fac_cells: 64,
                sdc_cells: 70,
            },
            DrmMode::C => Self {
                mode,
                fft_size: 176,
                num_carriers: 136,
                tu_sec: 176.0 / 12000.0,
                tg_sec: 64.0 / 12000.0,
                ts_sec: 240.0 / 12000.0,
                symbols_per_frame: 10,
                frames_per_superframe: 4,
                sample_rate_hz: 12000.0,
                cp_len: 64,
                fac_cells: 64,
                sdc_cells: 35,
            },
            DrmMode::D => Self {
                mode,
                fft_size: 128,
                num_carriers: 88,
                tu_sec: 128.0 / 12000.0,
                tg_sec: 88.0 / 12000.0,
                ts_sec: 216.0 / 12000.0,
                symbols_per_frame: 7,
                frames_per_superframe: 5,
                sample_rate_hz: 12000.0,
                cp_len: 88,
                fac_cells: 64,
                sdc_cells: 17,
            },
            DrmMode::E => Self {
                mode,
                fft_size: 2048,
                num_carriers: 213,
                tu_sec: 2048.0 / 192000.0,
                tg_sec: 256.0 / 192000.0,
                ts_sec: 2304.0 / 192000.0,
                symbols_per_frame: 40,
                frames_per_superframe: 4,
                sample_rate_hz: 192000.0,
                cp_len: 256,
                fac_cells: 64,
                sdc_cells: 70,
            },
        }
    }

    /// Total symbol length in samples (FFT size + guard interval).
    pub fn symbol_len(&self) -> usize { self.fft_size + self.cp_len }

    /// Samples per transmission frame.
    pub fn samples_per_frame(&self) -> usize {
        self.symbol_len() * self.symbols_per_frame
    }
}

// ---------------------------------------------------------------------------
// QAM Mapping / Demapping
// ---------------------------------------------------------------------------

/// QAM order supported by DRM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QamOrder {
    Qam4,   // QPSK — 2 bits/symbol
    Qam16,  // 16-QAM — 4 bits/symbol
    Qam64,  // 64-QAM — 6 bits/symbol
}

impl QamOrder {
    /// Bits per symbol for this order.
    pub fn bits_per_symbol(self) -> usize {
        match self {
            Self::Qam4  => 2,
            Self::Qam16 => 4,
            Self::Qam64 => 6,
        }
    }
}

/// DRM standard QAM mapper per ETSI ES 201 980 §8.
///
/// Uses Gray-coded square constellations normalised to unit average power.
pub struct DrmQamMapper {
    order: QamOrder,
}

impl DrmQamMapper {
    pub fn new(order: QamOrder) -> Self { Self { order } }

    /// Map a chunk of bits to a single complex QAM symbol.
    ///
    /// `bits` length must equal `order.bits_per_symbol()`.
    /// Returns unit-average-power symbol.
    pub fn map(&self, bits: &[bool]) -> Cx {
        match self.order {
            QamOrder::Qam4 => {
                // QPSK: Gray coded, 1/sqrt(2) normalisation
                let i = if bits[0] { -1.0_f64 } else { 1.0 };
                let q = if bits[1] { -1.0_f64 } else { 1.0 };
                Cx::new(i / 2.0_f64.sqrt(), q / 2.0_f64.sqrt())
            }
            QamOrder::Qam16 => {
                let (i, q) = gray_16qam_iq(bits[0], bits[1], bits[2], bits[3]);
                let scale = 1.0 / (10.0_f64).sqrt();
                Cx::new(i * scale, q * scale)
            }
            QamOrder::Qam64 => {
                let (i, q) = gray_64qam_iq(bits[0], bits[1], bits[2], bits[3], bits[4], bits[5]);
                let scale = 1.0 / (42.0_f64).sqrt();
                Cx::new(i * scale, q * scale)
            }
        }
    }

    /// Demap a QAM symbol to hard-decision bits.
    pub fn demap_hard(&self, sym: Cx) -> Vec<bool> {
        match self.order {
            QamOrder::Qam4 => {
                vec![sym.re < 0.0, sym.im < 0.0]
            }
            QamOrder::Qam16 => {
                let scale = (10.0_f64).sqrt();
                let i = sym.re * scale;
                let q = sym.im * scale;
                demap_16qam_hard(i, q)
            }
            QamOrder::Qam64 => {
                let scale = (42.0_f64).sqrt();
                let i = sym.re * scale;
                let q = sym.im * scale;
                demap_64qam_hard(i, q)
            }
        }
    }

    /// Compute soft LLRs for each bit position.
    pub fn demap_soft(&self, sym: Cx, noise_var: f64) -> Vec<f64> {
        let n = self.order.bits_per_symbol();
        let points = self.all_points();
        let mut llrs = vec![0.0_f64; n];
        for b in 0..n {
            let mut min0 = f64::MAX;
            let mut min1 = f64::MAX;
            for (bits, pt) in &points {
                let d2 = (sym.re - pt.re).powi(2) + (sym.im - pt.im).powi(2);
                if bits[b] { min1 = min1.min(d2); }
                else       { min0 = min0.min(d2); }
            }
            llrs[b] = (min1 - min0) / (2.0 * noise_var.max(1e-300));
        }
        llrs
    }

    /// Enumerate all (bits, symbol) pairs for this QAM order.
    fn all_points(&self) -> Vec<(Vec<bool>, Cx)> {
        let n = self.order.bits_per_symbol();
        let count = 1usize << n;
        (0..count).map(|idx| {
            let bits: Vec<bool> = (0..n).rev().map(|b| (idx >> b) & 1 == 1).collect();
            let sym = self.map(&bits);
            (bits, sym)
        }).collect()
    }
}

/// Gray-coded 16-QAM I/Q coordinates — returns {±1, ±3}.
///
/// Mapping: b0=sign of I, b1=sign of Q, b2=|I| level, b3=|Q| level.
fn gray_16qam_iq(b0: bool, b1: bool, b2: bool, b3: bool) -> (f64, f64) {
    let i_mag = if b2 { 1.0 } else { 3.0 };
    let q_mag = if b3 { 1.0 } else { 3.0 };
    let i = if b0 { -i_mag } else { i_mag };
    let q = if b1 { -q_mag } else { q_mag };
    (i, q)
}

fn demap_16qam_hard(i: f64, q: f64) -> Vec<bool> {
    let b0 = i < 0.0;
    let b1 = q < 0.0;
    let b2 = i.abs() < 2.0;   // inner: mag=1, outer: mag=3
    let b3 = q.abs() < 2.0;
    vec![b0, b1, b2, b3]
}

/// Gray-coded 64-QAM I/Q coordinates — returns {±1, ±3, ±5, ±7}.
///
/// Mapping: b0=sign of I, b1=sign of Q, (b2,b4)=I-magnitude Gray, (b3,b5)=Q-magnitude Gray.
/// Magnitude table: (b_msb=F,b_lsb=F)->7, (F,T)->5, (T,T)->3, (T,F)->1.
fn gray_64qam_iq(b0: bool, b1: bool, b2: bool, b3: bool, b4: bool, b5: bool) -> (f64, f64) {
    let i_mag: f64 = match (b2, b4) {
        (false, false) => 7.0,
        (false, true)  => 5.0,
        (true,  true)  => 3.0,
        (true,  false) => 1.0,
    };
    let q_mag: f64 = match (b3, b5) {
        (false, false) => 7.0,
        (false, true)  => 5.0,
        (true,  true)  => 3.0,
        (true,  false) => 1.0,
    };
    let i = if b0 { -i_mag } else { i_mag };
    let q = if b1 { -q_mag } else { q_mag };
    (i, q)
}

fn demap_64qam_hard(i: f64, q: f64) -> Vec<bool> {
    let b0 = i < 0.0;
    let b1 = q < 0.0;
    let ai = i.abs();
    let aq = q.abs();
    // b2: outer (mag 5 or 7) vs inner (mag 1 or 3)
    let b2 = ai < 4.0;
    let b3 = aq < 4.0;
    // b4: within the outer group (7 vs 5) or inner group (3 vs 1)
    // outer: b4=F->7, b4=T->5, so b4 = ai < 6 (5 < 6, 7 is not < 6)
    // inner: b4=T->3, b4=F->1, so b4 = ai >= 2 (3 >= 2 = T, 1 >= 2 = F)
    let b4 = if b2 { ai >= 2.0 } else { ai < 6.0 };
    let b5 = if b3 { aq >= 2.0 } else { aq < 6.0 };
    vec![b0, b1, b2, b3, b4, b5]
}

// ---------------------------------------------------------------------------
// Hierarchical Modulation (MSP / LSP)
// ---------------------------------------------------------------------------

/// Hierarchical modulation level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HierLevel {
    /// Most Significant Part — higher protection
    MSP,
    /// Least Significant Part — lower protection
    LSP,
}

/// Hierarchical 64-QAM modulator per ETSI ES 201 980 §8.2.
///
/// MSP occupies the 2 sign bits of I and Q plus inner/outer decision (16 inner points).
/// LSP carries the magnitude refinement bits (fine detail within each quadrant group).
pub struct HierarchicalMapper;

impl HierarchicalMapper {
    /// Map a hierarchical 64-QAM symbol.
    ///
    /// `msp_bits` = [b0, b1, b2, b3] — sign + inner/outer select for I and Q
    /// `lsp_bits` = [b4, b5] — magnitude refinement bits
    pub fn map_hier64(msp_bits: &[bool; 4], lsp_bits: &[bool; 2]) -> Cx {
        let all_bits = [msp_bits[0], msp_bits[1], msp_bits[2], msp_bits[3], lsp_bits[0], lsp_bits[1]];
        let (i, q) = gray_64qam_iq(all_bits[0], all_bits[1], all_bits[2], all_bits[3], all_bits[4], all_bits[5]);
        let scale = 1.0 / (42.0_f64).sqrt();
        Cx::new(i * scale, q * scale)
    }

    /// Extract MSP bits [b0,b1,b2,b3] from a received 64-QAM symbol.
    pub fn extract_msp(sym: Cx) -> [bool; 4] {
        let scale = (42.0_f64).sqrt();
        let i = sym.re * scale;
        let q = sym.im * scale;
        let b = demap_64qam_hard(i, q);
        [b[0], b[1], b[2], b[3]]
    }

    /// Extract LSP bits [b4,b5] from a received 64-QAM symbol.
    pub fn extract_lsp(sym: Cx) -> [bool; 2] {
        let scale = (42.0_f64).sqrt();
        let i = sym.re * scale;
        let q = sym.im * scale;
        let b = demap_64qam_hard(i, q);
        [b[4], b[5]]
    }
}

// ---------------------------------------------------------------------------
// Convolutional Encoder K=7 rate-1/4 (DRM mother code)
// ---------------------------------------------------------------------------

/// DRM convolutional encoder: K=7, rate 1/4 mother code.
///
/// Generators (octal): G0=133, G1=145, G2=171, G3=133 per ETSI ES 201 980 §9.1.
/// The shift register holds K-1=6 bits (64 states).
/// Rate 1/2, 2/3, 3/4, etc. are obtained by puncturing the rate-1/4 output.
pub struct DrmConvEncoder {
    /// Shift register state (6 bits, K-1 = 6)
    state: u8,
    /// Generator polynomials in octal (4 generators → rate 1/4)
    generators: [u8; 4],
}

impl DrmConvEncoder {
    /// Create the default DRM K=7 rate-1/4 mother encoder.
    pub fn new() -> Self {
        Self {
            state: 0,
            generators: [0o133, 0o145, 0o171, 0o133],
        }
    }

    /// Reset shift register to zero.
    pub fn reset(&mut self) { self.state = 0; }

    /// Encode one input bit, returning 4 output bits.
    ///
    /// State update: new_state = (old_state >> 1) | (b << 5)  — 6-bit shift register.
    /// Polynomial evaluation on 7-bit register: reg = state | (b << 6).
    pub fn encode_bit(&mut self, bit: bool) -> [bool; 4] {
        let b = bit as u8;
        // Update 6-bit state: shift right, insert b at MSB position
        self.state = ((self.state >> 1) | (b << 5)) & 0x3F;
        // 7-bit register for polynomial evaluation: b at bit-6, state at bits 5..0
        let reg = self.state | (b << 6);
        let mut out = [false; 4];
        for (k, &g) in self.generators.iter().enumerate() {
            out[k] = (reg & g).count_ones() % 2 == 1;
        }
        out
    }

    /// Encode a sequence of bits (resets state first).
    pub fn encode(&mut self, bits: &[bool]) -> Vec<bool> {
        self.reset();
        let mut out = Vec::with_capacity(bits.len() * 4);
        for &b in bits {
            let coded = self.encode_bit(b);
            out.extend_from_slice(&coded);
        }
        out
    }

    /// Tailbiting encode: pre-load shift register with last K-1 bits of input.
    ///
    /// Used for MSC per DRM spec. The decoder initialises to the same state.
    pub fn encode_tailbiting(&mut self, bits: &[bool]) -> Vec<bool> {
        // Pre-load 6-bit state with last 6 bits of input (in order)
        self.state = 0;
        let k = bits.len();
        let start = if k >= 6 { k - 6 } else { 0 };
        for &b in &bits[start..] {
            let b8 = b as u8;
            self.state = ((self.state >> 1) | (b8 << 5)) & 0x3F;
        }
        // Encode without further reset
        let mut out = Vec::with_capacity(bits.len() * 4);
        for &b in bits {
            let coded = self.encode_bit(b);
            out.extend_from_slice(&coded);
        }
        out
    }
}

impl Default for DrmConvEncoder { fn default() -> Self { Self::new() } }

/// Viterbi decoder for K=7, rate-1/4 DRM mother code (hard-decision).
///
/// Uses 64 states (2^(K-1)), add-compare-select (ACS) with traceback
/// that stores (prev_state, input_bit) pairs for correct path recovery.
pub struct DrmViterbiDecoder {
    generators: [u8; 4],
    num_states: usize,
}

impl DrmViterbiDecoder {
    pub fn new() -> Self {
        Self {
            generators: [0o133, 0o145, 0o171, 0o133],
            num_states: 64,
        }
    }

    /// Compute Hamming branch metric for transitioning from `state` via `bit`.
    fn branch_metric(&self, state: u8, bit: bool, received: &[bool; 4]) -> u32 {
        let b = bit as u8;
        let next_state = ((state >> 1) | (b << 5)) & 0x3F;
        let reg = next_state | (b << 6);
        let mut dist = 0u32;
        for (k, &g) in self.generators.iter().enumerate() {
            let coded = (reg & g).count_ones() % 2 == 1;
            if coded != received[k] { dist += 1; }
        }
        dist
    }

    /// Hard-decision Viterbi decode — returns information bits.
    ///
    /// `received` length must be a multiple of 4 (rate-1/4 coded bits).
    pub fn decode(&self, received: &[bool]) -> Vec<bool> {
        let n_states = self.num_states;
        let n_symbols = received.len() / 4;
        if n_symbols == 0 { return vec![]; }

        let inf_val = u32::MAX / 2;
        let mut pm = vec![inf_val; n_states];
        pm[0] = 0;

        // Traceback stores (previous_state, input_bit) at each time step
        let mut tb: Vec<Vec<(usize, bool)>> = vec![vec![(0usize, false); n_states]; n_symbols];

        for t in 0..n_symbols {
            let base = t * 4;
            let rx: [bool; 4] = [
                received.get(base).copied().unwrap_or(false),
                received.get(base + 1).copied().unwrap_or(false),
                received.get(base + 2).copied().unwrap_or(false),
                received.get(base + 3).copied().unwrap_or(false),
            ];
            let mut new_pm = vec![inf_val; n_states];

            for s in 0..n_states {
                if pm[s] == inf_val { continue; }
                for bit_v in 0u8..2 {
                    let bit = bit_v == 1;
                    let b = bit_v;
                    let nxt = (((s as u8) >> 1) | (b << 5)) as usize & (n_states - 1);
                    let bm = self.branch_metric(s as u8, bit, &rx);
                    let candidate = pm[s].saturating_add(bm);
                    if candidate < new_pm[nxt] {
                        new_pm[nxt] = candidate;
                        tb[t][nxt] = (s, bit);
                    }
                }
            }
            pm = new_pm;
        }

        // Find best final state
        let best_state = pm.iter().enumerate()
            .min_by_key(|&(_, &m)| m)
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Traceback using stored (prev_state, bit) pairs
        let mut bits = vec![false; n_symbols];
        let mut state = best_state;
        for t in (0..n_symbols).rev() {
            let (prev_state, bit) = tb[t][state];
            bits[t] = bit;
            state = prev_state;
        }
        bits
    }
}

impl Default for DrmViterbiDecoder { fn default() -> Self { Self::new() } }

// ---------------------------------------------------------------------------
// Puncturing
// ---------------------------------------------------------------------------

/// Puncturing pattern to achieve higher code rates from the rate-1/4 mother code.
///
/// The pattern is applied cyclically; `true` means "transmit this bit".
#[derive(Debug, Clone)]
pub struct PuncturePattern {
    pub pattern: Vec<bool>,
}

impl PuncturePattern {
    /// Rate 1/2: keep first 2 of each 4-bit group.
    pub fn rate_half() -> Self {
        Self { pattern: vec![true, false, true, false] }
    }
    /// Rate 1/3: keep every 3rd bit.
    pub fn rate_third() -> Self {
        Self { pattern: vec![true, false, false, false, true, false, false, false, true, false, false, false] }
    }
    /// Rate 3/4: puncture pattern for 16-QAM MSC protection level 2.
    pub fn rate_three_quarter() -> Self {
        Self { pattern: vec![true, true, true, false, true, true, true, false] }
    }

    /// Apply puncturing: keep only bits at `true` positions.
    pub fn puncture(&self, coded: &[bool]) -> Vec<bool> {
        coded.iter().enumerate()
            .filter(|(i, _)| self.pattern[i % self.pattern.len()])
            .map(|(_, &b)| b)
            .collect()
    }

    /// Depuncture: insert erasure zeros at punctured positions.
    pub fn depuncture(&self, received: &[bool]) -> Vec<bool> {
        let pat = &self.pattern;
        let ones: usize = pat.iter().filter(|&&b| b).count();
        let out_len = (received.len() * pat.len() + ones - 1) / ones;
        let mut out = Vec::with_capacity(out_len);
        let mut rx_idx = 0usize;
        let mut p_idx = 0usize;
        while out.len() < out_len {
            if pat[p_idx % pat.len()] {
                out.push(if rx_idx < received.len() { received[rx_idx] } else { false });
                rx_idx += 1;
            } else {
                out.push(false); // erasure / zero insertion
            }
            p_idx += 1;
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Energy Dispersal (PRBS Scrambling)
// ---------------------------------------------------------------------------

/// PRBS scrambler using polynomial x^9 + x^5 + 1 per ETSI ES 201 980 §10.
///
/// The same sequence is XORed for scrambling and de-scrambling.
pub struct EnergyDispersal {
    state: u16, // 9-bit LFSR (bits 0-8)
}

impl EnergyDispersal {
    /// Create PRBS with initial state 0x1FF (all ones) per spec.
    pub fn new() -> Self { Self { state: 0x1FF } }

    /// Reset to initial state.
    pub fn reset(&mut self) { self.state = 0x1FF; }

    /// Generate next pseudo-random bit.
    pub fn next_bit(&mut self) -> bool {
        // x^9 + x^5 + 1: feedback = bit8 XOR bit4
        let bit8 = (self.state >> 8) & 1;
        let bit4 = (self.state >> 4) & 1;
        let fb = (bit8 ^ bit4) as u16;
        let out_bit = bit8 == 1;
        self.state = ((self.state << 1) | fb) & 0x1FF;
        out_bit
    }

    /// Scramble (or de-scramble) a byte vector in-place.
    pub fn process_bytes(&mut self, data: &mut [u8]) {
        for byte in data.iter_mut() {
            let mut prbs_byte = 0u8;
            for bit in 0..8 {
                if self.next_bit() { prbs_byte |= 1 << (7 - bit); }
            }
            *byte ^= prbs_byte;
        }
    }

    /// Scramble a bit vector in-place.
    pub fn process_bits(&mut self, bits: &mut [bool]) {
        for b in bits.iter_mut() {
            *b ^= self.next_bit();
        }
    }
}

impl Default for EnergyDispersal { fn default() -> Self { Self::new() } }

// ---------------------------------------------------------------------------
// CRC-8 (FAC) and CRC-16 (SDC)
// ---------------------------------------------------------------------------

/// CRC-8 per ETSI ES 201 980: polynomial x^8 + x^4 + x^3 + x^2 + 1 (0x1D).
pub fn crc8_drm(data: &[u8]) -> u8 {
    const POLY: u8 = 0x1D;
    let mut crc = 0xFFu8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if crc & 0x80 != 0 { crc = (crc << 1) ^ POLY; }
            else { crc <<= 1; }
        }
    }
    crc  // Note: no final inversion for this DRM variant
}

/// CRC-16 CCITT per ETSI ES 201 980: polynomial x^16 + x^12 + x^5 + 1 (0x1021).
pub fn crc16_drm(data: &[u8]) -> u16 {
    const POLY: u16 = 0x1021;
    let mut crc = 0xFFFFu16;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 { crc = (crc << 1) ^ POLY; }
            else { crc <<= 1; }
        }
    }
    crc  // No final inversion
}

// ---------------------------------------------------------------------------
// Pilot Structure
// ---------------------------------------------------------------------------

/// Type of DRM pilot cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PilotType {
    /// Frequency reference pilot (boosted, fixed BPSK at symbol 0)
    FreqRef,
    /// Time reference pilot (boosted, known sequence at carrier 0)
    TimeRef,
    /// Gain reference pilot (scattered, for channel equalization)
    GainRef,
    /// Scattered pilot (channel estimation across time/frequency)
    Scattered,
}

/// Pilot cell descriptor.
#[derive(Debug, Clone)]
pub struct PilotCell {
    pub carrier_idx: usize,
    pub symbol_idx: usize,
    pub pilot_type: PilotType,
    /// Known transmitted value (used by channel estimator)
    pub value: Cx,
}

/// Generate pilot pattern for a given DRM mode and symbol index.
///
/// Returns a list of pilot cells for one OFDM symbol.
/// Based on ETSI ES 201 980 §8.4 pilot patterns.
pub fn generate_pilots(mode: DrmMode, symbol_idx: usize) -> Vec<PilotCell> {
    let mut pilots = Vec::new();
    let boost = 10.0_f64.powf(4.0 / 20.0); // +4 dB amplitude boost

    let nc = match mode {
        DrmMode::A => 288,
        DrmMode::B => 206,
        DrmMode::C => 136,
        DrmMode::D => 88,
        DrmMode::E => 213,
    };

    match mode {
        DrmMode::A | DrmMode::B => {
            // Frequency reference pilots at symbol 0, every 4th carrier
            if symbol_idx == 0 {
                for k in (0..nc.min(32)).step_by(4) {
                    pilots.push(PilotCell {
                        carrier_idx: k,
                        symbol_idx,
                        pilot_type: PilotType::FreqRef,
                        value: Cx::new(boost, 0.0),
                    });
                }
            }
            // Time reference pilot at carrier 0 every symbol
            pilots.push(PilotCell {
                carrier_idx: 0,
                symbol_idx,
                pilot_type: PilotType::TimeRef,
                value: Cx::from_polar(1.0, PI * (symbol_idx as f64) / 4.0),
            });
            // Scattered gain reference pilots (every 4th carrier, offset by symbol)
            let offset = symbol_idx % 4;
            for k in (offset..nc).step_by(4) {
                if k == 0 { continue; } // skip, already added as time ref
                pilots.push(PilotCell {
                    carrier_idx: k,
                    symbol_idx,
                    pilot_type: PilotType::GainRef,
                    value: Cx::from_polar(boost, 2.0 * PI * (k as f64 * symbol_idx as f64) / nc as f64),
                });
            }
        }
        DrmMode::C | DrmMode::D => {
            // Scattered every 3rd carrier
            let offset = symbol_idx % 3;
            for k in (offset..nc).step_by(3) {
                pilots.push(PilotCell {
                    carrier_idx: k,
                    symbol_idx,
                    pilot_type: PilotType::Scattered,
                    value: Cx::from_polar(boost, PI * k as f64 / nc as f64),
                });
            }
        }
        DrmMode::E => {
            // DRM+ Mode E: fewer scattered pilots
            let offset = symbol_idx % 6;
            for k in (offset..nc).step_by(6) {
                pilots.push(PilotCell {
                    carrier_idx: k,
                    symbol_idx,
                    pilot_type: PilotType::Scattered,
                    value: Cx::from_polar(1.0, 2.0 * PI * k as f64 / nc as f64),
                });
            }
        }
    }
    pilots
}

// ---------------------------------------------------------------------------
// Cell Interleaver
// ---------------------------------------------------------------------------

/// Cell-level interleaver per ETSI ES 201 980 §9.3.
///
/// Provides frequency diversity by permuting cell indices using a
/// step-coprime mapping.
pub struct CellInterleaver {
    perm: Vec<usize>,
    inv_perm: Vec<usize>,
}

impl CellInterleaver {
    /// Build interleaver for `n_cells` cells.
    pub fn new(n_cells: usize) -> Self {
        let step = Self::find_coprime_step(n_cells);
        let mut perm = vec![0usize; n_cells];
        let mut pos = 0usize;
        for idx in 0..n_cells {
            perm[idx] = pos;
            pos = (pos + step) % n_cells;
        }
        let mut inv_perm = vec![0usize; n_cells];
        for (i, &p) in perm.iter().enumerate() {
            inv_perm[p] = i;
        }
        Self { perm, inv_perm }
    }

    fn gcd(a: usize, b: usize) -> usize {
        if b == 0 { a } else { Self::gcd(b, a % b) }
    }

    fn find_coprime_step(n: usize) -> usize {
        if n <= 1 { return 1; }
        let phi = (5.0_f64.sqrt() - 1.0) / 2.0;
        let candidate = ((n as f64 * phi) as usize) | 1;
        for k in 0..n {
            let s = (candidate + k) % n;
            if s > 0 && Self::gcd(s, n) == 1 { return s; }
        }
        1
    }

    /// Interleave cells (scatter across carriers).
    pub fn interleave(&self, cells: &[Cx]) -> Vec<Cx> {
        assert_eq!(cells.len(), self.perm.len());
        let mut out = vec![Cx::zero(); cells.len()];
        for (i, &p) in self.perm.iter().enumerate() {
            out[p] = cells[i];
        }
        out
    }

    /// De-interleave: apply inverse permutation.
    pub fn deinterleave(&self, cells: &[Cx]) -> Vec<Cx> {
        assert_eq!(cells.len(), self.inv_perm.len());
        let mut out = vec![Cx::zero(); cells.len()];
        for (i, &p) in self.inv_perm.iter().enumerate() {
            out[p] = cells[i];
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Time Interleaver
// ---------------------------------------------------------------------------

/// Time interleaver for DRM MSC per ETSI ES 201 980 §9.4.
///
/// Distributes cell errors across multiple OFDM symbols in time
/// to combat burst fading.
pub struct TimeInterleaver {
    depth: usize,
    n_cells: usize,
    buf: Vec<Vec<Cx>>,
    write_idx: usize,
}

impl TimeInterleaver {
    /// Create a time interleaver.
    ///
    /// * `depth`   — number of time frames to interleave across
    /// * `n_cells` — number of cells per frame
    pub fn new(depth: usize, n_cells: usize) -> Self {
        let d = depth.max(1);
        Self {
            depth: d,
            n_cells,
            buf: vec![vec![Cx::zero(); n_cells]; d],
            write_idx: 0,
        }
    }

    /// Push a new frame's cells, return spread output cells.
    pub fn process_frame(&mut self, cells: &[Cx]) -> Vec<Cx> {
        assert_eq!(cells.len(), self.n_cells);
        self.buf[self.write_idx] = cells.to_vec();
        let mut out = Vec::with_capacity(self.n_cells);
        for i in 0..self.n_cells {
            let delay = (i * self.depth / self.n_cells.max(1)) % self.depth;
            let read_idx = (self.write_idx + self.depth - delay) % self.depth;
            out.push(self.buf[read_idx][i]);
        }
        self.write_idx = (self.write_idx + 1) % self.depth;
        out
    }
}

// ---------------------------------------------------------------------------
// OFDM Modulator
// ---------------------------------------------------------------------------

/// DRM OFDM modulator: maps frequency-domain cells to time-domain samples.
///
/// Uses Bluestein DFT to support arbitrary (non-power-of-2) FFT sizes.
pub struct DrmOfdmModulator {
    params: DrmParams,
}

impl DrmOfdmModulator {
    pub fn new(params: DrmParams) -> Self { Self { params } }

    /// Modulate one OFDM symbol.
    ///
    /// `cells` must contain at most `num_carriers` complex values.
    /// Guard interval is prepended. Returns time-domain samples.
    pub fn modulate_symbol(&self, cells: &[Cx]) -> Vec<Cx> {
        let n  = self.params.fft_size;
        let nc = self.params.num_carriers;
        let cp = self.params.cp_len;
        assert!(cells.len() <= nc, "Too many input cells ({} > {})", cells.len(), nc);

        // Map carriers into IFFT input vector
        // Carriers are placed symmetrically around DC:
        //   lower half at bins [n-half .. n-1], upper half at bins [1 .. upper_half]
        let mut freq_domain = vec![Cx::zero(); n];
        let half = nc / 2;
        for (i, &c) in cells.iter().enumerate() {
            if i < half {
                // Lower sideband: negative frequencies at top of FFT
                freq_domain[n - half + i] = c;
            } else {
                // Upper sideband: positive frequencies at bottom of FFT (skip DC=0)
                freq_domain[1 + (i - half)] = c;
            }
        }

        // IFFT (any size via Bluestein)
        let mut time_domain = ifft(&freq_domain);

        // Prepend cyclic prefix (copy last cp samples)
        let mut out = Vec::with_capacity(n + cp);
        out.extend_from_slice(&time_domain[n - cp..]);
        out.append(&mut time_domain);
        out
    }

    /// Modulate a full frame of OFDM symbols.
    pub fn modulate_frame(&self, symbols: &[Vec<Cx>]) -> Vec<Cx> {
        let mut out = Vec::new();
        for sym_cells in symbols {
            out.extend(self.modulate_symbol(sym_cells));
        }
        out
    }
}

// ---------------------------------------------------------------------------
// OFDM Demodulator
// ---------------------------------------------------------------------------

/// DRM OFDM demodulator: extracts frequency-domain cells from time-domain samples.
pub struct DrmOfdmDemodulator {
    params: DrmParams,
}

impl DrmOfdmDemodulator {
    pub fn new(params: DrmParams) -> Self { Self { params } }

    /// Access a reference to the parameters.
    pub fn params(&self) -> &DrmParams { &self.params }

    /// Demodulate one OFDM symbol from samples.
    ///
    /// `samples` must be at least `symbol_len()` samples long.
    /// Returns `num_carriers` complex frequency-domain cells.
    pub fn demodulate_symbol(&self, samples: &[Cx]) -> Vec<Cx> {
        let n  = self.params.fft_size;
        let cp = self.params.cp_len;
        let nc = self.params.num_carriers;
        let sym_len = n + cp;
        assert!(samples.len() >= sym_len, "Insufficient samples: {} < {}", samples.len(), sym_len);

        // Strip guard interval
        let ofdm_samples = &samples[cp..cp + n];

        // FFT (arbitrary size via Bluestein)
        let spectrum = fft(ofdm_samples);

        // Extract active carriers (inverse of modulator placement)
        let half = nc / 2;
        let mut cells = Vec::with_capacity(nc);
        // Lower sideband: bins [n-half .. n-1]
        for i in 0..half {
            cells.push(spectrum[n - half + i]);
        }
        // Upper sideband: bins [1 .. 1+upper_half-1]
        for i in half..nc {
            cells.push(spectrum[1 + (i - half)]);
        }
        cells
    }

    /// Demodulate a full frame.
    pub fn demodulate_frame(&self, samples: &[Cx]) -> Vec<Vec<Cx>> {
        let sym_len = self.params.symbol_len();
        let n_syms  = self.params.symbols_per_frame;
        assert!(samples.len() >= sym_len * n_syms);

        let mut symbols = Vec::with_capacity(n_syms);
        for s in 0..n_syms {
            let start = s * sym_len;
            symbols.push(self.demodulate_symbol(&samples[start..start + sym_len]));
        }
        symbols
    }
}

// ---------------------------------------------------------------------------
// FAC (Fast Access Channel)
// ---------------------------------------------------------------------------

/// FAC parameters extracted from a decoded FAC block.
///
/// FAC carries 64 QPSK cells per transmission frame per ETSI ES 201 980 §6.3.
#[derive(Debug, Clone, Default)]
pub struct FacParameters {
    /// Robustness mode (0=A, 1=B, 2=C, 3=D)
    pub mode: u8,
    /// Spectrum occupancy (0-5)
    pub spectrum_occupancy: u8,
    /// Interleaver depth (0=short, 1=long)
    pub interleaver_depth: u8,
    /// SDC mode (0=4-QAM, 1=16-QAM)
    pub sdc_mode: u8,
    /// Number of audio/data services
    pub num_services: u8,
    /// Programme type
    pub programme_type: u8,
    /// CRC-8 value stored in FAC
    pub crc8: u8,
    /// Whether CRC check passed
    pub crc_ok: bool,
}

/// Encode FAC parameters into a byte payload with CRC-8.
pub fn fac_encode(params: &FacParameters) -> Vec<u8> {
    let mut out = vec![0u8; 8];
    out[0] = (params.mode & 0x03)
           | ((params.spectrum_occupancy & 0x07) << 2)
           | ((params.interleaver_depth & 0x01) << 5);
    out[1] = (params.sdc_mode & 0x01)
           | ((params.num_services & 0x07) << 1)
           | ((params.programme_type & 0x1F) << 4);
    // Remaining bytes reserved (zero)
    let crc = crc8_drm(&out[..7]);
    out[7] = crc;
    out
}

/// Decode FAC payload bytes.
pub fn fac_decode(data: &[u8]) -> FacParameters {
    if data.len() < 8 {
        return FacParameters::default();
    }
    let stored_crc = data[7];
    let computed_crc = crc8_drm(&data[..7]);
    FacParameters {
        mode:               data[0] & 0x03,
        spectrum_occupancy: (data[0] >> 2) & 0x07,
        interleaver_depth:  (data[0] >> 5) & 0x01,
        sdc_mode:           data[1] & 0x01,
        num_services:       (data[1] >> 1) & 0x07,
        programme_type:     (data[1] >> 4) & 0x1F,
        crc8: stored_crc,
        crc_ok: stored_crc == computed_crc,
    }
}

// ---------------------------------------------------------------------------
// SDC (Service Description Channel)
// ---------------------------------------------------------------------------

/// SDC entity type per ETSI ES 201 980 §6.4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdcEntityType {
    MultiplexDescription = 0,
    Label               = 1,
    AudioInfo           = 2,
    DataInfo            = 3,
    TimeDate            = 4,
    AltFrequency        = 5,
}

/// SDC entity (variable-length data block within the SDC).
#[derive(Debug, Clone)]
pub struct SdcEntity {
    pub entity_type: SdcEntityType,
    pub length: usize,
    pub payload: Vec<u8>,
}

/// SDC block container.
#[derive(Debug, Clone)]
pub struct SdcBlock {
    pub entities: Vec<SdcEntity>,
    pub crc_ok: bool,
}

/// Encode an SDC block with CRC-16.
pub fn sdc_encode(entities: &[SdcEntity]) -> Vec<u8> {
    let mut body = Vec::new();
    for e in entities {
        body.push(((e.entity_type as u8) & 0x0F) | ((e.length.min(127) as u8) << 1));
        body.extend_from_slice(&e.payload[..e.length.min(e.payload.len())]);
    }
    let crc = crc16_drm(&body);
    body.push((crc >> 8) as u8);
    body.push((crc & 0xFF) as u8);
    body
}

/// Decode an SDC block.
pub fn sdc_decode(data: &[u8]) -> SdcBlock {
    if data.len() < 2 {
        return SdcBlock { entities: vec![], crc_ok: false };
    }
    let body = &data[..data.len() - 2];
    let stored_crc = ((data[data.len() - 2] as u16) << 8) | data[data.len() - 1] as u16;
    let computed_crc = crc16_drm(body);
    let mut entities = Vec::new();
    let mut i = 0;
    while i + 1 <= body.len() {
        let header = body[i];
        let etype_raw = header & 0x0F;
        let length = ((header >> 1) & 0x7F) as usize;
        let etype = match etype_raw {
            0 => SdcEntityType::MultiplexDescription,
            1 => SdcEntityType::Label,
            2 => SdcEntityType::AudioInfo,
            3 => SdcEntityType::DataInfo,
            4 => SdcEntityType::TimeDate,
            5 => SdcEntityType::AltFrequency,
            _ => SdcEntityType::MultiplexDescription,
        };
        let payload_end = (i + 1 + length).min(body.len());
        let payload = body[i + 1..payload_end].to_vec();
        entities.push(SdcEntity { entity_type: etype, length, payload });
        i = payload_end;
        if length == 0 { break; }
    }
    SdcBlock { entities, crc_ok: stored_crc == computed_crc }
}

// ---------------------------------------------------------------------------
// MSC Protection Level
// ---------------------------------------------------------------------------

/// MSC protection level per ETSI ES 201 980 §9.2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MscProtectionLevel {
    /// Code rate ~3/8 (strongest protection)
    Level0,
    /// Code rate ~1/2
    Level1,
    /// Code rate ~5/8
    Level2,
    /// Code rate ~3/4 (weakest protection)
    Level3,
}

impl MscProtectionLevel {
    /// Returns the nominal code rate as (numerator, denominator).
    pub fn code_rate(self) -> (u32, u32) {
        match self {
            Self::Level0 => (3, 8),
            Self::Level1 => (1, 2),
            Self::Level2 => (5, 8),
            Self::Level3 => (3, 4),
        }
    }

    /// Returns the required QAM order for this protection level.
    pub fn required_qam(self) -> QamOrder {
        match self {
            Self::Level0 | Self::Level1 => QamOrder::Qam4,
            Self::Level2 => QamOrder::Qam16,
            Self::Level3 => QamOrder::Qam64,
        }
    }
}

// ---------------------------------------------------------------------------
// Audio Super-Frame (AAC/xHE-AAC)
// ---------------------------------------------------------------------------

/// AAC audio super-frame structure per ETSI ES 201 980 §8.
///
/// Contains one or more Access Units (AUs) with 2-byte CRC protection.
#[derive(Debug, Clone)]
pub struct AudioSuperFrame {
    /// Number of Audio Units in this super-frame
    pub num_aus: usize,
    /// Start byte offsets of each AU within the payload
    pub au_start: Vec<usize>,
    /// AU payload bytes (between header and CRC)
    pub payload: Vec<u8>,
    /// Whether the super-frame CRC-16 passed
    pub crc_ok: bool,
}

impl AudioSuperFrame {
    /// Parse an audio super-frame from raw bytes.
    ///
    /// Header format: 1 byte containing (num_aus-1) in top nibble,
    /// followed by `num_aus` × 2-byte AU start offsets, then payload,
    /// then 2-byte CRC-16.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 4 { return None; }
        let num_aus = ((data[0] >> 4) & 0x0F) as usize + 1;
        let header_len = 1 + num_aus * 2;
        if data.len() < header_len + 2 { return None; }

        let mut au_start = Vec::with_capacity(num_aus);
        for i in 0..num_aus {
            let offset = 1 + i * 2;
            let start = ((data[offset] as usize) << 8) | data[offset + 1] as usize;
            au_start.push(start);
        }

        let crc_stored = ((data[data.len() - 2] as u16) << 8) | data[data.len() - 1] as u16;
        let crc_computed = crc16_drm(&data[..data.len() - 2]);

        Some(Self {
            num_aus,
            au_start,
            payload: data[header_len..data.len() - 2].to_vec(),
            crc_ok: crc_stored == crc_computed,
        })
    }

    /// Extract the n-th Access Unit bytes from the payload.
    pub fn extract_au(&self, au_idx: usize) -> Option<&[u8]> {
        if au_idx >= self.num_aus { return None; }
        let start = self.au_start[au_idx];
        let end = if au_idx + 1 < self.num_aus {
            self.au_start[au_idx + 1]
        } else {
            self.payload.len()
        };
        let start = start.min(self.payload.len());
        let end   = end.min(self.payload.len());
        if start > end { return None; }
        Some(&self.payload[start..end])
    }
}

// ---------------------------------------------------------------------------
// Channel Estimator (Pilot-aided ZF)
// ---------------------------------------------------------------------------

/// Pilot-aided channel estimator using zero-forcing equalization.
///
/// Estimates channel response H[k] at pilot positions from known pilot values,
/// then linearly interpolates to data subcarriers.
pub struct DrmChannelEstimator {
    n_carriers: usize,
}

impl DrmChannelEstimator {
    pub fn new(n_carriers: usize) -> Self { Self { n_carriers } }

    /// Estimate channel from pilot cells.
    ///
    /// `received_symbol[k]` = H[k] * known_pilot[k] at pilot positions.
    /// Returns channel estimate H[k] for every carrier k.
    pub fn estimate(
        &self,
        received_symbol: &[Cx],
        pilots: &[PilotCell],
    ) -> Vec<Cx> {
        let n = self.n_carriers;
        let mut h_est = vec![Cx::new(1.0, 0.0); n];

        // At each pilot: H[k] = received[k] / known_pilot[k]
        let mut pilot_positions: Vec<(usize, Cx)> = pilots.iter()
            .filter(|p| p.carrier_idx < n && p.carrier_idx < received_symbol.len())
            .map(|p| {
                let h = received_symbol[p.carrier_idx].div(&p.value);
                (p.carrier_idx, h)
            })
            .collect();

        if pilot_positions.is_empty() { return h_est; }

        pilot_positions.sort_by_key(|&(k, _)| k);

        // Set H at pilot positions
        for &(k, h) in &pilot_positions {
            h_est[k] = h;
        }

        // Linear interpolation between adjacent pilot positions
        for i in 0..pilot_positions.len().saturating_sub(1) {
            let (k0, h0) = pilot_positions[i];
            let (k1, h1) = pilot_positions[i + 1];
            for k in k0 + 1..k1 {
                let alpha = (k - k0) as f64 / (k1 - k0) as f64;
                h_est[k] = Cx::new(
                    h0.re + alpha * (h1.re - h0.re),
                    h0.im + alpha * (h1.im - h0.im),
                );
            }
        }

        // Extrapolate edges (hold nearest pilot value)
        if let Some(&(k_first, h_first)) = pilot_positions.first() {
            for k in 0..k_first { h_est[k] = h_first; }
        }
        if let Some(&(k_last, h_last)) = pilot_positions.last() {
            for k in (k_last + 1)..n { h_est[k] = h_last; }
        }

        h_est
    }

    /// Apply zero-forcing equalization: y[k] = received[k] / H[k].
    pub fn equalize(&self, cells: &[Cx], h_est: &[Cx]) -> Vec<Cx> {
        cells.iter().zip(h_est.iter())
            .map(|(x, h)| x.div(h))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Frame Timing
// ---------------------------------------------------------------------------

/// DRM frame timing parameters computed from mode.
#[derive(Debug, Clone)]
pub struct FrameTiming {
    /// Transmission frame duration in seconds
    pub frame_duration_sec: f64,
    /// Super-frame duration in seconds
    pub superframe_duration_sec: f64,
    /// Symbol rate (symbols/second)
    pub symbol_rate: f64,
    /// Guard-to-useful-time ratio (Tg/Tu)
    pub guard_ratio: f64,
}

impl FrameTiming {
    pub fn for_mode(params: &DrmParams) -> Self {
        let frame_duration_sec = params.ts_sec * params.symbols_per_frame as f64;
        let superframe_duration_sec = frame_duration_sec * params.frames_per_superframe as f64;
        let symbol_rate = 1.0 / params.ts_sec;
        let guard_ratio = params.tg_sec / params.tu_sec;
        Self { frame_duration_sec, superframe_duration_sec, symbol_rate, guard_ratio }
    }
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64     = 1e-9;
    const EPS_LOW: f64 = 1e-5;

    // -----------------------------------------------------------------------
    // Complex arithmetic
    // -----------------------------------------------------------------------

    #[test]
    fn test_cx_arithmetic() {
        let a = Cx::new(3.0, 4.0);
        let b = Cx::new(1.0, -2.0);
        let s = a.add(&b);
        assert!((s.re - 4.0).abs() < EPS);
        assert!((s.im - 2.0).abs() < EPS);
        // (3+4j)(1-2j) = 3-6j+4j+8 = 11-2j
        let m = a.mul(&b);
        assert!((m.re - 11.0).abs() < EPS);
        assert!((m.im - (-2.0)).abs() < EPS);
        assert!((a.mag() - 5.0).abs() < EPS);
        assert!((a.conj().im - (-4.0)).abs() < EPS);
    }

    #[test]
    fn test_cx_polar() {
        let c = Cx::from_polar(1.0, PI / 2.0);
        assert!(c.re.abs() < EPS_LOW);
        assert!((c.im - 1.0).abs() < EPS_LOW);
        assert!((c.arg() - PI / 2.0).abs() < EPS_LOW);
    }

    #[test]
    fn test_cx_div() {
        let a = Cx::new(6.0, 0.0);
        let b = Cx::new(2.0, 0.0);
        let r = a.div(&b);
        assert!((r.re - 3.0).abs() < EPS);
        // Division by zero returns zero
        let z = Cx::zero();
        let r2 = a.div(&z);
        assert_eq!(r2.re, 0.0);
    }

    #[test]
    fn test_cx_sub_and_scale() {
        let a = Cx::new(5.0, 3.0);
        let b = Cx::new(2.0, 1.0);
        let s = a.sub(&b);
        assert!((s.re - 3.0).abs() < EPS);
        assert!((s.im - 2.0).abs() < EPS);
        let sc = a.scale(2.0);
        assert!((sc.re - 10.0).abs() < EPS);
    }

    // -----------------------------------------------------------------------
    // FFT / IFFT roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_ifft_roundtrip_pow2_size8() {
        let n = 8;
        let input: Vec<Cx> = (0..n).map(|i| Cx::new(i as f64, 0.0)).collect();
        let reconstructed = ifft(&fft(&input));
        for (a, b) in input.iter().zip(reconstructed.iter()) {
            assert!((a.re - b.re).abs() < EPS_LOW);
            assert!((a.im - b.im).abs() < EPS_LOW);
        }
    }

    #[test]
    fn test_fft_ifft_roundtrip_pow2_size64() {
        let n = 64;
        let input: Vec<Cx> = (0..n).map(|i| Cx::from_polar(1.0, 2.0 * PI * i as f64 / n as f64)).collect();
        let reconstructed = ifft(&fft(&input));
        for (a, b) in input.iter().zip(reconstructed.iter()) {
            assert!((a.re - b.re).abs() < EPS_LOW);
        }
    }

    #[test]
    fn test_fft_ifft_roundtrip_arbitrary_size_9() {
        let input: Vec<Cx> = (0..9).map(|i| Cx::new(i as f64 + 1.0, 0.0)).collect();
        let reconstructed = ifft(&fft(&input));
        for (a, b) in input.iter().zip(reconstructed.iter()) {
            assert!((a.re - b.re).abs() < EPS_LOW, "re: {} vs {}", a.re, b.re);
        }
    }

    #[test]
    fn test_fft_ifft_roundtrip_arbitrary_size_176() {
        // Mode C FFT size
        let n = 176;
        let input: Vec<Cx> = (0..n).map(|i| Cx::from_polar(1.0, 2.0 * PI * i as f64 / n as f64)).collect();
        let reconstructed = ifft(&fft(&input));
        for (a, b) in input.iter().zip(reconstructed.iter()) {
            assert!((a.re - b.re).abs() < EPS_LOW, "mismatch at element");
        }
    }

    #[test]
    fn test_fft_dc_signal() {
        let n = 8;
        let input = vec![Cx::new(1.0, 0.0); n];
        let out = fft(&input);
        assert!((out[0].re - n as f64).abs() < EPS_LOW);
        for k in 1..n {
            assert!(out[k].mag() < EPS_LOW, "bin {} should be zero, got {}", k, out[k].mag());
        }
    }

    #[test]
    fn test_fft_single_tone() {
        let n = 16;
        let freq_bin = 3;
        let input: Vec<Cx> = (0..n).map(|t| {
            Cx::from_polar(1.0, 2.0 * PI * freq_bin as f64 * t as f64 / n as f64)
        }).collect();
        let out = fft(&input);
        assert!((out[freq_bin].mag() - n as f64).abs() < EPS_LOW);
        for k in 0..n {
            if k != freq_bin {
                assert!(out[k].mag() < EPS_LOW, "spurious at bin {}", k);
            }
        }
    }

    // -----------------------------------------------------------------------
    // DRM Mode Parameters
    // -----------------------------------------------------------------------

    #[test]
    fn test_drm_mode_a_params() {
        let p = DrmParams::for_mode(DrmMode::A);
        assert_eq!(p.fft_size, 576);
        assert_eq!(p.num_carriers, 288);
        assert_eq!(p.symbols_per_frame, 15);
        assert_eq!(p.fac_cells, 64);
    }

    #[test]
    fn test_drm_mode_b_params() {
        let p = DrmParams::for_mode(DrmMode::B);
        assert_eq!(p.fft_size, 256);
        assert_eq!(p.num_carriers, 206);
        assert_eq!(p.cp_len, 64);
        assert_eq!(p.frames_per_superframe, 3);
    }

    #[test]
    fn test_drm_mode_c_params() {
        let p = DrmParams::for_mode(DrmMode::C);
        assert_eq!(p.fft_size, 176);
        assert_eq!(p.num_carriers, 136);
        assert_eq!(p.symbols_per_frame, 10);
        assert_eq!(p.frames_per_superframe, 4);
    }

    #[test]
    fn test_drm_mode_d_params() {
        let p = DrmParams::for_mode(DrmMode::D);
        assert_eq!(p.fft_size, 128);
        assert_eq!(p.num_carriers, 88);
        assert_eq!(p.symbols_per_frame, 7);
        assert_eq!(p.cp_len, 88);
    }

    #[test]
    fn test_drm_mode_e_params() {
        let p = DrmParams::for_mode(DrmMode::E);
        assert_eq!(p.fft_size, 2048);
        assert_eq!(p.num_carriers, 213);
        assert_eq!(p.symbols_per_frame, 40);
        assert_eq!(p.frames_per_superframe, 4);
        assert!((p.sample_rate_hz - 192000.0).abs() < 1.0);
    }

    #[test]
    fn test_symbol_len() {
        let p = DrmParams::for_mode(DrmMode::B);
        assert_eq!(p.symbol_len(), 256 + 64);
    }

    #[test]
    fn test_frame_timing_mode_a() {
        let p = DrmParams::for_mode(DrmMode::A);
        let t = FrameTiming::for_mode(&p);
        assert!(t.frame_duration_sec > 0.0);
        assert!(t.symbol_rate > 0.0);
        assert!(t.guard_ratio > 0.0 && t.guard_ratio < 1.0);
    }

    // -----------------------------------------------------------------------
    // QAM Mapping
    // -----------------------------------------------------------------------

    #[test]
    fn test_qpsk_map_demap_roundtrip() {
        let mapper = DrmQamMapper::new(QamOrder::Qam4);
        let test_cases: &[&[bool]] = &[
            &[false, false],
            &[false, true],
            &[true,  false],
            &[true,  true],
        ];
        for bits in test_cases {
            let sym = mapper.map(bits);
            let recovered = mapper.demap_hard(sym);
            assert_eq!(&recovered[..], *bits, "QPSK roundtrip failed for {:?}", bits);
        }
    }

    #[test]
    fn test_qpsk_unit_average_power() {
        let mapper = DrmQamMapper::new(QamOrder::Qam4);
        let all_bits: &[&[bool]] = &[&[false, false], &[false, true], &[true, false], &[true, true]];
        let avg: f64 = all_bits.iter().map(|b| mapper.map(b).mag_sq()).sum::<f64>() / 4.0;
        assert!((avg - 1.0).abs() < EPS_LOW, "QPSK avg power = {}", avg);
    }

    #[test]
    fn test_16qam_map_demap_roundtrip() {
        let mapper = DrmQamMapper::new(QamOrder::Qam16);
        for idx in 0..16u8 {
            let bits: Vec<bool> = (0..4).rev().map(|b| (idx >> b) & 1 == 1).collect();
            let sym = mapper.map(&bits);
            let recovered = mapper.demap_hard(sym);
            assert_eq!(recovered, bits, "16QAM roundtrip failed for idx {}", idx);
        }
    }

    #[test]
    fn test_16qam_unit_average_power() {
        let mapper = DrmQamMapper::new(QamOrder::Qam16);
        let total: f64 = (0..16u8).map(|idx| {
            let bits: Vec<bool> = (0..4).rev().map(|b| (idx >> b) & 1 == 1).collect();
            mapper.map(&bits).mag_sq()
        }).sum::<f64>() / 16.0;
        assert!((total - 1.0).abs() < 0.01, "16QAM avg power = {}", total);
    }

    #[test]
    fn test_64qam_map_demap_roundtrip() {
        let mapper = DrmQamMapper::new(QamOrder::Qam64);
        for idx in 0..64u8 {
            let bits: Vec<bool> = (0..6).rev().map(|b| (idx >> b) & 1 == 1).collect();
            let sym = mapper.map(&bits);
            let recovered = mapper.demap_hard(sym);
            assert_eq!(recovered, bits, "64QAM roundtrip failed for idx {}", idx);
        }
    }

    #[test]
    fn test_64qam_bits_per_symbol() {
        assert_eq!(QamOrder::Qam64.bits_per_symbol(), 6);
        assert_eq!(QamOrder::Qam16.bits_per_symbol(), 4);
        assert_eq!(QamOrder::Qam4.bits_per_symbol(), 2);
    }

    #[test]
    fn test_soft_llr_qpsk_sign() {
        let mapper = DrmQamMapper::new(QamOrder::Qam4);
        let sym = mapper.map(&[false, false]);
        let llrs = mapper.demap_soft(sym, 0.1);
        // LLR positive means bit=0 more likely
        assert!(llrs[0] > 0.0);
        assert!(llrs[1] > 0.0);
    }

    // -----------------------------------------------------------------------
    // Hierarchical Modulation
    // -----------------------------------------------------------------------

    #[test]
    fn test_hierarchical_map_extract_msp() {
        let msp: [bool; 4] = [false, true, false, true];
        let lsp: [bool; 2] = [false, true];
        let sym = HierarchicalMapper::map_hier64(&msp, &lsp);
        let msp_out = HierarchicalMapper::extract_msp(sym);
        assert_eq!(msp_out, msp, "MSP bits mismatch");
    }

    #[test]
    fn test_hierarchical_map_extract_lsp() {
        let msp: [bool; 4] = [false, false, false, false];
        let lsp: [bool; 2] = [false, true];
        let sym = HierarchicalMapper::map_hier64(&msp, &lsp);
        let lsp_out = HierarchicalMapper::extract_lsp(sym);
        assert_eq!(lsp_out, lsp, "LSP bits mismatch");
    }

    #[test]
    fn test_hierarchical_all_msp_bits() {
        // All 4 MSP bit combinations (with fixed lsp)
        for idx in 0..16u8 {
            let msp = [(idx>>3)&1==1, (idx>>2)&1==1, (idx>>1)&1==1, idx&1==1];
            let lsp = [false, false];
            let sym = HierarchicalMapper::map_hier64(&msp, &lsp);
            let msp_out = HierarchicalMapper::extract_msp(sym);
            assert_eq!(msp_out, msp, "MSP mismatch for idx {}", idx);
        }
    }

    // -----------------------------------------------------------------------
    // Convolutional Encoder / Viterbi Decoder
    // -----------------------------------------------------------------------

    #[test]
    fn test_conv_encoder_output_length() {
        let mut enc = DrmConvEncoder::new();
        let bits = vec![true, false, true, true, false, false, true, false];
        let coded = enc.encode(&bits);
        assert_eq!(coded.len(), bits.len() * 4);
    }

    #[test]
    fn test_conv_encoder_deterministic() {
        let mut enc1 = DrmConvEncoder::new();
        let mut enc2 = DrmConvEncoder::new();
        let bits = vec![true, false, false, true, true, false];
        assert_eq!(enc1.encode(&bits), enc2.encode(&bits));
    }

    #[test]
    fn test_conv_viterbi_decode_no_errors() {
        let mut enc = DrmConvEncoder::new();
        let dec = DrmViterbiDecoder::new();
        let bits = vec![true, false, true, true, false, false, true, false];
        let coded = enc.encode(&bits);
        let decoded = dec.decode(&coded);
        assert_eq!(&decoded[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_conv_all_zeros_encodes_to_all_zeros() {
        let mut enc = DrmConvEncoder::new();
        let bits = vec![false; 8];
        let coded = enc.encode(&bits);
        // With all-zero input and zero initial state, output must be all zeros
        assert!(coded.iter().all(|&b| !b), "All-zero input produces all-zero coded");
    }

    #[test]
    fn test_conv_viterbi_short_sequence() {
        let mut enc = DrmConvEncoder::new();
        let dec = DrmViterbiDecoder::new();
        let bits = vec![true, true, false, false];
        let coded = enc.encode(&bits);
        let decoded = dec.decode(&coded);
        assert_eq!(&decoded[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_tailbiting_encode_length() {
        let mut enc = DrmConvEncoder::new();
        let bits = vec![true, false, true, false, true, false, true, false, true, false];
        let coded = enc.encode_tailbiting(&bits);
        assert_eq!(coded.len(), bits.len() * 4);
    }

    // -----------------------------------------------------------------------
    // Puncturing
    // -----------------------------------------------------------------------

    #[test]
    fn test_puncture_half_rate_length() {
        let pattern = PuncturePattern::rate_half();
        let coded: Vec<bool> = (0..16).map(|i| i % 3 == 0).collect();
        let punctured = pattern.puncture(&coded);
        assert_eq!(punctured.len(), coded.len() / 2);
    }

    #[test]
    fn test_depuncture_half_rate_restores_length() {
        let pattern = PuncturePattern::rate_half();
        let coded: Vec<bool> = (0..16).map(|i| i % 2 == 0).collect();
        let punctured = pattern.puncture(&coded);
        let depunctured = pattern.depuncture(&punctured);
        assert!(depunctured.len() >= coded.len());
    }

    #[test]
    fn test_puncture_rate_three_quarter() {
        let pattern = PuncturePattern::rate_three_quarter();
        let coded = vec![true; 16];
        let punctured = pattern.puncture(&coded);
        // Pattern has 6 trues in 8 positions: 16 * 6/8 = 12
        assert_eq!(punctured.len(), 12);
    }

    #[test]
    fn test_depuncture_zeros_at_erased_positions() {
        let pattern = PuncturePattern::rate_half();
        // Pattern: [true, false, true, false]
        // Received 4 bits (from 8 coded)
        let received = vec![true, true, true, true];
        let dep = pattern.depuncture(&received);
        // Position 1 should be false (erased)
        assert!(!dep[1], "Position 1 is erased, should be false");
    }

    // -----------------------------------------------------------------------
    // Energy Dispersal (PRBS Scrambling)
    // -----------------------------------------------------------------------

    #[test]
    fn test_prbs_scramble_descramble_bytes() {
        let original = vec![0xA5u8, 0x3C, 0x7E, 0x01, 0xFF];
        let mut data = original.clone();
        let mut scrambler = EnergyDispersal::new();
        scrambler.process_bytes(&mut data);
        assert_ne!(data, original, "Scrambled must differ from original");
        // De-scramble with fresh PRBS
        let mut descrambler = EnergyDispersal::new();
        descrambler.process_bytes(&mut data);
        assert_eq!(data, original, "De-scrambled must match original");
    }

    #[test]
    fn test_prbs_scramble_bits() {
        let original: Vec<bool> = vec![true, false, true, true, false, false, true, false];
        let mut bits = original.clone();
        let mut s1 = EnergyDispersal::new();
        s1.process_bits(&mut bits);
        let mut s2 = EnergyDispersal::new();
        s2.process_bits(&mut bits);
        assert_eq!(bits, original);
    }

    #[test]
    fn test_prbs_period_is_511() {
        // x^9 + x^5 + 1 has period 2^9 - 1 = 511 bits
        let mut p1 = EnergyDispersal::new();
        let mut p2 = EnergyDispersal::new();
        // Advance p2 by 511
        for _ in 0..511 { p2.next_bit(); }
        // Both should produce identical sequences now
        let s1: Vec<bool> = (0..511).map(|_| p1.next_bit()).collect();
        let s2: Vec<bool> = (0..511).map(|_| p2.next_bit()).collect();
        assert_eq!(s1, s2, "PRBS period should be 511");
    }

    #[test]
    fn test_prbs_changes_data() {
        let original = vec![0u8; 10];
        let mut data = original.clone();
        let mut s = EnergyDispersal::new();
        s.process_bytes(&mut data);
        // Not all zeros (PRBS initial state is 0x1FF, so first byte should be non-zero)
        assert_ne!(data, original);
    }

    // -----------------------------------------------------------------------
    // CRC-8 and CRC-16
    // -----------------------------------------------------------------------

    #[test]
    fn test_crc8_deterministic() {
        let data = vec![0x12u8, 0x34, 0x56, 0x78];
        assert_eq!(crc8_drm(&data), crc8_drm(&data));
    }

    #[test]
    fn test_crc8_single_bit_change_detected() {
        let data = vec![0xA5u8, 0x3C, 0x7E];
        let mut modified = data.clone();
        modified[1] ^= 0x01;
        assert_ne!(crc8_drm(&data), crc8_drm(&modified));
    }

    #[test]
    fn test_crc8_non_zero_for_nonempty() {
        // Non-trivial data should produce a CRC value that changes with content
        let d1 = vec![0x01u8];
        let d2 = vec![0x02u8];
        assert_ne!(crc8_drm(&d1), crc8_drm(&d2));
    }

    #[test]
    fn test_crc16_deterministic() {
        let data = vec![0x01u8, 0x02, 0x03, 0x04, 0x05, 0x06];
        assert_eq!(crc16_drm(&data), crc16_drm(&data));
    }

    #[test]
    fn test_crc16_single_bit_change_detected() {
        let data = vec![0xDEu8, 0xAD, 0xBE, 0xEF];
        let mut modified = data.clone();
        modified[2] ^= 0x80;
        assert_ne!(crc16_drm(&data), crc16_drm(&modified));
    }

    #[test]
    fn test_crc16_all_bytes_covered() {
        let data = vec![0x00u8, 0x01, 0x00, 0x00];
        let d2   = vec![0x00u8, 0x00, 0x00, 0x00];
        assert_ne!(crc16_drm(&data), crc16_drm(&d2));
    }

    // -----------------------------------------------------------------------
    // FAC encode / decode
    // -----------------------------------------------------------------------

    #[test]
    fn test_fac_encode_decode_roundtrip() {
        let params = FacParameters {
            mode: 1,
            spectrum_occupancy: 3,
            interleaver_depth: 1,
            sdc_mode: 0,
            num_services: 2,
            programme_type: 15,
            crc8: 0,
            crc_ok: false,
        };
        let encoded = fac_encode(&params);
        let decoded = fac_decode(&encoded);
        assert!(decoded.crc_ok, "FAC CRC should pass");
        assert_eq!(decoded.mode, params.mode);
        assert_eq!(decoded.spectrum_occupancy, params.spectrum_occupancy);
        assert_eq!(decoded.interleaver_depth, params.interleaver_depth);
        assert_eq!(decoded.num_services, params.num_services);
    }

    #[test]
    fn test_fac_corrupt_fails_crc() {
        let params = FacParameters { mode: 2, spectrum_occupancy: 1, ..Default::default() };
        let mut encoded = fac_encode(&params);
        encoded[3] ^= 0xFF;
        let decoded = fac_decode(&encoded);
        assert!(!decoded.crc_ok, "Corrupted FAC should fail CRC");
    }

    #[test]
    fn test_fac_empty_data() {
        let result = fac_decode(&[]);
        assert!(!result.crc_ok);
        assert_eq!(result.mode, 0);
    }

    // -----------------------------------------------------------------------
    // SDC encode / decode
    // -----------------------------------------------------------------------

    #[test]
    fn test_sdc_encode_decode_roundtrip() {
        let entities = vec![
            SdcEntity {
                entity_type: SdcEntityType::Label,
                length: 4,
                payload: vec![b'D', b'R', b'M', b'!'],
            },
        ];
        let encoded = sdc_encode(&entities);
        let decoded = sdc_decode(&encoded);
        assert!(decoded.crc_ok, "SDC CRC should pass");
        assert!(!decoded.entities.is_empty());
    }

    #[test]
    fn test_sdc_corrupt_fails_crc() {
        let entities = vec![SdcEntity {
            entity_type: SdcEntityType::TimeDate,
            length: 3,
            payload: vec![0x01, 0x02, 0x03],
        }];
        let mut encoded = sdc_encode(&entities);
        if let Some(b) = encoded.first_mut() { *b ^= 0xFF; }
        let decoded = sdc_decode(&encoded);
        assert!(!decoded.crc_ok);
    }

    // -----------------------------------------------------------------------
    // Cell Interleaver
    // -----------------------------------------------------------------------

    #[test]
    fn test_cell_interleaver_roundtrip() {
        let n = 64;
        let il = CellInterleaver::new(n);
        let cells: Vec<Cx> = (0..n).map(|i| Cx::new(i as f64, 0.0)).collect();
        let interleaved = il.interleave(&cells);
        let recovered   = il.deinterleave(&interleaved);
        for (a, b) in cells.iter().zip(recovered.iter()) {
            assert!((a.re - b.re).abs() < EPS);
        }
    }

    #[test]
    fn test_cell_interleaver_is_permutation() {
        let n = 32;
        let il = CellInterleaver::new(n);
        let cells: Vec<Cx> = (0..n).map(|i| Cx::new(i as f64, 0.0)).collect();
        let interleaved = il.interleave(&cells);
        let mut orig_vals: Vec<i64> = cells.iter().map(|c| c.re as i64).collect();
        let mut inter_vals: Vec<i64> = interleaved.iter().map(|c| c.re as i64).collect();
        orig_vals.sort();
        inter_vals.sort();
        assert_eq!(orig_vals, inter_vals);
    }

    #[test]
    fn test_cell_interleaver_size_1() {
        let il = CellInterleaver::new(1);
        let cells = vec![Cx::new(42.0, 7.0)];
        let out = il.interleave(&cells);
        assert!((out[0].re - 42.0).abs() < EPS);
    }

    // -----------------------------------------------------------------------
    // Time Interleaver
    // -----------------------------------------------------------------------

    #[test]
    fn test_time_interleaver_output_length() {
        let mut ti = TimeInterleaver::new(4, 16);
        let frame: Vec<Cx> = (0..16).map(|i| Cx::new(i as f64, 0.0)).collect();
        let out = ti.process_frame(&frame);
        assert_eq!(out.len(), 16);
    }

    #[test]
    fn test_time_interleaver_depth_1_passthrough() {
        // With depth=1, output should equal input after the first call
        let n = 8;
        let mut ti = TimeInterleaver::new(1, n);
        let frame: Vec<Cx> = (0..n).map(|i| Cx::new(i as f64, 0.0)).collect();
        let out = ti.process_frame(&frame);
        assert_eq!(out.len(), frame.len());
        // All read_idx == write_idx, so out == frame
        for (a, b) in frame.iter().zip(out.iter()) {
            assert!((a.re - b.re).abs() < EPS);
        }
    }

    // -----------------------------------------------------------------------
    // OFDM Modulate / Demodulate
    // -----------------------------------------------------------------------

    #[test]
    fn test_ofdm_modulate_symbol_length_mode_b() {
        let params = DrmParams::for_mode(DrmMode::B);
        let nc = params.num_carriers;
        let sym_len = params.symbol_len();
        let modulator = DrmOfdmModulator::new(params);
        let cells = vec![Cx::new(1.0, 0.0); nc];
        let out = modulator.modulate_symbol(&cells);
        assert_eq!(out.len(), sym_len);
    }

    #[test]
    fn test_ofdm_modulate_demodulate_roundtrip_mode_b() {
        let params = DrmParams::for_mode(DrmMode::B);
        let nc = params.num_carriers;
        let modulator   = DrmOfdmModulator::new(params.clone());
        let demodulator = DrmOfdmDemodulator::new(params.clone());
        let cells: Vec<Cx> = (0..nc).map(|i| Cx::from_polar(1.0, PI * i as f64 / nc as f64)).collect();
        let modulated   = modulator.modulate_symbol(&cells);
        let recovered   = demodulator.demodulate_symbol(&modulated);
        assert_eq!(recovered.len(), nc);
        for (k, (orig, rec)) in cells.iter().zip(recovered.iter()).enumerate() {
            assert!((orig.re - rec.re).abs() < EPS_LOW, "carrier {}: re {} vs {}", k, orig.re, rec.re);
            assert!((orig.im - rec.im).abs() < EPS_LOW, "carrier {}: im {} vs {}", k, orig.im, rec.im);
        }
    }

    #[test]
    fn test_ofdm_modulate_demodulate_roundtrip_mode_e() {
        // Mode E uses power-of-2 FFT=2048
        let params = DrmParams::for_mode(DrmMode::E);
        let nc = params.num_carriers;
        let modulator   = DrmOfdmModulator::new(params.clone());
        let demodulator = DrmOfdmDemodulator::new(params.clone());
        let cells: Vec<Cx> = (0..nc).map(|i| Cx::from_polar(1.0, 2.0 * PI * i as f64 / nc as f64)).collect();
        let modulated = modulator.modulate_symbol(&cells);
        let recovered = demodulator.demodulate_symbol(&modulated);
        assert_eq!(recovered.len(), nc);
        for (orig, rec) in cells.iter().zip(recovered.iter()) {
            assert!((orig.re - rec.re).abs() < EPS_LOW);
        }
    }

    #[test]
    fn test_ofdm_frame_roundtrip_mode_b() {
        let params = DrmParams::for_mode(DrmMode::B);
        let nc = params.num_carriers;
        let n_syms = params.symbols_per_frame;
        let modulator   = DrmOfdmModulator::new(params.clone());
        let demodulator = DrmOfdmDemodulator::new(params.clone());
        let frame_cells: Vec<Vec<Cx>> = (0..n_syms).map(|s| {
            (0..nc).map(|k| Cx::from_polar(1.0, PI * (s * nc + k) as f64 / (n_syms * nc) as f64)).collect()
        }).collect();
        let modulated = modulator.modulate_frame(&frame_cells);
        let demodulated = demodulator.demodulate_frame(&modulated);
        assert_eq!(demodulated.len(), n_syms);
        for (s, (orig_sym, rec_sym)) in frame_cells.iter().zip(demodulated.iter()).enumerate() {
            for (k, (orig, rec)) in orig_sym.iter().zip(rec_sym.iter()).enumerate() {
                assert!((orig.re - rec.re).abs() < EPS_LOW,
                    "Symbol {}, carrier {}: {} vs {}", s, k, orig.re, rec.re);
            }
        }
    }

    #[test]
    fn test_ofdm_guard_interval_stripped_correctly() {
        let params = DrmParams::for_mode(DrmMode::D);
        let nc = params.num_carriers;
        let demod = DrmOfdmDemodulator::new(params.clone());
        let modu  = DrmOfdmModulator::new(params.clone());
        let cells = vec![Cx::new(1.0, 0.0); nc];
        let sym = modu.modulate_symbol(&cells);
        let out = demod.demodulate_symbol(&sym);
        assert_eq!(out.len(), nc);
        // DC and carrier 0 should be recoverable (all-ones input)
        assert!(out[0].mag() > 0.5, "Carrier 0 should have energy");
    }

    // -----------------------------------------------------------------------
    // Pilot Structure
    // -----------------------------------------------------------------------

    #[test]
    fn test_pilot_generation_mode_a_symbol0() {
        let pilots = generate_pilots(DrmMode::A, 0);
        assert!(!pilots.is_empty());
        for p in &pilots {
            assert!(p.carrier_idx < 288);
        }
    }

    #[test]
    fn test_pilot_generation_mode_e() {
        let pilots = generate_pilots(DrmMode::E, 3);
        assert!(!pilots.is_empty());
        for p in &pilots {
            assert!(p.carrier_idx < 213);
        }
    }

    #[test]
    fn test_frequency_reference_pilots_boosted() {
        let pilots = generate_pilots(DrmMode::A, 0);
        let freq_ref: Vec<_> = pilots.iter().filter(|p| p.pilot_type == PilotType::FreqRef).collect();
        assert!(!freq_ref.is_empty());
        for p in &freq_ref {
            assert!(p.value.mag() > 1.0, "Freq ref pilot should be boosted");
        }
    }

    // -----------------------------------------------------------------------
    // Channel Estimator
    // -----------------------------------------------------------------------

    #[test]
    fn test_channel_estimator_unit_channel() {
        let n = 16;
        let estimator = DrmChannelEstimator::new(n);
        // Known pilot = 1+0j, received = 1+0j → H[k] = 1+0j
        let received = vec![Cx::new(1.0, 0.0); n];
        let pilots: Vec<PilotCell> = (0..4).map(|k| PilotCell {
            carrier_idx: k * 4,
            symbol_idx: 0,
            pilot_type: PilotType::GainRef,
            value: Cx::new(1.0, 0.0), // known transmitted pilot value
        }).collect();
        let h = estimator.estimate(&received, &pilots);
        for hk in &h {
            assert!((hk.re - 1.0).abs() < EPS_LOW, "H should be 1+0j");
            assert!(hk.im.abs() < EPS_LOW);
        }
    }

    #[test]
    fn test_channel_estimator_equalize_recovers_data() {
        let n = 8;
        let estimator = DrmChannelEstimator::new(n);
        let h_true = Cx::new(0.5, 0.3);
        // Data = all 1+0j; received = 1+0j * h_true = h_true at all carriers
        let data: Vec<Cx> = vec![Cx::new(1.0, 0.0); n];
        let received: Vec<Cx> = data.iter().map(|x| x.mul(&h_true)).collect();
        // Pilots: known tx pilot = 1+0j, received = h_true
        let pilots: Vec<PilotCell> = vec![
            PilotCell { carrier_idx: 0, symbol_idx: 0, pilot_type: PilotType::GainRef, value: Cx::new(1.0, 0.0) },
            PilotCell { carrier_idx: 4, symbol_idx: 0, pilot_type: PilotType::GainRef, value: Cx::new(1.0, 0.0) },
        ];
        let h_est = estimator.estimate(&received, &pilots);
        // H estimated at pilots: received[k] / 1 = h_true
        let equalized = estimator.equalize(&received, &h_est);
        for (orig, eq) in data.iter().zip(equalized.iter()) {
            assert!((orig.re - eq.re).abs() < EPS_LOW, "Equalized re mismatch");
            assert!((orig.im - eq.im).abs() < EPS_LOW, "Equalized im mismatch");
        }
    }

    // -----------------------------------------------------------------------
    // MSC Protection Level
    // -----------------------------------------------------------------------

    #[test]
    fn test_protection_level_code_rates() {
        assert_eq!(MscProtectionLevel::Level0.code_rate(), (3, 8));
        assert_eq!(MscProtectionLevel::Level3.code_rate(), (3, 4));
    }

    #[test]
    fn test_protection_level_qam_requirements() {
        assert_eq!(MscProtectionLevel::Level0.required_qam(), QamOrder::Qam4);
        assert_eq!(MscProtectionLevel::Level2.required_qam(), QamOrder::Qam16);
        assert_eq!(MscProtectionLevel::Level3.required_qam(), QamOrder::Qam64);
    }

    // -----------------------------------------------------------------------
    // Audio Super-Frame
    // -----------------------------------------------------------------------

    #[test]
    fn test_audio_superframe_parse_valid() {
        let num_aus = 2usize;
        let mut data = Vec::new();
        data.push(((num_aus - 1) as u8) << 4);
        data.push(0x00); data.push(0x00); // AU0 at offset 0
        data.push(0x00); data.push(0x03); // AU1 at offset 3
        data.extend_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE]);
        let crc = crc16_drm(&data);
        data.push((crc >> 8) as u8);
        data.push((crc & 0xFF) as u8);
        let sf = AudioSuperFrame::parse(&data);
        assert!(sf.is_some());
        let sf = sf.unwrap();
        assert!(sf.crc_ok);
        assert_eq!(sf.num_aus, num_aus);
    }

    #[test]
    fn test_audio_superframe_too_short() {
        assert!(AudioSuperFrame::parse(&[0u8; 2]).is_none());
    }

    #[test]
    fn test_audio_superframe_au_extraction() {
        let num_aus = 2usize;
        let mut data = Vec::new();
        data.push(((num_aus - 1) as u8) << 4);
        data.push(0x00); data.push(0x00);
        data.push(0x00); data.push(0x02);
        data.extend_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        let crc = crc16_drm(&data);
        data.push((crc >> 8) as u8);
        data.push((crc & 0xFF) as u8);
        if let Some(sf) = AudioSuperFrame::parse(&data) {
            assert!(sf.extract_au(0).is_some());
            assert!(sf.extract_au(2).is_none()); // out of range
        }
    }

    // -----------------------------------------------------------------------
    // Frame Timing
    // -----------------------------------------------------------------------

    #[test]
    fn test_frame_timing_all_modes_positive() {
        for mode in [DrmMode::A, DrmMode::B, DrmMode::C, DrmMode::D, DrmMode::E] {
            let p = DrmParams::for_mode(mode);
            let t = FrameTiming::for_mode(&p);
            assert!(t.frame_duration_sec > 0.0);
            assert!(t.symbol_rate > 0.0);
            assert!(t.guard_ratio > 0.0 && t.guard_ratio < 1.0,
                "Guard ratio {:?}: {}", mode, t.guard_ratio);
        }
    }

    #[test]
    fn test_frame_timing_superframe_larger_than_frame() {
        let p = DrmParams::for_mode(DrmMode::B);
        let t = FrameTiming::for_mode(&p);
        assert!(t.superframe_duration_sec > t.frame_duration_sec);
    }
}

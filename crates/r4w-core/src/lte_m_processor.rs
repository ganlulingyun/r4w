//! LTE-M / eMTC Cat-M1 (3GPP Rel-13 TS 36.211/36.212/36.213) Processor
//!
//! Implements narrowband IoT processing for LTE-M (Cat-M1):
//! - 1.4 MHz bandwidth (6 PRBs, 72 subcarriers), 15 kHz SCS
//! - MPDCCH (MTC Physical Downlink Control Channel)
//! - MPDSCH (MTC Physical Downlink Shared Channel)
//! - MPUSCH (MTC Physical Uplink Shared Channel, SC-FDMA)
//! - Coverage Enhancement Modes A/B (repetition combining)
//! - PSS/SSS cell search (ZC and m-sequence based)
//! - Narrowband frequency hopping (type 2, cell-ID seeded)
//! - HARQ process management (DL async, UL sync)
//! - Power saving (eDRX up to 43.69 min, PSM, WUS)
//! - Half-duplex FDD scheduling with guard periods
//! - VoLTE SPS (semi-persistent scheduling)
//!
//! All arithmetic implemented in pure Rust with `std` only.

// ============================================================
// Constants
// ============================================================

/// Number of physical resource blocks in 1.4 MHz bandwidth
pub const N_PRB: usize = 6;
/// Subcarriers per PRB
pub const SC_PER_PRB: usize = 12;
/// Total subcarriers in 6-PRB system (72)
pub const N_SC: usize = N_PRB * SC_PER_PRB;
/// Subcarrier spacing [Hz]
pub const SCS_HZ: f64 = 15_000.0;
/// Useful symbol duration [µs]
pub const SYMBOL_DURATION_US: f64 = 1_000_000.0 / SCS_HZ;
/// Normal cyclic prefix length for first symbol in slot (samples @ 1.92 MHz, for 128-pt FFT)
pub const CP_FIRST_LEN: usize = 10;
/// Normal cyclic prefix length for symbols 1-6 (samples @ 1.92 MHz, for 128-pt FFT)
pub const CP_NORMAL_LEN: usize = 9;
/// FFT size for 128-pt (covers 72 active SCs in 1.4 MHz)
pub const FFT_SIZE: usize = 128;
/// Symbols per slot
pub const SYMS_PER_SLOT: usize = 7;
/// Symbols per subframe
pub const SYMS_PER_SF: usize = 14;
/// Subframes per radio frame
pub const SF_PER_FRAME: usize = 10;
/// Maximum UE TX power Cat-M1 [dBm]
pub const MAX_TX_POWER_DBM: f64 = 23.0;
/// Maximum HARQ processes, downlink
pub const MAX_HARQ_DL: usize = 8;
/// Maximum HARQ processes, uplink
pub const MAX_HARQ_UL: usize = 8;

const PI: f64 = std::f64::consts::PI;

// ============================================================
// Complex number primitive (no external crates)
// ============================================================

/// Complex64 — pair of f64 representing a complex number.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    #[inline] pub fn new(re: f64, im: f64) -> Self { Self { re, im } }
    #[inline] pub fn zero() -> Self { Self { re: 0.0, im: 0.0 } }
    #[inline] pub fn from_polar(r: f64, theta: f64) -> Self {
        Self { re: r * theta.cos(), im: r * theta.sin() }
    }
    #[inline] pub fn magnitude(&self) -> f64 { (self.re * self.re + self.im * self.im).sqrt() }
    #[inline] pub fn magnitude_sq(&self) -> f64 { self.re * self.re + self.im * self.im }
    #[inline] pub fn conjugate(&self) -> Self { Self { re: self.re, im: -self.im } }
    #[inline] pub fn add(&self, o: &Self) -> Self { Self { re: self.re + o.re, im: self.im + o.im } }
    #[inline] pub fn sub(&self, o: &Self) -> Self { Self { re: self.re - o.re, im: self.im - o.im } }
    #[inline] pub fn mul(&self, o: &Self) -> Self {
        Self {
            re: self.re * o.re - self.im * o.im,
            im: self.re * o.im + self.im * o.re,
        }
    }
    #[inline] pub fn scale(&self, s: f64) -> Self { Self { re: self.re * s, im: self.im * s } }
    #[inline] pub fn arg(&self) -> f64 { self.im.atan2(self.re) }
}

impl std::ops::Add for Complex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self { Complex::add(&self, &rhs) }
}
impl std::ops::Mul for Complex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self { Complex::mul(&self, &rhs) }
}

// ============================================================
// Cooley-Tukey radix-2 DIT FFT (power-of-2, in-place)
// ============================================================

/// Bit-reversal permutation.
fn bit_reverse_copy(x: &[Complex], out: &mut Vec<Complex>) {
    let n = x.len();
    assert!(n.is_power_of_two());
    let bits = n.trailing_zeros() as usize;
    out.resize(n, Complex::zero());
    for i in 0..n {
        let mut rev = 0usize;
        let mut tmp = i;
        for _ in 0..bits { rev = (rev << 1) | (tmp & 1); tmp >>= 1; }
        out[rev] = x[i];
    }
}

/// In-place Cooley-Tukey FFT. `inverse=true` performs IFFT (divides by N).
pub fn fft_inplace(buf: &mut [Complex], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two());
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = sign * 2.0 * PI / len as f64;
        let w_step = Complex::from_polar(1.0, ang);
        let mut i = 0;
        while i < n {
            let mut w = Complex::new(1.0, 0.0);
            for j in 0..half {
                let u = buf[i + j];
                let v = buf[i + j + half].mul(&w);
                buf[i + j]       = u.add(&v);
                buf[i + j + half] = u.sub(&v);
                w = w.mul(&w_step);
            }
            i += len;
        }
        len <<= 1;
    }
    if inverse {
        let nf = n as f64;
        for s in buf.iter_mut() { *s = s.scale(1.0 / nf); }
    }
}

/// Forward FFT — returns new Vec.
pub fn fft(x: &[Complex]) -> Vec<Complex> {
    let mut out = Vec::new();
    bit_reverse_copy(x, &mut out);
    fft_inplace(&mut out, false);
    out
}

/// Inverse FFT — returns new Vec (divides by N internally).
pub fn ifft(x: &[Complex]) -> Vec<Complex> {
    let mut out = Vec::new();
    bit_reverse_copy(x, &mut out);
    fft_inplace(&mut out, true);
    out
}

// ============================================================
// OFDM subcarrier → FFT bin mapping (LTE DC-centred)
// ============================================================

/// Map subcarrier index (0..N_SC-1) to FFT bin index (0..FFT_SIZE-1).
///
/// LTE places the active subcarriers symmetrically around DC (bin 0):
/// - Lower N_SC/2 subcarriers → bins 1 .. N_SC/2  (positive frequencies)
/// - Upper N_SC/2 subcarriers → bins FFT_SIZE-N_SC/2 .. FFT_SIZE-1  (negative frequencies)
/// - DC (bin 0) and guard bands are unused.
///
/// For N_SC=72, FFT_SIZE=128:
///   sc  0..35 → bins  1..36
///   sc 36..71 → bins 92..127
#[inline]
fn sc_to_bin(sc: usize) -> usize {
    let half = N_SC / 2; // 36
    if sc < half {
        sc + 1                               // bins 1..36
    } else {
        FFT_SIZE - N_SC + sc                 // bins 92..127 (= 128-72+36..128-72+71)
    }
}

/// CP length for symbol index within a slot (0-6).
#[inline]
fn cp_length(sym_in_slot: usize) -> usize {
    if sym_in_slot == 0 { CP_FIRST_LEN } else { CP_NORMAL_LEN }
}

/// OFDM modulate one subframe (14 symbols).
/// `freq_grid`: 14 × N_SC complex frequency-domain symbols.
/// Returns flat time-domain sample Vec with cyclic prefix prepended to each symbol.
pub fn ofdm_modulate(freq_grid: &[Vec<Complex>]) -> Vec<Complex> {
    assert_eq!(freq_grid.len(), SYMS_PER_SF);
    let mut out = Vec::new();
    for (sym_idx, sc_vec) in freq_grid.iter().enumerate() {
        assert_eq!(sc_vec.len(), N_SC);
        // Map subcarriers into FFT input buffer (guard bands remain zero)
        let mut fft_in = vec![Complex::zero(); FFT_SIZE];
        for (sc, &val) in sc_vec.iter().enumerate() {
            fft_in[sc_to_bin(sc)] = val;
        }
        // IFFT to time domain
        let mut td = Vec::new();
        bit_reverse_copy(&fft_in, &mut td);
        fft_inplace(&mut td, true);
        // Prepend cyclic prefix
        let cp = cp_length(sym_idx % SYMS_PER_SLOT);
        let cp_start = FFT_SIZE - cp;
        for i in cp_start..FFT_SIZE { out.push(td[i]); }
        out.extend_from_slice(&td);
    }
    out
}

/// OFDM demodulate one subframe from flat time-domain samples.
/// Returns 14 × N_SC frequency-domain grid.
pub fn ofdm_demodulate(samples: &[Complex]) -> Vec<Vec<Complex>> {
    let mut grid = Vec::with_capacity(SYMS_PER_SF);
    let mut pos = 0;
    for sym_idx in 0..SYMS_PER_SF {
        let cp = cp_length(sym_idx % SYMS_PER_SLOT);
        pos += cp; // skip CP
        let spec = fft(&samples[pos..pos + FFT_SIZE]);
        pos += FFT_SIZE;
        let mut sc_vec = Vec::with_capacity(N_SC);
        for sc in 0..N_SC {
            sc_vec.push(spec[sc_to_bin(sc)]);
        }
        grid.push(sc_vec);
    }
    grid
}

// ============================================================
// PSS — Primary Synchronisation Signal (Zadoff-Chu length-63)
// ============================================================

/// Generate PSS sequence (62 samples, skipping DC position at k=31).
/// Root u ∈ {25, 29, 34} selects the three physical-layer cell ID groups.
/// Phase: d(n) = exp(-jπu·k(k+1)/63), k=n for n<31, k=n+1 for n≥31.
pub fn generate_pss(root_u: u32) -> Vec<Complex> {
    let n_zc: f64 = 63.0;
    let u = root_u as f64;
    (0u32..62).map(|n| {
        let k = if n < 31 { n } else { n + 1 } as f64;
        Complex::from_polar(1.0, -PI * u * k * (k + 1.0) / n_zc)
    }).collect()
}

/// Cell ID group (N_ID_2) → PSS root u.
pub fn pss_root(n_id_2: u8) -> u32 {
    match n_id_2 { 0 => 25, 1 => 29, _ => 34 }
}

/// Slide `pss` over `received` and return `(peak_offset, correlation_magnitude)`.
pub fn pss_correlate(received: &[Complex], pss: &[Complex]) -> (usize, f64) {
    let plen = pss.len();
    if received.len() < plen { return (0, 0.0); }
    let mut best_pos = 0;
    let mut best_corr = 0.0f64;
    for start in 0..=(received.len() - plen) {
        let mut acc = Complex::zero();
        for i in 0..plen {
            acc = acc.add(&received[start + i].mul(&pss[i].conjugate()));
        }
        let mag = acc.magnitude();
        if mag > best_corr { best_corr = mag; best_pos = start; }
    }
    (best_pos, best_corr)
}

// ============================================================
// SSS — Secondary Synchronisation Signal (m-sequence based)
// ============================================================

/// Generate a degree-5 m-sequence (31 chips) from a 5-bit initial state.
/// Polynomial: x^5 + x^2 + 1 → feedback taps at positions 2 and 4 (0-indexed).
fn m_seq_5(init: u8) -> [u8; 31] {
    let mut reg = [0u8; 5];
    let init5 = init & 0x1F;
    for i in 0..5 { reg[i] = (init5 >> (4 - i)) & 1; }
    let mut out = [0u8; 31];
    for chip in out.iter_mut() {
        *chip = reg[4];
        let fb = reg[2] ^ reg[4];
        reg[4] = reg[3]; reg[3] = reg[2]; reg[2] = reg[1]; reg[1] = reg[0]; reg[0] = fb;
    }
    out
}

/// Map binary chip (0/1) to BPSK symbol (+1/-1 as i8).
#[inline] fn bpsk_map(chip: u8) -> i8 { if chip == 0 { 1 } else { -1 } }

/// BPSK product of two chips: (bpsk_map(a)) * (bpsk_map(b)) = bpsk_map(a XOR b).
#[inline] fn bpsk_prod(a: u8, b: u8) -> i8 { bpsk_map(a ^ b) }

/// Generate SSS (62-element BPSK sequence) for given n_ID_1, n_ID_2, and subframe (0 or 5).
/// Implements 3GPP TS 36.211 §6.11.2.
pub fn generate_sss(n_id_1: u16, n_id_2: u8, subframe: u8) -> Vec<i8> {
    let q_prime = n_id_1 / 30;
    let q       = (n_id_1 + q_prime * (q_prime + 1) / 2) / 30;
    let m_prime = n_id_1 + q_prime * (q_prime + 1) / 2 - 30 * q;
    let m0      = (m_prime % 31) as usize;
    let m1      = ((m0 as u16 + m_prime / 31 + 1) % 31) as usize;

    let s = m_seq_5(1);       // init = 0b00001
    let c = m_seq_5(1);       // same base polynomial, different cyclic shift applied below
    let z = m_seq_5(1);

    // Cyclic shifts
    let s0: Vec<u8> = (0..31).map(|i: usize| s[(i + m0) % 31]).collect();
    let s1: Vec<u8> = (0..31).map(|i: usize| s[(i + m1) % 31]).collect();
    let c0: Vec<u8> = (0..31).map(|i: usize| c[(i + n_id_2 as usize) % 31]).collect();
    let c1: Vec<u8> = (0..31).map(|i: usize| c[(i + n_id_2 as usize + 3) % 31]).collect();
    let z1_m0: Vec<u8> = (0..31).map(|i: usize| z[(i + m0 % 8) % 31]).collect();
    let z1_m1: Vec<u8> = (0..31).map(|i: usize| z[(i + m1 % 8) % 31]).collect();

    let mut sss = vec![0i8; 62];
    if subframe == 0 {
        for i in 0..31 {
            // d(2i)   = s0(i) · c0(i)
            // d(2i+1) = s1(i) · c1(i) · z1_m0(i)
            sss[2 * i]     = bpsk_prod(s0[i], c0[i]);
            sss[2 * i + 1] = bpsk_map(s1[i] ^ c1[i] ^ z1_m0[i]);
        }
    } else {
        for i in 0..31 {
            sss[2 * i]     = bpsk_prod(s1[i], c0[i]);
            sss[2 * i + 1] = bpsk_map(s0[i] ^ c1[i] ^ z1_m1[i]);
        }
    }
    sss
}

/// Physical cell ID: N_cell_ID = 3 × n_ID_1 + n_ID_2.
pub fn cell_id(n_id_1: u16, n_id_2: u8) -> u16 { 3 * n_id_1 + n_id_2 as u16 }

// ============================================================
// QPP Interleaver (3GPP TS 36.212 §5.1.1 Table 5.1.3-3)
// ============================================================

/// Return QPP parameters (f1, f2) for block size K, or `None` if K is not in the table.
pub fn qpp_params(k: usize) -> Option<(usize, usize)> {
    // Selected entries: (K, f1, f2)
    const TABLE: &[(usize, usize, usize)] = &[
        (40,  3,  10), (48,  7,  12), (56, 19, 42), (64,   7,  16),
        (72,  7,  18), (80,  7,  20), (88, 11, 22), (96,   7,  24),
        (104, 11, 26), (112,  7,  28),(120, 41, 30), (128, 103,  32),
        (136, 15, 34), (144, 25,  36),(152, 75, 38), (160,  47,  40),
        (168, 91, 42), (176, 55,  44),(184, 31, 46), (192,  39,  48),
        (200, 15, 50), (208, 15,  52),(216,  9, 54), (224,  41,  56),
        (232, 17, 58), (240, 25,  60),(248, 37, 62), (256,  19,  32),
        (512, 7,  64), (1024, 11,128),(2048, 89,128),(4096,   7, 512),
        (6144,263,480),
    ];
    TABLE.iter().find(|&&(sz, _, _)| sz == k).map(|&(_, f1, f2)| (f1, f2))
}

/// QPP interleave: π(i) = (f1·i + f2·i²) mod K.
pub fn qpp_interleave(input: &[u8], f1: usize, f2: usize) -> Vec<u8> {
    let k = input.len();
    let mut out = vec![0u8; k];
    for i in 0..k { out[(f1 * i + f2 * i * i) % k] = input[i]; }
    out
}

/// QPP de-interleave (inverse permutation).
pub fn qpp_deinterleave(input: &[u8], f1: usize, f2: usize) -> Vec<u8> {
    let k = input.len();
    let mut out = vec![0u8; k];
    for i in 0..k { out[i] = input[(f1 * i + f2 * i * i) % k]; }
    out
}

// ============================================================
// Turbo Encoder (Rate 1/3, K=4, constituent RSC)
// ============================================================

/// RSC encode a bit vector. Returns (systematic, parity) streams of length N+4 (with tail bits).
/// Generator: (1, G2/G1) with G1 = 1+D+D^3, G2 = 1+D^2+D^3 (octal 13, 15).
pub fn rsc_encode(input: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let n = input.len();
    let mut sys = Vec::with_capacity(n + 4);
    let mut par = Vec::with_capacity(n + 4);
    let mut state = [0u8; 3]; // 3-bit shift register

    // Encode one bit: feedback RSC with memory 3
    let mut encode_bit = |bit: u8, st: &mut [u8; 3]| -> (u8, u8) {
        // Feedback: u_k = d_k XOR s1 XOR s2 (based on G1 = 1+D+D^3, D^3 tap = s2)
        let fb = bit ^ st[1] ^ st[2];
        // Parity: p_k = fb XOR s0 XOR s2 (G2 = 1+D^2+D^3 taps)
        let p = fb ^ st[0] ^ st[2];
        st[2] = st[1]; st[1] = st[0]; st[0] = fb;
        (bit, p)
    };

    for &b in input {
        let (s, p) = encode_bit(b, &mut state);
        sys.push(s); par.push(p);
    }
    // 3 tail bits to flush state
    for _ in 0..3 {
        let tail = state[1] ^ state[2];
        let (s, p) = encode_bit(tail, &mut state);
        sys.push(s); par.push(p);
    }
    (sys, par)
}

/// Turbo encoder: systematic + two RSC encoders (second on QPP-interleaved data).
pub struct TurboEncoder { pub k: usize, pub f1: usize, pub f2: usize }

impl TurboEncoder {
    /// Create for block size K. Returns `None` if K is not in the QPP table.
    pub fn new(k: usize) -> Option<Self> {
        qpp_params(k).map(|(f1, f2)| Self { k, f1, f2 })
    }

    /// Encode input bits. Returns (systematic, parity1, parity2) each of length K+3.
    pub fn encode(&self, input: &[u8]) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        assert_eq!(input.len(), self.k);
        let (sys, par1) = rsc_encode(input);
        let interleaved = qpp_interleave(input, self.f1, self.f2);
        let (_, par2)   = rsc_encode(&interleaved);
        (sys, par1, par2)
    }

    /// Multiplex [d_k, d'_k, d''_k, ...] for rate matching.
    pub fn multiplex(&self, sys: &[u8], par1: &[u8], par2: &[u8]) -> Vec<u8> {
        let len = sys.len().min(par1.len()).min(par2.len());
        let mut out = Vec::with_capacity(len * 3);
        for i in 0..len { out.push(sys[i]); out.push(par1[i]); out.push(par2[i]); }
        out
    }
}

// ============================================================
// Rate Matching — Circular Buffer (3GPP TS 36.212 §5.1.4)
// ============================================================

/// Rate matcher: wraps encoded bits into circular buffer and extracts E bits.
pub struct RateMatcher {
    /// Total circular buffer capacity = 3 × (K + tail_bits)
    pub n_cb: usize,
    /// Redundancy version (0-3)
    pub rv: u8,
}

impl RateMatcher {
    pub fn new(encoded_len: usize, rv: u8) -> Self {
        Self { n_cb: encoded_len, rv }
    }

    /// Starting position in circular buffer for this RV.
    pub fn start_offset(&self) -> usize {
        let offsets = [0usize, self.n_cb / 4, self.n_cb / 2, 3 * self.n_cb / 4];
        offsets[(self.rv as usize) % 4]
    }

    /// Extract `e_r` bits from `circular_buf` starting at RV offset, wrapping circularly.
    pub fn extract(&self, circular_buf: &[u8], e_r: usize) -> Vec<u8> {
        let n = circular_buf.len();
        if n == 0 { return vec![0u8; e_r]; }
        let k0 = self.start_offset() % n;
        (0..e_r).map(|i| circular_buf[(k0 + i) % n]).collect()
    }

    /// HARQ soft-bit combining (Chase combining): saturating addition of LLRs.
    pub fn combine(existing: &[i8], new_bits: &[i8]) -> Vec<i8> {
        existing.iter().zip(new_bits.iter()).map(|(&a, &b)| {
            (a as i16 + b as i16).clamp(-127, 127) as i8
        }).collect()
    }
}

// ============================================================
// DFT-Spreading (SC-FDMA for MPUSCH uplink)
// ============================================================

/// DFT-spread M input symbols: apply M-point DFT with energy normalization.
/// Used before subcarrier mapping in SC-FDMA uplink (MPUSCH).
/// `symbols.len()` must be a power of 2 for this implementation.
pub fn dft_spread(symbols: &[Complex]) -> Vec<Complex> {
    let m = symbols.len();
    let fft_len = m.next_power_of_two();
    let mut buf = symbols.to_vec();
    buf.resize(fft_len, Complex::zero());
    let mut tmp = Vec::new();
    bit_reverse_copy(&buf, &mut tmp);
    fft_inplace(&mut tmp, false);
    // Energy-normalise: divide by sqrt(M) so average power is preserved
    let norm = 1.0 / (m as f64).sqrt();
    tmp[..m].iter().map(|s| s.scale(norm)).collect()
}

/// IDFT de-spread: undo DFT spreading at SC-FDMA receiver.
/// Takes M spread symbols, produces M original symbols.
pub fn idft_despread(spread: &[Complex]) -> Vec<Complex> {
    let m = spread.len();
    // Undo energy normalisation (multiply by sqrt(M)) then IFFT
    let norm = (m as f64).sqrt();
    let mut buf: Vec<Complex> = spread.iter().map(|s| s.scale(norm)).collect();
    buf.resize(m.next_power_of_two(), Complex::zero());
    let mut tmp = Vec::new();
    bit_reverse_copy(&buf, &mut tmp);
    fft_inplace(&mut tmp, true); // IFFT divides by fft_len
    // Correct for zero-padding: IFFT of M-point DFT padded to fft_len gives x[n]/m * fft_len
    // but for m == next_power_of_two(m) this is exact
    tmp[..m].to_vec()
}

// ============================================================
// Modulation: QPSK, 16-QAM, pi/2-BPSK
// ============================================================

/// QPSK modulation (Gray coded). Input pairs of bits → complex symbols with |s|²=1.
pub fn qpsk_modulate(bits: &[u8]) -> Vec<Complex> {
    assert_eq!(bits.len() % 2, 0);
    let norm = 1.0 / 2.0f64.sqrt();
    bits.chunks(2).map(|c| {
        Complex::new(
            if c[0] == 0 {  norm } else { -norm },
            if c[1] == 0 {  norm } else { -norm },
        )
    }).collect()
}

/// 16-QAM modulation (Gray coded). Input 4 bits per symbol.
pub fn qam16_modulate(bits: &[u8]) -> Vec<Complex> {
    assert_eq!(bits.len() % 4, 0);
    let norm = 1.0 / 10.0f64.sqrt();
    bits.chunks(4).map(|c| {
        let i_amp = if c[1] == 0 { 3.0 } else { 1.0 };
        let q_amp = if c[3] == 0 { 3.0 } else { 1.0 };
        Complex::new(
            if c[0] == 0 {  i_amp } else { -i_amp } * norm,
            if c[2] == 0 {  q_amp } else { -q_amp } * norm,
        )
    }).collect()
}

/// pi/2-BPSK modulation for single-tone MPUSCH (minimum PAPR).
/// Phase alternates by π/2 each symbol: φ(n) = base_phase + n·π/2.
pub fn pi2_bpsk_modulate(bits: &[u8]) -> Vec<Complex> {
    bits.iter().enumerate().map(|(n, &b)| {
        let base = if b == 0 { 0.0 } else { PI };
        Complex::from_polar(1.0, base + n as f64 * PI / 2.0)
    }).collect()
}

/// QPSK hard-decision demodulation.
pub fn qpsk_demodulate(syms: &[Complex]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(syms.len() * 2);
    for s in syms {
        bits.push(if s.re >= 0.0 { 0 } else { 1 });
        bits.push(if s.im >= 0.0 { 0 } else { 1 });
    }
    bits
}

// ============================================================
// DCI Formats (MPDCCH)
// ============================================================

/// DCI format identifier for LTE-M.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DciFormat {
    /// 6-0A: UL scheduling, CE Mode A
    F6_0A,
    /// 6-0B: UL scheduling, CE Mode B
    F6_0B,
    /// 6-1A: DL scheduling, CE Mode A
    F6_1A,
    /// 6-1B: DL scheduling, CE Mode B
    F6_1B,
    /// 6-2: Paging / RAR / system info
    F6_2,
}

/// DCI Format 6-1A — downlink scheduling assignment for CE Mode A.
#[derive(Clone, Debug)]
pub struct Dci6_1A {
    /// Resource assignment (6 bits for 6 PRBs)
    pub resource_assignment: u8,
    /// MCS index (5 bits, 0-28)
    pub mcs: u8,
    /// HARQ process ID (3 bits, 0-7)
    pub harq_pid: u8,
    /// New Data Indicator
    pub ndi: bool,
    /// Redundancy Version (2 bits, 0-3)
    pub rv: u8,
    /// Repetition number (3 bits, maps 0-7 → 1/2/4/8/16/32/64/128)
    pub repetition: u8,
    /// Subframe bundling flag
    pub bundling: bool,
    /// TPC command for PUCCH (2 bits)
    pub tpc: u8,
    /// SRS request
    pub srs_req: bool,
}

impl Dci6_1A {
    /// Encode to bit vector (MSB first).
    pub fn encode(&self) -> Vec<u8> {
        let mut bits = Vec::new();
        let push_field = |bits: &mut Vec<u8>, val: u8, n: usize| {
            for i in (0..n).rev() { bits.push((val >> i) & 1); }
        };
        push_field(&mut bits, self.resource_assignment, 6);
        push_field(&mut bits, self.mcs, 5);
        push_field(&mut bits, self.harq_pid, 3);
        bits.push(self.ndi as u8);
        push_field(&mut bits, self.rv, 2);
        push_field(&mut bits, self.repetition, 3);
        bits.push(self.bundling as u8);
        push_field(&mut bits, self.tpc, 2);
        bits.push(self.srs_req as u8);
        bits
    }

    /// Decode from bit vector. Returns `None` if fewer than 24 bits provided.
    pub fn decode(bits: &[u8]) -> Option<Self> {
        if bits.len() < 24 { return None; }
        let mut idx = 0;
        let mut read = |n: usize| -> u8 {
            let mut v = 0u8;
            for i in 0..n { v = (v << 1) | bits[idx + i]; }
            idx += n;
            v
        };
        Some(Self {
            resource_assignment: read(6),
            mcs:                  read(5),
            harq_pid:             read(3),
            ndi:                  read(1) != 0,
            rv:                   read(2),
            repetition:           read(3),
            bundling:             read(1) != 0,
            tpc:                  read(2),
            srs_req:              read(1) != 0,
        })
    }

    /// Decode 3-bit repetition field to actual repetition count.
    pub fn repetition_count(&self) -> u32 {
        [1u32, 2, 4, 8, 16, 32, 64, 128][(self.repetition as usize) & 7]
    }
}

// ============================================================
// TBS (Transport Block Size) lookup table
// ============================================================

/// TBS lookup (3GPP TS 36.213 Table 7.1.7.2.1-1).
/// Returns TBS in bits for ITBS ∈ 0..10 and N_PRB ∈ 1..6.
pub fn tbs_lookup(i_tbs: usize, n_prb: usize) -> usize {
    const TABLE: &[[usize; 6]] = &[
        // N_PRB: 1    2    3    4    5    6
        [  16,   32,  56,  88, 120, 152], // I_TBS=0
        [  24,   56,  88, 144, 176, 208], // I_TBS=1
        [  32,   72, 144, 176, 208, 256], // I_TBS=2
        [  40,  104, 176, 208, 256, 328], // I_TBS=3
        [  56,  120, 208, 256, 328, 408], // I_TBS=4
        [  72,  144, 224, 328, 424, 504], // I_TBS=5
        [  88,  176, 256, 392, 504, 600], // I_TBS=6
        [ 104,  224, 328, 472, 584, 712], // I_TBS=7
        [ 120,  256, 392, 536, 680, 808], // I_TBS=8
        [ 136,  296, 456, 616, 776, 936], // I_TBS=9
        [ 144,  328, 504, 680, 872,1032], // I_TBS=10
    ];
    if i_tbs >= TABLE.len() || n_prb < 1 || n_prb > 6 { return 0; }
    TABLE[i_tbs][n_prb - 1]
}

/// MCS → (I_TBS, modulation_order) for DL.  mod_order: 2=QPSK, 4=16QAM.
pub fn mcs_to_tbs_mod(mcs_idx: usize) -> (usize, usize) {
    const TABLE: &[(usize, usize)] = &[
        (0,2),(1,2),(2,2),(3,2),(4,2),(5,2),(6,2),(7,2),(8,2),(9,2),     // 0-9: QPSK
        (10,4),(11,4),(12,4),(13,4),(14,4),(15,4),(16,4),(17,4),(18,4),   // 10-18: 16QAM
        (19,4),(20,4),(21,4),(22,4),(23,4),(24,4),(25,4),(26,4),           // 19-26: 16QAM
    ];
    if mcs_idx >= TABLE.len() { return (0, 2); }
    TABLE[mcs_idx]
}

// ============================================================
// Coverage Enhancement (CE) Modes A/B
// ============================================================

/// CE Mode selection per 3GPP TS 36.321.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CeMode {
    /// CE Mode A: normal coverage, up to 32 repetitions
    ModeA,
    /// CE Mode B: extended coverage, up to 2048 repetitions
    ModeB,
}

impl CeMode {
    pub fn max_repetitions(self) -> u32 { match self { CeMode::ModeA => 32, CeMode::ModeB => 2048 } }

    /// Classify based on path loss threshold (≤155.7 dB → Mode A, else Mode B).
    pub fn from_path_loss(pl_db: f64) -> Self {
        if pl_db <= 155.7 { CeMode::ModeA } else { CeMode::ModeB }
    }
}

/// MRC repetition combining: average N copies of a signal.
pub fn repetition_combine(copies: &[Vec<Complex>]) -> Vec<Complex> {
    if copies.is_empty() { return Vec::new(); }
    let len = copies[0].len();
    let mut acc = vec![Complex::zero(); len];
    for c in copies { for (i, s) in c.iter().enumerate() { acc[i] = acc[i].add(s); } }
    let n = copies.len() as f64;
    acc.iter().map(|s| s.scale(1.0 / n)).collect()
}

// ============================================================
// HARQ Process Manager
// ============================================================

/// State of a single HARQ process.
#[derive(Clone, Debug, PartialEq)]
pub enum HarqState {
    Idle,
    Pending { rv: u8, ndi: bool, retx_count: u8 },
    Acknowledged,
}

/// A single HARQ process with soft buffer.
#[derive(Clone, Debug)]
pub struct HarqProcess {
    pub pid: u8,
    pub state: HarqState,
    pub soft_buf: Vec<i8>,
    pub tbs: usize,
}

impl HarqProcess {
    pub fn new(pid: u8, tbs: usize) -> Self {
        Self { pid, state: HarqState::Idle, soft_buf: vec![0i8; 3 * tbs], tbs }
    }

    /// Schedule a new TX or retransmission.
    pub fn schedule_tx(&mut self, rv: u8, ndi: bool) {
        if ndi { self.soft_buf.fill(0); } // new data → flush
        self.state = HarqState::Pending { rv, ndi, retx_count: 0 };
    }

    /// Accumulate soft bits (HARQ combining).
    pub fn accumulate(&mut self, soft: &[i8]) {
        let len = self.soft_buf.len().min(soft.len());
        for i in 0..len {
            self.soft_buf[i] = (self.soft_buf[i] as i16 + soft[i] as i16).clamp(-127, 127) as i8;
        }
    }

    /// Positive acknowledgement.
    pub fn acknowledge(&mut self) { self.state = HarqState::Acknowledged; }

    /// Negative acknowledgement — increment RV.
    pub fn nack(&mut self) {
        if let HarqState::Pending { rv, retx_count, .. } = &mut self.state {
            *retx_count += 1;
            *rv = (*rv + 1) % 4;
        }
    }

    /// Reset for reuse.
    pub fn reset(&mut self) { self.state = HarqState::Idle; self.soft_buf.fill(0); }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HarqDirection { DL, UL }

/// HARQ manager for all DL or UL processes.
pub struct HarqManager {
    pub processes: Vec<HarqProcess>,
    pub direction: HarqDirection,
}

impl HarqManager {
    pub fn new(dir: HarqDirection, tbs: usize) -> Self {
        let n = if dir == HarqDirection::DL { MAX_HARQ_DL } else { MAX_HARQ_UL };
        Self { processes: (0..n).map(|pid| HarqProcess::new(pid as u8, tbs)).collect(), direction: dir }
    }

    /// Return PID of first idle process, if any.
    pub fn next_available(&self) -> Option<u8> {
        self.processes.iter().find(|p| p.state == HarqState::Idle).map(|p| p.pid)
    }

    /// Count currently pending processes.
    pub fn active_count(&self) -> usize {
        self.processes.iter().filter(|p| matches!(p.state, HarqState::Pending { .. })).count()
    }
}

// ============================================================
// Narrowband Frequency Hopping (Type 2)
// ============================================================

/// Frequency hopping configuration per 3GPP TS 36.211 §6.3.4.
pub struct FreqHopping {
    pub cell_id: u16,
    pub sys_bw_prb: usize,
    pub current_nb: usize,
}

impl FreqHopping {
    pub fn new(cell_id: u16, sys_bw_prb: usize) -> Self {
        Self { cell_id, sys_bw_prb, current_nb: 0 }
    }

    /// Number of 6-PRB narrowbands in system bandwidth.
    pub fn n_narrowbands(&self) -> usize { self.sys_bw_prb / N_PRB }

    /// Hopping offset for subframe n_f:
    /// offset = (N_cell_ID × ⌊n_f/2⌋ + n_f) mod N_NB
    pub fn hop_offset(&self, n_f: u32) -> usize {
        let n_nb = self.n_narrowbands();
        if n_nb == 0 { return 0; }
        ((self.cell_id as u64 * (n_f as u64 / 2) + n_f as u64) % n_nb as u64) as usize
    }

    /// Narrowband index after one hop.
    pub fn next_nb(&self, n_f: u32) -> usize {
        let n_nb = self.n_narrowbands();
        if n_nb == 0 { return self.current_nb; }
        (self.current_nb + self.hop_offset(n_f) + 1) % n_nb
    }

    /// First PRB index for narrowband `nb_idx`.
    pub fn nb_prb_offset(nb_idx: usize) -> usize { nb_idx * N_PRB }
}

// ============================================================
// Wake-Up Signal (WUS)
// ============================================================

/// WUS configuration per 3GPP TS 36.211 §6.11.3.
pub struct WusConfig {
    pub rnti: u16,
    pub n_wus_sym: usize,
    pub periodicity_sf: u32,
}

impl WusConfig {
    pub fn new(rnti: u16, n_wus_sym: usize, periodicity_sf: u32) -> Self {
        Self { rnti, n_wus_sym, periodicity_sf }
    }

    /// Generate RNTI-scrambled PN reference sequence (BPSK, unit magnitude).
    pub fn generate_wus_sequence(&self, cell_id: u16) -> Vec<Complex> {
        let seed = (self.rnti as u32) ^ ((cell_id as u32) << 16);
        let len = self.n_wus_sym * N_SC;
        let mut out = Vec::with_capacity(len);
        // Galois LFSR: x^31 + x^28 + 1
        let mut lfsr = (seed | 1) as u64;
        for _ in 0..len {
            let bit = (lfsr ^ (lfsr >> 3)) & 1;
            lfsr = (lfsr >> 1) | (bit << 30);
            out.push(Complex::from_polar(1.0, if bit == 0 { 0.0 } else { PI }));
        }
        out
    }

    /// Detect WUS by cross-correlating received signal against reference.
    /// Returns normalised correlation value (0.0–1.0).
    pub fn detect_wus(&self, received: &[Complex], reference: &[Complex]) -> f64 {
        let n = reference.len().min(received.len());
        if n == 0 { return 0.0; }
        let mut corr = Complex::zero();
        let mut ref_power = 0.0f64;
        for i in 0..n {
            corr = corr.add(&received[i].mul(&reference[i].conjugate()));
            ref_power += reference[i].magnitude_sq();
        }
        if ref_power < 1e-12 { return 0.0; }
        corr.magnitude() / ref_power.sqrt()
    }
}

// ============================================================
// eDRX and PSM Power Saving
// ============================================================

/// eDRX configuration (3GPP TS 24.008).
#[derive(Clone, Debug)]
pub struct EDrxConfig {
    pub edrx_cycle_s: f64,
    pub ptw_s: f64,
    pub hyperframe: u8,
}

impl EDrxConfig {
    /// Map 4-bit eDRX value to cycle length in seconds (LTE-M values).
    pub fn cycle_from_value(val: u8) -> f64 {
        const CYCLES: &[f64] = &[
            5.12, 10.24, 20.48, 40.96, 61.44, 81.92, 102.4, 122.88,
            143.36, 163.84, 327.68, 655.36, 1310.72, 2621.44, 5242.88, 10485.76,
        ];
        CYCLES[(val as usize).min(15)]
    }

    /// Fraction of time saved by eDRX compared to always-on.
    pub fn power_saving_ratio(&self, active_fraction: f64) -> f64 {
        let ptw_ratio = (self.ptw_s / self.edrx_cycle_s).min(1.0);
        let sleep_fraction = 1.0 - ptw_ratio;
        sleep_fraction * (1.0 - active_fraction)
    }

    /// Paging opportunity offset (seconds from cycle start) for given RNTI.
    pub fn paging_opportunity(&self, rnti: u16) -> f64 {
        let pf_idx = (rnti / 1024) % 16;
        pf_idx as f64 / 16.0 * self.ptw_s
    }
}

/// PSM configuration.
#[derive(Clone, Debug)]
pub struct PsmConfig {
    pub t3324_s: f64,  // Active timer
    pub t3412_s: f64,  // Extended TAU timer
}

impl PsmConfig {
    /// Decode T3324 timer from encoded 8-bit value (3GPP TS 24.008 Table 10.5.163a).
    pub fn t3324_from_value(val: u8) -> f64 {
        let unit  = (val >> 5) & 0x07;
        let count = (val & 0x1F) as f64;
        match unit {
            0b000 => count * 2.0,
            0b001 => count * 60.0,
            0b010 => count * 360.0,
            0b111 => 0.0,
            _     => count * 60.0,
        }
    }

    /// Fraction of time spent in deep sleep (PSM duty cycle).
    pub fn duty_cycle(&self) -> f64 {
        if self.t3412_s <= 0.0 { return 0.0; }
        ((self.t3412_s - self.t3324_s) / self.t3412_s).clamp(0.0, 1.0)
    }
}

// ============================================================
// Half-Duplex FDD Scheduling
// ============================================================

/// Half-duplex FDD timing parameters.
pub struct HalfDuplexFdd {
    pub timing_advance_sf: u32,
    pub tx_rx_guard_sf: u32,
    pub rx_tx_guard_sf: u32,
}

impl HalfDuplexFdd {
    pub fn new(ta_sf: u32) -> Self {
        Self { timing_advance_sf: ta_sf, tx_rx_guard_sf: 1, rx_tx_guard_sf: 1 }
    }

    /// UL subframe number given DL grant subframe.
    /// UL = (DL_sf + 4 + TA + tx_rx_guard) mod period.
    pub fn ul_subframe(&self, dl_sf: u32, period: u32) -> u32 {
        (dl_sf + 4 + self.timing_advance_sf + self.tx_rx_guard_sf) % period
    }

    /// True if enough time has elapsed since last DL activity to switch to UL.
    pub fn can_switch_to_ul(&self, current_sf: u32, last_dl_sf: u32) -> bool {
        current_sf >= last_dl_sf + self.tx_rx_guard_sf + 1
    }

    /// Throughput reduction factor due to guard periods.
    pub fn throughput_reduction(&self, subframes_per_cycle: u32) -> f64 {
        let guard = self.tx_rx_guard_sf + self.rx_tx_guard_sf + self.timing_advance_sf;
        1.0 - (guard as f64 / subframes_per_cycle as f64).min(1.0)
    }
}

// ============================================================
// VRB-to-PRB mapping (MPDCCH)
// ============================================================

/// VRB-to-PRB mapping mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VrbPrbMapping { Localized, Distributed }

/// Map VRBs to PRBs within the narrowband.
pub fn vrb_to_prb(vrb_indices: &[usize], mapping: VrbPrbMapping, cell_id: u16, sf: u32) -> Vec<usize> {
    match mapping {
        VrbPrbMapping::Localized => vrb_indices.to_vec(),
        VrbPrbMapping::Distributed => {
            let n_nb = N_PRB;
            let seed = (cell_id as u32 ^ sf) as usize;
            vrb_indices.iter().map(|&v| (v + seed) % n_nb).collect()
        }
    }
}

// ============================================================
// CRC-24A and CRC-16 CCITT
// ============================================================

/// CRC-24A per 3GPP TS 36.212 §5.1.1 (polynomial 0x864CFB).
pub fn crc24a(data: &[u8]) -> u32 {
    const POLY: u32 = 0x864C_FB;
    let mut crc: u32 = 0;
    for &byte in data {
        for i in (0..8usize).rev() {
            let bit = ((byte >> i) & 1) as u32;
            let msb = (crc >> 23) & 1;
            crc = ((crc << 1) | bit) & 0xFF_FFFF;
            if msb != 0 { crc ^= POLY; }
        }
    }
    crc & 0xFF_FFFF
}

/// CRC-16 CCITT (polynomial 0x1021, init 0xFFFF).
pub fn crc16(data: &[u8]) -> u16 {
    const POLY: u16 = 0x1021;
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ POLY } else { crc << 1 };
        }
    }
    crc
}

// ============================================================
// SPS for VoLTE
// ============================================================

/// Semi-Persistent Scheduling configuration for VoLTE.
pub struct SpsConfig {
    pub periodicity_sf: u32,
    pub c_rnti: u16,
    pub n_ul_processes: u8,
    pub voice_tbs_bytes: usize,
}

impl SpsConfig {
    /// Typical VoLTE preset (AMR 12.65 kbps, 20 ms frames).
    pub fn new_volte(c_rnti: u16) -> Self {
        Self { periodicity_sf: 20, c_rnti, n_ul_processes: 2, voice_tbs_bytes: 32 }
    }

    /// True if `sf_num` is an SPS transmission opportunity.
    pub fn is_sps_subframe(&self, sf_num: u32) -> bool { sf_num % self.periodicity_sf == 0 }

    /// Signalling overhead percentage avoided by SPS vs dynamic DCI scheduling.
    pub fn overhead_reduction_percent(&self) -> f64 {
        let dci_bits = 27.0;
        let voice_bits = (self.voice_tbs_bytes * 8) as f64;
        dci_bits / (voice_bits + dci_bits) * 100.0
    }
}

// ============================================================
// Link Budget
// ============================================================

/// LTE-M simplified link budget.
pub struct LteMBudget {
    pub tx_eirp_dbm: f64,
    pub noise_density_dbm: f64,
    pub rx_nf_db: f64,
    pub min_snr_db: f64,
    pub bw_hz: f64,
}

impl LteMBudget {
    /// Standard urban Cat-M1 scenario.
    pub fn standard_urban() -> Self {
        Self {
            tx_eirp_dbm:      23.0,
            noise_density_dbm: -174.0,
            rx_nf_db:           9.0,
            min_snr_db:        -6.0,
            bw_hz:              1.4e6,
        }
    }

    /// Maximum Coupling Loss [dB] = TX_EIRP − (noise_floor + min_SNR).
    pub fn mcl_db(&self) -> f64 {
        let noise_floor = self.noise_density_dbm
            + 10.0 * (self.bw_hz).log10()
            + self.rx_nf_db;
        self.tx_eirp_dbm - (noise_floor + self.min_snr_db)
    }

    /// Repetition gain [dB] = 10 log10(N_repetitions).
    pub fn ce_gain_db(n_rep: u32) -> f64 { 10.0 * (n_rep as f64).log10() }

    /// MCL with CE Mode B (2048 repetitions).
    pub fn mcl_db_ce_mode_b(&self) -> f64 { self.mcl_db() + Self::ce_gain_db(2048) }
}

// ============================================================
// MPDCCH Aggregation Levels
// ============================================================

/// MPDCCH aggregation level.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(usize)]
pub enum AggregationLevel { AL2 = 2, AL4 = 4, AL8 = 8, AL16 = 16, AL24 = 24 }

impl AggregationLevel {
    pub fn n_cce(self)      -> usize { self as usize }
    pub fn n_reg(self)      -> usize { self.n_cce() * 9 }
    /// QPSK-coded bits in MPDCCH at this AL.
    pub fn coded_bits(self) -> usize { self.n_reg() * 4 * 2 }
}

// ============================================================
// Full LTE-M Processor
// ============================================================

/// Top-level LTE-M processor state.
pub struct LteMProcessor {
    pub cell_id: u16,
    pub n_id_1:  u16,
    pub n_id_2:  u8,
    pub ce_mode: CeMode,
    pub harq_dl: HarqManager,
    pub harq_ul: HarqManager,
    pub sps:     Option<SpsConfig>,
    pub freq_hopping: Option<FreqHopping>,
    pub hd_fdd:  HalfDuplexFdd,
    pub edrx:    Option<EDrxConfig>,
    pub psm:     Option<PsmConfig>,
    pub sys_bw_prb: usize,
    pub subframe_counter: u32,
}

impl LteMProcessor {
    pub fn new(pcid: u16, sys_bw_prb: usize, tbs: usize) -> Self {
        Self {
            cell_id:    pcid,
            n_id_2:     (pcid % 3) as u8,
            n_id_1:      pcid / 3,
            ce_mode:    CeMode::ModeA,
            harq_dl:    HarqManager::new(HarqDirection::DL, tbs),
            harq_ul:    HarqManager::new(HarqDirection::UL, tbs),
            sps:        None,
            freq_hopping: None,
            hd_fdd:     HalfDuplexFdd::new(0),
            edrx:       None,
            psm:        None,
            sys_bw_prb,
            subframe_counter: 0,
        }
    }

    pub fn enable_freq_hopping(&mut self) {
        self.freq_hopping = Some(FreqHopping::new(self.cell_id, self.sys_bw_prb));
    }

    pub fn enable_sps(&mut self, c_rnti: u16) {
        self.sps = Some(SpsConfig::new_volte(c_rnti));
    }

    pub fn configure_edrx(&mut self, cycle_val: u8, ptw_s: f64) {
        self.edrx = Some(EDrxConfig {
            edrx_cycle_s: EDrxConfig::cycle_from_value(cycle_val),
            ptw_s,
            hyperframe: 0,
        });
    }

    pub fn configure_psm(&mut self, t3324_val: u8, t3412_s: f64) {
        self.psm = Some(PsmConfig {
            t3324_s: PsmConfig::t3324_from_value(t3324_val),
            t3412_s,
        });
    }

    /// Advance one subframe; returns (CeMode, total active HARQ processes).
    pub fn tick(&mut self) -> (CeMode, usize) {
        let active = self.harq_dl.active_count() + self.harq_ul.active_count();
        self.subframe_counter = (self.subframe_counter + 1)
            % (SF_PER_FRAME as u32 * 1024);
        (self.ce_mode, active)
    }

    pub fn pss(&self) -> Vec<Complex> { generate_pss(pss_root(self.n_id_2)) }
    pub fn sss(&self, subframe: u8)  -> Vec<i8>  { generate_sss(self.n_id_1, self.n_id_2, subframe) }

    /// Estimate MCL [dB] with N repetitions.
    pub fn mcl_estimate_db(&self, n_rep: u32) -> f64 {
        let b = LteMBudget::standard_urban();
        b.mcl_db() + LteMBudget::ce_gain_db(n_rep)
    }
}

// ============================================================
// Unit Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    // --------------------------------------------------------
    // 1. FFT correctness
    // --------------------------------------------------------
    #[test]
    fn test_fft_dc_tone() {
        let n = 128;
        let input = vec![Complex::new(1.0, 0.0); n];
        let out = fft(&input);
        assert!((out[0].re - n as f64).abs() < 1e-8, "DC bin = N");
        for k in 1..n { assert!(out[k].magnitude() < 1e-6, "non-DC should be ~0"); }
    }

    #[test]
    fn test_fft_ifft_roundtrip() {
        let n = 64;
        let input: Vec<Complex> = (0..n).map(|i| Complex::new(i as f64 * 0.1, i as f64 * 0.05)).collect();
        let freq = fft(&input);
        let rec  = ifft(&freq);
        for (a, b) in input.iter().zip(rec.iter()) {
            assert!((a.re - b.re).abs() < 1e-8);
            assert!((a.im - b.im).abs() < 1e-8);
        }
    }

    #[test]
    fn test_fft_single_tone() {
        let n = 32;
        let k_tone = 3usize;
        let input: Vec<Complex> = (0..n).map(|i| {
            Complex::from_polar(1.0, 2.0 * PI * k_tone as f64 * i as f64 / n as f64)
        }).collect();
        let out = fft(&input);
        let peak = out.iter().enumerate().max_by(|a, b| a.1.magnitude().partial_cmp(&b.1.magnitude()).unwrap()).unwrap().0;
        assert_eq!(peak, k_tone);
    }

    // --------------------------------------------------------
    // 2. Subcarrier → FFT bin mapping
    // --------------------------------------------------------
    #[test]
    fn test_sc_to_bin_lower() {
        assert_eq!(sc_to_bin(0),  1);
        assert_eq!(sc_to_bin(35), 36);
    }

    #[test]
    fn test_sc_to_bin_upper() {
        // Upper half: bin = FFT_SIZE - N_SC + sc = 128 - 72 + sc
        // sc=36 → 92, sc=71 → 127
        assert_eq!(sc_to_bin(36), 92);
        assert_eq!(sc_to_bin(71), 127);
    }

    #[test]
    fn test_sc_to_bin_no_dc() {
        // DC is bin 0; no subcarrier should map there
        for sc in 0..N_SC { assert_ne!(sc_to_bin(sc), 0, "SC {} maps to DC bin", sc); }
    }

    #[test]
    fn test_sc_to_bin_in_bounds() {
        for sc in 0..N_SC {
            let bin = sc_to_bin(sc);
            assert!(bin < FFT_SIZE, "bin {} out of range for sc {}", bin, sc);
        }
    }

    // --------------------------------------------------------
    // 3. OFDM modulate/demodulate
    // --------------------------------------------------------
    #[test]
    fn test_ofdm_sample_count() {
        let grid = vec![vec![Complex::zero(); N_SC]; SYMS_PER_SF];
        let td = ofdm_modulate(&grid);
        // 2 slots × [(10+128) + 6×(9+128)] = 2 × [138 + 822] = 2 × 960 = 1920
        let per_slot = (CP_FIRST_LEN + FFT_SIZE) + 6 * (CP_NORMAL_LEN + FFT_SIZE);
        assert_eq!(td.len(), 2 * per_slot, "Expected {} samples, got {}", 2 * per_slot, td.len());
    }

    #[test]
    fn test_ofdm_roundtrip() {
        let freq_grid: Vec<Vec<Complex>> = (0..SYMS_PER_SF).map(|sym| {
            (0..N_SC).map(|sc| Complex::from_polar(1.0, (sym * 7 + sc) as f64 * 0.3)).collect()
        }).collect();
        let td  = ofdm_modulate(&freq_grid);
        let rec = ofdm_demodulate(&td);
        for (s, r) in freq_grid.iter().zip(rec.iter()) {
            for (a, b) in s.iter().zip(r.iter()) {
                assert!((a.re - b.re).abs() < 1e-5, "OFDM roundtrip re mismatch");
                assert!((a.im - b.im).abs() < 1e-5, "OFDM roundtrip im mismatch");
            }
        }
    }

    #[test]
    fn test_ofdm_constants() {
        assert_eq!(N_SC, 72);
        assert_eq!(FFT_SIZE, 128);
        assert_eq!(SYMS_PER_SF, 14);
    }

    // --------------------------------------------------------
    // 4. PSS
    // --------------------------------------------------------
    #[test]
    fn test_pss_length() {
        for &r in &[25u32, 29, 34] { assert_eq!(generate_pss(r).len(), 62); }
    }

    #[test]
    fn test_pss_unit_magnitude() {
        for &r in &[25u32, 29, 34] {
            for s in generate_pss(r) { assert!((s.magnitude() - 1.0).abs() < 1e-10); }
        }
    }

    #[test]
    fn test_pss_roots_differ() {
        let p25 = generate_pss(25);
        let p29 = generate_pss(29);
        assert!(p25.iter().zip(p29.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-10));
    }

    #[test]
    fn test_pss_root_mapping() {
        assert_eq!(pss_root(0), 25);
        assert_eq!(pss_root(1), 29);
        assert_eq!(pss_root(2), 34);
    }

    #[test]
    fn test_pss_auto_correlation() {
        let pss = generate_pss(25);
        let (off, corr) = pss_correlate(&pss, &pss);
        assert_eq!(off, 0);
        assert!(corr > 60.0, "PSS auto-correlation peak: {}", corr);
    }

    #[test]
    fn test_pss_detection_with_offset() {
        let pss = generate_pss(29);
        let offset_true = 10;
        let mut rx = vec![Complex::zero(); offset_true];
        rx.extend_from_slice(&pss);
        rx.extend(vec![Complex::zero(); 10]);
        let (found, _) = pss_correlate(&rx, &pss);
        assert_eq!(found, offset_true);
    }

    // --------------------------------------------------------
    // 5. SSS
    // --------------------------------------------------------
    #[test]
    fn test_sss_length() { assert_eq!(generate_sss(0, 0, 0).len(), 62); }

    #[test]
    fn test_sss_bpsk_values() {
        let sss = generate_sss(10, 1, 0);
        for &s in &sss { assert!(s == 1 || s == -1, "SSS value {} is not ±1", s); }
    }

    #[test]
    fn test_sss_subframe_differs() {
        let sf0 = generate_sss(5, 0, 0);
        let sf5 = generate_sss(5, 0, 5);
        assert!(sf0.iter().zip(sf5.iter()).any(|(a, b)| a != b));
    }

    #[test]
    fn test_cell_id_formula() {
        assert_eq!(cell_id(0, 0), 0);
        assert_eq!(cell_id(1, 0), 3);
        assert_eq!(cell_id(0, 2), 2);
        assert_eq!(cell_id(55, 2), 167);
    }

    // --------------------------------------------------------
    // 6. QPP interleaver
    // --------------------------------------------------------
    #[test]
    fn test_qpp_roundtrip() {
        let (f1, f2) = qpp_params(40).unwrap();
        let data: Vec<u8> = (0..40u8).collect();
        assert_eq!(qpp_deinterleave(&qpp_interleave(&data, f1, f2), f1, f2), data);
    }

    #[test]
    fn test_qpp_is_permutation() {
        let (f1, f2) = qpp_params(40).unwrap();
        let data: Vec<u8> = (0..40u8).collect();
        let mut interleaved = qpp_interleave(&data, f1, f2);
        interleaved.sort();
        assert_eq!(interleaved, data);
    }

    #[test]
    fn test_qpp_params_known() {
        let (f1, f2) = qpp_params(40).unwrap();
        assert_eq!(f1, 3);
        assert_eq!(f2, 10);
    }

    #[test]
    fn test_qpp_params_not_found() { assert!(qpp_params(41).is_none()); }

    // --------------------------------------------------------
    // 7. RSC encoder
    // --------------------------------------------------------
    #[test]
    fn test_rsc_encode_all_zeros() {
        let (sys, par) = rsc_encode(&vec![0u8; 8]);
        assert!(sys[..8].iter().all(|&b| b == 0));
        assert!(par[..8].iter().all(|&b| b == 0));
    }

    #[test]
    fn test_rsc_encode_tail_bits() {
        let (sys, par) = rsc_encode(&vec![1u8; 16]);
        assert_eq!(sys.len(), 19); // 16 + 3 tail
        assert_eq!(par.len(), 19);
    }

    #[test]
    fn test_rsc_encode_binary_only() {
        let (sys, par) = rsc_encode(&[0,1,0,1,1,0,0,1]);
        assert!(sys.iter().chain(par.iter()).all(|&b| b == 0 || b == 1));
    }

    // --------------------------------------------------------
    // 8. Turbo encoder
    // --------------------------------------------------------
    #[test]
    fn test_turbo_encoder_output_sizes() {
        let enc = TurboEncoder::new(40).unwrap();
        let (s, p1, p2) = enc.encode(&vec![0u8; 40]);
        assert_eq!(s.len(), 43); // 40 + 3 tail
        assert_eq!(p1.len(), 43);
        assert_eq!(p2.len(), 43);
    }

    #[test]
    fn test_turbo_mux_length() {
        let enc = TurboEncoder::new(40).unwrap();
        let (s, p1, p2) = enc.encode(&vec![1u8; 40]);
        assert_eq!(enc.multiplex(&s, &p1, &p2).len(), 43 * 3);
    }

    #[test]
    fn test_turbo_encoder_invalid_k() { assert!(TurboEncoder::new(41).is_none()); }

    // --------------------------------------------------------
    // 9. Rate matching
    // --------------------------------------------------------
    #[test]
    fn test_rate_matcher_extract_wrap() {
        // 6 encoded bits in circular buffer; extract 10 (wraps twice)
        let circ: Vec<u8> = vec![1, 0, 1, 1, 0, 0];
        let rm = RateMatcher::new(6, 0);
        let out = rm.extract(&circ, 10);
        assert_eq!(out.len(), 10);
        // Elements at index 6, 7 should wrap to circ[0], circ[1]
        assert_eq!(out[6], circ[0]);
        assert_eq!(out[7], circ[1]);
    }

    #[test]
    fn test_rate_matcher_rv_offsets() {
        let rm0 = RateMatcher::new(100, 0);
        let rm1 = RateMatcher::new(100, 1);
        let rm2 = RateMatcher::new(100, 2);
        let rm3 = RateMatcher::new(100, 3);
        assert_eq!(rm0.start_offset(), 0);
        assert_eq!(rm1.start_offset(), 25);
        assert_eq!(rm2.start_offset(), 50);
        assert_eq!(rm3.start_offset(), 75);
    }

    #[test]
    fn test_harq_soft_combining() {
        let a = vec![10i8, -5, 20, -30, 0];
        let b = vec![-8i8,  3, 15,  10, 5];
        let c = RateMatcher::combine(&a, &b);
        assert_eq!(c[0], 2);    // 10-8
        assert_eq!(c[1], -2);   // -5+3
        assert_eq!(c[2], 35);   // 20+15
        assert_eq!(c[3], -20);  // -30+10
    }

    // --------------------------------------------------------
    // 10. Modulation
    // --------------------------------------------------------
    #[test]
    fn test_qpsk_roundtrip() {
        let bits = vec![0u8,0, 0,1, 1,0, 1,1];
        assert_eq!(qpsk_demodulate(&qpsk_modulate(&bits)), bits);
    }

    #[test]
    fn test_qpsk_unit_power() {
        let syms = qpsk_modulate(&[0,0, 1,1, 0,1, 1,0]);
        for s in syms { assert!((s.magnitude_sq() - 1.0).abs() < 1e-10); }
    }

    #[test]
    fn test_qam16_output_count() {
        assert_eq!(qam16_modulate(&vec![0u8; 40]).len(), 10);
    }

    #[test]
    fn test_qam16_distinct_points() {
        let mut pts = std::collections::HashSet::new();
        for b0 in 0..2u8 { for b1 in 0..2 { for b2 in 0..2 { for b3 in 0..2 {
            let s = qam16_modulate(&[b0,b1,b2,b3])[0];
            pts.insert(((s.re * 1000.0) as i32, (s.im * 1000.0) as i32));
        }}}}
        assert_eq!(pts.len(), 16);
    }

    #[test]
    fn test_pi2_bpsk_unit_mag() {
        for s in pi2_bpsk_modulate(&[0,1,0,0,1,1,1,0]) {
            assert!((s.magnitude() - 1.0).abs() < 1e-10);
        }
    }

    // --------------------------------------------------------
    // 11. DCI 6-1A
    // --------------------------------------------------------
    #[test]
    fn test_dci_encode_decode_roundtrip() {
        let dci = Dci6_1A {
            resource_assignment: 0b111001, mcs: 7, harq_pid: 3,
            ndi: true, rv: 2, repetition: 4, bundling: false, tpc: 1, srs_req: true,
        };
        let dec = Dci6_1A::decode(&dci.encode()).unwrap();
        assert_eq!(dec.resource_assignment, dci.resource_assignment);
        assert_eq!(dec.mcs,                 dci.mcs);
        assert_eq!(dec.harq_pid,            dci.harq_pid);
        assert_eq!(dec.ndi,                 dci.ndi);
        assert_eq!(dec.rv,                  dci.rv);
        assert_eq!(dec.repetition,          dci.repetition);
        assert_eq!(dec.bundling,            dci.bundling);
        assert_eq!(dec.tpc,                 dci.tpc);
        assert_eq!(dec.srs_req,             dci.srs_req);
    }

    #[test]
    fn test_dci_repetition_count() {
        let mk = |rep| Dci6_1A { resource_assignment:0, mcs:0, harq_pid:0, ndi:false, rv:0,
                                  repetition: rep, bundling:false, tpc:0, srs_req:false };
        assert_eq!(mk(0).repetition_count(), 1);
        assert_eq!(mk(3).repetition_count(), 8);
        assert_eq!(mk(7).repetition_count(), 128);
    }

    #[test]
    fn test_dci_decode_too_short() { assert!(Dci6_1A::decode(&vec![0u8; 10]).is_none()); }

    // --------------------------------------------------------
    // 12. TBS table
    // --------------------------------------------------------
    #[test]
    fn test_tbs_lookup_values() {
        assert_eq!(tbs_lookup(0, 1), 16);
        assert_eq!(tbs_lookup(0, 6), 152);
        assert_eq!(tbs_lookup(10, 6), 1032);
    }

    #[test]
    fn test_tbs_invalid_returns_zero() {
        assert_eq!(tbs_lookup(100, 1), 0);
        assert_eq!(tbs_lookup(0, 0), 0);
        assert_eq!(tbs_lookup(0, 7), 0);
    }

    #[test]
    fn test_tbs_monotone_in_prb() {
        for i in 0..5 {
            for n in 1..6 { assert!(tbs_lookup(i, n) <= tbs_lookup(i, n + 1)); }
        }
    }

    #[test]
    fn test_mcs_to_mod() {
        let (_, m0) = mcs_to_tbs_mod(0);
        assert_eq!(m0, 2); // QPSK
        let (_, m10) = mcs_to_tbs_mod(10);
        assert_eq!(m10, 4); // 16QAM
    }

    // --------------------------------------------------------
    // 13. DFT spreading
    // --------------------------------------------------------
    #[test]
    fn test_dft_spread_length() {
        let s: Vec<Complex> = (0..16).map(|i| Complex::from_polar(1.0, i as f64)).collect();
        assert_eq!(dft_spread(&s).len(), 16);
    }

    #[test]
    fn test_dft_spread_idft_roundtrip() {
        // Use power-of-2 length so zero-padding does not interfere
        let syms: Vec<Complex> = (0..8).map(|i| Complex::new(i as f64 * 0.5, -(i as f64) * 0.3)).collect();
        let spread   = dft_spread(&syms);
        let despread = idft_despread(&spread);
        for (a, b) in syms.iter().zip(despread.iter()) {
            assert!((a.re - b.re).abs() < 1e-6, "DFT roundtrip re: {} vs {}", a.re, b.re);
            assert!((a.im - b.im).abs() < 1e-6, "DFT roundtrip im: {} vs {}", a.im, b.im);
        }
    }

    // --------------------------------------------------------
    // 14. Frequency hopping
    // --------------------------------------------------------
    #[test]
    fn test_freq_hopping_n_nb() {
        assert_eq!(FreqHopping::new(0, 25).n_narrowbands(), 4);
        assert_eq!(FreqHopping::new(0, 50).n_narrowbands(), 8);
    }

    #[test]
    fn test_hop_offset_in_range() {
        let fh = FreqHopping::new(42, 25);
        for sf in 0..100 { assert!(fh.hop_offset(sf) < fh.n_narrowbands()); }
    }

    #[test]
    fn test_hop_pattern_varies() {
        let fh = FreqHopping::new(1, 50);
        let offs: Vec<usize> = (0..20).map(|sf| fh.hop_offset(sf)).collect();
        assert!(!offs.windows(2).all(|w| w[0] == w[1]));
    }

    #[test]
    fn test_nb_prb_offset() {
        assert_eq!(FreqHopping::nb_prb_offset(0), 0);
        assert_eq!(FreqHopping::nb_prb_offset(2), 12);
    }

    // --------------------------------------------------------
    // 15. CE Mode A/B
    // --------------------------------------------------------
    #[test]
    fn test_ce_mode_max_rep() {
        assert_eq!(CeMode::ModeA.max_repetitions(), 32);
        assert_eq!(CeMode::ModeB.max_repetitions(), 2048);
    }

    #[test]
    fn test_ce_mode_from_pl() {
        assert_eq!(CeMode::from_path_loss(140.0), CeMode::ModeA);
        assert_eq!(CeMode::from_path_loss(160.0), CeMode::ModeB);
    }

    #[test]
    fn test_repetition_combining() {
        let copies = vec![vec![Complex::new(2.0, 0.0)]; 4];
        let c = repetition_combine(&copies);
        assert!((c[0].re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_repetition_combining_empty() {
        assert!(repetition_combine(&[]).is_empty());
    }

    // --------------------------------------------------------
    // 16. HARQ
    // --------------------------------------------------------
    #[test]
    fn test_harq_initial_state() {
        let p = HarqProcess::new(0, 50);
        assert_eq!(p.state, HarqState::Idle);
        assert_eq!(p.soft_buf.len(), 150); // 3 × 50
    }

    #[test]
    fn test_harq_ndi_flushes() {
        let mut p = HarqProcess::new(0, 10);
        p.soft_buf.fill(42);
        p.schedule_tx(0, true);
        assert!(p.soft_buf.iter().all(|&x| x == 0));
    }

    #[test]
    fn test_harq_nack_increments_rv() {
        let mut p = HarqProcess::new(1, 10);
        p.schedule_tx(0, true);
        p.nack();
        if let HarqState::Pending { rv, retx_count, .. } = p.state {
            assert_eq!(rv, 1); assert_eq!(retx_count, 1);
        } else { panic!("Wrong state"); }
    }

    #[test]
    fn test_harq_accumulate() {
        let mut p = HarqProcess::new(0, 5);
        p.soft_buf = vec![10, -5, 20, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        p.accumulate(&[-3i8, 7, -8, 15, 2]);
        assert_eq!(p.soft_buf[0], 7);
        assert_eq!(p.soft_buf[1], 2);
        assert_eq!(p.soft_buf[3], -15);
    }

    #[test]
    fn test_harq_manager_available() {
        let m = HarqManager::new(HarqDirection::DL, 100);
        assert_eq!(m.next_available().unwrap(), 0);
    }

    #[test]
    fn test_harq_manager_active_count() {
        let mut m = HarqManager::new(HarqDirection::DL, 100);
        m.processes[0].schedule_tx(0, true);
        m.processes[3].schedule_tx(0, false);
        assert_eq!(m.active_count(), 2);
    }

    // --------------------------------------------------------
    // 17. eDRX / PSM
    // --------------------------------------------------------
    #[test]
    fn test_edrx_cycles() {
        assert!((EDrxConfig::cycle_from_value(0) - 5.12).abs() < 0.01);
        assert!((EDrxConfig::cycle_from_value(4) - 61.44).abs() < 0.1);
        assert!((EDrxConfig::cycle_from_value(14) - 5242.88).abs() < 0.1);
    }

    #[test]
    fn test_edrx_power_saving() {
        let cfg = EDrxConfig { edrx_cycle_s: 10.24, ptw_s: 1.28, hyperframe: 0 };
        let r = cfg.power_saving_ratio(0.1);
        assert!(r > 0.5 && r < 1.0);
    }

    #[test]
    fn test_psm_t3324_decode() {
        // unit=000 (2-sec), count=5 → 10 s
        assert!((PsmConfig::t3324_from_value(0b000_00101) - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_psm_duty_cycle() {
        let p = PsmConfig { t3324_s: 60.0, t3412_s: 3600.0 };
        assert!((p.duty_cycle() - (3540.0 / 3600.0)).abs() < 0.01);
    }

    // --------------------------------------------------------
    // 18. WUS
    // --------------------------------------------------------
    #[test]
    fn test_wus_length() {
        let w = WusConfig::new(0x1234, 4, 100);
        assert_eq!(w.generate_wus_sequence(42).len(), 4 * N_SC);
    }

    #[test]
    fn test_wus_unit_mag() {
        let w = WusConfig::new(0xABCD, 2, 256);
        for s in w.generate_wus_sequence(7) { assert!((s.magnitude() - 1.0).abs() < 1e-10); }
    }

    #[test]
    fn test_wus_self_detection() {
        let w = WusConfig::new(0x5A5A, 4, 64);
        let r = w.generate_wus_sequence(42);
        assert!(w.detect_wus(&r, &r) > 0.9);
    }

    #[test]
    fn test_wus_differs_by_rnti() {
        let w1 = WusConfig::new(1, 2, 64);
        let w2 = WusConfig::new(2, 2, 64);
        let s1 = w1.generate_wus_sequence(0);
        let s2 = w2.generate_wus_sequence(0);
        assert!(s1.iter().zip(s2.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-10));
    }

    // --------------------------------------------------------
    // 19. HD-FDD
    // --------------------------------------------------------
    #[test]
    fn test_hdfdd_ul_sf() {
        let hd = HalfDuplexFdd::new(2);
        // (0 + 4 + 2 + 1) % 10 = 7
        assert_eq!(hd.ul_subframe(0, 10), 7);
    }

    #[test]
    fn test_hdfdd_switch() {
        let hd = HalfDuplexFdd::new(0); // guard=1
        assert!( hd.can_switch_to_ul(3, 1)); // 3 >= 1+1+1=3 ✓
        assert!(!hd.can_switch_to_ul(1, 1)); // 1 < 3 ✗
    }

    #[test]
    fn test_hdfdd_throughput_reduction() {
        let hd = HalfDuplexFdd::new(2); // TA=2, tx_rx=1, rx_tx=1 → guard=4
        let r = hd.throughput_reduction(20);
        assert!((r - 0.8).abs() < 0.01);
    }

    // --------------------------------------------------------
    // 20. CRC
    // --------------------------------------------------------
    #[test]
    fn test_crc24a_deterministic() {
        let data = vec![1u8, 2, 3, 4];
        assert_eq!(crc24a(&data), crc24a(&data));
        assert!(crc24a(&data) <= 0xFF_FFFF);
    }

    #[test]
    fn test_crc24a_detects_error() {
        let d = vec![1u8, 2, 3, 4, 5];
        let mut c = d.clone(); c[2] ^= 0xFF;
        assert_ne!(crc24a(&d), crc24a(&c));
    }

    #[test]
    fn test_crc16_deterministic() {
        let data = b"LTE-M";
        assert_eq!(crc16(data), crc16(data));
    }

    // --------------------------------------------------------
    // 21. SPS / VoLTE
    // --------------------------------------------------------
    #[test]
    fn test_sps_subframe_check() {
        let sps = SpsConfig::new_volte(0x1234);
        assert!( sps.is_sps_subframe(0));
        assert!( sps.is_sps_subframe(20));
        assert!(!sps.is_sps_subframe(10));
    }

    #[test]
    fn test_sps_overhead_positive() {
        let r = SpsConfig::new_volte(0).overhead_reduction_percent();
        assert!(r > 0.0 && r < 100.0);
    }

    // --------------------------------------------------------
    // 22. Link budget
    // --------------------------------------------------------
    #[test]
    fn test_mcl_reasonable_range() {
        let mcl = LteMBudget::standard_urban().mcl_db();
        // ~132 dB without CE (baseline before repetition gain)
        assert!(mcl > 120.0 && mcl < 150.0, "MCL = {} dB", mcl);
    }

    #[test]
    fn test_ce_gain_values() {
        assert!((LteMBudget::ce_gain_db(1) - 0.0).abs() < 0.01);
        assert!((LteMBudget::ce_gain_db(2) - 3.01).abs() < 0.1);
        assert!((LteMBudget::ce_gain_db(4) - 6.02).abs() < 0.1);
    }

    #[test]
    fn test_mcl_ce_mode_b_greater() {
        let b = LteMBudget::standard_urban();
        assert!(b.mcl_db_ce_mode_b() > b.mcl_db() + 30.0);
    }

    // --------------------------------------------------------
    // 23. Aggregation levels
    // --------------------------------------------------------
    #[test]
    fn test_al_cce_count() {
        assert_eq!(AggregationLevel::AL2.n_cce(), 2);
        assert_eq!(AggregationLevel::AL24.n_cce(), 24);
    }

    #[test]
    fn test_al_coded_bits() {
        // AL2: 2×9×4×2 = 144
        assert_eq!(AggregationLevel::AL2.coded_bits(), 144);
        assert_eq!(AggregationLevel::AL4.coded_bits(), 288);
    }

    // --------------------------------------------------------
    // 24. Top-level processor
    // --------------------------------------------------------
    #[test]
    fn test_processor_cell_id_decomposition() {
        let p = LteMProcessor::new(100, 25, 200);
        assert_eq!(p.cell_id, 100);
        assert_eq!(p.n_id_2, 1);  // 100 % 3
        assert_eq!(p.n_id_1, 33); // 100 / 3
    }

    #[test]
    fn test_processor_tick() {
        let mut p = LteMProcessor::new(0, 25, 100);
        let (mode, _) = p.tick();
        assert_eq!(mode, CeMode::ModeA);
        assert_eq!(p.subframe_counter, 1);
    }

    #[test]
    fn test_processor_pss_sss_lengths() {
        let p = LteMProcessor::new(0, 25, 100);
        assert_eq!(p.pss().len(), 62);
        assert_eq!(p.sss(0).len(), 62);
        assert_eq!(p.sss(5).len(), 62);
    }

    #[test]
    fn test_processor_enable_hopping() {
        let mut p = LteMProcessor::new(10, 50, 100);
        p.enable_freq_hopping();
        assert!(p.freq_hopping.is_some());
    }

    #[test]
    fn test_processor_edrx() {
        let mut p = LteMProcessor::new(0, 25, 100);
        p.configure_edrx(4, 2.56); // value 4 = 61.44 s
        assert!((p.edrx.as_ref().unwrap().edrx_cycle_s - 61.44).abs() < 0.1);
    }

    #[test]
    fn test_processor_psm() {
        let mut p = LteMProcessor::new(0, 25, 100);
        p.configure_psm(0b000_00101, 3600.0); // 10 s active
        let psm = p.psm.as_ref().unwrap();
        assert!((psm.t3324_s - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_processor_mcl_with_reps() {
        let p = LteMProcessor::new(0, 25, 100);
        assert!(p.mcl_estimate_db(32) > p.mcl_estimate_db(1) + 10.0);
    }
}

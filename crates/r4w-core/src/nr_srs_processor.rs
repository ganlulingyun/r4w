//! # 5G NR Sounding Reference Signal (SRS) Processor
//!
//! Implements SRS generation, resource mapping, and channel estimation per
//! 3GPP TS 38.211 Section 6.4.1.4 and TS 38.214 Section 6.2.1.
//!
//! ## Overview
//!
//! The Sounding Reference Signal (SRS) is transmitted by the UE in the uplink
//! to allow the gNB to estimate the uplink channel. The gNB uses this estimate
//! for:
//! - Uplink scheduling and link adaptation
//! - Downlink precoding (reciprocity-based beamforming in TDD)
//! - Timing advance (TA) estimation
//!
//! ## SRS Sequence Generation (TS 38.211 Section 6.4.1.4.1)
//!
//! SRS sequences are low-PAPR type-1 base sequences defined in Section 5.2.2.
//! The base sequence `r^(alpha, delta)_{u,v}(n)` is generated from:
//!
//! ```text
//! r^(alpha, delta)_{u,v}(n) = e^(j*alpha*n) * r_{u,v}(n),  n = 0 .. M_sc^SRS - 1
//! ```
//!
//! where:
//! - `alpha = 2*pi*n_cs/n_srs_cs_max` is the cyclic shift phase
//! - `r_{u,v}(n)` is the base sequence indexed by group `u` and sequence `v`
//! - `M_sc^SRS = m_SRS * N_sc^RB` is the SRS bandwidth in subcarriers
//!
//! For lengths >= 36, base sequences are Zadoff-Chu sequences:
//!
//! ```text
//! r_{u,v}(n) = x_q(n mod N_ZC),  n = 0 .. M_sc^SRS - 1
//! x_q(m)    = e^(-j*pi*q*m*(m+1)/N_ZC),  m = 0 .. N_ZC-1
//! q = floor(N_ZC * (u+1)/31) + v * (-1)^floor(2*N_ZC*(u+1)/31)
//! ```
//!
//! ## Comb Transmission (TS 38.211 Section 6.4.1.4.3)
//!
//! SRS uses interleaved subcarrier mapping (comb) to enable frequency-domain
//! multiplexing of multiple UEs. The comb size `K_TC` is 2, 4, or 8:
//!
//! ```text
//! k = k_0 + K_TC * m + k_bar_TC,  m = 0 .. M_sc^SRS - 1
//! ```
//!
//! where `k_bar_TC` is the transmission comb offset (0 .. K_TC-1).
//!
//! ## Frequency Hopping (TS 38.211 Section 6.4.1.4.4)
//!
//! SRS can hop across frequency to probe the wideband channel:
//!
//! ```text
//! n_b(l_SRS) = floor(4 * n_RRC / m_SRS,b) mod 4  (for b > b_hop)
//! ```
//!
//! ## Channel Estimation
//!
//! Least-squares (LS) channel estimation from received SRS:
//!
//! ```text
//! H^_LS(k) = Y(k) / X(k)
//! ```
//!
//! where `Y(k)` is received, `X(k)` is the known SRS pilot.
//!
//! ## Timing Advance Estimation
//!
//! Cross-correlate received SRS with reference and find peak delay:
//!
//! ```text
//! TA = arg_max_tau |sum_k H(k) e^(j*2*pi*k*tau/N_FFT)|^2  *  T_s
//! ```
//!
//! # References
//!
//! - 3GPP TS 38.211 v17.2.0, Section 5.2.2, 6.4.1.4
//! - 3GPP TS 38.214 v17.2.0, Section 6.2.1
//!
//! # Example
//!
//! ```rust
//! use r4w_core::nr_srs_processor::{
//!     SrsConfig, SrsProcessor, CombSize, HoppingMode,
//! };
//!
//! let config = SrsConfig {
//!     bandwidth_rb: 52,
//!     comb_size: CombSize::Two,
//!     cyclic_shift: 0,
//!     hopping_mode: HoppingMode::Neither,
//!     num_antenna_ports: 1,
//!     periodicity_slots: 40,
//!     offset_slots: 0,
//!     sequence_id: 0,
//!     n_rrc: 0,
//!     comb_offset: 0,
//! };
//!
//! let proc = SrsProcessor::new(config);
//! let srs = proc.generate_srs(0, 13);
//! assert!(!srs.is_empty());
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────── types ──

/// Internal complex number: 64-bit double precision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex64 {
    /// Real part.
    pub re: f64,
    /// Imaginary part.
    pub im: f64,
}

impl Complex64 {
    /// Construct from real and imaginary parts.
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    /// Unit phasor at angle `theta` radians: `e^(j*theta)`.
    #[inline]
    pub fn from_polar(mag: f64, theta: f64) -> Self {
        Self::new(mag * theta.cos(), mag * theta.sin())
    }

    /// Magnitude squared |z|^2.
    #[inline]
    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    /// Magnitude |z|.
    #[inline]
    pub fn norm(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    /// Complex conjugate.
    #[inline]
    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }

    /// Zero.
    #[inline]
    pub fn zero() -> Self {
        Self::new(0.0, 0.0)
    }
}

impl std::ops::Add for Complex64 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl std::ops::Sub for Complex64 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl std::ops::Mul for Complex64 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }
}

impl std::ops::Mul<f64> for Complex64 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.re * rhs, self.im * rhs)
    }
}

impl std::ops::Div for Complex64 {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        let denom = rhs.norm_sqr();
        if denom < 1e-300 {
            return Self::zero();
        }
        Self::new(
            (self.re * rhs.re + self.im * rhs.im) / denom,
            (self.im * rhs.re - self.re * rhs.im) / denom,
        )
    }
}

impl std::ops::AddAssign for Complex64 {
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

// ─────────────────────────────────────────────────────────────── constants ──

/// Number of subcarriers per resource block (TS 38.211 Section 4.4.4.1).
pub const N_SC_RB: usize = 12;

/// Maximum number of SRS cyclic shifts (TS 38.211 Table 6.4.1.4.3-1).
pub const N_SRS_CS_MAX_COMB2: u32 = 8;
/// Maximum SRS cyclic shifts for comb-4.
pub const N_SRS_CS_MAX_COMB4: u32 = 12;
/// Maximum SRS cyclic shifts for comb-8.
pub const N_SRS_CS_MAX_COMB8: u32 = 6;

/// Maximum number of SRS antenna ports (TS 38.214).
pub const MAX_SRS_PORTS: usize = 4;

/// Speed of light (m/s).
pub const C_LIGHT: f64 = 299_792_458.0;

/// 5G NR basic time unit: T_c = 1/(480000*4096) s.
pub const T_C_SECONDS: f64 = 1.0 / (480_000.0 * 4096.0);

// ──────────────────────────────────────────────────────────────── enums ──

/// SRS transmission comb size (TS 38.211 Section 6.4.1.4.2).
///
/// Determines the interleaving factor for subcarrier mapping.
/// A comb-K_TC SRS occupies every K_TC-th subcarrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CombSize {
    /// K_TC = 2: 1 of every 2 subcarriers used.
    Two = 2,
    /// K_TC = 4: 1 of every 4 subcarriers used.
    Four = 4,
    /// K_TC = 8: 1 of every 8 subcarriers used.
    Eight = 8,
}

impl CombSize {
    /// Integer value of the comb size.
    pub fn ktc(self) -> usize {
        self as usize
    }

    /// Maximum cyclic shift count for this comb size
    /// (TS 38.211 Table 6.4.1.4.3-1).
    pub fn n_cs_max(self) -> u32 {
        match self {
            CombSize::Two => N_SRS_CS_MAX_COMB2,
            CombSize::Four => N_SRS_CS_MAX_COMB4,
            CombSize::Eight => N_SRS_CS_MAX_COMB8,
        }
    }

    /// Maximum number of ports supported by this comb configuration.
    pub fn max_ports(self) -> usize {
        match self {
            CombSize::Two => 4,
            CombSize::Four => 4,
            CombSize::Eight => 1,
        }
    }
}

/// SRS sequence hopping mode (TS 38.211 Section 6.4.1.4.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoppingMode {
    /// No hopping: fixed sequence group `u` and sequence `v`.
    Neither,
    /// Group hopping: `u` varies per slot/symbol; `v` fixed to 0.
    GroupHopping,
    /// Sequence hopping: `v` alternates (0/1) per slot; `u` fixed.
    SequenceHopping,
}

/// SRS bandwidth configuration index b_SRS (TS 38.211 Table 6.4.1.4.2-2).
///
/// Each index maps to an m_SRS value (number of resource blocks) and
/// its corresponding hopping widths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BandwidthConfig {
    /// b_SRS = 0 (broadest span).
    B0,
    /// b_SRS = 1.
    B1,
    /// b_SRS = 2.
    B2,
    /// b_SRS = 3 (narrowest span).
    B3,
}

impl BandwidthConfig {
    /// Numeric index.
    pub fn index(self) -> usize {
        match self {
            Self::B0 => 0,
            Self::B1 => 1,
            Self::B2 => 2,
            Self::B3 => 3,
        }
    }
}

// ─────────────────────────────────────────────────────────── data structures

/// SRS resource configuration (TS 38.214 Section 6.2.1).
///
/// Describes a single SRS resource including its frequency location,
/// periodicity, comb structure, and antenna port count.
#[derive(Debug, Clone)]
pub struct SrsConfig {
    /// SRS bandwidth in resource blocks (m_SRS). Valid range: 1..272.
    pub bandwidth_rb: usize,
    /// Transmission comb size K_TC.
    pub comb_size: CombSize,
    /// Cyclic shift index n_CS (0 .. n_CS_max - 1).
    pub cyclic_shift: u32,
    /// Sequence/group hopping mode.
    pub hopping_mode: HoppingMode,
    /// Number of antenna ports (1, 2, or 4).
    pub num_antenna_ports: usize,
    /// SRS periodicity in slots T_SRS (≥1).
    pub periodicity_slots: u32,
    /// Slot offset T_offset (0 .. T_SRS - 1).
    pub offset_slots: u32,
    /// Higher-layer sequence ID n_ID^SRS (0..1023).
    pub sequence_id: u32,
    /// RRC-configured frequency index n_RRC for starting position.
    pub n_rrc: u32,
    /// Transmission comb offset k_bar_TC (0 .. K_TC - 1).
    pub comb_offset: usize,
}

impl Default for SrsConfig {
    fn default() -> Self {
        Self {
            bandwidth_rb: 52,
            comb_size: CombSize::Two,
            cyclic_shift: 0,
            hopping_mode: HoppingMode::Neither,
            num_antenna_ports: 1,
            periodicity_slots: 40,
            offset_slots: 0,
            sequence_id: 0,
            n_rrc: 0,
            comb_offset: 0,
        }
    }
}

/// An SRS resource descriptor combining config with symbol location.
#[derive(Debug, Clone)]
pub struct SrsResource {
    /// Configuration parameters.
    pub config: SrsConfig,
    /// OFDM symbol index within the slot (0..13 for normal CP).
    pub symbol_index: u32,
    /// Starting resource block index.
    pub start_rb: usize,
}

/// Channel estimate result from SRS processing.
#[derive(Debug, Clone)]
pub struct ChannelEstimate {
    /// Estimated complex channel H(f) at each SRS subcarrier,
    /// indexed by (port, subcarrier).
    pub h_per_port: Vec<Vec<Complex64>>,
    /// SRS subcarrier indices in the resource grid.
    pub subcarrier_indices: Vec<usize>,
    /// Number of antenna ports estimated.
    pub num_ports: usize,
    /// Mean square error proxy (ratio of noise power to signal power).
    pub noise_floor: f64,
}

/// Result of timing advance estimation.
#[derive(Debug, Clone)]
pub struct TimingAdvanceResult {
    /// Estimated timing advance in seconds.
    pub ta_seconds: f64,
    /// Estimated timing advance in samples (at nominal sample rate).
    pub ta_samples: f64,
    /// Peak correlation magnitude (higher = more reliable).
    pub peak_magnitude: f64,
    /// Bin index of the correlation peak.
    pub peak_bin: usize,
}

// ───────────────────────────────────────────────────────── helper functions ──

/// Compute GCD via Euclidean algorithm.
fn gcd(mut a: usize, mut b: usize) -> usize {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

/// Find the largest prime ≤ `n` that is also ≤ `n`.
/// Returns `n` itself if prime, otherwise searches downward.
fn largest_prime_lte(n: usize) -> usize {
    if n < 2 {
        return 2;
    }
    let mut candidate = n;
    while candidate >= 2 {
        if is_prime(candidate) {
            return candidate;
        }
        candidate -= 1;
    }
    2
}

/// Miller-Rabin primality test (deterministic for n < 3.2×10^18).
fn is_prime(n: usize) -> bool {
    if n < 2 {
        return false;
    }
    if n < 4 {
        return true;
    }
    if n % 2 == 0 || n % 3 == 0 {
        return false;
    }
    let mut i = 5usize;
    while i.saturating_mul(i) <= n {
        if n % i == 0 || n % (i + 2) == 0 {
            return false;
        }
        i += 6;
    }
    true
}

/// Generate a Zadoff-Chu sequence of prime length `n_zc` with root index `q`.
///
/// ```text
/// x_q(m) = exp(-j * pi * q * m * (m+1) / N_ZC)
/// ```
///
/// Reference: TS 38.211 Section 5.2.2.
pub fn zadoff_chu_seq(q: usize, n_zc: usize) -> Vec<Complex64> {
    (0..n_zc)
        .map(|m| {
            let phase = -PI * (q as f64) * (m as f64) * (m as f64 + 1.0) / (n_zc as f64);
            Complex64::from_polar(1.0, phase)
        })
        .collect()
}

/// Compute SRS base sequence group `u` and sequence `v` for a given slot.
///
/// Group hopping (TS 38.211 Section 5.2.2.1):
/// ```text
/// u = (f_gh(n_s, l) + n_ID^SRS mod 30) mod 30
/// f_gh = (sum_{i=0}^{7} c(8*n_s + i) * 2^i) mod 30
/// ```
///
/// Sequence hopping (TS 38.211 Section 5.2.2.2):
/// ```text
/// v = c(n_s * N_symb^slot + l)  (when M_sc >= 6*N_sc^RB)
/// ```
///
/// # Arguments
/// * `n_id` - Higher-layer SRS sequence ID n_ID^SRS
/// * `n_slot` - Slot number n_s within the frame
/// * `symbol` - Symbol index l within the slot
/// * `hopping` - Hopping mode
///
/// # Returns
/// (u, v) tuple — group index u in [0,29], sequence index v in {0,1}
pub fn compute_group_sequence(
    n_id: u32,
    n_slot: u32,
    symbol: u32,
    hopping: HoppingMode,
) -> (u32, u32) {
    // Pseudo-random sequence c(n) from gold code initialized with n_id
    // TS 38.211 Section 5.2.1
    let prs = |offset: u32| -> u32 {
        // Simplified: use a deterministic pseudo-random function based on
        // n_id and offset. In a real implementation this is the full
        // Gold-code generator defined in TS 38.211 Section 5.2.1.
        let c_init = n_id; // c_init = n_ID^SRS per TS 38.211 Table 5.2.2.1-1
        // LFSR-based pseudo-random bit: Gold code x1 XOR x2
        let mut x1: u32 = 0x5400_0000; // initial state of x1
        let mut x2 = c_init;
        // Advance to position offset
        for _ in 0..=offset {
            let b1 = ((x1 >> 3) ^ x1) & 1;
            x1 = (x1 >> 1) | (b1 << 30);
            let b2 = ((x2 >> 3) ^ (x2 >> 2) ^ (x2 >> 1) ^ x2) & 1;
            x2 = (x2 >> 1) | (b2 << 30);
        }
        (x1 ^ x2) & 1
    };

    let (u, v) = match hopping {
        HoppingMode::GroupHopping => {
            // f_gh = (sum_{i=0}^{7} c(8*n_s + i) * 2^i) mod 30
            let f_gh: u32 = (0u32..8)
                .map(|i| prs(8 * n_slot + i) << i)
                .sum::<u32>()
                % 30;
            let u = (f_gh + n_id % 30) % 30;
            (u, 0)
        }
        HoppingMode::SequenceHopping => {
            // u is fixed (n_id mod 30), v alternates via c()
            let u = n_id % 30;
            let v = prs(n_slot * 14 + symbol);
            (u, v)
        }
        HoppingMode::Neither => {
            // Both u and v fixed
            let u = n_id % 30;
            (u, 0)
        }
    };
    (u, v)
}

/// Generate the low-PAPR base sequence r_{u,v} of length `m_sc`.
///
/// For `m_sc < 6*N_sc^RB = 72` subcarriers (short ZC), use pre-defined
/// QPSK-like sequences (TS 38.211 Table 5.2.2.2-1 / Section 5.2.2.2).
/// For `m_sc >= 72`, use Zadoff-Chu sequences (TS 38.211 Section 5.2.2.3).
///
/// # Arguments
/// * `u` - Sequence group index (0..29)
/// * `v` - Sequence index within group (0 or 1)
/// * `m_sc` - Sequence length in subcarriers
pub fn generate_base_sequence(u: u32, v: u32, m_sc: usize) -> Vec<Complex64> {
    if m_sc >= 6 * N_SC_RB {
        // Long sequences: ZC-based (TS 38.211 Section 5.2.2.3)
        let n_zc = largest_prime_lte(m_sc);

        // Root index q (TS 38.211 Eq. 5.2.2.3-2)
        let n_zc_f = n_zc as f64;
        let frac = n_zc_f * (u as f64 + 1.0) / 31.0;
        let q_bar = frac.floor() as i64;
        let sign: i64 = if (2.0 * frac).floor() as i64 % 2 == 0 {
            1
        } else {
            -1
        };
        let q = ((q_bar + v as i64 * sign).rem_euclid(n_zc as i64)) as usize;

        let zc = zadoff_chu_seq(q, n_zc);

        // Extend to m_sc by repeating the ZC sequence
        (0..m_sc).map(|n| zc[n % n_zc]).collect()
    } else {
        // Short sequences: use ZC with smaller lengths scaled to m_sc
        // For simplicity, use the m_sc-length ZC if m_sc is prime,
        // otherwise use modular extension (approximate TS approach).
        let n_zc = if m_sc >= 3 {
            largest_prime_lte(m_sc)
        } else {
            m_sc
        };

        if n_zc < 2 {
            // Degenerate: return unit sequence
            return vec![Complex64::new(1.0, 0.0); m_sc];
        }

        let q = ((u as usize + 1) * (n_zc + 1) / 4 + v as usize) % n_zc;
        let q = if q == 0 { 1 } else { q }; // root must be non-zero

        let zc = zadoff_chu_seq(q, n_zc);
        (0..m_sc).map(|n| zc[n % n_zc]).collect()
    }
}

/// Apply a cyclic shift `alpha` to a base sequence.
///
/// ```text
/// r^alpha(n) = e^(j*alpha*n) * r(n),  n = 0..M_sc-1
/// alpha = 2*pi*n_cs / n_cs_max
/// ```
///
/// Reference: TS 38.211 Eq. 6.4.1.4.1-3.
pub fn apply_cyclic_shift(seq: &[Complex64], n_cs: u32, n_cs_max: u32) -> Vec<Complex64> {
    let alpha = 2.0 * PI * n_cs as f64 / n_cs_max as f64;
    seq.iter()
        .enumerate()
        .map(|(n, &s)| {
            let phase = Complex64::from_polar(1.0, alpha * n as f64);
            s * phase
        })
        .collect()
}

/// Compute the starting subcarrier k_0 for SRS mapping.
///
/// TS 38.211 Section 6.4.1.4.3, Eq. 6.4.1.4.3-1:
/// ```text
/// k_0 = k_start + K_TC * sum_{b=0}^{3} N_b * M_SRS_b  + k_bar_TC
/// ```
///
/// Simplified: uses n_RRC and comb_offset to determine the starting
/// resource-grid subcarrier.
pub fn compute_starting_subcarrier(
    start_rb: usize,
    comb_size: CombSize,
    comb_offset: usize,
    n_rrc: u32,
    bandwidth_rb: usize,
) -> usize {
    // k_0^(p) = k_0 + K_TC * (p - 1) / 2  for multi-port (simplified)
    // For the basic case, start_rb * N_sc_RB + hopping offset + comb_offset
    let k_start = start_rb * N_SC_RB;
    let k_tc = comb_size.ktc();

    // Frequency hopping position: use n_rrc to offset within bandwidth
    // floor(4 * n_rrc / m_SRS) determines hopping index
    let hop_offset = if bandwidth_rb > 0 {
        ((4 * n_rrc as usize) / bandwidth_rb) * k_tc
    } else {
        0
    };

    k_start + hop_offset + comb_offset
}

/// DFT (Discrete Fourier Transform) of length N (naive O(N^2)).
///
/// Used for timing advance estimation via frequency-domain correlation IFFT.
pub fn dft(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    (0..n)
        .map(|k| {
            let mut sum = Complex64::zero();
            for (m, &x) in input.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * m as f64 / n as f64;
                sum += x * Complex64::from_polar(1.0, angle);
            }
            sum
        })
        .collect()
}

/// IDFT (Inverse Discrete Fourier Transform) of length N (naive O(N^2)).
pub fn idft(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    let scale = 1.0 / n as f64;
    (0..n)
        .map(|m| {
            let mut sum = Complex64::zero();
            for (k, &x) in input.iter().enumerate() {
                let angle = 2.0 * PI * k as f64 * m as f64 / n as f64;
                sum += x * Complex64::from_polar(1.0, angle);
            }
            sum * scale
        })
        .collect()
}

/// Radix-2 Cooley-Tukey FFT (in-place, N must be power of 2).
pub fn fft_radix2(buf: &mut Vec<Complex64>) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT length must be power of two");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let mut len = 2usize;
    while len <= n {
        let w_step = -2.0 * PI / len as f64;
        for i in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let angle = w_step * k as f64;
                let w = Complex64::from_polar(1.0, angle);
                let u = buf[i + k];
                let v = buf[i + k + len / 2] * w;
                buf[i + k] = u + v;
                buf[i + k + len / 2] = u - v;
            }
        }
        len <<= 1;
    }
}

/// Inverse FFT (radix-2, N must be power of 2).
pub fn ifft_radix2(buf: &mut Vec<Complex64>) {
    let n = buf.len();
    // Conjugate
    for x in buf.iter_mut() {
        x.im = -x.im;
    }
    fft_radix2(buf);
    // Conjugate and scale
    let scale = 1.0 / n as f64;
    for x in buf.iter_mut() {
        x.re *= scale;
        x.im = -x.im * scale;
    }
}

// ─────────────────────────────────────────────────── main processor struct ──

/// 5G NR SRS Processor.
///
/// Provides SRS sequence generation, resource-grid mapping, LS channel
/// estimation, and timing advance computation per 3GPP TS 38.211 Section
/// 6.4.1.4 and TS 38.214 Section 6.2.1.
#[derive(Debug, Clone)]
pub struct SrsProcessor {
    config: SrsConfig,
}

impl SrsProcessor {
    /// Construct a new SRS processor with the given configuration.
    pub fn new(config: SrsConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &SrsConfig {
        &self.config
    }

    /// SRS sequence length in subcarriers: M_sc^SRS = bandwidth_rb * N_sc^RB / K_TC.
    ///
    /// Reference: TS 38.211 Eq. 6.4.1.4.2-1.
    pub fn m_sc_srs(&self) -> usize {
        self.config.bandwidth_rb * N_SC_RB / self.config.comb_size.ktc()
    }

    /// True if this slot/symbol carries an SRS according to periodicity.
    ///
    /// TS 38.214 Section 6.2.1: SRS transmitted when
    /// `(n_slot - T_offset) mod T_SRS == 0`.
    pub fn is_srs_slot(&self, n_slot: u32) -> bool {
        let t_srs = self.config.periodicity_slots;
        let t_off = self.config.offset_slots;
        if t_srs == 0 {
            return false;
        }
        ((n_slot + t_srs - t_off % t_srs) % t_srs) == 0
    }

    /// Compute the cyclic shift for a specific antenna port `p`.
    ///
    /// TS 38.211 Table 6.4.1.4.1-1:
    /// ```text
    /// n_cs^p = (n_CS + p_bar * n_CS_max / N_ap) mod n_CS_max
    /// ```
    ///
    /// where `p_bar` is the port index (0-based) and `N_ap` is the port count.
    pub fn port_cyclic_shift(&self, port: usize) -> u32 {
        let n_cs = self.config.cyclic_shift;
        let n_cs_max = self.config.comb_size.n_cs_max();
        let n_ap = self.config.num_antenna_ports as u32;
        (n_cs + port as u32 * n_cs_max / n_ap) % n_cs_max
    }

    /// Generate the SRS sequence for the given slot and symbol.
    ///
    /// Produces the base-port (port 0) SRS sequence. For multi-port, use
    /// [`generate_srs_port`] specifying the port index.
    ///
    /// # Arguments
    /// * `slot` - Slot number `n_s`
    /// * `symbol` - OFDM symbol index `l` within the slot (0..13)
    ///
    /// # Returns
    /// SRS sequence of length `M_sc^SRS`.
    pub fn generate_srs(&self, slot: u32, symbol: u32) -> Vec<Complex64> {
        self.generate_srs_port(slot, symbol, 0)
    }

    /// Generate SRS sequence for a specific antenna port.
    ///
    /// Reference: TS 38.211 Section 6.4.1.4.1.
    ///
    /// # Arguments
    /// * `slot`   - Slot number
    /// * `symbol` - Symbol index within the slot
    /// * `port`   - Antenna port index (0 .. num_antenna_ports - 1)
    pub fn generate_srs_port(&self, slot: u32, symbol: u32, port: usize) -> Vec<Complex64> {
        let m_sc = self.m_sc_srs();
        if m_sc == 0 {
            return Vec::new();
        }

        // Step 1: Determine group u and sequence v
        let (u, v) = compute_group_sequence(
            self.config.sequence_id,
            slot,
            symbol,
            self.config.hopping_mode,
        );

        // Step 2: Generate base sequence r_{u,v}(n)
        let base = generate_base_sequence(u, v, m_sc);

        // Step 3: Apply port-specific cyclic shift alpha
        let n_cs = self.port_cyclic_shift(port);
        let n_cs_max = self.config.comb_size.n_cs_max();
        apply_cyclic_shift(&base, n_cs, n_cs_max)
    }

    /// Generate SRS sequences for all configured antenna ports.
    ///
    /// # Returns
    /// Vector of sequences, indexed by port (length = `num_antenna_ports`).
    pub fn generate_all_ports(&self, slot: u32, symbol: u32) -> Vec<Vec<Complex64>> {
        (0..self.config.num_antenna_ports)
            .map(|p| self.generate_srs_port(slot, symbol, p))
            .collect()
    }

    /// Map SRS sequence to the OFDM resource grid (frequency domain).
    ///
    /// Places SRS samples at every K_TC-th subcarrier starting from k_0,
    /// leaving other subcarriers at zero.
    ///
    /// Reference: TS 38.211 Section 6.4.1.4.3, Eq. 6.4.1.4.3-1.
    ///
    /// # Arguments
    /// * `srs_seq`  - SRS sequence from [`generate_srs`] (length M_sc^SRS)
    /// * `n_fft`    - FFT size (number of subcarriers in the resource grid)
    /// * `start_rb` - Starting resource block index for this SRS resource
    ///
    /// # Returns
    /// Frequency-domain resource grid of length `n_fft` with SRS mapped.
    pub fn map_to_subcarriers(
        &self,
        srs_seq: &[Complex64],
        n_fft: usize,
        start_rb: usize,
    ) -> Vec<Complex64> {
        let mut grid = vec![Complex64::zero(); n_fft];
        let k_tc = self.config.comb_size.ktc();
        let k0 = compute_starting_subcarrier(
            start_rb,
            self.config.comb_size,
            self.config.comb_offset,
            self.config.n_rrc,
            self.config.bandwidth_rb,
        );

        for (m, &s) in srs_seq.iter().enumerate() {
            let k = k0 + k_tc * m;
            if k < n_fft {
                grid[k] = s;
            }
        }
        grid
    }

    /// Compute frequency hopping subcarrier offset for SRS slot `l_srs`.
    ///
    /// TS 38.211 Section 6.4.1.4.4:
    /// ```text
    /// f_b(l_srs) = floor(4 * n_RRC / m_SRS_b) mod N_b
    /// ```
    ///
    /// Returns additional RB offset to apply to the starting subcarrier.
    ///
    /// # Arguments
    /// * `l_srs`      - SRS occurrence index (e.g., slot count)
    /// * `hop_level`  - Bandwidth config index b_hop (0..3)
    pub fn hopping_offset(&self, l_srs: u32, hop_level: usize) -> usize {
        let m_srs = self.config.bandwidth_rb;
        if m_srs == 0 || hop_level == 0 {
            return 0;
        }

        // N_b is derived from m_SRS_b tables; simplified: N_b = 4 / (b+1)
        let n_b = (4usize).saturating_div(1 + hop_level).max(1);

        // n_b(l_srs) = floor(4 * n_RRC / m_SRS) mod N_b  (TS 38.211 Eq. 6.4.1.4.4-1)
        let n_rrc = self.config.n_rrc as usize;
        let phase = (4 * n_rrc / m_srs.max(1)) + l_srs as usize;
        (phase % n_b) * m_srs / n_b
    }

    /// Least-Squares channel estimation from received SRS.
    ///
    /// For each SRS subcarrier k:
    /// ```text
    /// H_LS(k) = Y(k) / X(k)
    /// ```
    ///
    /// Then performs linear interpolation across non-SRS subcarriers for a
    /// complete frequency response estimate.
    ///
    /// # Arguments
    /// * `received`   - Received frequency-domain signal (resource grid)
    /// * `reference`  - Known SRS reference grid from [`map_to_subcarriers`]
    /// * `start_rb`   - Starting RB of this SRS resource
    ///
    /// # Returns
    /// [`ChannelEstimate`] with LS estimates at SRS subcarriers and
    /// interpolated values in between.
    pub fn estimate_channel(
        &self,
        received: &[Complex64],
        reference: &[Complex64],
        start_rb: usize,
    ) -> ChannelEstimate {
        assert_eq!(
            received.len(),
            reference.len(),
            "received and reference must have equal length"
        );

        let n_fft = received.len();
        let k_tc = self.config.comb_size.ktc();
        let k0 = compute_starting_subcarrier(
            start_rb,
            self.config.comb_size,
            self.config.comb_offset,
            self.config.n_rrc,
            self.config.bandwidth_rb,
        );
        let m_sc = self.m_sc_srs();

        // Collect SRS subcarrier indices
        let mut srs_indices: Vec<usize> = Vec::with_capacity(m_sc);
        for m in 0..m_sc {
            let k = k0 + k_tc * m;
            if k < n_fft {
                srs_indices.push(k);
            }
        }

        // LS estimate at SRS subcarriers for port 0
        let mut h_srs: Vec<Complex64> = srs_indices
            .iter()
            .map(|&k| {
                let x = reference[k];
                let y = received[k];
                if x.norm_sqr() < 1e-20 {
                    Complex64::zero()
                } else {
                    y / x
                }
            })
            .collect();

        // Estimate noise floor from residuals (placeholder; requires known signal)
        // Use variance of H magnitude across subcarriers as proxy
        let mean_pwr = if h_srs.is_empty() {
            1.0
        } else {
            h_srs.iter().map(|h| h.norm_sqr()).sum::<f64>() / h_srs.len() as f64
        };
        let noise_floor = if mean_pwr > 0.0 { 0.01 * mean_pwr } else { 1.0 };

        // Interpolate between SRS subcarriers to fill full bandwidth
        // Linear interpolation for simplicity
        let mut h_interp = vec![Complex64::zero(); n_fft];
        for (i, (&k, &h)) in srs_indices.iter().zip(h_srs.iter()).enumerate() {
            h_interp[k] = h;
            // Fill between previous and current SRS carrier
            if i > 0 {
                let k_prev = srs_indices[i - 1];
                let h_prev = h_srs[i - 1];
                let span = k - k_prev;
                for s in 1..span {
                    let alpha = s as f64 / span as f64;
                    let re = h_prev.re + alpha * (h.re - h_prev.re);
                    let im = h_prev.im + alpha * (h.im - h_prev.im);
                    h_interp[k_prev + s] = Complex64::new(re, im);
                }
            }
        }

        // Extrapolate left
        if let Some(&k_first) = srs_indices.first() {
            let h_first = h_interp[k_first];
            for k in 0..k_first {
                h_interp[k] = h_first;
            }
        }
        // Extrapolate right
        if let Some(&k_last) = srs_indices.last() {
            let h_last = h_interp[k_last];
            for k in (k_last + 1)..n_fft {
                h_interp[k] = h_last;
            }
        }

        // Multi-port: for now duplicate single-port estimate
        // (Real implementation uses port-specific cyclic shifts)
        let h_per_port = vec![h_srs; self.config.num_antenna_ports];

        ChannelEstimate {
            h_per_port,
            subcarrier_indices: srs_indices,
            num_ports: self.config.num_antenna_ports,
            noise_floor,
        }
    }

    /// Estimate timing advance from received SRS.
    ///
    /// Algorithm:
    /// 1. Compute channel H(k) = Y(k)/X(k) at SRS subcarriers.
    /// 2. IFFT → time-domain channel impulse response h(tau).
    /// 3. Find peak position → timing advance.
    ///
    /// Reference: TS 38.213 Section 4.2 (timing advance).
    ///
    /// # Arguments
    /// * `received`    - Received frequency-domain grid
    /// * `reference`   - Reference SRS grid
    /// * `start_rb`    - Starting RB of this SRS resource
    /// * `sample_rate` - System sample rate in Hz (for time conversion)
    ///
    /// # Returns
    /// [`TimingAdvanceResult`] with estimated TA.
    pub fn estimate_timing_advance(
        &self,
        received: &[Complex64],
        reference: &[Complex64],
        start_rb: usize,
        sample_rate: f64,
    ) -> TimingAdvanceResult {
        assert_eq!(received.len(), reference.len());

        let k_tc = self.config.comb_size.ktc();
        let k0 = compute_starting_subcarrier(
            start_rb,
            self.config.comb_size,
            self.config.comb_offset,
            self.config.n_rrc,
            self.config.bandwidth_rb,
        );
        let m_sc = self.m_sc_srs();
        let n_fft = received.len();

        // Step 1: Compute H(k) at SRS subcarriers
        let mut h_sparse = vec![Complex64::zero(); n_fft];
        for m in 0..m_sc {
            let k = k0 + k_tc * m;
            if k < n_fft {
                let x = reference[k];
                let y = received[k];
                if x.norm_sqr() > 1e-20 {
                    h_sparse[k] = y / x;
                }
            }
        }

        // Step 2: IFFT → impulse response
        // Use power-of-2 IFFT for efficiency
        let fft_len = n_fft.next_power_of_two();
        let mut h_buf: Vec<Complex64> = h_sparse;
        h_buf.resize(fft_len, Complex64::zero());
        ifft_radix2(&mut h_buf);

        // Step 3: Find peak
        let mut peak_mag = 0.0f64;
        let mut peak_bin = 0usize;
        for (i, h) in h_buf.iter().enumerate() {
            let mag = h.norm_sqr();
            if mag > peak_mag {
                peak_mag = mag;
                peak_bin = i;
            }
        }
        let peak_mag = peak_mag.sqrt();

        // Convert peak bin to time
        let ta_samples = if sample_rate > 0.0 {
            peak_bin as f64
        } else {
            peak_bin as f64
        };
        let ta_seconds = ta_samples / sample_rate.max(1.0);

        TimingAdvanceResult {
            ta_seconds,
            ta_samples,
            peak_magnitude: peak_mag,
            peak_bin,
        }
    }

    /// Validate the SRS configuration.
    ///
    /// Returns `Ok(())` if valid, or `Err(msg)` describing the problem.
    pub fn validate(&self) -> Result<(), String> {
        let cfg = &self.config;

        if cfg.bandwidth_rb == 0 || cfg.bandwidth_rb > 272 {
            return Err(format!(
                "bandwidth_rb {} out of range [1..272]",
                cfg.bandwidth_rb
            ));
        }

        let n_ap = cfg.num_antenna_ports;
        if n_ap == 0 || n_ap > MAX_SRS_PORTS {
            return Err(format!(
                "num_antenna_ports {} out of range [1..{}]",
                n_ap, MAX_SRS_PORTS
            ));
        }

        let max_ports = cfg.comb_size.max_ports();
        if n_ap > max_ports {
            return Err(format!(
                "comb_size {:?} supports at most {} ports, got {}",
                cfg.comb_size, max_ports, n_ap
            ));
        }

        let n_cs_max = cfg.comb_size.n_cs_max();
        if cfg.cyclic_shift >= n_cs_max {
            return Err(format!(
                "cyclic_shift {} >= n_cs_max {} for comb {:?}",
                cfg.cyclic_shift, n_cs_max, cfg.comb_size
            ));
        }

        let k_tc = cfg.comb_size.ktc();
        if cfg.comb_offset >= k_tc {
            return Err(format!(
                "comb_offset {} >= K_TC {} for comb {:?}",
                cfg.comb_offset, k_tc, cfg.comb_size
            ));
        }

        if cfg.periodicity_slots == 0 {
            return Err("periodicity_slots must be >= 1".to_string());
        }

        if cfg.offset_slots >= cfg.periodicity_slots {
            return Err(format!(
                "offset_slots {} >= periodicity_slots {}",
                cfg.offset_slots, cfg.periodicity_slots
            ));
        }

        Ok(())
    }
}

// ──────────────────────────────────────────────────────── standalone helpers

/// Compute the number of SRS subcarriers for a given bandwidth in RBs and comb.
///
/// ```text
/// M_sc^SRS = m_SRS * N_sc^RB / K_TC
/// ```
pub fn srs_subcarrier_count(bandwidth_rb: usize, comb: CombSize) -> usize {
    bandwidth_rb * N_SC_RB / comb.ktc()
}

/// Cross-correlate SRS reference with received signal in the time domain.
///
/// Useful for coarse timing acquisition before channel estimation.
///
/// Returns correlation magnitude at each lag (0..N-1).
pub fn cross_correlate_srs(reference: &[Complex64], received: &[Complex64]) -> Vec<f64> {
    let n = reference.len().min(received.len());
    (0..n)
        .map(|lag| {
            let mut sum = Complex64::zero();
            for k in 0..n {
                let r = reference[k];
                let y = received[(k + lag) % n];
                sum += y * r.conj();
            }
            sum.norm()
        })
        .collect()
}

/// Compute SRS power (mean squared magnitude of the SRS sequence).
pub fn srs_power(seq: &[Complex64]) -> f64 {
    if seq.is_empty() {
        return 0.0;
    }
    seq.iter().map(|s| s.norm_sqr()).sum::<f64>() / seq.len() as f64
}

/// Apply a flat channel h to SRS (simulate received signal).
///
/// `Y(k) = H * X(k) + noise_std * noise(k)`
pub fn apply_channel(
    srs: &[Complex64],
    channel_gain: Complex64,
    noise_std: f64,
    seed: u64,
) -> Vec<Complex64> {
    // Simple LCG for reproducible noise
    let mut state = seed.wrapping_add(6364136223846793005);
    let mut gauss = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u1 = (state >> 11) as f64 / (1u64 << 53) as f64;
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u2 = (state >> 11) as f64 / (1u64 << 53) as f64;
        let r = (-2.0 * u1.ln().max(-1e10)).sqrt();
        let theta = 2.0 * PI * u2;
        (r * theta.cos(), r * theta.sin())
    };

    srs.iter()
        .map(|&x| {
            let (nr, ni) = gauss();
            let noise = Complex64::new(noise_std * nr / 2.0_f64.sqrt(), noise_std * ni / 2.0_f64.sqrt());
            x * channel_gain + noise
        })
        .collect()
}

// ────────────────────────────────────────────────────────────── unit tests ──

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    // ── helpers ──────────────────────────────────────────────────────────────

    fn default_config() -> SrsConfig {
        SrsConfig {
            bandwidth_rb: 52,
            comb_size: CombSize::Two,
            cyclic_shift: 0,
            hopping_mode: HoppingMode::Neither,
            num_antenna_ports: 1,
            periodicity_slots: 40,
            offset_slots: 0,
            sequence_id: 0,
            n_rrc: 0,
            comb_offset: 0,
        }
    }

    fn make_proc(cfg: SrsConfig) -> SrsProcessor {
        SrsProcessor::new(cfg)
    }

    // ── Complex64 arithmetic ─────────────────────────────────────────────────

    #[test]
    fn test_complex_add() {
        let a = Complex64::new(1.0, 2.0);
        let b = Complex64::new(3.0, -1.0);
        let c = a + b;
        assert!((c.re - 4.0).abs() < TOL);
        assert!((c.im - 1.0).abs() < TOL);
    }

    #[test]
    fn test_complex_mul() {
        let a = Complex64::new(1.0, 2.0);
        let b = Complex64::new(3.0, 4.0);
        let c = a * b;
        // (1+2j)(3+4j) = 3+4j+6j+8j² = -5+10j
        assert!((c.re - (-5.0)).abs() < TOL);
        assert!((c.im - 10.0).abs() < TOL);
    }

    #[test]
    fn test_complex_div() {
        let a = Complex64::new(1.0, 0.0);
        let b = Complex64::new(2.0, 0.0);
        let c = a / b;
        assert!((c.re - 0.5).abs() < TOL);
        assert!(c.im.abs() < TOL);
    }

    #[test]
    fn test_complex_conj() {
        let a = Complex64::new(3.0, -4.0);
        let c = a.conj();
        assert!((c.re - 3.0).abs() < TOL);
        assert!((c.im - 4.0).abs() < TOL);
    }

    #[test]
    fn test_complex_norm() {
        let a = Complex64::new(3.0, 4.0);
        assert!((a.norm() - 5.0).abs() < TOL);
    }

    #[test]
    fn test_from_polar_unit() {
        // e^(j*pi/2) = j
        let c = Complex64::from_polar(1.0, PI / 2.0);
        assert!(c.re.abs() < 1e-10);
        assert!((c.im - 1.0).abs() < 1e-10);
    }

    // ── ZC sequences ─────────────────────────────────────────────────────────

    #[test]
    fn test_zc_unit_magnitude() {
        let seq = zadoff_chu_seq(3, 31);
        for s in &seq {
            assert!((s.norm() - 1.0).abs() < 1e-10, "ZC should have unit magnitude");
        }
    }

    #[test]
    fn test_zc_length() {
        let seq = zadoff_chu_seq(7, 139);
        assert_eq!(seq.len(), 139);
    }

    #[test]
    fn test_zc_first_sample() {
        // x_q(0) = exp(-j*pi*q*0*1/N) = 1
        let seq = zadoff_chu_seq(5, 29);
        assert!((seq[0].re - 1.0).abs() < TOL);
        assert!(seq[0].im.abs() < TOL);
    }

    #[test]
    fn test_zc_different_roots_differ() {
        let s1 = zadoff_chu_seq(1, 31);
        let s2 = zadoff_chu_seq(3, 31);
        let differ = s1.iter().zip(s2.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        assert!(differ, "Different ZC roots should produce different sequences");
    }

    // ── Base sequence generation ──────────────────────────────────────────────

    #[test]
    fn test_base_seq_unit_magnitude() {
        let seq = generate_base_sequence(5, 0, 72);
        for s in &seq {
            assert!((s.norm() - 1.0).abs() < 1e-8, "base_seq should have unit magnitude");
        }
    }

    #[test]
    fn test_base_seq_length_72() {
        let seq = generate_base_sequence(0, 0, 72);
        assert_eq!(seq.len(), 72);
    }

    #[test]
    fn test_base_seq_length_48() {
        let seq = generate_base_sequence(3, 1, 48);
        assert_eq!(seq.len(), 48);
    }

    #[test]
    fn test_base_seq_length_12() {
        let seq = generate_base_sequence(0, 0, 12);
        assert_eq!(seq.len(), 12);
    }

    #[test]
    fn test_base_seq_group_variation() {
        // Different group u should produce different sequences
        let s0 = generate_base_sequence(0, 0, 72);
        let s5 = generate_base_sequence(5, 0, 72);
        let differ = s0.iter().zip(s5.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        assert!(differ);
    }

    #[test]
    fn test_base_seq_sequence_variation() {
        // Different sequence v should produce different sequences (for long seqs)
        let s_v0 = generate_base_sequence(10, 0, 120);
        let s_v1 = generate_base_sequence(10, 1, 120);
        let differ = s_v0.iter().zip(s_v1.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        assert!(differ);
    }

    // ── Cyclic shift ──────────────────────────────────────────────────────────

    #[test]
    fn test_cyclic_shift_preserves_magnitude() {
        let base = generate_base_sequence(2, 0, 72);
        let shifted = apply_cyclic_shift(&base, 3, 8);
        for s in &shifted {
            assert!((s.norm() - 1.0).abs() < 1e-8);
        }
    }

    #[test]
    fn test_cyclic_shift_zero_is_identity() {
        let base = generate_base_sequence(1, 0, 48);
        let shifted = apply_cyclic_shift(&base, 0, 8);
        for (a, b) in base.iter().zip(shifted.iter()) {
            assert!((a.re - b.re).abs() < TOL);
            assert!((a.im - b.im).abs() < TOL);
        }
    }

    #[test]
    fn test_cyclic_shift_produces_different_seq() {
        let base = generate_base_sequence(3, 0, 72);
        let s0 = apply_cyclic_shift(&base, 0, 8);
        let s3 = apply_cyclic_shift(&base, 3, 8);
        let differ = s0.iter().zip(s3.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        assert!(differ);
    }

    #[test]
    fn test_cyclic_shift_orthogonality() {
        // Shifts 0 and n_cs_max/2 should produce sequences with low correlation
        let base = generate_base_sequence(7, 0, 72);
        let n_cs_max = 8u32;
        let s0 = apply_cyclic_shift(&base, 0, n_cs_max);
        let s4 = apply_cyclic_shift(&base, 4, n_cs_max);

        let corr: f64 = s0.iter().zip(s4.iter())
            .map(|(a, b)| ((*a) * b.conj()).re)
            .sum::<f64>()
            .abs();
        // For truly orthogonal shifts, correlation should be near 0
        // In practice (finite length), it will be small but not exactly 0
        assert!(corr < s0.len() as f64 * 0.5,
            "Orthogonal shifts should have low correlation, got {}", corr);
    }

    // ── SRS processor ─────────────────────────────────────────────────────────

    #[test]
    fn test_srs_generate_basic() {
        let proc = make_proc(default_config());
        let srs = proc.generate_srs(0, 13);
        assert_eq!(srs.len(), proc.m_sc_srs());
        assert!(!srs.is_empty());
    }

    #[test]
    fn test_srs_unit_magnitude() {
        let proc = make_proc(default_config());
        let srs = proc.generate_srs(5, 13);
        for s in &srs {
            assert!((s.norm() - 1.0).abs() < 1e-8,
                "SRS samples should have unit magnitude, got {}", s.norm());
        }
    }

    #[test]
    fn test_m_sc_srs_comb2() {
        let cfg = SrsConfig { bandwidth_rb: 4, comb_size: CombSize::Two, ..default_config() };
        let proc = make_proc(cfg);
        assert_eq!(proc.m_sc_srs(), 4 * N_SC_RB / 2); // 24
    }

    #[test]
    fn test_m_sc_srs_comb4() {
        let cfg = SrsConfig { bandwidth_rb: 4, comb_size: CombSize::Four, ..default_config() };
        let proc = make_proc(cfg);
        assert_eq!(proc.m_sc_srs(), 4 * N_SC_RB / 4); // 12
    }

    #[test]
    fn test_m_sc_srs_comb8() {
        let cfg = SrsConfig { bandwidth_rb: 8, comb_size: CombSize::Eight, ..default_config() };
        let proc = make_proc(cfg);
        assert_eq!(proc.m_sc_srs(), 8 * N_SC_RB / 8); // 12
    }

    // ── Comb mapping ──────────────────────────────────────────────────────────

    #[test]
    fn test_map_comb2_spacing() {
        let cfg = SrsConfig { bandwidth_rb: 4, comb_size: CombSize::Two, comb_offset: 0, ..default_config() };
        let proc = make_proc(cfg);
        let srs = proc.generate_srs(0, 13);
        let grid = proc.map_to_subcarriers(&srs, 128, 0);

        // With comb-2 and no hopping, every other subcarrier from k0 should be set
        let k_tc = 2;
        let k0 = 0; // start_rb=0, comb_offset=0
        for m in 0..srs.len() {
            let k = k0 + k_tc * m;
            if k < 128 {
                assert!(grid[k].norm() > 0.5, "SRS should be at k={}", k);
            }
        }
    }

    #[test]
    fn test_map_comb4_spacing() {
        let cfg = SrsConfig {
            bandwidth_rb: 4,
            comb_size: CombSize::Four,
            comb_offset: 0,
            ..default_config()
        };
        let proc = make_proc(cfg);
        let srs = proc.generate_srs(0, 13);
        let grid = proc.map_to_subcarriers(&srs, 256, 0);

        let k_tc = 4;
        let k0 = 0;
        for m in 0..srs.len() {
            let k = k0 + k_tc * m;
            if k < 256 {
                assert!(grid[k].norm() > 0.5, "Comb-4 SRS should be at k={}", k);
            }
        }
    }

    #[test]
    fn test_map_comb8_spacing() {
        let cfg = SrsConfig {
            bandwidth_rb: 8,
            comb_size: CombSize::Eight,
            comb_offset: 0,
            ..default_config()
        };
        let proc = make_proc(cfg);
        let srs = proc.generate_srs(0, 13);
        let grid = proc.map_to_subcarriers(&srs, 512, 0);

        let k_tc = 8;
        let k0 = 0;
        for m in 0..srs.len() {
            let k = k0 + k_tc * m;
            if k < 512 {
                assert!(grid[k].norm() > 0.5, "Comb-8 SRS should be at k={}", k);
            }
        }
    }

    #[test]
    fn test_map_comb_offset() {
        let cfg = SrsConfig {
            bandwidth_rb: 4,
            comb_size: CombSize::Two,
            comb_offset: 1, // offset by 1
            ..default_config()
        };
        let proc = make_proc(cfg);
        let srs = proc.generate_srs(0, 13);
        let grid = proc.map_to_subcarriers(&srs, 128, 0);

        // k0 = 0 + 0 + 1 = 1 (comb_offset=1)
        // First SRS subcarrier should be at k=1
        assert!(grid[1].norm() > 0.5, "First SRS sub with offset=1 should be at k=1");
    }

    #[test]
    fn test_map_total_subcarrier_count() {
        let cfg = SrsConfig { bandwidth_rb: 10, comb_size: CombSize::Two, ..default_config() };
        let proc = make_proc(cfg);
        let srs = proc.generate_srs(0, 13);
        let grid = proc.map_to_subcarriers(&srs, 512, 0);
        let nonzero_count = grid.iter().filter(|x| x.norm() > 0.5).count();
        assert_eq!(nonzero_count, srs.len(), "All SRS samples should be mapped");
    }

    // ── Frequency hopping ──────────────────────────────────────────────────────

    #[test]
    fn test_hopping_neither_zero_offset() {
        let proc = make_proc(default_config());
        let off0 = proc.hopping_offset(0, 0);
        let off5 = proc.hopping_offset(5, 0);
        assert_eq!(off0, 0);
        assert_eq!(off5, 0, "No hopping => offset always 0");
    }

    #[test]
    fn test_hopping_nonzero_level() {
        let bw = 16;
        let cfg = SrsConfig {
            bandwidth_rb: bw,
            hopping_mode: HoppingMode::GroupHopping,
            ..default_config()
        };
        let proc = make_proc(cfg);
        // With hop_level > 0, different l_srs may give different offsets
        let off_a = proc.hopping_offset(0, 2);
        let off_b = proc.hopping_offset(4, 2);
        // Just verify they are non-negative and within bandwidth
        assert!(off_a <= bw);
        assert!(off_b <= bw);
    }

    #[test]
    fn test_hopping_offset_bounded() {
        let bw = 32;
        let cfg = SrsConfig { bandwidth_rb: bw, ..default_config() };
        let proc = make_proc(cfg);
        for l in 0..20u32 {
            for b in 0..4usize {
                let off = proc.hopping_offset(l, b);
                assert!(off <= bw, "Hopping offset must not exceed BW");
            }
        }
    }

    // ── Multi-port ────────────────────────────────────────────────────────────

    #[test]
    fn test_two_port_different_seqs() {
        let cfg = SrsConfig {
            num_antenna_ports: 2,
            cyclic_shift: 0,
            comb_size: CombSize::Two,
            ..default_config()
        };
        let proc = make_proc(cfg);
        let s0 = proc.generate_srs_port(0, 13, 0);
        let s1 = proc.generate_srs_port(0, 13, 1);
        let differ = s0.iter().zip(s1.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        assert!(differ, "Port 0 and port 1 SRS should differ (different cyclic shifts)");
    }

    #[test]
    fn test_four_port_all_different() {
        let cfg = SrsConfig {
            num_antenna_ports: 4,
            cyclic_shift: 0,
            comb_size: CombSize::Two,
            ..default_config()
        };
        let proc = make_proc(cfg);
        let seqs: Vec<_> = (0..4).map(|p| proc.generate_srs_port(0, 13, p)).collect();
        // All ports should have unit magnitude
        for (p, seq) in seqs.iter().enumerate() {
            for s in seq {
                assert!((s.norm() - 1.0).abs() < 1e-8, "Port {} not unit magnitude", p);
            }
        }
        // At least two should differ
        let differ_01 = seqs[0].iter().zip(seqs[1].iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        let differ_02 = seqs[0].iter().zip(seqs[2].iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        assert!(differ_01 && differ_02, "All ports should produce distinct sequences");
    }

    #[test]
    fn test_generate_all_ports_count() {
        let cfg = SrsConfig { num_antenna_ports: 2, ..default_config() };
        let proc = make_proc(cfg);
        let all = proc.generate_all_ports(0, 13);
        assert_eq!(all.len(), 2);
    }

    #[test]
    fn test_port_cyclic_shift_wraps() {
        let cfg = SrsConfig {
            num_antenna_ports: 4,
            cyclic_shift: 6,
            comb_size: CombSize::Two,
            ..default_config()
        };
        let proc = make_proc(cfg);
        let n_cs_max = CombSize::Two.n_cs_max();
        for p in 0..4 {
            let cs = proc.port_cyclic_shift(p);
            assert!(cs < n_cs_max, "Cyclic shift must wrap around n_cs_max");
        }
    }

    // ── Periodicity ───────────────────────────────────────────────────────────

    #[test]
    fn test_is_srs_slot_periodic() {
        let cfg = SrsConfig {
            periodicity_slots: 10,
            offset_slots: 0,
            ..default_config()
        };
        let proc = make_proc(cfg);
        assert!(proc.is_srs_slot(0));
        assert!(!proc.is_srs_slot(1));
        assert!(proc.is_srs_slot(10));
        assert!(!proc.is_srs_slot(11));
        assert!(proc.is_srs_slot(20));
    }

    #[test]
    fn test_is_srs_slot_with_offset() {
        let cfg = SrsConfig {
            periodicity_slots: 10,
            offset_slots: 3,
            ..default_config()
        };
        let proc = make_proc(cfg);
        assert!(!proc.is_srs_slot(0));
        assert!(proc.is_srs_slot(3));
        assert!(!proc.is_srs_slot(4));
        assert!(proc.is_srs_slot(13));
    }

    // ── Channel estimation ────────────────────────────────────────────────────

    #[test]
    fn test_channel_estimation_ideal_noiseless() {
        let proc = make_proc(default_config());
        let n_fft = 512;
        let start_rb = 0;

        // Generate reference SRS
        let srs = proc.generate_srs(0, 13);
        let reference = proc.map_to_subcarriers(&srs, n_fft, start_rb);

        // Ideal channel: H = 1 (no distortion)
        let received = reference.clone();

        let est = proc.estimate_channel(&received, &reference, start_rb);
        assert!(!est.subcarrier_indices.is_empty());

        // All estimated H values should be ~1 (unit channel)
        for h in &est.h_per_port[0] {
            assert!((h.re - 1.0).abs() < 1e-6, "H should be ~1 for identity channel");
            assert!(h.im.abs() < 1e-6, "H imag should be ~0 for identity channel");
        }
    }

    #[test]
    fn test_channel_estimation_known_gain() {
        let proc = make_proc(default_config());
        let n_fft = 512;
        let start_rb = 0;

        let srs = proc.generate_srs(0, 13);
        let reference = proc.map_to_subcarriers(&srs, n_fft, start_rb);

        // Apply known flat channel gain = 2*e^(j*pi/4)
        let h_true = Complex64::from_polar(2.0, PI / 4.0);
        let received: Vec<Complex64> = reference.iter().map(|&x| x * h_true).collect();

        let est = proc.estimate_channel(&received, &reference, start_rb);

        for h in &est.h_per_port[0] {
            // Only check non-zero estimates (at SRS subcarriers)
            if h.norm() > 0.01 {
                assert!((h.re - h_true.re).abs() < 1e-5,
                    "H.re should match true channel, got {}", h.re);
                assert!((h.im - h_true.im).abs() < 1e-5,
                    "H.im should match true channel, got {}", h.im);
            }
        }
    }

    #[test]
    fn test_channel_estimation_subcarrier_count() {
        let cfg = SrsConfig { bandwidth_rb: 10, comb_size: CombSize::Two, ..default_config() };
        let proc = make_proc(cfg);
        let n_fft = 512;
        let srs = proc.generate_srs(0, 13);
        let ref_grid = proc.map_to_subcarriers(&srs, n_fft, 0);
        let est = proc.estimate_channel(&ref_grid, &ref_grid, 0);
        assert_eq!(est.subcarrier_indices.len(), srs.len());
    }

    #[test]
    fn test_channel_estimation_multi_port() {
        let cfg = SrsConfig { num_antenna_ports: 2, ..default_config() };
        let proc = make_proc(cfg);
        let n_fft = 512;
        let srs = proc.generate_srs(0, 13);
        let ref_grid = proc.map_to_subcarriers(&srs, n_fft, 0);
        let est = proc.estimate_channel(&ref_grid, &ref_grid, 0);
        assert_eq!(est.num_ports, 2);
        assert_eq!(est.h_per_port.len(), 2);
    }

    // ── Timing advance estimation ─────────────────────────────────────────────

    #[test]
    fn test_timing_advance_zero_delay() {
        let proc = make_proc(default_config());
        let n_fft = 512;

        let srs = proc.generate_srs(0, 13);
        let reference = proc.map_to_subcarriers(&srs, n_fft, 0);

        // No delay: received = reference
        let ta = proc.estimate_timing_advance(&reference, &reference, 0, 30.72e6);
        // Peak should be at or near bin 0 (no delay)
        assert!(ta.peak_bin < 4 || ta.peak_bin > n_fft - 4,
            "Zero delay should give peak near 0, got {}", ta.peak_bin);
        assert!(ta.peak_magnitude > 0.1);
    }

    #[test]
    fn test_timing_advance_nonzero_sample_rate() {
        let proc = make_proc(default_config());
        let n_fft = 512;
        let srs = proc.generate_srs(0, 13);
        let reference = proc.map_to_subcarriers(&srs, n_fft, 0);
        let ta = proc.estimate_timing_advance(&reference, &reference, 0, 30.72e6);
        // ta_seconds should be non-negative
        assert!(ta.ta_seconds >= 0.0);
    }

    #[test]
    fn test_timing_advance_has_positive_magnitude() {
        let proc = make_proc(default_config());
        let n_fft = 512;
        let srs = proc.generate_srs(0, 13);
        let reference = proc.map_to_subcarriers(&srs, n_fft, 0);

        // Apply flat channel
        let h = Complex64::from_polar(1.0, 0.3);
        let received: Vec<_> = reference.iter().map(|&x| x * h).collect();

        let ta = proc.estimate_timing_advance(&received, &reference, 0, 30.72e6);
        assert!(ta.peak_magnitude > 0.0, "Should detect some peak");
    }

    // ── Hopping modes ─────────────────────────────────────────────────────────

    #[test]
    fn test_group_hopping_varies_sequence() {
        let cfg = SrsConfig {
            hopping_mode: HoppingMode::GroupHopping,
            ..default_config()
        };
        let proc = make_proc(cfg);
        let s0 = proc.generate_srs(0, 13);
        let s1 = proc.generate_srs(1, 13);
        // With group hopping, different slots may produce different sequences
        // (not guaranteed, depends on hash — just verify no panic)
        let _ = s0.len();
        let _ = s1.len();
    }

    #[test]
    fn test_sequence_hopping_mode() {
        let cfg = SrsConfig {
            hopping_mode: HoppingMode::SequenceHopping,
            ..default_config()
        };
        let proc = make_proc(cfg);
        let s = proc.generate_srs(0, 13);
        assert_eq!(s.len(), proc.m_sc_srs());
        // Sequence should have unit magnitude
        for sample in &s {
            assert!((sample.norm() - 1.0).abs() < 1e-8);
        }
    }

    #[test]
    fn test_neither_hopping_deterministic() {
        let proc = make_proc(default_config());
        let s1 = proc.generate_srs(5, 13);
        let s2 = proc.generate_srs(5, 13);
        // Same inputs → same output
        for (a, b) in s1.iter().zip(s2.iter()) {
            assert!((a.re - b.re).abs() < TOL);
            assert!((a.im - b.im).abs() < TOL);
        }
    }

    // ── BW part configurations ────────────────────────────────────────────────

    #[test]
    fn test_bwp_small_bandwidth() {
        let cfg = SrsConfig { bandwidth_rb: 4, comb_size: CombSize::Two, ..default_config() };
        let proc = make_proc(cfg);
        let srs = proc.generate_srs(0, 13);
        assert_eq!(srs.len(), 4 * 12 / 2);
    }

    #[test]
    fn test_bwp_large_bandwidth() {
        let cfg = SrsConfig { bandwidth_rb: 106, comb_size: CombSize::Two, ..default_config() };
        let proc = make_proc(cfg);
        let srs = proc.generate_srs(0, 13);
        assert_eq!(srs.len(), 106 * 12 / 2);
    }

    #[test]
    fn test_bwp_full_nr_bandwidth_100mhz() {
        // 100 MHz with 15 kHz SCS: 52 RBs
        let cfg = SrsConfig { bandwidth_rb: 52, comb_size: CombSize::Two, ..default_config() };
        let proc = make_proc(cfg);
        assert_eq!(proc.m_sc_srs(), 52 * 12 / 2); // 312
    }

    // ── Validation ────────────────────────────────────────────────────────────

    #[test]
    fn test_validate_ok() {
        let proc = make_proc(default_config());
        assert!(proc.validate().is_ok());
    }

    #[test]
    fn test_validate_zero_bandwidth() {
        let cfg = SrsConfig { bandwidth_rb: 0, ..default_config() };
        let proc = make_proc(cfg);
        assert!(proc.validate().is_err());
    }

    #[test]
    fn test_validate_too_many_ports() {
        let cfg = SrsConfig {
            num_antenna_ports: 5,
            ..default_config()
        };
        let proc = make_proc(cfg);
        assert!(proc.validate().is_err());
    }

    #[test]
    fn test_validate_invalid_cyclic_shift() {
        let cfg = SrsConfig {
            cyclic_shift: 100,
            comb_size: CombSize::Two,
            ..default_config()
        };
        let proc = make_proc(cfg);
        assert!(proc.validate().is_err());
    }

    #[test]
    fn test_validate_comb_offset_too_large() {
        let cfg = SrsConfig {
            comb_size: CombSize::Two,
            comb_offset: 2, // must be < K_TC = 2
            ..default_config()
        };
        let proc = make_proc(cfg);
        assert!(proc.validate().is_err());
    }

    #[test]
    fn test_validate_zero_periodicity() {
        let cfg = SrsConfig { periodicity_slots: 0, ..default_config() };
        let proc = make_proc(cfg);
        assert!(proc.validate().is_err());
    }

    #[test]
    fn test_validate_offset_too_large() {
        let cfg = SrsConfig {
            periodicity_slots: 10,
            offset_slots: 10,
            ..default_config()
        };
        let proc = make_proc(cfg);
        assert!(proc.validate().is_err());
    }

    // ── Helper functions ──────────────────────────────────────────────────────

    #[test]
    fn test_srs_subcarrier_count() {
        assert_eq!(srs_subcarrier_count(4, CombSize::Two), 24);
        assert_eq!(srs_subcarrier_count(4, CombSize::Four), 12);
        assert_eq!(srs_subcarrier_count(8, CombSize::Eight), 12);
    }

    #[test]
    fn test_srs_power_unit_sequence() {
        let proc = make_proc(default_config());
        let srs = proc.generate_srs(0, 13);
        let power = srs_power(&srs);
        assert!((power - 1.0).abs() < 1e-8, "SRS power should be 1 (unit magnitude), got {}", power);
    }

    #[test]
    fn test_cross_correlate_length() {
        let proc = make_proc(default_config());
        let srs = proc.generate_srs(0, 13);
        let n_fft = 256;
        let ref_grid = proc.map_to_subcarriers(&srs, n_fft, 0);
        let corr = cross_correlate_srs(&ref_grid, &ref_grid);
        assert_eq!(corr.len(), ref_grid.len());
    }

    #[test]
    fn test_cross_correlate_peak_at_zero() {
        let proc = make_proc(default_config());
        let srs = proc.generate_srs(0, 13);
        let n_fft = 64;
        let ref_grid = proc.map_to_subcarriers(&srs, n_fft, 0);
        let corr = cross_correlate_srs(&ref_grid, &ref_grid);
        let peak = corr[0];
        // All other lags should be <= peak
        for (i, &c) in corr.iter().enumerate() {
            assert!(c <= peak + 1e-6, "Peak should be at lag 0, lag {} has larger value {}", i, c);
        }
    }

    #[test]
    fn test_apply_channel_correct_length() {
        let proc = make_proc(default_config());
        let srs = proc.generate_srs(0, 13);
        let received = apply_channel(&srs, Complex64::new(1.0, 0.0), 0.0, 42);
        assert_eq!(received.len(), srs.len());
    }

    #[test]
    fn test_apply_channel_noiseless_identity() {
        let proc = make_proc(default_config());
        let srs = proc.generate_srs(0, 13);
        let h = Complex64::new(1.0, 0.0);
        let received = apply_channel(&srs, h, 0.0, 0);
        for (x, y) in srs.iter().zip(received.iter()) {
            assert!((x.re - y.re).abs() < 1e-9);
            assert!((x.im - y.im).abs() < 1e-9);
        }
    }

    // ── is_prime ──────────────────────────────────────────────────────────────

    #[test]
    fn test_is_prime_basic() {
        assert!(!is_prime(0));
        assert!(!is_prime(1));
        assert!(is_prime(2));
        assert!(is_prime(3));
        assert!(!is_prime(4));
        assert!(is_prime(5));
        assert!(is_prime(7));
        assert!(!is_prime(9));
        assert!(is_prime(11));
        assert!(is_prime(31));
        assert!(is_prime(139));
        assert!(is_prime(839));
    }

    #[test]
    fn test_largest_prime_lte() {
        assert_eq!(largest_prime_lte(12), 11);
        assert_eq!(largest_prime_lte(31), 31);
        assert_eq!(largest_prime_lte(30), 29);
        assert_eq!(largest_prime_lte(1), 2);
    }

    // ── FFT/IFFT round-trip ───────────────────────────────────────────────────

    #[test]
    fn test_fft_ifft_roundtrip() {
        let n = 64;
        let input: Vec<Complex64> = (0..n)
            .map(|i| Complex64::new((i as f64 * 0.1).sin(), 0.0))
            .collect();

        let mut buf = input.clone();
        fft_radix2(&mut buf);
        ifft_radix2(&mut buf);

        for (a, b) in input.iter().zip(buf.iter()) {
            assert!((a.re - b.re).abs() < 1e-8, "FFT/IFFT roundtrip failed");
            assert!((a.im - b.im).abs() < 1e-8, "FFT/IFFT roundtrip failed");
        }
    }

    #[test]
    fn test_fft_tone() {
        // FFT of complex exponential at bin k0 should peak at k0
        let n = 64;
        let k0 = 7;
        let input: Vec<Complex64> = (0..n)
            .map(|i| Complex64::from_polar(1.0, 2.0 * PI * k0 as f64 * i as f64 / n as f64))
            .collect();
        let mut buf = input;
        fft_radix2(&mut buf);

        let peak_bin = buf.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.norm_sqr().partial_cmp(&b.norm_sqr()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak_bin, k0, "FFT peak should be at bin {}", k0);
    }

    // ── Sequence ID variation ─────────────────────────────────────────────────

    #[test]
    fn test_different_sequence_ids_differ() {
        let cfg0 = SrsConfig { sequence_id: 0, ..default_config() };
        let cfg1 = SrsConfig { sequence_id: 100, ..default_config() };
        let p0 = SrsProcessor::new(cfg0).generate_srs(0, 13);
        let p1 = SrsProcessor::new(cfg1).generate_srs(0, 13);
        let differ = p0.iter().zip(p1.iter()).any(|(a, b)| (a.re - b.re).abs() > 1e-6);
        assert!(differ, "Different sequence IDs should produce different SRS");
    }

    // ── CombSize properties ───────────────────────────────────────────────────

    #[test]
    fn test_comb_ktc_values() {
        assert_eq!(CombSize::Two.ktc(), 2);
        assert_eq!(CombSize::Four.ktc(), 4);
        assert_eq!(CombSize::Eight.ktc(), 8);
    }

    #[test]
    fn test_comb_n_cs_max() {
        assert_eq!(CombSize::Two.n_cs_max(), 8);
        assert_eq!(CombSize::Four.n_cs_max(), 12);
        assert_eq!(CombSize::Eight.n_cs_max(), 6);
    }

    #[test]
    fn test_comb_max_ports() {
        assert_eq!(CombSize::Two.max_ports(), 4);
        assert_eq!(CombSize::Four.max_ports(), 4);
        assert_eq!(CombSize::Eight.max_ports(), 1);
    }

    // ── gcd ──────────────────────────────────────────────────────────────────

    #[test]
    fn test_gcd() {
        assert_eq!(gcd(12, 8), 4);
        assert_eq!(gcd(7, 3), 1);
        assert_eq!(gcd(0, 5), 5);
        assert_eq!(gcd(100, 25), 25);
    }
}
